#include "motion_ctrl.h"
#include "bsp_motor.h"
#include "config.h"
#include "board.h"

#include <math.h>

/* 循迹 PID 状态 */
static float s_line_integ = 0.0f;
static float s_line_prev_err = 0.0f;
static bool  s_line_lost = false;
static float s_lost_start_mm = 0.0f;    /* 本次丢线开始时的总里程 */
static float s_lost_mm = 0.0f;          /* 本次连续丢线已行驶的距离 */

/* 定距直行 / 原地转向目标状态 */
static float s_tgt_dist_mm = 0.0f;
static float s_tgt_drive_spd = 0.0f;
static float s_tgt_turn_deg = 0.0f;
static float s_tgt_turn_spd = 0.0f;

/* 动作超时 */
static uint32_t s_act_t0 = 0U;
static uint32_t s_act_limit_ms = 0U;
static bool     s_act_timeout = false;

static void action_begin(float amount, float speed)
{
    float spd = (speed > 1.0f) ? speed : 1.0f;
    float ideal_ms = (amount / spd) * 1000.0f;
    s_act_t0 = HAL_GetTick();
    s_act_limit_ms = (uint32_t)(ideal_ms * CFG_MOTION_TIMEOUT_SCALE) + CFG_MOTION_TIMEOUT_EXTRA_MS;
    s_act_timeout = false;
}

/* 超时则停车并返回 true */
static bool action_expired(void)
{
    if ((uint32_t)(HAL_GetTick() - s_act_t0) > s_act_limit_ms) {
        s_act_timeout = true;
        BspMotor_Stop();
        return true;
    }
    return false;
}

bool Motion_ActionTimedOut(void)
{
    return s_act_timeout;
}

void Motion_ClearActionTimeout(void)
{
    s_act_timeout = false;
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

void Motion_Init(void)
{
    Motion_ResetLinePid();
}

void Motion_ResetLinePid(void)
{
    s_line_integ = 0.0f;
    s_line_prev_err = 0.0f;
    s_line_lost = false;
    s_lost_mm = 0.0f;
}

float Motion_GetLineLostMm(void)
{
    return s_lost_mm;
}

void Motion_LineFollowStep(float base_speed_mm_s, gray_bias_mode_t bias_mode, float dt_s)
{
    if (dt_s <= 0.001f) {
        return;
    }

    BspGray_Sample(bias_mode);
    const gray_state_t *g = BspGray_GetState();

    /* err < 0: 黑线在左，需左转 (右轮快、左轮慢)
     * err > 0: 黑线在右，需右转 (左轮快、右轮慢) */
    float err = g->error;
    s_line_integ = clampf(s_line_integ + err * dt_s, -2.0f, 2.0f);
    float deriv = (err - s_line_prev_err) / dt_s;
    s_line_prev_err = err;

    float diff = CFG_LINE_PID_KP * err +
                 CFG_LINE_PID_KI * s_line_integ +
                 CFG_LINE_PID_KD * deriv * 0.02f;
    diff = clampf(diff, -CFG_LINE_DIFF_MAX_MM_S, CFG_LINE_DIFF_MAX_MM_S);

    /* 大偏差（如 30cm 圆角急弯）适度降基速，防止甩出黑线 */
    float abs_err = fabsf(err);
    float spd = base_speed_mm_s;
    if (abs_err > 2.2f) {
        spd *= 0.72f;
    } else if (abs_err > 1.2f) {
        spd *= 0.86f;
    }

    /* 丢线：按上次方向低速找线，并累计丢线行驶距离供状态机判 LINE_LOST */
    const motor_state_t *m = BspMotor_GetState();
    if (!g->line_found) {
        if (!s_line_lost) {
            s_line_lost = true;
            s_lost_start_mm = m->total_dist_mm;
        }
        float d = m->total_dist_mm - s_lost_start_mm;
        s_lost_mm = (d >= 0.0f) ? d : -d;
        spd = base_speed_mm_s * CFG_LINE_LOST_SPEED_SCALE;
    } else {
        s_line_lost = false;
        s_lost_mm = 0.0f;
    }

    float left_spd  = spd + diff;
    float right_spd = spd - diff;
    BspMotor_SetDiffSpeed(left_spd, right_spd);
}

void Motion_StartDriveDist(float dist_mm, float speed_mm_s)
{
    BspMotor_ResetSegOdom();
    s_tgt_dist_mm = dist_mm;
    s_tgt_drive_spd = fabsf(speed_mm_s);
    action_begin(fabsf(dist_mm), s_tgt_drive_spd);
}

bool Motion_UpdateDriveDist(void)
{
    const motor_state_t *m = BspMotor_GetState();
    float remain = s_tgt_dist_mm - m->seg_dist_mm;
    float abs_rem = fabsf(remain);

    if (s_act_timeout || action_expired()) {
        return false;
    }

    if (abs_rem <= CFG_DIST_TOLERANCE_MM) {
        BspMotor_Stop();
        return true;
    }

    /* 末段 80mm 平滑减速，避免冲过目标点 */
    float spd = s_tgt_drive_spd;
    if (abs_rem < 80.0f) {
        float scale = 0.45f + 0.55f * (abs_rem / 80.0f);
        spd *= scale;
        if (spd < 75.0f) { spd = 75.0f; }
    }
    if (remain < 0.0f) {
        spd = -spd;
    }

    /* 航向角保持 P 纠偏：yaw > 0 表示偏左，需向右纠偏（左快右慢） */
    float corr = clampf(m->seg_yaw_deg * CFG_DRIVE_HEADING_KP, -80.0f, 80.0f);
    BspMotor_SetDiffSpeed(spd + corr, spd - corr);
    return false;
}

void Motion_StartTurnDeg(float angle_deg, float turn_speed_mm_s)
{
    BspMotor_ResetSegOdom();
    s_tgt_turn_deg = angle_deg;
    s_tgt_turn_spd = fabsf(turn_speed_mm_s);
    /* 原地转：轮子走的弧长 = 角度 x 等效轮距 / 2；按最低转速估算时间，偏保守 */
    float arc_mm = fabsf(angle_deg) * 0.0174533f * CFG_TRACK_WIDTH_MM * 0.5f;
    action_begin(arc_mm, CFG_TURN_MIN_SPEED_MM_S);
}

bool Motion_UpdateTurnDeg(void)
{
    const motor_state_t *m = BspMotor_GetState();
    float err_deg = s_tgt_turn_deg - m->seg_yaw_deg;

    if (s_act_timeout || action_expired()) {
        return false;
    }

    if (fabsf(err_deg) <= CFG_TURN_TOLERANCE_DEG) {
        BspMotor_Stop();
        return true;
    }

    float mag = fabsf(err_deg) * CFG_TURN_KP;
    mag = clampf(mag, CFG_TURN_MIN_SPEED_MM_S, s_tgt_turn_spd);

    if (err_deg > 0.0f) {
        /* 左转：左轮后退、右轮前进 */
        BspMotor_SetDiffSpeed(-mag, mag);
    } else {
        /* 右转：左轮前进、右轮后退 */
        BspMotor_SetDiffSpeed(mag, -mag);
    }
    return false;
}

void Motion_Stop(void)
{
    BspMotor_Stop();
}

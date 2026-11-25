#include "motion_ctrl.h"
#include "bsp_motor.h"
#include "config.h"
#include "board.h"

#include <math.h>

/* 循迹 PID 状态 */
static float s_line_integ = 0.0f;
static float s_line_prev_err = 0.0f;
static float s_line_d_filt = 0.0f;      /* 低通后的微分项（单位：每 20 ms 的误差变化） */
static bool  s_line_have_prev = false;  /* 复位后第一拍没有上一拍误差，不算微分，免得起步突跳 */
static bool  s_line_prev_found = true;  /* 上一拍灰度是否有线，用来发现“找回线”那一拍 */
static bool  s_line_prev_cross = false; /* 上一拍是否在路口保持中，用来发现保持结束那一拍 */
static bool  s_line_lost = false;
static float s_lost_start_mm = 0.0f;    /* 本次丢线开始时的总里程 */
static float s_lost_mm = 0.0f;          /* 本次连续丢线已行驶的距离 */
static float s_lost_start_yaw = 0.0f;   /* 本次丢线开始时的累计航向 */
static float s_lost_deg = 0.0f;         /* 本次连续丢线已转过的角度 */
static bool  s_in_cross = false;        /* 灰度正处在路口保持中 */
static float s_cross_start_mm = 0.0f;   /* 本次路口保持开始时的总里程 */

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
    s_line_d_filt = 0.0f;
    s_line_have_prev = false;
    s_line_prev_found = true;
    s_line_prev_cross = false;
    s_line_lost = false;
    s_lost_mm = 0.0f;
    s_lost_deg = 0.0f;
    s_in_cross = false;
}

float Motion_GetLineLostMm(void)
{
    return s_lost_mm;
}

float Motion_GetLineLostDeg(void)
{
    return s_lost_deg;
}

float Motion_LinePTerm(float err)
{
    return CFG_LINE_PID_KP * err * (CFG_LINE_KP_CENTER_SCALE + CFG_LINE_KP_EDGE_GAIN * fabsf(err));
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
    /* 8 路灰度的误差是一格 0.5 地跳，直接差分会把每次跳格放大成一个尖峰（直道高频抖），
     * 所以微分先做一阶低通。单位折算成“每 20 ms 的误差变化”，KD 的含义与控制周期无关 */
    /* 误差来源切换的那一拍不算微分：丢线后找回（找线误差 ±3.8 → 实测）、路口保持结束（保持值 → 实测）
     * 时误差是换了来源而不是车在快速偏离，差分会冒出一个假尖峰 */
    const bool src_switch = (g->line_found && !s_line_prev_found) ||
                            (s_line_prev_cross && (g->cross_frames == 0U));
    s_line_prev_found = g->line_found;
    s_line_prev_cross = (g->cross_frames > 0U);
    float d_raw = (s_line_have_prev && !src_switch) ? ((err - s_line_prev_err) / dt_s * 0.02f) : 0.0f;
    s_line_have_prev = true;
    s_line_prev_err = err;
    s_line_d_filt += CFG_LINE_D_LPF_ALPHA * (d_raw - s_line_d_filt);

    /* P 项中间软、两边硬：直道小偏差不来回摆，弯里大偏差给足差速（见 config.h） */
    float diff = Motion_LinePTerm(err) +
                 CFG_LINE_PID_KI * s_line_integ +
                 CFG_LINE_PID_KD * s_line_d_filt;
    diff = clampf(diff, -CFG_LINE_DIFF_MAX_MM_S, CFG_LINE_DIFF_MAX_MM_S);

    /* 偏差越大越减速（主文档 6.2：base' = base × (1 − k·|err|)）。连续降速，不用台阶：
     * 弯里误差在台阶阈值两侧跳时，基速会跟着一拍一拍地跳，车身就抖 */
    float abs_err = fabsf(err);
    float spd = base_speed_mm_s *
                clampf(1.0f - CFG_LINE_SLOW_PER_ERR * abs_err, CFG_LINE_SLOW_MIN_SCALE, 1.0f);

    /* 丢线：按上次方向低速找线，并累计丢线行驶距离供状态机判 LINE_LOST */
    const motor_state_t *m = BspMotor_GetState();
    /* 路口保持封顶：压黑够多时灰度保持上一拍误差直行，真横线只压几十毫米。连续“路口”超过
     * CFG_LINE_CROSS_MAX_MM 说明不是路口（阈值设错整排判黑、开进大片黑区），按丢线处理，
     * 好让 LINE_LOST 能停车；否则 line_found 一直为真，车会一直直行不停 */
    if (g->cross_frames > 0U) {
        if (!s_in_cross) {
            s_in_cross = true;
            s_cross_start_mm = m->total_dist_mm;
        }
    } else {
        s_in_cross = false;
    }
    const bool flooded = s_in_cross && (fabsf(m->total_dist_mm - s_cross_start_mm) > CFG_LINE_CROSS_MAX_MM);
    if (!g->line_found || flooded) {
        if (!s_line_lost) {
            s_line_lost = true;
            s_lost_start_mm = m->total_dist_mm;
            s_lost_start_yaw = m->total_yaw_deg;
        }
        float d = m->total_dist_mm - s_lost_start_mm;
        s_lost_mm = (d >= 0.0f) ? d : -d;
        /* 找线接近原地转，前进很慢，只按距离判要转好几圈才停，所以另按转角判 */
        s_lost_deg = fabsf(m->total_yaw_deg - s_lost_start_yaw);
        spd = base_speed_mm_s * CFG_LINE_LOST_SPEED_SCALE;
        /* 丢线时误差固定为 ±3.8，乘 KP 往往直接顶满差速限幅，车近乎原地猛转，
         * 再见到线时转速太快就冲过头（直角弯来回摆）。找线差速单独限幅 */
        diff = clampf(diff, -CFG_LINE_LOST_DIFF_MM_S, CFG_LINE_LOST_DIFF_MM_S);
    } else {
        s_line_lost = false;
        s_lost_mm = 0.0f;
        s_lost_deg = 0.0f;
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

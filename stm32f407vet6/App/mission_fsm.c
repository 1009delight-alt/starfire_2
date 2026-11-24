#include "mission_fsm.h"
#include "motion_ctrl.h"
#include "board.h"
#include "config.h"
#include "bsp_motor.h"
#include "bsp_gray.h"
#include "bsp_sonar.h"
#include "bsp_openmv.h"

#include <math.h>
#include <stdio.h>

/* 实现《星火杯赛题二比赛程序开发交接文档》第 6、7 章。 */

/* 避障子步骤 */
typedef enum {
    AVOID_STEP_TURN_OUT = 0,
    AVOID_STEP_MOVE_OUT,
    AVOID_STEP_TURN_PARALLEL,
    AVOID_STEP_MOVE_PARALLEL,
    AVOID_STEP_TURN_IN,
    AVOID_STEP_REJOIN_SEARCH
} avoid_step_t;

/* 倒车入库子步骤 */
typedef enum {
    PARK_STEP_EXTRA_FWD = 0,
    PARK_STEP_TURN_90,
    PARK_STEP_BACK_IN,
    PARK_STEP_HOLD_5S
} park_step_t;

static mission_profile_t s_profile = MISSION_PROFILE_FULL;
static mission_state_t   s_state   = ST_IDLE;
static avoid_step_t      s_avoid_step = AVOID_STEP_TURN_OUT;
static park_step_t       s_park_step  = PARK_STEP_EXTRA_FWD;
static mission_start_t   s_start_stage = START_FULL;

static float    s_stage_start_dist_mm = 0.0f;
static uint32_t s_step_enter_ms = 0U;
static uint32_t s_state_enter_ms = 0U;
static const char *s_fault_reason = "NONE";
static const char *s_block_reason = NULL;   /* IDLE 下不允许起跑的原因 */
static char     s_name_buf[16];

/* 按键：进入 IDLE/FAULT 后必须先松开，之后新按下的键才有效 */
static bool     s_key_armed = false;
static uint32_t s_key_arm_ms = 0U;

/* 分岔 */
static char     s_locked_fork_dir = 0;
static bool     s_fork_guess = false;
static bool     s_fork_waiting = false;
static bool     s_fork_wait_done = false;
static uint32_t s_fork_wait_t0 = 0U;
static uint32_t s_fork_last_query_ms = 0U;
static bool     s_fork_view_switched = false;

/* 回线确认 */
static uint8_t  s_rejoin_hits = 0U;

/* 四色与找库 */
static float    s_color_enter_mm = 0.0f;
static bool     s_color_done = false;
static float    s_color_done_mm = 0.0f;
static bool     s_corner4_valid = false;
static float    s_corner4_mm = 0.0f;
static bool     s_park_by_odom = false;

/* 弯道识别：每走 CORNER_SAMPLE_MM 记一个 (里程, 航向) 样本 */
#define CORNER_SAMPLE_MM    20.0f
#define CORNER_RING         32U
static float    s_cr_dist[CORNER_RING];
static float    s_cr_yaw[CORNER_RING];
static uint8_t  s_cr_head = 0U;
static uint8_t  s_cr_count = 0U;

static bool key_event_valid_after_arm(uint32_t now_ms, board_key_event_t *ev_out)
{
    board_key_event_t ev = Board_KeyTakeEvent();
    if (!s_key_armed) {
        if (!Board_KeyIsPressed()) {
            s_key_armed = true;
            s_key_arm_ms = now_ms;
        }
        return false;
    }
    if (ev == KEY_EVT_NONE) {
        return false;
    }
    /* 只接受在“松开确认”之后才开始的按压 */
    if ((int32_t)(Board_KeyLastPressMs() - s_key_arm_ms) <= 0) {
        return false;
    }
    *ev_out = ev;
    return true;
}

/* OpenMV 固定在车右前方（无云台），任务切换立即下发；与当前任务相同时不重发 */
static void view_set(openmv_task_t task)
{
    if (BspOpenmv_GetState()->target_task != task) {
        BspOpenmv_SetTask(task);
    }
}

static void corner_reset(void)
{
    s_cr_head = 0U;
    s_cr_count = 0U;
}

/* 返回 true 表示刚经过一个弯 */
static bool corner_update(void)
{
    const motor_state_t *m = BspMotor_GetState();
    float d = m->total_dist_mm;
    float y = m->total_yaw_deg;

    if (s_cr_count > 0U) {
        uint8_t last = (uint8_t)((s_cr_head + CORNER_RING - 1U) % CORNER_RING);
        if ((d - s_cr_dist[last]) < CORNER_SAMPLE_MM) {
            return false;
        }
    }
    s_cr_dist[s_cr_head] = d;
    s_cr_yaw[s_cr_head] = y;
    s_cr_head = (uint8_t)((s_cr_head + 1U) % CORNER_RING);
    if (s_cr_count < CORNER_RING) {
        s_cr_count++;
    }

    /* 找窗口内最早的样本 */
    for (uint8_t k = s_cr_count; k > 1U; --k) {
        uint8_t idx = (uint8_t)((s_cr_head + CORNER_RING - k) % CORNER_RING);
        if ((d - s_cr_dist[idx]) <= CFG_CORNER_WINDOW_MM) {
            if (fabsf(y - s_cr_yaw[idx]) >= CFG_CORNER_YAW_DEG) {
                corner_reset();     /* 同一个弯不重复计 */
                return true;
            }
            break;
        }
    }
    return false;
}

static void enter_state(mission_state_t next, uint32_t now_ms)
{
    const motor_state_t *m = BspMotor_GetState();
    s_state = next;
    s_stage_start_dist_mm = m->total_dist_mm;
    s_step_enter_ms = now_ms;
    s_state_enter_ms = now_ms;
    Motion_ResetLinePid();

    switch (next) {
    case ST_IDLE:
        BspMotor_Stop();
        Motion_ClearActionTimeout();
        view_set(OPENMV_TASK_IDLE);
        Board_LedSetPattern(LED_PAT_SLOW_BLINK);
        Board_KeyResetStartArm();
        s_key_armed = false;
        break;

    case ST_TRACK_TO_OBS:
        BspSonar_ResetTrigger();
        view_set(OPENMV_TASK_IDLE);
        Board_LedSetPattern(LED_PAT_FAST_BLINK);
        break;

    case ST_AVOID_OBS:
        s_avoid_step = AVOID_STEP_TURN_OUT;
        s_rejoin_hits = 0U;
        Motion_StartTurnDeg((float)CFG_AVOID_TURN_DIR * CFG_AVOID_OUT_ANGLE_DEG,
                            CFG_SPEED_TURN_INPLACE_MM_S);
        Board_LedSetPattern(LED_PAT_FAST_BLINK);
        break;

    case ST_FORK_PASS:
        s_locked_fork_dir = 0;
        BspOpenmv_ClearTurn();      /* 丢掉上一次跑分岔时留在驱动层的方向 */
        s_fork_guess = false;
        s_fork_waiting = false;
        s_fork_wait_done = false;
        s_fork_view_switched = false;
        view_set(OPENMV_TASK_TURN);
        Board_LedSetPattern(LED_PAT_FAST_BLINK);
        break;

    case ST_COLOR_SCAN:
        s_color_enter_mm = m->total_dist_mm;
        s_color_done = false;
        s_corner4_valid = false;
        corner_reset();
        view_set(OPENMV_TASK_COLOR);
        Board_LedSetPattern(LED_PAT_FAST_BLINK);
        break;

    case ST_PARK_SEARCH:
        s_park_by_odom = false;
        view_set(OPENMV_TASK_PARK);
        Board_LedSetPattern(LED_PAT_FAST_BLINK);
        break;

    case ST_PARK_REVERSE:
        view_set(OPENMV_TASK_IDLE);
        s_park_step = PARK_STEP_EXTRA_FWD;
        Motion_StartDriveDist(CFG_PARK_EXTRA_FWD_MM, CFG_SPEED_PARK_SEARCH_MM_S);
        Board_LedSetPattern(LED_PAT_FAST_BLINK);
        break;

    case ST_DONE:
        BspMotor_Stop();
        view_set(OPENMV_TASK_IDLE);
        Board_LedSetPattern(LED_PAT_DOUBLE_BLINK);
        break;

    case ST_FAULT:
    default:
        BspMotor_EmergencyStop();
        BspOpenmv_SetTask(OPENMV_TASK_IDLE);
        Board_LedSetPattern(LED_PAT_FAULT_BLINK);
        s_key_armed = false;
        break;
    }
}

void Mission_Init(mission_profile_t profile)
{
    s_profile = profile;
    s_fault_reason = "NONE";
    s_locked_fork_dir = 0;
    s_start_stage = (profile == MISSION_PROFILE_PARK_ONLY) ? START_PARK : START_FULL;
    BspMotor_ClearFault();
    enter_state(ST_IDLE, HAL_GetTick());
}

void Mission_RequestEmergencyStop(const char *reason)
{
    s_fault_reason = (reason != NULL) ? reason : "ESTOP";
    enter_state(ST_FAULT, HAL_GetTick());
}

static const char *start_name(mission_start_t st)
{
    switch (st) {
    case START_AVOID: return "AVOID";
    case START_FORK:  return "FORK";
    case START_COLOR: return "COLOR";
    case START_PARK:  return "PARK";
    case START_FULL:
    default:          return "FULL";
    }
}

/* 起跑前置条件；返回 NULL 表示可以起跑 */
static const char *check_start_ready(uint32_t now_ms)
{
    const motor_state_t  *mot = BspMotor_GetState();
    const openmv_state_t *mv  = BspOpenmv_GetState();
    (void)mv;
    (void)now_ms;

#if (CFG_REQUIRE_ENC_ONLINE_TO_START != 0U)
    if (!mot->enc_online) {
        return "WAIT:ENC";
    }
#endif
#if (CFG_REQUIRE_SONAR_TO_START != 0U) && (CFG_ENABLE_SONAR != 0U)
    if (((s_start_stage == START_FULL) || (s_start_stage == START_AVOID)) &&
        (s_profile != MISSION_PROFILE_PARK_ONLY) && !BspSonar_IsHealthy(now_ms)) {
        return "WAIT:SONAR";
    }
#endif
#if (CFG_REQUIRE_MV_ONLINE_TO_START != 0U) && (CFG_ENABLE_OPENMV != 0U)
    if ((s_profile != MISSION_PROFILE_AVOID_ONLY) && !mv->online) {
        return "WAIT:MV";
    }
#endif
    return NULL;
}

static void do_start(uint32_t now_ms)
{
    BspMotor_ClearFault();
    BspMotor_ResetTotalOdom();

    if (s_profile == MISSION_PROFILE_AVOID_ONLY) {
        enter_state(ST_TRACK_TO_OBS, now_ms);
        return;
    }

    switch (s_start_stage) {
    case START_AVOID:
        /* 保留已锁存的方向和四色；预置里程使避障门控立即生效 */
        BspMotor_SetTotalDist(CFG_STAGE_AVOID_START_MM);
        enter_state(ST_TRACK_TO_OBS, now_ms);
        break;
    case START_FORK:
        BspMotor_SetTotalDist(CFG_STAGE_FORK_START_MM);
        enter_state(ST_FORK_PASS, now_ms);
        break;
    case START_COLOR:
        /* 重做四色：清空主控与相机的颜色结果 */
        BspMotor_SetTotalDist(CFG_STAGE_COLOR_START_MM);
        BspOpenmv_ResetAll();
        enter_state(ST_COLOR_SCAN, now_ms);
        break;
    case START_PARK:
        /* 车放在第 4 弯之后的顶直道上：以起跑点作为找库里程基准，保留四色 */
        BspMotor_SetTotalDist(CFG_STAGE_PARK_START_MM);
        s_color_enter_mm = CFG_STAGE_PARK_START_MM;
        s_corner4_valid = true;
        s_corner4_mm = CFG_STAGE_PARK_START_MM;
        corner_reset();
        enter_state(ST_PARK_SEARCH, now_ms);
        break;
    case START_FULL:
    default:
        BspOpenmv_ResetAll();
        enter_state(ST_TRACK_TO_OBS, now_ms);
        break;
    }
}

static void step_avoid_obs(uint32_t now_ms)
{
    const float dir = (float)CFG_AVOID_TURN_DIR;
    const motor_state_t *m = BspMotor_GetState();

    switch (s_avoid_step) {
    case AVOID_STEP_TURN_OUT:
        if (Motion_UpdateTurnDeg()) {
            s_avoid_step = AVOID_STEP_MOVE_OUT;
            Motion_StartDriveDist(CFG_AVOID_OUT_DIST_MM, CFG_SPEED_AVOID_MM_S);
        }
        break;

    case AVOID_STEP_MOVE_OUT:
        if (Motion_UpdateDriveDist()) {
            s_avoid_step = AVOID_STEP_TURN_PARALLEL;
            Motion_StartTurnDeg(-dir * CFG_AVOID_OUT_ANGLE_DEG, CFG_SPEED_TURN_INPLACE_MM_S);
        }
        break;

    case AVOID_STEP_TURN_PARALLEL:
        if (Motion_UpdateTurnDeg()) {
            s_avoid_step = AVOID_STEP_MOVE_PARALLEL;
            Motion_StartDriveDist(CFG_AVOID_PARALLEL_DIST_MM, CFG_SPEED_AVOID_MM_S);
        }
        break;

    case AVOID_STEP_MOVE_PARALLEL:
        if (Motion_UpdateDriveDist()) {
            s_avoid_step = AVOID_STEP_TURN_IN;
            Motion_StartTurnDeg(-dir * CFG_AVOID_IN_ANGLE_DEG, CFG_SPEED_TURN_INPLACE_MM_S);
        }
        break;

    case AVOID_STEP_TURN_IN:
        if (Motion_UpdateTurnDeg()) {
            s_avoid_step = AVOID_STEP_REJOIN_SEARCH;
            s_rejoin_hits = 0U;
            Motion_StartDriveDist(CFG_AVOID_REJOIN_MAX_MM, CFG_SPEED_AVOID_MM_S * 0.85f);
        }
        break;

    case AVOID_STEP_REJOIN_SEARCH: {
        BspGray_Sample(GRAY_BIAS_CENTER);
        const gray_state_t *g = BspGray_GetState();
        /* 连续多帧看到线且偏差不大，才算回到原线（文档 6.4） */
        if ((m->seg_dist_mm > 60.0f) && g->line_found && (fabsf(g->error) <= CFG_REJOIN_MAX_ERR)) {
            if (s_rejoin_hits < 255U) { s_rejoin_hits++; }
        } else {
            s_rejoin_hits = 0U;
        }
        if (s_rejoin_hits >= CFG_REJOIN_CONFIRM) {
            BspMotor_Stop();
            if (s_profile == MISSION_PROFILE_AVOID_ONLY) {
                enter_state(ST_DONE, now_ms);
            } else {
                enter_state(ST_FORK_PASS, now_ms);
            }
        } else if (Motion_UpdateDriveDist()) {
            Mission_RequestEmergencyStop("REJOIN_FAIL");
        }
        break;
    }
    }
}

static void step_park_reverse(uint32_t now_ms)
{
    switch (s_park_step) {
    case PARK_STEP_EXTRA_FWD:
        if (Motion_UpdateDriveDist()) {
            s_park_step = PARK_STEP_TURN_90;
            Motion_StartTurnDeg(CFG_PARK_TURN_ANGLE_DEG, CFG_SPEED_TURN_INPLACE_MM_S);
        }
        break;

    case PARK_STEP_TURN_90:
        if (Motion_UpdateTurnDeg()) {
            s_park_step = PARK_STEP_BACK_IN;
            Motion_StartDriveDist(-CFG_PARK_BACK_DIST_MM, CFG_SPEED_REVERSE_PARK_MM_S);
        }
        break;

    case PARK_STEP_BACK_IN:
        if (Motion_UpdateDriveDist()) {
            s_park_step = PARK_STEP_HOLD_5S;
            s_step_enter_ms = now_ms;
            BspMotor_Stop();
        }
        break;

    case PARK_STEP_HOLD_5S:
        if ((uint32_t)(now_ms - s_step_enter_ms) >= CFG_PARK_HOLD_MS) {
            enter_state(ST_DONE, now_ms);
        }
        break;
    }
}

static bool check_line_lost(void)
{
    if ((Motion_GetLineLostMm() > CFG_LINE_LOST_MAX_MM) || (Motion_GetLineLostDeg() > CFG_LINE_LOST_MAX_DEG)) {
        Mission_RequestEmergencyStop("LINE_LOST");
        return true;
    }
    return false;
}

static void step_fork(uint32_t now_ms, float stage_dist, float dt_s)
{
    const openmv_state_t *mv = BspOpenmv_GetState();

    /* 只采信本轮已下发 TURN 且相机已确认 TURN 之后的方向 */
    if ((s_locked_fork_dir == 0) && (mv->turn_dir != 0) &&
        (mv->target_task == OPENMV_TASK_TURN) && (mv->ack_task == OPENMV_TASK_TURN)) {
        s_locked_fork_dir = mv->turn_dir;
    }

    /* 到偏置起点仍无方向：停车等待并查询，超时才用兜底方向（文档 6.3） */
    if ((stage_dist >= CFG_GATE_FORK_BIAS_START_MM) && (s_locked_fork_dir == 0) && !s_fork_wait_done) {
        if (!s_fork_waiting) {
            s_fork_waiting = true;
            s_fork_wait_t0 = now_ms;
            s_fork_last_query_ms = now_ms;
            BspMotor_Stop();
            BspOpenmv_Query();
        } else if ((uint32_t)(now_ms - s_fork_wait_t0) >= CFG_FORK_WAIT_MS) {
            s_locked_fork_dir = CFG_FORK_FALLBACK_DIR;
            s_fork_guess = true;
            s_fork_wait_done = true;
            s_fork_waiting = false;
        } else if ((uint32_t)(now_ms - s_fork_last_query_ms) >= CFG_OPENMV_CMD_RETRY_MS) {
            s_fork_last_query_ms = now_ms;
            BspOpenmv_Query();
        }
        if (s_fork_waiting) {
            return;
        }
    }
    if (s_fork_waiting && (s_locked_fork_dir != 0)) {
        s_fork_waiting = false;
        s_fork_wait_done = true;
    }

    gray_bias_mode_t bias = GRAY_BIAS_CENTER;
    if ((stage_dist >= CFG_GATE_FORK_BIAS_START_MM) && (stage_dist < CFG_GATE_FORK_DONE_MM)) {
        bias = (s_locked_fork_dir == 'L') ? GRAY_BIAS_LEFT : GRAY_BIAS_RIGHT;
    }
    Motion_LineFollowStep(CFG_SPEED_FORK_MM_S, bias, dt_s);
    if (check_line_lost()) {
        return;
    }

    if (stage_dist >= (CFG_GATE_FORK_DONE_MM + CFG_GATE_COLOR_ENTER_MM)) {
        enter_state(ST_COLOR_SCAN, now_ms);
    } else if ((stage_dist >= CFG_GATE_FORK_DONE_MM) && !s_fork_view_switched) {
        /* 出分岔后停掉转向识别，免得后面的牌子被误认 */
        s_fork_view_switched = true;
        view_set(OPENMV_TASK_IDLE);
    }
}

void Mission_Step20ms(uint32_t now_ms, float dt_s)
{
    const motor_state_t  *mot = BspMotor_GetState();
    const openmv_state_t *mv  = BspOpenmv_GetState();
    float stage_dist = mot->total_dist_mm - s_stage_start_dist_mm;
    board_key_event_t ev = KEY_EVT_NONE;

    /* 1. 运动态：驱动板失联、计数跳变、人工按键急停 */
    if ((s_state != ST_IDLE) && (s_state != ST_DONE) && (s_state != ST_FAULT)) {
        if (mot->rx_timeout_fault) {
            Mission_RequestEmergencyStop("ENC_TMO");
            return;
        }
        if (mot->enc_jump_fault) {
            Mission_RequestEmergencyStop("ENC_JUMP");
            return;
        }
        if (mot->enc_stall_fault) {
            Mission_RequestEmergencyStop("ENC_STALL");
            return;
        }
        if (Motion_ActionTimedOut()) {
            Mission_RequestEmergencyStop("MOTION_TMO");
            return;
        }
        if (Board_KeyCheckStartEdge(now_ms)) {
            Mission_RequestEmergencyStop("KEY_STOP");
            return;
        }
    }

    switch (s_state) {
    case ST_IDLE:
        s_block_reason = check_start_ready(now_ms);
        if (key_event_valid_after_arm(now_ms, &ev)) {
            if ((ev == KEY_EVT_LONG) && (s_profile == MISSION_PROFILE_FULL)) {
                s_start_stage = (mission_start_t)(((uint8_t)s_start_stage + 1U) % (uint8_t)START_COUNT);
            } else if ((ev == KEY_EVT_SHORT) && (s_block_reason == NULL)) {
                do_start(now_ms);
            }
        }
        break;

    case ST_TRACK_TO_OBS: {
        Motion_LineFollowStep(CFG_SPEED_CRUISE_MM_S, GRAY_BIAS_CENTER, dt_s);
        if (check_line_lost()) {
            break;
        }
        bool gate_ok = (s_profile == MISSION_PROFILE_AVOID_ONLY)
                       ? (stage_dist >= 150.0f)
                       : (mot->total_dist_mm >= CFG_GATE_OBS_ENABLE_MM);
#if (CFG_ENABLE_SONAR != 0U)
        /* 避障窗口内超声失效不能当作前方畅通（文档 2 章） */
        if (gate_ok && !BspSonar_IsHealthy(now_ms)) {
            Mission_RequestEmergencyStop("SONAR_LOST");
            break;
        }
#endif
        if (gate_ok && BspSonar_IsObstacleDetected(now_ms)) {
            BspMotor_Stop();
            enter_state(ST_AVOID_OBS, now_ms);
        }
        break;
    }

    case ST_AVOID_OBS:
        step_avoid_obs(now_ms);
        break;

    case ST_FORK_PASS:
        step_fork(now_ms, stage_dist, dt_s);
        break;

    case ST_COLOR_SCAN:
        Motion_LineFollowStep(CFG_SPEED_COLOR_MM_S, GRAY_BIAS_CENTER, dt_s);
        if (check_line_lost()) {
            break;
        }
        if (corner_update() &&
            ((mot->total_dist_mm - s_color_enter_mm) >= CFG_CORNER4_MIN_AFTER_COLOR_MM)) {
            s_corner4_valid = true;
            s_corner4_mm = mot->total_dist_mm;
        }
        if (!s_color_done) {
            if (mv->color_locked) {
                /* 4 色全部锁存，立即切 IDLE 防后续误改 */
                s_color_done = true;
                s_color_done_mm = mot->total_dist_mm;
                view_set(OPENMV_TASK_IDLE);
            } else if (stage_dist >= CFG_GATE_COLOR_EXIT_MM) {
                /* 走出色区仍未满 4 色：保留已识别结果，继续去拿入库分 */
                s_color_done = true;
                s_color_done_mm = mot->total_dist_mm;
                view_set(OPENMV_TASK_IDLE);
            }
        } else if ((mot->total_dist_mm - s_color_done_mm) >= CFG_GATE_PARK_ENTER_MM) {
            /* 从锁存（或走出色区）的位置起算，而不是从进入色区起算 */
            enter_state(ST_PARK_SEARCH, now_ms);
        }
        break;

    case ST_PARK_SEARCH: {
        Motion_LineFollowStep(CFG_SPEED_PARK_SEARCH_MM_S, GRAY_BIAS_CENTER, dt_s);
        if (check_line_lost()) {
            break;
        }
        if (corner_update() &&
            ((mot->total_dist_mm - s_color_enter_mm) >= CFG_CORNER4_MIN_AFTER_COLOR_MM)) {
            s_corner4_valid = true;
            s_corner4_mm = mot->total_dist_mm;
        }

        /* 最新一帧必须是 flag=1 且足够新（驱动层收到 flag=0 会立即清 park_seen） */
        bool p_aligned = (mv->target_task == OPENMV_TASK_PARK) && mv->park_seen &&
                         ((uint32_t)(now_ms - mv->park_last_ms) <= CFG_PARK_FRESH_MS) &&
                         (mv->park_area >= CFG_PARK_MIN_AREA_PX) &&
                         (mv->park_cx >= CFG_PARK_ALIGN_CX_MIN) &&
                         (mv->park_cx <= CFG_PARK_ALIGN_CX_MAX);
        bool odom_fallback = s_corner4_valid
            ? ((mot->total_dist_mm - s_corner4_mm) >= CFG_PARK_FALLBACK_MM)
            : ((mot->total_dist_mm - s_color_enter_mm) >= CFG_PARK_FALLBACK_ABS_MM);

        if (p_aligned || odom_fallback) {
            s_park_by_odom = !p_aligned;
            BspMotor_Stop();
            enter_state(ST_PARK_REVERSE, now_ms);
        }
        break;
    }

    case ST_PARK_REVERSE:
        step_park_reverse(now_ms);
        break;

    case ST_DONE:
        break;

    case ST_FAULT:
    default:
        /* 故障停机后长按 KEY 回到待机（保留已锁存的方向和四色），再选段起跑 */
        if (key_event_valid_after_arm(now_ms, &ev) && (ev == KEY_EVT_LONG)) {
            BspMotor_ClearFault();
            s_fault_reason = "NONE";
            enter_state(ST_IDLE, now_ms);
        }
        break;
    }
}

mission_state_t Mission_GetState(void)
{
    return s_state;
}

const char *Mission_GetStateName(void)
{
    switch (s_state) {
    case ST_IDLE:
        if (s_block_reason != NULL) {
            return s_block_reason;
        }
        (void)snprintf(s_name_buf, sizeof(s_name_buf), "IDLE>%s", start_name(s_start_stage));
        return s_name_buf;
    case ST_TRACK_TO_OBS: return "TRACK_OBS";
    case ST_AVOID_OBS:    return "AVOID_OBS";
    case ST_FORK_PASS:    return s_fork_waiting ? "FORK_WAIT" : "FORK_PASS";
    case ST_COLOR_SCAN:   return "COLOR_SCAN";
    case ST_PARK_SEARCH:  return "PARK_FIND";
    case ST_PARK_REVERSE: return s_park_by_odom ? "PARK_ODOM" : "PARK_REV";
    case ST_DONE:         return s_park_by_odom ? "DONE*" : "DONE";
    case ST_FAULT:        return s_fault_reason;
    default:              return "UNK";
    }
}

const char *Mission_GetFaultReason(void)
{
    return s_fault_reason;
}

/* OLED 显示用：已锁存的方向；兜底猜测时返回小写 'l'/'r' */
char Mission_GetForkDisplay(void)
{
    if (s_locked_fork_dir == 0) {
        return BspOpenmv_GetState()->turn_dir;
    }
    if (s_fork_guess) {
        return (s_locked_fork_dir == 'L') ? 'l' : 'r';
    }
    return s_locked_fork_dir;
}

mission_start_t Mission_GetStartStage(void)
{
    return s_start_stage;
}

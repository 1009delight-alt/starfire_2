/* 主控状态机电脑回放。
 *
 * 编译真实的 App/mission_fsm.c、App/motion_ctrl.c、Bsp/bsp_motor.c、Bsp/bsp_openmv.c，
 * 把串口、按键、灰度、超声换成本文件里可按脚本控制的模拟件（超声读数处理 sonar_logic.c 用真实代码单测）：
 *   - 电机板：解析主控发出的 $pwm，按一阶模型算轮速、积分出累计计数，每 10 ms 回一帧 $MAll；
 *   - 相机：解析 $CMD，回 $ACK、每 500 ms 心跳，按用例配置回转向 / P 牌结果；
 *   - 时钟：1 ms 一步；每 20 ms 依次调用 Mission_Step20ms 和 BspMotor_ControlStep，与 main.c 一致。
 *
 * 运行：test/sim/run_sim.ps1（或按其中的 gcc 命令手动编译）。全部通过时退出码为 0。
 */
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "board.h"
#include "config.h"
#include "bsp_uart.h"
#include "bsp_motor.h"
#include "bsp_openmv.h"
#include "bsp_gray.h"
#include "bsp_sonar.h"
#include "motion_ctrl.h"
#include "mission_fsm.h"
#include "gray_logic.h"
#include "sonar_logic.h"

/* ==================== 时钟 ==================== */
static uint32_t g_now = 0U;
SIM_RCC_t   sim_rcc;
SIM_USART_t sim_usart2;
uint32_t HAL_GetTick(void) { return g_now; }
void HAL_Delay(uint32_t ms) { g_now += ms; }

/* ==================== 串口 ==================== */
#define QCAP 8192
typedef struct { uint8_t buf[QCAP]; size_t head, tail; } queue_t;
static queue_t g_rx[BSP_UART_COUNT];

static void q_push_str(queue_t *q, const char *s)
{
    while (*s) {
        size_t next = (q->head + 1U) % QCAP;
        if (next == q->tail) { return; }
        q->buf[q->head] = (uint8_t)*s++;
        q->head = next;
    }
}

static void motor_on_tx(const char *s);
static void cam_on_tx(const char *s);

void BspUart_Init(bsp_uart_id_t id, uint32_t baud) { (void)id; (void)baud; }
void BspUart_IRQHandler(bsp_uart_id_t id) { (void)id; }
bool BspUart_ReadByte(bsp_uart_id_t id, uint8_t *out)
{
    queue_t *q = &g_rx[id];
    if (q->head == q->tail) { return false; }
    *out = q->buf[q->tail];
    q->tail = (q->tail + 1U) % QCAP;
    return true;
}
void BspUart_FlushRx(bsp_uart_id_t id) { g_rx[id].tail = g_rx[id].head; }
void BspUart_SendBytes(bsp_uart_id_t id, const uint8_t *d, size_t n)
{
    char tmp[256];
    if (n >= sizeof(tmp)) { n = sizeof(tmp) - 1U; }
    memcpy(tmp, d, n);
    tmp[n] = '\0';
    if (id == BSP_UART_MOTOR) { motor_on_tx(tmp); }
    else if (id == BSP_UART_OPENMV) { cam_on_tx(tmp); }
}
void BspUart_SendStr(bsp_uart_id_t id, const char *s) { BspUart_SendBytes(id, (const uint8_t *)s, strlen(s)); }
void BspUart_Printf(bsp_uart_id_t id, const char *fmt, ...) { (void)id; (void)fmt; }

/* ==================== 按键 / LED ==================== */
static board_key_event_t g_key_ev = KEY_EVT_NONE;
static uint32_t g_key_press_ms = 0U;
static bool g_estop = false;
board_key_event_t Board_KeyTakeEvent(void) { board_key_event_t e = g_key_ev; g_key_ev = KEY_EVT_NONE; return e; }
uint32_t Board_KeyLastPressMs(void) { return g_key_press_ms; }
bool Board_KeyIsPressed(void) { return false; }
void Board_KeyResetStartArm(void) { }
bool Board_KeyCheckStartEdge(uint32_t now_ms) { (void)now_ms; bool e = g_estop; g_estop = false; return e; }
void Board_LedSetPattern(board_led_pattern_t p) { (void)p; }
void Board_LedSet(bool on) { (void)on; }

/* ==================== 灰度 / 超声 ==================== */
static gray_state_t g_gray;
static bool g_line = true;
static bool g_gray_scripted = false;    /* true：按 g_gray_raw_hw 走真实的 GrayLogic_Process */
static uint16_t g_gray_raw_hw[8];
#define GRAY_BLACK_LOW_ADC  500U
#define GRAY_WHITE_LOW_ADC  3000U

/* 按“物理上从左到右”的 8 个字符设置灰度图样，'1' = 压线；自动套用装反和黑白极性配置 */
static void gray_pattern(const char *left_to_right, bool black_is_low, bool reverse, uint16_t raw_hw[8])
{
    for (int i = 0; i < 8; ++i) {
        bool black = (left_to_right[i] == '1');
        uint16_t v = black ? GRAY_BLACK_LOW_ADC : GRAY_WHITE_LOW_ADC;
        if (!black_is_low) { v = black ? GRAY_WHITE_LOW_ADC : GRAY_BLACK_LOW_ADC; }
        raw_hw[reverse ? (7 - i) : i] = v;
    }
}

static void sim_gray(const char *left_to_right)
{
    g_gray_scripted = true;
    gray_pattern(left_to_right, CFG_GRAY_BLACK_IS_LOW != 0U, CFG_GRAY_REVERSE_ORDER != 0U, g_gray_raw_hw);
}

void BspGray_Init(void) { }
void BspGray_Sample(gray_bias_mode_t m)
{
    if (g_gray_scripted) {
        GrayLogic_Process(&g_gray, g_gray_raw_hw, m, CFG_GRAY_BLACK_IS_LOW != 0U, CFG_GRAY_REVERSE_ORDER != 0U,
                          (uint8_t)CFG_GRAY_DEAD_MASK);
        return;
    }
    g_gray.line_found = g_line;
    g_gray.black_count = g_line ? 2U : 0U;
    g_gray.is_cross = false;
    g_gray.error = 0.0f;
}
const gray_state_t *BspGray_GetState(void) { return &g_gray; }
void BspGray_ResetTrack(void) { GrayLogic_ResetTrack(&g_gray); }
void BspGray_CalibStart(void) { }
void BspGray_CalibFeed(void) { }
void BspGray_CalibFinishAndApply(void) { }

static sonar_state_t g_sim_sonar;
static bool g_obstacle = false;
static bool g_sonar_ok = true;
bool BspSonar_Init(void) { return true; }
void BspSonar_Update(uint32_t now_ms) { (void)now_ms; }
void BspSonar_IRQHandler(void) { }
void BspSonar_ResetTrigger(void) { }
bool BspSonar_IsHealthy(uint32_t now_ms) { (void)now_ms; return g_sonar_ok; }
bool BspSonar_IsObstacleDetected(uint32_t now_ms) { (void)now_ms; return g_sonar_ok && g_obstacle; }
const sonar_state_t *BspSonar_GetState(void) { return &g_sim_sonar; }

/* ==================== 电机板模型 ==================== */
static int   g_pwm[4];
static float g_v[4];            /* 实际轮速 mm/s（前进为正） */
static int   g_drag_mask = 0;   /* bit i：第 i 轮被车身拖着，速度固定为 g_drag_v */
static float g_drag_v = 0.0f;
static double g_pos[4];         /* 实际轮行程 mm */
static int32_t g_raw_out[4];    /* 回传的累计计数 */
static bool  g_enc_frozen = false;
static bool  g_enc_silent = false;  /* 驱动板一帧 $MAll 都不回 */
static float g_load = 1.0f;     /* 负载系数：越小轮子越难转 */
static int   g_pwm_frames = 0;
static const int k_sign[4] = { CFG_ENC_SIGN_M1, CFG_ENC_SIGN_M2, CFG_ENC_SIGN_M3, CFG_ENC_SIGN_M4 };

static void motor_on_tx(const char *s)
{
    int a, b, c, d;
    if (sscanf(s, "$pwm:%d,%d,%d,%d#", &a, &b, &c, &d) == 4) {
        g_pwm[0] = a; g_pwm[1] = b; g_pwm[2] = c; g_pwm[3] = d;
        g_pwm_frames++;
    }
}

static void motor_world_1ms(void)
{
    for (int i = 0; i < 4; ++i) {
        float u = (float)g_pwm[i];
        float tgt = 0.0f;
        if (fabsf(u) > (float)CFG_WHEEL_PWM_DEADZONE) {
            tgt = (fabsf(u) - (float)CFG_WHEEL_PWM_DEADZONE) / CFG_WHEEL_PID_FF * g_load;
            if (u < 0.0f) { tgt = -tgt; }
        }
        g_v[i] += (tgt - g_v[i]) * (1.0f / 50.0f);      /* 时间常数约 50 ms */
        if ((g_drag_mask & (1 << i)) != 0) { g_v[i] = g_drag_v; }   /* 被车身拖着走：轮速不由本轮 PWM 决定 */
        g_pos[i] += g_v[i] * 0.001;
        if (!g_enc_frozen) {
            g_raw_out[i] = (int32_t)lround(g_pos[i] * CFG_ENC_COUNTS_PER_MM) * k_sign[i];
        }
    }
    if (((g_now % 10U) == 0U) && !g_enc_silent) {
        char f[64];
        snprintf(f, sizeof f, "$MAll:%ld,%ld,%ld,%ld#", (long)g_raw_out[0], (long)g_raw_out[1],
                 (long)g_raw_out[2], (long)g_raw_out[3]);
        q_push_str(&g_rx[BSP_UART_MOTOR], f);
    }
}

/* ==================== 相机模型 ==================== */
static char g_cam_task[8] = "IDLE";
static uint32_t g_cam_task_since = 0U;
static uint32_t g_cam_last_hb = 0U;
static uint32_t g_cam_last_turn = 0U;
static char g_cam_turn_answer = 0;      /* TURN 任务进入 200 ms 后开始回的方向；0 = 不回 */
static bool g_cam_park_aligned = false; /* PARK 任务下每 50 ms 回 $PARK:1,0,300# */
static uint32_t g_cam_last_park = 0U;
static int  g_cam_cmd_count[8];         /* 0 IDLE 1 TURN 2 COLOR 3 PARK 4 RESET 5 QUERY 6 COLOR:续接 */
static char g_cam_order[8];             /* 相机自己数到的颜色，如 "RGB"；重启清空 */
static char g_cam_last_seed[8];

static void cam_send(const char *body)
{
    char f[80];
    snprintf(f, sizeof f, "$%s#\r\n", body);
    q_push_str(&g_rx[BSP_UART_OPENMV], f);
}

static void cam_set_task(const char *t)
{
    if (strcmp(g_cam_task, t) != 0) {
        snprintf(g_cam_task, sizeof g_cam_task, "%s", t);
        g_cam_task_since = g_now;
    }
}

static void cam_send_color(void)
{
    char body[24];
    size_t n = strlen(g_cam_order);
    int k = snprintf(body, sizeof body, "COLOR:%u,", (unsigned)n);
    for (size_t i = 0; i < n; ++i) {
        body[k++] = g_cam_order[i];
        if (i + 1U < n) { body[k++] = '-'; }
    }
    body[k] = '\0';
    cam_send(body);
}

/* 相机看到下一块颜色（已满足连续帧和空框条件）：记入并上报 */
static void cam_see(char c)
{
    size_t n = strlen(g_cam_order);
    if ((strcmp(g_cam_task, "COLOR") != 0) || (n >= 4U)) { return; }
    g_cam_order[n] = c;
    g_cam_order[n + 1U] = '\0';
    cam_send_color();
}

/* $CMD:COLOR:<R-G># 续接：规则与 openmv4h7plus/MAIN/main.py 的 color_resume 相同 */
static void cam_color_resume(const char *seq_dash)
{
    char seed[8];
    size_t n = 0U;
    for (const char *q = seq_dash; (*q != '#') && (*q != '\0') && (n < 7U); ++q) {
        if (*q != '-') { seed[n++] = *q; }
    }
    seed[n] = '\0';
    snprintf(g_cam_last_seed, sizeof g_cam_last_seed, "%s", seed);
    size_t mine = strlen(g_cam_order);
    bool ahead = (mine > n) && (strncmp(g_cam_order, seed, n) == 0);
    if (!ahead) { snprintf(g_cam_order, sizeof g_cam_order, "%s", seed); }
    cam_set_task("COLOR");
    cam_send("ACK:COLOR");
    if (ahead) { cam_send_color(); }
}

static void cam_on_tx(const char *s)
{
    static const char *names[] = { "IDLE", "TURN", "COLOR", "PARK", "RESET", "QUERY" };
    const char *p = s;
    while ((p = strstr(p, "$CMD:")) != NULL) {
        p += 5;
        if (strncmp(p, "COLOR:", 6) == 0) {
            g_cam_cmd_count[6]++;
            cam_color_resume(p + 6);
            continue;
        }
        for (int i = 0; i < 6; ++i) {
            size_t n = strlen(names[i]);
            if ((strncmp(p, names[i], n) == 0) && (p[n] == '#')) {
                g_cam_cmd_count[i]++;
                char ack[16];
                if (i == 4) { cam_set_task("IDLE"); g_cam_order[0] = '\0'; cam_send("ACK:IDLE"); }
                else if (i == 5) { /* QUERY：心跳即可 */ g_cam_last_hb = 0U; }
                else { cam_set_task(names[i]); snprintf(ack, sizeof ack, "ACK:%s", names[i]); cam_send(ack); }
            }
        }
    }
}

static void cam_restart(void)
{
    g_cam_order[0] = '\0';
    snprintf(g_cam_task, sizeof g_cam_task, "IDLE");
    g_cam_task_since = g_now;
    cam_send("HB:IDLE,0");
    g_cam_last_hb = g_now;
}

static void cam_world_1ms(void)
{
    if ((g_now - g_cam_last_hb) >= 500U) {
        char hb[24];
        snprintf(hb, sizeof hb, "HB:%s,25", g_cam_task);
        cam_send(hb);
        g_cam_last_hb = g_now;
    }
    if ((strcmp(g_cam_task, "TURN") == 0) && g_cam_turn_answer &&
        ((g_now - g_cam_task_since) >= 200U) && ((g_now - g_cam_last_turn) >= 200U)) {
        char t[8] = "TURN:x";
        t[5] = g_cam_turn_answer;
        cam_send(t);
        g_cam_last_turn = g_now;
    }
    if ((strcmp(g_cam_task, "PARK") == 0) && g_cam_park_aligned && ((g_now - g_cam_last_park) >= 50U)) {
        cam_send("PARK:1,0,300");
        g_cam_last_park = g_now;
    }
}

/* ==================== 回放驱动 ==================== */
static void sim_reset_world(void)
{
    g_now = 0U;
    memset(g_rx, 0, sizeof g_rx);
    memset(g_pwm, 0, sizeof g_pwm);
    g_drag_mask = 0; g_drag_v = 0.0f;
    memset(g_v, 0, sizeof g_v);
    memset(g_pos, 0, sizeof g_pos);
    memset(g_raw_out, 0, sizeof g_raw_out);
    memset(g_cam_cmd_count, 0, sizeof g_cam_cmd_count);
    g_enc_frozen = false; g_enc_silent = false; g_load = 1.0f; g_line = true; g_obstacle = false; g_sonar_ok = true;
    g_cam_turn_answer = 0; g_cam_park_aligned = false;
    g_cam_order[0] = '\0'; g_cam_last_seed[0] = '\0';
    g_gray_scripted = false;
    memset(&g_gray, 0, sizeof g_gray);
    {
        static const uint16_t th[8] = CFG_GRAY_TH_DEFAULT;
        memcpy(g_gray.thresh, th, sizeof th);
    }
    snprintf(g_cam_task, sizeof g_cam_task, "IDLE");
    g_cam_last_hb = 0U;
    g_key_ev = KEY_EVT_NONE; g_estop = false;

    BspMotor_Init();
    BspOpenmv_Init();
    Motion_Init();
    Mission_Init(MISSION_PROFILE_FULL);
}

/* 推进 ms 毫秒 */
static void run_ms(uint32_t ms)
{
    for (uint32_t k = 0; k < ms; ++k) {
        g_now++;
        motor_world_1ms();
        cam_world_1ms();
        BspMotor_PollRx(g_now);
        BspOpenmv_Poll(g_now);
        if ((g_now % CFG_MAIN_CTRL_PERIOD_MS) == 0U) {
            Mission_Step20ms(g_now, CFG_MAIN_CTRL_PERIOD_MS * 0.001f);
            BspMotor_ControlStep(g_now, CFG_MAIN_CTRL_PERIOD_MS * 0.001f);
        }
    }
}

/* 推进到下一次控制周期之前（不执行该周期） */
static void run_to_before_step(void)
{
    while (((g_now + 1U) % CFG_MAIN_CTRL_PERIOD_MS) != 0U) { run_ms(1); }
}

static void key(board_key_event_t ev)
{
    g_key_ev = ev;
    g_key_press_ms = g_now;
    run_ms(40);
}

/* IDLE 下选段并起跑：长按 n 次（FULL→AVOID→FORK→COLOR→PARK），再短按 */
static void start_stage(int long_presses)
{
    for (int i = 0; i < long_presses; ++i) { key(KEY_EVT_LONG); }
    key(KEY_EVT_SHORT);
}

static bool wait_state(mission_state_t st, uint32_t max_ms)
{
    for (uint32_t t = 0; t < max_ms; t += 20U) {
        if (Mission_GetState() == st) { return true; }
        run_ms(20);
    }
    return Mission_GetState() == st;
}

static int g_fails = 0, g_checks = 0;
static void check(bool ok, const char *fmt, ...)
{
    va_list ap;
    g_checks++;
    if (!ok) { g_fails++; }
    printf("%s ", ok ? "ok  :" : "FAIL:");
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("   [t=%lu ms, state=%s]\n", (unsigned long)g_now, Mission_GetStateName());
}

static bool all_pwm_zero(void) { return !g_pwm[0] && !g_pwm[1] && !g_pwm[2] && !g_pwm[3]; }

/* ==================== 用例 ==================== */

/* 1. 相机重启回到 IDLE，主控应在一个重试周期左右重新下发当前任务 */
static void case_cam_restart(void)
{
    printf("\n== 1 相机重启后恢复任务\n");
    sim_reset_world();
    run_ms(1000);
    start_stage(2);                                   /* 从分岔起跑 */
    run_ms(800);                                      /* TURN 已下发，相机已确认 */
    check(strcmp(g_cam_task, "TURN") == 0, "起跑后相机处于 TURN");
    int before = g_cam_cmd_count[1];
    cam_restart();
    run_ms(CFG_OPENMV_CMD_RETRY_MS + 600U);
    check(g_cam_cmd_count[1] > before, "相机报 HB:IDLE 后主控重发 $CMD:TURN#（重发 %d 次）", g_cam_cmd_count[1] - before);
    check(strcmp(g_cam_task, "TURN") == 0, "相机回到 TURN");
    check(BspOpenmv_GetState()->resync_count >= 1U, "resync_count = %u", BspOpenmv_GetState()->resync_count);
}

/* 2. 残缺 / 不一致的 COLOR 报文不能改写或锁死四色；相机重启后的重新计数不能覆盖已有结果 */
static void case_color_frames(void)
{
    printf("\n== 2 四色报文校验\n");
    sim_reset_world();
    run_ms(1000);
    start_stage(3);                                   /* 从色区起跑 */
    run_ms(800);
    const openmv_state_t *mv = BspOpenmv_GetState();
    check(mv->ack_task == OPENMV_TASK_COLOR, "相机已确认 COLOR");

    q_push_str(&g_rx[BSP_UART_OPENMV], "COLOR:4,R#\r\n");
    run_ms(5);
    check(mv->color_count == 0U && !mv->color_locked, "缺 $ 的残帧（内容也不合法）被丢弃");
    q_push_str(&g_rx[BSP_UART_OPENMV], "COLOR:1,Y#\r\n");
    run_ms(5);
    check(mv->color_count == 0U, "缺 $ 的残帧（内容合法）也被丢弃");
    q_push_str(&g_rx[BSP_UART_OPENMV], "$COLOR:4,R#\r\n");
    run_ms(5);
    check(mv->color_count == 0U && !mv->color_locked, "数量与序列不符的帧被丢弃");
    q_push_str(&g_rx[BSP_UART_OPENMV], "$COLOR:3,R-G-B-Y#\r\n");
    run_ms(5);
    check(mv->color_count == 0U, "序列多于数量的帧被丢弃");
    q_push_str(&g_rx[BSP_UART_OPENMV], "$COLOR:1,R#\r\n");
    run_ms(5);
    check(mv->color_count == 1U && strcmp(mv->color_seq, "R") == 0, "合法帧 R 被接受");
    q_push_str(&g_rx[BSP_UART_OPENMV], "$COLOR:2,G-B#\r\n");
    run_ms(5);
    check(mv->color_count == 1U && mv->color_reject_count == 1U, "不以已有序列开头的帧被丢弃");
    q_push_str(&g_rx[BSP_UART_OPENMV], "$COLOR:2,R-G#\r\n");
    run_ms(5);
    check(mv->color_count == 2U && strcmp(mv->color_seq, "R-G") == 0, "R-G 被接受");

    /* 让模拟相机自己也数到 R-G（与主控一致） */
    cam_see('R');
    cam_see('G');
    run_ms(5);
    check(strcmp(g_cam_order, "RG") == 0 && strcmp(mv->color_seq, "R-G") == 0, "相机与主控都为 R-G");

    /* 一条不带续接的旧式重新计数帧不能覆盖 R-G */
    q_push_str(&g_rx[BSP_UART_OPENMV], "$COLOR:1,B#\r\n");
    run_ms(5);
    check(mv->color_count == 2U && strcmp(mv->color_seq, "R-G") == 0, "不连续的 1,B 不覆盖 R-G");

    /* 相机重启：主控应带着 R-G 重新下发 COLOR，相机续接后由它自己数出第 3、4 块 */
    cam_restart();
    run_ms(CFG_OPENMV_CMD_RETRY_MS + 600U);
    check(strcmp(g_cam_task, "COLOR") == 0, "相机重启后被拉回 COLOR");
    check(g_cam_cmd_count[6] >= 1 && strcmp(g_cam_last_seed, "RG") == 0,
          "主控下发的是续接命令 $CMD:COLOR:R-G#（收到 %d 次）", g_cam_cmd_count[6]);
    check(strcmp(g_cam_order, "RG") == 0, "相机按续接恢复为 R-G");
    cam_see('B');
    cam_see('Y');
    run_ms(5);
    check(mv->color_locked && strcmp(mv->color_seq, "R-G-B-Y") == 0,
          "相机续接后自己数出的 R-G-B-Y 被锁存（主控为 %s）", mv->color_seq);
}

/* 8. 畸形帧表：每一条都必须整帧丢弃、状态不变；最后用一条合法帧做对照 */
static void case_bad_frames(void)
{
    static const char *bad[] = {
        "$PARK:2,0,300#", "$PARK:1,0,300x#", "$PARK:1,101,300#", "$PARK:1,0,-5#", "$PARK:1,0,99999#",
        "$PARK:,0,300#", "$PARK:1,0#", "$PARK:1, 0,300#", "$PARK:1,0,300,7#", "$PARK:-1,0,300#",
        "$TURN:Lx#", "$TURN:#", "$TURN:l#",
        "$HB:PARK#", "$HB:FOO,20#", "$HB:IDLE,-1#", "$HB:IDLE,300#", "$HB:IDLE,25x#", "$HB:,25#",
        "$ACK:PARKX#", "$ACK:#", "$XYZ#", "$COLOR:5,R#", "$COLOR:1,R-G#",
    };
    printf("\n== 8 畸形帧表\n");
    sim_reset_world();
    run_ms(1000);
    start_stage(4);                                   /* 找库：相机在 PARK */
    run_ms(800);
    const openmv_state_t *mv = BspOpenmv_GetState();
    check(mv->ack_task == OPENMV_TASK_PARK && Mission_GetState() == ST_PARK_SEARCH, "相机已确认 PARK");
    int bad_ok = 0;
    for (size_t i = 0; i < sizeof bad / sizeof bad[0]; ++i) {
        openmv_state_t before = *mv;
        uint16_t bad_before = mv->bad_frame_count;
        run_to_before_step();
        char f[64];
        snprintf(f, sizeof f, "%s\r\n", bad[i]);
        q_push_str(&g_rx[BSP_UART_OPENMV], f);
        run_ms(1);
        bool same = (mv->park_seen == before.park_seen) && (mv->park_cx == before.park_cx) &&
                    (mv->park_area == before.park_area) && (mv->ack_task == before.ack_task) &&
                    (mv->fps == before.fps) && (mv->turn_dir == before.turn_dir) &&
                    (mv->color_count == before.color_count) && (mv->bad_frame_count == bad_before + 1U) &&
                    (Mission_GetState() == ST_PARK_SEARCH);
        if (same) { bad_ok++; } else { check(false, "畸形帧 %s 改变了状态或未计数", bad[i]); }
    }
    check(bad_ok == (int)(sizeof bad / sizeof bad[0]), "%d 条畸形帧全部丢弃且状态不变", bad_ok);

    /* 电机串口：第 4 个数后多余字符的 $MAll 也要整帧丢弃 */
    q_push_str(&g_rx[BSP_UART_MOTOR], "$MAll:999999,0,0,0x#");
    run_ms(1);
    check(BspMotor_GetState()->raw_enc[0] != 999999, "$MAll 带多余字符整帧丢弃");

    run_to_before_step();
    q_push_str(&g_rx[BSP_UART_OPENMV], "$PARK:1,0,300#\r\n");
    run_ms(1);
    check(Mission_GetState() == ST_PARK_REVERSE, "对照：合法的 $PARK:1,0,300# 触发倒库");
}

/* 9. 分岔左右选线：同一双线图样，锁 L 时左轮慢、锁 R 时右轮慢（经真实的灰度计算与循迹控制） */
static float fork_wheel_diff(char answer, bool rerun)
{
    if (!rerun) {
        sim_reset_world();
        run_ms(1000);
        sim_gray("00011000");
        g_cam_turn_answer = answer;
        start_stage(2);
    } else {
        g_estop = true;
        run_ms(40);
        key(KEY_EVT_LONG);
        sim_gray("00011000");
        g_cam_turn_answer = answer;
        key(KEY_EVT_SHORT);
    }
    run_ms(2600);                                     /* 走过偏置起点（初稿 350 mm） */
    sim_gray("11000011");                             /* 左右各一条线 */
    run_ms(40);
    const motor_state_t *m = BspMotor_GetState();
    float left = 0.5f * (m->target_spd_mm_s[0] + m->target_spd_mm_s[1]);
    float right = 0.5f * (m->target_spd_mm_s[2] + m->target_spd_mm_s[3]);
    check(Mission_GetForkDisplay() == answer, "锁定 %c", answer);
    printf("      左轮目标 %.0f mm/s，右轮目标 %.0f mm/s\n", left, right);
    return left - right;
}

static void case_fork_steer(void)
{
    printf("\n== 9 分岔左右选线\n");
    float dl = fork_wheel_diff('L', false);
    check(dl < -50.0f, "锁 L：左轮比右轮慢（差 %.0f mm/s）", dl);
    float dr = fork_wheel_diff('R', true);
    check(dr > 50.0f, "锁 R（重跑）：右轮比左轮慢（差 %.0f mm/s）", dr);
}

/* 10. 灰度装反、黑白极性相反时，只改配置就能得到正确的方向 */
static void case_gray_config(void)
{
    printf("\n== 10 灰度装反与极性\n");
    for (int rev = 0; rev <= 1; ++rev) {
        for (int low = 0; low <= 1; ++low) {
            gray_state_t st;
            uint16_t raw[8];
            static const uint16_t th[8] = CFG_GRAY_TH_DEFAULT;
            memset(&st, 0, sizeof st);
            memcpy(st.thresh, th, sizeof th);
            gray_pattern("11000000", low != 0, rev != 0, raw);       /* 线在物理左侧 */
            GrayLogic_Process(&st, raw, GRAY_BIAS_CENTER, low != 0, rev != 0, 0U);
            check(st.line_found && st.error < -2.0f, "装反=%d 黑为低=%d：线在左 → 误差 %.2f < 0", rev, low, st.error);
            gray_pattern("11000011", low != 0, rev != 0, raw);
            GrayLogic_Process(&st, raw, GRAY_BIAS_RIGHT, low != 0, rev != 0, 0U);
            check(st.is_split && st.error > 2.0f, "装反=%d 黑为低=%d：双线偏右 → 误差 %.2f > 0", rev, low, st.error);
        }
    }
}

/* 11. 坏通道屏蔽：坏通道读数不参与判线；两侧都压线时补上，不把一条线误判成分岔 */
static void case_gray_dead(void)
{
    printf("\n== 11 灰度坏通道屏蔽\n");
    static const uint16_t th[8] = CFG_GRAY_TH_DEFAULT;
    const uint8_t dead = (uint8_t)(1U << 6);
    for (int rev = 0; rev <= 1; ++rev) {
        gray_state_t st;
        uint16_t raw[8];
        memset(&st, 0, sizeof st);
        memcpy(st.thresh, th, sizeof th);

        gray_pattern("00000010", true, rev != 0, raw);            /* 只有坏通道读到“黑” */
        GrayLogic_Process(&st, raw, GRAY_BIAS_CENTER, true, rev != 0, dead);
        check(!st.line_found && st.bits == 0U, "装反=%d：坏通道单独读黑被忽略（bits=0x%02X）", rev, st.bits);

        gray_pattern("00000101", true, rev != 0, raw);            /* 线跨过坏通道 */
        GrayLogic_Process(&st, raw, GRAY_BIAS_CENTER, true, rev != 0, dead);
        check(!st.is_split && st.bits == 0xE0U && st.error > 2.0f,
              "装反=%d：两侧压线时补上，不判分岔（bits=0x%02X 误差 %.2f）", rev, st.bits, st.error);

        gray_pattern("00001000", true, rev != 0, raw);            /* 远离坏通道，结果不受影响 */
        GrayLogic_Process(&st, raw, GRAY_BIAS_CENTER, true, rev != 0, dead);
        check(st.bits == 0x10U && st.error > 0.0f && st.error < 1.0f,
              "装反=%d：其他通道照常（bits=0x%02X 误差 %.2f）", rev, st.bits, st.error);
    }
}

/* 4a. 定距动作中编码器冻结（照常回帧、计数不变） */
static void case_frozen_in_drive(void)
{
    printf("\n== 4a 入库定距动作中编码器冻结\n");
    sim_reset_world();
    run_ms(1000);
    g_cam_park_aligned = true;
    start_stage(4);                                   /* 从找库起跑 */
    check(wait_state(ST_PARK_REVERSE, 3000U), "看到 P 牌进入倒库");
    g_enc_frozen = true;
    uint32_t t0 = g_now;
    bool faulted = wait_state(ST_FAULT, 3000U);
    check(faulted && strcmp(Mission_GetStateName(), "ENC_STALL") == 0,
          "冻结后报 ENC_STALL，用时 %lu ms", (unsigned long)(g_now - t0));
    run_ms(300);
    check(all_pwm_zero(), "电机指令已归零");
}

/* 4b. 循迹中编码器冻结 */
static void case_frozen_in_track(void)
{
    printf("\n== 4b 循迹中编码器冻结\n");
    sim_reset_world();
    run_ms(1000);
    start_stage(0);                                   /* 全程 */
    run_ms(1500);
    check(Mission_GetState() == ST_TRACK_TO_OBS, "正在循迹");
    g_enc_frozen = true;
    uint32_t t0 = g_now;
    bool faulted = wait_state(ST_FAULT, 3000U);
    check(faulted && strcmp(Mission_GetStateName(), "ENC_STALL") == 0,
          "循迹中冻结报 ENC_STALL，用时 %lu ms", (unsigned long)(g_now - t0));
    run_ms(300);
    check(all_pwm_zero(), "电机指令已归零");
}

/* 4c. 轮子能转但极慢（超过堵转阈值）：靠动作超时兜底 */
static void case_motion_timeout(void)
{
    printf("\n== 4c 轮子极慢、到不了目标\n");
    sim_reset_world();
    run_ms(1000);
    g_cam_park_aligned = true;
    start_stage(4);
    check(wait_state(ST_PARK_REVERSE, 3000U), "进入倒库");
    g_load = 0.05f;
    uint32_t t0 = g_now;
    bool faulted = wait_state(ST_FAULT, 15000U);
    check(faulted && strcmp(Mission_GetStateName(), "MOTION_TMO") == 0,
          "动作超时报 MOTION_TMO，用时 %lu ms", (unsigned long)(g_now - t0));
    run_ms(300);
    check(all_pwm_zero(), "电机指令已归零");
}

/* 5. 分岔重跑：上一次留下 L，本轮相机只回 R */
static void case_fork_rerun(void)
{
    printf("\n== 5 分岔重跑不沿用旧方向\n");
    sim_reset_world();
    run_ms(1000);
    g_cam_turn_answer = 'L';
    start_stage(2);
    run_ms(1500);
    check(Mission_GetForkDisplay() == 'L', "第一次锁定 L");
    g_estop = true;
    run_ms(40);
    check(Mission_GetState() == ST_FAULT, "按键急停");
    key(KEY_EVT_LONG);                                /* 回待机，起跑段仍是 FORK */
    check(Mission_GetState() == ST_IDLE, "回到待机");
    g_cam_turn_answer = 'R';
    key(KEY_EVT_SHORT);
    run_ms(60);
    check(Mission_GetForkDisplay() != 'L', "重跑开始时不锁旧方向（当前 %c）",
          Mission_GetForkDisplay() ? Mission_GetForkDisplay() : '-');
    run_ms(2500);
    check(Mission_GetForkDisplay() == 'R', "最终锁定本轮的 R（当前 %c）",
          Mission_GetForkDisplay() ? Mission_GetForkDisplay() : '-');

    /* 无云台后 view_set 在任务相同时不重发：起跑前相机已在 TURN 且留有旧方向，进分岔不能直接沿用 */
    sim_reset_world();
    run_ms(1000);
    g_cam_turn_answer = 'L';
    BspOpenmv_SetTask(OPENMV_TASK_TURN);
    run_ms(500);
    check(BspOpenmv_GetState()->turn_dir == 'L', "起跑前驱动层留有旧方向 L");
    g_cam_turn_answer = 'R';
    g_cam_last_turn = g_now;                          /* 下一帧方向 200 ms 后才到 */
    start_stage(2);
    run_ms(20);
    check(Mission_GetForkDisplay() != 'L', "任务未重发时也不锁旧方向（当前 %c）",
          Mission_GetForkDisplay() ? Mission_GetForkDisplay() : '-');
    BspMotor_Stop();
}

/* 6. 同一控制周期内先“已对齐”后“已丢失” */
static void case_park_flip(void)
{
    printf("\n== 6 P 牌对齐后同周期丢失\n");
    sim_reset_world();
    run_ms(1000);
    start_stage(4);
    run_ms(800);
    check(BspOpenmv_GetState()->ack_task == OPENMV_TASK_PARK, "相机已确认 PARK");
    run_to_before_step();
    q_push_str(&g_rx[BSP_UART_OPENMV], "$PARK:1,0,300#\r\n$PARK:0,0,0#\r\n");
    run_ms(60);
    check(Mission_GetState() == ST_PARK_SEARCH, "最新一帧为丢失时不开始倒库");
    run_to_before_step();
    q_push_str(&g_rx[BSP_UART_OPENMV], "$PARK:1,0,300#\r\n");
    run_ms(1);
    check(Mission_GetState() == ST_PARK_REVERSE, "最新一帧为对齐时开始倒库");
    run_ms(CFG_PARK_FRESH_MS + 100U);
}

/* 7. 正常路径：找库 → 倒库 → 停稳 5 s → DONE */
static void case_park_happy(void)
{
    printf("\n== 7 正常入库流程\n");
    sim_reset_world();
    run_ms(1000);
    g_cam_park_aligned = true;
    start_stage(4);
    check(wait_state(ST_PARK_REVERSE, 3000U), "看到 P 牌进入倒库");
    g_cam_park_aligned = false;
    check(wait_state(ST_DONE, 20000U), "完成入库进入 DONE");
    check(all_pwm_zero(), "停车");
}

/* 12. 循迹微分：复位后第一拍不突跳；误差在相邻两格之间来回跳时，D 项被低通压住 */
static float line_step_diff(const char *pat)
{
    sim_gray(pat);
    Motion_LineFollowStep(140.0f, GRAY_BIAS_CENTER, 0.02f);
    const motor_state_t *m = BspMotor_GetState();
    float left = 0.5f * (m->target_spd_mm_s[0] + m->target_spd_mm_s[1]);
    float right = 0.5f * (m->target_spd_mm_s[2] + m->target_spd_mm_s[3]);
    return 0.5f * (left - right);
}

static void case_line_deriv(void)
{
    printf("\n== 12 循迹微分低通\n");
    sim_reset_world();
    Motion_ResetLinePid();
    float d0 = line_step_diff("00001100");                /* 误差 +1.0 */
    float p0 = Motion_LinePTerm(BspGray_GetState()->error);
    check(fabsf(d0 - p0) < 1.0f, "复位后第一拍只有 P 项（差速 %.1f，P %.1f）", d0, p0);

    float worst = 0.0f;
    for (int i = 0; i < 40; ++i) {                        /* 0 与 +0.5 每拍交替 */
        float d = line_step_diff((i & 1) ? "00011000" : "00001000");
        float dterm = fabsf(d - Motion_LinePTerm(BspGray_GetState()->error));
        if ((i >= 20) && (dterm > worst)) { worst = dterm; }
    }
    float raw = CFG_LINE_PID_KD * 0.5f;
    check(worst < 0.5f * raw, "误差跳格时 D 项峰值 %.1f < 不滤波时 %.1f 的一半", worst, raw);
    BspMotor_Stop();
}

/* 13. 内侧轮被拖着走：目标为正时允许有限的制动 PWM，但不超过 CFG_WHEEL_BRAKE_PWM */
static void case_wheel_brake(void)
{
    printf("\n== 13 内侧轮制动\n");
    sim_reset_world();
    run_ms(200);
    BspMotor_SetDiffSpeed(300.0f, 40.0f);
    g_drag_mask = (1 << 2) | (1 << 3);                  /* 右侧两轮被拖到 250 mm/s */
    g_drag_v = 250.0f;
    int min_pwm = 0;
    for (int t = 0; t < 1500; t += 20) {
        run_ms(20);
        if (g_pwm[2] < min_pwm) { min_pwm = g_pwm[2]; }
    }
    check(min_pwm < 0, "被拖的内侧轮输出制动 PWM（最低 %d）", min_pwm);
    check(min_pwm >= -CFG_WHEEL_BRAKE_PWM, "制动 PWM 不超过限幅 %d", CFG_WHEEL_BRAKE_PWM);
    check(g_pwm[0] > 0, "外侧轮仍正向驱动（%d）", g_pwm[0]);
    g_drag_mask = 0;
    BspMotor_Stop();
    run_ms(300);
}

/* 14. 直角弯相关：丢线找线差速单独限幅；降速连续；轮子正反转切换时清积分 */
static void case_corner(void)
{
    printf("\n== 14 直角弯\n");
    sim_reset_world();
    Motion_ResetLinePid();
    (void)line_step_diff("10000000");                     /* 线在最左，记下方向 */
    float lost = 0.0f;
    /* 丢线误差按一阶低通逼近 ±3.8，多走几帧让它到边界附近，才能看出差速限幅 */
    for (int i = 0; i < 40; ++i) { lost = line_step_diff("00000000"); }
    check(fabsf(lost) <= CFG_LINE_LOST_DIFF_MM_S + 0.5f && lost < 0.0f,
          "丢线找线差速 %.0f，方向向左且不超过 %.0f", lost, CFG_LINE_LOST_DIFF_MM_S);

    Motion_ResetLinePid();
    (void)line_step_diff("00011100");                     /* 误差 +0.5 */
    const motor_state_t *m = BspMotor_GetState();
    float avg_a = 0.25f * (m->target_spd_mm_s[0] + m->target_spd_mm_s[1] + m->target_spd_mm_s[2] + m->target_spd_mm_s[3]);
    Motion_ResetLinePid();
    (void)line_step_diff("00001110");                     /* 误差 +1.5 */
    float avg_b = 0.25f * (m->target_spd_mm_s[0] + m->target_spd_mm_s[1] + m->target_spd_mm_s[2] + m->target_spd_mm_s[3]);
    float want_a = 140.0f * (1.0f - CFG_LINE_SLOW_PER_ERR * 0.5f);
    float want_b = 140.0f * (1.0f - CFG_LINE_SLOW_PER_ERR * 1.5f);
    check(fabsf(avg_a - want_a) < 1.0f && fabsf(avg_b - want_b) < 1.0f,
          "降速连续：误差 0.5 → %.1f（应 %.1f），1.5 → %.1f（应 %.1f）", avg_a, want_a, avg_b, want_b);
    BspMotor_Stop();

    /* 轮子带不动时积分攒满，再把目标改成反向：第一拍输出就该是反向 */
    sim_reset_world();
    run_ms(200);
    g_load = 0.15f;
    BspMotor_SetDiffSpeed(300.0f, 300.0f);
    run_ms(1500);
    BspMotor_SetDiffSpeed(-30.0f, 300.0f);
    /* 目标按加速度上限从 300 降到 −30，约 (330/40) 拍后才过零；过零那一拍起输出就该是反向 */
    const int flip_steps = (int)ceilf(300.0f / (CFG_WHEEL_ACC_MAX_MM_S2 * 0.02f));
    run_to_before_step();
    run_ms(1);                                            /* 第 1 拍 */
    for (int k = 1; k < flip_steps; ++k) { run_ms(20); }  /* 第 flip_steps 拍：目标刚过零 */
    check(g_pwm[0] < 0, "目标过零后的第一拍左轮 PWM 为反向（%d）", g_pwm[0]);
    g_load = 1.0f;
    BspMotor_Stop();
    run_ms(300);
}


/* 15. 超声：读数处理（真实 sonar_logic.c）、失效急停、视觉任务立即下发（无云台） */
static void sonar_feed_mm(sonar_state_t *s, uint32_t mm, uint32_t t)
{
    SonarLogic_FeedEcho(s, (uint32_t)((float)mm / CFG_SONAR_MM_PER_US + 0.5f), t,
                        CFG_SONAR_MM_PER_US, CFG_SONAR_MAX_MM, CFG_OBS_TRIG_DIST_MM);
}

static void case_sonar(void)
{
    printf("\n== 15 超声与固定相机\n");
    sonar_state_t s;
    SonarLogic_Reset(&s);
    check(!s.online && !s.valid && s.near_hits == 0U, "复位后离线、无有效读数");

    sonar_feed_mm(&s, 500U, 100U);
    check(s.online && s.last_update_ms == 100U && s.raw_mm >= 499U && s.raw_mm <= 501U,
          "回波 -> 距离换算（500 mm -> %u）", (unsigned)s.raw_mm);

    /* 远处：38 ms 无目标回波记为空旷，算正常读数 */
    SonarLogic_Reset(&s);
    SonarLogic_FeedEcho(&s, 38000U, 10U, CFG_SONAR_MM_PER_US, CFG_SONAR_MAX_MM, CFG_OBS_TRIG_DIST_MM);
    check(s.online && !s.valid && s.raw_mm == SONAR_FAR_MM && s.near_hits == 0U, "无目标回波记为空旷且不算近");

    /* 单次近读数（地面反射等）不触发；连续近读数才累计 */
    SonarLogic_Reset(&s);
    sonar_feed_mm(&s, 900U, 0U); sonar_feed_mm(&s, 900U, 60U);
    sonar_feed_mm(&s, 150U, 120U);
    check(s.near_hits == 0U, "单次近读数被中值滤掉（hits=%u）", (unsigned)s.near_hits);
    sonar_feed_mm(&s, 900U, 180U); sonar_feed_mm(&s, 900U, 200U);   /* 把 150 挤出中值窗口 */
    sonar_feed_mm(&s, 200U, 240U);
    check(s.near_hits == 0U, "一次近读数不进入触发距离（hits=%u）", (unsigned)s.near_hits);
    sonar_feed_mm(&s, 190U, 300U);
    check(s.near_hits == 1U && s.dist_mm <= CFG_OBS_TRIG_DIST_MM, "两次近读数后中值进入触发距离（%u mm）", (unsigned)s.dist_mm);
    sonar_feed_mm(&s, 180U, 360U);
    check(s.near_hits >= CFG_OBS_CONFIRM_FRAMES, "连续近读数达到确认次数（hits=%u）", (unsigned)s.near_hits);

    /* 近距中偶尔漏一次回波（超量程）不应清零 */
    sonar_feed_mm(&s, 5000U, 420U);
    check(s.near_hits >= CFG_OBS_CONFIRM_FRAMES, "近距中漏一次回波不清零（hits=%u）", (unsigned)s.near_hits);
    sonar_feed_mm(&s, 5000U, 480U);
    check(s.near_hits == 0U, "连续两次空旷后清零");

    /* 视觉任务切换立即下发：从分岔起跑，40 ms 内相机就应收到 TURN */
    sim_reset_world();
    run_ms(1000);
    int turn0 = g_cam_cmd_count[1];
    start_stage(2);
    check(g_cam_cmd_count[1] > turn0 && strcmp(g_cam_task, "TURN") == 0, "起跑分岔段后立即下发 TURN（无云台等待）");
    BspMotor_Stop();

    /* 避障窗口内超声失效：急停报 SONAR_LOST */
    sim_reset_world();
    run_ms(1000);
    start_stage(1);                                   /* 从避障段起跑：门控立即生效 */
    run_ms(200);
    check(Mission_GetState() == ST_TRACK_TO_OBS, "避障段起跑后在循迹");
    g_sonar_ok = false;
    run_ms(60);
    check(Mission_GetState() == ST_FAULT && strcmp(Mission_GetFaultReason(), "SONAR_LOST") == 0,
          "超声失效急停（%s）", Mission_GetFaultReason());
    check(all_pwm_zero(), "急停后 PWM 全零");

    /* IDLE 下超声失效不允许从完整赛程起跑 */
    sim_reset_world();
    g_sonar_ok = false;
    run_ms(1000);
    key(KEY_EVT_SHORT);
    check(Mission_GetState() == ST_IDLE && strcmp(Mission_GetStateName(), "WAIT:SONAR") == 0,
          "超声失效时不起跑（%s）", Mission_GetStateName());
}


/* 16. 丢线找线近原地转：按转角也要报 LINE_LOST，不能转好几圈 */
static void case_lost_spin(void)
{
    printf("\n== 16 丢线转角保护\n");
    sim_reset_world();
    run_ms(1000);
    start_stage(2);                                   /* 从分岔起跑：没有避障门控干扰 */
    g_cam_turn_answer = 'R';
    sim_gray("00000011");                             /* 线在最右 */
    run_ms(300);
    float yaw0 = BspMotor_GetState()->total_yaw_deg;
    float dist0 = BspMotor_GetState()->total_dist_mm;
    sim_gray("00000000");                             /* 丢线 */
    bool got = wait_state(ST_FAULT, 6000U);
    float turned = fabsf(BspMotor_GetState()->total_yaw_deg - yaw0);
    check(got && strcmp(Mission_GetFaultReason(), "LINE_LOST") == 0, "丢线后报 LINE_LOST（%s）", Mission_GetFaultReason());
    check(turned <= CFG_LINE_LOST_MAX_DEG + 60.0f, "停下前转过 %.0f°，不超过 %.0f° 太多", turned, CFG_LINE_LOST_MAX_DEG);
    float moved = fabsf(BspMotor_GetState()->total_dist_mm - dist0);
    check(moved < CFG_LINE_LOST_MAX_MM, "是转角先到限（丢线后前进 %.0f mm）", moved);
}

/* 17. 路口一律直行：横线压满大半排时误差保持上一拍；横线残段与主线分成两段时跟主线；分岔不算路口 */
static void case_cross(void)
{
    printf("\n== 17 路口直行\n");
    static const uint16_t th[8] = CFG_GRAY_TH_DEFAULT;
    for (int rev = 0; rev <= 1; ++rev) {
        gray_state_t st;
        uint16_t raw[8];
        memset(&st, 0, sizeof st);
        memcpy(st.thresh, th, sizeof th);

        gray_pattern("00011100", true, rev != 0, raw);            /* 主线略偏右，误差 +0.5 */
        GrayLogic_Process(&st, raw, GRAY_BIAS_CENTER, true, rev != 0, 0U);
        float e0 = st.error;
        check(!st.is_cross && fabsf(e0 - 0.5f) < 0.01f, "装反=%d：正常压线不判路口（误差 %.2f）", rev, e0);

        gray_pattern("11111100", true, rev != 0, raw);            /* T 字：支线从左边接进来 */
        GrayLogic_Process(&st, raw, GRAY_BIAS_CENTER, true, rev != 0, 0U);
        check(fabsf(st.error - e0) < 0.01f, "装反=%d：左侧 T 字第一帧（未确认）误差已保持 %.2f（得 %.2f）",
              rev, e0, st.error);
        for (unsigned k = 1U; k < CFG_GRAY_CONFIRM_FRAMES; ++k) {
            check(!st.is_cross, "装反=%d：T 字第 %u 帧还不置路口标志", rev, k);
            GrayLogic_Process(&st, raw, GRAY_BIAS_CENTER, true, rev != 0, 0U);
        }
        check(st.is_cross && fabsf(st.error - e0) < 0.01f, "装反=%d：左侧 T 字连续 %u 帧判路口，误差保持 %.2f（得 %.2f）",
              rev, (unsigned)CFG_GRAY_CONFIRM_FRAMES, e0, st.error);
        gray_pattern("11111111", true, rev != 0, raw);            /* 十字：整排压黑 */
        GrayLogic_Process(&st, raw, GRAY_BIAS_CENTER, true, rev != 0, 0U);
        check(st.is_cross && fabsf(st.error - e0) < 0.01f && fabsf(st.last_valid_err - e0) < 0.01f,
              "装反=%d：十字判路口，误差和方向记忆都不变（%.2f / %.2f）", rev, st.error, st.last_valid_err);

        gray_pattern("11100110", true, rev != 0, raw);            /* 斜压横线：左边残段 + 主线 */
        GrayLogic_Process(&st, raw, GRAY_BIAS_CENTER, true, rev != 0, 0U);
        check(!st.is_cross && st.error > 0.5f, "装反=%d：横线残段与主线分开时跟主线（误差 %.2f > 0）", rev, st.error);

        gray_pattern("11100111", true, rev != 0, raw);            /* 分岔：两条分开的宽线 */
        GrayLogic_Process(&st, raw, GRAY_BIAS_RIGHT, true, rev != 0, 0U);
        check(!st.is_cross && st.is_split && st.error > 2.0f, "装反=%d：分岔不算路口，偏右选线照常（误差 %.2f）",
              rev, st.error);
    }

    /* 走真实的循迹：压上左侧 T 字时差速不向左打 */
    sim_reset_world();
    Motion_ResetLinePid();
    (void)line_step_diff("00011100");
    float d = line_step_diff("11111100");
    check(d > 0.0f, "循迹压上左侧 T 字：差速仍按原偏差向右（%.1f mm/s），不拐进支线", d);
    BspMotor_Stop();
}

/* 18. P 项变增益：小偏差比线性 P 软（直道不摆），大偏差比线性 P 硬（圆弧弯转得快） */
static void case_line_gain(void)
{
    printf("\n== 18 循迹 P 变增益\n");
    float p_small = Motion_LinePTerm(0.5f), p_big = Motion_LinePTerm(2.5f);
    float want_small = CFG_LINE_PID_KP * 0.5f * (CFG_LINE_KP_CENTER_SCALE + CFG_LINE_KP_EDGE_GAIN * 0.5f);
    float want_big = CFG_LINE_PID_KP * 2.5f * (CFG_LINE_KP_CENTER_SCALE + CFG_LINE_KP_EDGE_GAIN * 2.5f);
    /* 按当前配置核对（CENTER=1、EDGE=0 时就是线性 P） */
    check(fabsf(p_small - want_small) < 0.01f && fabsf(p_big - want_big) < 0.01f,
          "误差 0.5 / 2.5：P %.1f / %.1f（按配置应 %.1f / %.1f）", p_small, p_big, want_small, want_big);
    check(fabsf(Motion_LinePTerm(-2.5f) + p_big) < 0.01f, "左右对称");
    float prev = 0.0f;
    bool mono = true;
    for (int i = 1; i <= 38; ++i) {
        float p = Motion_LinePTerm(0.1f * (float)i);
        if (p <= prev) { mono = false; }
        prev = p;
    }
    check(mono, "0~3.8 单调增，不会偏得越多纠得越少");
}

/* 19. 灰度回差与两帧确认：线边缘读数在阈值附近抖时黑白不来回翻；全白、路口都要连续确认 */
static void case_gray_hyst(void)
{
    printf("\n== 19 灰度回差与两帧确认\n");
    static const uint16_t th[8] = CFG_GRAY_TH_DEFAULT;
    gray_state_t st;
    uint16_t raw[8];
    memset(&st, 0, sizeof st);
    memcpy(st.thresh, th, sizeof th);
    const uint16_t t = th[3];

    /* 通道 4 一直压线；通道 3 在阈值附近变化（不装反、黑为低、不屏蔽） */
    gray_pattern("00001000", true, false, raw);
    raw[3] = (uint16_t)(t + CFG_GRAY_HYST_ADC + 20U);            /* 明显白 */
    GrayLogic_Process(&st, raw, GRAY_BIAS_CENTER, true, false, 0U);
    raw[3] = (uint16_t)(t - CFG_GRAY_HYST_ADC / 2U);             /* 略低于阈值，但没越过回差 */
    GrayLogic_Process(&st, raw, GRAY_BIAS_CENTER, true, false, 0U);
    check((st.bits & 0x08U) == 0U, "白 → 略低于阈值（%u，阈值 %u）仍判白", raw[3], t);
    raw[3] = (uint16_t)(t - CFG_GRAY_HYST_ADC - 20U);            /* 越过回差 */
    GrayLogic_Process(&st, raw, GRAY_BIAS_CENTER, true, false, 0U);
    check((st.bits & 0x08U) != 0U, "越过 阈值−回差（%u）才变黑", raw[3]);
    raw[3] = (uint16_t)(t + CFG_GRAY_HYST_ADC / 2U);             /* 略高于阈值 */
    GrayLogic_Process(&st, raw, GRAY_BIAS_CENTER, true, false, 0U);
    check((st.bits & 0x08U) != 0U, "黑 → 略高于阈值（%u）仍判黑", raw[3]);
    raw[3] = (uint16_t)(t + CFG_GRAY_HYST_ADC + 20U);
    GrayLogic_Process(&st, raw, GRAY_BIAS_CENTER, true, false, 0U);
    check((st.bits & 0x08U) == 0U, "越过 阈值+回差（%u）才变白", raw[3]);

    /* 丢线确认：全白一帧不算丢线，误差保持；连续 CONFIRM 帧才算 */
    gray_pattern("00000011", true, false, raw);                  /* 线在右，误差 +3.0 */
    GrayLogic_Process(&st, raw, GRAY_BIAS_CENTER, true, false, 0U);
    float e0 = st.error;
    gray_pattern("00000000", true, false, raw);
    GrayLogic_Process(&st, raw, GRAY_BIAS_CENTER, true, false, 0U);
    check(st.line_found && fabsf(st.error - e0) < 0.01f, "全白第 1 帧不算丢线，误差保持 %.2f（得 %.2f）", e0, st.error);
    for (unsigned k = 1U; k < CFG_GRAY_CONFIRM_FRAMES; ++k) {
        GrayLogic_Process(&st, raw, GRAY_BIAS_CENTER, true, false, 0U);
    }
    check(!st.line_found && st.error > e0 + 0.01f && st.error < 3.79f,
          "连续 %u 帧全白才丢线，向右找线，误差从 %.2f 往 3.8 靠（得 %.2f）",
          (unsigned)CFG_GRAY_CONFIRM_FRAMES, e0, st.error);
    gray_pattern("00000100", true, false, raw);
    GrayLogic_Process(&st, raw, GRAY_BIAS_CENTER, true, false, 0U);
    check(st.line_found, "见到线一帧就算找回");

    /* 上电后从没见过线：全白仍是“没找到” */
    gray_state_t st2;
    memset(&st2, 0, sizeof st2);
    memcpy(st2.thresh, th, sizeof th);
    gray_pattern("00000000", true, false, raw);
    GrayLogic_Process(&st2, raw, GRAY_BIAS_CENTER, true, false, 0U);
    check(!st2.line_found, "上电全白不算找到线");
}

/* 20. 路口保持封顶：短横线照常直行；整排一直压黑（阈值设错、大片黑区）不能一直直行，要报 LINE_LOST */
static void case_cross_cap(void)
{
    printf("\n== 20 路口保持封顶\n");
    sim_reset_world();
    run_ms(1000);
    sim_gray("00011000");
    start_stage(0);                                   /* 全程：先循迹 */
    run_ms(500);
    sim_gray("11111111");                             /* 短横线：约 100 ms，远小于封顶距离 */
    run_ms(100);
    sim_gray("00011000");
    run_ms(200);
    check(Mission_GetState() == ST_TRACK_TO_OBS && Motion_GetLineLostMm() == 0.0f,
          "压过短横线照常循迹（%s）", Mission_GetStateName());

    float d0 = BspMotor_GetState()->total_dist_mm;
    sim_gray("11111111");                             /* 一直整排压黑 */
    bool got = wait_state(ST_FAULT, 8000U);
    float moved = BspMotor_GetState()->total_dist_mm - d0;
    check(got && strcmp(Mission_GetFaultReason(), "LINE_LOST") == 0, "一直整排压黑最终报 LINE_LOST（%s）",
          Mission_GetFaultReason());
    check(moved < CFG_LINE_CROSS_MAX_MM + CFG_LINE_LOST_MAX_MM + 60.0f, "停下前只走了 %.0f mm", moved);
}

/* 21. 避障回线前清灰度状态：绕行时不采灰度，回线第一帧不能沿用撞障前“在线上”的判断 */
static void case_rejoin_reset(void)
{
    printf("\n== 21 回线前清灰度状态\n");
    sim_reset_world();
    run_ms(1000);
    sim_gray("00011000");
    start_stage(1);                                   /* 从避障段起跑 */
    run_ms(800);
    g_obstacle = true;
    check(wait_state(ST_AVOID_OBS, 1000U), "看到障碍进入避障");
    g_obstacle = false;
    sim_gray("00000000");                             /* 绕行和回线一路都是白 */
    bool stale = false;
    for (uint32_t t = 0U; (t < 20000U) && (Mission_GetState() == ST_AVOID_OBS); ++t) {
        run_ms(1);
        if (g_gray.line_found && (g_gray.white_frames >= 1U)) { stale = true; }
    }
    check(!stale, "回线搜索的全白帧从不被当成有线");
    check(Mission_GetState() == ST_FAULT && strcmp(Mission_GetFaultReason(), "REJOIN_FAIL") == 0,
          "一直找不到线报 REJOIN_FAIL（%s）", Mission_GetFaultReason());
}

/* 22. 丢线误差平滑：线从中间消失时不一步跳到 ±3.8，每帧按一阶低通往边界靠，单调、不过冲 */
static void case_lost_smooth(void)
{
    printf("\n== 22 丢线误差平滑\n");
    static const uint16_t th[8] = CFG_GRAY_TH_DEFAULT;
    gray_state_t st;
    uint16_t raw[8];
    memset(&st, 0, sizeof st);
    memcpy(st.thresh, th, sizeof th);

    gray_pattern("00010000", true, false, raw);                  /* 线在中间偏左，误差 −0.5 */
    GrayLogic_Process(&st, raw, GRAY_BIAS_CENTER, true, false, 0U);
    float prev = st.error;
    gray_pattern("00000000", true, false, raw);
    float first_lost = 0.0f, max_step = 0.0f;
    bool mono = true, got_first = false;
    for (int i = 0; i < 40; ++i) {
        GrayLogic_Process(&st, raw, GRAY_BIAS_CENTER, true, false, 0U);
        if (!st.line_found && !got_first) { first_lost = st.error; got_first = true; }
        float step = prev - st.error;                            /* 向左找线，误差应越来越负 */
        if (step < -0.001f) { mono = false; }
        if (step > max_step) { max_step = step; }
        if (st.error < -3.8f - 0.001f) { mono = false; }
        prev = st.error;
    }
    check(got_first && first_lost > -1.2f, "确认丢线第一帧误差 %.2f，没有一步跳到 -3.8", first_lost);
    check(max_step <= CFG_GRAY_LOST_ERR_SMOOTH * 3.3f + 0.01f, "每帧最多变 %.2f", max_step);
    check(mono, "误差单调往左、不越过 -3.8");
    check(st.error < -3.7f, "约 40 帧后到边界附近（%.2f）", st.error);

    gray_pattern("00000100", true, false, raw);                  /* 重新看到线：立即用实际误差 */
    GrayLogic_Process(&st, raw, GRAY_BIAS_CENTER, true, false, 0U);
    check(st.line_found && fabsf(st.error - 1.5f) < 0.01f, "见到线误差立即为实测 1.50（得 %.2f）", st.error);
}

/* 23. 运动尖峰：死区补偿过零不硬切换；目标速度限加速度、停车立即生效；误差换来源那一拍不算微分 */
static void case_spikes(void)
{
    printf("\n== 23 运动尖峰\n");
    sim_reset_world();
    run_ms(200);
    BspMotor_SetDiffSpeed(10.0f, 10.0f);                  /* 内侧轮目标在 0 附近：+10 → −10 */
    run_ms(300);
    int p_pos = g_pwm[0];
    BspMotor_SetDiffSpeed(-10.0f, -10.0f);
    run_ms(20);
    int p_neg = g_pwm[0];
    check(abs(p_pos - p_neg) < 400, "目标 +10 → -10，PWM %d → %d，没有 ±%d 硬切换",
          p_pos, p_neg, CFG_WHEEL_PWM_DEADZONE);
    BspMotor_Stop();
    run_ms(300);

    sim_reset_world();
    run_ms(200);
    BspMotor_SetDiffSpeed(300.0f, 300.0f);                /* 从静止一步给到 300 */
    int prev = 0, first = 0, max_up = 0;
    for (int k = 0; k < 20; ++k) {
        run_ms(20);
        if (k == 0) { first = g_pwm[0]; }
        else if (g_pwm[0] - prev > max_up) { max_up = g_pwm[0] - prev; }
        prev = g_pwm[0];
    }
    check(first < 800, "起步第一拍 PWM %d，没有一步到满速对应的输出", first);
    check(max_up < 400, "之后每拍最多增 %d", max_up);
    check(BspMotor_GetState()->wheel_spd_mm_s[0] > 200.0f, "400 ms 内加到 %.0f mm/s",
          BspMotor_GetState()->wheel_spd_mm_s[0]);
    BspMotor_Stop();
    check(all_pwm_zero(), "BspMotor_Stop 立即发零 PWM，不受加速度限制");
    run_ms(300);
    BspMotor_SetDiffSpeed(300.0f, 300.0f);                /* 停车后再起步：要从 0 重新加速 */
    run_ms(20);
    check(g_pwm[0] < 800, "停车后再起步第一拍 PWM %d，不沿用停车前的速度", g_pwm[0]);
    BspMotor_Stop();
    run_ms(300);

    /* 丢线后找回：误差从找线值换成实测值，D 项不冒尖峰 */
    sim_reset_world();
    Motion_ResetLinePid();
    for (int i = 0; i < 5; ++i) { (void)line_step_diff("10000000"); }
    for (int i = 0; i < 30; ++i) { (void)line_step_diff("00000000"); }
    float d1 = line_step_diff("00000100");                /* 实测 +1.5 */
    float p1 = Motion_LinePTerm(1.5f);
    check(fabsf(d1 - p1) < 10.0f, "找回线那一拍差速 %.0f，约等于 P 项 %.0f", d1, p1);

    /* 路口保持结束：误差从保持值换成实测值，同样不冒尖峰 */
    Motion_ResetLinePid();
    for (int i = 0; i < 5; ++i) { (void)line_step_diff("00011000"); }
    for (int i = 0; i < 3; ++i) { (void)line_step_diff("11111111"); }
    float d2 = line_step_diff("00000011");                /* 实测 +3.0 */
    float p2 = Motion_LinePTerm(3.0f);
    check(fabsf(d2 - p2) < 10.0f, "路口保持结束那一拍差速 %.0f，约等于 P 项 %.0f", d2, p2);
    BspMotor_Stop();
}

/* 24. 审查问题复现：低速找线冻结、从无反馈、开环绕故障、斜入路口保持、整片黑回线 */
static void case_review_1125(void)
{
    printf("\n== 24 审查问题\n");

    /* P1-1 找库段线从中间消失，四轮目标 40 mm/s（低于 STALL_MIN_TGT），计数冻结也要报 ENC_STALL */
    sim_reset_world();
    run_ms(1000);
    sim_gray("00011000");
    start_stage(4);                                   /* 从找库段起跑 */
    run_ms(600);
    check(Mission_GetState() == ST_PARK_SEARCH, "正在找库（%s）", Mission_GetStateName());
    sim_gray("00000000");
    run_ms(200);
    float tgt0 = BspMotor_GetState()->target_spd_mm_s[0];
    check(fabsf(tgt0) < CFG_STALL_MIN_TGT_MM_S, "找线目标 %.0f mm/s，低于堵转门槛 %.0f", tgt0, CFG_STALL_MIN_TGT_MM_S);
    g_enc_frozen = true;
    uint32_t t0 = g_now;
    bool faulted = wait_state(ST_FAULT, 5000U);
    check(faulted && strcmp(Mission_GetFaultReason(), "ENC_STALL") == 0,
          "低速找线时冻结报 ENC_STALL（%s），用时 %lu ms", Mission_GetFaultReason(), (unsigned long)(g_now - t0));
    run_ms(300);
    check(all_pwm_zero(), "电机指令已归零");

    /* P1-2 上电起一帧反馈都没有，开环调试给 PWM：300 ms 后也要急停 */
    sim_reset_world();
    g_enc_silent = true;
    run_ms(100);
    BspMotor_SetRawPwm(750, 0, 0, 0);
    run_ms(1000);
    check(BspMotor_GetState()->rx_timeout_fault && all_pwm_zero(),
          "从没收到反馈也报超时并归零（fault=%d，M1 PWM %d）", (int)BspMotor_GetState()->rx_timeout_fault, g_pwm[0]);
    BspMotor_ClearFault();
    BspMotor_Stop();

    /* P1-3 故障已锁存时，开环指令不能发非零帧 */
    sim_reset_world();
    run_ms(200);
    BspMotor_SetDiffSpeed(100.0f, 100.0f);
    run_ms(300);
    g_enc_silent = true;                              /* 反馈中断 → 超时急停 */
    run_ms(600);
    check(BspMotor_HasFault(), "反馈中断后故障锁存");
    int frames0 = g_pwm_frames;
    BspMotor_SetRawPwm(750, 0, 0, 0);
    check(g_pwm[0] == 0 && g_pwm_frames > frames0, "故障时开环 750 只发出零帧（M1 PWM %d）", g_pwm[0]);
    BspMotor_ClearFault();
    BspMotor_Stop();

    /* P2-4 横线进来时先和主线连成 4 路，这一帧的误差被拉偏，不能被路口保持下去 */
    sim_reset_world();
    Motion_ResetLinePid();
    for (int i = 0; i < 5; ++i) { (void)line_step_diff("00011000"); }
    (void)line_step_diff("00011110");                 /* 4 路：横线正在进来 */
    float dc = 0.0f;
    for (int i = 0; i < 3; ++i) { dc = line_step_diff("00011111"); }
    check(fabsf(dc) < 0.5f * CFG_LINE_PID_KP, "主线居中斜入 T 字，路口保持差速 %.0f（未被支线拉偏）", dc);

    /* P2-4 报告原样：只有最右探头压线（+3.5）后整排压黑，保持值限幅，右轮不反转 */
    Motion_ResetLinePid();
    for (int i = 0; i < 3; ++i) { (void)line_step_diff("00000001"); }
    for (int i = 0; i < 4; ++i) { (void)line_step_diff("11111111"); }
    const motor_state_t *m = BspMotor_GetState();
    float l = 0.5f * (m->target_spd_mm_s[0] + m->target_spd_mm_s[1]);
    float r = 0.5f * (m->target_spd_mm_s[2] + m->target_spd_mm_s[3]);
    check(r > 0.0f && fabsf(g_gray.error) <= CFG_GRAY_CROSS_HOLD_MAX_ERR + 0.01f,
          "整排压黑时保持误差 %.2f，左/右目标 %.0f / %.0f，右轮不反转", g_gray.error, l, r);
    BspMotor_Stop();

    /* P2-5 避障回线时一直是整片黑：不能当成回到线上 */
    sim_reset_world();
    run_ms(1000);
    sim_gray("00011000");
    start_stage(1);
    run_ms(800);
    g_obstacle = true;
    check(wait_state(ST_AVOID_OBS, 1000U), "看到障碍进入避障");
    g_obstacle = false;
    sim_gray("11111111");
    bool passed = false;
    for (uint32_t t = 0U; (t < 20000U) && (Mission_GetState() == ST_AVOID_OBS); t += 20U) {
        run_ms(20);
    }
    passed = (Mission_GetState() == ST_FORK_PASS);
    check(!passed && strcmp(Mission_GetFaultReason(), "REJOIN_FAIL") == 0,
          "整片黑不算回线，报 REJOIN_FAIL（%s / %s）", Mission_GetStateName(), Mission_GetFaultReason());
}

/* 25. 二次审查：弯里内侧轮慢转不误报堵转；4 路宽线保持也受 80 mm 封顶 */
static void case_review2_1125(void)
{
    printf("\n== 25 二次审查\n");

    /* 基速 140、偏差 +1.5：右轮目标约 −5 mm/s，PWM 过死区后慢转，不是堵转 */
    sim_reset_world();
    run_ms(200);
    float spd = 140.0f * (1.0f - CFG_LINE_SLOW_PER_ERR * 1.5f);
    float dif = Motion_LinePTerm(1.5f);
    BspMotor_SetDiffSpeed(spd + dif, spd - dif);
    float r_tgt = spd - dif;
    run_ms(5000);
    check(!BspMotor_HasFault(), "内侧轮目标 %.1f mm/s、实测 %.1f mm/s，5 s 不报堵转（stall_wheel=%u）",
          r_tgt, BspMotor_GetState()->wheel_spd_mm_s[2], (unsigned)BspMotor_GetState()->stall_wheel);
    BspMotor_Stop();
    run_ms(300);

    /* 一直 4 路压黑（没到路口门槛 5 路，但误差在保持）：也要在封顶距离后转丢线、报 LINE_LOST */
    sim_reset_world();
    run_ms(1000);
    sim_gray("00011000");
    start_stage(0);
    run_ms(500);
    float d0 = BspMotor_GetState()->total_dist_mm;
    sim_gray("00111100");
    bool got = wait_state(ST_FAULT, 8000U);
    float moved = BspMotor_GetState()->total_dist_mm - d0;
    check(got && strcmp(Mission_GetFaultReason(), "LINE_LOST") == 0, "一直 4 路宽线最终报 LINE_LOST（%s）",
          Mission_GetFaultReason());
    check(moved < CFG_LINE_CROSS_MAX_MM + CFG_LINE_LOST_MAX_MM + 60.0f, "停下前只走了 %.0f mm", moved);
}

/* 26. 三次审查：回线不认两段分开的线；路口保持第一拍不带进入前的微分 */
static void case_review3_1125(void)
{
    printf("\n== 26 三次审查\n");

    /* 回线时一直看到两段分开的线（两路黑，不连成一段）：不能当成回到主线 */
    sim_reset_world();
    run_ms(1000);
    sim_gray("00011000");
    start_stage(1);
    run_ms(800);
    g_obstacle = true;
    check(wait_state(ST_AVOID_OBS, 1000U), "看到障碍进入避障");
    g_obstacle = false;
    sim_gray("00010001");
    for (uint32_t t = 0U; (t < 20000U) && (Mission_GetState() == ST_AVOID_OBS); t += 20U) {
        run_ms(20);
    }
    check(Mission_GetState() != ST_FORK_PASS && strcmp(Mission_GetFaultReason(), "REJOIN_FAIL") == 0,
          "两段分开的线不算回线（%s / %s）", Mission_GetStateName(), Mission_GetFaultReason());

    /* 居中 → 最右探头 → 整排黑：进入路口第一拍只有限幅后的 P，内侧轮不反转 */
    sim_reset_world();
    Motion_ResetLinePid();
    for (int i = 0; i < 5; ++i) { (void)line_step_diff("00011000"); }
    (void)line_step_diff("00000001");
    float d = line_step_diff("11111111");
    const motor_state_t *m = BspMotor_GetState();
    float r = 0.5f * (m->target_spd_mm_s[2] + m->target_spd_mm_s[3]);
    float p = Motion_LinePTerm(CFG_GRAY_CROSS_HOLD_MAX_ERR);
    check(fabsf(d - p) < 1.0f && r > 0.0f, "路口第一拍差速 %.1f（应只有 P %.1f），右轮目标 %.1f 不反转", d, p, r);

    /* 接着闪一帧全白（还没确认丢线）：沿用保持的限幅误差，不恢复进路口前的 +3.5 */
    float d3 = line_step_diff("00000000");
    float r3 = 0.5f * (m->target_spd_mm_s[2] + m->target_spd_mm_s[3]);
    check(fabsf(g_gray.error - CFG_GRAY_CROSS_HOLD_MAX_ERR) < 0.01f && r3 > 0.0f,
          "路口后闪白一帧，误差 %.2f、差速 %.1f，右轮目标 %.1f 不反转", g_gray.error, d3, r3);
    BspMotor_Stop();
}

/* 从某段起跑一路有线，返回进入目标状态时的总里程（参考跑，用来定位后面的场景） */
static float ref_enter_dist(int long_presses, mission_state_t st)
{
    sim_reset_world();
    run_ms(1000);
    sim_gray("00011000");
    start_stage(long_presses);
    if (!wait_state(st, 60000U)) { return -1.0f; }
    return BspMotor_GetState()->total_dist_mm;
}

/* 有线跑到总里程 d 处（超时返回 false） */
static bool run_until_dist(float d, uint32_t max_ms)
{
    for (uint32_t t = 0U; t < max_ms; t += 20U) {
        if (BspMotor_GetState()->total_dist_mm >= d) { return true; }
        run_ms(20);
    }
    return false;
}

/* 27. 四次审查：宽线夹单帧白不绕过封顶；跨赛段持续丢线不重算；丢线时不按 P 牌/里程入库 */
static void case_review4_1125(void)
{
    printf("\n== 27 四次审查\n");

    /* 9 帧四路黑 + 1 帧全白循环：没有重新看到窄线，封顶照样到、最终报 LINE_LOST */
    sim_reset_world();
    run_ms(1000);
    sim_gray("00011000");
    start_stage(0);
    run_ms(500);
    float d0 = BspMotor_GetState()->total_dist_mm;
    for (uint32_t t = 0U; (t < 8000U) && (Mission_GetState() != ST_FAULT); t += 200U) {
        sim_gray("00111100");
        run_ms(180);
        sim_gray("00000000");
        run_ms(20);
    }
    float moved = BspMotor_GetState()->total_dist_mm - d0;
    check(Mission_GetState() == ST_FAULT && strcmp(Mission_GetFaultReason(), "LINE_LOST") == 0,
          "宽线夹单帧白最终报 LINE_LOST（%s）", Mission_GetFaultReason());
    check(moved < CFG_LINE_CROSS_MAX_MM + CFG_LINE_LOST_MAX_MM + 60.0f, "停下前只走了 %.0f mm", moved);

    /* 色块段末尾开始全白，跨进找库段：丢线距离接着算，不从 0 重来 */
    float d_park = ref_enter_dist(3, ST_PARK_SEARCH);
    check(d_park > 0.0f, "参考跑：色块段 → 找库段在 %.0f mm", d_park);
    sim_reset_world();
    run_ms(1000);
    sim_gray("00011000");
    start_stage(3);
    run_until_dist(d_park - 150.0f, 60000U);
    check(Mission_GetState() == ST_COLOR_SCAN, "全白前仍在色块段（%s）", Mission_GetStateName());
    float dw = BspMotor_GetState()->total_dist_mm;
    sim_gray("00000000");
    bool crossed = false;
    for (uint32_t t = 0U; (t < 30000U) && (Mission_GetState() != ST_FAULT); t += 20U) {
        run_ms(20);
        if (Mission_GetState() == ST_PARK_SEARCH) { crossed = true; }
    }
    moved = BspMotor_GetState()->total_dist_mm - dw;
    check(crossed, "丢线途中切进了找库段");
    check(strcmp(Mission_GetFaultReason(), "LINE_LOST") == 0 && moved < CFG_LINE_LOST_MAX_MM + 40.0f,
          "跨段丢线按首次全白起算报 LINE_LOST，走了 %.0f mm（%s）", moved, Mission_GetFaultReason());

    /* 找库段：里程兜底点前 150 mm 开始全白，不给 P 牌：不能在丢线状态下入库 */
    float d_fb = ref_enter_dist(4, ST_PARK_REVERSE);
    check(d_fb > 0.0f, "参考跑：里程兜底入库在 %.0f mm", d_fb);
    sim_reset_world();
    run_ms(1000);
    sim_gray("00011000");
    start_stage(4);
    run_until_dist(d_fb - 150.0f, 60000U);
    sim_gray("00000000");
    bool parked = false;
    for (uint32_t t = 0U; (t < 30000U) && (Mission_GetState() != ST_FAULT); t += 20U) {
        run_ms(20);
        if (Mission_GetState() == ST_PARK_REVERSE) { parked = true; break; }
    }
    check(!parked && strcmp(Mission_GetFaultReason(), "LINE_LOST") == 0,
          "丢线时不按里程入库，最终报 LINE_LOST（%s / %s）", Mission_GetStateName(), Mission_GetFaultReason());

    /* 兜底点附近短暂丢线、随后找回：找回后照常入库 */
    sim_reset_world();
    run_ms(1000);
    sim_gray("00011000");
    start_stage(4);
    run_until_dist(d_fb - 20.0f, 60000U);
    sim_gray("00000000");
    parked = false;
    for (uint32_t t = 0U; t < 2000U; t += 20U) {
        run_ms(20);
        if (Mission_GetState() == ST_PARK_REVERSE) { parked = true; }
    }
    float past = BspMotor_GetState()->total_dist_mm - d_fb;
    check(!parked && past > 0.0f, "丢线中越过兜底点 %.0f mm 也不入库", past);
    sim_gray("00011000");
    check(wait_state(ST_PARK_REVERSE, 2000U), "找回线后照常入库（%s）", Mission_GetStateName());
}

/* 28. 急停后重跑：第一拍不沿用上一轮的灰度方向 */
static void case_restart_gray(void)
{
    printf("\n== 28 急停重跑清灰度\n");
    sim_reset_world();
    run_ms(1000);
    sim_gray("00000001");                             /* 线在最右，误差 +3.5 */
    start_stage(0);
    run_ms(300);
    g_estop = true;
    run_ms(40);
    g_estop = false;
    check(Mission_GetState() == ST_FAULT, "按键急停");
    key(KEY_EVT_LONG);
    check(Mission_GetState() == ST_IDLE, "回到待机");
    sim_gray("00000000");                             /* 放到全白处重跑 */
    g_key_ev = KEY_EVT_SHORT;
    g_key_press_ms = g_now;
    run_to_before_step();
    run_ms(1);                                        /* 起跑那一拍 */
    run_ms(20);                                       /* 第一拍循迹 */
    const motor_state_t *m = BspMotor_GetState();
    float l = 0.5f * (m->target_spd_mm_s[0] + m->target_spd_mm_s[1]);
    float r = 0.5f * (m->target_spd_mm_s[2] + m->target_spd_mm_s[3]);
    check(Mission_GetState() == ST_TRACK_TO_OBS, "已重新起跑（%s）", Mission_GetStateName());
    check(!g_gray.line_found && fabsf(l - r) < 1.0f,
          "全白第一拍不算有线、不转向（误差 %.2f，左/右 %.1f / %.1f）", g_gray.error, l, r);
    g_estop = true;
    run_ms(40);
    g_estop = false;
}

int main(void)
{
    case_cam_restart();
    case_color_frames();
    case_frozen_in_drive();
    case_frozen_in_track();
    case_motion_timeout();
    case_fork_rerun();
    case_park_flip();
    case_park_happy();
    case_bad_frames();
    case_fork_steer();
    case_gray_config();
    case_gray_dead();
    case_line_deriv();
    case_wheel_brake();
    case_corner();
    case_sonar();
    case_lost_spin();
    case_cross();
    case_line_gain();
    case_gray_hyst();
    case_cross_cap();
    case_rejoin_reset();
    case_lost_smooth();
    case_spikes();
    case_review_1125();
    case_review2_1125();
    case_review3_1125();
    case_review4_1125();
    case_restart_gray();
    printf("\n%d/%d passed\n", g_checks - g_fails, g_checks);
    return g_fails ? 1 : 0;
}

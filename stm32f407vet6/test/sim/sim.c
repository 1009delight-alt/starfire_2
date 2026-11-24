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
    g_gray.error = 0.0f;
}
const gray_state_t *BspGray_GetState(void) { return &g_gray; }
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
    if ((g_now % 10U) == 0U) {
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
    g_enc_frozen = false; g_load = 1.0f; g_line = true; g_obstacle = false; g_sonar_ok = true;
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
    float p0 = CFG_LINE_PID_KP * BspGray_GetState()->error;
    check(fabsf(d0 - p0) < 1.0f, "复位后第一拍只有 P 项（差速 %.1f，P %.1f）", d0, p0);

    float worst = 0.0f;
    for (int i = 0; i < 40; ++i) {                        /* 0 与 +0.5 每拍交替 */
        float d = line_step_diff((i & 1) ? "00011000" : "00001000");
        float dterm = fabsf(d - CFG_LINE_PID_KP * BspGray_GetState()->error);
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
    for (int i = 0; i < 5; ++i) { lost = line_step_diff("00000000"); }
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
    run_to_before_step();
    run_ms(1);
    check(g_pwm[0] < 0, "目标转反向后的第一拍左轮 PWM 为反向（%d）", g_pwm[0]);
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
    printf("\n%d/%d passed\n", g_checks - g_fails, g_checks);
    return g_fails ? 1 : 0;
}

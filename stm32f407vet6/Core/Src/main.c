#include "board.h"
#include "config.h"
#include "bsp_uart.h"
#include "bsp_motor.h"
#include "bsp_gray.h"
#include "bsp_sonar.h"
#include "bsp_openmv.h"
#include "bsp_oled.h"
#include "motion_ctrl.h"
#include "mission_fsm.h"
#include "ui_pages.h"

#include <stdio.h>

/* ============================================================================
 *  第十四届“星火杯”赛题二 · 分步调试窗口 & 比赛主入口 (main.c)
 *
 *  修改下面这一行 APP_RUN_MODE 即可切换调试/比赛模式，全部参数集中在 Core/Inc/config.h：
 *    RUN_MODE_RACE            : 完整比赛程序（一键启动跑完 5 项任务）
 *    RUN_MODE_DBG_MOTOR_ENC   : 架空电机与编码器方向/单圈计数标定
 *    RUN_MODE_DBG_MOTION_CAL  : 地面 1000mm 直线 & 原地 90° 转向标定
 *    RUN_MODE_DBG_GRAY        : 8 路灰度原始 ADC / 二值化 / 一键阈值校准
 *    RUN_MODE_DBG_SONAR       : 前向超声 HC-SR04 测距与避障触发测试
 *    RUN_MODE_DBG_OPENMV      : OpenMV 协议联调（按 KEY 轮切 IDLE/TURN/COLOR/PARK）
 *    RUN_MODE_DBG_LINE_ONLY   : 纯循迹 PID 调试（按 KEY 启停，长按切中/左/右偏置）
 *    RUN_MODE_DBG_AVOID_ONLY  : 单项避障绕行与回线测试
 *    RUN_MODE_DBG_PARK_ONLY   : 单项寻 P 牌与倒车入库停 5 秒测试
 * ============================================================================ */
#define RUN_MODE_RACE               0U
#define RUN_MODE_DBG_MOTOR_ENC      1U
#define RUN_MODE_DBG_MOTION_CAL     2U
#define RUN_MODE_DBG_GRAY           3U
#define RUN_MODE_DBG_SONAR          4U
#define RUN_MODE_DBG_OPENMV         5U
#define RUN_MODE_DBG_LINE_ONLY      6U
#define RUN_MODE_DBG_AVOID_ONLY     7U
#define RUN_MODE_DBG_PARK_ONLY      8U

#define APP_RUN_MODE               RUN_MODE_DBG_LINE_ONLY
/* 调试辅助变量（Keil / VS Code Watch 窗口可直接观测） */
volatile uint32_t g_main_loop_ticks = 0U;

static void system_bsp_init(void)
{
    Board_Init();

#if (CFG_ENABLE_DEBUG_UART != 0U) || (APP_RUN_MODE == RUN_MODE_DBG_LINE_ONLY)
    /* 纯循迹模式固定输出 USART6 日志，不受总开关影响（原来总开关为 0 时日志发不出去） */
    BspUart_Init(BSP_UART_DEBUG, 115200U);
#endif
    BspMotor_Init();
    BspOled_Init();
    BspGray_Init();
    (void)BspSonar_Init();
    BspOpenmv_Init();
    Motion_Init();

#if (CFG_ENABLE_IWDG != 0U)
    /* 所有阻塞初始化完成后再启动看门狗；主循环每圈喂狗 */
    Board_WatchdogStart(CFG_IWDG_RELOAD);
#endif
}

static void system_poll_fast(uint32_t now_ms)
{
#if (CFG_ENABLE_IWDG != 0U)
    Board_WatchdogKick();
#endif
    BspMotor_PollRx(now_ms);
    BspOpenmv_Poll(now_ms);
    BspSonar_Update(now_ms);
    Board_KeyUpdate(now_ms);
    Board_LedUpdate(now_ms);
}

/* ==================== 各分步调试模式实现 ==================== */

#if (APP_RUN_MODE == RUN_MODE_DBG_MOTOR_ENC)
/* 架空测试：短按 KEY 轮流给四轮低速 PWM 1.5s（M1->M2->M3->M4->全轮），长按清零编码器 */
static void run_dbg_motor_enc(void)
{
    uint8_t step = 0U;
    uint8_t running = 0U;
    uint32_t run_t0 = 0U, last_ctrl = 0U, last_oled = 0U;
    Board_LedSetPattern(LED_PAT_SLOW_BLINK);

    while (1) {
        uint32_t now = HAL_GetTick();
        system_poll_fast(now);
        (void)BspOled_FlushStep();   /* 每圈最多发一条 8 行条带（约 2 ms），不挤占控制周期 */

        board_key_event_t ev = Board_KeyTakeEvent();
        if (ev == KEY_EVT_LONG) {
            BspMotor_Stop();
            running = 0U;
            BspMotor_ResetTotalOdom();
        } else if (ev == KEY_EVT_SHORT) {
            if (running != 0U) {
                BspMotor_Stop();
                running = 0U;
            } else {
                const int16_t p = 750;
                running = 1U;
                run_t0 = now;
                if      (step == 0U) { BspMotor_SetRawPwm(p, 0, 0, 0); }
                else if (step == 1U) { BspMotor_SetRawPwm(0, p, 0, 0); }
                else if (step == 2U) { BspMotor_SetRawPwm(0, 0, p, 0); }
                else if (step == 3U) { BspMotor_SetRawPwm(0, 0, 0, p); }
                else                 { BspMotor_SetRawPwm(p, p, p, p); }
                step = (uint8_t)((step + 1U) % 5U);
            }
        }

        if ((running != 0U) && ((uint32_t)(now - run_t0) >= 1500U)) {
            BspMotor_Stop();
            running = 0U;
        }

        if ((uint32_t)(now - last_ctrl) >= CFG_MAIN_CTRL_PERIOD_MS) {
            float dt = (float)(now - last_ctrl) * 0.001f;
            last_ctrl = now;
            BspMotor_ControlStep(now, dt);
        }

        if ((uint32_t)(now - last_oled) >= CFG_OLED_REFRESH_PERIOD_MS) {
            last_oled = now;
            UiPage_MotorEnc(BspMotor_GetState(), step);
        }
    }
}
#endif

#if (APP_RUN_MODE == RUN_MODE_DBG_MOTION_CAL)
/* 地面里程/转角标定：短按 KEY 交替执行 [1000mm 直线] / [原地左转 90°]，运行中再按急停 */
static void run_dbg_motion_cal(void)
{
    uint8_t task = 0U;      /* 0=直行1000mm, 1=左转90deg */
    uint8_t active = 0U;
    uint32_t last_ctrl = 0U, last_oled = 0U;
    Board_LedSetPattern(LED_PAT_SLOW_BLINK);

    while (1) {
        uint32_t now = HAL_GetTick();
        system_poll_fast(now);
        (void)BspOled_FlushStep();   /* 每圈最多发一条 8 行条带（约 2 ms），不挤占控制周期 */

        board_key_event_t ev = Board_KeyTakeEvent();
        if (ev == KEY_EVT_SHORT) {
            if (active != 0U) {
                BspMotor_Stop();
                active = 0U;
                Board_LedSetPattern(LED_PAT_SLOW_BLINK);
            } else {
                BspMotor_ClearFault();
                Motion_ClearActionTimeout();
                active = 1U;
                Board_LedSetPattern(LED_PAT_FAST_BLINK);
                if (task == 0U) {
                    Motion_StartDriveDist(1000.0f, CFG_SPEED_CRUISE_MM_S);
                } else {
                    Motion_StartTurnDeg(90.0f, CFG_SPEED_TURN_INPLACE_MM_S);
                }
            }
        } else if ((ev == KEY_EVT_LONG) && (active == 0U)) {
            task ^= 1U;
        }

        if ((uint32_t)(now - last_ctrl) >= CFG_MAIN_CTRL_PERIOD_MS) {
            float dt = (float)(now - last_ctrl) * 0.001f;
            last_ctrl = now;
            if (active != 0U) {
                bool done = (task == 0U) ? Motion_UpdateDriveDist() : Motion_UpdateTurnDeg();
                if (done || BspMotor_HasFault() || Motion_ActionTimedOut()) {
                    BspMotor_Stop();
                    active = 0U;
                    Board_LedSetPattern(LED_PAT_DOUBLE_BLINK);
                }
            }
            BspMotor_ControlStep(now, dt);
        }

        if ((uint32_t)(now - last_oled) >= CFG_OLED_REFRESH_PERIOD_MS) {
            last_oled = now;
            UiPage_MotionCal(BspMotor_GetState(), active != 0U, task);
        }
    }
}
#endif

#if (APP_RUN_MODE == RUN_MODE_DBG_GRAY)
/* 8 路灰度调试：实时显示 8 路二值状态与 ADC 值；长按 KEY 开始/结束黑白扫描自动计算阈值 */
static void run_dbg_gray(void)
{
    uint8_t calib = 0U;
    uint32_t last_ctrl = 0U, last_oled = 0U;
    Board_LedSetPattern(LED_PAT_SLOW_BLINK);

    while (1) {
        uint32_t now = HAL_GetTick();
        system_poll_fast(now);
        (void)BspOled_FlushStep();   /* 每圈最多发一条 8 行条带（约 2 ms），不挤占控制周期 */

        if (Board_KeyTakeEvent() == KEY_EVT_LONG) {
            if (calib == 0U) {
                BspGray_CalibStart();
                calib = 1U;
                Board_LedSetPattern(LED_PAT_FAST_BLINK);
            } else {
                BspGray_CalibFinishAndApply();
                calib = 0U;
                Board_LedSetPattern(LED_PAT_DOUBLE_BLINK);
            }
        }

        if ((uint32_t)(now - last_ctrl) >= CFG_MAIN_CTRL_PERIOD_MS) {
            last_ctrl = now;
            BspGray_Sample(GRAY_BIAS_CENTER);
            if (calib != 0U) {
                BspGray_CalibFeed();
            }
            BspMotor_ControlStep(now, 0.02f);
        }

        if ((uint32_t)(now - last_oled) >= CFG_OLED_REFRESH_PERIOD_MS) {
            last_oled = now;
            UiPage_Gray(BspGray_GetState(), calib != 0U);
        }
    }
}
#endif

#if (APP_RUN_MODE == RUN_MODE_DBG_SONAR)
/* 前向超声 HC-SR04 测距调试 */
static void run_dbg_sonar(void)
{
    uint32_t last_ctrl = 0U, last_oled = 0U;
    Board_LedSetPattern(LED_PAT_SLOW_BLINK);

    while (1) {
        uint32_t now = HAL_GetTick();
        system_poll_fast(now);
        (void)BspOled_FlushStep();   /* 每圈最多发一条 8 行条带（约 2 ms），不挤占控制周期 */

        if ((uint32_t)(now - last_ctrl) >= CFG_MAIN_CTRL_PERIOD_MS) {
            last_ctrl = now;
            BspMotor_ControlStep(now, 0.02f);
        }

        if ((uint32_t)(now - last_oled) >= CFG_OLED_REFRESH_PERIOD_MS) {
            last_oled = now;
            UiPage_Sonar(BspSonar_GetState(), BspSonar_IsHealthy(now), BspSonar_IsObstacleDetected(now));
        }
    }
}
#endif

#if (APP_RUN_MODE == RUN_MODE_DBG_OPENMV)
/* OpenMV 联调：短按 KEY 轮切 IDLE -> TURN -> COLOR -> PARK，长按复位 */
static void run_dbg_openmv(void)
{
    openmv_task_t task = OPENMV_TASK_TURN;
    uint32_t last_ctrl = 0U, last_oled = 0U;
    BspOpenmv_SetTask(task);
    Board_LedSetPattern(LED_PAT_SLOW_BLINK);

    while (1) {
        uint32_t now = HAL_GetTick();
        system_poll_fast(now);
        (void)BspOled_FlushStep();   /* 每圈最多发一条 8 行条带（约 2 ms），不挤占控制周期 */

        board_key_event_t ev = Board_KeyTakeEvent();
        if (ev == KEY_EVT_LONG) {
            BspOpenmv_ResetAll();
            task = OPENMV_TASK_IDLE;
        } else if (ev == KEY_EVT_SHORT) {
            task = (openmv_task_t)(((uint8_t)task + 1U) % 4U);
            BspOpenmv_SetTask(task);
        }

        if ((uint32_t)(now - last_ctrl) >= CFG_MAIN_CTRL_PERIOD_MS) {
            last_ctrl = now;
            BspMotor_ControlStep(now, 0.02f);
        }

        if ((uint32_t)(now - last_oled) >= CFG_OLED_REFRESH_PERIOD_MS) {
            last_oled = now;
            const openmv_state_t *mv = BspOpenmv_GetState();
            UiPage_Openmv(mv, BspOpenmv_TaskName(mv->target_task), BspOpenmv_TaskName(mv->ack_task));
        }
    }
}
#endif

#if (APP_RUN_MODE == RUN_MODE_DBG_LINE_ONLY)
/* 纯循迹 PID 调试：短按 KEY 启停循迹，长按切换 CENTER / LEFT / RIGHT 偏置 */
static void run_dbg_line_only(void)
{
    uint8_t running = 0U;
    gray_bias_mode_t bias = GRAY_BIAS_CENTER;
    uint32_t last_ctrl = 0U, last_oled = 0U;
    Board_LedSetPattern(LED_PAT_SLOW_BLINK);

    while (1) {
        uint32_t now = HAL_GetTick();
        system_poll_fast(now);
        (void)BspOled_FlushStep();   /* 每圈最多发一条 8 行条带（约 2 ms），不挤占控制周期 */

        board_key_event_t ev = Board_KeyTakeEvent();
        if (ev == KEY_EVT_SHORT) {
            if (running != 0U) {
                running = 0U;
                BspMotor_Stop();
                Board_LedSetPattern(LED_PAT_SLOW_BLINK);
            } else {
                BspMotor_ClearFault();
                Motion_ResetLinePid();
                running = 1U;
                Board_LedSetPattern(LED_PAT_FAST_BLINK);
            }
        } else if ((ev == KEY_EVT_LONG) && (running == 0U)) {
            bias = (gray_bias_mode_t)(((uint8_t)bias + 1U) % 3U);
        }

        if ((uint32_t)(now - last_ctrl) >= CFG_MAIN_CTRL_PERIOD_MS) {
            float dt = (float)(now - last_ctrl) * 0.001f;
            last_ctrl = now;
            if (running != 0U) {
                if (BspMotor_HasFault()) {
                    running = 0U;
                    BspMotor_Stop();
                    Board_LedSetPattern(LED_PAT_FAULT_BLINK);
                } else {
                    /* 2026-11-22 临时：调试先用一半巡航速度，循迹稳了再改回 CFG_SPEED_CRUISE_MM_S */
                    Motion_LineFollowStep(CFG_SPEED_CRUISE_MM_S * 0.5f, bias, dt);
                }
            } else {
                BspGray_Sample(bias);
            }
            BspMotor_ControlStep(now, dt);
            if (running != 0U) {
                /* USART6 日志（PC6，115200），每 20 ms 一行：
                 * 毫秒, 误差×100, 左目标, 右目标, M1..M4 实测 (mm/s)。阻塞发送约 4 ms，只在本调试模式用 */
                const motor_state_t *ms = BspMotor_GetState();
                BspUart_Printf(BSP_UART_DEBUG, "%lu,%d,%d,%d,%d,%d,%d,%d\r\n", (unsigned long)now,
                               (int)(BspGray_GetState()->error * 100.0f),
                               (int)ms->target_spd_mm_s[0], (int)ms->target_spd_mm_s[2],
                               (int)ms->wheel_spd_mm_s[0], (int)ms->wheel_spd_mm_s[1],
                               (int)ms->wheel_spd_mm_s[2], (int)ms->wheel_spd_mm_s[3]);
            }
        }

        if ((uint32_t)(now - last_oled) >= CFG_OLED_REFRESH_PERIOD_MS) {
            last_oled = now;
            const char *bname = (bias == GRAY_BIAS_LEFT) ? "LEFT" :
                                ((bias == GRAY_BIAS_RIGHT) ? "RIGHT" : "CTR");
            UiPage_Line(BspGray_GetState(), BspMotor_GetState(), running != 0U, bname);
        }
    }
}
#endif

/* ==================== 正式比赛 / 单项全流程入口 ==================== */
#if (APP_RUN_MODE == RUN_MODE_RACE) || \
    (APP_RUN_MODE == RUN_MODE_DBG_AVOID_ONLY) || \
    (APP_RUN_MODE == RUN_MODE_DBG_PARK_ONLY)
static void run_mission(mission_profile_t profile)
{
    uint32_t last_ctrl = HAL_GetTick();
    uint32_t last_oled = 0U;

    Mission_Init(profile);

    while (1) {
        uint32_t now = HAL_GetTick();
        system_poll_fast(now);

        if ((uint32_t)(now - last_ctrl) >= CFG_MAIN_CTRL_PERIOD_MS) {
            float dt = (float)(now - last_ctrl) * 0.001f;
            last_ctrl = now;
            g_main_loop_ticks++;

            Mission_Step20ms(now, dt);
            BspMotor_ControlStep(now, dt);
            /* 屏幕每个控制周期最多发一条有变化的 8 行条带，阻塞约 2 ms */
            (void)BspOled_FlushStep();
        }

        if ((uint32_t)(now - last_oled) >= CFG_OLED_REFRESH_PERIOD_MS) {
            last_oled = now;
            const motor_state_t  *m  = BspMotor_GetState();
            const openmv_state_t *mv = BspOpenmv_GetState();
            const sonar_state_t  *us = BspSonar_GetState();

            BspOled_ShowRaceScreen(Mission_GetStateName(),
                                   m->total_dist_mm,
                                   mv->color_seq,
                                   mv->color_locked,
                                   Mission_GetForkDisplay(),
                                   BspSonar_IsHealthy(now) ? us->dist_mm : 0U,
                                   m->enc_online,
                                   mv->online);
        }
    }
}
#endif

int main(void)
{
    system_bsp_init();

#if   (APP_RUN_MODE == RUN_MODE_RACE)
    run_mission(MISSION_PROFILE_FULL);
#elif (APP_RUN_MODE == RUN_MODE_DBG_AVOID_ONLY)
    run_mission(MISSION_PROFILE_AVOID_ONLY);
#elif (APP_RUN_MODE == RUN_MODE_DBG_PARK_ONLY)
    run_mission(MISSION_PROFILE_PARK_ONLY);
#elif (APP_RUN_MODE == RUN_MODE_DBG_MOTOR_ENC)
    run_dbg_motor_enc();
#elif (APP_RUN_MODE == RUN_MODE_DBG_MOTION_CAL)
    run_dbg_motion_cal();
#elif (APP_RUN_MODE == RUN_MODE_DBG_GRAY)
    run_dbg_gray();
#elif (APP_RUN_MODE == RUN_MODE_DBG_SONAR)
    run_dbg_sonar();
#elif (APP_RUN_MODE == RUN_MODE_DBG_OPENMV)
    run_dbg_openmv();
#elif (APP_RUN_MODE == RUN_MODE_DBG_LINE_ONLY)
    run_dbg_line_only();
#else
#error "Invalid APP_RUN_MODE selected in main.c"
#endif

    return 0;
}

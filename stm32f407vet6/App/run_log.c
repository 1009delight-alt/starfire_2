#include "run_log.h"
#include "bsp_gray.h"
#include "bsp_motor.h"
#include "bsp_sonar.h"
#include "motion_ctrl.h"

typedef struct {
    uint32_t ms;            /* HAL_GetTick */
    int16_t  err100;        /* 灰度误差 × 100 */
    int16_t  tl, tr;        /* 左右轮组目标 (mm/s，左前/右前) */
    int16_t  vl, vr;        /* 左右轮组实测 (mm/s，前后两轮平均) */
    uint16_t sonar_mm;      /* 超声中值距离 (mm)，失效记 0 */
    uint8_t  bits;          /* 8 路判黑，bit0 = 屏上最左 */
    uint8_t  flags;         /* bit0 有线 bit1 分段 bit2 误差保持 bit3 丢线找线 bit4 路口确认 */
    uint8_t  state;         /* 比赛状态 mission_state_t；纯循迹模式记 0xFF */
    uint8_t  step;          /* 避障子步骤；其余记 0 */
    uint16_t seg_mm;        /* 当前动作段里程 (mm，取绝对值) */
} run_log_rec_t;            /* 24 字节 */

typedef struct {
    uint32_t      magic;
    uint32_t      head;     /* 下一帧写入位置 */
    uint32_t      count;    /* 已记帧数（封顶 RUN_LOG_N） */
    uint32_t      stop;     /* 停车原因，见 run_log.h */
    uint32_t      mode;     /* 1 纯循迹 2 比赛状态机；0 = 本次上电后还没跑过 */
    uint32_t      active;   /* 1 正在记录 */
    run_log_rec_t e[RUN_LOG_N];
} run_log_t;

/* 固定放在 CCM RAM 开头：工程里没有任何段放在 CCM，启动代码不清零它，ST 内置引导程序也不用它。
 * CCM 只有 CPU 能访问，这里也只由 CPU 读写 */
#define s_log  (*(volatile run_log_t *)RUN_LOG_ADDR)

void RunLog_Init(void)
{
    if ((s_log.magic != RUN_LOG_MAGIC) || (s_log.head >= RUN_LOG_N) || (s_log.count > RUN_LOG_N)) {
        s_log.head = 0U;
        s_log.count = 0U;
        s_log.stop = RUN_LOG_STOP_NONE;
        s_log.mode = 0U;
        s_log.magic = RUN_LOG_MAGIC;
    }
    s_log.active = 0U;       /* 复位后不接着记，保住复位前的记录 */
}

void RunLog_Start(uint8_t mode)
{
    s_log.head = 0U;
    s_log.count = 0U;
    s_log.stop = RUN_LOG_STOP_NONE;
    s_log.mode = mode;
    s_log.active = 1U;
}

bool RunLog_Active(void)
{
    return s_log.active != 0U;
}

void RunLog_Put(uint32_t now_ms, uint8_t state, uint8_t step)
{
    if (s_log.active == 0U) {
        return;
    }
    const gray_state_t  *g  = BspGray_GetState();
    const motor_state_t *ms = BspMotor_GetState();
    const sonar_state_t *us = BspSonar_GetState();
    volatile run_log_rec_t *e = &s_log.e[s_log.head];
    float seg = ms->seg_dist_mm;
    if (seg < 0.0f) { seg = -seg; }
    if (seg > 65535.0f) { seg = 65535.0f; }
    e->ms = now_ms;
    e->err100 = (int16_t)(g->error * 100.0f);
    e->tl = (int16_t)ms->target_spd_mm_s[0];
    e->tr = (int16_t)ms->target_spd_mm_s[2];
    e->vl = (int16_t)(0.5f * (ms->wheel_spd_mm_s[0] + ms->wheel_spd_mm_s[1]));
    e->vr = (int16_t)(0.5f * (ms->wheel_spd_mm_s[2] + ms->wheel_spd_mm_s[3]));
    e->sonar_mm = BspSonar_IsHealthy(now_ms) ? us->dist_mm : 0U;
    e->bits = g->bits;
    e->flags = (uint8_t)((g->line_found ? 0x01U : 0U) | (g->is_split ? 0x02U : 0U) | (g->held ? 0x04U : 0U) |
                         (Motion_IsLineLost() ? 0x08U : 0U) | (g->is_cross ? 0x10U : 0U));
    e->state = state;
    e->step = step;
    e->seg_mm = (uint16_t)seg;
    s_log.head = (s_log.head + 1U) % RUN_LOG_N;
    if (s_log.count < RUN_LOG_N) { s_log.count++; }
}

void RunLog_Stop(uint32_t reason)
{
    if (s_log.active != 0U) {
        s_log.stop = reason;
        s_log.active = 0U;
    }
}

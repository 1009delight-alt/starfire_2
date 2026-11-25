#ifndef MISSION_FSM_H
#define MISSION_FSM_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MISSION_PROFILE_FULL = 0,   /* 完整比赛：循迹 -> 避障 -> 分岔 -> 四色 -> 倒车入库 */
    MISSION_PROFILE_AVOID_ONLY, /* 单项调试：循迹 -> 避障回线 -> 停车 */
    MISSION_PROFILE_PARK_ONLY   /* 单项调试：循迹寻 P 牌 -> 倒车入库停 5 秒 */
} mission_profile_t;

typedef enum {
    ST_IDLE = 0,
    ST_TRACK_TO_OBS,
    ST_AVOID_OBS,
    ST_FORK_PASS,
    ST_COLOR_SCAN,
    ST_PARK_SEARCH,
    ST_PARK_REVERSE,
    ST_DONE,
    ST_FAULT
} mission_state_t;

/* 运行时选段起跑（IDLE 下长按循环选择，短按起跑） */
typedef enum {
    START_FULL = 0,     /* 全程：清空方向与四色 */
    START_AVOID,        /* 避障前：保留已锁存结果 */
    START_FORK,         /* 分岔前 */
    START_COLOR,        /* 色区前：重做四色 */
    START_PARK,         /* 第 4 弯后找库：保留四色 */
    START_COUNT
} mission_start_t;

void Mission_Init(mission_profile_t profile);
void Mission_Step20ms(uint32_t now_ms, float dt_s);
void Mission_RequestEmergencyStop(const char *reason);

mission_state_t Mission_GetState(void);
const char *Mission_GetStateName(void);
const char *Mission_GetFaultReason(void);
uint8_t Mission_GetSubStep(void);        /* 避障子步骤（AVOID_STEP_*）或入库子步骤，其余赛段 0；车上记录用 */
char Mission_GetForkDisplay(void);          /* 'L'/'R'；兜底猜测为 'l'/'r'；未知为 0 */
mission_start_t Mission_GetStartStage(void);

#endif /* MISSION_FSM_H */

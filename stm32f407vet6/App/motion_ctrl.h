#ifndef MOTION_CTRL_H
#define MOTION_CTRL_H

#include "bsp_gray.h"
#include <stdbool.h>
#include <stdint.h>

void Motion_Init(void);
void Motion_ResetLinePid(void);

/* 当前连续丢线已行驶的距离 (mm)，找到线后归零 */
float Motion_GetLineLostMm(void);
float Motion_GetLineLostDeg(void);     /* 本次连续丢线已转过的角度（绝对值） */

/* 循迹 PID 单步执行（每 20ms 调用一次） */
void Motion_LineFollowStep(float base_speed_mm_s, gray_bias_mode_t bias_mode, float dt_s);

/* 编码器闭环定距直行（dist_mm > 0 前进, < 0 倒车） */
void Motion_StartDriveDist(float dist_mm, float speed_mm_s);
bool Motion_UpdateDriveDist(void);  /* 到达目标距离并停车后返回 true */

/* 编码器闭环原地定角转向（angle_deg > 0 左转, < 0 右转） */
void Motion_StartTurnDeg(float angle_deg, float turn_speed_mm_s);
bool Motion_UpdateTurnDeg(void);    /* 到达目标角度并停车后返回 true */

/* 最近一次定距/定角动作是否超时（超时后 Update 不再推进，已停车） */
bool Motion_ActionTimedOut(void);
void Motion_ClearActionTimeout(void);

void Motion_Stop(void);

#endif /* MOTION_CTRL_H */

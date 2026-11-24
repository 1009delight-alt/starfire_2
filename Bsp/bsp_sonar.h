#ifndef BSP_SONAR_H
#define BSP_SONAR_H

/* 前向超声测距 HC-SR04：PB6 = Trig（推挽输出），PB7 = Echo（TIM4_CH2 双边沿输入捕获，1 MHz 计数）。
 * 每 CFG_SONAR_PERIOD_MS 触发一次；回波宽度在中断里测，主循环 BspSonar_Update 取结果，不阻塞。 */

#include <stdbool.h>
#include <stdint.h>

#include "sonar_logic.h"

extern sonar_state_t g_sonar;                     /* 只读观测用，代码里请用 BspSonar_GetState() */

bool BspSonar_Init(void);
void BspSonar_Update(uint32_t now_ms);
void BspSonar_IRQHandler(void);                    /* 在 TIM4_IRQHandler 中调用 */
void BspSonar_ResetTrigger(void);
bool BspSonar_IsHealthy(uint32_t now_ms);          /* 最近 CFG_SONAR_STALE_MS 内没收到回波为 false：不能当作前方畅通 */
bool BspSonar_IsObstacleDetected(uint32_t now_ms);
const sonar_state_t *BspSonar_GetState(void);

#endif /* BSP_SONAR_H */

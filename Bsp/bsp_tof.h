#ifndef BSP_TOF_H
#define BSP_TOF_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool     online;            /* VL53L0X 是否探测并初始化成功 */
    bool     valid;             /* 最近读数是否有效（< 2000mm） */
    uint16_t raw_mm;            /* 最新单次测距 (mm) */
    uint16_t dist_mm;           /* 3 点中值滤波后距离 (mm) */
    uint8_t  near_hits;         /* 连续 <= CFG_OBS_TRIG_DIST_MM 的帧数 */
    uint32_t last_update_ms;    /* 最近更新时间戳 */
} tof_state_t;

bool BspTof_Init(void);
void BspTof_Update(uint32_t now_ms);
void BspTof_ResetTrigger(void);
bool BspTof_IsHealthy(uint32_t now_ms);          /* 失效或过期时为 false：不能当作前方畅通 */
bool BspTof_IsObstacleDetected(uint32_t now_ms);
const tof_state_t *BspTof_GetState(void);

#endif /* BSP_TOF_H */

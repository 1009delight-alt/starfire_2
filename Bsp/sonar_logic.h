#ifndef SONAR_LOGIC_H
#define SONAR_LOGIC_H

/* HC-SR04 读数处理（纯函数，不碰硬件）：回波脉宽 -> 距离、3 点中值、近距连续计数。
 * 真机 bsp_sonar.c 与电脑回放 test/sim/sim.c 共用；改判定逻辑只改这里。 */

#include <stdbool.h>
#include <stdint.h>

#define SONAR_FAR_MM    2000U   /* 超量程 / 无回波目标时记的距离 */

typedef struct {
    bool     online;            /* 上电后至少收到过一次回波脉冲（模块无 ID，只能这样判断接没接） */
    bool     valid;             /* 中值距离在量程内（< SONAR_FAR_MM 且 <= CFG_SONAR_MAX_MM） */
    uint16_t raw_mm;            /* 最新单次测距 (mm)，超量程记 SONAR_FAR_MM */
    uint16_t dist_mm;           /* 3 点中值滤波后距离 (mm) */
    uint16_t echo_us;           /* 最新回波高电平宽度 (us) */
    uint8_t  near_hits;         /* 中值连续 <= 触发距离的次数 */
    uint16_t miss_count;        /* 触发后没等到回波、或回波一直不落的累计次数（只用于显示） */
    uint32_t last_update_ms;    /* 最近一次收到完整回波的时刻 */
    uint16_t med_buf[3];
    uint8_t  med_idx;
} sonar_state_t;

void SonarLogic_Reset(sonar_state_t *s);

/* 收到一次完整回波：echo_us = 高电平宽度。mm_per_us = 声速/2，max_mm = 量程上限，trig_mm = 避障触发距离 */
void SonarLogic_FeedEcho(sonar_state_t *s, uint32_t echo_us, uint32_t now_ms,
                         float mm_per_us, uint16_t max_mm, uint16_t trig_mm);

#endif /* SONAR_LOGIC_H */

#ifndef BSP_GRAY_H
#define BSP_GRAY_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    GRAY_BIAS_CENTER = 0,   /* 常规质心寻线 */
    GRAY_BIAS_LEFT,         /* 左分岔优先：跟踪最左侧连续黑线 */
    GRAY_BIAS_RIGHT         /* 右分岔优先：跟踪最右侧连续黑线 */
} gray_bias_mode_t;

typedef struct {
    uint16_t raw[8];        /* 8 路 12-bit ADC 原始值（已按左0~右7排序） */
    uint16_t thresh[8];     /* 8 路黑白分割阈值 */
    uint8_t  bits;          /* bit0=最左, bit7=最右, 1=压到黑线 */
    uint8_t  black_count;   /* 当前压到黑线的探头数量 */
    bool     line_found;    /* 是否检测到黑线 */
    bool     is_split;      /* 是否检测到两条分离黑线（分岔口特征） */
    float    error;         /* 当前误差 (-3.5 ~ +3.5，负=线在左需左转，正=线在右需右转) */
    float    last_valid_err;/* 最近一次有效误差（丢线方向记忆） */
} gray_state_t;

void BspGray_Init(void);
void BspGray_Sample(gray_bias_mode_t bias_mode);
const gray_state_t *BspGray_GetState(void);

/* 在线标定辅助（在 RUN_MODE_DBG_GRAY 中按 KEY 扫黑白地面自动计算阈值） */
void BspGray_CalibStart(void);
void BspGray_CalibFeed(void);
void BspGray_CalibFinishAndApply(void);

#endif /* BSP_GRAY_H */

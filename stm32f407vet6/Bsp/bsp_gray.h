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
    bool     is_cross;      /* 路口：压黑探头数 >= CFG_GRAY_CROSS_MIN_BLACK 且连成一段，连续确认后置位 */
    uint8_t  live_bits;     /* 上一帧好通道的黑白（不含坏通道补线），做回差用 */
    uint8_t  white_frames;  /* 连续全白帧数（丢线确认） */
    uint8_t  cross_frames;  /* 连续路口候选帧数（路口确认） */
    bool     held;          /* 本帧误差是保持值（路口候选或比正常线宽），不是实测 */
    float    error;         /* 当前误差 (-3.5 ~ +3.5，负=线在左需左转，正=线在右需右转) */
    float    last_valid_err;/* 最近一次有效误差（丢线方向记忆） */
} gray_state_t;

void BspGray_Init(void);
void BspGray_Sample(gray_bias_mode_t bias_mode);
const gray_state_t *BspGray_GetState(void);
/* 清掉跨帧状态（回差、丢线/路口确认计数、上一拍误差），阈值和原始值保留。
 * 停采一段时间后重新开始判线时调用（避障回线搜索），免得用到停采前留下的状态 */
void BspGray_ResetTrack(void);

/* 在线标定辅助（在 RUN_MODE_DBG_GRAY 中按 KEY 扫黑白地面自动计算阈值） */
void BspGray_CalibStart(void);
void BspGray_CalibFeed(void);
void BspGray_CalibFinishAndApply(void);

#endif /* BSP_GRAY_H */

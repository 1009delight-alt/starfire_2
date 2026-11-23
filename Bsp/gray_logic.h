#ifndef GRAY_LOGIC_H
#define GRAY_LOGIC_H

#include "bsp_gray.h"
#include <stdbool.h>
#include <stdint.h>

/* 灰度判线的纯计算部分：不碰硬件，真机和电脑回放共用。
 *   raw_hw       按硬件选通顺序（AD2..AD0 = 0..7）采到的 8 路 ADC 值
 *   black_is_low 1 = 压到黑线时 ADC 值变低
 *   reverse      1 = 硬件通道 0 在最右（装反），计算前翻转成“左 0 右 7”
 *   dead_mask    坏通道位图（逻辑顺序，bit i = 屏上从左数第 i 个方块，从 0 数）：读数忽略；
 *                两侧好通道都压线时补为压线，否则当白
 * 结果写入 st：raw/bits/black_count/line_found/is_split/error/last_valid_err。
 * st->thresh 按“左 0 右 7”的逻辑顺序给出。 */
void GrayLogic_Process(gray_state_t *st, const uint16_t raw_hw[8], gray_bias_mode_t bias,
                       bool black_is_low, bool reverse, uint8_t dead_mask);

#endif /* GRAY_LOGIC_H */

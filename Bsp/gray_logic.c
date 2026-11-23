#include "gray_logic.h"
#include "config.h"

static const float k_weights[8] = CFG_GRAY_WEIGHTS;

/* 偏左/偏右模式只取最左/最右的连续黑线段；居中模式取全部黑点的加权平均 */
static float compute_cluster_error(uint8_t bits, gray_bias_mode_t mode, float fallback)
{
    int start = -1, end = -1;

    if (mode == GRAY_BIAS_LEFT) {
        for (int i = 0; i < 8; ++i) {
            if ((bits & (1U << i)) != 0U) {
                if (start < 0) { start = i; }
                end = i;
            } else if (start >= 0) {
                break;
            }
        }
    } else if (mode == GRAY_BIAS_RIGHT) {
        for (int i = 7; i >= 0; --i) {
            if ((bits & (1U << i)) != 0U) {
                if (end < 0) { end = i; }
                start = i;
            } else if (end >= 0) {
                break;
            }
        }
    }

    if ((start >= 0) && (end >= start)) {
        float sum = 0.0f;
        int cnt = 0;
        for (int i = start; i <= end; ++i) {
            sum += k_weights[i];
            cnt++;
        }
        return sum / (float)cnt;
    }

    float sum = 0.0f;
    int cnt = 0;
    for (int i = 0; i < 8; ++i) {
        if ((bits & (1U << i)) != 0U) {
            sum += k_weights[i];
            cnt++;
        }
    }
    return (cnt > 0) ? (sum / (float)cnt) : fallback;
}

void GrayLogic_Process(gray_state_t *st, const uint16_t raw_hw[8], gray_bias_mode_t bias,
                       bool black_is_low, bool reverse, uint8_t dead_mask)
{
    uint8_t bits = 0U;
    uint8_t black_cnt = 0U;

    for (uint8_t hw = 0U; hw < 8U; ++hw) {
        uint8_t idx = reverse ? (uint8_t)(7U - hw) : hw;
        uint16_t val = raw_hw[hw];
        st->raw[idx] = val;
        if ((dead_mask & (1U << idx)) != 0U) {
            continue;   /* 坏通道读数不可信，先当白，下面再按两侧补 */
        }
        bool is_black = black_is_low ? (val < st->thresh[idx]) : (val > st->thresh[idx]);
        if (is_black) {
            bits |= (uint8_t)(1U << idx);
            black_cnt++;
        }
    }

    /* 坏通道两侧的好通道都压线时补为压线，避免一条线被坏通道劈成两段误判分岔。
     * 两端通道只有一侧邻居，不外推，保持白 */
    const uint8_t live_bits = bits;
    for (uint8_t i = 1U; i < 7U; ++i) {
        if (((dead_mask & (1U << i)) != 0U) &&
            ((live_bits & (1U << (i - 1U))) != 0U) && ((live_bits & (1U << (i + 1U))) != 0U)) {
            bits |= (uint8_t)(1U << i);
            black_cnt++;
        }
    }

    /* 统计连续黑线段数，>=2 说明遇到分岔口或双线 */
    uint8_t clusters = 0U;
    uint8_t prev = 0U;
    for (int i = 0; i < 8; ++i) {
        uint8_t cur = (uint8_t)((bits >> i) & 1U);
        if ((cur != 0U) && (prev == 0U)) {
            clusters++;
        }
        prev = cur;
    }

    st->bits = bits;
    st->black_count = black_cnt;
    st->line_found = (black_cnt > 0U);
    st->is_split = (clusters >= 2U);

    if (st->line_found) {
        st->error = compute_cluster_error(bits, bias, st->last_valid_err);
        st->last_valid_err = st->error;
    } else {
        /* 丢线时沿上次偏离方向加大纠偏找线 */
        if (st->last_valid_err < -0.2f) {
            st->error = -3.8f;
        } else if (st->last_valid_err > 0.2f) {
            st->error = 3.8f;
        } else {
            st->error = 0.0f;
        }
    }
}

#include "gray_logic.h"
#include "config.h"

static const float k_weights[8] = CFG_GRAY_WEIGHTS;

/* 偏左/偏右模式只取最左/最右的连续黑线段。居中模式只有一段时取全部黑点的加权平均；
 * 有两段以上（路口横线的残段、方框边线）时取离上一拍误差最近的一段，不被支线拉走 */
static float compute_cluster_error(uint8_t bits, gray_bias_mode_t mode, float fallback, uint8_t clusters)
{
    int start = -1, end = -1;

    if ((mode == GRAY_BIAS_CENTER) && (clusters >= 2U)) {
        float best_d = 1.0e9f;
        int i = 0;
        while (i < 8) {
            if ((bits & (1U << i)) == 0U) { ++i; continue; }
            int s0 = i;
            while ((i < 8) && ((bits & (1U << i)) != 0U)) { ++i; }
            float c = 0.5f * (k_weights[s0] + k_weights[i - 1]);
            float d = (c > fallback) ? (c - fallback) : (fallback - c);
            if (d < best_d) { best_d = d; start = s0; end = i - 1; }
        }
    }

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

void GrayLogic_ResetTrack(gray_state_t *st)
{
    st->live_bits = 0U;
    st->bits = 0U;
    st->black_count = 0U;
    st->white_frames = 0U;
    st->cross_frames = 0U;
    st->line_found = false;
    st->is_split = false;
    st->is_cross = false;
    st->held = false;
    st->error = 0.0f;
    st->last_valid_err = 0.0f;
}

void GrayLogic_Process(gray_state_t *st, const uint16_t raw_hw[8], gray_bias_mode_t bias,
                       bool black_is_low, bool reverse, uint8_t dead_mask)
{
    uint8_t bits = 0U;
    uint8_t black_cnt = 0U;
    const uint8_t prev_live = st->live_bits;

    for (uint8_t hw = 0U; hw < 8U; ++hw) {
        uint8_t idx = reverse ? (uint8_t)(7U - hw) : hw;
        uint16_t val = raw_hw[hw];
        st->raw[idx] = val;
        if ((dead_mask & (1U << idx)) != 0U) {
            continue;   /* 坏通道读数不可信，先当白，下面再按两侧补 */
        }
        /* 回差：上一帧是黑，要越过“阈值 + 回差”（往白的方向）才变白；上一帧是白，要越过
         * “阈值 − 回差”（往黑的方向）才变黑。探头在线边缘时不会每帧黑白来回翻 */
        bool was_black = (prev_live & (1U << idx)) != 0U;
        int32_t band = was_black ? (int32_t)CFG_GRAY_HYST_ADC : -(int32_t)CFG_GRAY_HYST_ADC;
        int32_t th = (int32_t)st->thresh[idx];
        bool is_black = black_is_low ? ((int32_t)val < th + band) : ((int32_t)val > th - band);
        if (is_black) {
            bits |= (uint8_t)(1U << idx);
            black_cnt++;
        }
    }

    /* 坏通道两侧的好通道都压线时补为压线，避免一条线被坏通道劈成两段误判分岔。
     * 两端通道只有一侧邻居，不外推，保持白 */
    const uint8_t live_bits = bits;
    st->live_bits = live_bits;   /* 下一帧按它做回差（不含坏通道补线） */
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
    st->is_split = (clusters >= 2U);
    /* 路口：赛道上的路口一律直行（车库等方框的边线横穿主线，成十字或 T 字）。横线压住大半排探头时
     * 加权平均会被一侧支线拉偏，所以压黑探头数够多且连成一段时判为路口：误差保持上一拍、
     * 不更新丢线方向记忆，直行压过去。分岔口是两段分开的线，不算路口，偏置选线照常。
     * 连续 CFG_GRAY_CONFIRM_FRAMES 帧才置 is_cross；还没确认的那一帧误差也先保持（保持一帧无害，
     * 被支线拉一帧反而会拐） */
    const bool cross_cand = (black_cnt >= CFG_GRAY_CROSS_MIN_BLACK) && (clusters == 1U);
    st->cross_frames = cross_cand ? (uint8_t)((st->cross_frames < 255U) ? (st->cross_frames + 1U) : 255U) : 0U;
    st->white_frames = (black_cnt == 0U) ? (uint8_t)((st->white_frames < 255U) ? (st->white_frames + 1U) : 255U) : 0U;
    st->is_cross = cross_cand && (st->cross_frames >= CFG_GRAY_CONFIRM_FRAMES);

    /* 丢线确认：有线时立刻算找到；全白要连续 CFG_GRAY_CONFIRM_FRAMES 帧才算丢线，
     * 不够帧数时保持上一帧的判断（上电后从没见过线时仍是“没找到”） */
    if (black_cnt > 0U) {
        st->line_found = true;
    } else if (st->white_frames >= CFG_GRAY_CONFIRM_FRAMES) {
        st->line_found = false;
    }

    /* 比正常线宽（1~3 路）还宽的一段：横线正在进来（先和主线连成 4 路，下一帧才到路口门槛），
     * 这时算出的误差已被支线拉偏，不能拿来转向，也不能记进 last_valid_err 被路口保持下去。
     * 保持的误差限在 ±CFG_GRAY_CROSS_HOLD_MAX_ERR：路口要直穿，上一拍偏得再大也只带小转向 */
    const bool wide = (clusters == 1U) && (black_cnt > CFG_GRAY_LINE_MAX_BLACK);
    st->held = cross_cand || wide;
    if (cross_cand || wide) {
        float h = st->last_valid_err;
        if (h > CFG_GRAY_CROSS_HOLD_MAX_ERR) { h = CFG_GRAY_CROSS_HOLD_MAX_ERR; }
        if (h < -CFG_GRAY_CROSS_HOLD_MAX_ERR) { h = -CFG_GRAY_CROSS_HOLD_MAX_ERR; }
        st->error = h;
    } else if (black_cnt > 0U) {
        st->error = compute_cluster_error(bits, bias, st->last_valid_err, clusters);
        st->last_valid_err = st->error;
    } else if (st->line_found) {
        /* 全白但还没确认丢线：误差保持，不急着打满找线 */
        st->error = st->last_valid_err;
    } else {
        /* 丢线时沿上次偏离方向加大纠偏找线。不一步跳到 ±3.8（线从中间消失时会猛转、D 项出尖峰），
         * 每帧按一阶低通往目标靠：err += k·(目标 − err)，起点就是确认帧保持下来的上一拍误差 */
        float target = 0.0f;
        if (st->last_valid_err < -0.2f) {
            target = -3.8f;
        } else if (st->last_valid_err > 0.2f) {
            target = 3.8f;
        }
        st->error += CFG_GRAY_LOST_ERR_SMOOTH * (target - st->error);
    }
}

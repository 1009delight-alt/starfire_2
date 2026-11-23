#include "bsp_openmv.h"
#include "bsp_uart.h"
#include "board.h"
#include "config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static openmv_state_t s_mv;
static char     s_rx_frame[64];
static uint8_t  s_rx_used = 0U;
static bool     s_in_frame = false;     /* 只有收到 $ 之后的字节才进帧缓冲 */
static uint8_t  s_ack_pending = 0U;
static uint32_t s_last_cmd_ms = 0U;

const char *BspOpenmv_TaskName(openmv_task_t task)
{
    switch (task) {
    case OPENMV_TASK_TURN:  return "TURN";
    case OPENMV_TASK_COLOR: return "COLOR";
    case OPENMV_TASK_PARK:  return "PARK";
    case OPENMV_TASK_IDLE:
    default:                return "IDLE";
    }
}

static bool parse_task_name(const char *s, openmv_task_t *out)
{
    if (strcmp(s, "IDLE") == 0)  { *out = OPENMV_TASK_IDLE;  return true; }
    if (strcmp(s, "TURN") == 0)  { *out = OPENMV_TASK_TURN;  return true; }
    if (strcmp(s, "COLOR") == 0) { *out = OPENMV_TASK_COLOR; return true; }
    if (strcmp(s, "PARK") == 0)  { *out = OPENMV_TASK_PARK;  return true; }
    return false;
}

static void send_task_cmd(openmv_task_t task)
{
    char buf[32];
    if ((task == OPENMV_TASK_COLOR) && (s_mv.color_count > 0U) && !s_mv.color_locked) {
        /* 续接：把已收到的颜色一起下发，相机重启后从下一块接着数（文档 4.2） */
        (void)snprintf(buf, sizeof(buf), "$CMD:COLOR:%s#\r\n", s_mv.color_seq);
    } else {
        (void)snprintf(buf, sizeof(buf), "$CMD:%s#\r\n", BspOpenmv_TaskName(task));
    }
    BspUart_SendStr(BSP_UART_OPENMV, buf);
}

/* 解析十进制整数：可选负号，1~5 位数字，范围 [lo, hi]；*pp 前进到数字之后 */
static bool parse_int(const char **pp, int lo, int hi, int *out)
{
    const char *p = *pp;
    bool neg = false;
    int v = 0, digits = 0;
    if (*p == '-') { neg = true; ++p; }
    while ((*p >= '0') && (*p <= '9')) {
        if (++digits > 5) { return false; }
        v = v * 10 + (*p - '0');
        ++p;
    }
    if (digits == 0) { return false; }
    if (neg) { v = -v; }
    if ((v < lo) || (v > hi)) { return false; }
    *out = v;
    *pp = p;
    return true;
}

/* 解析 "<n>,<SEQ>"：n 为 1~4，SEQ 恰好 n 个 R/G/B/Y，以 '-' 分隔，后面不能有多余字符 */
static bool parse_color(const char *p, int *cnt_out, const char **seq_out)
{
    if ((p[0] < '1') || (p[0] > '4') || (p[1] != ',')) {
        return false;
    }
    int cnt = p[0] - '0';
    const char *seq = p + 2;
    for (int i = 0; i < cnt; ++i) {
        char c = seq[2 * i];
        if ((c != 'R') && (c != 'G') && (c != 'B') && (c != 'Y')) {
            return false;
        }
        char sep = seq[2 * i + 1];
        if (i < cnt - 1) {
            if (sep != '-') { return false; }
        } else if (sep != '\0') {
            return false;
        }
    }
    *cnt_out = cnt;
    *seq_out = seq;
    return true;
}

/* "PARK:" 之后：<0|1>,<cx -100~100>,<area 0~19200>，不能有多余字符 */
static bool parse_park(const char *p, int *flag, int *cx, int *area)
{
    if (((p[0] != '0') && (p[0] != '1')) || (p[1] != ',')) {
        return false;
    }
    *flag = p[0] - '0';
    p += 2;
    if (!parse_int(&p, -100, 100, cx) || (*p++ != ',')) { return false; }
    if (!parse_int(&p, 0, 160 * 120, area) || (*p != '\0')) { return false; }
    return true;
}

/* "HB:" 之后：<任务名>,<fps 0~255>，不能有多余字符 */
static bool parse_hb(const char *p, openmv_task_t *task, int *fps)
{
    const char *comma = strchr(p, ',');
    char name[8];
    size_t len;
    if (comma == NULL) { return false; }
    len = (size_t)(comma - p);
    if ((len == 0U) || (len >= sizeof(name))) { return false; }
    memcpy(name, p, len);
    name[len] = '\0';
    if (!parse_task_name(name, task)) { return false; }
    p = comma + 1;
    if ((*p == '-') || !parse_int(&p, 0, 255, fps) || (*p != '\0')) { return false; }
    return true;
}

/* 处理一帧；任何字段不合法都整帧丢弃、不改状态，返回 false */
static bool handle_openmv_frame(const char *body, uint32_t now_ms)
{
    openmv_task_t t;

    if (strncmp(body, "ACK:", 4) == 0) {
        if (!parse_task_name(body + 4, &t)) {
            return false;
        }
        s_mv.ack_task = t;
        if (t == s_mv.target_task) {
            s_ack_pending = 0U;
        }
    } else if (strncmp(body, "TURN:", 5) == 0) {
        if ((strcmp(body + 5, "L") != 0) && (strcmp(body + 5, "R") != 0)) {
            return false;
        }
        /* 只采信相机已确认处于对应任务时的结果，避免切任务前排队的旧帧误入 */
        if ((s_mv.ack_task == OPENMV_TASK_TURN) && (s_mv.target_task == OPENMV_TASK_TURN)) {
            s_mv.turn_dir = body[5];
        }
    } else if (strncmp(body, "COLOR:", 6) == 0) {
        int cnt = 0;
        const char *seq = NULL;
        if (!parse_color(body + 6, &cnt, &seq)) {
            return false;
        }
        if ((s_mv.ack_task != OPENMV_TASK_COLOR) || (s_mv.target_task != OPENMV_TASK_COLOR) ||
            s_mv.color_locked || ((uint8_t)cnt < s_mv.color_count)) {
            return true;    /* 格式合法，只是当前不采信 */
        }
        /* 新序列必须以已收到的序列开头：相机重启后重新计数的结果不能覆盖已有答案 */
        if ((s_mv.color_count > 0U) &&
            (strncmp(seq, s_mv.color_seq, (size_t)(2U * s_mv.color_count - 1U)) != 0)) {
            s_mv.color_reject_count++;
            return true;
        }
        s_mv.color_count = (uint8_t)cnt;
        memcpy(s_mv.color_seq, seq, (size_t)(2 * cnt - 1));
        s_mv.color_seq[2 * cnt - 1] = '\0';
        if (s_mv.color_count >= 4U) {
            s_mv.color_locked = true;
        }
    } else if (strncmp(body, "PARK:", 5) == 0) {
        int flag = 0, cx = 0, area = 0;
        if (!parse_park(body + 5, &flag, &cx, &area)) {
            return false;
        }
        if ((s_mv.ack_task == OPENMV_TASK_PARK) && (s_mv.target_task == OPENMV_TASK_PARK)) {
            /* 以最新一帧为准：相机已做连续帧确认，flag=0 立即撤销“看见 P 牌” */
            s_mv.park_seen = (flag == 1);
            s_mv.park_cx = (flag == 1) ? (int16_t)cx : 0;
            s_mv.park_area = (flag == 1) ? (uint16_t)area : 0U;
            s_mv.park_last_ms = now_ms;
        }
    } else if (strncmp(body, "HB:", 3) == 0) {
        int fps = 0;
        if (!parse_hb(body + 3, &t, &fps)) {
            return false;
        }
        s_mv.ack_task = t;
        s_mv.fps = (uint8_t)fps;
        if (t == s_mv.target_task) {
            s_ack_pending = 0U;
        } else if (s_ack_pending == 0U) {
            /* 相机报告的任务与主控要求不符（如相机重启回到 IDLE）：重新下发 */
            s_ack_pending = 1U;
            s_mv.resync_count++;
        }
    } else {
        return false;
    }

    /* 只有合法帧才刷新在线状态 */
    s_mv.last_rx_ms = now_ms;
    s_mv.online = true;
    return true;
}

void BspOpenmv_Init(void)
{
    memset(&s_mv, 0, sizeof(s_mv));
    (void)snprintf(s_mv.color_seq, sizeof(s_mv.color_seq), "----");
    s_rx_used = 0U;
    s_in_frame = false;
    s_ack_pending = 0U;

#if (CFG_ENABLE_OPENMV != 0U)
    BspUart_Init(BSP_UART_OPENMV, 115200U);
    BspOpenmv_SetTask(OPENMV_TASK_IDLE);
#endif
}

void BspOpenmv_SetTask(openmv_task_t task)
{
    s_mv.target_task = task;
    if (task == OPENMV_TASK_TURN) {
        s_mv.turn_dir = 0;
    } else if (task == OPENMV_TASK_COLOR) {
        if (!s_mv.color_locked) {
            s_mv.color_count = 0U;
            (void)snprintf(s_mv.color_seq, sizeof(s_mv.color_seq), "----");
        }
    } else if (task == OPENMV_TASK_PARK) {
        s_mv.park_seen = false;
        s_mv.park_cx = 0;
        s_mv.park_area = 0U;
    }
#if (CFG_ENABLE_OPENMV != 0U)
    s_ack_pending = 1U;
    s_last_cmd_ms = HAL_GetTick();
    send_task_cmd(task);
#endif
}

void BspOpenmv_ResetAll(void)
{
    s_mv.turn_dir = 0;
    s_mv.color_count = 0U;
    s_mv.color_locked = false;
    (void)snprintf(s_mv.color_seq, sizeof(s_mv.color_seq), "----");
    s_mv.park_seen = false;
    s_mv.park_cx = 0;
    s_mv.park_area = 0U;
    s_mv.target_task = OPENMV_TASK_IDLE;
#if (CFG_ENABLE_OPENMV != 0U)
    BspUart_SendStr(BSP_UART_OPENMV, "$CMD:RESET#\r\n");
    s_ack_pending = 1U;
    s_last_cmd_ms = HAL_GetTick();
#endif
}

void BspOpenmv_ClearTurn(void)
{
    s_mv.turn_dir = 0;
}

void BspOpenmv_Query(void)
{
#if (CFG_ENABLE_OPENMV != 0U)
    BspUart_SendStr(BSP_UART_OPENMV, "$CMD:QUERY#\r\n");
#endif
}

void BspOpenmv_Poll(uint32_t now_ms)
{
#if (CFG_ENABLE_OPENMV == 0U)
    (void)now_ms;
    return;
#else
    uint8_t ch;
    while (BspUart_ReadByte(BSP_UART_OPENMV, &ch)) {
        if (ch == (uint8_t)'$') {
            s_rx_used = 0U;
            s_in_frame = true;
        } else if (!s_in_frame) {
            /* 帧外字节（缺 $ 的残帧、噪声）一律丢弃 */
        } else if (ch == (uint8_t)'#') {
            if (s_rx_used > 0U) {
                s_rx_frame[s_rx_used] = '\0';
                if (!handle_openmv_frame(s_rx_frame, now_ms)) {
                    s_mv.bad_frame_count++;
                }
            }
            s_rx_used = 0U;
            s_in_frame = false;
        } else if ((ch != (uint8_t)'\r') && (ch != (uint8_t)'\n')) {
            if (s_rx_used < (sizeof(s_rx_frame) - 1U)) {
                s_rx_frame[s_rx_used++] = (char)ch;
            } else {
                s_rx_used = 0U;         /* 超长：整帧丢弃，等下一个 $ */
                s_in_frame = false;
            }
        }
    }

    if (s_mv.online && ((uint32_t)(now_ms - s_mv.last_rx_ms) > 1500U)) {
        s_mv.online = false;
    }

    /* 若切模式未收到 ACK，则周期重发确保切换成功 */
    if ((s_ack_pending != 0U) && ((uint32_t)(now_ms - s_last_cmd_ms) >= CFG_OPENMV_CMD_RETRY_MS)) {
        s_last_cmd_ms = now_ms;
        send_task_cmd(s_mv.target_task);
    }
#endif
}

const openmv_state_t *BspOpenmv_GetState(void)
{
    return &s_mv;
}

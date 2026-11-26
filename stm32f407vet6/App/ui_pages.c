#include "ui_pages.h"
#include "bsp_oled.h"
#include "config.h"

#include <stdio.h>

/* ============================================================================
 *  分步调试页面（160x128）。统一版式：
 *    y = 0~17   彩色标题栏（2 倍字）：青色 = 待机，绿色 = 运行/在线，橙色 = 标定，红色 = 离线
 *    y = 20~116 主要数值用 2 倍字（12x16，一行 13 个字符），次要数值用 1 倍字
 *    y = 119    灰色按键提示
 * ============================================================================ */

#define LOW_BATT_V   7.0f    /* 主文档第 6 章：低于 7.0 V 提示换电池 */

static void draw_gray_blocks(uint8_t bits, uint8_t y, uint8_t h)
{
    for (uint8_t i = 0U; i < 8U; ++i) {
        bool blk = ((bits & (1U << i)) != 0U);
        BspOled_SetColor(blk ? OLED_C_WHITE : OLED_C_GRAY);
        BspOled_DrawRect((uint8_t)(2U + i * 20U), y, 16U, h, blk);
    }
    BspOled_SetColor(OLED_C_WHITE);
}

void UiPage_MotorEnc(const motor_state_t *m, uint8_t next_step)
{
    char buf[32];
    BspOled_Clear();
    BspOled_Title("MOTOR ENC", OLED_C_CYAN);

    BspOled_DrawStr(2U, 22U, "ENC:", 2U);
    BspOled_SetColor(m->enc_online ? OLED_C_GREEN : OLED_C_RED);
    BspOled_DrawStr(50U, 22U, m->enc_online ? "OK" : "LOST", 2U);
    BspOled_SetColor(((m->battery_v > 0.1f) && (m->battery_v < LOW_BATT_V)) ? OLED_C_ORANGE : OLED_C_YELLOW);
    BspOled_Printf(98U, 22U, 2U, "%4.1fV", m->battery_v);
    BspOled_SetColor(OLED_C_WHITE);

    /* 按车上位置显示（bsp_motor 内部数组按位置排，驱动板通道由 CFG_MOTOR_SWAP_LR 换算） */
    static const char *const k_pos[4] = { "LF", "LB", "RF", "RB" };
    for (uint8_t i = 0U; i < 4U; ++i) {
        BspOled_Printf(2U, (uint8_t)(40U + i * 16U), 2U, "%s%8ld", k_pos[i], (long)m->fwd_enc[i]);
    }
    BspOled_Printf(2U, 106U, 1U, "D:%5dmm   Y:%4ddeg", (int)m->total_dist_mm, (int)m->seg_yaw_deg);

    if (next_step < 4U) { (void)snprintf(buf, sizeof(buf), "Short:Run %s  Long:Clear", k_pos[next_step]); }
    else                { (void)snprintf(buf, sizeof(buf), "Short:Run ALL  Long:Clear"); }
    BspOled_Footer(buf);
}

void UiPage_MotionCal(const motor_state_t *m, bool active, uint8_t task)
{
    BspOled_Clear();
    BspOled_Title(active ? "MOTION RUN" : "MOTION IDLE", active ? OLED_C_GREEN : OLED_C_CYAN);

    BspOled_SetColor(OLED_C_YELLOW);
    BspOled_DrawStr(2U, 24U, (task == 0U) ? "FWD 1000mm" : "TURN +90deg", 2U);
    BspOled_SetColor(OLED_C_WHITE);
    BspOled_Printf(2U, 46U, 2U, "D:%6dmm", (int)m->seg_dist_mm);
    BspOled_Printf(2U, 64U, 2U, "Y:%6ddeg", (int)m->seg_yaw_deg);
    BspOled_Printf(2U, 88U, 1U, "VL:%5d  VR:%5d  mm/s",
                   (int)(0.5f * (m->wheel_spd_mm_s[0] + m->wheel_spd_mm_s[1])),
                   (int)(0.5f * (m->wheel_spd_mm_s[2] + m->wheel_spd_mm_s[3])));
    BspOled_Footer("Short:Go/Stop Long:Task");
}

void UiPage_Gray(const gray_state_t *g, bool calib)
{
    BspOled_Clear();
    BspOled_Title(calib ? "GRAY CALIB" : "GRAY LIVE", calib ? OLED_C_ORANGE : OLED_C_CYAN);

    /* 8 路黑白状态：实心白块 = 压到黑线 */
    draw_gray_blocks(g->bits, 21U, 12U);

    /* 8 路 ADC 柱状图（0~4095 映射到 46 px），红线 = 该路阈值 */
    const uint8_t bar_top = 35U, bar_h = 46U;
    for (uint8_t i = 0U; i < 8U; ++i) {
        uint8_t x = (uint8_t)(2U + i * 20U);
        uint32_t raw = (g->raw[i] > 4095U) ? 4095U : g->raw[i];
        uint32_t th  = (g->thresh[i] > 4095U) ? 4095U : g->thresh[i];
        uint8_t hv = (uint8_t)((raw * bar_h) / 4095U);
        uint8_t ht = (uint8_t)((th * bar_h) / 4095U);
        BspOled_SetColor(OLED_C_GRAY);
        BspOled_DrawRect(x, bar_top, 16U, bar_h, false);
        if (hv > 0U) {
            BspOled_SetColor(OLED_C_CYAN);
            BspOled_DrawRect(x, (uint8_t)(bar_top + bar_h - hv), 16U, hv, true);
        }
        BspOled_SetColor(OLED_C_RED);
        BspOled_DrawRect(x, (uint8_t)(bar_top + bar_h - ((ht > 0U) ? ht : 1U)), 16U, 1U, true);
    }

    /* 原始值两行，每行 4 路，40 px 一列 */
    BspOled_SetColor(OLED_C_WHITE);
    for (uint8_t i = 0U; i < 8U; ++i) {
        BspOled_Printf((uint8_t)(4U + (i % 4U) * 40U), (uint8_t)(84U + (i / 4U) * 9U), 1U, "%4u", g->raw[i]);
    }
    BspOled_Printf(2U, 103U, 2U, "E:%+.2f", g->error);
    if (g->is_cross) {
        BspOled_SetColor(OLED_C_GREEN);
        BspOled_DrawStr(112U, 103U, "CRS", 2U);
    } else if (g->is_split) {
        BspOled_SetColor(OLED_C_YELLOW);
        BspOled_DrawStr(112U, 103U, "SPL", 2U);
    }
    BspOled_Footer("Long: Start/End Calib");
}

void UiPage_Sonar(const sonar_state_t *s, bool healthy, bool obstacle)
{
    char buf[32];
    BspOled_Clear();
    BspOled_Title("HC-SR04", healthy ? OLED_C_GREEN : OLED_C_RED);

    BspOled_SetColor(healthy ? OLED_C_GREEN : OLED_C_RED);
    BspOled_DrawStr(2U, 24U, healthy ? "ECHO OK" : (s->online ? "NO ECHO" : "NO DEV"), 2U);
    BspOled_SetColor(OLED_C_YELLOW);
    if (s->valid) {
        BspOled_Printf(2U, 46U, 3U, "%4u mm", (unsigned)s->dist_mm);
    } else {
        BspOled_DrawStr(2U, 46U, " FAR   ", 3U);
    }
    BspOled_SetColor(OLED_C_WHITE);
    BspOled_Printf(2U, 76U, 1U, "RAW:%4umm ECHO:%5uus", (unsigned)s->raw_mm, (unsigned)s->echo_us);
    BspOled_Printf(2U, 86U, 1U, "HITS:%u  MISS:%u", (unsigned)s->near_hits, (unsigned)s->miss_count);
    BspOled_SetColor(obstacle ? OLED_C_RED : OLED_C_GREEN);
    BspOled_DrawStr(2U, 98U, obstacle ? "OBS: YES!" : "OBS: NO", 2U);
    (void)snprintf(buf, sizeof(buf), "Trigger <= %u mm", (unsigned)CFG_OBS_TRIG_DIST_MM);
    BspOled_Footer(buf);
}

void UiPage_Openmv(const openmv_state_t *mv, const char *cmd_name, const char *ack_name, bool running)
{
    BspOled_Clear();
    if (running) {
        BspOled_Title(mv->online ? "MV RUN" : "MV RUN OFF", mv->online ? OLED_C_GREEN : OLED_C_RED);
    } else {
        BspOled_Title(mv->online ? "OPENMV ON" : "OPENMV OFF", mv->online ? OLED_C_CYAN : OLED_C_RED);
    }

    BspOled_Printf(2U, 22U, 2U, "FPS:%3u", (unsigned)mv->fps);
    BspOled_SetColor(OLED_C_GRAY);
    BspOled_Printf(100U, 26U, 1U, "BAD:%u", (unsigned)mv->bad_frame_count);
    BspOled_SetColor(OLED_C_WHITE);
    BspOled_Printf(2U, 40U, 2U, "CMD:%s", cmd_name);
    BspOled_SetColor((mv->ack_task == mv->target_task) ? OLED_C_GREEN : OLED_C_ORANGE);
    BspOled_Printf(2U, 56U, 2U, "ACK:%s", ack_name);

    BspOled_SetColor(OLED_C_WHITE);
    BspOled_Printf(2U, 74U, 2U, "TURN:%c", (mv->turn_dir != 0) ? mv->turn_dir : '-');
    BspOled_SetColor(mv->park_seen ? OLED_C_GREEN : OLED_C_WHITE);
    BspOled_Printf(100U, 74U, 2U, "P:%d", mv->park_seen ? 1 : 0);
    BspOled_SetColor(OLED_C_WHITE);
    BspOled_Printf(2U, 92U, 1U, "PARK cx:%+4d  area:%5u", (int)mv->park_cx, (unsigned)mv->park_area);

    BspOled_DrawColorSeq(102U, mv->color_seq, 2U);
    BspOled_Footer(running ? "Short:Stop" : "Short:Go  Long:Task");
}

void UiPage_Line(const gray_state_t *g, const motor_state_t *m, bool running, const char *bias_name)
{
    BspOled_Clear();
    BspOled_Title(running ? "LINE RUN" : "LINE STOP", running ? OLED_C_GREEN : OLED_C_CYAN);

    BspOled_SetColor(OLED_C_YELLOW);
    BspOled_Printf(2U, 22U, 2U, "BIAS:%s", bias_name);
    draw_gray_blocks(g->bits, 41U, 18U);
    BspOled_Printf(2U, 64U, 2U, "ERR:%+.2f", g->error);
    BspOled_SetColor(g->is_cross ? OLED_C_GREEN : (g->is_split ? OLED_C_YELLOW : OLED_C_WHITE));
    BspOled_Printf(2U, 83U, 1U, "%-5s  D:%6dmm", g->is_cross ? "CROSS" : (g->is_split ? "SPLIT" : "LINE"),
                   (int)m->total_dist_mm);
    /* 左右轮目标/实测速度：差速给出来了轮子却没跟上，问题在轮速环或机械，不在循迹 PID */
    BspOled_SetColor(OLED_C_WHITE);
    BspOled_Printf(2U, 95U, 1U, "L T%+4d A%+4d mm/s",
                   (int)(0.5f * (m->target_spd_mm_s[0] + m->target_spd_mm_s[1])),
                   (int)(0.5f * (m->wheel_spd_mm_s[0] + m->wheel_spd_mm_s[1])));
    BspOled_Printf(2U, 106U, 1U, "R T%+4d A%+4d mm/s",
                   (int)(0.5f * (m->target_spd_mm_s[2] + m->target_spd_mm_s[3])),
                   (int)(0.5f * (m->wheel_spd_mm_s[2] + m->wheel_spd_mm_s[3])));
    BspOled_Footer("Short:Go/Stop Long:Bias");
}

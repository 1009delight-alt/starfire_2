#ifndef BSP_OLED_H
#define BSP_OLED_H

/* 车载显示屏驱动。2026-11-21 起硬件换成 1.8 寸 128x160 SPI 彩屏（ST7735S），
 * 横屏使用，逻辑画布 160x128。接口名沿用 BspOled_*，调试页面代码不用改。
 * 帧缓冲每像素 4 位（调色板下标），按 8 行一条带做脏检测。 */

#include <stdbool.h>
#include <stdint.h>

#define OLED_WIDTH      160U
#define OLED_HEIGHT     128U

/* 调色板下标 */
enum {
    OLED_C_BLACK = 0,
    OLED_C_WHITE,
    OLED_C_RED,
    OLED_C_GREEN,
    OLED_C_BLUE,
    OLED_C_YELLOW,
    OLED_C_CYAN,
    OLED_C_GRAY,
    OLED_C_ORANGE,
    OLED_C_COUNT
};

bool BspOled_Init(void);
bool BspOled_IsOnline(void);

void BspOled_Clear(void);
void BspOled_SetColor(uint8_t color);    /* 之后的点、框、字使用该前景色；Clear 后恢复白色 */
void BspOled_DrawPixel(uint8_t x, uint8_t y, bool on);
void BspOled_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool fill);
void BspOled_DrawChar(uint8_t x, uint8_t y, char ch, uint8_t scale);
void BspOled_DrawStr(uint8_t x, uint8_t y, const char *str, uint8_t scale);
void BspOled_Printf(uint8_t x, uint8_t y, uint8_t scale, const char *fmt, ...);
/* 页面公共元素：顶部彩色标题栏（2 倍黑字，高 18 px）、底部灰色操作提示（y = 119）、
 * 居中的四色序列（R/G/B/Y 用对应颜色）。调用后前景色恢复白色。 */
void BspOled_Title(const char *text, uint8_t bg_color);
void BspOled_Footer(const char *text);
void BspOled_DrawColorSeq(uint8_t y, const char *seq, uint8_t scale);
void BspOled_Flush(void);          /* 阻塞发送全部有变化的条带（整屏约 35 ms），只在初始化时用 */
bool BspOled_FlushStep(void);      /* 每次最多发一条带，返回 true 表示已同步 */

/* 比赛专用页面（只写帧缓冲）：顶部赛段状态，中下大号彩色持续锁定四色结果供裁判拍照 */
void BspOled_ShowRaceScreen(const char *stage_name,
                            float total_dist_mm,
                            const char *color_seq,
                            bool color_locked,
                            char turn_dir,
                            uint16_t tof_mm,
                            bool enc_ok,
                            bool mv_ok);

#endif /* BSP_OLED_H */

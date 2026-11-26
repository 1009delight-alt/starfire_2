/* 屏幕页面电脑预览与检查。
 * 编译真实的 Bsp/bsp_oled.c 与 App/ui_pages.c，HAL 换成本文件的替身：
 * SPI 字节流按 DC 脚电平拆成命令和数据，交给一个简化的 ST7735 模拟器（CASET/RASET/RAMWR），
 * 因此预览图是驱动真正发出去的像素，而不是直接读帧缓冲。
 * 用法：render.exe <输出目录>，每个页面输出一张 PPM（由 run_ui.ps1 转成 PNG 拼图）。 */
#include "stm32f4xx_hal.h"
#include "board.h"
#include "config.h"
#include "bsp_oled.h"
#include "ui_pages.h"

#include <stdio.h>
#include <string.h>

GPIO_TypeDef ui_gpioa, ui_gpiob, ui_gpioc, ui_gpioe;
SPI_TypeDef ui_spi2;

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { g_fail++; printf("FAIL: " __VA_ARGS__); printf("\n"); } } while (0)

/* ==================== ST7735 模拟器 ==================== */
#define W 160
#define H 128
static uint16_t lcd[H][W];
static bool cs_high = true, dc_data = true;
static uint8_t cur_cmd = 0;
static uint8_t args[16]; static int nargs = 0;
static int win_x0, win_x1, win_y0, win_y1, px, py;
static bool hi_pending = false; static uint8_t hi_byte;
static uint8_t madctl = 0xFF, colmod = 0xFF;
static bool disp_on = false;
static int ramwr_count = 0;           /* 本次统计窗口内的 RAMWR 次数 */
static long pixel_bytes = 0;
static int out_of_window = 0;
static int spi_while_cs_high = 0;

uint32_t HAL_GetTick(void) { return 0U; }
void HAL_Delay(uint32_t ms) { (void)ms; }
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init) { (void)port; (void)init; }
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *h) { return (h->Instance == SPI2) ? HAL_OK : HAL_ERROR; }

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState st)
{
    if (port == TFT_CS_PORT && pin == TFT_CS_PIN)  { cs_high = (st == GPIO_PIN_SET); }
    if (port == TFT_DC_PORT && pin == TFT_DC_PIN)  { dc_data = (st == GPIO_PIN_SET); }
}

static void lcd_byte(uint8_t b)
{
    if (!dc_data) {                     /* 命令 */
        cur_cmd = b; nargs = 0; hi_pending = false;
        if (b == 0x2C) { px = win_x0; py = win_y0; ramwr_count++; }
        if (b == 0x29) { disp_on = true; }
        return;
    }
    if (cur_cmd == 0x2C) {              /* 像素数据，RGB565 高字节在前 */
        pixel_bytes++;
        if (!hi_pending) { hi_byte = b; hi_pending = true; return; }
        hi_pending = false;
        if (px < 0 || px >= W || py < 0 || py >= H || py > win_y1) { out_of_window++; return; }
        lcd[py][px] = (uint16_t)((hi_byte << 8) | b);
        if (++px > win_x1) { px = win_x0; py++; }
        return;
    }
    if (nargs < 16) { args[nargs++] = b; }
    if (cur_cmd == 0x2A && nargs == 4) { win_x0 = args[1]; win_x1 = args[3]; }
    if (cur_cmd == 0x2B && nargs == 4) { win_y0 = args[1]; win_y1 = args[3]; }
    if (cur_cmd == 0x36 && nargs == 1) { madctl = args[0]; }
    if (cur_cmd == 0x3A && nargs == 1) { colmod = args[0]; }
}

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *h, uint8_t *data, uint16_t size, uint32_t timeout)
{
    (void)h; (void)timeout;
    if (cs_high) { spi_while_cs_high++; return HAL_OK; }
    for (uint16_t i = 0; i < size; ++i) { lcd_byte(data[i]); }
    return HAL_OK;
}

/* ==================== 工具 ==================== */
static void reset_counters(void) { ramwr_count = 0; pixel_bytes = 0; }

/* 反复调用 FlushStep 直到同步；检查每一步最多发一条带 */
static int flush_all(const char *name)
{
    int steps = 0;
    for (;;) {
        reset_counters();
        bool done = BspOled_FlushStep();
        CHECK(ramwr_count <= 1, "%s: one FlushStep sent %d bands", name, ramwr_count);
        CHECK(pixel_bytes <= W * 8 * 2, "%s: one FlushStep sent %ld bytes", name, pixel_bytes);
        if (done) { CHECK(ramwr_count == 0, "%s: FlushStep returned done but sent data", name); break; }
        if (++steps > 64) { CHECK(0, "%s: FlushStep never finished", name); break; }
    }
    return steps;
}

static void save_ppm(const char *dir, const char *name)
{
    char path[512];
    snprintf(path, sizeof(path), "%s/%s.ppm", dir, name);
    FILE *f = fopen(path, "wb");
    if (!f) { CHECK(0, "cannot write %s", path); return; }
    fprintf(f, "P6\n%d %d\n255\n", W, H);
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            uint16_t c = lcd[y][x];
            uint8_t rgb[3] = { (uint8_t)(((c >> 11) & 0x1F) * 255 / 31),
                               (uint8_t)(((c >> 5) & 0x3F) * 255 / 63),
                               (uint8_t)((c & 0x1F) * 255 / 31) };
            fwrite(rgb, 1, 3, f);
        }
    }
    fclose(f);
}

/* 画一页 → 分步刷新 → 再画一遍同样内容必须不再发送 → 保存 */
typedef void (*page_fn)(void);
static void run_page(const char *dir, const char *name, page_fn fn)
{
    fn();
    int steps = flush_all(name);
    fn();
    reset_counters();
    bool done = BspOled_FlushStep();
    CHECK(done && ramwr_count == 0, "%s: unchanged redraw still sent %d bands", name, ramwr_count);
    printf("%-12s bands sent: %2d\n", name, steps);
    save_ppm(dir, name);
}

/* ==================== 示例数据 ==================== */
static motor_state_t m;
static gray_state_t g;
static sonar_state_t t;
static openmv_state_t mv;

static void p_race(void)  { BspOled_ShowRaceScreen("COLOR_SCAN", 5234.0f, "R-G-B-Y", true, 'L', 245U, true, false); }
static void p_idle(void)  { BspOled_ShowRaceScreen("IDLE>FULL", 0.0f, "", false, 0, 0U, true, true); }
static void p_motor(void) { UiPage_MotorEnc(&m, 2U); }
static void p_cal(void)   { UiPage_MotionCal(&m, true, 1U); }
static void p_gray(void)  { UiPage_Gray(&g, false); }
static void p_sonar(void) { UiPage_Sonar(&t, true, true); }
static void p_mv(void)    { UiPage_Openmv(&mv, "COLOR", "TURN", true); }
static void p_line(void)  { UiPage_Line(&g, &m, true, "LEFT"); }

int main(int argc, char **argv)
{
    const char *dir = (argc > 1) ? argv[1] : ".";

    CHECK(BspOled_Init(), "BspOled_Init failed");
    CHECK(BspOled_IsOnline(), "not online after init");
    CHECK(colmod == 0x05, "COLMOD = %02X, want 05 (RGB565)", colmod);
    CHECK(madctl == (uint8_t)(CFG_TFT_MADCTL | (CFG_TFT_BGR ? 0x08 : 0)), "MADCTL = %02X", madctl);
    CHECK(disp_on, "DISPON not sent");
    CHECK(spi_while_cs_high == 0, "SPI bytes sent while CS high");
    for (int y = 0; y < H; ++y) for (int x = 0; x < W; ++x) {
        if (lcd[y][x] != 0) { CHECK(0, "screen not black after init at %d,%d", x, y); y = H; break; }
    }

    m.enc_online = true; m.battery_v = 7.4f;
    m.fwd_enc[0] = 12345; m.fwd_enc[1] = -234; m.fwd_enc[2] = 1234567; m.fwd_enc[3] = -9999999;
    m.total_dist_mm = 1234.5f; m.seg_dist_mm = 512.0f; m.seg_yaw_deg = -45.0f;
    m.wheel_spd_mm_s[0] = 300; m.wheel_spd_mm_s[1] = 310; m.wheel_spd_mm_s[2] = -290; m.wheel_spd_mm_s[3] = -300;
    const uint16_t raw[8] = { 300, 450, 1200, 3600, 3900, 2100, 600, 4095 };
    for (int i = 0; i < 8; ++i) { g.raw[i] = raw[i]; g.thresh[i] = 2000; }
    g.bits = 0x18; g.error = -0.75f; g.is_split = true; g.line_found = true;
    t.online = true; t.valid = true; t.dist_mm = 345; t.raw_mm = 350; t.echo_us = 2041; t.near_hits = 2; t.miss_count = 1;
    mv.online = true; mv.fps = 28; mv.target_task = OPENMV_TASK_COLOR; mv.ack_task = OPENMV_TASK_TURN;
    mv.turn_dir = 'L'; mv.park_seen = true; mv.park_cx = -35; mv.park_area = 1234; mv.bad_frame_count = 3;
    strcpy(mv.color_seq, "R-G-B");

    run_page(dir, "0_race",   p_race);
    run_page(dir, "1_idle",   p_idle);
    run_page(dir, "2_motor",  p_motor);
    run_page(dir, "3_cal",    p_cal);
    run_page(dir, "4_gray",   p_gray);
    run_page(dir, "5_sonar",  p_sonar);
    run_page(dir, "6_openmv", p_mv);
    run_page(dir, "7_line",   p_line);

    CHECK(out_of_window == 0, "%d pixels written outside the window", out_of_window);
    CHECK(spi_while_cs_high == 0, "SPI bytes sent while CS high");

    /* 只改一个数：只应重发受影响的条带 */
    p_sonar(); (void)flush_all("sonar-base");
    t.dist_mm = 346;
    p_sonar();
    int steps = flush_all("sonar-delta");
    CHECK(steps >= 1 && steps <= 4, "changing one number resent %d bands", steps);

    printf(g_fail ? "%d check(s) FAILED\n" : "all checks passed\n", g_fail);
    return g_fail ? 1 : 0;
}

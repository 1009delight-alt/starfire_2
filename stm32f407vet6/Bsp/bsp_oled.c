#include "bsp_oled.h"
#include "board.h"
#include "config.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ============================================================================
 *  1.8 寸 128x160 SPI TFT（ST7735S），横屏 160x128。
 *  SPI2：PB13 = SCK，PB15 = SDA(MOSI)；PB12 = CS，PB10 = A0(DC)，PB11 = RESET。
 *  屏只写不读（模块没有 MISO），因此 IsOnline 只表示初始化流程走完，不代表屏真的接好。
 *  帧缓冲 4 位调色板，16 条 8 行条带，只发送有变化的条带；
 *  每条带 160x8 像素 = 2560 字节 RGB565，SPI 10.5 MHz 下约 2 ms。
 *  阻塞发送，不用 DMA（缓冲区在普通 SRAM，不放 CCM）。
 * ============================================================================ */

#define BAND_H          8U
#define BAND_COUNT      (OLED_HEIGHT / BAND_H)
#define FB_STRIDE       (OLED_WIDTH / 2U)

static uint8_t s_fb[OLED_HEIGHT][FB_STRIDE];     /* 高半字节 = 偶数 x */
static uint8_t s_sent[OLED_HEIGHT][FB_STRIDE];   /* 屏上实际内容的影子副本 */
static uint8_t s_txbuf[OLED_WIDTH * BAND_H * 2U];
static bool    s_sent_valid = false;
static bool    s_online = false;
static uint8_t s_fg = OLED_C_WHITE;
static SPI_HandleTypeDef s_spi;

static const uint16_t k_palette[OLED_C_COUNT] = {
    0x0000U,  /* BLACK  */
    0xFFFFU,  /* WHITE  */
    0xF800U,  /* RED    */
    0x07E0U,  /* GREEN  */
    0x2B7FU,  /* BLUE（比纯蓝亮一些，黑底上看得清） */
    0xFFE0U,  /* YELLOW */
    0x07FFU,  /* CYAN   */
    0x8410U,  /* GRAY   */
    0xFD20U,  /* ORANGE */
};

/* 精简完整 5x7 可打印 ASCII 点阵（0x20 ' ' ~ 0x7E '~'） */
static const uint8_t k_font5x7[95][5] = {
    {0x00,0x00,0x00,0x00,0x00}, /*   */ {0x00,0x00,0x5F,0x00,0x00}, /* ! */
    {0x00,0x07,0x00,0x07,0x00}, /* " */ {0x14,0x7F,0x14,0x7F,0x14}, /* # */
    {0x24,0x2A,0x7F,0x2A,0x12}, /* $ */ {0x23,0x13,0x08,0x64,0x62}, /* % */
    {0x36,0x49,0x55,0x22,0x50}, /* & */ {0x00,0x05,0x03,0x00,0x00}, /* ' */
    {0x00,0x1C,0x22,0x41,0x00}, /* ( */ {0x00,0x41,0x22,0x1C,0x00}, /* ) */
    {0x14,0x08,0x3E,0x08,0x14}, /* * */ {0x08,0x08,0x3E,0x08,0x08}, /* + */
    {0x00,0x50,0x30,0x00,0x00}, /* , */ {0x08,0x08,0x08,0x08,0x08}, /* - */
    {0x00,0x60,0x60,0x00,0x00}, /* . */ {0x20,0x10,0x08,0x04,0x02}, /* / */
    {0x3E,0x51,0x49,0x45,0x3E}, /* 0 */ {0x00,0x42,0x7F,0x40,0x00}, /* 1 */
    {0x42,0x61,0x51,0x49,0x46}, /* 2 */ {0x21,0x41,0x45,0x4B,0x31}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10}, /* 4 */ {0x27,0x45,0x45,0x45,0x39}, /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30}, /* 6 */ {0x01,0x71,0x09,0x05,0x03}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36}, /* 8 */ {0x06,0x49,0x49,0x29,0x1E}, /* 9 */
    {0x00,0x36,0x36,0x00,0x00}, /* : */ {0x00,0x56,0x36,0x00,0x00}, /* ; */
    {0x08,0x14,0x22,0x41,0x00}, /* < */ {0x14,0x14,0x14,0x14,0x14}, /* = */
    {0x00,0x41,0x22,0x14,0x08}, /* > */ {0x02,0x01,0x51,0x09,0x06}, /* ? */
    {0x32,0x49,0x79,0x41,0x3E}, /* @ */ {0x7E,0x11,0x11,0x11,0x7E}, /* A */
    {0x7F,0x49,0x49,0x49,0x36}, /* B */ {0x3E,0x41,0x41,0x41,0x22}, /* C */
    {0x7F,0x41,0x41,0x22,0x1C}, /* D */ {0x7F,0x49,0x49,0x49,0x41}, /* E */
    {0x7F,0x09,0x09,0x09,0x01}, /* F */ {0x3E,0x41,0x49,0x49,0x7A}, /* G */
    {0x7F,0x08,0x08,0x08,0x7F}, /* H */ {0x00,0x41,0x7F,0x41,0x00}, /* I */
    {0x20,0x40,0x41,0x3F,0x01}, /* J */ {0x7F,0x08,0x14,0x22,0x41}, /* K */
    {0x7F,0x40,0x40,0x40,0x40}, /* L */ {0x7F,0x02,0x0C,0x02,0x7F}, /* M */
    {0x7F,0x04,0x08,0x10,0x7F}, /* N */ {0x3E,0x41,0x41,0x41,0x3E}, /* O */
    {0x7F,0x09,0x09,0x09,0x06}, /* P */ {0x3E,0x41,0x51,0x21,0x5E}, /* Q */
    {0x7F,0x09,0x19,0x29,0x46}, /* R */ {0x46,0x49,0x49,0x49,0x31}, /* S */
    {0x01,0x01,0x7F,0x01,0x01}, /* T */ {0x3F,0x40,0x40,0x40,0x3F}, /* U */
    {0x1F,0x20,0x40,0x20,0x1F}, /* V */ {0x3F,0x40,0x38,0x40,0x3F}, /* W */
    {0x63,0x14,0x08,0x14,0x63}, /* X */ {0x07,0x08,0x70,0x08,0x07}, /* Y */
    {0x61,0x51,0x49,0x45,0x43}, /* Z */ {0x00,0x7F,0x41,0x41,0x00}, /* [ */
    {0x02,0x04,0x08,0x10,0x20}, /* \ */ {0x00,0x41,0x41,0x7F,0x00}, /* ] */
    {0x04,0x02,0x01,0x02,0x04}, /* ^ */ {0x40,0x40,0x40,0x40,0x40}, /* _ */
    {0x00,0x01,0x02,0x04,0x00}, /* ` */ {0x20,0x54,0x54,0x54,0x78}, /* a */
    {0x7F,0x48,0x44,0x44,0x38}, /* b */ {0x38,0x44,0x44,0x44,0x20}, /* c */
    {0x38,0x44,0x44,0x48,0x7F}, /* d */ {0x38,0x54,0x54,0x54,0x18}, /* e */
    {0x08,0x7E,0x09,0x01,0x02}, /* f */ {0x0C,0x52,0x52,0x52,0x3E}, /* g */
    {0x7F,0x08,0x04,0x04,0x78}, /* h */ {0x00,0x44,0x7D,0x40,0x00}, /* i */
    {0x20,0x40,0x44,0x3D,0x00}, /* j */ {0x7F,0x10,0x28,0x44,0x00}, /* k */
    {0x00,0x41,0x7F,0x40,0x00}, /* l */ {0x7C,0x04,0x18,0x04,0x78}, /* m */
    {0x7C,0x08,0x04,0x04,0x78}, /* n */ {0x38,0x44,0x44,0x44,0x38}, /* o */
    {0x7C,0x14,0x14,0x14,0x08}, /* p */ {0x08,0x14,0x14,0x18,0x7C}, /* q */
    {0x7C,0x08,0x04,0x04,0x08}, /* r */ {0x48,0x54,0x54,0x54,0x20}, /* s */
    {0x04,0x3F,0x44,0x40,0x20}, /* t */ {0x3C,0x40,0x40,0x20,0x7C}, /* u */
    {0x1C,0x20,0x40,0x20,0x1C}, /* v */ {0x3C,0x40,0x30,0x40,0x3C}, /* w */
    {0x44,0x28,0x10,0x28,0x44}, /* x */ {0x0C,0x50,0x50,0x50,0x3C}, /* y */
    {0x44,0x64,0x54,0x4C,0x44}, /* z */ {0x00,0x08,0x36,0x41,0x00}, /* { */
    {0x00,0x00,0x7F,0x00,0x00}, /* | */ {0x00,0x41,0x36,0x08,0x00}, /* } */
    {0x10,0x08,0x08,0x10,0x08}  /* ~ */
};

/* ==================== SPI2 底层 ==================== */
static void lcd_cs(bool high)
{
    HAL_GPIO_WritePin(TFT_CS_PORT, TFT_CS_PIN, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void lcd_dc(bool data)
{
    HAL_GPIO_WritePin(TFT_DC_PORT, TFT_DC_PIN, data ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void lcd_rst(bool high)
{
    HAL_GPIO_WritePin(TFT_RST_PORT, TFT_RST_PIN, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static bool lcd_tx(const uint8_t *p, uint16_t n)
{
    return HAL_SPI_Transmit(&s_spi, (uint8_t *)p, n, CFG_TFT_SPI_TIMEOUT_MS) == HAL_OK;
}

static bool lcd_cmd(uint8_t c, const uint8_t *data, uint16_t n)
{
    bool ok;
    lcd_cs(false);
    lcd_dc(false);
    ok = lcd_tx(&c, 1U);
    if (ok && (n > 0U)) {
        lcd_dc(true);
        ok = lcd_tx(data, n);
    }
    lcd_cs(true);
    return ok;
}

static uint32_t spi_prescaler(void)
{
#if   (CFG_TFT_SPI_DIV == 2U)
    return SPI_BAUDRATEPRESCALER_2;      /* 21 MHz，超出 ST7735 标称，屏稳再用 */
#elif (CFG_TFT_SPI_DIV == 4U)
    return SPI_BAUDRATEPRESCALER_4;      /* 10.5 MHz */
#elif (CFG_TFT_SPI_DIV == 8U)
    return SPI_BAUDRATEPRESCALER_8;      /* 5.25 MHz，花屏时用 */
#else
#error "CFG_TFT_SPI_DIV must be 2, 4 or 8"
#endif
}

static bool lcd_set_window(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    uint8_t ca[4] = { 0U, (uint8_t)(x0 + CFG_TFT_X_OFFSET), 0U, (uint8_t)(x1 + CFG_TFT_X_OFFSET) };
    uint8_t ra[4] = { 0U, (uint8_t)(y0 + CFG_TFT_Y_OFFSET), 0U, (uint8_t)(y1 + CFG_TFT_Y_OFFSET) };
    bool ok = lcd_cmd(0x2AU, ca, 4U);            /* CASET */
    if (ok) { ok = lcd_cmd(0x2BU, ra, 4U); }     /* RASET */
    return ok;
}

static bool lcd_init_regs(void)
{
    static const uint8_t frmctr[]  = { 0x01U, 0x2CU, 0x2DU };
    static const uint8_t frmctr3[] = { 0x01U, 0x2CU, 0x2DU, 0x01U, 0x2CU, 0x2DU };
    static const uint8_t invctr[]  = { 0x07U };
    static const uint8_t pwctr1[]  = { 0xA2U, 0x02U, 0x84U };
    static const uint8_t pwctr2[]  = { 0xC5U };
    static const uint8_t pwctr3[]  = { 0x0AU, 0x00U };
    static const uint8_t pwctr4[]  = { 0x8AU, 0x2AU };
    static const uint8_t pwctr5[]  = { 0x8AU, 0xEEU };
    static const uint8_t vmctr1[]  = { 0x0EU };
    static const uint8_t colmod[]  = { 0x05U };  /* 16 位 RGB565 */
    static const uint8_t gmctrp[]  = { 0x02U, 0x1CU, 0x07U, 0x12U, 0x37U, 0x32U, 0x29U, 0x2DU,
                                       0x29U, 0x25U, 0x2BU, 0x39U, 0x00U, 0x01U, 0x03U, 0x10U };
    static const uint8_t gmctrn[]  = { 0x03U, 0x1DU, 0x07U, 0x06U, 0x2EU, 0x2CU, 0x29U, 0x2DU,
                                       0x2EU, 0x2EU, 0x37U, 0x3FU, 0x00U, 0x00U, 0x02U, 0x10U };
    uint8_t madctl = (uint8_t)(CFG_TFT_MADCTL | ((CFG_TFT_BGR != 0U) ? 0x08U : 0x00U));
    bool ok = true;

    ok = ok && lcd_cmd(0x01U, NULL, 0U);         /* SWRESET */
    HAL_Delay(150U);
    ok = ok && lcd_cmd(0x11U, NULL, 0U);         /* SLPOUT */
    HAL_Delay(120U);
    ok = ok && lcd_cmd(0xB1U, frmctr, sizeof(frmctr));
    ok = ok && lcd_cmd(0xB2U, frmctr, sizeof(frmctr));
    ok = ok && lcd_cmd(0xB3U, frmctr3, sizeof(frmctr3));
    ok = ok && lcd_cmd(0xB4U, invctr, sizeof(invctr));
    ok = ok && lcd_cmd(0xC0U, pwctr1, sizeof(pwctr1));
    ok = ok && lcd_cmd(0xC1U, pwctr2, sizeof(pwctr2));
    ok = ok && lcd_cmd(0xC2U, pwctr3, sizeof(pwctr3));
    ok = ok && lcd_cmd(0xC3U, pwctr4, sizeof(pwctr4));
    ok = ok && lcd_cmd(0xC4U, pwctr5, sizeof(pwctr5));
    ok = ok && lcd_cmd(0xC5U, vmctr1, sizeof(vmctr1));
    ok = ok && lcd_cmd((CFG_TFT_INVERT != 0U) ? 0x21U : 0x20U, NULL, 0U);  /* INVON / INVOFF */
    ok = ok && lcd_cmd(0x36U, &madctl, 1U);      /* MADCTL：方向与 RGB/BGR */
    ok = ok && lcd_cmd(0x3AU, colmod, sizeof(colmod));
    ok = ok && lcd_cmd(0xE0U, gmctrp, sizeof(gmctrp));
    ok = ok && lcd_cmd(0xE1U, gmctrn, sizeof(gmctrn));
    ok = ok && lcd_cmd(0x13U, NULL, 0U);         /* NORON */
    HAL_Delay(10U);
    ok = ok && lcd_cmd(0x29U, NULL, 0U);         /* DISPON */
    HAL_Delay(20U);
    return ok;
}

bool BspOled_Init(void)
{
    s_online = false;
    s_sent_valid = false;
    BspOled_Clear();

#if (CFG_ENABLE_OLED == 0U)
    return false;
#else
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();

    /* 控制脚先置无效电平再切输出，避免上电毛刺 */
    lcd_cs(true);
    lcd_dc(true);
    lcd_rst(true);
    gpio.Pin = TFT_CS_PIN | TFT_DC_PIN | TFT_RST_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = TFT_SCK_PIN | TFT_MOSI_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &gpio);

    s_spi.Instance = SPI2;
    s_spi.Init.Mode = SPI_MODE_MASTER;
    s_spi.Init.Direction = SPI_DIRECTION_2LINES;   /* MISO 不接，不配置引脚 */
    s_spi.Init.DataSize = SPI_DATASIZE_8BIT;
    s_spi.Init.CLKPolarity = SPI_POLARITY_LOW;
    s_spi.Init.CLKPhase = SPI_PHASE_1EDGE;
    s_spi.Init.NSS = SPI_NSS_SOFT;
    s_spi.Init.BaudRatePrescaler = spi_prescaler();
    s_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    s_spi.Init.TIMode = SPI_TIMODE_DISABLE;
    s_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    s_spi.Init.CRCPolynomial = 7U;
    if (HAL_SPI_Init(&s_spi) != HAL_OK) {
        return false;
    }

    /* 硬件复位 */
    HAL_Delay(5U);
    lcd_rst(false);
    HAL_Delay(20U);
    lcd_rst(true);
    HAL_Delay(120U);

    if (!lcd_init_regs()) {
        return false;
    }
    s_online = true;
    BspOled_Flush();      /* 整屏刷黑，清掉上电残影 */
    return true;
#endif
}

bool BspOled_IsOnline(void)
{
    return s_online;
}

void BspOled_Clear(void)
{
    memset(s_fb, 0, sizeof(s_fb));
    s_fg = OLED_C_WHITE;
}

void BspOled_SetColor(uint8_t color)
{
    s_fg = (color < (uint8_t)OLED_C_COUNT) ? color : (uint8_t)OLED_C_WHITE;
}

void BspOled_DrawPixel(uint8_t x, uint8_t y, bool on)
{
    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT)) {
        return;
    }
    uint8_t c = on ? s_fg : (uint8_t)OLED_C_BLACK;
    uint8_t *p = &s_fb[y][x >> 1];
    if ((x & 1U) == 0U) { *p = (uint8_t)((*p & 0x0FU) | (uint8_t)(c << 4)); }
    else                { *p = (uint8_t)((*p & 0xF0U) | c); }
}

void BspOled_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool fill)
{
    for (uint8_t dy = 0U; dy < h; ++dy) {
        for (uint8_t dx = 0U; dx < w; ++dx) {
            if (fill || (dx == 0U) || (dy == 0U) || (dx + 1U == w) || (dy + 1U == h)) {
                BspOled_DrawPixel((uint8_t)(x + dx), (uint8_t)(y + dy), true);
            }
        }
    }
}

void BspOled_DrawChar(uint8_t x, uint8_t y, char ch, uint8_t scale)
{
    if (scale == 0U) { scale = 1U; }
    uint8_t idx = ((ch >= 32) && (ch <= 126)) ? (uint8_t)(ch - 32) : 0U;
    for (uint8_t col = 0U; col < 5U; ++col) {
        uint8_t bits = k_font5x7[idx][col];
        for (uint8_t row = 0U; row < 7U; ++row) {
            if ((bits & (1U << row)) != 0U) {
                for (uint8_t sx = 0U; sx < scale; ++sx) {
                    for (uint8_t sy = 0U; sy < scale; ++sy) {
                        BspOled_DrawPixel((uint8_t)(x + col * scale + sx),
                                          (uint8_t)(y + row * scale + sy), true);
                    }
                }
            }
        }
    }
}

void BspOled_DrawStr(uint8_t x, uint8_t y, const char *str, uint8_t scale)
{
    if (str == NULL) { return; }
    if (scale == 0U) { scale = 1U; }
    uint8_t step = (uint8_t)(6U * scale);
    while (*str != '\0') {
        if ((uint16_t)x + (uint16_t)step > OLED_WIDTH) {
            break;
        }
        BspOled_DrawChar(x, y, *str, scale);
        x = (uint8_t)(x + step);
        str++;
    }
}

void BspOled_Printf(uint8_t x, uint8_t y, uint8_t scale, const char *fmt, ...)
{
    char buf[32];
    va_list ap;
    va_start(ap, fmt);
    (void)vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    BspOled_DrawStr(x, y, buf, scale);
}

void BspOled_Title(const char *text, uint8_t bg_color)
{
    BspOled_SetColor(bg_color);
    BspOled_DrawRect(0U, 0U, OLED_WIDTH, 18U, true);
    BspOled_SetColor(OLED_C_BLACK);
    BspOled_DrawStr(3U, 2U, text, 2U);
    BspOled_SetColor(OLED_C_WHITE);
}

void BspOled_Footer(const char *text)
{
    BspOled_SetColor(OLED_C_GRAY);
    BspOled_DrawStr(2U, 119U, text, 1U);
    BspOled_SetColor(OLED_C_WHITE);
}

void BspOled_DrawColorSeq(uint8_t y, const char *seq, uint8_t scale)
{
    if (scale == 0U) { scale = 1U; }
    if ((seq == NULL) || (seq[0] == '\0')) { seq = "----"; }
    uint16_t step = (uint16_t)(6U * scale);
    size_t slen = strlen(seq);
    size_t max_len = OLED_WIDTH / step;
    if (slen > max_len) { slen = max_len; }
    uint8_t x = (uint8_t)((OLED_WIDTH - slen * step) / 2U + scale / 2U);
    /* 字母一律白色：屏的色序与字母颜色对不上（2026-11-26 用户要求不再按颜色画） */
    BspOled_SetColor(OLED_C_WHITE);
    for (size_t i = 0U; i < slen; ++i) {
        BspOled_DrawChar(x, y, seq[i], scale);
        x = (uint8_t)(x + step);
    }
    BspOled_SetColor(OLED_C_WHITE);
}

static bool band_dirty(uint8_t band)
{
    return (!s_sent_valid) ||
           (memcmp(s_sent[band * BAND_H], s_fb[band * BAND_H], BAND_H * FB_STRIDE) != 0);
}

static void send_band(uint8_t band)
{
    uint8_t y0 = (uint8_t)(band * BAND_H);
    uint8_t *o = s_txbuf;

    for (uint8_t r = 0U; r < BAND_H; ++r) {
        const uint8_t *src = s_fb[y0 + r];
        for (uint8_t i = 0U; i < FB_STRIDE; ++i) {
            uint16_t c0 = k_palette[src[i] >> 4];
            uint16_t c1 = k_palette[src[i] & 0x0FU];
            *o++ = (uint8_t)(c0 >> 8); *o++ = (uint8_t)c0;
            *o++ = (uint8_t)(c1 >> 8); *o++ = (uint8_t)c1;
        }
    }

    bool ok = lcd_set_window(0U, y0, (uint8_t)(OLED_WIDTH - 1U), (uint8_t)(y0 + BAND_H - 1U));
    if (ok) {
        uint8_t c = 0x2CU;                       /* RAMWR */
        lcd_cs(false);
        lcd_dc(false);
        ok = lcd_tx(&c, 1U);
        lcd_dc(true);
        if (ok) { ok = lcd_tx(s_txbuf, (uint16_t)sizeof(s_txbuf)); }
        lcd_cs(true);
    }
    if (ok) {   /* 发送失败就不更新影子，下次重发 */
        memcpy(s_sent[y0], s_fb[y0], BAND_H * FB_STRIDE);
    }
}

/* 阻塞发送所有有变化的条带（整屏 16 条带约 35 ms）。只在初始化时用；主循环里一律用 FlushStep */
void BspOled_Flush(void)
{
    if (!s_online) {
        return;
    }
    if (!s_sent_valid) {
        memset(s_sent, 0xA5, sizeof(s_sent));   /* 强制首次全部重发 */
        s_sent_valid = true;
    }
    for (uint8_t band = 0U; band < BAND_COUNT; ++band) {
        if (band_dirty(band)) {
            send_band(band);
        }
    }
}

/* 比赛模式用：每次最多发送一条有变化的条带（约 2 ms），限制单次阻塞时间。
 * 返回 true 表示屏上内容已与帧缓冲一致。 */
bool BspOled_FlushStep(void)
{
    if (!s_online) {
        return true;
    }
    if (!s_sent_valid) {
        memset(s_sent, 0xA5, sizeof(s_sent));
        s_sent_valid = true;
    }
    for (uint8_t band = 0U; band < BAND_COUNT; ++band) {
        if (band_dirty(band)) {
            send_band(band);
            return false;
        }
    }
    return true;
}

void BspOled_ShowRaceScreen(const char *stage_name,
                            float total_dist_mm,
                            const char *color_seq,
                            bool color_locked,
                            char turn_dir,
                            uint16_t sonar_mm,
                            bool enc_ok,
                            bool mv_ok)
{
    if (!s_online) {
        return;
    }
    BspOled_Clear();

    /* 第 1 行：赛段名 + 在线状态指示 (E=编码器 M=OpenMV，绿色在线 / 红色 x 离线) */
    BspOled_Printf(2U, 3U, 1U, "%-20.20s", (stage_name != NULL) ? stage_name : "");
    BspOled_SetColor(enc_ok ? OLED_C_GREEN : OLED_C_RED);
    BspOled_DrawChar(140U, 3U, enc_ok ? 'E' : 'x', 1U);
    BspOled_SetColor(mv_ok ? OLED_C_GREEN : OLED_C_RED);
    BspOled_DrawChar(150U, 3U, mv_ok ? 'M' : 'x', 1U);

    /* 第 2 行：里程(cm) + 转向结果 + 超声距离(mm) */
    BspOled_SetColor(OLED_C_WHITE);
    BspOled_Printf(2U, 15U, 1U, "D:%4dcm  T:%c  S:%3dmm",
                   (int)(total_dist_mm / 10.0f),
                   (turn_dir != 0) ? turn_dir : '-',
                   (sonar_mm > 999U) ? 999 : (int)sonar_mm);

    /* 分割线 */
    BspOled_SetColor(OLED_C_GRAY);
    BspOled_DrawRect(0U, 26U, OLED_WIDTH, 1U, true);

    /* 四色标题 */
    BspOled_SetColor(OLED_C_WHITE);
    BspOled_DrawStr(2U, 32U, "COLOR ORDER:", 1U);
    if (color_locked) {
        BspOled_SetColor(OLED_C_GREEN);
        BspOled_DrawStr(80U, 32U, "[LOCK]", 1U);
    }

    /* 中下方 3 倍字号彩色四色结果（每字符 18x24），供裁判拍照计分；字母用对应颜色 */
    BspOled_SetColor(OLED_C_GRAY);
    BspOled_DrawRect(2U, 44U, 156U, 80U, false);

    BspOled_DrawColorSeq(73U, color_seq, 3U);
    /* 只更新帧缓冲；由主循环每 20ms 调用 BspOled_FlushStep() 逐条带发送 */
}

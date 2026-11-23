#include "bsp_tof.h"
#include "board.h"
#include "config.h"

#include <string.h>

#define VL53L0X_ADDR_7B                     0x29U
#define VL53L0X_REG_SYSRANGE_START          0x00U
#define VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG  0x01U
#define VL53L0X_REG_SYSTEM_INTERMEAS_PERIOD 0x04U
#define VL53L0X_REG_SYSTEM_INT_CFG_GPIO     0x0AU
#define VL53L0X_REG_SYSTEM_INT_CLEAR        0x0BU
#define VL53L0X_REG_RESULT_INT_STATUS       0x13U
#define VL53L0X_REG_RESULT_RANGE_STATUS     0x14U
#define VL53L0X_REG_FINAL_RANGE_MIN_CNT_RT  0x44U
#define VL53L0X_REG_MSRC_CONFIG_CONTROL     0x60U
#define VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH 0x84U
#define VL53L0X_REG_VHV_CFG_PAD_SCL_SDA_EXT 0x89U
#define VL53L0X_REG_GLOBAL_CFG_SPAD_EN_0    0xB0U
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID 0xC0U

static tof_state_t s_tof;
static uint8_t  s_stop_variable = 0U;
static uint16_t s_med_buf[3] = { 2000U, 2000U, 2000U };
static uint8_t  s_med_idx = 0U;

/* ==================== PB6/PB7 软件 I2C ==================== */
static void tof_scl(bool high)
{
    HAL_GPIO_WritePin(TOF_I2C_SCL_PORT, TOF_I2C_SCL_PIN, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void tof_sda(bool high)
{
    HAL_GPIO_WritePin(TOF_I2C_SDA_PORT, TOF_I2C_SDA_PIN, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static bool tof_sda_read(void)
{
    return (HAL_GPIO_ReadPin(TOF_I2C_SDA_PORT, TOF_I2C_SDA_PIN) == GPIO_PIN_SET);
}

static void tof_i2c_delay(void)
{
    Board_DelayUs(3U);
}

static void tof_i2c_init_pins(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();

    tof_scl(true);
    tof_sda(true);
    gpio.Pin = TOF_I2C_SCL_PIN | TOF_I2C_SDA_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* 9 个恢复脉冲，释放可能被异常复位锁住的 SDA */
    for (int i = 0; i < 9; ++i) {
        tof_scl(false); tof_i2c_delay();
        tof_scl(true);  tof_i2c_delay();
    }
    tof_sda(true);
}

static void tof_i2c_start(void)
{
    tof_sda(true);  tof_scl(true);  tof_i2c_delay();
    tof_sda(false); tof_i2c_delay();
    tof_scl(false); tof_i2c_delay();
}

static void tof_i2c_stop(void)
{
    tof_sda(false); tof_i2c_delay();
    tof_scl(true);  tof_i2c_delay();
    tof_sda(true);  tof_i2c_delay();
}

static bool tof_i2c_write_byte(uint8_t byte)
{
    for (int i = 7; i >= 0; --i) {
        tof_sda(((byte >> i) & 1U) != 0U);
        tof_i2c_delay();
        tof_scl(true);  tof_i2c_delay();
        tof_scl(false); tof_i2c_delay();
    }
    tof_sda(true);
    tof_i2c_delay();
    tof_scl(true);
    tof_i2c_delay();
    bool ack = !tof_sda_read();
    tof_scl(false);
    tof_i2c_delay();
    return ack;
}

static uint8_t tof_i2c_read_byte(bool send_ack)
{
    uint8_t val = 0U;
    tof_sda(true);
    for (int i = 7; i >= 0; --i) {
        tof_scl(true);
        tof_i2c_delay();
        if (tof_sda_read()) {
            val |= (uint8_t)(1U << i);
        }
        tof_scl(false);
        tof_i2c_delay();
    }
    tof_sda(!send_ack);
    tof_i2c_delay();
    tof_scl(true);  tof_i2c_delay();
    tof_scl(false); tof_i2c_delay();
    tof_sda(true);
    return val;
}

static bool vl_write_reg(uint8_t reg, uint8_t val)
{
    bool ok;
    tof_i2c_start();
    ok = tof_i2c_write_byte((uint8_t)(VL53L0X_ADDR_7B << 1));
    if (ok) { ok = tof_i2c_write_byte(reg); }
    if (ok) { ok = tof_i2c_write_byte(val); }
    tof_i2c_stop();
    return ok;
}

static bool vl_write_reg16(uint8_t reg, uint16_t val)
{
    bool ok;
    tof_i2c_start();
    ok = tof_i2c_write_byte((uint8_t)(VL53L0X_ADDR_7B << 1));
    if (ok) { ok = tof_i2c_write_byte(reg); }
    if (ok) { ok = tof_i2c_write_byte((uint8_t)(val >> 8)); }
    if (ok) { ok = tof_i2c_write_byte((uint8_t)(val & 0xFFU)); }
    tof_i2c_stop();
    return ok;
}

static bool vl_write_multi(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    bool ok;
    tof_i2c_start();
    ok = tof_i2c_write_byte((uint8_t)(VL53L0X_ADDR_7B << 1));
    if (ok) { ok = tof_i2c_write_byte(reg); }
    for (uint8_t i = 0U; ok && (i < len); ++i) {
        ok = tof_i2c_write_byte(buf[i]);
    }
    tof_i2c_stop();
    return ok;
}

static bool vl_read_reg(uint8_t reg, uint8_t *val)
{
    bool ok;
    tof_i2c_start();
    ok = tof_i2c_write_byte((uint8_t)(VL53L0X_ADDR_7B << 1));
    if (ok) { ok = tof_i2c_write_byte(reg); }
    if (ok) {
        tof_i2c_start();
        ok = tof_i2c_write_byte((uint8_t)((VL53L0X_ADDR_7B << 1) | 1U));
        if (ok) {
            *val = tof_i2c_read_byte(false);
        }
    }
    tof_i2c_stop();
    return ok;
}

static bool vl_read_multi(uint8_t reg, uint8_t *buf, uint8_t len)
{
    bool ok;
    tof_i2c_start();
    ok = tof_i2c_write_byte((uint8_t)(VL53L0X_ADDR_7B << 1));
    if (ok) { ok = tof_i2c_write_byte(reg); }
    if (ok) {
        tof_i2c_start();
        ok = tof_i2c_write_byte((uint8_t)((VL53L0X_ADDR_7B << 1) | 1U));
        if (ok) {
            for (uint8_t i = 0U; i < len; ++i) {
                buf[i] = tof_i2c_read_byte((i + 1U) < len);
            }
        }
    }
    tof_i2c_stop();
    return ok;
}

static bool vl_perform_single_ref_cal(uint8_t vhv_init_byte)
{
    if (!vl_write_reg(VL53L0X_REG_SYSRANGE_START, (uint8_t)(0x01U | vhv_init_byte))) {
        return false;
    }
    uint32_t t0 = HAL_GetTick();
    uint8_t status = 0U;
    do {
        if (!vl_read_reg(VL53L0X_REG_RESULT_INT_STATUS, &status)) {
            return false;
        }
        if ((uint32_t)(HAL_GetTick() - t0) > 150U) {
            return false;
        }
    } while ((status & 0x07U) == 0U);

    if (!vl_write_reg(VL53L0X_REG_SYSTEM_INT_CLEAR, 0x01U)) { return false; }
    if (!vl_write_reg(VL53L0X_REG_SYSRANGE_START, 0x00U))   { return false; }
    return true;
}

static bool vl_get_spad_info(uint8_t *count, bool *type_is_aperture)
{
    uint8_t tmp = 0U;
    uint32_t t0 = HAL_GetTick();

    if (!vl_write_reg(0x80U, 0x01U)) { return false; }
    if (!vl_write_reg(0xFFU, 0x01U)) { return false; }
    if (!vl_write_reg(0x00U, 0x00U)) { return false; }
    if (!vl_write_reg(0xFFU, 0x06U)) { return false; }
    if (!vl_read_reg(0x83U, &tmp))   { return false; }
    if (!vl_write_reg(0x83U, (uint8_t)(tmp | 0x04U))) { return false; }
    if (!vl_write_reg(0xFFU, 0x07U)) { return false; }
    if (!vl_write_reg(0x81U, 0x01U)) { return false; }
    if (!vl_write_reg(0x80U, 0x01U)) { return false; }
    if (!vl_write_reg(0x94U, 0x6BU)) { return false; }
    if (!vl_write_reg(0x83U, 0x00U)) { return false; }

    do {
        if (!vl_read_reg(0x83U, &tmp)) { return false; }
        if ((uint32_t)(HAL_GetTick() - t0) > 100U) { return false; }
    } while (tmp == 0U);

    if (!vl_write_reg(0x83U, 0x01U)) { return false; }
    if (!vl_read_reg(0x92U, &tmp))   { return false; }
    *count = (uint8_t)(tmp & 0x7FU);
    *type_is_aperture = (((tmp >> 7) & 0x01U) != 0U);

    if (!vl_write_reg(0x81U, 0x00U)) { return false; }
    if (!vl_write_reg(0xFFU, 0x06U)) { return false; }
    if (!vl_read_reg(0x83U, &tmp))   { return false; }
    if (!vl_write_reg(0x83U, (uint8_t)(tmp & (uint8_t)~0x04U))) { return false; }
    if (!vl_write_reg(0xFFU, 0x01U)) { return false; }
    if (!vl_write_reg(0x00U, 0x01U)) { return false; }
    if (!vl_write_reg(0xFFU, 0x00U)) { return false; }
    if (!vl_write_reg(0x80U, 0x00U)) { return false; }
    return true;
}

static bool vl_load_default_tuning(void)
{
    static const uint8_t k_pairs[][2] = {
        {0xFF,0x01},{0x00,0x00},{0xFF,0x00},{0x09,0x00},{0x10,0x00},{0x11,0x00},
        {0x24,0x01},{0x25,0xFF},{0x75,0x00},{0xFF,0x01},{0x4E,0x2C},{0x48,0x00},
        {0x30,0x20},{0xFF,0x00},{0x30,0x09},{0x54,0x00},{0x31,0x04},{0x32,0x03},
        {0x40,0x83},{0x46,0x25},{0x60,0x00},{0x27,0x00},{0x50,0x06},{0x51,0x00},
        {0x52,0x96},{0x56,0x08},{0x57,0x30},{0x61,0x00},{0x62,0x00},{0x64,0x00},
        {0x65,0x00},{0x66,0xA0},{0xFF,0x01},{0x22,0x32},{0x47,0x14},{0x49,0xFF},
        {0x4A,0x00},{0xFF,0x00},{0x7A,0x0A},{0x7B,0x00},{0x78,0x21},{0xFF,0x01},
        {0x23,0x34},{0x42,0x00},{0x44,0xFF},{0x45,0x26},{0x46,0x05},{0x40,0x40},
        {0x0E,0x06},{0x20,0x1A},{0x43,0x40},{0xFF,0x00},{0x34,0x03},{0x35,0x44},
        {0xFF,0x01},{0x31,0x04},{0x4B,0x09},{0x4C,0x05},{0x4D,0x04},{0xFF,0x00},
        {0x44,0x00},{0x45,0x20},{0x47,0x08},{0x48,0x28},{0x67,0x00},{0x70,0x04},
        {0x71,0x01},{0x72,0xFE},{0x76,0x00},{0x77,0x00},{0xFF,0x01},{0x0D,0x01},
        {0xFF,0x00},{0x80,0x01},{0x01,0xF8},{0xFF,0x01},{0x8E,0x01},{0x00,0x01},
        {0xFF,0x00},{0x80,0x00}
    };
    for (size_t i = 0U; i < (sizeof(k_pairs) / sizeof(k_pairs[0])); ++i) {
        if (!vl_write_reg(k_pairs[i][0], k_pairs[i][1])) {
            return false;
        }
    }
    return true;
}

static uint16_t median3(uint16_t a, uint16_t b, uint16_t c)
{
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) { uint16_t t = b; b = c; c = t; }
    if (a > b) { uint16_t t = a; a = b; b = t; }
    return b;
}

bool BspTof_Init(void)
{
    memset(&s_tof, 0, sizeof(s_tof));
    s_tof.dist_mm = 2000U;
    s_tof.raw_mm = 2000U;
    /* 中值缓冲初值取“远处”，避免上电前两帧中值为 0 被当成近距障碍 */
    s_med_buf[0] = s_med_buf[1] = s_med_buf[2] = 2000U;
    s_med_idx = 0U;

#if (CFG_ENABLE_TOF == 0U)
    return false;
#else
    tof_i2c_init_pins();
    HAL_Delay(5U);

    uint8_t model_id = 0U;
    if (!vl_read_reg(VL53L0X_REG_IDENTIFICATION_MODEL_ID, &model_id) || (model_id != 0xEEU)) {
        return false;
    }

    /* 2.8V I/O 模式 */
    uint8_t val = 0U;
    if (!vl_read_reg(VL53L0X_REG_VHV_CFG_PAD_SCL_SDA_EXT, &val)) { return false; }
    if (!vl_write_reg(VL53L0X_REG_VHV_CFG_PAD_SCL_SDA_EXT, (uint8_t)(val | 0x01U))) { return false; }

    /* 标准 I2C 模式并读取 stop_variable */
    if (!vl_write_reg(0x88U, 0x00U)) { return false; }
    if (!vl_write_reg(0x80U, 0x01U)) { return false; }
    if (!vl_write_reg(0xFFU, 0x01U)) { return false; }
    if (!vl_write_reg(0x00U, 0x00U)) { return false; }
    if (!vl_read_reg(0x91U, &s_stop_variable)) { return false; }
    if (!vl_write_reg(0x00U, 0x01U)) { return false; }
    if (!vl_write_reg(0xFFU, 0x00U)) { return false; }
    if (!vl_write_reg(0x80U, 0x00U)) { return false; }

    /* 关闭 SIGNAL_RATE_MSRC 与 SIGNAL_RATE_PRE_RANGE 限值，设 0.25 MCPS 门限 */
    if (!vl_read_reg(VL53L0X_REG_MSRC_CONFIG_CONTROL, &val)) { return false; }
    if (!vl_write_reg(VL53L0X_REG_MSRC_CONFIG_CONTROL, (uint8_t)(val | 0x12U))) { return false; }
    if (!vl_write_reg16(VL53L0X_REG_FINAL_RANGE_MIN_CNT_RT, 32U)) { return false; } /* 0.25 * 128 */
    if (!vl_write_reg(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xFFU)) { return false; }

    /* SPAD 标定 */
    uint8_t spad_count = 0U;
    bool spad_is_aperture = false;
    if (!vl_get_spad_info(&spad_count, &spad_is_aperture)) { return false; }

    uint8_t ref_spad_map[6];
    if (!vl_read_multi(VL53L0X_REG_GLOBAL_CFG_SPAD_EN_0, ref_spad_map, 6U)) { return false; }
    if (!vl_write_reg(0xFFU, 0x01U)) { return false; }
    if (!vl_write_reg(0x4FU, 0x00U)) { return false; }
    if (!vl_write_reg(0x4EU, 0x2CU)) { return false; }
    if (!vl_write_reg(0xFFU, 0x00U)) { return false; }
    if (!vl_write_reg(0xB6U, 0xB4U)) { return false; }

    uint8_t first_spad = spad_is_aperture ? 12U : 0U;
    uint8_t enabled = 0U;
    for (uint8_t i = 0U; i < 48U; ++i) {
        if ((i < first_spad) || (enabled == spad_count)) {
            ref_spad_map[i / 8U] &= (uint8_t)~(1U << (i % 8U));
        } else if (((ref_spad_map[i / 8U] >> (i % 8U)) & 0x01U) != 0U) {
            enabled++;
        }
    }
    if (!vl_write_multi(VL53L0X_REG_GLOBAL_CFG_SPAD_EN_0, ref_spad_map, 6U)) { return false; }

    /* 默认 Tuning 参数 */
    if (!vl_load_default_tuning()) { return false; }

    /* GPIO 新样本就绪中断（低电平有效） */
    if (!vl_write_reg(VL53L0X_REG_SYSTEM_INT_CFG_GPIO, 0x04U)) { return false; }
    if (!vl_read_reg(VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH, &val)) { return false; }
    if (!vl_write_reg(VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH, (uint8_t)(val & (uint8_t)~0x10U))) { return false; }
    if (!vl_write_reg(VL53L0X_REG_SYSTEM_INT_CLEAR, 0x01U)) { return false; }

    /* 使能 DSS + Pre-Range + Final-Range，执行 VHV & Phase 校准 */
    if (!vl_write_reg(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xE8U)) { return false; }
    if (!vl_write_reg(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x01U)) { return false; }
    if (!vl_perform_single_ref_cal(0x40U)) { return false; }
    if (!vl_write_reg(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x02U)) { return false; }
    if (!vl_perform_single_ref_cal(0x00U)) { return false; }
    if (!vl_write_reg(VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xE8U)) { return false; }

    /* 启动连续背靠背测距（~33ms 一帧，主循环非阻塞读取） */
    if (!vl_write_reg(0x80U, 0x01U)) { return false; }
    if (!vl_write_reg(0xFFU, 0x01U)) { return false; }
    if (!vl_write_reg(0x00U, 0x00U)) { return false; }
    if (!vl_write_reg(0x91U, s_stop_variable)) { return false; }
    if (!vl_write_reg(0x00U, 0x01U)) { return false; }
    if (!vl_write_reg(0xFFU, 0x00U)) { return false; }
    if (!vl_write_reg(0x80U, 0x00U)) { return false; }
    if (!vl_write_reg(VL53L0X_REG_SYSRANGE_START, 0x02U)) { return false; }

    s_tof.online = true;
    s_tof.last_update_ms = HAL_GetTick();
    return true;
#endif
}

void BspTof_Update(uint32_t now_ms)
{
    if (!s_tof.online) {
        return;
    }
    uint8_t status = 0U;
    if (!vl_read_reg(VL53L0X_REG_RESULT_INT_STATUS, &status)) {
        return;
    }
    if ((status & 0x07U) == 0U) {
        return;
    }

    uint8_t data[2] = {0};
    if (vl_read_multi((uint8_t)(VL53L0X_REG_RESULT_RANGE_STATUS + 10U), data, 2U)) {
        uint16_t mm = (uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
        (void)vl_write_reg(VL53L0X_REG_SYSTEM_INT_CLEAR, 0x01U);

        s_tof.raw_mm = mm;
        s_tof.last_update_ms = now_ms;

        if ((mm >= 20U) && (mm <= 1800U)) {
            s_med_buf[s_med_idx] = mm;
            s_med_idx = (uint8_t)((s_med_idx + 1U) % 3U);
            s_tof.dist_mm = median3(s_med_buf[0], s_med_buf[1], s_med_buf[2]);
            s_tof.valid = true;

            if (s_tof.dist_mm <= CFG_OBS_TRIG_DIST_MM) {
                if (s_tof.near_hits < 255U) { s_tof.near_hits++; }
            } else {
                s_tof.near_hits = 0U;
            }
        } else {
            s_tof.valid = false;
            s_tof.dist_mm = 2000U;
            s_tof.near_hits = 0U;
        }
    }
}

void BspTof_ResetTrigger(void)
{
    s_tof.near_hits = 0U;
}

/* 在线且最近 CFG_TOF_STALE_MS 内有新读数（有效或无效均可，说明传感器在工作） */
bool BspTof_IsHealthy(uint32_t now_ms)
{
    return s_tof.online && ((uint32_t)(now_ms - s_tof.last_update_ms) <= CFG_TOF_STALE_MS);
}

bool BspTof_IsObstacleDetected(uint32_t now_ms)
{
    return BspTof_IsHealthy(now_ms) && s_tof.valid && (s_tof.near_hits >= CFG_OBS_CONFIRM_FRAMES);
}

const tof_state_t *BspTof_GetState(void)
{
    return &s_tof;
}

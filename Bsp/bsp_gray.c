#include "bsp_gray.h"
#include "gray_logic.h"
#include "board.h"
#include "config.h"

#include <string.h>

static ADC_HandleTypeDef s_hadc1;
static gray_state_t s_gray;
static uint8_t  s_inited = 0U;

static uint16_t s_cal_min[8];
static uint16_t s_cal_max[8];

static void select_mux_channel(uint8_t ch)
{
    HAL_GPIO_WritePin(GRAY_AD0_PORT, GRAY_AD0_PIN, ((ch & 0x01U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GRAY_AD1_PORT, GRAY_AD1_PIN, ((ch & 0x02U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GRAY_AD2_PORT, GRAY_AD2_PIN, ((ch & 0x04U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint16_t read_adc_avg4(void)
{
    uint32_t sum = 0U;
    for (int i = 0; i < 4; ++i) {
        if (HAL_ADC_Start(&s_hadc1) == HAL_OK) {
            if (HAL_ADC_PollForConversion(&s_hadc1, 2U) == HAL_OK) {
                sum += HAL_ADC_GetValue(&s_hadc1);
            }
            (void)HAL_ADC_Stop(&s_hadc1);
        }
    }
    return (uint16_t)(sum >> 2);
}

void BspGray_Init(void)
{
    static const uint16_t k_def_th[8] = CFG_GRAY_TH_DEFAULT;
    memset(&s_gray, 0, sizeof(s_gray));
    memcpy(s_gray.thresh, k_def_th, sizeof(s_gray.thresh));

#if (CFG_ENABLE_GRAY == 0U)
    return;
#else
    GPIO_InitTypeDef gpio = {0};
    ADC_ChannelConfTypeDef ch_cfg = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();

    /* PE2/PE3/PE4 通道选择 AD0~AD2 */
    gpio.Pin = GRAY_AD0_PIN | GRAY_AD1_PIN | GRAY_AD2_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOE, &gpio);
    select_mux_channel(0U);

    /* PC0 模拟输入 ADC1_IN10 */
    gpio.Pin = GRAY_ADC_PIN;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GRAY_ADC_PORT, &gpio);

    /* ADC1: PCLK2=84MHz / 4 = 21MHz, 12-bit 单次软件触发 */
    s_hadc1.Instance = ADC1;
    s_hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    s_hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    s_hadc1.Init.ScanConvMode = DISABLE;
    s_hadc1.Init.ContinuousConvMode = DISABLE;
    s_hadc1.Init.DiscontinuousConvMode = DISABLE;
    s_hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    s_hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    s_hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    s_hadc1.Init.NbrOfConversion = 1U;
    s_hadc1.Init.DMAContinuousRequests = DISABLE;
    s_hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&s_hadc1) != HAL_OK) {
        Error_Handler();
    }

    ch_cfg.Channel = ADC_CHANNEL_10;
    ch_cfg.Rank = 1U;
    ch_cfg.SamplingTime = ADC_SAMPLETIME_56CYCLES;
    if (HAL_ADC_ConfigChannel(&s_hadc1, &ch_cfg) != HAL_OK) {
        Error_Handler();
    }

    s_inited = 1U;
#endif
}

void BspGray_Sample(gray_bias_mode_t bias_mode)
{
    if (s_inited == 0U) {
        return;
    }

    uint16_t raw_hw[8];
    for (uint8_t hw = 0U; hw < 8U; ++hw) {
        select_mux_channel(hw);
        Board_DelayUs(CFG_GRAY_SETTLE_US);
        raw_hw[hw] = read_adc_avg4();
    }
    /* 判线计算在 gray_logic.c，电脑回放直接测它 */
    GrayLogic_Process(&s_gray, raw_hw, bias_mode,
                      CFG_GRAY_BLACK_IS_LOW != 0U, CFG_GRAY_REVERSE_ORDER != 0U,
                      (uint8_t)CFG_GRAY_DEAD_MASK);
}

const gray_state_t *BspGray_GetState(void)
{
    return &s_gray;
}

void BspGray_CalibStart(void)
{
    for (int i = 0; i < 8; ++i) {
        s_cal_min[i] = 4095U;
        s_cal_max[i] = 0U;
    }
}

void BspGray_CalibFeed(void)
{
    for (int i = 0; i < 8; ++i) {
        uint16_t v = s_gray.raw[i];
        if (v < s_cal_min[i]) { s_cal_min[i] = v; }
        if (v > s_cal_max[i]) { s_cal_max[i] = v; }
    }
}

void BspGray_CalibFinishAndApply(void)
{
    for (int i = 0; i < 8; ++i) {
        if (s_cal_max[i] > (uint16_t)(s_cal_min[i] + 150U)) {
            s_gray.thresh[i] = (uint16_t)(((uint32_t)s_cal_min[i] + (uint32_t)s_cal_max[i]) >> 1);
        }
    }
}

#include "board.h"
#include "bsp_motor.h"

/* 按键状态机变量 */
static uint8_t  s_key_stable = 0U;
static uint32_t s_key_change_ms = 0U;
static uint32_t s_key_press_ms = 0U;
static uint8_t  s_key_long_fired = 0U;
static board_key_event_t s_key_event = KEY_EVT_NONE;

/* 启动防误触：上电或复位后必须检测到松开才允许下次按下触发 */
static uint8_t  s_start_armed = 0U;
static uint32_t s_start_deb_ms = 0U;
static uint8_t  s_start_last_raw = 1U;

/* LED 状态模式 */
static board_led_pattern_t s_led_pat = LED_PAT_OFF;

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* SkyStar: 8 MHz HSE -> 168 MHz SYSCLK, APB1=42MHz, APB2=84MHz */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 8;
    osc.PLL.PLLN = 336;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        Error_Handler();
    }

    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                    RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
    SystemCoreClockUpdate();
}

static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void Board_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    HAL_Init();
    SystemClock_Config();
    DWT_Init();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* 板载 LED PB2 */
    HAL_GPIO_WritePin(BOARD_LED_PORT, BOARD_LED_PIN, GPIO_PIN_RESET);
    gpio.Pin = BOARD_LED_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BOARD_LED_PORT, &gpio);

    /* 板载 KEY PA0 */
    gpio.Pin = BOARD_KEY_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(BOARD_KEY_PORT, &gpio);

    Board_KeyResetStartArm();
}

void Board_WatchdogStart(uint16_t reload_ms)
{
    /* 调试器暂停内核时冻结看门狗，避免断点调试被复位 */
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
    IWDG->KR  = 0x5555U;                    /* 解锁 PR/RLR */
    IWDG->PR  = 3U;                         /* LSI/32 ≈ 1 kHz */
    IWDG->RLR = (reload_ms > 4095U) ? 4095U : reload_ms;
    IWDG->KR  = 0xAAAAU;                    /* 装载 */
    IWDG->KR  = 0xCCCCU;                    /* 启动，之后无法关闭 */
}

void Board_WatchdogKick(void)
{
    IWDG->KR = 0xAAAAU;
}

bool Board_WasWatchdogReset(void)
{
    bool wd = ((RCC->CSR & RCC_CSR_IWDGRSTF) != 0U);
    RCC->CSR |= RCC_CSR_RMVF;
    return wd;
}

void Board_DelayUs(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((uint32_t)(DWT->CYCCNT - start) < ticks) {
        __NOP();
    }
}

bool Board_KeyIsPressed(void)
{
    return (HAL_GPIO_ReadPin(BOARD_KEY_PORT, BOARD_KEY_PIN) == GPIO_PIN_SET);
}

void Board_KeyUpdate(uint32_t now_ms)
{
    uint8_t raw = Board_KeyIsPressed() ? 1U : 0U;
    static uint8_t prev_raw = 0U;

    if (raw != prev_raw) {
        prev_raw = raw;
        s_key_change_ms = now_ms;
    }

    if ((uint32_t)(now_ms - s_key_change_ms) >= 25U) {
        if (raw != s_key_stable) {
            s_key_stable = raw;
            if (s_key_stable != 0U) {
                s_key_press_ms = now_ms;
                s_key_long_fired = 0U;
            } else {
                if (s_key_long_fired == 0U) {
                    s_key_event = KEY_EVT_SHORT;
                }
            }
        } else if ((s_key_stable != 0U) && (s_key_long_fired == 0U) &&
                   ((uint32_t)(now_ms - s_key_press_ms) >= 1000U)) {
            s_key_long_fired = 1U;
            s_key_event = KEY_EVT_LONG;
        }
    }
}

uint32_t Board_KeyLastPressMs(void)
{
    return s_key_press_ms;
}

board_key_event_t Board_KeyTakeEvent(void)
{
    board_key_event_t ev = s_key_event;
    s_key_event = KEY_EVT_NONE;
    return ev;
}

void Board_KeyResetStartArm(void)
{
    s_start_armed = 0U;
    s_start_deb_ms = HAL_GetTick();
    s_start_last_raw = 1U;
}

bool Board_KeyCheckStartEdge(uint32_t now_ms)
{
    uint8_t raw = Board_KeyIsPressed() ? 1U : 0U;
    if (raw != s_start_last_raw) {
        s_start_last_raw = raw;
        s_start_deb_ms = now_ms;
    }
    if ((uint32_t)(now_ms - s_start_deb_ms) < 30U) {
        return false;
    }
    if (s_start_armed == 0U) {
        if (raw == 0U) {
            s_start_armed = 1U;
        }
        return false;
    }
    if (raw != 0U) {
        s_start_armed = 0U;
        return true;
    }
    return false;
}

void Board_LedSet(bool on)
{
    HAL_GPIO_WritePin(BOARD_LED_PORT, BOARD_LED_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Board_LedSetPattern(board_led_pattern_t pat)
{
    s_led_pat = pat;
}

void Board_LedUpdate(uint32_t now_ms)
{
    switch (s_led_pat) {
    case LED_PAT_OFF:
        Board_LedSet(false);
        break;
    case LED_PAT_ON:
        Board_LedSet(true);
        break;
    case LED_PAT_SLOW_BLINK:
        Board_LedSet(((now_ms % 1000U) < 500U));
        break;
    case LED_PAT_FAST_BLINK:
        Board_LedSet(((now_ms % 300U) < 150U));
        break;
    case LED_PAT_DOUBLE_BLINK: {
        uint32_t p = now_ms % 1200U;
        Board_LedSet((p < 120U) || (p >= 240U && p < 360U));
        break;
    }
    case LED_PAT_FAULT_BLINK:
        Board_LedSet(((now_ms % 140U) < 70U));
        break;
    default:
        Board_LedSet(false);
        break;
    }
}

void Error_Handler(void)
{
    __disable_irq();
    BspMotor_PanicStop();   /* 驱动板失联会保持上次输出，先停车 */
    Board_LedSet(true);
    while (1) {
        __NOP();
    }
}

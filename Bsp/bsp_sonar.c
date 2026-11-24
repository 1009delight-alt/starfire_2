#include "bsp_sonar.h"
#include "board.h"
#include "config.h"

/* HC-SR04 时序：Trig 高电平 >= 10 us 后，模块发 8 个 40 kHz 脉冲，Echo 拉高，宽度 = 往返时间。
 * 无目标时 Echo 约 38 ms 后自己落下；部分仿制模块无回波时会一直保持高电平，需要另外处理。
 *
 * TIM4：84 MHz / 84 = 1 MHz 自由计数，16 位回绕 65.5 ms，大于最长回波 38 ms，差值直接按 uint16 相减。
 * CH2 双边沿捕获，中断里读 PB7 电平区分上升沿 / 下降沿。 */

#define SONAR_TRIG_PULSE_US     12U
#define SONAR_ECHO_START_MS     30U     /* 触发后这么久还没等到上升沿：本次无回波（模块没接 / 没供电） */
#define SONAR_ECHO_STUCK_MS     50U     /* 上升沿后这么久还没落：回波卡高，本次作废 */

/* 非 static：调试器“监视”/Live Watch 窗口可直接输入 g_sonar 观测（file-static 变量解析不到） */
sonar_state_t g_sonar;

static volatile uint16_t s_rise_cnt = 0U;
static volatile uint16_t s_width_us = 0U;
static volatile bool     s_have_rise = false;
static volatile bool     s_have_width = false;
static volatile uint32_t s_rise_ms = 0U;

static bool     s_inited = false;
static bool     s_waiting = false;          /* 已触发，等本次回波 */
static uint32_t s_trig_ms = 0U;

static bool echo_is_high(void)
{
    return (HAL_GPIO_ReadPin(SONAR_ECHO_PORT, SONAR_ECHO_PIN) == GPIO_PIN_SET);
}

bool BspSonar_Init(void)
{
    SonarLogic_Reset(&g_sonar);
#if (CFG_ENABLE_SONAR == 0U)
    return false;
#else
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    /* Trig：推挽输出，空闲低电平（3.3 V 高电平能触发 5 V 供电的 HC-SR04） */
    HAL_GPIO_WritePin(SONAR_TRIG_PORT, SONAR_TRIG_PIN, GPIO_PIN_RESET);
    gpio.Pin = SONAR_TRIG_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SONAR_TRIG_PORT, &gpio);

    /* Echo：TIM4_CH2 复用输入。PB7 为 5 V 容忍（FT）引脚；下拉使模块未接时稳定为低，按“无回波”处理 */
    gpio.Pin = SONAR_ECHO_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(SONAR_ECHO_PORT, &gpio);

    TIM4->CR1 = 0U;
    TIM4->PSC = 83U;                                    /* 84 MHz / 84 = 1 MHz */
    TIM4->ARR = 0xFFFFU;
    TIM4->CCMR1 = TIM_CCMR1_CC2S_0                      /* IC2 映射到 TI2 */
                | (3U << TIM_CCMR1_IC2F_Pos);           /* 输入滤波 fCK_INT, N=8 */
    TIM4->CCER = TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC2NP;   /* 双边沿 */
    TIM4->EGR = TIM_EGR_UG;
    TIM4->SR = 0U;
    TIM4->DIER = TIM_DIER_CC2IE;
    TIM4->CR1 = TIM_CR1_CEN;

    HAL_NVIC_SetPriority(TIM4_IRQn, 4U, 0U);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);

    s_inited = true;
    s_trig_ms = HAL_GetTick() - CFG_SONAR_PERIOD_MS;    /* 第一次 Update 立即触发 */
    return true;
#endif
}

void BspSonar_IRQHandler(void)
{
    uint32_t sr = TIM4->SR;
    if ((sr & TIM_SR_CC2IF) == 0U) {
        TIM4->SR = (uint16_t)~(sr & (TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC3IF | TIM_SR_CC4IF));
        return;
    }
    uint16_t cap = (uint16_t)TIM4->CCR2;                /* 读 CCR2 同时清 CC2IF */
    TIM4->SR = (uint16_t)~TIM_SR_CC2OF;
    if (echo_is_high()) {
        s_rise_cnt = cap;
        s_rise_ms = HAL_GetTick();
        s_have_rise = true;
    } else if (s_have_rise) {
        s_width_us = (uint16_t)(cap - s_rise_cnt);
        s_have_rise = false;
        s_have_width = true;
    }
}

void BspSonar_Update(uint32_t now_ms)
{
    if (!s_inited) {
        return;
    }

    if (s_waiting) {
        bool got = false, stuck = false;
        uint16_t width = 0U;

        __disable_irq();
        if (s_have_width) {
            s_have_width = false;
            width = s_width_us;
            got = true;
        } else if (s_have_rise && ((uint32_t)(now_ms - s_rise_ms) >= SONAR_ECHO_STUCK_MS)) {
            s_have_rise = false;            /* 卡高的这次作废，之后的下降沿不再配对 */
            stuck = true;
        }
        bool rising = s_have_rise;
        __enable_irq();

        if (got) {
            s_waiting = false;
            SonarLogic_FeedEcho(&g_sonar, width, now_ms, CFG_SONAR_MM_PER_US,
                                CFG_SONAR_MAX_MM, CFG_OBS_TRIG_DIST_MM);
        } else if (stuck ||
                   (!rising && ((uint32_t)(now_ms - s_trig_ms) >= SONAR_ECHO_START_MS))) {
            s_waiting = false;
            if (g_sonar.miss_count < 0xFFFFU) { g_sonar.miss_count++; }
        } else {
            return;                         /* 回波进行中 */
        }
    }

    if ((uint32_t)(now_ms - s_trig_ms) < CFG_SONAR_PERIOD_MS) {
        return;
    }
    if (echo_is_high()) {
        return;                             /* 上一次回波还没落（卡高），这时触发无效，等它落下 */
    }

    s_trig_ms = now_ms;
    __disable_irq();
    s_have_rise = false;
    s_have_width = false;
    __enable_irq();
    s_waiting = true;
    HAL_GPIO_WritePin(SONAR_TRIG_PORT, SONAR_TRIG_PIN, GPIO_PIN_SET);
    Board_DelayUs(SONAR_TRIG_PULSE_US);
    HAL_GPIO_WritePin(SONAR_TRIG_PORT, SONAR_TRIG_PIN, GPIO_PIN_RESET);
}

void BspSonar_ResetTrigger(void)
{
    g_sonar.near_hits = 0U;
}

/* 最近 CFG_SONAR_STALE_MS 内收到过完整回波（超量程也算，说明模块在工作） */
bool BspSonar_IsHealthy(uint32_t now_ms)
{
    return g_sonar.online && ((uint32_t)(now_ms - g_sonar.last_update_ms) <= CFG_SONAR_STALE_MS);
}

bool BspSonar_IsObstacleDetected(uint32_t now_ms)
{
    return BspSonar_IsHealthy(now_ms) && g_sonar.valid && (g_sonar.near_hits >= CFG_OBS_CONFIRM_FRAMES);
}

const sonar_state_t *BspSonar_GetState(void)
{
    return &g_sonar;
}

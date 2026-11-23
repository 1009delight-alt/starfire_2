#include "bsp_gimbal.h"
#include "board.h"
#include "config.h"

static TIM_HandleTypeDef s_htim4;
static uint16_t s_cur_pan_us  = CFG_GIMBAL_CENTER_PAN_US;
static uint16_t s_cur_tilt_us = CFG_GIMBAL_CENTER_TILT_US;
static uint16_t s_tgt_pan_us  = CFG_GIMBAL_CENTER_PAN_US;
static uint16_t s_tgt_tilt_us = CFG_GIMBAL_CENTER_TILT_US;
static uint8_t  s_inited = 0U;
static uint32_t s_reached_ms = 0U;   /* 脉宽最近一次到达目标的时刻 */
static bool     s_reached = true;

static uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

static void apply_pwm(uint16_t pan_us, uint16_t tilt_us)
{
    if (s_inited == 0U) {
        return;
    }
    __HAL_TIM_SET_COMPARE(&s_htim4, TIM_CHANNEL_3, pan_us);
    __HAL_TIM_SET_COMPARE(&s_htim4, TIM_CHANNEL_4, tilt_us);
}

void BspGimbal_Init(void)
{
#if (CFG_ENABLE_GIMBAL == 0U)
    return;
#else
    GPIO_InitTypeDef gpio = {0};
    TIM_OC_InitTypeDef oc = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    gpio.Pin = GIMBAL_PAN_PIN | GIMBAL_TILT_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* APB1 Timer Clock = 84 MHz -> PSC=83 得 1 MHz (1us/tick), ARR=19999 得 20ms (50Hz) */
    s_htim4.Instance = TIM4;
    s_htim4.Init.Prescaler = 83U;
    s_htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    s_htim4.Init.Period = 19999U;
    s_htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    s_htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&s_htim4) != HAL_OK) {
        Error_Handler();
    }

    s_cur_pan_us  = clamp_u16(CFG_GIMBAL_CENTER_PAN_US,  CFG_GIMBAL_PAN_MIN_US,  CFG_GIMBAL_PAN_MAX_US);
    s_cur_tilt_us = clamp_u16(CFG_GIMBAL_CENTER_TILT_US, CFG_GIMBAL_TILT_MIN_US, CFG_GIMBAL_TILT_MAX_US);
    s_tgt_pan_us  = s_cur_pan_us;
    s_tgt_tilt_us = s_cur_tilt_us;

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;

    oc.Pulse = s_cur_pan_us;
    if (HAL_TIM_PWM_ConfigChannel(&s_htim4, &oc, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
    oc.Pulse = s_cur_tilt_us;
    if (HAL_TIM_PWM_ConfigChannel(&s_htim4, &oc, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }

    s_inited = 1U;
    (void)HAL_TIM_PWM_Start(&s_htim4, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Start(&s_htim4, TIM_CHANNEL_4);
    apply_pwm(s_cur_pan_us, s_cur_tilt_us);
#endif
}

void BspGimbal_SetTargetUs(uint16_t pan_us, uint16_t tilt_us)
{
    uint16_t p = clamp_u16(pan_us,  CFG_GIMBAL_PAN_MIN_US,  CFG_GIMBAL_PAN_MAX_US);
    uint16_t t = clamp_u16(tilt_us, CFG_GIMBAL_TILT_MIN_US, CFG_GIMBAL_TILT_MAX_US);
    if ((p != s_tgt_pan_us) || (t != s_tgt_tilt_us)) {
        s_tgt_pan_us  = p;
        s_tgt_tilt_us = t;
        s_reached = false;
    }
}

void BspGimbal_SetPose(gimbal_pose_t pose)
{
    switch (pose) {
    case GIMBAL_POSE_SIGN:
        BspGimbal_SetTargetUs(CFG_GIMBAL_SIGN_PAN_US, CFG_GIMBAL_SIGN_TILT_US);
        break;
    case GIMBAL_POSE_COLOR:
        BspGimbal_SetTargetUs(CFG_GIMBAL_COLOR_PAN_US, CFG_GIMBAL_COLOR_TILT_US);
        break;
    case GIMBAL_POSE_PARK:
        BspGimbal_SetTargetUs(CFG_GIMBAL_PARK_PAN_US, CFG_GIMBAL_PARK_TILT_US);
        break;
    case GIMBAL_POSE_CENTER:
    default:
        BspGimbal_SetTargetUs(CFG_GIMBAL_CENTER_PAN_US, CFG_GIMBAL_CENTER_TILT_US);
        break;
    }
}

static uint16_t step_toward(uint16_t cur, uint16_t tgt, uint16_t step)
{
    if (cur < tgt) {
        uint16_t diff = (uint16_t)(tgt - cur);
        return (diff <= step) ? tgt : (uint16_t)(cur + step);
    }
    if (cur > tgt) {
        uint16_t diff = (uint16_t)(cur - tgt);
        return (diff <= step) ? tgt : (uint16_t)(cur - step);
    }
    return cur;
}

void BspGimbal_Update20ms(void)
{
    if (s_inited == 0U) {
        return;
    }
    s_cur_pan_us  = step_toward(s_cur_pan_us,  s_tgt_pan_us,  CFG_GIMBAL_STEP_US_PER_20MS);
    s_cur_tilt_us = step_toward(s_cur_tilt_us, s_tgt_tilt_us, CFG_GIMBAL_STEP_US_PER_20MS);
    apply_pwm(s_cur_pan_us, s_cur_tilt_us);
    if ((!s_reached) && (s_cur_pan_us == s_tgt_pan_us) && (s_cur_tilt_us == s_tgt_tilt_us)) {
        s_reached = true;
        s_reached_ms = HAL_GetTick();
    }
}

/* 脉宽到达目标后再等 CFG_GIMBAL_SETTLE_MS，才认为舵机停稳、画面可用。
 * 云台未启用时直接返回 true。 */
bool BspGimbal_IsSettled(uint32_t now_ms)
{
    if (s_inited == 0U) {
        return true;
    }
    return s_reached && ((uint32_t)(now_ms - s_reached_ms) >= CFG_GIMBAL_SETTLE_MS);
}

uint16_t BspGimbal_GetPanUs(void)
{
    return s_cur_pan_us;
}

uint16_t BspGimbal_GetTiltUs(void)
{
    return s_cur_tilt_us;
}

const char *BspGimbal_PoseName(gimbal_pose_t pose)
{
    switch (pose) {
    case GIMBAL_POSE_CENTER: return "CENTER";
    case GIMBAL_POSE_SIGN:   return "SIGN";
    case GIMBAL_POSE_COLOR:  return "COLOR";
    case GIMBAL_POSE_PARK:   return "PARK";
    default:                 return "UNK";
    }
}

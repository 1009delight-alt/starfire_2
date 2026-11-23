/* 电脑回放用的最小 HAL 替身：只提供被测源码用到的符号。 */
#ifndef SIM_STM32F4XX_HAL_H
#define SIM_STM32F4XX_HAL_H

#include <stdint.h>
#include <stdbool.h>

#define GPIO_PIN_0  0x0001U
#define GPIO_PIN_2  0x0004U
#define GPIO_PIN_3  0x0008U
#define GPIO_PIN_4  0x0010U
#define GPIO_PIN_6  0x0040U
#define GPIO_PIN_7  0x0080U
#define GPIO_PIN_8  0x0100U
#define GPIO_PIN_9  0x0200U
#define GPIO_PIN_10 0x0400U
#define GPIO_PIN_11 0x0800U

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);

typedef struct { volatile uint32_t APB1ENR; } SIM_RCC_t;
typedef struct { volatile uint32_t SR; volatile uint32_t DR; } SIM_USART_t;
extern SIM_RCC_t sim_rcc;
extern SIM_USART_t sim_usart2;
#define RCC                  (&sim_rcc)
#define USART2               (&sim_usart2)
#define RCC_APB1ENR_USART2EN (1UL << 17)
#define USART_SR_TXE         (1UL << 7)
#define USART_SR_TC          (1UL << 6)

#endif

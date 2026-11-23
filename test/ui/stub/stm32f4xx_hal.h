/* 屏幕页面电脑预览用的最小 HAL 替身：GPIO 与 SPI 由 render.c 实现，SPI 字节流交给 ST7735 模拟器。 */
#ifndef UI_STM32F4XX_HAL_H
#define UI_STM32F4XX_HAL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;

typedef struct { int id; } GPIO_TypeDef;
extern GPIO_TypeDef ui_gpioa, ui_gpiob, ui_gpioc, ui_gpioe;
#define GPIOA (&ui_gpioa)
#define GPIOB (&ui_gpiob)
#define GPIOC (&ui_gpioc)
#define GPIOE (&ui_gpioe)

#define GPIO_PIN_0  0x0001U
#define GPIO_PIN_1  0x0002U
#define GPIO_PIN_2  0x0004U
#define GPIO_PIN_3  0x0008U
#define GPIO_PIN_4  0x0010U
#define GPIO_PIN_5  0x0020U
#define GPIO_PIN_6  0x0040U
#define GPIO_PIN_7  0x0080U
#define GPIO_PIN_8  0x0100U
#define GPIO_PIN_9  0x0200U
#define GPIO_PIN_10 0x0400U
#define GPIO_PIN_11 0x0800U
#define GPIO_PIN_12 0x1000U
#define GPIO_PIN_13 0x2000U
#define GPIO_PIN_14 0x4000U
#define GPIO_PIN_15 0x8000U

typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
#define GPIO_MODE_OUTPUT_PP        1U
#define GPIO_MODE_AF_PP            2U
#define GPIO_NOPULL                0U
#define GPIO_SPEED_FREQ_HIGH       2U
#define GPIO_SPEED_FREQ_VERY_HIGH  3U
#define GPIO_AF5_SPI2              5U

typedef struct { int id; } SPI_TypeDef;
extern SPI_TypeDef ui_spi2;
#define SPI2 (&ui_spi2)
typedef struct {
    uint32_t Mode, Direction, DataSize, CLKPolarity, CLKPhase, NSS,
             BaudRatePrescaler, FirstBit, TIMode, CRCCalculation, CRCPolynomial;
} SPI_InitTypeDef;
typedef struct { SPI_TypeDef *Instance; SPI_InitTypeDef Init; } SPI_HandleTypeDef;
#define SPI_MODE_MASTER              1U
#define SPI_DIRECTION_2LINES         0U
#define SPI_DATASIZE_8BIT            0U
#define SPI_POLARITY_LOW             0U
#define SPI_PHASE_1EDGE              0U
#define SPI_NSS_SOFT                 1U
#define SPI_BAUDRATEPRESCALER_2      0U
#define SPI_BAUDRATEPRESCALER_4      1U
#define SPI_BAUDRATEPRESCALER_8      2U
#define SPI_FIRSTBIT_MSB             0U
#define SPI_TIMODE_DISABLE           0U
#define SPI_CRCCALCULATION_DISABLE   0U

#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_SPI2_CLK_ENABLE()  ((void)0)

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *h);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *h, uint8_t *data, uint16_t size, uint32_t timeout);

#endif

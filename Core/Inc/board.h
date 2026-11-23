#ifndef SKYSTAR_BOARD_H
#define SKYSTAR_BOARD_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* ==================== 引脚定义（严格对照装车接线总表） ==================== */
/* 板载 LED / KEY */
#define BOARD_LED_PORT          GPIOB
#define BOARD_LED_PIN           GPIO_PIN_2      /* 高电平点亮 */
#define BOARD_KEY_PORT          GPIOA
#define BOARD_KEY_PIN           GPIO_PIN_0      /* 按下高电平，内置下拉 */

/* 四路电机驱动 USART2 */
#define MOTOR_UART_TX_PORT      GPIOA
#define MOTOR_UART_TX_PIN       GPIO_PIN_2
#define MOTOR_UART_RX_PORT      GPIOA
#define MOTOR_UART_RX_PIN       GPIO_PIN_3

/* OpenMV USART1 */
#define OPENMV_UART_TX_PORT     GPIOA
#define OPENMV_UART_TX_PIN      GPIO_PIN_9
#define OPENMV_UART_RX_PORT     GPIOA
#define OPENMV_UART_RX_PIN      GPIO_PIN_10

/* 预留调试串口 USART6 */
#define DBG_UART_TX_PORT        GPIOC
#define DBG_UART_TX_PIN         GPIO_PIN_6
#define DBG_UART_RX_PORT        GPIOC
#define DBG_UART_RX_PIN         GPIO_PIN_7

/* 双轴云台 TIM4_CH3 / TIM4_CH4 */
#define GIMBAL_PAN_PORT         GPIOB
#define GIMBAL_PAN_PIN          GPIO_PIN_8      /* 水平轴 */
#define GIMBAL_TILT_PORT        GPIOB
#define GIMBAL_TILT_PIN         GPIO_PIN_9      /* 俯仰轴 */

/* 八路灰度传感器 */
#define GRAY_AD0_PORT           GPIOE
#define GRAY_AD0_PIN            GPIO_PIN_2
#define GRAY_AD1_PORT           GPIOE
#define GRAY_AD1_PIN            GPIO_PIN_3
#define GRAY_AD2_PORT           GPIOE
#define GRAY_AD2_PIN            GPIO_PIN_4
#define GRAY_ADC_PORT           GPIOC
#define GRAY_ADC_PIN            GPIO_PIN_0      /* ADC1_IN10 */

/* 前向激光测距 VL53L0X（软件 I2C） */
#define TOF_I2C_SCL_PORT        GPIOB
#define TOF_I2C_SCL_PIN         GPIO_PIN_6
#define TOF_I2C_SDA_PORT        GPIOB
#define TOF_I2C_SDA_PIN         GPIO_PIN_7

/* 四色结果显示屏：1.8 寸 128x160 SPI TFT（ST7735S），硬件 SPI2，只写 */
#define TFT_SCK_PORT            GPIOB
#define TFT_SCK_PIN             GPIO_PIN_13     /* 屏 SCK，SPI2_SCK  AF5 */
#define TFT_MOSI_PORT           GPIOB
#define TFT_MOSI_PIN            GPIO_PIN_15     /* 屏 SDA，SPI2_MOSI AF5 */
#define TFT_CS_PORT             GPIOB
#define TFT_CS_PIN              GPIO_PIN_12     /* 屏 CS，低有效 */
#define TFT_DC_PORT             GPIOB
#define TFT_DC_PIN              GPIO_PIN_10     /* 屏 A0（数据/命令），沿用原 OLED SCL 线 */
#define TFT_RST_PORT            GPIOB
#define TFT_RST_PIN             GPIO_PIN_11     /* 屏 RESET，低有效，沿用原 OLED SDA 线 */
/* 屏 LED（背光）直接接 3.3 V，不接 GPIO：背光电流超过单个引脚的推荐值 */

/* ==================== 按键事件与 LED 状态模式 ==================== */
typedef enum {
    KEY_EVT_NONE = 0,
    KEY_EVT_SHORT,          /* 短按释放（或触发） */
    KEY_EVT_LONG            /* 长按 >= 1000 ms */
} board_key_event_t;

typedef enum {
    LED_PAT_OFF = 0,
    LED_PAT_ON,
    LED_PAT_SLOW_BLINK,     /* 500ms 周期：待机 */
    LED_PAT_FAST_BLINK,     /* 150ms 周期：运行中 */
    LED_PAT_DOUBLE_BLINK,   /* 双闪：任务完成 */
    LED_PAT_FAULT_BLINK     /* 急闪：故障停机 */
} board_led_pattern_t;

void Board_Init(void);
void Error_Handler(void);

/* 独立看门狗（寄存器直写，不依赖 HAL_IWDG 模块） */
void Board_WatchdogStart(uint16_t reload_ms);
void Board_WatchdogKick(void);
bool Board_WasWatchdogReset(void);   /* 读取并清除复位标志 */

/* 基于 DWT CYCCNT 的精准微秒延时（不影响 HAL SysTick） */
void Board_DelayUs(uint32_t us);

/* 板载按键与状态灯 */
bool Board_KeyIsPressed(void);
void Board_KeyUpdate(uint32_t now_ms);
board_key_event_t Board_KeyTakeEvent(void);
uint32_t Board_KeyLastPressMs(void);    /* 最近一次（消抖后）按下的时刻 */
void Board_KeyResetStartArm(void);
bool Board_KeyCheckStartEdge(uint32_t now_ms); /* 必须先松开再按下才返回 true */

void Board_LedSet(bool on);
void Board_LedSetPattern(board_led_pattern_t pat);
void Board_LedUpdate(uint32_t now_ms);

#endif /* SKYSTAR_BOARD_H */

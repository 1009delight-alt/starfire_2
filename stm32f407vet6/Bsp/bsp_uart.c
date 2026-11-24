#include "bsp_uart.h"
#include "board.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define UART_RX_RING_SIZE   256U

typedef struct {
    UART_HandleTypeDef huart;
    uint8_t  rx_buf[UART_RX_RING_SIZE];
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    uint8_t  inited;
} uart_ctx_t;

static uart_ctx_t s_uart[BSP_UART_COUNT];

void BspUart_Init(bsp_uart_id_t id, uint32_t baudrate)
{
    GPIO_InitTypeDef gpio = {0};
    USART_TypeDef *inst = NULL;
    IRQn_Type irqn = USART1_IRQn;
    uint32_t prio = 2U;

    if (id >= BSP_UART_COUNT) {
        return;
    }

    if (id == BSP_UART_OPENMV) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART1_CLK_ENABLE();
        gpio.Pin = OPENMV_UART_TX_PIN | OPENMV_UART_RX_PIN;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(OPENMV_UART_TX_PORT, &gpio);
        inst = USART1;
        irqn = USART1_IRQn;
        prio = 2U;
    } else if (id == BSP_UART_MOTOR) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART2_CLK_ENABLE();
        gpio.Pin = MOTOR_UART_TX_PIN | MOTOR_UART_RX_PIN;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(MOTOR_UART_TX_PORT, &gpio);
        inst = USART2;
        irqn = USART2_IRQn;
        prio = 1U;  /* 电机编码器串口优先级高于视觉 */
    } else {
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_USART6_CLK_ENABLE();
        gpio.Pin = DBG_UART_TX_PIN | DBG_UART_RX_PIN;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF8_USART6;
        HAL_GPIO_Init(DBG_UART_TX_PORT, &gpio);
        inst = USART6;
        irqn = USART6_IRQn;
        prio = 3U;
    }

    s_uart[id].rx_head = 0U;
    s_uart[id].rx_tail = 0U;
    s_uart[id].huart.Instance = inst;
    s_uart[id].huart.Init.BaudRate = baudrate;
    s_uart[id].huart.Init.WordLength = UART_WORDLENGTH_8B;
    s_uart[id].huart.Init.StopBits = UART_STOPBITS_1;
    s_uart[id].huart.Init.Parity = UART_PARITY_NONE;
    s_uart[id].huart.Init.Mode = UART_MODE_TX_RX;
    s_uart[id].huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    s_uart[id].huart.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&s_uart[id].huart) != HAL_OK) {
        Error_Handler();
    }

    __HAL_UART_ENABLE_IT(&s_uart[id].huart, UART_IT_RXNE);
    HAL_NVIC_SetPriority(irqn, prio, 0U);
    HAL_NVIC_EnableIRQ(irqn);
    s_uart[id].inited = 1U;
}

void BspUart_IRQHandler(bsp_uart_id_t id)
{
    if ((id >= BSP_UART_COUNT) || (s_uart[id].inited == 0U)) {
        return;
    }

    USART_TypeDef *inst = s_uart[id].huart.Instance;
    uint32_t sr = inst->SR;
    uint32_t err_mask = USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE;

    if ((sr & (USART_SR_RXNE | err_mask)) != 0U) {
        uint8_t byte = (uint8_t)(inst->DR & 0xFFU);
        if ((sr & err_mask) == 0U) {
            uint16_t next = (uint16_t)((s_uart[id].rx_head + 1U) % UART_RX_RING_SIZE);
            if (next != s_uart[id].rx_tail) {
                s_uart[id].rx_buf[s_uart[id].rx_head] = byte;
                s_uart[id].rx_head = next;
            }
        }
    }
}

bool BspUart_ReadByte(bsp_uart_id_t id, uint8_t *out)
{
    if ((id >= BSP_UART_COUNT) || (out == NULL) || (s_uart[id].inited == 0U)) {
        return false;
    }
    if (s_uart[id].rx_head == s_uart[id].rx_tail) {
        return false;
    }
    *out = s_uart[id].rx_buf[s_uart[id].rx_tail];
    s_uart[id].rx_tail = (uint16_t)((s_uart[id].rx_tail + 1U) % UART_RX_RING_SIZE);
    return true;
}

void BspUart_FlushRx(bsp_uart_id_t id)
{
    if (id < BSP_UART_COUNT) {
        s_uart[id].rx_tail = s_uart[id].rx_head;
    }
}

void BspUart_SendBytes(bsp_uart_id_t id, const uint8_t *data, size_t len)
{
    if ((id >= BSP_UART_COUNT) || (s_uart[id].inited == 0U) || (data == NULL) || (len == 0U)) {
        return;
    }
    (void)HAL_UART_Transmit(&s_uart[id].huart, (uint8_t *)data, (uint16_t)len, 20U);
}

void BspUart_SendStr(bsp_uart_id_t id, const char *str)
{
    if (str != NULL) {
        BspUart_SendBytes(id, (const uint8_t *)str, strlen(str));
    }
}

void BspUart_Printf(bsp_uart_id_t id, const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    int n;

    if ((id >= BSP_UART_COUNT) || (s_uart[id].inited == 0U)) {
        return;
    }
    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) {
        size_t len = ((size_t)n < sizeof(buf)) ? (size_t)n : (sizeof(buf) - 1U);
        BspUart_SendBytes(id, (const uint8_t *)buf, len);
    }
}

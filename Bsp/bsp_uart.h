#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    BSP_UART_OPENMV = 0,    /* USART1: PA9(TX), PA10(RX) */
    BSP_UART_MOTOR  = 1,    /* USART2: PA2(TX), PA3(RX)  */
    BSP_UART_DEBUG  = 2,    /* USART6: PC6(TX), PC7(RX)  */
    BSP_UART_COUNT
} bsp_uart_id_t;

void BspUart_Init(bsp_uart_id_t id, uint32_t baudrate);
void BspUart_IRQHandler(bsp_uart_id_t id);

bool BspUart_ReadByte(bsp_uart_id_t id, uint8_t *out);
void BspUart_FlushRx(bsp_uart_id_t id);

void BspUart_SendBytes(bsp_uart_id_t id, const uint8_t *data, size_t len);
void BspUart_SendStr(bsp_uart_id_t id, const char *str);
void BspUart_Printf(bsp_uart_id_t id, const char *fmt, ...);

#endif /* BSP_UART_H */

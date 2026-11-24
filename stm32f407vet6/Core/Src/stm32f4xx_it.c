#include "board.h"
#include "bsp_uart.h"
#include "bsp_motor.h"
#include "bsp_sonar.h"

void SysTick_Handler(void)
{
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}

void USART1_IRQHandler(void)
{
    BspUart_IRQHandler(BSP_UART_OPENMV);
}

void USART2_IRQHandler(void)
{
    BspUart_IRQHandler(BSP_UART_MOTOR);
}

void USART6_IRQHandler(void)
{
    BspUart_IRQHandler(BSP_UART_DEBUG);
}

void TIM4_IRQHandler(void)
{
    BspSonar_IRQHandler();   /* HC-SR04 Echo 双边沿捕获 */
}

/* 所有致命异常：先直接发零 PWM 停车，再亮灯死等。
 * 看门狗不再被喂，约 300ms 后复位，复位后 BspMotor_Init 会再次发零 PWM。 */
static void fatal_stop(void)
{
    BspMotor_PanicStop();
    Board_LedSet(true);
    while (1) {
        __NOP();
    }
}

void HardFault_Handler(void)  { fatal_stop(); }
void MemManage_Handler(void)  { fatal_stop(); }
void BusFault_Handler(void)   { fatal_stop(); }
void UsageFault_Handler(void) { fatal_stop(); }

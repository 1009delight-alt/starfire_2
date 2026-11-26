#ifndef UI_PAGES_H
#define UI_PAGES_H

/* 分步调试模式的屏幕页面（160x128 彩屏）。
 * 这些函数只写帧缓冲，不读外设、不发送；调用方之后自己 BspOled_Flush()。
 * 不依赖硬件，电脑上可以渲染预览（test/ui/run_ui.ps1）。 */

#include <stdbool.h>
#include <stdint.h>

#include "bsp_motor.h"
#include "bsp_gray.h"
#include "bsp_sonar.h"
#include "bsp_openmv.h"

void UiPage_MotorEnc(const motor_state_t *m, uint8_t next_step);
void UiPage_MotionCal(const motor_state_t *m, bool active, uint8_t task);
void UiPage_Gray(const gray_state_t *g, bool calib);
void UiPage_Sonar(const sonar_state_t *s, bool healthy, bool obstacle);
void UiPage_Openmv(const openmv_state_t *mv, const char *cmd_name, const char *ack_name, bool running);
void UiPage_Line(const gray_state_t *g, const motor_state_t *m, bool running, const char *bias_name);

#endif /* UI_PAGES_H */

#ifndef BSP_GIMBAL_H
#define BSP_GIMBAL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    GIMBAL_POSE_CENTER = 0, /* 水平朝前、俯仰正位 */
    GIMBAL_POSE_SIGN,       /* 看右前方交通转向牌 */
    GIMBAL_POSE_COLOR,      /* 看右侧四色方块 */
    GIMBAL_POSE_PARK,       /* 看右侧车库 P 牌 */
    GIMBAL_POSE_COUNT
} gimbal_pose_t;

void BspGimbal_Init(void);
void BspGimbal_SetPose(gimbal_pose_t pose);
void BspGimbal_SetTargetUs(uint16_t pan_us, uint16_t tilt_us);
void BspGimbal_Update20ms(void);

bool BspGimbal_IsSettled(uint32_t now_ms);   /* 脉宽到位且已等待稳定时间 */
uint16_t BspGimbal_GetPanUs(void);
uint16_t BspGimbal_GetTiltUs(void);
const char *BspGimbal_PoseName(gimbal_pose_t pose);

#endif /* BSP_GIMBAL_H */

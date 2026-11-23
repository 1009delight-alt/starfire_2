#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int32_t  raw_enc[4];            /* 驱动板原始累计编码器计数 M1~M4 */
    int32_t  fwd_enc[4];            /* 已按物理前进方向符号化后的累计计数 */
    float    wheel_spd_mm_s[4];     /* 四轮实测速度 (mm/s) */
    float    target_spd_mm_s[4];    /* 四轮目标速度 (mm/s) */
    int16_t  last_pwm[4];           /* 最近下发的 PWM 值 */
    float    total_dist_mm;         /* 从发车起累计总里程 (mm) */
    float    seg_dist_mm;           /* 当前动作段相对里程 (mm) */
    float    seg_yaw_deg;           /* 当前动作段估算航向角 (deg, 左转正) */
    float    total_yaw_deg;         /* 从发车起累计航向角，用于弯道识别 */
    float    battery_v;             /* 驱动板回报电池电压 (V) */
    uint32_t last_rx_ms;            /* 最近收到有效编码器帧的时刻 */
    uint32_t rx_frame_count;        /* 累计收到编码器帧数 */
    bool     enc_online;            /* 编码器反馈是否在线 */
    bool     rx_timeout_fault;      /* 运动期间发生通信超时故障 */
    bool     enc_jump_fault;        /* 运动期间累计计数突变（驱动板复位）故障 */
    uint16_t enc_jump_count;        /* 累计检测到的计数突变次数 */
    bool     enc_stall_fault;       /* 有目标速度却持续不动：编码器冻结或轮子卡住 */
    uint8_t  stall_wheel;           /* 触发堵转的轮号 1~4 */
} motor_state_t;

void BspMotor_Init(void);
void BspMotor_PollRx(uint32_t now_ms);
void BspMotor_ControlStep(uint32_t now_ms, float dt_s);

/* 高层速度接口（mm/s）：左轮组(M1,M2)、右轮组(M3,M4) */
void BspMotor_SetDiffSpeed(float left_mm_s, float right_mm_s);
void BspMotor_SetWheelSpeed(float m1, float m2, float m3, float m4);

/* 台架调试直通开环 PWM（-3600~3600） */
void BspMotor_SetRawPwm(int16_t m1, int16_t m2, int16_t m3, int16_t m4);

/* 安全停车（发 $pwm:0,0,0,0# 释放输出并清零积分） */
void BspMotor_Stop(void);
void BspMotor_EmergencyStop(void);
void BspMotor_PanicStop(void);      /* 故障上下文：直接写寄存器发零 PWM */
void BspMotor_RequestVoltage(void);

/* 里程计重置 */
void BspMotor_ResetTotalOdom(void);
void BspMotor_ResetSegOdom(void);
void BspMotor_SetTotalDist(float mm);  /* 中途起跑时预置总里程 */
void BspMotor_ClearFault(void);

bool BspMotor_HasFault(void);
const motor_state_t *BspMotor_GetState(void);

#endif /* BSP_MOTOR_H */

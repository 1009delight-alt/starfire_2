#include "bsp_motor.h"
#include "bsp_uart.h"
#include "board.h"
#include "config.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static motor_state_t s_mot;
static int32_t  s_base_raw[4];
static int32_t  s_prev_fwd[4];
static uint8_t  s_have_base = 0U;

/* 速度 PID 内部状态 */
static float    s_pid_integ[4];
static float    s_pid_prev_err[4];
static float    s_pid_prev_tgt[4];      /* 上一周期目标，用来发现正反转切换 */
static float    s_cmd_spd[4];           /* 限加速度后实际送进轮速环的目标 (mm/s)，停车时清零 */
static uint8_t  s_raw_pwm_mode = 0U;
static int16_t  s_raw_pwm_cmd[4];
static uint8_t  s_driving = 0U;
static uint32_t s_last_upload_req_ms = 0U;
static uint32_t s_last_zero_keep_ms = 0U;
static uint32_t s_stall_ms[4];          /* 各轮“有目标却不动”的累计时间 */

/* 串口帧拼装 */
static char     s_rx_frame[80];
static uint8_t  s_rx_used = 0U;

static const int32_t k_enc_sign[4] = {
    CFG_ENC_SIGN_M1, CFG_ENC_SIGN_M2, CFG_ENC_SIGN_M3, CFG_ENC_SIGN_M4
};
static const float k_ff_scale[4] = {
    CFG_WHEEL_FF_SCALE_M1, CFG_WHEEL_FF_SCALE_M2, CFG_WHEEL_FF_SCALE_M3, CFG_WHEEL_FF_SCALE_M4
};

static float clampf(float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

static int16_t clamp_i16(int32_t v, int16_t lo, int16_t hi)
{
    if (v < (int32_t)lo) { return lo; }
    if (v > (int32_t)hi) { return hi; }
    return (int16_t)v;
}

static void send_pwm_frame(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    char buf[48];
    s_mot.last_pwm[0] = m1;
    s_mot.last_pwm[1] = m2;
    s_mot.last_pwm[2] = m3;
    s_mot.last_pwm[3] = m4;
    (void)snprintf(buf, sizeof(buf), "$pwm:%d,%d,%d,%d#",
                   (int)m1, (int)m2, (int)m3, (int)m4);
    BspUart_SendStr(BSP_UART_MOTOR, buf);
}

static void send_spd_frame(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    char buf[48];
    (void)snprintf(buf, sizeof(buf), "$spd:%d,%d,%d,%d#",
                   (int)m1, (int)m2, (int)m3, (int)m4);
    BspUart_SendStr(BSP_UART_MOTOR, buf);
}

static bool parse_4_int32(const char *s, int32_t out[4])
{
    char *endp = NULL;
    for (int i = 0; i < 4; ++i) {
        long val = strtol(s, &endp, 10);
        if (endp == s) {
            return false;
        }
        out[i] = (int32_t)val;
        if (i < 3) {
            if (*endp != ',') {
                return false;
            }
            s = endp + 1;
        }
    }
    /* 第 4 个数之后必须正好是帧尾 '#'，多余字符整帧丢弃 */
    return (endp != NULL) && (endp[0] == '#') && (endp[1] == '\0');
}

static void handle_motor_frame(const char *frame, uint32_t now_ms)
{
    if (strncmp(frame, "$MAll:", 6) == 0) {
        int32_t vals[4];
        if (parse_4_int32(frame + 6, vals)) {
            if (s_have_base == 0U) {
                for (int i = 0; i < 4; ++i) {
                    s_base_raw[i] = vals[i];
                    s_mot.raw_enc[i] = vals[i];
                    s_mot.fwd_enc[i] = 0;
                    s_prev_fwd[i] = 0;
                }
                s_have_base = 1U;
            } else {
                /* 驱动板复位或串口错位时累计计数会突变：重建基线保持里程连续，
                 * 运动中发生则置故障，由状态机停车（重连后不自行继续运动） */
                bool jump = false;
                for (int i = 0; i < 4; ++i) {
                    int32_t step = (int32_t)((uint32_t)vals[i] - (uint32_t)s_mot.raw_enc[i]);
                    if ((step > CFG_ENC_JUMP_COUNTS) || (step < -CFG_ENC_JUMP_COUNTS)) {
                        jump = true;
                    }
                }
                if (jump) {
                    for (int i = 0; i < 4; ++i) {
                        /* 新基线使 fwd_enc 保持跳变前的值 */
                        s_base_raw[i] = (int32_t)((uint32_t)vals[i] - (uint32_t)(s_mot.fwd_enc[i] * k_enc_sign[i]));
                        s_mot.raw_enc[i] = vals[i];
                    }
                    s_mot.enc_jump_count++;
                    if (s_driving != 0U) {
                        s_mot.enc_jump_fault = true;
                        BspMotor_EmergencyStop();
                    }
                } else {
                    for (int i = 0; i < 4; ++i) {
                        int32_t diff = (int32_t)((uint32_t)vals[i] - (uint32_t)s_base_raw[i]);
                        s_mot.raw_enc[i] = vals[i];
                        s_mot.fwd_enc[i] = diff * k_enc_sign[i];
                    }
                }
            }
            s_mot.last_rx_ms = now_ms;
            s_mot.rx_frame_count++;
            s_mot.enc_online = true;
        }
    } else if (strncmp(frame, "$Battery:", 9) == 0) {
        s_mot.battery_v = strtof(frame + 9, NULL);
    }
}

void BspMotor_Init(void)
{
    memset(&s_mot, 0, sizeof(s_mot));
    memset(s_pid_integ, 0, sizeof(s_pid_integ));
    memset(s_pid_prev_err, 0, sizeof(s_pid_prev_err));
    memset(s_cmd_spd, 0, sizeof(s_cmd_spd));
    s_have_base = 0U;
    s_raw_pwm_mode = 0U;
    s_driving = 0U;
    s_rx_used = 0U;

    BspUart_Init(BSP_UART_MOTOR, 115200U);

    /* 上电先两次发送零 PWM，开启累计编码器上报并查一次电池电压 */
    send_pwm_frame(0, 0, 0, 0);
    HAL_Delay(15U);
    send_pwm_frame(0, 0, 0, 0);
    HAL_Delay(15U);
    BspUart_SendStr(BSP_UART_MOTOR, "$upload:1,0,0#");
    HAL_Delay(15U);
    BspUart_SendStr(BSP_UART_MOTOR, "$read_vol#");
    s_last_upload_req_ms = HAL_GetTick();
}

void BspMotor_PollRx(uint32_t now_ms)
{
    uint8_t ch;
    while (BspUart_ReadByte(BSP_UART_MOTOR, &ch)) {
        if (ch == (uint8_t)'$') {
            s_rx_used = 0U;
            s_rx_frame[s_rx_used++] = '$';
        } else if (s_rx_used > 0U) {
            if (s_rx_used < (sizeof(s_rx_frame) - 1U)) {
                s_rx_frame[s_rx_used++] = (char)ch;
                if (ch == (uint8_t)'#') {
                    s_rx_frame[s_rx_used] = '\0';
                    handle_motor_frame(s_rx_frame, now_ms);
                    s_rx_used = 0U;
                }
            } else {
                s_rx_used = 0U;
            }
        }
    }

    if (s_mot.rx_frame_count > 0U) {
        if ((uint32_t)(now_ms - s_mot.last_rx_ms) > CFG_MOTOR_RX_TIMEOUT_MS) {
            s_mot.enc_online = false;
            if (s_driving != 0U) {
                s_mot.rx_timeout_fault = true;
                BspMotor_EmergencyStop();
            }
        }
    }

    /* 离线时周期补发 $upload:1,0,0#，兼容驱动板比主控晚通电 */
    if ((!s_mot.enc_online) && ((uint32_t)(now_ms - s_last_upload_req_ms) >= 500U)) {
        s_last_upload_req_ms = now_ms;
        if (s_driving == 0U) {
            send_pwm_frame(0, 0, 0, 0);
        }
        BspUart_SendStr(BSP_UART_MOTOR, "$upload:1,0,0#");
    }
}

void BspMotor_ControlStep(uint32_t now_ms, float dt_s)
{
    if (dt_s <= 0.001f) {
        return;
    }

    /* 1. 更新四轮速度与里程计 */
    if (s_have_base != 0U) {
        float d_mm[4];
        for (int i = 0; i < 4; ++i) {
            int32_t dc = s_mot.fwd_enc[i] - s_prev_fwd[i];
            s_prev_fwd[i] = s_mot.fwd_enc[i];
            d_mm[i] = (float)dc / CFG_ENC_COUNTS_PER_MM;
            float inst_spd = d_mm[i] / dt_s;
            /* 一阶低通滤波抑制单周期量化抖动 */
            s_mot.wheel_spd_mm_s[i] = 0.65f * s_mot.wheel_spd_mm_s[i] + 0.35f * inst_spd;
        }
        float left_d  = 0.5f * (d_mm[0] + d_mm[1]);
        float right_d = 0.5f * (d_mm[2] + d_mm[3]);
        float avg_d   = 0.5f * (left_d + right_d);

        s_mot.total_dist_mm += avg_d;
        s_mot.seg_dist_mm   += avg_d;
        float dyaw = ((right_d - left_d) / CFG_TRACK_WIDTH_MM) * 57.29578f;
        s_mot.seg_yaw_deg   += dyaw;
        s_mot.total_yaw_deg += dyaw;
    }

    /* 1.5 堵转/计数冻结检测（只在闭环速度模式下有意义） */
    if ((s_driving != 0U) && (s_raw_pwm_mode == 0U) && !s_mot.enc_stall_fault) {
        uint32_t dt_ms = (uint32_t)(dt_s * 1000.0f + 0.5f);
        for (int i = 0; i < 4; ++i) {
            if ((fabsf(s_mot.target_spd_mm_s[i]) >= CFG_STALL_MIN_TGT_MM_S) &&
                (fabsf(s_mot.wheel_spd_mm_s[i]) < CFG_STALL_MAX_SPD_MM_S)) {
                s_stall_ms[i] += dt_ms;
                if (s_stall_ms[i] >= CFG_STALL_MS) {
                    s_mot.enc_stall_fault = true;
                    s_mot.stall_wheel = (uint8_t)(i + 1);
                }
            } else {
                s_stall_ms[i] = 0U;
            }
        }
        if (s_mot.enc_stall_fault) {
            BspMotor_EmergencyStop();
        }
    }

    /* 2. 故障态或停止态：维持安全零 PWM */
    if (s_mot.rx_timeout_fault || s_mot.enc_jump_fault || s_mot.enc_stall_fault || (s_driving == 0U)) {
        if ((uint32_t)(now_ms - s_last_zero_keep_ms) >= 250U) {
            s_last_zero_keep_ms = now_ms;
            send_pwm_frame(0, 0, 0, 0);
        }
        return;
    }

    /* 3. 台架开环直通 PWM 模式 */
    if (s_raw_pwm_mode != 0U) {
        send_pwm_frame(s_raw_pwm_cmd[0], s_raw_pwm_cmd[1], s_raw_pwm_cmd[2], s_raw_pwm_cmd[3]);
        return;
    }

    /* 4. 闭环速度输出。目标速度按加速度上限逐周期逼近 target_spd_mm_s：上层一步给到位（起步、换赛段、
     * 找线降速、定距定角开始）时 PWM 不跟着突跳。急停和 BspMotor_Stop 不经过这里，立即生效 */
    const float dv_max = CFG_WHEEL_ACC_MAX_MM_S2 * dt_s;
    for (int i = 0; i < 4; ++i) {
        float tgt = clampf(s_mot.target_spd_mm_s[i], -CFG_WHEEL_SPD_MAX_MM_S, CFG_WHEEL_SPD_MAX_MM_S);
        s_cmd_spd[i] += clampf(tgt - s_cmd_spd[i], -dv_max, dv_max);
    }
#if (CFG_MOTOR_CTRL_MODE == MOTOR_MODE_DRIVER_SPD)
    int16_t sp[4];
    for (int i = 0; i < 4; ++i) {
        float clamped = s_cmd_spd[i];
        sp[i] = clamp_i16((int32_t)lroundf(clamped), -1000, 1000);
    }
    send_spd_frame(sp[0], sp[1], sp[2], sp[3]);
#else
    int16_t pwm_out[4];
    for (int i = 0; i < 4; ++i) {
        float tgt = s_cmd_spd[i];
        if (fabsf(tgt) < 1.0f) {
            s_pid_integ[i] = 0.0f;
            s_pid_prev_err[i] = 0.0f;
            pwm_out[i] = 0;
            continue;
        }

        /* 目标正反向切换（弯里内侧轮常见）时清积分：旧方向攒下的积分会把新方向的输出顶反，PWM 突跳 */
        if ((tgt > 0.0f) != (s_pid_prev_tgt[i] > 0.0f)) {
            s_pid_integ[i] = 0.0f;
            s_pid_prev_err[i] = 0.0f;
        }
        s_pid_prev_tgt[i] = tgt;

        float err = tgt - s_mot.wheel_spd_mm_s[i];
        s_pid_integ[i] = clampf(s_pid_integ[i] + err * dt_s * CFG_WHEEL_PID_KI * 50.0f,
                                -1200.0f, 1200.0f);
        float deriv = (err - s_pid_prev_err[i]) / dt_s;
        s_pid_prev_err[i] = err;

        float ff = tgt * CFG_WHEEL_PID_FF * k_ff_scale[i];
        /* 起步死区补偿在 ±CFG_WHEEL_DEAD_RAMP_MM_S 内按比例给：原来过零就从 +350 翻到 −350，
         * 弯里内侧轮目标在 0 附近来回时 PWM 一跳约 700 */
        float dead = (float)CFG_WHEEL_PWM_DEADZONE * clampf(tgt / CFG_WHEEL_DEAD_RAMP_MM_S, -1.0f, 1.0f);
        float u = dead + ff + CFG_WHEEL_PID_KP * err + s_pid_integ[i] + CFG_WHEEL_PID_KD * deriv * 0.02f;

        /* 与目标反向的输出只用来制动（内侧轮被拖着走时减速），限在 CFG_WHEEL_BRAKE_PWM 以内，防止反转抖动 */
        if (tgt > 0.0f && u < -(float)CFG_WHEEL_BRAKE_PWM) { u = -(float)CFG_WHEEL_BRAKE_PWM; }
        if (tgt < 0.0f && u > (float)CFG_WHEEL_BRAKE_PWM) { u = (float)CFG_WHEEL_BRAKE_PWM; }

        pwm_out[i] = clamp_i16((int32_t)lroundf(u), -CFG_WHEEL_PWM_MAX, CFG_WHEEL_PWM_MAX);
    }
    send_pwm_frame(pwm_out[0], pwm_out[1], pwm_out[2], pwm_out[3]);
#endif
}

void BspMotor_SetWheelSpeed(float m1, float m2, float m3, float m4)
{
    s_raw_pwm_mode = 0U;
    s_mot.target_spd_mm_s[0] = m1;
    s_mot.target_spd_mm_s[1] = m2;
    s_mot.target_spd_mm_s[2] = m3;
    s_mot.target_spd_mm_s[3] = m4;
    if ((fabsf(m1) < 1.0f) && (fabsf(m2) < 1.0f) && (fabsf(m3) < 1.0f) && (fabsf(m4) < 1.0f)) {
        BspMotor_Stop();
    } else {
        s_driving = 1U;
    }
}

void BspMotor_SetDiffSpeed(float left_mm_s, float right_mm_s)
{
    BspMotor_SetWheelSpeed(left_mm_s, left_mm_s, right_mm_s, right_mm_s);
}

void BspMotor_SetRawPwm(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    s_raw_pwm_mode = 1U;
    s_raw_pwm_cmd[0] = clamp_i16(m1, -CFG_WHEEL_PWM_MAX, CFG_WHEEL_PWM_MAX);
    s_raw_pwm_cmd[1] = clamp_i16(m2, -CFG_WHEEL_PWM_MAX, CFG_WHEEL_PWM_MAX);
    s_raw_pwm_cmd[2] = clamp_i16(m3, -CFG_WHEEL_PWM_MAX, CFG_WHEEL_PWM_MAX);
    s_raw_pwm_cmd[3] = clamp_i16(m4, -CFG_WHEEL_PWM_MAX, CFG_WHEEL_PWM_MAX);
    if ((m1 == 0) && (m2 == 0) && (m3 == 0) && (m4 == 0)) {
        BspMotor_Stop();
    } else {
        s_driving = 1U;
        send_pwm_frame(s_raw_pwm_cmd[0], s_raw_pwm_cmd[1], s_raw_pwm_cmd[2], s_raw_pwm_cmd[3]);
    }
}

void BspMotor_Stop(void)
{
    s_driving = 0U;
    s_raw_pwm_mode = 0U;
    for (int i = 0; i < 4; ++i) {
        s_mot.target_spd_mm_s[i] = 0.0f;
        s_cmd_spd[i] = 0.0f;
        s_pid_integ[i] = 0.0f;
        s_pid_prev_err[i] = 0.0f;
        s_raw_pwm_cmd[i] = 0;
        s_stall_ms[i] = 0U;
    }
    send_pwm_frame(0, 0, 0, 0);
    s_last_zero_keep_ms = HAL_GetTick();
}

void BspMotor_EmergencyStop(void)
{
    BspMotor_Stop();
    send_pwm_frame(0, 0, 0, 0);
}

/* 故障上下文专用：不依赖 HAL、中断和 SysTick，直接写 USART2 寄存器发零 PWM。
 * 驱动板断串口会保持上次输出，所以 HardFault / Error_Handler 必须先调用它。 */
void BspMotor_PanicStop(void)
{
    static const char k_zero[] = "$pwm:0,0,0,0#";
    if ((RCC->APB1ENR & RCC_APB1ENR_USART2EN) == 0U) {
        return;     /* 串口尚未初始化，驱动板也还没收到过任何运动指令 */
    }
    for (int rep = 0; rep < 3; ++rep) {
        for (size_t i = 0U; i < (sizeof(k_zero) - 1U); ++i) {
            uint32_t guard = 200000U;
            while (((USART2->SR & USART_SR_TXE) == 0U) && (guard-- > 0U)) { }
            USART2->DR = (uint8_t)k_zero[i];
        }
    }
    uint32_t guard = 200000U;
    while (((USART2->SR & USART_SR_TC) == 0U) && (guard-- > 0U)) { }
}

void BspMotor_RequestVoltage(void)
{
    BspUart_SendStr(BSP_UART_MOTOR, "$read_vol#");
}

void BspMotor_ResetTotalOdom(void)
{
    s_mot.total_dist_mm = 0.0f;
    s_mot.seg_dist_mm = 0.0f;
    s_mot.seg_yaw_deg = 0.0f;
    s_mot.total_yaw_deg = 0.0f;
    if (s_have_base != 0U) {
        for (int i = 0; i < 4; ++i) {
            s_base_raw[i] = s_mot.raw_enc[i];
            s_mot.fwd_enc[i] = 0;
            s_prev_fwd[i] = 0;
        }
    }
}

void BspMotor_SetTotalDist(float mm)
{
    s_mot.total_dist_mm = mm;
}

void BspMotor_ResetSegOdom(void)
{
    s_mot.seg_dist_mm = 0.0f;
    s_mot.seg_yaw_deg = 0.0f;
}

void BspMotor_ClearFault(void)
{
    s_mot.rx_timeout_fault = false;
    s_mot.enc_jump_fault = false;
    s_mot.enc_stall_fault = false;
    s_mot.stall_wheel = 0U;
}

bool BspMotor_HasFault(void)
{
    return s_mot.rx_timeout_fault || s_mot.enc_jump_fault || s_mot.enc_stall_fault;
}

const motor_state_t *BspMotor_GetState(void)
{
    return &s_mot;
}

"""OpenMV H7 Plus 双轴云台空载调试，在 OpenMV IDE 中运行一次。"""

import time
from pyb import Pin, Servo

# tilt_center：只调俯仰正位；tilt：俯仰测试；pan：水平测试；both：依次测试。
# pan 和 both 适用于底部也是 PWM 位置舵机的情况。
TEST_MODE = "both"
TILT_CENTER_DEG = -5  # 实机确认的俯仰正位；舵机被撞歪，向后实际行程约 80°
# 由水平轴实测点估算：目标实际角度 × 比例 = 舵机指令角度。
PAN_COMMAND_RATIO = 60 / 90
TEST_ANGLE = 90  # 水平为目标实际幅度，俯仰为相对正位的指令幅度
ISOLATE_IDLE_SIGNAL = True  # 单轴测试时，将未参与测试的信号脚拉低


def test_axis(servo, label, center_deg=0, ratio=1):
    print("Testing", label)
    print("TEST_ANGLE =", TEST_ANGLE, "calibration =", servo.calibration())
    offsets = (0,) if TEST_MODE == "tilt_center" else (0, TEST_ANGLE, 0, -TEST_ANGLE, 0)
    for offset in offsets:
        angle = center_deg + round(offset * ratio)
        print(label, "relative_deg =", offset, "command_deg =", angle)
        servo.angle(angle, 500)
        time.sleep_ms(1200)
        print("target_deg =", angle, "driver_pulse_us =", servo.pulse_width())
        time.sleep_ms(1800)  # 停稳后留时间观察支架位置
    if TEST_MODE == "tilt_center":
        print("Holding tilt center. Stop in IDE to adjust or switch to tilt.")
        while True:
            time.sleep_ms(200)
    print(label, "finished")


if TEST_MODE not in ("tilt_center", "tilt", "pan", "both"):
    raise ValueError("TEST_MODE must be tilt_center, tilt, pan or both")

if TEST_MODE != "both" and ISOLATE_IDLE_SIGNAL:
    idle_pin = "P7" if TEST_MODE == "pan" else "P8"
    Pin(idle_pin, Pin.OUT_PP, value=0)

if TEST_MODE != "pan":
    tilt = Servo(1)
if TEST_MODE in ("pan", "both"):
    pan = Servo(2)

if TEST_MODE == "both":
    tilt.angle(TILT_CENTER_DEG)
    pan.angle(0)
    time.sleep_ms(1000)

if TEST_MODE != "pan":
    test_axis(tilt, "tilt P7", TILT_CENTER_DEG)
if TEST_MODE in ("pan", "both"):
    test_axis(pan, "pan P8", ratio=PAN_COMMAND_RATIO)

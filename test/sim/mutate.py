"""变异检查：把每处修复单独撤回（或把逻辑故意写反），确认回放能抓到。

用法（在项目根目录）：python -I stm32f407vet6/test/sim/mutate.py [名称关键字 ...]
每一项都应显示“被抓到”。源码改动后，下面的查找串可能对不上（脚本会报 AssertionError），
这时按新源码更新查找串；新增修复时在 MUTS 里加一项。需要主机 gcc。
"""
import os
import shutil
import subprocess
import sys

import tempfile
root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # stm32f407vet6
work = tempfile.mkdtemp(prefix="sim_mut_")
sim = os.path.join(root, 'test', 'sim')
SRCS = ('App/mission_fsm.c', 'App/motion_ctrl.c', 'Bsp/bsp_motor.c', 'Bsp/bsp_openmv.c', 'Bsp/gray_logic.c')

MUTS = {
    # 上一轮的 6 处
    "1 相机重启不纠正任务": [("Bsp/bsp_openmv.c", r"} else if (s_ack_pending == 0U) {", r"} else if (0) {")],
    "2 帧外字节进缓冲": [("Bsp/bsp_openmv.c", r"} else if (!s_in_frame) {", r"} else if (0) {")],
    "4 无堵转检测": [("Bsp/bsp_motor.c", r"if (s_stall_ms[i] >= CFG_STALL_MS) {", r"if (0) {")],
    "4 无动作超时": [("App/motion_ctrl.c", r"if ((uint32_t)(HAL_GetTick() - s_act_t0) > s_act_limit_ms) {", r"if (0) {")],
    "5 锁存旧方向": [("App/mission_fsm.c", r"(mv->target_task == OPENMV_TASK_TURN) && (mv->ack_task == OPENMV_TASK_TURN)) {", r"1) {"),
                     ("App/mission_fsm.c", r"BspOpenmv_ClearTurn();      /*", r"/* BspOpenmv_ClearTurn(); */ /*")],
    # 本轮
    "P 牌 flag 任意非零": [("Bsp/bsp_openmv.c", r"if (((p[0] != '0') && (p[0] != '1')) || (p[1] != ',')) {", r"if ((p[0] < '0') || (p[0] > '9') || (p[1] != ',')) {"),
                         ("Bsp/bsp_openmv.c", r"s_mv.park_seen = (flag == 1);", r"s_mv.park_seen = (flag != 0);")],
    "P 牌尾部不检查": [("Bsp/bsp_openmv.c", r"if (!parse_int(&p, 0, 160 * 120, area) || (*p != '\0')) { return false; }", r"if (!parse_int(&p, 0, 160 * 120, area)) { return false; }")],
    "P 牌 cx 不限范围": [("Bsp/bsp_openmv.c", r"if (!parse_int(&p, -100, 100, cx)", r"if (!parse_int(&p, -99999, 99999, cx)")],
    "TURN 只看首字符": [("Bsp/bsp_openmv.c", r'if ((strcmp(body + 5, "L") != 0) && (strcmp(body + 5, "R") != 0)) {', r"if ((body[5] != 'L') && (body[5] != 'R')) {")],
    "HB 尾部不检查": [("Bsp/bsp_openmv.c", r"if ((*p == '-') || !parse_int(&p, 0, 255, fps) || (*p != '\0')) { return false; }", r"if (!parse_int(&p, -99999, 99999, fps)) { return false; }")],
    "MAll 尾部不检查": [("Bsp/bsp_motor.c", r"return (endp != NULL) && (endp[0] == '#') && (endp[1] == '\0');", r"return true;")],
    "四色不带续接": [("Bsp/bsp_openmv.c", r"if ((task == OPENMV_TASK_COLOR) && (s_mv.color_count > 0U) && !s_mv.color_locked) {", r"if (0) {")],
    "灰度忽略装反": [("Bsp/gray_logic.c", r"uint8_t idx = reverse ? (uint8_t)(7U - hw) : hw;", r"uint8_t idx = hw;")],
    "灰度偏置左右写反": [("Bsp/gray_logic.c", r"if (mode == GRAY_BIAS_LEFT) {", r"if (mode == GRAY_BIAS_RIGHT) {"),
                        ("Bsp/gray_logic.c", r"} else if (mode == GRAY_BIAS_RIGHT) {", r"} else if (mode == GRAY_BIAS_LEFT) {")],
    "灰度坏通道不屏蔽": [("Bsp/gray_logic.c", r"if ((dead_mask & (1U << idx)) != 0U) {", r"if (0) {")],
    "灰度坏通道不补线": [("Bsp/gray_logic.c", r"bits |= (uint8_t)(1U << i);", r"(void)0;")],
    "循迹差速符号写反": [("App/motion_ctrl.c", r"float left_spd  = spd + diff;", r"float left_spd  = spd - diff;"),
                        ("App/motion_ctrl.c", r"float right_spd = spd - diff;", r"float right_spd = spd + diff;")],
}

ONLY = sys.argv[1:]
for name, muts in MUTS.items():
    if ONLY and not any(o in name for o in ONLY):
        continue
    d = os.path.join(work, 'm')
    shutil.rmtree(d, ignore_errors=True)
    for sub in ('App', 'Bsp', 'Core'):
        shutil.copytree(os.path.join(root, sub), os.path.join(d, sub))
    for rel, old, new in muts:
        p = os.path.join(d, rel)
        s = open(p, encoding='utf-8').read()
        assert old in s, (name, old)
        open(p, 'w', encoding='utf-8').write(s.replace(old, new, 1))
    exe = os.path.join(work, 'm.exe')
    srcs = [os.path.join(sim, 'sim.c')] + [os.path.join(d, x) for x in SRCS]
    subprocess.run(['gcc', '-std=c11', '-w', '-O0', '-I', os.path.join(sim, 'stub'),
                    '-I', os.path.join(d, 'Core', 'Inc'), '-I', os.path.join(d, 'Bsp'),
                    '-I', os.path.join(d, 'App'), '-o', exe] + srcs + ['-lm'], check=True)
    r = subprocess.run([exe], capture_output=True)
    out = r.stdout.decode('utf-8', 'replace').splitlines()
    fails = [l for l in out if l.startswith('FAIL')]
    print("%-14s -> %s" % (name, "被抓到 (%d 项失败)" % len(fails) if fails else "未被抓到!"))
    for l in fails[:2]:
        print("      " + l.split('   [')[0])

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
SRCS = ('App/mission_fsm.c', 'App/motion_ctrl.c', 'Bsp/bsp_motor.c', 'Bsp/bsp_openmv.c', 'Bsp/gray_logic.c',
        'Bsp/sonar_logic.c')

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
    "12 循迹微分不滤波": [("App/motion_ctrl.c", r"s_line_d_filt += CFG_LINE_D_LPF_ALPHA * (d_raw - s_line_d_filt);", r"s_line_d_filt = d_raw;")],
    "12 复位后首拍微分突跳": [("App/motion_ctrl.c", r"float d_raw = (s_line_have_prev && !src_switch)", r"float d_raw = (!src_switch)")],
    "13 内侧轮不能制动": [("Bsp/bsp_motor.c", r"if (tgt > 0.0f && u < -(float)CFG_WHEEL_BRAKE_PWM) { u = -(float)CFG_WHEEL_BRAKE_PWM; }", r"if (tgt > 0.0f && u < 0.0f) { u = 0.0f; }")],
    "13 制动不限幅": [("Bsp/bsp_motor.c", r"if (tgt > 0.0f && u < -(float)CFG_WHEEL_BRAKE_PWM) { u = -(float)CFG_WHEEL_BRAKE_PWM; }", r"(void)0;")],
    "14 丢线差速不单独限幅": [("App/motion_ctrl.c", r"diff = clampf(diff, -CFG_LINE_LOST_DIFF_MM_S, CFG_LINE_LOST_DIFF_MM_S);", r"(void)0;")],
    "14 正反转切换不清积分": [("Bsp/bsp_motor.c", r"if ((tgt > 0.0f) != (s_pid_prev_tgt[i] > 0.0f)) {", r"if (0) {")],
    # 2026-11-23 激光换超声、取消云台
    "15 超声不做中值": [("Bsp/sonar_logic.c", r"s->dist_mm = median3(s->med_buf[0], s->med_buf[1], s->med_buf[2]);", r"s->dist_mm = mm;")],
    "15 空旷不进中值": [("Bsp/sonar_logic.c", r"s->med_buf[s->med_idx] = mm;", r"if (mm != SONAR_FAR_MM) s->med_buf[s->med_idx] = mm;")],
    "15 超声失效不急停": [("App/mission_fsm.c", r"if (gate_ok && !BspSonar_IsHealthy(now_ms)) {", r"if (0) {")],
    "15 超声失效仍起跑": [("App/mission_fsm.c", r"(s_profile != MISSION_PROFILE_PARK_ONLY) && !BspSonar_IsHealthy(now_ms)) {", r"0) {")],
    "16 丢线不按转角停": [("App/mission_fsm.c", r"|| (Motion_GetLineLostDeg() > CFG_LINE_LOST_MAX_DEG)", r"")],
    "15 视觉任务不下发": [("App/mission_fsm.c", r"if (BspOpenmv_GetState()->target_task != task) {", r"if (0) {")],
    # 2026-11-24 路口直行、P 变增益
    "17 路口不保持误差": [("Bsp/gray_logic.c", r"    if (cross_cand || wide) {", r"    if (0) {")],
    "17 分岔误判路口": [("Bsp/gray_logic.c", r"(black_cnt >= CFG_GRAY_CROSS_MIN_BLACK) && (clusters == 1U);", r"(black_cnt >= CFG_GRAY_CROSS_MIN_BLACK);")],
    "17 多段取全部平均": [("Bsp/gray_logic.c", r"if ((mode == GRAY_BIAS_CENTER) && (clusters >= 2U)) {", r"if (0) {")],
    # “18 P 不变增益”（改回线性 P）在 CENTER=1、EDGE=0 时与原码等价、抓不到，变增益重新启用后再加回：
    # ("App/motion_ctrl.c", r"return CFG_LINE_PID_KP * err * (CFG_LINE_KP_CENTER_SCALE + CFG_LINE_KP_EDGE_GAIN * fabsf(err));", r"return CFG_LINE_PID_KP * err;")
    "20 路口保持不封顶": [("App/motion_ctrl.c", r"if (!g->line_found || flooded) {", r"if (!g->line_found) {")],
    "21 回线前不清灰度": [("App/mission_fsm.c", r"            BspGray_ResetTrack();", r"            (void)0;")],
    "19 灰度无回差": [("Bsp/gray_logic.c", r"int32_t band = was_black ? (int32_t)CFG_GRAY_HYST_ADC : -(int32_t)CFG_GRAY_HYST_ADC;", r"int32_t band = 0;")],
    "19 丢线不确认": [("Bsp/gray_logic.c", r"} else if (st->white_frames >= CFG_GRAY_CONFIRM_FRAMES) {", r"} else if (1) {")],
    "19 路口不确认": [("Bsp/gray_logic.c", r"st->is_cross = cross_cand && (st->cross_frames >= CFG_GRAY_CONFIRM_FRAMES);", r"st->is_cross = cross_cand;")],
    "19 未确认丢线时误差乱跳": [("Bsp/gray_logic.c", r"""    } else if (st->line_found) {
        /* 全白但还没确认丢线""", r"""    } else if (0) {
        /* 全白但还没确认丢线""")],
    "23 死区补偿过零硬切换": [("Bsp/bsp_motor.c", r"float dead = (float)CFG_WHEEL_PWM_DEADZONE * clampf(tgt / CFG_WHEEL_DEAD_RAMP_MM_S, -1.0f, 1.0f);", r"float dead = (tgt > 0.0f) ? (float)CFG_WHEEL_PWM_DEADZONE : -(float)CFG_WHEEL_PWM_DEADZONE;")],
    "23 目标速度不限加速度": [("Bsp/bsp_motor.c", r"        s_cmd_spd[i] = nc;", r"        s_cmd_spd[i] = tgt; (void)nc;")],
    "29 换向减速不放宽": [("Bsp/bsp_motor.c", r"nc = c + clampf(d, -dv_dec, dv_dec);", r"nc = c + clampf(d, -dv_acc, dv_acc);")],
    "29 不做左右换算": [("Bsp/bsp_motor.c", r"#define POS_CH(p)  ((CFG_MOTOR_SWAP_LR != 0U) ? ((p) ^ 2) : (p))", r"#define POS_CH(p)  (p)")],
    "23 停车不清限速目标": [("Bsp/bsp_motor.c", r"""        s_cmd_spd[i] = 0.0f;
        s_last_pwm[i] = 0;""", r"""        s_last_pwm[i] = 0;""")],
    "23 换来源时仍算微分": [("App/motion_ctrl.c", r"float d_raw = (s_line_have_prev && !src_switch)", r"float d_raw = (s_line_have_prev)")],
    "24 堵转只看目标速度": [("Bsp/bsp_motor.c", r"(abs((int)s_last_pwm[i]) >= CFG_WHEEL_PWM_DEADZONE)", r"(0)")],
    "24 从无反馈不超时": [("Bsp/bsp_motor.c", r"} else if ((s_driving != 0U) && ((uint32_t)(now_ms - s_drive_start_ms) > CFG_MOTOR_RX_TIMEOUT_MS)) {", r"} else if (0) {")],
    "24 开环绕过故障": [("Bsp/bsp_motor.c", r"} else if (BspMotor_HasFault()) {", r"} else if (0) {")],
    "24 宽线照常算误差": [("Bsp/gray_logic.c", r"const bool wide = (clusters == 1U) && (black_cnt > CFG_GRAY_LINE_MAX_BLACK);", r"const bool wide = false;")],
    "24 路口保持不限幅": [("Bsp/gray_logic.c", r"if (h > CFG_GRAY_CROSS_HOLD_MAX_ERR) { h = CFG_GRAY_CROSS_HOLD_MAX_ERR; }", r"")],
    "24 回线不看线宽": [("App/mission_fsm.c", r"g->line_found && narrow &&", r"g->line_found &&")],
    "24 进出保持仍算微分": [("App/motion_ctrl.c", r"(s_line_prev_cross != g->held);", r"false;")],
    "25 按 PWM 判堵转不看目标": [("Bsp/bsp_motor.c", r"((atgt >= CFG_STALL_LOW_TGT_MM_S) &&", r"((1) &&")],
    "25 宽线保持不封顶": [("App/motion_ctrl.c", r"""    if (g->held) {
        if (!s_in_cross) {""", r"""    if (g->cross_frames > 0U) {
        if (!s_in_cross) {""")],
    "26 回线不看分段": [("App/mission_fsm.c", r"(g->black_count <= CFG_GRAY_LINE_MAX_BLACK) && !g->is_split;", r"(g->black_count <= CFG_GRAY_LINE_MAX_BLACK);")],
    "26 保持时仍带微分": [("App/motion_ctrl.c", r"        s_line_d_filt = 0.0f;", r"        (void)0;")],
    "27 单帧白清封顶起点": [("App/motion_ctrl.c", r"    } else if (g->black_count > 0U) {", r"    } else {")],
    "27 跨赛段复位丢线统计": [("App/mission_fsm.c", r"    if (!keep_line) {", r"    if (1) {")],
    "27 丢线时也入库": [("App/mission_fsm.c", r"if ((p_aligned || odom_fallback) && !Motion_IsLineLost()) {", r"if (p_aligned || odom_fallback) {")],
    "26 闪白恢复未限幅误差": [("Bsp/gray_logic.c", r"""会把限幅撤掉、内侧轮突然反转 */
        (void)0;""", r"""会把限幅撤掉、内侧轮突然反转 */
        st->error = st->last_valid_err;""")],
    "28 重跑不清灰度": [("App/mission_fsm.c", r"""按旧误差猛转 */
    BspGray_ResetTrack();""", r"""按旧误差猛转 */
    (void)0;""")],
    "22 丢线误差一步到边界": [("Bsp/gray_logic.c", r"st->error += CFG_GRAY_LOST_ERR_SMOOTH * (target - st->error);", r"st->error = target;")],
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
    cmd = ['gcc', '-std=c11', '-w', '-O0', '-I', os.path.join(sim, 'stub'),
           '-I', os.path.join(d, 'Core', 'Inc'), '-I', os.path.join(d, 'Bsp'),
           '-I', os.path.join(d, 'App'), '-o', exe] + srcs + ['-lm']
    # 本机 gcc 偶发 internal compiler error / 临时文件写失败，与变异无关：重试两次
    for attempt in range(3):
        if subprocess.run(cmd).returncode == 0:
            break
    else:
        subprocess.run(cmd, check=True)
    r = subprocess.run([exe], capture_output=True)
    out = r.stdout.decode('utf-8', 'replace').splitlines()
    fails = [l for l in out if l.startswith('FAIL')]
    print("%-14s -> %s" % (name, "被抓到 (%d 项失败)" % len(fails) if fails else "未被抓到!"))
    for l in fails[:2]:
        print("      " + l.split('   [')[0])

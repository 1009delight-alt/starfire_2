/* 复用既有电机/相机/灰度实码回放环境，专项验证新方框倒库。 */
#define main legacy_sim_main
#include "sim.c"
#undef main

static void box_start(bool camera_online)
{
    sim_reset_world();
    if (!camera_online) { g_cam_disabled = true; }
    run_ms(1800U);
    sim_gray("00011000");
    start_stage(4);
}

int main(void)
{
    sim_reset_world(); run_ms(1000U); start_stage(0); sim_gray("11111111"); run_ms(100U);
    check(Mission_GetState() == ST_TRACK_TO_OBS, "前面赛段遇到横线不触发倒库");
    box_start(false);
    check(!BspOpenmv_GetState()->online, "确认本次相机实际离线");
    check(Mission_GetState() == ST_PARK_SEARCH, "PARK 单项不要求相机在线");
    run_ms(120U);
    check(BspOpenmv_GetState()->target_task == OPENMV_TASK_IDLE, "方框找库不请求 P 牌识别");
    q_push_str(&g_rx[BSP_UART_OPENMV], "$PARK:1,0,300#\r\n");
    run_ms(40U);
    check(Mission_GetState() == ST_PARK_SEARCH, "P 牌帧不触发灰度倒库");
    sim_gray("11111111");
    run_ms(20U);
    sim_gray("00011000");
    run_ms(60U);
    check(Mission_GetState() == ST_PARK_SEARCH, "单帧全黑干扰不触发");
    sim_gray("11111111");
    check(wait_state(ST_PARK_REVERSE, 100U), "连续横线触发第一步后退");
    sim_gray("00011000");
    float backup0 = BspMotor_GetState()->total_dist_mm;
    run_ms(40U);
    check(BspMotor_GetState()->target_spd_mm_s[0] < 0.0f &&
          BspMotor_GetState()->target_spd_mm_s[2] < 0.0f,
          "先沿原路后退，不再前进680毫米");
    for (uint32_t t = 0; t < 15000U && Mission_GetState() == ST_PARK_REVERSE &&
         Mission_GetSubStep() == 0U; t++) { run_ms(1U); }
    float backed = backup0 - BspMotor_GetState()->total_dist_mm;
    check(Mission_GetState() == ST_PARK_REVERSE && Mission_GetSubStep() == 1U &&
          fabsf(backed - CFG_PARK_BOX_BACKUP_MM) < 30.0f,
          "后退约%.0f毫米后才开始转向（实测 %.0f）", CFG_PARK_BOX_BACKUP_MM, backed);
    check(wait_state(ST_DONE, 20000U), "转向、倒车390毫米、停稳后DONE");
    check(all_pwm_zero(), "完成后电机归零");

    box_start(true);
    run_ms(200U);
    g_cam_park_aligned = true;
    check(wait_state(ST_FAULT, 35000U) && strcmp(Mission_GetFaultReason(), "BOX_MISS") == 0,
          "没有方框即使P牌存在也不倒库，到搜索上限报BOX_MISS");
    check(all_pwm_zero(), "找不到方框后停车");

    sim_reset_world(); run_ms(1000U); sim_gray("11111111"); start_stage(4); run_ms(100U);
    check(Mission_GetState() == ST_PARK_SEARCH, "起跑已经压横线时不立即触发，必须先正常窄线布防");

    box_start(true); run_ms(120U); sim_gray("11111111");
    check(wait_state(ST_PARK_REVERSE, 100U), "冻结测试进入后退");
    g_enc_frozen = true;
    check(wait_state(ST_FAULT, 3000U) && strcmp(Mission_GetFaultReason(), "ENC_STALL") == 0,
          "后退时编码器冻结仍急停");
    check(all_pwm_zero(), "冻结急停归零");

    box_start(true); run_ms(120U); sim_gray("11111111");
    check(wait_state(ST_PARK_REVERSE, 100U), "按键测试进入后退");
    g_estop = true; run_ms(40U);
    check(Mission_GetState() == ST_FAULT && strcmp(Mission_GetFaultReason(), "KEY_STOP") == 0 &&
          all_pwm_zero(), "后退途中按键立即停车");

    box_start(true); run_ms(120U);
    check(fabsf((BspMotor_GetState()->target_spd_mm_s[0] +
                 BspMotor_GetState()->target_spd_mm_s[2]) * 0.5f - CFG_SPEED_CRUISE_MM_S) < 1.0f,
          "找框基速与普通巡航一致（%.0f毫米/秒）", CFG_SPEED_CRUISE_MM_S);
    /* 约20mm横边在364mm/s下仅经过55ms，模拟50ms有效压黑，不能额外等待到漏边。 */
    sim_gray("11111111"); run_ms(50U); sim_gray("00011000");
    check(Mission_GetState() == ST_PARK_REVERSE, "提速后短横边仍能触发，不漏过50ms黑线");
    printf("\nGRAY_BOX %d/%d passed\n", g_checks - g_fails, g_checks);
    return g_fails ? 1 : 0;
}

#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/* ============================================================================
 *  第十四届“星火杯”赛题二 · 整车集中参数修改窗口 (config.h)
 *  说明：现场调试（5 分钟窗口）只需修改本文件与 main.c 顶部的 APP_RUN_MODE。
 * ============================================================================ */

/* ==================== 1. 硬件开关与安全保护 ==================== */
#define CFG_ENABLE_OLED                 1U  /* SPI2 1.8 寸 ST7735S 彩屏（PB10~PB13、PB15） */
#define CFG_ENABLE_SONAR                1U  /* PB6 Trig / PB7 Echo(TIM4_CH2) HC-SR04 超声 */
#define CFG_ENABLE_GRAY                 1U  /* PE2~PE4 + PC0(ADC1_IN10)   */
#define CFG_ENABLE_OPENMV               1U  /* PA9/PA10  USART1 <-> OpenMV*/
#define CFG_ENABLE_DEBUG_UART           0U  /* PC6/PC7   USART6 遥测打印  */

#define CFG_MOTOR_RX_TIMEOUT_MS         300U /* 运动中编码器回传超时急停(ms) */
#define CFG_REQUIRE_ENC_ONLINE_TO_START 1U   /* 1: 必须收到编码器回传才允许发车 */
#define CFG_MAIN_CTRL_PERIOD_MS         20U  /* 主控制与速度闭环周期 20ms (50Hz) */
#define CFG_OLED_REFRESH_PERIOD_MS      120U /* 屏幕重绘周期；比赛模式每 20ms 只发一条有变化的 8 行条带 */

/* 1.8 寸 ST7735S 彩屏。以下为初稿值，上电看画面再改 */
#define CFG_TFT_SPI_DIV                 4U   /* SPI2 分频：APB1 42MHz / 4 = 10.5MHz；花屏改 8，稳定可试 2 */
#define CFG_TFT_SPI_TIMEOUT_MS          10U  /* 单次 SPI 发送超时 (ms) */
#define CFG_TFT_MADCTL                  0x60U /* 横屏方向：0x60 / 0xA0 互为上下颠倒；镜像时试 0x20 / 0xE0 */
#define CFG_TFT_BGR                     1U   /* 红蓝颠倒时改 0 */
#define CFG_TFT_INVERT                  0U   /* 黑白颠倒（底色发白）时改 1 */
#define CFG_TFT_X_OFFSET                0U   /* 画面边缘有一两列花点时调偏移，常见值 1 或 2 */
#define CFG_TFT_Y_OFFSET                0U

/* 独立看门狗：主循环卡死后复位，复位后 BspMotor_Init 立即发零 PWM。
 * LSI 约 32kHz，/32 分频 -> 约 1ms/计数；LSI 偏差大，实际超时约 0.6~1.7 倍 */
#define CFG_ENABLE_IWDG                 1U
#define CFG_IWDG_RELOAD                 300U /* 约 300ms */

/* 发车前置条件（从“避障之前”的赛段起跑时检查超声；需要视觉时检查 OpenMV） */
#define CFG_REQUIRE_SONAR_TO_START      1U
#define CFG_REQUIRE_MV_ONLINE_TO_START  1U
#define CFG_SONAR_STALE_MS              300U /* 超声超过该时间没收到完整回波视为失效（约 5 个测量周期） */
#define CFG_ENC_JUMP_COUNTS             4000 /* 相邻两帧累计计数跳变超过该值，视为驱动板复位 */

/* 编码器堵转/冻结检测：某轮目标 >= MIN_TGT 而实测 < MAX_SPD 持续 STALL_MS，报 ENC_STALL 急停。
 * 覆盖“驱动板照常回帧但计数不变”（编码器插头松）和轮子被卡住两种情况 */
#define CFG_STALL_MIN_TGT_MM_S          60.0f
/* 目标低于 MIN_TGT 时，PWM 已达起步死区且目标不低于此值才按堵转计时（低速找线 35~48 mm/s 冻结要能抓到；
 * 弯里内侧轮目标几 mm/s 慢转不能误报）。2026-11-25 新增 */
#define CFG_STALL_LOW_TGT_MM_S          25.0f
#define CFG_STALL_MAX_SPD_MM_S          10.0f
#define CFG_STALL_MS                    600U

/* 定距/定角动作时间上限 = 理论时间 x 系数 + 余量，超时报 MOTION_TMO */
#define CFG_MOTION_TIMEOUT_SCALE        3.0f
#define CFG_MOTION_TIMEOUT_EXTRA_MS     1000U

/* ==================== 2. 底盘控制模式与编码器标定 ==================== */
/* 0 = VET6 读取 $MAll 自控四轮速度 PID 下发 $pwm（默认，绕开驱动板参数差异）
 * 1 = 直接下发驱动板 $spd 速度闭环（需先将驱动板 Flash 参数刷为实车 310 电机） */
#define MOTOR_MODE_MCU_PID_PWM          0U
#define MOTOR_MODE_DRIVER_SPD           1U
#define CFG_MOTOR_CTRL_MODE             MOTOR_MODE_MCU_PID_PWM

/* 物理前进方向对应的编码器原始增量符号（+1 或 -1），顺序：M1左前, M2左后, M3右前, M4右后
 * 请先运行 RUN_MODE_DBG_MOTOR_ENC 逐轮前推/架空核对！ */
/* 2026-11-21 架空实测：正 PWM 四轮均物理向前，旧符号 -1 下 fwd_enc 为负，故四路改 +1 */
#define CFG_ENC_SIGN_M1                 (+1)
#define CFG_ENC_SIGN_M2                 (+1)
#define CFG_ENC_SIGN_M3                 (+1)
#define CFG_ENC_SIGN_M4                 (+1)

/* 轮系几何与脉冲换算（先运行 RUN_MODE_DBG_MOTOR_ENC & RUN_MODE_DBG_MOTION_CAL 标定） */
#define CFG_WHEEL_DIAMETER_MM           38.4f   /* 等效直径 (mm)：轮胎外径实测 48；2026-11-23 地面指令 1000mm 实走约 800mm，按 48×0.8 折算（含打滑） */
#define CFG_ENC_COUNTS_PER_REV          1066.0f /* 单轮一圈计数：2026-11-21 手拨 M1~M4 = 1082/1078/1053/1050 取平均；第 2 步地面 1000mm 再校 */
#define CFG_ENC_COUNTS_PER_MM           (CFG_ENC_COUNTS_PER_REV / (3.1415926f * CFG_WHEEL_DIAMETER_MM))
#define CFG_TRACK_WIDTH_MM              280.0f  /* 等效轮距 (mm)：几何约 175；2026-11-23 指令 90° 实转约 45°，连同上面的直径修正折算为 175×1.6（四轮原地转有侧滑） */

/* VET6 自控速度闭环（MOTOR_MODE_MCU_PID_PWM）参数 */
#define CFG_WHEEL_PID_KP                2.2f
#define CFG_WHEEL_PID_KI                0.35f
#define CFG_WHEEL_PID_KD                0.05f
#define CFG_WHEEL_PID_FF                3.2f    /* 前馈：目标 mm/s -> 基础 PWM 系数 */
#define CFG_WHEEL_PWM_DEADZONE          350     /* 克服静摩擦的最小起步 PWM（0~3600） */
/* 死区补偿过渡带 (mm/s)：|目标| 在此以内按比例给补偿，过零不再 ±350 硬切换。2026-11-25 新增，估值；
 * 太大低速起步无力，太小过零仍会跳 */
#define CFG_WHEEL_DEAD_RAMP_MM_S        40.0f
/* 轮速目标加速度上限 (mm/s²)：每 20 ms 最多变 40 mm/s。起步、换赛段、找线降速不再一步到位。
 * 急停和 BspMotor_Stop 不受限。2026-11-25 新增，估值；转向跟不上就加大 */
#define CFG_WHEEL_ACC_MAX_MM_S2         2000.0f
#define CFG_WHEEL_PWM_MAX               2400    /* 四轮统一安全 PWM 上限（协议满量程 3600） */
#define CFG_WHEEL_SPD_MAX_MM_S          650.0f  /* 最大允许目标轮速 (mm/s) */
/* 目标为正、轮子却被车身拖得比目标快时，允许输出的最大反向（制动）PWM。0 = 旧行为（最低只到 0，只能滑行）。
 * 2026-11-23 新增：差速转弯时内侧轮被外侧轮拖着走，减不下来，速度差被抹平 */
#define CFG_WHEEL_BRAKE_PWM             200

/* M4 硬件/负载补偿比例（如实测某轮阻力明显偏大，可微调前馈系数，默认全 1.0） */
#define CFG_WHEEL_FF_SCALE_M1           1.00f
#define CFG_WHEEL_FF_SCALE_M2           1.00f
#define CFG_WHEEL_FF_SCALE_M3           1.30f   /* 2026-11-21：M3 轮胎偏紧，同 PWM 计数约为其余三轮 77%，临时补偿；机械修好后改回 1.00 */
#define CFG_WHEEL_FF_SCALE_M4           1.00f

/* ==================== 3. 八路灰度传感器参数 ==================== */
/* 1 = 压到黑线时 ADC 电压变低（常见）；0 = 压到黑线时 ADC 电压变高 */
/* 2026-11-25 PWLink 实测：白底约 160、黑线约 4040（模块现为数字电平输出），改为 0 */
#define CFG_GRAY_BLACK_IS_LOW           0U
/* 0 = 通道 0 在最左、通道 7 在最右；1 = 镜像翻转（装反时只需改这里） */
/* 2026-11-22 实测：线在车左侧时 ERR 为正，模块装反（X1 在车右侧），改为 1 */
#define CFG_GRAY_REVERSE_ORDER          1U
/* 选通 AD0~AD2 后等待模拟开关稳定的微秒数 */
#define CFG_GRAY_SETTLE_US              40U
/* 默认 8 通道黑白分割阈值（12 位 ADC：0~4095），用 RUN_MODE_DBG_GRAY 标定后回填 */
/* 2026-11-22 实测（模块 5V 供电，OUT 经 10k/20k 分压到 PC0）：白底约 1590，黑约 136，取中值；赛道布上需复测 */
/* 2026-11-25 PWLink 实测：白底 156~165、黑线 4038~4042，只有两档（数字输出），取中间 2000 */
#define CFG_GRAY_TH_DEFAULT             { 2000U, 2000U, 2000U, 2000U, 2000U, 2000U, 2000U, 2000U }
/* 黑白回差 (ADC)：上一帧是黑，读数要越过 阈值+回差 才变白；是白，要越过 阈值−回差 才变黑。
 * 黑白读数差要明显大于 2 倍回差（实测约 1590 / 136）。2026-11-24 新增，估值；设 0 即无回差 */
#define CFG_GRAY_HYST_ADC               80U
/* 丢线和路口要连续这么多帧（每帧 20 ms）才成立；1 = 单帧判定。2026-11-24 新增 */
#define CFG_GRAY_CONFIRM_FRAMES         2U
/* 丢线后找线误差每帧向 ±3.8 靠近的比例：err += k·(±3.8 − err)，时间常数约 20 ms / k。
 * 0.15 时从 0.5 起 100 ms 到 2.0、200 ms 到 2.9、300 ms 到 3.4；1 = 一步到边界（旧行为）。2026-11-25 新增 */
#define CFG_GRAY_LOST_ERR_SMOOTH        0.15f
/* 坏通道屏蔽：bit i = 屏上从左数第 i 个方块（从 0 数，已含装反翻转）。被屏蔽的通道不参与判线，
 * 两侧好通道都压线时补为压线（防止一条线被劈成两段误判分岔），否则当白。0x00 = 不屏蔽。
 * 2026-11-22：模块 X7 撞坏，屏蔽；修好后改回 0x00。
 * REVERSE_ORDER=0 时 X7 在屏上左数第 7 块（序号 6）；改成 1 以后翻到左数第 2 块（序号 1） */
#define CFG_GRAY_DEAD_MASK              (1U << 1)
/* 8 路探头位置权重（从最左 ch0 到最右 ch7） */
#define CFG_GRAY_WEIGHTS                { -3.5f, -2.5f, -1.5f, -0.5f, 0.5f, 1.5f, 2.5f, 3.5f }
/* 路口判定：压黑探头数 >= 该值且连成一段判为路口，误差保持上一拍直行（赛道路口一律直行）。
 * 正常压线 1~3 路。2026-11-24 新增，估值：车放在十字、T 字上看 DBG_GRAY 页是否亮 CRS，平时压线不能亮 */
#define CFG_GRAY_CROSS_MIN_BLACK        5U
/* 正常压线最多几路（11-25 扫线实测 1~2 路）。连成一段却比它宽，说明横线正在进来：误差保持、
 * 不更新丢线方向记忆。回线确认也要求不超过它。2026-11-25 新增 */
#define CFG_GRAY_LINE_MAX_BLACK         3U
/* 路口（及横线进入时）保持的误差上限：路口要直穿，保持值偏得再大也只给小转向。2026-11-25 新增，估值 */
#define CFG_GRAY_CROSS_HOLD_MAX_ERR     1.0f
/* 路口保持最多持续的行驶距离 (mm)：真横线约 20 mm 宽，斜压也就几十 mm。超过就不是路口
 * （阈值设错整排判黑、开进大片黑区），按丢线处理，由 LINE_LOST 停车。2026-11-24 新增，估值 */
#define CFG_LINE_CROSS_MAX_MM           80.0f

/* ==================== 4. 循迹 PID 与各赛段目标速度 (mm/s) ==================== */
#define CFG_LINE_PID_KP                 80.0f   /* 线偏误差 -> 左右差速 (mm/s) */
#define CFG_LINE_PID_KI                 0.0f
#define CFG_LINE_PID_KD                 100.0f
/* P 项按误差大小变增益（2026-11-24 新增）：P = KP × err × (CENTER + EDGE × |err|)。
 * 直道上误差在 0、±0.5 之间跳，原来每跳一格差速 ±50，车身来回摆；R30 圆弧弯误差常在 1.5~2.5，
 * 原来差速给不够、转向慢。取 0.4/0.4 时：|err| 0.5 → 0.6 倍，1.5 → 1.0 倍（与原来相同），
 * 2.5 → 1.4 倍，3 → 1.6 倍。CENTER=1、EDGE=0 即旧的线性 P。直道还摆就减 CENTER，弯里转慢就加 EDGE */
#define CFG_LINE_KP_CENTER_SCALE        1.0f
#define CFG_LINE_KP_EDGE_GAIN           0.0f
/* 微分一阶低通系数（2026-11-23 新增）：8 路误差一格 0.5 地跳，直接差分会出尖峰让直道抖。
 * 1 = 不滤波；越小越平滑但越滞后，建议 0.25~0.5。KD 乘的是“每 20 ms 的误差变化”(低通后) */
#define CFG_LINE_D_LPF_ALPHA            0.35f
/* 2026-11-23 实车：架空时差速明显，落地后基本不回正（四轮差速侧滑大），KP=200 时误差 1.3 就顶到旧限幅 260，
 * 当时放大到 450。2026-11-24 实车：试过 500（修正过猛）、250，当前用户取 400：调试基速 140 时内侧轮最多反转到
 * -260，巡航 280 时到 -120。应不小于丢线限幅 CFG_LINE_LOST_DIFF_MM_S，否则后者不起作用 */
#define CFG_LINE_DIFF_MAX_MM_S          400.0f  /* 差速限幅 (mm/s) */

#define CFG_SPEED_CRUISE_MM_S           280.0f  /* 常规直道/前两弯巡航速度 */
#define CFG_SPEED_FORK_MM_S             190.0f  /* 路牌识别与分岔口速度 */
#define CFG_SPEED_COLOR_MM_S            170.0f  /* 左侧直道识别 4 色块速度（慢而稳） */
#define CFG_SPEED_PARK_SEARCH_MM_S      160.0f  /* 顶直道找 P 牌速度 */
#define CFG_SPEED_AVOID_MM_S            200.0f  /* 避障绕行速度 */
#define CFG_SPEED_TURN_INPLACE_MM_S     150.0f  /* 原地转向轮速 */
#define CFG_SPEED_REVERSE_PARK_MM_S     130.0f  /* 倒车入库速度 */

/* 直线里程保持与原地转向精度参数 */
#define CFG_DRIVE_HEADING_KP            4.5f    /* 定距直行航向纠偏 (mm/s per deg) */
#define CFG_TURN_KP                     4.2f    /* 原地转角比例控制 (mm/s per deg) */
#define CFG_TURN_MIN_SPEED_MM_S         70.0f   /* 原地转向最低克服静摩擦速度 */
#define CFG_TURN_TOLERANCE_DEG          2.0f    /* 原地转角到位容差 (deg) */
#define CFG_DIST_TOLERANCE_MM           8.0f    /* 定距直行到位容差 (mm) */

/* 丢线保护：8 路全白后按上次方向以 CFG_LINE_LOST_SPEED_SCALE 倍基速找线，累计超过该距离即停车报 LINE_LOST */
/* 2026-11-23：0.6 倍基速加差速限幅时，丢线后画大弧，直角弯转不回来。改为接近原地转：
 * 前进很慢、差速给足。调试基速 140 时约前进 35 mm/s，左右轮约 −265 / +335 */
#define CFG_LINE_LOST_SPEED_SCALE       0.25f
#define CFG_LINE_LOST_MAX_MM            250.0f
/* 丢线后累计转过该角度仍没找到线也报 LINE_LOST。2026-11-23 新增：找线改成近原地转后，巡航 280 时
 * 前进只有约 70 mm/s，只按 250 mm 判要转约 440° 才停。直角弯约需 90°，留余量取 180，估值 */
#define CFG_LINE_LOST_MAX_DEG           180.0f
/* 丢线找线时的差速上限 (mm/s)，比 CFG_LINE_DIFF_MAX_MM_S 小，防止直角弯猛转冲过头。2026-11-23 新增，估值 */
/* 220 时直角弯转不过来（2026-11-23 实车），改 300。冲过头就往下调，转不过就往上调 */
#define CFG_LINE_LOST_DIFF_MM_S         300.0f
/* 偏差降速：基速 × (1 − k·|err|)，不低于 MIN 倍。|err|=3 时约 0.64 倍。2026-11-23 由台阶改为连续，估值 */
#define CFG_LINE_SLOW_PER_ERR           0.12f
#define CFG_LINE_SLOW_MIN_SCALE         0.45f

/* 弯道识别：最近 CFG_CORNER_WINDOW_MM 行驶内航向变化 >= CFG_CORNER_YAW_DEG 记为经过一个弯 */
#define CFG_CORNER_WINDOW_MM            400.0f
#define CFG_CORNER_YAW_DEG              60.0f

/* ==================== 5. 前向超声避障参数 (HC-SR04) ==================== */
/* 测量周期：HC-SR04 手册建议 >= 60 ms，防止上一次的余波被当成本次回波 */
#define CFG_SONAR_PERIOD_MS             60U
/* 回波宽度 (us) -> 距离 (mm)：声速 / 2。20°C 约 0.1715；现场温差大时按 0.1657+0.0003×温度(°C) 改 */
#define CFG_SONAR_MM_PER_US             0.1715f
/* 量程上限 (mm)：超过记为“前方空旷”（2000）。HC-SR04 标称 4 m，远处读数波动大，避障用不到 */
#define CFG_SONAR_MAX_MM                1500U
#define CFG_OBS_TRIG_DIST_MM            240U    /* 障碍物触发距离 (mm)，需在障碍前 50cm 区域内开始避让 */
/* 3 点中值后连续几次 <= 触发距离才启动避障。超声 60 ms 一次，比原激光 33 ms 慢：
 * 中值要 2 次近读数 + 确认 2 次 ≈ 3 次测量 ≈ 180 ms，巡航 280 mm/s 时多走约 50 mm，与原激光 3 帧相当 */
#define CFG_OBS_CONFIRM_FRAMES          2U
/* 绕障侧向：+1 = 向左绕（下直道为赛道外侧），-1 = 向右绕（下直道为赛道内侧，推荐避免出布边） */
#define CFG_AVOID_TURN_DIR              (-1)
/* 4 步绕障几何参数（需确保全程在障碍物前后左右各 500mm 区域内且不碰 100mm 方块） */
#define CFG_AVOID_OUT_ANGLE_DEG         48.0f   /* 第 1 步：向绕行侧转出角度 (deg) */
#define CFG_AVOID_OUT_DIST_MM           230.0f  /* 第 2 步：斜出脱离障碍横向宽度 (mm) */
#define CFG_AVOID_PARALLEL_DIST_MM      260.0f  /* 第 3 步：回正车身平行越过障碍物 (mm) */
#define CFG_AVOID_IN_ANGLE_DEG          45.0f   /* 第 4 步：向黑线回切角度 (deg) */
#define CFG_AVOID_REJOIN_MAX_MM         380.0f  /* 斜回寻线最大距离保护 (mm)，超出报 REJOIN_FAIL */
#define CFG_REJOIN_CONFIRM              4U      /* 连续几帧看到线且 |err| <= 阈值才算回到原线 */
#define CFG_REJOIN_MAX_ERR              1.5f

/* ==================== 6. OpenMV 安装 ==================== */
/* 2026-11-23 起取消云台，OpenMV 固定装在车右前方，同一视角看转向牌、四色和 P 牌。
 * 视觉任务切换后立即下发，不再等舵机稳定。相机脚本的 ROI 和 P 牌对齐窗 (CFG_PARK_ALIGN_CX_*) 要按新视角重标 */

/* ==================== 7. 赛段里程门控窗口 (mm) ==================== */
/* 赛道总尺寸约 4500 x 3000 mm，顺时针运行；门控用于防止非对应赛段误触发 */
#define CFG_GATE_OBS_ENABLE_MM          3200.0f /* 过前两弯进入下直道后允许超声触发避障 */
#define CFG_GATE_FORK_BIAS_START_MM     350.0f  /* 避障回线后再走多少 mm 开启分岔偏置寻线 */
#define CFG_GATE_FORK_DONE_MM           1700.0f /* 避障回线后走过此距离认为已出分岔菱形区 */
#define CFG_GATE_COLOR_ENTER_MM         1200.0f /* 出分岔后过第 3 弯进入左直道色块区里程 */
#define CFG_GATE_COLOR_EXIT_MM          2400.0f /* 色块区最大里程（超时/漏色兜底切出） */
#define CFG_GATE_PARK_ENTER_MM          1100.0f /* 四色锁存后再走多少 mm 切到找库（P 牌）视角 */

/* 分岔方向缺失：到偏置起点仍无方向时停车等待，期间每 CFG_OPENMV_CMD_RETRY_MS 发一次 QUERY；
 * 超时才用兜底方向（'L' 或 'R'），并在 OLED 记 FORK_GUESS */
#define CFG_FORK_WAIT_MS                1500U
#define CFG_FORK_FALLBACK_DIR           'R'
#define CFG_OPENMV_CMD_RETRY_MS         300U

/* 找库里程兜底：
 * 进入色区后超过 CFG_CORNER4_MIN_AFTER_COLOR_MM 才出现的弯视为第 4 弯，
 * 从第 4 弯起走过 CFG_PARK_FALLBACK_MM 仍未对齐 P 牌就按里程触发入库；
 * 若第 4 弯也没识别到，从进入色区起走过 CFG_PARK_FALLBACK_ABS_MM 触发 */
#define CFG_CORNER4_MIN_AFTER_COLOR_MM  1500.0f
#define CFG_PARK_FALLBACK_MM            2200.0f
#define CFG_PARK_FALLBACK_ABS_MM        5100.0f

/* 中途起跑时预置的总里程（按交接文档 1.3 节估算，实测后回填），让后续门控继续有效 */
#define CFG_STAGE_AVOID_START_MM        CFG_GATE_OBS_ENABLE_MM
#define CFG_STAGE_FORK_START_MM         5500.0f
#define CFG_STAGE_COLOR_START_MM        8400.0f
#define CFG_STAGE_PARK_START_MM         11500.0f

/* ==================== 8. 倒车入库几何参数 ==================== */
/* OpenMV 上报 $PARK:1,cx,area#：当 area >= 阈值 且 cx 进入对齐窗，锁定 P 牌触发点 */
#define CFG_PARK_MIN_AREA_PX            160U
#define CFG_PARK_FRESH_MS               150U    /* 触发入库时最新 PARK 帧（flag=1）必须在多久之内 */
#define CFG_PARK_ALIGN_CX_MIN           (-25)   /* P 牌进入画面中心偏右/偏左触发窗口 (-100~100) */
#define CFG_PARK_ALIGN_CX_MAX           35
#define CFG_PARK_EXTRA_FWD_MM           240.0f  /* 看到对齐点后继续前移，使车中心对齐车库中线 */
#define CFG_PARK_TURN_ANGLE_DEG         90.0f   /* 原地左转 90°（正值为左转，使车尾正对右侧车库） */
#define CFG_PARK_BACK_DIST_MM           360.0f  /* 车尾先入直线倒车距离 (mm)，保证车身完全进 40x40cm 框 */
#define CFG_PARK_HOLD_MS                5500U   /* 赛题要求停稳至少 5s，留 0.5s 余量 */

#endif /* CAR_CONFIG_H */

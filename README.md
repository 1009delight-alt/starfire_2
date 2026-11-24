# 立创天空星 STM32F407VET6 开发工程

直接打开 `SkyStar.code-workspace`。这是 VS Code Keil Studio 使用的原生 CMSIS Solution 工程，已配置 Arm Compiler 6、CMSIS、STM32 HAL、启动文件、链接文件、代码补全及下载调试入口。

## 开始使用

1. 打开工作区后，等待右下角 Arm Tools 环境激活完成；如 VS Code 提示工作区信任，请按自己的判断选择。
2. 按 `Ctrl+Shift+B` 执行 `Build Debug`。也可以在左侧 CMSIS 视图使用 Build。
3. 接好 PWLink2 下载器（CMSIS-DAP）的 SWD 接口，在“运行和调试”中选择 `PWLink2(CMSIS-DAP)@pyOCD (launch)`，按 `F5` 下载并停在 `main`，再按 `F5` 继续运行。每次修改程序后先重新编译。
4. LED 每 500 ms 翻转一次；按住 PA0 用户按键时常亮。调试时可观察 `heartbeat_count`、`SystemCoreClock`、`uwTick`。

默认选择 `CMSIS-DAP@pyOCD`（PWLink2），SWD 频率为 1 MHz，pyOCD 用 `--probe cmsisdap:` 选择探针。Keil Studio 根据 `SkyStar.csolution.yml` 自动生成下载和调试配置。接上后可执行 `CMSIS TargetInfo` 任务或 `pyocd list` 确认能识别到 PWLink2。

## 板卡参数与连接

| 项目 | 配置 |
| --- | --- |
| MCU | STM32F407VET6；支持包中的型号为 STM32F407VETx |
| 内核 | Cortex-M4F，启用单精度 FPU |
| Flash | 512 KB，起始地址 `0x08000000` |
| 普通 SRAM | 128 KB，起始地址 `0x20000000`，当前默认使用 |
| CCM RAM | 64 KB，起始地址 `0x10000000`，当前未用于数据分配；不可作为 DMA 缓冲区 |
| 时钟 | HSE 8 MHz → PLL 168 MHz；APB1 42 MHz、APB2 84 MHz |
| 用户 LED | PB2，高电平点亮 |
| 用户按键 | PA0，按下为高电平，配置下拉 |
| 下载接口 | SWDIO → PA13，SWCLK → PA14，GND 共地；可连接 NRST 以便复位 |

开发板正常供电，PWLink2 的 VREF/VCC 检测脚应连接目标板 3.3 V 作为电平参考。具体供电输出脚的接法以下载器说明为准。正常从 Flash 启动时不要按住 BOOT 按键。核心板 Type-C 可供电及 USB 通信，下载调试使用外接 PWLink2。

本工程使用 STM32 HAL，未配置串口、USB、RTOS 或小车外设。外设可在 `Core/Src/board.c` 初始化，在 `Core/Inc/stm32f4xx_hal_conf.h` 启用 HAL 模块，并将对应驱动源文件加入 `SkyStar.cproject.yml`。完整 HAL 驱动代码已放入 `Drivers`，无需再次下载。

## 文件与输出

- `Core/Src/main.c`：应用入口和 LED／按键示例。
- `Core/Src/board.c`：168 MHz 时钟、GPIO 和错误处理。
- `Core/Inc/stm32f4xx_hal_conf.h`：HAL 模块开关。
- `SkyStar.csolution.yml`：芯片、支持包、编译器及调试适配器。
- `SkyStar.cproject.yml`：源文件及头文件路径。
- `out/SkyStar/SkyStar/Debug/`、`out/SkyStar/SkyStar/Release/`：对应配置的 `SkyStar.axf`、`SkyStar.hex`、`SkyStar.bin` 和 MAP 文件。
- `.packs/`：工程本地的 CMSIS 支持包；`.tools/`：下载缓存及验证日志。两者在资源管理器中隐藏，请保留。

命令行编译：

```powershell
.\scripts\build.ps1 -Configuration Debug
.\scripts\build.ps1 -Configuration Release
.\scripts\build.ps1 -Configuration Debug -Rebuild
```

脚本优先使用 Keil Studio 插件自带的 CMSIS 工具，本机其他构建工具通过 Arm Tools Environment Manager 管理。工程设置中的支持包和编译器路径针对当前电脑配置；移动工程后需要修改 `.vscode/settings.json` 中的对应路径。

## 版本与验证

2026-11-16 已完成 Debug、Release 实际编译，并使用 Keil Studio 插件自带的构建工具验证 Debug。pyOCD 已识别配置中的 STM32F407VETx 目标。当前没有检测到已连接的下载器，尚未进行实板烧录、LED 观察和断点测试。

官方 MDK 启动文件仍使用 ARMASM 语法，首次编译可能显示 ARMASM 弃用提示；当前编译通过。发生时钟初始化错误时程序进入 `Error_Handler`，可查看调试调用栈。

依赖版本：Arm Compiler 6.24.0；ARM CMSIS 6.3.0；Keil STM32F4xx_DFP 3.1.1；ST CMSIS Device F4 v2.6.11；ST STM32F4 HAL v1.8.6。官方库许可证随代码保留，具体版本记录见 `Drivers/source-versions.json`。

参考资料：

- [立创天空星 LED 官方说明](https://wiki.lckfb.com/zh-hans/tkx/tkx-stm32f407vxt6/beginner/led-principle.html)
- [立创天空星核心板硬件资料](https://openkits-wiki.easyeda.com/zh-hans/fdb/skystar.html)
- [立创官方板卡引脚说明](https://wiki.lckfb.com/zh-hans/fdb/micropython/introduction.html)
- [Keil Studio 安装与环境说明](https://mdk-packs.github.io/vscode-cmsis-solution-docs/installation.html)
- [ST CMSIS Device F4](https://github.com/STMicroelectronics/cmsis-device-f4/tree/v2.6.11)
- [ST STM32F4 HAL](https://github.com/STMicroelectronics/stm32f4xx-hal-driver/tree/v1.8.6)

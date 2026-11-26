# 主控状态机电脑回放：用主机 gcc 编译真实的状态机、运动控制、电机与视觉驱动，加上 sim.c 的模拟外设。
# 用法：在 stm32f407vet6 目录下执行 .\test\sim\run_sim.ps1；加 -SpeedScale 1.5 按该速度系数回放（覆盖 config.h）
param([string]$SpeedScale = "")
$ErrorActionPreference = "Continue"   # gcc 的警告写到 stderr，PowerShell 5.1 不能把它当异常
$root = Resolve-Path (Join-Path $PSScriptRoot "..\..")
$sim  = $PSScriptRoot
$out  = Join-Path $sim "sim.exe"
$src  = @(
    "$sim\sim.c",
    "$root\App\mission_fsm.c",
    "$root\App\motion_ctrl.c",
    "$root\Bsp\bsp_motor.c",
    "$root\Bsp\bsp_openmv.c",
    "$root\Bsp\gray_logic.c",
    "$root\Bsp\sonar_logic.c"
)
$defs = @()
if ($SpeedScale -ne "") { $defs += "-DCFG_SPEED_SCALE=$($SpeedScale)f" }
& gcc @defs -DCFG_PARK_USE_GRAY_BOX=0 -std=c11 -Wall -Wextra -Wno-unused-function -O1 -I "$sim\stub" -I "$root\Core\Inc" -I "$root\Bsp" -I "$root\App" -o $out @src -lm
if ($LASTEXITCODE -ne 0) { throw "编译失败" }
& $out
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
# 旧P牌回放继续保留；新默认灰度方框流程单独使用同一实码环境检查。
$src[0] = "$sim\gray_box_sim.c"
$out = Join-Path $sim "gray_box_sim.exe"
& gcc @defs -std=c11 -Wall -Wextra -Wno-unused-function -O1 -I "$sim\stub" -I "$root\Core\Inc" -I "$root\Bsp" -I "$root\App" -o $out @src -lm
if ($LASTEXITCODE -ne 0) { throw "灰度方框编译失败" }
& $out
exit $LASTEXITCODE

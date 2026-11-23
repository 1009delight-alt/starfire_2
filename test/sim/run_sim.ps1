# 主控状态机电脑回放：用主机 gcc 编译真实的状态机、运动控制、电机与视觉驱动，加上 sim.c 的模拟外设。
# 用法：在 stm32f407vet6 目录下执行 .\test\sim\run_sim.ps1
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
    "$root\Bsp\gray_logic.c"
)
& gcc -std=c11 -Wall -Wextra -Wno-unused-function -O1 -I "$sim\stub" -I "$root\Core\Inc" -I "$root\Bsp" -I "$root\App" -o $out @src -lm
if ($LASTEXITCODE -ne 0) { throw "编译失败" }
& $out
exit $LASTEXITCODE

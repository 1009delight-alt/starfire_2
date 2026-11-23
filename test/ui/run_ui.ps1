# 屏幕页面电脑预览：用主机 gcc 编译真实的 bsp_oled.c 与 ui_pages.c，SPI 输出交给 ST7735 模拟器，
# 检查分条带刷新，并把比赛页和 7 个调试页渲染成 test/ui/out/pages.png（放大 3 倍的拼图）。
# 用法：在 stm32f407vet6 目录下执行 .\test\ui\run_ui.ps1（需要 gcc 和 python）
$ErrorActionPreference = "Continue"
$root = Resolve-Path (Join-Path $PSScriptRoot "..\..")
$ui   = $PSScriptRoot
$out  = Join-Path $ui "out"
New-Item -ItemType Directory -Force $out | Out-Null
$exe  = Join-Path $out "render.exe"
$src  = @("$ui\render.c", "$root\Bsp\bsp_oled.c", "$root\App\ui_pages.c")
& gcc -std=c11 -Wall -Wextra -O1 -I "$ui\stub" -I "$root\Core\Inc" -I "$root\Bsp" -I "$root\App" -o $exe @src
if ($LASTEXITCODE -ne 0) { throw "编译失败" }
& $exe $out
$rc = $LASTEXITCODE
& python -I "$ui\ppm2png.py" $out
exit $rc

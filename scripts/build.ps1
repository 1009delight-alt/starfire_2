param(
    [ValidateSet('Debug', 'Release')][string]$Configuration = 'Debug',
    [switch]$Rebuild
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
$artifactRoot = Join-Path $env:USERPROFILE '.vcpkg\artifacts'
function Find-ArmTool([string]$pattern) {
    $found = Get-ChildItem -Path (Join-Path $artifactRoot $pattern) -File -ErrorAction SilentlyContinue |
        Sort-Object FullName -Descending | Select-Object -First 1
    if (!$found) { throw "Missing tool: $pattern. Open the project in VS Code and activate Arm Tools Environment." }
    return $found.FullName
}
$bundledTool = Get-ChildItem -Path (Join-Path $env:USERPROFILE '.vscode\extensions\arm.cmsis-csolution-*\tools\cmsis-toolbox\bin\cbuild.exe') -File -ErrorAction SilentlyContinue |
    Sort-Object LastWriteTime -Descending | Select-Object -First 1
if ($bundledTool) { $cbuild = $bundledTool.FullName }
else { $cbuild = Find-ArmTool '*\tools.open.cmsis.pack.cmsis.toolbox\2.14.1\bin\cbuild.exe' }
$armclang = Find-ArmTool '*\compilers.arm.armclang\6.24.0\bin\armclang.exe'
$cmake = Find-ArmTool '*\tools.kitware.cmake\4.3.3\bin\cmake.exe'
$ninja = Find-ArmTool '*\tools.ninja.build.ninja\1.13.2\ninja.exe'
$env:CMSIS_PACK_ROOT = Join-Path $projectRoot '.packs'
$env:AC6_TOOLCHAIN_6_24_0 = Split-Path $armclang -Parent
$env:PATH = @((Split-Path $cbuild -Parent), (Split-Path $cmake -Parent),
    (Split-Path $ninja -Parent), $env:AC6_TOOLCHAIN_6_24_0, $env:PATH) -join ';'
Push-Location $projectRoot
try {
    $buildArgs = @('SkyStar.csolution.yml', '--context', "SkyStar.$Configuration+SkyStar", '--update-rte')
    if ($Rebuild) { $buildArgs += '--rebuild' }
    & $cbuild @buildArgs
    if ($LASTEXITCODE -ne 0) { throw "Build failed ($LASTEXITCODE)." }
    if ($Configuration -eq 'Debug') {
        & $cbuild setup 'SkyStar.csolution.yml' --active SkyStar --update-rte
        if ($LASTEXITCODE -ne 0) { throw "Debug setup failed ($LASTEXITCODE)." }
    }
} finally {
    Pop-Location
}

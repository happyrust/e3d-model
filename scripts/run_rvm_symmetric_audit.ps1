#requires -Version 5.1
<#
.SYNOPSIS
    rvm-audit5 那 21 条 BRAN 批量跑「RVM ↔ RVM」，并与旧的「快照 ↔ elements.json」逐件对照。

.DESCRIPTION
    这是 `docs/2026-09-04-rvm-export-plan.md` M3 的批量验收：新路径必须与旧路径**逐件同档**。
    单个样本上一致说明不了什么——旧路径退回世界帧、新路径走局部帧这类差别，只有在
    带旋转的构件上才现形，所以要拿整批过。

    每条样本跑三步：
      1. gen_ams --rvm            生成侧同时出 elements.json 与 generated.rvm
      2. rvm_compare --generated  新路径：两份 .rvm 进同一个读法
      3. rvm_compare --facets     旧路径：快照 + rvm_facets_dump 的面片
    然后逐件比两份报告的 verdict 与 max_axis_delta_mm。

    `rvm_compare` 判 off 时走非零退出，那是**结果**不是脚本故障，所以不用 -ErrorAction Stop。

.EXAMPLE
    pwsh -File scripts/run_rvm_symmetric_audit.ps1
#>
param(
    [string]$Tag = 'rvm-symmetric-20260904',
    [string]$Sesno = '264'
)

$ErrorActionPreference = 'Continue'
$repo = Split-Path -Parent $PSScriptRoot
Set-Location $repo

$ams = 'D:\AVEVA\Projects\E3D3.1\AvevaMarineSample\ams000'
# ATTA 的 SPRE 指进共享目录工程；只点名这一个文件，别把整个 acp000 交给 --project-dir
# （两个工程的 7000/8189/8190/8191 同号，扫描器分不出正身就整组不外交）。
$acp7320 = 'D:\AVEVA\Projects\E3D3.1\AvevaCatalogue\acp000\acp7320_0001'
$template = 'E:\reverse\e3d\shadow_e3d31_aps_all'
$attlib = Join-Path $template 'attlib.dat'
$audit = 'D:\work\plant-code\old\gen-model\output\rvm-audit5'

# CARGO_TARGET_DIR 是共享池，别的会话随时会换掉里面的 exe：开跑前先钉一份副本。
$targetDir = cargo metadata --format-version 1 --no-deps | ConvertFrom-Json |
    Select-Object -ExpandProperty target_directory
$outRoot = Join-Path $repo "out\$Tag"
$pinned = Join-Path $outRoot 'bin'
New-Item -ItemType Directory -Force -Path $pinned | Out-Null
foreach ($name in @('gen_ams.exe', 'rvm_compare.exe')) {
    $src = Join-Path $targetDir "release\$name"
    if (-not (Test-Path $src)) { throw "缺可执行文件 $src，先 cargo build --release" }
    Copy-Item $src (Join-Path $pinned $name) -Force
}
$genAms = Join-Path $pinned 'gen_ams.exe'
$rvmCompare = Join-Path $pinned 'rvm_compare.exe'

# 每份 <stem>.rvm 配一份 .rvm.json / .facets.json，refno 由 driver 日志给出。
$cases = Get-ChildItem $audit -Filter '*.rvm' | ForEach-Object {
    $stem = $_.BaseName
    $log = Join-Path $audit "driver-$stem\check-driver.log"
    if (-not (Test-Path $log)) { return }
    $refno = [regex]::Match((Get-Content $log -Raw), 'Ref\s+=(\d+/\d+)').Groups[1].Value
    if (-not $refno) { return }
    [pscustomobject]@{
        Stem     = $stem
        Refno    = "=$refno"
        Rvm      = $_.FullName
        Snapshot = Join-Path $audit "$stem.rvm.json"
        Facets   = Join-Path $audit "$stem.facets.json"
    }
}
Write-Host "样本 $($cases.Count) 条"

$rows = @()
foreach ($case in $cases) {
    $caseOut = Join-Path $outRoot $case.Stem
    New-Item -ItemType Directory -Force -Path $caseOut | Out-Null

    $genLog = & $genAms --attlib $attlib --db (Join-Path $ams 'ams8000_0001') `
        --template $template --sesno $Sesno --root $case.Refno --out $caseOut `
        --project-dir $ams --catalogue $acp7320 --rvm 2>&1
    Set-Content -Path (Join-Path $caseOut 'gen.log') -Value $genLog -Encoding utf8
    if ($LASTEXITCODE -ne 0) {
        $rows += [pscustomobject]@{ Stem = $case.Stem; Status = "gen 失败($LASTEXITCODE)" }
        continue
    }

    $newOut = Join-Path $caseOut 'compare-rvm2rvm.json'
    & $rvmCompare --baseline $case.Rvm --generated (Join-Path $caseOut 'generated.rvm') `
        --out $newOut *> (Join-Path $caseOut 'compare-rvm2rvm.log')
    $newExit = $LASTEXITCODE

    $oldOut = Join-Path $caseOut 'compare-old.json'
    $oldArgs = @('--baseline', $case.Snapshot, '--elements', (Join-Path $caseOut 'elements.json'), '--out', $oldOut)
    if (Test-Path $case.Facets) { $oldArgs += @('--facets', $case.Facets) }
    & $rvmCompare @oldArgs *> (Join-Path $caseOut 'compare-old.log')
    $oldExit = $LASTEXITCODE

    $rows += [pscustomobject]@{
        Stem    = $case.Stem
        NewJson = $newOut
        OldJson = $oldOut
        NewExit = $newExit
        OldExit = $oldExit
    }
}

$rows | ConvertTo-Json -Depth 4 | Set-Content (Join-Path $outRoot 'runs.json') -Encoding utf8
Write-Host "`n逐件对照交给 tools/compare_reports.py："
Write-Host "  python tools/compare_reports.py `"$outRoot`""

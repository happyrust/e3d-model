#requires -Version 5.1
# 抽查 ams8000 的 BRAN：逐条按 rvm-audit5 的 E3D RVM 快照对拍。
# 用法： pwsh -File scripts/run_bran_rvm_audit.ps1 [-Tag bran-rvm-20260904]
param(
    [string]$Tag = 'bran-rvm-20260904'
)

# rvm_compare 判 off 时走 stderr + 非零退出，那是本脚本要收集的结果而不是脚本故障，
# 所以不能用 Stop：逐条看 $LASTEXITCODE，脚本自身的前置条件仍用 throw 挡。
$ErrorActionPreference = 'Continue'
$repo = Split-Path -Parent $PSScriptRoot
Set-Location $repo

$ams = 'D:\AVEVA\Projects\E3D3.1\AvevaMarineSample\ams000'
# ATTA 的 SPRE 指进共享目录工程（`/AVEVA-ATTA-SPEC/…` 在 db 7320，只存在于 AvevaCatalogue）。
# 只点名这一个文件，不把整个 acp000 交给 --project-dir：两个工程的 7000/8189/8190/8191
# 同号，扫描器分不出正身就整组不外交，26229 的 P 点会连 db 7000 都读不到。
$acp7320 = 'D:\AVEVA\Projects\E3D3.1\AvevaCatalogue\acp000\acp7320_0001'
$template = 'E:\reverse\e3d\shadow_e3d31_aps_all'
$attlib = Join-Path $template 'attlib.dat'
$audit = 'D:\work\plant-code\old\gen-model\output\rvm-audit5'
# 整目录注册，不点名 --catalogue：ATTA 的 SPRE→SCOM 链会走到 ams5100/5200/7741 这些
# 不在管件五库里的目录库，少开一个就把 GTYP 解成 None，那些站点被当成穿过、管身少画。

# 本机 CARGO_TARGET_DIR 指向共享池，别的会话随时会换掉里面的 exe：开跑前先钉一份副本。
$targetDir = cargo metadata --format-version 1 --no-deps | ConvertFrom-Json | Select-Object -ExpandProperty target_directory
$pinned = Join-Path $repo "out\$Tag\bin"
New-Item -ItemType Directory -Force -Path $pinned | Out-Null
$genAms = Join-Path $pinned 'gen_ams.exe'
$rvmCompare = Join-Path $pinned 'rvm_compare.exe'
foreach ($name in @('gen_ams.exe', 'rvm_compare.exe')) {
    $src = Join-Path $targetDir "release\$name"
    if (-not (Test-Path $src)) { throw "缺可执行文件 $src，先 cargo build --release" }
    Copy-Item $src (Join-Path $pinned $name) -Force
}

# 基准清单：每份 <seq>-<名字>.rvm.json 都配一份 .facets.json，refno 由 driver 日志给出。
$cases = Get-ChildItem $audit -Filter '*.rvm.json' | ForEach-Object {
    $stem = $_.Name -replace '\.rvm\.json$', ''
    $log = Join-Path $audit "driver-$stem\check-driver.log"
    if (-not (Test-Path $log)) { return }
    $text = Get-Content $log -Raw
    $refno = [regex]::Match($text, 'Ref\s+=(\d+/\d+)').Groups[1].Value
    if (-not $refno) { return }
    [pscustomobject]@{
        Stem     = $stem
        Refno    = "=$refno"
        Baseline = $_.FullName
        Facets   = Join-Path $audit "$stem.facets.json"
    }
}

$outRoot = Join-Path $repo "out\$Tag"
New-Item -ItemType Directory -Force -Path $outRoot | Out-Null
$rows = @()

foreach ($case in $cases) {
    $caseOut = Join-Path $outRoot $case.Stem
    $genArgs = @(
        '--attlib', $attlib,
        '--db', (Join-Path $ams 'ams8000_0001'),
        '--template', $template,
        '--sesno', '264',
        '--root', $case.Refno,
        '--out', $caseOut,
        '--project-dir', $ams,
        '--catalogue', $acp7320
    )

    $genLog = & $genAms @genArgs 2>&1
    $genExit = $LASTEXITCODE
    Set-Content -Path (Join-Path $caseOut 'gen.log') -Value $genLog -Encoding utf8 -ErrorAction SilentlyContinue
    if ($genExit -ne 0) {
        $rows += [pscustomobject]@{ Stem = $case.Stem; Refno = $case.Refno; Status = "gen 失败($genExit)" }
        continue
    }

    $cmpArgs = @(
        '--baseline', $case.Baseline,
        '--elements', (Join-Path $caseOut 'elements.json'),
        '--out', (Join-Path $caseOut 'rvm-compare.json')
    )
    if (Test-Path $case.Facets) { $cmpArgs += @('--facets', $case.Facets) }
    $cmpLog = & $rvmCompare @cmpArgs 2>&1
    $cmpExit = $LASTEXITCODE
    Set-Content -Path (Join-Path $caseOut 'rvm-compare.log') -Value $cmpLog -Encoding utf8

    $report = Get-Content (Join-Path $caseOut 'rvm-compare.json') -Raw | ConvertFrom-Json
    $sum = @{ both = 0; exact = 0; chord = 0; artifact = 0; off = 0; onlyBase = 0; onlyGen = 0 }
    foreach ($noun in $report.by_noun.PSObject.Properties) {
        $t = $noun.Value
        $sum.both += $t.both; $sum.exact += $t.exact; $sum.chord += $t.chord
        $sum.artifact += $t.reference_artifact; $sum.off += $t.off
        $sum.onlyBase += $t.only_baseline; $sum.onlyGen += $t.only_generated
    }
    $worst = ($report.pairs | Select-Object -First 1).max_axis_delta_mm
    $rows += [pscustomobject]@{
        Stem     = $case.Stem
        Refno    = $case.Refno
        Status   = if ($cmpExit -eq 0) { 'PASS' } else { 'OFF' }
        Both     = $sum.both
        Exact    = $sum.exact
        Chord    = $sum.chord
        Artifact = $sum.artifact
        Off      = $sum.off
        OnlyBase = $sum.onlyBase
        OnlyGen  = $sum.onlyGen
        WorstMm  = if ($null -ne $worst) { [math]::Round($worst, 4) } else { $null }
    }
}

$rows | Format-Table -AutoSize | Out-String -Width 200
$rows | ConvertTo-Json -Depth 3 | Set-Content -Path (Join-Path $outRoot 'summary.json') -Encoding utf8
"总计 {0} 条支管：PASS {1} / OFF {2}" -f $rows.Count,
    (@($rows | Where-Object Status -eq 'PASS').Count),
    (@($rows | Where-Object Status -ne 'PASS').Count)

#requires -Version 5.1
<#
.SYNOPSIS
    recheck-3 N0-1：工作树复跑全部语料库，把每本账刷成当前这版代码的数。

.DESCRIPTION
    这个脚本存在的理由是 recheck-4 §四那句话：`not_wired` 要变成「每次复跑自动求值」
    的差集，而不是有人隔几周去反编译里翻一遍。差集的判据已经在
    `core3d_registry::is_not_wired` 里了，缺的只是**定期把它跑在真语料上**。

    每个库一次全量生成，收 `report.json` 的六本账（visited / consumed / generated /
    catalog_pending / unknown_nouns / not_wired）+ `accounts_for` 是否平。

    `--sesno` 一律不给（取库的最新会话）：这个脚本量的是**账**不是几何回归，
    钉会话是 RVM 对拍那条线的事（见 run_bran_rvm_audit.ps1）。

.EXAMPLE
    pwsh -File scripts/run_corpus_census.ps1
    pwsh -File scripts/run_corpus_census.ps1 -Tag n0-1-20260905 -Only ams8000,aps250209
#>
param(
    [string]$Tag = ('census-' + (Get-Date -Format 'yyyyMMdd')),
    [string[]]$Only = @()
)

$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
Set-Location $repo

$marine = 'D:\AVEVA\Projects\E3D3.1\AvevaMarineSample\ams000'
$plant = 'D:\AVEVA\Projects\E3D3.1\AvevaPlantSample\aps000'
$template = 'E:\reverse\e3d\shadow_e3d31_aps_all'
$attlib = Join-Path $template 'attlib.dat'

# 每个库配自己的 --project-dir（只扫本层）。两个样例工程不能一起给：7000/8189/8190/8191
# 同号，扫描器分不出正身就整组不外交，目录链会连 db 7000 都读不到。
#
# 结构型材 / 墙目录只在 AvevaCatalogue 里（SCTN、GENSEC、STWALL 的 SPRE 都指进去），两个样例
# 工程的 --project-dir 都扫不到，而 acp000 也不能整目录给（7000 与 ams7000 同号又撞上）。
# 所以逐个文件显式钉——2026-09-05 的第一版 N0-1 一个都没钉，aps250110 的 GENSEC 828 /
# SCTN 20 / STWALL 56、aps250209 的 GENSEC 2698 全落在 catalogue_chain_broken 里，
# 而第一版表没有这一列，账面上看不出来。缺哪一个 db 看 report.json 的 catalogue_chain_broken：
#   7001   → 六库通用（ams1112 / ams7997 的 STWALL、aps250110 的 GENSEC/SCTN/STWALL）
#   250701 → aps250110 585 件、aps250209 2698 件 GENSEC 的 SPRE（SPCO 在这里）
#   250700 → 上面那些 SPCO 的 CATR 第 1 跳落在这里（SPRF）；只钉 250701 不钉它，链在第 1 跳断
#   7047   → aps250110 96 件
# ams7997 还有 481 只 SCTN 指向 db 7600：本机只有 aps000\aps7600 与 ZDJ\zdj7600，都不是
# 海工工程的，这里不钉——那是语料自身的悬空引用，钉错一个库比留着更糟。
$acp = 'D:\AVEVA\Projects\E3D3.1\AvevaCatalogue\acp000'
$catalogues = @(
    "$acp\acp7001_0001", "$acp\acp7047_0001",
    "$acp\acp250701_0001", "$acp\acp250700_0001"
)
$catalogueArgs = $catalogues | ForEach-Object { @('--catalogue', $_) }
$corpus = @(
    [pscustomobject]@{ Name = 'ams1112'; Db = "$marine\ams1112_0001"; Dir = $marine }
    [pscustomobject]@{ Name = 'ams7997'; Db = "$marine\ams7997_0001"; Dir = $marine }
    [pscustomobject]@{ Name = 'ams8000'; Db = "$marine\ams8000_0001"; Dir = $marine }
    [pscustomobject]@{ Name = 'aps250110'; Db = "$plant\aps250110_0001"; Dir = $plant }
    [pscustomobject]@{ Name = 'aps250164'; Db = "$plant\aps250164_0001"; Dir = $plant }
    [pscustomobject]@{ Name = 'aps250209'; Db = "$plant\aps250209_0001"; Dir = $plant }
)
if ($Only.Count -gt 0) { $corpus = $corpus | Where-Object { $Only -contains $_.Name } }

# 本机 CARGO_TARGET_DIR 指向共享池，别的会话随时会换掉里面的 exe：开跑前先钉一份副本，
# 免得六个库里前三个跑的是一版代码、后三个跑的是另一版，而表上看不出来。
$targetDir = cargo metadata --format-version 1 --no-deps | ConvertFrom-Json |
    Select-Object -ExpandProperty target_directory
$outRoot = Join-Path $repo "out\$Tag"
$pinned = Join-Path $outRoot 'bin'
New-Item -ItemType Directory -Force -Path $pinned | Out-Null
$src = Join-Path $targetDir 'release\gen_ams.exe'
if (-not (Test-Path $src)) { throw "缺 $src，先 cargo build --release --bin gen_ams" }
$genAms = Join-Path $pinned 'gen_ams.exe'
Copy-Item $src $genAms -Force
"钉住 {0}（{1}）" -f $genAms, (Get-Item $src).LastWriteTime

$rows = @()
foreach ($lib in $corpus) {
    $caseOut = Join-Path $outRoot $lib.Name
    New-Item -ItemType Directory -Force -Path $caseOut | Out-Null
    $log = & $genAms --attlib $attlib --db $lib.Db --template $template `
        --out $caseOut --project-dir $lib.Dir @catalogueArgs 2>&1
    $exit = $LASTEXITCODE
    Set-Content -Path (Join-Path $caseOut 'gen.log') -Value $log -Encoding utf8

    if ($exit -ne 0) {
        $rows += [pscustomobject]@{ Lib = $lib.Name; Status = "gen 失败($exit)" }
        continue
    }

    $r = Get-Content (Join-Path $caseOut 'report.json') -Raw | ConvertFrom-Json
    $sum = { param($m) if ($null -eq $m) { 0 } else { ($m.PSObject.Properties | Measure-Object -Property Value -Sum).Sum } }
    $rows += [pscustomobject]@{
        Lib       = $lib.Name
        Status    = if ($log -match '账已平') { 'BALANCED' } else { '账未判' }
        Visited   = $r.visited
        Consumed  = & $sum $r.consumed
        Generated = $r.generated
        Catalog   = & $sum $r.catalog_pending
        Unknown   = & $sum $r.unknown_nouns
        NotWired  = & $sum $r.not_wired
        # 这两本是「E3D 画了、我们没画」的另两种形态：中心线只画线、目录链断在库没开。
        # 第一版表没有它们，于是 aps250110 少了 900 件几何也看不出来（见 $catalogues）。
        Centreline  = & $sum $r.centreline_only
        ChainBroken = @($r.catalogue_chain_broken).Count
    }
}

$rows | Format-Table -AutoSize | Out-String -Width 200
$rows | ConvertTo-Json -Depth 3 | Set-Content -Path (Join-Path $outRoot 'summary.json') -Encoding utf8

# 逐 noun 的 not_wired 合表：这张表就是 recheck-4 §五 W4 的工作队列，按件数排序。
$byNoun = @{}
foreach ($lib in $corpus) {
    $path = Join-Path $outRoot "$($lib.Name)\report.json"
    if (-not (Test-Path $path)) { continue }
    $r = Get-Content $path -Raw | ConvertFrom-Json
    if ($null -eq $r.not_wired) { continue }
    foreach ($p in $r.not_wired.PSObject.Properties) {
        if (-not $byNoun.ContainsKey($p.Name)) { $byNoun[$p.Name] = @{} }
        $byNoun[$p.Name][$lib.Name] = $p.Value
    }
}
if ($byNoun.Count -gt 0) {
    Write-Host "`n=== not_wired 逐 noun（E3D 建、我们没接线）==="
    $byNoun.GetEnumerator() |
        ForEach-Object {
            [pscustomobject]@{
                Noun  = $_.Key
                Total = ($_.Value.Values | Measure-Object -Sum).Sum
                By    = ($_.Value.GetEnumerator() | Sort-Object Name |
                    ForEach-Object { "$($_.Key)=$($_.Value)" }) -join ' '
            }
        } | Sort-Object Total -Descending | Format-Table -AutoSize | Out-String -Width 200
} else {
    Write-Host "`nnot_wired 全库为零。"
}

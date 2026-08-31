"""NOUN 覆盖报告：把 `dump_categories` 的产物摊成人能读的缺口清单。

判据全部来自 Rust 侧的 `classify()`，本脚本**不含任何 noun→类别的对照表**——
此前的审计脚本抄了一份 `category.rs` 进 Python，分类表一改它不报错、只是悄悄
给出过期答案，那张影子表已经删掉，别再造第二张。

    cargo run --bin dump_categories -- data/noun-family-matrix.json out/noun-categories.json
    python scripts/noun_coverage_report.py
"""

import json
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DUMP = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "out" / "noun-categories.json"

FAMILY_MEANING = {
    "IPCOMP": "管件",
    "IHCOMP": "吊架件",
    "IFCOMP": "结构/型材件",
    "IPFCOM": "板墙楼板 / 建筑装配族",
    "IECOMP": "设备图元",
    "ICABCO": "电缆桥架",
    "INCOMP": "设计负图元",
    "IG2COM": "G2 几何集",
    "IGMCOM": "GM 几何集",
    "INGCOM": "负几何集",
    "ICCOMP": "目录件",
}

# Core3D 现代 CSG builder 已挂 plug 的 noun（既有计划 §2.4 的 12 个 addPlug 注册点）。
# 标出来是为了区分「旧路欠账」和「现代路欠账」——两者的实现依据不在一个地方。
MODERN_PLUG = set(
    """BOX CYLI CONE PYRA NBOX NCYL NCON NPYR NOZZ SUBCOM HVBRCO HVFLAN HVHACC
    HVSADD HVSPLR HVSTIF HVTPPO HVSKIR HVIDAM CTWALL BPANEL WLJOIN WLPROF WLPANE
    FLRLAY CLNTIL FPFITT ELFITT HVACFI INFITT STRFLT RLADDR SLADDR HRPOST HATTA
    TATTA RNODE CNODE CTSTRA CTCOUP CTBEND CTRISE CTTEE CTCROS CTREDU CWBRAN
    POINTR CABLE REFGLN REFGAR AIDLIN AIDARC AIDCIR AIDPOI LINDIM MLABEL
    STWELD MYBOX MYOBJ MYSHIP MYNOZ""".split()
)

if not DUMP.exists():
    raise SystemExit(f"找不到 {DUMP}；先跑 cargo run --bin dump_categories")

dump = json.loads(DUMP.read_text(encoding="utf-8"))
t = dump["totals"]
cascade = set(dump["cascade_families"])

print(f"字典 {dump['dictionary_version']}")
print(f"级联判定「该出几何」的 noun（覆盖率分母）  {t['in_cascade']}")
print(f"  ├─ 会建几何                              {t['cascade_builds']:4d}"
      f"   {t['cascade_builds'] / t['in_cascade']:.1%}")
print(f"  ├─ 记账不建（有名目）                    {t['cascade_ledgered']:4d}"
      f"   {t['cascade_ledgered'] / t['in_cascade']:.1%}")
print(f"  └─ 分类表欠账（Unknown）                 {t['cascade_unknown']:4d}"
      f"   {t['cascade_unknown'] / t['in_cascade']:.1%}  ★")

print("\n=== 级联内按类别分账 ===")
for cat, nouns in sorted(dump["cascade_by_category"].items(), key=lambda kv: -len(kv[1])):
    print(f"  {cat:20s} {len(nouns):4d}")

verdicts = {v["noun"]: v for v in dump["verdicts"]}
gaps = [v for v in dump["verdicts"] if v["in_cascade"] and v["category"] == "Unknown"]

print(f"\n=== 欠账 {len(gaps)} 个，按家族列全（* = Core3D 现代 plug 已挂）===")
by_family = defaultdict(list)
for v in gaps:
    key = ",".join(sorted(f for f in v["families"] if f in cascade))
    by_family[key].append(v["noun"])
for key in sorted(by_family, key=lambda k: -len(by_family[k])):
    meaning = " + ".join(FAMILY_MEANING.get(f, f) for f in key.split(","))
    names = sorted(by_family[key])
    print(f"\n-- {key}  {meaning}（{len(names)} 个）")
    print("   " + " ".join(f"{n}*" if n in MODERN_PLUG else n for n in names))

print("\n=== 级联内判为 NonGraphic / ProfileData / List 的 noun（要逐条给依据）===")
for cat in ("NonGraphic", "ProfileData", "List"):
    names = dump["cascade_by_category"].get(cat, [])
    if not names:
        continue
    print(f"-- {cat}（{len(names)}）")
    for n in sorted(names):
        fams = ",".join(f for f in verdicts[n]["families"] if f in cascade)
        print(f"   {n:10s} [{fams}]")

print("\n=== 正负配对：两侧类别不一致的（同一形状有两套实现的信号）===")
bad = [
    v
    for v in dump["verdicts"]
    if v["positive_equivalent"]
    and v["builds_geometry"] != verdicts[v["positive_equivalent"]]["builds_geometry"]
]
if bad:
    for v in bad:
        pos = verdicts[v["positive_equivalent"]]
        print(f"  {v['noun']:8s} {v['category']:18s} 建={v['builds_geometry']}"
              f"   ↔  正体 {pos['noun']:8s} {pos['category']:18s} 建={pos['builds_geometry']}")
else:
    print("  无——每对正负体的建/不建都一致")

"""重新生成 `data/noun-family-matrix.json`：全库 noun 的 core.dll 几何家族位。

**换 E3D 版本就要重跑这个脚本**，否则 `dump_categories` 的覆盖率分母会停在旧字典上。

## 这份数据是怎么来的（三步，缺一不可）

1. `GTGEOM`(0x10341d2e) 派发几何时走的是一串 `I*COM` 谓词；每个谓词的形状统一是
   `ATNINT(noun, &field_id, &value, &err)`——即「读 noun 字典的某个整数字段」。
2. `scripts/icom_field_ids.py` 在活桥上对 core.dll 跑一遍，把 22 个谓词各自读的
   字段号解出来（下面 `ICOM_FIELDS` 就是它的产物，改了要同步）：

       ida-bridge exec <core.dll 实例> -f scripts/icom_field_ids.py

3. 本脚本拿这些字段号去读 dabacon 字典 `attlib.dat`——**core.dll 自己读的就是这份**，
   同一个文件、同一套字段号，所以结论不是「推测 core.dll 会怎么做」而是它做的那件事。

## 为什么分 cascade / non-cascade 两组

只有 11 个家族真正出现在 `GTGEOM` 的谓词级联里，命中即旧路会给这个 noun 造几何段——
这 11 个的并集就是覆盖率的**分母**。其余家族（尤其 `IHLCOM` 的 85 个船体 noun）
走别的模块，落在 `unknown` 里不算这条几何路的欠账。分母算错了，覆盖率就没有意义。

## 用法

    python scripts/noun_family_probe.py [attlib.dat 路径] [noun_flags.json 路径]

`noun_flags.json` 只用来把 noun_hash 翻译成短名（它同时也是 NOUN→db1 数字码表）。
"""

import argparse
import json
import sys
from collections import Counter
from pathlib import Path

# `AttrDataFile` 实现了 ATTOPE/ATGTIX/ATNLOG 页链读取 + basetype 继承回溯，
# 与 aios-parse-pdms `dict.rs` 同一套；不在本 crate 里重造一份。
GEN_MODEL = Path(__file__).resolve().parents[3] / "gen-model"
sys.path.insert(0, str(GEN_MODEL))
from gm_noun_caps_probe import AttrDataFile  # noqa: E402

DEFAULT_ATTLIB = r"D:\AVEVA\Everything3D2.10\attlib.dat"
OUT = Path(__file__).resolve().parent.parent / "data" / "noun-family-matrix.json"

# 谓词 → 它读的 noun 字典字段号。由 scripts/icom_field_ids.py 从 core.dll 实测得出。
ICOM_FIELDS = {
    "IPCOMP": 602413,
    "IHCOMP": 595979,
    "IFCOMP": 600459,
    "IPFCOM": 603790,
    "IECOMP": 606263,
    "ICABCO": 591978,
    "INCOMP": 599651,
    "IG2COM": 605428,
    "IGMCOM": 604699,
    "INGCOM": 600170,
    "ICCOMP": 605100,
    "ICOCOM": 600375,
    "IFICOM": 605319,
    "IHLCOM": 599813,
    "IHVCOM": 591821,
    "IJOCOM": 597466,
    "IPLCOM": 601036,
    "IPRCOM": 595366,
    "IPTCOM": 604897,
}

# GTGEOM 的谓词级联里真正出现的家族 = 覆盖率的分母。
CASCADE = [
    "IPCOMP", "IHCOMP", "IFCOMP", "IPFCOM", "IECOMP", "ICABCO",
    "INCOMP", "IG2COM", "IGMCOM", "INGCOM", "ICCOMP",
]
NON_CASCADE = [f for f in ICOM_FIELDS if f not in CASCADE]

# 字典字段 778791：负体 → 正体等价，是正负配对的权威来源（不必在 Rust 里手写）。
FIELD_POSITIVE_EQUIVALENT = 778791


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("attlib", nargs="?", default=DEFAULT_ATTLIB)
    parser.add_argument("noun_flags", nargs="?", type=Path, default=GEN_MODEL / "noun_flags.json")
    parser.add_argument("--version", default="Everything3D 2.10")
    parser.add_argument("--out", type=Path, default=OUT)
    args = parser.parse_args()
    attlib = args.attlib
    flags_path = args.noun_flags
    dictionary_version = args.version
    out = args.out

    df = AttrDataFile(attlib)
    flags = json.loads(flags_path.read_text(encoding="utf-8"))
    names = {e["noun_hash"]: e["noun_name"] for e in flags}
    caps = {e["noun_hash"]: e for e in flags}

    absent = sorted(f for f in ICOM_FIELDS.values() if f not in df.field_index)
    if absent:
        print(f"⚠ 这些字段号在本版字典里不存在，对应家族本轮判不了：{absent}")

    rows = []
    for noun_hash in sorted(df.noun_index):
        cap = caps.get(noun_hash, {})
        pe = df.raw_field(noun_hash, FIELD_POSITIVE_EQUIVALENT)
        rows.append(
            {
                "noun_hash": noun_hash,
                "noun_name": names.get(noun_hash, ""),
                "families": [
                    pred
                    for pred, fid in ICOM_FIELDS.items()
                    if df.raw_field(noun_hash, fid) == 1
                ],
                "primitive": bool(cap.get("primitive")),
                "geomset": bool(cap.get("geomset")),
                "extrusion": bool(cap.get("extrusion")),
                "gb": cap.get("graphics_behaviour", 0),
                "positive_equivalent": (
                    names.get(pe, "") if pe not in (None, 0, -1) else ""
                ),
            }
        )

    doc = {
        "source": f"dabacon 字典 {attlib}（core.dll 读的就是这份）",
        "dictionaryVersion": dictionary_version,
        "derivation": (
            "字段号由 scripts/icom_field_ids.py 从 core.dll 的 22 个 I*COM 谓词逐个"
            "反编译解出；再由 scripts/noun_family_probe.py 按 ATTOPE/ATGTIX/ATNLOG "
            "页链 + basetype 继承读出每个 noun 的家族位。"
        ),
        "cascadeFamilies": CASCADE,
        "nonCascadeFamilies": NON_CASCADE,
        "note": (
            "cascadeFamilies = GTGEOM(0x10341d2e) 的谓词级联里真正出现的家族，"
            "命中即旧路会给它造几何段。IHLCOM（船体 85 个 noun）不在级联内，"
            "不属本条几何路。"
        ),
        "nouns": rows,
    }
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(
        json.dumps(doc, ensure_ascii=False, separators=(",", ":")), encoding="utf-8"
    )

    cascade = set(CASCADE)
    hits = Counter(p for r in rows for p in r["families"])
    print(f"字典 {dictionary_version}：noun {len(rows)} 个，字段 {len(df.field_index)} 个")
    for pred in ICOM_FIELDS:
        where = "级联内" if pred in cascade else "级联外"
        print(f"  {pred:8s} field={ICOM_FIELDS[pred]:<9} 命中 {hits.get(pred, 0):5d}  {where}")
    print(f"命中级联的 noun（覆盖率分母）：{sum(1 for r in rows if cascade & set(r['families']))}")
    print(f"写出 {out}（{out.stat().st_size} bytes）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

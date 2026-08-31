"""`A*` 设备图元族到底是不是设计图元的同形孪生——属性表实读对拍。

## 这个脚本回答什么

`category.rs` 里 `ABOX → Primitive(Prim::Box)` 这类接线，凭据不能是「名字看着像」。
本脚本把每个 `A*` noun 的**属性集合**与对应设计图元逐个求差：如果 `ABOX` 的尺寸属性
恰好就是 `BOX` 的 `XLEN/YLEN/ZLEN`、一个不多一个不少，那它们的几何由同一组标量完全
决定，接同一条几何路径才站得住。

## 数据来自哪

`gen-model/all_attr_info.json` 的 `noun_attr_info_map`：noun_hash → {属性哈希 → 属性名/
类型/默认值}，是 `aios_core` 运行时加载的那份元素属性 schema（由 `all_attr_info.bin`
转出），生产代码判属性类型用的就是它。noun_hash ↔ 短名的翻译用 `noun_flags.json`。

## 方法自带对照组

`AEXTR` 早就按同形接在 `LoopExtrusion` 上了。它与 `EXTR` 的关系必须被本脚本判成
「同形」——判据连一个已知正确的答案都复现不出来，就没资格用来接新的。

## 用法

    python scripts/a_family_attr_probe.py [all_attr_info.json] [noun_flags.json]

退出码 0 = 全部 12 对都判为同形；非 0 = 有对子对不上，接线前先看清楚差在哪。
"""

import json
import sys
from pathlib import Path

GEN_MODEL = Path(__file__).resolve().parents[3] / "gen-model"

# A* noun → 它声称同形的设计图元。12 个 IECOMP 成员，AEXTR 是对照组。
PAIRS = [
    ("ABOX", "BOX"),
    ("ACYLI", "CYLI"),
    ("APYRA", "PYRA"),
    ("ARTOR", "RTOR"),
    ("ACTOR", "CTOR"),
    ("ACONE", "CONE"),
    ("ADISH", "DISH"),
    ("ASNOU", "SNOU"),
    ("ASLCY", "SLCY"),
    ("APOLYH", "POLYHE"),
    ("AREVO", "REVO"),
    ("AEXTR", "EXTR"),
]

# 决定几何的标量属性。差集只看这一组：设计侧独有的 OWNER/LOCK/PURP 之类是记账属性，
# 不影响形状，拿它们算差异只会把结论淹掉。
DIMENSIONS = {
    "XLEN", "YLEN", "ZLEN",
    "DIAM", "HEIG", "RADI",
    "XBOT", "YBOT", "XTOP", "YTOP", "XOFF", "YOFF",
    "DBOT", "DTOP",
    "RINS", "ROUT", "ANGL",
}


def main() -> int:
    schema_path = Path(sys.argv[1]) if len(sys.argv) > 1 else GEN_MODEL / "all_attr_info.json"
    flags_path = Path(sys.argv[2]) if len(sys.argv) > 2 else GEN_MODEL / "noun_flags.json"

    schema = json.loads(schema_path.read_text(encoding="utf-8"))["noun_attr_info_map"]
    hashes = {
        entry["noun_name"]: str(entry["noun_hash"])
        for entry in json.loads(flags_path.read_text(encoding="utf-8"))
    }

    def attributes(noun):
        key = hashes.get(noun)
        if key is None or key not in schema:
            return None
        return {info["name"] for info in schema[key].values()}

    print(f"属性 schema：{schema_path}（{len(schema)} 个 noun）")
    print()
    print(f"{'A* noun':<8}{'设计图元':<10}{'尺寸属性':<52}{'判定'}")

    mismatched = []
    for a_noun, design in PAIRS:
        a_attrs, design_attrs = attributes(a_noun), attributes(design)
        if a_attrs is None or design_attrs is None:
            mismatched.append((a_noun, design, "schema 里查不到"))
            print(f"{a_noun:<8}{design:<10}{'—':<52}查不到")
            continue

        a_dims = sorted(DIMENSIONS & a_attrs)
        design_dims = sorted(DIMENSIONS & design_attrs)
        same = a_dims == design_dims
        # 设计图元独有的那几个必须全是记账属性；A* 独有任何一个都说明它另有形状参数。
        a_only = sorted(a_attrs - design_attrs)
        verdict = "同形" if same and not a_only else "对不上"
        if verdict != "同形":
            mismatched.append((a_noun, design, f"A* 独有={a_only} 尺寸差={set(a_dims) ^ set(design_dims)}"))
        print(f"{a_noun:<8}{design:<10}{','.join(a_dims) or '(无标量尺寸)':<52}{verdict}")

    print()
    for a_noun, design in PAIRS:
        a_attrs, design_attrs = attributes(a_noun), attributes(design)
        if a_attrs is None or design_attrs is None:
            continue
        print(f"{a_noun:<8}缺 {design} 的：{sorted(design_attrs - a_attrs)}")

    print()
    if mismatched:
        print("对不上的：")
        for a_noun, design, why in mismatched:
            print(f"  {a_noun} vs {design}：{why}")
        return 1

    print("12 对全部同形：A* 的尺寸属性与设计图元逐项相同，独有属性为空。")
    print("对照组 AEXTR/EXTR 判为同形——判据能复现一个已知正确的答案。")
    print()
    print("注意：属性表说不了成员结构。APOLYH/AREVO 的几何来自成员（B-rep 顶点面表 /")
    print("PLOO 剖面环），标量同形不足以接线，它们在 category.rs 里仍是 Unsupported。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

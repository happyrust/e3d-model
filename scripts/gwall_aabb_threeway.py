# -*- coding: utf-8 -*-
"""三方 AABB 对账：RVM facet 顶点实算 vs .rvm.json 快照字段 vs 我方最终网格。

裁决「AABB 大偏差」到底是几何差还是基准字段差。
"""

from __future__ import annotations

import json
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
GEN_MODEL = Path(r"d:\work\plant-code\old\gen-model")


def aabb_of(polys):
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    for poly in polys:
        for contour in poly:
            for v in contour:
                for i in range(3):
                    lo[i] = min(lo[i], v[i])
                    hi[i] = max(hi[i], v[i])
    return lo, hi


def fmt(lo, hi):
    return (
        f"[{lo[0]:9.1f},{lo[1]:9.1f},{lo[2]:8.1f}]-[{hi[0]:9.1f},{hi[1]:9.1f},{hi[2]:8.1f}]"
    )


def max_axis_delta(a, b):
    (alo, ahi), (blo, bhi) = a, b
    return max(max(abs(x - y) for x, y in zip(alo, blo)), max(abs(x - y) for x, y in zip(ahi, bhi)))


def main():
    facets = json.loads((HERE / "out/ams1112/rvm-facets-RR001.json").read_text(encoding="utf-8"))
    snap = json.loads(
        (GEN_MODEL / "test_data/rvm/1RS-WF03-W-C-RR001.rvm.json").read_text(encoding="utf-8")
    )
    els = json.loads((HERE / "out/ams1112/elements.json").read_text(encoding="utf-8"))

    facet_by_ord = {}
    for group in facets:
        m = re.match(r"^GWALL (\d+) of ", group["name"])
        if not m:
            continue
        polys = []
        declared = None
        for geom in group["geometries"]:
            if geom["kind"] == "FacetGroup":
                polys.extend(geom["polygons_world_mm"])
                declared = geom["bbox_world_mm"]
        facet_by_ord[int(m.group(1))] = (aabb_of(polys), declared)

    snap_by_ord = {}
    for member in snap["members"]:
        name = (member.get("name") or "").strip()
        m = re.match(r"^GWALL (\d+) of CWALL /1RS-WF03-W-C-RR001$", name)
        if not m:
            continue
        bb = member.get("aabb_world_mm")
        snap_by_ord[int(m.group(1))] = (bb[:3], bb[3:]) if bb else None

    ours_by_ord = {}
    for el in els:
        if el["noun"] == "GWALL" and el.get("owner_desc") == "CWALL /1RS-WF03-W-C-RR001":
            ours_by_ord[el["ordinal"]] = (el["aabb_min"], el["aabb_max"], el["volume_mm3"])

    print(
        f"{'ord':>3} | {'facet实算 vs 快照字段':>22} | {'facet实算 vs 我方最终':>22} | "
        f"{'快照字段 vs 我方最终':>20}"
    )
    for ord_n in sorted(facet_by_ord):
        (f_aabb, declared) = facet_by_ord[ord_n]
        s_aabb = snap_by_ord.get(ord_n)
        o = ours_by_ord.get(ord_n)
        d_fs = max_axis_delta(f_aabb, s_aabb) if s_aabb else float("nan")
        d_fo = max_axis_delta(f_aabb, (o[0], o[1])) if o else float("nan")
        d_so = max_axis_delta(s_aabb, (o[0], o[1])) if (s_aabb and o) else float("nan")
        print(f"{ord_n:>3} | {d_fs:>22.2f} | {d_fo:>22.2f} | {d_so:>20.2f}")

    print()
    print("样例 ord1 三份 AABB:")
    print("  facet实算 :", fmt(*facet_by_ord[1][0]))
    print("  dump声明  :", facet_by_ord[1][1])
    print("  快照字段  :", fmt(*snap_by_ord[1]) if snap_by_ord.get(1) else None)
    o = ours_by_ord.get(1)
    print("  我方最终  :", fmt(o[0], o[1]) if o else None)


if __name__ == "__main__":
    main()

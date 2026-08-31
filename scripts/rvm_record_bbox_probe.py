# -*- coding: utf-8 -*-
"""PRIM 记录 bbox 定性：它是世界盒还是局部盒？

对每件 GWALL FacetGroup:
  记录 bbox vs 顶点世界 AABB / 顶点局部 AABB 的最大轴差。
哪边接近零，记录 bbox 就是哪个坐标系的。
"""

import json
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
facets = json.loads((HERE / "out/ams1112/rvm-facets-RR001.json").read_text(encoding="utf-8"))

print(f"{'ord':>3} {'vs顶点世界AABB(mm)':>20} {'vs顶点局部AABB(mm)':>20}")
for g in facets:
    m = re.match(r"^GWALL (\d+) of ", g["name"])
    if not m:
        continue
    for geom in g["geometries"]:
        if geom["kind"] != "FacetGroup":
            continue
        rec = geom["bbox_record_m"]
        vs = [v for poly in geom["polygons_world_mm"] for cont in poly for v in cont]
        wlo = [min(v[i] for v in vs) / 1000 for i in range(3)]
        whi = [max(v[i] for v in vs) / 1000 for i in range(3)]
        c = geom["transform_cols_m"]
        cols = [(c[0], c[1], c[2]), (c[3], c[4], c[5]), (c[6], c[7], c[8])]
        t = (c[9], c[10], c[11])
        loc = []
        for v in vs:
            d = (v[0] / 1000 - t[0], v[1] / 1000 - t[1], v[2] / 1000 - t[2])
            loc.append(tuple(sum(col[i] * d[i] for i in range(3)) for col in cols))
        llo = [min(p[i] for p in loc) for i in range(3)]
        lhi = [max(p[i] for p in loc) for i in range(3)]
        d_w = max(
            max(abs(rec[i] - wlo[i]) for i in range(3)),
            max(abs(rec[3 + i] - whi[i]) for i in range(3)),
        ) * 1000
        d_l = max(
            max(abs(rec[i] - llo[i]) for i in range(3)),
            max(abs(rec[3 + i] - lhi[i]) for i in range(3)),
        ) * 1000
        print(f"{int(m.group(1)):>3} {d_w:>20.2f} {d_l:>20.2f}")

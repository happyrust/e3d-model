r"""逐件把 E3D 导出的 GWALL facet 轮廓对回我方 PLOO 环，坐实 XY 轮廓差异的形态。

输入（默认 out/ams1112/）：
  loops-RR001-gwall.json   我方环（局部 2D 剖面 [x,y,frad] + world 矩阵 + height）
  rvm-facets-RR001.json    E3D facet（polygons_world_mm，世界 mm）

做法：E3D 世界顶点用我方 world 矩阵的逆（R 正交 → R^T）转到墙局部系，
与我方环的局部 XY 范围逐件对比，报端头(x)延伸、边(y)外扩、Z 范围与面片计数。

    python tools/analyze_gwall_profiles.py [loops.json] [facets.json]
"""

import json
import re
import sys

LOOPS = sys.argv[1] if len(sys.argv) > 1 else r"D:\work\plant-code\old\vendor\e3d-model\out\ams1112\loops-RR001-gwall.json"
FACETS = sys.argv[2] if len(sys.argv) > 2 else r"D:\work\plant-code\old\vendor\e3d-model\out\ams1112\rvm-facets-RR001.json"

ours_list = json.load(open(LOOPS, encoding="utf-8"))
facets = json.load(open(FACETS, encoding="utf-8"))

fac_by_ord = {}
for g in facets:
    m = re.match(r"GWALL\s+(\d+)\s+of", g["name"])
    if m:
        fac_by_ord[int(m.group(1))] = g


def rng(vs):
    return (min(vs), max(vs))


print(f"我方 {len(ours_list)} 件 GWALL；E3D facet {len(fac_by_ord)} 件\n")
print(f"{'ord':>3} {'refno':>13} {'faces':>12} {'h':>6}{'sjus':>6}{'hole':>5}{'verd':>6}{'worst':>8}  端头Δx            边Δy             Δz")
print("-" * 120)

summary = {"exact": 0, "near": 0, "off": 0, "z_off": 0}

for ours in sorted(ours_list, key=lambda o: o["ordinal"]):
    ordn = ours["ordinal"]
    g = fac_by_ord.get(ordn)
    ring = ours["rings"][0] if ours["rings"] else []
    if not ring or not g:
        print(f"{ordn:>3} {ours['refno']:>13}  (缺环或缺 facet)")
        continue
    oxs = [v[0] for v in ring]
    oys = [v[1] for v in ring]
    oxr, oyr = rng(oxs), rng(oys)
    h = ours["height"]

    c0, c1, c2, t = ours["world"]

    def to_local(w):
        d = (w[0] - t[0], w[1] - t[1], w[2] - t[2])
        return (
            c0[0] * d[0] + c0[1] * d[1] + c0[2] * d[2],
            c1[0] * d[0] + c1[1] * d[1] + c1[2] * d[2],
            c2[0] * d[0] + c2[1] * d[1] + c2[2] * d[2],
        )

    lxs, lys, lzs = [], [], []
    npoly = ncont = nvert = 0
    for geo in g["geometries"]:
        polys = geo.get("polygons_world_mm")
        if not polys:
            continue
        for poly in polys:
            npoly += 1
            for contour in poly:
                ncont += 1
                for v in contour:
                    nvert += 1
                    lx, ly, lz = to_local(v)
                    lxs.append(lx)
                    lys.append(ly)
                    lzs.append(lz)
    if not lxs:
        print(f"{ordn:>3} {ours['refno']:>13}  (facet 无顶点)")
        continue
    exr, eyr, ezr = rng(lxs), rng(lys), rng(lzs)

    dx0 = exr[0] - oxr[0]
    dx1 = exr[1] - oxr[1]
    dy0 = eyr[0] - oyr[0]
    dy1 = eyr[1] - oyr[1]
    # 我方生成物含 elmodl 的 SJUS 下移（elmodl.rs:172 translate(0,0,-sjus_drop)）；
    # loop_dump 的 world 不含它，故这里手动把我方局部 z 基准放到 [-sjus_drop, h-sjus_drop]。
    sj = ours["sjus_drop"]
    dz0 = ezr[0] - (-sj)
    dz1 = ezr[1] - (h - sj)

    end_ext = max(abs(dx0), abs(dx1))
    y_wid = max(abs(dy0), abs(dy1))
    z_off = max(abs(dz0), abs(dz1))
    worst = max(end_ext, y_wid, z_off)
    verdict = "exact" if worst < 0.05 else ("off" if worst >= 1.0 else "near")
    summary[verdict] = summary.get(verdict, 0) + 1
    if z_off >= 1:
        summary["z_off"] += 1

    print(
        f"{ordn:>3} {ours['refno']:>13} {npoly:>4}p/{ncont:>3}c/{nvert:>4}v {h:>6.0f}{sj:>6.0f}{ncont - npoly:>5} {verdict:>5}{worst:>8.1f}  "
        f"[{dx0:+7.1f},{dx1:+7.1f}] [{dy0:+6.1f},{dy1:+6.1f}] [{dz0:+6.1f},{dz1:+6.1f}]"
    )

print("-" * 120)
print(f"判据(局部系, 含 SJUS 下移修正)：exact(<0.05mm) {summary['exact']} 件；near(<1mm) {summary['near']} 件；"
      f"off(≥1mm) {summary['off']} 件；其中 Z 仍偏(|Δz|≥1) {summary['z_off']} 件")
print("说明：dz 基准已含 elmodl 的 SJUS 下移([-sjus_drop, h-sjus_drop])；Δx=端头 Δy=墙厚；contour>polygon=开洞。")

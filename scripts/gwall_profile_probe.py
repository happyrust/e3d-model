# -*- coding: utf-8 -*-
"""GWALL 轮廓取证：把 E3D RVM facet 顶点投回我方墙局部系，对比 PLOO/PAVE 环。

输入:
  out/ams1112/rvm-facets-RR001.json   rvm_facets_dump 产物（世界 mm）
  out/ams1112/loops-RR001-gwall.json  loop_dump 产物（局部环 + 世界矩阵）

对每件 GWALL:
  1. E3D 顶点 -> 我方局部系（world 矩阵求逆）
  2. 找底面 z 簇，抽出底面轮廓
  3. 与我方环对比：顶点数 / 面积 / 逐顶点残差（含镜像假设）
"""

from __future__ import annotations

import json
import math
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
FACETS = HERE / "out/ams1112/rvm-facets-RR001.json"
LOOPS = HERE / "out/ams1112/loops-RR001-gwall.json"


def load():
    facets = json.loads(FACETS.read_text(encoding="utf-8"))
    loops = json.loads(LOOPS.read_text(encoding="utf-8"))
    gwalls = {}
    for group in facets:
        m = re.match(r"^GWALL (\d+) of ", group["name"])
        if m:
            gwalls[int(m.group(1))] = group
    ours = {entry["ordinal"]: entry for entry in loops}
    return gwalls, ours


def to_local(world_cols, point):
    c0, c1, c2, t = world_cols
    dx = point[0] - t[0]
    dy = point[1] - t[1]
    dz = point[2] - t[2]
    # R 正交，逆 = 转置；列主序 => local_i = c_i . d
    return (
        c0[0] * dx + c0[1] * dy + c0[2] * dz,
        c1[0] * dx + c1[1] * dy + c1[2] * dz,
        c2[0] * dx + c2[1] * dy + c2[2] * dz,
    )


def signed_area(ring):
    area = 0.0
    n = len(ring)
    for i in range(n):
        x1, y1 = ring[i][:2]
        x2, y2 = ring[(i + 1) % n][:2]
        area += x1 * y2 - x2 * y1
    return area * 0.5


def dedupe_ring(ring, eps=1e-4):
    out = []
    for p in ring:
        if out and abs(p[0] - out[-1][0]) < eps and abs(p[1] - out[-1][1]) < eps:
            continue
        out.append(p)
    if len(out) > 1 and abs(out[0][0] - out[-1][0]) < eps and abs(out[0][1] - out[-1][1]) < eps:
        out.pop()
    return out


def point_seg_dist(p, a, b):
    ax, ay = a
    bx, by = b
    px, py = p
    vx, vy = bx - ax, by - ay
    L2 = vx * vx + vy * vy
    if L2 <= 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / L2))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def boundary_dist(p, ring):
    n = len(ring)
    return min(point_seg_dist(p, ring[i][:2], ring[(i + 1) % n][:2]) for i in range(n))


def densify(ring, step=25.0):
    pts = []
    n = len(ring)
    for i in range(n):
        ax, ay = ring[i][:2]
        bx, by = ring[(i + 1) % n][:2]
        seg = math.hypot(bx - ax, by - ay)
        k = max(1, int(seg / step))
        for j in range(k):
            t = j / k
            pts.append((ax + t * (bx - ax), ay + t * (by - ay)))
    return pts


def ring_gap(ring_a, ring_b):
    """双向边界距离（采样版）：max(p in A) dist(p, B边界) 与反向。"""
    a2b = max(boundary_dist(p, ring_b) for p in densify(ring_a))
    b2a = max(boundary_dist(p, ring_a) for p in densify(ring_b))
    return a2b, b2a


def main():
    gwalls, ours = load()
    print(f"E3D GWALL 组 {len(gwalls)} 件, 我方 {len(ours)} 件")
    print()

    detail_ords = {int(x) for x in sys.argv[1:]} if len(sys.argv) > 1 else set()

    header = (
        f"{'ord':>3} {'refno':>14} {'E3D底轮廓':>10} {'我方环':>6} "
        f"{'底z(E3D)':>10} {'-drop':>8} {'面积E3D':>12} {'面积我方':>12} "
        f"{'gap E->我':>9} {'gap 我->E':>9} {'洞':>3}"
    )
    print(header)

    for ord_n in sorted(gwalls):
        group = gwalls[ord_n]
        our = ours.get(ord_n)
        if our is None:
            print(f"{ord_n:>3} 我方缺此件")
            continue
        geoms = [g for g in group["geometries"] if g["kind"] == "FacetGroup"]
        if not geoms:
            kinds = [g["kind"] for g in group["geometries"]]
            print(f"{ord_n:>3} E3D 几何不是 FacetGroup: {kinds}")
            continue

        world = our["world"]
        drop = our["sjus_drop"]
        height = our["height"]

        # 全部顶点 -> 局部
        polys_local = []
        for geom in geoms:
            for poly in geom["polygons_world_mm"]:
                contours_local = []
                for contour in poly:
                    contours_local.append([to_local(world, v) for v in contour])
                polys_local.append(contours_local)

        zs = [v[2] for poly in polys_local for contour in poly for v in contour]
        z_bot = min(zs)
        z_top = max(zs)

        # 底面 = 所有顶点都贴 z_bot 的 polygon
        bottom_polys = [
            poly
            for poly in polys_local
            if all(abs(v[2] - z_bot) < 0.5 for contour in poly for v in contour)
        ]
        # 底面外轮廓 = 底面各 contour 里 |面积| 最大者；其余 contour 记作洞
        bottom_contours = [c for poly in bottom_polys for c in poly]
        if not bottom_contours:
            print(f"{ord_n:>3} 找不到底面（z_bot={z_bot:.2f}, z_top={z_top:.2f}）")
            continue
        bottom_contours = [dedupe_ring([(v[0], v[1]) for v in c]) for c in bottom_contours]
        bottom_contours = [c for c in bottom_contours if len(c) >= 3]
        outer = max(bottom_contours, key=lambda c: abs(signed_area(c)))
        holes = len(bottom_contours) - 1

        our_ring = dedupe_ring([(v[0], v[1]) for v in our["rings"][0]])
        a_e3d = abs(signed_area(outer))
        a_our = abs(signed_area(our_ring))
        gap_eo, gap_oe = ring_gap(outer, our_ring)

        print(
            f"{ord_n:>3} {our['refno']:>14} {len(outer):>10} {len(our_ring):>6} "
            f"{z_bot:>10.2f} {-drop:>8.1f} {a_e3d:>12.0f} {a_our:>12.0f} "
            f"{gap_eo:>9.2f} {gap_oe:>9.2f} {holes:>3}"
        )

        if ord_n in detail_ords:
            print(f"    高度: 我方 {height:.1f}, E3D z跨 {z_top - z_bot:.1f}")
            print(f"    E3D 底面外轮廓（局部 XY）:")
            for v in outer:
                print(f"        ({v[0]:>10.2f}, {v[1]:>10.2f})")
            print(f"    我方环:")
            for v in our_ring:
                print(f"        ({v[0]:>10.2f}, {v[1]:>10.2f})")
            if holes:
                for i, c in enumerate(bottom_contours):
                    if c is outer:
                        continue
                    print(f"    洞{i}: {len(c)} 顶点, 面积 {abs(signed_area(c)):.0f}")


if __name__ == "__main__":
    main()

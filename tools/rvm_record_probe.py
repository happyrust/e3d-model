r"""直接读 `.rvm` 字节，把 AVEVA RVM 记录的骨架、单位与 **bbox 所在的帧**摊开。

比 `scripts/rvm_record_bbox_probe.py` 多两件事，也因此得出不同的结论：

1. **读原始字节**，不经 `rvm_facets_dump` / rvm-rs——那条链自己就带着帧问题
   （`parser/rvm.rs` 对每条记录一律 `bbox.transform(&transform)`），拿它当尺子量不出尺子的偏差。
2. **候选帧有三个不是两个**。老探针只比 `world` 与 `local`，两个都对不上，于是结论成了
   「PRIM 记录自带的 bbox 不可信」。真正命中的是第三个：

       rotated = AABB(matrix3 · v × 1000)      只乘旋转、不加平移，单位 mm

   `test_data/rvm/` 六份 E3D 3.1 导出物、432 条记录实测：

   | | bbox 帧 | 计数 |
   |---|---|---|
   | 解析图元 Box / Cylinder / Snout / CircularTorus | `local` | 196 / 196 |
   | FacetGroup，`WALL` / `STWALL` 族 | `local` | 8 |
   | FacetGroup，其余 | `rotated` | 228 |

   **帧由 E3D 写出端的代码路径决定，不由 kind 决定**——所以判帧只能逐记录自证，
   按 kind 查表迟早翻车。

顺带钉死的单位（同一条记录里有三套，写错就是整体差 1000 倍）：

    CNTB 平移        mm，绝对世界坐标（祖先链全 0，逐级累加会乘上嵌套深度）
    PRIM 矩阵 3×3    R × 0.001
    PRIM 矩阵平移    m
    顶点 / bbox / 图元参数   mm

    世界 mm = (matrix3 · v + translation) × 1000

用法：

    python tools/rvm_record_probe.py skeleton <a.rvm> [最多打印几条]
    python tools/rvm_record_probe.py facet    <a.rvm>            # 逐字节拆第一条 FacetGroup
    python tools/rvm_record_probe.py frames   <a.rvm> [<b.rvm> ...]   # 三帧判定 + 汇总
    python tools/rvm_record_probe.py fields   <a.rvm> [<b.rvm> ...]   # 常量字段普查

复跑（六份样本一次过，判帧 + 常量两张表）：

    python tools/rvm_record_probe.py frames ../../gen-model/test_data/rvm/*.rvm
    python tools/rvm_record_probe.py fields ../../gen-model/test_data/rvm/*.rvm
"""

import collections
import glob
import math
import struct
import sys

M_TO_MM = 1000.0

KINDS = {
    1: "Pyramid", 2: "Box", 3: "RectangularTorus", 4: "CircularTorus",
    5: "EllipticalDish", 6: "SphericalDish", 7: "Snout", 8: "Cylinder",
    9: "Sphere", 10: "Line", 11: "FacetGroup",
}
# kind -> 参数个数（PRIM 负载里 bbox 之后的 f32 数）
NPARAM = {1: 7, 2: 3, 3: 4, 4: 3, 5: 2, 6: 2, 7: 9, 8: 2, 9: 1, 10: 2}


def u32(b, o):
    return struct.unpack_from(">I", b, o)[0]


def f32(b, o):
    return struct.unpack_from(">f", b, o)[0]


def tag_at(b, o):
    """chunk tag = 4 个字符，各占一个 big-endian u32。"""
    return "".join(chr(u32(b, o + i * 4)) for i in range(4)).strip("\x00")


def read_string(b, o):
    """u32 字数（4 字节一字）+ NUL 补齐的字节。返回 (字串, 下一个偏移)。"""
    words = u32(b, o)
    return b[o + 4:o + 4 + words * 4].split(b"\x00")[0].decode("utf-8", "replace"), o + 4 + words * 4


def walk(path):
    """遍历 chunk，产出 (offset, tag, payload, 当前 CNTB 名字栈)。"""
    data = open(path, "rb").read()
    off = 0
    stack = []
    while off + 20 <= len(data) and not all(v == 0 for v in data[off:off + 16]):
        tag = tag_at(data, off)
        nxt = u32(data, off + 16)
        if nxt <= off or nxt > len(data):
            break
        payload = data[off + 20:nxt]
        if tag == "CNTB":
            stack.append(read_string(payload, 8)[0])
        yield off, tag, payload, list(stack)
        if tag == "CNTE" and stack:
            stack.pop()
        off = nxt
    yield off, "__EOF__", b"", []


def facet_vertices(payload):
    """FacetGroup 的局部顶点（mm）。非 FacetGroup 返回 []。"""
    if u32(payload, 8) != 11:
        return []
    p = 84
    out = []
    for _ in range(u32(payload, p)):
        p += 4 if False else 0  # 保持 p 语义清晰：num_polygons 已在下面统一推进
        break
    p = 84
    npoly = u32(payload, p)
    p += 4
    for _ in range(npoly):
        ncont = u32(payload, p)
        p += 4
        for _ in range(ncont):
            nvert = u32(payload, p)
            p += 4
            for _ in range(nvert):
                out.append([f32(payload, p), f32(payload, p + 4), f32(payload, p + 8)])
                p += 24  # 顶点 3×f32 + 法线 3×f32
    return out


def canonical_local_box(kind, p):
    """按 rvmparser 口径还原图元的规范局部盒（Z-up）。认不出的 kind 返回 None。"""
    if kind == 2:
        return [-p[0] / 2, -p[1] / 2, -p[2] / 2], [p[0] / 2, p[1] / 2, p[2] / 2]
    if kind == 8:
        return [-p[0], -p[0], -p[1] / 2], [p[0], p[0], p[1] / 2]
    if kind == 9:
        return [-p[0]] * 3, [p[0]] * 3
    if kind == 7:      # Snout: rb, rt, height, offset_x, offset_y, 4×shear
        # 同心锥用 max(rb, rt) 居中建盒即可，实测 4/4 残差 0。
        #
        # **偏心锥（ox/oy ≠ 0）的记录 bbox 不是紧包围盒**，别去凑。ams7999 `/1RCV003VP` 的
        # `Snout[rb 33.165, rt 42.21, h 115.2, ox 12.06, oy 0]` 记录写的是
        #     x ∈ [−48.240, +48.240]      48.24 = rt + ox/2，两侧都放到了这么宽
        #     y ∈ [−42.210, +48.240]      oy = 0 却上下不对称
        #     z ∈ [ −57.600, +57.600]     = ±h/2，这一轴是准的
        # 真正的紧盒（偏心两端各摊一半）是 x ∈ [−39.195, 48.240]、y ∈ ±39.195。
        # 也就是说 E3D 这一支给的是个**放宽过的估计**，而且放宽得并不对称——
        # 试过「偏心全给顶 / 全给底 / 两端各半」三种建模，残差只会从 6.03 变成 9.05，凑不上。
        # 保持简单模型：残差 ≤ |offset|/2，被 classify_frame 的容差吸收，判帧不受影响。
        r = max(p[0], p[1])
        return [-r, -r, -p[2] / 2], [r, r, p[2] / 2]
    if kind in (5, 6):
        return [-p[0], -p[0], 0.0], [p[0], p[0], p[1]]
    if kind == 4:      # CircularTorus: offset, radius, angle —— 绕 Z 扫，管截面在径向/Z 平面
        offset, radius, angle = p[0], p[1], p[2]
        pts = []
        for i in range(181):
            th = angle * i / 180.0
            for j in range(181):
                ph = 2.0 * math.pi * j / 180.0
                rr = offset + radius * math.cos(ph)
                pts.append([rr * math.cos(th), rr * math.sin(th), radius * math.sin(ph)])
        return _minmax(pts)
    if kind == 3:      # RectangularTorus: inner, outer, height, angle
        inner, outer, height, angle = p[0], p[1], p[2], p[3]
        pts = []
        for i in range(361):
            th = angle * i / 360.0
            for rr in (inner, outer):
                for z in (-height / 2, height / 2):
                    pts.append([rr * math.cos(th), rr * math.sin(th), z])
        return _minmax(pts)
    return None


def _minmax(pts):
    return ([min(q[a] for q in pts) for a in range(3)],
            [max(q[a] for q in pts) for a in range(3)])


def corners(lo, hi):
    return [[lo[0] if i & 1 else hi[0], lo[1] if i & 2 else hi[1], lo[2] if i & 4 else hi[2]]
            for i in range(8)]


def aabb6(pts):
    return [min(q[a] for q in pts) for a in range(3)] + [max(q[a] for q in pts) for a in range(3)]


def maxdiff(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


def classify_frame(payload):
    """判这条 PRIM 记录的 bbox 在哪个帧。

    返回 (帧名, 残差 mm, 各候选的残差) —— 帧名为 None 表示三个候选都对不上。
    """
    m = [f32(payload, 12 + i * 4) for i in range(12)]
    record = [f32(payload, 60 + i * 4) for i in range(6)]
    kind = u32(payload, 8)

    verts = facet_vertices(payload)
    if not verts:
        params = [f32(payload, 84 + i * 4) for i in range(NPARAM.get(kind, 0))]
        box = canonical_local_box(kind, params)
        if box is None:
            return None, None, {}
        verts = corners(*box)

    def rot(v):
        return [(m[0] * v[0] + m[3] * v[1] + m[6] * v[2]) * M_TO_MM,
                (m[1] * v[0] + m[4] * v[1] + m[7] * v[2]) * M_TO_MM,
                (m[2] * v[0] + m[5] * v[1] + m[8] * v[2]) * M_TO_MM]

    cands = {
        "local": aabb6(verts),
        "rotated": aabb6([rot(v) for v in verts]),
        "world": aabb6([[rot(v)[a] + m[9 + a] * M_TO_MM for a in range(3)] for v in verts]),
    }
    diffs = {k: maxdiff(v, record) for k, v in cands.items()}
    best = min(diffs, key=diffs.get)
    # 1mm + 该轴 8% 跨度：圆周按 1mm 弦高最少 8 段，极值角上没有顶点时最多缩进 7.6%。
    span = max(abs(record[a + 3] - record[a]) for a in range(3))
    if diffs[best] > 1.0 + 0.08 * span:
        return None, diffs[best], diffs
    return best, diffs[best], diffs


def cmd_skeleton(paths, limit=40):
    for path in paths:
        print(f"=== {path} ===")
        shown = 0
        counts = collections.Counter()
        for off, tag, d, stack in walk(path):
            if tag == "__EOF__":
                print(f"{off:#08x}  <16 zero bytes> -> stop")
                break
            counts[tag] += 1
            line = ""
            if tag == "HEAD":
                parts, p = [], 8
                while p + 4 <= len(d) and len(parts) < 5:
                    s, p = read_string(d, p)
                    parts.append(s)
                line = f"v={u32(d,0)} n={u32(d,4)} strings={parts}"
            elif tag == "MODL":
                name, p = read_string(d, 8)
                line = f"v={u32(d,0)} n={u32(d,4)} name={name!r} project={read_string(d,p)[0]!r}"
            elif tag == "CNTB":
                name, p = read_string(d, 8)
                line = (f"name={name!r} translation="
                        f"({f32(d,p):.2f},{f32(d,p+4):.2f},{f32(d,p+8):.2f}) material={u32(d,p+12)}")
            elif tag in ("PRIM", "OBST", "INSU"):
                k = u32(d, 8)
                m = [f32(d, 12 + i * 4) for i in range(12)]
                bb = [f32(d, 60 + i * 4) for i in range(6)]
                pr = [round(f32(d, 84 + i * 4), 1) for i in range(NPARAM.get(k, 0))]
                frame, resid, _ = classify_frame(d)
                line = (f"kind={k}({KINDS.get(k,'?')}) T=({m[9]:.4f},{m[10]:.4f},{m[11]:.4f})m "
                        f"bbox={[round(x,1) for x in bb]} params={pr} "
                        f"frame={frame or '??'}")
            elif tag in ("CNTE", "COLR") or tag.startswith("END"):
                line = f"payload={len(d)}B {d.hex()}"
            if shown < limit and line:
                indent = "  " * max(len(stack) - (1 if tag == "CNTB" else 0), 0)
                print(f"{off:#08x}  {indent}{tag:5s} {line}")
                shown += 1
        print(f"chunk counts: {dict(counts)}\n")


def cmd_facet(paths):
    for path in paths:
        print(f"=== {path} ===")
        for off, tag, d, stack in walk(path):
            if tag not in ("PRIM", "OBST", "INSU") or u32(d, 8) != 11:
                continue
            m = [f32(d, 12 + i * 4) for i in range(12)]
            scale = math.sqrt(m[0] ** 2 + m[1] ** 2 + m[2] ** 2)
            print(f"FacetGroup PRIM @ {off:#x}  owner={stack[-1] if stack else '?'!r}  payload={len(d)}B")
            print(f"  matrix 3x3     = {[round(v,6) for v in m[:9]]}   |col0| = {scale:.6f}  (= 0.001 → R×0.001)")
            print(f"  matrix transl  = {[round(v,6) for v in m[9:]]}  (m)")
            print(f"  record bbox    = {[round(v,4) for v in [f32(d,60+i*4) for i in range(6)]]}  (mm)")
            verts = facet_vertices(d)
            print(f"  num_polygons   = {u32(d,84)}   vertices = {len(verts)}")
            print(f"  vertex AABB    = {[round(v,4) for v in aabb6(verts)]}  (mm, 局部)")
            frame, resid, diffs = classify_frame(d)
            print(f"  bbox 帧        = {frame}  残差 {resid:.4f} mm   候选 "
                  f"{{{', '.join(f'{k}={v:.3f}' for k, v in diffs.items())}}}")
            break
        print()


def cmd_frames(paths):
    print(f"{'sample':<30}{'records':>9}{'local':>8}{'rotated':>9}{'world':>7}{'unknown':>9}{'worst mm':>11}")
    grand = collections.Counter()
    worst_all = 0.0
    for path in paths:
        tally = collections.Counter()
        worst = 0.0
        offenders = []
        for off, tag, d, stack in walk(path):
            if tag not in ("PRIM", "OBST", "INSU"):
                continue
            frame, resid, _ = classify_frame(d)
            if frame is None and resid is None:
                tally["skipped"] += 1      # 认不出规范盒的 kind（Line 等）
                continue
            tally[frame or "unknown"] += 1
            grand[frame or "unknown"] += 1
            if frame:
                worst = max(worst, resid)
            else:
                offenders.append((stack[-1] if stack else "?", KINDS.get(u32(d, 8), "?"), resid))
        worst_all = max(worst_all, worst)
        total = sum(v for k, v in tally.items() if k != "skipped")
        name = path.replace("\\", "/").rsplit("/", 1)[-1]
        print(f"{name:<30}{total:>9}{tally['local']:>8}{tally['rotated']:>9}"
              f"{tally['world']:>7}{tally['unknown']:>9}{worst:>11.4f}")
        for owner, kind, resid in offenders[:4]:
            print(f"    unknown: {kind} of {owner}  最小残差 {resid:.2f} mm")
    total = sum(grand.values())
    print(f"{'TOTAL':<30}{total:>9}{grand['local']:>8}{grand['rotated']:>9}"
          f"{grand['world']:>7}{grand['unknown']:>9}{worst_all:>11.4f}")
    print()
    print("判读：`unknown` 必须是 0。非 0 说明出现了本工具还没建模的图元，或 E3D 换了帧口径，")
    print("      读侧不得再拿记录字段当世界 AABB —— 退回顶点实算并出声。")


def cmd_fields(paths):
    c = collections.Counter()
    for path in paths:
        last = 0
        size = len(open(path, "rb").read())
        for off, tag, d, stack in walk(path):
            if tag == "__EOF__":
                last = off
                break
            if tag == "HEAD":
                c[f"HEAD v={u32(d,0)} n={u32(d,4)}"] += 1
            elif tag == "MODL":
                c[f"MODL v={u32(d,0)} n={u32(d,4)}"] += 1
            elif tag == "CNTB":
                p = 12 + u32(d, 8) * 4
                c[f"CNTB v={u32(d,0)} n={u32(d,4)} material={u32(d,p+12)}"] += 1
            elif tag == "CNTE":
                c[f"CNTE len={len(d)} body={d.hex()}"] += 1
            elif tag == "COLR":
                c[f"COLR len={len(d)} head={u32(d,0)},{u32(d,4)},{u32(d,8)} color={u32(d,12):08x}"] += 1
            elif tag in ("PRIM", "OBST", "INSU"):
                c[f"{tag} v={u32(d,0)} flags={u32(d,4)}"] += 1
            elif tag.startswith("END"):
                c[f"tag={tag!r} len={len(d)} body={d.hex()}"] += 1
        c[f"padding {last} -> {size} (160 的整数倍: {size % 160 == 0})"] += 1
    for k, v in sorted(c.items()):
        print(f"{v:6d}  {k}")


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    cmd, rest = sys.argv[1], sys.argv[2:]
    paths = [p for arg in rest if not arg.isdigit() for p in (glob.glob(arg) or [arg])]
    if cmd == "skeleton":
        limit = next((int(a) for a in rest if a.isdigit()), 40)
        cmd_skeleton(paths, limit)
    elif cmd == "facet":
        cmd_facet(paths)
    elif cmd == "frames":
        cmd_frames(paths)
    elif cmd == "fields":
        cmd_fields(paths)
    else:
        print(__doc__)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())

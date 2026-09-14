//! 把 RVM 里名字含指定子串的 group 的几何**原样**倒成 JSON（世界 mm）。
//!
//! `rvm_compare` 只比 AABB 不变量；这个把 FacetGroup 的多边形/轮廓/顶点全部倒出来，
//! 用于把 E3D 导出的剖面逐顶点对回库里的 PLOO/PAVE 环（GWALL 轮廓差异取证）。
//!
//! ```text
//! cargo run --release --example rvm_facets_dump -- \
//!     --rvm <path.rvm> --contains "of CWALL /1RS-WF03-W-C-RR001" --out facets.json
//! ```
//!
//! 顶点坐标 = rvm-rs 局部顶点 × group 内 transform × 1000（RVM 存米，输出世界 mm），
//! 与 gen-model `mesh_compare::add_local_triangulation` 同一口径。
//!
//! 非 FacetGroup 图元逐件与 PRIM 记录自带的局部 bbox 对账，按对账结果决定三角化是原样
//! 用还是先从 rvm-rs 旧版的 Y-up 局部系转回 RVM 的 Z-up 再乘 transform（见
//! [`rvm_local_vertices`]）；两种都对不上的件数与样例打在末尾——图元帧口径再漂就当场看得见。

use std::path::PathBuf;

use anyhow::{Context, bail};
use rvm_rs::export::Tessellate;
use rvm_rs::export::tessellator::get_scale;
use rvm_rs::parse_rvm;
use rvm_rs::store::Store;
use rvm_rs::store::geometry::GeometryKind;
use rvm_rs::store::node::{NodeId, NodeKind};

const M_TO_MM: f32 = 1000.0;

/// rvm-rs 到 rev 45470d2 为止的图元三角化用的是 **Y-up** 局部系：`Cylinder` / `Pyramid` 沿 +Y
/// 从 0 长到 height，`Snout` / 球面族 / 两种 Torus 把轴放在 Y、扫掠面放在 X–Z。RVM 的 PRIM
/// 记录却是 **Z-up**，且 `Cylinder` / `Pyramid` 沿 Z 居中（记录自带 bbox 就是 ±height/2）。
/// rvm-rs 自己的 glTF / OBJ 导出直接拿 transform 乘这些顶点，圆柱与弯头整体转了 90°——
/// 2026-09-02 用 ams7999 `/1RCV0101-60.3-VMB-R23-R342/B1` 的 `TUBE 2` 坐实：真值沿世界 Y
/// 水平 801 mm，倒出来却沿 Z 竖直、起点在圆柱中心（happyrust/rvm-rs#3，修复 #4）。
///
/// 这里不写死 rev：先按 PRIM 记录 bbox 判三角化**原样**是否已在记录帧里（修复后的 rvm-rs、
/// 以及 `Box` / `Sphere` 这类怎么放都对称的），是就原样用；不是再试 Y→Z 转换；两种都对不上
/// 照原样倒出并计一笔口径不合。这样 rvm-rs 升到修复版时本文件一行不用改，也不会双重转换。
fn y_up_to_record_frame(kind: &GeometryKind, tri: &rvm_rs::export::Triangulation) -> Vec<[f32; 3]> {
    let axis_shift = match kind {
        GeometryKind::Cylinder(cylinder) => -cylinder.height / 2.0,
        GeometryKind::Pyramid(pyramid) => -pyramid.height / 2.0,
        _ => 0.0,
    };
    tri.vertices
        .as_chunks::<3>()
        .0
        .iter()
        .map(|[x, y, z]| [*x, *z, *y + axis_shift])
        .collect()
}

fn as_is(tri: &rvm_rs::export::Triangulation) -> Vec<[f32; 3]> {
    tri.vertices.as_chunks::<3>().0.to_vec()
}

/// rvm-rs 的 `Pyramid::tessellate` 把 XOFF/YOFF **整个**记到顶面（底面心钉在原点）。
/// E3D 是两端各摊一半：libgm.dll `GM_Pyramid::calcFacets`（0x10094b10）底面四点取
/// ±xbot/2 − xoff/2、±ybot/2 − yoff/2，顶面四点取 ±xtop/2 + xoff/2、±ytop/2 + yoff/2。
/// 形状一模一样，只差一个整体平移 (+ox/2, +oy/2)——减掉它就回到记录帧。
///
/// 不减的后果：盒子沿偏心方向整体涨 |off|/2。`/TEMP_EQUIP` PIPE 2 的弯头（两只 YOFF 50
/// 的棱台）因此让基准 AABB 虚高 25 mm，我们本来画对的几何被判成 off。
///
/// 与 [`y_up_to_record_frame`] 同一条原则：这里只是**多给一个候选**，选不选由记录
/// bbox 说了算。上游改好后这一档匹配不上，自然不会被选中，本文件一行不用动。
fn undo_pyramid_shift(kind: &GeometryKind, points: &[[f32; 3]]) -> Option<Vec<[f32; 3]>> {
    let GeometryKind::Pyramid(pyramid) = kind else {
        return None;
    };
    let [ox, oy] = pyramid.offset;
    if ox == 0.0 && oy == 0.0 {
        return None;
    }
    Some(
        points
            .iter()
            .map(|[x, y, z]| [x - ox / 2.0, y - oy / 2.0, *z])
            .collect(),
    )
}

/// 选出与记录 bbox 同口径的那份局部顶点；`Some(true)` = 原样（或只去掉棱台偏心）就对，
/// `Some(false)` = 还用了 Y→Z 转换，`None` = 都对不上（照原样倒出）。
fn rvm_local_vertices(
    kind: &GeometryKind,
    tri: &rvm_rs::export::Triangulation,
    record: &[f32; 6],
) -> (Vec<[f32; 3]>, Option<bool>) {
    // 顺序即优先级：先不转帧、后转帧，每一档后面紧跟它去掉棱台偏心的版本。
    let mut candidates = Vec::with_capacity(4);
    for (points, in_record_frame) in [
        (as_is(tri), Some(true)),
        (y_up_to_record_frame(kind, tri), Some(false)),
    ] {
        let deshifted = undo_pyramid_shift(kind, &points);
        candidates.push((points, in_record_frame));
        candidates.extend(deshifted.map(|points| (points, in_record_frame)));
    }
    let fallback = candidates[0].0.clone();
    for (points, in_record_frame) in candidates {
        if aabb_of(&points).is_some_and(|aabb| matches_record_bbox(&aabb, record)) {
            return (points, in_record_frame);
        }
    }
    (fallback, None)
}

fn aabb_of(points: &[[f32; 3]]) -> Option<[f32; 6]> {
    let first = points.first()?;
    let mut lo = *first;
    let mut hi = *first;
    for point in points {
        for axis in 0..3 {
            lo[axis] = lo[axis].min(point[axis]);
            hi[axis] = hi[axis].max(point[axis]);
        }
    }
    Some([lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]])
}

/// 三角化局部 AABB 与 PRIM 记录 bbox 是否同一口径。容差 = 1 mm + 8 % 该轴记录跨度：
/// 圆周按 1 mm 弦高最少 8 段，某轴没有顶点正落在极值角上时最多缩进 r(1−cos(π/8)) ≈ 7.6 %。
fn matches_record_bbox(tessellated: &[f32; 6], record: &[f32; 6]) -> bool {
    (0..3).all(|axis| {
        let tolerance = 1.0 + 0.08 * (record[axis + 3] - record[axis]).abs();
        (tessellated[axis] - record[axis]).abs() <= tolerance
            && (tessellated[axis + 3] - record[axis + 3]).abs() <= tolerance
    })
}

/// 补密后每一圈允许的弦高（mm）。补密带来的残差要落在判据的噪声底下。
const RING_SAG_MM: f64 = 0.05;
/// 原采样弦高不超过它就不补：`rvm_compare` 的基本体弦高容差是 0.5mm，比它小一半以上
/// 的圈根本判不出差别，补了只是把产物撑大（一个 2684 件的 ZONE 从 1MB 撑到 19MB）。
const RING_SKIP_SAG_MM: f64 = 0.2;
/// 一圈最多补到多少个点。半径再大也不让产物失控；14 m 的罐顶按 0.05mm 弦高要 1176 个，
/// 封到 1200 刚好够，且 1200 是 4 的倍数，四个象限点必在网格上。
const RING_MAX_SAMPLES: usize = 1200;

/// 只补**回转体**。盒与棱锥的等高点不在一个圆上，三点定圆会把外接圆的极值补出去——
/// 实测一只 `BOX` 会被撑大 360 mm。下面每一圈还要再过一道残差门，双保险。
fn is_body_of_revolution(kind: &GeometryKind) -> bool {
    matches!(
        kind,
        GeometryKind::Cylinder(_)
            | GeometryKind::Sphere(_)
            | GeometryKind::SphericalDish(_)
            | GeometryKind::EllipticalDish(_)
            | GeometryKind::Snout(_)
    )
}

/// 把回转图元采样太稀的圈**补密**（记录帧，只补顶点不补面）。
///
/// rvm-rs 的 `sagitta_based_segment_count(..., 8, 64)` 把整圈采样数**封顶 64**，回转体
/// 每一圈还按 `sin θ` 再往下减。半径一大，容差就说了不算：`SphericalDish`
/// `base_radius=14000` 的底圈实测只有二十几个点，某个方向上没采到极值角，AABB 比真圆
/// 缩进 **94.7 mm**——而 E3D 在 PRIM 记录里写的 bbox 正是 ±14000，一分不差。那 94.7 mm
/// 是我们这套基准工具铺出来的，不是 E3D 的几何。
///
/// 每一圈按三点定圆求出**精确**的圆心与半径，采样不够密就按 [`RING_SAG_MM`] 重铺。
/// 补进去的点都严格落在解析面上，AABB 只会更接近真值，不会撑大。
///
/// 为什么不只补「每根轴上那两个极值点」（那样只要十几个点）：`rvm_compare` 是在
/// **元素局部帧**里比的，这里却只看得见世界帧，按世界轴算出来的极值点换个帧就不再是
/// 极值。同一个 ZONE 里六只一模一样的罐顶，只有机位正好轴对齐的那两只被修好——
/// 就是这么露的馅。
fn ring_densification(local: &[[f32; 3]]) -> Vec<[f32; 3]> {
    // 按 z 分圈：回转体的采样点天然落在若干等高圈上。
    let mut rings: Vec<(f32, Vec<[f32; 3]>)> = Vec::new();
    for &p in local {
        match rings.iter_mut().find(|(z, _)| (z - p[2]).abs() <= 1.0e-6) {
            Some((_, bag)) => bag.push(p),
            None => rings.push((p[2], vec![p])),
        }
    }
    let mut out = Vec::new();
    for (z, bag) in &rings {
        let Some((centre, radius)) = circle_through_spread_triple(bag) else {
            continue;
        };
        // 这一圈真的是个圆吗：所有点到拟合圆心的距离都得等于半径。不等就别补。
        let residual = bag
            .iter()
            .map(|p| {
                ((f64::from(p[0]) - centre[0]).hypot(f64::from(p[1]) - centre[1]) - radius).abs()
            })
            .fold(0.0_f64, f64::max);
        if residual > radius * 1.0e-4 + 1.0e-9 {
            continue;
        }
        // 原采样的弦高 r(1 − cos(π/n))；判据看不出差别就别补。
        let have = radius * (1.0 - (std::f64::consts::PI / bag.len() as f64).cos());
        if have <= RING_SKIP_SAG_MM {
            continue;
        }
        // 弦高 r(1 − cos(π/n)) ≤ RING_SAG_MM 要的最少段数；本来就够密就别动。
        let ratio = (1.0 - RING_SAG_MM / radius).clamp(-1.0, 1.0);
        let wanted = (std::f64::consts::PI / ratio.acos()).ceil();
        if !wanted.is_finite() {
            continue;
        }
        let wanted = (wanted as usize).min(RING_MAX_SAMPLES);
        if wanted <= bag.len() {
            continue;
        }
        for i in 0..wanted {
            let angle = std::f64::consts::TAU * i as f64 / wanted as f64;
            out.push([
                (centre[0] + radius * angle.cos()) as f32,
                (centre[1] + radius * angle.sin()) as f32,
                *z,
            ]);
        }
    }
    out
}

/// 三点定圆，挑面积最大的那一组三点以免共线病态。点都在同一个圆上时结果是精确的。
fn circle_through_spread_triple(ring: &[[f32; 3]]) -> Option<([f64; 2], f64)> {
    if ring.len() < 3 {
        return None;
    }
    let p = |i: usize| [f64::from(ring[i][0]), f64::from(ring[i][1])];
    let n = ring.len();
    let (a, b, c) = (p(0), p(n / 3), p(2 * n / 3));
    let d = 2.0 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]));
    if d.abs() <= 1.0e-9 {
        return None;
    }
    let sq = |v: [f64; 2]| v[0] * v[0] + v[1] * v[1];
    let centre = [
        (sq(a) * (b[1] - c[1]) + sq(b) * (c[1] - a[1]) + sq(c) * (a[1] - b[1])) / d,
        (sq(a) * (c[0] - b[0]) + sq(b) * (a[0] - c[0]) + sq(c) * (b[0] - a[0])) / d,
    ];
    let radius = (a[0] - centre[0]).hypot(a[1] - centre[1]);
    radius.is_finite().then_some((centre, radius))
}

fn triangulation_world_polygons(
    indices: &[u32],
    local: &[[f32; 3]],
    cols: &[f32; 12],
) -> Vec<Vec<Vec<[f32; 3]>>> {
    let world = |index: u32| {
        let [x, y, z] = local[index as usize];
        [
            (cols[0] * x + cols[3] * y + cols[6] * z + cols[9]) * M_TO_MM,
            (cols[1] * x + cols[4] * y + cols[7] * z + cols[10]) * M_TO_MM,
            (cols[2] * x + cols[5] * y + cols[8] * z + cols[11]) * M_TO_MM,
        ]
    };
    indices
        .as_chunks::<3>()
        .0
        .iter()
        .map(|face| vec![vec![world(face[0]), world(face[1]), world(face[2])]])
        .collect()
}

#[derive(serde::Serialize)]
struct DumpedGroup {
    name: String,
    /// CNTB 平移（RVM 原值，米）。
    translation_m: [f32; 3],
    geometries: Vec<DumpedGeometry>,
}

#[derive(serde::Serialize)]
struct DumpedGeometry {
    kind: String,
    /// 非 FacetGroup 的参数原样带出来（Debug 文本，取证够用）。
    params: Option<String>,
    /// Affine3A 列主序 3×3 + 平移（RVM 原值，米）。
    transform_cols_m: [f32; 12],
    /// PRIM 记录里原样存的 24 字节 bbox（rvm-rs `bbox_local`，未变换，米）。
    bbox_record_m: [f32; 6],
    bbox_world_mm: [f32; 6],
    /// 非 FacetGroup 图元：选定帧下的三角化局部 AABB（图元原单位），
    /// 以及它与 `bbox_record_m` 是否同一口径（见 [`matches_record_bbox`]）。
    tessellated_local_aabb: Option<[f32; 6]>,
    matches_record_bbox: Option<bool>,
    /// 非 FacetGroup 图元：三角化是否原样就在记录帧里（`false` = 做了 Y→Z 转换）。
    tessellation_in_record_frame: Option<bool>,
    /// FacetGroup：多边形 → 轮廓 → 顶点（**世界 mm**，已应用 transform）。
    polygons_world_mm: Option<Vec<Vec<Vec<[f32; 3]>>>>,
}

#[derive(Default)]
struct RecordAudit {
    primitives: usize,
    /// 原样就在记录帧里的件数（修复后的 rvm-rs 应等于 primitives − mismatches）。
    in_record_frame: usize,
    /// 靠 Y→Z 转换才对上记录 bbox 的件数（rev 45470d2 及之前的 rvm-rs）。
    converted: usize,
    mismatches: usize,
    samples: Vec<String>,
}

fn main() -> anyhow::Result<()> {
    let mut rvm_path: Option<PathBuf> = None;
    let mut contains: Option<String> = None;
    let mut out: Option<PathBuf> = None;

    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--rvm" => rvm_path = Some(PathBuf::from(value()?)),
            "--contains" => contains = Some(value()?),
            "--out" => out = Some(PathBuf::from(value()?)),
            other => bail!("不认识的参数 {other}"),
        }
    }
    let rvm_path = rvm_path.context("缺 --rvm")?;
    let contains = contains.context("缺 --contains")?;

    let bytes = std::fs::read(&rvm_path)
        .with_context(|| format!("读取 RVM 失败: {}", rvm_path.display()))?;
    let mut store = Store::new();
    parse_rvm(&bytes, &mut store).context("解析 RVM 失败")?;

    let mut dumped: Vec<DumpedGroup> = Vec::new();
    let mut audit = RecordAudit::default();
    fn walk(
        store: &Store,
        node_id: NodeId,
        contains: &str,
        dumped: &mut Vec<DumpedGroup>,
        audit: &mut RecordAudit,
    ) {
        let Some(node) = store.get_node(node_id) else {
            return;
        };
        if let NodeKind::Group(group) = &node.kind {
            let name = store.get_string(group.name).trim().to_string();
            if name.contains(contains) {
                let mut geometries = Vec::new();
                let mut link = group.first_geometry;
                while let Some(gid) = link {
                    let Some(geometry) = store.get_geometry(gid) else {
                        break;
                    };
                    let cols = geometry.transform.to_cols_array();
                    let bb = &geometry.bbox_world;
                    let bl = &geometry.bbox_local;
                    let record = [bl.min.x, bl.min.y, bl.min.z, bl.max.x, bl.max.y, bl.max.z];
                    let mut tessellated_local_aabb = None;
                    let mut record_match = None;
                    let mut in_record_frame = None;
                    let (kind, params, polygons) = match &geometry.kind {
                        GeometryKind::FacetGroup(fg) => {
                            let polys = fg
                                .polygons
                                .iter()
                                .map(|poly| {
                                    poly.contours
                                        .iter()
                                        .map(|contour| {
                                            contour
                                                .vertices
                                                .iter()
                                                .map(|v| {
                                                    let wx = cols[0] * v.x
                                                        + cols[3] * v.y
                                                        + cols[6] * v.z
                                                        + cols[9];
                                                    let wy = cols[1] * v.x
                                                        + cols[4] * v.y
                                                        + cols[7] * v.z
                                                        + cols[10];
                                                    let wz = cols[2] * v.x
                                                        + cols[5] * v.y
                                                        + cols[8] * v.z
                                                        + cols[11];
                                                    [wx * M_TO_MM, wy * M_TO_MM, wz * M_TO_MM]
                                                })
                                                .collect::<Vec<_>>()
                                        })
                                        .collect::<Vec<_>>()
                                })
                                .collect::<Vec<_>>();
                            ("FacetGroup".to_string(), None, Some(polys))
                        }
                        // 中心线不是体，没有面片可转；留一条空记录让对拍脚本按 kind 跳过。
                        GeometryKind::Line(line) => {
                            ("Line".to_string(), Some(format!("{line:?}")), None)
                        }
                        other => {
                            // PRIM 记录自带 bbox 已被实测证明不可信；非 FacetGroup 也必须
                            // 从实际图元三角化顶点重算 AABB，不能让对拍退回该字段。
                            let scale = get_scale(&geometry.transform.matrix3);
                            let tol = 1.0e-3_f32; // RVM 单位 m，即 1mm 弦高容差。
                            let tri = match other {
                                GeometryKind::Cylinder(g) => g.tessellate(tol, scale),
                                GeometryKind::Sphere(g) => g.tessellate(tol, scale),
                                GeometryKind::Box(g) => g.tessellate(tol, scale),
                                GeometryKind::Pyramid(g) => g.tessellate(tol, scale),
                                GeometryKind::CircularTorus(g) => g.tessellate(tol, scale),
                                GeometryKind::RectangularTorus(g) => g.tessellate(tol, scale),
                                GeometryKind::EllipticalDish(g) => g.tessellate(tol, scale),
                                GeometryKind::SphericalDish(g) => g.tessellate(tol, scale),
                                GeometryKind::Snout(g) => g.tessellate(tol, scale),
                                GeometryKind::Line(_) | GeometryKind::FacetGroup(_) => {
                                    unreachable!()
                                }
                            };
                            let kind_name = format!("{other:?}")
                                .split('(')
                                .next()
                                .unwrap_or("?")
                                .to_string();
                            let (local, frame) = rvm_local_vertices(other, &tri, &record);
                            let local_aabb = aabb_of(&local);
                            audit.primitives += 1;
                            in_record_frame = frame;
                            match frame {
                                Some(true) => audit.in_record_frame += 1,
                                Some(false) => audit.converted += 1,
                                None => {
                                    audit.mismatches += 1;
                                    if audit.samples.len() < 8 {
                                        audit.samples.push(format!(
                                            "{name} / {kind_name}: 三角化 {local_aabb:.1?} vs 记录 {record:.1?}"
                                        ));
                                    }
                                }
                            }
                            record_match = Some(frame.is_some());
                            tessellated_local_aabb = local_aabb;
                            let mut polys = triangulation_world_polygons(&tri.indices, &local, &cols);
                            // rvm-rs 的采样封顶会让大半径回转体缩进（见 [`ring_extremes`]）；
                            // 把每一圈的精确极值点补成一条附加多边形，AABB 才是解析值。
                            let extremes = if is_body_of_revolution(other) {
                                ring_densification(&local)
                            } else {
                                Vec::new()
                            };
                            if !extremes.is_empty() {
                                polys.push(vec![
                                    extremes
                                        .iter()
                                        .map(|v| {
                                            [
                                                (cols[0] * v[0]
                                                    + cols[3] * v[1]
                                                    + cols[6] * v[2]
                                                    + cols[9])
                                                    * M_TO_MM,
                                                (cols[1] * v[0]
                                                    + cols[4] * v[1]
                                                    + cols[7] * v[2]
                                                    + cols[10])
                                                    * M_TO_MM,
                                                (cols[2] * v[0]
                                                    + cols[5] * v[1]
                                                    + cols[8] * v[2]
                                                    + cols[11])
                                                    * M_TO_MM,
                                            ]
                                        })
                                        .collect(),
                                ]);
                            }
                            (kind_name, Some(format!("{other:?}")), Some(polys))
                        }
                    };
                    geometries.push(DumpedGeometry {
                        kind,
                        params,
                        transform_cols_m: cols,
                        bbox_record_m: record,
                        bbox_world_mm: [
                            bb.min.x * M_TO_MM,
                            bb.min.y * M_TO_MM,
                            bb.min.z * M_TO_MM,
                            bb.max.x * M_TO_MM,
                            bb.max.y * M_TO_MM,
                            bb.max.z * M_TO_MM,
                        ],
                        tessellated_local_aabb,
                        matches_record_bbox: record_match,
                        tessellation_in_record_frame: in_record_frame,
                        polygons_world_mm: polygons,
                    });
                    link = geometry.next;
                }
                dumped.push(DumpedGroup {
                    name,
                    translation_m: group.translation.to_array(),
                    geometries,
                });
            }
        }
        let mut child = node.first_child;
        while let Some(cid) = child {
            let Some(cnode) = store.get_node(cid) else {
                break;
            };
            walk(store, cid, contains, dumped, audit);
            child = cnode.next;
        }
    }
    for &root in store.roots() {
        walk(&store, root, &contains, &mut dumped, &mut audit);
    }

    println!("命中 {} 个 group（名字含 `{}`）", dumped.len(), contains);
    println!(
        "图元三角化 vs PRIM 记录 bbox：{} 件——原样在记录帧 {} 件，经 Y→Z 转换 {} 件，口径不合 {} 件",
        audit.primitives, audit.in_record_frame, audit.converted, audit.mismatches
    );
    for sample in &audit.samples {
        println!("  口径不合：{sample}");
    }
    let json = serde_json::to_vec(&dumped)?;
    match out {
        Some(path) => {
            std::fs::write(&path, &json)?;
            println!("写到 {}（{} 字节）", path.display(), json.len());
        }
        None => println!("{}", String::from_utf8_lossy(&json)),
    }
    Ok(())
}

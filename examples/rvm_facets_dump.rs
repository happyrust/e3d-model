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

/// 选出与记录 bbox 同口径的那份局部顶点；`Some(true)` = 原样就对，`Some(false)` = 用了 Y→Z
/// 转换，`None` = 两种都对不上（照原样倒出）。
fn rvm_local_vertices(
    kind: &GeometryKind,
    tri: &rvm_rs::export::Triangulation,
    record: &[f32; 6],
) -> (Vec<[f32; 3]>, Option<bool>) {
    let identity = as_is(tri);
    let Some(identity_aabb) = aabb_of(&identity) else {
        return (identity, None);
    };
    if matches_record_bbox(&identity_aabb, record) {
        return (identity, Some(true));
    }
    let converted = y_up_to_record_frame(kind, tri);
    match aabb_of(&converted) {
        Some(aabb) if matches_record_bbox(&aabb, record) => (converted, Some(false)),
        _ => (identity, None),
    }
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
                            (
                                kind_name,
                                Some(format!("{other:?}")),
                                Some(triangulation_world_polygons(&tri.indices, &local, &cols)),
                            )
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

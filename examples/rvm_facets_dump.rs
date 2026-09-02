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

use std::path::PathBuf;

use anyhow::{Context, bail};
use rvm_rs::export::Tessellate;
use rvm_rs::export::tessellator::get_scale;
use rvm_rs::parse_rvm;
use rvm_rs::store::Store;
use rvm_rs::store::geometry::GeometryKind;
use rvm_rs::store::node::{NodeId, NodeKind};

const M_TO_MM: f32 = 1000.0;

fn triangulation_world_polygons(
    tri: &rvm_rs::export::Triangulation,
    cols: &[f32; 12],
) -> Vec<Vec<Vec<[f32; 3]>>> {
    let world = |index: u32| {
        let i = index as usize * 3;
        let (x, y, z) = (tri.vertices[i], tri.vertices[i + 1], tri.vertices[i + 2]);
        [
            (cols[0] * x + cols[3] * y + cols[6] * z + cols[9]) * M_TO_MM,
            (cols[1] * x + cols[4] * y + cols[7] * z + cols[10]) * M_TO_MM,
            (cols[2] * x + cols[5] * y + cols[8] * z + cols[11]) * M_TO_MM,
        ]
    };
    tri.indices
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
    /// FacetGroup：多边形 → 轮廓 → 顶点（**世界 mm**，已应用 transform）。
    polygons_world_mm: Option<Vec<Vec<Vec<[f32; 3]>>>>,
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
    fn walk(store: &Store, node_id: NodeId, contains: &str, dumped: &mut Vec<DumpedGroup>) {
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
                            (
                                format!("{other:?}")
                                    .split('(')
                                    .next()
                                    .unwrap_or("?")
                                    .to_string(),
                                Some(format!("{other:?}")),
                                Some(triangulation_world_polygons(&tri, &cols)),
                            )
                        }
                    };
                    let bl = &geometry.bbox_local;
                    geometries.push(DumpedGeometry {
                        kind,
                        params,
                        transform_cols_m: cols,
                        bbox_record_m: [bl.min.x, bl.min.y, bl.min.z, bl.max.x, bl.max.y, bl.max.z],
                        bbox_world_mm: [
                            bb.min.x * M_TO_MM,
                            bb.min.y * M_TO_MM,
                            bb.min.z * M_TO_MM,
                            bb.max.x * M_TO_MM,
                            bb.max.y * M_TO_MM,
                            bb.max.z * M_TO_MM,
                        ],
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
            walk(store, cid, contains, dumped);
            child = cnode.next;
        }
    }
    for &root in store.roots() {
        walk(&store, root, &contains, &mut dumped);
    }

    println!("命中 {} 个 group（名字含 `{}`）", dumped.len(), contains);
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

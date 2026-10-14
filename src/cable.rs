//! `CABLE` / `EXTGEO`：电缆的「外部几何段」锚定（gen-model #35）。
//!
//! `createCurvesFromPath` 里 RPATH 成员的 `EXTGEO` 走 `sub_105D13B0` 出外部几何段：
//! 段的两个端点**不在本 RPATH 里**，而是 `STAREF` / `ENDREF` 指着的桥架附着件——
//! 「外部几何」指的就是借桥架（`CWAY → CWBRAN → RPATH`）的路由。
//!
//! 语料取证（2026-10-15，aps250176/aps250179 直读）：
//!
//! - `CABLE` 自带总端点 `STAREF`/`ENDREF`（语料里就有跨库指向）、`SPRE` 指向
//!   `TABITE→GPART→SCOM(GTYP=CABL)→CCORSE` 目录链；`RPATH` 成员混排
//!   `POINTR`（本地点串）与 `EXTGEO`（借桥架段）。
//! - `EXTGEO` 只有 `STAREF`/`ENDREF`/`XOFF`/`YGAP` 四个有效属性。
//! - 附着件三 noun：`HATTA`（CWBRAN 头）、`TATTA`（尾）、`RATTA`（途中，`CONSEQ` 名下，
//!   带 `POSREF`+`ZDIS`/`PKDI`）。T/HATTA 没有 POSREF——按「分支路径首/末点」解。
//! - 全语料：CABLE×116、EXTGEO×280、CWBRAN×1699、RATTA×349、T/HATTA×1695。
//!
//! 本模块只解算**锚点位置**；段体怎么画（顺桥架子路径还是直弦、`XOFF`/`YGAP` 怎么偏、
//! 电缆直径从哪条目录链出）依赖 `sub_105D13B0` 的语义钉死或 RVM 真值，见 `_probe` 测试。

use e3d_io::db_element::{DbElement, DbElementError};
use e3d_io::refno::RefNo;
use glam::DVec3;

use crate::rpath::{
    Resolved, RoutePath, first_rpath, number_or_zero, reference, resolve_anchor, straight_point,
};
use crate::transform::world_matrix;

/// `HATTA` / `TATTA` / `RATTA` / `RNODE`：桥架分支（`CWBRAN`）路径上的附着/路由节点。
/// 与 core 的 `CAB_CSGRouteNode` noun 闸（`sub_102B5530`：HATTA/TATTA/RATTA/RNODE）
/// 同一张名单；`RNODE` 挂 `RTFEAT` 下、带 `POSREF`+`ZDIS`+`XOFF`/`YOFF`/`ZOFF`。
pub fn is_cableway_attachment(noun: &str) -> bool {
    matches!(noun, "HATTA" | "TATTA" | "RATTA" | "RNODE")
}

/// 附着件所在的 `CWBRAN`：`HATTA`/`TATTA` 直接挂 CWBRAN 下，`RATTA` 隔一层 `CONSEQ`，
/// `RNODE` 隔一层 `RTFEAT`。
fn owning_branch(atta: &DbElement) -> Result<Option<DbElement>, DbElementError> {
    let mut cursor = atta.owner()?;
    for _ in 0..3 {
        let Some(owner) = cursor else {
            return Ok(None);
        };
        if owner.element_type()? == "CWBRAN" {
            return Ok(Some(owner));
        }
        cursor = owner.owner()?;
    }
    Ok(None)
}

/// 附着件在其 `CWBRAN` 路径上的落点：局部位置 + 站号（落在第几段、段内比例），
/// 供 EXTGEO 截取「两锚点之间的桥架子路径」。
#[derive(Debug, Clone)]
pub(crate) struct AttachmentAnchor {
    /// 附着件所在的 `CWBRAN`。
    pub cwbran: DbElement,
    /// CWBRAN 局部系里的位置。
    pub position: DVec3,
    /// 路径点下标：落点 >= points[station]，< points[station+1]（末点时 station=最后下标）。
    pub station: usize,
}

/// 附着件在其 `CWBRAN` **局部系**里的落点。
///
/// `RATTA`/`RNODE` 按 `POSREF`→`POINTR` + `ZDIS`/`PKDI` 在桥架路径上插值
/// （`sub_109E71E0`，与桥架件同一套锚定）；`HATTA`/`TATTA` 没有 `POSREF`，按分支
/// 路径首/末点解——这一约定由 `_probe` 语料探针的连续性检查兜底验证。
pub(crate) fn attachment_anchor(
    atta: &DbElement,
) -> Result<Option<AttachmentAnchor>, DbElementError> {
    let noun = atta.element_type()?;
    if !is_cableway_attachment(&noun) {
        return Ok(None);
    }
    let Some(cwbran) = owning_branch(atta)? else {
        return Ok(None);
    };
    let Some(rpath) = first_rpath(&cwbran)? else {
        return Ok(None);
    };
    let path = RoutePath::load(atta, &rpath, &cwbran)?;
    if path.points.is_empty() {
        return Ok(None);
    }
    let (position, station) = match noun.as_str() {
        "HATTA" => (path.points[0].pos, 0),
        "TATTA" => {
            let last = path.points.len() - 1;
            (path.points[last].pos, last)
        }
        // RATTA / RNODE：POSREF 链 + ZDIS/PKDI；`ZOFF` 抬高 CWBRAN 的 Z。
        _ => {
            let pkdi = number_or_zero(atta, "PKDI")?;
            let zdis = number_or_zero(atta, "ZDIS")?;
            let zoff = number_or_zero(atta, "ZOFF")?;
            let resolved = match reference(atta, "POSREF")? {
                Some(posref) => resolve_anchor(atta, &cwbran, posref, pkdi, zdis, true)?.1,
                // 无 POSREF：Core 的 `readMiddleFormDB` 那支取 RPATH 首段。
                _ => Resolved::Segment {
                    a: 0,
                    b: usize::from(path.points.len() > 1),
                    pkdi,
                    zdis,
                },
            };
            match resolved {
                Resolved::Segment { a, b, pkdi, zdis } => (
                    straight_point(&path, a, b, pkdi, zdis, zoff, 0.0).position,
                    a,
                ),
                Resolved::EndPoint(index) => {
                    (path.points[index].pos + DVec3::new(0.0, 0.0, zoff), index)
                }
            }
        }
    };
    Ok(Some(AttachmentAnchor {
        cwbran,
        position,
        station,
    }))
}

/// 附着件的**世界系**位置（`world(CWBRAN) ∘ 局部落点`）。
pub fn attachment_world_position(atta: &DbElement) -> Result<Option<DVec3>, DbElementError> {
    let Some(anchor) = attachment_anchor(atta)? else {
        return Ok(None);
    };
    Ok(Some(
        world_matrix(&anchor.cwbran)?.transform_point3(anchor.position),
    ))
}

/// `EXTGEO` 段两端的世界系位置：`(STAREF 端, ENDREF 端)`。任一端解不出（引用空、
/// 指向的不是附着件、附着件的桥架没路径）给 `None`。
pub fn extgeo_world_endpoints(
    extgeo: &DbElement,
) -> Result<Option<(DVec3, DVec3)>, DbElementError> {
    if extgeo.element_type()? != "EXTGEO" {
        return Ok(None);
    }
    let endpoint = |name: &str| -> Result<Option<DVec3>, DbElementError> {
        let Some(atta) = reference(extgeo, name)? else {
            return Ok(None);
        };
        attachment_world_position(&atta)
    };
    let (Some(start), Some(end)) = (endpoint("STAREF")?, endpoint("ENDREF")?) else {
        return Ok(None);
    };
    Ok(Some((start, end)))
}

/// `CABLE` 名下的第一个 `RPATH`。
pub(crate) fn cable_rpath(cable: &DbElement) -> Result<Option<DbElement>, DbElementError> {
    if cable.element_type()? != "CABLE" {
        return Ok(None);
    }
    first_rpath(cable)
}

/// `CABLE` RPATH 的成员序列里，每个成员提供的世界系端点：POINTR 是单点（自身
/// `POS` 经属主链）、EXTGEO 是借桥架的两端点。给连续性检查与将来的段体拼接用。
pub(crate) fn member_world_endpoints(
    cable: &DbElement,
) -> Result<Vec<(RefNo, String, Vec<DVec3>)>, DbElementError> {
    let mut out = Vec::new();
    let Some(rpath) = cable_rpath(cable)? else {
        return Ok(out);
    };
    for member in rpath.members()? {
        let noun = member.element_type()?;
        let points = match noun.as_str() {
            "POINTR" => vec![world_matrix(&member)?.transform_point3(DVec3::ZERO)],
            "EXTGEO" => extgeo_world_endpoints(&member)?
                .map(|(a, b)| vec![a, b])
                .unwrap_or_default(),
            _ => Vec::new(),
        };
        out.push((member.refno(), noun, points));
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::db_discovery::DirectoryDbResolver;
    use e3d_io::db_element::DbSet;
    use e3d_io::{ReadOnlyEngine, ScanTier};
    use std::path::PathBuf;
    use std::sync::Arc;

    fn corpus_dirs() -> Vec<PathBuf> {
        std::env::var("AIOS_CABLE_PROBE_DIRS")
            .map(|value| value.split(';').map(PathBuf::from).collect())
            .unwrap_or_else(|_| {
                vec![
                    PathBuf::from(r"D:\AVEVA\Projects\E3D3.1\AvevaPlantSample\aps000"),
                    PathBuf::from(r"D:\AVEVA\Projects\E3D3.1\AvevaMarineSample\ams000"),
                    PathBuf::from(r"D:\AVEVA\Projects\E3D3.1\AvevaCatalogue\acp000"),
                ]
            })
    }

    fn corpus_set(dirs: &[PathBuf]) -> Option<Arc<DbSet>> {
        let template =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../e3d-io/schema/e3d31/binaries");
        if !template.join("attlib.dat").exists() {
            return None;
        }
        let resolver = DirectoryDbResolver::scan(dirs, &template).ok()?;
        Some(Arc::new(
            DbSet::with_attlib_file_and_resolver(&template.join("attlib.dat"), Box::new(resolver))
                .ok()?,
        ))
    }

    /// A2 语料验证：全语料 `CABLE` 的 RPATH 成员链连续性——POINTR 端点与相邻
    /// `EXTGEO` 解出来的附着件世界位置应首尾相接（HATTA/TATTA 的「首/末点」约定、
    /// RATTA 的 `POSREF` 插值都在这条检查上显形）。断口 ≫ 桥架转弯半径就是语义错了。
    #[test]
    #[ignore = "真库语料探针：需要本机 E3D 样例项目；cargo test -- --ignored"]
    fn probe_extgeo_chain_continuity() {
        let dirs = corpus_dirs();
        let Some(set) = corpus_set(&dirs) else {
            eprintln!("schema/语料目录不在，跳过");
            return;
        };
        let cable_hash = e3d_attlib::db1_hash("CABLE");
        let desi_hash = e3d_attlib::db1_hash("DESI");

        let mut cables = 0usize;
        let mut extgeo_segments = 0usize;
        let mut extgeo_resolved = 0usize;
        let mut extgeo_unresolved = Vec::new();
        let mut joints = Vec::new();

        for dir in &dirs {
            let Ok(entries) = std::fs::read_dir(dir) else {
                continue;
            };
            for entry in entries.flatten() {
                let path = entry.path();
                if !path.is_file() {
                    continue;
                }
                let Ok(mut engine) = ReadOnlyEngine::open(&path) else {
                    continue;
                };
                if engine.descriptor().db_type_hash != desi_hash {
                    continue;
                }
                let Ok(scan) = engine.scan_elements(ScanTier::Header) else {
                    continue;
                };
                for scanned in scan.flatten() {
                    if scanned.noun_hash != cable_hash {
                        continue;
                    }
                    let cable = set.element(scanned.refno);
                    let Ok(members) = member_world_endpoints(&cable) else {
                        continue;
                    };
                    cables += 1;
                    let file = path.file_name().unwrap_or_default().to_string_lossy();
                    // 只查 EXTGEO 段的两端：STAREF 端应接上「前一个成员的末点」，
                    // ENDREF 端应被「后一个成员的首点」接上。连续 = 附着件锚定语义对。
                    for (index, (mref, noun, pts)) in members.iter().enumerate() {
                        if noun != "EXTGEO" {
                            continue;
                        }
                        extgeo_segments += 1;
                        if pts.len() != 2 {
                            extgeo_unresolved.push(format!(
                                "{file} cable {} extgeo {mref} 端点没解出",
                                cable.refno()
                            ));
                            continue;
                        }
                        extgeo_resolved += 1;
                        let before = index.checked_sub(1).and_then(|i| {
                            members[i]
                                .2
                                .last()
                                .map(|p| (members[i].0, members[i].1.clone(), *p))
                        });
                        let after = members
                            .get(index + 1)
                            .and_then(|(r, n, p)| p.first().map(|p| (*r, n.clone(), *p)));
                        let mut line = format!(
                            "{file} cable {} extgeo {mref}: start=({:.0},{:.0},{:.0}) end=({:.0},{:.0},{:.0})",
                            cable.refno(),
                            pts[0].x,
                            pts[0].y,
                            pts[0].z,
                            pts[1].x,
                            pts[1].y,
                            pts[1].z,
                        );
                        // 同一条 CWBRAN 时：两锚点站号差 = 途经的桥架 POINTR 数，
                        // 判「直弦 vs 顺路径」的关键证据。
                        let member = set.element(*mref);
                        let anchors = [reference(&member, "STAREF"), reference(&member, "ENDREF")];
                        if let [Ok(Some(s)), Ok(Some(e))] = anchors {
                            if let (Ok(Some(a)), Ok(Some(b))) =
                                (attachment_anchor(&s), attachment_anchor(&e))
                            {
                                let same = a.cwbran.refno() == b.cwbran.refno();
                                let mid = a.station.abs_diff(b.station).saturating_sub(1);
                                line += &format!(
                                    " | {}:{}→{}:{} via={} mid={}",
                                    s.element_type().unwrap_or_default(),
                                    a.station,
                                    e.element_type().unwrap_or_default(),
                                    b.station,
                                    if same { "同一CWBRAN" } else { "跨CWBRAN" },
                                    mid,
                                );
                            }
                        }
                        if let Some((r, n, p)) = before {
                            line += &format!(" | prev {r}({n}) gap={:.1}mm", p.distance(pts[0]));
                        }
                        if let Some((r, n, p)) = after {
                            line += &format!(" | next {r}({n}) gap={:.1}mm", p.distance(pts[1]));
                        }
                        joints.push(line);
                    }
                }
            }
        }
        eprintln!("cables={cables} extgeo={extgeo_segments} resolved={extgeo_resolved}");
        for line in &joints {
            eprintln!("  {line}");
        }
        for line in &extgeo_unresolved {
            eprintln!("  UNRESOLVED {line}");
        }
    }
}

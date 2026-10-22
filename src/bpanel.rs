//! 折弯板 `BPANEL`：压型墙板、屋面板、收边板（aps250110 `/AREA02_WS_CLADDING\*` 40 只、
//! `/AREA02_WS_ROOFING` 12 只）。
//!
//! 截面中线是名下 `RPATH` 的 `POINTR` 点串（板的局部 XY 平面），拐角按 `FRAD` 倒圆（与扶手
//! 同一套段表，[`crate::handrail::path_layout`]）；板厚 `WTHICK` 以中线对称（`WJUSL = CENT`，语料
//! 里全是），沿 `RPATH.YDIR`（全是 +Z）从 0 拉到板宽 `WIDT`。直段是一块矩形板，拐角是一节矩形截面
//! 圆环（内径 `FRAD − WTHICK/2`、外径 `FRAD + WTHICK/2`）。
//!
//! E3D TTY RVM（`aps250110_lv6_2013286654_1843.rvm`）有两种写法，弯的分段也不同：
//!
//! - 名下没有负体：每段一条 FacetGroup——N 个点 = N − 1 块直板 + N − 2 节弯，逐只 2N − 3 条
//!   （93 / 89 / 73 / 77 / 57 / 3 / 291…）。弯按曲面图元口径分段（[`part_rev_segments`]，至少 2 段）：
//!   R3.85 的墙板弯 2 段、R25 的屋面弯 2 / 3 段。
//! - 名下有开孔负体（NCYL / NPYR / NBOX）：整只扣完写一条，弯按布尔操作数的粗口径（10 mm 弦高的整圈
//!   段数按扫角缩，至少 1 段）。屋面板 9 / 10 / 11 的 R25 弯（外径 27.5、扫角 30.7°）只剩一条弦，
//!   外侧两端点相距 2·27.5·sin(θ/2) = 14.55、内侧 2·22.5·sin(θ/2) = 11.9，板的最低点就在这些弦上。

use e3d_io::db_element::DbElement;
use glam::{DMat3, DMat4, DVec3};
use manifold_csg::Manifold;

use crate::discretise::{PROFILE_CHORD_TOL_MM, circle_segments_uncapped};
use crate::elmodl::ModelError;
use crate::handrail::{PathSegment, path_layout};
use crate::solid::{SolidError, cut_by_plane, extrude_rings, part_rev_segments, placed};

/// [`bent_panel_local_solid`] 的结果，都在板的局部系。
pub struct BentPanel {
    /// 整只板（各段之并）；开孔负体由调用方扣。
    pub solid: Manifold,
    /// 各段：直板与拐角弧，按路径顺序。
    pub pieces: Vec<Manifold>,
}

/// `boolean_operand`：名下有负体要扣——弯走布尔操作数的粗分段（见模块文档）。
pub fn bent_panel_local_solid(
    el: &DbElement,
    boolean_operand: bool,
    notes: &mut Vec<String>,
) -> Result<BentPanel, ModelError> {
    let refno = el.refno().to_string();
    let positive = |name: &'static str| -> Result<f64, ModelError> {
        let value = crate::rpath::number_or_zero(el, name)?;
        if value > 0.0 {
            Ok(value)
        } else {
            Err(ModelError::MissingAttribute {
                refno: refno.clone(),
                noun: "BPANEL".into(),
                name,
                what: "BPANEL section",
            })
        }
    };
    let width = positive("WIDT")?;
    let thickness = positive("WTHICK")?;
    let justification = crate::rpath::word(el, "WJUSL")?;
    if justification.as_deref().is_some_and(|word| word != "CENT") {
        return Err(ModelError::UnresolvedAttribute {
            refno,
            noun: "BPANEL".into(),
            name: "WJUSL",
            what: "BPANEL section",
            detail: format!("{justification:?}：只取证过 CENT"),
        });
    }
    let rpath = crate::rpath::first_rpath(el)?.ok_or_else(|| ModelError::MissingAttribute {
        refno: refno.clone(),
        noun: "BPANEL".into(),
        name: "RPATH",
        what: "BPANEL path",
    })?;
    let up = crate::rpath::vector(&rpath, "YDIR")?
        .and_then(DVec3::try_normalize)
        .unwrap_or(DVec3::Z);
    let mut points = Vec::new();
    for member in rpath.members()? {
        let noun = member.element_type()?;
        if noun != "POINTR" {
            return Err(ModelError::UnresolvedAttribute {
                refno,
                noun: "BPANEL".into(),
                name: "RPATH",
                what: "BPANEL path",
                detail: format!("成员 {} 是 {noun}：只认 POINTR", member.refno()),
            });
        }
        points.push((
            crate::rpath::vector(&member, "POS")?.unwrap_or(DVec3::ZERO),
            crate::rpath::number_or_zero(&member, "FRAD")?,
        ));
    }
    let closed = crate::rpath::flag(el, "CLOSEF")?;
    let layout = path_layout(&points, closed, notes);
    if layout.segments.is_empty() {
        return Err(ModelError::Solid(SolidError::Degenerate {
            what: "BPANEL path",
            detail: format!("{} 个 POINTR 排不出一段", points.len()),
        }));
    }
    let pieces = layout
        .segments
        .iter()
        .map(|(_, segment)| piece(segment, up, thickness, width, boolean_operand))
        .collect::<Result<Vec<_>, _>>()?;
    let solid = Manifold::batch_union(&pieces);
    Ok(BentPanel { solid, pieces })
}

/// 一段：直段是厚 `thickness`、宽 `width` 的矩形板（`FRAD = 0` 的拐角先加长再按角平分面切），
/// 弧段是一节矩形截面圆环。两者都从中线沿 `up` 拉到 `width`。
fn piece(
    segment: &PathSegment,
    up: DVec3,
    thickness: f64,
    width: f64,
    boolean_operand: bool,
) -> Result<Manifold, ModelError> {
    let half = thickness / 2.0;
    match segment {
        PathSegment::Line {
            start,
            end,
            mitre_start,
            mitre_end,
        } => {
            let d = (*end - *start).normalize();
            let side = up
                .cross(d)
                .try_normalize()
                .ok_or_else(|| SolidError::Degenerate {
                    what: "BPANEL straight",
                    detail: format!("段方向 {d} 与 YDIR {up} 平行"),
                })?;
            // 斜接面把板端切斜，外侧探出 half·tan(θ/2)；先按这个量加长。法向 = unit(d_in + d_out)，
            // 它与 d 的夹角是 θ/2。
            let extension = |normal: &Option<DVec3>| {
                normal.map_or(0.0, |n| {
                    let cos_half = n.dot(d).abs().clamp(0.0, 1.0);
                    let sin_half = (1.0 - cos_half * cos_half).sqrt();
                    if cos_half > 1e-9 {
                        (half * sin_half / cos_half).min(thickness * 20.0)
                    } else {
                        0.0
                    }
                })
            };
            let a = *start - d * extension(mitre_start);
            let b = *end + d * extension(mitre_end);
            let corners: Vec<[f64; 3]> = [
                a - side * half,
                a + side * half,
                b - side * half,
                b + side * half,
            ]
            .into_iter()
            .flat_map(|p| [p, p + up * width])
            .map(|point| point.to_array())
            .collect();
            let mut solid = Manifold::hull_pts(&corners);
            if let Some(normal) = mitre_start {
                solid = cut_by_plane(&solid, *start, *normal);
            }
            if let Some(normal) = mitre_end {
                solid = cut_by_plane(&solid, *end, -*normal);
            }
            if solid.is_empty() || solid.num_tri() == 0 {
                return Err(ModelError::Solid(SolidError::EmptyManifold {
                    what: "BPANEL straight",
                }));
            }
            Ok(solid)
        }
        PathSegment::Arc {
            centre,
            radius,
            start_dir,
            axis,
            sweep,
        } => {
            let (inner, outer) = ((radius - half).max(0.0), radius + half);
            let sweep_deg = sweep.to_degrees();
            let segments = if boolean_operand {
                let full = f64::from(circle_segments_uncapped(outer, PROFILE_CHORD_TOL_MM));
                ((full * sweep_deg / 360.0).ceil() as usize).max(1)
            } else {
                part_rev_segments(outer, sweep_deg).max(2) as usize
            };
            // 在 YDIR 的法平面里从 `start_dir` 转过 `sweep`；绕 `axis` 转，换到绕 `up` 就是带符号的角。
            let turn = if axis.dot(up) < 0.0 { -sweep } else { *sweep };
            let at = |r: f64, k: usize| {
                let angle = turn * k as f64 / segments as f64;
                [r * angle.cos(), r * angle.sin()]
            };
            let mut ring: Vec<[f64; 2]> = (0..=segments).map(|k| at(outer, k)).collect();
            if inner > 0.0 {
                ring.extend((0..=segments).rev().map(|k| at(inner, k)));
            } else {
                ring.push([0.0, 0.0]);
            }
            let body = extrude_rings(vec![ring], width)?;
            let frame = DMat4::from_translation(*centre)
                * DMat4::from_mat3(DMat3::from_cols(*start_dir, up.cross(*start_dir), up));
            Ok(placed(&body, frame))
        }
    }
}

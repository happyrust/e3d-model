//! 布局式梯段（`STRFLT`，`INSTOP` 以 `LAYOUT` 开头）。
//!
//! 详图式梯段（`INSTOP` 以 `DETAIL` 开头，aps250110 `/B01_LEVEL_03_ACCESS/SF_001` 那些）的几何在 `TREADS` / `STRSTR` 成员上，
//! E3D 在 STRFLT 这一层不画；布局式梯段没有那些成员，E3D 给 STRFLT 自己写**一条** `FacetGroup`（aps250209 的 18 只，
//! TTY 导出 `verification/civil-tty-20261021/aps250209-w7t/`）。照那些面片还原出来的画法（局部系 = STRFLT 自己的，
//! 路径 = 它的 `RPATH` 点串，平台段 + 斜段）：
//!
//! 1. **斜梁**：两条宽 50 的带，贴在踏步宽 `TWIDTH` 之外（中心在 ±(TWIDTH/2 + 25)，正是扶手 HANDRA 的 `POS` x），上边是路径、
//!    沿路径的法向往下 `FTHICK`（斜段上是垂直于坡的深度：`/A01-AREA05-BASE-FL-STAIRS` STRFLT 1 顶端的竖直端面高 329 =
//!    262.75 / cosθ）。拐角按过拐点的角平分面斜接（平台带的底边一直画到与斜段带底边相交：STRFLT 1 底部平台 y 0..390.37，
//!    STRFLT 2 顶部平台的斜接边 B→C 在角平分面上）；开口端按 `RPATH` 的 `DRNS` / `DRNE` 切（这些梯段的 `DRNS = +X`
//!    与路径垂直 → 平头，`DRNE = +Y` → 顶端竖切），没有端向时是垂直于路径的平头。
//! 2. **踏步**：斜段上 `n = round(升高 / RISE)` 级，踏步进深 `g = 水平投影 / n`；第 k 级（k = 0..n）是一只盒：宽 `TWIDTH`，
//!    水平从第 k 条立板到第 k+1 条立板再往前探 `TREADO`（踢边），顶在 `z_s + k·RISE`、厚 `g / 10`（三条证据 26.28 / 24.37 / 24.91
//!    对 g 262.75 / 243.7 / 249.09；这三条里 `FTHICK` 恰好都等于 g，坐实是哪个还要一条 `FTHICK ≠ g` 的样本）。第 n 级是上层平台，
//!    不画；第 0 级只在斜段前面有平台段时画（STRFLT 1 的 y 302.45..590.2 有、从斜段起步的 STRFLT 2 没有）。
//! 3. **平台板**：水平段上宽 `TWIDTH`、厚 `g / 10` 的带，顶在路径上，拐角同样按角平分面斜接（STRFLT 2 顶部平台的板底收到
//!    −2688.86 = 拐点 − t·tan(θ/2)）。
//!
//! 没接的（语料里全是 0）：`ORISE` / `OGOING` / `UEXTNS` / `LEXTNS` / `UCUTDI` / `LCUTDI` / `STRJUS`，非零时记 note。
//! 踏步宽度外的斜梁宽 50 是常数（四种配置的 18 只都是），它和踏步厚都不在 STRFLT 的属性里。

use glam::DVec3;
use manifold_csg::Manifold;

use e3d_io::db_element::DbElement;

use crate::elmodl::ModelError;
use crate::handrail::{
    BandSection, OpenEnd, PathSegment, apply_end_directions, band_segment_solid, build_path,
    first_rpath, path_points,
};
use crate::solid::SolidError;

/// 斜梁宽（踏步宽度之外）。
pub const STRINGER_WIDTH: f64 = 50.0;
/// 踏步 / 平台板厚 = 进深的这个分数。
const TREAD_THICKNESS_OF_GOING: f64 = 0.1;

/// [`stair_flight_local_solid`] 的三种结局。
pub enum StairFlightBuild {
    /// 布局式几何，已在 STRFLT 局部系。
    Solid(Manifold),
    /// 详图式梯段：E3D 在 STRFLT 这一层不画，几何在 TREADS / STRSTR 成员上。
    Detailed(String),
    /// 布局式但没有可画的东西。
    NoGeometry(String),
}

/// STRFLT 上读到的布局参数。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlightParams {
    /// `RISE`：每级升高。
    pub rise: f64,
    /// `TWIDTH`：踏步宽（两条斜梁之间）。
    pub width: f64,
    /// `TREADO`：踏步踢边越过下一条立板的探出。
    pub nosing: f64,
    /// `FTHICK`：斜梁沿路径法向的深度。
    pub depth: f64,
}

/// 建出来的各部分计数，记 note 用。
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct FlightStats {
    pub stringers: usize,
    pub treads: usize,
    pub landings: usize,
    pub tread_thickness: Option<f64>,
}

/// 从 STRFLT 元素建布局式几何（局部系，属主世界系由调用方右乘）。
pub fn stair_flight_local_solid(
    el: &DbElement,
    notes: &mut Vec<String>,
) -> Result<StairFlightBuild, ModelError> {
    let instop = el.get_string("INSTOP")?.unwrap_or_default();
    if !instop.trim().to_ascii_uppercase().starts_with("LAYOUT") {
        return Ok(StairFlightBuild::Detailed(format!(
            "INSTOP = {instop:?}：不是布局式梯段，几何在 TREADS / STRSTR 成员上，E3D 在 STRFLT 这一层不画"
        )));
    }
    let Some(rpath) = first_rpath(el)? else {
        return Ok(StairFlightBuild::NoGeometry(
            "没有 RPATH 成员：梯段没有路径，E3D 同样不出几何".to_string(),
        ));
    };
    let raw = path_points(&rpath)?;
    notes.extend(raw.unwired.iter().map(|(_, why)| format!("{why}，已跳过")));
    let points: Vec<DVec3> = raw.points.iter().map(|(_, pos, _)| *pos).collect();
    let real =
        |name: &'static str| -> Result<f64, ModelError> { Ok(el.get_double(name)?.unwrap_or(0.0)) };
    let params = FlightParams {
        rise: real("RISE")?,
        width: real("TWIDTH")?,
        nosing: real("TREADO")?.max(0.0),
        depth: real("FTHICK")?,
    };
    for name in ["ORISE", "OGOING", "UEXTNS", "LEXTNS"] {
        let value = real(name)?;
        if value.abs() > 1e-9 {
            notes.push(format!(
                "{name} = {value}：梯段的这个偏置 / 延伸本期未接，按 0 画"
            ));
        }
    }
    match layout_flight_solid(&points, raw.drns, raw.drne, &params, notes)? {
        Some((solid, stats)) => {
            notes.push(format!(
                "布局式梯段：{} 个路径点，{} 节斜梁（宽 {STRINGER_WIDTH}、深 {}），{} 级踏步，{} 块平台板（板厚 {}）",
                points.len(),
                stats.stringers,
                params.depth,
                stats.treads,
                stats.landings,
                stats.tread_thickness.map_or("-".to_string(), |t| format!("{t:.2}")),
            ));
            Ok(StairFlightBuild::Solid(solid))
        }
        None => Ok(StairFlightBuild::NoGeometry(
            "路径、宽度或深度不足，一块都排不出来（见 notes）".to_string(),
        )),
    }
}

/// 布局式梯段的几何本身：`points` 是 RPATH 的点串（局部系），`drns` / `drne` 是它的端向（零向量 = 未设）。
/// 返回 `None` 表示没有一块可画（点不足两个、`TWIDTH` 不是正数…），原因在 `notes` 里。
pub fn layout_flight_solid(
    points: &[DVec3],
    drns: DVec3,
    drne: DVec3,
    params: &FlightParams,
    notes: &mut Vec<String>,
) -> Result<Option<(Manifold, FlightStats)>, ModelError> {
    if points.len() < 2 {
        notes.push(format!("RPATH 只有 {} 个 POINTR，排不出一段", points.len()));
        return Ok(None);
    }
    if !(params.width > 0.0) {
        notes.push(format!(
            "TWIDTH = {}：没有踏步宽度，排不出斜梁与踏步",
            params.width
        ));
        return Ok(None);
    }
    // 梯段的每一样都按没有倒角的路径排（拐角斜接）。
    let unfilleted: Vec<(DVec3, f64)> = points.iter().map(|point| (*point, 0.0)).collect();
    let mut segments = build_path(&unfilleted, false, notes);
    if segments.is_empty() {
        notes.push("路径点全部重合，没有一段有长度".to_string());
        return Ok(None);
    }
    apply_end_directions(&mut segments, false, drns, drne, notes);

    let going_of = |start: DVec3, end: DVec3| -> Option<(usize, f64)> {
        if !(params.rise > 0.0) {
            return None;
        }
        let run = DVec3::new(end.x - start.x, end.y - start.y, 0.0).length();
        let steps = ((end.z - start.z).abs() / params.rise).round().max(1.0) as usize;
        (run > 1e-6).then(|| (steps, run / steps as f64))
    };
    let is_slope = |segment: &PathSegment| match segment {
        PathSegment::Line { start, end, .. } => (end.z - start.z).abs() > 1e-6,
        PathSegment::Arc { .. } => false,
    };
    // 第一段斜段的进深定整只梯段的踏步 / 平台板厚。
    let tread_thickness = segments
        .iter()
        .find(|segment| is_slope(segment))
        .and_then(|segment| match segment {
            PathSegment::Line { start, end, .. } => going_of(*start, *end),
            PathSegment::Arc { .. } => None,
        })
        .map(|(_, going)| going * TREAD_THICKNESS_OF_GOING);

    let mut parts: Vec<Manifold> = Vec::new();
    let mut stats = FlightStats {
        tread_thickness,
        ..FlightStats::default()
    };
    if params.depth > 0.0 {
        for segment in &segments {
            for side in [-1.0, 1.0] {
                let band = BandSection {
                    across: side * (params.width / 2.0 + STRINGER_WIDTH / 2.0),
                    half_thickness: STRINGER_WIDTH / 2.0,
                    low: -params.depth,
                    high: 0.0,
                    open_end: OpenEnd::Perpendicular,
                };
                parts.push(band_segment_solid(segment, &band, "stair flight stringer")?);
                stats.stringers += 1;
            }
        }
    } else {
        notes.push(format!("FTHICK = {}：斜梁没有深度，不画斜梁", params.depth));
    }
    match tread_thickness {
        Some(thickness) => {
            for (index, segment) in segments.iter().enumerate() {
                let PathSegment::Line { start, end, .. } = segment else {
                    continue;
                };
                if !is_slope(segment) {
                    let band = BandSection {
                        across: 0.0,
                        half_thickness: params.width / 2.0,
                        low: -thickness,
                        high: 0.0,
                        open_end: OpenEnd::Perpendicular,
                    };
                    parts.push(band_segment_solid(segment, &band, "stair flight landing")?);
                    stats.landings += 1;
                    continue;
                }
                let Some((steps, going)) = going_of(*start, *end) else {
                    continue;
                };
                // 按上行方向排：下行的段把两端对调，踏步的踢边仍朝上层那一侧。第 0 级只在前面有别的段时画。
                let (low, high) = if end.z >= start.z {
                    (*start, *end)
                } else {
                    (*end, *start)
                };
                let horizontal = DVec3::new(high.x - low.x, high.y - low.y, 0.0).normalize();
                let across = horizontal.cross(DVec3::Z) * (params.width / 2.0);
                let first = if index > 0 { 0 } else { 1 };
                for k in first..steps {
                    let from = low + horizontal * (going * k as f64);
                    let to = low + horizontal * (going * (k + 1) as f64 + params.nosing);
                    let top = low.z + params.rise * k as f64;
                    let corners = [
                        DVec3::new(from.x, from.y, top - thickness) - across,
                        DVec3::new(from.x, from.y, top - thickness) + across,
                        DVec3::new(from.x, from.y, top) - across,
                        DVec3::new(from.x, from.y, top) + across,
                        DVec3::new(to.x, to.y, top - thickness) - across,
                        DVec3::new(to.x, to.y, top - thickness) + across,
                        DVec3::new(to.x, to.y, top) - across,
                        DVec3::new(to.x, to.y, top) + across,
                    ];
                    let tread = Manifold::hull_pts(&corners.map(|point| point.to_array()));
                    if tread.is_empty() || tread.num_tri() == 0 {
                        return Err(ModelError::Solid(SolidError::EmptyManifold {
                            what: "stair flight tread",
                        }));
                    }
                    parts.push(tread);
                    stats.treads += 1;
                }
            }
        }
        None => notes.push(format!(
            "RISE = {} 或没有斜段：算不出踏步进深，不画踏步与平台板",
            params.rise
        )),
    }
    let solid = match parts.as_slice() {
        [] => return Ok(None),
        [only] => only.clone(),
        many => Manifold::compose(many),
    };
    if solid.is_empty() || solid.num_tri() == 0 {
        return Err(ModelError::Solid(SolidError::EmptyManifold {
            what: "layout stair flight",
        }));
    }
    Ok(Some((solid, stats)))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn aabb(solid: &Manifold) -> ([f64; 3], [f64; 3]) {
        let bb = solid.bounding_box().expect("solid");
        (bb.min(), bb.max())
    }

    fn close(got: [f64; 3], want: [f64; 3]) -> bool {
        got.iter().zip(want).all(|(g, w)| (g - w).abs() < 0.05)
    }

    /// aps250209 `/A01-AREA05-BASE-FL-STAIRS` STRFLT 1：平台 302.45 + 10 级 198 × 262.75，`DRNS = +X`（垂直于路径，平头）、
    /// `DRNE = +Y`（顶端竖切）。E3D 面片的盒是 [−450, 0, −262.75] .. [450, 2954.95, 1980]：宽 900 = 800 + 2 × 50，长到最后一级
    /// 踏步的踢边 302.45 + 10 × 262.75 + 25，下到平台段斜梁的底，上到路径顶点。第 0 级踏步在（前面是平台）：9 + 1 级。
    #[test]
    fn a_flight_with_a_bottom_landing_matches_the_e3d_box() {
        let points = [
            DVec3::new(0.0, 0.0, 0.0),
            DVec3::new(0.0, 302.45, 0.0),
            DVec3::new(0.0, 2929.95, 1980.0),
        ];
        let params = FlightParams {
            rise: 198.0,
            width: 800.0,
            nosing: 25.0,
            depth: 262.75,
        };
        let mut notes = Vec::new();
        let (solid, stats) = layout_flight_solid(&points, DVec3::X, DVec3::Y, &params, &mut notes)
            .unwrap()
            .expect("geometry");
        assert_eq!(
            (stats.stringers, stats.treads, stats.landings),
            (4, 10, 1),
            "{stats:?}"
        );
        assert!((stats.tread_thickness.unwrap() - 26.275).abs() < 1e-9);
        let (lo, hi) = aabb(&solid);
        assert!(close(lo, [-450.0, 0.0, -262.75]), "{lo:?}");
        assert!(close(hi, [450.0, 2954.95, 1980.0]), "{hi:?}");
        assert!(notes.iter().any(|note| note.contains("DRNE")), "{notes:?}");
    }

    /// aps250209 `/A01-AREA05-BASE-FL-STAIRS` STRFLT 2：从斜段起步（11 级 183.64 × 243.7）再接 250 的顶部平台。E3D 的盒是
    /// [−450, 0, −194.63] .. [450, 2930.7, 2020.04]：底端是垂直于坡的平头（最低点 −262.75·cosθ = −194.63），没有第 0 级踏步，
    /// 第 11 级是平台板。
    #[test]
    fn a_flight_starting_on_the_slope_has_no_bottom_tread_and_a_perpendicular_foot() {
        let points = [
            DVec3::new(0.0, 0.0, 0.0),
            DVec3::new(0.0, 2680.7, 2020.04),
            DVec3::new(0.0, 2930.7, 2020.04),
        ];
        let params = FlightParams {
            rise: 183.64,
            width: 800.0,
            nosing: 25.0,
            depth: 243.7,
        };
        let mut notes = Vec::new();
        let (solid, stats) = layout_flight_solid(&points, DVec3::X, DVec3::Y, &params, &mut notes)
            .unwrap()
            .expect("geometry");
        assert_eq!(
            (stats.stringers, stats.treads, stats.landings),
            (4, 10, 1),
            "{stats:?}"
        );
        let (lo, hi) = aabb(&solid);
        assert!(close(lo, [-450.0, 0.0, -194.63]), "{lo:?}");
        assert!(close(hi, [450.0, 2930.7, 2020.04]), "{hi:?}");
    }
}

//! 布局式扶手（`HANDRA`，recheck-4 §五 W4-1 施工图·续 六）。
//!
//! 对标 Core3D `ASL_MDR_HandrailVisualisationManager::getCSGTree@0x109F96A0` →
//! `ASL_MDR_CSGHandRail`（ctor `0x109F52B0`，基类 **`DB_RoutingBranch`**）→ 引擎 `sub_109F5560`。
//! 扶手在 Core3D 里是一条路由支线：`RPATH` 的 `POINTR` 点串变成一串 `DB_Curve`
//! （`createCurvesFromPath@0x105B3670`），剖面沿着它扫（`do_solid_segments@0x105B57E0`），
//! 拐角切斜接面（`setMitrePlanes@0x105BC170`）。
//!
//! 语料里有两种扶手，闸在壳的第 45–50 行：`getAtt(ATT_LAYOUT)` 为假 HANDRA 自己什么都不画，
//! 几何在 `RAIL` / `KICKPL` / `HRPOST` 成员上（详图式，`aps250110` 的 11 件）。`LAYOUT`
//! 不在 attlib 里，是 core.dll 算的伪属性；本机两库对得上的判据是「有没有
//! `RAILSE` / `POSTSE` / `HRKPSE` / `HRPNSE` 成员」（[`DETAILED_MEMBER_NOUNS`]），**待核**。
//!
//! 布局式（`aps250209` 的 81 件）画三样东西：
//!
//! 1. **栏杆**：一个**固定 Ø40 的圆**（`sub_109F65B0`：四段 bulge = tan22.5° 的弧过
//!    (±20,0)(0,±20)，常量在 `0x10B801C0..F0`，不读任何属性）沿路径扫，`RAILHE` 里每个高度
//!    平移 (0,0,h) 各放一份。直段是圆柱，拐角在 `FRAD > 0` 时是一节圆环（`THRU` 弧），
//!    `FRAD = 0` 时两根圆柱各切一张斜接面（角平分面）。
//! 2. **立柱**：`sub_109F60C0`，Ø30 圆柱，高 `maxRAILHE − 20 − POSTOF`（柱顶停在顶栏杆轴线
//!    下 20，即 Ø40 栏杆的下缘），逐直段按 `TANGOF` / `EPOSTO` 退距、`POSTSP` 间距均分，
//!    共线接头不重复放柱，弧段不放柱。
//! 3. **门洞**（`HRGATE` 成员）与 **踢脚板**（`SHOWKP`）——本机语料里门一件都没有，踢脚板
//!    有但它的默认剖面（`ASL_CSGDefaultKickplate` 的 vtbl+52）还没拉出来，**都没接**，
//!    件数进 notes，不静默。
//!
//! **对拍**：`test_data/rvm/` 里没有任何一份含 HANDRA 的 E3D 导出，这个模块的几何
//! 到 2026-09-06 为止只有自证（单测里的解析几何），没有与 E3D 逐件对过。

use std::f64::consts::PI;

use glam::{DMat3, DMat4, DQuat, DVec3};
use manifold_csg::Manifold;

use e3d_io::db_element::DbElement;

use crate::elmodl::ModelError;
use crate::section::{
    SectionChain, SectionCutPlane, SectionCuts, SpinePath, SpineSegment, cut_plane_normal,
    evaluate_section_profile, placement_between, section_catalogue, section_local_solid,
    spine_swept_solid,
};
use crate::solid::{SolidError, circ_torus_solid, cut_by_plane, cylinder_segments, placed};

/// 栏杆管半径：`sub_109F65B0` 里写死的 Ø40 圆。
pub const RAIL_RADIUS: f64 = 20.0;
/// 立柱半径：`sub_109F60C0` 里 `gm_CreateCylinder(15.0, …)`。
pub const POST_RADIUS: f64 = 15.0;
/// 柱顶到顶栏杆轴线的间隙：`H = maxRAILHE − 20 − POSTOF`。
pub const POST_TOP_CLEARANCE: f64 = 20.0;
/// 段首 / 段尾余量不足这个数就不放端柱（`sub_109F60C0` 两处 `< 60.0`）。
pub const POST_MIN_END_RUN: f64 = 60.0;
/// 相邻直段方向差在这个范围内算共线（`D3_Vector::equals(…, 0.01, 0.01)`），接头处退距归零。
const COLLINEAR_TOL: f64 = 0.01;
/// 拐角小于这个角度（弧度）当直线接，不切斜接面也不放弧。
const STRAIGHT_JOINT_RAD: f64 = 1e-6;
/// 拐角大于这个角度不切斜接面——`setMitrePlanes` 对近乎折返的接头写「suppress the mitre」，
/// 而斜接面的延伸量 `r·tan(θ/2)` 在 180° 处发散。
const MAX_MITRE_RAD: f64 = 170.0 * PI / 180.0;

/// 有这些成员就是详图式扶手：几何在成员上，HANDRA 自己不画（`LAYOUT` 伪属性的替身，待核）。
pub const DETAILED_MEMBER_NOUNS: [&str; 4] = ["RAILSE", "POSTSE", "HRKPSE", "HRPNSE"];

/// [`handrail_local_solid`] 的三种结局。
pub enum HandrailBuild {
    /// 布局式几何，已在 HANDRA 局部系。
    Solid(Manifold),
    /// 详图式扶手：E3D 在 HANDRA 这一层同样什么都不画，几何在 RAIL / KICKPL / HRPOST 上。
    Detailed(String),
    /// 布局式但没有可画的东西（RAILHE 为空、路径不足两点…），E3D 同样出不了几何。
    NoGeometry(String),
}

/// HANDRA 上读到的布局参数（`sub_109F7150` 那张属性→字段表）。
#[derive(Debug, Clone, PartialEq)]
pub struct HandrailParams {
    /// `RAILHE`：每个高度一根栏杆。
    pub rail_heights: Vec<f64>,
    /// `POSTSP`：立柱间距。≤ 0 不放柱。
    pub post_spacing: f64,
    /// `TANGOF`：直段两端的立柱退距。< 0 不放柱。
    pub tangent_offset: f64,
    /// `EPOSTO`：开口支线两端用它替代 `TANGOF`。
    pub end_post_offset: f64,
    /// `POSTOF`：立柱竖向起点偏移。
    pub post_offset: f64,
    /// `CLOSEF`：路径闭合。
    pub closed: bool,
    /// `SHOWKP`：踢脚板（未接，只记账）。
    pub show_kickplate: bool,
}

/// 路径上的一段：直段或拐角弧。顺序与 Core3D 的 `DB_Curve` 表一致。
#[derive(Debug, Clone, PartialEq)]
pub enum PathSegment {
    Line {
        start: DVec3,
        end: DVec3,
        /// 段首要不要切斜接面：`Some(平面法向)`。法向 = `unit(d_in + d_out)`（两条轴线夹角的
        /// 角平分面，过拐点）；出边保留 `(p − 拐点)·法向 ≥ 0` 那一侧，入边保留另一侧。
        mitre_start: Option<DVec3>,
        mitre_end: Option<DVec3>,
    },
    Arc {
        centre: DVec3,
        radius: f64,
        /// 弧起点相对圆心的单位向量（圆环局部 +X）。
        start_dir: DVec3,
        /// 扫掠轴（圆环局部 +Z），起点绕它逆时针转 `sweep` 到终点。
        axis: DVec3,
        /// 扫角，弧度，(0, π)。
        sweep: f64,
    },
}

impl PathSegment {
    fn direction(&self) -> Option<DVec3> {
        match self {
            PathSegment::Line { start, end, .. } => (*end - *start).try_normalize(),
            PathSegment::Arc { .. } => None,
        }
    }
}

/// 从 HANDRA 元素建布局式几何（局部系，属主世界系由调用方右乘）。
pub fn handrail_local_solid(
    el: &DbElement,
    notes: &mut Vec<String>,
) -> Result<HandrailBuild, ModelError> {
    let refno = el.refno().to_string();
    let noun = el.element_type()?;

    let mut path_owner = None;
    let mut detailed = Vec::new();
    let mut gates = 0usize;
    for member in el.members()? {
        let member_noun = member.element_type()?;
        if DETAILED_MEMBER_NOUNS.contains(&member_noun.as_str()) {
            detailed.push(member_noun);
        } else if member_noun == "RPATH" && path_owner.is_none() {
            path_owner = Some(member);
        } else if member_noun == "HRGATE" {
            gates += 1;
        }
    }
    if !detailed.is_empty() {
        detailed.sort();
        detailed.dedup();
        return Ok(HandrailBuild::Detailed(format!(
            "详图式扶手（成员 {}）：E3D 在 HANDRA 这一层不画，几何在 RAIL / KICKPL / HRPOST 上",
            detailed.join("/")
        )));
    }
    let Some(rpath) = path_owner else {
        return Ok(HandrailBuild::NoGeometry(
            "没有 RPATH 成员：路由支线没有路径，E3D 同样不出几何".to_string(),
        ));
    };

    let params = read_params(el, &refno, &noun)?;
    if params.rail_heights.is_empty() {
        // `sub_109F7F10`：`RAILHE` 为空连剖面都不造，直接走不到扫掠。
        return Ok(HandrailBuild::NoGeometry(
            "RAILHE 为空：没有一根栏杆要画，E3D 同样不出几何".to_string(),
        ));
    }

    let raw = path_points(&rpath)?;
    notes.extend(raw.unwired.iter().map(|(_, why)| format!("{why}，已跳过")));
    let points: Vec<(DVec3, f64)> = raw
        .points
        .into_iter()
        .map(|(_, pos, fillet)| (pos, fillet))
        .collect();
    if points.len() < 2 {
        return Ok(HandrailBuild::NoGeometry(format!(
            "RPATH 只有 {} 个 POINTR，扫不出一段，E3D 同样不出几何",
            points.len()
        )));
    }

    let segments = build_path(&points, params.closed, notes);
    if segments.is_empty() {
        return Ok(HandrailBuild::NoGeometry(
            "路径点全部重合，没有一段有长度，E3D 同样不出几何".to_string(),
        ));
    }

    let mut parts = Vec::new();
    let mut rails = 0usize;
    for &height in &params.rail_heights {
        let lift = DMat4::from_translation(DVec3::new(0.0, 0.0, height));
        for segment in &segments {
            let body = rail_segment_solid(segment)?;
            parts.push(placed(&body, lift));
            rails += 1;
        }
    }
    if params.show_kickplate {
        for segment in &segments {
            // The default kickplate is attached at the path elevation, not at
            // either handrail height.
            parts.push(kickplate_segment_solid(segment)?);
        }
    }
    let posts = post_positions(&segments, &params);
    let post_height = params.rail_heights.iter().cloned().fold(0.0, f64::max)
        - POST_TOP_CLEARANCE
        - params.post_offset;
    let mut post_count = 0usize;
    if post_height > f64::EPSILON {
        let post = Manifold::cylinder(
            post_height,
            POST_RADIUS,
            POST_RADIUS,
            cylinder_segments(POST_RADIUS),
            false,
        );
        for foot in &posts {
            let at =
                DMat4::from_translation(DVec3::new(foot.x, foot.y, foot.z + params.post_offset));
            parts.push(placed(&post, at));
            post_count += 1;
        }
    } else if !posts.is_empty() {
        notes.push(format!(
            "立柱高 maxRAILHE − 20 − POSTOF = {post_height:.1} ≤ 0，{} 根立柱一根都不放（E3D 的 gm_CreateCylinder 同样建不出）",
            posts.len()
        ));
    }
    notes.push(format!(
        "布局式扶手：{} 段路径（{} 段弧）× {} 个高度 = {rails} 节栏杆，{post_count} 根立柱",
        segments.len(),
        segments
            .iter()
            .filter(|segment| matches!(segment, PathSegment::Arc { .. }))
            .count(),
        params.rail_heights.len(),
    ));
    if params.show_kickplate {
        notes.push("SHOWKP=true：已生成默认 8 x 110 mm 踢脚板".to_string());
    }
    if gates > 0 {
        notes.push(format!(
            "{gates} 个 HRGATE 成员：门与门洞（ASL_CSGRungLadderGate + 让位盒）本期未接"
        ));
    }

    let solid = match parts.as_slice() {
        [] => {
            return Ok(HandrailBuild::NoGeometry(
                "路径与高度都在，却一节栏杆都没建出来".to_string(),
            ));
        }
        [only] => only.clone(),
        many => Manifold::compose(many),
    };
    if solid.is_empty() || solid.num_tri() == 0 {
        return Err(ModelError::Solid(SolidError::EmptyManifold {
            what: "layout handrail",
        }));
    }
    Ok(HandrailBuild::Solid(solid))
}

fn read_params(el: &DbElement, refno: &str, noun: &str) -> Result<HandrailParams, ModelError> {
    let rail_heights = el
        .get_double_array("RAILHE")?
        .unwrap_or_default()
        .into_iter()
        .filter(|height| height.is_finite())
        .collect();
    let real =
        |name: &'static str| -> Result<f64, ModelError> { Ok(el.get_double(name)?.unwrap_or(0.0)) };
    let flag = |name: &'static str| -> Result<bool, ModelError> {
        Ok(el.get_bool(name)?.unwrap_or(false))
    };
    let _ = (refno, noun);
    Ok(HandrailParams {
        rail_heights,
        post_spacing: real("POSTSP")?,
        tangent_offset: real("TANGOF")?,
        end_post_offset: real("EPOSTO")?,
        post_offset: real("POSTOF")?,
        closed: flag("CLOSEF")?,
        show_kickplate: flag("SHOWKP")?,
    })
}

/// [`path_layout`] 的结果。
#[derive(Debug, Clone, PartialEq)]
pub struct PathLayout {
    /// 去重后还在用的点在入参里的下标：`kept[i]` 是第 i 个点的来源。
    /// 详图式成员的 `STAREF` / `ENDREF` 指的是原始 `POINTR`，要经这张表换成边号。
    pub kept: Vec<usize>,
    /// `(边号, 段)`。边 e 从点 e 走到点 `(e + 1) % kept.len()`；一条边最多出两段
    /// ——直段，以及它尾部那节圆角弧。
    pub segments: Vec<(usize, PathSegment)>,
    /// 边数：开口路径 `kept.len() − 1`，闭合 `kept.len()`。
    pub edge_count: usize,
    /// 去重、点数不足之后这条路径实际闭不闭合。
    pub closed: bool,
}

/// 点串 → 段表（`createCurvesFromPath` 的几何部分）。
///
/// 拐角 `FRAD > 0` 时插一节切于两邻边、半径 `FRAD` 的弧，两邻边各缩 `FRAD·tan(θ/2)`；
/// 缩不下（两端切点交叉）就退回斜接，记一条 note。`FRAD = 0` 的拐角两边各带一张斜接面。
pub fn build_path(
    points: &[(DVec3, f64)],
    closed: bool,
    notes: &mut Vec<String>,
) -> Vec<PathSegment> {
    path_layout(points, closed, notes)
        .segments
        .into_iter()
        .map(|(_, segment)| segment)
        .collect()
}

/// [`build_path`] 加上「哪一段来自哪条边」——详图式成员只扫 `STAREF → ENDREF`
/// 之间那一串边，得按边号切子段表。
pub fn path_layout(points: &[(DVec3, f64)], closed: bool, notes: &mut Vec<String>) -> PathLayout {
    // 去掉重合点：零长段既扫不出体，也会让方向求不出来。
    let mut pts: Vec<(DVec3, f64)> = Vec::with_capacity(points.len());
    let mut kept: Vec<usize> = Vec::with_capacity(points.len());
    for (index, &(p, f)) in points.iter().enumerate() {
        let distinct = match pts.last() {
            Some(&(q, _)) => (p - q).length() > 1e-6,
            None => true,
        };
        if distinct {
            pts.push((p, f));
            kept.push(index);
        }
    }
    if closed && pts.len() > 2 && (pts[0].0 - pts[pts.len() - 1].0).length() <= 1e-6 {
        pts.pop();
        kept.pop();
    }
    let n = pts.len();
    if n < 2 {
        return PathLayout {
            kept,
            segments: Vec::new(),
            edge_count: 0,
            closed: false,
        };
    }
    let closed = closed && n >= 3;
    let edge_count = if closed { n } else { n - 1 };
    let edge = |i: usize| -> (DVec3, DVec3) { (pts[i % n].0, pts[(i + 1) % n].0) };
    let dir = |i: usize| -> DVec3 {
        let (a, b) = edge(i);
        (b - a).normalize()
    };

    // 每个拐角：转角 θ、圆角切长 t（0 = 斜接或直通）。拐角 i 在点 i 上，
    // 入边 i−1、出边 i；开口路径的两个端点不是拐角。
    #[derive(Clone, Copy)]
    struct Joint {
        turn: f64,
        tangent: f64,
        fillet: f64,
    }
    let mut joints: Vec<Option<Joint>> = vec![None; n];
    for i in 0..n {
        let is_corner = closed || (i > 0 && i + 1 < n);
        if !is_corner {
            continue;
        }
        let d_in = dir((i + edge_count - 1) % edge_count);
        let d_out = dir(i % edge_count);
        let turn = d_in.dot(d_out).clamp(-1.0, 1.0).acos();
        let fillet = pts[i].1;
        let tangent = if turn > STRAIGHT_JOINT_RAD && turn < PI - STRAIGHT_JOINT_RAD && fillet > 0.0
        {
            fillet * (turn * 0.5).tan()
        } else {
            0.0
        };
        joints[i] = Some(Joint {
            turn,
            tangent,
            fillet,
        });
    }
    // 圆角切长压过邻边：整条邻边上的两个圆角都退回斜接（各缩一半也只是另一种猜）。
    for e in 0..edge_count {
        let (a, b) = edge(e);
        let len = (b - a).length();
        let head = e % n;
        let tail = (e + 1) % n;
        let t_head = joints[head].map_or(0.0, |j| j.tangent);
        let t_tail = joints[tail].map_or(0.0, |j| j.tangent);
        if t_head + t_tail > len + 1e-9 {
            notes.push(format!(
                "路径第 {e} 段长 {len:.1} 容不下两端圆角切长 {t_head:.1}+{t_tail:.1}，这两个拐角退回斜接"
            ));
            for j in [head, tail] {
                if let Some(joint) = joints[j].as_mut() {
                    joint.tangent = 0.0;
                }
            }
        }
    }

    let mut segments: Vec<(usize, PathSegment)> = Vec::with_capacity(edge_count * 2);
    for e in 0..edge_count {
        let (a, b) = edge(e);
        let d = dir(e);
        let head = e % n;
        let tail = (e + 1) % n;
        let mitre_at = |i: usize| -> Option<DVec3> {
            let joint = joints[i]?;
            if joint.tangent > 0.0
                || joint.turn <= STRAIGHT_JOINT_RAD
                || joint.turn >= MAX_MITRE_RAD
            {
                return None;
            }
            let d_in = dir((i + edge_count - 1) % edge_count);
            let d_out = dir(i % edge_count);
            (d_in + d_out).try_normalize()
        };
        let shrink_head = joints[head].map_or(0.0, |j| j.tangent);
        let shrink_tail = joints[tail].map_or(0.0, |j| j.tangent);
        let start = a + d * shrink_head;
        let end = b - d * shrink_tail;
        if (end - start).length() > 1e-6 {
            segments.push((
                e,
                PathSegment::Line {
                    start,
                    end,
                    mitre_start: mitre_at(head),
                    mitre_end: mitre_at(tail),
                },
            ));
        }
        // 这条边尾部的拐角圆弧（只在出边存在时放——开口路径最后一点不是拐角，joints 为 None）。
        if let Some(joint) = joints[tail].filter(|j| j.tangent > 0.0) {
            let d_in = d;
            let d_out = dir((e + 1) % edge_count);
            let corner = b;
            let bisector = (d_out - d_in).normalize();
            let centre = corner + bisector * (joint.fillet / (joint.turn * 0.5).cos());
            let arc_start = corner - d_in * joint.tangent;
            let start_dir = (arc_start - centre).normalize();
            let axis = d_in.cross(d_out).normalize();
            segments.push((
                e,
                PathSegment::Arc {
                    centre,
                    radius: joint.fillet,
                    start_dir,
                    axis,
                    sweep: joint.turn,
                },
            ));
        }
    }
    PathLayout {
        kept,
        segments,
        edge_count,
        closed,
    }
}

/// 一段路径上 z=0 那根栏杆：直段是（切过斜接面的）圆柱，弧段是一节圆环。
fn rail_segment_solid(segment: &PathSegment) -> Result<Manifold, ModelError> {
    match segment {
        PathSegment::Line {
            start,
            end,
            mitre_start,
            mitre_end,
        } => {
            let d = (*end - *start).normalize();
            let sloped = (end.z - start.z).abs() > 1e-9;
            let horizontal = (d.x * d.x + d.y * d.y).sqrt();
            let slope_extension = if sloped
                && horizontal > 1e-9
                && mitre_start.is_none()
                && mitre_end.is_none()
            {
                RAIL_RADIUS * d.z.abs() / horizontal
            } else {
                0.0
            };
            let extension = |normal: &Option<DVec3>| -> f64 {
                // 斜接面把圆柱端头切成椭圆，椭圆沿轴向探出 r·tan(θ/2)；圆柱先按这个量
                // 加长，再让切面把多出来的削掉。θ 由法向与轴向的夹角还原：
                // 法向 = unit(d_in + d_out)，它与轴向 d 的夹角正是 θ/2。
                normal.map_or(0.0, |n| {
                    let cos_half = n.dot(d).abs().clamp(0.0, 1.0);
                    let sin_half = (1.0 - cos_half * cos_half).max(0.0).sqrt();
                    let tan_half = if cos_half > 1e-9 {
                        sin_half / cos_half
                    } else {
                        0.0
                    };
                    (RAIL_RADIUS * tan_half).clamp(0.0, RAIL_RADIUS * 20.0)
                })
            };
            let ext_start = extension(mitre_start);
            let ext_end = extension(mitre_end);
            let origin = *start - d * (ext_start + if d.z < 0.0 { slope_extension } else { 0.0 });
            let length = (*end - *start).length()
                + ext_start
                + ext_end
                + slope_extension;
            let body = Manifold::cylinder(
                length,
                RAIL_RADIUS,
                RAIL_RADIUS,
                cylinder_segments(RAIL_RADIUS),
                false,
            );
            let frame = DMat4::from_translation(origin) * DMat4::from_mat3(frame_along(d));
            let mut solid = placed(&body, frame);
            // 出边留角平分面的正侧，入边留负侧：`cut_by_plane` 保留 `(p − point)·normal ≥ 0`。
            if let Some(normal) = mitre_start {
                solid = cut_by_plane(&solid, *start, *normal);
            }
            if let Some(normal) = mitre_end {
                solid = cut_by_plane(&solid, *end, -*normal);
            }
            // Open layout paths use horizontal end planes. Without these
            // caps an oblique rail keeps the circular section beyond the
            // endpoint in Z, while E3D stops exactly at the rail height.
            let keep_above = end.z >= start.z;
            if sloped && mitre_start.is_none() {
                solid = if keep_above {
                    cut_by_plane(&solid, *start, DVec3::Z)
                } else {
                    cut_by_plane(&solid, *start, -DVec3::Z)
                };
            }
            if sloped && mitre_end.is_none() {
                solid = if keep_above {
                    cut_by_plane(&solid, *end, -DVec3::Z)
                } else {
                    cut_by_plane(&solid, *end, DVec3::Z)
                };
            }
            if solid.is_empty() || solid.num_tri() == 0 {
                return Err(ModelError::Solid(SolidError::EmptyManifold {
                    what: "handrail straight",
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
            let torus = circ_torus_solid(
                radius - RAIL_RADIUS,
                radius + RAIL_RADIUS,
                sweep.to_degrees(),
            )?;
            let x = *start_dir;
            let z = *axis;
            let y = z.cross(x);
            let frame =
                DMat4::from_translation(*centre) * DMat4::from_mat3(DMat3::from_cols(x, y, z));
            Ok(placed(&torus, frame))
        }
    }
}

/// 默认布局式踢脚板：8 mm 厚、110 mm 高，沿同一条路径扫掠。
/// Core3D 的 `ASL_CSGDefaultKickplate` 使用这个固定截面；截面原点在板的
/// 中心线处，故沿截面 +Y 平移半高后再复用栏杆段标架。
fn kickplate_segment_solid(segment: &PathSegment) -> Result<Manifold, ModelError> {
    let PathSegment::Line {
        start,
        end,
        mitre_start,
        mitre_end,
    } = segment
    else {
        return Err(ModelError::Solid(SolidError::Degenerate {
            what: "kickplate arc",
            detail: "layout kickplate does not support arc segments".to_string(),
        }));
    };
    let d = (*end - *start).normalize();
    let horizontal = DVec3::new(d.x, d.y, 0.0).try_normalize().unwrap_or(DVec3::X);
    let normal = DVec3::Z.cross(horizontal);
    let half = normal * 4.0;
    let points = [
        *start - half,
        *start + half,
        *start - half + DVec3::Z * 110.0,
        *start + half + DVec3::Z * 110.0,
        *end - half,
        *end + half,
        *end - half + DVec3::Z * 110.0,
        *end + half + DVec3::Z * 110.0,
    ];
    let points = points.map(|point| point.to_array());
    let mut solid = Manifold::hull_pts(&points);
    if let Some(normal) = mitre_start {
        solid = cut_by_plane(&solid, *start, *normal);
    }
    if let Some(normal) = mitre_end {
        solid = cut_by_plane(&solid, *end, -*normal);
    }
    if solid.is_empty() || solid.num_tri() == 0 {
        return Err(ModelError::Solid(SolidError::EmptyManifold {
            what: "handrail kickplate",
        }));
    }
    Ok(solid)
}

/// 把局部 +Z 转到 `d` 的旋转（列 = 新基）。
fn frame_along(d: DVec3) -> DMat3 {
    let reference = if d.z.abs() < 0.9 { DVec3::Z } else { DVec3::X };
    let x = reference.cross(d).normalize();
    let y = d.cross(x);
    DMat3::from_cols(x, y, d)
}

/// 立柱脚点（`sub_109F60C0` 逐段的那套公式），局部系、未加 `POSTOF`。
pub fn post_positions(segments: &[PathSegment], params: &HandrailParams) -> Vec<DVec3> {
    // `sub_109F5560` 第 236 行：`POSTSP > 0 && TANGOF >= 0` 才进立柱组。
    if !(params.post_spacing > 0.0 && params.tangent_offset >= 0.0) {
        return Vec::new();
    }
    let count = segments.len();
    let mut feet = Vec::new();
    for (k, segment) in segments.iter().enumerate() {
        let PathSegment::Line { start, end, .. } = segment else {
            // THRU 弧段：`sub_10720990(seg) == THRU → return 0`。
            continue;
        };
        let length = (*end - *start).length();
        let d = (*end - *start) / length;

        let mut s = params.tangent_offset;
        let mut e = params.tangent_offset;
        let prev = if k == 0 {
            params.closed.then_some(count - 1)
        } else {
            Some(k - 1)
        };
        let next = if k + 1 == count {
            params.closed.then_some(0)
        } else {
            Some(k + 1)
        };
        if !params.closed {
            if k == 0 {
                s = params.end_post_offset;
            }
            if k + 1 == count {
                e = params.end_post_offset;
            }
        }
        // 前后是共线的直段：接头不重复放柱（`D3_Vector::equals(dir, dir, 0.01)`）。
        let collinear = |index: Option<usize>| -> bool {
            index
                .and_then(|i| segments[i].direction())
                .is_some_and(|other| (other - d).length() <= COLLINEAR_TOL)
        };
        if collinear(prev) {
            s = 0.0;
        }
        if collinear(next) {
            e = 0.0;
        }
        if s + e > length {
            continue;
        }
        let run = length - s - e;
        let n = (run / params.post_spacing).floor() as i64 - 1;
        let at = |dist: f64| *start + d * dist;

        // 段首柱：`L − e ≥ 60` 才放。
        if length - e >= POST_MIN_END_RUN {
            feet.push(at(s));
        }
        // `L − s < 60`：到此为止。
        if length - s < POST_MIN_END_RUN {
            continue;
        }
        if length - e >= POST_MIN_END_RUN && n >= 0 {
            let first = (run - n as f64 * params.post_spacing) * 0.5 + s;
            for i in 0..=n {
                feet.push(at(first + i as f64 * params.post_spacing));
            }
        }
        // 段尾柱，无条件。
        feet.push(at(length - e));
    }
    feet
}

// ---------------------------------------------------------------------------
// 详图式扶手的成员（recheck-4 §五 W4-1 施工图·续 七）。
// ---------------------------------------------------------------------------

/// 详图式扶手里**自己画几何**的两个成员。`ASL_MDR_CSGRail`（壳 `0x109F9C10`）与
/// `ASL_MDR_CSGKickplate`（壳 `0x109F9890`）的 20 个虚表槽逐槽相同
/// （`evidence/20260906-asl-builders/vtbl_dump_kickplate.py`），差别全在属性：
/// `JUSL` / `XOFF` / `ZOFF`。同族的 `HRPOST` 不在这里——它的 `SPRE` 指向一只没有
/// `CATR` 的 `GPART`，走的是目录构件那条路（`crate::catalogue`），不是截面链。
pub const DETAILED_ITEM_NOUNS: [&str; 2] = ["RAIL", "KICKPL"];

/// 这个 noun 是不是 [`DETAILED_ITEM_NOUNS`] 里的一个。
pub fn is_detailed_item(noun: &str) -> bool {
    DETAILED_ITEM_NOUNS
        .iter()
        .any(|known| noun.trim().eq_ignore_ascii_case(known))
}

/// [`detailed_item_local_solid`] 的三种结局，与 SCTN 的 `section_body` 同口径：
/// 「E3D 也只有中心线」与「目录链根本走不通」各有各的账，混成一句话就分不出
/// 是哪种欠账。
pub enum RailBuild {
    Solid(Manifold),
    CentrelineOnly(String),
    ChainBroken(String),
}

/// 详图式扶手成员（`RAIL` / `KICKPL`）的实体，在成员自己的局部系里
/// ——`RAILSE` / `HRKPSE` 与成员自身都没有 `POS`/`ORI`，那正是属主 HANDRA 的局部系。
///
/// 对标 `ASL_MDR_CSGRail`（ctor `sub_105A9420` → 引擎 `sub_105A97C0`）：
///
/// 1. ctor 连取两次 owner 拿属主 HANDRA（`sub_105A9420` 第 33–35 行）：路径与 `CLOSEF`
///    读的都是它的，成员自己只带端点引用与偏移。
/// 2. `downloadDataFromDB`（`sub_105AB2C0` 第 101–148 行）把 `STAREF` / `ENDREF` 换成
///    路径曲线的起止下标，取其中一串**连续**的曲线，闭合路径允许绕回去（[`curve_run`]）。
/// 3. `getProfile`（vtbl+52 `sub_105B9C50` = `DB_RoutingBranch::getProfile`）走
///    `SPRE → GSTR → GMSS`、逐成员按 `LEVE` 过显示筛——与 SCTN 逐句同形，
///    [`crate::section::section_catalogue`] 原样可用。
/// 4. `setMitrePlanes`（vtbl+64）+ `do_solid_segments`（vtbl+40，与布局式共用
///    `sub_105B57E0`）沿曲线扫，拐角切角平分面。
pub fn detailed_item_local_solid(
    el: &DbElement,
    notes: &mut Vec<String>,
) -> Result<RailBuild, ModelError> {
    let noun = el.element_type()?;
    let refno = el.refno().to_string();

    let Some(handrail) = owning_handrail(el)? else {
        return Ok(RailBuild::CentrelineOnly(
            "属主链上两级不是 HANDRA：扫掠路径无从谈起，E3D 的 ctor 同样取不到".to_string(),
        ));
    };
    let Some(rpath) = first_rpath(&handrail)? else {
        return Ok(RailBuild::CentrelineOnly(format!(
            "属主 HANDRA {} 没有 RPATH 成员：没有路径可扫",
            handrail.refno()
        )));
    };
    let raw = path_points(&rpath)?;
    let closed = handrail.get_bool("CLOSEF")?.unwrap_or(false);
    let layout = path_layout(
        &raw.points
            .iter()
            .map(|(_, pos, fillet)| (*pos, *fillet))
            .collect::<Vec<_>>(),
        closed,
        notes,
    );
    if layout.segments.is_empty() {
        return Ok(RailBuild::CentrelineOnly(format!(
            "属主 HANDRA {} 的 RPATH 扫不出一段（{} 个点）",
            handrail.refno(),
            raw.points.len()
        )));
    }

    // `STAREF` / `ENDREF` 指的是原始 POINTR，经 `kept` 换算成去重后的点号 = 曲线号。
    let point_index = |name: &'static str| -> Result<Option<usize>, ModelError> {
        let Some(target) = el.get_element(name)? else {
            return Ok(None);
        };
        let target = target.refno().to_string();
        Ok(raw
            .points
            .iter()
            .position(|(refno, _, _)| *refno == target)
            .and_then(|original| layout.kept.iter().position(|kept| *kept == original)))
    };
    let (Some(start), Some(end)) = (point_index("STAREF")?, point_index("ENDREF")?) else {
        return Ok(RailBuild::CentrelineOnly(
            "STAREF / ENDREF 不在属主 RPATH 的点串上，定不出要扫哪一段".to_string(),
        ));
    };
    let run = curve_run(start, end, layout.edge_count, layout.closed);
    if run.is_empty() {
        return Ok(RailBuild::CentrelineOnly(format!(
            "STAREF 点 {start} → ENDREF 点 {end} 在 {} 条曲线（{}）上取不出一段，E3D 同样一段不画",
            layout.edge_count,
            if layout.closed { "闭合" } else { "开口" }
        )));
    }
    // 要扫的这一串边里，有没有一条是本期不认的路径段（`CURVE`/`RSECT`/`EXTGEO`）。
    // 有就整根不画——照直段画出来的是一根形状不对的杆，而它会顶着这只 RAIL 的 refno
    // 进 `elements.json`，比缺一件更难发现。与 GENSEC 带弧交 `CentrelineOnly` 同口径。
    for (original, why) in &raw.unwired {
        let Some(edge) = layout.kept.iter().position(|kept| kept == original) else {
            continue;
        };
        if run.contains(&edge) {
            return Ok(RailBuild::CentrelineOnly(format!(
                "要扫的第 {edge} 段上有本期不认的路径段：{why}"
            )));
        }
    }

    let mut profile = match section_catalogue(el)? {
        SectionChain::NoSpre => {
            return Ok(RailBuild::CentrelineOnly(
                "无 SPRE：E3D 只画中心线，不出实体".to_string(),
            ));
        }
        SectionChain::Broken { hop, at, why } => {
            return Ok(RailBuild::ChainBroken(format!(
                "SPRE 链第 {hop} 跳断在 {at}：{why}"
            )));
        }
        SectionChain::Resolved(catalogue) => evaluate_section_profile(el, &catalogue)?,
    };
    notes.extend(profile.notes.iter().cloned());
    if profile.is_empty() {
        return Ok(RailBuild::CentrelineOnly(
            "目录截面一个成员都出不了形状（全被显示筛掉或是空占位），E3D 同样只画中心线"
                .to_string(),
        ));
    }

    let bang_deg = el.get_double("BANG")?.unwrap_or(0.0);
    let x_offset = el.get_double("XOFF")?.unwrap_or(0.0);
    let z_offset = el.get_double("ZOFF")?.unwrap_or(0.0);
    if x_offset != 0.0 {
        // `XOFF` 偏的是**路径**（`ASL_MDR_CSGRail::offsetPath`，`sub_105AB2C0` 第 154–246 行
        // 逐条曲线偏）。把截面在自己平面里横移同样的量与之等价：角平分面的法向
        // `unit(d_in + d_out)` 与两条侧向量之和垂直（`x_in·d_out + x_out·d_in = 0`），
        // 偏出去的拐点仍落在同一张斜接面上。于是折进 `justification`——放置时减的正是它。
        //
        // **正负没有基准可对**：`sub_105AB2C0` 传给 offsetPath 的是 `-this[63]`，
        // 而那个字段是不是 `XOFF`、offsetPath 的正向朝哪一侧，两处都没坐实。
        profile.justification[0] += x_offset;
        notes.push(format!(
            "XOFF={x_offset} 按截面平面 +x 侧偏（等价于偏路径）；偏向未对拍"
        ));
    }
    if let Some(ydir) = el.get_direction("YDIR")?
        && DVec3::from_array(ydir)
            .try_normalize()
            .is_some_and(|y| (y - DVec3::Z).length() > 1e-6)
    {
        notes.push(format!(
            "YDIR={ydir:?} 不是 +Z：截面 Y 向本期照 GTDIRS 的参考轴取（语料 13 件全是 +Z）"
        ));
    }

    let mut pieces: Vec<Manifold> = Vec::new();
    for index in &run {
        for (_, segment) in layout.segments.iter().filter(|(edge, _)| edge == index) {
            let PathSegment::Line {
                start,
                end,
                mitre_start,
                mitre_end,
            } = segment
            else {
                // `FRAD` 圆角弧：与 GENSEC 的 SPINE 弧同一件事——截面绕弧心回转。
                // 借 `spine_swept_solid` 单段跑一趟，省得把回转那段再抄一遍；
                // 弧与两邻边相切，不需要斜接面。
                match spine_swept_solid(&profile, &arc_as_spine(segment), bang_deg, notes)? {
                    Some(body) => pieces.push(body),
                    None => notes.push(format!("路径第 {index} 段（圆角弧）扫不出实体")),
                }
                continue;
            };
            let placement = match placement_between(&refno, &noun, *start, *end, bang_deg, &profile)
            {
                Ok(placement) => placement,
                Err(error) => {
                    notes.push(format!("路径第 {index} 段放置求不出，跳过该段：{error}"));
                    continue;
                }
            };
            // 斜接面的法向在 HANDRA 局部系里，`cut_plane_normal` 逆旋进段局部系并
            // 规约成朝段内（起点端 +Z、终点端 −Z）；没有斜接面时给零向量，
            // 它会被规约成轴向 `(0,0,1)`，`SectionCutPlane::is_axial` 于是跳过这一刀。
            let to_local = DMat3::from_mat4(placement.local).transpose();
            let cuts = SectionCuts {
                start: SectionCutPlane {
                    point: DVec3::ZERO,
                    normal: cut_plane_normal(to_local, mitre_start.unwrap_or(DVec3::ZERO), 1.0),
                },
                end: SectionCutPlane {
                    point: DVec3::new(0.0, 0.0, placement.length),
                    normal: cut_plane_normal(to_local, mitre_end.unwrap_or(DVec3::ZERO), -1.0),
                },
            };
            match section_local_solid(&profile, &placement, &cuts, notes)? {
                // 详图式扶手一段一块并列，RVM 那边也没有按 GMSS 成员分条的取证，
                // 逐成员那几块这里用不上。
                Some(built) => pieces.push(placed(&built.body, placement.local)),
                None => notes.push(format!("路径第 {index} 段扫不出实体")),
            }
        }
    }
    let solid = match pieces.as_slice() {
        [] => {
            return Ok(RailBuild::CentrelineOnly(format!(
                "{} 段路径一段实体都没扫出来",
                run.len()
            )));
        }
        // 相邻段在同一张斜接面上相接不重叠，与布局式一样并列不做布尔并。
        [only] => only.clone(),
        many => Manifold::compose(many),
    };
    notes.push(format!(
        "详图式{noun}：沿属主 RPATH 第 {:?} 段扫 {} 节，ZOFF={z_offset} JUSL={}",
        run,
        pieces.len(),
        profile.justification_line
    ));
    Ok(RailBuild::Solid(if z_offset == 0.0 {
        solid
    } else {
        // `ZOFF` 与布局式的 `RAILHE` 同一个意思：整根栏杆沿属主局部 +Z 抬起来
        // （语料里 RAIL 的 ZOFF 恰好就是属主的 `RAILHE` 两个值）。
        solid.translate(0.0, 0.0, z_offset)
    }))
}

/// 把一段圆角弧包成单段 [`SpinePath`]，好让 [`spine_swept_solid`] 去回转截面。
///
/// `ydir` 给 +Z：语料里 `RPATH` 的 `YDIR` 全是它，也与直段那边
/// [`crate::section::placement_between`] 的 GTDIRS 参考轴同向。直段不是弧的一律给
/// 一段零长直段——调用方只对 [`PathSegment::Arc`] 调它。
fn arc_as_spine(segment: &PathSegment) -> SpinePath {
    let segments = match segment {
        PathSegment::Arc {
            centre,
            radius,
            start_dir,
            axis,
            sweep,
        } => {
            let start = *centre + *start_dir * *radius;
            let end = *centre + DQuat::from_axis_angle(*axis, *sweep) * (*start_dir * *radius);
            vec![SpineSegment::Arc {
                start,
                end,
                centre: *centre,
                axis: *axis,
                sweep: *sweep,
            }]
        }
        PathSegment::Line { .. } => Vec::new(),
    };
    SpinePath {
        segments,
        ydir: DVec3::Z,
        drns: DVec3::ZERO,
        drne: DVec3::ZERO,
        curve_types: Vec::new(),
        problems: Vec::new(),
    }
}

/// `RAIL → RAILSE → HANDRA`（`KICKPL → HRKPSE → HANDRA`）：`sub_105A9420` 第 33–35 行
/// 连取两次 owner，取到的不是 HANDRA 就没有路径可用。
fn owning_handrail(el: &DbElement) -> Result<Option<DbElement>, ModelError> {
    let Some(set) = el.owner()? else {
        return Ok(None);
    };
    let Some(owner) = set.owner()? else {
        return Ok(None);
    };
    Ok(owner
        .element_type()?
        .eq_ignore_ascii_case("HANDRA")
        .then_some(owner))
}

/// HANDRA 的第一个 `RPATH` 成员（`sub_109F7150` 把它存进 `+128`）。
fn first_rpath(handrail: &DbElement) -> Result<Option<DbElement>, ModelError> {
    for member in handrail.members()? {
        if member.element_type()? == "RPATH" {
            return Ok(Some(member));
        }
    }
    Ok(None)
}

/// 一条 `RPATH` 读出来的路径。
struct RawPath {
    /// `POINTR` 的 refno、位置、`FRAD` 倒角半径。
    points: Vec<(String, DVec3, f64)>,
    /// 本期没接的那些路径段：`(夹在它前面的那个点在 points 里的下标, 说明)`。
    ///
    /// `createCurvesFromPath` 除点串外还认三种：`CURVE`/`RSECT` 走 vtbl+24 出圆弧、
    /// `EXTGEO` 走 `sub_105D13B0` 出外部几何段。它们夹在两个 `POINTR` 之间，
    /// **把那条边从直段换成别的东西**——只记一条 note 再照直段画，画出来的是一根
    /// 形状不对的杆；所以要把边号一并交出去，由调用方决定这一段还画不画。
    unwired: Vec<(usize, String)>,
}

/// 读一条 `RPATH` 的点串。
fn path_points(rpath: &DbElement) -> Result<RawPath, ModelError> {
    let mut points: Vec<(String, DVec3, f64)> = Vec::new();
    let mut unwired = Vec::new();
    for member in rpath.members()? {
        let member_noun = member.element_type()?;
        if member_noun != "POINTR" {
            unwired.push((
                points.len().saturating_sub(1),
                format!(
                    "RPATH 成员 {}（{member_noun}）不是 POINTR：这条边 E3D 画的是圆弧 / 外部几何段，本期未接",
                    member.refno()
                ),
            ));
            continue;
        }
        let pos = member
            .get_position("POS")?
            .map(DVec3::from_array)
            .ok_or_else(|| ModelError::MissingAttribute {
                refno: member.refno().to_string(),
                noun: "POINTR".into(),
                name: "POS",
                what: "handrail path point",
            })?;
        let fillet = member.get_double("FRAD")?.unwrap_or(0.0).max(0.0);
        points.push((member.refno().to_string(), pos, fillet));
    }
    Ok(RawPath { points, unwired })
}

/// `STAREF` / `ENDREF` 两个点号 → 要扫的曲线号串（`sub_105AB2C0` 第 101–148 行）。
///
/// 曲线 i 从点 i 走到点 i+1，所以起点就是 `STAREF` 的点号、止点是 `ENDREF` 的点号。
/// 闭合路径允许绕过去（`end ≤ start` 时补一圈），越界、`end == 0`、开口路径上
/// `end < start`、以及算出来超过一圈，一律空串——那正是 E3D 的 `this[74] = 0`，一段不画。
pub fn curve_run(start: usize, end: usize, edge_count: usize, closed: bool) -> Vec<usize> {
    if edge_count == 0 || start >= edge_count || end == 0 {
        return Vec::new();
    }
    let run = if closed {
        if end <= start {
            end + edge_count - start
        } else {
            end - start
        }
    } else if end < start {
        0
    } else {
        end - start
    };
    if run == 0 || run > edge_count {
        return Vec::new();
    }
    (0..run).map(|step| (start + step) % edge_count).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params(spacing: f64, tangent: f64, closed: bool) -> HandrailParams {
        HandrailParams {
            rail_heights: vec![1100.0, 600.0],
            post_spacing: spacing,
            tangent_offset: tangent,
            end_post_offset: 0.0,
            post_offset: 0.0,
            closed,
            show_kickplate: false,
        }
    }

    fn p(x: f64, y: f64) -> DVec3 {
        DVec3::new(x, y, 0.0)
    }

    /// 直角拐角不带圆角：两段各带一张斜接面，法向是角平分面 `unit(d_in + d_out)`。
    #[test]
    fn a_sharp_corner_gets_one_mitre_plane_shared_by_both_straights() {
        let mut notes = Vec::new();
        let segments = build_path(
            &[
                (p(0.0, 0.0), 0.0),
                (p(1000.0, 0.0), 0.0),
                (p(1000.0, 800.0), 0.0),
            ],
            false,
            &mut notes,
        );
        assert_eq!(segments.len(), 2, "{segments:?}");
        let expected = (DVec3::X + DVec3::Y).normalize();
        match (&segments[0], &segments[1]) {
            (
                PathSegment::Line {
                    mitre_start: None,
                    mitre_end: Some(n1),
                    ..
                },
                PathSegment::Line {
                    mitre_start: Some(n2),
                    mitre_end: None,
                    ..
                },
            ) => {
                assert!((*n1 - expected).length() < 1e-9, "{n1}");
                assert!((*n2 - expected).length() < 1e-9, "{n2}");
            }
            other => panic!("{other:?}"),
        }
        assert!(notes.is_empty(), "{notes:?}");
    }

    /// 带 FRAD 的拐角：两邻边各缩 `FRAD·tan(θ/2)`，中间一节弧切于两边、圆心在角平分线上。
    #[test]
    fn a_filleted_corner_shortens_both_straights_and_inserts_a_tangent_arc() {
        let mut notes = Vec::new();
        let segments = build_path(
            &[
                (p(0.0, 0.0), 0.0),
                (p(1000.0, 0.0), 100.0),
                (p(1000.0, 800.0), 0.0),
            ],
            false,
            &mut notes,
        );
        assert_eq!(segments.len(), 3, "{segments:?}");
        let PathSegment::Line { end, mitre_end, .. } = &segments[0] else {
            panic!("{segments:?}")
        };
        assert!((*end - p(900.0, 0.0)).length() < 1e-9, "{end}");
        assert!(mitre_end.is_none(), "圆角接头不切斜接面");
        let PathSegment::Arc {
            centre,
            radius,
            start_dir,
            axis,
            sweep,
        } = &segments[1]
        else {
            panic!("{segments:?}")
        };
        assert!((*centre - p(900.0, 100.0)).length() < 1e-9, "{centre}");
        assert_eq!(*radius, 100.0);
        assert!((*start_dir - (-DVec3::Y)).length() < 1e-9, "{start_dir}");
        assert!((*axis - DVec3::Z).length() < 1e-9, "{axis}");
        assert!(
            (sweep - std::f64::consts::FRAC_PI_2).abs() < 1e-9,
            "{sweep}"
        );
        let PathSegment::Line { start, .. } = &segments[2] else {
            panic!("{segments:?}")
        };
        assert!((*start - p(1000.0, 100.0)).length() < 1e-9, "{start}");
        // 弧终点 = 圆心 + 起向量绕轴转 θ，必须落在下一段的起点上。
        let rotated = glam::DQuat::from_axis_angle(*axis, *sweep) * (*start_dir * *radius);
        assert!((*centre + rotated - *start).length() < 1e-9);
    }

    /// 圆角切长压过邻边就退回斜接，并且说出来。
    #[test]
    fn an_oversized_fillet_falls_back_to_a_mitre_and_says_so() {
        let mut notes = Vec::new();
        let segments = build_path(
            &[
                (p(0.0, 0.0), 0.0),
                (p(100.0, 0.0), 300.0),
                (p(100.0, 800.0), 0.0),
            ],
            false,
            &mut notes,
        );
        assert_eq!(segments.len(), 2, "{segments:?}");
        assert!(matches!(
            &segments[0],
            PathSegment::Line {
                mitre_end: Some(_),
                ..
            }
        ));
        assert_eq!(notes.len(), 1, "{notes:?}");
    }

    /// `sub_109F60C0` 的公式：L=3000、s=e=300、POSTSP=1000 → n=1，中间两根居中，共四根。
    #[test]
    fn posts_follow_the_core3d_spacing_formula_on_an_open_run() {
        let mut notes = Vec::new();
        let segments = build_path(
            &[(p(0.0, 0.0), 0.0), (p(3000.0, 0.0), 0.0)],
            false,
            &mut notes,
        );
        // 开口支线首尾段用 EPOSTO（这里是 0）——单段既是首也是尾。
        let mut params = params(1000.0, 300.0, false);
        params.end_post_offset = 300.0;
        let feet: Vec<f64> = post_positions(&segments, &params)
            .iter()
            .map(|f| f.x)
            .collect();
        // run = 2400, n = floor(2400/1000) − 1 = 1, first = (2400 − 1000)/2 + 300 = 1000
        assert_eq!(feet, vec![300.0, 1000.0, 2000.0, 2700.0]);
    }

    /// 共线接头：前一段是同向直段，接头处退距归零，不会在同一点放两根。
    #[test]
    fn collinear_joints_drop_the_setback() {
        let mut notes = Vec::new();
        let segments = build_path(
            &[
                (p(0.0, 0.0), 0.0),
                (p(1000.0, 0.0), 0.0),
                (p(2000.0, 0.0), 0.0),
            ],
            false,
            &mut notes,
        );
        let mut params = params(5000.0, 300.0, false);
        params.end_post_offset = 100.0;
        let feet: Vec<f64> = post_positions(&segments, &params)
            .iter()
            .map(|f| f.x)
            .collect();
        // 段 0：s=EPOSTO=100，e=0（后段共线）→ 首柱 100，n<0，尾柱 1000
        // 段 1：s=0（前段共线），e=EPOSTO=100 → 首柱 1000（与上一段尾柱重合是 E3D 的行为，照抄），尾柱 1900
        assert_eq!(feet, vec![100.0, 1000.0, 1000.0, 1900.0]);
    }

    /// 弧段不放柱，且 POSTSP ≤ 0 一根都不放。
    #[test]
    fn arcs_carry_no_posts_and_zero_spacing_disables_posts() {
        let mut notes = Vec::new();
        let segments = build_path(
            &[
                (p(0.0, 0.0), 0.0),
                (p(1000.0, 0.0), 100.0),
                (p(1000.0, 800.0), 0.0),
            ],
            false,
            &mut notes,
        );
        let feet = post_positions(&segments, &params(700.0, 300.0, false));
        assert!(
            feet.iter()
                .all(|f| f.y.abs() < 1e-9 || (f.x - 1000.0).abs() < 1e-9),
            "{feet:?}"
        );
        assert!(post_positions(&segments, &params(0.0, 300.0, false)).is_empty());
    }

    /// 段表带得出「这一段来自第几条边」——详图式成员按边号切子段表，圆角拐角
    /// 让一条边出两段，边号必须跟着走。
    #[test]
    fn every_segment_knows_which_edge_it_came_from() {
        let mut notes = Vec::new();
        let layout = path_layout(
            &[
                (p(0.0, 0.0), 0.0),
                (p(1000.0, 0.0), 100.0),
                (p(1000.0, 800.0), 0.0),
            ],
            false,
            &mut notes,
        );
        assert_eq!(layout.kept, vec![0, 1, 2]);
        assert_eq!(layout.edge_count, 2);
        // 边 0 出「直段 + 尾部圆角弧」两段，边 1 只出直段。
        let edges: Vec<usize> = layout.segments.iter().map(|(edge, _)| *edge).collect();
        assert_eq!(edges, vec![0, 0, 1]);
        assert!(matches!(layout.segments[1].1, PathSegment::Arc { .. }));
        // 重合点被丢掉之后，`kept` 仍指得回原始下标。
        let mut notes = Vec::new();
        let layout = path_layout(
            &[
                (p(0.0, 0.0), 0.0),
                (p(0.0, 0.0), 0.0),
                (p(1000.0, 0.0), 0.0),
            ],
            false,
            &mut notes,
        );
        assert_eq!(layout.kept, vec![0, 2]);
        assert_eq!(layout.segments.len(), 1);
    }

    /// `sub_105AB2C0` 第 101–148 行：`STAREF` / `ENDREF` 两个点号 → 连续的曲线号串。
    #[test]
    fn the_curve_run_follows_the_start_and_end_point_references() {
        // 开口路径整条：6 点 5 边，点 0 → 点 5（语料 `2013286654/3467`）。
        assert_eq!(curve_run(0, 5, 5, false), vec![0, 1, 2, 3, 4]);
        // 闭合路径绕过去：6 点 6 边，点 3 → 点 2 走长的那一边（语料 `2013286654/3387`）。
        assert_eq!(curve_run(3, 2, 6, true), vec![3, 4, 5, 0, 1]);
        // 踢脚板一次只占一条边（语料 `2013286654/8648`）。
        assert_eq!(curve_run(1, 2, 2, false), vec![1]);
        // 闭合路径首尾同点 = 整整一圈。
        assert_eq!(curve_run(2, 2, 4, true), vec![2, 3, 0, 1]);
        // E3D 一段都不画的四种：起点越界、`end == 0`、开口路径上倒着来、算出来超过一圈。
        assert!(curve_run(5, 2, 5, false).is_empty());
        assert!(curve_run(2, 0, 5, true).is_empty());
        assert!(curve_run(3, 1, 5, false).is_empty());
        assert!(curve_run(0, 7, 5, false).is_empty());
    }

    /// 两根直角相接的 Ø40 管：过轴端点的斜接面切掉多少就补回多少（体积不变），
    /// 两根在同一张面上相接不重叠，外角在拐角外侧（+x 且 −y 那一象限）。
    ///
    /// 体积不变对两条对角面都成立，所以单靠它验不出面切反了；后两条才是判据——
    /// 面切反时两根管在内侧重叠一个楔、外侧张开一个缺口。
    #[test]
    fn mitred_straights_meet_on_the_bisector_plane() {
        let mut notes = Vec::new();
        let segments = build_path(
            &[
                (p(0.0, 0.0), 0.0),
                (p(500.0, 0.0), 0.0),
                (p(500.0, 500.0), 0.0),
            ],
            false,
            &mut notes,
        );
        let a = rail_segment_solid(&segments[0]).unwrap();
        let b = rail_segment_solid(&segments[1]).unwrap();
        let plain_a = Manifold::cylinder(
            500.0,
            RAIL_RADIUS,
            RAIL_RADIUS,
            cylinder_segments(RAIL_RADIUS),
            false,
        )
        .volume();
        for solid in [&a, &b] {
            assert!(
                ((solid.volume() - plain_a) / plain_a).abs() < 1e-3,
                "{}",
                solid.volume()
            );
        }
        // 相接不重叠：并集体积 = 两者之和。
        let union = &a + &b;
        assert!(
            ((union.volume() - a.volume() - b.volume()) / plain_a).abs() < 1e-3,
            "{}",
            union.volume()
        );
        // 入边探出拐点的那一楔在外侧（y < 0）：x > 500 且 y > 0 的部分应当为空。
        let quadrant = |solid: &Manifold, sx: f64, sy: f64| {
            cut_by_plane(
                &cut_by_plane(solid, p(500.0, 0.0), DVec3::X * sx),
                p(500.0, 0.0),
                DVec3::Y * sy,
            )
            .volume()
        };
        let report = format!(
            "a: (+x,+y)={:.0} (+x,−y)={:.0} (−x,+y)={:.0} (−x,−y)={:.0}; b: (+x,+y)={:.0} (+x,−y)={:.0} (−x,+y)={:.0} (−x,−y)={:.0}",
            quadrant(&a, 1.0, 1.0),
            quadrant(&a, 1.0, -1.0),
            quadrant(&a, -1.0, 1.0),
            quadrant(&a, -1.0, -1.0),
            quadrant(&b, 1.0, 1.0),
            quadrant(&b, 1.0, -1.0),
            quadrant(&b, -1.0, 1.0),
            quadrant(&b, -1.0, -1.0),
        );
        assert!(quadrant(&a, 1.0, 1.0) < 1.0, "{report}");
        // 探出的楔：{x ∈ [0, r], y ≤ −x, y² + z² ≤ r²} 的体积 = ∫₀ʳ 弓形面积 = (2/3)·r³。
        let wedge = 2.0 / 3.0 * RAIL_RADIUS.powi(3);
        assert!(
            ((quadrant(&a, 1.0, -1.0) - wedge) / wedge).abs() < 0.05,
            "{report}"
        );
        // 出边对称：它探到拐点下方（y < 0）的楔在 x > 500 一侧，(−x, −y) 象限为空。
        assert!(quadrant(&b, -1.0, -1.0) < 1.0, "{report}");
        assert!(
            ((quadrant(&b, 1.0, -1.0) - wedge) / wedge).abs() < 0.05,
            "{report}"
        );
    }
}

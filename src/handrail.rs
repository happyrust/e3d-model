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
//!    一份：`RAILHE` 是栏杆**顶**高——路径抬起 (0,0,h) 是顶母线，截面圆挂在它下面（圆心沿截面「上」
//!    向往下 20，见 [`section_up`]）。直段是圆柱、两端平头，拐角在 `FRAD > 0` 时是一节圆环（`THRU` 弧），
//!    `FRAD = 0` 时两根圆柱各切一张斜接面（角平分面）。
//! 2. **立柱**：`sub_109F60C0`，Ø30 圆柱，高 `maxRAILHE − 20 − POSTOF`（柱顶停在顶栏杆轴线
//!    下 20，即 Ø40 栏杆的下缘），逐直段按 `TANGOF` / `EPOSTO` 退距、`POSTSP` 间距均分，
//!    共线接头不重复放柱，弧段不放柱。
//! 3. **门洞**（`HRGATE` 成员）与 **踢脚板**（`SHOWKP`）——本机语料里门一件都没有，只进 notes；
//!    踢脚板照 E3D TTY 导出建：板厚 4、离路径 10 起高 100（截面「上」向），拐角与栏杆同样斜接。
//!
//! 取证（2026-10-22，修复计划 W7a）：aps250112 `/EQUIP_SUPPORT/SF_001` 名下的梯段扶手，平台段栏杆截面环中心在
//! `RAILHE − 20`（105980 / 103550 / 103050）、顶在 `RAILHE`；aps250110 `/B01_LEVEL_03_ACCESS/SF_002` 斜段端环
//! 垂直于轴、顶点正好在端点上方 `RAILHE` 处；踢脚板在两库的导出里都是 ±2 × [10, 110]。此前轴线放在 `RAILHE`、
//! 斜段两端用水平切面补包围盒，平台段就高出 20 mm。
//!
//! 开口路径的两端（2026-10-24，W7t）：`RPATH` 的 `DRNS` / `DRNE` 非零时，端面是**过端点、法向 = 端向**的平面，与 GENSEC
//! SPINE 的斜接同一条规则（[`spine_mitre_plane`]：与切向平行 / 垂直或为零时仍是垂直于轴的平头）。aps250209
//! `/A01-AREA05-BASE-FL-STAIRS` 的 16 只梯段扶手（路径 = 平台段 + 斜段、`DRNE = (0, 1, 0)`）在 E3D TTY 导出里斜段顶端是
//! `y = 端点` 的竖直面（端柱还在端点上），此前按平头画多伸 24.07 − 15 = 9.07 mm；aps250110 `SF_002` 的 `DRNS` / `DRNE` 是零、
//! aps250112 `SF_001` 的两端是平台段，两条规则在那里给出同一张面，所以此前没露出来。
//!
//! **对拍**：`test_data/rvm/` 里没有任何一份含 HANDRA 的 E3D 导出，这个模块的几何
//! 到 2026-09-06 为止只有自证（单测里的解析几何），没有与 E3D 逐件对过。

use std::f64::consts::PI;

use glam::{DMat3, DMat4, DQuat, DVec3};
use manifold_csg::Manifold;

use e3d_io::db_element::DbElement;

use crate::elmodl::ModelError;
use crate::section::{
    SectionChain, SectionCutPlane, SectionCuts, SectionProfile, SpinePath, SpineSegment,
    cut_plane_normal, evaluate_section_profile, placement_between, section_catalogue,
    section_local_solid, spine_mitre_plane, spine_swept_solid,
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
/// 布局式踢脚板：板厚、离路径的起高与板高（截面「上」向，E3D TTY 导出）。
const KICKPLATE_THICKNESS: f64 = 4.0;
const KICKPLATE_LIFT: f64 = 10.0;
const KICKPLATE_HEIGHT: f64 = 100.0;
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

    let mut segments = build_path(&points, params.closed, notes);
    if segments.is_empty() {
        return Ok(HandrailBuild::NoGeometry(
            "路径点全部重合，没有一段有长度，E3D 同样不出几何".to_string(),
        ));
    }
    apply_end_directions(&mut segments, params.closed, raw.drns, raw.drne, notes);

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
        // 踢脚板挂在路径标高上，不随栏杆抬；也**不跟圆角弧**：E3D 把直板一直画到原拐点、按角平分面斜接
        // （aps250209 `/A01-AREA05-4TH-FLOOR-HANDRAILS` FRAD 100 的水平拐角、`/A02-AREA07-HANDRAILS` 竖直面里的内弯拐角，
        // TTY 导出的板顶 / 板底都在过拐点的角平分面上相交），所以按没有倒角的路径重排一遍直段。
        let unfilleted: Vec<(DVec3, f64)> = points.iter().map(|&(point, _)| (point, 0.0)).collect();
        let mut plate_segments = build_path(&unfilleted, params.closed, &mut Vec::new());
        apply_end_directions(
            &mut plate_segments,
            params.closed,
            raw.drns,
            raw.drne,
            &mut Vec::new(),
        );
        for segment in &plate_segments {
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
        notes.push("SHOWKP=true：已生成默认踢脚板（板厚 4、离路径 10 起高 100）".to_string());
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

/// 开口路径的两端按 `RPATH` 的 `DRNS` / `DRNE` 切（Core3D `setMitrePlanes` 的端向那一半）：首段起点 / 末段终点
/// 本来是平头（`mitre_* = None`），端向非零且与该段既不平行也不垂直时，换成过端点、法向 = 端向的平面；
/// 法向按拐角斜接面的约定规约成**沿行进方向**（[`rail_segment_solid`] / [`kickplate_segment_solid`] 的起点留正侧、
/// 终点留负侧）。闭合路径没有端，不动。
pub fn apply_end_directions(
    segments: &mut [PathSegment],
    closed: bool,
    drns: DVec3,
    drne: DVec3,
    notes: &mut Vec<String>,
) {
    if closed || segments.is_empty() {
        return;
    }
    let last = segments.len() - 1;
    for (index, drn, is_start, label) in [(0usize, drns, true, "DRNS"), (last, drne, false, "DRNE")]
    {
        let PathSegment::Line {
            start,
            end,
            mitre_start,
            mitre_end,
        } = &mut segments[index]
        else {
            continue;
        };
        let slot = if is_start { mitre_start } else { mitre_end };
        if slot.is_some() {
            continue;
        }
        let tangent = *end - *start;
        let point = if is_start { *start } else { *end };
        let Some(plane) = spine_mitre_plane(drn, tangent, point, is_start) else {
            continue;
        };
        // `spine_mitre_plane` 把法向规约成指向体内；这里的槽位约定是沿行进方向（起点端两者一致，终点端相反）。
        let normal = if is_start {
            plane.normal
        } else {
            -plane.normal
        };
        *slot = Some(normal);
        notes.push(format!(
            "RPATH {label} = [{:.3}, {:.3}, {:.3}]：开口端按端向切（过端点的平面），不是垂直于轴的平头",
            drn.x, drn.y, drn.z
        ));
    }
}

/// 一段路径上顶母线在 z=0 的那根栏杆：直段是（切过斜接面的）圆柱，弧段是一节圆环。
///
/// 截面圆心在路径下方 `RAIL_RADIUS`（沿 [`section_up`]），两端平头、端环垂直于轴。斜接面过顶母线上的拐点：
/// 两段轴线各自垂直往下偏同一距离，交点仍在角平分面上。
fn rail_segment_solid(segment: &PathSegment) -> Result<Manifold, ModelError> {
    match segment {
        PathSegment::Line {
            start,
            end,
            mitre_start,
            mitre_end,
        } => {
            let d = (*end - *start).normalize();
            let down = -section_up(d) * RAIL_RADIUS;
            // 斜接面把截面推出 |v|·tan(θ/2)，截面离顶母线最远 2·RAIL_RADIUS；圆柱先加长这么多，
            // 再让切面把多出来的削掉。θ 由法向与轴向的夹角还原：法向 = unit(d_in + d_out)。
            let extension = |normal: &Option<DVec3>| -> f64 {
                normal.map_or(0.0, |n| 2.0 * RAIL_RADIUS * mitre_tan_half(n, d))
            };
            let ext_start = extension(mitre_start);
            let ext_end = extension(mitre_end);
            let origin = *start + down - d * ext_start;
            let length = (*end - *start).length() + ext_start + ext_end;
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
            // 同一个挂法：截面圆心沿起点处的「上」向往下 RAIL_RADIUS，拆成径向与轴向两份——水平弧
            // 整只下沉、竖直面里的弧改径向半径，扫的过程中这两份不变。
            let down = -section_up(axis.cross(*start_dir)) * RAIL_RADIUS;
            let tube = radius + down.dot(*start_dir);
            let torus =
                circ_torus_solid(tube - RAIL_RADIUS, tube + RAIL_RADIUS, sweep.to_degrees())?;
            let x = *start_dir;
            let z = *axis;
            let y = z.cross(x);
            let frame = DMat4::from_translation(*centre + *axis * down.dot(*axis))
                * DMat4::from_mat3(DMat3::from_cols(x, y, z));
            Ok(placed(&torus, frame))
        }
    }
}

/// 沿方向 `d` 的截面「上」向：路径所在竖直面里、垂直于路径、朝上（水平段就是 +Z）。竖直段无从谈起，取 +X。
fn section_up(d: DVec3) -> DVec3 {
    (DVec3::Z - d * d.z).try_normalize().unwrap_or(DVec3::X)
}

/// 斜接面（法向 `normal` = unit(d_in + d_out)）对轴向 `d` 的 tan(θ/2)：截面上离拐点每 1 mm 要多留的长度。
fn mitre_tan_half(normal: DVec3, d: DVec3) -> f64 {
    let cos_half = normal.dot(d).abs().clamp(0.0, 1.0);
    let sin_half = (1.0 - cos_half * cos_half).max(0.0).sqrt();
    if cos_half > 1e-9 {
        (sin_half / cos_half).min(20.0)
    } else {
        0.0
    }
}

/// 默认布局式踢脚板：板厚 4、离路径 10 起高 100，截面跟着路径的「上」向（斜段在竖直方向上是
/// 10 / cosθ 起、高 100 / cosθ），拐角一律加长后按过拐点的角平分面切（`FRAD` 的圆角只给栏杆，踢脚板不跟弧）；开口路径的两端是过端点的竖直面
/// （法向取路径的水平向），不是截面——斜段的板顶不往端点外俯出去。E3D TTY 导出：aps250112
/// `/EQUIP_SUPPORT/SF_001` 平台段 ±2 × [10, 110]，斜段 [12.53, 137.74]（cosθ = 0.7986），平台与斜段的斜接在
/// 顶边上退 110·tan(θ/2)；aps250110 `/B01_LEVEL_03_ACCESS/SF_001` 的单段斜扶手，板在两端立柱之外一点不露。
fn kickplate_segment_solid(segment: &PathSegment) -> Result<Manifold, ModelError> {
    band_segment_solid(
        segment,
        &BandSection {
            across: 0.0,
            half_thickness: KICKPLATE_THICKNESS / 2.0,
            low: KICKPLATE_LIFT,
            high: KICKPLATE_LIFT + KICKPLATE_HEIGHT,
            open_end: OpenEnd::Vertical,
        },
        "handrail kickplate",
    )
}

/// 没有端向（`mitre_* = None`）的开口端怎么收。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum OpenEnd {
    /// 过端点的竖直面，法向取路径的水平向（踢脚板：斜段的板顶不往端点外俯出去）。
    Vertical,
    /// 垂直于路径的平头（梯段斜梁：aps250209 `STRFLT 2 of /A01-AREA05-BASE-FL-STAIRS` 的底端）。
    Perpendicular,
}

/// 沿路径直段扫的矩形「带」：截面在路径的横向（`d × up`）上居中于 `across`、半宽 `half_thickness`，沿截面「上」向
/// （[`section_up`]）从 `low` 到 `high`（负值在路径下方）。踢脚板、梯段的斜梁与平台板都是它。
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct BandSection {
    pub across: f64,
    pub half_thickness: f64,
    pub low: f64,
    pub high: f64,
    pub open_end: OpenEnd,
}

/// 一段直路径上的带：斜接端加长到切得到角平分面（或端向面）再切；没有面的开口端按 [`OpenEnd`] 收。
/// 只收直段：踢脚板与梯段都按没有倒角的路径排（E3D 在拐点斜接，不跟 `FRAD` 的弧）。
pub(crate) fn band_segment_solid(
    segment: &PathSegment,
    band: &BandSection,
    what: &'static str,
) -> Result<Manifold, ModelError> {
    let PathSegment::Line {
        start,
        end,
        mitre_start,
        mitre_end,
    } = segment
    else {
        return Err(ModelError::Solid(SolidError::Degenerate {
            what,
            detail: "bands are built from the unfilleted path; E3D mitres them at the corner"
                .to_string(),
        }));
    };
    let d = (*end - *start).normalize();
    let up = section_up(d);
    let across = d.cross(up);
    let half = across * band.half_thickness;
    let shift = across * band.across;
    let reach = band.low.abs().max(band.high.abs());
    let horizontal = (d.x * d.x + d.y * d.y).sqrt();
    let level = match band.open_end {
        OpenEnd::Vertical => DVec3::new(d.x, d.y, 0.0).try_normalize(),
        OpenEnd::Perpendicular => None,
    };
    // 斜接端加长到切得到角平分面；竖切的开口端加长到离路径最远的那条边也越过端点的竖直面
    // （斜段上这条边比路径水平靠后 reach·sinθ）；平头的开口端不加长。
    let open_end = match (band.open_end, horizontal > 1e-9) {
        (OpenEnd::Vertical, true) => reach * d.z.abs() / horizontal,
        _ => 0.0,
    };
    let extension = |normal: &Option<DVec3>| -> f64 {
        normal.map_or(open_end, |n| reach * mitre_tan_half(n, d))
    };
    let first = *start - d * extension(mitre_start) + shift;
    let last = *end + d * extension(mitre_end) + shift;
    let low = up * band.low;
    let high = up * band.high;
    let points = [
        first - half + low,
        first + half + low,
        first - half + high,
        first + half + high,
        last - half + low,
        last + half + low,
        last - half + high,
        last + half + high,
    ];
    let points = points.map(|point| point.to_array());
    let mut solid = Manifold::hull_pts(&points);
    match (mitre_start, level) {
        (Some(normal), _) => solid = cut_by_plane(&solid, *start, *normal),
        (None, Some(level)) => solid = cut_by_plane(&solid, *start, level),
        (None, None) => {}
    }
    match (mitre_end, level) {
        (Some(normal), _) => solid = cut_by_plane(&solid, *end, -*normal),
        (None, Some(level)) => solid = cut_by_plane(&solid, *end, -level),
        (None, None) => {}
    }
    if solid.is_empty() || solid.num_tri() == 0 {
        return Err(ModelError::Solid(SolidError::EmptyManifold { what }));
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
    Solid {
        solid: Manifold,
        /// E3D 逐节写时的记录单位（局部系，已含 `ZOFF`）：直段一节一块（整张截面），`FRAD` 圆角弧每个 GMSS 成员一块。
        /// **只在路径带圆角弧时才给**：aps250112 `RAIL 1 of RAILSET 1 of /EL(+)104880_TOS_PLATE/HR_003`（3 直段 + 2 弧，管截面是两个
        /// SANN 半环）E3D 是 7 条面片——直段一条挤出（两个壳在同一条里），弧每个成员各走一次 `gm_CreateRevolution`、各一条；
        /// 只有直段的折线（拐角斜接）与带 `CURTYP THRU` 弧的路径 E3D 整根一条（aps250110 `/B01_LEVEL_01_HANDRAILS/HR_001` 3 节、
        /// `/BUILDING_01_LEVEL_03/HR_001` 4 直段 + 1 THRU 弧都是一条，aps250112 `RAIL 1 of RAIL 1 of /EL(+)102470_TOS_PLATE/HR_001`
        /// 一节直段 + 一只 0.085 mm 的退化圆角也是），那时这里是空表。
        pieces: Vec<Manifold>,
    },
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
///    路径曲线的起止下标，取其中一串**连续**的曲线，闭合路径允许绕回去（[`curve_run`]）；两端带
///    `SZDI` / `EZDI` 时从参照点量进去截一截（[`run_segments`]）。
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
    let start_dist = el.get_double("SZDI")?.unwrap_or(0.0);
    let end_dist = el.get_double("EZDI")?.unwrap_or(0.0);
    let run = run_segments(&layout, start, start_dist, end, end_dist);
    if run.is_empty() {
        return Ok(RailBuild::CentrelineOnly(format!(
            "STAREF 点 {start}（SZDI {start_dist}）→ ENDREF 点 {end}（EZDI {end_dist}）在 {} 条曲线（{}）上取不出一段，E3D 同样一段不画",
            layout.edge_count,
            if layout.closed { "闭合" } else { "开口" }
        )));
    }
    let mut run = run;
    let mut edges: Vec<usize> = run.iter().map(|(edge, _)| *edge).collect();
    edges.dedup();
    // 要扫的这一串边里夹着别的路径段（`CURVE`/`RSECT`/`EXTGEO`）：`CURTYP THRU` 的圆弧换掉那条边的直段
    // （[`swap_in_through_arc`]）；其余本期不认，整根不画——照直段画出来的是一根形状不对的杆，而它会
    // 顶着这只 RAIL 的 refno 进 `elements.json`，比缺一件更难发现。与 GENSEC 带弧交 `CentrelineOnly` 同口径。
    let mut arc_mitres: Vec<(usize, DVec3, DVec3)> = Vec::new();
    for (original, why) in &raw.unwired {
        let Some(edge) = layout.kept.iter().position(|kept| kept == original) else {
            continue;
        };
        if !edges.contains(&edge) {
            continue;
        }
        let swapped = if arc_mitres.iter().any(|(arc_edge, _, _)| *arc_edge == edge) {
            None
        } else {
            swap_in_through_arc(&rpath, &raw, &layout, edge, &mut run)?
        };
        let Some((start_mitre, end_mitre)) = swapped else {
            return Ok(RailBuild::CentrelineOnly(format!(
                "要扫的第 {edge} 段上有本期不认的路径段：{why}"
            )));
        };
        arc_mitres.push((edge, start_mitre, end_mitre));
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
    for (index, segment) in &run {
        let PathSegment::Line {
            start,
            end,
            mitre_start,
            mitre_end,
        } = segment
        else {
            // `FRAD` 圆角弧：与 GENSEC 的 SPINE 弧同一件事——截面绕弧心回转。
            // 借 `spine_swept_solid` 单段跑一趟，省得把回转那段再抄一遍；
            // 弧与两邻边相切，不需要斜接面。`CURTYP THRU` 的弧与邻边成角，两端的斜接面交给
            // `DRNS` / `DRNE`（过端点、法向 = 两侧切向之和，与直段的斜接面是同一张）。
            let mut spine = arc_as_spine(segment);
            if let Some((_, start_mitre, end_mitre)) =
                arc_mitres.iter().find(|(arc_edge, _, _)| arc_edge == index)
            {
                spine.drns = *start_mitre;
                spine.drne = *end_mitre;
            }
            // 弧逐 GMSS 成员各扫一块：E3D 对每个成员各走一次回转、各写一条面片（见 [`RailBuild::Solid`]）；直段是整张截面一次挤出。
            let mut swept_any = false;
            for member in &profile.members {
                let single = SectionProfile {
                    members: vec![member.clone()],
                    ..profile.clone()
                };
                if let Some(body) = spine_swept_solid(&single, &spine, bang_deg, notes)? {
                    pieces.push(body);
                    swept_any = true;
                }
            }
            if !swept_any {
                notes.push(format!("路径第 {index} 段（圆角弧）扫不出实体"));
            }
            continue;
        };
        let placement = match placement_between(&refno, &noun, *start, *end, bang_deg, &profile) {
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
        edges,
        pieces.len(),
        profile.justification_line
    ));
    // `ZOFF` 与布局式的 `RAILHE` 同一个意思：整根栏杆沿属主局部 +Z 抬起来
    // （语料里 RAIL 的 ZOFF 恰好就是属主的 `RAILHE` 两个值）。
    let lift = |body: &Manifold| {
        if z_offset == 0.0 {
            body.clone()
        } else {
            body.translate(0.0, 0.0, z_offset)
        }
    };
    // 只有 `FRAD` 圆角弧才让 E3D 逐节写：`CURTYP THRU` 的弧是路径自己的一段，整根仍是一条（aps250110
    // `/BUILDING_01_LEVEL_03/HR_001`：4 直段 + 1 段 R≈14 m 的 THRU 弧，E3D 一条 326 个多边形）；退化的圆角（弧长不到 1 mm，
    // aps250112 `/EL(+)102470_TOS_PLATE/HR_001` 的 RAILSET 1 末端那一只 0.085 mm）E3D 不写，也不算。
    let has_arc = run.iter().any(|(index, segment)| match segment {
        PathSegment::Arc { radius, sweep, .. } => {
            radius * sweep.abs() >= 1.0
                && !arc_mitres.iter().any(|(arc_edge, _, _)| arc_edge == index)
        }
        PathSegment::Line { .. } => false,
    });
    Ok(RailBuild::Solid {
        solid: lift(&solid),
        pieces: if has_arc {
            pieces.iter().map(lift).collect()
        } else {
            Vec::new()
        },
    })
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

/// 详图式扶手成员（立柱 `HRPOST`、端头 `HRTERM`、门 `HRGATE`、面板 `HRPANE`）在属主 RPATH 上的站位，
/// HANDRA 局部系（[`member_station`]）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PathStation {
    pub position: DVec3,
    /// 站点处路径按点序的走向，单位向量。
    pub travel: DVec3,
    /// `POSREF` 是开口路径的末点：`ZDIS` 沿进入段往回量。
    pub at_open_end: bool,
}

/// 从 `POSREF` 点出发、沿离开它的那条边走 `ZDIS` 得到的站位。
///
/// - 起量点是这条边直段的起点：拐点上 `FRAD` 圆角的切长（[`path_layout`] 的 `FRAD·tan(θ/2)`）先让掉。aps250112
///   `/EL(+)100000/HR_001`、`HR_002` 的 5 根立柱挂在 `FRAD 100` 的直角拐点上，E3D TTY 导出的站位比从拐点量远
///   100 mm；aps250110 拐点的 `FRAD` 全是 0，两种量法一样。
/// - 闭合路径（`CLOSEF`）末点那条边绕回首点：aps250110 `/BUILDING_01_LEVEL_03/HR_001` 挂在末点的 9 根立柱落在
///   末点到首点那 8385 mm 上。开口路径的末点没有离开的边，沿进入段往回量。
/// - 边上夹着 `CURTYP THRU` 的 `CURVE` 时站位在过三点的圆弧上、按弧长量（同一只扶手起点那段弧上的 16 根）。
/// - 越过直段（或圆弧）终点的 `ZDIS` 夹在终点上。
pub(crate) fn member_station(
    handrail: &DbElement,
    point: &DbElement,
    zdis: f64,
) -> Result<Option<PathStation>, ModelError> {
    let Some(rpath) = first_rpath(handrail)? else {
        return Ok(None);
    };
    let raw = path_points(&rpath)?;
    let target = point.refno().to_string();
    let Some(original) = raw.points.iter().position(|(refno, _, _)| *refno == target) else {
        return Ok(None);
    };
    let closed = handrail.get_bool("CLOSEF")?.unwrap_or(false);
    let mut notes = Vec::new();
    let layout = path_layout(
        &raw.points
            .iter()
            .map(|(_, pos, fillet)| (*pos, *fillet))
            .collect::<Vec<_>>(),
        closed,
        &mut notes,
    );
    if layout.edge_count == 0 {
        return Ok(None);
    }
    // 与前一点重合、去重时被丢掉的点，站在它前面那个保留点上。
    let Some(index) = layout.kept.iter().rposition(|kept| *kept <= original) else {
        return Ok(None);
    };
    let (edge, at_open_end) = if index < layout.edge_count {
        (index, false)
    } else {
        (index - 1, true)
    };
    let count = layout.kept.len();
    let vertex = |i: usize| raw.points[layout.kept[i % count]].1;
    let (a, b) = (vertex(edge), vertex(edge + 1));
    if let Some(arc) = through_arc(&rpath, layout.kept[edge], a, b)? {
        let along = if at_open_end { arc.length - zdis } else { zdis };
        let (position, travel) = arc.at(along.clamp(0.0, arc.length));
        return Ok(Some(PathStation {
            position,
            travel,
            at_open_end,
        }));
    }
    let Some(travel) = (b - a).try_normalize() else {
        return Ok(None);
    };
    let (start, end) = layout
        .segments
        .iter()
        .find_map(|(e, segment)| match segment {
            PathSegment::Line { start, end, .. } if *e == edge => Some((*start, *end)),
            _ => None,
        })
        .unwrap_or((a, b));
    let run = zdis.min((end - start).length());
    Ok(Some(PathStation {
        position: if at_open_end {
            end - travel * run
        } else {
            start + travel * run
        },
        travel,
        at_open_end,
    }))
}

/// 一条边上 `CURTYP THRU` 的圆弧：过边的两端与 `CURVE` 的 `POS`。
struct ThroughArc {
    centre: DVec3,
    radius: f64,
    /// 起点绕它正转到终点。
    axis: DVec3,
    start: DVec3,
    length: f64,
}

impl ThroughArc {
    /// 从起点沿弧走 `distance`：站点与该处切向。
    fn at(&self, distance: f64) -> (DVec3, DVec3) {
        let radial = crate::section::rotate_about(
            self.start - self.centre,
            self.axis,
            distance / self.radius,
        );
        (self.centre + radial, self.axis.cross(radial).normalize())
    }
}

/// 夹在第 `after` 个 `POINTR`（原始下标）与下一个之间的 `CURVE`，`CURTYP THRU` 时给出 `a → b` 那段圆弧；
/// 没有、类型不同或三点共线都是 `None`（照直段算）。
fn through_arc(
    rpath: &DbElement,
    after: usize,
    a: DVec3,
    b: DVec3,
) -> Result<Option<ThroughArc>, ModelError> {
    let mut seen = 0usize;
    let mut curve = None;
    for member in rpath.members()? {
        match member.element_type()?.as_str() {
            "POINTR" => seen += 1,
            "CURVE" if seen == after + 1 => {
                curve = Some(member);
                break;
            }
            _ => {}
        }
    }
    let Some(curve) = curve else {
        return Ok(None);
    };
    if curve.get_string("CURTYP")?.as_deref() != Some("THRU") {
        return Ok(None);
    }
    let Some(through) = curve.get_position("POS")?.map(DVec3::from_array) else {
        return Ok(None);
    };
    let normal = (through - a).cross(b - through);
    if normal.length_squared() <= 1e-12 {
        return Ok(None);
    }
    let ab = through - a;
    let ac = b - a;
    let centre = a
        + (normal.cross(ab) * ac.length_squared() + ac.cross(normal) * ab.length_squared())
            / (2.0 * normal.length_squared());
    let radial = a - centre;
    let radius = radial.length();
    if radius <= 1e-9 {
        return Ok(None);
    }
    let axis = normal.normalize();
    let mut sweep = axis
        .dot(radial.cross(b - centre))
        .atan2(radial.dot(b - centre));
    if sweep < 0.0 {
        sweep += std::f64::consts::TAU;
    }
    Ok(Some(ThroughArc {
        centre,
        radius,
        axis,
        start: a,
        length: radius * sweep,
    }))
}

/// 一条夹着 `CURTYP THRU` 的 `CURVE` 的边：把 [`path_layout`] 当直段排的那一段换成过三点的圆弧，
/// 两邻直段在公共拐点上的斜接面改用弧的切向。交回弧两端的斜接面法向（不切的是零向量）。
///
/// 拐点上带 `FRAD` 圆角、或这条边被 `SZDI` / `EZDI` 截过时交 `None`（本期不认）：aps250110
/// `/BUILDING_01_LEVEL_03/HR_001` 闭合路径首段那段弧（R ≈ 14828）两端都是尖角，E3D TTY 导出的
/// `RAIL 1` / `RAIL 2` 绕过它一圈到门洞。
fn swap_in_through_arc(
    rpath: &DbElement,
    raw: &RawPath,
    layout: &PathLayout,
    edge: usize,
    run: &mut [(usize, PathSegment)],
) -> Result<Option<(DVec3, DVec3)>, ModelError> {
    let count = layout.kept.len();
    let vertex = |i: usize| raw.points[layout.kept[i % count]].1;
    let (a, b) = (vertex(edge), vertex(edge + 1));
    let Some(arc) = through_arc(rpath, layout.kept[edge], a, b)? else {
        return Ok(None);
    };
    let Some(slot) = run.iter().position(|(e, segment)| {
        *e == edge
            && matches!(segment, PathSegment::Line { start, end, .. }
                if (*start - a).length() <= 1e-6 && (*end - b).length() <= 1e-6)
    }) else {
        return Ok(None);
    };
    let radial = a - arc.centre;
    let start_tangent = arc.axis.cross(radial).normalize();
    let end_tangent = arc.axis.cross(b - arc.centre).normalize();
    // 拐点两侧的行进切向 → 斜接面法向；直通、近乎折返、开口端点都不切。
    let mitre = |d_in: DVec3, d_out: DVec3| -> Option<DVec3> {
        let turn = d_in.dot(d_out).clamp(-1.0, 1.0).acos();
        if turn <= STRAIGHT_JOINT_RAD || turn >= MAX_MITRE_RAD {
            return None;
        }
        (d_in + d_out).try_normalize()
    };
    let has_previous = layout.closed || edge > 0;
    let has_next = layout.closed || edge + 1 < layout.edge_count;
    let start_mitre = if has_previous {
        (a - vertex(edge + count - 1))
            .try_normalize()
            .and_then(|d_in| mitre(d_in, start_tangent))
    } else {
        None
    };
    let end_mitre = if has_next {
        (vertex(edge + 2) - b)
            .try_normalize()
            .and_then(|d_out| mitre(end_tangent, d_out))
    } else {
        None
    };
    let previous = (edge + layout.edge_count - 1) % layout.edge_count;
    let next = (edge + 1) % layout.edge_count;
    for (e, segment) in run.iter_mut() {
        if let PathSegment::Line {
            start,
            end,
            mitre_start,
            mitre_end,
        } = segment
        {
            if has_previous && *e == previous && (*end - a).length() <= 1e-6 {
                *mitre_end = start_mitre;
            }
            if has_next && *e == next && (*start - b).length() <= 1e-6 {
                *mitre_start = end_mitre;
            }
        }
    }
    let sweep = arc.length / arc.radius;
    run[slot].1 = PathSegment::Arc {
        centre: arc.centre,
        radius: arc.radius,
        start_dir: radial / arc.radius,
        axis: arc.axis,
        sweep,
    };
    Ok(Some((
        start_mitre.unwrap_or(DVec3::ZERO),
        end_mitre.unwrap_or(DVec3::ZERO),
    )))
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

/// HANDRA（或 STRFLT）的第一个 `RPATH` 成员（`sub_109F7150` 把它存进 `+128`）。
pub(crate) fn first_rpath(handrail: &DbElement) -> Result<Option<DbElement>, ModelError> {
    for member in handrail.members()? {
        if member.element_type()? == "RPATH" {
            return Ok(Some(member));
        }
    }
    Ok(None)
}

/// 一条 `RPATH` 读出来的路径。
pub(crate) struct RawPath {
    /// `POINTR` 的 refno、位置、`FRAD` 倒角半径。
    pub(crate) points: Vec<(String, DVec3, f64)>,
    /// 本期没接的那些路径段：`(夹在它前面的那个点在 points 里的下标, 说明)`。
    ///
    /// `createCurvesFromPath` 除点串外还认三种：`CURVE`/`RSECT` 走 vtbl+24 出圆弧、
    /// `EXTGEO` 走 `sub_105D13B0` 出外部几何段。它们夹在两个 `POINTR` 之间，
    /// **把那条边从直段换成别的东西**——只记一条 note 再照直段画，画出来的是一根
    /// 形状不对的杆；所以要把边号一并交出去，由调用方决定这一段还画不画。
    pub(crate) unwired: Vec<(usize, String)>,
    /// `RPATH` 的 `DRNS` / `DRNE`（开口路径两端的端向）；未设读成零向量。
    pub(crate) drns: DVec3,
    pub(crate) drne: DVec3,
}

/// 读一条 `RPATH` 的点串（HANDRA 的，也是梯段 STRFLT 自己那条的）。
pub(crate) fn path_points(rpath: &DbElement) -> Result<RawPath, ModelError> {
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
    let direction = |name: &str| -> Result<DVec3, ModelError> {
        Ok(rpath
            .get_direction(name)?
            .map(DVec3::from_array)
            .filter(|v| v.is_finite())
            .unwrap_or(DVec3::ZERO))
    };
    Ok(RawPath {
        points,
        unwired,
        drns: direction("DRNS")?,
        drne: direction("DRNE")?,
    })
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

/// `STAREF` + `SZDI` → `ENDREF` + `EZDI` 之间要扫的段，按路径顺序。
///
/// 两端的距离与立柱的 `ZDIS` 同一口径（[`member_station`]）：从参照点那条边直段的起点量。都是 0 时就是
/// [`curve_run`] 那串边的全部段（圆角弧随它所在边的尾部）；有距离时首段直段从 `SZDI` 处截起、
/// 尾部补上 `ENDREF` 那条边的直段到 `EZDI` 处，截口不切斜接面。`STAREF == ENDREF` 是参照点出发那条边
/// 上的一截：aps250112 `/EL(+)104880_TOS*/HR_00x` 的 10 根栏杆（`SZDI 450 → EZDI 2196.4` 之类），E3D TTY
/// 导出的杆长正是两者之差。
pub fn run_segments(
    layout: &PathLayout,
    start: usize,
    start_dist: f64,
    end: usize,
    end_dist: f64,
) -> Vec<(usize, PathSegment)> {
    let line_of = |edge: usize| {
        layout
            .segments
            .iter()
            .find_map(|(e, segment)| match segment {
                PathSegment::Line { start, end, .. } if *e == edge => Some((*start, *end, segment)),
                _ => None,
            })
    };
    // 一条边的直段从起点量 `from..to` 那一截；截到的端口不切斜接面。
    let cut = |edge: usize, from: f64, to: Option<f64>| -> Option<PathSegment> {
        let (a, b, segment) = line_of(edge)?;
        let PathSegment::Line {
            mitre_start,
            mitre_end,
            ..
        } = segment
        else {
            return None;
        };
        let length = (b - a).length();
        let direction = (b - a).try_normalize()?;
        let from = from.clamp(0.0, length);
        let to = to.map_or(length, |to| to.clamp(0.0, length));
        if to - from <= 1e-6 {
            return None;
        }
        Some(PathSegment::Line {
            start: a + direction * from,
            end: a + direction * to,
            mitre_start: if from > 0.0 { None } else { *mitre_start },
            mitre_end: if to < length { None } else { *mitre_end },
        })
    };
    if start_dist == 0.0 && end_dist == 0.0 {
        return curve_run(start, end, layout.edge_count, layout.closed)
            .into_iter()
            .flat_map(|edge| {
                layout
                    .segments
                    .iter()
                    .filter(move |(e, _)| *e == edge)
                    .cloned()
            })
            .collect();
    }
    if start == end {
        return cut(start, start_dist, Some(end_dist))
            .map(|segment| vec![(start, segment)])
            .unwrap_or_default();
    }
    let run = curve_run(start, end, layout.edge_count, layout.closed);
    let mut out = Vec::new();
    for (step, edge) in run.iter().enumerate() {
        for (e, segment) in layout.segments.iter().filter(|(e, _)| e == edge) {
            if step == 0 && start_dist > 0.0 && matches!(segment, PathSegment::Line { .. }) {
                if let Some(first) = cut(*edge, start_dist, None) {
                    out.push((*e, first));
                }
                continue;
            }
            out.push((*e, segment.clone()));
        }
    }
    if !run.is_empty()
        && end_dist > 0.0
        && let Some(last) = cut(end, 0.0, Some(end_dist))
    {
        out.push((end, last));
    }
    out
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

    /// aps250209 梯段扶手的路径：平台段 302.45 再上斜段，`RPATH` 的 `DRNS = DRNE = +Y`。起点端向与平台段平行→仍是
    /// 平头；终点端向与斜段成角→末段尾部换成过终点、法向 +Y（沿行进方向）的竖直面；拐角那张斜接面不动。
    /// 零端向和闭合路径一张都不加。
    #[test]
    fn open_ends_follow_the_rpath_end_directions_when_they_are_oblique() {
        let mut notes = Vec::new();
        let points = [
            (DVec3::new(0.0, 0.0, 0.0), 0.0),
            (DVec3::new(0.0, 302.45, 0.0), 0.0),
            (DVec3::new(0.0, 2929.95, 1980.0), 0.0),
        ];
        let mut segments = build_path(&points, false, &mut notes);
        assert_eq!(segments.len(), 2, "{segments:?}");
        let corner = match &segments[1] {
            PathSegment::Line { mitre_start, .. } => mitre_start.expect("corner mitre"),
            other => panic!("{other:?}"),
        };
        apply_end_directions(&mut segments, false, DVec3::Y, DVec3::Y, &mut notes);
        match (&segments[0], &segments[1]) {
            (
                PathSegment::Line {
                    mitre_start: None,
                    mitre_end: Some(_),
                    ..
                },
                PathSegment::Line {
                    mitre_start: Some(n_corner),
                    mitre_end: Some(n_end),
                    ..
                },
            ) => {
                assert!((*n_corner - corner).length() < 1e-12, "{n_corner}");
                assert!((*n_end - DVec3::Y).length() < 1e-12, "{n_end}");
            }
            other => panic!("{other:?}"),
        }
        assert_eq!(notes.len(), 1, "{notes:?}");
        assert!(notes[0].contains("DRNE"), "{notes:?}");

        // 零端向（aps250110 SF_002 那样）：两端都是平头。
        let mut plain = build_path(&points, false, &mut Vec::new());
        apply_end_directions(&mut plain, false, DVec3::ZERO, DVec3::ZERO, &mut Vec::new());
        match &plain[1] {
            PathSegment::Line {
                mitre_end: None, ..
            } => {}
            other => panic!("{other:?}"),
        }
        // 闭合路径没有端。
        let square = [
            (p(0.0, 0.0), 0.0),
            (p(1000.0, 0.0), 0.0),
            (p(1000.0, 1000.0), 0.0),
            (p(0.0, 1000.0), 0.0),
        ];
        let mut closed = build_path(&square, true, &mut Vec::new());
        let before = closed.clone();
        apply_end_directions(&mut closed, true, DVec3::Z, DVec3::Z, &mut Vec::new());
        assert_eq!(closed, before);
    }

    /// 端向切过的栏杆实体：斜段终点 (0, 2929.95, 1980)、`DRNE = +Y`，整根栏杆在 y 上正好止于终点，而平头会多伸
    /// 截面圆下坠 20·sinθ 再加端环半径在 y 上的投影 20·sinθ（θ = 37° 时共 24.07 mm）。
    #[test]
    fn an_end_direction_cut_stops_the_rail_exactly_at_the_path_end() {
        let start = DVec3::new(0.0, 302.45, 0.0);
        let end = DVec3::new(0.0, 2929.95, 1980.0);
        let plain = rail_segment_solid(&PathSegment::Line {
            start,
            end,
            mitre_start: None,
            mitre_end: None,
        })
        .unwrap();
        let cut = rail_segment_solid(&PathSegment::Line {
            start,
            end,
            mitre_start: None,
            mitre_end: Some(DVec3::Y),
        })
        .unwrap();
        let y_max = |solid: &Manifold| solid.bounding_box().expect("rail").max()[1];
        let d = (end - start).normalize();
        let overhang = 2.0 * RAIL_RADIUS * d.z;
        assert!(
            (y_max(&plain) - (end.y + overhang)).abs() < 0.05,
            "{}",
            y_max(&plain)
        );
        assert!((y_max(&cut) - end.y).abs() < 0.05, "{}", y_max(&cut));
    }

    /// FRAD 100 的直角拐角：栏杆走弧，踢脚板不走——按没有倒角的路径排成两段直板，在过拐点 (1000, 0) 的角平分面上
    /// 斜接：第一段的外边（y = −2）画到 x = 1002、内边（y = +2）停在 998，板高 10..110。此前弧段在这里被拒，整只扶手都不出。
    /// 竖直面里的内弯拐角（平台接斜段，弧心在板一侧、FRAD < 板顶 110）同一条规则，所以不存在「截面跨过弧心」的问题。
    #[test]
    fn a_kickplate_is_mitred_at_the_corner_instead_of_following_the_fillet_arc() {
        let points = [
            (p(0.0, 0.0), 0.0),
            (p(1000.0, 0.0), 100.0),
            (p(1000.0, 800.0), 0.0),
        ];
        let unfilleted: Vec<(DVec3, f64)> = points.iter().map(|&(point, _)| (point, 0.0)).collect();
        let plates = build_path(&unfilleted, false, &mut Vec::new());
        assert_eq!(plates.len(), 2, "{plates:?}");
        assert!(
            plates
                .iter()
                .all(|segment| matches!(segment, PathSegment::Line { .. }))
        );
        let first = kickplate_segment_solid(&plates[0]).unwrap();
        let bb = first.bounding_box().unwrap();
        let (lo, hi) = (bb.min(), bb.max());
        for (got, want) in lo.iter().zip([0.0, -2.0, 10.0]) {
            assert!((got - want).abs() < 0.05, "min {lo:?}");
        }
        for (got, want) in hi.iter().zip([1002.0, 2.0, 110.0]) {
            assert!((got - want).abs() < 0.05, "max {hi:?}");
        }

        // 竖直面里的内弯：平台段接斜段（aps250209 `/A02-AREA07-HANDRAILS`），板底 10 与板顶 110 都在角平分面上相交。
        let stair = [
            (DVec3::new(0.0, 0.0, 0.0), 0.0),
            (DVec3::new(0.0, -1404.55, 0.0), 0.0),
            (DVec3::new(0.0, -3950.0, 2000.0), 0.0),
        ];
        let plates = build_path(&stair, false, &mut Vec::new());
        let flat = kickplate_segment_solid(&plates[0])
            .unwrap()
            .bounding_box()
            .unwrap();
        let d_out = (stair[2].0 - stair[1].0).normalize();
        let normal = (DVec3::NEG_Y + d_out).normalize();
        // 板底（z = 10）的远端落在斜接面上：n.y (y + 1404.55) + n.z·10 = 0 → y = −1401.09（E3D 导出里板底面的末端）；
        // 板顶收到 −1366.49，所以整块板的 y 最小就是板底那一点。
        let bottom_end = -1404.55 + normal.z * KICKPLATE_LIFT / normal.y.abs();
        assert!((bottom_end + 1401.09).abs() < 0.01, "{bottom_end}");
        assert!(
            (flat.min()[1] - bottom_end).abs() < 0.05,
            "{:?} vs {bottom_end}",
            flat.min()
        );
        assert!((flat.max()[2] - 110.0).abs() < 0.05 && (flat.min()[2] - 10.0).abs() < 0.05);
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

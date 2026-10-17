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

use crate::elmodl::ModelError;
use crate::solid::{SolidError, cut_by_plane, placed};
use e3d_io::db_element::{DbElement, DbElementError};
use e3d_io::db_param_env::DbElementParamEnv;
use e3d_io::record::descriptor::DescriptorValue;
use glam::{DMat3, DMat4, DQuat, DVec3};
use manifold_csg::Manifold;

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
                Resolved::Segment { a, b, pkdi, zdis } => {
                    // 非直类件带非零 ZDIS/PKDI 时 Core 在缩减段上量距（A.PPOS[2] 基线）；
                    // 零偏移仍落在原始角点。RATTA/297、RNODE/315 实测锚点证实。
                    let point = if pkdi.abs() > crate::rpath::ATTR_TOL
                        || zdis.abs() > crate::rpath::ATTR_TOL
                    {
                        crate::rpath::filleted_point(&path, a, b, pkdi, zdis, zoff, 0.0)
                    } else {
                        straight_point(&path, a, b, pkdi, zdis, zoff, 0.0)
                    };
                    (point.position, a)
                }
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

pub fn geometry_enabled() -> bool {
    std::env::var("E3D_MODEL_CABLE_GEOMETRY_EXPERIMENTAL").as_deref() == Ok("1")
}

pub(crate) fn cable_outer_diameter(cable: &DbElement) -> Result<f64, DbElementError> {
    let entry = reference(cable, "SPRE")?
        .ok_or_else(|| crate::rpath::failed(cable, "PROPRE OUTD", "CABLE 没有 SPRE"))?;
    let part = match entry.element_type()?.as_str() {
        "TABITE" => reference(&entry, "PRTREF")?,
        "GPART" => Some(entry),
        noun => {
            return Err(crate::rpath::failed(
                cable,
                "PROPRE OUTD",
                format!("未支持的规格入口 {noun}"),
            ));
        }
    }
    .ok_or_else(|| crate::rpath::failed(cable, "PROPRE OUTD", "TABITE 没有 PRTREF"))?;
    let properties = reference(&part, "CMPR")?
        .ok_or_else(|| crate::rpath::failed(cable, "PROPRE OUTD", "GPART 没有 CMPR"))?;
    if properties.element_type()? != "CABD" {
        return Err(crate::rpath::failed(
            cable,
            "PROPRE OUTD",
            "CMPR 不是 CABD，尚未支持该属性表",
        ));
    }
    let env = DbElementParamEnv::new(Some(cable.clone()), Some(properties.clone()));
    let diameter = properties.get_number("OUTD", &env)?;
    match diameter.value() {
        Some(value) if value.is_finite() && value >= 0.0 => Ok(value),
        _ => Err(crate::rpath::failed(
            cable,
            "PROPRE OUTD",
            format!("OUTD 未解或不是有效直径：{diameter:?}"),
        )),
    }
}

/// `CABD.MINBEN`：电缆最小弯曲半径（原生 `MDR_ConnectionBend` 的 ctor 经
/// `PROPRE`/`MINBEN` 限定符取值）。语料样本 = 6 × `OUTD`。
/// 取不到时按 6 倍外径兜底（与样本一致），仍显式记录来源。
pub(crate) fn cable_min_bend_radius(cable: &DbElement, diameter: f64) -> f64 {
    let radius = (|| -> Result<Option<f64>, DbElementError> {
        let entry = match reference(cable, "SPRE")? {
            Some(entry) => entry,
            None => return Ok(None),
        };
        let part = match entry.element_type()?.as_str() {
            "TABITE" => reference(&entry, "PRTREF")?,
            "GPART" => Some(entry),
            _ => None,
        };
        let Some(part) = part else { return Ok(None) };
        let Some(properties) = reference(&part, "CMPR")? else {
            return Ok(None);
        };
        if properties.element_type()? != "CABD" {
            return Ok(None);
        }
        let env = DbElementParamEnv::new(Some(cable.clone()), Some(properties.clone()));
        Ok(properties.get_number("MINBEN", &env)?.value())
    })();
    match radius {
        Ok(Some(value)) if value.is_finite() && value >= 0.0 => value,
        _ => diameter * 6.0,
    }
}

fn cable_code(el: &DbElement, name: &str) -> Result<u32, DbElementError> {
    match crate::rpath::optional(el.attribute(name))? {
        None | Some(DescriptorValue::Unset) => Ok(0),
        Some(DescriptorValue::Word { raw, .. }) => Ok(raw),
        Some(DescriptorValue::Int(value)) if value >= 0 => Ok(value as u32),
        value => Err(crate::rpath::failed(
            el,
            name,
            format!("不是非负代码：{value:?}"),
        )),
    }
}

fn route_station(path: &RoutePath, anchor: &AttachmentAnchor) -> Result<f64, DbElementError> {
    let index = anchor.station;
    if index + 1 == path.points.len() {
        return Ok(index as f64);
    }
    let delta = path.points[index + 1].pos - path.points[index].pos;
    let fraction = (anchor.position - path.points[index].pos).dot(delta) / delta.length_squared();
    if !fraction.is_finite() || !(-1e-8..=1.0 + 1e-8).contains(&fraction) {
        return Err(crate::rpath::failed(
            &anchor.cwbran,
            "POS",
            "附着件不在所选直段内",
        ));
    }
    Ok(index as f64 + fraction.clamp(0.0, 1.0))
}

pub(crate) fn extgeo_world_polyline(
    extgeo: &DbElement,
    diameter: f64,
) -> Result<Vec<DVec3>, DbElementError> {
    for name in ["XOFF", "YGAP"] {
        if number_or_zero(extgeo, name)?.abs() > crate::rpath::ATTR_TOL {
            return Err(crate::rpath::failed(
                extgeo,
                name,
                "非零 EXTGEO 偏移尚未通过原生对拍",
            ));
        }
    }
    let mut ends = Vec::new();
    for name in ["STAREF", "ENDREF"] {
        let attachment = reference(extgeo, name)?
            .ok_or_else(|| crate::rpath::failed(extgeo, name, "缺少锚点引用"))?;
        for offset in ["XOFF", "YOFF", "ZOFF"] {
            if number_or_zero(&attachment, offset)?.abs() > crate::rpath::ATTR_TOL {
                return Err(crate::rpath::failed(
                    &attachment,
                    offset,
                    "附着件偏移尚未接入电缆路由",
                ));
            }
        }
        ends.push(
            attachment_anchor(&attachment)?
                .ok_or_else(|| crate::rpath::failed(extgeo, name, "锚点没有可用桥架路由"))?,
        );
    }
    let (start, end) = (&ends[0], &ends[1]);
    if start.cwbran.refno() != end.cwbran.refno() {
        return Err(crate::rpath::failed(
            extgeo,
            "POS",
            "同一 EXTGEO 跨 CWBRAN，尚未还原连接路径",
        ));
    }
    let branch = &start.cwbran;
    let rpath = first_rpath(branch)?
        .ok_or_else(|| crate::rpath::failed(extgeo, "POS", "CWBRAN 没有 RPATH"))?;
    let path = RoutePath::load(extgeo, &rpath, branch)?;
    if path.closed {
        return Err(crate::rpath::failed(
            extgeo,
            "POS",
            "闭合桥架的区间选取尚未接入",
        ));
    }
    let justification = crate::rpath::word(branch, "CWJUSL")?.unwrap_or_default();
    let height = number_or_zero(branch, "VHEIG")?;
    let datum = match justification.as_str() {
        "" | "CENT" => 0.0,
        "CENC" => height * 0.5,
        "TOPC" => height,
        _ => {
            return Err(crate::rpath::failed(
                extgeo,
                "POS",
                format!("桥架横向对正 {justification} 尚未接入"),
            ));
        }
    };
    let lift = match cable_code(branch, "VSHAPE")? {
        0 => diameter * 0.5 - datum,
        1 => 0.0,
        shape => {
            return Err(crate::rpath::failed(
                extgeo,
                "VSHAPE",
                format!("未支持的桥架截面代码 {shape}"),
            ));
        }
    };
    let a = route_station(&path, start)?;
    let b = route_station(&path, end)?;
    // (station, 局部坐标, up 索引, 路径顶点下标)：锚点不带顶点下标，不参与 FRAD 展开。
    let mut raw: Vec<(f64, DVec3, usize, Option<usize>)> =
        vec![(a, start.position, start.station, None)];
    for (i, point) in path.points.iter().enumerate() {
        if (i as f64) > a.min(b) && (i as f64) < a.max(b) {
            raw.push((i as f64, point.pos, i, Some(i)));
        }
    }
    raw.push((b, end.position, end.station, None));
    raw.sort_by(|left, right| left.0.total_cmp(&right.0));
    if a > b {
        raw.reverse();
    }
    // 子路径沿桥架的**倒圆后**中心线走：FRAD 顶点展开成圆弧采样（切入→弧→切出），
    // 与锚定侧的缩减段量距是同一套口径。切线回缩超出相邻已发点时按可用长度
    // 等比缩半径，宁可近似也不让路径折返。
    let up_of = |station_f: f64, index: usize| -> Result<DVec3, DbElementError> {
        let at_vertex = (station_f - index as f64).abs() <= 1e-4;
        let up = if a > b && at_vertex {
            path.arrive_up(index).or_else(|| path.leave_up(index))
        } else {
            path.leave_up(index).or_else(|| path.arrive_up(index))
        };
        up.ok_or_else(|| crate::rpath::failed(extgeo, "POS", "桥架 up 向量未解"))
    };
    let mut locals: Vec<DVec3> = Vec::new();
    for (k, (station_f, pos, up_index, vertex)) in raw.iter().enumerate() {
        let arc = vertex.and_then(|i| {
            let tangent = path.fillet_tangent(i)?;
            // 跨度可能逆着路径方向遍历（ENDREF 的 station 在 STAREF 之前）：
            // 弧的进/出方向必须按遍次序的邻点差分，正序的 arrive/leave 会画反。
            let prev = k
                .checked_sub(1)
                .and_then(|index| raw.get(index))
                .map(|entry| entry.1)
                .unwrap_or(*pos);
            let next = raw.get(k + 1).map(|entry| entry.1).unwrap_or(*pos);
            let dir_in = crate::rpath::unit(*pos - prev)?;
            let dir_out = crate::rpath::unit(next - *pos)?;
            let room = prev.distance(*pos).min(pos.distance(next));
            if room <= 0.05 {
                return None;
            }
            let radius = path.points[i].frad.abs() * tangent.min(room) / tangent;
            fillet_arc_points(*pos, dir_in, dir_out, radius).map(|arc| (i, arc))
        });
        match arc {
            Some((i, arc)) => {
                let steps = arc.len() - 1;
                // 与 dir_in/dir_out 同理：逆序遍历时弧的「入口」是路径正序的离开侧。
                let (up_in, up_out) = if a > b {
                    (
                        path.leave_up(i).unwrap_or(DVec3::Z),
                        path.arrive_up(i).unwrap_or(DVec3::Z),
                    )
                } else {
                    (
                        path.arrive_up(i).unwrap_or(DVec3::Z),
                        path.leave_up(i).unwrap_or(DVec3::Z),
                    )
                };
                for (step, point) in arc.iter().enumerate() {
                    let f = step as f64 / steps as f64;
                    let up = crate::rpath::unit_or_zero(up_in * (1.0 - f) + up_out * f);
                    locals.push(*point + up * lift);
                }
            }
            None => {
                let up = if let Some(i) = *vertex {
                    let up_a = path.arrive_up(i).unwrap_or(DVec3::ZERO);
                    let up_l = path.leave_up(i).unwrap_or(DVec3::ZERO);
                    if up_a.length_squared() > 0.5
                        && up_l.length_squared() > 0.5
                        && up_a.dot(up_l).abs() < 0.5
                    {
                        up_a + up_l
                    } else {
                        up_of(*station_f, *up_index)?
                    }
                } else {
                    up_of(*station_f, *up_index)?
                };
                locals.push(*pos + up * lift);
            }
        }
    }
    let world = world_matrix(branch)?;
    locals
        .into_iter()
        .map(|point| Ok(world.transform_point3(point)))
        .collect()
}

/// 按弯曲半径与转角确定原生 E3D 弯头的分段数。
///
/// 实测对照（250179/135、16、817 全部 9 处弯头面片环数）：
/// - R ≤ 100（66mm）：90° 分 3 段（30°/段）
/// - 100 < R ≤ 200（150mm）：90° 分 4 段（22.5°/段）
/// - 200 < R ≤ 450（300/350.4mm）：90° 分 5 段（18°/段）
/// - 450 < R ≤ 750（600mm）：90° 分 6 段（15°/段；30° 恰为 2 段）
/// - R > 750（900mm）：90° 分 7 段（12.86°/段）
fn fillet_arc_steps(angle: f64, radius: f64) -> usize {
    let steps_per_90 = if radius <= 100.0 {
        3.0
    } else if radius <= 200.0 {
        4.0
    } else if radius <= 450.0 {
        5.0
    } else if radius <= 750.0 {
        6.0
    } else {
        7.0
    };
    ((angle / std::f64::consts::FRAC_PI_2) * steps_per_90)
        .round()
        .max(2.0) as usize
}

/// 两段方向在交点 `corner` 处的 `radius` 圆角：返回从切入点到切出点的
/// 折线采样（含两端点）。角度过小、相邻段长度放不下圆角或几何退化时返回
/// `None`——调用方按斜接处理。
fn fillet_arc_points(
    corner: DVec3,
    dir_in: DVec3,
    dir_out: DVec3,
    radius: f64,
) -> Option<Vec<DVec3>> {
    fillet_arc_points_with_steps(corner, dir_in, dir_out, radius, None)
}

fn fillet_arc_points_with_steps(
    corner: DVec3,
    dir_in: DVec3,
    dir_out: DVec3,
    radius: f64,
    steps_override: Option<usize>,
) -> Option<Vec<DVec3>> {
    let cosine = dir_in.dot(dir_out).clamp(-1.0, 1.0);
    let half = (1.0 - cosine) / 2.0;
    if half < 1e-9 {
        return None;
    }
    let sin_half = half.sqrt();
    let cos_half = (1.0 - half).sqrt();
    let tangent = radius * sin_half / cos_half;
    if !tangent.is_finite() || tangent <= crate::rpath::ATTR_TOL {
        return None;
    }
    let bisector = (dir_out - dir_in).try_normalize()?;
    // 圆心到角点距离 = r/cos(θ/2)：由 (center−t1)·d_in = 0 解出
    // k·|bisector·d_in| = t ⇒ k = r·tan(θ/2)/sin(θ/2) = r/cos(θ/2)。
    // θ=90° 时 sin=cos 掩盖过这里的错误；/817 的 30°/60° 弯把它显形了。
    let center = corner + bisector * (radius / cos_half);
    let t1 = corner - dir_in * tangent;
    let t2 = corner + dir_out * tangent;
    let radius_in = (t1 - center).try_normalize()?;
    let radius_out = (t2 - center).try_normalize()?;
    let axis = dir_in.cross(dir_out).try_normalize()?;
    if radius_in.cross(radius_out).dot(axis) < 0.0 {
        return None;
    }
    let angle = cosine.acos();
    let steps = steps_override
        .unwrap_or_else(|| fillet_arc_steps(angle, radius))
        .max(2);
    Some(
        (0..=steps)
            .map(|i| {
                let f = i as f64 / steps as f64;
                let radial = DQuat::from_axis_angle(axis, angle * f) * radius_in;
                center + radial * radius
            })
            .collect(),
    )
}

/// EXTGEO 端点的局部桥架「上」方向：沿子路径末端点所在段的离开/到达 up。
fn extgeo_end_up(
    extgeo: &DbElement,
    branch: &DbElement,
    path: &RoutePath,
    station: usize,
) -> Result<DVec3, DbElementError> {
    let local = path
        .leave_up(station)
        .or_else(|| path.arrive_up(station))
        .ok_or_else(|| crate::rpath::failed(extgeo, "POS", "桥架 up 向量未解"))?;
    let world = world_matrix(branch)?;
    Ok(world
        .transform_vector3(local)
        .try_normalize()
        .unwrap_or(DVec3::Z))
}

/// `EXTGEO` 的抬升子路径 + 两端锚点上下文（供连接弧用）。
pub(crate) struct ExtgeoSpan {
    /// 抬升后的世界系子路径点列（含两端锚点）。
    pub points: Vec<DVec3>,
    /// 起点锚点（世界系、已抬升）。
    pub start: DVec3,
    /// 终点锚点（世界系、已抬升）。
    pub end: DVec3,
    /// 起点处桥架的「上」方向（世界系）。
    pub start_up: DVec3,
    /// 末端锚点处桥架的「上」方向（世界系）。
    pub end_up: DVec3,
    /// 桥架半宽 `VWIDT * 0.5`。
    pub half_width: f64,
    /// 起点锚点所在桥架段朝行进方向的下游 `POINTR`（已抬升，供跨桥架连接弧瞄准）。
    pub aim_point: DVec3,
    /// 起点锚点所在桥架顶点的 `FRAD`。
    pub start_frad: f64,
    /// 终点锚点所在桥架顶点的 `FRAD`。
    pub end_frad: f64,
    /// 起点是否为 `RNODE`。
    pub start_is_rnode: bool,
    /// 终点是否为 `RNODE`。
    pub end_is_rnode: bool,
}

pub(crate) fn extgeo_world_span(
    extgeo: &DbElement,
    diameter: f64,
) -> Result<ExtgeoSpan, DbElementError> {
    let points = extgeo_world_polyline(extgeo, diameter)?;
    let (start, end) = match (points.first(), points.last()) {
        (Some(s), Some(e)) => (*s, *e),
        _ => return Err(crate::rpath::failed(extgeo, "POS", "EXTGEO 子路径为空")),
    };
    let start_atta = reference(extgeo, "STAREF")?
        .ok_or_else(|| crate::rpath::failed(extgeo, "STAREF", "缺少锚点引用"))?;
    let end_atta = reference(extgeo, "ENDREF")?
        .ok_or_else(|| crate::rpath::failed(extgeo, "ENDREF", "缺少锚点引用"))?;
    let start_is_rnode = start_atta.element_type()? == "RNODE";
    let end_is_rnode = end_atta.element_type()? == "RNODE";
    let start_anchor = attachment_anchor(&start_atta)?;
    let end_anchor = attachment_anchor(&end_atta)?;
    let mut start_up = DVec3::Z;
    let mut end_up = DVec3::Z;
    let mut half_width = 0.0;
    let mut aim_point = points.get(1).copied().unwrap_or(end);
    let mut start_frad = 0.0;
    let mut end_frad = 0.0;
    if let (Some(sa), Some(ea)) = (&start_anchor, &end_anchor) {
        let rpath = first_rpath(&sa.cwbran)?
            .ok_or_else(|| crate::rpath::failed(extgeo, "POS", "CWBRAN 没有 RPATH"))?;
        let path = RoutePath::load(extgeo, &rpath, &sa.cwbran)?;
        start_up = extgeo_end_up(extgeo, &sa.cwbran, &path, sa.station)?;
        end_up = extgeo_end_up(extgeo, &ea.cwbran, &path, ea.station)?;
        half_width = number_or_zero(&sa.cwbran, "VWIDT")?.abs() * 0.5;
        let justification = crate::rpath::word(&sa.cwbran, "CWJUSL")?.unwrap_or_default();
        let height = number_or_zero(&sa.cwbran, "VHEIG")?;
        let datum = match justification.as_str() {
            "CENC" => height * 0.5,
            "TOPC" => height,
            _ => 0.0,
        };
        let lift = if cable_code(&sa.cwbran, "VSHAPE")? == 0 {
            diameter * 0.5 - datum
        } else {
            0.0
        };
        let world = world_matrix(&sa.cwbran)?;
        let a = route_station(&path, sa)?;
        let b = route_station(&path, ea)?;
        let max_idx = path.points.len().saturating_sub(1);
        let s0 = sa.station.min(max_idx);
        let s1 = if b >= a {
            ((a.floor() as usize) + 1).min(max_idx)
        } else {
            (a.ceil() as usize).saturating_sub(1).min(max_idx)
        };
        aim_point = world.transform_point3(path.points[s1].pos) + start_up * lift;
        start_frad = path.points[s0].frad.abs();
        let e0 = ea.station.min(max_idx);
        end_frad = path.points[e0].frad.abs();
    }
    Ok(ExtgeoSpan {
        points,
        start,
        end,
        start_up,
        end_up,
        half_width,
        aim_point,
        start_frad,
        end_frad,
        start_is_rnode,
        end_is_rnode,
    })
}

struct ExtgeoExit {
    end: DVec3,
    last_dir: DVec3,
    up: DVec3,
    half_width: f64,
    end_frad: f64,
    end_is_rnode: bool,
}

pub(crate) fn cable_local_solid(
    cable: &DbElement,
    notes: &mut Vec<String>,
) -> Result<Option<Manifold>, ModelError> {
    let diameter = cable_outer_diameter(cable)?;
    if std::f64::consts::PI * (diameter * 0.5).powi(2) < 0.0001 {
        return Ok(None);
    }
    if crate::rpath::flag(cable, "LMIRR")? {
        return Err(crate::rpath::failed(cable, "LMIRR", "镜像电缆尚未通过原生对拍").into());
    }
    let Some(rpath) = cable_rpath(cable)? else {
        return Ok(None);
    };
    struct CablePiece {
        points: Vec<DVec3>,
        start_cap_normal: Option<DVec3>,
        end_cap_normal: Option<DVec3>,
    }
    let bend = cable_min_bend_radius(cable, diameter);
    let mut points: Vec<DVec3> = Vec::new();
    let mut cur_start_cap: Option<DVec3> = None;
    // 连续 EXTGEO 交界处分件扫掠：原生导出在同一路由上叠两条互不连通的管，
    // 每条独立扫掠、独立封顶。
    let mut pieces: Vec<CablePiece> = Vec::new();
    let terminal = |name: &str, number: &str| -> Result<Option<DVec3>, DbElementError> {
        if cable_code(cable, number)? != 0 {
            return Err(crate::rpath::failed(cable, number, "端点 P 点限定尚未接入"));
        }
        reference(cable, name)?
            .map(|el| world_matrix(&el).map(|m| m.transform_point3(DVec3::ZERO)))
            .transpose()
    };
    let start_terminal = terminal("STAREF", "STAPOI")?;
    // EXTGEO 段的末端锚点上下文：下一个成员到来时决定出口连接形状。
    let mut extgeo_exit: Option<ExtgeoExit> = None;
    let mut prev_extgeo = false;
    for member in rpath.members()? {
        match member.element_type()?.as_str() {
            "POINTR" => {
                if number_or_zero(&member, "FRAD")?.abs() > crate::rpath::ATTR_TOL {
                    return Err(
                        crate::rpath::failed(&member, "FRAD", "电缆自由路径倒圆尚未接入").into(),
                    );
                }
                let pos = world_matrix(&member)?.transform_point3(DVec3::ZERO);
                if let Some(exit) = extgeo_exit.take() {
                    if exit.end_is_rnode
                        && bend > 1e-3
                        && points.len() >= 2
                        && let Some(dir_out) = (pos - exit.end).try_normalize()
                        && let Some(arc) = fillet_arc_points_with_steps(
                            exit.end,
                            exit.last_dir,
                            dir_out,
                            bend,
                            Some(2),
                        )
                    {
                        let t = (exit.end - arc[0]).length();
                        let prev = points[points.len() - 2];
                        if t < prev.distance(exit.end) && t < exit.end.distance(pos) {
                            points.pop();
                            points.extend(arc);
                        }
                    }
                }
                points.push(pos);
            }
            "EXTGEO" => {
                let span = extgeo_world_span(&member, diameter)?;
                let mut merged_coincident = false;
                if let Some(exit) = extgeo_exit.take() {
                    let first_leg = span
                        .points
                        .get(1)
                        .and_then(|p| (*p - span.start).try_normalize());
                    if exit.end.distance(span.start) <= 1.0 {
                        // 共点交接（如 /16 的 205 HATTA == 297 RATTA）：不拆件、不悬空，
                        // 直接在交点处按桥架半宽倒圆并入同一条扫掠折线。
                        let r_bend = bend.max(exit.half_width);
                        if let Some(dir_out) = first_leg
                            && let Some(arc) =
                                fillet_arc_points(span.start, exit.last_dir, dir_out, r_bend)
                        {
                            let t = (span.start - arc[0]).length();
                            let prev_ok = points
                                .len()
                                .checked_sub(2)
                                .is_some_and(|i| points[i].distance(exit.end) > t + 1e-3);
                            let next_ok = span
                                .points
                                .get(1)
                                .is_some_and(|p| p.distance(span.start) > t + 1e-3);
                            if prev_ok && next_ok {
                                points.pop();
                                points.extend(arc);
                                points.extend(span.points.iter().skip(1).copied());
                                merged_coincident = true;
                            }
                        }
                        if !merged_coincident {
                            points.extend(span.points.iter().skip(1).copied());
                            merged_coincident = true;
                        }
                    } else {
                        let d_conn = (span.start - exit.end).try_normalize();
                        let aim_dir = (span.aim_point - exit.end).try_normalize();
                        let is_collinear = d_conn
                            .is_some_and(|dir| exit.last_dir.dot(dir).abs() >= 1.0 - 1e-6)
                            && first_leg
                                .is_some_and(|dir| exit.last_dir.dot(dir).abs() >= 1.0 - 1e-6);
                        if is_collinear {
                            // 共线跨接（如 /817 的 822 -> 823）：直接顺延，不生出口弯、不拆件。
                            points.extend(span.points.iter().copied());
                            merged_coincident = true;
                        } else {
                            let out_dir = match aim_dir {
                                Some(dir) if dir.dot(exit.up) < 1.0 - 1e-6 => dir,
                                _ => exit.up,
                            };
                            let cap_normal = d_conn.and_then(|dc| (out_dir + dc).try_normalize());
                            let is_vertical_exit = out_dir.dot(exit.up).abs() > 0.5;
                            let side = if is_vertical_exit {
                                exit.up
                                    .cross(exit.last_dir)
                                    .try_normalize()
                                    .map(|s| s * (diameter * 0.5))
                                    .unwrap_or(DVec3::ZERO)
                            } else {
                                DVec3::ZERO
                            };
                            let corner = exit.end + side;
                            let r_bend = if is_vertical_exit {
                                bend
                            } else if span.start_frad > 0.0 {
                                bend.max(exit.end_frad + span.start_frad)
                            } else {
                                bend.max(exit.half_width)
                            };
                            let cosine = exit.last_dir.dot(out_dir).clamp(-1.0, 1.0);
                            if cosine < 1.0 - 1e-6 && cosine > -1.0 + 1e-6 {
                                if let Some(arc) =
                                    fillet_arc_points(corner, exit.last_dir, out_dir, r_bend)
                                {
                                    let t = (corner - arc[0]).length();
                                    let incoming_long_enough =
                                        points.len().checked_sub(2).is_some_and(|i| {
                                            points[i].distance(exit.end)
                                                > t + crate::rpath::ATTR_TOL
                                        });
                                    if incoming_long_enough
                                        && let Some(last) = points.last_mut()
                                        && last.distance(exit.end) < crate::rpath::ATTR_TOL * 10.0
                                    {
                                        *last = arc[0];
                                        points.extend(arc.into_iter().skip(1));
                                    }
                                }
                            }
                            pieces.push(CablePiece {
                                points: std::mem::take(&mut points),
                                start_cap_normal: cur_start_cap.take(),
                                end_cap_normal: cap_normal,
                            });
                            points.push(exit.end);
                            cur_start_cap = cap_normal;
                        }
                    }
                    prev_extgeo = true;
                }
                // 入口：上一个非 EXTGEO 段（POINTR/端点）与本段之间做锚点圆角。
                if !merged_coincident {
                    if !prev_extgeo
                        && let Some(&prev) = points.last()
                        && prev.distance(span.start) > 3.0
                        && span.points.len() >= 2
                    {
                        let dir_in = (span.start - prev).try_normalize();
                        let dir_leg = (span.points[1] - span.start).try_normalize();
                        if let (Some(dir_in), Some(dir_leg)) = (dir_in, dir_leg) {
                            if span.start_is_rnode {
                                let lateral = span
                                    .start_up
                                    .cross(dir_leg)
                                    .try_normalize()
                                    .unwrap_or(DVec3::ZERO);
                                if dir_in.dot(lateral).abs() > 0.5
                                    && prev.distance(span.start) < span.half_width + bend - 1e-3
                                {
                                    return Ok(None);
                                }
                            }
                            let entry_steps = if bend > 300.0 && bend <= 450.0 {
                                Some(4)
                            } else {
                                None
                            };
                            if let Some(arc) = fillet_arc_points_with_steps(
                                span.start,
                                dir_in,
                                dir_leg,
                                bend,
                                entry_steps,
                            ) {
                                let t = (span.start - arc[0]).length();
                                if t < prev.distance(span.start)
                                    && t < span.points[1].distance(span.start)
                                {
                                    points.extend(arc.iter().copied());
                                    points.extend(span.points.iter().skip(1).copied());
                                } else {
                                    points.extend(span.points.iter().copied());
                                }
                            } else {
                                points.extend(span.points.iter().copied());
                            }
                        } else {
                            points.extend(span.points.iter().copied());
                        }
                    } else {
                        points.extend(span.points.iter().copied());
                    }
                }
                let count = span.points.len();
                let last_dir = count
                    .checked_sub(2)
                    .and_then(|i| (span.points[count - 1] - span.points[i]).try_normalize());
                extgeo_exit = last_dir.map(|dir| ExtgeoExit {
                    end: span.end,
                    last_dir: dir,
                    up: span.end_up,
                    half_width: span.half_width,
                    end_frad: span.end_frad,
                    end_is_rnode: span.end_is_rnode,
                });
                prev_extgeo = true;
            }
            noun => {
                return Err(crate::rpath::failed(
                    &member,
                    "POS",
                    format!("电缆路径段 {noun} 尚未接入"),
                )
                .into());
            }
        }
        if member.element_type()? != "EXTGEO" {
            prev_extgeo = false;
        }
    }
    let end_terminal = terminal("ENDREF", "ENDPOI")?;
    pieces.push(CablePiece {
        points,
        start_cap_normal: cur_start_cap.take(),
        end_cap_normal: None,
    });
    // 端子落在整条路由的首/末件上（连续 EXTGEO 分件后，末件只含尾部路由）。
    if let Some(point) = start_terminal
        && let Some(first) = pieces.first_mut()
        && first
            .points
            .first()
            .is_some_and(|head| head.distance(point) > 3.0)
    {
        first.points.insert(0, point);
    }
    if let Some(point) = end_terminal
        && let Some(last) = pieces.last_mut()
        && last
            .points
            .last()
            .is_some_and(|tail| tail.distance(point) > 3.0)
    {
        last.points.push(point);
    }
    let to_local = world_matrix(cable)?.inverse();
    let up = crate::rpath::vector(&rpath, "YDIR")?.unwrap_or(DVec3::Z);
    let bank = number_or_zero(cable, "BANG")?;
    let mut solid: Option<Manifold> = None;
    let mut swept_pieces = 0usize;
    for piece in &pieces {
        let local: Vec<_> = piece
            .points
            .iter()
            .map(|point| to_local.transform_point3(*point))
            .collect();
        let start_cap = piece
            .start_cap_normal
            .and_then(|n| to_local.transform_vector3(n).try_normalize());
        let end_cap = piece
            .end_cap_normal
            .and_then(|n| to_local.transform_vector3(n).try_normalize());
        if std::env::var("E3D_MODEL_CABLE_DEBUG_PATH").as_deref() == Ok("1") {
            for (i, point) in piece.points.iter().enumerate() {
                eprintln!("  cable piece[{i}] = {point:?}");
            }
        }
        let Some(part) =
            round_polyline_solid_with_caps(&local, diameter * 0.5, up, bank, start_cap, end_cap)?
        else {
            continue;
        };
        swept_pieces += 1;
        solid = Some(match solid {
            Some(body) => body.union(&part),
            None => part,
        });
    }
    notes.push(format!(
        "CABLE：OUTD={diameter}mm MINBEN={bend}mm，{} 个扫掠件；EXTGEO 子路径 + 边界圆角",
        swept_pieces
    ));
    Ok(solid)
}

#[cfg(test)]
fn attach_terminals(points: &mut Vec<DVec3>, start: Option<DVec3>, end: Option<DVec3>) {
    if points.is_empty() {
        return;
    }
    if let Some(point) = start
        && points[0].distance(point) > 3.0
    {
        points.insert(0, point);
    }
    if let Some(point) = end
        && points.last().unwrap().distance(point) > 3.0
    {
        points.push(point);
    }
}

#[cfg(test)]
fn round_polyline_solid(
    points: &[DVec3],
    radius: f64,
    up: DVec3,
    bank: f64,
) -> Result<Option<Manifold>, SolidError> {
    round_polyline_solid_with_caps(points, radius, up, bank, None, None)
}

fn round_polyline_solid_with_caps(
    points: &[DVec3],
    radius: f64,
    up: DVec3,
    bank: f64,
    start_cap_normal: Option<DVec3>,
    end_cap_normal: Option<DVec3>,
) -> Result<Option<Manifold>, SolidError> {
    if !radius.is_finite()
        || radius <= 0.0
        || !up.is_finite()
        || !bank.is_finite()
        || points.iter().any(|point| !point.is_finite())
    {
        return Err(SolidError::Degenerate {
            what: "cable path",
            detail: "非有限路径或非正半径".into(),
        });
    }
    // 相邻点合并阈值：库里不同记录写"同一个点"也会差到 ~1e-4 mm（如
    // `102901.515` vs `102901.515085`）；1e-6 的阈值会留下亚微米折返段，
    // 让斜接 degenerate。0.05mm 远低于电缆半径，又远大于浮点抖动。
    let mut clean: Vec<DVec3> = Vec::new();
    for point in points {
        if clean.last().is_none_or(|last| last.distance(*point) > 0.05) {
            clean.push(*point);
        }
    }
    // 亚毫米折返钉：库里相邻路径点可能只差零点几毫米却方向相反（实测
    // `319540.0 → 319539.75 → 319545`，cos = −1），斜接面在此退化。删掉这种
    // 尖钉中点，路径偏差 ≤ 钉高（<1mm，远低于半径）；超过 1mm 的折返仍报错。
    loop {
        let Some(index) = (1..clean.len().saturating_sub(1)).find(|&i| {
            let back = clean[i].distance(clean[i - 1]);
            back <= 1.0
                && (clean[i] - clean[i - 1])
                    .try_normalize()
                    .and_then(|d1| {
                        (clean[i + 1] - clean[i])
                            .try_normalize()
                            .map(|d2| d1.dot(d2))
                    })
                    .is_some_and(|cosine| cosine < -0.99)
        }) else {
            break;
        };
        clean.remove(index);
    }
    if clean.len() < 2 {
        return Ok(None);
    }
    if clean
        .windows(2)
        .any(|pair| pair[0].distance(pair[1]) <= 0.05)
    {
        return Err(SolidError::Degenerate {
            what: "cable mitre",
            detail: "斜接面退化".into(),
        });
    }
    let directions: Vec<_> = clean
        .windows(2)
        .map(|pair| (pair[1] - pair[0]).normalize())
        .collect();
    if let Some(turn) = (1..clean.len() - 1).find(|&i| directions[i - 1].dot(directions[i]) < -0.99)
    {
        let left = round_polyline_solid_with_caps(
            &clean[..=turn],
            radius,
            up,
            bank,
            start_cap_normal,
            None,
        )?;
        let right =
            round_polyline_solid_with_caps(&clean[turn..], radius, up, bank, None, end_cap_normal)?;
        return Ok(match (left, right) {
            (Some(a), Some(b)) => Some(a.union(&b)),
            (Some(a), None) => Some(a),
            (None, Some(b)) => Some(b),
            (None, None) => None,
        });
    }
    let mut normals = Vec::new();
    let mut extensions = Vec::new();
    for pair in directions.windows(2) {
        let (normal, extension) = match (pair[0] + pair[1]).try_normalize() {
            // ~180° 折返（电缆折回，如 /817 跨桥架的来回腿）：原生不是斜接也不是
            // 弧——两段管在折点用垂直于轴的端面平盖对接（原生面片实测：折点处
            // 一个垂直圆盘端盖）。斜接延伸在此发散，换成零延伸 + 垂直接口。
            _ if pair[0].dot(pair[1]) < -0.99 => (pair[0], 0.0),
            Some(normal) => {
                let cosine = normal.dot(pair[0]);
                if cosine < 1e-6 {
                    return Err(SolidError::Degenerate {
                        what: "cable mitre",
                        detail: "斜接面退化".into(),
                    });
                }
                (
                    normal,
                    radius * (1.0 - cosine * cosine).max(0.0).sqrt() / cosine + 0.001,
                )
            }
            None => {
                return Err(SolidError::Degenerate {
                    what: "cable mitre",
                    detail: "斜接面退化".into(),
                });
            }
        };
        normals.push(normal);
        extensions.push(extension);
    }
    let start_normal = start_cap_normal.filter(|n| directions[0].dot(*n) > 0.1);
    let end_normal = end_cap_normal.filter(|n| directions.last().is_some_and(|d| d.dot(*n) > 0.1));
    let cap_ext = |dir: DVec3, n: DVec3| -> f64 {
        let c = dir.dot(n).clamp(0.1, 1.0);
        radius * (1.0 - c * c).max(0.0).sqrt() / c + 0.001
    };
    let start_cap_ext = start_normal.map_or(0.0, |n| cap_ext(directions[0], n));
    let end_cap_ext = end_normal.map_or(0.0, |n| cap_ext(*directions.last().unwrap(), n));
    let first = directions[0];
    let mut y = (up - first * up.dot(first))
        .try_normalize()
        .unwrap_or_else(|| {
            let reference = if first.z.abs() < 0.9 {
                DVec3::Z
            } else {
                DVec3::Y
            };
            (reference - first * reference.dot(first)).normalize()
        });
    let can_direct_sweep = directions
        .windows(2)
        .all(|pair| pair[0].dot(pair[1]) >= -0.99)
        && clean.windows(2).enumerate().all(|(i, pair)| {
            let s = if i == 0 {
                start_cap_ext
            } else {
                extensions[i - 1]
            };
            let e = if i + 1 == directions.len() {
                end_cap_ext
            } else {
                extensions[i]
            };
            pair[0].distance(pair[1]) > s + e
        });
    if can_direct_sweep {
        const N: usize = 8;
        let m_count = clean.len();
        let mut vertices = Vec::with_capacity(m_count * N * 3);
        let mut y_cur = y;
        let bank_rot = DMat3::from_rotation_z(bank.to_radians());
        let mut frames = Vec::with_capacity(directions.len());
        for (i, &dir) in directions.iter().enumerate() {
            if i > 0 {
                y_cur = DQuat::from_rotation_arc(directions[i - 1], dir) * y_cur;
            }
            y_cur = (y_cur - dir * y_cur.dot(dir)).normalize();
            let basis = DMat3::from_cols(y_cur.cross(dir), y_cur, dir) * bank_rot;
            frames.push((basis.x_axis, basis.y_axis, dir));
        }
        for k in 0..m_count {
            let (x_axis, y_axis, dir) = if k < frames.len() {
                frames[k]
            } else {
                frames[frames.len() - 1]
            };
            let mitre = if k == 0 {
                start_normal
            } else if k + 1 < m_count {
                Some(normals[k - 1])
            } else {
                end_normal
            };
            for m in 0..N {
                let phi = (m as f64) * std::f64::consts::TAU / (N as f64);
                let (sin_m, cos_m) = phi.sin_cos();
                let offset = radius * (cos_m * x_axis + sin_m * y_axis);
                let pt = match mitre {
                    Some(n) => clean[k] + offset - dir * (offset.dot(n) / dir.dot(n)),
                    None => clean[k] + offset,
                };
                vertices.extend_from_slice(&[pt.x, pt.y, pt.z]);
            }
        }
        let mut triangles: Vec<u64> = Vec::with_capacity(((m_count - 1) * N * 2 + (N - 2) * 2) * 3);
        for m in 1..(N - 1) {
            triangles.extend([0, (m + 1) as u64, m as u64]);
        }
        for i in 0..(m_count - 1) {
            let base = (i * N) as u64;
            let next = ((i + 1) * N) as u64;
            for m in 0..N {
                let m_next = ((m + 1) % N) as u64;
                let m = m as u64;
                triangles.extend([base + m, base + m_next, next + m_next]);
                triangles.extend([base + m, next + m_next, next + m]);
            }
        }
        let last_base = ((m_count - 1) * N) as u64;
        for m in 1..(N - 1) {
            triangles.extend([last_base, last_base + m as u64, last_base + (m + 1) as u64]);
        }
        if let Ok(solid) = Manifold::from_mesh_f64(&vertices, 3, &triangles)
            && !solid.is_empty()
        {
            return Ok(Some(solid));
        }
    }
    let mut pieces = Vec::new();
    for (i, pair) in clean.windows(2).enumerate() {
        let direction = directions[i];
        if i > 0 {
            y = DQuat::from_rotation_arc(directions[i - 1], direction) * y;
        }
        y = (y - direction * y.dot(direction)).normalize();
        let start_extra = if i == 0 {
            start_cap_ext
        } else {
            extensions[i - 1]
        };
        let end_extra = if i + 1 == directions.len() {
            end_cap_ext
        } else {
            extensions[i]
        };
        let frame = DMat4::from_translation(pair[0] - direction * start_extra)
            * DMat4::from_mat3(
                DMat3::from_cols(y.cross(direction), y, direction)
                    * DMat3::from_rotation_z(bank.to_radians()),
            );
        let cylinder = Manifold::cylinder(
            pair[0].distance(pair[1]) + start_extra + end_extra,
            radius,
            radius,
            // 原生电缆 FacetGroup 环一律 8 边（/135 的 RVM 环点数实测），
            // 与负圆柱的轮廓口径不同——按半径离散只会制造镶嵌差异。
            8,
            false,
        );
        let mut piece = placed(&cylinder, frame);
        if i == 0 {
            if let Some(n) = start_normal {
                piece = cut_by_plane(&piece, pair[0], n);
            }
        } else {
            piece = cut_by_plane(&piece, pair[0] - direction * 1e-4, normals[i - 1]);
        }
        if i + 1 == directions.len() {
            if let Some(n) = end_normal {
                piece = cut_by_plane(&piece, pair[1], -n);
            }
        } else if let Some(normal) = normals.get(i) {
            piece = cut_by_plane(&piece, pair[1] + direction * 1e-4, -*normal);
        }
        if piece.is_empty() {
            return Err(SolidError::EmptyManifold {
                what: "cable segment",
            });
        }
        pieces.push(piece);
    }
    let mut parts = pieces.into_iter();
    Ok(parts
        .next()
        .map(|first| parts.fold(first, |body, part| body.union(&part))))
}

#[cfg(test)]
pub(crate) struct MemberEndpoints {
    pub refno: e3d_io::refno::RefNo,
    pub noun: String,
    pub points: Result<Vec<DVec3>, DbElementError>,
}

/// `CABLE` RPATH 的成员序列里，每个成员提供的世界系端点：POINTR 是单点（自身
/// `POS` 经属主链）、EXTGEO 是借桥架的两端点。给连续性检查与将来的段体拼接用。
#[cfg(test)]
pub(crate) fn member_world_endpoints(
    cable: &DbElement,
) -> Result<Vec<MemberEndpoints>, DbElementError> {
    let mut out = Vec::new();
    let Some(rpath) = cable_rpath(cable)? else {
        return Ok(out);
    };
    for member in rpath.members()? {
        let noun = member.element_type()?;
        let points = match noun.as_str() {
            "POINTR" => {
                world_matrix(&member).map(|world| vec![world.transform_point3(DVec3::ZERO)])
            }
            "EXTGEO" => extgeo_world_endpoints(&member)
                .map(|points| points.map(|(a, b)| vec![a, b]).unwrap_or_default()),
            _ => Ok(Vec::new()),
        };
        out.push(MemberEndpoints {
            refno: member.refno(),
            noun,
            points,
        });
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::db_discovery::DirectoryDbResolver;
    use e3d_io::db_element::{DbFilePin, DbFileResolver, DbSet};
    use e3d_io::refno::RefNo;
    use e3d_io::{ReadOnlyEngine, ScanTier};
    use std::path::{Path, PathBuf};
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

    struct ProbeResolver {
        local: DirectoryDbResolver,
        shared: Arc<DirectoryDbResolver>,
    }

    impl DbFileResolver for ProbeResolver {
        fn resolve(&self, dbno: u32) -> Option<DbFilePin> {
            self.local
                .resolve(dbno)
                .or_else(|| self.shared.resolve(dbno))
        }
    }

    fn corpus_set(source: &Path, template: &Path, shared: Arc<DirectoryDbResolver>) -> Arc<DbSet> {
        let local = DirectoryDbResolver::scan(
            &[source.parent().expect("语料文件必须有目录").to_path_buf()],
            template,
        )
        .expect("源工程目录必须可扫描");
        let set = Arc::new(
            DbSet::with_attlib_file_and_resolver(
                &template.join("attlib.dat"),
                Box::new(ProbeResolver { local, shared }),
            )
            .expect("schema 必须可读"),
        );
        set.add_db(DbFilePin {
            file: source.to_path_buf(),
            template: e3d_io::db_element::template_file_for(template, "DESI").unwrap(),
            db_type: Some("DESI".into()),
            sesno: None,
        })
        .expect("扫描到的源库必须可注册");
        set
    }

    /// A2 语料验证：全语料 `CABLE` 的 RPATH 成员链连续性——POINTR 端点与相邻
    /// `EXTGEO` 解出来的附着件世界位置应首尾相接（HATTA/TATTA 的「首/末点」约定、
    /// RATTA 的 `POSREF` 插值都在这条检查上显形）。断口 ≫ 桥架转弯半径就是语义错了。
    #[test]
    fn cable_terminal_connectors_respect_the_core_three_mm_gate() {
        let original = vec![DVec3::ZERO, DVec3::X * 10.0];
        let mut points = original.clone();
        attach_terminals(&mut points, Some(DVec3::NEG_X * 3.0), Some(DVec3::X * 13.0));
        assert_eq!(points, original);
        attach_terminals(
            &mut points,
            Some(DVec3::NEG_X * 3.001),
            Some(DVec3::X * 13.001),
        );
        assert_eq!(points.len(), 4);
        let mut empty = Vec::new();
        attach_terminals(&mut empty, Some(DVec3::ZERO), Some(DVec3::X * 10.0));
        assert!(empty.is_empty());
    }

    #[test]
    fn round_cable_straight_has_the_profile_area_times_length() {
        let solid = round_polyline_solid(&[DVec3::ZERO, DVec3::X * 10.0], 1.0, DVec3::Z, 0.0)
            .unwrap()
            .unwrap();
        let expected = 4.0 * std::f64::consts::FRAC_1_SQRT_2 * 10.0;
        assert!((solid.volume() - expected).abs() < 1e-8);
    }

    #[test]
    fn round_cable_mitres_join_without_losing_volume() {
        let solid = round_polyline_solid(
            &[DVec3::ZERO, DVec3::X * 10.0, DVec3::new(10.0, 10.0, 0.0)],
            1.0,
            DVec3::Z,
            0.0,
        )
        .unwrap()
        .unwrap();
        let expected = 4.0 * std::f64::consts::FRAC_1_SQRT_2 * 20.0;
        assert!(
            (solid.volume() - expected).abs() < 1e-6,
            "{} != {expected}",
            solid.volume()
        );
    }

    #[test]
    fn round_cable_rejects_nonfinite_and_reversed_paths() {
        assert!(round_polyline_solid(&[DVec3::ZERO, DVec3::NAN], 1.0, DVec3::Z, 0.0).is_err());
        assert!(
            round_polyline_solid(&[DVec3::ZERO, DVec3::X, DVec3::ZERO], 1.0, DVec3::Z, 0.0)
                .is_err()
        );
        assert!(
            round_polyline_solid(&[DVec3::ZERO, DVec3::X], f64::INFINITY, DVec3::Z, 0.0).is_err()
        );
        assert!(
            round_polyline_solid(&[DVec3::ZERO, DVec3::ZERO], 1.0, DVec3::Z, 0.0)
                .unwrap()
                .is_none()
        );
    }

    #[test]
    fn round_vertical_cable_uses_a_finite_frame() {
        let solid = round_polyline_solid(&[DVec3::ZERO, DVec3::Z * 10.0], 1.0, DVec3::Z, 0.0)
            .unwrap()
            .unwrap();
        assert!(solid.volume().is_finite());
        assert!((solid.volume() - 4.0 * std::f64::consts::FRAC_1_SQRT_2 * 10.0).abs() < 1e-8);
    }

    #[test]
    #[ignore = "需要 APS/ACP 语料，并设置 E3D_MODEL_CABLE_GEOMETRY_EXPERIMENTAL=1"]
    fn native_nonempty_cable_is_generated() {
        assert!(
            geometry_enabled(),
            "此回归须显式开启实验开关；默认输出保持不变"
        );
        let dirs = corpus_dirs();
        let template =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../e3d-io/schema/e3d31/binaries");
        let shared = Arc::new(DirectoryDbResolver::scan(&dirs, &template).unwrap());
        let source = dirs
            .iter()
            .map(|dir| dir.join("aps250179_0001"))
            .find(|path| path.is_file())
            .expect("缺少 APS 250179 语料");
        let set = corpus_set(&source, &template, shared.clone());
        for (id, expected_outd) in [(135, 58.4), (16, 28.0), (817, 16.5)] {
            let root = set.element(RefNo::new(2013286723, id));
            let outd = cable_outer_diameter(&root).unwrap();
            assert!((outd - expected_outd).abs() < 1e-3, "id={id} outd={outd}");
            let outcome = crate::pipeline::generate_subtree(root.clone()).unwrap();
            assert_eq!(
                outcome.elements.len(),
                1,
                "原生 RVM 有实体 ({id})：failed={:?}",
                outcome.report.failed
            );
            assert_eq!(outcome.elements[0].noun, "CABLE");
            assert!(!outcome.elements[0].solid.is_empty());
            assert!(crate::category::is_model_unit("CABLE"));
            assert_eq!(outcome.report.composites_built.get("CABLE"), Some(&1));
            assert!(!outcome.report.evidence_pending.contains_key("CABLE"));
            let unit =
                crate::elmodl::model_element(&root, crate::category::classify("CABLE")).unwrap();
            let crate::elmodl::Modeled::Solid(unit) = unit else {
                panic!("单元重建未产出电缆实体 ({id})")
            };
            assert!((unit.solid.volume() - outcome.elements[0].solid.volume()).abs() < 1e-8);
        }

        let source_176 = dirs
            .iter()
            .map(|dir| dir.join("aps250176_0001"))
            .find(|path| path.is_file())
            .expect("缺少 APS 250176 语料");
        let set_176 = corpus_set(&source_176, &template, shared);
        let root_575 = set_176.element(RefNo::new(2013286720, 575));
        let outcome_575 = crate::pipeline::generate_subtree(root_575.clone()).unwrap();
        assert!(
            outcome_575.elements.is_empty() && outcome_575.report.failed.is_empty(),
            "250176/575 原生 RVM 为 0 PRIM：elements={} failed={:?}",
            outcome_575.elements.len(),
            outcome_575.report.failed
        );
    }

    #[test]
    #[ignore = "需要 APS/ACP 语料；分别在实验开关关/开时运行"]
    fn cable_experimental_gate_and_partial_failure_are_explicit() {
        let dirs = corpus_dirs();
        let template =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../e3d-io/schema/e3d31/binaries");
        let shared = Arc::new(DirectoryDbResolver::scan(&dirs, &template).unwrap());
        let source = dirs
            .iter()
            .map(|dir| dir.join("aps250179_0001"))
            .find(|path| path.is_file())
            .expect("缺少 APS 250179 语料");
        let set = corpus_set(&source, &template, shared);
        assert_eq!(crate::category::is_model_unit("CABLE"), geometry_enabled());
        if geometry_enabled() {
            for id in [16, 817] {
                let root = set.element(RefNo::new(2013286723, id));
                let outcome = crate::pipeline::generate_subtree(root).unwrap();
                assert_eq!(outcome.elements.len(), 1);
                assert!(outcome.report.failed.is_empty());
                assert_eq!(outcome.report.composites_built.get("CABLE"), Some(&1));
            }
        } else {
            let root = set.element(RefNo::new(2013286723, 135));
            let outcome = crate::pipeline::generate_subtree(root.clone()).unwrap();
            assert!(outcome.elements.is_empty());
            assert_eq!(outcome.report.evidence_pending.get("CABLE"), Some(&1));
            assert!(matches!(
                crate::elmodl::model_element(&root, crate::category::classify("CABLE")).unwrap(),
                crate::elmodl::Modeled::Skipped { .. }
            ));
        }
    }

    /// A2/B 取证：打印一根 CABLE 的完整成员链——本 RPATH 成员（POINTR 位置 /
    /// EXTGEO 锚点）、CABLE 自身的 STAREF/ENDREF 端子、各 EXTGEO 两端附着件的
    /// 锚点位置与站号、以及所在 CWBRAN 路径点（含 FRAD/方向），和原生扫掠中心线
    /// 逐段对照。`AIOS_CABLE_DUMP_REFNO=dbno/id` 指定目标。
    #[test]
    #[ignore = "真库语料取证：需要本机 E3D 样例项目；cargo test -- --ignored"]
    fn probe_cable_route_dump() {
        let dirs = corpus_dirs();
        let template =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../e3d-io/schema/e3d31/binaries");
        let shared =
            Arc::new(DirectoryDbResolver::scan(&dirs, &template).expect("语料目录必须可扫描"));
        let target =
            std::env::var("AIOS_CABLE_DUMP_REFNO").unwrap_or_else(|_| "2013286723/135".into());
        let (dbno, id) = target.split_once('/').expect("refno 形如 dbno/id");
        let (dbno, id): (u32, u32) = (dbno.parse().unwrap(), id.parse().unwrap());
        let target_ref = RefNo::new(dbno, id);
        let desi_hash = e3d_attlib::db1_hash("DESI");
        let cable_hash = e3d_attlib::db1_hash("CABLE");
        let mut source = None;
        'outer: for dir in &dirs {
            for entry in std::fs::read_dir(dir).expect("语料目录必须可读") {
                let path = entry.expect("语料条目必须可读").path();
                if !path.is_file() {
                    continue;
                }
                let Ok(mut engine) = ReadOnlyEngine::open(&path) else {
                    continue;
                };
                if engine.descriptor().db_type_hash != desi_hash {
                    continue;
                }
                let scan = engine.scan_elements(ScanTier::Header).expect("头扫描失败");
                for scanned in scan.flatten() {
                    if scanned.refno == target_ref && scanned.noun_hash == cable_hash {
                        source = Some(path);
                        break 'outer;
                    }
                }
            }
        }
        let source = source.unwrap_or_else(|| panic!("语料里没有 CABLE {target}"));
        let set = corpus_set(&source, &template, shared);
        let cable = set.element(RefNo::new(dbno, id));
        let world_of = |el: &DbElement| {
            world_matrix(el)
                .map(|m| m.transform_point3(DVec3::ZERO))
                .unwrap_or(DVec3::NAN)
        };
        eprintln!("=== cable {target} ===");
        for name in ["STAREF", "ENDREF", "STBINF", "ENBINF"] {
            match reference(&cable, name) {
                Ok(Some(el)) => eprintln!(
                    "  {name} -> {} {} pos={:?}",
                    el.refno(),
                    el.element_type().unwrap_or_default(),
                    world_of(&el)
                ),
                other => eprintln!("  {name} -> {other:?}"),
            }
        }
        let Some(rpath) = cable_rpath(&cable).unwrap() else {
            panic!("没有 RPATH");
        };
        eprintln!("  RPATH {} members:", rpath.refno());
        for member in rpath.members().unwrap() {
            let noun = member.element_type().unwrap_or_default();
            eprint!("    {} {}", member.refno(), noun);
            for attr in ["FRAD", "XOFF", "YGAP"] {
                if let Ok(v) = number_or_zero(&member, attr)
                    && v != 0.0
                {
                    eprint!(" {attr}={v}");
                }
            }
            match noun.as_str() {
                "POINTR" => eprint!(" pos={:?}", world_of(&member)),
                "EXTGEO" => {
                    for name in ["STBINF", "ENBINF"] {
                        match reference(&member, name) {
                            Ok(Some(b)) => eprint!(
                                " {name}=({} {})",
                                b.refno(),
                                b.element_type().unwrap_or_default()
                            ),
                            Ok(None) => eprint!(" {name}=none"),
                            Err(e) => eprint!(" {name}=err({e:?})"),
                        }
                    }
                    for name in ["STAREF", "ENDREF"] {
                        match reference(&member, name) {
                            Ok(Some(atta)) => {
                                eprint!(
                                    " {name}={}({})",
                                    atta.refno(),
                                    atta.element_type().unwrap_or_default()
                                );
                                for attr in ["POSREF", "PKDI", "ZDIS", "ZOFF"] {
                                    if let Ok(v) = reference(&atta, attr) {
                                        if let Some(t) = v {
                                            eprint!(" {attr}=({})", t.refno());
                                        }
                                    } else if let Ok(v) = number_or_zero(&atta, attr) {
                                        if v != 0.0 {
                                            eprint!(" {attr}={v}");
                                        }
                                    }
                                }
                                match attachment_anchor(&atta) {
                                    Ok(Some(anchor)) => {
                                        let wp = world_matrix(&anchor.cwbran)
                                            .unwrap()
                                            .transform_point3(anchor.position);
                                        eprint!(
                                            "@{:?} station={} br={}",
                                            wp,
                                            anchor.station,
                                            anchor.cwbran.refno()
                                        );
                                    }
                                    other => eprint!(" anchor={other:?}"),
                                }
                            }
                            other => eprint!(" {name}={other:?}"),
                        }
                    }
                }
                _ => {}
            }
            eprintln!();
        }
        // 两条 CWBRAN 的路径点（含 FRAD，绕过 RoutePath::load 的拒绝）
        let mut seen: Vec<String> = Vec::new();
        for member in rpath.members().unwrap() {
            if member.element_type().unwrap_or_default() != "EXTGEO" {
                continue;
            }
            for name in ["STAREF", "ENDREF"] {
                let Ok(Some(atta)) = reference(&member, name) else {
                    continue;
                };
                let Ok(Some(branch)) = owning_branch(&atta) else {
                    continue;
                };
                let key = branch.refno().to_string();
                if seen.contains(&key) {
                    continue;
                }
                seen.push(key);
                let Ok(Some(brpath)) = first_rpath(&branch) else {
                    continue;
                };
                eprintln!("  CWBRAN {}:", branch.refno());
                for point in brpath.members().unwrap() {
                    let frad = number_or_zero(&point, "FRAD").unwrap_or(f64::NAN);
                    eprintln!(
                        "    {} {} pos={:?} FRAD={frad}",
                        point.refno(),
                        point.element_type().unwrap_or_default(),
                        world_of(&point)
                    );
                }
            }
        }
    }

    #[test]
    #[ignore = "真库语料探针：需要本机 E3D 样例项目；cargo test -- --ignored"]
    fn probe_extgeo_chain_continuity() {
        let dirs = corpus_dirs();
        let template =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../e3d-io/schema/e3d31/binaries");
        let shared =
            Arc::new(DirectoryDbResolver::scan(&dirs, &template).expect("语料目录必须可扫描"));
        let cable_hash = e3d_attlib::db1_hash("CABLE");
        let desi_hash = e3d_attlib::db1_hash("DESI");
        let extgeo_hash = e3d_attlib::db1_hash("EXTGEO");
        let mut scanned_cables = 0usize;
        let mut scanned_extgeo = 0usize;

        let mut cables = 0usize;
        let mut extgeo_segments = 0usize;
        let mut extgeo_resolved = 0usize;
        let mut extgeo_unresolved = Vec::new();
        let mut cable_errors = Vec::new();
        let mut samples = Vec::new();
        let mut joints = Vec::new();

        for dir in &dirs {
            let entries = std::fs::read_dir(dir).expect("语料目录必须可读");
            for entry in entries {
                let entry = entry.expect("语料文件条目必须可读");
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
                let scan = engine
                    .scan_elements(ScanTier::Header)
                    .expect("DESI 头扫描失败，不得把不完整语料当成通过");
                let mut set = None;
                for scanned in scan {
                    let scanned = scanned.expect("DESI 元素头必须可读");
                    if scanned.noun_hash == extgeo_hash {
                        scanned_extgeo += 1;
                    }
                    if scanned.noun_hash != cable_hash {
                        continue;
                    }
                    scanned_cables += 1;
                    let set =
                        set.get_or_insert_with(|| corpus_set(&path, &template, shared.clone()));
                    let cable = set.element(scanned.refno);
                    cables += 1;
                    let file = path.file_name().unwrap_or_default().to_string_lossy();
                    let members = match member_world_endpoints(&cable) {
                        Ok(members) => members,
                        Err(error) => {
                            cable_errors.push(format!("{file} cable {}: {error}", cable.refno()));
                            continue;
                        }
                    };
                    // 只查 EXTGEO 段的两端：STAREF 端应接上「前一个成员的末点」，
                    // ENDREF 端应被「后一个成员的首点」接上。连续 = 附着件锚定语义对。
                    for (index, member) in members.iter().enumerate() {
                        if member.noun != "EXTGEO" {
                            if let Err(error) = &member.points {
                                cable_errors
                                    .push(format!("{file} member {}: {error}", member.refno));
                            }
                            continue;
                        }
                        let mref = member.refno;
                        let element = set.element(mref);
                        let xoff = number_or_zero(&element, "XOFF").expect("EXTGEO XOFF 必须可读");
                        let ygap = number_or_zero(&element, "YGAP").expect("EXTGEO YGAP 必须可读");
                        let start_ref = reference(&element, "STAREF")
                            .expect("EXTGEO STAREF 必须可读")
                            .map(|el| el.refno().to_string());
                        let end_ref = reference(&element, "ENDREF")
                            .expect("EXTGEO ENDREF 必须可读")
                            .map(|el| el.refno().to_string());
                        extgeo_segments += 1;
                        let pts = match &member.points {
                            Ok(pts) if pts.len() == 2 => pts,
                            result => {
                                let reason = match result {
                                    Err(error) => error.to_string(),
                                    Ok(_) => "锚点引用为空、类型不支持或没有路由".into(),
                                };
                                extgeo_unresolved.push(format!(
                                    "{file} cable {} extgeo {mref}: {reason}",
                                    cable.refno()
                                ));
                                samples.push(serde_json::json!({
                                    "file": file, "cable": cable.refno().to_string(),
                                    "source_file": path, "extgeo": mref.to_string(),
                                    "start_ref": start_ref, "end_ref": end_ref,
                                    "xoff_mm": xoff, "ygap_mm": ygap,
                                    "status": "unresolved", "reason": reason,
                                }));
                                continue;
                            }
                        };
                        extgeo_resolved += 1;
                        let before = index.checked_sub(1).and_then(|i| {
                            members[i]
                                .points
                                .as_ref()
                                .ok()?
                                .last()
                                .map(|p| (members[i].refno, members[i].noun.clone(), *p))
                        });
                        let after = members.get(index + 1).and_then(|member| {
                            member
                                .points
                                .as_ref()
                                .ok()?
                                .first()
                                .map(|p| (member.refno, member.noun.clone(), *p))
                        });
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
                        let member = set.element(mref);
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
                        let before_gap = before.as_ref().map(|(_, _, p)| p.distance(pts[0]));
                        let after_gap = after.as_ref().map(|(_, _, p)| p.distance(pts[1]));
                        if let Some((r, n, p)) = before {
                            line += &format!(" | prev {r}({n}) gap={:.1}mm", p.distance(pts[0]));
                        }
                        if let Some((r, n, p)) = after {
                            line += &format!(" | next {r}({n}) gap={:.1}mm", p.distance(pts[1]));
                        }
                        line += &format!(" | XOFF={xoff} YGAP={ygap}");
                        samples.push(serde_json::json!({
                            "file": file, "cable": cable.refno().to_string(),
                            "source_file": path, "extgeo": mref.to_string(),
                            "start_ref": start_ref, "end_ref": end_ref, "status": "anchors_only",
                            "start_mm": pts[0].to_array(), "end_mm": pts[1].to_array(),
                            "before_gap_mm": before_gap, "after_gap_mm": after_gap,
                            "xoff_mm": xoff, "ygap_mm": ygap,
                        }));
                        joints.push(line);
                    }
                }
            }
        }
        samples.sort_by_key(|sample| {
            (
                sample["source_file"].to_string(),
                sample["extgeo"].to_string(),
            )
        });
        let offset_samples = samples
            .iter()
            .filter(|sample| {
                sample["xoff_mm"].as_f64().unwrap_or_default() != 0.0
                    || sample["ygap_mm"].as_f64().unwrap_or_default() != 0.0
            })
            .count();
        eprintln!("nonzero_offset_samples={offset_samples}");
        cable_errors.sort();
        joints.sort();
        extgeo_unresolved.sort();
        eprintln!(
            "scanned_cables={scanned_cables} scanned_extgeo={scanned_extgeo} cables={cables} extgeo={extgeo_segments} resolved={extgeo_resolved} unresolved={} cable_errors={}",
            extgeo_unresolved.len(),
            cable_errors.len()
        );
        if let Some(path) = std::env::var_os("AIOS_CABLE_PROBE_REPORT") {
            let report = serde_json::json!({
                "scanned_cables": scanned_cables, "scanned_extgeo": scanned_extgeo,
                "cables": cables, "extgeo": extgeo_segments, "resolved": extgeo_resolved,
                "unresolved": extgeo_unresolved.len(), "cable_errors": cable_errors,
                "samples": samples, "corpus_dirs": dirs,
                "nonzero_offset_samples": offset_samples,
                "geometry_validated": false,
            });
            std::fs::write(path, serde_json::to_vec_pretty(&report).unwrap()).unwrap();
        }
        for line in &cable_errors {
            eprintln!("  CABLE_ERROR {line}");
        }
        for line in &joints {
            eprintln!("  {line}");
        }
        for line in &extgeo_unresolved {
            eprintln!("  UNRESOLVED {line}");
        }
        assert!(scanned_cables > 0 && scanned_extgeo > 0);
        assert_eq!(cables, scanned_cables, "CABLE 解析失败不得静默漏计");
        assert_eq!(extgeo_segments, scanned_extgeo, "EXTGEO 成员不得静默漏计");
        assert_eq!(extgeo_resolved + extgeo_unresolved.len(), extgeo_segments);
        if std::env::var("AIOS_CABLE_PROBE_STRICT").as_deref() == Ok("1") {
            assert!(cable_errors.is_empty(), "CABLE 路由仍有解析错误");
            assert!(extgeo_unresolved.is_empty(), "EXTGEO 锚点仍有未解项");
        }
    }
}

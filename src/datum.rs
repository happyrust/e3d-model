//! GENSEC 名下的隐式标架：`JLDATU`（沿 SPINE 的站位）与 `PLDATU`（站位上某条截面 p-line）。
//!
//! 这两级在模板里都没有 POS/ORI，[`crate::transform::local_matrix`] 在它们身上给的是
//! 单位阵，于是挂在 GENSEC 上的 FIXING——连同它 TMPL 里的一切——全落在 GENSEC 原点
//! 附近。E3D 现算（`pllib/GPLDOP@0x103ADC48` 沿路径定站位，`pllib/GPLDAT@0x103AD778`
//! 定 p-line 标架）：
//!
//! - JLDATU：从 `HPREF` 指的 POINSP（未设 = SPINE 起点）沿路径走 `ZDIS`，落在两端之外
//!   时沿端切向直线外推。站位标架 = 该处的截面标架（X = Ydir × 切向，Y = 沿路径传递到
//!   此处的 SPINE Ydir，Z = 切向），与 [`crate::section::spine_swept_solid`] 同一口径。
//! - PLDATU：原点 = 站位 + 截面平面内 `POSL` 那条 p-line 的位置（减 JUSL 对齐、`LMIRR`
//!   镜像，再转 `BANG + 中性轴角`，与截面放置同一套）；GPLDAT 取 Z = p-line 的 PLAX 方向
//!   `(cos θ, sin θ, 0)`（`LMIRR` 时 x 取负），Y = `YDIR`（截面标架里的向量）去掉 Z 分量后
//!   归一，`ORNMAT(Z, Y)`。
//! - ENDATU：GENSEC 端部的 datum，直接挂在宿主名下、自己带 `ZDIS` / `POSL` / `YDIR`，没有
//!   JLDATU 一级。挂在哪一头看自带的 `HANICP`（[`endatu_at_end`]）：起点那头从 SPINE 起点走
//!   `ZDIS`、三轴就是站位的截面放置标架（Z = 切向）；终点那头从终点往回走 `ZDIS`、三轴绕截面
//!   Y 转 180°（Z = −切向），都让 Z 朝杆内。原点移到 `POSL` 那条 p-line（同 PLDATU 的位置
//!   口径），旋转不走 GPLDAT。E3D TTY 真值：ams7324 / ams7329 / ams7350 全部 258 只 ENDATU（起点 6、
//!   终点 252；`POSL` NA 68、BNA 190）逐件 ≤ 0.5 mm / 1e-5；未接的时候它们名下的 FIXING 丢了
//!   端部标架（gen-model `verification/fixing-tty-20261021/`）。
//! - SPINE 长度为 0（ams7330 `/SUPPORT/SPECIALS/BS/PIPE/3-BAR-2`、`4-BAR-2`，两点都在原点）：E3D 求不出
//!   站位，TTY 对 JLDATU 报 `pos;xdir;ydir;zdir` 求值失败，PLDATU 却照样有标架：站位在宿主原点，
//!   截面标架取单位阵（切向 +Z、Ydir +Y），不转 `BANG` 与中性轴角（这两根 `BANG` = −90°，转了就
//!   差 90°），其余照 GPLDAT。JLDATU / ENDATU 自己不给标架。

use std::f64::consts::FRAC_PI_2;

use glam::{DMat3, DMat4, DVec3};

use e3d_io::db_element::DbElement;
use e3d_io::db_param_env::DbElementParamEnv;

use crate::elmodl::ModelError;
use crate::section::{
    DEFAULT_JUSTIFICATION_LINE, SectionChain, SpinePath, SpineSegment, member_frame, rotate_about,
    rotate_between, section_catalogue, spine_path,
};

/// 解一个 GENSEC 名下 datum 用到的全部中间量；取证探针直接看这些。
#[derive(Debug, Clone)]
pub struct DatumSolve {
    pub host: DbElement,
    pub host_world: DMat4,
    pub zdis: f64,
    pub pkdi: f64,
    /// `HPREF` 那只 POINSP 在路径上的弧长；未设为 0。
    pub hpref_s: f64,
    pub path_length: f64,
    /// 站位（GENSEC 局部系）：点、单位切向、传递到此处的单位 Ydir。
    pub point: DVec3,
    pub tangent: DVec3,
    pub ydir: DVec3,
    /// `BANG + 中性轴转角`（弧度）。
    pub spin: f64,
    pub mirrored: bool,
    pub justification: [f64; 2],
    /// PLDATU 才有：`POSL`、那条 p-line 的平面位置与 PLAX 角、`YDIR` 原值。
    pub posl: Option<String>,
    pub pline_offset: Option<[f64; 2]>,
    pub pline_angle: Option<f64>,
    pub pl_ydir: Option<DVec3>,
    /// ENDATU：站位标架平移到 `POSL` 的 p-line，旋转不走 GPLDAT（见模块头）。
    pub end_datum: bool,
    /// ENDATU 挂在 SPINE 终点那头：站位从终点往回量 `ZDIS`，三轴绕截面 Y 转 180° 让 Z 朝杆内。
    pub at_end: bool,
    pub notes: Vec<String>,
}

impl DatumSolve {
    /// 站位的截面标架（不含 spin）：列 = (Ydir × 切向, Ydir, 切向)。
    pub fn section_rotation(&self) -> DMat3 {
        let x = self.ydir.cross(self.tangent).normalize();
        DMat3::from_cols(x, self.ydir, self.tangent)
    }

    /// 截面平面（放置后）→ GENSEC 局部系的旋转：截面标架再绕 Z 转 spin。
    pub fn profile_rotation(&self) -> DMat3 {
        self.section_rotation() * DMat3::from_rotation_z(self.spin)
    }

    /// p-line 在放置后截面平面里的位置：减对齐、镜像。
    pub fn placed_pline_offset(&self) -> Option<[f64; 2]> {
        let [px, py] = self.pline_offset?;
        let x = px - self.justification[0];
        Some([
            if self.mirrored { -x } else { x },
            py - self.justification[1],
        ])
    }

    /// p-line 的 PLAX 方向（放置后截面平面里）。
    pub fn placed_pline_axis(&self) -> Option<DVec3> {
        let angle = self.pline_angle?;
        let (sin, cos) = angle.sin_cos();
        Some(DVec3::new(if self.mirrored { -cos } else { cos }, sin, 0.0))
    }
}

/// 一只 datum 的宿主 GENSEC 与它自己的站位元素（PLDATU 取属主 JLDATU，JLDATU / ENDATU 取自身）。
fn host_and_joint(datum: &DbElement) -> Result<Option<(DbElement, DbElement)>, ModelError> {
    let joint = match datum.element_type()?.as_str() {
        "JLDATU" | "ENDATU" => datum.clone(),
        "PLDATU" => match datum.owner()? {
            Some(owner) if owner.element_type()? == "JLDATU" => owner,
            _ => return Ok(None),
        },
        _ => return Ok(None),
    };
    match joint.owner()? {
        Some(host) if host.element_type()? == "GENSEC" => Ok(Some((host, joint))),
        _ => Ok(None),
    }
}

/// 这只元素是不是 GENSEC 名下、标架要现算的 JLDATU / PLDATU / ENDATU（只看属主链，不求标架）。
pub fn is_gensec_datum(el: &DbElement) -> Result<bool, ModelError> {
    Ok(host_and_joint(el)?.is_some())
}

pub fn solve(datum: &DbElement) -> Result<Option<DatumSolve>, ModelError> {
    let Some((host, joint)) = host_and_joint(datum)? else {
        return Ok(None);
    };
    let mut notes = Vec::new();
    let Some(path) = spine_path(&host)? else {
        return Ok(None);
    };
    let stationless = path.segments.is_empty();
    if stationless && datum.element_type()? != "PLDATU" {
        return Ok(None);
    }
    let zdis = joint.get_double("ZDIS")?.unwrap_or(0.0);
    let pkdi = joint.get_double("PKDI")?.unwrap_or(0.0);
    let path_length: f64 = path.segments.iter().map(SpineSegment::length).sum();
    // ENDATU 的模板里没有 HPREF：缺属性与未设同义，从 SPINE 起点量。
    let hpref = match joint.get_element("HPREF") {
        Ok(point) => point,
        Err(e3d_io::db_element::DbElementError::UnknownAttribute { .. }) => None,
        Err(error) => return Err(error.into()),
    };
    let hpref_s = match hpref {
        Some(point) if point.element_type()? == "POINSP" => match point_on_host(&host, &point)? {
            Some(local) => path_parameter(&path, local),
            None => 0.0,
        },
        _ => 0.0,
    };
    let end_datum = datum.element_type()? == "ENDATU";
    let at_end = end_datum && endatu_at_end(datum, &path)?;
    let (point, tangent, ydir) = if stationless {
        notes.push("SPINE 长度为 0，没有站位：截面标架取宿主原点上的单位阵".into());
        (DVec3::ZERO, DVec3::Z, DVec3::Y)
    } else if at_end {
        station(&path, path_length - zdis, &mut notes)
    } else {
        station(&path, hpref_s + zdis, &mut notes)
    };

    let mut spin = if stationless {
        0.0
    } else {
        host.get_double("BANG")?.unwrap_or(0.0).to_radians()
    };
    let mirrored = matches!(host.get_bool("LMIRR"), Ok(Some(true)));
    let mut justification = [0.0; 2];
    let mut posl = None;
    let mut pline_offset = None;
    let mut pline_angle = None;
    let mut pl_ydir = None;
    if end_datum || datum.element_type()? == "PLDATU" {
        // 未设的 POSL 记成空串：不去截面里找线（见 [`datum_local`]）。
        let key = datum
            .get_string("POSL")?
            .map(|text| text.trim().to_ascii_uppercase())
            .unwrap_or_default();
        pl_ydir = datum.get_direction("YDIR")?.map(DVec3::from_array);
        posl = Some(key);
    }
    // 截面那边只要三样：JUSL 线的位置、中性轴线的转角（与 `section::evaluate_section_profile`
    // 同一口径）、POSL 那条线；环不用求。
    match section_catalogue(&host)? {
        SectionChain::Resolved(catalogue) => {
            let env = DbElementParamEnv::new(Some(host.clone()), Some(catalogue.sprf.clone()))
                .without_insulation();
            let line = |key: &str| -> Result<Option<crate::section::MemberFrame>, ModelError> {
                match find_pline(&catalogue.sprf, key)? {
                    Some(pline) => Ok(member_frame(&pline, &env).ok()),
                    None => Ok(None),
                }
            };
            let jusl = host
                .get_string("JUSL")
                .ok()
                .flatten()
                .map(|text| text.trim().to_ascii_uppercase())
                .filter(|text| !text.is_empty())
                .unwrap_or_else(|| DEFAULT_JUSTIFICATION_LINE.into());
            if let Some(frame) = line(&jusl)? {
                justification = frame.offset;
            }
            if !stationless {
                if let Some(frame) = line("NA")? {
                    spin -= frame.rotation;
                }
            }
            if let Some(key) = posl.as_ref().filter(|key| !key.is_empty()) {
                match line(key)? {
                    Some(frame) => {
                        pline_offset = Some(frame.offset);
                        pline_angle = Some(frame.rotation + FRAC_PI_2);
                    }
                    None => notes.push(format!(
                        "宿主截面 {} 没有 PKEY={key} 的 p-line",
                        catalogue.sprf.refno()
                    )),
                }
            }
        }
        other => notes.push(format!("宿主截面目录链未通：{other:?}")),
    }
    Ok(Some(DatumSolve {
        host_world: crate::transform::world_matrix(&host)?,
        host,
        zdis,
        pkdi,
        hpref_s,
        path_length,
        point,
        tangent,
        ydir,
        spin,
        mirrored,
        justification,
        posl,
        pline_offset,
        pline_angle,
        pl_ydir,
        end_datum,
        at_end,
        notes,
    }))
}

/// ENDATU 挂在 SPINE 哪一头：它自带的 `HANICP`（宿主局部系的端点）离终点比离起点近就是终点。
///
/// E3D TTY 真值里 258 只 ENDATU 按这条分头全部对上：`HANICP` = (0, 0, 0) 的 6 只在起点，其余
/// 252 只的 `HANICP` 就是终点（如 `/SUPPORT/FRAMES/BS/FT56/5-V1/PNOD`，`HANICP` = (0, 0, −313)，
/// 杆长 313）。缺属性、缺分量或两头一样近按起点。
fn endatu_at_end(datum: &DbElement, path: &SpinePath) -> Result<bool, ModelError> {
    let hanicp = match datum.get_double_array("HANICP") {
        Ok(value) => value,
        Err(e3d_io::db_element::DbElementError::UnknownAttribute { .. }) => None,
        Err(error) => return Err(error.into()),
    };
    let (Some([x, y, z, ..]), Some(first), Some(last)) = (
        hanicp.as_deref(),
        path.segments.first(),
        path.segments.last(),
    ) else {
        return Ok(false);
    };
    let handle = DVec3::new(*x, *y, *z);
    Ok((handle - last.end()).length() < (handle - first.start()).length())
}

/// 截面里的一条 p-line：平面位置 `(PX, PY)` 与 PLAX 角（弧度）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct SectionPline {
    pub(crate) offset: [f64; 2],
    pub(crate) angle: f64,
}

/// 宿主截面里 `key` 那条 p-line（与 [`solve`] 同一套求值）；目录链不通或没有这条线给 `None`。
/// 墙上 FIXING 的 PLDATU 也走它（`elmodl::wall_pline_datum`）。
pub(crate) fn section_pline(
    host: &DbElement,
    key: &str,
) -> Result<Option<SectionPline>, ModelError> {
    let SectionChain::Resolved(catalogue) = section_catalogue(host)? else {
        return Ok(None);
    };
    let env = DbElementParamEnv::new(Some(host.clone()), Some(catalogue.sprf.clone()))
        .without_insulation();
    Ok(match find_pline(&catalogue.sprf, key)? {
        Some(pline) => member_frame(&pline, &env).ok().map(|frame| SectionPline {
            offset: frame.offset,
            angle: frame.rotation + FRAC_PI_2,
        }),
        None => None,
    })
}

fn find_pline(sprf: &DbElement, key: &str) -> Result<Option<DbElement>, ModelError> {
    let Some(ptss) = sprf.get_element("PSTR")? else {
        return Ok(None);
    };
    for pline in ptss.members()? {
        if !pline.element_type()?.eq_ignore_ascii_case("PLIN") {
            continue;
        }
        let pkey = pline.get_string("PKEY")?.unwrap_or_default();
        if pkey.trim().eq_ignore_ascii_case(key) {
            return Ok(Some(pline));
        }
    }
    Ok(None)
}

/// POINSP 在 GENSEC 局部系里的位置（经 SPINE 的 POS/ORI）。
fn point_on_host(host: &DbElement, point: &DbElement) -> Result<Option<DVec3>, ModelError> {
    let Some(spine) = point.owner()? else {
        return Ok(None);
    };
    if spine.element_type()? != "SPINE"
        || spine.owner()?.map(|owner| owner.refno()) != Some(host.refno())
    {
        return Ok(None);
    }
    let local = point
        .get_position("POS")?
        .map(DVec3::from_array)
        .unwrap_or(DVec3::ZERO);
    Ok(Some(
        crate::transform::local_matrix(&spine)?.transform_point3(local),
    ))
}

/// 路径上离 `p` 最近那点的弧长。
fn path_parameter(path: &SpinePath, p: DVec3) -> f64 {
    let mut best = (f64::INFINITY, 0.0);
    let mut travelled = 0.0;
    for segment in &path.segments {
        let (distance, along) = match segment {
            SpineSegment::Straight { start, end } => {
                let d = *end - *start;
                let t = ((p - *start).dot(d) / d.length_squared()).clamp(0.0, 1.0);
                ((*start + d * t - p).length(), d.length() * t)
            }
            SpineSegment::Arc {
                start,
                centre,
                axis,
                sweep,
                ..
            } => {
                let r0 = *start - *centre;
                let v = p - *centre;
                let v = v - *axis * v.dot(*axis);
                let mut angle = axis.dot(r0.cross(v)).atan2(r0.dot(v));
                if angle < 0.0 {
                    angle += std::f64::consts::TAU;
                }
                let angle = angle.min(*sweep);
                let q = *centre + rotate_about(r0, *axis, angle);
                ((q - p).length(), r0.length() * angle)
            }
        };
        if distance < best.0 {
            best = (distance, travelled + along);
        }
        travelled += segment.length();
    }
    best.1
}

/// 路径上弧长 `s` 处的（点, 单位切向, 单位 Ydir）。两端之外沿端切向直线外推。
fn station(path: &SpinePath, s: f64, notes: &mut Vec<String>) -> (DVec3, DVec3, DVec3) {
    let (first_tangent, _) = path.segments[0].tangents();
    let mut ydir = path.ydir - first_tangent * path.ydir.dot(first_tangent);
    if ydir.length() < 1e-9 {
        let reference = if first_tangent.x.hypot(first_tangent.y) > 5e-6 {
            DVec3::Z
        } else {
            DVec3::Y
        };
        ydir = first_tangent.cross(reference.cross(first_tangent));
        notes.push("SPINE YDIR 与首段切向平行，改按 GTDIRS 参考轴".into());
    }
    let mut ydir = ydir.normalize();
    if s <= 0.0 {
        return (
            path.segments[0].start() + first_tangent * s,
            first_tangent,
            ydir,
        );
    }
    let mut remaining = s;
    let mut previous: Option<DVec3> = None;
    let count = path.segments.len();
    for (index, segment) in path.segments.iter().enumerate() {
        let (start_tangent, end_tangent) = segment.tangents();
        if let Some(previous) = previous {
            ydir = rotate_between(ydir, previous, start_tangent);
            ydir = (ydir - start_tangent * ydir.dot(start_tangent)).normalize();
        }
        let length = segment.length();
        let last = index + 1 == count;
        if remaining <= length || last {
            return match segment {
                SpineSegment::Straight { start, .. } => {
                    (*start + start_tangent * remaining, start_tangent, ydir)
                }
                SpineSegment::Arc {
                    start,
                    centre,
                    axis,
                    sweep,
                    ..
                } => {
                    let r0 = *start - *centre;
                    let radius = r0.length();
                    let angle = (remaining / radius).min(*sweep);
                    let overshoot = remaining - angle * radius;
                    let q = *centre + rotate_about(r0, *axis, angle);
                    let t = axis.cross(q - *centre).normalize();
                    let y = rotate_about(ydir, *axis, angle);
                    (q + t * overshoot.max(0.0), t, y)
                }
            };
        }
        if let SpineSegment::Arc { axis, sweep, .. } = segment {
            ydir = rotate_about(ydir, *axis, *sweep);
        }
        remaining -= length;
        previous = Some(end_tangent);
    }
    unreachable!("路径至少一段，末段必返回")
}

/// JLDATU / PLDATU / ENDATU（宿主是 GENSEC）的世界标架；别的元素交 `None`。
pub fn gensec_datum_world(datum: &DbElement) -> Result<Option<DMat4>, ModelError> {
    let Some(solve) = solve(datum)? else {
        return Ok(None);
    };
    Ok(Some(solve.host_world * datum_local(&solve)))
}

/// datum 在宿主 GENSEC 局部系里的标架。
pub fn datum_local(solve: &DatumSolve) -> DMat4 {
    let profile = solve.profile_rotation();
    if solve.end_datum {
        let offset = solve.placed_pline_offset().unwrap_or([0.0; 2]);
        let origin = solve.point + profile * DVec3::new(offset[0], offset[1], 0.0);
        let rotation = if solve.at_end {
            DMat3::from_cols(-profile.x_axis, profile.y_axis, -profile.z_axis)
        } else {
            profile
        };
        return DMat4::from_translation(origin) * DMat4::from_mat3(rotation);
    }
    if solve.posl.is_none() {
        return DMat4::from_translation(solve.point) * DMat4::from_mat3(profile);
    }
    // POSL 未设、或宿主截面里没有这条线：GPLDAT 的角取 0、位置留在 JUSL 线上。
    let offset = solve.placed_pline_offset().unwrap_or([0.0; 2]);
    let axis = solve.placed_pline_axis().unwrap_or(if solve.mirrored {
        DVec3::NEG_X
    } else {
        DVec3::X
    });
    let z = profile * axis;
    // YDIR 是截面放置标架（含 spin、不含镜像）里的向量；投影退化时 GPLDAT 取 VUNIT(3)，
    // 即该标架的 +Z = 站位切向。
    let raw_y = profile * solve.pl_ydir.unwrap_or(DVec3::Z);
    let mut y = raw_y - z * raw_y.dot(z);
    if y.length() < 1e-9 {
        y = solve.tangent;
    }
    let y = y.normalize();
    let x = y.cross(z);
    let origin = solve.point + profile * DVec3::new(offset[0], offset[1], 0.0);
    DMat4::from_translation(origin) * DMat4::from_mat3(DMat3::from_cols(x, y, z))
}

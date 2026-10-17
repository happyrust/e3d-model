//! 电缆桥架件（`CTSTRA`/`CTBEND`/`CTRISE`/`CTTEE`/`CTCROS`/`CTREDU`/`CTCOUP`）的现算放置。
//!
//! 这七个 noun 的属性表里**没有** `POS`/`ORI`（`CTRAY` 有，存的是 (0,0,0)）。E3D 把它们的
//! `POS`/`ORI` 注册成伪属性——Core3D `CAB_DB_PseudoPOSonCTELE::getAtt`（`sub_109C1420`）与
//! `CAB_DB_PseudoORIonCTELE::getAtt`（`sub_109C0940`）——每次读都按
//! `POSREF → RPATH 上的 POINTR` 加 `ZDIS/PKDI/ZOFF/BANG` 现算。本模块是那套算法的移植：
//!
//! | Core3D（Core3D.dll，idalib-35724） | 本模块 |
//! |---|---|
//! | `CAB_DBPseudoPOSandORIonCTMTRL::readMiddleFormDB`（`sub_109E7B60`） | [`placement`] 的属性读取 + 无 `POSREF` 兜底 |
//! | `MDR_DB_POSandORIonRPATH::readFormDB`（`sub_109E71E0`） | `resolve_anchor`：沿 `POSREF` 链落到路径点，定出 A→B 段 |
//! | `calculatePOS`（`sub_109E4DC0`）直线支 | `straight_position` |
//! | `calculateORI`（`sub_109E39D0`）直线支 | `straight_rotation` |
//! | `takeUpDir`（`sub_109E8EA0`） | `RoutePath::leave_up`（POINTR 的 `LYDIR`），兜底 `+Z` |
//! | `shiftAccordingToJustification`（`sub_109E83C0`） | `justification_shift` |
//! | `MDR_DB_PseudoAYDIR/LYDIR/PDIRonPOINTR`（`sub_105C0BA0/105C9730/105C9840`）、`RotateVector`（`sub_105CF730`） | `RoutePath` |
//!
//! 反编译原文全在 `evidence/20260902-hvac-ctray-census/`（`core3d-CAB_DB*.c`、
//! `core3d-MDR_DB*.c`、`core3d-RotateVector-*.c`、`libgeom-*.json`），README §三是它们的索引。
//!
//! # 坐标系
//!
//! 算出来的是 CTELE **相对属主**（`CTMTRL`）的局部放置：`POINTR.POS` 相对 `CWBRAN`（`RPATH`
//! 无 POS/ORI），`CTMTRL` 也无 POS/ORI，两者是同一个系；Core 只在带 `WRT` 的查询里才另做变换
//! （`sub_105D9AB0`），本模块不带 WRT。局部轴：**Y = 路径方向**（A→B），**Z = 路径的 up**
//! （`RPATH.YDIR` 经 `CWBRAN.BANG` 与各拐点的转折传播到本段），X = Y × Z（右手，朝行进方向的右侧）。
//! 与 `CTRAY` 目录几何一致：侧轨沿 +Y 拉 `PARA[10]`、横档跨 ±X、托盘底在 Z=0。
//!
//! # 移植范围
//!
//! 只移植了 **A、B 都是 `POINTR`、`FRAD = 0`** 的直线段——语料（ams251142–251161）里 RPATH 成员
//! 全是 POINTR、`FRAD` 全零。`CURVE`/`RSECT`/`EXTGEO` 段（`CURTYP/UBULG/PPOS[1..7]` 那支）与倒圆
//! 拐点（`PPOS[5]`/`FSTART` 语义未钉死）碰到就报 [`DbElementError::PseudoAttribute`]，不猜。
//!
//! Core 在几种数据关系断掉的情形下（`CWBRAN` 下没有 `RPATH`、`POSREF` 链落到别的 `CWBRAN`、
//! 路径只有一个点）会**成功返回 (0,0,0)**——托盘画在 CWBRAN 原点。无 POSREF 且无 RPATH
//! 的分支遵循该默认放置，带独立 anchor 以便诊断；其他断链仍保留原有错误。
//!
//! # 增量级联的欠账
//!
//! [`crate::transform::PLACEMENT_ATTRIBUTES`] 只列 `POS`/`ORI`。桥架件的世界矩阵还吃
//! `POSREF/ZDIS/PKDI/ZOFF/BANG`（自身）、`BANG/CLOSEF/CWJUSL/VWIDT/VHEIG/VSHAPE`（`CWBRAN`）、
//! `YDIR`（`RPATH`）与整条路径每个 `POINTR.POS`——跨元素的依赖，
//! [`crate::element_diff::ElementDiff::placement_input_changed`] 表达不了。改了路径点而不改
//! CTELE 自身，增量不会重建它下面的 `CTRAY`。这一条记在此处，等增量侧补「路径子树变更 →
//! 同 CWBRAN 下全部 CTELE 重建」的判据。

use std::f64::consts::SQRT_2;

use e3d_io::db_element::{DbElement, DbElementError};
use e3d_io::record::descriptor::DescriptorValue;
use glam::{DMat3, DMat4, DVec3};

use crate::rpath::{
    Resolved, RoutePath, failed, number_or_zero, optional, reference, resolve_anchor,
    resolve_without_posref, straight_point, straight_rotation, unit_or_zero, word,
};

pub use crate::rpath::Anchor;

/// 走 Core3D `CAB_DB_PseudoPOS/ORIonCTELE` 伪属性的七个 noun（`sub_109B7270` 注册表里
/// 逐个 `new` 的那七只）。
pub const CABLE_TRAY_ELEMENT_NOUNS: [&str; 7] = [
    "CTSTRA", "CTBEND", "CTRISE", "CTTEE", "CTCROS", "CTREDU", "CTCOUP",
];

/// 这个 noun 的 POS/ORI 是不是按本模块现算的。
pub fn is_cable_tray_element(noun: &str) -> bool {
    CABLE_TRAY_ELEMENT_NOUNS.contains(&noun)
}

/// 一只桥架件算出来的局部放置（相对属主 `CTMTRL`，见模块注释「坐标系」）。
#[derive(Debug, Clone, PartialEq)]
pub struct CableTrayPlacement {
    /// 局部位置（mm）。
    pub position: DVec3,
    /// 局部姿态，列 = 局部 X/Y/Z 轴在属主系里的方向。
    pub rotation: DMat3,
    /// 定位依据的路径点。
    pub anchor: Anchor,
}

/// 桥架件的局部矩阵 `T(POS) * R(ORI)`，给 [`crate::transform::local_matrix`] 用。
pub fn placement_matrix(el: &DbElement) -> Result<DMat4, DbElementError> {
    let placed = placement(el)?;
    Ok(DMat4::from_translation(placed.position) * DMat4::from_mat3(placed.rotation))
}

/// 现算一只桥架件的 POS/ORI（Core3D `CAB_DB_PseudoPOSonCTELE::getAtt` + `…ORIonCTELE::getAtt`，
/// 不带 WRT，即 `mode = 0`：全量对正偏移）。
///
/// `el` 必须是 [`CABLE_TRAY_ELEMENT_NOUNS`] 之一；解不出来报
/// [`DbElementError::PseudoAttribute`]，`detail` 里写清断在哪一环。
pub fn placement(el: &DbElement) -> Result<CableTrayPlacement, DbElementError> {
    let noun = el.element_type()?;
    if !is_cable_tray_element(&noun) {
        return Err(failed(el, "POS", format!("{noun} 不走桥架伪属性放置")));
    }
    // 包装函数：CWBRAN = owner(owner(CTELE))，无条件上溯两级。
    let ctmtrl = el
        .owner()?
        .ok_or_else(|| failed(el, "POS", "没有属主（应为 CTMTRL）"))?;
    let cwbran = ctmtrl.owner()?.ok_or_else(|| {
        failed(
            el,
            "POS",
            format!("属主 {} 没有属主（应为 CWBRAN）", ctmtrl.refno()),
        )
    })?;

    // readMiddleFormDB：四个数读不到就是 0。
    let pkdi = number_or_zero(el, "PKDI")?;
    let zdis = number_or_zero(el, "ZDIS")?;
    let zoff = number_or_zero(el, "ZOFF")?;
    let jusl = justification_offset(el, &cwbran)?;

    // 方向判定：直件/变径/接头在 PKDI = 0 时一律向前；其余看 PKDI、ZDIS 的符号。
    let forward_when_zero = matches!(noun.as_str(), "CTSTRA" | "CTREDU" | "CTCOUP");
    let posref = reference(el, "POSREF")?;
    // readMiddleFormDB (0x109E7B60) returns success when the RPATH search
    // reaches null. No endpoint was installed, so calculatePOS/ORI keep
    // their initial zero/identity values; the wrapper still applies angles.
    // Do not swallow errors for a present but broken POSREF or malformed path.
    if posref.is_none() && crate::rpath::first_rpath(&cwbran)?.is_none() {
        return Ok(CableTrayPlacement {
            position: DVec3::ZERO,
            rotation: apply_angles(el, DMat3::IDENTITY)?,
            anchor: Anchor::MissingRouteOrigin,
        });
    }
    let (path, anchor) = match posref {
        Some(posref) => resolve_anchor(el, &cwbran, posref, pkdi, zdis, forward_when_zero)?,
        None => resolve_without_posref(el, &cwbran, pkdi, zdis)?,
    };

    let (position, rotation) = match anchor {
        Resolved::Segment { a, b, pkdi, zdis } => {
            // Core 对非 STRA/COUP/REDU 且 ZDIS/PKDI 非零的件以 A.PPOS[2]（FRAD 出口
            // 切点）为基线在缩减段上量距；直件与零偏移件仍按原始角点。
            let point = if !matches!(noun.as_str(), "CTSTRA" | "CTCOUP" | "CTREDU")
                && (pkdi.abs() > crate::rpath::ATTR_TOL || zdis.abs() > crate::rpath::ATTR_TOL)
            {
                crate::rpath::filleted_point(&path, a, b, pkdi, zdis, zoff, jusl)
            } else {
                straight_point(&path, a, b, pkdi, zdis, zoff, jusl)
            };
            let mut position = point.position;
            if point.inside {
                position +=
                    justification_shift(el, &noun, &cwbran, &path, b, point.along, point.up)?;
            }
            let rotation = straight_rotation(el, &path, a, b)?;
            (position, rotation)
        }
        // readFormDB 兜底：POS = 端点 + ZOFF 直接加在 z 上；calculatePOS/ORI 因 A 或 B 为空
        // 走 LABEL_8 的空支，矩阵保持调用方传进来的单位阵。
        Resolved::EndPoint(index) => (
            path.points[index].pos + DVec3::new(0.0, 0.0, zoff),
            DMat3::IDENTITY,
        ),
    };

    Ok(CableTrayPlacement {
        position,
        rotation: apply_angles(el, rotation)?,
        anchor: anchor.anchor(&path),
    })
}

fn apply_angles(el: &DbElement, mut rotation: DMat3) -> Result<DMat3, DbElementError> {
    // CAB_DB_PseudoORIonCTELE::getAtt 尾巴：BANG 绕局部 Y（路径方向）、GANGLE 绕局部 Z，
    // 都是右乘（在局部系里转）。库里存的是度；Core 这里把读到的值直接喂给吃弧度的
    // `D3_Matrix::rotateY`（libgeom `sub_10035660` 直接 sincos），而同一个 BANG 在
    // `AYDIR`（`sub_105C0BA0`）里明确乘了 π/180——两处不一致，按度处理（语料里全是 0，
    // 数值上没区别；有非零样本时要拿 E3D 实测钉一次）。
    if let Some(bank) = optional(el.get_double("BANG"))?
        && bank != 0.0
    {
        rotation *= DMat3::from_rotation_y(bank.to_radians());
    }
    if let Some(gradient) = optional(el.get_double("GANGLE"))?
        && gradient != 0.0
    {
        rotation *= DMat3::from_rotation_z(gradient.to_radians());
    }

    Ok(rotation)
}

/// `readMiddleFormDB` 里 `XOFF` 那一格的口径：noun 上**有** `XOFF` 属性就把对正偏移置 0
/// （XOFF 本身由目录几何经 `ATTRIB RPRO XOFF` 消费，不在这里平移）；没有才退到
/// `WJUSL`（自身）× `WTHICK`（CWBRAN）——七个桥架 noun 都带 `XOFF`，这一支实际不会走到。
fn justification_offset(el: &DbElement, cwbran: &DbElement) -> Result<f64, DbElementError> {
    if el.has_attribute("XOFF")? {
        return Ok(0.0);
    }
    let Some(justification) = word(el, "WJUSL")? else {
        return Ok(0.0);
    };
    let Some(thickness) = optional(cwbran.get_double("WTHICK"))? else {
        return Ok(0.0);
    };
    Ok(match justification.to_ascii_uppercase().as_str() {
        "RIGH" => thickness * 0.5,
        "LEFT" => thickness * -0.5,
        _ => 0.0,
    })
}

/// `shiftAccordingToJustification`（`sub_109E83C0`），`mode = 0`（全量）。
///
/// 目录托盘的原点在**底面中线**上，路径线要落在别处就得把件反向平移：`CWJUSL` 的
/// `LEFT/RIGH` 横向挪半宽，`CEN*` 再往下半高，`TOP*` 往下整高；对不上的词（语料里的
/// `CENT`）不挪。`VSHAPE = 1` 不挪。`CTBEND` 的横向按两腿的角平分线取并除 `cos(ANGL/2)`
/// （斜接补偿）；`CTRISE` 的竖向取拐点前后两个 up 的平分线并把高度乘 √2。
/// `CTREDU` 多一段 `POSREF` 特判（`sub_109E6440`），语料里没有，未移植。
fn justification_shift(
    el: &DbElement,
    noun: &str,
    cwbran: &DbElement,
    path: &RoutePath,
    b: usize,
    along: DVec3,
    up: DVec3,
) -> Result<DVec3, DbElementError> {
    let Some(justification) = word(cwbran, "CWJUSL")? else {
        return Ok(DVec3::ZERO);
    };
    let width = number_or_zero(cwbran, "VWIDT")?;
    let mut height = number_or_zero(cwbran, "VHEIG")?;
    let shape_is_one = matches!(
        optional(cwbran.attribute("VSHAPE"))?,
        Some(DescriptorValue::Int(1)) | Some(DescriptorValue::Word { raw: 1, .. })
    );
    if shape_is_one {
        return Ok(DVec3::ZERO);
    }

    let lateral = if noun == "CTBEND" {
        let half_angle = number_or_zero(el, "ANGL")?.to_radians() * 0.5;
        let arrive = path.arrive_dir(b);
        let leave = path.leave_dir(b);
        unit_or_zero(arrive.cross(up) + leave.cross(up)) / half_angle.cos()
    } else {
        unit_or_zero(along.cross(up))
    };
    let downward = if noun == "CTRISE" {
        let arrive_up = path.arrive_up(b).unwrap_or(DVec3::ZERO);
        let leave_up = path.leave_up(b).unwrap_or(DVec3::ZERO);
        height *= SQRT_2;
        -unit_or_zero(unit_or_zero(arrive_up) + unit_or_zero(leave_up))
    } else {
        -unit_or_zero(up)
    };

    let half_width = lateral * (width * 0.5);
    let half_height = downward * (height * 0.5);
    let full_height = downward * height;
    Ok(match justification.to_ascii_uppercase().get(..4) {
        Some("LEFT") => half_width,
        Some("RIGH") => -half_width,
        Some("CENR") => half_height - half_width,
        Some("CENC") => half_height,
        Some("CENL") => half_height + half_width,
        Some("TOPR") => full_height - half_width,
        Some("TOPC") => full_height,
        Some("TOPL") => full_height + half_width,
        _ => DVec3::ZERO,
    })
}

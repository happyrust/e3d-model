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
//! 路径只有一个点）会**成功返回 (0,0,0)**——托盘画在 CWBRAN 原点。这里一律报错：几何画在错的地方
//! 是最高级别缺陷，报告里一条 `failed` 比一只静默落在原点的托盘便宜得多。
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
use std::fmt;

use e3d_io::db_element::{DbElement, DbElementError};
use e3d_io::record::descriptor::DescriptorValue;
use e3d_io::refno::RefNo;
use glam::{DMat3, DMat4, DQuat, DVec3};

/// 走 Core3D `CAB_DB_PseudoPOS/ORIonCTELE` 伪属性的七个 noun（`sub_109B7270` 注册表里
/// 逐个 `new` 的那七只）。
pub const CABLE_TRAY_ELEMENT_NOUNS: [&str; 7] = [
    "CTSTRA", "CTBEND", "CTRISE", "CTTEE", "CTCROS", "CTREDU", "CTCOUP",
];

/// `ZDIS/PKDI` 视同零、`XOFF/ZOFF` 视同不生效的门限（`calculatePOS` 里的 1e-8）。
const ATTR_TOL: f64 = 1e-8;
/// 段长退化门限（`calculatePOS`/`calculateORI` 里的 1e-10）。
const LENGTH_TOL: f64 = 1e-10;
/// 向量能不能归一化的门限。
const VECTOR_TOL: f64 = 1e-12;

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

/// 放置落在路径的哪一段上。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Anchor {
    /// 落在 A→B 这条直线段上（含两端；`ZDIS` 越界时钉在端点）。
    Segment { a: RefNo, b: RefNo },
    /// 路径在这一侧没有下一个点（Core `readFormDB` 的端点兜底）：
    /// POS = 该点 + (0,0,ZOFF)，ORI 单位阵（再叠 `BANG`）。
    EndPoint(RefNo),
}

impl fmt::Display for Anchor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Anchor::Segment { a, b } => write!(f, "{a}→{b}"),
            Anchor::EndPoint(point) => write!(f, "端点 {point}"),
        }
    }
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

    let (path, anchor) = match reference(el, "POSREF")? {
        Some(posref) => resolve_anchor(el, &noun, &cwbran, posref, pkdi, zdis)?,
        None => resolve_without_posref(el, &cwbran, pkdi, zdis)?,
    };

    let (position, mut rotation) = match anchor {
        Resolved::Segment { a, b, pkdi, zdis } => {
            let position =
                straight_position(el, &noun, &cwbran, &path, a, b, pkdi, zdis, zoff, jusl)?;
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

    Ok(CableTrayPlacement {
        position,
        rotation,
        anchor: match anchor {
            Resolved::Segment { a, b, .. } => Anchor::Segment {
                a: path.points[a].refno,
                b: path.points[b].refno,
            },
            Resolved::EndPoint(index) => Anchor::EndPoint(path.points[index].refno),
        },
    })
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

/// `readFormDB` 定出来的落点。`pkdi`/`zdis` 是沿 `POSREF` 链折算、并按行进方向翻转后的值。
#[derive(Debug, Clone, Copy)]
enum Resolved {
    Segment {
        a: usize,
        b: usize,
        pkdi: f64,
        zdis: f64,
    },
    EndPoint(usize),
}

/// `MDR_DB_POSandORIonRPATH::readFormDB`（`sub_109E71E0`）：沿 `POSREF` 链落到一个 `POINTR`，
/// 再按 `PKDI/ZDIS` 的符号决定这一段是从它往前（A = 它）还是往后（B = 它）。
fn resolve_anchor(
    el: &DbElement,
    noun: &str,
    cwbran: &DbElement,
    posref: DbElement,
    mut pkdi: f64,
    mut zdis: f64,
) -> Result<(RoutePath, Resolved), DbElementError> {
    let closed = flag(cwbran, "CLOSEF")?;

    // POSREF 指向另一只桥架件时逐级折算：PKDI 按比例复合，ZDIS 累加，直到落到 POINTR。
    let mut cursor = Some(posref);
    if cursor
        .as_ref()
        .map(|c| c.element_type())
        .transpose()?
        .as_deref()
        != Some("POINTR")
    {
        while let Some(current) = cursor.as_ref() {
            if !current.exists()? {
                cursor = None;
                break;
            }
            let hop_pkdi = number_or_zero(current, "PKDI")?;
            let hop_zdis = number_or_zero(current, "ZDIS")?;
            pkdi = if pkdi < 0.0 {
                (pkdi + 1.0) * hop_pkdi
            } else {
                (1.0 - hop_pkdi) * pkdi + hop_pkdi
            };
            zdis += hop_zdis;
            cursor = reference(current, "POSREF")?;
            if let Some(next) = cursor.as_ref()
                && next.element_type()? == "POINTR"
            {
                break;
            }
        }
    }

    let anchor_point = match cursor {
        Some(point) if point.exists()? => point,
        // 链断了：退到 CWBRAN 自己的 RPATH 第一个成员。
        _ => {
            let rpath = first_rpath(cwbran)?.ok_or_else(|| {
                failed(
                    el,
                    "POS",
                    format!(
                        "POSREF 链断了，且 CWBRAN {} 下没有 RPATH（Core 会给 (0,0,0)）",
                        cwbran.refno()
                    ),
                )
            })?;
            rpath.first_member()?.ok_or_else(|| {
                failed(
                    el,
                    "POS",
                    format!(
                        "POSREF 链断了，且 RPATH {} 没有成员（Core 会给 (0,0,0)）",
                        rpath.refno()
                    ),
                )
            })?
        }
    };

    // 落点必须挂在本 CWBRAN 的路径下；Core 对不上就原样返回 (0,0,0)。
    let rpath = anchor_point.owner()?.ok_or_else(|| {
        failed(
            el,
            "POS",
            format!("路径点 {} 没有属主", anchor_point.refno()),
        )
    })?;
    let rpath_owner = rpath.owner()?.map(|owner| owner.refno());
    if rpath_owner != Some(cwbran.refno()) {
        return Err(failed(
            el,
            "POS",
            format!(
                "POSREF 落到的 {} 属主链是 {} → {:?}，不在本 CWBRAN {} 下（Core 会给 (0,0,0)）",
                anchor_point.refno(),
                rpath.refno(),
                rpath_owner,
                cwbran.refno()
            ),
        ));
    }
    let path = RoutePath::load(el, &rpath, cwbran)?;
    let index = path.index_of(anchor_point.refno()).ok_or_else(|| {
        failed(
            el,
            "POS",
            format!(
                "路径点 {} 不在 {} 的成员表里",
                anchor_point.refno(),
                rpath.refno()
            ),
        )
    })?;
    let count = path.points.len();
    let last = count - 1;

    // 方向判定：直件/变径/接头在 PKDI = 0 时一律向前；其余看 PKDI、ZDIS 的符号。
    let forward = if pkdi == 0.0 && matches!(noun, "CTSTRA" | "CTREDU" | "CTCOUP") {
        true
    } else {
        pkdi > 0.0 || (pkdi == 0.0 && zdis > 0.0)
    };

    let (mut a, mut b): (Option<usize>, Option<usize>);
    if forward {
        if index != last {
            a = Some(index);
            b = Some(index + 1);
        } else {
            // A 已是末点：退一格，B 就是它自己。
            a = index.checked_sub(1);
            b = a.map(|a| a + 1);
        }
    } else {
        if index != 0 {
            b = Some(index);
            a = Some(index - 1);
            pkdi += 1.0;
            zdis = -zdis;
        } else {
            a = Some(0);
            b = (count > 1).then_some(1);
        }
        if a.is_none() {
            return Err(failed(
                el,
                "POS",
                "反向定位时前一个路径点为空（Core 返回失败）",
            ));
        }
        // Core 在这里还检查 A 是不是 CURVE 段；`RoutePath::load` 已把非 POINTR 成员拒掉。
    }

    // LABEL_46：端点兜底。
    if a.is_none() && b.is_none() {
        return Err(failed(
            el,
            "POS",
            format!(
                "路径 {} 只有一个点，定不出线段（Core 会给 (0,0,0)）",
                rpath.refno()
            ),
        ));
    }
    if a.is_none() {
        if !closed {
            let Some(b) = b else {
                unreachable!("上面已排除两端皆空")
            };
            return Ok((path, Resolved::EndPoint(b)));
        }
        a = Some(last);
    }
    if b.is_none() {
        if closed {
            b = Some(0);
        } else {
            let Some(a) = a else {
                unreachable!("A 为空的情形已在上面处理")
            };
            return Ok((path, Resolved::EndPoint(a)));
        }
    }
    let (Some(a), Some(b)) = (a, b) else {
        unreachable!("闭合路径补齐后 A、B 皆有值")
    };
    Ok((path, Resolved::Segment { a, b, pkdi, zdis }))
}

/// `readMiddleFormDB` 里没有 `POSREF` 的那一支：取 CWBRAN 的 RPATH 第一个 POINTR 为 A、
/// 它的下一个成员为 B；缺哪个 Core 都返回失败。
fn resolve_without_posref(
    el: &DbElement,
    cwbran: &DbElement,
    pkdi: f64,
    zdis: f64,
) -> Result<(RoutePath, Resolved), DbElementError> {
    let rpath = first_rpath(cwbran)?.ok_or_else(|| {
        failed(
            el,
            "POS",
            format!(
                "没有 POSREF，且 CWBRAN {} 下没有 RPATH（Core 会给 (0,0,0)）",
                cwbran.refno()
            ),
        )
    })?;
    let path = RoutePath::load(el, &rpath, cwbran)?;
    if path.points.is_empty() {
        return Err(failed(
            el,
            "POS",
            format!(
                "没有 POSREF，且 RPATH {} 下没有 POINTR（Core 返回失败）",
                rpath.refno()
            ),
        ));
    }
    if path.points.len() < 2 {
        return Err(failed(
            el,
            "POS",
            format!(
                "没有 POSREF，且 RPATH {} 只有一个点（Core 返回失败）",
                rpath.refno()
            ),
        ));
    }
    Ok((
        path,
        Resolved::Segment {
            a: 0,
            b: 1,
            pkdi,
            zdis,
        },
    ))
}

fn first_rpath(cwbran: &DbElement) -> Result<Option<DbElement>, DbElementError> {
    for member in cwbran.members()? {
        if member.element_type()? == "RPATH" {
            return Ok(Some(member));
        }
    }
    Ok(None)
}

/// `calculatePOS`（`sub_109E4DC0`）A、B 皆为 POINTR 的直线支。
///
/// Core 对非 STRA/COUP/REDU 且 `ZDIS`/`PKDI` 非零的件读 `A.PPOS[2]`、其余读 `A.POS`；
/// `FRAD = 0` 时 `PPOS[0..4]` 都是 `POS`（`MDR_DB_PseudoPPOSonPOINTR::getAtt` 0..4 同一支），
/// 这里统一用 `POS`（`RoutePath::load` 已把 `FRAD ≠ 0` 拒掉）。
#[allow(clippy::too_many_arguments)]
fn straight_position(
    el: &DbElement,
    noun: &str,
    cwbran: &DbElement,
    path: &RoutePath,
    a: usize,
    b: usize,
    pkdi: f64,
    zdis: f64,
    zoff: f64,
    jusl: f64,
) -> Result<DVec3, DbElementError> {
    let point_a = path.points[a].pos;
    let point_b = path.points[b].pos;
    let along = point_b - point_a;
    let length = along.length();
    let up = path.leave_up(a).unwrap_or(DVec3::Z);
    let distance = zdis + pkdi * length;

    // `dist >= 0`，或「纯 ZDIS 为负」（PKDI = 0 且 ZDIS < 0）允许从 A 往回量。
    if !(distance >= 0.0 || (pkdi == 0.0 && zdis < 0.0)) {
        return Ok(point_a + DVec3::new(0.0, 0.0, zoff));
    }
    if distance > length {
        return Ok(point_b + DVec3::new(0.0, 0.0, zoff));
    }
    if length < LENGTH_TOL {
        return Ok(point_a + DVec3::new(0.0, 0.0, zoff));
    }

    let mut position = point_a + along * (distance / length);
    if jusl.abs() > ATTR_TOL {
        position += unit_or_zero(along.cross(up)) * jusl;
    }
    if zoff.abs() > ATTR_TOL {
        position += unit_or_zero(up) * zoff;
    }
    position += justification_shift(el, noun, cwbran, path, b, along, up)?;
    Ok(position)
}

/// `calculateORI`（`sub_109E39D0`）直线支：X = 路径方向 × up，Y = 路径方向，Z = X × Y。
/// 段长退化时保持单位阵（Core 保留调用方传入的矩阵）。
fn straight_rotation(
    el: &DbElement,
    path: &RoutePath,
    a: usize,
    b: usize,
) -> Result<DMat3, DbElementError> {
    let along = path.points[b].pos - path.points[a].pos;
    if along.length() < LENGTH_TOL {
        return Ok(DMat3::IDENTITY);
    }
    let up = path.leave_up(a).unwrap_or(DVec3::Z);
    frame_xy(along.cross(up), along).ok_or_else(|| {
        failed(
            el,
            "ORI",
            format!(
                "段 {}→{} 的方向 {along:?} 与 up {up:?} 共线，定不出局部系",
                path.points[a].refno, path.points[b].refno
            ),
        )
    })
}

/// `D3_Transform::map(0, 0, X→x, Y→y)` 再 `getRot`：x 取准，y 在 x 的正交补里取，z = x × y。
fn frame_xy(x: DVec3, y: DVec3) -> Option<DMat3> {
    let x = unit(x)?;
    let y = unit(y - x * y.dot(x))?;
    Some(DMat3::from_cols(x, y, x.cross(y)))
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

/// 一条 `RPATH` 上按成员原序排好的 `POINTR`，连同算 up 向量要的 `CWBRAN.BANG/CLOSEF`
/// 与 `RPATH.YDIR`。POINTR 侧的伪属性 `PDIR`（到达/离开方向）、`AYDIR`/`LYDIR`（到达/离开 up）
/// 都在这里算，不再回库。
struct RoutePath {
    ydir: Option<DVec3>,
    /// `CWBRAN.BANG`（度）：路径起点 up 绕起始方向的预转。
    bank: f64,
    closed: bool,
    points: Vec<PathPoint>,
}

#[derive(Debug, Clone, Copy)]
struct PathPoint {
    refno: RefNo,
    pos: DVec3,
}

impl RoutePath {
    fn load(el: &DbElement, rpath: &DbElement, cwbran: &DbElement) -> Result<Self, DbElementError> {
        let mut points = Vec::new();
        for member in rpath.members()? {
            let noun = member.element_type()?;
            if noun != "POINTR" {
                return Err(failed(
                    el,
                    "POS",
                    format!(
                        "路径 {} 的成员 {} 是 {noun}：只移植了 POINTR 直线段，CURVE/RSECT/EXTGEO 那支未落地",
                        rpath.refno(),
                        member.refno()
                    ),
                ));
            }
            let frad = number_or_zero(&member, "FRAD")?;
            if frad.abs() > ATTR_TOL {
                return Err(failed(
                    el,
                    "POS",
                    format!(
                        "路径点 {} FRAD={frad}：倒圆拐点的 PPOS/PDIR 语义未钉死，不猜",
                        member.refno()
                    ),
                ));
            }
            points.push(PathPoint {
                refno: member.refno(),
                pos: vector(&member, "POS")?.unwrap_or(DVec3::ZERO),
            });
        }
        Ok(Self {
            ydir: vector(rpath, "YDIR")?,
            bank: number_or_zero(cwbran, "BANG")?,
            closed: flag(cwbran, "CLOSEF")?,
            points,
        })
    }

    fn index_of(&self, refno: RefNo) -> Option<usize> {
        self.points.iter().position(|point| point.refno == refno)
    }

    /// 前一个点：首点在闭合路径上接末点，开口路径上没有（`PDIR` 的 `prev/lastMember` 判法）。
    fn prev(&self, index: usize) -> Option<usize> {
        if index > 0 {
            Some(index - 1)
        } else if self.closed {
            Some(self.points.len() - 1)
        } else {
            None
        }
    }

    fn next(&self, index: usize) -> Option<usize> {
        if index + 1 < self.points.len() {
            Some(index + 1)
        } else if self.closed {
            Some(0)
        } else {
            None
        }
    }

    /// `PDIR[1]`：到达方向 = 本点 − 前一点，归一；没有前一点给零向量。
    fn arrive_dir(&self, index: usize) -> DVec3 {
        match self.prev(index) {
            Some(prev) => unit_or_zero(self.points[index].pos - self.points[prev].pos),
            None => DVec3::ZERO,
        }
    }

    /// `PDIR[2]`：离开方向 = 下一点 − 本点，归一；没有下一点给零向量。
    fn leave_dir(&self, index: usize) -> DVec3 {
        match self.next(index) {
            Some(next) => unit_or_zero(self.points[next].pos - self.points[index].pos),
            None => DVec3::ZERO,
        }
    }

    /// `AYDIR`（`sub_105C0BA0`）：到达本点时的 up。起点 up 取 `YDIR` 在起始方向正交补里的分量，
    /// 绕起始方向预转 `CWBRAN.BANG`，再逐个拐点按转折（到达→离开的最小旋转）传播到本点之前。
    /// `YDIR` 与起始方向共线时 Core 返回失败——调用方兜底 `+Z`。
    fn arrive_up(&self, index: usize) -> Option<DVec3> {
        let ydir = self.ydir?;
        let start_dir = self.leave_dir(0);
        let lateral = unit(ydir.cross(start_dir))?;
        let up0 = unit(start_dir.cross(lateral))?;
        let mut up = rotate_about(up0, start_dir, self.bank.to_radians());
        // 首点的转折是恒等（`sub_105CF860` 对 firstMember 原样返回），从第二个点起转。
        for turn in 1..index {
            up = self.turn(turn, up);
        }
        Some(up)
    }

    /// `LYDIR`（`sub_105C9730`）：离开本点时的 up = 到达 up 再过本点的转折。
    fn leave_up(&self, index: usize) -> Option<DVec3> {
        let up = self.arrive_up(index)?;
        Some(if index == 0 { up } else { self.turn(index, up) })
    }

    /// 过一个拐点：绕 `arrive × leave` 转 `angle(arrive, leave)`，即把到达方向带到离开方向的那次旋转。
    fn turn(&self, index: usize, up: DVec3) -> DVec3 {
        let arrive = self.arrive_dir(index);
        let leave = self.leave_dir(index);
        let rotated = rotate_about(up, arrive.cross(leave), angle_between(arrive, leave));
        unit(rotated).unwrap_or(up)
    }
}

/// `RotateVector`（`sub_105CF730`）：把 `axis` 映成 Z，在 XY 面转 `angle`（右手），再映回。
/// 轴退化（两方向共线）时 Core 的 `map` 装不出变换，角也是 0，原样返回。
fn rotate_about(vector: DVec3, axis: DVec3, angle: f64) -> DVec3 {
    match unit(axis) {
        Some(axis) if angle != 0.0 => DQuat::from_axis_angle(axis, angle) * vector,
        _ => vector,
    }
}

/// libgeom `D3_Vector::angle`：`atan2(|a×b|, a·b)`，两向量皆零给 0。
fn angle_between(a: DVec3, b: DVec3) -> f64 {
    let cross = a.cross(b).length();
    let dot = a.dot(b);
    if cross == 0.0 && dot == 0.0 {
        0.0
    } else {
        cross.atan2(dot)
    }
}

fn unit(vector: DVec3) -> Option<DVec3> {
    let length = vector.length();
    (length > VECTOR_TOL && length.is_finite()).then(|| vector / length)
}

fn unit_or_zero(vector: DVec3) -> DVec3 {
    unit(vector).unwrap_or(DVec3::ZERO)
}

// ---- 属性读取：E3D `getAtt` 口径 ------------------------------------------------

/// 属性不在该 noun 的模板里 = Core `getAtt` 返回 false；本模块把它与「unset」同样当缺省处理。
fn optional<T>(result: Result<Option<T>, DbElementError>) -> Result<Option<T>, DbElementError> {
    match result {
        Ok(value) => Ok(value),
        Err(DbElementError::UnknownAttribute { .. }) => Ok(None),
        Err(error) => Err(error),
    }
}

fn number_or_zero(el: &DbElement, name: &str) -> Result<f64, DbElementError> {
    Ok(optional(el.get_double(name))?.unwrap_or(0.0))
}

fn flag(el: &DbElement, name: &str) -> Result<bool, DbElementError> {
    Ok(optional(el.get_bool(name))?.unwrap_or(false))
}

fn reference(el: &DbElement, name: &str) -> Result<Option<DbElement>, DbElementError> {
    optional(el.get_element(name))
}

fn vector(el: &DbElement, name: &str) -> Result<Option<DVec3>, DbElementError> {
    Ok(optional(el.get_position(name))?.map(DVec3::from_array))
}

fn word(el: &DbElement, name: &str) -> Result<Option<String>, DbElementError> {
    optional(el.get_string(name))
}

fn failed(el: &DbElement, name: &str, detail: impl Into<String>) -> DbElementError {
    DbElementError::PseudoAttribute {
        refno: el.refno(),
        noun: el.element_type().unwrap_or_else(|_| "?".to_string()),
        name: name.to_string(),
        detail: detail.into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn refno(n: u32) -> RefNo {
        RefNo::new(9, n)
    }

    fn path(points: &[[f64; 3]], ydir: Option<DVec3>, bank: f64, closed: bool) -> RoutePath {
        RoutePath {
            ydir,
            bank,
            closed,
            points: points
                .iter()
                .enumerate()
                .map(|(i, p)| PathPoint {
                    refno: refno(i as u32 + 1),
                    pos: DVec3::from_array(*p),
                })
                .collect(),
        }
    }

    fn close(a: DVec3, b: DVec3) -> bool {
        (a - b).length() < 1e-9
    }

    /// 直路：YDIR = +Z、方向 +Y，up 就是 +Z，右手 X = Y × Z = +X。
    #[test]
    fn straight_path_up_is_ydir_and_frame_is_identity() {
        let path = path(
            &[[0.0, 0.0, 0.0], [0.0, 3000.0, 0.0], [0.0, 6000.0, 0.0]],
            Some(DVec3::Z),
            0.0,
            false,
        );
        assert!(close(path.leave_up(0).unwrap(), DVec3::Z));
        assert!(close(path.leave_up(1).unwrap(), DVec3::Z));
        let frame = frame_xy(DVec3::Y.cross(DVec3::Z), DVec3::Y).unwrap();
        assert!(close(frame.x_axis, DVec3::X));
        assert!(close(frame.y_axis, DVec3::Y));
        assert!(close(frame.z_axis, DVec3::Z));
    }

    /// 水平拐 90°：up 不变（绕 Z 转），到达/离开方向按邻点差归一。
    #[test]
    fn horizontal_turn_keeps_up_vertical() {
        let path = path(
            &[[0.0, 0.0, 0.0], [0.0, 1000.0, 0.0], [500.0, 1000.0, 0.0]],
            Some(DVec3::Z),
            0.0,
            false,
        );
        assert!(close(path.arrive_dir(1), DVec3::Y));
        assert!(close(path.leave_dir(1), DVec3::X));
        assert!(close(path.leave_up(1).unwrap(), DVec3::Z));
        assert!(close(path.leave_dir(2), DVec3::ZERO), "末点没有离开方向");
    }

    /// 竖向拐 90°（上升段）：up 跟着路径转成水平，之后再转回。
    #[test]
    fn vertical_riser_carries_up_around_the_bend() {
        let path = path(
            &[
                [0.0, 0.0, 0.0],
                [0.0, 1000.0, 0.0],
                [0.0, 1000.0, 800.0],
                [0.0, 1500.0, 800.0],
            ],
            Some(DVec3::Z),
            0.0,
            false,
        );
        // 段 1→2 朝 +Z 走，原本朝 +Z 的 up 被带到 -Y。
        assert!(
            close(path.leave_up(1).unwrap(), DVec3::NEG_Y),
            "{:?}",
            path.leave_up(1)
        );
        // 段 2→3 又回到 +Y，up 转回 +Z。
        assert!(
            close(path.leave_up(2).unwrap(), DVec3::Z),
            "{:?}",
            path.leave_up(2)
        );
        // 竖直段的局部系仍是右手且非退化。
        let frame = frame_xy(DVec3::Z.cross(DVec3::NEG_Y), DVec3::Z).unwrap();
        assert!(close(frame.x_axis, DVec3::X));
        assert!(close(frame.y_axis, DVec3::Z));
        assert!(close(frame.z_axis, DVec3::NEG_Y));
    }

    /// BANG 绕起始方向右手转；YDIR 与起始方向共线时 Core 失败，这里给 None。
    #[test]
    fn bank_rotates_start_up_and_collinear_ydir_fails() {
        let path = path(
            &[[0.0, 0.0, 0.0], [0.0, 1000.0, 0.0]],
            Some(DVec3::Z),
            90.0,
            false,
        );
        // 绕 +Y 右手转 90°：+Z → +X。
        assert!(
            close(path.leave_up(0).unwrap(), DVec3::X),
            "{:?}",
            path.leave_up(0)
        );
        let bad = path_with_ydir(DVec3::Y);
        assert!(bad.leave_up(0).is_none());
    }

    fn path_with_ydir(ydir: DVec3) -> RoutePath {
        path(
            &[[0.0, 0.0, 0.0], [0.0, 1000.0, 0.0]],
            Some(ydir),
            0.0,
            false,
        )
    }

    #[test]
    fn rotate_about_is_right_handed_and_angle_between_matches_libgeom() {
        assert!(close(
            rotate_about(DVec3::X, DVec3::Z, std::f64::consts::FRAC_PI_2),
            DVec3::Y
        ));
        assert!(
            close(rotate_about(DVec3::X, DVec3::ZERO, 1.0), DVec3::X),
            "零轴原样返回"
        );
        assert!((angle_between(DVec3::X, DVec3::Y) - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
        assert!((angle_between(DVec3::X, DVec3::NEG_X) - std::f64::consts::PI).abs() < 1e-12);
        assert_eq!(angle_between(DVec3::ZERO, DVec3::ZERO), 0.0);
    }

    #[test]
    fn anchor_display_names_the_points() {
        let text = Anchor::Segment {
            a: refno(3),
            b: refno(4),
        }
        .to_string();
        assert!(text.contains("9/3") && text.contains("9/4"), "{text}");
        assert!(Anchor::EndPoint(refno(7)).to_string().contains("9/7"));
    }
}

//! `RPATH` 上的现算放置：把 `(POSREF, PKDI, ZDIS, ZOFF)` 落成路径上的一个点与一个局部系。
//!
//! E3D 有一族元素的属性表里**没有** `POS`/`ORI`，位置注册成伪属性、每次读都沿属主的
//! `RPATH` 现算。本模块是这套算法的公共部分，两家在用：
//!
//! | 家族 | 容器 | Core3D 的伪属性类 | 调用方 |
//! |---|---|---|---|
//! | 电缆桥架件（`CTSTRA`/`CTBEND`/…） | `CWBRAN` | `MDR_DB_POSandORIonRPATH`（`sub_109E71E0`） | [`crate::cable_tray`] |
//! | 扶手成员（`HRPOST` 等） | `HANDRA` | `MDR_DBPseudoPOSandORIonHandra`（vtbl `0x10B8036C`） | [`crate::handrail`] |
//!
//! 反编译原文在 `evidence/20260902-hvac-ctray-census/`（桥架侧）与
//! `evidence/20260906-asl-builders/`（扶手侧）。
//!
//! # 这里管什么、不管什么
//!
//! 管：`POSREF` 链逐级折算、按 `PKDI`/`ZDIS` 的符号定出 A→B 那一段、段内插值、`ZOFF` 抬高，
//! 以及 up 向量沿路径的传播（`PDIR`/`AYDIR`/`LYDIR` 三个 POINTR 伪属性）。
//!
//! 不管：**各家自己的对正偏移**。桥架要按 `CWJUSL`/`VWIDT`/`VHEIG` 把件从底面中线挪开，
//! 扶手要按 `HRJUSL`/`XOFF` 挪——那是「这一族的几何原点在哪」，不是「路径上哪一点」。
//! [`straight_point`] 交回 `inside`，调用方据它决定加不加自己的偏移：Core 的三条早退
//! （距离为负、超出段长、段长退化）直接给端点，**不再加对正偏移**，这个口径要跟着。
//!
//! # 移植范围
//!
//! 只有 **A、B 都是 `POINTR`、`FRAD = 0`** 的直线段。`CURVE`/`RSECT`/`EXTGEO` 段与倒圆拐点
//! （`PPOS[5]`/`FSTART` 语义未钉死）碰到就报 [`DbElementError::PseudoAttribute`]，不猜。
//! 扶手那边有真弧段（`aps250110` 的 49 根柱子里 16 根坐在弧上），接它要先扩这里。

use std::fmt;

use e3d_io::db_element::{DbElement, DbElementError};
use e3d_io::refno::RefNo;
use glam::{DMat3, DQuat, DVec3};

/// `ZDIS/PKDI` 视同零、`XOFF/ZOFF` 视同不生效的门限（`calculatePOS` 里的 1e-8）。
pub(crate) const ATTR_TOL: f64 = 1e-8;
/// 段长退化门限（`calculatePOS`/`calculateORI` 里的 1e-10）。
pub(crate) const LENGTH_TOL: f64 = 1e-10;
/// 向量能不能归一化的门限。
pub(crate) const VECTOR_TOL: f64 = 1e-12;

/// 落点在路径上的位置。
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

/// `readFormDB` 定出来的落点，下标版（`Anchor` 是它的 refno 版，给报告用）。
#[derive(Debug, Clone, Copy)]
pub(crate) enum Resolved {
    Segment {
        a: usize,
        b: usize,
        pkdi: f64,
        zdis: f64,
    },
    EndPoint(usize),
}

impl Resolved {
    /// 换成 refno 版，给报告与诊断用。
    pub(crate) fn anchor(&self, path: &RoutePath) -> Anchor {
        match *self {
            Resolved::Segment { a, b, .. } => Anchor::Segment {
                a: path.points[a].refno,
                b: path.points[b].refno,
            },
            Resolved::EndPoint(index) => Anchor::EndPoint(path.points[index].refno),
        }
    }
}

/// `MDR_DB_POSandORIonRPATH::readFormDB`（`sub_109E71E0`）：沿 `POSREF` 链落到一个 `POINTR`，
/// 再按 `PKDI/ZDIS` 的符号决定这一段是从它往前（A = 它）还是往后（B = 它）。
///
/// `forward_when_zero`：`PKDI`、`ZDIS` 都不表态时朝哪边。桥架的直件 / 变径 / 接头一律向前，
/// 其余看符号——这一条是各家自己的名单，所以由调用方算好传进来。
pub(crate) fn resolve_anchor(
    el: &DbElement,
    container: &DbElement,
    posref: DbElement,
    mut pkdi: f64,
    mut zdis: f64,
    forward_when_zero: bool,
) -> Result<(RoutePath, Resolved), DbElementError> {
    let closed = flag(container, "CLOSEF")?;

    // POSREF 指向另一只同族件时逐级折算：PKDI 按比例复合，ZDIS 累加，直到落到 POINTR。
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
        // 链断了：退到容器自己的 RPATH 第一个成员。
        _ => {
            let rpath = first_rpath(container)?.ok_or_else(|| {
                failed(
                    el,
                    "POS",
                    format!(
                        "POSREF 链断了，且 {} 下没有 RPATH（Core 会给 (0,0,0)）",
                        container.refno()
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

    // 落点必须挂在本容器的路径下；Core 对不上就原样返回 (0,0,0)。
    let rpath = anchor_point.owner()?.ok_or_else(|| {
        failed(
            el,
            "POS",
            format!("路径点 {} 没有属主", anchor_point.refno()),
        )
    })?;
    let rpath_owner = rpath.owner()?.map(|owner| owner.refno());
    if rpath_owner != Some(container.refno()) {
        return Err(failed(
            el,
            "POS",
            format!(
                "POSREF 落到的 {} 属主链是 {} → {:?}，不在本 {} 下（Core 会给 (0,0,0)）",
                anchor_point.refno(),
                rpath.refno(),
                rpath_owner,
                container.refno()
            ),
        ));
    }
    let path = RoutePath::load(el, &rpath, container)?;
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

    let forward = if pkdi == 0.0 && forward_when_zero {
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

/// `readMiddleFormDB` 里没有 `POSREF` 的那一支：取容器的 RPATH 第一个 POINTR 为 A、
/// 它的下一个成员为 B；缺哪个 Core 都返回失败。
pub(crate) fn resolve_without_posref(
    el: &DbElement,
    container: &DbElement,
    pkdi: f64,
    zdis: f64,
) -> Result<(RoutePath, Resolved), DbElementError> {
    let rpath = first_rpath(container)?.ok_or_else(|| {
        failed(
            el,
            "POS",
            format!(
                "没有 POSREF，且 {} 下没有 RPATH（Core 会给 (0,0,0)）",
                container.refno()
            ),
        )
    })?;
    let path = RoutePath::load(el, &rpath, container)?;
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

pub(crate) fn first_rpath(container: &DbElement) -> Result<Option<DbElement>, DbElementError> {
    for member in container.members()? {
        if member.element_type()? == "RPATH" {
            return Ok(Some(member));
        }
    }
    Ok(None)
}

/// [`straight_point`] 的结果。
#[derive(Debug, Clone, Copy)]
pub(crate) struct StraightPoint {
    /// 路径上的落点，已含 `ZOFF` 与调用方传进来的横向量 `jusl`。
    pub position: DVec3,
    /// 真落在段内（走完了插值那一步）。Core 的三条早退——距离为负、超出段长、段长退化
    /// ——直接给端点，**调用方的对正偏移那一步要跟着跳过**。
    pub inside: bool,
    /// A→B（未归一），给调用方算自己的横向偏移用。
    pub along: DVec3,
    /// 本段的 up（`LYDIR`），兜底 `+Z`。
    pub up: DVec3,
}

/// `calculatePOS`（`sub_109E4DC0`）A、B 皆为 POINTR 的直线支。
///
/// Core 对非 STRA/COUP/REDU 且 `ZDIS`/`PKDI` 非零的件读 `A.PPOS[2]`、其余读 `A.POS`；
/// `FRAD = 0` 时 `PPOS[0..4]` 都是 `POS`（`MDR_DB_PseudoPPOSonPOINTR::getAtt` 0..4 同一支），
/// 这里统一用 `POS`（[`RoutePath::load`] 已把 `FRAD ≠ 0` 拒掉）。
pub(crate) fn straight_point(
    path: &RoutePath,
    a: usize,
    b: usize,
    pkdi: f64,
    zdis: f64,
    zoff: f64,
    jusl: f64,
) -> StraightPoint {
    let point_a = path.points[a].pos;
    let point_b = path.points[b].pos;
    let along = point_b - point_a;
    let length = along.length();
    let up = path.leave_up(a).unwrap_or(DVec3::Z);
    let distance = zdis + pkdi * length;
    let outside = |position: DVec3| StraightPoint {
        position,
        inside: false,
        along,
        up,
    };

    // `dist >= 0`，或「纯 ZDIS 为负」（PKDI = 0 且 ZDIS < 0）允许从 A 往回量。
    if !(distance >= 0.0 || (pkdi == 0.0 && zdis < 0.0)) {
        return outside(point_a + DVec3::new(0.0, 0.0, zoff));
    }
    if distance > length {
        return outside(point_b + DVec3::new(0.0, 0.0, zoff));
    }
    if length < LENGTH_TOL {
        return outside(point_a + DVec3::new(0.0, 0.0, zoff));
    }

    let mut position = point_a + along * (distance / length);
    if jusl.abs() > ATTR_TOL {
        position += unit_or_zero(along.cross(up)) * jusl;
    }
    if zoff.abs() > ATTR_TOL {
        position += unit_or_zero(up) * zoff;
    }
    StraightPoint {
        position,
        inside: true,
        along,
        up,
    }
}

/// `calculateORI`（`sub_109E39D0`）直线支：X = 路径方向 × up，Y = 路径方向，Z = X × Y。
/// 段长退化时保持单位阵（Core 保留调用方传入的矩阵）。
pub(crate) fn straight_rotation(
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
pub(crate) fn frame_xy(x: DVec3, y: DVec3) -> Option<DMat3> {
    let x = unit(x)?;
    let y = unit(y - x * y.dot(x))?;
    Some(DMat3::from_cols(x, y, x.cross(y)))
}

/// 一条 `RPATH` 上按成员原序排好的 `POINTR`，连同算 up 向量要的容器 `BANG/CLOSEF`
/// 与 `RPATH.YDIR`。POINTR 侧的伪属性 `PDIR`（到达/离开方向）、`AYDIR`/`LYDIR`（到达/离开 up）
/// 都在这里算，不再回库。
pub(crate) struct RoutePath {
    pub(crate) ydir: Option<DVec3>,
    /// 容器的 `BANG`（度）：路径起点 up 绕起始方向的预转。
    pub(crate) bank: f64,
    pub(crate) closed: bool,
    pub(crate) points: Vec<PathPoint>,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct PathPoint {
    pub(crate) refno: RefNo,
    pub(crate) pos: DVec3,
}

impl RoutePath {
    pub(crate) fn load(
        el: &DbElement,
        rpath: &DbElement,
        container: &DbElement,
    ) -> Result<Self, DbElementError> {
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
            bank: number_or_zero(container, "BANG")?,
            closed: flag(container, "CLOSEF")?,
            points,
        })
    }

    pub(crate) fn index_of(&self, refno: RefNo) -> Option<usize> {
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
    pub(crate) fn arrive_dir(&self, index: usize) -> DVec3 {
        match self.prev(index) {
            Some(prev) => unit_or_zero(self.points[index].pos - self.points[prev].pos),
            None => DVec3::ZERO,
        }
    }

    /// `PDIR[2]`：离开方向 = 下一点 − 本点，归一；没有下一点给零向量。
    pub(crate) fn leave_dir(&self, index: usize) -> DVec3 {
        match self.next(index) {
            Some(next) => unit_or_zero(self.points[next].pos - self.points[index].pos),
            None => DVec3::ZERO,
        }
    }

    /// `AYDIR`（`sub_105C0BA0`）：到达本点时的 up。起点 up 取 `YDIR` 在起始方向正交补里的分量，
    /// 绕起始方向预转容器的 `BANG`，再逐个拐点按转折（到达→离开的最小旋转）传播到本点之前。
    /// `YDIR` 与起始方向共线时 Core 返回失败——调用方兜底 `+Z`。
    pub(crate) fn arrive_up(&self, index: usize) -> Option<DVec3> {
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
    pub(crate) fn leave_up(&self, index: usize) -> Option<DVec3> {
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
pub(crate) fn rotate_about(vector: DVec3, axis: DVec3, angle: f64) -> DVec3 {
    match unit(axis) {
        Some(axis) if angle != 0.0 => DQuat::from_axis_angle(axis, angle) * vector,
        _ => vector,
    }
}

/// libgeom `D3_Vector::angle`：`atan2(|a×b|, a·b)`，两向量皆零给 0。
pub(crate) fn angle_between(a: DVec3, b: DVec3) -> f64 {
    let cross = a.cross(b).length();
    let dot = a.dot(b);
    if cross == 0.0 && dot == 0.0 {
        0.0
    } else {
        cross.atan2(dot)
    }
}

pub(crate) fn unit(vector: DVec3) -> Option<DVec3> {
    let length = vector.length();
    (length > VECTOR_TOL && length.is_finite()).then(|| vector / length)
}

pub(crate) fn unit_or_zero(vector: DVec3) -> DVec3 {
    unit(vector).unwrap_or(DVec3::ZERO)
}

// ---- 属性读取：E3D `getAtt` 口径 ------------------------------------------------

/// 属性不在该 noun 的模板里 = Core `getAtt` 返回 false；本模块把它与「unset」同样当缺省处理。
pub(crate) fn optional<T>(
    result: Result<Option<T>, DbElementError>,
) -> Result<Option<T>, DbElementError> {
    match result {
        Ok(value) => Ok(value),
        Err(DbElementError::UnknownAttribute { .. }) => Ok(None),
        Err(error) => Err(error),
    }
}

pub(crate) fn number_or_zero(el: &DbElement, name: &str) -> Result<f64, DbElementError> {
    Ok(optional(el.get_double(name))?.unwrap_or(0.0))
}

pub(crate) fn flag(el: &DbElement, name: &str) -> Result<bool, DbElementError> {
    Ok(optional(el.get_bool(name))?.unwrap_or(false))
}

pub(crate) fn reference(el: &DbElement, name: &str) -> Result<Option<DbElement>, DbElementError> {
    optional(el.get_element(name))
}

pub(crate) fn vector(el: &DbElement, name: &str) -> Result<Option<DVec3>, DbElementError> {
    Ok(optional(el.get_position(name))?.map(DVec3::from_array))
}

pub(crate) fn word(el: &DbElement, name: &str) -> Result<Option<String>, DbElementError> {
    optional(el.get_string(name))
}

pub(crate) fn failed(el: &DbElement, name: &str, detail: impl Into<String>) -> DbElementError {
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

    pub(super) fn path(
        points: &[[f64; 3]],
        ydir: Option<DVec3>,
        bank: f64,
        closed: bool,
    ) -> RoutePath {
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

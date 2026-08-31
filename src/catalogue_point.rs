//! 目录侧 P 点求解 —— 把 [`crate::route::PPointResolver`] 那条缝接上。
//!
//! [`crate::route`] 把「构件的离开 / 到达点」留成了一个 trait，因为设计元素上的
//! `LEAV` / `ARRI` 只是 **P 点编号**，坐标在目录里。这个模块走完那条链：
//!
//! ```text
//! 设计构件 --SPRE--> SPCO --CATR--> SCOM --PTRE--> PTSE --成员--> PTAX / PTCA
//! ```
//!
//! 有的构件的 `SPRE` 直接落在 `SCOM` 上，`CATR` 那一跳就省了；ams8000 上 2362 个
//! 构件的 `SPRE` 全落在 `SPCO`，但这条捷径在别的库里出现过，照走。
//!
//! P 点上存的多半不是数而是**目录表达式**，求值要 e3d-io 的
//! [`DbElementParamEnv`]：它同时握着设计构件与目录件，因为 `ANGL` / `RADI` /
//! `DESP` 只在设计侧、`PARA` 只在目录侧，缺一边就有一类名字答不上。
//!
//! ## 坐标系
//!
//! P 点的位置与轴都在**构件自己的坐标系**里。世界坐标要再套设计构件的世界矩阵
//! （[`crate::transform::world_matrix`]，`POS`/`ORI` 沿属主链折叠）。方向只过
//! 旋转部分，不吃平移。
//!
//! ## 两种 P 点，方向各有各的家
//!
//! `PTAX` 是「轴 + 距离」：方向来自 `PAXI`（[`axis_spec`]），位置是方向乘 `PDIS`。
//! `PTCA` 直接给 `PX`/`PY`/`PZ` 三个坐标，方向来自 `PTCD`（[`direction_spec`]）。
//! 两种都做。
//!
//! **`PTCA` 的方向不在 `DIR` 上。** `DIR` 读出来是个正经的三元实数组，很像方向，
//! 但 ams8000 的 516 个 `PTCA` 上它无一例外都是 `[0, 1, 0]`——那是模板缺省值。
//! 第一版拿它当方向用，`examples/tube_axis_check` 当场把账掀了：弯头的两个口解出
//! **同一个**方向（都是构件局部 +Y 转到世界系），而管身照建不误、长度也像模像样，
//! 只是朝向全错。这种错法比解不出恶劣得多——记账上看不见，几何上看得见。
//!
//! 真方向在 `PTCD`，自成一套字编码：
//!
//! ```text
//! [总长(含自身), 5|6, 2, opcode, 方向码, <PML 程序> 方向码, …, 61]
//! ```
//!
//! `opcode = 20 + 方向个数`，方向码 `11..16` 是 `X -X Y -Y Z -Z`，每段 PML 求值出
//! 一个角度（度），语义是「把当前方向朝下一个具名轴转这么多」。ams8000 全库 516 个
//! `PTCA` 端点 100% 解得出，其中 323 个是裸轴（opcode 21）、193 个带一次转角（22）。
//! 用 `examples/ppoint_probe` 的「PTCD 解码结局」一节复算。

use glam::DVec3;

use e3d_io::db_element::{DbElement, DbElementError};
use e3d_io::db_param_env::DbElementParamEnv;
use e3d_io::record::axis_spec;
use e3d_io::record::descriptor::DescriptorValue;
use e3d_io::record::direction_spec;

use crate::route::{PPointResolver, RoutePoint};
use crate::transform::world_matrix;

/// 设计构件指向规格件（多半是 `SPCO`，偶尔直接是 `SCOM`）。
const SPEC_REF: &str = "SPRE";
/// 规格件指向目录件。
const CATALOGUE_REF: &str = "CATR";
/// 目录件指向 P 点集 `PTSE`。
const POINT_SET_REF: &str = "PTRE";
/// 直接给三个坐标的那种 P 点；其余（`PTAX`）是「轴 + 距离」。
const CARTESIAN_POINT: &str = "PTCA";

/// 方向向量短到这个长度以下就当没方向。
///
/// 与 [`crate::route::orimat`] 的退化阈值同源：定不出朝向的管身宁可记
/// `unbuildable`，也不能拿一个近乎零的向量归一化出一个随机方向。
const DIRECTION_TOL: f64 = 1.0e-9;

/// 走目录链解 P 点的 [`PPointResolver`]。
///
/// 无状态：每次解都重走一遍链。ams8000 全库 2612 个构件 × 两端实测 5 s 量级，
/// 还不值得为它引一层缓存——缓存要按「构件 + 目录件」这一对做键（求值环境两边
/// 都要），键搞错了会把甲零件的尺寸发给乙零件，代价远大于省下的时间。
#[derive(Debug, Default, Clone, Copy)]
pub struct CataloguePoints;

impl PPointResolver for CataloguePoints {
    fn leave(
        &self,
        member: &DbElement,
        ppoint: Option<u32>,
    ) -> Result<Option<RoutePoint>, DbElementError> {
        resolve(member, ppoint)
    }

    fn arrive(
        &self,
        member: &DbElement,
        ppoint: Option<u32>,
    ) -> Result<Option<RoutePoint>, DbElementError> {
        resolve(member, ppoint)
    }
}

/// 两端走的是同一条链、同一个 `PTSE`，只是编号不同。
///
/// 链上任何一跳缺失都返回 `Ok(None)`——那是「这个构件的这一端解不出」，是记账
/// 里的一类，不是读库出错。只有真正读坏了才 `Err`。
fn resolve(member: &DbElement, ppoint: Option<u32>) -> Result<Option<RoutePoint>, DbElementError> {
    let Some(number) = ppoint else {
        return Ok(None);
    };
    let Some(component) = catalogue_component(member)? else {
        return Ok(None);
    };
    let Some(point) = point_numbered(&component, number)? else {
        return Ok(None);
    };

    let env = DbElementParamEnv::new(Some(member.clone()), Some(component.clone()));
    let local = if point.element_type()?.eq_ignore_ascii_case(CARTESIAN_POINT) {
        cartesian(&point, &env)
    } else {
        axial(&point, &env)
    };
    let Some((pos, dir)) = local else {
        return Ok(None);
    };

    let world = world_matrix(member)?;
    let dir = world.transform_vector3(dir);
    if !(dir.length() > DIRECTION_TOL) || !dir.is_finite() {
        return Ok(None);
    }
    let pos = world.transform_point3(pos);
    if !pos.is_finite() {
        return Ok(None);
    }

    Ok(Some(RoutePoint {
        pos,
        dir: dir.normalize(),
        bore: number_of(&point, "PBOR", &env),
    }))
}

/// `PTCA`：三个坐标表达式 + PTCD 方向规格。
fn cartesian(point: &DbElement, env: &DbElementParamEnv) -> Option<(DVec3, DVec3)> {
    let pos = DVec3::new(
        number_of(point, "PX", env)?,
        number_of(point, "PY", env)?,
        number_of(point, "PZ", env)?,
    );
    let words = match point.attribute("PTCD").ok()?? {
        DescriptorValue::RawWords(words) | DescriptorValue::WordArray(words) => words,
        DescriptorValue::IntArray(words) => words.into_iter().map(|word| word as u32).collect(),
        _ => return None,
    };
    let dir = DVec3::from_array(direction_spec::direction(&words, env)?);
    Some((pos, dir))
}

/// `SPRE`（→ `CATR`）→ 目录件。
fn catalogue_component(member: &DbElement) -> Result<Option<DbElement>, DbElementError> {
    let Some(mut current) = member.get_element(SPEC_REF)? else {
        return Ok(None);
    };
    // 有些项目是设计 SPCO → SCOM；有些还会经过目录侧 SPCO。按 refno 有界追踪，
    // 只在真正到达 SCOM 后读 PTRE，避免把中间规格节点误当几何组件。
    let mut seen = std::collections::HashSet::new();
    for _ in 0..8 {
        if !seen.insert(current.refno()) {
            return Ok(None);
        }
        if current.element_type()?.eq_ignore_ascii_case("SCOM") {
            return Ok(Some(current));
        }
        let next = match current.get_element(CATALOGUE_REF) {
            Ok(next) => next,
            // SELE 等选择节点可能出现在规格引用链中，但它们没有 CATR；这是链到此
            // 无法继续，不是模板或文件损坏。
            Err(DbElementError::UnknownAttribute { .. }) => None,
            Err(error) => return Err(error),
        };
        let Some(next) = next else {
            return Ok(None);
        };
        current = next;
    }
    Ok(None)
}

/// 目录件的 `PTSE` 里编号为 `number` 的那个 P 点。
fn point_numbered(component: &DbElement, number: u32) -> Result<Option<DbElement>, DbElementError> {
    let Some(set) = component.get_element(POINT_SET_REF)? else {
        return Ok(None);
    };
    for point in set.members()? {
        if point_number(&point) == Some(number) {
            return Ok(Some(point));
        }
    }
    Ok(None)
}

/// P 点自己的 `NUMB`。
fn point_number(point: &DbElement) -> Option<u32> {
    match point.attribute("NUMB") {
        Ok(Some(DescriptorValue::Word { raw, .. })) => Some(raw),
        Ok(Some(DescriptorValue::Int(value))) => u32::try_from(value).ok(),
        _ => None,
    }
}

/// `PTAX`：沿 `PAXI` 给的轴走 `PDIS`。
///
/// 两样缺一不可。轴解不出（`Pn` 那种指向另一个 P 点的形式，或未钉死的形状）
/// 就整条弃掉——只有距离没有方向，落不成一个点。
fn axial(point: &DbElement, env: &DbElementParamEnv) -> Option<(DVec3, DVec3)> {
    let axis = axis_words(point)?;
    let dir = DVec3::from_array(axis_spec::direction(&axis, env)?);
    let distance = number_of(point, "PDIS", env)?;
    Some((dir * distance, dir))
}

/// `PAXI` 的原始词。实现里两种数组形状都出现过，都收。
fn axis_words(point: &DbElement) -> Option<Vec<u32>> {
    match point.attribute("PAXI") {
        Ok(Some(DescriptorValue::RawWords(words))) => Some(words),
        Ok(Some(DescriptorValue::WordArray(words))) => Some(words),
        Ok(Some(DescriptorValue::IntArray(words))) => {
            Some(words.into_iter().map(|word| word as u32).collect())
        }
        _ => None,
    }
}

/// 一个属性读成数，含它可能带的目录表达式。求不出给 `None`，不折成 0。
fn number_of(element: &DbElement, name: &str, env: &DbElementParamEnv) -> Option<f64> {
    element.get_number(name, env).ok()?.value()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 编号缺省时不该去猜一个「第一个 P 点」。
    ///
    /// `LEAV` / `ARRI` 读不出编号的构件，E3D 也不知道该连哪一端;
    /// 随手取 `NUMB=1` 会让这类构件长出一根指向随机方位的管。
    #[test]
    fn a_member_without_a_point_number_resolves_to_nothing() {
        let set = std::sync::Arc::new(e3d_io::db_element::DbSet::new(std::sync::Arc::new(
            e3d_attlib::AttlibData::default(),
        )));
        let member = set.element(e3d_io::refno::RefNo::new(0, 0));
        assert_eq!(resolve(&member, None).unwrap(), None);
    }

    /// trait 是对象安全的，用起来才能塞进 `&dyn PPointResolver`。
    #[test]
    fn the_resolver_is_object_safe() {
        fn assert_object_safe(_: &dyn PPointResolver) {}
        assert_object_safe(&CataloguePoints);
    }
}

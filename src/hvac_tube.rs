//! 风管隐式直段的**断面**——管道那支 `CGETOD`（外径）在风管上的对应物。
//!
//! 管道的隐式管身只有一个尺寸：离开侧 stub（`HSTU`/`LSTU`）指向的 TUBI 目录件 `PARA[2]`
//! 外径，取不到回落到达侧通径（[`crate::route::tube_outer_diameter`]）。风管的 TUBI 目录件
//! 没有 `PARA`，断面写在它的几何集里，而且是**表达式**：
//!
//! ```text
//! LSTU = /CADCHVACSPEC/RTUBEB  --CATR-->  SCOM /RTUBE2  --GMRE-->  BOXING /RTUBE2-BOXI
//!     PAXI = Y                                   （轴向：管身局部 Y）
//!     PXLE = ( ATTRIB LEAWID OF PREV )           （沿局部 X）
//!     PZLE = ( ATTRIB LEAHEI OF PREV )           （沿局部 Z）
//! LSTU = /CADCHVACSPEC/RTUBEA  -->  /RTUBE1-BOXI：PXLE 读 LEAHEI、PZLE 读 LEAWID（两名对调）
//! LSTU = /CADCHVACSPEC/RTUBEC  -->  /RTUBE3-BOXI：同 RTUBE2
//! LSTU = /CADCHVACSPEC/CTUBE   -->  TUBE /CTUBE-TUBE：PDIA = ( ATTRIB LBOR OF PREV )
//! ```
//!
//! `PREV` 是**离开侧构件**，`LEAWID` / `LEAHEI` / `LBOR` 是它的三个伪属性（`ARRWID` /
//! `ARRHEI` / `ABOR` 是到达侧那三个），值就是它 `LEAV`（/ `ARRI`）P 点上的 `PWID` / `PHEI` /
//! `PBOR`——ams7997 TAPE `24381/30293` 的 `Q LEAWID / LEAHEI / LBOR` 给 1460 / 1520 / 2980，
//! 它离开 P2 的 `PWID / PHEI / PBOR` 求出来逐位相同（`e3d-hvac-all-121918/c7_leave_dims.log`
//! 对 `examples` 探针，2026-09-18）。矩形断面还差一个滚转：E3D 按它自己的单轴 `ORI Y IS <离开方向>`
//! 在离开侧构件局部系里定管身的 Z（贴局部 Z，离开方向就是 ±Z 时取局部 Y；**不是** P 点的
//! `PZAXI`，见 [`leave_frame_z_axis`]），`PZLE` 沿它。
//!
//! 所以这里做的事与管道那支同形，只是多两格：
//!
//! | 步骤 | 管道 | 风管（本模块） |
//! |---|---|---|
//! | 选件 | 离开侧 stub → SCOM | 同一个 stub、同一条链（[`tube_component_of_stub`]） |
//! | 尺寸 | `PARA[2]` | 几何集里第一只**按表现级可见**的 BOXING / TUBE 的 `PXLE`·`PZLE` / `PDIA`，表达式里的 `… OF PREV` 由 [`PreviousElementEnv`] 答成离开 P 点的 `PWID`·`PHEI`·`PBOR` |
//! | 滚转 | 无（圆） | 单轴 `ORI Y IS <离开方向>` 在构件局部系里算出的 Z 折到世界系（[`leave_frame_z_axis`]） |
//! | 回落 | 到达侧通径 | **没有**——任一格解不出就是 [`crate::route::TubeOutcome::NoRadius`] |
//!
//! # 头槽位：E3D 自己在这个库上一根都画不出来
//!
//! 容器头读 `HSTU`，风管的 `HSTU` 指向 **H 族**（`/CADCHVACSPEC/HRTUBEA|B` → `/HRTUBE1|2-BOXI`，
//! `HCTUBE` → `/HCTUBE-TUBE`），表达式是 `( :HXYsize[1|2] )`——BRAN 上的一个 UDA，不是
//! `OF PREV`。ams7997 上 7 条头槽位有缝的支管 E3D 全部报 `Catalogue error … unable to evaluate
//! catalogue expression for attribute PXLE`、一根没画（`e3d-hvac-all-121918/h1_export_hvac.log`）。
//! 这里的求值环境**不挂设计元素**（TUBI 没有自己的设计件，`PREV` 才是那个构件），UDA 自然
//! 答不上，头槽位一律 [`crate::route::TubeOutcome::NoRadius`]——与 E3D 在这个库上的表现一致。
//! 若将来某个库上 E3D 画得出 H 族头管，要补的是 `:HXYsize` 那一格，不是这条路。
//!
//! 取证：`_runs/ams-init-1112-7997-20260917-mc/ledger-seg15-frameall.md` §10–§13，
//! 全库 216 条风管 BRAN 对 E3D：16 段真实直段逐根重合、0 段多画（§13）。

use glam::DVec3;

use e3d_io::db_element::{DbElement, DbElementError};
use e3d_io::db_param_env::DbElementParamEnv;
use e3d_io::record::catalogue_eval::{AttribRef, ParamArray, ParamEnv};
use e3d_io::record::catalogue_pml::PREVIOUS_ELEMENT_NAME;

use crate::catalogue::{display_filtered, repre_level_of};
use crate::catalogue_point::{
    catalogue_component, catalogue_point_local, number_of, point_numbered, tube_component_of_stub,
};
use crate::elmodl::ModelError;
use crate::route::{RouteEnd, TubeSection, ppoint_number};
use crate::transform::world_matrix;

/// 容器头 / 尾读的 stub——与管道那支 `GTTUBE` 同一份（先 `HSTU`，再 `HSRO`）。
const CONTAINER_STUBS: [&str; 2] = ["HSTU", "HSRO"];
/// 构件离开侧读的 stub（先 `LSTU`，再 `LSRO`）。
const MEMBER_STUBS: [&str; 2] = ["LSTU", "LSRO"];
/// TUBI 几何集里承担管身的两种图元 noun：矩形 BOXING（字典短名 `BOXI`）与圆 TUBE。
const BOX_PRIMITIVE: &str = "BOXI";
const ROUND_PRIMITIVE: &str = "TUBE";
/// 方向向量短到这个长度以下就当没方向，与 [`crate::catalogue_point`] 同一阈值。
const DIRECTION_TOL: f64 = 1.0e-9;

/// 风管隐式直段解出来的断面，连同它从哪儿来（进 note，供对拍时按目录件分账）。
#[derive(Debug, Clone, PartialEq)]
pub struct DuctSection {
    pub section: TubeSection,
    /// `<SCOM 名> <图元 noun> <图元名>`，例如 `/RTUBE2 BOXI /RTUBE2-BOXI`。
    pub source: String,
}

/// 一条风管槽位**离开侧**的断面；解不出任一格给 `None`（调用方按
/// [`crate::route::TubeOutcome::NoRadius`] 记账，不回落）。只有读库出错才 `Err`。
pub fn duct_departure_section(from: &RouteEnd) -> Result<Option<DuctSection>, DbElementError> {
    let (departure, stubs): (&DbElement, &[&str]) = match from {
        RouteEnd::ContainerHead(container) | RouteEnd::ContainerTail(container) => {
            (container, &CONTAINER_STUBS)
        }
        RouteEnd::MemberLeave { member, .. } | RouteEnd::MemberArrive { member, .. } => {
            (member, &MEMBER_STUBS)
        }
    };
    let mut component = None;
    for stub in stubs {
        if let Some(found) = tube_component_of_stub(departure, stub)? {
            component = Some(found);
            break;
        }
    }
    let Some(component) = component else {
        return Ok(None);
    };
    let Some(primitive) = tube_primitive(&component, departure)? else {
        return Ok(None);
    };
    let env = PreviousElementEnv {
        inner: DbElementParamEnv::catalogue_only(component.clone()).without_insulation(),
        departure: departure.clone(),
    };
    let noun = primitive.element_type()?;
    let section = match noun.as_str() {
        BOX_PRIMITIVE => {
            let (Some(x_length), Some(z_length)) = (
                evaluated(&primitive, "PXLE", &env),
                evaluated(&primitive, "PZLE", &env),
            ) else {
                return Ok(None);
            };
            let Some(z_axis) = leave_frame_z_axis(departure)? else {
                return Ok(None);
            };
            TubeSection::Rectangular {
                x_length,
                z_length,
                z_axis,
            }
        }
        ROUND_PRIMITIVE => {
            let Some(outer_diameter) = evaluated(&primitive, "PDIA", &env) else {
                return Ok(None);
            };
            TubeSection::Round { outer_diameter }
        }
        _ => return Ok(None),
    };
    Ok(Some(DuctSection {
        section,
        source: format!(
            "{} {noun} {}",
            component.stored_name()?.unwrap_or_default(),
            primitive.stored_name()?.unwrap_or_default()
        ),
    }))
}

/// TUBI 目录件几何集里承担管身的那只图元：第一只按**离开侧构件的表现级**可见的 BOXING / TUBE。
///
/// 每个 `RTUBE*` / `CTUBE` 的 GMSE 都是两只同 noun 的图元：`-BOXI` / `-TUBE`（`LEVE [0,10]`）
/// 与 `-INSU`（`LEVE [7,10]`，保温体），窄口径导出（`LEVEL HVAC 6`）只有前者在场；
/// 用 [`display_filtered`] 那一道筛而不是认名字后缀，与目录构件的图元筛同一份口径。
fn tube_primitive(
    component: &DbElement,
    departure: &DbElement,
) -> Result<Option<DbElement>, DbElementError> {
    let set = match component.get_element("GMRE") {
        Ok(set) => set,
        Err(DbElementError::UnknownAttribute { .. }) => None,
        Err(error) => return Err(error),
    };
    let Some(set) = set else {
        return Ok(None);
    };
    let level = match repre_level_of(departure) {
        Ok(level) => level,
        Err(ModelError::Db(error)) => return Err(error),
        Err(_) => return Ok(None),
    };
    for primitive in set.members()? {
        let noun = primitive.element_type()?;
        if noun != BOX_PRIMITIVE && noun != ROUND_PRIMITIVE {
            continue;
        }
        match display_filtered(&primitive, level) {
            Ok(None) => return Ok(Some(primitive)),
            Ok(Some(_)) => continue,
            Err(ModelError::Db(error)) => return Err(error),
            Err(_) => continue,
        }
    }
    Ok(None)
}

/// 图元的一个尺寸属性求成有限正数；求不出、非正、非有限都是「解不出」。
fn evaluated(primitive: &DbElement, name: &str, env: &dyn ParamEnv) -> Option<f64> {
    primitive
        .get_number(name, env)
        .ok()?
        .value()
        .filter(|value| value.is_finite() && *value > 0.0)
}

/// 矩形直段 `PZLE` 那条边的世界朝向：E3D 给管身定向用的是它自己的**单轴 `ORI Y IS <离开方向>`**
/// 规矩，在**离开侧构件的局部系**里算——Z 尽量贴局部 Z（构件的 U），离开方向本身就是 ±Z 时 Z 取
/// 局部 Y（构件的 N）——算出来的那根 Z 折到世界系就是这一格（[`duct_frame_z_axis`]）。
///
/// 这一格不读 P 点的 `PZAXI`，也不是 [`crate::route::orimat`] 的 X̂/Ŷ/Ẑ 顺序。三种读法在 ams7997 的 16 段
/// 真实直段上是这么分出来的（`e3d-frame-all-110248/frame-all.rvm`、`e3d-hvac-all-121918/hvac-all.rvm`，
/// 逐段比世界 AABB）：
///
/// - 13 段离开侧是 STRT / TAPE / DAMP / BRCO，离开轴 = 局部 Z：三种读法都给局部 Y，分不出。
///   `/-VES-S-2-L-4910` 那段（500 沿世界 Y、600 沿世界 Z）排掉了「贴世界 U」这一种。
/// - 3 段离开侧是 BEND（`LSTU=RTUBEA` → `/RTUBE1-BOXI`，`PXLE = LEAHEI`、`PZLE = LEAWID`，与
///   `RTUBE2` 两名对调）。E3D 把 `PZLE` 摆在弯头**局部 Z 去掉沿轴分量**的方向上：两只 90° 弯头
///   （`/-CSV-S-3-L-1702` 91510、`/-CCV-S-2-H-1103` 58345，离开轴 = 局部 X）是局部 Z，那只 39.5° 弯头
///   （无名支管 `24381/79980` 的 79981，离开轴 `(sin 39.5°, 0, cos 39.5°)`）是 `(−cos, 0, sin)`——
///   「贴 `PZAXI`」在三只上全反，「`ORIMAT`」在 39.5° 那只上反（它挑的是局部 Y）。
///   三只弯头的 `Q LEAWID / LEAHEI` 与 P 点的 `PWID / PHEI` 逐位相同（`c9_bend_leawid.log`），
///   所以差别只能在定向，不在取数。
///
/// 物理上这与目录的两名对调正好抵消：弯头之后高（`LEAHEI`）仍沿弯头的转轴（局部 Y），与直段之后
/// 高沿 `PZAXI`（也是局部 Y）一致——目录作者写 `RTUBEA` 就是为了在 E3D 这条定向规矩下保住这一点。
///
/// 读不到 P 点、离开方向解不出、构件世界矩阵退化都是 `None`：滚转不能猜，见
/// [`crate::route::build_tube`]。
fn leave_frame_z_axis(member: &DbElement) -> Result<Option<DVec3>, DbElementError> {
    let Some(number) = ppoint_number(member, "LEAV")? else {
        return Ok(None);
    };
    let Some(component) = design_component(member)? else {
        return Ok(None);
    };
    let Some(local) = catalogue_point_local(member, &component, number)? else {
        return Ok(None);
    };
    let Some(z_local) = duct_frame_z_axis(local.dir) else {
        return Ok(None);
    };
    let direction = world_matrix(member)?.transform_vector3(z_local);
    Ok((direction.is_finite() && direction.length() > DIRECTION_TOL).then_some(direction))
}

/// E3D 单轴 `ORI Y IS <axis>` 的 Z：`axis` 所在坐标系的 Ẑ 去掉沿 `axis` 的分量；`axis` 本身
/// 就是 ±Ẑ（投影退化）时取 Ŷ（去掉沿 `axis` 的分量）。返回单位向量；`axis` 退化给 `None`。
///
/// 纯函数，坐标系由调用方定——[`leave_frame_z_axis`] 在离开侧构件的局部系里调它。
/// 与 [`crate::route::orimat`] 的区别只在挑第二根轴的顺序：那边是 X̂ / Ŷ / Ẑ 依次叉乘取第一根不退化的
/// （管道圆柱周向起点的口径），这边是「贴 Ẑ、退化才 Ŷ」（E3D 定向语句的口径）。
pub(crate) fn duct_frame_z_axis(axis: DVec3) -> Option<DVec3> {
    if !(axis.length() > DIRECTION_TOL) || !axis.is_finite() {
        return None;
    }
    let axis = axis.normalize();
    for reference in [DVec3::Z, DVec3::Y] {
        let projected = reference - axis * reference.dot(axis);
        if projected.length() > DIRECTION_TOL {
            return Some(projected.normalize());
        }
    }
    None
}

/// 构件自己的目录件（`SPRE → … → SCOM`）。容器（BRAN）模板里没有 `SPRE`，按「没有目录件」
/// 而不是缺属性报错——头槽位的 `PREV` 就是容器自己，它本来就没有 P 点可读。
fn design_component(member: &DbElement) -> Result<Option<DbElement>, DbElementError> {
    match catalogue_component(member) {
        Ok(component) => Ok(component),
        Err(DbElementError::UnknownAttribute { .. }) => Ok(None),
        Err(error) => Err(error),
    }
}

/// 风管构件的六个断面伪属性：`LEAWID` / `LEAHEI` / `LBOR` 读 `LEAV` P 点的 `PWID` / `PHEI` /
/// `PBOR`，`ARRWID` / `ARRHEI` / `ABOR` 读 `ARRI` P 点的同三格。别的名字给 `None`。
///
/// 这六个名字在 e3d-io 的字典里都不是构件的存储属性（TAPE 的 27 个字段里没有它们），
/// E3D 是现算的；这里按 P 点算，与 `Q LEAWID` 逐位对得上（模块文档）。
pub fn hvac_pseudo_attribute(
    member: &DbElement,
    name: &str,
) -> Result<Option<f64>, DbElementError> {
    let (end, attribute) = match name.trim().to_ascii_uppercase().as_str() {
        "LEAWID" => ("LEAV", "PWID"),
        "LEAHEI" => ("LEAV", "PHEI"),
        "LBOR" => ("LEAV", "PBOR"),
        "ARRWID" => ("ARRI", "PWID"),
        "ARRHEI" => ("ARRI", "PHEI"),
        "ABOR" => ("ARRI", "PBOR"),
        _ => return Ok(None),
    };
    let Some(number) = ppoint_number(member, end)? else {
        return Ok(None);
    };
    let Some(component) = design_component(member)? else {
        return Ok(None);
    };
    let Ok(point) = point_numbered(&component, number)? else {
        return Ok(None);
    };
    let env =
        DbElementParamEnv::new(Some(member.clone()), Some(component.clone())).without_insulation();
    Ok(number_of(&point, attribute, &env))
}

/// TUBI 目录件表达式的求值环境：`PARAM n` 等照常问目录件；`ATTRIB <名> OF PREV` 问
/// **离开侧构件**的伪属性（[`hvac_pseudo_attribute`]）。
///
/// 不挂设计元素：隐式管身没有自己的设计件，`PREV` 才是那个构件。H 族的 `( :HXYsize[n] )`
/// 因此答不上——那正是 E3D 在这个库上的表现（模块文档「头槽位」一节）。
struct PreviousElementEnv {
    inner: DbElementParamEnv,
    departure: DbElement,
}

impl ParamEnv for PreviousElementEnv {
    fn param(&self, n: i32) -> Option<f64> {
        self.inner.param(n)
    }

    fn iparam(&self, n: i32) -> Option<f64> {
        self.inner.iparam(n)
    }

    fn parameter(&self, array: ParamArray, n: i32) -> Option<f64> {
        self.inner.parameter(array, n)
    }

    fn design_dimension(&self, id: i32) -> Option<f64> {
        self.inner.design_dimension(id)
    }

    fn attribute_number(&self, attribute: &AttribRef<'_>) -> Option<f64> {
        match attribute.owner_chain {
            [owner]
                if owner.eq_ignore_ascii_case(PREVIOUS_ELEMENT_NAME)
                    && attribute.subscript.is_none()
                    && attribute.qualifier.is_none() =>
            {
                hvac_pseudo_attribute(&self.departure, attribute.name)
                    .ok()
                    .flatten()
            }
            _ => self.inner.attribute_number(attribute),
        }
    }

    /// 按字典键问的 UDA 交给内层。这里**没有** `PREV` 那一支：`( :HXYsize[2] )` 写下的是键，
    /// 不带属主链（`ledger-seg15-frameall.md` §19.1），所以拦不着也不该拦；内层挂的是目录件，
    /// 答的就是目录件自己的 UDA。
    ///
    /// 不转发的话这里落回 [`ParamEnv::uda_number`] 的默认 `None`——**目录件身上真存着的 UDA
    /// 也读不出来**，而且无声。本库看不出差别（H 族头槽位那条路的键属于 acp7006 那本字典、
    /// 元素上没值，§19.2），换一座库就看得出，所以钉在这儿。
    fn uda_number(&self, key: u32, subscript: Option<f64>) -> Option<f64> {
        self.inner.uda_number(key, subscript)
    }

    fn fast_path_missing_parameter_is_zero(&self, name: &str, subscript: Option<f64>) -> bool {
        self.inner
            .fast_path_missing_parameter_is_zero(name, subscript)
    }

    fn direction_angle(&self, program: &[i32]) -> Option<f64> {
        self.inner.direction_angle(program)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// E3D 单轴定向的 Z：贴 Ẑ，轴就是 ±Ẑ 时取 Ŷ。数是三只弯头与一只直段的离开轴：
    /// 90° 弯头离开轴 = 局部 X → Z 取局部 Z；39.5° 弯头离开轴 `(sin, 0, cos)` → Z 取 `(−cos, 0, sin)`
    /// （即局部 Z 去掉沿轴分量）；直段离开轴 = 局部 Z → Z 取局部 Y。`ORIMAT` 在 39.5° 那只上
    /// 会挑局部 Y，`PZAXI`（局部 Y）在三只弯头上全反——见 [`leave_frame_z_axis`] 的取证。
    #[test]
    fn the_duct_frame_z_hugs_local_z_and_falls_back_to_local_y() {
        let z = duct_frame_z_axis(DVec3::X).expect("90° 弯头");
        assert!((z - DVec3::Z).length() < 1.0e-12, "{z:?}");

        let (sin, cos) = 39.5_f64.to_radians().sin_cos();
        let z = duct_frame_z_axis(DVec3::new(sin, 0.0, cos)).expect("39.5° 弯头");
        assert!((z - DVec3::new(-cos, 0.0, sin)).length() < 1.0e-12, "{z:?}");
        assert!(z.dot(DVec3::new(sin, 0.0, cos)).abs() < 1.0e-12, "与轴正交");

        let z = duct_frame_z_axis(DVec3::Z).expect("直段");
        assert!((z - DVec3::Y).length() < 1.0e-12, "{z:?}");
        let z = duct_frame_z_axis(DVec3::new(0.0, 0.0, -1.0)).expect("反着离开的直段");
        assert!((z - DVec3::Y).length() < 1.0e-12, "{z:?}");
        // 库里的离开轴带 6e-17 的浮点残渣，仍按「就是 Z」处理。
        let z = duct_frame_z_axis(DVec3::new(1.0e-17, -6.0e-17, 1.0)).expect("近似 Z");
        assert!((z - DVec3::Y).length() < 1.0e-9, "{z:?}");

        assert_eq!(duct_frame_z_axis(DVec3::ZERO), None);
        assert_eq!(duct_frame_z_axis(DVec3::new(f64::NAN, 0.0, 0.0)), None);
    }

    /// 六个伪属性各归各的 P 点与格子；不认识的名字不猜。
    #[test]
    fn the_six_section_pseudo_attributes_map_to_their_point_and_slot() {
        let set = std::sync::Arc::new(e3d_io::db_element::DbSet::new(std::sync::Arc::new(
            e3d_attlib::AttlibData::default(),
        )));
        let member = set.element(e3d_io::refno::RefNo::new(0, 0));
        // 没有库：认识的名字读到「没有 P 点编号」→ None，不认识的名字在读库之前就 None。
        assert!(matches!(
            hvac_pseudo_attribute(&member, "LEAWID"),
            Err(DbElementError::DbNotRegistered { dbno: 0 })
        ));
        assert_eq!(hvac_pseudo_attribute(&member, "PXLE").unwrap(), None);
        assert_eq!(hvac_pseudo_attribute(&member, "").unwrap(), None);
    }

    /// `… OF PREV` 只截这一种形态：单个 `PREV`、无下标、无限定词；别的一律交还内层环境
    /// （内层不挂设计元素，H 族的 `:HXYsize[n]` 就是在这里答不上的）。
    #[test]
    fn only_a_bare_previous_element_chain_is_intercepted() {
        let set = std::sync::Arc::new(e3d_io::db_element::DbSet::new(std::sync::Arc::new(
            e3d_attlib::AttlibData::default(),
        )));
        let departure = set.element(e3d_io::refno::RefNo::new(0, 0));
        let env = PreviousElementEnv {
            inner: DbElementParamEnv::new(None, None),
            departure,
        };
        let prev = ["PREV"];
        // 库没注册：伪属性读库出错 → 环境答不上（不是 0）。
        assert_eq!(
            env.attribute_number(&AttribRef {
                name: "LEAWID",
                subscript: None,
                qualifier: None,
                owner_chain: &prev,
            }),
            None
        );
        let subscripted = AttribRef {
            name: ":HXYsize",
            subscript: Some(2.0),
            qualifier: None,
            owner_chain: &[],
        };
        assert_eq!(env.attribute_number(&subscripted), None);
        let two_owners = ["PREV", "PIPE"];
        assert_eq!(
            env.attribute_number(&AttribRef {
                name: "LEAWID",
                subscript: None,
                qualifier: None,
                owner_chain: &two_owners,
            }),
            None
        );
        assert_eq!(env.param(1), None);
    }

    /// `PreviousElementEnv` 不许把按字典键问的 UDA 吞掉。
    ///
    /// 由来：`9508ac4` 给 `ParamEnv` 加了 `uda_number`，默认实现是 `None`；本环境当时没转发，
    /// 于是目录件身上**真存着**的 UDA 也读不出来，而且无声（`ledger-seg15-frameall.md` §19.5）。
    /// 本库看不出差别——H 族头槽位那条路的键 `0x17EF4B61` 属于 `acp7006` 那本字典、元素上没值
    /// （§19.2）——所以这里不拿目录件当语料，直接拿一个**确实存着 `:HXYsize` 的设计元素**
    /// 当内层宿主：转发没接上时它答 `None`，接上了答 675 / 600。
    ///
    /// 语料同 `catalogue_visited_real.rs`（E3D 3.1 `AvevaMarineSample` + 影子模板）；值的出处是
    /// E3D 自己的 `Q :HXYsize`（`c4_hxysize.log`，§18.2 八条逐一对过）。缺语料默认红，
    /// 与那道门同一口径：确属无语料环境显式设 `E3D_SKIP_REAL_GATES=1`。
    #[test]
    fn a_uda_asked_by_dictionary_key_is_forwarded_to_the_inner_environment() {
        use std::path::{Path, PathBuf};
        use std::sync::Arc;

        use e3d_attlib::AttlibData;
        use e3d_io::db_element::{DbFilePin, DbSet, template_file_for};
        use e3d_io::refno::RefNo;
        use e3d_io::uda_catalog::UdaCatalog;

        const AMS_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaMarineSample\ams000";
        const DESIGN_DB: &str = "ams7997_0001";
        /// `*CNPESTD/DICT`：`/HXYsize` 的 `UKEY` 就在这本里。
        const DICTIONARY_DB: &str = "ams5100_0001";
        const TEMPLATE_DIR: &str = r"E:\reverse\e3d\shadow_e3d31_aps_all";
        /// `/-CCV-S-2-L-1405`，E3D `Q :HXYsize` = `675 600`。
        const BRAN_1405: RefNo = RefNo::new(24_381, 59_287);
        /// `ams5100` 那条 `/HXYsize` 的 `UKEY`（§18.1）。
        const HXYSIZE_AMS_KEY: u32 = 0x2652_AB52;

        let design = PathBuf::from(AMS_DIR).join(DESIGN_DB);
        let dictionary = PathBuf::from(AMS_DIR).join(DICTIONARY_DB);
        let dicvir = PathBuf::from(TEMPLATE_DIR).join("dicvir.dat");
        let attlib_file = PathBuf::from(TEMPLATE_DIR).join("attlib.dat");
        let missing = [
            design.as_path(),
            dictionary.as_path(),
            dicvir.as_path(),
            attlib_file.as_path(),
        ]
        .into_iter()
        .filter(|path| !path.exists())
        .map(|path| path.display().to_string())
        .collect::<Vec<_>>();
        if !missing.is_empty() {
            assert!(
                std::env::var_os("E3D_SKIP_REAL_GATES").is_some(),
                "缺语料：{}；确属无语料环境请显式设 E3D_SKIP_REAL_GATES=1",
                missing.join("、")
            );
            return;
        }

        let attlib = Arc::new(AttlibData::parse_file(&attlib_file).expect("attlib 解析得开"));
        let catalog =
            Arc::new(UdaCatalog::read(&dictionary, &dicvir, &attlib).expect("字典库读得开"));
        let definition = catalog
            .by_key(HXYSIZE_AMS_KEY)
            .expect("ams5100 里有这个 UKEY");
        assert_eq!(definition.name, "HXYsize");

        let set = Arc::new(DbSet::new(Arc::clone(&attlib)).with_uda_catalog(catalog));
        set.add_db(DbFilePin {
            file: design,
            template: template_file_for(Path::new(TEMPLATE_DIR), "DESI").expect("DESI 模板"),
            db_type: Some("DESI".to_string()),
            sesno: None,
        })
        .expect("设计库开得了");
        let bran = set.element(BRAN_1405);

        // 内层自己答得上——转发与否的差别才量得出来。
        let inner = DbElementParamEnv::catalogue_only(bran.clone()).without_insulation();
        assert_eq!(inner.uda_number(HXYSIZE_AMS_KEY, Some(1.0)), Some(675.0));

        let env = PreviousElementEnv {
            inner,
            departure: bran,
        };
        assert_eq!(
            (
                env.uda_number(HXYSIZE_AMS_KEY, Some(1.0)),
                env.uda_number(HXYSIZE_AMS_KEY, Some(2.0)),
            ),
            (Some(675.0), Some(600.0)),
            "没转发就会落回 ParamEnv::uda_number 的默认 None"
        );
        // 字典里没有的键仍然答不上（`uda_by_key` 在里面报 Err，不冒充 0）。
        assert_eq!(env.uda_number(0xDEAD_BEEF, Some(1.0)), None);
    }
}

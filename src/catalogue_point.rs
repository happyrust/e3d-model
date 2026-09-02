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

use std::fmt;

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
/// 表驱动规格项（`TABITE`）到目录零件（`GPART`）的那一跳。
const PART_REF: &str = "PRTREF";
/// 目录件指向 P 点集 `PTSE`。
const POINT_SET_REF: &str = "PTRE";
/// 直接给三个坐标的那种 P 点；其余（`PTAX`）是「轴 + 距离」。
const CARTESIAN_POINT: &str = "PTCA";

/// 方向向量短到这个长度以下就当没方向。
///
/// 与 [`crate::route::orimat`] 的退化阈值同源：定不出朝向的管身宁可记
/// `unbuildable`，也不能拿一个近乎零的向量归一化出一个随机方向。
const DIRECTION_TOL: f64 = 1.0e-9;
/// Core3D `pplib/NAXIS` subtracts 1000 from a point-axis number and negates
/// the resolved point direction.  The point position remains the axis datum.
const NEGATED_POINT_BASE: u32 = 1000;

/// 走目录链解 P 点的 [`PPointResolver`]。
///
/// 无状态：每次解都重走一遍链。ams8000 全库 2612 个构件 × 两端实测 5 s 量级，
/// 还不值得为它引一层缓存——缓存要按「构件 + 目录件」这一对做键（求值环境两边
/// 都要），键搞错了会把甲零件的尺寸发给乙零件，代价远大于省下的时间。
#[derive(Debug, Default, Clone, Copy)]
pub struct CataloguePoints;

/// One catalogue axis in the design member's local frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct CatalogueAxis {
    /// `Pn` axes pass through the referenced P point; named axes pass through
    /// the component origin.
    pub origin: DVec3,
    pub direction: DVec3,
}

/// Why a catalogue axis (or the `Pn` point behind it) did not resolve.
///
/// The text is what the local-failure note carries, so the account can tell
/// "the PTSE has no such point" (E3D draws nothing either) from "the point is
/// there but its direction or position did not evaluate" (our decoder gap).
/// Both used to surface as the same `缺属性 PAXI`.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum CatalogueAxisFailure {
    /// Neither a named axis nor a `Pn` reference.
    NotAnAxis { words: Vec<u32> },
    /// The component has no `PTRE` / `PTSE` to look a `Pn` up in.
    NoPointSet,
    /// `Pn` names a point the PTSE does not have.
    PointMissing { number: u32, available: Vec<u32> },
    /// The point's `PX`/`PY`/`PZ` (or `PDIS`) did not evaluate.
    PointPosition { number: u32, noun: String },
    /// The point's direction did not evaluate: `PTCD` program failed with no
    /// `DIR` behind it, `PAXI` unresolved, or a zero / non-finite vector.
    PointDirection { number: u32, noun: String },
    /// A named axis that came out zero or non-finite.
    Degenerate { words: Vec<u32> },
}

impl fmt::Display for CatalogueAxisFailure {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotAnAxis { words } => write!(f, "轴规格 {words:?} 既不是具名轴也不是 Pn"),
            Self::NoPointSet => write!(f, "目录件没有 PTRE/PTSE，Pn 无处可解"),
            Self::PointMissing { number, available } if available.is_empty() => {
                write!(f, "P{number} 不在 PTSET（PTSET 是空的）")
            }
            Self::PointMissing { number, available } => {
                write!(f, "P{number} 不在 PTSET（有")?;
                for n in available {
                    write!(f, " P{n}")?;
                }
                write!(f, "）")
            }
            Self::PointPosition { number, noun } => write!(f, "P{number}（{noun}）位置求不出"),
            Self::PointDirection { number, noun } => write!(f, "P{number}（{noun}）方向解不出"),
            Self::Degenerate { words } => write!(f, "轴 {words:?} 解出零向量或非有限方向"),
        }
    }
}

/// Which half of a P point failed to evaluate.
enum PointPart {
    Position,
    Direction,
}

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
    let Some(local) = catalogue_point_local(member, &component, number)? else {
        return Ok(None);
    };

    let world = world_matrix(member)?;
    let dir = world.transform_vector3(local.dir);
    if !(dir.length() > DIRECTION_TOL) || !dir.is_finite() {
        return Ok(None);
    }
    // 斜接弯头（切割拼成的 BEND）：管身端点取**构件自身原点**，不取目录
    // leave/arrive P 点。`isValidTubeLength`（Core3D `0x1057be50`）的端点选取原样：
    // 非 BRAN 端 `IsMitredBend ? POS : LPOS/APOS`；**方向仍读 LDIR/ADIR**，
    // 所以上面那步目录方向解析照旧，只有位置换源。构件原点经属主链折叠
    // （`world_matrix` 的平移列），与本函数输出的世界系口径一致。
    // 目录链断掉的斜接弯头仍如实解不出：方向没有别的来源，不拿缺省值顶。
    let pos = if is_mitred_bend(member)? {
        world.w_axis.truncate()
    } else {
        world.transform_point3(local.pos)
    };
    if !pos.is_finite() {
        return Ok(None);
    }

    Ok(Some(RoutePoint {
        pos,
        dir: dir.normalize(),
        bore: local.bore,
    }))
}

/// 斜接弯头判定，抄 `FAB_PipePieceManager::IsMitredBend`（Core3D `0x105773d0`）：
/// `DB_Element::type == NOUN_BEND && DB_Element::getInt(ATT_ACTNCU, 0) != 0`。
///
/// `ACTNCU`（actual number of cuts）非零 = 这只 BEND 是直段切割拼的斜接弯头，
/// 不是光滑弯管。但 `ACTNCU` 是**伪属性**（`DB_PseudoACTNCU` → `DB_ImplicitBend`
/// 构造器 `0x108BF610` / `getACTNCU` `0x108BFD50`），dabacon 记录里没有这一格；
/// 存储侧的展开是：`NCUTS` 为基值（字典缺省 0），`CPAR[3] >= 0` 时覆盖之
/// （缺省 −1 = 不覆盖），最后钳到 `[0, 25]`。字典不识 `NCUTS` 时 `getAtt` 失败、
/// `ACTNCU` 未定义，`getInt` 落回缺省 0——不是斜接，也不是错。
///
/// ams8000 622 / ams7997 3320 只 BEND 实测 `NCUTS` 全 0、`CPAR` 全缺席
/// （`examples/mitre_probe`）：现有语料没有斜接弯头，这条分支目前只有反编译
/// 背书，RVM 侧的最终确认要等带斜接弯头的语料（G0-2）。
fn is_mitred_bend(member: &DbElement) -> Result<bool, DbElementError> {
    if member.element_type()? != "BEND" {
        return Ok(false);
    }
    let ncuts = match member.attribute("NCUTS") {
        Ok(Some(DescriptorValue::Word { raw, .. })) => i64::from(raw),
        Ok(Some(DescriptorValue::Int(number))) => i64::from(number),
        // 属性没写：按字典缺省 0 起算，CPAR 仍可覆盖。
        Ok(None) => 0,
        // 形状意外：当不成计数，如实按「不是斜接」。
        Ok(Some(_)) => return Ok(false),
        Err(DbElementError::UnknownAttribute { .. }) => return Ok(false),
        Err(error) => return Err(error),
    };
    let cuts = match cpar_third(member)? {
        Some(value) if value >= 0.0 => value as i64,
        _ => ncuts,
    };
    Ok(cuts.clamp(0, 25) != 0)
}

/// `DB_ImplicitBend` 的覆盖臂：`CPAR` 第 3 个参数（一基），读不出按缺省 −1
/// 处理（即不覆盖 `NCUTS`）。
fn cpar_third(member: &DbElement) -> Result<Option<f64>, DbElementError> {
    let values = match member.get_double_array("CPAR") {
        Ok(values) => values,
        Err(DbElementError::UnknownAttribute { .. }) => None,
        Err(error) => return Err(error),
    };
    Ok(values.and_then(|values| values.get(2).copied()))
}

/// Resolve one catalogue P point in the design member's **local** frame.
///
/// Catalogue geometry uses the same point set as route endpoint placement:
/// `SEXT.PAAX = P6`, for example, means the direction of point 6.  Keeping the
/// local resolver here prevents the catalogue builder from reimplementing the
/// `SPRE -> SCOM -> PTRE` expression environment or accidentally applying the
/// design member's world matrix twice.
pub(crate) fn catalogue_point_local(
    member: &DbElement,
    component: &DbElement,
    number: u32,
) -> Result<Option<RoutePoint>, DbElementError> {
    Ok(catalogue_point_local_checked(member, component, number)?.ok())
}

/// [`catalogue_point_local`] with the reason for an unresolved point kept.
fn catalogue_point_local_checked(
    member: &DbElement,
    component: &DbElement,
    number: u32,
) -> Result<Result<RoutePoint, CatalogueAxisFailure>, DbElementError> {
    let point = match point_numbered(component, number)? {
        Ok(point) => point,
        Err(failure) => return Ok(Err(failure)),
    };
    let env =
        DbElementParamEnv::new(Some(member.clone()), Some(component.clone())).without_insulation();
    let noun = point.element_type()?;
    let local = match noun.as_str() {
        CARTESIAN_POINT => cartesian(&point, &env),
        "PTMI" => mixed(&point, &env),
        _ => axial(&point, &env),
    };
    let (pos, dir) = match local {
        Ok(local) => local,
        Err(PointPart::Position) => {
            return Ok(Err(CatalogueAxisFailure::PointPosition { number, noun }));
        }
        Err(PointPart::Direction) => {
            return Ok(Err(CatalogueAxisFailure::PointDirection { number, noun }));
        }
    };
    if !pos.is_finite() {
        return Ok(Err(CatalogueAxisFailure::PointPosition { number, noun }));
    }
    if !dir.is_finite() || !(dir.length() > DIRECTION_TOL) {
        return Ok(Err(CatalogueAxisFailure::PointDirection { number, noun }));
    }
    Ok(Ok(RoutePoint {
        pos,
        dir: dir.normalize(),
        bore: number_of(&point, "PBOR", &env),
    }))
}

/// `PTCA`：三个坐标表达式 + PTCD 方向规格。
fn cartesian(point: &DbElement, env: &DbElementParamEnv) -> Result<(DVec3, DVec3), PointPart> {
    let pos = DVec3::new(
        number_of(point, "PX", env).ok_or(PointPart::Position)?,
        number_of(point, "PY", env).ok_or(PointPart::Position)?,
        number_of(point, "PZ", env).ok_or(PointPart::Position)?,
    );
    // Core3D `pplib/QPDIR` (0x103BAC30) -> `DPARAM` (0x103BF9C0): the PTCD
    // program is authoritative when it is set, and an unset or empty PTCD reads
    // the `DIR` real array instead (its template default is local +Y).  The
    // YKS handwheel points P4/P5/P7 carry no PTCD at all; E3D draws their
    // spokes and rim along DIR, so a missing PTCD must not fail the point.
    // DIR stays a fallback rather than the authority because expression-backed
    // PTCA directions (PCLA torus points with ASIN(PARA[...]) in PTCD) differ
    // from it.
    let words = match point.attribute("PTCD").ok().flatten() {
        Some(DescriptorValue::RawWords(words) | DescriptorValue::WordArray(words)) => words,
        Some(DescriptorValue::IntArray(words)) => {
            words.into_iter().map(|word| word as u32).collect()
        }
        Some(_) | None => Vec::new(),
    };
    let dir = direction_spec::direction(&words, env)
        .map(DVec3::from_array)
        .or_else(|| {
            point
                .get_direction("DIR")
                .ok()
                .flatten()
                .map(DVec3::from_array)
        })
        .ok_or(PointPart::Direction)?;
    Ok((pos, dir))
}

fn mixed(point: &DbElement, env: &DbElementParamEnv) -> Result<(DVec3, DVec3), PointPart> {
    let pos = DVec3::new(
        number_of(point, "PX", env).ok_or(PointPart::Position)?,
        number_of(point, "PY", env).ok_or(PointPart::Position)?,
        number_of(point, "PZ", env).ok_or(PointPart::Position)?,
    );
    let words = axis_words(point).ok_or(PointPart::Direction)?;
    let dir = axis_spec::direction(&words, env).ok_or(PointPart::Direction)?;
    Ok((pos, DVec3::from_array(dir)))
}

/// `SPRE`（→ `CATR`）→ 目录件。
pub(crate) fn catalogue_component(member: &DbElement) -> Result<Option<DbElement>, DbElementError> {
    let Some(current) = member.get_element(SPEC_REF)? else {
        return Ok(None);
    };
    follow_catalogue_component(current)
}

/// Resolve a catalogue axis, including Core's `Pn` form.
///
/// Plain/named axes stay delegated to e3d-io's single axis decoder.  Only the
/// form that necessarily needs the surrounding PTSE (`[3,n]`, optionally
/// count-prefixed as `[2,3,n]`) is completed here from the shared P-point
/// resolver. Both the direction and the datum come back; when the axis does
/// not resolve, the reason does, so the caller's local-failure note can say
/// which of the two things happened.
pub(crate) fn resolve_catalogue_axis(
    member: &DbElement,
    component: &DbElement,
    words: &[u32],
) -> Result<Result<CatalogueAxis, CatalogueAxisFailure>, DbElementError> {
    let env =
        DbElementParamEnv::new(Some(member.clone()), Some(component.clone())).without_insulation();
    if let Some(direction) = axis_spec::direction(words, &env) {
        let direction = DVec3::from_array(direction);
        return Ok(
            if direction.length() > DIRECTION_TOL && direction.is_finite() {
                Ok(CatalogueAxis {
                    origin: DVec3::ZERO,
                    direction: direction.normalize(),
                })
            } else {
                Err(CatalogueAxisFailure::Degenerate {
                    words: words.to_vec(),
                })
            },
        );
    }
    let payload = match words {
        [3, number] => Some([*number]),
        [count, rest @ ..] if *count as usize == rest.len() => match rest {
            [3, number] => Some([*number]),
            _ => None,
        },
        _ => None,
    };
    let Some([encoded_number]) = payload else {
        return Ok(Err(CatalogueAxisFailure::NotAnAxis {
            words: words.to_vec(),
        }));
    };
    let (number, negated) = if encoded_number >= NEGATED_POINT_BASE {
        (encoded_number - NEGATED_POINT_BASE, true)
    } else {
        (encoded_number, false)
    };
    Ok(
        catalogue_point_local_checked(member, component, number)?.map(|point| CatalogueAxis {
            origin: point.pos,
            direction: if negated { -point.dir } else { point.dir },
        }),
    )
}

/// Resolve the runtime TUBI component selected by a route departure element.
///
/// `GTTUBE` chooses the stub on the **departure** side (`HSTU`/`HSRO` for a
/// container head, `LSTU`/`LSRO` for a member leave).  The stub is normally an
/// `SPCO`; its `CATR` reaches the `SCOM` whose one-based `PARA[2]` is CGETOD's
/// outside diameter.  Keeping this beside the P-point resolver means both
/// consumers share the same bounded SPCO -> SCOM traversal.
pub(crate) fn tube_outer_diameter_from_stub(
    departure: &DbElement,
    stub_attributes: &[&str],
) -> Result<Option<f64>, DbElementError> {
    for name in stub_attributes {
        let start = match departure.get_element(name) {
            Ok(start) => start,
            Err(DbElementError::UnknownAttribute { .. }) => None,
            Err(error) => return Err(error),
        };
        let Some(start) = start else {
            continue;
        };
        let Some(component) = follow_catalogue_component(start)? else {
            continue;
        };
        let para = match component.get_double_array("PARA") {
            Ok(para) => para,
            Err(DbElementError::UnknownAttribute { .. }) => None,
            Err(error) => return Err(error),
        };
        if let Some(diameter) = para
            .as_deref()
            .and_then(|values| values.get(1))
            .copied()
            .filter(|value| value.is_finite() && *value > 0.0)
        {
            return Ok(Some(diameter));
        }
    }
    Ok(None)
}

fn follow_catalogue_component(mut current: DbElement) -> Result<Option<DbElement>, DbElementError> {
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
            Ok(Some(next)) => Some(next),
            // SELE 等选择节点可能出现在规格引用链中，但它们没有 CATR；这是链到此
            // 无法继续，不是模板或文件损坏。
            //
            // E3D 3.1 的表驱动规格（电缆桥架 `CTRAY.SPRE`）多一跳：`TABITE` 没有 CATR，
            // 经 `PRTREF` 到 `GPART`，`GPART.CATR` 才到 SCOM（`catalogue_probe` 已跟到底）。
            Ok(None) | Err(DbElementError::UnknownAttribute { .. }) => {
                match current.get_element(PART_REF) {
                    Ok(next) => next,
                    Err(DbElementError::UnknownAttribute { .. }) => None,
                    Err(error) => return Err(error),
                }
            }
            Err(error) => return Err(error),
        };
        let Some(next) = next else {
            return Ok(None);
        };
        current = next;
    }
    Ok(None)
}

/// 目录件的 `PTSE` 里编号为 `number` 的那个 P 点；没有就把有的编号一并交出去，
/// 记账时能看出是"P 点根本不存在"而不是"解不出"。
fn point_numbered(
    component: &DbElement,
    number: u32,
) -> Result<Result<DbElement, CatalogueAxisFailure>, DbElementError> {
    let Some(set) = component.get_element(POINT_SET_REF)? else {
        return Ok(Err(CatalogueAxisFailure::NoPointSet));
    };
    let mut available = Vec::new();
    for point in set.members()? {
        match point_number(&point) {
            Some(found) if found == number => return Ok(Ok(point)),
            Some(found) => available.push(found),
            None => {}
        }
    }
    available.sort_unstable();
    Ok(Err(CatalogueAxisFailure::PointMissing {
        number,
        available,
    }))
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
fn axial(point: &DbElement, env: &DbElementParamEnv) -> Result<(DVec3, DVec3), PointPart> {
    let axis = axis_words(point).ok_or(PointPart::Direction)?;
    let dir = DVec3::from_array(axis_spec::direction(&axis, env).ok_or(PointPart::Direction)?);
    let distance = number_of(point, "PDIS", env).ok_or(PointPart::Position)?;
    Ok((dir * distance, dir))
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

/// 一个属性读成数，含它可能带的目录表达式。
///
/// Core3D `pplib/SPARAM` (`0x103C4BB4`) 对四词全零元组返回显式数值 0；它不是
/// “求不出”。PTCA 点大量用这个编码表示原点坐标，若当 unresolved，`Pn` 目录轴和
/// 路由端点会一起消失。
fn number_of(element: &DbElement, name: &str, env: &DbElementParamEnv) -> Option<f64> {
    let outcome = element.get_number(name, env).ok()?;
    outcome.value().or_else(|| {
        matches!(
            element.attribute(name),
            Ok(Some(DescriptorValue::RawWords(words)))
                if !words.is_empty() && words.iter().all(|word| *word == 0)
        )
        .then_some(0.0)
    })
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

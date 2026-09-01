//! 单元素建模（对标 `build/ELMODL` + 负几何 `build/LNEGIT`）。
//!
//! 一个正实体元素在这里走完：取剖面/参数 → libgm 语义建局部实体 →
//! 世界矩阵放置 → 收集负几何子元素（含 CMPF 里的）→ 世界系 CSG 扣减。
//!
//! 失败口径：
//! - **数据坏**（环退化、属性缺到建不出体、owner 链断）→ `Err(ModelError)`，
//!   由管线记进 `failed`，响亮。
//! - **E3D 也不出几何**（高度 ≈ 0，`loop_model` 的既有跳过口径）→
//!   [`Modeled::Skipped`]，记进 `skipped`，有数可查。
//! - **本期不做**（NCON/NDIS 一族曲面基本体）→ 同样 `Skipped`，reason 说清是
//!   里程碑边界。
//! - 负几何单个失败**不拖垮正体**：记进 [`GeneratedElement::negatives_skipped`]，
//!   正体照出——RVM 对拍时这类元素会以体积偏大暴露，报告里有对应记录。

use glam::{DMat4, DVec3};
use manifold_csg::Manifold;

use e3d_io::db_element::{DbElement, DbElementError};
use e3d_io::refno::RefNo;

use crate::catalogue::{CatalogueMeshCache, model_catalogue_member_with_cache};
use crate::category::{Category, Prim, classify};
use crate::discretise::{flatten_ring, has_core3d_skipped_link};
use crate::extrusion_reuse::{ExtrusionVariant, cached_extrusion};
use crate::polyhedron::{PolyhedronError, polyhedron_faces, polyhedron_solid};
use crate::primitive_instance::{
    PrimitiveBuild, PrimitiveInstance, instantiate_box, instantiate_cylinder,
};
use crate::profile::{LoopProfile, ProfileError, loop_profile};
use crate::solid::{
    MANIFOLD_NUDGE_MM, SolidError, circ_torus_solid, cleanup_floor_boolean_components,
    csg_cylinder_solid, csg_cylinder_solid_with_segments, extrude_rings, inflate_about_center,
    placed, pyramid_solid, rect_torus_solid, revolve_rings_inflated, snout_segments, snout_solid,
    subtract_negatives,
};
use crate::transform::world_matrix;

/// SJUS 下移只对这几个 noun 生效（gen-model `loop_model` 的既有闸门；
/// NXTR 自己的 PLOO 带 SJUS 时不动，只记 note，等 RVM 对拍裁决）。
const SJUS_NOUNS: [&str; 3] = ["PANE", "FLOOR", "GWALL"];

#[derive(Debug, thiserror::Error)]
pub enum ModelError {
    #[error(transparent)]
    Db(#[from] DbElementError),
    #[error(transparent)]
    Profile(#[from] ProfileError),
    #[error(transparent)]
    Polyhedron(#[from] PolyhedronError),
    #[error(transparent)]
    Solid(#[from] SolidError),
    #[error("{refno}（{noun}）外环折线化后退化，建不出剖面")]
    OuterRingCollapsed { refno: String, noun: String },
    #[error("{refno}（{noun}）缺属性 {name}，建不出 {what}")]
    MissingAttribute {
        refno: String,
        noun: String,
        name: &'static str,
        what: &'static str,
    },
    #[error("{refno}（{noun}）属性 {name} 求值失败，建不出 {what}: {detail}")]
    UnresolvedAttribute {
        refno: String,
        noun: String,
        name: &'static str,
        what: &'static str,
        detail: String,
    },
}

/// Core3D validates primitive dimensions before asking libgm for a body. A
/// dimensionally empty primitive is therefore an explicit no-shape result,
/// not a decoder failure. Keep missing attributes and I/O/type errors loud.
fn primitive_no_shape_reason(error: &ModelError) -> Option<String> {
    match error {
        ModelError::Solid(SolidError::BadHeight { .. })
        | ModelError::Solid(SolidError::Degenerate { .. }) => Some(error.to_string()),
        _ => None,
    }
}

/// An absent outline, or a physically present loop with zero vertices, gives
/// Core3D no positive section to extrude. One/two-point rings remain malformed
/// data and stay in `failed`; only the unambiguous no-body cases are skipped.
fn profile_no_shape_reason(error: &ModelError) -> Option<String> {
    match error {
        ModelError::Profile(ProfileError::NoLoops { .. })
        | ModelError::Profile(ProfileError::DegenerateRing { points: 0, .. }) => {
            Some(error.to_string())
        }
        _ => None,
    }
}

/// SBFI 套管的产物名，同时是「这条错来自套管本体」的判据。
const SBFI_WHAT: &str = "SBFI annular sleeve";

/// 套管尺寸至少要凑齐这么多个 `DESP` 分量：`[ID, OD, THK, LENGTH, .., X, Y, Z]`。
const SBFI_DESP_MIN: usize = 10;

/// SBFI 的套管尺寸直接读设计侧 `DESP`，凑不齐分量的那些**不是套管**，不是坏数据。
///
/// 这条门一度只写在 `pipeline.rs` 的 match 守卫里（`desp.len() >= 10`），产出侧没有。
/// 于是同一个短 `DESP` 的 SBFI：全量那边守卫不匹配、当没这件事；增量那边直接调
/// [`model_element`]，拿到 `Degenerate` 记成 `regen_failed` 并发 Remove。
/// 两边的**动作集**恰好还对得上（目标端本来也没有它），只有账不一样——
/// 没有任何等式判据会因此变红，跟 `SBFI` 那次静默漏件是同一个形状：判据写了两份。
///
/// 所以门收在产出侧，全量与增量共用同一条 [`model_element`]，`pipeline.rs` 不再另写。
fn sbfitting_no_shape_reason(error: &ModelError) -> Option<String> {
    match error {
        // DESP 缺席或短于 10 个分量：这个 SBFI 没带套管尺寸。
        ModelError::MissingAttribute { name: "DESP", .. } => Some(error.to_string()),
        ModelError::Solid(SolidError::Degenerate { what, .. }) if *what == SBFI_WHAT => {
            Some(error.to_string())
        }
        _ => None,
    }
}

/// 一条被跳过的负几何（不拖垮正体，但必须有名有姓）。
#[derive(Debug, Clone)]
pub struct SkippedNegative {
    pub refno: RefNo,
    pub noun: String,
    pub reason: String,
}

/// 建成的元素：世界系实体 + 账目。
#[derive(
    Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize,
)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum GeometryId {
    /// 库中真实存在的元素。
    Element { refno: String },
    /// 运行时派生管身：由容器和它跨过的**两个站点**共同确定。
    ///
    /// # 为什么是两端而不是序号
    ///
    /// 这里曾经是 `{container, preceding, ordinal}`，`ordinal` 就是槽位在容器里的
    /// 下标（`route::Route::tubes` 条数恒为成员数 + 1）。于是在 BRAN 头部插进一个
    /// 构件，**后面每一根管身的键都平移一位**——两端做差分看到的是「全部旧管身
    /// 消失、全部新管身出现」，逐根比对彻底失去意义，而这正是增量要用它的地方。
    ///
    /// 换成两端站点后，在 A、B 之间插进 X：只有 `(A, B)` 这一条消失，
    /// `(A, X)` 与 `(X, B)` 两条新增，**其余键一个都不动**。
    /// 链头站与链尾站都站在容器自己身上（头点 `HPOS`、尾点 `TPOS` 是容器的属性），
    /// 所以空容器那唯一一条槽位退化成 `(c, c, c)`，仍然是一个合法且唯一的键。
    ///
    /// 同一个容器内相邻站点对唯一，键因此不需要序号来去重。
    ImpliedTube {
        container_refno: String,
        from_refno: String,
        to_refno: String,
    },
}

impl std::fmt::Display for GeometryId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            GeometryId::Element { refno } => write!(f, "element:{refno}"),
            GeometryId::ImpliedTube {
                container_refno,
                from_refno,
                to_refno,
            } => write!(f, "implied-tube:{container_refno}:{from_refno}:{to_refno}"),
        }
    }
}

pub struct GeneratedElement {
    /// 几何缓存、RVM 配对和增量更新使用的稳定身份。
    pub geometry_id: GeometryId,
    /// 来源元素保持不变；派生几何也不得用伪 refno 覆盖它。
    pub refno: RefNo,
    pub noun: String,
    pub name: Option<String>,
    /// 世界系实体，负几何已扣。
    pub solid: Manifold,
    /// 本元素局部系到世界系的刚体变换。实体里的 SJUS 下移保留在局部几何中，
    /// 因而把最终网格乘这个矩阵的逆即可得到与 E3D facet 可比的元素局部坐标。
    pub world: DMat4,
    /// Present only when the visible result is an unbooled reusable primitive.
    /// Persistence may write this as legacy `inst_geo + geo_relate.trans`
    /// instead of allocating a per-element `booled_id` mesh.
    pub primitive_instance: Option<PrimitiveInstance>,
    pub negatives_applied: usize,
    pub negatives_skipped: Vec<SkippedNegative>,
    /// 倒圆越界、SJUS 未生效等非致命观察。
    pub notes: Vec<String>,
}

/// 建模结论：出了实体 / 有依据地没出。
pub enum Modeled {
    Solid(Box<GeneratedElement>),
    Skipped { reason: String },
}

/// 对一个**正实体**类别的元素建模。调用方负责用 [`classify`] 先分好类；
/// 传进来的类别不产正体时按里程碑边界跳过。
///
/// 每次调用自带一次性网格池——批量调用方（全量管线、增量执行）应改走
/// [`model_element_with_cache`] 共享同一个池，语义一字不差，只省重复建体。
pub fn model_element(el: &DbElement, category: Category) -> Result<Modeled, ModelError> {
    model_element_with_cache(el, category, &mut CatalogueMeshCache::default())
}

/// 与 [`model_element`] 相同，但由调用者持有世代网格池：同批内容相同的
/// 环拉伸（正体与负体）只建一次网格，命中者取克隆——克隆与直建同输入
/// 逐位一致，输出身份、世界矩阵、CSG 次序均不受影响。
pub fn model_element_with_cache(
    el: &DbElement,
    category: Category,
    cache: &mut CatalogueMeshCache,
) -> Result<Modeled, ModelError> {
    // 路由成员的实体不是设计侧正体，而是目录 GMRE 程序求值的产物——与全量管线
    // pipeline 的 RouteMember 臂共用同一个 [`model_catalogue_member_with_cache`]。
    // 这条臂必须存在的理由与 `category::is_model_unit` 文档里那次 SBFI 事故同型：
    // 全量为成员产出 element:refno 键的实体，增量上卷到成员后走的是这里；两边
    // 不是同一份答案，被删/被改的成员实体就既不重建也不出 Remove。
    //
    // 有意不走下面共享的负体扣减路：成员的负体语义在目录侧（SBOX 底下的
    // NSCY/NSBO/NLCY，已在求值臂内扣完），设计侧子负体在全量管线里同样不由
    // 路由成员消费，两边照旧各记各的账。
    if category == Category::RouteMember {
        let build = model_catalogue_member_with_cache(el, cache)?;
        return Ok(match build.generated {
            Some(generated) => Modeled::Solid(Box::new(generated)),
            None => Modeled::Skipped {
                reason: format!(
                    "目录侧无可见几何（无 SPRE/GMRE 或成员全退化；局部失败 {} 条），E3D 同样不出几何",
                    build.diagnostics.len()
                ),
            },
        });
    }
    let noun = el.element_type()?;
    let mut notes = Vec::new();
    // Decide the instancing boundary before constructing the positive body.
    // Any boolean operand stays at real dimensions: unit scaling can differ by
    // a few ulps and coincident-face CSG may amplify that into another topology.
    let mut negatives = Vec::new();
    collect_negatives(el, &mut negatives)?;
    let had_negative_members = !negatives.is_empty();

    let (local, primitive_instance) = match category {
        // 与基本体同一条实例化边界：正体一旦有负体要扣，就按布尔参与者烘焙直建。
        Category::LoopExtrusion => match extrusion_local_solid(
            el,
            &noun,
            true,
            0.0,
            &mut notes,
            cache,
            !had_negative_members,
        ) {
            Ok(LocalSolid::Solid(s)) => (s, None),
            Ok(LocalSolid::ZeroHeight(h)) => {
                return Ok(Modeled::Skipped {
                    reason: format!("HEIG={h}（≈0，E3D 同样不出几何）"),
                });
            }
            Err(error) => {
                if let Some(reason) = profile_no_shape_reason(&error) {
                    return Ok(Modeled::Skipped {
                        reason: format!("{reason}（无正轮廓，Core3D 不产几何）"),
                    });
                }
                return Err(error);
            }
        },
        Category::Primitive(prim) => {
            let built = match primitive_local_solid(el, &noun, prim, !had_negative_members) {
                Ok(built) => built,
                Err(error) => {
                    if let Some(reason) = primitive_no_shape_reason(&error) {
                        return Ok(Modeled::Skipped {
                            reason: format!("{reason}（Core3D 尺寸校验不产几何）"),
                        });
                    }
                    return Err(error);
                }
            };
            (built.solid, built.instance)
        }
        Category::Revolution => (revolution_local_solid(el, &noun, 0.0, &mut notes)?, None),
        Category::Polyhedron => (polyhedron_local_solid(el, &mut notes)?, None),
        Category::Catalog if noun == "SBFI" => match sbfitting_local_solid(el, &mut notes) {
            Ok(solid) => (solid, None),
            Err(error) => {
                if let Some(reason) = sbfitting_no_shape_reason(&error) {
                    return Ok(Modeled::Skipped {
                        reason: format!("{reason}（DESP 没带套管尺寸，E3D 同样不出几何）"),
                    });
                }
                return Err(error);
            }
        },
        Category::Catalog if noun == "WALL" => (wall_local_solid(el, &mut notes)?, None),
        other => {
            return Ok(Modeled::Skipped {
                reason: format!("类别 {other:?} 不产正实体"),
            });
        }
    };

    let world = if category == Category::Catalog && noun == "SBFI" {
        world_matrix(el)? * sbfitting_catalogue_alignment(el)?
    } else {
        world_matrix(el)?
    };
    let pos_world = placed(&local, world);

    // 负几何（LNEGIT）：直属负体 + 直属 CMPF 里的深层负体。
    let mut applied = Vec::new();
    let mut skipped = Vec::new();
    for neg in negatives {
        let reason = match negative_world_solid(&neg, noun == "SNOU", cache) {
            Ok(NegativeSolid::Solid(solid)) => {
                applied.push(solid);
                continue;
            }
            Ok(NegativeSolid::Skipped(reason)) => reason,
            Err(error) => error.to_string(),
        };
        skipped.push(SkippedNegative {
            refno: neg.refno(),
            noun: neg.element_type().unwrap_or_else(|_| "?".into()),
            reason,
        });
    }
    if noun == "WALL" {
        let mut fixings = Vec::new();
        collect_host_fixings(el, &mut fixings)?;
        for fixing in fixings {
            match fixing_world_cutter(el, &fixing) {
                Ok(solid) => applied.push(solid),
                Err(error) => skipped.push(SkippedNegative {
                    refno: fixing.refno(),
                    noun: "FIXING".into(),
                    reason: error.to_string(),
                }),
            }
        }
    }
    let negatives_applied = applied.len();
    let mut solid = subtract_negatives(pos_world, applied);
    if noun == "FLOOR" {
        solid = cleanup_floor_boolean_components(solid, negatives_applied);
    }
    if solid.is_empty() || solid.num_tri() == 0 {
        if had_negative_members {
            return Ok(Modeled::Skipped {
                reason: format!(
                    "应用 {negatives_applied} 个负体后可见实体为空（Core3D 同样无可显示几何）"
                ),
            });
        }
        return Err(ModelError::Solid(SolidError::EmptyManifold {
            what: "element after negatives",
        }));
    }

    Ok(Modeled::Solid(Box::new(GeneratedElement {
        geometry_id: GeometryId::Element {
            refno: el.refno().to_string(),
        },
        refno: el.refno(),
        noun,
        name: el.stored_name()?,
        solid,
        world,
        primitive_instance: (!had_negative_members)
            .then_some(primitive_instance)
            .flatten(),
        negatives_applied,
        negatives_skipped: skipped,
        notes,
    })))
}

/// Build a design WALL from its SPINE.  The wall profile is the catalogue
/// rectangle `(thickness, height)` carried by DESP; the design SPINE supplies
/// the path.  The first supported path is E3D's common three-point circular
/// spine (`POINSP, CURVE THRU, POINSP`).
fn wall_local_solid(el: &DbElement, notes: &mut Vec<String>) -> Result<Manifold, ModelError> {
    let desp = el
        .get_double_array("DESP")?
        .ok_or_else(|| ModelError::MissingAttribute {
            refno: el.refno().to_string(),
            noun: "WALL".into(),
            name: "DESP",
            what: "WALL profile",
        })?;
    if desp.len() < 2 {
        return Err(ModelError::UnresolvedAttribute {
            refno: el.refno().to_string(),
            noun: "WALL".into(),
            name: "DESP",
            what: "WALL profile",
            detail: format!("DESP length {} < 2", desp.len()),
        });
    }
    let (thickness, height) = (desp[0], desp[1]);
    if !(thickness > f64::EPSILON && height > f64::EPSILON) {
        return Err(ModelError::Solid(SolidError::Degenerate {
            what: "WALL profile",
            detail: format!("thickness/height = {thickness}/{height}"),
        }));
    }

    let spine = el
        .members()?
        .find_map(|member| match member.element_type() {
            Ok(noun) if noun == "SPINE" => Some(Ok(member)),
            Ok(_) => None,
            Err(error) => Some(Err(error)),
        })
        .transpose()?
        .ok_or_else(|| ModelError::MissingAttribute {
            refno: el.refno().to_string(),
            noun: "WALL".into(),
            name: "SPINE",
            what: "WALL path",
        })?;
    let path = spine.members()?.collect::<Vec<_>>();
    if path.len() != 3 {
        return Err(ModelError::UnresolvedAttribute {
            refno: el.refno().to_string(),
            noun: "WALL".into(),
            name: "SPINE",
            what: "three-point circular WALL path",
            detail: format!("member count {} != 3", path.len()),
        });
    }
    let nouns = path
        .iter()
        .map(DbElement::element_type)
        .collect::<Result<Vec<_>, _>>()?;
    if nouns != ["POINSP", "CURVE", "POINSP"] {
        return Err(ModelError::UnresolvedAttribute {
            refno: el.refno().to_string(),
            noun: "WALL".into(),
            name: "SPINE",
            what: "three-point circular WALL path",
            detail: format!("members = {nouns:?}"),
        });
    }
    let point = |node: &DbElement| -> Result<DVec3, ModelError> {
        node.get_position("POS")?
            .map(DVec3::from_array)
            .ok_or_else(|| ModelError::MissingAttribute {
                refno: node.refno().to_string(),
                noun: node.element_type().unwrap_or_else(|_| "?".into()),
                name: "POS",
                what: "WALL spine point",
            })
    };
    let (p0, pm, p1) = (point(&path[0])?, point(&path[1])?, point(&path[2])?);
    if p0.z.abs().max(pm.z.abs()).max(p1.z.abs()) > 0.051 {
        return Err(ModelError::UnresolvedAttribute {
            refno: spine.refno().to_string(),
            noun: "SPINE".into(),
            name: "POS",
            what: "planar WALL spine",
            detail: format!("non-planar z = [{}, {}, {}]", p0.z, pm.z, p1.z),
        });
    }
    let (center, radius) = circle_through_xy(p0, pm, p1)?;
    let a0 = (p0.y - center.y).atan2(p0.x - center.x);
    let am = (pm.y - center.y).atan2(pm.x - center.x);
    let a1 = (p1.y - center.y).atan2(p1.x - center.x);
    let ccw_mid = positive_angle(am - a0);
    let ccw_end = positive_angle(a1 - a0);
    let sweep = if ccw_mid <= ccw_end + 1e-9 {
        ccw_end
    } else {
        -(std::f64::consts::TAU - ccw_end)
    };
    let inner = radius - thickness;
    if !(inner >= 0.0 && sweep.abs() > 1e-9) {
        return Err(ModelError::Solid(SolidError::Degenerate {
            what: "WALL circular spine",
            detail: format!(
                "radius/thickness/sweep = {radius}/{thickness}/{}deg",
                sweep.to_degrees()
            ),
        }));
    }
    let torus = rect_torus_solid(inner, radius, height, sweep.to_degrees())?;
    let local = DMat4::from_translation(DVec3::new(center.x, center.y, height * 0.5))
        * DMat4::from_rotation_z(a0)
        * DMat4::from_rotation_x(-std::f64::consts::FRAC_PI_2);
    notes.push(format!(
        "WALL circular SPINE center=({:.6},{:.6}) radius={radius:.6} sweep={:.6}deg",
        center.x,
        center.y,
        sweep.to_degrees()
    ));
    Ok(placed(&torus, local))
}

fn positive_angle(angle: f64) -> f64 {
    angle.rem_euclid(std::f64::consts::TAU)
}

fn circle_through_xy(p0: DVec3, p1: DVec3, p2: DVec3) -> Result<(DVec3, f64), ModelError> {
    let d = 2.0 * (p0.x * (p1.y - p2.y) + p1.x * (p2.y - p0.y) + p2.x * (p0.y - p1.y));
    if d.abs() <= 1e-9 {
        return Err(ModelError::Solid(SolidError::Degenerate {
            what: "WALL circular spine",
            detail: "three points are collinear".into(),
        }));
    }
    let q0 = p0.x * p0.x + p0.y * p0.y;
    let q1 = p1.x * p1.x + p1.y * p1.y;
    let q2 = p2.x * p2.x + p2.y * p2.y;
    let center = DVec3::new(
        (q0 * (p1.y - p2.y) + q1 * (p2.y - p0.y) + q2 * (p0.y - p1.y)) / d,
        (q0 * (p2.x - p1.x) + q1 * (p0.x - p2.x) + q2 * (p1.x - p0.x)) / d,
        0.0,
    );
    Ok((center, (p0 - center).length()))
}

fn collect_host_fixings(el: &DbElement, out: &mut Vec<DbElement>) -> Result<(), DbElementError> {
    for member in el.members()? {
        if member.element_type()? == "FIXING" {
            if member
                .get_double_array("DESP")?
                .is_some_and(|desp| desp.len() >= SBFI_DESP_MIN)
            {
                out.push(member);
            }
        } else {
            collect_host_fixings(&member, out)?;
        }
    }
    Ok(())
}

fn fixing_world_cutter(wall: &DbElement, fixing: &DbElement) -> Result<Manifold, ModelError> {
    let desp = fixing
        .get_double_array("DESP")?
        .ok_or_else(|| ModelError::MissingAttribute {
            refno: fixing.refno().to_string(),
            noun: "FIXING".into(),
            name: "DESP",
            what: "host cutter",
        })?;
    let pldatu = fixing
        .owner()?
        .ok_or_else(|| ModelError::MissingAttribute {
            refno: fixing.refno().to_string(),
            noun: "FIXING".into(),
            name: "OWNER",
            what: "PLDATU host frame",
        })?;
    let jldatu = pldatu
        .owner()?
        .ok_or_else(|| ModelError::MissingAttribute {
            refno: pldatu.refno().to_string(),
            noun: "PLDATU".into(),
            name: "OWNER",
            what: "JLDATU host frame",
        })?;
    let zdis = require_double(&jldatu, "JLDATU", "ZDIS", "WALL joint datum")?;
    let (p0, pm, p1) = wall_spine_points(wall)?;
    let (curve_center, radius) = circle_through_xy(p0, pm, p1)?;
    let a0 = (p0.y - curve_center.y).atan2(p0.x - curve_center.x);
    let theta = a0 + zdis / radius;
    let station = curve_center + DVec3::new(radius * theta.cos(), radius * theta.sin(), 0.0);
    let inward = (curve_center - station).normalize();
    let up = DVec3::Z;
    let along = inward.cross(up).normalize();
    let joint = DMat4::from_cols(
        inward.extend(0.0),
        up.extend(0.0),
        along.extend(0.0),
        station.extend(1.0),
    );
    // PLDATU POSL=OBOW is E3D's implicit wall pline: PLAX=-X. With
    // YDIR=+Z, cal_ori_by_ydir gives columns (-Y,+Z,-X).
    let pldatum = DMat4::from_cols(
        DVec3::NEG_Y.extend(0.0),
        DVec3::Z.extend(0.0),
        DVec3::NEG_X.extend(0.0),
        DVec3::ZERO.extend(1.0),
    );
    let local_to_wall = joint * pldatum * crate::transform::local_matrix(fixing)?;
    let local = match desp.as_slice() {
        [_, outer_diam, _, length, _, _, _, x, y, z, ..] if *outer_diam > 0.0 && *length > 0.0 => {
            let segments = if *outer_diam <= 240.0 { 8 } else { 12 };
            csg_cylinder_solid_with_segments(*outer_diam, *length, segments)?.translate(
                *x,
                *y,
                *z - *length * 0.5,
            )
        }
        _ => {
            return Err(ModelError::UnresolvedAttribute {
                refno: fixing.refno().to_string(),
                noun: "FIXING".into(),
                name: "DESP",
                what: "host cutter",
                detail: format!("unsupported layout {desp:?}"),
            });
        }
    };
    Ok(placed(&local, world_matrix(wall)? * local_to_wall))
}

fn wall_spine_points(wall: &DbElement) -> Result<(DVec3, DVec3, DVec3), ModelError> {
    let spine = wall
        .members()?
        .find_map(|member| {
            (member.element_type().ok().as_deref() == Some("SPINE")).then_some(member)
        })
        .ok_or_else(|| ModelError::MissingAttribute {
            refno: wall.refno().to_string(),
            noun: "WALL".into(),
            name: "SPINE",
            what: "WALL host frame",
        })?;
    let path = spine.members()?.collect::<Vec<_>>();
    if path.len() != 3 {
        return Err(ModelError::UnresolvedAttribute {
            refno: spine.refno().to_string(),
            noun: "SPINE".into(),
            name: "MEMBERS",
            what: "WALL host frame",
            detail: format!("member count {} != 3", path.len()),
        });
    }
    let point = |node: &DbElement| -> Result<DVec3, ModelError> {
        node.get_position("POS")?
            .map(DVec3::from_array)
            .ok_or_else(|| ModelError::MissingAttribute {
                refno: node.refno().to_string(),
                noun: node.element_type().unwrap_or_else(|_| "?".into()),
                name: "POS",
                what: "WALL host frame",
            })
    };
    Ok((point(&path[0])?, point(&path[1])?, point(&path[2])?))
}

/// Build the design-side SBFI sleeve carried directly by `DESP`.
///
/// E3D's RVM for this family is an annular extrusion, not the two solid
/// cylinders used by the legacy generator.  The observed descriptor layout is
/// `[ID, OD, THK, LENGTH, .., X, Y, Z]` (millimetres); the sleeve ends at its
/// local datum, hence it occupies `Z-LENGTH .. Z`.
fn sbfitting_local_solid(el: &DbElement, notes: &mut Vec<String>) -> Result<Manifold, ModelError> {
    let refno = el.refno().to_string();
    let desp = el
        .get_double_array("DESP")?
        .ok_or_else(|| ModelError::MissingAttribute {
            refno: refno.clone(),
            noun: "SBFI".into(),
            name: "DESP",
            what: "SBFI annular sleeve",
        })?;
    sbfitting_from_desp(&desp, notes).map_err(ModelError::Solid)
}

/// SBFI's facet coordinates are catalogue coordinates. `ZDIR` is the catalogue
/// Z axis expressed in the design element frame; E3D completes the right-handed
/// frame from the design Z-up direction. For 17496/152095 this maps catalogue
/// X/Y/Z to design +Z/-Y/+X, exactly the transform stored in the RVM group.
fn sbfitting_catalogue_alignment(el: &DbElement) -> Result<DMat4, ModelError> {
    let raw = el.get_direction("ZDIR")?.unwrap_or([0.0, 0.0, 1.0]);
    // Descriptor direction triples use E3D's bearing handedness; the second
    // component is opposite to the right-handed design-frame Y used by glam.
    let z = DVec3::new(raw[0], -raw[1], raw[2]).normalize_or_zero();
    if z.length_squared() <= f64::EPSILON {
        return Err(ModelError::Solid(SolidError::Degenerate {
            what: "SBFI catalogue alignment",
            detail: format!("ZDIR = {raw:?}"),
        }));
    }
    let reference = if z.dot(DVec3::Z).abs() < 0.999_999 {
        DVec3::Z
    } else {
        DVec3::X
    };
    let x = (reference - z * reference.dot(z)).normalize();
    let y = z.cross(x).normalize();
    Ok(DMat4::from_cols(
        x.extend(0.0),
        y.extend(0.0),
        z.extend(0.0),
        DVec3::ZERO.extend(1.0),
    ))
}

fn sbfitting_from_desp(desp: &[f64], notes: &mut Vec<String>) -> Result<Manifold, SolidError> {
    if desp.len() < SBFI_DESP_MIN {
        return Err(SolidError::Degenerate {
            what: SBFI_WHAT,
            detail: format!("DESP length {} < {SBFI_DESP_MIN}", desp.len()),
        });
    }
    let (inner_diam, outer_diam, wall, length) = (desp[0], desp[1], desp[2], desp[3]);
    if !(inner_diam > 0.0 && outer_diam > inner_diam && length > f64::EPSILON) {
        return Err(SolidError::Degenerate {
            what: SBFI_WHAT,
            detail: format!("ID/OD/LENGTH = {inner_diam}/{outer_diam}/{length}"),
        });
    }

    let radial_wall = (outer_diam - inner_diam) * 0.5;
    if wall > 0.0 && (radial_wall - wall).abs() > 0.051 {
        notes.push(format!(
            "SBFI DESP wall {wall} differs from (OD-ID)/2 = {radial_wall}; ID/OD take precedence"
        ));
    }

    // The reference RVM uses twelve circumferential panels for this catalogue
    // fitting.  Keeping that tessellation also makes topology comparisons
    // deterministic instead of merely matching the bounding box.
    const SEGMENTS: usize = 8;
    let ring = |radius: f64, reverse: bool| {
        let mut points = (0..SEGMENTS)
            .map(|i| {
                let angle = std::f64::consts::TAU * i as f64 / SEGMENTS as f64;
                [radius * angle.cos(), radius * angle.sin()]
            })
            .collect::<Vec<_>>();
        if reverse {
            points.reverse();
        }
        points
    };
    let solid = extrude_rings(
        vec![ring(outer_diam * 0.5, false), ring(inner_diam * 0.5, true)],
        length,
    )?;
    Ok(solid.translate(desp[7], desp[8], desp[9] - length))
}

enum LocalSolid {
    Solid(Manifold),
    ZeroHeight(f64),
}

/// 环拉伸的局部实体：剖面 → 折线化 → 挤出 `[0, HEIG]` → SJUS 下移。
///
/// `grow`：负体让刀量（正体传 0）——截面 miter offset + 高度两端各让，
/// 在局部系做，摆到世界系后依然每个面都让足（见 `solid` 模块文档）。
fn extrusion_local_solid(
    el: &DbElement,
    noun: &str,
    allow_sjus: bool,
    grow: f64,
    notes: &mut Vec<String>,
    cache: &mut CatalogueMeshCache,
    reuse: bool,
) -> Result<LocalSolid, ModelError> {
    let profile = loop_profile(el)?;
    if !(profile.height > f64::EPSILON) {
        return Ok(LocalSolid::ZeroHeight(profile.height));
    }
    let has_skipped_link = profile
        .rings
        .first()
        .is_some_and(|ring| has_core3d_skipped_link(ring));
    let main_component_only =
        grow == 0.0 && (noun == "GWALL" || (noun == "FLOOR" && has_skipped_link));
    let rings = flatten_profile_rings(el, noun, &profile, notes)?;
    // SJUS 下移留在池外逐元素应用；池键覆盖环组/高度/让刀量/变体全部构造输入。
    let variant = if main_component_only {
        ExtrusionVariant::LargestComponent
    } else {
        ExtrusionVariant::Inflated { grow }
    };
    let mut solid = cached_extrusion(cache, rings, profile.height, variant, reuse)?;
    if profile.sjus_drop != 0.0 {
        if allow_sjus && SJUS_NOUNS.contains(&noun) {
            solid = solid.translate(0.0, 0.0, -profile.sjus_drop);
        } else {
            notes.push(format!(
                "PLOO 带 SJUS（下移 {}mm）但 noun={noun} 不在生效名单，按 fast_model 口径未应用",
                profile.sjus_drop
            ));
        }
    }
    Ok(LocalSolid::Solid(solid))
}

/// 环组折线化：外环塌掉是硬错误，孔环塌掉记 note 后丢弃。
fn flatten_profile_rings(
    el: &DbElement,
    noun: &str,
    profile: &LoopProfile,
    notes: &mut Vec<String>,
) -> Result<Vec<Vec<[f64; 2]>>, ModelError> {
    let mut rings: Vec<Vec<[f64; 2]>> = Vec::with_capacity(profile.rings.len());
    for (index, ring) in profile.rings.iter().enumerate() {
        match flatten_ring(ring) {
            Some((poly, ring_notes)) => {
                if !ring_notes.overshoot_fillets.is_empty() {
                    notes.push(format!(
                        "环 {index} 倒圆切点越界（顶点 {:?}），按 E3D 近越界吸附口径处理",
                        ring_notes.overshoot_fillets
                    ));
                }
                if !ring_notes.degenerate_corners.is_empty() {
                    notes.push(format!(
                        "环 {index} 有退化角（顶点 {:?}），倒圆被忽略",
                        ring_notes.degenerate_corners
                    ));
                }
                rings.push(poly);
            }
            None if index == 0 => {
                return Err(ModelError::OuterRingCollapsed {
                    refno: el.refno().to_string(),
                    noun: noun.to_string(),
                });
            }
            None => {
                notes.push(format!("孔环 {index} 折线化后退化，丢弃"));
            }
        }
    }
    Ok(rings)
}

fn require_double(
    el: &DbElement,
    noun: &str,
    name: &'static str,
    what: &'static str,
) -> Result<f64, ModelError> {
    match el.get_double(name)? {
        Some(v) => Ok(v),
        None => Err(ModelError::MissingAttribute {
            refno: el.refno().to_string(),
            noun: noun.to_string(),
            name,
            what,
        }),
    }
}

/// 基本体局部实体（libgm `gm_Create*` 语义，原点居中）。
fn primitive_local_solid(
    el: &DbElement,
    noun: &str,
    prim: Prim,
    allow_instance: bool,
) -> Result<PrimitiveBuild, ModelError> {
    match prim {
        Prim::Box => {
            let x = require_double(el, noun, "XLEN", "box")?;
            let y = require_double(el, noun, "YLEN", "box")?;
            let z = require_double(el, noun, "ZLEN", "box")?;
            Ok(instantiate_box(x, y, z, allow_instance)?)
        }
        Prim::Cylinder => {
            let d = require_double(el, noun, "DIAM", "cylinder")?;
            let h = require_double(el, noun, "HEIG", "cylinder")?;
            if allow_instance {
                Ok(instantiate_cylinder(d, h, true)?)
            } else {
                Ok(PrimitiveBuild {
                    solid: csg_cylinder_solid(d, h)?,
                    instance: None,
                })
            }
        }
        Prim::Pyramid => {
            let xbot = require_double(el, noun, "XBOT", "pyramid")?;
            let ybot = require_double(el, noun, "YBOT", "pyramid")?;
            let h = require_double(el, noun, "HEIG", "pyramid")?;
            Ok(PrimitiveBuild {
                solid: pyramid_solid(
                    xbot,
                    ybot,
                    default_zero(el, "XTOP")?,
                    default_zero(el, "YTOP")?,
                    h,
                    default_zero(el, "XOFF")?,
                    default_zero(el, "YOFF")?,
                )?,
                instance: None,
            })
        }
        Prim::RectTorus => {
            let rout = require_double(el, noun, "ROUT", "rectangular torus")?;
            let h = require_double(el, noun, "HEIG", "rectangular torus")?;
            let angle = require_double(el, noun, "ANGL", "rectangular torus")?;
            Ok(PrimitiveBuild {
                solid: rect_torus_solid(default_zero(el, "RINS")?, rout, h, angle)?,
                instance: None,
            })
        }
        Prim::CircTorus => {
            let rout = require_double(el, noun, "ROUT", "circular torus")?;
            let angle = require_double(el, noun, "ANGL", "circular torus")?;
            Ok(PrimitiveBuild {
                solid: circ_torus_solid(default_zero(el, "RINS")?, rout, angle)?,
                instance: None,
            })
        }
        Prim::Snout => Ok(PrimitiveBuild {
            solid: snout_solid(
                require_double(el, noun, "DBOT", "snout")?,
                require_double(el, noun, "DTOP", "snout")?,
                require_double(el, noun, "HEIG", "snout")?,
                default_zero(el, "XOFF")?,
                default_zero(el, "YOFF")?,
            )?,
            instance: None,
        }),
    }
}

/// 多面体局部实体（`CSG_BasicPOL::getPrimGeom` @0x10726f10 → `sub_1071DCF0`，
/// 7UW4 §3.7.4）：面片采集（两变体） → 带孔三角化 → 绕向一致化 → 流形校验。
/// 退化面记 `notes`；壳不闭合按数据坏上抛（= `gm_ValidateObject` 不过即弃的口径）。
fn polyhedron_local_solid(el: &DbElement, notes: &mut Vec<String>) -> Result<Manifold, ModelError> {
    let faces = polyhedron_faces(el)?;
    Ok(polyhedron_solid(&faces, notes)?)
}

/// 回转体局部实体（`gm_CreateRevolution`）：环剖面绕局部 X 轴转 ANGL 度。
///
/// 段数走的是**挤出**那条口径的折线化（[`flatten_ring`]），而 libgm 的回转轮廓
/// 另有一套 `GM_Profile::setNSteps`（与配对 span 取大、整条超 1000 点放大容差重算）。
/// gen-model 也还没有那一支（它的 `tessellate_revolution` 同样借用挤出口径），
/// 两仓在这一点上齐平——RVM 对拍要是在回转体上出段数级偏差，先补的是这一条。
fn revolution_local_solid(
    el: &DbElement,
    noun: &str,
    grow: f64,
    notes: &mut Vec<String>,
) -> Result<Manifold, ModelError> {
    let angle = require_double(el, noun, "ANGL", "revolution")?;
    let profile = loop_profile(el)?;
    let rings = flatten_profile_rings(el, noun, &profile, notes)?;
    // 「≈360 / >360 / ==0 一律当整圈」——aios-core `tessellate_revolution` 的归一。
    let angle = if angle == 0.0 || angle.abs() >= 359.99 {
        360.0
    } else {
        angle
    };
    Ok(revolve_rings_inflated(rings, angle, grow)?)
}

/// PDMS 缺省语义：属性未设（省存）或该 noun 模板里没有 → 取 0。
///
/// RINS / XTOP / XOFF 这几个 0 是有意义的取值（贴轴环面、退化成锥的棱台），
/// 缺省被省存后读回 `None`，不能当成「缺属性」硬报错。
fn default_zero(el: &DbElement, name: &str) -> Result<f64, DbElementError> {
    match el.get_double(name) {
        Ok(v) => Ok(v.unwrap_or(0.0)),
        Err(DbElementError::UnknownAttribute { .. }) => Ok(0.0),
        Err(e) => Err(e),
    }
}

/// 收集要扣的负几何：直属负体成员 + 直属 CMPF 成员**深层**里的负体
/// （gen-model `loop_model` 的同款圈定）。
///
/// # 圈定只向下，不向兄弟
///
/// [`Category::PrimitiveContainer`]（`TMPL`）底下负体与正体是并排挂着的，看起来
/// 像是该由正体兄弟扣掉。**实测证否**：`aps250110` 上照这条规则跑，5 个
/// `TMPL "Fire Hose Wheel and Housing"` 的 31 件零件**全部被扣空**——它那件
/// `NXTR` 是 1000×450×700 的**墙上凹龛**，而水带箱正是装在凹龛里的，拿凹龛扣箱子
/// 当然一件不剩；`TMPL "Flexible End Plate - 150mmx8mm"` 的端板则只剩两条 8.4mm
/// 窄条，因为那件 `NBOX` 的 133.2×203.2 是 203×133 工字梁的截面、给梁让位用的。
///
/// 也就是说这些负体扣的是**容器之外的宿主**（墙 / 梁 / 被连接的构件）。E3D 的这条
/// 「构件在别处、洞开在宿主上」的路我们整条都还没有；在找到宿主的判据之前，
/// 它们由管线记进 `Report::host_negatives` 挂账，不参与任何扣减。
fn collect_negatives(el: &DbElement, out: &mut Vec<DbElement>) -> Result<(), DbElementError> {
    for member in el.members()? {
        let noun = member.element_type()?;
        let cat = classify(&noun);
        if cat.is_negative() {
            out.push(member);
        } else if noun == "CMPF" {
            collect_negatives_deep(&member, out)?;
        }
    }
    Ok(())
}

fn collect_negatives_deep(el: &DbElement, out: &mut Vec<DbElement>) -> Result<(), DbElementError> {
    for member in el.members()? {
        let noun = member.element_type()?;
        if classify(&noun).is_negative() {
            out.push(member);
        } else {
            collect_negatives_deep(&member, out)?;
        }
    }
    Ok(())
}

/// 单个负体的建模结论。
enum NegativeSolid {
    Solid(Manifold),
    /// 有依据地不出，理由要具体到能归因——属主的体积会因此偏大，
    /// RVM 对拍时这句话就是判据。
    Skipped(String),
}

/// 负体建体 + 让刀（都在负体自己的局部系）+ 世界放置。
///
/// 让刀口径：挤出/回转 = 截面 miter offset（斜边也精确外移
/// [`MANIFOLD_NUDGE_MM`]）；
/// 基本体 = 局部系逐轴放大——box/cylinder 的面在局部系轴对齐，精确；
/// pyramid 斜面 / torus 径向面的外移量略小于 restol，出现共壁案例再按面法向补。
fn negative_world_solid(
    neg: &DbElement,
    snout_rvm_facets: bool,
    cache: &mut CatalogueMeshCache,
) -> Result<NegativeSolid, ModelError> {
    let noun = neg.element_type()?;
    let mut notes = Vec::new();
    let local = match classify(&noun) {
        Category::NegExtrusion => {
            // 负体是布尔运算数：按边界烘焙直建（cached_extrusion 只记账不入池）。
            match extrusion_local_solid(
                neg,
                &noun,
                false,
                MANIFOLD_NUDGE_MM,
                &mut notes,
                cache,
                false,
            )? {
                LocalSolid::Solid(s) => s,
                LocalSolid::ZeroHeight(h) => {
                    return Ok(NegativeSolid::Skipped(format!(
                        "{noun} HEIG={h}（≈0，E3D 同样不出几何）"
                    )));
                }
            }
        }
        Category::NegPrimitive(Prim::Cylinder) if snout_rvm_facets => inflate_about_center(
            &csg_cylinder_solid_with_segments(
                require_double(neg, &noun, "DIAM", "cylinder")?,
                require_double(neg, &noun, "HEIG", "cylinder")?,
                snout_segments(0.0, require_double(neg, &noun, "DIAM", "cylinder")? / 2.0),
            )?,
            MANIFOLD_NUDGE_MM,
        ),
        Category::NegPrimitive(prim) => inflate_about_center(
            &primitive_local_solid(neg, &noun, prim, false)?.solid,
            MANIFOLD_NUDGE_MM,
        ),
        Category::NegRevolution => {
            revolution_local_solid(neg, &noun, MANIFOLD_NUDGE_MM, &mut notes)?
        }
        Category::NegPolyhedron => {
            // 任意朝向面片的让刀走 AABB 逐轴放大——与 pyramid/torus 同一精度口径
            // （斜面外移量略小于 restol，出现共壁案例再按面法向补，见模块文档）。
            inflate_about_center(&polyhedron_local_solid(neg, &mut notes)?, MANIFOLD_NUDGE_MM)
        }
        Category::NegUnimplemented(why) => {
            return Ok(NegativeSolid::Skipped(format!("{noun}: {why}")));
        }
        // 几何集负体（INGCOM）：是负几何，但几何集整条路本期不建，所以只能跳过并
        // 记在属主头上。少了这一支它会掉进下面的 unreachable! —— 一个 NSBO 挂在
        // PANE 底下就能让整轮生成 panic。
        Category::NegGeometrySet => {
            return Ok(NegativeSolid::Skipped(format!(
                "{noun}: 几何集负体（INGCOM），本期不建几何集"
            )));
        }
        other => {
            // collect_negatives 只按 is_negative() 收，走到这里是分类表自相矛盾。
            unreachable!("负体收集给了非负类别 {other:?}（noun={noun}）");
        }
    };
    let world = world_matrix(neg)?;
    Ok(NegativeSolid::Solid(placed(&local, world)))
}

/// 世界矩阵 × 局部 SJUS 下移的复合（暴露给需要单独拿变换的调用方/测试）。
pub fn world_with_sjus(world: DMat4, sjus_drop: f64) -> DMat4 {
    world * DMat4::from_translation(DVec3::new(0.0, 0.0, -sjus_drop))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn three_spine_points_recover_their_circle() {
        let center = DVec3::new(120.0, -45.0, 0.0);
        let radius = 3200.0;
        let point =
            |angle: f64| center + DVec3::new(radius * angle.cos(), radius * angle.sin(), 0.0);
        let (actual_center, actual_radius) =
            circle_through_xy(point(-0.8), point(0.15), point(1.1)).expect("valid arc");
        assert!((actual_center - center).length() < 1e-9);
        assert!((actual_radius - radius).abs() < 1e-9);
    }

    #[test]
    fn collinear_wall_spine_is_rejected() {
        let error = circle_through_xy(DVec3::ZERO, DVec3::X, DVec3::X * 2.0)
            .expect_err("a straight line has no finite circle");
        assert!(matches!(
            error,
            ModelError::Solid(SolidError::Degenerate {
                what: "WALL circular spine",
                ..
            })
        ));
    }

    #[test]
    fn only_dimensionally_empty_primitives_are_no_shape_results() {
        let bad_height = ModelError::Solid(SolidError::BadHeight {
            what: "pyramid",
            height: 0.0,
        });
        let degenerate = ModelError::Solid(SolidError::Degenerate {
            what: "circular torus",
            detail: "RINS/ROUT = 0/0".into(),
        });
        let empty_manifold = ModelError::Solid(SolidError::EmptyManifold { what: "pyramid" });
        assert!(primitive_no_shape_reason(&bad_height).is_some());
        assert!(primitive_no_shape_reason(&degenerate).is_some());
        assert!(primitive_no_shape_reason(&empty_manifold).is_none());
    }

    #[test]
    fn only_absent_or_zero_point_profiles_are_no_shape_results() {
        let no_loops = ModelError::Profile(ProfileError::NoLoops {
            refno: "1/2".into(),
            noun: "EXTR".into(),
        });
        let zero = ModelError::Profile(ProfileError::DegenerateRing {
            refno: "1/3".into(),
            ring_index: 0,
            points: 0,
        });
        let two = ModelError::Profile(ProfileError::DegenerateRing {
            refno: "1/4".into(),
            ring_index: 0,
            points: 2,
        });
        assert!(profile_no_shape_reason(&no_loops).is_some());
        assert!(profile_no_shape_reason(&zero).is_some());
        assert!(profile_no_shape_reason(&two).is_none());
    }

    /// 短 `DESP` 的 SBFI 必须落 `Skipped`，不能落 `failed`。
    ///
    /// 这一条钉的是全量与增量的一处账目分歧：这道门原先只写在 `pipeline.rs` 的 match
    /// 守卫里，全量当没这件事，增量（直接调 [`model_element`]）却记成 `regen_failed`
    /// 并发 Remove。两边的动作集恰好一致，所以 `accounts_for` 那类等式判据一个都不响，
    /// 只有把两份报告并排看才发现得了。门收回产出侧之后，这个测试守住它别再跑出去。
    #[test]
    fn a_short_sbfi_desp_is_no_shape_not_a_failure() {
        let mut notes = Vec::new();
        let error = ModelError::Solid(
            sbfitting_from_desp(&[100.0, 124.0, 12.0], &mut notes).expect_err("DESP 只有 3 个分量"),
        );
        let reason = sbfitting_no_shape_reason(&error).expect("短 DESP 是「不是套管」，不是坏数据");
        assert!(
            reason.contains("DESP length 3 < 10"),
            "理由要带上实际长度：{reason}"
        );

        // 尺寸校验不过（OD 不大于 ID）同样是 E3D 不出几何的形状，与正体那条
        // `primitive_no_shape_reason` 同款，不该记成失败。
        let bad_dims = ModelError::Solid(
            sbfitting_from_desp(
                &[
                    124.0, 100.0, 12.0, 1000.0, 0.0, 0.0, 1400.0, 3000.0, 0.0, 0.0,
                ],
                &mut notes,
            )
            .expect_err("OD 必须大于 ID"),
        );
        assert!(sbfitting_no_shape_reason(&bad_dims).is_some());

        // 反面：套管对齐算不出来是真出了问题（ZDIR 退化），不许被顺手咽掉。
        let alignment = ModelError::Solid(SolidError::Degenerate {
            what: "SBFI catalogue alignment",
            detail: "ZDIR = [0.0, 0.0, 0.0]".into(),
        });
        assert!(
            sbfitting_no_shape_reason(&alignment).is_none(),
            "对齐退化是坏数据，必须留在 failed 里"
        );
    }

    #[test]
    fn sbfitting_desp_builds_a_through_bore_not_solid_cylinders() {
        let mut notes = Vec::new();
        let solid = sbfitting_from_desp(
            &[
                100.0, 124.0, 12.0, 1000.0, 0.0, 0.0, 1400.0, 3000.0, 0.0, 0.0,
            ],
            &mut notes,
        )
        .expect("valid SBFI DESP");
        let bounds = solid.bounding_box().expect("SBFI bounds");
        assert_eq!(solid.genus(), 1, "SBFI must retain its through bore");
        assert_eq!(solid.num_tri(), 64, "octagonal annular extrusion");
        assert_eq!(bounds.min(), [2938.0, -62.0, -1000.0]);
        assert!((bounds.max()[0] - 3062.0).abs() < 1e-9);
        assert!((bounds.max()[1] - 62.0).abs() < 1e-9);
        assert!(bounds.max()[2].abs() < 1e-9);
        assert!(notes.is_empty());
    }
}

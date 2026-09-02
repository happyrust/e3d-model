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
//! - **本期不做**（NDIS/NSLC 一族曲面基本体）→ 同样 `Skipped`，reason 说清是
//!   里程碑边界。
//! - 负几何单个失败**不拖垮正体**：记进 [`GeneratedElement::negatives_skipped`]，
//!   正体照出——RVM 对拍时这类元素会以体积偏大暴露，报告里有对应记录。

use glam::{DMat4, DVec3};
use manifold_csg::Manifold;

use e3d_io::db_element::{DbElement, DbElementError};
use e3d_io::record::descriptor::DescriptorValue;
use e3d_io::refno::RefNo;

use crate::catalogue::{CatalogueMeshCache, model_catalogue_member_with_cache, repre_level_of};
use crate::category::{Category, CompositeFamily, Prim, classify};
use crate::discretise::{flatten_ring, has_core3d_skipped_link};
use crate::extrusion_reuse::{ExtrusionVariant, cached_extrusion};
use crate::polyhedron::{PolyhedronError, polyhedron_faces, polyhedron_solid};
use crate::primitive_instance::{
    PrimitiveBuild, PrimitiveInstance, instantiate_box, instantiate_cylinder,
};
use crate::profile::{LoopProfile, ProfileError, loop_profile};
use crate::solid::{
    MANIFOLD_NUDGE_MM, SolidError, circ_torus_solid, cleanup_floor_boolean_components,
    csg_cylinder_solid_with_segments, extrude_rings, inflate_about_center,
    negative_cylinder_segments, negative_part_rev_segments, placed, pyramid_solid,
    rect_torus_solid, rect_torus_solid_with_segments, revolve_rings_inflated, snout_segments,
    snout_solid, subtract_negatives,
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
    /// 运行时派生管身：容器、两端站点、及同站点对内的出现序共同确定。
    /// 两端使普通插入的差分保持局部；出现序只在同一对 `(from, to)` 重复出现
    /// （成员被重复挂进成员表、路由折返）时消歧，否则两条真实槽位会碰成同一个
    /// GeometryId。
    ImpliedTube {
        container_refno: String,
        /// 槽位在**同 `(from, to)` 站点对内**的出现序（[`crate::route::slot_ordinals`]，
        /// 0 基链序）。它不是槽位下标——下标在链头插一件就整链平移，键的局部性
        /// 就没了（`route.rs` 的 `inserting_*` 测试钉着这条）。旧持久化 JSON 没有
        /// 该字段，反序列化时回落到 `u32::MAX`；首次 v2 发布会以完整根 manifest
        /// 替换旧槽位。
        #[serde(default = "legacy_route_ordinal")]
        route_ordinal: u32,
        from_refno: String,
        to_refno: String,
    },
}

const fn legacy_route_ordinal() -> u32 {
    u32::MAX
}

impl std::fmt::Display for GeometryId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            GeometryId::Element { refno } => write!(f, "element:{refno}"),
            GeometryId::ImpliedTube {
                container_refno,
                route_ordinal,
                from_refno,
                to_refno,
            } => write!(
                f,
                "implied-tube:{container_refno}:{route_ordinal}:{from_refno}:{to_refno}"
            ),
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
    /// RVM-compatible, non-boolean component list.  Present only when E3D
    /// preserves touching catalogue primitives as separate display meshes.
    pub parts: Option<Vec<Manifold>>,
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

impl GeneratedElement {
    pub fn mesh(&self) -> crate::mesh::TriMesh {
        self.parts.as_deref().map_or_else(
            || crate::mesh::TriMesh::from_manifold(&self.solid),
            crate::mesh::TriMesh::from_manifolds,
        )
    }

    pub fn components(&self) -> Vec<Manifold> {
        self.parts.clone().unwrap_or_else(|| self.solid.decompose())
    }

    pub fn volume(&self) -> f64 {
        self.parts.as_ref().map_or_else(
            || self.solid.volume(),
            |parts| parts.iter().map(Manifold::volume).sum(),
        )
    }

    pub fn genus(&self) -> i32 {
        self.parts.as_ref().map_or_else(
            || self.solid.genus(),
            |parts| parts.iter().map(Manifold::genus).sum(),
        )
    }
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
                    "目录侧无可见几何，E3D 同样不出几何：{}（局部失败 {} 条）",
                    build
                        .no_shape
                        .unwrap_or_else(|| "无 SPRE/GMRE 或成员全退化".to_string()),
                    build.diagnostics.len()
                ),
            },
        });
    }
    let noun = el.element_type()?;
    // 电缆桥架托盘：与管线 `Composite(CableTray) if noun == "CTRAY"` 臂同一份目录求值。
    // 判据在 `category::is_model_unit`（CTRAY 是单元、桥架件不是），这里只是它的执行臂
    // ——少了它，增量上卷到 CTRAY 之后就掉进下面的设计侧正体路，对着一个没有
    // XLEN/DIAM 的复合件读尺寸，报出来的是「属性缺失」而不是「托盘变了」。
    if matches!(category, Category::Composite(CompositeFamily::CableTray)) {
        if !noun.eq_ignore_ascii_case("CTRAY") {
            return Ok(Modeled::Skipped {
                reason: format!("{noun} 是桥架件，只算放置，几何在它下面的 CTRAY 上"),
            });
        }
        let build = model_catalogue_member_with_cache(el, cache)?;
        return Ok(match build.generated {
            Some(generated) => Modeled::Solid(Box::new(generated)),
            None => Modeled::Skipped {
                reason: format!(
                    "目录链（SPRE→…→SCOM→GMRE）没有产出可见几何：{}（局部失败 {} 条）",
                    build.no_shape.unwrap_or_else(|| "原因未记".to_string()),
                    build.diagnostics.len()
                ),
            },
        });
    }
    let mut notes = Vec::new();
    // Decide the instancing boundary before constructing the positive body.
    // Any boolean operand stays at real dimensions: unit scaling can differ by
    // a few ulps and coincident-face CSG may amplify that into another topology.
    let mut negatives = Vec::new();
    collect_negatives(el, &mut negatives)?;
    // DRWAND 的表现级门先过（Core3D `csg_gml4/DRWAND` 0x1008BEE0：对负体自己跑
    // `rules/STYSET` 后判 `LEVE.lo <= level <= LEVE.hi`）。被门挡下的负体不是失败，
    // 是 E3D 同样不切的刀——记进 negatives_skipped 带上区间与当前级，RVM 对拍时
    // 体积差就从这一行归因（evidence/20260902-drwant-stylset-level-rule §4–5）。
    // 门放在实例化决策之前：全部刀都被挡下的正体没有布尔参与者，照旧可以实例化。
    // 级按宿主的学科取（MODCMP 交给 ELMODL 的那一个数：设计基本体 / 负体走通用档
    // `REPRESENTATION LEVEL n`，NOZZ 走 `LEVEL NOZZ`），与目录成员的显示过滤同一张表。
    let level = repre_level_of(el)?;
    let mut skipped = Vec::new();
    negatives.retain(|neg| {
        let (lo, hi) = negative_leve(neg);
        if design_leve_admits(lo, hi, level) {
            return true;
        }
        skipped.push(SkippedNegative {
            refno: neg.refno(),
            noun: neg.element_type().unwrap_or_else(|_| "?".into()),
            reason: format!("LEVE=[{lo},{hi}] 不含表现级 {level}（DRWAND 门，E3D 同样不切）"),
        });
        false
    });
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
    // 刀的线性尺寸以宿主尺度封顶（E3D 拿 3.2e14 mm 当「无限长」哨兵，Manifold 的
    // epsilon 跟运算数包围盒走，一把哨兵刀会让整组布尔在 320 mm 精度下失稳）。
    let extent_cap = negative_extent_cap(&pos_world);
    let mut applied = Vec::new();
    for neg in negatives {
        let reason = match negative_world_solid(&neg, noun == "SNOU", cache, extent_cap, &mut notes)
        {
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
        parts: None,
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
    if path.len() != 3 && path.len() != 5 {
        return Err(ModelError::UnresolvedAttribute {
            refno: el.refno().to_string(),
            noun: "WALL".into(),
            name: "SPINE",
            what: "circular WALL path",
            detail: format!("member count {} is neither 3 nor 5", path.len()),
        });
    }
    let nouns = path
        .iter()
        .map(DbElement::element_type)
        .collect::<Result<Vec<_>, _>>()?;
    let supported_nouns = nouns == ["POINSP", "CURVE", "POINSP"]
        || nouns == ["POINSP", "CURVE", "POINSP", "CURVE", "POINSP"];
    if !supported_nouns {
        return Err(ModelError::UnresolvedAttribute {
            refno: el.refno().to_string(),
            noun: "WALL".into(),
            name: "SPINE",
            what: "circular WALL path",
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
    let points = path.iter().map(point).collect::<Result<Vec<_>, _>>()?;
    if points.iter().map(|point| point.z.abs()).fold(0.0, f64::max) > 0.051 {
        return Err(ModelError::UnresolvedAttribute {
            refno: spine.refno().to_string(),
            noun: "SPINE".into(),
            name: "POS",
            what: "planar WALL spine",
            detail: format!(
                "non-planar z = {:?}",
                points.iter().map(|point| point.z).collect::<Vec<_>>()
            ),
        });
    }
    let (center, radius, a0, sweep) = circular_spine_geometry(&points)?;
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
        * DMat4::from_rotation_z(a0);
    notes.push(format!(
        "WALL circular SPINE center=({:.6},{:.6}) radius={radius:.6} sweep={:.6}deg",
        center.x,
        center.y,
        sweep.to_degrees()
    ));
    Ok(placed(&torus, local))
}

fn circular_spine_geometry(points: &[DVec3]) -> Result<(DVec3, f64, f64, f64), ModelError> {
    let (p0, pm, p1) = (points[0], points[1], points[2]);
    let (center, radius) = circle_through_xy(p0, pm, p1)?;
    let a0 = (p0.y - center.y).atan2(p0.x - center.x);
    if points.len() == 5 {
        let closure_error = (points[4] - p0).length();
        let radial_error = (points[3] - center).length() - radius;
        if closure_error > 0.051 || radial_error.abs() > 0.051 {
            return Err(ModelError::Solid(SolidError::Degenerate {
                what: "WALL full-circle spine",
                detail: format!(
                    "closure/radial error = {closure_error}/{}",
                    radial_error.abs()
                ),
            }));
        }
        let r0 = p0 - center;
        let rq = pm - center;
        let cross_z = r0.x * rq.y - r0.y * rq.x;
        if cross_z.abs() <= 1e-9 {
            return Err(ModelError::Solid(SolidError::Degenerate {
                what: "WALL full-circle spine",
                detail: "first CURVE point does not establish direction".into(),
            }));
        }
        return Ok((center, radius, a0, cross_z.signum() * std::f64::consts::TAU));
    }
    let am = (pm.y - center.y).atan2(pm.x - center.x);
    let a1 = (p1.y - center.y).atan2(p1.x - center.x);
    let ccw_mid = positive_angle(am - a0);
    let ccw_end = positive_angle(a1 - a0);
    let sweep = if ccw_mid <= ccw_end + 1e-9 {
        ccw_end
    } else {
        -(std::f64::consts::TAU - ccw_end)
    };
    Ok((center, radius, a0, sweep))
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
    if path.len() != 3 && path.len() != 5 {
        return Err(ModelError::UnresolvedAttribute {
            refno: spine.refno().to_string(),
            noun: "SPINE".into(),
            name: "MEMBERS",
            what: "WALL host frame",
            detail: format!("member count {} is neither 3 nor 5", path.len()),
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
                if !ring_notes.consumed_fillets.is_empty() {
                    notes.push(format!(
                        "环 {index} 倒圆两侧 line span 均不超过 0.1mm（顶点 {:?}），保留理论切点圆弧",
                        ring_notes.consumed_fillets
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
                    // E3D emits the final LNEGIT result as a coarse FacetGroup.
                    // The positive CYLI must use the same circle grid as its
                    // negative operands; otherwise the hole is topologically
                    // right but the outer wall remains a different polygon.
                    solid: csg_cylinder_solid_with_segments(
                        d,
                        h,
                        negative_cylinder_segments(d * 0.5),
                    )?,
                    instance: None,
                })
            }
        }
        Prim::Cone => Ok(PrimitiveBuild {
            solid: snout_solid(
                require_double(el, noun, "DBOT", "cone")?,
                require_double(el, noun, "DTOP", "cone")?,
                require_double(el, noun, "HEIG", "cone")?,
                0.0,
                0.0,
            )?,
            instance: None,
        }),
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
            let rins = default_zero(el, "RINS")?;
            let rout = require_double(el, noun, "ROUT", "rectangular torus")?;
            let h = require_double(el, noun, "HEIG", "rectangular torus")?;
            let angle = require_double(el, noun, "ANGL", "rectangular torus")?;
            Ok(PrimitiveBuild {
                solid: rect_torus_solid(rins, rout, h, angle)?,
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

/// 负体线性尺寸**被判为哨兵**的门槛。E3D 拿 3.2037e14 mm 当「无限长」（FLOOR 72959 的两把
/// NCYL）；Manifold 的 epsilon = 1e-12 × 包围盒尺度，到 1e9 mm 才涨到 1e-3 mm——仍在
/// [`crate::solid::RES_TOL_MM`] 之下，而工厂模型里没有任何合法尺寸接近它。
///
/// **触发门槛与钳到的值是两回事。** 2026-09-02 WF03 复跑抓到的教训：一开始拿宿主尺度
/// （8×对角线）同时当触发门槛，GWALL `17496/116569` 那把 92 m 长的 NXTR 修边刀被当成哨兵
/// 整刀跳过，墙末端多出 147.5 mm、体积 2.79e9 → 6.26e9（07:56 基线 exact → off）。
/// 长出宿主几倍甚至几十倍的刀是合法的（整层楼的修边刀、跨房间的开槛刀），只有量级离谱的才是哨兵。
const NEGATIVE_EXTENT_SENTINEL_MM: f64 = 1.0e9;

/// 哨兵尺寸钳到的目标 = 宿主正体世界包围盒对角线 × 本因子，且不低于
/// [`NEGATIVE_EXTENT_CAP_FLOOR_MM`]。
///
/// 取值依据 `evidence/20260902-floor-72959-giant-ncyl` §3：把哨兵高度钳到 2e4–1e11 mm
/// 之间任何一个值，FLOOR 72959 的体积、分量数、genus 都稳定一致；因子 8 保证钳过的刀
/// 仍贯穿宿主并留有余量。
const NEGATIVE_EXTENT_CAP_FACTOR: f64 = 8.0;
const NEGATIVE_EXTENT_CAP_FLOOR_MM: f64 = 1.0e4;

/// 这个线性尺寸是不是 E3D 的「无限长」哨兵（或干脆不是有限数）。
fn is_sentinel_extent(value: f64) -> bool {
    !value.is_finite() || value.abs() >= NEGATIVE_EXTENT_SENTINEL_MM
}

fn negative_extent_cap(host_world: &Manifold) -> f64 {
    let Some(bb) = host_world.bounding_box() else {
        return NEGATIVE_EXTENT_CAP_FLOOR_MM;
    };
    let (lo, hi) = (bb.min(), bb.max());
    let diagonal = DVec3::new(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]).length();
    if diagonal.is_finite() {
        (diagonal * NEGATIVE_EXTENT_CAP_FACTOR).max(NEGATIVE_EXTENT_CAP_FLOOR_MM)
    } else {
        NEGATIVE_EXTENT_CAP_FLOOR_MM
    }
}

/// 负体的一个线性参数（HEIG 一类）是哨兵尺寸（[`is_sentinel_extent`]）时钳到宿主尺度
/// 上限 `cap`，符号照旧，并把这一刀记进宿主的 notes——基本体以自身原点为中心，钳高度
/// 不改变它贯穿宿主这件事。低于门槛的一律原样放行，哪怕它比宿主长几十倍。
fn clamp_negative_extent(
    neg: &DbElement,
    noun: &str,
    attr: &str,
    value: f64,
    cap: f64,
    host_notes: &mut Vec<String>,
) -> f64 {
    if !is_sentinel_extent(value) {
        return value;
    }
    host_notes.push(format!(
        "负体 {} {noun} {attr}={value:.3e} 是 E3D 的「无限长」哨兵（≥ {NEGATIVE_EXTENT_SENTINEL_MM:.0e} mm），\
         已钳到宿主尺度上限 {cap:.0} mm（不钳会让 Manifold epsilon 涨到宿主尺度的量级、整组布尔失稳）",
        neg.refno()
    ));
    if value.is_finite() {
        cap.copysign(value)
    } else {
        cap
    }
}

/// 设计侧负体的 `LEVE=[lo,hi]`；未设 / 缺 / 形状不对按 `gml_bkend/CRDESI` 对
/// `DGTERR == 18` 的处置给 `[0,10]`。负值按 0 读（level 是无符号的表现级）。
fn negative_leve(neg: &DbElement) -> (u32, u32) {
    let pair = match neg.attribute("LEVE") {
        Ok(Some(DescriptorValue::IntArray(v))) if v.len() >= 2 => {
            Some((i64::from(v[0]), i64::from(v[1])))
        }
        Ok(Some(DescriptorValue::RawWords(v) | DescriptorValue::WordArray(v))) if v.len() >= 2 => {
            Some((i64::from(v[0]), i64::from(v[1])))
        }
        _ => None,
    };
    let as_level = |value: i64| value.clamp(0, i64::from(u32::MAX)) as u32;
    pair.map_or((0, 10), |(lo, hi)| (as_level(lo), as_level(hi)))
}

/// `csg_gml4/DRWAND`（0x1008BEE0）对负体的门：闭区间 `lo <= level <= hi`，逐字照抄。
///
/// 与目录成员那条 [`crate::catalogue`] 的筛有意不同：那边 `hi == 0` 读作「不设上限」是
/// 对 addin 导出（`C-IY-1R330-B.rvm`）拟合出来的经验口径；设计侧负体只有 DRWAND 一条
/// 反编译在案的规则，语料里也没有 `[x,0]` 的刀，不先验地把那条例外带过来。
pub(crate) fn design_leve_admits(lo: u32, hi: u32, level: u32) -> bool {
    lo <= level && level <= hi
}

/// 负体建体 + 让刀（都在负体自己的局部系）+ 世界放置。
///
/// 让刀口径：挤出/回转 = 截面 miter offset（斜边也精确外移
/// [`MANIFOLD_NUDGE_MM`]）；
/// 基本体 = 局部系逐轴放大——box/cylinder 的面在局部系轴对齐，精确；
/// pyramid 斜面 / torus 径向面的外移量略小于 restol，出现共壁案例再按面法向补。
///
/// `extent_cap`：宿主尺度上限（[`negative_extent_cap`]）。NCYL 的 HEIG 在建体前钳到它；
/// 其余形状建完后若任一局部尺寸仍超上限，整刀跳过并记账（语料里只有 NCYL 带哨兵）。
fn negative_world_solid(
    neg: &DbElement,
    snout_rvm_facets: bool,
    cache: &mut CatalogueMeshCache,
    extent_cap: f64,
    host_notes: &mut Vec<String>,
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
        Category::NegPrimitive(Prim::Cylinder) if snout_rvm_facets => {
            let diameter = require_double(neg, &noun, "DIAM", "cylinder")?;
            let height = clamp_negative_extent(
                neg,
                &noun,
                "HEIG",
                require_double(neg, &noun, "HEIG", "cylinder")?,
                extent_cap,
                host_notes,
            );
            inflate_about_center(
                &csg_cylinder_solid_with_segments(
                    diameter,
                    height,
                    snout_segments(0.0, diameter / 2.0),
                )?,
                MANIFOLD_NUDGE_MM,
            )
        }
        Category::NegPrimitive(Prim::Cylinder) => {
            let diameter = require_double(neg, &noun, "DIAM", "cylinder")?;
            let height = clamp_negative_extent(
                neg,
                &noun,
                "HEIG",
                require_double(neg, &noun, "HEIG", "cylinder")?,
                extent_cap,
                host_notes,
            );
            let cutter = csg_cylinder_solid_with_segments(
                diameter,
                height,
                negative_cylinder_segments(diameter * 0.5),
            )?;
            inflate_about_center(&cutter, MANIFOLD_NUDGE_MM)
        }
        Category::NegPrimitive(Prim::RectTorus) => {
            let rins = default_zero(neg, "RINS")?;
            let rout = require_double(neg, &noun, "ROUT", "rectangular torus")?;
            let height = require_double(neg, &noun, "HEIG", "rectangular torus")?;
            let angle = require_double(neg, &noun, "ANGL", "rectangular torus")?;
            let cutter = rect_torus_solid_with_segments(
                rins,
                rout,
                height,
                angle,
                negative_part_rev_segments(rout, angle),
            )?;
            inflate_about_center(&cutter, MANIFOLD_NUDGE_MM)
        }
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
    // 钳法只接了 NCYL 的 HEIG；别的形状若仍带哨兵尺寸（[`is_sentinel_extent`]，不是
    // 「比宿主大」——92 m 的 NXTR 修边刀是合法的），宁可少一刀并记账，也不把它送进
    // 布尔去毒化整组。
    if let Some(bb) = local.bounding_box() {
        let (lo, hi) = (bb.min(), bb.max());
        let extent = (0..3).fold(0.0_f64, |acc, axis| acc.max(hi[axis] - lo[axis]));
        if is_sentinel_extent(extent) {
            return Ok(NegativeSolid::Skipped(format!(
                "{noun} 局部尺寸 {extent:.3e} mm 是 E3D 的「无限长」哨兵（≥ {NEGATIVE_EXTENT_SENTINEL_MM:.0e} mm，\
                 宿主尺度上限 {extent_cap:.0} mm）；只有 NCYL 接了钳法，其余先跳过并记账，避免整组布尔失稳"
            )));
        }
    }
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
    fn the_design_negative_level_gate_is_drwand_closed_interval() {
        // FLOOR 17496/72959 的两类刀：[0,10] 在 0 级过、[2,10] 在 0 级被拒（RVM 逐把吻合）。
        assert!(design_leve_admits(0, 10, 0));
        assert!(!design_leve_admits(2, 10, 0));
        assert!(!design_leve_admits(2, 10, 1));
        assert!(design_leve_admits(2, 10, 2));
        assert!(design_leve_admits(2, 10, 10));
        assert!(!design_leve_admits(2, 10, 11));
        // 与目录侧「hi == 0 不设上限」有意不同：DRWAND 是逐字的闭区间。
        assert!(design_leve_admits(0, 0, 0));
        assert!(!design_leve_admits(0, 0, 6));
    }

    #[test]
    fn the_negative_extent_cap_follows_the_host_diagonal_with_a_floor() {
        // 10.2 m × 10.2 m × 1.535 m 的 FLOOR：对角线 ≈ 14.5 m，上限 ≈ 116 m——落在
        // evidence/20260902-floor-72959-giant-ncyl §3 实测稳定的 2e4–1e11 mm 区间里。
        let floor = crate::solid::box_solid(10200.0, 10200.0, 1535.0).unwrap();
        let cap = negative_extent_cap(&floor);
        let diagonal = DVec3::new(10200.0, 10200.0, 1535.0).length();
        assert!((cap - diagonal * NEGATIVE_EXTENT_CAP_FACTOR).abs() < 1e-6);
        assert!(cap > 2.0e4 && cap < 1.0e11);
        // 小宿主不把上限压到比 1e4 mm 还低：一把贯穿 100 mm 小盒的 5 m 合法长刀不该被钳。
        let small = crate::solid::box_solid(100.0, 100.0, 100.0).unwrap();
        assert_eq!(negative_extent_cap(&small), NEGATIVE_EXTENT_CAP_FLOOR_MM);
    }

    /// 触发门槛是哨兵量级，不是「比宿主大」。
    ///
    /// WF03 GWALL `17496/116569` 名下那把 NXTR `17496/116587` 局部尺寸 9.2e4 mm，是整层楼的
    /// 修边刀；宿主上限 38 780 mm。拿上限当门槛会把它整刀跳过，墙末端多出 147.5 mm
    /// （07:56 基线 exact → off）。FLOOR 72959 的 3.2037e14 才是哨兵，钳到宿主上限。
    #[test]
    fn only_sentinel_extents_are_clamped_or_skipped() {
        assert!(!is_sentinel_extent(9.2e4));
        assert!(!is_sentinel_extent(-9.2e4));
        assert!(!is_sentinel_extent(5.0e8));
        assert!(is_sentinel_extent(3.2037e14));
        assert!(is_sentinel_extent(NEGATIVE_EXTENT_SENTINEL_MM));
        assert!(is_sentinel_extent(f64::INFINITY));
        assert!(is_sentinel_extent(f64::NAN));
        // 门槛必须远高于任何宿主尺度上限能到的值，否则又退回「比宿主大就是哨兵」。
        let floor = crate::solid::box_solid(10200.0, 10200.0, 1535.0).unwrap();
        assert!(negative_extent_cap(&floor) * 1000.0 < NEGATIVE_EXTENT_SENTINEL_MM);
    }

    #[test]
    fn implied_tube_v2_identity_includes_route_ordinal() {
        let tube = |route_ordinal| GeometryId::ImpliedTube {
            container_refno: "24383/73948".into(),
            route_ordinal,
            from_refno: "24383/10".into(),
            to_refno: "24383/20".into(),
        };
        assert_ne!(
            tube(0),
            tube(1),
            "repeated endpoints are distinct route slots"
        );
        assert_ne!(tube(0).to_string(), tube(1).to_string());
    }

    #[test]
    fn implied_tube_legacy_json_remains_readable() {
        let value = r#"{"kind":"implied_tube","container_refno":"24383/73948","from_refno":"24383/10","to_refno":"24383/20"}"#;
        let decoded: GeometryId = serde_json::from_str(value).expect("legacy geometry id");
        assert!(matches!(
            decoded,
            GeometryId::ImpliedTube {
                route_ordinal: u32::MAX,
                ..
            }
        ));
    }

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
    fn five_member_closed_spine_recovers_a_full_circle_and_direction() {
        let ccw = [
            DVec3::new(5000.0, 0.0, 0.0),
            DVec3::new(0.0, 5000.0, 0.0),
            DVec3::new(-5000.0, 0.0, 0.0),
            DVec3::new(0.0, -5000.0, 0.0),
            DVec3::new(5000.0, 0.0, 0.0),
        ];
        let (center, radius, start, sweep) =
            circular_spine_geometry(&ccw).expect("closed full circle");
        assert!(center.length() < 1e-9);
        assert!((radius - 5000.0).abs() < 1e-9);
        assert!(start.abs() < 1e-9);
        assert!((sweep - std::f64::consts::TAU).abs() < 1e-9);

        let clockwise = [ccw[0], ccw[3], ccw[2], ccw[1], ccw[4]];
        let (_, _, _, sweep) = circular_spine_geometry(&clockwise).expect("clockwise full circle");
        assert!((sweep + std::f64::consts::TAU).abs() < 1e-9);
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

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
//! - **本期不做**（`Category::NegUnimplemented` 里剩下的字典兜底项）→ 同样 `Skipped`，
//!   reason 说清是里程碑边界。DISH/SLCY 与 NDIS/NSLC 自 G2-2 起走基本体臂，不在此列。
//! - 负几何单个失败**不拖垮正体**：记进 [`GeneratedElement::negatives_skipped`]，
//!   正体照出——RVM 对拍时这类元素会以体积偏大暴露，报告里有对应记录。

use std::cell::Cell;

use glam::{DMat3, DMat4, DVec3};
use manifold_csg::Manifold;

use e3d_io::db_element::{DbElement, DbElementError};
use e3d_io::record::descriptor::DescriptorValue;
use e3d_io::refno::RefNo;

use crate::catalogue::{CatalogueMeshCache, model_catalogue_member_with_cache, repre_level_of};
use crate::category::{Category, CompositeFamily, Prim, classify};
use crate::discretise::{flatten_ring, has_core3d_skipped_link};
use crate::extrusion_reuse::{ExtrusionVariant, cached_extrusion};
use crate::polyhedron::{
    PolyhedronError, legacy_polygon_panels, polyhedron_faces, polyhedron_solid,
};
use crate::primitive_instance::{
    AnalyticPrimitive, PrimitiveBuild, PrimitiveInstance, instantiate_box,
    instantiate_centered_cylinder,
};
use crate::profile::{LoopProfile, ProfileError, loop_profile};
use crate::section::{
    SectionChain, SectionSource, evaluate_section_profile, mitre_extension_length,
    mitre_extension_solid, section_catalogue, section_cut_planes, section_local_solid,
    section_placement, spine_mitre_plane, spine_path, spine_swept_solid,
};
use crate::solid::{
    MANIFOLD_NUDGE_MM, SolidError, circ_torus_solid, circ_torus_solid_with_segments,
    cleanup_floor_boolean_components, csg_cylinder_solid_with_segments, cylinder_segments,
    dish_knuckle_radius, drop_subresolution_films, elliptical_dish_solid_with_segments,
    extrude_rings, inflate_about_center, negative_cylinder_segments, negative_part_rev_segments,
    placed, pyramid_solid, rect_torus_solid, rect_torus_solid_with_segments,
    revolve_rings_profile_faceted_inflated, slope_ended_cylinder_solid_with_segments,
    snap_negatives_to_host_faces, snout_segments, snout_solid_with_segments,
    spherical_dish_solid_with_segments, subtract_negatives,
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
    /// 型材截面求值那一路的错（`crate::section`）。装箱是因为
    /// [`crate::section::SectionError`] 自己也带一个 `ModelError`（截面要问表现级），
    /// 两个类型互相内嵌，不装箱算不出大小。
    #[error(transparent)]
    Section(Box<crate::section::SectionError>),
}

impl From<crate::section::SectionError> for ModelError {
    fn from(error: crate::section::SectionError) -> Self {
        // 截面那侧包着的本来就是 ModelError（表现级那一条）时原样还回来，
        // 别套成 `Section(Model(...))` 让报告多一层壳。
        match error {
            crate::section::SectionError::Model(inner) => inner,
            other => ModelError::Section(Box::new(other)),
        }
    }
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

/// 一块**没有体**的目录图元：参数化算下来零径或零高，Core 不出体，我们也建不出网格。
///
/// 留着它是因为 E3D 的 RVM 驱动不在这一步收手。圆柱照走 DRCYL：格子空着的那只是
/// `Cylinder[radius=0 height=-0]`（基准 80 根上 132 条，全在阀门身上），`PHEI` 真解成 0
/// 的那只是 `Cylinder[radius=57 height=-0]`，半径照留。零高但直径还在的 LSNO 照走 DRSNO：
/// `/1WCC188VN` 的 `13245/585575` 写成 `Snout[84.15, 84.15, 0, 0, 0]`。不写的话两侧记录
/// 条数对不上，几何本来是一致的。
#[derive(Debug, Clone, PartialEq)]
pub enum DegenerateCatalogueShape {
    Cylinder {
        radius_mm: f64,
        height_mm: f64,
    },
    /// 半径，不是直径。`height_mm` 为 0 时是一张圆盘，E3D 仍写 `Snout`。
    Snout {
        bottom_radius_mm: f64,
        top_radius_mm: f64,
        height_mm: f64,
        offset_x_mm: f64,
    },
}

pub struct DegenerateCataloguePart {
    /// 目录图元的名词（`SCYL` / `LCYL` / `LSNO` / `SCON`）。
    pub noun: String,
    /// 图元在元素局部系里的放置：平移是图元原点，Z 轴是它的轴向。
    pub local_transform: DMat4,
    pub shape: DegenerateCatalogueShape,
}

/// 与 `parts` 逐块对齐的**记录事实**：这一块要写几条记录、用哪只标架写。
///
/// 标架定**记录的矩阵**——E3D 的 DRCYL / DRSNO 写的是目录图元自己的标架（`ORIMAT(PAXI)`、
/// LSNO 的 PAAX/PBAX 标架），Z 列带着目录轴的符号、X/Y 列是 ORIMAT 的副轴。网格识别器
/// 反推出来的轴符号任意、副轴随 `from_rotation_arc`，ams7999 `/1WCC318VN` 16 件里只有 3 件
/// 四列全同；几何一样，矩阵不一样（happyrust/e3d-model#1）。
///
/// 装在 [`GeneratedElement::part_record_repeats`] 里（名字还是旧的「份数」，字段名没改：
/// `pipeline.rs` 的隐式管身字面量正被别的会话改着，这里不动它的一个字节；份数仍是这个
/// 结构的一格）。
#[derive(Debug, Clone, PartialEq)]
pub struct CataloguePartRecord {
    /// 这一块在 GMSE 里被几个成员算成了同一只网格、同一个放置，RVM 就写几条记录。
    pub repeats: usize,
    /// 放置标架（元素局部系）：列 = 图元自己的 X / Y / Z，平移 = 图元自己的原点，
    /// 就是 [`crate::catalogue`] 把局部网格摆到元素里用的那只矩阵。没有标架的（型材成员）
    /// 是 `None`，记录照网格反推。
    pub frame: Option<DMat4>,
    /// 目录 `SDSH` 且 `PRAD > 0`。E3D 的 `DRDISH` 据此写 `EllipticalDish`，不看形状：
    /// r = h 时形体退化成半球，与球形碟逐点相同，网格认不出种类。
    pub knuckled_dish: bool,
}

pub struct GeneratedElement {
    /// 几何缓存、RVM 配对和增量更新使用的稳定身份。
    pub geometry_id: GeometryId,
    /// 来源元素保持不变；派生几何也不得用伪 refno 覆盖它。
    pub refno: RefNo,
    pub noun: String,
    /// The route endpoints are present, but E3D rejects this implied tube as a
    /// solid. Consumers keep it as a dashed diagnostic centre line.
    pub is_invalid_tubi: bool,
    pub name: Option<String>,
    /// 世界系实体，负几何已扣。
    pub solid: Manifold,
    /// RVM-compatible, non-boolean component list. Present for visible catalogue
    /// programs because E3D preserves their primitives as separate display meshes.
    pub parts: Option<Vec<Manifold>>,
    /// 与 `parts` 逐块对齐的目录图元名词（`SBOX` / `SCYL` / `LPYR` / `LSNO`…）。
    /// 空表 = 出身不明（设计侧实体、型材本体都没有）。
    ///
    /// **RVM 的记录种类由出身定，不由网格长相定。** E3D 的写出器按目录图元挑
    /// （LPYR→DRPYR、LSNO/SCON→DRSNO、SBOX→DRBOX…），一只 PAAX/PBAX 退化成正方盒的
    /// LPYR 照样写 `Pyramid`。网格识别器不知来历，只能按长相先中先得，于是 300 只阀门的
    /// 手轮座写成了 `Box`、等径 LSNO 写成了 `Cylinder`——几何 exact，种类却不一致。
    /// **记录的矩阵也由出身定**：标架在 [`Self::part_record_repeats`] 里一块一只。
    pub part_catalogue_nouns: Vec<String>,
    /// 与 `parts` 逐块对齐的**记录事实**（[`CataloguePartRecord`]）：份数 + 放置标架。
    /// 空表 = 每块一条、没有标架。
    ///
    /// 份数：体只放一块（两块全同的体并起来是自交的，体积也会翻倍），但 E3D 的 RVM 驱动
    /// 逐成员写记录、不管它们是不是长在同一个地方：`/Copy-of-7#管线-1/B1` 的 `VALV 3`
    /// 基准里第 5、6 两条是字节一样的两只圆柱。只写一条，两侧记录条数就对不上，而几何本来一致。
    /// 标架：圆柱 / 圆台记录的矩阵照它写，不照网格反推（happyrust/e3d-model#1）。
    pub part_record_repeats: Vec<CataloguePartRecord>,
    /// 退化成零尺寸的目录图元。Core 与我们都不出体，**E3D 的 RVM 驱动照样落一条记录**，
    /// 于是基准里凭空多出一条 `Cylinder[r=0 h=0]`。
    pub degenerate_parts: Vec<DegenerateCataloguePart>,
    /// 本元素局部系到世界系的刚体变换。实体里的 SJUS 下移保留在局部几何中，
    /// 因而把最终网格乘这个矩阵的逆即可得到与 E3D facet 可比的元素局部坐标。
    pub world: DMat4,
    /// Present only when the visible result is an unbooled reusable primitive.
    /// Persistence may write this as legacy `inst_geo + geo_relate.trans`
    /// instead of allocating a per-element `booled_id` mesh.
    pub primitive_instance: Option<PrimitiveInstance>,
    /// 网格之外的解析身份（[`AnalyticPrimitive`]），只在独立正体、且网格识别器认不出那族
    /// 形状时才有——目前就是 SLCY。RVM 写出端拿它直出记录，仍要过自校闸。
    pub analytic_primitive: Option<AnalyticPrimitive>,
    pub negatives_applied: usize,
    /// `negatives_applied` 里来自目录件自己 `NGMR`（→ NGMS）的那几把：扣的是自己的正体，
    /// 不是记录级 owned 负体。RVM 写出端靠它区分两类记录写法——桥架/托盘的记录级负体
    /// E3D 照写未扣减的 SBOX/SEXT/SREV（`write_uncut_catalogue_parts`），而 NGMS 自扣
    /// 的正体 E3D 写**扣过的**整只面片（aps7200 `/STAIR-STEEL-PRE-12/BASE` 的 Ø20 栓孔板）。
    pub own_ngms_cuts: usize,
    pub negatives_skipped: Vec<SkippedNegative>,
    /// 已解释的关节刀：没有几何源、但其拓扑类已被原生证据判定「端面已覆盖、不新增刀」
    /// 的条目（`JointCutOutcome::CoveredByEndFaces`，[`JointClass`]）。与
    /// `negatives_skipped` 并列、不互相冲抵；reason 前冠类别标签与证据标签。
    pub joint_cuts_explained: Vec<SkippedNegative>,
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

    /// 各分量亏格之和（一块分量的亏格 = 穿透它的环柄数，一只贯穿孔加一）。
    ///
    /// 多分量的 [`Manifold`] 直接问 `genus()` 拿到的是 1 − χ/2 把所有分量的欧拉数混在一起：
    /// 两块实心体算成 −1，一块两孔梁加一片零厚尖条（`restore_sloped_joint_tips` 给斜接梁补的
    /// 那片，E3D 自己的 facets 也留着它）算成 1——`aps250110 /1819` 的真库门就是这样把两只腹板
    /// 螺栓孔数成一只的。所以先 `decompose` 再逐块求和；单分量结果不变，SCTN 那种本就分块交出来的
    /// 照旧按 `parts` 求和。
    pub fn genus(&self) -> i32 {
        self.parts.as_ref().map_or_else(
            || self.solid.decompose().iter().map(Manifold::genus).sum(),
            |parts| parts.iter().map(Manifold::genus).sum(),
        )
    }
}

/// 「有依据地没出几何」的分类。报告按它分账。
///
/// 分类跟着结论一起从这里交出去，而不是让报告去猜 `reason` 那句话——判据写第二份，
/// 两份迟早分岔，而分岔的表现一律是「两边的账各自都平、数对不上」
/// （见 [`crate::category::is_model_unit`] 文档里那次 SBFI 漏件）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SkipKind {
    /// E3D 同样不出几何的常规无形状：零高、尺寸退化、目录侧没有可见图元。
    NoShape,
    /// 型材求不到截面（无 `SPRE`、或 GMSS 成员全被显示筛掉）：E3D 只画一条中心线
    /// （`gen-model/test_data/rvm/1RX03-MSUP-E0110.rvm` 里 8 根 MSUP 杆件全是 `Line`）。
    ///
    /// 与 [`SkipKind::NoShape`] 分开：那条是「E3D 也什么都不画」，这条是**E3D 画了一条线
    /// 而我们一条都还不画**——账上混在一起，中心线这笔欠款就永远看不见。
    CentrelineOnly,
    /// 目录链断：引用悬空、库没开、缺 `CATR` / `GSTR`。数据或配置问题，要点名。
    CatalogueChainBroken,
}

/// 建模结论：出了实体 / 有依据地没出。
pub enum Modeled {
    Solid(Box<GeneratedElement>),
    Skipped { kind: SkipKind, reason: String },
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
    let _timing = crate::perf::elmodl_scope();
    // 路由成员的实体不是设计侧正体，而是目录 GMRE 程序求值的产物——与全量管线
    // pipeline 的 RouteMember 臂共用同一个 [`model_catalogue_member_with_cache`]。
    // 这条臂必须存在的理由与 `category::is_model_unit` 文档里那次 SBFI 事故同型：
    // 全量为成员产出 element:refno 键的实体，增量上卷到成员后走的是这里；两边
    // 不是同一份答案，被删/被改的成员实体就既不重建也不出 Remove。
    //
    // 有意不走下面共享的负体扣减路：成员的负体语义在目录侧（SBOX 底下的
    // NSCY/NSBO/NLCY，已在求值臂内扣完），设计侧子负体在全量管线里同样不由
    // 路由成员消费，两边照旧各记各的账。
    //
    // `NOZZ` 走同一条：它的实体同样是目录件 GMRE 的求值结果，只是入口那一跳是
    // `CATR`（`category::enters_catalogue_by_catref`）。全量在管线里与路由成员共用
    // 那条臂，增量就得在这里共用同一份答案，否则又是一次 SBFI 同型缺口。
    //
    // `FIXING` 也走这一条：它同属目录件，但几何不是型材截面而是目录构件的 GMRE
    // 求值结果，入口与路由成员一样是 `SPRE`（`category::builds_from_catalogue_component`）。
    // `FITT` 同路：SCTN / STWALL 上的配件（aps250110 墙上的门窗 `FITTING n of STWALL m`、
    // ams7997 梁上的端板 `FITTING n of SCTN …`）是 SFIT 的 GMRE 求值结果，标架由属主现算
    // （`transform::ImplicitFrame::SectionFitting`），`world_matrix` 已经给出；E3D 的 RVM
    // 给每只各开一组，梁 / 墙的组里只有本体。哪些 FITT 算单元由 `category::is_model_unit_of` 定。
    let noun = el.element_type()?;
    if category == Category::RouteMember
        || (matches!(category, Category::Composite(CompositeFamily::CoreClass28))
            && crate::category::enters_catalogue_by_catref(&noun))
        || (category == Category::Catalog
            && crate::category::builds_from_catalogue_component(&noun))
    {
        let build = model_catalogue_member_with_cache(el, cache)?;
        return Ok(match build.generated {
            Some(generated) => Modeled::Solid(Box::new(generated)),
            None => Modeled::Skipped {
                kind: SkipKind::NoShape,
                reason: format!(
                    "目录侧无可见几何，E3D 同样不出几何：{}（局部失败 {} 条）",
                    build
                        .no_shape
                        .unwrap_or_else(|| "无 SPRE/CATR/GMRE 或成员全退化".to_string()),
                    build.diagnostics.len()
                ),
            },
        });
    }
    // 电缆桥架托盘：与管线 `Composite(CableTray) if noun == "CTRAY"` 臂同一份目录求值。
    // 判据在 `category::is_model_unit`（CTRAY 是单元、桥架件不是），这里只是它的执行臂
    // ——少了它，增量上卷到 CTRAY 之后就掉进下面的设计侧正体路，对着一个没有
    // XLEN/DIAM 的复合件读尺寸，报出来的是「属性缺失」而不是「托盘变了」。
    if matches!(category, Category::Composite(CompositeFamily::CableTray)) && noun != "CABLE" {
        if !noun.eq_ignore_ascii_case("CTRAY") {
            return Ok(Modeled::Skipped {
                kind: SkipKind::NoShape,
                reason: format!("{noun} 是桥架件，只算放置，几何在它下面的 CTRAY 上"),
            });
        }
        let build = model_catalogue_member_with_cache(el, cache)?;
        return Ok(match build.generated {
            Some(generated) => Modeled::Solid(Box::new(generated)),
            None => Modeled::Skipped {
                kind: SkipKind::NoShape,
                reason: format!(
                    "目录链（SPRE→…→SCOM→GMRE）没有产出可见几何：{}（局部失败 {} 条）",
                    build.no_shape.unwrap_or_else(|| "原因未记".to_string()),
                    build.diagnostics.len()
                ),
            },
        });
    }
    // DRWAND 的表现级门对**正体**同样成立，不只对刀。SLYK `/Big-HS-SEED-ZONE` 里十只
    // 种子盒写的是 `LEVE=[7,10]`，TTY 会话停在 `LEVEL 0`，E3D 的窄口径 RVM 里它们连
    // 一个组都没有；我们照建不误，十只凭空多出来。门只挡这一只元素的几何，不剪它的
    // 成员——E3D 也照样把上面的 STRU 容器导了出来，只是空的。
    let level = repre_level_of(el)?;
    let (leve_lo, leve_hi) = negative_leve(el);
    if !design_leve_admits(leve_lo, leve_hi, level) {
        return Ok(Modeled::Skipped {
            kind: SkipKind::NoShape,
            reason: format!(
                "LEVE=[{leve_lo},{leve_hi}] 不含表现级 {level}（DRWAND 门，E3D 同样不画）"
            ),
        });
    }

    let mut notes = Vec::new();
    // Decide the instancing boundary before constructing the positive body.
    // Any boolean operand stays at real dimensions: unit scaling can differ by
    // a few ulps and coincident-face CSG may amplify that into another topology.
    let mut negatives = Vec::new();
    let gate_timing = crate::perf::neg_gate_scope();
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
    let mut joint_cuts_explained = Vec::new();
    let mut fixing_catalogue_cuts = Vec::new();
    if noun == "GENSEC" || noun == "SCTN" {
        collect_joint_negatives(
            el,
            &noun,
            &mut negatives,
            &mut fixing_catalogue_cuts,
            &mut skipped,
            &mut joint_cuts_explained,
            &mut notes,
            cache,
        )?;
        collect_host_fixing_negatives(
            el,
            &mut negatives,
            &mut fixing_catalogue_cuts,
            &mut skipped,
            &mut notes,
            cache,
        )?;
        if noun == "SCTN" {
            collect_member_joint_negatives(
                el,
                &mut negatives,
                &mut fixing_catalogue_cuts,
                &mut skipped,
                &mut joint_cuts_explained,
                &mut notes,
                cache,
            )?;
            collect_owner_template_negatives(
                el,
                &mut negatives,
                &mut fixing_catalogue_cuts,
                &mut skipped,
                &mut notes,
                cache,
            )?;
        }
    }
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
    drop(gate_timing);
    let had_negative_members = !negatives.is_empty() || !fixing_catalogue_cuts.is_empty();

    // SCTN 的局部系不是它自己的 POS/ORI——那两个属性它根本没存（实测 `24381/47163`
    // 只有 POSS/POSE/DRNS/DRNE/BANG/JUSL），标架是 GTSTDT 从 POSS→POSE + BANG +
    // JUSL p-line 推出来的，要在**属主**世界系上右乘。拿到值的那一路在这里留一手，
    // 下面算 `world` 时用。
    let mut section_frame: Option<DMat4> = None;
    // 截面本体并集前的那几块（局部系）。RVM 的记录单位是 GMSS 成员，不是并集后的一块。
    let mut section_members: Vec<Manifold> = Vec::new();
    let mut section_member_sources: Vec<SectionSource> = Vec::new();

    let local_timing = crate::perf::local_solid_scope();
    let mut analytic_primitive: Option<AnalyticPrimitive> = None;
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
                    kind: SkipKind::NoShape,
                    reason: format!("HEIG={h}（≈0，E3D 同样不出几何）"),
                });
            }
            Err(error) => {
                if let Some(reason) = profile_no_shape_reason(&error) {
                    return Ok(Modeled::Skipped {
                        kind: SkipKind::NoShape,
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
                            kind: SkipKind::NoShape,
                            reason: format!("{reason}（Core3D 尺寸校验不产几何）"),
                        });
                    }
                    return Err(error);
                }
            };
            analytic_primitive = built.analytic;
            (built.solid, built.instance)
        }
        Category::Revolution => (revolution_local_solid(el, &noun, 0.0, &mut notes)?, None),
        Category::Polyhedron => (polyhedron_local_solid(el, &mut notes)?, None),
        // 布局式扶手（`crate::handrail`）：判据在 `category::is_model_unit` 的
        // `Composite(Structural) => HANDRA`，这里是它的执行臂；管线的同名臂与增量上卷
        // 都走到这里。详图式（几何在 RAIL/KICKPL/HRPOST 成员上）与没东西可画的两种结局
        // 都是「E3D 在 HANDRA 这一层同样不画」，落 `Skipped(NoShape)`。
        Category::Composite(CompositeFamily::Structural) if noun == "HANDRA" => {
            match crate::handrail::handrail_local_solid(el, &mut notes)? {
                crate::handrail::HandrailBuild::Solid(solid) => (solid, None),
                crate::handrail::HandrailBuild::Detailed(reason)
                | crate::handrail::HandrailBuild::NoGeometry(reason) => {
                    return Ok(Modeled::Skipped {
                        kind: SkipKind::NoShape,
                        reason: if notes.is_empty() {
                            reason
                        } else {
                            format!("{reason}; {}", notes.join("; "))
                        },
                    });
                }
            }
        }
        Category::Catalog if noun == "SBFI" => match sbfitting_local_solid(el, &mut notes) {
            Ok(solid) => (solid, None),
            Err(error) => {
                if let Some(reason) = sbfitting_no_shape_reason(&error) {
                    // 嵌套在 `CMFI` 下的 `SBFI` 没有 DESP 时并非无件可出：原生 RVM
                    // 里的 `SBFITTING` 组就是 `SPRE→SFIT` 的目录求值结果（aps7200
                    // `23584/8625`/`8629`，与 `FITT` 同一块带孔平板、平放标架）。
                    // 沿 `builds_from_catalogue_component` 那条臂求值，标架走
                    // `transform::ImplicitFrame::SubFitting`。
                    if el.get_element("SPRE")?.is_some() {
                        let build = model_catalogue_member_with_cache(el, cache)?;
                        return Ok(match build.generated {
                            Some(generated) => Modeled::Solid(Box::new(generated)),
                            None => Modeled::Skipped {
                                kind: SkipKind::NoShape,
                                reason: format!(
                                    "{reason}；目录侧也无可见几何：{}（局部失败 {} 条）",
                                    build.no_shape.unwrap_or_else(|| "原因未记".to_string()),
                                    build.diagnostics.len()
                                ),
                            },
                        });
                    }
                    return Ok(Modeled::Skipped {
                        kind: SkipKind::NoShape,
                        reason: format!("{reason}（DESP 没带套管尺寸，E3D 同样不出几何）"),
                    });
                }
                return Err(error);
            }
        },
        Category::Catalog if noun == "WALL" => (wall_local_solid(el, &mut notes)?, None),
        // 详图式扶手的栏杆与踢脚板：目录截面沿属主 HANDRA 的 RPATH 扫一段
        // （`crate::handrail`，recheck-4 §五 W4-1 施工图·续 七）。截面链与 SCTN 同一条，
        // 三种结局也同口径。
        Category::Catalog if crate::handrail::is_detailed_item(&noun) => {
            match crate::handrail::detailed_item_local_solid(el, &mut notes)? {
                crate::handrail::RailBuild::Solid(solid) => (solid, None),
                crate::handrail::RailBuild::CentrelineOnly(reason) => {
                    return Ok(Modeled::Skipped {
                        kind: SkipKind::CentrelineOnly,
                        reason: format!("{reason}; {}", notes.join("; ")),
                    });
                }
                crate::handrail::RailBuild::ChainBroken(reason) => {
                    return Ok(Modeled::Skipped {
                        kind: SkipKind::CatalogueChainBroken,
                        reason: format!("{reason}; {}", notes.join("; ")),
                    });
                }
            }
        }
        Category::Catalog if is_section_like(&noun) => match section_body(el, &mut notes)? {
            SectionBody::Solid {
                solid,
                members,
                member_sources,
                frame,
            } => {
                section_frame = Some(frame);
                section_members = members;
                section_member_sources = member_sources;
                (solid, None)
            }
            // 本体建不出来就是这一件没有：名下 FITT 的几何不在这里兜——它们自己是单元
            // （`category::is_model_unit_of`），全量遍历各建各的，E3D 的 RVM 也各写一组。
            SectionBody::CentrelineOnly(reason) => {
                return Ok(Modeled::Skipped {
                    kind: SkipKind::CentrelineOnly,
                    reason: format!("{reason}; {}", notes.join("; ")),
                });
            }
            SectionBody::NoShape(reason) => {
                return Ok(Modeled::Skipped {
                    kind: SkipKind::NoShape,
                    reason: format!("{reason}; {}", notes.join("; ")),
                });
            }
            SectionBody::ChainBroken(reason) => {
                return Ok(Modeled::Skipped {
                    kind: SkipKind::CatalogueChainBroken,
                    reason: format!("{reason}; {}", notes.join("; ")),
                });
            }
        },
        Category::Composite(_) if noun == "CABLE" => {
            if !crate::cable::geometry_enabled() {
                return Ok(Modeled::Skipped {
                    kind: SkipKind::NoShape,
                    reason: "实验性电缆几何未启用；原生 RVM 对拍仍未全过".into(),
                });
            }
            match crate::cable::cable_local_solid(el, &mut notes)? {
                Some(solid) => (solid, None),
                None => {
                    return Ok(Modeled::Skipped {
                        kind: SkipKind::NoShape,
                        reason: "电缆 OUTD 为零、没有路由或路由全部退化".into(),
                    });
                }
            }
        }
        other => {
            return Ok(Modeled::Skipped {
                kind: SkipKind::NoShape,
                reason: format!("类别 {other:?} 不产正实体"),
            });
        }
    };
    drop(local_timing);

    let world = if let Some(frame) = section_frame {
        world_matrix(el)? * frame
    } else if category == Category::Catalog && noun == "SBFI" {
        sbfitting_world(el)?
    } else {
        world_matrix(el)?
    };
    let place_timing = crate::perf::place_world_scope();
    let pos_world = placed(&local, world);
    drop(place_timing);
    let positive_bounds = pos_world.bounding_box().map(|bounds| {
        let min = bounds.min();
        let max = bounds.max();
        ([min[0], min[1], min[2]], [max[0], max[1], max[2]])
    });

    let negatives_timing = crate::perf::negatives_scope();
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
    applied.extend(fixing_catalogue_cuts);
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
    if noun == "GWALL" {
        let mut sleeves = Vec::new();
        collect_host_sleeves(el, &mut sleeves)?;
        for sleeve in sleeves {
            match sleeve_world_cutter(&sleeve) {
                Ok(solid) => applied.push(solid),
                Err(error) => skipped.push(SkippedNegative {
                    refno: sleeve.refno(),
                    noun: "SBFI".into(),
                    reason: error.to_string(),
                }),
            }
        }
    }
    let negatives_applied = applied.len();
    // 近共面的刀先按 libgm 的归一化口径吸附到宿主面上，再做差——不然差出来的是
    // 一张贴在主体上的亚容差皮，事后的 `drop_subresolution_films` 也收不着它。
    let applied = snap_negatives_to_host_faces(&pos_world, applied);
    let mut solid = subtract_negatives(pos_world, applied);
    let mut restored_joint_tips = false;
    if noun == "GENSEC" && negatives_applied > 0 {
        if let Some((positive_min, positive_max)) = positive_bounds {
            let (restored, did_restore) =
                restore_sloped_joint_tips(el, solid, positive_min, positive_max)?;
            solid = restored;
            restored_joint_tips = did_restore;
        }
    }
    if noun == "FLOOR" {
        solid = cleanup_floor_boolean_components(solid, negatives_applied);
    }
    if negatives_applied > 0 && !restored_joint_tips {
        solid = drop_subresolution_films(solid);
    }
    drop(negatives_timing);
    let materialize_timing = crate::perf::materialize_scope();
    let is_empty = solid.is_empty() || solid.num_tri() == 0;
    drop(materialize_timing);
    if is_empty {
        if had_negative_members {
            return Ok(Modeled::Skipped {
                kind: SkipKind::NoShape,
                reason: format!(
                    "应用 {negatives_applied} 个负体后可见实体为空（Core3D 同样无可显示几何）"
                ),
            });
        }
        return Err(ModelError::Solid(SolidError::EmptyManifold {
            what: "element after negatives",
        }));
    }

    // SCTN 名下的 FITT **不**并进梁体：它自己是一件（`category::is_model_unit_of`，
    // 与 STWALL 名下的同一口径），E3D 的 RVM 也给它自己开一组——ams7997
    // `/1TFA004MP-SUPP-01-V1` 的组里只有梁那一条面片，`FITTING 1 / 2 of SCTN …` 各是自己的
    // CNTB（`test_data/rvm/SCTN-HW150-C80.rvm`）。2026-09-29 之前这里把 FITT 的目录几何拼进
    // 梁的 `parts`，梁的 AABB 因此吞掉端板，而 FITT 自己在模型集里没有一行。
    let mut part_catalogue_nouns: Vec<String> = Vec::new();
    let parts = if noun == "SCTN" {
        // 本体那几块：E3D 逐 GMSS 成员写记录（管型材的截面是两个 `PANG=180` 的 SANN 半环，
        // 基准 RVM 里就是同心反向的两条面片），所以记录单位取并集前的成员，而不是并集后的
        // 一块。带负体时只有并集体是可见结果——`parts` 按定义是「非布尔」的那一份，退回整只。
        if negatives_applied == 0 && section_members.len() > 1 {
            // 逐块的出身跟着一起交出去：E3D 对 `SREC` 成员走 DRPYR，对 `SPRO` / `SANN`
            // 才走拉伸体的面片（见 [`GeneratedElement::part_catalogue_nouns`]）。
            part_catalogue_nouns = section_member_sources
                .iter()
                .map(|source| section_source_noun(*source).to_string())
                .collect();
            Some(
                section_members
                    .iter()
                    .map(|member| placed(member, world))
                    .collect(),
            )
        } else {
            None
        }
    } else {
        None
    };

    Ok(Modeled::Solid(Box::new(GeneratedElement {
        geometry_id: GeometryId::Element {
            refno: el.refno().to_string(),
        },
        refno: el.refno(),
        noun,
        is_invalid_tubi: false,
        name: el.stored_name()?,
        solid,
        parts,
        part_catalogue_nouns,
        part_record_repeats: Vec::new(),
        degenerate_parts: Vec::new(),
        world,
        primitive_instance: (!had_negative_members)
            .then_some(primitive_instance)
            .flatten(),
        analytic_primitive: (!had_negative_members)
            .then_some(analytic_primitive)
            .flatten(),
        negatives_applied,
        own_ngms_cuts: 0,
        negatives_skipped: skipped,
        joint_cuts_explained,
        notes,
    })))
}

/// E3D's FacetGroup writer keeps a zero-thickness tip where a shallowly sloped
/// GENSEC meets an end-plate NBOX.  The manifold boolean quite correctly drops
/// that degenerate facet, which otherwise shortens the generated AABB by the
/// cutter depth.  Restore only the small missing face interval, and only when
/// the path is genuinely sloped; ordinary horizontal end-plate beams retain
/// their real shortened bounds.
fn restore_sloped_joint_tips(
    el: &DbElement,
    mut solid: Manifold,
    positive_min: [f64; 3],
    positive_max: [f64; 3],
) -> Result<(Manifold, bool), ModelError> {
    let Some(path) = spine_path(el)? else {
        return Ok((solid, false));
    };
    let world_rotation = DMat3::from_mat4(world_matrix(el)?);
    let sloped = path.segments.iter().any(|segment| {
        let (start, end) = segment.tangents();
        let start = world_rotation * start;
        let end = world_rotation * end;
        let horizontal = start.x.hypot(start.y).max(end.x.hypot(end.y));
        let shallow_turn = |t: DVec3| {
            let dominant = t.x.abs().max(t.y.abs());
            let minor = t.x.abs().min(t.y.abs());
            dominant > 1e-9 && minor / dominant > 1e-7 && minor / dominant < 0.01
        };
        horizontal > 1e-9 && (shallow_turn(start) || shallow_turn(end))
    });
    if !sloped {
        return Ok((solid, false));
    }
    let Some(bounds) = solid.bounding_box() else {
        return Ok((solid, false));
    };
    let current_min = bounds.min();
    let current_max = bounds.max();
    let mut tips = Vec::new();
    const TIP_THICKNESS: f64 = 0.01;
    const MAX_TIP_GAP: f64 = 10.0;
    for axis in 0..3 {
        let min_gap = current_min[axis] - positive_min[axis];
        if (0.1..=MAX_TIP_GAP).contains(&min_gap) {
            tips.push(joint_tip_slab(
                axis,
                true,
                positive_min,
                current_min,
                current_max,
                min_gap,
                TIP_THICKNESS,
            )?);
        }
        let max_gap = positive_max[axis] - current_max[axis];
        if (0.1..=MAX_TIP_GAP).contains(&max_gap) {
            tips.push(joint_tip_slab(
                axis,
                false,
                positive_max,
                current_min,
                current_max,
                max_gap,
                TIP_THICKNESS,
            )?);
        }
    }
    let restored = !tips.is_empty();
    if restored {
        let mut pieces = Vec::with_capacity(tips.len() + 1);
        pieces.push(solid);
        pieces.extend(tips);
        solid = Manifold::compose(&pieces);
    }
    Ok((solid, restored))
}

fn joint_tip_slab(
    axis: usize,
    at_min: bool,
    positive: [f64; 3],
    current_min: [f64; 3],
    current_max: [f64; 3],
    gap: f64,
    thickness: f64,
) -> Result<Manifold, ModelError> {
    let mut dims = [
        (current_max[0] - current_min[0]).max(thickness),
        (current_max[1] - current_min[1]).max(thickness),
        (current_max[2] - current_min[2]).max(thickness),
    ];
    dims[axis] = gap + thickness;
    let thin_axis = (axis + 1) % 3;
    dims[thin_axis] = thickness;
    let mut origin = current_min;
    origin[axis] = if at_min {
        positive[axis]
    } else {
        current_max[axis]
    };
    let center = [
        origin[0] + dims[0] * 0.5,
        origin[1] + dims[1] * 0.5,
        origin[2] + dims[2] * 0.5,
    ];
    Ok(crate::solid::box_solid(dims[0], dims[1], dims[2])?
        .translate(center[0], center[1], center[2]))
}

/// 走 [`section_body`] 那条截面链的 noun：`SPRE` 目录截面挤成直段、两端切平面。
///
/// `SCTN` 与 `GENSEC` 是原住民；`STWALL`（直墙）2026-09-06 按 recheck-4 §十接进来——
/// 语料里它的形状与 SCTN 逐条同型：`SPRE → SPRO`（`PX/PY = ATTRIB DESP[n]`，按设计件
/// 自己的 `DESP` 参数化的矩形）、`POSS → POSE`、`DRNS`/`DRNE` 端向。它**不是** `WALL`：
/// `WALL` 的路径是 SPINE 圆弧，截面虽然也取 SPRE 目录截面，实体却是沿弧扫出来的（[`wall_local_solid`]）。
///
/// 这份名单与 [`crate::category::is_model_unit`] 的 `Catalog` 臂必须逐项对齐：
/// 那边多认一个、这边没有臂，落到 `other` 臂是 `Skipped(NoShape)`，账面上看着像
/// 「E3D 也不出几何」，实际是没接线。
fn is_section_like(noun: &str) -> bool {
    matches!(noun, "SCTN" | "GENSEC" | "STWALL")
}

/// [`section_body`] 的三种结局。三条各有各的账，混成一句话就分不出
/// 「E3D 画了中心线我们没画」与「目录链根本走不通」。
enum SectionBody {
    /// Authoritatively empty finite dimensions, distinct from malformed input.
    NoShape(String),
    /// 切完两端的裸梁 + 它的局部标架（要在属主世界系上右乘）。
    Solid {
        solid: Manifold,
        /// 并集前的逐 GMSS 成员实体（与 `solid` 同一局部系），RVM 按它们写记录。
        /// GENSEC 的扫掠还没按成员拆，那一支交空表 = 「只有整只一块」。
        members: Vec<Manifold>,
        /// 与 `members` 逐块对齐的出身，见 [`crate::section::SectionSolid::member_sources`]。
        member_sources: Vec<SectionSource>,
        frame: DMat4,
    },
    CentrelineOnly(String),
    ChainBroken(String),
}

/// 截面成员的出身写成目录名词，交给 RVM 写出器挑写出器用。
fn section_source_noun(source: SectionSource) -> &'static str {
    match source {
        SectionSource::Srec => "SREC",
        SectionSource::Sann => "SANN",
        SectionSource::Spro { .. } => "SPRO",
    }
}

/// SCTN 直段（G1-1a 第 2 + 3 步）：目录截面 → GTSTDT 标架 → 挤出 → 两端切平面。
///
/// 四段对标 `DB_Sctn::DB_makeCSG@0x10738C30` 的前两个阶段（调试串 `profile` → `xsec1`）：
/// 截面子树逐 GMSS 成员建体并集，再由 `DB_Scan_to_GML` 槽 28 拿两张切面切。
///
/// **第 4 阶段（负体）**：GENSEC 的 JOIS/JOIE 指向 FIXING 时，它 TMPL 里的负体
/// 在 [`model_element_with_cache`] 的负体段扣（[`collect_joint_negatives`]）；SCTN 的
/// JOIS/JOIE（SJOI/PJOI 族）与成员链（FITT/CMFI(+SBFI)/SNOD→SJOI/SCOJ→SUBJ）同函数
/// 按各自 mode 扫。没有几何源（TMPL 成员与 SPRE→CATR 都没有）的刀源记进
/// `negatives_skipped` 并冠拓扑类别标签（[`JointClass`]）；其中 J-A 类（JOIS/JOIE →
/// 另一根 SCTN 的 SNOD 上 `CREF` 回指宿主的 SJOI）有原生证据证明端面已覆盖，
/// 另记 `joint_cuts_explained`。有源但刀被 NAPP 门全挡下或源里本就没负体的记 note。
fn section_body(el: &DbElement, notes: &mut Vec<String>) -> Result<SectionBody, ModelError> {
    let catalogue = match section_catalogue(el)? {
        SectionChain::NoSpre => {
            return Ok(SectionBody::CentrelineOnly(
                "无 SPRE：E3D 只画中心线，不出实体".to_string(),
            ));
        }
        SectionChain::Broken { hop, at, why } => {
            return Ok(SectionBody::ChainBroken(format!(
                "SPRE 链第 {hop} 跳断在 {at}：{why}"
            )));
        }
        SectionChain::Resolved(catalogue) => catalogue,
    };
    let profile = evaluate_section_profile(el, &catalogue)?;
    notes.extend(profile.notes.iter().cloned());
    if profile.is_empty() {
        return Ok(SectionBody::CentrelineOnly(
            "目录截面一个成员都出不了形状（全被显示筛掉或是空占位），E3D 同样只画中心线"
                .to_string(),
        ));
    }
    // SCTN 走 `POSS→POSE` + `DRNS`/`DRNE` 斜切；GENSEC 没有这四条属性，路径在 SPINE 上
    // （直段 + THRU/FILL/CENT 弧），截面沿路径扫掠（`DB_Gensec@0x10730090` /
    // `sub_107318E0`，见 [`crate::section::spine_swept_solid`]）。截面链两边共用。
    if el.element_type()? == "GENSEC" {
        let Some(path) = spine_path(el)? else {
            return Ok(SectionBody::CentrelineOnly(
                "没有 SPINE：E3D 同样不出实体".to_string(),
            ));
        };
        if path.segments.is_empty() {
            return Ok(SectionBody::CentrelineOnly(format!(
                "SPINE 建不出任何一段（E3D 同样报错不出实体）：{}",
                path.problems.join("；")
            )));
        }
        notes.extend(path.problems.iter().cloned());
        if path.has_arc() {
            notes.push(format!(
                "GENSEC 曲线扫掠：{} 段（弧型 {}）",
                path.segments.len(),
                path.curve_types.join(",")
            ));
        }
        let bang_deg = el.get_double("BANG")?.unwrap_or(0.0);
        // 实体已在 GENSEC 自身局部系里（SPINE 点经 SPINE 的 POS/ORI 折算），标架取单位阵。
        return match spine_swept_solid(&profile, &path, bang_deg, notes)? {
            // 扫掠是逐成员**再逐路径段**并起来的，拆回记录单位是另一件事，这里先不拆。
            Some(solid) => Ok(SectionBody::Solid {
                solid,
                members: Vec::new(),
                member_sources: Vec::new(),
                frame: DMat4::IDENTITY,
            }),
            None => Ok(SectionBody::CentrelineOnly(
                "截面折线化 / 沿 SPINE 扫掠之后没有实体".to_string(),
            )),
        };
    }
    let placement = match section_placement(el, &profile) {
        Ok(placement) => placement,
        Err(error @ crate::section::SectionError::EmptyLength { .. }) => {
            return Ok(SectionBody::NoShape(error.to_string()));
        }
        Err(error) => return Err(error.into()),
    };
    let cuts = section_cut_planes(el, &placement)?;
    match section_local_solid(&profile, &placement, &cuts, notes)? {
        Some(built) => Ok(SectionBody::Solid {
            solid: built.body,
            members: built.members,
            member_sources: built.member_sources,
            frame: placement.local,
        }),
        None => Ok(SectionBody::CentrelineOnly(
            "截面折线化 / 挤出 / 端切之后没有实体".to_string(),
        )),
    }
}

/// Build a design WALL from its SPINE.  The wall profile is the SPRE catalogue
/// rectangle sized by DESP, shifted by the JUSL p-line and mirrored by LMIRR
/// ([`wall_section`]); the design SPINE supplies the path.  The first supported
/// path is E3D's common three-point circular spine (`POINSP, CURVE THRU, POINSP`).
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
    let travel = sweep.signum();
    let section = wall_section(el, thickness, height, notes)?;
    let (inner, outer) = circular_wall_radii(radius, section.x, travel);
    let section_height = section.y[1] - section.y[0];
    if !(inner >= 0.0 && section_height > f64::EPSILON && sweep.abs() > 1e-9) {
        return Err(ModelError::Solid(SolidError::Degenerate {
            what: "WALL circular spine",
            detail: format!(
                "radius = {radius}, section x {:?} y {:?}, sweep = {}deg",
                section.x,
                section.y,
                sweep.to_degrees()
            ),
        }));
    }
    // WALL is emitted as a baked FacetGroup. Core3D builds the curved path
    // through the profile tessellator (10 mm chord grid), rather than the
    // 0.5 mm analytic-primitive grid used by RTOR. RR001 WALL 2 therefore has
    // 15 equal sweep intervals and WALL 4 has three. The interval count follows
    // the section's outer radius, which is the spine only for walls on the
    // inner side (ams7997 `24381/1932`, `24381/4302` sit outside their spines).
    let sweep_deg = sweep.to_degrees();
    let sweep_segments = negative_part_rev_segments(outer, sweep_deg);
    let mut torus =
        rect_torus_solid_with_segments(inner, outer, section_height, sweep_deg, sweep_segments)?;
    let mid_height = 0.5 * (section.y[0] + section.y[1]);
    let local = DMat4::from_translation(DVec3::new(center.x, center.y, mid_height))
        * DMat4::from_rotation_z(a0);
    torus = placed(&torus, local);

    // 两端斜接：端面法向挂在 SPINE 的 `DRNS` / `DRNE` 上（Core3D `setMitrePlanes` 同一判定，
    // 见 [`crate::section::spine_mitre_plane`]）。端向缺席 / 与切向垂直或平行 → 径向平头；
    // 其余按那张平面切：截面内角落在体内一侧就削掉，落在体外一侧就先沿切向补挤一截再切
    // （`" Start-mitre extension"`）。
    //
    // 两件真机取证是同一条规则的两个符号：ams1112 `/1RS-WF03-W-C-RR001` WALL 4（3 个扫掠
    // 区间）SPINE `DRNS = E`，E3D 从起点外延 1.41° 到 `x = pt0.x` 那张竖直面——此前这里按
    // 「`sweep_segments == 3` 就补一个默认 Y 向的三角楔」硬编码；`/1RS-WF05-W-C-RR001`
    // WALL 4（`17496/107037`，9 个区间）SPINE 同样 `DRNS = E`，E3D 却把起点内角**削**到
    // `x = pt0.x`（比径向平头缩进 284 mm），旧的硬编码碰不到它。
    let local_rotation = DMat3::from_mat4(crate::transform::local_matrix(&spine)?);
    let spine_direction = |name: &str| -> DVec3 {
        match spine.get_direction(name) {
            Ok(Some(raw)) => local_rotation * DVec3::from_array(raw),
            _ => DVec3::ZERO,
        }
    };
    let profile_ring = section.ring();
    let mut cuts = Vec::new();
    for (label, name, angle, is_start) in [
        ("起点", "DRNS", a0, true),
        ("终点", "DRNE", a0 + sweep, false),
    ] {
        let drn = spine_direction(name);
        if drn.length_squared() <= 1e-18 {
            continue;
        }
        let radial = DVec3::new(angle.cos(), angle.sin(), 0.0);
        let ccw = DVec3::new(-angle.sin(), angle.cos(), 0.0);
        let point = center + radial * radius;
        let Some(plane) = spine_mitre_plane(drn, ccw * travel, point, is_start) else {
            notes.push(format!(
                "WALL {label}端 SPINE {name} {:?} 与切向垂直或平行：不斜接，径向平头",
                drn.to_array()
            ));
            continue;
        };
        let frame = wall_end_frame(point, angle, travel);
        let body_dir = if is_start { 1.0 } else { -1.0 };
        let rings = std::slice::from_ref(&profile_ring);
        let extended = mitre_extension_length(rings, frame, &plane, body_dir);
        if let Some(extension) = mitre_extension_solid(rings, frame, &plane, body_dir)? {
            torus = &torus + &extension;
        }
        notes.push(format!(
            "WALL {label}端按 SPINE {name} {:?} 斜切：过 ({:.3},{:.3}) 法向 {:?}，补挤 {extended:.3} mm",
            drn.to_array(),
            point.x,
            point.y,
            plane.normal.to_array()
        ));
        cuts.push((label, plane));
    }
    for (label, plane) in cuts {
        torus = crate::solid::cut_by_plane(&torus, plane.point, plane.normal);
        if torus.is_empty() {
            return Err(ModelError::Solid(SolidError::Degenerate {
                what: "WALL mitre",
                detail: format!(
                    "{label}端斜切面（法向 {:?}）把整堵墙切空",
                    plane.normal.to_array()
                ),
            }));
        }
    }
    notes.push(format!(
        "WALL circular SPINE center=({:.6},{:.6}) radius={radius:.6} sweep={:.6}deg",
        center.x,
        center.y,
        sweep.to_degrees()
    ));
    Ok(torus)
}

/// 圆弧墙的截面范围，在截面系里：x 朝行进方向的左手侧（俯视）、y 朝上，原点在 spine 上。
#[derive(Debug, Clone, Copy, PartialEq)]
struct WallSection {
    x: [f64; 2],
    y: [f64; 2],
}

impl WallSection {
    /// 截面矩形，逆时针（Positive 填充下算实心）。
    fn ring(&self) -> Vec<[f64; 2]> {
        let ([x0, x1], [y0, y1]) = (self.x, self.y);
        vec![[x0, y0], [x1, y0], [x1, y1], [x0, y1]]
    }

    fn mirrored(self) -> Self {
        Self {
            x: [-self.x[1], -self.x[0]],
            y: self.y,
        }
    }
}

/// 圆弧墙的截面：与 SCTN / GENSEC 同一条截面链（[`evaluate_section_profile`]），SPRE 目录截面
/// 减去 `JUSL` 那条 p-line 的 `(PX, PY)`，`LMIRR` 时再沿 x 镜像。
///
/// 语料里 62 堵圆弧墙的 SPRE 都是 `/Single_leaf_wall` 或 `/Variable_thk_wall`：DESP 参数化的
/// `[0, 厚] × [0, 高]` 矩形，`NA` 与 `OBOW` 在 x = 0，`IBOW` 在 x = 厚。没有 SPRE、链断或截面求不出
/// 时按 DESP 矩形、原点对齐（仍认 `LMIRR`），记 note。
fn wall_section(
    el: &DbElement,
    thickness: f64,
    height: f64,
    notes: &mut Vec<String>,
) -> Result<WallSection, ModelError> {
    let desp_rectangle = WallSection {
        x: [0.0, thickness],
        y: [0.0, height],
    };
    let fallback = if matches!(el.get_bool("LMIRR"), Ok(Some(true))) {
        desp_rectangle.mirrored()
    } else {
        desp_rectangle
    };
    let catalogue = match section_catalogue(el)? {
        SectionChain::Resolved(catalogue) => catalogue,
        SectionChain::NoSpre => {
            notes.push("WALL 无 SPRE：截面按 DESP 矩形、原点对齐".to_string());
            return Ok(fallback);
        }
        SectionChain::Broken { hop, at, why } => {
            notes.push(format!(
                "WALL SPRE 链第 {hop} 跳断在 {at}：{why}；截面按 DESP 矩形、原点对齐"
            ));
            return Ok(fallback);
        }
    };
    let profile = evaluate_section_profile(el, &catalogue)?;
    notes.extend(profile.notes.iter().cloned());
    match wall_section_from_profile(&profile) {
        Some(section) => {
            let rectangle = profile.members.len() == 1
                && profile.members[0].rings.len() == 1
                && profile.members[0].rings[0].len() == 4;
            if !rectangle {
                notes.push(format!(
                    "WALL 目录截面不是单个四点环（{} 个成员）：按外包矩形 x {:?} y {:?} 扫",
                    profile.members.len(),
                    section.x,
                    section.y
                ));
            }
            Ok(section)
        }
        None => {
            notes.push("WALL 目录截面没有顶点：截面按 DESP 矩形、原点对齐".to_string());
            Ok(fallback)
        }
    }
}

/// 目录截面的外包矩形，已减 `JUSL` 偏移、已按 `LMIRR` 镜像（点 x → `−(x − jx)`，见
/// [`crate::section::SectionProfile::mirrored`]）。一个顶点都没有时是 `None`。
fn wall_section_from_profile(profile: &crate::section::SectionProfile) -> Option<WallSection> {
    let [jx, jy] = profile.justification;
    let mut x = [f64::INFINITY, f64::NEG_INFINITY];
    let mut y = [f64::INFINITY, f64::NEG_INFINITY];
    for vertex in profile
        .members
        .iter()
        .flat_map(|member| member.rings.iter().flatten())
    {
        x = [x[0].min(vertex.x - jx), x[1].max(vertex.x - jx)];
        y = [y[0].min(vertex.y - jy), y[1].max(vertex.y - jy)];
    }
    if !(x[0] <= x[1] && y[0] <= y[1]) {
        return None;
    }
    let section = WallSection { x, y };
    Some(if profile.mirrored {
        section.mirrored()
    } else {
        section
    })
}

/// 圆弧墙的 `(内径, 外径)`。截面 x 朝行进方向的左手侧：逆时针扫掠左手是圆心那侧，
/// `[r − x₁, r − x₀]`；顺时针是外侧，`[r + x₀, r + x₁]`。
///
/// E3D TTY RVM 对拍过的四种组合：ams1112 的 14 堵逆时针 `NA`（内侧）；ams7997 `/1RB-CIVI`、
/// `/1RC-CIVI` 的 24 堵顺时针 `OBOW`（外侧）；`24381/1597` 顺时针 `NA` + `LMIRR`（内侧）；
/// `/1SL-WF01-W` 的 `24381/31100`、`24381/31106` 逆时针 `IBOW`（外侧 [28100, 28900]）。墙上 FIXING
/// 的 pline 基另有一套随行进方向与 `LMIRR` 的翻法（[`fixing_host_frame`]）。
fn circular_wall_radii(radius: f64, x: [f64; 2], travel: f64) -> (f64, f64) {
    if travel < 0.0 {
        (radius + x[0], radius + x[1])
    } else {
        (radius - x[1], radius - x[0])
    }
}

/// 圆弧墙斜接用的端标架，就是该端的截面系：X 朝行进方向的左手侧、Y 朝上、Z 沿行进方向，
/// 右手系；体在起点端沿 +Z、终点端沿 −Z。`travel` 是扫掠角的符号。
fn wall_end_frame(point: DVec3, angle: f64, travel: f64) -> DMat4 {
    let radial = DVec3::new(angle.cos(), angle.sin(), 0.0);
    let ccw = DVec3::new(-angle.sin(), angle.cos(), 0.0);
    DMat4::from_translation(point)
        * DMat4::from_mat3(DMat3::from_cols(-radial * travel, DVec3::Z, ccw * travel))
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

/// 一个 FIXING 的世界标架（**不含** FIXING 自己的 `local_matrix`）。
///
/// `world_matrix(fixing)` 对它是错的：链上 `JLDATU` 与 `PLDATU` 两级**都没有 POS/ORI**，
/// `local_matrix` 在它们身上返回单位阵，于是落点既丢了关节站位、也丢了 pline 基变换。
/// 这两级的标架是**隐式**的：
///
/// - JLDATU：位置由 `ZDIS` 沿宿主 WALL 的 spine 弧长定出来（`joint`）。
/// - PLDATU：`POSL=OBOW` 是 E3D 的隐式墙 pline（PLAX=-X），配 YDIR=+Z 由
///   `cal_ori_by_ydir` 得到列 (-Y, +Z, -X)（`pldatum`）。
///
/// 开孔刀（[`fixing_world_cutter`]）与 FIXING **自己那一件实例**必须共用这一处：
/// 两边各算一遍的那一版里，实例走的是 `world_matrix(fixing)`，1112 的 255 个 FIXING
/// 因此全挤在 x∈[-2520,-1920] 的一条窄带上，对着 E3D TTY 的 RVM 基准逐个偏 13~25 m
/// （`docs/evidence/2026-09-15-cata-dependency-cache.md`）。
///
/// 负扫掠墙（ams7997 `=24381/1776`，sweep≈−180°）的 pline 基两列随行进方向翻转
/// （见下面 `pldatum` 的构造）；翻转已经把 E3D 的镜像语义吸收进标架，FIXING 自己的
/// `ORI` 在 [`fixing_local_matrix`] 里原样读即可。
pub(crate) fn fixing_host_frame(wall: &DbElement, fixing: &DbElement) -> Result<DMat4, ModelError> {
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
    let (curve_center, radius, a0, sweep) = circular_spine_geometry(&[p0, pm, p1])?;
    // 站位半径取 spine 半径本身：ams7997 `=24381/1776` 的原生 RVM 里全部 PLDATUM
    // 原点都落在 R=16500 的 spine 圆上（6/6 逐件 0.00 mm）。曾按 `radius +
    // wall_profile_thickness` 加壁厚——那条链路读的是 SPRF 的 `PARA[0]`，对
    // `Single_leaf_wall` 拿回来的是名义参数 10000，站位被推出 10 m；ams1112
    // 当年能对上只是因为它的 `Variable_thk_wall` 没有 `PARA`、厚度回落成 0。
    let travel = if sweep < 0.0 { -1.0 } else { 1.0 };
    // LMIRR 墙（ams7997 `=24381/1597`，CWALL `/1RS-WF02-W-C-R2407`）：型材镜像后 PLDATU 绕沿墙轴
    // 转 180°——x、z 两列取负，y 列与原点不动。E3D TTY `ORI WRT WORLD`：镜像墙的 PLDATU
    // 在 JLDATU 系里是 (+J.Y, J.Z, +J.X)，正常墙是 (−J.Y, J.Z, −J.X)；不翻时这堵墙上的
    // FIXING 竖向按 POS.x 翻号、水平镜像（偏 1.2–3 m）。
    let mirror = if matches!(wall.get_bool("LMIRR"), Ok(Some(true))) {
        -1.0
    } else {
        1.0
    };
    let joint = wall_joint_frame(curve_center, radius, a0, sweep, zdis);
    // PLDATU POSL=OBOW is E3D's implicit wall pline: PLAX=-X. With
    // YDIR=+Z, cal_ori_by_ydir gives columns (-Y,+Z,-X).
    //
    // 负扫掠墙（travel=-1）上同一根 p-line 的 y/z 两列随行进方向翻转：
    // (-Y, -Z, +X)。ams7997 `=24381/1776` 的 6 只 FIXING 按此对原生 RVM
    // FacetGroup 变换逐件 0.00–0.03 mm；正扫掠（ams1112，sweep=+354.8°）
    // 时与旧版逐位一致，原有 38 件验证不受影响。
    //
    // OBOW（ams1112 WF03 `/1RS-WF03-W-C-RR001` 的 38 只穿墙套管 FIXING，WALL FUNC=RING）：
    // E3D TTY RVM 里 PLDATUM 的原点就是 JLDATUM 的站位（高差 0），FIXING 的 POS 三分量
    // 全部从那里量——38/38 满足 |POS| = |T_fixi − T_pldatu|、竖向 = −POS.x、水平模长相等，
    // 最大残差 0.001 mm（plant-model-5 `outputs/rvm-check-20261010/wall-1112/_hypothesis.py`）。
    // 下面那套 IBOW 抬高 + `fixing_local_matrix` 清 POS[1]/POS[2] 套在它身上，38 只全部
    // off 0.8–12.2 m。这里不抬高，POS 由 [`fixing_local_matrix`] 原样保留。
    let ibow_z = if pldatu_posl(fixing)?.as_deref() == Some("OBOW") {
        0.0
    } else {
        // Wall POSL=IBOW is the inner-bottom line, not the catalogue origin.  In
        // the AMS wall family the line sits one 600 mm fixing clearance below the
        // wall top (DESP[0]); leaving it at zero drops every wall FIXING by
        // `height - 600` (4000 -> 3400 mm in the real sample).
        let wall_height = wall
            .get_double_array("DESP")?
            .and_then(|values| values.first().copied())
            .filter(|value| value.is_finite() && *value > 0.0)
            .unwrap_or(0.0);
        // Door FIXINGs carry the opening height/width in a two-value DESP and
        // are centred in the wall: the live 1700x2045 door spans from the wall
        // base to 2045 mm.  Window FIXINGs use the standard IBOW datum instead.
        let door_fixing = fixing
            .get_double_array("DESP")?
            .is_some_and(|values| values.len() >= 2);
        // The standard wall fixing datum stays at the 3400 mm design level for
        // the shorter 3800 mm wall family as well: its generated fixing is 1100
        // mm above the wall base in both the 4000 and 3800 mm live samples.  Do
        // not lower the frame just because DESP reports the shorter wall height.
        if door_fixing {
            wall_height * 0.5
                + fixing
                    .get_double_array("DESP")?
                    .and_then(|values| values.get(1).copied())
                    .filter(|value| value.is_finite())
                    .unwrap_or(0.0)
        } else {
            (wall_height - 600.0).max(3400.0)
        }
    };
    let pldatum = DMat4::from_translation(DVec3::new(0.0, ibow_z, 0.0))
        * DMat4::from_cols(
            (DVec3::NEG_Y * mirror).extend(0.0),
            (DVec3::Z * travel).extend(0.0),
            (DVec3::NEG_X * (travel * mirror)).extend(0.0),
            DVec3::ZERO.extend(1.0),
        );
    Ok(world_matrix(wall)? * joint * pldatum)
}

/// JLDATU 在圆弧墙 spine 上的站位标架（WALL 局部系）：列 = (朝圆心, 上, 沿墙)，原点在站位。
///
/// `ZDIS` 是沿 spine 起点量的弧长。落在 spine 之内时站位在弧上、沿墙向取该点切向；
/// **越过两端时 E3D 不再沿弧走，而是从端点沿端切向直线外推**，沿墙向也钉在端切向上——
/// 取证 ams1112 `/1RS03LL0018T`（WF03 RR001 WALL 4，弧长 2378 mm，`ZDIS` 2440）：E3D TTY
/// `Q ORI` 给 JLDATU `Z is E 9.0681 S` = 墙**终点**切向（弧上 2440 处该是 E 8.87 S），
/// RVM 里那只 FIXING 的原点按直线外推算得 0.00 mm、按弧外推 0.11 mm、按弧上 2440 处的
/// 切向算 6.5 mm / 0.2°（FIXING 自己在 PLDATU 里还偏 1865 mm，杠杆把 0.2° 放大成 6.5 mm）。
fn wall_joint_frame(center: DVec3, radius: f64, a0: f64, sweep: f64, zdis: f64) -> DMat4 {
    wall_joint_frame_with_radii(center, radius, radius, a0, sweep, zdis)
}

fn wall_joint_frame_with_radii(
    center: DVec3,
    station_radius: f64,
    orientation_radius: f64,
    a0: f64,
    sweep: f64,
    zdis: f64,
) -> DMat4 {
    let length = orientation_radius * sweep.abs();
    let travel = if sweep < 0.0 { -1.0 } else { 1.0 };
    let point = |angle: f64| {
        center
            + DVec3::new(
                station_radius * angle.cos(),
                station_radius * angle.sin(),
                0.0,
            )
    };
    // 弧上 `angle` 处沿行进方向的单位切向。
    let tangent = |angle: f64| DVec3::new(-angle.sin(), angle.cos(), 0.0) * travel;
    let station_length = station_radius * sweep.abs();
    let (station_angle, station_overshoot) = if zdis < 0.0 {
        (a0, zdis)
    } else if zdis > station_length {
        (a0 + sweep, zdis - station_length)
    } else {
        (a0 + travel * zdis / station_radius, 0.0)
    };
    let direction_angle = if zdis < 0.0 {
        a0
    } else if zdis > length {
        a0 + sweep
    } else {
        a0 + travel * zdis / orientation_radius
    };
    let station = point(station_angle) + tangent(station_angle) * station_overshoot;
    let inward = (center - point(direction_angle)).normalize();
    let up = DVec3::Z;
    let along = inward.cross(up).normalize();
    DMat4::from_cols(
        inward.extend(0.0),
        up.extend(0.0),
        along.extend(0.0),
        station.extend(1.0),
    )
}

/// FIXING 的宿主 WALL。`collect_host_fixings` 是从 WALL 递归**下来**的、层数不固定
/// （FIXING 不一定恰好在 WALL → JLDATU → PLDATU 之下三层），所以反过来按 noun 上溯。
pub(crate) fn fixing_host_wall(fixing: &DbElement) -> Result<Option<DbElement>, ModelError> {
    let mut cursor = fixing.owner()?;
    let mut hops = 0;
    while let Some(current) = cursor {
        if current.element_type()? == "WALL" {
            return Ok(Some(current));
        }
        hops += 1;
        if hops > 8 {
            return Ok(None);
        }
        cursor = current.owner()?;
    }
    Ok(None)
}

/// FIXING 属主 PLDATU 的 `POSL`（大写、去空白）；属主不是 PLDATU 或没设返回 `None`。
///
/// 墙上 FIXING 的 PLDATU 标架按这条 p-line 分两套（见 [`fixing_host_frame`] /
/// [`fixing_local_matrix`]）：`OBOW` 原点在 JLDATU 站位、FIXING 的 POS 全量生效；
/// `IBOW`（aps250110 门窗）沿用 09-28 对 RVM 拟合的那套抬高 + 清 POS[1]/POS[2]。
fn pldatu_posl(fixing: &DbElement) -> Result<Option<String>, ModelError> {
    let Some(pldatu) = fixing.owner()? else {
        return Ok(None);
    };
    if pldatu.element_type()? != "PLDATU" {
        return Ok(None);
    }
    Ok(pldatu
        .get_string("POSL")?
        .map(|text| text.trim().to_ascii_uppercase())
        .filter(|text| !text.is_empty()))
}

/// WALL FIXING 的 `POS[1]`/`POS[2]` 是目录件的定位输入，不是目录几何
/// 实例的附加平移。Core3D 将它们留给墙体 p-line / SFIT 求值；若把完整
/// `local_matrix(FIXING)` 再乘到目录实体上，窗件会按 `POS[1]` 在墙向重复
/// 平移（APS250110 的 77.965/50.847/20.339 mm 三组 RVM 偏差正好相等）。
/// `POS[0]` 与 `ORI` 仍是实例标架的一部分；门的 `DESP[1]` 高程偏置在
/// `fixing_host_frame` 中消费，不能再把 `POS[1]` 当作第二次平移。
///
/// 以上只对 `POSL=IBOW` 那一族成立。`POSL=OBOW` 的穿墙套管（ams1112）E3D 把 POS 三分量
/// 全部用上（取证见 [`fixing_host_frame`]），这里原样返回 `local_matrix`。
///
/// `ORI` 在两条 p-line 族上都按原样 `Rz(γ)·Ry(β)·Rx(α)` 读——负扫掠墙的镜像差异
/// 已经由 [`fixing_host_frame`] 的 pline 基翻转吸收（ams7997 `=24381/1776` 6 只
/// FIXING 用原样 ORI 对原生 RVM FacetGroup 逐件 ~1e-6），局部矩阵这里不再补偿。
pub(crate) fn fixing_local_matrix(fixing: &DbElement) -> Result<DMat4, ModelError> {
    let local = crate::transform::local_matrix(fixing)?;
    if pldatu_posl(fixing)?.as_deref() == Some("OBOW") {
        return Ok(local);
    }
    Ok(DMat4::from_cols(
        local.x_axis,
        local.y_axis,
        local.z_axis,
        DVec3::new(local.w_axis.x, 0.0, 0.0).extend(1.0),
    ))
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
    let local_to_world = fixing_host_frame(wall, fixing)? * fixing_local_matrix(fixing)?;
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
    Ok(placed(&local, local_to_world))
}

/// 宿主墙名下的套管（`GWALL` → `CMPF` → `SBFI`，层数不固定，与
/// [`collect_host_fixings`] 同一种下钻）。
///
/// 只收带得起 `DESP` 的那些：套管本体与它在墙上开的洞出自同一份描述子，
/// 凑不齐 10 项的连套管自己都建不出来（[`sbfitting_from_desp`]）。
fn collect_host_sleeves(el: &DbElement, out: &mut Vec<DbElement>) -> Result<(), DbElementError> {
    for member in el.members()? {
        if member.element_type()? == "SBFI" {
            if member
                .get_double_array("DESP")?
                .is_some_and(|desp| desp.len() >= SBFI_DESP_MIN)
            {
                out.push(member);
            }
        } else {
            collect_host_sleeves(&member, out)?;
        }
    }
    Ok(())
}

/// 一根套管在宿主墙上开的洞（世界系）。
///
/// 与 `FIXING` 那条（[`fixing_world_cutter`]）同型、同一份 `DESP` 布局，只是标架不同：
/// `FIXING` 的属主链上 `JLDATU`/`PLDATU` 两级没有 POS/ORI、要现算
/// （[`fixing_host_frame`]），套管的属主 `CMPF` 是有位置的普通元素，
/// 直接与套管实例共用 [`sbfitting_world`]。
///
/// 这条路 2026-09-15 才接上。在那之前 `GWALL` 名下一个洞都不开：ams1112 的 200 根套管
/// （宿主全是 `GWALL`，落在 54 堵墙上）在 RVM 对拍上表现为「某几堵墙 Hausdorff 几百毫米」，
/// 量级正是半个墙厚——洞的内壁在我们这边是实心，最近的表面只好退到墙的外表面
/// （`docs/evidence/2026-09-15-cata-dependency-cache.md`）。
fn sleeve_world_cutter(sleeve: &DbElement) -> Result<Manifold, ModelError> {
    let desp = sleeve
        .get_double_array("DESP")?
        .ok_or_else(|| ModelError::MissingAttribute {
            refno: sleeve.refno().to_string(),
            noun: "SBFI".into(),
            name: "DESP",
            what: "host bore",
        })?;
    let local = sbfitting_bore_from_desp(&desp).map_err(ModelError::Solid)?;
    Ok(placed(&local, sbfitting_world(sleeve)?))
}

/// 套管那件实例的世界标架。开孔刀（[`sleeve_world_cutter`]）与实例必须共用这一处：
/// 两边各算一遍，洞与套管就会错位——`FIXING` 在 2026-09-15 刚栽过这一跤
/// （实例走 `world_matrix`、开孔刀走 `fixing_host_frame`，255 件偏 13~25 m）。
fn sbfitting_world(el: &DbElement) -> Result<DMat4, ModelError> {
    Ok(world_matrix(el)? * sbfitting_catalogue_alignment(el)?)
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
/// cylinders used by the legacy generator.  The descriptor layout is
/// `[ID, OD, THK, LENGTH, EXT_DATUM, EXT_FAR, .., X, Y, Z]` (millimetres); the
/// sleeve's `LENGTH` ends at its local datum `Z` and each end is lengthened by its
/// extension, so it occupies `Z-LENGTH-EXT_FAR .. Z+EXT_DATUM`
/// ([`sbfitting_axial_span`]).
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
/// transverse axes with Core3D's `ORIMAT` rule.
fn sbfitting_catalogue_alignment(el: &DbElement) -> Result<DMat4, ModelError> {
    let raw = el.get_direction("ZDIR")?.unwrap_or([0.0, 0.0, 1.0]);
    sbfitting_alignment_from_zdir(raw)
}

fn sbfitting_alignment_from_zdir(raw: [f64; 3]) -> Result<DMat4, ModelError> {
    // Descriptor direction triples use E3D's bearing handedness; the second
    // component is opposite to the right-handed design-frame Y used by glam.
    let z = DVec3::new(raw[0], -raw[1], raw[2]).normalize_or_zero();
    if z.length_squared() <= f64::EPSILON {
        return Err(ModelError::Solid(SolidError::Degenerate {
            what: "SBFI catalogue alignment",
            detail: format!("ZDIR = {raw:?}"),
        }));
    }
    // core.dll!ORIMAT (0x526D674) tries VUNIT(1..3), takes
    // X = normalize(Z cross unit), then Y = Z cross X. VUNIT(1) is north/Y;
    // when Z is parallel to it the next candidate is up/Z, then east/X.
    // Choosing projected global Z unconditionally loses both the hemisphere
    // sign and the cardinal-direction fallback, moving off-axis DESP sleeves
    // to the opposite side of their host wall.
    let candidate = [DVec3::Y, DVec3::Z, DVec3::X]
        .into_iter()
        .find(|candidate| z.dot(*candidate).abs() < 0.999_999)
        .expect("a finite unit vector cannot be parallel to all basis axes");
    let x = z.cross(candidate).normalize();
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

    let (bottom, span) = sbfitting_axial_span(desp);
    let solid = extrude_rings(
        vec![
            sbfitting_ring(outer_diam * 0.5, false),
            sbfitting_ring(inner_diam * 0.5, true),
        ],
        span,
    )?;
    Ok(solid.translate(desp[7], desp[8], bottom))
}

/// 套管沿局部 Z 的 `(底, 长)`：本体 `LENGTH` 止于基准 `Z`，`DESP[4]` 接在基准那头、`DESP[5]` 接在
/// 另一头。
///
/// 目录件的写法（SFIT `/GW-HOLE` 的 GMSE 13244/695337，DESP 下标从 1 起）：两只 LCYL 从中点各朝
/// 一头长出去，`PTDI` 分别是 `DESP[5] + DESP[4]/2`（PAXI 1001，朝基准）与 `DESP[6] + DESP[4]/2`
/// （PAXI 1002）。ams1112 `/1RS03CC0028T` 的 SBFI（`DESP[4]=50`）在 E3D TTY RVM 里是 z ∈ [−1600, 50]；
/// 同库另外 136 只两项都是 0。
fn sbfitting_axial_span(desp: &[f64]) -> (f64, f64) {
    let (length, past_datum, past_far_end) = (desp[3], desp[4], desp[5]);
    (
        desp[9] - length - past_far_end,
        length + past_datum + past_far_end,
    )
}

/// The reference RVM uses eight circumferential panels for this catalogue
/// fitting.  Keeping that tessellation also makes topology comparisons
/// deterministic instead of merely matching the bounding box.
const SBFI_SEGMENTS: usize = 8;

fn sbfitting_ring(radius: f64, reverse: bool) -> Vec<[f64; 2]> {
    let mut points = (0..SBFI_SEGMENTS)
        .map(|i| {
            let angle = std::f64::consts::TAU * i as f64 / SBFI_SEGMENTS as f64;
            [radius * angle.cos(), radius * angle.sin()]
        })
        .collect::<Vec<_>>();
    if reverse {
        points.reverse();
    }
    points
}

/// 套管在宿主墙上开的那个洞：与 [`sbfitting_from_desp`] 同一份 `DESP`、同一组顶点、
/// 同一处平移，只是把环换成 `OD` 实心柱。
///
/// 判据是 ams1112 `1RS-WF03-W-C-RR001` 的逐点实测：RVM 里那些洞的内壁是**内接**于
/// `OD/2` 的八边形（采样点径向距离落在边心距 `62·cos22.5°=57.3` 到顶点 `62` 之间），
/// 洞的 z 跨度与 `DESP[1]` 逐字相同（124 mm 那只）。所以洞不是「OD 加余量」，
/// 就是套管外圆本身——两者必须同一组顶点，不能一边圆一边多边形。
fn sbfitting_bore_from_desp(desp: &[f64]) -> Result<Manifold, SolidError> {
    if desp.len() < SBFI_DESP_MIN {
        return Err(SolidError::Degenerate {
            what: SBFI_WHAT,
            detail: format!("DESP length {} < {SBFI_DESP_MIN}", desp.len()),
        });
    }
    let (outer_diam, length) = (desp[1], desp[3]);
    if !(outer_diam > 0.0 && length > f64::EPSILON) {
        return Err(SolidError::Degenerate {
            what: SBFI_WHAT,
            detail: format!("OD/LENGTH = {outer_diam}/{length}"),
        });
    }
    let (bottom, span) = sbfitting_axial_span(desp);
    let bore = extrude_rings(vec![sbfitting_ring(outer_diam * 0.5, false)], span)?;
    Ok(bore.translate(desp[7], desp[8], bottom))
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
                // A standalone design CYLI uses the same centered Z frame as
                // the Cylinder RVM record (AMS 8000, 24384/26294). The legacy
                // gm_CreateCylinder helper instead starts at the local Y base.
                Ok(instantiate_centered_cylinder(d, h)?)
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
                    analytic: None,
                })
            }
        }
        Prim::Cone => Ok(PrimitiveBuild {
            solid: design_snout_solid(
                require_double(el, noun, "DBOT", "cone")?,
                require_double(el, noun, "DTOP", "cone")?,
                require_double(el, noun, "HEIG", "cone")?,
                0.0,
                0.0,
                !allow_instance,
            )?,
            instance: None,
            analytic: None,
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
                analytic: None,
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
                analytic: None,
            })
        }
        Prim::CircTorus => {
            let rout = require_double(el, noun, "ROUT", "circular torus")?;
            let angle = require_double(el, noun, "ANGL", "circular torus")?;
            Ok(PrimitiveBuild {
                solid: circ_torus_solid(default_zero(el, "RINS")?, rout, angle)?,
                instance: None,
                analytic: None,
            })
        }
        Prim::Snout => Ok(PrimitiveBuild {
            solid: design_snout_solid(
                require_double(el, noun, "DBOT", "snout")?,
                require_double(el, noun, "DTOP", "snout")?,
                require_double(el, noun, "HEIG", "snout")?,
                default_zero(el, "XOFF")?,
                default_zero(el, "YOFF")?,
                !allow_instance,
            )?,
            instance: None,
            analytic: None,
        }),
        Prim::Dish => Ok(PrimitiveBuild {
            solid: dish_local_solid(
                require_double(el, noun, "DIAM", "dish")?,
                require_double(el, noun, "HEIG", "dish")?,
                default_zero(el, "RADI")?,
                !allow_instance,
            )?,
            instance: None,
            analytic: None,
        }),
        Prim::SlopedCylinder => {
            let radius = require_double(el, noun, "DIAM", "slope-ended cylinder")? * 0.5;
            let height = require_double(el, noun, "HEIG", "slope-ended cylinder")?;
            let xtop_deg = fold_shear_angle(default_zero(el, "XTSH")?);
            let ytop_deg = fold_shear_angle(default_zero(el, "YTSH")?);
            let xbot_deg = fold_shear_angle(default_zero(el, "XBSH")?);
            let ybot_deg = fold_shear_angle(default_zero(el, "YBSH")?);
            let segments = if allow_instance {
                cylinder_segments(radius)
            } else {
                negative_cylinder_segments(radius)
            };
            Ok(PrimitiveBuild {
                solid: slope_ended_cylinder_solid_with_segments(
                    radius, height, xtop_deg, ytop_deg, xbot_deg, ybot_deg, segments,
                )?,
                instance: None,
                // 独立正体才带解析身份：进了布尔的 E3D 只会写面片，参数也没处用。
                analytic: allow_instance.then_some(AnalyticPrimitive::SlopedCylinder {
                    radius,
                    height,
                    xtop_deg,
                    ytop_deg,
                    xbot_deg,
                    ybot_deg,
                    segments,
                }),
            })
        }
    }
}

/// Standalone AMS 8000 CONE/SNOU export as analytic Snout records. Boolean
/// results instead bake a 10 mm circle grid (250 mm radius: 12 vertices).
fn design_snout_solid(
    dbot: f64,
    dtop: f64,
    height: f64,
    xoff: f64,
    yoff: f64,
    boolean_operand: bool,
) -> Result<Manifold, SolidError> {
    let radius = dbot.max(dtop) * 0.5;
    let segments = if boolean_operand {
        negative_cylinder_segments(radius)
    } else {
        cylinder_segments(radius)
    };
    snout_solid_with_segments(dbot, dtop, height, xoff, yoff, segments)
}

/// 设计侧 DISH 的三路（`CSG_BasicDIS::getPrimGeom@0x10726d10`，旧路 `CRDESI@0x10343b80`
/// case 703003/912101 同一分支）：`HEIG <= 0` 退成 1 mm 厚的圆盘 `gm_CreateCylinder(DIAM/2, 1.0)`；
/// `RADI > 0` 走折边碟 `gm_CreateEllipticalDish(r, h, k)`，折边半径 `k` 由 CREDSH 的公式
/// 从 r/h 算出（[`dish_knuckle_radius`]），RADI 的数值本身不进几何；否则球形碟
/// `gm_CreateSphericalDish(r, h)`。与目录 SDSH 臂（`catalogue::evaluate_sdsh`）同一组
/// libgm 原语，差别只在属性来源。
///
/// 局部帧照 libgm：底圆在 XY 平面、顶点在 +Z，**不做** CYLI 那一转——ams7997
/// `/Copy-of-1TFM020MN冷凝罐` 两只 DISH（24381/107780、107781）的 RVM 记录帧与元素世界帧
/// 逐列相同。退化圆盘走 libgm 圆柱的居中 Z 帧（[`csg_cylinder_solid_with_segments`]）。
///
/// `boolean_operand`：带负体的正体（与所有负体自身）按 E3D 最终 LNEGIT FacetGroup 的粗口径
/// （[`negative_cylinder_segments`]）画圆周，与刀同网格——理由与 `Prim::Cylinder` 那支相同，
/// 实测见 [`spherical_dish_solid_with_segments`]；独立正体走 libgm 0.5 mm 曲面口径。
fn dish_local_solid(
    diam: f64,
    heig: f64,
    radi: f64,
    boolean_operand: bool,
) -> Result<Manifold, SolidError> {
    let radius = diam * 0.5;
    let segments = boolean_operand.then(|| negative_cylinder_segments(radius));
    if !(heig > 0.0) {
        return csg_cylinder_solid_with_segments(
            diam,
            1.0,
            segments.unwrap_or_else(|| cylinder_segments(radius)),
        );
    }
    if radi > 0.0 {
        elliptical_dish_solid_with_segments(
            radius,
            heig,
            dish_knuckle_radius(radius, heig),
            segments,
        )
    } else {
        spherical_dish_solid_with_segments(radius, heig, segments)
    }
}

/// SLCY 的四个斜切角在建体前各折回一次到 (−90°, 90°]：`> 90 → −180`、`< −90 → +180`
/// （`CSG_BasicSLC::getPrimGeom@0x107272d0`，旧路 `CRDESI` 1026046/599765 臂逐字相同）。
/// 折完仍落在 ±90° 上的，libgm 交空面片组，[`slope_ended_cylinder_solid`] 报退化。
fn fold_shear_angle(deg: f64) -> f64 {
    let mut a = deg;
    if a > 90.0 {
        a -= 180.0;
    }
    if a < -90.0 {
        a += 180.0;
    }
    a
}

/// 多面体局部实体（`CSG_BasicPOL::getPrimGeom` @0x10726f10 → `sub_1071DCF0`，
/// 7UW4 §3.7.4）：面片采集（两变体） → 带孔三角化 → 绕向一致化 → 流形校验。
/// 退化面记 `notes`；壳不闭合按数据坏上抛（= `gm_ValidateObject` 不过即弃的口径）。
fn polyhedron_local_solid(el: &DbElement, notes: &mut Vec<String>) -> Result<Manifold, ModelError> {
    let faces = polyhedron_faces(el)?;
    if el.element_type()? == "POHE" {
        return Ok(legacy_polygon_panels(&faces, notes)?);
    }
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
    // Both REVO and NREV are baked profile FacetGroups in AMS 8000:
    // R=500, ANGL=180 gives eight sweep intervals, including after edits.
    Ok(revolve_rings_profile_faceted_inflated(rings, angle, grow)?)
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
/// 也就是说这些负体扣的是**容器之外的宿主**（墙 / 梁 / 被连接的构件）。宿主的判据
/// 目前只接了一条：FIXING 是某根 GENSEC 的 `JOIS` / `JOIE` 关节时，梁从另一头把它们
/// 取走（[`collect_joint_negatives`]）。其余宿主（FIXING 自己的属主、墙上凹龛那类）
/// 还没有，管线照旧把 TMPL 名下的负体记进 `Report::host_negatives`。
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

/// GENSEC / SCTN 两端的关节负体：`JOIS` / `JOIE` 指向的关节元素给这根构件的刀
/// （[`fixing_negatives`]）。
///
/// 对标 `DB_Sctn::DB_makeCSG@0x10738C30`：截面实体切完两端之后，对 JOIS、JOIE 的目标各调
/// 一次 `sub_10722D80(..., mode 0)`，wrapper 把 mode 0 改写成负体 mode 2
/// （`2 − (mode≠0)`，recheck-2 §六锚点表）——SCTN 的关节负体过 `NAPP & 2` 门。
/// GENSEC 侧的同一调用留在 [`JOINT_MODE`]（mode 0，`NAPP & 1`）——那是 `aps250110`
/// 逐件 TTY 验过的口径（檐口 JOIN 的 NSEX `NAPP 1` 只有 mode 0 放行），不动它。
///
/// `aps250110` 上 1505 根 GENSEC 的 522 个非空关节全部指向 FIXING，FIXING 的 `CREF` 全部回指
/// 这根梁：
///
/// - 带 TMPL 的 229 个是「Flexible / Full Depth End Plate」与「Web Cleats」三种节点模板，负体是
///   NBOX（NAPP -1，ZLEN 8 的端板让位）、NCYL（NAPP 7，螺栓孔）与少数 NXTR（NAPP -1，翼缘切角）
///   ——端板那一刀就是梁对 E3D TTY RVM 两端各长 8 mm 的来源。
/// - `SPRE → CATR` 是目录 JOIN 的那批（檐口加腋节点）负体在 `NGMR` 里：NSEX（NAPP 1）按连接对象
///   的翼缘宽（`APAR[2]`）与设计表的 `BONE` / `BTWO` 围出截面，挤 `PARA[1] + 1` = 16 mm——斜梁
///   檐口端对 E3D 长出的就是这 16 mm；NSCY（NAPP 6，螺栓孔）在 mode 0 不放行。
///
/// SCTN 的 JOIS/JOIE 指向的是 `SJOI`（aps250110 实测 `SCTN.JOIE → SJOI`，SJOI 的 `CREF`
/// 回指被切的 SCTN；aps7200 一窗 94 只 SJOI）。SJOI 多数没有 TMPL 成员、`SPRE` 也空——按
/// `sub_10722680` 的几何源口径（TMPL 成员 / SPRE→CATR）它产不出负体。**只有真没有几何源
/// 才记 skipped**：有源的关节无论刀被 NAPP 门挡下、全部放行还是源里本就没有负体，都不算
/// skipped（见 [`JointCutOutcome`]）。无源条目按 [`JointClass`] 分型：J-A（跨截面 SNOD-SJOI）
/// 已有原生证据（§12①）移进 `explained` 账，其余类别留 skipped 待证。
///
/// 关节的 `CUTP` 并不经这里生效：Core3D `RECON0 → STRCNT → JTCONN → PUTEND` 把关节端切写回
/// SCTN 自己的 `POSS/POSE/DRNS/DRNE`，正体由 [`crate::section::section_cut_planes`] 照这四项裁两端；
/// `CUTB` 在 `JTCONN` 里读到但未见参与写回。这一行 skipped 只说明「这把刀源不存在」，
/// 不等于「缺一刀」——是否另有原生额外端切，待同根 RECONNECT/SAVEWORK 前后对拍（#34 D1–D3）。
///
/// `SUBJ` 关节（`DB_makeCSG` 连它的属主一起扫）与 FIXING 套 FIXING（`DB_Gensec::makeCSG` 沿关节
/// 目标的属主链往上、只要还是 FIXING 就接着扫）语料里都没有，没接。
///
/// 关节读不出来（悬空引用、成员表读挂）不拖垮构件：记进 `skipped`，构件照出。
fn collect_joint_negatives(
    el: &DbElement,
    noun: &str,
    negatives: &mut Vec<DbElement>,
    catalogue_cuts: &mut Vec<Manifold>,
    skipped: &mut Vec<SkippedNegative>,
    explained: &mut Vec<SkippedNegative>,
    notes: &mut Vec<String>,
    cache: &mut CatalogueMeshCache,
) -> Result<(), ModelError> {
    // SCTN 的 JOIS/JOIE 经 `sub_10722D80(mode 0)` 落到负体 mode 2；GENSEC 留在 mode 0。
    let joint_mode = if noun == "SCTN" {
        SCTN_JOINT_MODE
    } else {
        JOINT_MODE
    };
    for (attr, joint) in [
        ("JOIS", el.get_element("JOIS")?),
        ("JOIE", el.get_element("JOIE")?),
    ] {
        let Some(joint) = joint else {
            continue;
        };
        let joint_noun = joint.element_type()?;
        // GENSEC 的关节集保持 `FIXING` 单值（语料实测如此）；SCTN 的 JOIS/JOIE 实测指 SJOI，
        // SUBJ / PJOI / SCOJ / PCOJ / FIXING 同属一族一起放行——没有几何源的由
        // `fixing_negatives` 自然给零并落下面的 skipped 记账。
        let accepted = match noun {
            "GENSEC" => joint_noun == "FIXING",
            _ => matches!(
                joint_noun.as_str(),
                "FIXING" | "SJOI" | "PJOI" | "SCOJ" | "PCOJ" | "SUBJ"
            ),
        };
        if !accepted {
            continue;
        }
        match fixing_negatives(&joint, joint_mode, negatives, catalogue_cuts, notes, cache) {
            Ok(tally) if tally.rejected > 0 => {
                notes.push(format!(
                    "{attr} 关节 {}：NAPP 门在关节路径（mode {joint_mode}）挡下 {} 把刀，E3D 同样不拿它们切构件",
                    joint.refno(),
                    tally.rejected
                ));
            }
            Ok(tally) => match JointCutOutcome::of(tally) {
                JointCutOutcome::NoSource => {
                    if noun == "SCTN" {
                        match joint_class(el, &joint) {
                            class @ JointClass::AcrossSnod => explained.push(SkippedNegative {
                                refno: joint.refno(),
                                noun: joint_noun.clone(),
                                reason: format!(
                                    "[{}] {attr} 关节没有几何源（TMPL 成员与 SPRE→CATR 都没有），不出负体：\
                                     关节的 CUTP 端切已由 SCTN 正体的 DRNS/DRNE 端面覆盖——同拓扑原生样本\
                                     250110 /68 与生成体双向最大差 0.007071 mm、/70 原生 RECONNECT END 幂等\
                                     （gen-model #34 §12①）；caveat：/68 自身 RECONNECT 被 core.dll 崩溃挡住，\
                                     扰动后是否回写未测，登记于「已解释」而非消除",
                                    class.label()
                                ),
                            }),
                            class => skipped.push(SkippedNegative {
                                refno: joint.refno(),
                                noun: joint_noun.clone(),
                                reason: format!(
                                    "[{}] {attr} 关节没有几何源（TMPL 成员与 SPRE→CATR 都没有），不出负体；\
                                     CUTP 端切若经 RECONNECT 写回 SCTN 的 DRNS/DRNE 则已由端面覆盖，\
                                     是否另有原生额外刀待同根前后对拍（#34 D1–D3）",
                                    class.label()
                                ),
                            }),
                        }
                    }
                }
                JointCutOutcome::SourceWithoutCuts => notes.push(format!(
                    "{attr} 关节 {}：有几何源但其中没有负体，不出刀",
                    joint.refno()
                )),
                JointCutOutcome::Cut => {}
            },
            Err(error) => skipped.push(SkippedNegative {
                refno: joint.refno(),
                noun: joint_noun,
                reason: format!("{attr} 关节读不出来，端头负体没扣：{error}"),
            }),
        }
    }
    Ok(())
}

/// SCTN 的 JOIS / JOIE 关节在 `sub_10722D80(mode 0)` 里被改写成负体 mode 2（`NAPP & 2` 门）。
const SCTN_JOINT_MODE: u32 = 2;

/// SCTN 成员链上的关节 / 配件负体（`DB_Sctn::DB_makeCSG` 的 `sub_10721600(mode 3)` 扫描，
/// recheck-2 §2.3 / recheck-3 N1-3）：
///
/// - 成员 `FITT` / `CMFI` → mode 3；`CMFI` 名下的 `SBFI` → mode 6；
/// - 成员 `SNOD` 名下的 `SJOI` → mode 1；
/// - 成员 `SCOJ` → mode 2，其名下 `SUBJ` → mode 1。
///
/// 几何源统一走 `sub_10722680` 同款解析（TMPL 成员 → 设计负体；`SPRE→CATR` → 目录负体），
/// NAPP 门按各 mode 取位——`NAPP = −1` 只放 mode 3–6（`design_napp_admits` 的 `0|3|4|5|6|9`
/// 与本路径的 mode 集 {1,2,3,6} 取交正好是 {3,6}）。`FITT` 成员下的负体定位吃
/// `ImplicitFrame::SectionFitting`（`transform.rs` 已接）；`SNOD` 链下的关节元素若真解出
/// 负体，标架吃 `ImplicitFrame::SectionNode` 的暂定实现（GJTORI/GJTORG 内部未翻，RVM 校）。
///
/// 成员读挂只记 `skipped`，构件照出。
fn collect_member_joint_negatives(
    el: &DbElement,
    negatives: &mut Vec<DbElement>,
    catalogue_cuts: &mut Vec<Manifold>,
    skipped: &mut Vec<SkippedNegative>,
    explained: &mut Vec<SkippedNegative>,
    notes: &mut Vec<String>,
    cache: &mut CatalogueMeshCache,
) -> Result<(), ModelError> {
    let mut feed = |source: &DbElement, mode: u32, via: &str| match fixing_negatives(
        source,
        mode,
        negatives,
        catalogue_cuts,
        notes,
        cache,
    ) {
        Ok(tally) if tally.rejected > 0 => notes.push(format!(
            "{via} {}：NAPP 门（mode {mode}）挡下 {} 把刀，E3D 同样不切",
            source.refno(),
            tally.rejected
        )),
        Ok(tally) => match JointCutOutcome::of(tally) {
            JointCutOutcome::NoSource => {
                let class =
                    member_chain_class(&source.element_type().unwrap_or_else(|_| "?".into()));
                let record = |reason: String| SkippedNegative {
                    refno: source.refno(),
                    noun: source.element_type().unwrap_or_else(|_| "?".into()),
                    reason,
                };
                match class {
                    // J-B：宿主自己 SNOD 下的无源 SJOI。原生证据：同一 SCTN 的
                    // `export` 子树里 SJOINT 记录零图元（SNODE 只出 3 条 Line），
                    // 宿主正体与生成体双向采样最大差 0.0063 mm、体积相对差 2.3e-6
                    // （250110 /78、/84 两样本，sctn_250110_jb-20261018）——嵌套关节
                    // 既不产出也不吃掉几何。
                    JointClass::OwnSnod => explained.push(record(format!(
                        "[{}] {via} 没有几何源（TMPL 成员与 SPRE→CATR 都没有），不出负体：\
                         同拓扑原生样本 250110 /78、/84 的 SJOINT 记录零图元、宿主正体双向\
                         最大差 0.0063 mm、体积差 2.3e-6（gen-model #34 sctn_250110_jb-20261018）；\
                         caveat：样本只在 250110 墙框一族，其它族嵌套关节先按同类记账",
                        class.label()
                    ))),
                    _ => skipped.push(record(format!(
                        "[{}] {via} 没有几何源（TMPL 成员与 SPRE→CATR 都没有），不出负体；\
                         是否另有原生额外刀待同根前后对拍",
                        class.label()
                    ))),
                }
            }
            JointCutOutcome::SourceWithoutCuts => notes.push(format!(
                "{via} {}：有几何源但其中没有负体，不出刀",
                source.refno()
            )),
            JointCutOutcome::Cut => {}
        },
        Err(error) => skipped.push(SkippedNegative {
            refno: source.refno(),
            noun: source.element_type().unwrap_or_else(|_| "?".into()),
            reason: format!("{via} 负体读不出来：{error}"),
        }),
    };
    for member in el.members()? {
        let member_noun = member.element_type()?;
        if let Some(mode) = member_chain_mode(&member_noun, None) {
            feed(&member, mode, "成员配件");
        }
        if matches!(member_noun.as_str(), "CMFI" | "SNOD" | "SCOJ") {
            for sub in member.members()? {
                let sub_noun = sub.element_type()?;
                if let Some(mode) = member_chain_mode(&member_noun, Some(&sub_noun)) {
                    feed(&sub, mode, "成员链下嵌套关节");
                }
            }
        }
    }
    Ok(())
}

/// `sub_10721600(mode 3)` 的成员链扫描表：`FITT` / `CMFI` 直接成 mode 3；嵌套第二层里
/// `CMFI` 名下的 `SBFI` 是 mode 6、`SNOD` 名下的 `SJOI` 是 mode 1、`SCOJ` 本身是 mode 2、
/// 其名下 `SUBJ` 是 mode 1。表外的成员 / 嵌套组合不参与这条扫描（返回 `None`）。
fn member_chain_mode(member_noun: &str, sub_noun: Option<&str>) -> Option<u32> {
    match (member_noun, sub_noun) {
        ("FITT" | "CMFI", None) => Some(3),
        ("CMFI", Some("SBFI")) => Some(6),
        ("SNOD", Some("SJOI")) => Some(1),
        ("SCOJ", None) => Some(2),
        ("SCOJ", Some("SUBJ")) => Some(1),
        _ => None,
    }
}

/// SCTN 属主是 `TMPL` 时的模板负体（`DB_Sctn::DB_makeCSG`：`owner==TMPL →
/// sub_10722D80(mode 7)`，`NAPP & 4` 门；`TMPL.owner==FIXING → sub_10721930` 的
/// FIXING 扫描臂——后者把模板所属的 FIXING 当宿主刀源，沿用宿主扫描的
/// SFIT?3:1 改写）。
///
/// aps250110 / aps7200 的 SCTN 大量挂在 `TMPL ← FITT ← STWALL` 组合件里（实测
/// `2013286654/68 SCTN ← TMPL/25148 ← FITT/56`）：模板里的负体（端板让位、连接
/// 件切口）按 mode 7 扣到截面件上。TMPL 的负体成员是设计负体，走
/// [`design_napp_admits`]。
fn collect_owner_template_negatives(
    el: &DbElement,
    negatives: &mut Vec<DbElement>,
    catalogue_cuts: &mut Vec<Manifold>,
    skipped: &mut Vec<SkippedNegative>,
    notes: &mut Vec<String>,
    cache: &mut CatalogueMeshCache,
) -> Result<(), ModelError> {
    let Some(owner) = el.owner()? else {
        return Ok(());
    };
    if owner.element_type()? != "TMPL" {
        return Ok(());
    }
    if let Some(grand) = owner.owner()?
        && grand.element_type()?.as_str() == "FIXING"
    {
        match fixing_negatives(
            &grand,
            HOST_SCAN_MODE,
            negatives,
            catalogue_cuts,
            notes,
            cache,
        ) {
            Ok(tally) => {
                if tally.rejected > 0 {
                    notes.push(format!(
                        "属主链 TMPL←FIXING {}：NAPP 门挡下 {} 把刀",
                        grand.refno(),
                        tally.rejected
                    ));
                }
            }
            Err(error) => skipped.push(SkippedNegative {
                refno: grand.refno(),
                noun: "FIXING".into(),
                reason: format!("属主链 TMPL←FIXING 扫描读不出负体：{error}"),
            }),
        }
        return Ok(());
    }
    let mut template_negatives = Vec::new();
    collect_negatives(&owner, &mut template_negatives)?;
    let before = template_negatives.len();
    template_negatives.retain(|neg| design_napp_admits(7, negative_napp(neg)));
    let rejected = before - template_negatives.len();
    if rejected > 0 {
        notes.push(format!(
            "属主 TMPL {}：NAPP 门（mode 7，NAPP & 4）挡下 {rejected} 把刀，E3D 同样不切",
            owner.refno()
        ));
    }
    negatives.extend(template_negatives);
    Ok(())
}

/// FIXING 的 `AANGXZ` / `AANGYZ`（度），见 [`attachment_angles`]。
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct AttachmentAngles {
    pub xz: f64,
    pub yz: f64,
}

/// Core3D `pllib/PATANG` 现算的两个伪属性：FIXING 连着的截面（`CREF`，它的 JOIS 或 JOIE 得指回这只
/// FIXING）在关节那一端、指向截面里面的轴向，换到 FIXING 的标架里，投到 YZ / XZ 平面上从 Z 轴量起的
/// 转角。`AANGYZ` 是绕 X 的右手转角（Z 转向 −Y 为正），`AANGXZ` 绕 Y（Z 转向 +X 为正）。
///
/// 目录的檐口加腋节点 JOIN `/J2S4_EAV_PFI_PFI_003` 拿它定斜梁的斜度：设计表 `ANYZ = -AANGYZ`，再推出
/// `BLEN = APAR[1] / COS(ANYZ)`、`OPYZ = ADJA * TAN(ANYZ)`，刀与加腋板的尺寸都吃它。E3D TTY 在 aps250110
/// 的四只檐口 FIXING 上 `Q AANGYZ` 都是 -14.481°、`Q AANGXZ` 是 0°（斜梁 1000 / 3872 的坡）。
///
/// 接了 GENSEC（轴向取 SPINE 在那一端的切向）与 SCTN（直段，`unit(POSE − POSS)` 端切向，
/// 起端正向 / 末端反向）。`CREF` 空、不回指、截面缺 SPINE 或 POSS/POSE 退化都不答——
/// 表达式照旧按未解报。
pub(crate) fn attachment_angles(
    fixing: &DbElement,
) -> Result<Option<AttachmentAngles>, ModelError> {
    if fixing.element_type()? != "FIXING" {
        return Ok(None);
    }
    let Some(section) = fixing.get_element("CREF")? else {
        return Ok(None);
    };
    let section_noun = section.element_type()?;
    if section_noun != "GENSEC" && section_noun != "SCTN" {
        return Ok(None);
    }
    let at_start = section.get_ref("JOIS")? == Some(fixing.refno());
    if !at_start && section.get_ref("JOIE")? != Some(fixing.refno()) {
        return Ok(None);
    }
    let tangent = if section_noun == "GENSEC" {
        let Some(path) = crate::section::spine_path(&section)? else {
            return Ok(None);
        };
        let tangent = if at_start {
            path.segments.first().map(|segment| segment.tangents().0)
        } else {
            path.segments.last().map(|segment| -segment.tangents().1)
        };
        let Some(tangent) = tangent else {
            return Ok(None);
        };
        tangent
    } else {
        // SCTN 是直段：`POSS → POSE` 就是轴向；起端（JOIS）指向截面里面 = 轴正向，
        // 末端（JOIE）= 轴反向，与 GENSEC 的端切向口径一致。
        let poss = section
            .get_position("POSS")?
            .map(DVec3::from_array)
            .unwrap_or(DVec3::ZERO);
        let pose = section
            .get_position("POSE")?
            .map(DVec3::from_array)
            .unwrap_or(poss);
        let axis = pose - poss;
        if axis.length() <= 0.001 || !axis.is_finite() {
            return Ok(None);
        }
        let axis = axis.normalize();
        if at_start { axis } else { -axis }
    };
    let into_section = DMat3::from_mat4(world_matrix(&section)?) * tangent;
    let local = DMat3::from_mat4(world_matrix(fixing)?).inverse() * into_section;
    if !local.is_finite() || local.length_squared() < 1e-18 {
        return Ok(None);
    }
    Ok(Some(AttachmentAngles {
        xz: local.x.atan2(local.z).to_degrees(),
        yz: (-local.y).atan2(local.z).to_degrees(),
    }))
}

/// 宿主扫描的 mode：`sub_10722680` 取到几何源之后才改写成 [`HOST_FIXING_MODE`] /
/// [`HOST_FITTING_MODE`]。
const HOST_SCAN_MODE: u32 = 9;

/// 一只 FIXING 给别的构件扣的刀，按 `sub_10722680` 取几何源：首个 `TMPL` 成员，没有就是
/// `SPRE → CATR` 的目录件。`mode` 是 [`HOST_SCAN_MODE`] 时按源改写（SFIT 3，否则 1），别的原样用。
///
/// TMPL 的设计负体（与 [`collect_negatives`] 同一圈定：直属负体 + CMPF 深层）过
/// [`design_napp_admits`] 交回 `negatives`，照直属负体走表现级门与世界放置；目录件 `NGMR` 的负体
/// 过 [`catalogue_napp_admits`] 后求值，已在世界系，进 `catalogue_cuts`（单把求不出的进 `notes`）。
/// 返回 [`FixingCutTally`]：有没有几何源、交出几把、NAPP 门挡下几把。
///
/// 只返回「挡下的把数」时，「没有几何源」与「有源且刀全部放行」都是 0，调用方会把后者
/// 误记成没源的 skipped——这正是改成三项账的原因。
fn fixing_negatives(
    fixing: &DbElement,
    mode: u32,
    negatives: &mut Vec<DbElement>,
    catalogue_cuts: &mut Vec<Manifold>,
    notes: &mut Vec<String>,
    cache: &mut CatalogueMeshCache,
) -> Result<FixingCutTally, ModelError> {
    let template = fixing
        .members()?
        .into_iter()
        .find(|member| matches!(member.element_type().as_deref(), Ok("TMPL")));
    if let Some(template) = template {
        let mode = if mode == HOST_SCAN_MODE {
            HOST_FIXING_MODE
        } else {
            mode
        };
        let mut found = Vec::new();
        collect_negatives(&template, &mut found)?;
        let before = found.len();
        found.retain(|neg| design_napp_admits(mode, negative_napp(neg)));
        let tally = FixingCutTally {
            has_source: true,
            admitted: found.len(),
            rejected: before - found.len(),
        };
        negatives.extend(found);
        return Ok(tally);
    }
    let Some(component) = crate::catalogue_point::catalogue_component(fixing)? else {
        return Ok(FixingCutTally::default());
    };
    let mode = match mode {
        HOST_SCAN_MODE if component.element_type()? == "SFIT" => HOST_FITTING_MODE,
        HOST_SCAN_MODE => HOST_FIXING_MODE,
        other => other,
    };
    let rejected = Cell::new(0usize);
    let admit = |neg: &DbElement| {
        let admitted = catalogue_napp_admits(mode, negative_napp(neg));
        if !admitted {
            rejected.set(rejected.get() + 1);
        }
        admitted
    };
    let cuts = crate::catalogue::catalogue_host_cuts(fixing, &admit, cache)?;
    let admitted = cuts.cutters.len();
    catalogue_cuts.extend(cuts.cutters);
    notes.extend(cuts.diagnostics.into_iter().map(|line| {
        format!(
            "FIXING {} 的目录负体（mode {mode}）：{line}",
            fixing.refno()
        )
    }));
    Ok(FixingCutTally {
        has_source: true,
        admitted,
        rejected: rejected.get(),
    })
}

/// [`fixing_negatives`] 的账。
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct FixingCutTally {
    /// 找到了几何源：首个 `TMPL` 成员，或 `SPRE → CATR` 的目录件。
    has_source: bool,
    /// 过了 NAPP 门、交给调用方的刀数（设计负体 + 求得的目录刀）。
    admitted: usize,
    /// NAPP 门挡下的把数。
    rejected: usize,
}

/// 关节 / 成员链刀源在 NAPP 门之外的归类（NAPP 挡下的把数另记 note）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum JointCutOutcome {
    /// 没有几何源——记 `negatives_skipped`，或按 [`JointClass`] 落进「已解释」账。
    NoSource,
    /// 有源，但源里一把刀也没有（挡下的也没有）：E3D 同样不出刀，记 note。
    SourceWithoutCuts,
    /// 有源且有刀（交出或被挡下）：不记 skipped。
    Cut,
}

impl JointCutOutcome {
    fn of(tally: FixingCutTally) -> Self {
        if !tally.has_source {
            Self::NoSource
        } else if tally.admitted == 0 && tally.rejected == 0 {
            Self::SourceWithoutCuts
        } else {
            Self::Cut
        }
    }
}

/// 没有几何源的关节 / 成员刀源按拓扑分型（gen-model #34 D 线 P1–P4，plan §2）。
///
/// 分型照拓扑硬编码，不照字符串：[`JointClass::AcrossSnod`]（J-A）与
/// [`JointClass::OwnSnod`]（J-B）都有同根原生证据允许移出 skipped 记「已解释」
/// ——J-A：§12①（250110 /68 双向 0.007 mm、/70 RECONNECT END 幂等）；J-B：250110
/// /78、/84 两样本（宿主正体双向 0.0063 mm、体积差 2.3e-6、SJOINT 记录零图元）。
/// 其余类别照旧记 `negatives_skipped`，reason 前冠标签，等各自的同根原生对拍
/// （R3：结论不外推）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum JointClass {
    /// J-A：`JOIS`/`JOIE` 指向的 `SJOI` 挂在**另一根** SCTN 的 `SNOD` 下、
    /// 且 `CREF` 回指宿主（250110 /68 → /72 ← SNOD/71 ← SCTN/70，CREF=/68）。
    AcrossSnod,
    /// J-B：`SJOI` 挂在宿主自己名下的 `SNOD` 下（成员链 mode 1 同型）。
    /// 已解释证据：`sctn_250110_jb-20261018`（/78、/84）。
    OwnSnod,
    /// J-C：`JOIS`/`JOIE` 指向 `PJOI` 且没有几何源（J-D 是有源，不进这条路）。
    ProfileJoint,
    /// M：成员链上无源的 `FITT` / `CMFI` / `SBFI`（FITT 自己另出几何，
    /// 成员链只看它的负体）。
    MemberFitting,
    /// 其余拓扑（`SCOJ`/`PCOJ`/`SUBJ`/`FIXING` 直连、属主链读不出来、`CREF` 不回指等）。
    Unclassified,
}

impl JointClass {
    fn label(self) -> &'static str {
        match self {
            Self::AcrossSnod => "J-A",
            Self::OwnSnod => "J-B",
            Self::ProfileJoint => "J-C",
            Self::MemberFitting => "M",
            Self::Unclassified => "未分型",
        }
    }
}

/// `JOIS`/`JOIE` 指向的关节按拓扑分型（成员链不走这里，见 [`member_chain_class`]）。
///
/// 判不出来的按 [`JointClass::Unclassified`]——宁可留在 skipped 也别把没证据的
/// 拓扑错记进「已解释」。`SJOI` 的跨截面判据与 §11 的 `/72` 逐条对齐：`owner` 是
/// `SNOD`、`SNOD` 的属主是**另一根** `SCTN`、关节 `CREF` 回指宿主。
fn joint_class(host: &DbElement, joint: &DbElement) -> JointClass {
    match joint.element_type().as_deref() {
        Ok("PJOI") => JointClass::ProfileJoint,
        Ok("SJOI") => {
            let Some(snod) = joint.owner().ok().flatten() else {
                return JointClass::Unclassified;
            };
            if snod.element_type().ok().as_deref() != Some("SNOD") {
                return JointClass::Unclassified;
            }
            // CREF 不回指宿主的 SJOI 不是 /72 那个拓扑，不外推。
            if joint.get_ref("CREF").ok().flatten() != Some(host.refno()) {
                return JointClass::Unclassified;
            }
            let Some(section) = snod.owner().ok().flatten() else {
                return JointClass::Unclassified;
            };
            if section.element_type().ok().as_deref() != Some("SCTN") {
                return JointClass::Unclassified;
            }
            if section.refno() == host.refno() {
                JointClass::OwnSnod
            } else {
                JointClass::AcrossSnod
            }
        }
        _ => JointClass::Unclassified,
    }
}

/// 成员链刀源的类别（`member_chain_mode` 那张表的对偶）。
fn member_chain_class(source_noun: &str) -> JointClass {
    match source_noun {
        "FITT" | "CMFI" | "SBFI" => JointClass::MemberFitting,
        "SJOI" => JointClass::OwnSnod,
        _ => JointClass::Unclassified,
    }
}

/// 负体的 `NAPP`。文件里是一个字，`-1` 存成 `0xFFFF_FFFF`，按有符号读；未设、模板里
/// 没有、形状不对都是 `None`（NAPP 门对「读不到」放行）。
pub(crate) fn negative_napp(neg: &DbElement) -> Option<i32> {
    match neg.attribute("NAPP") {
        Ok(Some(DescriptorValue::Word { raw, .. })) => Some(raw as i32),
        Ok(Some(DescriptorValue::Int(value))) => Some(value),
        _ => None,
    }
}

/// 关节路径（JOIS / JOIE）的 mode。
const JOINT_MODE: u32 = 0;
/// 宿主 FIXING 扫描在几何源不是 SFIT 时落到的 mode（`sub_10722680` 把 9 改写成 `2*(TYPE==SFIT)+1`）。
const HOST_FIXING_MODE: u32 = 1;
/// 宿主 FIXING 扫描在几何源是 SFIT 时落到的 mode。
const HOST_FITTING_MODE: u32 = 3;

/// `sub_1072B780`（设计图元）的 NAPP 门：读不到 → 放行；`-1` 只在 mode 0/3/4/5/6/9 放行；
/// 其余按 mode 取位（[`napp_mask`]）。
fn design_napp_admits(mode: u32, napp: Option<i32>) -> bool {
    match napp {
        None => true,
        Some(-1) => matches!(mode, 0 | 3 | 4 | 5 | 6 | 9),
        Some(value) => value & napp_mask(mode) != 0,
    }
}

/// `sub_1072A790`（目录图元）的 NAPP 门：与设计那道只差 `-1` 在 mode 9 不放行。
pub(crate) fn catalogue_napp_admits(mode: u32, napp: Option<i32>) -> bool {
    if std::env::var_os("E3D_MODEL_NAPP_OPEN").is_some() {
        return true;
    }
    match napp {
        None => true,
        Some(-1) => matches!(mode, 0 | 3 | 4 | 5 | 6),
        Some(value) => value & napp_mask(mode) != 0,
    }
}

/// 两道 NAPP 门共用的取位：mode 0 看 `& 1`，mode 7 看 `& 4`，mode 1–6 / 8 / 9 看 `& 2`。
fn napp_mask(mode: u32) -> i32 {
    match mode {
        0 => 1,
        7 => 4,
        _ => 2,
    }
}

/// GENSEC 自己 datum 树里的 FIXING 给它扣的负体。
///
/// 对标 `DB_Gensec::makeCSG@0x107311D0`：JOIS / JOIE 之后对自己名下每只 FIXING 调一次
/// `sub_10722D80(..., mode 9)`；取几何源时（`sub_10722680`：FIXING 的首个 `TMPL` 成员，没有
/// 就是 `SPRE → CATR` 的目录件）把 9 改写成「源是 SFIT ? 3 : 1」。
///
/// - TMPL 源（端板、腹板连接件）：mode 1，设计负体过 [`design_napp_admits`]——NBOX / NXTR 的
///   `-1` 不放行（端板让位只给梁），NCYL 的 7 放行：螺栓孔穿过柱翼缘。
/// - 目录源：`NGMR` 名下的负体过 [`catalogue_napp_admits`]。柱底板 SFIT 是 mode 3，NSBO（NAPP 2）
///   按 `OPAR` 取柱截面、切掉柱底一个板厚，NSCY（NAPP 4，锚栓孔）不放行；JOIN（檐口、加腋
///   节点）是 mode 1，NSCY（NAPP 6）放行，NSEX / NSBO（NAPP 1 / -1）不放行。
///
/// 真值（aps250110 E3D TTY RVM）：柱 `/277` 名下两只 Flexible、两只 Full Depth 端板共 20 个
/// 螺栓孔，E3D 那组面片的亏格正好是 20；24 根柱底比 E3D 低出的 30 / 20 mm 是底板厚。
///
/// 设计负体交回 `negatives`（照直属负体走表现级门与世界放置），目录负体已在世界系，进
/// `catalogue_cuts`（[`fixing_negatives`]）。一只 FIXING 读挂只记 `skipped`，不拖垮这根构件。
fn collect_host_fixing_negatives(
    el: &DbElement,
    negatives: &mut Vec<DbElement>,
    catalogue_cuts: &mut Vec<Manifold>,
    skipped: &mut Vec<SkippedNegative>,
    notes: &mut Vec<String>,
    cache: &mut CatalogueMeshCache,
) -> Result<(), ModelError> {
    let mut fixings = Vec::new();
    collect_datum_fixings(el, &mut fixings)?;
    let mut rejected = 0usize;
    for fixing in fixings {
        match fixing_negatives(
            &fixing,
            HOST_SCAN_MODE,
            negatives,
            catalogue_cuts,
            notes,
            cache,
        ) {
            Ok(tally) => rejected += tally.rejected,
            Err(error) => skipped.push(SkippedNegative {
                refno: fixing.refno(),
                noun: "FIXING".into(),
                reason: format!("宿主 FIXING 扫描读不出这只 FIXING 的负体：{error}"),
            }),
        }
    }
    if rejected > 0 {
        notes.push(format!(
            "宿主 FIXING 扫描：NAPP 门挡下 {rejected} 把刀，E3D 同样不拿它们切这根构件"
        ));
    }
    Ok(())
}

/// 构件自己 datum 树里的 FIXING：沿成员往下找，SPINE（路径）与 TMPL（FIXING 自己的几何）
/// 两种子树不进；FIXING 名下再挂的 FIXING 也算。
fn collect_datum_fixings(el: &DbElement, out: &mut Vec<DbElement>) -> Result<(), DbElementError> {
    for member in el.members()? {
        match member.element_type()?.as_str() {
            "SPINE" | "TMPL" => {}
            "FIXING" => {
                collect_datum_fixings(&member, out)?;
                out.push(member);
            }
            _ => collect_datum_fixings(&member, out)?,
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
        Category::NegPrimitive(Prim::CircTorus) => {
            let rins = default_zero(neg, "RINS")?;
            let rout = require_double(neg, &noun, "ROUT", "circular torus")?;
            let angle = require_double(neg, &noun, "ANGL", "circular torus")?;
            // AMS 8000 HOST_NCTO: ROUT=520, RINS=250, ANGL=180 has
            // eight sweep intervals and twelve tube intervals in its RVM.
            let cutter = circ_torus_solid_with_segments(
                rins,
                rout,
                angle,
                negative_part_rev_segments(rout, angle),
                negative_cylinder_segments((rout - rins) * 0.5),
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
    fn design_snout_uses_analytic_or_boolean_grid_from_ams8000() {
        let standalone = design_snout_solid(500.0, 320.0, 600.0, 0.0, 0.0, false).unwrap();
        let cutter = design_snout_solid(500.0, 320.0, 600.0, 0.0, 0.0, true).unwrap();
        // Saved standalone RVM Snout and the cavity in HOST_NCON, after resize.
        assert!((standalone.volume() - 80_358_356.1773).abs() / 80_358_356.1773 < 0.01);
        let rvm_cavity_volume = 1200.0 * 1200.0 * 800.0 - 1_075_139_827.1983;
        assert!((cutter.volume() - rvm_cavity_volume).abs() / rvm_cavity_volume < 0.0001);
        assert!(standalone.volume() > cutter.volume());
    }

    #[test]
    fn design_revolution_matches_ams8000_baked_profile_volume() {
        let profile = vec![vec![
            [0.0, 200.0],
            [500.0, 200.0],
            [500.0, 500.0],
            [0.0, 500.0],
        ]];
        let solid = revolve_rings_profile_faceted_inflated(profile, 180.0, 0.0).unwrap();
        let rvm_volume = 160_726_996.3865;
        assert!((solid.volume() - rvm_volume).abs() / rvm_volume < 0.0001);
    }

    #[test]
    fn the_joint_napp_gate_is_mode_0_of_sub_1072b780() {
        // 读不到放行；-1 放行（aps250110 节点模板的 NBOX / NXTR）；其余看 `& 1`（NCYL 的 7 放行）。
        assert!(design_napp_admits(JOINT_MODE, None));
        assert!(design_napp_admits(JOINT_MODE, Some(-1)));
        assert!(design_napp_admits(JOINT_MODE, Some(7)));
        assert!(design_napp_admits(JOINT_MODE, Some(1)));
        for napp in [0, 2, 4, 6] {
            assert!(!design_napp_admits(JOINT_MODE, Some(napp)), "NAPP={napp}");
        }
    }

    /// `attachment_angles` 对 aps250110 四只檐口 FIXING 与 E3D TTY 的 `Q AANGYZ` / `Q AANGXZ` 逐只相等
    /// （-14.481° / 0°，TTY 打到千分之一度）。库或模板不在就跳过。
    #[test]
    fn attachment_angles_match_e3d_on_the_eaves_joints() {
        let plant = std::path::Path::new(r"D:\AVEVA\Projects\E3D3.1\AvevaPlantSample\aps000");
        let acp = std::path::Path::new(r"D:\AVEVA\Projects\E3D3.1\AvevaCatalogue\acp000");
        let template = std::path::Path::new(r"E:\reverse\e3d\shadow_e3d31_aps_all");
        if !plant.join("aps250110_0001").exists()
            || !acp.exists()
            || !template.join("attlib.dat").exists()
        {
            return;
        }
        // 斜梁的截面目录在 250701：datum 标架的 JUSL 偏移要它。
        let resolver = crate::db_discovery::DirectoryDbResolver::scan(
            &[plant.to_path_buf(), acp.to_path_buf()],
            template,
        )
        .unwrap();
        let set = std::sync::Arc::new(
            e3d_io::db_element::DbSet::with_attlib_file_and_resolver(
                &template.join("attlib.dat"),
                Box::new(resolver),
            )
            .unwrap(),
        );
        set.add_db(e3d_io::db_element::DbFilePin {
            file: plant.join("aps250110_0001"),
            template: e3d_io::db_element::template_file_for(template, "DESI").unwrap(),
            db_type: Some("DESI".into()),
            sesno: None,
        })
        .unwrap();
        for w1 in [2232, 2246, 2283, 2334] {
            let fixing = set.element(RefNo::new(2013286654, w1));
            let angles = attachment_angles(&fixing)
                .unwrap()
                .expect("檐口 FIXING 连着斜梁");
            assert!((angles.yz - -14.481).abs() < 5e-4, "{w1}: {angles:?}");
            assert!(angles.xz.abs() < 5e-4, "{w1}: {angles:?}");
        }
        let beam_fixing = set.element(RefNo::new(2013286654, 283));
        assert!(
            attachment_angles(&beam_fixing).unwrap().is_some(),
            "端板 FIXING 也连着梁"
        );
        assert_eq!(
            attachment_angles(&set.element(RefNo::new(2013286654, 208))).unwrap(),
            None,
            "CREF 空的 WALL FIXING"
        );
    }

    #[test]
    fn the_host_fixing_scan_gates_are_modes_1_and_3() {
        // TMPL 源 mode 1：端板让位 NBOX / NXTR 的 -1 挡下，螺栓孔 NCYL 的 7 放行。
        assert!(!design_napp_admits(HOST_FIXING_MODE, Some(-1)));
        assert!(design_napp_admits(HOST_FIXING_MODE, Some(7)));
        // SFIT 源 mode 3：底板 NSBO 的 2 放行、锚栓孔 NSCY 的 4 挡下；目录的 -1 在 mode 3 放行。
        assert!(catalogue_napp_admits(HOST_FITTING_MODE, Some(2)));
        assert!(!catalogue_napp_admits(HOST_FITTING_MODE, Some(4)));
        assert!(catalogue_napp_admits(HOST_FITTING_MODE, Some(-1)));
        // JOIN 源 mode 1：NSCY 的 6 放行，NSEX 的 1 与 -1 挡下。
        assert!(catalogue_napp_admits(HOST_FIXING_MODE, Some(6)));
        assert!(!catalogue_napp_admits(HOST_FIXING_MODE, Some(1)));
        assert!(!catalogue_napp_admits(HOST_FIXING_MODE, Some(-1)));
        // 两道门只差 -1 在 mode 9 上：设计图元放行，目录图元不放行。
        assert!(design_napp_admits(9, Some(-1)));
        assert!(!catalogue_napp_admits(9, Some(-1)));
        // mode 7（TMPL 属主）看 `& 4`。
        assert!(design_napp_admits(7, Some(4)));
        assert!(!design_napp_admits(7, Some(3)));
    }

    /// SCTN 端关节负体经 `sub_10722D80` 的 mode-2 改写（`2 − (mode≠0)`），过 `NAPP & 2`
    /// 门——与 GENSEC 关节的 mode 0（`& 1`、`-1` 放行）有意不同：SJOI 链上若出现
    /// `NAPP = -1` 的刀，E3D 在 mode 2 下也不拿它切。
    #[test]
    fn the_sctn_joint_gate_is_mode_2_not_mode_0() {
        assert_eq!(SCTN_JOINT_MODE, 2);
        // -1 只放 mode 0/3/4/5/6/9：mode 2 挡下，mode 0 放行——两态对照钉死。
        assert!(!design_napp_admits(SCTN_JOINT_MODE, Some(-1)));
        assert!(design_napp_admits(JOINT_MODE, Some(-1)));
        // 门位 `& 2`：NAPP 带 2 位（2 / 3 / 6 / 7）放行，1 / 4 / 8 挡下。
        for napp in [2, 3, 6, 7] {
            assert!(
                design_napp_admits(SCTN_JOINT_MODE, Some(napp)),
                "NAPP={napp}"
            );
        }
        for napp in [1, 4, 8] {
            assert!(
                !design_napp_admits(SCTN_JOINT_MODE, Some(napp)),
                "NAPP={napp}"
            );
        }
        // 目录件一侧同一道门（与 mode 9 的那点差与这里无关）。
        assert!(catalogue_napp_admits(SCTN_JOINT_MODE, Some(2)));
        assert!(!catalogue_napp_admits(SCTN_JOINT_MODE, Some(1)));
        assert!(!catalogue_napp_admits(SCTN_JOINT_MODE, Some(-1)));
    }

    /// `sub_10721600(mode 3)` 的成员链扫描表（recheck-2 §2.3）：`FITT`/`CMFI` 直接成
    /// mode 3，`CMFI` 名下 `SBFI` 是 mode 6，`SNOD` 名下 `SJOI` 是 mode 1，`SCOJ` 本身
    /// mode 2、其名下 `SUBJ` mode 1；表外组合不进这条扫描。
    #[test]
    fn the_sctn_member_chain_scan_table_is_sub_10721600() {
        assert_eq!(member_chain_mode("FITT", None), Some(3));
        assert_eq!(member_chain_mode("CMFI", None), Some(3));
        assert_eq!(member_chain_mode("CMFI", Some("SBFI")), Some(6));
        assert_eq!(member_chain_mode("SNOD", Some("SJOI")), Some(1));
        assert_eq!(member_chain_mode("SCOJ", None), Some(2));
        assert_eq!(member_chain_mode("SCOJ", Some("SUBJ")), Some(1));
        // 表外：SNOD 自己不是刀源（只是节点载体），FITT 不往下扫，别的嵌套一律不算。
        assert_eq!(member_chain_mode("SNOD", None), None);
        assert_eq!(member_chain_mode("FITT", Some("SBFI")), None);
        assert_eq!(member_chain_mode("TMPL", None), None);
        assert_eq!(member_chain_mode("SNOD", Some("FITT")), None);
        // 成员链 mode 集 {1,2,3,6}：`NAPP = -1` 只在 3 / 6 放行。
        for mode in [1, 2] {
            assert!(!design_napp_admits(mode, Some(-1)), "mode {mode}");
        }
        for mode in [3, 6] {
            assert!(design_napp_admits(mode, Some(-1)), "mode {mode}");
        }
    }

    /// 「有源且刀全部放行」曾因只返回挡下把数（= 0）被误记成没源的 skipped；
    /// 现在只有 `has_source == false` 才归 `NoSource`。
    #[test]
    fn only_a_missing_geometry_source_is_accounted_as_skipped() {
        let tally = |has_source, admitted, rejected| FixingCutTally {
            has_source,
            admitted,
            rejected,
        };
        assert_eq!(
            JointCutOutcome::of(FixingCutTally::default()),
            JointCutOutcome::NoSource
        );
        // 回归点：有源、刀全部放行、一把没挡——旧口径在这里返回 0 并记 skipped。
        assert_eq!(JointCutOutcome::of(tally(true, 3, 0)), JointCutOutcome::Cut);
        assert_eq!(JointCutOutcome::of(tally(true, 2, 1)), JointCutOutcome::Cut);
        assert_eq!(JointCutOutcome::of(tally(true, 0, 2)), JointCutOutcome::Cut);
        assert_eq!(
            JointCutOutcome::of(tally(true, 0, 0)),
            JointCutOutcome::SourceWithoutCuts
        );
    }

    /// aps250110 的 SCTN 端关节：`SCTN 2013286654/68 JOIE → SJOI 72`，SJOI 无 TMPL/SPRE
    /// 几何源——这正是 §12① 有原生证据的 J-A 拓扑（SNOD/71←SCTN/70、CREF=/68），从
    /// `negatives_skipped` 移进 `joint_cuts_explained` 单独记账；同一只 SJOI/72 作为
    /// `/70` 的 SNOD 成员属 J-B，留在 skipped。语料不在就跳过（同檐口测试那道门）。
    #[test]
    fn sctn_endpoint_joints_without_a_geometry_source_are_accounted() {
        let plant = std::path::Path::new(r"D:\AVEVA\Projects\E3D3.1\AvevaPlantSample\aps000");
        let acp = std::path::Path::new(r"D:\AVEVA\Projects\E3D3.1\AvevaCatalogue\acp000");
        let template = std::path::Path::new(r"E:\reverse\e3d\shadow_e3d31_aps_all");
        if !plant.join("aps250110_0001").exists()
            || !acp.exists()
            || !template.join("attlib.dat").exists()
        {
            return;
        }
        let resolver = crate::db_discovery::DirectoryDbResolver::scan(
            &[plant.to_path_buf(), acp.to_path_buf()],
            template,
        )
        .unwrap();
        let set = std::sync::Arc::new(
            e3d_io::db_element::DbSet::with_attlib_file_and_resolver(
                &template.join("attlib.dat"),
                Box::new(resolver),
            )
            .unwrap(),
        );
        set.add_db(e3d_io::db_element::DbFilePin {
            file: plant.join("aps250110_0001"),
            template: e3d_io::db_element::template_file_for(template, "DESI").unwrap(),
            db_type: Some("DESI".into()),
            sesno: None,
        })
        .unwrap();
        let sctn = set.element(RefNo::new(2013286654, 68));
        let modeled =
            model_element(&sctn, crate::category::classify("SCTN")).expect("SCTN/68 建模不该报错");
        let Modeled::Solid(generated) = modeled else {
            panic!("SCTN/68 该出实体");
        };
        assert!(
            generated
                .joint_cuts_explained
                .iter()
                .any(|skip| skip.refno.to_string() == "2013286654/72"
                    && skip.noun == "SJOI"
                    && skip.reason.starts_with("[J-A]")),
            "JOIE→SJOI/72 是 J-A 拓扑（§12① 已证端面覆盖），要记「已解释」账：{:?}",
            generated.joint_cuts_explained
        );
        assert!(
            !generated
                .negatives_skipped
                .iter()
                .any(|skip| skip.refno.to_string() == "2013286654/72"),
            "J-A 不许同时挂 skipped：{:?}",
            generated.negatives_skipped
        );
        // 成员链侧：邻居 SCTN/70 名下 SNOD/71 里的 SJOI/72 同样没源，属 J-B——/78、/84
        // 同根原生对拍后移进「已解释」账（sctn_250110_jb-20261018），不再挂 skipped。
        let neighbor = set.element(RefNo::new(2013286654, 70));
        let Modeled::Solid(neighbor_generated) =
            model_element(&neighbor, crate::category::classify("SCTN")).expect("SCTN/70 建模")
        else {
            panic!("SCTN/70 该出实体");
        };
        assert!(
            neighbor_generated
                .joint_cuts_explained
                .iter()
                .any(|skip| skip.refno.to_string() == "2013286654/72"
                    && skip.reason.starts_with("[J-B]")),
            "SNOD/71 名下的 SJOI/72（J-B）要记「已解释」账：{:?}",
            neighbor_generated.joint_cuts_explained
        );
        assert!(
            !neighbor_generated
                .negatives_skipped
                .iter()
                .any(|skip| skip.refno.to_string() == "2013286654/72"),
            "J-B 不许同时挂 skipped：{:?}",
            neighbor_generated.negatives_skipped
        );
    }

    /// P7 守卫：SCTN 名下 `SNOD` 成员的 `SJOI` 目前全语料交不出一把刀
    /// （几何源多半有——7200 有几十只 SPRE 解得开——但 mode 1 下产出为零，
    /// 报告里记「有几何源但其中没有负体」）。一旦语料里出现真能交出负体的
    /// SNOD→SJOI，`ImplicitFrame::SectionNode` 的暂定标架（GJTORI/GJTORG 未翻）
    /// 就被真数据考验——先在这里红出来点名，去处理 P7 再放行那把刀。
    /// 语料不在就跳过（同檐口测试那道门）。
    #[test]
    fn no_snod_sjoi_in_the_corpus_carries_a_geometry_source() {
        let plant = std::path::Path::new(r"D:\AVEVA\Projects\E3D3.1\AvevaPlantSample\aps000");
        let acp = std::path::Path::new(r"D:\AVEVA\Projects\E3D3.1\AvevaCatalogue\acp000");
        let template = std::path::Path::new(r"E:\reverse\e3d\shadow_e3d31_aps_all");
        let dbs = ["aps7200_0001", "aps250110_0001"];
        if !dbs.iter().all(|db| plant.join(db).exists())
            || !acp.exists()
            || !template.join("attlib.dat").exists()
        {
            return;
        }
        let resolver = crate::db_discovery::DirectoryDbResolver::scan(
            &[plant.to_path_buf(), acp.to_path_buf()],
            template,
        )
        .unwrap();
        let set = std::sync::Arc::new(
            e3d_io::db_element::DbSet::with_attlib_file_and_resolver(
                &template.join("attlib.dat"),
                Box::new(resolver),
            )
            .unwrap(),
        );
        let mut checked = 0usize;
        let mut sourced: Vec<String> = Vec::new();
        let mut cache = CatalogueMeshCache::default();
        for db in dbs {
            let file = plant.join(db);
            let dbno = set
                .add_db(e3d_io::db_element::DbFilePin {
                    file: file.clone(),
                    template: e3d_io::db_element::template_file_for(template, "DESI").unwrap(),
                    db_type: Some("DESI".into()),
                    sesno: None,
                })
                .unwrap();
            let mut engine = e3d_io::engine::ReadOnlyEngine::open(&file).unwrap();
            let refnos: Vec<RefNo> = engine
                .scan_elements(e3d_io::engine::ScanTier::Header)
                .unwrap()
                .map(|scanned| scanned.unwrap().refno)
                .filter(|refno| refno.dbno() == dbno)
                .collect();
            for refno in refnos {
                let joint = set.element(refno);
                if joint.element_type().ok().as_deref() != Some("SJOI") {
                    continue;
                }
                let Ok(Some(snod)) = joint.owner() else {
                    continue;
                };
                if snod.element_type().ok().as_deref() != Some("SNOD") {
                    continue;
                }
                let Ok(Some(section)) = snod.owner() else {
                    continue;
                };
                if section.element_type().ok().as_deref() != Some("SCTN") {
                    continue;
                }
                checked += 1;
                // 真正会造出刀的才会用 `ImplicitFrame::SectionNode` 那把暂定标架：
                // 几何源在、NAPP 门（mode 1）放行至少一把——与生成路径同一个
                // fixing_negatives 判定。
                let mut negatives = Vec::new();
                let mut cuts = Vec::new();
                let mut feed_notes = Vec::new();
                match fixing_negatives(
                    &joint,
                    1,
                    &mut negatives,
                    &mut cuts,
                    &mut feed_notes,
                    &mut cache,
                ) {
                    Ok(tally) if tally.admitted > 0 => sourced.push(format!(
                        "{refno}（SNOD {} ← SCTN {}）交出 {} 把刀",
                        snod.refno(),
                        section.refno(),
                        tally.admitted
                    )),
                    _ => {}
                }
            }
        }
        assert!(checked > 0, "一个 SNOD→SJOI 都没找到，门没有判别力");
        assert!(
            sourced.is_empty(),
            "{} 只 SNOD→SJOI 能交出负体，P7（关节标架未翻）必须先处理再放行：{sourced:?}",
            sourced.len()
        );
    }

    /// S1 只读普查（gen-model #34 plan §S1）：全 APS 的 DESI 库过一遍，给每只 SCTN 的
    /// JOIS/JOIE 关节打 [`JointClass`] 并记有没有几何源（J-D = PJOI 有源，普查它就是
    /// 为了证明这一类到底存不存在）；另外把 JUSL 在 PSTR 里找不到的 SCTN 连同该 PSTR
    /// 实际有哪些 PKEY 一起列出来（P5 的前置事实：FOC/LBOA 是「目录里真没有这条线」
    /// 还是「换个名字在」）。
    ///
    /// 输出按行打印（`--nocapture`），交由 shell 重定向到 `classes.json` / `jusl-missing.txt`
    /// 落账；不进默认测试集——全语料扫一遍要读十几个库，故 `#[ignore]`。
    #[test]
    #[ignore]
    fn census_sctn_joint_classes_and_missing_jusl() {
        let plant = std::path::Path::new(r"D:\AVEVA\Projects\E3D3.1\AvevaPlantSample\aps000");
        let acp = std::path::Path::new(r"D:\AVEVA\Projects\E3D3.1\AvevaCatalogue\acp000");
        let template = std::path::Path::new(r"E:\reverse\e3d\shadow_e3d31_aps_all");
        if !plant.exists() || !acp.exists() || !template.join("attlib.dat").exists() {
            return;
        }
        let resolver = crate::db_discovery::DirectoryDbResolver::scan(
            &[plant.to_path_buf(), acp.to_path_buf()],
            template,
        )
        .unwrap();
        let set = std::sync::Arc::new(
            e3d_io::db_element::DbSet::with_attlib_file_and_resolver(
                &template.join("attlib.dat"),
                Box::new(resolver),
            )
            .unwrap(),
        );
        let mut cache = CatalogueMeshCache::default();
        let mut sctn_seen = 0usize;
        let mut files = std::fs::read_dir(plant)
            .unwrap()
            .filter_map(|entry| entry.ok().map(|e| e.path()))
            .filter(|path| {
                path.file_name()
                    .and_then(|name| name.to_str())
                    .is_some_and(|name| name.starts_with("aps") && name.ends_with("_0001"))
            })
            .collect::<Vec<_>>();
        files.sort();
        for file in files {
            let mut engine = match e3d_io::engine::ReadOnlyEngine::open(&file) {
                Ok(engine) => engine,
                Err(_) => continue,
            };
            if engine.descriptor().db_type_name().as_deref() != Some("DESI") {
                continue;
            }
            let db_name = file.file_name().unwrap().to_string_lossy().to_string();
            let refnos: Vec<RefNo> = engine
                .scan_elements(e3d_io::engine::ScanTier::Header)
                .unwrap()
                .filter_map(|scanned| scanned.ok().map(|s| s.refno))
                .collect();
            for refno in refnos {
                let sctn = set.element(refno);
                if sctn.element_type().ok().as_deref() != Some("SCTN") {
                    continue;
                }
                sctn_seen += 1;
                for attr in ["JOIS", "JOIE"] {
                    let Ok(Some(joint)) = sctn.get_element(attr) else {
                        continue;
                    };
                    let Ok(joint_noun) = joint.element_type() else {
                        continue;
                    };
                    if joint_noun != "PJOI" {
                        continue;
                    }
                    let mut negatives = Vec::new();
                    let mut cuts = Vec::new();
                    let mut feed_notes = Vec::new();
                    let source = match fixing_negatives(
                        &joint,
                        SCTN_JOINT_MODE,
                        &mut negatives,
                        &mut cuts,
                        &mut feed_notes,
                        &mut cache,
                    ) {
                        Ok(tally) if tally.has_source => {
                            format!(
                                "有源 admitted={} rejected={}",
                                tally.admitted, tally.rejected
                            )
                        }
                        Ok(_) => "无源".to_string(),
                        Err(error) => format!("读不出：{error}"),
                    };
                    println!(
                        "JOINT {db_name} {refno} {attr}→PJOI {} {source}",
                        joint.refno()
                    );
                }
                let jusl = sctn
                    .get_string("JUSL")
                    .ok()
                    .flatten()
                    .map(|text| text.trim().to_ascii_uppercase())
                    .filter(|text| !text.is_empty())
                    .unwrap_or_else(|| "NA".to_string());
                let Ok(SectionChain::Resolved(catalogue)) = section_catalogue(&sctn) else {
                    continue;
                };
                let Ok(profile) = evaluate_section_profile(&sctn, &catalogue) else {
                    continue;
                };
                // 请求的线与实际解析到的线一致才算正常：JUSL 缺失回退中性轴、
                // 或连中性轴都没有按原点，都要打出来。
                if profile.justification_found && profile.justification_line == jusl {
                    continue;
                }
                let keys = catalogue
                    .sprf
                    .get_element("PSTR")
                    .ok()
                    .flatten()
                    .map(|ptss| {
                        ptss.members()
                            .map(|members| {
                                members
                                    .into_iter()
                                    .filter(|m| {
                                        m.element_type()
                                            .ok()
                                            .as_deref()
                                            .is_some_and(|n| n.eq_ignore_ascii_case("PLIN"))
                                    })
                                    .filter_map(|m| m.get_string("PKEY").ok().flatten())
                                    .map(|key| key.trim().to_ascii_uppercase())
                                    .collect::<Vec<_>>()
                            })
                            .unwrap_or_default()
                    })
                    .unwrap_or_default();
                println!(
                    "JUSL {db_name} {refno} JUSL={jusl} resolved={} SPRF={} PKEYs=[{}]",
                    profile.justification_line,
                    catalogue.sprf.refno(),
                    keys.join(",")
                );
            }
        }
        println!("CENSUS-TOTAL sctn={sctn_seen}");
        assert!(sctn_seen > 0, "一根 SCTN 都没扫到，普查没有判别力");
    }

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

    /// 一堵圆弧墙照 [`wall_local_solid`] 的路数建出来：`[inner, outer]` 矩形截面绕圆心扫
    /// `sweep`，再按 SPINE `DRNS`（起点端向）斜接。返回 (斜接前, 斜接后, 补挤长度)。
    fn mitred_circular_wall(
        center: DVec3,
        outer_start: DVec3,
        thickness: f64,
        height: f64,
        sweep: f64,
        drns: DVec3,
    ) -> (Manifold, Manifold, f64) {
        let radius = (outer_start - center).length();
        let a0 = (outer_start.y - center.y).atan2(outer_start.x - center.x);
        let (inner, outer) = circular_wall_radii(radius, [0.0, thickness], sweep.signum());
        let sweep_deg = sweep.to_degrees();
        let segments = negative_part_rev_segments(outer, sweep_deg);
        let torus = rect_torus_solid_with_segments(inner, outer, height, sweep_deg, segments)
            .expect("wall torus");
        let local = DMat4::from_translation(DVec3::new(center.x, center.y, height * 0.5))
            * DMat4::from_rotation_z(a0);
        let plain = placed(&torus, local);

        let travel = sweep.signum();
        let radial = DVec3::new(a0.cos(), a0.sin(), 0.0);
        let ccw = DVec3::new(-a0.sin(), a0.cos(), 0.0);
        let point = center + radial * radius;
        let plane = spine_mitre_plane(drns, ccw * travel, point, true).expect("a real mitre");
        let frame = wall_end_frame(point, a0, travel);
        let ring = vec![
            [0.0, 0.0],
            [thickness, 0.0],
            [thickness, height],
            [0.0, height],
        ];
        let extended = mitre_extension_length(&[ring.clone()], frame, &plane, 1.0);
        let mut mitred = plain.clone();
        if let Some(extension) =
            mitre_extension_solid(&[ring], frame, &plane, 1.0).expect("extension")
        {
            mitred = &mitred + &extension;
        }
        let mitred = crate::solid::cut_by_plane(&mitred, plane.point, plane.normal);
        (plain, mitred, extended)
    }

    /// ams1112 `/1RS-WF03-W-C-RR001` WALL 4（3 个扫掠区间，SPINE `DRNS = E`）：E3D 从起点
    /// **外延**到 `x = pt0.x` 那张竖直面。此前按「默认 +Y 向三角楔」硬编码出来的第三个角
    /// 是 (−5058.2188, −15289.8818)；按 DRNS 斜接补挤出来的那一角必须落在同一处。
    #[test]
    fn wf03_wall_4_drns_east_extends_the_start_to_the_vertical_plane() {
        let center = DVec3::new(0.1140846, 0.4991176, 0.0);
        let outer = DVec3::new(-5058.21875, -16648.55664, 0.0);
        let thickness = 1300.0;
        let height = 3900.0;
        // 起点 −106.90°，终点 −99.07°（`docs/2026-08-12_live-test-ledger.md`）。
        let sweep = 7.83f64.to_radians();
        let (plain, mitred, extended) =
            mitred_circular_wall(center, outer, thickness, height, sweep, DVec3::X);
        // 内侧切线与 `x = pt0.x` 的交点离内角 394.97 mm，就是要补挤的长度。
        assert!((extended - 394.97).abs() < 0.05, "{extended}");
        // 那一角：起点 + 厚度·朝内 − 补挤·切向。
        let radial = (outer - center).normalize();
        let ccw = DVec3::new(-radial.y, radial.x, 0.0);
        let corner = outer - radial * thickness - ccw * extended;
        assert!((corner.x - -5058.2188).abs() < 0.01, "{corner:?}");
        assert!((corner.y - -15289.8818).abs() < 0.01, "{corner:?}");
        // 多出来的体积 = 三角楔（底 394.97 × 高 1300 / 2）× 墙高。
        let wedge = 0.5 * extended * thickness * height;
        let delta = mitred.volume() - plain.volume();
        assert!((delta - wedge).abs() / wedge < 0.01, "{delta} vs {wedge}");
        // 平面本身不动外角：包围盒 x 最小值仍是 pt0.x。
        let bb = mitred.bounding_box().expect("solid");
        assert!((bb.min()[0] - outer.x).abs() < 0.01, "{:?}", bb.min());
        // 楔子与本体要并成一块，不能是两片只贴着脸的壳（内部那张重合面会留在网格里）。
        assert_eq!(mitred.decompose().len(), 1, "延伸段没有与本体咬合");
    }

    /// ams1112 `/1RS-WF05-W-C-RR001` WALL 4（`17496/107037`，9 个区间，SPINE `Drnstart E`）：
    /// 同一张 `x = pt0.x` 的面在这里把起点内角**削掉**——E3D TTY 导出的 RVM 起点端内外两角
    /// 同 x = 5492.3；径向平头的内角在 x = 5208.2（差 284 mm，2026-09-23 对拍抓到的那一件）。
    #[test]
    fn wf05_wall_4_drns_east_trims_the_start_to_the_vertical_plane() {
        let center = DVec3::ZERO;
        let outer = DVec3::new(5492.3, -16510.4, 0.0);
        let thickness = 900.0;
        let height = 5980.0;
        let end = DVec3::new(13653.2, -10786.5, 0.0);
        let sweep = (end.y.atan2(end.x) - outer.y.atan2(outer.x)).rem_euclid(std::f64::consts::TAU);
        let (plain, mitred, extended) =
            mitred_circular_wall(center, outer, thickness, height, sweep, DVec3::X);
        assert_eq!(extended, 0.0, "内角在体内一侧，不补挤");
        let before = plain.bounding_box().expect("solid");
        let after = mitred.bounding_box().expect("solid");
        assert!((before.min()[0] - 5208.2).abs() < 0.5, "{:?}", before.min());
        assert!((after.min()[0] - 5492.3).abs() < 0.01, "{:?}", after.min());
        // 别的边不动：外角 y、终点端、高度。
        assert!((after.min()[1] - before.min()[1]).abs() < 1e-6);
        assert!((after.max()[0] - before.max()[0]).abs() < 1e-6);
        assert!((after.max()[1] - before.max()[1]).abs() < 1e-6);
        assert!((after.max()[2] - height).abs() < 1e-6 && after.min()[2].abs() < 1e-6);
        assert!(mitred.volume() < plain.volume());
    }

    /// 截面 x 朝行进方向的左手侧：ams7997 `/1RB-WF05-W-C-RR001` WALL 1（`24381/44295`，spine 半径
    /// 26500、顺时针 90°、墙厚 1500、JUSL=OBOW）在 E3D TTY RVM 里是 [26500, 28000]，不是 [25000, 26500]；
    /// 端标架 X 跟着朝外，仍是右手系。镜像或 `IBOW` 的截面落在 x ≤ 0：`24381/1597`（R 8599.94、顺时针、
    /// 墙厚 600、`LMIRR`）是 [7999.94, 8599.94]，`24381/31100`（R 28100、逆时针、墙厚 800、`IBOW`）是
    /// [28100, 28900]。
    #[test]
    fn a_clockwise_wall_sits_outside_its_spine() {
        let sweep = -std::f64::consts::FRAC_PI_2;
        assert_eq!(
            circular_wall_radii(26500.0, [0.0, 1500.0], sweep.signum()),
            (26500.0, 28000.0)
        );
        assert_eq!(
            circular_wall_radii(26500.0, [0.0, 1500.0], -sweep.signum()),
            (25000.0, 26500.0)
        );
        assert_eq!(
            circular_wall_radii(8599.94, [-600.0, 0.0], -1.0),
            (8599.94 - 600.0, 8599.94)
        );
        assert_eq!(
            circular_wall_radii(28100.0, [-800.0, 0.0], 1.0),
            (28100.0, 28900.0)
        );
        let frame = wall_end_frame(DVec3::new(26500.0, 0.0, 0.0), 0.0, sweep.signum());
        let (x, y, z) = (
            frame.x_axis.truncate(),
            frame.y_axis.truncate(),
            frame.z_axis.truncate(),
        );
        assert!((x - DVec3::X).length() < 1e-12, "截面朝外：{x:?}");
        assert!(
            (z - DVec3::NEG_Y).length() < 1e-12,
            "Z 沿行进方向（顺时针在 0° 处朝 −Y）：{z:?}"
        );
        assert!((x.cross(y) - z).length() < 1e-12, "右手系");
    }

    /// 目录截面 → 圆弧墙截面：减 JUSL 线、`LMIRR` 沿 x 镜像。`/Single_leaf_wall` 的 `NA` / `OBOW` 在
    /// (0, 0)、`IBOW` 在 (厚, 0)（目录截面探针对 ams1112 / ams7997 全部 62 堵墙的读数）。
    #[test]
    fn wall_section_follows_the_jusl_line_and_lmirr() {
        let ring = [[0.0, 0.0], [800.0, 0.0], [800.0, 2400.0], [0.0, 2400.0]]
            .into_iter()
            .map(|[x, y]| crate::discretise::FilletVertex { x, y, frad: 0.0 })
            .collect::<Vec<_>>();
        let section = |justification: [f64; 2], mirrored: bool| {
            let profile = crate::section::SectionProfile {
                members: vec![crate::section::SectionMember {
                    source: crate::section::SectionSource::Spro { vertices: 4 },
                    member_refno: "0/0".into(),
                    rings: vec![ring.clone()],
                }],
                justification,
                justification_line: String::new(),
                justification_found: true,
                neutral_axis_spin: 0.0,
                level: 0,
                mirrored,
                notes: Vec::new(),
            };
            wall_section_from_profile(&profile).expect("vertices")
        };
        let height = [0.0, 2400.0];
        assert_eq!(section([0.0, 0.0], false).x, [0.0, 800.0]);
        assert_eq!(section([800.0, 0.0], false).x, [-800.0, 0.0]);
        assert_eq!(section([0.0, 0.0], true).x, [-800.0, 0.0]);
        assert_eq!(section([800.0, 0.0], true).x, [0.0, 800.0]);
        assert_eq!(section([800.0, 0.0], true).y, height);
        let ring = section([800.0, 0.0], false).ring();
        let twice_area: f64 = (0..ring.len())
            .map(|i| {
                let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
                a[0] * b[1] - b[0] * a[1]
            })
            .sum();
        assert!(twice_area > 0.0, "逆时针：{ring:?}");
    }

    /// JLDATU 站位：弧内按弧长走、沿墙向取该点切向（WF05 RR001 `/1RS05CC0013T`，`ZDIS` 3000，
    /// E3D `Q POS` (8243, −15323)、`Q ORI` `Z is E 28.279 N`）；越过终点从终点沿终点切向直线外推、
    /// 沿墙向钉在终点切向（WF03 RR001 `/1RS03LL0018T`，`ZDIS` 2440 > 弧长 2378，E3D
    /// `Z is E 9.0681 S`）。
    #[test]
    fn wall_joint_frame_follows_the_arc_inside_and_the_end_tangent_line_beyond() {
        // WF05 WALL 4：圆心原点、R 17400、起点 (5492.3, −16510.4)、终点 (13653.2, −10786.5)。
        let (center, radius) = (DVec3::ZERO, 17400.0);
        let a0 = (-16510.4f64).atan2(5492.3);
        let sweep = ((-10786.5f64).atan2(13653.2) - a0).rem_euclid(std::f64::consts::TAU);
        let inside = wall_joint_frame(center, radius, a0, sweep, 3000.0);
        let station = inside.transform_point3(DVec3::ZERO);
        assert!(
            (station.x - 8243.0).abs() < 1.0 && (station.y - -15323.0).abs() < 1.0,
            "{station:?}"
        );
        let along = inside.transform_vector3(DVec3::Z);
        let expected = DVec3::new(
            28.279f64.to_radians().cos(),
            28.279f64.to_radians().sin(),
            0.0,
        );
        assert!(along.abs_diff_eq(expected, 2e-4), "{along:?}");

        // WF03 WALL 4：三点弧 pt0 / thru / pt1（`docs/2026-08-12_live-test-ledger.md`）。
        let p0 = DVec3::new(-5058.21875, -16648.55664, 0.0);
        let pm = DVec3::new(-3909.413, -16955.131, 0.0);
        let p1 = DVec3::new(-2742.352, -17182.535, 0.0);
        let (center, radius, a0, sweep) = circular_spine_geometry(&[p0, pm, p1]).expect("arc");
        let length = radius * sweep;
        assert!((length - 2378.0).abs() < 1.0, "{length}");
        let beyond = wall_joint_frame(center, radius, a0, sweep, 2440.0);
        let along = beyond.transform_vector3(DVec3::Z);
        let end_tangent = DVec3::new(
            (-9.0681f64).to_radians().cos(),
            (-9.0681f64).to_radians().sin(),
            0.0,
        );
        assert!(
            along.abs_diff_eq(end_tangent, 2e-4),
            "沿墙向该钉在终点切向：{along:?}"
        );
        let station = beyond.transform_point3(DVec3::ZERO);
        let expected = p1 + end_tangent * (2440.0 - length);
        assert!(
            station.abs_diff_eq(expected, 0.05),
            "{station:?} vs {expected:?}"
        );
        // 弧上外推会差 0.11 mm——E3D 的 RVM 原点按直线外推对到 0.00 mm，按弧 0.11 mm。
        let on_arc_angle = a0 + 2440.0 / radius;
        let on_arc = center
            + DVec3::new(
                radius * on_arc_angle.cos(),
                radius * on_arc_angle.sin(),
                0.0,
            );
        assert!(
            (station - on_arc).length() > 0.05,
            "{station:?} vs {on_arc:?}"
        );
    }

    /// `setMitrePlanes` 的两道门：端向与切向垂直 / 平行都不斜接；其余法向指向体内。
    #[test]
    fn spine_mitre_plane_gates_perpendicular_and_parallel_and_points_into_the_body() {
        let p = DVec3::new(1.0, 2.0, 3.0);
        assert!(spine_mitre_plane(DVec3::ZERO, DVec3::X, p, true).is_none());
        assert!(
            spine_mitre_plane(DVec3::Y, DVec3::X, p, true).is_none(),
            "垂直"
        );
        assert!(
            spine_mitre_plane(DVec3::X, DVec3::X, p, true).is_none(),
            "平行"
        );
        assert!(
            spine_mitre_plane(-DVec3::X, DVec3::X, p, false).is_none(),
            "反向平行"
        );
        let slanted = DVec3::new(1.0, 1.0, 0.0).normalize();
        let start = spine_mitre_plane(-slanted, DVec3::X, p, true).expect("起点斜切");
        assert!(start.normal.dot(DVec3::X) > 0.0, "起点端法向朝行进方向");
        assert_eq!(start.point, p);
        let end = spine_mitre_plane(slanted, DVec3::X, p, false).expect("终点斜切");
        assert!(end.normal.dot(DVec3::X) < 0.0, "终点端法向逆行进方向");
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

    /// `DESP[4]` / `DESP[5]` 把套管往两头接长（[`sbfitting_axial_span`]）。ams1112 `/1RS03CC0028T` 的
    /// SBFI（`DESP[4]=50`）在 E3D TTY RVM 里是 z ∈ [−1600, 50]；开孔刀跟着一起接长，与套管同壳同位。
    #[test]
    fn sleeve_extensions_lengthen_both_the_sleeve_and_its_bore() {
        let mut notes = Vec::new();
        let datum_only = [
            150.0, 186.0, 18.0, 1600.0, 50.0, 0.0, 2000.0, -2320.0, 0.0, 0.0,
        ];
        let sleeve = sbfitting_from_desp(&datum_only, &mut notes).expect("valid SBFI DESP");
        let bounds = sleeve.bounding_box().expect("SBFI bounds");
        assert!((bounds.min()[2] + 1600.0).abs() < 1e-9);
        assert!((bounds.max()[2] - 50.0).abs() < 1e-9);

        let both_ends = [
            150.0, 186.0, 18.0, 1600.0, 50.0, 30.0, 2000.0, -2320.0, 0.0, 0.0,
        ];
        let sleeve = sbfitting_from_desp(&both_ends, &mut notes).expect("valid SBFI DESP");
        let bore = sbfitting_bore_from_desp(&both_ends).expect("valid SBFI DESP");
        for solid in [&sleeve, &bore] {
            let bounds = solid.bounding_box().expect("bounds");
            assert!((bounds.min()[2] + 1630.0).abs() < 1e-9);
            assert!((bounds.max()[2] - 50.0).abs() < 1e-9);
        }
    }

    /// 套管在宿主墙上开的洞 = 套管外圆那只实心八棱柱，与套管本体**同一组顶点、
    /// 同一处平移**。
    ///
    /// 判据是 ams1112 `1RS-WF03-W-C-RR001` 的逐点实测：RVM 里洞的内壁是内接于 `OD/2`
    /// 的八边形（采样点径向距离落在边心距 57.3 到顶点 62 之间），洞的 z 跨度与
    /// `DESP[1]` 逐字相同。洞若不与套管外壁同一组顶点（比如按真圆去切），
    /// 两者之间会差一个弦高，对拍上就是整圈几毫米的薄壳。
    #[test]
    fn a_sleeve_bores_its_own_outer_octagon_through_the_host() {
        let desp = [
            100.0, 124.0, 12.0, 1000.0, 0.0, 0.0, 1400.0, 3000.0, 0.0, 0.0,
        ];
        let mut notes = Vec::new();
        let sleeve = sbfitting_from_desp(&desp, &mut notes).expect("valid SBFI DESP");
        let bore = sbfitting_bore_from_desp(&desp).expect("valid SBFI DESP");

        // 洞是实心的：没有通孔，体积 = 内接八边形面积 × 长。
        assert_eq!(bore.genus(), 0, "开孔刀不该带套管自己的内孔");
        let octagon = 8.0 * 0.5 * 62.0 * 62.0 * (std::f64::consts::TAU / 8.0).sin();
        assert!((bore.volume() - octagon * 1000.0).abs() / (octagon * 1000.0) < 1e-6);
        assert!(bore.volume() > sleeve.volume(), "洞比环多一根内芯");

        // 同壳同位：包围盒与套管本体逐轴相同。
        let sleeve_bounds = sleeve.bounding_box().expect("sleeve bounds");
        let bore_bounds = bore.bounding_box().expect("bore bounds");
        for axis in 0..3 {
            assert!((sleeve_bounds.min()[axis] - bore_bounds.min()[axis]).abs() < 1e-9);
            assert!((sleeve_bounds.max()[axis] - bore_bounds.max()[axis]).abs() < 1e-9);
        }

        // 凑不齐 10 项的 DESP 连套管都建不出来，洞同样不许现编一个。
        assert!(sbfitting_bore_from_desp(&[100.0, 124.0, 12.0]).is_err());
    }

    #[test]
    fn sbfitting_alignment_matches_rvm_catalogue_axes() {
        // AMS 1112 SBFI 17496/124197: descriptor ZDIR becomes
        // (-0.258819,+0.965926,0). RVM maps catalogue X to -Z and keeps that
        // direction as catalogue Z.
        let m = sbfitting_alignment_from_zdir([-0.258819, -0.965926, 0.0]).expect("valid ZDIR");
        assert!((m.x_axis.truncate() - DVec3::NEG_Z).length() < 1e-12);
        assert!((m.z_axis.truncate() - DVec3::new(-0.258819, 0.965926, 0.0)).length() < 1e-6);

        // ORIMAT's singular branch: Z=-Y cannot cross the first Y candidate,
        // so it advances to global Z and obtains X=-X, Y=-Z.
        let cardinal = sbfitting_alignment_from_zdir([0.0, 1.0, 0.0]).expect("valid ZDIR");
        assert!((cardinal.x_axis.truncate() - DVec3::NEG_X).length() < 1e-12);
        assert!((cardinal.y_axis.truncate() - DVec3::NEG_Z).length() < 1e-12);
        assert!((cardinal.z_axis.truncate() - DVec3::NEG_Y).length() < 1e-12);
    }
}

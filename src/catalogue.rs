//! Design-member catalogue geometry.
//!
//! Route members are instances: the design record carries selection and
//! dimensions, while the reusable shape program lives under
//! `SPRE -> SCOM -> GMRE`.  This module evaluates that program without copying
//! the existing profile discretiser or solid builders.  The first proven slice
//! is Core3D `QSSEXT`: `SEXT -> SLOO -> SVER` becomes the same
//! [`crate::solid::extrude_rings`] body used by design-side extrusions.

use std::collections::HashMap;

use glam::{DMat3, DMat4, DVec3};
use manifold_csg::Manifold;

use e3d_io::db_element::DbElement;
use e3d_io::db_param_env::DbElementParamEnv;
use e3d_io::record::catalogue_eval::AttributeNumber;
use e3d_io::record::descriptor::DescriptorValue;

use crate::catalogue_point::{
    CatalogueAxis, catalogue_component, catalogue_entry_attribute, resolve_catalogue_axis,
};
use crate::category::{RepreDiscipline, repre_discipline};
use crate::discretise::{FilletVertex, flatten_ring};
use crate::elmodl::{DegenerateCataloguePart, GeneratedElement, GeometryId, ModelError};
use crate::primitive_instance::instantiate_box;
use crate::route::orimat;
use crate::solid::{
    SolidError, box_solid, circ_torus_solid, csg_cylinder_solid, cylinder_segments,
    dish_knuckle_radius, elliptical_dish_solid, extrude_rings, placed, pyramid_solid,
    rect_torus_solid, revolve_rings_profile_faceted, slope_ended_cylinder_solid, snout_solid,
    snout_solid_with_segments, spherical_dish_solid, unit_sphere_solid,
};
use crate::transform::world_matrix;

const GEOMETRY_REF: &str = "GMRE";
const AXIS_TOL: f64 = 1.0e-9;
const HEIGHT_TOL_MM: f64 = 1.0e-6;
const SEXT_TESSELLATION_VERSION: u32 = 1;
const SREV_TESSELLATION_VERSION: u32 = 2;
const SBOX_TESSELLATION_VERSION: u32 = 1;
const NSCY_AXIAL_OVERCUT_MM: f64 = 0.01;
const CYLINDER_TESSELLATION_VERSION: u32 = 1;
const TORUS_TESSELLATION_VERSION: u32 = 1;
const SPHERE_TESSELLATION_VERSION: u32 = 1;
const PYRAMID_TESSELLATION_VERSION: u32 = 1;
/// 2：目录 LSNO/SCON 正体从设计侧 SNOU 的 5 mm FacetGroup 口径改到 libgm 曲面口径
/// （`cylinder_segments(大端半径)`），HVAC BEND 法兰环 16 段 → 48 段。
const SNOUT_TESSELLATION_VERSION: u32 = 2;
const SLOPE_CYLINDER_TESSELLATION_VERSION: u32 = 1;
const DISH_TESSELLATION_VERSION: u32 = 1;
/// CRCATI 给圆盘臂的厚度（`unk_10B4E108` = 1.0）：SDIS 本体，以及 `PHEI = 0` 的 SDSH
/// 退化路，都是 `gm_CreateCylinder(PDIA / 2, 1.0)`。
const DISC_THICKNESS_MM: f64 = 1.0;

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, serde::Serialize)]
pub struct CatalogueMeshCacheStats {
    pub source_sext: usize,
    pub evaluated_sext: usize,
    pub source_sbox: usize,
    pub evaluated_sbox: usize,
    pub source_nscy: usize,
    pub evaluated_nscy: usize,
    pub source_nsbo: usize,
    pub evaluated_nsbo: usize,
    pub source_nlcy: usize,
    pub evaluated_nlcy: usize,
    pub source_nsex: usize,
    pub evaluated_nsex: usize,
    pub source_nssl: usize,
    pub evaluated_nssl: usize,
    pub source_nsds: usize,
    pub evaluated_nsds: usize,
    pub source_nssp: usize,
    pub evaluated_nssp: usize,
    pub source_srev: usize,
    pub evaluated_srev: usize,
    pub source_scyl: usize,
    pub evaluated_scyl: usize,
    pub source_lcyl: usize,
    pub evaluated_lcyl: usize,
    pub source_lpyr: usize,
    pub evaluated_lpyr: usize,
    pub source_scto: usize,
    pub evaluated_scto: usize,
    pub source_srto: usize,
    pub evaluated_srto: usize,
    pub source_ssph: usize,
    pub evaluated_ssph: usize,
    pub source_sslc: usize,
    pub evaluated_sslc: usize,
    pub source_sdsh: usize,
    pub evaluated_sdsh: usize,
    pub source_sdis: usize,
    pub evaluated_sdis: usize,
    /// 正体名下被表现级 / TUFL 筛掉的负体（`CSG_TreeBuilderCat::addNegatives` 对每把刀也过
    /// 一遍 `isWanted`）。不是失败、不落 note，只在这里记一笔，别让「刀少了」变成静默。
    pub display_filtered_negatives: usize,
    pub local_mesh_hits: usize,
    pub local_mesh_misses: usize,
    pub unique_placed_parts: usize,
    pub duplicate_placed_parts: usize,
}

/// Generation-scoped cache for evaluated catalogue profiles.
///
/// It deliberately stores member-local meshes only. World placement and final
/// member identity stay outside the cache, so equal catalogue shapes across
/// different design members reuse triangulation without aliasing their output
/// elements or transforms.
///
/// 设计侧环拉伸（PANE/GWALL/FLOOR/EXTR 及负体 NXTR）的同批复用池也挂在
/// 这里随批次同生共死，逻辑与统计都在 [`crate::extrusion_reuse`]。
#[derive(Default)]
pub struct CatalogueMeshCache {
    local_meshes: HashMap<CatalogueMeshKey, CachedCatalogueMesh>,
    /// 按 `CachedCatalogueMesh::id` 索引的三角面数，建体那一次记下来。
    ///
    /// 存在的理由只有一个：判「这件有没有成体」不该把 CSG 树逼出来。`Manifold` 的
    /// 变换与 compose 都是惰性的，一句 `num_tri()` 就会把本件在**局部系**先物化一遍，
    /// 而随后 `placed(.., world)` 再取网格时又物化一遍——同一批顶点算了两轮。
    /// 面数不随仿射变换改变，从这里读就够。
    mesh_tri_counts: Vec<usize>,
    stats: CatalogueMeshCacheStats,
    pub(crate) extrusions: crate::extrusion_reuse::ExtrusionMeshStore,
}

impl CatalogueMeshCache {
    pub fn stats(&self) -> CatalogueMeshCacheStats {
        self.stats
    }
}

impl CatalogueMeshCacheStats {
    pub(crate) fn checked_delta_from(self, before: Self) -> Self {
        macro_rules! delta {
            ($field:ident) => {
                self.$field
                    .checked_sub(before.$field)
                    .expect("catalogue cache statistics must be monotonic")
            };
        }

        Self {
            source_sext: delta!(source_sext),
            evaluated_sext: delta!(evaluated_sext),
            source_sbox: delta!(source_sbox),
            evaluated_sbox: delta!(evaluated_sbox),
            source_nscy: delta!(source_nscy),
            evaluated_nscy: delta!(evaluated_nscy),
            source_nsbo: delta!(source_nsbo),
            evaluated_nsbo: delta!(evaluated_nsbo),
            source_nlcy: delta!(source_nlcy),
            evaluated_nlcy: delta!(evaluated_nlcy),
            source_nsex: delta!(source_nsex),
            evaluated_nsex: delta!(evaluated_nsex),
            source_nssl: delta!(source_nssl),
            evaluated_nssl: delta!(evaluated_nssl),
            source_nsds: delta!(source_nsds),
            evaluated_nsds: delta!(evaluated_nsds),
            source_nssp: delta!(source_nssp),
            evaluated_nssp: delta!(evaluated_nssp),
            source_srev: delta!(source_srev),
            evaluated_srev: delta!(evaluated_srev),
            source_scyl: delta!(source_scyl),
            evaluated_scyl: delta!(evaluated_scyl),
            source_lcyl: delta!(source_lcyl),
            evaluated_lcyl: delta!(evaluated_lcyl),
            source_lpyr: delta!(source_lpyr),
            evaluated_lpyr: delta!(evaluated_lpyr),
            source_scto: delta!(source_scto),
            evaluated_scto: delta!(evaluated_scto),
            source_srto: delta!(source_srto),
            evaluated_srto: delta!(evaluated_srto),
            source_ssph: delta!(source_ssph),
            evaluated_ssph: delta!(evaluated_ssph),
            source_sslc: delta!(source_sslc),
            evaluated_sslc: delta!(evaluated_sslc),
            source_sdsh: delta!(source_sdsh),
            evaluated_sdsh: delta!(evaluated_sdsh),
            source_sdis: delta!(source_sdis),
            evaluated_sdis: delta!(evaluated_sdis),
            display_filtered_negatives: delta!(display_filtered_negatives),
            local_mesh_hits: delta!(local_mesh_hits),
            local_mesh_misses: delta!(local_mesh_misses),
            unique_placed_parts: delta!(unique_placed_parts),
            duplicate_placed_parts: delta!(duplicate_placed_parts),
        }
    }

    pub(crate) fn accumulate(&mut self, delta: Self) {
        macro_rules! add {
            ($field:ident) => {
                self.$field += delta.$field
            };
        }

        add!(source_sext);
        add!(evaluated_sext);
        add!(source_sbox);
        add!(evaluated_sbox);
        add!(source_nscy);
        add!(evaluated_nscy);
        add!(source_nsbo);
        add!(evaluated_nsbo);
        add!(source_nlcy);
        add!(evaluated_nlcy);
        add!(source_nsex);
        add!(evaluated_nsex);
        add!(source_nssl);
        add!(evaluated_nssl);
        add!(source_nsds);
        add!(evaluated_nsds);
        add!(source_nssp);
        add!(evaluated_nssp);
        add!(source_srev);
        add!(evaluated_srev);
        add!(source_scyl);
        add!(evaluated_scyl);
        add!(source_lcyl);
        add!(evaluated_lcyl);
        add!(source_lpyr);
        add!(evaluated_lpyr);
        add!(source_scto);
        add!(evaluated_scto);
        add!(source_srto);
        add!(evaluated_srto);
        add!(source_ssph);
        add!(evaluated_ssph);
        add!(source_sslc);
        add!(evaluated_sslc);
        add!(source_sdsh);
        add!(evaluated_sdsh);
        add!(source_sdis);
        add!(evaluated_sdis);
        add!(display_filtered_negatives);
        add!(local_mesh_hits);
        add!(local_mesh_misses);
        add!(unique_placed_parts);
        add!(duplicate_placed_parts);
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
struct SextMeshKey {
    tessellation_version: u32,
    height_bits: u64,
    ring_lengths: Vec<usize>,
    coordinate_bits: Vec<u64>,
}

/// SREV 的网格键：轮廓（与 SEXT 同一份采集）+ 扫角 + 名下负体。
/// 负体进键与 [`SboxMeshKey`] 同理：同轮廓不同刀的两个 SREV 不是同一张网格。
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
struct SrevMeshKey {
    tessellation_version: u32,
    angle_bits: u64,
    ring_lengths: Vec<usize>,
    coordinate_bits: Vec<u64>,
    cutters: Vec<CatalogueCutKey>,
}

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
enum CatalogueMeshKey {
    /// 无刀的 SEXT，也是 NSEX 刀自己的轮廓键（刀不再下钻负体，永远无刀）。
    Sext(SextMeshKey),
    /// 名下挂了负体的 SEXT：与 [`CatalogueMeshKey::Cut`] 同理，
    /// 同轮廓不同刀的两个 SEXT 不是同一张网格；无刀时退回 `Sext`，键不变。
    CutSext {
        base: SextMeshKey,
        cutters: Vec<CatalogueCutKey>,
    },
    Srev(SrevMeshKey),
    Sbox(SboxMeshKey),
    Cylinder {
        tessellation_version: u32,
        diameter_bits: u64,
        height_bits: u64,
    },
    /// 名下挂了负体的圆柱 / 棱锥 / 锥台 / 环面 / 球 / 碟 / 斜切圆柱 / 圆盘：
    /// 同一张正体网格配不同的刀不是同一张网格；无刀时退回 `base` 本身，键不变。
    /// `CSG_TreeBuilderCat::getCSGTree`（`0x1072F5D0`）对**每一个**有成员的正体都开
    /// `Combination(3)` 差集，不分正体 noun，所以这一层包装对所有正体臂通用。
    Cut {
        base: Box<CatalogueMeshKey>,
        cutters: Vec<CatalogueCutKey>,
    },
    Pyramid {
        tessellation_version: u32,
        parameter_bits: [u64; 7],
    },
    Snout {
        tessellation_version: u32,
        parameter_bits: [u64; 5],
    },
    CircularTorus {
        tessellation_version: u32,
        inner_bits: u64,
        outer_bits: u64,
        angle_bits: u64,
    },
    RectangularTorus {
        tessellation_version: u32,
        inner_bits: u64,
        outer_bits: u64,
        height_bits: u64,
        angle_bits: u64,
    },
    Sphere {
        tessellation_version: u32,
        segments: i32,
    },
    /// SSLC：半径、高度、四个斜切角（度，已按 CRCATI 折回 (−90°, 90°)）。
    SlopeCylinder {
        tessellation_version: u32,
        parameter_bits: [u64; 6],
    },
    /// SDSH：球形碟 / 折边碟。`PRAD` 只当开关，折边半径由 `CREDSH` 从 r/h 算出，
    /// 所以键里只有 r、h 与开关本身。
    Dish {
        tessellation_version: u32,
        knuckled: bool,
        radius_bits: u64,
        height_bits: u64,
    },
}

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
struct SboxMeshKey {
    tessellation_version: u32,
    dimension_bits: [u64; 3],
    cutters: Vec<CatalogueCutKey>,
}

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
enum CatalogueCutKey {
    Cylinder {
        diameter_bits: u64,
        height_bits: u64,
        transform_bits: [u64; 16],
    },
    Box {
        dimension_bits: [u64; 3],
        transform_bits: [u64; 16],
    },
    Pyramid {
        parameter_bits: [u64; 7],
        transform_bits: [u64; 16],
    },
    /// NSEX：与正体 SEXT 同一份轮廓键（环组 + 高度），外加放置矩阵。
    Extrusion {
        profile: SextMeshKey,
        transform_bits: [u64; 16],
    },
    /// NLSN / NSCO：与正体 LSNO / SCON 同一组参数（含负臂 +0.01 轴向过切后的高度）。
    Snout {
        parameter_bits: [u64; 5],
        transform_bits: [u64; 16],
    },
    /// NSCT：与正体 SCTO 同一组半径与扫角。
    CircularTorus {
        inner_bits: u64,
        outer_bits: u64,
        angle_bits: u64,
        transform_bits: [u64; 16],
    },
    /// NSRT：与正体 SRTO 同一组半径、高度与扫角。
    RectangularTorus {
        inner_bits: u64,
        outer_bits: u64,
        height_bits: u64,
        angle_bits: u64,
        transform_bits: [u64; 16],
    },
    /// NSRE：与正体 SREV 同一份轮廓与扫角（CRCATI 968617/643505 正负同臂）。
    Revolution {
        angle_bits: u64,
        ring_lengths: Vec<usize>,
        coordinate_bits: Vec<u64>,
        transform_bits: [u64; 16],
    },
    /// NSSP：与正体 SSPH 同一个单位球网格（CRCATI 701101/860747 正负同臂），
    /// 半径在放置矩阵的缩放里。
    Sphere {
        segments: i32,
        transform_bits: [u64; 16],
    },
    /// NSDS：与正体 SDSH 同一组 r / h / 折边开关（CRCATI 702883/908861 正负同臂）；
    /// `PHEI = 0` 的退化路是 1 mm 圆盘，走 [`CatalogueCutKey::Cylinder`]。
    Dish {
        knuckled: bool,
        radius_bits: u64,
        height_bits: u64,
        transform_bits: [u64; 16],
    },
    /// NSSL：与正体 SSLC 同一组半径、高度、四个斜切角（CRCATI 599770/782015 正负同臂）。
    SlopeCylinder {
        parameter_bits: [u64; 6],
        transform_bits: [u64; 16],
    },
}

struct CachedCatalogueMesh {
    id: usize,
    solid: Manifold,
}

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
struct PlacedPartKey {
    mesh_id: usize,
    transform_bits: [u64; 16],
}

struct EvaluatedSext {
    mesh_key: CatalogueMeshKey,
    rings: Vec<Vec<[f64; 2]>>,
    height: f64,
    transform: DMat4,
    /// 名下的成员级刀集，已回到 SEXT 局部系。[`evaluate_sext`] 只出轮廓（NSEX 刀走它，
    /// 刀不下钻负体），恒为空；正体走 [`evaluate_sext_body`] 才收集。
    cutters: Vec<EvaluatedCatalogueCut>,
    diagnostics: Vec<String>,
}

struct EvaluatedSrev {
    mesh_key: CatalogueMeshKey,
    rings: Vec<Vec<[f64; 2]>>,
    angle_deg: f64,
    cutters: Vec<EvaluatedCatalogueCut>,
    transform: DMat4,
    diagnostics: Vec<String>,
}

struct EvaluatedSbox {
    mesh_key: CatalogueMeshKey,
    dimensions: DVec3,
    cutters: Vec<EvaluatedCatalogueCut>,
    transform: DMat4,
    diagnostics: Vec<String>,
}

struct EvaluatedNscy {
    diameter: f64,
    height: f64,
    transform: DMat4,
}

struct EvaluatedCylinder {
    mesh_key: CatalogueMeshKey,
    diameter: f64,
    height: f64,
    transform: DMat4,
    cutters: Vec<EvaluatedCatalogueCut>,
    diagnostics: Vec<String>,
}

/// 一只 SCYL / LCYL 求值下来的两种结局。**「不出体」不等于「不出记录」**：零径或零高的
/// 那只 Core 不出体，E3D 的 RVM 驱动照样按 DRCYL 落一条记录，参数就是算出来的那一对
/// ——两种来路都见得到，一条都不少：
///
/// - 尺寸格子空着（`PDIA`/`PHEI` 是没填过的全零元组）：`/Copy-(2)-of-GV0406C` 的
///   `Cylinder[radius=0 height=-0]`，基准 80 根上这样的有 132 条；
/// - 参数真解成零高（`PDIA` 解出 114、`PHEI` 由 PARA 解出 0）：`/LA-0301` 第 45 条
///   `Cylinder[radius=57 height=-0]`，半径照留。
enum CylinderOutcome {
    Built(EvaluatedCylinder),
    Bodyless {
        transform: DMat4,
        radius: f64,
        height: f64,
    },
}

struct EvaluatedPyramid {
    mesh_key: CatalogueMeshKey,
    parameters: [f64; 7],
    transform: DMat4,
}

struct EvaluatedSnout {
    mesh_key: CatalogueMeshKey,
    parameters: [f64; 5],
    transform: DMat4,
}

struct EvaluatedSphere {
    mesh_key: CatalogueMeshKey,
    /// 单位球的放置：平移到球心，再按半径缩放。网格和布尔用这只。
    transform: DMat4,
    /// 记录矩阵：`ORIMAT(PAXI)`，平移在球心，不含半径缩放。E3D 的球记录 Z 是这条轴。
    record_frame: DMat4,
}

struct EvaluatedSlopeCylinder {
    mesh_key: CatalogueMeshKey,
    radius: f64,
    height: f64,
    /// `[xtop, ytop, xbot, ybot]`，度；相对 [`csrtax`] 标架的 X/Y。
    shear_deg: [f64; 4],
    transform: DMat4,
}

#[derive(Clone, Copy)]
enum EvaluatedDishBody {
    /// `gm_CreateSphericalDish(r, h)`。
    Spherical,
    /// `CREDSH` → `gm_CreateEllipticalDish(r, h, k)`，k 由 r/h 算出。
    Knuckled,
}

struct EvaluatedDish {
    mesh_key: CatalogueMeshKey,
    body: EvaluatedDishBody,
    radius: f64,
    height: f64,
    transform: DMat4,
}

/// QSDISH 的两条出路：有高度是碟，`PHEI = 0` 时 CRCATI 退到和 SDIS 同一条 1 mm 圆盘。
enum EvaluatedSdsh {
    Dish(EvaluatedDish),
    Disc(EvaluatedCylinder),
}

enum EvaluatedTorusKind {
    Circular,
    Rectangular { height: f64 },
}

struct EvaluatedTorus {
    mesh_key: CatalogueMeshKey,
    kind: EvaluatedTorusKind,
    inner: f64,
    outer: f64,
    angle_deg: f64,
    transform: DMat4,
}

enum EvaluatedCatalogueCut {
    Cylinder(EvaluatedNscy),
    Box {
        dimensions: DVec3,
        transform: DMat4,
    },
    Pyramid {
        parameters: [f64; 7],
        transform: DMat4,
    },
    /// NSEX 负挤出：轮廓与放置和正体 SEXT 同一个求值器给出。
    Extrusion {
        rings: Vec<Vec<[f64; 2]>>,
        height: f64,
        transform: DMat4,
    },
    /// NLSN / NSCO 负锥台：参数与放置和正体 LSNO / SCON 同一个求值器给出，
    /// 高度已带负臂的 +0.01 轴向过切（§2.3 圆柱/锥族）。
    Snout {
        parameters: [f64; 5],
        transform: DMat4,
    },
    /// NSCT / NSRT 负环面段：与正体 SCTO / SRTO 同一个求值器给出。
    Torus {
        kind: EvaluatedTorusKind,
        inner: f64,
        outer: f64,
        angle_deg: f64,
        transform: DMat4,
    },
    /// NSRE 负回转：轮廓、扫角与放置和正体 SREV 同一个求值器给出。
    Revolution {
        rings: Vec<Vec<[f64; 2]>>,
        angle_deg: f64,
        transform: DMat4,
    },
    /// NSSP 负球：与正体 SSPH 同一个求值器（`QSSPHE` → `gm_CreateSphere(PDIA/2)`），
    /// 单位球网格，半径进放置矩阵的缩放；CRCATI 的负臂**没有** +0.01 过切。
    Sphere {
        segments: i32,
        transform: DMat4,
    },
    /// NSDS 负碟：与正体 SDSH 同一个求值器（`QSDISH`，球形 / 折边两路），
    /// CRCATI 的负臂**没有** +0.01 过切；`PHEI = 0` 的退化路落 [`Self::Cylinder`]（1 mm 圆盘）。
    Dish {
        body: EvaluatedDishBody,
        radius: f64,
        height: f64,
        transform: DMat4,
    },
    /// NSSL 负斜切圆柱：与正体 SSLC 同一个求值器（`QSSLCY` + `CSRTAX` 标架 + 角度折回），
    /// CRCATI 的负臂**没有** +0.01 过切。
    SlopeCylinder {
        radius: f64,
        height: f64,
        /// `[xtop, ytop, xbot, ybot]`，度。
        shear_deg: [f64; 4],
        transform: DMat4,
    },
}

/// Result of evaluating one design member's catalogue program.
pub struct CatalogueMemberBuild {
    pub generated: Option<GeneratedElement>,
    /// Core rejects one bad catalogue primitive and continues the GMRE.  Keep
    /// those local failures visible instead of turning the whole member into a
    /// false failure or silently dropping them.
    pub diagnostics: Vec<String>,
    /// `generated` 为 `None` 时的**依据**：链断在哪一跳、还是目录件本来就不画。
    ///
    /// 「没出几何」在报告上必须与「漏了」分得开（原则 III）。ams8000 的 560 条 BRAN 体检
    /// （`evidence/20260902-ams8000-bran-audit`）里 101 只 MDS `…/SREF` ATTA 的 SCOM 没有
    /// GMRE、3 只 FTUB 没有 SPRE、1 只 FTUB 的 LCYL 零高——几何结论全对，报告里却一笔都
    /// 没有，从报告上看不出是有意不画还是漏了。有形状时恒为 `None`。
    pub no_shape: Option<String>,
}

/// 表现级（`REPRESENTATION LEVEL …`）按学科分六档，见 [`RepreDiscipline`]。
///
/// 每档一个数，进程内缓存：环境变量 [`RepreDiscipline::env_var`] 覆盖，否则
/// [`RepreDiscipline::default_level`]——ams7997 那批 TTY 基线的 `Q REPRE` 口径：`LEVEL 6`、
/// `LEVEL HVAC 6`，`PIPE / NOZZ / STRU / CABLE` 停在 `LEVINI` 的 0。以前这里只有一个数
/// （默认 6，`E3D_REPRE_LEVEL`），把管道构件也放在 6 级——COUP `/ABCF331DF` 两片 LEVE (6,8)
/// 的 LSNO「E3D 不画、我们画」就是这么来的；同一口径下风管的 32 件 (6,10) 又必须是 6 级，
/// 一个数管不住两边（`evidence/20260902-repre-level-tty-experiments/`）。GUI / addin 导出
/// （`C-IY-1R330-B.rvm`，`GENMODEL_RVM_LEVEL=6`）管道在 6 级，对拍那类基线设
/// `E3D_REPRE_LEVEL_PIPE=6`。
///
/// 设计侧负体的 DRWAND 门（`elmodl::design_leve_admits`）与目录成员的显示过滤读的是同一张表
/// （[`repre_level_of`]）——Core3D 里 MODCMP 交给 ELMODL 的就是同一个数，这里不许长出第二份。
pub(crate) fn repre_level_for(discipline: RepreDiscipline) -> u32 {
    static LEVELS: std::sync::OnceLock<[u32; RepreDiscipline::COUNT]> = std::sync::OnceLock::new();
    LEVELS.get_or_init(|| {
        let mut levels = [0u32; RepreDiscipline::COUNT];
        for discipline in RepreDiscipline::ALL {
            levels[discipline.index()] = std::env::var(discipline.env_var())
                .ok()
                .and_then(|value| value.trim().parse().ok())
                .unwrap_or_else(|| discipline.default_level());
        }
        levels
    })[discipline.index()]
}

/// 被建元素按 MODCMP 落的学科：noun 分派 + 属主链上是否有 `HVAC`（`IHVCOM` 的 `CLIMBS`）。
pub(crate) fn repre_discipline_of(el: &DbElement) -> Result<RepreDiscipline, ModelError> {
    let noun = el.element_type()?;
    Ok(repre_discipline(&noun, has_hvac_ancestor(el)))
}

/// 被建元素的表现级：[`repre_discipline_of`] 那一档的数。
pub(crate) fn repre_level_of(el: &DbElement) -> Result<u32, ModelError> {
    Ok(repre_level_for(repre_discipline_of(el)?))
}

/// `nounlib/IHVCOM` 的 `CLIMBS(HVAC)`：沿属主链找 `HVAC`。读不出属主（根、悬空、库未注册）
/// 按「不在风管下」——那是管道库的常态形态，不能因为它把整条管道链判成风管。
fn has_hvac_ancestor(el: &DbElement) -> bool {
    let mut current = el.clone();
    for _ in 0..64 {
        let owner = match current.owner() {
            Ok(Some(owner)) if owner.refno() != current.refno() => owner,
            _ => return false,
        };
        match owner.element_type() {
            Ok(noun) if noun.trim().eq_ignore_ascii_case("HVAC") => return true,
            Ok(_) => {}
            Err(_) => return false,
        }
        current = owner;
    }
    false
}

/// `REPRE OBST ON/OFF`——`csg_gml4/DR3DWA` / `DRWANT` 里 `OBST > 0 ∧ obst_on` 那一项的开关。
///
/// ams7997 那批 TTY 基线宏写的是 `repre obst off`；GUI / addin 导出（`C-IY-1R330-B.rvm`）开着：
/// 每只 FTUBE 在 RVM 里多一只 `OBST=1 / TUFL=false` 的净空盒（`SBOX 13244/51888`，
/// `PZLE = ATTRIB RPRO CLEA` = 100，坐在托盘顶上），就是它把 35 对 AABB 的 z 顶抬高 100 mm
/// （`evidence/20260902-repre-level-tty-experiments/` §9）。环境变量 `E3D_REPRE_OBST=on|off`，
/// 缺省 off（TTY 口径）；对拍 GUI / addin 基线时与 `E3D_REPRE_LEVEL_PIPE=6` 一起设。
/// `on` 对应 E3D 的一级障碍表现；`all` / `2` 才同时放行 `OBST=2`。AMS 的 addin RVM
/// 把 `RESERVED/OBST=1` 导出而不含 `OBSTRUCTION/OBST=2`，为这条等级边界提供了判据。
pub(crate) fn repre_obst_level() -> u32 {
    static LEVEL: std::sync::OnceLock<u32> = std::sync::OnceLock::new();
    *LEVEL.get_or_init(|| {
        match std::env::var("E3D_REPRE_OBST")
            .unwrap_or_default()
            .trim()
            .to_ascii_lowercase()
            .as_str()
        {
            "on" | "1" | "true" | "yes" => 1,
            "all" | "2" => 2,
            _ => 0,
        }
    })
}

pub(crate) fn repre_obst_on() -> bool {
    repre_obst_level() > 0
}

/// E3D 的几何集成员显示过滤（GTGEOM 侧 `sub_107149B0` 那一签：
/// `LEVE[2]/OBST/TVIS/BVIS/CLFL/TUFL`），落到装配路上按 `REPR TUBE ON / CL OFF / OBST OFF`
/// 这套导出口径的两条筛：
///
/// - **LEVE 区间**：`lo <= level` 且（`hi == 0` 视为不设上限，否则 `level <= hi`）。
///   证据：`C-IY-1R330-B.rvm`（addin 导出，level 6）里 `LEVE=[0,0]` 的 CABLEWAY、
///   `[1,10]` 的 BOARD/RESERVED 同时在场——`[0,0]` 若按闭区间读在 6 级必被排掉，
///   所以上限 0 只能读作「不设限」；`1RX07-LCT.rvm`（TTY 裸会话，level 0）里
///   `[1,10]` 全不在而 `[0,0]` 在，钉住下限按闭区间读。
/// - **TUFL == false 一律不画**（Paragon 参考手册 6.2/6.3 的字面口径：`REPRESENTATION`
///   的 ON 项与置 TRUE 的旗一致才画；`TUBE ON` 只放行 `TUFL=true`，`CL OFF` 把
///   `CLFL=true` 那一路关死，`OBST OFF` 不给 OBST 1/2 开后门）。理由标签按成因分三档，
///   只为账目可读：`obst`（OBST=2 硬障碍体）、`centreline-only`（CLFL=true 的中心线标记，
///   手册 TEE 例子给中心线小球 SSPH 用的正是 `OBST 0 TUFLA FALSE CLFL TRUE`）、
///   `tube-off`（其余，典型是 OBST=1 的 CLASH/预留盒）。
///
/// 证据（evidence/20260902-g01c-sslc-sdsh-sdis，ams7997 RVM，4871 个配对成员）：3074 个按
/// 图元导出的成员逐 kind 计数对上 E3D 的，原两旗联判（`OBST=2 ∧ TUFL=false`）2508、
/// 加 `centreline-only` 2972、`TUFL=false` 一律不画 **3034**；三档合起来没有一件是把 RVM 里
/// 有的藏掉。SDIS 233/233、LSNO 1142、SCON 128、LCYL 156、SSPH 64（`TUFL=false/CLFL=true`）
/// 与 SCYL 100、DAMP `/RVCD-CLASH` 盒（`OBST 1/TUFL=false/CLFL=false`）在 RVM 里一件都没有。
///
/// **`OBST` 不是「画不画」的旗**：`OBST=2`（硬障碍）正是普通实心构件的常态值。2026-09-14
/// 试过把 `OBST=2 ∧ TUFL=true` 也按障碍体收走，SLYK 80 个 ZONE 当场 26 个从 PASS 掉到 OFF，
/// `/Copy-of-装卸车PI` 一个 ZONE 就少了 1490 件（ELBOW / FLANGE / FILTER 的本体），
/// 8729 只图元被误筛。要判「这只图元画不画」只看 TUFL / CLFL / LEVE 这三样。
///
/// 曾经的反例已经消化：`C-IY` 基准里 `OBST=1/TUFL=false` 的净空盒被 E3D 导出，是因为那份
/// addin 导出开着一级 `REPRE OBST`（TTY 基线宏是 `repre obst off`）。DR3DWA / DRWANT 的公式是
/// `in_level ∧ (OBST>0 ∧ obst_level>=OBST ∨ TUFL ∧ tube_on ∨ CLFL ∧ cl_on)`——`TUFL=false`
/// 一律不画只是 `obst off / cl off` 下的特例；对拍托盘语料时设
/// `E3D_REPRE_OBST=on`，需要二级障碍体时设 `all`。
/// HANG 24381/177986 的 HELEMENT 把 `CLFL=true` 的三个 SCYL 和一个 SEXT 全部导出——
/// 它们 `TUFL=true`，本筛照画；"CLFL 即隐藏"仍然是错的读法。
///
/// 曾经的反例已经消化：`C-IY` 基准里 `OBST=1/TUFL=false` 的净空盒被 E3D 导出，是因为那份
/// addin 导出开着一级 `REPRE OBST`（TTY 基线宏是 `repre obst off`）——对拍托盘语料时设
/// `E3D_REPRE_OBST=on`，需要二级障碍体时设 `all`。
/// HANG 24381/177986 的 HELEMENT 把 `CLFL=true` 的三个 SCYL 和一个 SEXT 全部导出——
/// 它们 `OBST=0 ∧ TUFL=true`，本筛照画；"CLFL 即隐藏"仍然是错的读法。
///
/// `level` 是**成员**那一档的表现级（[`repre_level_of`]）：同一 GMSE 被管道构件与风管构件
/// 共用时，两边按各自的 `LEVEL PIPE` / `LEVEL HVAC` 筛。
pub(crate) fn display_filtered(
    primitive: &DbElement,
    level: u32,
) -> Result<Option<&'static str>, ModelError> {
    if let Some(DescriptorValue::RawWords(words) | DescriptorValue::WordArray(words)) =
        primitive.attribute("LEVE")?
        && let (Some(&lo), Some(&hi)) = (words.first(), words.get(1))
    {
        if lo > level || (hi != 0 && hi < level) {
            return Ok(Some("leve"));
        }
    }
    let obstruction = match primitive.attribute("OBST")? {
        Some(DescriptorValue::Word { raw, .. }) => raw,
        Some(DescriptorValue::Int(value)) => u32::try_from(value).unwrap_or(0),
        _ => 0,
    };
    if primitive.get_bool("TUFL")? != Some(false) {
        return Ok(None);
    }
    // DR3DWA / DRWANT 的 `OBST>0 ∧ obst_on` 项：`REPRE OBST ON` 时障碍体（OBST 1/2）照画。
    if obstruction > 0 && obstruction <= repre_obst_level() {
        return Ok(None);
    }
    Ok(Some(if obstruction == 2 {
        "obst"
    } else if primitive.get_bool("CLFL")? == Some(true) {
        "centreline-only"
    } else {
        "tube-off"
    }))
}

/// Filter optional catalogue parts controlled by route design properties.
///
/// Core's `csg_gml4/DRWANT` calls `GATLG1` for cable-way member types before
/// applying the ordinary representation flags.  The ACP tray catalogue uses
/// the primitive label (stored as its name suffix) together with `RPRO COVR`,
/// `OUTA`, and `OUTB`: zero means no cover, while the OUT flags document
/// `0: Have, 1: No`.  These controls are independent of whether a profile
/// expression happens to multiply a dimension by the flag.
fn application_filtered(
    primitive: &DbElement,
    env: &DbElementParamEnv,
) -> Result<Option<&'static str>, ModelError> {
    let Some(name) = primitive.stored_name()? else {
        return Ok(None);
    };
    let label = name.to_ascii_uppercase();
    if label.contains("COVER") && env.rpro("COVR").is_some_and(|value| value <= 0.0) {
        return Ok(Some("rpro-covr"));
    }
    if label.contains("L_SPLICE_PLATE") && env.rpro("OUTA").is_some_and(|value| value != 0.0) {
        return Ok(Some("rpro-outa"));
    }
    if label.contains("R_SPLICE_PLATE") && env.rpro("OUTB").is_some_and(|value| value != 0.0) {
        return Ok(Some("rpro-outb"));
    }
    Ok(None)
}

/// Build the visible catalogue geometry carried by one design route member.
///
/// A missing component/GMRE is a no-shape result. Primitive-local failures are
/// retained in [`CatalogueMemberBuild::diagnostics`] while the remaining
/// primitives continue, matching CRCATI's per-primitive error boundary.
pub fn model_catalogue_member(member: &DbElement) -> Result<CatalogueMemberBuild, ModelError> {
    model_catalogue_member_with_cache(member, &mut CatalogueMeshCache::default())
}

/// Cached variant used by full-tree generation. The uncached public wrapper
/// above keeps direct/incremental callers on the same evaluator semantics.
pub fn model_catalogue_member_with_cache(
    member: &DbElement,
    cache: &mut CatalogueMeshCache,
) -> Result<CatalogueMemberBuild, ModelError> {
    let _timing = crate::perf::catalogue_scope();
    // The DESP shortcut represents the level-0 physical body only. At higher
    // HVAC representation levels E3D also emits the catalogue's level-specific
    // primitives, so bypassing the catalogue would silently drop them.
    if repre_level_of(member)? == 0
        && let Some(build) = model_hvac_rectangular_basic(member)?
    {
        return Ok(build);
    }
    let chain = crate::perf::cat_chain_scope();
    let component = catalogue_component(member)?;
    drop(chain);
    let Some(component) = component else {
        // 两种没形状要分开记：没有入口属性是设计数据没给规格（E3D 同样不出几何）；有了
        // 却走不到 SCOM 是链断（SPCO 无 CATR/PRTREF、引用悬空、库没开），得指出从哪儿断。
        let entry = catalogue_entry_attribute(&member.element_type()?);
        let no_shape = match member.get_element(entry)? {
            None => format!("设计件无 {entry}：没有目录链，无几何（E3D 同样不出）"),
            Some(spec) => format!(
                "目录链从 {entry} {} 走不到 SCOM（SPCO 无 CATR/PRTREF 或引用悬空），无几何",
                spec.refno()
            ),
        };
        return Ok(CatalogueMemberBuild {
            generated: None,
            diagnostics: Vec::new(),
            no_shape: Some(no_shape),
        });
    };
    let Some(geometry) = component.get_element(GEOMETRY_REF)? else {
        // 目录件没有 GMRE 就是「按设计不画」——MDS 支撑参考（`/SPECIAL.500`，PTSE
        // `/PTSE-NODRAW`）整族如此，E3D 一件都不画。
        return Ok(CatalogueMemberBuild {
            generated: None,
            diagnostics: Vec::new(),
            no_shape: Some(format!(
                "目录件 {}{} 无 GMRE：按设计不画，无几何",
                component.refno(),
                component
                    .stored_name()?
                    .map(|name| format!(" {name}"))
                    .unwrap_or_default()
            )),
        });
    };
    let env = crate::catalogue_point::member_env(member, &component);
    let mut solids = Vec::new();
    let mut ignored_lines = 0usize;
    let mut sext_total = 0usize;
    let mut zero_height = 0usize;
    // SEXT 之外各臂求值返回 `Ok(None)` 的次数：零高 / 退化，与 Core 一样不出体。此前这一档
    // 一笔不记，一只 LCYL 全零高的 FTUB 在报告上与「漏建」长得一样。
    let mut degenerate_other = 0usize;
    let mut primitives = 0usize;
    // 放置 → 先放那一块在 `solids` 里的下标；重复放置在它身上记 `repeats`（见 `place_part_once`）。
    let mut placed_parts: HashMap<PlacedPartKey, usize> = HashMap::new();
    // 零尺寸的那几只：不进 `solids`（没体可建），但要跟着写记录。
    let mut degenerate_parts: Vec<DegenerateCataloguePart> = Vec::new();
    let mut member_duplicates = 0usize;
    let mut catalogue_negatives = 0usize;
    // 过滤记录与 diagnostics 分开收：`failed` 的账目 = diagnostics.len()，
    // 被表现级筛掉不是失败，不能混进去。
    let mut display_filter_notes = Vec::new();
    let mut diagnostics = Vec::new();
    let mut saw_display_primitive = false;
    // 表现级按成员的学科取（MODCMP：管道构件 `LEVEL PIPE`、风管 `LEVEL HVAC`…），整只成员一个数。
    let member_discipline = repre_discipline_of(member)?;
    let member_level = repre_level_for(member_discipline);

    let prims_timing = crate::perf::cat_prims_scope();
    for primitive in geometry.members()? {
        primitives += 1;
        let primitive_noun = primitive.element_type()?;
        // LINE / SLINE 在 CRCATI 里同走 `gml_bkend/CRCURV`：出曲线不出体，只记账。
        let curve_only = primitive_noun == "LINE" || primitive_noun == "SLINE";
        if !curve_only && let Some(reason) = application_filtered(&primitive, &env)? {
            display_filter_notes.push(format!(
                "catalogue primitive {} noun {primitive_noun} application-filtered by {reason}",
                primitive.refno(),
            ));
            continue;
        }
        // LINE 成员本来就只记账不建体，不进显示过滤，保住 centre_lines 的账目口径。
        if !curve_only && let Some(reason) = display_filtered(&primitive, member_level)? {
            display_filter_notes.push(format!(
                "catalogue primitive {} noun {primitive_noun} display-filtered by {reason} (repre {} {member_level}, obst {})",
                primitive.refno(),
                member_discipline.command(),
                if repre_obst_on() { "on" } else { "off" },
            ));
            continue;
        }
        if !curve_only {
            saw_display_primitive = true;
        }
        match primitive_noun.as_str() {
            "SEXT" => {
                sext_total += 1;
                cache.stats.source_sext += 1;
                match evaluate_sext_body(member, &component, &primitive, &env, cache) {
                    Ok(Some(mut evaluated)) => {
                        cache.stats.evaluated_sext += 1;
                        catalogue_negatives += evaluated.cutters.len();
                        diagnostics.append(&mut evaluated.diagnostics);
                        let mesh_id = match cached_mesh_id(cache, &evaluated.mesh_key, || {
                            build_sext(&evaluated)
                        }) {
                            Ok(mesh_id) => mesh_id,
                            Err(error) => {
                                diagnostics.push(format!(
                                    "catalogue primitive {} skipped by Core-style local failure: {error}",
                                    primitive.refno()
                                ));
                                continue;
                            }
                        };
                        if !place_part_once(
                            cache,
                            &mut placed_parts,
                            &mut solids,
                            &primitive_noun,
                            &evaluated.mesh_key,
                            mesh_id,
                            evaluated.transform,
                        ) {
                            member_duplicates += 1;
                        }
                    }
                    Ok(None) => zero_height += 1,
                    Err(error) => diagnostics.push(format!(
                        "catalogue primitive {} skipped by Core-style local failure: {error}",
                        primitive.refno()
                    )),
                }
            }
            "SREV" => {
                cache.stats.source_srev += 1;
                match evaluate_srev(member, &component, &primitive, &env, cache) {
                    Ok(Some(mut evaluated)) => {
                        cache.stats.evaluated_srev += 1;
                        catalogue_negatives += evaluated.cutters.len();
                        diagnostics.append(&mut evaluated.diagnostics);
                        let mesh_id = match cached_mesh_id(cache, &evaluated.mesh_key, || {
                            build_srev(&evaluated)
                        }) {
                            Ok(mesh_id) => mesh_id,
                            Err(error) => {
                                diagnostics.push(format!(
                                    "catalogue primitive {} skipped by Core-style local failure: {error}",
                                    primitive.refno()
                                ));
                                continue;
                            }
                        };
                        if !place_part_once(
                            cache,
                            &mut placed_parts,
                            &mut solids,
                            &primitive_noun,
                            &evaluated.mesh_key,
                            mesh_id,
                            evaluated.transform,
                        ) {
                            member_duplicates += 1;
                        }
                    }
                    Ok(None) => degenerate_other += 1,
                    Err(error) => diagnostics.push(format!(
                        "catalogue primitive {} skipped by Core-style local failure: {error}",
                        primitive.refno()
                    )),
                }
            }
            "SBOX" => {
                cache.stats.source_sbox += 1;
                match evaluate_sbox(member, &component, &primitive, &env, cache) {
                    Ok(Some(mut evaluated)) => {
                        cache.stats.evaluated_sbox += 1;
                        catalogue_negatives += evaluated.cutters.len();
                        diagnostics.append(&mut evaluated.diagnostics);
                        let mesh_id = match cached_mesh_id(cache, &evaluated.mesh_key, || {
                            build_sbox(&evaluated)
                        }) {
                            Ok(mesh_id) => mesh_id,
                            Err(error) => {
                                diagnostics.push(format!(
                                    "catalogue primitive {} skipped by Core-style local failure: {error}",
                                    primitive.refno()
                                ));
                                continue;
                            }
                        };
                        if !place_part_once(
                            cache,
                            &mut placed_parts,
                            &mut solids,
                            &primitive_noun,
                            &evaluated.mesh_key,
                            mesh_id,
                            evaluated.transform,
                        ) {
                            member_duplicates += 1;
                        }
                    }
                    Ok(None) => degenerate_other += 1,
                    Err(error) => diagnostics.push(format!(
                        "catalogue primitive {} skipped by Core-style local failure: {error}",
                        primitive.refno()
                    )),
                }
            }
            "LPYR" => {
                cache.stats.source_lpyr += 1;
                match evaluate_lpyr(member, &component, &primitive, &env) {
                    Ok(Some(evaluated)) => {
                        cache.stats.evaluated_lpyr += 1;
                        let (mesh_key, cutters, mut cut_diagnostics) = owned_negatives_of(
                            member,
                            &component,
                            &primitive,
                            &env,
                            cache,
                            evaluated.mesh_key,
                            evaluated.transform,
                        );
                        catalogue_negatives += cutters.len();
                        diagnostics.append(&mut cut_diagnostics);
                        let outcome = place_cached_part(
                            cache,
                            &mut placed_parts,
                            &mut solids,
                            &primitive_noun,
                            &mesh_key,
                            evaluated.transform,
                            || {
                                let [xbot, ybot, xtop, ytop, height, xoff, yoff] =
                                    evaluated.parameters;
                                subtract_owned_negatives(
                                    pyramid_solid(xbot, ybot, xtop, ytop, height, xoff, yoff)?,
                                    &cutters,
                                    "catalogue LPYR after negatives",
                                )
                            },
                        );
                        match outcome {
                            Ok(true) => {}
                            Ok(false) => member_duplicates += 1,
                            Err(error) => diagnostics.push(local_failure(&primitive, &error)),
                        }
                    }
                    Ok(None) => degenerate_other += 1,
                    Err(error) => diagnostics.push(local_failure(&primitive, &error)),
                }
            }
            noun @ ("LSNO" | "SCON") => {
                match evaluate_catalogue_snout(member, &component, &primitive, &env, noun) {
                    Ok(Some(evaluated)) => {
                        let (mesh_key, cutters, mut cut_diagnostics) = owned_negatives_of(
                            member,
                            &component,
                            &primitive,
                            &env,
                            cache,
                            evaluated.mesh_key,
                            evaluated.transform,
                        );
                        catalogue_negatives += cutters.len();
                        diagnostics.append(&mut cut_diagnostics);
                        let outcome = place_cached_part(
                            cache,
                            &mut placed_parts,
                            &mut solids,
                            &primitive_noun,
                            &mesh_key,
                            evaluated.transform,
                            || {
                                let [dbot, dtop, height, xoff, yoff] = evaluated.parameters;
                                // libgm `GM_Snout` 走曲面口径（与 GM_Cylinder 同一条 0.5 mm 规则，
                                // 喂大端半径），不是设计侧 SNOU FacetGroup 那条 5 mm。
                                subtract_owned_negatives(
                                    snout_solid_with_segments(
                                        dbot,
                                        dtop,
                                        height,
                                        xoff,
                                        yoff,
                                        cylinder_segments(dbot.max(dtop) * 0.5),
                                    )?,
                                    &cutters,
                                    "catalogue snout after negatives",
                                )
                            },
                        );
                        match outcome {
                            Ok(true) => {}
                            Ok(false) => member_duplicates += 1,
                            Err(error) => diagnostics.push(local_failure(&primitive, &error)),
                        }
                    }
                    Ok(None) => degenerate_other += 1,
                    Err(error) => diagnostics.push(local_failure(&primitive, &error)),
                }
            }
            noun @ ("SCYL" | "LCYL") => {
                if noun == "SCYL" {
                    cache.stats.source_scyl += 1;
                } else {
                    cache.stats.source_lcyl += 1;
                }
                match evaluate_catalogue_cylinder(member, &component, &primitive, &env, noun, cache)
                {
                    Ok(CylinderOutcome::Built(mut evaluated)) => {
                        catalogue_negatives += evaluated.cutters.len();
                        diagnostics.append(&mut evaluated.diagnostics);
                        if noun == "SCYL" {
                            cache.stats.evaluated_scyl += 1;
                        } else {
                            cache.stats.evaluated_lcyl += 1;
                        }
                        let mesh_id = match cached_mesh_id(cache, &evaluated.mesh_key, || {
                            build_catalogue_cylinder(&evaluated)
                        }) {
                            Ok(mesh_id) => mesh_id,
                            Err(error) => {
                                diagnostics.push(format!(
                                    "catalogue primitive {} skipped by Core-style local failure: {error}",
                                    primitive.refno()
                                ));
                                continue;
                            }
                        };
                        if !place_part_once(
                            cache,
                            &mut placed_parts,
                            &mut solids,
                            &primitive_noun,
                            &evaluated.mesh_key,
                            mesh_id,
                            evaluated.transform,
                        ) {
                            member_duplicates += 1;
                        }
                    }
                    Ok(CylinderOutcome::Bodyless {
                        transform,
                        radius,
                        height,
                    }) => {
                        degenerate_other += 1;
                        degenerate_parts.push(DegenerateCataloguePart {
                            noun: primitive_noun.clone(),
                            local_transform: transform,
                            radius_mm: radius,
                            height_mm: height,
                        });
                    }
                    Err(error) => diagnostics.push(format!(
                        "catalogue primitive {} skipped by Core-style local failure: {error}",
                        primitive.refno()
                    )),
                }
            }
            noun @ ("SCTO" | "SRTO") => {
                if noun == "SCTO" {
                    cache.stats.source_scto += 1;
                } else {
                    cache.stats.source_srto += 1;
                }
                match evaluate_catalogue_torus(member, &component, &primitive, &env, noun) {
                    Ok(Some(evaluated)) => {
                        if noun == "SCTO" {
                            cache.stats.evaluated_scto += 1;
                        } else {
                            cache.stats.evaluated_srto += 1;
                        }
                        let (mesh_key, cutters, mut cut_diagnostics) = owned_negatives_of(
                            member,
                            &component,
                            &primitive,
                            &env,
                            cache,
                            evaluated.mesh_key,
                            evaluated.transform,
                        );
                        catalogue_negatives += cutters.len();
                        diagnostics.append(&mut cut_diagnostics);
                        let outcome = place_cached_part(
                            cache,
                            &mut placed_parts,
                            &mut solids,
                            &primitive_noun,
                            &mesh_key,
                            evaluated.transform,
                            || {
                                let body = match evaluated.kind {
                                    EvaluatedTorusKind::Circular => circ_torus_solid(
                                        evaluated.inner,
                                        evaluated.outer,
                                        evaluated.angle_deg,
                                    )?,
                                    EvaluatedTorusKind::Rectangular { height } => rect_torus_solid(
                                        evaluated.inner,
                                        evaluated.outer,
                                        height,
                                        evaluated.angle_deg,
                                    )?,
                                };
                                subtract_owned_negatives(
                                    body,
                                    &cutters,
                                    "catalogue torus after negatives",
                                )
                            },
                        );
                        match outcome {
                            Ok(true) => {}
                            Ok(false) => member_duplicates += 1,
                            Err(error) => diagnostics.push(format!(
                                "{}; torus inner={} outer={} angle_deg={}",
                                local_failure(&primitive, &error),
                                evaluated.inner,
                                evaluated.outer,
                                evaluated.angle_deg,
                            )),
                        }
                    }
                    Ok(None) => degenerate_other += 1,
                    Err(error) => diagnostics.push(local_failure(&primitive, &error)),
                }
            }
            "SSPH" => {
                cache.stats.source_ssph += 1;
                match evaluate_catalogue_sphere(member, &component, &primitive, &env) {
                    Ok(Some(evaluated)) => {
                        cache.stats.evaluated_ssph += 1;
                        let segments = match &evaluated.mesh_key {
                            CatalogueMeshKey::Sphere { segments, .. } => *segments,
                            _ => unreachable!("sphere evaluator returned a non-sphere key"),
                        };
                        // 单位球的放置矩阵带半径缩放，刀随 `transform.inverse()` 一起缩进单位球
                        // 系再做差集，等比缩放不改变布尔结果。
                        let (mesh_key, cutters, mut cut_diagnostics) = owned_negatives_of(
                            member,
                            &component,
                            &primitive,
                            &env,
                            cache,
                            evaluated.mesh_key,
                            evaluated.transform,
                        );
                        catalogue_negatives += cutters.len();
                        diagnostics.append(&mut cut_diagnostics);
                        let outcome = place_cached_part(
                            cache,
                            &mut placed_parts,
                            &mut solids,
                            &primitive_noun,
                            &mesh_key,
                            evaluated.transform,
                            || {
                                subtract_owned_negatives(
                                    unit_sphere_solid(segments)?,
                                    &cutters,
                                    "catalogue SSPH after negatives",
                                )
                            },
                        );
                        match outcome {
                            Ok(true) => {
                                // 网格矩阵带半径缩放；记录要的是 ORIMAT(PAXI)，不是那只缩放。
                                if let Some(part) = solids.last_mut() {
                                    part.frame = evaluated.record_frame;
                                }
                            }
                            Ok(false) => member_duplicates += 1,
                            Err(error) => diagnostics.push(local_failure(&primitive, &error)),
                        }
                    }
                    Ok(None) => degenerate_other += 1,
                    Err(error) => diagnostics.push(local_failure(&primitive, &error)),
                }
            }
            "SSLC" => {
                cache.stats.source_sslc += 1;
                match evaluate_sslc(member, &component, &primitive, &env) {
                    Ok(Some(evaluated)) => {
                        cache.stats.evaluated_sslc += 1;
                        let [xtop, ytop, xbot, ybot] = evaluated.shear_deg;
                        let (mesh_key, cutters, mut cut_diagnostics) = owned_negatives_of(
                            member,
                            &component,
                            &primitive,
                            &env,
                            cache,
                            evaluated.mesh_key,
                            evaluated.transform,
                        );
                        catalogue_negatives += cutters.len();
                        diagnostics.append(&mut cut_diagnostics);
                        let outcome = place_cached_part(
                            cache,
                            &mut placed_parts,
                            &mut solids,
                            &primitive_noun,
                            &mesh_key,
                            evaluated.transform,
                            || {
                                subtract_owned_negatives(
                                    slope_ended_cylinder_solid(
                                        evaluated.radius,
                                        evaluated.height,
                                        xtop,
                                        ytop,
                                        xbot,
                                        ybot,
                                    )?,
                                    &cutters,
                                    "catalogue SSLC after negatives",
                                )
                            },
                        );
                        match outcome {
                            Ok(true) => {}
                            Ok(false) => member_duplicates += 1,
                            Err(error) => diagnostics.push(local_failure(&primitive, &error)),
                        }
                    }
                    Ok(None) => degenerate_other += 1,
                    Err(error) => diagnostics.push(local_failure(&primitive, &error)),
                }
            }
            "SDSH" => {
                cache.stats.source_sdsh += 1;
                match evaluate_sdsh(member, &component, &primitive, &env) {
                    Ok(Some(evaluated)) => {
                        cache.stats.evaluated_sdsh += 1;
                        let (base_key, transform) = match &evaluated {
                            EvaluatedSdsh::Dish(dish) => (dish.mesh_key.clone(), dish.transform),
                            EvaluatedSdsh::Disc(disc) => (disc.mesh_key.clone(), disc.transform),
                        };
                        let (mesh_key, cutters, mut cut_diagnostics) = owned_negatives_of(
                            member, &component, &primitive, &env, cache, base_key, transform,
                        );
                        catalogue_negatives += cutters.len();
                        diagnostics.append(&mut cut_diagnostics);
                        let outcome = place_cached_part(
                            cache,
                            &mut placed_parts,
                            &mut solids,
                            &primitive_noun,
                            &mesh_key,
                            transform,
                            || {
                                let body = match &evaluated {
                                    EvaluatedSdsh::Dish(dish) => build_dish(dish)?,
                                    EvaluatedSdsh::Disc(disc) => {
                                        csg_cylinder_solid(disc.diameter, disc.height)?
                                    }
                                };
                                subtract_owned_negatives(
                                    body,
                                    &cutters,
                                    "catalogue SDSH after negatives",
                                )
                            },
                        );
                        match outcome {
                            Ok(true) => {}
                            Ok(false) => member_duplicates += 1,
                            Err(error) => diagnostics.push(local_failure(&primitive, &error)),
                        }
                    }
                    Ok(None) => degenerate_other += 1,
                    Err(error) => diagnostics.push(local_failure(&primitive, &error)),
                }
            }
            "SDIS" => {
                cache.stats.source_sdis += 1;
                match evaluate_sdis(member, &component, &primitive, &env) {
                    Ok(Some(disc)) => {
                        cache.stats.evaluated_sdis += 1;
                        let (mesh_key, cutters, mut cut_diagnostics) = owned_negatives_of(
                            member,
                            &component,
                            &primitive,
                            &env,
                            cache,
                            disc.mesh_key,
                            disc.transform,
                        );
                        catalogue_negatives += cutters.len();
                        diagnostics.append(&mut cut_diagnostics);
                        let outcome = place_cached_part(
                            cache,
                            &mut placed_parts,
                            &mut solids,
                            &primitive_noun,
                            &mesh_key,
                            disc.transform,
                            || {
                                subtract_owned_negatives(
                                    csg_cylinder_solid(disc.diameter, disc.height)?,
                                    &cutters,
                                    "catalogue SDIS after negatives",
                                )
                            },
                        );
                        match outcome {
                            Ok(true) => {}
                            Ok(false) => member_duplicates += 1,
                            Err(error) => diagnostics.push(local_failure(&primitive, &error)),
                        }
                    }
                    Ok(None) => degenerate_other += 1,
                    Err(error) => diagnostics.push(local_failure(&primitive, &error)),
                }
            }
            // GMRE LINE entries are centreline metadata, not visible solids. CRCATI
            // (`0x10345ACC`) 把 LINE(640317) 与 SLINE(3471112) 送进同一个 `gml_bkend/CRCURV`
            // (`0x103474C7`)——出的是曲线不是实体，两个 noun 同一口径。此前只认 LINE，
            // SLINE 会落 `is not implemented`，把一条 E3D 也不出体的线记成欠账。
            "LINE" | "SLINE" => ignored_lines += 1,
            unsupported => diagnostics.push(format!(
                "catalogue primitive {} noun {unsupported} is not implemented",
                primitive.refno()
            )),
        }
    }
    drop(prims_timing);
    let display_filtered_members = display_filter_notes.len();
    if solids.is_empty() {
        // GMRE 在、图元一个都没成体：把每个图元的去向数出来，读报告的人不用再翻目录。
        let no_shape = format!(
            "目录件 {} 的 GMRE {} 共 {primitives} 个图元无一成体：display-filtered {display_filtered_members} / \
             零高或退化 {}（SEXT {zero_height}）/ 局部失败 {} / LINE {ignored_lines} / 重复放置 {member_duplicates}",
            component.refno(),
            geometry.refno(),
            zero_height + degenerate_other,
            diagnostics.len(),
        );
        diagnostics.extend(display_filter_notes);
        return Ok(CatalogueMemberBuild {
            generated: None,
            diagnostics,
            no_shape: Some(no_shape),
        });
    }

    let compose_timing = crate::perf::cat_compose_scope();
    // 「本件有没有成体」查缓存里记下的面数，不问 `local`：问一句就把局部系那一遍
    // 物化出来了，而这批顶点马上还要在世界系再算一遍。`placed_parts` 与 `solids`
    // 逐个对应（重复放置只在先放那一块上记了一笔，没另放体），面数相加就是汇总后的面数。
    let assembly_tris: usize = placed_parts
        .keys()
        .map(|part| cache.mesh_tri_counts[part.mesh_id])
        .sum();
    if assembly_tris == 0 {
        return Err(ModelError::Solid(SolidError::EmptyManifold {
            what: "catalogue assembly",
        }));
    }
    // 一件图元的「汇总」就是它自己：`Manifold::compose` 走的是从头重建 Impl 那条路
    // （重算半边、重排顶点、重算包围盒），对单件是净浪费，而且它是**急切**的——
    // 那一下就把惰性变换逼成了真顶点。ams8000 每件平均 1.18 个图元。
    let local = match solids.as_slice() {
        [only] => only.solid.clone(),
        many => Manifold::compose(
            &many
                .iter()
                .map(|part| part.solid.clone())
                .collect::<Vec<_>>(),
        ),
    };
    // 目录件自己的 `NGMR`（→ NGMS）是给**宿主**扣的刀（[`catalogue_host_cuts`]：柱底板让柱让位、
    // 门窗在墙上开孔、关节切梁端），**不扣它自己**。2026-09-28 `6ff5dac` 曾在这里按 mode-0 NAPP 门
    // 把它扣到组装好的正体上；aps250110 的 E3D TTY RVM 说不对——57 只墙上门窗里 50 只、50 只
    // FIXING 里 8 只的正体会被自己的 NGMS 扣掉一截（门 `/IS-SRO-NS/900x2045` 的 NSBO 是整个
    // 门洞，扣完只剩门扇伸出墙外的那一角；檐口加腋节点 `2013286654/208` 扣完只剩 31 mm 一片），
    // 而 RVM 里 107 只全按**未扣**的 `parts` 逐件对上。对拍此前看不出来是因为 `mesh()` 走 `parts`；
    // 它唯一露头的地方是正体被整只扣空时的 `EmptyManifold`——ams7997 62 只 STWALL 上的
    // `/W-HOLE` 墙洞 FITT 因此整件 failed。
    let world = catalogue_member_world(member)?;
    let noun = member.element_type()?;
    // E3D RVM keeps the visible catalogue primitives as independent display
    // components, including overlapping ones. Composing an AMS VALV's 18 SCYL
    // and 2 LSNO parts changed three occupied ray intervals and lost 26.95% of
    // the RVM component-sum volume. Preserve the already filtered parts here;
    // `solid` remains available to callers that explicitly need one manifold.
    let preserve_components = preserves_catalogue_components(saw_display_primitive);
    let parts = preserve_components.then(|| {
        solids
            .iter()
            .map(|part| placed(&part.solid, world))
            .collect::<Vec<_>>()
    });
    // 与 `parts` 逐块对齐的出身。RVM 的记录种类照它挑写出器，见
    // [`crate::elmodl::GeneratedElement::part_catalogue_nouns`]。
    let part_catalogue_nouns = if preserve_components {
        solids.iter().map(|part| part.noun.clone()).collect()
    } else {
        Vec::new()
    };
    // 与 `parts` 逐块对齐的记录事实：同一放置出现了几次，RVM 就写几条（体只有一块）；
    // 放置标架，记录的矩阵照它写（见 [`crate::elmodl::CataloguePartRecord`]）。
    let part_record_repeats = if preserve_components {
        solids
            .iter()
            .map(|part| crate::elmodl::CataloguePartRecord {
                repeats: part.repeats,
                frame: Some(part.frame),
                knuckled_dish: part.knuckled_dish,
            })
            .collect()
    } else {
        Vec::new()
    };
    // 零尺寸记录跟 `parts` 同一条命：整只元素合成一条时，多挂一条零尺寸的没有对应物。
    if !preserve_components {
        degenerate_parts.clear();
    }
    let world_solid = placed(&local, world);
    drop(compose_timing);
    let notes = vec![format!(
        "catalogue assembly: built={} sext_total={} sext_zero_height={} degenerate_other={} zero_size_records={} failed={} centre_lines={} display_filtered={} duplicate_placements={} component={}",
        solids.len(),
        sext_total,
        zero_height,
        degenerate_other,
        degenerate_parts.len(),
        diagnostics.len(),
        ignored_lines,
        display_filtered_members,
        member_duplicates,
        component.refno()
    )];
    diagnostics.extend(display_filter_notes);
    Ok(CatalogueMemberBuild {
        no_shape: None,
        generated: Some(GeneratedElement {
            geometry_id: GeometryId::Element {
                refno: member.refno().to_string(),
            },
            refno: member.refno(),
            noun,
            is_invalid_tubi: false,
            name: member.stored_name()?,
            solid: world_solid,
            parts,
            part_catalogue_nouns,
            part_record_repeats,
            degenerate_parts,
            world,
            primitive_instance: None,
            analytic_primitive: None,
            negatives_applied: catalogue_negatives,
            negatives_skipped: Vec::new(),
            notes,
        }),
        diagnostics,
    })
}

/// [`catalogue_host_cuts`] 的结果：世界系的刀，外加求值失败的那几行。
pub(crate) struct CatalogueHostCuts {
    pub cutters: Vec<Manifold>,
    pub diagnostics: Vec<String>,
}

/// 目录件的 `NGMR`（→ `NGMS`）。`NGMR` 只在结构目录件（`SFIT` / `JOIN` / `STCA` 这一族）的
/// 模板里；管件 `SCOM`、HVAC / 桥架构件的模板根本没有这个属性，e3d-io 对「模板里没有」是
/// `Err(UnknownAttribute)` 而不是 `Ok(None)`（与逻辑 unset 分开），直接 `get_element` 会把
/// aps250110 的 34 只 FLAN / ELBO / VALV / NOZZ / REDU / TEE 整件打死。这里先问模板认不认识，
/// 不认识 = 没有负体集，与「有属性但 unset」同一口径。
fn component_negative_set(
    component: &DbElement,
) -> Result<Option<DbElement>, e3d_io::db_element::DbElementError> {
    if !component.has_attribute("NGMR")? {
        return Ok(None);
    }
    component.get_element("NGMR")
}

/// 目录件 `NGMR` 名下给**宿主**扣的负体：`member`（FIXING 这类设计件）在目录件那一侧挂的
/// `NGMS`，逐把先过调用方的 `admit`（NAPP 门由调用方按路径定），再过与正体同一张显示筛，
/// 在 `member` 的目录局部系求值后放到世界系（与正体同一个 [`catalogue_member_world`]）。
///
/// 刀与宿主共壁是常态（柱底板那把 NSBO 的四个侧面正好落在柱翼缘外表面与翼缘边上），所以
/// 与设计侧负体同一口径，在刀自己的局部系让 [`crate::solid::MANIFOLD_NUDGE_MM`] 再放置。
///
/// 没有目录件、目录件没有 `NGMR` 都是空集。单把刀求不出按 Core 的逐图元错误边界记进
/// `diagnostics`，其余照扣。
pub(crate) fn catalogue_host_cuts(
    member: &DbElement,
    admit: &dyn Fn(&DbElement) -> bool,
    cache: &mut CatalogueMeshCache,
) -> Result<CatalogueHostCuts, ModelError> {
    let empty = CatalogueHostCuts {
        cutters: Vec::new(),
        diagnostics: Vec::new(),
    };
    let Some(component) = catalogue_component(member)? else {
        return Ok(empty);
    };
    let Some(negative_set) = component_negative_set(&component)? else {
        return Ok(empty);
    };
    let env = crate::catalogue_point::member_env(member, &component);
    let (evaluated, diagnostics) = collect_owned_negatives_where(
        member,
        &component,
        &negative_set,
        &env,
        cache,
        DMat4::IDENTITY,
        admit,
    )?;
    if evaluated.is_empty() {
        return Ok(CatalogueHostCuts {
            cutters: Vec::new(),
            diagnostics,
        });
    }
    let world = catalogue_member_world(member)?;
    let mut cutters = Vec::with_capacity(evaluated.len());
    for cutter in &evaluated {
        let (local, transform) = cutter_solid(cutter)?;
        let local = crate::solid::inflate_about_center(&local, crate::solid::MANIFOLD_NUDGE_MM);
        cutters.push(placed(&local, world * transform));
    }
    Ok(CatalogueHostCuts {
        cutters,
        diagnostics,
    })
}

/// 目录件成员的世界矩阵。
///
/// `FIXING` 是例外：它的 owner 链上 `JLDATU` 与 `PLDATU` 两级**都没有 POS/ORI**，标架是
/// 隐式的（关节位置由 `ZDIS` 沿宿主 WALL 的 spine 定，PLDATU 的 `POSL=OBOW` 是 E3D 的
/// 隐式墙 pline），`world_matrix` 在这两级上乘的是单位阵。墙上开孔的那把刀早就算对了
/// （[`crate::elmodl::fixing_host_frame`]），而**它自己那一件实例**走的是这里——两处
/// 各算各的，于是 1112 的 255 个 FIXING 全挤在 x∈[-2520,-1920] 的一条窄带上，对着
/// E3D TTY 的 RVM 基准逐个偏 13~25 m。现在两处共用同一个标架。
fn catalogue_member_world(member: &DbElement) -> Result<DMat4, ModelError> {
    if member.element_type()? == "FIXING"
        && let Some(wall) = crate::elmodl::fixing_host_wall(member)?
    {
        return Ok(crate::elmodl::fixing_host_frame(&wall, member)?
            * crate::elmodl::fixing_local_matrix(member)?);
    }
    Ok(world_matrix(member)?)
}

fn preserves_catalogue_components(saw_display_primitive: bool) -> bool {
    saw_display_primitive
}

/// Core expands HVAC `DESP` through xrules before catalogue evaluation. The
/// common rectangular straight/branch families reduce to one visible box in a
/// narrow (`insu/obst off`) RVM export, so keep that body on the canonical box
/// instancing path instead of rebuilding the catalogue body/insulation SBOXes.
fn model_hvac_rectangular_basic(
    member: &DbElement,
) -> Result<Option<CatalogueMemberBuild>, ModelError> {
    // 模板里根本没有 `SPRE` 的 noun（`NOZZ` 就是）不是风管构件，与「有这个属性但空着」
    // 同样直接出局。缺属性在这里当错误抛，会让所有从 `CATR` 起跳的设计件在读到风管
    // 捷径这一步就整只失败——错误说的还是一个它本来就不该有的属性。
    let spec = match member.get_element("SPRE") {
        Ok(spec) => spec,
        Err(e3d_io::db_element::DbElementError::UnknownAttribute { .. }) => return Ok(None),
        Err(error) => return Err(error.into()),
    };
    let Some(spec) = spec else {
        return Ok(None);
    };
    let Some(spec_name) = spec.stored_name()? else {
        return Ok(None);
    };
    let Some(desp) = member.get_double_array("DESP")? else {
        return Ok(None);
    };
    let Some((dimensions, centre_offset)) = hvac_rectangular_box(&spec_name, &desp) else {
        return Ok(None);
    };

    let mut built = instantiate_box(dimensions.x, dimensions.y, dimensions.z, true)?;
    let centre = DMat4::from_translation(centre_offset);
    built.solid = placed(&built.solid, centre);
    // 这一族不发实例身份：内核对这只箱子施加了**两次**局部变换（缩放，然后
    // 居中平移），而能记进 `PrimitiveInstance` 的只有一个矩阵。把两者折叠成
    // `centre * scale` 记下来，数学上等价、位模式上不等价——消费方拿它从规范
    // 网格重算世界顶点时，会与内核差几个 ULP（ams7997 实测 STRT 366 件 +
    // BRCO 89 件，最大 1.5e-11 mm；`examples/instance_transform_bits.rs` 里
    // 「折叠」那一列就是这个现象）。宁可少一族实例化，不要一个记错的变换。
    built.instance = None;
    let world = world_matrix(member)?;
    let noun = member.element_type()?;
    Ok(Some(CatalogueMemberBuild {
        generated: Some(GeneratedElement {
            geometry_id: GeometryId::Element {
                refno: member.refno().to_string(),
            },
            refno: member.refno(),
            noun,
            is_invalid_tubi: false,
            name: member.stored_name()?,
            solid: placed(&built.solid, world),
            parts: None,
            part_catalogue_nouns: Vec::new(),
            part_record_repeats: Vec::new(),
            degenerate_parts: Vec::new(),
            world,
            primitive_instance: built.instance,
            analytic_primitive: None,
            negatives_applied: 0,
            negatives_skipped: Vec::new(),
            notes: vec![format!(
                "HVAC rectangular basic instance: spec={spec_name} dimensions={}/{}/{}",
                dimensions.x, dimensions.y, dimensions.z
            )],
        }),
        diagnostics: Vec::new(),
        no_shape: None,
    }))
}

fn hvac_rectangular_box(spec_name: &str, desp: &[f64]) -> Option<(DVec3, DVec3)> {
    let (dimensions, centre_offset) = if spec_name.ends_with("/RSTRA") {
        let dimensions = DVec3::new(*desp.get(1)?, *desp.get(2)?, *desp.get(5)?);
        (dimensions, DVec3::Z * (dimensions.z * 0.5))
    } else if spec_name.ends_with("/RSBRCO") {
        let dimensions = DVec3::new(*desp.get(6)?, *desp.get(40)?, *desp.get(41)?);
        // BRCO's catalogue branch leaves along local -X; the member's world
        // orientation maps that to the RVM-observed positive world branch axis.
        (dimensions, -DVec3::X * (dimensions.x * 0.5))
    } else {
        return None;
    };
    (dimensions.is_finite() && dimensions.min_element() > 0.0)
        .then_some((dimensions, centre_offset))
}

/// Core3D `pplib/QLPYRA` (`0x103C07B4`) evaluates an LPYR as one reusable
/// pyramid/frustum.  `PAAX` is the height axis, `PBAX` the local X axis, and
/// `PCAX` only fixes the sign of the local Y offset after Core's ORTHOG step.
fn evaluate_lpyr(
    member: &DbElement,
    component: &DbElement,
    pyramid: &DbElement,
    env: &DbElementParamEnv,
) -> Result<Option<EvaluatedPyramid>, ModelError> {
    evaluate_lpyr_as(
        member,
        component,
        pyramid,
        env,
        "catalogue LPYR",
        "catalogue LPYR axis",
    )
}

/// [`evaluate_lpyr`] 的本体，正体 LPYR 与刀 NLPY 共用：同一组属性（PBTP/PCTP/PBBT/PCBT、
/// PTDI/PBDI、PBOF/PCOF）、同一套三根轴（PAAX 高、PBAX 局部 X、PCAX 定 Y 偏移的号），
/// 只有报错里的名字不同。
fn evaluate_lpyr_as(
    member: &DbElement,
    component: &DbElement,
    pyramid: &DbElement,
    env: &DbElementParamEnv,
    what: &'static str,
    axis_what: &'static str,
) -> Result<Option<EvaluatedPyramid>, ModelError> {
    let xtop = required_number(pyramid, "PBTP", env, what)?.abs();
    let ytop = required_number(pyramid, "PCTP", env, what)?.abs();
    let xbot = required_number(pyramid, "PBBT", env, what)?.abs();
    let ybot = required_number(pyramid, "PCBT", env, what)?.abs();
    let (top_distance, bottom_distance, xoff, yoff) = lpyr_axial_and_offsets(
        required_number(pyramid, "PTDI", env, what)?,
        required_number(pyramid, "PBDI", env, what)?,
        required_number(pyramid, "PBOF", env, what)?,
        required_number(pyramid, "PCOF", env, what)?,
    );

    let pa = catalogue_axis_or_fail(member, component, pyramid, "PAAX", axis_what)?;
    let pb = catalogue_axis_or_fail(member, component, pyramid, "PBAX", axis_what)?;
    let pc = catalogue_axis_or_fail(member, component, pyramid, "PCAX", axis_what)?;

    lpyr_geometry(
        [xbot, ybot, xtop, ytop],
        [top_distance, bottom_distance, xoff, yoff],
        pa,
        pb,
        pc,
    )
}

fn lpyr_geometry(
    [mut xbot, mut ybot, mut xtop, mut ytop]: [f64; 4],
    [top_distance, bottom_distance, xoff, mut yoff]: [f64; 4],
    pa: CatalogueAxis,
    mut pb: CatalogueAxis,
    pc: CatalogueAxis,
) -> Result<Option<EvaluatedPyramid>, ModelError> {
    let mut height = top_distance - bottom_distance;
    if height <= 0.0 {
        height = -height;
        // QLPYRA (Core3D 0x103C07B4) calls HCHECK on PBAX, not PAAX.
        // Axial coordinates retain their sign along the original PAAX.
        pb.direction = -pb.direction;
        std::mem::swap(&mut xbot, &mut xtop);
        std::mem::swap(&mut ybot, &mut ytop);
    }
    if height < HEIGHT_TOL_MM {
        return Ok(None);
    }
    let (x, y, z) = orthog(pa.direction, pb.direction);
    if pc.direction.dot(y) < 0.0 {
        yoff = -yoff;
    }
    let rotation = DMat3::from_cols(x, y, z);
    let centre = DVec3::new(
        xoff * 0.5,
        yoff * 0.5,
        (top_distance + bottom_distance) * 0.5,
    );
    let parameters = [xbot, ybot, xtop, ytop, height, xoff, yoff];
    let parameter_bits = parameters.map(|value| value.to_bits());
    Ok(Some(EvaluatedPyramid {
        mesh_key: CatalogueMeshKey::Pyramid {
            tessellation_version: PYRAMID_TESSELLATION_VERSION,
            parameter_bits,
        },
        parameters,
        transform: DMat4::from_translation(pa.origin + rotation * centre)
            * DMat4::from_mat3(rotation),
    }))
}

/// Core `mathlib/ORTHOG`（`core.dll` `0x526B878`）：把 `(z, x)` 两根轴正交化成一套
/// 右手标架，返回 `(x, y, z)`。**每一步退化都有替补，它不会失败**——错号是置了，
/// 但 `QLPYRA` 读都不读（`Core3D.dll` `0x103C07B4`，`ORTHOG` 之后直接往下走）。
///
/// 逐条照抄：`z` 归一不了换 `(1,0,0)`，`x` 归一不了换 `(0,1,0)`；`y = z × x` 退化时
/// 依次退到 `z × (0,1,0)`、`z × (1,0,0)`；最后 `x = y × z` 把 x 拉回垂直。
///
/// 这条闸此前是「三根轴有一根退化就整只图元报废」。SLYK 的
/// `/Z9B43H-16C-FBP-GM` 手轮座 `LPYR 16596/10594` 正撞上它：`PAAX` 写 `P11` 取反、
/// `PBAX` 写 `P1`，两根解出来都是 +X，`z × x` 是零。E3D 走 ORTHOG 退到 `z × (0,1,0)`，
/// 照常写出 `Pyramid[315 × 198.125, h 263.1]`；我们整只丢掉——80 根严尺子的基准里
/// 300 只阀门各少这么一块（`source_lpyr 7 / evaluated_lpyr 0`，见 21491/10802 的
/// `report.json`）。
fn orthog(z: DVec3, x: DVec3) -> (DVec3, DVec3, DVec3) {
    let normalised = |v: DVec3| (v.length_squared() > AXIS_TOL).then(|| v.normalize());
    let z = normalised(z).unwrap_or(DVec3::X);
    let x = normalised(x).unwrap_or(DVec3::Y);
    let y = normalised(z.cross(x))
        .or_else(|| normalised(z.cross(DVec3::Y)))
        .or_else(|| normalised(z.cross(DVec3::X)))
        // z 是单位向量，`z × Y` 与 `z × X` 不会同时退化，这一支到不了。
        .unwrap_or(DVec3::Y);
    (y.cross(z), y, z)
}

fn lpyr_axial_and_offsets(
    top_distance: f64,
    bottom_distance: f64,
    xoff: f64,
    yoff: f64,
) -> (f64, f64, f64, f64) {
    (top_distance, bottom_distance, xoff, yoff)
}

fn evaluate_catalogue_snout(
    member: &DbElement,
    component: &DbElement,
    snout: &DbElement,
    env: &DbElementParamEnv,
    noun: &str,
) -> Result<Option<EvaluatedSnout>, ModelError> {
    let (dbot, dtop, height, xoff, centre, rotation) = if noun == "SCON" {
        let diameter = required_number(snout, "PDIA", env, "catalogue SCON")?.abs();
        let distance = required_number(snout, "PDIS", env, "catalogue SCON")?;
        let axis = catalogue_axis_or_fail(member, component, snout, "PAXI", "catalogue SCON axis")?;
        let (dbot, dtop, height, centre, rotation) = scone_frame(diameter, distance, axis)?;
        (dbot, dtop, height, 0.0, centre, rotation)
    } else {
        let dbot = required_number(snout, "PBDM", env, "catalogue LSNO")?.abs();
        let dtop = required_number(snout, "PTDM", env, "catalogue LSNO")?.abs();
        let bottom = required_number(snout, "PBDI", env, "catalogue LSNO")?;
        let top = required_number(snout, "PTDI", env, "catalogue LSNO")?;
        let offset = required_number(snout, "POFF", env, "catalogue LSNO")?;
        let pa = catalogue_axis_or_fail(member, component, snout, "PAAX", "catalogue LSNO axis")?;
        let pb = catalogue_axis_or_fail(
            member,
            component,
            snout,
            "PBAX",
            "catalogue LSNO offset axis",
        )?;
        let (dbot, dtop, height, offset, ends_swapped) =
            normalize_lsnout_span(dbot, dtop, bottom, top, offset);
        // 底顶对调时 E3D 的 DRSNO 把局部 X 也调成 −PBAX，`XOFF` 照 `POFF` 的号写；此前这里
        // 保留 PBAX、把 `XOFF` 取反——同一个实体，矩阵的 X/Y 列和 XOFF 的符号却与 E3D 相反
        // （ams7999 `/1WCC318VN` 的两只 `PBDI 57.5 / PTDI 0 / POFF 7.4` LSNO，E3D 写
        // `X = −PBAX, XOFF = +7.4`）。见 [`normalize_lsnout_span`]。
        let pb = if ends_swapped {
            crate::catalogue_point::CatalogueAxis {
                direction: -pb.direction,
                ..pb
            }
        } else {
            pb
        };
        let (centre, rotation) = lsnout_frame(pa, pb, bottom, top, offset)?;
        (dbot, dtop, height, offset, centre, rotation)
    };
    if height < HEIGHT_TOL_MM || dbot.max(dtop) < HEIGHT_TOL_MM {
        return Ok(None);
    }
    let parameters = [dbot, dtop, height, xoff, 0.0];
    Ok(Some(EvaluatedSnout {
        mesh_key: CatalogueMeshKey::Snout {
            tessellation_version: SNOUT_TESSELLATION_VERSION,
            parameter_bits: parameters.map(|value| value.to_bits()),
        },
        parameters,
        transform: DMat4::from_translation(centre) * DMat4::from_mat3(rotation),
    }))
}

/// 目录 SCON 的形体与放置：`(DBOT, DTOP, 高, 中心, 标架)`。
///
/// Core3D `pplib/QSCONE`（`0x103BFE40`）读带符号 `PDIS`、`|PDIA|`、`PAXI`；`PDIS ≤ 0` 走
/// `HCHECK`（高度取反、轴反向）；中心 = 轴原点 + 轴向 × PDIS/2。CRCATI（`0x10345ACC`）的
/// 818038/829400 臂随后 `ORIMAT(轴)`，再 `gm_CreateSnout(0, PDIA/2, PDIS)`——**底半径 0**。
///
/// 也就是：**锥顶坐在 PAXI 的原点（局部 −Z 端），底圆在沿轴 PDIS 处（局部 +Z 端）**。
/// PARAGON 手册 §7.4.3 只说 `PDIS` 是「vertex 高出 base 的高度」，没说哪头在原点；E3D 自己的
/// RVM 导出定了这件事：`/OILSTD-COUPLING-QUICKFIT-SWF/GMSE_001` 的 SCON（`PAXI P2 PDIS 42
/// PDIA 66`）写成 `Snout[rb=0, rt=33, h=42]`，锥顶就压在 P2 上、底圆朝构件里缩 42 mm。
///
/// 2026-09-14 之前这里反着放（底圆在原点、锥顶在 PDIS）：AABB 一模一样，整只锥却调了个头，
/// 80 根严尺子里 7 只 PCOM 的圆锥都是这一条——对拍量 AABB 看不出来，是写出器把它认成
/// `SphericalDish` 才露出来。
fn scone_frame(
    diameter: f64,
    distance: f64,
    mut axis: crate::catalogue_point::CatalogueAxis,
) -> Result<(f64, f64, f64, DVec3, DMat3), ModelError> {
    if distance < 0.0 {
        axis.direction = -axis.direction;
    }
    let height = distance.abs();
    let rotation = orimat(axis.direction).ok_or_else(|| {
        ModelError::Solid(SolidError::Degenerate {
            what: "catalogue SCON axis",
            detail: format!("PAXI={:?}", axis.direction),
        })
    })?;
    Ok((
        0.0,
        diameter,
        height,
        axis.origin + axis.direction.normalize() * (height * 0.5),
        rotation,
    ))
}

/// `(底径, 顶径, 高, 偏移, 底顶是否对调)`，把 PBDI/PTDI 的跨度整理成沿 PAAX 从低到高。
///
/// 跨度为负（PTDI < PBDI）时两端标签互换，PAAX 不动。偏移**不取反**：对调后的偏移方向
/// 由调用方把局部 X 换成 −PBAX 来表达——E3D 的 DRSNO 就是这么写的（`X = −PBAX`、
/// `XOFF = POFF`），这样记录的矩阵与 XOFF 的符号都与 E3D 相同；此前这里取反偏移、X 保持
/// PBAX，实体一样，矩阵不一样。
fn normalize_lsnout_span(
    dbot: f64,
    dtop: f64,
    bottom: f64,
    top: f64,
    offset: f64,
) -> (f64, f64, f64, f64, bool) {
    let height = top - bottom;
    if height < 0.0 {
        (dtop, dbot, -height, offset, true)
    } else {
        (dbot, dtop, height, offset, false)
    }
}

fn lsnout_frame(
    pa: crate::catalogue_point::CatalogueAxis,
    pb: crate::catalogue_point::CatalogueAxis,
    bottom: f64,
    top: f64,
    offset: f64,
) -> Result<(DVec3, DMat3), ModelError> {
    // PBDI/PTDI are coordinates along the source PAAX. A negative span swaps
    // mesh end labels, but must not reverse or relocate the primitive axis.
    let z = pa.direction.normalize_or_zero();
    let x = (pb.direction - z * pb.direction.dot(z)).normalize_or_zero();
    let y = z.cross(x).normalize_or_zero();
    if z.length_squared() <= AXIS_TOL
        || x.length_squared() <= AXIS_TOL
        || y.length_squared() <= AXIS_TOL
    {
        return Err(ModelError::Solid(SolidError::Degenerate {
            what: "catalogue LSNO axes",
            detail: format!("PAAX={:?}, PBAX={:?}", pa.direction, pb.direction),
        }));
    }
    Ok((
        pa.origin + z * ((bottom + top) * 0.5) + x * (offset * 0.5),
        DMat3::from_cols(x, y, z),
    ))
}

/// 刀 NLPY 与正体 LPYR 走**同一条**几何规则（[`evaluate_lpyr_as`]）：PAAX 定高、PBAX 定局部 X、
/// PCAX 定 Y 偏移的号，PBOF/PCOF 进放置。
///
/// 此前这里只读 PAAX、拿 `orimat` 随手配一对横向轴、横向偏移也不进放置——方的、居中的刀
/// 看不出来，长宽不等或带偏移的刀就转到别的方向去了。CADC 风管的裤形三通 `/RBREECH`
/// （ams7997 `24381/100890`）正撞上：`NLPYRAMID 2 of /RBREECH-BODY1` 与 `/RBREECH-NGAP`
/// 都是 `PBBT 0 / PBTP DESP[13] = 265`（沿 PBAX = P6 = X）× `PCBT ≈ 788 / PCTP 810`（沿 Y）
/// 的楔形刀，被转成 x 向 810 宽、y 向 0 → 265 的楔——V 口切到了腿上、V 里的钣金填块 `GAP`
/// 反而没被 `NGAP` 掏空（seg15 账 §26.12；真库门 `tests/hvac_rbreech_gap_real.rs`）。
fn evaluate_nlpy(
    member: &DbElement,
    component: &DbElement,
    pyramid: &DbElement,
    env: &DbElementParamEnv,
) -> Result<Option<EvaluatedPyramid>, ModelError> {
    if pyramid.get_bool("TUFL")? == Some(false) {
        return Ok(None);
    }
    evaluate_lpyr_as(
        member,
        component,
        pyramid,
        env,
        "catalogue NLPY",
        "catalogue NLPY axis",
    )
}

/// SEXT / SREV 共用的轮廓采集（Core 侧同为环组装配器 `sub_10715170`）：
/// `SLOO → SVER` 的 PX/PY/PRAD，圆角折线化后成环组。两臂各写一份就是两份
/// 轮廓规则，键与几何都会静默分叉——纪律同 `route.rs` 只许有一份串接规则。
///
/// # 零面积剖面回 `Ok(None)`，不是错
///
/// 第一环（外边界，`rings[0]`，见 [`crate::solid::extrude_rings`]）求值后塌成线或点
/// ——`flatten_ring` 焊掉重点后不足 3 点——是目录把这个图元**参数化关掉**了：ams8000 托盘
/// 的 `*-GS-COVER-*` 盖板 PX/PY 乘 `ATTRIB RPRO COVR`（无盖时 0）、零角 BEND 的走线段
/// PX ∝ `TAN(ATTRIB ANGL / 2)`，全库 124 条环 PRAD 全 0，圆角从未参与
/// （`evidence/20260902-bran-audit-baseline-4c4ec21/collapsed-profiles-ams8000.txt`）。
/// Core 对零面积剖面不出体，与零高 SEXT / 零径 LCYL 同一档「零高或退化」。孔环塌掉只是
/// 一个零面积的孔，丢掉照建（`gm_AddCurve` 对它也无事可做）。**一个 SLOO 都没有**才是数据错，
/// 仍报 [`ModelError::OuterRingCollapsed`]。
fn profile_rings(
    primitive: &DbElement,
    env: &DbElementParamEnv,
    noun: &'static str,
) -> Result<Option<Vec<Vec<[f64; 2]>>>, ModelError> {
    let mut rings = Vec::new();
    let mut loops = 0usize;
    for loop_element in primitive.members()? {
        if !loop_element.element_type()?.eq_ignore_ascii_case("SLOO") {
            continue;
        }
        loops += 1;
        let mut vertices = Vec::new();
        for vertex in loop_element.members()? {
            if !vertex.element_type()?.eq_ignore_ascii_case("SVER") {
                continue;
            }
            vertices.push(FilletVertex {
                x: required_number(&vertex, "PX", env, "catalogue profile vertex")?,
                y: required_number(&vertex, "PY", env, "catalogue profile vertex")?,
                // PRAD 语义同设计侧 FRAD：负 = 以顶点为心的凹圆角（discretise 模块
                // 文档第 4 条）。旧 `.abs()` 会把负值翻成凸圆角——摘除；全库 124 条
                // 环 PRAD 全 0（上面那份 evidence），摘除不改变现有输出。
                frad: optional_number(&vertex, "PRAD", env).unwrap_or(0.0),
            });
        }
        let Some((ring, _notes)) = flatten_ring(&vertices) else {
            if rings.is_empty() {
                // 外环零面积：整个图元不出体。
                return Ok(None);
            }
            // 孔环零面积：没有孔可扣，丢掉。
            continue;
        };
        rings.push(ring);
    }
    if loops == 0 {
        return Err(ModelError::OuterRingCollapsed {
            refno: primitive.refno().to_string(),
            noun: noun.into(),
        });
    }
    let Some((largest_index, _)) = rings
        .iter()
        .enumerate()
        .max_by(|(_, left), (_, right)| ring_area(left).total_cmp(&ring_area(right)))
    else {
        return Ok(None);
    };
    let largest_contains_all = rings
        .iter()
        .enumerate()
        .filter(|(index, _)| *index != largest_index)
        .all(|(_, ring)| {
            ring.iter()
                .all(|point| point_in_ring(*point, &rings[largest_index]))
        });
    let outer_index = if largest_contains_all { largest_index } else { 0 };
    let outer = rings[outer_index].clone();
    let mut selected = vec![outer.clone()];
    for (index, ring) in rings.into_iter().enumerate() {
        if index == outer_index || ring_area(&ring) >= ring_area(&outer) {
            continue;
        }
        // A disjoint loop cannot be a hole of this SEXT. Core keeps the first
        // construction loop and ignores later auxiliary loops rather than
        // turning them into a second solid or extending the member's AABB.
        if !ring.iter().all(|point| point_in_ring(*point, &outer)) {
            // Core's profile builder keeps the first disjoint construction
            // loop as the visible profile and ignores later auxiliary loops.
            continue;
        }
        selected.push(ring);
    }
    Ok(Some(selected))
}

fn ring_area(ring: &[[f64; 2]]) -> f64 {
    ring.iter()
        .zip(ring.iter().cycle().skip(1))
        .take(ring.len())
        .map(|([x1, y1], [x2, y2])| x1 * y2 - x2 * y1)
        .sum::<f64>()
        .abs()
        * 0.5
}

fn point_in_ring(point: [f64; 2], ring: &[[f64; 2]]) -> bool {
    let mut inside = false;
    for (a, b) in ring.iter().zip(ring.iter().cycle().skip(1)).take(ring.len()) {
        let [x, y] = point;
        let [ax, ay] = *a;
        let [bx, by] = *b;
        if (ay > y) != (by > y) {
            let crossing_x = (bx - ax) * (y - ay) / (by - ay) + ax;
            if x < crossing_x {
                inside = !inside;
            }
        }
    }
    inside
}

/// PAAX/PBAX → 右手标架（Core `ORTHOG`：保第一根轴，第二根由补全的第三根反求）。
/// SEXT 与 SREV 同一份：轮廓平面都是 (PAAX, PBAX 正交化) 张成的 XY。
fn profile_frame(
    member: &DbElement,
    component: &DbElement,
    primitive: &DbElement,
    what: &'static str,
) -> Result<(crate::catalogue_point::CatalogueAxis, DMat3), ModelError> {
    let pa = catalogue_axis_or_fail(member, component, primitive, "PAAX", what)?;
    let pb = catalogue_axis_or_fail(member, component, primitive, "PBAX", what)?;
    let x = pa.direction.normalize_or_zero();
    let z = x.cross(pb.direction).normalize_or_zero();
    if x.length_squared() <= AXIS_TOL || z.length_squared() <= AXIS_TOL {
        return Err(ModelError::Solid(SolidError::Degenerate {
            what,
            detail: format!("PAAX={:?}, PBAX={:?}", pa.direction, pb.direction),
        }));
    }
    let y = z.cross(x).normalize();
    Ok((pa, DMat3::from_cols(x, y, z)))
}

fn evaluate_sext(
    member: &DbElement,
    component: &DbElement,
    sext: &DbElement,
    env: &DbElementParamEnv,
) -> Result<Option<EvaluatedSext>, ModelError> {
    let height = required_number(sext, "PHEI", env, "catalogue SEXT")?.abs();
    if height < HEIGHT_TOL_MM {
        return Ok(None);
    }

    let Some(rings) = profile_rings(sext, env, "SEXT")? else {
        return Ok(None);
    };
    let (pa, rotation) = profile_frame(member, component, sext, "catalogue SEXT axes")?;
    let raw_position = DVec3::new(
        required_number(sext, "PX", env, "catalogue SEXT position")?,
        required_number(sext, "PY", env, "catalogue SEXT position")?,
        required_number(sext, "PZ", env, "catalogue SEXT position")?,
    );
    // QSSEXT（0x103B7FCC）：位置 = 标架 · (PX,PY,PZ)（TRAVEC 旋进标架）。
    let transform = DMat4::from_translation(pa.origin + rotation * raw_position)
        * DMat4::from_mat3(rotation);
    let mesh_key = CatalogueMeshKey::Sext(sext_mesh_key(&rings, height)?);
    Ok(Some(EvaluatedSext {
        mesh_key,
        rings,
        height,
        transform,
        cutters: Vec::new(),
        diagnostics: Vec::new(),
    }))
}

/// 正体 SEXT：轮廓 + 放置（[`evaluate_sext`]）再加名下的成员级刀集。
///
/// ams5052 `/ZSPE660NN2-GS` 的 SEXT 名下挂着 1 只 NSEX（内腔，四周内缩 2 mm）
/// 与 4 只 NSCY（底板安装孔）；E3D 的 TTY RVM 把它画成 2 mm 薄壳，此前这里只认
/// SLOO、其余子元素一律跳过，出来的是实心块（体积 3.13e6 对 E3D 的 ≈1.5e6 mm³），
/// 而报告上 `negatives_skipped=0`、一条 note 都没有。两个工程目录的 30 个 CATA 库里
/// 挂在 SEXT 名下的负体有 3.8 万件（NSEX / NSBO / NLPY / NSRE / NSCY…），是 SBOX 名下的五倍。
///
/// 刀的收集与 SREV 同一份（[`collect_owned_negatives`]）：SEXT 的放置带标架旋转，
/// 回到局部系走逆矩阵，不能像 SBOX 那样只减 position。
fn evaluate_sext_body(
    member: &DbElement,
    component: &DbElement,
    sext: &DbElement,
    env: &DbElementParamEnv,
    cache: &mut CatalogueMeshCache,
) -> Result<Option<EvaluatedSext>, ModelError> {
    let Some(mut evaluated) = evaluate_sext(member, component, sext, env)? else {
        return Ok(None);
    };
    let rebase = evaluated.transform.inverse();
    let (cutters, diagnostics) =
        collect_owned_negatives(member, component, sext, env, cache, rebase)?;
    if !cutters.is_empty() {
        evaluated.mesh_key = CatalogueMeshKey::CutSext {
            base: sext_mesh_key(&evaluated.rings, evaluated.height)?,
            cutters: catalogue_cut_keys(&cutters)?,
        };
    }
    evaluated.cutters = cutters;
    evaluated.diagnostics = diagnostics;
    Ok(Some(evaluated))
}

/// QSSREV（`0x103B81CC`）的扫角归一：`|PANG| < 0.001` 缺省成整圈 360，
/// `|PANG| > 360` 带符号钳到 ±360。与设计侧 REV「≈360 一律当整圈」不同——
/// 目录臂对 -359.9 这类近整圈**保留部分扫角**，别把两条归一互相抄。
fn qssrev_angle(raw: f64) -> f64 {
    if raw.abs() < 0.001 {
        return 360.0;
    }
    raw.clamp(-360.0, 360.0)
}

/// 目录回转体（CRCATI 968617/643505 → `pplib/QSSREV` + `gm_CreateRevolution`）。
///
/// 轮廓与 SEXT 同一个组装器（[`profile_rings`]），标架同一份 ORTHOG
/// （[`profile_frame`]），回转轴 = 标架局部 X（`gm_CreateRevolution` 的
/// rot_dir=X / rot_pt=0，与设计侧 [`crate::solid::revolve_rings`] 同一约定）。
///
/// **位置与 SEXT 不同**：QSSREV 把 PX/PY/PZ **不旋转**地加在 PAAX 轴原点上
/// （`*a3 += PX; …`），而 QSSEXT 先 TRAVEC 旋进标架——两段反编译都在案，
/// 别顺手「统一」成同一种。
///
/// 负角语义：Core 走起始相位 +180（`v90 = 180.0`）扫 |PANG|；本仓沿用
/// `revolve_rings` 文档记载的镜像等价实现，partial 负回转的相位约定
/// 等 RVM 语料坐实（`solid.rs` 的既有挂账，两条路同一笔账）。
///
/// 名下负体与 SBOX 同款成员级刀集：ams5052 实测 SREV 名下挂着
/// NSRT/NLSN/NSRE/NLPY/NSEX/NSBO/NLCY/NSCY，全部复用各自正体的求值器。
fn evaluate_srev(
    member: &DbElement,
    component: &DbElement,
    srev: &DbElement,
    env: &DbElementParamEnv,
    cache: &mut CatalogueMeshCache,
) -> Result<Option<EvaluatedSrev>, ModelError> {
    let angle_deg = qssrev_angle(required_number(srev, "PANG", env, "catalogue SREV")?);
    let Some(rings) = profile_rings(srev, env, "SREV")? else {
        return Ok(None);
    };
    let (pa, rotation) = profile_frame(member, component, srev, "catalogue SREV axes")?;
    let raw_position = DVec3::new(
        required_number(srev, "PX", env, "catalogue SREV position")?,
        required_number(srev, "PY", env, "catalogue SREV position")?,
        required_number(srev, "PZ", env, "catalogue SREV position")?,
    );
    // QSSREV：位置 = 轴原点 + 原始 (PX,PY,PZ)，不旋转（见函数文档）。
    let transform = DMat4::from_translation(pa.origin + raw_position) * DMat4::from_mat3(rotation);
    // 刀的求值都在部件局部系；SREV 的放置带旋转，回到 SREV 局部系要走逆矩阵
    // （SBOX 是纯平移，减 position 就够，这里不够）。
    let rebase = transform.inverse();
    let (cutters, diagnostics) =
        collect_owned_negatives(member, component, srev, env, cache, rebase)?;

    let cutter_keys = catalogue_cut_keys(&cutters)?;
    let (ring_lengths, coordinate_bits) = profile_key_bits(&rings, "catalogue SREV profile")?;
    let mesh_key = CatalogueMeshKey::Srev(SrevMeshKey {
        tessellation_version: SREV_TESSELLATION_VERSION,
        angle_bits: finite_bits(angle_deg, "catalogue SREV angle")?,
        ring_lengths,
        coordinate_bits,
        cutters: cutter_keys,
    });
    Ok(Some(EvaluatedSrev {
        mesh_key,
        rings,
        angle_deg,
        cutters,
        transform,
        diagnostics,
    }))
}

/// 轮廓类正体（SREV / SEXT）名下的成员级刀集：逐个子元素按 noun 复用各自正体的
/// 求值器，刀在部件局部系求出，再乘 `rebase` 回到属主的局部系。SLOO 是属主自己的
/// 剖面成员，不是刀。认不出的 noun 落一行 `is not implemented`——**不许静默跳过**：
/// 这份收集器此前只有 SREV 一份、SBOX 另抄一份，SEXT 什么都没有，于是 SEXT 名下
/// 的负体连一行 note 都不留就没了（见 [`evaluate_sext_body`]）。
///
/// 与 SBOX 那份的分岔只剩两处，都有理由：SBOX 是纯平移放置，求值器直接减 position；
/// NLSN / NSCO / NSRE 至今没有语料在 SBOX 名下出现过（守卫
/// `both_negative_collectors_recognise_the_same_nouns` 钉着这张单子）。
fn collect_owned_negatives(
    member: &DbElement,
    component: &DbElement,
    owner: &DbElement,
    env: &DbElementParamEnv,
    cache: &mut CatalogueMeshCache,
    rebase: DMat4,
) -> Result<(Vec<EvaluatedCatalogueCut>, Vec<String>), ModelError> {
    collect_owned_negatives_where(member, component, owner, env, cache, rebase, &|_| true)
}

/// [`collect_owned_negatives`]，但每把刀先问 `admit`：答否的不求值、不记账。
fn collect_owned_negatives_where(
    member: &DbElement,
    component: &DbElement,
    owner: &DbElement,
    env: &DbElementParamEnv,
    cache: &mut CatalogueMeshCache,
    rebase: DMat4,
    admit: &dyn Fn(&DbElement) -> bool,
) -> Result<(Vec<EvaluatedCatalogueCut>, Vec<String>), ModelError> {
    let mut cutters = Vec::new();
    let mut diagnostics = Vec::new();
    // 每把刀也过一遍与正体同一张显示筛（`CSG_TreeBuilderCat::addNegatives` 对每个负体调
    // `isWanted`，LEVE / TUFL 同一套）。此前这里不筛：ams7997 的 CADC 风管目录给每只本体都挂一把
    // 内缩 5 mm 的刀（`/RSTRA-NBODY` `PXLE = DESP[2] − 5`、`/CSBRCO-NBREXT` `PDIA = DESP[41] − 5`、
    // `/RSTHRE-NBODY` 的 loop 逐点 −5……），**全是 `LEVE 9 10`**；E3D 在 `LEVEL HVAC 6` 下不扣它们
    // ——TTY RVM（`e3d-hvac-obst-135124/hvac-lvl6.rvm`）里 STRT 是实心 `Cylinder` / `Box`、THRE
    // 是 10 面实心 FacetGroup——而引擎把 2181 只风管构件全扣成了 2.5 mm 薄壳（STRT 1700×930 的
    // 网格体积 1.80e7 对实心 1.87e9 mm³），BRCO `/RSBRCO` 2.5×2.5×117 的 LPYR 本体更是被自己那把
    // `2.5 − 5` 的刀整只吃掉（=24381/57376 少一根 E3D 画了的杆，AABB 差 62.7 mm）。
    // 被筛掉的刀不是失败、不落 note，只进 `display_filtered_negatives` 的账。
    let member_level = repre_level_of(member)?;
    for child in owner.members()? {
        let noun = child.element_type()?;
        if noun.eq_ignore_ascii_case("SLOO") || !admit(&child) {
            continue;
        }
        if display_filtered(&child, member_level)?.is_some() {
            cache.stats.display_filtered_negatives += 1;
            continue;
        }
        if noun.eq_ignore_ascii_case("NSCY") {
            cache.stats.source_nscy += 1;
            match evaluate_nscy(member, component, &child, env, DVec3::ZERO) {
                Ok(Some(cutter)) => {
                    cache.stats.evaluated_nscy += 1;
                    cutters.push(EvaluatedCatalogueCut::Cylinder(EvaluatedNscy {
                        transform: rebase * cutter.transform,
                        ..cutter
                    }));
                }
                Ok(None) => {}
                Err(error) => diagnostics.push(format!(
                    "catalogue negative {} skipped by Core-style local failure: {error}",
                    child.refno()
                )),
            }
        } else if noun.eq_ignore_ascii_case("NLCY") {
            cache.stats.source_nlcy += 1;
            match evaluate_nlcy(member, component, &child, env, DVec3::ZERO) {
                Ok(Some(cutter)) => {
                    cache.stats.evaluated_nlcy += 1;
                    cutters.push(EvaluatedCatalogueCut::Cylinder(EvaluatedNscy {
                        transform: rebase * cutter.transform,
                        ..cutter
                    }));
                }
                Ok(None) => {}
                Err(error) => diagnostics.push(format!(
                    "catalogue negative {} skipped by Core-style local failure: {error}",
                    child.refno()
                )),
            }
        } else if noun.eq_ignore_ascii_case("NSBO") {
            cache.stats.source_nsbo += 1;
            match evaluate_nsbo(&child, env, DVec3::ZERO) {
                Ok(Some(EvaluatedCatalogueCut::Box {
                    dimensions,
                    transform: cutter_transform,
                })) => {
                    cache.stats.evaluated_nsbo += 1;
                    cutters.push(EvaluatedCatalogueCut::Box {
                        dimensions,
                        transform: rebase * cutter_transform,
                    });
                }
                Ok(Some(_)) => unreachable!("evaluate_nsbo returns a box cutter"),
                Ok(None) => {}
                Err(error) => diagnostics.push(format!(
                    "catalogue negative {} skipped by Core-style local failure: {error}",
                    child.refno()
                )),
            }
        } else if noun.eq_ignore_ascii_case("NLPY") {
            match evaluate_nlpy(member, component, &child, env) {
                Ok(Some(cutter)) => cutters.push(EvaluatedCatalogueCut::Pyramid {
                    parameters: cutter.parameters,
                    transform: rebase * cutter.transform,
                }),
                Ok(None) => {}
                Err(error) => diagnostics.push(format!(
                    "catalogue negative {} skipped by Core-style local failure: {error}",
                    child.refno()
                )),
            }
        } else if noun.eq_ignore_ascii_case("NSEX") {
            cache.stats.source_nsex += 1;
            match evaluate_sext(member, component, &child, env) {
                Ok(Some(cutter)) => {
                    cache.stats.evaluated_nsex += 1;
                    cutters.push(EvaluatedCatalogueCut::Extrusion {
                        rings: cutter.rings,
                        height: cutter.height,
                        transform: rebase * cutter.transform,
                    });
                }
                Ok(None) => {}
                Err(error) => diagnostics.push(format!(
                    "catalogue negative {} skipped by Core-style local failure: {error}",
                    child.refno()
                )),
            }
        } else if noun.eq_ignore_ascii_case("NLSN") || noun.eq_ignore_ascii_case("NSCO") {
            let twin = if noun.eq_ignore_ascii_case("NLSN") {
                "LSNO"
            } else {
                "SCON"
            };
            match evaluate_catalogue_snout(member, component, &child, env, twin) {
                Ok(Some(cutter)) => {
                    let mut parameters = cutter.parameters;
                    // §2.3：+0.01 轴向过切只属于圆柱/锥族的负臂；锥台居中建体，
                    // 加在高度上中心不动（与 NSCY 同款）。
                    parameters[2] += NSCY_AXIAL_OVERCUT_MM;
                    cutters.push(EvaluatedCatalogueCut::Snout {
                        parameters,
                        transform: rebase * cutter.transform,
                    });
                }
                Ok(None) => {}
                Err(error) => diagnostics.push(format!(
                    "catalogue negative {} skipped by Core-style local failure: {error}",
                    child.refno()
                )),
            }
        } else if noun.eq_ignore_ascii_case("NSCT") || noun.eq_ignore_ascii_case("NSRT") {
            let twin = if noun.eq_ignore_ascii_case("NSCT") {
                "SCTO"
            } else {
                "SRTO"
            };
            match evaluate_catalogue_torus(member, component, &child, env, twin) {
                Ok(Some(cutter)) => cutters.push(EvaluatedCatalogueCut::Torus {
                    kind: cutter.kind,
                    inner: cutter.inner,
                    outer: cutter.outer,
                    angle_deg: cutter.angle_deg,
                    transform: rebase * cutter.transform,
                }),
                Ok(None) => {}
                Err(error) => diagnostics.push(format!(
                    "catalogue negative {} skipped by Core-style local failure: {error}",
                    child.refno()
                )),
            }
        } else if noun.eq_ignore_ascii_case("NSRE") {
            // 嵌套负回转只取轮廓 + 扫角 + 放置（与 NSEX 刀不再下钻负体同款）。
            let nested = (|| -> Result<Option<EvaluatedCatalogueCut>, ModelError> {
                let angle_deg =
                    qssrev_angle(required_number(&child, "PANG", env, "catalogue NSRE")?);
                // 零面积剖面的刀没有东西可扣，跳过（同 NSEX 的 `Ok(None)`）。
                let Some(rings) = profile_rings(&child, env, "NSRE")? else {
                    return Ok(None);
                };
                let (child_pa, child_rotation) =
                    profile_frame(member, component, &child, "catalogue NSRE axes")?;
                let child_position = DVec3::new(
                    required_number(&child, "PX", env, "catalogue NSRE position")?,
                    required_number(&child, "PY", env, "catalogue NSRE position")?,
                    required_number(&child, "PZ", env, "catalogue NSRE position")?,
                );
                let child_transform = DMat4::from_translation(child_pa.origin + child_position)
                    * DMat4::from_mat3(child_rotation);
                Ok(Some(EvaluatedCatalogueCut::Revolution {
                    rings,
                    angle_deg,
                    transform: rebase * child_transform,
                }))
            })();
            match nested {
                Ok(Some(cutter)) => cutters.push(cutter),
                Ok(None) => {}
                Err(error) => diagnostics.push(format!(
                    "catalogue negative {} skipped by Core-style local failure: {error}",
                    child.refno()
                )),
            }
        } else if noun.eq_ignore_ascii_case("NSSP") {
            // CRCATI 701101/860747：负球与正体 SSPH 同臂（`QSSPHE` → ORIMAT →
            // `gm_CreateSphere(PDIA/2)`），**没有** +0.01 过切（反编译 2026-09-06 核实，
            // 过切只在 NSCO/NLCY/NSCY/NLSN 四臂上 `v48 += 0.01`）。
            cache.stats.source_nssp += 1;
            match evaluate_catalogue_sphere(member, component, &child, env) {
                Ok(Some(cutter)) => {
                    cache.stats.evaluated_nssp += 1;
                    let CatalogueMeshKey::Sphere { segments, .. } = cutter.mesh_key else {
                        unreachable!("sphere evaluator returned a non-sphere key")
                    };
                    cutters.push(EvaluatedCatalogueCut::Sphere {
                        segments,
                        transform: rebase * cutter.transform,
                    });
                }
                Ok(None) => {}
                Err(error) => diagnostics.push(format!(
                    "catalogue negative {} skipped by Core-style local failure: {error}",
                    child.refno()
                )),
            }
        } else if noun.eq_ignore_ascii_case("NSDS") {
            // CRCATI 702883/908861：负碟与正体 SDSH 同臂（`QSDISH`，球形 / 折边 / `PHEI=0`
            // 退化成 1 mm 圆盘三路都一样），**没有** +0.01 过切。
            cache.stats.source_nsds += 1;
            match evaluate_sdsh(member, component, &child, env) {
                Ok(Some(EvaluatedSdsh::Dish(dish))) => {
                    cache.stats.evaluated_nsds += 1;
                    cutters.push(EvaluatedCatalogueCut::Dish {
                        body: dish.body,
                        radius: dish.radius,
                        height: dish.height,
                        transform: rebase * dish.transform,
                    });
                }
                Ok(Some(EvaluatedSdsh::Disc(disc))) => {
                    cache.stats.evaluated_nsds += 1;
                    cutters.push(EvaluatedCatalogueCut::Cylinder(EvaluatedNscy {
                        diameter: disc.diameter,
                        height: disc.height,
                        transform: rebase * disc.transform,
                    }));
                }
                Ok(None) => {}
                Err(error) => diagnostics.push(format!(
                    "catalogue negative {} skipped by Core-style local failure: {error}",
                    child.refno()
                )),
            }
        } else if noun.eq_ignore_ascii_case("NSSL") {
            // CRCATI 599770/782015：负斜切圆柱与正体 SSLC 同臂（`QSSLCY` → ORIMAT →
            // `CSRTAX` 覆盖标架 → 四角折回 → `gm_CreateSlopeEndedCylinder`），**没有** +0.01 过切。
            // ams7997 的 C5SEGB / CELBOW（HVAC 分段弯头）一件就挂 4–10 只，此前整族静默漏掉。
            cache.stats.source_nssl += 1;
            match evaluate_sslc(member, component, &child, env) {
                Ok(Some(cutter)) => {
                    cache.stats.evaluated_nssl += 1;
                    cutters.push(EvaluatedCatalogueCut::SlopeCylinder {
                        radius: cutter.radius,
                        height: cutter.height,
                        shear_deg: cutter.shear_deg,
                        transform: rebase * cutter.transform,
                    });
                }
                Ok(None) => {}
                Err(error) => diagnostics.push(format!(
                    "catalogue negative {} skipped by Core-style local failure: {error}",
                    child.refno()
                )),
            }
        } else {
            // CRCATI 还剩 NBXI(726152) / NTUB(586670) 两条负臂没接：它们的正体 BOXI / TUBE
            // 本仓同样未建（`is not implemented`），30 个目录库的负体普查里一件都没有。
            diagnostics.push(format!(
                "catalogue negative {} noun {noun} is not implemented",
                child.refno()
            ));
        }
    }
    Ok((cutters, diagnostics))
}

fn evaluate_sbox(
    member: &DbElement,
    component: &DbElement,
    sbox: &DbElement,
    env: &DbElementParamEnv,
    cache: &mut CatalogueMeshCache,
) -> Result<Option<EvaluatedSbox>, ModelError> {
    let dimensions = DVec3::new(
        required_number(sbox, "PXLE", env, "catalogue SBOX")?.abs(),
        required_number(sbox, "PYLE", env, "catalogue SBOX")?.abs(),
        required_number(sbox, "PZLE", env, "catalogue SBOX")?.abs(),
    );
    // 任一边长为零的 SBOX 与零高 LCYL / SEXT 是同一件事：Core 不出体，目录里拿它当「关掉这个
    // 图元」的合法写法（HEADSTRT 的 `ZLEN=0`）。在这里按零高或退化收口；此前放行到 `box_solid`
    // 再撞 `Degenerate`，报告上就成了「局部失败」，与真出错的图元分不开。名下的负体一并不看，
    // Core 对不存在的正体也不会去扣。
    if dimensions.min_element() < HEIGHT_TOL_MM {
        return Ok(None);
    }
    let position = DVec3::new(
        required_number(sbox, "PX", env, "catalogue SBOX position")?,
        required_number(sbox, "PY", env, "catalogue SBOX position")?,
        required_number(sbox, "PZ", env, "catalogue SBOX position")?,
    );
    // SBOX 是纯平移放置：回到 SBOX 局部系就是减 position，与 `transform.inverse()` 逐位相同
    // （`translate(-p) * translate(c) * R == translate(c - p) * R`），所以名下的刀直接走
    // SREV / SEXT / 圆柱共用的那份收集器。此前这里另抄一份只认 NSCY/NSBO/NLCY/NLPY/NSEX/
    // NSCT/NSRT，NLSN/NSCO/NSRE 落 `is not implemented`——30 个目录库里 SBOX 名下有 14 只
    // NLSN、12 只 NSDS，"没语料"那条理由不成立；两份收集器的 noun 单子也再不用靠守卫对齐。
    let (cutters, diagnostics) = collect_owned_negatives(
        member,
        component,
        sbox,
        env,
        cache,
        DMat4::from_translation(-position),
    )?;
    let cutter_keys = catalogue_cut_keys(&cutters)?;
    let mesh_key = CatalogueMeshKey::Sbox(SboxMeshKey {
        tessellation_version: SBOX_TESSELLATION_VERSION,
        dimension_bits: [
            finite_bits(dimensions.x, "catalogue SBOX dimensions")?,
            finite_bits(dimensions.y, "catalogue SBOX dimensions")?,
            finite_bits(dimensions.z, "catalogue SBOX dimensions")?,
        ],
        cutters: cutter_keys,
    });
    Ok(Some(EvaluatedSbox {
        mesh_key,
        dimensions,
        cutters,
        transform: DMat4::from_translation(position),
        diagnostics,
    }))
}

/// Core3D `pplib/QLCYLI` (`0x103C0374`) builds both LCYL and NLCY from the
/// same two axial distances: height is `abs(PBDI-PTDI)` and the centre is their
/// midpoint along `PAXI`.  NLCY differs only by being consumed as a cutter.
fn evaluate_nlcy(
    member: &DbElement,
    component: &DbElement,
    nlcy: &DbElement,
    env: &DbElementParamEnv,
    sbox_position: DVec3,
) -> Result<Option<EvaluatedNscy>, ModelError> {
    let diameter = required_number(nlcy, "PDIA", env, "catalogue NLCY")?.abs();
    let top = required_number(nlcy, "PTDI", env, "catalogue NLCY")?;
    let bottom = required_number(nlcy, "PBDI", env, "catalogue NLCY")?;
    let height = (bottom - top).abs();
    // 零尺寸的刀与零尺寸的正体同一口径：Core 不出体，也就没有东西可扣——**跳过这把刀**，
    // 正体照建。此前回 `Degenerate`（零径更是漏判、放行到 `csg_cylinder_solid` 才撞），而
    // `build_sbox` / `build_srev` 对刀的 `?` 会把整只正体一起带走：ams8000 39 只 ATTA 的 SBOX
    // 就是这样被一把 `XLEN=0` 的 NSBO 整只跳掉的。
    if height < HEIGHT_TOL_MM || diameter < HEIGHT_TOL_MM {
        return Ok(None);
    }
    let axis = catalogue_axis_or_fail(member, component, nlcy, "PAXI", "catalogue NLCY axis")?;
    let rotation = orimat(axis.direction).ok_or_else(|| {
        ModelError::Solid(SolidError::Degenerate {
            what: "catalogue NLCY axis",
            detail: format!("PAXI={:?}", axis.direction),
        })
    })?;
    let centre = axis.origin + axis.direction * ((bottom + top) * 0.5) - sbox_position;
    Ok(Some(EvaluatedNscy {
        diameter,
        height,
        transform: DMat4::from_translation(centre) * DMat4::from_mat3(rotation),
    }))
}

fn evaluate_nsbo(
    nsbo: &DbElement,
    env: &DbElementParamEnv,
    sbox_position: DVec3,
) -> Result<Option<EvaluatedCatalogueCut>, ModelError> {
    let dimensions = DVec3::new(
        required_number(nsbo, "PXLE", env, "catalogue NSBO")?.abs(),
        required_number(nsbo, "PYLE", env, "catalogue NSBO")?.abs(),
        required_number(nsbo, "PZLE", env, "catalogue NSBO")?.abs(),
    );
    // 任一边长为零的刀不扣（见 `evaluate_nlcy`）。
    if dimensions.min_element() < HEIGHT_TOL_MM {
        return Ok(None);
    }
    let position = DVec3::new(
        required_number(nsbo, "PX", env, "catalogue NSBO position")?,
        required_number(nsbo, "PY", env, "catalogue NSBO position")?,
        required_number(nsbo, "PZ", env, "catalogue NSBO position")?,
    );
    Ok(Some(EvaluatedCatalogueCut::Box {
        dimensions,
        transform: DMat4::from_translation(position - sbox_position),
    }))
}

fn evaluate_nscy(
    member: &DbElement,
    component: &DbElement,
    nscy: &DbElement,
    env: &DbElementParamEnv,
    sbox_position: DVec3,
) -> Result<Option<EvaluatedNscy>, ModelError> {
    let diameter = required_number(nscy, "PDIA", env, "catalogue NSCY")?.abs();
    let mut height = required_number(nscy, "PHEI", env, "catalogue NSCY")?;
    let mut distance = required_number(nscy, "PDIS", env, "catalogue NSCY")?;
    // 零高 / 零径的刀不扣（见 `evaluate_nlcy`）；+0.01 的轴向过切只加在真有高度的刀上。
    if height.abs() < HEIGHT_TOL_MM || diameter < HEIGHT_TOL_MM {
        return Ok(None);
    }
    let mut axis = catalogue_axis_or_fail(member, component, nscy, "PAXI", "catalogue NSCY axis")?;
    if height <= 0.0 {
        height = -height;
        distance = -distance;
        axis.direction = -axis.direction;
    }
    let rotation = orimat(axis.direction).ok_or_else(|| {
        ModelError::Solid(SolidError::Degenerate {
            what: "catalogue NSCY axis",
            detail: format!("PAXI={:?}", axis.direction),
        })
    })?;
    // Core QSCYLI places the primitive at PDIS + PHEI/2 along PAXI. CRCATI
    // then adds 0.01 mm only to an NSCY's height, leaving that centre fixed.
    let centre = axis.origin + axis.direction * (distance + height * 0.5) - sbox_position;
    Ok(Some(EvaluatedNscy {
        diameter,
        height: height + NSCY_AXIAL_OVERCUT_MM,
        transform: DMat4::from_translation(centre) * DMat4::from_mat3(rotation),
    }))
}

fn uses_clfl_centreline_frame(member_noun: &str, primitive_noun: &str, clfl: bool) -> bool {
    // CLFL is a display flag, not an alternate SCYL axis. QSCYLI
    // (Core3D 0x103BF3FC) and CRCATI (0x10345ACC) use PAXI and its
    // midpoint without a quarter-turn. E3D TTY RVM for CPV STRT
    // 24381/76730..76732 confirms this for the body and all four flanges.
    // Keep the existing support-specific convention scoped to the two
    // families it was introduced for; never apply it to route components.
    clfl && primitive_noun.eq_ignore_ascii_case("SCYL")
        && (member_noun.eq_ignore_ascii_case("HELE") || member_noun.eq_ignore_ascii_case("PCLA"))
}

/// ATTA uses the catalogue axis directly; its member ORI is applied once by
/// world_matrix. Rotating here based on an Euler-angle sign double-applies
/// placement and turns PAXI=-Z into a horizontal cylinder (24384/26234,26235).
fn catalogue_cylinder_placement(
    member_noun: &str,
    noun: &str,
    clfl: bool,
    axis: CatalogueAxis,
    centre_distance: f64,
    height: f64,
) -> (DMat3, DVec3) {
    let frame = orimat(axis.direction).expect("validated catalogue cylinder axis");
    if uses_clfl_centreline_frame(member_noun, noun, clfl) {
        let rotation = frame * DMat3::from_rotation_x(-std::f64::consts::FRAC_PI_2);
        (
            rotation,
            axis.origin + axis.direction * centre_distance + rotation.z_axis * (height * 0.5),
        )
    } else {
        (frame, axis.origin + axis.direction * centre_distance)
    }
}

fn evaluate_catalogue_cylinder(
    member: &DbElement,
    component: &DbElement,
    cylinder: &DbElement,
    env: &DbElementParamEnv,
    noun: &str,
    cache: &mut CatalogueMeshCache,
) -> Result<CylinderOutcome, ModelError> {
    let diameter = required_number(cylinder, "PDIA", env, "catalogue cylinder")?.abs();
    let mut axis = catalogue_axis_or_fail(
        member,
        component,
        cylinder,
        "PAXI",
        "catalogue cylinder axis",
    )?;
    let (height, centre_distance) = if noun == "SCYL" {
        let mut height = required_number(cylinder, "PHEI", env, "catalogue SCYL")?;
        let mut distance = required_number(cylinder, "PDIS", env, "catalogue SCYL")?;
        if height <= 0.0 {
            height = -height;
            distance = -distance;
            axis.direction = -axis.direction;
        }
        (height, distance + height * 0.5)
    } else {
        let top = required_number(cylinder, "PTDI", env, "catalogue LCYL")?;
        let bottom = required_number(cylinder, "PBDI", env, "catalogue LCYL")?;
        let centre = (bottom + top) * 0.5;
        ((bottom - top).abs(), centre)
    };
    let member_noun = member.element_type()?;
    let clfl = cylinder.get_bool("CLFL")? == Some(true);
    let (rotation, centre) =
        catalogue_cylinder_placement(&member_noun, noun, clfl, axis, centre_distance, height);
    let transform = DMat4::from_translation(centre) * DMat4::from_mat3(rotation);
    // 零高与零径同一口径（`disc_at` / SSLC 那几条臂早就这么判）：都是 Core 不出体的合法
    // 参数化关法，归零高或退化。零径此前漏判，放行到 `csg_cylinder_solid` 撞 `Degenerate`
    // 被记成局部失败——ams7997 FLEX 的 40 只 `PDIA=0` LCYL 一库就是 480 条。
    //
    // 放置在这一步之前就算好：不出体的那只也要出记录，得把放置和参数一起交出去
    // （见 [`CylinderOutcome`] 与 [`crate::elmodl::DegenerateCataloguePart`]）。
    if height < HEIGHT_TOL_MM || diameter < HEIGHT_TOL_MM {
        return Ok(CylinderOutcome::Bodyless {
            transform,
            radius: diameter * 0.5,
            height,
        });
    }
    let rebase = transform.inverse();
    // 圆柱正体（SCYL/LCYL）名下的负体走 SREV/SEXT 同一份收集器：此前这条臂只认
    // NSCY/NLCY，其余（NSCT/NSRT/NSBO/NSEX/NLPY/NLSN/NSCO/NSRE）一律落
    // `noun … is not implemented` 静默丢掉——ams7997 的 INST 24381/637 那只 SCYL 名下 4 只
    // NSCT（法兰环切槽）、另有 21 只 NSBO 就是这么漏的：拿这只 INST 对 E3D 的 TTY RVM，
    // 补上后 E3D 顶点离我们表面 >0.5mm 的从 48 个（最大 1.39mm）降到 0（最大 0.49mm）。
    // 回位约定与 SEXT 同（`rebase = transform.inverse()`，圆柱放置带轴向旋转，减 position 不够）。
    let (cutters, diagnostics) =
        collect_owned_negatives(member, component, cylinder, env, cache, rebase)?;
    let base = CatalogueMeshKey::Cylinder {
        tessellation_version: CYLINDER_TESSELLATION_VERSION,
        diameter_bits: finite_bits(diameter, "catalogue cylinder diameter")?,
        height_bits: finite_bits(height, "catalogue cylinder height")?,
    };
    let mesh_key = if cutters.is_empty() {
        base
    } else {
        CatalogueMeshKey::Cut {
            base: Box::new(base),
            cutters: catalogue_cut_keys(&cutters)?,
        }
    };
    Ok(CylinderOutcome::Built(EvaluatedCylinder {
        mesh_key,
        diameter,
        height,
        transform,
        cutters,
        diagnostics,
    }))
}

/// Boolean catalogue cylinders use the FacetGroup grid for both operands.
/// Plain cylinders keep their existing finer primitive tessellation.
fn build_catalogue_cylinder(e: &EvaluatedCylinder) -> Result<Manifold, ModelError> {
    use crate::solid::{csg_cylinder_solid_with_segments, negative_cylinder_segments};
    if e.cutters.is_empty() {
        return Ok(csg_cylinder_solid(e.diameter, e.height)?);
    }
    let mut body = csg_cylinder_solid_with_segments(
        e.diameter,
        e.height,
        negative_cylinder_segments(e.diameter / 2.0),
    )?;
    for cutter in &e.cutters {
        let (local, transform) = match cutter {
            EvaluatedCatalogueCut::Cylinder(c) => (
                csg_cylinder_solid_with_segments(
                    c.diameter,
                    c.height,
                    negative_cylinder_segments(c.diameter / 2.0),
                )?,
                c.transform,
            ),
            _ => cutter_solid(cutter)?,
        };
        body = &body - &placed(&local, transform);
    }
    if body.is_empty() || body.num_tri() == 0 {
        return Err(ModelError::Solid(SolidError::EmptyManifold {
            what: "catalogue cylinder after negatives",
        }));
    }
    Ok(body)
}

fn cached_mesh_id(
    cache: &mut CatalogueMeshCache,
    key: &CatalogueMeshKey,
    build: impl FnOnce() -> Result<Manifold, ModelError>,
) -> Result<usize, ModelError> {
    if let Some(cached) = cache.local_meshes.get(key) {
        cache.stats.local_mesh_hits += 1;
        return Ok(cached.id);
    }
    let _timing = crate::perf::mesh_build_scope();
    let id = cache.local_meshes.len();
    let solid = build()?;
    debug_assert_eq!(cache.mesh_tri_counts.len(), id, "mesh id 就是这张表的下标");
    cache.mesh_tri_counts.push(solid.num_tri());
    cache
        .local_meshes
        .insert(key.clone(), CachedCatalogueMesh { id, solid });
    cache.stats.local_mesh_misses += 1;
    Ok(id)
}

/// Core3D `QSSPHE` (`0x103C2480`) reads `abs(PDIA)`, resolves `PAXI`, then
/// translates the axis origin by `PDIS * normalize(PAXI)`. CRCATI
/// (`0x10345ACC`) passes `PDIA / 2` to the sphere constructor.
fn evaluate_catalogue_sphere(
    member: &DbElement,
    component: &DbElement,
    sphere: &DbElement,
    env: &DbElementParamEnv,
) -> Result<Option<EvaluatedSphere>, ModelError> {
    let diameter = required_number(sphere, "PDIA", env, "catalogue sphere")?.abs();
    let radius = diameter * 0.5;
    if !(radius > f64::EPSILON) {
        return Ok(None);
    }
    let axis = catalogue_axis_or_fail(member, component, sphere, "PAXI", "catalogue sphere axis")?;
    let direction = axis.direction.normalize_or_zero();
    if direction.length_squared() <= AXIS_TOL {
        return Err(ModelError::Solid(SolidError::Degenerate {
            what: "catalogue sphere axis",
            detail: format!("PAXI={:?}", axis.direction),
        }));
    }
    let distance = required_number(sphere, "PDIS", env, "catalogue sphere")?;
    let centre = axis.origin + direction * distance;
    let segments = cylinder_segments(radius);
    let rotation = orimat(direction).ok_or_else(|| {
        ModelError::Solid(SolidError::Degenerate {
            what: "catalogue sphere axis",
            detail: format!("PAXI={direction:?} 配不出 ORIMAT"),
        })
    })?;
    Ok(Some(EvaluatedSphere {
        mesh_key: CatalogueMeshKey::Sphere {
            tessellation_version: SPHERE_TESSELLATION_VERSION,
            segments,
        },
        transform: DMat4::from_translation(centre) * DMat4::from_scale(DVec3::splat(radius)),
        record_frame: DMat4::from_translation(centre) * DMat4::from_mat3(rotation),
    }))
}

// ---------------------------------------------------------------------------
// G0-1c 低频臂：SSLC 斜切圆柱 / SDSH 碟形封头 / SDIS 圆盘。
//
// 三条 pplib 求值器（QSSLCY / QSDISH / QSDISC）与 CRCATI 对应的 case 臂都反编译在案，
// 属性名从 `unk_10B6F5xx/6xx/8xx` 的 27 进制哈希反查（PDIS/PHEI/PDIA/PRAD/PAXI/
// PXTS/PYTS/PXBS/PYBS），不是按形参猜的。形体本身见 `solid.rs` 同名 builder。
// ---------------------------------------------------------------------------

/// 一块建好的目录图元：实体，加上它出自哪个目录图元。
///
/// 出身要一路带到 RVM 写出器，因为**记录种类由目录图元定，不由网格长相定**：E3D 的
/// LPYR 一律走 DRPYR、LSNO/SCON 一律走 DRSNO，一只 PAAX/PBAX 退化成正方盒的 LPYR
/// 照样写 `Pyramid`。只交网格出去，下游的识别器就只能照长相猜，猜出来的是 `Box`。
pub(crate) struct CataloguePart {
    pub(crate) solid: Manifold,
    /// 目录图元的名词（`SBOX` / `SCYL` / `LPYR` / `LSNO`…）。
    pub(crate) noun: String,
    /// 把局部网格摆进元素局部系的那只矩阵：列 = 图元自己的 X / Y / Z（圆柱是 `ORIMAT(PAXI)`、
    /// LSNO 是 PAAX / PBAX 标架…），平移 = 图元自己的原点。RVM 写出器照它写记录的矩阵，
    /// 见 [`crate::elmodl::CataloguePartOrigin`]。
    pub(crate) frame: DMat4,
    /// 这一个放置在 GMSE 里出现了几次：1 = 只此一块；2 = 另有一个成员算出了同一只网格、
    /// 同一个放置。体只放一块，记录按这个数写，见 [`place_part_once`]。
    pub(crate) repeats: usize,
    /// 网格键是折边碟（`PRAD > 0`），含被负体包了一层的那只。写出器据此选
    /// `EllipticalDish`，不从网格长相猜——r = h 时两种碟是同一只半球。
    pub(crate) knuckled_dish: bool,
}

/// 取网格 → 放置去重 → 摆放。G0-1c 三条臂共用这一份；其余臂的手抄版见计划 G2-5。
fn place_cached_part(
    cache: &mut CatalogueMeshCache,
    placed_parts: &mut HashMap<PlacedPartKey, usize>,
    solids: &mut Vec<CataloguePart>,
    noun: &str,
    mesh_key: &CatalogueMeshKey,
    transform: DMat4,
    build: impl FnOnce() -> Result<Manifold, ModelError>,
) -> Result<bool, ModelError> {
    let mesh_id = cached_mesh_id(cache, mesh_key, build)?;
    Ok(place_part_once(
        cache,
        placed_parts,
        solids,
        noun,
        mesh_key,
        mesh_id,
        transform,
    ))
}

/// 把一块已进缓存的目录网格按 `transform` 放进 `solids`。返回是否真的放了一块新体。
///
/// **同一只网格、同一个放置只放一块体，重复的只记一笔。** 两块全同的体并起来是自交的，
/// `Manifold::compose` 吃不下；体积也会翻倍。但 E3D 的 RVM 驱动逐 GMSE 成员写记录，
/// 不管它们是不是长在同一个地方：`/Copy-of-7#管线-1/B1` 的 `VALV 3` 基准里第 5、6 两条是
/// **字节一样**的两只圆柱（80 根严尺子上这样的重复 54 处，45 件元素）。所以记录那一层要
/// 照着补——`placed_parts` 记的是先放那一块在 `solids` 里的下标，重复时给它 `repeats` 加一，
/// 写 RVM 时那一块的记录多写几遍（[`crate::elmodl::GeneratedElement::part_record_repeats`]）。
fn place_part_once(
    cache: &mut CatalogueMeshCache,
    placed_parts: &mut HashMap<PlacedPartKey, usize>,
    solids: &mut Vec<CataloguePart>,
    noun: &str,
    mesh_key: &CatalogueMeshKey,
    mesh_id: usize,
    transform: DMat4,
) -> bool {
    let placed_key = PlacedPartKey {
        mesh_id,
        transform_bits: matrix_bits(transform),
    };
    if let Some(&index) = placed_parts.get(&placed_key) {
        solids[index].repeats += 1;
        cache.stats.duplicate_placed_parts += 1;
        return false;
    }
    placed_parts.insert(placed_key, solids.len());
    cache.stats.unique_placed_parts += 1;
    let local_mesh = &cache
        .local_meshes
        .get(mesh_key)
        .expect("catalogue mesh inserted above")
        .solid;
    solids.push(CataloguePart {
        solid: placed(local_mesh, transform),
        noun: noun.to_string(),
        frame: transform,
        repeats: 1,
        knuckled_dish: mesh_key_is_knuckled_dish(mesh_key),
    });
    true
}

/// `SDSH` 的 `PRAD > 0` 留在网格键的 `knuckled` 上。负体把键包成 [`CatalogueMeshKey::Cut`]
/// 之后，开关仍在里面那只碟上。
fn mesh_key_is_knuckled_dish(key: &CatalogueMeshKey) -> bool {
    match key {
        CatalogueMeshKey::Dish { knuckled, .. } => *knuckled,
        CatalogueMeshKey::Cut { base, .. } => mesh_key_is_knuckled_dish(base),
        _ => false,
    }
}

fn local_failure(primitive: &DbElement, error: &ModelError) -> String {
    format!(
        "catalogue primitive {} skipped by Core-style local failure: {error}",
        primitive.refno()
    )
}

/// `CSG_TreeBuilderCat::getCSGTree`（`0x1072F5D0`）：正体只要有成员就开一个
/// `gm_CreateCombination(3)` 差集，`addNegatives`（`0x1072F480`）逐成员过 `isWanted`
/// 后送进 CRCATI，建得出来的全扣掉——**不分正体 noun**。SEXT / SREV / SBOX / 圆柱四臂在
/// 各自的求值器里收刀；棱锥 / 锥台 / 环面 / 球 / 斜切圆柱 / 碟 / 圆盘七臂此前一把都不收，
/// 42 个目录库里这八种属主名下约 5200 只负体连一行 note 都没有就没了（LPYR 3683 /
/// LSNO 794 / SSLC 413 / SRTO 161 / SCTO 95 / SDSH 25 / SSPH 22 / SCON 2；
/// 普查 2026-09-06）。
///
/// 这里做 Core3D 那一步的前半：收刀、把网格键包成 [`CatalogueMeshKey::Cut`]——无刀时键
/// 不变，缓存不分岔。刀已回到正体局部系（`transform.inverse()`，与圆柱臂同一约定）。
/// 收刀本身失败不拦正体：Core3D 也是正体先进组合、刀再逐把加，这里落一行 note 然后
/// 按无刀建体。
fn owned_negatives_of(
    member: &DbElement,
    component: &DbElement,
    primitive: &DbElement,
    env: &DbElementParamEnv,
    cache: &mut CatalogueMeshCache,
    base_key: CatalogueMeshKey,
    transform: DMat4,
) -> (CatalogueMeshKey, Vec<EvaluatedCatalogueCut>, Vec<String>) {
    let negatives_failure = |error: &ModelError| {
        format!(
            "catalogue primitive {} negatives skipped by Core-style local failure: {error}",
            primitive.refno()
        )
    };
    let (cutters, mut diagnostics) = match collect_owned_negatives(
        member,
        component,
        primitive,
        env,
        cache,
        transform.inverse(),
    ) {
        Ok(collected) => collected,
        Err(error) => return (base_key, Vec::new(), vec![negatives_failure(&error)]),
    };
    if cutters.is_empty() {
        return (base_key, cutters, diagnostics);
    }
    match catalogue_cut_keys(&cutters) {
        Ok(keys) => (
            CatalogueMeshKey::Cut {
                base: Box::new(base_key),
                cutters: keys,
            },
            cutters,
            diagnostics,
        ),
        Err(error) => {
            diagnostics.push(negatives_failure(&error));
            (base_key, Vec::new(), diagnostics)
        }
    }
}

/// Core3D 那一步的后半：正体局部网格逐把扣掉名下的刀。空刀集原样返回——无刀时与旧路
/// 逐位相同。扣空了是错（Core 的差集出空体同样什么都不画，但这里要它落 note 而不是静默）。
fn subtract_owned_negatives(
    mut body: Manifold,
    cutters: &[EvaluatedCatalogueCut],
    what: &'static str,
) -> Result<Manifold, ModelError> {
    for cutter in cutters {
        let (local, transform) = cutter_solid(cutter)?;
        body = &body - &placed(&local, transform);
    }
    if !cutters.is_empty() && (body.is_empty() || body.num_tri() == 0) {
        return Err(ModelError::Solid(SolidError::EmptyManifold { what }));
    }
    Ok(body)
}

/// `CSRTAX`（core.dll mathlib，`0x560ad30`）：轴向量 → 标架，CRCATI 只在 SSLC/NSSL 臂上
/// 用它**整个覆盖**前一步 `ORIMAT` 的结果（`CSRTAX(a2, dir, &3)`：轴放进第 3 列）。
///
/// ```text
/// Z = normalize(dir)                       退化 → None
/// d = Z · Ẑ
/// | |d| − 1 | ≥ 1e-5 :  X = normalize(Z × Ẑ),  Y = normalize(Z × X)
/// 否则 d < 0          :  X = −X̂,  Y = Ŷ         （Z 仍是 normalize(dir)）
/// 否则                 :  X = X̂,   Y = Ŷ
/// ```
///
/// 与 [`orimat`] 只差副轴的 tie-break：ORIMAT 先试 `Z × X̂`，CSRTAX 一律 `Z × Ẑ`。
/// 竖直轴两者一致，**水平轴不同**（`+X̂` 轴：ORIMAT 给 X=Ẑ/Y=−Ŷ，CSRTAX 给 X=−Ŷ/Y=−Ẑ），
/// 而 `PXTS/PYTS/PXBS/PYBS` 四个斜切角正是相对这套 X/Y 定义的——拿 ORIMAT 摆 SSLC，
/// 端面就歪到别的方向去（旧 gen-model 在 `24381/47066` 上反复对不上的根子）。
pub(crate) fn csrtax(dir: DVec3) -> Option<DMat3> {
    let length = dir.length();
    if !(length > AXIS_TOL) || !dir.is_finite() {
        return None;
    }
    let z = dir / length;
    let along_world_z = z.dot(DVec3::Z);
    if (along_world_z.abs() - 1.0).abs() >= 1.0e-5 {
        let x = z.cross(DVec3::Z).normalize();
        let y = z.cross(x).normalize();
        Some(DMat3::from_cols(x, y, z))
    } else if along_world_z < 0.0 {
        Some(DMat3::from_cols(-DVec3::X, DVec3::Y, z))
    } else {
        Some(DMat3::from_cols(DVec3::X, DVec3::Y, z))
    }
}

/// CRCATI 599770 臂的角度折回：`> 90 → −180`，`< −90 → +180`，各只折一次；
/// 折后仍 `|角| ≥ 90` 的，`GM_SlopeEndCyl::calcFacets` 交空面片组。
fn fold_shear_angle(deg: f64) -> f64 {
    if deg > 90.0 {
        deg - 180.0
    } else if deg < -90.0 {
        deg + 180.0
    } else {
        deg
    }
}

/// 开关 / 角度一类的槽位：Core 的 `DGETI` 对 unset 给 0（`APARAM` 的整数回落、
/// `DECARG` 同款），照抄；能解的公式照解，解不出仍按 [`required_number`] 报错。
fn number_or_zero_when_unset(
    element: &DbElement,
    name: &'static str,
    env: &DbElementParamEnv,
    what: &'static str,
) -> Result<f64, ModelError> {
    if matches!(element.get_number(name, env)?, AttributeNumber::Unset) {
        return Ok(0.0);
    }
    required_number(element, name, env, what)
}

fn unit_axis(
    axis: &crate::catalogue_point::CatalogueAxis,
    what: &'static str,
) -> Result<DVec3, ModelError> {
    let direction = axis.direction.normalize_or_zero();
    if direction.length_squared() <= AXIS_TOL {
        return Err(ModelError::Solid(SolidError::Degenerate {
            what,
            detail: format!("PAXI={:?}", axis.direction),
        }));
    }
    Ok(direction)
}

/// CRCATI 的圆盘：`gm_CreateCylinder(PDIA / 2, 1.0)`，`ORIMAT(轴)` 标架，对中放在 `centre`。
/// 与 SCYL 同一张 [`CatalogueMeshKey::Cylinder`] 网格，直径相同的圆盘只三角化一次。
fn disc_at(
    centre: DVec3,
    direction: DVec3,
    diameter: f64,
    what: &'static str,
) -> Result<Option<EvaluatedCylinder>, ModelError> {
    if diameter < HEIGHT_TOL_MM {
        return Ok(None);
    }
    let rotation = orimat(direction).ok_or_else(|| {
        ModelError::Solid(SolidError::Degenerate {
            what,
            detail: format!("PAXI={direction:?}"),
        })
    })?;
    Ok(Some(EvaluatedCylinder {
        mesh_key: CatalogueMeshKey::Cylinder {
            tessellation_version: CYLINDER_TESSELLATION_VERSION,
            diameter_bits: finite_bits(diameter, "catalogue disc diameter")?,
            height_bits: canonical_bits(DISC_THICKNESS_MM),
        },
        diameter,
        height: DISC_THICKNESS_MM,
        cutters: Vec::new(),
        diagnostics: Vec::new(),
        transform: DMat4::from_translation(centre) * DMat4::from_mat3(rotation),
    }))
}

/// Core3D `QSDISC`（`pplib`，`0x103C25E0`）+ CRCATI 912106 臂：读 `PDIS`、`|PDIA|`、`PAXI`，
/// 轴原点沿单位轴平移 `PDIS`，标架 `ORIMAT`，形体是 1 mm 厚的圆盘（见 [`disc_at`]）。
fn evaluate_sdis(
    member: &DbElement,
    component: &DbElement,
    disc: &DbElement,
    env: &DbElementParamEnv,
) -> Result<Option<EvaluatedCylinder>, ModelError> {
    let distance = required_number(disc, "PDIS", env, "catalogue SDIS")?;
    let diameter = required_number(disc, "PDIA", env, "catalogue SDIS")?.abs();
    let axis = catalogue_axis_or_fail(member, component, disc, "PAXI", "catalogue SDIS axis")?;
    let direction = unit_axis(&axis, "catalogue SDIS axis")?;
    disc_at(
        axis.origin + direction * distance,
        direction,
        diameter,
        "catalogue SDIS axis",
    )
}

/// Core3D `QSDISH`（`pplib`，`0x103C0CBC`）+ CRCATI 702883/908861 臂。
///
/// 读 `PDIS`、带符号 `PHEI`、`|PDIA|`、`|PRAD|`、`PAXI`。`PHEI ≤ 0` 走 `HCHECK`：高度取反、
/// 轴取反、`PDIS` 取反——底面圆心因此**始终**落在 `origin + PDIS·PAXI`，碟只是改朝
/// `sign(PHEI)·PAXI` 鼓起。标架 `ORIMAT(轴)`，libgm 的碟底在 z = 0、顶点在 z = +h。
///
/// CRCATI 再分三路：`PDIA ≤ 0` 无形体；`PHEI > 0` 且 `PRAD > 0` → `CREDSH` 折边碟
/// （折边半径由 r/h 算出，`PRAD` 的数值不进几何）；`PRAD ≤ 0` → 球形碟；
/// `PHEI = 0` → 1 mm 圆盘，与 SDIS 同一条 `gm_CreateCylinder`。
/// 负臂 NSDS 与正臂同一条路，**没有** NSCY/NLCY/NSCO 那道 +0.01 过切。
fn evaluate_sdsh(
    member: &DbElement,
    component: &DbElement,
    dish: &DbElement,
    env: &DbElementParamEnv,
) -> Result<Option<EvaluatedSdsh>, ModelError> {
    let mut distance = required_number(dish, "PDIS", env, "catalogue SDSH")?;
    let mut height = required_number(dish, "PHEI", env, "catalogue SDSH")?;
    let diameter = required_number(dish, "PDIA", env, "catalogue SDSH")?.abs();
    let knuckle_switch = number_or_zero_when_unset(dish, "PRAD", env, "catalogue SDSH")?.abs();
    let axis = catalogue_axis_or_fail(member, component, dish, "PAXI", "catalogue SDSH axis")?;
    let mut direction = unit_axis(&axis, "catalogue SDSH axis")?;
    if height <= 0.0 {
        height = -height;
        direction = -direction;
        distance = -distance;
    }
    let base_centre = axis.origin + direction * distance;
    if !(diameter > 0.0) {
        return Ok(None);
    }
    if height <= 0.0 {
        return Ok(
            disc_at(base_centre, direction, diameter, "catalogue SDSH axis")?
                .map(EvaluatedSdsh::Disc),
        );
    }
    let rotation = orimat(direction).ok_or_else(|| {
        ModelError::Solid(SolidError::Degenerate {
            what: "catalogue SDSH axis",
            detail: format!("PAXI={direction:?}"),
        })
    })?;
    let radius = diameter * 0.5;
    let body = if knuckle_switch > 0.0 {
        EvaluatedDishBody::Knuckled
    } else {
        EvaluatedDishBody::Spherical
    };
    Ok(Some(EvaluatedSdsh::Dish(EvaluatedDish {
        mesh_key: CatalogueMeshKey::Dish {
            tessellation_version: DISH_TESSELLATION_VERSION,
            knuckled: matches!(body, EvaluatedDishBody::Knuckled),
            radius_bits: finite_bits(radius, "catalogue SDSH radius")?,
            height_bits: finite_bits(height, "catalogue SDSH height")?,
        },
        body,
        radius,
        height,
        transform: DMat4::from_translation(base_centre) * DMat4::from_mat3(rotation),
    })))
}

fn build_dish(dish: &EvaluatedDish) -> Result<Manifold, SolidError> {
    match dish.body {
        EvaluatedDishBody::Spherical => spherical_dish_solid(dish.radius, dish.height),
        EvaluatedDishBody::Knuckled => elliptical_dish_solid(
            dish.radius,
            dish.height,
            dish_knuckle_radius(dish.radius, dish.height),
        ),
    }
}

/// Core3D `QSSLCY`（`pplib`，`0x103C0514`）+ CRCATI 599770/782015 臂。
///
/// 读 `PDIS`、带符号 `PHEI`、`|PDIA|`、四个斜切角 `PXTS/PYTS/PXBS/PYBS`（`APARAM`：度→弧度，
/// CRCATI 再乘回 57.2958，净值就是**度**；字典把这四个槽标成角度型，整数槽是百分度，
/// e3d-io 解码时已除 100）、`PAXI`。`PHEI ≤ 0` 走 `HCHECK`：高度、轴、`PDIS`、
/// **四个角一起取反**；圆柱中心在 `origin + (PDIS + h/2)·轴`，与 SCYL 同款。
///
/// 标架先 `ORIMAT` 后被 [`csrtax`] **整个覆盖**，四个角相对那套 X/Y：
/// `z_top = +h/2 + x·tan(PXTS) + y·tan(PYTS)`，`z_bot = −h/2 + x·tan(PXBS) + y·tan(PYBS)`。
/// 角度按 [`fold_shear_angle`] 折回一次；折后仍 `|角| ≥ 90°` 的，libgm 交空面片组，
/// 这里同样不出形体。形体是 `gm_CreateSlopeEndedCylinder(r, h, PXBS, PYBS, PXTS, PYTS)`。
fn evaluate_sslc(
    member: &DbElement,
    component: &DbElement,
    cylinder: &DbElement,
    env: &DbElementParamEnv,
) -> Result<Option<EvaluatedSlopeCylinder>, ModelError> {
    let mut distance = required_number(cylinder, "PDIS", env, "catalogue SSLC")?;
    let mut height = required_number(cylinder, "PHEI", env, "catalogue SSLC")?;
    let diameter = required_number(cylinder, "PDIA", env, "catalogue SSLC")?.abs();
    let mut shear_deg = [
        number_or_zero_when_unset(cylinder, "PXTS", env, "catalogue SSLC")?,
        number_or_zero_when_unset(cylinder, "PYTS", env, "catalogue SSLC")?,
        number_or_zero_when_unset(cylinder, "PXBS", env, "catalogue SSLC")?,
        number_or_zero_when_unset(cylinder, "PYBS", env, "catalogue SSLC")?,
    ];
    let axis = catalogue_axis_or_fail(member, component, cylinder, "PAXI", "catalogue SSLC axis")?;
    let mut direction = unit_axis(&axis, "catalogue SSLC axis")?;
    if height <= 0.0 {
        height = -height;
        direction = -direction;
        distance = -distance;
        shear_deg = shear_deg.map(|angle| -angle);
    }
    let centre = axis.origin + direction * (distance + height * 0.5);
    let shear_deg = shear_deg.map(fold_shear_angle);
    if height < HEIGHT_TOL_MM || diameter < HEIGHT_TOL_MM {
        return Ok(None);
    }
    if shear_deg
        .iter()
        .any(|angle| !angle.is_finite() || angle.abs() >= 90.0)
    {
        return Ok(None);
    }
    let rotation = csrtax(direction).ok_or_else(|| {
        ModelError::Solid(SolidError::Degenerate {
            what: "catalogue SSLC axis",
            detail: format!("PAXI={direction:?}"),
        })
    })?;
    let radius = diameter * 0.5;
    let parameters = [
        radius,
        height,
        shear_deg[0],
        shear_deg[1],
        shear_deg[2],
        shear_deg[3],
    ];
    let mut parameter_bits = [0u64; 6];
    for (bits, value) in parameter_bits.iter_mut().zip(parameters) {
        *bits = finite_bits(value, "catalogue SSLC parameter")?;
    }
    Ok(Some(EvaluatedSlopeCylinder {
        mesh_key: CatalogueMeshKey::SlopeCylinder {
            tessellation_version: SLOPE_CYLINDER_TESSELLATION_VERSION,
            parameter_bits,
        },
        radius,
        height,
        shear_deg,
        transform: DMat4::from_translation(centre) * DMat4::from_mat3(rotation),
    }))
}

fn evaluate_catalogue_torus(
    member: &DbElement,
    component: &DbElement,
    torus: &DbElement,
    env: &DbElementParamEnv,
    noun: &str,
) -> Result<Option<EvaluatedTorus>, ModelError> {
    let pa = catalogue_axis_or_fail(member, component, torus, "PAAX", "catalogue torus axis")?;
    let pb = catalogue_axis_or_fail(member, component, torus, "PBAX", "catalogue torus axis")?;
    let Some((rotation, centre, centre_radius, angle_deg)) = torus_frame(pa, pb)? else {
        return Ok(None);
    };
    if angle_deg.abs() <= 1.0e-6 {
        return Ok(None);
    }
    let diameter = required_number(torus, "PDIA", env, "catalogue torus")?.abs();
    let inner = (centre_radius - diameter * 0.5).max(0.0);
    let outer = centre_radius + diameter * 0.5;
    let kind = if noun == "SCTO" {
        EvaluatedTorusKind::Circular
    } else {
        EvaluatedTorusKind::Rectangular {
            height: required_number(torus, "PHEI", env, "catalogue SRTO")?.abs(),
        }
    };
    let mesh_key = match kind {
        EvaluatedTorusKind::Circular => CatalogueMeshKey::CircularTorus {
            tessellation_version: TORUS_TESSELLATION_VERSION,
            inner_bits: finite_bits(inner, "catalogue SCTO inner radius")?,
            outer_bits: finite_bits(outer, "catalogue SCTO outer radius")?,
            angle_bits: finite_bits(angle_deg, "catalogue SCTO angle")?,
        },
        EvaluatedTorusKind::Rectangular { height } => CatalogueMeshKey::RectangularTorus {
            tessellation_version: TORUS_TESSELLATION_VERSION,
            inner_bits: finite_bits(inner, "catalogue SRTO inner radius")?,
            outer_bits: finite_bits(outer, "catalogue SRTO outer radius")?,
            height_bits: finite_bits(height, "catalogue SRTO height")?,
            angle_bits: finite_bits(angle_deg, "catalogue SRTO angle")?,
        },
    };
    Ok(Some(EvaluatedTorus {
        mesh_key,
        kind,
        inner,
        outer,
        angle_deg,
        // The shared builders retain libgm's native XY-sweep/Z-height frame,
        // which is the same frame completed by PAAX/PBAX here.
        transform: DMat4::from_translation(centre) * DMat4::from_mat3(rotation),
    }))
}

/// 两根轴定一只环：`None` = 这只环退化，E3D 也不画。
///
/// 两根轴平行时圆心只能由两个 P 点的间距定：间距垂直于轴，就是那只 180° 的 U 形弯；
/// 间距为零、或顺着轴，圆心与半径都无解——**那不是我们解不开，是它本来就不是一只环**。
///
/// 这两种此前报 `Err`，于是 80 根的取证账上挂着 50 条「catalogue torus axes 尺寸退化」，
/// 看着像 50 只丢掉的环。逐件对过基准，**E3D 一只都没画**：
///
/// * 间距为零（36 条，ATTA 上的 `23045/16` 与 `/17`）：`/PS-0003` 那只 ATTA 两侧都只有
///   一条 `Sphere`，kind 一致、AABB 差 0。
/// * 间距顺着轴（14 条，PCOM 上的 `21385/8231` 与 `/8233`，delta 480 / 540 mm）：
///   `PCOM 1 /Copy-(2)-of-油气回收供油管线/B2` 两侧都只有三条 `Cylinder`，同样逐项对上。
///
/// 所以这里退成 `Ok(None)`，与别的退化图元一样计进 `degenerate_other`——账要是一直
/// 喊狼来了，下一个人还得把这条路再走一遍。真正解不开的（半径非有限、轴退化）仍旧报错。
fn torus_frame(
    pa: crate::catalogue_point::CatalogueAxis,
    pb: crate::catalogue_point::CatalogueAxis,
) -> Result<Option<(DMat3, DVec3, f64, f64)>, ModelError> {
    let a = pa.direction.normalize_or_zero();
    let mut y = -pb.direction.normalize_or_zero();
    let mut z = a.cross(pb.direction).normalize_or_zero();
    let mut x = y.cross(z).normalize_or_zero();
    let cosine = a.dot(y);
    let (centre, radius, angle) = if cosine.abs() > 0.99999 {
        let delta = pb.origin - pa.origin;
        x = delta.normalize_or_zero();
        if x.length_squared() <= AXIS_TOL || x.dot(y).abs() > 1.0e-5 {
            return Ok(None);
        }
        z = x.cross(y).normalize_or_zero();
        (
            (pa.origin + pb.origin) * 0.5,
            delta.length() * 0.5,
            if cosine > 0.0 {
                0.0
            } else {
                std::f64::consts::PI
            },
        )
    } else {
        let sine = (1.0 - cosine * cosine).sqrt();
        let radius = ((pa.origin - pb.origin).dot(y) / sine).abs();
        (pb.origin - radius * x, radius, sine.atan2(cosine))
    };
    if !(radius > HEIGHT_TOL_MM)
        || !centre.is_finite()
        || x.length_squared() <= AXIS_TOL
        || y.length_squared() <= AXIS_TOL
        || z.length_squared() <= AXIS_TOL
    {
        return Err(ModelError::Solid(SolidError::Degenerate {
            what: "catalogue torus frame",
            detail: format!("radius={radius} centre={centre:?} axes={x:?}/{y:?}/{z:?}"),
        }));
    }
    y = z.cross(x).normalize_or_zero();
    Ok(Some((
        DMat3::from_cols(x, y, z),
        centre,
        radius,
        angle.to_degrees(),
    )))
}

/// 一把刀的键。SBOX 与 SREV 的刀集共用这一份——键的口径分叉，缓存就会把
/// 两把不同的刀当同一把用。
fn catalogue_cut_keys(
    cutters: &[EvaluatedCatalogueCut],
) -> Result<Vec<CatalogueCutKey>, ModelError> {
    cutters
        .iter()
        .map(|cutter| match cutter {
            EvaluatedCatalogueCut::Cylinder(cutter) => Ok(CatalogueCutKey::Cylinder {
                diameter_bits: finite_bits(cutter.diameter, "catalogue NSCY diameter")?,
                height_bits: finite_bits(cutter.height, "catalogue NSCY height")?,
                transform_bits: matrix_bits(cutter.transform),
            }),
            EvaluatedCatalogueCut::Box {
                dimensions,
                transform,
            } => Ok(CatalogueCutKey::Box {
                dimension_bits: [
                    finite_bits(dimensions.x, "catalogue NSBO dimensions")?,
                    finite_bits(dimensions.y, "catalogue NSBO dimensions")?,
                    finite_bits(dimensions.z, "catalogue NSBO dimensions")?,
                ],
                transform_bits: matrix_bits(*transform),
            }),
            EvaluatedCatalogueCut::Pyramid {
                parameters,
                transform,
            } => Ok(CatalogueCutKey::Pyramid {
                parameter_bits: parameters.map(|value| value.to_bits()),
                transform_bits: matrix_bits(*transform),
            }),
            EvaluatedCatalogueCut::Extrusion {
                rings,
                height,
                transform,
            } => Ok(CatalogueCutKey::Extrusion {
                profile: sext_mesh_key(rings, *height)?,
                transform_bits: matrix_bits(*transform),
            }),
            EvaluatedCatalogueCut::Snout {
                parameters,
                transform,
            } => Ok(CatalogueCutKey::Snout {
                parameter_bits: parameters.map(|value| value.to_bits()),
                transform_bits: matrix_bits(*transform),
            }),
            EvaluatedCatalogueCut::Torus {
                kind,
                inner,
                outer,
                angle_deg,
                transform,
            } => Ok(match kind {
                EvaluatedTorusKind::Circular => CatalogueCutKey::CircularTorus {
                    inner_bits: finite_bits(*inner, "catalogue NSCT inner radius")?,
                    outer_bits: finite_bits(*outer, "catalogue NSCT outer radius")?,
                    angle_bits: finite_bits(*angle_deg, "catalogue NSCT angle")?,
                    transform_bits: matrix_bits(*transform),
                },
                EvaluatedTorusKind::Rectangular { height } => CatalogueCutKey::RectangularTorus {
                    inner_bits: finite_bits(*inner, "catalogue NSRT inner radius")?,
                    outer_bits: finite_bits(*outer, "catalogue NSRT outer radius")?,
                    height_bits: finite_bits(*height, "catalogue NSRT height")?,
                    angle_bits: finite_bits(*angle_deg, "catalogue NSRT angle")?,
                    transform_bits: matrix_bits(*transform),
                },
            }),
            EvaluatedCatalogueCut::Revolution {
                rings,
                angle_deg,
                transform,
            } => {
                let (ring_lengths, coordinate_bits) =
                    profile_key_bits(rings, "catalogue NSRE profile")?;
                Ok(CatalogueCutKey::Revolution {
                    angle_bits: finite_bits(*angle_deg, "catalogue NSRE angle")?,
                    ring_lengths,
                    coordinate_bits,
                    transform_bits: matrix_bits(*transform),
                })
            }
            EvaluatedCatalogueCut::Sphere {
                segments,
                transform,
            } => Ok(CatalogueCutKey::Sphere {
                segments: *segments,
                transform_bits: matrix_bits(*transform),
            }),
            EvaluatedCatalogueCut::Dish {
                body,
                radius,
                height,
                transform,
            } => Ok(CatalogueCutKey::Dish {
                knuckled: matches!(body, EvaluatedDishBody::Knuckled),
                radius_bits: finite_bits(*radius, "catalogue NSDS radius")?,
                height_bits: finite_bits(*height, "catalogue NSDS height")?,
                transform_bits: matrix_bits(*transform),
            }),
            EvaluatedCatalogueCut::SlopeCylinder {
                radius,
                height,
                shear_deg,
                transform,
            } => Ok(CatalogueCutKey::SlopeCylinder {
                parameter_bits: [
                    *radius,
                    *height,
                    shear_deg[0],
                    shear_deg[1],
                    shear_deg[2],
                    shear_deg[3],
                ]
                .map(f64::to_bits),
                transform_bits: matrix_bits(*transform),
            }),
        })
        .collect()
}

/// 一把刀的局部实体与放置。SBOX 与 SREV 的扣减共用这一份建刀路。
fn cutter_solid(cutter: &EvaluatedCatalogueCut) -> Result<(Manifold, DMat4), ModelError> {
    Ok(match cutter {
        EvaluatedCatalogueCut::Cylinder(cutter) => (
            csg_cylinder_solid(cutter.diameter, cutter.height)?,
            cutter.transform,
        ),
        EvaluatedCatalogueCut::Box {
            dimensions,
            transform,
        } => (
            box_solid(dimensions.x, dimensions.y, dimensions.z)?,
            *transform,
        ),
        EvaluatedCatalogueCut::Pyramid {
            parameters,
            transform,
        } => {
            let [xbot, ybot, xtop, ytop, height, xoff, yoff] = *parameters;
            (
                pyramid_solid(xbot, ybot, xtop, ytop, height, xoff, yoff)?,
                *transform,
            )
        }
        // NSEX 与正体 SEXT 同一条建体路（extrude_rings），无过切。
        EvaluatedCatalogueCut::Extrusion {
            rings,
            height,
            transform,
        } => (extrude_rings(rings.clone(), *height)?, *transform),
        EvaluatedCatalogueCut::Snout {
            parameters,
            transform,
        } => {
            let [dbot, dtop, height, xoff, yoff] = *parameters;
            (snout_solid(dbot, dtop, height, xoff, yoff)?, *transform)
        }
        EvaluatedCatalogueCut::Torus {
            kind,
            inner,
            outer,
            angle_deg,
            transform,
        } => (
            match kind {
                EvaluatedTorusKind::Circular => circ_torus_solid(*inner, *outer, *angle_deg)?,
                EvaluatedTorusKind::Rectangular { height } => {
                    rect_torus_solid(*inner, *outer, *height, *angle_deg)?
                }
            },
            *transform,
        ),
        // NSRE 与正体 SREV 同走目录 FacetGroup 的 10 mm 分片网格，无过切（§2.3）。
        EvaluatedCatalogueCut::Revolution {
            rings,
            angle_deg,
            transform,
        } => (
            revolve_rings_profile_faceted(rings.clone(), *angle_deg)?,
            *transform,
        ),
        EvaluatedCatalogueCut::Sphere {
            segments,
            transform,
        } => (unit_sphere_solid(*segments)?, *transform),
        EvaluatedCatalogueCut::Dish {
            body,
            radius,
            height,
            transform,
        } => (
            match body {
                EvaluatedDishBody::Spherical => spherical_dish_solid(*radius, *height)?,
                EvaluatedDishBody::Knuckled => {
                    elliptical_dish_solid(*radius, *height, dish_knuckle_radius(*radius, *height))?
                }
            },
            *transform,
        ),
        EvaluatedCatalogueCut::SlopeCylinder {
            radius,
            height,
            shear_deg,
            transform,
        } => (
            slope_ended_cylinder_solid(
                *radius,
                *height,
                shear_deg[0],
                shear_deg[1],
                shear_deg[2],
                shear_deg[3],
            )?,
            *transform,
        ),
    })
}

fn build_sbox(evaluated: &EvaluatedSbox) -> Result<Manifold, ModelError> {
    let mut body = box_solid(
        evaluated.dimensions.x,
        evaluated.dimensions.y,
        evaluated.dimensions.z,
    )?;
    for cutter in &evaluated.cutters {
        let (local, transform) = cutter_solid(cutter)?;
        body = &body - &placed(&local, transform);
    }
    if body.is_empty() || body.num_tri() == 0 {
        return Err(ModelError::Solid(SolidError::EmptyManifold {
            what: "catalogue SBOX after negatives",
        }));
    }
    Ok(body)
}

/// SEXT 局部网格：轮廓沿局部 Z 挤出（与 NSEX 刀同一条 `extrude_rings`），再扣掉
/// 名下的成员级刀集。无刀时与旧路逐位相同——刀集为空的键仍是 `Sext`。
fn build_sext(evaluated: &EvaluatedSext) -> Result<Manifold, ModelError> {
    let mut body = extrude_rings(evaluated.rings.clone(), evaluated.height)?;
    for cutter in &evaluated.cutters {
        let (local, transform) = cutter_solid(cutter)?;
        body = &body - &placed(&local, transform);
    }
    if body.is_empty() || body.num_tri() == 0 {
        return Err(ModelError::Solid(SolidError::EmptyManifold {
            what: "catalogue SEXT after negatives",
        }));
    }
    Ok(body)
}

/// SREV 局部网格：轮廓绕局部 X 回转，再扣掉名下的成员级刀集。
fn build_srev(evaluated: &EvaluatedSrev) -> Result<Manifold, ModelError> {
    let mut body = revolve_rings_profile_faceted(evaluated.rings.clone(), evaluated.angle_deg)?;
    for cutter in &evaluated.cutters {
        let (local, transform) = cutter_solid(cutter)?;
        body = &body - &placed(&local, transform);
    }
    if body.is_empty() || body.num_tri() == 0 {
        return Err(ModelError::Solid(SolidError::EmptyManifold {
            what: "catalogue SREV after negatives",
        }));
    }
    Ok(body)
}

fn profile_key_bits(
    rings: &[Vec<[f64; 2]>],
    what: &'static str,
) -> Result<(Vec<usize>, Vec<u64>), ModelError> {
    let mut ring_lengths = Vec::with_capacity(rings.len());
    let mut coordinate_bits = Vec::new();
    for ring in rings {
        ring_lengths.push(ring.len());
        for point in ring {
            coordinate_bits.push(finite_bits(point[0], what)?);
            coordinate_bits.push(finite_bits(point[1], what)?);
        }
    }
    Ok((ring_lengths, coordinate_bits))
}

fn sext_mesh_key(rings: &[Vec<[f64; 2]>], height: f64) -> Result<SextMeshKey, ModelError> {
    let (ring_lengths, coordinate_bits) = profile_key_bits(rings, "catalogue SEXT profile")?;
    Ok(SextMeshKey {
        tessellation_version: SEXT_TESSELLATION_VERSION,
        height_bits: finite_bits(height, "catalogue SEXT height")?,
        ring_lengths,
        coordinate_bits,
    })
}

fn matrix_bits(matrix: DMat4) -> [u64; 16] {
    matrix.to_cols_array().map(canonical_bits)
}

fn finite_bits(value: f64, what: &'static str) -> Result<u64, ModelError> {
    if !value.is_finite() {
        return Err(ModelError::Solid(SolidError::Degenerate {
            what,
            detail: format!("non-finite cache coordinate {value}"),
        }));
    }
    Ok(canonical_bits(value))
}

pub(crate) fn canonical_bits(value: f64) -> u64 {
    if value == 0.0 {
        0.0f64.to_bits()
    } else {
        value.to_bits()
    }
}

pub(crate) fn required_number(
    element: &DbElement,
    name: &'static str,
    env: &DbElementParamEnv,
    what: &'static str,
) -> Result<f64, ModelError> {
    let outcome = element.get_number(name, env)?;
    // Some catalogue SVER coordinates are stored as an explicit all-zero
    // tuple but the parameter evaluator reports that tuple as unresolved.
    // The tuple is the value in that case, just as it is for NotNumeric.
    if matches!(
        outcome,
        AttributeNumber::NotNumeric | AttributeNumber::Unresolved { .. }
    ) && raw_zero_tuple(element, name)
    {
        return Ok(0.0);
    }
    outcome
        .value()
        .filter(|value| value.is_finite())
        .ok_or_else(|| ModelError::UnresolvedAttribute {
            refno: element.refno().to_string(),
            noun: element.element_type().unwrap_or_else(|_| "?".into()),
            name,
            what,
            detail: format!("{outcome:?}; stored={:?}", element.attribute(name)),
        })
}

pub(crate) fn optional_number(
    element: &DbElement,
    name: &str,
    env: &DbElementParamEnv,
) -> Option<f64> {
    let outcome = element.get_number(name, env).ok()?;
    outcome
        .value()
        .filter(|value| value.is_finite())
        .or_else(|| {
            (matches!(outcome, AttributeNumber::NotNumeric) && raw_zero_tuple(element, name))
                .then_some(0.0)
        })
}

/// Catalogue coordinate/radius slots use an all-zero tuple for an explicit
/// numeric zero when no negated-hash PML body exists. `DbElement::get_number`
/// has already checked that explicit stream before returning `NotNumeric`, so
/// this narrow geometry seam does not mask a stored formula.
fn raw_zero_tuple(element: &DbElement, name: &str) -> bool {
    matches!(
        element.attribute(name),
        Ok(Some(DescriptorValue::RawWords(words)))
            if !words.is_empty() && words.iter().all(|word| *word == 0)
    )
}

fn required_words(element: &DbElement, name: &'static str) -> Result<Vec<u32>, ModelError> {
    let words = match element.attribute(name)? {
        Some(DescriptorValue::RawWords(words) | DescriptorValue::WordArray(words)) => Some(words),
        Some(DescriptorValue::IntArray(words)) => {
            Some(words.into_iter().map(|word| word as u32).collect())
        }
        _ => None,
    };
    words.ok_or_else(|| missing(element, name, "catalogue axis"))
}

fn missing(element: &DbElement, name: &'static str, what: &'static str) -> ModelError {
    ModelError::MissingAttribute {
        refno: element.refno().to_string(),
        noun: element.element_type().unwrap_or_else(|_| "?".into()),
        name,
        what,
    }
}

/// Resolve a primitive's axis attribute (`PAXI` / `PAAX` / `PBAX` / `PCAX`) or
/// fail the primitive Core-style with the cause spelled out.
///
/// An absent attribute is still [`missing`]. An attribute that is there but
/// does not resolve used to be reported the same way, which lumped together
/// "`Pn` is not in the PTSE" (E3D draws nothing either; ams7997 has 124 such
/// LCYL/SCTO in the HJXSWA valves) with "the point exists but its PTCD/PX did
/// not evaluate" (our decoder's gap). The account needs the two apart.
fn catalogue_axis_or_fail(
    member: &DbElement,
    component: &DbElement,
    primitive: &DbElement,
    name: &'static str,
    what: &'static str,
) -> Result<CatalogueAxis, ModelError> {
    resolve_catalogue_axis(member, component, &required_words(primitive, name)?)?.map_err(
        |failure| ModelError::UnresolvedAttribute {
            refno: primitive.refno().to_string(),
            noun: primitive.element_type().unwrap_or_else(|_| "?".into()),
            name,
            what,
            detail: failure.to_string(),
        },
    )
}

#[cfg(test)]
mod tests {
    #[test]
    fn gril_negative_lpyr_stays_between_native_axial_endpoints() {
        // /RGRILOFF-BODY 15194/3412 as used by GRIL 24381/57342.
        let axis = |direction| CatalogueAxis {
            origin: DVec3::ZERO,
            direction,
        };
        let geometry = lpyr_geometry(
            [2.5, 2.5, 72.5, 72.5],
            [-5.0, -2.0, 0.0, 0.0],
            axis(DVec3::X),
            axis(-DVec3::Z),
            axis(DVec3::Y),
        )
        .unwrap()
        .unwrap();
        let [xb, yb, xt, yt, h, xo, yo] = geometry.parameters;
        let body = placed(
            &pyramid_solid(xb, yb, xt, yt, h, xo, yo).unwrap(),
            geometry.transform,
        );
        let bounds = body.bounding_box().unwrap();
        assert!((bounds.min()[0] + 5.0).abs() < 1e-9, "{:?}", bounds);
        assert!((bounds.max()[0] + 2.0).abs() < 1e-9, "{:?}", bounds);
        assert!((geometry.transform.z_axis.truncate() - DVec3::X).length() < 1e-12);
        assert!((geometry.transform.x_axis.truncate() - DVec3::Z).length() < 1e-12);
    }

    /// SLYK `/Z9B43H-16C-FBP-GM` 的手轮座 `LPYR 16596/10594`：`PAAX`（`P11` 取反）
    /// 与 `PBAX` 解出来是同一根 +X，`z × x` 退化。Core 的 `ORTHOG` 退到 `z × (0,1,0)`，
    /// 标架成 x=+Y、y=+Z、z=+X——E3D 那条 `Pyramid[bottom 315×198.125, h 263.1]` 的
    /// 世界盒正是 263.1(X) × 315(Y) × 198.125(Z)。此前我们判轴退化、整只丢掉。
    #[test]
    fn parallel_lpyr_axes_go_through_cores_orthog_instead_of_failing() {
        let axis = |direction| CatalogueAxis {
            origin: DVec3::ZERO,
            direction,
        };
        let geometry = lpyr_geometry(
            [315.0, 198.125, 315.0, 198.125],
            [131.55, -131.55, 0.0, 0.0],
            axis(DVec3::X),
            axis(DVec3::X),
            axis(DVec3::Y),
        )
        .unwrap()
        .unwrap();
        let [xb, yb, xt, yt, h, xo, yo] = geometry.parameters;
        assert!((h - 263.1).abs() < 1e-9, "height {h}");
        let body = placed(
            &pyramid_solid(xb, yb, xt, yt, h, xo, yo).unwrap(),
            geometry.transform,
        );
        let bounds = body.bounding_box().unwrap();
        let span = |axis: usize| bounds.max()[axis] - bounds.min()[axis];
        assert!((span(0) - 263.1).abs() < 1e-6, "{bounds:?}");
        assert!((span(1) - 315.0).abs() < 1e-6, "{bounds:?}");
        assert!((span(2) - 198.125).abs() < 1e-6, "{bounds:?}");
    }

    /// 三根轴都解不出来时 Core 也不失败：z 换 `(1,0,0)`、x 换 `(0,1,0)`。
    #[test]
    fn orthog_substitutes_cores_fallback_axes() {
        assert_eq!(
            orthog(DVec3::ZERO, DVec3::ZERO),
            (DVec3::Y, DVec3::Z, DVec3::X)
        );
        // x ∥ z：y 退到 z × (0,1,0) = −X，x 再由 y × z 拉成 +Y。
        assert_eq!(orthog(DVec3::Z, DVec3::Z), (DVec3::Y, -DVec3::X, DVec3::Z));
    }

    #[test]
    fn lpyr_uses_ptdi_pbdi_for_axis_and_pbof_pcof_for_offsets() {
        assert_eq!(
            lpyr_axial_and_offsets(50.0, 0.0, 7.0, -3.0),
            (50.0, 0.0, 7.0, -3.0)
        );
    }

    use std::collections::BTreeSet;

    use super::*;

    /// /-CPV-S-2-L-1301, STRT 24381/76730: /CSTRA-BODY has
    /// PAXI=Z, CLFL=true, DESP[2]=1600 and DESP[6]=1220.
    #[test]
    fn cpv_strt_body_and_flanges_stay_on_the_catalogue_axis() {
        let axis = CatalogueAxis {
            origin: DVec3::ZERO,
            direction: DVec3::Z,
        };
        for (diameter, height, distance) in [
            (1600.0, 1220.0, 0.0),
            (1700.0, 5.0, 0.0),
            (1610.0, 50.0, 0.0),
            (1700.0, 5.0, 1215.0),
            (1610.0, 50.0, 1170.0),
        ] {
            let (rotation, centre) = catalogue_cylinder_placement(
                "STRT",
                "SCYL",
                true,
                axis,
                distance + height / 2.0,
                height,
            );
            let body = placed(
                &csg_cylinder_solid(diameter, height).unwrap(),
                DMat4::from_translation(centre) * DMat4::from_mat3(rotation),
            );
            let bounds = body.bounding_box().unwrap();
            assert!(
                (rotation.z_axis - DVec3::Z).length() < 1e-12,
                "STRT cylinder axis must follow PAXI, got {:?}",
                rotation.z_axis
            );
            assert!((bounds.min()[2] - distance).abs() < 1e-9);
            assert!((bounds.max()[2] - distance - height).abs() < 1e-9);
        }
    }

    #[test]
    fn atta_clfl_cylinder_keeps_the_catalogue_axis() {
        assert!(!uses_clfl_centreline_frame("ATTA", "SCYL", true));
        assert!(uses_clfl_centreline_frame("HELE", "SCYL", true));
        assert!(uses_clfl_centreline_frame("PCLA", "SCYL", true));
        assert!(!uses_clfl_centreline_frame("HELE", "LCYL", true));
        assert!(!uses_clfl_centreline_frame("HELE", "SCYL", false));
    }

    fn test_cut_cylinder(cutters: Vec<EvaluatedCatalogueCut>) -> EvaluatedCylinder {
        let base = CatalogueMeshKey::Cylinder {
            tessellation_version: CYLINDER_TESSELLATION_VERSION,
            diameter_bits: 60.3_f64.to_bits(),
            height_bits: 250.0_f64.to_bits(),
        };
        let mesh_key = if cutters.is_empty() {
            base
        } else {
            CatalogueMeshKey::Cut {
                base: Box::new(base),
                cutters: catalogue_cut_keys(&cutters).unwrap(),
            }
        };
        EvaluatedCylinder {
            mesh_key,
            diameter: 60.3,
            height: 250.0,
            transform: DMat4::IDENTITY,
            cutters,
            diagnostics: vec![],
        }
    }

    fn test_bore(diameter: f64, transform: DMat4) -> EvaluatedCatalogueCut {
        EvaluatedCatalogueCut::Cylinder(EvaluatedNscy {
            diameter,
            height: 252.0,
            transform,
        })
    }

    #[test]
    fn catalogue_cylinder_without_cuts_retains_primitive_mesh() {
        let e = test_cut_cylinder(vec![]);
        assert_eq!(
            build_catalogue_cylinder(&e).unwrap().volume().to_bits(),
            csg_cylinder_solid(60.3, 250.0).unwrap().volume().to_bits()
        );
    }

    #[test]
    fn catalogue_cylinder_child_bore_makes_an_octagonal_annulus() {
        let e = test_cut_cylinder(vec![test_bore(40.3, DMat4::IDENTITY)]);
        let body = build_catalogue_cylinder(&e).unwrap();
        let expected =
            4.0 * std::f64::consts::FRAC_1_SQRT_2 * (30.15_f64.powi(2) - 20.15_f64.powi(2)) * 250.0;
        assert!((body.volume() - expected).abs() < 1e-6);
        assert_eq!(body.genus(), 1);
    }

    #[test]
    fn catalogue_cylinder_cut_key_includes_bore_and_placement() {
        let a = test_cut_cylinder(vec![test_bore(40.3, DMat4::IDENTITY)]);
        let b = test_cut_cylinder(vec![test_bore(30.3, DMat4::IDENTITY)]);
        let c = test_cut_cylinder(vec![test_bore(40.3, DMat4::from_translation(DVec3::X))]);
        assert_ne!(a.mesh_key, b.mesh_key);
        assert_ne!(a.mesh_key, c.mesh_key);
        assert_ne!(a.mesh_key, test_cut_cylinder(vec![]).mesh_key);
    }

    #[test]
    fn catalogue_cylinder_transverse_cut_removes_the_pipe_saddle() {
        let bore = || test_bore(40.3, DMat4::IDENTITY);
        let plain = build_catalogue_cylinder(&test_cut_cylinder(vec![bore()])).unwrap();
        let transverse = test_bore(
            60.0,
            DMat4::from_translation(DVec3::new(0.0, 0.0, 125.0))
                * DMat4::from_rotation_y(std::f64::consts::FRAC_PI_2),
        );
        let cut = build_catalogue_cylinder(&test_cut_cylinder(vec![bore(), transverse])).unwrap();
        assert!(cut.volume() < plain.volume() - 1000.0);
        assert!(!cut.is_empty());
    }

    #[test]
    fn atta_negative_z_must_not_get_an_extra_roll() {
        // Actual screenshot members, including their rotation and DESP[1].
        for (ori, height, end_z) in [
            ([0.0, 0.0, -150.0], 250.0, -250.0),
            ([180.0, 0.0, -150.0], 150.0, 150.0),
        ] {
            let axis = CatalogueAxis {
                origin: DVec3::ZERO,
                direction: -DVec3::Z,
            };
            let (rotation, centre) =
                catalogue_cylinder_placement("ATTA", "SCYL", true, axis, height / 2.0, height);
            let world = crate::transform::ori_matrix(ori);
            assert!(
                (world * (centre - rotation.z_axis * height / 2.0)).abs_diff_eq(DVec3::ZERO, 1e-9)
            );
            assert!(
                (world * (centre + rotation.z_axis * height / 2.0))
                    .abs_diff_eq(DVec3::new(0.0, 0.0, end_z), 1e-9)
            );
        }
    }

    #[test]
    fn atta_catalogue_axis_and_offset_do_not_depend_on_member_euler_sign() {
        for direction in [DVec3::X, -DVec3::Y, -DVec3::Z] {
            let axis = CatalogueAxis {
                origin: DVec3::new(3.0, 4.0, 5.0),
                direction,
            };
            let (rotation, centre) =
                catalogue_cylinder_placement("ATTA", "SCYL", true, axis, 17.0 + 50.0, 100.0);
            assert!(rotation.z_axis.abs_diff_eq(direction, 1e-9));
            assert!(centre.abs_diff_eq(axis.origin + direction * 67.0, 1e-9));
        }
    }

    #[test]
    fn visible_catalogue_programs_keep_independent_components() {
        // AMS /A0Ph33VNN0FF is a VALV with overlapping SCYL/LSNO parts.
        // RVM keeps every part independently; composing them changes both
        // occupied ray intervals and the summed display volume.
        assert!(preserves_catalogue_components(true));
        assert!(!preserves_catalogue_components(false));
    }

    /// 跨度为负时底顶对调、PAAX 不动、偏移**不取反**；偏移的方向改由调用方把局部 X 换成
    /// −PBAX 来表达（E3D 的 DRSNO 写 `X = −PBAX, XOFF = POFF`）。两种写法是同一个实体：
    /// 下面两组 `lsnout_frame` 用翻过的 PBAX + 正偏移算出的中心，与此前「PBAX + 负偏移」一样。
    #[test]
    fn negative_lsnout_span_reparameterizes_ends_without_reversing_paax() {
        assert_eq!(
            normalize_lsnout_span(84.42, 66.33, 115.2, 0.0, 12.06),
            (66.33, 84.42, 115.2, 12.06, true)
        );
        assert_eq!(
            normalize_lsnout_span(66.33, 84.42, 0.0, 115.2, 12.06),
            (66.33, 84.42, 115.2, 12.06, false)
        );

        let (centre, rotation) = lsnout_frame(
            crate::catalogue_point::CatalogueAxis {
                origin: DVec3::new(-144.0, 0.0, 0.0),
                direction: DVec3::X,
            },
            crate::catalogue_point::CatalogueAxis {
                origin: DVec3::ZERO,
                direction: -DVec3::Z, // PBAX = +Z，底顶对调后翻成 −Z
            },
            129.6,
            14.4,
            12.06,
        )
        .unwrap();
        assert!(centre.abs_diff_eq(DVec3::new(-72.0, 0.0, -6.03), 1.0e-9));
        assert!(rotation.z_axis.abs_diff_eq(DVec3::X, 1.0e-9));
        assert!(rotation.x_axis.abs_diff_eq(-DVec3::Z, 1.0e-9));

        let (centre, rotation) = lsnout_frame(
            crate::catalogue_point::CatalogueAxis {
                origin: DVec3::new(144.0, 0.0, 0.0),
                direction: -DVec3::X,
            },
            crate::catalogue_point::CatalogueAxis {
                origin: DVec3::ZERO,
                direction: DVec3::Z, // PBAX = −Z，翻成 +Z
            },
            129.6,
            14.4,
            12.06,
        )
        .unwrap();
        assert!(centre.abs_diff_eq(DVec3::new(72.0, 0.0, 6.03), 1.0e-9));
        assert!(rotation.z_axis.abs_diff_eq(-DVec3::X, 1.0e-9));
    }

    /// `/OILSTD-COUPLING-QUICKFIT-SWF/GMSE_001` 的 SCON：`PAXI P2 PDIS 42 PDIA 66`，P2 在
    /// 构件 +X 端、轴朝构件里（−X）。E3D 写 `Snout[rb=0, rt=33, h=42]`、局部 +Z = −X：
    /// 锥顶压在 P2 上，底圆在 42 mm 深处。
    #[test]
    fn scone_puts_the_apex_on_the_paxi_origin_and_the_base_at_pdis() {
        let axis = crate::catalogue_point::CatalogueAxis {
            origin: DVec3::new(70.0, 0.0, 0.0),
            direction: -DVec3::X,
        };
        let (dbot, dtop, height, centre, rotation) = scone_frame(66.0, 42.0, axis).unwrap();
        assert_eq!((dbot, dtop, height), (0.0, 66.0, 42.0));
        assert!(rotation.z_axis.abs_diff_eq(-DVec3::X, 1.0e-9));
        assert!(centre.abs_diff_eq(DVec3::new(49.0, 0.0, 0.0), 1.0e-9));
        // 局部 −Z 端（底、半径 0）落回轴原点，+Z 端（顶、底圆）在 PDIS 处。
        let apex = centre - rotation.z_axis * (height * 0.5);
        let base = centre + rotation.z_axis * (height * 0.5);
        assert!(apex.abs_diff_eq(axis.origin, 1.0e-9));
        assert!(base.abs_diff_eq(DVec3::new(28.0, 0.0, 0.0), 1.0e-9));

        // `PDIS < 0` 走 HCHECK：轴反向、高度取正，锥顶照样在原点，底圆挪到另一侧。
        let (dbot, dtop, height, centre, rotation) = scone_frame(66.0, -42.0, axis).unwrap();
        assert_eq!((dbot, dtop, height), (0.0, 66.0, 42.0));
        assert!(rotation.z_axis.abs_diff_eq(DVec3::X, 1.0e-9));
        assert!((centre - rotation.z_axis * (height * 0.5)).abs_diff_eq(axis.origin, 1.0e-9));
        assert!(
            (centre + rotation.z_axis * (height * 0.5))
                .abs_diff_eq(DVec3::new(112.0, 0.0, 0.0), 1.0e-9)
        );
    }

    #[test]
    fn rectangular_hvac_specs_extract_one_canonical_box() {
        let mut straight = vec![0.0; 90];
        straight[1] = 580.0;
        straight[2] = 580.0;
        straight[5] = 1150.0;
        assert_eq!(
            hvac_rectangular_box("/CADCHVACSPEC/RSTRA", &straight),
            Some((
                DVec3::new(580.0, 580.0, 1150.0),
                DVec3::new(0.0, 0.0, 575.0)
            ))
        );

        let mut branch = vec![0.0; 90];
        branch[6] = 400.0;
        branch[40] = 500.0;
        branch[41] = 500.0;
        assert_eq!(
            hvac_rectangular_box("/CADCHVACSPEC/RSBRCO", &branch),
            Some((
                DVec3::new(400.0, 500.0, 500.0),
                DVec3::new(-200.0, 0.0, 0.0)
            ))
        );
    }

    #[test]
    fn rectangular_hvac_fast_path_rejects_other_or_degenerate_specs() {
        let mut desp = vec![0.0; 90];
        desp[1] = 580.0;
        desp[2] = 580.0;
        assert_eq!(hvac_rectangular_box("/CADCHVACSPEC/RROUND", &desp), None);
        assert_eq!(hvac_rectangular_box("/CADCHVACSPEC/RSTRA", &desp), None);
    }

    #[test]
    fn cache_report_can_measure_one_root_without_double_counting_prior_roots() {
        fn all(value: usize) -> CatalogueMeshCacheStats {
            CatalogueMeshCacheStats {
                source_sext: value,
                evaluated_sext: value,
                source_sbox: value,
                evaluated_sbox: value,
                source_nscy: value,
                evaluated_nscy: value,
                source_nsbo: value,
                evaluated_nsbo: value,
                source_nlcy: value,
                evaluated_nlcy: value,
                source_nsex: value,
                evaluated_nsex: value,
                source_nssl: value,
                evaluated_nssl: value,
                source_nsds: value,
                evaluated_nsds: value,
                source_nssp: value,
                evaluated_nssp: value,
                source_srev: value,
                evaluated_srev: value,
                source_scyl: value,
                evaluated_scyl: value,
                source_lcyl: value,
                evaluated_lcyl: value,
                source_lpyr: value,
                evaluated_lpyr: value,
                source_scto: value,
                evaluated_scto: value,
                source_srto: value,
                evaluated_srto: value,
                source_ssph: value,
                evaluated_ssph: value,
                source_sslc: value,
                evaluated_sslc: value,
                source_sdsh: value,
                evaluated_sdsh: value,
                source_sdis: value,
                evaluated_sdis: value,
                display_filtered_negatives: value,
                local_mesh_hits: value,
                local_mesh_misses: value,
                unique_placed_parts: value,
                duplicate_placed_parts: value,
            }
        }

        let before = all(7);
        let after = all(12);
        let delta = after.checked_delta_from(before);
        assert_eq!(delta, all(5));

        let mut merged = all(1);
        merged.accumulate(delta);
        assert_eq!(merged, all(6));
    }

    #[test]
    fn orthogonal_completion_keeps_first_axis_and_right_handedness() {
        let x = DVec3::X;
        let supplied_y = DVec3::new(0.1, 1.0, 0.0).normalize();
        let z = x.cross(supplied_y).normalize();
        let y = z.cross(x).normalize();
        assert_eq!(x, DVec3::X);
        assert!((x.dot(y)).abs() < 1.0e-12);
        assert!(x.cross(y).dot(z) > 1.0 - 1.0e-12);
    }

    /// CSRTAX 与 ORIMAT 竖直轴同解、水平轴分道：SSLC 的四个斜切角认的是 CSRTAX 那套 X/Y。
    #[test]
    fn csrtax_puts_the_axis_in_z_and_breaks_ties_against_world_z() {
        let up = csrtax(DVec3::Z).unwrap();
        assert_eq!(up, DMat3::IDENTITY);
        let down = csrtax(DVec3::NEG_Z).unwrap();
        assert_eq!(down.x_axis, -DVec3::X);
        assert_eq!(down.y_axis, DVec3::Y);
        assert_eq!(down.z_axis, -DVec3::Z);

        // 水平 +X 轴：CSRTAX 的 X = Z × Ẑ = −Ŷ、Y = Z × X = −Ẑ；ORIMAT 给的是 X = Ẑ、Y = −Ŷ。
        let east = csrtax(DVec3::X * 3.0).unwrap();
        assert!((east.x_axis - (-DVec3::Y)).length() < 1e-12);
        assert!((east.y_axis - (-DVec3::Z)).length() < 1e-12);
        assert!((east.z_axis - DVec3::X).length() < 1e-12);
        let east_orimat = orimat(DVec3::X).unwrap();
        assert!((east_orimat.x_axis - DVec3::Z).length() < 1e-12);
        assert!((east_orimat.y_axis - (-DVec3::Y)).length() < 1e-12);

        for dir in [
            DVec3::new(1.0, 2.0, 3.0),
            DVec3::new(-0.3, 0.9, -0.1),
            DVec3::new(0.0, 1.0, 0.0),
            DVec3::new(1.0e-7, 0.0, 1.0),
        ] {
            let frame = csrtax(dir).unwrap();
            assert!((frame.z_axis - dir.normalize()).length() < 1e-12);
            assert!(frame.x_axis.dot(frame.y_axis).abs() < 1e-12);
            assert!(frame.x_axis.cross(frame.y_axis).dot(frame.z_axis) > 1.0 - 1e-9);
        }
        assert!(csrtax(DVec3::ZERO).is_none());
    }

    #[test]
    fn sslc_shear_angles_fold_once_into_the_signed_ninety_degree_band() {
        assert_eq!(fold_shear_angle(135.0), -45.0);
        assert_eq!(fold_shear_angle(-135.0), 45.0);
        assert_eq!(fold_shear_angle(45.0), 45.0);
        assert_eq!(fold_shear_angle(-89.9), -89.9);
        // 边界照 CRCATI 原样：恰好 ±90 不折，折一次仍 ≥ 90 的也不再折——两者都由
        // `|角| ≥ 90 → 无形体` 那一道收走。
        assert_eq!(fold_shear_angle(90.0), 90.0);
        assert_eq!(fold_shear_angle(270.0), 90.0);
    }

    /// SDSH 的开关语义：`PRAD` 只决定走折边碟还是球形碟，折边半径本身来自 r/h。
    #[test]
    fn dish_bodies_key_on_the_switch_not_on_the_requested_knuckle() {
        let knuckled = EvaluatedDish {
            mesh_key: CatalogueMeshKey::Dish {
                tessellation_version: DISH_TESSELLATION_VERSION,
                knuckled: true,
                radius_bits: 0,
                height_bits: 0,
            },
            body: EvaluatedDishBody::Knuckled,
            radius: 500.0,
            height: 200.0,
            transform: DMat4::IDENTITY,
        };
        let spherical = EvaluatedDish {
            body: EvaluatedDishBody::Spherical,
            mesh_key: CatalogueMeshKey::Dish {
                tessellation_version: DISH_TESSELLATION_VERSION,
                knuckled: false,
                radius_bits: 0,
                height_bits: 0,
            },
            ..knuckled
        };
        let a = build_dish(&knuckled).unwrap();
        let b = build_dish(&spherical).unwrap();
        assert_ne!(knuckled.mesh_key, spherical.mesh_key);
        assert!(a.volume() > b.volume(), "折边碟比同底同高的球形碟装得更多");
        let bb = a.bounding_box().unwrap();
        assert!(
            bb.min()[2].abs() < 1e-9 && (bb.max()[2] - 200.0).abs() < 1e-9,
            "底在 z=0、顶在 z=h"
        );
    }

    #[test]
    fn catalogue_spheres_share_one_unit_mesh_per_segment_class() {
        let a = CatalogueMeshKey::Sphere {
            tessellation_version: SPHERE_TESSELLATION_VERSION,
            segments: 32,
        };
        let b = CatalogueMeshKey::Sphere {
            tessellation_version: SPHERE_TESSELLATION_VERSION,
            segments: 32,
        };
        let finer = CatalogueMeshKey::Sphere {
            tessellation_version: SPHERE_TESSELLATION_VERSION,
            segments: 36,
        };
        assert_eq!(a, b);
        assert_ne!(a, finer);
    }

    #[test]
    fn nscy_core_centre_keeps_the_axial_overcut_symmetric() {
        let axis = DVec3::X;
        let distance = -2.5;
        let height = 5.0;
        let centre = axis * (distance + height * 0.5);
        assert_eq!(centre, DVec3::ZERO);
        assert_eq!(height + NSCY_AXIAL_OVERCUT_MM, 5.01);
    }

    /// QSSREV（`0x103B81CC`）的扫角归一逐条钉死：`|PANG| < 0.001` 缺省整圈，
    /// 超 ±360 带符号钳住，其余原样。与设计侧 REV「≈360 一律当整圈」是两条
    /// 规则——目录臂对 -359.9 这类近整圈保留部分扫角，互相抄就是静默错。
    #[test]
    fn qssrev_defaults_a_silent_sweep_to_a_full_circle_and_clamps_to_signed_360() {
        assert_eq!(qssrev_angle(0.0), 360.0);
        assert_eq!(qssrev_angle(0.0005), 360.0);
        assert_eq!(qssrev_angle(-0.0005), 360.0);
        assert_eq!(qssrev_angle(400.0), 360.0);
        assert_eq!(qssrev_angle(-400.0), -360.0);
        assert_eq!(qssrev_angle(90.0), 90.0);
        assert_eq!(qssrev_angle(-90.0), -90.0);
        assert_eq!(qssrev_angle(-359.9), -359.9, "近整圈的部分扫角要保留");
    }

    fn srev_fixture(
        rings: Vec<Vec<[f64; 2]>>,
        angle_deg: f64,
        cutters: Vec<EvaluatedCatalogueCut>,
    ) -> EvaluatedSrev {
        EvaluatedSrev {
            mesh_key: CatalogueMeshKey::Srev(SrevMeshKey {
                tessellation_version: SREV_TESSELLATION_VERSION,
                angle_bits: 0,
                ring_lengths: Vec::new(),
                coordinate_bits: Vec::new(),
                cutters: Vec::new(),
            }),
            rings,
            angle_deg,
            cutters,
            transform: DMat4::IDENTITY,
            diagnostics: Vec::new(),
        }
    }

    /// 整圈目录 SREV 与同一条 profile-faceted 构造路必须逐位一致；设计侧
    /// REV/解析环面使用另一条 0.5 mm 网格，不能再作为目录 FacetGroup 的锚点。
    #[test]
    fn a_full_circle_srev_matches_the_rectangular_torus_bit_for_bit() {
        // 轮廓在 (轴向 X, 半径 Y)：半径 300..500、轴向 ±100。
        let ring = vec![
            [-100.0, 300.0],
            [100.0, 300.0],
            [100.0, 500.0],
            [-100.0, 500.0],
        ];
        let body = build_srev(&srev_fixture(vec![ring], 360.0, Vec::new())).unwrap();
        let torus = revolve_rings_profile_faceted(
            vec![vec![
                [-100.0, 300.0],
                [100.0, 300.0],
                [100.0, 500.0],
                [-100.0, 500.0],
            ]],
            360.0,
        )
        .unwrap();
        assert!(
            (body.volume() - torus.volume()).abs() / torus.volume() < 1e-12,
            "SREV {} vs 环面 {}",
            body.volume(),
            torus.volume()
        );
    }

    /// E3D RVM exports catalogue SREV as a coarse FacetGroup, not as a curved
    /// analytic primitive.  The ACP1000 60-degree bends use the 10 mm profile
    /// facet grid: radius 250 mm resolves to two sweep segments (16 triangles
    /// for this rectangular fixture).  The 0.5 mm primitive grid produces 40
    /// triangles and changes orientation-dependent world AABBs by 5--7 mm.
    #[test]
    fn catalogue_srev_uses_the_rvm_profile_facet_grid() {
        let ring = vec![[-25.0, 50.0], [25.0, 50.0], [25.0, 250.0], [-25.0, 250.0]];
        let body = build_srev(&srev_fixture(vec![ring], 60.0, Vec::new())).unwrap();
        assert_eq!(body.num_tri(), 20);
    }

    /// NSRE 刀从 SREV 上扣掉的必须恰好是它自己的回转体：两边同走
    /// `revolve_rings_profile_faceted`、同一外半径定段数，径向 400..500 的整圈刀贯穿轴向后，
    /// 剩下的体积就是两只回转体的解析差。
    #[test]
    fn an_nsre_cutter_removes_exactly_its_own_revolution() {
        let outer = vec![
            [-100.0, 300.0],
            [100.0, 300.0],
            [100.0, 500.0],
            [-100.0, 500.0],
        ];
        // 刀在轴向两头各探出 10mm（无过切，探出来自刀自己的轮廓）。
        let groove = vec![
            [-110.0, 400.0],
            [110.0, 400.0],
            [110.0, 500.0],
            [-110.0, 500.0],
        ];
        let body = build_srev(&srev_fixture(
            vec![outer.clone()],
            360.0,
            vec![EvaluatedCatalogueCut::Revolution {
                rings: vec![groove],
                angle_deg: 360.0,
                transform: DMat4::IDENTITY,
            }],
        ))
        .unwrap();
        let full = revolve_rings_profile_faceted(vec![outer], 360.0).unwrap();
        let removed = revolve_rings_profile_faceted(
            vec![vec![
                [-100.0, 400.0],
                [100.0, 400.0],
                [100.0, 500.0],
                [-100.0, 500.0],
            ]],
            360.0,
        )
        .unwrap();
        let expected = full.volume() - removed.volume();
        assert!(
            (body.volume() - expected).abs() / expected < 1e-9,
            "体积 {} 应为 {expected}",
            body.volume()
        );
        assert_eq!(body.genus(), 1, "扣掉外圈后仍是一只环");
    }

    /// 刀参数到建体的逐位映射：Snout 与 Torus 两种新刀的参数顺序一旦写错，
    /// 体积立刻对不上（其余账面全绿），所以在这里钉死。
    #[test]
    fn snout_and_torus_cutters_map_their_parameters_straight_through() {
        let (snout, _) = cutter_solid(&EvaluatedCatalogueCut::Snout {
            parameters: [100.0, 50.0, 80.0, 12.0, 0.0],
            transform: DMat4::IDENTITY,
        })
        .unwrap();
        let direct = snout_solid(100.0, 50.0, 80.0, 12.0, 0.0).unwrap();
        assert_eq!(snout.volume().to_bits(), direct.volume().to_bits());

        let (torus, _) = cutter_solid(&EvaluatedCatalogueCut::Torus {
            kind: EvaluatedTorusKind::Rectangular { height: 40.0 },
            inner: 200.0,
            outer: 260.0,
            angle_deg: 90.0,
            transform: DMat4::IDENTITY,
        })
        .unwrap();
        let direct = rect_torus_solid(200.0, 260.0, 40.0, 90.0).unwrap();
        assert_eq!(torus.volume().to_bits(), direct.volume().to_bits());
    }

    /// 两根轴平行、间距却定不出圆心的，是一只**不存在的环**，不是一次失败：
    /// 报 `Err` 会在取证账上凭空挂 50 条「丢了 50 只环」（见 [`torus_frame`] 的取证）。
    /// 同时钉住正常那只仍照解——退化判据松了会把真环也吞掉。
    #[test]
    fn a_torus_whose_axes_are_parallel_without_a_centre_is_not_a_torus() {
        use crate::catalogue_point::CatalogueAxis;
        let axis = |origin: DVec3, direction: DVec3| CatalogueAxis { origin, direction };

        // 间距为零：ATTA 上的 23045/16、23045/17 就是这一种。
        assert!(
            torus_frame(axis(DVec3::ZERO, DVec3::X), axis(DVec3::ZERO, -DVec3::X))
                .unwrap()
                .is_none()
        );
        // 间距顺着轴：PCOM 上的 21385/8231、21385/8233，delta 480 mm。
        assert!(
            torus_frame(
                axis(DVec3::ZERO, DVec3::X),
                axis(DVec3::new(480.0, 0.0, 0.0), -DVec3::X),
            )
            .unwrap()
            .is_none()
        );

        // 真环照解：两根轴差 90°，圆心半径由两个 P 点定。
        let (_, centre, radius, angle_deg) = torus_frame(
            axis(DVec3::ZERO, DVec3::X),
            axis(DVec3::new(100.0, 100.0, 0.0), DVec3::Y),
        )
        .unwrap()
        .expect("正交两轴是一只真环");
        assert!((radius - 100.0).abs() < 1.0e-9, "半径 {radius}");
        assert!((angle_deg.abs() - 90.0).abs() < 1.0e-9, "扫角 {angle_deg}");
        assert!(centre.is_finite());
    }

    /// NSEX 从 SBOX 上扣掉的必须恰好是「轮廓 × 高度」的棱柱——CRCATI 里
    /// 942751/1008005 正负同臂、几何同构、**无过切**（对比 NSCY 的 +0.01）。
    /// 轮廓贯穿箱体时，体积差就是精确的解析值。
    #[test]
    fn an_nsex_cutter_removes_exactly_the_profile_prism() {
        // 40×40 方轮廓、贯穿 Z 向 30mm 高的 100×100 箱体。
        let ring = vec![[-20.0, -20.0], [20.0, -20.0], [20.0, 20.0], [-20.0, 20.0]];
        let evaluated = EvaluatedSbox {
            mesh_key: CatalogueMeshKey::Sbox(SboxMeshKey {
                tessellation_version: SBOX_TESSELLATION_VERSION,
                dimension_bits: [0; 3],
                cutters: Vec::new(),
            }),
            dimensions: DVec3::new(100.0, 100.0, 30.0),
            cutters: vec![EvaluatedCatalogueCut::Extrusion {
                rings: vec![ring],
                // 贯穿并两侧探出：没有过切，探出必须来自轮廓自己的高度。
                height: 40.0,
                transform: DMat4::from_translation(DVec3::new(0.0, 0.0, -20.0)),
            }],
            transform: DMat4::IDENTITY,
            diagnostics: Vec::new(),
        };
        let body = build_sbox(&evaluated).expect("sbox with NSEX cutter");
        let expected = 100.0 * 100.0 * 30.0 - 40.0 * 40.0 * 30.0;
        assert!(
            (body.volume() - expected).abs() < 1e-6,
            "体积 {} 应为 {expected}",
            body.volume()
        );
        assert_eq!(body.genus(), 1, "贯穿孔应当带来一个环柄");
    }

    fn sext_fixture(
        rings: Vec<Vec<[f64; 2]>>,
        height: f64,
        cutters: Vec<EvaluatedCatalogueCut>,
    ) -> EvaluatedSext {
        EvaluatedSext {
            mesh_key: CatalogueMeshKey::Sext(SextMeshKey {
                tessellation_version: SEXT_TESSELLATION_VERSION,
                height_bits: 0,
                ring_lengths: Vec::new(),
                coordinate_bits: Vec::new(),
            }),
            rings,
            height,
            transform: DMat4::IDENTITY,
            cutters,
            diagnostics: Vec::new(),
        }
    }

    /// SEXT 名下的 NSEX 刀扣掉的必须恰好是它自己的挤出体：两边同走 `extrude_rings`，
    /// 内缩 10 的方孔沿高度贯穿后，剩下的是两只棱柱的解析差，且多出一个环柄。
    /// ams5052 `/ZSPE660NN2-GS` 的仪表外壳就是这个形态（外壳 − 内缩 2 mm 的 NSEX）。
    #[test]
    fn an_nsex_cutter_hollows_its_own_sext() {
        let outer = vec![[0.0, 0.0], [100.0, 0.0], [100.0, 100.0], [0.0, 100.0]];
        let hole = vec![[10.0, 10.0], [90.0, 10.0], [90.0, 90.0], [10.0, 90.0]];
        // 刀比外壳高 20，两头各探出 10（无过切，探出来自刀自己的高度）。
        let body = build_sext(&sext_fixture(
            vec![outer.clone()],
            50.0,
            vec![EvaluatedCatalogueCut::Extrusion {
                rings: vec![hole],
                height: 70.0,
                transform: DMat4::from_translation(DVec3::new(0.0, 0.0, -10.0)),
            }],
        ))
        .unwrap();
        let expected = 100.0 * 100.0 * 50.0 - 80.0 * 80.0 * 50.0;
        assert!(
            (body.volume() - expected).abs() < 1e-6,
            "体积 {} 应为 {expected}",
            body.volume()
        );
        assert_eq!(body.genus(), 1, "贯穿的方孔应当带来一个环柄");

        // 无刀时必须与旧路（裸 extrude_rings）逐位相同——键没变，缓存也不该分岔。
        let plain = build_sext(&sext_fixture(vec![outer.clone()], 50.0, Vec::new())).unwrap();
        let reference = extrude_rings(vec![outer], 50.0).unwrap();
        assert_eq!(plain.num_tri(), reference.num_tri());
        assert!((plain.volume() - reference.volume()).abs() < 1e-9);
    }

    /// `owned_negatives_of` 的后半：**任一**正体网格都能扣名下的刀——`getCSGTree` 的差集
    /// 不看正体是什么。拿 LPYR 的建体函数当样本（四边等长的棱台就是盒子），竖直打一个通孔。
    #[test]
    fn subtract_owned_negatives_hollows_any_positive_body() {
        let body = pyramid_solid(100.0, 100.0, 100.0, 100.0, 50.0, 0.0, 0.0).unwrap();
        let solid_volume = body.volume();

        // 空刀集：原样返回，与旧路逐位相同。
        let plain = subtract_owned_negatives(body.clone(), &[], "test").unwrap();
        assert_eq!(plain.num_tri(), body.num_tri());
        assert_eq!(plain.volume().to_bits(), solid_volume.to_bits());

        // Ø20 × 60 的圆柱刀居中穿过 50 高的体：体积少一根圆柱，多一个环柄。
        let bore = EvaluatedCatalogueCut::Cylinder(EvaluatedNscy {
            diameter: 20.0,
            height: 60.0,
            transform: DMat4::IDENTITY,
        });
        let cut =
            subtract_owned_negatives(body.clone(), std::slice::from_ref(&bore), "test").unwrap();
        let removed = solid_volume - cut.volume();
        // 刀是 `cutter_solid` 建的内接多边形柱（Ø20 分 12 段，截面 3r² = 300），60 高的刀只有
        // 体内那 50 高被扣掉：扣掉的体积 = 刀体积 × 50/60，逐位可算。
        let (bore_solid, _) = cutter_solid(&bore).unwrap();
        let expected = bore_solid.volume() * 50.0 / 60.0;
        assert!(
            (removed - expected).abs() < 1e-6,
            "扣掉 {removed}，应为刀在体内的那段 {expected}"
        );
        assert_eq!(cut.genus(), 1, "贯穿孔应当带来一个环柄");

        // 刀把正体整个吃掉不是「没东西」，是要落 note 的错。
        let swallow = EvaluatedCatalogueCut::Box {
            dimensions: DVec3::new(400.0, 400.0, 400.0),
            transform: DMat4::IDENTITY,
        };
        assert!(
            subtract_owned_negatives(body, std::slice::from_ref(&swallow), "test").is_err(),
            "扣空的正体必须报错，不能静默出空体"
        );
    }

    /// 刀的 noun 单子全 crate 只许有一份（`collect_owned_negatives`），而且每一条正体臂都
    /// 必须走它——SREV / SEXT / SBOX / 圆柱在求值器里收，其余七臂在 dispatch 里经
    /// `owned_negatives_of` 收。
    ///
    /// ams7997 的 24381/637 名下 8 件 NSRT 就是这么漏的：SREV 那条早就接了
    /// `evaluate_catalogue_torus`，SBOX 这条没接，报告上只落一行
    /// `noun NSRT is not implemented`，账面全绿。给一条加了臂、另一条忘了，
    /// 行为上看不出来，只能从源码钉。同一只 INST 的 SEXT 名下 1 只 NSEX + 4 只 NSCY
    /// 漏得更彻底——连 note 都没有——所以 SEXT 不另抄一份，直接走 SREV 那份。
    #[test]
    fn both_negative_collectors_recognise_the_same_nouns() {
        fn nouns(body: &str) -> BTreeSet<String> {
            let mut found = BTreeSet::new();
            for line in body.lines().map(str::trim) {
                if line.starts_with("//") {
                    continue;
                }
                for chunk in line.split("eq_ignore_ascii_case(\"").skip(1) {
                    if let Some((noun, _)) = chunk.split_once('"') {
                        found.insert(noun.to_string());
                    }
                }
            }
            found
        }

        let source = include_str!("catalogue.rs");
        let srev = source
            .split_once("\nfn collect_owned_negatives(")
            .expect("collect_owned_negatives 必须在")
            .1
            .split_once("\nfn evaluate_sbox(")
            .expect("collect_owned_negatives 在 evaluate_sbox 之前收尾")
            .0;
        // SREV / SEXT / 圆柱三条正体臂都必须真的走这份收集器——只要有一条改回自己抄，
        // 这张单子就又只钉住一部分。圆柱臂 2026-09-06 从「只认 NSCY/NLCY、其余 not implemented」
        // 并进来（INST 24381/637 的 SCYL 名下 4 NSCT + 21 NSBO 曾这么静默漏掉）。
        for arm in [
            "\nfn evaluate_srev(",
            "\nfn evaluate_sext_body(",
            "\nfn evaluate_catalogue_cylinder(",
            "\nfn owned_negatives_of(",
        ] {
            let body = source
                .split_once(arm)
                .unwrap_or_else(|| panic!("{arm} 必须在"))
                .1
                .split_once("\nfn ")
                .expect("函数之后还有函数")
                .0;
            assert!(
                body.contains("collect_owned_negatives("),
                "{} 没有调用 collect_owned_negatives",
                arm.trim()
            );
        }
        // 其余七条正体臂（LPYR / LSNO|SCON / SCTO|SRTO / SSPH / SSLC / SDSH / SDIS）在
        // dispatch 里走 `owned_negatives_of`：`CSG_TreeBuilderCat::getCSGTree` 对每一个有
        // 成员的正体都开差集，不分 noun。少一条就是那一种属主名下的刀又静默没了
        // （42 个目录库里 LPYR 名下 3683 只、LSNO 794 只……普查 2026-09-06）。
        let dispatch = source
            .split_once("\npub fn model_catalogue_member_with_cache(")
            .expect("model_catalogue_member_with_cache 必须在")
            .1
            .split_once("\nfn preserves_catalogue_components(")
            .expect("dispatch 在 preserves_catalogue_components 之前收尾")
            .0;
        let arms_with_owned_negatives = dispatch.matches("owned_negatives_of(").count();
        assert_eq!(
            arms_with_owned_negatives, 7,
            "dispatch 里走 owned_negatives_of 的正体臂应为 7 条（LPYR / LSNO|SCON / SCTO|SRTO / \
             SSPH / SSLC / SDSH / SDIS），现在 {arms_with_owned_negatives} 条"
        );
        let sbox = source
            .split_once("\nfn evaluate_sbox(")
            .expect("evaluate_sbox 必须在")
            .1
            .split_once("\nfn evaluate_nlcy(")
            .expect("evaluate_sbox 在 evaluate_nlcy 之前收尾")
            .0;

        // SBOX 名下原来另抄一份收集器，noun 单子靠这条守卫与 SREV 那份对齐；2026-09-06 并进
        // 共享的那份之后，SBOX 臂里不该再出现任何按 noun 分岔的代码——再出现就是又抄回去了。
        assert!(
            sbox.contains("collect_owned_negatives("),
            "evaluate_sbox 没有调用 collect_owned_negatives"
        );
        let sbox_private: Vec<String> = nouns(sbox).into_iter().collect();
        assert!(
            sbox_private.is_empty(),
            "evaluate_sbox 又自己认起 noun 来了：{sbox_private:?}——刀的 noun 单子只许在 \
             collect_owned_negatives 里有一份"
        );
        // 共享收集器认全 CRCATI（`0x10345ACC`）的负臂：15 个 N* noun 里只差 NBXI / NTUB
        // （正体 BOXI / TUBE 是 1 mm stub、42 个目录库里负体零件），少一个就是那种刀静默没了。
        let recognised = nouns(srev);
        for noun in [
            "NSCY", "NLCY", "NSBO", "NLPY", "NSEX", "NLSN", "NSCO", "NSCT", "NSRT", "NSRE", "NSSP",
            "NSDS", "NSSL",
        ] {
            assert!(
                recognised.contains(noun),
                "collect_owned_negatives 不认 {noun}，这种刀会落 `is not implemented`"
            );
        }
    }

    /// 同一只网格、同一个放置放两次：体只有一块，先放那一块记 `repeats = 2`；
    /// 换个放置就是另一块体。E3D 对重复的成员照写一条记录，记录份数从这里来。
    #[test]
    fn a_repeated_placement_adds_a_record_not_a_body() {
        let mut cache = CatalogueMeshCache::default();
        let mut placed_parts: HashMap<PlacedPartKey, usize> = HashMap::new();
        let mut solids: Vec<CataloguePart> = Vec::new();
        let key = CatalogueMeshKey::Cylinder {
            tessellation_version: 0,
            diameter_bits: 40.0_f64.to_bits(),
            height_bits: 74.0_f64.to_bits(),
        };
        let mesh_id =
            cached_mesh_id(&mut cache, &key, || Ok(csg_cylinder_solid(40.0, 74.0)?)).unwrap();
        let at = DMat4::from_translation(DVec3::new(10.0, 0.0, 0.0));
        let elsewhere = DMat4::from_translation(DVec3::new(-10.0, 0.0, 0.0));

        assert!(place_part_once(
            &mut cache,
            &mut placed_parts,
            &mut solids,
            "SCYL",
            &key,
            mesh_id,
            at,
        ));
        assert!(!place_part_once(
            &mut cache,
            &mut placed_parts,
            &mut solids,
            "SCYL",
            &key,
            mesh_id,
            at,
        ));
        assert!(place_part_once(
            &mut cache,
            &mut placed_parts,
            &mut solids,
            "SCYL",
            &key,
            mesh_id,
            elsewhere,
        ));

        assert_eq!(solids.len(), 2, "重复放置不得另放一块体");
        assert_eq!(
            solids.iter().map(|part| part.repeats).collect::<Vec<_>>(),
            [2, 1],
            "重复记在先放那一块身上"
        );
        assert_eq!(cache.stats.unique_placed_parts, 2);
        assert_eq!(cache.stats.duplicate_placed_parts, 1);
        assert_eq!(placed_parts.len(), 2);
    }
}

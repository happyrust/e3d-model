//! Design-member catalogue geometry.
//!
//! Route members are instances: the design record carries selection and
//! dimensions, while the reusable shape program lives under
//! `SPRE -> SCOM -> GMRE`.  This module evaluates that program without copying
//! the existing profile discretiser or solid builders.  The first proven slice
//! is Core3D `QSSEXT`: `SEXT -> SLOO -> SVER` becomes the same
//! [`crate::solid::extrude_rings`] body used by design-side extrusions.

use std::collections::{HashMap, HashSet};

use glam::{DMat3, DMat4, DVec3};
use manifold_csg::Manifold;

use e3d_io::db_element::DbElement;
use e3d_io::db_param_env::DbElementParamEnv;
use e3d_io::record::catalogue_eval::AttributeNumber;
use e3d_io::record::descriptor::DescriptorValue;

use crate::catalogue_point::{CatalogueAxis, catalogue_component, resolve_catalogue_axis};
use crate::category::{RepreDiscipline, repre_discipline};
use crate::discretise::{FilletVertex, flatten_ring};
use crate::elmodl::{GeneratedElement, GeometryId, ModelError};
use crate::primitive_instance::instantiate_box;
use crate::route::orimat;
use crate::solid::{
    SolidError, box_solid, circ_torus_solid, csg_cylinder_solid, cylinder_segments,
    dish_knuckle_radius, elliptical_dish_solid, extrude_rings, placed, pyramid_solid,
    rect_torus_solid, revolve_rings, slope_ended_cylinder_solid, snout_solid,
    snout_solid_with_segments, spherical_dish_solid, unit_sphere_solid,
};
use crate::transform::world_matrix;

const GEOMETRY_REF: &str = "GMRE";
const AXIS_TOL: f64 = 1.0e-9;
const HEIGHT_TOL_MM: f64 = 1.0e-6;
const SEXT_TESSELLATION_VERSION: u32 = 1;
const SREV_TESSELLATION_VERSION: u32 = 1;
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
    Sext(SextMeshKey),
    Srev(SrevMeshKey),
    Sbox(SboxMeshKey),
    Cylinder {
        tessellation_version: u32,
        diameter_bits: u64,
        height_bits: u64,
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
    transform: DMat4,
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
pub(crate) fn repre_obst_on() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| {
        std::env::var("E3D_REPRE_OBST").ok().is_some_and(|value| {
            matches!(
                value.trim().to_ascii_lowercase().as_str(),
                "on" | "1" | "true" | "yes"
            )
        })
    })
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
/// 曾经的反例已经消化：`C-IY` 基准里 `OBST=1/TUFL=false` 的净空盒被 E3D 导出，是因为那份
/// addin 导出开着 `REPRE OBST`（TTY 基线宏是 `repre obst off`）。DR3DWA / DRWANT 的公式是
/// `in_level ∧ (OBST>0 ∧ obst_on ∨ TUFL ∧ tube_on ∨ CLFL ∧ cl_on)`——`TUFL=false` 一律不画只是
/// `obst off / cl off` 下的特例；[`repre_obst_on`] 打开时 OBST 1/2 的图元照画（对拍托盘语料
/// 时设 `E3D_REPRE_OBST=on`）。
/// HANG 24381/177986 的 HELEMENT 把 `CLFL=true` 的三个 SCYL 和一个 SEXT 全部导出——
/// 它们 `TUFL=true`，本筛照画；"CLFL 即隐藏"仍然是错的读法。
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
    if obstruction > 0 && repre_obst_on() {
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
    if let Some(build) = model_hvac_rectangular_basic(member)? {
        return Ok(build);
    }
    let Some(component) = catalogue_component(member)? else {
        // 两种没形状要分开记：没有 SPRE 是设计数据没给规格（E3D 同样不出几何）；有 SPRE
        // 却走不到 SCOM 是链断（SPCO 无 CATR/PRTREF、引用悬空、库没开），得指出从哪儿断。
        let no_shape = match member.get_element("SPRE")? {
            None => "路由成员无 SPRE：没有目录链，无几何（E3D 同样不出）".to_string(),
            Some(spec) => format!(
                "目录链从 SPRE {} 走不到 SCOM（SPCO 无 CATR/PRTREF 或引用悬空），无几何",
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
    let env =
        DbElementParamEnv::new(Some(member.clone()), Some(component.clone())).without_insulation();
    let mut solids = Vec::new();
    let mut ignored_lines = 0usize;
    let mut sext_total = 0usize;
    let mut zero_height = 0usize;
    // SEXT 之外各臂求值返回 `Ok(None)` 的次数：零高 / 退化，与 Core 一样不出体。此前这一档
    // 一笔不记，一只 LCYL 全零高的 FTUB 在报告上与「漏建」长得一样。
    let mut degenerate_other = 0usize;
    let mut primitives = 0usize;
    let mut placed_parts = HashSet::new();
    let mut member_duplicates = 0usize;
    let mut catalogue_negatives = 0usize;
    // 过滤记录与 diagnostics 分开收：`failed` 的账目 = diagnostics.len()，
    // 被表现级筛掉不是失败，不能混进去。
    let mut display_filter_notes = Vec::new();
    let mut diagnostics = Vec::new();
    let mut saw_display_primitive = false;
    let mut all_display_primitives_are_clfl = true;
    // 表现级按成员的学科取（MODCMP：管道构件 `LEVEL PIPE`、风管 `LEVEL HVAC`…），整只成员一个数。
    let member_discipline = repre_discipline_of(member)?;
    let member_level = repre_level_for(member_discipline);

    for primitive in geometry.members()? {
        primitives += 1;
        let primitive_noun = primitive.element_type()?;
        // LINE 成员本来就只记账不建体，不进显示过滤，保住 centre_lines 的账目口径。
        if primitive_noun != "LINE"
            && let Some(reason) = display_filtered(&primitive, member_level)?
        {
            display_filter_notes.push(format!(
                "catalogue primitive {} noun {primitive_noun} display-filtered by {reason} (repre {} {member_level}, obst {})",
                primitive.refno(),
                member_discipline.command(),
                if repre_obst_on() { "on" } else { "off" },
            ));
            continue;
        }
        if primitive_noun != "LINE" {
            saw_display_primitive = true;
            all_display_primitives_are_clfl &= primitive.get_bool("CLFL")? == Some(true);
        }
        match primitive_noun.as_str() {
            "SEXT" => {
                sext_total += 1;
                cache.stats.source_sext += 1;
                match evaluate_sext(member, &component, &primitive, &env) {
                    Ok(Some(evaluated)) => {
                        cache.stats.evaluated_sext += 1;
                        let mesh_id = match cached_mesh_id(cache, &evaluated.mesh_key, || {
                            Ok(extrude_rings(evaluated.rings.clone(), evaluated.height)?)
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
                        let placed_key = PlacedPartKey {
                            mesh_id,
                            transform_bits: matrix_bits(evaluated.transform),
                        };
                        if !placed_parts.insert(placed_key) {
                            member_duplicates += 1;
                            cache.stats.duplicate_placed_parts += 1;
                            continue;
                        }
                        cache.stats.unique_placed_parts += 1;
                        let local_mesh = &cache
                            .local_meshes
                            .get(&evaluated.mesh_key)
                            .expect("catalogue mesh inserted above")
                            .solid;
                        solids.push(placed(local_mesh, evaluated.transform));
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
                        let placed_key = PlacedPartKey {
                            mesh_id,
                            transform_bits: matrix_bits(evaluated.transform),
                        };
                        if !placed_parts.insert(placed_key) {
                            member_duplicates += 1;
                            cache.stats.duplicate_placed_parts += 1;
                            continue;
                        }
                        cache.stats.unique_placed_parts += 1;
                        let local_mesh = &cache
                            .local_meshes
                            .get(&evaluated.mesh_key)
                            .expect("catalogue mesh inserted above")
                            .solid;
                        solids.push(placed(local_mesh, evaluated.transform));
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
                        let placed_key = PlacedPartKey {
                            mesh_id,
                            transform_bits: matrix_bits(evaluated.transform),
                        };
                        if !placed_parts.insert(placed_key) {
                            member_duplicates += 1;
                            cache.stats.duplicate_placed_parts += 1;
                            continue;
                        }
                        cache.stats.unique_placed_parts += 1;
                        let local_mesh = &cache
                            .local_meshes
                            .get(&evaluated.mesh_key)
                            .expect("catalogue mesh inserted above")
                            .solid;
                        solids.push(placed(local_mesh, evaluated.transform));
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
                        let mesh_id = match cached_mesh_id(cache, &evaluated.mesh_key, || {
                            let [xbot, ybot, xtop, ytop, height, xoff, yoff] = evaluated.parameters;
                            Ok(pyramid_solid(xbot, ybot, xtop, ytop, height, xoff, yoff)?)
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
                        let placed_key = PlacedPartKey {
                            mesh_id,
                            transform_bits: matrix_bits(evaluated.transform),
                        };
                        if !placed_parts.insert(placed_key) {
                            member_duplicates += 1;
                            cache.stats.duplicate_placed_parts += 1;
                            continue;
                        }
                        cache.stats.unique_placed_parts += 1;
                        let local_mesh = &cache
                            .local_meshes
                            .get(&evaluated.mesh_key)
                            .expect("catalogue mesh inserted above")
                            .solid;
                        solids.push(placed(local_mesh, evaluated.transform));
                    }
                    Ok(None) => degenerate_other += 1,
                    Err(error) => diagnostics.push(format!(
                        "catalogue primitive {} skipped by Core-style local failure: {error}",
                        primitive.refno()
                    )),
                }
            }
            noun @ ("LSNO" | "SCON") => {
                match evaluate_catalogue_snout(member, &component, &primitive, &env, noun) {
                    Ok(Some(evaluated)) => {
                        let mesh_id = match cached_mesh_id(cache, &evaluated.mesh_key, || {
                            let [dbot, dtop, height, xoff, yoff] = evaluated.parameters;
                            // libgm `GM_Snout` 走曲面口径（与 GM_Cylinder 同一条 0.5 mm 规则，
                            // 喂大端半径），不是设计侧 SNOU FacetGroup 那条 5 mm。
                            Ok(snout_solid_with_segments(
                                dbot,
                                dtop,
                                height,
                                xoff,
                                yoff,
                                cylinder_segments(dbot.max(dtop) * 0.5),
                            )?)
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
                        let placed_key = PlacedPartKey {
                            mesh_id,
                            transform_bits: matrix_bits(evaluated.transform),
                        };
                        if !placed_parts.insert(placed_key) {
                            member_duplicates += 1;
                            cache.stats.duplicate_placed_parts += 1;
                            continue;
                        }
                        cache.stats.unique_placed_parts += 1;
                        let local_mesh = &cache
                            .local_meshes
                            .get(&evaluated.mesh_key)
                            .expect("catalogue mesh inserted above")
                            .solid;
                        solids.push(placed(local_mesh, evaluated.transform));
                    }
                    Ok(None) => degenerate_other += 1,
                    Err(error) => diagnostics.push(format!(
                        "catalogue primitive {} skipped by Core-style local failure: {error}",
                        primitive.refno()
                    )),
                }
            }
            noun @ ("SCYL" | "LCYL") => {
                if noun == "SCYL" {
                    cache.stats.source_scyl += 1;
                } else {
                    cache.stats.source_lcyl += 1;
                }
                match evaluate_catalogue_cylinder(member, &component, &primitive, &env, noun) {
                    Ok(Some(evaluated)) => {
                        if noun == "SCYL" {
                            cache.stats.evaluated_scyl += 1;
                        } else {
                            cache.stats.evaluated_lcyl += 1;
                        }
                        let mesh_id = match cached_mesh_id(cache, &evaluated.mesh_key, || {
                            Ok(csg_cylinder_solid(evaluated.diameter, evaluated.height)?)
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
                        let placed_key = PlacedPartKey {
                            mesh_id,
                            transform_bits: matrix_bits(evaluated.transform),
                        };
                        if !placed_parts.insert(placed_key) {
                            member_duplicates += 1;
                            cache.stats.duplicate_placed_parts += 1;
                            continue;
                        }
                        cache.stats.unique_placed_parts += 1;
                        let local_mesh = &cache
                            .local_meshes
                            .get(&evaluated.mesh_key)
                            .expect("catalogue mesh inserted above")
                            .solid;
                        solids.push(placed(local_mesh, evaluated.transform));
                    }
                    Ok(None) => degenerate_other += 1,
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
                        let mesh_id = match cached_mesh_id(cache, &evaluated.mesh_key, || {
                            Ok(match evaluated.kind {
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
                            })
                        }) {
                            Ok(mesh_id) => mesh_id,
                            Err(error) => {
                                diagnostics.push(format!(
                                    "catalogue primitive {} skipped by Core-style local failure: {error}; torus inner={} outer={} angle_deg={}",
                                    primitive.refno(),
                                    evaluated.inner,
                                    evaluated.outer,
                                    evaluated.angle_deg,
                                ));
                                continue;
                            }
                        };
                        let placed_key = PlacedPartKey {
                            mesh_id,
                            transform_bits: matrix_bits(evaluated.transform),
                        };
                        if !placed_parts.insert(placed_key) {
                            member_duplicates += 1;
                            cache.stats.duplicate_placed_parts += 1;
                            continue;
                        }
                        cache.stats.unique_placed_parts += 1;
                        let local_mesh = &cache
                            .local_meshes
                            .get(&evaluated.mesh_key)
                            .expect("catalogue mesh inserted above")
                            .solid;
                        solids.push(placed(local_mesh, evaluated.transform));
                    }
                    Ok(None) => degenerate_other += 1,
                    Err(error) => diagnostics.push(format!(
                        "catalogue primitive {} skipped by Core-style local failure: {error}",
                        primitive.refno()
                    )),
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
                        let mesh_id = match cached_mesh_id(cache, &evaluated.mesh_key, || {
                            Ok(unit_sphere_solid(segments)?)
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
                        let placed_key = PlacedPartKey {
                            mesh_id,
                            transform_bits: matrix_bits(evaluated.transform),
                        };
                        if !placed_parts.insert(placed_key) {
                            member_duplicates += 1;
                            cache.stats.duplicate_placed_parts += 1;
                            continue;
                        }
                        cache.stats.unique_placed_parts += 1;
                        let local_mesh = &cache
                            .local_meshes
                            .get(&evaluated.mesh_key)
                            .expect("catalogue mesh inserted above")
                            .solid;
                        solids.push(placed(local_mesh, evaluated.transform));
                    }
                    Ok(None) => degenerate_other += 1,
                    Err(error) => diagnostics.push(format!(
                        "catalogue primitive {} skipped by Core-style local failure: {error}",
                        primitive.refno()
                    )),
                }
            }
            "SSLC" => {
                cache.stats.source_sslc += 1;
                match evaluate_sslc(member, &component, &primitive, &env) {
                    Ok(Some(evaluated)) => {
                        cache.stats.evaluated_sslc += 1;
                        let [xtop, ytop, xbot, ybot] = evaluated.shear_deg;
                        let outcome = place_cached_part(
                            cache,
                            &mut placed_parts,
                            &mut solids,
                            &evaluated.mesh_key,
                            evaluated.transform,
                            || {
                                Ok(slope_ended_cylinder_solid(
                                    evaluated.radius,
                                    evaluated.height,
                                    xtop,
                                    ytop,
                                    xbot,
                                    ybot,
                                )?)
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
                        let outcome = match &evaluated {
                            EvaluatedSdsh::Dish(dish) => place_cached_part(
                                cache,
                                &mut placed_parts,
                                &mut solids,
                                &dish.mesh_key,
                                dish.transform,
                                || Ok(build_dish(dish)?),
                            ),
                            EvaluatedSdsh::Disc(disc) => place_cached_part(
                                cache,
                                &mut placed_parts,
                                &mut solids,
                                &disc.mesh_key,
                                disc.transform,
                                || Ok(csg_cylinder_solid(disc.diameter, disc.height)?),
                            ),
                        };
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
                        let outcome = place_cached_part(
                            cache,
                            &mut placed_parts,
                            &mut solids,
                            &disc.mesh_key,
                            disc.transform,
                            || Ok(csg_cylinder_solid(disc.diameter, disc.height)?),
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
            // GMRE LINE entries are centreline metadata, not visible solids.
            "LINE" => ignored_lines += 1,
            unsupported => diagnostics.push(format!(
                "catalogue primitive {} noun {unsupported} is not implemented",
                primitive.refno()
            )),
        }
    }
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

    let local = Manifold::compose(&solids);
    if local.is_empty() || local.num_tri() == 0 {
        return Err(ModelError::Solid(SolidError::EmptyManifold {
            what: "catalogue assembly",
        }));
    }
    let world = world_matrix(member)?;
    let noun = member.element_type()?;
    // E3D keeps CLFL display programs as independent RVM components even when
    // their primitives overlap. NPPIPE ATTA is noun-wide rather than flag-wide:
    // RVM 24383/73951 and /73956 retain both visible cylinders, while the binary
    // catalogue flags do not expose a stable CLFL discriminator for this pair.
    let preserve_components = preserves_catalogue_components(
        &noun,
        saw_display_primitive,
        all_display_primitives_are_clfl,
    );
    let parts = preserve_components.then(|| {
        solids
            .iter()
            .map(|solid| placed(solid, world))
            .collect::<Vec<_>>()
    });
    let notes = vec![format!(
        "catalogue assembly: built={} sext_total={} sext_zero_height={} degenerate_other={} failed={} centre_lines={} display_filtered={} duplicate_placements={} component={}",
        solids.len(),
        sext_total,
        zero_height,
        degenerate_other,
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
            name: member.stored_name()?,
            solid: placed(&local, world),
            parts,
            world,
            primitive_instance: None,
            negatives_applied: catalogue_negatives,
            negatives_skipped: Vec::new(),
            notes,
        }),
        diagnostics,
    })
}

fn preserves_catalogue_components(
    member_noun: &str,
    saw_display_primitive: bool,
    all_display_primitives_are_clfl: bool,
) -> bool {
    saw_display_primitive
        && (all_display_primitives_are_clfl
            || member_noun.eq_ignore_ascii_case("ATTA"))
}

/// Core expands HVAC `DESP` through xrules before catalogue evaluation. The
/// common rectangular straight/branch families reduce to one visible box in a
/// narrow (`insu/obst off`) RVM export, so keep that body on the canonical box
/// instancing path instead of rebuilding the catalogue body/insulation SBOXes.
fn model_hvac_rectangular_basic(
    member: &DbElement,
) -> Result<Option<CatalogueMemberBuild>, ModelError> {
    let Some(spec) = member.get_element("SPRE")? else {
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
    if let Some(instance) = &mut built.instance {
        instance.local_transform = centre * instance.local_transform;
    }
    let world = world_matrix(member)?;
    let noun = member.element_type()?;
    Ok(Some(CatalogueMemberBuild {
        generated: Some(GeneratedElement {
            geometry_id: GeometryId::Element {
                refno: member.refno().to_string(),
            },
            refno: member.refno(),
            noun,
            name: member.stored_name()?,
            solid: placed(&built.solid, world),
            parts: None,
            world,
            primitive_instance: built.instance,
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
    let mut xtop = required_number(pyramid, "PBTP", env, "catalogue LPYR")?.abs();
    let mut ytop = required_number(pyramid, "PCTP", env, "catalogue LPYR")?.abs();
    let mut xbot = required_number(pyramid, "PBBT", env, "catalogue LPYR")?.abs();
    let mut ybot = required_number(pyramid, "PCBT", env, "catalogue LPYR")?.abs();
    let top_distance = required_number(pyramid, "PBOF", env, "catalogue LPYR")?;
    let bottom_distance = required_number(pyramid, "PCOF", env, "catalogue LPYR")?;
    let xoff = required_number(pyramid, "PTDI", env, "catalogue LPYR")?;
    let mut yoff = required_number(pyramid, "PBDI", env, "catalogue LPYR")?;

    let mut pa = catalogue_axis_or_fail(member, component, pyramid, "PAAX", "catalogue LPYR axis")?;
    let pb = catalogue_axis_or_fail(member, component, pyramid, "PBAX", "catalogue LPYR axis")?;
    let pc = catalogue_axis_or_fail(member, component, pyramid, "PCAX", "catalogue LPYR axis")?;

    let mut height = top_distance - bottom_distance;
    if height <= 0.0 {
        height = -height;
        pa.direction = -pa.direction;
        std::mem::swap(&mut xbot, &mut xtop);
        std::mem::swap(&mut ybot, &mut ytop);
    }
    if height < HEIGHT_TOL_MM {
        return Ok(None);
    }
    let z = pa.direction.normalize_or_zero();
    let x = pb.direction.normalize_or_zero();
    let y = z.cross(x).normalize_or_zero();
    if x.length_squared() <= AXIS_TOL
        || y.length_squared() <= AXIS_TOL
        || z.length_squared() <= AXIS_TOL
    {
        return Err(ModelError::Solid(SolidError::Degenerate {
            what: "catalogue LPYR axes",
            detail: format!(
                "PAAX={:?}, PBAX={:?}, PCAX={:?}",
                pa.direction, pb.direction, pc.direction
            ),
        }));
    }
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
        let mut axis =
            catalogue_axis_or_fail(member, component, snout, "PAXI", "catalogue SCON axis")?;
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
        (
            diameter,
            0.0,
            height,
            0.0,
            axis.origin + axis.direction.normalize() * (height * 0.5),
            rotation,
        )
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
        let (dbot, dtop, height, offset) = normalize_lsnout_span(dbot, dtop, bottom, top, offset);
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

fn normalize_lsnout_span(
    dbot: f64,
    dtop: f64,
    bottom: f64,
    top: f64,
    offset: f64,
) -> (f64, f64, f64, f64) {
    let height = top - bottom;
    if height < 0.0 {
        // Re-parameterising the span from low to high swaps both end labels
        // and the end-to-end eccentric offset. PAAX itself remains unchanged.
        (dtop, dbot, -height, -offset)
    } else {
        (dbot, dtop, height, offset)
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

fn evaluate_nlpy(
    member: &DbElement,
    component: &DbElement,
    pyramid: &DbElement,
    env: &DbElementParamEnv,
) -> Result<Option<EvaluatedPyramid>, ModelError> {
    if pyramid.get_bool("TUFL")? == Some(false) {
        return Ok(None);
    }
    let mut xtop = required_number(pyramid, "PBTP", env, "catalogue NLPY")?.abs();
    let mut ytop = required_number(pyramid, "PCTP", env, "catalogue NLPY")?.abs();
    let mut xbot = required_number(pyramid, "PBBT", env, "catalogue NLPY")?.abs();
    let mut ybot = required_number(pyramid, "PCBT", env, "catalogue NLPY")?.abs();
    let top = required_number(pyramid, "PTDI", env, "catalogue NLPY")?;
    let bottom = required_number(pyramid, "PBDI", env, "catalogue NLPY")?;
    let xoff = required_number(pyramid, "PBOF", env, "catalogue NLPY")?;
    let yoff = required_number(pyramid, "PCOF", env, "catalogue NLPY")?;
    let mut axis =
        catalogue_axis_or_fail(member, component, pyramid, "PAAX", "catalogue NLPY axis")?;
    let mut height = top - bottom;
    if height < 0.0 {
        height = -height;
        axis.direction = -axis.direction;
        std::mem::swap(&mut xbot, &mut xtop);
        std::mem::swap(&mut ybot, &mut ytop);
    }
    if height < HEIGHT_TOL_MM {
        return Ok(None);
    }
    let rotation = orimat(axis.direction).ok_or_else(|| {
        ModelError::Solid(SolidError::Degenerate {
            what: "catalogue NLPY axis",
            detail: format!("PAAX={:?}", axis.direction),
        })
    })?;
    let parameters = [xbot, ybot, xtop, ytop, height, xoff, yoff];
    Ok(Some(EvaluatedPyramid {
        mesh_key: CatalogueMeshKey::Pyramid {
            tessellation_version: PYRAMID_TESSELLATION_VERSION,
            parameter_bits: parameters.map(|value| value.to_bits()),
        },
        parameters,
        transform: DMat4::from_translation(axis.origin + axis.direction * ((bottom + top) * 0.5))
            * DMat4::from_mat3(rotation),
    }))
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
                frad: optional_number(&vertex, "PRAD", env).unwrap_or(0.0).abs(),
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
    Ok(Some(rings))
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
    let transform =
        DMat4::from_translation(pa.origin + rotation * raw_position) * DMat4::from_mat3(rotation);
    let mesh_key = CatalogueMeshKey::Sext(sext_mesh_key(&rings, height)?);
    Ok(Some(EvaluatedSext {
        mesh_key,
        rings,
        height,
        transform,
    }))
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

    let mut cutters = Vec::new();
    let mut diagnostics = Vec::new();
    for child in srev.members()? {
        let noun = child.element_type()?;
        if noun.eq_ignore_ascii_case("SLOO") {
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
        } else {
            diagnostics.push(format!(
                "catalogue negative {} noun {noun} is not implemented",
                child.refno()
            ));
        }
    }

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
    let mut cutters = Vec::new();
    let mut diagnostics = Vec::new();
    for child in sbox.members()? {
        let noun = child.element_type()?;
        if noun.eq_ignore_ascii_case("NSCY") {
            cache.stats.source_nscy += 1;
            match evaluate_nscy(member, component, &child, env, position) {
                Ok(Some(cutter)) => {
                    cache.stats.evaluated_nscy += 1;
                    cutters.push(EvaluatedCatalogueCut::Cylinder(cutter));
                }
                Ok(None) => {}
                Err(error) => diagnostics.push(format!(
                    "catalogue negative {} skipped by Core-style local failure: {error}",
                    child.refno()
                )),
            }
        } else if noun.eq_ignore_ascii_case("NSBO") {
            cache.stats.source_nsbo += 1;
            match evaluate_nsbo(&child, env, position) {
                Ok(Some(cutter)) => {
                    cache.stats.evaluated_nsbo += 1;
                    cutters.push(cutter);
                }
                Ok(None) => {}
                Err(error) => diagnostics.push(format!(
                    "catalogue negative {} skipped by Core-style local failure: {error}",
                    child.refno()
                )),
            }
        } else if noun.eq_ignore_ascii_case("NLCY") {
            cache.stats.source_nlcy += 1;
            match evaluate_nlcy(member, component, &child, env, position) {
                Ok(Some(cutter)) => {
                    cache.stats.evaluated_nlcy += 1;
                    cutters.push(EvaluatedCatalogueCut::Cylinder(cutter));
                }
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
                    transform: DMat4::from_translation(-position) * cutter.transform,
                }),
                Ok(None) => {}
                Err(error) => diagnostics.push(format!(
                    "catalogue negative {} skipped by Core-style local failure: {error}",
                    child.refno()
                )),
            }
        } else if noun.eq_ignore_ascii_case("NSEX") {
            // CRCATI 942751/1008005：正体 SEXT 与负体 NSEX 落同一个臂
            // （`QSSEXT` 0x103B7FCC，高度 ≥1e-6 才建），几何完全同构、负性由
            // 消费方施加，且**没有** NSCY 那条 +0.01 轴向过切——反编译核实，
            // 见方案文档 §2.1/§2.3。放置照 NLPY 的样式回到 SBOX 局部系。
            cache.stats.source_nsex += 1;
            match evaluate_sext(member, component, &child, env) {
                Ok(Some(cutter)) => {
                    cache.stats.evaluated_nsex += 1;
                    cutters.push(EvaluatedCatalogueCut::Extrusion {
                        rings: cutter.rings,
                        height: cutter.height,
                        transform: DMat4::from_translation(-position) * cutter.transform,
                    });
                }
                Ok(None) => {}
                Err(error) => diagnostics.push(format!(
                    "catalogue negative {} skipped by Core-style local failure: {error}",
                    child.refno()
                )),
            }
        } else {
            diagnostics.push(format!(
                "catalogue negative {} noun {noun} is not implemented",
                child.refno()
            ));
        }
    }
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
    clfl && primitive_noun.eq_ignore_ascii_case("SCYL") && !member_noun.eq_ignore_ascii_case("ATTA")
}

fn atta_clfl_roll(
    member_noun: &str,
    primitive_noun: &str,
    clfl: bool,
    orientation: Option<&[f64]>,
) -> Option<f64> {
    (clfl
        && primitive_noun.eq_ignore_ascii_case("SCYL")
        && member_noun.eq_ignore_ascii_case("ATTA"))
    .then(|| {
        if orientation
            .and_then(|values| values.first())
            .is_some_and(|angle| *angle < 0.0)
        {
            std::f64::consts::FRAC_PI_2
        } else {
            -std::f64::consts::FRAC_PI_2
        }
    })
}

fn evaluate_catalogue_cylinder(
    member: &DbElement,
    component: &DbElement,
    cylinder: &DbElement,
    env: &DbElementParamEnv,
    noun: &str,
) -> Result<Option<EvaluatedCylinder>, ModelError> {
    let diameter = required_number(cylinder, "PDIA", env, "catalogue cylinder")?.abs();
    let mut axis = catalogue_axis_or_fail(
        member,
        component,
        cylinder,
        "PAXI",
        "catalogue cylinder axis",
    )?;
    let (height, start_distance, centre_distance) = if noun == "SCYL" {
        let mut height = required_number(cylinder, "PHEI", env, "catalogue SCYL")?;
        let mut distance = required_number(cylinder, "PDIS", env, "catalogue SCYL")?;
        if height <= 0.0 {
            height = -height;
            distance = -distance;
            axis.direction = -axis.direction;
        }
        (height, distance, distance + height * 0.5)
    } else {
        let top = required_number(cylinder, "PTDI", env, "catalogue LCYL")?;
        let bottom = required_number(cylinder, "PBDI", env, "catalogue LCYL")?;
        let centre = (bottom + top) * 0.5;
        ((bottom - top).abs(), centre, centre)
    };
    // 零高与零径同一口径（`disc_at` / SSLC 那几条臂早就这么判）：都是 Core 不出体的合法
    // 参数化关法，归零高或退化。零径此前漏判，放行到 `csg_cylinder_solid` 撞 `Degenerate`
    // 被记成局部失败——ams7997 FLEX 的 40 只 `PDIA=0` LCYL 一库就是 480 条。
    if height < HEIGHT_TOL_MM || diameter < HEIGHT_TOL_MM {
        return Ok(None);
    }
    let frame = orimat(axis.direction).ok_or_else(|| {
        ModelError::Solid(SolidError::Degenerate {
            what: "catalogue cylinder axis",
            detail: format!("PAXI={:?}", axis.direction),
        })
    })?;
    // CLFL display programs use the RVM centre-line frame: PAXI is their datum
    // direction while the cylinder runs along frame Y.  NPPIPE ATTA is the
    // proven exception: RVM 24383/73952 and /73953 keep PDIS on PAXI but use
    // the member's signed orientation to choose which half-axis the attachment
    // occupies.  Applying the generic CLFL centre offset moves both by 100 mm.
    let member_noun = member.element_type()?;
    let clfl = cylinder.get_bool("CLFL")? == Some(true);
    let (rotation, centre) = if uses_clfl_centreline_frame(&member_noun, noun, clfl) {
        let rotation = frame * DMat3::from_rotation_x(-std::f64::consts::FRAC_PI_2);
        (
            rotation,
            axis.origin + axis.direction * centre_distance + rotation.z_axis * (height * 0.5),
        )
    } else if let Some(roll) = atta_clfl_roll(
        &member_noun,
        noun,
        clfl,
        member.get_double_array("ORI")?.as_deref(),
    ) {
        let rotation = frame * DMat3::from_rotation_x(roll);
        (
            rotation,
            axis.origin + axis.direction * start_distance + rotation.z_axis * (height * 0.5),
        )
    } else {
        (frame, axis.origin + axis.direction * centre_distance)
    };
    Ok(Some(EvaluatedCylinder {
        mesh_key: CatalogueMeshKey::Cylinder {
            tessellation_version: CYLINDER_TESSELLATION_VERSION,
            diameter_bits: finite_bits(diameter, "catalogue cylinder diameter")?,
            height_bits: finite_bits(height, "catalogue cylinder height")?,
        },
        diameter,
        height,
        transform: DMat4::from_translation(centre) * DMat4::from_mat3(rotation),
    }))
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
    let id = cache.local_meshes.len();
    cache.local_meshes.insert(
        key.clone(),
        CachedCatalogueMesh {
            id,
            solid: build()?,
        },
    );
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
    Ok(Some(EvaluatedSphere {
        mesh_key: CatalogueMeshKey::Sphere {
            tessellation_version: SPHERE_TESSELLATION_VERSION,
            segments,
        },
        transform: DMat4::from_translation(centre) * DMat4::from_scale(DVec3::splat(radius)),
    }))
}

// ---------------------------------------------------------------------------
// G0-1c 低频臂：SSLC 斜切圆柱 / SDSH 碟形封头 / SDIS 圆盘。
//
// 三条 pplib 求值器（QSSLCY / QSDISH / QSDISC）与 CRCATI 对应的 case 臂都反编译在案，
// 属性名从 `unk_10B6F5xx/6xx/8xx` 的 27 进制哈希反查（PDIS/PHEI/PDIA/PRAD/PAXI/
// PXTS/PYTS/PXBS/PYBS），不是按形参猜的。形体本身见 `solid.rs` 同名 builder。
// ---------------------------------------------------------------------------

/// 取网格 → 放置去重 → 摆放。G0-1c 三条臂共用这一份；其余臂的手抄版见计划 G2-5。
fn place_cached_part(
    cache: &mut CatalogueMeshCache,
    placed_parts: &mut HashSet<PlacedPartKey>,
    solids: &mut Vec<Manifold>,
    mesh_key: &CatalogueMeshKey,
    transform: DMat4,
    build: impl FnOnce() -> Result<Manifold, ModelError>,
) -> Result<bool, ModelError> {
    let mesh_id = cached_mesh_id(cache, mesh_key, build)?;
    let placed_key = PlacedPartKey {
        mesh_id,
        transform_bits: matrix_bits(transform),
    };
    if !placed_parts.insert(placed_key) {
        cache.stats.duplicate_placed_parts += 1;
        return Ok(false);
    }
    cache.stats.unique_placed_parts += 1;
    let local_mesh = &cache
        .local_meshes
        .get(mesh_key)
        .expect("catalogue mesh inserted above")
        .solid;
    solids.push(placed(local_mesh, transform));
    Ok(true)
}

fn local_failure(primitive: &DbElement, error: &ModelError) -> String {
    format!(
        "catalogue primitive {} skipped by Core-style local failure: {error}",
        primitive.refno()
    )
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
    let (rotation, centre, centre_radius, angle_deg) = torus_frame(pa, pb)?;
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

fn torus_frame(
    pa: crate::catalogue_point::CatalogueAxis,
    pb: crate::catalogue_point::CatalogueAxis,
) -> Result<(DMat3, DVec3, f64, f64), ModelError> {
    let a = pa.direction.normalize_or_zero();
    let mut y = -pb.direction.normalize_or_zero();
    let mut z = a.cross(pb.direction).normalize_or_zero();
    let mut x = y.cross(z).normalize_or_zero();
    let cosine = a.dot(y);
    let (centre, radius, angle) = if cosine.abs() > 0.99999 {
        let delta = pb.origin - pa.origin;
        x = delta.normalize_or_zero();
        if x.length_squared() <= AXIS_TOL || x.dot(y).abs() > 1.0e-5 {
            return Err(ModelError::Solid(SolidError::Degenerate {
                what: "catalogue torus axes",
                detail: format!("parallel PAAX/PBAX with delta={delta:?}"),
            }));
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
    Ok((
        DMat3::from_cols(x, y, z),
        centre,
        radius,
        angle.to_degrees(),
    ))
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
        // NSRE 与正体 SREV 同一条建体路（revolve_rings），无过切（§2.3）。
        EvaluatedCatalogueCut::Revolution {
            rings,
            angle_deg,
            transform,
        } => (revolve_rings(rings.clone(), *angle_deg)?, *transform),
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

/// SREV 局部网格：轮廓绕局部 X 回转，再扣掉名下的成员级刀集。
fn build_srev(evaluated: &EvaluatedSrev) -> Result<Manifold, ModelError> {
    let mut body = revolve_rings(evaluated.rings.clone(), evaluated.angle_deg)?;
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
    if outcome.value().is_none() && raw_zero_tuple(element, name) {
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
        .or_else(|| raw_zero_tuple(element, name).then_some(0.0))
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
    use super::*;

    #[test]
    fn atta_clfl_cylinder_keeps_the_catalogue_axis() {
        assert!(!uses_clfl_centreline_frame("ATTA", "SCYL", true));
        assert!(uses_clfl_centreline_frame("HELE", "SCYL", true));
        assert!(uses_clfl_centreline_frame("PCLA", "SCYL", true));
        assert!(!uses_clfl_centreline_frame("HELE", "LCYL", true));
        assert!(!uses_clfl_centreline_frame("HELE", "SCYL", false));
        assert_eq!(
            atta_clfl_roll("ATTA", "SCYL", true, Some(&[-90.0, 0.0, 90.0])),
            Some(std::f64::consts::FRAC_PI_2)
        );
        assert_eq!(
            atta_clfl_roll("ATTA", "SCYL", true, Some(&[90.0, 0.0, 90.0])),
            Some(-std::f64::consts::FRAC_PI_2)
        );
    }

    #[test]
    fn atta_mixed_clfl_program_keeps_independent_components() {
        assert!(preserves_catalogue_components("ATTA", true, false));
        assert!(!preserves_catalogue_components("VALV", true, false));
        assert!(preserves_catalogue_components("HELE", true, true));
        assert!(!preserves_catalogue_components("ATTA", false, true));
    }

    #[test]
    fn negative_lsnout_span_reparameterizes_ends_without_reversing_paax() {
        assert_eq!(
            normalize_lsnout_span(84.42, 66.33, 115.2, 0.0, 12.06),
            (66.33, 84.42, 115.2, -12.06)
        );

        let (centre, rotation) = lsnout_frame(
            crate::catalogue_point::CatalogueAxis {
                origin: DVec3::new(-144.0, 0.0, 0.0),
                direction: DVec3::X,
            },
            crate::catalogue_point::CatalogueAxis {
                origin: DVec3::ZERO,
                direction: DVec3::Z,
            },
            129.6,
            14.4,
            -12.06,
        )
        .unwrap();
        assert!(centre.abs_diff_eq(DVec3::new(-72.0, 0.0, -6.03), 1.0e-9));
        assert!(rotation.z_axis.abs_diff_eq(DVec3::X, 1.0e-9));

        let (centre, rotation) = lsnout_frame(
            crate::catalogue_point::CatalogueAxis {
                origin: DVec3::new(144.0, 0.0, 0.0),
                direction: -DVec3::X,
            },
            crate::catalogue_point::CatalogueAxis {
                origin: DVec3::ZERO,
                direction: -DVec3::Z,
            },
            129.6,
            14.4,
            -12.06,
        )
        .unwrap();
        assert!(centre.abs_diff_eq(DVec3::new(72.0, 0.0, 6.03), 1.0e-9));
        assert!(rotation.z_axis.abs_diff_eq(-DVec3::X, 1.0e-9));
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

    /// 整圈 SREV 的矩形轮廓必须与矩形环面**逐位**同体积——回转轴、段数口径
    /// 与设计侧回转是同一条路，这条一红不是轴摆错就是段数分了岔。
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
        let torus = rect_torus_solid(300.0, 500.0, 200.0, 360.0).unwrap();
        assert!(
            (body.volume() - torus.volume()).abs() / torus.volume() < 1e-12,
            "SREV {} vs 环面 {}",
            body.volume(),
            torus.volume()
        );
    }

    /// NSRE 刀从 SREV 上扣掉的必须恰好是它自己的回转体：两边同走
    /// `revolve_rings`、同一外半径定段数，径向 400..500 的整圈刀贯穿轴向后，
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
        let full = revolve_rings(vec![outer], 360.0).unwrap();
        let removed = revolve_rings(
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
}

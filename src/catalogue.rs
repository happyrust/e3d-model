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

use crate::catalogue_point::{catalogue_axis, catalogue_component};
use crate::discretise::{FilletVertex, flatten_ring};
use crate::elmodl::{GeneratedElement, GeometryId, ModelError};
use crate::primitive_instance::instantiate_box;
use crate::route::orimat;
use crate::solid::{
    SolidError, box_solid, circ_torus_solid, csg_cylinder_solid, cylinder_segments,
    dish_knuckle_radius, elliptical_dish_solid, extrude_rings, placed, pyramid_solid,
    rect_torus_solid, slope_ended_cylinder_solid, snout_solid_with_segments,
    spherical_dish_solid, unit_sphere_solid,
};
use crate::transform::world_matrix;

const GEOMETRY_REF: &str = "GMRE";
const AXIS_TOL: f64 = 1.0e-9;
const HEIGHT_TOL_MM: f64 = 1.0e-6;
const SEXT_TESSELLATION_VERSION: u32 = 1;
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

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
enum CatalogueMeshKey {
    Sext(SextMeshKey),
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
}

/// Result of evaluating one design member's catalogue program.
pub struct CatalogueMemberBuild {
    pub generated: Option<GeneratedElement>,
    /// Core rejects one bad catalogue primitive and continues the GMRE.  Keep
    /// those local failures visible instead of turning the whole member into a
    /// false failure or silently dropping them.
    pub diagnostics: Vec<String>,
}

/// 表现级（`REPRE LEVEL`）。几何集成员的 `LEVE=[lo,hi]` 与它比较决定画不画。
///
/// 默认 6：E3D GUI 交互与 `GenModelRvmExport` addin 导出（`GENMODEL_RVM_LEVEL=6`）
/// 都在这一级。TTY 裸会话实测在 0 级（`1RX07-LCT.rvm`：同一目录件族只出
/// `LEVE=[0,0]` 的 CABLEWAY 盒，`[1,10]` 的 BOARD/RESERVED 全不在）。
/// 对拍不同口径的基准时用 `E3D_REPRE_LEVEL` 覆盖本轮进程。
///
/// 设计侧负体的 DRWAND 门（`elmodl::design_leve_admits`）读的也是这一个数——Core3D 里
/// 两条路共用同一组 `rules/STYSET` 全局，这里就不许长出第二份「当前表现级」。
pub(crate) fn repre_level() -> u32 {
    static LEVEL: std::sync::OnceLock<u32> = std::sync::OnceLock::new();
    *LEVEL.get_or_init(|| {
        std::env::var("E3D_REPRE_LEVEL")
            .ok()
            .and_then(|value| value.trim().parse().ok())
            .unwrap_or(6)
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
/// **仍未消化的反例**：`C-IY` 基准里 `OBST=1/TUFL=false` 的 RESERVED 盒被 E3D 导出。
/// 那份是 addin 导出，导出口径（`REPR OBST`）可能与 ams7997 这批 TTY 导出不同；
/// 对拍托盘语料时若 RESERVED 盒对不上，先查那次导出的 REPR 设置，别回头改这里。
/// HANG 24381/177986 的 HELEMENT 把 `CLFL=true` 的三个 SCYL 和一个 SEXT 全部导出——
/// 它们 `TUFL=true`，本筛照画；"CLFL 即隐藏"仍然是错的读法。
fn display_filtered(primitive: &DbElement) -> Result<Option<&'static str>, ModelError> {
    if let Some(DescriptorValue::RawWords(words) | DescriptorValue::WordArray(words)) =
        primitive.attribute("LEVE")?
        && let (Some(&lo), Some(&hi)) = (words.first(), words.get(1))
    {
        let level = repre_level();
        if lo > level || (hi != 0 && hi < level) {
            return Ok(Some("leve"));
        }
    }
    let hard_obstruction = matches!(
        primitive.attribute("OBST")?,
        Some(DescriptorValue::Word { raw: 2, .. }) | Some(DescriptorValue::Int(2))
    );
    if primitive.get_bool("TUFL")? != Some(false) {
        return Ok(None);
    }
    Ok(Some(if hard_obstruction {
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
        return Ok(CatalogueMemberBuild {
            generated: None,
            diagnostics: Vec::new(),
        });
    };
    let Some(geometry) = component.get_element(GEOMETRY_REF)? else {
        return Ok(CatalogueMemberBuild {
            generated: None,
            diagnostics: Vec::new(),
        });
    };
    let env =
        DbElementParamEnv::new(Some(member.clone()), Some(component.clone())).without_insulation();
    let mut solids = Vec::new();
    let mut ignored_lines = 0usize;
    let mut sext_total = 0usize;
    let mut zero_height = 0usize;
    let mut placed_parts = HashSet::new();
    let mut member_duplicates = 0usize;
    let mut catalogue_negatives = 0usize;
    // 过滤记录与 diagnostics 分开收：`failed` 的账目 = diagnostics.len()，
    // 被表现级筛掉不是失败，不能混进去。
    let mut display_filter_notes = Vec::new();
    let mut diagnostics = Vec::new();
    let mut saw_display_primitive = false;
    let mut all_display_primitives_are_clfl = true;

    for primitive in geometry.members()? {
        let primitive_noun = primitive.element_type()?;
        // LINE 成员本来就只记账不建体，不进显示过滤，保住 centre_lines 的账目口径。
        if primitive_noun != "LINE"
            && let Some(reason) = display_filtered(&primitive)?
        {
            display_filter_notes.push(format!(
                "catalogue primitive {} noun {primitive_noun} display-filtered by {reason} (repre level {})",
                primitive.refno(),
                repre_level(),
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
                    Ok(None) => {}
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
                    Ok(None) => {}
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
                    Ok(None) => {}
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
                    Ok(None) => {}
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
                    Ok(None) => {}
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
                    Ok(None) => {}
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
                    Ok(None) => {}
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
                    Ok(None) => {}
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
                    Ok(None) => {}
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
        diagnostics.extend(display_filter_notes);
        return Ok(CatalogueMemberBuild {
            generated: None,
            diagnostics,
        });
    }

    let local = Manifold::compose(&solids);
    if local.is_empty() || local.num_tri() == 0 {
        return Err(ModelError::Solid(SolidError::EmptyManifold {
            what: "catalogue assembly",
        }));
    }
    let world = world_matrix(member)?;
    // E3D keeps a CLFL display program as independent RVM components even when
    // its primitives overlap.  Require the whole visible program to be CLFL so
    // a mixed catalogue solid does not accidentally bypass its boolean compose.
    let preserve_components = saw_display_primitive && all_display_primitives_are_clfl;
    let parts = preserve_components.then(|| {
        solids
            .iter()
            .map(|solid| placed(solid, world))
            .collect::<Vec<_>>()
    });
    let noun = member.element_type()?;
    let notes = vec![format!(
        "catalogue assembly: built={} sext_total={} sext_zero_height={} failed={} centre_lines={} display_filtered={} duplicate_placements={} component={}",
        solids.len(),
        sext_total,
        zero_height,
        diagnostics.len(),
        ignored_lines,
        display_filtered_members,
        member_duplicates,
        component.refno()
    )];
    diagnostics.extend(display_filter_notes);
    Ok(CatalogueMemberBuild {
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

    let mut pa = catalogue_axis(member, component, &required_words(pyramid, "PAAX")?)?
        .ok_or_else(|| missing(pyramid, "PAAX", "catalogue LPYR axis"))?;
    let pb = catalogue_axis(member, component, &required_words(pyramid, "PBAX")?)?
        .ok_or_else(|| missing(pyramid, "PBAX", "catalogue LPYR axis"))?;
    let pc = catalogue_axis(member, component, &required_words(pyramid, "PCAX")?)?
        .ok_or_else(|| missing(pyramid, "PCAX", "catalogue LPYR axis"))?;

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
        let mut axis = catalogue_axis(member, component, &required_words(snout, "PAXI")?)?
            .ok_or_else(|| missing(snout, "PAXI", "catalogue SCON axis"))?;
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
        let pa = catalogue_axis(member, component, &required_words(snout, "PAAX")?)?
            .ok_or_else(|| missing(snout, "PAAX", "catalogue LSNO axis"))?;
        let pb = catalogue_axis(member, component, &required_words(snout, "PBAX")?)?
            .ok_or_else(|| missing(snout, "PBAX", "catalogue LSNO offset axis"))?;
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
    let mut axis = catalogue_axis(member, component, &required_words(pyramid, "PAAX")?)?
        .ok_or_else(|| missing(pyramid, "PAAX", "catalogue NLPY axis"))?;
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

    let mut rings = Vec::new();
    for loop_element in sext.members()? {
        if !loop_element.element_type()?.eq_ignore_ascii_case("SLOO") {
            continue;
        }
        let mut vertices = Vec::new();
        for vertex in loop_element.members()? {
            if !vertex.element_type()?.eq_ignore_ascii_case("SVER") {
                continue;
            }
            vertices.push(FilletVertex {
                x: required_number(&vertex, "PX", env, "catalogue SEXT vertex")?,
                y: required_number(&vertex, "PY", env, "catalogue SEXT vertex")?,
                frad: optional_number(&vertex, "PRAD", env).unwrap_or(0.0).abs(),
            });
        }
        let Some((ring, _notes)) = flatten_ring(&vertices) else {
            return Err(ModelError::OuterRingCollapsed {
                refno: sext.refno().to_string(),
                noun: "SEXT".into(),
            });
        };
        rings.push(ring);
    }
    if rings.is_empty() {
        return Err(ModelError::OuterRingCollapsed {
            refno: sext.refno().to_string(),
            noun: "SEXT".into(),
        });
    }

    let pa = catalogue_axis(member, component, &required_words(sext, "PAAX")?)?
        .ok_or_else(|| missing(sext, "PAAX", "catalogue SEXT axis"))?;
    let pb = catalogue_axis(member, component, &required_words(sext, "PBAX")?)?
        .ok_or_else(|| missing(sext, "PBAX", "catalogue SEXT axis"))?;
    let x = pa.direction.normalize_or_zero();
    let z = x.cross(pb.direction).normalize_or_zero();
    if x.length_squared() <= AXIS_TOL || z.length_squared() <= AXIS_TOL {
        return Err(ModelError::Solid(SolidError::Degenerate {
            what: "catalogue SEXT axes",
            detail: format!("PAAX={:?}, PBAX={:?}", pa.direction, pb.direction),
        }));
    }
    // Core ORTHOG keeps the first axis and recomputes the second from the
    // completed right-handed third axis.
    let y = z.cross(x).normalize();
    let rotation = DMat3::from_cols(x, y, z);
    let raw_position = DVec3::new(
        required_number(sext, "PX", env, "catalogue SEXT position")?,
        required_number(sext, "PY", env, "catalogue SEXT position")?,
        required_number(sext, "PZ", env, "catalogue SEXT position")?,
    );
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
                Ok(cutter) => {
                    cache.stats.evaluated_nscy += 1;
                    cutters.push(EvaluatedCatalogueCut::Cylinder(cutter));
                }
                Err(error) => diagnostics.push(format!(
                    "catalogue negative {} skipped by Core-style local failure: {error}",
                    child.refno()
                )),
            }
        } else if noun.eq_ignore_ascii_case("NSBO") {
            cache.stats.source_nsbo += 1;
            match evaluate_nsbo(&child, env, position) {
                Ok(cutter) => {
                    cache.stats.evaluated_nsbo += 1;
                    cutters.push(cutter);
                }
                Err(error) => diagnostics.push(format!(
                    "catalogue negative {} skipped by Core-style local failure: {error}",
                    child.refno()
                )),
            }
        } else if noun.eq_ignore_ascii_case("NLCY") {
            cache.stats.source_nlcy += 1;
            match evaluate_nlcy(member, component, &child, env, position) {
                Ok(cutter) => {
                    cache.stats.evaluated_nlcy += 1;
                    cutters.push(EvaluatedCatalogueCut::Cylinder(cutter));
                }
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
    let cutter_keys = cutters
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
        })
        .collect::<Result<Vec<_>, ModelError>>()?;
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
) -> Result<EvaluatedNscy, ModelError> {
    let diameter = required_number(nlcy, "PDIA", env, "catalogue NLCY")?.abs();
    let top = required_number(nlcy, "PTDI", env, "catalogue NLCY")?;
    let bottom = required_number(nlcy, "PBDI", env, "catalogue NLCY")?;
    let height = (bottom - top).abs();
    if height < HEIGHT_TOL_MM {
        return Err(ModelError::Solid(SolidError::Degenerate {
            what: "catalogue NLCY",
            detail: format!("PTDI/PBDI={top}/{bottom}"),
        }));
    }
    let axis = catalogue_axis(member, component, &required_words(nlcy, "PAXI")?)?
        .ok_or_else(|| missing(nlcy, "PAXI", "catalogue NLCY axis"))?;
    let rotation = orimat(axis.direction).ok_or_else(|| {
        ModelError::Solid(SolidError::Degenerate {
            what: "catalogue NLCY axis",
            detail: format!("PAXI={:?}", axis.direction),
        })
    })?;
    let centre = axis.origin + axis.direction * ((bottom + top) * 0.5) - sbox_position;
    Ok(EvaluatedNscy {
        diameter,
        height,
        transform: DMat4::from_translation(centre) * DMat4::from_mat3(rotation),
    })
}

fn evaluate_nsbo(
    nsbo: &DbElement,
    env: &DbElementParamEnv,
    sbox_position: DVec3,
) -> Result<EvaluatedCatalogueCut, ModelError> {
    let dimensions = DVec3::new(
        required_number(nsbo, "PXLE", env, "catalogue NSBO")?.abs(),
        required_number(nsbo, "PYLE", env, "catalogue NSBO")?.abs(),
        required_number(nsbo, "PZLE", env, "catalogue NSBO")?.abs(),
    );
    let position = DVec3::new(
        required_number(nsbo, "PX", env, "catalogue NSBO position")?,
        required_number(nsbo, "PY", env, "catalogue NSBO position")?,
        required_number(nsbo, "PZ", env, "catalogue NSBO position")?,
    );
    Ok(EvaluatedCatalogueCut::Box {
        dimensions,
        transform: DMat4::from_translation(position - sbox_position),
    })
}

fn evaluate_nscy(
    member: &DbElement,
    component: &DbElement,
    nscy: &DbElement,
    env: &DbElementParamEnv,
    sbox_position: DVec3,
) -> Result<EvaluatedNscy, ModelError> {
    let diameter = required_number(nscy, "PDIA", env, "catalogue NSCY")?.abs();
    let mut height = required_number(nscy, "PHEI", env, "catalogue NSCY")?;
    let mut distance = required_number(nscy, "PDIS", env, "catalogue NSCY")?;
    let mut axis = catalogue_axis(member, component, &required_words(nscy, "PAXI")?)?
        .ok_or_else(|| missing(nscy, "PAXI", "catalogue NSCY axis"))?;
    if height <= 0.0 {
        height = -height;
        distance = -distance;
        axis.direction = -axis.direction;
    }
    if height < HEIGHT_TOL_MM {
        return Err(ModelError::Solid(SolidError::Degenerate {
            what: "catalogue NSCY",
            detail: format!("PHEI={height}"),
        }));
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
    Ok(EvaluatedNscy {
        diameter,
        height: height + NSCY_AXIAL_OVERCUT_MM,
        transform: DMat4::from_translation(centre) * DMat4::from_mat3(rotation),
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
    let mut axis = catalogue_axis(member, component, &required_words(cylinder, "PAXI")?)?
        .ok_or_else(|| missing(cylinder, "PAXI", "catalogue cylinder axis"))?;
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
        ((bottom - top).abs(), (bottom + top) * 0.5)
    };
    if height < HEIGHT_TOL_MM {
        return Ok(None);
    }
    let frame = orimat(axis.direction).ok_or_else(|| {
        ModelError::Solid(SolidError::Degenerate {
            what: "catalogue cylinder axis",
            detail: format!("PAXI={:?}", axis.direction),
        })
    })?;
    // CLFL cylinders are exported by E3D as centre-line primitives: PAXI is
    // the frame Z/datum direction, while the RVM Cylinder's longitudinal axis
    // is that frame's Y.  Ordinary catalogue solids keep the historical
    // QSCYLI convention where PAXI is the solid axis itself.
    let (rotation, centre) = if cylinder.get_bool("CLFL")? == Some(true) && noun == "SCYL" {
        let rotation = frame * DMat3::from_rotation_x(-std::f64::consts::FRAC_PI_2);
        (
            rotation,
            axis.origin + axis.direction * centre_distance + rotation.z_axis * (height * 0.5),
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
    let axis = catalogue_axis(member, component, &required_words(sphere, "PAXI")?)?
        .ok_or_else(|| missing(sphere, "PAXI", "catalogue sphere axis"))?;
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
    let axis = catalogue_axis(member, component, &required_words(disc, "PAXI")?)?
        .ok_or_else(|| missing(disc, "PAXI", "catalogue SDIS axis"))?;
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
    let axis = catalogue_axis(member, component, &required_words(dish, "PAXI")?)?
        .ok_or_else(|| missing(dish, "PAXI", "catalogue SDSH axis"))?;
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
        return Ok(disc_at(base_centre, direction, diameter, "catalogue SDSH axis")?
            .map(EvaluatedSdsh::Disc));
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
    let axis = catalogue_axis(member, component, &required_words(cylinder, "PAXI")?)?
        .ok_or_else(|| missing(cylinder, "PAXI", "catalogue SSLC axis"))?;
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
    if shear_deg.iter().any(|angle| !angle.is_finite() || angle.abs() >= 90.0) {
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
    let pa = catalogue_axis(member, component, &required_words(torus, "PAAX")?)?
        .ok_or_else(|| missing(torus, "PAAX", "catalogue torus axis"))?;
    let pb = catalogue_axis(member, component, &required_words(torus, "PBAX")?)?
        .ok_or_else(|| missing(torus, "PBAX", "catalogue torus axis"))?;
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
        // The shared torus builders rotate Manifold's native Z-axis revolve
        // into their public local-Y convention. Core's catalogue frame is the
        // native primitive frame, so cancel that adapter before placement.
        // AMS7997 24381/104871 otherwise lands one centre-radius away from its
        // RVM CircularTorus despite having identical radius and sweep.
        transform: DMat4::from_translation(centre)
            * DMat4::from_mat3(rotation)
            * DMat4::from_rotation_x(-std::f64::consts::FRAC_PI_2),
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

fn build_sbox(evaluated: &EvaluatedSbox) -> Result<Manifold, ModelError> {
    let mut body = box_solid(
        evaluated.dimensions.x,
        evaluated.dimensions.y,
        evaluated.dimensions.z,
    )?;
    for cutter in &evaluated.cutters {
        let (local, transform) = match cutter {
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
        };
        body = &body - &placed(&local, transform);
    }
    if body.is_empty() || body.num_tri() == 0 {
        return Err(ModelError::Solid(SolidError::EmptyManifold {
            what: "catalogue SBOX after negatives",
        }));
    }
    Ok(body)
}

fn sext_mesh_key(rings: &[Vec<[f64; 2]>], height: f64) -> Result<SextMeshKey, ModelError> {
    let mut ring_lengths = Vec::with_capacity(rings.len());
    let mut coordinate_bits = Vec::new();
    for ring in rings {
        ring_lengths.push(ring.len());
        for point in ring {
            coordinate_bits.push(finite_bits(point[0], "catalogue SEXT profile")?);
            coordinate_bits.push(finite_bits(point[1], "catalogue SEXT profile")?);
        }
    }
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

fn required_number(
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

fn optional_number(element: &DbElement, name: &str, env: &DbElementParamEnv) -> Option<f64> {
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

#[cfg(test)]
mod tests {
    use super::*;

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
        assert!(bb.min()[2].abs() < 1e-9 && (bb.max()[2] - 200.0).abs() < 1e-9, "底在 z=0、顶在 z=h");
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

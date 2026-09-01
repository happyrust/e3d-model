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
use e3d_io::record::descriptor::DescriptorValue;

use crate::catalogue_point::{catalogue_axis, catalogue_component};
use crate::discretise::{FilletVertex, flatten_ring};
use crate::elmodl::{GeneratedElement, GeometryId, ModelError};
use crate::primitive_instance::instantiate_box;
use crate::route::orimat;
use crate::solid::{
    SolidError, box_solid, circ_torus_solid, csg_cylinder_solid, cylinder_segments, extrude_rings,
    placed, pyramid_solid, rect_torus_solid, snout_solid, unit_sphere_solid,
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
}

/// Result of evaluating one design member's catalogue program.
pub struct CatalogueMemberBuild {
    pub generated: Option<GeneratedElement>,
    /// Core rejects one bad catalogue primitive and continues the GMRE.  Keep
    /// those local failures visible instead of turning the whole member into a
    /// false failure or silently dropping them.
    pub diagnostics: Vec<String>,
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
    let mut diagnostics = Vec::new();

    for primitive in geometry.members()? {
        let primitive_noun = primitive.element_type()?;
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
                            Ok(snout_solid(dbot, dtop, height, xoff, yoff)?)
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
            // GMRE LINE entries are centreline metadata, not visible solids.
            "LINE" => ignored_lines += 1,
            unsupported => diagnostics.push(format!(
                "catalogue primitive {} noun {unsupported} is not implemented",
                primitive.refno()
            )),
        }
    }
    if solids.is_empty() {
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
    let noun = member.element_type()?;
    let notes = vec![format!(
        "catalogue assembly: built={} sext_total={} sext_zero_height={} failed={} centre_lines={} duplicate_placements={} component={}",
        solids.len(),
        sext_total,
        zero_height,
        diagnostics.len(),
        ignored_lines,
        member_duplicates,
        component.refno()
    )];
    Ok(CatalogueMemberBuild {
        generated: Some(GeneratedElement {
            geometry_id: GeometryId::Element {
                refno: member.refno().to_string(),
            },
            refno: member.refno(),
            noun,
            name: member.stored_name()?,
            solid: placed(&local, world),
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
    if pyramid.get_bool("TUFL")? == Some(false) {
        return Ok(None);
    }
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
    if snout.get_bool("TUFL")? == Some(false) {
        return Ok(None);
    }

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
        let mut dbot = required_number(snout, "PBDM", env, "catalogue LSNO")?.abs();
        let mut dtop = required_number(snout, "PTDM", env, "catalogue LSNO")?.abs();
        let bottom = required_number(snout, "PBDI", env, "catalogue LSNO")?;
        let top = required_number(snout, "PTDI", env, "catalogue LSNO")?;
        let mut offset = required_number(snout, "POFF", env, "catalogue LSNO")?;
        let mut pa = catalogue_axis(member, component, &required_words(snout, "PAAX")?)?
            .ok_or_else(|| missing(snout, "PAAX", "catalogue LSNO axis"))?;
        let pb = catalogue_axis(member, component, &required_words(snout, "PBAX")?)?
            .ok_or_else(|| missing(snout, "PBAX", "catalogue LSNO offset axis"))?;
        let mut height = top - bottom;
        if height < 0.0 {
            height = -height;
            pa.direction = -pa.direction;
            offset = -offset;
            std::mem::swap(&mut dbot, &mut dtop);
        }
        let z = pa.direction.normalize_or_zero();
        let x = (pb.direction - z * pb.direction.dot(z)).normalize_or_zero();
        let y = z.cross(x).normalize_or_zero();
        if x.length_squared() <= AXIS_TOL || y.length_squared() <= AXIS_TOL {
            return Err(ModelError::Solid(SolidError::Degenerate {
                what: "catalogue LSNO axes",
                detail: format!("PAAX={:?}, PBAX={:?}", pa.direction, pb.direction),
            }));
        }
        (
            dbot,
            dtop,
            height,
            offset,
            pa.origin + z * ((bottom + top) * 0.5) + x * (offset * 0.5),
            DMat3::from_cols(x, y, z),
        )
    };
    if height < HEIGHT_TOL_MM || dbot.max(dtop) < HEIGHT_TOL_MM {
        return Ok(None);
    }
    let parameters = [dbot, dtop, height, xoff, 0.0];
    Ok(Some(EvaluatedSnout {
        mesh_key: CatalogueMeshKey::Snout {
            tessellation_version: 1,
            parameter_bits: parameters.map(|value| value.to_bits()),
        },
        parameters,
        transform: DMat4::from_translation(centre) * DMat4::from_mat3(rotation),
    }))
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
    if sext.get_bool("TUFL")? == Some(false) {
        return Ok(None);
    }
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
    if sbox.get_bool("TUFL")? == Some(false) {
        return Ok(None);
    }
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
    if cylinder.get_bool("TUFL")? == Some(false) {
        return Ok(None);
    }
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
    let rotation = orimat(axis.direction).ok_or_else(|| {
        ModelError::Solid(SolidError::Degenerate {
            what: "catalogue cylinder axis",
            detail: format!("PAXI={:?}", axis.direction),
        })
    })?;
    let centre = axis.origin + axis.direction * centre_distance;
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
    if sphere.get_bool("TUFL")? == Some(false) {
        return Ok(None);
    }
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

fn evaluate_catalogue_torus(
    member: &DbElement,
    component: &DbElement,
    torus: &DbElement,
    env: &DbElementParamEnv,
    noun: &str,
) -> Result<Option<EvaluatedTorus>, ModelError> {
    if torus.get_bool("TUFL")? == Some(false) {
        return Ok(None);
    }
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
}

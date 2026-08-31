//! Reusable canonical meshes for affine-instantiable basic primitives.
//!
//! The key contains shape/tessellation identity only. Absolute dimensions live
//! in [`PrimitiveInstance::local_transform`]. Segment counts are computed from
//! the real source dimensions before the mesh is normalised.

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, OnceLock, RwLock};

use glam::{DMat4, DVec3};
use manifold_csg::Manifold;
use serde::{Deserialize, Serialize};

use crate::solid::{SolidError, box_solid, cylinder_segments, cylinder_solid, placed};

pub const INSTANCE_CACHE_ENV: &str = "E3D_MODEL_NO_INSTANCE_CACHE";

/// Versioned identity of one canonical mesh. Do not add refno, placement, or
/// absolute dimensions here: doing so degenerates back to one mesh per element.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(tag = "family", rename_all = "snake_case")]
pub enum PrimitiveMeshKey {
    BoxV1,
    CylinderV2 { segments: i32 },
}

/// Canonical mesh placement in an element's local coordinate system.
#[derive(Debug, Clone)]
pub struct PrimitiveInstance {
    pub key: PrimitiveMeshKey,
    pub local_transform: DMat4,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize)]
pub struct PrimitiveCacheStats {
    pub hits: u64,
    pub misses: u64,
    pub bypasses: u64,
    /// Basic primitives that participate in a boolean are deliberately built
    /// at their real dimensions. Tiny rounding differences from unit scaling
    /// can change coincident-face CSG topology, so these are outside the
    /// instancing boundary.
    pub baked_operands: u64,
    pub unique_meshes: usize,
}

#[derive(Default)]
struct PrimitiveMeshCache {
    meshes: RwLock<HashMap<PrimitiveMeshKey, Arc<Manifold>>>,
    hits: AtomicU64,
    misses: AtomicU64,
    bypasses: AtomicU64,
    baked_operands: AtomicU64,
}

static CACHE: OnceLock<PrimitiveMeshCache> = OnceLock::new();

fn cache() -> &'static PrimitiveMeshCache {
    CACHE.get_or_init(PrimitiveMeshCache::default)
}

pub fn instance_cache_enabled() -> bool {
    std::env::var_os(INSTANCE_CACHE_ENV).is_none()
}

pub fn primitive_cache_stats() -> PrimitiveCacheStats {
    let cache = cache();
    PrimitiveCacheStats {
        hits: cache.hits.load(Ordering::Relaxed),
        misses: cache.misses.load(Ordering::Relaxed),
        bypasses: cache.bypasses.load(Ordering::Relaxed),
        baked_operands: cache.baked_operands.load(Ordering::Relaxed),
        unique_meshes: cache.meshes.read().expect("primitive cache poisoned").len(),
    }
}

pub fn canonical_primitive_mesh(key: PrimitiveMeshKey) -> Result<Arc<Manifold>, SolidError> {
    cached_canonical(key)
}

fn build_canonical(key: PrimitiveMeshKey) -> Result<Manifold, SolidError> {
    match key {
        PrimitiveMeshKey::BoxV1 => box_solid(1.0, 1.0, 1.0),
        PrimitiveMeshKey::CylinderV2 { segments } => {
            // Unit radius and unit height. `segments` was calculated from the
            // real radius before normalisation and is therefore part of key.
            let solid = Manifold::cylinder(1.0, 1.0, 1.0, segments, true);
            if solid.is_empty() || solid.num_tri() == 0 {
                Err(SolidError::EmptyManifold {
                    what: "canonical cylinder",
                })
            } else {
                Ok(solid)
            }
        }
    }
}

fn cached_canonical(key: PrimitiveMeshKey) -> Result<Arc<Manifold>, SolidError> {
    let cache = cache();
    if let Some(found) = cache
        .meshes
        .read()
        .expect("primitive cache poisoned")
        .get(&key)
        .cloned()
    {
        cache.hits.fetch_add(1, Ordering::Relaxed);
        return Ok(found);
    }

    let built = Arc::new(build_canonical(key)?);
    let mut meshes = cache.meshes.write().expect("primitive cache poisoned");
    match meshes.entry(key) {
        std::collections::hash_map::Entry::Occupied(entry) => {
            cache.hits.fetch_add(1, Ordering::Relaxed);
            Ok(entry.get().clone())
        }
        std::collections::hash_map::Entry::Vacant(entry) => {
            cache.misses.fetch_add(1, Ordering::Relaxed);
            entry.insert(built.clone());
            Ok(built)
        }
    }
}

pub(crate) struct PrimitiveBuild {
    pub solid: Manifold,
    pub instance: Option<PrimitiveInstance>,
}

pub(crate) fn instantiate_box(
    x: f64,
    y: f64,
    z: f64,
    allow_instance: bool,
) -> Result<PrimitiveBuild, SolidError> {
    // Keep the old validation/error text in the bypass path and validate before
    // caching so an invalid element can never create a reusable entry.
    if !allow_instance {
        cache().baked_operands.fetch_add(1, Ordering::Relaxed);
        return Ok(PrimitiveBuild {
            solid: box_solid(x, y, z)?,
            instance: None,
        });
    }
    if !instance_cache_enabled() {
        cache().bypasses.fetch_add(1, Ordering::Relaxed);
        return Ok(PrimitiveBuild {
            solid: box_solid(x, y, z)?,
            instance: None,
        });
    }
    let _ = box_solid(x, y, z)?;
    let key = PrimitiveMeshKey::BoxV1;
    let local_transform = DMat4::from_scale(DVec3::new(x, y, z));
    let canonical = cached_canonical(key)?;
    Ok(PrimitiveBuild {
        solid: placed(canonical.as_ref(), local_transform),
        instance: Some(PrimitiveInstance {
            key,
            local_transform,
        }),
    })
}

pub(crate) fn instantiate_cylinder(
    diam: f64,
    height: f64,
    allow_instance: bool,
) -> Result<PrimitiveBuild, SolidError> {
    // Keep the shared-instance path identical to the baked Core3D semantics.
    // A negative Z scale would add a reflection which the centred baked body
    // does not have; only the magnitude participates in the local geometry.
    let height = height.abs();
    if !allow_instance {
        cache().baked_operands.fetch_add(1, Ordering::Relaxed);
        return Ok(PrimitiveBuild {
            solid: cylinder_solid(diam, height)?,
            instance: None,
        });
    }
    if !instance_cache_enabled() {
        cache().bypasses.fetch_add(1, Ordering::Relaxed);
        return Ok(PrimitiveBuild {
            solid: cylinder_solid(diam, height)?,
            instance: None,
        });
    }
    // Preserve validation and calculate N from the real radius before replacing
    // the shape with the unit-radius canonical mesh.
    let _ = cylinder_solid(diam, height)?;
    let radius = diam / 2.0;
    let key = PrimitiveMeshKey::CylinderV2 {
        segments: cylinder_segments(radius),
    };
    let local_transform = DMat4::from_scale(DVec3::new(radius, radius, height));
    let canonical = cached_canonical(key)?;
    Ok(PrimitiveBuild {
        solid: placed(canonical.as_ref(), local_transform),
        instance: Some(PrimitiveInstance {
            key,
            local_transform,
        }),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_same_geometry(a: &Manifold, b: &Manifold) {
        assert_eq!(a.num_tri(), b.num_tri());
        assert_eq!(a.genus(), b.genus());
        let a_bounds = a.bounding_box().unwrap();
        let b_bounds = b.bounding_box().unwrap();
        assert_eq!(a_bounds.min(), b_bounds.min());
        assert_eq!(a_bounds.max(), b_bounds.max());
        assert_eq!(a.volume().to_bits(), b.volume().to_bits());
    }

    #[test]
    fn box_unit_instance_matches_direct_build() {
        let instanced = instantiate_box(12.0, 34.0, 56.0, true).unwrap();
        let direct = box_solid(12.0, 34.0, 56.0).unwrap();
        assert_eq!(instanced.instance.unwrap().key, PrimitiveMeshKey::BoxV1);
        assert_same_geometry(&instanced.solid, &direct);
    }

    #[test]
    fn cylinder_key_splits_only_at_segment_class() {
        let a = instantiate_cylinder(2.0, 10.0, true).unwrap();
        let b = instantiate_cylinder(4.0, 30.0, true).unwrap();
        let c = instantiate_cylinder(200.0, 30.0, true).unwrap();
        assert_eq!(
            a.instance.as_ref().unwrap().key,
            b.instance.as_ref().unwrap().key
        );
        assert_ne!(
            a.instance.as_ref().unwrap().key,
            c.instance.as_ref().unwrap().key
        );
        assert_same_geometry(&a.solid, &cylinder_solid(2.0, 10.0).unwrap());
        assert_same_geometry(&c.solid, &cylinder_solid(200.0, 30.0).unwrap());
    }

    #[test]
    fn key_json_is_stable_and_contains_segment_count() {
        let key = PrimitiveMeshKey::CylinderV2 { segments: 32 };
        assert_eq!(
            serde_json::to_string(&key).unwrap(),
            r#"{"family":"cylinder_v2","segments":32}"#
        );
    }

    #[test]
    fn boolean_operand_keeps_the_direct_constructor() {
        let built = instantiate_cylinder(200.0, 30.0, false).unwrap();
        assert!(built.instance.is_none());
        assert_same_geometry(&built.solid, &cylinder_solid(200.0, 30.0).unwrap());
    }

    #[test]
    fn negative_cylinder_height_matches_baked_and_instanced_paths() {
        let instanced = instantiate_cylinder(51.67, -183.15, true).unwrap();
        let baked = instantiate_cylinder(51.67, -183.15, false).unwrap();
        assert_eq!(
            instanced.instance.as_ref().unwrap().local_transform,
            DMat4::from_scale(DVec3::new(25.835, 25.835, 183.15))
        );
        assert_same_geometry(&instanced.solid, &baked.solid);
    }
}

//! Reusable canonical meshes for affine-instantiable basic primitives.
//!
//! The key contains shape/tessellation identity only. Absolute dimensions live
//! in [`PrimitiveInstance::local_transform`]. Segment counts are computed from
//! the real source dimensions before the mesh is normalised.

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, OnceLock, PoisonError, RwLock, RwLockReadGuard, RwLockWriteGuard};

use glam::{DMat4, DVec3};
use manifold_csg::Manifold;
use serde::{Deserialize, Serialize};

use crate::solid::{
    BOX_WHAT, CYLINDER_WHAT, SolidError, box_solid, csg_cylinder_solid, cylinder_segments,
    cylinder_solid, cylinder_tilt, ensure_solid, placed, validate_box_dims, validate_cylinder_dims,
};

pub const INSTANCE_CACHE_ENV: &str = "E3D_MODEL_NO_INSTANCE_CACHE";

/// Versioned identity of one canonical mesh. Do not add refno, placement, or
/// absolute dimensions here: doing so degenerates back to one mesh per element.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(tag = "family", rename_all = "snake_case")]
pub enum PrimitiveMeshKey {
    BoxV1,
    CylinderV3 { segments: i32 },
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

impl PrimitiveMeshCache {
    /// 中毒锁照常取用。里面是一张 `key -> canonical 网格` 的表，每个值都是
    /// [`build_canonical`] 一次性建完才插进去的纯函数产物；某个线程建网格时
    /// panic 只会让它自己那次 `insert` 没发生，不会留下半张表。为这个把整场
    /// 跑批（AMS1112 三万个元素）连坐掉，代价与风险完全不成比例。
    fn read_meshes(&self) -> RwLockReadGuard<'_, HashMap<PrimitiveMeshKey, Arc<Manifold>>> {
        self.meshes.read().unwrap_or_else(PoisonError::into_inner)
    }

    fn write_meshes(&self) -> RwLockWriteGuard<'_, HashMap<PrimitiveMeshKey, Arc<Manifold>>> {
        self.meshes.write().unwrap_or_else(PoisonError::into_inner)
    }
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
        unique_meshes: cache.read_meshes().len(),
    }
}

pub fn canonical_primitive_mesh(key: PrimitiveMeshKey) -> Result<Arc<Manifold>, SolidError> {
    cached_canonical(key)
}

fn build_canonical(key: PrimitiveMeshKey) -> Result<Manifold, SolidError> {
    match key {
        PrimitiveMeshKey::BoxV1 => box_solid(1.0, 1.0, 1.0),
        PrimitiveMeshKey::CylinderV3 { segments } => {
            // Unit radius and unit height. `segments` was calculated from the
            // real radius before normalisation and is therefore part of key.
            let solid = Manifold::cylinder(1.0, 1.0, 1.0, segments, false);
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
    if let Some(found) = cache.read_meshes().get(&key).cloned() {
        cache.hits.fetch_add(1, Ordering::Relaxed);
        return Ok(found);
    }

    let built = Arc::new(build_canonical(key)?);
    let mut meshes = cache.write_meshes();
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

/// 设计图元的**解析描述**（元素局部系），供 RVM 写出端直接落记录、不经网格识别。
///
/// 与 [`PrimitiveInstance`] 的分工：那一份是「规范网格 + 仿射」，只覆盖能仿射出来的盒子与
/// 圆柱；斜切圆柱不是圆柱的仿射像（两端是斜平面、侧壁不斜），没有规范体可放，网格识别器
/// 也认不出斜切来——只能把参数原样带到写出端。**写出端必须过自校闸**（候选的世界 AABB 对
/// 网格实算的），这里只是参数的搬运，不替它担保。
#[derive(Debug, Clone, PartialEq)]
pub enum AnalyticPrimitive {
    /// SLCY（`gm_CreateSlopeEndedCylinder`）：轴在局部 Z、沿 Z 居中，两端按
    /// `z = ±h/2 + x·tan(xs) + y·tan(ys)` 斜切。角度为**度**、已折回 (−90°, 90°)，
    /// 与 [`crate::solid::slope_ended_cylinder_solid_with_segments`] 同参同序。
    ///
    /// E3D 的 RVM 驱动对它走 DRSNO，写 9 参 `Snout[r, r, HEIG, 0, 0, XBSH, YBSH, XTSH, YTSH]`
    /// （角度转弧度、符号照抄），记录帧 = 元素世界帧。取证：`/盘梯底部梯子`（21909/51286）
    /// 表现级 6 的基准，8 只 SLCY 逐字段对上设计属性。
    SlopedCylinder {
        radius: f64,
        height: f64,
        xtop_deg: f64,
        ytop_deg: f64,
        xbot_deg: f64,
        ybot_deg: f64,
        /// 建网格时的周向段数，写出端按它算网格与解析面之间的离散化缺口。
        segments: i32,
    },
    /// 四棱台（设计 PYRA / 目录 LPYR）：`[xbot, ybot, xtop, ytop, xoff, yoff, height]`，RVM `Pyramid` 的参数序，
    /// 在图元自己的标架里（轴局部 Z、原点在两个面心的中点，与 [`crate::solid::pyramid_solid`] 同一约定）。
    ///
    /// 棱台的网格识别器认得出形状，但哪根边是「高」只能猜：平行六面体（偏心 LPYR）和楔块（底边为 0 的 PYRA）有多种等价写法，
    /// 识别器挑了斜边当轴。E3D 的 DRPYR 不认形状，写的就是属性 / 目录参数：ams7324 `/SUPPORT/FRAMES/BS/FT48B/6-SUBS1` 的
    /// 设计 PYRA（XBOT 10 / YBOT 0 / XTOP 10 / YTOP 51 / XOFF 0 / YOFF −25.5 / HEIG 51）E3D 写 `Pyramid[10, 0, 10, 51, 0, -25.5, 51]`，
    /// ams7997 风管 `/-VES-E-4-L-2501` OFST 1 的偏心 LPYR E3D 写 `Pyramid[500, 500, 500, 500, 400, 0, 800]`；我们按网格认成
    /// `[10, 72.12, 10, 0, 0, 0, 36.06]` / `[500, 894.43, 500, 894.43, 0, 223.61, 447.21]`（happyrust/e3d-model#7 第 3 类）。
    /// 几何同一只，只是写法；写出端照样过自校闸。
    Pyramid { parameters: [f64; 7] },
}

pub(crate) struct PrimitiveBuild {
    pub solid: Manifold,
    pub instance: Option<PrimitiveInstance>,
    /// 网格之外的解析身份（见 [`AnalyticPrimitive`]）；有它的图元 `instance` 一定是 `None`。
    pub analytic: Option<AnalyticPrimitive>,
}

pub(crate) fn instantiate_box(
    x: f64,
    y: f64,
    z: f64,
    allow_instance: bool,
) -> Result<PrimitiveBuild, SolidError> {
    // Keep the old validation/error text in the bypass path and validate before
    // caching so an invalid element can never create a reusable entry. The
    // check is dimensions-only: building the real-size solid just to drop it
    // made every "hit" cost more than a direct build.
    if !allow_instance {
        cache().baked_operands.fetch_add(1, Ordering::Relaxed);
        return Ok(PrimitiveBuild {
            solid: box_solid(x, y, z)?,
            instance: None,
            analytic: None,
        });
    }
    if !instance_cache_enabled() {
        cache().bypasses.fetch_add(1, Ordering::Relaxed);
        return Ok(PrimitiveBuild {
            solid: box_solid(x, y, z)?,
            instance: None,
            analytic: None,
        });
    }
    validate_box_dims(x, y, z)?;
    let key = PrimitiveMeshKey::BoxV1;
    let local_transform = DMat4::from_scale(DVec3::new(x, y, z));
    let canonical = cached_canonical(key)?;
    Ok(PrimitiveBuild {
        solid: ensure_solid(placed(canonical.as_ref(), local_transform), BOX_WHAT)?,
        instance: Some(PrimitiveInstance {
            key,
            local_transform,
        }),
        analytic: None,
    })
}

pub(crate) fn instantiate_cylinder(
    diam: f64,
    height: f64,
    allow_instance: bool,
) -> Result<PrimitiveBuild, SolidError> {
    // E3D CYLI is based at the origin and runs along signed local Y. Preserve
    // that sign in the instance transform; the canonical body runs +Z [0, 1].
    if !allow_instance {
        cache().baked_operands.fetch_add(1, Ordering::Relaxed);
        return Ok(PrimitiveBuild {
            solid: cylinder_solid(diam, height)?,
            instance: None,
            analytic: None,
        });
    }
    if !instance_cache_enabled() {
        cache().bypasses.fetch_add(1, Ordering::Relaxed);
        return Ok(PrimitiveBuild {
            solid: cylinder_solid(diam, height)?,
            instance: None,
            analytic: None,
        });
    }
    // Preserve validation and calculate N from the real radius before replacing
    // the shape with the unit-radius canonical mesh. Dimensions only. This used
    // to build the real cylinder and drop it on the floor, so a "hit" cost more
    // than the direct build it was meant to beat: R=23400 (484 segments),
    // release, 2000 iterations, hit 1.581ms against direct 1.278ms. Validating
    // without building brings the hit to 0.329ms.
    validate_cylinder_dims(diam, height)?;
    let radius = diam / 2.0;
    let key = PrimitiveMeshKey::CylinderV3 {
        segments: cylinder_segments(radius),
    };
    let local_transform = DMat4::from_rotation_x(cylinder_tilt(height))
        * DMat4::from_scale(DVec3::new(radius, radius, height.abs()));
    let canonical = cached_canonical(key)?;
    Ok(PrimitiveBuild {
        solid: ensure_solid(placed(canonical.as_ref(), local_transform), CYLINDER_WHAT)?,
        instance: Some(PrimitiveInstance {
            key,
            local_transform,
        }),
        analytic: None,
    })
}

/// Instantiate a centered, local-`Z` cylinder using the same `CylinderV3`
/// canonical body as an ordinary CYLI.
///
/// Route-derived TUBI bodies have no negative operands and differ only by
/// radius, length and placement. Keeping those dimensions in the transform
/// avoids baking one mesh per route span without adding another primitive key.
pub(crate) fn instantiate_centered_cylinder(
    diam: f64,
    height: f64,
) -> Result<PrimitiveBuild, SolidError> {
    if !instance_cache_enabled() {
        cache().bypasses.fetch_add(1, Ordering::Relaxed);
        return Ok(PrimitiveBuild {
            solid: csg_cylinder_solid(diam, height)?,
            instance: None,
            analytic: None,
        });
    }
    validate_cylinder_dims(diam, height)?;
    let radius = diam / 2.0;
    let key = PrimitiveMeshKey::CylinderV3 {
        segments: cylinder_segments(radius),
    };
    // The canonical cylinder occupies local Z=[0,1], while TUMAT expects a
    // body centered on Z=0. Translation follows scale in application order.
    let local_transform = DMat4::from_translation(DVec3::new(0.0, 0.0, -height / 2.0))
        * DMat4::from_scale(DVec3::new(radius, radius, height));
    let canonical = cached_canonical(key)?;
    Ok(PrimitiveBuild {
        solid: ensure_solid(placed(canonical.as_ref(), local_transform), CYLINDER_WHAT)?,
        instance: Some(PrimitiveInstance {
            key,
            local_transform,
        }),
        analytic: None,
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
    fn centered_route_cylinder_reuses_cylinder_v3_without_moving_its_midpoint() {
        let instanced = instantiate_centered_cylinder(200.0, 3000.0).unwrap();
        let direct = csg_cylinder_solid(200.0, 3000.0).unwrap();
        assert_eq!(
            instanced.instance.as_ref().unwrap().key,
            PrimitiveMeshKey::CylinderV3 {
                segments: cylinder_segments(100.0),
            }
        );
        assert_same_geometry(&instanced.solid, &direct);
        let bounds = instanced.solid.bounding_box().unwrap();
        assert!((bounds.min()[2] + 1500.0).abs() < 1e-9);
        assert!((bounds.max()[2] - 1500.0).abs() < 1e-9);
    }

    #[test]
    fn key_json_is_stable_and_contains_segment_count() {
        let key = PrimitiveMeshKey::CylinderV3 { segments: 32 };
        assert_eq!(
            serde_json::to_string(&key).unwrap(),
            r#"{"family":"cylinder_v3","segments":32}"#
        );
    }

    #[test]
    fn boolean_operand_keeps_the_direct_constructor() {
        let built = instantiate_cylinder(200.0, 30.0, false).unwrap();
        assert!(built.instance.is_none());
        assert_same_geometry(&built.solid, &cylinder_solid(200.0, 30.0).unwrap());
    }

    /// 实例化路不再白建一遍真实尺寸的实体，但拒绝非法尺寸的那句话必须一字不变。
    ///
    /// 校验从 `box_solid` / `cylinder_solid` 里抽了出来，这是重构唯一可能悄悄
    /// 改坏的地方：三条路（实例、烘焙、直建）报的必须还是同一句。NaN 一并钉住
    /// ——`!(x > 0.0)` 就是为了连它一起挡。
    #[test]
    fn rejected_dimensions_read_the_same_on_every_path() {
        fn refusal<T>(built: Result<T, SolidError>) -> String {
            built.err().expect("这组尺寸本来就该被拒").to_string()
        }

        for (x, y, z) in [
            (0.0, 10.0, 10.0),
            (10.0, -1.0, 10.0),
            (10.0, 10.0, f64::NAN),
        ] {
            let direct = refusal(box_solid(x, y, z));
            assert_eq!(refusal(instantiate_box(x, y, z, true)), direct);
            assert_eq!(refusal(instantiate_box(x, y, z, false)), direct);
        }
        for (d, h) in [(0.0, 10.0), (-5.0, 10.0), (10.0, 0.0), (10.0, f64::NAN)] {
            let direct = refusal(cylinder_solid(d, h));
            assert_eq!(refusal(instantiate_cylinder(d, h, true)), direct);
            assert_eq!(refusal(instantiate_cylinder(d, h, false)), direct);
        }
    }

    /// 缓存锁被毒掉之后照常出网格：一个线程 panic 不该让整场跑批陪葬。
    #[test]
    fn a_poisoned_cache_lock_does_not_take_the_run_down() {
        let killed = std::thread::spawn(|| {
            let _held = cache().write_meshes();
            panic!("deliberately poisoning the primitive cache");
        })
        .join();
        assert!(killed.is_err(), "这个线程本来就该 panic");

        let mesh = canonical_primitive_mesh(PrimitiveMeshKey::BoxV1).unwrap();
        assert!(!mesh.is_empty() && mesh.num_tri() > 0);
        assert!(primitive_cache_stats().unique_meshes >= 1);
        assert_same_geometry(
            &instantiate_box(3.0, 4.0, 5.0, true).unwrap().solid,
            &box_solid(3.0, 4.0, 5.0).unwrap(),
        );
    }

    #[test]
    fn negative_cylinder_height_matches_baked_and_instanced_paths() {
        let instanced = instantiate_cylinder(51.67, -183.15, true).unwrap();
        let baked = instantiate_cylinder(51.67, -183.15, false).unwrap();
        assert_eq!(
            instanced.instance.as_ref().unwrap().local_transform,
            DMat4::from_rotation_x(std::f64::consts::FRAC_PI_2)
                * DMat4::from_scale(DVec3::new(25.835, 25.835, 183.15))
        );
        assert_same_geometry(&instanced.solid, &baked.solid);
        let bounds = baked.solid.bounding_box().unwrap();
        assert!(
            (bounds.min()[1] + 183.15).abs() < 1e-9,
            "{:?} {:?}",
            bounds.min(),
            bounds.max()
        );
        assert!(bounds.max()[1].abs() < 1e-9);
    }
}

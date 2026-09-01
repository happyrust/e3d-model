//! 设计侧环拉伸的世代内网格复用（纯记忆化，不改变任何构造路径）。
//!
//! ams1112 全量里 4476 件生成物有 2931 件与同批某件形状完全相同（480 件
//! 300×300×25 的 PANE 是最大的一族），而设计侧挤出此前每件都重跑
//! earcut + Manifold 建体。目录件早已按「内容位模式 → 局部网格」复用
//! （[`crate::catalogue::CatalogueMeshCache`]），这里把同一纪律搬到设计侧：
//!
//! - **键 = 构造函数的全部输入**：折线化后的环组坐标位模式 + 高度 + 让刀量 +
//!   变体（GWALL 主连通片 / 普通挤出）。SJUS 下移与世界矩阵留在键外，
//!   由每个元素自己在取出的网格上应用——与目录缓存「放置留在缓存外」同款边界。
//! - **命中返回克隆**：[`Manifold`] 克隆是 C 侧句柄拷贝（内部共享），
//!   与直建同输入产物逐位一致，所以复用不可能改变输出。
//! - **非有限输入不建键**：NaN/∞ 走与无缓存路一字不差的直建分支，
//!   该报什么错照报什么错。
//! - **建体失败不入缓存**：与 [`crate::catalogue`] 的 `cached_mesh_id` 同口径，
//!   坏数据不留可复用条目。
//!
//! **布尔参与者留在真实构建路径上**（与 [`crate::primitive_instance`] 的
//! `baked_operands` 同一边界，但理由不同）：本缓存不做归一化，命中网格与
//! 直建逐位同几何，可 manifold 的布尔输出**枚举顺序**依赖运算数的内部
//! originalID——复用（共享句柄）与逐件新建的 ID 关系不同。ams1112 实测：
//! 全库 4627 件产物里，恰有 4 件带负体的 FLOOR 在复用运算数后 OBJ 三角
//! 枚举被重排（顶点值集合与全部不变量逐位不变）。枚举重排语义上无害，
//! 但共面负体的 CSG 对这类内部次序敏感是既有案底（`restol` 让刀的由来），
//! 所以负体与带负体的宿主正体一律烘焙直建，池子只服务直接成体的挤出。
//!
//! 未覆盖（审计留账，后续按证据推进）：回转体、B-rep 多面体、SBFI 套管、
//! 隐式管身——各自的构造输入面还没有像挤出这样收敛成纯值。

use std::collections::HashMap;

use manifold_csg::Manifold;

use crate::catalogue::{CatalogueMeshCache, canonical_bits};
use crate::solid::{SolidError, extrude_rings_inflated, extrude_rings_largest_component};

/// 挤出变体：与 [`crate::solid`] 的两个公开入口一一对应。
/// 选哪条路属于构造输入，必须进键——GWALL 的主连通片规则与普通挤出
/// 对同一组环产出的实体可以不同。
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum ExtrusionVariant {
    /// [`extrude_rings_largest_component`]（GWALL / 带 skipped-link 的 FLOOR）。
    LargestComponent,
    /// [`extrude_rings_inflated`]；正体 `grow = 0`，负体带让刀量。
    Inflated { grow: f64 },
}

/// 一张挤出网格的内容身份。字段全部来自构造输入的位模式，无 refno、
/// 无放置——同形状跨元素同键，这正是复用的依据。
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub(crate) struct ExtrusionMeshKey {
    tessellation_version: u32,
    largest_component_only: bool,
    grow_bits: u64,
    height_bits: u64,
    ring_lengths: Vec<usize>,
    coordinate_bits: Vec<u64>,
}

/// 挤出折线化/建体算法一旦改变（弦高、绕向规范化、earcut 版本），
/// 抬这个号让旧键整体作废——与目录缓存的 `*_TESSELLATION_VERSION` 同一纪律。
const EXTRUSION_TESSELLATION_VERSION: u32 = 1;

/// 复用统计。进 [`crate::pipeline::Report`]，与目录缓存统计同样按
/// 调用增量结算（见 `checked_delta_from` / `accumulate`）。
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, serde::Serialize)]
pub struct ExtrusionReuseStats {
    /// 直接取到既有网格的次数（省了一次 earcut + 建体）。
    pub hits: usize,
    /// 首次见到该形状、实建并入缓存的次数。
    pub misses: usize,
    /// 非有限输入绕过缓存直建的次数。不为零本身就值得看一眼数据。
    pub non_finite_bypasses: usize,
    /// 布尔参与者（负体、带负体的宿主正体）按边界烘焙直建的次数，
    /// 对应 [`crate::primitive_instance`] 的 `baked_operands`。
    pub boolean_operand_bypasses: usize,
}

impl ExtrusionReuseStats {
    pub(crate) fn checked_delta_from(self, before: Self) -> Self {
        Self {
            hits: self
                .hits
                .checked_sub(before.hits)
                .expect("extrusion reuse statistics must be monotonic"),
            misses: self
                .misses
                .checked_sub(before.misses)
                .expect("extrusion reuse statistics must be monotonic"),
            non_finite_bypasses: self
                .non_finite_bypasses
                .checked_sub(before.non_finite_bypasses)
                .expect("extrusion reuse statistics must be monotonic"),
            boolean_operand_bypasses: self
                .boolean_operand_bypasses
                .checked_sub(before.boolean_operand_bypasses)
                .expect("extrusion reuse statistics must be monotonic"),
        }
    }

    pub(crate) fn accumulate(&mut self, delta: Self) {
        self.hits += delta.hits;
        self.misses += delta.misses;
        self.non_finite_bypasses += delta.non_finite_bypasses;
        self.boolean_operand_bypasses += delta.boolean_operand_bypasses;
    }
}

/// 世代内挤出网格池。挂在 [`CatalogueMeshCache`] 上随生成批次同生共死，
/// 单独成类型只是为了把设计侧复用的代码留在本模块。
#[derive(Default)]
pub(crate) struct ExtrusionMeshStore {
    meshes: HashMap<ExtrusionMeshKey, Manifold>,
    stats: ExtrusionReuseStats,
}

impl CatalogueMeshCache {
    /// 本批次至今的挤出复用统计（含全部根的累计，调用方自行做增量）。
    pub fn extrusion_stats(&self) -> ExtrusionReuseStats {
        self.extrusions.stats
    }
}

/// 键构造。任一输入非有限返回 `None`，调用方须走直建分支。
fn extrusion_mesh_key(
    rings: &[Vec<[f64; 2]>],
    height: f64,
    variant: ExtrusionVariant,
) -> Option<ExtrusionMeshKey> {
    let (largest_component_only, grow) = match variant {
        ExtrusionVariant::LargestComponent => (true, 0.0),
        ExtrusionVariant::Inflated { grow } => (false, grow),
    };
    if !height.is_finite() || !grow.is_finite() {
        return None;
    }
    let mut ring_lengths = Vec::with_capacity(rings.len());
    let mut coordinate_bits = Vec::new();
    for ring in rings {
        ring_lengths.push(ring.len());
        for point in ring {
            if !point[0].is_finite() || !point[1].is_finite() {
                return None;
            }
            coordinate_bits.push(canonical_bits(point[0]));
            coordinate_bits.push(canonical_bits(point[1]));
        }
    }
    Some(ExtrusionMeshKey {
        tessellation_version: EXTRUSION_TESSELLATION_VERSION,
        largest_component_only,
        grow_bits: canonical_bits(grow),
        height_bits: canonical_bits(height),
        ring_lengths,
        coordinate_bits,
    })
}

/// 与今天两条直建路完全同函数同参的建体，misses 与 bypass 分支共用。
fn build_extrusion(
    rings: Vec<Vec<[f64; 2]>>,
    height: f64,
    variant: ExtrusionVariant,
) -> Result<Manifold, SolidError> {
    match variant {
        ExtrusionVariant::LargestComponent => extrude_rings_largest_component(rings, height),
        ExtrusionVariant::Inflated { grow } => extrude_rings_inflated(rings, height, grow),
    }
}

/// 设计侧挤出的唯一入口：查池 → 命中克隆 / 未中直建并入池。
///
/// `rings` 按值接收并在 miss 时**移动**进构造函数——与无缓存时代的
/// 调用形状一致，不为建键多克隆一次环组。
///
/// `reusable = false` 表示这张网格会参与布尔（负体自身，或带负体的宿主
/// 正体）：按模块文档的边界烘焙直建，既不查池也不入池。
pub(crate) fn cached_extrusion(
    cache: &mut CatalogueMeshCache,
    rings: Vec<Vec<[f64; 2]>>,
    height: f64,
    variant: ExtrusionVariant,
    reusable: bool,
) -> Result<Manifold, SolidError> {
    if !reusable {
        cache.extrusions.stats.boolean_operand_bypasses += 1;
        return build_extrusion(rings, height, variant);
    }
    let Some(key) = extrusion_mesh_key(&rings, height, variant) else {
        cache.extrusions.stats.non_finite_bypasses += 1;
        return build_extrusion(rings, height, variant);
    };
    if let Some(found) = cache.extrusions.meshes.get(&key) {
        cache.extrusions.stats.hits += 1;
        return Ok(found.clone());
    }
    let built = build_extrusion(rings, height, variant)?;
    cache.extrusions.stats.misses += 1;
    cache.extrusions.meshes.insert(key, built.clone());
    Ok(built)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pane_rings() -> Vec<Vec<[f64; 2]>> {
        vec![vec![[0.0, 0.0], [300.0, 0.0], [300.0, 300.0], [0.0, 300.0]]]
    }

    fn assert_same_geometry(a: &Manifold, b: &Manifold) {
        assert_eq!(a.num_tri(), b.num_tri());
        assert_eq!(a.genus(), b.genus());
        let a_bounds = a.bounding_box().unwrap();
        let b_bounds = b.bounding_box().unwrap();
        assert_eq!(a_bounds.min(), b_bounds.min());
        assert_eq!(a_bounds.max(), b_bounds.max());
        assert_eq!(a.volume().to_bits(), b.volume().to_bits());
    }

    /// 命中路交出的网格必须与直建逐位同几何——复用不许改输出。
    #[test]
    fn hit_returns_the_same_geometry_as_a_direct_build() {
        let mut cache = CatalogueMeshCache::default();
        let variant = ExtrusionVariant::Inflated { grow: 0.0 };
        let first = cached_extrusion(&mut cache, pane_rings(), 25.0, variant, true).unwrap();
        let second = cached_extrusion(&mut cache, pane_rings(), 25.0, variant, true).unwrap();
        let direct = extrude_rings_inflated(pane_rings(), 25.0, 0.0).unwrap();
        assert_same_geometry(&first, &direct);
        assert_same_geometry(&second, &direct);
        let stats = cache.extrusion_stats();
        assert_eq!((stats.hits, stats.misses), (1, 1));
    }

    /// 高度 / 让刀量 / 变体 / 顶点坐标，任一不同都不许撞键。
    #[test]
    fn key_splits_on_every_construction_input() {
        let mut cache = CatalogueMeshCache::default();
        let base = ExtrusionVariant::Inflated { grow: 0.0 };
        cached_extrusion(&mut cache, pane_rings(), 25.0, base, true).unwrap();
        cached_extrusion(&mut cache, pane_rings(), 30.0, base, true).unwrap();
        cached_extrusion(
            &mut cache,
            pane_rings(),
            25.0,
            ExtrusionVariant::Inflated { grow: 0.051 },
            true,
        )
        .unwrap();
        cached_extrusion(
            &mut cache,
            pane_rings(),
            25.0,
            ExtrusionVariant::LargestComponent,
            true,
        )
        .unwrap();
        let mut moved = pane_rings();
        moved[0][2] = [301.0, 300.0];
        cached_extrusion(&mut cache, moved, 25.0, base, true).unwrap();
        let stats = cache.extrusion_stats();
        assert_eq!((stats.hits, stats.misses), (0, 5));
    }

    /// ±0.0 归一：同一形状写成 -0.0 与 0.0 不许分裂成两张网格。
    #[test]
    fn negative_zero_coordinates_share_the_positive_zero_key() {
        let mut cache = CatalogueMeshCache::default();
        let variant = ExtrusionVariant::Inflated { grow: 0.0 };
        cached_extrusion(&mut cache, pane_rings(), 25.0, variant, true).unwrap();
        let mut negated = pane_rings();
        negated[0][0] = [-0.0, -0.0];
        cached_extrusion(&mut cache, negated, 25.0, variant, true).unwrap();
        let stats = cache.extrusion_stats();
        assert_eq!((stats.hits, stats.misses), (1, 1));
    }

    /// 布尔参与者：同形状反复来也一律直建，不查池、不入池、不产生命中。
    #[test]
    fn boolean_operands_are_baked_outside_the_pool() {
        let mut cache = CatalogueMeshCache::default();
        let variant = ExtrusionVariant::Inflated { grow: 0.051 };
        let first = cached_extrusion(&mut cache, pane_rings(), 25.0, variant, false).unwrap();
        let second = cached_extrusion(&mut cache, pane_rings(), 25.0, variant, false).unwrap();
        assert_same_geometry(&first, &second);
        assert_same_geometry(
            &first,
            &extrude_rings_inflated(pane_rings(), 25.0, 0.051).unwrap(),
        );
        let stats = cache.extrusion_stats();
        assert_eq!(
            (stats.hits, stats.misses, stats.boolean_operand_bypasses),
            (0, 0, 2)
        );
        // 之后同形状的可复用调用必须是 miss：烘焙路不许偷偷往池里塞条目。
        cached_extrusion(&mut cache, pane_rings(), 25.0, variant, true).unwrap();
        assert_eq!(cache.extrusion_stats().misses, 1);
    }

    /// 非有限输入：绕过缓存、错误与直建一字不差、不留条目。
    #[test]
    fn non_finite_input_bypasses_the_cache_and_reports_the_direct_error() {
        let mut cache = CatalogueMeshCache::default();
        let variant = ExtrusionVariant::Inflated { grow: 0.0 };
        let mut poisoned = pane_rings();
        poisoned[0][1] = [f64::NAN, 0.0];
        let cached = cached_extrusion(&mut cache, poisoned.clone(), 25.0, variant, true);
        let direct = extrude_rings_inflated(poisoned, 25.0, 0.0);
        match (cached, direct) {
            (Ok(a), Ok(b)) => assert_same_geometry(&a, &b),
            (Err(a), Err(b)) => assert_eq!(a.to_string(), b.to_string()),
            (cached, direct) => panic!("缓存路与直建路结论不一致: {cached:?} vs {direct:?}"),
        }
        let stats = cache.extrusion_stats();
        assert_eq!(
            (stats.hits, stats.misses, stats.non_finite_bypasses),
            (0, 0, 1)
        );
    }

    /// 建体失败不入缓存：零高度挤出报错后，池子必须还是空的。
    #[test]
    fn a_failed_build_leaves_no_reusable_entry() {
        let mut cache = CatalogueMeshCache::default();
        let variant = ExtrusionVariant::Inflated { grow: 0.0 };
        assert!(cached_extrusion(&mut cache, pane_rings(), 0.0, variant, true).is_err());
        let stats = cache.extrusion_stats();
        assert_eq!((stats.hits, stats.misses), (0, 0));
        // 同一形状换成合法高度必须是 miss 而不是撞上失败残留。
        cached_extrusion(&mut cache, pane_rings(), 25.0, variant, true).unwrap();
        assert_eq!(cache.extrusion_stats().misses, 1);
    }

    /// 统计增量与累计的守恒（与目录缓存统计同一纪律）。
    #[test]
    fn stats_delta_and_accumulate_round_trip() {
        let before = ExtrusionReuseStats {
            hits: 2,
            misses: 3,
            non_finite_bypasses: 1,
            boolean_operand_bypasses: 4,
        };
        let after = ExtrusionReuseStats {
            hits: 7,
            misses: 4,
            non_finite_bypasses: 1,
            boolean_operand_bypasses: 9,
        };
        let delta = after.checked_delta_from(before);
        assert_eq!(
            delta,
            ExtrusionReuseStats {
                hits: 5,
                misses: 1,
                non_finite_bypasses: 0,
                boolean_operand_bypasses: 5
            }
        );
        let mut total = before;
        total.accumulate(delta);
        assert_eq!(total, after);
    }
}

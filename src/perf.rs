//! 生成期分档计时：读取那一档量完之后，剩下的时间到底花在哪。
//!
//! 与 [`e3d_io::read_stats`] 同一套形状（进程级 relaxed 原子量、取区间差值），
//! 区别是这里量的是**几何侧**的几个choke point，读取那一档由 e3d-io 自己记。
//!
//! **各档是嵌套的，不是并列的**：`catalogue` 里面套着 `mesh_build`，`mesh_build`
//! 出来的实体后面又会被 `tri_mesh` 三角化。把它们相加没有意义——看的是各自的绝对值
//! 和调用次数，判断该往哪一层下刀。
//!
//! 计时器本身：每档一次 `Instant::now()` + 两个 `fetch_add`。调用次数在千到万这个
//! 量级时可以忽略；真要量微秒级的内层循环，别往里加档，去开采样分析器。

use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

macro_rules! define_phases {
    ($($field:ident => $nanos:ident / $calls:ident, $doc:literal;)+) => {
        $(
            static $nanos: AtomicU64 = AtomicU64::new(0);
            static $calls: AtomicU64 = AtomicU64::new(0);
        )+

        /// 某一刻的分档累计。差值用 [`Phases::delta_from`]。
        #[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
        pub struct Phases {
            $(
                #[doc = $doc]
                pub $field: PhaseTotal,
            )+
        }

        pub fn snapshot() -> Phases {
            Phases {
                $(
                    $field: PhaseTotal {
                        nanos: $nanos.load(Ordering::Relaxed),
                        calls: $calls.load(Ordering::Relaxed),
                    },
                )+
            }
        }

        /// 清零。基准跑之间用。
        pub fn reset() {
            $(
                $nanos.store(0, Ordering::Relaxed);
                $calls.store(0, Ordering::Relaxed);
            )+
        }

        impl Phases {
            /// 本区间的增量。
            pub fn delta_from(&self, earlier: Phases) -> Phases {
                Phases {
                    $($field: self.$field.delta_from(earlier.$field),)+
                }
            }

            /// 一行一档，直接进日志。
            pub fn summary(&self) -> String {
                [$((stringify!($field), self.$field),)+]
                    .iter()
                    .map(|(name, total)| format!("{name} {}ms/{}", total.millis(), total.calls))
                    .collect::<Vec<_>>()
                    .join(" | ")
            }
        }
    };
}

define_phases! {
    catalogue => CATALOGUE_NANOS / CATALOGUE_CALLS,
        "目录件整条路（`model_catalogue_member_with_cache`）：链走查 + 表达式求值 + 建体 + 摆放。";
    cat_chain => CAT_CHAIN_NANOS / CAT_CHAIN_CALLS,
        "目录链走查 `SPRE→SPCO→CATR→SCOM`（`catalogue_component`）——纯读，不建任何体。";
    cat_prims => CAT_PRIMS_NANOS / CAT_PRIMS_CALLS,
        "GMRE 图元循环：逐图元求值 + 取缓存网格 + `placed`。含 `mesh_build`。";
    cat_compose => CAT_COMPOSE_NANOS / CAT_COMPOSE_CALLS,
        "`Manifold::compose` 汇总本件图元 + 末尾整件 `placed` 到世界系。";
    mesh_build => MESH_BUILD_NANOS / MESH_BUILD_CALLS,
        "目录局部网格缓存未命中时真建的那一次（`cached_mesh_id` 的 build 闭包）。";
    elmodl => ELMODL_NANOS / ELMODL_CALLS,
        "设计侧建体整条路（`elmodl::model_element_with_cache`）：收刀 + 正体 + 摆放 + 扣负体。\
         路由成员与 CTRAY 从这里转进目录路，那两支的 `catalogue` 也算在本档里面。";
    local_solid => LOCAL_SOLID_NANOS / LOCAL_SOLID_CALLS,
        "正体局部实体那一次分派（拉伸 / 基本体 / 旋转 / 多面体 / SBFI / WALL / SCTN）。";
    place_world => PLACE_WORLD_NANOS / PLACE_WORLD_CALLS,
        "正体乘世界矩阵（`placed`）——惰性变换在这里被物化成真顶点。";
    negatives => NEGATIVES_NANOS / NEGATIVES_CALLS,
        "负体：逐把刀建体 + `subtract_negatives` 的布尔扣减。没有刀的元素也计一次。";
    neg_gate => NEG_GATE_NANOS / NEG_GATE_CALLS,
        "建正体之前那一段：收直属负体 + 读表现级 + DRWAND 级门筛刀。全是库读。";
    materialize => MATERIALIZE_NANOS / MATERIALIZE_CALLS,
        "出门前那句 `is_empty() / num_tri()`——`placed` 与布尔都是惰性的，真顶点在这里才算出来。";
    world_matrix => WORLD_MATRIX_NANOS / WORLD_MATRIX_CALLS,
        "owner 链上溯折叠世界矩阵（`transform::world_matrix`）。";
    route_walk => ROUTE_WALK_NANOS / ROUTE_WALK_CALLS,
        "路由走查（`route::walk_route`）：排成员、解 ATTA、算隐式管身的两端。";
    route_tubes => ROUTE_TUBES_NANOS / ROUTE_TUBES_CALLS,
        "隐式管身建体（`route::build_route_tubes`），逐容器一次。";
    tri_mesh => TRI_MESH_NANOS / TRI_MESH_CALLS,
        "`Manifold` → `TriMesh` 三角网格提取（`TriMesh::from_manifold`）。";
}

/// 一档的累计耗时与调用次数。
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PhaseTotal {
    pub nanos: u64,
    pub calls: u64,
}

impl PhaseTotal {
    pub fn duration(&self) -> Duration {
        Duration::from_nanos(self.nanos)
    }

    pub fn millis(&self) -> u128 {
        self.duration().as_millis()
    }

    /// 每次调用的平均耗时；一次都没调时给 `None`（不是 0——那是两回事）。
    pub fn per_call(&self) -> Option<Duration> {
        (self.calls > 0).then(|| Duration::from_nanos(self.nanos / self.calls))
    }

    fn delta_from(&self, earlier: PhaseTotal) -> PhaseTotal {
        PhaseTotal {
            nanos: self.nanos.saturating_sub(earlier.nanos),
            calls: self.calls.saturating_sub(earlier.calls),
        }
    }
}

/// RAII 计时守卫：`?` 提前返回也照样记账，这是不用手写 `elapsed()` 的全部理由。
pub struct Scope {
    nanos: &'static AtomicU64,
    calls: &'static AtomicU64,
    started: Instant,
}

impl Drop for Scope {
    fn drop(&mut self) {
        self.nanos
            .fetch_add(self.started.elapsed().as_nanos() as u64, Ordering::Relaxed);
        self.calls.fetch_add(1, Ordering::Relaxed);
    }
}

fn scope(nanos: &'static AtomicU64, calls: &'static AtomicU64) -> Scope {
    Scope {
        nanos,
        calls,
        started: Instant::now(),
    }
}

pub(crate) fn catalogue_scope() -> Scope {
    scope(&CATALOGUE_NANOS, &CATALOGUE_CALLS)
}

pub(crate) fn cat_chain_scope() -> Scope {
    scope(&CAT_CHAIN_NANOS, &CAT_CHAIN_CALLS)
}

pub(crate) fn cat_prims_scope() -> Scope {
    scope(&CAT_PRIMS_NANOS, &CAT_PRIMS_CALLS)
}

pub(crate) fn cat_compose_scope() -> Scope {
    scope(&CAT_COMPOSE_NANOS, &CAT_COMPOSE_CALLS)
}

pub(crate) fn mesh_build_scope() -> Scope {
    scope(&MESH_BUILD_NANOS, &MESH_BUILD_CALLS)
}

pub(crate) fn elmodl_scope() -> Scope {
    scope(&ELMODL_NANOS, &ELMODL_CALLS)
}

pub(crate) fn local_solid_scope() -> Scope {
    scope(&LOCAL_SOLID_NANOS, &LOCAL_SOLID_CALLS)
}

pub(crate) fn place_world_scope() -> Scope {
    scope(&PLACE_WORLD_NANOS, &PLACE_WORLD_CALLS)
}

pub(crate) fn negatives_scope() -> Scope {
    scope(&NEGATIVES_NANOS, &NEGATIVES_CALLS)
}

pub(crate) fn neg_gate_scope() -> Scope {
    scope(&NEG_GATE_NANOS, &NEG_GATE_CALLS)
}

pub(crate) fn materialize_scope() -> Scope {
    scope(&MATERIALIZE_NANOS, &MATERIALIZE_CALLS)
}

pub(crate) fn route_walk_scope() -> Scope {
    scope(&ROUTE_WALK_NANOS, &ROUTE_WALK_CALLS)
}

pub(crate) fn route_tubes_scope() -> Scope {
    scope(&ROUTE_TUBES_NANOS, &ROUTE_TUBES_CALLS)
}

pub(crate) fn world_matrix_scope() -> Scope {
    scope(&WORLD_MATRIX_NANOS, &WORLD_MATRIX_CALLS)
}

pub(crate) fn tri_mesh_scope() -> Scope {
    scope(&TRI_MESH_NANOS, &TRI_MESH_CALLS)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 守卫析构就记账，`?` 提前返回不会漏掉一档。
    #[test]
    fn a_scope_records_on_drop_even_when_the_body_bails_out() {
        let before = snapshot();
        fn bails() -> Result<(), ()> {
            let _guard = catalogue_scope();
            Err(())
        }
        assert!(bails().is_err());
        let delta = snapshot().delta_from(before);
        assert_eq!(delta.catalogue.calls, 1);
    }

    /// 一次都没调时平均值是 `None`，不是 0——「没跑过」与「跑了但零耗时」不是一回事。
    #[test]
    fn an_untouched_phase_has_no_average() {
        assert_eq!(PhaseTotal::default().per_call(), None);
        assert_eq!(
            PhaseTotal {
                nanos: 300,
                calls: 3
            }
            .per_call(),
            Some(Duration::from_nanos(100))
        );
    }
}

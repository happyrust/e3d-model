//! e3d-model：按 core.dll（Core3D 管线）方案另立的一套模型生成算法，
//! 数据一律经 `e3d-io` 直读 dabacon 库文件，不连任何数据库。
//!
//! 模块 ↔ core.dll/Core3D 对应（逆向依据：gen-model `teach/0009`、
//! `上下文/会话-2026-08-30-模型生成core-dll全流程与数据接口.md`）：
//!
//! | 本 crate | Core3D / core.dll | 职责 |
//! |---|---|---|
//! | [`pipeline`] | `add/ADDDES` | 设计树 DFS 遍历（成员原序），逐元素派发 |
//! | [`increment`] | `DB_IndexTableCompare` + `PartialUpdateDesiMgr` | 会话窗口差分 → 单元级增量重建 |
//! | [`category`] | `build/MODCMP` + nounlib `I*COM` 谓词 | 按 noun 分类挑建模方式 |
//! | [`elmodl`] | `build/ELMODL` + `build/LNEGIT` | 单元素建模：正体 + 负几何 CSG |
//! | [`solid`] | `cachegml/GTGEOM` → libgm `gm_Create*` | 几何构建（挤出/基本体/回转） |
//! | [`profile`] | `DGETI/GATPOS` 等属性读 | PLOO/PAVE、LOOP/VERT 环与高度采集 |
//! | [`polyhedron`] | `CSG_BasicPOL::getPrimGeom` → `sub_1071DCF0` | POLYHE/POHE/NPOLYH 显式 B-rep 建体 |
//! | [`discretise`] | libgm `GM_Profile` 折线化 | 带 FRAD 倒圆的环离散 |
//! | [`transform`] | `MAT16/CONCAT/GLSMAT` | POS/ORI 折叠出世界矩阵 |
//! | [`mesh`] | `gm_CreateFacetStructure` | 三角网格产物 + OBJ 导出 |
//!
//! CSG 内核用 manifold-csg（与 gen-model ADR-029 同款），对标 libgm 的
//! `gm_CreateCombination`；负体扣减前沿轴让 `restol = 0.051mm`，同 libgm
//! `GM_User::restol_` 口径（共壁负体不让开会碎成薄片，见 [`elmodl`]）。

// 几何入参校验一律写成 `!(x > 0.0)` 而不是 `x <= 0.0`：属性读出来的尺寸可能是 NaN，
// 而 NaN 参与的比较一律为 false，`x <= 0.0` 会把 NaN 当合法值放行，一路带进 CSG 内核
// 变成整块吞掉的坏网格。取反式让 NaN 落进拒绝分支，是刻意的，不是笔误。
#![allow(clippy::neg_cmp_op_on_partial_ord)]

pub mod category;
pub mod discretise;
pub mod elmodl;
pub mod increment;
pub mod mesh;
pub mod pipeline;
pub mod polyhedron;
pub mod primitive_instance;
pub mod profile;
pub mod solid;
pub mod transform;

pub use category::{Category, classify};
pub use elmodl::{GeneratedElement, ModelError};
pub use increment::{
    IncrementError, IncrementOutcome, IncrementReport, WindowDiff, collect_window, increment_update,
};
pub use mesh::TriMesh;
pub use pipeline::{GenerationOutcome, Report, generate_subtree};
pub use primitive_instance::{
    PrimitiveCacheStats, PrimitiveInstance, PrimitiveMeshKey, primitive_cache_stats,
};

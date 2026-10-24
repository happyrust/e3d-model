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
//! | [`route`] | `cachelib/GTTUBE` + `TUMAT` + `ORIMAT` + `pplib/CGETOD` | 路由链遍历与隐式管身 |
//! | [`section`] | `getProfileFromDB` + `pplib/QSPROF` + `pllib/GJLORI` | 型材截面（SPRO/SREC/SANN）求值与 JUSL 对齐 |
//! | [`catalogue_point`] | `catdblib/GATCAT` + `exprlib/EXEV*` | 目录侧 P 点求解（`SPRE→SCOM→PTSE`） |
//! | [`elmodl`] | `build/ELMODL` + `build/LNEGIT` | 单元素建模：正体 + 负几何 CSG |
//! | [`solid`] | `cachegml/GTGEOM` → libgm `gm_Create*` | 几何构建（挤出/基本体/回转） |
//! | [`profile`] | `DGETI/GATPOS` 等属性读 | PLOO/PAVE、LOOP/VERT 环与高度采集 |
//! | [`polyhedron`] | `CSG_BasicPOL::getPrimGeom` → `sub_1071DCF0` | POLYHE/POHE/NPOLYH 显式 B-rep 建体 |
//! | [`discretise`] | libgm `GM_Profile` 折线化 | 带 FRAD 倒圆的环离散 |
//! | [`transform`] | `MAT16/CONCAT/GLSMAT` | POS/ORI 折叠出世界矩阵 |
//! | [`rpath`] | `MDR_DB_POSandORIonRPATH` + `MDR_DB_PseudoAYDIR/LYDIR/PDIRonPOINTR` | RPATH 锚定公共件：`POSREF/PKDI/ZDIS/ZOFF` 落成路径上的点与局部系，各家再加自己的对正偏移 |
//! | [`cable_tray`] | `CAB_DB_PseudoPOS/ORIonCTELE::getAtt` | 桥架件（CTSTRA/CTBEND/…）没存 POS/ORI，走 [`rpath`] 锚定再加 `CWJUSL` 对正 |
//! | [`mesh`] | `gm_CreateFacetStructure` | 三角网格产物 + OBJ 导出 |
//!
//! CSG 内核用 manifold-csg（与 gen-model ADR-029 同款），对标 libgm 的
//! `gm_CreateCombination`；负体扣减前沿轴让 `restol = 0.051mm`，同 libgm
//! `GM_User::restol_` 口径（共壁负体不让开会碎成薄片，见 [`elmodl`]）。

// 几何入参校验一律写成 `!(x > 0.0)` 而不是 `x <= 0.0`：属性读出来的尺寸可能是 NaN，
// 而 NaN 参与的比较一律为 false，`x <= 0.0` 会把 NaN 当合法值放行，一路带进 CSG 内核
// 变成整块吞掉的坏网格。取反式让 NaN 落进拒绝分支，是刻意的，不是笔误。
#![allow(clippy::neg_cmp_op_on_partial_ord)]

pub mod attribute_reads;
pub mod bpanel;
pub mod cable;
pub mod cable_tray;
pub mod catalogue;
pub mod catalogue_point;
pub mod category;
pub mod core3d_registry;
pub mod datum;
pub mod db_discovery;
pub mod discretise;
pub mod element_diff;
pub mod elmodl;
pub mod extrusion_reuse;
pub mod genpri;
pub mod handrail;
pub mod hvac_tube;
pub mod increment;
pub mod ledger;
pub mod mesh;
pub mod perf;
pub mod pipeline;
pub mod polyhedron;
pub mod primitive_instance;
pub mod profile;
pub mod route;
pub mod rpath;
pub mod rvm_read;
pub mod rvm_write;
pub mod section;
pub mod solid;
pub mod stair;
pub mod stair_flight;
pub mod transform;
pub mod udet;

pub use attribute_reads::{AttributeReadSet, attribute_read_set, reads_attribute};
pub use cable_tray::{CABLE_TRAY_ELEMENT_NOUNS, CableTrayPlacement, is_cable_tray_element};
pub use catalogue::{
    CatalogueMeshCache, CatalogueMeshCacheStats, model_catalogue_member,
    model_catalogue_member_with_cache,
};
pub use catalogue_point::{
    CataloguePoint, CataloguePointSet, CataloguePoints, UnresolvedCataloguePoint,
    catalogue_point_set,
};
pub use category::{
    BuildDisposition, Category, CompositeFamily, NounDecision, classify, decision,
    route_container_nouns, route_member_nouns,
};
pub use db_discovery::{DirectoryDbResolver, DiscoveredDb, DiscoveryError, SkippedFile};
pub use element_diff::{ElementDiff, MemberDelta, diff_element};
pub use elmodl::{GeneratedElement, GeometryId, ModelError};
pub use extrusion_reuse::ExtrusionReuseStats;
pub use increment::{
    AffectedClosure, IncrementError, IncrementOutcome, IncrementReport, NoModelTier, UpdatePlan,
    WindowDiff, collect_window, increment_update,
};
pub use ledger::{ChangeKind, ChangeLedger, ChangeTally, LedgerEntry};
pub use mesh::TriMesh;
pub use pipeline::{
    GenerationOutcome, ImpliedTubeDegenerate, ImpliedTubes, Report, generate_subtree,
    generate_subtree_with_cache,
};
pub use primitive_instance::{
    AnalyticPrimitive, PrimitiveCacheStats, PrimitiveInstance, PrimitiveMeshKey,
    primitive_cache_stats,
};
pub use route::{
    CatalogueNotWired, PPointResolver, Route, RouteEnd, RoutePoint, TubeGeometry, TubeOutcome,
    TubeSlot, build_tube, orimat, tube_length, tube_outer_diameter, tumat, walk_route,
};

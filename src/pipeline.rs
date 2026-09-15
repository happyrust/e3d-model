//! 设计树遍历与总账（对标 `add/ADDDES` 的 DFS + `Core3D` 的批任务收口）。
//!
//! 遍历规则（成员一律文件原序）：
//!
//! | 类别 | 动作 |
//! |---|---|
//! | `List` | 无 |
//! | `LoopExtrusion` / `Primitive` / `Revolution` / `Polyhedron` | [`model_element`] |
//! | `Catalog` / `Unsupported` / `Unknown` | 记账 |
//! | `RouteContainer` | 记账，并按 `RouteWalker` 的串接方式数出隐式管身 |
//! | `RouteMember` | 记账 |
//! | `PrimitiveContainer` | 记账；负体成员认领进 [`Report::host_negatives`]，本期不扣 |
//! | `Composite(CableTray)` | `CTSTRA/CTBEND/…` 现算放置（[`crate::cable_tray`]）；`CTRAY` 走目录链建体；其余（CWAY/CWBRAN）记账 |
//! | `NonGraphic` | 记账（判「无三维表示」的放行要留名，见 [`Report::non_graphic`]） |
//! | `Neg*` / `ProfileData` | 属主消费掉了就不会走到这里；走到了就是孤儿，记账 |
//!
//! **下钻是无条件的**，只有一个例外：被本元素消费掉的成员（自己的剖面环与负体）
//! 不再入栈，改为整棵子树计入 [`Report::consumed`]。
//!
//! 「本元素不建模」**不等于**「它下面没有能建模的东西」——这条曾经反过来写过一次：
//! `Catalog` / `Unsupported` / `NonGraphic` 都不下钻，而 ams1112 的 4275 个 PANE
//! 全挂在被判成目录件的 CWALL 与未实现项的 CFLOOR 底下，于是全库 30940 个元素
//! 只走到 160 个，总账上却只有一行「catalog: CWALL 121」，看不出丢了一整片船体。
//!
//! 账本原则（gen-model 宪法同款）：静默缺件是最高级别缺陷。
//! [`Report::accounts_for`] 就是这条的硬判据——`visited + consumed` 必须等于
//! 子树的元素总数，一个都不许蒸发。

use std::collections::BTreeMap;

use e3d_io::db_element::{DbElement, DbElementError};
use e3d_io::refno::RefNo;

use crate::cable_tray::{self, is_cable_tray_element};
use crate::catalogue::{
    CatalogueMeshCache, CatalogueMeshCacheStats, model_catalogue_member_with_cache,
};
use crate::catalogue_point::{CataloguePoints, tube_outer_diameter_from_stub};
use crate::category::{
    BuildDisposition, Category, CompositeFamily, decision, derives_implied_tubes,
    enters_catalogue_by_catref, graphics_ignored_below, is_model_unit,
};
use crate::elmodl::{GeneratedElement, GeometryId, Modeled, SkipKind, model_element_with_cache};
use crate::extrusion_reuse::ExtrusionReuseStats;
use crate::route::{
    ArriveCheck, PPointResolver, Route, RouteEnd, SlotAlignment, TubeOutcome, ZERO_LENGTH_TOL_MM,
    bore_mismatch, build_tube, resolve_end, slot_alignment, slot_ordinals, tube_length,
    tube_outer_diameter, walk_route,
};

/// 一条有名有姓的记录。
#[derive(Debug, Clone, serde::Serialize)]
pub struct Incident {
    pub refno: String,
    pub noun: String,
    pub detail: String,
}

/// 隐式管身账（算法文档 §2.3 的字段口径）。
///
/// 管身是运行时派生件：库里没有 TUBI 记录，它**既不是 `visited` 也不是 `consumed`**。
/// 塞进那两本账会让 [`Report::accounts_for`] 失真——那是全 crate 唯一的硬判据，
/// 让它变松比少一本账严重得多。所以单开这一本，与主账互不相干。
#[derive(Debug, Default, serde::Serialize)]
pub struct ImpliedTubes {
    /// 管身条数，按 `RouteWalker` 的串接方式数出来：每个路由成员前面一条，
    /// 末尾到容器 `TPOS` 再补一条，即 `路由成员数 + 1`。
    pub count: usize,
    /// 已实际生成圆柱实体的管身数。
    pub built: usize,
    /// 管身总长（mm）。**只有 `measured == count` 时才是 `Some`**——量了一部分就
    /// 报总长，写出来的数看着跟真的一样，却少了没量到的那些。宁可给 `None`。
    pub total_length: Option<f64>,
    /// 已经量出长度的槽位数。`count - measured` 就是还卡在
    /// [`ImpliedTubeDegenerate`] 各项上的条数。
    pub measured: usize,
    /// 已量出的那部分长度之和（mm）。名字不叫 `total_*`，因为在
    /// `measured == count` 之前它只是个部分和。
    pub measured_length_mm: f64,
    /// CGETOD 从离开侧的 TUBI 目录件 `PARA[2]` 取得真外径的槽位数。
    pub catalogue_od: usize,
    /// 目录件或外径缺失，按 Core 的规则回落到到达侧 bore 的槽位数。
    pub bore_fallback: usize,
    /// 按属主容器 noun 分账：BRAN 一支和 HANG 一支的管身要能分开看。
    pub by_owner_noun: BTreeMap<String, usize>,
    /// 退化情形明细，P2 出几何时才填得上。
    pub degenerate: ImpliedTubeDegenerate,
}

impl ImpliedTubes {
    /// 全部槽位都量过了才给总长，否则留 `None`。
    ///
    /// 分成一步而不是边量边写，是因为「量全了没有」只有走完整棵子树才知道；
    /// 中途写进去的部分和一旦被下游读走，就再也没有地方能看出它是残的。
    pub fn settle_total_length(&mut self) {
        self.total_length = (self.measured == self.count).then_some(self.measured_length_mm);
    }
}

/// 建不出几何的管身分类计数（算法文档 §4 的退化表）。
#[derive(Debug, Default, serde::Serialize)]
pub struct ImpliedTubeDegenerate {
    /// 低于最小管长（`length <` [`crate::route::ZERO_LENGTH_TOL_MM`]，1.0mm，
    /// 两端点重合是其特例）。
    pub zero_length: usize,
    /// P 点取不到（`GTTUBE` 的 ok 标志为 0）：链走通了但目录里没这个 P 点，
    /// 或轴向/距离表达式求不出值。
    pub missing_ppoint: usize,
    /// P 点所在的库根本没打开——`SPRE`/`CATR` 指向未注册的 dbno。
    ///
    /// 与 `missing_ppoint` 分开记：那条是数据缺，这条是**跑的时候少给了库**，
    /// 补个 `--catalogue` 就能变成真管。混在一起会把配置问题读成数据问题。
    pub ppoint_db_not_open: usize,
    /// 未打开目录库按 dbno 分账，供启动器补齐 `--catalogue`。
    pub ppoint_db_not_open_by_dbno: BTreeMap<String, usize>,
    /// 库已打开但引用的历史目录元素不在所钉会话中。
    pub ppoint_target_not_found: usize,
    pub ppoint_target_not_found_by_dbno: BTreeMap<String, usize>,
    /// 两端通径不一致，按到达侧取值照常建，但要留痕。
    pub bore_mismatch: usize,
    /// 两端都解出来了，却定不出朝向或取不到半径。
    ///
    /// 与 `missing_ppoint` 分开：那条是「还没接上」，这条是「接上了但数据坏」，
    /// 混在一起会让 P1 的进度看起来比实际好。
    pub unbuildable: usize,
    /// 偏轴：到达点偏离离开轴线（或离开点偏离到达轴线）超过 [`crate::route::TUBE_ALIGNMENT_TOL`]
    /// 但仍朝前。E3D 不画这种管（[`SlotAlignment::Misaligned`]）。
    pub misaligned: usize,
    /// 反向：到达点在离开点身后。E3D 不画，数据一致性检查会报「负管长」
    /// （[`SlotAlignment::Reversed`]）。
    pub reversed: usize,
}

/// 生成总账（可直接 serde_json 落盘）。
#[derive(Debug, Default, serde::Serialize)]
pub struct Report {
    /// 遍历到的元素总数。
    pub visited: usize,
    /// 建成实体数。
    pub generated: usize,
    /// 正体被有依据跳过（零高度 / 里程碑边界）。
    pub skipped: Vec<Incident>,
    /// 数据坏建不出来（响亮账）。
    pub failed: Vec<Incident>,
    /// 目录件记账：noun → 数量（二期 G4）。
    pub catalog_pending: BTreeMap<String, usize>,
    /// 只画中心线的型材：noun → 件数（[`crate::elmodl::SkipKind::CentrelineOnly`]）。
    ///
    /// ams8000 的 528 只 SCTN 里 500 只没有 `SPRE`（MSUP 杆件），E3D 的 TTY 导出给的是
    /// 一根 `Line`、没有实体。**不并进 [`Report::skipped`]**：那本账的语义是「E3D 也什么
    /// 都不画」，而这些是 **E3D 画了一条线、我们一条都还不画**——混在一起，中心线这笔
    /// 欠款就再也看不见了。它也仍留在 [`Report::evidence_pending`] 里，正是这个原因。
    pub centreline_only: BTreeMap<String, usize>,
    /// 目录链断的明细（[`crate::elmodl::SkipKind::CatalogueChainBroken`]）：引用悬空、
    /// 库没开、缺 `CATR`/`GSTR`。
    ///
    /// 与上一本分开、且逐条点名：ams8000 那 24 只指向 db 5052 的 refno 不在所钉会话的
    /// 索引里，是数据或 `--catalogue` 配置问题，要能顺着 refno 查下去；一个计数查不动。
    pub catalogue_chain_broken: Vec<Incident>,
    /// 路由容器记账：noun → 数量。BRAN/HANG/LUG/SUPC/TRUNNI 五族**都记**——
    /// HANG 与风管 BRAN（属主 HVAC）也在账上，只是不派生管身
    /// （[`crate::category::derives_implied_tubes`]）。
    pub route_containers: BTreeMap<String, usize>,
    /// 图元容器记账：noun → 数量。[`Category::PrimitiveContainer`]，目前只有 `TMPL`。
    ///
    /// 单开一本而不是并进 [`Report::proven_non_graphic`]：容器自己确实不出几何，
    /// 但它**替宿主持有负体**，与 SITE/ZONE 那种纯组织节点不是一回事。
    pub primitive_containers: BTreeMap<String, usize>,
    /// 宿主负体：被图元容器认领、但**一刀都还没扣**的负几何，noun → 数量。
    ///
    /// `aps250110` 里 `TMPL` 底下 1559 件（`NCYL` 1282 / `NBOX` 232 / `NXTR` 45）。
    /// 它们扣的不是容器自己的正体成员，而是容器之外的宿主——消防水带箱的 `NXTR`
    /// 是墙上的凹龛，钢结构端板的 `NBOX` 是给 203×133 工字梁让位的梁廓开口。
    /// E3D 这条「构件在别处、洞开在宿主上」的路我们整条都还没有。
    ///
    /// **为什么非要单开一本**：这些元素被 [`Category::PrimitiveContainer`] 剪枝之后
    /// 只剩 [`Report::consumed`] 里的一个数，而那本账里绝大多数是被属主真正吃进
    /// 实体的剖面环与负体。混在一起，「已经扣掉了」和「还没有人扣」就长得一模一样
    /// ——报告全绿，宿主上却少了洞。放进 [`Report::orphans`] 也不对：那本的语义是
    /// 「数据或分类有问题」，而这些负体数据没问题、分类也没问题，缺的是机制。
    ///
    /// 这本账不进 [`Report::accounts_for`] 的等式（元素已在 `consumed` 里算过一次），
    /// 与 [`Report::implied_tubes`] 同一个道理。
    pub host_negatives: BTreeMap<String, usize>,
    /// 路由成员记账：noun → 数量。目录驱动，几何压在 P1。
    ///
    /// 与 [`Report::catalog_pending`] 分开记：目录件是型材与墙族那一路，路由成员
    /// 另有隐式管身要配对，两本混在一起就看不出管路到底缺了多少件。
    pub route_members: BTreeMap<String, usize>,
    /// 已求值的目录 SEXT 在整次生成中的瞬态复用统计。它不改变输出身份或持久化格式。
    pub catalogue_mesh_cache: CatalogueMeshCacheStats,
    /// 设计侧环拉伸（含负体）的同批网格复用统计，见 [`crate::extrusion_reuse`]。
    /// 与上一本同为瞬态观测账，不改变输出身份或持久化格式。
    pub extrusion_mesh_cache: ExtrusionReuseStats,
    /// 隐式管身账，见 [`ImpliedTubes`]。
    pub implied_tubes: ImpliedTubes,
    /// 几何集正体记账：noun → 数量。[`Category::GeometrySet`] 那 26 个 noun。
    ///
    /// 与 [`Report::unsupported`] 分开：几何集是 core.dll 里**另一条建模路**，
    /// 不是「实现没跟上」而是建模概念缺失。混进 unsupported 会让「本期不做的
    /// 已知项」和「压根没想过的东西」同账。
    pub geometry_sets: BTreeMap<String, usize>,
    /// 几何集负体记账：noun → 数量。[`Category::NegGeometrySet`] 那 16 个 INGCOM noun。
    ///
    /// 不并进 [`Report::orphans`]：孤儿那本账的语义是「数据或分类有问题」，而一个
    /// `NSBO` 挂在几何集属主底下是完全正常的数据——只是几何集这条路本期整条不建，
    /// 属主自己都在 [`Report::geometry_sets`] 里记着。混进孤儿账会让它天天有值，
    /// 真正的孤儿（挂错属主的 PLOO）就淹在里面了。
    pub geometry_set_negatives: BTreeMap<String, usize>,
    /// 认识但本期不做：noun → 数量。
    pub unsupported: BTreeMap<String, usize>,
    /// 分类表没认领：noun → 数量。出现即分类表欠账。
    pub unknown_nouns: BTreeMap<String, usize>,
    /// **E3D 建、我们没接线**的件数：noun → 数量。判据是
    /// [`crate::core3d_registry::is_not_wired`]，即「[`classify`] 判 `Unknown` 或
    /// `NonGraphic` ∧ 该 noun 在 Core3D 的 builder 注册表里」。
    ///
    /// 这是总纲 §4 的 D3 闸门第一次有一个**跟着实跑走的数**。在它之前，
    /// 「E3D 也不画」与「我们还没建」的分界只存在于 recheck 文档里，
    /// 靠人每隔几周去反编译里翻一遍；`catalog_pending` 那一万件到底欠多少，
    /// 报告上答不出来。
    ///
    /// **交叉记账，不进 [`Report::accounts_for`] 的等式**：这里的每一件都已经在
    /// [`Report::unknown_nouns`] 或 [`Report::proven_non_graphic`] 里数过一次了。
    /// 与 [`Report::implied_tubes`] / [`Report::host_negatives`] 同一个道理——
    /// 它是从另一个角度照同一批元素，不是第三本收支账。
    ///
    /// 只数**遍历到的**元素：被属主消费掉的成员不入栈，因此不在这本账上。
    /// 那是对的——一个被 `SCTN` 吃进去的 `PLOO` 不欠任何人一份几何。
    pub not_wired: BTreeMap<String, usize>,
    /// 判「无三维表示」而放过的：noun → 数量。[`Category::NonGraphic`]。
    ///
    /// 这本账立出来之前，`NonGraphic` 与 [`Category::List`] 共用一个空分支，元素只进
    /// `visited` 总数、按 noun 一笔不记——于是「判它无几何」这个结论**在实跑里无法证伪**：
    /// 报告上看不出哪些 noun 是以这个理由被放过的，更看不出放过了多少件。而
    /// `NOZZ`/`ELCONN` 恰恰被 `GTGEOM` 硬编码送进复合几何路，判据方向是反的。
    ///
    /// 记了账才谈得上复核：先让每一次「无三维表示」的放行都在报告里留名。
    pub proven_non_graphic: BTreeMap<String, usize>,
    /// 已有明确建模路径、但 builder 或目标语料证据尚未闭环：noun → 数量。
    ///
    /// 每个 [`BuildDisposition::EvidencePending`] 的元素先按 noun 记一笔，**产出了就当场
    /// 回冲**（[`decrement`]）：目录件 model unit 建成、桥架件放置算出、`CTRAY` 目录链出实体、
    /// 路由成员出形状、路由容器补出至少一根管身、以及容器按
    /// [`crate::category::derives_implied_tubes`] 判定本来就不该有管身。
    ///
    /// **不回冲的两处是有意的**：路由成员「解出来但没形状」还没有对着 E3D 逐件核过；
    /// [`Report::centreline_only`] 那 500 只 MSUP 杆件是 E3D 画了线、我们一条都不画。
    /// 两笔都是真欠款，回冲掉就再也看不见了。
    ///
    /// 回冲这件事漏一处的表现不是报错，是**排期依据失真**：`RouteMember` 那 92 个 noun
    /// 一度整批不回冲，于是 ams7997 的 `BEND` 3320、`ATTA` 3226 全挂在这本账上，而那一跑
    /// `route_members=11448`、`generated=50501` —— 报告说「待建」的绝大多数其实早建好了。
    pub evidence_pending: BTreeMap<String, usize>,
    /// 已实际完成成员聚合并产出实体的复合构件：noun → 数量。
    ///
    /// 桥架族两种口径：`CTRAY` 计入是目录几何真建出来了；`CTSTRA`/`CTBEND`/… 计入是
    /// 它没存的 POS/ORI 现算成功（[`crate::cable_tray::placement`]）——那七个 noun 自己
    /// 不出实体，实体在下面的 `CTRAY` 上。
    pub composites_built: BTreeMap<String, usize>,
    /// 每个实际出现 NOUN 的分类依据；同一 NOUN 只保留稳定的一条依据。
    pub noun_evidence: BTreeMap<String, String>,
    /// 被属主消费掉、因而没单独遍历的子树：**被剪掉的那个成员的 noun** → 整棵子树的
    /// 元素数。它与 [`Report::visited`] 相加必须等于子树总元素数，见
    /// [`Report::accounts_for`]。
    pub consumed: BTreeMap<String, usize>,
    /// 没有属主消费的剖面环 / 负几何：noun → 数量。
    /// 出现即数据或分类有问题——一个 PLOO 挂在 ZONE 底下，它的 PAVE 就没人读。
    pub orphans: BTreeMap<String, usize>,
    /// 负几何被跳过的明细（正体照出，体积会偏大，对拍时按这本账归因）。
    pub negatives_skipped: Vec<Incident>,
    /// 非致命观察（倒圆越界、SJUS 未生效、孔环退化等）。
    pub notes: Vec<Incident>,
}

/// 一次生成的完整产出。
pub struct GenerationOutcome {
    pub elements: Vec<GeneratedElement>,
    pub report: Report,
    /// DFS 访问到的每一个元素，**按遍历序**、含根、含没出几何的容器与记账项；
    /// 被属主消费掉的成员（剖面环 / 负体）不入栈，因此也不在这里——它们在
    /// [`Report::consumed`] 那本账上。`visited.len() == report.visited` 恒成立。
    ///
    /// 消费方拿它做「生成一根顺手把子树的数据面终态写进库」（gen-model 读透缓存的
    /// 写透，`docs/plans/2026-09-05-kv-mem-zero-ingest-read-through-plan.md` §3.2）：
    /// 子树是什么只有这一份答案，另起一次枚举就是第二份。
    pub visited: Vec<RefNo>,
    /// 这次生成**真读到**的目录元素：不在根所在库、且所在库类型为 `CATA` 的每一个元素，
    /// 按首次读到的顺序去重。「真读」= e3d-io 走到了 `parsed` / `extraction`（含缓存命中），
    /// 只造了句柄没读的不算；记录点在 e3d-io 的读入口（`read_stats::trace_element_reads`），
    /// 不是逐个调用点手记，所以目录链上每一跳（`SPRE→SPCO→CATR→SCOM→GMSE/PTSE/…`、
    /// P 点求值、剖面、桥架目录件）都在里面，新增一条读法也不用改这里。
    ///
    /// 它是 gen-model 依赖闭包（ADR-064 D5 `cata_dependency`）的对拍 oracle：
    /// `catalogue_visited ⊆ closure(seed(visited))` 必须成立，闭包比实读宽允许、窄就是漏。
    /// 纯输出、不带句柄（ADR-057 B1）。
    ///
    /// 两条边界：跨到**别的设计库**的读（`HREF` / `TREF` 指向另一库的 NOZZ 之类）不在这里
    /// ——它们不是目录依赖；目录局部网格缓存命中的目录件**不会再被读**，因此也不在这里——
    /// 要一根完整的目录实读集，给它一个空的 [`CatalogueMeshCache`]。
    pub catalogue_visited: Vec<RefNo>,
}

/// 从 `root` 起 DFS 生成整棵子树。遍历中的 I/O 错误（读不到成员表）直接上抛——
/// 那是库文件层面的问题，不是单元素账。
pub fn generate_subtree(root: DbElement) -> Result<GenerationOutcome, DbElementError> {
    generate_subtree_with_cache(root, &mut CatalogueMeshCache::default())
}

/// 与 [`generate_subtree`] 相同，但由调用者持有目录局部网格缓存。
///
/// 批量生成多个独立根时复用同一缓存，可以避免对相同目录基本体重复求值和三角化；
/// 输出元素身份、世界变换和最终 CSG 仍逐根独立。报告中的缓存统计只包含本次调用增量。
pub fn generate_subtree_with_cache(
    root: DbElement,
    catalogue_mesh_cache: &mut CatalogueMeshCache,
) -> Result<GenerationOutcome, DbElementError> {
    let mut elements = Vec::new();
    let mut report = Report::default();
    let mut visited = Vec::new();
    let cache_stats_before = catalogue_mesh_cache.stats();
    let extrusion_stats_before = catalogue_mesh_cache.extrusion_stats();
    // 目录实读集从这里开始记（e3d-io 读入口的线程局部记录器），到 DFS 结束收账；
    // 中途 `?` 早退时记录器随作用域丢弃，不会留下一层挂着。
    let root_dbno = root.refno().dbno();
    let set = root.db_set().clone();
    let read_trace = e3d_io::read_stats::trace_element_reads();
    // 显式栈，避免深树递归爆栈；压栈顺序反转以保持文件原序遍历。
    let mut stack: Vec<DbElement> = vec![root];

    while let Some(el) = stack.pop() {
        report.visited += 1;
        visited.push(el.refno());
        let noun = el.element_type()?;
        let noun_decision = decision(&noun);
        let category = noun_decision.category;
        let refno = el.refno().to_string();
        report
            .noun_evidence
            .entry(noun.clone())
            .or_insert_with(|| noun_decision.evidence.to_owned());
        if noun_decision.disposition == BuildDisposition::EvidencePending {
            *report.evidence_pending.entry(noun.clone()).or_insert(0) += 1;
        }
        // 记在 match 外面：这本账横跨 Unknown 与 NonGraphic 两个分支，而这两个分支
        // 之所以要一起数，正是因为它们在报告上长得完全不一样——一个挂着欠账、一个
        // 记在「不欠账」那本。分头记就等于让判据跟着分支走，走着走着就只剩一半。
        if crate::core3d_registry::is_not_wired(&noun) {
            *report.not_wired.entry(noun.clone()).or_insert(0) += 1;
        }

        match category {
            Category::List => {}
            Category::NonGraphic => {
                *report.proven_non_graphic.entry(noun.clone()).or_insert(0) += 1;
            }
            Category::LoopExtrusion
            | Category::Primitive(_)
            | Category::Revolution
            | Category::Polyhedron => {
                match model_element_with_cache(&el, category, catalogue_mesh_cache) {
                    Ok(Modeled::Solid(generated)) => {
                        for skip in &generated.negatives_skipped {
                            report.negatives_skipped.push(Incident {
                                refno: skip.refno.to_string(),
                                noun: skip.noun.clone(),
                                detail: format!("属主 {refno}（{noun}）: {}", skip.reason),
                            });
                        }
                        for note in &generated.notes {
                            report.notes.push(Incident {
                                refno: refno.clone(),
                                noun: noun.clone(),
                                detail: note.clone(),
                            });
                        }
                        record_generated(&mut report, &noun, 1);
                        elements.push(*generated);
                    }
                    Ok(Modeled::Skipped { kind, reason }) => {
                        record_skip(&mut report, kind, refno, noun, reason);
                    }
                    Err(error) => report.failed.push(Incident {
                        refno,
                        noun,
                        detail: error.to_string(),
                    }),
                }
            }
            // 目录件里唯独 SBFI 出实体。判据取自 `category::is_model_unit`，增量的
            // 上卷读的是同一个函数——这份名单一旦两处各写一份，被删的 SBFI 就会在
            // 增量里永远上卷不到单元、永远不出 Remove，而两边的账各自还都是平的。
            //
            // 「DESP 够不够长」那道门也曾写在这里，同样只写了这一份：增量直接调
            // `model_element`，短 DESP 在那边成了 `regen_failed`，在这边却是没这件事。
            // 现在门在 `elmodl` 里，短 DESP 两边统一落 `Skipped`。
            Category::Catalog if is_model_unit(&noun) => {
                match model_element_with_cache(&el, category, catalogue_mesh_cache) {
                    Ok(Modeled::Solid(generated)) => {
                        for note in &generated.notes {
                            report.notes.push(Incident {
                                refno: refno.clone(),
                                noun: noun.clone(),
                                detail: note.clone(),
                            });
                        }
                        record_generated(&mut report, &noun, 1);
                        elements.push(*generated);
                    }
                    Ok(Modeled::Skipped { kind, reason }) => {
                        record_skip(&mut report, kind, refno, noun, reason);
                    }
                    Err(error) => report.failed.push(Incident {
                        refno,
                        noun,
                        detail: error.to_string(),
                    }),
                }
            }
            Category::Catalog => {
                *report.catalog_pending.entry(noun.clone()).or_insert(0) += 1;
            }
            Category::RouteContainer => {
                *report.route_containers.entry(noun.clone()).or_insert(0) += 1;
                // 风管分支（属主 HVAC）与 HANG 一样只下钻成员、不补管身，见谓词文档。
                if !derives_implied_tubes(&el)? {
                    // 不派生管身是**谓词给的结论**，不是 builder 没写：几何全在成员上。
                    // 留在 evidence_pending 里等于说「这只 HANG 还欠一份几何」。
                    decrement(&mut report.evidence_pending, &noun);
                } else {
                    let walk_timing = crate::perf::route_walk_scope();
                    let route = walk_route(&el)?;
                    drop(walk_timing);
                    if route.pass_through_unresolved > 0 {
                        report.notes.push(Incident {
                            refno: refno.clone(),
                            noun: noun.clone(),
                            detail: format!(
                                "{} 只 ATTA 的目录链走不到 SCOM，按穿过处理；若是 ANCI 型支撑，管身会比 E3D 少几段",
                                route.pass_through_unresolved
                            ),
                        });
                    }
                    let tubes = route.tubes.len();
                    report.implied_tubes.count += tubes;
                    *report
                        .implied_tubes
                        .by_owner_noun
                        .entry(noun.clone())
                        .or_insert(0) += tubes;
                    let tubes_timing = crate::perf::route_tubes_scope();
                    let built = build_route_tubes(
                        &el,
                        &route,
                        &CataloguePoints,
                        &mut report.implied_tubes,
                        &mut elements,
                    )?;
                    drop(tubes_timing);
                    if built > 0 {
                        record_generated(&mut report, &noun, built);
                    }
                }
            }
            Category::RouteMember => {
                *report.route_members.entry(noun.clone()).or_insert(0) += 1;
                build_catalogue_geometry(
                    &el,
                    &refno,
                    &noun,
                    false,
                    catalogue_mesh_cache,
                    &mut report,
                    &mut elements,
                );
            }
            // 类 28 的 `NOZZ` 与路由成员**几何同源**：实体都是目录件 GMRE 的求值结果，
            // 只是入口从 `SPRE` 换成 `CATR`（[`enters_catalogue_by_catref`]），所以共用同一个
            // 执行体而不是再抄一份求值。它自己的成员聚合（`sub_107210A0` 那条 builder 链）
            // 是另一半，还没有——SLYK 那 34 只 NOZZ 一个成员都没有，这一半就是全部。
            Category::Composite(CompositeFamily::CoreClass28)
                if enters_catalogue_by_catref(&noun) =>
            {
                build_catalogue_geometry(
                    &el,
                    &refno,
                    &noun,
                    true,
                    catalogue_mesh_cache,
                    &mut report,
                    &mut elements,
                );
            }
            Category::PrimitiveContainer => {
                *report.primitive_containers.entry(noun.clone()).or_insert(0) += 1;
            }
            // 电缆桥架：CTSTRA/CTBEND/… 自己不出实体，它们的「建」是把没存的 POS/ORI
            // 现算出来（`cable_tray::placement`），几何在它们下面的 CTRAY 上经目录链
            // （SPRE→TABITE→PRTREF→GPART→CATR→SCOM→GMRE）建。放置解不出来就记 failed——
            // 那一只 CTRAY 随后也会在 world_matrix 里撞上同一条错，两处都响是刻意的：
            // 一处指出病灶（CTELE），一处指出后果（CTRAY 没建）。
            Category::Composite(CompositeFamily::CableTray) if is_cable_tray_element(&noun) => {
                match cable_tray::placement(&el) {
                    Ok(_) => {
                        decrement(&mut report.evidence_pending, &noun);
                        *report.composites_built.entry(noun.clone()).or_insert(0) += 1;
                    }
                    Err(error) => report.failed.push(Incident {
                        refno,
                        noun,
                        detail: error.to_string(),
                    }),
                }
            }
            Category::Composite(CompositeFamily::CableTray) if noun == "CTRAY" => {
                match model_catalogue_member_with_cache(&el, catalogue_mesh_cache) {
                    Ok(build) => {
                        for detail in &build.diagnostics {
                            report.notes.push(Incident {
                                refno: refno.clone(),
                                noun: noun.clone(),
                                detail: detail.clone(),
                            });
                        }
                        match build.generated {
                            Some(generated) => {
                                for note in &generated.notes {
                                    report.notes.push(Incident {
                                        refno: refno.clone(),
                                        noun: noun.clone(),
                                        detail: note.clone(),
                                    });
                                }
                                record_generated(&mut report, &noun, 1);
                                *report.composites_built.entry(noun.clone()).or_insert(0) += 1;
                                elements.push(generated);
                            }
                            // 路由成员没形状是常态（隐式管身补），托盘没形状不是：
                            // 目录链断在哪一环要留名，不能让它静默留在 evidence_pending 里。
                            None => report.skipped.push(Incident {
                                refno,
                                noun,
                                detail: format!(
                                    "目录链（SPRE→…→SCOM→GMRE）没有产出可见几何：{}",
                                    build.no_shape.unwrap_or_else(|| "原因未记".to_string())
                                ),
                            }),
                        }
                    }
                    Err(error) => report.failed.push(Incident {
                        refno,
                        noun,
                        detail: error.to_string(),
                    }),
                }
            }
            // 布局式扶手（`crate::handrail`）：判据在 `category::is_model_unit`，执行臂在
            // `model_element`；这里与 CTRAY 一样只做投递。跳过的两种结局（详图式 / 没东西
            // 可画）都是「E3D 在 HANDRA 这一层同样不画」——留在 evidence_pending 里等于说
            // 这只 HANDRA 还欠一份几何，而它欠的（若有）是 RAIL/KICKPL/HRPOST 成员各自的账。
            Category::Composite(CompositeFamily::Structural) if is_model_unit(&noun) => {
                match model_element_with_cache(&el, category, catalogue_mesh_cache) {
                    Ok(Modeled::Solid(generated)) => {
                        for note in &generated.notes {
                            report.notes.push(Incident {
                                refno: refno.clone(),
                                noun: noun.clone(),
                                detail: note.clone(),
                            });
                        }
                        record_generated(&mut report, &noun, 1);
                        *report.composites_built.entry(noun.clone()).or_insert(0) += 1;
                        elements.push(*generated);
                    }
                    Ok(Modeled::Skipped { kind, reason }) => {
                        decrement(&mut report.evidence_pending, &noun);
                        record_skip(&mut report, kind, refno, noun, reason);
                    }
                    Err(error) => report.failed.push(Incident {
                        refno,
                        noun,
                        detail: error.to_string(),
                    }),
                }
            }
            Category::Composite(_) => {
                // 类 28 与其余复合族必须经真实成员聚合 builder 才能转入 composites_built。
                // 在此之前只进独立的 evidence_pending 账，绝不把“已分类”冒充“已建”。
            }
            Category::GeometrySet => {
                *report.geometry_sets.entry(noun.clone()).or_insert(0) += 1;
            }
            Category::NegGeometrySet => {
                *report
                    .geometry_set_negatives
                    .entry(noun.clone())
                    .or_insert(0) += 1;
            }
            Category::Unsupported(_) => {
                *report.unsupported.entry(noun.clone()).or_insert(0) += 1;
            }
            Category::Unknown => {
                *report.unknown_nouns.entry(noun.clone()).or_insert(0) += 1;
            }
            // 属主消费掉的剖面与负体根本不入栈，能走到这里的都没人认领。
            Category::ProfileData
            | Category::NegExtrusion
            | Category::NegRevolution
            | Category::NegPolyhedron
            | Category::NegPrimitive(_)
            | Category::NegUnimplemented(_) => {
                *report.orphans.entry(noun.clone()).or_insert(0) += 1;
            }
        }

        push_members(&el, category, &mut stack, &mut report)?;
    }

    report.implied_tubes.settle_total_length();
    report.catalogue_mesh_cache = catalogue_mesh_cache
        .stats()
        .checked_delta_from(cache_stats_before);
    report.extrusion_mesh_cache = catalogue_mesh_cache
        .extrusion_stats()
        .checked_delta_from(extrusion_stats_before);
    debug_assert_eq!(
        visited.len(),
        report.visited,
        "访问名单与 visited 计数必须同步增长"
    );
    let catalogue_visited = catalogue_reads(&set, root_dbno, read_trace.finish());
    Ok(GenerationOutcome {
        elements,
        report,
        visited,
        catalogue_visited,
    })
}

/// 一次生成的全部实读里挑出目录侧：不在根所在库、且所在库在池里登记为 `CATA`。
///
/// 库类型只问池里**已经开着**的库（读过的元素所在库必然开着），按 dbno 记一次；
/// 类型字解不出来的库当非目录——宁少不多是这里的口径：这份名单是对拍 oracle 的
/// 「实读」一侧，把一个非目录元素混进去会让闭包对拍误报「漏」。
fn catalogue_reads(
    set: &e3d_io::db_element::DbSet,
    root_dbno: u32,
    reads: Vec<RefNo>,
) -> Vec<RefNo> {
    let mut is_catalogue: BTreeMap<u32, bool> = BTreeMap::new();
    reads
        .into_iter()
        .filter(|refno| {
            let dbno = refno.dbno();
            if dbno == root_dbno {
                return false;
            }
            *is_catalogue.entry(dbno).or_insert_with(|| {
                set.db_type(dbno)
                    .is_some_and(|db_type| db_type.eq_ignore_ascii_case("CATA"))
            })
        })
        .collect()
}

/// 把一条「有依据地没出几何」按 [`SkipKind`] 记进对应的那本账。
///
/// 分类由 [`crate::elmodl::model_element_with_cache`] 随结论一起交出来，这里只做投递——
/// 报告不去猜 `reason` 那句话怎么写的。三本账的语义差别见 [`Report::centreline_only`]。
fn record_skip(report: &mut Report, kind: SkipKind, refno: String, noun: String, reason: String) {
    match kind {
        SkipKind::NoShape => report.skipped.push(Incident {
            refno,
            noun,
            detail: reason,
        }),
        // 500 只 MSUP 杆件的理由是同一句话，逐条留名只会把 notes 淹掉；件数按 noun 记。
        SkipKind::CentrelineOnly => {
            *report.centreline_only.entry(noun).or_insert(0) += 1;
        }
        SkipKind::CatalogueChainBroken => report.catalogue_chain_broken.push(Incident {
            refno,
            noun,
            detail: reason,
        }),
    }
}

fn decrement(ledger: &mut BTreeMap<String, usize>, noun: &str) {
    if let Some(count) = ledger.get_mut(noun) {
        *count -= 1;
        if *count == 0 {
            ledger.remove(noun);
        }
    }
}

/// 目录件 GMRE 求值那条臂的执行体：路由成员与类 28 的 `NOZZ` 共用。
///
/// 两者的几何是同一件事——设计件指到一只目录件，实体由它的 GMRE 程序求值得出，键就是
/// 设计件自己的 refno。差别只有两处，都由 `class28` 这一个参数说清：入口属性在
/// [`crate::category::enters_catalogue_by_catref`] 里分岔，产出记在哪本账上在这里分岔
/// （NOZZ 不是路由成员，记 `composites_built`，与 CTRAY 同档）。
///
/// 抽成函数而不是让两条臂共用一个带 guard 的或模式：`Category::RouteMember | …
/// if guard` 的 guard 盖住整个模式，编译器不认它必然覆盖 `RouteMember`，match 当场
/// 不完备。
fn build_catalogue_geometry(
    el: &DbElement,
    refno: &str,
    noun: &str,
    class28: bool,
    cache: &mut CatalogueMeshCache,
    report: &mut Report,
    elements: &mut Vec<GeneratedElement>,
) {
    match model_catalogue_member_with_cache(el, cache) {
        Ok(build) => {
            for detail in &build.diagnostics {
                report.notes.push(Incident {
                    refno: refno.to_string(),
                    noun: noun.to_string(),
                    detail: detail.clone(),
                });
            }
            match build.generated {
                Some(generated) => {
                    for note in &generated.notes {
                        report.notes.push(Incident {
                            refno: refno.to_string(),
                            noun: noun.to_string(),
                            detail: note.clone(),
                        });
                    }
                    record_generated(report, noun, 1);
                    if class28 {
                        *report.composites_built.entry(noun.to_string()).or_insert(0) += 1;
                    }
                    elements.push(generated);
                }
                // 路由成员没形状是常态（无 SPRE 的 FTUB、无 GMRE 的 MDS 支撑参考、全零高
                // 的目录件），不进 skipped；但**依据**必须留在 notes 里，否则报告上分不出
                // 「有意不画」与「漏建」（ams8000 体检 105 件）。
                //
                // 这一支**不**回冲 evidence_pending：没出形状的那几件是否真该没有，还没有
                // 对着 E3D 逐件核过，账上留着才看得见。
                None => {
                    if let Some(reason) = build.no_shape {
                        report.notes.push(Incident {
                            refno: refno.to_string(),
                            noun: noun.to_string(),
                            detail: reason,
                        });
                    }
                }
            }
        }
        Err(error) => report.failed.push(Incident {
            refno: refno.to_string(),
            noun: noun.to_string(),
            detail: error.to_string(),
        }),
    }
}

/// 记 `n` 件产出：`generated` 加上去，同时把这个 noun 的 [`Report::evidence_pending`]
/// 回冲一笔。
///
/// **DFS 里不许再直接写 `report.generated += …`**（由
/// `generated_is_only_recorded_together_with_the_pending_retirement` 钉住）。两件事拆成
/// 两行写过一次，代价是 `RouteMember` 那 92 个 noun 整批只加不冲：ams7997 的 `BEND` 3320、
/// `ATTA` 3226 挂在待建账上，而那一跑它们绝大多数早就建出来了——报告不报错，只是把
/// 排期依据读反。
///
/// 回冲按 noun 记一笔，不按产出件数：待建账数的是**元素**，一只路由容器补出 12 根管身
/// 仍然只是一只不再欠几何的容器。[`Category`] 判 `Built` 的 noun 从来没进过这本账，
/// [`decrement`] 对缺席的键是空操作，所以每个产出点都能无条件调它。
fn record_generated(report: &mut Report, noun: &str, n: usize) {
    report.generated += n;
    decrement(&mut report.evidence_pending, noun);
}

/// 单独为**一个**路由容器产出它名下该有的全部隐式管身——增量重建派生几何走这一条。
///
/// # 为什么增量不许自己再写一遍
///
/// 这条与 [`generate_subtree`] 里那段路由容器分支**是同一段代码**：同一个
/// [`walk_route`]、同一个 [`CataloguePoints`]、同一个 [`build_route_tubes`]。
/// 键怎么定、哪些槽位退化得建不出来，全量和增量只能有一份答案——两边各写一份的
/// 下场不是编译错误，是真库门上「全量说有这根、增量说没有」的静默分歧，机理与
/// [`crate::category::is_model_unit`] 文档里记的那次 `SBFI` 漏件一模一样。
///
/// 记账用的 [`ImpliedTubes`] 在这里是**就地丢弃**的：增量报告有自己的账目口径，
/// 这条只负责交出「该有哪几根、长什么样」。
pub fn route_tubes_of(container: &DbElement) -> Result<Vec<GeneratedElement>, DbElementError> {
    if !derives_implied_tubes(container)? {
        return Ok(Vec::new());
    }
    let route = walk_route(container)?;
    let mut discarded = ImpliedTubes::default();
    let mut elements = Vec::new();
    build_route_tubes(
        container,
        &route,
        &CataloguePoints,
        &mut discarded,
        &mut elements,
    )?;
    Ok(elements)
}

fn invalid_tubi_reason(alignment: SlotAlignment) -> Option<&'static str> {
    match alignment {
        SlotAlignment::Misaligned { .. } => Some("misaligned"),
        SlotAlignment::Reversed { .. } => Some("reversed"),
        SlotAlignment::Aligned { .. } | SlotAlignment::Indeterminate => None,
    }
}

/// 量一个容器上的全部管身槽位，逐类记账（算法文档 §4 的退化表），能建的建出来。
///
/// 管身没有自己的库记录，`refno` 一律填容器的（绝不制造伪 refno），身份全靠
/// [`GeometryId::ImpliedTube`] 那把两端站点的键——**一个容器 refno 名下会有 N 根管身，
/// 只有这把键区分得开它们**。这也是模型集、增量 Remove 与真库门都必须按
/// [`GeometryId`] 而不是按 refno 索引的原因。
fn build_route_tubes(
    container: &DbElement,
    route: &Route,
    resolver: &dyn PPointResolver,
    ledger: &mut ImpliedTubes,
    elements: &mut Vec<GeneratedElement>,
) -> Result<usize, DbElementError> {
    let mut built_count = 0usize;
    // 序号是同 (from,to) 站点对内的出现序，与增量侧 `route_slot_keys` 同一份
    // [`slot_ordinals`]——两边各自 enumerate 就是两份串接规则，键会静默分叉。
    let ordinals = slot_ordinals(&route.tubes);
    for (slot, route_ordinal) in route.tubes.iter().zip(ordinals) {
        let ends = resolve_end(&slot.from, resolver)
            .and_then(|from| Ok((from, resolve_end(&slot.to, resolver)?)));
        let (Some(from), Some(to)) = (match ends {
            Ok(ends) => ends,
            // 库没开是配置问题，不是这棵子树坏了：记一笔接着量下一根。
            Err(DbElementError::DbNotRegistered { dbno }) => {
                ledger.degenerate.ppoint_db_not_open += 1;
                *ledger
                    .degenerate
                    .ppoint_db_not_open_by_dbno
                    .entry(dbno.to_string())
                    .or_insert(0) += 1;
                continue;
            }
            Err(DbElementError::NotFound { dbno, .. }) => {
                ledger.degenerate.missing_ppoint += 1;
                ledger.degenerate.ppoint_target_not_found += 1;
                *ledger
                    .degenerate
                    .ppoint_target_not_found_by_dbno
                    .entry(dbno.to_string())
                    .or_insert(0) += 1;
                continue;
            }
            Err(error) => return Err(error),
        }) else {
            ledger.degenerate.missing_ppoint += 1;
            continue;
        };
        // E3D 的第一道门：槽位要与两端方向共轴且朝前，否则不画（见
        // `TUBE_ALIGNMENT_TOL`）。零长槽位先让 `build_tube` 按 zero_length 记账——
        // 两点重合时夹角没有意义。容器尾只有真读到 TDIR 才查到达侧。
        let mut invalid_reason = None;
        if tube_length(from.pos, to.pos) >= ZERO_LENGTH_TOL_MM {
            let arrive = match &slot.to {
                RouteEnd::MemberArrive { .. } | RouteEnd::MemberLeave { .. } => ArriveCheck::Member,
                RouteEnd::ContainerTail(container) => {
                    if container.get_direction("TDIR")?.is_some() {
                        ArriveCheck::Member
                    } else {
                        ArriveCheck::Skip
                    }
                }
                RouteEnd::ContainerHead(_) => ArriveCheck::Skip,
            };
            let alignment = slot_alignment(&from, &to, arrive);
            invalid_reason = invalid_tubi_reason(alignment);
            match alignment {
                SlotAlignment::Misaligned { .. } => {
                    ledger.degenerate.misaligned += 1;
                }
                SlotAlignment::Reversed { .. } => {
                    ledger.degenerate.reversed += 1;
                }
                SlotAlignment::Aligned { .. } | SlotAlignment::Indeterminate => {}
            }
        }
        // E3D 的第二道门（`build/LIMPOK`）：两端公称通径不等就不画，**整条路由上每个
        // 槽位一视同仁**，不分头 / 构件 / 尾（见 [`crate::route::BORE_MATCH_TOL_MM`]）。
        //
        // 头槽位先有原生 AMS 探针：只改 `HBOR` 500 → 25，RVM 里那根头管就没了，改回
        // 500 又回来；只改 stub 的公称通径则不动它。尾与构件槽位 2026-09-15 由
        // `Q TGEOM` / `Q TEXIST` 这对伪属性收口——BRAN `21491/9826`（B3）逐件问下来
        // 7 true / 14 false，与本仓槽位表逐格重合，唯一相反的那格正是尾槽位
        // `21491/9846:LEAVE(P2) → TAIL`：E3D 说 `TGEOM=false`（基准 RVM 里连
        // `TUBE 8` 这个 member 都没有），而它恰好是全模型 5 907 根建成管里唯一一根
        // 两端通径不等的（200 vs `TBOR` 250）。
        if bore_mismatch(&from, &to) {
            ledger.degenerate.bore_mismatch += 1;
            invalid_reason.get_or_insert("bore mismatch");
        }
        // Native VALV and COUP probes retain identical P-point coordinates
        // when ORIL is cleared, but no longer export the head tube solid.
        if let (RouteEnd::ContainerHead(_), RouteEnd::MemberArrive { member, .. }) =
            (&slot.from, &slot.to)
        {
            match member.get_bool("ORIL") {
                Ok(Some(false)) => {
                    invalid_reason.get_or_insert("head arrival orientation unset");
                }
                Ok(_) | Err(DbElementError::UnknownAttribute { .. }) => {}
                Err(error) => return Err(error),
            }
        }
        // Native AMS BRAN probes with both null and non-null HREF remove
        // the head solid when HSTU is cleared. Check the stored references
        // before resolving the catalogue: an unavailable target is not null.
        if let RouteEnd::ContainerHead(departure) = &slot.from {
            if departure.element_type()?.eq_ignore_ascii_case("BRAN") {
                let head_stub_null = match departure.get_ref("HSTU") {
                    Ok(value) => value.is_none(),
                    Err(DbElementError::UnknownAttribute { .. }) => false,
                    Err(error) => return Err(error),
                };
                if head_stub_null {
                    let alternate_null = match departure.get_ref("HSRO") {
                        Ok(value) => value.is_none(),
                        Err(DbElementError::UnknownAttribute { .. }) => true,
                        Err(error) => return Err(error),
                    };
                    if alternate_null {
                        invalid_reason.get_or_insert("head stub unset");
                    }
                }
            }
        }
        // CGETOD 的 A 是槽位离开侧：容器头读 HSTU/HSRO，普通构件离开读
        // LSTU/LSRO。目录 PARA 是一基下标，PARA[2] 即 Rust values[1]。
        let catalogue_od_result = match &slot.from {
            crate::route::RouteEnd::ContainerHead(departure)
            | crate::route::RouteEnd::ContainerTail(departure) => {
                tube_outer_diameter_from_stub(departure, &["HSTU", "HSRO"])
            }
            crate::route::RouteEnd::MemberLeave { member, .. }
            | crate::route::RouteEnd::MemberArrive { member, .. } => {
                tube_outer_diameter_from_stub(member, &["LSTU", "LSRO"])
            }
        };
        // CGETOD 本来就是可失败查询：目录库没打开、历史 stub 不在所钉会话，
        // 都必须走 GTTUBG 的 arrive-bore 回落，不能把整棵生成打断。
        let catalogue_od = match catalogue_od_result {
            Ok(value) => value,
            Err(DbElementError::DbNotRegistered { .. } | DbElementError::NotFound { .. }) => None,
            Err(error) => return Err(error),
        };
        if catalogue_od.is_some() {
            ledger.catalogue_od += 1;
        } else {
            ledger.bore_fallback += 1;
        }
        let diameter = tube_outer_diameter(catalogue_od, None, to.bore);
        match build_tube(&from, &to, diameter) {
            TubeOutcome::Built(tube) => {
                ledger.measured += 1;
                ledger.measured_length_mm += tube.length_mm;
                // Alignment and head-condition checks are native E3D draw
                // gates.  Keep their measurements and detailed ledger counts,
                // but never leak a diagnostic cylinder into the model/RVM.
                if invalid_reason.is_some() {
                    continue;
                }
                ledger.built += 1;
                built_count += 1;
                let (from_station, to_station) = slot.station_refnos();
                elements.push(GeneratedElement {
                    geometry_id: GeometryId::ImpliedTube {
                        container_refno: container.refno().to_string(),
                        route_ordinal,
                        from_refno: from_station.to_string(),
                        to_refno: to_station.to_string(),
                    },
                    // 管身没有库记录；来源元素固定保存容器，绝不制造伪 refno。
                    refno: container.refno(),
                    noun: "TUBI".into(),
                    is_invalid_tubi: false,
                    name: None,
                    solid: tube.solid,
                    parts: None,
                    part_catalogue_nouns: Vec::new(),
                    part_record_repeats: Vec::new(),
                    degenerate_parts: Vec::new(),
                    world: tube.placement,
                    primitive_instance: tube.primitive_instance,
                    analytic_primitive: None,
                    negatives_applied: 0,
                    negatives_skipped: Vec::new(),
                    notes: vec![format!(
                        "implied tube length={}mm radius={}mm source={}{}",
                        tube.length_mm,
                        tube.radius_mm,
                        if catalogue_od.is_some() {
                            "catalogue PARA[2]"
                        } else {
                            "arrive bore fallback"
                        },
                        "",
                    )],
                });
            }
            TubeOutcome::ZeroLength { .. } => ledger.degenerate.zero_length += 1,
            TubeOutcome::NoDirection | TubeOutcome::NoRadius | TubeOutcome::Failed(_) => {
                ledger.degenerate.unbuildable += 1
            }
        }
    }
    Ok(built_count)
}

/// 成员入栈。被本元素消费掉的（自己的剖面环与负体）不入栈，整棵子树计进
/// [`Report::consumed`]——不计的话它们就从账上蒸发了。
///
/// 第二种消费与类别无关：本元素 `graphicsBehaviour == 1`（[`graphics_ignored_below`]）
/// 时它的**全部**成员都是几何数据，Core3D 一件都不当独立图形项画
/// （`isGraphicsIgnoredBetween`）。这里同样整棵计进 `consumed`、不入栈——增量那头
/// `nearest_unit` / `collect_unit_subtree` 按同一个谓词跨过它，两边才对得上。
fn push_members(
    el: &DbElement,
    category: Category,
    stack: &mut Vec<DbElement>,
    report: &mut Report,
) -> Result<(), DbElementError> {
    let consumes = category.consumes_members();
    let ignores_members = graphics_ignored_below(&el.element_type()?);
    let mut batch = Vec::new();
    for member in el.members()? {
        let noun = member.element_type()?;
        let member_category = decision(&noun).category;
        let consumed = ignores_members
            || (consumes
                && (matches!(member_category, Category::ProfileData)
                    || member_category.is_negative()));
        if consumed {
            // 图元容器「认领」负体只是为了不让它们落进孤儿账，一刀都没扣
            // （宿主在容器之外，见 [`Category::PrimitiveContainer`]）。剪枝之后
            // 它们只剩 `consumed` 里的一个数，与被属主真吃掉的剖面环混在一本账上
            // ——单看那本分不出「扣掉了」和「还没人扣」。所以在这里再记一笔。
            if matches!(category, Category::PrimitiveContainer) && member_category.is_negative() {
                *report.host_negatives.entry(noun.clone()).or_insert(0) += 1;
            }
            *report.consumed.entry(noun).or_insert(0) += subtree_size(&member)?;
        } else {
            batch.push(member);
        }
    }
    // 栈是后进先出：反着压，弹出来才是文件原序。
    stack.extend(batch.into_iter().rev());
    Ok(())
}

/// 一棵子树的元素数（含自身）。只用来记账，不读属性。
fn subtree_size(root: &DbElement) -> Result<usize, DbElementError> {
    let mut count = 0usize;
    let mut stack = vec![root.clone()];
    while let Some(el) = stack.pop() {
        count += 1;
        stack.extend(el.members()?);
    }
    Ok(count)
}

impl Report {
    /// 并进另一棵子树的账（多根库逐根生成时用）。
    ///
    /// 放在结构体自己身上而不是调用方：漏并一本账就等于凭空少掉一批元素，
    /// 而调用方那边看不出来——`consumed` 刚加上时就是漏在 `gen_ams` 里，
    /// 靠 [`Report::accounts_for`] 才逮住。
    pub fn merge(&mut self, other: Report) {
        let Report {
            visited,
            generated,
            skipped,
            failed,
            catalog_pending,
            centreline_only,
            catalogue_chain_broken,
            route_containers,
            route_members,
            catalogue_mesh_cache,
            extrusion_mesh_cache,
            primitive_containers,
            host_negatives,
            implied_tubes,
            geometry_sets,
            geometry_set_negatives,
            unsupported,
            unknown_nouns,
            not_wired,
            proven_non_graphic,
            evidence_pending,
            composites_built,
            noun_evidence,
            consumed,
            orphans,
            negatives_skipped,
            notes,
        } = other;
        self.visited += visited;
        self.generated += generated;
        self.skipped.extend(skipped);
        self.failed.extend(failed);
        self.catalogue_chain_broken.extend(catalogue_chain_broken);
        self.negatives_skipped.extend(negatives_skipped);
        self.notes.extend(notes);
        self.catalogue_mesh_cache.accumulate(catalogue_mesh_cache);
        self.extrusion_mesh_cache.accumulate(extrusion_mesh_cache);
        self.implied_tubes.count += implied_tubes.count;
        self.implied_tubes.built += implied_tubes.built;
        self.implied_tubes.measured += implied_tubes.measured;
        self.implied_tubes.measured_length_mm += implied_tubes.measured_length_mm;
        self.implied_tubes.catalogue_od += implied_tubes.catalogue_od;
        self.implied_tubes.bore_fallback += implied_tubes.bore_fallback;
        self.implied_tubes.total_length =
            match (self.implied_tubes.total_length, implied_tubes.total_length) {
                // 一边还没量过就整体算没量过：把 None 当 0 加进去会得到一个偏小的
                // 「总长」，而它看起来跟真数一模一样。
                (Some(a), Some(b)) => Some(a + b),
                _ => None,
            };
        let degenerate = &mut self.implied_tubes.degenerate;
        degenerate.zero_length += implied_tubes.degenerate.zero_length;
        degenerate.missing_ppoint += implied_tubes.degenerate.missing_ppoint;
        degenerate.ppoint_db_not_open += implied_tubes.degenerate.ppoint_db_not_open;
        degenerate.bore_mismatch += implied_tubes.degenerate.bore_mismatch;
        degenerate.unbuildable += implied_tubes.degenerate.unbuildable;
        degenerate.misaligned += implied_tubes.degenerate.misaligned;
        degenerate.reversed += implied_tubes.degenerate.reversed;
        degenerate.ppoint_target_not_found += implied_tubes.degenerate.ppoint_target_not_found;
        for (dbno, count) in implied_tubes.degenerate.ppoint_db_not_open_by_dbno {
            *degenerate
                .ppoint_db_not_open_by_dbno
                .entry(dbno)
                .or_insert(0) += count;
        }
        for (dbno, count) in implied_tubes.degenerate.ppoint_target_not_found_by_dbno {
            *degenerate
                .ppoint_target_not_found_by_dbno
                .entry(dbno)
                .or_insert(0) += count;
        }
        for (target, source) in [
            (&mut self.catalog_pending, catalog_pending),
            (&mut self.centreline_only, centreline_only),
            (&mut self.route_containers, route_containers),
            (&mut self.route_members, route_members),
            (&mut self.primitive_containers, primitive_containers),
            (&mut self.host_negatives, host_negatives),
            (
                &mut self.implied_tubes.by_owner_noun,
                implied_tubes.by_owner_noun,
            ),
            (&mut self.geometry_sets, geometry_sets),
            (&mut self.geometry_set_negatives, geometry_set_negatives),
            (&mut self.unsupported, unsupported),
            (&mut self.unknown_nouns, unknown_nouns),
            (&mut self.not_wired, not_wired),
            (&mut self.proven_non_graphic, proven_non_graphic),
            (&mut self.evidence_pending, evidence_pending),
            (&mut self.composites_built, composites_built),
            (&mut self.consumed, consumed),
            (&mut self.orphans, orphans),
        ] {
            for (noun, count) in source {
                *target.entry(noun).or_insert(0) += count;
            }
        }
        for (noun, evidence) in noun_evidence {
            self.noun_evidence.entry(noun).or_insert(evidence);
        }
    }

    /// 被属主消费掉、没单独遍历的元素总数。
    pub fn consumed_total(&self) -> usize {
        self.consumed.values().sum()
    }

    /// 遍历有没有漏人：`visited + consumed` 应当等于子树的元素总数。
    ///
    /// `tree_total` 从索引侧独立数出来（`scan_elements`），两边同源就验不出东西。
    /// 差值为正 = 有子树被静默跳过了，那正是「CWALL 不下钻」那次事故的形状：
    /// 报告全绿，30940 个元素只走到 160 个。
    pub fn accounts_for(&self, tree_total: usize) -> Result<(), String> {
        let seen = self.visited + self.consumed_total();
        if seen == tree_total {
            return Ok(());
        }
        Err(format!(
            "账对不上：visited {} + consumed {} = {seen}，索引侧 {tree_total}，差 {}",
            self.visited,
            self.consumed_total(),
            tree_total as i64 - seen as i64,
        ))
    }

    /// 隐式管身总数（派生件，不进 [`Report::accounts_for`] 的等式）。
    pub fn implied_tube_total(&self) -> usize {
        self.implied_tubes.count
    }

    /// E3D 建、我们没接线的件数合计，见 [`Report::not_wired`]。
    ///
    /// 这是 D3 闸门上那个该逐周下降的数。它降到零之前，`unknown=0` 说明不了任何事。
    pub fn not_wired_total(&self) -> usize {
        self.not_wired.values().sum()
    }

    /// 各本账 + 生成数是否把 visited 解释干净。对拍前先跑这个自检，
    /// 账对不上就别谈几何。
    pub fn totals_line(&self) -> String {
        format!(
            "visited={} consumed={} generated={} skipped={} failed={} catalog={} \
             route_containers={} route_members={} implied_tubes={} tubes_measured={} \
             tubes_built={} tubes_catalogue_od={} tubes_bore_fallback={} tubes_no_ppoint={} \
             tubes_misaligned={} tubes_reversed={} \
             unsupported={} unknown={} not_wired={} orphans={} neg_skipped={}",
            self.visited,
            self.consumed_total(),
            self.generated,
            self.skipped.len(),
            self.failed.len(),
            self.catalog_pending.values().sum::<usize>(),
            self.route_containers.values().sum::<usize>(),
            self.route_members.values().sum::<usize>(),
            self.implied_tubes.count,
            self.implied_tubes.measured,
            self.implied_tubes.built,
            self.implied_tubes.catalogue_od,
            self.implied_tubes.bore_fallback,
            self.implied_tubes.degenerate.missing_ppoint,
            self.implied_tubes.degenerate.misaligned,
            self.implied_tubes.degenerate.reversed,
            self.unsupported.values().sum::<usize>(),
            self.unknown_nouns.values().sum::<usize>(),
            self.not_wired_total(),
            self.orphans.values().sum::<usize>(),
            self.negatives_skipped.len(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn misaligned_and_reversed_tubes_are_kept_as_invalid_diagnostics() {
        assert_eq!(
            invalid_tubi_reason(SlotAlignment::Misaligned { deviation_deg: 3.0 }),
            Some("misaligned")
        );
        assert_eq!(
            invalid_tubi_reason(SlotAlignment::Reversed {
                deviation_deg: 178.0
            }),
            Some("reversed")
        );
        assert_eq!(
            invalid_tubi_reason(SlotAlignment::Aligned {
                leave_deg: 0.0,
                arrive_deg: Some(0.0),
            }),
            None
        );
    }

    /// 产出与回冲必须一起发生：DFS 里不许再直接写 `report.generated += …`。
    ///
    /// 这条不看行为看源码，因为它防的正是「行为看不出来」的那种漏：忘了回冲，
    /// 报告照常全绿、`accounts_for` 照常平，只有 [`Report::evidence_pending`] 悄悄
    /// 把已经建好的件继续算作待建。`RouteMember` 那 92 个 noun 就是这么挂了整批的。
    ///
    /// 唯一允许直接加的地方是 [`record_generated`] 自己与 [`Report::merge`]。
    #[test]
    fn generated_is_only_recorded_together_with_the_pending_retirement() {
        let source = include_str!("pipeline.rs");
        let body = source
            .split_once("pub fn generate_subtree_with_cache(")
            .expect("generate_subtree_with_cache 必须在")
            .1
            .split_once("\nfn record_skip(")
            .expect("DFS 主体在 record_skip 之前收尾")
            .0;
        let offenders: Vec<&str> = body
            .lines()
            .map(str::trim)
            .filter(|line| !line.starts_with("//"))
            .filter(|line| line.contains("report.generated +="))
            .collect();
        assert!(
            offenders.is_empty(),
            "DFS 里请改调 record_generated（产出即回冲 evidence_pending）：{offenders:?}"
        );
        assert!(
            body.contains("record_generated(&mut report"),
            "DFS 至少要有一个产出点走 record_generated，否则这条守卫是空的"
        );
    }

    /// `catalogue_visited` 的来源只有一个：DFS 前开的 e3d-io 读记录器，DFS 后收账、按目录侧
    /// 过滤。它是 gen-model 依赖闭包的对拍 oracle（ADR-064 D5），记录器开在栈建好之前、收在
    /// 回执组装之前——挪到别处（比如只包一部分分支）名单就静默变短，而对拍只会因此更容易绿。
    /// 这条看源码，因为「少记」在行为上看不出来。
    #[test]
    fn catalogue_visited_is_traced_around_the_whole_dfs() {
        let source = include_str!("pipeline.rs");
        let body = source
            .split_once("pub fn generate_subtree_with_cache(")
            .expect("generate_subtree_with_cache 必须在")
            .1
            .split_once("\nfn catalogue_reads(")
            .expect("catalogue_reads 紧跟 DFS 之后")
            .0;
        let opened = body
            .find("e3d_io::read_stats::trace_element_reads()")
            .expect("DFS 前要开读记录器");
        let stack_built = body
            .find("let mut stack: Vec<DbElement> = vec![root];")
            .expect("显式栈");
        let finished = body.find("read_trace.finish()").expect("DFS 后要收账");
        let outcome = body.find("Ok(GenerationOutcome {").expect("回执");
        assert!(
            opened < stack_built,
            "记录器要在压栈之前打开，根自己的读也在作用域里"
        );
        assert!(
            stack_built < finished && finished < outcome,
            "收账在 DFS 之后、回执之前"
        );
        assert!(
            body.contains("catalogue_visited,") || body.contains("catalogue_visited:"),
            "回执里要带 catalogue_visited"
        );
        assert_eq!(
            body.matches("trace_element_reads()").count(),
            1,
            "一次生成只开一个记录器；分支里再开会把那一段的读从外层账上拿走"
        );
    }

    /// [`record_generated`] 本身：产出计数照加，待建账当场少一笔，加没加过的 noun 不炸。
    #[test]
    fn recording_a_build_retires_one_pending_entry() {
        let mut report = Report::default();
        report.evidence_pending.insert("BEND".into(), 2);

        record_generated(&mut report, "BEND", 1);
        assert_eq!(report.generated, 1);
        assert_eq!(report.evidence_pending["BEND"], 1);

        // 回冲按 noun 记一笔，与产出件数无关：一只 BRAN 补 12 根管身仍只销一只容器。
        record_generated(&mut report, "BEND", 12);
        assert_eq!(report.generated, 13);
        assert!(
            !report.evidence_pending.contains_key("BEND"),
            "冲到 0 的键要移掉，否则报告上留一行 BEND=0"
        );

        // 判 Built 的 noun 从没进过这本账，回冲必须是空操作而不是下溢。
        record_generated(&mut report, "BOX", 1);
        assert_eq!(report.generated, 14);
        assert!(report.evidence_pending.is_empty());
    }

    /// 隐式管身是派生件，不许挤进 `visited + consumed` 那道等式。
    ///
    /// 把管身算成 visited 会让 `accounts_for` 变成「索引数 + 管身数」才平——判据一松，
    /// 真丢了子树也照样过。这是这本账必须独立的全部理由。
    #[test]
    fn implied_tubes_stay_out_of_the_balance_equation() {
        let mut report = Report {
            visited: 10,
            ..Default::default()
        };
        report.consumed.insert("PLOO".into(), 4);
        assert!(report.accounts_for(14).is_ok());

        report.implied_tubes.count = 561;
        report
            .implied_tubes
            .by_owner_noun
            .insert("BRAN".into(), 561);
        assert!(
            report.accounts_for(14).is_ok(),
            "管身进账后 14 这个总数不该改变"
        );
        assert_eq!(report.implied_tube_total(), 561);
    }

    /// 新开的三本账都要并进 `merge`，漏一本就等于凭空少掉一批元素。
    ///
    /// 这条是回归钉：`consumed` 当年就是漏在 `gen_ams` 的多根合并里，报告看着正常，
    /// 靠 `accounts_for` 才逮住——而路由三本账没有那样的等式兜底，只能靠这条测试。
    #[test]
    fn merge_carries_every_route_ledger() {
        let mut left = Report::default();
        left.route_containers.insert("BRAN".into(), 2);
        left.route_members.insert("ELBO".into(), 3);
        left.primitive_containers.insert("TMPL".into(), 6);
        left.host_negatives.insert("NCYL".into(), 40);
        left.implied_tubes.count = 5;
        left.implied_tubes.built = 3;
        left.implied_tubes.catalogue_od = 2;
        left.implied_tubes.bore_fallback = 1;
        left.implied_tubes.by_owner_noun.insert("BRAN".into(), 5);
        left.implied_tubes.degenerate.zero_length = 1;
        left.proven_non_graphic.insert("TEXT".into(), 2);
        left.not_wired.insert("HANDRA".into(), 40);
        left.evidence_pending.insert("NOZZ".into(), 2);
        left.composites_built.insert("NOZZ".into(), 1);
        left.noun_evidence.insert("NOZZ".into(), "ida".into());

        let mut right = Report::default();
        right.route_containers.insert("HANG".into(), 1);
        right.route_members.insert("ELBO".into(), 4);
        right.primitive_containers.insert("TMPL".into(), 7);
        right.host_negatives.insert("NCYL".into(), 2);
        right.implied_tubes.count = 2;
        right.implied_tubes.built = 2;
        right.implied_tubes.catalogue_od = 1;
        right.implied_tubes.bore_fallback = 1;
        right.implied_tubes.by_owner_noun.insert("HANG".into(), 2);
        right.implied_tubes.degenerate.bore_mismatch = 3;
        right.implied_tubes.degenerate.ppoint_db_not_open = 4;
        right
            .implied_tubes
            .degenerate
            .ppoint_db_not_open_by_dbno
            .insert("5053".into(), 4);
        right.proven_non_graphic.insert("TEXT".into(), 5);
        right.not_wired.insert("HANDRA".into(), 41);
        right.not_wired.insert("RNODE".into(), 7);
        right.evidence_pending.insert("NOZZ".into(), 3);
        right.composites_built.insert("NOZZ".into(), 4);
        right.noun_evidence.insert("NOZZ".into(), "newer".into());

        left.merge(right);

        assert_eq!(left.route_containers["BRAN"], 2);
        assert_eq!(left.route_containers["HANG"], 1);
        assert_eq!(left.route_members["ELBO"], 7, "同 noun 要相加而不是覆盖");
        assert_eq!(left.primitive_containers["TMPL"], 13);
        assert_eq!(left.host_negatives["NCYL"], 42);
        assert_eq!(left.implied_tubes.count, 7);
        assert_eq!(left.implied_tubes.built, 5);
        assert_eq!(left.implied_tubes.catalogue_od, 3);
        assert_eq!(left.implied_tubes.bore_fallback, 2);
        assert_eq!(left.implied_tubes.by_owner_noun["BRAN"], 5);
        assert_eq!(left.implied_tubes.by_owner_noun["HANG"], 2);
        assert_eq!(left.implied_tubes.degenerate.zero_length, 1);
        assert_eq!(left.implied_tubes.degenerate.bore_mismatch, 3);
        assert_eq!(left.implied_tubes.degenerate.ppoint_db_not_open, 4);
        assert_eq!(
            left.implied_tubes.degenerate.ppoint_db_not_open_by_dbno["5053"],
            4
        );
        assert_eq!(left.proven_non_graphic["TEXT"], 7);
        assert_eq!(left.not_wired["HANDRA"], 81);
        assert_eq!(left.not_wired["RNODE"], 7);
        assert_eq!(left.not_wired_total(), 88);
        assert_eq!(left.evidence_pending["NOZZ"], 5);
        assert_eq!(left.composites_built["NOZZ"], 5);
        assert_eq!(left.noun_evidence["NOZZ"], "ida");
    }

    /// `not_wired` 是交叉记账，不许挤进 `visited + consumed` 那道等式。
    ///
    /// 这本账里的每一件都已经在 `unknown_nouns` 或 `proven_non_graphic` 里数过一次；
    /// 让它进等式，`accounts_for` 就变成「元素数 + 一部分元素再数一遍」才平，
    /// 于是真丢了子树反而能靠这本账把差额补齐——判据被自己的观测数据毁掉。
    #[test]
    fn not_wired_stays_out_of_the_balance_equation() {
        let mut report = Report {
            visited: 10,
            ..Default::default()
        };
        report.consumed.insert("PLOO".into(), 4);
        report.unknown_nouns.insert("HANDRA".into(), 3);
        report.proven_non_graphic.insert("RNODE".into(), 2);
        assert!(report.accounts_for(14).is_ok());

        report.not_wired.insert("HANDRA".into(), 3);
        report.not_wired.insert("RNODE".into(), 2);
        assert!(
            report.accounts_for(14).is_ok(),
            "欠账账进来后 14 这个总数不该改变"
        );
        assert_eq!(report.not_wired_total(), 5);
        assert!(report.totals_line().contains("not_wired=5"));
    }

    /// 总长只要有一边没量过，合并结果就必须是 `None`。
    ///
    /// 把 `None` 当 0 加进去会得到一个偏小的总长，而它长得跟真数一模一样——
    /// 这类假数据比缺字段难查得多。
    #[test]
    fn an_unmeasured_total_length_poisons_the_sum_instead_of_counting_as_zero() {
        let mut measured = Report::default();
        measured.implied_tubes.total_length = Some(1200.0);
        let mut also_measured = Report::default();
        also_measured.implied_tubes.total_length = Some(300.0);
        let mut unmeasured = Report::default();

        let mut both = Report::default();
        both.implied_tubes.total_length = Some(1200.0);
        both.merge(also_measured);
        assert_eq!(both.implied_tubes.total_length, Some(1500.0));

        measured.merge(Report::default());
        assert_eq!(
            measured.implied_tubes.total_length, None,
            "对方没量过，合出来的总长就不能当成量过的"
        );

        unmeasured.merge(both);
        assert_eq!(unmeasured.implied_tubes.total_length, None);
    }
}

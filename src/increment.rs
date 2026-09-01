//! 模型增量更新：两个会话之间只重建受影响的元素。
//!
//! ## 管线分层（对标 core.dll / Core3D 的五层，取证见
//! `gen-model/docs/plans/2026-08-31-core-aligned-increment-architecture.md`）
//!
//! ```text
//! L0 会话定位   两端各钉一个 sesno 的 DbSet        ← switchToOldSession
//! L1 索引候选   collect_window → IndexCandidate    ← DB_IndexTableCompare【只是候选】
//! L2 逐元素差分 element_diff::diff_element         ← DB_Compare::checkEle【权威判据】
//! L3 语义账     ledger::ChangeLedger               ← DB_UserChanges
//! L4 单元上卷   plan_update → UpdatePlan           ← findTopLevelElement
//! ```
//!
//! **L1 只是候选过滤器，不是结论。** 索引说的 `Modified` 是「记录位置变了」＝
//! 记录被重写过，不等于内容变了。core 拿到候选一定回去做记录级、逐属性、按类型
//! 比值的差分（[`crate::element_diff`]），我们照做。少了这一层，「被重写」与
//! 「真的变了」分不开，而这两件事在真实语料里差着数量级。
//!
//! 三段式，段间只传数据：
//!
//! 1. **窗口收集** [`collect_window`]：e3d-io 的会话索引差分
//!    （对标 `core.dll::DB_IndexTableCompare`——删除 = 键集差、修改 = 记录位置变、
//!    flag 不参与判定）。先全量拿完 [`CompletedIndexDiff`] 再消费，差分中途失败
//!    整个窗口失败，没有半截结果可以拿去更新模型（ADR-017 整窗口口径）。
//! 2. **候选分类** [`plan_update`]：把索引级候选（refno + 增/删/改）折算成
//!    **模型单元**动作。模型单元的判据只有一份，在 [`is_model_unit`]——全量管线按它
//!    产出、这里按它上卷，两边各写一份名单就会静默漏件（见该函数文档记的那次事故）。
//!    与 Core3D `PartialUpdateDesiMgr` 的 significant/primitive 上卷同构：
//!    - 候选自己是单元 → 单元就是它；
//!    - 候选是被消费的数据（剖面环 / 负体，含 CMPF 里的深层负体）→ 沿 owner 链
//!      上卷到最近的单元属主（`SignificantOwner` 语义：从自身起判、无深度上限、
//!      List 容器天然被跨过）；
//!    - 删除的单元不上卷，直接出 Remove（Core R15 同款）；
//!    - 上卷路上跨过路由容器（含候选自身）→ 容器进派生集，名下隐式管身按两端键
//!      重算（见下「派生单元」一节）；
//!    - 改动落在没有单元归属、也不在任何路由射程里的元素上（容器改名、NonGraphic、
//!      未建的目录件）→ 记 `no_model` 账，一个都不许蒸发。
//! 3. **执行** [`execute_plan`]：逐单元重建（与全量管线同一条 [`model_element`]），
//!    产出 Upsert / Remove 动作。重建结果为 Skipped / 失败时**发 Remove**——
//!    Core 的次序是先擦后画，画不出来那一行就该保持擦掉的状态，绝不留陈旧网格。
//!    派生容器同款走 [`route_tubes_of`]，旧槽位键做差发 Remove。
//!
//! ## 与 Core3D 的一处有意分歧：变换级联
//!
//! 我们的产物是**世界系网格**（世界矩阵烘进顶点），Core3D 的图形段树在绘制时
//! 才 `CONCAT` 矩阵——所以容器（ZONE/STRU/…）POS/ORI 一动，Core 不用重画任何
//! 后代，我们却必须重建其子树内全部正体单元。而这些后代的**索引记录根本没变**
//! （世界位置是读取时沿 owner 链推导的），差分不会点名它们。
//!
//! **这一条不因为补齐 L2/L3 而消失，它是产物形态决定的。** core 的五层管线里
//! 没有任何一层对应 [`collect_unit_subtree`]，不是因为 core 更聪明，是因为
//! core 的产物是带矩阵的段树而我们的是烘死的三角形。照搬 core 把级联删掉，
//! `increment_real` 的 `Cascaded` 样本会当场证明它是静默缺件。
//!
//! L2 改的是**触发判据**，不是级联本身：从「两端 `(POS, ORI)` 不等」换成
//! [`ElementDiff::placement_input_changed`]。差别在于后者还认**改挂**——世界矩阵
//! 是沿 owner 链折叠出来的，链换了一条，`(POS, ORI)` 一个字没动，矩阵照样全变。
//! 那正是老实现漏掉的一半。
//!
//! ## 账本
//!
//! gen-model 宪法同款：静默缺件是最高级别缺陷。每个候选必须落进
//! `rolled_up` / `no_model` / `unresolved` 三本账之一（[`IncrementReport::accounts_for`]
//! 硬判据），执行侧 `regenerated + regen_skipped + regen_failed == units_to_regenerate`。
//!
//! ## 派生单元：隐式管身的增量走另一条腿
//!
//! 路由容器不是模型单元——它自己 refno 上没有「一件」几何。其中 RVM 坐实会补管
//! 的四族（[`crate::category::DERIVED_ROUTE_CONTAINER_NOUNS`]，HANG 除外：E3D
//! 不在 HANG 成员间补管）名下派生出 N 根按
//! [`crate::elmodl::GeometryId::ImpliedTube`]（两端站点 + 同站点对内出现序）
//! 索引的管身。它们的增量动作因此与模型单元不同：
//!
//! - **计划**：上卷路上跨过路由容器（[`is_derived_unit`]，含候选自身）、或级联
//!   子树里扫到路由容器，都进 [`UpdatePlan::regenerate_derived`]；同时在 **base 端**
//!   用 [`route_slot_keys`] 收这些容器的全部旧槽位键（[`UpdatePlan::derived_stale`]）。
//!   旧键收的是**全部槽位**而不是「上次真建成的那批」——计划期不建几何，无从知道
//!   哪些槽位当时退化了。多出来的 Remove 落在从未存在的键上，而 Remove 本就是
//!   幂等的「确保不存在」，宁多勿漏。
//! - **执行**：容器在 target 端还活着就 [`route_tubes_of`] 整链重算（与全量管线
//!   同一段串接代码），产出 upsert；已删或不再是路由容器则一根不建（retired）。
//!   旧键里没被这一轮重建接住的，统一发 Remove——与模型单元同一条
//!   「先擦后画，画不出保持擦掉」。
//!
//! 真库门今天在这条腿上没有判别力（门只开 DESI 库，P 点解不出、两端全量
//! `implied_tubes.built` 都是 0，管身键两侧都不出现）；等目录库进门它自动生效。
//! **目录件自身内容变了仍不在射程内**——那是跨库失效通道的活（设计文档 §3.4），
//! 不做进窗口差分。
//!
//! [`world_matrix`]: crate::transform::world_matrix

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;
use std::sync::Arc;

use e3d_io::db_element::{DbElementError, DbSet};
use e3d_io::error::E3dReaderError;
use e3d_io::index::{CompletedIndexDiff, IndexCandidate, IndexDiff, IndexTreeError};
use e3d_io::page::PageId;
use e3d_io::refno::RefNo;
use e3d_io::session::DbView;

use crate::catalogue::CatalogueMeshCache;
use crate::category::{classify, is_derived_unit, is_model_unit};
use crate::element_diff::{ElementDiff, diff_element};
use crate::elmodl::{GeneratedElement, GeometryId, Modeled, model_element_with_cache};
use crate::ledger::{ChangeLedger, ChangeTally};
use crate::pipeline::{Incident, route_tubes_of};
use crate::route::route_slot_keys;

/// 上卷 / 子树遍历的环保护。dabacon 树深实测个位数（与 `transform` 同一口径）。
const MAX_ANCESTOR_DEPTH: usize = 128;

#[derive(Debug, thiserror::Error)]
pub enum IncrementError {
    #[error(transparent)]
    Io(#[from] E3dReaderError),
    #[error("索引差分失败，窗口作废：{0}")]
    Index(#[from] IndexTreeError),
    #[error("会话 {sesno} 不在 {file} 的会话链上（链上有 {available:?}）")]
    UnknownSession {
        file: String,
        sesno: u32,
        available: Vec<u32>,
    },
    #[error("窗口不是正向的：base={base} 必须在 target={target} 之前（按链序，不是数字大小）")]
    WindowNotForward { base: u32, target: u32 },
}

/// 一个收集完成的增量窗口：两端会话 + 全量差分结果。
pub struct WindowDiff {
    pub base_sesno: u32,
    pub target_sesno: u32,
    pub base_root: PageId,
    pub target_root: PageId,
    pub diff: CompletedIndexDiff,
}

/// 收集 `base_sesno → target_sesno` 的索引差分窗口。
///
/// 两个端点都必须在文件的会话链上，且 base 在 target 之前（**链序**为准——
/// 链不保证连号，数字大小不算数）。差分任一步失败整体失败。
pub fn collect_window(
    db_file: &Path,
    base_sesno: u32,
    target_sesno: u32,
) -> Result<WindowDiff, IncrementError> {
    let mut view = DbView::open(db_file)?;
    let sessions = view.walk_sessions()?;
    let position = |sesno: u32| sessions.iter().position(|s| s.sesno == sesno);
    let missing = |sesno: u32| IncrementError::UnknownSession {
        file: db_file.display().to_string(),
        sesno,
        available: sessions.iter().map(|s| s.sesno).collect(),
    };
    let target_at = position(target_sesno).ok_or_else(|| missing(target_sesno))?;
    let base_at = position(base_sesno).ok_or_else(|| missing(base_sesno))?;
    // 链是新→旧，base 必须比 target 旧 = 位置更靠后。
    if base_at <= target_at {
        return Err(IncrementError::WindowNotForward {
            base: base_sesno,
            target: target_sesno,
        });
    }
    let base_root = sessions[base_at].index_root;
    let target_root = sessions[target_at].index_root;
    let diff = IndexDiff::new(view.pages_mut(), base_root, target_root).finish()?;
    Ok(WindowDiff {
        base_sesno,
        target_sesno,
        base_root,
        target_root,
        diff,
    })
}

/// 增量总账（可直接 serde_json 落盘）。
#[derive(Debug, Default, serde::Serialize)]
pub struct IncrementReport {
    pub base_sesno: u32,
    pub target_sesno: u32,
    /// 索引差分候选总数（inserted + modified + deleted）。
    pub candidates: usize,
    pub inserted: usize,
    pub modified: usize,
    pub deleted: usize,
    /// 位置相同仅 flag 不同的键数。真实语料上恒 0，非零值得人看（e3d-io R5 口径）。
    pub flag_only_drifts: u64,
    /// L3 语义账：这一窗的改动各是什么（对标 `DB_UserChanges` 的语义桶）。
    pub changes: ChangeTally,
    /// 索引点了名、L2 逐字节比下来内容一个字没动的候选数。
    ///
    /// 它们既没上卷也不是 `no_model`，单独一本账——这是 L1 与 L2 之间的落差，
    /// 与「这个改动落在没有模型归属的元素上」是两回事。老实现没有 L2，
    /// 这些候选一律当「变了」照常上卷重建。
    pub unchanged: usize,
    /// 折算进了某个单元动作（重建 / 移除，含被去重掉的）的候选数。
    pub rolled_up: usize,
    /// 没落到任何模型单元的候选：noun → 数量（容器改名、NonGraphic、目录件……）。
    pub no_model: BTreeMap<String, usize>,
    /// 计划重算派生几何的路由容器：容器 noun → 个数（按容器去重，不按候选）。
    ///
    /// 这不是第四本账——点名这些容器的候选已按各自去向记进 `rolled_up` /
    /// `no_model`，这里是横切的一笔，让「这一窗动了哪些链」一眼数得出来。
    pub derived_planned: BTreeMap<String, usize>,
    /// base 端旧槽位键读取失败的容器（容器在 base 端存在、链却走不通）。
    ///
    /// 出现即数据或读层问题，且后果与 `unresolved` 同级：说不出旧键就删不掉
    /// 旧管身，静默陈旧与静默缺件是同一级缺陷。不并进 `unresolved`——那本账
    /// 按候选计数、参与候选恒等式，这本按容器计。
    pub derived_stale_unreadable: Vec<Incident>,
    /// 连分类都做不了的候选（端点读不出 noun / owner 链断）。出现即数据或读层问题。
    pub unresolved: Vec<Incident>,
    /// 变换级联：哪个元素的世界矩阵输入变了、扇出了几个单元。
    ///
    /// 触发判据是 [`ElementDiff::placement_input_changed`]，含 `POS`/`ORI` 与**改挂**。
    pub cascades: Vec<Incident>,
    /// 计划出的单元数。
    pub units_to_regenerate: usize,
    pub units_to_remove: usize,
    /// 计划重算派生几何的容器数（= `derived_planned` 各项之和）。
    pub derived_to_regenerate: usize,
    /// 执行账：三者之和必须等于 `units_to_regenerate`。
    pub regenerated: usize,
    pub regen_skipped: Vec<Incident>,
    pub regen_failed: Vec<Incident>,
    /// 派生执行账：三者之和必须等于 `derived_to_regenerate`。
    ///
    /// `derived_regenerated` = 容器在 target 端活着、整链重算过（可能一根管身都
    /// 没建成——退化槽位不出几何，但旧键照样会被清）。
    pub derived_regenerated: usize,
    /// 容器在 target 端已删、或类型变得不再是路由容器：不重算，旧键全清。
    pub derived_retired: usize,
    /// 容器在 target 端读挂 / 链走不通：与模型单元的 `regen_failed` 同款，
    /// 旧键全清（先擦后画，画不出保持擦掉），且必须响。
    pub derived_failed: Vec<Incident>,
    /// 这一轮 upsert 的隐式管身根数（横切计数，已含在 `upserts` 里）。
    pub tube_upserts: usize,
    /// 这一轮 Remove 的管身键数（横切计数，已含在 `removals` 里）。
    pub tube_removals: usize,
    /// 非致命观察（级联检查读挂但单元已入账、成员表疑似回环等）。
    pub notes: Vec<Incident>,
}

impl IncrementReport {
    /// 账要闭合：每个候选恰好落一本账；每个待重建单元恰好有一个执行结论。
    pub fn accounts_for(&self) -> Result<(), String> {
        let classified = self.rolled_up
            + self.unchanged
            + self.no_model.values().sum::<usize>()
            + self.unresolved.len();
        if classified != self.candidates {
            return Err(format!(
                "候选账不平：rolled_up {} + unchanged {} + no_model {} + unresolved {} \
                 = {classified}，候选 {}",
                self.rolled_up,
                self.unchanged,
                self.no_model.values().sum::<usize>(),
                self.unresolved.len(),
                self.candidates,
            ));
        }
        let executed = self.regenerated + self.regen_skipped.len() + self.regen_failed.len();
        if executed != self.units_to_regenerate {
            return Err(format!(
                "执行账不平：regenerated {} + skipped {} + failed {} = {executed}，计划 {}",
                self.regenerated,
                self.regen_skipped.len(),
                self.regen_failed.len(),
                self.units_to_regenerate,
            ));
        }
        let derived = self.derived_regenerated + self.derived_retired + self.derived_failed.len();
        if derived != self.derived_to_regenerate {
            return Err(format!(
                "派生执行账不平：regenerated {} + retired {} + failed {} = {derived}，计划 {}",
                self.derived_regenerated,
                self.derived_retired,
                self.derived_failed.len(),
                self.derived_to_regenerate,
            ));
        }
        Ok(())
    }

    pub fn totals_line(&self) -> String {
        format!(
            "window {}→{}: candidates={} (+{} ~{} -{}) rolled_up={} unchanged={} no_model={} \
             unresolved={} cascades={} units regen={} remove={} derived={} | \
             regenerated={} skipped={} failed={} | derived regen={} retired={} failed={} \
             tubes +{} -{}",
            self.base_sesno,
            self.target_sesno,
            self.candidates,
            self.inserted,
            self.modified,
            self.deleted,
            self.rolled_up,
            self.unchanged,
            self.no_model.values().sum::<usize>(),
            self.unresolved.len(),
            self.cascades.len(),
            self.units_to_regenerate,
            self.units_to_remove,
            self.derived_to_regenerate,
            self.regenerated,
            self.regen_skipped.len(),
            self.regen_failed.len(),
            self.derived_regenerated,
            self.derived_retired,
            self.derived_failed.len(),
            self.tube_upserts,
            self.tube_removals,
        )
    }

    /// L3 语义账那一行：这一窗到底发生了什么。
    pub fn changes_line(&self) -> String {
        let ChangeTally {
            created,
            deleted,
            type_changed,
            reparented,
            members_changed,
            attributes_modified,
            opaque_record_changes,
            unchanged,
            created_subtree_roots,
            deleted_subtree_roots,
        } = self.changes;
        format!(
            "changes: created={created}（子树顶 {created_subtree_roots}） \
             deleted={deleted}（子树顶 {deleted_subtree_roots}） type_changed={type_changed} \
             reparented={reparented} members={members_changed} attrs={attributes_modified} \
             opaque={opaque_record_changes} unchanged={unchanged}"
        )
    }
}

/// 分类完成、还没执行的更新计划。键用 `(word0, word1)`（`RefNo` 不带 `Ord`，
/// 这里只要一个稳定的去重序，不承载索引侧的无符号比较语义）。
pub struct UpdatePlan {
    pub regenerate: BTreeSet<(u32, u32)>,
    pub remove: BTreeSet<(u32, u32)>,
    /// 需要重算名下派生几何的容器（[`is_derived_unit`]，今天只有会补管身的
    /// 那四族路由容器，HANG 不在内）。
    ///
    /// 与 `regenerate` 分开，因为**执行动作不同**：模型单元是「按 refno 删一件画
    /// 一件」，这些是「两端各算一遍链上的槽位键，新的 upsert、旧键里没被接住的
    /// remove」。混进 `regenerate` 会让 `execute_plan` 对着容器 refno 调
    /// [`model_element`]，那是一件根本不存在的东西。
    pub regenerate_derived: BTreeSet<(u32, u32)>,
    /// 上面那些容器在 **base 端**名下的全部槽位键。
    ///
    /// 在这里算而不是留给 `execute_plan`：`execute_plan` 按设计只读 target
    /// （文档里那句「与全量管线走同一条 `model_element`」），而旧键只有 base 说了算。
    /// [`route_slot_keys`] 只走成员表、不建几何，正好是 `plan_update`「只读两端
    /// DbSet」的活。
    pub derived_stale: BTreeSet<GeometryId>,
    /// L3 语义账。计数已经折进 [`IncrementReport::changes`]，这里保留逐条明细，
    /// 给探针与真库门查「这一窗到底改了什么」用。
    pub ledger: ChangeLedger,
    pub report: IncrementReport,
}

/// 一次增量更新的完整产出：模型集上的动作 + 总账。
///
/// 两侧**同一把键**：`upserts` 里每件自带 [`GeneratedElement::geometry_id`]，
/// `removals` 也说 [`GeometryId`]。`removals` 是「确保不存在」（幂等，目标本来
/// 不在模型集里也不算错）。两个列表都已排序去重。
///
/// # 这里曾经是 `Vec<RefNo>`
///
/// 上行按 [`GeometryId`] 索引、下行按 refno 发 Remove，是同一条通路上的两种键。
/// 一个路由容器名下有 N 根隐式管身共享同一个 refno（`pipeline` 绝不制造伪 refno），
/// 于是**加得进一根管身，永远删不掉一根**。换成同一把键之后，派生单元那条腿
/// （模块文档「派生单元」一节）才接得通：管身的 Remove 说得出是哪一根。
pub struct IncrementOutcome {
    pub upserts: Vec<GeneratedElement>,
    pub removals: Vec<GeometryId>,
    pub report: IncrementReport,
}

fn key(refno: RefNo) -> (u32, u32) {
    (refno.word0, refno.word1)
}

fn refno_of(key: (u32, u32)) -> RefNo {
    RefNo::new(key.0, key.1)
}

/// 一件挂在自己 refno 上的实体的键。模型单元走这条。
fn element_id(unit: (u32, u32)) -> GeometryId {
    GeometryId::Element {
        refno: refno_of(unit).to_string(),
    }
}

/// 一根隐式管身的键：容器 + 同站点对内出现序 + 两个站点
/// （[`crate::route::route_slot_keys`] 给的那三元组）。
fn tube_id(container: RefNo, slot: (u32, RefNo, RefNo)) -> GeometryId {
    GeometryId::ImpliedTube {
        container_refno: container.to_string(),
        route_ordinal: slot.0,
        from_refno: slot.1.to_string(),
        to_refno: slot.2.to_string(),
    }
}

fn incident(refno: RefNo, noun: impl Into<String>, detail: impl Into<String>) -> Incident {
    Incident {
        refno: refno.to_string(),
        noun: noun.into(),
        detail: detail.into(),
    }
}

/// 一次上卷的结果。
struct Rollup {
    /// 起点元素自己的 noun。上卷本来就要读它，顺手交出来给账本用，省一次记录解析。
    noun: String,
    /// 起点元素的属主。L3 的祖先抢占要它（`isElementTopLevelCreate` 的判据就是
    /// 「我的属主在不在同一个桶里」）。
    owner: Option<RefNo>,
    /// 最近的模型单元（含自身）。`None` = 这条改动上方没有任何会被建模的东西。
    unit: Option<RefNo>,
    /// 路上跨过的最近一个路由容器（含自身）。它不会成为 `unit`——[`is_model_unit`]
    /// 有意排除它，因为它自己 refno 上没有「一件」几何——而是走 [`is_derived_unit`]
    /// 那条路，让整条链上的隐式管身按两端键重算一遍。
    ///
    /// 带上 refno 而不只是 noun：光有 noun 只够记一笔「这一窗可能有管身陈旧了」，
    /// 点不出是哪条链，也就动不了手。
    route_container: Option<(RefNo, String)>,
}

/// 沿 owner 链同时找最近的模型单元与最近的路由容器（都含自身）。判据分别是
/// [`is_model_unit`]（与全量管线同一份）与 [`is_derived_unit`]。
/// `SignificantOwner` 语义：从自身起判、跨过一切不命中的属主。
///
/// # 两条搜索必须解耦：命中单元不许提前收工
///
/// 这里一度是「碰到模型单元立即返回」。那对单元搜索没错，却把容器搜索一并砍了：
/// 链上**自身是模型单元**的成员（SBFI 是现成的例子——目录件里唯独它出几何；
/// SUPC 族名下挂型钢件同理）一挪位，早退让上层的 BRAN/SUPC 永远进不了
/// `regenerate_derived`，它两侧管身的世界坐标就静默过期了。级联也救不了——
/// [`collect_unit_subtree`] 只收候选**子树内**，不收它上方的容器。
/// 所以命中单元后继续上爬，直到容器也有了、或到根。
fn nearest_unit(set: &Arc<DbSet>, refno: RefNo) -> Result<Rollup, DbElementError> {
    let mut current = set.element(refno);
    let start_noun = current.element_type()?;
    let start_owner = current.owner()?.map(|owner| owner.refno());
    let mut noun = start_noun.clone();
    let mut depth = 0usize;
    let mut unit: Option<RefNo> = None;
    let mut route_container: Option<(RefNo, String)> = None;
    loop {
        if unit.is_none() && is_model_unit(&noun) {
            unit = Some(current.refno());
        }
        if route_container.is_none() && is_derived_unit(&noun) {
            route_container = Some((current.refno(), noun.clone()));
        }
        if unit.is_some() && route_container.is_some() {
            break;
        }
        match current.owner()? {
            // owner 指向自己是 dabacon 的另一种根形态，不是环——真库门的
            // `roots_at` 就是这个口径（「owner 指向自己、无效、或不在本库索引里」
            // 都算根）。判成 OwnerCycle 会把这类根下整棵子树的候选统统打进
            // unresolved，报的是「数据问题」而数据没有问题。
            // 真正的环（A→B→…→A）仍由 MAX_ANCESTOR_DEPTH 兜住。
            Some(owner) if owner.refno() == current.refno() => break,
            Some(owner) => {
                depth += 1;
                if depth > MAX_ANCESTOR_DEPTH {
                    return Err(DbElementError::OwnerCycle {
                        start: refno,
                        at: owner.refno(),
                    });
                }
                current = owner;
                noun = current.element_type()?;
            }
            None => break,
        }
    }
    Ok(Rollup {
        noun: start_noun,
        owner: start_owner,
        unit,
        route_container,
    })
}

/// 收集 `root` 子树内（不含 root 自身）全部模型单元与派生单元。
/// 返回子树内**命中**的单元总数（模型 + 派生，含已在计划集里的）。
///
/// # fanout 数的是命中，不是新插入
///
/// 返回值决定调用方的 `contributed`（这个候选折没折进单元动作）。这里一度返回
/// 「新插进集合的个数」：子树单元恰好已全部被同窗其它候选点名时返回 0，候选被
/// 记成 `no_model`——而那本账的语义是「改动落在没有模型归属的元素上」，它明明有。
/// 去重是计划集自己的事，不该改写「这个候选贡献了什么」；[`plan_update`] 里
/// `mark_derived` 对同一件事早已是这个口径（容器已在计划里，候选照样算 rolled_up）。
///
/// # 为什么派生单元也要一起收
///
/// 我们的产出是**世界坐标网格**。一个 ZONE 挪了位置，它名下每根隐式管身的顶点
/// 全都过期了，可路由容器不是模型单元、[`is_model_unit`] 那一关根本拦不住它——
/// 只收 `out` 的话，级联会不声不响地把整片路由留在原地。真库门今天抓不到这个：
/// `implied_tubes.built` 还是 0，没有管身可对拍。等目录求值一落地它立刻变成
/// 「全量说管身动了、增量说没动」的静默分歧，所以现在就得收全。
///
/// 消费关系不影响收集：被消费的子树（剖面环、负体、CMPF）里本就没有单元，
/// 走一遍只是白读几页。成员表出现回环用 visited 兜住并记 note——那是数据坏，
/// 不该让级联把进程挂死。
fn collect_unit_subtree(
    set: &Arc<DbSet>,
    root: RefNo,
    out: &mut BTreeSet<(u32, u32)>,
    derived: &mut BTreeSet<(u32, u32)>,
    notes: &mut Vec<Incident>,
) -> Result<usize, DbElementError> {
    let mut hits = 0usize;
    let mut visited: BTreeSet<(u32, u32)> = BTreeSet::new();
    let mut stack = vec![set.element(root)];
    visited.insert(key(root));
    while let Some(el) = stack.pop() {
        for member in el.members()? {
            if !visited.insert(key(member.refno())) {
                notes.push(incident(
                    member.refno(),
                    member.element_type().unwrap_or_else(|_| "?".into()),
                    format!("成员表回环（{root} 子树级联遍历中二次到达），已跳过"),
                ));
                continue;
            }
            let noun = member.element_type()?;
            if is_model_unit(&noun) {
                out.insert(key(member.refno()));
                hits += 1;
            }
            if is_derived_unit(&noun) {
                derived.insert(key(member.refno()));
                hits += 1;
            }
            stack.push(member);
        }
    }
    Ok(hits)
}

/// 把索引候选折算成模型单元动作。只读两端 `DbSet`，不建任何几何。
pub fn plan_update(base: &Arc<DbSet>, target: &Arc<DbSet>, window: &WindowDiff) -> UpdatePlan {
    let mut report = IncrementReport {
        base_sesno: window.base_sesno,
        target_sesno: window.target_sesno,
        candidates: window.diff.changes.len(),
        inserted: window.diff.tally.inserted as usize,
        modified: window.diff.tally.modified as usize,
        deleted: window.diff.tally.deleted as usize,
        flag_only_drifts: window.diff.diagnostics.flag_only_drifts,
        ..IncrementReport::default()
    };
    let mut ledger = ChangeLedger::new();
    let mut regenerate: BTreeSet<(u32, u32)> = BTreeSet::new();
    let mut remove: BTreeSet<(u32, u32)> = BTreeSet::new();
    let mut regenerate_derived: BTreeSet<(u32, u32)> = BTreeSet::new();
    // 删除候选上卷到的属主单元。属主自己也被删时要归并进那条 Remove，
    // 所以等 remove 集收齐了再定去向。
    let mut deferred: Vec<RefNo> = Vec::new();

    fn note_no_model(report: &mut IncrementReport, noun: &str) {
        *report.no_model.entry(noun.to_string()).or_insert(0) += 1;
    }

    /// 上卷跨过了路由容器就把容器记进派生集。返回值 = 这个候选折进了派生动作
    /// （去重不影响折算：容器已在计划里，候选照样算 rolled_up）。
    fn mark_derived(derived: &mut BTreeSet<(u32, u32)>, rollup: &Rollup) -> bool {
        match &rollup.route_container {
            Some((container, _)) => {
                derived.insert(key(*container));
                true
            }
            None => false,
        }
    }

    for change in &window.diff.changes {
        match *change {
            IndexCandidate::Inserted { refno, .. } => match nearest_unit(target, refno) {
                Ok(rollup) => {
                    ledger.created(refno, rollup.noun.clone(), rollup.owner);
                    let mut contributed = mark_derived(&mut regenerate_derived, &rollup);
                    if let Some(unit) = rollup.unit {
                        regenerate.insert(key(unit));
                        contributed = true;
                    }
                    if contributed {
                        report.rolled_up += 1;
                    } else {
                        note_no_model(&mut report, &rollup.noun);
                    }
                }
                Err(error) => {
                    report.unresolved.push(incident(
                        refno,
                        "?",
                        format!("新增候选上卷失败：{error}"),
                    ));
                }
            },
            IndexCandidate::Deleted { refno, .. } => match nearest_unit(base, refno) {
                Ok(rollup) => {
                    ledger.deleted(refno, rollup.noun.clone(), rollup.owner);
                    // 容器记的是 base 端的链：被删的路由构件只有 base 认得它挂在哪条
                    // 链上；容器自己被删也走这条（执行侧 retired，旧键全清）。
                    let mut contributed = mark_derived(&mut regenerate_derived, &rollup);
                    match rollup.unit {
                        // 单元自己被删 → 直接 Remove（Core R15 同款，不再上卷）。
                        Some(unit) if unit == refno => {
                            remove.insert(key(unit));
                            contributed = true;
                        }
                        Some(unit) => {
                            deferred.push(unit);
                            contributed = true;
                        }
                        None => {}
                    }
                    if contributed {
                        report.rolled_up += 1;
                    } else {
                        note_no_model(&mut report, &rollup.noun);
                    }
                }
                Err(error) => {
                    report.unresolved.push(incident(
                        refno,
                        "?",
                        format!("删除候选上卷失败：{error}"),
                    ));
                }
            },
            IndexCandidate::Modified { refno, .. } => {
                // L2：索引只说记录被重写过，这一步才回答到底哪里变了。
                let diff = match diff_element(base, target, refno) {
                    Ok(diff) => diff,
                    Err(error) => {
                        report.unresolved.push(incident(
                            refno,
                            "?",
                            format!("逐元素差分失败：{error}"),
                        ));
                        continue;
                    }
                };
                ledger.record(refno, &diff);
                if diff.is_unchanged() {
                    report.unchanged += 1;
                    continue;
                }

                let mut contributed = false;
                let mut resolved = true;
                match nearest_unit(target, refno) {
                    Ok(rollup) => {
                        contributed = mark_derived(&mut regenerate_derived, &rollup);
                        if let Some(unit) = rollup.unit {
                            regenerate.insert(key(unit));
                            contributed = true;
                        }
                    }
                    Err(error) => {
                        resolved = false;
                        report.unresolved.push(incident(
                            refno,
                            "?",
                            format!("修改候选上卷失败：{error}"),
                        ));
                    }
                }

                // 改挂：旧属主那一侧的单元也要重建。core 的
                // `DB_UserChanges::elementIncluded(el, 旧属主)` 记的正是旧属主——
                // 一个 PLOO 从 PANE1 挪到 PANE2，PANE1 少了一个剖面环，同样得重画。
                // 只上卷 target 端就永远看不见 PANE1。
                //
                // 类型变更同款回 base 端：它从前是什么只有 base 说得出。BRAN 变成
                // 别的 noun，旧管身的键只有 base 端的链认得（容器进派生集，执行侧
                // retired 清旧键）；单元变成非单元，旧几何也得有人发 Remove
                // （base 端上卷进 regenerate，执行侧按 target 分类走 skip + Remove）。
                if resolved && (diff.owner_changed.is_some() || diff.type_changed.is_some()) {
                    match nearest_unit(base, refno) {
                        Ok(rollup) => {
                            contributed |= mark_derived(&mut regenerate_derived, &rollup);
                            if let Some(unit) = rollup.unit {
                                regenerate.insert(key(unit));
                                contributed = true;
                            }
                        }
                        Err(error) => {
                            resolved = false;
                            report.unresolved.push(incident(
                                refno,
                                "?",
                                format!("改挂/类型变更后回溯 base 端失败：{error}"),
                            ));
                        }
                    }
                }

                // 变换级联（见模块文档「与 Core3D 的一处有意分歧」）。
                if resolved && diff.placement_input_changed() {
                    match collect_unit_subtree(
                        target,
                        refno,
                        &mut regenerate,
                        &mut regenerate_derived,
                        &mut report.notes,
                    ) {
                        Ok(fanout) => {
                            report.cascades.push(incident(
                                refno,
                                diff.noun.clone(),
                                format!(
                                    "{}，级联重建子树内 {fanout} 个单元（含派生容器）",
                                    cascade_reason(&diff)
                                ),
                            ));
                            contributed = contributed || fanout > 0;
                        }
                        Err(error) => {
                            // 级联收集失败 = 有一片子树的世界矩阵可能已过期而我们
                            // 点不出名字——这不是 note 级别的观察，必须亮成
                            // unresolved 才对得起「静默缺件」那条宪法。
                            resolved = false;
                            report.unresolved.push(incident(
                                refno,
                                diff.noun.clone(),
                                format!("世界矩阵输入变了但级联收集失败：{error}"),
                            ));
                        }
                    }
                }

                if resolved {
                    if contributed {
                        report.rolled_up += 1;
                    } else {
                        note_no_model(&mut report, &diff.noun);
                    }
                }
            }
        }
    }

    for unit in deferred {
        // 属主也被删了 → 它自己的 Deleted 候选已经出了 Remove；还活着 → 重建。
        if !remove.contains(&key(unit)) {
            regenerate.insert(key(unit));
        }
    }
    // Remove 赢：同一单元既要删又要建说明分类端点混了，删除语义优先。
    for gone in &remove {
        regenerate.remove(gone);
    }

    // 派生容器的旧槽位键 + 计划账。旧键只有 base 说了算（`execute_plan` 按设计
    // 只读 target），且收的是**全部槽位**而非「上次真建成的那批」——计划期不建
    // 几何，无从知道哪些槽位当时退化；多出来的 Remove 落在从未存在的键上，
    // 幂等无害（见模块文档「派生单元」）。
    let mut derived_stale: BTreeSet<GeometryId> = BTreeSet::new();
    for &container in &regenerate_derived {
        let refno = refno_of(container);
        let noun = target
            .element(refno)
            .element_type()
            .or_else(|_| base.element(refno).element_type())
            .unwrap_or_else(|_| "?".into());
        *report.derived_planned.entry(noun.clone()).or_insert(0) += 1;
        match route_slot_keys(&base.element(refno)) {
            Ok(slots) => {
                for slot in slots {
                    derived_stale.insert(tube_id(refno, slot));
                }
            }
            // 容器自己在 base 端不存在：这一窗新建的链，没有旧键可清。
            // 只豁免容器本身——链走到一半某个成员读不出是数据问题，得响。
            Err(DbElementError::NotFound { refno: missing, .. }) if missing == refno => {}
            Err(error) => {
                report.derived_stale_unreadable.push(incident(
                    refno,
                    noun,
                    format!("base 端旧槽位键读不出，旧管身可能清不掉：{error}"),
                ));
            }
        }
    }

    ledger.finish();
    report.changes = ledger.tally();
    report.units_to_regenerate = regenerate.len();
    report.units_to_remove = remove.len();
    report.derived_to_regenerate = regenerate_derived.len();
    UpdatePlan {
        regenerate,
        remove,
        regenerate_derived,
        derived_stale,
        ledger,
        report,
    }
}

/// 级联理由的可读形态。报告里要说得出是「谁动了」，不能只有一个 fanout 数字。
fn cascade_reason(diff: &ElementDiff) -> String {
    if diff.type_changed.is_some() {
        return "类型变更".into();
    }
    if diff.owner_changed.is_some() {
        return "改挂（owner 链换了一条，局部 POS/ORI 未必动）".into();
    }
    if diff.opaque {
        return "记录变了但指不出哪一项，保守级联".into();
    }
    let moved: Vec<&str> = crate::transform::PLACEMENT_ATTRIBUTES
        .into_iter()
        .filter(|name| diff.attributes.iter().any(|changed| changed == name))
        .collect();
    format!("{} 变了", moved.join("/"))
}

/// 执行计划：逐单元重建。与全量管线走同一条 [`model_element`]，产出动作集；
/// 派生容器走 [`route_tubes_of`]（与全量同一段串接代码），旧键做差发 Remove。
pub fn execute_plan(target: &Arc<DbSet>, plan: UpdatePlan) -> IncrementOutcome {
    let UpdatePlan {
        regenerate,
        remove,
        regenerate_derived,
        derived_stale,
        ledger: _,
        mut report,
    } = plan;
    let mut upserts: Vec<GeneratedElement> = Vec::new();
    let mut removals: BTreeSet<GeometryId> = remove.into_iter().map(element_id).collect();
    // 一次执行共享一个世代网格池：级联重建（容器一挪、整片 PANE 重来）里
    // 同形状只建一次网格；命中克隆与直建逐位一致，不改任何单元的产出。
    let mut mesh_cache = CatalogueMeshCache::default();

    for unit in regenerate {
        let refno = refno_of(unit);
        let el = target.element(refno);
        let noun = match el.element_type() {
            Ok(noun) => noun,
            Err(error) => {
                removals.insert(element_id(unit));
                report
                    .regen_failed
                    .push(incident(refno, "?", format!("读不出 noun：{error}")));
                continue;
            }
        };
        let category = classify(&noun);
        if !is_model_unit(&noun) {
            // 合法到达：类型变更把 base 端的单元送了进来，而它在 target 端的新 noun
            // 不再出几何——与 `Modeled::Skipped` 同款处置（先擦后画，画不出保持
            // 擦掉），不算失败。计划集没被手工改过的话，这里只有类型变更能到。
            removals.insert(element_id(unit));
            report.regen_skipped.push(incident(
                refno,
                noun,
                format!("target 端分类为 {category:?}，不成模型单元（类型变更后不再出几何）"),
            ));
            continue;
        }
        match model_element_with_cache(&el, category, &mut mesh_cache) {
            Ok(Modeled::Solid(generated)) => {
                report.regenerated += 1;
                upserts.push(*generated);
            }
            Ok(Modeled::Skipped { reason }) => {
                // E3D 同样不出几何 → 模型集里不该有它的行（先擦后画，画不出保持擦掉）。
                removals.insert(element_id(unit));
                report.regen_skipped.push(incident(refno, noun, reason));
            }
            Err(error) => {
                removals.insert(element_id(unit));
                report
                    .regen_failed
                    .push(incident(refno, noun, error.to_string()));
            }
        }
    }

    // 派生容器：target 端整链重算，旧键里没被接住的发 Remove。
    // `kept` 只收这一轮真建成的管身键——退化槽位不出几何，它的旧键就该被清掉。
    let mut kept: BTreeSet<GeometryId> = BTreeSet::new();
    for container in regenerate_derived {
        let refno = refno_of(container);
        let el = target.element(refno);
        match el.element_type() {
            // 容器在 target 端已删：退休，旧键由下面的差集统一清。
            Err(DbElementError::NotFound { .. }) => {
                report.derived_retired += 1;
            }
            Err(error) => {
                report
                    .derived_failed
                    .push(incident(refno, "?", format!("读不出 noun：{error}")));
            }
            Ok(noun) if !is_derived_unit(&noun) => {
                // 类型变更把容器变成了别的东西：名下不该再有管身，旧键全清。
                report.derived_retired += 1;
                report.notes.push(incident(
                    refno,
                    noun,
                    "计划时是路由容器，target 端类型已变，按退休清掉旧管身",
                ));
            }
            Ok(noun) => match route_tubes_of(&el) {
                Ok(tubes) => {
                    report.derived_regenerated += 1;
                    for tube in tubes {
                        kept.insert(tube.geometry_id.clone());
                        report.tube_upserts += 1;
                        upserts.push(tube);
                    }
                }
                Err(error) => {
                    report
                        .derived_failed
                        .push(incident(refno, noun, error.to_string()));
                }
            },
        }
    }
    for stale in derived_stale {
        if !kept.contains(&stale) && removals.insert(stale) {
            report.tube_removals += 1;
        }
    }

    upserts.sort_by(|left, right| left.geometry_id.cmp(&right.geometry_id));
    IncrementOutcome {
        upserts,
        removals: removals.into_iter().collect(),
        report,
    }
}

/// 分类 + 执行一步到位。
pub fn increment_update(
    base: &Arc<DbSet>,
    target: &Arc<DbSet>,
    window: &WindowDiff,
) -> IncrementOutcome {
    let plan = plan_update(base, target, window);
    execute_plan(target, plan)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn incident_n(n: usize) -> Vec<Incident> {
        (0..n)
            .map(|i| Incident {
                refno: format!("1/{i}"),
                noun: "PANE".into(),
                detail: "test".into(),
            })
            .collect()
    }

    /// 候选账：三本账之和必须等于候选数，多一少一都要响。
    #[test]
    fn candidate_accounting_must_close() {
        let mut report = IncrementReport {
            candidates: 5,
            rolled_up: 3,
            unresolved: incident_n(1),
            ..IncrementReport::default()
        };
        report.no_model.insert("ZONE".into(), 1);
        assert!(report.accounts_for().is_ok());

        report.rolled_up = 2;
        let err = report.accounts_for().unwrap_err();
        assert!(err.contains("候选账不平"), "{err}");
    }

    /// L2 判「内容没变」的候选自成一本账，也必须进恒等式。
    ///
    /// 它不能并进 `no_model`：那本账的语义是「改动落在没有模型归属的元素上」，
    /// 而这些候选压根就没有改动。混在一起，「索引虚报了多少」就再也读不出来。
    #[test]
    fn an_unchanged_candidate_has_its_own_account() {
        let mut report = IncrementReport {
            candidates: 4,
            rolled_up: 1,
            unchanged: 3,
            ..IncrementReport::default()
        };
        assert!(report.accounts_for().is_ok());

        report.unchanged = 2;
        let err = report.accounts_for().unwrap_err();
        assert!(err.contains("unchanged 2"), "{err}");
    }

    /// 派生执行账：regenerated + retired + failed 必须等于计划容器数。
    ///
    /// 单独一条恒等式而不是并进单元执行账：两边的「计划数」来源不同
    /// （模型单元 vs 派生容器），混在一起，哪边不平就再也读不出来。
    #[test]
    fn derived_execution_accounting_must_close() {
        let report = IncrementReport {
            derived_to_regenerate: 3,
            derived_regenerated: 1,
            derived_retired: 1,
            derived_failed: incident_n(1),
            ..IncrementReport::default()
        };
        assert!(report.accounts_for().is_ok());

        let report = IncrementReport {
            derived_to_regenerate: 3,
            derived_regenerated: 1,
            derived_retired: 1,
            ..IncrementReport::default()
        };
        let err = report.accounts_for().unwrap_err();
        assert!(err.contains("派生执行账不平"), "{err}");
    }

    /// 执行账：regenerated + skipped + failed 必须等于计划单元数。
    #[test]
    fn execution_accounting_must_close() {
        let report = IncrementReport {
            units_to_regenerate: 4,
            regenerated: 2,
            regen_skipped: incident_n(1),
            regen_failed: incident_n(1),
            ..IncrementReport::default()
        };
        assert!(report.accounts_for().is_ok());

        let report = IncrementReport {
            units_to_regenerate: 4,
            regenerated: 2,
            ..IncrementReport::default()
        };
        let err = report.accounts_for().unwrap_err();
        assert!(err.contains("执行账不平"), "{err}");
    }

    /// refno 键往返无损（word1 高位置位的 refno 也一样）。
    #[test]
    fn refno_key_round_trips() {
        for (w0, w1) in [(17496u32, 1u32), (1u32, 0x7FFF_FFFFu32), (0x2001, 42)] {
            let refno = RefNo::new(w0, w1);
            assert_eq!(refno_of(key(refno)), refno);
        }
    }
}

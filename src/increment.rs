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
use crate::category::{classify, graphics_ignored_below, is_derived_unit, is_model_unit};
use crate::element_diff::{ElementDiff, diff_element};
use crate::elmodl::{GeneratedElement, GeometryId, Modeled, model_element_with_cache};
use crate::ledger::{ChangeLedger, ChangeTally, Disposition};
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
    /// `rolled_up` 里**只**靠变换级联折进去的候选数（自身与上方都没有单元 / 容器，
    /// 是子树里的后代被它带动）：典型是挪了位的 ZONE / EQUI。横切计数，不是第四本账。
    pub cascade_only: usize,
    /// 全部级联命中的单元 / 派生容器数之和（含已在计划集里的），与 `cascades.len()`
    /// 一起读：几处漂移、总共带动了多少件。
    pub cascade_hits: usize,
    /// 账本（L3）上出现过的不同 refno 数。候选除 `unchanged` 之外每个都得在账本上有一条
    /// （读不出的记 [`crate::ledger::ChangeKind::Unresolved`]），所以它必须等于
    /// `rolled_up + no_model + unresolved`——这是 L3 对 L4 的守恒，`accounts_for` 判。
    pub ledger_refnos: usize,
    /// 候选数超过 [`PlanLimits::max_candidates`]，规划器一个候选都没分类就退化成
    /// 「整库重建」的原因。非空时候选账不成立（什么都没分类），计划集全空，
    /// 调用方**必须**走全量生成而不是执行这份空计划。
    pub degraded_to_full: Option<String>,
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
    /// 账要闭合：每个候选恰好落一本账；账本上每个 refno 恰好对应一本候选账；
    /// 每个待重建单元恰好有一个执行结论。
    ///
    /// 退化成整库重建（`degraded_to_full`）的报告没有分类过任何候选：候选账不成立、
    /// 三本账与账本都必须是空的——空计划被当成「没变」执行才是这里要拦的事。
    pub fn accounts_for(&self) -> Result<(), String> {
        let classified = self.rolled_up
            + self.unchanged
            + self.no_model.values().sum::<usize>()
            + self.unresolved.len();
        if let Some(reason) = &self.degraded_to_full {
            if classified != 0
                || self.ledger_refnos != 0
                || self.units_to_regenerate != 0
                || self.units_to_remove != 0
                || self.derived_to_regenerate != 0
            {
                return Err(format!(
                    "退化成整库重建（{reason}）的计划不该分类过任何候选：\
                     classified {classified} ledger {} units {}/{}/{}",
                    self.ledger_refnos,
                    self.units_to_regenerate,
                    self.units_to_remove,
                    self.derived_to_regenerate,
                ));
            }
            return Ok(());
        }
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
        // L3 对 L4：除 unchanged 之外每个候选都在账本上、且只有一个去向。
        let ledgered =
            self.rolled_up + self.no_model.values().sum::<usize>() + self.unresolved.len();
        if self.ledger_refnos != ledgered {
            return Err(format!(
                "账本不平：账本上 {} 个 refno，候选账 rolled_up {} + no_model {} + unresolved {} \
                 = {ledgered}",
                self.ledger_refnos,
                self.rolled_up,
                self.no_model.values().sum::<usize>(),
                self.unresolved.len(),
            ));
        }
        if self.cascade_only > self.rolled_up {
            return Err(format!(
                "cascade_only {} 是 rolled_up {} 的子集，不能比它大",
                self.cascade_only, self.rolled_up
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
            unresolved,
            unchanged,
            created_subtree_roots,
            deleted_subtree_roots,
        } = self.changes;
        format!(
            "changes: created={created}（子树顶 {created_subtree_roots}） \
             deleted={deleted}（子树顶 {deleted_subtree_roots}） type_changed={type_changed} \
             reparented={reparented} members={members_changed} attrs={attributes_modified} \
             opaque={opaque_record_changes} unchanged={unchanged} unresolved={unresolved} \
             | ledger_refnos={} cascade_only={} cascade_hits={}",
            self.ledger_refnos, self.cascade_only, self.cascade_hits
        )
    }
}

/// [`plan_update_bounded`] 的护栏。`Default` = 不设限，即 [`plan_update`] 的行为。
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PlanLimits {
    /// 候选数**超过**它（严格大于）就不逐候选分类，直接退化成整库重建：
    /// 一窗动了大半个库时，逐候选走 owner 链比整库重算还贵（ams1112 721→722 的
    /// 24674 个候选 6.4 s，而全量生成本身也就这个量级），而且这种窗口多半是
    /// 合并 / 回退 / 重建，差分结论本来就不值得信。`None` = 不设限。
    ///
    /// 这里只认绝对数：`WindowDiff` 不知道 target 端一共有多少键。想按比例定
    /// （gen-model 用的是索引键数的 30%），调用方自己乘好再传进来。
    pub max_candidates: Option<usize>,
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
    /// 非空 = 这份计划**不是**增量：候选数超过 [`PlanLimits::max_candidates`]，规划器
    /// 一个候选都没分类，四个集合全空。调用方要走整库重建；把它交给 [`execute_plan`]
    /// 只会得到一份零动作的产出（报告带 `degraded_to_full`），不是「没变」。
    pub full_rebuild: Option<String>,
    pub report: IncrementReport,
}

impl UpdatePlan {
    /// 计划期能对的账：候选账（报告）+ 账本守恒（L3 每个 refno 恰好一个去向，且去向
    /// 计数与报告的三本候选账逐项相等）。执行账要 [`execute_plan`] 之后才成立，
    /// 那一半在 [`IncrementReport::accounts_for`] 里、由产出方的报告去对。
    pub fn accounts_for(&self) -> Result<(), String> {
        let classified = self.report.rolled_up
            + self.report.unchanged
            + self.report.no_model.values().sum::<usize>()
            + self.report.unresolved.len();
        if self.full_rebuild.is_none() && classified != self.report.candidates {
            return Err(format!(
                "候选账不平：rolled_up {} + unchanged {} + no_model {} + unresolved {} \
                 = {classified}，候选 {}",
                self.report.rolled_up,
                self.report.unchanged,
                self.report.no_model.values().sum::<usize>(),
                self.report.unresolved.len(),
                self.report.candidates,
            ));
        }
        let tally = self.ledger.accounts_for()?;
        let no_model = self.report.no_model.values().sum::<usize>();
        if tally.rolled_up != self.report.rolled_up
            || tally.no_model != no_model
            || tally.unresolved != self.report.unresolved.len()
        {
            return Err(format!(
                "账本去向与候选账不一致：账本 rolled_up {} / no_model {} / unresolved {}，\
                 报告 {} / {no_model} / {}",
                tally.rolled_up,
                tally.no_model,
                tally.unresolved,
                self.report.rolled_up,
                self.report.unresolved.len(),
            ));
        }
        if self.report.ledger_refnos != self.ledger.distinct_refnos() {
            return Err(format!(
                "报告记的账本 refno 数 {} 与账本实际 {} 不一致",
                self.report.ledger_refnos,
                self.ledger.distinct_refnos()
            ));
        }
        Ok(())
    }
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
///
/// # `graphicsBehaviour == 1` 的祖先让其下的一切都不算独立件
///
/// Core3D `DES_DrawList::isGraphicsIgnoredBetween`：从元素上溯，路上有一个中间祖先
/// 带 gb==1（[`graphics_ignored_below`]），这个元素的图形就被忽略——它是那个祖先
/// 属主几何的**数据**。所以上爬时每碰到一个 gb==1 节点，就把它**下方**找到的单元与
/// 容器整个作废（含候选自身），从它往上重找。这决定了这条链必须**走到根**：在中途
/// 收工，就不知道更上面还有没有一个 gb==1 把刚找到的单元打回数据。
fn nearest_unit(set: &Arc<DbSet>, refno: RefNo) -> Result<Rollup, DbElementError> {
    let mut current = set.element(refno);
    let start_noun = current.element_type()?;
    let start_owner = current.owner()?.map(|owner| owner.refno());
    let mut noun = start_noun.clone();
    let mut depth = 0usize;
    let mut unit: Option<RefNo> = None;
    let mut route_container: Option<(RefNo, String)> = None;
    loop {
        // 只看**祖先**：候选自己带 gb==1 说的是它的孩子，不是它自己。
        if depth > 0 && graphics_ignored_below(&noun) {
            unit = None;
            route_container = None;
        }
        if unit.is_none() && is_model_unit(&noun) {
            unit = Some(current.refno());
        }
        if route_container.is_none() && is_derived_unit(&noun) {
            route_container = Some((current.refno(), noun.clone()));
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
///
/// # `visited` 是整份计划共用的（[`CascadeVisits`]）
///
/// 它一度每次调用新建：一窗里 ZONE 与它名下的 EQUI 都挪了位，EQUI 的子树就被读两遍；
/// 候选按 refno 序来，谁先谁后随机，大窗口里重叠子树重复读 N 遍是常态。现在一棵子树
/// 只走一次——后来的级联碰到**别的级联**已走过的节点就地停下，那片子树的单元早已在
/// 计划集里，记进 [`CascadeFanout::overlapped`] 而不是白读。同一次遍历里二次到达才是
/// 成员表回环，两者靠每次遍历的世代号分开。
///
/// # gb==1 节点不往下走
///
/// 它的成员是它属主几何的数据（[`graphics_ignored_below`]，与 `nearest_unit` 同口径），
/// 不是独立单元；节点自己按 [`is_model_unit`] / [`is_derived_unit`] 照常判（今天恒否）。
#[derive(Debug, Clone, Copy)]
struct CascadeFanout {
    /// 这次遍历新到达并命中的单元 / 派生容器数（含已在计划集里的）。
    hits: usize,
    /// 撞上别的级联已走过的节点数（含根自己）。非零 = 这个候选的子树（部分）
    /// 早已被本窗更早的级联覆盖，它照样算折进了计划。
    overlapped: usize,
}

/// 一份计划内全部级联遍历的到达记录：节点 → 第几次级联到的它。
#[derive(Default)]
struct CascadeVisits {
    seen: BTreeMap<(u32, u32), usize>,
    generation: usize,
}

fn collect_unit_subtree(
    set: &Arc<DbSet>,
    root: RefNo,
    out: &mut BTreeSet<(u32, u32)>,
    derived: &mut BTreeSet<(u32, u32)>,
    notes: &mut Vec<Incident>,
    visits: &mut CascadeVisits,
) -> Result<CascadeFanout, DbElementError> {
    visits.generation += 1;
    let generation = visits.generation;
    let mut fanout = CascadeFanout {
        hits: 0,
        overlapped: 0,
    };
    // 根自己已被更早的级联走过：整棵子树都在计划里了，一页都不必再读。
    if visits.seen.insert(key(root), generation).is_some() {
        fanout.overlapped += 1;
        return Ok(fanout);
    }
    let mut stack = vec![set.element(root)];
    while let Some(el) = stack.pop() {
        for member in el.members()? {
            match visits.seen.insert(key(member.refno()), generation) {
                Some(earlier) if earlier == generation => {
                    notes.push(incident(
                        member.refno(),
                        member.element_type().unwrap_or_else(|_| "?".into()),
                        format!("成员表回环（{root} 子树级联遍历中二次到达），已跳过"),
                    ));
                    continue;
                }
                Some(_) => {
                    fanout.overlapped += 1;
                    continue;
                }
                None => {}
            }
            let noun = member.element_type()?;
            if is_model_unit(&noun) {
                out.insert(key(member.refno()));
                fanout.hits += 1;
            }
            if is_derived_unit(&noun) {
                derived.insert(key(member.refno()));
                fanout.hits += 1;
            }
            if graphics_ignored_below(&noun) {
                continue;
            }
            stack.push(member);
        }
    }
    Ok(fanout)
}

/// 电缆桥架分支容器的 noun：桥架件放置的全部跨元素输入都在它名下。
const CABLE_TRAY_BRANCH_NOUN: &str = "CWBRAN";

/// 候选（含自身）上溯碰到的最近 `CWBRAN`。不在任何桥架分支里 → `None`；
/// 起点在这一端不存在也是 `None`（Inserted 在 base、Deleted 在 target 是常态）。
///
/// # 为什么桥架要单独一条腿
///
/// 桥架件（CTSTRA/CTBEND/…）的模板里没有 POS/ORI，E3D 把放置注册成伪属性现算
/// （[`crate::cable_tray::placement`]）：输入是自己的 `POSREF/PKDI/ZDIS/ZOFF/BANG/GANGLE`、
/// `POSREF` 链上**别的桥架件**的 `PKDI/ZDIS`、以及 `CWBRAN` 名下 `RPATH → POINTR` 那一片
/// **兄弟**子树（`PPOS/PDIR/VERTDI/CURTYP/UBULG`）加 `CWBRAN` 自己的 `WTHICK/CLOSEF/VSHAPE`。
/// 这些边一条都不在 owner 链上：改一个 `POINTR`，`nearest_unit` 上卷到 RPATH/CWBRAN 都不是
/// 单元、`placement_input_changed` 只认 POS/ORI，于是落 `no_model`，而它旁边每一只
/// `CTRAY` 的世界矩阵都已经变了（`transform.rs` 记了两个月的「已知欠账」）。
///
/// 判据取最保守也最便宜的一档：**候选落在某个 `CWBRAN` 子树里（且自己不是 `CTRAY`），
/// 就把整个 `CWBRAN` 子树级联重建**。一条 `CWBRAN` 通常几十个成员，逐条依赖建图不值；
/// Core3D 那边伪属性根本没有变更检测，重画就是整条重算，我们只是把「整条」显式写出来。
/// `CTRAY` 自身的改动（SPRE 换目录件）只影响自己，上卷到自己就够，不触发整条。
fn cable_tray_branch_of(set: &Arc<DbSet>, refno: RefNo) -> Result<Option<RefNo>, DbElementError> {
    let mut current = set.element(refno);
    match current.exists() {
        Ok(true) => {}
        Ok(false) | Err(DbElementError::NotFound { .. }) => return Ok(None),
        Err(error) => return Err(error),
    }
    let mut depth = 0usize;
    loop {
        if current.element_type()? == CABLE_TRAY_BRANCH_NOUN {
            return Ok(Some(current.refno()));
        }
        match current.owner()? {
            Some(owner) if owner.refno() == current.refno() => return Ok(None),
            Some(owner) => {
                depth += 1;
                if depth > MAX_ANCESTOR_DEPTH {
                    return Err(DbElementError::OwnerCycle {
                        start: refno,
                        at: owner.refno(),
                    });
                }
                current = owner;
            }
            None => return Ok(None),
        }
    }
}

/// 把索引候选折算成模型单元动作。只读两端 `DbSet`，不建任何几何。不设候选上限
/// （[`PlanLimits::default`]）——真库门与探针要的就是「无论多大都逐候选给出结论」。
pub fn plan_update(base: &Arc<DbSet>, target: &Arc<DbSet>, window: &WindowDiff) -> UpdatePlan {
    plan_update_bounded(base, target, window, PlanLimits::default())
}

/// 带护栏的 [`plan_update`]：候选数超过 `limits.max_candidates` 就不逐候选分类，
/// 返回一份 [`UpdatePlan::full_rebuild`] 非空的空计划，报告记 `degraded_to_full`。
pub fn plan_update_bounded(
    base: &Arc<DbSet>,
    target: &Arc<DbSet>,
    window: &WindowDiff,
    limits: PlanLimits,
) -> UpdatePlan {
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
    if let Some(limit) = limits.max_candidates
        && report.candidates > limit
    {
        let reason = format!(
            "candidates {} > max_candidates {limit}; window {}→{} degraded to a full rebuild",
            report.candidates, window.base_sesno, window.target_sesno
        );
        report.degraded_to_full = Some(reason.clone());
        return UpdatePlan {
            regenerate: BTreeSet::new(),
            remove: BTreeSet::new(),
            regenerate_derived: BTreeSet::new(),
            derived_stale: BTreeSet::new(),
            ledger: ChangeLedger::new(),
            full_rebuild: Some(reason),
            report,
        };
    }
    let mut ledger = ChangeLedger::new();
    let mut regenerate: BTreeSet<(u32, u32)> = BTreeSet::new();
    let mut remove: BTreeSet<(u32, u32)> = BTreeSet::new();
    let mut regenerate_derived: BTreeSet<(u32, u32)> = BTreeSet::new();
    let mut visits = CascadeVisits::default();
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

    /// 一个候选的收尾：去向进账本，计数进报告。`no_model` 的 noun 用候选自己的。
    fn settle(
        report: &mut IncrementReport,
        ledger: &mut ChangeLedger,
        refno: RefNo,
        noun: &str,
        contributed: bool,
    ) {
        if contributed {
            report.rolled_up += 1;
            ledger.dispose(refno, Disposition::RolledUp);
        } else {
            note_no_model(report, noun);
            ledger.dispose(refno, Disposition::NoModel);
        }
    }

    /// 桥架分支级联（见 [`cable_tray_branch_of`]）：候选在 `lookup` 这一端落在某个 CWBRAN
    /// 子树里、且自己不是 CTRAY → 在 target 端把那个 CWBRAN 的子树整个收进计划。
    /// 返回这次级联折进了什么（`None` = 与桥架无关，或 CWBRAN 在 target 端已不存在
    /// ——那时它名下的托盘都是 Deleted 候选，各自出 Remove）。
    #[allow(clippy::too_many_arguments)]
    fn cascade_cable_tray_branch(
        lookup: &Arc<DbSet>,
        target: &Arc<DbSet>,
        refno: RefNo,
        noun: &str,
        regenerate: &mut BTreeSet<(u32, u32)>,
        regenerate_derived: &mut BTreeSet<(u32, u32)>,
        report: &mut IncrementReport,
        visits: &mut CascadeVisits,
    ) -> Result<Option<CascadeFanout>, DbElementError> {
        if noun.eq_ignore_ascii_case("CTRAY") {
            return Ok(None);
        }
        let Some(branch) = cable_tray_branch_of(lookup, refno)? else {
            return Ok(None);
        };
        match target.element(branch).exists() {
            Ok(true) => {}
            Ok(false) | Err(DbElementError::NotFound { .. }) => return Ok(None),
            Err(error) => return Err(error),
        }
        let fanout = collect_unit_subtree(
            target,
            branch,
            regenerate,
            regenerate_derived,
            &mut report.notes,
            visits,
        )?;
        report.cascades.push(incident(
            refno,
            noun,
            format!(
                "桥架放置输入变了（{noun} 落在 CWBRAN {branch} 子树里），级联重建整条分支 \
                 {} 个单元{}",
                fanout.hits,
                if fanout.overlapped > 0 {
                    format!("，另 {} 处已由本窗更早的级联覆盖", fanout.overlapped)
                } else {
                    String::new()
                }
            ),
        ));
        report.cascade_hits += fanout.hits;
        Ok(Some(fanout))
    }

    /// 读不出来的候选：报告与账本各记一条，去向 `Unresolved`。账本上已经有它的
    /// 条目（L2 差分成功、L4 上卷才失败）就不再补 `Unresolved` 条目，只登记去向。
    fn give_up(
        report: &mut IncrementReport,
        ledger: &mut ChangeLedger,
        refno: RefNo,
        noun: &str,
        detail: String,
        already_ledgered: bool,
    ) {
        report
            .unresolved
            .push(incident(refno, noun, detail.clone()));
        if !already_ledgered {
            ledger.unresolved(refno, noun, detail);
        }
        ledger.dispose(refno, Disposition::Unresolved);
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
                    // 新插进桥架分支的路径点 / 桥架件：整条分支的放置都变了。
                    match cascade_cable_tray_branch(
                        target,
                        target,
                        refno,
                        &rollup.noun,
                        &mut regenerate,
                        &mut regenerate_derived,
                        &mut report,
                        &mut visits,
                    ) {
                        Ok(fanout) => {
                            let cascaded = fanout
                                .is_some_and(|fanout| fanout.hits > 0 || fanout.overlapped > 0);
                            if cascaded && !contributed {
                                report.cascade_only += 1;
                            }
                            contributed |= cascaded;
                            settle(&mut report, &mut ledger, refno, &rollup.noun, contributed);
                        }
                        Err(error) => give_up(
                            &mut report,
                            &mut ledger,
                            refno,
                            &rollup.noun,
                            format!("新增候选的桥架分支级联失败：{error}"),
                            true,
                        ),
                    }
                }
                Err(error) => give_up(
                    &mut report,
                    &mut ledger,
                    refno,
                    "?",
                    format!("新增候选上卷失败：{error}"),
                    false,
                ),
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
                    // 从桥架分支里删掉一个路径点 / 桥架件：分支还在的话，剩下的托盘
                    // 全都挪了位。分支在 base 端找，级联在 target 端做。
                    match cascade_cable_tray_branch(
                        base,
                        target,
                        refno,
                        &rollup.noun,
                        &mut regenerate,
                        &mut regenerate_derived,
                        &mut report,
                        &mut visits,
                    ) {
                        Ok(fanout) => {
                            let cascaded = fanout
                                .is_some_and(|fanout| fanout.hits > 0 || fanout.overlapped > 0);
                            if cascaded && !contributed {
                                report.cascade_only += 1;
                            }
                            contributed |= cascaded;
                            settle(&mut report, &mut ledger, refno, &rollup.noun, contributed);
                        }
                        Err(error) => give_up(
                            &mut report,
                            &mut ledger,
                            refno,
                            &rollup.noun,
                            format!("删除候选的桥架分支级联失败：{error}"),
                            true,
                        ),
                    }
                }
                Err(error) => give_up(
                    &mut report,
                    &mut ledger,
                    refno,
                    "?",
                    format!("删除候选上卷失败：{error}"),
                    false,
                ),
            },
            IndexCandidate::Modified { refno, .. } => {
                // L2：索引只说记录被重写过，这一步才回答到底哪里变了。
                let diff = match diff_element(base, target, refno) {
                    Ok(diff) => diff,
                    Err(error) => {
                        give_up(
                            &mut report,
                            &mut ledger,
                            refno,
                            "?",
                            format!("逐元素差分失败：{error}"),
                            false,
                        );
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
                // 级联之外有没有折进什么：自身 / 上方的单元或容器，或改挂 / 类型变更
                // 回 base 端找到的旧单元。级联单独记，才分得出「只靠级联」的那些。
                let mut direct_contribution = false;
                match nearest_unit(target, refno) {
                    Ok(rollup) => {
                        contributed = mark_derived(&mut regenerate_derived, &rollup);
                        if let Some(unit) = rollup.unit {
                            regenerate.insert(key(unit));
                            contributed = true;
                        }
                        direct_contribution = contributed;
                    }
                    Err(error) => {
                        resolved = false;
                        give_up(
                            &mut report,
                            &mut ledger,
                            refno,
                            &diff.noun,
                            format!("修改候选上卷失败：{error}"),
                            true,
                        );
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
                            direct_contribution |= contributed;
                        }
                        Err(error) => {
                            resolved = false;
                            give_up(
                                &mut report,
                                &mut ledger,
                                refno,
                                &diff.noun,
                                format!("改挂/类型变更后回溯 base 端失败：{error}"),
                                true,
                            );
                        }
                    }
                }

                // 变换级联（见模块文档「与 Core3D 的一处有意分歧」）。
                let mut cascade_contributed = false;
                if resolved && diff.placement_input_changed() {
                    match collect_unit_subtree(
                        target,
                        refno,
                        &mut regenerate,
                        &mut regenerate_derived,
                        &mut report.notes,
                        &mut visits,
                    ) {
                        Ok(fanout) => {
                            report.cascades.push(incident(
                                refno,
                                diff.noun.clone(),
                                format!(
                                    "{}，级联重建子树内 {} 个单元（含派生容器）{}",
                                    cascade_reason(&diff),
                                    fanout.hits,
                                    if fanout.overlapped > 0 {
                                        format!(
                                            "，另 {} 处已由本窗更早的级联覆盖",
                                            fanout.overlapped
                                        )
                                    } else {
                                        String::new()
                                    }
                                ),
                            ));
                            report.cascade_hits += fanout.hits;
                            // 子树里有单元、或子树早被别的级联收进计划，这个候选都算
                            // 折进了计划；两样都没有才是「改动落在没有模型归属的元素上」。
                            cascade_contributed |= fanout.hits > 0 || fanout.overlapped > 0;
                        }
                        Err(error) => {
                            // 级联收集失败 = 有一片子树的世界矩阵可能已过期而我们
                            // 点不出名字——这不是 note 级别的观察，必须亮成
                            // unresolved 才对得起「静默缺件」那条宪法。
                            resolved = false;
                            give_up(
                                &mut report,
                                &mut ledger,
                                refno,
                                &diff.noun,
                                format!("世界矩阵输入变了但级联收集失败：{error}"),
                                true,
                            );
                        }
                    }
                }

                // 桥架分支级联：候选在 CWBRAN 子树里（RPATH/POINTR/桥架件/CWBRAN 自己）
                // 改了任何东西，整条分支的托盘放置都可能变——这些输入不在 owner 链上，
                // 上面那两步都够不着（见 `cable_tray_branch_of`）。
                if resolved {
                    match cascade_cable_tray_branch(
                        target,
                        target,
                        refno,
                        &diff.noun,
                        &mut regenerate,
                        &mut regenerate_derived,
                        &mut report,
                        &mut visits,
                    ) {
                        Ok(fanout) => {
                            cascade_contributed |= fanout
                                .is_some_and(|fanout| fanout.hits > 0 || fanout.overlapped > 0);
                        }
                        Err(error) => {
                            resolved = false;
                            give_up(
                                &mut report,
                                &mut ledger,
                                refno,
                                &diff.noun,
                                format!("桥架分支级联失败：{error}"),
                                true,
                            );
                        }
                    }
                }

                if resolved {
                    if cascade_contributed && !direct_contribution {
                        report.cascade_only += 1;
                    }
                    contributed = contributed || cascade_contributed;
                    settle(&mut report, &mut ledger, refno, &diff.noun, contributed);
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
    report.ledger_refnos = ledger.distinct_refnos();
    report.units_to_regenerate = regenerate.len();
    report.units_to_remove = remove.len();
    report.derived_to_regenerate = regenerate_derived.len();
    UpdatePlan {
        regenerate,
        remove,
        regenerate_derived,
        derived_stale,
        ledger,
        full_rebuild: None,
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
///
/// 退化成整库重建的计划（[`UpdatePlan::full_rebuild`]）这里**不执行**：它没有任何
/// 动作可执行，返回的产出是空的、报告带 `degraded_to_full` 与一条 note。调用方拿到它
/// 该走全量生成，把空产出当「这一窗没变」发布出去就是静默缺件。
pub fn execute_plan(target: &Arc<DbSet>, plan: UpdatePlan) -> IncrementOutcome {
    let UpdatePlan {
        regenerate,
        remove,
        regenerate_derived,
        derived_stale,
        ledger: _,
        full_rebuild,
        mut report,
    } = plan;
    if let Some(reason) = full_rebuild {
        report.notes.push(Incident {
            refno: String::new(),
            noun: String::new(),
            detail: format!("计划已退化成整库重建，未执行任何单元动作：{reason}"),
        });
        return IncrementOutcome {
            upserts: Vec::new(),
            removals: Vec::new(),
            report,
        };
    }
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
            ledger_refnos: 5,
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
            ledger_refnos: 1,
            ..IncrementReport::default()
        };
        assert!(report.accounts_for().is_ok());

        report.unchanged = 2;
        let err = report.accounts_for().unwrap_err();
        assert!(err.contains("unchanged 2"), "{err}");
    }

    /// L3 对 L4 的守恒：除 unchanged 之外每个候选都在账本上——账本 refno 数必须等于
    /// rolled_up + no_model + unresolved。少了就是有候选（多半是读不出的那些）没进账。
    #[test]
    fn the_ledger_must_name_every_classified_candidate() {
        let mut report = IncrementReport {
            candidates: 6,
            rolled_up: 2,
            unchanged: 2,
            unresolved: incident_n(1),
            ledger_refnos: 4,
            ..IncrementReport::default()
        };
        report.no_model.insert("ZONE".into(), 1);
        assert!(report.accounts_for().is_ok());

        // 读不出的那个候选没进账本：候选账照样平，账本账要响。
        report.ledger_refnos = 3;
        let err = report.accounts_for().unwrap_err();
        assert!(err.contains("账本不平"), "{err}");

        // cascade_only 是 rolled_up 的子集。
        report.ledger_refnos = 4;
        report.cascade_only = 3;
        let err = report.accounts_for().unwrap_err();
        assert!(err.contains("cascade_only"), "{err}");
    }

    /// 退化成整库重建的报告：什么都没分类，候选账不适用，但计划集与账本必须是空的
    /// ——一份带动作的「退化」计划是自相矛盾的，得响。
    #[test]
    fn a_degraded_plan_has_no_classification_and_no_actions() {
        let report = IncrementReport {
            candidates: 24674,
            degraded_to_full: Some("candidates 24674 > max_candidates 1000".into()),
            ..IncrementReport::default()
        };
        assert!(report.accounts_for().is_ok(), "{:?}", report.accounts_for());

        let report = IncrementReport {
            candidates: 24674,
            degraded_to_full: Some("candidates 24674 > max_candidates 1000".into()),
            units_to_regenerate: 1,
            ..IncrementReport::default()
        };
        let err = report.accounts_for().unwrap_err();
        assert!(err.contains("退化成整库重建"), "{err}");

        // 没有退化标记，候选却一个没分类：这是普通的候选账不平。
        let report = IncrementReport {
            candidates: 24674,
            ..IncrementReport::default()
        };
        assert!(report.accounts_for().unwrap_err().contains("候选账不平"));
    }

    /// `PlanLimits::default()` 不设限——`plan_update` 的行为一个字不变。
    #[test]
    fn default_limits_are_unbounded() {
        assert_eq!(PlanLimits::default().max_candidates, None);
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

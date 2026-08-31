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
//!    - 改动落在没有单元归属的元素上（容器改名、NonGraphic、未建的目录件）→ 记
//!      `no_model` 账，一个都不许蒸发。
//! 3. **执行** [`execute_plan`]：逐单元重建（与全量管线同一条 [`model_element`]），
//!    产出 Upsert / Remove 动作。重建结果为 Skipped / 失败时**发 Remove**——
//!    Core 的次序是先擦后画，画不出来那一行就该保持擦掉的状态，绝不留陈旧网格。
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
//! ## 已知口子：隐式管身不在增量的射程内
//!
//! 全量管线会为路由容器产出隐式管身，它们在模型集里按
//! [`crate::elmodl::GeometryId::ImpliedTube`] 索引，而这里的 Remove 是按 refno 发的——
//! 容器 refno 上并没有「一件」几何可删，管身也没有自己的库记录可供索引差分点名。所以
//! [`is_model_unit`] 有意把路由容器排除在外，代价是**一条 BRAN 底下的构件改了，
//! 它两侧的管身不会跟着重建**。
//!
//! 这个口子今天还不会咬人（真库门那几个库里 `implied_tubes.built` 都是 0），
//! 但它与 SBFI 那条是同一个形状，只是还没被点着。堵它要先把模型集的主键从 refno
//! 换成 [`crate::elmodl::GeometryId`]，那会一起动到 `update_ams` 与 RVM 对拍，
//! 不该塞进这次修复。
//! [`plan_update`] 为此单开一本 [`IncrementReport::stale_route_containers`]：口子敞着
//! 可以，敞着而没人数得出来不行。
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

use crate::category::{Category, classify, is_model_unit};
use crate::element_diff::{ElementDiff, diff_element};
use crate::elmodl::{GeneratedElement, Modeled, model_element};
use crate::ledger::{ChangeLedger, ChangeTally};
use crate::pipeline::Incident;

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
    /// 落在某条路由射程里、因而**可能让隐式管身过期**的候选：容器 noun → 数量。
    ///
    /// 这不是第四本账——这些候选已经按各自的去向记进了 `rolled_up` / `no_model`，
    /// 这里是横切的一笔。管身不按 refno 索引，本期增量动不了它们（见模块文档
    /// 「已知口子」）；非零就意味着这一窗的管身可能已经陈旧，是那个口子的上界。
    /// 单开一本而不是写进 `notes`：notes 会被淹，这条要能一眼数出来。
    pub stale_route_containers: BTreeMap<String, usize>,
    /// 连分类都做不了的候选（端点读不出 noun / owner 链断）。出现即数据或读层问题。
    pub unresolved: Vec<Incident>,
    /// 变换级联：哪个元素的世界矩阵输入变了、扇出了几个单元。
    ///
    /// 触发判据是 [`ElementDiff::placement_input_changed`]，含 `POS`/`ORI` 与**改挂**。
    pub cascades: Vec<Incident>,
    /// 计划出的单元数。
    pub units_to_regenerate: usize,
    pub units_to_remove: usize,
    /// 执行账：三者之和必须等于 `units_to_regenerate`。
    pub regenerated: usize,
    pub regen_skipped: Vec<Incident>,
    pub regen_failed: Vec<Incident>,
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
        Ok(())
    }

    pub fn totals_line(&self) -> String {
        format!(
            "window {}→{}: candidates={} (+{} ~{} -{}) rolled_up={} unchanged={} no_model={} \
             unresolved={} cascades={} stale_routes={} units regen={} remove={} | \
             regenerated={} skipped={} failed={}",
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
            self.stale_route_containers.values().sum::<usize>(),
            self.units_to_regenerate,
            self.units_to_remove,
            self.regenerated,
            self.regen_skipped.len(),
            self.regen_failed.len(),
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
    /// L3 语义账。计数已经折进 [`IncrementReport::changes`]，这里保留逐条明细，
    /// 给探针与真库门查「这一窗到底改了什么」用。
    pub ledger: ChangeLedger,
    pub report: IncrementReport,
}

/// 一次增量更新的完整产出：模型集上的动作 + 总账。
///
/// `upserts` 按 refno 覆盖旧网格；`removals` 是「确保不存在」（幂等，目标本来
/// 不在模型集里也不算错）。两个列表都已排序去重。
pub struct IncrementOutcome {
    pub upserts: Vec<GeneratedElement>,
    pub removals: Vec<RefNo>,
    pub report: IncrementReport,
}

fn key(refno: RefNo) -> (u32, u32) {
    (refno.word0, refno.word1)
}

fn refno_of(key: (u32, u32)) -> RefNo {
    RefNo::new(key.0, key.1)
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
    /// 路上跨过的最近一个路由容器的 noun。管身按容器 refno 索引不出来（见模块文档
    /// 「已知口子」），所以它不会变成单元；但落在管身射程里的改动必须数得出来。
    route_container: Option<String>,
}

/// 沿 owner 链找最近的模型单元（含自身）。判据是 [`is_model_unit`]，与全量管线
/// 同一份。`SignificantOwner` 语义：从自身起判、跨过一切不成单元的属主。
fn nearest_unit(set: &Arc<DbSet>, refno: RefNo) -> Result<Rollup, DbElementError> {
    let mut current = set.element(refno);
    let start_noun = current.element_type()?;
    let start_owner = current.owner()?.map(|owner| owner.refno());
    let mut noun = start_noun.clone();
    let mut depth = 0usize;
    let mut route_container = None;
    loop {
        let found = |unit, route_container| Rollup {
            noun: start_noun.clone(),
            owner: start_owner,
            unit,
            route_container,
        };
        if is_model_unit(&noun) {
            return Ok(found(Some(current.refno()), route_container));
        }
        if route_container.is_none() && classify(&noun) == Category::RouteContainer {
            route_container = Some(noun.clone());
        }
        match current.owner()? {
            Some(owner) => {
                depth += 1;
                if depth > MAX_ANCESTOR_DEPTH || owner.refno() == current.refno() {
                    return Err(DbElementError::OwnerCycle {
                        start: refno,
                        at: owner.refno(),
                    });
                }
                current = owner;
                noun = current.element_type()?;
            }
            None => return Ok(found(None, route_container)),
        }
    }
}

/// 收集 `root` 子树内（不含 root 自身）全部模型单元。返回新加进 `out` 的个数。
///
/// 消费关系不影响收集：被消费的子树（剖面环、负体、CMPF）里本就没有单元，
/// 走一遍只是白读几页。成员表出现回环用 visited 兜住并记 note——那是数据坏，
/// 不该让级联把进程挂死。
fn collect_unit_subtree(
    set: &Arc<DbSet>,
    root: RefNo,
    out: &mut BTreeSet<(u32, u32)>,
    notes: &mut Vec<Incident>,
) -> Result<usize, DbElementError> {
    let mut added = 0usize;
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
            if is_model_unit(&noun) && out.insert(key(member.refno())) {
                added += 1;
            }
            stack.push(member);
        }
    }
    Ok(added)
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
    // 删除候选上卷到的属主单元。属主自己也被删时要归并进那条 Remove，
    // 所以等 remove 集收齐了再定去向。
    let mut deferred: Vec<RefNo> = Vec::new();

    fn note_no_model(report: &mut IncrementReport, noun: &str) {
        *report.no_model.entry(noun.to_string()).or_insert(0) += 1;
    }

    fn note_route_container(report: &mut IncrementReport, rollup: &Rollup) {
        if let Some(noun) = &rollup.route_container {
            *report
                .stale_route_containers
                .entry(noun.clone())
                .or_insert(0) += 1;
        }
    }

    for change in &window.diff.changes {
        match *change {
            IndexCandidate::Inserted { refno, .. } => match nearest_unit(target, refno) {
                Ok(rollup) => {
                    ledger.created(refno, rollup.noun.clone(), rollup.owner);
                    note_route_container(&mut report, &rollup);
                    match rollup.unit {
                        Some(unit) => {
                            regenerate.insert(key(unit));
                            report.rolled_up += 1;
                        }
                        None => note_no_model(&mut report, &rollup.noun),
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
                    note_route_container(&mut report, &rollup);
                    match rollup.unit {
                        // 单元自己被删 → 直接 Remove（Core R15 同款，不再上卷）。
                        Some(unit) if unit == refno => {
                            remove.insert(key(unit));
                            report.rolled_up += 1;
                        }
                        Some(unit) => {
                            deferred.push(unit);
                            report.rolled_up += 1;
                        }
                        None => note_no_model(&mut report, &rollup.noun),
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
                        note_route_container(&mut report, &rollup);
                        if let Some(unit) = rollup.unit {
                            regenerate.insert(key(unit));
                            contributed = true;
                        } else if !diff.placement_input_changed() {
                            note_no_model(&mut report, &rollup.noun);
                            continue;
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
                if resolved && diff.owner_changed.is_some() {
                    match nearest_unit(base, refno) {
                        Ok(rollup) => {
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
                                format!("改挂后回溯旧属主失败：{error}"),
                            ));
                        }
                    }
                }

                // 变换级联（见模块文档「与 Core3D 的一处有意分歧」）。
                if resolved && diff.placement_input_changed() {
                    match collect_unit_subtree(target, refno, &mut regenerate, &mut report.notes) {
                        Ok(fanout) => {
                            report.cascades.push(incident(
                                refno,
                                diff.noun.clone(),
                                format!(
                                    "{}，级联重建子树内 {fanout} 个单元",
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

    ledger.finish();
    report.changes = ledger.tally();
    report.units_to_regenerate = regenerate.len();
    report.units_to_remove = remove.len();
    UpdatePlan {
        regenerate,
        remove,
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

/// 执行计划：逐单元重建。与全量管线走同一条 [`model_element`]，产出动作集。
pub fn execute_plan(target: &Arc<DbSet>, plan: UpdatePlan) -> IncrementOutcome {
    let UpdatePlan {
        regenerate,
        remove,
        ledger: _,
        mut report,
    } = plan;
    let mut upserts: Vec<GeneratedElement> = Vec::new();
    let mut removals: BTreeSet<(u32, u32)> = remove;

    for unit in regenerate {
        let refno = refno_of(unit);
        let el = target.element(refno);
        let noun = match el.element_type() {
            Ok(noun) => noun,
            Err(error) => {
                removals.insert(unit);
                report
                    .regen_failed
                    .push(incident(refno, "?", format!("读不出 noun：{error}")));
                continue;
            }
        };
        let category = classify(&noun);
        if !is_model_unit(&noun) {
            // 上卷只会选模型单元；走到这里是分类在两次调用间变了或计划被手工改过。
            // 响亮记账（不 panic：一个坏单元不该打崩整个窗口）。
            removals.insert(unit);
            report.regen_failed.push(incident(
                refno,
                noun,
                format!("重建单元分类为 {category:?}，不成模型单元"),
            ));
            continue;
        }
        match model_element(&el, category) {
            Ok(Modeled::Solid(generated)) => {
                report.regenerated += 1;
                upserts.push(*generated);
            }
            Ok(Modeled::Skipped { reason }) => {
                // E3D 同样不出几何 → 模型集里不该有它的行（先擦后画，画不出保持擦掉）。
                removals.insert(unit);
                report.regen_skipped.push(incident(refno, noun, reason));
            }
            Err(error) => {
                removals.insert(unit);
                report
                    .regen_failed
                    .push(incident(refno, noun, error.to_string()));
            }
        }
    }

    upserts.sort_by(|left, right| left.geometry_id.cmp(&right.geometry_id));
    IncrementOutcome {
        upserts,
        removals: removals.into_iter().map(refno_of).collect(),
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

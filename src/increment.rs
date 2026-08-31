//! 模型增量更新：两个会话之间只重建受影响的元素。
//!
//! 三段式，段间只传数据：
//!
//! 1. **窗口收集** [`collect_window`]：e3d-io 的会话索引差分
//!    （对标 `core.dll::DB_IndexTableCompare`——删除 = 键集差、修改 = 记录位置变、
//!    flag 不参与判定）。先全量拿完 [`CompletedIndexDiff`] 再消费，差分中途失败
//!    整个窗口失败，没有半截结果可以拿去更新模型（ADR-017 整窗口口径）。
//! 2. **候选分类** [`plan_update`]：把索引级候选（refno + 增/删/改）折算成
//!    **模型单元**动作。模型单元 = 一个产正实体的元素（[`Category::is_positive_solid`]），
//!    与 Core3D `PartialUpdateDesiMgr` 的 significant/primitive 上卷同构：
//!    - 候选自己是正体 → 单元就是它；
//!    - 候选是被消费的数据（剖面环 / 负体，含 CMPF 里的深层负体）→ 沿 owner 链
//!      上卷到最近的正体属主（`SignificantOwner` 语义：从自身起判、无深度上限、
//!      List 容器天然被跨过）；
//!    - 删除的正体不上卷，直接出 Remove（Core R15 同款）；
//!    - 改动落在没有正体归属的元素上（容器改名、NonGraphic、目录件）→ 记
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
//! [`plan_update`] 对每个 Modified 候选比对两端 [`local_placement`]，漂移即级联
//! （见 [`IncrementReport::cascades`]）。判据与 [`world_matrix`] 读同一条路，
//! 不会出现「矩阵变了但级联说没变」。
//!
//! ## 账本
//!
//! gen-model 宪法同款：静默缺件是最高级别缺陷。每个候选必须落进
//! `rolled_up` / `no_model` / `unresolved` 三本账之一（[`IncrementReport::accounts_for`]
//! 硬判据），执行侧 `regenerated + regen_skipped + regen_failed == units_to_regenerate`。
//!
//! [`world_matrix`]: crate::transform::world_matrix
//! [`Category::is_positive_solid`]: crate::category::Category::is_positive_solid

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;
use std::sync::Arc;

use e3d_io::db_element::{DbElementError, DbSet};
use e3d_io::error::E3dReaderError;
use e3d_io::index::{CompletedIndexDiff, IndexCandidate, IndexDiff, IndexTreeError};
use e3d_io::page::PageId;
use e3d_io::refno::RefNo;
use e3d_io::session::DbView;

use crate::category::classify;
use crate::elmodl::{GeneratedElement, Modeled, model_element};
use crate::pipeline::Incident;
use crate::transform::local_placement;

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
    /// 折算进了某个单元动作（重建 / 移除，含被去重掉的）的候选数。
    pub rolled_up: usize,
    /// 没落到任何模型单元的候选：noun → 数量（容器改名、NonGraphic、目录件……）。
    pub no_model: BTreeMap<String, usize>,
    /// 连分类都做不了的候选（端点读不出 noun / owner 链断）。出现即数据或读层问题。
    pub unresolved: Vec<Incident>,
    /// 变换级联：哪个元素的 POS/ORI 漂了、扇出了几个单元。
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
        let classified =
            self.rolled_up + self.no_model.values().sum::<usize>() + self.unresolved.len();
        if classified != self.candidates {
            return Err(format!(
                "候选账不平：rolled_up {} + no_model {} + unresolved {} = {classified}，候选 {}",
                self.rolled_up,
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
            "window {}→{}: candidates={} (+{} ~{} -{}) rolled_up={} no_model={} unresolved={} \
             cascades={} units regen={} remove={} | regenerated={} skipped={} failed={}",
            self.base_sesno,
            self.target_sesno,
            self.candidates,
            self.inserted,
            self.modified,
            self.deleted,
            self.rolled_up,
            self.no_model.values().sum::<usize>(),
            self.unresolved.len(),
            self.cascades.len(),
            self.units_to_regenerate,
            self.units_to_remove,
            self.regenerated,
            self.regen_skipped.len(),
            self.regen_failed.len(),
        )
    }
}

/// 分类完成、还没执行的更新计划。键用 `(word0, word1)`（`RefNo` 不带 `Ord`，
/// 这里只要一个稳定的去重序，不承载索引侧的无符号比较语义）。
pub struct UpdatePlan {
    pub regenerate: BTreeSet<(u32, u32)>,
    pub remove: BTreeSet<(u32, u32)>,
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

/// 沿 owner 链找最近的正体单元（含自身）。找不到 = 这条改动上方没有任何
/// 会被建模的东西。`SignificantOwner` 语义：从自身起判、跨过一切非正体属主。
fn nearest_unit(set: &Arc<DbSet>, refno: RefNo) -> Result<Option<RefNo>, DbElementError> {
    let mut current = set.element(refno);
    let mut depth = 0usize;
    loop {
        let noun = current.element_type()?;
        if classify(&noun).is_positive_solid() {
            return Ok(Some(current.refno()));
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
            }
            None => return Ok(None),
        }
    }
}

/// 两端读同一元素的 `(POS, ORI)`，不相等即变换漂移。
fn placement_drifted(
    base: &Arc<DbSet>,
    target: &Arc<DbSet>,
    refno: RefNo,
) -> Result<bool, DbElementError> {
    let before = local_placement(&base.element(refno))?;
    let after = local_placement(&target.element(refno))?;
    Ok(before != after)
}

/// 收集 `root` 子树内（不含 root 自身）全部正体单元。返回新加进 `out` 的个数。
///
/// 消费关系不影响收集：被消费的子树（剖面环、负体、CMPF）里本就没有正体，
/// 走一遍只是白读几页。成员表出现回环用 visited 兜住并记 note——那是数据坏，
/// 不该让级联把进程挂死。
fn collect_positive_subtree(
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
            if classify(&noun).is_positive_solid() && out.insert(key(member.refno())) {
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
    let mut regenerate: BTreeSet<(u32, u32)> = BTreeSet::new();
    let mut remove: BTreeSet<(u32, u32)> = BTreeSet::new();
    // 删除候选上卷到的属主单元。属主自己也被删时要归并进那条 Remove，
    // 所以等 remove 集收齐了再定去向。
    let mut deferred: Vec<RefNo> = Vec::new();

    fn note_no_model(report: &mut IncrementReport, set: &Arc<DbSet>, refno: RefNo) {
        match set.element(refno).element_type() {
            Ok(noun) => *report.no_model.entry(noun).or_insert(0) += 1,
            Err(error) => report.unresolved.push(incident(
                refno,
                "?",
                format!("读不出 noun，无法归类：{error}"),
            )),
        }
    }

    for change in &window.diff.changes {
        match *change {
            IndexCandidate::Inserted { refno, .. } => match nearest_unit(target, refno) {
                Ok(Some(unit)) => {
                    regenerate.insert(key(unit));
                    report.rolled_up += 1;
                }
                Ok(None) => note_no_model(&mut report, target, refno),
                Err(error) => {
                    report.unresolved.push(incident(
                        refno,
                        "?",
                        format!("新增候选上卷失败：{error}"),
                    ));
                }
            },
            IndexCandidate::Deleted { refno, .. } => match nearest_unit(base, refno) {
                Ok(Some(unit)) if unit == refno => {
                    remove.insert(key(unit));
                    report.rolled_up += 1;
                }
                Ok(Some(unit)) => {
                    deferred.push(unit);
                    report.rolled_up += 1;
                }
                Ok(None) => note_no_model(&mut report, base, refno),
                Err(error) => {
                    report.unresolved.push(incident(
                        refno,
                        "?",
                        format!("删除候选上卷失败：{error}"),
                    ));
                }
            },
            IndexCandidate::Modified { refno, .. } => {
                let mut contributed = false;
                let mut readable = true;
                match nearest_unit(target, refno) {
                    Ok(Some(unit)) => {
                        regenerate.insert(key(unit));
                        contributed = true;
                    }
                    Ok(None) => {}
                    Err(error) => {
                        readable = false;
                        report.unresolved.push(incident(
                            refno,
                            "?",
                            format!("修改候选上卷失败：{error}"),
                        ));
                    }
                }
                // 变换级联（见模块文档「与 Core3D 的一处有意分歧」）。
                if readable {
                    match placement_drifted(base, target, refno) {
                        Ok(true) => {
                            match collect_positive_subtree(
                                target,
                                refno,
                                &mut regenerate,
                                &mut report.notes,
                            ) {
                                Ok(fanout) => {
                                    let noun = target
                                        .element(refno)
                                        .element_type()
                                        .unwrap_or_else(|_| "?".into());
                                    report.cascades.push(incident(
                                        refno,
                                        noun,
                                        format!("POS/ORI 漂移，级联重建子树内 {fanout} 个单元"),
                                    ));
                                    contributed = contributed || fanout > 0;
                                }
                                Err(error) => {
                                    // 级联收集失败 = 有一片子树的世界矩阵可能已过期
                                    // 而我们点不出名字——这不是 note 级别的观察，
                                    // 必须亮成 unresolved 才对得起「静默缺件」那条宪法。
                                    readable = false;
                                    report.unresolved.push(incident(
                                        refno,
                                        "?",
                                        format!("POS/ORI 漂移但级联收集失败：{error}"),
                                    ));
                                }
                            }
                        }
                        Ok(false) => {}
                        Err(error) => {
                            if contributed {
                                report.notes.push(incident(
                                    refno,
                                    "?",
                                    format!("单元已入账，但级联检查读挂：{error}"),
                                ));
                            } else {
                                readable = false;
                                report.unresolved.push(incident(
                                    refno,
                                    "?",
                                    format!("级联检查失败：{error}"),
                                ));
                            }
                        }
                    }
                }
                if readable {
                    if contributed {
                        report.rolled_up += 1;
                    } else {
                        note_no_model(&mut report, target, refno);
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

    report.units_to_regenerate = regenerate.len();
    report.units_to_remove = remove.len();
    UpdatePlan {
        regenerate,
        remove,
        report,
    }
}

/// 执行计划：逐单元重建。与全量管线走同一条 [`model_element`]，产出动作集。
pub fn execute_plan(target: &Arc<DbSet>, plan: UpdatePlan) -> IncrementOutcome {
    let UpdatePlan {
        regenerate,
        remove,
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
        if !category.is_positive_solid() {
            // 上卷只会选正体；走到这里是分类在两次调用间变了或计划被手工改过。
            // 响亮记账（不 panic：一个坏单元不该打崩整个窗口）。
            removals.insert(unit);
            report.regen_failed.push(incident(
                refno,
                noun,
                format!("重建单元分类为 {category:?}，不产正实体"),
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

    upserts.sort_by_key(|generated| key(generated.refno));
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

//! L3 语义账：把 L2 的逐元素差分归进「一次改动可能是什么」的有限几种。
//!
//! 对标 core.dll 的 `DB_UserChanges`（语义版 `DB_DB::elementsChangedBetween`，
//! `0x58ffc50`）与 `DB_RawChanges` 的祖先抢占（`isElementTopLevelCreate`
//! `0x5983b80` / `isElementTopLevelDelete` `0x5983bd0`）。
//!
//! ## 这一层负责的事只有一件：让每一条改动都有名字
//!
//! [`crate::element_diff`] 回答「哪里变了」，这一层回答「这算什么变化」。
//! 分开是因为两者的权威性不同：前者是字节比对的事实，后者是分类判断，会随着
//! 对 core 语义理解的加深而调整。混在一起写，调整分类就得动比对代码。
//!
//! ## 祖先抢占：core 敢丢，我们只敢标
//!
//! core 对整棵新建/删除的子树只上报最顶那一个——`isElementTopLevelCreate` 的判据是
//! 「我的属主**不在**新建集合里」。它敢这么丢，是因为消费侧
//! （`DES_DrawList::getChildTOPFElements`，`0x105166d0`）能反问**已经画出来的那份**
//! 图形段树，问出这棵子树底下曾经有过哪些顶层单元。
//!
//! 我们没有那份输入。删除侧尤其致命：`ams1112` 721→722 一次没掉 3348 件正体，
//! 它们的属主多半也在同一批里；照 core 丢掉非顶元素，L4 就再也点不出那 3348 个
//! refno，而全量生成说它们全没了——正是本仓最高级别的静默缺件。
//!
//! 所以这里把抢占算出来、记成 [`LedgerEntry::preempted_by`] 一个**标记**，
//! 条目一条不少。等「已产出模型集」那份输入到位（设计文档 §3.5），L4 才谈得上
//! 只消费子树顶。在那之前，这个字段的价值是让报告说得出
//! 「3348 条删除，其中 12 条是子树根」，而不是一个读不出结构的 3348。

use std::collections::BTreeSet;

use e3d_io::refno::RefNo;

use crate::element_diff::ElementDiff;
use crate::pipeline::Incident;

/// 一条改动算哪一类。与 `checkEle` 的回调族一一对应。
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ChangeKind {
    /// base 端没有、target 端有（`createFirst`/`createAfter`/`createNull`）。
    Created,
    /// base 端有、target 端没有（`deleteAll`）。
    Deleted,
    /// noun 变了（`dealWithChangeType`：先删，再把整棵子树重建）。
    TypeChanged,
    /// 属主变了（`includeFirst`/`includeAfter`，回调里读的是**旧属主**）。
    Reparented,
    /// 成员表变了（core 把它表达成属主身上的 `ATT_MEMB` 属性变化）。
    MembersChanged,
    /// 某些属性的值变了（`attChange`）。
    AttributesModified,
    /// 记录字节变了，但说不出是哪一项（见 `ElementDiff::opaque`）。
    OpaqueRecordChange,
    /// 索引点了名，L2/L4 却连它是什么都读不出来（端点读挂 / owner 链断）。
    ///
    /// 不对应任何 `checkEle` 回调——core 在这里会直接抛错。单列一种是为了让账闭合：
    /// 每个候选都必须在账上有一条，读不出的也不例外，否则「候选 24674、账上 24670」
    /// 那 4 条就没人说得出去了哪里（[`ChangeLedger::accounts_for`]）。
    Unresolved,
}

impl ChangeKind {
    pub fn label(self) -> &'static str {
        match self {
            ChangeKind::Created => "新建",
            ChangeKind::Deleted => "删除",
            ChangeKind::TypeChanged => "类型变更",
            ChangeKind::Reparented => "改挂",
            ChangeKind::MembersChanged => "成员表变更",
            ChangeKind::AttributesModified => "属性修改",
            ChangeKind::OpaqueRecordChange => "记录变更（未指名）",
            ChangeKind::Unresolved => "无法分类",
        }
    }
}

/// 一个候选在 L4 的去向。与 `IncrementReport` 的三本候选账一一对应：
/// 每个进了账本的 refno 必须恰好有一个去向，[`ChangeLedger::accounts_for`] 据此对账。
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Disposition {
    /// 折进了某个单元 / 派生容器动作（含被去重掉的，含只靠级联折进去的）。
    RolledUp,
    /// 改动落在没有模型归属的元素上。
    NoModel,
    /// 连分类都做不了。
    Unresolved,
}

/// [`ChangeLedger::accounts_for`] 对出来的去向计数。
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct DispositionTally {
    pub rolled_up: usize,
    pub no_model: usize,
    pub unresolved: usize,
}

/// 账上的一条。
///
/// 一个元素可以出现在多条上：core 的 `checkEle` 对同一个元素既可能调
/// `includeAfter`（改挂）又调 `attChange`（属性变），我们照样分开记——
/// 合成一条就说不清「这一窗有多少次改挂」。
#[derive(Debug, Clone, PartialEq)]
pub struct LedgerEntry {
    pub refno: RefNo,
    pub noun: String,
    pub kind: ChangeKind,
    pub detail: String,
    /// 我的属主也在同一个桶里（仅 [`ChangeKind::Created`] / [`ChangeKind::Deleted`]
    /// 参与计算）。`Some(属主)` = 我不是这棵新建/删除子树的顶。
    pub preempted_by: Option<RefNo>,
}

impl LedgerEntry {
    pub fn as_incident(&self) -> Incident {
        Incident {
            refno: self.refno.to_string(),
            noun: self.noun.clone(),
            detail: self.detail.clone(),
        }
    }
}

/// 各桶的计数，直接进报告。
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, serde::Serialize)]
pub struct ChangeTally {
    pub created: usize,
    pub deleted: usize,
    pub type_changed: usize,
    pub reparented: usize,
    pub members_changed: usize,
    pub attributes_modified: usize,
    pub opaque_record_changes: usize,
    /// 读不出来、分类不了的候选条目数（[`ChangeKind::Unresolved`]）。
    pub unresolved: usize,
    /// 索引说记录被重写、逐字节比下来内容一个字没动的候选数。
    ///
    /// 这不是一种改动，是 L1 与 L2 的**落差**：非零说明索引候选里有多少是虚的。
    /// 老实现没有这一层，这些候选一律当「变了」照常重建。
    pub unchanged: usize,
    /// 新建条目里属主不在新建集合中的那些——整棵新建子树的顶。
    pub created_subtree_roots: usize,
    /// 删除条目里属主不在删除集合中的那些——整棵删除子树的顶。
    pub deleted_subtree_roots: usize,
}

/// 一个窗口的全部语义改动。
#[derive(Debug, Default)]
pub struct ChangeLedger {
    entries: Vec<LedgerEntry>,
    unchanged: usize,
    /// 建账时记下的属主，`finish` 用它算祖先抢占。
    owners: Vec<(usize, RefNo)>,
    /// 每个候选在 L4 的去向，按登记顺序；重复登记由 [`Self::accounts_for`] 判出来。
    dispositions: Vec<((u32, u32), Disposition)>,
}

impl ChangeLedger {
    pub fn new() -> Self {
        Self::default()
    }

    /// 读不出来的候选也要有一条。`noun` 不知道就给 `"?"`。
    pub fn unresolved(&mut self, refno: RefNo, noun: impl Into<String>, detail: impl Into<String>) {
        self.entries.push(LedgerEntry {
            refno,
            noun: noun.into(),
            kind: ChangeKind::Unresolved,
            detail: detail.into(),
            preempted_by: None,
        });
    }

    /// 登记一个候选的去向。每个进了账本的 refno 恰好登记一次；`unchanged` 的候选
    /// 不进账本也不登记（它们不是改动）。
    pub fn dispose(&mut self, refno: RefNo, disposition: Disposition) {
        self.dispositions
            .push(((refno.word0, refno.word1), disposition));
    }

    /// 账本上出现过的**不同** refno 数。一个元素可以有多条（改挂 + 属性变），
    /// 但候选是按 refno 来的，与候选账对齐要按 refno 数。
    pub fn distinct_refnos(&self) -> usize {
        self.entries
            .iter()
            .map(|entry| (entry.refno.word0, entry.refno.word1))
            .collect::<BTreeSet<_>>()
            .len()
    }

    /// L3 对 L4 的守恒：账本上每个 refno 恰好有一个去向、没有凭空的去向。
    /// 返回去向计数，调用方再与 `IncrementReport` 的三本候选账逐项核对。
    pub fn accounts_for(&self) -> Result<DispositionTally, String> {
        let on_ledger: BTreeSet<(u32, u32)> = self
            .entries
            .iter()
            .map(|entry| (entry.refno.word0, entry.refno.word1))
            .collect();
        let mut seen: BTreeSet<(u32, u32)> = BTreeSet::new();
        let mut tally = DispositionTally::default();
        for (key, disposition) in &self.dispositions {
            if !on_ledger.contains(key) {
                return Err(format!(
                    "{}/{} 有去向 {disposition:?} 却不在账本上",
                    key.0, key.1
                ));
            }
            if !seen.insert(*key) {
                return Err(format!("{}/{} 的去向登记了两次", key.0, key.1));
            }
            match disposition {
                Disposition::RolledUp => tally.rolled_up += 1,
                Disposition::NoModel => tally.no_model += 1,
                Disposition::Unresolved => tally.unresolved += 1,
            }
        }
        if let Some(missing) = on_ledger.difference(&seen).next() {
            return Err(format!(
                "{}/{} 在账本上却没有去向（rolled_up / no_model / unresolved 之一）",
                missing.0, missing.1
            ));
        }
        Ok(tally)
    }

    /// 新建。`owner` 取 **target** 端的属主。
    pub fn created(&mut self, refno: RefNo, noun: String, owner: Option<RefNo>) {
        self.push_rooted(
            LedgerEntry {
                refno,
                noun,
                kind: ChangeKind::Created,
                detail: "base 端不存在".into(),
                preempted_by: None,
            },
            owner,
        );
    }

    /// 删除。`owner` 取 **base** 端的属主。
    pub fn deleted(&mut self, refno: RefNo, noun: String, owner: Option<RefNo>) {
        self.push_rooted(
            LedgerEntry {
                refno,
                noun,
                kind: ChangeKind::Deleted,
                detail: "target 端不存在".into(),
                preempted_by: None,
            },
            owner,
        );
    }

    /// 把一份 L2 差分展开进账。内容没变的候选只累计到
    /// [`ChangeTally::unchanged`]，不占条目。
    pub fn record(&mut self, refno: RefNo, diff: &ElementDiff) {
        if diff.is_unchanged() {
            self.unchanged += 1;
            return;
        }
        let noun = diff.noun.clone();
        let mut push = |kind: ChangeKind, detail: String| {
            self.entries.push(LedgerEntry {
                refno,
                noun: noun.clone(),
                kind,
                detail,
                preempted_by: None,
            });
        };

        if let Some((before, after)) = &diff.type_changed {
            push(
                ChangeKind::TypeChanged,
                format!("{before} → {after}；整棵子树按删除后重建处置"),
            );
        }
        if let Some((before, after)) = &diff.owner_changed {
            let shown = |owner: &Option<String>| owner.clone().unwrap_or_else(|| "（无）".into());
            push(
                ChangeKind::Reparented,
                format!("属主 {} → {}", shown(before), shown(after)),
            );
        }
        if let Some(members) = &diff.members {
            let detail = if members.reordered {
                "成员次序变了，集合没变".to_string()
            } else {
                format!("成员 +{} -{}", members.added.len(), members.removed.len())
            };
            push(ChangeKind::MembersChanged, detail);
        }
        if !diff.attributes.is_empty() {
            push(
                ChangeKind::AttributesModified,
                format!("{} 变了", diff.attributes.join(" / ")),
            );
        }
        if diff.opaque {
            push(
                ChangeKind::OpaqueRecordChange,
                "记录字节不同，属性表逐项相等（UDA 或未解码的存储形态）".into(),
            );
        }
    }

    fn push_rooted(&mut self, entry: LedgerEntry, owner: Option<RefNo>) {
        if let Some(owner) = owner {
            self.owners.push((self.entries.len(), owner));
        }
        self.entries.push(entry);
    }

    /// 算祖先抢占。**只标记，不删条目**（见模块文档）。
    pub fn finish(&mut self) {
        let bucket = |kind: ChangeKind, entries: &[LedgerEntry]| -> BTreeSet<(u32, u32)> {
            entries
                .iter()
                .filter(|entry| entry.kind == kind)
                .map(|entry| (entry.refno.word0, entry.refno.word1))
                .collect()
        };
        let created = bucket(ChangeKind::Created, &self.entries);
        let deleted = bucket(ChangeKind::Deleted, &self.entries);
        for (at, owner) in &self.owners {
            let entry = &mut self.entries[*at];
            let same_bucket = match entry.kind {
                ChangeKind::Created => &created,
                ChangeKind::Deleted => &deleted,
                _ => continue,
            };
            if same_bucket.contains(&(owner.word0, owner.word1)) {
                entry.preempted_by = Some(*owner);
            }
        }
    }

    pub fn entries(&self) -> &[LedgerEntry] {
        &self.entries
    }

    pub fn of_kind(&self, kind: ChangeKind) -> impl Iterator<Item = &LedgerEntry> {
        self.entries.iter().filter(move |entry| entry.kind == kind)
    }

    pub fn tally(&self) -> ChangeTally {
        let count = |kind: ChangeKind| self.of_kind(kind).count();
        let roots = |kind: ChangeKind| {
            self.of_kind(kind)
                .filter(|entry| entry.preempted_by.is_none())
                .count()
        };
        ChangeTally {
            created: count(ChangeKind::Created),
            deleted: count(ChangeKind::Deleted),
            type_changed: count(ChangeKind::TypeChanged),
            reparented: count(ChangeKind::Reparented),
            members_changed: count(ChangeKind::MembersChanged),
            attributes_modified: count(ChangeKind::AttributesModified),
            opaque_record_changes: count(ChangeKind::OpaqueRecordChange),
            unresolved: count(ChangeKind::Unresolved),
            unchanged: self.unchanged,
            created_subtree_roots: roots(ChangeKind::Created),
            deleted_subtree_roots: roots(ChangeKind::Deleted),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::element_diff::MemberDelta;

    fn diff(noun: &str) -> ElementDiff {
        ElementDiff {
            refno: "1/2".into(),
            noun: noun.into(),
            type_changed: None,
            owner_changed: None,
            members: None,
            attributes: Vec::new(),
            opaque: false,
        }
    }

    /// 同一个元素既改挂又改属性，账上就该是两条。
    ///
    /// core 的 `checkEle` 对这种情况同样是 `includeAfter` 与 `attChange` 各调一次。
    /// 合成一条的代价是「这一窗有多少次改挂」再也数不出来。
    #[test]
    fn one_element_can_owe_several_entries() {
        let mut ledger = ChangeLedger::new();
        ledger.record(
            RefNo::new(1, 2),
            &ElementDiff {
                owner_changed: Some((Some("1/7".into()), Some("1/8".into()))),
                attributes: vec!["POS".into()],
                ..diff("PANE")
            },
        );
        ledger.finish();

        let tally = ledger.tally();
        assert_eq!(tally.reparented, 1);
        assert_eq!(tally.attributes_modified, 1);
        assert_eq!(ledger.entries().len(), 2);
    }

    /// 内容没变的候选不占条目，只累计——它是 L1 与 L2 的落差，不是一种改动。
    #[test]
    fn an_unchanged_candidate_is_a_gap_not_a_change() {
        let mut ledger = ChangeLedger::new();
        ledger.record(RefNo::new(1, 2), &diff("PANE"));
        ledger.finish();

        assert!(ledger.entries().is_empty());
        assert_eq!(ledger.tally().unchanged, 1);
    }

    /// 祖先抢占只标记不丢弃：整棵删除子树的条目一条不少，但数得出哪几条是顶。
    ///
    /// 丢掉非顶条目是 core 的做法，它有「已画出的图形段树」可以反问；我们没有，
    /// 丢了就再也点不出那些 refno——`ams1112` 那 3348 件正体会整片蒸发。
    #[test]
    fn subtree_roots_are_marked_and_nothing_is_dropped() {
        let (root, child, grandchild) = (RefNo::new(1, 1), RefNo::new(1, 2), RefNo::new(1, 3));
        let mut ledger = ChangeLedger::new();
        ledger.deleted(root, "CWALL".into(), Some(RefNo::new(1, 99)));
        ledger.deleted(child, "PANE".into(), Some(root));
        ledger.deleted(grandchild, "PLOO".into(), Some(child));
        ledger.finish();

        let tally = ledger.tally();
        assert_eq!(tally.deleted, 3, "条目一条都不许少");
        assert_eq!(tally.deleted_subtree_roots, 1, "只有 CWALL 是子树顶");

        let preempted: Vec<_> = ledger
            .of_kind(ChangeKind::Deleted)
            .filter(|entry| entry.preempted_by.is_some())
            .map(|entry| entry.noun.as_str())
            .collect();
        assert_eq!(preempted, vec!["PANE", "PLOO"]);
    }

    /// 新建与删除各算各的：属主在**另一个**桶里不算抢占。
    #[test]
    fn preemption_does_not_cross_buckets() {
        let owner = RefNo::new(1, 1);
        let mut ledger = ChangeLedger::new();
        ledger.deleted(owner, "CWALL".into(), None);
        ledger.created(RefNo::new(1, 2), "PANE".into(), Some(owner));
        ledger.finish();

        let tally = ledger.tally();
        assert_eq!(tally.created_subtree_roots, 1, "属主在删除桶里，不抢占新建");
        assert_eq!(tally.deleted_subtree_roots, 1);
    }

    /// L3 对 L4 的守恒：账本上每个 refno 恰好一个去向，缺一个、多一个、登记两次都要响。
    ///
    /// 读不出的候选也得先进账本再登记去向——`unresolved` 那本账此前只在报告里有个数，
    /// 账本上没有它们，「候选 N、账本 N−4」那 4 条去了哪里没人说得出。
    #[test]
    fn every_ledgered_refno_has_exactly_one_disposition() {
        let (a, b, c) = (RefNo::new(1, 1), RefNo::new(1, 2), RefNo::new(1, 3));
        let mut ledger = ChangeLedger::new();
        ledger.created(a, "BOX".into(), None);
        ledger.record(
            b,
            &ElementDiff {
                owner_changed: Some((Some("1/7".into()), Some("1/8".into()))),
                attributes: vec!["POS".into()],
                ..diff("PANE")
            },
        );
        ledger.unresolved(c, "?", "owner 链断");
        ledger.finish();

        // 少一个去向：b 有两条条目，仍只算一个 refno、要一个去向。
        ledger.dispose(a, Disposition::RolledUp);
        ledger.dispose(c, Disposition::Unresolved);
        let err = ledger.accounts_for().unwrap_err();
        assert!(err.contains("1/2") && err.contains("没有去向"), "{err}");

        ledger.dispose(b, Disposition::NoModel);
        assert_eq!(
            ledger.accounts_for().unwrap(),
            DispositionTally {
                rolled_up: 1,
                no_model: 1,
                unresolved: 1,
            }
        );
        assert_eq!(ledger.distinct_refnos(), 3);
        assert_eq!(ledger.tally().unresolved, 1);

        // 登记两次是规划器的 bug，不是数据问题。
        ledger.dispose(a, Disposition::NoModel);
        let err = ledger.accounts_for().unwrap_err();
        assert!(err.contains("两次"), "{err}");

        // 凭空的去向：账本上没有的 refno。
        let mut stray = ChangeLedger::new();
        stray.dispose(RefNo::new(9, 9), Disposition::RolledUp);
        let err = stray.accounts_for().unwrap_err();
        assert!(err.contains("不在账本上"), "{err}");
    }

    /// 成员表的三种变化各自说得出话。
    #[test]
    fn a_member_change_says_which_kind_it_is() {
        let mut ledger = ChangeLedger::new();
        ledger.record(
            RefNo::new(1, 2),
            &ElementDiff {
                members: Some(MemberDelta {
                    reordered: true,
                    ..MemberDelta::default()
                }),
                ..diff("BRAN")
            },
        );
        ledger.finish();
        assert_eq!(ledger.tally().members_changed, 1);
        assert!(ledger.entries()[0].detail.contains("次序"));
    }
}

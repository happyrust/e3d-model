//! 设计树遍历与总账（对标 `add/ADDDES` 的 DFS + `Core3D` 的批任务收口）。
//!
//! 遍历规则（成员一律文件原序）：
//!
//! | 类别 | 动作 |
//! |---|---|
//! | `List` | 无 |
//! | `LoopExtrusion` / `Primitive` / `Revolution` / `Polyhedron` | [`model_element`] |
//! | `Catalog` / `Unsupported` / `Unknown` | 记账 |
//! | `NonGraphic` | 无 |
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

use crate::category::{Category, classify};
use crate::elmodl::{GeneratedElement, Modeled, model_element};

/// 一条有名有姓的记录。
#[derive(Debug, Clone, serde::Serialize)]
pub struct Incident {
    pub refno: String,
    pub noun: String,
    pub detail: String,
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
    /// 认识但本期不做：noun → 数量。
    pub unsupported: BTreeMap<String, usize>,
    /// 分类表没认领：noun → 数量。出现即分类表欠账。
    pub unknown_nouns: BTreeMap<String, usize>,
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
}

/// 从 `root` 起 DFS 生成整棵子树。遍历中的 I/O 错误（读不到成员表）直接上抛——
/// 那是库文件层面的问题，不是单元素账。
pub fn generate_subtree(root: DbElement) -> Result<GenerationOutcome, DbElementError> {
    let mut elements = Vec::new();
    let mut report = Report::default();
    // 显式栈，避免深树递归爆栈；压栈顺序反转以保持文件原序遍历。
    let mut stack: Vec<DbElement> = vec![root];

    while let Some(el) = stack.pop() {
        report.visited += 1;
        let noun = el.element_type()?;
        let category = classify(&noun);
        let refno = el.refno().to_string();

        match category {
            Category::List | Category::NonGraphic => {}
            Category::LoopExtrusion
            | Category::Primitive(_)
            | Category::Revolution
            | Category::Polyhedron => match model_element(&el, category) {
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
                    report.generated += 1;
                    elements.push(*generated);
                }
                Ok(Modeled::Skipped { reason }) => report.skipped.push(Incident {
                    refno,
                    noun,
                    detail: reason,
                }),
                Err(error) => report.failed.push(Incident {
                    refno,
                    noun,
                    detail: error.to_string(),
                }),
            },
            Category::Catalog => {
                *report.catalog_pending.entry(noun.clone()).or_insert(0) += 1;
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

    Ok(GenerationOutcome { elements, report })
}

/// 成员入栈。被本元素消费掉的（自己的剖面环与负体）不入栈，整棵子树计进
/// [`Report::consumed`]——不计的话它们就从账上蒸发了。
fn push_members(
    el: &DbElement,
    category: Category,
    stack: &mut Vec<DbElement>,
    report: &mut Report,
) -> Result<(), DbElementError> {
    let consumes = category.consumes_members();
    let mut batch = Vec::new();
    for member in el.members()? {
        let noun = member.element_type()?;
        let member_category = classify(&noun);
        let consumed = consumes
            && (matches!(member_category, Category::ProfileData) || member_category.is_negative());
        if consumed {
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
            unsupported,
            unknown_nouns,
            consumed,
            orphans,
            negatives_skipped,
            notes,
        } = other;
        self.visited += visited;
        self.generated += generated;
        self.skipped.extend(skipped);
        self.failed.extend(failed);
        self.negatives_skipped.extend(negatives_skipped);
        self.notes.extend(notes);
        for (target, source) in [
            (&mut self.catalog_pending, catalog_pending),
            (&mut self.unsupported, unsupported),
            (&mut self.unknown_nouns, unknown_nouns),
            (&mut self.consumed, consumed),
            (&mut self.orphans, orphans),
        ] {
            for (noun, count) in source {
                *target.entry(noun).or_insert(0) += count;
            }
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

    /// 各本账 + 生成数是否把 visited 解释干净。对拍前先跑这个自检，
    /// 账对不上就别谈几何。
    pub fn totals_line(&self) -> String {
        format!(
            "visited={} consumed={} generated={} skipped={} failed={} catalog={} \
             unsupported={} unknown={} orphans={} neg_skipped={}",
            self.visited,
            self.consumed_total(),
            self.generated,
            self.skipped.len(),
            self.failed.len(),
            self.catalog_pending.values().sum::<usize>(),
            self.unsupported.values().sum::<usize>(),
            self.unknown_nouns.values().sum::<usize>(),
            self.orphans.values().sum::<usize>(),
            self.negatives_skipped.len(),
        )
    }
}

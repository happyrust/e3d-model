//! L2 逐元素差分：把索引候选的「记录位置变了」折算成「到底哪里变了」。
//!
//! 对标 core.dll 的 `DB_Compare::checkEle`（`0x5a45cd0`）与它的底层原语
//! `DB_Element::hasAttributeChangedBetween`（`0x593b930`）。
//!
//! ## 为什么必须有这一层
//!
//! 索引差分（`e3d_io::index::IndexDiff`，对标 `DB_IndexTableCompare`）给出的
//! `Modified` 只说明**记录位置变了**——记录被重写过。core 拿到这个候选之后一定
//! 回去做记录级、逐属性、按类型比值的权威差分，`checkEle` 才是它的结论。
//! 少了这一层，「记录被重写」与「内容真的变了」就分不开，而这两件事在真实语料里
//! 差着数量级。
//!
//! ## 与 core 的一处实现差异（更省，不更松）
//!
//! `hasAttributeChangedBetween` 每比一个属性都要 `switchToOldSession` 到两端各读
//! 一次，所以它按 `DB_Attribute::type()` 分派、一次只比一个。我们两端各是一个钉死
//! 会话的 `DbSet`，一次 `DbElement::extraction` 就拿到整张属性表，而
//! `DescriptorValue` 本身就是 core 要靠 type code 分派才能得到的那个联合体——
//! 于是同样的语义只要一趟线性比较。判据的**严格程度**不变：逐属性、按值、
//! 两端各读一次。
//!
//! ## 记录变了却说不出哪里变了，不许当没变
//!
//! 属性表只覆盖模板声明的属性。纯 UDA、以及本 crate 尚未解码的存储形态，
//! 都可能让记录字节不同而属性表逐项相等。那种情况记 [`ElementDiff::opaque`]，
//! 由调用方按「变了」处置——把它折算成「没变」就是静默缺件。

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use e3d_io::db_element::{DbElement, DbElementError, DbSet};
use e3d_io::record::descriptor::DescriptorValue;
use e3d_io::refno::RefNo;

use crate::transform::PLACEMENT_ATTRIBUTES;

/// 成员表在两端之间的变化（对标 `checkEle` 的 `reorder*` 与次层级成员分支）。
#[derive(Debug, Clone, PartialEq, Eq, Default, serde::Serialize)]
pub struct MemberDelta {
    pub added: Vec<String>,
    pub removed: Vec<String>,
    /// 两端成员集合相同、次序不同。core 把它记成 `reorderFirst`/`reorderAfter`；
    /// 对我们而言它同样有意义：`BRAN` 的成员序就是管路走向。
    pub reordered: bool,
}

impl MemberDelta {
    fn is_empty(&self) -> bool {
        self.added.is_empty() && self.removed.is_empty() && !self.reordered
    }
}

/// 一个元素在窗口两端之间的完整差分。
///
/// 字段与 `checkEle` 的回调一一对应：`type_changed` ↔ `dealWithChangeType`、
/// `owner_changed` ↔ `includeFirst`/`includeAfter`、`members` ↔ `reorder*` 与
/// `*SecondaryList`、`attributes` ↔ `attChange`。
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct ElementDiff {
    pub refno: String,
    /// 目标端的 noun。`type_changed` 非空时这是**新**类型。
    pub noun: String,
    /// noun 变了：`(base, target)`。core 走 `dealWithChangeType`——先整体删除，
    /// 再把子树重建。类型变了之后属性表比不了（两个 noun 的模板根本不是一张表），
    /// 所以命中这一条时 `attributes` 恒空。
    pub type_changed: Option<(String, String)>,
    /// 属主变了：`(base 端属主, target 端属主)`，无效 ref 记 `None`。
    ///
    /// **这就是改挂。** core 在 `checkEle` 里显式测 `ATT_OWNER`，命中调
    /// `DB_UserChanges::elementIncluded(el, 旧属主)`，把元素、旧属主、新属主
    /// 三个都记进账。
    pub owner_changed: Option<(Option<String>, Option<String>)>,
    /// 成员表变化；无变化时 `None`。
    pub members: Option<MemberDelta>,
    /// 值不相等的属性名，字典序。
    pub attributes: Vec<String>,
    /// 记录字节不同，但上面四项都说没变。见模块文档最后一节。
    pub opaque: bool,
}

impl ElementDiff {
    /// 两端记录逐字节相同：这个候选只是被重写过，内容一个字都没动。
    pub fn is_unchanged(&self) -> bool {
        self.type_changed.is_none()
            && self.owner_changed.is_none()
            && self.members.is_none()
            && self.attributes.is_empty()
            && !self.opaque
    }

    /// 这个元素一动，**它子树里所有已建模元素的世界矩阵**变不变。
    ///
    /// 两条来源，缺一不可（见 [`crate::transform::world_matrix`]）：
    ///
    /// 1. [`PLACEMENT_ATTRIBUTES`]（`POS`/`ORI`）任一变 —— 本级局部矩阵变了；
    /// 2. **属主变了** —— 局部矩阵没动，但折叠时右乘的那条链换了一条。
    ///
    /// 第 2 条是老实现漏掉的那一半：旧的 `placement_drifted` 只比两端的
    /// `(POS, ORI)`，一个元素整体改挂到别的属主下、自己的 POS/ORI 一个字没动时，
    /// 它判「没漂移」，于是子树整片不重建，而世界矩阵其实全变了。
    ///
    /// 类型变了、以及记录变了却指不出哪一项，都保守地算变了。
    pub fn placement_input_changed(&self) -> bool {
        self.type_changed.is_some()
            || self.owner_changed.is_some()
            || self.opaque
            || self
                .attributes
                .iter()
                .any(|name| PLACEMENT_ATTRIBUTES.contains(&name.as_str()))
    }

    fn unchanged(refno: RefNo, noun: String) -> Self {
        Self {
            refno: refno.to_string(),
            noun,
            type_changed: None,
            owner_changed: None,
            members: None,
            attributes: Vec::new(),
            opaque: false,
        }
    }
}

/// 比一个元素在两端的记录（对标 `DB_Compare::checkEle`）。
///
/// 两端都必须有这个元素——`Modified` 候选按定义就是两端都在。任一端读不出来
/// 是数据或读层问题，原样上浮，不折算成「变了」。
pub fn diff_element(
    base: &Arc<DbSet>,
    target: &Arc<DbSet>,
    refno: RefNo,
) -> Result<ElementDiff, DbElementError> {
    let base_el = base.element(refno);
    let target_el = target.element(refno);
    let before = base_el.parsed()?;
    let after = target_el.parsed()?;

    let noun = target_el.element_type()?;

    // 记录级快筛：位置变了不等于内容变了。这一步既省掉绝大多数候选的属性抽取，
    // 也是 UDA 与未解码存储形态的唯一哨兵（属性表覆盖不到它们）。
    let record_identical = before.noun_hash == after.noun_hash
        && before.owner == after.owner
        && before.members == after.members
        && before.implicit_data == after.implicit_data
        && before.explicit_attributes == after.explicit_attributes;
    if record_identical {
        return Ok(ElementDiff::unchanged(refno, noun));
    }

    let type_changed = (before.noun_hash != after.noun_hash).then(|| {
        (
            base_el.element_type().unwrap_or_else(|_| "?".into()),
            noun.clone(),
        )
    });

    let owner_changed = (before.owner != after.owner).then(|| {
        let name = |refno: RefNo| refno.is_valid().then(|| refno.to_string());
        (name(before.owner), name(after.owner))
    });

    let members = member_delta(&before.members, &after.members);

    // 类型变了就没有共同的属性表可比。core 同样不比：`dealWithChangeType` 直接
    // 走「删除 + 整棵子树重建」。
    let attributes = if type_changed.is_some() {
        Vec::new()
    } else {
        changed_attributes(&base_el, &target_el)?
    };

    let opaque = type_changed.is_none()
        && owner_changed.is_none()
        && members.is_none()
        && attributes.is_empty();

    Ok(ElementDiff {
        refno: refno.to_string(),
        noun,
        type_changed,
        owner_changed,
        members,
        attributes,
        opaque,
    })
}

fn member_delta(before: &[RefNo], after: &[RefNo]) -> Option<MemberDelta> {
    if before == after {
        return None;
    }
    let counted = |list: &[RefNo]| -> BTreeMap<(u32, u32), usize> {
        let mut counts = BTreeMap::new();
        for refno in list {
            *counts.entry((refno.word0, refno.word1)).or_insert(0) += 1;
        }
        counts
    };
    let (left, right) = (counted(before), counted(after));
    let surplus = |from: &BTreeMap<(u32, u32), usize>, other: &BTreeMap<(u32, u32), usize>| {
        from.iter()
            .filter(|(key, count)| other.get(*key).copied().unwrap_or(0) < **count)
            .map(|(key, _)| RefNo::new(key.0, key.1).to_string())
            .collect::<Vec<_>>()
    };
    let added = surplus(&right, &left);
    let removed = surplus(&left, &right);
    let delta = MemberDelta {
        reordered: added.is_empty() && removed.is_empty(),
        added,
        removed,
    };
    (!delta.is_empty()).then_some(delta)
}

/// 逐属性按值比两端，返回不相等的属性名（字典序）。
///
/// 按属性 hash 配对而不是按下标：两端的模板理论上同源同序，但靠下标对齐会把
/// 「模板换了一版」变成整表错位，而错位的表现是一大片属性同时报变。
fn changed_attributes(
    base_el: &DbElement,
    target_el: &DbElement,
) -> Result<Vec<String>, DbElementError> {
    let before = base_el.extraction()?;
    let after = target_el.extraction()?;
    let indexed: BTreeMap<u32, &Option<DescriptorValue>> = before
        .attributes
        .iter()
        .map(|attribute| (attribute.hash, &attribute.value))
        .collect();

    let mut changed: Vec<String> = Vec::new();
    for attribute in &after.attributes {
        let same = match indexed.get(&attribute.hash) {
            Some(previous) => values_equal(previous, &attribute.value),
            // base 端模板里根本没有这个属性：有值才算变化。
            None => !is_set(&attribute.value),
        };
        if !same {
            changed.push(attribute.name.clone());
        }
    }
    let present: BTreeSet<u32> = after.attributes.iter().map(|a| a.hash).collect();
    for attribute in &before.attributes {
        if !present.contains(&attribute.hash) && is_set(&attribute.value) {
            changed.push(attribute.name.clone());
        }
    }

    changed.sort();
    changed.dedup();
    Ok(changed)
}

/// `Some(Unset)` 与 `None` 是同一件事——这是 `DbElement::attribute` 的既定契约
/// （「属性在模板里但逻辑 unset」与「本记录没解出值」都给 `None`）。抽取层保留
/// 两种表示，比对层要是不归一，同一个「没有值」会因存储形态不同被报成变化。
fn is_set(value: &Option<DescriptorValue>) -> bool {
    !matches!(value, None | Some(DescriptorValue::Unset))
}

fn values_equal(left: &Option<DescriptorValue>, right: &Option<DescriptorValue>) -> bool {
    match (is_set(left), is_set(right)) {
        (false, false) => true,
        (true, true) => descriptor_values_equal(
            left.as_ref().expect("is_set"),
            right.as_ref().expect("is_set"),
        ),
        _ => false,
    }
}

/// `DescriptorValue` 的等值判定。
///
/// 不能直接用 derive 的 `PartialEq`：那条路上 `f64` 的 `NaN != NaN`，而未设置的
/// 实数槽解出 NaN 是常有的事——两端都是 NaN 会被报成「变了」，于是一批根本没动的
/// 元素每一窗都重建一次。这里把 `NaN == NaN` 判真、`-0.0 == 0.0` 判真：两者产出的
/// 几何逐位相同，判等只会少建，不会漏建。**其余一切差异照旧算变化。**
fn descriptor_values_equal(left: &DescriptorValue, right: &DescriptorValue) -> bool {
    match (left, right) {
        (DescriptorValue::Real(left), DescriptorValue::Real(right)) => reals_equal(*left, *right),
        (DescriptorValue::RealArray(left), DescriptorValue::RealArray(right)) => {
            left.len() == right.len()
                && left
                    .iter()
                    .zip(right)
                    .all(|(left, right)| reals_equal(*left, *right))
        }
        _ => left == right,
    }
}

fn reals_equal(left: f64, right: f64) -> bool {
    left == right || (left.is_nan() && right.is_nan())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn diff_with(attributes: Vec<&str>) -> ElementDiff {
        ElementDiff {
            refno: "1/2".into(),
            noun: "PANE".into(),
            type_changed: None,
            owner_changed: None,
            members: None,
            attributes: attributes.into_iter().map(str::to_owned).collect(),
            opaque: false,
        }
    }

    /// 未设置的实数槽两端都解出 NaN，是「没变」。
    ///
    /// derive 的 `PartialEq` 在这里会说「变了」，而它一变就是每一窗重建同一批元素，
    /// 报告上看不出任何异常——只是增量永远比它该有的规模大一圈。
    #[test]
    fn two_nans_are_the_same_value() {
        assert!(descriptor_values_equal(
            &DescriptorValue::Real(f64::NAN),
            &DescriptorValue::Real(f64::NAN),
        ));
        assert!(descriptor_values_equal(
            &DescriptorValue::RealArray(vec![1.0, f64::NAN]),
            &DescriptorValue::RealArray(vec![1.0, f64::NAN]),
        ));
        assert!(descriptor_values_equal(
            &DescriptorValue::Real(0.0),
            &DescriptorValue::Real(-0.0),
        ));
    }

    /// 判等只放过「产出几何逐位相同」的那两种，真实数值差异一律算变化。
    #[test]
    fn a_real_difference_is_still_a_change() {
        assert!(!descriptor_values_equal(
            &DescriptorValue::Real(1.0),
            &DescriptorValue::Real(1.000_000_000_1),
        ));
        assert!(!descriptor_values_equal(
            &DescriptorValue::Real(1.0),
            &DescriptorValue::Real(f64::NAN),
        ));
        assert!(!descriptor_values_equal(
            &DescriptorValue::RealArray(vec![1.0]),
            &DescriptorValue::RealArray(vec![1.0, 2.0]),
        ));
        // 类型换了也是变化：Int 1 与 Real 1.0 不是同一个值。
        assert!(!descriptor_values_equal(
            &DescriptorValue::Int(1),
            &DescriptorValue::Real(1.0),
        ));
    }

    /// unset 与「有值」永远不等价——E3D 的 unset 与 0/空串/Nulref 都不是一回事；
    /// 而 `Some(Unset)` 与 `None` 是同一件事，不许被报成变化。
    #[test]
    fn unset_is_not_a_value_but_it_only_has_one_meaning() {
        assert!(!values_equal(&None, &Some(DescriptorValue::Int(0))));
        assert!(!values_equal(&None, &Some(DescriptorValue::Bool(false))));
        assert!(values_equal(&None, &None));
        assert!(values_equal(&None, &Some(DescriptorValue::Unset)));
        assert!(values_equal(
            &Some(DescriptorValue::Unset),
            &Some(DescriptorValue::Unset)
        ));
    }

    /// 成员表：加、减、纯换序三种要分得开。
    #[test]
    fn a_member_delta_separates_reordering_from_membership() {
        let a = RefNo::new(1, 10);
        let b = RefNo::new(1, 11);
        let c = RefNo::new(1, 12);

        assert_eq!(member_delta(&[a, b], &[a, b]), None);

        let reorder = member_delta(&[a, b], &[b, a]).expect("换序算变化");
        assert!(reorder.reordered);
        assert!(reorder.added.is_empty() && reorder.removed.is_empty());

        let grew = member_delta(&[a], &[a, c]).expect("加成员算变化");
        assert_eq!(grew.added, vec![c.to_string()]);
        assert!(grew.removed.is_empty());
        assert!(!grew.reordered, "加了成员就不是单纯换序");

        let shrank = member_delta(&[a, b], &[a]).expect("减成员算变化");
        assert_eq!(shrank.removed, vec![b.to_string()]);
    }

    /// 世界矩阵的输入有两条来源，`placement_input_changed` 必须两条都认。
    ///
    /// 第二条（改挂）是老实现漏掉的那一半：`placement_drifted` 只比 `(POS, ORI)`，
    /// 元素整体挂到别的属主下、自己 POS/ORI 一个字没动时它说「没漂移」，
    /// 于是子树整片不重建——而世界矩阵全变了。
    #[test]
    fn a_reparent_moves_the_world_even_when_pos_and_ori_hold_still() {
        let moved = ElementDiff {
            owner_changed: Some((Some("1/7".into()), Some("1/8".into()))),
            ..diff_with(vec![])
        };
        assert!(moved.placement_input_changed());
        assert!(!moved.is_unchanged());

        assert!(diff_with(vec!["POS"]).placement_input_changed());
        assert!(diff_with(vec!["ORI"]).placement_input_changed());
        // 与矩阵无关的属性不该触发整棵子树重建。
        assert!(!diff_with(vec!["DIAM", "NAME"]).placement_input_changed());
    }

    /// 判据的属性名只能有一份：这里读的就是矩阵折叠读的那张表。
    #[test]
    fn the_cascade_trigger_reads_the_same_names_the_matrix_does() {
        for name in PLACEMENT_ATTRIBUTES {
            assert!(
                diff_with(vec![name]).placement_input_changed(),
                "{name} 是矩阵输入，必须触发级联"
            );
        }
    }

    /// 记录变了却指不出哪里变了，只能按「变了」处置，不许静默算没变。
    #[test]
    fn an_opaque_record_change_is_a_change() {
        let opaque = ElementDiff {
            opaque: true,
            ..diff_with(vec![])
        };
        assert!(!opaque.is_unchanged());
        assert!(opaque.placement_input_changed(), "说不出哪变了就得保守级联");
    }
}

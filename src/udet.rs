//! UDET（用户自定义元素类型）名字表：把设计元素的 `UDTYPE` 字翻回 E3D 用的 `:名`。
//!
//! E3D 允许工程在 DICT 库里给基础名词派生子类型。`/石楼装卸车-SUPP1` 的 17 个支架
//! 就是这么建的：库里的 noun 仍是 `ZONE/STRU/FRMW/SCTN/FIXING`，但每个元素的
//! `UDTYPE` 指着一条 `UDET` 定义，而 E3D 的显示名与 RVM 导出名一律用那条定义的
//! `DYUDNA` 加个冒号——`:SDS_Supp` / `:SDS_SubSupp` / `:SDS_CompSet` / `:SDS_Pole` /
//! `:SDS_Plate` / `:SDS_Entity`。
//!
//! 这不是显示口味问题：对拍主键是「属主名 + noun + 序号」，基准侧写的是 UDET 名，
//! 生成侧发基础名词，**两侧一件都配不上**（2026-09-14 实测：该 ZONE 120 件只在基准、
//! 34 件只在生成，`rvm_compare` 直接以「主键口径对不上」退出）。
//!
//! 序号同样得按 UDET 名数：`:SDS_Plate` 与 `:SDS_Entity` 同是 `FIXING`，按基础名词
//! 数会得到 `FIXING 1..4`，E3D 给的是 `:SDS_Plate 1` 与 `:SDS_Entity 1..3`。
//!
//! 表本身极小（YCYK-E3D 实测 10 条）：UDET 躺在 DICT 库里，从库自报的 WORL 往下
//! DFS 一趟就取全。`UDTYPE` 的字值就是 UDET 的 `UKEY`，两侧同一个 u32
//! （实测 `/SDS_Support` UKEY=538101933，`/PS-0001_Solid` 的 UDTYPE 就是它）。

use std::collections::{BTreeMap, HashSet};
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};

use e3d_io::db_element::{DbElement, DbElementError, DbSet};
use e3d_io::record::descriptor::DescriptorValue;

/// 设计元素身上指向 UDET 定义的字。
const UDTYPE: &str = "UDTYPE";
/// UDET 定义自报的键，与设计元素的 `UDTYPE` 同一个 u32。
const UKEY: &str = "UKEY";
/// UDET 定义自报的类型名，E3D 显示时前面加冒号。
const DYUDNA: &str = "DYUDNA";

/// UKEY → `:名` 的表，外加一本「UDTYPE 有值却查不到」的账。
#[derive(Debug, Default)]
pub struct UdetCatalog {
    by_key: BTreeMap<u32, String>,
    /// 静默退回基础名词会让整族对拍无声错配，计数让它在总账里出声。
    unresolved: AtomicUsize,
}

impl UdetCatalog {
    /// 空表：没有 DICT 库可扫时的中性值，[`Self::display_noun`] 一律给基础名词。
    pub fn empty() -> Self {
        Self::default()
    }

    /// 从既有的 UKEY → `:名` 对建表。名字里的冒号由调用方给；扫描走
    /// [`Self::scan`]，这条是给测试与调用方自带表用的。
    pub fn from_pairs(pairs: impl IntoIterator<Item = (u32, String)>) -> Self {
        Self {
            by_key: pairs.into_iter().collect(),
            unresolved: AtomicUsize::new(0),
        }
    }

    /// 扫一批 DICT 库，把里面的 UDET 定义收成表。
    ///
    /// 每个库从它自报的 WORL 往下走成员表。DICT 库是几十个元素的量级
    /// （YCYK-E3D 的 6662 号库 68 个），一趟 DFS 的代价可以忽略；UDET 底下不再有
    /// UDET，遇到就不往下走。
    pub fn scan(set: &Arc<DbSet>, dbnos: &[u32]) -> Result<Self, DbElementError> {
        let mut by_key = BTreeMap::new();
        for &dbno in dbnos {
            let world = set.metadata(dbno)?.world;
            let mut seen: HashSet<_> = HashSet::new();
            let mut stack = vec![set.element(world)];
            while let Some(element) = stack.pop() {
                if !seen.insert(element.refno()) {
                    continue;
                }
                if element.element_type()? == "UDET" {
                    if let Some((key, noun)) = read_udet(&element)? {
                        by_key.insert(key, noun);
                    }
                    continue;
                }
                stack.extend(element.members()?);
            }
        }
        Ok(Self {
            by_key,
            unresolved: AtomicUsize::new(0),
        })
    }

    pub fn len(&self) -> usize {
        self.by_key.len()
    }

    pub fn is_empty(&self) -> bool {
        self.by_key.is_empty()
    }

    /// `UDTYPE` 有值却不在表里的次数。非零 = 这次生成里有一族元素按基础名词发了键，
    /// 对拍会把它们记成「只在生成」。
    pub fn unresolved(&self) -> usize {
        self.unresolved.load(Ordering::Relaxed)
    }

    /// 一个元素在 E3D 眼里的名词：有 UDET 就是 `:名`，否则是库里的基础名词。
    pub fn display_noun(&self, element: &DbElement) -> Result<String, DbElementError> {
        let base = element.element_type()?;
        if self.by_key.is_empty() {
            return Ok(base);
        }
        // `UDTYPE` 不在所有名词的模板里（`PANE` 有、`DDSE` 没有），问之前先确认，
        // 否则 `attribute` 会按「拼错的属性名」报错。
        if !element.has_attribute(UDTYPE)? {
            return Ok(base);
        }
        let Some(DescriptorValue::Word { raw, .. }) = element.attribute(UDTYPE)? else {
            return Ok(base);
        };
        Ok(self.noun_for(raw, &base))
    }

    /// 纯查表那一步：`udtype` 为 0（未派生）或表里没有时退回 `base`。
    ///
    /// 空表 = 这次根本没扫 DICT 库，没有「查不到」可言，别把每个元素都记进账。
    fn noun_for(&self, udtype: u32, base: &str) -> String {
        if udtype == 0 || self.by_key.is_empty() {
            return base.to_string();
        }
        match self.by_key.get(&udtype) {
            Some(noun) => noun.clone(),
            None => {
                self.unresolved.fetch_add(1, Ordering::Relaxed);
                base.to_string()
            }
        }
    }
}

/// 一条 UDET 定义里本表要的两格。缺任一格的定义不进表——宁可退回基础名词，
/// 也不拿半条记录编一个名字出来。
fn read_udet(element: &DbElement) -> Result<Option<(u32, String)>, DbElementError> {
    let Some(DescriptorValue::Word { raw, .. }) = element.attribute(UKEY)? else {
        return Ok(None);
    };
    let Some(name) = element.get_string(DYUDNA)? else {
        return Ok(None);
    };
    let name = name.trim();
    if raw == 0 || name.is_empty() {
        return Ok(None);
    }
    Ok(Some((raw, format!(":{name}"))))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// YCYK-E3D `/SolidSupport_ElementType` 下实测的三条：UDTYPE 的字就是 UKEY，
    /// 显示名是 `DYUDNA` 前面加冒号。
    fn measured() -> UdetCatalog {
        UdetCatalog::from_pairs([
            (538101933, ":SDS_Supp".to_string()),
            (538101939, ":SDS_Pole".to_string()),
            (538101940, ":SDS_Plate".to_string()),
            (538101938, ":SDS_Entity".to_string()),
        ])
    }

    #[test]
    fn a_known_udtype_replaces_the_base_noun() {
        let catalog = measured();
        assert_eq!(catalog.noun_for(538101933, "ZONE"), ":SDS_Supp");
        assert_eq!(catalog.noun_for(538101939, "SCTN"), ":SDS_Pole");
        assert_eq!(catalog.unresolved(), 0);
    }

    /// 同一个基础名词派生出两种 UDET：序号必须按 UDET 名分开数，所以两者不能同名。
    #[test]
    fn two_udets_over_one_base_noun_stay_distinct() {
        let catalog = measured();
        assert_ne!(
            catalog.noun_for(538101940, "FIXING"),
            catalog.noun_for(538101938, "FIXING")
        );
    }

    /// 没派生（UDTYPE 未设 / 为 0）就是基础名词，且不算「查不到」。
    #[test]
    fn an_unset_udtype_is_not_a_miss() {
        let catalog = measured();
        assert_eq!(catalog.noun_for(0, "ZONE"), "ZONE");
        assert_eq!(catalog.unresolved(), 0);
    }

    /// 查不到的 UDTYPE 退回基础名词，但要在账上留一条——这一族的对拍键是错的。
    #[test]
    fn an_unknown_udtype_falls_back_but_counts() {
        let catalog = measured();
        assert_eq!(catalog.noun_for(123456, "SCTN"), "SCTN");
        assert_eq!(catalog.unresolved(), 1);
    }

    /// 空表是中性的：没扫 DICT 库时一切照旧，也不该把每个元素都记成查不到。
    #[test]
    fn an_empty_catalog_changes_nothing() {
        let catalog = UdetCatalog::empty();
        assert!(catalog.is_empty());
        assert_eq!(catalog.noun_for(538101939, "SCTN"), "SCTN");
        assert_eq!(catalog.unresolved(), 0);
    }
}

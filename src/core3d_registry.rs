//! Core3D 会为哪些 noun 建几何——**从活体 IDB 枚举出来的那张表**。
//!
//! 这个模块回答的是 [`crate::category::classify`] 回答不了的那一半问题。
//! `classify` 说的是「**我们**打算怎么建它」；这里说的是「**E3D** 会不会建它」。
//! 总纲 §4 的 D3 闸门要求「分清『E3D 也不画』与『我们还没建』」——
//! 判据的另一半就在这张表里。
//!
//! # 为什么必须是独立的一张表，而不是靠字典家族位
//!
//! 已有的覆盖率钉（`tests/noun_coverage.rs`）拿 `GTGEOM` 级联家族位当分母。
//! 那个分母对旧 FORTRAN 路是对的，但**现代 CSG 路根本不看家族位**：
//! `CSG_TreeBuilder::getCSGTree@0x10715b30` 按 `actualType → type → hardType` 三查
//! noun→builder 注册表，查到就建。于是「有专用 builder、却没有级联家族位」的 noun
//! 落在那个分母**之外**——`HANDRA` / `RAIL` / `KICKPL` / `STRLNG` 四个正是如此，
//! 它们判 `Unknown` 而覆盖率钉断言 `unknown.len() == 0` 照样绿。
//!
//! 一个分母看不见的东西，换一个独立来源的分母就看得见了。这就是这张表的全部用处。
//!
//! # 表从哪来
//!
//! `data/core3d-builder-registry.json`，由 `tools/core3d_registry_dump.py`（在 IDA 里跑）
//! + `tools/build_core3d_registry.py` 对活体 `Core3D.dll` IDB 枚举生成，逐条带地址：
//!
//! - `CSG_TreeBuilder::addPlug@0x107158B0` 的 **80 个调用点**（13 个注册函数）；
//! - `DB_ModelBuilder::addPlug@0x108E79D0` 的 1 个调用点；
//! - 旧路 `CRDESI@0x10343B80` 的 **41 条 switch 臂**（case 立即数反哈希）。
//!
//! 每个 builder 的虚表第 0 槽（`getCSGTree`）函数体大小也记了下来：139~2261 字节，
//! **没有一个是 `return 0` 的桩**——所以「注册了」等于「真会建」，不需要再逐个反编译
//! 才敢把它当欠账。
//!
//! **内嵌而不是运行时读盘**：与 `route-nouns.json` / `noun-family-matrix.json` 同理，
//! 这是编译期就定死的常量，不该因为工作目录不同就换答案。

use std::collections::BTreeMap;
use std::sync::OnceLock;

use crate::category::{Category, classify};

const REGISTRY_JSON: &str = include_str!("../data/core3d-builder-registry.json");

#[derive(serde::Deserialize)]
struct RegistryFile {
    nouns: Vec<PlugEntry>,
    #[serde(rename = "legacySwitchArms")]
    legacy_switch_arms: Vec<LegacyEntry>,
}

#[derive(serde::Deserialize)]
struct PlugEntry {
    noun: String,
    builder: String,
    #[serde(rename = "getCsgTree")]
    get_csg_tree: Option<String>,
    #[serde(rename = "getCsgTreeBytes")]
    get_csg_tree_bytes: Option<u32>,
    sites: Vec<String>,
}

#[derive(serde::Deserialize)]
struct LegacyEntry {
    noun: String,
    code: i64,
}

/// Core3D 建这个 noun 的那条路。
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Core3dPath {
    /// 现代 C++ CSG 路：noun 挂在 `CSG_TreeBuilder` 的 builder 注册表上。
    CsgPlug {
        /// builder 类名，例如 `CAB_CSGRouteNode`。
        builder: &'static str,
        /// 该 builder 虚表第 0 槽 `getCSGTree` 的地址。
        get_csg_tree: Option<&'static str>,
        /// `getCSGTree` 的函数体字节数——用来一眼看出它是不是桩。
        get_csg_tree_bytes: Option<u32>,
        /// `addPlug` 的调用点地址（同一个 noun 可能注册多次）。
        sites: &'static [&'static str],
    },
    /// 旧 FORTRAN 路：`CRDESI@0x10343B80` 的 switch 有它的臂。
    LegacySwitch {
        /// noun 哈希码，即 switch 的 `case` 立即数。
        code: i64,
    },
}

impl Core3dPath {
    /// 一句能直接贴进报告 / 断言失败信息的出处。
    pub fn evidence(&self) -> String {
        match self {
            Core3dPath::CsgPlug {
                builder,
                get_csg_tree,
                get_csg_tree_bytes,
                sites,
            } => {
                let at = get_csg_tree.unwrap_or("?");
                let bytes = get_csg_tree_bytes.unwrap_or(0);
                format!(
                    "CSG builder {builder}::getCSGTree@{at}（{bytes} 字节），\
                     addPlug 调用点 {}",
                    sites.join(" / ")
                )
            }
            Core3dPath::LegacySwitch { code } => {
                format!("旧路 CRDESI@0x10343B80 的 case {code}")
            }
        }
    }
}

/// 已解析的注册表：noun → Core3D 走的那条（或那两条）路。
type Registry = BTreeMap<&'static str, Vec<Core3dPath>>;

/// 解析内嵌注册表，只做一次。
///
/// 解析失败直接 panic，理由与 [`crate::category`] 里那两份内嵌清单一字不差：
/// 它是编译期定死的常量，解析不了说明这份构建从一开始就是坏的。安静地退化成
/// 「注册表为空、差集恒为零」远比崩掉更难查——那正是报告全绿、欠账看不见的那种失败。
fn registry() -> &'static Registry {
    static REGISTRY: OnceLock<Registry> = OnceLock::new();
    REGISTRY.get_or_init(|| {
        let parsed: RegistryFile = serde_json::from_str(REGISTRY_JSON)
            .expect("内嵌的 core3d-builder-registry.json 解析失败");
        let mut out: Registry = BTreeMap::new();
        for entry in parsed.nouns {
            let sites: &'static [&'static str] = Box::leak(
                entry
                    .sites
                    .into_iter()
                    .map(|s| &*Box::leak(s.into_boxed_str()))
                    .collect::<Vec<_>>()
                    .into_boxed_slice(),
            );
            out.entry(Box::leak(entry.noun.into_boxed_str()))
                .or_default()
                .push(Core3dPath::CsgPlug {
                    builder: Box::leak(entry.builder.into_boxed_str()),
                    get_csg_tree: entry.get_csg_tree.map(|s| &*Box::leak(s.into_boxed_str())),
                    get_csg_tree_bytes: entry.get_csg_tree_bytes,
                    sites,
                });
        }
        for entry in parsed.legacy_switch_arms {
            out.entry(Box::leak(entry.noun.into_boxed_str()))
                .or_default()
                .push(Core3dPath::LegacySwitch { code: entry.code });
        }
        out
    })
}

/// Core3D 有没有为这个 noun 建几何的路径；有的话是哪几条。
///
/// 返回非空**不等于**每一件都会出几何——builder 可以对某一件返回空树。
/// 它等于的是：**举证责任翻转了**。判这个 noun「E3D 也不画」，现在需要一条
/// 带地址的反证，而不是默认值。
pub fn core3d_paths(noun: &str) -> &'static [Core3dPath] {
    let noun = noun.trim().to_ascii_uppercase();
    registry()
        .get(noun.as_str())
        .map(Vec::as_slice)
        .unwrap_or(&[])
}

/// Core3D 会为这个 noun 建几何。
pub fn builds_geometry(noun: &str) -> bool {
    !core3d_paths(noun).is_empty()
}

/// 注册表里的全部 noun（给测试与报告点名用）。
pub fn registered_nouns() -> impl Iterator<Item = &'static str> {
    registry().keys().copied()
}

/// **E3D 建、我们没认领**——总纲 §4 那道 D3 闸门的判据，全 crate 只有这一份。
///
/// 两边各判一半：[`crate::category::classify`] 说我们打算怎么建它，这张表说 E3D 会不会建它。
/// 两个 `Unknown` / `NonGraphic` 分别是：
///
/// - `Unknown` = 「还没搞清楚」。注册表里有它就说明搞清楚了：E3D 有名有姓的 builder 会建它。
/// - `NonGraphic` = 「**E3D 也不画**」。这是一句关于 E3D 的正面断言，注册表直接反驳它。
///
/// 判据写成函数而不是在用它的两处各写一遍 `matches!(…) && builds_geometry(…)`，
/// 是因为那两处一个是特征化基线（`tests/noun_coverage.rs`）、一个是实跑件数
/// （[`crate::pipeline::Report::not_wired`]），**它们必须数的是同一批 noun**。
/// 各写一份，先改的那一处会让基线照绿、报告上的件数却已经换了口径——
/// 而这两个数分歧的时候，谁都不会响。
///
/// # 已知盲区：只查了 `actualType` 这一格
///
/// `CSG_TreeBuilder::getCSGTree@0x10715B30` 按 **`actualType` → `type` → `hardType`
/// 三查**注册表。这张表是按 `actualType` 建的，所以「自己没有 builder、靠 `GTYP`
/// 重定向到别人那一格」的 noun 在这里**查不到，于是不算欠账**——而它其实有几何。
///
/// 已经逮到一个：`STWALL` 身上 `GTYP = WALL`，它按 `WALL` 建，语料里 302 件带完整的
/// `SPRE → SPRO + 4×SPVE` 参数化剖面链（recheck-4 §十）。它判 `Catalog` 所以没落进这本账，
/// 但换一个判 `Unknown` 的同型 noun 就会静默漏掉。
///
/// 收口要读语料侧的 `GTYP`，不是再翻一遍反编译——见 recheck-4 §10.5。
pub fn is_not_wired(noun: &str) -> bool {
    matches!(classify(noun), Category::Unknown | Category::NonGraphic) && builds_geometry(noun)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_embedded_registry_parses_and_is_not_empty() {
        // 注册表解析成空表是最坏的失败模式：差集恒为零，欠账断言永远绿。
        assert!(registered_nouns().count() > 100, "注册表条目数不对");
    }

    #[test]
    fn rnode_carries_a_real_builder_not_a_stub() {
        // §3.1 的那条判据反转就靠这个事实：1909 字节的 getCSGTree 不是「E3D 也不画」。
        let paths = core3d_paths("RNODE");
        let Some(Core3dPath::CsgPlug {
            builder,
            get_csg_tree_bytes,
            ..
        }) = paths.first()
        else {
            panic!("RNODE 应当在 CSG 注册表里，实得 {paths:?}");
        };
        assert_eq!(*builder, "CAB_CSGRouteNode");
        assert!(
            get_csg_tree_bytes.unwrap_or(0) > 1000,
            "RNODE 的 builder 变小了，重新确认它是不是被改成了桩"
        );
    }

    #[test]
    fn the_legacy_switch_contributes_ruled_solids() {
        // RULS/NRUL 是旧路唯一一对我们连 noun 都没认领的臂，别让它从表里掉出去。
        assert!(builds_geometry("RULS"), "RULS 应当在旧路 switch 里");
        assert!(builds_geometry("NRUL"), "NRUL 应当在旧路 switch 里");
    }

    /// 判据的三种落法各钉一个，免得它退化成「注册表里有就算欠账」。
    #[test]
    fn the_not_wired_predicate_needs_both_halves() {
        // NonGraphic + 1909 字节的 builder：判据反转那一类，最该被数出来的。
        assert!(is_not_wired("RNODE"));
        // Unknown + 具名 builder：ASL 那族连账本都没进对的四个之一。2026-09-06 那族
        // 接了三个——HANDRA 布局式、RAIL / KICKPL 详图式，只剩 STRLNG 还在这本账上。
        assert!(is_not_wired("STRLNG"));
        for wired in ["HANDRA", "RAIL", "KICKPL"] {
            assert!(!is_not_wired(wired), "{wired} 接了线，不再是欠账");
        }
        // 注册表里有，但我们已经建了——建了就不是欠账。
        assert!(!is_not_wired("BOX"));
        // 我们没认领，E3D 也没有 builder——这种落 unknown_nouns，不落这本账。
        assert!(!is_not_wired("ZZZZNOSUCHNOUN"));
    }
}

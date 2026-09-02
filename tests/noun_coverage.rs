//! NOUN 覆盖的回归钉，判据一律取自 `data/noun-family-matrix.json`
//! （dabacon 字典实读，core.dll 自己读的那份）。
//!
//! 计划：`gen-model/.planning/2026-08-31-noun-coverage-closure/task_plan.md`
//!
//! 这里刻意**不写**任何「noun → 类别」的对照表。覆盖审计此前用的就是一张从
//! `category.rs` 手抄进 Python 的影子表——分类表一改它不报错，只是悄悄给出过期答案。
//! 用一张会过期的表去审「有没有漏」是自相矛盾的，所以本文件只做两件事：
//! 拿字典当分母算覆盖率，以及拿字典去反查 `category.rs` 里写的 noun 名存不存在。

use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;

use e3d_model::category::{Category, classify, is_model_unit};

#[derive(serde::Deserialize)]
struct Matrix {
    #[serde(rename = "cascadeFamilies")]
    cascade_families: Vec<String>,
    nouns: Vec<Row>,
}

#[derive(serde::Deserialize)]
struct Row {
    noun_name: String,
    families: Vec<String>,
}

fn manifest(rel: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(rel)
}

fn matrix() -> Matrix {
    let path = manifest("data/noun-family-matrix.json");
    let bytes = std::fs::read(&path).unwrap_or_else(|e| {
        panic!(
            "读不到 {}：{e}（重跑 scripts/noun_family_probe.py）",
            path.display()
        )
    });
    serde_json::from_slice(&bytes).expect("noun-family-matrix.json 解析失败")
}

/// 命中 `GTGEOM` 谓词级联的 noun = core.dll 旧路会给它造几何段的全集 = 覆盖率的分母。
fn cascade_nouns(m: &Matrix) -> Vec<&Row> {
    let cascade: BTreeSet<&str> = m.cascade_families.iter().map(String::as_str).collect();
    m.nouns
        .iter()
        .filter(|r| r.families.iter().any(|f| cascade.contains(f.as_str())))
        .collect()
}

/// 覆盖率基线。数字写死是有意的：**它只该因为有人真的改了覆盖面而变**。
///
/// 分母（277）会随 E3D 字典版本变——换字典重跑 `scripts/noun_family_probe.py` 之后
/// 这条会响，那时要连同计划里的数字一起改，而不是把断言放宽。
#[test]
fn cascade_coverage_matches_the_recorded_baseline() {
    let m = matrix();
    let rows = cascade_nouns(&m);

    let mut built = 0usize;
    let mut ledgered = 0usize;
    let mut unknown: Vec<&str> = Vec::new();
    for row in &rows {
        let category = classify(&row.noun_name);
        if category.builds_geometry() {
            built += 1;
        } else if matches!(category, Category::Unknown) {
            unknown.push(&row.noun_name);
        } else {
            ledgered += 1;
        }
    }

    assert_eq!(rows.len(), 327, "级联内 noun 总数（覆盖率的分母）变了");
    // 39 = 33 + DISH/SLCY/NDIS/NSLC/ADISH/ASLCY（G2-2，2026-09-02）。
    assert_eq!(built, 39, "会建几何的 noun 数变了");
    assert_eq!(ledgered, 288, "记账不建的 noun 数变了");
    assert_eq!(
        unknown.len(),
        0,
        "分类表欠账数变了。降了是好事——把这里的数字一起改掉；\
         涨了说明有 noun 掉出了分类表。当前欠账：{unknown:?}"
    );
}

/// 欠账按家族分布。总数不动、分布变了同样是覆盖面出了变化，得看得见。
#[test]
fn every_e3d31_cascade_family_has_a_modelling_route() {
    let m = matrix();
    let cascade: BTreeSet<&str> = m.cascade_families.iter().map(String::as_str).collect();

    let mut by_family: BTreeMap<&str, usize> = BTreeMap::new();
    for row in cascade_nouns(&m) {
        if !matches!(classify(&row.noun_name), Category::Unknown) {
            continue;
        }
        for family in row.families.iter().filter(|f| cascade.contains(f.as_str())) {
            *by_family.entry(family.as_str()).or_default() += 1;
        }
    }

    let expected: BTreeMap<&str, usize> = BTreeMap::new();
    assert_eq!(by_family, expected, "欠账的家族分布变了");
}

/// **反查 `category.rs` 里写的 noun 名在字典里存不存在。**
///
/// 这条钉的是最阴的一类错：分类表里写了一个**不存在**的 noun。它不会报错、不会崩，
/// 只是那一支永远匹配不上——`POGON` 就是这么来的（多面体的面其实叫 `POLFAC`）。
/// 后果还不是建不出几何（`polyhedron.rs` 按成员位置走，几何照出），而是**账错了**：
/// 面落 `unknown`、其下 `POLOOP`/`LOOPTS` 落 `orphans`，而 `accounts_for` 是等式判据、
/// 元素没蒸发就照样平——**没有任何现有门禁会因此变红**。
///
/// 实现上直接读 `src/category.rs` 源码取字面量，而不是在这里再抄一份 noun 名单：
/// 抄一份就等于又造了一张会过期的影子表，正是本文件要消灭的东西。
#[test]
fn every_noun_named_in_the_classifier_actually_exists_in_the_dictionary() {
    let m = matrix();
    let known: BTreeSet<String> = m
        .nouns
        .iter()
        .map(|r| r.noun_name.clone())
        .filter(|n| !n.is_empty())
        .collect();

    let source = std::fs::read_to_string(manifest("src/category.rs")).expect("读不到 category.rs");
    let mut missing: Vec<String> = source
        .split('"')
        .skip(1)
        .step_by(2)
        // noun 短名：全大写字母数字、2~8 字符。分类表里的其它字面量（错误文案、
        // 属性名注释）都不长这样。
        .filter(|lit| {
            (2..=8).contains(&lit.len())
                && lit
                    .chars()
                    .all(|c| c.is_ascii_uppercase() || c.is_ascii_digit())
                && lit.chars().any(|c| c.is_ascii_uppercase())
        })
        .filter(|lit| !m.cascade_families.iter().any(|family| family == lit))
        .filter(|lit| !known.contains(*lit))
        .map(str::to_string)
        .collect();
    missing.sort();
    missing.dedup();

    // 空的：缺陷 1 已修（`POGON` → `POLFAC` + `POGO`），分类表里每个 noun 名现在
    // 都在字典里查得到。这一格保持空——再出现任何名字都是新写进去的笔误。
    const KNOWN_BOGUS: &[&str] = &[];

    assert_eq!(
        missing, KNOWN_BOGUS,
        "category.rs 里出现了字典查无的 noun 名：这一支永远匹配不上，\
         几何可能照出但账是错的，且不会有任何门禁报错"
    );
}

/// 字典给出的正负配对（`positiveEquivalent`）与分类表的一致性。
///
/// 同一个形状的正体与负体必须走同一条几何路径，否则会出现「负体扣得出、正体建不出」
/// 这种一半一半的状态——RVM 对拍时表现为体积偏差，但归因会指向错的地方。
#[test]
fn negative_primitives_and_their_positive_twins_agree() {
    // 12 个设计负图元（INCOMP 家族），配对取自字典字段 positiveEquivalent(778791)。
    for (negative, positive) in [
        ("NBOX", "BOX"),
        ("NCYL", "CYLI"),
        ("NPYR", "PYRA"),
        ("NRTO", "RTOR"),
        ("NCTO", "CTOR"),
        ("NPOLYH", "POLYHE"),
        ("NXTR", "EXTR"),
        ("NSNO", "SNOU"),
        ("NCON", "CONE"),
        // G2-2（2026-09-02）：DISH/SLCY 两对从「同时不建」翻成「同时可建」，
        // Core3D 现代路正负同挂 `CSG_BasicDIS` / `CSG_BasicSLC`。
        ("NDIS", "DISH"),
        ("NSLC", "SLCY"),
    ] {
        assert!(
            classify(negative).builds_geometry() && classify(positive).builds_geometry(),
            "{negative}/{positive} 必须同时可建"
        );
    }
}

/// 计划 §4 缺陷 2/3/4（记账部分）已关，这条改成钉住**修完的样子**防回退。
///
/// 它此前是特征化测试（断言缺陷还在，修好就变红）。N1 把三条都关了，于是断言
/// 整个翻面：翻面本身就是「基线跟着覆盖面一起动」这套机制在起作用。
#[test]
fn the_n1_defects_are_closed() {
    // 缺陷 2：NSLC 是设计负图元（primitive + INCOMP，正体 SLCY），补回负几何家族；
    // G2-2 之后它与正体同臂可建，但仍必须是负体。
    assert!(
        matches!(classify("NSLC"), Category::NegPrimitive(_)) && classify("NSLC").is_negative(),
        "NSLC 是设计负图元，得留在负几何家族里让属主记账"
    );
    // 缺陷 2：这三个字典里 geomset=true 且属 INGCOM，是几何集负体不是设计负图元。
    for noun in ["NSCY", "NSBO", "NLCY"] {
        assert_eq!(classify(noun), Category::NegGeometrySet, "{noun}");
    }

    // 缺陷 3：`A*` 族 5 个同形的接进基本体，其余显式记账，一个都不许留在 Unknown。
    assert_eq!(classify("AEXTR"), Category::LoopExtrusion);
    for noun in [
        "ABOX", "ACYLI", "APYRA", "ARTOR", "ACTOR", "ACONE", "ADISH", "ASNOU", "ASLCY", "APOLYH",
        "AREVO",
    ] {
        assert!(
            !matches!(classify(noun), Category::Unknown),
            "{noun} 是 IECOMP 设备图元，不该再落 Unknown"
        );
    }

    // 缺陷 4 的记账部分：几何集三族全部认领，且本期一个都不建。
    // 名单从字典现取而不是在这里抄一份——抄一份就又是一张会过期的影子表。
    let m = matrix();
    let sets: Vec<&Row> = m
        .nouns
        .iter()
        .filter(|r| {
            r.families
                .iter()
                .any(|f| matches!(f.as_str(), "IGMCOM" | "IG2COM" | "INGCOM"))
        })
        .collect();
    assert_eq!(sets.len(), 44, "几何集三族的 noun 数变了");
    for row in sets {
        let category = classify(&row.noun_name);
        assert!(
            matches!(category, Category::GeometrySet | Category::NegGeometrySet),
            "{} 是几何集 noun，实得 {category:?}",
            row.noun_name
        );
        assert!(
            !category.builds_geometry(),
            "{} 本期不建几何集",
            row.noun_name
        );
    }
}

/// 计划 §4 缺陷 5 已由 ida-bridge 证据关闭；缺陷 6 的 SPINE 属主语义仍待 RVM 对拍。
///
/// 这两条不像 2/3 那样是纯事实错误，改判前得先有出处（函数地址 / 字典字段 /
/// RVM 实测），所以先把现状钉住——真去改判词的时候这条会红，逼着连同证据一起更新。
#[test]
fn the_n2_class28_fix_is_closed_and_spine_is_still_evidence_pending() {
    // 缺陷 5：GTGEOM 的 IPCOMP|IHCOMP 分支**硬编码** NOZZ(1062572)、ELCONN(209154074)
    // 走复合几何，分类表却判它们无三维表示，两边直接冲突。
    for noun in ["NOZZ", "ELCONN"] {
        assert!(
            matches!(classify(noun), Category::Composite(_)),
            "{noun} 必须走 Core class 28 复合成员聚合路径"
        );
    }
    // 缺陷 6：SPINE 带 IPFCOM 家族位却被当作属主剖面数据，
    // 而 out/ams1112/report.json 的 orphans 里 SPINE 有 14 个没人消费。
    assert_eq!(
        classify("SPINE"),
        Category::ProfileData,
        "SPINE 判词已改，把 N2 的取证结论一起写进来"
    );
}

/// 会被属主消费剪掉的 noun，永远不能同时是模型单元。
///
/// 全量遍历在 `pipeline::push_members` 里把「属主 `consumes_members()` 且成员是
/// `ProfileData` 或负体」的成员整棵剪掉——那些元素连访问都不会被访问，自然不产出
/// 任何实体。增量那一侧的子树收集不认这条剪枝规则，它只按 `is_model_unit` 挑单元。
/// 两条路要不打架，唯一的条件就是**可被剪掉的 noun 永远不是模型单元**；一旦某个
/// noun 两头都占，增量就会为一件全量根本没产出的元素发 Remove / Regen，而两边
/// 各自的账还都是平的。
///
/// 这条今天成立，是 `is_positive_solid()` 与 `is_negative()` 互斥的副产品，
/// **没有任何东西钉住它**：给某个剖面族补一条正体路径，或者照 `SBFI` 的样子在
/// 目录件之外再开一个「例外也算单元」的口子，都会在不报错的情况下让它失效。
///
/// 这已经是「同一个判据写两份」的第三例了。前两例——`SBFI` 不在增量的单元名单里、
/// 短 `DESP` 的门只写在管线守卫里——都是先静默漏了件、再靠真库门的数字对不上
/// 才被逮住的。这一条趁还没出事先钉上。
#[test]
fn a_noun_the_traversal_can_prune_is_never_a_model_unit() {
    let mut pruned = 0usize;
    for row in matrix().nouns {
        let category = classify(&row.noun_name);
        if !(matches!(category, Category::ProfileData) || category.is_negative()) {
            continue;
        }
        pruned += 1;
        assert!(
            !is_model_unit(&row.noun_name),
            "{} 既会被属主消费剪掉、又被判成模型单元：\
             全量不会为它产出实体，增量却会为它发 Remove",
            row.noun_name
        );
    }
    // 字典里一个可剪的 noun 都没匹配上，说明分类表或字典换了形状，
    // 这条断言就成了一句空话——空转的回归钉比没有更坏。
    assert!(
        pruned > 0,
        "字典里没有任何可被消费剪掉的 noun，这条钉子失效了"
    );
}

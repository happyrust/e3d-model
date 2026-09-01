//! 导出 `classify()` 对全库 noun 的判定，供 NOUN 覆盖审计消费。
//!
//! 存在的理由：覆盖审计要回答「哪些 noun 没被考虑到」，判据必须是 `category.rs`
//! **本身**。此前审计脚本用的是从 `category.rs` 手抄进 Python 的影子表——分类表一改
//! 它不会报错，只会悄悄给出过期答案，而「用一张会过期的表去审有没有漏」是自相矛盾的。
//!
//! 输入 `data/noun-family-matrix.json`：全部 noun 的 dabacon 字典家族位，字段号由
//! `scripts/icom_field_ids.py` 从 core.dll 的 22 个 `I*COM` 谓词反编译解出。
//! `cascadeFamilies` 是 `GTGEOM`(0x10341d2e) 谓词级联里真正出现的 11 个家族——
//! 命中即旧路会给这个 noun 造几何段，这就是覆盖率的分母。
//!
//! ```powershell
//! cargo run --bin dump_categories -- data/noun-family-matrix.json out/noun-categories.json
//! ```

use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;

use e3d_model::category::{Category, classify, decision};

#[derive(serde::Deserialize)]
struct Matrix {
    #[serde(rename = "dictionaryVersion")]
    dictionary_version: String,
    #[serde(rename = "cascadeFamilies")]
    cascade_families: Vec<String>,
    nouns: Vec<MatrixRow>,
}

#[derive(serde::Deserialize)]
struct MatrixRow {
    noun_name: String,
    noun_hash: i64,
    families: Vec<String>,
    primitive: bool,
    geomset: bool,
    extrusion: bool,
    positive_equivalent: String,
}

#[derive(serde::Serialize)]
struct Verdict {
    noun: String,
    noun_hash: i64,
    category: String,
    disposition: String,
    evidence: String,
    /// 命中 `GTGEOM` 级联 = core.dll 旧路会给它造几何段。
    in_cascade: bool,
    families: Vec<String>,
    builds_geometry: bool,
    is_negative: bool,
    consumes_members: bool,
    primitive: bool,
    geomset: bool,
    extrusion: bool,
    positive_equivalent: String,
    /// 正体等价的类别——负体建得出、正体建不出（或反过来）是同一个形状有两套实现的信号。
    positive_equivalent_category: String,
}

#[derive(serde::Serialize)]
struct Dump {
    dictionary_version: String,
    cascade_families: Vec<String>,
    totals: Totals,
    /// 级联内按类别分账；`Unknown` 那一格就是「分类表欠账」的确切数目。
    cascade_by_category: BTreeMap<String, Vec<String>>,
    verdicts: Vec<Verdict>,
}

#[derive(serde::Serialize)]
struct Totals {
    nouns: usize,
    in_cascade: usize,
    cascade_builds: usize,
    cascade_ledgered: usize,
    cascade_unknown: usize,
}

/// 稳定的类别名。不用 `Debug`：那是给人看的，改一次枚举名就会让审计基线整体漂移，
/// 而漂移的表现是 diff 里几百行噪声，真正的覆盖变化淹在里面。
fn category_name(category: Category) -> String {
    match category {
        Category::List => "List".into(),
        Category::LoopExtrusion => "LoopExtrusion".into(),
        Category::NegExtrusion => "NegExtrusion".into(),
        Category::Revolution => "Revolution".into(),
        Category::NegRevolution => "NegRevolution".into(),
        Category::Polyhedron => "Polyhedron".into(),
        Category::NegPolyhedron => "NegPolyhedron".into(),
        Category::Primitive(prim) => format!("Primitive({prim:?})"),
        Category::NegPrimitive(prim) => format!("NegPrimitive({prim:?})"),
        Category::NegUnimplemented(_) => "NegUnimplemented".into(),
        Category::GeometrySet => "GeometrySet".into(),
        Category::NegGeometrySet => "NegGeometrySet".into(),
        Category::Catalog => "Catalog".into(),
        Category::RouteContainer => "RouteContainer".into(),
        Category::RouteMember => "RouteMember".into(),
        Category::PrimitiveContainer => "PrimitiveContainer".into(),
        Category::Composite(family) => format!("Composite({family:?})"),
        Category::ProfileData => "ProfileData".into(),
        Category::NonGraphic => "NonGraphic".into(),
        Category::Unsupported(_) => "Unsupported".into(),
        Category::Unknown => "Unknown".into(),
    }
}

fn main() -> anyhow::Result<()> {
    let mut args = std::env::args().skip(1);
    let input = PathBuf::from(
        args.next()
            .unwrap_or_else(|| "data/noun-family-matrix.json".into()),
    );
    let output = PathBuf::from(
        args.next()
            .unwrap_or_else(|| "out/noun-categories.json".into()),
    );

    let matrix: Matrix = serde_json::from_slice(&std::fs::read(&input)?)?;
    let cascade: BTreeSet<&str> = matrix.cascade_families.iter().map(String::as_str).collect();

    let mut verdicts = Vec::with_capacity(matrix.nouns.len());
    let mut cascade_by_category: BTreeMap<String, Vec<String>> = BTreeMap::new();
    let mut totals = Totals {
        nouns: matrix.nouns.len(),
        in_cascade: 0,
        cascade_builds: 0,
        cascade_ledgered: 0,
        cascade_unknown: 0,
    };

    for row in &matrix.nouns {
        let noun_decision = decision(&row.noun_name);
        let category = noun_decision.category;
        let in_cascade = row.families.iter().any(|f| cascade.contains(f.as_str()));
        // 有依据地不建仍算「记账不建」：账上有名有姓，与分类表欠账是两回事。
        // 判据只有 `Category::builds_geometry()` 一处，这里不再自己拼一遍——
        // 拼一遍就等于又造了一张会跟本体走散的影子表。
        let builds = category.builds_geometry();

        if in_cascade {
            totals.in_cascade += 1;
            let name = category_name(category);
            if builds {
                totals.cascade_builds += 1;
            } else if name == "Unknown" {
                totals.cascade_unknown += 1;
            } else {
                totals.cascade_ledgered += 1;
            }
            cascade_by_category
                .entry(name)
                .or_default()
                .push(row.noun_name.clone());
        }

        verdicts.push(Verdict {
            noun: row.noun_name.clone(),
            noun_hash: row.noun_hash,
            category: category_name(category),
            disposition: format!("{:?}", noun_decision.disposition),
            evidence: noun_decision.evidence.to_owned(),
            in_cascade,
            families: row.families.clone(),
            builds_geometry: builds,
            is_negative: category.is_negative(),
            consumes_members: category.consumes_members(),
            primitive: row.primitive,
            geomset: row.geomset,
            extrusion: row.extrusion,
            positive_equivalent: row.positive_equivalent.clone(),
            positive_equivalent_category: if row.positive_equivalent.is_empty() {
                String::new()
            } else {
                category_name(classify(&row.positive_equivalent))
            },
        });
    }

    for names in cascade_by_category.values_mut() {
        names.sort();
    }

    eprintln!(
        "字典 {} — noun {} 个；命中 GTGEOM 级联 {}：会建 {} / 记账不建 {} / 分类表欠账 {}",
        matrix.dictionary_version,
        totals.nouns,
        totals.in_cascade,
        totals.cascade_builds,
        totals.cascade_ledgered,
        totals.cascade_unknown,
    );

    let dump = Dump {
        dictionary_version: matrix.dictionary_version,
        cascade_families: matrix.cascade_families,
        totals,
        cascade_by_category,
        verdicts,
    };
    if let Some(parent) = output.parent() {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(&output, serde_json::to_vec_pretty(&dump)?)?;
    eprintln!("写出 {}", output.display());
    Ok(())
}

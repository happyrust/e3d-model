//! SREV 普查：数目录库里 SREV / NSRE 的件数，抽样打出参数形状与成员树。
//!
//! G0-1b（目录回转体臂）动手前的语料确认：QSSREV（`0x103B81CC`）读的是
//! PANG / 两根轴（PAAX、PBAX）/ PX PY PZ，轮廓与 SEXT 同一个组装器
//! （SLOO → SVER）。这支探针回答「活库里的 SREV 真的长这样吗」，
//! 以及 NSRE 在语料里有没有实例（没有就照 G0-1c 的纪律不动它）。
//!
//! ```text
//! cargo run --example srev_probe -- --attlib <attlib.dat> --db <ams5052_0001> \
//!     --template <dir|catvir.dat> [--db-type CATA] [--samples N]
//! ```

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::Arc;

use anyhow::{Context, bail};
use e3d_io::db_element::{DbElement, DbElementError, DbFilePin, DbSet, template_file_for};
use e3d_io::engine::{ReadOnlyEngine, ScanTier};

fn show_attr(el: &DbElement, name: &str) -> String {
    match el.attribute(name) {
        Ok(Some(value)) => format!("{value:?}"),
        Ok(None) => "（未设）".into(),
        Err(DbElementError::UnknownAttribute { .. }) => "（字典无此属性）".into(),
        Err(error) => format!("（读取失败：{error}）"),
    }
}

fn main() -> anyhow::Result<()> {
    let mut attlib: Option<PathBuf> = None;
    let mut db_file: Option<PathBuf> = None;
    let mut template: Option<PathBuf> = None;
    let mut db_type = "CATA".to_string();
    let mut samples = 3usize;

    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--attlib" => attlib = Some(PathBuf::from(value()?)),
            "--db" => db_file = Some(PathBuf::from(value()?)),
            "--template" => template = Some(PathBuf::from(value()?)),
            "--db-type" => db_type = value()?,
            "--samples" => samples = value()?.parse().context("--samples 要个数")?,
            other => bail!("不认识的参数 {other}"),
        }
    }
    let attlib = attlib.context("缺 --attlib")?;
    let db_file = db_file.context("缺 --db")?;
    let template = template.context("缺 --template")?;
    let template = if template.is_dir() {
        template_file_for(&template, &db_type)?
    } else {
        template
    };

    let set = Arc::new(DbSet::with_attlib_file(&attlib)?);
    set.add_db(DbFilePin {
        file: db_file.clone(),
        template,
        db_type: Some(db_type),
        sesno: None,
    })?;

    let mut engine = ReadOnlyEngine::open(&db_file)
        .with_context(|| format!("开库扫 SREV {}", db_file.display()))?;

    let mut srev_total = 0usize;
    let mut nsre_total = 0usize;
    let mut owner_nouns: BTreeMap<String, usize> = BTreeMap::new();
    let mut member_nouns: BTreeMap<String, usize> = BTreeMap::new();
    let mut shown = 0usize;

    for scanned in engine.scan_elements(ScanTier::Header)? {
        let refno = scanned?.refno;
        let el = set.element(refno);
        let noun = el.element_type()?;
        if noun == "NSRE" {
            nsre_total += 1;
            continue;
        }
        if noun != "SREV" {
            continue;
        }
        srev_total += 1;
        let owner_noun = match el.owner()? {
            Some(owner) => owner.element_type().unwrap_or_else(|_| "?".into()),
            None => "（无属主）".into(),
        };
        *owner_nouns.entry(owner_noun).or_insert(0) += 1;
        for member in el.members()? {
            *member_nouns
                .entry(member.element_type().unwrap_or_else(|_| "?".into()))
                .or_insert(0) += 1;
        }
        if shown < samples {
            shown += 1;
            println!("── SREV {refno} ──");
            for name in [
                "PANG", "PAAX", "PBAX", "PX", "PY", "PZ", "LEVE", "OBST", "TUFL", "CLFL",
            ] {
                println!("  {name} = {}", show_attr(&el, name));
            }
            for member in el.members()? {
                let member_noun = member.element_type()?;
                println!("  成员 {member_noun} {}", member.refno());
                if member_noun == "SLOO" {
                    for (index, vertex) in member.members()?.enumerate() {
                        if index >= 4 {
                            println!("    …（其余顶点略）");
                            break;
                        }
                        println!(
                            "    {} {}: PX={} PY={} PRAD={}",
                            vertex.element_type()?,
                            vertex.refno(),
                            show_attr(&vertex, "PX"),
                            show_attr(&vertex, "PY"),
                            show_attr(&vertex, "PRAD"),
                        );
                    }
                }
            }
        }
    }

    println!("SREV 总数: {srev_total}，NSRE 总数: {nsre_total}");
    println!("SREV 属主 noun 分布: {owner_nouns:?}");
    println!("SREV 成员 noun 分布: {member_nouns:?}");
    Ok(())
}

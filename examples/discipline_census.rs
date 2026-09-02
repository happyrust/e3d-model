//! 按学科普查项目目录里的 DESI 库：哪几本库装着 HVAC（通风）与 CWAY（电缆桥架）
//! 子树，子树里各 noun 有多少件。只读索引头（`ScanTier::Header`），不解析记录体，
//! 1.2GB 的库也能几秒扫完；库文件用 `DirectoryDbResolver` 认，与 `gen_ams --project-dir`
//! 同一套判据。
//!
//! ```text
//! cargo run --release --example discipline_census -- \
//!     --template <模板目录> --project-dir <库目录> [--project-dir ...] \
//!     [--root-noun HVAC --root-noun CWAY] [--min 1]
//! ```
//!
//! 输出按库列出每个学科根（默认 HVAC / CWAY）名下的元素数与 noun 分布，
//! 供挑选 `gen_ams` 的对拍语料。

use std::collections::{BTreeMap, HashMap};
use std::path::PathBuf;

use anyhow::{Context, bail};
use e3d_attlib::db1_dehash;
use e3d_io::engine::{ReadOnlyEngine, ScanTier};
use e3d_io::refno::RefNo;
use e3d_model::db_discovery::DirectoryDbResolver;

fn main() -> anyhow::Result<()> {
    let mut template: Option<PathBuf> = None;
    let mut project_dirs: Vec<PathBuf> = Vec::new();
    let mut root_nouns: Vec<String> = Vec::new();
    let mut min = 1usize;
    let mut samples = 0usize;

    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--template" => template = Some(PathBuf::from(value()?)),
            "--project-dir" => project_dirs.push(PathBuf::from(value()?)),
            "--root-noun" => root_nouns.push(value()?.to_uppercase()),
            "--min" => min = value()?.parse()?,
            "--samples" => samples = value()?.parse()?,
            other => bail!("不认识的参数 {other}"),
        }
    }
    let template = template.context("缺 --template")?;
    if project_dirs.is_empty() {
        bail!("至少给一个 --project-dir");
    }
    if root_nouns.is_empty() {
        root_nouns = vec!["HVAC".into(), "CWAY".into()];
    }

    let resolver = DirectoryDbResolver::scan(&project_dirs, &template)?;
    println!("{}", resolver.summary());

    let mut desi: Vec<_> = resolver
        .discovered()
        .iter()
        .filter(|db| db.db_type == "DESI" && db.ext_no <= 1)
        .collect();
    desi.sort_by_key(|db| db.dbno);
    println!("DESI 主文件 {} 本\n", desi.len());

    let mut grand: BTreeMap<String, BTreeMap<String, usize>> = BTreeMap::new();

    for db in desi {
        let mut engine = match ReadOnlyEngine::open(&db.file) {
            Ok(engine) => engine,
            Err(error) => {
                println!("db {} {}: 开库失败 {error}", db.dbno, db.file.display());
                continue;
            }
        };
        let mut owner_of: HashMap<RefNo, RefNo> = HashMap::new();
        let mut noun_of: HashMap<RefNo, String> = HashMap::new();
        let mut order: Vec<RefNo> = Vec::new();
        let mut bad = 0usize;
        for scanned in engine.scan_elements(ScanTier::Header)? {
            let scanned = match scanned {
                Ok(scanned) => scanned,
                Err(_) => {
                    bad += 1;
                    continue;
                }
            };
            owner_of.insert(scanned.refno, scanned.owner);
            noun_of.insert(
                scanned.refno,
                db1_dehash(scanned.noun_hash).unwrap_or_else(|| format!("#{}", scanned.noun_hash)),
            );
            order.push(scanned.refno);
        }

        // 每个元素的学科根：沿 owner 链上溯，命中 root_nouns 之一即停。
        let mut discipline: HashMap<RefNo, Option<String>> = HashMap::with_capacity(order.len());
        for &start in &order {
            if discipline.contains_key(&start) {
                continue;
            }
            let mut path = Vec::new();
            let mut cursor = start;
            let found = loop {
                if let Some(known) = discipline.get(&cursor) {
                    break known.clone();
                }
                path.push(cursor);
                let noun = noun_of.get(&cursor).map(String::as_str).unwrap_or("");
                if root_nouns.iter().any(|r| r == noun) {
                    break Some(noun.to_string());
                }
                let Some(&owner) = owner_of.get(&cursor) else {
                    break None;
                };
                if owner == cursor || !owner.is_valid() || !owner_of.contains_key(&owner) {
                    break None;
                }
                if path.len() > 256 {
                    break None;
                }
                cursor = owner;
            };
            for node in path {
                discipline.insert(node, found.clone());
            }
        }

        let mut per_root: BTreeMap<String, BTreeMap<String, usize>> = BTreeMap::new();
        let mut sample_refnos: BTreeMap<(String, String), Vec<RefNo>> = BTreeMap::new();
        for &refno in &order {
            if let Some(Some(root)) = discipline.get(&refno) {
                *per_root
                    .entry(root.clone())
                    .or_default()
                    .entry(noun_of[&refno].clone())
                    .or_insert(0) += 1;
                if samples > 0 {
                    let bucket = sample_refnos
                        .entry((root.clone(), noun_of[&refno].clone()))
                        .or_default();
                    if bucket.len() < samples {
                        bucket.push(refno);
                    }
                }
            }
        }
        let hits: usize = per_root.values().map(|m| m.values().sum::<usize>()).sum();
        if hits < min {
            continue;
        }
        println!(
            "== db {} {} — 索引 {} 元素（坏记录 {bad}）",
            db.dbno,
            db.file.display(),
            order.len()
        );
        for (root, nouns) in &per_root {
            let total: usize = nouns.values().sum();
            let mut rows: Vec<(&String, &usize)> = nouns.iter().collect();
            rows.sort_by(|a, b| b.1.cmp(a.1).then(a.0.cmp(b.0)));
            let line = rows
                .iter()
                .map(|(noun, count)| format!("{noun} {count}"))
                .collect::<Vec<_>>()
                .join(", ");
            println!("   {root} 子树 {total} 件: {line}");
            if samples > 0 {
                for (noun, _) in &rows {
                    if let Some(refnos) = sample_refnos.get(&(root.clone(), (*noun).clone())) {
                        let list = refnos
                            .iter()
                            .map(ToString::to_string)
                            .collect::<Vec<_>>()
                            .join(" ");
                        println!("      {noun:<8} {list}");
                    }
                }
            }
            for (noun, count) in nouns {
                *grand
                    .entry(root.clone())
                    .or_default()
                    .entry(noun.clone())
                    .or_insert(0) += count;
            }
        }
    }

    println!("\n=== 全目录合计 ===");
    for (root, nouns) in &grand {
        let total: usize = nouns.values().sum();
        let mut rows: Vec<(&String, &usize)> = nouns.iter().collect();
        rows.sort_by(|a, b| b.1.cmp(a.1).then(a.0.cmp(b.0)));
        println!("{root} 子树 {total} 件:");
        for (noun, count) in rows {
            println!("   {noun:<8} {count:>7}");
        }
    }
    Ok(())
}

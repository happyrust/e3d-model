//! 目录库负体普查：CRCATI（Core3D `0x10345ACC`）认的 36 个图元 noun，在每个 CATA 库里
//! 各有多少、刀挂在哪种正体名下、正体又挂在谁名下。
//!
//! 存在的理由：`CSG_TreeBuilderCat::getCSGTree`（`0x1072F5D0`）对**每一个**有成员的正体都开
//! `Combination(3)` 差集、`addNegatives`（`0x1072F480`）逐成员送进 CRCATI——不分正体 noun。
//! 我们这边哪条正体臂没收刀，那种属主名下的刀就静默没了，报告上连一行 note 都没有；
//! 这张「刀 → 属主」表就是那本账的分母（2026-09-06：LPYR 名下 3683、LSNO 794、SSLC 413……
//! 八种属主合计约 5200 只，见 recheck-4 §十一）。
//!
//! ```text
//! cargo run --release --example catalogue_negative_census -- \
//!     --attlib <attlib.dat> --template <catvir.dat> <db-file-or-dir>...
//! ```
//!
//! 给目录就扫里面每个长得像 db 文件的名字，按文件头的库类型只留 CATA；给文件就只看它。

use std::collections::{BTreeMap, HashMap};
use std::path::PathBuf;
use std::sync::Arc;

use anyhow::Context;
use e3d_io::db_element::{DbFilePin, DbSet};
use e3d_io::engine::{ReadOnlyEngine, ScanTier};
use e3d_io::refno::RefNo;

/// CRCATI switch 的 36 条臂（`tools/noun_decode.py` 反哈希），正负成对。
const CRCATI: &[&str] = &[
    "SBOX", "NSBO", "SREC", "SANN", "SPRO", "BOXI", "NBXI", "TUBE", "NTUB", "SCON", "NSCO", "SSPH",
    "NSSP", "SDIS", "LCYL", "NLCY", "SCYL", "NSCY", "LSNO", "NLSN", "LINE", "SLINE", "SCTO",
    "NSCT", "SRTO", "NSRT", "SDSH", "NSDS", "SSLC", "NSSL", "LPYR", "NLPY", "SEXT", "NSEX", "SREV",
    "NSRE",
];

fn is_negative(noun: &str) -> bool {
    noun.starts_with('N') && CRCATI.contains(&noun)
}

fn main() -> anyhow::Result<()> {
    let mut attlib: Option<PathBuf> = None;
    let mut template: Option<PathBuf> = None;
    let mut inputs: Vec<PathBuf> = Vec::new();
    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        match flag.as_str() {
            "--attlib" => attlib = Some(PathBuf::from(it.next().context("--attlib 缺值")?)),
            "--template" => template = Some(PathBuf::from(it.next().context("--template 缺值")?)),
            other => inputs.push(PathBuf::from(other)),
        }
    }
    let attlib = attlib.context("缺 --attlib")?;
    let template = template.context("缺 --template（CATA 库的 catvir.dat）")?;

    let mut files: Vec<PathBuf> = Vec::new();
    for input in inputs {
        if input.is_dir() {
            for entry in std::fs::read_dir(&input)? {
                let path = entry?.path();
                let Some(name) = path.file_name().map(|n| n.to_string_lossy().to_string()) else {
                    continue;
                };
                // `ams7997_0001` 这一形：三个小写字母 + 库号 + `_` + 分卷号。
                let looks_like_db = path.is_file()
                    && name.len() >= 8
                    && name[..3].chars().all(|c| c.is_ascii_lowercase())
                    && name[3..].chars().all(|c| c.is_ascii_digit() || c == '_');
                if looks_like_db {
                    files.push(path);
                }
            }
        } else {
            files.push(input);
        }
    }
    files.sort();

    // (属主 noun, 子 noun) → 件数，只留至少一头是 CRCATI noun 的边。
    let mut edges: BTreeMap<(String, String), usize> = BTreeMap::new();
    let mut noun_totals: BTreeMap<String, usize> = BTreeMap::new();
    let mut cata_dbs = 0usize;

    for file in &files {
        let engine = match ReadOnlyEngine::open(file) {
            Ok(engine) => engine,
            Err(error) => {
                eprintln!("跳过 {}: {error}", file.display());
                continue;
            }
        };
        let db_type = engine.descriptor().db_type_name().unwrap_or_default();
        if db_type != "CATA" {
            continue;
        }
        drop(engine);
        cata_dbs += 1;

        let set = Arc::new(DbSet::with_attlib_file(&attlib)?);
        let dbno = set.add_db(DbFilePin {
            file: file.clone(),
            template: template.clone(),
            db_type: Some("CATA".to_string()),
            sesno: None,
        })?;
        let mut engine = match set.pinned_sesno(dbno) {
            Some(sesno) => ReadOnlyEngine::open_at(file, sesno),
            None => ReadOnlyEngine::open(file),
        }?;
        let mut owner_of: HashMap<RefNo, RefNo> = HashMap::new();
        let mut order: Vec<RefNo> = Vec::new();
        for scanned in engine.scan_elements(ScanTier::Header)? {
            let scanned = scanned?;
            owner_of.insert(scanned.refno, scanned.owner);
            order.push(scanned.refno);
        }
        let mut noun_of: HashMap<RefNo, String> = HashMap::with_capacity(order.len());
        for &refno in &order {
            let noun = set
                .element(refno)
                .element_type()
                .unwrap_or_else(|error| format!("<{error}>"));
            noun_of.insert(refno, noun);
        }
        let mut negatives_here = 0usize;
        for &refno in &order {
            let child = noun_of[&refno].as_str();
            let parent = owner_of
                .get(&refno)
                .and_then(|owner| noun_of.get(owner))
                .map(String::as_str)
                .unwrap_or("<无 owner>");
            let child_is_crcati = CRCATI.contains(&child);
            if child_is_crcati {
                *noun_totals.entry(child.to_string()).or_insert(0) += 1;
            }
            if is_negative(child) {
                negatives_here += 1;
            }
            if child_is_crcati || CRCATI.contains(&parent) {
                *edges
                    .entry((parent.to_string(), child.to_string()))
                    .or_insert(0) += 1;
            }
        }
        let label = file
            .file_name()
            .map(|name| name.to_string_lossy().to_string())
            .unwrap_or_default();
        println!(
            "{label:<20} elements={:<8} crcati-negatives={negatives_here}",
            order.len()
        );
    }

    println!("\nCATA 库数 = {cata_dbs}");
    println!("\n=== CRCATI noun 总数（全部目录库）===");
    for noun in CRCATI {
        println!(
            "{noun:<8} {:>8}",
            noun_totals.get(*noun).copied().unwrap_or(0)
        );
    }

    let print_edges = |title: &str, keep: &dyn Fn(&str, &str) -> bool, arrow: &str| {
        println!("\n=== {title} ===");
        let mut rows: Vec<(&(String, String), &usize)> = edges
            .iter()
            .filter(|((parent, child), _)| keep(parent, child))
            .collect();
        rows.sort_by(|a, b| b.1.cmp(a.1));
        for ((parent, child), count) in rows {
            if arrow == "under" {
                println!("{child:<8} under {parent:<10} {count:>8}");
            } else {
                println!("{parent:<8} → {child:<10} {count:>8}");
            }
        }
    };
    print_edges(
        "负体 noun → 属主 noun（谁名下挂着刀）",
        &|_, child| is_negative(child),
        "under",
    );
    print_edges(
        "正体 noun ← 属主 noun（正体挂在谁名下）",
        &|_, child| CRCATI.contains(&child) && !is_negative(child),
        "under",
    );
    print_edges(
        "正体名下的非负体成员（正体 → 子）",
        &|parent, child| CRCATI.contains(&parent) && !is_negative(parent) && !is_negative(child),
        "→",
    );
    Ok(())
}

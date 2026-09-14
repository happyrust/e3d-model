//! `PTCA.PTCD`（P 点方向程序）的存储形态普查。
//!
//! [`e3d_io::record::direction_spec`] 只认 Core `DORTXT` 那一族 opcode 21/22/23
//! （一条轴，后面跟若干「转角 → 下一条轴」）。SLYK 的软管目录件
//! `/CIR-FILEXFILL-HOUSE-200` 上出现了 opcode 52，解不出来就退回 `DIR` 的模板
//! 缺省值 `[0,1,0]`，整只软管被摆到 +Y 上（RVM 对拍差 820 mm）。
//!
//! 要不要为它新开一支，先得知道这种形态有多少、长什么样、稳不稳定。本例只数，
//! 不解释：按 opcode 分组，给出计数、词长分布、末两词与样例原始词。
//!
//! ```text
//! cargo run --release --example ptcd_census -- --attlib <attlib.dat> \
//!     --template <desvir/catvir.dat> --db <acw5001_0001> [--db ...] [--sample 3]
//! ```

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::Arc;

use anyhow::{Context, bail};
use e3d_io::db_element::{DbElement, DbFilePin, DbSet};
use e3d_io::engine::{ReadOnlyEngine, ScanTier};
use e3d_io::record::descriptor::DescriptorValue;
use e3d_io::refno::RefNo;

/// 一种 opcode 下攒到的证据。
#[derive(Default)]
struct Bucket {
    count: usize,
    /// 词数 → 出现次数。同一 opcode 下长度散开说明它带可变长的表达式。
    lengths: BTreeMap<usize, usize>,
    /// 末两词，用来看终止符是不是 61。
    tails: BTreeMap<String, usize>,
    /// opcode 后面紧跟的两词，用来看有没有别的前缀。
    heads: BTreeMap<String, usize>,
    samples: Vec<(String, Vec<u32>)>,
}

fn main() -> anyhow::Result<()> {
    let mut attlib: Option<PathBuf> = None;
    let mut template: Option<PathBuf> = None;
    let mut db_type = "CATA".to_string();
    let mut dbs: Vec<PathBuf> = Vec::new();
    let mut sample = 2usize;

    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--attlib" => attlib = Some(PathBuf::from(value()?)),
            "--template" => template = Some(PathBuf::from(value()?)),
            "--db-type" => db_type = value()?,
            "--db" => dbs.push(PathBuf::from(value()?)),
            "--sample" => sample = value()?.parse()?,
            other => bail!("不认识的参数 {other}"),
        }
    }
    let attlib = attlib.context("缺 --attlib")?;
    let template = template.context("缺 --template")?;
    if dbs.is_empty() {
        bail!("至少给一个 --db");
    }

    let set = Arc::new(DbSet::with_attlib_file(&attlib)?);
    let mut buckets: BTreeMap<u32, Bucket> = BTreeMap::new();
    let mut without_ptcd = 0usize;
    let mut total = 0usize;

    for db_file in &dbs {
        let dbno = set.add_db(DbFilePin {
            file: db_file.clone(),
            template: template.clone(),
            db_type: Some(db_type.clone()),
            sesno: None,
        })?;
        let mut engine = match set.pinned_sesno(dbno) {
            Some(sesno) => ReadOnlyEngine::open_at(db_file, sesno),
            None => ReadOnlyEngine::open(db_file),
        }?;
        let mut refnos: Vec<RefNo> = Vec::new();
        for scanned in engine.scan_elements(ScanTier::Header)? {
            refnos.push(scanned?.refno);
        }
        println!(
            "{} dbno={dbno} 索引 {} 个元素",
            db_file.display(),
            refnos.len()
        );

        for refno in refnos {
            let element = set.element(refno);
            if element.element_type().unwrap_or_default() != "PTCA" {
                continue;
            }
            total += 1;
            let words = match element.attribute("PTCD") {
                Ok(Some(DescriptorValue::RawWords(words) | DescriptorValue::WordArray(words))) => {
                    words
                }
                Ok(Some(DescriptorValue::IntArray(words))) => {
                    words.into_iter().map(|word| word as u32).collect()
                }
                _ => {
                    without_ptcd += 1;
                    continue;
                }
            };
            // 头三词是「声明长度 / 5|6 / 2」，第四词才是 opcode；短于四词的另记为 0。
            let opcode = words.get(3).copied().unwrap_or(0);
            let bucket = buckets.entry(opcode).or_default();
            bucket.count += 1;
            *bucket.lengths.entry(words.len()).or_default() += 1;
            *bucket.tails.entry(join(words.iter().rev().take(2))).or_default() += 1;
            *bucket.heads.entry(join(words.iter().skip(4).take(2))).or_default() += 1;
            if bucket.samples.len() < sample {
                bucket.samples.push((describe(&element), words));
            }
        }
    }

    println!("\nPTCA 共 {total} 只，其中 {without_ptcd} 只没有 PTCD");
    for (opcode, bucket) in &buckets {
        println!(
            "\nopcode {opcode}: {} 只\n  词长 {:?}\n  opcode 后两词 {:?}\n  末两词 {:?}",
            bucket.count, bucket.lengths, bucket.heads, bucket.tails
        );
        for (who, words) in &bucket.samples {
            println!("  {who}\n    {words:?}");
        }
    }
    Ok(())
}

fn join<'a>(words: impl Iterator<Item = &'a u32>) -> String {
    words.map(|word| word.to_string()).collect::<Vec<_>>().join(" ")
}

fn describe(element: &DbElement) -> String {
    let owner = element
        .owner()
        .ok()
        .flatten()
        .and_then(|owner| owner.stored_name().ok().flatten())
        .unwrap_or_default();
    format!(
        "{} NUMB={:?} owner={owner}",
        element.refno(),
        element.attribute("NUMB").ok().flatten()
    )
}

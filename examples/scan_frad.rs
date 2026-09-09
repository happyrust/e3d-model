//! 扫一个库的全部元素，报 FRAD 的正 / 零 / 负直方图 + 每个负 FRAD 顶点（含最近的
//! 命名祖先与其 noun）。用来找「真实含负 FRAD 的构件」做负倒角（`mthArcRadius`）对拍。
//!
//! FRAD 落在轮廓顶点（PLOO/LOOP 的 PAVE/VERT，DESI 库）或截面点（SPRO/SREC，目录库）上，
//! 所以 DESI 库与目录库都要扫。
//!
//! ```text
//! cargo run --release --example scan_frad -- --attlib <attlib.dat> --db <ams1112_0001> \
//!     [--db-type DESI]
//! ```

use std::path::PathBuf;
use std::sync::Arc;

use anyhow::{Context, bail};
use e3d_io::db_element::{DbElement, DbFilePin, DbSet, template_file_for};
use e3d_io::engine::{ReadOnlyEngine, ScanTier};

/// 从一个顶点往上找最近的「有名字」祖先，返回 `(名字, 那个祖先的 noun, 爬了几级)`。
fn named_ancestor(el: &DbElement) -> (String, String, usize) {
    let mut cur = el.clone();
    for depth in 0..12 {
        if let Ok(Some(name)) = cur.stored_name() {
            let noun = cur.element_type().unwrap_or_default();
            return (name, noun, depth);
        }
        match cur.owner() {
            Ok(Some(owner)) => cur = owner,
            _ => break,
        }
    }
    ("<无名>".to_string(), el.element_type().unwrap_or_default(), 0)
}

fn main() -> anyhow::Result<()> {
    let mut attlib: Option<PathBuf> = None;
    let mut db_file: Option<PathBuf> = None;
    let mut template: Option<PathBuf> = None;
    let mut db_type = "DESI".to_string();

    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--attlib" => attlib = Some(PathBuf::from(value()?)),
            "--db" => db_file = Some(PathBuf::from(value()?)),
            "--template" => template = Some(PathBuf::from(value()?)),
            "--db-type" => db_type = value()?,
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

    let mut engine =
        ReadOnlyEngine::open(&db_file).with_context(|| format!("开库 {}", db_file.display()))?;

    let (mut total, mut with_frad, mut pos, mut zero, mut neg) = (0u64, 0u64, 0u64, 0u64, 0u64);
    let mut hits: Vec<(String, String, String, usize, f64)> = Vec::new();
    for scanned in engine.scan_elements(ScanTier::Header)? {
        let refno = scanned?.refno;
        total += 1;
        let el = set.element(refno);
        let frad = match el.get_double("FRAD") {
            Ok(Some(v)) => v,
            _ => continue,
        };
        with_frad += 1;
        if frad <= -0.1 {
            neg += 1;
            let noun = el.element_type().unwrap_or_default();
            let (name, anc_noun, depth) = named_ancestor(&el);
            if hits.len() < 500 {
                hits.push((format!("{} {noun}", el.refno()), name, anc_noun, depth, frad));
            }
        } else if frad >= 0.1 {
            pos += 1;
        } else {
            zero += 1;
        }
    }

    println!(
        "db={}  elems={total}  有FRAD={with_frad}  ->  正{pos} / 近零{zero} / 负{neg}",
        db_file.display()
    );
    for (who, name, anc_noun, depth, frad) in &hits {
        println!("  负 FRAD {frad:>10.3}  {who}  <- {anc_noun} {name}（第 {depth} 级祖先）");
    }
    if neg == 0 {
        println!("  （本库无负 FRAD）");
    }
    Ok(())
}

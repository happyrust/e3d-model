//! 扫一个库的全部元素，报**带 ANGL 属性的元素**按 noun 分组的角度直方图，重点标出
//! `ANGL ≤ 0` 与 `ANGL > 360` 的构件（含最近命名祖先）。
//!
//! 用途：Core3D 对 CTOR / RTOR 建体时把 ANGL 原样当「终止角」传给 libgm
//! （`CSG_BasicCTO/RTO::getPrimGeom`，起始角恒 0），libgm 的
//! `d2_numberOfSegmentsForPartRev` 会把区间归一化到 `(0, 360]`——ANGL=−90 是
//! 0°→270° 的**逆时针**弧、ANGL=0 是**整圈**。本仓 `gen_circular_torus` /
//! `gen_rectangular_torus` 把负角按「顺时针扫 |ANGL|」画。两种口径只在
//! ANGL ∉ (0, 360] 时可分——这支扫描回答「真实语料里有没有这种角」。
//!
//! ```text
//! cargo run --release --example scan_torus_angl -- --attlib <attlib.dat> --db <xxx_0001> \
//!     [--db-type DESI]
//! ```

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::Arc;

use anyhow::{Context, bail};
use e3d_io::db_element::{DbElement, DbFilePin, DbSet, template_file_for};
use e3d_io::engine::{ReadOnlyEngine, ScanTier};

/// 从一个元素往上找最近的「有名字」祖先，返回 `(名字, 那个祖先的 noun, 爬了几级)`。
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
    (
        "<无名>".to_string(),
        el.element_type().unwrap_or_default(),
        0,
    )
}

#[derive(Default)]
struct Buckets {
    /// ANGL ∈ (0, 360]：两种口径一致的正常档。
    normal: u64,
    /// |ANGL| ≤ 1e-6：libgm 归一化成整圈，本仓 `check_valid` 拒绝。
    zero: u64,
    /// ANGL < 0：libgm 走 0→ANGL+360 的逆时针弧，本仓画顺时针 |ANGL|。
    negative: u64,
    /// ANGL > 360：libgm 减圈归一化。
    over: u64,
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

    let (mut total, mut with_angl) = (0u64, 0u64);
    let mut by_noun: BTreeMap<String, Buckets> = BTreeMap::new();
    let mut hits: Vec<(String, String, String, usize, f64)> = Vec::new();
    for scanned in engine.scan_elements(ScanTier::Header)? {
        let refno = scanned?.refno;
        total += 1;
        let el = set.element(refno);
        let angl = match el.get_double("ANGL") {
            Ok(Some(v)) => v,
            _ => continue,
        };
        with_angl += 1;
        let noun = el.element_type().unwrap_or_default();
        let b = by_noun.entry(noun.clone()).or_default();
        if angl.abs() <= 1e-6 {
            b.zero += 1;
        } else if angl < 0.0 {
            b.negative += 1;
        } else if angl > 360.0 {
            b.over += 1;
        } else {
            b.normal += 1;
            continue;
        }
        // 只有异常档才留明细
        if hits.len() < 500 {
            let (name, anc_noun, depth) = named_ancestor(&el);
            hits.push((
                format!("{} {noun}", el.refno()),
                name,
                anc_noun,
                depth,
                angl,
            ));
        }
    }

    let unusual: u64 = by_noun.values().map(|b| b.zero + b.negative + b.over).sum();
    println!(
        "db={}  elems={total}  有ANGL={with_angl}  异常(≤0或>360)={unusual}",
        db_file.display()
    );
    for (noun, b) in &by_noun {
        println!(
            "  {noun:<6} 正常(0,360]={:<6} 零={:<4} 负={:<4} 超360={}",
            b.normal, b.zero, b.negative, b.over
        );
    }
    for (who, name, anc_noun, depth, angl) in &hits {
        println!("  异常 ANGL {angl:>12.4}  {who}  <- {anc_noun} {name}（第 {depth} 级祖先）");
    }
    Ok(())
}

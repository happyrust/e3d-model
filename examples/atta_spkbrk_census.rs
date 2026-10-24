//! 路由成员的 `SPKBRK` 普查：按（容器属主 noun、成员 noun、目录 `SCOM.GTYP`、`SPKBRK`）分组计数，
//! `--verbose` 逐只列。
//!
//! E3D 按 ATTA 自己的 `SPKBRK` 定它断不断隐式管身（[`e3d_model::route::station_verdict`]）；这支探针就是
//! 把那条规则钉下来的普查：ams8000 梯架支撑（`GTYP=ANCI`）233 只真、`/M-OR-1RX-3081D` 与 `/M-OR-1RX-3AF0A`
//! 的 6 只没存；管道支撑附着点（`GTYP=ATTA/PCLA`）与风管链上的 MDS 支撑全没存；ams7997
//! `/Copy-of-NP-R322003-DAB` 是 `GTYP=ATTA` 里唯一置真的。STIF / BRCO / SPLR 的模板里没有这个属性，
//! 会列成 `err:`。
//!
//! ```text
//! cargo run --release --example atta_spkbrk_census -- --attlib <attlib.dat> --db <ams8000_0001> \
//!     --template <dir> --project-dir <dir> [--project-dir ...] [--nouns ATTA,STIF,BRCO] [--verbose]
//! ```

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use anyhow::{Context, bail};
use e3d_io::db_element::{DbElement, DbFilePin, DbSet, template_file_for};
use e3d_io::engine::{ReadOnlyEngine, ScanTier};
use e3d_io::refno::RefNo;
use e3d_model::db_discovery::DirectoryDbResolver;

fn main() -> anyhow::Result<()> {
    let mut attlib: Option<PathBuf> = None;
    let mut db_file: Option<PathBuf> = None;
    let mut template: Option<PathBuf> = None;
    let mut project_dirs: Vec<PathBuf> = Vec::new();
    let mut nouns: Vec<String> = vec!["ATTA".into(), "STIF".into(), "BRCO".into()];
    let mut verbose = false;
    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--attlib" => attlib = Some(PathBuf::from(value()?)),
            "--db" => db_file = Some(PathBuf::from(value()?)),
            "--template" => template = Some(PathBuf::from(value()?)),
            "--project-dir" => project_dirs.push(PathBuf::from(value()?)),
            "--nouns" => nouns = value()?.split(',').map(|s| s.trim().to_uppercase()).collect(),
            "--verbose" => verbose = true,
            other => bail!("不认识的参数 {other}"),
        }
    }
    let attlib = attlib.context("缺 --attlib")?;
    let db_file = db_file.context("缺 --db")?;
    let template = template.context("缺 --template")?;
    let template_dir = if template.is_dir() {
        template.clone()
    } else {
        template.parent().unwrap_or(Path::new(".")).to_path_buf()
    };
    let template_file = template_file_for(&template_dir, "DESI")?;
    let set = if project_dirs.is_empty() {
        Arc::new(DbSet::with_attlib_file(&attlib)?)
    } else {
        let resolver = DirectoryDbResolver::scan(&project_dirs, &template_dir)?;
        Arc::new(DbSet::with_attlib_file_and_resolver(&attlib, Box::new(resolver))?)
    };
    set.add_db(DbFilePin {
        file: db_file.clone(),
        template: template_file,
        db_type: Some("DESI".into()),
        sesno: None,
    })?;

    let mut engine = ReadOnlyEngine::open(&db_file)?;
    let mut brans: Vec<RefNo> = Vec::new();
    for scanned in engine.scan_elements(ScanTier::Header)? {
        let refno = scanned?.refno;
        if set.element(refno).element_type()? == "BRAN" {
            brans.push(refno);
        }
    }
    println!("BRAN {} 条", brans.len());

    // (owner noun, member noun, GTYP, SPKBRK) -> count
    let mut tally: BTreeMap<(String, String, String, String), usize> = BTreeMap::new();
    for refno in brans {
        let bran = set.element(refno);
        let owner_noun = match bran.owner()? {
            Some(owner) => owner.element_type()?,
            None => "-".into(),
        };
        for member in bran.members()? {
            let noun = member.element_type()?;
            if !nouns.iter().any(|n| *n == noun) {
                continue;
            }
            let spkbrk = match member.attribute("SPKBRK") {
                Ok(Some(value)) => format!("{value:?}"),
                Ok(None) => "unset".into(),
                Err(e) => format!("err:{e}"),
            };
            let gtyp = catalogue_gtyp(&member).unwrap_or_else(|| "?".into());
            *tally
                .entry((owner_noun.clone(), noun.clone(), gtyp.clone(), spkbrk.clone()))
                .or_default() += 1;
            if verbose {
                println!(
                    "  {} {} {} {:?} owner={} GTYP={} SPKBRK={}",
                    bran.refno(),
                    member.refno(),
                    noun,
                    member.stored_name()?.unwrap_or_default(),
                    owner_noun,
                    gtyp,
                    spkbrk
                );
            }
        }
    }
    println!("owner noun | member noun | SCOM GTYP | SPKBRK | count");
    for ((owner, noun, gtyp, spkbrk), count) in &tally {
        println!("{owner:<6} {noun:<5} {gtyp:<6} {spkbrk:<12} {count}");
    }
    Ok(())
}

fn catalogue_gtyp(member: &DbElement) -> Option<String> {
    let spre = member.get_element("SPRE").ok().flatten()?;
    let mut cursor = spre;
    for _ in 0..6 {
        let noun = cursor.element_type().ok()?;
        if noun == "SCOM" {
            return match cursor.attribute("GTYP") {
                Ok(Some(value)) => Some(format!("{value:?}").replace("Word { raw: ", "").replace(", text: ", "/")),
                _ => Some("-".into()),
            };
        }
        cursor = cursor.get_element("CATR").ok().flatten()?;
    }
    None
}

//! 目录程序取证：把一个设计构件的 `SPRE → (CATR)* → SCOM → GMRE` 链整条打出来，
//! 逐个几何成员列出**原始存值**与按 `DbElementParamEnv` 求值的结果。
//!
//! `catalogue.rs` 的诊断只说「PXLE 求值失败」；这里回答的是「存的是什么、
//! 环境答不上的是哪个操作数」。库注册与 `gen_ams` 同一套（`--project-dir` /
//! `--catalogue`）。
//!
//! ```text
//! cargo run --release --example catalogue_probe -- --attlib <attlib.dat> \
//!     --db <ams7997_0001> --template <dir> [--catalogue <file> ...] \
//!     [--project-dir <dir> ...] --refno 24381/30274 [--refno ...]
//! ```

use std::path::{Path, PathBuf};
use std::sync::Arc;

use anyhow::{Context, bail};
use e3d_io::db_element::{DbElement, DbFilePin, DbSet, template_file_for};
use e3d_io::db_param_env::DbElementParamEnv;
use e3d_io::record::descriptor::stored_expression_words;
use e3d_io::refno::RefNo;
use e3d_model::db_discovery::DirectoryDbResolver;

const NUMERIC_ATTRS: &[&str] = &[
    "PXLE", "PYLE", "PZLE", "PX", "PY", "PZ", "PDIA", "PHEI", "PDIS", "PBDI", "PTDI", "PBOF",
    "PCOF", "PBTP", "PCTP", "PBBT", "PCBT", "PTDM", "PBDM", "POFF", "PRAD", "PANG", "PDEP", "PRAF",
    "PAXI", "PAAX", "PBAX", "PCAX", "PBOR", "PWID", "PHEIG",
];

fn main() -> anyhow::Result<()> {
    let mut attlib: Option<PathBuf> = None;
    let mut db_file: Option<PathBuf> = None;
    let mut template: Option<PathBuf> = None;
    let mut db_type = "DESI".to_string();
    let mut catalogues: Vec<PathBuf> = Vec::new();
    let mut project_dirs: Vec<PathBuf> = Vec::new();
    let mut refnos: Vec<RefNo> = Vec::new();

    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--attlib" => attlib = Some(PathBuf::from(value()?)),
            "--db" => db_file = Some(PathBuf::from(value()?)),
            "--template" => template = Some(PathBuf::from(value()?)),
            "--db-type" => db_type = value()?,
            "--catalogue" => catalogues.push(PathBuf::from(value()?)),
            "--project-dir" => project_dirs.push(PathBuf::from(value()?)),
            "--refno" => {
                let text = value()?;
                let (w0, w1) = text
                    .trim_start_matches('=')
                    .split_once('/')
                    .with_context(|| format!("--refno 要 w0/w1，得到 {text}"))?;
                refnos.push(RefNo::new(w0.trim().parse()?, w1.trim().parse()?));
            }
            other => bail!("不认识的参数 {other}"),
        }
    }
    let attlib = attlib.context("缺 --attlib")?;
    let db_file = db_file.context("缺 --db")?;
    let template = template.context("缺 --template")?;
    if refnos.is_empty() {
        bail!("至少给一个 --refno");
    }
    let template_dir = if template.is_dir() {
        template.clone()
    } else {
        template.parent().unwrap_or(Path::new(".")).to_path_buf()
    };
    let template_file = if template.is_dir() {
        template_file_for(&template, &db_type)?
    } else {
        template.clone()
    };

    let set = if project_dirs.is_empty() {
        Arc::new(DbSet::with_attlib_file(&attlib)?)
    } else {
        let resolver = DirectoryDbResolver::scan(&project_dirs, &template_dir)?;
        Arc::new(DbSet::with_attlib_file_and_resolver(
            &attlib,
            Box::new(resolver),
        )?)
    };
    set.add_db(DbFilePin {
        file: db_file,
        template: template_file,
        db_type: Some(db_type),
        sesno: None,
    })?;
    if !catalogues.is_empty() {
        let cata_template = template_file_for(&template_dir, "CATA")?;
        for file in &catalogues {
            set.add_db(DbFilePin {
                file: file.clone(),
                template: cata_template.clone(),
                db_type: Some("CATA".to_string()),
                sesno: None,
            })?;
        }
    }

    for refno in refnos {
        probe(&set.element(refno))?;
        println!();
    }
    Ok(())
}

fn dump_attrs(prefix: &str, el: &DbElement) -> anyhow::Result<()> {
    let mut line = Vec::new();
    for name in el.attribute_names()? {
        if let Ok(Some(value)) = el.attribute(&name) {
            line.push(format!("{name}={value:?}"));
        }
    }
    if !line.is_empty() {
        println!("{prefix} {}", line.join("  "));
    }
    Ok(())
}

fn follow_to_scom(mut current: DbElement) -> anyhow::Result<Option<DbElement>> {
    let mut seen = std::collections::HashSet::new();
    for _ in 0..8 {
        if !seen.insert(current.refno()) {
            return Ok(None);
        }
        let noun = current.element_type()?;
        println!(
            "  链 {} {} {:?}",
            current.refno(),
            noun,
            current.stored_name()?
        );
        if noun.eq_ignore_ascii_case("SCOM") {
            return Ok(Some(current));
        }
        // E3D 3.1 表驱动规格（电缆桥架）：TABITE → PRTREF → GPART → CATR → SCOM。
        let next = match current.get_element("CATR") {
            Ok(Some(next)) => Some(next),
            _ => current.get_element("PRTREF").unwrap_or_default(),
        };
        let Some(next) = next else {
            return Ok(None);
        };
        current = next;
    }
    Ok(None)
}

fn probe(member: &DbElement) -> anyhow::Result<()> {
    println!(
        "=== {} {} {:?}",
        member.refno(),
        member.element_type()?,
        member.stored_name()?
    );
    dump_attrs("  设计构件", member)?;
    let Some(spec) = member.get_element("SPRE")? else {
        println!("  无 SPRE");
        return Ok(());
    };
    let Some(scom) = follow_to_scom(spec)? else {
        println!("  SPRE 链未到 SCOM");
        return Ok(());
    };
    dump_attrs("  SCOM", &scom)?;
    let env = DbElementParamEnv::new(Some(member.clone()), Some(scom.clone())).without_insulation();

    for set_attr in ["GMRE", "PTRE"] {
        let Some(gmset) = scom.get_element(set_attr)? else {
            println!("  {set_attr}: 无");
            continue;
        };
        println!(
            "  {set_attr} → {} {} {:?}",
            gmset.refno(),
            gmset.element_type()?,
            gmset.stored_name()?
        );
        for primitive in gmset.members()? {
            let noun = primitive.element_type()?;
            println!("    -- {} {noun}", primitive.refno());
            dump_attrs("       原始", &primitive)?;
            let mut evaluated = Vec::new();
            for name in NUMERIC_ATTRS {
                if let Ok(outcome) = primitive.get_number(name, &env)
                    && primitive.attribute(name).ok().flatten().is_some()
                {
                    evaluated.push(format!("{name}={outcome:?}"));
                }
            }
            if !evaluated.is_empty() {
                println!("       求值 {}", evaluated.join("  "));
            }
            // 显式流里的 PML 词原样打出来：`Unresolved { form: Pml, text: None }` 只说
            // 「渲染不出」，要看词才知道缺的是哪个词元。
            let parsed = primitive.parsed()?;
            let mut raw_words = Vec::new();
            for name in NUMERIC_ATTRS {
                let hash = e3d_attlib::db1_hash(name);
                if let Some(words) = stored_expression_words(&parsed, hash) {
                    raw_words.push(format!("{name}={words:?}"));
                }
            }
            if !raw_words.is_empty() {
                println!("       PML词 {}", raw_words.join("  "));
            }
        }
    }
    Ok(())
}

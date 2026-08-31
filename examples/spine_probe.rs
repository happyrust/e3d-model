//! Read-only SPINE/CURVE ownership probe for the NOUN closure ledger.

use std::collections::BTreeSet;
use std::path::PathBuf;
use std::sync::Arc;

use anyhow::{Context, bail};
use e3d_io::db_element::{DbFilePin, DbSet, template_file_for};
use e3d_io::engine::{ReadOnlyEngine, ScanTier};

fn main() -> anyhow::Result<()> {
    let mut attlib = None;
    let mut db = None;
    let mut template = None;
    let mut nouns = BTreeSet::new();
    let mut args = std::env::args().skip(1);
    while let Some(flag) = args.next() {
        let mut value = || args.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--attlib" => attlib = Some(PathBuf::from(value()?)),
            "--db" => db = Some(PathBuf::from(value()?)),
            "--template" => template = Some(PathBuf::from(value()?)),
            "--noun" => {
                nouns.insert(value()?.to_ascii_uppercase());
            }
            other => bail!("不认识的参数 {other}"),
        }
    }
    let attlib = attlib.context("缺 --attlib")?;
    let db = db.context("缺 --db")?;
    let template = template.context("缺 --template")?;
    if nouns.is_empty() {
        nouns.extend(["SPINE".into(), "CURVE".into()]);
    }
    let template = if template.is_dir() {
        template_file_for(&template, "DESI")?
    } else {
        template
    };
    let set = Arc::new(DbSet::with_attlib_file(attlib)?);
    let dbno = set.add_db(DbFilePin {
        file: db.clone(),
        template,
        db_type: Some("DESI".into()),
        sesno: None,
    })?;
    let mut engine = match set.pinned_sesno(dbno) {
        Some(sesno) => ReadOnlyEngine::open_at(&db, sesno),
        None => ReadOnlyEngine::open(&db),
    }?;
    for scanned in engine.scan_elements(ScanTier::Header)? {
        let scanned = scanned?;
        let element = set.element(scanned.refno);
        let noun = element.element_type()?;
        if !nouns.contains(&noun) {
            continue;
        }
        let owner = element.owner()?;
        let owner_text = match owner {
            Some(owner) => format!("{} {}", owner.refno(), owner.element_type()?),
            None => "<none>".into(),
        };
        let members = element
            .members()?
            .map(|member| Ok(format!("{} {}", member.refno(), member.element_type()?)))
            .collect::<anyhow::Result<Vec<_>>>()?;
        println!(
            "{} {} owner=[{}] members=[{}]",
            element.refno(),
            noun,
            owner_text,
            members.join(", ")
        );
    }
    Ok(())
}

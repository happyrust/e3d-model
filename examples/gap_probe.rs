//! G0-1c 缺臂普查：在 `--project-dir` 发现的某个库里找指定 noun 的目录基本体，
//! 打出**全部已设属性**与成员树——SSLC/SDSH/SDIS 动手前的语料确认
//! （属性名以字典为准，别按反编译形参猜）。
//!
//! ```text
//! cargo run --example gap_probe -- --attlib <attlib.dat> --template <dir> \
//!     --project-dir <库目录>... --dbno N --noun SSLC [--samples 3]
//! ```

use std::path::PathBuf;
use std::sync::Arc;

use anyhow::{Context, bail};
use e3d_io::db_element::{DbElement, DbElementError, DbSet};
use e3d_io::engine::{ReadOnlyEngine, ScanTier};
use e3d_model::db_discovery::DirectoryDbResolver;

fn show_all_attrs(el: &DbElement, indent: &str) -> anyhow::Result<()> {
    for name in el.attribute_names()? {
        match el.attribute(&name) {
            Ok(Some(value)) => println!("{indent}{name} = {value:?}"),
            Ok(None) => {}
            Err(DbElementError::UnknownAttribute { .. }) => {}
            Err(error) => println!("{indent}{name} = （读取失败：{error}）"),
        }
    }
    Ok(())
}

fn main() -> anyhow::Result<()> {
    let mut attlib: Option<PathBuf> = None;
    let mut template: Option<PathBuf> = None;
    let mut project_dirs = Vec::new();
    let mut dbno: Option<u32> = None;
    let mut noun = String::new();
    let mut samples = 3usize;

    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--attlib" => attlib = Some(PathBuf::from(value()?)),
            "--template" => template = Some(PathBuf::from(value()?)),
            "--project-dir" => project_dirs.push(PathBuf::from(value()?)),
            "--dbno" => dbno = Some(value()?.parse().context("--dbno 要库号")?),
            "--noun" => noun = value()?,
            "--samples" => samples = value()?.parse().context("--samples 要个数")?,
            other => bail!("不认识的参数 {other}"),
        }
    }
    let attlib = attlib.context("缺 --attlib")?;
    let template = template.context("缺 --template")?;
    let dbno = dbno.context("缺 --dbno")?;
    if noun.is_empty() {
        bail!("缺 --noun");
    }

    let resolver = DirectoryDbResolver::scan(&project_dirs, &template)?;
    let pin = {
        use e3d_io::db_element::DbFileResolver as _;
        resolver
            .resolve(dbno)
            .with_context(|| format!("库 {dbno} 不在扫描目录里"))?
    };
    println!("库 {dbno} → {}（{:?}）", pin.file.display(), pin.db_type);

    let set = Arc::new(DbSet::with_attlib_file_and_resolver(
        &attlib,
        Box::new(resolver),
    )?);

    let mut engine =
        ReadOnlyEngine::open(&pin.file).with_context(|| format!("开库 {}", pin.file.display()))?;
    let mut total = 0usize;
    let mut shown = 0usize;
    let mut owner_nouns = std::collections::BTreeMap::<String, usize>::new();
    let mut member_nouns = std::collections::BTreeMap::<String, usize>::new();
    for scanned in engine.scan_elements(ScanTier::Header)? {
        let refno = scanned?.refno;
        let el = set.element(refno);
        if el.element_type()? != noun {
            continue;
        }
        total += 1;
        if let Some(owner) = el.owner()? {
            *owner_nouns
                .entry(owner.element_type().unwrap_or_else(|_| "?".into()))
                .or_insert(0) += 1;
        }
        for member in el.members()? {
            *member_nouns
                .entry(member.element_type().unwrap_or_else(|_| "?".into()))
                .or_insert(0) += 1;
        }
        if shown < samples {
            shown += 1;
            println!("── {noun} {refno} ──");
            show_all_attrs(&el, "  ")?;
            for member in el.members()? {
                println!("  成员 {} {}", member.element_type()?, member.refno());
                show_all_attrs(&member, "    ")?;
            }
        }
    }
    println!("{noun} 总数: {total}");
    println!("属主 noun 分布: {owner_nouns:?}");
    println!("成员 noun 分布: {member_nouns:?}");
    Ok(())
}

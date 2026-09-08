//! GENSEC 取证：把一本库里 GENSEC 的路径形状普查出来。
//!
//! GENSEC 与 SCTN 共用同一条截面链（`SPRE→SPCO→CATR→SPRF→GSTR→GMSS`），差别只在路径：
//! SCTN 是 `POSS→POSE` 一条直段，GENSEC 是一条 SPINE（POINSP 点串 + 可选 CURVE 弧）。
//! 这个探针回答两件事：**几件是直的**（SPINE 只有两点、没有 CURVE），
//! 以及**直的那些能不能直接套 SCTN 那条已经验过的直段路**。
//!
//! ```text
//! cargo run --release --example gensec_probe -- --attlib <attlib.dat> \
//!     --db <ams7324_0001> --template <dir> [--detail 5]
//! ```

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::Arc;

use anyhow::{Context, bail};
use e3d_io::db_element::{DbElement, DbFilePin, DbSet, template_file_for};
use e3d_io::engine::{ReadOnlyEngine, ScanTier};

fn main() -> anyhow::Result<()> {
    let mut attlib: Option<PathBuf> = None;
    let mut db: Option<PathBuf> = None;
    let mut template: Option<PathBuf> = None;
    let mut detail = 4usize;
    let mut curved_only = false;

    let mut args = std::env::args().skip(1);
    while let Some(flag) = args.next() {
        if flag == "--curved-only" {
            curved_only = true;
            continue;
        }
        let mut value = || args.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--attlib" => attlib = Some(PathBuf::from(value()?)),
            "--db" => db = Some(PathBuf::from(value()?)),
            "--template" => template = Some(PathBuf::from(value()?)),
            "--detail" => detail = value()?.parse()?,
            other => bail!("不认识的参数 {other}"),
        }
    }
    let attlib = attlib.context("缺 --attlib")?;
    let db = db.context("缺 --db")?;
    let template = template.context("缺 --template")?;
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

    let mut shape = BTreeMap::<String, usize>::new();
    let mut attrs = BTreeMap::<String, usize>::new();
    // 顶层属主（WORL 之下第一层）→ 件数：分清 SITE 下的真模型与 TPWL 下的模板。
    let mut roots = BTreeMap::<String, usize>::new();
    // 弧的类型串（如 `THRU` / `FILL,FILL,CENT,FILL,FILL`）→ 件数。
    let mut curve_types = BTreeMap::<String, usize>::new();
    let mut shown = 0usize;
    let mut total = 0usize;

    for scanned in engine.scan_elements(ScanTier::Header)? {
        let scanned = scanned?;
        let element = set.element(scanned.refno);
        if element.element_type()? != "GENSEC" {
            continue;
        }
        total += 1;

        // 顶层属主：沿 owner 链走到 WORL 的直接成员。
        {
            let mut top = element.clone();
            while let Some(owner) = top.owner()? {
                if owner.element_type()? == "WORL" {
                    break;
                }
                top = owner;
            }
            let key = format!(
                "{} {}",
                top.element_type()?,
                top.stored_name()?.unwrap_or_default()
            );
            *roots.entry(key).or_default() += 1;
        }

        // 路径：SPINE 下的 POINSP / CURVE 序列。
        let mut spine_members: Vec<String> = Vec::new();
        let mut spine_refno = None;
        let mut types: Vec<String> = Vec::new();
        for member in element.members()? {
            if member.element_type()? == "SPINE" {
                spine_refno = Some(member.refno().to_string());
                for point in member.members()? {
                    let noun = point.element_type()?;
                    if noun == "CURVE" {
                        let curtyp = match point.attribute("CURTYP") {
                            Ok(Some(value)) => format!("{value:?}"),
                            _ => "?".to_string(),
                        };
                        let curtyp = curtyp
                            .rsplit("Some(\"")
                            .next()
                            .unwrap_or("?")
                            .trim_end_matches("\") }")
                            .to_string();
                        types.push(curtyp);
                    }
                    spine_members.push(noun);
                }
            }
        }
        let points = spine_members.iter().filter(|t| *t == "POINSP").count();
        let curves = spine_members.iter().filter(|t| *t == "CURVE").count();
        if curves > 0 {
            *curve_types.entry(types.join(",")).or_default() += 1;
        }

        // 有哪些定位属性可用。模板里没有这条属性时 `get_*` 直接报错，所以只数「读得到」。
        for name in ["POSS", "POSE", "POS"] {
            let present = matches!(element.get_position(name), Ok(Some(_)));
            *attrs
                .entry(format!("{name} {}", if present { "有" } else { "无" }))
                .or_default() += 1;
        }
        for name in ["DRNS", "DRNE"] {
            if matches!(element.get_direction(name), Ok(Some(_))) {
                *attrs.entry(format!("{name} 有")).or_default() += 1;
            }
        }

        let key = if spine_refno.is_none() {
            "无 SPINE".to_string()
        } else if curves > 0 {
            format!("POINSP {points} + CURVE {curves}（带弧）")
        } else {
            format!("POINSP {points}（无弧）")
        };
        *shape.entry(key).or_default() += 1;

        if shown < detail && (!curved_only || curves > 0) {
            shown += 1;
            dump(&element, spine_refno.as_deref(), &spine_members)?;
        }
    }

    println!("\n=== GENSEC {total} 件，路径形状分布 ===");
    for (key, count) in &shape {
        println!("  {key:<34} x{count}");
    }
    println!("\n=== 弧类型串（带弧的件） ===");
    for (key, count) in &curve_types {
        println!("  {key:<40} x{count}");
    }
    println!("\n=== 顶层属主 ===");
    for (key, count) in &roots {
        println!("  {key:<40} x{count}");
    }
    println!("\n=== 定位属性 ===");
    for (key, count) in &attrs {
        println!("  {key:<12} x{count}");
    }
    Ok(())
}

fn dump(element: &DbElement, spine: Option<&str>, members: &[String]) -> anyhow::Result<()> {
    println!(
        "--- GENSEC {} {:?}",
        element.refno(),
        element.stored_name()?
    );
    let mut line = Vec::new();
    for name in element.attribute_names()? {
        if let Ok(Some(value)) = element.attribute(&name) {
            line.push(format!("{name}={value:?}"));
        }
    }
    println!("    属性: {}", line.join("  "));
    println!("    SPINE = {spine:?}  成员 = {members:?}");
    for member in element.members()? {
        if member.element_type()? != "SPINE" {
            continue;
        }
        for point in member.members()? {
            let mut fields = Vec::new();
            for name in point.attribute_names()? {
                if let Ok(Some(value)) = point.attribute(&name) {
                    fields.push(format!("{name}={value:?}"));
                }
            }
            println!(
                "      {} {} : {}",
                point.refno(),
                point.element_type()?,
                fields.join("  ")
            );
        }
    }
    Ok(())
}

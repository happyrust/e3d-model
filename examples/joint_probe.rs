//! 关节取证：一根 GENSEC 的 JOIS/JOIE 连到哪只 FIXING、它自己的 JLDATU/PLDATU/FIXING
//! 子树长什么样、每只 FIXING 的负体从哪来（TMPL 成员还是目录 JOIN 的 NGMR 集）、各带
//! 什么 `NAPP`。
//!
//! ```text
//! cargo run --release --example joint_probe -- --attlib <attlib.dat> --template <dir> \
//!     --db <aps250110_0001> --project-dir <aps000> [--catalogue <acp..._0001> ...] \
//!     --refno 2013286654/2209 [--refno ...]
//! ```

use std::path::PathBuf;
use std::sync::Arc;

use anyhow::{Context, bail};
use e3d_io::db_element::{DbElement, DbFilePin, DbSet, template_file_for};
use e3d_io::refno::RefNo;
use e3d_model::db_discovery::DirectoryDbResolver;
use e3d_model::transform::{local_matrix, world_matrix};

fn main() -> anyhow::Result<()> {
    let mut attlib: Option<PathBuf> = None;
    let mut db_file: Option<PathBuf> = None;
    let mut template: Option<PathBuf> = None;
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
    let template_dir = template.context("缺 --template")?;

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
        template: template_file_for(&template_dir, "DESI")?,
        db_type: Some("DESI".into()),
        sesno: None,
    })?;
    if !catalogues.is_empty() {
        let cata = template_file_for(&template_dir, "CATA")?;
        for file in catalogues {
            set.add_db(DbFilePin {
                file,
                template: cata.clone(),
                db_type: Some("CATA".into()),
                sesno: None,
            })?;
        }
    }

    for refno in refnos {
        let el = set.element(refno);
        println!(
            "##### {} {} {:?}",
            el.refno(),
            el.element_type()?,
            el.stored_name()?
        );
        attrs("  自身", &el)?;
        print_world("  世界", &el)?;
        for name in ["JOIS", "JOIE"] {
            match el.get_element(name) {
                Ok(Some(fixing)) => {
                    println!("  {name} → {} {}", fixing.refno(), fixing.element_type()?);
                    fixing_dump("    ", &fixing)?;
                }
                Ok(None) => println!("  {name} 空"),
                Err(error) => println!("  {name} 读不到：{error}"),
            }
        }
        subtree("  ", &el, 0)?;
    }
    Ok(())
}

fn attrs(prefix: &str, el: &DbElement) -> anyhow::Result<()> {
    let mut line = Vec::new();
    for name in el.attribute_names()? {
        if let Ok(Some(value)) = el.attribute(&name) {
            line.push(format!("{name}={value:?}"));
        }
    }
    println!("{prefix} {}", line.join("  "));
    Ok(())
}

fn print_world(prefix: &str, el: &DbElement) -> anyhow::Result<()> {
    let w = world_matrix(el)?;
    let l = local_matrix(el)?;
    println!(
        "{prefix} world_matrix T=[{:.3}, {:.3}, {:.3}] X=[{:.4}, {:.4}, {:.4}] Z=[{:.4}, {:.4}, {:.4}]  local T=[{:.3}, {:.3}, {:.3}]",
        w.w_axis.x,
        w.w_axis.y,
        w.w_axis.z,
        w.x_axis.x,
        w.x_axis.y,
        w.x_axis.z,
        w.z_axis.x,
        w.z_axis.y,
        w.z_axis.z,
        l.w_axis.x,
        l.w_axis.y,
        l.w_axis.z
    );
    Ok(())
}

/// SPINE / JLDATU / PLDATU / FIXING 这一支往下走；别的成员只报一行。
fn subtree(prefix: &str, el: &DbElement, depth: usize) -> anyhow::Result<()> {
    for member in el.members()? {
        let noun = member.element_type()?;
        match noun.as_str() {
            "SPINE" | "POINSP" | "CURVE" | "JLDATU" | "PLDATU" => {
                println!("{prefix}{} {noun}", member.refno());
                attrs(&format!("{prefix}  "), &member)?;
                if depth < 6 {
                    subtree(&format!("{prefix}  "), &member, depth + 1)?;
                }
            }
            "FIXING" => {
                println!("{prefix}{} FIXING", member.refno());
                fixing_dump(&format!("{prefix}  "), &member)?;
            }
            _ => println!("{prefix}{} {noun}", member.refno()),
        }
    }
    Ok(())
}

fn owning_section(fixing: &DbElement) -> anyhow::Result<Option<DbElement>> {
    let mut cursor = fixing.owner()?;
    while let Some(current) = cursor {
        let noun = current.element_type()?;
        if noun != "PLDATU" && noun != "JLDATU" && noun != "FIXING" {
            return Ok(Some(current));
        }
        cursor = current.owner()?;
    }
    Ok(None)
}

fn fixing_dump(prefix: &str, fixing: &DbElement) -> anyhow::Result<()> {
    attrs(&format!("{prefix}属性"), fixing)?;
    print_world(&format!("{prefix}"), fixing)?;
    let mut owner = fixing.owner()?;
    let mut chain = Vec::new();
    while let Some(current) = owner {
        chain.push(format!("{} {}", current.element_type()?, current.refno()));
        if chain.len() > 5 {
            break;
        }
        owner = current.owner()?;
    }
    println!("{prefix}属主链 {}", chain.join(" ← "));
    let mut tmpl = None;
    for member in fixing.members()? {
        if member.element_type()? == "TMPL" {
            tmpl = Some(member);
            break;
        }
    }
    if let Some(tmpl) = tmpl {
        println!("{prefix}TMPL {} {:?}", tmpl.refno(), tmpl.stored_name()?);
        attrs(&format!("{prefix}  "), &tmpl)?;
        for member in tmpl.members()? {
            println!("{prefix}  - {} {}", member.refno(), member.element_type()?);
            attrs(&format!("{prefix}      "), &member)?;
        }
        return Ok(());
    }
    let Some(spec) = fixing.get_element("SPRE")? else {
        println!("{prefix}无 SPRE、无 TMPL");
        return Ok(());
    };
    println!(
        "{prefix}SPRE {} {} {:?}",
        spec.refno(),
        spec.element_type()?,
        spec.stored_name()?
    );
    let join = match spec.get_element("CATR")? {
        Some(join) => join,
        None => spec.clone(),
    };
    println!(
        "{prefix}目录件 {} {} {:?}",
        join.refno(),
        join.element_type()?,
        join.stored_name()?
    );
    attrs(&format!("{prefix}  "), &join)?;
    if let Some(dtse) = join.get_element("DTRE")? {
        println!("{prefix}  DTRE → {} {}", dtse.refno(), dtse.element_type()?);
        for row in dtse.members()? {
            attrs(
                &format!("{prefix}    · {} {}", row.refno(), row.element_type()?),
                &row,
            )?;
        }
    }
    for (label, section) in [
        ("CREF(附着)", fixing.get_element("CREF")?),
        ("属主型材", owning_section(fixing)?),
    ] {
        let Some(section) = section else {
            println!("{prefix}  {label} 空");
            continue;
        };
        let sprf = section
            .get_element("SPRE")?
            .and_then(|spco| spco.get_element("CATR").ok().flatten());
        match sprf {
            Some(sprf) => println!(
                "{prefix}  {label} {} {} → SPRF {} {:?} PARA={:?}",
                section.refno(),
                section.element_type()?,
                sprf.refno(),
                sprf.stored_name()?,
                sprf.attribute("PARA")?
            ),
            None => println!("{prefix}  {label} {} 走不到 SPRF", section.refno()),
        }
    }
    for set_name in ["NGMR", "GMRE"] {
        let Some(geometry) = join.get_element(set_name)? else {
            println!("{prefix}  {set_name} 空");
            continue;
        };
        println!(
            "{prefix}  {set_name} → {} {}",
            geometry.refno(),
            geometry.element_type()?
        );
        for primitive in geometry.members()? {
            println!(
                "{prefix}    - {} {}",
                primitive.refno(),
                primitive.element_type()?
            );
            attrs(&format!("{prefix}        "), &primitive)?;
            for child in primitive.members()? {
                println!(
                    "{prefix}        · {} {}",
                    child.refno(),
                    child.element_type()?
                );
                attrs(&format!("{prefix}            "), &child)?;
                for grandchild in child.members()? {
                    println!(
                        "{prefix}            · {} {}",
                        grandchild.refno(),
                        grandchild.element_type()?
                    );
                    attrs(&format!("{prefix}                "), &grandchild)?;
                }
            }
        }
    }
    Ok(())
}

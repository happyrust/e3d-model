//! 单元素取证：把一个元素的 owner 链、POS/ORI、世界矩阵、环剖面原始顶点全打出来。
//!
//! RVM 对拍出偏差时先跑它——报告只告诉你差了多少，这里告诉你差在哪一步。
//!
//! ```text
//! cargo run --release --example element_probe -- --attlib <attlib.dat> \
//!     --db <ams1112_0001> --template <dir|desvir.dat> [--db-type DESI] \
//!     --refno 17496/117202 [--refno ...]
//! ```

use std::path::PathBuf;
use std::sync::Arc;

use anyhow::{Context, bail};
use e3d_io::db_element::{DbElement, DbFilePin, DbSet, template_file_for};
use e3d_io::refno::RefNo;
use e3d_model::transform::{local_matrix, world_matrix};

fn main() -> anyhow::Result<()> {
    let mut attlib: Option<PathBuf> = None;
    let mut db_file: Option<PathBuf> = None;
    let mut template: Option<PathBuf> = None;
    let mut db_type = "DESI".to_string();
    let mut sesno: Option<u32> = None;
    let mut refnos: Vec<RefNo> = Vec::new();

    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--attlib" => attlib = Some(PathBuf::from(value()?)),
            "--db" => db_file = Some(PathBuf::from(value()?)),
            "--template" => template = Some(PathBuf::from(value()?)),
            "--db-type" => db_type = value()?,
            "--sesno" => sesno = Some(value()?.parse().context("--sesno 要无符号整数")?),
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
    let template = if template.is_dir() {
        template_file_for(&template, &db_type)?
    } else {
        template
    };
    if refnos.is_empty() {
        bail!("至少给一个 --refno");
    }

    let set = Arc::new(DbSet::with_attlib_file(&attlib)?);
    let dbno = set.add_db(DbFilePin {
        file: db_file,
        template,
        db_type: Some(db_type),
        sesno,
    })?;
    println!("会话 {:?}", set.pinned_sesno(dbno));

    for refno in refnos {
        dump(&set.element(refno))?;
        println!();
    }
    Ok(())
}

/// 把元素上**实际存了值**的属性全打出来。挑着打就只会看见自己想到的那几个。
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

fn dump(el: &DbElement) -> anyhow::Result<()> {
    println!(
        "=== {} {} {:?}",
        el.refno(),
        el.element_type()?,
        el.stored_name()?
    );

    let mut chain = vec![el.clone()];
    let mut cursor = el.owner()?;
    while let Some(current) = cursor {
        chain.push(current.clone());
        cursor = current.owner()?;
    }
    println!("owner 链（根 → 自身）：");
    for node in chain.iter().rev() {
        println!(
            "  {} {:<8} {:<24} POS={:?} ORI={:?}",
            node.refno(),
            node.element_type()?,
            node.stored_name()?.unwrap_or_default(),
            node.get_position("POS").ok().flatten(),
            node.get_position("ORI").ok().flatten(),
        );
        let _ = local_matrix(node);
    }
    let world = world_matrix(el)?;
    println!("世界矩阵（列）：");
    for (label, col) in [
        ("X", world.x_axis),
        ("Y", world.y_axis),
        ("Z", world.z_axis),
        ("T", world.w_axis),
    ] {
        println!("  {label} = [{:.4}, {:.4}, {:.4}]", col.x, col.y, col.z);
    }

    dump_attrs("  自身", el)?;

    for member in el.members()? {
        let noun = member.element_type()?;
        if noun != "PLOO" && noun != "LOOP" {
            println!(
                "  成员 {} {} {:?}",
                member.refno(),
                noun,
                member.stored_name()?
            );
            continue;
        }
        println!(
            "  环 {} {noun} HEIG={:?} SJUS={:?} POS={:?} ORI={:?}",
            member.refno(),
            member.get_double("HEIG").ok().flatten(),
            member.get_string("SJUS").ok().flatten(),
            member.get_position("POS").ok().flatten(),
            member.get_position("ORI").ok().flatten(),
        );
        dump_attrs("    环", &member)?;
        let mut lo = [f64::MAX; 2];
        let mut hi = [f64::MIN; 2];
        for vertex in member.members()? {
            let vnoun = vertex.element_type()?;
            let pos = vertex
                .get_position("POS")
                .ok()
                .flatten()
                .unwrap_or([0.0; 3]);
            println!("    {vnoun} {}", vertex.refno());
            dump_attrs("      ", &vertex)?;
            for axis in 0..2 {
                lo[axis] = lo[axis].min(pos[axis]);
                hi[axis] = hi[axis].max(pos[axis]);
            }
        }
        if lo[0] <= hi[0] {
            println!(
                "    局部环包围盒 X[{}, {}] Y[{}, {}]",
                lo[0], hi[0], lo[1], hi[1]
            );
            let corners = [
                glam::DVec3::new(lo[0], lo[1], 0.0),
                glam::DVec3::new(hi[0], hi[1], 0.0),
            ];
            for corner in corners {
                let w = world.transform_point3(corner);
                println!(
                    "    世界系角点 {:?} → [{:.3}, {:.3}, {:.3}]",
                    corner, w.x, w.y, w.z
                );
            }
        }
    }
    Ok(())
}

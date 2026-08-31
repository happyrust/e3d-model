//! 把一个属主下某个 noun 的**原始环剖面 + 世界矩阵**整批倒成 JSON。
//!
//! RVM 对拍报「差了多少」，`element_probe` 报「一件差在哪」，这个报「一批的原料」——
//! 拿到手就能在外面拿几种摆放假设逐个试，不用为每个假设改一次生成代码。
//!
//! ```text
//! cargo run --release --example loop_dump -- --attlib <attlib.dat> --db <db> \
//!     --template <dir> --owner /1RS-WF03-W-C-RR001 --noun GWALL --out dump.json
//! ```

use std::path::PathBuf;
use std::sync::Arc;

use anyhow::{Context, bail};
use e3d_io::db_element::{DbFilePin, DbSet, template_file_for};
use e3d_model::profile::loop_profile;
use e3d_model::transform::world_matrix;

#[derive(serde::Serialize)]
struct Dumped {
    refno: String,
    noun: String,
    ordinal: usize,
    pos: Option<[f64; 3]>,
    ori: Option<[f64; 3]>,
    /// 世界矩阵，列主序 [c0 c1 c2 t]。
    world: [[f64; 3]; 4],
    height: f64,
    sjus_drop: f64,
    /// 环组，每个顶点是 [x, y, frad]。
    rings: Vec<Vec<[f64; 3]>>,
}

fn main() -> anyhow::Result<()> {
    let mut attlib: Option<PathBuf> = None;
    let mut db_file: Option<PathBuf> = None;
    let mut template: Option<PathBuf> = None;
    let mut db_type = "DESI".to_string();
    let mut owner_name: Option<String> = None;
    let mut noun_filter: Option<String> = None;
    let mut out: Option<PathBuf> = None;

    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--attlib" => attlib = Some(PathBuf::from(value()?)),
            "--db" => db_file = Some(PathBuf::from(value()?)),
            "--template" => template = Some(PathBuf::from(value()?)),
            "--db-type" => db_type = value()?,
            "--owner" => owner_name = Some(value()?),
            "--noun" => noun_filter = Some(value()?),
            "--out" => out = Some(PathBuf::from(value()?)),
            other => bail!("不认识的参数 {other}"),
        }
    }
    let attlib = attlib.context("缺 --attlib")?;
    let db_file = db_file.context("缺 --db")?;
    let template = template.context("缺 --template")?;
    let owner_name = owner_name.context("缺 --owner")?;
    let template = if template.is_dir() {
        template_file_for(&template, &db_type)?
    } else {
        template
    };

    let set = Arc::new(DbSet::with_attlib_file(&attlib)?);
    let dbno = set.add_db(DbFilePin {
        file: db_file,
        template,
        db_type: Some(db_type),
        sesno: None,
    })?;

    let found = set.find_named(dbno, &owner_name)?;
    let [owner_ref] = found[..] else {
        bail!(
            "`{owner_name}` 在 db {dbno} 里命中 {} 个，要恰好 1 个",
            found.len()
        );
    };
    let owner = set.element(owner_ref);

    let mut dumped = Vec::new();
    let mut seen = std::collections::BTreeMap::<String, usize>::new();
    for member in owner.members()? {
        let noun = member.element_type()?;
        let ordinal = seen.entry(noun.clone()).or_insert(0);
        *ordinal += 1;
        if noun_filter.as_deref().is_some_and(|want| want != noun) {
            continue;
        }
        let profile = match loop_profile(&member) {
            Ok(profile) => profile,
            Err(error) => {
                eprintln!("{} {noun}: {error}", member.refno());
                continue;
            }
        };
        let world = world_matrix(&member)?;
        dumped.push(Dumped {
            refno: member.refno().to_string(),
            noun,
            ordinal: *ordinal,
            pos: member.get_position("POS").ok().flatten(),
            ori: member.get_orientation("ORI").ok().flatten(),
            world: [
                world.x_axis.truncate().to_array(),
                world.y_axis.truncate().to_array(),
                world.z_axis.truncate().to_array(),
                world.w_axis.truncate().to_array(),
            ],
            height: profile.height,
            sjus_drop: profile.sjus_drop,
            rings: profile
                .rings
                .iter()
                .map(|ring| ring.iter().map(|v| [v.x, v.y, v.frad]).collect())
                .collect(),
        });
    }

    println!("{} 下 {} 件", owner_name, dumped.len());
    let json = serde_json::to_vec_pretty(&dumped)?;
    match out {
        Some(path) => {
            std::fs::write(&path, json)?;
            println!("写到 {}", path.display());
        }
        None => println!("{}", String::from_utf8_lossy(&json)),
    }
    Ok(())
}

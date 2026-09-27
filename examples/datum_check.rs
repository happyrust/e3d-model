//! GENSEC 名下 JLDATU / PLDATU 隐式标架对 E3D TTY 真值（`datum-frames.json`，
//! `Q POS/ORI WRT /*` 逐级打出来的 FIXING 属主链）的逐件核对。
//!
//! ```text
//! cargo run --release --example datum_check -- --attlib <attlib.dat> --template <dir> \
//!     --db <aps250110_0001> --project-dir <aps000> --project-dir <acp000> \
//!     --truth datum-frames.json --out datum-check.jsonl
//! ```

use std::collections::HashSet;
use std::io::Write;
use std::path::PathBuf;
use std::sync::Arc;

use anyhow::{Context, bail};
use e3d_io::db_element::{DbFilePin, DbSet, template_file_for};
use e3d_io::refno::RefNo;
use e3d_model::datum::{datum_local, solve};
use e3d_model::db_discovery::DirectoryDbResolver;
use e3d_model::section::section_fitting_placement;
use e3d_model::transform::world_matrix;
use glam::DMat4;
use serde_json::{Value, json};

fn main() -> anyhow::Result<()> {
    let mut attlib: Option<PathBuf> = None;
    let mut db_file: Option<PathBuf> = None;
    let mut template: Option<PathBuf> = None;
    let mut project_dirs: Vec<PathBuf> = Vec::new();
    let mut truth: Option<PathBuf> = None;
    let mut out: Option<PathBuf> = None;
    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--attlib" => attlib = Some(PathBuf::from(value()?)),
            "--db" => db_file = Some(PathBuf::from(value()?)),
            "--template" => template = Some(PathBuf::from(value()?)),
            "--project-dir" => project_dirs.push(PathBuf::from(value()?)),
            "--truth" => truth = Some(PathBuf::from(value()?)),
            "--out" => out = Some(PathBuf::from(value()?)),
            other => bail!("不认识的参数 {other}"),
        }
    }
    let template_dir = template.context("缺 --template")?;
    let resolver = DirectoryDbResolver::scan(&project_dirs, &template_dir)?;
    let set = Arc::new(DbSet::with_attlib_file_and_resolver(
        &attlib.context("缺 --attlib")?,
        Box::new(resolver),
    )?);
    set.add_db(DbFilePin {
        file: db_file.context("缺 --db")?,
        template: template_file_for(&template_dir, "DESI")?,
        db_type: Some("DESI".into()),
        sesno: None,
    })?;

    let chains: Vec<Vec<Value>> =
        serde_json::from_reader(std::fs::File::open(truth.context("缺 --truth")?)?)?;
    let mut writer = std::io::BufWriter::new(std::fs::File::create(out.context("缺 --out")?)?);
    let mut seen = HashSet::new();
    let (mut done, mut failed) = (0usize, 0usize);
    for chain in &chains {
        for level in chain {
            let noun = level["type"].as_str().unwrap_or("");
            if noun == "FITT" {
                let Some((w0, w1)) = level["refno"].as_str().and_then(|t| t.split_once('/')) else {
                    continue;
                };
                let el = set.element(RefNo::new(w0.parse()?, w1.parse()?));
                let solve = || -> anyhow::Result<Value> {
                    let host = el.owner()?.context("FITT 没有属主")?;
                    let placed = world_matrix(&host)? * section_fitting_placement(&host, &el)?;
                    Ok(json!({
                        "refno": level["refno"], "type": "FITT",
                        "host": format!("{} {}", host.element_type()?, host.refno()),
                        "truth": level, "gen": frame_json(placed),
                        "generic": frame_json(world_matrix(&el)?),
                    }))
                };
                let row = solve().unwrap_or_else(|error| {
                    json!({"refno": level["refno"], "type": "FITT", "error": error.to_string()})
                });
                writeln!(writer, "{row}")?;
                continue;
            }
            if noun == "FIXING" {
                let Some((w0, w1)) = level["refno"].as_str().and_then(|t| t.split_once('/')) else {
                    continue;
                };
                let el = set.element(RefNo::new(w0.parse()?, w1.parse()?));
                let row = match world_matrix(&el) {
                    Ok(world) => json!({
                        "refno": level["refno"], "type": "FIXING", "host": "",
                        "truth": level, "gen": frame_json(world),
                    }),
                    Err(error) => {
                        json!({"refno": level["refno"], "type": "FIXING", "error": error.to_string()})
                    }
                };
                writeln!(writer, "{row}")?;
                continue;
            }
            if noun != "JLDATU" && noun != "PLDATU" {
                continue;
            }
            let Some(text) = level["refno"].as_str() else {
                continue;
            };
            if !seen.insert(text.to_string()) {
                continue;
            }
            let Some((w0, w1)) = text.split_once('/') else {
                continue;
            };
            let el = set.element(RefNo::new(w0.parse()?, w1.parse()?));
            let row = match solve(&el) {
                Ok(Some(s)) => {
                    done += 1;
                    let world = s.host_world * datum_local(&s);
                    json!({
                        "refno": text, "type": noun, "host": s.host.refno().to_string(),
                        "truth": level,
                        "gen": frame_json(world),
                        "host_world": frame_json(s.host_world),
                        "zdis": s.zdis, "pkdi": s.pkdi, "hpref_s": s.hpref_s,
                        "length": s.path_length,
                        "point": s.point.to_array(), "tangent": s.tangent.to_array(),
                        "ydir": s.ydir.to_array(), "spin_deg": s.spin.to_degrees(),
                        "mirrored": s.mirrored, "jusl": s.justification,
                        "posl": s.posl, "pline_offset": s.pline_offset,
                        "pline_angle_deg": s.pline_angle.map(f64::to_degrees),
                        "pl_ydir": s.pl_ydir.map(|v| v.to_array()),
                        "notes": s.notes,
                    })
                }
                Ok(None) => json!({"refno": text, "type": noun, "skip": "no gensec host"}),
                Err(error) => {
                    failed += 1;
                    json!({"refno": text, "type": noun, "error": error.to_string()})
                }
            };
            writeln!(writer, "{row}")?;
        }
    }
    eprintln!("solved {done}, failed {failed}, distinct {}", seen.len());
    Ok(())
}

fn frame_json(m: DMat4) -> Value {
    json!({
        "wp": m.w_axis.truncate().to_array(),
        "wx": m.x_axis.truncate().to_array(),
        "wy": m.y_axis.truncate().to_array(),
        "wz": m.z_axis.truncate().to_array(),
    })
}

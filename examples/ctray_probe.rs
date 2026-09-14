//! 电缆桥架放置取证：把一本 DESI 库里每条 `CWBRAN` 的路径（`RPATH → POINTR`）、
//! 材料层（`CTMTRL → CTSTRA/CTBEND/CTRISE/CTTEE/… → CTRAY`）连同放置要读的全部
//! 属性打出来，并对整库做普查——`Core3D` 的 `CAB_DBPseudoPOSandORIonCTMTRL`
//! 按 `POSREF/ZDIS/PKDI/XOFF/ZOFF/BANG` 现算 POS/ORI，落地前得知道语料里这些格
//! 到底长什么样（哪些分支真会走到、哪些只有反编译背书）。
//!
//! ```text
//! cargo run --release --example ctray_probe -- --attlib <attlib.dat> \
//!     --template <dir> --db <ams251142_0001> [--db ...] \
//!     [--project-dir <dir> ...] [--dump 2] [--placement]
//! ```
//!
//! `--dump N`：前 N 条 CWBRAN 逐元素全打；`--placement`：对每个 CTELE 调
//! [`e3d_model::cable_tray::placement`] 打出算出来的 POS/ORI，供人工核对。

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use anyhow::{Context, bail};
use e3d_attlib::db1_dehash;
use e3d_io::db_element::{DbElement, DbFilePin, DbSet, template_file_for};
use e3d_io::engine::{ReadOnlyEngine, ScanTier};
use e3d_io::record::descriptor::DescriptorValue;
use e3d_model::cable_tray::{self, CABLE_TRAY_ELEMENT_NOUNS};
use e3d_model::catalogue::{CatalogueMeshCache, model_catalogue_member_with_cache};
use e3d_model::db_discovery::DirectoryDbResolver;

#[derive(Default)]
struct Census {
    cwbran: usize,
    ctele_by_noun: BTreeMap<String, usize>,
    posref_target_noun: BTreeMap<String, usize>,
    posref_missing: usize,
    pkdi_nonzero: usize,
    zdis_negative: usize,
    xoff_nonzero: usize,
    zoff_nonzero: usize,
    bang_nonzero: usize,
    cwbran_bang_nonzero: usize,
    cwbran_closef: usize,
    cwjusl: BTreeMap<String, usize>,
    vshape: BTreeMap<String, usize>,
    ydir: BTreeMap<String, usize>,
    rpath_member_noun: BTreeMap<String, usize>,
    frad_nonzero: usize,
    ctray_with_desp: usize,
    ctray_without_spre: usize,
    ctele_without_ctray: usize,
    placement_ok: BTreeMap<String, usize>,
    placement_anchor: BTreeMap<String, usize>,
    placement_err: BTreeMap<String, usize>,
    ctray_pos_nonzero: usize,
    ctray_ori_nonzero: usize,
    ctray_heig: BTreeMap<String, usize>,
    ctray_total: usize,
    generation_ok: usize,
    generation_none: usize,
    generation_err: BTreeMap<String, usize>,
}

fn main() -> anyhow::Result<()> {
    let mut attlib: Option<PathBuf> = None;
    let mut template: Option<PathBuf> = None;
    let mut dbs: Vec<PathBuf> = Vec::new();
    let mut catalogues: Vec<PathBuf> = Vec::new();
    let mut project_dirs: Vec<PathBuf> = Vec::new();
    let mut dump = 0usize;
    let mut placement = false;
    let mut generate = false;

    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--attlib" => attlib = Some(PathBuf::from(value()?)),
            "--template" => template = Some(PathBuf::from(value()?)),
            "--db" => dbs.push(PathBuf::from(value()?)),
            "--catalogue" => catalogues.push(PathBuf::from(value()?)),
            "--project-dir" => project_dirs.push(PathBuf::from(value()?)),
            "--dump" => dump = value()?.parse()?,
            "--placement" => placement = true,
            "--generate" => generate = true,
            other => bail!("不认识的参数 {other}"),
        }
    }
    let attlib = attlib.context("缺 --attlib")?;
    let template = template.context("缺 --template")?;
    if dbs.is_empty() {
        bail!("至少给一个 --db");
    }
    let template_dir = if template.is_dir() {
        template.clone()
    } else {
        template.parent().unwrap_or(Path::new(".")).to_path_buf()
    };
    let desi_template = template_file_for(&template_dir, "DESI")?;

    let set = if project_dirs.is_empty() {
        Arc::new(DbSet::with_attlib_file(&attlib)?)
    } else {
        let resolver = DirectoryDbResolver::scan(&project_dirs, &template_dir)?;
        Arc::new(DbSet::with_attlib_file_and_resolver(
            &attlib,
            Box::new(resolver),
        )?)
    };
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

    let mut census = Census::default();
    let mut catalogue_cache = CatalogueMeshCache::default();
    let mut dumped = 0usize;
    for db_file in &dbs {
        let dbno = set.add_db(DbFilePin {
            file: db_file.clone(),
            template: desi_template.clone(),
            db_type: Some("DESI".to_string()),
            sesno: None,
        })?;
        let pinned = set.pinned_sesno(dbno);
        let mut engine = match pinned {
            Some(sesno) => ReadOnlyEngine::open_at(db_file, sesno),
            None => ReadOnlyEngine::open(db_file),
        }?;
        let mut cwbrans = Vec::new();
        for scanned in engine.scan_elements(ScanTier::Header)? {
            let scanned = scanned?;
            if db1_dehash(scanned.noun_hash).as_deref() == Some("CWBRAN") {
                cwbrans.push(scanned.refno);
            }
        }
        println!(
            "== db {dbno} {} — CWBRAN {} 条",
            db_file.display(),
            cwbrans.len()
        );
        for refno in cwbrans {
            let branch = set.element(refno);
            let verbose = dumped < dump;
            if verbose {
                dumped += 1;
            }
            probe_branch(
                &branch,
                verbose,
                placement,
                generate,
                &mut catalogue_cache,
                &mut census,
            )?;
        }
    }

    println!("\n=== 普查 ===");
    println!("CWBRAN {}", census.cwbran);
    println!(
        "  BANG≠0 {}  CLOSEF=true {}",
        census.cwbran_bang_nonzero, census.cwbran_closef
    );
    println!("  CWJUSL {:?}", census.cwjusl);
    println!("  VSHAPE {:?}", census.vshape);
    println!("RPATH.YDIR {:?}", census.ydir);
    println!(
        "RPATH 成员 noun {:?}  FRAD≠0 {}",
        census.rpath_member_noun, census.frad_nonzero
    );
    println!("CTELE {:?}", census.ctele_by_noun);
    println!(
        "  POSREF→ {:?}  POSREF 缺 {}",
        census.posref_target_noun, census.posref_missing
    );
    println!(
        "  PKDI≠0 {}  ZDIS<0 {}  XOFF≠0 {}  ZOFF≠0 {}  BANG≠0 {}",
        census.pkdi_nonzero,
        census.zdis_negative,
        census.xoff_nonzero,
        census.zoff_nonzero,
        census.bang_nonzero
    );
    println!(
        "  无 CTRAY 成员 {}  CTRAY 带 DESP {}  CTRAY 无 SPRE {}",
        census.ctele_without_ctray, census.ctray_with_desp, census.ctray_without_spre
    );
    println!(
        "  CTRAY POS≠0 {}  ORI≠0 {}  HEIG {:?}",
        census.ctray_pos_nonzero, census.ctray_ori_nonzero, census.ctray_heig
    );
    if placement {
        println!("放置：解出 {:?}", census.placement_ok);
        println!("  落点 {:?}", census.placement_anchor);
        println!("  出错 {:?}", census.placement_err);
        if generate {
            println!(
                "目录生成：CTRAY {}  成功 {}  无几何 {}  出错 {:?}",
                census.ctray_total,
                census.generation_ok,
                census.generation_none,
                census.generation_err
            );
            println!("目录缓存 {:?}", catalogue_cache.stats());
        }
    }
    Ok(())
}

fn number(el: &DbElement, name: &str) -> Option<f64> {
    el.get_double(name).ok().flatten()
}

fn text(el: &DbElement, name: &str) -> String {
    match el.attribute(name) {
        Ok(Some(DescriptorValue::Word {
            text: Some(text), ..
        })) => text,
        Ok(Some(DescriptorValue::Word { raw, .. })) => format!("#{raw}"),
        Ok(Some(DescriptorValue::Text(text))) => text,
        Ok(Some(other)) => format!("{other:?}"),
        Ok(None) => "unset".into(),
        Err(_) => "n/a".into(),
    }
}

fn fmt3(value: Option<[f64; 3]>) -> String {
    match value {
        Some([x, y, z]) => format!("({x:.3}, {y:.3}, {z:.3})"),
        None => "unset".into(),
    }
}

fn probe_branch(
    branch: &DbElement,
    verbose: bool,
    placement: bool,
    generate: bool,
    catalogue_cache: &mut CatalogueMeshCache,
    census: &mut Census,
) -> anyhow::Result<()> {
    census.cwbran += 1;
    let bang = number(branch, "BANG").unwrap_or(0.0);
    if bang.abs() > 1e-9 {
        census.cwbran_bang_nonzero += 1;
    }
    if branch.get_bool("CLOSEF")? == Some(true) {
        census.cwbran_closef += 1;
    }
    *census.cwjusl.entry(text(branch, "CWJUSL")).or_insert(0) += 1;
    *census.vshape.entry(text(branch, "VSHAPE")).or_insert(0) += 1;
    if verbose {
        println!(
            "\n--- CWBRAN {} {:?} POS={} ORI={} CWJUSL={} VWIDT={:?} VHEIG={:?} VSHAPE={} BANG={bang} CLOSEF={:?} BRAD={:?}",
            branch.refno(),
            branch.stored_name()?,
            fmt3(branch.get_position("POS")?),
            fmt3(branch.get_orientation("ORI")?),
            text(branch, "CWJUSL"),
            number(branch, "VWIDT"),
            number(branch, "VHEIG"),
            text(branch, "VSHAPE"),
            branch.get_bool("CLOSEF")?,
            number(branch, "BRAD"),
        );
    }
    for member in branch.members()? {
        match member.element_type()?.as_str() {
            "RPATH" => {
                let ydir = member.get_direction("YDIR")?;
                *census.ydir.entry(fmt3(ydir)).or_insert(0) += 1;
                if verbose {
                    println!("  RPATH {} YDIR={}", member.refno(), fmt3(ydir));
                }
                for point in member.members()? {
                    let noun = point.element_type()?;
                    *census.rpath_member_noun.entry(noun.clone()).or_insert(0) += 1;
                    let frad = number(&point, "FRAD").unwrap_or(0.0);
                    if frad.abs() > 1e-9 {
                        census.frad_nonzero += 1;
                    }
                    if verbose {
                        println!(
                            "    {noun} {} POS={} FRAD={frad} POITYP={}",
                            point.refno(),
                            fmt3(point.get_position("POS").unwrap_or(None)),
                            text(&point, "POITYP")
                        );
                    }
                }
            }
            "CTMTRL" => {
                if verbose {
                    println!(
                        "  CTMTRL {} TRAYW={:?} TRAYH={:?} CTSPE={:?}",
                        member.refno(),
                        number(&member, "TRAYW"),
                        number(&member, "TRAYH"),
                        member.get_ref("CTSPE")?,
                    );
                }
                for ctele in member.members()? {
                    probe_ctele(
                        &ctele,
                        verbose,
                        placement,
                        generate,
                        catalogue_cache,
                        census,
                    )?;
                }
            }
            _ => {}
        }
    }
    Ok(())
}

fn probe_ctele(
    ctele: &DbElement,
    verbose: bool,
    placement: bool,
    generate: bool,
    catalogue_cache: &mut CatalogueMeshCache,
    census: &mut Census,
) -> anyhow::Result<()> {
    let noun = ctele.element_type()?;
    *census.ctele_by_noun.entry(noun.clone()).or_insert(0) += 1;
    let posref = ctele.get_element("POSREF").unwrap_or(None);
    match &posref {
        Some(target) => {
            let target_noun = target.element_type().unwrap_or_else(|e| format!("<{e}>"));
            *census.posref_target_noun.entry(target_noun).or_insert(0) += 1;
        }
        None => census.posref_missing += 1,
    }
    let zdis = number(ctele, "ZDIS").unwrap_or(0.0);
    let pkdi = number(ctele, "PKDI").unwrap_or(0.0);
    let xoff = number(ctele, "XOFF").unwrap_or(0.0);
    let zoff = number(ctele, "ZOFF").unwrap_or(0.0);
    let bang = number(ctele, "BANG").unwrap_or(0.0);
    if pkdi.abs() > 1e-9 {
        census.pkdi_nonzero += 1;
    }
    if zdis < 0.0 {
        census.zdis_negative += 1;
    }
    if xoff.abs() > 1e-9 {
        census.xoff_nonzero += 1;
    }
    if zoff.abs() > 1e-9 {
        census.zoff_nonzero += 1;
    }
    if bang.abs() > 1e-9 {
        census.bang_nonzero += 1;
    }
    if verbose {
        let mut extra = Vec::new();
        for name in [
            "OLENG", "ALENG", "LLENG", "SLENG", "RADI", "ANGL", "CTTYP", "TRAYW", "TRAYH",
        ] {
            if let Ok(Some(value)) = ctele.attribute(name) {
                extra.push(format!("{name}={value:?}"));
            }
        }
        println!(
            "    {noun} {} POSREF={} ZDIS={zdis} PKDI={pkdi} XOFF={xoff} ZOFF={zoff} BANG={bang} {}",
            ctele.refno(),
            posref
                .as_ref()
                .map(|t| format!("{}({})", t.refno(), t.element_type().unwrap_or_default()))
                .unwrap_or_else(|| "none".into()),
            extra.join(" ")
        );
    }
    if placement && CABLE_TRAY_ELEMENT_NOUNS.contains(&noun.as_str()) {
        match cable_tray::placement(ctele) {
            Ok(placed) => {
                *census.placement_ok.entry(noun.clone()).or_insert(0) += 1;
                let anchor_kind = match placed.anchor {
                    cable_tray::Anchor::Segment { .. } => "segment",
                    cable_tray::Anchor::EndPoint(_) => "endpoint",
                    cable_tray::Anchor::MissingRouteOrigin => "missing-route-origin",
                };
                *census
                    .placement_anchor
                    .entry(format!("{noun}:{anchor_kind}"))
                    .or_insert(0) += 1;
                if verbose {
                    let m = placed.rotation;
                    println!(
                        "      → POS=({:.3}, {:.3}, {:.3}) dir(Y)=({:.4}, {:.4}, {:.4}) up(Z)=({:.4}, {:.4}, {:.4}) right(X)=({:.4}, {:.4}, {:.4}) 落点 {}",
                        placed.position.x,
                        placed.position.y,
                        placed.position.z,
                        m.y_axis.x,
                        m.y_axis.y,
                        m.y_axis.z,
                        m.z_axis.x,
                        m.z_axis.y,
                        m.z_axis.z,
                        m.x_axis.x,
                        m.x_axis.y,
                        m.x_axis.z,
                        placed.anchor,
                    );
                }
            }
            Err(error) => {
                let text = error.to_string();
                // 按「detail 的前 40 个字符」分桶，同一种断法归一格。
                let key: String = text
                    .split(": ")
                    .last()
                    .unwrap_or(&text)
                    .chars()
                    .take(40)
                    .collect();
                *census.placement_err.entry(key).or_insert(0) += 1;
                if verbose {
                    println!("      → 放置出错: {error}");
                }
            }
        }
    }
    let mut saw_ctray = false;
    for part in ctele.members()? {
        if part.element_type()? != "CTRAY" {
            continue;
        }
        saw_ctray = true;
        census.ctray_total += 1;
        let spre = part.get_element("SPRE").unwrap_or(None);
        if spre.is_none() {
            census.ctray_without_spre += 1;
        }
        if part.get_double_array("DESP").ok().flatten().is_some() {
            census.ctray_with_desp += 1;
        }
        let part_pos = part.get_position("POS").unwrap_or(None);
        let part_ori = part.get_orientation("ORI").unwrap_or(None);
        if part_pos.is_some_and(|p| p.iter().any(|v| v.abs() > 1e-9)) {
            census.ctray_pos_nonzero += 1;
        }
        if part_ori.is_some_and(|o| o.iter().any(|v| v.abs() > 1e-9)) {
            census.ctray_ori_nonzero += 1;
        }
        *census
            .ctray_heig
            .entry(
                number(&part, "HEIG")
                    .map(|h| format!("{h}"))
                    .unwrap_or_else(|| "unset".into()),
            )
            .or_insert(0) += 1;
        if generate {
            match model_catalogue_member_with_cache(&part, catalogue_cache) {
                Ok(build) if build.generated.is_some() => census.generation_ok += 1,
                Ok(_) => census.generation_none += 1,
                Err(error) => {
                    let key: String = error.to_string().chars().take(80).collect();
                    *census.generation_err.entry(key).or_insert(0) += 1;
                }
            }
        }
        if verbose {
            println!(
                "      CTRAY {} SPRE={} HEIG={:?} DESP={:?} POS={} ORI={}",
                part.refno(),
                spre.as_ref()
                    .map(|s| format!(
                        "{} {}",
                        s.refno(),
                        s.stored_name().ok().flatten().unwrap_or_default()
                    ))
                    .unwrap_or_else(|| "none".into()),
                number(&part, "HEIG"),
                part.get_double_array("DESP").ok().flatten(),
                fmt3(part.get_position("POS").unwrap_or(None)),
                fmt3(part.get_orientation("ORI").unwrap_or(None)),
            );
        }
    }
    if !saw_ctray {
        census.ctele_without_ctray += 1;
    }
    Ok(())
}

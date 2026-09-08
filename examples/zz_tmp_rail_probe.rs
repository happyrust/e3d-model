//! 临时探针：aps250110 详图式扶手（RAIL / KICKPL / HRPOST）的路径、端点引用与截面链。
//! 用完删。

use std::path::{Path, PathBuf};
use std::sync::Arc;

use anyhow::{Context, bail};
use e3d_io::db_element::{DbElement, DbFilePin, DbSet, template_file_for};
use e3d_io::engine::{ReadOnlyEngine, ScanTier};
use e3d_model::db_discovery::DirectoryDbResolver;
use e3d_model::section::{SectionChain, evaluate_section_profile, section_catalogue};

fn main() -> anyhow::Result<()> {
    let mut attlib: Option<PathBuf> = None;
    let mut db_file: Option<PathBuf> = None;
    let mut template: Option<PathBuf> = None;
    let mut catalogues: Vec<PathBuf> = Vec::new();
    let mut project_dirs: Vec<PathBuf> = Vec::new();
    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--attlib" => attlib = Some(PathBuf::from(value()?)),
            "--db" => db_file = Some(PathBuf::from(value()?)),
            "--template" => template = Some(PathBuf::from(value()?)),
            "--catalogue" => catalogues.push(PathBuf::from(value()?)),
            "--project-dir" => project_dirs.push(PathBuf::from(value()?)),
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
    let template_file = if template.is_dir() {
        template_file_for(&template, "DESI")?
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
    let dbno = set.add_db(DbFilePin {
        file: db_file.clone(),
        template: template_file,
        db_type: Some("DESI".to_string()),
        sesno: None,
    })?;
    let cata_template = template_file_for(&template_dir, "CATA")?;
    for file in &catalogues {
        set.add_db(DbFilePin {
            file: file.clone(),
            template: cata_template.clone(),
            db_type: Some("CATA".to_string()),
            sesno: None,
        })?;
    }
    let mut engine = match set.pinned_sesno(dbno) {
        Some(s) => ReadOnlyEngine::open_at(&db_file, s),
        None => ReadOnlyEngine::open(&db_file),
    }?;
    for scanned in engine.scan_elements(ScanTier::Header)? {
        let scanned = scanned?;
        let el = set.element(scanned.refno);
        let Ok(noun) = el.element_type() else {
            continue;
        };
        if noun != "HANDRA" {
            continue;
        }
        let members: Vec<DbElement> = el.members()?.collect();
        let nouns: Vec<String> = members
            .iter()
            .map(|m| m.element_type().unwrap_or_default())
            .collect();
        let detailed = nouns
            .iter()
            .any(|n| matches!(n.as_str(), "RAILSE" | "POSTSE" | "HRKPSE" | "HRPNSE"));
        println!(
            "\n=== HANDRA {} {:?} CLOSEF={:?} SHOWKP={:?} RAILHE={:?} POS={:?} ORI={:?} 成员 {:?} {}",
            el.refno(),
            el.stored_name()?,
            el.get_bool("CLOSEF")?,
            el.get_bool("SHOWKP")?,
            el.get_double_array("RAILHE")?,
            el.get_position("POS")?,
            el.get_direction("ORI").ok().flatten(),
            nouns,
            if detailed {
                "【详图式】"
            } else {
                "【布局式】"
            }
        );
        let mut points: Vec<(String, [f64; 3], f64)> = Vec::new();
        for m in &members {
            if m.element_type()? == "RPATH" {
                println!(
                    "  RPATH {} YDIR={:?} DRNS={:?} DRNE={:?}",
                    m.refno(),
                    m.get_direction("YDIR")?,
                    m.get_direction("DRNS")?,
                    m.get_direction("DRNE")?
                );
                for p in m.members()? {
                    let pn = p.element_type()?;
                    if pn != "POINTR" {
                        println!("    !! 路径成员 {} 是 {pn}", p.refno());
                        continue;
                    }
                    let pos = p.get_position("POS")?.unwrap_or([f64::NAN; 3]);
                    let frad = p.get_double("FRAD")?.unwrap_or(0.0);
                    points.push((p.refno().to_string(), pos, frad));
                }
                for (i, (r, pos, frad)) in points.iter().enumerate() {
                    println!("    [{i:3}] {r:<18} POS={pos:?} FRAD={frad}");
                }
            }
        }
        if !detailed {
            continue;
        }
        let index_of = |target: Option<DbElement>| -> String {
            match target {
                None => "None".to_string(),
                Some(t) => {
                    let r = t.refno().to_string();
                    match points.iter().position(|(pr, _, _)| *pr == r) {
                        Some(i) => format!("[{i}] {r}"),
                        None => format!("?? {r} ({})", t.element_type().unwrap_or_default()),
                    }
                }
            }
        };
        for set_el in &members {
            let sn = set_el.element_type()?;
            if !matches!(sn.as_str(), "RAILSE" | "POSTSE" | "HRKPSE" | "HRPNSE") {
                continue;
            }
            println!(
                "  {sn} {} POS={:?} ORI={:?}",
                set_el.refno(),
                set_el.get_position("POS").ok().flatten(),
                set_el.get_direction("ORI").ok().flatten()
            );
            for item in set_el.members()? {
                let inoun = item.element_type()?;
                let attr = |name: &str| -> String {
                    if let Ok(Some(v)) = item.get_double(name) {
                        return format!("{v}");
                    }
                    if let Ok(Some(v)) = item.get_string(name) {
                        return v;
                    }
                    if let Ok(Some(v)) = item.get_bool(name) {
                        return format!("{v}");
                    }
                    if let Ok(Some(v)) = item.get_direction(name) {
                        return format!("{v:?}");
                    }
                    "-".to_string()
                };
                match inoun.as_str() {
                    "RAIL" | "KICKPL" => {
                        println!(
                            "    {inoun} {} FUNC={} JUSL={} XOFF={} ZOFF={} BANG={} LMIRR={} RADS={} BENDCO={} YDIR={} DRNS={} DRNE={} LEVE={:?}",
                            item.refno(),
                            attr("FUNC"),
                            attr("JUSL"),
                            attr("XOFF"),
                            attr("ZOFF"),
                            attr("BANG"),
                            attr("LMIRR"),
                            attr("RADS"),
                            attr("BENDCO"),
                            attr("YDIR"),
                            attr("DRNS"),
                            attr("DRNE"),
                            item.get_double_array("LEVE").ok().flatten()
                        );
                        println!(
                            "      STAREF={}  ENDREF={}  SPKDI={} EPKDI={} SZDI={} EZDI={}",
                            index_of(item.get_element("STAREF")?),
                            index_of(item.get_element("ENDREF")?),
                            attr("SPKDI"),
                            attr("EPKDI"),
                            attr("SZDI"),
                            attr("EZDI")
                        );
                    }
                    "HRPOST" => {
                        println!(
                            "    {inoun} {} POSREF={} LENG={} ZDIS={} XOFF={} ZOFF={} HEIG={} PKDI={} VERTDI={} HRJUSL={} DRNS={} DRNE={} POS={:?} ORI={:?}",
                            item.refno(),
                            index_of(item.get_element("POSREF")?),
                            attr("LENG"),
                            attr("ZDIS"),
                            attr("XOFF"),
                            attr("ZOFF"),
                            attr("HEIG"),
                            attr("PKDI"),
                            attr("VERTDI"),
                            attr("HRJUSL"),
                            attr("DRNS"),
                            attr("DRNE"),
                            item.get_position("POS").ok().flatten(),
                            item.get_direction("ORI").ok().flatten()
                        );
                        let sub: Vec<String> = item
                            .members()?
                            .map(|m| m.element_type().unwrap_or_default())
                            .collect();
                        println!("      成员 {sub:?}");
                        // ACC_CSG_HRPost 的 ctor（sub_109FF790）拿的是 POSS / POSE，
                        // 而 HRPOST 的模板里没有这两条：看看这一层读不读得出来。
                        println!(
                            "      POSS={:?} POSE={:?}",
                            item.get_position("POSS"),
                            item.get_position("POSE")
                        );
                    }
                    other => {
                        println!("    {other} {}", item.refno());
                        continue;
                    }
                }
                match section_catalogue(&item)? {
                    SectionChain::NoSpre => println!("      链：无 SPRE"),
                    SectionChain::Broken { hop, at, why } => {
                        println!("      链：断在第 {hop} 跳 {at}：{why}")
                    }
                    SectionChain::Resolved(cat) => {
                        println!(
                            "      链：SPCO={:?} SPRF={} {:?} GMSS={}",
                            cat.spco.as_ref().map(|s| s.refno().to_string()),
                            cat.sprf.refno(),
                            cat.sprf.stored_name()?,
                            cat.gmss.refno()
                        );
                        match evaluate_section_profile(&item, &cat) {
                            Ok(profile) => {
                                println!(
                                    "      截面：{} 成员 JUSL={} found={} offset={:?} spin={:.4} mirrored={} level={}",
                                    profile.members.len(),
                                    profile.justification_line,
                                    profile.justification_found,
                                    profile.justification,
                                    profile.neutral_axis_spin,
                                    profile.mirrored,
                                    profile.level
                                );
                                for m in &profile.members {
                                    let outer: Vec<String> = m.rings[0]
                                        .iter()
                                        .map(|v| format!("({:.1},{:.1} r{:.0})", v.x, v.y, v.frad))
                                        .collect();
                                    println!(
                                        "        {:?} {} 环={} 外环 {}",
                                        m.source,
                                        m.member_refno,
                                        m.rings.len(),
                                        outer.join(" ")
                                    );
                                }
                                for n in &profile.notes {
                                    println!("        note: {n}");
                                }
                            }
                            Err(e) => println!("      截面求值失败：{e}"),
                        }
                    }
                }
            }
        }
    }
    Ok(())
}

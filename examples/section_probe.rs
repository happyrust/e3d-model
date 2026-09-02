//! 型材取证：SCTN / GENSEC 的 `SPRE` 链走到哪、截面目录件长什么样。
//!
//! G1-1a 第 1 步（`docs/2026-09-02-development-plan-core3d-recheck.md` §三）：Core3D
//! `getProfileFromDB@0x10410b30 → sub_10410BB0`（`profileExtractFromDB`）只认一张 noun 表
//! （PLOO/LOOP/SPRO/SLOO/SREC/SANN/PAVE/VERT/…），从截面元素取顶点 + bulge 拼成 `D2_Profile`，
//! 再对 SPRO/SREC/SANN 减去 `sub_10399B54` 算出的截面偏移。这个探针先回答「库里的 SCTN 到底
//! 引到哪种截面元素、成员与属性是什么」，实现照它的答案接线，不按手册猜。
//!
//! ```text
//! cargo run --release --example section_probe -- --attlib <attlib.dat> \
//!     --db <ams8000_0001> --template <dir> [--catalogue <file> ...] \
//!     [--project-dir <dir> ...] [--noun SCTN] [--limit 5] [--refno w0/w1 ...]
//! ```

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use anyhow::{Context, bail};
use e3d_io::db_element::{DbElement, DbFilePin, DbSet, template_file_for};
use e3d_io::engine::{ReadOnlyEngine, ScanTier};
use e3d_io::refno::RefNo;
use e3d_model::db_discovery::DirectoryDbResolver;
use e3d_model::discretise::flatten_ring;
use e3d_model::section::{
    SectionChain, SectionProfile, SectionSource, evaluate_section_profile, section_catalogue,
};

fn main() -> anyhow::Result<()> {
    let mut attlib: Option<PathBuf> = None;
    let mut db_file: Option<PathBuf> = None;
    let mut template: Option<PathBuf> = None;
    let mut db_type = "DESI".to_string();
    let mut catalogues: Vec<PathBuf> = Vec::new();
    let mut project_dirs: Vec<PathBuf> = Vec::new();
    let mut refnos: Vec<RefNo> = Vec::new();
    let mut noun_filter = "SCTN".to_string();
    let mut limit = 5usize;
    let mut census_only = false;
    let mut evaluate = false;
    let mut out: Option<PathBuf> = None;

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
            "--noun" => noun_filter = value()?.to_ascii_uppercase(),
            "--limit" => limit = value()?.parse().context("--limit 要整数")?,
            "--census" => census_only = true,
            "--evaluate" => evaluate = true,
            "--out" => out = Some(PathBuf::from(value()?)),
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
    let dbno = set.add_db(DbFilePin {
        file: db_file.clone(),
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

    if refnos.is_empty() {
        let pinned = set.pinned_sesno(dbno);
        let mut engine = match pinned {
            Some(s) => ReadOnlyEngine::open_at(&db_file, s),
            None => ReadOnlyEngine::open(&db_file),
        }?;
        let mut census: BTreeMap<String, usize> = BTreeMap::new();
        // SPRE 链的形态普查：终点 noun（或「无 SPRE」/「悬空」）→ 件数，附一个样本 refno。
        let mut chain_shapes: BTreeMap<String, (usize, RefNo)> = BTreeMap::new();
        let mut all: Vec<RefNo> = Vec::new();
        for scanned in engine.scan_elements(ScanTier::Header)? {
            let scanned = scanned?;
            let el = set.element(scanned.refno);
            let Ok(noun) = el.element_type() else {
                continue;
            };
            if !noun.eq_ignore_ascii_case(&noun_filter) {
                continue;
            }
            *census.entry(noun.clone()).or_default() += 1;
            all.push(scanned.refno);
            let shape = chain_shape(&el);
            let entry = chain_shapes.entry(shape).or_insert((0, scanned.refno));
            entry.0 += 1;
            if !census_only && !evaluate && refnos.len() < limit {
                refnos.push(scanned.refno);
            }
        }
        println!("{noun_filter} 普查：{census:?}");
        for (shape, (count, sample)) in &chain_shapes {
            println!("  链形态 {shape}: {count} 件，样本 {sample}");
        }
        if evaluate {
            return evaluate_all(&set, &all, limit, out.as_deref());
        }
        if census_only {
            return Ok(());
        }
        println!("取前 {} 件逐个打印", refnos.len());
    }

    if evaluate {
        return evaluate_all(&set, &refnos, limit.max(refnos.len()), out.as_deref());
    }
    for refno in refnos {
        probe(&set.element(refno))?;
        println!();
    }
    Ok(())
}

#[derive(serde::Serialize)]
struct MemberDump {
    source: String,
    member_refno: String,
    /// 折线化（含 PRAD 倒圆）后的环，外环在前；已减去 JUSL 偏移。
    rings_flattened: Vec<Vec<[f64; 2]>>,
}

#[derive(serde::Serialize)]
struct ProfileDump {
    refno: String,
    name: Option<String>,
    members: Vec<MemberDump>,
    justification_line: String,
    justification_found: bool,
    justification: [f64; 2],
    level: u32,
    /// 全部成员外环的总 AABB（截面平面，已减去 JUSL 偏移）。
    aabb_min: [f64; 2],
    aabb_max: [f64; 2],
    notes: Vec<String>,
}

fn source_tag(source: SectionSource) -> String {
    match source {
        SectionSource::Spro { vertices } => format!("SPRO({vertices})"),
        SectionSource::Srec => "SREC".to_string(),
        SectionSource::Sann => "SANN".to_string(),
    }
}

fn profile_key(profile: &SectionProfile) -> String {
    let mut sources: Vec<String> = profile
        .members
        .iter()
        .map(|member| source_tag(member.source))
        .collect();
    sources.sort();
    sources.join("+")
}

fn dump_profile(el: &DbElement, profile: &SectionProfile) -> anyhow::Result<ProfileDump> {
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    let mut members = Vec::new();
    for member in &profile.members {
        let mut rings_flattened = Vec::new();
        for ring in &member.rings {
            let Some((points, _)) = flatten_ring(ring) else {
                bail!(
                    "{} 成员 {} 的截面环折线化后退化",
                    el.refno(),
                    member.member_refno
                );
            };
            rings_flattened.push(
                points
                    .into_iter()
                    .map(|[x, y]| [x - profile.justification[0], y - profile.justification[1]])
                    .collect::<Vec<_>>(),
            );
        }
        if let Some(outer) = rings_flattened.first() {
            for p in outer {
                for axis in 0..2 {
                    lo[axis] = lo[axis].min(p[axis]);
                    hi[axis] = hi[axis].max(p[axis]);
                }
            }
        }
        members.push(MemberDump {
            source: source_tag(member.source),
            member_refno: member.member_refno.clone(),
            rings_flattened,
        });
    }
    Ok(ProfileDump {
        refno: el.refno().to_string(),
        name: el.stored_name()?,
        members,
        justification_line: profile.justification_line.clone(),
        justification_found: profile.justification_found,
        justification: profile.justification,
        level: profile.level,
        aabb_min: lo,
        aabb_max: hi,
        notes: profile.notes.clone(),
    })
}

/// 全库逐件求截面并分账；前 `limit` 件成功的打印 AABB，`--out` 给了就把它们的环倒成 JSON。
fn evaluate_all(
    set: &Arc<DbSet>,
    refnos: &[RefNo],
    limit: usize,
    out: Option<&Path>,
) -> anyhow::Result<()> {
    let mut tally: BTreeMap<String, usize> = BTreeMap::new();
    let mut samples: BTreeMap<String, RefNo> = BTreeMap::new();
    let mut dumps: Vec<ProfileDump> = Vec::new();
    let mut bump = |key: String, refno: RefNo| {
        *tally.entry(key.clone()).or_default() += 1;
        samples.entry(key).or_insert(refno);
    };
    for &refno in refnos {
        let el = set.element(refno);
        let chain = match section_catalogue(&el) {
            Ok(chain) => chain,
            Err(error) => {
                bump(format!("链: 错误 {error}"), refno);
                continue;
            }
        };
        let catalogue = match chain {
            SectionChain::NoSpre => {
                bump("无 SPRE（E3D 只画中心线）".to_string(), refno);
                continue;
            }
            SectionChain::Broken { hop, why, .. } => {
                bump(format!("链断[{hop}]: {why}"), refno);
                continue;
            }
            SectionChain::Resolved(catalogue) => catalogue,
        };
        match evaluate_section_profile(&el, &catalogue) {
            Ok(profile) if profile.is_empty() => {
                let why = profile
                    .notes
                    .iter()
                    .map(|note| note.replace(&refno.to_string(), "<refno>"))
                    .collect::<Vec<_>>()
                    .join(" | ");
                bump(format!("GMSS 无成员出形状: {why}"), refno);
            }
            Ok(profile) => {
                let key = format!(
                    "求出 {} / JUSL {}{}",
                    profile_key(&profile),
                    profile.justification_line,
                    if profile.justification_found {
                        ""
                    } else {
                        "（未找到，按原点）"
                    }
                );
                bump(key, refno);
                if dumps.len() < limit {
                    match dump_profile(&el, &profile) {
                        Ok(dump) => dumps.push(dump),
                        Err(error) => bump(format!("折线化失败: {error}"), refno),
                    }
                }
            }
            Err(error) => {
                // 把 refno 从错误文案里抠掉，同类错误才聚得到一起。
                let text = error.to_string().replace(&refno.to_string(), "<refno>");
                bump(format!("求值失败: {text}"), refno);
            }
        }
    }
    println!("== 截面求值分账（{} 件）", refnos.len());
    for (key, count) in &tally {
        println!("  {count:>6}  {key}   样本 {}", samples[key]);
    }
    println!("== 前 {} 件成功样本", dumps.len());
    for dump in &dumps {
        println!(
            "  {} {:?} 成员 {} JUSL {}={:?} AABB [{:.2}, {:.2}]..[{:.2}, {:.2}] notes {}",
            dump.refno,
            dump.name,
            dump.members
                .iter()
                .map(|m| format!(
                    "{}@{} 环点数 {:?}",
                    m.source,
                    m.member_refno,
                    m.rings_flattened.iter().map(Vec::len).collect::<Vec<_>>()
                ))
                .collect::<Vec<_>>()
                .join(" + "),
            dump.justification_line,
            dump.justification,
            dump.aabb_min[0],
            dump.aabb_min[1],
            dump.aabb_max[0],
            dump.aabb_max[1],
            dump.notes.len()
        );
        for note in &dump.notes {
            println!("      note: {note}");
        }
    }
    if let Some(path) = out {
        std::fs::write(path, serde_json::to_vec_pretty(&dumps)?)?;
        println!("写到 {}", path.display());
    }
    Ok(())
}

/// `SPRE → (CATR | GSTR | PRTREF)*` 一路走到底，用「每一跳的 noun」拼成链形态字串。
fn chain_shape(el: &DbElement) -> String {
    let Ok(Some(spec)) = el.get_element("SPRE") else {
        return "无 SPRE".to_string();
    };
    let mut current = spec;
    let mut hops = Vec::new();
    let mut seen = std::collections::HashSet::new();
    for _ in 0..8 {
        if !seen.insert(current.refno()) {
            hops.push("回环".to_string());
            break;
        }
        match current.element_type() {
            Ok(noun) => hops.push(noun),
            Err(_) => {
                hops.push("悬空".to_string());
                break;
            }
        }
        let next = match current.get_element("CATR") {
            Ok(Some(next)) => Some(next),
            _ => match current.get_element("GSTR") {
                Ok(Some(next)) => Some(next),
                _ => current.get_element("PRTREF").unwrap_or_default(),
            },
        };
        match next {
            Some(next) => current = next,
            None => {
                // 终点带成员（GMSS）就把成员 noun 一起写进形态，实现前得知道要接哪几种截面臂。
                if let Ok(members) = current.members() {
                    let mut nouns: Vec<String> = members
                        .into_iter()
                        .filter_map(|m| m.element_type().ok())
                        .collect();
                    nouns.sort();
                    nouns.dedup();
                    if !nouns.is_empty() {
                        hops.push(format!("[{}]", nouns.join(",")));
                    }
                }
                break;
            }
        }
    }
    hops.join(" → ")
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

fn dump_subtree(prefix: &str, el: &DbElement, depth: usize) -> anyhow::Result<()> {
    for member in el.members()? {
        println!(
            "{prefix}-- {} {} {:?}",
            member.refno(),
            member.element_type()?,
            member.stored_name()?
        );
        dump_attrs(&format!("{prefix}     "), &member)?;
        if depth > 0 {
            dump_subtree(&format!("{prefix}   "), &member, depth - 1)?;
        }
    }
    Ok(())
}

fn probe(el: &DbElement) -> anyhow::Result<()> {
    println!(
        "=== {} {} {:?}",
        el.refno(),
        el.element_type()?,
        el.stored_name()?
    );
    dump_attrs("  自身", el)?;
    if let Some(owner) = el.owner()? {
        println!(
            "  属主 {} {} {:?}",
            owner.refno(),
            owner.element_type()?,
            owner.stored_name()?
        );
    }
    let Some(spec) = el.get_element("SPRE")? else {
        println!("  无 SPRE");
        return Ok(());
    };
    let mut current = spec;
    let mut seen = std::collections::HashSet::new();
    for hop in 0..8 {
        if !seen.insert(current.refno()) {
            println!("  链回环，停");
            break;
        }
        let noun = current.element_type()?;
        println!(
            "  链[{hop}] {} {} {:?}",
            current.refno(),
            noun,
            current.stored_name()?
        );
        dump_attrs("        ", &current)?;
        // 到了带几何成员的目录件就把成员树打两层。
        if matches!(
            noun.as_str(),
            "SPRO" | "SREC" | "SANN" | "SCOM" | "PRFL" | "SPRF" | "GMSS" | "GMSE" | "SLOO"
        ) {
            dump_subtree("        ", &current, 2)?;
        }
        let next = match current.get_element("CATR") {
            Ok(Some(next)) => Some(next),
            _ => match current.get_element("GSTR") {
                Ok(Some(next)) => Some(next),
                _ => current.get_element("PRTREF").unwrap_or_default(),
            },
        };
        let Some(next) = next else {
            // 终点：把它名下再打两层，看截面数据挂在哪。
            if !matches!(noun.as_str(), "SPRO" | "SREC" | "SANN" | "SCOM") {
                dump_subtree("        ", &current, 2)?;
            }
            break;
        };
        current = next;
    }
    // 成员：SCTN 名下的关节 / 端部件也顺手看一眼（只一层）。
    dump_subtree("  成员 ", el, 0)?;
    Ok(())
}

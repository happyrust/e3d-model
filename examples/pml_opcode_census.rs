//! 目录 PML 操作码普查：`catalogue_pml::render` 拒掉的程序，按第一个认不出的操作码或形状分组。
//!
//! 库注册与 `catalogue_probe` / `ptcd_census` 同一套（`--project-dir` / `--catalogue`）。
//! 程序取自显式流里 `is_program` 为真的整段，以及 `PTCD` 方向规格里嵌着的转角/分量程序
//! （`direction_spec` 的 opcode 21+ 与 52）。判定只调用公开的 `is_program` / `render`；
//! 分组键是对照 `catalogue_pml::parse` 走到第一处 `None` 的那个词。走完仍渲染失败的
//! 记成 `render/stack`，不设「未知」组。
//!
//! ```text
//! cargo run --release --example pml_opcode_census -- --attlib <attlib.dat> \
//!     --template <schema-dir> --project-dir <ams000> [--project-dir ...] \
//!     --catalogue <ams7000_0001> [--catalogue ...] \
//!     --design <ams7999_0001> [--design ...] [--out report.md]
//! ```

use std::collections::{BTreeMap, HashMap, HashSet};
use std::path::{Path, PathBuf};
use std::sync::Arc;

use anyhow::{Context, bail};
use e3d_attlib::{SYSTEM_HASH_MAX, db1_dehash, lookup_system_name};
use e3d_io::db_element::{DbElement, DbFilePin, DbSet, template_file_for};
use e3d_io::engine::{ReadOnlyEngine, ScanTier};
use e3d_io::record::catalogue_pml::{is_program, render};
use e3d_io::record::explicit::ExplicitValue;
use e3d_io::refno::RefNo;
use e3d_model::db_discovery::DirectoryDbResolver;

const OP_PUSH_NUMBER: i32 = 101;
const OP_ATTRIBUTE: i32 = 106;
const OP_PUSH_TEXT: i32 = 118;
const OP_PI: i32 = 111;
const NUMBER_SHAPE: i32 = 6;
const NUMBER_WITH_UNIT_SHAPE: i32 = 9;
const NUMBER_TYPED_SHAPE: i32 = 7;
const ATTRIBUTE_SHAPE: i32 = 2;
const ATTRIBUTE_SHAPE_OWNED: i32 = 4;
const OP_OWNER_CHAIN: i32 = 1602;
const ATTRIBUTE_TERMINATOR: i32 = 1701;
const CHAIN_OPEN: i32 = 2;
const CHAIN_CLOSE: i32 = 6;
const NAME_FLOOR: i32 = 0x0008_1BF2;
const PREVIOUS_ELEMENT: i32 = 10;
const DOUBLE_FLAG: i32 = 0x4000_0000;
const MAX_SCALED_EXPONENT: i32 = 64;

/// 单词操作码：`catalogue_pml::parse` 里 `consumed = 1` 的那一支。
const SINGLE_WORD_OPS: &[i32] = &[
    OP_PI, 401, 601, 603, 605, 607, 801, 802, 803, 804, 805, 901, 902, 903, 904, 906, 907, 1001,
    1002, 1005, 1007, 1008, 1009, 1308, 1314, 1410, 1822,
];

struct Group {
    programs: usize,
    elements: HashSet<RefNo>,
    nouns: BTreeMap<String, usize>,
    attrs: BTreeMap<String, usize>,
    scoms: HashSet<RefNo>,
    sample_refno: RefNo,
    sample_noun: String,
    sample_attr: String,
    sample_db: String,
    sample_words: Vec<i32>,
    /// 几何 noun（PTCA / PTMI / PTAX）的第一份全文，DATA 样例不够用。
    geo_noun: String,
    geo_attr: String,
    geo_db: String,
    geo_refno: RefNo,
    geo_words: Vec<i32>,
    geo_elements: HashSet<RefNo>,
    /// 最多四份全文，用来看 1802 是不是都长一样。
    extra: Vec<(String, String, String, RefNo, Vec<i32>)>,
}

fn main() -> anyhow::Result<()> {
    let mut attlib: Option<PathBuf> = None;
    let mut template: Option<PathBuf> = None;
    let mut catalogues: Vec<PathBuf> = Vec::new();
    let mut designs: Vec<PathBuf> = Vec::new();
    let mut project_dirs: Vec<PathBuf> = Vec::new();
    let mut out_path: Option<PathBuf> = None;

    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--attlib" => attlib = Some(PathBuf::from(value()?)),
            "--template" => template = Some(PathBuf::from(value()?)),
            "--catalogue" => catalogues.push(PathBuf::from(value()?)),
            "--design" => designs.push(PathBuf::from(value()?)),
            "--project-dir" => project_dirs.push(PathBuf::from(value()?)),
            "--out" => out_path = Some(PathBuf::from(value()?)),
            other => bail!("不认识的参数 {other}"),
        }
    }
    let attlib = attlib.context("缺 --attlib")?;
    let template = template.context("缺 --template")?;
    if catalogues.is_empty() {
        bail!("至少给一个 --catalogue");
    }
    let template_dir = if template.is_dir() {
        template.clone()
    } else {
        template.parent().unwrap_or(Path::new(".")).to_path_buf()
    };
    let cata_template = template_file_for(&template_dir, "CATA")?;
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

    let mut groups: BTreeMap<String, Group> = BTreeMap::new();
    let mut owners: HashMap<RefNo, RefNo> = HashMap::new();
    let mut by_set: HashMap<RefNo, Vec<RefNo>> = HashMap::new();
    let mut programs_ok = 0usize;
    let mut programs_rejected = 0usize;
    let mut per_db: Vec<String> = Vec::new();

    for file in &catalogues {
        let dbno = set.add_db(DbFilePin {
            file: file.clone(),
            template: cata_template.clone(),
            db_type: Some("CATA".to_string()),
            sesno: None,
        })?;
        let label = file_label(file);
        let refnos = scan_refnos(&set, file, dbno)?;
        let elements = refnos.len();
        println!("{label} dbno={dbno} 元素 {elements}");
        let mut ok_here = 0usize;
        let mut bad_here = 0usize;
        for refno in refnos {
            let element = set.element(refno);
            let noun = element.element_type().unwrap_or_else(|_| "?".to_string());
            let parsed = match element.parsed() {
                Ok(parsed) => parsed,
                Err(_) => continue,
            };
            if parsed.owner.is_valid() {
                owners.insert(refno, parsed.owner);
            }
            if noun == "SCOM" {
                note_scom_sets(&element, refno, &mut by_set);
            }
            for attribute in &parsed.explicit_attributes {
                let Some(words) = candidate_words(&attribute.value) else {
                    continue;
                };
                let name = attr_name(attribute.hash);
                if is_program(&words) {
                    tally(
                        &words,
                        &name,
                        refno,
                        &noun,
                        &label,
                        &mut programs_ok,
                        &mut programs_rejected,
                        &mut ok_here,
                        &mut bad_here,
                        &mut groups,
                    );
                    continue;
                }
                for (index, program) in embedded_programs(&words) {
                    let attr = format!("{name}[{index}]");
                    tally(
                        &program,
                        &attr,
                        refno,
                        &noun,
                        &label,
                        &mut programs_ok,
                        &mut programs_rejected,
                        &mut ok_here,
                        &mut bad_here,
                        &mut groups,
                    );
                }
            }
        }
        let line = format!("{label}: 元素 {elements}，读通 {ok_here}，被拒 {bad_here}");
        println!("  {line}");
        per_db.push(line);
    }

    let mut design_by_scom: HashMap<RefNo, Vec<String>> = HashMap::new();
    for file in &designs {
        let dbno = set.add_db(DbFilePin {
            file: file.clone(),
            template: desi_template.clone(),
            db_type: Some("DESI".to_string()),
            sesno: None,
        })?;
        let label = file_label(file);
        let refnos = scan_refnos(&set, file, dbno)?;
        println!("{label} dbno={dbno} 设计元素 {}", refnos.len());
        let mut linked = 0usize;
        for refno in refnos {
            let element = set.element(refno);
            let Ok(Some(spec)) = element.get_element("SPRE") else {
                continue;
            };
            let Some(scom) = follow_to_scom(spec) else {
                continue;
            };
            let name = element
                .stored_name()
                .ok()
                .flatten()
                .unwrap_or_default();
            let zone = zone_of(&element);
            design_by_scom
                .entry(scom)
                .or_default()
                .push(format!("{label} {refno} {name} | {zone}"));
            linked += 1;
        }
        println!("  {label}: SPRE 链到 SCOM 的设计件 {linked}");
    }

    for group in groups.values_mut() {
        for element in &group.elements {
            for scom in scoms_for(*element, &owners, &by_set) {
                group.scoms.insert(scom);
            }
        }
    }

    let mut rows: Vec<(&String, &Group)> = groups.iter().collect();
    rows.sort_by(|a, b| b.1.programs.cmp(&a.1.programs).then(a.0.cmp(b.0)));

    let mut report = String::new();
    let header = format!(
        "程序读通 {programs_ok}，被拒 {programs_rejected}，分组 {}（无「未知」组）",
        rows.len()
    );
    println!("\n{header}");
    report.push_str(&header);
    report.push('\n');
    for line in &per_db {
        report.push_str(line);
        report.push('\n');
    }
    report.push('\n');

    for (key, group) in &rows {
        let design_items: usize = group
            .scoms
            .iter()
            .map(|scom| design_by_scom.get(scom).map(Vec::len).unwrap_or(0))
            .sum();
        let nouns = group
            .nouns
            .iter()
            .map(|(noun, count)| format!("{noun} {count}"))
            .collect::<Vec<_>>()
            .join(", ");
        let line = format!(
            "{key}: 程序 {}，图元/P点 {}，SCOM {}，设计件 {design_items}（{nouns}）",
            group.programs,
            group.elements.len(),
            group.scoms.len(),
        );
        println!("\n{line}");
        println!(
            "  代表 {} {} {} {}",
            group.sample_db, group.sample_refno, group.sample_noun, group.sample_attr
        );
        println!("  词 {}", fmt_words(&group.sample_words));
        report.push_str(&line);
        report.push('\n');
        report.push_str(&format!(
            "  代表 {} {} {} {}\n  词 {}\n",
            group.sample_db, group.sample_refno, group.sample_noun, group.sample_attr,
            fmt_words(&group.sample_words)
        ));
        if group.nouns.contains_key("PTCA")
            || group.nouns.contains_key("PTMI")
            || group.nouns.contains_key("PTAX")
        {
            let mut geo_scoms: HashSet<RefNo> = HashSet::new();
            for element in &group.geo_elements {
                for scom in scoms_for(*element, &owners, &by_set) {
                    geo_scoms.insert(scom);
                }
            }
            let mut geo_zones: BTreeMap<String, usize> = BTreeMap::new();
            for scom in &geo_scoms {
                for line in design_by_scom.get(scom).into_iter().flatten() {
                    let zone = line.split(" | ").nth(1).unwrap_or(line);
                    *geo_zones.entry(zone.to_string()).or_default() += 1;
                }
            }
            let geo_design: usize = geo_zones.values().sum();
            println!(
                "  只算 PTCA/PTMI/PTAX 的设计件 {geo_design}（SCOM {}）",
                geo_scoms.len()
            );
            report.push_str(&format!(
                "  只算 PTCA/PTMI/PTAX 的设计件 {geo_design}（SCOM {}）\n",
                geo_scoms.len()
            ));
            let mut shown = 0usize;
            for scom in &geo_scoms {
                for line in design_by_scom.get(scom).into_iter().flatten() {
                    if shown < 8 {
                        println!("  几何设计 {line}");
                        shown += 1;
                    }
                }
            }
            println!("  几何设计件按属主（路径含 refno；超过 25 条只印前 25）：");
            report.push_str("  几何设计件按属主：\n");
            let mut baseline = BTreeMap::<&str, usize>::new();
            let needles = [
                ("24383/66457", "66457"),
                ("24381/100817", "CUP"),
                ("1RX07-LCT", "1RX07-LCT"),
                ("C-OR-1R345", "C-OR"),
                ("C-IY-1R330", "C-IY"),
                ("RR001", "RR001"),
            ];
            let mut printed = 0usize;
            for (zone, count) in &geo_zones {
                if printed < 25 {
                    let line = format!("    {count} {zone}");
                    println!("{line}");
                    report.push_str(&line);
                    report.push('\n');
                    printed += 1;
                }
                for (needle, label) in needles {
                    if zone.contains(needle) {
                        *baseline.entry(label).or_default() += count;
                    }
                }
            }
            let hit = if baseline.is_empty() {
                "基线命中：66457 / CUP(24381/100817) / 1RX07-LCT / C-OR / C-IY / RR001 都是 0".to_string()
            } else {
                format!("基线命中 {baseline:?}（其余为 0）")
            };
            println!("  {hit}");
            report.push_str(&format!("  {hit}\n"));
            if !group.geo_words.is_empty() {
                let line = format!(
                    "  几何样例 {} {} {} {} 词 {}",
                    group.geo_db,
                    group.geo_refno,
                    group.geo_noun,
                    group.geo_attr,
                    fmt_words_full(&group.geo_words)
                );
                println!("{line}");
                report.push_str(&line);
                report.push('\n');
            }
            for (db, noun, attr, refno, words) in &group.extra {
                let line = format!("  全文 {db} {refno} {noun} {attr} {}", fmt_words_full(words));
                println!("{line}");
                report.push_str(&line);
                report.push('\n');
            }
        }
    }

    if let Some(path) = &out_path {
        if let Some(parent) = path.parent()
            && !parent.as_os_str().is_empty()
        {
            std::fs::create_dir_all(parent)?;
        }
        std::fs::write(path, &report)?;
        println!("\n写出 {}", path.display());
    }
    Ok(())
}

fn tally(
    words: &[i32],
    attr: &str,
    refno: RefNo,
    noun: &str,
    db: &str,
    programs_ok: &mut usize,
    programs_rejected: &mut usize,
    ok_here: &mut usize,
    bad_here: &mut usize,
    groups: &mut BTreeMap<String, Group>,
) {
    if render(words).is_some() {
        *programs_ok += 1;
        *ok_here += 1;
        return;
    }
    *programs_rejected += 1;
    *bad_here += 1;
    let key = first_decline(words);
    let group = groups.entry(key).or_insert_with(|| Group {
        programs: 0,
        elements: HashSet::new(),
        nouns: BTreeMap::new(),
        attrs: BTreeMap::new(),
        scoms: HashSet::new(),
        sample_refno: refno,
        sample_noun: noun.to_string(),
        sample_attr: attr.to_string(),
        sample_db: db.to_string(),
        sample_words: words.to_vec(),
        geo_noun: String::new(),
        geo_attr: String::new(),
        geo_db: String::new(),
        geo_refno: refno,
        geo_words: Vec::new(),
        geo_elements: HashSet::new(),
        extra: Vec::new(),
    });
    group.programs += 1;
    group.elements.insert(refno);
    *group.nouns.entry(noun.to_string()).or_default() += 1;
    *group.attrs.entry(attr.to_string()).or_default() += 1;
    if matches!(noun, "PTCA" | "PTMI" | "PTAX") {
        group.geo_elements.insert(refno);
        if group.geo_words.is_empty() {
            group.geo_noun = noun.to_string();
            group.geo_attr = attr.to_string();
            group.geo_db = db.to_string();
            group.geo_refno = refno;
            group.geo_words = words.to_vec();
        }
    }
    if group.extra.len() < 4 {
        group
            .extra
            .push((db.to_string(), noun.to_string(), attr.to_string(), refno, words.to_vec()));
    }
}

fn note_scom_sets(element: &DbElement, scom: RefNo, by_set: &mut HashMap<RefNo, Vec<RefNo>>) {
    for name in ["GMRE", "PTRE", "DTRE"] {
        if let Ok(Some(target)) = element.get_element(name) {
            by_set.entry(target.refno()).or_default().push(scom);
        }
    }
}

fn follow_to_scom(mut current: DbElement) -> Option<RefNo> {
    let mut seen = HashSet::new();
    for _ in 0..8 {
        if !seen.insert(current.refno()) {
            return None;
        }
        let noun = current.element_type().unwrap_or_default();
        if noun.eq_ignore_ascii_case("SCOM") {
            return Some(current.refno());
        }
        let next = match current.get_element("CATR") {
            Ok(Some(next)) => Some(next),
            _ => current.get_element("PRTREF").ok().flatten(),
        }?;
        current = next;
    }
    None
}

fn scoms_for(
    element: RefNo,
    owners: &HashMap<RefNo, RefNo>,
    by_set: &HashMap<RefNo, Vec<RefNo>>,
) -> Vec<RefNo> {
    let mut found: Vec<RefNo> = Vec::new();
    let mut current = Some(element);
    let mut seen = HashSet::new();
    for _ in 0..12 {
        let Some(refno) = current else { break };
        if !seen.insert(refno) {
            break;
        }
        if let Some(scoms) = by_set.get(&refno) {
            found.extend(scoms.iter().copied());
        }
        current = owners.get(&refno).copied().filter(|owner| owner.is_valid());
    }
    found.sort_by_key(|refno: &RefNo| (refno.word0, refno.word1));
    found.dedup();
    found
}

fn scan_refnos(set: &DbSet, file: &Path, dbno: u32) -> anyhow::Result<Vec<RefNo>> {
    let mut engine = match set.pinned_sesno(dbno) {
        Some(sesno) => ReadOnlyEngine::open_at(file, sesno),
        None => ReadOnlyEngine::open(file),
    }?;
    let mut refnos = Vec::new();
    for scanned in engine.scan_elements(ScanTier::Header)? {
        refnos.push(scanned?.refno);
    }
    Ok(refnos)
}

fn candidate_words(value: &ExplicitValue) -> Option<Vec<i32>> {
    match value {
        ExplicitValue::IntArray(words) if words.len() >= 2 => Some(words.clone()),
        ExplicitValue::RawWords(words)
            if words.len() >= 2
                && words[0] as usize == words.len()
                && matches!(words[1], 1 | 5 | 6) =>
        {
            Some(words.iter().map(|word| *word as i32).collect())
        }
        _ => None,
    }
}

fn attr_name(stored_hash: u32) -> String {
    let hash = stored_hash.wrapping_neg();
    lookup_system_name(hash)
        .map(str::to_string)
        .or_else(|| db1_dehash(hash))
        .map(|name| name.trim().to_string())
        .filter(|name| !name.is_empty())
        .unwrap_or_else(|| format!("0x{hash:08X}"))
}

fn file_label(path: &Path) -> String {
    path.file_name()
        .map(|name| name.to_string_lossy().into_owned())
        .unwrap_or_else(|| path.display().to_string())
}

fn fmt_words(words: &[i32]) -> String {
    const LIMIT: usize = 80;
    let shown = words.len().min(LIMIT);
    let mut text = words[..shown]
        .iter()
        .map(|word| word.to_string())
        .collect::<Vec<_>>()
        .join(" ");
    if words.len() > LIMIT {
        text.push_str(&format!(" …共{}词", words.len()));
    }
    text
}

fn fmt_words_full(words: &[i32]) -> String {
    words
        .iter()
        .map(|word| word.to_string())
        .collect::<Vec<_>>()
        .join(" ")
}

fn zone_of(element: &DbElement) -> String {
    let mut current = element.clone();
    let mut parts = Vec::new();
    for _ in 0..16 {
        let noun = current.element_type().unwrap_or_default();
        let name = current.stored_name().ok().flatten().unwrap_or_default();
        parts.push(format!("{noun} {} {name}", current.refno()));
        if noun == "WORLD" {
            break;
        }
        match current.owner() {
            Ok(Some(owner)) => current = owner,
            _ => break,
        }
    }
    parts.join(" < ")
}

/// `render` 已经拒绝。返回第一个对不上 `parse` 的操作码或形状，保证不是「未知」。
fn first_decline(words: &[i32]) -> String {
    let Some(program) = program_body(words) else {
        return "not-program".to_string();
    };
    let mut cursor = 0;
    while cursor < program.len() {
        let rest = &program[cursor..];
        match rest[0] {
            OP_PUSH_NUMBER => match consume_number(rest) {
                Ok(used) => cursor += used,
                Err(reason) => return reason,
            },
            OP_PUSH_TEXT => match consume_text(rest) {
                Ok(used) => cursor += used,
                Err(reason) => return reason,
            },
            OP_ATTRIBUTE => match consume_attribute(rest) {
                Ok(used) => cursor += used,
                Err(reason) => return reason,
            },
            op if SINGLE_WORD_OPS.contains(&op) => cursor += 1,
            op => return format!("opcode {op}"),
        }
    }
    "render/stack".to_string()
}

fn program_body(words: &[i32]) -> Option<&[i32]> {
    let (declared, rest) = words.split_first()?;
    if *declared as usize != words.len() {
        return None;
    }
    match rest.split_first() {
        Some((1, program)) if !program.is_empty() => Some(program),
        _ => None,
    }
}

fn number_decodes(first: i32, _second: i32, third: i32) -> bool {
    let _ = first;
    third & DOUBLE_FLAG != 0 || third.abs() < MAX_SCALED_EXPONENT
}

fn consume_number(words: &[i32]) -> Result<usize, String> {
    let shape = words.get(1).copied().unwrap_or(-1);
    let decoded = |first, second, third, used| {
        if number_decodes(first, second, third) {
            Ok(used)
        } else {
            Err("101/decode".to_string())
        }
    };
    match words {
        [OP_PUSH_NUMBER, NUMBER_SHAPE, first, second, third, 0, end, ..]
            if *end == 0 || *end == NUMBER_SHAPE =>
        {
            decoded(*first, *second, *third, 7)
        }
        [OP_PUSH_NUMBER, NUMBER_WITH_UNIT_SHAPE, first, second, third, 0, 11, 1, 1, 0, ..] => {
            decoded(*first, *second, *third, 10)
        }
        [OP_PUSH_NUMBER, NUMBER_TYPED_SHAPE, first, second, third, 0, 11, 1, ..] => {
            decoded(*first, *second, *third, 8)
        }
        _ => Err(format!("101/shape {shape}")),
    }
}

fn consume_text(words: &[i32]) -> Result<usize, String> {
    let [OP_PUSH_TEXT, count, rest @ ..] = words else {
        return Err("118/shape".to_string());
    };
    let count = usize::try_from(*count).map_err(|_| "118/shape".to_string())?;
    let chars = rest.get(..count).ok_or_else(|| "118/shape".to_string())?;
    if chars.iter().any(|word| u8::try_from(*word).is_err()) {
        return Err("118/char".to_string());
    }
    Ok(count + 2)
}

fn consume_attribute(words: &[i32]) -> Result<usize, String> {
    if let Some(result) = previous_element(words) {
        return result;
    }
    if let Some(result) = owner_chain(words) {
        return result;
    }
    plain_attribute(words)
}

fn name_ok(hash: i32) -> bool {
    let hash = hash as u32;
    lookup_system_name(hash)
        .map(str::to_string)
        .or_else(|| db1_dehash(hash))
        .is_some_and(|name| !name.trim().is_empty())
}

fn attribute_word_ok(word: i32) -> bool {
    name_ok(word) || u32::try_from(word).ok().is_some_and(|key| key > SYSTEM_HASH_MAX)
}

fn previous_element(words: &[i32]) -> Option<Result<usize, String>> {
    let [
        OP_ATTRIBUTE,
        ATTRIBUTE_SHAPE,
        name,
        1,
        1,
        0,
        OP_OWNER_CHAIN,
        3,
        CHAIN_OPEN,
        PREVIOUS_ELEMENT,
        ATTRIBUTE_TERMINATOR,
        ..,
    ] = words
    else {
        return None;
    };
    Some(if attribute_word_ok(*name) {
        Ok(11)
    } else {
        Err("106/name".to_string())
    })
}

fn owner_chain(words: &[i32]) -> Option<Result<usize, String>> {
    let [OP_ATTRIBUTE, ATTRIBUTE_SHAPE_OWNED, name, 1, _, 0, OP_OWNER_CHAIN, length, chain @ ..] =
        words
    else {
        return None;
    };
    let Ok(length) = usize::try_from(*length) else {
        return Some(Err("106/chain".to_string()));
    };
    let Some(chain) = chain.get(..length) else {
        return Some(Err("106/chain".to_string()));
    };
    let [CHAIN_OPEN, owners @ .., CHAIN_CLOSE, ATTRIBUTE_TERMINATOR] = chain else {
        return Some(Err("106/chain".to_string()));
    };
    if !attribute_word_ok(*name) {
        return Some(Err("106/name".to_string()));
    }
    let mut named = 0usize;
    for word in owners {
        if *word < NAME_FLOOR {
            continue;
        }
        if !name_ok(*word) {
            return Some(Err("106/chain-name".to_string()));
        }
        named += 1;
    }
    if named == 0 {
        return Some(Err("106/chain".to_string()));
    }
    Some(Ok(8 + length))
}

fn plain_attribute(words: &[i32]) -> Result<usize, String> {
    let [OP_ATTRIBUTE, ATTRIBUTE_SHAPE, name, first, second, third, end_first, end_second, ..] =
        words
    else {
        return Err(format!(
            "106/shape {}",
            words.get(1).copied().unwrap_or(-1)
        ));
    };
    if [*end_first, *end_second] != [1601, 1701] {
        return Err("106/end".to_string());
    }
    if !attribute_word_ok(*name) {
        return Err("106/name".to_string());
    }
    match ([*first, *second], *third) {
        ([-1, -1], 0) | ([1, 1], 0) => Ok(8),
        ([1, 1], qualifier) => {
            if name_ok(qualifier) {
                Ok(8)
            } else {
                Err("106/qualifier".to_string())
            }
        }
        ([index, repeat], 0) if index == repeat && index >= 2 => Ok(8),
        _ => Err(format!("106/qual {first} {second} {third}")),
    }
}

/// `PTCD` 这一类方向规格里嵌着的 PML 程序。外层不是程序（第二词是 5 或 6）。
fn embedded_programs(words: &[i32]) -> Vec<(usize, Vec<i32>)> {
    if words.len() < 5 || words[0] as usize != words.len() {
        return Vec::new();
    }
    if words[1] != 5 && words[1] != 6 || words[2] != 2 {
        return Vec::new();
    }
    if words[3] == 52 {
        return component_programs(&words[4..]);
    }
    if words[3] < 21 {
        return Vec::new();
    }
    let turns = (words[3] - 21) as usize;
    let Some((&61, body)) = words[4..].split_last() else {
        return Vec::new();
    };
    let mut rest = body;
    if rest.is_empty() {
        return Vec::new();
    }
    rest = &rest[1..];
    let mut found = Vec::new();
    for _ in 0..turns {
        let Some((&length, tail)) = rest.split_first() else {
            break;
        };
        let Ok(length) = usize::try_from(length) else {
            break;
        };
        if length == 0 || tail.len() < length {
            break;
        }
        let program = tail[..length].to_vec();
        rest = &tail[length..];
        let Some((&_axis, tail)) = rest.split_first() else {
            break;
        };
        rest = tail;
        if is_program(&program) {
            found.push((found.len(), program));
        }
    }
    found
}

fn component_programs(body: &[i32]) -> Vec<(usize, Vec<i32>)> {
    let Some(rest) = body.strip_prefix(&[1, 33]) else {
        return Vec::new();
    };
    let Some(mut rest) = rest.strip_suffix(&[61, 71]) else {
        return Vec::new();
    };
    let mut found = Vec::new();
    while let Some((&_axis, tail)) = rest.split_first() {
        let Some((&length, tail)) = tail.split_first() else {
            break;
        };
        let Ok(length) = usize::try_from(length) else {
            break;
        };
        if length == 0 || tail.len() < length {
            break;
        }
        let program = tail[..length].to_vec();
        rest = &tail[length..];
        if is_program(&program) {
            found.push((found.len(), program));
        }
    }
    found
}

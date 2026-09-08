//! `RPRO TLEN` 到底该取哪个数：整库把每根 FTUB 的三个候选并排列出来。
//!
//! 目录里管身长度只写成 `PXLE = ( ATTRIB RPRO TLEN )`，而 `RPRO TLEN` 在库里
//! **没有存值**——e3d-io 现在拿设计实例的 `HEIG` 顶上（`db_param_env.rs`，那条
//! 推断自认只有一个样本）。ams8000 `/C-OR-1R310-K` 的 `FTUB 1` 打脸了它：
//! `HEIG=3300`，E3D 的 RVM 面片却只有 3000，而同一件的目录 `SCOM.PARA[3]`
//! 正好是 3000（`/ACP1000-VFVL-100` 的标准段长）。
//!
//! 一个样本判不了案，所以本探针整库量三样并列：
//!
//! - `HEIG`：现在在用的那个数；
//! - `SCOM.PARA[3]`：目录标准段长，怀疑它是上限；
//! - **到下一件成员的实际间距**：同属主下按成员序取下一件，两件 `POS` 在同一个
//!   属主系里，直接相减即可——不必上世界系。E3D 把下一件摆在本件离开点上，
//!   所以这个间距就是 E3D 自己认的 `TLEN`，是本探针里唯一的第三方证词。
//!
//! 判据看最后那张表：`HEIG > PARA[3]` 的那些行里，间距贴着谁，`TLEN` 就是谁。
//!
//! ```text
//! cargo run --release --example ftub_tlen_probe -- --attlib <attlib.dat> \
//!     --db <ams8000_0001> --template <dir> [--db-type DESI] [--sesno 264] \
//!     [--project-dir <dir> ...] [--catalogue <file> ...] [--rows 40]
//! ```

use std::collections::HashSet;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use anyhow::{Context, bail};
use e3d_io::db_element::{DbElement, DbFilePin, DbSet, template_file_for};
use e3d_io::engine::{ReadOnlyEngine, ScanTier};
use e3d_io::record::descriptor::{DescriptorValue, stored_expression_words};
use e3d_io::refno::RefNo;
use e3d_model::db_discovery::DirectoryDbResolver;

/// 判「贴着」的容差：目录长度都是整毫米，0.5 mm 足够分开两个候选。
const SAME_MM: f64 = 0.5;

struct Row {
    refno: RefNo,
    heig: f64,
    para3: Option<f64>,
    gap: Option<f64>,
}

fn main() -> anyhow::Result<()> {
    let mut attlib: Option<PathBuf> = None;
    let mut db_file: Option<PathBuf> = None;
    let mut template: Option<PathBuf> = None;
    let mut db_type = "DESI".to_string();
    let mut sesno: Option<u32> = None;
    let mut catalogues: Vec<PathBuf> = Vec::new();
    let mut project_dirs: Vec<PathBuf> = Vec::new();
    let mut rows_shown = 40usize;
    let mut dtse_of: Option<RefNo> = None;

    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--attlib" => attlib = Some(PathBuf::from(value()?)),
            "--db" => db_file = Some(PathBuf::from(value()?)),
            "--template" => template = Some(PathBuf::from(value()?)),
            "--db-type" => db_type = value()?,
            "--sesno" => sesno = Some(value()?.parse().context("--sesno 要无符号整数")?),
            "--catalogue" => catalogues.push(PathBuf::from(value()?)),
            "--project-dir" => project_dirs.push(PathBuf::from(value()?)),
            "--rows" => rows_shown = value()?.parse().context("--rows 要无符号整数")?,
            "--dtse-of" => {
                let text = value()?;
                let (w0, w1) = text
                    .trim_start_matches('=')
                    .split_once('/')
                    .with_context(|| format!("--dtse-of 要 w0/w1，得到 {text}"))?;
                dtse_of = Some(RefNo::new(w0.trim().parse()?, w1.trim().parse()?));
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
        sesno,
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
    let pinned = set.pinned_sesno(dbno);
    println!("库 {} dbno={dbno} 钉在会话 {pinned:?}", db_file.display());

    if let Some(refno) = dtse_of {
        dump_dtse(&set.element(refno))?;
        return Ok(());
    }

    let mut engine = match pinned {
        Some(session) => ReadOnlyEngine::open_at(&db_file, session),
        None => ReadOnlyEngine::open(&db_file),
    }?;
    let mut refnos = Vec::new();
    for scanned in engine.scan_elements(ScanTier::Header)? {
        refnos.push(scanned?.refno);
    }
    drop(engine);
    println!("索引元素 {}", refnos.len());

    let mut rows = Vec::new();
    let (mut no_heig, mut no_para3, mut no_gap) = (0usize, 0usize, 0usize);
    for refno in refnos {
        let element = set.element(refno);
        if !matches!(element.element_type().as_deref(), Ok("FTUB")) {
            continue;
        }
        let Some(heig) = real(&element, "HEIG") else {
            no_heig += 1;
            continue;
        };
        let para3 =
            scom_of(&element)
                .ok()
                .flatten()
                .and_then(|scom| match scom.attribute("PARA") {
                    Ok(Some(DescriptorValue::RealArray(values))) => values.get(2).copied(),
                    _ => None,
                });
        if para3.is_none() {
            no_para3 += 1;
        }
        let gap = gap_to_next_sibling(&element);
        if gap.is_none() {
            no_gap += 1;
        }
        rows.push(Row {
            refno,
            heig,
            para3,
            gap,
        });
    }
    println!(
        "FTUB {} 根（无 HEIG {no_heig} 根未计）；取不到 SCOM.PARA[3] {no_para3} 根，取不到下一件间距 {no_gap} 根",
        rows.len()
    );

    // 全体：间距贴 HEIG 还是贴 min(HEIG, PARA[3])。两者相等的行不参与裁决，
    // 单独计数——它们对两个假说一视同仁，混进去只会把票数一起抬高。
    let (mut undecidable, mut like_heig, mut like_cap, mut neither) =
        (0usize, 0usize, 0usize, 0usize);
    let mut over_cap = Vec::new();
    for row in &rows {
        let (Some(para3), Some(gap)) = (row.para3, row.gap) else {
            continue;
        };
        let cap = row.heig.min(para3);
        if (row.heig - cap).abs() <= SAME_MM {
            undecidable += 1;
            continue;
        }
        over_cap.push(row);
        match (
            (gap - row.heig).abs() <= SAME_MM,
            (gap - cap).abs() <= SAME_MM,
        ) {
            (true, _) => like_heig += 1,
            (_, true) => like_cap += 1,
            _ => neither += 1,
        }
    }
    println!(
        "HEIG ≤ PARA[3]（两个假说同解，不裁决）{undecidable} 根；HEIG > PARA[3] {} 根，其中间距贴 HEIG {like_heig} / 贴 PARA[3] {like_cap} / 两个都不贴 {neither}",
        over_cap.len()
    );

    if !over_cap.is_empty() {
        println!(
            "\n{:>16}  {:>10}  {:>10}  {:>12}",
            "refno", "HEIG", "PARA[3]", "到下一件"
        );
        for row in over_cap.iter().take(rows_shown) {
            println!(
                "{:>16}  {:>10.2}  {:>10.2}  {:>12}",
                row.refno.to_string(),
                row.heig,
                row.para3.unwrap_or(f64::NAN),
                row.gap
                    .map(|gap| format!("{gap:.2}"))
                    .unwrap_or_else(|| "-".to_string())
            );
        }
        if over_cap.len() > rows_shown {
            println!("… 还有 {} 行", over_cap.len() - rows_shown);
        }
    }
    Ok(())
}

/// 目录件 `DTRE → DTSE` 下每一行 `DATA` 的 `DKEY` / `PURP` 与 `PPRO` / `DPRO` 的**原始
/// PML 词流**。
///
/// 光看 E3D 渲染出来的 `( MIN ( ATTRIB HEIG , ATTRIB PARA[3 ] ) )` 不够——要给
/// `catalogue_pml` 补操作码，得知道 `MIN` 在词流里是哪个数。
fn dump_dtse(catalogue: &DbElement) -> anyhow::Result<()> {
    let Some(table) = catalogue.get_element("DTRE")? else {
        bail!("{} 没有 DTRE", catalogue.refno());
    };
    println!("DTSE {} {:?}", table.refno(), table.stored_name()?);
    for row in table.members()? {
        let key = match row.attribute("DKEY") {
            Ok(Some(DescriptorValue::Word { text, .. })) => text.unwrap_or_default(),
            _ => String::new(),
        };
        let purpose = match row.attribute("PURP") {
            Ok(Some(DescriptorValue::Word { text, .. })) => text.unwrap_or_default(),
            _ => String::new(),
        };
        println!("  {} DKEY={key:<6} PURP={purpose:<6}", row.refno());
        let parsed = row.parsed()?;
        for name in ["PPRO", "DPRO"] {
            if let Ok(Some(value)) = row.attribute(name) {
                println!("     {name} = {value:?}");
            }
            if let Some(words) = stored_expression_words(&parsed, e3d_attlib::db1_hash(name)) {
                println!("     {name} 词 {words:?}");
            }
        }
    }
    Ok(())
}

fn real(element: &DbElement, name: &str) -> Option<f64> {
    match element.attribute(name) {
        Ok(Some(DescriptorValue::Real(value))) => Some(value),
        _ => None,
    }
}

fn position(element: &DbElement) -> Option<[f64; 3]> {
    match element.attribute("POS") {
        Ok(Some(DescriptorValue::RealArray(values))) if values.len() >= 3 => {
            Some([values[0], values[1], values[2]])
        }
        _ => None,
    }
}

/// `SPRE → (CATR|PRTREF)* → SCOM`，与 `catalogue_probe::follow_to_scom` 同一条链。
fn scom_of(member: &DbElement) -> anyhow::Result<Option<DbElement>> {
    let Some(mut current) = member.get_element("SPRE")? else {
        return Ok(None);
    };
    let mut seen = HashSet::new();
    for _ in 0..8 {
        if !seen.insert(current.refno()) {
            return Ok(None);
        }
        if current.element_type()?.eq_ignore_ascii_case("SCOM") {
            return Ok(Some(current));
        }
        let next = match current.get_element("CATR") {
            Ok(Some(next)) => Some(next),
            _ => current.get_element("PRTREF").unwrap_or_default(),
        };
        let Some(next) = next else {
            return Ok(None);
        };
        current = next;
    }
    Ok(None)
}

/// 同属主下、成员序上紧挨着的下一件与本件的 `POS` 距离。
///
/// 两件的 `POS` 都在属主系里，相减即可；本件是属主最后一件时给 `None`。
fn gap_to_next_sibling(element: &DbElement) -> Option<f64> {
    let here = position(element)?;
    let owner = element.owner().ok()??;
    let mut found = false;
    for sibling in owner.members().ok()? {
        if found {
            let there = position(&sibling)?;
            return Some(
                (0..3)
                    .map(|axis| (there[axis] - here[axis]).powi(2))
                    .sum::<f64>()
                    .sqrt(),
            );
        }
        found = sibling.refno() == element.refno();
    }
    None
}

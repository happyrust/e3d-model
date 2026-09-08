//! 临时探针（用完即删）：列出 SYS 库里的全部 MDB 及其 CURD 成员的 DBNO / STYP / NAME。
use std::path::PathBuf;
use std::sync::Arc;

use e3d_io::db_element::{DbFilePin, DbSet, template_file_for};
use e3d_io::engine::{ReadOnlyEngine, ScanTier};

fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let tpl = PathBuf::from(&args[0]);
    let sys = PathBuf::from(&args[1]);
    let wanted_dbno: Option<u32> = args.get(2).and_then(|s| s.parse().ok());

    let set = Arc::new(DbSet::with_attlib_file(tpl.join("attlib.dat"))?);
    let dbno = set.add_db(DbFilePin {
        file: sys.clone(),
        template: template_file_for(&tpl, "SYST")?,
        db_type: Some("SYST".into()),
        sesno: None,
    })?;
    let mut engine = match set.pinned_sesno(dbno) {
        Some(sesno) => ReadOnlyEngine::open_at(&sys, sesno),
        None => ReadOnlyEngine::open(&sys),
    }?;
    for scanned in engine.scan_elements(ScanTier::Header)? {
        let scanned = scanned?;
        let element = set.element(scanned.refno);
        if args.get(4).is_some() {
            // 第 5 个参数任意值：列出全部 DB 元素里 DBNO >= 100000 的。
            if element.element_type()? == "DB" {
                let dbno = format!("{:?}", element.attribute("DBNO")?);
                let n: i64 = dbno
                    .trim_start_matches(|c: char| !c.is_ascii_digit())
                    .chars()
                    .take_while(|c| c.is_ascii_digit())
                    .collect::<String>()
                    .parse()
                    .unwrap_or(-1);
                if n >= 100000 {
                    println!(
                        "DB {} dbno={n} name={} STYP={:?} PROJ={:?}",
                        element.refno(),
                        element.stored_name()?.unwrap_or_default(),
                        element.attribute("STYP")?,
                        element.attribute("PROJ")?
                    );
                }
            }
            continue;
        }
        if element.element_type()? != "MDB" {
            continue;
        }
        let name = element.stored_name()?.unwrap_or_default();
        let curd = element.get_element_array("CURD")?;
        let mut rows = Vec::new();
        let mut hit = false;
        for db in &curd {
            let raw = |name: &str| -> String {
                match db.attribute(name) {
                    Ok(Some(value)) => format!("{value:?}"),
                    _ => "?".to_string(),
                }
            };
            let dbno_text = raw("DBNO");
            let dbno: i64 = dbno_text
                .trim_start_matches(|c: char| !c.is_ascii_digit())
                .chars()
                .take_while(|c| c.is_ascii_digit())
                .collect::<String>()
                .parse()
                .unwrap_or(-1);
            let styp = raw("STYP");
            let dbname = db.stored_name()?.unwrap_or_default();
            if wanted_dbno.is_some_and(|w| w as i64 == dbno) {
                hit = true;
            }
            rows.push(format!("{dbno}:{styp}:{dbname}"));
        }
        let name_filter = args.get(3).map(|s| s.as_str());
        if (wanted_dbno.is_none() || hit) && name_filter.is_none_or(|f| name == f) {
            println!("MDB {name}  ({} dbs)", curd.len());
            if wanted_dbno.is_some() || name_filter.is_some() {
                for r in rows {
                    println!("    {r}");
                }
            }
        }
    }
    Ok(())
}

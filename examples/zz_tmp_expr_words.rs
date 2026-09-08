//! 临时探针（用完即删）：打印一个元素显式流里的公式词与各属性原值。
use std::path::PathBuf;
use std::sync::Arc;

use e3d_io::db_element::{DbFilePin, DbSet, template_file_for};
use e3d_io::refno::RefNo;
use e3d_model::db_discovery::DirectoryDbResolver;

fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let tpl = PathBuf::from(&args[0]);
    let db = PathBuf::from(&args[1]);
    let dirs: Vec<PathBuf> = args[2].split(';').map(PathBuf::from).collect();
    let resolver = DirectoryDbResolver::scan(&dirs, &tpl)?;
    let set = Arc::new(DbSet::with_attlib_file_and_resolver(
        tpl.join("attlib.dat"),
        Box::new(resolver),
    )?);
    set.add_db(DbFilePin {
        file: db,
        template: template_file_for(&tpl, "DESI")?,
        db_type: Some("DESI".into()),
        sesno: None,
    })?;
    for text in &args[3..] {
        let (w0, w1) = text.split_once('/').unwrap();
        let el = set.element(RefNo::new(w0.parse()?, w1.parse()?));
        println!("=== {} {:?}", el.refno(), el.element_type());
        let parsed = el.parsed()?;
        for a in &parsed.explicit_attributes {
            println!(
                "  explicit hash={} (neg={}) type={} words={} uda={} value={:?}",
                a.hash,
                a.hash.wrapping_neg(),
                a.type_code,
                a.data_words,
                a.is_uda,
                a.value
            );
        }
        for name in el.attribute_names()? {
            if let Ok(Some(v)) = el.attribute(&name) {
                println!("  {name} = {v:?}");
            }
        }
        for m in el.members()? {
            let mut line = Vec::new();
            for name in m.attribute_names()? {
                if let Ok(Some(v)) = m.attribute(&name) {
                    line.push(format!("{name}={v:?}"));
                }
            }
            println!(
                "    member {} {:?}: {}",
                m.refno(),
                m.element_type(),
                line.join("  ")
            );
        }
    }
    Ok(())
}

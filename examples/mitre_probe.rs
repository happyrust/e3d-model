//! 斜接弯头普查：数一个设计库里 BEND 的 `ACTNCU` 分布，把命中 `IsMitredBend`
//! （`noun == BEND && ACTNCU != 0`，Core3D `0x105773d0`）的弯头逐个点名。
//!
//! `catalogue_point` 的斜接分支（管身端点取构件自身原点、不取目录 leave/arrive
//! P 点，抄 `isValidTubeLength@0x1057be50` 的端点选取）改的就是这批件；这支探针
//! 回答「这批件」在给定语料里有多少、都挂在哪条链上——语料里一只都没有时，
//! 那条分支就只有反编译背书，RVM 对拍（G0-2）要另找带斜接弯头的库。
//!
//! ```text
//! cargo run --example mitre_probe -- --attlib <attlib.dat> --db <ams8000_0001> \
//!     --template <dir|desvir.dat> [--db-type DESI] [--sesno N]
//! ```

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::Arc;

use anyhow::{Context, bail};
use e3d_io::db_element::{DbElementError, DbFilePin, DbSet, template_file_for};
use e3d_io::engine::{ReadOnlyEngine, ScanTier};
use e3d_io::record::descriptor::DescriptorValue;

fn main() -> anyhow::Result<()> {
    let mut attlib: Option<PathBuf> = None;
    let mut db_file: Option<PathBuf> = None;
    let mut template: Option<PathBuf> = None;
    let mut db_type = "DESI".to_string();
    let mut sesno: Option<u32> = None;

    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--attlib" => attlib = Some(PathBuf::from(value()?)),
            "--db" => db_file = Some(PathBuf::from(value()?)),
            "--template" => template = Some(PathBuf::from(value()?)),
            "--db-type" => db_type = value()?,
            "--sesno" => sesno = Some(value()?.parse().context("--sesno 要无符号整数")?),
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

    let set = Arc::new(DbSet::with_attlib_file(&attlib)?);
    set.add_db(DbFilePin {
        file: db_file.clone(),
        template,
        db_type: Some(db_type),
        sesno,
    })?;

    let mut engine = match sesno {
        Some(s) => ReadOnlyEngine::open_at(&db_file, s),
        None => ReadOnlyEngine::open(&db_file),
    }
    .with_context(|| format!("开库扫 BEND {}", db_file.display()))?;

    let mut bends = 0usize;
    let mut histogram: BTreeMap<i64, usize> = BTreeMap::new();
    let mut dictionary_missing = 0usize;
    let mut odd_shapes = 0usize;
    let mut mitred: Vec<(String, i64, String)> = Vec::new();

    for scanned in engine.scan_elements(ScanTier::Header)? {
        let refno = scanned?.refno;
        let el = set.element(refno);
        if el.element_type()? != "BEND" {
            continue;
        }
        bends += 1;
        // ACTNCU 是伪属性（DB_PseudoACTNCU → DB_ImplicitBend）：存储侧的来源是
        // NCUTS（getAtt 读得出才算定义），CPAR[3] >= 0 时覆盖，最后钳到 [0,25]。
        let ncuts = match el.attribute("NCUTS") {
            Ok(Some(DescriptorValue::Word { raw, .. })) => Some(i64::from(raw)),
            Ok(Some(DescriptorValue::Int(number))) => Some(i64::from(number)),
            Ok(None) => None,
            Ok(Some(other)) => {
                println!("BEND {refno} 的 NCUTS 形状意外：{other:?}");
                odd_shapes += 1;
                continue;
            }
            Err(DbElementError::UnknownAttribute { .. }) => {
                dictionary_missing += 1;
                continue;
            }
            Err(error) => return Err(error.into()),
        };
        let cpar = match el.attribute("CPAR") {
            Ok(value) => value,
            Err(DbElementError::UnknownAttribute { .. }) => None,
            Err(error) => return Err(error.into()),
        };
        if let Some(cpar) = &cpar {
            println!("BEND {refno} 有 CPAR：{cpar:?}");
        }
        let Some(cuts) = ncuts else {
            // NCUTS 没写：ACTNCU 未定义，getInt 缺省 0，不是斜接。
            *histogram.entry(-1).or_insert(0) += 1;
            continue;
        };
        *histogram.entry(cuts).or_insert(0) += 1;
        if cuts != 0 {
            let owner = match el.owner()? {
                Some(owner) => format!(
                    "{} {}",
                    owner.element_type().unwrap_or_else(|_| "?".into()),
                    owner.refno()
                ),
                None => "（无属主）".into(),
            };
            mitred.push((refno.to_string(), cuts, owner));
        }
    }

    println!("BEND 总数: {bends}");
    println!("NCUTS 直方图（-1 = 属性没写；值 → 件数）: {histogram:?}");
    if dictionary_missing > 0 {
        println!("字典不识 NCUTS 的 BEND: {dictionary_missing}（属性名或字典版本要再查）");
    }
    if odd_shapes > 0 {
        println!("ACTNCU 形状意外的 BEND: {odd_shapes}");
    }
    println!("斜接弯头（ACTNCU != 0）: {} 件", mitred.len());
    for (refno, cuts, owner) in mitred.iter().take(40) {
        println!("  BEND {refno} ACTNCU={cuts} 属主 {owner}");
    }
    if mitred.len() > 40 {
        println!("  …（其余 {} 件略）", mitred.len() - 40);
    }
    Ok(())
}

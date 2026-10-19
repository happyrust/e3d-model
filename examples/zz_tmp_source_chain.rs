//! 临时探针（用完即删）：对指定设计元素走 SPRE→CATR 目录链，dump 每一跳与
//! 组件里的负体候选成员（N* noun），验证 skipped J-C/M 判定是否真无源。
//!
//! 用法：zz_tmp_source_chain <plant-dir> <acp-dir> <template-dir> <db-file> <refno> [<refno>...]
use std::path::PathBuf;
use std::sync::Arc;

use e3d_io::db_element::{DbElement, DbFilePin, DbSet, template_file_for};
use e3d_io::record::descriptor::DescriptorValue;
use e3d_io::refno::RefNo;

fn parse_refno(s: &str) -> anyhow::Result<RefNo> {
    let (a, b) = s
        .split_once('/')
        .ok_or_else(|| anyhow::anyhow!("bad refno {s}"))?;
    Ok(RefNo::new(a.parse()?, b.parse()?))
}

fn describe(el: &DbElement) -> String {
    let noun = el.element_type().unwrap_or_else(|_| "?".into());
    let name = el.stored_name().ok().flatten().unwrap_or_default();
    format!("{} {noun} {name}", el.refno())
}

fn follow(el: &DbElement, attr: &str) -> Option<DbElement> {
    match el.get_element(attr) {
        Ok(v) => v,
        Err(e) => {
            println!("    {attr} -> Err({e})");
            None
        }
    }
}

fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let [plant, acp, template, db_file] = &args[..4] else {
        return Err(anyhow::anyhow!(
            "usage: zz_tmp_source_chain <plant-dir> <acp-dir> <template-dir> <db-file> <refno>..."
        ));
    };
    let template = PathBuf::from(template);
    let resolver =
        e3d_model::db_discovery::DirectoryDbResolver::scan(&[plant.into(), acp.into()], &template)?;
    let set = Arc::new(DbSet::with_attlib_file_and_resolver(
        template.join("attlib.dat"),
        Box::new(resolver),
    )?);
    set.add_db(DbFilePin {
        file: db_file.into(),
        template: template_file_for(&template, "DESI")?,
        db_type: Some("DESI".into()),
        sesno: None,
    })?;

    for arg in args.iter().skip(4) {
        if let Some(spec) = arg.strip_prefix("name:") {
            let (dbno, name) = spec
                .split_once(':')
                .ok_or_else(|| anyhow::anyhow!("name: 要 name:<dbno>:<name>"))?;
            let hits = set.find_named(dbno.parse()?, name)?;
            println!("name {name} in db {dbno} -> {hits:?}");
            continue;
        }
        let refno = parse_refno(arg)?;
        let el = set.element(refno);
        println!("=== {}", describe(&el));
        let mut cur = el.clone();
        while let Ok(Some(owner)) = cur.owner() {
            println!("  owner: {}", describe(&owner));
            cur = owner;
        }
        let tmpl: Vec<String> = el
            .members_of_type("TMPL")
            .unwrap_or_default()
            .iter()
            .map(describe)
            .collect();
        println!("  TMPL members: {tmpl:?}");
        let all_members: Vec<String> = el
            .members()
            .map(|c| c.map(|m| describe(&m)).collect())
            .unwrap_or_default();
        println!("  all members: {all_members:?}");
        for attr in [
            "CUTB", "DRNS", "DRNE", "JSTX", "JUSL", "POSS", "POSE", "POS", "ORIE", "ORI", "BANG",
            "DELP", "DESP", "ARRI", "LEAV", "JOIS", "JOIE", "CREF", "CTRF", "CTRA", "PX", "PY",
            "PZ", "PHEI", "PWID", "PBDM", "PBOR", "POSL", "ZDIS", "PXGE", "PYGE", "PZGE", "PDIA",
            "PRAD", "PTYP", "POWN", "GTFM", "PKEY", "DX", "DY", "CPAR", "PARA", "NAME", "PURP",
            "NUMB", "PXLE", "PYLE", "PZLE", "PXDI", "PYDI", "PZDI", "PAXI", "LEVE", "OBST", "TUFL",
            "PRFL", "GMRE", "NGMR", "NAPP", "CATR", "PRTREF", "PTRE", "PBOF", "POFF", "PBTP",
            "PCTP", "PDIS",
        ] {
            match el.attribute(attr) {
                Ok(Some(v)) => println!("  {attr} = {v:?}"),
                Ok(None) => {}
                Err(e) => println!("  {attr} -> Err({e})"),
            }
        }
        let Some(spre) = follow(&el, "SPRE") else {
            println!("  SPRE: none -> no catalogue source");
            continue;
        };
        println!("  SPRE -> {}", describe(&spre));
        // 探针需要看清链上每一格：GMRE（组件终点判定）/ CATR / PRTREF / 成员表。
        for hop_attr in ["GMRE", "CATR", "PRTREF", "PTRE"] {
            match spre.get_element(hop_attr) {
                Ok(Some(target)) => println!("    {hop_attr} -> {}", describe(&target)),
                Ok(None) => println!("    {hop_attr}: unset"),
                Err(e) => println!("    {hop_attr} -> Err({e})"),
            }
        }
        let spre_members: Vec<String> = spre
            .members()
            .map(|c| c.map(|m| describe(&m)).collect())
            .unwrap_or_default();
        println!("    SPRE members: {spre_members:?}");
        let spre_type = spre.element_type().unwrap_or_default();
        let has_gmre = spre.get_element("GMRE").ok().flatten().is_some();
        let component = if spre_type == "SCOM" || has_gmre {
            Some(spre.clone())
        } else {
            match spre.get_element("CATR") {
                Ok(v) => v,
                Err(e) => {
                    println!("    CATR -> Err({e})");
                    None
                }
            }
        };
        let Some(comp) = component else {
            println!("  component: none");
            continue;
        };
        println!("  component: {}", describe(&comp));
        let members: Vec<DbElement> = comp.members()?.collect();
        for member in &members {
            let noun = member.element_type().unwrap_or_default();
            if noun.starts_with('N') || noun == "GMRE" || noun == "GMSE" || noun == "SDTE" {
                println!("    member {}", describe(member));
            }
        }
        // GMRE/GMSE 深一层。
        for member in &members {
            let noun = member.element_type().unwrap_or_default();
            if noun == "GMRE" || noun == "GMSE" {
                for sub in member.members()? {
                    println!("      gm member {}", describe(&sub));
                }
            }
        }
        if let Ok(Some(DescriptorValue::RefNoArray(refs))) = comp.attribute("CATR") {
            println!("    comp CATR refs: {refs:?}");
        }
    }
    Ok(())
}

//! 管子件外径取证（RG）：验证 CGETOD 的 OD 来源链，在真库上钉死后再落 G1。
//!
//! 本轮 ida-bridge 反编译（core.dll `GATREA` 0x5a36580、Core3D `GATCAT` 0x1035c340、
//! `GATCRF` 0x1035d7d8）解出的链：
//!
//! ```text
//! 离开侧设计元素 --stub--> [若 TYPE==TABITE 再 .PRTREF] --> 管子件
//!   stub = LSTU (IPCOMP 管件成员) / HSTU (BRAN·TRUNNI 容器头)
//!        / HSRO (其它路由容器) / LSRO (其它)
//!   OD   = 管子件.PARA[2]          （PARA 直读，1 起下标；保温再加 IPAR[1]）
//!   GMRE = 管子件.CATR.GMRE        （几何集另走一跳 CATR）
//! ```
//!
//! 这支探针不改任何生成路径，只回答三个计数题：
//! 1. 每一类离开侧元素，stub 引用在不在、解不解得通、落到哪个 noun。
//! 2. 解通的管子件有没有 `PARA`，`PARA[2]` 是多少，与该端 P 点通径 `PBOR` 差多少。
//! 3. 管子件带几何集（`CATR.GMRE`）的占比 —— G2 该不该做的依据。
//!
//! ```text
//! cargo run --release --example tube_od_probe -- --attlib <attlib.dat> \
//!     --template <shadow 目录> --design <ams8000_0001> \
//!     --catalogue <ams5052_0001> --catalogue <ams5053_0001> [--sample 6]
//! ```

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use anyhow::{Context, bail};
use e3d_io::db_element::{DbElement, DbElementError, DbFilePin, DbSet, template_file_for};
use e3d_io::engine::{ReadOnlyEngine, ScanTier};
use e3d_io::record::descriptor::DescriptorValue;
use e3d_io::refno::RefNo;
use e3d_model::category::{Category, classify};

const TABITE: &str = "TABITE";
/// TABITE 间接：`GATCRF` 里 `TYPE==TABITE` 时接着走的那一跳。
const TABITE_FOLLOW: &str = "PRTREF";
/// 几何集要在管子件上再走一跳 `CATR` 才到目录 SCOM。
const CATALOGUE_REF: &str = "CATR";

#[derive(Default)]
struct Tally {
    containers: usize,
    members: usize,
    /// `元素类 / stub 名` → 出现次数。
    stub_present: BTreeMap<String, usize>,
    /// `元素类 / stub 名 → 目标 noun` → 次数。
    stub_target: BTreeMap<String, usize>,
    /// 解通的管子件里 `PARA` 长度分布。
    para_len: BTreeMap<usize, usize>,
    /// 有 `PARA` 的管子件数、其中 `PARA[2]` 是有限正数的数。
    with_para: usize,
    with_od: usize,
    /// SPCO 自身（= `v65.TUBI` 目标）直接带 PARA / GMRE 的数——决定 E3D 用 OD 还是 bore。
    spco_has_para: usize,
    spco_has_gmre: usize,
    /// 管子件带几何集（`SCOM.GMRE` 命中）的数。
    with_gmre: usize,
    /// 几何集成员的 noun → 次数（G2 到底要认哪几种 TYPE）。
    gmre_member_types: BTreeMap<String, usize>,
    /// 跨库缺口。
    missing_dbs: BTreeMap<u32, usize>,
}

fn main() -> anyhow::Result<()> {
    let mut attlib: Option<PathBuf> = None;
    let mut template: Option<PathBuf> = None;
    let mut design: Option<PathBuf> = None;
    let mut catalogues: Vec<PathBuf> = Vec::new();
    let mut sample = 6usize;

    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--attlib" => attlib = Some(PathBuf::from(value()?)),
            "--template" | "--template-dir" => template = Some(PathBuf::from(value()?)),
            "--design" => design = Some(PathBuf::from(value()?)),
            "--catalogue" => catalogues.push(PathBuf::from(value()?)),
            "--sample" => sample = value()?.parse().context("--sample 要无符号整数")?,
            other => bail!("不认识的参数 {other}"),
        }
    }
    let attlib = attlib.context("缺 --attlib")?;
    let template = template.context("缺 --template")?;
    let design = design.context("缺 --design")?;

    let set = Arc::new(DbSet::with_attlib_file(&attlib)?);
    let design_dbno = set.add_db(DbFilePin {
        file: design.clone(),
        template: template_file_for(&template, "DESI")?,
        db_type: Some("DESI".to_string()),
        sesno: None,
    })?;
    println!("设计库 {} dbno={design_dbno}", design.display());
    for file in &catalogues {
        let dbno = set.add_db(DbFilePin {
            file: file.clone(),
            template: template_file_for(&template, "CATA")?,
            db_type: Some("CATA".to_string()),
            sesno: None,
        })?;
        println!("目录库 {} dbno={dbno}", file.display());
    }

    let mut tally = Tally::default();
    let mut shown = 0usize;
    for root in roots_of(&design)? {
        let mut stack = vec![set.element(root)];
        while let Some(el) = stack.pop() {
            let noun = el.element_type()?;
            if classify(&noun) == Category::RouteContainer {
                tally.containers += 1;
                let detail = shown < sample;
                probe_leave_side(&el, &noun, &mut tally, detail);
                if detail {
                    shown += 1;
                }
                for member in el.members()? {
                    if matches!(classify(&member.element_type()?), Category::RouteMember) {
                        tally.members += 1;
                        let detail = shown < sample;
                        probe_leave_side(&member, &member.element_type()?, &mut tally, detail);
                        if detail {
                            shown += 1;
                        }
                    }
                }
            }
            stack.extend(el.members()?);
        }
    }

    report(&tally);
    Ok(())
}

/// 一个离开侧元素（容器头，或链上一件构件）走 stub → 管子件，取 PARA / GMRE。
fn probe_leave_side(el: &DbElement, noun: &str, tally: &mut Tally, detail: bool) {
    let class = element_class(noun);
    // stub 的优先序照 GATCAT：容器头先 HSTU（BRAN/TRUNNI）/HSRO，构件先 LSTU/LSRO。
    let order: &[&str] = match class {
        "容器" => &["HSTU", "HSRO", "LSTU", "LSRO"],
        _ => &["LSTU", "LSRO", "HSTU", "HSRO"],
    };

    let mut resolved: Option<(String, DbElement)> = None;
    for stub in order {
        match el.get_element(stub) {
            Ok(Some(target)) => {
                *tally
                    .stub_present
                    .entry(format!("{class} / {stub}"))
                    .or_insert(0) += 1;
                if resolved.is_none() {
                    resolved = Some(((*stub).to_string(), target));
                }
            }
            Ok(None) => {}
            Err(error) => note_missing_db(&error, tally),
        }
    }

    let Some((stub, target)) = resolved else {
        if detail {
            println!("\n--- {} {noun}（{class}）: 四个 stub 都读不到", el.refno());
        }
        return;
    };

    // TABITE 间接：GATCRF 里 TYPE==TABITE 就接着走 PRTREF。
    let component = follow_tabite(target, tally);
    let comp_noun = component
        .as_ref()
        .and_then(|c| c.element_type().ok())
        .unwrap_or_else(|| "(读不出)".to_string());
    *tally
        .stub_target
        .entry(format!("{class} / {stub} → {comp_noun}"))
        .or_insert(0) += 1;

    let Some(spco) = component else {
        return;
    };

    // ★ 决定 G1 成立与否的岔口：GTTUBG/CGETOD 读的是 `v65.TUBI` 目标（= SPCO）身上的
    // GMRE / PARA。SPCO 自己有没有这两样，直接决定 E3D 走「几何集」还是「默认圆柱」、
    // 用「OD」还是「回落 bore」。在这里如实数一遍。
    if matches!(spco.attribute("PARA"), Ok(Some(_))) {
        tally.spco_has_para += 1;
    }
    if matches!(spco.get_element("GMRE"), Ok(Some(_))) {
        tally.spco_has_gmre += 1;
    }

    // stub → SPCO；真正的目录件（PARA/GMRE 都在它身上）在 SPCO.CATR 的 SCOM 上。
    let scom = spco
        .get_element(CATALOGUE_REF)
        .ok()
        .flatten()
        .and_then(|c| follow_tabite(c, tally));
    let scom_noun = scom
        .as_ref()
        .and_then(|c| c.element_type().ok())
        .unwrap_or_else(|| "(无 CATR)".to_string());

    let para = scom.as_ref().and_then(|scom| match scom.attribute("PARA") {
        Ok(Some(DescriptorValue::RealArray(items))) => Some(items),
        Ok(Some(DescriptorValue::IntArray(items))) => {
            Some(items.into_iter().map(f64::from).collect::<Vec<_>>())
        }
        Ok(_) | Err(_) => None,
    });

    if let Some(para) = &para {
        tally.with_para += 1;
        *tally.para_len.entry(para.len()).or_insert(0) += 1;
    }
    // OD = PARA[2]，1 起下标 → 0 起的 [1]。
    let od = para.as_ref().and_then(|p| p.get(1).copied());
    if od.is_some_and(|v| v.is_finite() && v > 0.0) {
        tally.with_od += 1;
    }

    // 几何集在 SCOM.GMRE 上（G2 占比 + 成员 TYPE）。
    let gmre = scom
        .as_ref()
        .and_then(|s| s.get_element("GMRE").ok().flatten());
    if gmre.is_some() {
        tally.with_gmre += 1;
    }
    if let Some(gmre) = &gmre
        && let Ok(members) = gmre.members()
    {
        for m in members {
            if let Ok(t) = m.element_type() {
                *tally.gmre_member_types.entry(t).or_insert(0) += 1;
            }
        }
    }

    if detail {
        println!(
            "\n--- {} {noun}（{class}）: stub={stub} → {comp_noun} --CATR--> {scom_noun}",
            el.refno()
        );
        println!("      PARA = {:?}", para);
        println!("      OD(PARA[2]) = {od:?}   GMRE={}", gmre.is_some());
        if let Some(gmre) = &gmre {
            let kinds: Vec<String> = gmre
                .members()
                .map(|ms| ms.filter_map(|m| m.element_type().ok()).collect())
                .unwrap_or_default();
            println!("      几何集成员 TYPE：{kinds:?}");
        }
    }
}

/// TABITE 间接一跳。目标不是 TABITE 就原样返回。
fn follow_tabite(target: DbElement, tally: &mut Tally) -> Option<DbElement> {
    match target.element_type() {
        Ok(noun) if noun.eq_ignore_ascii_case(TABITE) => match target.get_element(TABITE_FOLLOW) {
            Ok(inner) => inner,
            Err(error) => {
                note_missing_db(&error, tally);
                None
            }
        },
        Ok(_) => Some(target),
        Err(error) => {
            note_missing_db(&error, tally);
            None
        }
    }
}

fn element_class(noun: &str) -> &'static str {
    match classify(noun) {
        Category::RouteContainer => "容器",
        _ => "构件",
    }
}

fn note_missing_db(error: &DbElementError, tally: &mut Tally) {
    if let DbElementError::DbNotRegistered { dbno } = error {
        *tally.missing_dbs.entry(*dbno).or_insert(0) += 1;
    }
}

fn roots_of(db_file: &Path) -> anyhow::Result<Vec<RefNo>> {
    let mut engine =
        ReadOnlyEngine::open(db_file).with_context(|| format!("开库扫根 {}", db_file.display()))?;
    let mut owners = Vec::new();
    let mut indexed = std::collections::HashSet::new();
    for scanned in engine.scan_elements(ScanTier::Header)? {
        let scanned = scanned?;
        indexed.insert(scanned.refno);
        owners.push((scanned.refno, scanned.owner));
    }
    Ok(owners
        .iter()
        .filter(|(refno, owner)| owner == refno || !owner.is_valid() || !indexed.contains(owner))
        .map(|(refno, _)| *refno)
        .collect())
}

fn report(tally: &Tally) {
    println!("\n================ 汇总 ================");
    println!("路由容器 {}  路由成员 {}", tally.containers, tally.members);

    println!("\nstub 引用在不在（元素类 / stub 名 → 次数）：");
    for (key, count) in &tally.stub_present {
        println!("  {key:<20} {count:>6}");
    }

    println!("\nstub 解到哪个 noun：");
    for (key, count) in &tally.stub_target {
        println!("  {key:<36} {count:>6}");
    }

    println!(
        "\n★ SPCO(= v65.TUBI 目标)自身：直接带 PARA 的 {}，直接带 GMRE 的 {}",
        tally.spco_has_para, tally.spco_has_gmre
    );
    println!(
        "（对照）SPCO.CATR 的 SCOM：有 PARA 的 {}，PARA[2] 为有限正数的 {}，带 GMRE 的 {}",
        tally.with_para, tally.with_od, tally.with_gmre
    );
    println!("PARA 长度分布：");
    for (len, count) in &tally.para_len {
        println!("  len={len:<3} {count:>6}");
    }

    println!("\n几何集成员 TYPE（G2 要认哪几种）：");
    for (noun, count) in &tally.gmre_member_types {
        println!("  {noun:<8} {count:>6}");
    }

    if tally.missing_dbs.is_empty() {
        println!("\n跨库：没有未注册的库号。");
    } else {
        println!("\n未注册的库号（跨库缺口）：");
        for (dbno, count) in &tally.missing_dbs {
            println!("  db {dbno:<8} 被引用 {count:>6} 次");
        }
    }
}

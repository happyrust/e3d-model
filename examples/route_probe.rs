//! 路由链取证：管身两端点在**设计侧**到底读得到多少。
//!
//! P2 的四条几何公式（`algorithm-bran-hang.md` §3）都要两个 P 点，而两个 P 点
//! 有两种来源，可得性完全不同：
//!
//! - **容器自身属性**（`HPOS/HDIR/HBOR` 与 `TPOS/TDIR/TBOR`）—— 直读，现在就有。
//! - **构件的目录侧 P 点**（`LPOS/LDIR/LBOR` 与 `APOS/ADIR/ABOR`）—— 要走
//!   `SPRE → CATE/SCOM → GMSE` 那条目录链，即 P1 的前置。设计元素上的
//!   `ARRI`/`LEAV` 只是**P 点编号**，不是坐标。
//!
//! 所以「P2 能做多少」不是判断题而是计数题，这支探针就是去数它：整库有多少条
//! 管身槽位两端全落在容器属性上（现在就能出几何），多少条至少有一端要等目录链。
//! 不数就动手，写出来的东西会在 2612 个构件上静默退化成零长管。
//!
//! ```text
//! cargo run --release --example route_probe -- --attlib <attlib.dat> \
//!     --db <ams8000_0001> --template <目录|desvir.dat> [--sample 5]
//! ```

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use anyhow::{Context, bail};
use e3d_io::db_element::{DbElement, DbFilePin, DbSet, template_file_for};
use e3d_io::engine::{ReadOnlyEngine, ScanTier};
use e3d_io::refno::RefNo;
use e3d_model::category::{Category, classify};
use e3d_model::route::{
    TubeOutcome, TubeSection, build_tube, container_head, container_tail, tube_length,
    tube_outer_diameter,
};

/// 容器头/尾点的三件套属性名，与算法文档 §3.1 的 `IHEAD` / `TAIL` 常量区对齐。
const HEAD_ATTRS: [&str; 3] = ["HPOS", "HDIR", "HBOR"];
const TAIL_ATTRS: [&str; 3] = ["TPOS", "TDIR", "TBOR"];
/// 构件侧「离开 / 到达」P 点编号，以及通往目录的引用。
const MEMBER_ATTRS: [&str; 6] = ["LEAV", "ARRI", "SPRE", "CATR", "POS", "ORI"];

#[derive(Default)]
struct Tally {
    containers: BTreeMap<String, usize>,
    /// 容器 noun → 头尾三件套齐全的容器数。
    head_complete: BTreeMap<String, usize>,
    tail_complete: BTreeMap<String, usize>,
    /// 路由成员 noun → 数量。
    members: BTreeMap<String, usize>,
    /// 成员属性名 → 有值的成员数。
    member_attr_present: BTreeMap<String, usize>,
    /// 管身槽位：两端来源组合 → 条数。
    tube_ends: BTreeMap<&'static str, usize>,
    /// 空容器（无路由成员）的头尾管，两端全在容器属性上。
    resolvable_now: usize,
    tube_total: usize,
}

fn main() -> anyhow::Result<()> {
    let mut attlib: Option<PathBuf> = None;
    let mut db_file: Option<PathBuf> = None;
    let mut template: Option<PathBuf> = None;
    let mut db_type = "DESI".to_string();
    let mut sesno: Option<u32> = None;
    let mut sample = 3usize;

    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--attlib" => attlib = Some(PathBuf::from(value()?)),
            "--db" => db_file = Some(PathBuf::from(value()?)),
            "--template" => template = Some(PathBuf::from(value()?)),
            "--db-type" => db_type = value()?,
            "--sesno" => sesno = Some(value()?.parse().context("--sesno 要无符号整数")?),
            "--sample" => sample = value()?.parse().context("--sample 要无符号整数")?,
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
    let dbno = set.add_db(DbFilePin {
        file: db_file.clone(),
        template,
        db_type: Some(db_type),
        sesno,
    })?;
    println!(
        "库 {} dbno={dbno} 会话 {:?}",
        db_file.display(),
        set.pinned_sesno(dbno)
    );

    let mut tally = Tally::default();
    let mut shown = 0usize;
    for root in roots_of(&db_file, sesno)? {
        let mut stack = vec![set.element(root)];
        while let Some(el) = stack.pop() {
            let noun = el.element_type()?;
            if matches!(classify(&noun), Category::RouteContainer) {
                let detail = shown < sample;
                walk_container(&el, &noun, &mut tally, detail)?;
                if detail {
                    shown += 1;
                }
            }
            stack.extend(el.members()?);
        }
    }

    report(&tally);
    Ok(())
}

fn roots_of(db_file: &Path, sesno: Option<u32>) -> anyhow::Result<Vec<RefNo>> {
    let mut engine = match sesno {
        Some(s) => ReadOnlyEngine::open_at(db_file, s),
        None => ReadOnlyEngine::open(db_file),
    }
    .with_context(|| format!("开库扫根 {}", db_file.display()))?;
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

/// 有值才算「读得到」：属性在模板里但没存值，对几何来说与缺失等价。
fn present(el: &DbElement, name: &str) -> bool {
    matches!(el.attribute(name), Ok(Some(_)))
}

fn walk_container(
    container: &DbElement,
    noun: &str,
    tally: &mut Tally,
    detail: bool,
) -> anyhow::Result<()> {
    *tally.containers.entry(noun.to_string()).or_insert(0) += 1;
    let head_ok = HEAD_ATTRS.iter().all(|name| present(container, name));
    let tail_ok = TAIL_ATTRS.iter().all(|name| present(container, name));
    if head_ok {
        *tally.head_complete.entry(noun.to_string()).or_insert(0) += 1;
    }
    if tail_ok {
        *tally.tail_complete.entry(noun.to_string()).or_insert(0) += 1;
    }

    let mut route_members = Vec::new();
    for member in container.members()? {
        let member_noun = member.element_type()?;
        if !matches!(classify(&member_noun), Category::RouteMember) {
            continue;
        }
        *tally.members.entry(member_noun.clone()).or_insert(0) += 1;
        for name in MEMBER_ATTRS {
            if present(&member, name) {
                *tally
                    .member_attr_present
                    .entry(name.to_string())
                    .or_insert(0) += 1;
            }
        }
        route_members.push((member, member_noun));
    }

    // 槽位口径与 `pipeline::implied_tube_count` 一致：成员数 + 1（尾管）。
    // 两端来源按算法文档 §3.1 分：容器端走自身属性，构件端走目录链。
    let slots = route_members.len() + 1;
    tally.tube_total += slots;
    if route_members.is_empty() {
        *tally.tube_ends.entry("容器头 → 容器尾").or_insert(0) += 1;
        if head_ok && tail_ok {
            tally.resolvable_now += 1;
        }
    } else {
        *tally.tube_ends.entry("容器头 → 构件").or_insert(0) += 1;
        *tally.tube_ends.entry("构件 → 容器尾").or_insert(0) += 1;
        let between = slots.saturating_sub(2);
        if between > 0 {
            *tally.tube_ends.entry("构件 → 构件").or_insert(0) += between;
        }
    }

    // 空容器一律打印：它是眼下唯一两端都解得出的那一类，管身建不建得成全看它。
    if detail || route_members.is_empty() {
        println!(
            "\n--- {} {} {:?} 头齐={head_ok} 尾齐={tail_ok} 路由成员={}",
            container.refno(),
            noun,
            container.stored_name()?,
            route_members.len()
        );
        println!("    走一遍 build_tube: {}", probe_build(container)?);
        for name in HEAD_ATTRS.iter().chain(TAIL_ATTRS.iter()) {
            println!(
                "    {name} = {:?}",
                container.attribute(name).ok().flatten()
            );
        }
        for (member, member_noun) in route_members.iter().take(4) {
            let values = MEMBER_ATTRS
                .iter()
                .map(|name| format!("{name}={:?}", member.attribute(name).ok().flatten()))
                .collect::<Vec<_>>()
                .join("  ");
            println!("    成员 {} {member_noun} {values}", member.refno());
        }
    }
    Ok(())
}

/// 拿容器头尾两点真走一遍 [`build_tube`]，把卡在哪一步说出来。
///
/// 报告里 `unbuildable` 只是个计数，看不出是缺半径还是定不出朝向；而这两种
/// 病因在 P1 那边要修的地方完全不同。
fn probe_build(container: &DbElement) -> anyhow::Result<String> {
    let (Some(head), Some(tail)) = (container_head(container)?, container_tail(container)?) else {
        return Ok("头或尾解不出".to_string());
    };
    let diameter = tube_outer_diameter(None, None, tail.bore);
    let verdict = match build_tube(&head, &tail, TubeSection::round(diameter)) {
        TubeOutcome::Built(tube) => format!(
            "建成，长 {:.3}mm 半径 {:?}mm",
            tube.length_mm,
            tube.radius_mm()
        ),
        TubeOutcome::ZeroLength { length_mm } => format!("零长 {length_mm:.6}mm"),
        TubeOutcome::NoDirection => "定不出朝向".to_string(),
        TubeOutcome::NoRadius => format!("取不到半径（到达侧通径 {:?}）", tail.bore),
        TubeOutcome::Failed(error) => format!("内核未出体: {error}"),
    };
    Ok(format!(
        "{verdict}；头 bore={:?} 尾 bore={:?} 直线距离 {:.3}mm",
        head.bore,
        tail.bore,
        tube_length(head.pos, tail.pos)
    ))
}

fn report(tally: &Tally) {
    println!("\n================ 汇总 ================");
    println!("路由容器：");
    for (noun, count) in &tally.containers {
        println!(
            "  {noun:<8} {count:>6}   头三件套齐 {:>6}   尾三件套齐 {:>6}",
            tally.head_complete.get(noun).copied().unwrap_or(0),
            tally.tail_complete.get(noun).copied().unwrap_or(0),
        );
    }
    println!("路由成员（前 12 类）：");
    let mut members: Vec<_> = tally.members.iter().collect();
    members.sort_by(|a, b| b.1.cmp(a.1));
    for (noun, count) in members.iter().take(12) {
        println!("  {noun:<8} {count:>6}");
    }
    println!("成员属性有值数：");
    for (name, count) in &tally.member_attr_present {
        println!("  {name:<6} {count:>6}");
    }
    println!("管身槽位按两端来源：");
    for (kind, count) in &tally.tube_ends {
        println!("  {kind:<18} {count:>6}");
    }
    println!(
        "槽位合计 {}，其中两端全在容器属性上（现在就能出几何）{}，\
         其余 {} 条至少一端要等目录链（P1 前置）",
        tally.tube_total,
        tally.resolvable_now,
        tally.tube_total - tally.resolvable_now,
    );
}

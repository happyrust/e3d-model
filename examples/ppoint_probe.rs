//! 构件侧 P 点取证：`SPRE → SPCO.CATR → SCOM.PTRE → PTSE → PT**` 这条链
//! 到底在哪一跳断，断了多少条，以及活下来的那些把值存成了什么形状。
//!
//! P2 的 3171 条管身槽位卡在「构件的离开/到达 P 点读不到」。算法文档 §3.1 说
//! `LPOS/LDIR/LBOR` 与 `APOS/ADIR/ABOR` 是**目录侧**的值，设计元素上的
//! `LEAV`/`ARRI` 只是 P 点编号。要把它们解出来，得先知道三件事，而这三件事
//! 都是计数题不是判断题：
//!
//! 1. **链在哪一跳断。** 每一跳都可能 Nulref、跨到未注册的库、或目标 noun 不对。
//!    不数就写解析器，写出来的东西会在某一跳上静默退化成「取不到」，
//!    而报告只会说 `missing_ppoint`，和现在没接目录链时长得一模一样。
//! 2. **`PDIS`/`PBOR`/`PAXI` 存成什么。** 数、五词元组、还是显式流里的 PML 正文。
//!    形状决定要不要求值器，也决定求值器要吃哪一种。
//! 3. **表达式向环境问什么。** `PARAM n`、`ATTRIB PARA[i]`、`ATTRIB RPRO TLEN`
//!    问的不是同一个元素。`DbElementParamEnv` 的四个方法各自接到谁身上，
//!    只能由这份普查回答——猜错了求出来的是另一个零件的尺寸，几何照样出得来。
//!
//! ```text
//! cargo run --release --example ppoint_probe -- --attlib <attlib.dat> \
//!     --template-dir <shadow 目录> --design <ams8000_0001> \
//!     --catalogue <ams5052_0001> --catalogue <ams5053_0001> [--sample 6]
//! ```

use std::collections::{BTreeMap, HashSet};
use std::path::{Path, PathBuf};
use std::sync::Arc;

use anyhow::{Context, bail};
use e3d_io::db_element::{DbElement, DbElementError, DbFilePin, DbSet, template_file_for};
use e3d_io::db_param_env::DbElementParamEnv;
use e3d_io::engine::{ReadOnlyEngine, ScanTier};
use e3d_io::record::catalogue_eval::AttributeNumber;
use e3d_io::record::descriptor::DescriptorValue;
use e3d_io::record::direction_spec;
use e3d_io::refno::RefNo;
use e3d_model::category::{Category, classify};

/// 从构件通往目录的引用属性，按算法文档 §3.1 的 `GATPOS/GATDIR/GATREA` 走法。
const SPEC_REF: &str = "SPRE";
const CATALOGUE_REF: &str = "CATR";
const POINT_SET_REF: &str = "PTRE";

/// P 点身上要普查存储形状的几何属性。
const POINT_ATTRS: [&str; 5] = ["PDIS", "PBOR", "PAXI", "PCON", "PDIA"];
/// 目录件身上要普查在否的属性——求值环境挂谁身上要靠这张表定。
const COMPONENT_ATTRS: [&str; 7] = ["PARA", "RPRO", "GMRE", "PTRE", "CATR", "IPAR", "SKEY"];
/// 第一轮普查在表达式里实际出现过的 `ATTRIB` 名字。
///
/// 求值器要回答 `ATTRIB PARA[1]` / `ATTRIB ANGL` / `ATTRIB RPRO TLEN`，
/// 但**问的是谁身上的**是这份普查唯一要定的事：P 点自己、目录件、还是设计构件。
/// 猜错了照样求得出数，只是那是另一个零件的尺寸。
const EXPR_NAMES: [&str; 6] = ["PARA", "ANGL", "RPRO", "DESP", "RADI", "TLEN"];

/// 链上每一跳的名字，断在哪一跳就记哪一跳。
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
enum Stage {
    NoSpecRef,
    SpecUnreadable,
    NoCatalogueRef,
    CatalogueUnreadable,
    NoPointSetRef,
    PointSetUnreadable,
    PointSetEmpty,
    NoSuchPointNumber,
    Resolved,
}

impl Stage {
    fn label(self) -> &'static str {
        match self {
            Stage::NoSpecRef => "构件没有 SPRE",
            Stage::SpecUnreadable => "SPRE 指向的元素读不出",
            Stage::NoCatalogueRef => "SPCO 没有 CATR",
            Stage::CatalogueUnreadable => "CATR 指向的 SCOM 读不出",
            Stage::NoPointSetRef => "目录件没有 PTRE",
            Stage::PointSetUnreadable => "PTRE 指向的 PTSE 读不出",
            Stage::PointSetEmpty => "PTSE 没有成员",
            Stage::NoSuchPointNumber => "PTSE 里没有该编号的 P 点",
            Stage::Resolved => "走通，取到 P 点元素",
        }
    }
}

#[derive(Default)]
struct Tally {
    members: usize,
    /// 离开侧 / 到达侧各自走到哪一跳。
    leave_stage: BTreeMap<Stage, usize>,
    arrive_stage: BTreeMap<Stage, usize>,
    /// P 点编号的实测取值组合。
    point_numbers: BTreeMap<String, usize>,
    /// 走通的 P 点元素 noun → 条数。
    point_nouns: BTreeMap<String, usize>,
    /// P 点属性名 → 存储形状 → 条数。这张表决定求值器要吃哪几种。
    shapes: BTreeMap<String, BTreeMap<String, usize>>,
    /// 表达式原文 → 出现次数。
    expressions: BTreeMap<String, usize>,
    /// 未注册的库号（跨库缺口）。
    missing_dbs: BTreeMap<u32, usize>,
    /// 目录件身上有值的属性。
    component_attrs: BTreeMap<String, usize>,
    /// `SPRE` 落在哪个 noun 上。
    spec_nouns: BTreeMap<String, usize>,
    /// 表达式里的名字 → 它在链上哪个元素身上有值 → 条数。
    attr_home: BTreeMap<String, BTreeMap<&'static str, usize>>,
    /// `RPRO` 指向的元素 noun → 它身上有哪些名字。
    rpro_targets: BTreeMap<String, BTreeMap<String, usize>>,
    /// 每种 P 点 noun 打印一次全属性清单。
    dumped_point_nouns: HashSet<String>,
    /// 缓存：同一个目录件被成百上千个构件共用，普查一次就够。
    seen_component: HashSet<RefNo>,
    /// 缓存：同一个 P 点同理。
    seen_point: HashSet<RefNo>,
    /// 缓存：`ATTRIB` 归属只按「P 点 + 目录件」这一对去重。
    seen_home_pair: HashSet<(RefNo, RefNo)>,
    /// 求值结果：属性名 → 结果类别 → 条数。**按实例计**，不去重——
    /// 要回答的是「3171 条槽位里有多少条现在能出几何」，那是实例数不是种类数。
    resolved: BTreeMap<&'static str, BTreeMap<&'static str, usize>>,
    /// 求不出来的公式原文 → 实例条数。报告里能直接看见卡在哪一条。
    unresolved_text: BTreeMap<String, usize>,
    /// 两端（离开 + 到达）的 `PDIS` 都求得出的槽位端点数。
    both_ends_ready: usize,
    ends_seen: usize,
    /// 卡在 `RPRO` 上的公式，按「设计件 noun + 目录件 noun」分。
    /// RPRO 若真是「回指设计件」的伪属性，这张表就该按设计件类型聚成几堆。
    rpro_by_noun: BTreeMap<String, usize>,
    /// 头几个踩到 RPRO 的现场，整链全属性拍下来。
    rpro_dumped: usize,
    /// 已经自检过的弯头个数。
    bend_checked: usize,
    /// `PAXI` 的原始词形 → 实例条数。要把轴解成向量，先得知道有几种形态。
    axis_words: BTreeMap<String, usize>,
    /// `PTCA` 的方向来源形态 → 实例条数。
    ptca_dir: BTreeMap<String, usize>,
    /// `PTCD` 解码结局 → 实例条数。`direction_spec::direction` 到底答不答得出。
    ptcd_outcome: BTreeMap<String, usize>,
    /// `PTCD` 里转角轴与当前方向不正交的链 → 实例条数。
    ///
    /// 解码器把「朝下一轴转 θ」算成 `v·cosθ + n·sinθ` 再归一化，只有 `n ⊥ v`
    /// 时才等于真旋转。这本账就是要回答「不正交的链在实测库里到底有没有」——
    /// 若恒为空，那个隐患就只是理论上的。
    ptcd_oblique: BTreeMap<String, usize>,
}

fn main() -> anyhow::Result<()> {
    let mut attlib: Option<PathBuf> = None;
    let mut template_dir: Option<PathBuf> = None;
    let mut design: Option<PathBuf> = None;
    let mut catalogues: Vec<PathBuf> = Vec::new();
    let mut sample = 4usize;

    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--attlib" => attlib = Some(PathBuf::from(value()?)),
            "--template-dir" => template_dir = Some(PathBuf::from(value()?)),
            "--design" => design = Some(PathBuf::from(value()?)),
            "--catalogue" => catalogues.push(PathBuf::from(value()?)),
            "--sample" => sample = value()?.parse().context("--sample 要无符号整数")?,
            other => bail!("不认识的参数 {other}"),
        }
    }
    let attlib = attlib.context("缺 --attlib")?;
    let template_dir = template_dir.context("缺 --template-dir")?;
    let design = design.context("缺 --design")?;

    let set = Arc::new(DbSet::with_attlib_file(&attlib)?);
    let design_dbno = set.add_db(DbFilePin {
        file: design.clone(),
        template: template_file_for(&template_dir, "DESI")?,
        db_type: Some("DESI".to_string()),
        sesno: None,
    })?;
    println!("设计库 {} dbno={design_dbno}", design.display());
    for file in &catalogues {
        let dbno = set.add_db(DbFilePin {
            file: file.clone(),
            template: template_file_for(&template_dir, "CATA")?,
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
            if matches!(classify(&noun), Category::RouteContainer) {
                for member in el.members()? {
                    let member_noun = member.element_type()?;
                    if !matches!(classify(&member_noun), Category::RouteMember) {
                        continue;
                    }
                    let detail = shown < sample;
                    walk_member(&member, &member_noun, &mut tally, detail);
                    if detail {
                        shown += 1;
                    }
                }
            }
            stack.extend(el.members()?);
        }
    }

    report(&tally);
    Ok(())
}

fn roots_of(db_file: &Path) -> anyhow::Result<Vec<RefNo>> {
    let mut engine =
        ReadOnlyEngine::open(db_file).with_context(|| format!("开库扫根 {}", db_file.display()))?;
    let mut owners = Vec::new();
    let mut indexed = HashSet::new();
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

/// 属性有值就给回来。库没注册这一类单独记账——它是跨库缺口，不是数据缺失。
fn read(el: &DbElement, name: &str, tally: &mut Tally) -> Option<DescriptorValue> {
    match el.attribute(name) {
        Ok(value) => value,
        Err(error) => {
            note_missing_db(&error, tally);
            None
        }
    }
}

fn note_missing_db(error: &DbElementError, tally: &mut Tally) {
    if let DbElementError::DbNotRegistered { dbno } = error {
        *tally.missing_dbs.entry(*dbno).or_insert(0) += 1;
    }
}

/// 引用属性 → 目标句柄，且目标真读得出 noun 才算这一跳走通。
fn hop(el: &DbElement, name: &str, tally: &mut Tally) -> Result<Option<(DbElement, String)>, ()> {
    let target = match el.get_element(name) {
        Ok(Some(target)) => target,
        Ok(None) => return Ok(None),
        Err(error) => {
            note_missing_db(&error, tally);
            return Ok(None);
        }
    };
    match target.element_type() {
        Ok(noun) => Ok(Some((target, noun))),
        Err(error) => {
            note_missing_db(&error, tally);
            Err(())
        }
    }
}

/// P 点编号：设计侧存的是 `Word`（实测 `LEAV=2 / ARRI=1`），也接受 `Int`。
fn point_number(value: &DescriptorValue) -> Option<i64> {
    match value {
        DescriptorValue::Int(n) => Some(i64::from(*n)),
        DescriptorValue::Word { raw, .. } => Some(i64::from(*raw)),
        DescriptorValue::Real(x) if x.fract() == 0.0 => Some(*x as i64),
        _ => None,
    }
}

fn shape_of(value: &DescriptorValue) -> String {
    match value {
        DescriptorValue::Unset => "Unset".into(),
        DescriptorValue::Bool(_) => "Bool".into(),
        DescriptorValue::Int(_) => "Int".into(),
        DescriptorValue::Real(_) => "Real".into(),
        DescriptorValue::Text(text) => match text.contains("ATTRIB") || text.starts_with("( ") {
            true => "Text(PML 表达式)".into(),
            false => "Text".into(),
        },
        DescriptorValue::RefNo(_) => "RefNo".into(),
        DescriptorValue::Word { .. } => "Word".into(),
        DescriptorValue::IntArray(items) => format!("IntArray[{}]", items.len()),
        DescriptorValue::RealArray(items) => format!("RealArray[{}]", items.len()),
        DescriptorValue::RefNoArray(items) => format!("RefNoArray[{}]", items.len()),
        DescriptorValue::WordArray(items) => format!("WordArray[{}]", items.len()),
        DescriptorValue::BoolArray(items) => format!("BoolArray[{}]", items.len()),
        DescriptorValue::RawWords(words) => match e3d_io::record::catalogue_expr::render(words) {
            Some(_) => format!("RawWords[{}](元组表达式)", words.len()),
            None => format!("RawWords[{}](非表达式)", words.len()),
        },
    }
}

/// 全属性清单里一行一个值，长数组截断——要的是「有什么」不是「是多少」。
fn short(value: &DescriptorValue) -> String {
    let text = match expression_text(value) {
        Some(expression) => format!("{} = {expression}", shape_of(value)),
        None => format!("{value:?}"),
    };
    match text.char_indices().nth(110) {
        Some((cut, _)) => format!("{}…", &text[..cut]),
        None => text,
    }
}

/// 这个值向求值环境问的是什么——只看形状，不猜数。
fn expression_text(value: &DescriptorValue) -> Option<String> {
    match value {
        DescriptorValue::Text(text) if text.contains('(') || text.contains("ATTRIB") => {
            Some(text.clone())
        }
        DescriptorValue::RawWords(words) => e3d_io::record::catalogue_expr::render(words),
        _ => None,
    }
}

fn walk_member(member: &DbElement, member_noun: &str, tally: &mut Tally, detail: bool) {
    tally.members += 1;

    let leave = read(member, "LEAV", tally).as_ref().and_then(point_number);
    let arrive = read(member, "ARRI", tally).as_ref().and_then(point_number);
    *tally
        .point_numbers
        .entry(format!("LEAV={leave:?} ARRI={arrive:?}"))
        .or_insert(0) += 1;

    // 目录链走一次就够：离开与到达是同一个 PTSE 里的两个编号。
    let (component, point_set) = match resolve_point_set(member, tally) {
        Ok(found) => found,
        Err(stage) => {
            *tally.leave_stage.entry(stage).or_insert(0) += 1;
            *tally.arrive_stage.entry(stage).or_insert(0) += 1;
            if detail {
                println!(
                    "\n--- {} {member_noun} 断在：{}",
                    member.refno(),
                    stage.label()
                );
            }
            return;
        }
    };

    let points = match point_set.members() {
        Ok(members) => members,
        Err(error) => {
            note_missing_db(&error, tally);
            let stage = Stage::PointSetUnreadable;
            *tally.leave_stage.entry(stage).or_insert(0) += 1;
            *tally.arrive_stage.entry(stage).or_insert(0) += 1;
            return;
        }
    };
    if points.remaining() == 0 {
        let stage = Stage::PointSetEmpty;
        *tally.leave_stage.entry(stage).or_insert(0) += 1;
        *tally.arrive_stage.entry(stage).or_insert(0) += 1;
        return;
    }

    let mut by_number: BTreeMap<i64, (DbElement, String)> = BTreeMap::new();
    for point in points {
        let Ok(noun) = point.element_type() else {
            continue;
        };
        if let Some(number) = read(&point, "NUMB", tally).as_ref().and_then(point_number) {
            by_number.insert(number, (point, noun));
        }
    }

    for (side, wanted, is_leave) in [("离开", leave, true), ("到达", arrive, false)] {
        let stage = match wanted.and_then(|n| by_number.get(&n)) {
            Some((point, noun)) => {
                *tally.point_nouns.entry(noun.clone()).or_insert(0) += 1;
                census_point(point, noun, tally);
                census_attr_home(point, &component, member, tally);
                census_resolution(point, noun, &component, member, tally);
                Stage::Resolved
            }
            None => Stage::NoSuchPointNumber,
        };
        let table = match is_leave {
            true => &mut tally.leave_stage,
            false => &mut tally.arrive_stage,
        };
        *table.entry(stage).or_insert(0) += 1;
        if detail {
            println!("    {side}侧 P 点编号 {wanted:?} → {}", stage.label());
        }
    }

    if detail {
        dump_detail(member, member_noun, &by_number, tally);
    }
}

/// 走 `SPRE → SPCO.CATR → SCOM.PTRE → PTSE`，断在哪一跳就把哪一跳还回去。
///
/// 给回目录件与点集两个句柄：求值环境挂在目录件上，点在点集里。
fn resolve_point_set(
    member: &DbElement,
    tally: &mut Tally,
) -> Result<(DbElement, DbElement), Stage> {
    let spec = match hop(member, SPEC_REF, tally) {
        Ok(Some(found)) => found,
        Ok(None) => return Err(Stage::NoSpecRef),
        Err(()) => return Err(Stage::SpecUnreadable),
    };
    if tally.spec_nouns.is_empty() {
        dump_all(&spec.0, "SPRE 目标");
        dump_all(member, "设计构件");
    }
    *tally.spec_nouns.entry(spec.1.clone()).or_insert(0) += 1;

    // 有的构件直接指目录件，`SPRE` 那一跳落在 SCOM 上而不是 SPCO 上。
    let component = match spec.1.eq_ignore_ascii_case("SCOM") {
        true => spec.0,
        false => match hop(&spec.0, CATALOGUE_REF, tally) {
            Ok(Some(found)) => found.0,
            Ok(None) => return Err(Stage::NoCatalogueRef),
            Err(()) => return Err(Stage::CatalogueUnreadable),
        },
    };

    census_component(&component, tally);

    match hop(&component, POINT_SET_REF, tally) {
        Ok(Some((point_set, _))) => Ok((component, point_set)),
        Ok(None) => Err(Stage::NoPointSetRef),
        Err(()) => Err(Stage::PointSetUnreadable),
    }
}

/// 表达式里的名字到底挂在链上哪个元素身上。
///
/// 同一个名字可能在多处都有（`PARA` 目录件有、设计构件也可能有），所以这里
/// 三处都查、都记，让数字自己说话，而不是查到第一处就收工。
fn census_attr_home(
    point: &DbElement,
    component: &DbElement,
    member: &DbElement,
    tally: &mut Tally,
) {
    if !tally
        .seen_home_pair
        .insert((point.refno(), component.refno()))
    {
        return;
    }

    // 候选宿主要摆全。第一轮只查了三处，`RPRO` 三处都没有——那说明它挂在
    // 链上别的元素身上，而不是「不存在」。少摆一处，结论就会是错的那一种。
    let mut hosts: Vec<(String, DbElement)> = vec![
        ("P点".to_string(), point.clone()),
        ("目录件".to_string(), component.clone()),
        ("设计构件".to_string(), member.clone()),
    ];
    if let Ok(Some(owner)) = point.owner() {
        hosts.push(("P点集".to_string(), owner));
    }
    if let Ok(Some(owner)) = component.owner() {
        hosts.push(("目录件属主".to_string(), owner));
    }
    if let Ok(Some(owner)) = member.owner() {
        hosts.push(("构件属主".to_string(), owner));
    }
    if let Ok(Some(spec)) = member.get_element(SPEC_REF) {
        hosts.push(("SPCO".to_string(), spec));
    }

    for name in EXPR_NAMES {
        for (where_, el) in &hosts {
            if matches!(el.attribute(name), Ok(Some(_))) {
                *tally
                    .attr_home
                    .entry(name.to_string())
                    .or_default()
                    .entry(leak(where_))
                    .or_insert(0) += 1;
            }
        }
    }

    // `ATTRIB RPRO TLEN` 要跟着 `RPRO` 跳一步，跳到哪、那边有什么，得看见才算数。
    for (where_, el) in &hosts {
        let Ok(Some(target)) = el.get_element("RPRO") else {
            continue;
        };
        let Ok(noun) = target.element_type() else {
            continue;
        };
        let bucket = tally
            .rpro_targets
            .entry(format!("{where_}.RPRO → {noun}"))
            .or_default();
        *bucket.entry("(命中次数)".to_string()).or_insert(0) += 1;
        for name in ["TLEN", "RADI", "PARA", "ANGL"] {
            if matches!(target.attribute(name), Ok(Some(_))) {
                *bucket.entry(name.to_string()).or_insert(0) += 1;
            }
        }
    }
}

/// 拿新的 [`DbElementParamEnv`] 真求一遍，看有多少条落地成数。
///
/// 这是整支探针唯一的产出数：P2 那 3171 条「缺 P 点」的槽位，接上目录求值之后
/// 到底还剩多少条缺。数不出来就只是「实现了求值器」，不是「解封了 P2」。
fn census_resolution(
    point: &DbElement,
    noun: &str,
    component: &DbElement,
    member: &DbElement,
    tally: &mut Tally,
) {
    let env = DbElementParamEnv::new(Some(member.clone()), Some(component.clone()));

    // 轴与方向的形态普查：`PDIS` 求出来只是一半，还得知道往哪个方向量。
    match noun.eq_ignore_ascii_case("PTCA") {
        true => {
            let shape = match point.attribute("DIR") {
                Ok(Some(DescriptorValue::RealArray(items))) => format!("DIR 三元组 {items:?}"),
                Ok(Some(other)) => format!("DIR {}", short(&other)),
                Ok(None) => "DIR 无值".to_string(),
                Err(_) => "DIR 模板无此名".to_string(),
            };
            *tally.ptca_dir.entry(shape).or_insert(0) += 1;
            census_ptcd(point, &env, tally);
        }
        false => {
            let shape = match point.attribute("PAXI") {
                Ok(Some(DescriptorValue::RawWords(words))) => format!("{words:?}"),
                Ok(Some(DescriptorValue::IntArray(words))) => format!("int {words:?}"),
                Ok(Some(other)) => format!("其它 {}", short(&other)),
                Ok(None) => "无值".to_string(),
                Err(_) => "模板无此名".to_string(),
            };
            *tally.axis_words.entry(shape).or_insert(0) += 1;
        }
    }

    // `PTAX` 用「轴 + 距离」定位，`PTCA` 直接给三个坐标。两条路都要能走。
    let wanted: &[&'static str] = match noun.eq_ignore_ascii_case("PTCA") {
        true => &["PX", "PY", "PZ", "PBOR"],
        false => &["PDIS", "PBOR"],
    };

    let mut positional_ok = true;
    for name in wanted {
        let outcome = match point.get_number(name, &env) {
            Ok(outcome) => outcome,
            // 模板里没这个名字：算一类，别混进「求不出」。
            Err(_) => {
                *tally
                    .resolved
                    .entry(name)
                    .or_default()
                    .entry("模板无此名")
                    .or_insert(0) += 1;
                if *name != "PBOR" {
                    positional_ok = false;
                }
                continue;
            }
        };
        *tally
            .resolved
            .entry(name)
            .or_default()
            .entry(kind_label(&outcome))
            .or_insert(0) += 1;
        if let AttributeNumber::Unresolved { form, text } = &outcome {
            let text = text.clone().unwrap_or_else(|| "(渲染不出)".to_string());
            if text.contains("RPRO") {
                census_rpro(&text, point, component, member, tally);
            }
            *tally
                .unresolved_text
                .entry(format!("{name} [{}]: {text}", form.as_str()))
                .or_insert(0) += 1;
        }
        if *name != "PBOR" && outcome.value().is_none() {
            positional_ok = false;
        }
    }

    tally.ends_seen += 1;
    if positional_ok {
        tally.both_ends_ready += 1;
    }

    // 刚接上的 `RPRO RADI` 一口气多解了 138 个弯头，值对不对得自己验：
    // 弯头 P 点的 `PDIS` 就该等于 `RADI * tan(ANGL/2)`，两边都从库里读，
    // 对不上就是 `PURP=ATTR` 读错了元素。
    let uses_design_property = matches!(
        point.attribute("PDIS"),
        Ok(Some(DescriptorValue::Text(ref text))) if text.contains("RPRO")
    );
    if tally.bend_checked < 3 && uses_design_property {
        let radi = member.get_number("RADI", &env).ok().and_then(|n| n.value());
        let angl = member.get_number("ANGL", &env).ok().and_then(|n| n.value());
        let pdis = point.get_number("PDIS", &env).ok().and_then(|n| n.value());
        if let (Some(radi), Some(angl), Some(pdis)) = (radi, angl, pdis) {
            tally.bend_checked += 1;
            let expect = radi * (angl / 2.0).to_radians().tan();
            println!(
                "  弯头自检 {}：RADI={radi} ANGL={angl} → PDIS={pdis}，\
                 RADI*tan(ANGL/2)={expect:.6}，差 {:.9}",
                member.refno(),
                (pdis - expect).abs(),
            );
            if let Ok(Some(value)) = point.attribute("PDIS") {
                println!("      这个弯头的 PDIS 公式：{}", short(&value));
            }
            match component.get_element("DTRE") {
                Ok(Some(table)) => {
                    let keys: Vec<String> = table
                        .members()
                        .map(|rows| {
                            rows.filter_map(|row| word_of(&row, "DKEY").zip(word_of(&row, "PURP")))
                                .map(|(key, purpose)| format!("{key}={purpose}"))
                                .collect()
                        })
                        .unwrap_or_default();
                    println!("      它的设计表：{}", keys.join(" "));
                    println!("      设计件的 DESP：{:?}", member.attribute("DESP"));
                    if let Ok(rows) = table.members() {
                        for row in rows {
                            if word_of(&row, "DKEY").as_deref() == Some("RADI") {
                                dump_all(&row, "      RADI 行");
                            }
                        }
                    }
                }
                _ => println!("      它的目录件没有 DTRE"),
            }
        }
    }
}

/// 把踩到 `RPRO` 的现场记下来：RPRO 不在字典里，只能靠数据反推它指向谁。
fn census_rpro(
    text: &str,
    point: &DbElement,
    component: &DbElement,
    member: &DbElement,
    tally: &mut Tally,
) {
    // 只要公式里那个跟在 RPRO 后面的名字，别把整条式子当键。
    let wanted = text
        .split_whitespace()
        .skip_while(|word| *word != "RPRO")
        .nth(1)
        .unwrap_or("?");
    let design = member.element_type().unwrap_or_else(|_| "?".into());
    let cata = component.element_type().unwrap_or_else(|_| "?".into());
    *tally
        .rpro_by_noun
        .entry(format!("RPRO {wanted:<5} 设计件 {design:<6} 目录件 {cata}"))
        .or_insert(0) += 1;

    if tally.rpro_dumped >= 2 {
        return;
    }
    tally.rpro_dumped += 1;
    println!("\n######## RPRO 现场 {}：{text}", tally.rpro_dumped);
    dump_all(member, "设计构件");
    dump_all(component, "目录件");
    dump_all(point, "P 点");
    // 设计件的属主往往是 BRAN/PIPE，长度类的量可能挂在那儿。
    if let Ok(Some(owner)) = member.owner() {
        dump_all(&owner, "设计件的属主");
    }
    // `DTRE` 是目录件的设计表引用。若 `RPRO TLEN` 是「按名字查设计参数」，
    // 名字表就该在这棵树上，展开一层看得见就认。
    if let Ok(Some(table)) = component.get_element("DTRE") {
        dump_all(&table, "目录件的 DTRE");
        if let Ok(rows) = table.members() {
            for row in rows.take(40) {
                dump_all(&row, "  DTRE 的成员");
            }
        }
    }
}

/// 一个 word 属性的文本形态，读不出就 `None`。
fn word_of(element: &DbElement, name: &str) -> Option<String> {
    match element.attribute(name) {
        Ok(Some(DescriptorValue::Word { text, .. })) => text,
        _ => None,
    }
}

fn kind_label(outcome: &AttributeNumber) -> &'static str {
    match outcome {
        AttributeNumber::Stored(_) => "直存的数",
        AttributeNumber::Evaluated { form, .. } => match form.as_str() {
            "tuple" => "求出来了(元组)",
            _ => "求出来了(PML)",
        },
        AttributeNumber::Unset => "unset",
        AttributeNumber::NotNumeric => "不是数",
        AttributeNumber::Unresolved { .. } => "求不出",
    }
}

/// 宿主名字来自编译期常量组合，泄漏一次换 `&'static str` 做键。
fn leak(where_: &str) -> &'static str {
    match where_ {
        "P点" => "P点",
        "目录件" => "目录件",
        "设计构件" => "设计构件",
        "P点集" => "P点集",
        "目录件属主" => "目录件属主",
        "构件属主" => "构件属主",
        "SPCO" => "SPCO",
        other => Box::leak(other.to_string().into_boxed_str()),
    }
}

/// `PTCA` 的真方向存在 `PTCD` 里。这本账回答两件事：解得出几条，以及
/// 解码器那个「非正交转角」的隐患在实测库里到底触不触得到。
fn census_ptcd(point: &DbElement, env: &DbElementParamEnv, tally: &mut Tally) {
    let words = match point.attribute("PTCD") {
        Ok(Some(DescriptorValue::RawWords(words) | DescriptorValue::WordArray(words))) => words,
        Ok(Some(DescriptorValue::IntArray(words))) => {
            words.into_iter().map(|word| word as u32).collect()
        }
        Ok(Some(other)) => {
            *tally
                .ptcd_outcome
                .entry(format!("非字数组 {}", short(&other)))
                .or_insert(0) += 1;
            return;
        }
        Ok(None) => {
            *tally.ptcd_outcome.entry("无值".to_string()).or_insert(0) += 1;
            return;
        }
        Err(_) => {
            *tally
                .ptcd_outcome
                .entry("模板无此名".to_string())
                .or_insert(0) += 1;
            return;
        }
    };

    // opcode = 20 + 方向个数，是「几段转角」的唯一来源，分开记才看得出
    // 解不出的是集中在某一种形状上，还是散落在表达式里。
    let opcode = words.get(3).copied().unwrap_or(0);
    let outcome = match direction_spec::direction(&words, env) {
        Some(_) => "解出",
        None => "解不出",
    };
    let rendered = match direction_spec::render(&words) {
        Some(text) => text,
        None => "（渲染也不认）".to_string(),
    };
    *tally
        .ptcd_outcome
        .entry(format!("opcode {opcode} {outcome}"))
        .or_insert(0) += 1;

    if let Some(worst) = oblique_turn(&words, env) {
        *tally
            .ptcd_oblique
            .entry(format!("|cos|={worst:.3}  {rendered}"))
            .or_insert(0) += 1;
    }
}

/// 沿 `PTCD` 的转角链走一遍，回报「转角轴与当时方向」最不正交的那一处的 `|cos|`。
///
/// 全正交（解码器的算式成立）就返回 `None`。阈值取 `1e-9`：轴都是 `±X/±Y/±Z`
/// 的单位向量，点积要么恰好是 0 要么是 ±1 量级，不存在需要放宽的中间地带。
fn oblique_turn(words: &[u32], env: &DbElementParamEnv) -> Option<f64> {
    const AXES: [[f64; 3]; 6] = [
        [1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, 0.0, -1.0],
    ];
    let axis = |code: u32| {
        AXES.get(usize::try_from(code.checked_sub(11)?).ok()?)
            .copied()
    };

    let [declared, leading, 2, opcode, body @ ..] = words else {
        return None;
    };
    if *declared as usize != words.len() || !(*leading == 5 || *leading == 6) {
        return None;
    }
    let turns = usize::try_from(opcode.checked_sub(21)?).ok()?;
    let (61, body) = body.split_last()? else {
        return None;
    };
    let (first, mut rest) = body.split_first()?;
    let mut vector = axis(*first)?;
    let mut worst = 0.0_f64;
    for _ in 0..turns {
        let length = usize::try_from(*rest.first()?).ok()?;
        let program: Vec<i32> = rest.get(..length)?.iter().map(|w| *w as i32).collect();
        let angle = e3d_io::record::catalogue_pml::evaluate_words(&program, env)?.to_radians();
        rest = rest.get(length..)?;
        let (next, tail) = rest.split_first()?;
        let next = axis(*next)?;
        let dot = vector[0] * next[0] + vector[1] * next[1] + vector[2] * next[2];
        worst = worst.max(dot.abs());
        let mut turned = [0.0; 3];
        for i in 0..3 {
            turned[i] = vector[i] * angle.cos() + next[i] * angle.sin();
        }
        let length = (turned[0].powi(2) + turned[1].powi(2) + turned[2].powi(2)).sqrt();
        // 长度归零只可能是转角轴与当前方向反平行、正好抵消——那是最不正交的一种。
        if length <= 0.0 || !length.is_finite() {
            return Some(1.0);
        }
        vector = [turned[0] / length, turned[1] / length, turned[2] / length];
        rest = tail;
    }
    (worst > 1.0e-9).then_some(worst)
}

/// 目录件身上有哪些属性——求值环境挂谁身上要靠这张表定。
fn census_component(component: &DbElement, tally: &mut Tally) {
    if !tally.seen_component.insert(component.refno()) {
        return;
    }
    if tally.seen_component.len() == 1 {
        dump_all(component, "目录件");
    }
    for name in COMPONENT_ATTRS {
        if matches!(component.attribute(name), Ok(Some(_))) {
            *tally.component_attrs.entry(name.to_string()).or_insert(0) += 1;
        }
    }
}

/// 一个元素的全部有值属性，一次性摊开。
fn dump_all(el: &DbElement, label: &str) {
    let noun = el.element_type().unwrap_or_else(|_| "?".to_string());
    println!("\n=== {label} {noun} 全属性（样本 {}）：", el.refno());
    match el.attribute_names() {
        Ok(names) => {
            for name in names {
                if let Ok(Some(value)) = el.attribute(&name) {
                    println!("    {name:<8} {}", short(&value));
                }
            }
        }
        Err(error) => println!("    属性名读不出: {error}"),
    }
}

/// P 点元素的几何属性各自存成什么形状。
fn census_point(point: &DbElement, noun: &str, tally: &mut Tally) {
    // `PTCA` 没有 `PDIS`/`PAXI`，它的坐标存在别的名字下。每种 noun 打一次
    // 全属性清单，省得下一轮又为「那 60 条无值是怎么回事」再跑一趟。
    if tally.dumped_point_nouns.insert(noun.to_string()) {
        println!("\n=== {noun} 全属性（样本 {}）：", point.refno());
        match point.attribute_names() {
            Ok(names) => {
                for name in names {
                    if let Ok(Some(value)) = point.attribute(&name) {
                        println!("    {name:<8} {}", short(&value));
                    }
                }
            }
            Err(error) => println!("    属性名读不出: {error}"),
        }
    }
    if !tally.seen_point.insert(point.refno()) {
        return;
    }
    for name in POINT_ATTRS {
        let shape = match read(point, name, tally) {
            Some(value) => {
                if let Some(text) = expression_text(&value) {
                    *tally
                        .expressions
                        .entry(format!("{name}: {text}"))
                        .or_insert(0) += 1;
                }
                shape_of(&value)
            }
            None => "(无值)".to_string(),
        };
        *tally
            .shapes
            .entry(name.to_string())
            .or_default()
            .entry(shape)
            .or_insert(0) += 1;
    }
}

fn dump_detail(
    member: &DbElement,
    member_noun: &str,
    by_number: &BTreeMap<i64, (DbElement, String)>,
    tally: &mut Tally,
) {
    println!("--- {} {member_noun} 的 P 点集：", member.refno());
    for (number, (point, noun)) in by_number.iter().take(6) {
        let values = POINT_ATTRS
            .iter()
            .map(|name| format!("{name}={:?}", read(point, name, tally)))
            .collect::<Vec<_>>()
            .join("  ");
        println!("    P{number} {} {noun} {values}", point.refno());
    }
}

fn percent(part: usize, whole: usize) -> f64 {
    match whole {
        0 => 0.0,
        _ => part as f64 * 100.0 / whole as f64,
    }
}

fn report(tally: &Tally) {
    println!("\n================ 汇总 ================");
    println!("路由成员 {}", tally.members);

    println!("\nP 点编号实测取值：");
    for (combo, count) in &tally.point_numbers {
        println!("  {combo:<28} {count:>6}");
    }

    println!("\nSPRE 落在哪个 noun 上：");
    for (noun, count) in &tally.spec_nouns {
        println!("  {noun:<8} {count:>6}");
    }

    for (side, table) in [
        ("离开侧", &tally.leave_stage),
        ("到达侧", &tally.arrive_stage),
    ] {
        println!("\n{side}目录链走到哪一跳：");
        for (stage, count) in table.iter() {
            println!("  {:<28} {count:>6}", stage.label());
        }
    }

    println!("\n走通的 P 点元素 noun：");
    for (noun, count) in &tally.point_nouns {
        println!("  {noun:<8} {count:>6}");
    }

    println!(
        "\nP 点属性的存储形状（按去重后的 {} 个 P 点算）：",
        tally.seen_point.len()
    );
    for (name, shapes) in &tally.shapes {
        println!("  {name}:");
        let mut rows: Vec<_> = shapes.iter().collect();
        rows.sort_by(|a, b| b.1.cmp(a.1));
        for (shape, count) in rows {
            println!("    {shape:<28} {count:>6}");
        }
    }

    println!(
        "\n目录件属性有值的（去重后 {} 个目录件）：",
        tally.seen_component.len()
    );
    for (name, count) in &tally.component_attrs {
        println!("  {name:<6} {count:>6}");
    }

    println!(
        "\n表达式原文，去重 {} 条（按出现次数前 30）：",
        tally.expressions.len()
    );
    let mut rows: Vec<_> = tally.expressions.iter().collect();
    rows.sort_by(|a, b| b.1.cmp(a.1).then(a.0.cmp(b.0)));
    for (text, count) in rows.iter().take(30) {
        println!("  {count:>6}x  {text}");
    }

    println!("\n表达式里的名字挂在链上哪个元素身上：");
    for (name, homes) in &tally.attr_home {
        let mut rows: Vec<_> = homes.iter().collect();
        rows.sort_by(|a, b| b.1.cmp(a.1));
        let line = rows
            .iter()
            .map(|(where_, count)| format!("{where_} {count}"))
            .collect::<Vec<_>>()
            .join("   ");
        println!("  {name:<6} {line}");
    }
    for name in EXPR_NAMES {
        if !tally.attr_home.contains_key(name) {
            println!("  {name:<6} 三处都没有");
        }
    }

    println!(
        "\n★ 真求值：按实例算，{} 个走通的端点里定位属性全求出的 {}（{:.1}%）",
        tally.ends_seen,
        tally.both_ends_ready,
        percent(tally.both_ends_ready, tally.ends_seen),
    );
    for (name, kinds) in &tally.resolved {
        let total: usize = kinds.values().sum();
        println!("  {name}（{total} 次）:");
        let mut rows: Vec<_> = kinds.iter().collect();
        rows.sort_by(|a, b| b.1.cmp(a.1));
        for (kind, count) in rows {
            println!(
                "    {kind:<20} {count:>6}  {:>5.1}%",
                percent(*count, total)
            );
        }
    }

    if tally.unresolved_text.is_empty() {
        println!("\n没有求不出的公式。");
    } else {
        println!(
            "\n求不出的公式，去重 {} 条（按出现次数前 20）：",
            tally.unresolved_text.len()
        );
        let mut rows: Vec<_> = tally.unresolved_text.iter().collect();
        rows.sort_by(|a, b| b.1.cmp(a.1).then(a.0.cmp(b.0)));
        for (text, count) in rows.iter().take(20) {
            println!("  {count:>6}x  {text}");
        }
    }

    println!("\nPTAX 的 PAXI 词形（按实例）：");
    let mut rows: Vec<_> = tally.axis_words.iter().collect();
    rows.sort_by(|a, b| b.1.cmp(a.1).then(a.0.cmp(b.0)));
    for (shape, count) in rows {
        println!("  {count:>6}x  {shape}");
    }

    println!("\nPTCA 的方向来源（按实例）：");
    let mut rows: Vec<_> = tally.ptca_dir.iter().collect();
    rows.sort_by(|a, b| b.1.cmp(a.1).then(a.0.cmp(b.0)));
    for (shape, count) in rows {
        println!("  {count:>6}x  {shape}");
    }

    println!("\nPTCD 解码结局（按实例）：");
    let mut rows: Vec<_> = tally.ptcd_outcome.iter().collect();
    rows.sort_by(|a, b| b.1.cmp(a.1).then(a.0.cmp(b.0)));
    let decoded: usize = tally
        .ptcd_outcome
        .iter()
        .filter(|(shape, _)| shape.ends_with("解出"))
        .map(|(_, count)| *count)
        .sum();
    let ptcd_total: usize = tally.ptcd_outcome.values().sum();
    for (shape, count) in rows {
        println!("  {count:>6}x  {shape}");
    }
    println!(
        "  —— 解出 {decoded}/{ptcd_total}（{:.1}%）",
        percent(decoded, ptcd_total)
    );

    println!("\nPTCD 非正交转角（解码器算式在这些链上不成立）：");
    match tally.ptcd_oblique.is_empty() {
        true => println!("  无。实测库里每一段转角都朝着与当前方向正交的轴。"),
        false => {
            let mut rows: Vec<_> = tally.ptcd_oblique.iter().collect();
            rows.sort_by(|a, b| b.1.cmp(a.1).then(a.0.cmp(b.0)));
            for (shape, count) in rows.iter().take(20) {
                println!("  {count:>6}x  {shape}");
            }
        }
    }

    println!("\nRPRO 卡点分布（按实例）：");
    let mut rows: Vec<_> = tally.rpro_by_noun.iter().collect();
    rows.sort_by(|a, b| b.1.cmp(a.1).then(a.0.cmp(b.0)));
    for (combo, count) in rows {
        println!("  {count:>6}x  {combo}");
    }

    println!("\nRPRO 跳一步之后：");
    for (path, names) in &tally.rpro_targets {
        let mut rows: Vec<_> = names.iter().collect();
        rows.sort_by(|a, b| b.1.cmp(a.1));
        let line = rows
            .iter()
            .map(|(name, count)| format!("{name} {count}"))
            .collect::<Vec<_>>()
            .join("   ");
        println!("  {path:<28} {line}");
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

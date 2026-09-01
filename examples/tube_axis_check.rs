//! 管身自证：量出来的管，走向必须跟它两端 P 点的轴对得上。
//!
//! P 点求解没有现成真值可对——E3D 不把隐式管身存进库。但有一条几何硬约束可以
//! 自己跟自己对：管身是**沿着连接轴**长出来的，所以
//!
//! ```text
//! normalize(to.pos − from.pos)  ∥  ±from.dir
//!                               ∥  ±to.dir
//! ```
//!
//! **判的是共线，不是同向。** 两端各自朝里还是朝外是 E3D 的约定问题，猜错符号
//! 会把一批全对的管误判成全错（本探针第一版就栽在这上面）。所以夹角一律折进
//! `[0°, 90°]`，符号另开一栏单独统计——那栏才是用来读约定的。
//!
//! 求错了 P 点——取错编号、忘了转世界系、轴向解码反了——共线度立刻散开；
//! 求对了，它必须死死压在 0° 上。
//!
//! ```text
//! cargo run --example tube_axis_check -- --attlib <attlib.dat> \
//!     --db <ams8000_0001> --template <目录> \
//!     --catalogue <ams5052_0001> [--catalogue ...] [--sample 8]
//! ```

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use anyhow::{Context, bail};
use e3d_io::db_element::{DbElement, DbElementError, DbFilePin, DbSet, template_file_for};
use e3d_io::engine::{ReadOnlyEngine, ScanTier};
use e3d_io::record::descriptor::DescriptorValue;
use e3d_io::refno::RefNo;
use e3d_model::catalogue_point::CataloguePoints;
use e3d_model::category::{Category, classify};
use e3d_model::route::{RouteEnd, ZERO_LENGTH_TOL_MM, resolve_end, tube_length, walk_route};
use e3d_model::transform::world_matrix;

/// 夹角分箱上界（度）。最后一箱是「其余」。
const BUCKETS: [f64; 5] = [0.01, 0.1, 1.0, 5.0, 30.0];
const BUCKET_KEYS: [&str; 7] = [
    "< 0.01°",
    "< 0.1°",
    "< 1°",
    "< 5°",
    "< 30°",
    "≥ 30°",
    "方向缺失",
];
/// 算「对齐」的那几箱。
const ALIGNED_KEYS: [&str; 3] = ["< 0.01°", "< 0.1°", "< 1°"];

/// 横向偏移分箱上界（mm）。
///
/// **夹角是个会骗人的判据。** 同样 10 mm 的横向偏差，落在 5 m 长的管上是 0.1°、
/// 落在 10 mm 长的槽位上就是 90°——而路由链里恰恰塞满了几毫米到几十毫米的短槽位
/// （相邻两件几乎贴着）。按夹角统计，短槽位会把整张表拖垮，读出来的结论是
/// 「P 点求解全错」，而真实情况可能只是「有一批位置差了几十毫米」。
/// 所以横向偏移要按**毫米**量，与槽位长度无关。
const LATERAL_BUCKETS: [f64; 6] = [0.001, 0.01, 0.1, 1.0, 10.0, 100.0];
const LATERAL_KEYS: [&str; 7] = [
    "< 1µm",
    "< 0.01mm",
    "< 0.1mm",
    "< 1mm",
    "< 10mm",
    "< 100mm",
    "≥ 100mm",
];
/// 算「位置对上了」的那几箱：1 mm 以内。E3D 的位置属性本身就只存到 0.01 mm。
const LATERAL_OK: [&str; 4] = ["< 1µm", "< 0.01mm", "< 0.1mm", "< 1mm"];

/// 槽长分箱上界（mm），只给 `--noun` 那一族用。
const LENGTH_BUCKETS: [f64; 5] = [1.0, 10.0, 100.0, 1000.0, 10000.0];
const LENGTH_KEYS: [&str; 6] = ["< 1mm", "< 10mm", "< 100mm", "< 1m", "< 10m", "≥ 10m"];

#[derive(Default)]
struct Tally {
    slots: usize,
    /// 两端都解出来、且非零长——只有这些才有「走向」可言。
    measured: usize,
    zero_length: usize,
    unresolved: usize,
    db_not_open: usize,
    /// 没开的库号 → 被它挡住的槽位条数。
    ///
    /// `db_not_open` 只说「有多少条卡在配置上」，说不出**该去开哪个库**，而这
    /// 两件事的差别就是「知道有盲区」与「能把盲区消掉」。`DbNotRegistered` 本来
    /// 就带着 dbno，把它扔掉纯属浪费。
    db_not_open_by_dbno: BTreeMap<u32, usize>,
    /// 库开了、但索引里没有那个 refno。库号 → 条数。
    ///
    /// 与 [`Tally::db_not_open`] 是**两件事**，混成一栏会让「开错文件」伪装成
    /// 「还没配」：前者是没注册过这个库号，后者是注册了一个自报同号的文件而它
    /// 里面根本没这件元素——跨工程的库号会撞车（`aps7600_0001` 与本工程的 db
    /// 7600 就是两个不同的库），撞上了照样开得起来、照样自报 7600，只有下降到
    /// 索引这一步才露馅。分开记，才看得出该去补配置还是该换文件。
    not_in_index_by_dbno: BTreeMap<u32, usize>,
    /// 端型组合（如 `构件离开 → 构件到达`）→ 条数。
    by_ends: BTreeMap<&'static str, usize>,
    /// 量出来的按端型组合分：容器端与构件端要分开看，混在一起
    /// 看不出错在哪一侧。
    measured_by_ends: BTreeMap<&'static str, usize>,
    /// (端型组合, 夹角分箱) → 条数。出发端轴与走向的**共线度**。
    from_angle: BTreeMap<(&'static str, &'static str), usize>,
    /// 同上，到达端。
    to_angle: BTreeMap<(&'static str, &'static str), usize>,
    /// (端型组合, 符号模式) → 条数。只统计两端都共线（< 1°）的那些，
    /// 否则符号无意义。用来读出 E3D 的朝向约定。
    signs: BTreeMap<(&'static str, &'static str), usize>,
    /// (端型组合, 横向偏移分箱) → 条数。横向偏移 = `to.pos` 到「过 `from.pos`
    /// 沿 `from.dir` 的直线」的垂距（mm），与槽位长短无关，见 [`LATERAL_BUCKETS`]。
    lateral: BTreeMap<(&'static str, &'static str), usize>,
    /// (端型组合, 两端轴夹角分箱) → 条数。**只看两端轴自己，不看位置。**
    ///
    /// 两端 P 点的轴本该反平行（一件的出口对着下一件的入口）。这一栏与横向偏移
    /// 分开看才分得出两种完全不同的病：轴对不上 = 朝向解码错；轴对得上而位置差
    /// 几十毫米 = 位置那一路错，或者模型里本来就是错开的。
    axis_pair: BTreeMap<(&'static str, &'static str), usize>,
    /// 横向偏移最大的若干条，留证（含容器名，便于回到 E3D 里核对）。
    worst: Vec<(f64, String)>,
    /// 偏移形态 → 条数。把 `to.pos - from.pos` 里垂直于 `from.dir` 的那一半
    /// 拆成「纯铅垂 / 纯水平 / 混合」。
    ///
    /// 这是分辨「模型本来就错开」与「我们少算了一层变换」的关键：真实设计里的
    /// 错开是杂的，而少算变换会让偏移集中在某一个方向上。
    lateral_shape: BTreeMap<&'static str, usize>,
    /// 有偏移的容器 refno 集合，与容器总数对比。**偏移是集中在少数容器上，
    /// 还是摊在几乎每个容器上**——前者是数据脏，后者是我们错。
    offending_containers: std::collections::HashSet<RefNo>,
    containers: std::collections::HashSet<RefNo>,
    /// 至少有一条槽位量得出来的容器。只有这些才有资格进偏移的分母——
    /// 拿全部容器当分母会被大量「零长 / P 点库没开」的容器稀释成好看的数字。
    measured_containers: std::collections::HashSet<RefNo>,
    /// 偏移量的整数毫米值 → 条数。整数偏移（100.000 / 400.000）几乎必然是
    /// 设计师手填的，坐标算错很难恰好落在整毫米上。
    round_offsets: BTreeMap<i64, usize>,
    /// (离开端构件的 noun, 槽位结局) → 条数。见 [`note_leave_noun`]。
    leave_outcome: BTreeMap<(String, &'static str), usize>,
    /// (离开端构件的 noun, 横向偏移分箱) → 条数。
    ///
    /// 与 [`Tally::lateral`] 是同一批数据换一刀切法，而这一刀切在**元件族**上。
    /// 端型那一刀分得出「错在容器端还是构件端」，分不出「哪一族元件的目录链解
    /// 得不对」，可后者才是能落到具体一本目录、一条 `PDIS` 表达式上的维度。
    ///
    /// 更要紧的是：只有这一栏能判「零长率低」到底是不是病。零长率低单看是**没有
    /// 结论**的——`ATTA` 这种附着点本来就常跟下一站隔着一整根管，间隙是它应有的
    /// 样子；`FTUB` 自己就是那根管，间隙才反常。分不出「该有的间隙」与「解错位
    /// 置解出来的间隙」，就只能拿零长率当质量指标读，而那是错的。
    leave_noun_lateral: BTreeMap<(String, &'static str), usize>,
    /// (离开端构件的 noun, 两端轴夹角分箱) → 条数。同上，只看朝向不看位置。
    leave_noun_axis: BTreeMap<(String, &'static str), usize>,
    /// `--noun` 指定要盯的那一族。为空则下面两栏不记。
    focus_noun: Option<String>,
    /// 盯住那一族的槽长分布。
    ///
    /// 汇总表答得了「这一族准不准」，答不了「它的间隙是个什么东西」——而这两件
    /// 事导向完全不同的结论：紧贴着下一件却差了几十毫米，是解错了；隔着一整根
    /// 管，那间隙本来就该在那儿。差别全写在槽长上。
    focus_lengths: BTreeMap<&'static str, usize>,
    /// 盯住那一族的逐条明细 `(横向偏移, 槽长, 行)`，留证用。
    ///
    /// 打印两刀：按横向偏移降序列歪着的（真正需要看脸的那几条），再按槽长升序
    /// 列 10mm 以下的短槽——「本该贴着装却差了一点」正是解错位置该有的形态，
    /// 而这种缝横向未必过 1mm，只按横向排序它们永远轮不到露脸。
    focus_detail: Vec<(f64, f64, String)>,
    /// 夹角超过 30° 的样本，留证。**按端型各留几条**——只留全局前几条的话，
    /// 条数最多的那一类会把别的类挤没，而恰恰是别的类才分得出错在哪一侧。
    outliers: BTreeMap<&'static str, Vec<String>>,
}

fn main() -> anyhow::Result<()> {
    let mut attlib: Option<PathBuf> = None;
    let mut db_file: Option<PathBuf> = None;
    let mut template: Option<PathBuf> = None;
    let mut catalogues: Vec<PathBuf> = Vec::new();
    let mut db_type = "DESI".to_string();
    let mut sesno: Option<u32> = None;
    let mut sample = 8usize;
    let mut dump: Option<RefNo> = None;
    let mut focus_noun: Option<String> = None;

    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--attlib" => attlib = Some(PathBuf::from(value()?)),
            "--db" => db_file = Some(PathBuf::from(value()?)),
            "--template" => template = Some(PathBuf::from(value()?)),
            "--catalogue" => catalogues.push(PathBuf::from(value()?)),
            "--db-type" => db_type = value()?,
            "--sesno" => sesno = Some(value()?.parse().context("--sesno 要无符号整数")?),
            "--sample" => sample = value()?.parse().context("--sample 要无符号整数")?,
            "--noun" => focus_noun = Some(value()?.to_uppercase()),
            "--dump" => dump = Some(parse_refno(&value()?)?),
            other => bail!("不认识的参数 {other}"),
        }
    }
    let attlib = attlib.context("缺 --attlib")?;
    let db_file = db_file.context("缺 --db")?;
    let template_dir = template.context("缺 --template")?;

    let set = Arc::new(DbSet::with_attlib_file(&attlib)?);
    let dbno = set.add_db(DbFilePin {
        file: db_file.clone(),
        template: template_file_for(&template_dir, &db_type)?,
        db_type: Some(db_type),
        sesno,
    })?;
    println!("设计库 {} dbno={dbno}", db_file.display());
    for file in &catalogues {
        let cat = set.add_db(DbFilePin {
            file: file.clone(),
            template: template_file_for(&template_dir, "CATA")?,
            db_type: Some("CATA".to_string()),
            sesno: None,
        })?;
        println!("目录库 {} dbno={cat}", file.display());
    }

    if let Some(refno) = dump {
        return dump_container(&set.element(refno));
    }

    let mut tally = Tally {
        focus_noun,
        ..Tally::default()
    };
    for root in roots_of(&db_file, sesno)? {
        let mut stack = vec![set.element(root)];
        while let Some(el) = stack.pop() {
            if matches!(classify(&el.element_type()?), Category::RouteContainer) {
                check_container(&el, &mut tally, sample)?;
            }
            stack.extend(el.members()?);
        }
    }

    report(&mut tally);
    Ok(())
}

fn parse_refno(text: &str) -> anyhow::Result<RefNo> {
    let (w0, w1) = text
        .trim_start_matches('=')
        .split_once('/')
        .with_context(|| format!("--dump 要 `w0/w1` 形式，得到 {text}"))?;
    Ok(RefNo::new(w0.parse()?, w1.parse()?))
}

/// 把一个容器的整条链摊开：容器头尾、每个构件的定位属性、它的目录件、
/// 目录件上每个 P 点的原始属性与解出来的结果。
///
/// 分箱统计只说「有多少条不对」，摊开才说得出「哪一跳错了」。
fn dump_container(container: &DbElement) -> anyhow::Result<()> {
    println!(
        "\n=== 容器 {} {} {:?}",
        container.refno(),
        container.element_type()?,
        container.stored_name()?
    );
    for name in container.attribute_names()? {
        if let Ok(Some(value)) = container.attribute(&name) {
            println!("    {name:<7} = {value:?}");
        }
    }
    println!(
        "    容器世界矩阵平移 = {:?}",
        round3(world_matrix(container)?.w_axis.truncate().to_array())
    );

    let route = walk_route(container)?;
    for member in &route.members {
        println!(
            "\n  -- 构件 {} {} {:?}",
            member.refno(),
            member.element_type()?,
            member.stored_name()?
        );
        // HEIG 在列：`RPRO TLEN` 的求值就是拿它顶的，FTUB 的轴向缝要靠
        // 「HEIG 与两件实距之差」来分「目录求值错」与「模型本来就短一截」。
        for name in ["POS", "ORI", "ARRI", "LEAV", "ANGL", "DESP", "HEIG"] {
            println!(
                "      {name:<5} = {:?}",
                member.attribute(name).ok().flatten()
            );
        }
        let world = world_matrix(member)?;
        println!(
            "      世界矩阵平移 = {:?}",
            round3(world.w_axis.truncate().to_array())
        );

        let Some(spec) = member.get_element("SPRE")? else {
            println!("      SPRE 缺");
            continue;
        };
        let component = if spec.element_type()?.eq_ignore_ascii_case("SCOM") {
            Some(spec.clone())
        } else {
            spec.get_element("CATR")?
        };
        println!(
            "      SPRE → {} {}   目录件 → {}",
            spec.refno(),
            spec.element_type()?,
            component
                .as_ref()
                .map(|c| format!("{} {:?}", c.refno(), c.stored_name().ok().flatten()))
                .unwrap_or_else(|| "缺".to_string())
        );
        let Some(component) = component else { continue };
        let Some(set) = component.get_element("PTRE")? else {
            println!("      PTRE 缺");
            continue;
        };
        // 模板声明的属性全打出来：只挑自己想得到的名字，就永远发现不了
        // 那个真正存着方向的属性。
        for point in set.members()? {
            let attrs = point
                .attribute_names()?
                .iter()
                .filter_map(|name| {
                    point
                        .attribute(name)
                        .ok()
                        .flatten()
                        .map(|v| format!("{name}={v:?}"))
                })
                .collect::<Vec<_>>()
                .join("  ");
            println!(
                "      P点 {} {}  {attrs}",
                point.refno(),
                point.element_type()?
            );
            // PTCD 是 PTCA 的真方向所在（`DIR` 是常量占位）。先拿现成的两个
            // 解码器各试一遍，能读通就不用另起炉灶。
            if let Ok(Some(DescriptorValue::RawWords(words))) = point.attribute("PTCD") {
                let signed: Vec<i32> = words.iter().map(|w| *w as i32).collect();
                println!(
                    "          PTCD 按 PML 读 = {:?}；按轴规格读 = {:?}",
                    e3d_io::record::catalogue_pml::render(&signed),
                    e3d_io::record::axis_spec::render(&words),
                );
            }
        }
        for (label, end) in [
            (
                "到达",
                RouteEnd::MemberArrive {
                    member: member.clone(),
                    ppoint: ppoint_of(member, "ARRI"),
                },
            ),
            (
                "离开",
                RouteEnd::MemberLeave {
                    member: member.clone(),
                    ppoint: ppoint_of(member, "LEAV"),
                },
            ),
        ] {
            match resolve_end(&end, &CataloguePoints) {
                Ok(Some(p)) => println!(
                    "      {label}解出 pos={:?} dir={:?} bore={:?}",
                    round3(p.pos.to_array()),
                    round3(p.dir.to_array()),
                    p.bore
                ),
                Ok(None) => println!("      {label}解不出"),
                Err(error) => println!("      {label}读错: {error}"),
            }
        }
    }
    Ok(())
}

/// 复刻 `route` 私有的编号读法，只为 dump 用。
fn ppoint_of(member: &DbElement, name: &str) -> Option<u32> {
    match member.attribute(name) {
        Ok(Some(DescriptorValue::Word { raw, .. })) => Some(raw),
        Ok(Some(DescriptorValue::Int(v))) => u32::try_from(v).ok(),
        _ => None,
    }
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

/// 按「离开端是哪种构件」记槽位结局。
///
/// 这是验 `RPRO TLEN` 的关键账：`FTUB` 的 P2 距离整条来自
/// `PDIS = ( ATTRIB RPRO TLEN )`，而 `TLEN` 又是 `db_param_env` 拿该实例的
/// `HEIG` 顶上的。若这个顶法读错了，`FTUB` 的离开点就会落在下一站之外，
/// 该 noun 的「零长」比例会当场塌下来——它与下一站的位置是两个独立来源
/// （一个走目录求值，一个是下一个构件自己的 `POS`），对得上不可能是巧合。
fn note_leave_noun(tally: &mut Tally, end: &RouteEnd, outcome: &'static str) {
    let RouteEnd::MemberLeave { member, .. } = end else {
        return;
    };
    let Ok(noun) = member.element_type() else {
        return;
    };
    *tally.leave_outcome.entry((noun, outcome)).or_insert(0) += 1;
}

/// 按「离开端是哪种构件」记这条间隙的质量。只有有间隙的槽位走这里。
fn note_leave_noun_quality(
    tally: &mut Tally,
    end: &RouteEnd,
    lateral: &'static str,
    axis: &'static str,
) {
    let RouteEnd::MemberLeave { member, .. } = end else {
        return;
    };
    let Ok(noun) = member.element_type() else {
        return;
    };
    *tally
        .leave_noun_lateral
        .entry((noun.clone(), lateral))
        .or_insert(0) += 1;
    *tally.leave_noun_axis.entry((noun, axis)).or_insert(0) += 1;
}

/// 盯住 `--noun` 那一族有间隙的槽位：记槽长分布 + 逐条留证。
///
/// 质量表答得了「这一族的间隙歪不歪」，答不了「这些间隙是个什么东西」，而后者
/// 才是「该不该在这条链上出隐式管身」的判据。两种间隙在质量表上长得一模一样：
/// 相邻两件本该贴着装、却差了几毫米，是解错了位置，那几毫米不能出管；隔着一整
/// 根管的几百上千毫米，是这条链本来就该长出来的一根真管。差别只写在槽长上。
fn note_focus(
    tally: &mut Tally,
    end: &RouteEnd,
    container: &DbElement,
    length: f64,
    lateral: f64,
    pair_deg: f64,
    across: glam::DVec3,
) {
    let RouteEnd::MemberLeave { member, .. } = end else {
        return;
    };
    let Ok(noun) = member.element_type() else {
        return;
    };
    if tally.focus_noun.as_deref() != Some(noun.as_str()) {
        return;
    }
    *tally
        .focus_lengths
        .entry(length_bucket(length))
        .or_insert(0) += 1;
    let flat = (across.x * across.x + across.y * across.y).sqrt();
    tally.focus_detail.push((
        lateral,
        length,
        format!(
            "槽长 {length:>9.2}mm  横向 {lateral:>8.2}mm（水平 {flat:>7.2} 铅垂 {:>7.2}）\
             两端轴 {pair_deg:>6.2}°  离开件 {}  容器 {} {}",
            across.z,
            member.refno(),
            container.refno(),
            container.stored_name().ok().flatten().unwrap_or_default(),
        ),
    ));
}

fn check_container(container: &DbElement, tally: &mut Tally, sample: usize) -> anyhow::Result<()> {
    tally.containers.insert(container.refno());
    let route = walk_route(container)?;
    for slot in &route.tubes {
        tally.slots += 1;
        let ends = end_pair(&slot.from, &slot.to);
        *tally.by_ends.entry(ends).or_insert(0) += 1;

        let resolved = resolve_end(&slot.from, &CataloguePoints)
            .and_then(|from| Ok((from, resolve_end(&slot.to, &CataloguePoints)?)));
        let (from, to) = match resolved {
            Ok((Some(from), Some(to))) => (from, to),
            Ok(_) => {
                tally.unresolved += 1;
                continue;
            }
            Err(DbElementError::DbNotRegistered { dbno }) => {
                tally.db_not_open += 1;
                *tally.db_not_open_by_dbno.entry(dbno).or_insert(0) += 1;
                continue;
            }
            Err(DbElementError::NotFound { dbno, .. }) => {
                tally.db_not_open += 1;
                *tally.not_in_index_by_dbno.entry(dbno).or_insert(0) += 1;
                continue;
            }
            Err(error) => return Err(error.into()),
        };

        let length = tube_length(from.pos, to.pos);
        if length < ZERO_LENGTH_TOL_MM {
            tally.zero_length += 1;
            note_leave_noun(tally, &slot.from, "零长（离开点正落在下一站上）");
            continue;
        }
        note_leave_noun(tally, &slot.from, "有间隙");
        tally.measured += 1;
        tally.measured_containers.insert(container.refno());
        *tally.measured_by_ends.entry(ends).or_insert(0) += 1;

        // 共线度：折进 [0°, 90°]，同向反向都算对齐。
        let along = (to.pos - from.pos) / length;
        let from_deg = axis_deg(along, from.dir);
        let to_deg = axis_deg(along, to.dir);
        *tally
            .from_angle
            .entry((ends, bucket(from_deg)))
            .or_insert(0) += 1;
        *tally.to_angle.entry((ends, bucket(to_deg))).or_insert(0) += 1;
        if from_deg < 1.0 && to_deg < 1.0 {
            let pattern = match (
                along.dot(from.dir.normalize_or_zero()) > 0.0,
                along.dot(to.dir.normalize_or_zero()) > 0.0,
            ) {
                (true, true) => "from 顺 / to 顺",
                (true, false) => "from 顺 / to 逆",
                (false, true) => "from 逆 / to 顺",
                (false, false) => "from 逆 / to 逆",
            };
            *tally.signs.entry((ends, pattern)).or_insert(0) += 1;
        }

        // 横向偏移：to.pos 离「过 from.pos 沿 from.dir 的直线」有多远（mm）。
        // 与夹角是同一件事的两种量法，但这一种不随槽位长短放大。
        let lateral = lateral_mm(from.pos, from.dir, to.pos);
        let lateral_key = lateral_bucket(lateral);
        *tally.lateral.entry((ends, lateral_key)).or_insert(0) += 1;
        // 两端轴自己对不对得上，与位置无关。
        let pair_deg = axis_deg(from.dir, to.dir);
        let axis_key = bucket(pair_deg);
        *tally.axis_pair.entry((ends, axis_key)).or_insert(0) += 1;
        note_leave_noun_quality(tally, &slot.from, lateral_key, axis_key);
        // 横向那一段本身也要拆开。设计师手填的错开是按平面与标高分别取整的
        // （抬高 500 + 平移 200），合成之后是 538.52 这种看不出名堂的杂数；
        // 拆回水平与铅垂两截，「这是人填的」与「这是我们算歪的」才分得开。
        let across = (to.pos - from.pos)
            - from.dir.normalize_or_zero() * (to.pos - from.pos).dot(from.dir.normalize_or_zero());
        note_focus(
            tally, &slot.from, container, length, lateral, pair_deg, across,
        );
        if lateral.is_finite() && lateral > 1.0 {
            tally.offending_containers.insert(container.refno());
            let flat = (across.x * across.x + across.y * across.y).sqrt();
            let shape = match (flat > 1.0, across.z.abs() > 1.0) {
                (false, true) => "纯铅垂（XY 对齐，只差 Z）",
                (true, false) => "纯水平（Z 对齐，只差 XY）",
                (true, true) => "混合",
                (false, false) => "都不到 1mm（不该到这）",
            };
            *tally.lateral_shape.entry(shape).or_insert(0) += 1;
            if (lateral - lateral.round()).abs() < 1.0e-3 {
                *tally
                    .round_offsets
                    .entry(lateral.round() as i64)
                    .or_insert(0) += 1;
            }
            tally.worst.push((
                lateral,
                format!(
                    "{:>9.2}mm 横向  {ends}  槽长 {length:8.2}mm  {} {:?}\n\
                     {:>13}from.pos={:?} dir={:?}\n{:>13}  to.pos={:?} dir={:?}",
                    lateral,
                    container.refno(),
                    container.stored_name().ok().flatten().unwrap_or_default(),
                    "",
                    round3(from.pos.to_array()),
                    round3(from.dir.to_array()),
                    "",
                    round3(to.pos.to_array()),
                    round3(to.dir.to_array()),
                ),
            ));
        }

        let bucket = tally.outliers.entry(ends).or_default();
        if (from_deg > 30.0 || to_deg > 30.0) && bucket.len() < sample {
            bucket.push(format!(
                "{} {ends} 长 {length:.2}mm 出发端偏 {from_deg:.2}° 到达端偏 {to_deg:.2}°\n\
                 {:>8}from.pos={:?} from.dir={:?}\n{:>8}  to.pos={:?}   to.dir={:?}",
                container.refno(),
                "",
                round3(from.pos.to_array()),
                round3(from.dir.to_array()),
                "",
                round3(to.pos.to_array()),
                round3(to.dir.to_array()),
            ));
        }
    }
    Ok(())
}

/// 与**轴**的夹角（度），折进 `[0°, 90°]`：同向反向一视同仁。
fn axis_deg(along: glam::DVec3, dir: glam::DVec3) -> f64 {
    let dir = dir.normalize_or_zero();
    if dir == glam::DVec3::ZERO {
        return f64::NAN;
    }
    along.dot(dir).abs().clamp(0.0, 1.0).acos().to_degrees()
}

/// `to` 离「过 `from` 沿 `dir` 的直线」的垂距（mm）。`dir` 退化时给 NaN。
fn lateral_mm(from: glam::DVec3, dir: glam::DVec3, to: glam::DVec3) -> f64 {
    let dir = dir.normalize_or_zero();
    if dir == glam::DVec3::ZERO {
        return f64::NAN;
    }
    let delta = to - from;
    (delta - dir * delta.dot(dir)).length()
}

fn lateral_bucket(mm: f64) -> &'static str {
    if mm.is_nan() {
        return "≥ 100mm";
    }
    match LATERAL_BUCKETS.iter().position(|limit| mm < *limit) {
        Some(i) => LATERAL_KEYS[i],
        None => "≥ 100mm",
    }
}

fn length_bucket(mm: f64) -> &'static str {
    match LENGTH_BUCKETS.iter().position(|limit| mm < *limit) {
        Some(i) => LENGTH_KEYS[i],
        None => "≥ 10m",
    }
}

fn bucket(deg: f64) -> &'static str {
    if deg.is_nan() {
        return "方向缺失";
    }
    match BUCKETS.iter().position(|limit| deg < *limit) {
        Some(0) => "< 0.01°",
        Some(1) => "< 0.1°",
        Some(2) => "< 1°",
        Some(3) => "< 5°",
        Some(4) => "< 30°",
        _ => "≥ 30°",
    }
}

fn end_pair(from: &RouteEnd, to: &RouteEnd) -> &'static str {
    match (from, to) {
        (RouteEnd::ContainerHead(_), RouteEnd::ContainerTail(_)) => "容器头 → 容器尾",
        (RouteEnd::ContainerHead(_), _) => "容器头 → 构件到达",
        (_, RouteEnd::ContainerTail(_)) => "构件离开 → 容器尾",
        _ => "构件离开 → 构件到达",
    }
}

fn round3(v: [f64; 3]) -> [f64; 3] {
    v.map(|x| (x * 1000.0).round() / 1000.0)
}

/// 某个 noun 落在 `keys` 那几个分箱里的条数。
fn noun_ok(
    hist: &BTreeMap<(String, &'static str), usize>,
    noun: &str,
    keys: &[&'static str],
) -> usize {
    keys.iter()
        .map(|key| hist.get(&(noun.to_string(), *key)).copied().unwrap_or(0))
        .sum()
}

fn report(tally: &mut Tally) {
    println!("\n================ 管身走向自证 ================");
    println!("槽位 {}", tally.slots);
    println!(
        "  两端解出且非零长 {:>6}  ← 下面的夹角只统计这些",
        tally.measured
    );
    println!("  零长             {:>6}", tally.zero_length);
    println!("  解不出 P 点      {:>6}", tally.unresolved);
    println!("  P 点所在库没开   {:>6}", tally.db_not_open);
    for (dbno, count) in &tally.db_not_open_by_dbno {
        println!("      db {dbno:<6} {count:>6}  没注册    ← 用 --catalogue 把它开上");
    }
    for (dbno, count) in &tally.not_in_index_by_dbno {
        println!("      db {dbno:<6} {count:>6}  开错文件  ← 自报同号，但索引里没有那些 refno");
    }

    println!("\n端型组合（槽位 / 其中量出）：");
    for (kind, count) in &tally.by_ends {
        println!(
            "  {kind:<20} {count:>6} / {:>4}",
            tally.measured_by_ends.get(kind).copied().unwrap_or(0)
        );
    }

    // 分端型看：错在容器端还是构件端，只有拆开才看得出来。
    for (title, hist) in [
        ("出发端轴与走向的共线度", &tally.from_angle),
        ("到达端轴与走向的共线度", &tally.to_angle),
    ] {
        println!("\n{title}：");
        for (kind, total) in &tally.measured_by_ends {
            let row = BUCKET_KEYS
                .iter()
                .map(|key| {
                    let count = hist.get(&(*kind, *key)).copied().unwrap_or(0);
                    format!("{key} {count:>4}")
                })
                .collect::<Vec<_>>()
                .join("  ");
            println!("  {kind:<20} n={total:<5} {row}");
        }
    }

    // 位置那一路单独量。夹角会被短槽位放大，毫米不会。
    println!("\n横向偏移（to.pos 离 from 那条轴多远，mm；与槽位长短无关）：");
    for (kind, total) in &tally.measured_by_ends {
        let row = LATERAL_KEYS
            .iter()
            .map(|key| {
                let count = tally.lateral.get(&(*kind, *key)).copied().unwrap_or(0);
                format!("{key} {count:>4}")
            })
            .collect::<Vec<_>>()
            .join("  ");
        println!("  {kind:<20} n={total:<5} {row}");
    }

    println!("\n两端轴自己的夹角（只看朝向，不看位置；反平行折进 [0°,90°]）：");
    for (kind, total) in &tally.measured_by_ends {
        let row = BUCKET_KEYS
            .iter()
            .map(|key| {
                let count = tally.axis_pair.get(&(*kind, *key)).copied().unwrap_or(0);
                format!("{key} {count:>4}")
            })
            .collect::<Vec<_>>()
            .join("  ");
        println!("  {kind:<20} n={total:<5} {row}");
    }

    println!("\n朝向约定（只统计两端都共线的那些）：");
    for ((kind, pattern), count) in &tally.signs {
        println!("  {kind:<20} {pattern:<18} {count:>5}");
    }

    println!("\n偏差 > 30° 的样本（P 点求解出错的话就在这里现形）：");
    for (kind, lines) in &tally.outliers {
        if lines.is_empty() {
            continue;
        }
        println!("  【{kind}】");
        for line in lines {
            println!("  {line}");
        }
    }

    let aligned = |hist: &BTreeMap<(&'static str, &'static str), usize>| -> usize {
        hist.iter()
            .filter(|((_, bucket), _)| ALIGNED_KEYS.contains(bucket))
            .map(|(_, count)| count)
            .sum()
    };
    let (from_ok, to_ok) = (aligned(&tally.from_angle), aligned(&tally.to_angle));
    println!(
        "\n判据一（夹角）：共线（< 1°）占比 —— 出发端 {}/{}（{:.1}%），到达端 {}/{}（{:.1}%）。",
        from_ok,
        tally.measured,
        100.0 * from_ok as f64 / tally.measured.max(1) as f64,
        to_ok,
        tally.measured,
        100.0 * to_ok as f64 / tally.measured.max(1) as f64,
    );

    let axis_ok: usize = tally
        .axis_pair
        .iter()
        .filter(|((_, b), _)| ALIGNED_KEYS.contains(b))
        .map(|(_, c)| c)
        .sum();
    let pos_ok: usize = tally
        .lateral
        .iter()
        .filter(|((_, b), _)| LATERAL_OK.contains(b))
        .map(|(_, c)| c)
        .sum();
    // 可解 = 两端 P 点都解出来了的槽位。库没开与解不出 P 点的不算——那两类
    // 是配置与欠账，混进分母会把求解正确率稀释得没法读。
    let resolvable = tally.zero_length + tally.measured;
    println!(
        "判据二（朝向）：两端轴反平行（< 1°）{}/{}（{:.1}%）——这一条只考朝向解码。\n\
         判据三（位置）：横向偏移 < 1mm 的 {}/{}（{:.1}%）。\n\n\
         **判据三的分母是有间隙的槽位，不要拿它当求解正确率读。** 求解正确率看下面\n\
         那条：把两端重合的槽位排除在外再算「对了几条」，等于专挑模型自己歪的地方\n\
         打分，会把脏数据读成我们的错误率。",
        axis_ok,
        tally.measured,
        100.0 * axis_ok as f64 / tally.measured.max(1) as f64,
        pos_ok,
        tally.measured,
        100.0 * pos_ok as f64 / tally.measured.max(1) as f64,
    );

    // 求解正确率的主证据。两端各自独立走完
    // `SPRE→SPCO→CATR→SCOM→PTRE→PTSE→PTAX/PTCA` 再乘各自构件的世界矩阵，
    // 还能落到 ZERO_LENGTH_TOL_MM 以内——目录链、PML 求值、轴解码、ORI 折叠、
    // 世界矩阵，任何一环有系统性错误，这批槽位都会当场散架。所以这个比例才是
    // 「链对不对」的答案，判据三只是「剩下那些有间隙的槽位歪不歪」。
    let skewed = tally.measured - pos_ok;
    println!(
        "\n求解正确率：可解槽位 {resolvable} 条（= 零长 {} + 有间隙 {}；\
         库没开的 {} 条与解不出 P 点的 {} 条不入分母）中，\n\
         \x20 {} 条（{:.1}%）两端重合到 {ZERO_LENGTH_TOL_MM}mm 以内。\n\
         模型体检：{} 条（占可解槽位 {:.1}%）两端解得出、但走向是歪的。\
         这一批是模型自己的几何，\n\
         \x20 不是求解错误——同一套链在上面那 {} 条上是精确的。",
        tally.zero_length,
        tally.measured,
        tally.db_not_open,
        tally.unresolved,
        tally.zero_length,
        100.0 * tally.zero_length as f64 / resolvable.max(1) as f64,
        skewed,
        100.0 * skewed as f64 / resolvable.max(1) as f64,
        tally.zero_length,
    );

    println!("\n按离开端构件分的槽位结局（`RPRO TLEN` 的验算在这里）：");
    let mut nouns: BTreeMap<&str, (usize, usize)> = BTreeMap::new();
    for ((noun, outcome), count) in &tally.leave_outcome {
        let slot = nouns.entry(noun.as_str()).or_insert((0, 0));
        match outcome.starts_with("零长") {
            true => slot.0 += *count,
            false => slot.1 += *count,
        }
    }
    let mut rows: Vec<_> = nouns.into_iter().collect();
    rows.sort_by(|a, b| (b.1.0 + b.1.1).cmp(&(a.1.0 + a.1.1)).then(a.0.cmp(b.0)));
    for (noun, (zero, gap)) in rows.iter().take(12) {
        let total = zero + gap;
        println!(
            "  {noun:<8} 零长 {zero:>5} / 共 {total:>5}（{:.1}%），有间隙 {gap}",
            100.0 * *zero as f64 / total.max(1) as f64
        );
    }

    // 上面那张表**不能**当质量表读。零长率低只说明「这一族元件常跟下一站隔着
    // 距离」，而附着点本来就该那样——间隙不是错，歪掉才是。所以同一批间隙再按
    // 元件族量一次轴与位置：解错了 P 点，歪的比例会在这张表上单独凸出来；本来
    // 就该有间隙的，这张表跟别人一样漂亮。
    println!("\n有间隙的那些，质量如何（这张才是质量表；上面那张是形态表）：");
    for (noun, (_, gap)) in rows.iter().take(12) {
        if *gap == 0 {
            continue;
        }
        let axis = noun_ok(&tally.leave_noun_axis, noun, &ALIGNED_KEYS);
        let pos = noun_ok(&tally.leave_noun_lateral, noun, &LATERAL_OK);
        println!(
            "  {noun:<8} n={gap:<5} 两端轴反平行 {axis:>4}（{:>5.1}%）   横向偏移 < 1mm {pos:>4}（{:>5.1}%）",
            100.0 * axis as f64 / (*gap).max(1) as f64,
            100.0 * pos as f64 / (*gap).max(1) as f64,
        );
    }

    if let Some(noun) = tally.focus_noun.clone() {
        let total: usize = tally.focus_lengths.values().sum();
        println!(
            "\n【--noun {noun}】它有间隙的 {total} 条，间隙有多长：\n\
             \x20 几毫米 = 两件本该贴着装却差开了，是解错位置；几百毫米往上 = 中间\n\
             \x20 本来就有一根真管。这两种在上面那张质量表上分不出来。"
        );
        for key in LENGTH_KEYS {
            let count = tally.focus_lengths.get(key).copied().unwrap_or(0);
            println!(
                "  {key:<8} {count:>5}（{:>5.1}%）",
                100.0 * count as f64 / total.max(1) as f64
            );
        }
        tally
            .focus_detail
            .sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
        // 只列歪着的那些，不列全部：这一族正常的样本彼此长得一模一样，列出来
        // 只会把要看的那几条挤下去。上限 30 条，够看清是集中在一族容器上还是摊开的。
        let skewed = tally
            .focus_detail
            .iter()
            .filter(|(mm, _, _)| *mm > 1.0)
            .count();
        println!(
            "  横向偏移 > 1mm 的 {skewed}/{total} 条，列前 {}：",
            skewed.min(30)
        );
        for (_, _, line) in tally.focus_detail.iter().take(skewed.min(30)) {
            println!("    {line}");
        }
        // 第二刀：短槽。零长判到 ZERO_LENGTH_TOL_MM 为止，从那往上到几毫米的缝
        // 是「两件本该贴着装」的强候选——解错位置（目录求值差一口、世界矩阵少一环）
        // 会做出这种缝，设计师手滑挪了 0.1mm 也会。分不出谁是谁就谈不上收口，
        // 所以逐条列出来供 --dump 追查。
        let mut shortest: Vec<&(f64, f64, String)> = tally
            .focus_detail
            .iter()
            .filter(|(_, length, _)| *length < 10.0)
            .collect();
        shortest.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        println!(
            "  槽长 < 10mm 的 {} 条（零长线 {ZERO_LENGTH_TOL_MM}mm 以上、贴装缝候选），列前 {}：",
            shortest.len(),
            shortest.len().min(12)
        );
        for (_, _, line) in shortest.iter().take(12) {
            println!("    {line}");
        }
    }

    let shaped: usize = tally.lateral_shape.values().sum();
    println!("\n偏移形态（横向 > 1mm 的 {shaped} 条）：");
    let mut rows: Vec<_> = tally.lateral_shape.iter().collect();
    rows.sort_by(|a, b| b.1.cmp(a.1).then(a.0.cmp(b.0)));
    for (shape, count) in rows {
        println!("  {count:>6}x  {shape}");
    }
    println!(
        "涉及容器 {}/{}（{:.1}%，分母只算「至少量出一条槽位」的容器；全部容器 {}）",
        tally.offending_containers.len(),
        tally.measured_containers.len(),
        100.0 * tally.offending_containers.len() as f64
            / tally.measured_containers.len().max(1) as f64,
        tally.containers.len(),
    );

    let round: usize = tally.round_offsets.values().sum();
    println!(
        "\n偏移量恰好落在整毫米上的 {round}/{shaped} 条 —— 整毫米几乎必是设计师手填，\n\
         坐标算错很难恰好凑整。出现最多的几个值："
    );
    let mut rows: Vec<_> = tally.round_offsets.iter().collect();
    rows.sort_by(|a, b| b.1.cmp(a.1).then(a.0.cmp(b.0)));
    for (value, count) in rows.iter().take(10) {
        println!("  {count:>6}x  {value}mm");
    }

    tally
        .worst
        .sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
    println!(
        "\n横向偏移 > 1mm 的共 {} 条，最大的 {} 条：",
        tally.worst.len(),
        tally.worst.len().min(12),
    );
    for (_, line) in tally.worst.iter().take(12) {
        println!("  {line}");
    }
}

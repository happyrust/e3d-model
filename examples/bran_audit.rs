//! BRAN 逐条体检：一个库里每一条 BRAN 的路由成员与隐式管身到底生成成什么样。
//!
//! `gen_ams --route-only` 的 `routes.json` 只给每条 BRAN 一行计数（成员几件、生成几件、
//! 管身几根），管身账（`ImpliedTubes`）又只在整库层面记退化**计数**——一条 BRAN 少了
//! 哪一根管、卡在哪一步，两处都答不出来。本探针走**同一条生产路径**
//! （`generate_subtree_with_cache` + `walk_route` / `resolve_end` / `build_tube`），
//! 把每条 BRAN 展开成成员表 + 槽位表，再按下面的口径下结论：
//!
//! - **成员**：`Category::RouteMember` 的每一件都该有几何，除非它的目录几何被表现规则
//!   （`display-filtered by …`）或 Core 式局部失败整件收走——那两种在 `report.notes` 里
//!   有名有姓，算「有依据地没出」；其余没出几何的成员算缺口。
//! - **槽位**（`walk_route` 的串接，ATTA 不推进链）：两端 P 点都解得出且长度 ≥ 1 mm 的
//!   必须建成；`ZeroLength` 是正常的对接（FTUB↔FTUB）；`missing_ppoint` / `NoDirection` /
//!   `NoRadius` / 头尾点解不出是缺口（尾点未设导致的 `NoRadius` 归数据）；两端通径不一致留痕。
//!   **属主是 `HVAC` 的分支反过来**：风管没有隐式管身，建出来的每一根都是缺陷。
//! - **几何**：每件实体体积 > 0、AABB 有限、分量数 1（多分量记 warn）。
//!
//! 结论按属主 noun（PIPE / HVAC）分开汇总——`BRAN` 一个 noun 两个学科共用。
//!
//! ```text
//! cargo run --release --example bran_audit -- --attlib <attlib.dat> --db <ams8000_0001> \
//!     --template <dir> --catalogue <ams7000_0001> [--catalogue ...] \
//!     --project-dir <dir> [--project-dir ...] [--out out/bran-audit] [--only 24384/2774]
//! ```

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use anyhow::{Context, bail};
use e3d_io::db_element::{DbFilePin, DbSet, template_file_for};
use e3d_io::engine::{ReadOnlyEngine, ScanTier};
use e3d_io::refno::RefNo;
use e3d_model::GeometryId;
use e3d_model::catalogue::CatalogueMeshCache;
use e3d_model::catalogue_point::CataloguePoints;
use e3d_model::category::{Category, classify, derives_implied_tubes};
use e3d_model::db_discovery::DirectoryDbResolver;
use e3d_model::pipeline::{GenerationOutcome, generate_subtree_with_cache};
use e3d_model::route::{
    ArriveCheck, RouteEnd, SlotAlignment, TubeOutcome, TubeSection, ZERO_LENGTH_TOL_MM,
    bore_mismatch, build_tube, resolve_end, slot_alignment, slot_ordinals, tube_length,
    tube_outer_diameter, walk_route,
};

struct Args {
    attlib: PathBuf,
    db_file: PathBuf,
    template: PathBuf,
    sesno: Option<u32>,
    out: PathBuf,
    catalogues: Vec<PathBuf>,
    project_dirs: Vec<PathBuf>,
    only: Option<RefNo>,
}

fn parse_args() -> anyhow::Result<Args> {
    let mut attlib = None;
    let mut db_file = None;
    let mut template = None;
    let mut sesno = None;
    let mut out = PathBuf::from("out/bran-audit");
    let mut catalogues = Vec::new();
    let mut project_dirs = Vec::new();
    let mut only = None;
    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--attlib" => attlib = Some(PathBuf::from(value()?)),
            "--db" => db_file = Some(PathBuf::from(value()?)),
            "--template" => template = Some(PathBuf::from(value()?)),
            "--sesno" => sesno = Some(value()?.parse().context("--sesno 要无符号整数")?),
            "--out" => out = PathBuf::from(value()?),
            "--catalogue" => catalogues.push(PathBuf::from(value()?)),
            "--project-dir" => project_dirs.push(PathBuf::from(value()?)),
            "--only" => only = Some(parse_refno(&value()?)?),
            other => bail!("不认识的参数 {other}"),
        }
    }
    Ok(Args {
        attlib: attlib.context("缺 --attlib")?,
        db_file: db_file.context("缺 --db")?,
        template: template.context("缺 --template")?,
        sesno,
        out,
        catalogues,
        project_dirs,
        only,
    })
}

fn parse_refno(text: &str) -> anyhow::Result<RefNo> {
    let trimmed = text.trim_start_matches('=');
    let (w0, w1) = trimmed
        .split_once('/')
        .with_context(|| format!("refno 要 `w0/w1` 形式，得到 {text}"))?;
    Ok(RefNo::new(w0.trim().parse()?, w1.trim().parse()?))
}

fn scan_brans(db_file: &Path, sesno: Option<u32>, set: &Arc<DbSet>) -> anyhow::Result<Vec<RefNo>> {
    let mut engine = match sesno {
        Some(s) => ReadOnlyEngine::open_at(db_file, s),
        None => ReadOnlyEngine::open(db_file),
    }
    .with_context(|| format!("开库扫 BRAN {}", db_file.display()))?;
    let mut roots = Vec::new();
    for scanned in engine.scan_elements(ScanTier::Header)? {
        let refno = scanned?.refno;
        if set.element(refno).element_type()? == "BRAN" {
            roots.push(refno);
        }
    }
    Ok(roots)
}

#[derive(serde::Serialize)]
struct MemberRow {
    refno: String,
    noun: String,
    name: Option<String>,
    spre: bool,
    /// 这件东西是否推进隐式管身链（`walk_route` 的站点）。
    station: bool,
    generated: bool,
    /// 没出几何时的依据（来自 `report.notes` / `skipped` / `failed`），空 = 无依据的缺口。
    reasons: Vec<String>,
    volume_mm3: Option<f64>,
    components: Option<usize>,
}

#[derive(serde::Serialize)]
struct SlotRow {
    ordinal: u32,
    from: String,
    to: String,
    /// `built` / `zero_length` / `misaligned` / `reversed` / `missing_ppoint` / `no_direction` /
    /// `no_radius` / `failed` / `db_not_open` / `target_not_found`
    outcome: String,
    length_mm: Option<f64>,
    radius_mm: Option<f64>,
    bore_mismatch: bool,
    /// 管身方向与离开侧 P 点方向的夹角（度，折进 0..=180）；> 90 说明管往回长（重叠假管的形状）。
    backtrack_deg: Option<f64>,
    /// 管身方向与到达侧 P 点方向（取反）的夹角（度）；容器尾不查，为 None。
    arrive_deg: Option<f64>,
    /// 两端解出的 P 点（世界系 pos / dir / bore），解不出为 None。
    from_point: Option<([f64; 3], [f64; 3], Option<f64>)>,
    to_point: Option<([f64; 3], [f64; 3], Option<f64>)>,
}

#[derive(serde::Serialize)]
struct BranRow {
    refno: String,
    name: Option<String>,
    /// 属主 noun（`PIPE` / `HVAC` / …）：同一个 `BRAN` noun 两个学科共用，ams7997 里
    /// 管道与风管各占一半，不分开看数字就是两本账搅在一起。
    owner_noun: String,
    head: bool,
    tail: bool,
    members: Vec<MemberRow>,
    slots: Vec<SlotRow>,
    generated_elements: usize,
    failed: Vec<String>,
    verdict: String,
    problems: Vec<String>,
}

fn end_label(end: &RouteEnd) -> String {
    match end {
        RouteEnd::ContainerHead(c) => format!("{}:HEAD", c.refno()),
        RouteEnd::ContainerTail(c) => format!("{}:TAIL", c.refno()),
        RouteEnd::MemberLeave { member, ppoint } => {
            format!(
                "{}:LEAVE(P{})",
                member.refno(),
                ppoint.map_or("?".into(), |p| p.to_string())
            )
        }
        RouteEnd::MemberArrive { member, ppoint } => {
            format!(
                "{}:ARRIVE(P{})",
                member.refno(),
                ppoint.map_or("?".into(), |p| p.to_string())
            )
        }
    }
}

fn audit_bran(
    set: &Arc<DbSet>,
    refno: RefNo,
    cache: &mut CatalogueMeshCache,
) -> anyhow::Result<BranRow> {
    let bran = set.element(refno);
    let name = bran.stored_name()?;
    let owner_noun = match bran.owner()? {
        Some(owner) => owner.element_type()?,
        None => "<none>".to_string(),
    };
    // 与生产同一个谓词：风管分支（属主 HVAC）没有隐式管身——直段是显式 STRT，E3D 对间隙
    // 只报数据一致性错误、不补画（`evidence/20260902-hvac-ctray-census/
    // rvm-BRAN-24381-30598-primitives.json` 只有 4 Box + 1 Pyramid）。这类容器的槽位只
    // 列出来看间隙、不建；生产若建了任何一根就是缺陷。
    let derives_tubes = derives_implied_tubes(&bran)?;
    let GenerationOutcome {
        elements, report, ..
    } = generate_subtree_with_cache(bran.clone(), cache)?;

    // 成员 → 生成结果。
    let mut generated: BTreeMap<String, (f64, usize)> = BTreeMap::new();
    let mut built_tubes: BTreeMap<(String, String, u32), (f64, f64)> = BTreeMap::new();
    for element in &elements {
        match &element.geometry_id {
            GeometryId::ImpliedTube {
                route_ordinal,
                from_refno,
                to_refno,
                ..
            } => {
                let (mut length, mut radius) = (f64::NAN, f64::NAN);
                for note in &element.notes {
                    if let Some(rest) = note.strip_prefix("implied tube length=") {
                        let mut parts = rest.split(' ');
                        if let Some(l) = parts.next() {
                            length = l.trim_end_matches("mm").parse().unwrap_or(f64::NAN);
                        }
                        if let Some(r) = parts.next().and_then(|p| p.strip_prefix("radius=")) {
                            radius = r.trim_end_matches("mm").parse().unwrap_or(f64::NAN);
                        }
                    }
                }
                built_tubes.insert(
                    (from_refno.clone(), to_refno.clone(), *route_ordinal),
                    (length, radius),
                );
            }
            GeometryId::Element { .. } => {
                let entry = generated
                    .entry(element.refno.to_string())
                    .or_insert((0.0, 0));
                entry.0 += element.volume();
                entry.1 += element.components().len();
            }
        }
    }
    let mut incidents: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for incident in report
        .notes
        .iter()
        .chain(report.skipped.iter())
        .chain(report.failed.iter())
        .chain(report.negatives_skipped.iter())
    {
        incidents
            .entry(incident.refno.clone())
            .or_default()
            .push(incident.detail.clone());
    }

    let mut problems = Vec::new();
    let mut members = Vec::new();
    let mut station_count = 0usize;
    for member in bran.members()? {
        let noun = member.element_type()?;
        let is_route_member = matches!(classify(&noun), Category::RouteMember);
        let station = is_route_member && noun != "ATTA";
        if station {
            station_count += 1;
        }
        let key = member.refno().to_string();
        let produced = generated.get(&key).copied();
        let mut reasons = incidents.get(&key).cloned().unwrap_or_default();
        // 成员自己的目录成员被逐件过滤时，事故记在目录图元 refno 上、正文里带成员 refno。
        if produced.is_none() && reasons.is_empty() {
            for (_, details) in incidents.iter() {
                for d in details {
                    if d.contains(&key) {
                        reasons.push(d.clone());
                    }
                }
            }
        }
        if is_route_member && produced.is_none() && reasons.is_empty() {
            let spre = member.get_element("SPRE")?.is_some();
            if !spre {
                // 没有 SPRE 就没有目录链：E3D 同样不出几何。数据侧的缺，但报告里该有一笔。
                problems.push(format!(
                    "[data] 成员 {key} {noun} 无 SPRE，无目录几何（报告无记账）"
                ));
            } else {
                problems.push(format!(
                    "[account] 成员 {key} {noun} 没有几何且报告无任何记账依据"
                ));
            }
        }
        if let Some((volume, _components)) = produced {
            // 目录装配多分量是常态（E3D RVM 也按图元逐个导出），不算问题。
            if !(volume > 0.0) || !volume.is_finite() {
                problems.push(format!("[bad] 成员 {key} {noun} 体积非正/非有限: {volume}"));
            }
        }
        members.push(MemberRow {
            refno: key,
            noun,
            name: member.stored_name()?,
            spre: member.get_element("SPRE")?.is_some(),
            station,
            generated: produced.is_some(),
            reasons,
            volume_mm3: produced.map(|g| g.0),
            components: produced.map(|g| g.1),
        });
    }

    // 槽位：与生产同一份 walk_route / slot_ordinals；建成的从生成结果里认，其余现场分类。
    let route = walk_route(&bran)?;
    let ordinals = slot_ordinals(&route.tubes);
    let resolver = CataloguePoints;
    let mut slots = Vec::new();
    let mut head = false;
    let mut tail = false;
    for (slot, ordinal) in route.tubes.iter().zip(ordinals) {
        let (from_station, to_station) = slot.station_refnos();
        let key = (from_station.to_string(), to_station.to_string(), ordinal);
        let from = resolve_end(&slot.from, &resolver);
        let to = resolve_end(&slot.to, &resolver);
        if matches!(slot.from, RouteEnd::ContainerHead(_)) {
            head = matches!(from, Ok(Some(_)));
        }
        if matches!(slot.to, RouteEnd::ContainerTail(_)) {
            tail = matches!(to, Ok(Some(_)));
        }
        let mut row = SlotRow {
            ordinal,
            from: end_label(&slot.from),
            to: end_label(&slot.to),
            outcome: String::new(),
            length_mm: None,
            radius_mm: None,
            bore_mismatch: false,
            backtrack_deg: None,
            arrive_deg: None,
            from_point: None,
            to_point: None,
        };
        let as_point =
            |p: &e3d_model::route::RoutePoint| (p.pos.to_array(), p.dir.to_array(), p.bore);
        if let Ok(Some(p)) = &from {
            row.from_point = Some(as_point(p));
        }
        if let Ok(Some(p)) = &to {
            row.to_point = Some(as_point(p));
        }
        let classify_error = |e: &e3d_io::db_element::DbElementError| match e {
            e3d_io::db_element::DbElementError::DbNotRegistered { .. } => "db_not_open",
            e3d_io::db_element::DbElementError::NotFound { .. } => "target_not_found",
            _ => "resolve_error",
        };
        match (&from, &to) {
            (Ok(Some(from)), Ok(Some(to))) => {
                row.bore_mismatch = bore_mismatch(from, to);
                let axis = to.pos - from.pos;
                if axis.length() > 0.0 && from.dir.length() > 0.0 {
                    let cos = axis.normalize().dot(from.dir.normalize()).clamp(-1.0, 1.0);
                    row.backtrack_deg = Some(cos.acos().to_degrees());
                }
                let arrive = match &slot.to {
                    RouteEnd::MemberArrive { .. } | RouteEnd::MemberLeave { .. } => {
                        ArriveCheck::Member
                    }
                    RouteEnd::ContainerTail(container) => {
                        if container.get_direction("TDIR")?.is_some() {
                            ArriveCheck::Member
                        } else {
                            ArriveCheck::Skip
                        }
                    }
                    RouteEnd::ContainerHead(_) => ArriveCheck::Skip,
                };
                if axis.length() > 0.0 && to.dir.length() > 0.0 && arrive == ArriveCheck::Member {
                    let cos = axis.normalize().dot(-to.dir.normalize()).clamp(-1.0, 1.0);
                    row.arrive_deg = Some(cos.acos().to_degrees());
                }
                // 生产路径的对齐门（E3D 只画两端共轴且朝前的槽位）：这里同一把尺子先量一遍，
                // 被门挡下的槽位是数据形态，不是缺口。
                let gate = if tube_length(from.pos, to.pos) >= ZERO_LENGTH_TOL_MM {
                    Some(slot_alignment(from, to, arrive))
                } else {
                    None
                };
                // 与生产路径相同的原生 E3D 绘制门。坐标本身可建出圆柱并不代表 E3D 会画：
                // 两端通径不等（`build/LIMPOK`，每个槽位都查），或头槽位上到达件
                // ORIL=false / BRAN 没有 HSTU/HSRO 时，原生 RVM 都没有那根管。
                let mut native_gate = None;
                if row.bore_mismatch {
                    native_gate = Some("bore mismatch");
                }
                if let RouteEnd::ContainerHead(departure) = &slot.from {
                    if let RouteEnd::MemberArrive { member, .. } = &slot.to {
                        match member.get_bool("ORIL") {
                            Ok(Some(false)) => {
                                native_gate.get_or_insert("head arrival orientation unset");
                            }
                            Ok(_)
                            | Err(e3d_io::db_element::DbElementError::UnknownAttribute {
                                ..
                            }) => {}
                            Err(error) => return Err(error.into()),
                        }
                    }
                    if departure.element_type()?.eq_ignore_ascii_case("BRAN") {
                        let head_stub_null = match departure.get_ref("HSTU") {
                            Ok(value) => value.is_none(),
                            Err(e3d_io::db_element::DbElementError::UnknownAttribute {
                                ..
                            }) => false,
                            Err(error) => return Err(error.into()),
                        };
                        if head_stub_null {
                            let alternate_null = match departure.get_ref("HSRO") {
                                Ok(value) => value.is_none(),
                                Err(e3d_io::db_element::DbElementError::UnknownAttribute {
                                    ..
                                }) => true,
                                Err(error) => return Err(error.into()),
                            };
                            if alternate_null {
                                native_gate.get_or_insert("head stub unset");
                            }
                        }
                    }
                }
                if let Some((length, radius)) = built_tubes.get(&key) {
                    row.outcome = "built".into();
                    row.length_mm = Some(*length);
                    row.radius_mm = Some(*radius);
                    if !derives_tubes {
                        problems.push(format!(
                            "[bad] 槽位 {} → {} 不派生管身的容器（属主 {owner_noun}）却建了隐式管身（长 {length:.1}mm，r={radius:.1}）：E3D 对 HVAC 不画管身（rvm-BRAN-24381-30598 参照）",
                            row.from, row.to
                        ));
                    }
                } else if !derives_tubes {
                    // 不派生管身的容器：槽位只记两端间距，供看风管间隙用，不算缺口。
                    row.outcome = "not_derived".into();
                    row.length_mm = Some((to.pos - from.pos).length());
                } else if let Some(SlotAlignment::Misaligned { deviation_deg }) = gate {
                    row.outcome = "misaligned".into();
                    row.length_mm = Some(tube_length(from.pos, to.pos));
                    problems.push(format!(
                        "[data] 槽位 {} → {} 偏轴 {deviation_deg:.2}°（长 {:.1}mm）：E3D 不画",
                        row.from,
                        row.to,
                        row.length_mm.unwrap_or(f64::NAN)
                    ));
                } else if let Some(SlotAlignment::Reversed { deviation_deg }) = gate {
                    row.outcome = "reversed".into();
                    row.length_mm = Some(tube_length(from.pos, to.pos));
                    problems.push(format!(
                        "[data] 槽位 {} → {} 反向（{deviation_deg:.0}°，长 {:.1}mm）：负管长形态，E3D 不画",
                        row.from,
                        row.to,
                        row.length_mm.unwrap_or(f64::NAN)
                    ));
                } else if let Some(reason) = native_gate {
                    row.outcome = "native_filtered".into();
                    row.length_mm = Some(tube_length(from.pos, to.pos));
                    problems.push(format!(
                        "[data] 槽位 {} → {} 被 E3D 绘制门挡下（{reason}，长 {:.1}mm）",
                        row.from,
                        row.to,
                        row.length_mm.unwrap_or(f64::NAN)
                    ));
                } else {
                    match build_tube(
                        from,
                        to,
                        TubeSection::round(tube_outer_diameter(None, None, to.bore)),
                    ) {
                        TubeOutcome::Built(tube) => {
                            // 生产路径没建、这里却建得出：只可能是外径来源不同，照样算缺口。
                            row.outcome = "built_here_only".into();
                            row.length_mm = Some(tube.length_mm);
                            row.radius_mm = tube.radius_mm();
                            problems.push(format!(
                                "[bad] 槽位 {} → {} 生产未建但现场可建（长 {:.1}mm）",
                                row.from, row.to, tube.length_mm
                            ));
                        }
                        TubeOutcome::ZeroLength { length_mm } => {
                            row.outcome = "zero_length".into();
                            row.length_mm = Some(length_mm);
                        }
                        TubeOutcome::NoDirection => {
                            row.outcome = "no_direction".into();
                            problems
                                .push(format!("[bad] 槽位 {} → {} 定不出朝向", row.from, row.to));
                        }
                        TubeOutcome::NoRadius => {
                            row.outcome = "no_radius".into();
                            // 外径先取离开侧管子目录件、取不到回落到达侧通径；两边都没有只在
                            // 到达端是通径 0 的容器尾点时出现——尾点没设（TPOS 原点、TBOR 0），
                            // E3D 同样画不出零径管，是数据缺不是我们的账。
                            let tail_unset = matches!(slot.to, RouteEnd::ContainerTail(_))
                                && !to.bore.is_some_and(|bore| bore > 0.0);
                            if tail_unset {
                                problems.push(format!(
                                    "[data] 槽位 {} → {} 尾点未设（TBOR 0 / 未设），两侧都没有外径来源",
                                    row.from, row.to
                                ));
                            } else {
                                problems.push(format!(
                                    "[bad] 槽位 {} → {} 取不到半径",
                                    row.from, row.to
                                ));
                            }
                        }
                        TubeOutcome::Failed(error) => {
                            row.outcome = format!("failed: {error}");
                            problems.push(format!(
                                "[bad] 槽位 {} → {} 建体失败: {error}",
                                row.from, row.to
                            ));
                        }
                    }
                }
                // 生产路径已经不建偏轴 / 反向的管身；这里若还看到 built 且偏轴，说明生产与
                // 现场的门不是同一把尺子——那是 bad。
                if row.outcome == "built"
                    && matches!(
                        gate,
                        Some(SlotAlignment::Misaligned { .. } | SlotAlignment::Reversed { .. })
                    )
                {
                    problems.push(format!(
                        "[bad] 槽位 {} → {} 生产建了管身，但按对齐门它该被挡下（离开侧 {:.2}°，到达侧 {:?}）",
                        row.from,
                        row.to,
                        row.backtrack_deg.unwrap_or(f64::NAN),
                        row.arrive_deg
                    ));
                }
            }
            (Err(e), _) | (_, Err(e)) => {
                row.outcome = classify_error(e).into();
                problems.push(format!("[data] 槽位 {} → {}: {}", row.from, row.to, e));
            }
            (Ok(None), _) | (_, Ok(None)) => {
                row.outcome = "missing_ppoint".into();
                let which = match (&from, &to) {
                    (Ok(None), Ok(None)) => "两端",
                    (Ok(None), _) => "离开端",
                    _ => "到达端",
                };
                // 站点没有 SPRE 就没有 P 点：数据缺；有 SPRE 却解不出才是我们的账。
                let station_has_spre = |end: &RouteEnd| match end {
                    RouteEnd::MemberLeave { member, .. }
                    | RouteEnd::MemberArrive { member, .. } => {
                        member.get_element("SPRE").ok().flatten().is_some()
                    }
                    _ => true,
                };
                let unresolved_has_spre = match (&from, &to) {
                    (Ok(None), _) => station_has_spre(&slot.from),
                    _ => station_has_spre(&slot.to),
                };
                let class = if unresolved_has_spre { "bad" } else { "data" };
                problems.push(format!(
                    "[{class}] 槽位 {} → {} {which} P 点解不出{}",
                    row.from,
                    row.to,
                    if unresolved_has_spre {
                        ""
                    } else {
                        "（该站点无 SPRE）"
                    }
                ));
            }
        }
        slots.push(row);
    }
    if !route.tubes.is_empty() {
        if !head {
            problems.push("[bad] 容器头点（HPOS/HDIR）解不出".into());
        }
        if !tail {
            problems.push("[bad] 容器尾点（TPOS/TDIR）解不出".into());
        }
    }
    let failed: Vec<String> = report
        .failed
        .iter()
        .map(|i| format!("{} {}: {}", i.refno, i.noun, i.detail))
        .collect();
    for f in &failed {
        // 生产路径判 failed 的都是数据断链（悬空引用等）；本探针不替它改判。
        problems.push(format!("[data] failed: {f}"));
    }
    let _ = station_count;
    // 结论取最重的一档：bad（生成侧缺陷）> account（几何对、报告没记账）> data（数据侧）> ok。
    let verdict = if problems.iter().any(|p| p.starts_with("[bad]")) {
        "bad"
    } else if problems.iter().any(|p| p.starts_with("[account]")) {
        "account"
    } else if !problems.is_empty() {
        "data"
    } else {
        "ok"
    };
    Ok(BranRow {
        refno: refno.to_string(),
        name,
        owner_noun,
        head,
        tail,
        members,
        slots,
        generated_elements: elements.len(),
        failed,
        verdict: verdict.into(),
        problems,
    })
}

fn main() -> anyhow::Result<()> {
    let args = parse_args()?;
    let template_dir = if args.template.is_dir() {
        args.template.clone()
    } else {
        args.template
            .parent()
            .unwrap_or(Path::new("."))
            .to_path_buf()
    };
    let template_file = if args.template.is_dir() {
        template_file_for(&args.template, "DESI")?
    } else {
        args.template.clone()
    };
    let set = if args.project_dirs.is_empty() {
        std::sync::Arc::new(DbSet::with_attlib_file(&args.attlib)?)
    } else {
        let resolver = DirectoryDbResolver::scan(&args.project_dirs, &template_dir)?;
        std::sync::Arc::new(DbSet::with_attlib_file_and_resolver(
            &args.attlib,
            Box::new(resolver),
        )?)
    };
    let dbno = set.add_db(DbFilePin {
        file: args.db_file.clone(),
        template: template_file,
        db_type: Some("DESI".into()),
        sesno: args.sesno,
    })?;
    println!(
        "库 {} 打开为 dbno={dbno}，钉在会话 {:?}",
        args.db_file.display(),
        set.pinned_sesno(dbno)
    );
    if !args.catalogues.is_empty() {
        let catalogue_template = template_file_for(&template_dir, "CATA")?;
        for file in &args.catalogues {
            let cat = set.add_db(DbFilePin {
                file: file.clone(),
                template: catalogue_template.clone(),
                db_type: Some("CATA".into()),
                sesno: None,
            })?;
            println!("目录库 {} 打开为 dbno={cat}", file.display());
        }
    }

    let brans = match args.only {
        Some(refno) => vec![refno],
        None => scan_brans(&args.db_file, args.sesno, &set)?,
    };
    println!("BRAN {} 条", brans.len());
    std::fs::create_dir_all(&args.out)?;

    let mut cache = CatalogueMeshCache::default();
    let mut rows = Vec::new();
    let mut verdicts: BTreeMap<String, usize> = BTreeMap::new();
    let mut outcome_counts: BTreeMap<String, usize> = BTreeMap::new();
    let mut problem_kinds: BTreeMap<String, usize> = BTreeMap::new();
    let mut members_total = 0usize;
    let mut members_generated = 0usize;
    let mut members_unexplained = 0usize;
    for refno in brans {
        let row = audit_bran(&set, refno, &mut cache)?;
        *verdicts.entry(row.verdict.clone()).or_insert(0) += 1;
        for slot in &row.slots {
            *outcome_counts.entry(slot.outcome.clone()).or_insert(0) += 1;
        }
        for member in &row.members {
            if member.station || member.noun == "ATTA" {
                members_total += 1;
                if member.generated {
                    members_generated += 1;
                } else if member.reasons.is_empty() {
                    members_unexplained += 1;
                }
            }
        }
        for problem in &row.problems {
            let kind = problem
                .split(['：', ':', '（', '('])
                .next()
                .unwrap_or(problem)
                .split_whitespace()
                .filter(|w| !w.contains('/') && !w.ends_with("mm") && !w.ends_with('°'))
                .collect::<Vec<_>>()
                .join(" ");
            *problem_kinds.entry(kind).or_insert(0) += 1;
        }
        rows.push(row);
    }

    std::fs::write(
        args.out.join("bran_audit.json"),
        serde_json::to_vec_pretty(&rows)?,
    )?;
    let mut summary = String::new();
    summary.push_str(&format!("BRAN 总数 {}；结论 {verdicts:?}\n", rows.len()));
    summary.push_str(&format!(
        "路由成员 {members_total}（生成 {members_generated}，无依据缺口 {members_unexplained}）\n"
    ));
    summary.push_str(&format!("槽位结论 {outcome_counts:?}\n"));
    // 按属主学科各记一行：同一库里 PIPE 与 HVAC 的 BRAN 对「隐式管身」的期待相反。
    let mut owners: BTreeMap<&str, Vec<&BranRow>> = BTreeMap::new();
    for row in &rows {
        owners.entry(row.owner_noun.as_str()).or_default().push(row);
    }
    summary.push_str("按属主 noun：\n");
    for (owner, group) in &owners {
        let mut group_verdicts: BTreeMap<&str, usize> = BTreeMap::new();
        let mut group_outcomes: BTreeMap<&str, usize> = BTreeMap::new();
        let (mut total, mut generated, mut unexplained) = (0usize, 0usize, 0usize);
        let mut built_length_mm = 0.0f64;
        for row in group {
            *group_verdicts.entry(row.verdict.as_str()).or_insert(0) += 1;
            for slot in &row.slots {
                *group_outcomes.entry(slot.outcome.as_str()).or_insert(0) += 1;
                if slot.outcome == "built" {
                    built_length_mm += slot.length_mm.unwrap_or(0.0);
                }
            }
            for member in &row.members {
                if member.station || member.noun == "ATTA" {
                    total += 1;
                    if member.generated {
                        generated += 1;
                    } else if member.reasons.is_empty() {
                        unexplained += 1;
                    }
                }
            }
        }
        summary.push_str(&format!(
            "  {owner:<6} BRAN {:>4}  结论 {group_verdicts:?}  成员 {total}（生成 {generated}，无依据缺口 {unexplained}）  槽位 {group_outcomes:?}  建成管身合计 {:.1} m\n",
            group.len(),
            built_length_mm / 1000.0
        ));
    }
    summary.push_str("问题种类：\n");
    for (kind, count) in &problem_kinds {
        summary.push_str(&format!("  {count:>5}  {kind}\n"));
    }
    summary.push_str("\n生成侧缺陷 / 记账缺口的 BRAN（bad / account）：\n");
    for row in rows
        .iter()
        .filter(|r| r.verdict == "bad" || r.verdict == "account")
    {
        summary.push_str(&format!(
            "  {} {}  成员 {} / 槽位 {} / 生成 {}\n",
            row.refno,
            row.name.clone().unwrap_or_default(),
            row.members.len(),
            row.slots.len(),
            row.generated_elements
        ));
        for problem in &row.problems {
            summary.push_str(&format!("      - {problem}\n"));
        }
    }
    std::fs::write(args.out.join("bran_audit_summary.txt"), &summary)?;
    print!("{summary}");
    println!("产物: {}", args.out.display());
    Ok(())
}

//! 型材截面（G1-1a 第 1 步）：把 SCTN / GENSEC 的目录截面求成一个二维环组。
//!
//! 对标 Core3D `getProfileFromDB@0x10410b30 → sub_10410BB0`（`profileExtractFromDB`）：
//!
//! 1. 截面元素从哪来：`SCTN.SPRE → SPCO --CATR--> SPRF --GSTR--> GMSS`，GMSS 的成员是
//!    二维几何集元素 `SPRO`（任意截面，顶点 `SPVE`）/ `SREC`（矩形）/ `SANN`（圆环）。
//!    实测（`examples/section_probe --census`）：ams7997 1762 只 SCTN 里 1761 只走这条链
//!    （SPRO 173 / SPRO+SREC 1554 / SANN 34），ams8000 528 只里 500 只**没有 SPRE**——
//!    MSUP 杆件，E3D 的 TTY 导出只给一根中心线（`1RX03-MSUP-E0110.rvm` 全是 `Line`），
//!    没有实体；这里同样交 [`SectionChain::NoSpre`]，由调用方记账。
//! 2. 选哪个成员：同一 GMSS 常放两份表现（SPRO `LEVE [0,10]` + SREC `LEVE [14,14]`），
//!    按 `CSG_TreeBuilderOptions::isWanted` 那套筛（[`crate::catalogue::display_filtered`]，
//!    级别取 `LEVEL STRU`——[`crate::catalogue::repre_level_of`]），取**文件序第一个**过筛的。
//! 3. 截面自己的标架：`pplib/QSPROF@0x103B8410`——成员的 `PX/PY` 是平移，`PLAX` 两个整数经
//!    `pplib/GTPLAX@0x103C5FB8` 变成一个角 θ，再转 `θ − 90°`（[`plax_angle`]、[`plax_rotation`]）。
//!    `PLAX = [1, 2]` 是恒等，`[2, 2]` 是转 180°。
//! 4. 顶点：`SPVE` 的 `PX / PY / PRAD`（目录表达式，`ATTRIB PARA[n]` 指 SPRF 的 `PARA`、
//!    `ATTRIB DESP[n]` 指设计件的 `DESP`，全部经 e3d-io `DbElementParamEnv` 求值）；`PRAD` 是
//!    顶点倒圆半径，与设计侧 PAVE 的 `FRAD` 同义，交给 [`crate::discretise::flatten_ring`] 折线化。
//!    SREC 的 `PXLE / PYLE` 是边长；SANN 的 `PRAD` 是外半径、`PWID` 是壁厚，`PANG` 是扫角。
//! 5. 对齐：`pllib/GJLORI@0x10399B54 → pplib/GPLDET@0x103C2724`——在 SPRF 的 `PSTR`（PTSS
//!    p-line 集）里找 `PKEY == SCTN.JUSL`（缺省 `NA`）的 `PLIN`，它的 `PX / PY` 就是截面要
//!    平移的量：`getProfileFromDB` 末尾把变换沿截面平面减去 `(v77, v90)`，让 JUSL 线落在
//!    `POSS → POSE` 轴上。这里只算出偏移（[`SectionProfile::justification`]），三维放置归第 2 步。
//!
//! 不在本步里的：SCTN 的两端切平面 / BANG / 关节 / MEML（`DB_Sctn::DB_makeCSG@0x10738C30`），
//! GENSEC 的 SPINE 扫掠（`sub_107318E0`）。

use std::f64::consts::{FRAC_PI_2, PI};

use e3d_io::db_element::{DbElement, DbElementError};
use e3d_io::db_param_env::DbElementParamEnv;
use e3d_io::record::descriptor::DescriptorValue;

use crate::catalogue::{display_filtered, optional_number, repre_level_of, required_number};
use crate::discretise::FilletVertex;
use crate::elmodl::ModelError;
use crate::solid::cylinder_segments;

/// 缺省对齐线：`JUSL` 未设时 E3D 用中性轴。
pub const DEFAULT_JUSTIFICATION_LINE: &str = "NA";

#[derive(Debug, thiserror::Error)]
pub enum SectionError {
    #[error(transparent)]
    Db(#[from] DbElementError),
    #[error(transparent)]
    Model(#[from] ModelError),
    #[error("{refno}（{noun}）的 PLAX {plax:?} 不是 GTPLAX 认得的轴码")]
    BadPlax {
        refno: String,
        noun: String,
        plax: Vec<i32>,
    },
    #[error("{refno}（{noun}）截面退化：{detail}")]
    Degenerate {
        refno: String,
        noun: String,
        detail: String,
    },
}

/// `SPRE` 链走到的地方。
#[derive(Debug, Clone)]
pub enum SectionChain {
    /// 设计件没有 `SPRE`（MSUP 杆件的常态）：E3D 只画中心线，不出实体。
    NoSpre,
    /// 链在第 `hop` 跳断了：引用悬空、库没开、或没有 `CATR`/`GSTR`。
    Broken { hop: usize, at: String, why: String },
    /// 走到了 GMSS。
    Resolved(SectionCatalogue),
}

/// 一条走通的截面目录链。
#[derive(Debug, Clone)]
pub struct SectionCatalogue {
    pub spco: DbElement,
    /// 型材目录件：`PARA`（截面参数）、`GTYP`、`PSTR`（p-line 集）、`GSTR`（二维几何集）。
    pub sprf: DbElement,
    pub gmss: DbElement,
}

/// 截面来自 GMSS 的哪种成员。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SectionSource {
    /// 任意截面：`SPVE` 顶点环。
    Spro { vertices: usize },
    /// 矩形。
    Srec,
    /// 圆环 / 圆环扇形（`PANG` 不满一圈时是一个 C 形单环）。
    Sann,
}

/// GMSS 里一个过了显示筛、且真有形状的成员。
#[derive(Debug, Clone)]
pub struct SectionMember {
    pub source: SectionSource,
    /// 该成员在 GMSS 里的 refno，记账用。
    pub member_refno: String,
    /// `rings[0]` 外边界，其余是孔（整圈 SANN 的内圆）。顶点带倒圆半径，尚未折线化。
    pub rings: Vec<Vec<FilletVertex>>,
}

/// 求好的二维截面：环在截面自己的平面里（x 右、y 上），单位 mm。
///
/// 一个 GMSS 可以放多个同时过筛的成员——ams7997 的 `DTUB` 管型材是两个 `PANG=180` 的 SANN
/// 各转 180° 拼成整圈；H 型钢的 GMSS 里还带一个没有顶点的 `CLFL` 占位 SPRO。E3D 逐成员建体
/// 后并集（`DB_Sctn` 的截面子树），这里同样把**全部**有形状的成员交出去，由建体方并集。
#[derive(Debug, Clone)]
pub struct SectionProfile {
    pub members: Vec<SectionMember>,
    /// `JUSL` p-line 在同一平面里的位置。放置时把整个截面平移 `-justification`。
    pub justification: [f64; 2],
    /// 实际用到的对齐线名（`JUSL`，缺省 `NA`）。
    pub justification_line: String,
    /// 对齐线是否真在 `PSTR` 里找到；没找到按 (0, 0) 对齐并记 note。
    pub justification_found: bool,
    /// 用的表现级（`LEVEL STRU`）。
    pub level: u32,
    pub notes: Vec<String>,
}

impl SectionProfile {
    /// 没有任何成员出得了形状（全被筛掉、或只剩空占位 SPRO）：E3D 同样只画中心线。
    pub fn is_empty(&self) -> bool {
        self.members.is_empty()
    }
}

/// `SPRE → SPCO → CATR → SPRF → GSTR → GMSS`。
pub fn section_catalogue(design: &DbElement) -> Result<SectionChain, SectionError> {
    let Some(spco) = design.get_element("SPRE")? else {
        return Ok(SectionChain::NoSpre);
    };
    let hop = |hop: usize, el: &DbElement, why: String| SectionChain::Broken {
        hop,
        at: el.refno().to_string(),
        why,
    };
    let spco_noun = match spco.element_type() {
        Ok(noun) => noun,
        Err(error) => return Ok(hop(0, &spco, format!("SPRE 悬空：{error}"))),
    };
    if !spco_noun.eq_ignore_ascii_case("SPCO") {
        return Ok(hop(0, &spco, format!("SPRE 指向 {spco_noun}，不是 SPCO")));
    }
    let Some(sprf) = spco.get_element("CATR")? else {
        return Ok(hop(0, &spco, "SPCO 无 CATR".to_string()));
    };
    let sprf_noun = match sprf.element_type() {
        Ok(noun) => noun,
        Err(error) => return Ok(hop(1, &sprf, format!("CATR 悬空：{error}"))),
    };
    if !sprf_noun.eq_ignore_ascii_case("SPRF") {
        return Ok(hop(1, &sprf, format!("CATR 指向 {sprf_noun}，不是 SPRF")));
    }
    let Some(gmss) = sprf.get_element("GSTR")? else {
        return Ok(hop(1, &sprf, "SPRF 无 GSTR".to_string()));
    };
    match gmss.element_type() {
        Ok(noun) if noun.eq_ignore_ascii_case("GMSS") => {}
        Ok(noun) => return Ok(hop(2, &gmss, format!("GSTR 指向 {noun}，不是 GMSS"))),
        Err(error) => return Ok(hop(2, &gmss, format!("GSTR 悬空：{error}"))),
    }
    Ok(SectionChain::Resolved(SectionCatalogue {
        spco,
        sprf,
        gmss,
    }))
}

/// 对一条走通的链求截面：GMSS 里每个过显示筛的 SPRO / SREC / SANN 各出一组环。
///
/// 单个成员求不出（顶点表达式解不开、尺寸退化）只记 note、不拖垮别的成员——与 CRCATI
/// 逐图元的错误边界同口径；没有顶点的 SPRO（H 型钢 GMSS 里的 `CLFL` 占位）静默跳过。
pub fn evaluate_section_profile(
    design: &DbElement,
    catalogue: &SectionCatalogue,
) -> Result<SectionProfile, SectionError> {
    let level = repre_level_of(design)?;
    let env = DbElementParamEnv::new(Some(design.clone()), Some(catalogue.sprf.clone()))
        .without_insulation();
    let mut notes = Vec::new();
    let mut members = Vec::new();
    let mut filtered = Vec::new();
    for member in catalogue.gmss.members()? {
        let noun = member.element_type()?;
        if !matches!(noun.as_str(), "SPRO" | "SREC" | "SANN") {
            continue;
        }
        if let Some(why) = display_filtered(&member, level)? {
            filtered.push(format!("{} {noun} {why}", member.refno()));
            continue;
        }
        let frame = member_frame(&member, &env)?;
        let built = match noun.as_str() {
            "SPRO" => spro_rings(&member, &env, &frame),
            "SREC" => srec_rings(&member, &env, &frame),
            "SANN" => sann_rings(&member, &env, &frame, &mut notes),
            _ => unreachable!("上面只放行这三种 noun"),
        };
        match built {
            Ok(Some((source, rings))) => members.push(SectionMember {
                source,
                member_refno: member.refno().to_string(),
                rings,
            }),
            Ok(None) => notes.push(format!(
                "{} {noun} 没有顶点（CLFL 占位），不出形状",
                member.refno()
            )),
            Err(SectionError::Db(error)) => return Err(SectionError::Db(error)),
            Err(error) => notes.push(format!("{} {noun} 求不出：{error}", member.refno())),
        }
    }
    if !filtered.is_empty() {
        notes.push(format!(
            "display-filtered at LEVEL STRU {level}: {}",
            filtered.join(", ")
        ));
    }

    let justification_line = design
        .get_string("JUSL")
        .ok()
        .flatten()
        .map(|text| text.trim().to_ascii_uppercase())
        .filter(|text| !text.is_empty())
        .unwrap_or_else(|| DEFAULT_JUSTIFICATION_LINE.to_string());
    let (justification, justification_found) =
        justification_offset(&catalogue.sprf, &justification_line, &env, &mut notes)?;

    Ok(SectionProfile {
        members,
        justification,
        justification_line,
        justification_found,
        level,
        notes,
    })
}

/// 成员在 GMSS 平面里的标架：平移 `(PX, PY)` + `PLAX` 给的旋转（`pplib/QSPROF`）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MemberFrame {
    pub offset: [f64; 2],
    /// 旋转角（弧度）= `plax_angle − 90°`。
    pub rotation: f64,
}

impl MemberFrame {
    /// 把成员局部 (x, y) 放进 GMSS 平面。
    pub fn apply(&self, x: f64, y: f64) -> [f64; 2] {
        let (sin, cos) = self.rotation.sin_cos();
        [
            self.offset[0] + x * cos - y * sin,
            self.offset[1] + x * sin + y * cos,
        ]
    }
}

fn member_frame(member: &DbElement, env: &DbElementParamEnv) -> Result<MemberFrame, SectionError> {
    let offset = [
        optional_number(member, "PX", env).unwrap_or(0.0),
        optional_number(member, "PY", env).unwrap_or(0.0),
    ];
    let plax = plax_words(member)?;
    let angle = plax_angle(&plax).ok_or_else(|| SectionError::BadPlax {
        refno: member.refno().to_string(),
        noun: member.element_type().unwrap_or_else(|_| "?".into()),
        plax: plax.clone(),
    })?;
    Ok(MemberFrame {
        offset,
        rotation: plax_rotation(angle),
    })
}

fn plax_words(member: &DbElement) -> Result<Vec<i32>, SectionError> {
    Ok(match member.attribute("PLAX")? {
        Some(DescriptorValue::RawWords(words) | DescriptorValue::WordArray(words)) => {
            words.into_iter().map(|word| word as i32).collect()
        }
        Some(DescriptorValue::IntArray(words)) => words,
        // 未设按恒等：`[1, 2]`。
        _ => vec![1, 2],
    })
}

/// `pplib/GTPLAX@0x103C5FB8`：两个整数的轴码 → 角（弧度，折进 [−π, π]）。
///
/// - 简单码（`|a| ∈ {1, 2}`）：`[1, 1] → 0`、`[1, 2] → 90°`、`[2, 1] → 180°`、`[2, 2] → −90°`；
///   其余 → 无效（Core 报 212）。
/// - 角码（`|a| ≥ 10`）：`b` 是百分之一度，`|a| / 10` 选象限、`|a| % 10` 选正负：
///   十位 2 → `90° ∓ b`，4 → `180° ± b`，5 → `−90° ± b`，其它 → `±b`（个位 2 取正、否则取负）。
///   `GTPLAX` 的第三种（带量纲的表达式）语料里没有，未接。
pub fn plax_angle(plax: &[i32]) -> Option<f64> {
    let (&a, &b) = (plax.first()?, plax.get(1)?);
    let magnitude = a.abs();
    let angle = if magnitude == 1 || magnitude == 2 {
        match (magnitude, b) {
            (1, 1) => 0.0,
            (1, 2) => FRAC_PI_2,
            (2, 1) => PI,
            (2, 2) => -FRAC_PI_2,
            _ => return None,
        }
    } else if magnitude >= 10 {
        let theta = f64::from(b) / 5_729.577_951_308_232; // 百分之一度 → 弧度
        let ones = magnitude % 10;
        match magnitude / 10 {
            2 => {
                if ones == 1 {
                    FRAC_PI_2 - theta
                } else {
                    FRAC_PI_2 + theta
                }
            }
            3 => return None,
            4 => {
                if ones == 2 {
                    PI - theta
                } else {
                    PI + theta
                }
            }
            5 => {
                if ones == 1 {
                    theta - FRAC_PI_2
                } else {
                    -theta - FRAC_PI_2
                }
            }
            _ => {
                if ones == 2 {
                    theta
                } else {
                    -theta
                }
            }
        }
    } else {
        return None;
    };
    let mut angle = angle;
    while angle.abs() > PI {
        angle -= if angle < 0.0 { -2.0 * PI } else { 2.0 * PI };
    }
    Some(angle)
}

/// `pplib/QSPROF`：成员标架的旋转 = `plax_angle − 90°`（`[1, 2]` 恒等）。
pub fn plax_rotation(plax_angle: f64) -> f64 {
    plax_angle - FRAC_PI_2
}

type MemberRings = Option<(SectionSource, Vec<Vec<FilletVertex>>)>;

fn vertex(frame: &MemberFrame, x: f64, y: f64, frad: f64) -> FilletVertex {
    let [px, py] = frame.apply(x, y);
    FilletVertex { x: px, y: py, frad }
}

/// SPRO：`SPVE` 顶点环。没有顶点交 `None`（占位成员），1~2 个顶点是数据坏。
fn spro_rings(
    spro: &DbElement,
    env: &DbElementParamEnv,
    frame: &MemberFrame,
) -> Result<MemberRings, SectionError> {
    let mut ring = Vec::new();
    for spve in spro.members()? {
        if !spve.element_type()?.eq_ignore_ascii_case("SPVE") {
            continue;
        }
        let x = required_number(&spve, "PX", env, "section vertex")?;
        let y = required_number(&spve, "PY", env, "section vertex")?;
        let frad = optional_number(&spve, "PRAD", env).unwrap_or(0.0).max(0.0);
        ring.push(vertex(frame, x, y, frad));
    }
    if ring.is_empty() {
        return Ok(None);
    }
    if ring.len() < 3 {
        return Err(SectionError::Degenerate {
            refno: spro.refno().to_string(),
            noun: "SPRO".to_string(),
            detail: format!("只有 {} 个 SPVE 顶点", ring.len()),
        });
    }
    Ok(Some((
        SectionSource::Spro {
            vertices: ring.len(),
        },
        vec![ring],
    )))
}

/// SREC：`PXLE × PYLE` 的矩形，中心在成员标架原点。
fn srec_rings(
    srec: &DbElement,
    env: &DbElementParamEnv,
    frame: &MemberFrame,
) -> Result<MemberRings, SectionError> {
    let xlen = required_number(srec, "PXLE", env, "section rectangle")?;
    let ylen = required_number(srec, "PYLE", env, "section rectangle")?;
    if !(xlen > 0.0 && ylen > 0.0) {
        return Err(SectionError::Degenerate {
            refno: srec.refno().to_string(),
            noun: "SREC".to_string(),
            detail: format!("PXLE/PYLE = {xlen}/{ylen}"),
        });
    }
    let (hx, hy) = (xlen * 0.5, ylen * 0.5);
    let ring = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
        .into_iter()
        .map(|(x, y)| vertex(frame, x, y, 0.0))
        .collect();
    Ok(Some((SectionSource::Srec, vec![ring])))
}

/// SANN：外半径 `PRAD`、壁厚 `PWID`、扫角 `PANG`。
///
/// 整圈（`PANG` 未设 / 0 / ±360）= 外圆环 + 内圆孔；不满一圈 = 一个 C 形单环：外弧从成员
/// 局部 +X 起逆时针扫 `PANG`（负角顺时针），再沿内弧回来。ams7997 的 `DTUB` 管型材就是两个
/// `PANG=180` 的 SANN（`PLAX [1,2]` 与 `[2,2]`，后者转 180°）拼成整圈。圆周段数按 libgm
/// 曲面口径（[`cylinder_segments`]）随扫角等比例缩，与 `part_rev_segments` 同规则。
fn sann_rings(
    sann: &DbElement,
    env: &DbElementParamEnv,
    frame: &MemberFrame,
    notes: &mut Vec<String>,
) -> Result<MemberRings, SectionError> {
    let radius = required_number(sann, "PRAD", env, "section annulus")?;
    let width = optional_number(sann, "PWID", env).unwrap_or(0.0);
    if !(radius > 0.0) {
        return Err(SectionError::Degenerate {
            refno: sann.refno().to_string(),
            noun: "SANN".to_string(),
            detail: format!("PRAD = {radius}"),
        });
    }
    let inner = if width > 0.0 && width < radius {
        Some(radius - width)
    } else {
        if width != 0.0 {
            notes.push(format!(
                "SANN {} PWID={width} 不在 (0, PRAD={radius}) 内，按实心圆画",
                sann.refno()
            ));
        }
        None
    };
    let sweep = optional_number(sann, "PANG", env).unwrap_or(0.0);
    Ok(Some((
        SectionSource::Sann,
        annulus_rings(frame, radius, inner, sweep),
    )))
}

/// SANN 的环几何（纯函数，便于单测）：见 [`sann_rings`]。
pub fn annulus_rings(
    frame: &MemberFrame,
    radius: f64,
    inner: Option<f64>,
    sweep_deg: f64,
) -> Vec<Vec<FilletVertex>> {
    let full_circle =
        sweep_deg == 0.0 || (sweep_deg.abs() - 360.0).abs() <= 1e-6 || sweep_deg.abs() > 360.0;
    let full_segments = cylinder_segments(radius).max(3);
    if full_circle {
        let circle = |r: f64| -> Vec<FilletVertex> {
            (0..full_segments)
                .map(|i| {
                    let theta = std::f64::consts::TAU * f64::from(i) / f64::from(full_segments);
                    let (sin, cos) = theta.sin_cos();
                    vertex(frame, r * cos, r * sin, 0.0)
                })
                .collect()
        };
        let mut rings = vec![circle(radius)];
        if let Some(inner) = inner {
            rings.push(circle(inner));
        }
        return rings;
    }
    let segments = ((f64::from(full_segments) * sweep_deg.abs() / 360.0).ceil() as i32).max(2);
    let arc = |r: f64, from_deg: f64, to_deg: f64| {
        (0..=segments)
            .map(|i| {
                let t = f64::from(i) / f64::from(segments);
                let theta = (from_deg + (to_deg - from_deg) * t).to_radians();
                let (sin, cos) = theta.sin_cos();
                vertex(frame, r * cos, r * sin, 0.0)
            })
            .collect::<Vec<_>>()
    };
    let mut ring = arc(radius, 0.0, sweep_deg);
    match inner {
        Some(inner) => ring.extend(arc(inner, sweep_deg, 0.0)),
        None => ring.push(vertex(frame, 0.0, 0.0, 0.0)),
    }
    vec![ring]
}

/// `SPRF.PSTR`（PTSS）里 `PKEY == jusl` 的 `PLIN` 的 `(PX, PY)`；找不到按原点并记 note。
fn justification_offset(
    sprf: &DbElement,
    jusl: &str,
    env: &DbElementParamEnv,
    notes: &mut Vec<String>,
) -> Result<([f64; 2], bool), SectionError> {
    let Some(ptss) = sprf.get_element("PSTR")? else {
        notes.push(format!(
            "SPRF {} 无 PSTR，JUSL {jusl} 按原点对齐",
            sprf.refno()
        ));
        return Ok(([0.0, 0.0], false));
    };
    for pline in ptss.members()? {
        if !pline.element_type()?.eq_ignore_ascii_case("PLIN") {
            continue;
        }
        let key = pline
            .get_string("PKEY")
            .ok()
            .flatten()
            .unwrap_or_default()
            .trim()
            .to_ascii_uppercase();
        if key != jusl {
            continue;
        }
        let frame = member_frame(&pline, env)?;
        // p-line 自己的 PX/PY 就是位置；PLAX 只给方向，不动位置。
        return Ok((frame.offset, true));
    }
    notes.push(format!(
        "PSTR {} 里没有 PKEY={jusl} 的 PLIN，按原点对齐",
        ptss.refno()
    ));
    Ok(([0.0, 0.0], false))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-12
    }

    #[test]
    fn simple_axis_codes_follow_gtplax() {
        assert!(close(plax_angle(&[1, 1]).unwrap(), 0.0));
        assert!(close(plax_angle(&[1, 2]).unwrap(), FRAC_PI_2));
        assert!(close(plax_angle(&[2, 1]).unwrap(), PI));
        assert!(close(plax_angle(&[2, 2]).unwrap(), -FRAC_PI_2));
        assert!(plax_angle(&[1, 3]).is_none());
        assert!(plax_angle(&[3, 1]).is_none());
        assert!(plax_angle(&[1]).is_none());
    }

    #[test]
    fn angle_codes_pick_quadrant_by_tens_and_sign_by_ones() {
        // 十位 2、个位 1：90° − 30°。
        assert!(close(
            plax_angle(&[21, 3000]).unwrap(),
            FRAC_PI_2 - 30f64.to_radians()
        ));
        // 十位 4、个位 2：180° − 45°。
        assert!(close(
            plax_angle(&[42, 4500]).unwrap(),
            PI - 45f64.to_radians()
        ));
        // 十位 5、个位 1：30° − 90°。
        assert!(close(
            plax_angle(&[51, 3000]).unwrap(),
            30f64.to_radians() - FRAC_PI_2
        ));
        // 十位 1、个位 2：+b；个位 1：−b。
        assert!(close(plax_angle(&[12, 1000]).unwrap(), 10f64.to_radians()));
        assert!(close(plax_angle(&[11, 1000]).unwrap(), -10f64.to_radians()));
        // 十位 3 在 GTPLAX 里是空臂。
        assert!(plax_angle(&[31, 1000]).is_none());
    }

    #[test]
    fn a_full_annulus_is_two_rings_and_a_half_annulus_is_one_c_shape() {
        let frame = MemberFrame {
            offset: [0.0, 0.0],
            rotation: 0.0,
        };
        let full = annulus_rings(&frame, 201.0, Some(197.0), 0.0);
        assert_eq!(full.len(), 2);
        assert!(full[0].iter().all(|v| close(v.x.hypot(v.y), 201.0)));
        assert!(full[1].iter().all(|v| close(v.x.hypot(v.y), 197.0)));

        // ams7997 DTUB：PANG=180 的两个 SANN 各占半圈。
        let half = annulus_rings(&frame, 201.0, Some(197.0), 180.0);
        assert_eq!(half.len(), 1);
        let ring = &half[0];
        assert!(
            ring.iter().all(|v| v.y >= -1e-9),
            "半环全在上半平面：{ring:?}"
        );
        let radii: Vec<f64> = ring.iter().map(|v| v.x.hypot(v.y)).collect();
        assert!(radii.iter().all(|r| close(*r, 201.0) || close(*r, 197.0)));
        assert!(close(ring[0].x, 201.0) && close(ring[0].y, 0.0));
        assert!(close(ring[ring.len() - 1].x, 197.0) && close(ring[ring.len() - 1].y, 0.0));

        // 转 180° 的那一份（PLAX [2, 2]）落在下半平面，两份拼成整圈。
        let flipped = MemberFrame {
            offset: [0.0, 0.0],
            rotation: plax_rotation(plax_angle(&[2, 2]).unwrap()),
        };
        let other = annulus_rings(&flipped, 201.0, Some(197.0), 180.0);
        assert!(other[0].iter().all(|v| v.y <= 1e-9), "{:?}", other[0]);

        // 没有壁厚的扇形收口到圆心。
        let wedge = annulus_rings(&frame, 10.0, None, 90.0);
        let last = wedge[0].last().unwrap();
        assert!(close(last.x, 0.0) && close(last.y, 0.0));
    }

    #[test]
    fn plax_1_2_is_the_identity_frame_and_2_2_flips() {
        let identity = MemberFrame {
            offset: [0.0, 0.0],
            rotation: plax_rotation(plax_angle(&[1, 2]).unwrap()),
        };
        let p = identity.apply(3.0, 4.0);
        assert!(close(p[0], 3.0) && close(p[1], 4.0));

        let flipped = MemberFrame {
            offset: [1.0, 1.0],
            rotation: plax_rotation(plax_angle(&[2, 2]).unwrap()),
        };
        let q = flipped.apply(3.0, 4.0);
        assert!(close(q[0], -2.0) && close(q[1], -3.0), "{q:?}");
    }
}

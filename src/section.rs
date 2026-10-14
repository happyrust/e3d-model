//! 型材截面（G1-1a 第 1 步）：把 SCTN / GENSEC 的目录截面求成一个二维环组。
//!
//! 对标 Core3D `getProfileFromDB@0x10410b30 → sub_10410BB0`（`profileExtractFromDB`）：
//!
//! 1. 截面元素从哪来：`SCTN.SPRE → SPCO --CATR--> SPRF --GSTR--> GMSS`（`SPRE` 也可以
//!    直接指到 `SPRF`，见 [`section_catalogue`]），GMSS 的成员是
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
//!    **这条线只出平移**；截面绕轴转多少是另一条线的事，见 [`SectionProfile::neutral_axis_spin`]。
//!
//! 第 2 步（放置）与第 3 步（两端切面）接在文件后半段：[`section_placement`] /
//! [`section_cut_planes`] / [`section_local_solid`]。
//!
//! GENSEC 的 SPINE 路径（直段 + THRU/FILL/CENT 弧）与沿路径的扫掠（`DB_Gensec@0x10730090` /
//! `sub_107318E0`）在文件末段：[`spine_path`] / [`spine_swept_solid`]。
//!
//! 不在本文件里的：SCTN 的负体——JOIS/JOIE 关节与成员链上的 FITT / CMFI(+SBFI) /
//! SNOD→SJOI / SCOJ→SUBJ（`DB_Sctn::DB_makeCSG@0x10738C30` 的第 4 阶段，排在 G1-1c
//! 目录路之后）、MEML 那条 member line（`sub_10739770` → `gm_CreateStraight`，是线不是体）。

use std::f64::consts::{FRAC_PI_2, PI};

use glam::{DMat3, DMat4, DVec3};
use manifold_csg::Manifold;

use e3d_io::db_element::{DbElement, DbElementError};
use e3d_io::db_param_env::DbElementParamEnv;
use e3d_io::record::descriptor::DescriptorValue;

use crate::catalogue::{display_filtered, optional_number, repre_level_of, required_number};
use crate::discretise::{FilletVertex, flatten_ring};
use crate::elmodl::ModelError;
use crate::solid::{MANIFOLD_NUDGE_MM, cut_by_plane, cylinder_segments, extrude_rings, placed};

/// 缺省对齐线：`JUSL` 未设时 E3D 用中性轴。
pub const DEFAULT_JUSTIFICATION_LINE: &str = "NA";

#[derive(Debug, thiserror::Error)]
pub enum SectionError {
    #[error("{refno}（{noun}）有限端点间长 {length} mm 不超过 GTDIRS 的 0.001 mm 门，没有截面实体")]
    EmptyLength {
        refno: String,
        noun: String,
        length: f64,
    },
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
    /// 途经的规格件；`SPRE` 直接指到目录件时没有它（见 [`section_catalogue`]）。
    pub spco: Option<DbElement>,
    /// 型材目录件：`PARA`（截面参数）、`GTYP`、`PSTR`（p-line 集）、`GSTR`（二维几何集）。
    pub sprf: DbElement,
    pub gmss: DbElement,
}

/// `GATGOC` 沿 `CATR` / `PRTREF` 跳的上限。语料里最长的链是 1 跳（`SPCO → SPRF`），
/// 这个数只防目录里的环。
const MAX_CATALOGUE_HOPS: usize = 8;

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
    /// `pllib/ROTNAX@0x103A3398` 给截面标架的那一转（弧度）：`90° − 中性轴线的 PLAX 角`。
    ///
    /// **取的是中性轴线，不是 `JUSL` 那条线。** ROTNAX 不收外部传进来的 p-line，它自己
    /// 调 `pllib/GTNAID@0x103AC154`（"get NA id"）去找，而 GTNAID 只认三个键
    /// （`NA` = 531482、726125、726137，后两个是别名，本机语料里没出现过）；其余一律
    /// `NULIFY`，ROTNAX 随即走 `MCOPY` 那一支**原样拷矩阵、不转**。`JUSL` 只管平移
    /// （`GCORG3` 的 `POSS − R·(PX, PY, 0)`），不进这一转。
    ///
    /// 找不到中性轴线就是 0。曾经这里取的是 `JUSL` 那条线的 PLAX：`NA` 的 `[1, 2]` 恰好
    /// 也算出 0，所以 1756 根 NA 型材一直是对的，而 6 根 `JUSL=RTOS`（`[2, 2]` → 180°）
    /// 绕自身反了半圈——截面尺寸、长度、包围盒全对，只有非对称截面的朝向反了，
    /// AABB 层面的对拍看不出来。见 `docs` 里 S4001 / HW150 / C80 三样本的逐顶点记录。
    pub neutral_axis_spin: f64,
    /// 用的表现级（`LEVEL STRU`）。
    pub level: u32,
    /// 设计侧的 `LMIRR` 翻转旗（`DB_Gensec` 构造函数读的那一条）。
    ///
    /// 真时整张截面**连同它的 p-line 一起**沿自身 x 镜像，所以放在这里而不是放置那一步：
    /// 镜像与对齐同属「截面长什么样、原点落在哪」，最终点是 `−(x − jx)`。
    /// 先前整条链一次都没读过它——1374 件 CT01 那批 GENSEC 全是 `LMIRR=true`，
    /// 长度与截面尺寸都对，只有 x 方向整体反了 24.6 mm（不对称角钢的长短边调了个头）。
    pub mirrored: bool,
    pub notes: Vec<String>,
}

impl SectionProfile {
    /// 没有任何成员出得了形状（全被筛掉、或只剩空占位 SPRO）：E3D 同样只画中心线。
    pub fn is_empty(&self) -> bool {
        self.members.is_empty()
    }
}

/// `SPRE → [SPCO → CATR]* → SPRF → GSTR → GMSS`。
///
/// 第一段照 `catdblib/GATGOC@0x1035CFB0`（Core3D，"get and go to component"）走：
/// 从设计件 `SPRE` 出发，落在 `SPCO` / `GPART` / `PRTELE` 上就再跟 `CATR`，落在
/// `TABITEM` 上就跟 `PRTREF`，落在**其它任何 noun 上就是目录件本身**。所以 `SPRE`
/// 直接指向 `SPRF` 是 Core 认的合法形态，不是断链——ams7324 有 75 只 SCTN + 72 只
/// GENSEC 这么写（样本 `23708/25780`），此前一律按「SPRE 指向 SPRF，不是 SPCO」记成断链。
pub fn section_catalogue(design: &DbElement) -> Result<SectionChain, SectionError> {
    let Some(mut cursor) = design.get_element("SPRE")? else {
        return Ok(SectionChain::NoSpre);
    };
    let hop = |hop: usize, el: &DbElement, why: String| SectionChain::Broken {
        hop,
        at: el.refno().to_string(),
        why,
    };
    let mut spco = None;
    let mut hops = 0usize;
    let sprf = loop {
        let via_in = if hops == 0 { "SPRE" } else { "CATR" };
        let noun = match cursor.element_type() {
            Ok(noun) => noun.to_ascii_uppercase(),
            Err(error) => return Ok(hop(hops, &cursor, format!("{via_in} 悬空：{error}"))),
        };
        let via = match noun.as_str() {
            "SPCO" | "GPART" | "PRTELE" => "CATR",
            "TABITEM" => "PRTREF",
            _ => break cursor,
        };
        if hops >= MAX_CATALOGUE_HOPS {
            return Ok(hop(
                hops,
                &cursor,
                format!("{noun} 链超过 {MAX_CATALOGUE_HOPS} 跳"),
            ));
        }
        let Some(next) = cursor.get_element(via)? else {
            return Ok(hop(hops, &cursor, format!("{noun} 无 {via}")));
        };
        if noun == "SPCO" {
            spco = Some(cursor);
        }
        cursor = next;
        hops += 1;
    };
    let sprf_noun = sprf.element_type()?;
    if !sprf_noun.eq_ignore_ascii_case("SPRF") {
        let via = if hops == 0 { "SPRE" } else { "CATR" };
        return Ok(hop(
            hops,
            &sprf,
            format!("{via} 指向 {sprf_noun}，不是 SPRF"),
        ));
    }
    let Some(gmss) = sprf.get_element("GSTR")? else {
        return Ok(hop(hops, &sprf, "SPRF 无 GSTR".to_string()));
    };
    match gmss.element_type() {
        Ok(noun) if noun.eq_ignore_ascii_case("GMSS") => {}
        Ok(noun) => {
            return Ok(hop(hops + 1, &gmss, format!("GSTR 指向 {noun}，不是 GMSS")));
        }
        Err(error) => return Ok(hop(hops + 1, &gmss, format!("GSTR 悬空：{error}"))),
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
    let jusl = justification_offset(&catalogue.sprf, &justification_line, &env, &mut notes)?;

    Ok(SectionProfile {
        members,
        justification: jusl.offset,
        justification_line,
        justification_found: jusl.found,
        neutral_axis_spin: neutral_axis_spin(&catalogue.sprf, &env)?,
        level,
        // `LMIRR` 只在 GENSEC 的模板里；SCTN 的模板根本没有这条属性，`get_bool` 会直接
        // 报错而不是给 `None`，所以这里只认「读得到且为真」，读不到一律不镜像。
        mirrored: matches!(design.get_bool("LMIRR"), Ok(Some(true))),
        notes,
    })
}

/// 角钢 `SCTN` 的 FacetGroup 记录原点要落在目录轮廓原点，而不是 `JUSL` 对齐轴。
///
/// 建体时截面已经减过 [`SectionProfile::justification`]，记录原点在 `POSS`。
/// E3D 把记录原点留在目录轮廓原点，局部坐标是没减过的目录点；世界顶点相同。
/// 返回值就是要加回局部点、并从记录原点里减掉的 `(PX, PY)`。
///
/// 只对 `GTYP=ANG` 且偏移非零。`JUSL` 已经在目录原点（CPAR 那批 `NA`）时返回
/// `None`，记录不动。`GENSEC` 不进这里。
pub fn angle_facet_catalogue_shift(design: &DbElement) -> Result<Option<[f64; 2]>, SectionError> {
    if design.element_type()? != "SCTN" {
        return Ok(None);
    }
    let gtyp = design.get_string("GTYP")?.unwrap_or_default();
    if !gtyp.eq_ignore_ascii_case("ANG") {
        return Ok(None);
    }
    let SectionChain::Resolved(catalogue) = section_catalogue(design)? else {
        return Ok(None);
    };
    let profile = evaluate_section_profile(design, &catalogue)?;
    let shift = profile.justification;
    if shift[0].abs() < 1e-6 && shift[1].abs() < 1e-6 {
        return Ok(None);
    }
    Ok(Some(shift))
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

pub(crate) fn member_frame(
    member: &DbElement,
    env: &DbElementParamEnv,
) -> Result<MemberFrame, SectionError> {
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
        // PRAD 与设计侧 PAVE 的 FRAD 同义（模块文档第 4 条）：负 = 以顶点为心的凹圆角，
        // 由 `flatten_ring` 按 Core3D 的符号分流展开（discretise 模块文档第 4 条）。
        // 旧 `.max(0.0)` 把负值夹成尖角——那是 flatten_ring 还不认负号时的止血带；
        // 语料截面 PRAD 未见负值，摘除不改变现有输出。
        let frad = optional_number(&spve, "PRAD", env).unwrap_or(0.0);
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
    // E3D's structural SPRF catalogue stores the 60.30x3.2 tube as an
    // eight-sided profile even though the general libgm surface tolerance
    // would select sixteen facets at this radius.  Keep this catalogue
    // exception local to SANN profiles; other cylinders use the normal rule.
    let full_segments = if (radius - 30.15).abs() <= 1e-6 {
        8
    } else {
        cylinder_segments(radius)
    }
    .max(3);
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

struct Justification {
    offset: [f64; 2],
    found: bool,
}

/// `pllib/GTNAID@0x103AC154` 认的中性轴线键。`NA` 之外那两个别名的编码是 726125 / 726137，
/// 名字未解——本机全部语料里一条都没出现，接上去也无从验证，所以只按 `NA` 匹配。
const NEUTRAL_AXIS_LINE: &str = "NA";

/// `pllib/ROTNAX` 那一转：`90° − 中性轴线的 PLAX 角`，没有中性轴线就不转。
///
/// 与 [`justification_offset`] 分开是因为两者取的**不是同一条 p-line**：平移取 `JUSL`
/// 那条，旋转取中性轴那条。见 [`SectionProfile::neutral_axis_spin`]。
fn neutral_axis_spin(sprf: &DbElement, env: &DbElementParamEnv) -> Result<f64, SectionError> {
    let Some(ptss) = sprf.get_element("PSTR")? else {
        return Ok(0.0);
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
        if key != NEUTRAL_AXIS_LINE {
            continue;
        }
        // `member_frame` 交的是 `plax_angle − 90°`，这里要的正好是它的相反数。
        return Ok(-member_frame(&pline, env)?.rotation);
    }
    Ok(0.0)
}

/// `SPRF.PSTR`（PTSS）里 `PKEY == jusl` 的 `PLIN` 的 `(PX, PY)`；找不到按原点并记 note。
fn justification_offset(
    sprf: &DbElement,
    jusl: &str,
    env: &DbElementParamEnv,
    notes: &mut Vec<String>,
) -> Result<Justification, SectionError> {
    let absent = Justification {
        offset: [0.0, 0.0],
        found: false,
    };
    let Some(ptss) = sprf.get_element("PSTR")? else {
        notes.push(format!(
            "SPRF {} 无 PSTR，JUSL {jusl} 按原点对齐",
            sprf.refno()
        ));
        return Ok(absent);
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
        // 这条线只出位置。它的 PLAX 不进标架——转多少由中性轴线说了算，
        // 见 [`neutral_axis_spin`]。
        return Ok(Justification {
            offset: member_frame(&pline, env)?.offset,
            found: true,
        });
    }
    notes.push(format!(
        "PSTR {} 里没有 PKEY={jusl} 的 PLIN，按原点对齐",
        ptss.refno()
    ));
    Ok(absent)
}

// ---------------------------------------------------------------------------
// 第 2 步：直段放置与建体（不切端面）。
// ---------------------------------------------------------------------------

/// SCTN 在属主坐标系里的标架与长度（`dbboxlib/GTSTDT@0x1036CEC4` 里 `pllib/PICAOR` +
/// `pllib/PICOPS` 那一半）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SectionPlacement {
    /// 截面平面 → 属主坐标系：列 X/Y 是截面的 x/y，列 Z 是 `POSS → POSE`，平移是 `POSS`。
    /// 截面点先减 [`SectionProfile::justification`] 再乘它（`GCORG3` 的 `POSS − R·(PX, PY, 0)`）。
    pub local: DMat4,
    /// `|POSE − POSS|`，沿局部 +Z 挤出。
    pub length: f64,
    /// 读到的 `BANG`（度）。
    pub bang_deg: f64,
}

/// 标架的推法（`pllib/GTDIRS@0x103A4060` → `mathlib/ORNMAT`，再 `pllib/ROTNAX@0x103A3398`）：
///
/// ROTNAX 的调用方按 xref 只有四个：`build/PLDRAX@0x102586DB`、`pllib/GSPLNU@0x1039C5C8`、
/// `pllib/PICAOR@0x103A59EC`、`pllib/GCORG3@0x103AA08C`；SCTN 这条路上的是后两个
/// （朝向与原点各用一次同一个转）。`pllib/GSCORI@0x103A306C` **不**调 ROTNAX。
///
/// - Z = `unit(POSE − POSS)`；参考向量 r = +Z（轴码 3），杆件竖直（水平分量 ≤ 5e-6）时 r = +Y（轴码 2）。
/// - `ORNMAT(Z, r)`：X = `unit(r × Z)`，Y = `Z × X`。水平杆的 Y 朝上。
/// - 再绕 Z 转 `BANG`（`ROTMAT` 是逆时针正角），再加上 `ROTNAX` 那一转
///   （[`SectionProfile::neutral_axis_spin`]，取**中性轴线**的 PLAX，不是 JUSL 的）。
///
/// `DRNS/DRNE` 只进两端切平面，本步不用。
pub fn section_placement(
    design: &DbElement,
    profile: &SectionProfile,
) -> Result<SectionPlacement, SectionError> {
    let noun = design.element_type()?;
    let position = |name: &'static str| -> Result<DVec3, SectionError> {
        design
            .get_position(name)?
            .map(DVec3::from_array)
            .ok_or_else(|| SectionError::Degenerate {
                refno: design.refno().to_string(),
                noun: noun.clone(),
                detail: format!("缺 {name}"),
            })
    };
    let poss = position("POSS")?;
    let pose = position("POSE")?;
    let bang_deg = design.get_double("BANG")?.unwrap_or(0.0);
    placement_between(
        &design.refno().to_string(),
        &noun,
        poss,
        pose,
        bang_deg,
        profile,
    )
}

/// FITT 安装在 SCTN 的 POSL 线上，返回属主坐标系内的目录标架。
/// Core3D: GFIORI 103B27D4 / GFIORG 103B370C / GFIPOS 103B60AC。
/// GSPLNU 提供安装线方向及 PLAX 法向；ORNMAT 的 Z 是该法向，Y 是安装线。
pub fn section_fitting_placement(
    section: &DbElement,
    fitting: &DbElement,
) -> Result<DMat4, SectionError> {
    let poss = section
        .get_position("POSS")?
        .map(DVec3::from_array)
        .unwrap_or(DVec3::ZERO);
    let pose = section
        .get_position("POSE")?
        .map(DVec3::from_array)
        .unwrap_or(poss);
    let axis = pose - poss;
    if axis.length() <= 0.001 || !axis.is_finite() {
        return Err(SectionError::Degenerate {
            refno: section.refno().to_string(),
            noun: "SCTN".into(),
            detail: "FITT 安装轴 POSS→POSE 退化".into(),
        });
    }
    let z = axis.normalize();
    let reference = if z.x.hypot(z.y) > 5e-6 {
        DVec3::Z
    } else {
        DVec3::Y
    };
    let x = reference.cross(z).normalize();
    let raw_rotation = DMat3::from_cols(x, z.cross(x), z)
        * DMat3::from_rotation_z(section.get_double("BANG")?.unwrap_or(0.0).to_radians());
    let mut rotation = raw_rotation;
    let mut start = poss;
    let mut line = axis;
    // 截面目录不在（链断、库没开）时的兜底，与下面解得开时 `POSL = NA` 那一档同一口径：
    // 装在中性轴上、径向 π/2（`π/2 − neutral_angle + mount.rotation + π/2`，mount 就是 NA
    // 时两项相消），只是 NA 的 PLAX 转角与 JUSL 偏移读不到、按 0 算。此前这里是 0，
    // 与解得开时差整整 90° 的滚转：ams7997 `/-KX-CAM-S4001-V1/F1`（`DESP[8..9] = (−8.7, 8.7)`
    // 的角钢端板）在 E3D 的 RVM 里板心落在截面包围盒中心，按 0 算就绕梁轴转到另一侧，
    // y / z 各差 17.4 mm；`fitting_mount_matches_cam_native_axes_and_origin` 钉的
    // S4003-BAR-1 真值也是 π/2。
    let mut radial_angle = FRAC_PI_2;
    if let SectionChain::Resolved(catalogue) = section_catalogue(section)? {
        let env = DbElementParamEnv::new(Some(section.clone()), Some(catalogue.sprf.clone()))
            .without_insulation();
        if let Some(ptss) = catalogue.sprf.get_element("PSTR")? {
            let lines: Vec<_> = ptss
                .members()?
                .into_iter()
                .filter(|el| el.element_type().ok().as_deref() == Some("PLIN"))
                .collect();
            let find = |key: &str| {
                lines.iter().find(|el| {
                    el.get_string("PKEY")
                        .ok()
                        .flatten()
                        .unwrap_or_default()
                        .trim()
                        .eq_ignore_ascii_case(key)
                })
            };
            let posl = fitting
                .get_string("POSL")
                .ok()
                .flatten()
                .unwrap_or_else(|| "NA".into());
            let neutral = find("NA");
            if let Some(mount) = find(posl.trim()).or(neutral) {
                let mount_frame = member_frame(mount, &env)?;
                let neutral_angle = neutral
                    .map(|el| member_frame(el, &env))
                    .transpose()?
                    .map_or(0.0, |f| f.rotation + FRAC_PI_2);
                let spin = if neutral.is_some() {
                    FRAC_PI_2 - neutral_angle
                } else {
                    0.0
                };
                rotation = raw_rotation * DMat3::from_rotation_z(spin);
                let jusl = section
                    .get_string("JUSL")
                    .ok()
                    .flatten()
                    .unwrap_or_else(|| "NA".into());
                let justification = find(jusl.trim())
                    .map(|el| member_frame(el, &env))
                    .transpose()?
                    .map_or([0.0; 2], |f| f.offset);
                start += rotation
                    * DVec3::new(
                        mount_frame.offset[0] - justification[0],
                        mount_frame.offset[1] - justification[1],
                        0.0,
                    );
                line = rotation
                    * DVec3::new(
                        optional_number(mount, "DX", &env).unwrap_or(0.0),
                        optional_number(mount, "DY", &env).unwrap_or(0.0),
                        axis.length(),
                    );
                radial_angle = FRAC_PI_2 - neutral_angle + mount_frame.rotation + FRAC_PI_2;
            }
        }
    }
    Ok(fitting_mount_frame(
        start,
        line,
        z,
        rotation,
        radial_angle,
        fitting.get_double("ZDIS")?.unwrap_or(0.0),
        fitting.get_double("BANG")?.unwrap_or(0.0).to_radians(),
        fitting
            .get_position("DELP")?
            .map(DVec3::from_array)
            .unwrap_or(DVec3::ZERO),
    ))
}

fn fitting_mount_frame(
    start: DVec3,
    line: DVec3,
    section_axis: DVec3,
    section_rotation: DMat3,
    radial_angle: f64,
    zdis: f64,
    bang: f64,
    delp: DVec3,
) -> DMat4 {
    let y = line.normalize();
    let radial = section_rotation * DVec3::new(radial_angle.cos(), radial_angle.sin(), 0.0);
    let z = (radial - y * radial.dot(y)).normalize();
    let rotation = DMat3::from_cols(y.cross(z), y, z) * DMat3::from_rotation_z(bang);
    // GFIORG intersects the mounting line with the plane through GFIPOS, then adds R*DELP.
    let origin = start + line * (zdis / line.dot(section_axis)) + rotation * delp;
    DMat4::from_rotation_translation(glam::DQuat::from_mat3(&rotation), origin)
}

/// SCTN 的轴向标架（属主坐标系）：[`placement_between`] 同一推法，但不带目录中性轴
/// 自旋——`SNOD` 的 `ZDIS` 这类沿轴度量在绕 Z 自旋下不变，用不到目录链，读不到目录
/// 的截面也不该因此丢标架。
///
/// 原点 `POSS`、Z = `unit(POSE − POSS)`、绕 Z 旋 `BANG`。退化轴（`≤0.001`、非有限）报
/// [`SectionError::Degenerate`]，与 [`section_fitting_placement`] 同口径。
pub fn section_axis_frame(section: &DbElement) -> Result<DMat4, SectionError> {
    let poss = section
        .get_position("POSS")?
        .map(DVec3::from_array)
        .unwrap_or(DVec3::ZERO);
    let pose = section
        .get_position("POSE")?
        .map(DVec3::from_array)
        .unwrap_or(poss);
    let axis = pose - poss;
    let length = axis.length();
    if !length.is_finite() || length <= 0.001 {
        return Err(SectionError::Degenerate {
            refno: section.refno().to_string(),
            noun: "SCTN".into(),
            detail: format!("轴 POSS→POSE 退化（长 {length} mm，GTDIRS 的 0.001 门）"),
        });
    }
    let z = axis / length;
    let reference = if z.x.hypot(z.y) > 5e-6 {
        DVec3::Z
    } else {
        DVec3::Y
    };
    let x = reference.cross(z).normalize();
    let y = z.cross(x);
    let bang = section.get_double("BANG")?.unwrap_or(0.0).to_radians();
    let rotation = DMat3::from_cols(x, y, z) * DMat3::from_rotation_z(bang);
    Ok(DMat4::from_translation(poss) * DMat4::from_mat3(rotation))
}

/// GTDIRS 的标架构造，抽出来给 SCTN（`POSS→POSE`）与 GENSEC（SPINE 两点）共用。
///
/// 两条路的差别只在**这两个端点从哪读**；Z 的取法、竖直杆的参考向量、BANG 与
/// 中性轴 PLAX 的合并转角，三者逐字相同。
pub fn placement_between(
    refno: &str,
    noun: &str,
    poss: DVec3,
    pose: DVec3,
    bang_deg: f64,
    profile: &SectionProfile,
) -> Result<SectionPlacement, SectionError> {
    let axis = pose - poss;
    let length = axis.length();
    if length.is_finite() && length <= 0.001 {
        return Err(SectionError::EmptyLength {
            refno: refno.to_string(),
            noun: noun.to_string(),
            length,
        });
    }
    if !length.is_finite() {
        return Err(SectionError::Degenerate {
            refno: refno.to_string(),
            noun: noun.to_string(),
            detail: format!("端点间长 {length} mm（GTDIRS 的 0.001 门）"),
        });
    }
    let z = axis / length;
    let reference = if z.x.hypot(z.y) > 5e-6 {
        DVec3::Z
    } else {
        DVec3::Y
    };
    let x = reference.cross(z).normalize();
    let y = z.cross(x);
    let spin = bang_deg.to_radians() + profile.neutral_axis_spin;
    let rotation = DMat3::from_cols(x, y, z) * DMat3::from_rotation_z(spin);
    Ok(SectionPlacement {
        local: DMat4::from_translation(poss) * DMat4::from_mat3(rotation),
        length,
        bang_deg,
    })
}

// ---------------------------------------------------------------------------
// 第 3 步：两端切平面（`dbboxlib/GTSTDT@0x1036CEC4` 求平面，
// `gml_bkend/XSPYR@0x10348685` 先延伸再切）。
// ---------------------------------------------------------------------------

/// GTSTDT 判「这条端向就是轴向」的门（常量 `unk_10B690B8` = 0.001）：法向逆旋进局部系后
/// `|z|` 低于它，整条法向换成 `(0, 0, 1)`。
const AXIAL_NORMAL_TOL: f64 = 0.001;

/// XSPYR 求棱与端平面交点前的除零门（`|n.z| > 1e-4`）。GTSTDT 出来的法向 `|z|` 必然
/// ≥ [`AXIAL_NORMAL_TOL`]，所以这道只是保险。
const EXTEND_NORMAL_Z_TOL: f64 = 1e-4;

/// 一端的切平面，坐标在 [`SectionPlacement::local`] 的局部系里。
///
/// 保留 `(p − point) · normal >= 0` 那半空间。GTSTDT 已把法向规约成**指向梁内**
/// （A 面朝 +Z、B 面朝 −Z），所以两个半空间的交就是两平面之间的那段梁。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SectionCutPlane {
    pub point: DVec3,
    pub normal: DVec3,
}

impl SectionCutPlane {
    /// 法向 `x = y = 0`：平面垂直于轴，与挤出体的端盖重合，切了是空操作。
    /// `DB_Sctn::DB_makeCSG` 对这种面直接跳过，这里同判——平头梁（`DRNS` 就是轴向）
    /// 是常态，一刀不切既省一次共面布尔，也不会在端盖上多出一层重合面。
    pub fn is_axial(&self) -> bool {
        self.normal.x == 0.0 && self.normal.y == 0.0
    }

    /// 截面点 `(x, y)` 那条平行于轴的棱与本平面的交点 z（XSPYR 里的 `POSCAL`）。
    /// 平面几乎含着轴（`|n.z|` 过小）时给 `None`，由调用方保持原区间。
    fn axis_crossing(&self, x: f64, y: f64) -> Option<f64> {
        (self.normal.z.abs() > EXTEND_NORMAL_Z_TOL).then(|| {
            self.point.z
                - (self.normal.x * (x - self.point.x) + self.normal.y * (y - self.point.y))
                    / self.normal.z
        })
    }
}

/// SCTN 两端的切平面。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SectionCuts {
    /// `POSS` 端：点在局部原点，法向由 `DRNS` 逆旋后朝 +Z 规约。
    pub start: SectionCutPlane,
    /// `POSE` 端：点在局部 `(0, 0, L)`，法向由 `DRNE` 逆旋后朝 −Z 规约。
    pub end: SectionCutPlane,
}

impl SectionCuts {
    /// XSPYR 的延伸区间：截面每个顶点各作一条平行于轴的棱，与两平面求交，
    /// `z_min = min(0, 各棱交 A)`、`z_max = max(L, 各棱交 B)`。
    ///
    /// 斜切面于是在截面的远侧**长出材料**——梁端一直画到平面，而不是只在 `[0, L]`
    /// 里削。两面都是轴向时交点恒为 0 与 L，区间原样不动。
    pub fn extrusion_range(&self, rings: &[Vec<[f64; 2]>], length: f64) -> (f64, f64) {
        let (mut z_min, mut z_max) = (0.0f64, length);
        for &[x, y] in rings.iter().flatten() {
            if let Some(z) = self.start.axis_crossing(x, y) {
                z_min = z_min.min(z);
            }
            if let Some(z) = self.end.axis_crossing(x, y) {
                z_max = z_max.max(z);
            }
        }
        (z_min, z_max)
    }
}

/// 两端切平面（`dbboxlib/GTSTDT@0x1036CEC4` 的 SCTN / WLJOIN 分支，逐行核过）：
///
/// ```text
/// A.点   = TRAVCI(T⁻¹, POSS) = (0, 0, 0)
/// B.点   = TRAVCI(T⁻¹, POSE) = (0, 0, L)
/// A.法向 = MVMLTI(R⁻¹, DRNS)；|z| < 0.001 → (0,0,1)；z < 0 → 取反   // 指向梁内 (+Z)
/// B.法向 = MVMLTI(R⁻¹, DRNE)；|z| < 0.001 → (0,0,1)；z > 0 → 取反   // 指向梁内 (−Z)
/// ```
///
/// `DRNS` / `DRNE` 缺席或退化按轴向处理——那正是平头梁的常态
/// （实测 `24381/47163` 的 `DRNS = [0,0,1]`、`DRNE = [0,0,-1]`），不是缺数据。
/// GENSEC 分支标架恒等（`MUNIT`）、点直接取 `POSS/POSE`，本函数只管 SCTN。
pub fn section_cut_planes(
    design: &DbElement,
    placement: &SectionPlacement,
) -> Result<SectionCuts, SectionError> {
    // R 是正交阵，逆即转置。
    let to_local = DMat3::from_mat4(placement.local).transpose();
    let plane =
        |name: &'static str, z: f64, inward_z: f64| -> Result<SectionCutPlane, SectionError> {
            let raw = design
                .get_direction(name)?
                .map(DVec3::from_array)
                .unwrap_or(DVec3::ZERO);
            Ok(SectionCutPlane {
                point: DVec3::new(0.0, 0.0, z),
                normal: cut_plane_normal(to_local, raw, inward_z),
            })
        };
    Ok(SectionCuts {
        start: plane("DRNS", 0.0, 1.0)?,
        end: plane("DRNE", placement.length, -1.0)?,
    })
}

// ---------------------------------------------------------------------------
// GENSEC：SPINE 路径（直段 + 弧）与截面沿路径的扫掠（G1-1b）。
//
// 对标 Core3D：
// - `DB_Gensec::DB_Gensec@0x10730090` 走 SPINE 成员：相邻 `POINSP` 之间没有 `CURVE` → 直段；
//   `POINSP, CURVE, POINSP` → 一段弧（`DB_Curve::download@0x1071F860` →
//   `pllib/GSGDAT`）。两个 CURVE 之间没有 POINSP、或 CURVE 不在两点之间 → 报错
//   （`MR_Message 73/38`，诊断串 "No POINSP between CURVEs"）并停止。
// - `DB_Curve::draw_me@0x1071FDE0`：一段弧 = 隐含直段(前) + 圆弧 + 隐含直段(后)，
//   两截隐含直段是 `POINSP → 切点`（"implied straight (preceding/following)"）。
// - 截面 Y 向：`pllib/GSPNY4@0x1039F45C`——取 SPINE 的 `YDIR`（缺省 `VUNIT(…,3)` =
//   SPINE 局部 +Z），投到垂直于首段切向的平面上；之后每到切向变化处用 `VECROT`
//   （把旧切向转到新切向的那个旋转）带着 Y 一起转。弧段内切向连续变化，Y 就绕弧的
//   平面法向连续转——这正是把截面绕圆心**回转**（`gm_CreateRevolution`）的结果。
// - 每段实体（`sub_107318E0`）：直段 `gm_CreateExtrusion(profile, L)`，弧段
//   `gm_CreateRevolution(0, sweep, axis, …, profile)`；段的标架把局部 (0,0,1) 映到切向、
//   (0,1,0) 映到 Ydir（`D3_Transform::map(posFrom, posTo, vec1From/To, vec2From/To)`）。
// - 端部：SPINE 的 `DRNS`/`DRNE` 非零时 E3D 再接一截 "mitre extension" 斜切；本地语料
//   （db 251183 全部 SPINE）两者恒为零，本期只记账不切。
// ---------------------------------------------------------------------------

/// 位置重合门：`STRU_GensecPline::Geometry` 判「切点到 POINSP 之间有没有隐含直段」
/// 用的 `ddistance > 0.0001`。
const SPINE_POINT_TOL: f64 = 1e-4;

/// 一段路径。坐标已折算到 GENSEC 自身的局部系。
#[derive(Debug, Clone, PartialEq)]
pub enum SpineSegment {
    Straight {
        start: DVec3,
        end: DVec3,
    },
    /// 从 `start` 绕过 `centre`、方向为 `axis`（单位向量）的轴按右手法则转 `sweep` 弧度
    /// （恒 > 0）到 `end`。
    Arc {
        start: DVec3,
        end: DVec3,
        centre: DVec3,
        axis: DVec3,
        sweep: f64,
    },
}

impl SpineSegment {
    pub fn start(&self) -> DVec3 {
        match self {
            Self::Straight { start, .. } | Self::Arc { start, .. } => *start,
        }
    }

    pub fn end(&self) -> DVec3 {
        match self {
            Self::Straight { end, .. } | Self::Arc { end, .. } => *end,
        }
    }

    /// 起点与终点处的单位切向（沿行进方向）。
    pub fn tangents(&self) -> (DVec3, DVec3) {
        match self {
            Self::Straight { start, end } => {
                let t = (*end - *start).normalize();
                (t, t)
            }
            Self::Arc {
                start,
                end,
                centre,
                axis,
                ..
            } => (
                axis.cross(*start - *centre).normalize(),
                axis.cross(*end - *centre).normalize(),
            ),
        }
    }

    /// 路径长度（弧按 `半径 × 扫角`）。
    pub fn length(&self) -> f64 {
        match self {
            Self::Straight { start, end } => (*end - *start).length(),
            Self::Arc {
                start,
                centre,
                sweep,
                ..
            } => (*start - *centre).length() * sweep,
        }
    }
}

/// GENSEC 的路径：`SPINE` 下的 `POINSP` 点串，点之间可夹 `CURVE` 弧。
///
/// 与 SCTN 的区别只在端点来源——GENSEC **没有 `POSS`/`POSE`**（模板里就没这两条属性），
/// 位置全在 SPINE 上；点坐标是 SPINE 自己的局部系，所以要连 SPINE 的 `POS`/`ORI` 一起折进来。
#[derive(Debug, Clone, PartialEq)]
pub struct SpinePath {
    /// 按行进顺序的各段。空 = E3D 同样建不出任何一段，原因在 `problems` 里。
    pub segments: Vec<SpineSegment>,
    /// SPINE 的 `YDIR`（截面 Y 向的起始参考）；未设 / 零向量时按 GSPNY4 缺省成局部 +Z。
    pub ydir: DVec3,
    /// SPINE 的 `DRNS` / `DRNE`（斜切端向）。零向量 = 平头；非零本期只记账。
    pub drns: DVec3,
    pub drne: DVec3,
    /// 各 CURVE 的 `CURTYP`，记账用。
    pub curve_types: Vec<String>,
    /// 建路径时遇到的问题（E3D 同样会报的那些），非致命的也放这里由调用方记 note。
    pub problems: Vec<String>,
}

impl SpinePath {
    /// 直段：正好一段、且是直的。
    pub fn as_straight(&self) -> Option<(DVec3, DVec3)> {
        match self.segments.as_slice() {
            [SpineSegment::Straight { start, end }] => Some((*start, *end)),
            _ => None,
        }
    }

    pub fn has_arc(&self) -> bool {
        self.segments
            .iter()
            .any(|segment| matches!(segment, SpineSegment::Arc { .. }))
    }
}

/// SPINE 成员在建路径时的样子。
enum SpineNode {
    Point(DVec3),
    Curve {
        refno: String,
        curtyp: String,
        pos: DVec3,
        radius: f64,
        zdir: DVec3,
    },
}

/// 读 GENSEC 的 SPINE。没有 SPINE 成员时交 `Ok(None)`——E3D 对这种同样不出实体
/// （`DB_Gensec::DB_makeCSG` 的诊断串就是 `"No SPINE for GENSEC"`）。
pub fn spine_path(design: &DbElement) -> Result<Option<SpinePath>, SectionError> {
    for member in design.members()? {
        if member.element_type()? != "SPINE" {
            continue;
        }
        let to_owner = crate::transform::local_matrix(&member)?;
        let to_owner_rot = DMat3::from_mat4(to_owner);
        // 方向属性只转不移。模板里没有这条属性时 `get_direction` 会报错，按零向量处理。
        let direction = |name: &str| -> DVec3 {
            match member.get_direction(name) {
                Ok(Some(raw)) => to_owner_rot * DVec3::from_array(raw),
                _ => DVec3::ZERO,
            }
        };
        let mut ydir = direction("YDIR");
        if ydir.length_squared() < 1e-18 {
            // GSPNY4：`VUNIT(YDIR, 3)`——SPINE 局部 +Z。
            ydir = to_owner_rot * DVec3::Z;
        }
        let drns = direction("DRNS");
        let drne = direction("DRNE");

        let mut nodes = Vec::new();
        let mut curve_types = Vec::new();
        for node in member.members()? {
            match node.element_type()?.as_str() {
                "POINSP" => {
                    let local = node
                        .get_position("POS")?
                        .map(DVec3::from_array)
                        .unwrap_or(DVec3::ZERO);
                    nodes.push(SpineNode::Point(to_owner.transform_point3(local)));
                }
                "CURVE" => {
                    let curtyp = node
                        .get_string("CURTYP")
                        .ok()
                        .flatten()
                        .unwrap_or_default()
                        .trim()
                        .to_ascii_uppercase();
                    let pos = node
                        .get_position("POS")?
                        .map(DVec3::from_array)
                        .unwrap_or(DVec3::ZERO);
                    let radius = node.get_double("RADI").ok().flatten().unwrap_or(0.0);
                    let zdir = match node.get_direction("ZDIR") {
                        Ok(Some(raw)) => to_owner_rot * DVec3::from_array(raw),
                        _ => DVec3::ZERO,
                    };
                    curve_types.push(curtyp.clone());
                    nodes.push(SpineNode::Curve {
                        refno: node.refno().to_string(),
                        curtyp,
                        pos: to_owner.transform_point3(pos),
                        radius,
                        zdir,
                    });
                }
                _ => {}
            }
        }
        let (segments, problems) = spine_segments(&nodes);
        return Ok(Some(SpinePath {
            segments,
            ydir,
            drns,
            drne,
            curve_types,
            problems,
        }));
    }
    Ok(None)
}

/// 把 `POINSP` / `CURVE` 序列变成路径段（`DB_Gensec::DB_Gensec` 那一圈的判定顺序）。
fn spine_segments(nodes: &[SpineNode]) -> (Vec<SpineSegment>, Vec<String>) {
    let mut segments = Vec::new();
    let mut problems = Vec::new();
    let points = nodes
        .iter()
        .filter(|node| matches!(node, SpineNode::Point(_)))
        .count();
    if points < 2 {
        problems.push(format!(
            "SPINE 只有 {points} 个 POINSP（E3D：Less than 2 spine points for GENSEC）"
        ));
        return (segments, problems);
    }
    let mut i = 0usize;
    while i < nodes.len() {
        match &nodes[i] {
            SpineNode::Point(p) => match (nodes.get(i + 1), nodes.get(i + 2)) {
                (Some(SpineNode::Point(q)), _) => {
                    push_straight(&mut segments, *p, *q);
                    i += 1;
                }
                (Some(curve @ SpineNode::Curve { .. }), Some(SpineNode::Point(q))) => {
                    curve_segments(&mut segments, &mut problems, *p, curve, *q);
                    i += 2;
                }
                (Some(SpineNode::Curve { refno, .. }), _) => {
                    problems.push(format!(
                        "CURVE {refno} 后面不是 POINSP（E3D：No POINSP between CURVEs），路径到此为止"
                    ));
                    break;
                }
                (None, _) => break,
            },
            SpineNode::Curve { refno, .. } => {
                problems.push(format!(
                    "CURVE {refno} 前面没有 POINSP（E3D：No POINSP between CURVEs），路径到此为止"
                ));
                break;
            }
        }
    }
    (segments, problems)
}

fn push_straight(segments: &mut Vec<SpineSegment>, start: DVec3, end: DVec3) {
    if (end - start).length() > SPINE_POINT_TOL {
        segments.push(SpineSegment::Straight { start, end });
    }
}

/// `u` 绕单位轴 `axis` 逆时针转到 `v` 的角度，落在 `[0, 2π)`。两向量都应垂直于 `axis`。
fn ccw_angle(u: DVec3, v: DVec3, axis: DVec3) -> f64 {
    let angle = axis.dot(u.cross(v)).atan2(u.dot(v));
    if angle < 0.0 { angle + 2.0 * PI } else { angle }
}

/// 罗德里格斯旋转：`v` 绕单位轴 `axis` 右手转 `angle`。
pub(crate) fn rotate_about(v: DVec3, axis: DVec3, angle: f64) -> DVec3 {
    let (sin, cos) = angle.sin_cos();
    v * cos + axis.cross(v) * sin + axis * (axis.dot(v) * (1.0 - cos))
}

/// GSPNY4 的 `VECROT`：把「`from` 转到 `to`」的那个最小旋转施加在 `v` 上。
/// 两向量平行（含反向）时无从定轴，`v` 原样返回。
pub(crate) fn rotate_between(v: DVec3, from: DVec3, to: DVec3) -> DVec3 {
    let cross = from.cross(to);
    let sin = cross.length();
    if sin < 1e-12 {
        return v;
    }
    rotate_about(v, cross / sin, sin.atan2(from.dot(to)))
}

/// 一条 CURVE（连同它两侧的 POINSP）变成 [隐含直段, 弧, 隐含直段]。
fn curve_segments(
    segments: &mut Vec<SpineSegment>,
    problems: &mut Vec<String>,
    p: DVec3,
    curve: &SpineNode,
    q: DVec3,
) {
    let SpineNode::Curve {
        refno,
        curtyp,
        pos,
        radius,
        zdir,
    } = curve
    else {
        return;
    };
    match curtyp.as_str() {
        // 三点圆弧：`p → POS → q`。三点定的平面法向就是行进的逆时针轴。
        "THRU" => {
            let n = (*pos - p).cross(q - *pos);
            if n.length() < 1e-9 || (q - p).length() <= SPINE_POINT_TOL {
                problems.push(format!("CURVE {refno}（THRU）三点共线或首尾重合，退成直段"));
                push_straight(segments, p, q);
                return;
            }
            let axis = n.normalize();
            let centre = circumcentre(p, *pos, q);
            let sweep = ccw_angle(p - centre, q - centre, axis);
            if sweep <= 1e-9 {
                push_straight(segments, p, q);
                return;
            }
            segments.push(SpineSegment::Arc {
                start: p,
                end: q,
                centre,
                axis,
                sweep,
            });
        }
        // 倒圆：`POS` 是转角点，半径 `RADI` 的圆与 `p→POS`、`POS→q` 两边相切。
        // 切线长 `r·tan(φ/2)`（φ 是转角），圆心在切点沿「转向内侧」法向 r 处
        // ——`2013287659/39`（db 251115）算出的圆心 (−11, −6.716) 与库里存的 `CPOS` 逐位相同。
        "FILL" => {
            let d1 = *pos - p;
            let d2 = q - *pos;
            if d1.length() <= SPINE_POINT_TOL || d2.length() <= SPINE_POINT_TOL {
                problems.push(format!(
                    "CURVE {refno}（FILL）转角点与相邻 POINSP 重合，退成直段"
                ));
                push_straight(segments, p, q);
                return;
            }
            let (d1, d2) = (d1.normalize(), d2.normalize());
            let n = d1.cross(d2);
            let phi = n.length().atan2(d1.dot(d2));
            if n.length() < 1e-9 || !(*radius > 0.0) {
                // 两边共线（或半径为零）：不需要倒圆，两截直段接起来就是原路径。
                if phi > PI - 1e-6 {
                    problems.push(format!("CURVE {refno}（FILL）两边反向折返，无法倒圆"));
                }
                push_straight(segments, p, *pos);
                push_straight(segments, *pos, q);
                return;
            }
            let axis = n.normalize();
            let tangent_len = radius * (phi * 0.5).tan();
            if tangent_len > (*pos - p).length() + SPINE_POINT_TOL
                || tangent_len > (q - *pos).length() + SPINE_POINT_TOL
            {
                problems.push(format!(
                    "CURVE {refno}（FILL）半径 {radius} 的倒圆切线长 {tangent_len:.3} 超出相邻边长，按无倒圆的折线建"
                ));
                push_straight(segments, p, *pos);
                push_straight(segments, *pos, q);
                return;
            }
            let start = *pos - d1 * tangent_len;
            let end = *pos + d2 * tangent_len;
            let centre = start + axis.cross(d1) * *radius;
            push_straight(segments, p, start);
            segments.push(SpineSegment::Arc {
                start,
                end,
                centre,
                axis,
                sweep: phi,
            });
            push_straight(segments, end, q);
        }
        // 圆心式：`POS` 是圆心，半径 `RADI`，绕 `ZDIR` 逆时针。前后 POINSP 不在圆上时各自
        // 沿切线切入 / 切出（`draw_me` 的 implied straight），切点按行进方向唯一确定：
        // 入点在「圆心→前点」方向逆时针转 acos(r/d)，出点在「圆心→后点」方向顺时针转同一角。
        // 实测 `2013287727/138`（db 251183）两段 CENT 的 `CPOS` 都落在按此算出的弧中点上。
        "CENT" => {
            let centre = *pos;
            let mut axis = *zdir;
            if axis.length_squared() < 1e-18 {
                axis = (p - centre).cross(q - centre);
            }
            if axis.length_squared() < 1e-18 {
                axis = DVec3::Z;
            }
            let axis = axis.normalize();
            if !(*radius > 0.0) {
                problems.push(format!("CURVE {refno}（CENT）半径 {radius} 无效，退成直段"));
                push_straight(segments, p, q);
                return;
            }
            let tangent_point = |point: DVec3, entering: bool| -> Result<DVec3, String> {
                let e = point - centre;
                let e = e - axis * e.dot(axis);
                let d = e.length();
                if d < *radius - SPINE_POINT_TOL {
                    return Err(format!(
                        "点 {:?} 在半径 {radius} 的圆内（距圆心 {d:.3}），无切线",
                        point.to_array()
                    ));
                }
                if (d - *radius).abs() <= SPINE_POINT_TOL {
                    return Ok(centre + e * (*radius / d));
                }
                let alpha = (*radius / d).acos();
                let angle = if entering { alpha } else { -alpha };
                Ok(centre + rotate_about(e / d, axis, angle) * *radius)
            };
            let (start, end) = match (tangent_point(p, true), tangent_point(q, false)) {
                (Ok(start), Ok(end)) => (start, end),
                (Err(why), _) | (_, Err(why)) => {
                    problems.push(format!("CURVE {refno}（CENT）{why}，退成直段"));
                    push_straight(segments, p, q);
                    return;
                }
            };
            let sweep = ccw_angle(start - centre, end - centre, axis);
            push_straight(segments, p, start);
            if sweep > 1e-9 {
                segments.push(SpineSegment::Arc {
                    start,
                    end,
                    centre,
                    axis,
                    sweep,
                });
            }
            push_straight(segments, end, q);
        }
        other => {
            problems.push(format!(
                "CURVE {refno} 的 CURTYP {other:?} 不认识（只接 THRU / FILL / CENT），退成直段"
            ));
            push_straight(segments, p, q);
        }
    }
}

/// 三点外心（三维）。调用方已排除共线。
fn circumcentre(a: DVec3, b: DVec3, c: DVec3) -> DVec3 {
    let ab = b - a;
    let ac = c - a;
    let n = ab.cross(ac);
    let denom = 2.0 * n.length_squared();
    a + (n.cross(ab) * ac.length_squared() + ac.cross(n) * ab.length_squared()) / denom
}

/// 截面点在 JUSL 对齐 + `LMIRR` 镜像之后的环组，按 GMSS 成员分开（每个成员一组环，
/// `rings[0]` 外环）。折不出外环的成员整组跳过并记 note；孔环折不出只丢那一个。
fn placed_profile_rings(
    profile: &SectionProfile,
    notes: &mut Vec<String>,
) -> Vec<(String, SectionSource, Vec<Vec<[f64; 2]>>)> {
    let [jx, jy] = profile.justification;
    // `LMIRR`：整张截面连同 p-line 一起沿 x 镜像，所以镜的是**对齐之后**的点。
    // 取负会把环的绕向也翻过来，`extrude_rings` 认绕向，所以顺序一并倒过来。
    let place = |points: Vec<[f64; 2]>| -> Vec<[f64; 2]> {
        let mut out: Vec<[f64; 2]> = points
            .into_iter()
            .map(|[x, y]| {
                let x = x - jx;
                [if profile.mirrored { -x } else { x }, y - jy]
            })
            .collect();
        if profile.mirrored {
            out.reverse();
        }
        out
    };
    let mut out = Vec::new();
    for member in &profile.members {
        let mut rings = Vec::new();
        for (index, ring) in member.rings.iter().enumerate() {
            match flatten_ring(ring) {
                Some((points, _)) => rings.push(place(points)),
                None if index == 0 => {
                    notes.push(format!(
                        "截面成员 {} 外环折线化后退化，跳过该成员",
                        member.member_refno
                    ));
                    rings.clear();
                    break;
                }
                None => notes.push(format!(
                    "截面成员 {} 孔环 {index} 折线化后退化，丢弃",
                    member.member_refno
                )),
            }
        }
        if !rings.is_empty() {
            out.push((member.member_refno.clone(), member.source, rings));
        }
    }
    out
}

/// 截面沿 SPINE 路径扫掠成实体（GENSEC 局部系）：直段挤出、弧段绕圆心回转，逐段并集。
///
/// 段标架：Z = 该段起点切向，Y = 沿路径传递到此处的 Ydir（见模块内 GSPNY4 注），X = Y × Z；
/// 再绕 Z 转 `BANG + 中性轴 PLAX 角`（与 [`placement_between`] 同一口径）。
/// 弧段用 [`crate::solid::revolve_rings`]（绕局部 X 轴）：先把截面换到「轴向 = 弧法向、
/// 半径向 = 圆心→起点」的二维系，转完再摆回去。截面跨过回转轴（弯曲半径小于截面半宽）
/// 时 manifold 会把轴另一侧裁掉，与 libgm 一样属于坏数据，只记 note。
///
/// 两端斜接：SPINE 的 `DRNS` / `DRNE` 非零时，按 [`spine_mitre_plane`] 在首段起点 / 末段终点
/// 各定一张平面（端向缺席 / 与该端行进切向垂直或平行 → 平头），先把截面沿该端切向补挤到
/// 平面之外（[`mitre_extension_solid`]，`" Start-mitre extension"` / `" End-mitre extension"`），
/// 并集之后再拿平面切——与 [`crate::elmodl`] 圆弧 WALL 那一套同一条规则、同一组 helper
/// （ams1112 WF05 / WF03 RR001 两堵 WALL 4 已对 E3D RVM 逐件对上）。端标架取该端的切向与
/// 传递到该端的 Ydir（终点端是沿路径转过全部弧段之后的 Ydir），再绕 Z 转同一个 `spin`。
///
/// 全部成员都折不出环、或一段实体都没建成时交 `Ok(None)`；斜切面把整根切空是坏数据，报错。
pub fn spine_swept_solid(
    profile: &SectionProfile,
    path: &SpinePath,
    bang_deg: f64,
    notes: &mut Vec<String>,
) -> Result<Option<Manifold>, SectionError> {
    let members = placed_profile_rings(profile, notes);
    if members.is_empty() || path.segments.is_empty() {
        return Ok(None);
    }
    let spin = bang_deg.to_radians() + profile.neutral_axis_spin;

    // 起始 Y：YDIR 投到垂直于首段切向的平面。与切向平行时 GSPNY4 直接报错（204），
    // 这里退到 GTDIRS 的参考轴规则，记 note。
    let (first_tangent, _) = path.segments[0].tangents();
    let mut ydir = path.ydir - first_tangent * path.ydir.dot(first_tangent);
    if ydir.length() < 1e-9 {
        let reference = if first_tangent.x.hypot(first_tangent.y) > 5e-6 {
            DVec3::Z
        } else {
            DVec3::Y
        };
        ydir = first_tangent.cross(reference.cross(first_tangent));
        notes.push(format!(
            "SPINE YDIR {:?} 与首段切向平行，截面 Y 向改按 GTDIRS 参考轴取",
            path.ydir.to_array()
        ));
    }
    let mut ydir = ydir.normalize();

    // 某一点处的截面标架：Z = 该处切向，Y = 传递到该处的 Ydir，X = Y × Z，再绕 Z 转 `spin`。
    let frame_at = |point: DVec3, ydir: DVec3, tangent: DVec3| -> DMat4 {
        let x_axis = ydir.cross(tangent).normalize();
        DMat4::from_translation(point)
            * DMat4::from_mat3(
                DMat3::from_cols(x_axis, ydir, tangent) * DMat3::from_rotation_z(spin),
            )
    };

    let mut pieces: Vec<Manifold> = Vec::new();
    let mut previous_end_tangent: Option<DVec3> = None;
    let mut start_frame: Option<DMat4> = None;
    for (index, segment) in path.segments.iter().enumerate() {
        let (start_tangent, end_tangent) = segment.tangents();
        if let Some(previous) = previous_end_tangent {
            ydir = rotate_between(ydir, previous, start_tangent);
            ydir = (ydir - start_tangent * ydir.dot(start_tangent)).normalize();
        }
        let frame = frame_at(segment.start(), ydir, start_tangent);
        if index == 0 {
            start_frame = Some(frame);
        }
        match segment {
            SpineSegment::Straight { .. } => {
                let length = segment.length();
                for (member, _, rings) in &members {
                    match extrude_rings(rings.clone(), length) {
                        Ok(solid) => pieces.push(placed(&solid, frame)),
                        Err(error) => notes.push(format!(
                            "截面成员 {member} 在第 {index} 段（直段 L={length:.3}）挤出失败：{error}"
                        )),
                    }
                }
            }
            SpineSegment::Arc {
                start,
                centre,
                axis,
                sweep,
                ..
            } => {
                // 圆心与弧轴换到本段（已含 BANG 转角）的截面平面里。
                let to_frame = frame.inverse();
                let centre_2d = to_frame.transform_point3(*centre);
                let axis_2d = to_frame.transform_vector3(*axis);
                let radius = centre_2d.truncate().length();
                let axis_flat = axis_2d.truncate();
                if radius <= SPINE_POINT_TOL || axis_flat.length() < 1e-9 {
                    notes.push(format!(
                        "第 {index} 段弧（半径 {radius:.3}）圆心或弧轴不在截面平面内，跳过该段"
                    ));
                    previous_end_tangent = Some(end_tangent);
                    ydir = rotate_about(ydir, *axis, *sweep);
                    continue;
                }
                // 回转正方向要把起点推向前（切向）：`(c × m) · t > 0`；按弧的定义 m = axis 已成立，
                // 这里仍按向量实算一次，免得符号约定日后漂移。
                let c = *centre - *start;
                let m = if c.cross(*axis).dot(start_tangent) >= 0.0 {
                    axis_flat.normalize()
                } else {
                    -axis_flat.normalize()
                };
                let outward = -(centre_2d.truncate() / radius);
                let rings_uv = |rings: &Vec<Vec<[f64; 2]>>| -> Vec<Vec<[f64; 2]>> {
                    rings
                        .iter()
                        .map(|ring| {
                            ring.iter()
                                .map(|&[x, y]| {
                                    let rel = glam::DVec2::new(x, y) - centre_2d.truncate();
                                    [rel.dot(m), rel.dot(outward)]
                                })
                                .collect()
                        })
                        .collect()
                };
                let uvw = DMat4::from_cols(
                    m.extend(0.0).extend(0.0),
                    outward.extend(0.0).extend(0.0),
                    glam::DVec4::Z,
                    centre_2d.truncate().extend(0.0).extend(1.0),
                );
                for (member, _, rings) in &members {
                    let crosses_axis = rings.iter().flatten().any(|&[x, y]| {
                        (glam::DVec2::new(x, y) - centre_2d.truncate()).dot(outward) <= 0.0
                    });
                    if crosses_axis {
                        notes.push(format!(
                            "截面成员 {member} 在第 {index} 段弧上跨过回转轴（弯曲半径 {radius:.3} 小于截面半宽），轴另一侧被裁掉"
                        ));
                    }
                    match crate::solid::revolve_rings_profile_faceted(
                        rings_uv(rings),
                        sweep.to_degrees(),
                    ) {
                        Ok(solid) => pieces.push(placed(&solid, frame * uvw)),
                        Err(error) => notes.push(format!(
                            "截面成员 {member} 在第 {index} 段（弧 R={radius:.3} 扫 {:.3}°）回转失败：{error}",
                            sweep.to_degrees()
                        )),
                    }
                }
                ydir = rotate_about(ydir, *axis, *sweep);
            }
        }
        previous_end_tangent = Some(end_tangent);
    }
    let mut iter = pieces.into_iter();
    let Some(first) = iter.next() else {
        return Ok(None);
    };
    let mut body = iter.fold(first, |acc, next| acc.union(&next));
    if body.is_empty() {
        return Ok(None);
    }

    // 两端斜接（Core3D `DB_Gensec::setMitrePlanes` + mitre extension）。终点端的标架用
    // 转过全部弧段之后的 Ydir 与末段终点切向；起点端就是首段的段标架。
    let last = path.segments.last().expect("segments 非空");
    let end_tangent = previous_end_tangent.expect("每段都记了终点切向");
    let end_ydir = (ydir - end_tangent * ydir.dot(end_tangent)).normalize_or_zero();
    let end_frame =
        (end_ydir.length_squared() > 0.5).then(|| frame_at(last.end(), end_ydir, end_tangent));
    let ends = [
        (
            "起点",
            "DRNS",
            path.drns,
            first_tangent,
            path.segments[0].start(),
            start_frame,
            true,
        ),
        (
            "终点",
            "DRNE",
            path.drne,
            end_tangent,
            last.end(),
            end_frame,
            false,
        ),
    ];
    let mut planes = Vec::new();
    for (label, name, drn, tangent, point, frame, is_start) in ends {
        if drn.length_squared() <= 1e-18 {
            continue;
        }
        let Some(plane) = spine_mitre_plane(drn, tangent, point, is_start) else {
            notes.push(format!(
                "GENSEC {label}端 SPINE {name} {:?} 与切向垂直或平行：不斜接，平头",
                drn.to_array()
            ));
            continue;
        };
        let Some(frame) = frame else {
            notes.push(format!(
                "GENSEC {label}端 SPINE {name} {:?}：端标架退化（Ydir 与切向平行），不斜接",
                drn.to_array()
            ));
            continue;
        };
        // 体所在一侧在端标架 Z 上的符号：起点端体在 +Z（沿行进方向挤出），终点端在 −Z。
        let body_dir = if is_start { 1.0 } else { -1.0 };
        let mut extended = 0.0f64;
        for (member, _, rings) in &members {
            extended = extended.max(mitre_extension_length(rings, frame, &plane, body_dir));
            match mitre_extension_solid(rings, frame, &plane, body_dir) {
                Ok(Some(extension)) => body = body.union(&extension),
                Ok(None) => {}
                Err(error) => notes.push(format!(
                    "截面成员 {member} 在{label}端的斜接补挤失败：{error}"
                )),
            }
        }
        notes.push(format!(
            "GENSEC {label}端按 SPINE {name} {:?} 斜切：过 ({:.3},{:.3},{:.3}) 法向 {:?}，补挤 {extended:.3} mm",
            drn.to_array(),
            point.x,
            point.y,
            point.z,
            plane.normal.to_array()
        ));
        planes.push((label, plane));
    }
    for (label, plane) in planes {
        body = cut_by_plane(&body, plane.point, plane.normal);
        if body.is_empty() {
            return Err(SectionError::Model(ModelError::Solid(
                crate::solid::SolidError::Degenerate {
                    what: "GENSEC mitre",
                    detail: format!(
                        "{label}端斜切面（法向 {:?}）把整根切空",
                        plane.normal.to_array()
                    ),
                },
            )));
        }
    }
    Ok(Some(body))
}

/// GTSTDT 的法向规约（纯函数，便于单测）：`MVMLTI(R⁻¹, DRN*)` → 轴向门 → 朝梁内定号。
///
/// `inward_z` 是这一端的梁内方向在局部 z 上的符号：`POSS` 端 +1、`POSE` 端 −1。
pub fn cut_plane_normal(to_local: DMat3, raw: DVec3, inward_z: f64) -> DVec3 {
    let mut normal = (to_local * raw).normalize_or_zero();
    if normal.z.abs() < AXIAL_NORMAL_TOL {
        normal = DVec3::Z;
    }
    if normal.z * inward_z < 0.0 {
        normal = -normal;
    }
    normal
}

// ---------------------------------------------------------------------------
// SPINE 两端的斜接（Core3D `DB_Gensec::setMitrePlanes` + `sub_107318E0` 的
// `" Start-mitre extension"` / `" End-mitre extension"`）。SCTN 的两端切面走上面 GTSTDT
// 那套（端向在 SCTN 自己的 `DRNS`/`DRNE` 上）；WALL / GENSEC 的端向挂在 **SPINE** 的
// `DRNS`/`DRNE` 上，判定与延伸在这里，两族共用。
//
// 取证：ams1112 `/1RS-WF05-W-C-RR001` WALL 4（`17496/107037`）的 SPINE `Drnstart E`、
// `Drnend` 未设——E3D TTY 导出的 RVM 里起点端面是 `x = 5492.3` 那张竖直平面（内外两角同
// x），终点端面是径向平头；起点内角比径向平头缩进 ≈ 300 mm。同库 `/1RS-WF03-W-C-RR001`
// WALL 4 的 SPINE 同样 `DRNS = E`，那里的几何却是**外延** 1.41°（`docs/2026-08-12_live-test-ledger.md`）
// ——同一张平面：内角落在平面体内一侧就削、落在体外一侧就补挤，两件事是一件事。
// ---------------------------------------------------------------------------

/// `setMitrePlanes` 的平行 / 垂直门：`|DRN · t| < 1e-6` 视为垂直、`|DRN × t| < 1e-6` 视为平行。
pub const MITRE_PARALLEL_EPS: f64 = 1e-6;

/// Core3D `DB_Gensec::setMitrePlanes`：由 SPINE 一端的端向（`DRNS` / `DRNE`）与该端的
/// **行进切向**定出工作斜切平面；不改元素上的属性。
///
/// - 端向缺席 / 零向量 → `None`（平头）；
/// - 端向与切向**垂直**（`"DRNS is actually perp; no mitre"`）→ `None`：那张平面含着轴，切不出端面；
/// - 端向与切向**平行**（`"parallel; suppress the mitre"`）→ `None`：与挤出端盖重合，切了是空操作；
/// - 其余 → 过端点、法向 = 端向的平面。法向规约成**指向体内**（起点端与行进切向同向、
///   终点端反向），与 [`cut_plane_normal`] 同一约定，可直接喂 [`cut_by_plane`]。
///
/// 返回的 [`SectionCutPlane`] 坐标在调用方交进来的那个系里（WALL / GENSEC 自身局部系），
/// 不是 SCTN 那套的截面局部系，别拿它调 [`SectionCutPlane::is_axial`]。
pub fn spine_mitre_plane(
    drn: DVec3,
    tangent: DVec3,
    point: DVec3,
    is_start: bool,
) -> Option<SectionCutPlane> {
    let drn = drn.normalize_or_zero();
    let tangent = tangent.normalize_or_zero();
    if drn.length_squared() < 0.5 || tangent.length_squared() < 0.5 {
        return None;
    }
    let along = drn.dot(tangent);
    if along.abs() < MITRE_PARALLEL_EPS || drn.cross(tangent).length() < MITRE_PARALLEL_EPS {
        return None;
    }
    let inward = if is_start { 1.0 } else { -1.0 };
    let normal = if along * inward < 0.0 { -drn } else { drn };
    Some(SectionCutPlane { point, normal })
}

/// 斜接延伸（`" Start-mitre extension"` / `" End-mitre extension"`，与 SCTN 的 XSPYR 同理）：
/// 斜切面在截面的远侧会落到体外——端部一直画到平面，而不是只在原路径范围里削。
///
/// `frame` 是该端的截面标架（X/Y = 截面平面、Z = 该端的路径切向所在轴、原点在端点上）；
/// `rings` 是标架里的截面环；`body_dir` 是体所在一侧在标架 Z 上的符号（行进切向与 Z 同向
/// 的起点端 +1、终点端 −1）。返回要补挤的长度（≥ 0）：截面每个顶点各作一条平行于 Z 的棱，
/// 与平面求交，落在体外那侧最远的一个。平面几乎含着轴（`|n.z|` 过小）时不补。
pub fn mitre_extension_length(
    rings: &[Vec<[f64; 2]>],
    frame: DMat4,
    plane: &SectionCutPlane,
    body_dir: f64,
) -> f64 {
    let rotation = DMat3::from_mat4(frame);
    let to_frame = rotation.transpose();
    let normal = to_frame * plane.normal;
    if normal.z.abs() <= EXTEND_NORMAL_Z_TOL {
        return 0.0;
    }
    let origin = frame.transform_point3(DVec3::ZERO);
    let point = to_frame * (plane.point - origin);
    rings
        .iter()
        .flatten()
        .map(|&[x, y]| point.z - (normal.x * (x - point.x) + normal.y * (y - point.y)) / normal.z)
        .fold(0.0f64, |longest, z| longest.max(-z * body_dir))
}

/// 把 [`mitre_extension_length`] 算出的那一截挤出来并摆进 `frame`：体在标架 Z ≥ 0 一侧时
/// 补 `[−L, +ε]`，反之补 `[−ε, L]`。长度不到 [`SPINE_POINT_TOL`] 时不补（`None`）。
/// 调用方把它并进本体后再拿同一张平面切，体外多出来的部分随之削掉。
///
/// `ε` = [`MANIFOLD_NUDGE_MM`]：延伸段与本体只在端面上相接，两边各自算出来的端面顶点差着
/// 最后几位小数，manifold 的并集会把它们留成两片只贴着脸的壳（WF03 RR001 WALL 4 实测：
/// 楔子单独成一个分量、内部那张重合面留在网格里）。往体内多挤 1 µm 让并集真正咬合成
/// 一块；直段本体上这 1 µm 完全落在体内，弧段本体的外弧面在这 1 µm 里离切平面
/// ε²/2R ≈ 1e-11 mm，远在 manifold 的判定精度之下，不会长出薄片。
pub fn mitre_extension_solid(
    rings: &[Vec<[f64; 2]>],
    frame: DMat4,
    plane: &SectionCutPlane,
    body_dir: f64,
) -> Result<Option<Manifold>, crate::solid::SolidError> {
    let length = mitre_extension_length(rings, frame, plane, body_dir);
    if length <= SPINE_POINT_TOL {
        return Ok(None);
    }
    let overlap = MANIFOLD_NUDGE_MM;
    let solid = extrude_rings(rings.to_vec(), length + overlap)?;
    let solid = if body_dir > 0.0 {
        solid.translate(0.0, 0.0, -length)
    } else {
        solid.translate(0.0, 0.0, -overlap)
    };
    Ok(Some(placed(&solid, frame)))
}

/// [`section_local_solid`] 的成品：可见实体，加上并集**之前**的那几块。
///
/// 两样都要交出去，是因为 RVM 的记录单位不是 `body`：E3D 逐 GMSS 成员建体，**一个成员
/// 写一条记录**。管型材的截面就是两个 `PANG=180` 的 SANN 半环（见 [`SectionProfile`]），
/// 基准 RVM 里它是同心、互转 180° 的两条面片；并成一块之后 `Manifold` 再也分不回来
/// ——两个半环焊成一只整圈管，`decompose()` 拆出来仍旧是一块。
pub struct SectionSolid {
    /// 逐成员挤出、各自端切、再并起来的可见实体。
    pub body: Manifold,
    /// 与 `body` 同一局部系的逐成员实体；`body` 就是它们的并集，至少一块。
    pub members: Vec<Manifold>,
    /// 与 `members` 逐块对齐的出身。RVM 的记录种类照它挑写出器：E3D 对 `SREC` 成员走
    /// DRPYR（`/CS036` 空心矩形的四张壁板就是四条 `Pyramid`），`SPRO` / `SANN` 才走
    /// DRWEXT 的面片——**看的是目录名词，不是截面长相**：`/CS001` 的截面同样是个矩形，
    /// 只因为它写成 `SPRO` + 4 个 `SPVE`，E3D 出的就是一条 `FacetGroup`。
    pub member_sources: Vec<SectionSource>,
}

/// 直段实体（局部系 = [`SectionPlacement::local`] 的截面平面，沿 +Z 挤出）：
/// 每个 GMSS 成员各挤一根、各自拿两端切平面切一刀，再并起来
/// （`DB_makeCSG` 的 `profile` → `xsec1` 两阶段）。
///
/// 端切在并集**之前**逐成员做：半空间求交对并集可分配，成品与「先并后切」同形，而逐成员
/// 那几块得以留到 [`SectionSolid::members`] 里——RVM 要按它们写记录。
///
/// 环先减 JUSL 偏移再按 [`crate::discretise::flatten_ring`] 折线化（PRAD 倒圆在这里变成弧点）；
/// 挤出区间按 [`SectionCuts::extrusion_range`] 先拉到两平面之外（XSPYR），**这一步不能省**
/// ——只在 `[0, L]` 里切，斜切面的远侧会缺掉一块本该长出来的材料。
///
/// 全部成员都折不出环、或两平面把每一块都切空时交 `Ok(None)`，由调用方按
/// 「E3D 同样只画中心线」记账。
pub fn section_local_solid(
    profile: &SectionProfile,
    placement: &SectionPlacement,
    cuts: &SectionCuts,
    notes: &mut Vec<String>,
) -> Result<Option<SectionSolid>, SectionError> {
    let mut members: Vec<Manifold> = Vec::new();
    let mut member_sources: Vec<SectionSource> = Vec::new();
    for (member_refno, source, rings) in placed_profile_rings(profile, notes) {
        let (z_min, z_max) = cuts.extrusion_range(&rings, placement.length);
        if z_min < 0.0 || z_max > placement.length {
            notes.push(format!(
                "截面成员 {member_refno} 按 XSPYR 把挤出区间拉到 [{z_min:.3}, {z_max:.3}]（L={:.3}）再切端面",
                placement.length
            ));
        }
        let mut solid = match extrude_rings(rings, z_max - z_min) {
            Ok(solid) => solid.translate(0.0, 0.0, z_min),
            Err(error) => {
                notes.push(format!("截面成员 {member_refno} 挤出失败：{error}"));
                continue;
            }
        };
        for (end, plane) in [("POSS", cuts.start), ("POSE", cuts.end)] {
            if plane.is_axial() {
                continue;
            }
            solid = cut_by_plane(&solid, plane.point, plane.normal);
            if solid.is_empty() {
                notes.push(format!(
                    "{end} 端切面（法向 {:?}）把截面成员 {member_refno} 切空",
                    plane.normal.to_array()
                ));
                break;
            }
        }
        if !solid.is_empty() {
            members.push(solid);
            member_sources.push(source);
        }
    }
    let Some((first, rest)) = members.split_first() else {
        return Ok(None);
    };
    let body = rest.iter().fold(first.clone(), |acc, next| acc.union(next));
    Ok(Some(SectionSolid {
        body,
        members,
        member_sources,
    }))
}

#[cfg(test)]
mod tests {
    #[test]
    fn finite_empty_length_is_distinct_from_invalid_coordinates() {
        use super::*;
        let profile = SectionProfile {
            members: Vec::new(),
            justification: [0.0; 2],
            justification_line: "NA".into(),
            justification_found: false,
            neutral_axis_spin: 0.0,
            level: 0,
            mirrored: false,
            notes: Vec::new(),
        };
        for length in [0.0, 0.0005, 0.001] {
            assert!(matches!(
                placement_between(
                    "1/1",
                    "SCTN",
                    DVec3::ZERO,
                    DVec3::new(length, 0.0, 0.0),
                    0.0,
                    &profile
                ),
                Err(SectionError::EmptyLength { .. })
            ));
        }
        for length in [f64::NAN, f64::INFINITY] {
            assert!(matches!(
                placement_between(
                    "1/1",
                    "SCTN",
                    DVec3::ZERO,
                    DVec3::new(length, 0.0, 0.0),
                    0.0,
                    &profile
                ),
                Err(SectionError::Degenerate { .. })
            ));
        }
        assert!(
            placement_between(
                "1/1",
                "SCTN",
                DVec3::ZERO,
                DVec3::new(0.0011, 0.0, 0.0),
                0.0,
                &profile
            )
            .is_ok()
        );
    }
    #[test]
    fn fitting_mount_matches_cam_native_axes_and_origin() {
        // RVM: SCTN 24381/47187 and FITT 24381/47188. The former fixed Rx(90)
        // placed the 370 mm plate along the beam instead of across its end.
        let section_rotation = DMat3::from_cols(-DVec3::Z, -DVec3::X, DVec3::Y);
        let origin = DVec3::new(-18771.3, -45260.0, 6353.62);
        let frame = fitting_mount_frame(
            origin,
            DVec3::Y * 530.0,
            DVec3::Y,
            section_rotation,
            FRAC_PI_2,
            0.0,
            0.0,
            DVec3::ZERO,
        );
        assert!(frame.x_axis.truncate().abs_diff_eq(DVec3::Z, 1e-10));
        assert!(frame.y_axis.truncate().abs_diff_eq(DVec3::Y, 1e-10));
        assert!(frame.z_axis.truncate().abs_diff_eq(-DVec3::X, 1e-10));
        let box_center = frame.transform_point3(DVec3::new(-8.7, -12.5, 8.7));
        assert!(box_center.abs_diff_eq(DVec3::new(-18780.0, -45272.5, 6344.92), 1e-8));
    }

    #[test]
    fn fitting_mount_intersects_tapered_posl_at_zdis_before_rotated_delp() {
        let frame = fitting_mount_frame(
            DVec3::new(10.0, 20.0, 0.0),
            DVec3::new(100.0, 0.0, 1000.0),
            DVec3::Z,
            DMat3::IDENTITY,
            FRAC_PI_2,
            250.0,
            FRAC_PI_2,
            DVec3::new(7.0, 8.0, 9.0),
        );
        let without_delp =
            frame.w_axis.truncate() - frame.transform_vector3(DVec3::new(7.0, 8.0, 9.0));
        assert!(without_delp.abs_diff_eq(DVec3::new(35.0, 20.0, 250.0), 1e-10));
        assert!((DMat3::from_mat4(frame).determinant() - 1.0).abs() < 1e-10);
    }

    use std::f64::consts::FRAC_1_SQRT_2;

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

        // The structural 60.30x3.2 catalogue tube is exported by E3D as
        // eight facets, despite the general surface-tolerance rule selecting
        // sixteen at this radius.
        let structural = annulus_rings(&frame, 30.15, None, 0.0);
        assert_eq!(structural[0].len(), 8);

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

    /// GTSTDT 的两条规约：轴向门 0.001，以及「两端法向都指向梁内」。
    ///
    /// 定号弄反的表现不是报错，是端部整块材料留在了外面（或整根梁被切没）——
    /// RVM 对拍时看到的是体积对不上，看不出错在符号。
    #[test]
    fn cut_plane_normals_are_gated_at_0_001_and_point_into_the_beam() {
        let identity = DMat3::IDENTITY;
        // 实测 24381/47163：DRNS=+Z、DRNE=−Z，两端都已指向梁内，原样保留。
        assert_eq!(cut_plane_normal(identity, DVec3::Z, 1.0), DVec3::Z);
        assert_eq!(cut_plane_normal(identity, DVec3::NEG_Z, -1.0), DVec3::NEG_Z);
        // 反着存的同样要被扳回梁内：A 端恒 +Z、B 端恒 −Z。
        assert_eq!(cut_plane_normal(identity, DVec3::NEG_Z, 1.0), DVec3::Z);
        assert_eq!(cut_plane_normal(identity, DVec3::Z, -1.0), DVec3::NEG_Z);
        // |z| < 0.001 → 整条换成 (0,0,1)，再按端别定号；B 端于是成了 −Z。
        let nearly_flat = DVec3::new(1.0, 0.0, 0.0005).normalize();
        assert_eq!(cut_plane_normal(identity, nearly_flat, 1.0), DVec3::Z);
        assert_eq!(cut_plane_normal(identity, nearly_flat, -1.0), DVec3::NEG_Z);
        // 缺 DRNS（读成零向量）按轴向走，不是错。
        assert_eq!(cut_plane_normal(identity, DVec3::ZERO, 1.0), DVec3::Z);
        // 斜切：45° 端向逆旋后保号，x/y 分量留着，才切得动。
        let oblique = DVec3::new(0.0, 1.0, 1.0).normalize();
        let normal = cut_plane_normal(identity, oblique, 1.0);
        assert!(close(normal.z, FRAC_1_SQRT_2) && close(normal.y, FRAC_1_SQRT_2));
        assert!(
            !SectionCutPlane {
                point: DVec3::ZERO,
                normal
            }
            .is_axial()
        );
        // R⁻¹ 真的作用上了：梁轴朝世界 +X 时，世界 +X 的端向在局部就是 +Z。
        let to_local = DMat3::from_cols(DVec3::Y, DVec3::Z, DVec3::X).transpose();
        assert_eq!(cut_plane_normal(to_local, DVec3::X, 1.0), DVec3::Z);
    }

    /// XSPYR：斜切端要先把挤出区间拉到平面之外，再切。
    ///
    /// 只在 `[0, L]` 里切的话，斜面远侧那块本该长出来的材料就没了——09-02 计划正是
    /// 漏了这一步。两端都是轴向面时区间必须一动不动，否则平头梁会平白变长。
    #[test]
    fn xspyr_extends_the_extrusion_past_an_oblique_end_and_leaves_flat_ends_alone() {
        let square = vec![vec![
            [-10.0, -10.0],
            [10.0, -10.0],
            [10.0, 10.0],
            [-10.0, 10.0],
        ]];
        let axial = SectionCuts {
            start: SectionCutPlane {
                point: DVec3::ZERO,
                normal: DVec3::Z,
            },
            end: SectionCutPlane {
                point: DVec3::new(0.0, 0.0, 100.0),
                normal: DVec3::NEG_Z,
            },
        };
        assert_eq!(axial.extrusion_range(&square, 100.0), (0.0, 100.0));
        assert!(axial.start.is_axial() && axial.end.is_axial());

        // 两端各切 45°：A 面法向 (0, −1, 1)/√2 过原点，棱上最深的交点是 y = +10 处的
        // z = −10；B 面法向 (0, −1, −1)/√2 过 z=100，最远的交点是 y = −10 处的 z = 110。
        let sloped = SectionCuts {
            start: SectionCutPlane {
                point: DVec3::ZERO,
                normal: DVec3::new(0.0, -1.0, 1.0).normalize(),
            },
            end: SectionCutPlane {
                point: DVec3::new(0.0, 0.0, 100.0),
                normal: DVec3::new(0.0, -1.0, -1.0).normalize(),
            },
        };
        let (z_min, z_max) = sloped.extrusion_range(&square, 100.0);
        assert!(
            close(z_min, -10.0) && close(z_max, 110.0),
            "{z_min} {z_max}"
        );
    }

    /// 端切的成品体积：20 × 20 的方料、L = 100，两端各按 45° 斜切且斜向相同，
    /// 切出来是个平行六面体——体积仍是 400 × 100，只是两端各挪了位置。
    ///
    /// 这一条同时钉住「先延伸再切」：不延伸的话两端各缺一个 20 × 20 × 10 / 2 的楔子，
    /// 体积会掉 4000（10%）。
    #[test]
    fn a_beam_mitred_the_same_way_at_both_ends_keeps_its_volume() {
        let profile = SectionProfile {
            members: vec![SectionMember {
                source: SectionSource::Srec,
                member_refno: "test/srec".to_string(),
                rings: vec![vec![
                    FilletVertex {
                        x: -10.0,
                        y: -10.0,
                        frad: 0.0,
                    },
                    FilletVertex {
                        x: 10.0,
                        y: -10.0,
                        frad: 0.0,
                    },
                    FilletVertex {
                        x: 10.0,
                        y: 10.0,
                        frad: 0.0,
                    },
                    FilletVertex {
                        x: -10.0,
                        y: 10.0,
                        frad: 0.0,
                    },
                ]],
            }],
            justification: [0.0, 0.0],
            justification_line: DEFAULT_JUSTIFICATION_LINE.to_string(),
            justification_found: true,
            neutral_axis_spin: 0.0,
            level: 0,
            mirrored: false,
            notes: Vec::new(),
        };
        let placement = SectionPlacement {
            local: DMat4::IDENTITY,
            length: 100.0,
            bang_deg: 0.0,
        };
        let cuts = SectionCuts {
            start: SectionCutPlane {
                point: DVec3::ZERO,
                normal: DVec3::new(0.0, -1.0, 1.0).normalize(),
            },
            end: SectionCutPlane {
                point: DVec3::new(0.0, 0.0, 100.0),
                normal: DVec3::new(0.0, 1.0, -1.0).normalize(),
            },
        };
        let mut notes = Vec::new();
        let solid = section_local_solid(&profile, &placement, &cuts, &mut notes)
            .expect("斜切梁应当建得出来")
            .expect("两端 45° 平行斜切不会把梁切空")
            .body;
        assert!(
            (solid.volume() - 40_000.0).abs() < 1e-3,
            "体积 {}，期望 40000（不延伸就切会掉到 36000）",
            solid.volume()
        );
        let bounds = solid.bounding_box().expect("非空体必有包围盒");
        // 斜面沿 +Y 抬 20mm：z 从 −10 一直到 110。
        assert!((bounds.min()[2] + 10.0).abs() < 1e-6, "{:?}", bounds.min());
        assert!((bounds.max()[2] - 110.0).abs() < 1e-6, "{:?}", bounds.max());
        assert!(
            notes.iter().any(|note| note.contains("XSPYR")),
            "延伸要留痕：{notes:?}"
        );
    }

    /// 平头梁一刀都不切：法向 `x = y = 0` 的面 `DB_makeCSG` 直接跳过，成品就是
    /// 一根干净的 `[0, L]` 挤出体，不该多出一层与端盖重合的切面。
    #[test]
    fn a_flat_ended_beam_is_a_plain_extrusion() {
        let profile = SectionProfile {
            members: vec![SectionMember {
                source: SectionSource::Srec,
                member_refno: "test/flat".to_string(),
                rings: vec![vec![
                    FilletVertex {
                        x: 0.0,
                        y: 0.0,
                        frad: 0.0,
                    },
                    FilletVertex {
                        x: 40.0,
                        y: 0.0,
                        frad: 0.0,
                    },
                    FilletVertex {
                        x: 40.0,
                        y: 40.0,
                        frad: 0.0,
                    },
                    FilletVertex {
                        x: 0.0,
                        y: 40.0,
                        frad: 0.0,
                    },
                ]],
            }],
            justification: [0.0, 0.0],
            justification_line: DEFAULT_JUSTIFICATION_LINE.to_string(),
            justification_found: true,
            neutral_axis_spin: 0.0,
            level: 0,
            mirrored: false,
            notes: Vec::new(),
        };
        let placement = SectionPlacement {
            local: DMat4::IDENTITY,
            length: 387.5,
            bang_deg: -90.0,
        };
        let cuts = SectionCuts {
            start: SectionCutPlane {
                point: DVec3::ZERO,
                normal: DVec3::Z,
            },
            end: SectionCutPlane {
                point: DVec3::new(0.0, 0.0, 387.5),
                normal: DVec3::NEG_Z,
            },
        };
        let mut notes = Vec::new();
        let solid = section_local_solid(&profile, &placement, &cuts, &mut notes)
            .unwrap()
            .unwrap()
            .body;
        assert_eq!(solid.num_tri(), 12, "六面盒子 12 个三角形，切面会加面");
        assert!(
            (solid.volume() - 40.0 * 40.0 * 387.5).abs() < 1e-6,
            "{}",
            solid.volume()
        );
        assert!(notes.is_empty(), "平头梁不该有延伸留痕：{notes:?}");
    }

    /// RVM 的记录单位是 GMSS 成员，不是并集后的那一块——所以 `members` 不能在并集里丢掉。
    ///
    /// 管型材（`/CHN/CONCRETE_PROFILES/.../cs035`，设计侧 `DESP=[1000, 900]`）的截面在目录里
    /// 是两个 `PANG=180` 的 SANN 半环。两个半环挤出来贴面相接，一并就焊成一只整圈管，
    /// `decompose()` 再也拆不回两块；而基准 RVM 里它是**两条**面片记录（同心、互转 180°、
    /// 各 20 个多边形 108 顶点，`30101/13283` 等 68 根皆然）。只交并集体出去，这 68 根就
    /// 永远是「E3D 2 条对我们 1 条」。
    #[test]
    fn a_tube_profile_keeps_its_two_half_annuli_after_the_union_welds_them_into_one() {
        let upper = MemberFrame {
            offset: [0.0, 0.0],
            rotation: 0.0,
        };
        let lower = MemberFrame {
            offset: [0.0, 0.0],
            rotation: plax_rotation(plax_angle(&[2, 2]).unwrap()),
        };
        let half = |refno: &str, frame: &MemberFrame| SectionMember {
            source: SectionSource::Sann,
            member_refno: refno.to_string(),
            rings: annulus_rings(frame, 500.0, Some(450.0), 180.0),
        };
        let profile = SectionProfile {
            members: vec![half("test/sann-up", &upper), half("test/sann-down", &lower)],
            justification: [0.0, 0.0],
            justification_line: DEFAULT_JUSTIFICATION_LINE.to_string(),
            justification_found: true,
            neutral_axis_spin: 0.0,
            level: 0,
            mirrored: false,
            notes: Vec::new(),
        };
        let placement = SectionPlacement {
            local: DMat4::IDENTITY,
            length: 1500.0,
            bang_deg: 0.0,
        };
        let cuts = SectionCuts {
            start: SectionCutPlane {
                point: DVec3::ZERO,
                normal: DVec3::Z,
            },
            end: SectionCutPlane {
                point: DVec3::new(0.0, 0.0, 1500.0),
                normal: DVec3::NEG_Z,
            },
        };
        let mut notes = Vec::new();
        let built = section_local_solid(&profile, &placement, &cuts, &mut notes)
            .expect("管型材应当建得出来")
            .expect("平头管不会被端切切空");
        assert_eq!(built.members.len(), 2, "两个 SANN 成员各留一块：{notes:?}");
        assert_eq!(
            built.member_sources,
            vec![SectionSource::Sann, SectionSource::Sann],
            "出身与那几块逐块对齐——RVM 的记录种类照它挑写出器"
        );
        for piece in &built.members {
            assert_eq!(piece.genus(), 0, "半环是个 C 形，没有环柄");
        }
        assert_eq!(built.body.genus(), 1, "两个半环焊成整圈管，并集是亏格 1");
        assert_eq!(
            built.body.decompose().len(),
            1,
            "焊完只剩一块——记录数塌成 1 的由来就在这里"
        );
        let halves: f64 = built.members.iter().map(Manifold::volume).sum();
        assert!(
            (built.body.volume() - halves).abs() < 1e-3,
            "贴面相接不重叠：并集 {} 该等于两半之和 {halves}",
            built.body.volume()
        );
    }

    /// 出身要与那几块**逐块对齐**，混在一起就写错记录种类。
    ///
    /// `/CS036` 空心矩形的四张壁板是 `SREC`（E3D 走 DRPYR），`/CS035` 管型材的两个半环是
    /// `SANN`（走拉伸体的面片）。同一张截面里两种成员并排时，谁是谁只能靠这份对齐表说了算
    /// ——按长相分不开：矩形截面的 `SPRO` 挤出来和 `SREC` 一模一样。
    #[test]
    fn member_sources_line_up_with_the_pieces_they_came_from() {
        let frame = MemberFrame {
            offset: [0.0, 0.0],
            rotation: 0.0,
        };
        let plate = SectionMember {
            source: SectionSource::Srec,
            member_refno: "test/srec".to_string(),
            rings: vec![vec![
                vertex(&frame, -500.0, 900.0, 0.0),
                vertex(&frame, 500.0, 900.0, 0.0),
                vertex(&frame, 500.0, 950.0, 0.0),
                vertex(&frame, -500.0, 950.0, 0.0),
            ]],
        };
        let ring = SectionMember {
            source: SectionSource::Sann,
            member_refno: "test/sann".to_string(),
            rings: annulus_rings(&frame, 500.0, Some(450.0), 360.0),
        };
        let profile = SectionProfile {
            members: vec![plate, ring],
            justification: [0.0, 0.0],
            justification_line: DEFAULT_JUSTIFICATION_LINE.to_string(),
            justification_found: true,
            neutral_axis_spin: 0.0,
            level: 0,
            mirrored: false,
            notes: Vec::new(),
        };
        let placement = SectionPlacement {
            local: DMat4::IDENTITY,
            length: 1500.0,
            bang_deg: 0.0,
        };
        let cuts = SectionCuts {
            start: SectionCutPlane {
                point: DVec3::ZERO,
                normal: DVec3::Z,
            },
            end: SectionCutPlane {
                point: DVec3::new(0.0, 0.0, 1500.0),
                normal: DVec3::NEG_Z,
            },
        };
        let mut notes = Vec::new();
        let built = section_local_solid(&profile, &placement, &cuts, &mut notes)
            .expect("两个成员都建得出来")
            .expect("平头不会被端切切空");
        assert_eq!(built.members.len(), 2, "{notes:?}");
        assert_eq!(
            built.member_sources,
            vec![SectionSource::Srec, SectionSource::Sann]
        );
        // 两块摆在不同高度上：顺序错了这两个包围盒立刻对调。
        let plate_low = built.members[0].bounding_box().unwrap().min()[1];
        let ring_low = built.members[1].bounding_box().unwrap().min()[1];
        assert!(plate_low > 800.0, "板在 y≈900 那一档：{plate_low}");
        assert!(ring_low < -400.0, "环跨到 y≈−500：{ring_low}");
    }

    fn curve(curtyp: &str, pos: [f64; 3], radius: f64) -> SpineNode {
        SpineNode::Curve {
            refno: format!("test/{curtyp}"),
            curtyp: curtyp.to_string(),
            pos: DVec3::from_array(pos),
            radius,
            zdir: DVec3::Z,
        }
    }

    fn point(pos: [f64; 3]) -> SpineNode {
        SpineNode::Point(DVec3::from_array(pos))
    }

    fn arc_midpoint(segment: &SpineSegment) -> DVec3 {
        match segment {
            SpineSegment::Arc {
                start,
                centre,
                axis,
                sweep,
                ..
            } => *centre + rotate_about(*start - *centre, *axis, *sweep * 0.5),
            other => panic!("不是弧：{other:?}"),
        }
    }

    /// `CURTYP FILL`：db 251115 `2013287659/39`——库里存的 `CPOS (−11, −6.716)` 就是倒圆圆心，
    /// 按「切线长 r·tan(φ/2)、圆心在切点内侧 r 处」算出来要逐位对上。
    #[test]
    fn fillet_centre_matches_the_cpos_stored_by_e3d() {
        let nodes = [
            point([-1.0, -12.132, 0.0]),
            curve("FILL", [-1.0, 0.0, 0.0], 10.0),
            point([-42.696, 17.041, 0.0]),
        ];
        let (segments, problems) = spine_segments(&nodes);
        assert!(problems.is_empty(), "{problems:?}");
        assert_eq!(segments.len(), 3, "{segments:?}");
        let SpineSegment::Arc {
            start,
            end,
            centre,
            axis,
            sweep,
        } = &segments[1]
        else {
            panic!("中间那段该是弧：{segments:?}");
        };
        assert!(
            centre.abs_diff_eq(DVec3::new(-11.0, -6.716, 0.0), 2e-3),
            "{centre:?}"
        );
        assert!(
            start.abs_diff_eq(DVec3::new(-1.0, -6.716, 0.0), 2e-3),
            "{start:?}"
        );
        assert!(
            axis.abs_diff_eq(DVec3::Z, 1e-9),
            "左转 → 逆时针轴 +Z：{axis:?}"
        );
        // 转角 = 两边夹角：atan2(41.696, 17.041) 的余角… 直接对切点到圆心的距离。
        assert!(((*end - *centre).length() - 10.0).abs() < 1e-9);
        assert!(*sweep > 0.0 && *sweep < PI);
        // 隐含直段：前一截从 POINSP 到切点。
        assert!(
            matches!(&segments[0], SpineSegment::Straight { start: s, end: e } if s.abs_diff_eq(DVec3::new(-1.0, -12.132, 0.0), 1e-9) && e.abs_diff_eq(*start, 1e-9))
        );
        assert!(
            matches!(&segments[2], SpineSegment::Straight { start: s, .. } if s.abs_diff_eq(*end, 1e-9))
        );
    }

    /// `CURTYP CENT`：db 251183 `2013287727/138`——两个 POINSP 都在圆外，切入/切出点由
    /// 「绕 ZDIR 逆时针」唯一确定；库里存的 `CPOS (−11, 23.5)` / `(−11, −23.5)` 正是
    /// 两段弧各自的中点。
    #[test]
    fn centre_arcs_enter_and_leave_on_tangents_and_their_midpoints_match_cpos() {
        let nodes = [
            point([0.0, 0.0, 0.0]),
            curve("CENT", [-11.0, 12.5, 0.0], 11.0),
            point([-22.0, 0.0, 0.0]),
            curve("CENT", [-11.0, -12.5, 0.0], 11.0),
            point([0.0, 0.0, 0.0]),
        ];
        let (segments, problems) = spine_segments(&nodes);
        assert!(problems.is_empty(), "{problems:?}");
        let arcs: Vec<&SpineSegment> = segments
            .iter()
            .filter(|segment| matches!(segment, SpineSegment::Arc { .. }))
            .collect();
        assert_eq!(arcs.len(), 2, "{segments:?}");
        let first_mid = arc_midpoint(arcs[0]);
        let second_mid = arc_midpoint(arcs[1]);
        assert!(
            first_mid.abs_diff_eq(DVec3::new(-11.0, 23.5, 0.0), 1e-6),
            "{first_mid:?}"
        );
        assert!(
            second_mid.abs_diff_eq(DVec3::new(-11.0, -23.5, 0.0), 1e-6),
            "{second_mid:?}"
        );
        // 切入点 (0, 12.5)：从 (0,0) 竖直向上切入，先走一截 12.5 的隐含直段。
        assert!(
            matches!(&segments[0], SpineSegment::Straight { start, end } if start.abs_diff_eq(DVec3::ZERO, 1e-9) && end.abs_diff_eq(DVec3::new(0.0, 12.5, 0.0), 1e-6))
        );
        let total: f64 = segments.iter().map(SpineSegment::length).sum();
        // 两段 180° 弧 + 四截 12.5 的直段。
        assert!(
            (total - (2.0 * PI * 11.0 + 4.0 * 12.5)).abs() < 1e-6,
            "{total}"
        );
    }

    /// `CURTYP CENT` 的另一形：前点在圆上、后点在圆外（db 251115 `2013287659/43` 的卡子）——
    /// 半圆从 0° 逆时针到 180°，再一截 4.88 的隐含直段竖直落到后点。
    #[test]
    fn centre_arc_with_start_on_circle_sweeps_half_a_turn_to_the_tangent_point() {
        let nodes = [
            point([-87.5, 50.352, 0.0]),
            curve("CENT", [-102.5, 50.352, 0.0], 15.0),
            point([-117.5, 45.47, 0.0]),
        ];
        let (segments, problems) = spine_segments(&nodes);
        assert!(problems.is_empty(), "{problems:?}");
        assert_eq!(segments.len(), 2, "{segments:?}");
        let SpineSegment::Arc { end, sweep, .. } = &segments[0] else {
            panic!("{segments:?}");
        };
        assert!((sweep - PI).abs() < 1e-6, "{sweep}");
        assert!(
            end.abs_diff_eq(DVec3::new(-117.5, 50.352, 0.0), 1e-6),
            "{end:?}"
        );
        assert!(arc_midpoint(&segments[0]).abs_diff_eq(DVec3::new(-102.5, 65.352, 0.0), 1e-6));
        assert!(
            matches!(&segments[1], SpineSegment::Straight { end: e, .. } if e.abs_diff_eq(DVec3::new(-117.5, 45.47, 0.0), 1e-9))
        );
    }

    /// `CURTYP THRU`：db 251183 `2013287727/2315`——过点弧，圆心在原点、半径 31、扫 225°，
    /// 方向由过点决定（这里是逆时针）。
    #[test]
    fn thru_arc_passes_through_pos_and_takes_the_long_way_when_pos_says_so() {
        let nodes = [
            point([31.0, 0.0, 0.0]),
            curve("THRU", [-11.863, 28.64, 0.0], 0.0),
            point([-21.92, -21.92, 0.0]),
        ];
        let (segments, problems) = spine_segments(&nodes);
        assert!(problems.is_empty(), "{problems:?}");
        assert_eq!(segments.len(), 1);
        let SpineSegment::Arc {
            centre,
            axis,
            sweep,
            ..
        } = &segments[0]
        else {
            panic!("{segments:?}");
        };
        assert!(centre.abs_diff_eq(DVec3::ZERO, 2e-3), "{centre:?}");
        assert!(axis.abs_diff_eq(DVec3::Z, 1e-9), "{axis:?}");
        assert!(
            (sweep.to_degrees() - 225.0).abs() < 0.05,
            "{}",
            sweep.to_degrees()
        );
    }

    /// 截面 Y 向沿路径的传递：一根 Ø16 圆杆沿「上行 → 90° 倒圆 → 左行」的 L 形路径扫掠
    /// （db 251183 `2013287727/67`），包围盒要正好是路径外扩一个半径、两端平头。
    #[test]
    fn sweeping_a_round_bar_along_a_filleted_l_path_hugs_the_path_by_one_radius() {
        let nodes = [
            point([0.0, 50.0, 0.0]),
            curve("FILL", [0.0, 212.0, 0.0], 50.0),
            point([-253.0, 212.0, 0.0]),
        ];
        let (segments, problems) = spine_segments(&nodes);
        assert!(problems.is_empty(), "{problems:?}");
        let path = SpinePath {
            segments,
            ydir: DVec3::Z,
            drns: DVec3::ZERO,
            drne: DVec3::ZERO,
            curve_types: vec!["FILL".to_string()],
            problems: Vec::new(),
        };
        let ring: Vec<FilletVertex> = (0..48)
            .map(|i| {
                let a = 2.0 * PI * f64::from(i) / 48.0;
                FilletVertex {
                    x: 8.0 * a.cos(),
                    y: 8.0 * a.sin(),
                    frad: 0.0,
                }
            })
            .collect();
        let profile = SectionProfile {
            members: vec![SectionMember {
                source: SectionSource::Spro { vertices: 48 },
                member_refno: "test/bar".to_string(),
                rings: vec![ring],
            }],
            justification: [0.0, 0.0],
            justification_line: DEFAULT_JUSTIFICATION_LINE.to_string(),
            justification_found: true,
            neutral_axis_spin: 0.0,
            level: 0,
            mirrored: false,
            notes: Vec::new(),
        };
        let mut notes = Vec::new();
        let solid = spine_swept_solid(&profile, &path, 0.0, &mut notes)
            .unwrap()
            .expect("该有实体");
        let bbox = solid.bounding_box().expect("非空实体有包围盒");
        let (min, max) = (bbox.min(), bbox.max());
        assert!((min[0] - -253.0).abs() < 1e-6, "{min:?}");
        assert!((max[0] - 8.0).abs() < 1e-3, "{max:?}");
        assert!((min[1] - 50.0).abs() < 1e-6, "{min:?}");
        assert!((max[1] - 220.0).abs() < 1e-3, "{max:?}");
        assert!(
            (min[2] - -8.0).abs() < 1e-3 && (max[2] - 8.0).abs() < 1e-3,
            "{min:?} {max:?}"
        );
        // FacetGroup 回转用 E3D 的离散环带，端部/弧段的三角化会让体积
        // 比连续扫掠的截面积 × 中心线长略低；保留一个小的离散化余量。
        let length: f64 = path.segments.iter().map(SpineSegment::length).sum();
        let expected = PI * 64.0 * length;
        assert!(
            (solid.volume() - expected).abs() / expected < 0.03,
            "体积 {} vs {expected}",
            solid.volume()
        );
    }

    /// GENSEC 两端按 SPINE `DRNS` / `DRNE` 斜接：沿 +X 的 1000 mm 直段、100×100 方截面，两端端向
    /// 都是 XY 面里偏 X 30°。截面 x 轴落在世界 +Y 上，两张斜面在截面远侧各落到原路径之外
    /// 50·tan30°，要先沿切向补挤再切——成品是一根平行四边形柱：x 伸到 [−28.868, 1028.868]，
    /// 体积与平头同为 100×100×1000。端向与切向垂直（含着轴）那端照旧平头。
    #[test]
    fn spine_swept_solid_mitres_both_ends_on_drns_and_drne() {
        let square = || SectionProfile {
            members: vec![SectionMember {
                source: SectionSource::Spro { vertices: 4 },
                member_refno: "test/square".to_string(),
                rings: vec![
                    [[-50.0, -50.0], [50.0, -50.0], [50.0, 50.0], [-50.0, 50.0]]
                        .map(|[x, y]| FilletVertex { x, y, frad: 0.0 })
                        .to_vec(),
                ],
            }],
            justification: [0.0, 0.0],
            justification_line: DEFAULT_JUSTIFICATION_LINE.to_string(),
            justification_found: true,
            neutral_axis_spin: 0.0,
            level: 0,
            mirrored: false,
            notes: Vec::new(),
        };
        let path = |drns: DVec3, drne: DVec3| {
            let (segments, problems) =
                spine_segments(&[point([0.0, 0.0, 0.0]), point([1000.0, 0.0, 0.0])]);
            assert!(problems.is_empty(), "{problems:?}");
            SpinePath {
                segments,
                ydir: DVec3::Z,
                drns,
                drne,
                curve_types: Vec::new(),
                problems: Vec::new(),
            }
        };
        let slant = DVec3::new(30f64.to_radians().cos(), 30f64.to_radians().sin(), 0.0);
        let overhang = 50.0 * 30f64.to_radians().tan();

        let mut notes = Vec::new();
        let solid = spine_swept_solid(&square(), &path(slant, slant), 0.0, &mut notes)
            .unwrap()
            .expect("该有实体");
        let bbox = solid.bounding_box().expect("非空实体有包围盒");
        let (min, max) = (bbox.min(), bbox.max());
        assert!((min[0] + overhang).abs() < 1e-4, "{min:?} {notes:?}");
        assert!(
            (max[0] - (1000.0 + overhang)).abs() < 1e-4,
            "{max:?} {notes:?}"
        );
        assert!((min[1] + 50.0).abs() < 1e-6 && (max[1] - 50.0).abs() < 1e-6);
        assert!((min[2] + 50.0).abs() < 1e-6 && (max[2] - 50.0).abs() < 1e-6);
        assert!(
            (solid.volume() - 1.0e7).abs() < 10.0,
            "体积 {}",
            solid.volume()
        );
        assert_eq!(
            notes.iter().filter(|note| note.contains("斜切")).count(),
            2,
            "{notes:?}"
        );

        let mut notes = Vec::new();
        let solid = spine_swept_solid(&square(), &path(DVec3::Y, slant), 0.0, &mut notes)
            .unwrap()
            .expect("该有实体");
        let bbox = solid.bounding_box().expect("非空实体有包围盒");
        assert!(bbox.min()[0].abs() < 1e-6, "{:?} {notes:?}", bbox.min());
        assert!(
            notes.iter().any(|note| note.contains("垂直或平行")),
            "{notes:?}"
        );
    }
}

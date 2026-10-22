//! 把 `gen_ams` 的生成物与一份 **E3D TTY 导出的 RVM 快照**逐件对拍。
//!
//! ```text
//! rvm_compare --baseline <*.rvm.json> --elements <out/.../elements.json> \
//!             [--facets <rvm_facets_dump 的 json>] [--out <compare.json>] [--noun GWALL]
//! ```
//!
//! ## 基准 AABB 必须来自面片顶点（`--facets`），不能信快照字段
//!
//! 2026-08-31 实测（20 件 GWALL 三方对账）：`.rvm` 的 PRIM 记录自带的 bbox 与**同一条
//! 记录里的面片顶点**对不上（最大差 5.4m），rvm-rs 又把它当局部盒再乘了一次几何
//! transform（`parser/rvm.rs`：`geo.bbox_world = bbox.transform(&transform)`），快照的
//! `aabb_world_mm` 原样收编了这个值。拿它当判据，20 件顶点级全等的墙全部误判 `off`。
//! `--facets` 给的是 `rvm_facets_dump` 从面片顶点实算的世界坐标——有它就用它，
//! 没有的成员退回快照字段并在 `aabb_source` 里写明，退回件数进报告。
//!
//! ## 主键
//!
//! RVM 里没有 refno。E3D 给无名元素的显示名是「NOUN n of <属主显示名>」，快照
//! 因此带 `owner_desc` + `ordinal`；`gen_ams` 用同一口径发同一对键（见
//! `gen_ams::owner_keys`）。对拍主键 = **属主名 + noun 前 4 字 + 序号**：
//!
//! - 只取属主显示名里的 `/名` 那一截。快照把 noun 写成全称（`CMPFITTING`），
//!   e3d-io 给的是库里的短名（`CMPF`），拿属主 noun 当键会整族错配。
//! - noun 取前 4 字：快照里是截断过的（`GWALL` → `GWAL`）。
//! - **管身（`TUBI`）例外**：E3D 的 `TUBE n` 在一部分支管上不是画出来的顺序（ams8000
//!   `/M-OR-1R323-A` 画的两根叫 TUBE 2 / 5），所以同一属主下的 TUBI 按世界 AABB 中心最近
//!   配对（[`pair_tubes_by_centre`]），序号只用来分组与显示；两侧序号不同时 `pairs[].key`
//!   写成 `TUBI 2 /X ↔ TUBI 1 /X`。
//! - **写在容器那一组里的管身**：不开「隐式管导成单独容器」的导出（APS 普查）里 E3D 把隐式管
//!   直接写在容器自己那一组里，组名就是容器名；我们同一容器名下的 `TUBI n` 并成一件按容器名
//!   配对（[`merge_container_tubes`]，RVM ↔ RVM 才并）。
//!
//! ## 判据
//!
//! 有 facet 和生成侧 `world/local_aabb_*` 时，把 facet 顶点乘 `world^-1`，
//! **在元素局部系比较 AABB**。只有旧版 elements.json 或缺 facet 时才退回世界 AABB；
//! 旋转墙的世界 X/Y AABB 不能代表墙自身的端头/厚度方向。
//!
//! | 档 | 阈值 | 含义 |
//! |---|---|---|
//! | `exact` | ≤ [`EXACT_MM`] | 只差 RVM 的 f32 舍入 |
//! | `chord` | ≤ 该 noun 的弦高容差 | 基本体 0.5mm；挤出轮廓 10mm |
//! | `reference_artifact` | 总 AABB 超差，显著连通分量≤弦高容差 | Core/libgm 参考中有断开且窄于一个弦高的薄片；原始差仍写入报告 |
//! | `film` | 多出来的邨分最薄处 ≤ [`FILM_MM`]，去掉它两侧在弦高内一致 | 连在主体上的零厚残膜 / 发丝，两侧都可能有；原始差仍写入报告 |
//! | `off` | 更大 | 要查 |
//!
//! ## 贴体薄膜（`film`）
//!
//! `reference_artifact` 只认**断开**的薄片。负体切剩的膜与发丝连在主体上，不是独立分量：
//! ams1112 `/1RS-WF03-W-C-RR001` GWALL 13 里 E3D 在两把相邻 NXTR 之间留下 0.01 mm 宽、
//! 26 mm 高的膜；ams7997 `/1LR-WF05-S-C` FLOOR 6 是一片 3290 mm 长、最厚 0.011 mm 的楔形膜；
//! `/1RX-FN01-F001` FLOOR 12 反过来是我们切剩一根 100 mm 长的零厚发丝。包围盒被它们拉长，
//! 按盒差都是 `off`。
//!
//! 判法（只有 RVM ↔ RVM、局部帧那条路，两侧都有面片轮廓）：对每一侧，取落在对侧局部
//! AABB 加一个弦高之外的面片顶点，连同面片边穿过那只盒面的交点——这是「多出来的那一块」
//! 的外形点（只拿盒外顶点不行：真的长出一截时，盒外顶点只是端面那几个，共面，厚度
//! 也是 0）。量它们在三个主方向（协方差特征向量）上的最小跨度：膜是膜厚，发丝趋近 0，
//! 真缺一块或多一块的件有实际厚度。每一侧多出来的部分都不超过 [`FILM_MM`]、且去掉它们以后
//! （只留对侧盒内的顶点）两侧 AABB 在弦高内一致，才记 `film`。膜厚与膜在哪一侧写进
//! `pairs[]`。
//!
//! 为什么不比体积：RVM 快照只存 primitive 的包围盒与拓扑计数，没有体积；
//! 要比体积得把 `.rvm` 的三角面自己读出来算，那是下一步（本工具留了
//! `bbox_degenerate` / 几何计数两条旁证先兜着）。
//!
//! ## 不判的东西也要说出来
//!
//! 快照是 `export_scope=narrow` 的一棵子树，里面既有我们建的 GWALL，也有整片
//! 目录件（SBFITTING/FIXING/PLDATUM）。工具按 noun 分组报「两边都有 / 只在基准 /
//! 只在生成」，**只对两边都有的判 AABB**——把只在基准那一侧的当成失败，报告就成了
//! 一张恒红的表，没人会再看第二眼。
//!
//! ## 只有中心线的成员不进对拍
//!
//! RVM 的 `Line`（kind 10）不是实体，**它的包围盒是导出器现编的**：
//! `expdri/DRWLIN`（`expdri.dll` `0x10002B92`）拿两个端点算出长度 `L`，写下的盒子是
//! 写死的 `[-5, -5, -L/2, 5, 5, L/2]`，两个参数是字面量 `5.0` 与 `L`。那个 ±5 mm
//! 是这段代码里的常数，不是模型里的任何尺寸。
//!
//! 另外两条旁证：导出分派 `expdri/EXPFPR` 里每种实体图元都有 `n` 与 `n+0x32` 两个
//! 入口（第二表现），只有 `0xB`（线）是光杆一个；libgm 里 `GM_Line` 是条棱
//! （`point0`/`point1`/`getPEdge`/`getEdgeType`），是这一族里唯一**没有
//! `calcFacets`** 的类——它从不产生面。
//!
//! 所以拿它判 AABB 就是拿导出器的常数当判据。[`e3d_model::rvm_read`] 早就这么定了
//! （`Primitive::Line` 没有采样点、`world_aabb_mm()` 给 `None`、帧记
//! `BboxFrame::NotApplicable`），于是 RVM ↔ RVM 那条路**两侧都已经**把它们丢掉了；
//! 这里让快照那条路跟上，口径只此一份。
//!
//! **这不是把欠款抹掉。** 「E3D 画了一条线、我们一条都还不画」记在
//! `Report::centreline_only` 与 `evidence_pending` 上（SLYK `/石楼装卸车-SUPP1`
//! 的 17 根无剖面 SCTN 就在那本账里），那本账才是它的归属；本工具报
//! `centreline_only_baseline`，让它在对拍报告里也点得出名字。

use std::collections::BTreeMap;
use std::path::PathBuf;

use anyhow::{Context, bail};
use serde::{Deserialize, Serialize};

/// 只差 RVM f32 舍入的量级（mm）。坐标量级 ~2e4，f32 相对精度 ~1e-7。
const EXACT_MM: f64 = 0.05;

/// `film` 那一档允许的最大膜厚（mm）。实测的膜与发丝都在 0.011 mm 以下。
const FILM_MM: f64 = 0.05;

fn chord_tolerance_mm(noun: &str) -> f64 {
    match noun4(noun).as_str() {
        "PANE" | "FLOO" | "GWAL" => e3d_model::discretise::PROFILE_CHORD_TOL_MM,
        _ => e3d_model::discretise::CHORD_TOL_MM,
    }
}

#[derive(Clone, Deserialize)]
struct Snapshot {
    meta: SnapshotMeta,
    members: Vec<SnapshotMember>,
}

#[derive(Clone, Deserialize)]
struct SnapshotMeta {
    dbnum: Option<u32>,
    root_name: Option<String>,
    export_scope: Option<String>,
    member_count: Option<usize>,
    geometry_count: Option<usize>,
}

#[derive(Clone, Deserialize)]
struct SnapshotMember {
    name: Option<String>,
    noun: Option<String>,
    owner_desc: Option<String>,
    ordinal: Option<usize>,
    aabb_world_mm: Option<[f64; 6]>,
    #[serde(default)]
    geometries: Vec<serde_json::Value>,
}

/// `rvm_facets_dump` 产物里本工具要吃的那几格。
#[derive(Deserialize)]
struct FacetGroupDump {
    name: String,
    geometries: Vec<FacetGeometryDump>,
}

#[derive(Deserialize)]
struct FacetGeometryDump {
    polygons_world_mm: Option<Vec<Vec<Vec<[f32; 3]>>>>,
}

#[derive(Clone)]
struct FacetData {
    vertices: Vec<[f64; 3]>,
    components: Vec<Vec<[f64; 3]>>,
}

#[derive(Clone, Deserialize)]
struct GeneratedComponent {
    local_aabb_min: [f64; 3],
    local_aabb_max: [f64; 3],
}

#[derive(Clone, Deserialize)]
struct Generated {
    refno: String,
    noun: String,
    name: Option<String>,
    owner_desc: Option<String>,
    ordinal: usize,
    volume_mm3: f64,
    aabb_min: [f64; 3],
    aabb_max: [f64; 3],
    #[serde(default)]
    local_aabb_min: Option<[f64; 3]>,
    #[serde(default)]
    local_aabb_max: Option<[f64; 3]>,
    /// 局部系到世界系，列主序 `[c0,c1,c2,t]`。
    #[serde(default)]
    world: Option<[[f64; 3]; 4]>,
    #[serde(default)]
    components: Vec<GeneratedComponent>,
    /// 本元素写出的 RVM 图元种类，按记录顺序。只有 RVM ↔ RVM 这条路填得出来
    /// （`elements.json` 里没有这个概念），走老路时是空的，kind 一致率随之标为「不适用」。
    #[serde(default)]
    kinds: Vec<String>,
    genus: i32,
    negatives_applied: usize,
    negatives_skipped: usize,
}

/// RVM 没 refno：具名元素直接按 E3D 名称；无名元素按属主+noun+序号。
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
enum Key {
    Name {
        name: String,
        noun: String,
    },
    OwnerOrdinal {
        owner: String,
        noun: String,
        ordinal: usize,
    },
}

impl Key {
    fn noun(&self) -> &str {
        match self {
            Self::Name { noun, .. } | Self::OwnerOrdinal { noun, .. } => noun,
        }
    }

    fn display(&self) -> String {
        match self {
            Self::Name { name, noun } => format!("{noun} {name}"),
            Self::OwnerOrdinal {
                owner,
                noun,
                ordinal,
            } => format!("{noun} {ordinal} {owner}"),
        }
    }
}

/// 属主显示名归一化为稳定键。
///
/// 命名属主只保留 `/名`；无名属主必须保留每一层 noun + ordinal，否则
/// `SUBEQUIPMENT 1..20 of EQUIPMENT /X` 会全部塌成 `/X`。全名词与短名词统一
/// 取前四字，使 RVM 的 `SUBEQUIPMENT` 与 e3d-io 的 `SUBE` 可以相配。
fn owner_key(desc: &str) -> String {
    let desc = desc.trim();
    if let Some((left, parent)) = desc.split_once(" of ")
        && let Some((noun, ordinal)) = left.rsplit_once(' ')
        && ordinal.parse::<usize>().is_ok()
    {
        return format!("{} {ordinal} of {}", noun4(noun), owner_key(parent));
    }
    match desc.rsplit_once(' ') {
        Some((noun, name)) if !noun.contains(' ') && name.starts_with('/') => name.to_string(),
        _ => desc.to_string(),
    }
}

/// 这一条几何是不是中心线（RVM `Line`，kind 10）。见模块文档「只有中心线的成员不进对拍」。
///
/// 两种快照形态都要认：`rvm_verify import` 写的是对象（`{"kind":"Line",…}`），
/// [`load_rvm_side`] 写的是光一个 kind 名。少认一种，同一份 RVM 走两条路会判出两个结果。
fn is_centreline(geometry: &serde_json::Value) -> bool {
    let kind = match geometry {
        serde_json::Value::String(kind) => Some(kind.as_str()),
        other => other.get("kind").and_then(serde_json::Value::as_str),
    };
    kind == Some("Line")
}

/// 取前四字是为了让 RVM 的全名词（`SUBEQUIPMENT`）配上 e3d-io 的短名词（`SUBE`）。
/// 用户自定义元素类型（`:` 开头）没有短名词，两侧写的是同一串，截了反而撞车：
/// `/石楼装卸车-SUPP1` 里 `:SDS_Pole` / `:SDS_Plate` / `:SDS_Entity` 全塌成 `:SDS`，
/// 同一属主同一序号下三件挤成一个键，整个 ZONE 判不下去。
fn noun4(noun: &str) -> String {
    if noun.starts_with(':') {
        return noun.to_string();
    }
    noun.chars().take(4).collect()
}

#[derive(Serialize)]
struct Pair {
    key: String,
    refno: String,
    noun: String,
    /// 逐轴 |生成 − 基准| 的最大值（mm）。
    max_axis_delta_mm: f64,
    /// 参考与生成的完整包围盒最大差；该值永远不被薄片过滤覆盖。
    raw_max_axis_delta_mm: f64,
    /// `reference_artifact` 与 `film` 才写：丢弃断开薄片或贴体薄膜之后的差。
    significant_max_axis_delta_mm: Option<f64>,
    /// 分档：exact / chord / reference_artifact / film / off。
    verdict: &'static str,
    /// 只有 `film` 才写：多出来那部分在主方向上的最小跨度（mm）与它在哪一侧
    /// （`baseline` / `generated` / `both`）。
    film_thickness_mm: Option<f64>,
    film_side: Option<&'static str>,
    /// 基准 AABB 的来源：`facets`（顶点实算，可信）/ `snapshot`（PRIM bbox 字段，见模块文档）。
    aabb_source: &'static str,
    /// `local` 为可信判据；`world` 是缺局部帧时的兼容回退。
    comparison_frame: &'static str,
    /// 两侧写出的 RVM 图元种类是否同一套（多重集比较，顺序无关）。
    /// 只有 RVM ↔ RVM 有值；这是「我们的几何分解是否与 E3D 同构」的度量，比 AABB 严。
    kind_match: Option<bool>,
    baseline_kinds: Vec<String>,
    generated_kinds: Vec<String>,
    delta_min_mm: [f64; 3],
    delta_max_mm: [f64; 3],
    baseline_aabb: [f64; 6],
    generated_aabb: [f64; 6],
    significant_baseline_aabb: Option<[f64; 6]>,
    significant_generated_aabb: Option<[f64; 6]>,
    volume_mm3: f64,
    genus: i32,
    negatives_applied: usize,
    negatives_skipped: usize,
}

#[derive(Serialize, Default)]
struct NounTally {
    both: usize,
    only_baseline: usize,
    only_generated: usize,
    exact: usize,
    chord: usize,
    reference_artifact: usize,
    film: usize,
    off: usize,
    /// 两侧图元种类同一套 / 不同。只有 RVM ↔ RVM 填得出来，见 [`Pair::kind_match`]。
    kind_match: usize,
    kind_differ: usize,
}

#[derive(Serialize)]
struct CompareReport {
    baseline: String,
    elements: String,
    facets: Option<String>,
    /// 基准 AABB 用了面片实算的件数 / 退回快照字段的件数。
    aabb_from_facets: usize,
    aabb_from_snapshot: usize,
    dbnum: Option<u32>,
    root_name: Option<String>,
    export_scope: Option<String>,
    baseline_members: Option<usize>,
    baseline_geometries: Option<usize>,
    /// 属主无名、发不出主键的生成元素——这些既没判也没配上，必须留名。
    unkeyable_generated: Vec<String>,
    /// 基准里带几何但发不出主键的成员，同理。
    unkeyable_baseline: Vec<String>,
    /// 基准里只有 `Line` 的成员：E3D 画了中心线、没画实体，不进判据（见模块文档）。
    /// 留名而不是静默丢掉——这是一笔欠款，不是一件不存在的东西。
    centreline_only_baseline: Vec<String>,
    by_noun: BTreeMap<String, NounTally>,
    pairs: Vec<Pair>,
    only_baseline: Vec<String>,
    only_generated: Vec<String>,
    /// 并成一件去同名配对的路由容器数：E3D 把隐式管写在容器自己那一组里，见
    /// [`merge_container_tubes`]。
    container_tube_groups: usize,
}

/// 基准侧一条待配记录：成员本身、面片（缺面片的走 snapshot AABB）、世界 AABB、AABB 来源标记。
///
/// 面片这一格**借**着 `facet_vertices` 里的那一份，不复制：解析图元的「面片」是我们按面
/// 采出来的点云，一只 90° 弯头就有 2.3 万点，`/Copy-of-装卸车PI` 那种管线根整份 6800 万点
/// （≈1.7 GB）。多留一份就多 1.7 GB，debug 下直接顶穿提交上限。
type BaseEntry<'a> = (
    &'a SnapshotMember,
    Option<&'a FacetData>,
    [f64; 6],
    &'static str,
);

/// `FTUBE 1 of BRANCH /C-IY-1R330-B` → `("FTUBE", 1, "BRANCH /C-IY-1R330-B")`。
/// 命名元素（`/C-IY-1R330-B`）没有这个形态，返回 None。
fn parse_default_display_name(name: &str) -> Option<(String, usize, String)> {
    let (head, owner) = name.trim().split_once(" of ")?;
    let (noun, ordinal) = head.rsplit_once(' ')?;
    Some((
        normalise_rvm_noun(noun.trim()),
        ordinal.trim().parse().ok()?,
        owner.trim().to_string(),
    ))
}

/// E3D 把隐含管身导成 `TUBE n of BRANCH /X`，库里的 noun 是 `TUBI`。
/// 两侧 `noun4()` 之后是 `TUBE` vs `TUBI`，**配不上**，而且 `pair_tubes_by_centre`
/// 只对 `noun == "TUBI"` 生效，于是整族管身两边各成孤儿。
/// `rvm_verify import` 早就在做这个归一（快照里记的是 `TUBI`），
/// RVM ↔ RVM 这条路自己读文件，就得自己归一。
///
/// 2026-09-04 的 21 条 BRAN 批量对拍里，`26229-Copy-of-1RCV0101` 的 4 根管身
/// 就是这么漏掉的（旧路径 `TUBI 1..4` 有，新路径没有）。
fn normalise_rvm_noun(noun: &str) -> String {
    match noun {
        "TUBE" => "TUBI".to_string(),
        other => other.to_string(),
    }
}

/// 一份 `.rvm` 摊平后的三样东西：基准形态的成员表、面片索引、生成形态的元素表。
/// 同一份文件既能当基准也能当生成侧，所以三样一起给。
type RvmSide = (
    Vec<SnapshotMember>,
    BTreeMap<String, FacetData>,
    Vec<Generated>,
);

/// 一份 `.rvm` 摊成本工具原有的两种形态，好让配对与判档一行不改地跑在上面。
///
/// 这是「RVM ↔ RVM 对称对拍」的全部实现：两侧都走 [`e3d_model::rvm_read`]，
/// 于是「基准 AABB 怎么算」只剩一处代码，也不再需要 `--facets` 这条旁路
/// ——面片顶点本来就在文件里。
///
/// `want_facets` 关掉时不建面片索引。**生成侧一定要关**：判档只从生成侧取
/// `Generated`（世界 / 局部 AABB 与分量都已经在里面了），那份索引一行都读不到，
/// 却要为每只弯头留 2.3 万个采样点——`/Copy-of-装卸车PI` 白占 1.7 GB。
fn load_rvm_side(path: &std::path::Path, want_facets: bool) -> anyhow::Result<RvmSide> {
    let file = e3d_model::rvm_read::RvmFile::read(path)?;
    let unknown = file.bbox_frame_tally().get("unknown").copied().unwrap_or(0);
    if unknown > 0 {
        bail!(
            "{} 里有 {unknown} 条记录的 bbox 帧认不出来——先查 rvm_read，别拿它当判据",
            path.display()
        );
    }

    let mut members = Vec::new();
    let mut facets = BTreeMap::new();
    let mut generated = Vec::new();

    for member in &file.members {
        let Some(world_aabb) = member.world_aabb_mm() else {
            continue; // 只有中心线或没有几何的容器，不参与判定。
        };
        let default_name = parse_default_display_name(&member.name);
        let (noun, owner_desc, ordinal, named) = match &default_name {
            Some((noun, ordinal, owner)) => (noun.clone(), Some(owner.clone()), *ordinal, None),
            // 命名元素：noun 从 RVM 里读不出来，只能靠同名配对（Key::Name）。
            None => (String::new(), None, 0, Some(member.name.clone())),
        };

        members.push(SnapshotMember {
            name: Some(member.name.clone()),
            noun: (!noun.is_empty()).then(|| noun.clone()),
            owner_desc: owner_desc.clone(),
            ordinal: default_name.as_ref().map(|d| d.1),
            aabb_world_mm: Some(world_aabb),
            // 存 kind 名而不是占位 null：基准侧的 kind 一致率要拿它算。
            geometries: member
                .geometries
                .iter()
                .map(|g| serde_json::Value::String(g.primitive.name().to_string()))
                .collect(),
        });

        // 局部帧取第一条几何的（我们自己写出的 RVM 每个成员就一条；E3D 的多条共用元素帧）。
        let first = &member.geometries[0];
        let rotation = first.rotation();
        let world = [
            [rotation[0], rotation[1], rotation[2]],
            [rotation[3], rotation[4], rotation[5]],
            [rotation[6], rotation[7], rotation[8]],
            first.translation_mm,
        ];
        // 解析图元没有真顶点，用规范盒的八个角顶上：否则同一份 RVM 里 Box 成员退回
        // 世界帧、FacetGroup 成员走局部帧，两类成员的判据就不是一套了。
        let world_vertices = member.world_hull_points_mm();
        let inverse = world_matrix(world).inverse();
        let local = world_vertices.iter().map(|vertex| {
            (inverse * glam::DVec3::from_array(*vertex).extend(1.0))
                .truncate()
                .to_array()
        });
        let (local_min, local_max) = match aabb6(local) {
            Some(a) => ([a[0], a[1], a[2]], [a[3], a[4], a[5]]),
            None => ([0.0; 3], [0.0; 3]),
        };

        // reference_artifact 那一档两侧都按连通分量滤掉窄于弦高的断开薄片
        // （[`significant_components_aabb`]）。基准侧连顶点一起留进面片索引；生成侧只留每个分量的
        // 局部 AABB，顶点当场丢掉。
        let parts = if world_vertices.is_empty() {
            Vec::new()
        } else {
            let polygons = member
                .geometries
                .iter()
                .flat_map(|geometry| match &geometry.primitive {
                    e3d_model::rvm_read::Primitive::FacetGroup(polygons) => polygons
                        .iter()
                        .map(|polygon| {
                            polygon
                                .contours
                                .iter()
                                .flat_map(|c| c.vertices.iter())
                                .map(|v| geometry.to_world_mm(*v))
                                .collect::<Vec<_>>()
                        })
                        .collect::<Vec<_>>(),
                    // 解析图元当成一个整块，连通分量分析（reference_artifact 判档）拿它当一片。
                    _ => match geometry.world_hull_points_mm() {
                        points if points.is_empty() => Vec::new(),
                        points => vec![points],
                    },
                })
                .collect::<Vec<_>>();
            connected_polygon_components(&polygons)
        };
        let components = parts
            .iter()
            .filter_map(|part| {
                aabb6(part.iter().map(|vertex| {
                    (inverse * glam::DVec3::from_array(*vertex).extend(1.0))
                        .truncate()
                        .to_array()
                }))
            })
            .map(|a| GeneratedComponent {
                local_aabb_min: [a[0], a[1], a[2]],
                local_aabb_max: [a[3], a[4], a[5]],
            })
            .collect();
        if want_facets && !world_vertices.is_empty() {
            facets.insert(
                member.name.trim().to_string(),
                FacetData {
                    components: parts,
                    vertices: world_vertices,
                },
            );
        }

        generated.push(Generated {
            refno: member.path.clone(),
            noun,
            name: named,
            owner_desc,
            ordinal,
            // RVM 里没有体积 / 亏格 / 负体账，这几项在判档里只作展示，填 0 不影响判定。
            volume_mm3: 0.0,
            aabb_min: [world_aabb[0], world_aabb[1], world_aabb[2]],
            aabb_max: [world_aabb[3], world_aabb[4], world_aabb[5]],
            local_aabb_min: Some(local_min),
            local_aabb_max: Some(local_max),
            world: Some(world),
            components,
            kinds: member
                .geometries
                .iter()
                .map(|g| g.primitive.name().to_string())
                .collect(),
            genus: 0,
            negatives_applied: 0,
            negatives_skipped: 0,
        });
    }

    Ok((members, facets, generated))
}

/// E3D 导出不开「隐式管导成单独容器」时（APS 那批 TTY 普查就是），路由容器的隐式管直接写在
/// 容器自己那一组里，组名就是容器名：aps250104 `/PS-01-00111/B1`（TRUNNI）一条 `Cylinder`，
/// aps250110 `/KIOSK_PUMP_01_DISCHARGE-B1`（BRAN）两条。`gen_ams` 照「单独容器」的口径写成
/// `TUBI n of <容器>` 那几组，于是两边各成孤儿：基准那组没有同名生成元素、发不出主键，
/// 我们那几根落进只在生成侧。
///
/// 这里把同一容器名下的那几根并成一件，名字就是容器名，交给同名配对：世界 AABB 取并、
/// 记录种类按记录拼起来；局部帧不给，几根管各有各的轴，判档走世界帧。只并基准里**确有**
/// 同名、带几何、又不是 `NOUN n of …` 形态的成员，且生成侧没有同名元素的容器——开了「单独
/// 容器」的导出里容器那一组没有几何，一根都不会被并。返回并过的容器数。
fn merge_container_tubes(baseline: &[SnapshotMember], generated: &mut Vec<Generated>) -> usize {
    let named: std::collections::BTreeSet<String> = generated
        .iter()
        .filter_map(|element| element.name.as_deref())
        .map(|name| name.trim().to_string())
        .collect();
    let containers: std::collections::BTreeSet<String> = baseline
        .iter()
        .filter(|member| member.noun.is_none() && !member.geometries.is_empty())
        .filter_map(|member| member.name.as_deref())
        .map(|name| name.trim().to_string())
        .filter(|name| name.starts_with('/') && !named.contains(name))
        .collect();
    let mut merged: BTreeMap<String, Generated> = BTreeMap::new();
    let mut kept = Vec::with_capacity(generated.len());
    for element in generated.drain(..) {
        let container = (element.noun == "TUBI")
            .then_some(element.owner_desc.as_deref())
            .flatten()
            .and_then(|desc| desc.trim().split_once(' '))
            .map(|(noun, name)| (noun.to_string(), name.trim().to_string()))
            .filter(|(_, name)| containers.contains(name));
        let Some((noun, name)) = container else {
            kept.push(element);
            continue;
        };
        if let Some(whole) = merged.get_mut(&name) {
            for axis in 0..3 {
                whole.aabb_min[axis] = whole.aabb_min[axis].min(element.aabb_min[axis]);
                whole.aabb_max[axis] = whole.aabb_max[axis].max(element.aabb_max[axis]);
            }
            whole.volume_mm3 += element.volume_mm3;
            whole.kinds.extend(element.kinds);
            continue;
        }
        merged.insert(
            name.clone(),
            Generated {
                refno: format!("{noun} {name} 名下的隐式管"),
                noun,
                name: Some(name),
                owner_desc: None,
                ordinal: 0,
                volume_mm3: element.volume_mm3,
                aabb_min: element.aabb_min,
                aabb_max: element.aabb_max,
                local_aabb_min: None,
                local_aabb_max: None,
                world: None,
                components: Vec::new(),
                kinds: element.kinds,
                genus: 0,
                negatives_applied: 0,
                negatives_skipped: 0,
            },
        );
    }
    let count = merged.len();
    kept.extend(merged.into_values());
    *generated = kept;
    count
}

fn main() -> anyhow::Result<()> {
    let (baseline_path, elements_path, facets_path, out_path, noun_filter, generated_rvm) =
        parse_args()?;

    // RVM ↔ RVM：两侧同一个读法，`--facets` 不再需要（面片就在文件里）。
    //
    // 两侧的点云都**只留一份**：生成侧不建面片索引，基准侧的三样东西一路 move 到位。
    // 早先每一步都 `clone()`，同一份点云同时活着四份，管线根就是这么 OOM 的。
    let generated: Option<Vec<Generated>> = match generated_rvm.as_deref() {
        Some(path) => Some(load_rvm_side(path, false)?.2),
        None => None,
    };
    let baseline_rvm = match &generated {
        Some(_) => Some(load_rvm_side(&baseline_path, true)?),
        None => None,
    };

    let (baseline_members, baseline_facets) = match baseline_rvm {
        Some((members, facets, _)) => (Some(members), Some(facets)),
        None => (None, None),
    };
    let snapshot: Snapshot = match baseline_members {
        Some(members) => Snapshot {
            meta: SnapshotMeta {
                dbnum: None,
                root_name: members.first().and_then(|m| m.name.clone()),
                export_scope: Some("rvm".to_string()),
                member_count: Some(members.len()),
                geometry_count: Some(members.iter().map(|m| m.geometries.len()).sum()),
            },
            members,
        },
        None => serde_json::from_slice(
            &std::fs::read(&baseline_path)
                .with_context(|| format!("读基准 {}", baseline_path.display()))?,
        )
        .context("解析 RVM 快照 JSON")?,
    };
    let mut generated: Vec<Generated> = match generated {
        Some(generated) => generated,
        None => serde_json::from_slice(
            &std::fs::read(&elements_path)
                .with_context(|| format!("读生成物 {}", elements_path.display()))?,
        )
        .context("解析 elements.json")?,
    };
    let container_tube_groups = if generated_rvm.is_some() {
        merge_container_tubes(&snapshot.members, &mut generated)
    } else {
        0
    };

    // ---- 面片实算 AABB（可信基准，见模块文档） ----
    let facet_vertices: BTreeMap<String, FacetData> = match (baseline_facets, &facets_path) {
        (Some(facets), _) => facets,
        (None, None) => BTreeMap::new(),
        (None, Some(path)) => {
            let groups: Vec<FacetGroupDump> = serde_json::from_slice(
                &std::fs::read(path).with_context(|| format!("读面片 {}", path.display()))?,
            )
            .context("解析 rvm_facets_dump JSON")?;
            groups
                .into_iter()
                .filter_map(|group| {
                    let mut vertices = Vec::new();
                    let mut polygons = Vec::new();
                    for geometry in &group.geometries {
                        let Some(geometry_polygons) = &geometry.polygons_world_mm else {
                            continue;
                        };
                        for polygon in geometry_polygons {
                            let polygon_vertices = polygon
                                .iter()
                                .flatten()
                                .map(|vertex| {
                                    [vertex[0] as f64, vertex[1] as f64, vertex[2] as f64]
                                })
                                .collect::<Vec<_>>();
                            vertices.extend(polygon_vertices.iter().copied());
                            polygons.push(polygon_vertices);
                        }
                    }
                    (!vertices.is_empty()).then(|| {
                        (
                            group.name.trim().to_string(),
                            FacetData {
                                vertices,
                                components: connected_polygon_components(&polygons),
                            },
                        )
                    })
                })
                .collect()
        }
    };

    let keep = |noun: &str| {
        noun_filter
            .as_deref()
            .is_none_or(|want| noun4(want) == noun4(noun))
    };

    // 具名元素（AMS1112 的 PANE 主力）在 RVM 中没有 noun/owner_desc/ordinal，
    // 但 E3D 名称本身唯一且两侧都有。先建生成侧名字表，供基准侧补出主键和 noun。
    let mut generated_by_name: BTreeMap<String, &Generated> = BTreeMap::new();
    for element in &generated {
        let Some(name) = element
            .name
            .as_deref()
            .map(str::trim)
            .filter(|n| !n.is_empty())
        else {
            continue;
        };
        if generated_by_name
            .insert(name.to_string(), element)
            .is_some()
        {
            bail!("生成侧 E3D 名称重复：{name}");
        }
    }

    // ---- 基准侧：只要带几何且有世界 AABB 的成员 ----
    let mut base: BTreeMap<Key, BaseEntry<'_>> = BTreeMap::new();
    let mut unkeyable_baseline = Vec::new();
    let mut centreline_only_baseline = Vec::new();
    let (mut aabb_from_facets, mut aabb_from_snapshot) = (0usize, 0usize);
    for member in &snapshot.members {
        if member.geometries.is_empty() {
            continue;
        }
        if member.geometries.iter().all(is_centreline) {
            centreline_only_baseline.push(member.name.clone().unwrap_or_else(|| "?".to_string()));
            continue;
        }
        let facets = member
            .name
            .as_deref()
            .and_then(|name| facet_vertices.get(name.trim()));
        let (world_aabb, aabb_source) = match (facets, member.aabb_world_mm) {
            (Some(facets), _) => (
                aabb6(facets.vertices.iter().copied()).expect("non-empty facets"),
                "facets",
            ),
            (None, Some(aabb)) => (aabb, "snapshot"),
            (None, None) => {
                unkeyable_baseline.push(format!(
                    "{}（有几何但快照没写世界 AABB）",
                    member.name.as_deref().unwrap_or("?")
                ));
                continue;
            }
        };
        let named = member
            .name
            .as_deref()
            .map(str::trim)
            .and_then(|name| generated_by_name.get(name).map(|element| (name, *element)))
            .filter(|(_, element)| keep(&element.noun));
        let key = if let Some((name, element)) = named {
            Key::Name {
                name: name.to_string(),
                noun: noun4(&element.noun),
            }
        } else {
            let (Some(noun), Some(desc), Some(ordinal)) =
                (&member.noun, &member.owner_desc, member.ordinal)
            else {
                // 有 noun 过滤时，无法确认 noun 的无名基准不应污染该 noun 的未配对账；
                // 仍留在 unkeyable 里，明确它没有进入判据。
                unkeyable_baseline.push(format!(
                    "{}（缺 noun/owner_desc/ordinal，且无同名生成元素）",
                    member.name.as_deref().unwrap_or("?")
                ));
                continue;
            };
            if !keep(noun) {
                continue;
            }
            Key::OwnerOrdinal {
                owner: owner_key(desc),
                noun: noun4(noun),
                ordinal,
            }
        };
        match aabb_source {
            "facets" => aabb_from_facets += 1,
            _ => aabb_from_snapshot += 1,
        }
        if base
            .insert(key.clone(), (member, facets, world_aabb, aabb_source))
            .is_some()
        {
            bail!("基准里主键重复：{key:?}——`n of owner` 序号不唯一，对拍口径不成立");
        }
    }

    // ---- 生成侧 ----
    let mut ours: BTreeMap<Key, &Generated> = BTreeMap::new();
    let mut unkeyable_generated = Vec::new();
    for element in &generated {
        if !keep(&element.noun) {
            continue;
        }
        let key = if let Some(name) = element
            .name
            .as_deref()
            .map(str::trim)
            .filter(|n| !n.is_empty())
        {
            Key::Name {
                name: name.to_string(),
                noun: noun4(&element.noun),
            }
        } else {
            let Some(desc) = &element.owner_desc else {
                unkeyable_generated.push(format!("{} {}（属主无名）", element.refno, element.noun));
                continue;
            };
            Key::OwnerOrdinal {
                owner: owner_key(desc),
                noun: noun4(&element.noun),
                ordinal: element.ordinal,
            }
        };
        if ours.insert(key.clone(), element).is_some() {
            bail!("生成侧主键重复：{key:?}");
        }
    }

    // ---- 管身按几何配对 ----
    // E3D 的 `TUBE n` 在一部分支管上不是画出来的顺序（`/M-OR-1R323-A` 画的两根叫 TUBE 2 / 5，
    // `/LV-CO-1R330-A` 唯一一根叫 TUBE 3），拿序号当键会把几何相同的两根判成两边各多一根。
    // 同一属主下的 TUBI 改按世界 AABB 中心最近配对，序号只用来分组与显示。
    let base_tubes: Vec<(&Key, [f64; 6])> = base
        .iter()
        .filter(|(key, _)| key.noun() == "TUBI")
        .map(|(key, (_, _, aabb, _))| (key, *aabb))
        .collect();
    let our_tubes: Vec<(&Key, [f64; 6])> = ours
        .iter()
        .filter(|(key, _)| key.noun() == "TUBI")
        .map(|(key, element)| {
            (
                key,
                [
                    element.aabb_min[0],
                    element.aabb_min[1],
                    element.aabb_min[2],
                    element.aabb_max[0],
                    element.aabb_max[1],
                    element.aabb_max[2],
                ],
            )
        })
        .collect();
    let tube_pairs = pair_tubes_by_centre(&base_tubes, &our_tubes);
    let matched_ours: std::collections::BTreeSet<&Key> = tube_pairs.values().copied().collect();
    let counterpart = |key: &Key| -> Option<(&Key, &Generated)> {
        if key.noun() == "TUBI" {
            let ours_key = tube_pairs.get(key)?;
            ours.get(ours_key).map(|element| (*ours_key, *element))
        } else {
            ours.get_key_value(key).map(|(k, e)| (k, *e))
        }
    };

    // ---- 配对 ----
    let mut by_noun: BTreeMap<String, NounTally> = BTreeMap::new();
    let mut pairs = Vec::new();
    let mut only_baseline = Vec::new();
    let mut only_generated = Vec::new();
    let mut film_candidates = Vec::new();

    for (key, (member, facets, baseline_world_aabb, aabb_source)) in &base {
        let tally = by_noun.entry(key.noun().to_string()).or_default();
        let Some((ours_key, element)) = counterpart(key) else {
            tally.only_baseline += 1;
            only_baseline.push(member.name.clone().unwrap_or_else(|| format!("{key:?}")));
            continue;
        };
        tally.both += 1;

        let generated_world_aabb = [
            element.aabb_min[0],
            element.aabb_min[1],
            element.aabb_min[2],
            element.aabb_max[0],
            element.aabb_max[1],
            element.aabb_max[2],
        ];
        let local_pair = facets
            .as_ref()
            .zip(element.world)
            .zip(element.local_aabb_min.zip(element.local_aabb_max));
        let (baseline_aabb, generated_aabb, comparison_frame) = match local_pair {
            Some(((facets, world), (local_min, local_max))) => {
                let inverse = world_matrix(world).inverse();
                let local = facets.vertices.iter().map(|vertex| {
                    (inverse * glam::DVec3::from_array(*vertex).extend(1.0))
                        .truncate()
                        .to_array()
                });
                (
                    aabb6(local).context("facet 顶点转局部系后为空")?,
                    [
                        local_min[0],
                        local_min[1],
                        local_min[2],
                        local_max[0],
                        local_max[1],
                        local_max[2],
                    ],
                    "local",
                )
            }
            None => (*baseline_world_aabb, generated_world_aabb, "world"),
        };
        let delta_min_mm = std::array::from_fn(|i| generated_aabb[i] - baseline_aabb[i]);
        let delta_max_mm = std::array::from_fn(|i| generated_aabb[3 + i] - baseline_aabb[3 + i]);
        let max_axis_delta_mm = aabb_delta(baseline_aabb, generated_aabb);
        let tolerance = chord_tolerance_mm(&element.noun);
        let mut significant_baseline_aabb = None;
        let mut significant_generated_aabb = None;
        let mut significant_max_axis_delta_mm = None;
        let verdict = if max_axis_delta_mm <= EXACT_MM {
            tally.exact += 1;
            "exact"
        } else if max_axis_delta_mm <= tolerance {
            tally.chord += 1;
            "chord"
        } else if comparison_frame == "local" {
            let inverse =
                world_matrix(element.world.expect("local comparison has world")).inverse();
            let baseline_components = facets.as_ref().map(|facets| {
                facets
                    .components
                    .iter()
                    .filter_map(|component| {
                        let local = component
                            .iter()
                            .map(|vertex| {
                                (inverse * glam::DVec3::from_array(*vertex).extend(1.0))
                                    .truncate()
                                    .to_array()
                            })
                            .collect::<Vec<_>>();
                        let bounds = aabb6(local.iter().copied())?;
                        let planar = local.iter().map(|p| [p[0], p[1]]).collect::<Vec<_>>();
                        Some((bounds, planar_width(&planar)))
                    })
                    .collect::<Vec<_>>()
            });
            let generated_components = element
                .components
                .iter()
                .map(|component| {
                    aabb_component([
                        component.local_aabb_min[0],
                        component.local_aabb_min[1],
                        component.local_aabb_min[2],
                        component.local_aabb_max[0],
                        component.local_aabb_max[1],
                        component.local_aabb_max[2],
                    ])
                })
                .collect::<Vec<_>>();
            let significant = baseline_components
                .as_deref()
                .and_then(|components| significant_components_aabb(components, tolerance))
                .zip(significant_components_aabb(
                    &generated_components,
                    tolerance,
                ));
            match significant {
                Some(((baseline, baseline_filtered), (generated, _))) if baseline_filtered > 0 => {
                    let delta = aabb_delta(baseline, generated);
                    significant_baseline_aabb = Some(baseline);
                    significant_generated_aabb = Some(generated);
                    significant_max_axis_delta_mm = Some(delta);
                    if delta <= tolerance {
                        tally.reference_artifact += 1;
                        "reference_artifact"
                    } else {
                        tally.off += 1;
                        "off"
                    }
                }
                _ => {
                    tally.off += 1;
                    "off"
                }
            }
        } else {
            tally.off += 1;
            "off"
        };
        if verdict == "off" && comparison_frame == "local" {
            film_candidates.push(FilmCandidate {
                pair: pairs.len(),
                noun: key.noun().to_string(),
                baseline: member
                    .name
                    .as_deref()
                    .unwrap_or_default()
                    .trim()
                    .to_string(),
                generated: element.refno.clone(),
                world: element.world.expect("local comparison has world"),
                tolerance,
            });
        }

        // 图元种类是否同一套。多重集比较：RVM 里同一个成员的几条记录没有稳定顺序。
        let mut baseline_kinds: Vec<String> = member
            .geometries
            .iter()
            .filter_map(|g| g.as_str().map(str::to_string))
            .collect();
        let mut generated_kinds = element.kinds.clone();
        let kind_match = (!baseline_kinds.is_empty() && !generated_kinds.is_empty()).then(|| {
            baseline_kinds.sort();
            generated_kinds.sort();
            if baseline_kinds == generated_kinds {
                tally.kind_match += 1;
                true
            } else {
                tally.kind_differ += 1;
                false
            }
        });

        pairs.push(Pair {
            key: if ours_key == key {
                key.display()
            } else {
                // 几何配对且两侧序号不同：两边的号都留下，读报告时才知道 E3D 的 n 是几。
                format!("{} ↔ {}", key.display(), ours_key.display())
            },
            refno: element.refno.clone(),
            noun: element.noun.clone(),
            max_axis_delta_mm,
            raw_max_axis_delta_mm: max_axis_delta_mm,
            significant_max_axis_delta_mm,
            verdict,
            film_thickness_mm: None,
            film_side: None,
            aabb_source,
            comparison_frame,
            kind_match,
            baseline_kinds,
            generated_kinds,
            delta_min_mm,
            delta_max_mm,
            baseline_aabb,
            generated_aabb,
            significant_baseline_aabb,
            significant_generated_aabb,
            volume_mm3: element.volume_mm3,
            genus: element.genus,
            negatives_applied: element.negatives_applied,
            negatives_skipped: element.negatives_skipped,
        });
    }
    for (key, element) in &ours {
        let paired = if key.noun() == "TUBI" {
            matched_ours.contains(key)
        } else {
            base.contains_key(key)
        };
        if !paired {
            by_noun
                .entry(key.noun().to_string())
                .or_default()
                .only_generated += 1;
            only_generated.push(format!(
                "{} {} {}",
                element.refno,
                element.noun,
                key.display()
            ));
        }
    }

    // ---- 贴体薄膜（见模块文档）：只为 off 的那几件把两侧面片轮廓再读一遍 ----
    if let Some(generated_path) = generated_rvm.as_deref()
        && !film_candidates.is_empty()
    {
        let baseline_contours = member_contours(
            &baseline_path,
            &film_candidates.iter().map(|c| c.baseline.clone()).collect(),
            false,
        )?;
        let generated_contours = member_contours(
            generated_path,
            &film_candidates
                .iter()
                .map(|c| c.generated.clone())
                .collect(),
            true,
        )?;
        for candidate in &film_candidates {
            let (Some(baseline), Some(generated)) = (
                baseline_contours.get(&candidate.baseline),
                generated_contours.get(&candidate.generated),
            ) else {
                continue;
            };
            let inverse = world_matrix(candidate.world).inverse();
            let local = |contours: &[Vec<[f64; 3]>]| -> Vec<Vec<[f64; 3]>> {
                contours
                    .iter()
                    .map(|contour| {
                        contour
                            .iter()
                            .map(|vertex| {
                                (inverse * glam::DVec3::from_array(*vertex).extend(1.0))
                                    .truncate()
                                    .to_array()
                            })
                            .collect()
                    })
                    .collect()
            };
            let Some(film) =
                attached_film(&local(baseline), &local(generated), candidate.tolerance)
            else {
                continue;
            };
            let pair = &mut pairs[candidate.pair];
            pair.verdict = "film";
            pair.film_thickness_mm = Some(film.thickness_mm);
            pair.film_side = Some(film.side);
            pair.significant_max_axis_delta_mm =
                Some(aabb_delta(film.baseline_kept, film.generated_kept));
            pair.significant_baseline_aabb = Some(film.baseline_kept);
            pair.significant_generated_aabb = Some(film.generated_kept);
            let tally = by_noun
                .get_mut(&candidate.noun)
                .expect("candidate noun is tallied");
            tally.off -= 1;
            tally.film += 1;
        }
    }

    pairs.sort_by(|a, b| b.max_axis_delta_mm.total_cmp(&a.max_axis_delta_mm));

    let report = CompareReport {
        baseline: baseline_path.display().to_string(),
        elements: elements_path.display().to_string(),
        facets: facets_path.as_ref().map(|p| p.display().to_string()),
        aabb_from_facets,
        aabb_from_snapshot,
        dbnum: snapshot.meta.dbnum,
        root_name: snapshot.meta.root_name.clone(),
        export_scope: snapshot.meta.export_scope.clone(),
        baseline_members: snapshot.meta.member_count,
        baseline_geometries: snapshot.meta.geometry_count,
        unkeyable_generated,
        unkeyable_baseline,
        centreline_only_baseline,
        by_noun,
        pairs,
        only_baseline,
        only_generated,
        container_tube_groups,
    };

    println!(
        "基准 {} (dbnum={:?} scope={:?})  AABB 来源: 面片实算 {} / 快照字段 {}",
        report.root_name.as_deref().unwrap_or("?"),
        report.dbnum,
        report.export_scope.as_deref().unwrap_or("?"),
        report.aabb_from_facets,
        report.aabb_from_snapshot,
    );
    for (noun, tally) in &report.by_noun {
        println!(
            "{noun}: 两侧都有 {} (exact {} / chord {} / reference_artifact {} / film {} / off {})，只在基准 {}，只在生成 {}",
            tally.both,
            tally.exact,
            tally.chord,
            tally.reference_artifact,
            tally.film,
            tally.off,
            tally.only_baseline,
            tally.only_generated
        );
    }
    if !report.unkeyable_generated.is_empty() || !report.unkeyable_baseline.is_empty() {
        println!(
            "发不出主键：生成侧 {}，基准侧 {}",
            report.unkeyable_generated.len(),
            report.unkeyable_baseline.len()
        );
    }
    if report.container_tube_groups > 0 {
        println!(
            "隐式管写在容器那一组里：{} 只容器的管身并成一件按容器名配对",
            report.container_tube_groups
        );
    }
    if !report.centreline_only_baseline.is_empty() {
        println!(
            "基准里只有中心线、不进判据的成员 {} 件（E3D 画了线、我们还不画，账在 \
             report.json 的 centreline_only 上）",
            report.centreline_only_baseline.len()
        );
    }
    for pair in report.pairs.iter().take(10) {
        println!(
            "  {:>18} raw={:>9.4} mm sig={:>9}  {}  ({}, 基准={})",
            pair.verdict,
            pair.raw_max_axis_delta_mm,
            pair.significant_max_axis_delta_mm
                .map(|value| format!("{value:.4}"))
                .unwrap_or_else(|| "-".to_string()),
            pair.key,
            pair.refno,
            pair.aabb_source
        );
    }

    if let Some(path) = out_path {
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir).ok();
        }
        std::fs::write(&path, serde_json::to_vec_pretty(&report)?)?;
        println!("明细写到 {}", path.display());
    }

    let off: usize = report.by_noun.values().map(|t| t.off).sum();
    let both: usize = report.by_noun.values().map(|t| t.both).sum();
    // 两侧都没有可判的件是「这棵子树本来就不画」，不是口径对不上：SLYK `/发油泵棚一基础`
    // 那种 ZONE，E3D 导出的 RVM 里一个带几何的成员都没有，我们也一件没生成。只有一侧
    // 有东西才是真配不上。
    if both == 0 {
        if !report.only_baseline.is_empty() || !report.only_generated.is_empty() {
            bail!("一件都没配上——主键口径对不上，先看 only_baseline / only_generated");
        }
        println!("两侧都没有可判的件：这棵子树本来就不出几何");
    }
    if off > 0 {
        bail!("{off}/{both} 件超出弦高容差，看上面的 off 明细");
    }
    Ok(())
}

/// 同一属主下的管身按世界 AABB **中心最近**贪心配对；两边的键仍是 `TUBI n of owner`，
/// 但 n 只用来分组（`owner`）与显示，不参与配对。
///
/// 接受的条件：中心距不超过两者中较长的一根的半长（AABB 对角线的一半）——管身两根若不至少
/// 首尾相接就不算同一根；一根最多配一根。返回 `基准键 → 生成键`。
fn pair_tubes_by_centre<'a>(
    base: &[(&'a Key, [f64; 6])],
    ours: &[(&'a Key, [f64; 6])],
) -> BTreeMap<&'a Key, &'a Key> {
    fn owner(key: &Key) -> Option<&str> {
        match key {
            Key::OwnerOrdinal { owner, .. } => Some(owner),
            Key::Name { .. } => None,
        }
    }
    fn centre(aabb: &[f64; 6]) -> [f64; 3] {
        std::array::from_fn(|i| (aabb[i] + aabb[3 + i]) / 2.0)
    }
    fn half_diagonal(aabb: &[f64; 6]) -> f64 {
        (0..3)
            .map(|i| (aabb[3 + i] - aabb[i]).powi(2))
            .sum::<f64>()
            .sqrt()
            / 2.0
    }
    fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
        (0..3).map(|i| (a[i] - b[i]).powi(2)).sum::<f64>().sqrt()
    }

    let mut candidates: Vec<(f64, usize, usize)> = Vec::new();
    for (i, (base_key, base_aabb)) in base.iter().enumerate() {
        for (j, (our_key, our_aabb)) in ours.iter().enumerate() {
            if owner(base_key) != owner(our_key) {
                continue;
            }
            let gap = distance(centre(base_aabb), centre(our_aabb));
            let reach = half_diagonal(base_aabb).max(half_diagonal(our_aabb));
            if gap <= reach {
                candidates.push((gap, i, j));
            }
        }
    }
    candidates.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)).then(a.2.cmp(&b.2)));
    let mut taken_base = vec![false; base.len()];
    let mut taken_ours = vec![false; ours.len()];
    let mut pairs = BTreeMap::new();
    for (_, i, j) in candidates {
        if taken_base[i] || taken_ours[j] {
            continue;
        }
        taken_base[i] = true;
        taken_ours[j] = true;
        pairs.insert(base[i].0, ours[j].0);
    }
    pairs
}

fn world_matrix(columns: [[f64; 3]; 4]) -> glam::DMat4 {
    glam::DMat4::from_cols(
        glam::DVec3::from_array(columns[0]).extend(0.0),
        glam::DVec3::from_array(columns[1]).extend(0.0),
        glam::DVec3::from_array(columns[2]).extend(0.0),
        glam::DVec3::from_array(columns[3]).extend(1.0),
    )
}

fn aabb6(points: impl IntoIterator<Item = [f64; 3]>) -> Option<[f64; 6]> {
    let mut points = points.into_iter();
    let first = points.next()?;
    let mut lo = first;
    let mut hi = first;
    for point in points {
        for axis in 0..3 {
            lo[axis] = lo[axis].min(point[axis]);
            hi[axis] = hi[axis].max(point[axis]);
        }
    }
    Some([lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]])
}

fn aabb_delta(a: [f64; 6], b: [f64; 6]) -> f64 {
    a.into_iter().zip(b).fold(0.0_f64, |worst, (left, right)| {
        worst.max((left - right).abs())
    })
}

/// 用共享顶点把 RVM facet polygon 分成连通分量。RVM 顶点是 f32，
/// 0.01mm 量化只用于拓扑键，包围盒始终用原始坐标。
fn connected_polygon_components(polygons: &[Vec<[f64; 3]>]) -> Vec<Vec<[f64; 3]>> {
    if polygons.is_empty() {
        return Vec::new();
    }
    fn find(parent: &mut [usize], mut node: usize) -> usize {
        while parent[node] != node {
            parent[node] = parent[parent[node]];
            node = parent[node];
        }
        node
    }
    let mut parent = (0..polygons.len()).collect::<Vec<_>>();
    let mut owner = BTreeMap::<[i64; 3], usize>::new();
    for (index, polygon) in polygons.iter().enumerate() {
        for vertex in polygon {
            let key = std::array::from_fn(|axis| (vertex[axis] * 100.0).round() as i64);
            if let Some(&other) = owner.get(&key) {
                let left = find(&mut parent, index);
                let right = find(&mut parent, other);
                if left != right {
                    parent[right] = left;
                }
            } else {
                owner.insert(key, index);
            }
        }
    }
    let mut components = BTreeMap::<usize, Vec<[f64; 3]>>::new();
    for (index, polygon) in polygons.iter().enumerate() {
        let root = find(&mut parent, index);
        components
            .entry(root)
            .or_default()
            .extend(polygon.iter().copied());
    }
    components.into_values().collect()
}

/// 一个连通分量：局部 AABB 与局部 XY 里的最小宽度。
type Component = ([f64; 6], f64);

/// 只有 AABB 的分量（生成侧）：宽度取 XY 短边。
fn aabb_component(bounds: [f64; 6]) -> Component {
    let width = (bounds[3] - bounds[0])
        .abs()
        .min((bounds[4] - bounds[1]).abs());
    (bounds, width)
}

/// 局部 XY 点集凸包的最小宽度（逐条包边量对面最远点）。共线或不足三个不同点时是 0。
///
/// 斜放的细条 AABB 并不窄：ams7997 `/1RX-FN09-F001` FLOOR 2 的 E3D 面片里有一块只有两个
/// 不同 XY 点的零厚片，AABB 584 × 103 mm，宽度是 0。
fn planar_width(points: &[[f64; 2]]) -> f64 {
    let mut sorted = points.to_vec();
    sorted.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
    sorted.dedup();
    if sorted.len() < 3 {
        return 0.0;
    }
    let cross = |o: [f64; 2], a: [f64; 2], b: [f64; 2]| {
        (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    };
    let mut hull: Vec<[f64; 2]> = Vec::with_capacity(sorted.len() + 1);
    for point in sorted.iter().copied() {
        while hull.len() >= 2 && cross(hull[hull.len() - 2], hull[hull.len() - 1], point) <= 0.0 {
            hull.pop();
        }
        hull.push(point);
    }
    let lower = hull.len() + 1;
    for point in sorted.iter().rev().skip(1).copied() {
        while hull.len() >= lower && cross(hull[hull.len() - 2], hull[hull.len() - 1], point) <= 0.0
        {
            hull.pop();
        }
        hull.push(point);
    }
    hull.pop();
    if hull.len() < 3 {
        return 0.0;
    }
    (0..hull.len())
        .map(|index| {
            let (a, b) = (hull[index], hull[(index + 1) % hull.len()]);
            let length = (b[0] - a[0]).hypot(b[1] - a[1]);
            hull.iter()
                .map(|&point| cross(a, b, point).abs() / length)
                .fold(0.0_f64, f64::max)
        })
        .fold(f64::INFINITY, f64::min)
}

/// 去掉局部 XY 里最小宽度小于轮廓弦高的断开薄片；投影面积最大的主分量永远保留。
/// 返回并集 AABB 和被过滤的分量数。
fn significant_components_aabb(
    components: &[Component],
    tolerance: f64,
) -> Option<([f64; 6], usize)> {
    let main = components
        .iter()
        .enumerate()
        .max_by(|(_, (a, _)), (_, (b, _))| {
            let area =
                |bounds: &[f64; 6]| (bounds[3] - bounds[0]).abs() * (bounds[4] - bounds[1]).abs();
            area(a).total_cmp(&area(b))
        })
        .map(|(index, _)| index)?;
    let kept = components
        .iter()
        .enumerate()
        .filter_map(|(index, (bounds, width))| {
            (index == main || *width >= tolerance).then_some(*bounds)
        })
        .collect::<Vec<_>>();
    let mut combined = kept[0];
    for bounds in kept.iter().skip(1) {
        for axis in 0..3 {
            combined[axis] = combined[axis].min(bounds[axis]);
            combined[3 + axis] = combined[3 + axis].max(bounds[3 + axis]);
        }
    }
    Some((combined, components.len() - kept.len()))
}

/// 一件待判薄膜的 off：它在 `pairs` 里的位置，以及回头去两份 RVM 里找它要用的东西。
struct FilmCandidate {
    pair: usize,
    noun: String,
    /// 基准侧成员名（去掉首尾空白，与面片索引同一个键）。
    baseline: String,
    /// 生成侧成员路径（RVM ↔ RVM 时 `Generated::refno` 就是它）。
    generated: String,
    world: [[f64; 3]; 4],
    tolerance: f64,
}

/// 只为 `wanted` 里的成员，从 RVM 把 FacetGroup 的轮廓（世界 mm）再读一遍。`by_path` 为真按成员
/// 路径取键，否则按去掉首尾空白的成员名。带解析图元的成员不给——它们没有面片边可裁。
///
/// 两侧的面片索引只留了顶点与连通分量（见 [`load_rvm_side`]），轮廓只对这几件现读，
/// 整份点云不多留一份。
fn member_contours(
    path: &std::path::Path,
    wanted: &std::collections::BTreeSet<String>,
    by_path: bool,
) -> anyhow::Result<BTreeMap<String, Vec<Vec<[f64; 3]>>>> {
    let file = e3d_model::rvm_read::RvmFile::read(path)?;
    let mut out = BTreeMap::new();
    for member in &file.members {
        let key = if by_path {
            member.path.clone()
        } else {
            member.name.trim().to_string()
        };
        if !wanted.contains(&key) {
            continue;
        }
        let mut contours = Vec::new();
        let mut analytic = false;
        for geometry in &member.geometries {
            match &geometry.primitive {
                e3d_model::rvm_read::Primitive::FacetGroup(polygons) => {
                    for polygon in polygons {
                        for contour in &polygon.contours {
                            contours.push(
                                contour
                                    .vertices
                                    .iter()
                                    .map(|vertex| geometry.to_world_mm(*vertex))
                                    .collect(),
                            );
                        }
                    }
                }
                e3d_model::rvm_read::Primitive::Line(_) => {}
                _ => analytic = true,
            }
        }
        if !analytic && !contours.is_empty() {
            out.insert(key, contours);
        }
    }
    Ok(out)
}

/// 判成 `film` 的那一对：膜厚、膜在哪一侧、两侧去掉膜以后的局部 AABB。
struct Film {
    thickness_mm: f64,
    side: &'static str,
    baseline_kept: [f64; 6],
    generated_kept: [f64; 6],
}

/// 一侧相对于一只盒子多出来的部分。
struct Excess {
    /// 盒外顶点，加上面片边穿过盒面的交点：多出来那一块的外形点。
    points: Vec<[f64; 3]>,
    /// 盒内顶点的 AABB，即去掉多出来那一块以后的主体。
    kept: Option<[f64; 6]>,
}

fn excess_outside(contours: &[Vec<[f64; 3]>], bounds: [f64; 6]) -> Excess {
    let inside =
        |p: &[f64; 3]| (0..3).all(|axis| p[axis] >= bounds[axis] && p[axis] <= bounds[axis + 3]);
    let mut points = Vec::new();
    let mut kept = Vec::new();
    for contour in contours {
        for (index, &a) in contour.iter().enumerate() {
            if inside(&a) {
                kept.push(a);
            } else {
                points.push(a);
            }
            let b = contour[(index + 1) % contour.len()];
            if let Some((enter, leave)) = segment_in_box(a, b, bounds) {
                for t in [enter, leave] {
                    if t > 0.0 && t < 1.0 {
                        points.push(std::array::from_fn(|axis| {
                            a[axis] + (b[axis] - a[axis]) * t
                        }));
                    }
                }
            }
        }
    }
    Excess {
        points,
        kept: aabb6(kept),
    }
}

/// 线段 a→b 落在盒内的参数区间 `[t0, t1]`（Liang–Barsky）；不相交时 None。
fn segment_in_box(a: [f64; 3], b: [f64; 3], bounds: [f64; 6]) -> Option<(f64, f64)> {
    let (mut t0, mut t1) = (0.0_f64, 1.0_f64);
    for axis in 0..3 {
        let d = b[axis] - a[axis];
        for (p, q) in [
            (-d, a[axis] - bounds[axis]),
            (d, bounds[axis + 3] - a[axis]),
        ] {
            if p == 0.0 {
                if q < 0.0 {
                    return None;
                }
            } else if p < 0.0 {
                let r = q / p;
                if r > t1 {
                    return None;
                }
                t0 = t0.max(r);
            } else {
                let r = q / p;
                if r < t0 {
                    return None;
                }
                t1 = t1.min(r);
            }
        }
    }
    Some((t0, t1))
}

/// 点集在三个主方向（协方差的特征向量）上的跨度取最小：薄片给膜厚，发丝趋近 0。
fn min_principal_span(points: &[[f64; 3]]) -> f64 {
    let count = points.len() as f64;
    let mean: [f64; 3] =
        std::array::from_fn(|axis| points.iter().map(|p| p[axis]).sum::<f64>() / count);
    let mut covariance = [[0.0_f64; 3]; 3];
    for point in points {
        let d: [f64; 3] = std::array::from_fn(|axis| point[axis] - mean[axis]);
        for row in 0..3 {
            for col in 0..3 {
                covariance[row][col] += d[row] * d[col];
            }
        }
    }
    symmetric_eigenvectors(covariance)
        .iter()
        .map(|axis| {
            let (lo, hi) = points
                .iter()
                .map(|p| p[0] * axis[0] + p[1] * axis[1] + p[2] * axis[2])
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| {
                    (lo.min(v), hi.max(v))
                });
            hi - lo
        })
        .fold(f64::INFINITY, f64::min)
}

/// 3×3 对称阵的三个特征向量（循环 Jacobi）。
fn symmetric_eigenvectors(mut a: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..32 {
        let off = a[0][1].abs() + a[0][2].abs() + a[1][2].abs();
        let diagonal = a[0][0].abs() + a[1][1].abs() + a[2][2].abs();
        if off == 0.0 || off <= f64::EPSILON * diagonal {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            if a[p][q] == 0.0 {
                continue;
            }
            let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            for row in a.iter_mut() {
                let (rp, rq) = (row[p], row[q]);
                row[p] = c * rp - s * rq;
                row[q] = s * rp + c * rq;
            }
            for k in 0..3 {
                let (pk, qk) = (a[p][k], a[q][k]);
                a[p][k] = c * pk - s * qk;
                a[q][k] = s * pk + c * qk;
            }
            for row in v.iter_mut() {
                let (rp, rq) = (row[p], row[q]);
                row[p] = c * rp - s * rq;
                row[q] = s * rp + c * rq;
            }
        }
    }
    std::array::from_fn(|col| [v[0][col], v[1][col], v[2][col]])
}

/// 两侧（局部 mm 轮廓）是否只差一层贴体薄膜，见模块文档「贴体薄膜」。
fn attached_film(
    baseline: &[Vec<[f64; 3]>],
    generated: &[Vec<[f64; 3]>],
    tolerance: f64,
) -> Option<Film> {
    let bounds = |contours: &[Vec<[f64; 3]>]| aabb6(contours.iter().flatten().copied());
    let grown = |b: [f64; 6]| -> [f64; 6] {
        std::array::from_fn(|i| {
            if i < 3 {
                b[i] - tolerance
            } else {
                b[i] + tolerance
            }
        })
    };
    let baseline_excess = excess_outside(baseline, grown(bounds(generated)?));
    let generated_excess = excess_outside(generated, grown(bounds(baseline)?));
    let mut thickness_mm = 0.0_f64;
    let mut sides = Vec::new();
    for (side, excess) in [
        ("baseline", &baseline_excess),
        ("generated", &generated_excess),
    ] {
        if excess.points.is_empty() {
            continue;
        }
        let span = min_principal_span(&excess.points);
        if span > FILM_MM {
            return None;
        }
        thickness_mm = thickness_mm.max(span);
        sides.push(side);
    }
    let side = match sides.as_slice() {
        [] => return None,
        [one] => *one,
        _ => "both",
    };
    let (baseline_kept, generated_kept) = (baseline_excess.kept?, generated_excess.kept?);
    (aabb_delta(baseline_kept, generated_kept) <= tolerance).then_some(Film {
        thickness_mm,
        side,
        baseline_kept,
        generated_kept,
    })
}

/// `(baseline, elements, facets, out, noun, generated_rvm)`
///
/// 两种模式：
/// - `--baseline <*.rvm.json> --elements <elements.json> [--facets …]` —— 老路，不动。
/// - `--baseline <a.rvm> --generated <b.rvm>` —— RVM ↔ RVM，两侧同一个读法，
///   不需要 `--elements`，也不需要 `--facets`（面片就在文件里）。
type Args = (
    PathBuf,
    PathBuf,
    Option<PathBuf>,
    Option<PathBuf>,
    Option<String>,
    Option<PathBuf>,
);

fn parse_args() -> anyhow::Result<Args> {
    let mut baseline = None;
    let mut elements = None;
    let mut facets = None;
    let mut out = None;
    let mut noun = None;
    let mut generated = None;
    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--baseline" => baseline = Some(PathBuf::from(value()?)),
            "--elements" => elements = Some(PathBuf::from(value()?)),
            "--facets" => facets = Some(PathBuf::from(value()?)),
            "--out" => out = Some(PathBuf::from(value()?)),
            "--noun" => noun = Some(value()?),
            "--generated" => generated = Some(PathBuf::from(value()?)),
            other => bail!("不认识的参数 {other}"),
        }
    }
    if generated.is_some() {
        if elements.is_some() {
            bail!(
                "--generated 与 --elements 只能给一个：前者是 RVM ↔ RVM，后者是快照 ↔ elements.json"
            );
        }
        if facets.is_some() {
            bail!("--generated 模式下面片就在 .rvm 里，不要再给 --facets");
        }
    }
    Ok((
        baseline.context("缺 --baseline")?,
        match (&generated, elements) {
            (Some(_), None) => PathBuf::new(),
            (_, Some(path)) => path,
            (None, None) => bail!("缺 --elements（或用 --generated 走 RVM ↔ RVM）"),
        },
        facets,
        out,
        noun,
        generated,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inverse_world_recovers_local_aabb_for_rotated_element() {
        let angle = 37_f64.to_radians();
        let world = glam::DMat4::from_translation(glam::DVec3::new(100.0, -50.0, 7.0))
            * glam::DMat4::from_rotation_z(angle);
        let local = [
            [-10.0, -2.0, -3.0],
            [20.0, -2.0, -3.0],
            [-10.0, 4.0, 5.0],
            [20.0, 4.0, 5.0],
        ];
        let world_points: Vec<_> = local
            .iter()
            .map(|p| {
                (world * glam::DVec3::from_array(*p).extend(1.0))
                    .truncate()
                    .to_array()
            })
            .collect();
        let columns = [
            world.x_axis.truncate().to_array(),
            world.y_axis.truncate().to_array(),
            world.z_axis.truncate().to_array(),
            world.w_axis.truncate().to_array(),
        ];
        let inverse = world_matrix(columns).inverse();
        let recovered = world_points.iter().map(|p| {
            (inverse * glam::DVec3::from_array(*p).extend(1.0))
                .truncate()
                .to_array()
        });
        let aabb = aabb6(recovered).unwrap();
        assert!((aabb[0] + 10.0).abs() < 1e-9);
        assert!((aabb[1] + 2.0).abs() < 1e-9);
        assert!((aabb[2] + 3.0).abs() < 1e-9);
        assert!((aabb[3] - 20.0).abs() < 1e-9);
        assert!((aabb[4] - 4.0).abs() < 1e-9);
        assert!((aabb[5] - 5.0).abs() < 1e-9);
    }

    /// 中心线在两种快照形态里都要认得出来，混了实体的成员则照旧进判据。
    ///
    /// 形态一是 `rvm_verify import` 的对象，形态二是 [`load_rvm_side`] 的 kind 名。
    /// 只认一种的话，同一份 RVM 走快照路与走 RVM ↔ RVM 会判出两个结果。
    #[test]
    fn a_line_is_a_centreline_in_both_snapshot_shapes() {
        let object = serde_json::json!({"kind": "Line", "detail": {"start_radius": 5.0}});
        assert!(is_centreline(&object));
        assert!(is_centreline(&serde_json::Value::String("Line".into())));
        assert!(!is_centreline(&serde_json::json!({"kind": "Box"})));
        assert!(!is_centreline(&serde_json::Value::String("Box".into())));
        // 「全是中心线」才不进判据：线旁边还有实体的成员照判。
        let solid = serde_json::Value::String("Cylinder".into());
        assert!(![object, solid].iter().all(is_centreline));
    }

    #[test]
    fn noun_filter_uses_the_same_four_character_key_on_both_sides() {
        assert_eq!(noun4("GWALL"), noun4("GWAL"));
        assert_ne!(noun4("GWALL"), noun4("PANE"));
    }

    /// 用户自定义元素类型整串进键：截成四字会把同一属主下的三种支撑件挤成一个。
    #[test]
    fn user_defined_element_types_keep_their_full_noun() {
        assert_eq!(noun4(":SDS_Pole"), ":SDS_Pole");
        assert_ne!(noun4(":SDS_Pole"), noun4(":SDS_Plate"));
        assert_ne!(noun4(":SDS_Plate"), noun4(":SDS_Entity"));
    }

    /// 管身按几何配对：E3D 的 `TUBE 2 / TUBE 5` 要能对上我们的 `TUBI 1 / TUBI 2`；
    /// 隔着半根以上、或属主不同的不配；一根只配一根。
    #[test]
    fn tubes_pair_by_centre_regardless_of_ordinal() {
        let tube = |owner: &str, ordinal: usize| Key::OwnerOrdinal {
            owner: owner.to_string(),
            noun: "TUBI".to_string(),
            ordinal,
        };
        // 两根沿 X 的管：中心 (0,0,0) 长 3.5，中心 (100,0,0) 长 3.5；第三根属主不同。
        let (e2, e5, other) = (tube("/A", 2), tube("/A", 5), tube("/B", 1));
        let (o1, o2, o3) = (tube("/A", 1), tube("/A", 2), tube("/A", 3));
        let around = |x: f64, half: f64| [x - half, -12.5, -12.5, x + half, 12.5, 12.5];
        let base = vec![
            (&e2, around(0.0, 1.75)),
            (&e5, around(100.0, 1.75)),
            (&other, around(0.0, 1.75)),
        ];
        // 我们的第三根离得远（隔 50 mm，半长只有 1.75）——不许硬配。
        let ours = vec![
            (&o1, around(0.0, 1.75)),
            (&o2, around(100.0, 1.75)),
            (&o3, around(50.0, 1.75)),
        ];
        let pairs = pair_tubes_by_centre(&base, &ours);
        assert_eq!(pairs.get(&e2), Some(&&o1));
        assert_eq!(pairs.get(&e5), Some(&&o2));
        assert_eq!(pairs.get(&other), None, "属主不同不配");
        assert_eq!(pairs.len(), 2);

        // 中心距在半长以内才算同一根：1000 mm 的管偏 400 mm 仍配，偏 600 mm 不配。
        let long = |x: f64| [x - 500.0, -12.5, -12.5, x + 500.0, 12.5, 12.5];
        let (b, near, far) = (tube("/C", 1), tube("/C", 1), tube("/C", 2));
        assert_eq!(
            pair_tubes_by_centre(&[(&b, long(0.0))], &[(&near, long(400.0))]).len(),
            1
        );
        assert_eq!(
            pair_tubes_by_centre(&[(&b, long(0.0))], &[(&far, long(600.0))]).len(),
            0
        );
    }

    /// 不开「隐式管导成单独容器」的导出（APS 普查）：E3D 的隐式管就在容器那一组里，我们的
    /// `TUBI n of <容器>` 并成一件、按容器名配对；容器组没几何或基准里没有那只容器的不并。
    #[test]
    fn implied_tubes_merge_into_the_container_group_e3d_wrote_them_in() {
        let baseline_member = |name: &str, geometries: usize| SnapshotMember {
            name: Some(name.to_string()),
            noun: None,
            owner_desc: None,
            ordinal: None,
            aabb_world_mm: Some([0.0; 6]),
            geometries: vec![serde_json::Value::String("Cylinder".into()); geometries],
        };
        let tube = |owner: &str, ordinal: usize, x: f64| Generated {
            refno: format!("/GENERATED/TUBI {ordinal} of {owner}"),
            noun: "TUBI".to_string(),
            name: None,
            owner_desc: Some(owner.to_string()),
            ordinal,
            volume_mm3: 1.0,
            aabb_min: [x, 0.0, 0.0],
            aabb_max: [x + 10.0, 5.0, 5.0],
            local_aabb_min: Some([0.0; 3]),
            local_aabb_max: Some([10.0, 5.0, 5.0]),
            world: Some([
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
                [x, 0.0, 0.0],
            ]),
            components: Vec::new(),
            kinds: vec!["Cylinder".to_string()],
            genus: 0,
            negatives_applied: 0,
            negatives_skipped: 0,
        };
        let baseline = vec![
            baseline_member("/KIOSK-B1", 2),
            baseline_member("/SEPARATE-B1", 0),
        ];
        let mut generated = vec![
            tube("BRAN /KIOSK-B1", 1, 0.0),
            tube("BRAN /KIOSK-B1", 2, 100.0),
            tube("BRAN /SEPARATE-B1", 1, 0.0),
            tube("SUPC /ELSEWHERE-B1", 1, 0.0),
        ];
        assert_eq!(merge_container_tubes(&baseline, &mut generated), 1);
        assert_eq!(generated.len(), 3);
        let whole = generated
            .iter()
            .find(|element| element.name.as_deref() == Some("/KIOSK-B1"))
            .expect("并成一件的容器");
        assert_eq!(whole.noun, "BRAN");
        assert_eq!(whole.aabb_min, [0.0, 0.0, 0.0]);
        assert_eq!(whole.aabb_max, [110.0, 5.0, 5.0]);
        assert_eq!(whole.kinds, ["Cylinder", "Cylinder"]);
        assert!(whole.world.is_none(), "几根管各有各的轴，判档走世界帧");
        assert_eq!(
            generated
                .iter()
                .filter(|element| element.noun == "TUBI")
                .count(),
            2,
            "容器组没几何、或基准里没有那只容器的，照旧按管身配"
        );
    }

    #[test]
    fn nested_unnamed_owner_key_keeps_each_ordinal_layer() {
        assert_eq!(
            owner_key("SUBEQUIPMENT 20 of EQUIPMENT /Copy-of-R43-R740-002"),
            "SUBE 20 of /Copy-of-R43-R740-002"
        );
        assert_ne!(
            owner_key("SUBEQUIPMENT 1 of EQUIPMENT /Copy-of-R43-R740-002"),
            owner_key("SUBEQUIPMENT 2 of EQUIPMENT /Copy-of-R43-R740-002")
        );
        assert_eq!(
            owner_key("CWALL /1RS-WF03-W-C-RR001"),
            "/1RS-WF03-W-C-RR001"
        );
    }

    #[test]
    fn extrusion_nouns_use_profile_chord_tolerance() {
        assert_eq!(chord_tolerance_mm("PANE"), 10.0);
        assert_eq!(chord_tolerance_mm("FLOOR"), 10.0);
        assert_eq!(chord_tolerance_mm("GWALL"), 10.0);
        assert_eq!(chord_tolerance_mm("CYLI"), 0.5);
    }

    #[test]
    fn polygon_components_join_only_through_shared_quantized_vertices() {
        let polygons = vec![
            vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
            vec![[1.004, 0.0, 0.0], [2.0, 0.0, 0.0]],
            vec![[20.0, 0.0, 0.0], [21.0, 0.0, 0.0]],
        ];
        let components = connected_polygon_components(&polygons);
        assert_eq!(components.len(), 2);
        assert_eq!(components.iter().map(Vec::len).sum::<usize>(), 6);
    }

    #[test]
    fn significant_aabb_keeps_main_and_drops_sub_chord_satellite() {
        let components = [
            [-100.0, -200.0, -10.0, 100.0, 200.0, 0.0],
            [300.0, -500.0, -10.0, 305.0, -400.0, 0.0],
        ]
        .map(aabb_component);
        let (aabb, filtered) = significant_components_aabb(&components, 10.0).unwrap();
        assert_eq!(filtered, 1);
        assert_eq!(aabb, components[0].0);
    }

    /// ams7997 `/1RX-FN09-F001` FLOOR 2：E3D 在负体拐角留下一块只有两个不同 XY 点的零厚片
    /// （(30612.16, 4890.037) ↔ (31196.15, 4993.01)，70 mm 高）。AABB 短边 103 mm 留得住，宽度是 0。
    #[test]
    fn a_diagonal_zero_width_sheet_is_not_significant() {
        let sheet = [
            [30612.16, 4890.037],
            [31196.15, 4993.01],
            [31196.15, 4993.01],
        ];
        assert_eq!(planar_width(&sheet), 0.0);
        let components = [
            aabb_component([0.0, 24888.836, -70.0, 15225.079, 31600.0, 0.0]),
            (
                [30612.16, 4890.037, -70.0, 31196.15, 4993.01, 0.0],
                planar_width(&sheet),
            ),
        ];
        let (aabb, filtered) = significant_components_aabb(&components, 10.0).unwrap();
        assert_eq!(filtered, 1);
        assert_eq!(aabb, components[0].0);
    }

    #[test]
    fn planar_width_measures_across_a_rotated_strip() {
        let (angle, length, width) = (0.3_f64, 500.0, 4.0);
        let (sin, cos) = angle.sin_cos();
        let corners = [[0.0, 0.0], [length, 0.0], [length, width], [0.0, width]]
            .map(|[x, y]| [x * cos - y * sin, x * sin + y * cos]);
        assert!((planar_width(&corners) - width).abs() < 1e-9);
        let bounds_short_side = corners
            .iter()
            .map(|p| p[1])
            .fold(f64::NEG_INFINITY, f64::max)
            - corners.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min);
        assert!(
            bounds_short_side > 100.0,
            "AABB 短边量不出它窄：{bounds_short_side}"
        );
    }

    /// RVM ↔ RVM 这条路的生成侧也要有分量账：一个成员里两块不连通的面片读回来是两个分量。
    /// 没有它，基准侧滤掉了窄于弦高的断开薄片、生成侧却交不出分量，reference_artifact 那一档
    /// 在 RVM ↔ RVM 上永远进不去（ams1112 FLOOR 9 of CFLOOR /1RS-WF05-F-C-F001 就这样判成了 off）。
    fn cuboid_contours(lo: [f64; 3], hi: [f64; 3]) -> Vec<Vec<[f64; 3]>> {
        let corner = |x: bool, y: bool, z: bool| {
            [
                if x { hi[0] } else { lo[0] },
                if y { hi[1] } else { lo[1] },
                if z { hi[2] } else { lo[2] },
            ]
        };
        let (o, i) = (false, true);
        vec![
            vec![
                corner(o, o, o),
                corner(i, o, o),
                corner(i, i, o),
                corner(o, i, o),
            ],
            vec![
                corner(o, o, i),
                corner(i, o, i),
                corner(i, i, i),
                corner(o, i, i),
            ],
            vec![
                corner(o, o, o),
                corner(i, o, o),
                corner(i, o, i),
                corner(o, o, i),
            ],
            vec![
                corner(o, i, o),
                corner(i, i, o),
                corner(i, i, i),
                corner(o, i, i),
            ],
            vec![
                corner(o, o, o),
                corner(o, i, o),
                corner(o, i, i),
                corner(o, o, i),
            ],
            vec![
                corner(i, o, o),
                corner(i, i, o),
                corner(i, i, i),
                corner(i, o, i),
            ],
        ]
    }

    #[test]
    fn a_thin_fin_on_top_is_a_film_but_a_longer_block_is_not() {
        let block = cuboid_contours([0.0; 3], [1000.0, 200.0, 3000.0]);
        // GWALL 13 那种：顶上多一片 0.01 mm 厚、26 mm 高的膜。
        let mut with_fin = block.clone();
        with_fin.extend(cuboid_contours(
            [500.0, 0.0, 3000.0],
            [500.01, 200.0, 3026.0],
        ));
        let film = attached_film(&with_fin, &block, 10.0).expect("a fin is a film");
        assert_eq!(film.side, "baseline");
        assert!(
            (film.thickness_mm - 0.01).abs() < 1e-6,
            "{}",
            film.thickness_mm
        );
        assert!(aabb_delta(film.baseline_kept, film.generated_kept) < 1e-9);
        // 真的长出 50 mm：盒外顶点只是端面四个点（共面），算上侧边与盒面的交点才量得出厚度。
        let longer = cuboid_contours([0.0; 3], [1000.0, 200.0, 3050.0]);
        assert!(attached_film(&longer, &block, 10.0).is_none());
    }

    #[test]
    fn a_zero_thickness_hair_on_the_generated_side_is_a_film() {
        // FN01 FLOOR 12 那种：生成侧切剩一根 100 mm 长的零厚发丝（退化三角）。
        let block = cuboid_contours([0.0; 3], [1000.0, 1000.0, 200.0]);
        let mut with_hair = block.clone();
        with_hair.push(vec![
            [1000.0, 0.0, 0.0],
            [1100.0, 0.0, 0.0],
            [1000.0, 0.0, 0.0],
        ]);
        let film = attached_film(&block, &with_hair, 10.0).expect("a hair is a film");
        assert_eq!(film.side, "generated");
        assert!(film.thickness_mm < 1e-9, "{}", film.thickness_mm);
    }

    #[test]
    fn principal_span_finds_the_thickness_of_a_tilted_sheet() {
        // 绕 z 转 30° 的 0.02 mm 厚薄片：轴对齐的盒宽几百毫米，主方向上就是 0.02。
        let (c, s) = (30f64.to_radians().cos(), 30f64.to_radians().sin());
        let points = cuboid_contours([0.0; 3], [800.0, 0.02, 300.0])
            .into_iter()
            .flatten()
            .map(|[x, y, z]| [c * x - s * y, s * x + c * y, z])
            .collect::<Vec<_>>();
        let span = min_principal_span(&points);
        assert!((span - 0.02).abs() < 1e-6, "{span}");
    }

    #[test]
    fn rvm_side_reports_components_for_the_generated_side() {
        fn cuboid(lo: [f64; 3], hi: [f64; 3], base: u32) -> (Vec<[f64; 3]>, Vec<[u32; 3]>) {
            let vertices = (0..8)
                .map(|i| {
                    [
                        if i & 1 == 0 { lo[0] } else { hi[0] },
                        if i & 2 == 0 { lo[1] } else { hi[1] },
                        if i & 4 == 0 { lo[2] } else { hi[2] },
                    ]
                })
                .collect();
            let faces: [[u32; 4]; 6] = [
                [0, 2, 3, 1],
                [4, 5, 7, 6],
                [0, 1, 5, 4],
                [2, 6, 7, 3],
                [0, 4, 6, 2],
                [1, 3, 7, 5],
            ];
            let triangles = faces
                .iter()
                .flat_map(|[a, b, c, d]| {
                    [
                        [a + base, b + base, c + base],
                        [a + base, c + base, d + base],
                    ]
                })
                .collect();
            (vertices, triangles)
        }
        let (mut vertices, mut triangles) = cuboid([0.0, 0.0, -100.0], [1000.0, 800.0, 0.0], 0);
        let (sliver_vertices, sliver_triangles) =
            cuboid([2000.0, 0.0, -100.0], [2002.0, 500.0, 0.0], 8);
        vertices.extend(sliver_vertices);
        triangles.extend(sliver_triangles);
        let identity = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let geometry = e3d_model::rvm_write::facet_group_from_world_triangles(
            identity, [0.0; 3], &vertices, &triangles,
        );
        let file = e3d_model::rvm_read::RvmFile {
            header: Default::default(),
            members: vec![e3d_model::rvm_read::RvmMember {
                name: "FLOOR 1 of CFLOOR /T".to_string(),
                path: "/FLOOR 1 of CFLOOR /T".to_string(),
                parent: None,
                translation_mm: [0.0; 3],
                material: 0,
                geometries: vec![geometry],
            }],
        };
        let path =
            std::env::temp_dir().join(format!("rvm_compare_components_{}.rvm", std::process::id()));
        file.write(&path).unwrap();
        let loaded = load_rvm_side(&path, false);
        std::fs::remove_file(&path).ok();
        let (_, facets, generated) = loaded.unwrap();
        assert!(facets.is_empty(), "生成侧不建面片索引");
        let [element] = generated.as_slice() else {
            panic!("应该恰好一个成员");
        };
        let mut components = element
            .components
            .iter()
            .map(|c| {
                let (lo, hi) = (c.local_aabb_min, c.local_aabb_max);
                aabb_component([lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]])
            })
            .collect::<Vec<_>>();
        components.sort_by(|a, b| a.0[0].total_cmp(&b.0[0]));
        assert_eq!(components.len(), 2, "{components:?}");
        assert!((components[1].0[3] - components[1].0[0] - 2.0).abs() < 0.01);
        let (aabb, filtered) = significant_components_aabb(&components, 10.0).unwrap();
        assert_eq!(filtered, 1);
        assert!((aabb[3] - 1000.0).abs() < 0.01, "{aabb:?}");
    }
}

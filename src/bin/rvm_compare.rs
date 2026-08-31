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
//! | `off` | 更大 | 要查 |
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

use std::collections::BTreeMap;
use std::path::PathBuf;

use anyhow::{Context, bail};
use serde::{Deserialize, Serialize};

/// 只差 RVM f32 舍入的量级（mm）。坐标量级 ~2e4，f32 相对精度 ~1e-7。
const EXACT_MM: f64 = 0.05;

fn chord_tolerance_mm(noun: &str) -> f64 {
    match noun4(noun).as_str() {
        "PANE" | "FLOO" | "GWAL" => e3d_model::discretise::PROFILE_CHORD_TOL_MM,
        _ => e3d_model::discretise::CHORD_TOL_MM,
    }
}

#[derive(Deserialize)]
struct Snapshot {
    meta: SnapshotMeta,
    members: Vec<SnapshotMember>,
}

#[derive(Deserialize)]
struct SnapshotMeta {
    dbnum: Option<u32>,
    root_name: Option<String>,
    export_scope: Option<String>,
    member_count: Option<usize>,
    geometry_count: Option<usize>,
}

#[derive(Deserialize)]
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

#[derive(Deserialize)]
struct GeneratedComponent {
    local_aabb_min: [f64; 3],
    local_aabb_max: [f64; 3],
}

#[derive(Deserialize)]
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

fn noun4(noun: &str) -> String {
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
    /// 只有 `reference_artifact` 才写：丢弃小于一个轮廓弦高的断开薄片后的差。
    significant_max_axis_delta_mm: Option<f64>,
    /// 分档：exact / chord / reference_artifact / off。
    verdict: &'static str,
    /// 基准 AABB 的来源：`facets`（顶点实算，可信）/ `snapshot`（PRIM bbox 字段，见模块文档）。
    aabb_source: &'static str,
    /// `local` 为可信判据；`world` 是缺局部帧时的兼容回退。
    comparison_frame: &'static str,
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
    off: usize,
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
    by_noun: BTreeMap<String, NounTally>,
    pairs: Vec<Pair>,
    only_baseline: Vec<String>,
    only_generated: Vec<String>,
}

/// 基准侧一条待配记录：成员本身、面片（缺面片的走 snapshot AABB）、世界 AABB、AABB 来源标记。
type BaseEntry<'a> = (
    &'a SnapshotMember,
    Option<FacetData>,
    [f64; 6],
    &'static str,
);

fn main() -> anyhow::Result<()> {
    let (baseline_path, elements_path, facets_path, out_path, noun_filter) = parse_args()?;

    let snapshot: Snapshot = serde_json::from_slice(
        &std::fs::read(&baseline_path)
            .with_context(|| format!("读基准 {}", baseline_path.display()))?,
    )
    .context("解析 RVM 快照 JSON")?;
    let generated: Vec<Generated> = serde_json::from_slice(
        &std::fs::read(&elements_path)
            .with_context(|| format!("读生成物 {}", elements_path.display()))?,
    )
    .context("解析 elements.json")?;

    // ---- 面片实算 AABB（可信基准，见模块文档） ----
    let facet_vertices: BTreeMap<String, FacetData> = match &facets_path {
        None => BTreeMap::new(),
        Some(path) => {
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
    let (mut aabb_from_facets, mut aabb_from_snapshot) = (0usize, 0usize);
    for member in &snapshot.members {
        if member.geometries.is_empty() {
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
            .insert(
                key.clone(),
                (member, facets.cloned(), world_aabb, aabb_source),
            )
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

    // ---- 配对 ----
    let mut by_noun: BTreeMap<String, NounTally> = BTreeMap::new();
    let mut pairs = Vec::new();
    let mut only_baseline = Vec::new();
    let mut only_generated = Vec::new();

    for (key, (member, facets, baseline_world_aabb, aabb_source)) in &base {
        let tally = by_noun.entry(key.noun().to_string()).or_default();
        let Some(element) = ours.get(key) else {
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
                        aabb6(component.iter().map(|vertex| {
                            (inverse * glam::DVec3::from_array(*vertex).extend(1.0))
                                .truncate()
                                .to_array()
                        }))
                    })
                    .collect::<Vec<_>>()
            });
            let generated_components = element
                .components
                .iter()
                .map(|component| {
                    [
                        component.local_aabb_min[0],
                        component.local_aabb_min[1],
                        component.local_aabb_min[2],
                        component.local_aabb_max[0],
                        component.local_aabb_max[1],
                        component.local_aabb_max[2],
                    ]
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

        pairs.push(Pair {
            key: key.display(),
            refno: element.refno.clone(),
            noun: element.noun.clone(),
            max_axis_delta_mm,
            raw_max_axis_delta_mm: max_axis_delta_mm,
            significant_max_axis_delta_mm,
            verdict,
            aabb_source,
            comparison_frame,
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
        if !base.contains_key(key) {
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
        by_noun,
        pairs,
        only_baseline,
        only_generated,
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
            "{noun}: 两侧都有 {} (exact {} / chord {} / reference_artifact {} / off {})，只在基准 {}，只在生成 {}",
            tally.both,
            tally.exact,
            tally.chord,
            tally.reference_artifact,
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
    if both == 0 {
        bail!("一件都没配上——主键口径对不上，先看 only_baseline / only_generated");
    }
    if off > 0 {
        bail!("{off}/{both} 件超出弦高容差，看上面的 off 明细");
    }
    Ok(())
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

/// 去掉局部 XY 中任一尺寸小于轮廓弦高的断开薄片；投影面积
/// 最大的主分量永远保留。返回并集 AABB 和被过滤的分量数。
fn significant_components_aabb(
    components: &[[f64; 6]],
    tolerance: f64,
) -> Option<([f64; 6], usize)> {
    let main = components
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| {
            let area =
                |bounds: &[f64; 6]| (bounds[3] - bounds[0]).abs() * (bounds[4] - bounds[1]).abs();
            area(a).total_cmp(&area(b))
        })
        .map(|(index, _)| index)?;
    let kept = components
        .iter()
        .enumerate()
        .filter_map(|(index, bounds)| {
            let dx = (bounds[3] - bounds[0]).abs();
            let dy = (bounds[4] - bounds[1]).abs();
            (index == main || dx.min(dy) >= tolerance).then_some(*bounds)
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

type Args = (
    PathBuf,
    PathBuf,
    Option<PathBuf>,
    Option<PathBuf>,
    Option<String>,
);

fn parse_args() -> anyhow::Result<Args> {
    let mut baseline = None;
    let mut elements = None;
    let mut facets = None;
    let mut out = None;
    let mut noun = None;
    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--baseline" => baseline = Some(PathBuf::from(value()?)),
            "--elements" => elements = Some(PathBuf::from(value()?)),
            "--facets" => facets = Some(PathBuf::from(value()?)),
            "--out" => out = Some(PathBuf::from(value()?)),
            "--noun" => noun = Some(value()?),
            other => bail!("不认识的参数 {other}"),
        }
    }
    Ok((
        baseline.context("缺 --baseline")?,
        elements.context("缺 --elements")?,
        facets,
        out,
        noun,
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

    #[test]
    fn noun_filter_uses_the_same_four_character_key_on_both_sides() {
        assert_eq!(noun4("GWALL"), noun4("GWAL"));
        assert_ne!(noun4("GWALL"), noun4("PANE"));
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
        ];
        let (aabb, filtered) = significant_components_aabb(&components, 10.0).unwrap();
        assert_eq!(filtered, 1);
        assert_eq!(aabb, components[0]);
    }
}

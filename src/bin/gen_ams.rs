//! ams 库直读生成探针：开库 → 找根 → 生成 → OBJ + 不变量 + 总账落盘。
//!
//! 用法（路径全部显式给，不猜安装位置）：
//!
//! ```text
//! gen_ams --attlib <attlib.dat> --db <ams1112_0001> --template <模板文件或目录> \
//!         [--db-type DESI] [--sesno N] [--root w0/w1] [--out out/ams1112] \
//!         [--no-instance-cache]
//! ```
//!
//! 产物：
//! - `<out>/model.obj`：全部实体，一个 OBJ object 一个元素（`refno noun name`）。
//! - `<out>/elements.json`：逐元素不变量（体积/AABB/三角数/亏格/负体数）——
//!   RVM 对拍就吃这个。
//! - `<out>/report.json`：管线总账（skipped/failed/catalog/unsupported/unknown）。

use std::collections::BTreeMap;
use std::io::Write as _;
use std::path::{Path, PathBuf};

use anyhow::{Context, bail};
use e3d_io::db_element::{DbFilePin, DbSet, template_file_for};
use e3d_io::engine::{ReadOnlyEngine, ScanTier};
use e3d_io::refno::RefNo;
use e3d_model::mesh::TriMesh;
use e3d_model::pipeline::{GenerationOutcome, Report, generate_subtree};

struct Args {
    attlib: PathBuf,
    db_file: PathBuf,
    template: PathBuf,
    db_type: String,
    sesno: Option<u32>,
    root: Option<RefNo>,
    out: PathBuf,
    no_instance_cache: bool,
}

fn parse_args() -> anyhow::Result<Args> {
    let mut attlib = None;
    let mut db_file = None;
    let mut template = None;
    let mut db_type = "DESI".to_string();
    let mut sesno = None;
    let mut root = None;
    let mut out = PathBuf::from("out/gen_ams");
    let mut no_instance_cache = false;

    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--attlib" => attlib = Some(PathBuf::from(value()?)),
            "--db" => db_file = Some(PathBuf::from(value()?)),
            "--template" => template = Some(PathBuf::from(value()?)),
            "--db-type" => db_type = value()?,
            "--sesno" => sesno = Some(value()?.parse().context("--sesno 要无符号整数")?),
            "--root" => root = Some(parse_refno(&value()?)?),
            "--out" => out = PathBuf::from(value()?),
            "--no-instance-cache" => no_instance_cache = true,
            other => bail!("不认识的参数 {other}"),
        }
    }
    Ok(Args {
        attlib: attlib.context("缺 --attlib")?,
        db_file: db_file.context("缺 --db")?,
        template: template.context("缺 --template")?,
        db_type,
        sesno,
        root,
        out,
        no_instance_cache,
    })
}

fn parse_refno(text: &str) -> anyhow::Result<RefNo> {
    let trimmed = text.trim_start_matches('=');
    let (w0, w1) = trimmed
        .split_once('/')
        .with_context(|| format!("--root 要 `w0/w1` 形式，得到 {text}"))?;
    Ok(RefNo::new(w0.trim().parse()?, w1.trim().parse()?))
}

/// 索引侧的独立普查：树根 + 元素总数。
///
/// 总数是给 [`Report::accounts_for`] 当判据的。它必须来自索引（`scan_elements`）
/// 而不是管线自己的遍历——同源就验不出「有子树没走到」。
struct IndexCensus {
    roots: Vec<RefNo>,
    total: usize,
}

/// 树根 = owner 指向自己、无效、或不在本库索引里的元素（WORL 语义）。
fn scan_index(db_file: &Path, sesno: Option<u32>) -> anyhow::Result<IndexCensus> {
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
    let roots = owners
        .iter()
        .filter(|(refno, owner)| owner == refno || !owner.is_valid() || !indexed.contains(owner))
        .map(|(refno, _)| *refno)
        .collect();
    Ok(IndexCensus {
        roots,
        total: indexed.len(),
    })
}

#[derive(serde::Serialize)]
struct ElementInvariants {
    refno: String,
    noun: String,
    name: Option<String>,
    /// 属主的显示名（`NOUN /名`），属主无名时为 `None`。见 [`owner_keys`]。
    owner_desc: Option<String>,
    /// 本元素在属主**同 noun 兄弟**里的序号，文件原序 1 起。
    ordinal: usize,
    volume_mm3: f64,
    aabb_min: [f64; 3],
    aabb_max: [f64; 3],
    /// 元素局部系 AABB。RVM facet 必须先乘 `world^-1` 再与它比较；
    /// 旋转元素不能拿世界 AABB 的 X/Y 轴直接对拍。
    local_aabb_min: [f64; 3],
    local_aabb_max: [f64; 3],
    /// 元素局部系到世界系，列主序 `[c0,c1,c2,t]`。
    world: [[f64; 3]; 4],
    triangles: usize,
    genus: i32,
    /// 布尔运算后的连通体明细。只看总 AABB 会把主体和远端碎片混在一起。
    components: Vec<ComponentInvariants>,
    negatives_applied: usize,
    negatives_skipped: usize,
}

#[derive(serde::Serialize)]
struct ComponentInvariants {
    volume_mm3: f64,
    local_aabb_min: [f64; 3],
    local_aabb_max: [f64; 3],
    triangles: usize,
    genus: i32,
}

/// 对拍主键：属主显示名 + 同 noun 兄弟序号。
///
/// E3D 给无名元素的显示名是「NOUN n of <属主显示名>」，RVM 导出物里的
/// `owner_desc` / `ordinal` 正是这一对（实例：`GWALL 1 of CWALL /1RS-WF03-W-C-RR001`）。
/// 生成侧的元素大多无名，refno 又不进 RVM，所以这是两侧唯一对得上的身份。
///
/// 一次走完每个属主的成员表就把整组序号发完；逐元素回头数在 PANE 密集的 CWALL 上
/// 是几百万步的重复遍历。
///
/// 属主无名时给 `None` 而不是拼一个凑合的名字：对拍工具据此把它报成「配不上」，
/// 而不是拿一个对不上的键去误配——错配比缺配难查得多。
fn owner_keys(
    set: &std::sync::Arc<DbSet>,
    owners: &std::collections::HashSet<RefNo>,
) -> anyhow::Result<std::collections::HashMap<RefNo, (Option<String>, usize)>> {
    let mut keys = std::collections::HashMap::new();
    for owner_ref in owners {
        let owner = set.element(*owner_ref);
        let desc = match owner.stored_name()? {
            Some(name) => Some(format!("{} {name}", owner.element_type()?)),
            None => None,
        };
        let mut seen: BTreeMap<String, usize> = BTreeMap::new();
        for member in owner.members()? {
            let noun = member.element_type()?;
            let ordinal = seen.entry(noun).or_insert(0);
            *ordinal += 1;
            keys.insert(member.refno(), (desc.clone(), *ordinal));
        }
    }
    Ok(keys)
}

fn main() -> anyhow::Result<()> {
    let args = parse_args()?;
    if args.no_instance_cache {
        std::env::set_var(e3d_model::primitive_instance::INSTANCE_CACHE_ENV, "1");
    }
    let t0 = std::time::Instant::now();

    let template_file = if args.template.is_dir() {
        template_file_for(&args.template, &args.db_type)?
    } else {
        args.template.clone()
    };

    let set = std::sync::Arc::new(DbSet::with_attlib_file(&args.attlib)?);
    let dbno = set.add_db(DbFilePin {
        file: args.db_file.clone(),
        template: template_file,
        db_type: Some(args.db_type.clone()),
        sesno: args.sesno,
    })?;
    let pinned = set.pinned_sesno(dbno);
    println!(
        "库 {} 打开为 dbno={dbno}，钉在会话 {pinned:?}",
        args.db_file.display()
    );

    // 只在从头整库生成时才拿索引总数当判据；`--root` 是子树，对不上是应该的。
    let (roots, index_total) = match args.root {
        Some(refno) => (vec![refno], None),
        None => {
            let census = scan_index(&args.db_file, args.sesno)?;
            println!(
                "索引 {} 个元素，扫到 {} 个根: {:?}",
                census.total,
                census.roots.len(),
                census.roots
            );
            (census.roots, Some(census.total))
        }
    };
    if roots.is_empty() {
        bail!("没有可用的根元素（--root 也没给）");
    }

    std::fs::create_dir_all(&args.out)
        .with_context(|| format!("建输出目录 {}", args.out.display()))?;

    let mut merged = Report::default();
    let mut invariants = Vec::new();
    let obj_path = args.out.join("model.obj");
    let mut obj = std::io::BufWriter::new(
        std::fs::File::create(&obj_path).with_context(|| format!("建 {}", obj_path.display()))?,
    );
    let mut index_offset = 0u32;
    let mut noun_counts: BTreeMap<String, usize> = BTreeMap::new();

    for root in roots {
        let GenerationOutcome { elements, report } = generate_subtree(set.element(root))?;
        let owners = elements
            .iter()
            .map(|generated| set.element(generated.refno).owner())
            .collect::<Result<Vec<_>, _>>()?
            .into_iter()
            .flatten()
            .map(|owner| owner.refno())
            .collect();
        let keys = owner_keys(&set, &owners)?;
        for generated in &elements {
            let mesh = TriMesh::from_manifold(&generated.solid);
            let (lo, hi) = mesh.aabb().unwrap_or(([0.0; 3], [0.0; 3]));
            let inv_world = generated.world.inverse();
            let local_vertices = mesh.vertices.iter().map(|vertex| {
                (inv_world * glam::DVec3::from_array(*vertex).extend(1.0))
                    .truncate()
                    .to_array()
            });
            let (local_lo, local_hi) =
                aabb_of_points(local_vertices).unwrap_or(([0.0; 3], [0.0; 3]));
            let mut components = generated
                .solid
                .decompose()
                .into_iter()
                .map(|component| {
                    let component_mesh = TriMesh::from_manifold(&component);
                    let local_vertices = component_mesh.vertices.iter().map(|vertex| {
                        (inv_world * glam::DVec3::from_array(*vertex).extend(1.0))
                            .truncate()
                            .to_array()
                    });
                    let (component_lo, component_hi) =
                        aabb_of_points(local_vertices).unwrap_or(([0.0; 3], [0.0; 3]));
                    ComponentInvariants {
                        volume_mm3: component.volume(),
                        local_aabb_min: component_lo,
                        local_aabb_max: component_hi,
                        triangles: component_mesh.triangle_count(),
                        genus: component.genus(),
                    }
                })
                .collect::<Vec<_>>();
            components.sort_by(|a, b| b.volume_mm3.total_cmp(&a.volume_mm3));
            let (owner_desc, ordinal) = keys.get(&generated.refno).cloned().unwrap_or((None, 0));
            invariants.push(ElementInvariants {
                refno: generated.refno.to_string(),
                noun: generated.noun.clone(),
                name: generated.name.clone(),
                owner_desc,
                ordinal,
                volume_mm3: generated.solid.volume(),
                aabb_min: lo,
                aabb_max: hi,
                local_aabb_min: local_lo,
                local_aabb_max: local_hi,
                world: [
                    generated.world.x_axis.truncate().to_array(),
                    generated.world.y_axis.truncate().to_array(),
                    generated.world.z_axis.truncate().to_array(),
                    generated.world.w_axis.truncate().to_array(),
                ],
                triangles: mesh.triangle_count(),
                genus: generated.solid.genus(),
                components,
                negatives_applied: generated.negatives_applied,
                negatives_skipped: generated.negatives_skipped.len(),
            });
            *noun_counts.entry(generated.noun.clone()).or_insert(0) += 1;
            let label = match &generated.name {
                Some(name) => format!("{} {} {}", generated.refno, generated.noun, name),
                None => format!("{} {}", generated.refno, generated.noun),
            };
            index_offset = mesh.write_obj_object(&mut obj, &label, index_offset)?;
        }
        merged.merge(report);
    }
    obj.flush()?;

    let elements_path = args.out.join("elements.json");
    std::fs::write(&elements_path, serde_json::to_vec_pretty(&invariants)?)?;
    let report_path = args.out.join("report.json");
    std::fs::write(&report_path, serde_json::to_vec_pretty(&merged)?)?;
    let cache_path = args.out.join("primitive-cache.json");
    let cache_stats = e3d_model::primitive_cache_stats();
    std::fs::write(&cache_path, serde_json::to_vec_pretty(&cache_stats)?)?;

    println!("{}", merged.totals_line());
    println!("按 noun 生成: {noun_counts:?}");
    println!("基本体实例缓存: {cache_stats:?}");
    println!(
        "产物: {} / {} / {}（{} ms）",
        obj_path.display(),
        elements_path.display(),
        report_path.display(),
        t0.elapsed().as_millis()
    );

    // 产物先落盘再判账：账不平时那三份文件正是查因用的。
    if let Some(total) = index_total {
        merged.accounts_for(total).map_err(|detail| {
            anyhow::anyhow!(
                "{detail}\n有元素既没被遍历也没被属主消费——先查 report.json 的 consumed/orphans"
            )
        })?;
        println!("账已平：visited + consumed = 索引 {total}");
    }
    Ok(())
}

fn aabb_of_points(points: impl IntoIterator<Item = [f64; 3]>) -> Option<([f64; 3], [f64; 3])> {
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
    Some((lo, hi))
}

//! ams 库直读生成探针：开库 → 找根 → 生成 → OBJ + 不变量 + 总账落盘。
//!
//! 用法（路径全部显式给，不猜安装位置）：
//!
//! ```text
//! gen_ams --attlib <attlib.dat> --db <ams1112_0001> --template <模板文件或目录> \
//!         [--db-type DESI] [--sesno N] [--root w0/w1] [--out out/ams1112] \
//!         [--catalogue <ams5052_0001> ...] [--project-dir <库目录> ...] \
//!         [--route-only] [--no-instance-cache]
//! ```
//!
//! `--catalogue` 可重复，把 `SPRE`/`CATR` 指向的目录库一并开进同一个 [`DbSet`]。
//! 不给也能跑，只是隐式管身的 P 点全落在 `ppoint_db_not_open` 那本账上。
//!
//! `--project-dir` 可重复：扫这些目录（只扫本层）建「库号 → 文件」表，引用跳到
//! 没开过的库时按需自动开，模板按库类型从 `--template` 的目录派生——一个目录顶
//! 一串 `--catalogue`。两者可并用：`add_db` 先登记的文件不会被 resolver 顶掉。
//! 目录仍然是显式给的，这条不猜安装位置的边界没有动。
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
use e3d_model::catalogue::CatalogueMeshCache;
use e3d_model::db_discovery::DirectoryDbResolver;
use e3d_model::mesh::TriMesh;
use e3d_model::pipeline::{GenerationOutcome, Report, generate_subtree_with_cache};

struct Args {
    attlib: PathBuf,
    db_file: PathBuf,
    template: PathBuf,
    db_type: String,
    sesno: Option<u32>,
    root: Option<RefNo>,
    out: PathBuf,
    catalogues: Vec<PathBuf>,
    project_dirs: Vec<PathBuf>,
    route_only: bool,
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
    let mut catalogues = Vec::new();
    let mut project_dirs = Vec::new();
    let mut route_only = false;
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
            "--catalogue" => catalogues.push(PathBuf::from(value()?)),
            "--project-dir" => project_dirs.push(PathBuf::from(value()?)),
            "--route-only" => route_only = true,
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
        catalogues,
        project_dirs,
        route_only,
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

/// 独立扫出本库全部 BRAN/HANG 容器；仍由 [`generate_subtree`] 生成，避免审计探针
/// 另造一条比生产路径更容易“通过”的模型链。
fn scan_route_roots(
    db_file: &Path,
    sesno: Option<u32>,
    set: &std::sync::Arc<DbSet>,
) -> anyhow::Result<Vec<RefNo>> {
    let mut engine = match sesno {
        Some(s) => ReadOnlyEngine::open_at(db_file, s),
        None => ReadOnlyEngine::open(db_file),
    }
    .with_context(|| format!("开库扫 BRAN/HANG {}", db_file.display()))?;
    let mut roots = Vec::new();
    for scanned in engine.scan_elements(ScanTier::Header)? {
        let refno = scanned?.refno;
        if matches!(set.element(refno).element_type()?.as_str(), "BRAN" | "HANG") {
            roots.push(refno);
        }
    }
    Ok(roots)
}

#[derive(serde::Serialize)]
struct ElementInvariants {
    geometry_id: e3d_model::GeometryId,
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

#[derive(serde::Serialize)]
struct RouteRootAudit {
    refno: String,
    noun: String,
    name: Option<String>,
    direct_members: usize,
    direct_member_nouns: BTreeMap<String, usize>,
    direct_route_members: usize,
    /// Route members whose SPRE resolves to a catalogue/specification element.
    direct_route_members_with_spre: usize,
    /// Route members whose SPRE is unset/nulref. E3D can legitimately export
    /// zero geometry for a HANG made only from such placeholder PCLA members.
    direct_route_members_without_spre: usize,
    generated_elements: usize,
    generated_real_elements: usize,
    generated_implied_tubes: usize,
    skipped: usize,
    failed: usize,
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
/// 属主无名时必须递归复现 E3D 的默认显示名。例如 7997 的设备基本体位于无名
/// SUBE 下，RVM 键是 `CYLINDER 18 of SUBEQUIPMENT 20 of EQUIPMENT /X`；只看
/// 最近的命名祖先会把 20 个 SUBE 的同序号基本体错配在一起。
fn owner_display_desc(
    set: &std::sync::Arc<DbSet>,
    refno: RefNo,
    cache: &mut std::collections::HashMap<RefNo, Option<String>>,
    visiting: &mut std::collections::HashSet<RefNo>,
) -> anyhow::Result<Option<String>> {
    if let Some(desc) = cache.get(&refno) {
        return Ok(desc.clone());
    }
    if !visiting.insert(refno) {
        bail!("owner 显示名链成环: {refno}");
    }

    let element = set.element(refno);
    let noun = element.element_type()?;
    let desc = match element.stored_name()? {
        Some(name) => Some(format!("{noun} {name}")),
        None => match element.owner()? {
            None => None,
            Some(parent) => match owner_display_desc(set, parent.refno(), cache, visiting)? {
                None => None,
                Some(parent_desc) => {
                    let mut ordinal = 0usize;
                    let mut found = false;
                    for sibling in parent.members()? {
                        if sibling.element_type()? == noun {
                            ordinal += 1;
                        }
                        if sibling.refno() == refno {
                            found = true;
                            break;
                        }
                    }
                    if !found {
                        bail!("{refno} 不在属主 {} 的成员表里", parent.refno());
                    }
                    Some(format!("{noun} {ordinal} of {parent_desc}"))
                }
            },
        },
    };
    visiting.remove(&refno);
    cache.insert(refno, desc.clone());
    Ok(desc)
}

fn owner_keys(
    set: &std::sync::Arc<DbSet>,
    owners: &std::collections::HashSet<RefNo>,
) -> anyhow::Result<std::collections::HashMap<RefNo, (Option<String>, usize)>> {
    let mut keys = std::collections::HashMap::new();
    let mut desc_cache = std::collections::HashMap::new();
    for owner_ref in owners {
        let owner = set.element(*owner_ref);
        let desc = owner_display_desc(
            set,
            *owner_ref,
            &mut desc_cache,
            &mut std::collections::HashSet::new(),
        )?;
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
        // SAFETY: 参数刚解析完、任何工作线程都还没起，此刻进程里只有主线程能碰环境块。
        unsafe { std::env::set_var(e3d_model::primitive_instance::INSTANCE_CACHE_ENV, "1") };
    }
    let t0 = std::time::Instant::now();

    // 模板目录：`--template` 给目录就是它本身，给文件就是它躺的目录。
    // resolver 按库类型派生模板、显式 `--catalogue` 取 CATA 模板，都从这里出。
    let template_dir = if args.template.is_dir() {
        args.template.clone()
    } else {
        args.template
            .parent()
            .unwrap_or(Path::new("."))
            .to_path_buf()
    };
    let template_file = if args.template.is_dir() {
        template_file_for(&args.template, &args.db_type)?
    } else {
        args.template.clone()
    };

    let set = if args.project_dirs.is_empty() {
        std::sync::Arc::new(DbSet::with_attlib_file(&args.attlib)?)
    } else {
        let resolver = DirectoryDbResolver::scan(&args.project_dirs, &template_dir)?;
        for skipped in resolver.skipped() {
            println!(
                "project-dir 诊断 {}: {}",
                skipped.file.display(),
                skipped.reason
            );
        }
        println!("{}", resolver.summary());
        std::sync::Arc::new(DbSet::with_attlib_file_and_resolver(
            &args.attlib,
            Box::new(resolver),
        )?)
    };
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

    // 目录库跟设计库一个池子：P 点在 SPRE→SCOM→PTSE 链的另一头，跨库。
    // 没给 --catalogue 就别去找 CATA 模板，免得模板目录里没有它时白报一个错。
    let catalogue_template = if args.catalogues.is_empty() {
        PathBuf::new()
    } else {
        template_file_for(&template_dir, "CATA")?
    };
    for file in &args.catalogues {
        let cat_dbno = set.add_db(DbFilePin {
            file: file.clone(),
            template: catalogue_template.clone(),
            db_type: Some("CATA".to_string()),
            sesno: None,
        })?;
        println!("目录库 {} 打开为 dbno={cat_dbno}", file.display());
    }

    // 只在从头整库生成时才拿索引总数当判据；`--root` 是子树，对不上是应该的。
    let (roots, index_total) = match args.root {
        Some(refno) => (vec![refno], None),
        None if args.route_only => {
            let roots = scan_route_roots(&args.db_file, args.sesno, &set)?;
            println!("route-only 扫到 {} 个 BRAN/HANG 根", roots.len());
            (roots, None)
        }
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
    if roots.is_empty() && !args.route_only {
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
    let mut route_audit = Vec::new();
    let mut catalogue_mesh_cache = CatalogueMeshCache::default();

    for root in roots {
        let route_element = args.route_only.then(|| set.element(root));
        let route_identity = route_element
            .as_ref()
            .map(|element| {
                let mut direct_member_nouns = BTreeMap::new();
                let mut direct_route_members = 0;
                let mut direct_route_members_with_spre = 0;
                let mut direct_route_members_without_spre = 0;
                for member in element.members()? {
                    let noun = member.element_type()?;
                    *direct_member_nouns.entry(noun.clone()).or_insert(0) += 1;
                    if matches!(
                        e3d_model::category::decision(&noun).category,
                        e3d_model::category::Category::RouteMember
                    ) {
                        direct_route_members += 1;
                        if member.get_element("SPRE")?.is_some() {
                            direct_route_members_with_spre += 1;
                        } else {
                            direct_route_members_without_spre += 1;
                        }
                    }
                }
                Ok::<_, e3d_io::db_element::DbElementError>((
                    element.element_type()?,
                    element.stored_name()?,
                    direct_member_nouns.values().sum(),
                    direct_member_nouns,
                    direct_route_members,
                    direct_route_members_with_spre,
                    direct_route_members_without_spre,
                ))
            })
            .transpose()?;
        let GenerationOutcome { elements, report } =
            generate_subtree_with_cache(set.element(root), &mut catalogue_mesh_cache)?;
        if let Some((
            noun,
            name,
            direct_members,
            direct_member_nouns,
            direct_route_members,
            direct_route_members_with_spre,
            direct_route_members_without_spre,
        )) = route_identity
        {
            route_audit.push(RouteRootAudit {
                refno: root.to_string(),
                noun,
                name,
                direct_members,
                direct_member_nouns,
                direct_route_members,
                direct_route_members_with_spre,
                direct_route_members_without_spre,
                generated_elements: elements.len(),
                generated_real_elements: elements
                    .iter()
                    .filter(|element| {
                        matches!(element.geometry_id, e3d_model::GeometryId::Element { .. })
                    })
                    .count(),
                generated_implied_tubes: elements
                    .iter()
                    .filter(|element| {
                        matches!(
                            element.geometry_id,
                            e3d_model::GeometryId::ImpliedTube { .. }
                        )
                    })
                    .count(),
                skipped: report.skipped.len(),
                failed: report.failed.len(),
            });
        }
        if args.route_only {
            merged.merge(report);
            continue;
        }
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
            let mesh = generated.mesh();
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
                .components()
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
                geometry_id: generated.geometry_id.clone(),
                refno: generated.refno.to_string(),
                noun: generated.noun.clone(),
                name: generated.name.clone(),
                owner_desc,
                ordinal,
                volume_mm3: generated.volume(),
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
                genus: generated.genus(),
                components,
                negatives_applied: generated.negatives_applied,
                negatives_skipped: generated.negatives_skipped.len(),
            });
            *noun_counts.entry(generated.noun.clone()).or_insert(0) += 1;
            let label = match &generated.name {
                Some(name) => format!("{} {} {}", generated.geometry_id, generated.noun, name),
                None => format!("{} {}", generated.geometry_id, generated.noun),
            };
            index_offset = mesh.write_obj_object(&mut obj, &label, index_offset)?;
        }
        merged.merge(report);
    }
    debug_assert_eq!(
        merged.catalogue_mesh_cache,
        catalogue_mesh_cache.stats(),
        "merged per-root catalogue-cache deltas must equal final batch counters",
    );
    obj.flush()?;

    let elements_path = args.out.join("elements.json");
    std::fs::write(&elements_path, serde_json::to_vec_pretty(&invariants)?)?;
    let report_path = args.out.join("report.json");
    std::fs::write(&report_path, serde_json::to_vec_pretty(&merged)?)?;
    let cache_path = args.out.join("primitive-cache.json");
    let cache_stats = e3d_model::primitive_cache_stats();
    std::fs::write(&cache_path, serde_json::to_vec_pretty(&cache_stats)?)?;
    if args.route_only {
        std::fs::write(
            args.out.join("routes.json"),
            serde_json::to_vec_pretty(&route_audit)?,
        )?;
    }

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

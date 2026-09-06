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
    /// 给了就额外写一份 `<out>/generated.rvm`，见 [`e3d_model::rvm_write`]。
    rvm: bool,
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
    let mut rvm = false;

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
            "--rvm" => rvm = true,
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
        rvm,
    })
}

/// 解析图元自校的容差（mm）。两侧都是 f64 算到底，只差 RVM 那一次 f32 落盘，
/// 坐标量级 ~2e4 时是 ~2e-3 mm；给 0.01 留一个数量级。
const ANALYTIC_TOL_MM: f64 = 0.01;

/// 一个组件 → 一条 RVM 记录。能认成解析图元就写解析图元，否则写 FacetGroup。
///
/// **自校闸**：解析候选算出来的世界 AABB 必须与这块网格实算的一致，否则退回面片。
/// 规范体约定写错了照样能写出一份解析得开但几何是错的 RVM，这种错不该靠人眼发现。
#[allow(clippy::too_many_arguments)]
fn rvm_component_geometry(
    world: &glam::DMat4,
    rotation: [f64; 9],
    local_vertices: &[[f64; 3]],
    triangles: &[[u32; 3]],
    world_lo: [f64; 3],
    world_hi: [f64; 3],
    analytic_count: &mut usize,
    rejected_count: &mut usize,
) -> e3d_model::rvm_read::RvmGeometry {
    let candidate = e3d_model::rvm_write::box_from_local_mesh(local_vertices, triangles)
        .map(|lengths| {
            let centre = [0, 1, 2].map(|axis| {
                let lo = local_vertices.iter().fold(f64::MAX, |m, v| m.min(v[axis]));
                let hi = local_vertices.iter().fold(f64::MIN, |m, v| m.max(v[axis]));
                (lo + hi) / 2.0
            });
            e3d_model::rvm_write::box_geometry(world, lengths, centre)
        })
        .or_else(|| {
            let (radius, height, axis, centre) =
                e3d_model::rvm_write::cylinder_from_local_mesh(local_vertices, triangles)?;
            Some(e3d_model::rvm_write::cylinder_geometry(
                world, radius, height, axis, centre,
            ))
        })
        .or_else(|| {
            let torus = e3d_model::rvm_write::torus_from_local_mesh(local_vertices, triangles)?;
            Some(e3d_model::rvm_write::circular_torus_geometry(world, &torus))
        });
    let had_candidate = candidate.is_some();
    let accepted = candidate.filter(|geometry| match geometry.world_aabb_mm() {
        Some(a) => (0..3).all(|axis| {
            (a[axis] - world_lo[axis]).abs() <= ANALYTIC_TOL_MM
                && (a[axis + 3] - world_hi[axis]).abs() <= ANALYTIC_TOL_MM
        }),
        None => false,
    });
    match &accepted {
        Some(_) => *analytic_count += 1,
        None if had_candidate => *rejected_count += 1,
        None => {}
    }
    accepted.unwrap_or_else(|| {
        e3d_model::rvm_write::facet_group_from_world_triangles(
            rotation,
            world.w_axis.truncate().to_array(),
            local_vertices,
            triangles,
        )
    })
}

/// E3D 给 RVM 组用的显示名：命名元素直接是 `/NAME`，无名元素是
/// `<NOUN> <n> of <OWNER 显示名>`。
///
/// noun 这里发的是库里的短名（`FTUB`），E3D 发的是全称（`FTUBE`）。
/// **对拍不受影响**——`rvm_compare` 两侧都做 `noun4()` 截断；人读时差一截尾巴。
fn rvm_display_name(
    name: Option<&String>,
    noun: &str,
    owner_desc: Option<&String>,
    ordinal: usize,
) -> String {
    match (name, owner_desc) {
        (Some(name), _) => name.clone(),
        (None, Some(owner)) => format!("{noun} {ordinal} of {owner}"),
        (None, None) => format!("{noun} {ordinal}"),
    }
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
    /// 管身（`TUBI`）的属主是容器本身，见 [`tube_pair_key`]。
    owner_desc: Option<String>,
    /// 本元素在属主**同 noun 兄弟**里的序号，文件原序 1 起。
    /// 管身：容器内建成管身的路由顺序，1 起。
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
    desc_cache: &mut std::collections::HashMap<RefNo, Option<String>>,
) -> anyhow::Result<std::collections::HashMap<RefNo, (Option<String>, usize)>> {
    let mut keys = std::collections::HashMap::new();
    for owner_ref in owners {
        let owner = set.element(*owner_ref);
        let desc = owner_display_desc(
            set,
            *owner_ref,
            desc_cache,
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

/// 管身的对拍主键：**属主 = 容器本身**（`BRAN /名` 或无名时的递归显示名），
/// **序号 = 该容器内建成管身的路由顺序**，1 起连续。
///
/// 管身没有库记录，`GeneratedElement::refno` 填的是容器；拿它去查 [`owner_keys`]
/// 得到的是「容器在其属主里的序号」（`PIPE /1RX-345` + 支管的兄弟序），一条支管的
/// 全部管身同键，`rvm_compare` 当场以「生成侧主键重复」退出（2026-09-01 记）。
/// E3D 把管身导成独立容器 `TUBE n of BRANCH /X`（`rvm_verify import` 记成 noun `TUBI`），
/// n 是**画出来的**管身沿路由的顺序；本函数按 [`e3d_model::pipeline::build_route_tubes`]
/// 的推入序——即路由槽位顺序——发号，零长 / 未建成的槽位不占号，与之同一口径。
/// 2026-09-02 用 ams7999 `/1RCV0101-60.3-VMB-R23-R342/B1`（`test_data/rvm/1RCV0101-B1.rvm`）
/// 核过：E3D `TUBE 1..3` 与我们的 `TUBI 1..3` 逐根配对且局部 AABB 差 ≤ 0.24 mm。
fn tube_pair_key(
    set: &std::sync::Arc<DbSet>,
    container: RefNo,
    desc_cache: &mut std::collections::HashMap<RefNo, Option<String>>,
    tube_ordinals: &mut std::collections::HashMap<RefNo, usize>,
) -> anyhow::Result<(Option<String>, usize)> {
    let desc = owner_display_desc(
        set,
        container,
        desc_cache,
        &mut std::collections::HashSet::new(),
    )?;
    let ordinal = tube_ordinals.entry(container).or_insert(0);
    *ordinal += 1;
    Ok((desc, *ordinal))
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
    // 生成侧 RVM 的成员表。0 号是根容器（平移全 0，只提供层级），元素全挂在它下面。
    // E3D 会把 SITE/ZONE/PIPE 祖先一并导出来，我们暂时只给一层——`rvm_compare` 的
    // 主键只用成员名，不看祖先层数（见 `Key` / `owner_key`）。
    let mut rvm_members = vec![e3d_model::rvm_write::container_member(
        "/GENERATED",
        "/GENERATED",
        None,
    )];
    // 写成解析图元的件数 / 有可复用基本体但自校没过、退回 FacetGroup 的件数。
    let (mut rvm_analytic, mut rvm_analytic_rejected) = (0usize, 0usize);
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
        let mut desc_cache = std::collections::HashMap::new();
        let keys = owner_keys(&set, &owners, &mut desc_cache)?;
        // 管身在 `elements` 里按容器的路由槽位顺序推入（`build_route_tubes` 顺序 push、
        // 生成物不重排），逐容器计数就是路由序。
        let mut tube_ordinals = std::collections::HashMap::new();
        for generated in &elements {
            let mesh = generated.mesh();
            let (lo, hi) = mesh.aabb().unwrap_or(([0.0; 3], [0.0; 3]));
            let inv_world = generated.world.inverse();
            let to_local = |vertex: &[f64; 3]| {
                (inv_world * glam::DVec3::from_array(*vertex).extend(1.0))
                    .truncate()
                    .to_array()
            };
            let local_vertices: Vec<[f64; 3]> = mesh.vertices.iter().map(to_local).collect();
            let (local_lo, local_hi) =
                aabb_of_points(local_vertices.iter().copied()).unwrap_or(([0.0; 3], [0.0; 3]));
            let mut components = generated
                .components()
                .into_iter()
                .map(|component| {
                    let component_mesh = TriMesh::from_manifold(&component);
                    let (component_lo, component_hi) =
                        aabb_of_points(component_mesh.vertices.iter().map(to_local))
                            .unwrap_or(([0.0; 3], [0.0; 3]));
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
            let (owner_desc, ordinal) = match &generated.geometry_id {
                e3d_model::GeometryId::ImpliedTube { .. } => {
                    tube_pair_key(&set, generated.refno, &mut desc_cache, &mut tube_ordinals)?
                }
                e3d_model::GeometryId::Element { .. } => {
                    keys.get(&generated.refno).cloned().unwrap_or((None, 0))
                }
            };
            if args.rvm && !mesh.indices.is_empty() {
                let world = &generated.world;
                let rotation = [
                    world.x_axis.x,
                    world.x_axis.y,
                    world.x_axis.z,
                    world.y_axis.x,
                    world.y_axis.y,
                    world.y_axis.z,
                    world.z_axis.x,
                    world.z_axis.y,
                    world.z_axis.z,
                ];
                // **逐组件一条记录**，不是逐元素一条。E3D 就是按目录件拆的
                // （`GeneratedElement::parts` 那条注释说的就是这件事：
                // 「E3D preserves touching catalogue primitives as separate display meshes」），
                // 合成一条会让 `Cylinder×2 → FacetGroup` 这类差异永远消不掉。
                // `components()` 有 `parts` 就用它，没有就按连通性拆——单块实体拆出来还是一块。
                let component_meshes: Vec<TriMesh> = generated
                    .components()
                    .iter()
                    .map(TriMesh::from_manifold)
                    .filter(|m| !m.indices.is_empty())
                    .collect();
                // `PrimitiveInstance` 描述的是**整个元素**（`local_transform` 在元素局部系里），
                // 所以只有单组件时它才说得准；多组件的逐块自己认几何。
                let whole_element_instance = (component_meshes.len() == 1)
                    .then_some(generated.primitive_instance.as_ref())
                    .flatten()
                    .and_then(|instance| {
                        e3d_model::rvm_write::analytic_from_instance(&generated.world, instance)
                    })
                    .filter(|geometry| match geometry.world_aabb_mm() {
                        Some(a) => (0..3).all(|axis| {
                            (a[axis] - lo[axis]).abs() <= ANALYTIC_TOL_MM
                                && (a[axis + 3] - hi[axis]).abs() <= ANALYTIC_TOL_MM
                        }),
                        None => false,
                    });

                let mut geometries = Vec::with_capacity(component_meshes.len());
                if let Some(geometry) = whole_element_instance {
                    rvm_analytic += 1;
                    geometries.push(geometry);
                } else {
                    for component in &component_meshes {
                        let local: Vec<[f64; 3]> =
                            component.vertices.iter().map(to_local).collect();
                        let tris: Vec<[u32; 3]> = component.indices.as_chunks::<3>().0.to_vec();
                        let (clo, chi) = component.aabb().unwrap_or(([0.0; 3], [0.0; 3]));
                        geometries.push(rvm_component_geometry(
                            world,
                            rotation,
                            &local,
                            &tris,
                            clo,
                            chi,
                            &mut rvm_analytic,
                            &mut rvm_analytic_rejected,
                        ));
                    }
                }
                let name = rvm_display_name(
                    generated.name.as_ref(),
                    &generated.noun,
                    owner_desc.as_ref(),
                    ordinal,
                );
                rvm_members.push(e3d_model::rvm_read::RvmMember {
                    path: format!("/GENERATED/{name}"),
                    name,
                    parent: Some(0),
                    translation_mm: world.w_axis.truncate().to_array(),
                    material: 1,
                    geometries,
                });
            }
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
    if args.rvm {
        let rvm_path = args.out.join("generated.rvm");
        let file = e3d_model::rvm_read::RvmFile {
            header: e3d_model::rvm_write::generated_header(
                format!(
                    "generated by e3d-model from {} (root {})",
                    args.db_file.display(),
                    args.root
                        .map(|r| r.to_string())
                        .unwrap_or_else(|| "<whole db>".to_string()),
                ),
                "AMS",
                "/ALL",
            ),
            members: rvm_members,
        };
        file.write(&rvm_path)?;
        println!(
            "RVM: {} 个成员（解析图元 {rvm_analytic} / 自校未过退回面片 {rvm_analytic_rejected}）-> {}",
            file.members.len() - 1,
            rvm_path.display()
        );
    }
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

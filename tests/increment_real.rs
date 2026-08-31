//! 真库门：**增量说的话，必须和「两端各跑一次全量生成」说的话一样。**
//!
//! 口径是 e3d-io `index_diff_real` 那道自洽门（"A diff must say exactly what two
//! full walks say"）往上挪一格。那一层验的是索引差分对得上两次全量*遍历*；
//! 这一层验的是增量重建对得上两次全量*生成*：
//!
//! - **全量是裁判，增量是考生。** 两端各跑一遍 `generate_subtree`，按 refno 比
//!   不变量（noun / 体积 / AABB / 三角数 / 亏格 / 世界矩阵），得出「本该动的那批」。
//! - **考生可以多算，一件都不许漏。** 保守重建只是白干活，漏重建是静默缺件——
//!   本仓最高级别缺陷。所以覆盖断言是单边的：全量差集 ⊆ 增量动作集。
//! - **多算不是白纸支票。** 增量重建出来的每一件，不变量都必须与目标端全量生成的
//!   同一件逐项吻合（它们本来就该走同一条 `model_element`）；而且 upsert 数必须
//!   真的小于全量件数，否则「整库重建」也能骗过覆盖断言。
//!
//! 窗口一律取会话**链**最新的两环（不是 `(N-1, N)`——链才是先后的权威）。
//! 语料按形状挑，三个库分别压住插入 / 修改 / 删除三条路径，见 [`SAMPLES`]。
//!
//! 缺 E3D 安装或影子资产就跳过，与仓里其它真库门一个惯例。

use std::collections::{BTreeMap, BTreeSet, HashSet};
use std::path::{Path, PathBuf};
use std::sync::Arc;

use e3d_attlib::AttlibData;
use e3d_io::db_element::{DbFilePin, DbSet, template_file_for};
use e3d_io::engine::{ReadOnlyEngine, ScanTier};
use e3d_io::refno::RefNo;
use e3d_io::session::DbView;
use e3d_model::elmodl::GeneratedElement;
use e3d_model::increment::{collect_window, increment_update, plan_update};
use e3d_model::mesh::TriMesh;
use e3d_model::pipeline::generate_subtree;

const AMS_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaMarineSample\ams000";
const ATTLIB: &str = r"E:\reverse\e3d\shadow_e3d31_aps_all\attlib.dat";
const TEMPLATE_DIR: &str = r"E:\reverse\e3d\shadow_e3d31_aps_all";
const DB_TYPE: &str = "DESI";

/// 按窗口形状挑的门，三个库压住三条不同的路径（形状由
/// [`census_of_increment_windows`] 普查得来，注释里的数字是普查当时的实测）：
///
/// | 库 | 窗口 | 形状 | 压的是 |
/// |---|---|---|---|
/// | `ams8000_0001` | 263→264 | `+0 ~2 -0` | 修改：窗口极窄，最能看出增量有没有在整库重建 |
/// | `ams6890_0001` | 278→279 | `+258 ~1 -0` | 插入：新元素要能找到最近的正体属主 |
/// | `ams7999_0001` | 184→185 | `+0 ~1 -49` | 删除：删掉的数据要上卷成属主重建，删掉的正体直接移除 |
const SAMPLES: &[&str] = &["ams8000_0001", "ams6890_0001", "ams7999_0001"];

fn fixtures_present() -> bool {
    for path in [AMS_DIR, ATTLIB, TEMPLATE_DIR] {
        if !Path::new(path).exists() {
            eprintln!("跳过：缺 {path}");
            return false;
        }
    }
    true
}

/// 语料里所有 `ams<n>_0001` 设计库，按文件名排序。
fn corpus() -> Vec<PathBuf> {
    let mut files: Vec<PathBuf> = std::fs::read_dir(AMS_DIR)
        .expect("语料目录可读")
        .filter_map(|entry| {
            let entry = entry.expect("目录项");
            let name = entry.file_name().to_string_lossy().into_owned();
            let looks_like_db = name.starts_with("ams") && name.ends_with("_0001");
            (looks_like_db && entry.metadata().ok()?.is_file()).then(|| entry.path())
        })
        .collect();
    files.sort();
    files
}

/// 一个库最新的两个会话（链序，不是数字大小）。只有一个会话时 `None`。
fn newest_window(db_file: &Path) -> Option<(u32, u32)> {
    let mut view = DbView::open(db_file).ok()?;
    let sessions = view.walk_sessions().ok()?;
    (sessions.len() >= 2).then(|| (sessions[1].sesno, sessions[0].sesno))
}

fn key(refno: RefNo) -> (u32, u32) {
    (refno.word0, refno.word1)
}

/// attlib 是几十兆的共享资产，普查要开几百个池，解析一次传引用。
fn shared_attlib() -> Arc<AttlibData> {
    Arc::new(AttlibData::parse_file(ATTLIB).expect("attlib 解析得开"))
}

/// 把库钉在某个会话上开一个读取池（与 `update_ams` 同款）。
fn open_pinned(attlib: &Arc<AttlibData>, db_file: &Path, sesno: u32) -> Arc<DbSet> {
    let set = Arc::new(DbSet::new(attlib.clone()));
    let template = template_file_for(Path::new(TEMPLATE_DIR), DB_TYPE).expect("模板文件找得到");
    set.add_db(DbFilePin {
        file: db_file.to_path_buf(),
        template,
        db_type: Some(DB_TYPE.to_string()),
        sesno: Some(sesno),
    })
    .unwrap_or_else(|error| panic!("{} 钉在会话 {sesno}：{error}", db_file.display()));
    set
}

/// 树根 = owner 指向自己、无效、或不在本库索引里的元素（与 `gen_ams` 同一口径）。
fn roots_at(db_file: &Path, sesno: u32) -> Vec<RefNo> {
    let mut engine = ReadOnlyEngine::open_at(db_file, sesno).expect("开库扫根");
    let mut owners = Vec::new();
    let mut indexed = HashSet::new();
    for scanned in engine.scan_elements(ScanTier::Header).expect("扫索引") {
        let scanned = scanned.expect("索引项");
        indexed.insert(scanned.refno);
        owners.push((scanned.refno, scanned.owner));
    }
    owners
        .iter()
        .filter(|(refno, owner)| owner == refno || !owner.is_valid() || !indexed.contains(owner))
        .map(|(refno, _)| *refno)
        .collect()
}

/// 一件生成物身上可以拿来对账的量。
///
/// 不直接比网格顶点：同一条管线跑两遍本就该给出同一份网格，但拿顶点数组当判据
/// 会把「manifold 内部换了个三角化顺序」也报成回归。体积 / 包围盒 / 三角数 /
/// 亏格 / 世界矩阵这一组既钉得住几何，又不会被无害的重排绊倒。
#[derive(Debug, Clone)]
struct Invariants {
    noun: String,
    volume: f64,
    aabb_min: [f64; 3],
    aabb_max: [f64; 3],
    triangles: usize,
    genus: i32,
    world: [[f64; 3]; 4],
}

/// 相对容差。同一份输入跑两遍应当逐位相同，留这点余量是给 manifold 的布尔
/// 在不同调用序下的末位差异，不是给语义差异的。
fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-9 * a.abs().max(b.abs()).max(1.0)
}

fn triples_close(a: &[f64; 3], b: &[f64; 3]) -> bool {
    a.iter().zip(b).all(|(x, y)| close(*x, *y))
}

impl Invariants {
    fn of(generated: &GeneratedElement) -> Self {
        let mesh = TriMesh::from_manifold(&generated.solid);
        let (aabb_min, aabb_max) = mesh.aabb().unwrap_or(([0.0; 3], [0.0; 3]));
        Self {
            noun: generated.noun.clone(),
            volume: generated.solid.volume(),
            aabb_min,
            aabb_max,
            triangles: mesh.triangle_count(),
            genus: generated.solid.genus(),
            world: [
                generated.world.x_axis.truncate().to_array(),
                generated.world.y_axis.truncate().to_array(),
                generated.world.z_axis.truncate().to_array(),
                generated.world.w_axis.truncate().to_array(),
            ],
        }
    }

    fn agrees_with(&self, other: &Self) -> bool {
        self.noun == other.noun
            && self.triangles == other.triangles
            && self.genus == other.genus
            && close(self.volume, other.volume)
            && triples_close(&self.aabb_min, &other.aabb_min)
            && triples_close(&self.aabb_max, &other.aabb_max)
            && self
                .world
                .iter()
                .zip(&other.world)
                .all(|(a, b)| triples_close(a, b))
    }

    fn summary(&self) -> String {
        format!(
            "{} 体积 {:.6} 三角 {} 亏格 {}",
            self.noun, self.volume, self.triangles, self.genus
        )
    }
}

/// 把整个库在某个会话上全量生成一遍，收成 refno → 不变量。**这是裁判。**
fn full_generation(
    attlib: &Arc<AttlibData>,
    db_file: &Path,
    sesno: u32,
) -> BTreeMap<(u32, u32), Invariants> {
    let set = open_pinned(attlib, db_file, sesno);
    let mut snapshot = BTreeMap::new();
    for root in roots_at(db_file, sesno) {
        let outcome = generate_subtree(set.element(root)).unwrap_or_else(|error| {
            panic!(
                "{} 会话 {sesno} 从 {root} 全量生成：{error}",
                db_file.display()
            )
        });
        for generated in &outcome.elements {
            snapshot.insert(key(generated.refno), Invariants::of(generated));
        }
    }
    snapshot
}

/// **主门。** 三个库各跑一次「两端全量当裁判」的对账。
#[test]
fn an_increment_says_what_two_full_generations_say() {
    if !fixtures_present() {
        return;
    }
    let attlib = shared_attlib();
    for name in SAMPLES {
        let db_file = Path::new(AMS_DIR).join(name);
        let Some((base_sesno, target_sesno)) = newest_window(&db_file) else {
            panic!("{name}: 只有一个会话，挑错语料了");
        };

        // 裁判：两端各全量生成一遍，逐件比不变量。
        let base_full = full_generation(&attlib, &db_file, base_sesno);
        let target_full = full_generation(&attlib, &db_file, target_sesno);
        let appeared: BTreeSet<(u32, u32)> = target_full
            .keys()
            .filter(|refno| !base_full.contains_key(*refno))
            .copied()
            .collect();
        let vanished: BTreeSet<(u32, u32)> = base_full
            .keys()
            .filter(|refno| !target_full.contains_key(*refno))
            .copied()
            .collect();
        let changed: BTreeSet<(u32, u32)> = target_full
            .iter()
            .filter(|(refno, after)| {
                base_full
                    .get(*refno)
                    .is_some_and(|before| !before.agrees_with(after))
            })
            .map(|(refno, _)| *refno)
            .collect();

        // 考生：一次增量。
        let window = collect_window(&db_file, base_sesno, target_sesno).unwrap_or_else(|error| {
            panic!("{name}: 收集窗口 {base_sesno}→{target_sesno}：{error}")
        });
        let base_set = open_pinned(&attlib, &db_file, base_sesno);
        let target_set = open_pinned(&attlib, &db_file, target_sesno);
        let outcome = increment_update(&base_set, &target_set, &window);

        println!(
            "{name}: {base_sesno} -> {target_sesno}  全量 {} -> {} 件（新增 {} 变化 {} 消失 {}）；\
             增量 upsert {} remove {}",
            base_full.len(),
            target_full.len(),
            appeared.len(),
            changed.len(),
            vanished.len(),
            outcome.upserts.len(),
            outcome.removals.len(),
        );
        println!("  {}", outcome.report.totals_line());

        outcome
            .report
            .accounts_for()
            .unwrap_or_else(|detail| panic!("{name}: 增量账不平：{detail}"));

        let upserts: BTreeMap<(u32, u32), Invariants> = outcome
            .upserts
            .iter()
            .map(|generated| (key(generated.refno), Invariants::of(generated)))
            .collect();
        assert_eq!(
            upserts.len(),
            outcome.upserts.len(),
            "{name}: upsert 里有重复 refno，同一件被重建了两次"
        );
        let removals: BTreeSet<(u32, u32)> =
            outcome.removals.iter().map(|refno| key(*refno)).collect();

        // 一件都不许漏：全量说变了的，增量必须动过。
        for refno in appeared.iter().chain(changed.iter()) {
            assert!(
                upserts.contains_key(refno),
                "{name}: 全量说 {}/{} 在 {base_sesno}→{target_sesno} 里变了（{}），\
                 增量却没重建它",
                refno.0,
                refno.1,
                target_full[refno].summary(),
            );
        }
        for refno in &vanished {
            assert!(
                removals.contains(refno),
                "{name}: 全量说 {}/{} 在 {target_sesno} 上没了（{} 时还是 {}），\
                 增量却没把它移除",
                refno.0,
                refno.1,
                base_sesno,
                base_full[refno].summary(),
            );
        }

        // 多算可以，但算出来的必须跟全量一模一样。
        for (refno, rebuilt) in &upserts {
            let Some(judge) = target_full.get(refno) else {
                panic!(
                    "{name}: 增量重建了 {}/{}（{}），但目标端全量生成里根本没有这一件",
                    refno.0,
                    refno.1,
                    rebuilt.summary(),
                );
            };
            assert!(
                rebuilt.agrees_with(judge),
                "{name}: {}/{} 增量重建成 {}，全量生成是 {}",
                refno.0,
                refno.1,
                rebuilt.summary(),
                judge.summary(),
            );
        }

        // 覆盖断言是单边的，所以还得有人拦住「整库重建」这种作弊解。
        assert!(
            upserts.len() < target_full.len(),
            "{name}: 目标端一共 {} 件，增量重建了 {} 件——这不叫增量",
            target_full.len(),
            upserts.len(),
        );
    }
}

/// 普查语料里每个多会话库的窗口**折算成模型单元之后**是什么形状。
///
/// 按索引候选数挑库会挑到一堆空窗口：`ams6890` 的 258 个新增全是 `no_model`，
/// `ams8000` 的 2 个修改也是——真实语料里的会话大多在动非建模元素。能压住上卷、
/// 级联、重建这三条语义的，只有 `units_to_regenerate > 0` 的窗口，这个普查就是
/// 用来把它们捞出来的。跑 `plan_update` 但不执行，所以不建模、不算太贵。
#[test]
#[ignore = "普查全语料折算成模型单元后的窗口形状；run with --ignored --nocapture"]
fn census_of_model_unit_windows() {
    if !fixtures_present() {
        return;
    }
    let attlib = shared_attlib();
    let mut multi_session = 0usize;
    let mut with_units = Vec::new();
    for db_file in corpus() {
        let Some((base, target)) = newest_window(&db_file) else {
            continue;
        };
        multi_session += 1;
        let name = db_file.file_name().unwrap().to_string_lossy().into_owned();
        let Ok(window) = collect_window(&db_file, base, target) else {
            println!("{name}: {base} -> {target}  差分失败");
            continue;
        };
        if window.diff.changes.is_empty() {
            continue;
        }
        let plan = plan_update(
            &open_pinned(&attlib, &db_file, base),
            &open_pinned(&attlib, &db_file, target),
            &window,
        );
        if plan.regenerate.is_empty() && plan.remove.is_empty() {
            continue;
        }
        println!(
            "{name}: {base} -> {target}  候选 {} → 重建 {} 移除 {}（级联 {}，未归类 {}）",
            plan.report.candidates,
            plan.regenerate.len(),
            plan.remove.len(),
            plan.report.cascades.len(),
            plan.report.unresolved.len(),
        );
        with_units.push((name, plan.regenerate.len(), plan.remove.len()));
    }
    with_units.sort_by_key(|(_, regen, _)| std::cmp::Reverse(*regen));
    println!(
        "\n多会话库 {multi_session} 个，其中 {} 个窗口真的落到了模型单元；按重建量前 20：",
        with_units.len()
    );
    for (name, regen, remove) in with_units.iter().take(20) {
        println!("  {name}: 重建 {regen}，移除 {remove}");
    }
}

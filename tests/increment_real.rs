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
//! 窗口是**钉死**的：真实语料里绝大多数会话只动非建模元素，顺手取「最新两环」
//! 挑到的全是空窗口。三个窗口由 [`probe_chain_for_model_change`] 在会话链上
//! 二分出来，分别压住插入 / 修改 / 删除三条路径，见 [`SAMPLES`]。
//!
//! 缺 E3D 安装或影子资产就跳过，与仓里其它真库门一个惯例。

use std::collections::{BTreeMap, BTreeSet, HashSet};
use std::path::Path;
use std::sync::Arc;
use std::time::{Duration, Instant};

use e3d_attlib::AttlibData;
use e3d_io::db_element::{DbFilePin, DbSet, template_file_for};
use e3d_io::engine::{ReadOnlyEngine, ScanTier};
use e3d_io::refno::RefNo;
use e3d_io::session::DbView;
use e3d_model::category::is_model_unit;
use e3d_model::elmodl::GeneratedElement;
use e3d_model::increment::{collect_window, execute_plan, plan_update};
use e3d_model::ledger::ChangeKind;
use e3d_model::mesh::TriMesh;
use e3d_model::pipeline::generate_subtree;
use e3d_model::transform::local_placement;

const AMS_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaMarineSample\ams000";
const ATTLIB: &str = r"E:\reverse\e3d\shadow_e3d31_aps_all\attlib.dat";
const TEMPLATE_DIR: &str = r"E:\reverse\e3d\shadow_e3d31_aps_all";
const DB_TYPE: &str = "DESI";

/// 一个样本压的是哪条路径。**不是注释，是断言**：裁判必须真的在这个窗口上
/// 看见对应的变化，看不见就说明窗口空了（语料换了 / 会话号写错），门要当场喊。
#[derive(Clone, Copy, PartialEq, Eq)]
enum Shape {
    Inserted,
    Modified,
    Deleted,
    /// 变换级联：容器 POS/ORI 一动，子树里**索引记录根本没变**的后代也得重建。
    /// 这条压的是与 Core3D 的那处有意分歧，断言比另外三条多两级，见主门。
    Cascaded,
    /// 改挂：元素换了属主，自己的 `(POS, ORI)` 一个字没动。
    ///
    /// 世界矩阵是沿 owner 链折叠出来的，链换一条就全变，而老实现的判据是两端比
    /// `(POS, ORI)`，对这种改动一律判「没漂移」。
    ///
    /// **这条样本压的是「L2 看得见改挂」，不是「结果因此不同」——后者本语料证不了。**
    /// `probe_chain_for_reparent` 扫过全部 443 个库，一共只有 22 个改挂窗口
    /// （ams8000 十八个、ams7999 四个），逐个查下来：
    ///
    /// - 22 个的本级 `(POS, ORI)` **全都没动**，老判据的盲区是真的、命中率 100%；
    /// - 但被改挂的要么是 `BOX`（自己就是模型单元，索引已经点了它的名，
    ///   target 端上卷照样重建它），要么是 `FTUB`（自己不是单元、子树里没有单元、
    ///   旧属主那侧也没有单元，两种实现都不产生动作）。
    ///
    /// 也就是说：**没有一个语料窗口能把修好的实现和没修的区分开。** 修复的正确性
    /// 靠的是 `world_matrix` 沿 owner 链折叠这条构造性事实，以及
    /// `element_diff` 里那条谓词单元测试；本样本只钉住「真实数据里的改挂确实是
    /// 老判据看不见的那个形状」。真正会漏几何的形状（改挂一个自己不是单元、
    /// 底下却挂着单元的容器）在这份语料里一次都没发生过。
    Reparented,
}

impl Shape {
    fn label(self) -> &'static str {
        match self {
            Shape::Inserted => "插入",
            Shape::Modified => "修改",
            Shape::Deleted => "删除",
            Shape::Cascaded => "级联",
            Shape::Reparented => "改挂",
        }
    }
}

/// 一个钉死的窗口：库、两端会话、以及它压的是哪条路径。
struct Sample {
    db: &'static str,
    base: u32,
    target: u32,
    shape: Shape,
}

/// 四个窗口压住插入 / 修改 / 删除 / 级联四条路径。
///
/// **窗口必须钉死，不能取「最新两环」**：真实语料里绝大多数会话只动非建模元素，
/// 顺手取最新两环挑到的全是空窗口——ams8000 最新窗口的 2 个修改、ams6890 最新窗口的
/// 258 个新增，折算到模型单元之后全是 `no_model`，跑了等于没跑。下面这些窗口是
/// [`probe_chain_for_model_change`] / [`probe_chain_profile`] / [`probe_chain_for_cascade`]
/// 在链上实测出来的，注释里的件数是探针当时的读数：
///
/// | 库 | 窗口 | 全量件数 | 压的是 |
/// |---|---|---|---|
/// | `ams7999_0001` | 45→46 | 2098 → 2101 | 插入：新增的正体要自己成为单元，一件不许漏 |
/// | `ams8000_0001` | 255→256 | 153 → 153 | 修改：件数不动、内容变了，最能看出有没有在整库重建 |
/// | `ams1112_0001` | 721→722 | 7824 → 4476 | 删除：3348 件正体一次性没掉，删除要跟着上卷 |
/// | `ams8000_0001` | 195→196 | 153 → 153 | 级联：索引里只有一个 EQUI 候选，变的却是它子树里的后代 |
const SAMPLES: &[Sample] = &[
    Sample {
        db: "ams7999_0001",
        base: 45,
        target: 46,
        shape: Shape::Inserted,
    },
    Sample {
        db: "ams8000_0001",
        base: 255,
        target: 256,
        shape: Shape::Modified,
    },
    Sample {
        db: "ams1112_0001",
        base: 721,
        target: 722,
        shape: Shape::Deleted,
    },
    // 索引差分在这一环上只点了一个名字：EQUI `24384/26186` 的记录变了。它自己不产
    // 正体，真正要重建的是它子树里那件后代——而那件后代的索引记录**一个字节都没动**。
    // 换句话说，把级联那段代码删掉，这个窗口的增量会一件都不重建，而全量说变了一件。
    Sample {
        db: "ams8000_0001",
        base: 195,
        target: 196,
        shape: Shape::Cascaded,
    },
    // `BOX 24384/25802` 从属主 `24384/25801` 挪到 `24384/25803`，自己的 POS/ORI
    // 一个字没动 —— 判据只比 `(POS, ORI)` 的老实现在这里说「没漂移」。
    // 它压的是 L2 的检出，不是结果差异，理由见 `Shape::Reparented`。
    Sample {
        db: "ams8000_0001",
        base: 45,
        target: 46,
        shape: Shape::Reparented,
    },
];

fn fixtures_present() -> bool {
    for path in [AMS_DIR, ATTLIB, TEMPLATE_DIR] {
        if !Path::new(path).exists() {
            eprintln!("跳过：缺 {path}");
            return false;
        }
    }
    true
}

/// 一个库的会话链，**新 → 旧**（链序才是先后的权威，会话号不保证连号）。
fn chain_of(db_file: &Path) -> Option<Vec<u32>> {
    let mut view = DbView::open(db_file).ok()?;
    let sessions = view.walk_sessions().ok()?;
    (sessions.len() >= 2).then(|| sessions.iter().map(|session| session.sesno).collect())
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

type Snapshot = BTreeMap<(u32, u32), Invariants>;

/// 两份快照逐件一致（件数、键集、不变量都算）。
fn snapshots_agree(before: &Snapshot, after: &Snapshot) -> bool {
    before.len() == after.len()
        && before.iter().all(|(refno, left)| {
            after
                .get(refno)
                .is_some_and(|right| left.agrees_with(right))
        })
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
    for sample in SAMPLES {
        let Sample {
            db: name,
            base: base_sesno,
            target: target_sesno,
            shape,
        } = *sample;
        let db_file = Path::new(AMS_DIR).join(name);

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

        // 空窗口自检：这道门此前挑窗口挑的是索引候选数，三个样本折算到模型单元
        // 之后全是空的，全绿了几轮什么都没验。样本声明压哪条路径，裁判就必须真的
        // 在那条路径上看见东西。
        let exercised = match shape {
            Shape::Inserted => appeared.len(),
            Shape::Modified | Shape::Cascaded | Shape::Reparented => changed.len(),
            Shape::Deleted => vanished.len(),
        };
        assert!(
            exercised > 0,
            "{name}: 窗口 {base_sesno}→{target_sesno} 声明压「{}」，\
             两端全量生成却一件都没{}——这道门在空转，\
             用 probe_chain_for_model_change 重新挑窗口",
            shape.label(),
            shape.label(),
        );

        // 考生：一次增量。
        let window = collect_window(&db_file, base_sesno, target_sesno).unwrap_or_else(|error| {
            panic!("{name}: 收集窗口 {base_sesno}→{target_sesno}：{error}")
        });
        let base_set = open_pinned(&attlib, &db_file, base_sesno);
        let target_set = open_pinned(&attlib, &db_file, target_sesno);
        // 分两步而不是 `increment_update`：改挂那条断言要查 L3 的逐条明细，
        // 而计划一旦交给 `execute_plan` 就消费掉了。
        let plan = plan_update(&base_set, &target_set, &window);
        let reparented: Vec<RefNo> = plan
            .ledger
            .of_kind(ChangeKind::Reparented)
            .map(|entry| entry.refno)
            .collect();
        let outcome = execute_plan(&target_set, plan);

        println!(
            "{name}（{}）: {base_sesno} -> {target_sesno}  全量 {} -> {} 件\
             （新增 {} 变化 {} 消失 {}）；增量 upsert {} remove {}",
            shape.label(),
            base_full.len(),
            target_full.len(),
            appeared.len(),
            changed.len(),
            vanished.len(),
            outcome.upserts.len(),
            outcome.removals.len(),
        );
        println!("  {}", outcome.report.totals_line());
        println!("  {}", outcome.report.changes_line());

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

        // 反向这条更要命：多重建只是白干活，多移除是把一件真实存在的几何从模型集里
        // 抹掉。所以 Remove 虽然是幂等的「确保不存在」，也不许落在目标端还活着的件上。
        for refno in &removals {
            assert!(
                !target_full.contains_key(refno),
                "{name}: 增量把 {}/{} 移除了，但目标端全量生成里它还在（{}）",
                refno.0,
                refno.1,
                target_full[refno].summary(),
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

        // 级联样本还要再证一件事：上面那条「一件都不许漏」分不清后代是**被级联捞回来的**
        // 还是**它自己也在候选里**。差分只点了容器的名字，后代的索引记录一个字节没动，
        // 所以这里要求至少有一件「变了但没有候选」的——那正是级联唯一的存在理由。
        if shape == Shape::Cascaded {
            assert!(
                !outcome.report.cascades.is_empty(),
                "{name}: 样本声明压级联，plan_update 却一次 POS/ORI 漂移都没检出",
            );
            let named_by_index: BTreeSet<(u32, u32)> = window
                .diff
                .changes
                .iter()
                .map(|candidate| key(candidate.refno()))
                .collect();
            let silent: Vec<(u32, u32)> = changed
                .iter()
                .filter(|refno| !named_by_index.contains(*refno))
                .copied()
                .collect();
            assert!(
                !silent.is_empty(),
                "{name}: 窗口 {base_sesno}→{target_sesno} 里每一件变了的都自带索引候选，\
                 级联就算整段删掉也不会漏件——这个窗口证明不了级联，\
                 用 probe_chain_for_cascade 换一个",
            );
            println!(
                "  级联：{} 处 POS/ORI 漂移；{} 件后代索引记录没变，只有级联捞得回来",
                outcome.report.cascades.len(),
                silent.len(),
            );
        }

        // 改挂样本要再证一件事：L2 真的看见了这条改挂，而且它正是老判据看不见的
        // 那个形状——两端 `(POS, ORI)` 逐项相等。
        //
        // 这里**不**断言「不修就会漏几何」：全语料 443 个库只有 22 个改挂，
        // 没有一个能把两种实现区分开（见 `Shape::Reparented` 的文档）。断言写成
        // 它证不了的样子，比不断言更糟。
        if shape == Shape::Reparented {
            assert!(
                !reparented.is_empty(),
                "{name}: 样本声明压改挂，L2 却一个 owner 变更都没差分出来",
            );
            let held_still: Vec<RefNo> = reparented
                .iter()
                .filter(|refno| {
                    let before = local_placement(&base_set.element(**refno)).ok();
                    let after = local_placement(&target_set.element(**refno)).ok();
                    before == after
                })
                .copied()
                .collect();
            assert!(
                !held_still.is_empty(),
                "{name}: 窗口 {base_sesno}→{target_sesno} 里每个改挂元素的 POS/ORI 也都变了，\
                 只比 (POS, ORI) 的老判据照样抓得到——这个窗口证明不了改挂，\
                 用 probe_chain_for_reparent 换一个",
            );
            println!(
                "  改挂：{} 处 owner 变更，其中 {} 处本级 POS/ORI 纹丝没动（{}）",
                reparented.len(),
                held_still.len(),
                held_still
                    .iter()
                    .map(RefNo::to_string)
                    .collect::<Vec<_>>()
                    .join(" "),
            );
        }
    }
}

/// 在一个库的会话链上**二分**出「模型真的变过」的那一环。
///
/// 挑窗口一度是按索引候选数挑的，挑出来全是空窗口：`ams6890` 最新窗口的 258 个
/// 新增全落 `no_model`，`ams8000` 的 2 个修改也是——真实语料里的会话大多在动
/// 非建模元素。改按模型单元折算（对全语料跑 `plan_update`）也不行：443 个库、
/// 2.4 GB，逐候选走 owner 链是随机读，实测半小时只啃得动几个库。
///
/// 所以这里换成拿**主门那位裁判**去二分：链头与链尾各全量生成一次，确认这个库的
/// 模型这辈子变过，再对半收敛到相邻两环。代价是 `2 + log2(链长)` 次全量生成
/// （几百环的链也就十来次），换来的窗口一定不是空窗口。[`SAMPLES`] 里的会话号
/// 就是这么定出来的。
///
/// 默认跑 [`PROBE_LIBS`]，`PROBE_LIBS=ams1234_0001,ams5678_0001` 可临时换一批。
#[test]
#[ignore = "二分会话链找出模型真的变过的窗口；run with --ignored --nocapture"]
fn probe_chain_for_model_change() {
    if !fixtures_present() {
        return;
    }
    let attlib = shared_attlib();
    for name in probe_libs() {
        let db_file = Path::new(AMS_DIR).join(&name);
        let Some(chain) = chain_of(&db_file) else {
            println!("\n=== {name}: 开不了库，或链上只有一个会话 ===");
            continue;
        };
        let oldest = chain.len() - 1;
        println!(
            "\n=== {name}: 链 {} 环，{} … {} ===",
            chain.len(),
            chain[0],
            chain[oldest],
        );

        let (mut newer_full, cost) = timed_full_generation(&attlib, &db_file, chain[0]);
        println!(
            "  链头 {}：全量 {} 件，{:.1}s",
            chain[0],
            newer_full.len(),
            cost.as_secs_f64(),
        );
        let (oldest_full, cost) = timed_full_generation(&attlib, &db_file, chain[oldest]);
        println!(
            "  链尾 {}：全量 {} 件，{:.1}s",
            chain[oldest],
            oldest_full.len(),
            cost.as_secs_f64(),
        );
        if snapshots_agree(&newer_full, &oldest_full) {
            println!("  → 整条链上模型一件没变过，这个库挑不出窗口");
            continue;
        }

        // 不变量：`chain[newer]` 与 `chain[older]` 两端的全量结果一定不一致。
        let (mut newer, mut older) = (0usize, oldest);
        while older - newer > 1 {
            let mid = newer + (older - newer) / 2;
            let (mid_full, cost) = timed_full_generation(&attlib, &db_file, chain[mid]);
            let agrees = snapshots_agree(&newer_full, &mid_full);
            println!(
                "    链位 {mid}（会话 {}）：全量 {} 件，{:.1}s，与 {} {}",
                chain[mid],
                mid_full.len(),
                cost.as_secs_f64(),
                chain[newer],
                if agrees { "一致" } else { "不一致" },
            );
            if agrees {
                newer = mid;
                newer_full = mid_full;
            } else {
                older = mid;
            }
        }
        println!(
            "  → 窗口 {} -> {}：模型就是在这一环上变的",
            chain[older], chain[newer],
        );
    }
}

/// 在会话链上逐环找**变换级联**窗口：容器 POS/ORI 一动，子树里索引记录根本
/// 没变的后代也必须跟着重建（见 `increment` 模块文档「与 Core3D 的一处有意分歧」）。
///
/// 二分那套在这里不好使。[`probe_chain_for_model_change`] 只保证收敛到「模型变过」
/// 的一环，而级联要的条件更窄：得有某个 **Modified 候选自己的 `(POS, ORI)` 漂了**。
/// 链上绝大多数环都不满足，二分收敛到的那一环通常是插入或属性改动。
///
/// 所以这里逐环扫，但**一件几何都不建**：只做 `collect_window` + `plan_update`
/// （读两端 DbSet 走 owner 链，不进 `model_element`）。候选数超过
/// `PROBE_MAX_CANDIDATES`（默认 4000）的窗口直接跳过——ams1112 那种 24674 候选的
/// 窗口逐个走 owner 链要好几分钟，而级联窗口通常只有个位数候选，不值得为它等。
#[test]
#[ignore = "逐环扫会话链，找 POS/ORI 漂移触发级联的窗口；run with --ignored --nocapture"]
fn probe_chain_for_cascade() {
    if !fixtures_present() {
        return;
    }
    let attlib = shared_attlib();
    let cap = probe_max_candidates();
    for name in probe_libs() {
        let db_file = Path::new(AMS_DIR).join(&name);
        let Some(chain) = chain_of(&db_file) else {
            println!("\n=== {name}: 开不了库，或链上只有一个会话 ===");
            continue;
        };
        println!(
            "\n=== {name}: 链 {} 环（新 → 旧），候选上限 {cap} ===",
            chain.len()
        );
        let started = Instant::now();
        let (mut hits, mut skipped) = (0usize, 0usize);
        // 链是新 → 旧，所以一对里前者是 target、后者是 base。
        for pair in chain.windows(2) {
            let (target_sesno, base_sesno) = (pair[0], pair[1]);
            let window = match collect_window(&db_file, base_sesno, target_sesno) {
                Ok(window) => window,
                Err(error) => {
                    println!("  {base_sesno}→{target_sesno}: 收窗口失败：{error}");
                    continue;
                }
            };
            // 级联只可能由 Modified 候选触发：插入的元素在 base 端读不出旧位置，
            // 删除的元素在 target 端没了子树。没有修改就一定不会有漂移。
            if window.diff.tally.modified == 0 {
                continue;
            }
            let candidates = window.diff.changes.len();
            if candidates > cap {
                skipped += 1;
                continue;
            }
            let base_set = open_pinned(&attlib, &db_file, base_sesno);
            let target_set = open_pinned(&attlib, &db_file, target_sesno);
            let plan = plan_update(&base_set, &target_set, &window);
            if plan.report.cascades.is_empty() {
                continue;
            }
            hits += 1;
            println!(
                "  ★ {base_sesno}→{target_sesno}: {}",
                plan.report.totals_line()
            );
            for incident in &plan.report.cascades {
                println!(
                    "      {} {} — {}",
                    incident.refno, incident.noun, incident.detail
                );
            }
        }
        println!(
            "  → {hits} 个级联窗口，{skipped} 环因候选超上限跳过，耗时 {:.1}s",
            started.elapsed().as_secs_f64(),
        );
    }
}

/// 在会话链上逐环找**改挂**窗口：某个元素换了属主。
///
/// 这条压的是 P0-1。改挂的元素自己的 `(POS, ORI)` 可以一个字不动，而它的世界矩阵
/// 是沿 owner 链折叠出来的——链换了一条，整棵子树的世界矩阵全变。老实现的
/// `placement_drifted` 只比两端的 `(POS, ORI)`，对这种改动一律判「没漂移」。
///
/// 找它不能靠件数二分（[`probe_chain_for_model_change`] 收敛到的多半是插入或
/// 属性改动），得直接问 L2：`plan_update` 之后看 `report.changes.reparented`。
/// 与 [`probe_chain_for_cascade`] 一样只做 `collect_window` + `plan_update`，
/// 一件几何都不建，并按 `PROBE_MAX_CANDIDATES` 跳过大窗口。
#[test]
#[ignore = "逐环扫会话链，找改挂（属主变更）窗口；run with --ignored --nocapture"]
fn probe_chain_for_reparent() {
    if !fixtures_present() {
        return;
    }
    let attlib = shared_attlib();
    let cap = probe_max_candidates();
    for name in probe_libs() {
        let db_file = Path::new(AMS_DIR).join(&name);
        let Some(chain) = chain_of(&db_file) else {
            println!("\n=== {name}: 开不了库，或链上只有一个会话 ===");
            continue;
        };
        println!(
            "\n=== {name}: 链 {} 环（新 → 旧），候选上限 {cap} ===",
            chain.len()
        );
        let started = Instant::now();
        let (mut hits, mut skipped) = (0usize, 0usize);
        for pair in chain.windows(2) {
            let (target_sesno, base_sesno) = (pair[0], pair[1]);
            let window = match collect_window(&db_file, base_sesno, target_sesno) {
                Ok(window) => window,
                Err(error) => {
                    println!("  {base_sesno}→{target_sesno}: 收窗口失败：{error}");
                    continue;
                }
            };
            // 改挂只落在 Modified 候选上：属主在记录头里，改挂必然重写记录，
            // 而两端都还在的元素才谈得上「换了属主」。
            if window.diff.tally.modified == 0 {
                continue;
            }
            if window.diff.changes.len() > cap {
                skipped += 1;
                continue;
            }
            let base_set = open_pinned(&attlib, &db_file, base_sesno);
            let target_set = open_pinned(&attlib, &db_file, target_sesno);
            let plan = plan_update(&base_set, &target_set, &window);
            if plan.report.changes.reparented == 0 {
                continue;
            }
            hits += 1;
            println!(
                "  ★ {base_sesno}→{target_sesno}: {}",
                plan.report.changes_line()
            );
            for entry in plan.ledger.of_kind(ChangeKind::Reparented) {
                // 两笔事实。第一笔：改挂的元素自己的 (POS, ORI) 动了没有——没动
                // 才谈得上老判据会说「没漂移」。
                let before = local_placement(&base_set.element(entry.refno)).ok();
                let after = local_placement(&target_set.element(entry.refno)).ok();
                let held_still = before == after;
                // 第二笔更要紧：这条改挂**改不改得了结果**。元素自己就是单元时，
                // 索引已经点了它的名，target 端上卷照样重建它，修不修一个样；
                // 只有「自己不是单元，但子树里有单元 / 旧属主那边有单元」才是
                // 老实现真的会漏几何的形状。
                let itself = is_model_unit(&entry.noun);
                let below = unit_count_below(&target_set, entry.refno);
                let old_side = unit_above(&base_set, entry.refno).filter(|_| !itself);
                let discriminating = !itself && (below > 0 || old_side.is_some());
                println!(
                    "      {} {} — {}；本级 POS/ORI {}；自己是单元 {}；子树内单元 {below}；\
                     旧属主侧单元 {}{}",
                    entry.refno,
                    entry.noun,
                    entry.detail,
                    if held_still { "没动" } else { "也变了" },
                    if itself { "是" } else { "否" },
                    old_side
                        .map(|unit| unit.to_string())
                        .unwrap_or_else(|| "无".into()),
                    if discriminating && held_still {
                        "  ←★ 老实现会漏几何"
                    } else {
                        ""
                    },
                );
            }
        }
        println!(
            "  → {hits} 个改挂窗口，{skipped} 环因候选超上限跳过，耗时 {:.1}s",
            started.elapsed().as_secs_f64(),
        );
    }
}

/// 逐环全量生成，把一整条会话链上的模型件数画出来。
///
/// 二分只交得出**一个**边界，挑不了「我要一个有插入的窗口」。链短的库（ams1112
/// 只有 45 环）直接逐环跑一遍最省事：件数在哪一环涨、哪一环跌，一眼就看得见。
/// 链长的库别用这个，用 [`probe_chain_for_model_change`]。
#[test]
#[ignore = "逐环画出一条会话链上的模型件数；run with --ignored --nocapture"]
fn probe_chain_profile() {
    if !fixtures_present() {
        return;
    }
    let attlib = shared_attlib();
    for name in probe_libs() {
        let db_file = Path::new(AMS_DIR).join(&name);
        let Some(chain) = chain_of(&db_file) else {
            println!("\n=== {name}: 开不了库，或链上只有一个会话 ===");
            continue;
        };
        println!("\n=== {name}: 链 {} 环（新 → 旧）===", chain.len());
        let mut previous: Option<usize> = None;
        for (at, sesno) in chain.iter().enumerate() {
            let (snapshot, cost) = timed_full_generation(&attlib, &db_file, *sesno);
            let delta = match previous {
                // 链是新→旧，所以「上一环」在更新的一侧：本环 → 上一环才是时间正向。
                Some(newer) => format!("{:+}", newer as i64 - snapshot.len() as i64),
                None => "-".to_string(),
            };
            println!(
                "  链位 {at:>3} 会话 {sesno:>5}：{:>6} 件（往新一环 {delta}），{:.1}s",
                snapshot.len(),
                cost.as_secs_f64(),
            );
            previous = Some(snapshot.len());
        }
    }
}

/// `root` 子树内（不含自身）的模型单元数。改挂判据的判别力就看它：
/// 元素自己不是单元、子树里却有，老实现漏的就是这些。
fn unit_count_below(set: &Arc<DbSet>, root: RefNo) -> usize {
    let mut found = 0usize;
    let mut seen: BTreeSet<(u32, u32)> = BTreeSet::from([key(root)]);
    let mut stack = vec![set.element(root)];
    while let Some(element) = stack.pop() {
        let Ok(members) = element.members() else {
            continue;
        };
        for member in members {
            if !seen.insert(key(member.refno())) {
                continue;
            }
            if member.element_type().is_ok_and(|noun| is_model_unit(&noun)) {
                found += 1;
            }
            stack.push(member);
        }
    }
    found
}

/// 沿 owner 链找最近的模型单元（不含自身）。改挂时旧属主那一侧的单元同样要重建
/// （core 的 `elementIncluded` 记的就是旧属主），只上卷 target 端永远看不见它。
fn unit_above(set: &Arc<DbSet>, from: RefNo) -> Option<RefNo> {
    let mut cursor = set.element(from).owner().ok().flatten();
    for _ in 0..128 {
        let element = cursor?;
        if element
            .element_type()
            .is_ok_and(|noun| is_model_unit(&noun))
        {
            return Some(element.refno());
        }
        cursor = element.owner().ok().flatten();
    }
    None
}

/// [`probe_chain_for_model_change`] 默认探的库，覆盖大中小三种规模。
const PROBE_LIBS: &[&str] = &["ams8000_0001", "ams6890_0001", "ams1112_0001"];

fn probe_libs() -> Vec<String> {
    match std::env::var("PROBE_LIBS") {
        Ok(list) => list
            .split(',')
            .map(str::trim)
            .filter(|name| !name.is_empty())
            .map(str::to_owned)
            .collect(),
        Err(_) => PROBE_LIBS.iter().map(|name| (*name).to_owned()).collect(),
    }
}

/// [`probe_chain_for_cascade`] 跳过大窗口的阈值。逐候选走 owner 链是随机读，
/// 万级候选的窗口一个就要几分钟，而级联窗口通常只有个位数候选。
fn probe_max_candidates() -> usize {
    std::env::var("PROBE_MAX_CANDIDATES")
        .ok()
        .and_then(|raw| raw.trim().parse().ok())
        .unwrap_or(4000)
}

fn timed_full_generation(
    attlib: &Arc<AttlibData>,
    db_file: &Path,
    sesno: u32,
) -> (Snapshot, Duration) {
    let started = Instant::now();
    let snapshot = full_generation(attlib, db_file, sesno);
    (snapshot, started.elapsed())
}

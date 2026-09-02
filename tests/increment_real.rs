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
//! 缺 E3D 安装或影子资产**默认红**（[`fixtures_present`]），与仓里探针类真库门的
//! 软跳过惯例有意不同：这道门是增量正确性的唯一裁判，缺席必须出声。
//! 确属无语料环境，显式设 `E3D_SKIP_REAL_GATES=1` 换回跳过。

use std::collections::{BTreeMap, BTreeSet, HashSet};
use std::path::{Path, PathBuf};
use std::sync::{Arc, OnceLock};
use std::time::{Duration, Instant};

use e3d_attlib::AttlibData;
use e3d_io::db_element::{DbFilePin, DbFileResolver, DbSet, template_file_for};
use e3d_io::engine::{ReadOnlyEngine, ScanTier};
use e3d_io::refno::RefNo;
use e3d_io::session::DbView;
use e3d_model::category::is_model_unit;
use e3d_model::db_discovery::DirectoryDbResolver;
use e3d_model::elmodl::{GeneratedElement, GeometryId};
use e3d_model::increment::{collect_window, execute_plan, plan_update};
use e3d_model::ledger::ChangeKind;
use e3d_model::mesh::TriMesh;
use e3d_model::pipeline::generate_subtree;
use e3d_model::transform::local_placement;

const AMS_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaMarineSample\ams000";
/// 跨项目目录库（7002 / 7320 在这里，见 `docs/2026-09-01-next-step-development-plan.md` G0-5）。
const ACP_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaCatalogue\acp000";
/// 库 7000 在 ams000 与 acp000 各有一份常规命名的主文件，扫目录的 resolver 按设计拒仲裁；
/// 本项目那份要显式钉（与 `gen_ams --catalogue ams7000` 同一口径）。
const PROJECT_CATALOGUE: &str = "ams7000_0001";
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
    /// 电缆桥架：托盘 `CTRAY` 的几何在两端全量里变了（新增 / 变化 / 消失任一）。
    ///
    /// 它压的是两条别的样本压不到的腿：① `CTRAY` 是模型单元（`is_model_unit`，
    /// 2026-09-02 之前是 false，增量对它一律 `no_model`）；② 桥架件的放置不读自己的
    /// POS/ORI，读 `CWBRAN` 名下 RPATH/POINTR 与 POSREF 链——不在 owner 链上的输入，
    /// 靠 `cable_tray_branch_of` 的整条分支级联才够得着。目录库不进门这条腿无从验证：
    /// 托盘几何全走目录链（SPRE→TABITE→PRTREF→GPART→CATR→SCOM→GMRE）。
    CableTray,
    /// 管路分支：BRAN 名下的路由成员（ELBO/BEND/FTUB/…目录实体）或隐式管身
    /// （[`GeometryId::ImpliedTube`]）在两端全量里变了。
    ///
    /// 它压的是派生腿：成员一动，`nearest_unit` 既要把成员本身收进 `regenerate`，
    /// 又要把它所在的 BRAN 收进 `regenerate_derived`（`mark_derived`），执行侧
    /// `route_tubes_of` 整链重算、base 端 `route_slot_keys` 的旧键做差发 Remove。
    /// 目录库不进门时这条腿在门上没有判别力（管身外径要目录、成员实体要目录，
    /// 两端全量一根都建不出，`implied_tubes.built = 0`）。
    Branch,
    /// 整条 BRAN 被删：容器自己是 Deleted 候选，target 端没有它。
    ///
    /// 派生腿的另一半：`execute_plan` 对 target 端已不存在的容器不重算链
    /// （`derived_retired`），而是把 base 端 `route_slot_keys` 收来的全部旧管身键发
    /// Remove；名下的路由成员各自出 Remove。漏掉这一半的表现是「BRAN 删了，管身还挂在
    /// 模型集里」——Remove 是幂等的「确保不存在」，多发无害，少发就是陈旧几何。
    BranchRemoved,
}

impl Shape {
    fn label(self) -> &'static str {
        match self {
            Shape::Inserted => "插入",
            Shape::Modified => "修改",
            Shape::Deleted => "删除",
            Shape::Cascaded => "级联",
            Shape::Reparented => "改挂",
            Shape::CableTray => "桥架",
            Shape::Branch => "管路分支",
            Shape::BranchRemoved => "删管路分支",
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
/// | `ams251142_0001` | 2→9 | 0 → 全库（CTRAY 128） | 桥架：托盘经目录链成件，每只都得自己成为单元（目录库进门后才看得见；样本随 `cable_tray` 模块入库，见 `SAMPLES` 末尾） |
/// | `ams8000_0001` | 249→250 | 管身 336 → 336（内容变） | 管路分支：成员挪位，BRAN 进派生集、隐式管身按两站点键做差重算（目录库进门后才看得见） |
/// | `ams7997_0001` | 11→12 | 删 BRAN `24381/322`（3 成员、4 管身槽位） | 删管路分支：容器退休，base 端旧管身键全部 Remove（目录库进门后才看得见） |
///
/// 件数是 DESI-only 时代的读数；目录库进门（2026-09-02）后裁判还看见路由成员与管身，
/// ams7999 是 13283 件、ams8000 是 2853 件，窗口两端的差集不变。
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
    // 电缆桥架库 ams251142（`ZONE → CWAY → CWBRAN → {RPATH→POINTR, CTMTRL→CTSTRA/CTBEND/…→CTRAY}`）
    // 链上只有 4 环（1, 2, 9, 10）：128 只托盘全部在 2→9 这一环里出现，之后再没动过
    // （`PROBE_NOUN=CTRAY probe_chain_for_model_change` 扫过 20 本 CWAY 库，形状全一样：
    // 一次性导入，没有一本有「改一个路径点」那种窗口）。它压的是**托盘的插入路径**：
    // 128 个 CTRAY 每个都得自己成为单元被重建——`is_model_unit(CTRAY)` 为 false 时它们
    // 全落 `no_model`，全量说新增 128 件、增量一件不建。分支级联那条腿（改 RPATH/POINTR）
    // 本语料没有窗口可压，留待有桥架编辑历史的库。
    //
    // 样本本身随 `cable_tray` 模块（桥架件放置 + CTRAY 目录臂）一起入库：没有它，
    // 两端全量都建不出托盘，这道门会按「空窗口」当场喊。届时只需加：
    // `Sample { db: "ams251142_0001", base: 2, target: 9, shape: Shape::CableTray }`。
    Sample {
        db: "ams251142_0001",
        base: 2,
        target: 9,
        shape: Shape::CableTray,
    },
    // 管路库 ams8000：`PROBE_NOUN=TUBI probe_chain_for_model_change` 在 266 环的链上二分，
    // 隐式管身（336 根）在 249→250 这一环变了内容、根数没变——某个 BRAN 里的构件挪了位 /
    // 换了件，管身跟着重算。链上 134/200/233/242/246/248 各环的管身也都与链头不同，
    // 管路窗口并不稀缺，这里只钉一个。目录库不进门时两端一根管都建不出，这条门开不了。
    Sample {
        db: "ams8000_0001",
        base: 249,
        target: 250,
        shape: Shape::Branch,
    },
    // 支撑 / 通风库 ams7997：`probe_chain_for_branch_delete` 在 106 环的链上找到 6 个删 BRAN 的环，
    // 取最小的一个——11→12 删掉 BRAN `24381/322`（名下 3 个路由成员，base 端 4 个管身槽位），
    // 全窗只有 5 个候选。它压的是容器**退休**：target 端没有这条 BRAN，增量不重算链，
    // 把 base 端收来的 4 把旧键全部发 Remove。ams7999 86→87（删 `24383/101244`，20 个成员）
    // 是同类里更大的一个，需要时可加。
    Sample {
        db: "ams7997_0001",
        base: 11,
        target: 12,
        shape: Shape::BranchRemoved,
    },
];

/// `GATE_ONLY=ams8000_0001:249:250`（逗号分隔多个）只跑点名的窗口：带目录库的裁判一窗
/// 要几十秒，调一个新样本时不必把六窗全跑一遍。不设就全跑。
fn gate_only() -> Option<Vec<(String, u32, u32)>> {
    let raw = std::env::var("GATE_ONLY").ok()?;
    let picked: Vec<(String, u32, u32)> = raw
        .split(',')
        .map(str::trim)
        .filter(|item| !item.is_empty())
        .map(|item| {
            let mut parts = item.split(':');
            let db = parts.next().expect("库名").to_string();
            let base = parts.next().expect("base").parse().expect("base 是数字");
            let target = parts
                .next()
                .expect("target")
                .parse()
                .expect("target 是数字");
            (db, base, target)
        })
        .collect();
    Some(picked)
}

/// 缺语料默认**红**，不再静默变绿。
///
/// 这里一度是 `eprintln` + `return`：换台没有语料盘的机器，整道门一个断言都
/// 不跑照样全绿——门等于没有（设计文档 §4 点名的欠账）。仓里其它真库门的软跳过
/// 惯例是给探针与解码器的；这道门是本 crate 增量正确性的**唯一裁判**，它不在场
/// 必须让人知道。确属无语料环境（只改纯函数、CI 沙箱），显式设
/// `E3D_SKIP_REAL_GATES=1` 换回软跳过——跳过该是一个决定，不该是一个默认。
fn fixtures_present() -> bool {
    let missing: Vec<&str> = [AMS_DIR, ATTLIB, TEMPLATE_DIR]
        .into_iter()
        .filter(|path| !Path::new(path).exists())
        .collect();
    if missing.is_empty() {
        return true;
    }
    if std::env::var_os("E3D_SKIP_REAL_GATES").is_some() {
        eprintln!(
            "跳过（E3D_SKIP_REAL_GATES 已设）：缺 {}",
            missing.join("、")
        );
        return false;
    }
    panic!(
        "真库门语料缺失：{}。这道门是增量正确性的唯一裁判，缺席不许静默变绿；\
         确属无语料环境要跳过，显式设 E3D_SKIP_REAL_GATES=1。",
        missing.join("、"),
    );
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

/// 目录库按需补开：同一份扫描表交给每个端点池（与 `update_ams --project-dir` 同款）。
///
/// 这道门此前只钉 DESI 一个库：路由成员 / 托盘经 SPRE 走目录求值，目录库不在池里
/// 就 `DbNotRegistered` → 两端全量都建不出、增量也建不出，账平门绿，**派生腿与
/// 目录实体在门上没有判别力**（模块文档「派生单元」那段的自述）。目录库进门之后，
/// 五个老窗口的裁判会多看见管件 / 管身，桥架窗口才可能被看见。
struct SharedResolver(Arc<DirectoryDbResolver>);

impl DbFileResolver for SharedResolver {
    fn resolve(&self, dbno: u32) -> Option<DbFilePin> {
        self.0.resolve(dbno)
    }
}

/// 扫 ams000（+ acp000，存在才扫）一次，整个测试进程复用。
fn shared_resolver() -> Arc<DirectoryDbResolver> {
    static RESOLVER: OnceLock<Arc<DirectoryDbResolver>> = OnceLock::new();
    RESOLVER
        .get_or_init(|| {
            let mut dirs = vec![PathBuf::from(AMS_DIR)];
            if Path::new(ACP_DIR).is_dir() {
                dirs.push(PathBuf::from(ACP_DIR));
            } else {
                eprintln!("目录库目录 {ACP_DIR} 不在：跨项目目录（7002/7320）不进门，引用它们的单元两端都建不出");
            }
            let resolver = DirectoryDbResolver::scan(&dirs, Path::new(TEMPLATE_DIR))
                .expect("扫库目录建 dbno → 文件表");
            println!("{}", resolver.summary());
            Arc::new(resolver)
        })
        .clone()
}

/// 把库钉在某个会话上开一个读取池（与 `update_ams` 同款）：DESI 钉会话，目录库
/// 不钉（两端读同一时点的目录，目录变化不在窗口射程内——那是跨库失效通道的活）。
fn open_pinned(attlib: &Arc<AttlibData>, db_file: &Path, sesno: u32) -> Arc<DbSet> {
    let set = Arc::new(DbSet::with_resolver(
        attlib.clone(),
        Box::new(SharedResolver(shared_resolver())),
    ));
    let template = template_file_for(Path::new(TEMPLATE_DIR), DB_TYPE).expect("模板文件找得到");
    set.add_db(DbFilePin {
        file: db_file.to_path_buf(),
        template,
        db_type: Some(DB_TYPE.to_string()),
        sesno: Some(sesno),
    })
    .unwrap_or_else(|error| panic!("{} 钉在会话 {sesno}：{error}", db_file.display()));
    // 本项目那份 7000：先于任何按需解析登记，resolver 就不会再为 7000 拒仲裁。
    let catalogue = Path::new(AMS_DIR).join(PROJECT_CATALOGUE);
    if catalogue.is_file() {
        set.add_db(DbFilePin {
            file: catalogue.clone(),
            template: template_file_for(Path::new(TEMPLATE_DIR), "CATA").expect("CATA 模板"),
            db_type: Some("CATA".to_string()),
            sesno: None,
        })
        .unwrap_or_else(|error| panic!("{} 作目录库登记：{error}", catalogue.display()));
    }
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

/// 裁判的键是 [`GeometryId`]，**不是 refno**。
///
/// 这里一度是 `BTreeMap<(u32, u32), _>`：refno 那把键在计划集、Remove、模型集、
/// 裁判四处都用着，而产物侧早就是 `GeometryId` 了。一个路由容器名下的 N 根隐式
/// 管身共享同一个容器 refno，按 refno 收全量快照会把它们**塌成一条**——那条
/// 「upsert 里有重复 refno」的断言会当场炸，炸得对，但它给的理由是错的
/// （不是「同一件被重建了两次」，是「键根本不够用」）。
type Snapshot = BTreeMap<GeometryId, Invariants>;

/// 两份快照逐件一致（件数、键集、不变量都算）。
fn snapshots_agree(before: &Snapshot, after: &Snapshot) -> bool {
    before.len() == after.len()
        && before
            .iter()
            .all(|(id, left)| after.get(id).is_some_and(|right| left.agrees_with(right)))
}

/// 把整个库在某个会话上全量生成一遍，收成 [`GeometryId`] → 不变量。**这是裁判。**
fn full_generation(attlib: &Arc<AttlibData>, db_file: &Path, sesno: u32) -> Snapshot {
    let set = open_pinned(attlib, db_file, sesno);
    let mut snapshot = Snapshot::new();
    for root in roots_at(db_file, sesno) {
        let outcome = generate_subtree(set.element(root)).unwrap_or_else(|error| {
            panic!(
                "{} 会话 {sesno} 从 {root} 全量生成：{error}",
                db_file.display()
            )
        });
        for generated in &outcome.elements {
            snapshot.insert(generated.geometry_id.clone(), Invariants::of(generated));
        }
    }
    snapshot
}

/// 这件几何**挂在哪个库元素上**：实体是它自己，隐式管身是它的容器。
///
/// 只用来跟索引候选对名字（管身没有库记录，索引差分永远点不到它）。
/// 别拿它当键用——一个容器 refno 名下会有多根管身，那正是换键要解决的问题。
fn owning_refno(id: &GeometryId) -> &str {
    match id {
        GeometryId::Element { refno } => refno,
        GeometryId::ImpliedTube {
            container_refno, ..
        } => container_refno,
    }
}

/// **主门。** 三个库各跑一次「两端全量当裁判」的对账。
#[test]
fn an_increment_says_what_two_full_generations_say() {
    if !fixtures_present() {
        return;
    }
    let attlib = shared_attlib();
    let only = gate_only();
    for sample in SAMPLES {
        let Sample {
            db: name,
            base: base_sesno,
            target: target_sesno,
            shape,
        } = *sample;
        if let Some(only) = &only
            && !only.iter().any(|(db, base, target)| {
                db == name && *base == base_sesno && *target == target_sesno
            })
        {
            continue;
        }
        let db_file = Path::new(AMS_DIR).join(name);

        // 裁判：两端各全量生成一遍，逐件比不变量。
        let base_full = full_generation(&attlib, &db_file, base_sesno);
        let target_full = full_generation(&attlib, &db_file, target_sesno);
        let appeared: BTreeSet<GeometryId> = target_full
            .keys()
            .filter(|id| !base_full.contains_key(*id))
            .cloned()
            .collect();
        let vanished: BTreeSet<GeometryId> = base_full
            .keys()
            .filter(|id| !target_full.contains_key(*id))
            .cloned()
            .collect();
        let changed: BTreeSet<GeometryId> = target_full
            .iter()
            .filter(|(id, after)| {
                base_full
                    .get(*id)
                    .is_some_and(|before| !before.agrees_with(after))
            })
            .map(|(id, _)| id.clone())
            .collect();

        // 空窗口自检：这道门此前挑窗口挑的是索引候选数，三个样本折算到模型单元
        // 之后全是空的，全绿了几轮什么都没验。样本声明压哪条路径，裁判就必须真的
        // 在那条路径上看见东西。
        let trays_touched = || {
            let is_tray = |id: &GeometryId, judge: &Snapshot| {
                judge.get(id).is_some_and(|inv| inv.noun == "CTRAY")
            };
            appeared
                .iter()
                .chain(changed.iter())
                .filter(|id| is_tray(id, &target_full))
                .count()
                + vanished.iter().filter(|id| is_tray(id, &base_full)).count()
        };
        // 管路分支：数的是隐式管身与路由成员（BRAN 名下能变的两种几何）。
        let route_touched = || {
            let is_route = |id: &GeometryId, judge: &Snapshot| {
                matches!(id, GeometryId::ImpliedTube { .. })
                    || judge.get(id).is_some_and(|inv| {
                        matches!(
                            e3d_model::category::classify(&inv.noun),
                            e3d_model::category::Category::RouteMember
                        )
                    })
            };
            appeared
                .iter()
                .chain(changed.iter())
                .filter(|id| is_route(id, &target_full))
                .count()
                + vanished
                    .iter()
                    .filter(|id| is_route(id, &base_full))
                    .count()
        };
        let exercised = match shape {
            Shape::Inserted => appeared.len(),
            Shape::Modified | Shape::Cascaded | Shape::Reparented => changed.len(),
            Shape::Deleted => vanished.len(),
            Shape::CableTray => trays_touched(),
            Shape::Branch => route_touched(),
            // 删分支：裁判必须看见路由几何**消失**，不是变了。
            Shape::BranchRemoved => vanished
                .iter()
                .filter(|id| {
                    matches!(id, GeometryId::ImpliedTube { .. })
                        || base_full.get(*id).is_some_and(|inv| {
                            matches!(
                                e3d_model::category::classify(&inv.noun),
                                e3d_model::category::Category::RouteMember
                            )
                        })
                })
                .count(),
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
        // 计划期的账先对：候选账 + L3 每个 refno 恰好一个去向、去向计数与报告一致。
        plan.accounts_for()
            .unwrap_or_else(|detail| panic!("{name}: 计划账不平：{detail}"));
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

        let upserts: BTreeMap<GeometryId, Invariants> = outcome
            .upserts
            .iter()
            .map(|generated| (generated.geometry_id.clone(), Invariants::of(generated)))
            .collect();
        assert_eq!(
            upserts.len(),
            outcome.upserts.len(),
            "{name}: upsert 里有重复的 GeometryId，同一件被重建了两次"
        );
        let removals: BTreeSet<GeometryId> = outcome.removals.iter().cloned().collect();

        // 一件都不许漏：全量说变了的，增量必须动过。
        for id in appeared.iter().chain(changed.iter()) {
            assert!(
                upserts.contains_key(id),
                "{name}: 全量说 {id} 在 {base_sesno}→{target_sesno} 里变了（{}），增量却没重建它",
                target_full[id].summary(),
            );
        }
        for id in &vanished {
            assert!(
                removals.contains(id),
                "{name}: 全量说 {id} 在 {target_sesno} 上没了（{base_sesno} 时还是 {}），\
                 增量却没把它移除",
                base_full[id].summary(),
            );
        }

        // 反向这条更要命：多重建只是白干活，多移除是把一件真实存在的几何从模型集里
        // 抹掉。所以 Remove 虽然是幂等的「确保不存在」，也不许落在目标端还活着的件上。
        for id in &removals {
            assert!(
                !target_full.contains_key(id),
                "{name}: 增量把 {id} 移除了，但目标端全量生成里它还在（{}）",
                target_full[id].summary(),
            );
        }

        // 多算可以，但算出来的必须跟全量一模一样。
        for (id, rebuilt) in &upserts {
            let Some(judge) = target_full.get(id) else {
                panic!(
                    "{name}: 增量重建了 {id}（{}），但目标端全量生成里根本没有这一件",
                    rebuilt.summary(),
                );
            };
            assert!(
                rebuilt.agrees_with(judge),
                "{name}: {id} 增量重建成 {}，全量生成是 {}",
                rebuilt.summary(),
                judge.summary(),
            );
        }

        // 覆盖断言是单边的，所以还得有人拦住「整库重建」这种作弊解。
        // 例外只有一种：base 端本来就一件几何都没有（库在这一环里被整体导入，
        // 桥架库 ams251142 的 2→9 就是这样），那时「全建」就是正确答案，不是作弊。
        if base_full.is_empty() {
            println!("  base 端零件（整体导入窗口），不查「增量 < 全量」这条");
        } else {
            assert!(
                upserts.len() < target_full.len(),
                "{name}: 目标端一共 {} 件，增量重建了 {} 件——这不叫增量",
                target_full.len(),
                upserts.len(),
            );
        }

        // 级联样本还要再证一件事：上面那条「一件都不许漏」分不清后代是**被级联捞回来的**
        // 还是**它自己也在候选里**。差分只点了容器的名字，后代的索引记录一个字节没动，
        // 所以这里要求至少有一件「变了但没有候选」的——那正是级联唯一的存在理由。
        if shape == Shape::Cascaded {
            assert!(
                !outcome.report.cascades.is_empty(),
                "{name}: 样本声明压级联，plan_update 却一次 POS/ORI 漂移都没检出",
            );
            let named_by_index: BTreeSet<String> = window
                .diff
                .changes
                .iter()
                .map(|candidate| candidate.refno().to_string())
                .collect();
            let silent: Vec<&GeometryId> = changed
                .iter()
                .filter(|id| !named_by_index.contains(owning_refno(id)))
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

        // 桥架样本：裁判看见的每一只托盘都得由增量亲手重建——不是被别的单元顺带。
        // 上面的覆盖断言已经保证了这一点；这里把数字打出来，让「128 只托盘、128 条
        // CTRAY upsert」在日志里能对上号，并钉住计划把它们记成了 rolled_up 而不是 no_model。
        if shape == Shape::CableTray {
            let tray_upserts = upserts
                .iter()
                .filter(|(_, inv)| inv.noun == "CTRAY")
                .count();
            println!(
                "  桥架：裁判 {} 只托盘变了，增量重建 {tray_upserts} 只；no_model 里 CTRAY = {}",
                trays_touched(),
                outcome.report.no_model.get("CTRAY").copied().unwrap_or(0),
            );
            assert_eq!(
                outcome.report.no_model.get("CTRAY").copied().unwrap_or(0),
                0,
                "{name}: 有 CTRAY 候选落进了 no_model——托盘不再是模型单元了？"
            );
            assert!(tray_upserts > 0, "{name}: 一只托盘都没重建");
        }

        // 管路样本要再证派生腿真的动了：BRAN 进了 `regenerate_derived`（而不是靠成员
        // 重建顺带），执行侧真重算了整链，且管身的 upsert/remove 都走 `ImpliedTube` 键。
        if shape == Shape::Branch {
            let tube_upserts = upserts
                .keys()
                .filter(|id| matches!(id, GeometryId::ImpliedTube { .. }))
                .count();
            let tube_removals = removals
                .iter()
                .filter(|id| matches!(id, GeometryId::ImpliedTube { .. }))
                .count();
            println!(
                "  管路：裁判 {} 件路由几何变了；派生容器计划 {} / 重算 {} / 退休 {} / 失败 {}；\
                 管身 upsert {tube_upserts} remove {tube_removals}",
                route_touched(),
                outcome.report.derived_to_regenerate,
                outcome.report.derived_regenerated,
                outcome.report.derived_retired,
                outcome.report.derived_failed.len(),
            );
            assert!(
                outcome.report.derived_to_regenerate > 0,
                "{name}: 管身变了，却没有一个 BRAN 进 regenerate_derived——派生腿没接上"
            );
            assert!(
                outcome.report.derived_failed.is_empty(),
                "{name}: 派生容器重算失败：{:?}",
                outcome.report.derived_failed
            );
            assert!(
                tube_upserts > 0,
                "{name}: 一根管身都没 upsert，裁判却说管身变了"
            );
        }

        // 删分支样本：容器必须走**退休**（不重算、清旧键），旧管身键必须真的被 Remove。
        if shape == Shape::BranchRemoved {
            let tube_removals = removals
                .iter()
                .filter(|id| matches!(id, GeometryId::ImpliedTube { .. }))
                .count();
            let vanished_tubes = vanished
                .iter()
                .filter(|id| matches!(id, GeometryId::ImpliedTube { .. }))
                .count();
            println!(
                "  删分支：裁判 {exercised} 件路由几何消失（其中管身 {vanished_tubes}）；\
                 派生容器计划 {} / 重算 {} / 退休 {} / 失败 {}；管身 Remove {tube_removals}",
                outcome.report.derived_to_regenerate,
                outcome.report.derived_regenerated,
                outcome.report.derived_retired,
                outcome.report.derived_failed.len(),
            );
            assert!(
                outcome.report.derived_retired > 0,
                "{name}: 整条 BRAN 删了，却没有一个容器走退休——旧管身键谁来清？"
            );
            assert!(
                outcome.report.derived_failed.is_empty(),
                "{name}: 派生容器处理失败：{:?}",
                outcome.report.derived_failed
            );
            assert!(
                tube_removals >= vanished_tubes,
                "{name}: 裁判说 {vanished_tubes} 根管身没了，增量只 Remove 了 {tube_removals} 根"
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

/// **改挂的祖先闭包必须两端都收。** `BOX 24384/25802` 在 ams8000 45→46 从 EQUI
/// `24384/25801` 挪到 EQUI `24384/25803`：新属主链只有 target 图认得，旧属主链只有
/// base 图认得，而消费端（gen-model 凭证前移）问的正是「旧根 25801 这一窗有没有被
/// 波及」——旧根的 manifest 里还留着这件 BOX，不重算就是残留（ADR-009 两端根都重算）。
///
/// 这条门钉的是 [`UpdatePlan::affected_closure`] 的一个真实漏洞：同一件元素先走 target
/// 链再走 base 链时，「已在闭包里就早退」的捷径把 base 链整条吞掉，旧属主永远收不进来。
/// 捷径只能在**同一端图**内成立——两端图上同一个 refno 的祖先不是同一条链。
#[test]
fn the_affected_closure_reaches_both_owners_of_a_reparented_element() {
    if !fixtures_present() {
        return;
    }
    let attlib = shared_attlib();
    let db_file = Path::new(AMS_DIR).join("ams8000_0001");
    let (base_sesno, target_sesno) = (45, 46);
    let base = open_pinned(&attlib, &db_file, base_sesno);
    let target = open_pinned(&attlib, &db_file, target_sesno);
    let window = collect_window(&db_file, base_sesno, target_sesno).expect("收集 45→46");
    let plan = plan_update(&base, &target, &window);

    let moved = RefNo::new(24384, 25802);
    let old_owner = RefNo::new(24384, 25801);
    let new_owner = RefNo::new(24384, 25803);
    let reparented: Vec<RefNo> = plan
        .ledger
        .of_kind(ChangeKind::Reparented)
        .map(|entry| entry.refno)
        .collect();
    assert_eq!(
        reparented,
        vec![moved],
        "窗口前提：这一环恰好改挂了那件 BOX"
    );

    let closure = plan.affected_closure(&base, &target);
    assert!(closure.is_complete(), "{:?}", closure.unreadable);
    assert!(closure.contains(moved), "改挂元素自己必须在闭包里");
    assert!(
        closure.contains(new_owner),
        "新属主 {new_owner}（target 链）必须在闭包里"
    );
    assert!(
        closure.contains(old_owner),
        "旧属主 {old_owner}（base 链）必须在闭包里——它的 manifest 里还留着搬走的那件"
    );
    let touched = plan.touches_roots(&[old_owner, new_owner], &base, &target);
    assert_eq!(
        touched,
        BTreeSet::from([key(old_owner), key(new_owner)]),
        "两端根都要被判成受波及"
    );
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

        // `PROBE_NOUN=CTRAY` 之类：只拿这一种 noun 的件做裁判，二分收敛到的就是
        // 「这种件变了」的那一环，而不是库里随便哪件（桥架库里多半是支架 TMPL 的盒子）。
        let noun_filter = probe_noun_filter();
        let timed_full_generation = |sesno: u32| {
            let (mut snapshot, cost) = timed_full_generation(&attlib, &db_file, sesno);
            if let Some(noun) = &noun_filter {
                snapshot.retain(|_, inv| inv.noun == *noun);
            }
            (snapshot, cost)
        };
        let (mut newer_full, cost) = timed_full_generation(chain[0]);
        println!(
            "  链头 {}：全量 {} 件{}，{:.1}s",
            chain[0],
            newer_full.len(),
            noun_filter
                .as_ref()
                .map(|noun| format!("（只看 {noun}）"))
                .unwrap_or_default(),
            cost.as_secs_f64(),
        );
        let (oldest_full, cost) = timed_full_generation(chain[oldest]);
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
            let (mid_full, cost) = timed_full_generation(chain[mid]);
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

/// 在会话链上逐环找**删整条 BRAN**的窗口：路由容器自己是 Deleted 候选。
///
/// 这条压的是派生腿的「退休」半边：容器在 target 端没了，`execute_plan` 不重算链，
/// 而是把 base 端 `route_slot_keys` 收来的全部旧管身键发 Remove（`derived_retired`）。
/// 与 [`probe_chain_for_cascade`] 同款只做 `collect_window` + `plan_update`，一件几何都
/// 不建；命中就把被删容器、它名下被删的路由成员数、以及计划里的 `derived` 数打出来。
/// `PROBE_CONTAINER_NOUN`（默认 `BRAN`）可换成 LUG/SUPC/TRUNNI。
#[test]
#[ignore = "逐环扫会话链，找删整条路由容器（BRAN）的窗口；run with --ignored --nocapture"]
fn probe_chain_for_branch_delete() {
    if !fixtures_present() {
        return;
    }
    let attlib = shared_attlib();
    let cap = probe_max_candidates();
    let container_noun = std::env::var("PROBE_CONTAINER_NOUN")
        .ok()
        .map(|noun| noun.trim().to_ascii_uppercase())
        .filter(|noun| !noun.is_empty())
        .unwrap_or_else(|| "BRAN".to_string());
    for name in probe_libs() {
        let db_file = Path::new(AMS_DIR).join(&name);
        let Some(chain) = chain_of(&db_file) else {
            println!("\n=== {name}: 开不了库，或链上只有一个会话 ===");
            continue;
        };
        println!(
            "\n=== {name}: 链 {} 环（新 → 旧），找被删的 {container_noun}，候选上限 {cap} ===",
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
            // 删容器一定带 Deleted 候选；没有删除的环直接跳过，连 DbSet 都不开。
            if window.diff.tally.deleted == 0 {
                continue;
            }
            if window.diff.changes.len() > cap {
                skipped += 1;
                continue;
            }
            let base_set = open_pinned(&attlib, &db_file, base_sesno);
            let target_set = open_pinned(&attlib, &db_file, target_sesno);
            let plan = plan_update(&base_set, &target_set, &window);
            let deleted_containers: Vec<RefNo> = plan
                .ledger
                .of_kind(ChangeKind::Deleted)
                .filter(|entry| entry.noun == container_noun)
                .map(|entry| entry.refno)
                .collect();
            if deleted_containers.is_empty() {
                continue;
            }
            hits += 1;
            let deleted_members = plan
                .ledger
                .of_kind(ChangeKind::Deleted)
                .filter(|entry| {
                    matches!(
                        e3d_model::category::classify(&entry.noun),
                        e3d_model::category::Category::RouteMember
                    )
                })
                .count();
            // 被删容器在 base 端有几根管身槽位——退休时要清的旧键数。
            let stale_slots: usize = deleted_containers
                .iter()
                .map(|container| {
                    e3d_model::route::route_slot_keys(&base_set.element(*container))
                        .map(|slots| slots.len())
                        .unwrap_or(0)
                })
                .sum();
            println!(
                "  ★ {base_sesno}→{target_sesno}: 删 {container_noun} {}（{}），名下被删路由成员 {deleted_members}，\
                 base 端旧管身槽位 {stale_slots}，计划 derived={} remove={} | {}",
                deleted_containers.len(),
                deleted_containers
                    .iter()
                    .map(RefNo::to_string)
                    .collect::<Vec<_>>()
                    .join(" "),
                plan.regenerate_derived.len(),
                plan.remove.len(),
                plan.report.totals_line(),
            );
        }
        println!(
            "  → {hits} 个删{container_noun}窗口，{skipped} 环因候选超上限跳过，耗时 {:.1}s",
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
        // `PROBE_LIBS=ALL_DESI`：ams000 里扫到的全部 DESI 主文件（按库号），给要普查全语料的探针。
        Ok(list) if list.trim().eq_ignore_ascii_case("ALL_DESI") => {
            let ams_dir = Path::new(AMS_DIR);
            let mut names: Vec<String> = shared_resolver()
                .discovered()
                .iter()
                .filter(|db| db.db_type == "DESI" && db.ext_no <= 1)
                .filter(|db| db.file.parent() == Some(ams_dir))
                .filter_map(|db| {
                    db.file
                        .file_name()
                        .map(|name| name.to_string_lossy().into_owned())
                })
                .collect();
            names.sort();
            names.dedup();
            names
        }
        Ok(list) => list
            .split(',')
            .map(str::trim)
            .filter(|name| !name.is_empty())
            .map(str::to_owned)
            .collect(),
        Err(_) => PROBE_LIBS.iter().map(|name| (*name).to_owned()).collect(),
    }
}

/// [`probe_chain_for_model_change`] 的裁判只看这一种 noun（`PROBE_NOUN=CTRAY`）；不设就全看。
fn probe_noun_filter() -> Option<String> {
    std::env::var("PROBE_NOUN")
        .ok()
        .map(|noun| noun.trim().to_ascii_uppercase())
        .filter(|noun| !noun.is_empty())
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

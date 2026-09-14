//! 真库门：`GenerationOutcome::catalogue_visited` 说的必须是**真读到的目录元素**。
//!
//! 这份名单是 gen-model 依赖闭包（ADR-064 D5）的对拍 oracle：闭包 ⊇ 实读才算闭包没漏。
//! oracle 自己先得可信——它要是少记了（某条目录读法没经过 e3d-io 的读入口）或多记了
//! （把设计侧元素混进来），对拍就会在错的地方绿或红。这里钉三件事：
//!
//! 1. **只有目录侧**：每个 refno 不在根所在库、且所在库在池里登记为 `CATA`。
//! 2. **第一跳一个不漏**：BRAN 名下每个带 `SPRE` 的路由成员，其 SPCO 都在名单里——
//!    路由走查（`walk_route`）为每个成员求到达 / 离开 P 点，必经 `SPRE→SPCO`。
//! 3. **链走到底**：名单里有 SCOM 与点集 / 几何集元素（PTSE / GMSE 一族），不是只停在 SPCO。
//!
//! 缺 E3D 安装或影子资产**默认红**（与 `increment_real.rs` 同一口径）：这道门是 oracle
//! 唯一的真库裁判，缺席必须出声；确属无语料环境显式设 `E3D_SKIP_REAL_GATES=1` 换回跳过。

use std::collections::{BTreeMap, HashSet};
use std::path::{Path, PathBuf};
use std::sync::Arc;

use e3d_attlib::AttlibData;
use e3d_io::db_element::{DbFilePin, DbFileResolver, DbSet, template_file_for};
use e3d_io::engine::{ReadOnlyEngine, ScanTier};
use e3d_io::refno::RefNo;
use e3d_model::catalogue::CatalogueMeshCache;
use e3d_model::db_discovery::DirectoryDbResolver;
use e3d_model::pipeline::generate_subtree_with_cache;

const AMS_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaMarineSample\ams000";
const ACP_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaCatalogue\acp000";
const DESIGN_DB: &str = "ams8000_0001";
/// 库 7000 在 ams000 与 acp000 各有一份，扫目录的 resolver 按设计拒仲裁；本项目那份显式钉。
const PROJECT_CATALOGUE: &str = "ams7000_0001";
const ATTLIB: &str = r"E:\reverse\e3d\shadow_e3d31_aps_all\attlib.dat";
const TEMPLATE_DIR: &str = r"E:\reverse\e3d\shadow_e3d31_aps_all";

fn fixtures_present() -> bool {
    let design = Path::new(AMS_DIR).join(DESIGN_DB);
    let missing: Vec<String> = [
        Path::new(AMS_DIR),
        design.as_path(),
        Path::new(ATTLIB),
        Path::new(TEMPLATE_DIR),
    ]
    .into_iter()
    .filter(|path| !path.exists())
    .map(|path| path.display().to_string())
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
        "真库门语料缺失：{}。catalogue_visited 的可信度只有这道门在裁，缺席不许静默变绿；\
         确属无语料环境要跳过，显式设 E3D_SKIP_REAL_GATES=1。",
        missing.join("、"),
    );
}

struct SharedResolver(Arc<DirectoryDbResolver>);

impl DbFileResolver for SharedResolver {
    fn resolve(&self, dbno: u32) -> Option<DbFilePin> {
        self.0.resolve(dbno)
    }
}

/// 设计库钉最新会话、目录库按需补开（与 `increment_real.rs::open_pinned` 同款）。
fn open_design_set() -> (Arc<DbSet>, PathBuf, u32) {
    let mut dirs = vec![PathBuf::from(AMS_DIR)];
    if Path::new(ACP_DIR).is_dir() {
        dirs.push(PathBuf::from(ACP_DIR));
    }
    let resolver = Arc::new(
        DirectoryDbResolver::scan(&dirs, Path::new(TEMPLATE_DIR))
            .expect("扫库目录建 dbno → 文件表"),
    );
    println!("{}", resolver.summary());
    let attlib = Arc::new(AttlibData::parse_file(ATTLIB).expect("attlib 解析得开"));
    let set = Arc::new(DbSet::with_resolver(
        attlib,
        Box::new(SharedResolver(resolver)),
    ));
    let design = Path::new(AMS_DIR).join(DESIGN_DB);
    let dbno = set
        .add_db(DbFilePin {
            file: design.clone(),
            template: template_file_for(Path::new(TEMPLATE_DIR), "DESI").expect("DESI 模板"),
            db_type: Some("DESI".to_string()),
            sesno: None,
        })
        .unwrap_or_else(|error| panic!("{} 开最新会话：{error}", design.display()));
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
    (set, design, dbno)
}

/// 树根 = owner 指向自己、无效、或不在本库索引里的元素（与 `gen_ams` 同一口径）。
fn roots_of(db_file: &Path) -> Vec<RefNo> {
    let mut engine = ReadOnlyEngine::open(db_file).expect("开库扫根");
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

#[test]
fn catalogue_visited_is_exactly_the_catalogue_side_of_what_generation_read() {
    if !fixtures_present() {
        return;
    }
    let (set, design, design_dbno) = open_design_set();
    let roots = roots_of(&design);
    assert!(!roots.is_empty(), "{DESIGN_DB} 扫不出树根");

    let mut visited: Vec<RefNo> = Vec::new();
    let mut catalogue: HashSet<RefNo> = HashSet::new();
    let mut per_root: BTreeMap<String, (usize, usize)> = BTreeMap::new();
    for root in &roots {
        // 每根一个空的目录网格缓存：缓存命中的目录件不会再被读，也就不会进名单，
        // 这道门要的是每根**完整**的实读集。
        let mut cache = CatalogueMeshCache::default();
        let outcome = generate_subtree_with_cache(set.element(*root), &mut cache)
            .unwrap_or_else(|error| panic!("{root} 生成失败：{error}"));
        per_root.insert(
            root.to_string(),
            (outcome.visited.len(), outcome.catalogue_visited.len()),
        );
        // 名单自己不重复。
        let mut seen = HashSet::new();
        for refno in &outcome.catalogue_visited {
            assert!(
                seen.insert(*refno),
                "{root} 的 catalogue_visited 里 {refno} 出现了两次"
            );
        }
        visited.extend(outcome.visited);
        catalogue.extend(outcome.catalogue_visited);
    }
    println!(
        "roots={} visited={} catalogue_visited={}",
        roots.len(),
        visited.len(),
        catalogue.len()
    );
    for (root, (visited, catalogue)) in &per_root {
        println!("  {root}: visited {visited}, catalogue {catalogue}");
    }
    assert!(
        !catalogue.is_empty(),
        "ams8000 有几百条管线，目录实读集不可能为空"
    );

    // 1. 只有目录侧。
    for refno in &catalogue {
        assert_ne!(
            refno.dbno(),
            design_dbno,
            "{refno} 在设计库自己里，却被记成目录实读"
        );
        let db_type = set.db_type(refno.dbno());
        assert_eq!(
            db_type.as_deref(),
            Some("CATA"),
            "{refno} 所在库 {} 的类型是 {db_type:?}，不是 CATA",
            refno.dbno()
        );
    }

    // 2. 第一跳一个不漏：BRAN 成员的 SPRE → SPCO。
    let mut checked = 0usize;
    let mut missing: Vec<String> = Vec::new();
    let mut by_noun: BTreeMap<String, usize> = BTreeMap::new();
    for refno in &visited {
        let el = set.element(*refno);
        let Ok(Some(owner)) = el.owner() else {
            continue;
        };
        if owner.element_type().ok().as_deref() != Some("BRAN") {
            continue;
        }
        let Ok(Some(spre)) = el.get_ref("SPRE") else {
            continue;
        };
        // 目录库没开（resolver 供不出）的引用两边都读不到，不在这条断言的射程内。
        if set.db_type(spre.dbno()).as_deref() != Some("CATA") {
            continue;
        }
        checked += 1;
        let noun = el.element_type().unwrap_or_default();
        *by_noun.entry(noun.clone()).or_insert(0) += 1;
        if !catalogue.contains(&spre) {
            missing.push(format!("{refno} ({noun}) SPRE → {spre}"));
        }
    }
    println!("BRAN 成员 SPRE 抽查 {checked} 条，按 noun：{by_noun:?}");
    assert!(
        checked > 0,
        "没有一个 BRAN 成员带指向已开目录库的 SPRE，门没有判别力"
    );
    assert!(
        missing.is_empty(),
        "{} 个 BRAN 成员的 SPCO 没进 catalogue_visited（路由走查必经 SPRE→SPCO，少了就是读入口漏记）：{:?}",
        missing.len(),
        &missing[..missing.len().min(12)]
    );

    // 3. 链走到底：不止 SPCO。
    let mut nouns: BTreeMap<String, usize> = BTreeMap::new();
    for refno in &catalogue {
        let noun = set
            .element(*refno)
            .element_type()
            .unwrap_or_else(|_| "?".into());
        *nouns.entry(noun).or_insert(0) += 1;
    }
    println!("catalogue_visited 按 noun：{nouns:?}");
    for required in ["SPCO", "SCOM"] {
        assert!(
            nouns.get(required).copied().unwrap_or(0) > 0,
            "目录实读集里没有 {required}，目录链没走到底：{nouns:?}"
        );
    }
    assert!(
        ["PTSE", "GMSE", "GMSS", "NGMS"]
            .iter()
            .any(|noun| nouns.contains_key(*noun)),
        "目录实读集里没有点集 / 几何集元素（PTSE / GMSE / GMSS / NGMS），P 点与几何求值没读到目录：{nouns:?}"
    );
}

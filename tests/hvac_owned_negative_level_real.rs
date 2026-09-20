//! 真库门：目录正体名下的**成员级负体**也要过表现级筛（`CSG_TreeBuilderCat::addNegatives` 对每把刀
//! 调 `isWanted`），不然 ams7997 的风管全被扣成 2.5 mm 薄壳。
//!
//! 语料（2026-09-19，`_runs/ams-init-1112-7997-20260917-mc/e3d-hvac-tight-154700/p3_probe_negatives.log`）：
//! CADC 风管目录给每只本体挂一把内缩 5 mm 的刀——`/RSTHRE-BODY`（SEXT）名下 `/RSTHRE-NBODY`（NSEX，
//! loop 逐点 −5）、`/CSTRA-BODY`（SCYL）名下 NSCY `PDIA = DESP[2] − 5`、`/RSTRA-BODY`（SBOX）名下
//! `/RSTRA-NBODY` `PXLE = DESP[2] − 5`——**全是 `LEVE 9 10`**。E3D 在 `LEVEL HVAC 6` 下不扣它们：
//! TTY RVM（`e3d-hvac-obst-135124/hvac-lvl6.rvm`，`repre level hvac 6 / insu off / obst off`）里
//! STRT 是实心 `Cylinder` / `Box`、THRE 是 10 面实心 FacetGroup。引擎此前对成员级刀不筛，于是：
//!
//! - THRE `24381/30501`（DESP 500/500/500/500，DESP[12]=DESP[13]=DESP[54]=DESP[55]=150）本体 loop 面积
//!   475 000 mm² × 高 500 = **237 500 000 mm³**，引擎出的是 4 113 688 mm³ 的空壳（32 个顶点，内圈 ±247.5）；
//! - STRT `24381/76697`（Ø1600 × 930，法兰 Ø1700 / Ø1610）实心 ≈ 1.87e9，引擎 1.80e7；
//! - BRCO `24381/57376`（`/RSBRCO`，DESP[41]=DESP[42]=2.5）的 LPYR 本体 2.5×2.5×117 被自己那把
//!   `2.5 − 5` 的刀整只吃掉，E3D 画了这根杆（RVM `Pyramid 2.5,2.5,2.5,2.5,0,0,117`）。
//!
//! 缺 E3D 安装或影子资产**默认红**（与 `increment_real.rs` 同一口径）；确属无语料环境显式设
//! `E3D_SKIP_REAL_GATES=1` 换回跳过。

use std::path::{Path, PathBuf};
use std::sync::Arc;

use e3d_attlib::AttlibData;
use e3d_io::db_element::{DbFilePin, DbFileResolver, DbSet, template_file_for};
use e3d_io::refno::RefNo;
use e3d_model::catalogue::{CatalogueMeshCache, model_catalogue_member_with_cache};
use e3d_model::db_discovery::DirectoryDbResolver;

const AMS_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaMarineSample\ams000";
const ACP_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaCatalogue\acp000";
const DESIGN_DB: &str = "ams7997_0001";
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
        eprintln!("跳过（E3D_SKIP_REAL_GATES 已设）：缺 {}", missing.join("、"));
        return false;
    }
    panic!(
        "真库门语料缺失：{}。缺席不许静默变绿；确属无语料环境要跳过，显式设 E3D_SKIP_REAL_GATES=1。",
        missing.join("、"),
    );
}

struct SharedResolver(Arc<DirectoryDbResolver>);

impl DbFileResolver for SharedResolver {
    fn resolve(&self, dbno: u32) -> Option<DbFilePin> {
        self.0.resolve(dbno)
    }
}

/// 设计库开最新会话、目录库按需补开（与 `catalogue_visited_real.rs::open_design_set` 同款）。
fn open_design_set() -> Arc<DbSet> {
    let mut dirs = vec![PathBuf::from(AMS_DIR)];
    if Path::new(ACP_DIR).is_dir() {
        dirs.push(PathBuf::from(ACP_DIR));
    }
    let resolver = Arc::new(
        DirectoryDbResolver::scan(&dirs, Path::new(TEMPLATE_DIR)).expect("扫库目录建 dbno → 文件表"),
    );
    let attlib = Arc::new(AttlibData::parse_file(ATTLIB).expect("attlib 解析得开"));
    let set = Arc::new(DbSet::with_resolver(attlib, Box::new(SharedResolver(resolver))));
    let design = Path::new(AMS_DIR).join(DESIGN_DB);
    set.add_db(DbFilePin {
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
    set
}

#[test]
fn hvac_level_9_owned_negatives_are_not_applied_at_hvac_level_6() {
    if !fixtures_present() {
        return;
    }
    // 缺省表现级就是 `LEVEL HVAC 6`（`RepreDiscipline::default_level`）；显式钉住，免得环境变量漂。
    assert!(
        std::env::var_os("E3D_REPRE_LEVEL_HVAC").is_none(),
        "这道门按缺省 HVAC 6 级裁，别带着 E3D_REPRE_LEVEL_HVAC 跑"
    );
    let set = open_design_set();
    let mut cache = CatalogueMeshCache::default();
    let build = |refno: RefNo, cache: &mut CatalogueMeshCache| {
        let member = set.element(refno);
        let noun = member.element_type().expect("noun");
        let built = model_catalogue_member_with_cache(&member, cache)
            .unwrap_or_else(|error| panic!("{refno} {noun} 建模失败：{error}"));
        let generated = built.generated.unwrap_or_else(|| {
            panic!(
                "{refno} {noun} 没出几何：{:?} / {:?}",
                built.no_shape, built.diagnostics
            )
        });
        println!(
            "{refno} {noun}: solid volume {:.1} mm³, genus {}, parts {:?}, diagnostics {:?}",
            generated.solid.volume(),
            generated.solid.genus(),
            generated.parts.as_ref().map(Vec::len),
            built.diagnostics
        );
        generated
    };

    // 1. THRE /RSTHRE：本体是 loop 475 000 mm² × 500 的实心棱柱；四只法兰 LPYR 的 PTDI 全 0、
    //    两只 SBOX 接头 PZLE 全 0，都不出体（E3D 的 RVM 里也只有一只 10 面 FacetGroup + 零高 Pyramid）。
    let thre = build(RefNo::new(24381, 30501), &mut cache);
    let expected_thre = 475_000.0 * 500.0;
    assert!(
        (thre.solid.volume() - expected_thre).abs() < 1.0,
        "THRE 本体应是实心 {expected_thre} mm³（E3D 6 级不扣 LEVE 9–10 的 NSEX），实得 {}",
        thre.solid.volume()
    );
    assert_eq!(thre.solid.genus(), 0, "实心棱柱没有环柄，也没有内腔");

    // 2. 圆风管 STRT /CSTRA Ø1600 × 930 + 两端法兰：实心 ≈ 1.874e9（内接多边形略小），不是 1.8e7 的壳。
    let strt = build(RefNo::new(24381, 76697), &mut cache);
    let solid_cylinder = std::f64::consts::PI * 800.0 * 800.0 * 930.0;
    assert!(
        strt.solid.volume() > 0.98 * solid_cylinder,
        "圆风管本体应当实心（≥ 0.98 × {solid_cylinder:.0}），实得 {}",
        strt.solid.volume()
    );

    // 3. 矩形风管 STRT /RSTRA 400 × 200 × 1225：本体实心 98e6。
    let rect = build(RefNo::new(24381, 57355), &mut cache);
    assert!(
        rect.solid.volume() > 0.99 * 400.0 * 200.0 * 1225.0,
        "矩形风管本体应当实心，实得 {}",
        rect.solid.volume()
    );

    // 4. BRCO /RSBRCO：2.5 × 2.5 × 117 的 LPYR 本体要在（E3D 画了这根杆），它自己的 NLPY 是 LEVE 9–10。
    let brco = build(RefNo::new(24381, 57376), &mut cache);
    let parts = brco.parts.as_deref().unwrap_or_default();
    let rod = 2.5 * 2.5 * 117.0;
    assert!(
        parts.iter().any(|part| (part.volume() - rod).abs() < 0.01),
        "BRCO 应有一块 {rod} mm³ 的 LPYR 本体，实得各块体积 {:?}",
        parts.iter().map(|part| part.volume()).collect::<Vec<_>>()
    );

    // 5. 账：筛掉的刀记在 `display_filtered_negatives`，不是失败、不落 note。
    let stats = cache.stats();
    assert!(
        stats.display_filtered_negatives >= 4,
        "四只构件至少各筛掉一把 LEVE 9–10 的刀，实得 {}",
        stats.display_filtered_negatives
    );
    assert!(
        stats.evaluated_nsex + stats.evaluated_nsbo + stats.evaluated_nscy == 0,
        "6 级下这四只构件名下没有一把刀该被求值：nsex {} nsbo {} nscy {}",
        stats.evaluated_nsex,
        stats.evaluated_nsbo,
        stats.evaluated_nscy
    );
}

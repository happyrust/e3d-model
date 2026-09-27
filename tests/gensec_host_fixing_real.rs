//! 真库门：GENSEC 自己 datum 树里的 FIXING 给它扣的负体（`elmodl::collect_host_fixing_negatives`，
//! Core3D 的 mode 9 扫描）与 E3D 一致。
//!
//! 真值是 E3D TTY 只读导出的 RVM（gen-model `output/gensec-joint-20260927/aps250110-site.rvm`，
//! /SITE-STRUCTURAL-AREA02）：柱底板 SFIT 的 NSBO（NAPP 2，尺寸按 `OPAR` 取柱截面）切掉柱底一个
//! 板厚，端板 TMPL 的 NCYL（NAPP 7）与檐口 / 加腋 JOIN 的 NSCY（NAPP 6）在柱上开螺栓孔。没接这条
//! 之前柱底比 E3D 低 30 / 20 mm、柱身一个孔都没有；孔数拿 E3D 那组面片的亏格对。
//!
//! 缺 E3D 安装或影子资产默认红（与 `increment_real.rs` 同一口径）；无语料环境显式设
//! `E3D_SKIP_REAL_GATES=1` 换回跳过。

use std::path::{Path, PathBuf};
use std::sync::Arc;

use e3d_io::db_element::{DbFilePin, DbSet, template_file_for};
use e3d_io::refno::RefNo;
use e3d_model::category::classify;
use e3d_model::db_discovery::DirectoryDbResolver;
use e3d_model::elmodl::{Modeled, model_element};

const PLANT_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaPlantSample\aps000";
const ACP_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaCatalogue\acp000";
const DESIGN_DB: &str = "aps250110_0001";
const TEMPLATE_DIR: &str = r"E:\reverse\e3d\shadow_e3d31_aps_all";

fn fixtures_present() -> bool {
    let design = Path::new(PLANT_DIR).join(DESIGN_DB);
    let attlib = Path::new(TEMPLATE_DIR).join("attlib.dat");
    let missing: Vec<String> = [design.as_path(), Path::new(ACP_DIR), attlib.as_path()]
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
        "真库门语料缺失：{}。缺席不许静默变绿；确属无语料环境要跳过，显式设 E3D_SKIP_REAL_GATES=1。",
        missing.join("、"),
    );
}

/// aps250110 设计库 + 扫 aps000 / acp000 的 resolver（柱截面目录在 250700 / 250701，底板 SFIT
/// 与端板模板的目录在 250702 / 250705）。
fn aps_set() -> Arc<DbSet> {
    let template = Path::new(TEMPLATE_DIR);
    let resolver = DirectoryDbResolver::scan(
        &[PathBuf::from(PLANT_DIR), PathBuf::from(ACP_DIR)],
        template,
    )
    .unwrap();
    let set = Arc::new(
        DbSet::with_attlib_file_and_resolver(&template.join("attlib.dat"), Box::new(resolver))
            .unwrap(),
    );
    set.add_db(DbFilePin {
        file: Path::new(PLANT_DIR).join(DESIGN_DB),
        template: template_file_for(template, "DESI").unwrap(),
        db_type: Some("DESI".into()),
        sesno: None,
    })
    .unwrap();
    set
}

/// 柱：世界包围盒逐分量对 E3D RVM（RVM 值取到 0.1 mm），刀数钉住，孔数 = E3D 面片的亏格。
#[test]
fn host_fixings_cut_column_bases_and_bolt_holes_like_e3d() {
    if !fixtures_present() {
        return;
    }
    let set = aps_set();
    let cases: [(u32, [f64; 6], usize, i32, &str); 3] = [
        (
            277,
            [-312654.6, 318339.8, 100030.0, -312345.4, 318660.2, 107800.0],
            21,
            20,
            "GENSEC 1 of /B01_LEVEL_01_COLUMNS（30 mm 底板 + 两只 Flexible、两只 Full Depth 端板的 20 个螺栓孔）",
        ),
        (
            1823,
            [-288086.8, 314458.4, 100420.0, -287883.2, 314661.6, 104880.0],
            19,
            12,
            "GENSEC 1 of /AREA02_PR_ROW_P2.F（20 mm 底板）",
        ),
        (
            900,
            [-292654.6, 334339.8, 100030.0, -292345.4, 334660.2, 115500.0],
            101,
            88,
            "GENSEC 10 of /B02_LEVEL_01_COLUMNS（30 mm 底板 + 端板与 JOIN 节点的 88 个孔）",
        ),
    ];
    for (w1, rvm, negatives, holes, what) in cases {
        let el = set.element(RefNo::new(2013286654, w1));
        let generated = match model_element(&el, classify("GENSEC")).unwrap() {
            Modeled::Solid(generated) => generated,
            Modeled::Skipped { reason, .. } => panic!("{what} 没出实体：{reason}"),
        };
        let bb = generated.solid.bounding_box().expect("柱非空");
        let (lo, hi) = (bb.min(), bb.max());
        let got = [lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]];
        for (axis, (g, r)) in got.iter().zip(rvm).enumerate() {
            assert!(
                (g - r).abs() <= 0.1,
                "{what}：包围盒第 {axis} 分量 {g:.3}，E3D RVM {r:.1}（全部 {got:?}）"
            );
        }
        assert_eq!(generated.negatives_applied, negatives, "{what}");
        assert!(
            generated.negatives_skipped.is_empty(),
            "{what}：{:?}",
            generated.negatives_skipped
        );
        assert_eq!(generated.genus(), holes, "{what}：孔数");
    }
}

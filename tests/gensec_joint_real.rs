//! 真库门：GENSEC 两端的关节负体（`elmodl::collect_joint_negatives`）扣出来的梁端与 E3D 一致。
//!
//! 真值是 E3D TTY 只读导出的 RVM（gen-model `output/gensec-joint-20260927/aps250110-site.rvm`，
//! /SITE-STRUCTURAL-AREA02）：梁的 JOIS / JOIE 指向柱上的 FIXING，FIXING 名下节点模板里那只
//! NBOX（端板让位，NAPP -1）扣掉梁端；没接关节负体之前我们两端各长 8 mm。
//!
//! 缺 E3D 安装或影子资产默认红（与 `increment_real.rs` 同一口径）；无语料环境显式设
//! `E3D_SKIP_REAL_GATES=1` 换回跳过。

use std::path::{Path, PathBuf};
use std::sync::Arc;

use e3d_io::db_element::{DbFilePin, DbSet, template_file_for};
use e3d_io::refno::RefNo;
use e3d_model::category::classify;
use e3d_model::db_discovery::DirectoryDbResolver;
use e3d_model::elmodl::{GeneratedElement, Modeled, model_element};

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

/// aps250110 设计库 + 扫 aps000 / acp000 的 resolver（截面目录在 250701，另一端的关节
/// FIXING 可能在 250108）。
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

fn beam(set: &Arc<DbSet>, w1: u32, what: &str) -> Box<GeneratedElement> {
    let el = set.element(RefNo::new(2013286654, w1));
    match model_element(&el, classify("GENSEC")).unwrap() {
        Modeled::Solid(generated) => generated,
        Modeled::Skipped { reason, .. } => panic!("{what} 没出实体：{reason}"),
    }
}

/// 端板梁：世界包围盒逐分量对 E3D RVM（RVM 值取到 0.1 mm），关节负体个数钉住。
#[test]
fn joint_negatives_trim_end_plate_beams_like_e3d() {
    if !fixtures_present() {
        return;
    }
    let set = aps_set();
    let cases: [(u32, [f64; 6], usize, &str); 2] = [
        (
            261,
            [-302985.1, 326433.4, 103596.8, -298514.9, 326566.6, 103800.0],
            10,
            "GENSEC 2 of /B01_LEVEL_01_BEAMS（两端 Flexible End Plate：NBOX + 4 NCYL）",
        ),
        (
            272,
            [-303073.6, 318668.2, 103540.4, -302926.4, 326331.8, 103800.0],
            14,
            "GENSEC 8 of /B01_LEVEL_01_BEAMS（两端 Full Depth End Plate）",
        ),
    ];
    for (w1, rvm, negatives, what) in cases {
        let generated = beam(&set, w1, what);
        let bb = generated.solid.bounding_box().expect("梁非空");
        let (lo, hi) = (bb.min(), bb.max());
        let got = [lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]];
        for (axis, (g, r)) in got.iter().zip(rvm).enumerate() {
            assert!(
                (g - r).abs() <= 0.1,
                "{what}：包围盒第 {axis} 分量 {g:.3}，E3D RVM {r:.1}（全部 {got:?}）"
            );
        }
        assert_eq!(generated.negatives_applied, negatives, "{what}");
        assert!(generated.negatives_skipped.is_empty(), "{what}");
    }
}

/// 斜梁 GENSEC 1 of /AREA02_PR_AXIS_P2.1：JOIS 是 Web Cleats（10 mm 让位 + 穿腹板的螺栓孔），
/// JOIE 的 FIXING 在另一个库（250108），Full Depth End Plate（8 mm）。梁斜 1.03 mm / 4676.8 mm，
/// 翼缘尖在刀的侧面外漂 0.002 mm，两边都留一条零厚尖条把包围盒撑开（E3D 撑回原长，我们的让刀
/// 把起点推到 4.5 mm），所以这根不比包围盒，比 E3D facets 里的切面：y = 314448.4（10 mm 切面）、
/// 309789.6（8 mm 切面），以及两只腹板螺栓孔（y = 314408.4 ± 11）。
#[test]
fn joint_negatives_cut_the_skewed_web_cleat_beam_on_e3d_planes() {
    if !fixtures_present() {
        return;
    }
    let set = aps_set();
    let generated = beam(&set, 1819, "GENSEC 1 of /AREA02_PR_AXIS_P2.1");
    let mesh = generated.mesh();
    let on_plane = |y: f64| mesh.vertices.iter().any(|v| (v[1] - y).abs() <= 0.01);
    for (y, what) in [
        (314448.4, "JOIS Web Cleats 的 10 mm 让位切面"),
        (
            309789.6,
            "JOIE（跨库 FIXING）Full Depth End Plate 的 8 mm 切面",
        ),
        (314397.4, "腹板螺栓孔一侧"),
        (314419.4, "腹板螺栓孔另一侧"),
    ] {
        assert!(on_plane(y), "网格里没有落在 y = {y} 的顶点：{what}");
    }
    assert_eq!(generated.negatives_applied, 12);
    assert_eq!(generated.genus(), 2, "两只穿腹板的螺栓孔");
}

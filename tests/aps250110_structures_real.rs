//! 真库门：aps250110 里此前不出的土建件（修复计划 W7c），世界包围盒对 E3D TTY 导出。
//!
//! 真值是 gen-model `verification/civil-tty-20261021/aps250110_lv6_2013286654_2.rvm`（E3D 3.1 只读导出，
//! `output/rvm_group_aabb.py` 量的世界包围盒，mm）。
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

fn aps_set() -> Arc<DbSet> {
    let template = Path::new(TEMPLATE_DIR);
    let resolver = DirectoryDbResolver::scan(
        &[PathBuf::from(PLANT_DIR), PathBuf::from(ACP_DIR)],
        template,
    )
    .unwrap();
    let set = Arc::new(
        DbSet::with_attlib_file_and_resolver(template.join("attlib.dat"), Box::new(resolver))
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

/// 按 noun 建一件，世界包围盒逐分量对 E3D。
fn assert_world_box(set: &Arc<DbSet>, noun: &str, cases: &[(u32, [f64; 6], &str)]) {
    for (w1, e3d, what) in cases {
        let el = set.element(RefNo::new(2013286654, *w1));
        let generated = match model_element(&el, classify(noun)).unwrap() {
            Modeled::Solid(generated) => generated,
            Modeled::Skipped { reason, .. } => panic!("{what} 没出实体：{reason}"),
        };
        let bb = generated.solid.bounding_box().expect("非空");
        let (lo, hi) = (bb.min(), bb.max());
        let got = [lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]];
        for (axis, (g, e)) in got.iter().zip(e3d).enumerate() {
            assert!(
                (g - e).abs() <= 0.06,
                "{what}：世界包围盒第 {axis} 分量 {g:.3}，E3D {e:.2}（全部 {got:?}）"
            );
        }
    }
}

/// `CWALL` 名下的两堵圆弧外墙：SPRE 是 `/100_thk_wall`，厚在目录 `PARA`、DESP 只有高一项
/// （4000 / 3800），此前报「DESP length 1 < 2」整件失败。
#[test]
fn single_desp_walls_take_their_section_from_the_catalogue() {
    if !fixtures_present() {
        return;
    }
    let set = aps_set();
    assert_world_box(
        &set,
        "WALL",
        &[
            (
                205,
                [
                    -312654.61, 316366.07, 100000.00, -298345.39, 318309.00, 104000.00,
                ],
                "WALL 1 of CWALL /B01_LEV01_EXT_WALLS",
            ),
            (
                397,
                [
                    -312654.61, 316366.07, 104000.00, -298345.39, 318309.00, 107800.00,
                ],
                "WALL 1 of CWALL /B01_LEV02_EXT_WALLS",
            ),
        ],
    );
}

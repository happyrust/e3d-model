//! 真库门：aps250162 的 75 只 GENPRI（`/MEI-JCAN01`–`05` 各 15 只）全部建成封闭实体，局部包围盒
//! 对 E3D TTY 导出的面片。
//!
//! 真值是 gen-model `verification/civil-tty-20261021/aps_survey_lv6_2013286706_1037.rvm`（E3D 3.1 只读
//! 导出）里 `GENPRI n of EQUIPMENT /MEI-JCAN01` 的 FacetGroup：矩阵平移是设备原点，顶点在设备局部系
//! （mm），GENPRI 自己的 POS / ORI 全是 0。
//!
//! 缺 E3D 安装或影子资产默认红（与 `increment_real.rs` 同一口径）；无语料环境显式设
//! `E3D_SKIP_REAL_GATES=1` 换回跳过。

use std::path::{Path, PathBuf};
use std::sync::Arc;

use e3d_io::db_element::{DbFilePin, DbSet, template_file_for};
use e3d_io::refno::RefNo;
use e3d_io::{ReadOnlyEngine, ScanTier};
use e3d_model::category::classify;
use e3d_model::db_discovery::DirectoryDbResolver;
use e3d_model::elmodl::{Modeled, model_element};
use glam::DVec3;

const PLANT_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaPlantSample\aps000";
const ACP_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaCatalogue\acp000";
const DESIGN_DB: &str = "aps250162_0001";
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

#[test]
fn every_genpri_of_aps250162_closes_into_a_solid() {
    if !fixtures_present() {
        return;
    }
    let set = aps_set();
    let mut engine = ReadOnlyEngine::open(Path::new(PLANT_DIR).join(DESIGN_DB)).unwrap();
    let refnos: Vec<RefNo> = engine
        .scan_elements(ScanTier::Header)
        .unwrap()
        .filter_map(|scanned| scanned.ok())
        .map(|scanned| scanned.refno)
        .collect();
    let mut built = 0;
    for refno in refnos {
        let el = set.element(refno);
        if el.element_type().unwrap() != "GENPRI" {
            continue;
        }
        match model_element(&el, classify("GENPRI")).unwrap() {
            Modeled::Solid(generated) => {
                assert!(generated.solid.num_tri() > 0, "{refno}");
                built += 1;
            }
            Modeled::Skipped { reason, .. } => panic!("{refno} 没出实体：{reason}"),
        }
    }
    assert_eq!(built, 75);
}

/// 局部包围盒逐分量对 E3D（取到 0.01 mm）：斜放的短管 GENPRI 1、细圆柱 GENPRI 6、板边 R5 半圆不切的
/// GENPRI 10 是 exact，端面圆弧交叉的 GENPRI 13 在弦高档。
#[test]
fn genpri_local_boxes_match_the_e3d_export() {
    if !fixtures_present() {
        return;
    }
    let set = aps_set();
    let cases: [(u32, [f64; 6], f64, &str); 4] = [
        (
            1039,
            [252.75, 60.37, 154.07, 342.75, 143.45, 198.01],
            0.06,
            "GENPRI 1 of /MEI-JCAN01",
        ),
        (
            1078,
            [257.75, 60.17, 242.85, 337.75, 66.07, 248.75],
            0.06,
            "GENPRI 6 of /MEI-JCAN01",
        ),
        (
            1092,
            [335.75, 55.03, 184.79, 337.75, 119.28, 253.62],
            0.06,
            "GENPRI 10 of /MEI-JCAN01",
        ),
        (
            1103,
            [259.75, 54.80, 214.26, 335.75, 83.82, 254.11],
            0.5,
            "GENPRI 13 of /MEI-JCAN01",
        ),
    ];
    for (w1, e3d, tolerance, what) in cases {
        let el = set.element(RefNo::new(2013286706, w1));
        let Modeled::Solid(generated) = model_element(&el, classify("GENPRI")).unwrap() else {
            panic!("{what} 没出实体");
        };
        let to_local = generated.world.inverse();
        let mesh = generated.mesh();
        let mut lo = DVec3::INFINITY;
        let mut hi = DVec3::NEG_INFINITY;
        for v in &mesh.vertices {
            let p = to_local.transform_point3(DVec3::new(v[0], v[1], v[2]));
            lo = lo.min(p);
            hi = hi.max(p);
        }
        let got = [lo.x, lo.y, lo.z, hi.x, hi.y, hi.z];
        for (axis, (g, e)) in got.iter().zip(e3d).enumerate() {
            assert!(
                (g - e).abs() <= tolerance,
                "{what}: 局部包围盒第 {axis} 分量 {g:.3}，E3D {e:.2}（全部 {got:?}）"
            );
        }
    }
}

//! 真库门：SCTN 名下的 FITT 自己成件，标架与 E3D 一致（`category::is_model_unit_of` +
//! `section::section_fitting_placement` + `catalogue::model_catalogue_member`）。
//!
//! 真值是 E3D TTY 只读导出的 RVM（gen-model `test_data/rvm/KX-CAM-S4001.rvm`，STRU /-KX-CAM-S4001，
//! ams7997）：E3D 给每只 FITT 各开一组、梁的组里只有梁——`/-KX-CAM-S4001-V1/F1` 是自己一条
//! Box 270 × 20 × 270，板心落在 L40 角钢截面包围盒的中心。`SCTN-HW150-C80.rvm` 同型：
//! `/1TFA004MP-SUPP-01-V1` 的组里只有梁那一条面片，`FITTING 1 / 2 of SCTN …` 各自一组。
//!
//! 这两只角钢的截面目录（db 7600 的会话 12）本机没有，`section_fitting_placement` 走的是**兜底**
//! 那一档（装在中性轴、径向 π/2）；此前兜底按 0 算，板绕梁轴转了 90°，y / z 各差 17.4 mm。
//! 兜底口径就该与解得开时 `POSL = NA` 那一档相同，这里拿 E3D 的板钉住它。
//!
//! 缺 E3D 安装或影子资产默认红（与 `increment_real.rs` 同一口径）；无语料环境显式设
//! `E3D_SKIP_REAL_GATES=1` 换回跳过。

use std::path::{Path, PathBuf};
use std::sync::Arc;

use e3d_io::db_element::{DbFilePin, DbSet, template_file_for};
use e3d_io::refno::RefNo;
use e3d_model::catalogue::model_catalogue_member;
use e3d_model::category::is_model_unit_of;
use e3d_model::db_discovery::DirectoryDbResolver;

const AMS_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaMarineSample\ams000";
const ACP_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaCatalogue\acp000";
const DESIGN_DB: &str = "ams7997_0001";
const TEMPLATE_DIR: &str = r"E:\reverse\e3d\shadow_e3d31_aps_all";

fn fixtures_present() -> bool {
    fixtures_present_for(DESIGN_DB)
}

fn fixtures_present_for(design_db: &str) -> bool {
    let design = Path::new(AMS_DIR).join(design_db);
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

/// ams7997 设计库 + 扫 ams000 / acp000 的 resolver（端板 SFIT `/M8XP1` 在 13244；角钢截面的
/// 7600 本机没有对得上的会话，SCTN 记 chain_broken、FITT 走兜底标架）。
fn ams_set() -> Arc<DbSet> {
    ams_set_for(DESIGN_DB)
}

fn ams_set_for(design_db: &str) -> Arc<DbSet> {
    let template = Path::new(TEMPLATE_DIR);
    let resolver =
        DirectoryDbResolver::scan(&[PathBuf::from(AMS_DIR), PathBuf::from(ACP_DIR)], template)
            .unwrap();
    let set = Arc::new(
        DbSet::with_attlib_file_and_resolver(&template.join("attlib.dat"), Box::new(resolver))
            .unwrap(),
    );
    set.add_db(DbFilePin {
        file: Path::new(AMS_DIR).join(design_db),
        template: template_file_for(template, "DESI").unwrap(),
        db_type: Some("DESI".into()),
        sesno: None,
    })
    .unwrap();
    set
}

/// 三块端板：世界包围盒逐分量对 E3D RVM 的 Box（`KX-CAM-S4001.rvm` 两块取到 0.1 mm；
/// `/-KX-CAM-S4003-BAR-1/F1` 的真值是 `section::tests::fitting_mount_matches_cam_native_axes_and_origin`
/// 钉的那只 RVM Box 的中心）。SCTN 名下的 FITT 得先是单元，否则全量根本不会为它出这一件。
#[test]
fn section_fittings_stand_alone_where_e3d_draws_them() {
    if !fixtures_present() {
        return;
    }
    let set = ams_set();
    let cases: [(u32, [f64; 6], &str); 3] = [
        (
            47164,
            [-14570.0, -44825.0, 5830.0, -14550.0, -44555.0, 6100.0],
            "/-KX-CAM-S4001-V1/F1（Box 270 × 20 × 270，板心在角钢包围盒中心）",
        ),
        (
            47166,
            [-14570.0, -45315.0, 5830.0, -14550.0, -45045.0, 6100.0],
            "/-KX-CAM-S4001-V3/F1",
        ),
        (
            47188,
            [-18835.0, -45285.0, 6159.92, -18725.0, -45260.0, 6529.92],
            "/-KX-CAM-S4003-BAR-1/F1（370 × 25 × 110 的板横在梁端，中心 (-18780, -45272.5, 6344.92)）",
        ),
    ];
    for (w1, rvm, what) in cases {
        let fitting = set.element(RefNo::new(24381, w1));
        assert!(
            is_model_unit_of(&fitting).unwrap(),
            "{what}：SCTN 名下的 FITT 是自己一件"
        );
        let generated = model_catalogue_member(&fitting)
            .unwrap()
            .generated
            .unwrap_or_else(|| panic!("{what} 没出实体"));
        let mesh = generated.mesh();
        let (lo, hi) = mesh.aabb().expect("端板非空");
        let got = [lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]];
        for (axis, (g, r)) in got.iter().zip(rvm).enumerate() {
            assert!(
                (g - r).abs() <= 0.1,
                "{what}：包围盒第 {axis} 分量 {g:.3}，E3D RVM {r:.2}（全部 {got:?}）"
            );
        }
    }
    // 端板挂在梁下、两边各是一件：SCTN 不再把 FITT 的几何拼进 parts。角钢截面目录本机缺席，
    // 这里只能钉「两边各是单元」这一半；梁的组里只有梁那一半由 aps250110 的 STWALL 门窗
    // （57 / 57）与 `SCTN-HW150-C80.rvm` 的 `/1TFA004MP-SUPP-01-V1`（一条面片、两只空 FITTING 组）盯着。
    let section = set.element(RefNo::new(24381, 47163));
    assert_eq!(section.element_type().unwrap(), "SCTN");
    assert!(is_model_unit_of(&section).unwrap(), "SCTN 自己照旧是单元");
    let owner = set
        .element(RefNo::new(24381, 47164))
        .owner()
        .unwrap()
        .expect("端板有属主");
    assert_eq!(
        owner.refno(),
        section.refno(),
        "端板挂在 /-KX-CAM-S4001-V1 下"
    );
}

/// 截面 `SPRE` 在开着的库里悬空：ams8000 `/1CSV-HVACHB` 暖通支架的角钢指向 db 5052 里已不存在的
/// 元素，E3D 的梁只画一条中心线，端板的径向落在截面 x 上（`section_fitting_placement` 的悬空那一档）。
/// 真值是 E3D TTY 只读导出的 Box 世界包围盒（gen-model `verification/civil-tty-20261021/`
/// `ams8000_lv6_24384_25586.rvm` / `ams8000_lv6_24384_24945.rvm`）。按库没开那档的 π/2 算，
/// 270 × 110 那块绕梁轴转了 90°（差 110 mm），方板的板心偏 19.8 mm。
#[test]
fn fittings_on_a_section_with_a_dangling_spre_turn_like_e3d() {
    if !fixtures_present_for("ams8000_0001") {
        return;
    }
    let set = ams_set_for("ams8000_0001");
    let cases: [(u32, [f64; 6], &str); 2] = [
        (
            25650,
            [
                29824.102, -8158.805, 16949.999, 29996.153, -7870.214, 16969.999,
            ],
            "/-SR-CSV-S6137-V4/F1（Box 270 × 20 × 110，竖杆向下）",
        ),
        (
            24951,
            [28122.853, -6692.617, 8480.0, 28450.152, -6365.318, 8500.0],
            "/-SR-CSV-S5001-V3/F1（Box 270 × 20 × 270，竖杆向上）",
        ),
    ];
    for (w1, rvm, what) in cases {
        let fitting = set.element(RefNo::new(24384, w1));
        assert!(is_model_unit_of(&fitting).unwrap(), "{what}");
        let generated = model_catalogue_member(&fitting)
            .unwrap()
            .generated
            .unwrap_or_else(|| panic!("{what} 没出实体"));
        let (lo, hi) = generated.mesh().aabb().expect("端板非空");
        let got = [lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]];
        for (axis, (g, r)) in got.iter().zip(rvm).enumerate() {
            assert!(
                (g - r).abs() <= 0.1,
                "{what}：包围盒第 {axis} 分量 {g:.3}，E3D RVM {r:.3}（全部 {got:?}）"
            );
        }
    }
}

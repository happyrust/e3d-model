//! 真库门：aps7200 里三种现算标架与 E3D 一致（经 `transform::world_matrix`）。
//!
//! - 踢脚板 GENSEC 的 JLDATU 按 `PKDI` 比例站位（`datum::solve`）：`/EAST-KICKPLATE` 是直梁、
//!   `/OUTER-KICKPLATE` 是 165° 的三点圆弧，两只都是 `PKDI = 0.5`、`ZDIS = 0`。
//! - `/INNER-KICKPLATE` 的 `CURVE`（`CURTYP CENT`）两端落在 `RADI` 圆内（`section::centre_arc_through`），
//!   它的 JLDATU 是 `ZDIS = 1531`。
//! - `PANE` 上 `CMPF` 名下的 `SBFI` 贴在 CMPF `SJUS` 指的板面（`transform::ImplicitFrame::PanelSubFitting`）。
//!
//! 真值是 E3D TTY 只读宏在 APS 项目副本上逐件打出的 `POS/ORI WRT WORLD`（gen-model
//! `verification/civil-tty-20261021/aps7200-w7k-truth/aps7200_w7k_truth.psv`）。aps7200 不在 APS 的
//! MDB `/ALL` 里，E3D 只能在副本上导；设计库文件照样在 aps000。
//!
//! 缺 E3D 安装或影子资产默认红（与 `increment_real.rs` 同一口径）；无语料环境显式设
//! `E3D_SKIP_REAL_GATES=1` 换回跳过。

use std::path::{Path, PathBuf};
use std::sync::Arc;

use e3d_io::db_element::{DbFilePin, DbSet, template_file_for};
use e3d_io::refno::RefNo;
use e3d_model::db_discovery::DirectoryDbResolver;
use e3d_model::transform::world_matrix;
use glam::{DMat4, DVec3};

const APS_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaPlantSample\aps000";
const ACP_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaCatalogue\acp000";
const DESIGN_DB: &str = "aps7200_0001";
const TEMPLATE_DIR: &str = r"E:\reverse\e3d\shadow_e3d31_aps_all";
/// aps7200 在 refno 里的库号。
const DB_REFNO: u32 = 23584;

fn fixtures_present() -> bool {
    let attlib = Path::new(TEMPLATE_DIR).join("attlib.dat");
    let design = Path::new(APS_DIR).join(DESIGN_DB);
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

/// aps7200 + 扫 aps000 / acp000 的 resolver。
fn aps_set() -> Arc<DbSet> {
    let template = Path::new(TEMPLATE_DIR);
    let resolver =
        DirectoryDbResolver::scan(&[PathBuf::from(APS_DIR), PathBuf::from(ACP_DIR)], template)
            .unwrap();
    let set = Arc::new(
        DbSet::with_attlib_file_and_resolver(&template.join("attlib.dat"), Box::new(resolver))
            .unwrap(),
    );
    set.add_db(DbFilePin {
        file: Path::new(APS_DIR).join(DESIGN_DB),
        template: template_file_for(template, "DESI").unwrap(),
        db_type: Some("DESI".into()),
        sesno: None,
    })
    .unwrap();
    set
}

/// (w1, E3D 平移, E3D X 列, Y 列, Z 列, 平移容差 mm, 轴容差, 说明)
type FrameCase = (
    u32,
    [f64; 3],
    [f64; 3],
    [f64; 3],
    [f64; 3],
    f64,
    f64,
    &'static str,
);

fn assert_frame(world: DMat4, case: &FrameCase) {
    let (_, translation, x_axis, y_axis, z_axis, position_tol, axis_tol, what) = *case;
    let got_t = world.w_axis.truncate();
    let want_t = DVec3::from_array(translation);
    assert!(
        (got_t - want_t).abs().max_element() <= position_tol,
        "{what}：平移 {got_t:.4}，E3D TTY {want_t:.4}（差 {:.4}）",
        got_t - want_t
    );
    for (name, got, want) in [
        ("X", world.x_axis.truncate(), DVec3::from_array(x_axis)),
        ("Y", world.y_axis.truncate(), DVec3::from_array(y_axis)),
        ("Z", world.z_axis.truncate(), DVec3::from_array(z_axis)),
    ] {
        assert!(
            (got - want).abs().max_element() <= axis_tol,
            "{what}：{name} 列 {got:.9}，E3D TTY {want:.9}"
        );
    }
}

fn check(cases: &[FrameCase]) {
    let set = aps_set();
    for case in cases {
        let world = world_matrix(&set.element(RefNo::new(DB_REFNO, case.0))).unwrap();
        assert_frame(world, case);
    }
}

/// `PKDI` 是按路径全长的比例：直梁 1020 mm 的 `PKDI = 0.5` 在 510 mm 处，165° 圆弧上的在弧长中点。
/// 同一根直梁上 `ZDIS = 820`、`PKDI = 0` 的那只照旧只走 `ZDIS`。
#[test]
fn kick_plate_datums_take_their_proportional_distance() {
    if !fixtures_present() {
        return;
    }
    check(&[
        (
            8317,
            [6669.4351, 8520.8787, 21695.0],
            [-0.258819045, -0.965925826, 0.0],
            [0.0, 0.0, -1.0],
            [0.965925826, -0.258819045, 0.0],
            0.01,
            1e-6,
            "JLDATUM 1 of GENSEC /EAST-KICKPLATE（PKDI 0.5）",
        ),
        (
            8320,
            [6968.8721, 8440.6448, 21695.0],
            [-0.258819045, -0.965925826, 0.0],
            [0.0, 0.0, -1.0],
            [0.965925826, -0.258819045, 0.0],
            0.01,
            1e-6,
            "JLDATUM 2 of GENSEC /EAST-KICKPLATE（ZDIS 820）",
        ),
        (
            8305,
            [5118.5545, 7015.8260, 21705.0],
            [0.130502468, 0.991447984, 0.0],
            [0.0, 0.0, -1.0],
            [-0.991447984, 0.130502468, 0.0],
            0.01,
            1e-6,
            "JLDATUM 1 of GENSEC /OUTER-KICKPLATE（圆弧上的 PKDI 0.5）",
        ),
    ]);
}

/// `CENT` 的两端距圆心 850.06 / 849.67，都在 `RADI = 1050` 的圆内：E3D 沿以 `POS` 为圆心、半径取终点
/// 距离的劣弧量 `ZDIS`。按切线构造时这里退成直段，站位差 0.7 m。
#[test]
fn a_centre_curve_with_its_ends_inside_radi_runs_through_its_ends() {
    if !fixtures_present() {
        return;
    }
    check(&[(
        8309,
        [4966.8009, 8096.7907, 21705.0],
        [0.462769810, 0.886478484, 0.0],
        [0.0, 0.0, -1.0],
        [-0.886478484, 0.462769810, 0.0],
        0.05,
        1e-4,
        "JLDATUM 1 of GENSEC /INNER-KICKPLATE（CENT，ZDIS 1531）",
    )]);
}

/// 30 mm 楼板上 `SJUS UTOP` 的 CMPF 在板底，它名下的 SBFI 在板顶。
#[test]
fn a_sub_fitting_of_a_panel_compound_fitting_sits_on_its_surface() {
    if !fixtures_present() {
        return;
    }
    check(&[
        (
            6547,
            [9630.0017, 8855.8016, 5880.0],
            [-0.000192021, 0.999999982, 0.0],
            [-0.999999982, -0.000192021, 0.0],
            [0.0, 0.0, 1.0],
            0.01,
            1e-6,
            "CMPFITTING 1 of PANEL 1 of FRMWORK /FLR-PLT1",
        ),
        (
            6548,
            [9630.0017, 8855.8016, 5910.0],
            [-0.000192021, 0.999999982, 0.0],
            [-0.999999982, -0.000192021, 0.0],
            [0.0, 0.0, 1.0],
            0.01,
            1e-6,
            "SBFITTING 1 of CMPFITTING 1 of PANEL 1 of FRMWORK /FLR-PLT1",
        ),
    ]);
}

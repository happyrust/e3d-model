//! 真库门：WALL 名下 `JLDATU → PLDATU(POSL=OBOW / IBOW) → FIXING` 的隐式标架与 E3D 一致
//! （`elmodl::fixing_host_frame` + `elmodl::fixing_local_matrix`，经 `catalogue::model_catalogue_member`）。
//!
//! 真值是 E3D TTY 只读导出的 RVM（gen-model `test_data/rvm/1RS-WF03-W-C-RR001.rvm`，CWALL
//! `/1RS-WF03-W-C-RR001`，ams1112）：其中 38 只穿墙套管 FIXING 的记录矩阵与平移。E3D 把 PLDATUM
//! 的原点放在 JLDATUM 的站位上（高差 0），FIXING 的 `POS` 三分量全部从那里量——38/38 满足
//! `|POS| = |T_fixi − T_pldatu|`、竖向 = `−POS.x`、水平模长相等，最大残差 0.001 mm
//! （plant-model-5 `outputs/rvm-check-20261010/wall-1112/_hypothesis.py`）。
//!
//! 修前这 38 只全部 off 0.8–12.2 m：09-28 为 aps250110 门窗（`POSL=IBOW`）拟合的那套
//! 「PLDATU 抬高 `(height−600).max(3400)` + 清 FIXING `POS[1]/POS[2]`」被套在了 OBOW 上。
//!
//! 缺 E3D 安装或影子资产默认红（与 `increment_real.rs` 同一口径）；无语料环境显式设
//! `E3D_SKIP_REAL_GATES=1` 换回跳过。

use std::path::{Path, PathBuf};
use std::sync::Arc;

use e3d_io::db_element::{DbFilePin, DbSet, template_file_for};
use e3d_io::refno::RefNo;
use e3d_model::catalogue::model_catalogue_member;
use e3d_model::db_discovery::DirectoryDbResolver;
use glam::{DMat4, DVec3};

const AMS_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaMarineSample\ams000";
const APS_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaPlantSample\aps000";
const ACP_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaCatalogue\acp000";
const DESIGN_DB: &str = "ams1112_0001";
const DESIGN_DB_7997: &str = "ams7997_0001";
const DESIGN_DB_APS: &str = "aps250110_0001";
const TEMPLATE_DIR: &str = r"E:\reverse\e3d\shadow_e3d31_aps_all";

fn fixtures_present() -> bool {
    fixtures_present_for(&[DESIGN_DB])
}

fn fixtures_present_for(dbs: &[&str]) -> bool {
    fixtures_present_in(AMS_DIR, dbs)
}

fn fixtures_present_in(dir: &str, dbs: &[&str]) -> bool {
    let design: Vec<PathBuf> = dbs.iter().map(|db| Path::new(dir).join(db)).collect();
    let attlib = Path::new(TEMPLATE_DIR).join("attlib.dat");
    let mut missing: Vec<String> = design
        .iter()
        .filter(|path| !path.exists())
        .map(|path| path.display().to_string())
        .collect();
    for extra in [Path::new(ACP_DIR), attlib.as_path()] {
        if !extra.exists() {
            missing.push(extra.display().to_string());
        }
    }
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

/// ams 设计库 + 扫 ams000 / acp000 的 resolver（套管 SPRE 在 ams5052，墙型材 SPRE 在 acp7001）。
fn ams_set_db(design_db: &str) -> Arc<DbSet> {
    project_set_db(AMS_DIR, design_db)
}

/// `dir` 里的设计库 + 扫 `dir` / acp000 的 resolver。
fn project_set_db(dir: &str, design_db: &str) -> Arc<DbSet> {
    let template = Path::new(TEMPLATE_DIR);
    let resolver =
        DirectoryDbResolver::scan(&[PathBuf::from(dir), PathBuf::from(ACP_DIR)], template).unwrap();
    let set = Arc::new(
        DbSet::with_attlib_file_and_resolver(&template.join("attlib.dat"), Box::new(resolver))
            .unwrap(),
    );
    set.add_db(DbFilePin {
        file: Path::new(dir).join(design_db),
        template: template_file_for(template, "DESI").unwrap(),
        db_type: Some("DESI".into()),
        sesno: None,
    })
    .unwrap();
    set
}

fn ams_set() -> Arc<DbSet> {
    ams_set_db(DESIGN_DB)
}

/// 四只套管（三堵不同的 WALL、两种 ORI 姿态）：实例世界矩阵的平移与三列对 E3D RVM 的
/// FacetGroup 记录（`translation_mm`、`matrix3 × 1000`）。平移取到 0.01 mm，列取到 1e-6。
#[test]
fn obow_wall_fixings_sit_where_e3d_puts_them() {
    if !fixtures_present() {
        return;
    }
    let set = ams_set();
    // (w1, E3D 平移, E3D X 列, Y 列, Z 列, 说明)
    let cases: [(u32, [f64; 3], [f64; 3], [f64; 3], [f64; 3], &str); 4] = [
        (
            137183,
            [-15559.963, -2743.641, 2150.0],
            [-0.173648179, 0.984807732, 0.0],
            [0.0, 0.0, 1.0],
            [0.984807732, 0.173648179, 0.0],
            "/1RS03TT3502T（WALL 17496/105912，POS.x = −ZDIS = −2550，套管轴水平）",
        ),
        (
            137995,
            [15893.803, 2233.7283, 1900.0],
            [0.139173106, -0.990268076, 0.0],
            [0.0, 0.0, 1.0],
            [-0.990268076, -0.139173106, 0.0],
            "/1RS03CC0008T（WALL 17496/105930，Ø240 × 1400）",
        ),
        (
            124140,
            [-15112.144, -8725.0, 2500.0],
            [0.0, 0.0, 1.0],
            [-0.500000024, 0.866025395, 0.0],
            [-0.866025395, -0.500000024, 0.0],
            "/1RS03LL0021T（WALL 17496/105935，X 列朝上的那种 ORI）",
        ),
        (
            124187,
            [-5071.725, -16853.416, 284.0],
            [0.0, 0.0, 1.0],
            [-0.9575804, 0.288166193, 0.0],
            [-0.288166193, -0.9575804, 0.0],
            "/1RS03EE7005T（WALL 17496/105940，|POS| 只有 447 mm 的近站位件）",
        ),
    ];
    for (w1, translation, x_axis, y_axis, z_axis, what) in cases {
        let fixing = set.element(RefNo::new(17496, w1));
        assert_eq!(fixing.element_type().unwrap(), "FIXING", "{what}");
        let generated = model_catalogue_member(&fixing)
            .unwrap()
            .generated
            .unwrap_or_else(|| panic!("{what} 没出实体"));
        let world = generated.world;
        let got_t = world.w_axis.truncate();
        let want_t = DVec3::from_array(translation);
        assert!(
            (got_t - want_t).abs().max_element() <= 0.01,
            "{what}：平移 {got_t:.3}，E3D RVM {want_t:.3}（差 {:.3}）",
            got_t - want_t
        );
        for (name, got, want) in [
            ("X", world.x_axis.truncate(), DVec3::from_array(x_axis)),
            ("Y", world.y_axis.truncate(), DVec3::from_array(y_axis)),
            ("Z", world.z_axis.truncate(), DVec3::from_array(z_axis)),
        ] {
            assert!(
                (got - want).abs().max_element() <= 1e-6,
                "{what}：{name} 列 {got:.9}，E3D RVM {want:.9}"
            );
        }
    }
}

/// 负扫掠墙族：ams7997 `=24381/1776`（CWALL `/1RS-WF02-W-C-RR002`，spine R=16500，
/// sweep≈−180°）名下 6 只穿墙套管 FIXING——真值取自 E3D TTY 只读导出的
/// `wall_1776.rvm` 各 FacetGroup 记录矩阵（gen-model `verification/fixing-7997-20261019/`，
/// `native_fixings.json` 的 `transform_cols_m` × 1000）。
///
/// 这族墙暴露出两个正扫掠 1112 族盖不住的问题：JLDATU 站位在 spine 半径上（不是
/// 「spine+SPRF PARA[0]」——`Single_leaf_wall` 的 PARA[0]=10000 是名义参数不是壁厚），
/// 且 p-line 基的两列随扫掠方向翻转、FIXING `ORI` 的绕 y 分量跟着取负。修前 6 件
/// 全偏 ~10–42 m、朝向镜像。
#[test]
fn obow_negative_sweep_wall_fixings_sit_where_e3d_puts_them() {
    if !fixtures_present_for(&[DESIGN_DB_7997]) {
        return;
    }
    let set = ams_set_db(DESIGN_DB_7997);
    // (w1, E3D 平移, E3D X 列, Y 列, Z 列, 说明)
    let cases: [(u32, [f64; 3], [f64; 3], [f64; 3], [f64; 3], &str); 6] = [
        (
            1794,
            [-15557.533, -6178.354, 1700.0],
            [0.4202437, -0.9074113, 0.0],
            [0.0, 0.0, 1.0],
            [-0.9074113, -0.4202437, 0.0],
            "JLDATUM /1RS02KK1011T（ZDIS 1320，ORI[0,-69.75,-90]，Y 列朝上）",
        ),
        (
            1800,
            [-4481.073, 18413.069, 1700.0],
            [-0.9612618, -0.27563694, 0.0],
            [0.0, 0.0, 1.0],
            [-0.27563694, 0.9612618, 0.0],
            "JLDATUM /1RS02KK1015T（ZDIS 1320，ORI[180,-11.40,90]）",
        ),
        (
            1815,
            [-6245.18, -14736.508, 1200.0],
            [-0.8241263, 0.56640606, 0.0],
            [0.0, 0.0, -1.0],
            [-0.56640606, -0.8241263, 0.0],
            "JLDATUM /1RS02II0003T（ZDIS 1820，ORI[0,40.84,90]，Y 列朝下）",
        ),
        (
            1827,
            [-16039.04, 4159.048, 700.0],
            [0.0, 0.0, -1.0],
            [0.15988038, 0.9871364, 0.0],
            [0.9871364, -0.15988038, 0.0],
            "JLDATUM /1RS02TT0034T（ZDIS 2320，ORI[-72.72,0,180]，X 列朝下）",
        ),
        (
            1866,
            [-1055.5061, 18995.43, 1700.0],
            [-0.9976246, -0.06888548, 0.0],
            [0.0, 0.0, 1.0],
            [-0.06888548, 0.9976246, 0.0],
            "JLDATUM /1RS02LL0001T（ZDIS 1320，ORI[180,0.65,90]）",
        ),
        (
            1881,
            [-2834.3074, -15615.322, 500.0],
            [0.0, 0.0, -1.0],
            [0.9851994, -0.17141257, 0.0],
            [-0.17141257, -0.9851994, 0.0],
            "JLDATUM /1RS02LL0007T（ZDIS 2520，ORI[18.64,0,180]）",
        ),
    ];
    for (w1, translation, x_axis, y_axis, z_axis, what) in cases {
        let fixing = set.element(RefNo::new(24381, w1));
        assert_eq!(fixing.element_type().unwrap(), "FIXING", "{what}");
        let generated = model_catalogue_member(&fixing)
            .unwrap()
            .generated
            .unwrap_or_else(|| panic!("{what} 没出实体"));
        let world = generated.world;
        let got_t = world.w_axis.truncate();
        let want_t = DVec3::from_array(translation);
        assert!(
            (got_t - want_t).abs().max_element() <= 0.5,
            "{what}：平移 {got_t:.3}，E3D RVM {want_t:.3}（差 {:.3}）",
            got_t - want_t
        );
        for (name, got, want) in [
            ("X", world.x_axis.truncate(), DVec3::from_array(x_axis)),
            ("Y", world.y_axis.truncate(), DVec3::from_array(y_axis)),
            ("Z", world.z_axis.truncate(), DVec3::from_array(z_axis)),
        ] {
            assert!(
                (got - want).abs().max_element() <= 1e-6,
                "{what}：{name} 列 {got:.9}，E3D RVM {want:.9}"
            );
        }
    }
}

/// LMIRR 墙族：ams7997 `=24381/1597`（CWALL `/1RS-WF02-W-C-R2407`，`LMIRR=true`）名下两只穿墙套管
/// FIXING——真值是 E3D TTY 只读宏逐件打出的 `POS/ORI WRT WORLD`（gen-model
/// `verification/fixing-tty-20261021/frames_dbg_truth.psv`）。
///
/// 镜像墙的 PLDATU 在 JLDATU 系里是 (+J.Y, J.Z, +J.X)，即绕沿墙轴转了 180°。修前这两只竖向按
/// `POS.x` 翻号、水平镜像，偏 1.2–3 m、三轴反向。
#[test]
fn obow_mirrored_wall_fixings_sit_where_e3d_puts_them() {
    if !fixtures_present_for(&[DESIGN_DB_7997]) {
        return;
    }
    let set = ams_set_db(DESIGN_DB_7997);
    // (w1, E3D 平移, E3D X 列, Y 列, Z 列, 说明)
    let cases: [(u32, [f64; 3], [f64; 3], [f64; 3], [f64; 3], &str); 2] = [
        (
            1600,
            [-2927.7162, 9652.6324, 2620.0],
            [-0.93667308, -0.350205, 0.0],
            [0.0, 0.0, 1.0],
            [-0.350205, 0.93667308, 0.0],
            "JLDATUM /1RS02CC0008T（ZDIS 600）",
        ),
        (
            1609,
            [-5031.6354, 8413.2596, 1700.0],
            [0.0, 0.0, -1.0],
            [-0.798636877, -0.601813209, 0.0],
            [-0.601813209, 0.798636877, 0.0],
            "JLDATUM /1RS02LL0024T（ZDIS 1520，X 列朝下）",
        ),
    ];
    for (w1, translation, x_axis, y_axis, z_axis, what) in cases {
        let fixing = set.element(RefNo::new(24381, w1));
        assert_eq!(fixing.element_type().unwrap(), "FIXING", "{what}");
        let generated = model_catalogue_member(&fixing)
            .unwrap()
            .generated
            .unwrap_or_else(|| panic!("{what} 没出实体"));
        let world = generated.world;
        let got_t = world.w_axis.truncate();
        let want_t = DVec3::from_array(translation);
        assert!(
            (got_t - want_t).abs().max_element() <= 0.01,
            "{what}：平移 {got_t:.3}，E3D TTY {want_t:.3}（差 {:.3}）",
            got_t - want_t
        );
        for (name, got, want) in [
            ("X", world.x_axis.truncate(), DVec3::from_array(x_axis)),
            ("Y", world.y_axis.truncate(), DVec3::from_array(y_axis)),
            ("Z", world.z_axis.truncate(), DVec3::from_array(z_axis)),
        ] {
            assert!(
                (got - want).abs().max_element() <= 1e-6,
                "{what}：{name} 列 {got:.9}，E3D TTY {want:.9}"
            );
        }
    }
}

/// 实例世界矩阵对 E3D TTY 的 `POS/ORI WRT WORLD`：平移取到 0.01 mm，三列取到 1e-6。
fn assert_tty_frame(world: DMat4, translation: [f64; 3], axes: [[f64; 3]; 3], what: &str) {
    let got_t = world.w_axis.truncate();
    let want_t = DVec3::from_array(translation);
    assert!(
        (got_t - want_t).abs().max_element() <= 0.01,
        "{what}：平移 {got_t:.3}，E3D TTY {want_t:.3}（差 {:.3}）",
        got_t - want_t
    );
    let columns = [world.x_axis, world.y_axis, world.z_axis];
    for ((name, got), want) in ["X", "Y", "Z"].into_iter().zip(columns).zip(axes) {
        let (got, want) = (got.truncate(), DVec3::from_array(want));
        assert!(
            (got - want).abs().max_element() <= 1e-6,
            "{what}：{name} 列 {got:.9}，E3D TTY {want:.9}"
        );
    }
}

/// `POSL=IBOW` 墙族：ams7997 `/Copy-of-R211-M01`（`24381/2067`，顺时针墙 `24381/1932`，厚 900，
/// 目录「简图1-双开门」）。真值是 E3D TTY 只读宏打出的 `POS/ORI WRT WORLD` 与同一会话导出的
/// RVM（gen-model `verification/civil-tty-20261021/ibow_ams_truth.psv`、`ibow_ams.rvm`）。
///
/// PLDATU 原点在 IBOW p-line 上（离 JUSL 线一个壁厚），Z 沿 IBOW 的 PLAX，Y 取 PLDATU 的 `YDIR`。
/// 修前按 aps250110 拟合的「抬高 (DESP[0]−600).max(3400) + 清 POS[1]/POS[2]」放偏 1.9 m、X 轴反向。
///
/// 两块门扇再拿 RVM 的世界包围盒钉住：P8 / P12 的 PTCD `Y ( ATTRIB RPRO G ) Z` 里 `G` 是
/// `PTYP DIST`，E3D 求不出（`unable to evaluate catalogue expression for attribute PTCD`），方向回
/// FIXING 的 +Y，门扇贴在墙面里；把 G 当度数读，门扇就转出墙面（偏 750 mm）。
#[test]
fn ibow_wall_fixing_sits_where_e3d_puts_it() {
    if !fixtures_present_for(&[DESIGN_DB_7997]) {
        return;
    }
    let set = ams_set_db(DESIGN_DB_7997);
    let what = "/Copy-of-R211-M01";
    let fixing = set.element(RefNo::new(24381, 2067));
    assert_eq!(fixing.element_type().unwrap(), "FIXING", "{what}");
    let generated = model_catalogue_member(&fixing)
        .unwrap()
        .generated
        .unwrap_or_else(|| panic!("{what} 没出实体"));
    assert_tty_frame(
        generated.world,
        [14466.341, -8353.3382, 2900.0],
        [
            [0.0, 0.0, 1.0],
            [-0.568542971, -0.822653566, 0.0],
            [0.822653566, -0.568542971, 0.0],
        ],
        what,
    );
    let bb = generated.solid.bounding_box().expect("门扇非空");
    let (lo, hi) = (bb.min(), bb.max());
    let got = [lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]];
    let rvm = [13613.526, -9672.6, 1250.0, 14892.748, -7651.067, 2900.0];
    for (axis, (g, r)) in got.iter().zip(rvm).enumerate() {
        assert!(
            (g - r).abs() <= 0.01,
            "{what}：包围盒第 {axis} 分量 {g:.3}，E3D RVM {r:.3}（全部 {got:?}）"
        );
    }
}

/// 同一族的 aps250110 门窗：`/B01_LEV01_EXT_WALLS` 与 `/B01_LEV02_EXT_WALLS` 两堵外墙（墙厚 100）
/// 上的 5 只窗、1 只门（真值 gen-model `verification/civil-tty-20261021/ibow_aps_truth.psv`）。
#[test]
fn ibow_aps_wall_fixings_sit_where_e3d_puts_them() {
    if !fixtures_present_in(APS_DIR, &[DESIGN_DB_APS]) {
        return;
    }
    let set = project_set_db(APS_DIR, DESIGN_DB_APS);
    let window = "Window-Spec/DG-S/1200x1200";
    // (w1, E3D 平移, E3D X 列, Y 列, Z 列, 说明)
    let cases: [(u32, [f64; 3], [f64; 3], [f64; 3], [f64; 3], &str); 6] = [
        (
            208,
            [-309432.068, 316887.646, 101100.0],
            [0.0, 0.0, -1.0],
            [0.964307371, -0.264785373, 0.0],
            [-0.264785373, -0.964307371, 0.0],
            window,
        ),
        (
            211,
            [-301049.023, 317040.361, 101100.0],
            [0.0, 0.0, -1.0],
            [0.954024003, 0.299730216, 0.0],
            [0.299730216, -0.954024003, 0.0],
            window,
        ),
        (
            214,
            [-305479.528, 316357.624, 100000.0],
            [0.0, 0.0, -1.0],
            [0.999999049, 0.001378897, 0.0],
            [0.001378897, -0.999999049, 0.0],
            "Doors-Spec/EE-DOO-NS/1710x2045",
        ),
        (
            400,
            [-309432.023, 316887.81, 105100.0],
            [0.0, 0.0, -1.0],
            [0.964307371, -0.264785373, 0.0],
            [-0.264785373, -0.964307371, 0.0],
            window,
        ),
        (
            403,
            [-305479.528, 316357.704, 105100.0],
            [0.0, 0.0, -1.0],
            [0.999999049, 0.001378897, 0.0],
            [0.001378897, -0.999999049, 0.0],
            window,
        ),
        (
            406,
            [-301049.032, 317040.389, 105100.0],
            [0.0, 0.0, -1.0],
            [0.954024003, 0.299730216, 0.0],
            [0.299730216, -0.954024003, 0.0],
            window,
        ),
    ];
    for (w1, translation, x_axis, y_axis, z_axis, spec) in cases {
        let what = format!("=2013286654/{w1}（{spec}）");
        let fixing = set.element(RefNo::new(2013286654, w1));
        assert_eq!(fixing.element_type().unwrap(), "FIXING", "{what}");
        let generated = model_catalogue_member(&fixing)
            .unwrap()
            .generated
            .unwrap_or_else(|| panic!("{what} 没出实体"));
        assert_tty_frame(
            generated.world,
            translation,
            [x_axis, y_axis, z_axis],
            &what,
        );
    }
}

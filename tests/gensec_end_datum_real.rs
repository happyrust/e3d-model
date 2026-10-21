//! 真库门：GENSEC 端部 datum（`ENDATU`）的隐式标架与 E3D 一致（`datum::solve` 的 ENDATU 臂，
//! 经 `transform::world_matrix`），以及挂在它名下的 FIXING 实例标架（`catalogue::model_catalogue_member`）。
//!
//! 真值是 E3D TTY 只读宏逐件打出的 `POS/ORI WRT WORLD`（gen-model
//! `verification/fixing-tty-20261021/`：`datums_truth.psv`、`fixing_rest_truth.psv`）。ams7324 的
//! MDS 支架框（`/SUPPORT/FRAMES/BS/…`）每根 GENSEC 带一只 ENDATU，挂在哪一头看它自带的 `HANICP`。
//! 接上之前 ENDATU 没有隐式标架、按单位阵折进 owner 链：名下 FIXING 全丢了端部标架，挂在
//! 终点那头的还要再偏一个杆长。
//!
//! 缺 E3D 安装或影子资产默认红（与 `increment_real.rs` 同一口径）；无语料环境显式设
//! `E3D_SKIP_REAL_GATES=1` 换回跳过。

use std::path::{Path, PathBuf};
use std::sync::Arc;

use e3d_io::db_element::{DbFilePin, DbSet, template_file_for};
use e3d_io::refno::RefNo;
use e3d_model::catalogue::model_catalogue_member;
use e3d_model::db_discovery::DirectoryDbResolver;
use e3d_model::transform::world_matrix;
use glam::{DMat4, DVec3};

const AMS_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaMarineSample\ams000";
const ACP_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaCatalogue\acp000";
const DESIGN_DB: &str = "ams7324_0001";
const TEMPLATE_DIR: &str = r"E:\reverse\e3d\shadow_e3d31_aps_all";
/// ams7324 在 refno 里的库号。
const DB_REFNO: u32 = 23708;

fn fixtures_present() -> bool {
    let attlib = Path::new(TEMPLATE_DIR).join("attlib.dat");
    let design = Path::new(AMS_DIR).join(DESIGN_DB);
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

/// ams7324 + 扫 ams000 / acp000 的 resolver（支架型材的 SPRE 在别的目录库）。
fn ams_set() -> Arc<DbSet> {
    let template = Path::new(TEMPLATE_DIR);
    let resolver =
        DirectoryDbResolver::scan(&[PathBuf::from(AMS_DIR), PathBuf::from(ACP_DIR)], template)
            .unwrap();
    let set = Arc::new(
        DbSet::with_attlib_file_and_resolver(&template.join("attlib.dat"), Box::new(resolver))
            .unwrap(),
    );
    set.add_db(DbFilePin {
        file: Path::new(AMS_DIR).join(DESIGN_DB),
        template: template_file_for(template, "DESI").unwrap(),
        db_type: Some("DESI".into()),
        sesno: None,
    })
    .unwrap();
    set
}

/// (w1, E3D 平移, E3D X 列, Y 列, Z 列, 说明)
type FrameCase = (u32, [f64; 3], [f64; 3], [f64; 3], [f64; 3], &'static str);

fn assert_frame(world: DMat4, case: &FrameCase) {
    let (_, translation, x_axis, y_axis, z_axis, what) = *case;
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

/// 两头、两种 `POSL`：起点那头的三轴就是站位的截面放置标架（Z = 切向），终点那头绕截面 Y
/// 转 180°（Z = −切向），都让 Z 朝杆内。
#[test]
fn end_datums_sit_where_e3d_puts_them() {
    if !fixtures_present() {
        return;
    }
    let set = ams_set();
    let cases: [FrameCase; 5] = [
        (
            50469,
            [11979.5, 13950.0, 20.0],
            [0.0, -1.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
            "FT62/1-V1/PNOD（起点，spine 朝下、BANG −90°，POSL NA）",
        ),
        (
            50471,
            [9112.3, 14150.0, -200.0],
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            "FT64/1-V1/PNOD（起点，spine 朝上、BANG −90°，POSL NA）",
        ),
        (
            50289,
            [9050.0, 1165.5, 24737.0],
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            "FT56/5-V1/PNOD（终点，HANICP (0, 0, −313)，POSL NA）",
        ),
        (
            50305,
            [27200.0, 5010.5, 220.0],
            [0.0, -1.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
            "FT54A/1-FRMW1/PN1（终点，HANICP (0, 0, 300)，POSL BNA）",
        ),
        (
            50307,
            [27200.0, 5010.5, -220.0],
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            "FT54A/1-FRMW1/PN2（终点，HANICP (0, 0, −300)，POSL BNA）",
        ),
    ];
    for case in &cases {
        let datum = set.element(RefNo::new(DB_REFNO, case.0));
        assert_eq!(datum.element_type().unwrap(), "ENDATU", "{}", case.5);
        assert_frame(world_matrix(&datum).unwrap(), case);
    }
}

/// ENDATU 名下的关节 FIXING（GENSEC 的 `JOIS` 就指它）：实例标架 = ENDATU 标架 × 自己的 POS/ORI。
/// 修前 ENDATU 按单位阵，这两只的朝向就是自己的 `ORI`（位置碰巧对：起点与 GENSEC 原点重合）。
#[test]
fn end_datum_fixings_sit_where_e3d_puts_them() {
    if !fixtures_present() {
        return;
    }
    let set = ams_set();
    let cases: [FrameCase; 2] = [
        (
            50470,
            [11979.5, 13950.0, 20.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, -1.0],
            "FT62/1-V1/JOINT1（ORI [0, 0, 90]）",
        ),
        (
            50472,
            [9112.3, 14150.0, -200.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, -1.0],
            "FT64/1-V1/JOINT1（ORI [180, 0, −90]）",
        ),
    ];
    for case in &cases {
        let fixing = set.element(RefNo::new(DB_REFNO, case.0));
        assert_eq!(fixing.element_type().unwrap(), "FIXING", "{}", case.5);
        let generated = model_catalogue_member(&fixing)
            .unwrap()
            .generated
            .unwrap_or_else(|| panic!("{} 没出实体", case.5));
        assert_frame(generated.world, case);
    }
}

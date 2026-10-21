//! 真库门：GENSEC 端部 datum（`ENDATU`）的隐式标架与 E3D 一致（`datum::solve` 的 ENDATU 臂，
//! 经 `transform::world_matrix`），以及挂在它名下的 FIXING 实例标架（`catalogue::model_catalogue_member`）。
//!
//! 真值是 E3D TTY 只读宏逐件打出的 `POS/ORI WRT WORLD`（gen-model
//! `verification/fixing-tty-20261021/`：`datums_truth.psv`、`fixing_rest_truth.psv`）。ams7324 的
//! MDS 支架框（`/SUPPORT/FRAMES/BS/…`）每根 GENSEC 带一只 ENDATU，挂在哪一头看它自带的 `HANICP`。
//! 接上之前 ENDATU 没有隐式标架、按单位阵折进 owner 链：名下 FIXING 全丢了端部标架，挂在
//! 终点那头的还要再偏一个杆长。零长度 SPINE 上的 PLDATU（ams7330）、
//! 端部关节 FIXING 的目录负体（`JOIN` 的 NGMR）截杆、圆管圆周 p-line 上的 PLDATU 也在这里。
//!
//! 缺 E3D 安装或影子资产默认红（与 `increment_real.rs` 同一口径）；无语料环境显式设
//! `E3D_SKIP_REAL_GATES=1` 换回跳过。

use std::path::{Path, PathBuf};
use std::sync::Arc;

use e3d_io::db_element::{DbFilePin, DbSet, template_file_for};
use e3d_io::refno::RefNo;
use e3d_model::catalogue::model_catalogue_member;
use e3d_model::category::classify;
use e3d_model::db_discovery::DirectoryDbResolver;
use e3d_model::elmodl::{Modeled, model_element};
use e3d_model::transform::world_matrix;
use glam::{DMat4, DVec3};

const AMS_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaMarineSample\ams000";
const ACP_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaCatalogue\acp000";
const DESIGN_DB: &str = "ams7324_0001";
const DESIGN_DB_7330: &str = "ams7330_0001";
const TEMPLATE_DIR: &str = r"E:\reverse\e3d\shadow_e3d31_aps_all";
/// ams7324 在 refno 里的库号。
const DB_REFNO: u32 = 23708;
/// ams7330 在 refno 里的库号。
const DB_REFNO_7330: u32 = 23714;

fn fixtures_present() -> bool {
    fixtures_present_for(DESIGN_DB)
}

fn fixtures_present_for(design_db: &str) -> bool {
    let attlib = Path::new(TEMPLATE_DIR).join("attlib.dat");
    let design = Path::new(AMS_DIR).join(design_db);
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
    ams_set_db(DESIGN_DB)
}

fn ams_set_db(design_db: &str) -> Arc<DbSet> {
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

/// MDS 框 FT54A / FT55A 的短杆：端部 ENDATU 名下的关节 FIXING 走 `SPRE → CATR` 到 `JOIN`
/// （`/E3D/Joint-for-FT55/JOIN`，GMRE 未设、NGMR 一把 NSCY：Ø`DESP[1]` × `DESP[2]`，从端面往杆里
/// 截 100 mm）。E3D TTY RVM 里 300 mm 的杆只剩 200 mm；目录链在没有 GMRE 的 JOIN 上断掉时这把刀丢了，
/// BS 标准的 FT54A–D、FT55A–D 共 36 根杆都长了 100 mm。
#[test]
fn joint_catalogue_negatives_trim_mds_bars() {
    if !fixtures_present() {
        return;
    }
    let set = ams_set();
    // (w1, E3D RVM 世界包围盒 [lo; hi], 说明)
    let cases: [(u32, [f64; 6], &str); 2] = [
        (
            47418,
            [27200.0, 4975.0, -119.995, 27250.0, 5025.0, 80.0],
            "FT54A/1-BAR-2",
        ),
        (
            47609,
            [29150.0, 5075.0, -80.0, 29199.998, 5125.0, 119.995],
            "FT55A/1-BAR-1",
        ),
    ];
    for (w1, rvm, what) in cases {
        let el = set.element(RefNo::new(DB_REFNO, w1));
        let generated = match model_element(&el, classify("GENSEC")).unwrap() {
            Modeled::Solid(generated) => generated,
            Modeled::Skipped { reason, .. } => panic!("{what} 没出实体：{reason}"),
        };
        let bb = generated.solid.bounding_box().expect("杆非空");
        let (lo, hi) = (bb.min(), bb.max());
        let got = [lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]];
        for (axis, (g, r)) in got.iter().zip(rvm).enumerate() {
            assert!(
                (g - r).abs() <= 0.01,
                "{what}：包围盒第 {axis} 分量 {g:.3}，E3D RVM {r:.3}（全部 {got:?}）"
            );
        }
    }
}

/// SPINE 长度为 0 的 GENSEC（ams7330 MDS 特殊支架里的 PowerClick 接头杆，两只 POINSP 都在原点，
/// `BANG` = −90°）：E3D TTY 对它的 JLDATU 报求值失败，PLDATU 与名下 FIXING 却有标架——站位在
/// GENSEC 原点、截面标架取单位阵、不转 `BANG`，Z 沿 `POSL` NA 的 PLAX。修前两级都退成单位阵。
#[test]
fn stationless_pline_datums_sit_where_e3d_puts_them() {
    if !fixtures_present_for(DESIGN_DB_7330) {
        return;
    }
    let set = ams_set_db(DESIGN_DB_7330);
    // (w1, E3D 平移的 y, 说明)；四件的三列都是 (−X, +Z, +Y)。
    let cases = [
        (1294, 2250.0, "3-BAR-2/S1/PLDAT"),
        (1295, 2250.0, "3-BAR-2/S1/J1（PowerClick 接头）"),
        (1307, 3000.0, "4-BAR-2/S1/PLDAT"),
        (1309, 3000.0, "4-BAR-2/S1/J1（PowerClick 接头）"),
    ];
    for (w1, y, what) in cases {
        let case: FrameCase = (
            w1,
            [271.5, y, -31.5],
            [-1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
            what,
        );
        let element = set.element(RefNo::new(DB_REFNO_7330, w1));
        let world = match element.element_type().unwrap().as_str() {
            "FIXING" => {
                model_catalogue_member(&element)
                    .unwrap()
                    .generated
                    .unwrap_or_else(|| panic!("{what} 没出实体"))
                    .world
            }
            _ => world_matrix(&element).unwrap(),
        };
        assert_frame(world, &case);
    }
}

/// MDS 斜撑 FT44 的 PLDATU：`POSL` 是 CTUB p-line 集里的圆周线 VV（315°）/ PP（225°），这两条的 `PX` / `PY`
/// 程序都空着。E3D 把线放在管外圆上，站位沿 PLAX 离 JLDATU 一个外半径（这根 CHS 88.9 是 44.45 mm）；修前当
/// 常量 0 放在圆心。名下的 JOINT1 FIXING 没有 SPRE、不出实体，标架跟着 PLDATU 走。
#[test]
fn circumference_pline_datums_sit_on_the_tube_wall() {
    if !fixtures_present() {
        return;
    }
    let set = ams_set();
    let vv = (
        [-0.707106781, 0.707106781, 0.0],
        [0.0, 0.0, -1.0],
        [-0.707106781, -0.707106781, 0.0],
    );
    let pp = (
        [-0.707106781, -0.707106781, 0.0],
        [0.0, 0.0, -1.0],
        [0.707106781, -0.707106781, 0.0],
    );
    // (w1, 名词, E3D 平移, 三列, 说明)
    let cases = [
        (
            45947,
            "PLDATU",
            [-206.4309, -2706.4309, -7.88],
            vv,
            "FT44/6-V1/S2/PLDAT（POSL VV）",
        ),
        (
            45948,
            "FIXING",
            [-237.8618, -2675.0, 172.49],
            vv,
            "FT44/6-V1/S2/JOINT1",
        ),
        (
            45957,
            "PLDATU",
            [-143.5691, -2356.4309, -9.27],
            pp,
            "FT44/6-V2/S2/PLDAT（POSL PP）",
        ),
        (
            45958,
            "FIXING",
            [-112.1382, -2325.0, -189.14],
            pp,
            "FT44/6-V2/S2/JOINT1",
        ),
    ];
    for (w1, noun, translation, (x, y, z), what) in cases {
        let element = set.element(RefNo::new(DB_REFNO, w1));
        assert_eq!(element.element_type().unwrap(), noun, "{what}");
        assert_frame(
            world_matrix(&element).unwrap(),
            &(w1, translation, x, y, z, what),
        );
    }
}

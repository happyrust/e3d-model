//! 真库门：APS 详图式扶手（修复计划 W7a），立柱 / 端头的标架与目录件、栏杆的世界包围盒对 E3D TTY 导出。
//!
//! 真值是 gen-model `verification/civil-tty-20261021/` 下 aps250112（`aps_survey_lv6_2013286656_2.rvm`）与
//! aps250110（`aps250110_lv6_2013286654_2.rvm`）的 E3D 3.1 只读导出：模板件的每个图元与我们同名图元的
//! 刚体差（`output/rvm_frame_delta.py`）归零，目录件与栏杆的世界包围盒（`output/rvm_prim_dump.py`）逐分量对上。
//!
//! 缺 E3D 安装或影子资产默认红（与 `increment_real.rs` 同一口径）；无语料环境显式设
//! `E3D_SKIP_REAL_GATES=1` 换回跳过。

use std::path::{Path, PathBuf};
use std::sync::Arc;

use e3d_io::db_element::{DbFilePin, DbSet, template_file_for};
use e3d_io::refno::RefNo;
use e3d_model::category::{classify, is_model_unit_of};
use e3d_model::db_discovery::DirectoryDbResolver;
use e3d_model::elmodl::{Modeled, model_element};
use e3d_model::transform::world_matrix;

const PLANT_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaPlantSample\aps000";
const ACP_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaCatalogue\acp000";
const TEMPLATE_DIR: &str = r"E:\reverse\e3d\shadow_e3d31_aps_all";

/// 一只设计库：主文件与 refno 首字。
struct Design {
    file: &'static str,
    word0: u32,
}

const APS250110: Design = Design {
    file: "aps250110_0001",
    word0: 2013286654,
};
const APS250112: Design = Design {
    file: "aps250112_0001",
    word0: 2013286656,
};

fn fixtures_present(design: &Design) -> bool {
    let file = Path::new(PLANT_DIR).join(design.file);
    let attlib = Path::new(TEMPLATE_DIR).join("attlib.dat");
    let missing: Vec<String> = [file.as_path(), Path::new(ACP_DIR), attlib.as_path()]
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

fn aps_set(design: &Design) -> Arc<DbSet> {
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
        file: Path::new(PLANT_DIR).join(design.file),
        template: template_file_for(template, "DESI").unwrap(),
        db_type: Some("DESI".into()),
        sesno: None,
    })
    .unwrap();
    set
}

/// 一件的世界标架：原点与局部 +X 对 E3D。
fn assert_frame(set: &Arc<DbSet>, w1: u32, origin: [f64; 3], x_axis: [f64; 3], what: &str) {
    let world = world_matrix(&set.element(RefNo::new(APS250112.word0, w1))).unwrap();
    let got = world.w_axis.truncate().to_array();
    let x = world.x_axis.truncate().to_array();
    for axis in 0..3 {
        assert!(
            (got[axis] - origin[axis]).abs() <= 0.06,
            "{what}：原点 {got:?}，E3D {origin:?}"
        );
        assert!(
            (x[axis] - x_axis[axis]).abs() <= 1e-4,
            "{what}：局部 +X {x:?}，E3D {x_axis:?}"
        );
    }
}

/// 按 noun 建一件，世界包围盒逐分量对 E3D。
fn assert_world_box(
    set: &Arc<DbSet>,
    design: &Design,
    noun: &str,
    w1: u32,
    e3d: [f64; 6],
    what: &str,
) {
    let el = set.element(RefNo::new(design.word0, w1));
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

/// `/EL(+)100000` 两只扶手的模板件：立柱挂在 `FRAD 100` 的直角拐点上，`ZDIS` 从圆角切点量（比从拐点量远 100）；
/// 端头与立柱同一条朝向规则（`LEFT` 顺路径、`RIGH` 逆路径，开口末点的路径方向取进入段）——此前按目录号
/// 36004 特判，两只转反 180°。
#[test]
fn template_members_stand_past_the_fillet_and_turn_by_their_justification() {
    if !fixtures_present(&APS250112) {
        return;
    }
    let set = aps_set(&APS250112);
    assert_frame(
        &set,
        1093,
        [-306327.27, 298000.00, 100000.00],
        [0.0, -1.0, 0.0],
        "HRPOST 1 of /EL(+)100000/HR_002（LEFT，ZDIS 300）",
    );
    assert_frame(
        &set,
        1300,
        [-306327.27, 295000.00, 100000.00],
        [0.0, -1.0, 0.0],
        "HRPOST 4 of /EL(+)100000/HR_002（ZDIS 3300）",
    );
    assert_frame(
        &set,
        791,
        [-308109.33, 293804.60, 100000.00],
        [1.0, 0.0, 0.0],
        "HRPOST 1 of /EL(+)100000/HR_001（RIGH，ZDIS 290.69）",
    );
    assert_frame(
        &set,
        863,
        [-308103.60, 294204.60, 100000.00],
        [1.0, 0.0, 0.0],
        "HRTERM 1 of /EL(+)100000/HR_001（末点，LEFT）",
    );
    assert_frame(
        &set,
        994,
        [-307718.63, 294373.71, 100000.00],
        [0.0, 1.0, 0.0],
        "HRTERM 2 of /EL(+)100000/HR_001（起点，RIGH）",
    );
    assert_frame(
        &set,
        1372,
        [-306577.27, 294649.56, 100000.00],
        [-1.0, 0.0, 0.0],
        "HRTERM 1 of /EL(+)100000/HR_002（末点，LEFT）",
    );
    assert_frame(
        &set,
        1463,
        [-306723.67, 298400.00, 100000.00],
        [-1.0, 0.0, 0.0],
        "HRTERM 2 of /EL(+)100000/HR_002（起点，RIGH）",
    );
}

/// 目录构件式的立柱与端头（`SPRE → GPART --CATR--> SFIT`）：自己成件，标架与模板件同一条规则——
/// `Return` 回环起点 `LEFT`、末点 `RIGH`，两只都朝扶手里；`Pads` 两端都是 `RIGH`。`XOFF` 偏在路径右侧
/// （`XOFF 260` 的立柱、`XOFF 216` / `−40` 的端头）。模板件（GPART 没有 CATR）不自己成件。
#[test]
fn catalogue_posts_and_terminals_build_from_their_sfit_on_the_same_rule() {
    if !fixtures_present(&APS250112) {
        return;
    }
    let set = aps_set(&APS250112);
    let word0 = APS250112.word0;
    assert!(
        is_model_unit_of(&set.element(RefNo::new(word0, 665))).unwrap(),
        "目录构件式立柱自己成件"
    );
    assert!(
        !is_model_unit_of(&set.element(RefNo::new(word0, 1093))).unwrap(),
        "模板式立柱的几何在 TMPL 成员上"
    );
    assert_world_box(
        &set,
        &APS250112,
        "HRPOST",
        665,
        [
            -310743.41, 297763.20, 104900.00, -310643.41, 297913.20, 106031.50,
        ],
        "HRPOST 1 of /EL(+)104880_TOS/HR_001",
    );
    assert_frame(
        &set,
        665,
        [-310743.39, 297838.21, 104900.00],
        [0.0, -1.0, 0.0],
        "HRPOST 1 of /EL(+)104880_TOS/HR_001（LEFT，XOFF 260）",
    );
    assert_frame(
        &set,
        671,
        [-310699.39, 298288.21, 104900.00],
        [0.0, -1.0, 0.0],
        "HRTERM 1 of /EL(+)104880_TOS/HR_001（Return 起点，LEFT，XOFF 216）",
    );
    assert_frame(
        &set,
        672,
        [-310699.39, 295641.81, 104900.00],
        [0.0, 1.0, 0.0],
        "HRTERM 2 of /EL(+)104880_TOS/HR_001（Return 末点，RIGH，XOFF 216）",
    );
    assert_frame(
        &set,
        596,
        [-314088.39, 301156.61, 104900.00],
        [1.0, 0.0, 0.0],
        "HRTERM 1 of /EL(+)104880_TOS_PLATE/HR_001（Pads 末点，RIGH，XOFF −40）",
    );
    assert_frame(
        &set,
        597,
        [-310651.59, 301156.61, 104900.00],
        [1.0, 0.0, 0.0],
        "HRTERM 2 of /EL(+)104880_TOS_PLATE/HR_001（Pads 起点，RIGH）",
    );
}

/// 栏杆：`STAREF == ENDREF` 带 `SZDI` / `EZDI` 的是参照点出发那条边上的一截（aps250112，此前判成「取不出一段」）；
/// 闭合路径上夹着 `CURTYP THRU` 圆弧的那根绕过弧到门洞（aps250110，此前整根不画）。
#[test]
fn rails_take_their_szdi_cut_and_sweep_through_arcs() {
    if !fixtures_present(&APS250112) || !fixtures_present(&APS250110) {
        return;
    }
    assert_world_box(
        &aps_set(&APS250112),
        &APS250112,
        "RAIL",
        669,
        [
            -310716.25, 296091.80, 105983.15, -310682.55, 297838.20, 106016.85,
        ],
        "RAIL 1 of /EL(+)104880_TOS/HR_001（SZDI 450 → EZDI 2196.4，XOFF 216）",
    );
    assert_world_box(
        &aps_set(&APS250110),
        &APS250110,
        "RAIL",
        3387,
        [
            -312703.74, 316430.35, 109200.85, -298296.24, 326709.39, 109249.15,
        ],
        "RAIL 1 of /BUILDING_01_LEVEL_03/HR_001（闭合路径，首段 THRU 弧）",
    );
}

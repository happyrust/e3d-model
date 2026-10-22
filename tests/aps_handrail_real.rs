//! 真库门：APS 详图式扶手（修复计划 W7a），立柱 / 端头的标架对 E3D TTY 导出。
//!
//! 真值是 gen-model `verification/civil-tty-20261021/aps_survey_lv6_2013286656_2.rvm`（aps250112，E3D 3.1 只读
//! 导出）：模板件的每个图元与我们同名图元的刚体差（`output/rvm_frame_delta.py`）归零。
//!
//! 缺 E3D 安装或影子资产默认红（与 `increment_real.rs` 同一口径）；无语料环境显式设
//! `E3D_SKIP_REAL_GATES=1` 换回跳过。

use std::path::{Path, PathBuf};
use std::sync::Arc;

use e3d_io::db_element::{DbFilePin, DbSet, template_file_for};
use e3d_io::refno::RefNo;
use e3d_model::db_discovery::DirectoryDbResolver;
use e3d_model::transform::world_matrix;

const PLANT_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaPlantSample\aps000";
const ACP_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaCatalogue\acp000";
const TEMPLATE_DIR: &str = r"E:\reverse\e3d\shadow_e3d31_aps_all";

/// 一只设计库：主文件与 refno 首字。
struct Design {
    file: &'static str,
    word0: u32,
}

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

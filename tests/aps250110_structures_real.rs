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
use e3d_model::elmodl::{GeneratedElement, Modeled, model_element};
use e3d_model::stair::tread_size;

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

fn generated(set: &Arc<DbSet>, noun: &str, w1: u32, what: &str) -> Box<GeneratedElement> {
    let el = set.element(RefNo::new(2013286654, w1));
    match model_element(&el, classify(noun)).unwrap() {
        Modeled::Solid(generated) => generated,
        Modeled::Skipped { reason, .. } => panic!("{what} 没出实体：{reason}"),
    }
}

/// 世界包围盒逐分量对 E3D。
fn assert_box(generated: &GeneratedElement, e3d: [f64; 6], what: &str) {
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

/// 按 noun 建一件，世界包围盒逐分量对 E3D。
fn assert_world_box(set: &Arc<DbSet>, noun: &str, cases: &[(u32, [f64; 6], &str)]) {
    for (w1, e3d, what) in cases {
        assert_box(&generated(set, noun, *w1, what), *e3d, what);
    }
}

/// 折弯板：没有开孔的一段一块（N 个点 → 2N − 3 块，与 E3D 的 FacetGroup 条数相同），开孔的整只扣完
/// 一块。屋面板 9 的最低点在只剩一条弦的 R25 弯上（开孔板的弯走布尔操作数的粗分段）。
#[test]
fn bent_panels_follow_their_rpath_and_the_e3d_record_split() {
    if !fixtures_present() {
        return;
    }
    let set = aps_set();
    let cases: [(u32, [f64; 6], Option<usize>, &str); 6] = [
        (
            17197,
            [
                -321183.30, 334709.00, 100010.07, -321133.30, 337673.00, 102440.07,
            ],
            None,
            "BPANEL 1 of SBFRAMEWORK /AREA02_WS_CLADDING\\WEST（3 只 NCYL 开孔）",
        ),
        (
            17301,
            [
                -321183.32, 337511.99, 100010.07, -321133.30, 340490.99, 102440.07,
            ],
            Some(93),
            "BPANEL 3 of SBFRAMEWORK /AREA02_WS_CLADDING\\WEST",
        ),
        (
            18191,
            [
                -305136.32, 340871.99, 100010.07, -305086.31, 343209.01, 102440.07,
            ],
            Some(73),
            "BPANEL 2 of SBFRAMEWORK /AREA02_WS_CLADDING\\EAST",
        ),
        (
            18961,
            [
                -305254.00, 343168.02, 100010.07, -305120.99, 343292.99, 104035.07,
            ],
            Some(3),
            "BPANEL 1 of SBFRAMEWORK /AREA02_WS_CLADDING\\TRIM（板厚 10）",
        ),
        (
            19295,
            [
                -310730.91, 339389.77, 104499.66, -304993.15, 341633.36, 105103.80,
            ],
            Some(291),
            "BPANEL 3 of FRMWORK /AREA02_WS_ROOFING",
        ),
        (
            20149,
            [
                -316394.89, 341242.06, 104017.35, -310699.89, 343478.38, 104630.24,
            ],
            None,
            "BPANEL 9 of FRMWORK /AREA02_WS_ROOFING（NBOX 开孔）",
        ),
    ];
    for (w1, e3d, pieces, what) in cases {
        let panel = generated(&set, "BPANEL", w1, what);
        assert_eq!(panel.parts.as_ref().map(Vec::len), pieces, "{what}");
        assert_box(&panel, e3d, what);
    }
}

/// `STRFLT` 两段梯的踏步：目录 SBOX 的宽 / 深是随梯段现算的 `TWIDTH` / `TREADD`，厚是目录
/// `PARA[3]` = 20。E3D 的盒深 269.654 / 297.924，我们按梯段 RPATH 算出的差 ±0.005。
#[test]
fn stair_treads_are_the_catalogue_box_sized_by_the_flight() {
    if !fixtures_present() {
        return;
    }
    let set = aps_set();
    for (w1, depth) in [(8586, 269.654), (8618, 297.924)] {
        let size = tread_size(&set.element(RefNo::new(2013286654, w1)))
            .unwrap()
            .expect("踏步在梯段里");
        assert_eq!(size.width, 800.0, "{w1}");
        assert!((size.depth - depth).abs() < 0.01, "{w1}: {}", size.depth);
    }
    assert_world_box(
        &set,
        "TREAD",
        &[
            (
                8586,
                [
                    -302326.20, 319658.99, 108164.36, -302056.55, 320458.99, 108184.36,
                ],
                "TREAD 1 of TREADSET 1 of STRFLT /B01_LEVEL_03_ACCESS/SF_001",
            ),
            (
                8618,
                [
                    -299729.62, 320645.96, 110008.00, -298929.62, 320943.88, 110028.00,
                ],
                "TREAD 1 of TREADSET 1 of STRFLT /B01_LEVEL_03_ACCESS/SF_002",
            ),
        ],
    );
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

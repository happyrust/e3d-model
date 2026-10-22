//! 真库门：`gen_ams` 写 RVM 时，扣过负体的件在 `export holes on`（TTY 取证的口径，缺省）下是一条
//! 布尔结果面片，`E3D_EXPORT_HOLES=off`（C-IY-1R330-B 那份 addin 导出的口径）下桥架那一类
//! 可见图元仍各写一条未扣减的。
//!
//! MDS 关节 `/SUPPORT/FRAMES/BS/FT03H/9-V1/S1/IJOINT1`（ams7324 `23708/42148`）的目录件是 JOIN
//! `/HCS-VT41-68/2`：GMSE 两只 SBOX，每只名下一只 NSCY（图元级负体，就是那个孔）。E3D TTY
//! 导出（gen-model `verification/civil-tty-20261021/mds_bs_lv6_23708_42032.rvm`）写的是一条 24 个
//! 多边形、28 条轮廓的带孔 FacetGroup；此前我们按桥架的口径写成两条未扣减的 Box，孔在 RVM
//! 里丢了。
//!
//! ams7997 `/SSC测试` 整站对拍（gen-model `verification/civil-tty-20261021/ams7997_ssc_lv6.rvm`）里
//! 另外两类：截面就是一只 SREC 的 STWALL，E3D 走 DRPYR 写一条 `Pyramid`，高沿挤出方向；风管
//! `/-VES-E-4-L-2501` 的 BEND 1 / THRE 1 本体是一条面片，另带 2 / 4 块高为 0 的 LPYR 板，Core 不出体，
//! E3D 照写 `Pyramid[500, 500, 500, 500, 0, ±0, -0]`。
//!
//! 缺 E3D 安装或影子资产默认红（与 `increment_real.rs` 同一口径）；无语料环境显式设
//! `E3D_SKIP_REAL_GATES=1` 换回跳过。

use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

use e3d_model::rvm_read::{Primitive, RvmFile, RvmMember};

const AMS_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaMarineSample\ams000";
const ACP_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaCatalogue\acp000";
const MDS_DB: &str = "ams7324_0001";
const SSC_DB: &str = "ams7997_0001";
const CABLE_EQUIP_DB: &str = "ams8000_0001";
const APS_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaPlantSample\aps000";
const APS_STRUCTURAL_DB: &str = "aps250110_0001";
const APS_SUPPORTS_DB: &str = "aps250104_0001";
const APS_PIPING_DB: &str = "aps250124_0001";
const APS_HVAC_DB: &str = "aps250142_0001";
const APS_VALVE_DB: &str = "aps250132_0001";
/// 不在 APS 的 `/ALL` 里，E3D 只能在项目副本上导；设计库文件照样在 aps000。
const APS_COPY_ONLY_DB: &str = "aps7200_0001";
const TEMPLATE_DIR: &str = r"E:\reverse\e3d\shadow_e3d31_aps_all";
const IJOINT1: &str = "23708/42148";
const IJOINT1_NAME: &str = "/SUPPORT/FRAMES/BS/FT03H/9-V1/S1/IJOINT1";

fn fixtures_present(design_db: &str) -> bool {
    fixtures_present_in(AMS_DIR, design_db)
}

fn fixtures_present_in(project_dir: &str, design_db: &str) -> bool {
    let attlib = Path::new(TEMPLATE_DIR).join("attlib.dat");
    let design = Path::new(project_dir).join(design_db);
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

/// 只生成 `root` 这一只，返回写出的 RVM 与 `elements.json`。
fn generate(design_db: &str, root: &str, holes: &str, out: &Path) -> (RvmFile, serde_json::Value) {
    generate_in(AMS_DIR, design_db, root, holes, out)
}

/// [`generate`]，设计库在另一个项目目录里（APS）。
fn generate_in(
    project_dir: &str,
    design_db: &str,
    root: &str,
    holes: &str,
    out: &Path,
) -> (RvmFile, serde_json::Value) {
    let template = Path::new(TEMPLATE_DIR);
    let status = Command::new(env!("CARGO_BIN_EXE_gen_ams"))
        .arg("--attlib")
        .arg(template.join("attlib.dat"))
        .arg("--template")
        .arg(template)
        .arg("--db")
        .arg(Path::new(project_dir).join(design_db))
        .args(["--db-type", "DESI", "--root", root])
        .args(["--project-dir", project_dir, "--project-dir", ACP_DIR])
        .arg("--out")
        .arg(out)
        .arg("--rvm")
        .env("E3D_EXPORT_HOLES", holes)
        .stdout(Stdio::null())
        .status()
        .unwrap();
    assert!(
        status.success(),
        "gen_ams {root} (holes {holes}) 失败：{status}"
    );
    let rvm = RvmFile::read(out.join("generated.rvm")).unwrap();
    let elements =
        serde_json::from_slice(&std::fs::read(out.join("elements.json")).unwrap()).unwrap();
    (rvm, elements)
}

fn member<'a>(rvm: &'a RvmFile, name: &str) -> &'a RvmMember {
    rvm.members
        .iter()
        .find(|member| member.name.trim() == name)
        .unwrap_or_else(|| panic!("RVM 里没有 {name}"))
}

fn record_kinds(rvm: &RvmFile, name: &str) -> Vec<&'static str> {
    member(rvm, name)
        .geometries
        .iter()
        .map(|geometry| geometry.primitive.name())
        .collect()
}

fn snouts(rvm: &RvmFile, name: &str) -> Vec<[f64; 9]> {
    let mut out: Vec<[f64; 9]> = member(rvm, name)
        .geometries
        .iter()
        .filter_map(|geometry| match geometry.primitive {
            Primitive::Snout(parameters) => Some(parameters),
            _ => None,
        })
        .collect();
    out.sort_by(|a, b| a.partial_cmp(b).unwrap());
    out
}

fn pyramids(rvm: &RvmFile, name: &str) -> Vec<[f64; 7]> {
    member(rvm, name)
        .geometries
        .iter()
        .filter_map(|geometry| match geometry.primitive {
            Primitive::Pyramid(parameters) => Some(parameters),
            _ => None,
        })
        .collect()
}

#[test]
fn a_joint_with_primitive_holes_is_one_holed_facet_group() {
    if !fixtures_present(MDS_DB) {
        return;
    }
    let scratch: PathBuf =
        std::env::temp_dir().join(format!("e3d-model-rvm-kinds-{}", std::process::id()));

    let (rvm, elements) = generate(MDS_DB, IJOINT1, "on", &scratch.join("holes-on"));
    assert_eq!(record_kinds(&rvm, IJOINT1_NAME), ["FacetGroup"]);
    let element = &elements[0];
    assert_eq!(element["negatives_applied"], 2, "{element}");
    // 两块板各一个通孔：写进 RVM 的就是这只亏格 2 的网格。
    assert_eq!(element["genus"], 2, "{element}");

    let (rvm, _) = generate(MDS_DB, IJOINT1, "off", &scratch.join("holes-off"));
    assert_eq!(record_kinds(&rvm, IJOINT1_NAME), ["Box", "Box"]);

    std::fs::remove_dir_all(&scratch).ok();
}

/// 预期值是 E3D TTY 导出里的记录原样（种类顺序、`Pyramid` 七个参数）。
#[test]
fn single_srec_walls_and_flat_duct_plates_write_e3d_pyramids() {
    if !fixtures_present(SSC_DB) {
        return;
    }
    let scratch: PathBuf =
        std::env::temp_dir().join(format!("e3d-model-rvm-kinds-ssc-{}", std::process::id()));
    let walls = [
        (
            "24381/15586",
            "STWALL 1 of CWALL /1SL-WF08-W-C-S8038",
            [150.0, 700.0, 150.0, 700.0, 0.0, 0.0, 1835.0],
        ),
        (
            "24381/15726",
            "STWALL 1 of CWALL /1SL-WF08-W-C-S8019",
            [200.0, 200.0, 200.0, 200.0, 0.0, 0.0, 1200.0],
        ),
    ];
    for (root, name, want) in walls {
        let (rvm, _) = generate(SSC_DB, root, "on", &scratch.join(root.replace('/', "_")));
        assert_eq!(record_kinds(&rvm, name), ["Pyramid"], "{name}");
        let got = pyramids(&rvm, name)[0];
        assert!(
            got.iter().zip(want).all(|(a, b)| (a - b).abs() < 1e-3),
            "{name}: {got:?}"
        );
    }

    let fittings = [
        ("24381/30463", "BEND 1 of BRAN /-VES-E-4-L-2501", 2),
        ("24381/30501", "THRE 1 of BRAN /-VES-E-4-L-2501", 4),
    ];
    for (root, name, plates) in fittings {
        let (rvm, _) = generate(SSC_DB, root, "on", &scratch.join(root.replace('/', "_")));
        let mut want = vec!["FacetGroup"];
        want.extend(std::iter::repeat_n("Pyramid", plates));
        assert_eq!(record_kinds(&rvm, name), want, "{name}");
        for plate in pyramids(&rvm, name) {
            assert_eq!(
                plate[..5],
                [500.0, 500.0, 500.0, 500.0, 0.0],
                "{name}: {plate:?}"
            );
            assert!(plate[5] == 0.0, "{name}: {plate:?}");
            assert!(
                plate[6] == 0.0 && plate[6].is_sign_negative(),
                "{name}: {plate:?}"
            );
        }
    }

    std::fs::remove_dir_all(&scratch).ok();
}

/// happyrust/e3d-model#6 的暖通区：5 节分段弯头 `BEND 1 of BRANCH /-CAM-E-2-H-5202`（`24381/47002`，
/// 目录 `/C5SEGB`）的 SSLC 斜切节，E3D 写带斜切角的 `Snout`（11.25°）；软接头 `/1CAM501FL`
/// （`24381/47029`，目录 `/CFLEX`）的 27 只零径延长段，E3D 写 `Snout[0, 0, 2, …]`。预期值是 TTY
/// 导出（plant-model-6 `outputs/t43-kx-cam-e01/kx-cam-e01.rvm`）里的记录原样。
#[test]
fn hvac_mitred_bends_and_flex_extensions_write_e3d_snouts() {
    if !fixtures_present(SSC_DB) {
        return;
    }
    let scratch: PathBuf =
        std::env::temp_dir().join(format!("e3d-model-rvm-kinds-hvac-{}", std::process::id()));
    let shear = 11.25f64.to_radians();

    let bend = "BEND 1 of BRAN /-CAM-E-2-H-5202";
    let (rvm, _) = generate(SSC_DB, "24381/47002", "on", &scratch.join("bend"));
    let kinds = record_kinds(&rvm, bend);
    assert_eq!(
        kinds.iter().filter(|kind| **kind == "Snout").count(),
        9,
        "{kinds:?}"
    );
    assert_eq!(
        kinds.iter().filter(|kind| **kind == "Cylinder").count(),
        4,
        "{kinds:?}"
    );
    assert!(!kinds.contains(&"FacetGroup"), "{kinds:?}");
    let sheared: Vec<[f64; 9]> = snouts(&rvm, bend)
        .into_iter()
        .filter(|snout| snout[5..].iter().any(|angle| *angle != 0.0))
        .collect();
    let mut want = vec![
        [175.0, 175.0, 104.429, 0.0, 0.0, 0.0, 0.0, -shear, 0.0],
        [175.0, 175.0, 104.429, 0.0, 0.0, 0.0, 0.0, 0.0, -shear],
        [175.0, 175.0, 208.858, 0.0, 0.0, 0.0, shear, 0.0, -shear],
        [175.0, 175.0, 208.858, 0.0, 0.0, 0.0, shear, 0.0, -shear],
        [175.0, 175.0, 208.858, 0.0, 0.0, 0.0, shear, 0.0, -shear],
    ];
    want.sort_by(|a, b| a.partial_cmp(b).unwrap());
    assert_eq!(sheared.len(), want.len(), "{sheared:?}");
    for (got, want) in sheared.iter().zip(&want) {
        assert!(
            got.iter().zip(want).all(|(a, b)| (a - b).abs() < 1e-2),
            "{got:?} vs {want:?}"
        );
    }

    let flex = "/1CAM501FL";
    let (rvm, _) = generate(SSC_DB, "24381/47029", "on", &scratch.join("flex"));
    let zero: Vec<[f64; 9]> = snouts(&rvm, flex)
        .into_iter()
        .filter(|snout| snout[0] == 0.0 && snout[1] == 0.0)
        .collect();
    assert_eq!(zero.len(), 27, "{zero:?}");
    assert!(
        zero.iter()
            .all(|snout| (snout[2] - 2.0).abs() < 1e-9 && snout[3..].iter().all(|v| *v == 0.0)),
        "{zero:?}"
    );

    std::fs::remove_dir_all(&scratch).ok();
}

/// 类 28 的 `ELCONN`（电气连接件）只有 `CATR`（→ TABITE → PRTREF → GPART → SCOM），没有 `SPRE`。
/// ams8000 设备 `/1-CAM-043-BC`（`24384/24873`）名下两只：E3D TTY 导出（gen-model
/// `verification/civil-tty-20261021/ams8000_lv6_24384_24828.rvm`）各一组面片，接线盒那只 7 条（5 块 SEXT，
/// 整圈 SREV E3D 拆成两半写，我们写一条），底座那只 1 条。此前我们一只不出。世界包围盒取 E3D 面片的。
#[test]
fn electrical_connections_enter_their_catalogue_by_catr() {
    if !fixtures_present(CABLE_EQUIP_DB) {
        return;
    }
    let scratch: PathBuf =
        std::env::temp_dir().join(format!("e3d-model-rvm-kinds-elconn-{}", std::process::id()));
    let (rvm, elements) = generate(CABLE_EQUIP_DB, "24384/24873", "on", &scratch);
    let cases = [
        (
            "ELCONN 1 of EQUI /1-CAM-043-BC",
            "24384/24874",
            [-1860.572, 20562.219, 1125.0],
            [-1646.403, 20821.404, 1275.0],
        ),
        (
            "ELCONN 2 of EQUI /1-CAM-043-BC",
            "24384/24875",
            [-1793.836, 20492.162, 80.0],
            [-1382.164, 20903.838, 1300.0],
        ),
    ];
    for (name, refno, lo, hi) in cases {
        let kinds = record_kinds(&rvm, name);
        assert!(
            !kinds.is_empty() && kinds.iter().all(|kind| *kind == "FacetGroup"),
            "{name}: {kinds:?}"
        );
        let element = elements
            .as_array()
            .unwrap()
            .iter()
            .find(|element| element["refno"] == refno)
            .unwrap_or_else(|| panic!("elements.json 里没有 {refno}"));
        for axis in 0..3 {
            let got = (
                element["aabb_min"][axis].as_f64().unwrap(),
                element["aabb_max"][axis].as_f64().unwrap(),
            );
            assert!(
                (got.0 - lo[axis]).abs() <= 0.01 && (got.1 - hi[axis]).abs() <= 0.01,
                "{name}：包围盒第 {axis} 分量 {got:?}，E3D ({}, {})",
                lo[axis],
                hi[axis]
            );
        }
    }

    std::fs::remove_dir_all(&scratch).ok();
}

/// 目录 `SEXT` 不论截面长什么样，E3D 都写 `FacetGroup`：ams8000 竖梯 `FTUB 1 of BRAN /C-GR-1RX-3AB0A`
/// （`24384/3200`，目录 `/ACP1000-RFVL-R3B-500`）的 11 根槽钢与 22 块圆板全是 SEXT，E3D TTY 导出
/// （`ams8000_lv6_24384_2498.rvm`）写 33 条面片；此前我们把圆板认成了 22 条 `Cylinder`。
#[test]
fn catalogue_extrusions_stay_facet_groups_even_when_round() {
    if !fixtures_present(CABLE_EQUIP_DB) {
        return;
    }
    let scratch: PathBuf =
        std::env::temp_dir().join(format!("e3d-model-rvm-kinds-riser-{}", std::process::id()));
    let (rvm, _) = generate(CABLE_EQUIP_DB, "24384/3199", "on", &scratch);
    let kinds = record_kinds(&rvm, "FTUB 1 of BRAN /C-GR-1RX-3AB0A");
    assert_eq!(kinds, vec!["FacetGroup"; 33]);

    std::fs::remove_dir_all(&scratch).ok();
}

/// aps250110 门框模板（`FITT 1 of STWALL 1 of CWALL /B01_LEV02_INT_WALLS`）里 45° 斜接的 FBAR：截面就是
/// 一只 SREC，但端头斜切就不是棱台，E3D TTY 导出（gen-model `verification/civil-tty-20261021/`
/// `aps250110_lv6_2013286654_2.rvm`）写一条面片；平头的同类 STWALL 仍写 `Pyramid`（上面那条）。
/// 扶手立柱模板里 `RADI = 25` 的设计 DISH 是折边碟，E3D 写 `EllipticalDish`，按长相会认成球形碟。
#[test]
fn mitred_flat_bars_and_knuckled_dishes_follow_e3d_writers() {
    if !fixtures_present_in(APS_DIR, APS_STRUCTURAL_DB) {
        return;
    }
    let scratch: PathBuf =
        std::env::temp_dir().join(format!("e3d-model-rvm-kinds-aps-{}", std::process::id()));
    let (rvm, _) = generate_in(
        APS_DIR,
        APS_STRUCTURAL_DB,
        "2013286654/444",
        "on",
        &scratch.join("fbar"),
    );
    assert_eq!(
        record_kinds(
            &rvm,
            "SCTN 2 of TMPL 1 of FITT 1 of STWALL 1 of CWALL /B01_LEV02_INT_WALLS"
        ),
        ["FacetGroup"]
    );
    let (rvm, _) = generate_in(
        APS_DIR,
        APS_STRUCTURAL_DB,
        "2013286654/3545",
        "on",
        &scratch.join("dish"),
    );
    assert_eq!(
        record_kinds(
            &rvm,
            "DISH 1 of TMPL 1 of HRPOST 1 of POSTSE 1 of HANDRA /B01_LEVEL_01_HANDRAILS/HR_001"
        ),
        ["EllipticalDish"]
    );

    std::fs::remove_dir_all(&scratch).ok();
}

/// aps250104 支架库（E3D TTY 导出 gen-model `verification/civil-tty-20261021/aps_survey_lv6_2013286648_2.rvm`）：
/// - 锚固件 `/PS-01-00122.1`（ANCI，目录 `/BT23-100`）扣过三把负体，E3D 只写一条布尔结果面片；目录里
///   `PDIA = DESP[11] = 0` 的那只 LCYL 没有体，不在布尔结果里，也不另写一条 `Cylinder`。
/// - 它的属主 `SUPC /PS-01-00122/B1` 名下 E3D 不补管身；`TRUNNI /PS-01-00111/B1` 照补一根，
///   E3D 写在耳轴自己那一组里：`Cylinder[44.45, 524.56]`。
#[test]
fn holed_anchors_drop_bodyless_cylinders_and_only_trunnions_grow_tubes() {
    if !fixtures_present_in(APS_DIR, APS_SUPPORTS_DB) {
        return;
    }
    let scratch: PathBuf =
        std::env::temp_dir().join(format!("e3d-model-rvm-supports-aps-{}", std::process::id()));
    let (rvm, elements) = generate_in(
        APS_DIR,
        APS_SUPPORTS_DB,
        "2013286648/559",
        "on",
        &scratch.join("supc"),
    );
    assert_eq!(record_kinds(&rvm, "/PS-01-00122.1"), ["FacetGroup"]);
    let anchor = elements
        .as_array()
        .unwrap()
        .iter()
        .find(|element| element["refno"] == "2013286648/560")
        .expect("锚固件 2013286648/560");
    assert_eq!(anchor["negatives_applied"], 3, "{anchor}");
    assert!(
        rvm.members
            .iter()
            .all(|member| !member.name.trim().starts_with("TUBI")),
        "SUPC 名下不补管身"
    );

    let (rvm, _) = generate_in(
        APS_DIR,
        APS_SUPPORTS_DB,
        "2013286648/489",
        "on",
        &scratch.join("trunni"),
    );
    let tube = member(&rvm, "TUBI 1 of TRUNNI /PS-01-00111/B1");
    let [geometry] = tube.geometries.as_slice() else {
        panic!("耳轴管身应是一条记录，实际 {} 条", tube.geometries.len());
    };
    let Primitive::Cylinder([radius, height]) = geometry.primitive else {
        panic!("耳轴管身应是 Cylinder，实际 {}", geometry.primitive.name());
    };
    assert!(
        (radius - 44.45).abs() < 0.01 && (height - 524.56).abs() < 0.01,
        "Cylinder[{radius}, {height}]"
    );

    std::fs::remove_dir_all(&scratch).ok();
}

/// aps250124 管道库 `/100-LAYOUT300-307/B1`（E3D TTY 导出 gen-model
/// `verification/civil-tty-20261021/aps_survey_lv6_2013286668_54.rvm`）有一段按 1:75 放坡：两只弯头转了
/// 坡度角，中间那只 TEE（`2013286668/683`）没转，靠 `SLODIR` 把 P1 / P2 各压低 0.7639°。按目录方向判，
/// TEE 两侧的槽位各偏 0.764°、不建；E3D 两根都画了（`Cylinder[57.15, 1042.81]`、`[57.15, 711.16]`），
/// 这条支管一共 10 根管身。
#[test]
fn sloped_tee_points_take_their_slodir_directions() {
    if !fixtures_present_in(APS_DIR, APS_PIPING_DB) {
        return;
    }
    let scratch: PathBuf =
        std::env::temp_dir().join(format!("e3d-model-rvm-slope-aps-{}", std::process::id()));
    let (rvm, _) = generate_in(APS_DIR, APS_PIPING_DB, "2013286668/141", "on", &scratch);
    let mut lengths: Vec<f64> = rvm
        .members
        .iter()
        .filter(|member| {
            let name = member.name.trim();
            name.starts_with("TUBI ") && name.ends_with(" of BRAN /100-LAYOUT300-307/B1")
        })
        .flat_map(|member| &member.geometries)
        .filter_map(|geometry| match geometry.primitive {
            Primitive::Cylinder([_, height]) => Some(height),
            _ => None,
        })
        .collect();
    lengths.sort_by(f64::total_cmp);
    assert_eq!(lengths.len(), 10, "{lengths:?}");
    for want in [711.16, 1042.81] {
        assert!(
            lengths.iter().any(|length| (length - want).abs() < 0.01),
            "缺 {want} 那根：{lengths:?}"
        );
    }

    std::fs::remove_dir_all(&scratch).ok();
}

/// aps250142 风管 `/SUPPLY_WORKSHOP-001`（E3D TTY 导出 gen-model
/// `verification/civil-tty-20261021/aps_survey_lv6_2013286686_2.rvm`）只有一根管身：BRCO `2013286686/56` 到
/// 下一件的 144.7 mm 矩形管。分流片 SPLR `2013286686/18` 伸进下一件 25 mm，E3D 不拿它当端点，那里没有管。
#[test]
fn duct_splitters_do_not_end_a_duct() {
    if !fixtures_present_in(APS_DIR, APS_HVAC_DB) {
        return;
    }
    let scratch: PathBuf =
        std::env::temp_dir().join(format!("e3d-model-rvm-splitter-aps-{}", std::process::id()));
    let (rvm, _) = generate_in(APS_DIR, APS_HVAC_DB, "2013286686/4", "on", &scratch);
    let tubes: Vec<&RvmMember> = rvm
        .members
        .iter()
        .filter(|member| {
            let name = member.name.trim();
            name.starts_with("TUBI ") && name.ends_with(" of BRAN /SUPPLY_WORKSHOP-001")
        })
        .collect();
    let [tube] = tubes.as_slice() else {
        panic!("这条风管只有一根管身，实际 {} 根", tubes.len());
    };
    let [geometry] = tube.geometries.as_slice() else {
        panic!("管身应是一条记录，实际 {} 条", tube.geometries.len());
    };
    let Primitive::Box(lengths) = geometry.primitive else {
        panic!("管身应是 Box，实际 {}", geometry.primitive.name());
    };
    assert!(
        lengths.iter().any(|length| (length - 144.7).abs() < 0.01),
        "Box{lengths:?}"
    );

    std::fs::remove_dir_all(&scratch).ok();
}

/// aps250132 阀门 `/V118`（`2013286676/1591`，E3D TTY 导出 gen-model
/// `verification/civil-tty-20261021/aps_survey_lv6_2013286676_490.rvm`）：目录 `LINE`（`PTS 0 3`、`TUFL` 真）
/// E3D 写一条 `Line[5, 292]`，原点在阀杆 P0→P3 的中点，Z 沿阀杆 `(0, -0.0872, 0.9962)`，X 是世界 −X。
#[test]
fn valve_stem_lines_write_e3d_line_records() {
    if !fixtures_present_in(APS_DIR, APS_VALVE_DB) {
        return;
    }
    let scratch: PathBuf =
        std::env::temp_dir().join(format!("e3d-model-rvm-line-aps-{}", std::process::id()));
    let (rvm, _) = generate_in(APS_DIR, APS_VALVE_DB, "2013286676/1591", "on", &scratch);
    let lines: Vec<_> = member(&rvm, "/V118")
        .geometries
        .iter()
        .filter(|geometry| matches!(geometry.primitive, Primitive::Line(_)))
        .collect();
    let [line] = lines.as_slice() else {
        panic!("/V118 应有一条 Line，实际 {} 条", lines.len());
    };
    let Primitive::Line([width, length]) = line.primitive else {
        unreachable!();
    };
    assert!(
        width == 5.0 && (length - 292.0).abs() < 0.01,
        "Line[{width}, {length}]"
    );
    let origin = line.translation_mm;
    for (got, want) in origin.iter().zip([-313092.99, 300822.81, 101490.45]) {
        assert!((got - want).abs() < 0.05, "原点 {origin:?}");
    }
    let axes = line.matrix3.map(|value| value * 1000.0);
    for (got, want) in axes[6..9].iter().zip([0.0, -0.0872, 0.9962]) {
        assert!((got - want).abs() < 1e-3, "Z {axes:?}");
    }
    for (got, want) in axes[0..3].iter().zip([-1.0, 0.0, 0.0]) {
        assert!((got - want).abs() < 1e-3, "X {axes:?}");
    }

    std::fs::remove_dir_all(&scratch).ok();
}

/// aps7200 矩形风管弯头 `ELBOW 1 of BRANCH /HVAC/RECT/M1/B2`（`23584/7314`，目录 acp7021 `/DBS`；E3D TTY
/// 在项目副本上导出 gen-model `verification/civil-tty-20261021/aps7200-w7h/aps7200_lv6_23584_7298.rvm`）：
/// `P2` / `P6` 的轴是 `Y DESP 14 -Z`、`DESP[14] = 90`，E3D 把本体写成
/// `RectangularTorus[100, 300, 400, 90°]`。此前那个角按 `PARAM 64` 读成 0，弯头是直的、本体整只没了。
#[test]
fn rectangular_duct_elbows_turn_by_their_design_angle() {
    if !fixtures_present_in(APS_DIR, APS_COPY_ONLY_DB) {
        return;
    }
    let scratch: PathBuf = std::env::temp_dir().join(format!(
        "e3d-model-rvm-rect-elbow-aps-{}",
        std::process::id()
    ));
    let (rvm, _) = generate_in(APS_DIR, APS_COPY_ONLY_DB, "23584/7314", "on", &scratch);
    let tori: Vec<[f64; 4]> = member(&rvm, "ELBO 1 of BRAN /HVAC/RECT/M1/B2")
        .geometries
        .iter()
        .filter_map(|geometry| match geometry.primitive {
            Primitive::RectangularTorus(parameters) => Some(parameters),
            _ => None,
        })
        .collect();
    let [[inner, outer, height, angle]] = tori.as_slice() else {
        panic!("弯头本体应是一条 RectangularTorus，实际 {tori:?}");
    };
    assert!(
        (inner - 100.0).abs() < 0.01
            && (outer - 300.0).abs() < 0.01
            && (height - 400.0).abs() < 0.01
            && (angle - std::f64::consts::FRAC_PI_2).abs() < 1e-4,
        "RectangularTorus[{inner}, {outer}, {height}, {angle}]"
    );

    std::fs::remove_dir_all(&scratch).ok();
}

/// `elements.json` 里一件的世界包围盒对一组 E3D 值，逐分量不超过 `tol` mm。
fn assert_world_aabb(elements: &serde_json::Value, refno: &str, want: [f64; 6], tol: f64) {
    let element = elements
        .as_array()
        .unwrap()
        .iter()
        .find(|element| element["refno"] == refno)
        .unwrap_or_else(|| panic!("elements.json 里没有 {refno}"));
    let got: Vec<f64> = (0..3)
        .map(|axis| element["aabb_min"][axis].as_f64().unwrap())
        .chain((0..3).map(|axis| element["aabb_max"][axis].as_f64().unwrap()))
        .collect();
    for (axis, (g, w)) in got.iter().zip(want).enumerate() {
        assert!(
            (g - w).abs() <= tol,
            "{refno}：包围盒第 {axis} 分量 {g:.3}，E3D {w:.3}（全部 {got:?}）"
        );
    }
}

/// 没有 POS / ORI 的两类件，标架都由属主现算：
/// - 扶手面板 `HRPANE 1 of HRPNSE 1 of HANDRA /B01_LEVEL_03_ACCESS/HR_001`：站位与立柱同一套（`POSREF` +
///   `ZDIS`），再加 `XOFF 69 / ZOFF -170`；模板里的 `PANE 2`（`2013286654/8658`）此前偏 3.3 m。
/// - 墙板上的 `PFIT 1 of PANE 1 of FRMW /B02_LEVEL_01_WALLS`（`2013286654/639`，`BANG = 90`）：目录实例绕板法向转
///   `BANG`，此前 1600 × 300 的板长短边对调。
/// 真值是 E3D TTY 导出里这两件面片的世界包围盒。
#[test]
fn handrail_panels_and_panel_fittings_sit_where_e3d_draws_them() {
    if !fixtures_present_in(APS_DIR, APS_STRUCTURAL_DB) {
        return;
    }
    let scratch: PathBuf =
        std::env::temp_dir().join(format!("e3d-model-rvm-frames-aps-{}", std::process::id()));
    let (_, elements) = generate_in(
        APS_DIR,
        APS_STRUCTURAL_DB,
        "2013286654/8650",
        "on",
        &scratch.join("hrpane"),
    );
    assert_world_aabb(
        &elements,
        "2013286654/8658",
        [
            -299564.62, 326238.75, 112530.0, -299384.66, 326253.75, 112630.0,
        ],
        0.1,
    );
    let (_, elements) = generate_in(
        APS_DIR,
        APS_STRUCTURAL_DB,
        "2013286654/639",
        "on",
        &scratch.join("pfit"),
    );
    assert_world_aabb(
        &elements,
        "2013286654/639",
        [
            -289685.0, 318239.75, 103960.0, -288065.0, 318439.75, 104280.01,
        ],
        0.1,
    );

    std::fs::remove_dir_all(&scratch).ok();
}

/// MDS 的结构关节 ams7324 `/MDS/FT09/9-V1/S1/IJOINT1`（`23708/25783`，目录 `/HCS-VT41-68/2`）：连接梁
/// `/MDS/FT09/9-BAR-1` 转了 180°（`BANG 180`），E3D 的关节 Z 与 `CUTP` 反向，两块角板落在梁那一侧。世界
/// 包围盒取 TTY RVM（gen-model `test_data/rvm/SCTN-CPAR-SPRF-7324.rvm`）面片的；按 `CUTP` 同向放，整只翻到
/// 柱的另一侧、差 62 mm。
#[test]
fn mds_section_joints_turn_with_their_rolled_beam() {
    if !fixtures_present(MDS_DB) {
        return;
    }
    let scratch: PathBuf =
        std::env::temp_dir().join(format!("e3d-model-rvm-mds-sjoint-{}", std::process::id()));
    let (_, elements) = generate(MDS_DB, "23708/25783", "on", &scratch);
    assert_world_aabb(
        &elements,
        "23708/25783",
        [10020.8, 1829.5, 0.0, 10061.8, 1870.5, 41.0],
        0.05,
    );

    std::fs::remove_dir_all(&scratch).ok();
}

/// aps7200 柱上 SNOD `/B-3-SNOD-7`（`23584/6167`）名下的法兰端板 `SJOI 1`（`23584/6168`，目录
/// `/6M16_flange`）：E3D TTY 在项目副本上导出（gen-model
/// `verification/civil-tty-20261021/aps7200-w7h/aps7200_lv6_23584_6100.rvm`）写 1 条 `Box[206.4, 206.8, 12]`
/// （端板）、13 条 `Cylinder` 与 12 条面片。它 NGMS 里的 NSBO（`NAPP -1`）把梁 `/B-BC-1A`（`23584/6319`，`JOIE`
/// 就指这只关节）的端头退到端板内侧面，梁比不切短 11.95 mm；世界包围盒取 E3D 面片的。此前关节记「无三维
/// 表示」、梁端没切。
#[test]
fn section_joint_flanges_draw_and_cut_their_beam() {
    if !fixtures_present_in(APS_DIR, APS_COPY_ONLY_DB) {
        return;
    }
    let scratch: PathBuf =
        std::env::temp_dir().join(format!("e3d-model-rvm-sjoint-aps-{}", std::process::id()));
    let (rvm, _) = generate_in(
        APS_DIR,
        APS_COPY_ONLY_DB,
        "23584/6167",
        "on",
        &scratch.join("joint"),
    );
    let name = "SJOI 1 of SNOD /B-3-SNOD-7";
    let kinds = record_kinds(&rvm, name);
    let count = |kind: &str| kinds.iter().filter(|got| **got == kind).count();
    assert_eq!(
        (
            count("Box"),
            count("Cylinder"),
            count("FacetGroup"),
            kinds.len()
        ),
        (1, 13, 12, 26),
        "{kinds:?}"
    );
    let boxes: Vec<[f64; 3]> = member(&rvm, name)
        .geometries
        .iter()
        .filter_map(|geometry| match geometry.primitive {
            Primitive::Box(parameters) => Some(parameters),
            _ => None,
        })
        .collect();
    let [[x, y, z]] = boxes.as_slice() else {
        panic!("端板应是一条 Box，实际 {boxes:?}");
    };
    assert!(
        (x - 206.4).abs() < 0.01 && (y - 206.8).abs() < 0.01 && (z - 12.0).abs() < 0.01,
        "Box[{x}, {y}, {z}]"
    );
    let (_, elements) = generate_in(
        APS_DIR,
        APS_COPY_ONLY_DB,
        "23584/6319",
        "on",
        &scratch.join("beam"),
    );
    assert_world_aabb(
        &elements,
        "23584/6319",
        [6686.0, 1923.05, 5672.1, 7665.1, 2056.95, 5878.9],
        0.05,
    );

    std::fs::remove_dir_all(&scratch).ok();
}

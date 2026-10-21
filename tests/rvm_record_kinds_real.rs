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
const TEMPLATE_DIR: &str = r"E:\reverse\e3d\shadow_e3d31_aps_all";
const IJOINT1: &str = "23708/42148";
const IJOINT1_NAME: &str = "/SUPPORT/FRAMES/BS/FT03H/9-V1/S1/IJOINT1";

fn fixtures_present(design_db: &str) -> bool {
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

/// 只生成 `root` 这一只，返回写出的 RVM 与 `elements.json`。
fn generate(design_db: &str, root: &str, holes: &str, out: &Path) -> (RvmFile, serde_json::Value) {
    let template = Path::new(TEMPLATE_DIR);
    let status = Command::new(env!("CARGO_BIN_EXE_gen_ams"))
        .arg("--attlib")
        .arg(template.join("attlib.dat"))
        .arg("--template")
        .arg(template)
        .arg("--db")
        .arg(Path::new(AMS_DIR).join(design_db))
        .args(["--db-type", "DESI", "--root", root])
        .args(["--project-dir", AMS_DIR, "--project-dir", ACP_DIR])
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

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
//! 缺 E3D 安装或影子资产默认红（与 `increment_real.rs` 同一口径）；无语料环境显式设
//! `E3D_SKIP_REAL_GATES=1` 换回跳过。

use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

use e3d_model::rvm_read::RvmFile;

const AMS_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaMarineSample\ams000";
const ACP_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaCatalogue\acp000";
const DESIGN_DB: &str = "ams7324_0001";
const TEMPLATE_DIR: &str = r"E:\reverse\e3d\shadow_e3d31_aps_all";
const IJOINT1: &str = "23708/42148";
const IJOINT1_NAME: &str = "/SUPPORT/FRAMES/BS/FT03H/9-V1/S1/IJOINT1";

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

/// 只生成 `root` 这一只，返回写出的 RVM 与 `elements.json`。
fn generate(root: &str, holes: &str, out: &Path) -> (RvmFile, serde_json::Value) {
    let template = Path::new(TEMPLATE_DIR);
    let status = Command::new(env!("CARGO_BIN_EXE_gen_ams"))
        .arg("--attlib")
        .arg(template.join("attlib.dat"))
        .arg("--template")
        .arg(template)
        .arg("--db")
        .arg(Path::new(AMS_DIR).join(DESIGN_DB))
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

fn record_kinds(rvm: &RvmFile, name: &str) -> Vec<&'static str> {
    let member = rvm
        .members
        .iter()
        .find(|member| member.name.trim() == name)
        .unwrap_or_else(|| panic!("RVM 里没有 {name}"));
    member
        .geometries
        .iter()
        .map(|geometry| geometry.primitive.name())
        .collect()
}

#[test]
fn a_joint_with_primitive_holes_is_one_holed_facet_group() {
    if !fixtures_present() {
        return;
    }
    let scratch: PathBuf =
        std::env::temp_dir().join(format!("e3d-model-rvm-kinds-{}", std::process::id()));

    let (rvm, elements) = generate(IJOINT1, "on", &scratch.join("holes-on"));
    assert_eq!(record_kinds(&rvm, IJOINT1_NAME), ["FacetGroup"]);
    let element = &elements[0];
    assert_eq!(element["negatives_applied"], 2, "{element}");
    // 两块板各一个通孔：写进 RVM 的就是这只亏格 2 的网格。
    assert_eq!(element["genus"], 2, "{element}");

    let (rvm, _) = generate(IJOINT1, "off", &scratch.join("holes-off"));
    assert_eq!(record_kinds(&rvm, IJOINT1_NAME), ["Box", "Box"]);

    std::fs::remove_dir_all(&scratch).ok();
}

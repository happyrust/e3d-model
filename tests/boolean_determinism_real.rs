//! 真库门：同一个根连生成两遍，写出的 RVM 必须逐字节相同（e3d-model#9）。
//!
//! ams7997 `EQUI /Copy-of-1RCS000BA-BODY-NEW`（`24381/156976`）的 `SUBE 8` 下有六只扣过负体的 CYLI：
//! 此前连跑两遍 5 / 6 只的三角剖分不同（体积相同、三角数不同），几何不差，但 gen-model 按内容哈希存烘焙网格
//! （`baked_mesh_id`），每次重生成都是新 id。根因不在 manifold（并行与否都确定），在 `solid::face_planes`：
//! 近共面吸附的面表用 `HashMap` 分桶，每个进程一个种子，同量外推的先后随之变，布尔结果的剖分也就变了。
//! 改成有序桶后这只 EQUI 连跑五遍逐字节相同，这条门把它钉住。
//!
//! 缺 E3D 安装或影子资产默认红（与 `rvm_record_kinds_real.rs` 同一口径）；无语料环境显式设
//! `E3D_SKIP_REAL_GATES=1` 换回跳过。

use std::path::Path;
use std::process::{Command, Stdio};

const AMS_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaMarineSample\ams000";
const ACP_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaCatalogue\acp000";
const TEMPLATE_DIR: &str = r"E:\reverse\e3d\shadow_e3d31_aps_all";
const SSC_DB: &str = "ams7997_0001";
/// `EQUI /Copy-of-1RCS000BA-BODY-NEW`：`SUBE 8` 下六只扣过负体的 CYLI。
const BOOLEAN_EQUI: &str = "24381/156976";

fn fixtures_present() -> bool {
    let attlib = Path::new(TEMPLATE_DIR).join("attlib.dat");
    let design = Path::new(AMS_DIR).join(SSC_DB);
    let missing: Vec<String> = [design.as_path(), Path::new(ACP_DIR), attlib.as_path()]
        .into_iter()
        .filter(|path| !path.exists())
        .map(|path| path.display().to_string())
        .collect();
    if missing.is_empty() {
        return true;
    }
    if std::env::var_os("E3D_SKIP_REAL_GATES").is_some() {
        eprintln!("跳过（E3D_SKIP_REAL_GATES 已设）：缺 {}", missing.join("、"));
        return false;
    }
    panic!(
        "真库门语料缺失：{}。缺席不许静默变绿；确属无语料环境要跳过，显式设 E3D_SKIP_REAL_GATES=1。",
        missing.join("、"),
    );
}

/// 只生成 `root` 这一只，返回 `generated.rvm` 的字节与 `elements.json` 里每件的三角数。
fn generate(root: &str, out: &Path) -> (Vec<u8>, Vec<(String, u64)>) {
    let template = Path::new(TEMPLATE_DIR);
    let status = Command::new(env!("CARGO_BIN_EXE_gen_ams"))
        .arg("--attlib")
        .arg(template.join("attlib.dat"))
        .arg("--template")
        .arg(template)
        .arg("--db")
        .arg(Path::new(AMS_DIR).join(SSC_DB))
        .args(["--db-type", "DESI", "--root", root])
        .args(["--project-dir", AMS_DIR, "--project-dir", ACP_DIR])
        .arg("--out")
        .arg(out)
        .arg("--rvm")
        .stdout(Stdio::null())
        .status()
        .unwrap();
    assert!(status.success(), "gen_ams {root} 失败：{status}");
    let rvm = std::fs::read(out.join("generated.rvm")).unwrap();
    let elements: serde_json::Value =
        serde_json::from_slice(&std::fs::read(out.join("elements.json")).unwrap()).unwrap();
    let triangles = elements
        .as_array()
        .unwrap()
        .iter()
        .map(|element| {
            (
                element["refno"].as_str().unwrap().to_string(),
                element["triangles"].as_u64().unwrap_or(0),
            )
        })
        .collect();
    (rvm, triangles)
}

#[test]
fn generating_the_same_root_twice_writes_byte_identical_rvm() {
    if !fixtures_present() {
        return;
    }
    let dir = std::env::temp_dir().join(format!(
        "e3d-model-boolean-determinism-{}",
        std::process::id()
    ));
    let first_dir = dir.join("first");
    let second_dir = dir.join("second");
    std::fs::create_dir_all(&first_dir).unwrap();
    std::fs::create_dir_all(&second_dir).unwrap();

    let (first_rvm, first_triangles) = generate(BOOLEAN_EQUI, &first_dir);
    let (second_rvm, second_triangles) = generate(BOOLEAN_EQUI, &second_dir);
    let _ = std::fs::remove_dir_all(&dir);

    let differing: Vec<String> = first_triangles
        .iter()
        .zip(&second_triangles)
        .filter(|(a, b)| a != b)
        .map(|(a, b)| format!("{} 三角数 {} / {}", a.0, a.1, b.1))
        .collect();
    assert!(
        differing.is_empty(),
        "同一个根两遍生成的三角剖分不同（布尔非确定性回来了：查布尔路径上有没有按 HashMap 迭代序定顺序的地方，如 `solid::face_planes`）：{}",
        differing.join("；")
    );
    assert!(
        first_triangles.len() >= 700,
        "这只 EQUI 该有 700 多件，只得到 {}",
        first_triangles.len()
    );
    assert!(
        first_rvm == second_rvm,
        "同一个根两遍生成的 generated.rvm 字节不同（{} / {} 字节）",
        first_rvm.len(),
        second_rvm.len()
    );
}

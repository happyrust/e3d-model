//! 真库门：零长 SCTN 名下的 SNOD→SJOI 不许进 `failed`，整根照常出件。
//!
//! ams7330 `/MDS/SPECIALS/PIPE/3`（`FRMW 23714/998`）与 `/PIPE/4`（`FRMW 23714/1023`）各有一根 POSS 与 POSE 重合的
//! `SCTN`（`23714/1011`、`23714/1025`，轴长 0 mm），名下 `SNOD`→`SJOI`（`23714/1013`、`23714/1027`）。E3D 的 TTY 整库
//! RVM 导出（`mds_xfr_lv6_23714_1.rvm`）里这些件都没有图元，同框的 `23714/1001`、`1024`、`1029` 逐件 exact。
//! 此前 SJOI 从宿主轴架现算标架时拿到 `Degenerate` 记成 `failed`；gen-model 服务端按「根上有 failed 就整根不发」
//! 处置，2026-10-26 线上 7330 初始化回执里这两框的 STRU / FRMW 四根 failed、0 条记录，好杆件一并丢了。
//! 现在这类关节记 skipped（no shape），这条门把「这两根的 failed 为空、关节在 skipped、好杆件照出」钉住。
//!
//! 缺 E3D 安装或影子资产默认红（与 `boolean_determinism_real.rs` 同一口径）；无语料环境显式设
//! `E3D_SKIP_REAL_GATES=1` 换回跳过。

use std::path::Path;
use std::process::{Command, Stdio};

const AMS_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaMarineSample\ams000";
const ACP_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaCatalogue\acp000";
const TEMPLATE_DIR: &str = r"E:\reverse\e3d\shadow_e3d31_aps_all";
const SPECIALS_DB: &str = "ams7330_0001";

/// (根, 零长宿主 SCTN, 其 SNOD 下的 SJOI, 同框该照常出件的杆件)。
const FRAMES: [(&str, &str, &str, &[&str]); 2] = [
    ("23714/998", "23714/1011", "23714/1013", &["23714/1001"]),
    ("23714/1023", "23714/1025", "23714/1027", &["23714/1024", "23714/1029"]),
];

fn fixtures_present() -> bool {
    let attlib = Path::new(TEMPLATE_DIR).join("attlib.dat");
    let design = Path::new(AMS_DIR).join(SPECIALS_DB);
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

/// 只生成 `root` 这一只，返回 `report.json` 与 `elements.json`。
fn generate(root: &str, out: &Path) -> (serde_json::Value, serde_json::Value) {
    let template = Path::new(TEMPLATE_DIR);
    let status = Command::new(env!("CARGO_BIN_EXE_gen_ams"))
        .arg("--attlib")
        .arg(template.join("attlib.dat"))
        .arg("--template")
        .arg(template)
        .arg("--db")
        .arg(Path::new(AMS_DIR).join(SPECIALS_DB))
        .args(["--db-type", "DESI", "--root", root])
        .args(["--project-dir", AMS_DIR, "--project-dir", ACP_DIR])
        .arg("--out")
        .arg(out)
        .stdout(Stdio::null())
        .status()
        .unwrap();
    assert!(status.success(), "gen_ams {root} 失败：{status}");
    let report = serde_json::from_slice(&std::fs::read(out.join("report.json")).unwrap()).unwrap();
    let elements =
        serde_json::from_slice(&std::fs::read(out.join("elements.json")).unwrap()).unwrap();
    (report, elements)
}

fn incidents<'a>(report: &'a serde_json::Value, book: &str) -> Vec<(&'a str, &'a str)> {
    report[book]
        .as_array()
        .map(|items| {
            items
                .iter()
                .map(|item| {
                    (
                        item["refno"].as_str().unwrap_or(""),
                        item["detail"].as_str().unwrap_or(""),
                    )
                })
                .collect()
        })
        .unwrap_or_default()
}

#[test]
fn frames_with_a_zero_length_section_publish_without_failed_elements() {
    if !fixtures_present() {
        return;
    }
    let dir = std::env::temp_dir().join(format!(
        "e3d-model-zero-length-section-joint-{}",
        std::process::id()
    ));
    for (root, host, joint, members) in FRAMES {
        let out = dir.join(root.replace('/', "_"));
        std::fs::create_dir_all(&out).unwrap();
        let (report, elements) = generate(root, &out);

        let failed = incidents(&report, "failed");
        assert!(
            failed.is_empty(),
            "{root}：零长宿主上的关节又进了 failed（gen-model 会据此整根不发）：{failed:?}"
        );
        let skipped = incidents(&report, "skipped");
        assert!(
            skipped
                .iter()
                .any(|(refno, detail)| *refno == joint && detail.contains(host) && detail.contains("0.001 mm")),
            "{root}：SJOI {joint} 该在 skipped 里点名没有轴的宿主 {host}：{skipped:?}"
        );
        assert!(
            skipped.iter().any(|(refno, _)| *refno == host),
            "{root}：零长 SCTN {host} 自己也该在 skipped 里：{skipped:?}"
        );

        let generated: Vec<&str> = elements
            .as_array()
            .unwrap()
            .iter()
            .filter_map(|element| element["refno"].as_str())
            .collect();
        for member in members {
            assert!(
                generated.contains(member),
                "{root}：同框杆件 {member} 该照常出件，实际只有 {generated:?}"
            );
        }
        assert!(
            !generated.contains(&joint) && !generated.contains(&host),
            "{root}：零长 SCTN 与其关节不该出件（E3D 也不画）：{generated:?}"
        );
    }
    let _ = std::fs::remove_dir_all(&dir);
}

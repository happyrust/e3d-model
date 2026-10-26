//! 真库门：生成时随几何交出的目录 P 点集（[`e3d_model::GenerationOutcome::point_sets`] → `gen_ams` 的 `point_sets.json`）
//! 对 E3D 自己算的设计级伪属性 `PPOS[n]` / `PDIR[n]` / `PPBORE[n]`，逐成员、逐点比位置 / 方向 / 通径。
//!
//! 真值 `tests/fixtures/ptset/ams-ptset-truth-20261027.psv`：gen-model `verification/civil-tty-20261021/ptset_truth.mac`
//! 在只读 TTY 会话（AMS `/ALL`，运行时 `shadow_e3d31_gen_model_test`，2026-10-27）里取：8 个根（ams8000 两条管道 BRAN +
//! 一条带阀门的、ams7997 风管 BRAN `/-VES-E-4-L-2501` + HANG + 带 NOZZ 的 SUBE、ams7999 带 1WCC 阀门的 BRAN + HANG）
//! 215 个直属成员、1471 个 P 点。坐标在**成员自己的坐标系**（PML `POSITION.wrt(成员)`），与我们交出的局部系同口径；
//! 通径 `Unset` 对 `None`。在这之前 `tests/` 里 P 点的真值是 0 条，间接证据只有 LEAV / ARRI 定的管身。
//!
//! 门同时钉住**覆盖面**：真值里每个路由成员 / NOZZ（含没有目录链的）在 `point_sets.json` 里都有一条，点的编号集合相同，
//! 没有一个点落在 `unresolved`。容差：位置 1e-3 mm，方向 1e-6，通径 1e-6。
//!
//! 缺 E3D 安装或影子资产默认红（与 `rvm_record_kinds_real.rs` 同一口径）；无语料环境显式设
//! `E3D_SKIP_REAL_GATES=1` 换回跳过。

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;
use std::process::{Command, Stdio};

const AMS_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaMarineSample\ams000";
const ACP_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaCatalogue\acp000";
/// AMS 的 MDB `/ALL` 挂着 ZDJ 的目录库（ams7999 / ams8000 的 `1WCC` / `1RCV` 管件的 SPRE 落在 db 7600 `/NPHS-CATA`）。
const ZDJ_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\ZDJ\ZDJ000";
/// 库 7000 在 ams000 与 acp000 各有一份主文件，目录扫描分不出正身；与 `output/s1_regress.py` 同样显式点名。
const EXPLICIT_CATALOGUES: [&str; 2] = [
    r"D:\AVEVA\Projects\E3D3.1\AvevaMarineSample\ams000\ams7000_0001",
    r"D:\AVEVA\Projects\E3D3.1\SCB\scb000\scb6000_0001",
];
const TEMPLATE_DIR: &str = r"E:\reverse\e3d\shadow_e3d31_aps_all";
const TRUTH: &str = include_str!("fixtures/ptset/ams-ptset-truth-20261027.psv");

const POS_TOL_MM: f64 = 1e-3;
const DIR_TOL: f64 = 1e-6;
const BORE_TOL_MM: f64 = 1e-6;

/// 真值里的 refno 库号 → 设计库文件。
fn design_db(dbno: u32) -> &'static str {
    match dbno {
        24381 => "ams7997_0001",
        24383 => "ams7999_0001",
        24384 => "ams8000_0001",
        other => panic!("真值里有未登记的库号 {other}"),
    }
}

fn fixtures_present(design_dbs: &BTreeSet<&'static str>) -> bool {
    let attlib = Path::new(TEMPLATE_DIR).join("attlib.dat");
    let mut required = vec![
        Path::new(ACP_DIR).to_path_buf(),
        Path::new(ZDJ_DIR).to_path_buf(),
        attlib,
    ];
    required.extend(
        EXPLICIT_CATALOGUES
            .iter()
            .map(|path| Path::new(path).to_path_buf()),
    );
    required.extend(design_dbs.iter().map(|db| Path::new(AMS_DIR).join(db)));
    let missing: Vec<String> = required
        .iter()
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

/// 只生成 `root` 这一只，返回 `point_sets.json`。
fn generate(design_db: &str, root: &str, out: &Path) -> serde_json::Value {
    let template = Path::new(TEMPLATE_DIR);
    let status = Command::new(env!("CARGO_BIN_EXE_gen_ams"))
        .arg("--attlib")
        .arg(template.join("attlib.dat"))
        .arg("--template")
        .arg(template)
        .arg("--db")
        .arg(Path::new(AMS_DIR).join(design_db))
        .args(["--db-type", "DESI", "--root", root])
        .args([
            "--project-dir",
            AMS_DIR,
            "--project-dir",
            ZDJ_DIR,
            "--project-dir",
            ACP_DIR,
        ])
        .args(
            EXPLICIT_CATALOGUES
                .iter()
                .flat_map(|path| ["--catalogue", path]),
        )
        .arg("--out")
        .arg(out)
        .stdout(Stdio::null())
        .status()
        .unwrap();
    assert!(status.success(), "gen_ams {root} 失败：{status}");
    serde_json::from_slice(&std::fs::read(out.join("point_sets.json")).unwrap()).unwrap()
}

/// 真值的一行：一个成员的一个 P 点，或者「这个成员没有目录链」（`number` 为 `None`，原因在 `err`）。
#[derive(Debug)]
struct TruthRow {
    root: String,
    member: String,
    noun: String,
    number: Option<u32>,
    pos: [f64; 3],
    dir: [f64; 3],
    bore: Option<f64>,
    err: String,
}

fn mm(field: &str) -> f64 {
    field
        .trim()
        .trim_end_matches("mm")
        .parse()
        .unwrap_or_else(|_| panic!("真值里的长度读不成数：{field:?}"))
}

fn parse_truth() -> Vec<TruthRow> {
    let mut rows = Vec::new();
    for (index, line) in TRUTH.trim_start_matches('\u{feff}').lines().enumerate() {
        if index == 0 || line.trim().is_empty() {
            continue;
        }
        let f: Vec<&str> = line.split('|').collect();
        assert_eq!(f.len(), 20, "真值第 {} 行不是 20 列：{line}", index + 1);
        let refno = |field: &str| field.trim().trim_start_matches('=').to_string();
        let number = f[4].trim();
        if number.is_empty() {
            rows.push(TruthRow {
                root: refno(f[0]),
                member: refno(f[1]),
                noun: f[2].trim().to_string(),
                number: None,
                pos: [0.0; 3],
                dir: [0.0; 3],
                bore: None,
                err: f[19].trim().to_string(),
            });
            continue;
        }
        let bore = match f[18].trim() {
            "" | "Unset" => None,
            value => Some(mm(value)),
        };
        rows.push(TruthRow {
            root: refno(f[0]),
            member: refno(f[1]),
            noun: f[2].trim().to_string(),
            number: Some(number.parse().unwrap()),
            pos: [mm(f[6]), mm(f[7]), mm(f[8])],
            dir: [
                f[9].trim().parse().unwrap(),
                f[10].trim().parse().unwrap(),
                f[11].trim().parse().unwrap(),
            ],
            bore,
            err: f[19].trim().to_string(),
        });
    }
    rows
}

/// 真值里的这个成员该不该在 `point_sets.json` 里：路由成员与 NOZZ 要，SUBE / EQUI 下的设计基本体（CYLI / BOX / PYRA）不要。
fn expected_in_point_sets(noun: &str) -> bool {
    noun == "NOZZ"
        || matches!(
            e3d_model::decision(noun).category,
            e3d_model::Category::RouteMember
        )
}

fn xyz(value: &serde_json::Value) -> [f64; 3] {
    let a = value.as_array().unwrap();
    [
        a[0].as_f64().unwrap(),
        a[1].as_f64().unwrap(),
        a[2].as_f64().unwrap(),
    ]
}

fn max_abs_diff(a: [f64; 3], b: [f64; 3]) -> f64 {
    (0..3).map(|i| (a[i] - b[i]).abs()).fold(0.0, f64::max)
}

#[test]
fn generated_point_sets_match_e3d_ppos_pdir_pbore() {
    let truth = parse_truth();
    let mut by_root: BTreeMap<String, Vec<&TruthRow>> = BTreeMap::new();
    for row in &truth {
        by_root.entry(row.root.clone()).or_default().push(row);
    }
    let dbs: BTreeSet<&'static str> = by_root
        .keys()
        .map(|root| design_db(root.split('/').next().unwrap().parse().unwrap()))
        .collect();
    if !fixtures_present(&dbs) {
        return;
    }
    let dir = std::env::temp_dir().join(format!("e3d-model-ptset-{}", std::process::id()));

    let mut mismatches: Vec<String> = Vec::new();
    let mut compared_points = 0usize;
    let mut compared_members = 0usize;
    for (root, rows) in &by_root {
        let db = design_db(root.split('/').next().unwrap().parse().unwrap());
        let out = dir.join(root.replace('/', "_"));
        std::fs::create_dir_all(&out).unwrap();
        let file = generate(db, root, &out);
        assert_eq!(
            file["eval_version"].as_u64(),
            Some(u64::from(e3d_model::PTSET_EVAL_VERSION)),
            "{root}：point_sets.json 的 eval_version 不是库里的常量"
        );
        let records: BTreeMap<&str, &serde_json::Value> = file["members"]
            .as_array()
            .unwrap()
            .iter()
            .map(|record| (record["refno"].as_str().unwrap(), record))
            .collect();

        let mut members: BTreeMap<&str, Vec<&TruthRow>> = BTreeMap::new();
        for row in rows {
            members.entry(row.member.as_str()).or_default().push(row);
        }
        let mut root_points = 0usize;
        for (member, rows) in &members {
            let noun = rows[0].noun.as_str();
            let Some(record) = records.get(member) else {
                if expected_in_point_sets(noun) {
                    mismatches.push(format!(
                        "{root} {member} {noun}：point_sets.json 里没有这件（E3D 有 {} 个点{}）",
                        rows.iter().filter(|row| row.number.is_some()).count(),
                        if rows[0].err.is_empty() {
                            String::new()
                        } else {
                            format!("，{}", rows[0].err)
                        },
                    ));
                }
                continue;
            };
            compared_members += 1;
            let generated_points = record["points"].as_array().unwrap();
            let unresolved = record["unresolved"].as_array().unwrap();
            if rows.len() == 1 && rows[0].number.is_none() {
                // E3D 这件没有目录链：我们也不许凭空解出点来。
                if !generated_points.is_empty() || !unresolved.is_empty() {
                    mismatches.push(format!(
                        "{root} {member} {noun}：E3D 没有 P 点（{}），我们交了 {} 个点 / {} 个未解",
                        rows[0].err,
                        generated_points.len(),
                        unresolved.len()
                    ));
                }
                continue;
            }
            let truth_numbers: BTreeSet<u32> = rows.iter().filter_map(|row| row.number).collect();
            let generated_numbers: BTreeSet<u32> = generated_points
                .iter()
                .map(|point| point["number"].as_u64().unwrap() as u32)
                .collect();
            let unresolved_numbers: BTreeSet<u32> = unresolved
                .iter()
                .map(|point| point["number"].as_u64().unwrap() as u32)
                .collect();
            let ours: BTreeSet<u32> = generated_numbers
                .union(&unresolved_numbers)
                .copied()
                .collect();
            if ours != truth_numbers {
                mismatches.push(format!(
                    "{root} {member} {noun}：P 点编号集合不同：E3D {truth_numbers:?}，我们 {ours:?}"
                ));
            }
            for number in &unresolved_numbers {
                let reason = unresolved
                    .iter()
                    .find(|point| point["number"].as_u64() == Some(u64::from(*number)))
                    .and_then(|point| point["reason"].as_str())
                    .unwrap_or("");
                mismatches.push(format!(
                    "{root} {member} {noun} P{number}：E3D 解得出，我们记 unresolved：{reason}"
                ));
            }
            for row in rows {
                let Some(number) = row.number else { continue };
                let Some(point) = generated_points
                    .iter()
                    .find(|point| point["number"].as_u64() == Some(u64::from(number)))
                else {
                    continue;
                };
                root_points += 1;
                let pos = xyz(&point["pos"]);
                let dir = xyz(&point["dir"]);
                let bore = point["bore"].as_f64();
                let pos_diff = max_abs_diff(pos, row.pos);
                let dir_diff = max_abs_diff(dir, row.dir);
                let bore_ok = match (bore, row.bore) {
                    (None, None) => true,
                    (Some(a), Some(b)) => (a - b).abs() <= BORE_TOL_MM,
                    _ => false,
                };
                if pos_diff > POS_TOL_MM || dir_diff > DIR_TOL || !bore_ok {
                    mismatches.push(format!(
                        "{root} {member} {noun} P{number}：位置差 {pos_diff:.6} mm（E3D {:?} / 我们 {:?}），方向差 {dir_diff:.3e}（E3D {:?} / 我们 {:?}），通径 E3D {:?} / 我们 {:?}",
                        row.pos, pos, row.dir, dir, row.bore, bore
                    ));
                }
            }
        }
        eprintln!(
            "{root}：真值成员 {} / point_sets.json 成员 {} / 比了 {root_points} 个点",
            members.len(),
            records.len()
        );
        compared_points += root_points;
    }
    eprintln!(
        "共比了 {compared_members} 件 {compared_points} 个点，{} 处不对",
        mismatches.len()
    );
    assert!(
        compared_points >= 1400,
        "比到的点太少（{compared_points}），真值或生成物有一边不对劲"
    );
    assert!(
        mismatches.is_empty(),
        "P 点与 E3D 不对（共 {} 处，列前 60）：\n{}",
        mismatches.len(),
        mismatches
            .iter()
            .take(60)
            .cloned()
            .collect::<Vec<_>>()
            .join("\n")
    );
}

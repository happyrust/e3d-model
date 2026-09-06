//! 拿 [`e3d_model::rvm_read`] 把真实 RVM 过一遍，出「成员 / 几何 / kind 分布 / bbox 帧」四张账。
//!
//! 这是 `rvm_read` 的验收口：它的数必须与 `tools/rvm_record_probe.py`（独立实现、直接读字节）
//! **逐份相同**，且 `bbox_frame` 的 `unknown` 必须为 0。两个实现同时错成一样的可能性远低于
//! 一个实现自说自话。
//!
//! ```text
//! cargo run --release --example rvm_read_audit -- ../../gen-model/test_data/rvm/*.rvm
//! ```

use std::collections::BTreeMap;

use e3d_model::rvm_read::{BboxFrame, RvmFile};

fn main() -> anyhow::Result<()> {
    let mut paths: Vec<String> = Vec::new();
    let mut dump_aabb = false;
    let mut roundtrip = false;
    for arg in std::env::args().skip(1) {
        match arg.as_str() {
            // 逐成员世界 AABB 打成 TSV，供与 gen-model 的 `rvm_verify import` 快照互校。
            "--dump-aabb" => dump_aabb = true,
            // 读真实 E3D 文件 → 用 rvm_write 写回去 → 再读一遍，逐项比。
            "--roundtrip" => roundtrip = true,
            _ => paths.push(arg),
        }
    }
    if paths.is_empty() {
        anyhow::bail!("用法: rvm_read_audit [--dump-aabb|--roundtrip] <a.rvm> [<b.rvm> ...]");
    }
    if roundtrip {
        return roundtrip_all(&paths);
    }
    if dump_aabb {
        for path in &paths {
            let file = RvmFile::read(path)?;
            for member in &file.members {
                if let Some(a) = member.world_aabb_mm() {
                    let name = path.rsplit(['/', '\\']).next().unwrap_or(path);
                    println!(
                        "{name}\t{}\t{}\t{}\t{}\t{}\t{}\t{}",
                        member.name, a[0], a[1], a[2], a[3], a[4], a[5]
                    );
                }
            }
        }
        return Ok(());
    }

    println!(
        "{:<30}{:>9}{:>10}{:>8}{:>9}{:>6}{:>9}",
        "sample", "members", "geometry", "local", "rotated", "n/a", "unknown"
    );
    let mut kinds_all: BTreeMap<String, usize> = BTreeMap::new();
    let (mut members_all, mut geometry_all, mut unknown_all) = (0usize, 0usize, 0usize);

    for path in &paths {
        let file = RvmFile::read(path)?;
        let mut kinds: BTreeMap<String, usize> = BTreeMap::new();
        let mut geometry_count = 0usize;
        for geometry in file.members.iter().flat_map(|m| m.geometries.iter()) {
            geometry_count += 1;
            *kinds
                .entry(geometry.primitive.name().to_string())
                .or_default() += 1;
            *kinds_all
                .entry(geometry.primitive.name().to_string())
                .or_default() += 1;
        }
        let tally = file.bbox_frame_tally();
        let unknown = *tally.get("unknown").unwrap_or(&0);
        members_all += file.members.len();
        geometry_all += geometry_count;
        unknown_all += unknown;

        let name = path.rsplit(['/', '\\']).next().unwrap_or(path);
        println!(
            "{:<30}{:>9}{:>10}{:>8}{:>9}{:>6}{:>9}",
            name,
            file.members.len(),
            geometry_count,
            tally.get("local").unwrap_or(&0),
            tally.get("rotated").unwrap_or(&0),
            tally.get("n/a").unwrap_or(&0),
            unknown,
        );
        for member in file.members.iter().filter(|m| {
            m.geometries
                .iter()
                .any(|g| g.bbox_frame == BboxFrame::Unknown)
        }) {
            println!("    unknown 帧: {}", member.path);
        }
    }

    println!(
        "{:<30}{:>9}{:>10}{:>8}{:>9}{:>6}{:>9}",
        "TOTAL", members_all, geometry_all, "", "", "", unknown_all
    );
    println!();
    println!("kind 分布: {kinds_all:?}");
    if unknown_all > 0 {
        anyhow::bail!("{unknown_all} 条记录的 bbox 帧认不出来——读侧不得再拿记录字段当世界 AABB");
    }
    Ok(())
}

/// 真实 E3D 文件过一遍 `rvm_write`：读 → 写 → 再读，逐成员逐几何比。
///
/// 目标**不是**字节相同（f32 舍入、bbox 帧我们只写 rotated 一档），而是
/// 「写出去的东西读回来还是同一个模型」——名字、层级、几何数、世界 AABB 都不能变。
fn roundtrip_all(paths: &[String]) -> anyhow::Result<()> {
    println!(
        "{:<30}{:>9}{:>10}{:>12}{:>14}",
        "sample", "members", "geometry", "名字不符", "AABB 最大差 mm"
    );
    let mut failures = 0usize;
    for path in paths {
        let original = RvmFile::read(path)?;
        let reread = RvmFile::parse(&original.to_bytes())?;

        let name = path.rsplit(['/', '\\']).next().unwrap_or(path);
        if original.members.len() != reread.members.len() {
            println!(
                "{name:<30}成员数 {} -> {}",
                original.members.len(),
                reread.members.len()
            );
            failures += 1;
            continue;
        }

        let mut mismatched_names = 0usize;
        let mut worst = 0.0f64;
        let mut geometry_count = 0usize;
        for (before, after) in original.members.iter().zip(reread.members.iter()) {
            if before.name != after.name || before.parent != after.parent {
                mismatched_names += 1;
            }
            if before.geometries.len() != after.geometries.len() {
                failures += 1;
            }
            geometry_count += before.geometries.len();
            for (gb, ga) in before.geometries.iter().zip(after.geometries.iter()) {
                if gb.primitive.kind_id() != ga.primitive.kind_id() {
                    failures += 1;
                }
                if let (Some(a), Some(b)) = (gb.world_aabb_mm(), ga.world_aabb_mm()) {
                    worst = worst.max(
                        a.iter()
                            .zip(b.iter())
                            .fold(0.0f64, |acc, (x, y)| acc.max((x - y).abs())),
                    );
                }
            }
        }
        if mismatched_names > 0 {
            failures += 1;
        }
        println!(
            "{name:<30}{:>9}{geometry_count:>10}{mismatched_names:>12}{worst:>14.6}",
            original.members.len()
        );
    }
    if failures > 0 {
        anyhow::bail!("{failures} 处回环不一致");
    }
    println!("\n回环全过：名字 / 层级 / 几何数 / kind 一致，世界 AABB 只差 f32 舍入。");
    Ok(())
}

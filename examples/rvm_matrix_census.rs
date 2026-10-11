//! 两条 RVM 的记录矩阵逐列普查。
//!
//! 按组名对齐（只统计基准侧出现的组；生成文件常常是整库，库外的组不算配不上）。
//! 组内同 kind 按世界 AABB 中心就近配对，再把 X/Y/Z/T 和参数分到互斥档：
//! 四列+参数全同、只差绕 Z 的滚角、Z 反号、只差平移、只差参数、帧不同（参数相同）、
//! 多项不同、配不上。
//!
//! 318VN 自检：`--contains 1WCC318VN` 应得到 15 条全同、1 条 FacetGroup 帧不同。
//!
//! ```text
//! cargo run --release --example rvm_matrix_census -- \
//!     --baseline a.rvm --generated b.rvm --contains 1WCC318VN --out census.md
//! ```

use std::collections::BTreeMap;
use std::path::PathBuf;

use e3d_model::rvm_read::{Primitive, RvmFile, RvmGeometry, RvmMember};

const ROT_TOL: f64 = 1e-3;
const TRANS_TOL_MM: f64 = 0.05;
const PARAM_TOL: f64 = 0.05;
/// 组名对不上时，世界中心近到这个距离仍视为同一件。几何真相同的中心差远小于它。
const RESCUE_MM: f64 = 5.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Bucket {
    Exact,
    Roll,
    ZFlip,
    Translation,
    Params,
    Frame,
    Mixed,
    Unmatched,
}

impl Bucket {
    fn label(self) -> &'static str {
        match self {
            Self::Exact => "四列+参数全同",
            Self::Roll => "只差X·Y滚角",
            Self::ZFlip => "Z反号",
            Self::Translation => "T不同",
            Self::Params => "参数不同",
            Self::Frame => "帧不同",
            Self::Mixed => "多项不同",
            Self::Unmatched => "配不上",
        }
    }
}

#[derive(Clone)]
struct Record {
    member: String,
    kind: String,
    center: [f64; 3],
    cols: [[f64; 3]; 3],
    translation: [f64; 3],
    params: Vec<f64>,
}

struct Row {
    kind: String,
    bucket: Bucket,
    member: String,
    detail: String,
}

fn main() -> anyhow::Result<()> {
    let mut baseline = None;
    let mut generated = None;
    let mut contains = None;
    let mut out = None;
    let mut args = std::env::args().skip(1);
    while let Some(flag) = args.next() {
        let value = args
            .next()
            .ok_or_else(|| anyhow::anyhow!("缺少 {flag} 的值"))?;
        match flag.as_str() {
            "--baseline" => baseline = Some(PathBuf::from(value)),
            "--generated" => generated = Some(PathBuf::from(value)),
            "--contains" => contains = Some(value),
            "--out" => out = Some(PathBuf::from(value)),
            other => anyhow::bail!("未知参数 {other}"),
        }
    }
    let baseline = baseline.ok_or_else(|| anyhow::anyhow!("需要 --baseline"))?;
    let generated = generated.ok_or_else(|| anyhow::anyhow!("需要 --generated"))?;

    let left = RvmFile::read(&baseline)?;
    let right = RvmFile::read(&generated)?;
    let rows = census(&left, &right, contains.as_deref());
    let summary = summarize(&rows);
    let text = render(&baseline, &generated, contains.as_deref(), &summary, &rows);
    if let Some(path) = &out {
        if let Some(parent) = path.parent() {
            if !parent.as_os_str().is_empty() {
                std::fs::create_dir_all(parent)?;
            }
        }
        std::fs::write(path, &text)?;
        eprintln!("wrote {}", path.display());
    }
    print!("{text}");
    Ok(())
}

fn census(left: &RvmFile, right: &RvmFile, contains: Option<&str>) -> Vec<Row> {
    let left_recs = collect_records(&left.members, contains);
    let right_recs = collect_records(&right.members, contains);
    let mut right_by_name: BTreeMap<&str, Vec<usize>> = BTreeMap::new();
    for (index, rec) in right_recs.iter().enumerate() {
        right_by_name
            .entry(rec.member.as_str())
            .or_default()
            .push(index);
    }
    let left_names: std::collections::BTreeSet<&str> =
        left_recs.iter().map(|rec| rec.member.as_str()).collect();
    let mut used = vec![false; right_recs.len()];
    let mut rows = Vec::new();
    let mut pending = Vec::new();

    let mut left_by_name: BTreeMap<&str, Vec<usize>> = BTreeMap::new();
    for (index, rec) in left_recs.iter().enumerate() {
        left_by_name
            .entry(rec.member.as_str())
            .or_default()
            .push(index);
    }
    for (name, left_indexes) in &left_by_name {
        let empty = Vec::new();
        let right_indexes = right_by_name.get(name).unwrap_or(&empty);
        for left_index in left_indexes {
            let left_rec = &left_recs[*left_index];
            if let Some(hit) = best_same_kind(left_rec, &right_recs, right_indexes, &used, None) {
                used[hit] = true;
                rows.push(classify(left_rec, &right_recs[hit]));
            } else {
                pending.push(*left_index);
            }
        }
    }

    for left_index in pending {
        let left_rec = &left_recs[left_index];
        let all: Vec<usize> = (0..right_recs.len())
            .filter(|index| !used[*index])
            .collect();
        if let Some(hit) = best_same_kind(left_rec, &right_recs, &all, &used, Some(RESCUE_MM)) {
            used[hit] = true;
            rows.push(classify(left_rec, &right_recs[hit]));
            continue;
        }
        let near = nearest_any(left_rec, &right_recs, &used, RESCUE_MM);
        let why = match near {
            Some(kind) => format!("5mm 内没有同 kind，近处是 {kind}"),
            None => "没有同名同 kind，5mm 内也没有".to_string(),
        };
        rows.push(unmatched(left_rec, &why));
    }

    for (index, rec) in right_recs.iter().enumerate() {
        if used[index] || !left_names.contains(rec.member.as_str()) {
            continue;
        }
        rows.push(unmatched(rec, "生成侧同名组里多出来的"));
    }
    rows
}

fn best_same_kind(
    left: &Record,
    right_recs: &[Record],
    candidates: &[usize],
    used: &[bool],
    max_dist: Option<f64>,
) -> Option<usize> {
    let mut best: Option<(usize, f64, f64)> = None;
    for index in candidates {
        let index = *index;
        if used[index] || right_recs[index].kind != left.kind {
            continue;
        }
        let dist = pair_dist(left, &right_recs[index]);
        if max_dist.is_some_and(|limit| dist > limit) {
            continue;
        }
        let dot = dot3(left.cols[2], right_recs[index].cols[2]).abs();
        let better = match best {
            None => true,
            Some((_, best_dist, best_dot)) => {
                dist < best_dist - 1e-6 || ((dist - best_dist).abs() <= 1e-6 && dot > best_dot)
            }
        };
        if better {
            best = Some((index, dist, dot));
        }
    }
    best.map(|(index, _, _)| index)
}

fn nearest_any(
    left: &Record,
    right_recs: &[Record],
    used: &[bool],
    max_dist: f64,
) -> Option<String> {
    let mut best: Option<(f64, &str)> = None;
    for (index, rec) in right_recs.iter().enumerate() {
        if used[index] {
            continue;
        }
        let dist = pair_dist(left, rec);
        if dist > max_dist {
            continue;
        }
        if best.is_none_or(|(best_dist, _)| dist < best_dist) {
            best = Some((dist, rec.kind.as_str()));
        }
    }
    best.map(|(_, kind)| kind.to_string())
}

fn collect_records(members: &[RvmMember], contains: Option<&str>) -> Vec<Record> {
    let mut records = Vec::new();
    for member in members {
        if contains
            .is_some_and(|needle| !member.name.contains(needle) && !member.path.contains(needle))
        {
            continue;
        }
        records.extend(
            member
                .geometries
                .iter()
                .filter_map(|geometry| record(&member.name, geometry)),
        );
    }
    records
}

fn record(member: &str, geometry: &RvmGeometry) -> Option<Record> {
    let center = world_center(geometry)?;
    let rotation = geometry.rotation();
    Some(Record {
        member: member.to_string(),
        kind: geometry.primitive.name().to_string(),
        center,
        cols: [
            [rotation[0], rotation[1], rotation[2]],
            [rotation[3], rotation[4], rotation[5]],
            [rotation[6], rotation[7], rotation[8]],
        ],
        translation: geometry.translation_mm,
        params: params_of(&geometry.primitive),
    })
}

fn classify(left: &Record, right: &Record) -> Row {
    let x = col_close(left.cols[0], right.cols[0]);
    let y = col_close(left.cols[1], right.cols[1]);
    let z = col_close(left.cols[2], right.cols[2]);
    let z_neg = col_close(left.cols[2], neg(right.cols[2]));
    let t = trans_close(left.translation, right.translation);
    let p = params_close(&left.params, &right.params);
    let bucket = if x && y && z && t && p {
        Bucket::Exact
    } else if z && t && p && !(x && y) {
        Bucket::Roll
    } else if z_neg && !z && t && p {
        Bucket::ZFlip
    } else if x && y && z && !t && p {
        Bucket::Translation
    } else if x && y && z && t && !p {
        Bucket::Params
    } else if p && !(x && y && z && t) {
        Bucket::Frame
    } else {
        Bucket::Mixed
    };
    let d_t = dist3(left.translation, right.translation);
    let z_dot = dot3(left.cols[2], right.cols[2]);
    let p_delta = param_delta(&left.params, &right.params);
    let params = if p {
        String::new()
    } else {
        format!(" Pe={:?} Pg={:?}", left.params, right.params)
    };
    Row {
        kind: left.kind.clone(),
        bucket,
        member: left.member.clone(),
        detail: format!(
            "X{} Y{} Z{} T{} P{} dT={d_t:.2} zdot={z_dot:.3} pΔ={p_delta:.3} Ze=({:.3},{:.3},{:.3}) Zg=({:.3},{:.3},{:.3}){params}",
            mark(x),
            mark(y),
            mark(z),
            mark(t),
            mark(p),
            left.cols[2][0],
            left.cols[2][1],
            left.cols[2][2],
            right.cols[2][0],
            right.cols[2][1],
            right.cols[2][2],
        ),
    }
}

fn param_delta(left: &[f64], right: &[f64]) -> f64 {
    if left.len() != right.len() {
        return f64::INFINITY;
    }
    left.iter()
        .zip(right)
        .map(|(a, b)| (a - b).abs())
        .fold(0.0, f64::max)
}

fn unmatched(rec: &Record, why: &str) -> Row {
    Row {
        kind: rec.kind.clone(),
        bucket: Bucket::Unmatched,
        member: rec.member.clone(),
        detail: why.to_string(),
    }
}

fn summarize(rows: &[Row]) -> BTreeMap<(String, Bucket), (usize, String, String)> {
    let mut map = BTreeMap::<(String, Bucket), (usize, String, String)>::new();
    for row in rows {
        let entry = map.entry((row.kind.clone(), row.bucket)).or_insert((
            0,
            row.member.clone(),
            row.detail.clone(),
        ));
        entry.0 += 1;
    }
    map
}

fn render(
    baseline: &std::path::Path,
    generated: &std::path::Path,
    contains: Option<&str>,
    summary: &BTreeMap<(String, Bucket), (usize, String, String)>,
    rows: &[Row],
) -> String {
    let mut counts = BTreeMap::<Bucket, usize>::new();
    for row in rows {
        *counts.entry(row.bucket).or_default() += 1;
    }
    let mut text = String::new();
    text.push_str("# 矩阵逐列普查\n\n");
    text.push_str(&format!("- baseline: `{}`\n", baseline.display()));
    text.push_str(&format!("- generated: `{}`\n", generated.display()));
    if let Some(needle) = contains {
        text.push_str(&format!("- contains: `{needle}`\n"));
    }
    text.push_str(&format!("- 记录: {}\n", rows.len()));
    text.push_str(&format!(
        "- 合计: 全同 {} / 滚角 {} / Z反号 {} / T不同 {} / 参数不同 {} / 帧不同 {} / 多项 {} / 配不上 {}\n\n",
        counts.get(&Bucket::Exact).copied().unwrap_or(0),
        counts.get(&Bucket::Roll).copied().unwrap_or(0),
        counts.get(&Bucket::ZFlip).copied().unwrap_or(0),
        counts.get(&Bucket::Translation).copied().unwrap_or(0),
        counts.get(&Bucket::Params).copied().unwrap_or(0),
        counts.get(&Bucket::Frame).copied().unwrap_or(0),
        counts.get(&Bucket::Mixed).copied().unwrap_or(0),
        counts.get(&Bucket::Unmatched).copied().unwrap_or(0),
    ));
    text.push_str("| kind | 档 | 件数 | 代表组名 | 代表细目 |\n|---|---|---:|---|---|\n");
    for ((kind, bucket), (count, sample, detail)) in summary {
        text.push_str(&format!(
            "| {kind} | {} | {count} | {sample} | {detail} |\n",
            bucket.label()
        ));
    }
    let other: Vec<&Row> = rows
        .iter()
        .filter(|row| row.bucket != Bucket::Exact)
        .collect();
    if !other.is_empty() && other.len() <= 40 {
        text.push_str("\n| kind | 档 | 组名 | 细目 |\n|---|---|---|---|\n");
        for row in other {
            text.push_str(&format!(
                "| {} | {} | {} | {} |\n",
                row.kind,
                row.bucket.label(),
                row.member,
                row.detail
            ));
        }
    }
    text
}

fn params_of(primitive: &Primitive) -> Vec<f64> {
    match primitive {
        Primitive::Pyramid(v) => v.to_vec(),
        Primitive::Box(v) => v.to_vec(),
        Primitive::RectangularTorus(v) => v.to_vec(),
        Primitive::CircularTorus(v) => v.to_vec(),
        Primitive::EllipticalDish(v) => v.to_vec(),
        Primitive::SphericalDish(v) => v.to_vec(),
        Primitive::Snout(v) => v.to_vec(),
        Primitive::Cylinder(v) => v.to_vec(),
        Primitive::Sphere(v) => v.to_vec(),
        Primitive::Line(v) => v.to_vec(),
        Primitive::FacetGroup(_) => Vec::new(),
        Primitive::Unknown { params, .. } => params.clone(),
    }
}

/// 世界 AABB 中心（mm）。点集故意取少：两边用同一套点，中心仍能把同件配上，
/// 又避开 `local_hull_points_mm` 在球面上的上万个采样。
fn world_center(geometry: &RvmGeometry) -> Option<[f64; 3]> {
    let local = local_points(&geometry.primitive);
    let mut points = local.iter().map(|point| geometry.to_world_mm(*point));
    let first = points.next()?;
    let mut lo = first;
    let mut hi = first;
    for point in points {
        for axis in 0..3 {
            lo[axis] = lo[axis].min(point[axis]);
            hi[axis] = hi[axis].max(point[axis]);
        }
    }
    Some([
        (lo[0] + hi[0]) * 0.5,
        (lo[1] + hi[1]) * 0.5,
        (lo[2] + hi[2]) * 0.5,
    ])
}

fn local_points(primitive: &Primitive) -> Vec<[f64; 3]> {
    match primitive {
        Primitive::Box([x, y, z]) => corners(*x, *y, *z, 0.0, 0.0),
        Primitive::Cylinder([r, h]) => ring(*r, 0.0, 0.0, -h / 2.0)
            .into_iter()
            .chain(ring(*r, 0.0, 0.0, h / 2.0))
            .collect(),
        Primitive::Sphere([r]) => axial(*r),
        Primitive::Pyramid([bx, by, tx, ty, ox, oy, h]) => {
            corners(*bx, *by, 0.0, -ox / 2.0, -oy / 2.0)
                .into_iter()
                .map(|[x, y, _]| [x, y, -h / 2.0])
                .chain(
                    corners(*tx, *ty, 0.0, ox / 2.0, oy / 2.0)
                        .into_iter()
                        .map(|[x, y, _]| [x, y, h / 2.0]),
                )
                .collect()
        }
        Primitive::Snout([rb, rt, h, ox, oy, ..]) => ring(*rb, -ox / 2.0, -oy / 2.0, -h / 2.0)
            .into_iter()
            .chain(ring(*rt, ox / 2.0, oy / 2.0, h / 2.0))
            .collect(),
        Primitive::EllipticalDish([r, h]) | Primitive::SphericalDish([r, h]) => {
            ring(*r, 0.0, 0.0, 0.0)
                .into_iter()
                .chain(std::iter::once([0.0, 0.0, *h]))
                .collect()
        }
        Primitive::CircularTorus([offset, radius, angle]) => torus_points(*offset, *radius, *angle),
        Primitive::RectangularTorus([inner, outer, h, angle]) => {
            let mut points = Vec::new();
            for theta in [0.0, *angle] {
                for radius in [*inner, *outer] {
                    for z in [-h / 2.0, h / 2.0] {
                        points.push([radius * theta.cos(), radius * theta.sin(), z]);
                    }
                }
            }
            points
        }
        Primitive::FacetGroup(polygons) => polygons
            .iter()
            .flat_map(|polygon| polygon.contours.iter())
            .flat_map(|contour| contour.vertices.iter().copied())
            .collect(),
        Primitive::Line(_) | Primitive::Unknown { .. } => Vec::new(),
    }
}

fn corners(x: f64, y: f64, z: f64, cx: f64, cy: f64) -> Vec<[f64; 3]> {
    let mut points = Vec::with_capacity(8);
    for sx in [-0.5, 0.5] {
        for sy in [-0.5, 0.5] {
            for sz in [-0.5, 0.5] {
                points.push([cx + sx * x, cy + sy * y, sz * z]);
            }
        }
    }
    points
}

fn ring(radius: f64, cx: f64, cy: f64, z: f64) -> Vec<[f64; 3]> {
    (0..8)
        .map(|i| {
            let theta = std::f64::consts::TAU * i as f64 / 8.0;
            [cx + radius * theta.cos(), cy + radius * theta.sin(), z]
        })
        .collect()
}

fn axial(radius: f64) -> Vec<[f64; 3]> {
    vec![
        [radius, 0.0, 0.0],
        [-radius, 0.0, 0.0],
        [0.0, radius, 0.0],
        [0.0, -radius, 0.0],
        [0.0, 0.0, radius],
        [0.0, 0.0, -radius],
    ]
}

fn torus_points(offset: f64, radius: f64, angle: f64) -> Vec<[f64; 3]> {
    let mut points = Vec::new();
    for theta in [0.0, angle * 0.5, angle] {
        for (radial, z) in [(radius, 0.0), (-radius, 0.0), (0.0, radius), (0.0, -radius)] {
            let major = offset + radial;
            points.push([major * theta.cos(), major * theta.sin(), z]);
        }
    }
    points
}

fn col_close(a: [f64; 3], b: [f64; 3]) -> bool {
    (0..3).all(|i| (a[i] - b[i]).abs() <= ROT_TOL)
}

fn trans_close(a: [f64; 3], b: [f64; 3]) -> bool {
    (0..3).all(|i| (a[i] - b[i]).abs() <= TRANS_TOL_MM)
}

fn params_close(a: &[f64], b: &[f64]) -> bool {
    a.len() == b.len() && a.iter().zip(b).all(|(x, y)| (x - y).abs() <= PARAM_TOL)
}

fn neg(v: [f64; 3]) -> [f64; 3] {
    [-v[0], -v[1], -v[2]]
}

/// 世界盒心对盒心、原点对原点，以及盒心对原点。环面的记录原点是扫掠圆心，
/// 面片的盒心才落在实体上，只比同类点会把同一件看成离得很远。
fn pair_dist(left: &Record, right: &Record) -> f64 {
    dist3(left.center, right.center)
        .min(dist3(left.translation, right.translation))
        .min(dist3(left.center, right.translation))
        .min(dist3(left.translation, right.center))
}

fn dist3(a: [f64; 3], b: [f64; 3]) -> f64 {
    let d = [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn mark(ok: bool) -> &'static str {
    if ok { "=" } else { "≠" }
}

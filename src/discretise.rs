//! 带倒圆的环折线化（对标 libgm `GM_Profile` 的轮廓离散）。
//!
//! 输入是 PLOO/LOOP 的顶点序列，每个顶点 `(x, y, frad)`：`frad > 0` 表示该角
//! 用半径 `frad` 的圆弧倒掉。输出平面多边形（顶点序保持输入绕向）。
//!
//! 离散口径：轮廓弦高容差 [`PROFILE_CHORD_TOL_MM`]，段数走 [`circle_segments_uncapped`]
//! ——`d2_numberOfSegmentsForCircle` 的公式；弧内点再按 libgm
//! `D2_Span::getApproxPolyLineInSteps` 铺到 `k·2π/n` 的全局角度格子上。
//! 因此段数和相位都与 gen-model `libgm_discretise` 使用同一规则。
//!
//! 两条 E3D 语义必须照搬（gen-model RM13 与 manifold_tessellate 的实测结论）：
//!
//! 1. `FRAD == 半边长` 时倒角把两条直边整条吃光，正方形退化成正圆——切点恰落在
//!    边中点，公式自然成立，不需要特判。
//! 2. `mthArcFillet` 本身不裁剪。连续 span 链中只吸附两倍弦高容差内的近越界；
//!    若原始 PAVE 链含 Core3D 会跳过的退化线段（相邻点距不超过 0.1mm），轮廓
//!    组装器会把**两端都越界**的弧重新接到相邻 PAVE，但仍保留原圆心、半径和
//!    弧内相位点；单端越界仍保留原弧。
//!    两条路径都在 [`FlattenNotes::overshoot_fillets`] 记账。

/// 曲面基本体的弦高容差（mm），绝对量。
///
/// 对齐 libgm `GM_User::arctol_`：Core3D 主初始化处 `gm_SetDefaultFacetTolerance(0.5)`。
/// 它是绝对量而非按构件尺度的比例量——比例量会让同一个半径在正体与负体上分成不同
/// 段数，差集在共壁处留一层皮（gen-model RM13 穹顶的实测教训）。
pub const CHORD_TOL_MM: f64 = 0.5;
/// RVM/GM_Extrusion 的二维轮廓折线化容差。它与曲面基本体的 facet tolerance
/// 不是同一个入口：AMS1112 RVM 的半径 16725.256165mm 圆弧实测为 92 段整圆格，
/// 反算弦高 9.75mm，对应调用侧传入 10mm。
pub const PROFILE_CHORD_TOL_MM: f64 = 10.0;
/// 低于该长度（mm）的边视为退化。
const EDGE_EPS: f64 = 1e-9;
/// Core3D `sub_106200B0` 添加直线 span 前的端点距离门槛。
const PROFILE_LINK_TOL_MM: f64 = 0.1;
/// libgeom 判断角度格点是否严格落在弧内时使用的容差（弧度）。
const SPAN_EPS: f64 = 0.0000306;

/// 环顶点：xy 坐标 + 倒圆半径。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FilletVertex {
    pub x: f64,
    pub y: f64,
    pub frad: f64,
}

/// 折线化的诊断。留名，不静默。
#[derive(Debug, Default, Clone, PartialEq)]
pub struct FlattenNotes {
    /// 切点越过邻边长度的角（顶点下标）。E3D 语义是不裁照铺，这里只记账。
    pub overshoot_fillets: Vec<usize>,
    /// 三点共线或边退化、倒圆被忽略的角。
    pub degenerate_corners: Vec<usize>,
}

/// 原始 PAVE 环是否含 Core3D `sub_106200B0` 不会发出的退化 line span。
pub fn has_core3d_skipped_link(ring: &[FilletVertex]) -> bool {
    let n = ring.len();
    n >= 2
        && (0..n).any(|i| {
            let a = ring[i];
            let b = ring[(i + 1) % n];
            (a.x - b.x).hypot(a.y - b.y) <= PROFILE_LINK_TOL_MM
        })
}

/// 把带倒圆的环折线化成平面多边形。
///
/// 顶点不足 3 给 `None`。输出**不含**重复首点。
pub fn flatten_ring(ring: &[FilletVertex]) -> Option<(Vec<[f64; 2]>, FlattenNotes)> {
    if ring.len() < 3 {
        return None;
    }
    let n = ring.len();
    let mut out: Vec<[f64; 2]> = Vec::with_capacity(n * 4);
    let mut notes = FlattenNotes::default();
    // Core3D 在相邻端点距离 <= 0.1mm 时不发该直线 span。这样的原始链由后续
    // 轮廓组装重新接续；AMS1112 WF05 两个大 FRAD FLOOR 正是这条路径。
    let has_skipped_link = has_core3d_skipped_link(ring);

    for i in 0..n {
        let p = ring[i];
        let requested_radius = p.frad.abs();
        if requested_radius <= EDGE_EPS {
            out.push([p.x, p.y]);
            continue;
        }
        let prev = ring[(i + n - 1) % n];
        let next = ring[(i + 1) % n];
        let corner = [p.x, p.y];
        // 从角点出发指向两邻点的单位向量。
        let to_prev = [prev.x - p.x, prev.y - p.y];
        let to_next = [next.x - p.x, next.y - p.y];
        let len_prev = (to_prev[0] * to_prev[0] + to_prev[1] * to_prev[1]).sqrt();
        let len_next = (to_next[0] * to_next[0] + to_next[1] * to_next[1]).sqrt();
        if len_prev <= EDGE_EPS || len_next <= EDGE_EPS {
            notes.degenerate_corners.push(i);
            out.push(corner);
            continue;
        }
        let u = [to_prev[0] / len_prev, to_prev[1] / len_prev];
        let w = [to_next[0] / len_next, to_next[1] / len_next];
        let cos2a = (u[0] * w[0] + u[1] * w[1]).clamp(-1.0, 1.0);
        // 2α 是两边夹角；α∈(0, π/2)。共线（夹角 0 或 π）没有唯一的倒圆。
        let two_alpha = cos2a.acos();
        if two_alpha <= 1e-9 || (std::f64::consts::PI - two_alpha) <= 1e-9 {
            notes.degenerate_corners.push(i);
            out.push(corner);
            continue;
        }
        let alpha = two_alpha / 2.0;
        // `mthArcFillet` 本身不限。普通连续链只吸附近越界；若本环含 Core3D
        // 会跳过的退化 line span，组装层则重接两端都越界的弧，但不改圆心/半径。
        let tan_alpha = alpha.tan();
        let requested_t = requested_radius / tan_alpha;
        let overshoots_prev = requested_t > len_prev;
        let overshoots_next = requested_t > len_next;
        let reconnect_both_ends = has_skipped_link && overshoots_prev && overshoots_next;
        if overshoots_prev || overshoots_next {
            notes.overshoot_fillets.push(i);
        }
        let min_edge = len_prev.min(len_next);
        let radius = if !reconnect_both_ends
            && requested_t > min_edge
            && requested_t - min_edge <= 2.0 * CHORD_TOL_MM
        {
            min_edge * tan_alpha
        } else {
            requested_radius
        };
        // 切点距角点 t = r / tan(α)；圆心沿角平分线距角点 d = r / sin(α)。
        let t = radius / tan_alpha;
        let bis = {
            let s = [u[0] + w[0], u[1] + w[1]];
            let l = (s[0] * s[0] + s[1] * s[1]).sqrt();
            if l <= EDGE_EPS {
                notes.degenerate_corners.push(i);
                out.push(corner);
                continue;
            }
            [s[0] / l, s[1] / l]
        };
        let dist = radius / alpha.sin();
        let center = [corner[0] + bis[0] * dist, corner[1] + bis[1] * dist];
        let start = if reconnect_both_ends {
            [prev.x, prev.y]
        } else {
            [corner[0] + u[0] * t, corner[1] + u[1] * t]
        };
        let end = if reconnect_both_ends {
            [next.x, next.y]
        } else {
            [corner[0] + w[0] * t, corner[1] + w[1] * t]
        };

        // 从 start 到 end 的圆弧（不过角点一侧）。夹角 = π − 2α < π，
        // 所以「转角绝对值最小的方向」就是要的那段弧。
        let a0 = (start[1] - center[1]).atan2(start[0] - center[0]);
        let a1 = (end[1] - center[1]).atan2(end[0] - center[0]);
        let mut delta = a1 - a0;
        while delta > std::f64::consts::PI {
            delta -= 2.0 * std::f64::consts::PI;
        }
        while delta < -std::f64::consts::PI {
            delta += 2.0 * std::f64::consts::PI;
        }
        let segments = circle_segments_uncapped(radius, PROFILE_CHORD_TOL_MM);
        out.extend(phase_locked_arc_points(
            start, end, center, radius, a0, delta, segments,
        ));
    }

    // 相邻重复点焊掉（倒角吃光整条边时切点会与邻角切点重合）。
    let mut welded: Vec<[f64; 2]> = Vec::with_capacity(out.len());
    for p in out {
        if let Some(last) = welded.last()
            && (last[0] - p[0]).abs() <= 1e-6
            && (last[1] - p[1]).abs() <= 1e-6
        {
            continue;
        }
        welded.push(p);
    }
    while welded.len() >= 2 {
        let first = welded[0];
        let last = *welded.last().unwrap();
        if (first[0] - last[0]).abs() <= 1e-6 && (first[1] - last[1]).abs() <= 1e-6 {
            welded.pop();
        } else {
            break;
        }
    }
    if welded.len() < 3 {
        return None;
    }
    Some((welded, notes))
}

/// `d2_numberOfSegmentsForCircle(radius, tol)`（libgeom 3.1 `0x1002BA70`）：整圆分几段。
///
/// ```text
/// step = 2·acos(1 − |tol/R|)   弦高 R(1 − cos(step/2)) ≤ tol 的最大圆心角
/// step 封顶 45°                ⇒ 整圆最少 8 段
/// n = ceil(360/step)，再向上取到 4 的倍数
/// ```
///
/// 「4 的倍数」不是凑整，是让 0/90/180/270 四个象限点落在网格上；少了它段数会与
/// E3D 差 1~3 段。本函数**不封顶**：libgeom 的公式自己没有上限，上限是各
/// `GM_*::calcFacets` 各自加的（曲面原语 1000，见 [`crate::solid::cylinder_segments`]），
/// 而挤出截面那条路（`GM_Extrusion::calcFacets` → `D2_Span::getApproxPolyLine`）
/// 压根不封顶——倒角弧走的就是这一支。
pub fn circle_segments_uncapped(radius: f64, chord_tol: f64) -> i32 {
    if !(radius > 0.0) {
        return 1;
    }
    let x = (1.0 - (chord_tol / radius).abs()).max(0.0);
    let mut step_deg = x.acos().to_degrees() * 2.0;
    if !(step_deg > 0.0) || step_deg > 45.0 {
        step_deg = 45.0;
    }
    let n = (360.0 / step_deg).ceil() as i32;
    (n + 3) & !3
}

/// 按整圆段数建立 `k·2π/n` 全局相位格，只保留严格落在弧内的格点，并保留真实端点。
/// 这是截面弧的 `D2_Span::getApproxPolyLineInSteps` 口径，不是回转原语的均分扫角口径。
fn phase_locked_arc_points(
    start: [f64; 2],
    end: [f64; 2],
    center: [f64; 2],
    radius: f64,
    a0: f64,
    delta: f64,
    segments: i32,
) -> Vec<[f64; 2]> {
    let a1 = a0 + delta;
    let n = segments.max(1);
    let step = std::f64::consts::TAU / f64::from(n);
    let mut points = Vec::with_capacity(n as usize + 2);
    points.push(start);

    // 与 libgm `span_polyline_in_steps` 保持同一趟 angle → t → theta 浮点运算。
    let point_at = |angle: f64| {
        let t = (angle - a0) / delta;
        let theta = a0 + t * delta;
        [
            center[0] + radius * theta.cos(),
            center[1] + radius * theta.sin(),
        ]
    };
    if delta > 0.0 {
        for k in (1 - n)..n {
            let angle = f64::from(k) * step;
            if angle > a0 + SPAN_EPS && angle < a1 - SPAN_EPS {
                points.push(point_at(angle));
            }
        }
    } else {
        for k in ((1 - n)..n).rev() {
            let angle = f64::from(k) * step;
            if angle < a0 - SPAN_EPS && angle > a1 + SPAN_EPS {
                points.push(point_at(angle));
            }
        }
    }
    points.push(end);
    points
}

/// 有符号面积（鞋带公式）。正 = 逆时针。
pub fn signed_area(poly: &[[f64; 2]]) -> f64 {
    let n = poly.len();
    let mut acc = 0.0;
    for i in 0..n {
        let a = poly[i];
        let b = poly[(i + 1) % n];
        acc += a[0] * b[1] - b[0] * a[1];
    }
    acc / 2.0
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v(x: f64, y: f64) -> FilletVertex {
        FilletVertex { x, y, frad: 0.0 }
    }

    #[test]
    fn square_without_fillets_passes_through() {
        let ring = [v(0.0, 0.0), v(100.0, 0.0), v(100.0, 100.0), v(0.0, 100.0)];
        let (poly, notes) = flatten_ring(&ring).unwrap();
        assert_eq!(poly.len(), 4);
        assert!(notes.overshoot_fillets.is_empty());
        assert!((signed_area(&poly) - 10_000.0).abs() < 1e-9);
    }

    #[test]
    fn fillet_arc_interior_points_use_the_full_circle_phase_grid() {
        let a0: f64 = 0.1;
        let delta: f64 = 1.8;
        let start = [a0.cos(), a0.sin()];
        let end = [(a0 + delta).cos(), (a0 + delta).sin()];
        let points = phase_locked_arc_points(start, end, [0.0, 0.0], 1.0, a0, delta, 8);

        assert_eq!(points.first(), Some(&start));
        assert_eq!(points.last(), Some(&end));
        let angles: Vec<f64> = points[1..points.len() - 1]
            .iter()
            .map(|p| p[1].atan2(p[0]))
            .collect();
        assert_eq!(angles.len(), 2);
        assert!((angles[0] - std::f64::consts::FRAC_PI_4).abs() < 1e-12);
        assert!((angles[1] - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
    }

    #[test]
    fn one_fillet_rounds_the_corner_and_shrinks_area_by_the_corner_bite() {
        // 100×100 正方形，右下角 r=10 倒圆：面积减 (r² − πr²/4)。
        let ring = [
            v(0.0, 0.0),
            FilletVertex {
                x: 100.0,
                y: 0.0,
                frad: 10.0,
            },
            v(100.0, 100.0),
            v(0.0, 100.0),
        ];
        let (poly, notes) = flatten_ring(&ring).unwrap();
        assert!(notes.overshoot_fillets.is_empty());
        assert!(poly.len() > 5, "弧必须被离散出点来，实得 {}", poly.len());
        let exact = 10_000.0 - (100.0 - std::f64::consts::PI * 25.0);
        let got = signed_area(&poly);
        assert!(
            (got - exact).abs() / exact < 1e-3,
            "面积 {got}，期望 {exact}"
        );
    }

    /// RM13 穹顶的把戏：正方形四角 FRAD = 半边长 → 退化成整圆。
    #[test]
    fn fillets_eating_every_edge_turn_a_square_into_a_circle() {
        const R: f64 = 23_400.0;
        let ring = [
            FilletVertex {
                x: -R,
                y: -R,
                frad: R,
            },
            FilletVertex {
                x: -R,
                y: R,
                frad: R,
            },
            FilletVertex {
                x: R,
                y: R,
                frad: R,
            },
            FilletVertex {
                x: R,
                y: -R,
                frad: R,
            },
        ];
        let (poly, notes) = flatten_ring(&ring).unwrap();
        assert!(notes.overshoot_fillets.is_empty(), "{notes:?}");
        let exact = std::f64::consts::PI * R * R;
        let got = signed_area(&poly).abs();
        assert!(
            (got - exact).abs() / exact < 2e-3,
            "圆面积 {got}，期望 {exact}（相对差 {}）",
            ((got - exact) / exact).abs()
        );
        // 所有点都应落在半径 R 的圆上（切点在边中点，圆心在原点）。
        for p in &poly {
            let r = (p[0] * p[0] + p[1] * p[1]).sqrt();
            assert!((r - R).abs() < R * 1e-6, "点 {p:?} 半径 {r}");
        }
    }

    #[test]
    fn significantly_oversized_fillet_is_kept_unclipped_and_reported() {
        // 直角 r=150、边长 100：切长 t = r/tan(45°) = 150 越过两条邻边。
        // 越界 50mm 远大于 2×facet_tol，保留自交，只记账。
        let ring = [
            v(0.0, 0.0),
            FilletVertex {
                x: 100.0,
                y: 0.0,
                frad: 150.0,
            },
            v(100.0, 100.0),
            v(0.0, 100.0),
        ];
        let (poly, notes) = flatten_ring(&ring).unwrap();
        assert_eq!(notes.overshoot_fillets, vec![1]);
        // 起切点 = 角点 (100,0) 沿边往回 150 → (-50, 0)，必须原样保留。
        let has_start = poly
            .iter()
            .any(|p| (p[0] + 50.0).abs() < 1e-6 && p[1].abs() < 1e-6);
        assert!(has_start, "显著越界倒角的切点必须保留");
    }

    #[test]
    fn near_overshoot_is_snapped_to_the_adjacent_edge() {
        let ring = [
            v(0.0, 0.0),
            FilletVertex {
                x: 100.0,
                y: 0.0,
                frad: 100.5,
            },
            v(100.0, 100.0),
            v(0.0, 100.0),
        ];
        let (poly, notes) = flatten_ring(&ring).unwrap();
        assert_eq!(notes.overshoot_fillets, vec![1]);
        assert!(poly.iter().all(|p| p[0] >= -1e-6 && p[1] >= -1e-6));
    }

    #[test]
    fn skipped_link_reconnects_oversized_fillet_without_changing_its_circle() {
        // AMS1112 FLOOR 17496/79648 的最小复刻：重复 PAVE 令 Core3D 跳过一条
        // 退化 line span；大 FRAD 的两端都越过邻点。E3D 把端点接回上下边界，
        // 但保留圆弧内部的 +X 极值（若裁半径，该极值会明显内缩）。
        let ring = [
            v(-1840.0, 30.0),
            v(-1840.0, 1130.0),
            v(-1840.0, 6080.0),
            v(-1340.0, 6080.0),
            v(-1340.0, 9990.0),
            v(-1340.0, 9990.0),
            v(10.0, 9990.0),
            v(401.554687, 9989.988281),
            FilletVertex {
                x: 1609.007812,
                y: 5019.327148,
                frad: 23300.0,
            },
            v(627.72, 30.0),
        ];
        let (poly, notes) = flatten_ring(&ring).unwrap();
        assert_eq!(notes.overshoot_fillets, vec![8]);
        let min_y = poly.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min);
        let max_y = poly.iter().map(|p| p[1]).fold(f64::NEG_INFINITY, f64::max);
        let max_x = poly.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max);
        assert!((min_y - 30.0).abs() < 1e-9, "min_y={min_y}");
        assert!((max_y - 9990.0).abs() < 1e-9, "max_y={max_y}");
        assert!(
            max_x > 1050.0,
            "原半径圆弧的 +X 极值必须保留，max_x={max_x}"
        );
    }
}

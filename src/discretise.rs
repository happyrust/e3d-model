//! 带倒圆的环折线化（对标 libgm `GM_Profile` 的轮廓离散）。
//!
//! 输入是 PLOO/LOOP 的顶点序列，每个顶点 `(x, y, frad)`：`frad > 0` 表示该角用
//! 半径 `frad` 的相切**凸**圆角倒掉，`frad < 0` 是以角点为心、半径 `|frad|` 的
//! **凹**圆角（见下第 4 条）。输出平面多边形（顶点序保持输入绕向）。
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
//!    弧内相位点；单端越界仍保留原弧。重接只换 span 的两个端点，相位点仍铺在理论
//!    切点之间的整段弧上：ams7997 NXTR `24381/42463` 的 `FRAD 31441` 切点远在 90 km 外，
//!    按「两个 PAVE 之间的角度」只剩一个格点，它切的 FN09 FLOOR 4 会在邻近负体交界处
//!    剩一块 10×4 mm 的碎片，E3D TTY RVM 里没有。
//!    两条路径都在 [`FlattenNotes::overshoot_fillets`] 记账。
//! 3. 若一个孤立**凸**倒圆的两个理论切点都只给相邻直线留下 `<= 0.1mm`，
//!    Core3D 仍保留按理论切点构造的原圆弧；不能把整段弧当成 consumed 删除，也不能
//!    把端点强吸到相邻 PAVE。凹角和真正越界仍走第 2 条。
//! 4. 负 FRAD 走 `mth::mthArcRadius`（libgeom 3.1 `0x10042ba0`）而非 `mthArcFillet`：
//!    Core3D `MDR_Point` 倒角方法（3.1 `sub_10621980`）按 FRAD 符号分流，负支把角点
//!    沿两条单位邻边向量各退 `|FRAD|` 得弧的两个端点、取劣弧，圆心恰落在**角点**上
//!    ——凹圆角，与正 FRAD 同一张弦、反向鼓。门槛照同一处反编译：`fabs(FRAD) < 0.1`
//!    与近共线（单位邻边叉积 `sin 2α ≤ 0.1`）不倒角。**这两道门只加在负支**：正支的
//!    微小半径与近共线行为是 RVM 对拍钉住的既有口径，原样不动（见 `NEG_FRAD_EPS`）。
//!    取证：gen-model `docs/evidence/2026-09-09-ida-negative-frad-mtharcradius.md`；
//!    真实负 FRAD 只在 ZDJ（甲板板 PANE −25、支座 SBFR −5），AVEVA 样例库全零。

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
/// Core3D `MDR_Point` 倒角方法（3.1 `sub_10621980`）开头的
/// `if (fabs(radius) < 0.1) return 0`：`|FRAD|` 不足 0.1mm 不倒角。**只用于负 FRAD
/// 支**——正支维持本文件既有的 `EDGE_EPS` 门（微小正 FRAD 照发亚 0.1mm 小弧），那是
/// RVM 对拍钉住的行为，语料 FRAD 直方图里「近零」一档又占绝大多数，动它是另一个决定。
const NEG_FRAD_EPS: f64 = 0.1;
/// 同一方法在符号分流前的 `D3_Vector::equals(cross(d0, d1), 0, 0.1, 0.1)`：单位邻边
/// 叉积（= `sin 2α`）不足它就不倒角——夹角离 0°/180° 不到约 5.74° 的角全挡掉。
/// 同样只用于负支，理由同上。
const NEG_CROSS_EPS: f64 = 0.1;

/// 环顶点：xy 坐标 + 倒圆半径（正 = 相切凸圆角，负 = 以角点为心的凹圆角，
/// 见模块文档第 4 条）。
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
    /// 两侧剩余直线都不超过 Core3D 门槛、但仍按理论切点保留的孤立倒圆。
    pub consumed_fillets: Vec<usize>,
    /// 与下一个角的倒圆在共用边上撞上、已按两条弦的交点接起来的角
    /// （见 [`trim_overlapping_fillets`]）。
    pub merged_fillets: Vec<usize>,
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
    let raw_area = (0..n)
        .map(|i| {
            let a = ring[i];
            let b = ring[(i + 1) % n];
            a.x * b.y - b.x * a.y
        })
        .sum::<f64>()
        * 0.5;
    // 逐角分开收：相邻两个倒圆在同一条边上吃过头时要按弧与弧的交点拼接，
    // 那一步得知道哪几个点属于哪个角（见 [`trim_overlapping_fillets`]）。
    let mut arcs: Vec<Vec<[f64; 2]>> = vec![Vec::new(); n];
    // 每个角的切点离角点多远（不倒圆的是 0）。判断两个倒圆有没有在共用边上撞上就看它。
    let mut tangent_of: Vec<f64> = vec![0.0; n];
    let mut notes = FlattenNotes::default();
    // Core3D 在相邻端点距离 <= 0.1mm 时不发该直线 span。这样的原始链由后续
    // 轮廓组装重新接续；AMS1112 WF05 两个大 FRAD FLOOR 正是这条路径。
    let has_skipped_link = has_core3d_skipped_link(ring);

    for i in 0..n {
        let out = &mut arcs[i];
        let p = ring[i];
        let requested_radius = p.frad.abs();
        if requested_radius <= EDGE_EPS {
            out.push([p.x, p.y]);
            continue;
        }
        // 负支的 `fabs(radius) < 0.1` 门（`NEG_FRAD_EPS`）：不足门槛原样留尖角。
        if p.frad < 0.0 && requested_radius < NEG_FRAD_EPS {
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
        if p.frad < 0.0 {
            // 负 FRAD 走 `mth::mthArcRadius`（模块文档第 4 条）：角点沿两条单位邻边
            // 向量各退 `|FRAD|` 得弧的两个端点，弧以**角点为心**、半径 `|FRAD|`、取
            // 劣弧（扫角 = 2α < π，归一到 (−π, π] 的最小转角就是它）——凹圆角，与正
            // FRAD 同一张弦、反向鼓。
            if two_alpha.sin() <= NEG_CROSS_EPS {
                // Core3D 分流前的 cross 门：近共线不倒角。少这道门，一个几乎拉直的
                // 角会被展开成接近半圈的大凹弧，凭空挖走一大块材料。
                notes.degenerate_corners.push(i);
                out.push(corner);
                continue;
            }
            // 退距恒为 `|FRAD|`（不是正支的 R/tanα）。`mthArcRadius` 不裁不吸附，
            // 端点越过邻点只记账——语料里的负 FRAD（ZDJ 甲板板 −25、支座 −5）都远
            // 小于邻边，越界支路没有实测样例，不套用正支那套近越界吸附。
            if requested_radius > len_prev || requested_radius > len_next {
                notes.overshoot_fillets.push(i);
            }
            let start = [
                corner[0] + u[0] * requested_radius,
                corner[1] + u[1] * requested_radius,
            ];
            let end = [
                corner[0] + w[0] * requested_radius,
                corner[1] + w[1] * requested_radius,
            ];
            let a0 = (start[1] - corner[1]).atan2(start[0] - corner[0]);
            let a1 = (end[1] - corner[1]).atan2(end[0] - corner[0]);
            let mut delta = a1 - a0;
            while delta > std::f64::consts::PI {
                delta -= 2.0 * std::f64::consts::PI;
            }
            while delta < -std::f64::consts::PI {
                delta += 2.0 * std::f64::consts::PI;
            }
            let segments = circle_segments_uncapped(requested_radius, PROFILE_CHORD_TOL_MM);
            out.extend(phase_locked_arc_points(
                start,
                end,
                corner,
                requested_radius,
                a0,
                delta,
                segments,
            ));
            continue;
        }
        let alpha = two_alpha / 2.0;
        // `mthArcFillet` 本身不限。普通连续链只吸附近越界；若本环含 Core3D
        // 会跳过的退化 line span，组装层则重接两端都越界的弧，但不改圆心/半径。
        let tan_alpha = alpha.tan();
        let requested_t = requested_radius / tan_alpha;
        let incoming = [p.x - prev.x, p.y - prev.y];
        let outgoing = [next.x - p.x, next.y - p.y];
        let turn = incoming[0] * outgoing[1] - incoming[1] * outgoing[0];
        let convex = turn * raw_area > 0.0;
        let consumes_prev = (len_prev - requested_t).abs() <= PROFILE_LINK_TOL_MM;
        let consumes_next = (len_next - requested_t).abs() <= PROFILE_LINK_TOL_MM;
        let reconnect_consumed = prev.frad.abs() <= EDGE_EPS
            && next.frad.abs() <= EDGE_EPS
            && convex
            && requested_t <= len_prev
            && requested_t <= len_next
            && consumes_prev
            && consumes_next;
        if reconnect_consumed {
            notes.consumed_fillets.push(i);
        }
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
        let tangent_start = [corner[0] + u[0] * t, corner[1] + u[1] * t];
        let tangent_end = [corner[0] + w[0] * t, corner[1] + w[1] * t];
        let start = if reconnect_both_ends {
            [prev.x, prev.y]
        } else {
            tangent_start
        };
        let end = if reconnect_both_ends {
            [next.x, next.y]
        } else {
            tangent_end
        };

        // 弧的角度范围始终是理论切点之间那段（不过角点一侧，夹角 = π − 2α < π，
        // 「转角绝对值最小的方向」就是它）。重接只换端点，不换角度范围。
        let a0 = (tangent_start[1] - center[1]).atan2(tangent_start[0] - center[0]);
        let a1 = (tangent_end[1] - center[1]).atan2(tangent_end[0] - center[0]);
        let mut delta = a1 - a0;
        while delta > std::f64::consts::PI {
            delta -= 2.0 * std::f64::consts::PI;
        }
        while delta < -std::f64::consts::PI {
            delta += 2.0 * std::f64::consts::PI;
        }
        let segments = circle_segments_uncapped(radius, PROFILE_CHORD_TOL_MM);
        tangent_of[i] = t;
        out.extend(phase_locked_arc_points(
            start, end, center, radius, a0, delta, segments,
        ));
    }

    trim_overlapping_fillets(ring, &tangent_of, &mut arcs, &mut notes);
    let out: Vec<[f64; 2]> = arcs.into_iter().flatten().collect();

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

/// 共用一条边的两个倒圆吃过了头时，把两条折线在**交点**处接起来。
///
/// 一条边上两个角各吃掉 `t` 长，`t_i + t_j` 超过边长时两个切点就换了位置：这条边的
/// 直线段方向反过来，轮廓折回去，多出一个绕向相反的小耳朵。Core3D 不在这里裁——
/// `sub_106200B0` 只在两点相距 `<= 0.1mm` 时跳过那条直线 span，反向照发；耳朵是后面
/// 铺面片时按填充规则消掉的。落到我们这条路上，网格器把它当成两块独立实体留了下来。
///
/// 取证（SLYK `/Copy-of-立柱` 的 FLOOR，3500×16000、四角 `FRAD=2000`，短边上两个倒圆
/// 重叠 500mm）：E3D 自己导出的 RVM 顶点串在两条弧之间只有一个点 `(-1750, 24.62)`，
/// 正是两条**折线**（不是两个圆）末尾那对弦的交点——按 11.25° 相位格算出来是
/// `(-1750, 24.622)`。我们没裁，于是在 x∈[-2000,-1500] 上多出两块 500×24.62 的小舌头，
/// 两端各把 AABB 顶出 24.625mm。
///
/// 交点取「从 i 的尾巴往回、从 j 的头往前」第一对真相交的弦；找不到就原样留着并记账，
/// 不猜。
fn trim_overlapping_fillets(
    ring: &[FilletVertex],
    tangent_of: &[f64],
    arcs: &mut [Vec<[f64; 2]>],
    notes: &mut FlattenNotes,
) {
    let n = ring.len();
    for i in 0..n {
        let j = (i + 1) % n;
        if tangent_of[i] <= 0.0 || tangent_of[j] <= 0.0 {
            continue;
        }
        let edge = (ring[j].x - ring[i].x).hypot(ring[j].y - ring[i].y);
        if tangent_of[i] + tangent_of[j] <= edge + EDGE_EPS {
            continue;
        }
        let Some((cut_i, cut_j, crossing)) = first_chord_crossing(&arcs[i], &arcs[j]) else {
            notes.overshoot_fillets.push(i);
            continue;
        };
        arcs[i].truncate(cut_i + 1);
        arcs[i].push(crossing);
        arcs[j].drain(..=cut_j);
        notes.merged_fillets.push(i);
    }
}

/// 两条折线首尾相撞的那一对弦：`a` 从尾巴往回数，`b` 从头往前数，取先撞上的。
///
/// 返回 `(a 上保留到的点下标, b 上丢弃到的点下标, 交点)`。
fn first_chord_crossing(a: &[[f64; 2]], b: &[[f64; 2]]) -> Option<(usize, usize, [f64; 2])> {
    if a.len() < 2 || b.len() < 2 {
        return None;
    }
    // 先按「离接缝多远」排，越靠近接缝的那一对越先试——重叠只发生在接缝附近。
    for depth in 0..(a.len() - 1) + (b.len() - 1) {
        for back in 0..=depth {
            let forward = depth - back;
            if back >= a.len() - 1 || forward >= b.len() - 1 {
                continue;
            }
            let ai = a.len() - 2 - back;
            if let Some(point) = segment_crossing(a[ai], a[ai + 1], b[forward], b[forward + 1]) {
                return Some((ai, forward, point));
            }
        }
    }
    None
}

/// 两条线段的真交点（含端点，不含平行/共线）。
fn segment_crossing(p0: [f64; 2], p1: [f64; 2], q0: [f64; 2], q1: [f64; 2]) -> Option<[f64; 2]> {
    let r = [p1[0] - p0[0], p1[1] - p0[1]];
    let s = [q1[0] - q0[0], q1[1] - q0[1]];
    let denom = r[0] * s[1] - r[1] * s[0];
    if denom.abs() <= EDGE_EPS {
        return None;
    }
    let d = [q0[0] - p0[0], q0[1] - p0[1]];
    let t = (d[0] * s[1] - d[1] * s[0]) / denom;
    let u = (d[0] * r[1] - d[1] * r[0]) / denom;
    (0.0..=1.0).contains(&t).then(|| ()).and_then(|()| {
        (0.0..=1.0)
            .contains(&u)
            .then_some([p0[0] + r[0] * t, p0[1] + r[1] * t])
    })
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
/// 整圆最少分成几段。45° 的步长封顶（见 [`circle_segments_uncapped`]）决定了这个数，
/// 与容差、半径都无关——`cylinder_segments(1.0)` 撞的就是它。
///
/// 反过来用是它的另一半价值：**端盖少于 8 个顶点的网格不可能是离散出来的圆**，
/// 那是一个真多边形。`rvm_write` 的圆柱 / 圆台识别拿它当下限，见
/// [`crate::rvm_write::MIN_CIRCLE_SEGMENTS`]。
pub const MIN_CIRCLE_SEGMENTS: i32 = 8;

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

    /// SLYK `/Copy-of-立柱` 的 FLOOR：3500×16000 的板，四角 `FRAD=2000`，短边只有
    /// 3500——两个倒圆在它上面重叠 500mm。两条折线要在交点处接起来，不能各自把切点
    /// 铺到对面去，否则短边上会多出一个绕向相反的小耳朵，AABB 两端各被顶出 24.625mm。
    ///
    /// `(-1750, 24.62)` 是 E3D 自己导出的 RVM 里那一段的唯一顶点（`output/slyk-rvm`
    /// 的 21491-11385 基准），按 11.25° 相位格算出来是 24.622。
    #[test]
    fn overlapping_fillets_on_a_short_edge_meet_at_the_chord_crossing() {
        let r = |x: f64, y: f64| FilletVertex { x, y, frad: 2000.0 };
        let ring = [
            r(0.0, 0.0),
            r(0.0, 16000.0),
            r(-3500.0, 16000.0),
            r(-3500.0, 0.0),
        ];
        let (poly, notes) = flatten_ring(&ring).unwrap();
        assert_eq!(notes.merged_fillets.len(), 2, "两条短边各接一次");

        // 板不许越过原始矩形，也不许在短边上留下折回去的耳朵。
        let y_min = poly.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min);
        let y_max = poly.iter().map(|p| p[1]).fold(f64::NEG_INFINITY, f64::max);
        assert!(
            (y_min - 24.622).abs() < 0.01,
            "短边最低点应落在两条弦的交点上，得到 {y_min}"
        );
        assert!(
            (y_max - (16000.0 - 24.622)).abs() < 0.01,
            "另一条短边同理，得到 {y_max}"
        );
        // 长边方向不受影响：切点 2000 远小于 16000。
        let x_min = poly.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min);
        let x_max = poly.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max);
        assert!((x_min + 3500.0).abs() < 1e-6, "{x_min}");
        assert!(x_max.abs() < 1e-6, "{x_max}");
        // 交点两侧的那两个格点也要在，它们是 E3D 顶点串里紧挨着的两个。
        for x in [-1890.2, -1609.8] {
            assert!(
                poly.iter()
                    .any(|p| (p[0] - x).abs() < 0.1 && (p[1] - 38.43).abs() < 0.1),
                "缺 E3D 顶点 ({x}, 38.43)"
            );
        }
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

    #[test]
    fn reconnected_fillet_keeps_the_phase_points_between_its_theoretical_tangents() {
        // ams7997 NXTR 24381/42463（FN09 FLOOR 4 名下）的环：V2/V3 重复、V9 = V1，
        // V4 的 FRAD 31441 切点距角点 90.5 km。重接后的弧从 V2 出发、绕整段理论弧回到 V5，
        // 最后一个相位点紧挨理论切点，进 V5 的那条边与 V4→V5 几乎共线。
        let ring = [
            v(0.0, 0.0),
            v(0.0, -3300.0),
            v(0.0, -3300.0),
            FilletVertex {
                x: -1836.77,
                y: -200.0,
                frad: 31441.04,
            },
            v(-2267.32, -3389.94),
            v(-2554.84, -103.49),
            v(-1504.03, -35.0),
            FilletVertex {
                x: -461.47,
                y: -100.0,
                frad: 33469.93,
            },
            v(0.0, 0.0),
        ];
        let (poly, notes) = flatten_ring(&ring).unwrap();
        assert!(notes.overshoot_fillets.contains(&3), "{notes:?}");
        let v4 = [-1836.77, -200.0];
        let v5 = [-2267.32, -3389.94];
        let at = poly
            .iter()
            .position(|p| (p[0] - v5[0]).abs() < 1e-9 && (p[1] - v5[1]).abs() < 1e-9)
            .expect("V5 在折线里");
        let before = poly[at - 1];
        let reach = (before[0] - v5[0]).hypot(before[1] - v5[1]);
        assert!(reach > 50_000.0, "进 V5 的点应在理论切点附近：{before:?}");
        let edge = (v5[0] - v4[0]).hypot(v5[1] - v4[1]);
        let along = ((v5[0] - v4[0]) * (before[0] - v5[0]) + (v5[1] - v4[1]) * (before[1] - v5[1]))
            / (edge * reach);
        assert!(
            along > 0.999_999,
            "进 V5 的边应沿 V4→V5 的延长线，cos = {along}"
        );
        assert!(
            poly.len() > 40,
            "141.7° 的理论弧应铺满相位点：{} 个点",
            poly.len()
        );
    }

    #[test]
    fn isolated_fillet_that_consumes_both_line_spans_keeps_theoretical_arc() {
        // AMS1112 FLOOR 17496/100202 -> NXTR 17496/100327。FRAD 的两个理论切点
        // 距相邻 PAVE 都不到 0.001mm；Core3D 的 0.1mm line-span 门禁不发这两段
        // 短线，但 RVM 仍保留原半径圆弧和理论切点。
        let ring = [
            v(0.0, 0.0),
            v(0.0, 986.872314),
            v(0.0, 1181.872314),
            v(390.0, 1181.872314),
            v(390.0, 986.872314),
            v(390.0, 0.001953),
            v(433.926574, -76.082763),
            v(419.318847, -50.781982),
            FilletVertex {
                x: 194.999816,
                y: -75.548339,
                frad: 2044.089965,
            },
            v(-29.319198, -50.781982),
        ];
        let (poly, notes) = flatten_ring(&ring).unwrap();
        assert_eq!(notes.consumed_fillets, vec![8]);
        assert!(
            poly.iter()
                .any(|p| { (p[0] - 195.0).abs() < 5.0 && p[1] < -55.0 }),
            "被门禁吸附的原半径圆弧必须留下中部弧点: {poly:?}"
        );
        assert!(poly.contains(&[419.318847, -50.781982]));
        assert!(poly.contains(&[-29.319198, -50.781982]));
        assert!(poly.iter().any(|p| {
            (p[0] - 419.318847).abs() < 0.001
                && p[0] != 419.318847
                && (p[1] + 50.781982).abs() < 0.001
        }));
    }

    #[test]
    fn concave_near_linear_fillet_is_kept() {
        // 同一 FLOOR 的 NXTR 17496/100363 也是近共线且两侧只剩亚毫米 span，
        // 但它在顺时针外环上是凹角；E3D RVM 保留该弧，不能套用凸角门禁。
        let ring = [
            v(0.0, 0.0),
            v(1453.391601, 30.21289),
            v(1647.533203, 29.314697),
            v(1647.533203, -358.9729),
            v(1453.391601, -359.871215),
            v(397.920654, -381.812377),
            v(375.148925, -466.802246),
            FilletVertex {
                x: 168.901611,
                y: -220.386962,
                frad: 3036.120117,
            },
            v(-84.358764, -22.605224),
        ];
        let (poly, notes) = flatten_ring(&ring).unwrap();
        assert!(notes.consumed_fillets.is_empty(), "{notes:?}");
        assert!(poly.len() > ring.len(), "凹角圆弧必须离散保留: {poly:?}");
    }

    // ─── 负 FRAD（`mthArcRadius` 凹圆角，模块文档第 4 条）────────────────────

    /// 负 FRAD 的弧以角点为心、半径 `|FRAD|`、取劣弧：端点各距角点 `|FRAD|` 落在两邻边
    /// 上，弧上每个点到角点等距，没有点钻进半径以内。100×100 正方形右下角 FRAD=−25 挖掉
    /// 的正是「角点为心的内接扇」——直角劣弧落在 45° 相位格上恰是两段，扇面积
    /// `½R²·2·sin(π/4)`。
    #[test]
    fn negative_frad_is_a_concave_arc_centred_on_the_vertex() {
        let ring = [
            v(0.0, 0.0),
            FilletVertex {
                x: 100.0,
                y: 0.0,
                frad: -25.0,
            },
            v(100.0, 100.0),
            v(0.0, 100.0),
        ];
        let (poly, notes) = flatten_ring(&ring).unwrap();
        assert!(notes.overshoot_fillets.is_empty(), "{notes:?}");
        assert!(notes.degenerate_corners.is_empty(), "{notes:?}");
        // 端点 = 角点沿两邻边各退 25。
        assert!(
            poly.iter()
                .any(|p| (p[0] - 75.0).abs() < 1e-9 && p[1].abs() < 1e-9)
        );
        assert!(
            poly.iter()
                .any(|p| (p[0] - 100.0).abs() < 1e-9 && (p[1] - 25.0).abs() < 1e-9)
        );
        let corner = [100.0_f64, 0.0];
        for p in &poly {
            let d = (p[0] - corner[0]).hypot(p[1] - corner[1]);
            assert!(d > 25.0 - 1e-6, "凹弧内侧不许有点：{p:?} 距角点 {d}");
            assert!(
                d > 99.0 || (d - 25.0).abs() < 1e-6,
                "弧点必须落在以角点为心 R=25 的圆上：{p:?} 距角点 {d}"
            );
        }
        // 挖掉的面积 = 从角点扇出的两个三角形（相位格把 90° 劣弧分在 135° 一个格点上）。
        let removed = 10_000.0 - signed_area(&poly);
        let expected = 0.5 * 625.0 * 2.0 * std::f64::consts::FRAC_PI_4.sin();
        assert!(
            (removed - expected).abs() < 1e-6,
            "挖掉 {removed}，期望内接扇 {expected}"
        );
    }

    /// 正 / 负 FRAD 在直角处退距相同（`R·cot45° = |FRAD|`）：同一张弦、反向鼓——
    /// 凸弧的弧内点比 R 更靠角点，凹弧的弧内点恒在 R 上；凸圆角留下的材料更多。
    #[test]
    fn positive_and_negative_frad_bulge_opposite_on_the_same_chord() {
        let with_frad = |frad: f64| {
            [
                v(0.0, 0.0),
                FilletVertex {
                    x: 100.0,
                    y: 0.0,
                    frad,
                },
                v(100.0, 100.0),
                v(0.0, 100.0),
            ]
        };
        let (pos, _) = flatten_ring(&with_frad(25.0)).unwrap();
        let (neg, _) = flatten_ring(&with_frad(-25.0)).unwrap();
        // 同一张弦：两个端点在两条折线里都原样出现。
        for target in [[75.0, 0.0], [100.0, 25.0]] {
            for poly in [&pos, &neg] {
                assert!(
                    poly.iter()
                        .any(|p| (p[0] - target[0]).abs() < 1e-9
                            && (p[1] - target[1]).abs() < 1e-9),
                    "弦端点 {target:?} 必须保留"
                );
            }
        }
        // 反向鼓（用到角点的距离判）：弧内点凸支 < 25、凹支 = 25。
        let corner = [100.0_f64, 0.0];
        let arc_interior = |poly: &[[f64; 2]]| -> Vec<f64> {
            poly.iter()
                .map(|p| (p[0] - corner[0]).hypot(p[1] - corner[1]))
                .filter(|d| *d < 99.0 && (*d - 25.0).abs() > 1e-9)
                .collect()
        };
        let pos_interior = arc_interior(&pos);
        let neg_interior = arc_interior(&neg);
        assert!(!pos_interior.is_empty(), "凸弧应有弧内点");
        assert!(pos_interior.iter().all(|d| *d < 25.0 - 1e-6));
        assert!(neg_interior.is_empty(), "凹弧所有点都该在 R=25 上：{neg:?}");
        assert!(
            signed_area(&pos) > signed_area(&neg) + 100.0,
            "凸圆角留下的材料必须多于凹圆角"
        );
    }

    /// `|FRAD| < 0.1` 的负倒角照 Core3D 的 `fabs` 门槛不倒角，顶点原样留下
    /// （正支的微小半径行为不在本条约束里，维持既有 RVM 口径）。
    #[test]
    fn a_tiny_negative_frad_stays_sharp() {
        let ring = [
            v(0.0, 0.0),
            FilletVertex {
                x: 100.0,
                y: 0.0,
                frad: -0.05,
            },
            v(100.0, 100.0),
            v(0.0, 100.0),
        ];
        let (poly, notes) = flatten_ring(&ring).unwrap();
        assert_eq!(poly.len(), 4, "|FRAD|=0.05 < 0.1，四个顶点原样留下");
        assert!(poly.contains(&[100.0, 0.0]));
        assert!((signed_area(&poly) - 10_000.0).abs() < 1e-9);
        assert!(notes.degenerate_corners.is_empty());
    }

    /// 近共线（`sin 2α ≤ 0.1`，Core3D 分流前的 cross 门）的负倒角不倒——不然一个几乎
    /// 拉直的角会被展开成接近半圈的大凹弧，凭空挖走一大块材料。
    #[test]
    fn a_near_collinear_negative_frad_stays_sharp() {
        // 中间顶点两邻边夹角约 176°：sin ≈ 0.07 < 0.1。
        let (sin4, cos4) = (4.0_f64).to_radians().sin_cos();
        let ring = [
            v(-100.0, 0.0),
            FilletVertex {
                x: 0.0,
                y: 0.0,
                frad: -25.0,
            },
            v(100.0 * cos4, 100.0 * sin4),
            v(0.0, 200.0),
        ];
        let (poly, notes) = flatten_ring(&ring).unwrap();
        assert_eq!(poly.len(), 4, "近共线负倒角必须留尖角：{poly:?}");
        assert!(poly.contains(&[0.0, 0.0]));
        assert_eq!(notes.degenerate_corners, vec![1]);
    }

    /// 负倒角的退距就是 `|FRAD|` 本身：超过邻边长时照 `mthArcRadius` 原样展开
    /// （不裁、不吸附——负支没有正支那套近越界吸附的实测依据），只记账。
    #[test]
    fn an_oversized_negative_frad_is_kept_unclipped_and_reported() {
        let ring = [
            v(0.0, 0.0),
            FilletVertex {
                x: 100.0,
                y: 0.0,
                frad: -150.0,
            },
            v(100.0, 100.0),
            v(0.0, 100.0),
        ];
        let (poly, notes) = flatten_ring(&ring).unwrap();
        assert_eq!(notes.overshoot_fillets, vec![1]);
        // 两个端点越过邻点也原样保留：(100,0)−(150,0) 与 (100,0)+(0,150)。
        assert!(
            poly.iter()
                .any(|p| (p[0] + 50.0).abs() < 1e-9 && p[1].abs() < 1e-9)
        );
        assert!(
            poly.iter()
                .any(|p| (p[0] - 100.0).abs() < 1e-9 && (p[1] - 150.0).abs() < 1e-9)
        );
    }

    /// 真实数据钉：ZDJ 甲板板 PANE `/DK_11000_TOS_P01`（zdj7150，refno 15342/605）的
    /// 外环——语料里仅有的负 FRAD 家族之一（AVEVA 样例库全零），顶点
    /// (402.105, 299.084) FRAD=−25。凹弧以该顶点为心、半径 25，绕向不翻转，挖掉的
    /// 面积不超过以角点为心的理论扇形。数据来自 gen-model
    /// `examples/verify_neg_frad_panel` 的同源导出（`out/neg-frad-verify/`）。
    #[test]
    fn zdj_deck_panel_negative_frad_carves_a_concave_arc() {
        let corner = [402.105_010_986_328_1, 299.084_014_892_578_1];
        let ring = [
            v(0.0, 0.0),
            v(-16.419_000_625_610_35, 24.927_000_045_776_367),
            FilletVertex {
                x: corner[0],
                y: corner[1],
                frad: -25.0,
            },
            v(402.104_003_906_25, 0.0),
        ];
        let sharp: Vec<[f64; 2]> = ring.iter().map(|p| [p.x, p.y]).collect();
        let (poly, notes) = flatten_ring(&ring).unwrap();
        assert!(notes.overshoot_fillets.is_empty(), "{notes:?}");
        assert!(notes.degenerate_corners.is_empty(), "{notes:?}");
        // 角点消失，弧点全部落在以角点为心 R=25 的圆上（其余顶点都离角点 ≥ 300）。
        for p in &poly {
            let d = (p[0] - corner[0]).hypot(p[1] - corner[1]);
            assert!(d > 25.0 - 1e-6, "凹弧内侧不许有点：{p:?} 距角点 {d}");
            assert!(
                d > 299.0 || (d - 25.0).abs() < 1e-6,
                "弧点必须落在 R=25 上：{p:?} 距角点 {d}"
            );
        }
        // 这条环是顺时针（有向面积为负）；凹圆角挖料，|面积| 变小、绕向不变，
        // 挖掉的量在 (0, 理论扇形 ½R²·2α] 之间。
        let area_sharp = signed_area(&sharp);
        let area_fixed = signed_area(&poly);
        assert!(area_sharp < 0.0 && area_fixed < 0.0, "绕向必须保持顺时针");
        let removed = area_sharp.abs() - area_fixed.abs();
        let u = [ring[1].x - corner[0], ring[1].y - corner[1]];
        let w = [ring[3].x - corner[0], ring[3].y - corner[1]];
        let two_alpha = ((u[0] * w[0] + u[1] * w[1]) / (u[0].hypot(u[1]) * w[0].hypot(w[1])))
            .clamp(-1.0, 1.0)
            .acos();
        let sector = 0.5 * 625.0 * two_alpha;
        assert!(
            removed > 0.8 * sector && removed <= sector + 1e-9,
            "挖掉 {removed}，理论扇形 {sector}（内接扇应略小于它）"
        );
    }
}

//! 实体构建与 CSG（对标 libgm：`gm_CreateExtrusion` / `gm_CreateBox` /
//! `gm_CreateCylinder` / `gm_CreateCombination`）。
//!
//! 内核用 manifold-csg（gen-model ADR-029 同款）。三条从 gen-model 实测里搬来的
//! 硬语义：
//!
//! 1. 截面填充用 `FillRule::NonZero`：E3D 的倒圆不裁剪，切点越界的自交轮廓
//!    libgm 照铺，NonZero 能把它化解成正确的实心区域。
//! 2. 环绕向按**外环**统一翻正：外环顺时针就整组反转，孔环靠与外环相反的绕向
//!    在 NonZero 下扣掉。
//! 3. libgm 的面级判定容差是 [`RES_TOL_MM`]（`GM_User::restol_` =
//!    `gm_SetResolutionTolerance(0.051)`）；manifold-csg 没有等价的布尔判定参数，
//!    因而负体做差前只用 [`MANIFOLD_NUDGE_MM`] 做最小几何让刀。PDMS 负体常与母体共壁，
//!    不让开，差集沿共壁碎成薄片或留一层皮（gen-model `manifold_csg.rs` 的
//!    实测：穹顶 NREV 碎成 132 壳、GWALL 穿墙洞留皮 p95=753.9）。
//!
//!    **让刀必须在负体自己的局部系、建体时做**（挤出/回转走截面 miter offset，
//!    见 [`extrude_rings_inflated`]；基本体在局部系逐轴放大）。曾经的做法是世界系
//!    AABB 逐轴缩放，ams1112 GWALL `17496/116549` 当场翻车：NXTR 的切边与墙外边
//!    共线（建模者画平的，存储值差 ~0.02mm），该边在世界系里近乎穿过负体 AABB 中心，
//!    逐轴缩放对这种边的外推量趋近 0，差完留下一根 0.01mm 厚、贯穿全高的针，
//!    把 AABB 顶出去 63.5mm。截面 offset 是**平行外扩**，斜边不转动，不吃摆放角度。

use glam::DMat4;
use manifold_csg::{CrossSection, FillRule, JoinType, Manifold};

use crate::discretise::{PROFILE_CHORD_TOL_MM, circle_segments_uncapped, signed_area};
use crate::transform::dmat4_to_affine4x3;

/// libgm 面级布尔判定容差（mm），`GM_User::restol_`。
pub const RES_TOL_MM: f64 = 0.051;
/// manifold-csg 共壁穿透扰动（mm）。
///
/// 这不是把 libgm 的判定容差直接变成几何膨胀：ams1112 实测 `1e-6` 尚未切穿，
/// `0.001` 同时使 GWALL 17496/116569 与 17496/116549 对拍 exact，而 `0.01`
/// 已令前者发生拓扑跃迁并过切 25.88mm。
pub const MANIFOLD_NUDGE_MM: f64 = 0.001;

/// 圆离散的弦高容差（mm），与 [`crate::discretise::CHORD_TOL_MM`] 同源。
pub const FACET_TOL_MM: f64 = crate::discretise::CHORD_TOL_MM;

/// Manifold 密集负体布尔的等价分组上限。
///
/// Core3D/libgm 按 LNEGIT 成员顺序逐刀；集合上 `A-B-C == A-(B∪C)`。
/// Manifold 在 AMS1112 的 25+ 刀 FLOOR 上逐刀会累积细长边界误差，每 24 刀
/// 先求并集可把最大 AABB 差从 22.6344mm 降到 2.7308mm，且全 AMS1112
/// 没有引入新 AABB 回归。
const BOOLEAN_GROUP_SIZE: usize = 24;

#[derive(Debug, thiserror::Error)]
pub enum SolidError {
    #[error("{what} 高度 {height} 不是正数")]
    BadHeight { what: &'static str, height: f64 },
    #[error("{what} 尺寸退化: {detail}")]
    Degenerate { what: &'static str, detail: String },
    #[error("{what} 截面填充后为空（环全退化或互相抵消）")]
    EmptySection { what: &'static str },
    #[error("{what} 生成的流形为空")]
    EmptyManifold { what: &'static str },
}

/// 已折线化的环 → 沿 +Z 挤出 `[0, height]`。
///
/// `rings[0]` 是外边界；全组绕向按外环有向面积统一翻正（gen-model
/// `extrude_flat_polygons` 同款）。
///
/// 对标 Core3D 现代 CSG 路 `CSG_BasicEXT::getPrimGeom`(0x10726e50 → `sub_1071C0E0`)：
/// 第一环 `gm_CreateExtrusionGroup(curve, H)` 作外边界、其余环 `gm_AddCurve(curve, group)`
/// 作孔（活桥反编译，7UW4 档案 §3.7.2）。本 crate 把外环 + 孔放进单张 NonZero 截面，
/// 与 libgm「挤出组 + AddCurve」同解，见模块文档第 1、2 条。
pub fn extrude_rings(rings: Vec<Vec<[f64; 2]>>, height: f64) -> Result<Manifold, SolidError> {
    extrude_rings_grown(rings, height, 0.0, false)
}

/// GWALL 的 Core3D wall-profile 路只保留归一化后的主连通片；普通 PANE/FLOOR
/// 允许一张 profile 产生多个实体岛，不能套用这条规则。
pub fn extrude_rings_largest_component(
    rings: Vec<Vec<[f64; 2]>>,
    height: f64,
) -> Result<Manifold, SolidError> {
    extrude_rings_grown(rings, height, 0.0, true)
}

/// 负体版挤出：截面沿边法向 miter offset 外扩 `grow`，高度两端各让 `grow`
/// （z ∈ [−grow, height + grow]）。
///
/// offset 是**平行外扩**——每条边沿自己的法向精确外移 `grow`，与摆放角度无关。
/// 这是共壁负体让刀的正确形态；逐轴缩放对斜边只保得住 AABB 六个面（见模块文档）。
pub fn extrude_rings_inflated(
    rings: Vec<Vec<[f64; 2]>>,
    height: f64,
    grow: f64,
) -> Result<Manifold, SolidError> {
    extrude_rings_grown(rings, height, grow, false)
}

fn extrude_rings_grown(
    mut rings: Vec<Vec<[f64; 2]>>,
    height: f64,
    grow: f64,
    largest_component_only: bool,
) -> Result<Manifold, SolidError> {
    const WHAT: &str = "extrusion";
    if !(height > f64::EPSILON) {
        return Err(SolidError::BadHeight { what: WHAT, height });
    }
    if rings.is_empty() || rings[0].len() < 3 {
        return Err(SolidError::Degenerate {
            what: WHAT,
            detail: format!("外环顶点数 {}", rings.first().map_or(0, Vec::len)),
        });
    }
    if signed_area(&rings[0]) < 0.0 {
        for ring in &mut rings {
            ring.reverse();
        }
    }
    let mut section = CrossSection::from_polygons_with_fill_rule(&rings, FillRule::NonZero);
    if largest_component_only {
        // WallJoiner/GM_Profile 的墙体主片语义：自交归一化产生的游离小岛不是
        // 第二堵墙。孔仍留在所选分量内部。
        let components = section.decompose();
        section = components
            .into_iter()
            .max_by(|a, b| a.area().total_cmp(&b.area()))
            .unwrap_or(section);
    }
    if grow > 0.0 {
        // miter_limit 2.0（Clipper2 缺省）：尖角超过 2×grow 截平——让刀只关心
        // 「每个面至少外移 grow」，尖角截平不破坏这个保证。
        section = section.offset(grow, JoinType::Miter, 2.0, 16);
    }
    if section.is_empty() {
        return Err(SolidError::EmptySection { what: WHAT });
    }
    let solid = Manifold::extrude(&section, height + 2.0 * grow);
    let solid = if grow > 0.0 {
        solid.translate(0.0, 0.0, -grow)
    } else {
        solid
    };
    ensure_solid(solid, WHAT)
}

pub(crate) const BOX_WHAT: &str = "box";
pub(crate) const CYLINDER_WHAT: &str = "cylinder";
pub(crate) const SPHERE_WHAT: &str = "sphere";

/// [`box_solid`] 的尺寸校验，单独一份、**不构造几何**。
///
/// 给 [`crate::primitive_instance`] 用：实例化路径要在取 canonical 单位网格之前
/// 把非法尺寸挡掉，否则一个坏元素会留下一条可复用的缓存条目。文案与
/// [`box_solid`] 共用同一个 `what`，两条路报出来的必须逐字同一句。
pub fn validate_box_dims(xlen: f64, ylen: f64, zlen: f64) -> Result<(), SolidError> {
    if !(xlen > 0.0 && ylen > 0.0 && zlen > 0.0) {
        return Err(SolidError::Degenerate {
            what: BOX_WHAT,
            detail: format!("XLEN/YLEN/ZLEN = {xlen}/{ylen}/{zlen}"),
        });
    }
    Ok(())
}

/// [`cylinder_solid`] 的尺寸校验，单独一份、**不构造几何**。见
/// [`validate_box_dims`] 的说明。
pub fn validate_cylinder_dims(diam: f64, heig: f64) -> Result<(), SolidError> {
    if !(diam > 0.0 && heig.abs() > f64::EPSILON) {
        return Err(SolidError::Degenerate {
            what: CYLINDER_WHAT,
            detail: format!("DIAM/HEIG = {diam}/{heig}"),
        });
    }
    Ok(())
}

/// `gm_CreateBox`：XLEN/YLEN/ZLEN，原点居中。
pub fn box_solid(xlen: f64, ylen: f64, zlen: f64) -> Result<Manifold, SolidError> {
    validate_box_dims(xlen, ylen, zlen)?;
    ensure_solid(Manifold::cube(xlen, ylen, zlen, true), BOX_WHAT)
}

/// `gm_CreateCylinder`：DIAM/HEIG，轴向为局部 Y，原点在底面中心。
/// 正 HEIG 占 `[0, H]`，负 HEIG 占 `[-|H|, 0]`。该约定由 AMS7997
/// `/Copy-of-R43-R740-002` 的 E3D RVM 面片逐件坐实，不是从旧实现推断。
pub fn cylinder_solid(diam: f64, heig: f64) -> Result<Manifold, SolidError> {
    validate_cylinder_dims(diam, heig)?;
    let radius = diam / 2.0;
    let solid = Manifold::cylinder(heig.abs(), radius, radius, cylinder_segments(radius), false);
    ensure_solid(
        placed(&solid, DMat4::from_rotation_x(cylinder_tilt(heig))),
        CYLINDER_WHAT,
    )
}

/// CYLI 把 canonical 的 +Z 体摆到局部 Y 的那一转，正负 HEIG 各一个方向。
///
/// 直建路（[`cylinder_solid`]）和实例化路（[`crate::primitive_instance`]）必须
/// 转同一个角，否则同一个元素两条路出来的朝向相反。
pub fn cylinder_tilt(heig: f64) -> f64 {
    if heig.is_sign_positive() {
        -std::f64::consts::FRAC_PI_2
    } else {
        std::f64::consts::FRAC_PI_2
    }
}

/// E3D `LNEGIT` 组合路径里的圆柱约定：轴向局部 Z、原点居中。
///
/// 普通 CYLI 的导出约定见 [`cylinder_solid`]；但 AMS7997 的三件带负体 CYLI
/// `24381/130755,130757,130767` 的 RVM 面片明确显示，进入组合体后正体和 NCYL
/// 刀具都按这一套居中 Z 约定构造。两条路径不能再共用一个隐含约定。
pub fn csg_cylinder_solid(diam: f64, heig: f64) -> Result<Manifold, SolidError> {
    csg_cylinder_solid_with_segments(diam, heig, cylinder_segments(diam / 2.0))
}

/// 指定段数的局部 Z 轴居中圆柱，供已经由外部 RVM facet 策略定形的 baked 组合体。
pub fn csg_cylinder_solid_with_segments(
    diam: f64,
    heig: f64,
    segments: i32,
) -> Result<Manifold, SolidError> {
    const WHAT: &str = "CSG cylinder";
    let height = heig.abs();
    if !(diam > 0.0 && height > f64::EPSILON) {
        return Err(SolidError::Degenerate {
            what: WHAT,
            detail: format!("DIAM/HEIG = {diam}/{heig}"),
        });
    }
    let radius = diam / 2.0;
    ensure_solid(
        Manifold::cylinder(height, radius, radius, segments, true),
        WHAT,
    )
}

/// Unit-radius sphere with libgm's radius-dependent facet count supplied by
/// the caller. Catalogue spheres scale this canonical body at placement time,
/// so equal segment classes share one mesh even when their diameters differ.
pub fn unit_sphere_solid(segments: i32) -> Result<Manifold, SolidError> {
    if segments < 4 {
        return Err(SolidError::Degenerate {
            what: SPHERE_WHAT,
            detail: format!("segments = {segments}"),
        });
    }
    ensure_solid(Manifold::sphere(1.0, segments), SPHERE_WHAT)
}

/// `gm_CreatePyramid`：底面 XBOT×YBOT 在 z = −H/2，顶面 XTOP×YTOP 在 z = +H/2，
/// XOFF/YOFF 是顶面中心相对底面中心的偏移，**上下各摊一半**——原点落在两个面心的
/// 中点，不是底面中心（rvmparser 与 aios-core `Pyramid` 同一约定，RVM 对拍靠它对齐）。
///
/// 用凸包建，不自己铺三角：四个侧面各由上下两条**平行**边张成，平行线必共面，
/// 所以 8 个角点的凸包精确就是这个棱台。顶面退化成线（楔形）或点（四棱锥）时
/// 同一句话照样成立，省掉三种退化的分支——而每一个分支都是一处可能悄悄出错的地方。
pub fn pyramid_solid(
    xbot: f64,
    ybot: f64,
    xtop: f64,
    ytop: f64,
    height: f64,
    xoff: f64,
    yoff: f64,
) -> Result<Manifold, SolidError> {
    const WHAT: &str = "pyramid";
    if !(height > 0.0) {
        return Err(SolidError::BadHeight { what: WHAT, height });
    }
    if xbot < 0.0 || ybot < 0.0 || xtop < 0.0 || ytop < 0.0 {
        return Err(SolidError::Degenerate {
            what: WHAT,
            detail: format!("边长为负 XBOT/YBOT/XTOP/YTOP = {xbot}/{ybot}/{xtop}/{ytop}"),
        });
    }
    // 上下两个面至少要有一个有面积；两个都退化就没有实体
    // （aios-core `Pyramid::check_valid` 的 `size_flag` 同款）。
    if !(xbot * ybot > f64::EPSILON || xtop * ytop > f64::EPSILON) {
        return Err(SolidError::Degenerate {
            what: WHAT,
            detail: format!(
                "上下面都无面积 XBOT×YBOT={} XTOP×YTOP={}",
                xbot * ybot,
                xtop * ytop
            ),
        });
    }

    // 亚微米级半边长按 0 处理（gen-model `gen_pyramid` 的 `DEGENERATE_EDGE`）：
    // 该退化成点/线的就干净退化，别留一圈狭长三角把凸包搅出噪声面。
    let snap = |v: f64| if v < DEGENERATE_EDGE_MM { 0.0 } else { v };
    let (hx_b, hy_b) = (snap(xbot / 2.0), snap(ybot / 2.0));
    let (hx_t, hy_t) = (snap(xtop / 2.0), snap(ytop / 2.0));
    let (ox, oy) = (xoff / 2.0, yoff / 2.0);
    let h2 = height / 2.0;

    let corners = [
        [-hx_b - ox, -hy_b - oy, -h2],
        [hx_b - ox, -hy_b - oy, -h2],
        [hx_b - ox, hy_b - oy, -h2],
        [-hx_b - ox, hy_b - oy, -h2],
        [-hx_t + ox, -hy_t + oy, h2],
        [hx_t + ox, -hy_t + oy, h2],
        [hx_t + ox, hy_t + oy, h2],
        [-hx_t + ox, hy_t + oy, h2],
    ];
    ensure_solid(Manifold::hull_pts(&corners), WHAT)
}

/// `GM_Snout`：两端直径 DBOT/DTOP，高 HEIG，局部 Z 轴且原点居中。
/// XOFF/YOFF 是顶面中心相对底面中心的偏移；线性剪切自然把偏移上下各摊一半。
pub fn snout_solid(
    dbot: f64,
    dtop: f64,
    height: f64,
    xoff: f64,
    yoff: f64,
) -> Result<Manifold, SolidError> {
    const WHAT: &str = "snout";
    if !(height > f64::EPSILON) {
        return Err(SolidError::BadHeight { what: WHAT, height });
    }
    if dbot < 0.0 || dtop < 0.0 || !(dbot > f64::EPSILON || dtop > f64::EPSILON) {
        return Err(SolidError::Degenerate {
            what: WHAT,
            detail: format!("DBOT/DTOP = {dbot}/{dtop}"),
        });
    }
    let (rb, rt) = (dbot / 2.0, dtop / 2.0);
    let body = Manifold::cylinder(height, rb, rt, snout_segments(rb, rt), true);
    let shear = DMat4::from_cols_array(&[
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        xoff / height,
        yoff / height,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]);
    ensure_solid(placed(&body, shear), WHAT)
}

/// `GM_RectTorus`：矩形截面（径向 RINS→ROUT，轴向 ±HEIG/2）绕局部 **Y** 轴，
/// 从 +X 扫向 +Z，扫角 ANGL 度。ANGL 为负时反向扫。
pub fn rect_torus_solid(
    rins: f64,
    rout: f64,
    height: f64,
    angle_deg: f64,
) -> Result<Manifold, SolidError> {
    const WHAT: &str = "rectangular torus";
    if !(height > 0.0) {
        return Err(SolidError::BadHeight { what: WHAT, height });
    }
    // `fmax(RINS, 0)`：Core3D `CSG_BasicRTO::getPrimGeom` 在属性读取处就夹，
    // 负内半径照常建体（内缘贴轴），不是拒绝。
    let rins = rins.max(0.0);
    if !(rout - rins > f64::EPSILON) {
        return Err(SolidError::Degenerate {
            what: WHAT,
            detail: format!("RINS/ROUT = {rins}/{rout}，截面无宽度"),
        });
    }
    let h2 = height / 2.0;
    let section = vec![vec![[rins, -h2], [rout, -h2], [rout, h2], [rins, h2]]];
    let solid = swept_solid(section, rout, angle_deg, WHAT)?;
    Ok(placed(
        &solid,
        DMat4::from_rotation_x(std::f64::consts::FRAC_PI_2),
    ))
}

/// `GM_CircTorus`：圆管截面绕局部 **Y** 轴，从 +X 扫向 +Z，扫角 ANGL 度。
/// 管半径 (ROUT−RINS)/2，中心线半径 (ROUT+RINS)/2。
///
/// 两个方向喂**两个不同的半径**（libgm §7.9.1 点名最容易照抄错的一条）：
/// 环向喂 ROUT，管截面喂管半径。
pub fn circ_torus_solid(rins: f64, rout: f64, angle_deg: f64) -> Result<Manifold, SolidError> {
    const WHAT: &str = "circular torus";
    let rins = rins.max(0.0);
    let tube_r = (rout - rins) / 2.0;
    if !(tube_r > f64::EPSILON) {
        return Err(SolidError::Degenerate {
            what: WHAT,
            detail: format!("RINS/ROUT = {rins}/{rout}，管半径不是正数"),
        });
    }
    let center_r = (rout + rins) / 2.0;
    let n = circular_torus_tube_segments(rins, rout);
    let tube: Vec<[f64; 2]> = (0..n)
        .map(|k| {
            let a = std::f64::consts::TAU * f64::from(k) / f64::from(n);
            // 贴轴（RINS=0）时 θ=π 那一点算出来可能差一个 ulp 到负半平面，
            // manifold 的 revolve 会把它裁掉、在轴心留一圈针状面；这里直接夹住。
            [(center_r + tube_r * a.cos()).max(0.0), tube_r * a.sin()]
        })
        .collect();
    let solid = swept_solid(vec![tube], rout, angle_deg, WHAT)?;
    Ok(placed(
        &solid,
        DMat4::from_rotation_x(std::f64::consts::FRAC_PI_2),
    ))
}

/// `gm_CreateRevolution`：环剖面绕**局部 X 轴**（过原点）回转 ANGL 度
/// （aios-core `Revolution::default` 的 `rot_dir = X` / `rot_pt = 0`）。
///
/// `rings[0]` 是外轮廓，其余是孔；坐标是局部 XY 平面上已折线化的点。
///
/// manifold 的 `revolve` 只认一种摆放：截面在 XY、绕自身 Y 轴转、轴落在结果的 Z。
/// 所以先把轮廓换算进「(半径, 轴向)」二维系，转完再拿一个纯旋转把 Z 摆回局部 X。
/// 轮廓整条在轴的哪一侧由最远点定；落在 −Y 侧就把标架绕轴翻 180°（半径方向与
/// 出平面基向量同时取反，保住右手系），而不是去裁轮廓。
///
/// 对标 Core3D `CSG_BasicREV::getPrimGeom`(0x107270c0 → `sub_1071C4F0`)（活桥反编译，
/// 7UW4 §3.7.5）：外环 `gm_CreateRevolution`、其余环 `gm_AddCurve` 作孔。两处**有意与
/// libgm 分歧**，都因 manifold 自带几何内核、无需复刻 libgm 的实现约束（planning §1.1）：
/// 1. libgm 单次回转扫角 ≤180°，>180° 拆两段各建再 `gm_CreateCombination(1)` 并起来；
///    manifold 的 `revolve` 一次能到 360°，本 crate 不拆分。
/// 2. libgm 负角走 `off = 180` 起始相位偏移；本 crate 改为按 |ANGL| 正扫后整体回转
///    −|ANGL|（几何等价于镜像象限，见 `swept_solid_grown`）。partial 负回转的起始
///    相位尚无 RVM 数据坐实——ams1112 仅 2 件 NREV 且为坏数据；出现可对拍的 partial
///    负回转再核 `off` 约定。
pub fn revolve_rings(rings: Vec<Vec<[f64; 2]>>, angle_deg: f64) -> Result<Manifold, SolidError> {
    revolve_rings_grown(rings, angle_deg, 0.0)
}

/// 负体版回转：剖面在（半径, 轴向）平面里 miter offset 外扩 `grow` 再回转。
///
/// 部分扫角的两张径向切面**不让**（offset 只作用于剖面平面）——
/// 共壁病的高发面是剖面轮廓那一侧，径向切面贴母体的案例出现时再补。
pub fn revolve_rings_inflated(
    rings: Vec<Vec<[f64; 2]>>,
    angle_deg: f64,
    grow: f64,
) -> Result<Manifold, SolidError> {
    revolve_rings_grown(rings, angle_deg, grow)
}

fn revolve_rings_grown(
    mut rings: Vec<Vec<[f64; 2]>>,
    angle_deg: f64,
    grow: f64,
) -> Result<Manifold, SolidError> {
    const WHAT: &str = "revolution";
    if rings.is_empty() || rings[0].len() < 3 {
        return Err(SolidError::Degenerate {
            what: WHAT,
            detail: format!("外环顶点数 {}", rings.first().map_or(0, Vec::len)),
        });
    }

    let farthest =
        rings.iter().flatten().fold(
            0.0_f64,
            |far, p| if p[1].abs() > far.abs() { p[1] } else { far },
        );
    let (radial_sign, out_of_plane) = if farthest < 0.0 {
        (-1.0, -1.0)
    } else {
        (1.0, 1.0)
    };

    let mut max_radius = 0.0_f64;
    for ring in &mut rings {
        for p in ring.iter_mut() {
            let mut radius = radial_sign * p[1];
            // `movePointsOntoYAxis`：贴轴顶点的半径精确置 0，
            // 否则回转后轴心留一圈纳米级针状面。
            if radius.abs() < NORM_TOL_MM {
                radius = 0.0;
            }
            max_radius = max_radius.max(radius);
            *p = [radius, p[0]];
        }
    }
    if !(max_radius > f64::EPSILON) {
        return Err(SolidError::Degenerate {
            what: WHAT,
            detail: "轮廓整条塌在回转轴上".into(),
        });
    }
    // 换算到「(半径, 轴向)」这一步的行列式是 −1，绕向会翻过来；按外环统一翻回，
    // 所有环一起翻，保住外环与孔的相对绕向。
    if signed_area(&rings[0]) < 0.0 {
        for ring in &mut rings {
            ring.reverse();
        }
    }

    let solid = swept_solid_grown(rings, max_radius, angle_deg, grow, WHAT)?;
    // manifold 交回来的是「轴在 +Z、半径铺在 XY」。摆回局部系：
    // +X → 局部 +Y（半径方向），+Y → 出平面，+Z → 局部 +X（回转轴）。
    let frame = DMat4::from_cols(
        glam::DVec4::new(0.0, radial_sign, 0.0, 0.0),
        glam::DVec4::new(0.0, 0.0, out_of_plane, 0.0),
        glam::DVec4::new(1.0, 0.0, 0.0, 0.0),
        glam::DVec4::W,
    );
    Ok(placed(&solid, frame))
}

/// 截面绕 Z 轴扫 `angle_deg` 度。`radius_for_segments` 是喂给段数规则的那个半径。
///
/// 负扫角不交给 manifold（它按 `slice * dPhi` 铺点，负步长会把三角绕向翻里朝外）：
/// 按 |ANGL| 正扫再整体回转 −|ANGL|，几何完全等价。
fn swept_solid(
    section: Vec<Vec<[f64; 2]>>,
    radius_for_segments: f64,
    angle_deg: f64,
    what: &'static str,
) -> Result<Manifold, SolidError> {
    swept_solid_grown(section, radius_for_segments, angle_deg, 0.0, what)
}

fn swept_solid_grown(
    section: Vec<Vec<[f64; 2]>>,
    radius_for_segments: f64,
    angle_deg: f64,
    grow: f64,
    what: &'static str,
) -> Result<Manifold, SolidError> {
    if !angle_deg.is_finite() || angle_deg == 0.0 {
        return Err(SolidError::Degenerate {
            what,
            detail: format!("ANGL = {angle_deg}"),
        });
    }
    let sweep = angle_deg.abs().min(360.0);
    let mut cross = CrossSection::from_polygons_with_fill_rule(&section, FillRule::NonZero);
    if grow > 0.0 {
        // 外扩越过回转轴（半径 < 0）的部分由 manifold 的 revolve 裁掉——
        // 裁的是轴，正是膨胀该停下的地方。
        cross = cross.offset(grow, JoinType::Miter, 2.0, 16);
    }
    if cross.is_empty() {
        return Err(SolidError::EmptySection { what });
    }
    let segments = part_rev_segments(radius_for_segments, sweep);
    // Manifold::Revolve treats values <= 2 as "use the global default" and
    // then truncates `default * sweep / 360`. A valid narrow E3D sweep can
    // therefore become zero divisions (AMS7997 BEND 24381/104871). Three is
    // the smallest explicit division count accepted by that API.
    let solid = Manifold::revolve(&cross, segments.max(3), sweep);
    let solid = if angle_deg < 0.0 {
        solid.rotate(0.0, 0.0, -sweep)
    } else {
        solid
    };
    ensure_solid(solid, what)
}

/// 曲面原语的逐段封顶，对齐 libgm 的 1000。
///
/// 公式自己不封顶，封顶是每个 `GM_*::calcFacets` 各加各的
/// （超了还打一条「facet tolerance too small for radius, adjusted」）。
/// 所以它跟段数规则一样是复刻项，不是画质旋钮：取别的数，撞顶的那些圆就跟 E3D
/// 对不上。按 0.5mm 容差，R=23400 才 484 段，离顶还远。
pub const MAX_SEGMENTS: i32 = 1000;

/// `GM_Cylinder::calcFacets` 的整圆段数：喂自己的半径，封顶 [`MAX_SEGMENTS`]。
pub fn cylinder_segments(radius: f64) -> i32 {
    circle_segments_uncapped(radius, FACET_TOL_MM).min(MAX_SEGMENTS)
}

/// E3D RVM 的 SNOU FacetGroup 使用粗粒度 baked facet；`24381/130759,130763`
/// 的 R=27.5mm 外圈及其中心孔均为 8 段。SNOU 直接按唯一外部基准定形。
pub fn snout_segments(r_bottom: f64, r_top: f64) -> i32 {
    circle_segments_uncapped(r_bottom.max(r_top), 5.0).min(MAX_SEGMENTS)
}

/// `d2_numberOfSegmentsForPartRev`：一段扫角分几段。
///
/// **整圈段数按扫角等比例缩**，最少 2 段——不是拿扫角除步长，那样跟 E3D 差一段。
/// 整圈那一步走曲面原语口径（封顶 [`MAX_SEGMENTS`]）；截面弧那一支不封顶，在
/// [`crate::discretise`] 里。两条路各有各的调用点，不要互相顶替。
pub fn part_rev_segments(radius: f64, sweep_deg: f64) -> i32 {
    let full = circle_segments_uncapped(radius, FACET_TOL_MM).min(MAX_SEGMENTS);
    let sweep = sweep_deg.abs();
    // libgm 在 1e-6 度内把 0 与 360 都判成整圈。
    if sweep <= 1e-6 || (sweep - 360.0).abs() <= 1e-6 || sweep > 360.0 {
        return full;
    }
    ((f64::from(full) * sweep / 360.0).ceil() as i32).max(2)
}

/// `GM_CircTorus` 管截面方向的段数：喂 `(ROUT − RINS) / 2`，**不是**外半径。
pub fn circular_torus_tube_segments(rins: f64, rout: f64) -> i32 {
    circle_segments_uncapped((rout - rins) * 0.5, FACET_TOL_MM).min(MAX_SEGMENTS)
}

/// 回转轮廓的轴心吸附阈值（mm），`GM_User::normtol_`。
///
/// 值是 libgm 的初值 1e-6 而不是 `restol_` 那个 0.051：Core3D 虽然也调了
/// `gm_SetDefaultNormalisationTolerance`，但 `GM_User::normtol(double)` 这个写入器
/// 在 libgm 内零调用，运行期恒为初值（gen-model `libgm_discretise::NORM_TOL` 同源）。
pub const NORM_TOL_MM: f64 = 1e-6;

/// 半边长低于这个值（mm，亚微米）就当 0（gen-model `DEGENERATE_EDGE` 同值）。
const DEGENERATE_EDGE_MM: f64 = 0.001;

/// 世界系放置：4×4 → manifold 的 4×3 仿射。
pub fn placed(solid: &Manifold, world: DMat4) -> Manifold {
    solid.transform(&dmat4_to_affine4x3(world))
}

/// 以自身包围盒中心为原点逐轴放大，每轴向外让 `grow` 毫米
/// （gen-model `inflate_about_center` 同款：逐轴绝对量，退化轴不动）。
///
/// **只适用于面与所在系轴对齐的体**（局部系里的 box/cylinder 这类）：
/// 它保证的是 AABB 六个面各外移 `grow`，斜面的外移量随其到中心的距离缩水，
/// 贴中心的斜边甚至只是原地打转（见模块文档的 GWALL 案例）。
/// 挤出/回转负体走 [`extrude_rings_inflated`] / [`revolve_rings_inflated`]。
pub fn inflate_about_center(solid: &Manifold, grow: f64) -> Manifold {
    let Some(bb) = solid.bounding_box() else {
        return solid.clone();
    };
    let (lo, hi) = (bb.min(), bb.max());
    let center = glam::DVec3::new(
        (lo[0] + hi[0]) * 0.5,
        (lo[1] + hi[1]) * 0.5,
        (lo[2] + hi[2]) * 0.5,
    );
    let axis_scale = |min: f64, max: f64| {
        let extent = max - min;
        if extent > f64::EPSILON {
            (extent + 2.0 * grow) / extent
        } else {
            1.0
        }
    };
    let scale = glam::DVec3::new(
        axis_scale(lo[0], hi[0]),
        axis_scale(lo[1], hi[1]),
        axis_scale(lo[2], hi[2]),
    );
    let m = DMat4::from_translation(center)
        * DMat4::from_scale(scale)
        * DMat4::from_translation(-center);
    solid.transform(&dmat4_to_affine4x3(m))
}

/// 正体 − 负体组（都已在世界系）。
///
/// 负体必须**已在各自局部系让好刀**再进来（`elmodl::negative_world_solid` 负责）：
/// 挤出/回转在建体时截面 offset，基本体在局部系 [`inflate_about_center`]。
/// 这里不再让——世界系里让刀对旋转过的负体保证不了每个面都外移（见模块文档）。
pub fn subtract_negatives(pos: Manifold, negs: Vec<Manifold>) -> Manifold {
    // Core3D 的 LNEGIT 按成员原序逐刀布尔；逆序会让 AMS1112 另外 4 块
    // 本来正确的 FLOOR 回归。少量刀仍严格逐刀，只对密集刀做保序分组。
    if negs.len() <= BOOLEAN_GROUP_SIZE {
        return negs.into_iter().fold(pos, |body, neg| &body - &neg);
    }
    negs.chunks(BOOLEAN_GROUP_SIZE).fold(pos, |body, group| {
        let cutters = Manifold::batch_union(group);
        &body - &cutters
    })
}

/// 清掉 FLOOR 密集/单刀布尔产生的亚 facet 卫星碎片。
///
/// 判据用 `volume / FLOOR轴向高度` 作为投影面积代理；低于一个
/// `PROFILE_CHORD_TOL_MM²` 单元的独立分量不可能贡献一个完整 RVM facet。主分量
/// 永远保留。只对单刀或至少 10 刀的结果启用：少量多刀产生的细长独立片在
/// AMS1112 确有合法 RVM 对应（例如 17496/79729），不可笼统删除。
pub fn cleanup_floor_boolean_components(solid: Manifold, negative_count: usize) -> Manifold {
    if negative_count != 1 && negative_count < 10 {
        return solid;
    }
    let Some(bounds) = solid.bounding_box() else {
        return solid;
    };
    let height = bounds.max()[2] - bounds.min()[2];
    if !(height > f64::EPSILON) {
        return solid;
    }
    let components = solid.decompose();
    if components.len() <= 1 {
        return solid;
    }
    let main = components
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.volume().abs().total_cmp(&b.volume().abs()))
        .map(|(index, _)| index)
        .unwrap_or(0);
    let min_volume = PROFILE_CHORD_TOL_MM * PROFILE_CHORD_TOL_MM * height;
    let kept = components
        .into_iter()
        .enumerate()
        .filter_map(|(index, component)| {
            (index == main || component.volume().abs() >= min_volume).then_some(component)
        })
        .collect::<Vec<_>>();
    Manifold::compose(&kept)
}

pub(crate) fn ensure_solid(solid: Manifold, what: &'static str) -> Result<Manifold, SolidError> {
    if solid.is_empty() || solid.num_tri() == 0 {
        return Err(SolidError::EmptyManifold { what });
    }
    Ok(solid)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn snout_is_closed_centered_and_preserves_volume_under_eccentric_shear() {
        let (dbot, dtop, height, xoff, yoff) = (55.0, 32.0, 30.0, 8.0, -4.0);
        let solid = snout_solid(dbot, dtop, height, xoff, yoff).unwrap();
        assert!(!solid.is_empty());
        assert_eq!(solid.genus(), 0);
        let analytic = std::f64::consts::PI
            * height
            * ((dbot / 2.0_f64).powi(2) + (dbot / 2.0) * (dtop / 2.0) + (dtop / 2.0_f64).powi(2))
            / 3.0;
        let segments = snout_segments(dbot / 2.0, dtop / 2.0) as f64;
        assert_eq!(segments, 8.0, "RVM 样本外圈是 8 段");
        let inscribed = analytic
            * (segments * (std::f64::consts::TAU / segments).sin() / std::f64::consts::TAU);
        assert!((solid.volume() - inscribed).abs() / inscribed < 1e-12);
        let bb = solid.bounding_box().unwrap();
        assert!((bb.min()[2] + height / 2.0).abs() < 1e-9);
        assert!((bb.max()[2] - height / 2.0).abs() < 1e-9);
        assert!((bb.min()[0] - (-dbot / 2.0 - xoff / 2.0)).abs() < 1e-9);
        assert!((bb.max()[0] - (dbot / 2.0 - xoff / 2.0)).abs() < 1e-9);
    }

    #[test]
    fn rvm_snout_with_two_box_trims_and_a_central_hole_keeps_the_observed_surface() {
        let positive = snout_solid(55.0, 32.0, 30.0, 0.0, 0.0).unwrap();
        let trim = box_solid(220.0, 55.0, 220.0).unwrap();
        let lower = placed(
            &trim,
            DMat4::from_translation(glam::DVec3::new(0.0, -37.5, 0.0)),
        );
        let upper = placed(
            &trim,
            DMat4::from_translation(glam::DVec3::new(0.0, 37.5, 0.0)),
        );
        let bore = csg_cylinder_solid_with_segments(20.0, 30.0, snout_segments(0.0, 10.0)).unwrap();
        let cut = subtract_negatives(
            positive,
            vec![
                inflate_about_center(&lower, MANIFOLD_NUDGE_MM),
                inflate_about_center(&upper, MANIFOLD_NUDGE_MM),
                inflate_about_center(&bore, MANIFOLD_NUDGE_MM),
            ],
        );
        assert_eq!(cut.num_tri(), 56, "两件 E3D RVM 样本均为 56 三角形");
        assert_eq!(cut.decompose().len(), 2, "中心孔把窄带分成左右两瓣");
        assert!((cut.volume() - 15_125.707).abs() < 0.1, "{}", cut.volume());
    }

    #[test]
    fn extrusion_volume_matches_profile_area_times_height() {
        let square = vec![[0.0, 0.0], [100.0, 0.0], [100.0, 100.0], [0.0, 100.0]];
        let solid = extrude_rings(vec![square], 10.0).unwrap();
        assert!(
            (solid.volume() - 100_000.0).abs() < 1e-6,
            "{}",
            solid.volume()
        );
    }

    #[test]
    fn a_hole_ring_subtracts_under_nonzero_fill() {
        let outer = vec![[0.0, 0.0], [100.0, 0.0], [100.0, 100.0], [0.0, 100.0]];
        // 孔环与外环反绕向（数据侧的约定），NonZero 下扣掉。
        let hole = vec![[20.0, 20.0], [20.0, 80.0], [80.0, 80.0], [80.0, 20.0]];
        let solid = extrude_rings(vec![outer, hole], 10.0).unwrap();
        let expect = (10_000.0 - 3_600.0) * 10.0;
        assert!(
            (solid.volume() - expect).abs() < 1e-6,
            "体积 {}，期望 {expect}",
            solid.volume()
        );
        assert_eq!(solid.genus(), 1, "带孔挤出体应是亏格 1 的环体");
    }

    #[test]
    fn floor_boolean_cleanup_drops_only_subfacet_satellites() {
        let main = Manifold::cube(100.0, 100.0, 100.0, true);
        let satellite = Manifold::cube(1.0, 1.0, 100.0, true).translate(1000.0, 0.0, 0.0);
        let joined = Manifold::compose(&[main, satellite]);
        assert_eq!(joined.decompose().len(), 2);
        let cleaned = cleanup_floor_boolean_components(joined, 1);
        assert_eq!(cleaned.decompose().len(), 1);
        let bounds = cleaned.bounding_box().unwrap();
        assert!(bounds.max()[0] <= 50.0 + 1e-9);
    }

    #[test]
    fn clockwise_outer_ring_is_flipped_whole() {
        let outer_cw = vec![[0.0, 0.0], [0.0, 100.0], [100.0, 100.0], [100.0, 0.0]];
        let solid = extrude_rings(vec![outer_cw], 10.0).unwrap();
        assert!((solid.volume() - 100_000.0).abs() < 1e-6);
    }

    #[test]
    fn cylinder_starts_at_origin_along_local_y() {
        let solid = cylinder_solid(200.0, 80.0).unwrap();
        let bb = solid.bounding_box().unwrap();
        assert!(bb.min()[1].abs() < 1e-9, "{:?} {:?}", bb.min(), bb.max());
        assert!((bb.max()[1] - 80.0).abs() < 1e-9);
        assert!((bb.min()[2] + 100.0).abs() < 1e-9);
        assert!((bb.max()[2] - 100.0).abs() < 1e-9);
    }

    #[test]
    fn negative_cylinder_height_reverses_the_local_y_direction() {
        let positive = cylinder_solid(51.67, 183.15).unwrap();
        let negative = cylinder_solid(51.67, -183.15).unwrap();
        assert_eq!(positive.num_tri(), negative.num_tri());
        let positive_bounds = positive.bounding_box().unwrap();
        let negative_bounds = negative.bounding_box().unwrap();
        assert!(
            positive_bounds.min()[1].abs() < 1e-9,
            "{:?} {:?}",
            positive_bounds.min(),
            positive_bounds.max()
        );
        assert!((positive_bounds.max()[1] - 183.15).abs() < 1e-9);
        assert!((negative_bounds.min()[1] + 183.15).abs() < 1e-9);
        assert!(negative_bounds.max()[1].abs() < 1e-9);
        assert_eq!(positive.volume().to_bits(), negative.volume().to_bits());
    }

    #[test]
    fn csg_cylinder_is_centered_on_local_z() {
        let solid = csg_cylinder_solid(32.0, -30.0).unwrap();
        let bounds = solid.bounding_box().unwrap();
        assert_eq!(bounds.min(), [-16.0, -16.0, -15.0]);
        assert_eq!(bounds.max(), [16.0, 16.0, 15.0]);
    }

    /// 圆柱体积是**内接正多边形**棱柱的体积，不是 πr²h：R=100 按 0.5mm 弦高容差
    /// 分 32 段，内接 32 边形比真圆少 0.64%。这里钉多边形的精确值，把那 0.64%
    /// 当成段数规则的确定后果记下来——拿 πr²h 配一个「差不多」的相对容差去断言，
    /// 松了盖不住段数规则变化，紧了直接是假红（本条曾按 5e-3 写过，必红）。
    #[test]
    fn cylinder_volume_is_the_inscribed_prism_not_the_circle() {
        let (radius, height) = (100.0_f64, 80.0_f64);
        let n = cylinder_segments(radius);
        assert_eq!(n, 32, "R=100 @0.5mm 容差应当是 32 段");

        let polygon_area =
            0.5 * f64::from(n) * radius * radius * (std::f64::consts::TAU / f64::from(n)).sin();
        let solid = cylinder_solid(2.0 * radius, height).unwrap();
        let expect = polygon_area * height;
        assert!(
            (solid.volume() - expect).abs() / expect < 1e-9,
            "体积 {}，内接 {n} 边形棱柱应为 {expect}",
            solid.volume()
        );

        // 与真圆的缺口只由段数决定；它变了就是段数规则变了。
        let circle = std::f64::consts::PI * radius * radius * height;
        let deficit = (circle - solid.volume()) / circle;
        assert!(
            (0.0064..0.0065).contains(&deficit),
            "32 段的体积缺口应当是 0.64%，实得 {deficit}"
        );
    }

    /// 棱台的体积有闭式解：拟柱体公式 `V = h/6·(A底 + 4·A中 + A顶)`。
    /// 上下面尺寸沿 z 线性变化 ⇒ 面积是 z 的二次式 ⇒ 辛普森积分精确，
    /// 这里能钉到 1e-9，不必给「差不多」的容差。棱台是平面体，没有段数近似。
    #[test]
    fn pyramid_frustum_volume_matches_the_prismatoid_formula() {
        let (xb, yb, xt, yt, h) = (4000.0, 3000.0, 2000.0, 1500.0, 5000.0);
        let solid = pyramid_solid(xb, yb, xt, yt, h, 0.0, 0.0).unwrap();
        let mid = ((xb + xt) / 2.0) * ((yb + yt) / 2.0);
        let expect = h / 6.0 * (xb * yb + 4.0 * mid + xt * yt);
        assert!(
            (solid.volume() - expect).abs() / expect < 1e-9,
            "体积 {}，拟柱体公式 {expect}",
            solid.volume()
        );
    }

    /// XOFF/YOFF 只是把顶面平移，逐层截面面积一个也没变——祖暅原理，体积必须一样。
    /// 这一条同时钉住「偏移上下各摊一半」：摊错了体积仍然不变，但包围盒会偏，
    /// 所以下面接着验两个面心。
    #[test]
    fn pyramid_offset_shifts_the_top_face_without_changing_volume() {
        let (xb, yb, xt, yt, h) = (4000.0, 3000.0, 2000.0, 1500.0, 5000.0);
        let (xoff, yoff) = (1000.0, 500.0);
        let straight = pyramid_solid(xb, yb, xt, yt, h, 0.0, 0.0).unwrap();
        let leaning = pyramid_solid(xb, yb, xt, yt, h, xoff, yoff).unwrap();
        assert!((straight.volume() - leaning.volume()).abs() / straight.volume() < 1e-9);

        // 原点落在两个面心的中点：底面心 (−xoff/2, −yoff/2)，顶面心 (+xoff/2, +yoff/2)。
        let bb = leaning.bounding_box().unwrap();
        assert!((bb.min()[2] + h / 2.0).abs() < 1e-9);
        assert!((bb.max()[2] - h / 2.0).abs() < 1e-9);
        assert!(
            (bb.min()[0] - (-xb / 2.0 - xoff / 2.0)).abs() < 1e-9,
            "{:?}",
            bb.min()
        );
        assert!(
            (bb.max()[0] - (xt / 2.0 + xoff / 2.0)).abs() < 1e-9,
            "{:?}",
            bb.max()
        );
    }

    /// 顶面退化成点 = 四棱锥，体积正好是外接长方体的三分之一。
    #[test]
    fn pyramid_with_a_collapsed_top_is_a_cone_of_one_third() {
        let (xb, yb, h) = (4000.0, 3000.0, 5000.0);
        let solid = pyramid_solid(xb, yb, 0.0, 0.0, h, 0.0, 0.0).unwrap();
        let expect = xb * yb * h / 3.0;
        assert!(
            (solid.volume() - expect).abs() / expect < 1e-9,
            "体积 {}，期望 {expect}",
            solid.volume()
        );
    }

    /// 矩形环面的体积也有闭式解，而且**把段数一起钉住**：
    /// 环向分 m 段、每段夹角 dφ 的多面体环，体积 = `m · h · (Rout²−Rins²)/2 · sin(dφ)`
    /// （每段是两个内接三角形柱之差）。段数规则一变，这个数就变。
    #[test]
    fn rect_torus_volume_is_the_inscribed_wedge_sum() {
        let (rins, rout, h, angle) = (300.0_f64, 500.0_f64, 200.0_f64, 90.0_f64);
        let solid = rect_torus_solid(rins, rout, h, angle).unwrap();
        let m = f64::from(part_rev_segments(rout, angle));
        let d_phi = angle.to_radians() / m;
        let expect = m * h * (rout * rout - rins * rins) / 2.0 * d_phi.sin();
        assert!(
            (solid.volume() - expect).abs() / expect < 1e-9,
            "体积 {}，内接楔形和 {expect}（{m} 段）",
            solid.volume()
        );
        // 与真环面（Rout²−Rins²）/2·θ·h 的缺口只由段数决定。
        let exact = (rout * rout - rins * rins) / 2.0 * angle.to_radians() * h;
        assert!(solid.volume() < exact, "内接体必须小于真环面");
    }

    /// 环面扫掠从 +X 起、朝 +Z 转；ANGL 为负就是朝 −Z 转。
    #[test]
    fn a_negative_sweep_mirrors_the_quadrant_but_not_the_volume() {
        let ccw = rect_torus_solid(300.0, 500.0, 200.0, 90.0).unwrap();
        let cw = rect_torus_solid(300.0, 500.0, 200.0, -90.0).unwrap();
        assert!((ccw.volume() - cw.volume()).abs() / ccw.volume() < 1e-12);
        assert!(
            ccw.bounding_box().unwrap().min()[2] > -1e-9,
            "正扫应落在 z ≥ 0"
        );
        assert!(
            cw.bounding_box().unwrap().max()[2] < 1e-9,
            "负扫应落在 z ≤ 0"
        );
    }

    /// 圆管环面：两个方向喂两个不同的半径，这是 libgm §7.9.1 点名最容易抄错的一条。
    /// 环向喂 ROUT，管截面喂 (ROUT−RINS)/2；抄成同一个半径，管壁段数会差好几倍。
    #[test]
    fn circ_torus_feeds_two_different_radii_to_the_segment_rule() {
        let (rins, rout) = (300.0, 500.0);
        assert_eq!(
            circular_torus_tube_segments(rins, rout),
            cylinder_segments(100.0)
        );
        assert_ne!(
            circular_torus_tube_segments(rins, rout),
            cylinder_segments(rout),
            "管截面段数不能拿外半径去算"
        );
    }

    /// 圆管环面的包围盒是精确量：段数取到 4 的倍数 ⇒ 管截面在 θ=0/±90° 上有顶点，
    /// 环向在 φ=0 上有顶点，于是 x 的最大值恰是 ROUT、y 的幅值恰是管半径。
    /// 体积则是内接的（两个方向都是内接多边形），只断言它落在真环面之下。
    #[test]
    fn circ_torus_touches_rout_and_stays_inside_the_true_torus() {
        let (rins, rout) = (300.0_f64, 500.0_f64);
        let solid = circ_torus_solid(rins, rout, 360.0).unwrap();
        let tube_r = (rout - rins) / 2.0;
        let bb = solid.bounding_box().unwrap();
        assert!((bb.max()[0] - rout).abs() < 1e-9, "{:?}", bb.max());
        assert!((bb.max()[1] - tube_r).abs() < 1e-9, "{:?}", bb.max());
        assert!((bb.min()[1] + tube_r).abs() < 1e-9, "{:?}", bb.min());
        assert_eq!(solid.genus(), 1, "整圈圆管环面是亏格 1");

        // 帕普斯：V = 2π·R中心 · πr²。两个方向都内接 ⇒ 必小于它，且缺口在 1% 内。
        let pappus =
            std::f64::consts::TAU * ((rout + rins) / 2.0) * std::f64::consts::PI * tube_r * tube_r;
        let deficit = (pappus - solid.volume()) / pappus;
        assert!(
            (0.0..0.01).contains(&deficit),
            "内接缺口应在 (0, 1%)，实得 {deficit}"
        );
    }

    /// AMS7997 BEND 24381/104871 evaluates its catalogue SCTO to this narrow
    /// sweep.  E3D exports a visible bend, so the local builder must not return
    /// an empty manifold merely because the sweep is about ten degrees.
    #[test]
    fn narrow_catalogue_circular_torus_is_a_solid() {
        let solid = circ_torus_solid(66.75, 93.45, 9.99786).unwrap();
        assert!(!solid.is_empty());
        assert!(solid.num_tri() > 0);
        assert!(solid.volume() > 0.0);
    }

    /// 回转体与矩形环面是两条独立的实现路径（一条走 PLOO 环、一条走参数），
    /// 但整圈回转一个矩形轮廓就是矩形环面。体积必须**逐位**相等——两边喂给
    /// `part_rev_segments` 的都是外半径，段数一样，网格就该一样。
    ///
    /// 这一条是回转体唯一的权威锚点：它一红，要么是轴摆错了，要么是段数口径分了岔。
    #[test]
    fn a_full_revolution_of_a_rectangle_is_the_rectangular_torus() {
        let (rins, rout, h) = (300.0_f64, 500.0_f64, 200.0_f64);
        // 轮廓在局部 XY：X 是回转轴向（±h/2），Y 是半径（rins..rout）。
        let ring = vec![
            [-h / 2.0, rins],
            [h / 2.0, rins],
            [h / 2.0, rout],
            [-h / 2.0, rout],
        ];
        let revolved = revolve_rings(vec![ring], 360.0).unwrap();
        let torus = rect_torus_solid(rins, rout, h, 360.0).unwrap();
        assert!(
            (revolved.volume() - torus.volume()).abs() / torus.volume() < 1e-12,
            "回转 {} vs 环面 {}",
            revolved.volume(),
            torus.volume()
        );
        // 回转轴是局部 X：轴向铺在 x，半径铺在 yz。
        let bb = revolved.bounding_box().unwrap();
        assert!((bb.min()[0] + h / 2.0).abs() < 1e-9, "{:?}", bb.min());
        assert!((bb.max()[0] - h / 2.0).abs() < 1e-9, "{:?}", bb.max());
        assert!((bb.max()[1] - rout).abs() < 1e-9, "{:?}", bb.max());
    }

    /// 轮廓整条落在轴的 −Y 侧时不许去裁轮廓，而是把标架绕轴翻 180°：
    /// 得到的实体与镜像轮廓同体积，且仍是右手系（法向朝外，体积为正）。
    #[test]
    fn a_profile_on_the_far_side_of_the_axis_is_reframed_not_clipped() {
        let (rins, rout, h) = (300.0_f64, 500.0_f64, 200.0_f64);
        let near = vec![
            [-h / 2.0, rins],
            [h / 2.0, rins],
            [h / 2.0, rout],
            [-h / 2.0, rout],
        ];
        let far: Vec<[f64; 2]> = near.iter().map(|p| [p[0], -p[1]]).collect();
        let a = revolve_rings(vec![near], 360.0).unwrap();
        let b = revolve_rings(vec![far], 360.0).unwrap();
        assert!(b.volume() > 0.0, "翻标架后体积必须为正（法向没朝里）");
        assert!((a.volume() - b.volume()).abs() / a.volume() < 1e-12);
    }

    #[test]
    fn coplanar_negative_still_cuts_through_after_inflation() {
        // 与 gen-model 的回归同款：负体出口面与母体外表面差 0.01mm，
        // 让完 restol 必须挖穿（亏格 1）。让刀在负体局部系做（这里轴对齐，
        // `inflate_about_center` 精确），`subtract_negatives` 自己不再让。
        let block = Manifold::cube(200.0, 100.0, 200.0, true);
        let neg = Manifold::cube(80.0, 109.99, 80.0, true).transform(&dmat4_to_affine4x3(
            DMat4::from_translation(glam::DVec3::new(0.0, -5.005, 0.0)),
        ));
        let cut = subtract_negatives(block, vec![inflate_about_center(&neg, RES_TOL_MM)]);
        assert!(!cut.is_empty());
        assert_eq!(
            cut.genus(),
            1,
            "近共面负体必须挖穿；亏格 0 = 外表面留了一层皮"
        );
    }

    /// ams1112 GWALL `17496/116549` 的最小复刻：负体切边与母体外边**共线**
    /// （存储值差 ~0.02mm），且该边穿过负体的 AABB 中心——逐轴缩放让刀对这种边
    /// 的外推量为 0，差完留一根贴边的针；截面 offset 让刀必须把它吃干净。
    #[test]
    fn flush_diagonal_negative_leaves_no_needle() {
        // 母体：带一条斜边 (600,0)→(0,200) 的四边形，挤 100 高。
        let pos = extrude_rings(
            vec![vec![
                [600.0, 0.0],
                [0.0, 200.0],
                [-700.0, -100.0],
                [600.0, -100.0],
            ]],
            100.0,
        )
        .unwrap();
        // 负体：直角三角形，斜边 (600,0)→(0,199.98) 与母体斜边几乎共线
        //（x=0 端差 0.02mm），且恰穿其 AABB 中心 (300, 99.99)。
        let neg = extrude_rings_inflated(
            vec![vec![[0.0, 0.0], [600.0, 0.0], [0.0, 199.98]]],
            100.0,
            RES_TOL_MM,
        )
        .unwrap();
        let cut = subtract_negatives(pos, vec![neg]);
        assert!(!cut.is_empty());

        // 斜边中段（x ∈ [10, 590]）0.5mm 带内不许剩任何顶点——针状残料的
        // 顶点全落在这条带里。母体保留的两个真角点 (600,0)/(0,200) 在带外。
        let (props, n_props, _tri) = cut.to_mesh_f64();
        let a = glam::DVec2::new(600.0, 0.0);
        let b = glam::DVec2::new(0.0, 200.0);
        let ab = b - a;
        let mut needle = Vec::new();
        for chunk in props.chunks(n_props.max(3)) {
            let p = glam::DVec2::new(chunk[0], chunk[1]);
            if p.x < 10.0 || p.x > 590.0 {
                continue;
            }
            let t = ((p - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0);
            if (p - (a + ab * t)).length() < 0.5 {
                needle.push([p.x, p.y]);
            }
        }
        assert!(needle.is_empty(), "切边带内剩了针状残料顶点: {needle:?}");
    }

    /// 按 Core3D 主初始化的 `facet_tol = 0.5mm` 手算的对照表，与 gen-model
    /// `libgm_discretise::circle_segments_match_libgm_at_the_core3d_default_tolerance`
    /// 同一组数——两份实现分处两个 crate，只能靠同一张表钉住它们不漂。
    #[test]
    fn cylinder_segments_match_the_libgm_table() {
        for (radius, expect) in [
            (1.0, 8), // tol/R 太大 → 步长封顶 45°
            (25.0, 16),
            (100.0, 32),
            (250.0, 52),
            (3_000.0, 176),
            (23_400.0, 484), // RM13 穹顶那个圆；旧的 256 上限会在这里静默截断
        ] {
            let got = cylinder_segments(radius);
            assert_eq!(got, expect, "R={radius} 应当是 {expect} 段，实得 {got}");
        }
    }

    #[test]
    fn unit_sphere_is_a_closed_unit_radius_body() {
        let sphere = unit_sphere_solid(32).expect("unit sphere");
        assert!(!sphere.is_empty());
        assert!(sphere.num_tri() > 0);
        let exact = 4.0 * std::f64::consts::PI / 3.0;
        assert!((sphere.volume() - exact).abs() < exact * 0.05);
    }

    #[test]
    fn segment_rule_is_monotone_and_capped_at_libgms_thousand() {
        assert_eq!(MAX_SEGMENTS, 1000);
        assert_eq!(cylinder_segments(0.1), 8);
        let small = cylinder_segments(50.0);
        let big = cylinder_segments(5_000.0);
        assert!(small >= 8, "small={small}");
        assert!(big >= small);
        assert_eq!(small % 4, 0);
        assert_eq!(big % 4, 0);
        // 半径再大也不许越过 libgm 的 1000。
        assert!(cylinder_segments(1.0e9) <= MAX_SEGMENTS);
    }
}

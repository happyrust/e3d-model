//! 把 [`RvmFile`] 写回 AVEVA RVM 二进制——生成侧导出 RVM 的那一半。
//!
//! 与 [`crate::rvm_read`] 共用同一套结构，所以「写出去再读回来必须一模一样」是可判的
//! （`tests` 里就是这么验的），格式知识也只有一份。
//!
//! # 目标不是字节相同
//!
//! `HEAD` 带时间戳、用户名、产品横幅，f32 舍入两侧不会一致，E3D 的三角化是它自己的。
//! 判据落在**结构与几何**：成员在不在、名字对不对、世界 AABB 差多少。想 `fc /b` 的话
//! 这条路从一开始就走不通，别在这上面花时间。
//!
//! # 写出口径
//!
//! 逐字段的实测依据见 `crate::rvm_read` 的模块文档与
//! `docs/2026-09-04-rvm-export-plan.md` §2；这里只说我们**选了什么**：
//!
//! | 项 | 写什么 |
//! |---|---|
//! | `HEAD` banner | `e3d-model …` —— **不模仿 E3D 的横幅**，免得快照分不清来源 |
//! | 祖先 `CNTB` | 平移全 0（E3D 就是这么写的，逐级累加会乘上嵌套深度） |
//! | 成员 `CNTB` | 元素世界原点 **mm** |
//! | `PRIM` 矩阵 | `R × 0.001` + 世界平移（**m**），顶点留在元素局部系 mm |
//! | `PRIM` bbox | `AABB(matrix3 · v × 1000)`——§2.3 里 228/236 的那一档。**只用这一个帧**， |
//! | | 不复现 `WALL`/`STWALL` 的例外；读侧的自证分支照样认得出来 |
//! | `CNTE` | `00000001 00000002`（**不是** `1,1`；`END:` 才是 `1,1`） |
//! | 尾部 | `END:` 之后直接 16 个 0 字节，不补到 160 边界 |
//!
//! 为什么不偷懒把顶点直接写成世界 mm、矩阵写成 `I × 0.001` + `t = 0`：那样合法、也读得回，
//! 但**丢掉局部帧**。`rvm_compare` 的可信判据是「顶点乘 `world⁻¹` 后在元素局部系比 AABB」，
//! 旋转构件的世界 X/Y AABB 代表不了它自己的端头/厚度方向。

use std::path::Path;

use anyhow::{Context, Result};

use crate::primitive_instance::AnalyticPrimitive;
use crate::rvm_read::{
    Contour, M_TO_MM, Polygon, Primitive, RvmFile, RvmGeometry, RvmHeader, RvmMember,
};

/// `CNTE` 的负载。与 `END:` 的 `1,1` **不是同一个**，抄错了 rvm-rs 照样解得开
/// （`parse_cnte` 不看负载），要等第三方工具才炸。
const CNTE_BODY: [u32; 2] = [1, 2];
const END_BODY: [u32; 2] = [1, 1];
/// 六份 E3D 样本里 `COLR` 恒为这个值；本仓暂不做逐元素配色。
const DEFAULT_COLOR: u32 = 0x8282_8200;

impl RvmFile {
    pub fn write(&self, path: impl AsRef<Path>) -> Result<()> {
        let path = path.as_ref();
        if let Some(dir) = path.parent().filter(|d| !d.as_os_str().is_empty()) {
            std::fs::create_dir_all(dir)
                .with_context(|| format!("建目录失败: {}", dir.display()))?;
        }
        std::fs::write(path, self.to_bytes())
            .with_context(|| format!("写 RVM 失败: {}", path.display()))
    }

    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();

        let mut head = Vec::new();
        push_u32(&mut head, 1);
        push_u32(&mut head, 2);
        for text in [
            &self.header.banner,
            &self.header.note,
            &self.header.date,
            &self.header.user,
            &self.header.encoding,
        ] {
            push_string(&mut head, text);
        }
        chunk(&mut out, "HEAD", &head);

        let mut modl = Vec::new();
        push_u32(&mut modl, 1);
        push_u32(&mut modl, 1);
        push_string(&mut modl, &self.header.model);
        push_string(&mut modl, &self.header.project);
        chunk(&mut out, "MODL", &modl);

        // members 是前序（父一定排在子之前），据此判断每个成员之后要收几个 CNTE。
        let mut open: Vec<usize> = Vec::new();
        for (index, member) in self.members.iter().enumerate() {
            while let Some(&top) = open.last() {
                if member.parent == Some(top) {
                    break;
                }
                close_group(&mut out);
                open.pop();
            }
            let mut cntb = Vec::new();
            push_u32(&mut cntb, 1);
            push_u32(&mut cntb, 2);
            push_string(&mut cntb, &member.name);
            for axis in 0..3 {
                push_f32(&mut cntb, member.translation_mm[axis] as f32);
            }
            push_u32(&mut cntb, member.material);
            chunk(&mut out, "CNTB", &cntb);

            for geometry in &member.geometries {
                chunk(&mut out, &geometry.tag, &geometry_body(geometry));
            }
            open.push(index);
        }
        while open.pop().is_some() {
            close_group(&mut out);
        }

        let mut colr = Vec::new();
        for value in [1, 1, 1, DEFAULT_COLOR] {
            push_u32(&mut colr, value);
        }
        chunk(&mut out, "COLR", &colr);

        let mut end = Vec::new();
        for value in END_BODY {
            push_u32(&mut end, value);
        }
        chunk(&mut out, "END:", &end);

        // 读侧靠「连续 16 个 0 字节」停。
        out.extend_from_slice(&[0u8; 16]);
        out
    }
}

fn close_group(out: &mut Vec<u8>) {
    let mut body = Vec::new();
    for value in CNTE_BODY {
        push_u32(&mut body, value);
    }
    chunk(out, "CNTE", &body);
}

fn geometry_body(geometry: &RvmGeometry) -> Vec<u8> {
    let mut body = Vec::new();
    push_u32(&mut body, 1);
    push_u32(&mut body, geometry.flags);
    push_u32(&mut body, geometry.primitive.kind_id());
    for value in geometry.matrix3 {
        push_f32(&mut body, value as f32);
    }
    for axis in 0..3 {
        // 文件里矩阵平移存的是米。
        push_f32(&mut body, (geometry.translation_mm[axis] / M_TO_MM) as f32);
    }
    for value in geometry.record_bbox {
        push_f32(&mut body, value as f32);
    }
    // `OBST` / `INSU` 的等级字夹在 bbox 与参数之间；本仓只写 `PRIM`，这一支是为回环留的。
    if let Some(level) = geometry.level {
        push_u32(&mut body, level);
    }

    match &geometry.primitive {
        Primitive::Pyramid(p) => push_f32s(&mut body, p),
        Primitive::Box(p) => push_f32s(&mut body, p),
        Primitive::RectangularTorus(p) => push_f32s(&mut body, p),
        Primitive::CircularTorus(p) => push_f32s(&mut body, p),
        Primitive::EllipticalDish(p) => push_f32s(&mut body, p),
        Primitive::SphericalDish(p) => push_f32s(&mut body, p),
        Primitive::Snout(p) => push_f32s(&mut body, p),
        Primitive::Cylinder(p) => push_f32s(&mut body, p),
        Primitive::Sphere(p) => push_f32s(&mut body, p),
        Primitive::Line(p) => push_f32s(&mut body, p),
        Primitive::FacetGroup(polygons) => {
            push_u32(&mut body, polygons.len() as u32);
            for polygon in polygons {
                push_u32(&mut body, polygon.contours.len() as u32);
                for contour in &polygon.contours {
                    push_u32(&mut body, contour.vertices.len() as u32);
                    for (vertex, normal) in contour.vertices.iter().zip(contour.normals.iter()) {
                        push_f32s(&mut body, vertex);
                        push_f32s(&mut body, normal);
                    }
                }
            }
        }
        Primitive::Unknown { params, .. } => push_f32s(&mut body, params),
    }
    body
}

/// 从一份**世界 mm** 的三角网格造一条 FacetGroup 记录。
///
/// `world_to_local` 是元素世界矩阵的逆（列主序 3×3 + 平移 mm）：顶点先落回元素局部系，
/// 旋转再进 `matrix3`，这样导出的 RVM 与 E3D 一样带得出局部帧，`rvm_compare` 的
/// local 帧判据才有得比。
///
/// 一个三角形写成一个 polygon（1 个 contour、3 个顶点），法线取面法线写三份——
/// E3D 也是逐顶点存（实测同一个三角的三个法线相同）。
pub fn facet_group_from_world_triangles(
    rotation: [f64; 9],
    origin_mm: [f64; 3],
    local_vertices_mm: &[[f64; 3]],
    triangles: &[[u32; 3]],
) -> RvmGeometry {
    let polygons = triangles
        .iter()
        .map(|tri| {
            let v = tri.map(|i| local_vertices_mm[i as usize]);
            let normal = face_normal(&v);
            Polygon {
                contours: vec![Contour {
                    vertices: v.to_vec(),
                    normals: vec![normal; 3],
                }],
            }
        })
        .collect();

    let matrix3 = rotation.map(|value| value / M_TO_MM);
    let mut geometry = RvmGeometry {
        tag: "PRIM".to_string(),
        flags: 1,
        primitive: Primitive::FacetGroup(polygons),
        matrix3,
        translation_mm: origin_mm,
        record_bbox: [0.0; 6],
        bbox_frame: crate::rvm_read::BboxFrame::Rotated,
        level: None,
    };
    geometry.record_bbox = rotated_frame_bbox(&geometry);
    geometry
}

/// §2.3 的 rotated 帧：`AABB(matrix3 · v × 1000)`，乘过旋转、未加平移。
fn rotated_frame_bbox(geometry: &RvmGeometry) -> [f64; 6] {
    let m = &geometry.matrix3;
    let mut lo = [f64::MAX; 3];
    let mut hi = [f64::MIN; 3];
    for v in geometry.local_vertices_mm() {
        let r = [
            (m[0] * v[0] + m[3] * v[1] + m[6] * v[2]) * M_TO_MM,
            (m[1] * v[0] + m[4] * v[1] + m[7] * v[2]) * M_TO_MM,
            (m[2] * v[0] + m[5] * v[1] + m[8] * v[2]) * M_TO_MM,
        ];
        for axis in 0..3 {
            lo[axis] = lo[axis].min(r[axis]);
            hi[axis] = hi[axis].max(r[axis]);
        }
    }
    if lo[0] > hi[0] {
        return [0.0; 6];
    }
    [lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]]
}

fn face_normal(v: &[[f64; 3]; 3]) -> [f64; 3] {
    let a = [v[1][0] - v[0][0], v[1][1] - v[0][1], v[1][2] - v[0][2]];
    let b = [v[2][0] - v[0][0], v[2][1] - v[0][1], v[2][2] - v[0][2]];
    let n = [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ];
    let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if len > 0.0 {
        n.map(|c| c / len)
    } else {
        [0.0, 0.0, 1.0]
    }
}

/// 弦高（mm）：半径 `radius` 的圆弧被一段圆心角 `step` 的弦切下来时，弧与弦之间的最大缺口
/// `r·(1 − cos(step/2))`。整圆 n 等分就是熟悉的 `r·(1 − cos(π/n))`。
///
/// 这是解析图元与它的离散网格之间**唯一**的系统差：网格顶点落在曲面上、边落在曲面内侧，
/// 所以任何方向上网格的支撑值都比解析面小，最多小这一个弦高。
///
/// 为什么不拿定段数的那个容差当常数（[`crate::solid::cylinder_segments`] 是按弦高 ≤ 0.5 mm
/// 定段数的）：同一份网格可能出自不同口径——曲面 0.5 mm、轮廓
/// [`PROFILE_CHORD_TOL_MM`](crate::discretise::PROFILE_CHORD_TOL_MM)、SNOU 5 mm，
/// 而且 45° 封顶一生效实际弦高就比容差小一截。从网格自己的段数回算，不用猜它出自哪一档。
/// 端盖多边形至少要有几个顶点，才可能是一个**离散出来的圆**。
///
/// 值来自 libgeom 自己的规则：`d2_numberOfSegmentsForCircle`（`0x1002BA70`）把步长
/// 封顶在 45°，所以整圆最少 8 段，而且还要向上取到 4 的倍数——见
/// [`crate::discretise::circle_segments_uncapped`] 与 `cylinder_segments(1.0) == 8`。
/// 我们自己的每一条曲面口径（曲面 0.5 mm、轮廓 10 mm、SNOU 5 mm）都从那个公式出，
/// 没有一条能产出 4 段或 6 段的圆。
///
/// **没有下限时这道自校闸是形同虚设的**，因为它放行的弦高与半径成正比：
/// 4 段时 `r(1 − cos45°) = 0.293 r`，正好是它本该拦住的那个量级。2026-09-14 实测，
/// SLYK `/石楼装卸车-SUPP1` 的 17 块支架底板（E3D 写 `Pyramid`，230×230 → 200×200）
/// 被认成了 `Snout`：半径取了正方形的外接圆 162.63 mm（真值 ±115），
/// 而 4 段允许的弦高恰好是 47.63 mm，于是 47.63 mm 的错误**一字不差地**从闸门下走过去。
/// 同一条缝还让 35 只六角螺母（E3D 写 `FacetGroup`）写成了圆柱。
///
/// 卡住之后这两族退回 `FacetGroup`——面片带的是真顶点，宁可文件大一点。
pub const MIN_CIRCLE_SEGMENTS: usize = crate::discretise::MIN_CIRCLE_SEGMENTS as usize;

pub fn chord_sagitta_mm(radius: f64, step: f64) -> f64 {
    if !(radius > 0.0) || !(step > 0.0) || !step.is_finite() {
        return 0.0;
    }
    radius * (1.0 - (step / 2.0).cos())
}

/// 一个解析候选：图元记录，加上它与源网格之间允许的离散化缺口。
///
/// 两样东西必须一起走。网格是内接的、解析面是外接的，「这个候选是不是这块网格」
/// 就得按 [`chord_sagitta_mm`] 的尺度问；把图元单独递出去，下一个人只能拿个常数去卡它
/// ——那正是 2026-09-06 之前 `1RCV0101` 的 14 根真圆柱被退回面片的原因：它们的
/// AABB 差 0.06–0.35 mm，全都是自己那 8/16/28 段离散该有的弦高，闸门却按 0.01 mm 判。
#[derive(Debug, Clone)]
pub struct AnalyticCandidate {
    pub geometry: RvmGeometry,
    /// 网格与解析面之间系统差的上界（mm）。网格本身就是精确形体时为 0。
    pub sagitta_mm: f64,
}

impl AnalyticCandidate {
    /// 自校闸：候选算出来的世界 AABB 必须与网格实算的一致，否则调用方退回 FacetGroup。
    ///
    /// 规范体约定写错了照样能写出一份解析得开、几何却是错的 RVM，这种错不该靠人眼发现。
    /// 闸门留得住它：约定错的量级是半径或半高（几十上百 mm），而这里放行的只有
    /// `tol_mm`（f32 落盘那一档）加一个弦高（实测 ≤ 0.5 mm），两者差两三个数量级。
    pub fn matches_mesh_aabb(&self, mesh_lo: [f64; 3], mesh_hi: [f64; 3], tol_mm: f64) -> bool {
        let slack = tol_mm + self.sagitta_mm;
        match self.world_aabb_mm() {
            Some(a) => (0..3).all(|axis| {
                (a[axis] - mesh_lo[axis]).abs() <= slack
                    && (a[axis + 3] - mesh_hi[axis]).abs() <= slack
            }),
            None => false,
        }
    }

    /// 候选记录的世界 AABB（mm）。
    ///
    /// 球单独算：球的世界盒就是球心 ± r，与记录带的旋转无关，不必绕采样点。
    /// 读侧 [`crate::rvm_read::Primitive::local_hull_points_mm`] 现在也按球面逐圈采
    /// （早先是规范盒的八个角，元素转 45° 时世界盒多出 `(√2 − 1)·r`，这一格当年就是为绕开它
    /// 才单独写的），但采样点撑不到极值、每侧短 7.5e-5·r；闸门这一侧直接给精确值。
    pub fn world_aabb_mm(&self) -> Option<[f64; 6]> {
        match self.geometry.primitive {
            Primitive::Sphere([r]) => {
                let c = self.geometry.translation_mm;
                Some([c[0] - r, c[1] - r, c[2] - r, c[0] + r, c[1] + r, c[2] + r])
            }
            _ => self.geometry.world_aabb_mm(),
        }
    }
}

/// 可复用基本体 → RVM 解析图元（`Box(2)` / `Cylinder(8)`）。凑不上就返回 None，由调用方退回 FacetGroup。
///
/// 三套规范体各自的约定（`solid.rs` / `primitive_instance.rs`，都是对着 E3D RVM 面片坐实过的）：
///
/// | | 规范体 | RVM 要的 |
/// |---|---|---|
/// | `BoxV1` | `Manifold::cube(1,1,1, true)`，**居中**单位立方 | `Box`：三个边长，居中 |
/// | `CylinderV3` | `Manifold::cylinder(1,1,1, n, false)`，半径 1、**Z ∈ [0,1] 不居中** | `Cylinder`：半径 + 高，**沿 Z 居中** |
///
/// 所以圆柱要把原点从底面挪到中点（沿它自己的局部 Z 走 h/2），盒子不用。
///
/// **调用方必须校验**：走 [`AnalyticCandidate::matches_mesh_aabb`] 对一遍网格，对不上就别用。
/// 约定写错了照样能写出一个解析得开的 RVM，只是几何是错的——这种错不该靠人眼发现。
pub fn analytic_from_instance(
    world: &glam::DMat4,
    instance: &crate::primitive_instance::PrimitiveInstance,
) -> Option<AnalyticCandidate> {
    use crate::primitive_instance::PrimitiveMeshKey;

    let (scale, rotation, translation) = instance.local_transform.to_scale_rotation_translation();
    if !scale.is_finite() || !translation.is_finite() || scale.min_element() <= 0.0 {
        return None;
    }

    let (primitive, centre_offset, sagitta_mm) = match instance.key {
        // 规范立方是精确形体，没有离散化缺口。
        PrimitiveMeshKey::BoxV1 => (
            Primitive::Box([scale.x, scale.y, scale.z]),
            glam::DVec3::ZERO,
            0.0,
        ),
        PrimitiveMeshKey::CylinderV3 { segments } => {
            // 半径来自 X/Y 两轴，非各向同性缩放就不是圆柱了，交给 FacetGroup。
            if (scale.x - scale.y).abs() > 1e-9 * scale.x.max(1.0) {
                return None;
            }
            (
                Primitive::Cylinder([scale.x, scale.z]),
                glam::DVec3::new(0.0, 0.0, scale.z / 2.0),
                // 段数就在键里（规范体是 `Manifold::cylinder(…, segments, …)`），
                // 半径是缩放后的 `scale.x`——弦高按世界尺寸算才是 mm。
                chord_sagitta_mm(scale.x, std::f64::consts::TAU / f64::from(segments)),
            )
        }
    };

    // 局部旋转叠进世界旋转；原点是「规范体中心」在世界里的位置。
    let local_centre = translation + rotation * centre_offset;
    let placed = *world * glam::DMat4::from_rotation_translation(rotation, local_centre);
    let cols = placed.to_cols_array();
    let matrix3 = [
        cols[0], cols[1], cols[2], cols[4], cols[5], cols[6], cols[8], cols[9], cols[10],
    ]
    .map(|value| value / M_TO_MM);

    let mut geometry = RvmGeometry {
        tag: "PRIM".to_string(),
        flags: 1,
        primitive,
        matrix3,
        translation_mm: [cols[12], cols[13], cols[14]],
        record_bbox: [0.0; 6],
        bbox_frame: crate::rvm_read::BboxFrame::Local,
        level: None,
    };
    // 解析图元的记录 bbox 走**局部帧**——E3D 的 196/196 都是这一档。
    geometry.record_bbox = geometry.primitive.local_aabb_mm()?;
    Some(AnalyticCandidate {
        geometry,
        sagitta_mm,
    })
}

/// 设计图元的解析描述（[`AnalyticPrimitive`]）→ RVM 记录。目前只有 SLCY → 9 参 `Snout`：
/// E3D 的 RVM 驱动对 SLCY 走 DRSNO，写 `Snout[r, r, HEIG, 0, 0, XBSH, YBSH, XTSH, YTSH]`，
/// 角度转弧度、符号照抄；记录帧 = 元素世界帧、原点 = 元素原点——与
/// [`crate::solid::slope_ended_cylinder_solid`] 的局部帧（轴在 Z、沿 Z 居中）正好是同一个，
/// 所以世界矩阵直接落记录。取证：`/盘梯底部梯子`（21909/51286）表现级 6 基准里的 8 只 SLCY，
/// 矩阵 = 属主的旋转、平移 = 元素 POS 的世界坐标、四个角逐字段对上设计属性。
///
/// 网格识别器认不出斜切圆柱（两端不是圆、也不平行），这一族只能从属性直出；**调用方必须过
/// [`AnalyticCandidate::matches_mesh_aabb`]**，这里只是参数的搬运。离散化缺口：斜切面上网格
/// 顶点与解析极值之间的缺口是径向弦高再乘 `√(tan²xs + tan²ys)`（z 方向随斜率放大），
/// 两端取大者、再与 1 取大——不斜切时退回普通圆柱那一档。
pub fn analytic_from_primitive(
    world: &glam::DMat4,
    primitive: &AnalyticPrimitive,
) -> Option<AnalyticCandidate> {
    match *primitive {
        AnalyticPrimitive::SlopedCylinder {
            radius,
            height,
            xtop_deg,
            ytop_deg,
            xbot_deg,
            ybot_deg,
            segments,
        } => {
            if !(radius > 0.0 && height > 0.0) || segments < 3 {
                return None;
            }
            let slope = |x_deg: f64, y_deg: f64| {
                (x_deg.to_radians().tan().powi(2) + y_deg.to_radians().tan().powi(2)).sqrt()
            };
            let sagitta_mm = chord_sagitta_mm(radius, std::f64::consts::TAU / f64::from(segments))
                * slope(xtop_deg, ytop_deg)
                    .max(slope(xbot_deg, ybot_deg))
                    .max(1.0);
            let primitive = Primitive::Snout([
                radius,
                radius,
                height,
                0.0,
                0.0,
                xbot_deg.to_radians(),
                ybot_deg.to_radians(),
                xtop_deg.to_radians(),
                ytop_deg.to_radians(),
            ]);
            let cols = world.to_cols_array();
            let matrix3 = [
                cols[0], cols[1], cols[2], cols[4], cols[5], cols[6], cols[8], cols[9], cols[10],
            ]
            .map(|value| value / M_TO_MM);
            let mut geometry = RvmGeometry {
                tag: "PRIM".to_string(),
                flags: 1,
                primitive,
                matrix3,
                translation_mm: [cols[12], cols[13], cols[14]],
                record_bbox: [0.0; 6],
                bbox_frame: crate::rvm_read::BboxFrame::Local,
                level: None,
            };
            // 记录 bbox 是**真几何**的局部盒（带斜切），不抄 E3D 那只度当弧度的盒
            // （见 [`Primitive::declared_box_mm`]）：那只盒能比实体还小——顶端 −67.5° 时它的
            // 盒顶只到 65.8 mm，几何却到 540.8 mm——抄过来就是把体裁掉，按 bbox 取舍的下游会
            // 直接丢件。读侧两只盒都认得（判帧时真几何与那只盒各试一遍），Local 帧不受影响。
            geometry.record_bbox = geometry.primitive.local_aabb_mm()?;
            Some(AnalyticCandidate {
                geometry,
                sagitta_mm,
            })
        }
    }
}

/// 网格本身就是一只轴对齐长方体时，给出它的三个边长（局部 mm）。否则 None。
///
/// **为什么不走 `PrimitiveInstance`**：目录件那一支（`catalogue.rs` 的 HVAC 矩形基本体）
/// **刻意**把 `instance` 置成 `None`——内核对那只箱子施加了两次局部变换，折叠成一个矩阵后
/// 与内核差几个 ULP（实测最大 1.5e-11 mm），持久化那一侧要位一致，宁可少一族实例化。
/// 那个决定是对的，不该为 RVM 去改：**1.5e-11 mm 比我们写进 f32 的舍入低 12 个数量级**，
/// 对导出毫无影响，但对持久化是真问题。所以 RVM 这一侧自己认几何，不去动那条契约。
///
/// 判据：去重后正好 8 个顶点、12 个三角面，且顶点集合恰好是它自己 AABB 的八个角。
/// 满足这三条的网格**就是**那只盒子，不是「像」——调用方的 AABB 自校闸再兜一道。
///
/// 21 条样本里这一支捞回 39 件 `FTUB`：E3D 写 `Box`，我们本来写面片。
pub fn box_from_local_mesh(vertices: &[[f64; 3]], triangles: &[[u32; 3]]) -> Option<[f64; 3]> {
    if triangles.len() != 12 {
        return None;
    }
    let mut unique: Vec<[f64; 3]> = Vec::with_capacity(8);
    for v in vertices {
        if !unique.iter().any(|u| same_point(u, v)) {
            unique.push(*v);
            if unique.len() > 8 {
                return None;
            }
        }
    }
    if unique.len() != 8 {
        return None;
    }
    let lo = [0, 1, 2].map(|a| unique.iter().fold(f64::MAX, |m, v| m.min(v[a])));
    let hi = [0, 1, 2].map(|a| unique.iter().fold(f64::MIN, |m, v| m.max(v[a])));
    let lengths = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
    if lengths.iter().any(|l| !(*l > 0.0)) {
        return None;
    }
    // 每个顶点都必须落在某个角上；差一个就说明它不是盒子（斜切、倒角、开孔都在这儿被挡下）。
    let corners = [0usize, 1, 2, 3, 4, 5, 6, 7].map(|i| {
        [
            if i & 1 == 0 { lo[0] } else { hi[0] },
            if i & 2 == 0 { lo[1] } else { hi[1] },
            if i & 4 == 0 { lo[2] } else { hi[2] },
        ]
    });
    if !unique
        .iter()
        .all(|v| corners.iter().any(|c| same_point(c, v)))
    {
        return None;
    }
    // 盒心不在局部原点时，得由调用方把偏移折进矩阵平移；这里只报边长与中心。
    Some(lengths)
}

/// 盒面判定的点重合容差：CSG 出来的顶点是 f64 算到底的，同一个角不会差过这个量级。
fn same_point(a: &[f64; 3], b: &[f64; 3]) -> bool {
    (0..3).all(|axis| (a[axis] - b[axis]).abs() <= 1e-6)
}

/// 轴向候选：端盖是平面多边形，**它的法向就是轴向**，而端盖法向一定出现在三角面法向里。
/// 方向不分正负，统一朝一侧（[`canonical_axis_sign`]），省一半候选。
fn face_normal_axes(points: &[glam::DVec3], triangles: &[[u32; 3]]) -> Vec<glam::DVec3> {
    let mut axes: Vec<glam::DVec3> = Vec::new();
    for tri in triangles {
        let [a, b, c] = tri.map(|i| points[i as usize]);
        let normal = (b - a).cross(c - a);
        if normal.length_squared() <= 0.0 {
            continue;
        }
        let normal = canonical_axis_sign(normal.normalize());
        if !axes
            .iter()
            .any(|existing| (*existing - normal).length() < 1e-9)
        {
            axes.push(normal);
        }
    }
    axes
}

/// 一个分量小到这个量级就当 0，轮到下一个分量定符号。
///
/// 端盖法向是 `(b − a) × (c − a)` 算出来的：轴**正好**躺在某个坐标面内时，理论为 0 的那个
/// 分量实际是 ±1e-16 级的舍入噪声，拿它跟 0 比大小，符号就由噪声定。ams7999
/// `/1WCC318VN` 手轮的两根辐条 LCYL（`PAXI P8` / `P9`，都在构件局部 XY 面内）就是这样
/// 一根与目录轴同向、一根反向地写进 RVM——几何对称看不出来，RVM 矩阵的 Z 列却与 E3D
/// 的 DRCYL（直接写目录轴）不同号（`plant-model-5/outputs/rvm-check-20261010/README.md`
/// 「段 3」）。
const AXIS_SIGN_TOL: f64 = 1e-9;

/// 把一个无所谓正负的轴向统一到一侧：先看 z，|z| 在 [`AXIS_SIGN_TOL`] 内再看 y，再看 x；
/// 第一个有分量的若为负就整体取反。
///
/// 这只保证**同一根轴每次给同一个符号**，不保证与 E3D 同号——E3D 写的是目录图元自己的
/// 轴，要逐列对上得把「出身」那条通道里的轴向带到写出器（happyrust/e3d-model#1）。
fn canonical_axis_sign(axis: glam::DVec3) -> glam::DVec3 {
    let leading = [axis.z, axis.y, axis.x]
        .into_iter()
        .find(|component| component.abs() > AXIS_SIGN_TOL);
    match leading {
        Some(component) if component < 0.0 => -axis,
        _ => axis,
    }
}

/// 顶点沿 `axis` 恰好落在两个平面上时给出 `(低档, 高档)` 投影值（相对 `origin`）。
/// 这是「两个端盖」这句话的判据：多一档就说明侧面上还有顶点，那就不是圆柱也不是圆台。
fn two_levels_along(
    points: &[glam::DVec3],
    axis: glam::DVec3,
    origin: glam::DVec3,
) -> Option<(f64, f64)> {
    let mut levels: Vec<f64> = Vec::with_capacity(2);
    for p in points {
        let t = (*p - origin).dot(axis);
        if !levels.iter().any(|l| (l - t).abs() <= 1e-6) {
            levels.push(t);
            if levels.len() > 2 {
                return None;
            }
        }
    }
    match levels[..] {
        [a, b] => Some((a.min(b), a.max(b))),
        _ => None,
    }
}

/// 落在 `axis` 上 `level` 那一档的顶点共圆时，给出 `(圆心, 半径, 顶点数)`。
fn circular_cap(
    points: &[glam::DVec3],
    axis: glam::DVec3,
    origin: glam::DVec3,
    level: f64,
) -> Option<(glam::DVec3, f64, usize)> {
    let mut unique: Vec<glam::DVec3> = Vec::new();
    for p in points {
        if ((*p - origin).dot(axis) - level).abs() <= 1e-6
            && !unique.iter().any(|q| (*q - *p).length() <= 1e-6)
        {
            unique.push(*p);
        }
    }
    if unique.len() < 3 {
        return None;
    }
    let centre = unique.iter().copied().sum::<glam::DVec3>() / unique.len() as f64;
    let radius = (unique[0] - centre).length();
    if !(radius > 0.0)
        || unique
            .iter()
            .any(|p| ((*p - centre).length() - radius).abs() > 1e-6)
    {
        return None;
    }
    Some((centre, radius, unique.len()))
}

/// 落在 `axis` 上 `level` 那一档的顶点去重后只剩**一个**时给出它——圆锥的锥顶。
/// 一个都没有、或不止一个，都是 None。
fn lone_vertex_at(
    points: &[glam::DVec3],
    axis: glam::DVec3,
    origin: glam::DVec3,
    level: f64,
) -> Option<glam::DVec3> {
    let mut lone: Option<glam::DVec3> = None;
    for p in points {
        if ((*p - origin).dot(axis) - level).abs() > 1e-6 {
            continue;
        }
        match lone {
            Some(q) if (q - *p).length() <= 1e-6 => {}
            Some(_) => return None,
            None => lone = Some(*p),
        }
    }
    lone
}

/// 一根圆柱的识别结果，全在局部系 mm。
#[derive(Debug, Clone, Copy)]
pub struct RecognisedCylinder {
    pub radius: f64,
    pub height: f64,
    /// 轴向（RVM 的局部 +Z）。
    pub axis: glam::DVec3,
    /// 轴心（两个端盖的中点）。
    pub centre: glam::DVec3,
    /// 端盖多边形的边数，也就是整圆被分成了几段。
    pub segments: usize,
}

impl RecognisedCylinder {
    /// 圆柱面与它的内接棱柱之间的缺口，见 [`chord_sagitta_mm`]。
    pub fn sagitta_mm(&self) -> f64 {
        chord_sagitta_mm(self.radius, std::f64::consts::TAU / self.segments as f64)
    }
}

/// 网格是一根正棱柱（= 离散圆柱）时把它解出来。
///
/// 只在**逐组件**这一层调用才说得准：一个组件就是一个目录图元，多个图元合成的网格不该被认成圆柱。
///
/// 认法：圆柱的两个端盖是平面 N 边形，**端盖法向就是轴向**，而端盖法向一定出现在三角面法向里。
/// 所以把去重后的面法向逐个当轴试一遍，满足下面三条的那个就是：
///
/// 1. 顶点沿该轴的投影只有**两个**取值（两个端盖，间距 = 高）；
/// 2. 顶点到轴线的距离**恒定**（= 半径）；
/// 3. 半径与高都为正。
///
/// 三条同时成立的网格就是那根圆柱。写错了还有调用方的 AABB 自校闸兜底。
pub fn cylinder_from_local_mesh(
    vertices: &[[f64; 3]],
    triangles: &[[u32; 3]],
) -> Option<RecognisedCylinder> {
    use glam::DVec3;
    if triangles.len() < 8 || vertices.len() < 6 {
        return None;
    }
    let points: Vec<DVec3> = vertices.iter().copied().map(DVec3::from_array).collect();
    let centroid = points.iter().copied().sum::<DVec3>() / points.len() as f64;

    for axis in face_normal_axes(&points, triangles) {
        // 1. 沿轴只有两个投影值。
        let Some((low, high)) = two_levels_along(&points, axis, centroid) else {
            continue;
        };
        let height = high - low;
        if !(height > 0.0) {
            continue;
        }
        // 2. 到轴线的距离恒定。
        let radial = |p: &DVec3| {
            let d = *p - centroid;
            (d - axis * d.dot(axis)).length()
        };
        let radius = radial(&points[0]);
        if !(radius > 0.0) || points.iter().any(|p| (radial(p) - radius).abs() > 1e-6) {
            continue;
        }
        let centre = centroid + axis * (low + high) / 2.0;
        // 段数 = 落在一侧端盖上的不同顶点数。自校闸要拿它回算弦高，
        // 「网格比解析面细多少」这件事只有这块网格自己说得准。
        let (_, _, segments) = circular_cap(&points, axis, centroid, low)?;
        if segments < MIN_CIRCLE_SEGMENTS {
            continue; // 六棱柱不是圆柱，见 [`MIN_CIRCLE_SEGMENTS`]。
        }
        return Some(RecognisedCylinder {
            radius,
            height,
            axis,
            centre,
            segments,
        });
    }
    None
}

/// 一段圆台（`Snout`）的识别结果，全在局部系 mm。
#[derive(Debug, Clone, Copy)]
pub struct RecognisedSnout {
    /// `axis` 负向那一端的半径。
    pub bottom_radius: f64,
    /// `axis` 正向那一端的半径。
    pub top_radius: f64,
    pub height: f64,
    /// 顶面中心相对底面中心的**横向**偏移长度。局部 +X 就取这个方向，于是 RVM 的 `YOFF` 恒为 0。
    pub offset: f64,
    /// 两个盖心的中点。
    pub centre: glam::DVec3,
    /// 底 → 顶（RVM 的局部 +Z）。
    pub axis: glam::DVec3,
    /// 偏移方向（RVM 的局部 +X）。
    pub start: glam::DVec3,
    /// 端盖多边形的边数，两端相同。
    pub segments: usize,
}

impl RecognisedSnout {
    /// 锥面与它的内接棱台之间的缺口。粗的那一端说了算，见 [`chord_sagitta_mm`]。
    pub fn sagitta_mm(&self) -> f64 {
        chord_sagitta_mm(
            self.bottom_radius.max(self.top_radius),
            std::f64::consts::TAU / self.segments as f64,
        )
    }
}

/// 网格是一段圆台（= 离散 `Snout`）时把它解出来。
///
/// 认法与 [`cylinder_from_local_mesh`] 同源，只是两端各有各的半径、各有各的盖心：
///
/// 1. 顶点沿候选轴的投影只有**两个**取值（两个端盖，间距 = 高）；
/// 2. 每一档的顶点各自共圆，给出该端的半径与盖心；
/// 3. 两端盖心的横向错位就是 RVM 的 `XOFF`（`YOFF` 由取轴方式吃掉）。
///
/// 三条成立时顶点集合恰好是两个圆，闭合网格只能是它们张成的那只圆台。
///
/// **圆锥**也在这儿认：一端是整圈端盖、另一端去重后只剩锥顶一个点（第 2 条对那一端
/// 退化成「半径 0」）。E3D 把目录 SCON 写成 `Snout`，锥顶那头半径 0 且在**底**
/// （`gm_CreateSnout(0, PDIA/2, PDIS)`，见 `catalogue.rs` 的 `scone_frame`），这里照样把
/// 锥顶放到底端，记录与 E3D 同一个朝向。2026-09-14 之前圆锥在这儿认不出来，落到后面的
/// [`spherical_dish_from_local_mesh`]——底环加锥顶恰好共一只球——7 只 PCOM 的圆锥被写成了碟。
///
/// 两种情况**故意**认不出来，退回面片：两端等径且不偏心的是圆柱，`Cylinder` 更贴 E3D；
/// 带剪切的端面不垂直于轴，第 1 条就过不去。
pub fn snout_from_local_mesh(
    vertices: &[[f64; 3]],
    triangles: &[[u32; 3]],
) -> Option<RecognisedSnout> {
    snout_from_local_mesh_inner(vertices, triangles, false)
}

/// 同 [`snout_from_local_mesh`]，但**不**把两端等径且不偏心的那只让给 `Cylinder`。
///
/// 出身是 `LSNO` / `SCON` 时用这一条：E3D 的 DRSNO 不看圆台退化成什么，照写 `Snout`
/// ——按长相认时让给 `Cylinder` 是对的（长得像圆柱的多半就是 SCYL），按出身认时不是。
pub fn snout_from_local_mesh_including_cylinders(
    vertices: &[[f64; 3]],
    triangles: &[[u32; 3]],
) -> Option<RecognisedSnout> {
    snout_from_local_mesh_inner(vertices, triangles, true)
}

fn snout_from_local_mesh_inner(
    vertices: &[[f64; 3]],
    triangles: &[[u32; 3]],
    keep_cylinders: bool,
) -> Option<RecognisedSnout> {
    use glam::DVec3;
    if triangles.len() < 8 || vertices.len() < 6 {
        return None;
    }
    let points: Vec<DVec3> = vertices.iter().copied().map(DVec3::from_array).collect();
    let centroid = points.iter().copied().sum::<DVec3>() / points.len() as f64;

    for axis in face_normal_axes(&points, triangles) {
        let Some((low, high)) = two_levels_along(&points, axis, centroid) else {
            continue;
        };
        let height = high - low;
        if !(height > 0.0) {
            continue;
        }
        let bottom = circular_cap(&points, axis, centroid, low);
        let top = circular_cap(&points, axis, centroid, high);
        // 圆锥：锥顶那一档不是圆盘，是孤零零一个点。轴的正负由面法向定，锥顶落在哪一端
        // 都可能；统一把它翻到底端（`-axis`），让 `bottom_radius == 0`。
        let (
            axis,
            (bottom_centre, bottom_radius, segments),
            (top_centre, top_radius, top_segments),
        ) = match (bottom, top) {
            (Some(bottom), Some(top)) => (axis, bottom, top),
            (Some(cap), None) => match lone_vertex_at(&points, axis, centroid, high) {
                Some(apex) => (-axis, (apex, 0.0, cap.2), cap),
                None => continue,
            },
            (None, Some(cap)) => match lone_vertex_at(&points, axis, centroid, low) {
                Some(apex) => (axis, (apex, 0.0, cap.2), cap),
                None => continue,
            },
            (None, None) => continue,
        };
        if segments != top_segments {
            continue; // 两端段数不同，不是一次回转扫出来的。
        }
        if segments < MIN_CIRCLE_SEGMENTS {
            continue; // 四棱台不是圆台，见 [`MIN_CIRCLE_SEGMENTS`]。
        }
        let arm = top_centre - bottom_centre;
        let lateral = arm - axis * arm.dot(axis);
        let offset = lateral.length();
        if !keep_cylinders && (bottom_radius - top_radius).abs() <= 1e-9 && offset <= 1e-9 {
            return None; // 这是圆柱，让 `cylinder_from_local_mesh` 去接。
        }
        return Some(RecognisedSnout {
            bottom_radius,
            top_radius,
            height,
            offset,
            centre: (bottom_centre + top_centre) / 2.0,
            axis,
            start: if offset > 1e-9 {
                lateral / offset
            } else {
                axis.any_orthonormal_vector()
            },
            segments,
        });
    }
    None
}

/// 一段圆环（弯头）的识别结果，全在局部系 mm。
#[derive(Debug, Clone, Copy)]
pub struct RecognisedTorus {
    /// 扫掠半径：扫掠轴到管心的距离，对应 RVM 的 `offset`。
    pub offset: f64,
    /// 管截面半径，对应 RVM 的 `radius`。
    pub radius: f64,
    /// 包角（弧度）。
    pub angle: f64,
    /// 扫掠中心。
    pub centre: glam::DVec3,
    /// 扫掠轴（RVM 的局部 +Z）。
    pub axis: glam::DVec3,
    /// 起始管心方向（RVM 的局部 +X）。
    pub start: glam::DVec3,
    /// 管截面多边形的边数（整圆分几段）。
    pub tube_segments: usize,
    /// 扫掠折线的段数：顶点分布在 `sweep_steps + 1` 圈截面上。
    pub sweep_steps: usize,
}

impl RecognisedTorus {
    /// 圆环面与它的内接网格之间的缺口。
    ///
    /// 两处离散**都**要算：管截面是内接多边形（缺口按管半径），扫掠是内接折线
    /// （缺口按扫掠半径）。两者能在同一个方向上叠加，所以取和不取大。
    pub fn sagitta_mm(&self) -> f64 {
        chord_sagitta_mm(
            self.radius,
            std::f64::consts::TAU / self.tube_segments as f64,
        ) + chord_sagitta_mm(self.offset, self.angle / self.sweep_steps as f64)
    }
}

/// 网格是一段**部分圆环**（弯头 / 弯管）时把它解出来。整圈圆环没有端盖，认不出来，返回 None。
///
/// 认法走**两个端盖**：部分圆环的两头是两张平面圆盘，半径就是管截面半径 `r`。
///
/// 1. 按面法向分组，找出两组「法向相同、顶点共圆」的三角面——那就是端盖；
/// 2. 扫掠轴 `a = n₁ × n₂`（轴同时垂直于两个端盖法向）；
/// 3. 端盖中心 `c₁ c₂` 各自沿盖内的径向 `n × a` 走，两条线的交点就是扫掠中心 `O`；
/// 4. `offset = |O − c₁|`，`angle` = `(c₁−O)` 与 `(c₂−O)` 的夹角。
///
/// 每一步都能被下一步证伪（两个 `offset` 必须相等、所有顶点到管心圆的距离必须都是 `r`），
/// 对不上就返回 None。再加调用方的 AABB 自校闸，认错了写不出去。
pub fn torus_from_local_mesh(
    vertices: &[[f64; 3]],
    triangles: &[[u32; 3]],
) -> Option<RecognisedTorus> {
    use glam::DVec3;
    let points: Vec<DVec3> = vertices.iter().copied().map(DVec3::from_array).collect();
    if points.len() < 12 || triangles.len() < 12 {
        return None;
    }

    // 按面法向分组。端盖是平面圆盘，它的三角面共享同一个法向且数量可观。
    let mut groups: Vec<(DVec3, Vec<[u32; 3]>)> = Vec::new();
    for tri in triangles {
        let [a, b, c] = tri.map(|i| points[i as usize]);
        let normal = (b - a).cross(c - a);
        if normal.length_squared() <= 0.0 {
            continue;
        }
        let normal = normal.normalize();
        match groups
            .iter_mut()
            .find(|(n, _)| (*n - normal).length() < 1e-7)
        {
            Some((_, members)) => members.push(*tri),
            None => groups.push((normal, vec![*tri])),
        }
    }
    // 取三角面最多的几组，每组再按共顶点连通性拆开。
    //
    // **为什么要拆**：90° 弯头的两个端盖法向相反、各成一组，直接取前两组就行；但包角
    // 180° 时起点盖背朝扫掠方向、终点盖顺着，转过半圈两者**共面且同向**，会落进同一组。
    // 合在一起 16 个顶点不共圆，整组就被判成「不是端盖」——`1RCV002VP` 的两条 π 弯
    // 因此一个盖也认不出来。两个盖不共顶点，按连通性拆得开。
    groups.sort_by_key(|(_, members)| std::cmp::Reverse(members.len()));
    let caps: Vec<(DVec3, DVec3, f64, usize)> = groups
        .iter()
        .take(4)
        .flat_map(|(normal, members)| {
            connected_triangle_groups(members)
                .into_iter()
                .map(|indices| (*normal, indices))
        })
        .filter_map(|(normal, indices)| {
            let mut unique: Vec<DVec3> = Vec::new();
            for index in &indices {
                let p = points[*index as usize];
                if !unique.iter().any(|u| (*u - p).length() < 1e-9) {
                    unique.push(p);
                }
            }
            if unique.len() < 6 {
                return None;
            }
            let centre = unique.iter().copied().sum::<DVec3>() / unique.len() as f64;
            let radius = (unique[0] - centre).length();
            if !(radius > 0.0)
                || unique
                    .iter()
                    .any(|p| ((*p - centre).length() - radius).abs() > 1e-6)
            {
                return None; // 不共圆，不是端盖。
            }
            // 端盖上的顶点数 = 管截面多边形的边数，自校闸拿它回算弦高。
            Some((normal, centre, radius, unique.len()))
        })
        .collect();
    if caps.len() < 2 {
        return None;
    }
    let (n1, c1, r1, tube_segments) = caps[0];
    let (n2, c2, r2, _) = caps[1];
    if (r1 - r2).abs() > 1e-6 {
        return None;
    }
    let radius = r1;

    // 两个端盖法向都垂直于扫掠轴，所以一般情况下叉乘就是轴。**半圈弯头例外**：包角 180°
    // 时两个端盖反向平行，叉乘退化成 0——`1RCV002VP` 的两条 π 弯就是这么被当成「直管」
    // 丢掉的。这时改用另一组正交关系：轴同时垂直于端盖法向与两个盖心的连线（半圈时盖心
    // 连线过扫掠中心，必与轴垂直）。**直管在这一支照样退化**：它的盖心连线与法向共线，
    // 叉乘仍是 0，所以这个回退没有放松「不是弯头就别认」这条。
    let axis = match n1.cross(n2) {
        from_normals if from_normals.length_squared() >= 1e-12 => from_normals,
        _ => n1.cross(c2 - c1),
    };
    if axis.length_squared() < 1e-12 {
        return None;
    }
    let axis = axis.normalize();

    // 端盖平面内的径向：管心从这个方向离开扫掠中心。
    let d1 = n1.cross(axis).normalize();
    let d2 = n2.cross(axis).normalize();
    // 半圈时两条径向线重合，交点不唯一——两臂等长反向，扫掠中心就是两个盖心的中点。
    // 取错了下面「两臂等长」与「每个顶点都在管面上」两道校验会把它挡回去。
    let centre = line_intersection(c1, d1, c2, d2).unwrap_or_else(|| (c1 + c2) / 2.0);

    let arm1 = c1 - centre;
    let arm2 = c2 - centre;
    let offset = arm1.length();
    if !(offset > 0.0) || (offset - arm2.length()).abs() > 1e-6 {
        return None;
    }

    // 每个顶点到管心圆的距离都必须是 radius，否则它不是这段圆环。
    let on_tube = |p: &DVec3| {
        let w = *p - centre;
        let z = w.dot(axis);
        let planar = w - axis * z;
        let rho = planar.length();
        ((rho - offset).powi(2) + z * z).sqrt()
    };
    if points.iter().any(|p| (on_tube(p) - radius).abs() > 1e-6) {
        return None;
    }

    // 包角。两个端盖之间有两段弧，实体占的是哪一段，端盖本身说不出来
    // ——90° 的弯头与 270° 的弯头端盖完全一样。所以**拿实体自己的顶点定**：
    // 以某个端盖为起点绕 +axis 量角，真正的起点会让所有顶点的角位落在 [0, 包角] 内，
    // 取到的最大角位就是包角；反着取会得到 360° − 包角。两个候选取小的那个。
    //
    // 内缘贴轴（`RINS = 0`）的顶点平面半径是 0，没有角位。变换之后剩下 ~1e-13 mm
    // 的噪声方向，`atan2` 会把它记成任意角，把 90° 弯头量成 180°/270°。矩形环识别
    // 已经丢掉这一档。阈值 1e-4 mm：ams7997 那五只的贴轴半径都在 7e-13 mm 以下，
    // 管面上相邻真顶点的平面半径是毫米级。
    const AXIS_RHO_MM: f64 = 1e-4;
    let arm_dir = |arm: DVec3| (arm - axis * arm.dot(axis)).normalize();
    let sweep_extent = |start: DVec3| {
        let side = axis.cross(start);
        points.iter().fold(0.0f64, |acc, p| {
            let w = *p - centre;
            let planar = w - axis * w.dot(axis);
            if planar.length() < AXIS_RHO_MM {
                return acc;
            }
            let mut phi = planar.dot(side).atan2(planar.dot(start));
            if phi < -1e-9 {
                phi += std::f64::consts::TAU;
            }
            acc.max(phi)
        })
    };
    let (start, angle) = [arm_dir(arm1), arm_dir(arm2)]
        .into_iter()
        .map(|s| (s, sweep_extent(s)))
        .min_by(|a, b| a.1.total_cmp(&b.1))?;
    if !(angle > 0.0) {
        return None;
    }

    // 扫掠折线的段数：顶点的角位有几个不同值，就是几圈截面，段数少一。
    // 贴轴顶点的噪声角位不算一圈，否则弦高按多出来的那段算，闸门余量是错的。
    let side = axis.cross(start);
    let mut rings: Vec<f64> = Vec::new();
    for p in &points {
        let planar = (*p - centre) - axis * (*p - centre).dot(axis);
        if planar.length() < AXIS_RHO_MM {
            continue;
        }
        let mut phi = planar.dot(side).atan2(planar.dot(start));
        if phi < -1e-9 {
            phi += std::f64::consts::TAU;
        }
        if !rings.iter().any(|r| (r - phi).abs() <= 1e-7) {
            rings.push(phi);
        }
    }

    Some(RecognisedTorus {
        offset,
        radius,
        angle,
        centre,
        axis,
        start,
        tube_segments,
        sweep_steps: rings.len().saturating_sub(1).max(1),
    })
}

/// 把一组共面三角面按**共顶点**拆成连通块，每块给出它自己的顶点索引。
///
/// 只服务于端盖识别：同一个平面上可能坐着两个互不相连的盘（见 [`torus_from_local_mesh`]
/// 里 180° 那一段），把它们当成一个盘就什么都认不出来。
fn connected_triangle_groups(triangles: &[[u32; 3]]) -> Vec<Vec<u32>> {
    let mut components: Vec<Vec<u32>> = Vec::new();
    for tri in triangles {
        // 与这个三角面共顶点的已有块全部并进来——一次可能连通好几块。
        let mut merged: Vec<u32> = tri.to_vec();
        components.retain(|component| {
            if component.iter().any(|vertex| tri.contains(vertex)) {
                merged.extend_from_slice(component);
                false
            } else {
                true
            }
        });
        merged.sort_unstable();
        merged.dedup();
        components.push(merged);
    }
    components
}

/// 两条共面直线的交点。异面或平行时 None。
fn line_intersection(
    p1: glam::DVec3,
    d1: glam::DVec3,
    p2: glam::DVec3,
    d2: glam::DVec3,
) -> Option<glam::DVec3> {
    let cross = d1.cross(d2);
    let denom = cross.length_squared();
    if denom < 1e-12 {
        return None;
    }
    let t = (p2 - p1).cross(d2).dot(cross) / denom;
    let hit = p1 + d1 * t;
    // 共面校验：交点必须同时落在第二条线上。
    let w = hit - p2;
    (w - d2 * w.dot(d2)).length().le(&1e-6).then_some(hit)
}

/// 由 [`torus_from_local_mesh`] 认出来的圆环造一条 `CircularTorus` 记录。
///
/// RVM 的局部帧：扫掠绕 **+Z**、从 **+X** 起转。
pub fn circular_torus_geometry(world: &glam::DMat4, torus: &RecognisedTorus) -> AnalyticCandidate {
    let x = torus.start;
    let z = torus.axis;
    let y = z.cross(x);
    let frame = glam::DMat4::from_cols(
        x.extend(0.0),
        y.extend(0.0),
        z.extend(0.0),
        torus.centre.extend(1.0),
    );
    let placed = *world * frame;
    let cols = placed.to_cols_array();
    let matrix3 = [
        cols[0], cols[1], cols[2], cols[4], cols[5], cols[6], cols[8], cols[9], cols[10],
    ]
    .map(|value| value / M_TO_MM);
    let primitive = Primitive::CircularTorus([torus.offset, torus.radius, torus.angle]);
    let record_bbox = primitive.local_aabb_mm().unwrap_or([0.0; 6]);
    AnalyticCandidate {
        geometry: RvmGeometry {
            tag: "PRIM".to_string(),
            flags: 1,
            primitive,
            matrix3,
            translation_mm: [cols[12], cols[13], cols[14]],
            record_bbox,
            bbox_frame: crate::rvm_read::BboxFrame::Local,
            level: None,
        },
        sagitta_mm: torus.sagitta_mm(),
    }
}

/// 一段矩形环（目录 `SRTO` / 设计 `RTOR`）的识别结果，全在局部系 mm。
#[derive(Debug, Clone, Copy)]
pub struct RecognisedRectTorus {
    /// 内半径，对应 RVM 的 `inner`。`RINS` 被内核夹成 0（内缘贴轴）时就是 0。
    pub inner: f64,
    /// 外半径，对应 RVM 的 `outer`。
    pub outer: f64,
    /// 截面高（沿扫掠轴），对应 RVM 的 `height`。
    pub height: f64,
    /// 包角（弧度）；整圈是 `TAU`。
    pub angle: f64,
    /// 扫掠中心，落在截面高度的中平面上。
    pub centre: glam::DVec3,
    /// 扫掠轴（RVM 的局部 +Z）：从 `start` 起绕它右手转 `angle`，正好盖住实体。
    pub axis: glam::DVec3,
    /// 起始径向（RVM 的局部 +X）。
    pub start: glam::DVec3,
    /// 扫掠折线的段数：部分环的顶点分布在 `sweep_steps + 1` 圈截面上，整圈是 `sweep_steps` 圈。
    pub sweep_steps: usize,
}

impl RecognisedRectTorus {
    /// 环面与它的内接网格之间的缺口。
    ///
    /// 截面本身是矩形、没有离散化，只剩扫掠折线那一处；按外半径算——AABB 的极值
    /// 只可能落在外弧上，内弧的弦离真面再远也顶不到盒边。
    pub fn sagitta_mm(&self) -> f64 {
        chord_sagitta_mm(self.outer, self.angle / self.sweep_steps as f64)
    }
}

/// 网格是一段**矩形环**（矩形截面绕轴扫过 `angle`）时把它解出来；整圈也认。
///
/// 认法**先定轴、再定心**，不靠端盖：整圈没有端盖，半圈的两个端盖又反向平行
/// （`/TT-0301` 那一族 18 件全是 180°），圆环那把靠端盖叉乘定轴的路子在这里两头都走不通。
///
/// 1. 环的顶面 / 底面是两张平面环带，法向就是轴向——候选轴取自面法向，三角面多的先试；
/// 2. 沿轴所有顶点恰落两档 → 截面高 `h`；
/// 3. 顶面边界多边形上相邻三点共圆，外接圆心就是扫掠中心——三点里混进径向边的那几组
///    给出的圆心过不了下一步，跳过就是；
/// 4. 每个顶点到轴的距离必须恰好只有两档（`inner` / `outer`），轴向恰好 `±h/2`，
///    且每一圈截面四个角都在；
/// 5. 顶点按角位分圈，相邻两圈之间有三角面相连才算实体走过那一段：一段都没断就是整圈
///    （`angle = 2π`），断开一处就是从它下一圈起、转到它上一圈止；断开两处不是一段环。
///
/// 每一步都能被下一步证伪，对不上就返回 None；调用方的 AABB 自校闸再兜一道。
///
/// **只该由出身（`SRTO`）挑中它上场，别放进按长相轮的兜底次序**：矩形轮廓的 `SREV` /
/// 设计 `REVO` 回转出来的网格与它一模一样，而 E3D 对那两族写的是烘好的 `FacetGroup`。
pub fn rect_torus_from_local_mesh(
    vertices: &[[f64; 3]],
    triangles: &[[u32; 3]],
) -> Option<RecognisedRectTorus> {
    use glam::DVec3;
    let points: Vec<DVec3> = vertices.iter().copied().map(DVec3::from_array).collect();
    if points.len() < 8 || triangles.len() < 12 {
        return None;
    }

    // 候选轴：面法向去重（正反算同一个方向），三角面多的先试——顶面 / 底面各有 2N 个
    // 三角面，侧面每段只有 2 个，通常第一个候选就中。
    let mut groups: Vec<(DVec3, usize)> = Vec::new();
    for tri in triangles {
        let [a, b, c] = tri.map(|i| points[i as usize]);
        let normal = (b - a).cross(c - a);
        if normal.length_squared() <= 0.0 {
            continue;
        }
        let normal = normal.normalize();
        match groups
            .iter_mut()
            .find(|(n, _)| (*n - normal).length() < 1e-7 || (*n + normal).length() < 1e-7)
        {
            Some((_, count)) => *count += 1,
            None => groups.push((normal, 1)),
        }
    }
    groups.sort_by_key(|(_, count)| std::cmp::Reverse(*count));
    groups
        .iter()
        .find_map(|(axis, _)| rect_torus_about_axis(&points, triangles, *axis))
}

/// [`rect_torus_from_local_mesh`] 的第 2、3 步：按给定的轴向定截面高与扫掠中心。
fn rect_torus_about_axis(
    points: &[glam::DVec3],
    triangles: &[[u32; 3]],
    axis: glam::DVec3,
) -> Option<RecognisedRectTorus> {
    let origin = points[0];
    let (lo, hi) = two_levels_along(points, axis, origin)?;
    let height = hi - lo;
    if !(height > 1e-6) {
        return None;
    }
    let mid = origin + axis * ((lo + hi) / 2.0);

    // 顶面 = 三个顶点都落在高档的三角面。侧面与端盖的每个三角面都跨两档，混不进来。
    let top: Vec<[u32; 3]> = triangles
        .iter()
        .copied()
        .filter(|tri| {
            tri.iter()
                .all(|i| ((points[*i as usize] - origin).dot(axis) - hi).abs() <= 1e-6)
        })
        .collect();
    if top.len() < 2 {
        return None;
    }
    // 顶面的边界边 = 只在一个三角面里出现的边。
    let mut edge_counts: Vec<((u32, u32), usize)> = Vec::new();
    for tri in &top {
        for k in 0..3 {
            let (a, b) = (tri[k], tri[(k + 1) % 3]);
            let key = (a.min(b), a.max(b));
            match edge_counts.iter_mut().find(|(e, _)| *e == key) {
                Some((_, n)) => *n += 1,
                None => edge_counts.push((key, 1)),
            }
        }
    }
    let boundary: Vec<(u32, u32)> = edge_counts
        .into_iter()
        .filter(|(_, n)| *n == 1)
        .map(|(e, _)| e)
        .collect();
    let mut boundary_vertices: Vec<u32> = boundary.iter().flat_map(|(a, b)| [*a, *b]).collect();
    boundary_vertices.sort_unstable();
    boundary_vertices.dedup();

    // 边界上每个顶点恰有两个邻点；(邻点, 顶点, 邻点) 共圆时外接圆心就是扫掠中心。
    // 顶点是端盖角上那四个时三点跨了径向边，算出来的圆心过不了校验，换下一个。
    for &v in &boundary_vertices {
        let mut neighbours = boundary.iter().filter_map(|(a, b)| {
            if *a == v {
                Some(*b)
            } else if *b == v {
                Some(*a)
            } else {
                None
            }
        });
        let (Some(p), Some(r), None) = (neighbours.next(), neighbours.next(), neighbours.next())
        else {
            continue;
        };
        let Some(centre) = circumcentre(points[p as usize], points[v as usize], points[r as usize])
        else {
            continue;
        };
        // 圆心落回中平面：三点都在高档，算出来的圆心也在高档。
        let centre = centre - axis * (centre - mid).dot(axis);
        if let Some(found) = rect_torus_about(points, triangles, axis, centre, height) {
            return Some(found);
        }
    }
    None
}

/// [`rect_torus_from_local_mesh`] 的第 4、5 步：轴与中心都定了，校验每个顶点并量出包角。
fn rect_torus_about(
    points: &[glam::DVec3],
    triangles: &[[u32; 3]],
    axis: glam::DVec3,
    centre: glam::DVec3,
    height: f64,
) -> Option<RecognisedRectTorus> {
    use glam::DVec3;
    let h2 = height / 2.0;

    // 逐顶点：轴向 ±h/2，径向只许两档。贴轴的顶点（内半径夹成 0 那一档）没有角位。
    let mut radii: Vec<f64> = Vec::with_capacity(2);
    let mut reference: Option<DVec3> = None;
    let mut cylindrical: Vec<(f64, f64, DVec3)> = Vec::with_capacity(points.len());
    for p in points {
        let w = *p - centre;
        let z = w.dot(axis);
        if (z.abs() - h2).abs() > 1e-6 {
            return None;
        }
        let planar = w - axis * z;
        let rho = planar.length();
        if !radii.iter().any(|r| (r - rho).abs() <= 1e-6) {
            radii.push(rho);
            if radii.len() > 2 {
                return None;
            }
        }
        if rho > 1e-6 && reference.is_none() {
            reference = Some(planar / rho);
        }
        cylindrical.push((rho, z, planar));
    }
    if radii.len() != 2 {
        return None;
    }
    let inner = radii[0].min(radii[1]);
    let outer = radii[0].max(radii[1]);
    let inner = if inner <= 1e-6 { 0.0 } else { inner };
    if !(outer > inner) {
        return None;
    }
    let x0 = reference?;
    let y0 = axis.cross(x0);

    // 顶点按角位分圈。`ring_of[i]` 是顶点 i 所在的圈；贴轴的顶点记 None。
    let mut ring_angles: Vec<f64> = Vec::new();
    let mut ring_of: Vec<Option<usize>> = Vec::with_capacity(points.len());
    // 每圈四个角（外低、外高、内低、内高）都得在；内半径为 0 时只查外侧两个。
    let mut corners: Vec<[bool; 4]> = Vec::new();
    for (rho, z, planar) in &cylindrical {
        if *rho <= 1e-6 {
            ring_of.push(None);
            continue;
        }
        let mut phi = planar.dot(y0).atan2(planar.dot(x0));
        if phi < -1e-7 {
            phi += std::f64::consts::TAU;
        } else if phi < 0.0 {
            phi = 0.0;
        }
        let ring = match ring_angles.iter().position(|a| (a - phi).abs() <= 1e-7) {
            Some(index) => index,
            None => {
                ring_angles.push(phi);
                corners.push([false; 4]);
                ring_angles.len() - 1
            }
        };
        let slot = usize::from((rho - outer).abs() > 1e-6) * 2 + usize::from(*z > 0.0);
        corners[ring][slot] = true;
        ring_of.push(Some(ring));
    }
    if ring_angles.len() < 3 {
        return None;
    }
    let required = if inner > 0.0 { 4 } else { 2 };
    if corners
        .iter()
        .any(|slots| slots.iter().take(required).any(|present| !present))
    {
        return None;
    }

    // 圈按角位排序；相邻两圈（含末圈接首圈）之间要有跨这两圈的三角面才算实体走过那一段。
    let mut order: Vec<usize> = (0..ring_angles.len()).collect();
    order.sort_by(|a, b| ring_angles[*a].total_cmp(&ring_angles[*b]));
    let mut rank = vec![0usize; ring_angles.len()];
    for (position, ring) in order.iter().enumerate() {
        rank[*ring] = position;
    }
    let count = order.len();
    let mut linked = vec![false; count];
    for tri in triangles {
        let mut spanned: [Option<usize>; 2] = [None, None];
        let mut ok = true;
        for i in tri {
            let Some(ring) = ring_of[*i as usize] else {
                continue;
            };
            let position = rank[ring];
            match spanned {
                [Some(a), _] if a == position => {}
                [_, Some(b)] if b == position => {}
                [None, _] => spanned[0] = Some(position),
                [Some(_), None] => spanned[1] = Some(position),
                _ => {
                    ok = false;
                    break;
                }
            }
        }
        if !ok {
            continue;
        }
        if let [Some(a), Some(b)] = spanned {
            let (lo, hi) = (a.min(b), a.max(b));
            if hi - lo == 1 {
                linked[lo] = true;
            } else if lo == 0 && hi == count - 1 {
                linked[count - 1] = true;
            }
        }
    }
    let gaps: Vec<usize> = (0..count).filter(|k| !linked[*k]).collect();
    let (start_position, angle, sweep_steps) = match gaps[..] {
        // 一段都没断：整圈。起点随便取一圈——记录绕回来是同一只环。
        [] => (0, std::f64::consts::TAU, count),
        // 断开一处：从它下一圈起，转到它上一圈止。
        [gap] => {
            let start = (gap + 1) % count;
            let sweep = (ring_angles[order[gap]] - ring_angles[order[start]])
                .rem_euclid(std::f64::consts::TAU);
            (start, sweep, count - 1)
        }
        _ => return None,
    };
    if !(angle > 0.0) {
        return None;
    }
    let phi_start = ring_angles[order[start_position]];
    let start = x0 * phi_start.cos() + y0 * phi_start.sin();

    Some(RecognisedRectTorus {
        inner,
        outer,
        height,
        angle,
        centre,
        axis,
        start,
        sweep_steps,
    })
}

/// 三点的外接圆心（三维，落在三点所在的平面内）。共线时 None。
fn circumcentre(p: glam::DVec3, q: glam::DVec3, r: glam::DVec3) -> Option<glam::DVec3> {
    let a = p - r;
    let b = q - r;
    let cross = a.cross(b);
    let denom = 2.0 * cross.length_squared();
    if denom < 1e-18 {
        return None;
    }
    Some(r + (b * a.length_squared() - a * b.length_squared()).cross(cross) / denom)
}

/// 由 [`rect_torus_from_local_mesh`] 认出来的矩形环造一条 `RectangularTorus` 记录。
///
/// RVM 的局部帧与圆环同款：扫掠绕 **+Z**、从 **+X** 起转；截面沿 Z 居中（±h/2）。
pub fn rectangular_torus_geometry(
    world: &glam::DMat4,
    torus: &RecognisedRectTorus,
) -> AnalyticCandidate {
    let x = torus.start;
    let z = torus.axis;
    let y = z.cross(x);
    let frame = glam::DMat4::from_cols(
        x.extend(0.0),
        y.extend(0.0),
        z.extend(0.0),
        torus.centre.extend(1.0),
    );
    let placed = *world * frame;
    let cols = placed.to_cols_array();
    let matrix3 = [
        cols[0], cols[1], cols[2], cols[4], cols[5], cols[6], cols[8], cols[9], cols[10],
    ]
    .map(|value| value / M_TO_MM);
    let primitive =
        Primitive::RectangularTorus([torus.inner, torus.outer, torus.height, torus.angle]);
    let record_bbox = primitive.local_aabb_mm().unwrap_or([0.0; 6]);
    AnalyticCandidate {
        geometry: RvmGeometry {
            tag: "PRIM".to_string(),
            flags: 1,
            primitive,
            matrix3,
            translation_mm: [cols[12], cols[13], cols[14]],
            record_bbox,
            bbox_frame: crate::rvm_read::BboxFrame::Local,
            level: None,
        },
        sagitta_mm: torus.sagitta_mm(),
    }
}

/// 由 [`cylinder_from_local_mesh`] 认出来的圆柱造一条 `Cylinder` 记录。
///
/// RVM 的 `Cylinder` 轴向是**局部 Z、沿 Z 居中**，所以要把识别出来的轴转到 Z。
pub fn cylinder_geometry(world: &glam::DMat4, cylinder: &RecognisedCylinder) -> AnalyticCandidate {
    let RecognisedCylinder {
        radius,
        height,
        axis,
        centre,
        ..
    } = *cylinder;
    let to_z = glam::DQuat::from_rotation_arc(glam::DVec3::Z, axis);
    let placed = *world * glam::DMat4::from_rotation_translation(to_z, centre);
    let cols = placed.to_cols_array();
    let matrix3 = [
        cols[0], cols[1], cols[2], cols[4], cols[5], cols[6], cols[8], cols[9], cols[10],
    ]
    .map(|value| value / M_TO_MM);
    AnalyticCandidate {
        geometry: RvmGeometry {
            tag: "PRIM".to_string(),
            flags: 1,
            primitive: Primitive::Cylinder([radius, height]),
            matrix3,
            translation_mm: [cols[12], cols[13], cols[14]],
            record_bbox: [
                -radius,
                -radius,
                -height / 2.0,
                radius,
                radius,
                height / 2.0,
            ],
            bbox_frame: crate::rvm_read::BboxFrame::Local,
            level: None,
        },
        sagitta_mm: cylinder.sagitta_mm(),
    }
}

/// 给一只**没有体**的目录圆柱造一条 `Cylinder` 记录：半径与高里至少一个是 0，记录剩下的
/// 只有它的放置。
///
/// 这不是我们发明的写法。Core 对零径 / 零高的 SCYL 不出体，E3D 的 RVM 驱动却照走 DRCYL：
/// `/Copy-(2)-of-GV0406C` 那只格子全空的写成 `Cylinder[radius=0 height=-0]`（基准 80 根上
/// 132 条），`/LA-0301` 第 45 条 `PHEI` 真解成 0 的写成 `Cylinder[radius=57 height=-0]`
/// ——半径照留。不跟着写，两侧的记录条数就永远差这一条，而几何本来是一致的。
///
/// `local` 是图元在元素局部系里的放置（平移是原点，Z 是轴向），与
/// [`crate::elmodl::DegenerateCataloguePart::local_transform`] 同一份。
pub fn bodyless_cylinder_geometry(
    world: &glam::DMat4,
    local: &glam::DMat4,
    radius: f64,
    height: f64,
) -> crate::rvm_read::RvmGeometry {
    let cols = (*world * *local).to_cols_array();
    let matrix3 = [
        cols[0], cols[1], cols[2], cols[4], cols[5], cols[6], cols[8], cols[9], cols[10],
    ]
    .map(|value| value / M_TO_MM);
    RvmGeometry {
        tag: "PRIM".to_string(),
        flags: 1,
        primitive: Primitive::Cylinder([radius, height]),
        matrix3,
        translation_mm: [cols[12], cols[13], cols[14]],
        record_bbox: [
            -radius,
            -radius,
            -height / 2.0,
            radius,
            radius,
            height / 2.0,
        ],
        bbox_frame: crate::rvm_read::BboxFrame::Local,
        level: None,
    }
}

/// 由 [`snout_from_local_mesh`] 认出来的圆台造一条 `Snout` 记录。
///
/// RVM 的 `Snout`：轴向局部 **Z**、原点在两个面心的中点，`XOFF`/`YOFF` 是顶面中心相对
/// 底面中心的偏移，**上下各摊一半**。局部 +X 取偏移方向，于是 `YOFF` 恒为 0；剪切一律 0
/// ——带剪切的形体识别器根本不认（见它的文档）。
pub fn snout_geometry(world: &glam::DMat4, snout: &RecognisedSnout) -> AnalyticCandidate {
    let x = snout.start;
    let z = snout.axis;
    let y = z.cross(x);
    let frame = glam::DMat4::from_cols(
        x.extend(0.0),
        y.extend(0.0),
        z.extend(0.0),
        snout.centre.extend(1.0),
    );
    let placed = *world * frame;
    let cols = placed.to_cols_array();
    let matrix3 = [
        cols[0], cols[1], cols[2], cols[4], cols[5], cols[6], cols[8], cols[9], cols[10],
    ]
    .map(|value| value / M_TO_MM);
    let primitive = Primitive::Snout([
        snout.bottom_radius,
        snout.top_radius,
        snout.height,
        snout.offset,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]);
    let record_bbox = primitive.local_aabb_mm().unwrap_or([0.0; 6]);
    AnalyticCandidate {
        geometry: RvmGeometry {
            tag: "PRIM".to_string(),
            flags: 1,
            primitive,
            matrix3,
            translation_mm: [cols[12], cols[13], cols[14]],
            record_bbox,
            bbox_frame: crate::rvm_read::BboxFrame::Local,
            level: None,
        },
        sagitta_mm: snout.sagitta_mm(),
    }
}

/// 两个单位轴是否共线（同向或反向都算）。出身标架的 Z 与识别出的轴夹角超过这个量级，
/// 就说明出身对不上这块网格，记录照识别结果写。
fn axes_collinear(a: glam::DVec3, b: glam::DVec3) -> bool {
    a.cross(b).length() <= 1e-6 && a.dot(b).abs() > 0.5
}

/// 出身的放置标架在手时，圆柱记录的**朝向照它写**，尺寸与轴心仍取识别结果。
///
/// E3D 的 DRCYL 写的是目录图元自己的标架：Z 列 = `PAXI`（含符号）、X/Y 列 = `ORIMAT` 的
/// 副轴——正是 [`crate::catalogue`] 放这块网格时用的那只矩阵。[`cylinder_geometry`] 从网格
/// 反推：端盖法向的符号由 [`canonical_axis_sign`] 定、副轴由 `from_rotation_arc` 定，两者都
/// 与 E3D 不同——ams7999 `/1WCC318VN` 16 件里只有 3 件四列全同，其余 X/Y 列不同、5 件 Z 列
/// 反号（happyrust/e3d-model#1）。几何一样（圆柱关于中心对称），只是矩阵不一样。
///
/// 半径 / 高 / 轴心是网格说了算的量，照旧取识别结果；自校闸也照旧按写出的记录重算 AABB。
/// 识别轴与出身的 Z 不共线（夹角 > 1e-6）→ 出身对不上这块网格，退回 [`cylinder_geometry`]。
pub fn cylinder_geometry_in_frame(
    world: &glam::DMat4,
    cylinder: &RecognisedCylinder,
    frame: &glam::DMat4,
) -> AnalyticCandidate {
    let rotation = glam::DMat3::from_mat4(*frame);
    if !axes_collinear(rotation.z_axis, cylinder.axis) {
        return cylinder_geometry(world, cylinder);
    }
    let mut candidate = cylinder_geometry(world, cylinder);
    let placed = *world
        * (glam::DMat4::from_translation(cylinder.centre) * glam::DMat4::from_mat3(rotation));
    let cols = placed.to_cols_array();
    candidate.geometry.matrix3 = [
        cols[0], cols[1], cols[2], cols[4], cols[5], cols[6], cols[8], cols[9], cols[10],
    ]
    .map(|value| value / M_TO_MM);
    candidate.geometry.translation_mm = [cols[12], cols[13], cols[14]];
    candidate
}

/// 出身的放置标架在手时，圆台记录的**朝向与底顶次序照它写**，尺寸与盖心仍取识别结果。
///
/// E3D 的 DRSNO 写目录 LSNO / SCON 自己的标架（`PAAX` 为 Z、偏移轴为 X），`PBDM` 是底、
/// `PTDM` 是顶；识别器的轴符号任意，反过来时把两端半径对调了写（[`snout_geometry`] 的
/// 文档就写着这一条）。这里把识别结果换到出身的标架里：识别轴与 Z 反向就底顶对调，
/// 偏移向量跟着取反，再投到出身的 X / Y 上得 `XOFF` / `YOFF`。不共线 → 退回 [`snout_geometry`]。
pub fn snout_geometry_in_frame(
    world: &glam::DMat4,
    snout: &RecognisedSnout,
    frame: &glam::DMat4,
) -> AnalyticCandidate {
    let rotation = glam::DMat3::from_mat4(*frame);
    if !axes_collinear(rotation.z_axis, snout.axis) {
        return snout_geometry(world, snout);
    }
    let same_way = rotation.z_axis.dot(snout.axis) > 0.0;
    let (bottom_radius, top_radius) = if same_way {
        (snout.bottom_radius, snout.top_radius)
    } else {
        (snout.top_radius, snout.bottom_radius)
    };
    // 识别出的偏移：底心 → 顶心的横向部分，沿识别轴正向那端是顶。换轴向后底顶互换，
    // 这个向量整个反号。
    let offset = snout.start * snout.offset * if same_way { 1.0 } else { -1.0 };
    // 盖心相减带 1e-13 级的舍入渣（`offset_x: 5.75e-13`），E3D 写的是 0.0；1e-9 mm 以下当 0。
    let snap = |value: f64| if value.abs() < 1e-9 { 0.0 } else { value };
    let (xoff, yoff) = (
        snap(offset.dot(rotation.x_axis)),
        snap(offset.dot(rotation.y_axis)),
    );
    let placed =
        *world * (glam::DMat4::from_translation(snout.centre) * glam::DMat4::from_mat3(rotation));
    let cols = placed.to_cols_array();
    let matrix3 = [
        cols[0], cols[1], cols[2], cols[4], cols[5], cols[6], cols[8], cols[9], cols[10],
    ]
    .map(|value| value / M_TO_MM);
    let primitive = Primitive::Snout([
        bottom_radius,
        top_radius,
        snout.height,
        xoff,
        yoff,
        0.0,
        0.0,
        0.0,
        0.0,
    ]);
    let record_bbox = primitive.local_aabb_mm().unwrap_or([0.0; 6]);
    AnalyticCandidate {
        geometry: RvmGeometry {
            tag: "PRIM".to_string(),
            flags: 1,
            primitive,
            matrix3,
            translation_mm: [cols[12], cols[13], cols[14]],
            record_bbox,
            bbox_frame: crate::rvm_read::BboxFrame::Local,
            level: None,
        },
        sagitta_mm: snout.sagitta_mm(),
    }
}

/// 由 [`box_from_local_mesh`] 认出来的盒子造一条 `Box` 记录。
/// `centre_local_mm` 是盒心在元素局部系里的位置（一般不是原点）。
///
/// 盒子没有曲面，网格就是形体本身，所以 [`AnalyticCandidate::sagitta_mm`] 是 0。
pub fn box_geometry(
    world: &glam::DMat4,
    lengths: [f64; 3],
    centre_local_mm: [f64; 3],
) -> AnalyticCandidate {
    let placed = *world * glam::DMat4::from_translation(glam::DVec3::from_array(centre_local_mm));
    let cols = placed.to_cols_array();
    let matrix3 = [
        cols[0], cols[1], cols[2], cols[4], cols[5], cols[6], cols[8], cols[9], cols[10],
    ]
    .map(|value| value / M_TO_MM);
    let half = lengths.map(|l| l / 2.0);
    AnalyticCandidate {
        geometry: RvmGeometry {
            tag: "PRIM".to_string(),
            flags: 1,
            primitive: Primitive::Box(lengths),
            matrix3,
            translation_mm: [cols[12], cols[13], cols[14]],
            record_bbox: [-half[0], -half[1], -half[2], half[0], half[1], half[2]],
            bbox_frame: crate::rvm_read::BboxFrame::Local,
            level: None,
        },
        sagitta_mm: 0.0,
    }
}

/// 一只四棱台（`Pyramid`）的识别结果，全在局部系 mm。
#[derive(Debug, Clone, Copy)]
pub struct RecognisedPyramid {
    /// 底面（`axis` 负向那一端）沿局部 X / Y 的边长。
    pub bottom: [f64; 2],
    /// 顶面沿局部 X / Y 的边长；退化成一条线或一个点时对应的边为 0。
    pub top: [f64; 2],
    pub height: f64,
    /// 顶面心相对底面心的横向偏移（局部 X / Y 分量），对应 RVM 的 `XOFF` / `YOFF`。
    pub offset: [f64; 2],
    /// 两个面心的中点（RVM 原点）。
    pub centre: glam::DVec3,
    /// 底 → 顶（RVM 的局部 +Z）。
    pub axis: glam::DVec3,
    /// 底面一条边的方向（RVM 的局部 +X）。
    pub x: glam::DVec3,
}

/// 网格是一只四棱台（= `Pyramid`，含退化成楔形 / 四棱锥的）时把它解出来。
///
/// 认法：两个端面平行（顶点沿端面法向只落在**两个**档位上），每一档的顶点各自是一个
/// **矩形**（4 点）、一条线（2 点）或一个点，而且两端的边**同向**——上下两条边平行，
/// 四个侧面才各自共面，8 个角点的凸包才恰好是那只棱台（[`crate::solid::pyramid_solid`]
/// 就是这么建的）。两个面心的横向错位就是 `XOFF` / `YOFF`，上下各摊一半。
///
/// 顶点数只能是 5 / 6 / 8（至少一端是完整的矩形），三角面数 `2V − 4`：凸包不会多铺一片。
/// 侧面法向也会被当轴试一遍，但侧面是梯形，过不了矩形那一关；等边不偏心的侧面是矩形，
/// 那时认出来的是同一只实体的另一种参数写法，几何不变。
///
/// **上下同形且不偏心的一律不认**（返回 None）：那是一只盒子，`Box` 那条记录才贴 E3D。
/// 轴对齐的盒子 [`box_from_local_mesh`] 早一步就接了；转过角度的暂时仍退面片。
pub fn pyramid_from_local_mesh(
    vertices: &[[f64; 3]],
    triangles: &[[u32; 3]],
) -> Option<RecognisedPyramid> {
    pyramid_from_local_mesh_inner(vertices, triangles, false)
}

/// 同 [`pyramid_from_local_mesh`]，但**不**把上下同形不偏心的那只让给 `Box`。
///
/// 出身是 `LPYR` 时用这一条：E3D 的 DRPYR 不看棱台退化成什么，照写 `Pyramid`。300 只
/// 阀门的手轮座正是这一档——PAAX 与 PBAX 都解出 +X，轴由 Core 的 ORTHOG 补出来，成品是
/// 一只正方盒；基准 RVM 里它仍是 `Pyramid`，我们按长相认就写成了 `Box`。
pub fn pyramid_from_local_mesh_including_boxes(
    vertices: &[[f64; 3]],
    triangles: &[[u32; 3]],
) -> Option<RecognisedPyramid> {
    pyramid_from_local_mesh_inner(vertices, triangles, true)
}

fn pyramid_from_local_mesh_inner(
    vertices: &[[f64; 3]],
    triangles: &[[u32; 3]],
    keep_boxes: bool,
) -> Option<RecognisedPyramid> {
    use glam::DVec3;
    let mut points: Vec<DVec3> = Vec::with_capacity(8);
    for v in vertices {
        let p = DVec3::from_array(*v);
        if !points.iter().any(|q| (*q - p).length() <= 1e-6) {
            points.push(p);
            if points.len() > 8 {
                return None;
            }
        }
    }
    if points.len() < 5 || triangles.len() != 2 * points.len() - 4 {
        return None;
    }
    let centroid = points.iter().copied().sum::<DVec3>() / points.len() as f64;

    for axis in face_normal_axes(&points, triangles) {
        let Some((low, high)) = two_levels_along(&points, axis, centroid) else {
            continue;
        };
        let height = high - low;
        if !(height > 0.0) {
            continue;
        }
        let at_level = |level: f64| -> Vec<DVec3> {
            points
                .iter()
                .copied()
                .filter(|p| ((*p - centroid).dot(axis) - level).abs() <= 1e-6)
                .collect()
        };
        let (bottom, top) = (at_level(low), at_level(high));
        // 边的方向从完整的那个矩形上取；两端都退化就撑不起「同向」这句判断。
        let Some(edge) =
            rectangle_edge_direction(&bottom).or_else(|| rectangle_edge_direction(&top))
        else {
            continue;
        };
        let x = (edge - axis * edge.dot(axis)).normalize();
        let y = axis.cross(x);
        let bottom_centre = bottom.iter().copied().sum::<DVec3>() / bottom.len() as f64;
        let top_centre = top.iter().copied().sum::<DVec3>() / top.len() as f64;
        let (Some(bottom_lengths), Some(top_lengths)) = (
            rectangle_extents(&bottom, bottom_centre, x, y),
            rectangle_extents(&top, top_centre, x, y),
        ) else {
            continue;
        };
        let arm = top_centre - bottom_centre;
        let offset = [arm.dot(x), arm.dot(y)];
        if !keep_boxes
            && (bottom_lengths[0] - top_lengths[0]).abs() <= 1e-9
            && (bottom_lengths[1] - top_lengths[1]).abs() <= 1e-9
            && offset.iter().all(|o| o.abs() <= 1e-9)
        {
            return None; // 这是盒子，不抢 `Box` 的记录。
        }
        return Some(RecognisedPyramid {
            bottom: bottom_lengths,
            top: top_lengths,
            height,
            offset,
            centre: (bottom_centre + top_centre) / 2.0,
            axis,
            x,
        });
    }
    None
}

/// 四个共面点是一个矩形时给出它的一条边方向；不是四个点、或不是矩形（梯形、菱形、
/// 一般四边形）时 None。
fn rectangle_edge_direction(cap: &[glam::DVec3]) -> Option<glam::DVec3> {
    let [p0, rest @ ..] = cap else {
        return None;
    };
    let [a, b, c] = rest else {
        return None;
    };
    // 离 p0 最远的是对角，其余两个是邻点。
    let mut rest = [*a, *b, *c];
    rest.sort_by(|a, b| (*a - *p0).length().total_cmp(&(*b - *p0).length()));
    let [n1, n2, diagonal] = rest;
    let (e1, e2) = (n1 - *p0, n2 - *p0);
    let (l1, l2) = (e1.length(), e2.length());
    if !(l1 > 0.0 && l2 > 0.0) {
        return None;
    }
    // 邻边垂直、第四个点落在 p0 + e1 + e2：两句合起来就是「矩形」。
    if e1.dot(e2).abs() > 1e-9 * l1 * l2 || (*p0 + e1 + e2 - diagonal).length() > 1e-6 {
        return None;
    }
    Some(e1 / l1)
}

/// 一档顶点在 `(u, v)` 里的边长 `[沿 u, 沿 v]`：每个点到档心的 u / v 分量的绝对值必须
/// 各自恒定，点数必须与「几条边非零」对得上（4 / 2 / 1），否则 None。
fn rectangle_extents(
    cap: &[glam::DVec3],
    centre: glam::DVec3,
    u: glam::DVec3,
    v: glam::DVec3,
) -> Option<[f64; 2]> {
    let first = *cap.first()? - centre;
    let (eu, ev) = (first.dot(u).abs(), first.dot(v).abs());
    if cap.iter().any(|p| {
        let d = *p - centre;
        (d.dot(u).abs() - eu).abs() > 1e-6 || (d.dot(v).abs() - ev).abs() > 1e-6
    }) {
        return None;
    }
    let snap = |e: f64| if e <= 1e-6 { 0.0 } else { e };
    let (eu, ev) = (snap(eu), snap(ev));
    let expected = match (eu > 0.0, ev > 0.0) {
        (true, true) => 4,
        (false, false) => 1,
        _ => 2,
    };
    (cap.len() == expected).then_some([2.0 * eu, 2.0 * ev])
}

/// 由 [`pyramid_from_local_mesh`] 认出来的四棱台造一条 `Pyramid` 记录。
///
/// RVM 的 `Pyramid`：轴向局部 **Z**、原点在两个面心的中点，底面心 (−XOFF/2, −YOFF/2, −H/2)、
/// 顶面心 (+XOFF/2, +YOFF/2, +H/2)（`GM_Pyramid::calcFacets`，见
/// [`crate::rvm_read::Primitive::local_hull_points_mm`]）。多面体就是形体本身，缺口为 0。
pub fn pyramid_geometry(world: &glam::DMat4, pyramid: &RecognisedPyramid) -> AnalyticCandidate {
    let x = pyramid.x;
    let z = pyramid.axis;
    let y = z.cross(x);
    let frame = glam::DMat4::from_cols(
        x.extend(0.0),
        y.extend(0.0),
        z.extend(0.0),
        pyramid.centre.extend(1.0),
    );
    let placed = *world * frame;
    let cols = placed.to_cols_array();
    let matrix3 = [
        cols[0], cols[1], cols[2], cols[4], cols[5], cols[6], cols[8], cols[9], cols[10],
    ]
    .map(|value| value / M_TO_MM);
    let primitive = Primitive::Pyramid([
        pyramid.bottom[0],
        pyramid.bottom[1],
        pyramid.top[0],
        pyramid.top[1],
        pyramid.offset[0],
        pyramid.offset[1],
        pyramid.height,
    ]);
    let record_bbox = primitive.local_aabb_mm().unwrap_or([0.0; 6]);
    AnalyticCandidate {
        geometry: RvmGeometry {
            tag: "PRIM".to_string(),
            flags: 1,
            primitive,
            matrix3,
            translation_mm: [cols[12], cols[13], cols[14]],
            record_bbox,
            bbox_frame: crate::rvm_read::BboxFrame::Local,
            level: None,
        },
        sagitta_mm: 0.0,
    }
}

/// 出身的放置标架在手时，四棱台记录的**轴向照目录 Z 写**。
///
/// [`pyramid_geometry`] 的 Z 是网格认出来的那根轴。E3D 的 DRPYR 写的是目录图元自己的
/// 标架：高沿 Z，底 / 顶边沿 X / Y。阀门 `/1WCC096VN`（`24383/66600`）上两边不共线——
/// E3D 把 120 mm 当沿 X 的高，网格把 245 mm 当沿 −Z 的高，底边和高度对调了。
///
/// 两档：
/// - 目录 Z 与识别轴共线：矩阵改用标架（含符号和滚角），反向时底顶对调、偏移反号，
///   再投到标架的 X / Y 上。尺寸仍是网格的。
/// - 上下同形、不偏心（长得像盒子的棱台）：三条边长可以换轴。目录的 X / Y / Z 各自
///   与识别轴或一条底边共线时，把对应边长写成底 X、底 Y、高。对不上任何一条就退回
///   [`pyramid_geometry`]。有锥度或偏心、但目录 Z 又不贴识别轴时同样退回——锥度只沿
///   识别轴，换到一条底边上就不是同一只 RVM `Pyramid`。
pub fn pyramid_geometry_in_frame(
    world: &glam::DMat4,
    pyramid: &RecognisedPyramid,
    frame: &glam::DMat4,
) -> AnalyticCandidate {
    let rotation = glam::DMat3::from_mat4(*frame);
    let axis = pyramid.axis.normalize_or_zero();
    let x_axis = pyramid.x.normalize_or_zero();
    let y_axis = axis.cross(x_axis);
    if axis == glam::DVec3::ZERO || x_axis == glam::DVec3::ZERO || y_axis.length_squared() < 0.5 {
        return pyramid_geometry(world, pyramid);
    }
    let y_axis = y_axis.normalize();

    let snap = |value: f64| if value.abs() < 1e-9 { 0.0 } else { value };
    let place = |primitive: Primitive| {
        let placed = *world
            * (glam::DMat4::from_translation(pyramid.centre) * glam::DMat4::from_mat3(rotation));
        let cols = placed.to_cols_array();
        let matrix3 = [
            cols[0], cols[1], cols[2], cols[4], cols[5], cols[6], cols[8], cols[9], cols[10],
        ]
        .map(|value| value / M_TO_MM);
        let record_bbox = primitive.local_aabb_mm().unwrap_or([0.0; 6]);
        AnalyticCandidate {
            geometry: RvmGeometry {
                tag: "PRIM".to_string(),
                flags: 1,
                primitive,
                matrix3,
                translation_mm: [cols[12], cols[13], cols[14]],
                record_bbox,
                bbox_frame: crate::rvm_read::BboxFrame::Local,
                level: None,
            },
            sagitta_mm: 0.0,
        }
    };

    if axes_collinear(rotation.z_axis, axis) {
        let same_way = rotation.z_axis.dot(axis) > 0.0;
        let (bottom, top) = if same_way {
            (pyramid.bottom, pyramid.top)
        } else {
            (pyramid.top, pyramid.bottom)
        };
        let sign = if same_way { 1.0 } else { -1.0 };
        let offset = (x_axis * pyramid.offset[0] + y_axis * pyramid.offset[1]) * sign;
        return place(Primitive::Pyramid([
            bottom[0],
            bottom[1],
            top[0],
            top[1],
            snap(offset.dot(rotation.x_axis)),
            snap(offset.dot(rotation.y_axis)),
            pyramid.height,
        ]));
    }

    let boxy = (pyramid.top[0] - pyramid.bottom[0]).abs() < 1e-6
        && (pyramid.top[1] - pyramid.bottom[1]).abs() < 1e-6
        && pyramid.offset[0].abs() < 1e-6
        && pyramid.offset[1].abs() < 1e-6;
    if !boxy {
        return pyramid_geometry(world, pyramid);
    }
    let extent = |dir: glam::DVec3| {
        if axes_collinear(dir, axis) {
            Some(pyramid.height)
        } else if axes_collinear(dir, x_axis) {
            Some(pyramid.bottom[0])
        } else if axes_collinear(dir, y_axis) {
            Some(pyramid.bottom[1])
        } else {
            None
        }
    };
    let (Some(bottom_x), Some(bottom_y), Some(height)) = (
        extent(rotation.x_axis),
        extent(rotation.y_axis),
        extent(rotation.z_axis),
    ) else {
        return pyramid_geometry(world, pyramid);
    };
    place(Primitive::Pyramid([
        bottom_x, bottom_y, bottom_x, bottom_y, 0.0, 0.0, height,
    ]))
}

/// 一只球的识别结果，全在局部系 mm。
#[derive(Debug, Clone, Copy)]
pub struct RecognisedSphere {
    pub radius: f64,
    pub centre: glam::DVec3,
    /// 内接多面体离球面最远处的缺口 = 半径 − 内切球半径，见 [`sphere_from_local_mesh`]。
    pub sagitta_mm: f64,
}

/// 一只离散球至少有几个顶点。
///
/// 球走 `Manifold::sphere`（manifold `constructors.cpp`：八面体每条边分 `n = ⌈段数/4⌉` 段
/// 再投到球面上，顶点 `4n² + 2`），段数走 [`crate::solid::cylinder_segments`]，
/// 与圆柱同一条 45° 封顶、最少 8 段的规则——`n ≥ 2`，顶点至少 18 个。
/// 立方体（8）、八面体（6）、正十二面体（20 但内切比不够，见下）在这儿或下一道被挡下。
pub const MIN_SPHERE_VERTICES: usize = 18;

/// 内切球半径与外接球半径之比的下限——「所有顶点与中心等距」这一句单独不够：
/// 立方体、正多面体、以及任何高度凑成 `h²/4 + r² = R²` 的正棱柱，顶点也全在一只球上。
///
/// 最粗的那只合法的球（8 段，八面体每边一分为二）里，三个棱中点张成的那一片到球心
/// `√(2/3)·r ≈ 0.8165 r`；而正十二面体是 0.7947，内接于球的正棱柱不到 `1/√2`，
/// 立方体 0.5774。0.8 落在两边之间。
pub const MIN_SPHERE_INRADIUS_RATIO: f64 = 0.8;

/// 网格是一只离散球时把它解出来。
///
/// 1. 所有顶点到中心**等距**（= 半径）。中心取顶点云 AABB 的中心：离散球的顶点集中心对称，
///    AABB 中心就是球心，重复顶点也带不偏它；
/// 2. 顶点至少 [`MIN_SPHERE_VERTICES`] 个，内切比至少 [`MIN_SPHERE_INRADIUS_RATIO`]，
///    把恰好也满足第 1 条的多面体挡在外面。
///
/// 缺口直接从网格量：凸多面体在任何方向上的支撑值都不小于它的内切球半径，所以
/// `半径 − 内切球半径` 就是它与球面之间**最大**的系统差——不必知道这块网格是怎么离散的。
pub fn sphere_from_local_mesh(
    vertices: &[[f64; 3]],
    triangles: &[[u32; 3]],
) -> Option<RecognisedSphere> {
    use glam::DVec3;
    if vertices.len() < MIN_SPHERE_VERTICES || triangles.len() < 2 * MIN_SPHERE_VERTICES - 4 {
        return None;
    }
    let points: Vec<DVec3> = vertices.iter().copied().map(DVec3::from_array).collect();
    let (lo, hi) = points.iter().fold((points[0], points[0]), |(lo, hi), p| {
        (lo.min(*p), hi.max(*p))
    });
    let centre = (lo + hi) / 2.0;
    let radius = (points[0] - centre).length();
    if !(radius > 0.0)
        || points
            .iter()
            .any(|p| ((*p - centre).length() - radius).abs() > 1e-6)
    {
        return None;
    }
    let inradius = inscribed_radius(&points, triangles, centre)?;
    if !(inradius >= MIN_SPHERE_INRADIUS_RATIO * radius) {
        return None;
    }
    Some(RecognisedSphere {
        radius,
        centre,
        sagitta_mm: radius - inradius,
    })
}

/// 多面体的内切球半径：每张面的平面到 `centre` 的距离取最小。退化面跳过；一张都没有时 None。
fn inscribed_radius(
    points: &[glam::DVec3],
    triangles: &[[u32; 3]],
    centre: glam::DVec3,
) -> Option<f64> {
    let mut inradius = f64::MAX;
    for tri in triangles {
        let [a, b, c] = tri.map(|i| points[i as usize]);
        let normal = (b - a).cross(c - a);
        if normal.length_squared() <= 0.0 {
            continue;
        }
        inradius = inradius.min(normal.normalize().dot(a - centre).abs());
    }
    (inradius < f64::MAX).then_some(inradius)
}

/// 由 [`sphere_from_local_mesh`] 认出来的球造一条 `Sphere` 记录。
///
/// 球没有朝向，但记录**照样带元素的旋转**（E3D 就是这么写的：`/PS-0001` 那条 `Sphere`
/// 的矩阵是元素的 90° 置换）。`rvm_compare` 拿成员第一条几何的旋转当局部帧，球若写成
/// 不转的，轮到它排第一时两侧的帧就对不上——2026-09-14 实测 16 只阀球被判 off，
/// 差的正是 `(√2 − 1)·r`。世界盒不受影响：[`AnalyticCandidate::world_aabb_mm`] 给球单独算。
pub fn sphere_geometry(world: &glam::DMat4, sphere: &RecognisedSphere) -> AnalyticCandidate {
    let placed = *world * glam::DMat4::from_translation(sphere.centre);
    let cols = placed.to_cols_array();
    let matrix3 = [
        cols[0], cols[1], cols[2], cols[4], cols[5], cols[6], cols[8], cols[9], cols[10],
    ]
    .map(|value| value / M_TO_MM);
    let r = sphere.radius;
    AnalyticCandidate {
        geometry: RvmGeometry {
            tag: "PRIM".to_string(),
            flags: 1,
            primitive: Primitive::Sphere([r]),
            matrix3,
            translation_mm: [cols[12], cols[13], cols[14]],
            record_bbox: [-r, -r, -r, r, r, r],
            bbox_frame: crate::rvm_read::BboxFrame::Local,
            level: None,
        },
        sagitta_mm: sphere.sagitta_mm,
    }
}

/// 出身的放置标架在手时，球记录的**旋转照目录标架写**，球心和半径不动。
///
/// [`sphere_geometry`] 只乘元素世界帧。ZONE 66457 的 11 只阀球（代表 `/1WCC189VN`，
/// `24383/66791`）E3D 的 Z 躺在 XY 里，我们写成 `(0,0,1)`：半径和 T 相同，列不同。
/// 球没有几何朝向，但这条矩阵是 E3D 记下来的目录标架。
pub fn sphere_geometry_in_frame(
    world: &glam::DMat4,
    sphere: &RecognisedSphere,
    frame: &glam::DMat4,
) -> AnalyticCandidate {
    // 球的放置矩阵有时是「平移 × 半径缩放」，三列不是单位轴。先丢掉缩放，只留方向。
    let raw = glam::DMat3::from_mat4(*frame);
    let z = raw.z_axis.normalize_or_zero();
    let mut x = raw.x_axis - z * raw.x_axis.dot(z);
    if z.length_squared() < 0.25 || x.length_squared() < 1e-8 {
        return sphere_geometry(world, sphere);
    }
    x = x.normalize();
    let rotation = glam::DMat3::from_cols(x, z.cross(x), z);
    let placed = *world
        * (glam::DMat4::from_translation(sphere.centre) * glam::DMat4::from_mat3(rotation));
    let cols = placed.to_cols_array();
    let matrix3 = [
        cols[0], cols[1], cols[2], cols[4], cols[5], cols[6], cols[8], cols[9], cols[10],
    ]
    .map(|value| value / M_TO_MM);
    let r = sphere.radius;
    AnalyticCandidate {
        geometry: RvmGeometry {
            tag: "PRIM".to_string(),
            flags: 1,
            primitive: Primitive::Sphere([r]),
            matrix3,
            translation_mm: [cols[12], cols[13], cols[14]],
            record_bbox: [-r, -r, -r, r, r, r],
            bbox_frame: crate::rvm_read::BboxFrame::Local,
            level: None,
        },
        sagitta_mm: sphere.sagitta_mm,
    }
}

/// 一只球形碟（`SphericalDish`）的识别结果，全在局部系 mm。
#[derive(Debug, Clone, Copy)]
pub struct RecognisedSphericalDish {
    /// 底圆半径。
    pub base_radius: f64,
    /// 底圆到顶点的高。
    pub height: f64,
    /// 底圆圆心（RVM 原点）。
    pub centre: glam::DVec3,
    /// 底面法向、指向顶点（RVM 的局部 +Z）。
    pub axis: glam::DVec3,
    /// 底环多边形的边数。
    pub segments: usize,
    /// 网格与解析碟之间系统差的上界，见 [`spherical_dish_from_local_mesh`]。
    pub sagitta_mm: f64,
}

/// 网格是一只球形碟（球冠 + 平底，[`crate::solid::spherical_dish_solid`] 的形体）时把它解出来。
///
/// 认法从**底盘**入手：它是网格里唯一一张平面圆盘，它的三角面共享同一个法向，数量至少
/// [`MIN_CIRCLE_SEGMENTS`]；球冠上的面片法向各不相同。所以按法向排序、取共面片最多的那一两组当候选：
///
/// 1. 顶点沿候选法向全在一侧，落在底面那一档的是底环（共圆、给出底圆半径与圆心）加至多一个盘心；
/// 2. 另一端只有**一个**顶点且在轴上，那是碟顶，给出高；
/// 3. 过底圆与碟顶、球心在轴上的球是唯一的：`R = (r² + h²) / 2h`，球心在底圆心上方 `h − R`；
///    除盘心以外的**每个**顶点都得在这只球上。带折边的碟（`EllipticalDish`）折边环上的顶点
///    不在冠球上，在这儿被挡回去。
///
/// 缺口两处都算、取大：底环是内接多边形（[`chord_sagitta_mm`]）；球冠是内接面片，
/// 凸体在任何方向的支撑值不小于它到球心最近的那张面片，所以 `R − min(面片到球心距离)` 是上界。
pub fn spherical_dish_from_local_mesh(
    vertices: &[[f64; 3]],
    triangles: &[[u32; 3]],
) -> Option<RecognisedSphericalDish> {
    use glam::DVec3;
    if vertices.len() < MIN_CIRCLE_SEGMENTS + 2 || triangles.len() < 2 * MIN_CIRCLE_SEGMENTS {
        return None;
    }
    let points: Vec<DVec3> = vertices.iter().copied().map(DVec3::from_array).collect();
    base_plane_candidates(&points, triangles)
        .into_iter()
        .find_map(|normal| spherical_dish_on_base(&points, triangles, normal))
}

/// 碟类网格共用的第一步：底盘那张平面圆盘的法向候选，按共面片数从多到少取前两个。
///
/// 底盘是网格里唯一一张平面圆盘，它的三角面共享同一个法向，数量至少
/// [`MIN_CIRCLE_SEGMENTS`]；曲面上的面片法向各不相同，凑不出这么大一组。
fn base_plane_candidates(points: &[glam::DVec3], triangles: &[[u32; 3]]) -> Vec<glam::DVec3> {
    use glam::DVec3;
    let mut normals: Vec<DVec3> = triangles
        .iter()
        .filter_map(|tri| {
            let [a, b, c] = tri.map(|i| points[i as usize]);
            let n = (b - a).cross(c - a);
            (n.length_squared() > 0.0).then(|| n.normalize())
        })
        .collect();
    normals.sort_by(|a, b| {
        a.x.total_cmp(&b.x)
            .then(a.y.total_cmp(&b.y))
            .then(a.z.total_cmp(&b.z))
    });
    let mut candidates: Vec<(DVec3, usize)> = Vec::new();
    let mut start = 0;
    for i in 1..=normals.len() {
        if i == normals.len() || (normals[i] - normals[start]).length() > 1e-7 {
            if i - start >= MIN_CIRCLE_SEGMENTS {
                candidates.push((normals[start], i - start));
            }
            start = i;
        }
    }
    candidates.sort_by_key(|(_, count)| std::cmp::Reverse(*count));
    candidates
        .into_iter()
        .take(2)
        .map(|(normal, _)| normal)
        .collect()
}

/// 碟类网格共用的第二步：底盘法向定了之后的底圆、碟顶与轴。曲面本身留给调用方判。
struct DishBase {
    centre: glam::DVec3,
    axis: glam::DVec3,
    base_radius: f64,
    height: f64,
    segments: usize,
    base_level: f64,
    levels: Vec<f64>,
}

impl DishBase {
    /// 每个顶点配一个「是不是盘心」的标记——盘心不在任何一段曲面上，判曲面时要跳过。
    fn surface_points<'a>(
        &'a self,
        points: &'a [glam::DVec3],
    ) -> impl Iterator<Item = (glam::DVec3, bool)> + 'a {
        points.iter().zip(self.levels.iter()).map(|(p, t)| {
            let on_base_hub =
                (*t - self.base_level).abs() <= 1e-6 && (*p - self.centre).length() <= 1e-6;
            (*p, on_base_hub)
        })
    }

    /// 顶点在碟自己那套柱坐标里的 `(离轴距, 离底面高)`。
    fn cylindrical(&self, p: glam::DVec3) -> (f64, f64) {
        let arm = p - self.centre;
        let axial = arm.dot(self.axis);
        ((arm - self.axis * axial).length(), axial)
    }
}

/// 底盘法向已定时解出底圆、碟顶与轴；不是碟的形状（两端都是整圈、只有两档高度、碟顶
/// 不唯一或不在轴上、底面不是圆盘）一律否掉。
fn dish_base_and_apex(points: &[glam::DVec3], normal: glam::DVec3) -> Option<DishBase> {
    use glam::DVec3;
    let levels: Vec<f64> = points.iter().map(|p| p.dot(normal)).collect();
    let (lowest, highest) = levels
        .iter()
        .fold((f64::MAX, f64::MIN), |(lo, hi), t| (lo.min(*t), hi.max(*t)));
    let count_at = |level: f64| levels.iter().filter(|t| (*t - level).abs() <= 1e-6).count();
    // 底盘坐在哪一端：整圈顶点的那一端。两端都是整圈的（圆柱）不是碟。
    let (axis, base_level, apex_level) = match (
        count_at(lowest) >= MIN_CIRCLE_SEGMENTS,
        count_at(highest) >= MIN_CIRCLE_SEGMENTS,
    ) {
        (true, false) => (normal, lowest, highest),
        (false, true) => (-normal, highest, lowest),
        _ => return None,
    };
    let height = (apex_level - base_level).abs();
    if !(height > 0.0) {
        return None;
    }
    // 碟的曲面在底环与碟顶之间至少还有一圈纬线。顶点只落两档的是**圆锥**：底环加锥顶
    // 恰好也共一只球，冠球那条判据挡不住它，得在这儿先否掉——圆锥归
    // [`snout_from_local_mesh`]。
    if !levels
        .iter()
        .any(|t| (*t - base_level).abs() > 1e-6 && (*t - apex_level).abs() > 1e-6)
    {
        return None;
    }

    let unique_at = |level: f64| -> Vec<DVec3> {
        let mut out: Vec<DVec3> = Vec::new();
        for (p, t) in points.iter().zip(levels.iter()) {
            if (*t - level).abs() <= 1e-6 && !out.iter().any(|q| (*q - *p).length() <= 1e-6) {
                out.push(*p);
            }
        }
        out
    };
    // 底环 + 至多一个盘心。
    let base = unique_at(base_level);
    let centre = base.iter().copied().sum::<DVec3>() / base.len() as f64;
    let base_radius = base
        .iter()
        .map(|p| (*p - centre).length())
        .fold(0.0, f64::max);
    if !(base_radius > 0.0) {
        return None;
    }
    let mut segments = 0usize;
    for p in &base {
        let d = (*p - centre).length();
        if (d - base_radius).abs() <= 1e-6 {
            segments += 1;
        } else if d > 1e-6 {
            return None; // 既不在环上也不是盘心，底面不是圆盘。
        }
    }
    if segments < MIN_CIRCLE_SEGMENTS {
        return None;
    }
    // 碟顶：唯一一个顶点，且在轴上。
    let apex = unique_at(apex_level);
    let [apex] = apex[..] else {
        return None;
    };
    let arm = apex - centre;
    if (arm - axis * arm.dot(axis)).length() > 1e-6 {
        return None;
    }
    Some(DishBase {
        centre,
        axis,
        base_radius,
        height,
        segments,
        base_level,
        levels,
    })
}

/// [`spherical_dish_from_local_mesh`] 的后半段：底盘法向已定，把碟解出来或否掉。
fn spherical_dish_on_base(
    points: &[glam::DVec3],
    triangles: &[[u32; 3]],
    normal: glam::DVec3,
) -> Option<RecognisedSphericalDish> {
    let base = dish_base_and_apex(points, normal)?;
    let (base_radius, height) = (base.base_radius, base.height);
    // 过底圆与碟顶、球心在轴上的球是唯一的：`R = (r² + h²) / 2h`，球心在底圆心上方 `h − R`；
    // 除盘心以外的**每个**顶点都得在这只球上。带折边的碟折边环上的顶点不在冠球上，在这儿
    // 被挡回去，归 [`elliptical_dish_from_local_mesh`]。
    let sphere_radius = (base_radius * base_radius + height * height) / (2.0 * height);
    let sphere_centre = base.centre + base.axis * (height - sphere_radius);
    if base.surface_points(points).any(|(p, on_base_hub)| {
        !on_base_hub && ((p - sphere_centre).length() - sphere_radius).abs() > 1e-6
    }) {
        return None;
    }

    // 缺口：底环的弦高，与球冠面片到球心的欠缺，取大。底盘那些片不算（它们本来就是平的）。
    let rim_gap = chord_sagitta_mm(base_radius, std::f64::consts::TAU / base.segments as f64);
    let mut cap_gap = 0.0f64;
    for tri in triangles {
        if tri
            .iter()
            .all(|i| (base.levels[*i as usize] - base.base_level).abs() <= 1e-6)
        {
            continue;
        }
        let [a, b, c] = tri.map(|i| points[i as usize]);
        let n = (b - a).cross(c - a);
        if n.length_squared() <= 0.0 {
            continue;
        }
        cap_gap = cap_gap.max(sphere_radius - n.normalize().dot(a - sphere_centre).abs());
    }
    Some(RecognisedSphericalDish {
        base_radius,
        height,
        centre: base.centre,
        axis: base.axis,
        segments: base.segments,
        sagitta_mm: rim_gap.max(cap_gap),
    })
}

/// 由 [`spherical_dish_from_local_mesh`] 认出来的碟造一条 `SphericalDish` 记录。
///
/// RVM 的 `SphericalDish`：参数 `[底圆半径, 高]`，原点在**底圆圆心**、底圆在 z = 0、
/// 顶点在 z = +h（E3D 实测：`DISH 1 of EQUIPMENT 1 of ZONE 3 of /Copy-of-泵棚B版`
/// 记录平移正是底圆心，世界盒 z 从平移起往上 100）。绕轴对称，局部 X 随便取一个。
pub fn spherical_dish_geometry(
    world: &glam::DMat4,
    dish: &RecognisedSphericalDish,
) -> AnalyticCandidate {
    dish_geometry(
        world,
        dish.axis,
        dish.centre,
        Primitive::SphericalDish([dish.base_radius, dish.height]),
        dish.sagitta_mm,
    )
}

/// 两种碟共用的记录：原点放在底圆心、局部 +Z 沿轴指向碟顶，绕轴对称所以局部 X 随便取一个。
fn dish_geometry(
    world: &glam::DMat4,
    axis: glam::DVec3,
    centre: glam::DVec3,
    primitive: Primitive,
    sagitta_mm: f64,
) -> AnalyticCandidate {
    let z = axis;
    let x = z.any_orthonormal_vector();
    let y = z.cross(x);
    let frame = glam::DMat4::from_cols(
        x.extend(0.0),
        y.extend(0.0),
        z.extend(0.0),
        centre.extend(1.0),
    );
    let placed = *world * frame;
    let cols = placed.to_cols_array();
    let matrix3 = [
        cols[0], cols[1], cols[2], cols[4], cols[5], cols[6], cols[8], cols[9], cols[10],
    ]
    .map(|value| value / M_TO_MM);
    let record_bbox = primitive.local_aabb_mm().unwrap_or([0.0; 6]);
    AnalyticCandidate {
        geometry: RvmGeometry {
            tag: "PRIM".to_string(),
            flags: 1,
            primitive,
            matrix3,
            translation_mm: [cols[12], cols[13], cols[14]],
            record_bbox,
            bbox_frame: crate::rvm_read::BboxFrame::Local,
            level: None,
        },
        sagitta_mm,
    }
}

/// 一只折边碟（RVM 的 `EllipticalDish`）的识别结果，全在局部系 mm。
#[derive(Debug, Clone, Copy)]
pub struct RecognisedEllipticalDish {
    /// 底圆半径。
    pub base_radius: f64,
    /// 底圆到顶点的高。
    pub height: f64,
    /// 底圆圆心（RVM 原点）。
    pub centre: glam::DVec3,
    /// 底面法向、指向顶点（RVM 的局部 +Z）。
    pub axis: glam::DVec3,
    /// 底环多边形的边数。
    pub segments: usize,
    /// 网格与解析碟之间系统差的上界，见 [`elliptical_dish_from_local_mesh`]。
    pub sagitta_mm: f64,
}

/// 网格是一只折边碟（[`crate::solid::elliptical_dish_solid`] 的形体）时把它解出来。
///
/// E3D 所谓 elliptical dish 不是椭球，是**冠球 + 折边环**的 torispherical 头：冠球半径
/// Rc =（`GM_EDish::radiusOfHub`），折边是绕 z 轴、管半径 k、中心圆半径 r − k 的环
/// （`GM_EDish::calcFacetsWithoutSurfaces`，libgm `0x10054ab0`）。两段在极角
/// φt = acos(1 − (h − k)/(Rc − k)) 处相切。
///
/// 折边半径不进 RVM，也不必从网格量：目录那条路（CRCATI 碟形臂 → `CREDSH`）的 `PRAD`
/// 只当开关，k 由 r / h 算出（[`crate::solid::dish_knuckle_radius`] 再过
/// `knuckleRadiusToUse` 的夹取）。所以这里拿底圆半径与高把 k、Rc **重算**一遍，再要求
/// 每个顶点都落在冠球或折边环上——量不出来的东西不猜，猜错了就是一只解析得开、形状却
/// 不对的碟。
///
/// 顺序上它排在 [`spherical_dish_from_local_mesh`] 后面：球形碟的顶点全在冠球上，先被
/// 那一条认走；折边碟在那里因折边环的顶点不在冠球上被挡回来，才轮到这里。
///
/// **认不出半球形的折边碟。** r 与 h 相等时 `GM_EDish` 退化成整个半球
/// （`isSpherical`），与 `GM_SDish` 逐点相同，上一条会先把它认成 `SphericalDish`；
/// 而 E3D 写的是 `EllipticalDish`——`expdri/DRDISH`（`0x10003fb8`）只看折边半径是不是
/// 零来选 5（椭圆碟）还是 6（球形碟），不看形状。网格里没有这个信息，只能等写侧能拿到
/// 模型那一侧的碟种类时才分得开。
pub fn elliptical_dish_from_local_mesh(
    vertices: &[[f64; 3]],
    triangles: &[[u32; 3]],
) -> Option<RecognisedEllipticalDish> {
    use glam::DVec3;
    if vertices.len() < MIN_CIRCLE_SEGMENTS + 2 || triangles.len() < 2 * MIN_CIRCLE_SEGMENTS {
        return None;
    }
    let points: Vec<DVec3> = vertices.iter().copied().map(DVec3::from_array).collect();
    base_plane_candidates(&points, triangles)
        .into_iter()
        .find_map(|normal| elliptical_dish_on_base(&points, triangles, normal))
}

/// [`elliptical_dish_from_local_mesh`] 的后半段：底盘法向已定，把碟解出来或否掉。
fn elliptical_dish_on_base(
    points: &[glam::DVec3],
    triangles: &[[u32; 3]],
    normal: glam::DVec3,
) -> Option<RecognisedEllipticalDish> {
    let base = dish_base_and_apex(points, normal)?;
    let (base_radius, height) = (base.base_radius, base.height);
    // 半球那一档归球形碟认（见函数文档）：这里认下来只会写出同一个形状、另一个种类。
    if (base_radius - height).abs() <= 1e-6 {
        return None;
    }
    // 折边与冠球按建模那一侧同一条公式重算，不从网格里猜。
    let knuckle = crate::solid::edish_knuckle_to_use(
        base_radius,
        height,
        crate::solid::dish_knuckle_radius(base_radius, height),
    );
    let hub = crate::solid::edish_radius_of_hub(base_radius, height, knuckle);
    if !hub.is_finite()
        || !(knuckle > 0.0)
        || (hub - knuckle).abs() <= f64::EPSILON
        || (height - knuckle).abs() <= f64::EPSILON
    {
        return None;
    }
    // 冠球心在轴上 h − Rc 处；折边环的中心圆半径 r − k、在底面上。
    let hub_centre_z = height - hub;
    let knuckle_ring = base_radius - knuckle;
    // 点到冠球面 / 折边环面的距离：顶点判「在面上」用，面片的缺口也用它量。
    let off_surface = |p: glam::DVec3| {
        let (radial, axial) = base.cylindrical(p);
        let on_hub = ((radial * radial + (axial - hub_centre_z).powi(2)).sqrt() - hub).abs();
        let on_knuckle = (((radial - knuckle_ring).powi(2) + axial * axial).sqrt() - knuckle).abs();
        on_hub.min(on_knuckle)
    };
    for (p, on_base_hub) in base.surface_points(points) {
        if on_base_hub {
            continue; // 盘心不在任何一段曲面上。
        }
        if off_surface(p) > 1e-6 {
            return None;
        }
    }
    // 缺口：底环的弦高，与曲面面片到冠球 / 折边环的欠缺，取大。顶点都在面上，
    // 面片是内接的，欠缺最大处在片中间——拿重心与三条边的中点量。底盘那些片不算。
    let rim_gap = chord_sagitta_mm(base_radius, std::f64::consts::TAU / base.segments as f64);
    let mut cap_gap = 0.0f64;
    for tri in triangles {
        if tri
            .iter()
            .all(|i| (base.levels[*i as usize] - base.base_level).abs() <= 1e-6)
        {
            continue;
        }
        let [a, b, c] = tri.map(|i| points[i as usize]);
        for q in [
            (a + b + c) / 3.0,
            (a + b) / 2.0,
            (b + c) / 2.0,
            (c + a) / 2.0,
        ] {
            cap_gap = cap_gap.max(off_surface(q));
        }
    }
    Some(RecognisedEllipticalDish {
        base_radius,
        height,
        centre: base.centre,
        axis: base.axis,
        segments: base.segments,
        sagitta_mm: rim_gap.max(cap_gap),
    })
}

/// 由 [`elliptical_dish_from_local_mesh`] 认出来的碟造一条 `EllipticalDish` 记录。
///
/// RVM 的 `EllipticalDish` 与 `SphericalDish` 同形：参数 `[底圆半径, 高]`，原点在底圆
/// 圆心、底圆在 z = 0、顶点在 z = +h。折边半径不落盘——`expdri/DRDISH` 只写这两个数，
/// 也不必落：k 是 r / h 的函数（`CREDSH`），读侧从 (r, h) 就能把冠球 + 折边环原样还原
/// （[`crate::rvm_read::Primitive::local_hull_points_mm`] 2026-09-19 起正是这么读的，
/// 与 E3D 的 `GM_EDish` 同形），所以落盘这一步**没有形状损失**；自校闸放行的只有面片的
/// 内接缺口（底环弦高与冠 / 折边面片到曲面的欠缺，取大），与其它曲面图元一个口径。
/// rvm-rs 一类阅览器把这条记录画成半椭球，那是阅览器的近似，不进闸门。
pub fn elliptical_dish_geometry(
    world: &glam::DMat4,
    dish: &RecognisedEllipticalDish,
) -> AnalyticCandidate {
    dish_geometry(
        world,
        dish.axis,
        dish.centre,
        Primitive::EllipticalDish([dish.base_radius, dish.height]),
        dish.sagitta_mm,
    )
}

/// 生成侧的 `HEAD` / `MODL`。banner 刻意不像 E3D，好让快照一眼看出来源。
pub fn generated_header(
    note: impl Into<String>,
    model: impl Into<String>,
    project: impl Into<String>,
) -> RvmHeader {
    RvmHeader {
        banner: format!(
            "e3d-model {} (generated, not an AVEVA export)",
            env!("CARGO_PKG_VERSION")
        ),
        note: note.into(),
        date: String::new(),
        user: whoami(),
        encoding: "Unicode UTF-8".to_string(),
        model: model.into(),
        project: project.into(),
    }
}

fn whoami() -> String {
    std::env::var("USERNAME")
        .or_else(|_| std::env::var("USER"))
        .unwrap_or_else(|_| "unknown".to_string())
}

/// 祖先容器：平移全 0，只提供层级。
pub fn container_member(
    name: impl Into<String>,
    path: impl Into<String>,
    parent: Option<usize>,
) -> RvmMember {
    RvmMember {
        name: name.into(),
        path: path.into(),
        parent,
        translation_mm: [0.0; 3],
        material: 1,
        geometries: Vec::new(),
    }
}

fn chunk(out: &mut Vec<u8>, tag: &str, body: &[u8]) {
    for c in tag.chars() {
        push_u32(out, c as u32);
    }
    let next = out.len() + 4 + body.len();
    push_u32(out, next as u32);
    out.extend_from_slice(body);
}

/// u32 字数（4 字节一字）+ NUL 补齐的字节。
///
/// 字数是 `ceil(len / 4)`，**长度正好是 4 的倍数时不补 NUL**——E3D 的 `/ALL`
/// 就是 1 个字、4 个字节、没有终止符。读侧按 NUL 切，两种都吃得下，
/// 但字数算错一位整条链就错位了。
fn push_string(out: &mut Vec<u8>, text: &str) {
    let bytes = text.as_bytes();
    let words = bytes.len().div_ceil(4);
    push_u32(out, words as u32);
    out.extend_from_slice(bytes);
    out.resize(out.len() + words * 4 - bytes.len(), 0);
}

fn push_u32(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_be_bytes());
}

fn push_f32(out: &mut Vec<u8>, value: f32) {
    out.extend_from_slice(&value.to_bits().to_be_bytes());
}

fn push_f32s(out: &mut Vec<u8>, values: &[f64]) {
    for value in values {
        push_f32(out, *value as f32);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rvm_read::BboxFrame;

    /// 网格摆到 `world` 之后的世界 AABB，自校闸的另一半判据。
    fn world_aabb_of(vertices: &[[f64; 3]], world: &glam::DMat4) -> ([f64; 3], [f64; 3]) {
        vertices.iter().fold(
            ([f64::MAX; 3], [f64::MIN; 3]),
            |(mut lo, mut hi), vertex| {
                let w = (*world * glam::DVec3::from_array(*vertex).extend(1.0)).truncate();
                for axis in 0..3 {
                    lo[axis] = lo[axis].min(w[axis]);
                    hi[axis] = hi[axis].max(w[axis]);
                }
                (lo, hi)
            },
        )
    }

    fn sample() -> RvmFile {
        // /SITE → /ZONE → BOX 1 of EQUIPMENT /E1（带一个单位立方的 FacetGroup）
        let mut members = vec![
            container_member("/SITE", "/SITE", None),
            container_member("/ZONE", "/SITE/ZONE", Some(0)),
        ];

        let local: Vec<[f64; 3]> = vec![
            [-50.0, -25.0, -15.0],
            [50.0, -25.0, -15.0],
            [50.0, 25.0, -15.0],
            [-50.0, 25.0, -15.0],
        ];
        let triangles = [[0u32, 1, 2], [0, 2, 3]];
        // 绕 Z 转 90°：列主序 [c0, c1, c2]。
        let rotation = [0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0];
        let geometry = facet_group_from_world_triangles(
            rotation,
            [1000.0, 2000.0, 3000.0],
            &local,
            &triangles,
        );

        members.push(RvmMember {
            name: "BOX 1 of EQUIPMENT /E1".to_string(),
            path: "/SITE/ZONE/BOX 1 of EQUIPMENT /E1".to_string(),
            parent: Some(1),
            translation_mm: [1000.0, 2000.0, 3000.0],
            material: 1,
            geometries: vec![geometry],
        });

        RvmFile {
            header: generated_header("round-trip fixture", "AMS", "/ALL"),
            members,
        }
    }

    #[test]
    fn round_trips_through_the_reader() {
        let written = sample();
        let read = RvmFile::parse(&written.to_bytes()).unwrap();

        assert_eq!(read.header.model, "AMS");
        assert_eq!(read.header.project, "/ALL");
        assert_eq!(read.header.encoding, "Unicode UTF-8");
        assert_eq!(read.members.len(), written.members.len());

        for (got, want) in read.members.iter().zip(written.members.iter()) {
            assert_eq!(got.name, want.name);
            assert_eq!(got.parent, want.parent);
            assert_eq!(got.geometries.len(), want.geometries.len());
            for axis in 0..3 {
                assert!((got.translation_mm[axis] - want.translation_mm[axis]).abs() < 1e-3);
            }
        }
    }

    #[test]
    fn written_records_satisfy_the_rotated_frame_invariant() {
        // 这正是 E3D 的 228 条记录满足的那条不变量，我们写出来的也必须满足，
        // 否则读侧会把它判成 `Unknown`。
        let read = RvmFile::parse(&sample().to_bytes()).unwrap();
        let geometry = &read.members[2].geometries[0];
        assert_eq!(geometry.bbox_frame, BboxFrame::Rotated);
        assert_eq!(read.bbox_frame_tally().get("unknown"), None);
    }

    #[test]
    fn world_aabb_survives_the_round_trip() {
        let written = sample();
        let read = RvmFile::parse(&written.to_bytes()).unwrap();
        let before = written.members[2].world_aabb_mm().unwrap();
        let after = read.members[2].world_aabb_mm().unwrap();
        for axis in 0..6 {
            assert!(
                (before[axis] - after[axis]).abs() < 1e-3,
                "{before:?} vs {after:?}"
            );
        }
        // 局部 100×50 的板绕 Z 转 90° 之后，X 跨度变 50、Y 跨度变 100。
        assert!((after[3] - after[0] - 50.0).abs() < 1e-3, "{after:?}");
        assert!((after[4] - after[1] - 100.0).abs() < 1e-3, "{after:?}");
    }

    #[test]
    fn cnte_count_balances_cntb() {
        let bytes = sample().to_bytes();
        let count = |needle: &str| {
            let tag: Vec<u8> = needle
                .chars()
                .flat_map(|c| (c as u32).to_be_bytes())
                .collect();
            bytes.windows(16).filter(|w| *w == tag.as_slice()).count()
        };
        assert_eq!(count("CNTB"), 3);
        assert_eq!(count("CNTE"), 3);
    }

    /// 拿 `circ_torus_solid` 自己造一段弯头再认回来——真值是已知的，不用靠肉眼。
    #[test]
    fn recognises_a_quarter_bend_built_by_the_solid_kernel() {
        // circ_torus_solid(inner, outer, angle_deg)：管心半径 = (inner+outer)/2，
        // 管截面半径 = (outer−inner)/2。
        let (inner, outer, sweep_deg) = (95.0_f64, 105.0_f64, 90.0_f64);
        let solid = crate::solid::circ_torus_solid(inner, outer, sweep_deg).unwrap();
        let mesh = crate::mesh::TriMesh::from_manifold(&solid);
        let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();

        let torus = torus_from_local_mesh(&mesh.vertices, &triangles)
            .expect("内核造出来的弯头必须认得回来");
        assert!(
            (torus.offset - (inner + outer) / 2.0).abs() < 1e-6,
            "offset={} ",
            torus.offset
        );
        assert!(
            (torus.radius - (outer - inner) / 2.0).abs() < 1e-6,
            "radius={}",
            torus.radius
        );
        assert!(
            (torus.angle.to_degrees() - sweep_deg).abs() < 1e-6,
            "angle={}",
            torus.angle.to_degrees()
        );
    }

    /// `RINS = 0` 的弯头内缘贴轴。顶点被夹到轴上之后，变换还会留下 ~1e-12 mm 的
    /// 平面分量；那个方向没有意义，量角必须把它丢掉，否则 90° 会被抬成 270°。
    #[test]
    fn an_on_axis_pinch_vertex_does_not_inflate_the_sweep() {
        let solid = crate::solid::circ_torus_solid(0.0, 33.0, 90.0).unwrap();
        let mesh = crate::mesh::TriMesh::from_manifold(&solid);
        let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();
        let clean = torus_from_local_mesh(&mesh.vertices, &triangles).expect("贴轴弯头");
        assert!(
            (clean.angle.to_degrees() - 90.0).abs() < 1e-3,
            "clean angle={}",
            clean.angle.to_degrees()
        );
        assert!((clean.offset - 16.5).abs() < 1e-6, "{}", clean.offset);
        assert!((clean.radius - 16.5).abs() < 1e-6, "{}", clean.radius);

        let mut vertices = mesh.vertices.clone();
        let (index, rho) = vertices
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let w = glam::DVec3::from_array(*p) - clean.centre;
                let planar = w - clean.axis * w.dot(clean.axis);
                (i, planar.length())
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .unwrap();
        assert!(rho < 1e-6, "内核应有贴轴顶点，ρ={rho}");
        // 从起点往扫掠反方向拨 1e-12 mm。旧算法的 atan2 会把这一下记成将近 360°。
        let backward = clean.axis.cross(clean.start).normalize();
        let nudged = glam::DVec3::from_array(vertices[index]) - backward * 1e-12;
        vertices[index] = nudged.to_array();
        let noisy = torus_from_local_mesh(&vertices, &triangles).expect("噪声贴轴顶点仍是弯头");
        assert!(
            (noisy.angle.to_degrees() - 90.0).abs() < 1e-3,
            "noisy angle={} ρ={rho}",
            noisy.angle.to_degrees()
        );
        assert!((noisy.offset - 16.5).abs() < 1e-6, "{}", noisy.offset);
        assert!((noisy.radius - 16.5).abs() < 1e-6, "{}", noisy.radius);
    }

    #[test]
    fn a_straight_cylinder_is_not_mistaken_for_a_bend() {
        // 两个端盖平行 → 轴叉乘退化 → 必须拒绝，否则直管会被写成一段包角为 0 的弯头。
        let solid = crate::solid::csg_cylinder_solid(100.0, 400.0).unwrap();
        let mesh = crate::mesh::TriMesh::from_manifold(&solid);
        let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();
        assert!(torus_from_local_mesh(&mesh.vertices, &triangles).is_none());
        // 而圆柱识别必须认得出它。
        let cylinder =
            cylinder_from_local_mesh(&mesh.vertices, &triangles).expect("直管应认成圆柱");
        assert!(
            (cylinder.radius - 50.0).abs() < 1e-6,
            "radius={}",
            cylinder.radius
        );
        assert!(
            (cylinder.height - 400.0).abs() < 1e-6,
            "height={}",
            cylinder.height
        );
    }

    /// 半圈弯头的两个端盖反向平行，`n1 × n2` 退化——`1RCV002VP` 的两条 π 弯就是这么
    /// 被当成直管丢掉的。轴改由「法向 × 盖心连线」定，这条钉住它。
    #[test]
    fn a_half_turn_bend_is_not_mistaken_for_a_straight_tube() {
        let (inner, outer) = (152.5_f64, 162.5_f64);
        let solid = crate::solid::circ_torus_solid(inner, outer, 180.0).unwrap();
        let mesh = crate::mesh::TriMesh::from_manifold(&solid);
        let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();
        let torus = torus_from_local_mesh(&mesh.vertices, &triangles)
            .expect("包角 180° 的弯头必须认得回来");
        assert!(
            (torus.angle - std::f64::consts::PI).abs() < 1e-6,
            "angle={}",
            torus.angle.to_degrees()
        );
        assert!(
            (torus.offset - (inner + outer) / 2.0).abs() < 1e-6,
            "{torus:?}"
        );
        assert!(
            (torus.radius - (outer - inner) / 2.0).abs() < 1e-6,
            "{torus:?}"
        );
    }

    /// 拿 `rect_torus_solid` 自己造几段矩形环再认回来——半圈（`/TT-0301` 那一族的 180°）、
    /// 四分之一、过半（270°：包角得取实体走过的那一侧，不是端盖夹的锐角）、整圈、贴轴。
    #[test]
    fn recognises_rectangular_tori_built_by_the_solid_kernel() {
        for (rins, rout, height, sweep_deg) in [
            (45.0_f64, 55.0_f64, 25.0_f64, 180.0_f64),
            (200.0, 260.0, 40.0, 90.0),
            (300.0, 500.0, 200.0, 270.0),
            (90.0, 110.0, 10.0, 360.0),
            (0.0, 50.0, 20.0, 120.0),
        ] {
            let solid = crate::solid::rect_torus_solid(rins, rout, height, sweep_deg).unwrap();
            let mesh = crate::mesh::TriMesh::from_manifold(&solid);
            let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();
            let torus = rect_torus_from_local_mesh(&mesh.vertices, &triangles)
                .unwrap_or_else(|| panic!("矩形环应认得回来: {rins}/{rout}/{height}/{sweep_deg}"));
            assert!((torus.inner - rins).abs() < 1e-6, "{torus:?}");
            assert!((torus.outer - rout).abs() < 1e-6, "{torus:?}");
            assert!((torus.height - height).abs() < 1e-6, "{torus:?}");
            assert!(
                (torus.angle.to_degrees() - sweep_deg).abs() < 1e-6,
                "{sweep_deg}°: {torus:?}"
            );
            assert!(torus.centre.length() < 1e-6, "{torus:?}");
            assert!((torus.axis.z.abs() - 1.0).abs() < 1e-9, "{torus:?}");

            // 写出来的记录必须过自校闸——起点或轴向取反了，部分环的 AABB 立刻翻到另一侧。
            let world = glam::DMat4::from_rotation_z(0.4) * glam::DMat4::from_rotation_x(0.7);
            let (lo, hi) = world_aabb_of(&mesh.vertices, &world);
            let candidate = rectangular_torus_geometry(&world, &torus);
            assert!(
                candidate.matches_mesh_aabb(lo, hi, 0.01),
                "{rins}/{rout}/{height}/{sweep_deg}: sagitta={} 候选={:?} 网格=({lo:?}, {hi:?})",
                candidate.sagitta_mm,
                candidate.world_aabb_mm()
            );
            assert_eq!(candidate.geometry.primitive.kind_id(), 3);
        }
    }

    /// 弯头、直管、盒子都不是矩形环：弯头的顶点不落两档，直管与盒子的顶面顶点只有一个半径。
    #[test]
    fn other_shapes_are_not_mistaken_for_a_rectangular_torus() {
        for solid in [
            crate::solid::circ_torus_solid(95.0, 105.0, 90.0).unwrap(),
            crate::solid::csg_cylinder_solid(100.0, 400.0).unwrap(),
            crate::solid::box_solid(100.0, 60.0, 40.0).unwrap(),
        ] {
            let mesh = crate::mesh::TriMesh::from_manifold(&solid);
            let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();
            assert!(rect_torus_from_local_mesh(&mesh.vertices, &triangles).is_none());
        }
    }

    /// 轴正好躺在坐标面内时，端盖法向理论为 0 的那个分量只剩舍入噪声，符号不能让它定。
    /// ams7999 `/1WCC318VN` 手轮两根辐条（构件局部 XY 面内）此前一根同向、一根反向地
    /// 写进 RVM，就是 `z < 0.0` 这一判被 ±1e-16 的噪声左右了。
    #[test]
    fn axis_sign_is_not_decided_by_rounding_noise() {
        use glam::DVec3;
        let eps = 1e-17;
        // (1,−1,0) × (ε,ε,1) = (−1, −1, 2ε)：z 是 +2e-17 的噪声。旧判法「z > 0 就保留」
        // 留下 y < 0 的那一侧；按 y 定符号该翻到 (+, +, ·)。
        let points = [
            DVec3::ZERO,
            DVec3::new(1.0, -1.0, 0.0),
            DVec3::new(eps, eps, 1.0),
        ];
        let [axis] = face_normal_axes(&points, &[[0, 1, 2]])[..] else {
            panic!("one cap normal");
        };
        assert!(axis.x > 0.0 && axis.y > 0.0, "{axis:?}");
        // 噪声换个符号，答案不变。
        let points = [
            DVec3::ZERO,
            DVec3::new(1.0, -1.0, 0.0),
            DVec3::new(-eps, -eps, 1.0),
        ];
        let [other] = face_normal_axes(&points, &[[0, 1, 2]])[..] else {
            panic!("one cap normal");
        };
        assert!((axis - other).length() < 1e-9, "{axis:?} vs {other:?}");
        // 真有 z 分量时照旧由 z 说了算；z、y 都在容差内才轮到 x。
        assert_eq!(
            canonical_axis_sign(DVec3::new(0.3, 0.4, -0.5)),
            DVec3::new(-0.3, -0.4, 0.5)
        );
        assert_eq!(
            canonical_axis_sign(DVec3::new(-1.0, 1e-12, -1e-12)),
            DVec3::new(1.0, -1e-12, 1e-12)
        );
    }

    #[test]
    fn recognises_a_snout_built_by_the_solid_kernel() {
        // 同心的与偏心的各来一只：`1RCV002VP` 两种都有（XOFF 12.06 那两条是偏心的）。
        for (dbot, dtop, height, xoff) in [(66.33, 60.30, 14.4, 0.0), (66.33, 84.42, 115.2, 12.06)]
        {
            let solid = crate::solid::snout_solid(dbot, dtop, height, xoff, 0.0).unwrap();
            let mesh = crate::mesh::TriMesh::from_manifold(&solid);
            let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();
            let snout = snout_from_local_mesh(&mesh.vertices, &triangles)
                .unwrap_or_else(|| panic!("圆台应认得回来: {dbot}/{dtop}/{height}/{xoff}"));
            // 轴向由面法向定，可能与内核的 +Z 反向；反过来时两端半径跟着换个个儿。
            let (rb, rt) = if snout.axis.z >= 0.0 {
                (snout.bottom_radius, snout.top_radius)
            } else {
                (snout.top_radius, snout.bottom_radius)
            };
            assert!((rb - dbot / 2.0).abs() < 1e-6, "rb={rb}");
            assert!((rt - dtop / 2.0).abs() < 1e-6, "rt={rt}");
            assert!((snout.height - height).abs() < 1e-6, "h={}", snout.height);
            assert!((snout.offset - xoff).abs() < 1e-6, "off={}", snout.offset);

            // 写出来的记录必须过自校闸：偏心那只的 XOFF 写丢了，AABB 立刻差 12 mm。
            let world = glam::DMat4::from_rotation_z(0.4) * glam::DMat4::from_rotation_x(0.7);
            let (lo, hi) = world_aabb_of(&mesh.vertices, &world);
            let candidate = snout_geometry(&world, &snout);
            assert!(
                candidate.matches_mesh_aabb(lo, hi, 0.01),
                "{dbot}/{dtop}/{height}/{xoff}: sagitta={}",
                candidate.sagitta_mm
            );
        }
    }

    /// `/OILSTD-COUPLING-QUICKFIT-SWF` 的 SCON：E3D 写 `Snout[rb=0, rt=33, h=42]`，锥顶在底。
    /// 我们的圆锥网格是 `snout_solid(dbot, 0, h)`——锥顶在内核的 +Z——认回来后记录里
    /// 锥顶照样得在底端，且不论内核把它放在哪一头。
    #[test]
    fn recognises_a_cone_as_a_snout_with_the_apex_at_the_bottom() {
        for (dbot, height) in [(66.0, 42.0), (114.0, 60.0)] {
            let solid = crate::solid::snout_solid(dbot, 0.0, height, 0.0, 0.0).unwrap();
            let mesh = crate::mesh::TriMesh::from_manifold(&solid);
            let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();
            let snout = snout_from_local_mesh(&mesh.vertices, &triangles)
                .unwrap_or_else(|| panic!("圆锥应认成半径 0 的圆台: {dbot}/0/{height}"));
            assert_eq!(snout.bottom_radius, 0.0, "{snout:?}");
            assert!((snout.top_radius - dbot / 2.0).abs() < 1e-6, "{snout:?}");
            assert!((snout.height - height).abs() < 1e-6, "{snout:?}");
            assert!(snout.offset.abs() < 1e-9, "{snout:?}");
            // 内核的锥顶在 +Z：底 → 顶的轴必须指向 −Z，锥顶才落在底端。
            assert!(snout.axis.z < -0.999, "{snout:?}");
            assert!(snout.segments >= MIN_CIRCLE_SEGMENTS, "{snout:?}");

            let world = glam::DMat4::from_rotation_z(0.4) * glam::DMat4::from_rotation_x(0.7);
            let (lo, hi) = world_aabb_of(&mesh.vertices, &world);
            let candidate = snout_geometry(&world, &snout);
            assert!(
                matches!(candidate.geometry.primitive, Primitive::Snout(p) if p[0] == 0.0 && (p[1] - dbot / 2.0).abs() < 1e-6),
                "{:?}",
                candidate.geometry.primitive
            );
            assert!(
                candidate.matches_mesh_aabb(lo, hi, 0.01),
                "{dbot}/0/{height}: sagitta={}",
                candidate.sagitta_mm
            );
            // 圆锥不是碟：底环加锥顶恰好共球，靠「冠面至少一圈纬线」这条挡下。
            assert!(
                spherical_dish_from_local_mesh(&mesh.vertices, &triangles).is_none(),
                "{dbot}/0/{height} 被认成了碟"
            );
        }
    }

    #[test]
    fn a_cylinder_is_not_offered_as_a_snout() {
        // 两端等径又不偏心的是圆柱：`Cylinder` 那条记录更贴 E3D，圆台识别器必须让路。
        let solid = crate::solid::csg_cylinder_solid(100.0, 400.0).unwrap();
        let mesh = crate::mesh::TriMesh::from_manifold(&solid);
        let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();
        assert!(snout_from_local_mesh(&mesh.vertices, &triangles).is_none());
    }

    #[test]
    fn a_bodyless_cylinder_keeps_its_placement_and_whatever_parameters_it_had() {
        // 没有体也要有位置与参数：`/Copy-(2)-of-GV0406C` 那条是 [0, 0]，`/LA-0301` 第 45 条
        // 是 [57, 0]——半径照留，靠这一对与放置才对得上号。
        let world = glam::DMat4::from_rotation_z(0.4) * glam::DMat4::from_rotation_x(0.7);
        let local = glam::DMat4::from_translation(glam::DVec3::new(11.0, -22.0, 33.0));
        let want = (world * local).w_axis.truncate();
        for (radius, height) in [(0.0, 0.0), (57.0, 0.0)] {
            let record = bodyless_cylinder_geometry(&world, &local, radius, height);
            assert!(
                matches!(record.primitive, Primitive::Cylinder([r, h]) if r == radius && h == height),
                "{record:?}"
            );
            assert_eq!(
                record.record_bbox,
                [-radius, -radius, 0.0, radius, radius, 0.0]
            );
            for axis in 0..3 {
                assert!(
                    (record.translation_mm[axis] - want[axis]).abs() < 1e-9,
                    "{record:?}"
                );
            }
        }
    }

    #[test]
    fn a_cylindrical_snout_is_kept_when_the_catalogue_says_it_came_from_an_lsno() {
        // 同一只网格，按长相让给圆柱、按出身照认圆台：E3D 的 DRSNO 不看 LSNO 退化成什么。
        let solid = crate::solid::csg_cylinder_solid(100.0, 400.0).unwrap();
        let mesh = crate::mesh::TriMesh::from_manifold(&solid);
        let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();
        let snout = snout_from_local_mesh_including_cylinders(&mesh.vertices, &triangles)
            .expect("出身是 LSNO 的等径圆台");
        assert!((snout.bottom_radius - 50.0).abs() < 1e-6, "{snout:?}");
        assert!((snout.top_radius - 50.0).abs() < 1e-6, "{snout:?}");
        assert!((snout.height - 400.0).abs() < 1e-6, "{snout:?}");
        assert!(snout.offset.abs() < 1e-9, "{snout:?}");

        let world = glam::DMat4::from_rotation_z(0.4) * glam::DMat4::from_rotation_x(0.7);
        let (lo, hi) = world_aabb_of(&mesh.vertices, &world);
        let candidate = snout_geometry(&world, &snout);
        assert!(
            candidate.matches_mesh_aabb(lo, hi, 0.01),
            "sagitta={}",
            candidate.sagitta_mm
        );
    }

    #[test]
    fn chord_sagitta_is_the_gap_between_an_inscribed_polygon_and_its_circle() {
        // 独立量法：取圆上相邻两个格点，弦中点离圆心多远，缺口就是半径减它。
        let (radius, segments) = (5.0_f64, 8.0_f64);
        let step = std::f64::consts::TAU / segments;
        let a = glam::DVec2::new(radius, 0.0);
        let b = glam::DVec2::new(radius * step.cos(), radius * step.sin());
        let measured = radius - ((a + b) / 2.0).length();
        assert!((chord_sagitta_mm(radius, step) - measured).abs() < 1e-12);
        // 退化输入不该把容差算成 NaN 交出去——那会让自校闸的比较恒为 false。
        assert_eq!(chord_sagitta_mm(radius, 0.0), 0.0);
        assert_eq!(chord_sagitta_mm(0.0, step), 0.0);
        assert_eq!(chord_sagitta_mm(radius, f64::INFINITY), 0.0);
    }

    /// 自校闸放行的尺度必须是这块网格自己的弦高，不是一个常数。
    ///
    /// 这是 `1RCV0101` 那 14 根真圆柱被退回面片的复现：它们几何没错，只是网格内接、
    /// 解析面外接，AABB 天生差一个弦高，而闸门拿 0.01 mm 判。
    #[test]
    fn a_rotated_cylinder_clears_the_gate_by_its_own_chord_height() {
        // r = 5 → 8 段（段数规则的 45° 封顶那一档），弦高 5·(1−cos(π/8)) ≈ 0.381 mm。
        let solid = crate::solid::csg_cylinder_solid(10.0, 200.0).unwrap();
        let mesh = crate::mesh::TriMesh::from_manifold(&solid);
        let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();
        let cylinder = cylinder_from_local_mesh(&mesh.vertices, &triangles).unwrap();
        assert_eq!(cylinder.segments, 8);
        assert_eq!(
            cylinder.segments as i32,
            crate::solid::cylinder_segments(5.0)
        );

        // 段数规则把 0/90/180/270 铺在网格上，所以轴对齐时 AABB 一点不差；缺口是转起来
        // 才有的，而现场没有一根管子是轴对齐的。
        let world = glam::DMat4::from_rotation_z(0.4) * glam::DMat4::from_rotation_x(0.7);
        let (lo, hi) = world_aabb_of(&mesh.vertices, &world);

        let candidate = cylinder_geometry(&world, &cylinder);
        assert!(
            (candidate.sagitta_mm - 5.0 * (1.0 - (std::f64::consts::PI / 8.0).cos())).abs() < 1e-12
        );
        assert!(
            candidate.matches_mesh_aabb(lo, hi, 0.01),
            "sagitta={} 的圆柱必须过闸",
            candidate.sagitta_mm
        );
        // 同一个候选，把弦高抹掉就是改动前的闸门——它退回面片，这条才是回归。
        let fixed_tolerance = AnalyticCandidate {
            sagitta_mm: 0.0,
            ..candidate
        };
        assert!(!fixed_tolerance.matches_mesh_aabb(lo, hi, 0.01));
    }

    /// 弯头两处离散（管截面 + 扫掠）都要算进缺口，否则整族 BEND 永远退回面片。
    #[test]
    fn a_rotated_elbow_clears_the_gate_by_both_of_its_chord_heights() {
        let (inner, outer, sweep_deg) = (95.0_f64, 105.0_f64, 90.0_f64);
        let solid = crate::solid::circ_torus_solid(inner, outer, sweep_deg).unwrap();
        let mesh = crate::mesh::TriMesh::from_manifold(&solid);
        let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();
        let torus = torus_from_local_mesh(&mesh.vertices, &triangles).unwrap();
        // 顶点数说得清这两个数：每圈截面 tube_segments 个点，共 sweep_steps + 1 圈。
        assert_eq!(
            torus.tube_segments * (torus.sweep_steps + 1),
            mesh.vertices.len(),
            "tube={} steps={}",
            torus.tube_segments,
            torus.sweep_steps
        );

        let world = glam::DMat4::from_rotation_z(0.4) * glam::DMat4::from_rotation_x(0.7);
        let (lo, hi) = world_aabb_of(&mesh.vertices, &world);

        let candidate = circular_torus_geometry(&world, &torus);
        assert!(candidate.matches_mesh_aabb(lo, hi, 0.01));
        let fixed_tolerance = AnalyticCandidate {
            sagitta_mm: 0.0,
            ..candidate
        };
        assert!(!fixed_tolerance.matches_mesh_aabb(lo, hi, 0.01));
    }

    #[test]
    fn strings_that_are_a_multiple_of_four_bytes_get_no_terminator() {
        let mut out = Vec::new();
        push_string(&mut out, "/ALL");
        assert_eq!(out, vec![0, 0, 0, 1, b'/', b'A', b'L', b'L']);

        let mut out = Vec::new();
        push_string(&mut out, "AMS");
        assert_eq!(out, vec![0, 0, 0, 1, b'A', b'M', b'S', 0]);
    }

    fn sorted(pair: [f64; 2]) -> [f64; 2] {
        let mut pair = pair.map(f64::abs);
        pair.sort_by(f64::total_cmp);
        pair
    }

    fn assert_pair_eq(got: [f64; 2], want: [f64; 2], what: &str) {
        let (got, want) = (sorted(got), sorted(want));
        assert!(
            (got[0] - want[0]).abs() < 1e-6 && (got[1] - want[1]).abs() < 1e-6,
            "{what}: got {got:?} want {want:?}"
        );
    }

    /// 拿 `pyramid_solid` 自己造几只棱台再认回来：完整的棱台、偏心的、退化成楔形与四棱锥的。
    #[test]
    fn recognises_pyramids_built_by_the_solid_kernel() {
        // `/电动消防冷却水泵P-3601` 的 PYRA（570×120 → 65×120）、`:SDS_Entity 3` 的支架底板
        // 加了偏心、一只楔形、一只四棱锥。
        for (xb, yb, xt, yt, h, xo, yo) in [
            (570.0, 120.0, 65.0, 120.0, 330.0, 0.0, 0.0),
            (230.0, 230.0, 200.0, 200.0, 10.0, 30.0, -20.0),
            (100.0, 80.0, 0.0, 80.0, 50.0, 0.0, 12.0),
            (100.0, 80.0, 0.0, 0.0, 50.0, -7.0, 0.0),
        ] {
            let solid = crate::solid::pyramid_solid(xb, yb, xt, yt, h, xo, yo).unwrap();
            let mesh = crate::mesh::TriMesh::from_manifold(&solid);
            let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();
            let pyramid = pyramid_from_local_mesh(&mesh.vertices, &triangles)
                .unwrap_or_else(|| panic!("棱台应认得回来: {xb}/{yb}/{xt}/{yt}/{h}/{xo}/{yo}"));
            assert!((pyramid.height - h).abs() < 1e-6, "h={}", pyramid.height);
            // 轴向由面法向定，可能与内核的 +Z 反向；反过来时上下面跟着换。
            // 底面哪条边算 X 也由识别器定，边长按集合比；偏心同理按绝对值的集合比。
            let (bottom, top) = if pyramid.axis.z >= 0.0 {
                (pyramid.bottom, pyramid.top)
            } else {
                (pyramid.top, pyramid.bottom)
            };
            assert_pair_eq(bottom, [xb, yb], "bottom");
            assert_pair_eq(top, [xt, yt], "top");
            assert_pair_eq(pyramid.offset, [xo, yo], "offset");

            // 写出来的记录必须过自校闸（带旋转）：偏心记错一半，AABB 立刻差 |off|/2。
            let world = glam::DMat4::from_rotation_z(0.4) * glam::DMat4::from_rotation_x(0.7);
            let (lo, hi) = world_aabb_of(&mesh.vertices, &world);
            let candidate = pyramid_geometry(&world, &pyramid);
            assert_eq!(candidate.geometry.primitive.kind_id(), 1);
            assert_eq!(candidate.sagitta_mm, 0.0);
            assert!(
                candidate.matches_mesh_aabb(lo, hi, 0.01),
                "{xb}/{yb}/{xt}/{yt}/{h}/{xo}/{yo}: {:?}",
                candidate.geometry.world_aabb_mm()
            );
        }
    }

    #[test]
    fn a_box_is_not_offered_as_a_pyramid_but_a_sheared_one_is() {
        // 上下同形、不偏心 = 盒子：`Box` 那条记录才贴 E3D，识别器让路。
        let solid =
            crate::solid::pyramid_solid(100.0, 100.0, 100.0, 100.0, 50.0, 0.0, 0.0).unwrap();
        let mesh = crate::mesh::TriMesh::from_manifold(&solid);
        let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();
        assert!(pyramid_from_local_mesh(&mesh.vertices, &triangles).is_none());

        // 同形但偏心的平行六面体不是盒子，得认，而且写出来要过闸。
        let solid =
            crate::solid::pyramid_solid(100.0, 100.0, 100.0, 100.0, 50.0, 20.0, 0.0).unwrap();
        let mesh = crate::mesh::TriMesh::from_manifold(&solid);
        let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();
        let pyramid =
            pyramid_from_local_mesh(&mesh.vertices, &triangles).expect("偏心的平行六面体");
        let world = glam::DMat4::from_rotation_z(0.4) * glam::DMat4::from_rotation_x(0.7);
        let (lo, hi) = world_aabb_of(&mesh.vertices, &world);
        assert!(pyramid_geometry(&world, &pyramid).matches_mesh_aabb(lo, hi, 0.01));
    }

    #[test]
    fn a_box_shaped_pyramid_is_kept_when_the_catalogue_says_it_came_from_an_lpyr() {
        // 300 只阀门的手轮座就是这一只：PAAX 与 PBAX 都解出 +X，成品是一只正方盒，
        // 基准 RVM 里它仍是 `Pyramid`。按长相让给 `Box`，按出身照认棱台。
        let solid =
            crate::solid::pyramid_solid(100.0, 100.0, 100.0, 100.0, 50.0, 0.0, 0.0).unwrap();
        let mesh = crate::mesh::TriMesh::from_manifold(&solid);
        let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();
        let pyramid = pyramid_from_local_mesh_including_boxes(&mesh.vertices, &triangles)
            .expect("出身是 LPYR 的棱台");
        // 盒子的三条棱都能当轴，认出来的是哪一条由面法向的先后决定；能钉的是「两端同形、
        // 不偏心，三条边长就是这只盒子的」——记录写出去仍是同一只盒，只是躺法不同。
        assert_pair_eq(pyramid.bottom, pyramid.top, "两端同形");
        assert_pair_eq(pyramid.offset, [0.0, 0.0], "offset");
        let mut edges = [pyramid.bottom[0], pyramid.bottom[1], pyramid.height];
        edges.sort_by(f64::total_cmp);
        assert!(
            (edges[0] - 50.0).abs() < 1e-6
                && (edges[1] - 100.0).abs() < 1e-6
                && (edges[2] - 100.0).abs() < 1e-6,
            "{edges:?}"
        );

        let world = glam::DMat4::from_rotation_z(0.4) * glam::DMat4::from_rotation_x(0.7);
        let (lo, hi) = world_aabb_of(&mesh.vertices, &world);
        let candidate = pyramid_geometry(&world, &pyramid);
        assert_eq!(candidate.geometry.primitive.kind_id(), 1);
        assert_eq!(candidate.sagitta_mm, 0.0);
        assert!(candidate.matches_mesh_aabb(lo, hi, 0.01));
    }

    #[test]
    fn a_frustum_with_a_trapezoid_side_is_recognised_along_its_true_axis() {
        // 570×120 → 65×120 的 ±Y 侧面是竖直矩形，法向 ±Y 也让顶点只落两档；但那两档
        // 各是一个梯形，过不了矩形那一关，识别器必须落到真正的底面法向 Z 上。
        let solid =
            crate::solid::pyramid_solid(570.0, 120.0, 65.0, 120.0, 330.0, 0.0, 0.0).unwrap();
        let mesh = crate::mesh::TriMesh::from_manifold(&solid);
        let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();
        let pyramid = pyramid_from_local_mesh(&mesh.vertices, &triangles).unwrap();
        assert!(pyramid.axis.z.abs() > 1.0 - 1e-9, "axis={:?}", pyramid.axis);
        assert!((pyramid.height - 330.0).abs() < 1e-6);
    }

    /// 拿 `unit_sphere_solid` 造几只球再认回来，缺口按网格自己的内切球算。
    #[test]
    fn recognises_spheres_built_by_the_solid_kernel() {
        for (segments, radius) in [(8, 5.0_f64), (12, 10.0), (24, 50.0)] {
            let solid = crate::solid::unit_sphere_solid(segments)
                .unwrap()
                .scale(radius, radius, radius)
                .translate(3.0, -4.0, 5.0);
            let mesh = crate::mesh::TriMesh::from_manifold(&solid);
            let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();
            let sphere = sphere_from_local_mesh(&mesh.vertices, &triangles)
                .unwrap_or_else(|| panic!("{segments} 段的球应认得回来"));
            assert!((sphere.radius - radius).abs() < 1e-9, "r={}", sphere.radius);
            assert!(
                (sphere.centre - glam::DVec3::new(3.0, -4.0, 5.0)).length() < 1e-9,
                "centre={:?}",
                sphere.centre
            );
            if segments == 8 {
                // 八面体每边一分为二：三个棱中点张成的那一片到球心 √(2/3)·r。
                let want = (1.0 - (2.0_f64 / 3.0).sqrt()) * radius;
                assert!(
                    (sphere.sagitta_mm - want).abs() < 1e-9,
                    "sagitta={}",
                    sphere.sagitta_mm
                );
            }
            assert!(sphere.sagitta_mm > 0.0 && sphere.sagitta_mm < 0.2 * radius);

            let world = glam::DMat4::from_rotation_z(0.4) * glam::DMat4::from_rotation_x(0.7);
            let (lo, hi) = world_aabb_of(&mesh.vertices, &world);
            let candidate = sphere_geometry(&world, &sphere);
            assert_eq!(candidate.geometry.primitive.kind_id(), 9);
            assert!(candidate.matches_mesh_aabb(lo, hi, 0.01), "{segments} 段");
            // 记录带着元素的旋转（与 E3D 同口径），世界盒仍是球心 ± r：闸门那一侧不能被
            // 八个角转出来的大盒糊住。
            let rotation = candidate.geometry.rotation();
            let expected = (world * glam::DMat4::from_translation(sphere.centre)).to_cols_array();
            for (got, want) in rotation.iter().zip([
                expected[0],
                expected[1],
                expected[2],
                expected[4],
                expected[5],
                expected[6],
                expected[8],
                expected[9],
                expected[10],
            ]) {
                assert!((got - want).abs() < 1e-12, "{rotation:?}");
            }
            let aabb = candidate.world_aabb_mm().unwrap();
            let centre = world.transform_point3(sphere.centre);
            for axis in 0..3 {
                assert!(
                    (aabb[axis] - (centre[axis] - radius)).abs() < 1e-9,
                    "{aabb:?}"
                );
                assert!(
                    (aabb[axis + 3] - (centre[axis] + radius)).abs() < 1e-9,
                    "{aabb:?}"
                );
            }
            // 读侧按球面采样算出来的世界盒与球心 ± r 只差采样误差（每侧 7.5e-5·r，
            // `rvm_read.rs` 的 Sphere 分支）；早先它是八个角、转了就大一圈，闸门这一格
            // 正是为绕开它才单独写的，两边现在要对得上。
            let by_hull = candidate.geometry.world_aabb_mm().unwrap();
            for axis in 0..3 {
                assert!(
                    (by_hull[axis] - aabb[axis]).abs() < 1e-3 * radius
                        && (by_hull[axis + 3] - aabb[axis + 3]).abs() < 1e-3 * radius,
                    "{by_hull:?} vs {aabb:?}"
                );
            }
        }
    }

    #[test]
    fn polyhedra_with_equidistant_corners_are_not_spheres() {
        // 立方体：八个角与中心等距，但只有 8 个顶点、内切比 0.577。
        let cube =
            crate::solid::pyramid_solid(100.0, 100.0, 100.0, 100.0, 100.0, 0.0, 0.0).unwrap();
        let mesh = crate::mesh::TriMesh::from_manifold(&cube);
        let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();
        assert!(sphere_from_local_mesh(&mesh.vertices, &triangles).is_none());

        // 直径 = 高的圆柱：全部顶点到中心都是 √2·r，48 个顶点够数，但内切比不到 1/√2。
        let cylinder = crate::solid::csg_cylinder_solid(100.0, 100.0).unwrap();
        let mesh = crate::mesh::TriMesh::from_manifold(&cylinder);
        let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();
        assert!(mesh.vertices.len() >= MIN_SPHERE_VERTICES);
        assert!(sphere_from_local_mesh(&mesh.vertices, &triangles).is_none());
    }

    /// 拿 `spherical_dish_solid` 造几只碟再认回来：浅碟、半球、很浅的碟。
    #[test]
    fn recognises_spherical_dishes_built_by_the_solid_kernel() {
        // `DISH 1 of EQUIPMENT 1 of ZONE 3 of /Copy-of-泵棚B版`：底圆半径 225、高 100。
        for (radius, height) in [(225.0_f64, 100.0_f64), (100.0, 100.0), (60.0, 15.0)] {
            let solid = crate::solid::spherical_dish_solid(radius, height).unwrap();
            let mesh = crate::mesh::TriMesh::from_manifold(&solid);
            let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();
            let dish = spherical_dish_from_local_mesh(&mesh.vertices, &triangles)
                .unwrap_or_else(|| panic!("碟应认得回来: {radius}/{height}"));
            assert!((dish.base_radius - radius).abs() < 1e-6, "{dish:?}");
            assert!((dish.height - height).abs() < 1e-6, "{dish:?}");
            // 内核把底圆放在 z = 0、顶点在 +z：轴必须指向 +Z，原点必须是底圆心。
            assert!(dish.axis.z > 1.0 - 1e-9, "{dish:?}");
            assert!(dish.centre.length() < 1e-6, "{dish:?}");
            assert!(
                dish.segments >= MIN_CIRCLE_SEGMENTS && dish.segments % 4 == 0,
                "{dish:?}"
            );
            assert!(
                dish.sagitta_mm > 0.0 && dish.sagitta_mm < 0.02 * radius.max(height),
                "{dish:?}"
            );

            let world = glam::DMat4::from_rotation_z(0.4) * glam::DMat4::from_rotation_x(0.7);
            let (lo, hi) = world_aabb_of(&mesh.vertices, &world);
            let candidate = spherical_dish_geometry(&world, &dish);
            assert_eq!(candidate.geometry.primitive.kind_id(), 6);
            assert!(
                candidate.matches_mesh_aabb(lo, hi, 0.01),
                "{radius}/{height}: sagitta={} {:?} vs {lo:?}..{hi:?}",
                candidate.sagitta_mm,
                candidate.geometry.world_aabb_mm()
            );
        }
    }

    #[test]
    fn a_knuckled_dish_and_a_sphere_are_not_offered_as_spherical_dishes() {
        // 带折边的碟：折边环上的顶点不在冠球上，第 3 条把它挡回去。
        let (radius, height) = (225.0, 100.0);
        let knuckle = crate::solid::dish_knuckle_radius(radius, height);
        let solid = crate::solid::elliptical_dish_solid(radius, height, knuckle).unwrap();
        let mesh = crate::mesh::TriMesh::from_manifold(&solid);
        let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();
        assert!(spherical_dish_from_local_mesh(&mesh.vertices, &triangles).is_none());

        // 整球没有平底。
        let solid = crate::solid::unit_sphere_solid(12)
            .unwrap()
            .scale(50.0, 50.0, 50.0);
        let mesh = crate::mesh::TriMesh::from_manifold(&solid);
        let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();
        assert!(spherical_dish_from_local_mesh(&mesh.vertices, &triangles).is_none());

        // 圆柱两端都是整圈平盘，不是碟。
        let solid = crate::solid::csg_cylinder_solid(100.0, 40.0).unwrap();
        let mesh = crate::mesh::TriMesh::from_manifold(&solid);
        let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();
        assert!(spherical_dish_from_local_mesh(&mesh.vertices, &triangles).is_none());
    }

    /// 拿 `elliptical_dish_solid` 造几只折边碟再认回来，落盘写成 `EllipticalDish`（5）。
    ///
    /// 折边半径按目录那条路来（`PRAD` 只是开关，k 由 r / h 算出），与识别侧重算的那一份
    /// 必须是同一个数——不然冠球半径就错了，每个顶点都判不在面上。
    #[test]
    fn recognises_knuckled_dishes_built_by_the_solid_kernel() {
        // `VALV 1 /Copy-of-7#管线-1/B1` 那一族：E3D 写 EllipticalDish，我们原先退回面片。
        for (radius, height) in [(225.0_f64, 100.0_f64), (158.0, 90.0), (60.0, 15.0)] {
            let knuckle = crate::solid::dish_knuckle_radius(radius, height);
            let solid = crate::solid::elliptical_dish_solid(radius, height, knuckle).unwrap();
            let mesh = crate::mesh::TriMesh::from_manifold(&solid);
            let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();
            let dish = elliptical_dish_from_local_mesh(&mesh.vertices, &triangles)
                .unwrap_or_else(|| panic!("折边碟应认得回来: {radius}/{height}"));
            assert!((dish.base_radius - radius).abs() < 1e-6, "{dish:?}");
            assert!((dish.height - height).abs() < 1e-6, "{dish:?}");
            assert!(dish.axis.z > 1.0 - 1e-9, "{dish:?}");
            assert!(dish.centre.length() < 1e-6, "{dish:?}");
            assert_eq!(
                elliptical_dish_geometry(&glam::DMat4::IDENTITY, &dish)
                    .geometry
                    .primitive
                    .kind_id(),
                5,
                "RVM 的椭圆碟是 5"
            );
        }
    }

    /// 折边碟过自校闸靠的只是面片的内接缺口（弦高一档），不靠「形状损失」的放行量。
    ///
    /// 2026-09-19 之前读侧把 RVM 的 `EllipticalDish` 当半椭球采（rvm-rs 的画法），闸门只好
    /// 放行半椭球与折边头之间几毫米的 Hausdorff 距离；现在读侧按 E3D `GM_EDish` 的
    /// 冠球 + 折边环还原（k 由 r / h 算出，`expdri/DRDISH` 只写 (r, h) 并不丢形状），
    /// 两边是同一个面，放行量回到弦高——r 与 h 写反那种约定错差两个数量级，照样拦下。
    #[test]
    fn a_knuckled_dish_clears_the_gate_by_its_facet_gap_alone() {
        let (radius, height) = (225.0_f64, 100.0_f64);
        let knuckle = crate::solid::dish_knuckle_radius(radius, height);
        let solid = crate::solid::elliptical_dish_solid(radius, height, knuckle).unwrap();
        let mesh = crate::mesh::TriMesh::from_manifold(&solid);
        let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();
        let dish = elliptical_dish_from_local_mesh(&mesh.vertices, &triangles).unwrap();

        // 放行量是弦高那一档（FACET_TOL_MM = 0.5 的内接面片，片中间最多 ~1 mm），
        // 不再是半椭球 vs 折边头那几毫米（这只 225/100 的碟两条母线径向最多差 7.8 mm）。
        assert!(
            (0.0..1.5).contains(&dish.sagitta_mm),
            "折边碟的缺口应是弦高量级，得到 {}",
            dish.sagitta_mm
        );

        let world = glam::DMat4::from_rotation_z(0.4) * glam::DMat4::from_rotation_x(0.7);
        let (lo, hi) = world_aabb_of(&mesh.vertices, &world);
        let candidate = elliptical_dish_geometry(&world, &dish);
        assert!(
            candidate.matches_mesh_aabb(lo, hi, 0.01),
            "sagitta={} {:?} vs {lo:?}..{hi:?}",
            candidate.sagitta_mm,
            candidate.geometry.world_aabb_mm()
        );

        // 闸门还咬得动：把 r 与 h 写反（同一类约定错）差 100 mm 以上，这点放行量兜不住。
        let swapped = RecognisedEllipticalDish {
            base_radius: dish.height,
            height: dish.base_radius,
            ..dish
        };
        assert!(
            !elliptical_dish_geometry(&world, &swapped).matches_mesh_aabb(lo, hi, 0.01),
            "r/h 写反必须被自校闸拦下"
        );
    }

    /// 球形碟不能被折边碟这一条顺手认走：两条都认得下来时写出的种类不一样，
    /// 而 E3D 是按折边半径是不是零来选的（`expdri/DRDISH` `0x10003fb8`）。
    #[test]
    fn a_plain_spherical_dish_is_not_offered_as_a_knuckled_one() {
        for (radius, height) in [(225.0_f64, 100.0_f64), (60.0, 15.0)] {
            let solid = crate::solid::spherical_dish_solid(radius, height).unwrap();
            let mesh = crate::mesh::TriMesh::from_manifold(&solid);
            let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();
            assert!(
                elliptical_dish_from_local_mesh(&mesh.vertices, &triangles).is_none(),
                "{radius}/{height} 是球形碟"
            );
        }
        // 半球：两种碟逐点相同，归球形碟那一条（函数文档里记着这笔认不出的账）。
        let solid = crate::solid::elliptical_dish_solid(100.0, 100.0, 100.0).unwrap();
        let mesh = crate::mesh::TriMesh::from_manifold(&solid);
        let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();
        assert!(elliptical_dish_from_local_mesh(&mesh.vertices, &triangles).is_none());
    }

    /// `/1WCC096VN`（24383/66600）：网格把 245 mm 当沿 −Z 的高，目录把 120 mm 当沿 +X 的高。
    /// 上下同形、不偏心，边长换到标架上之后参数是 E3D 的 `[245, 206, 245, 206, 0, 0, 120]`。
    #[test]
    fn pyramid_in_frame_swaps_height_onto_the_catalogue_axis() {
        let pyramid = RecognisedPyramid {
            bottom: [120.0, 206.0],
            top: [120.0, 206.0],
            height: 245.0,
            offset: [0.0, 0.0],
            centre: glam::DVec3::ZERO,
            axis: glam::DVec3::new(0.0, 0.0, -1.0),
            x: glam::DVec3::X,
        };
        // 目录 Z = +X（120 mm 那条边），目录 X = 网格轴（245 mm）。
        let frame = glam::DMat4::from_cols(
            glam::DVec3::new(0.0, 0.0, -1.0).extend(0.0),
            glam::DVec3::Y.extend(0.0),
            glam::DVec3::X.extend(0.0),
            glam::DVec4::W,
        );
        let got = pyramid_geometry_in_frame(&glam::DMat4::IDENTITY, &pyramid, &frame);
        match got.geometry.primitive {
            Primitive::Pyramid(params) => {
                for (got, want) in params.iter().zip([245.0, 206.0, 245.0, 206.0, 0.0, 0.0, 120.0]) {
                    assert!((got - want).abs() < 1e-6, "{params:?} vs {want}");
                }
            }
            other => panic!("不是 Pyramid：{other:?}"),
        }
        let z = [
            got.geometry.matrix3[6] * M_TO_MM,
            got.geometry.matrix3[7] * M_TO_MM,
            got.geometry.matrix3[8] * M_TO_MM,
        ];
        assert!((z[0] - 1.0).abs() < 1e-9 && z[1].abs() < 1e-9 && z[2].abs() < 1e-9, "{z:?}");
    }

    /// 目录轴和三条边都不共线时不改参数，仍按网格轴写。
    #[test]
    fn pyramid_in_frame_falls_back_when_the_catalogue_axis_misses_the_box() {
        let pyramid = RecognisedPyramid {
            bottom: [120.0, 206.0],
            top: [120.0, 206.0],
            height: 245.0,
            offset: [0.0, 0.0],
            centre: glam::DVec3::ZERO,
            axis: glam::DVec3::Z,
            x: glam::DVec3::X,
        };
        let frame = glam::DMat4::from_mat3(glam::DMat3::from_axis_angle(
            glam::DVec3::new(1.0, 0.3, 0.2).normalize(),
            0.4,
        ));
        let got = pyramid_geometry_in_frame(&glam::DMat4::IDENTITY, &pyramid, &frame);
        let plain = pyramid_geometry(&glam::DMat4::IDENTITY, &pyramid);
        match (got.geometry.primitive, plain.geometry.primitive) {
            (Primitive::Pyramid(a), Primitive::Pyramid(b)) => assert_eq!(a, b),
            other => panic!("{other:?}"),
        }
    }

    /// `/1WCC189VN`：E3D 的球 Z 躺在 XY，半径和球心不变，矩阵改用目录标架。
    #[test]
    fn sphere_in_frame_uses_the_catalogue_axes_and_keeps_the_centre() {
        let sphere = RecognisedSphere {
            radius: 12.5,
            centre: glam::DVec3::new(17700.0, -10780.0, 7900.0),
            sagitta_mm: 0.2,
        };
        let z = glam::DVec3::new(-0.961, -0.276, 0.0).normalize();
        let x = glam::DVec3::Z.cross(z).normalize();
        let y = z.cross(x);
        let frame = glam::DMat4::from_cols(
            x.extend(0.0),
            y.extend(0.0),
            z.extend(0.0),
            glam::DVec4::W,
        );
        let got = sphere_geometry_in_frame(&glam::DMat4::IDENTITY, &sphere, &frame);
        match got.geometry.primitive {
            Primitive::Sphere([r]) => assert!((r - 12.5).abs() < 1e-9),
            other => panic!("{other:?}"),
        }
        let t = got.geometry.translation_mm;
        assert!((t[0] - sphere.centre.x).abs() < 1e-6);
        assert!((t[1] - sphere.centre.y).abs() < 1e-6);
        assert!((t[2] - sphere.centre.z).abs() < 1e-6);
        let got_z = glam::DVec3::new(
            got.geometry.matrix3[6],
            got.geometry.matrix3[7],
            got.geometry.matrix3[8],
        ) * M_TO_MM;
        assert!((got_z - z).length() < 1e-9, "{got_z:?} vs {z:?}");
    }
}

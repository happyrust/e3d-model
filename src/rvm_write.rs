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
        match self.geometry.world_aabb_mm() {
            Some(a) => (0..3).all(|axis| {
                (a[axis] - mesh_lo[axis]).abs() <= slack
                    && (a[axis + 3] - mesh_hi[axis]).abs() <= slack
            }),
            None => false,
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
/// 方向不分正负，统一朝一侧，省一半候选。
fn face_normal_axes(points: &[glam::DVec3], triangles: &[[u32; 3]]) -> Vec<glam::DVec3> {
    let mut axes: Vec<glam::DVec3> = Vec::new();
    for tri in triangles {
        let [a, b, c] = tri.map(|i| points[i as usize]);
        let normal = (b - a).cross(c - a);
        if normal.length_squared() <= 0.0 {
            continue;
        }
        let normal = normal.normalize();
        let normal = if normal.z < 0.0
            || (normal.z == 0.0 && (normal.y < 0.0 || (normal.y == 0.0 && normal.x < 0.0)))
        {
            -normal
        } else {
            normal
        };
        if !axes
            .iter()
            .any(|existing| (*existing - normal).length() < 1e-9)
        {
            axes.push(normal);
        }
    }
    axes
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
/// 三种情况**故意**认不出来，退回面片：两端等径且不偏心的是圆柱，`Cylinder` 更贴 E3D；
/// 带剪切的端面不垂直于轴，第 1 条就过不去；锥顶退化成一点的圆锥只有一个顶点，
/// 撑不起「共圆」这句判断。
pub fn snout_from_local_mesh(
    vertices: &[[f64; 3]],
    triangles: &[[u32; 3]],
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
        let (Some(bottom), Some(top)) = (
            circular_cap(&points, axis, centroid, low),
            circular_cap(&points, axis, centroid, high),
        ) else {
            continue;
        };
        let ((bottom_centre, bottom_radius, segments), (top_centre, top_radius, top_segments)) =
            (bottom, top);
        if segments != top_segments {
            continue; // 两端段数不同，不是一次回转扫出来的。
        }
        let arm = top_centre - bottom_centre;
        let lateral = arm - axis * arm.dot(axis);
        let offset = lateral.length();
        if (bottom_radius - top_radius).abs() <= 1e-9 && offset <= 1e-9 {
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
    let arm_dir = |arm: DVec3| (arm - axis * arm.dot(axis)).normalize();
    let sweep_extent = |start: DVec3| {
        let side = axis.cross(start);
        points.iter().fold(0.0f64, |acc, p| {
            let w = *p - centre;
            let planar = w - axis * w.dot(axis);
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
    let side = axis.cross(start);
    let mut rings: Vec<f64> = Vec::new();
    for p in &points {
        let planar = (*p - centre) - axis * (*p - centre).dot(axis);
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

    #[test]
    fn a_cylinder_is_not_offered_as_a_snout() {
        // 两端等径又不偏心的是圆柱：`Cylinder` 那条记录更贴 E3D，圆台识别器必须让路。
        let solid = crate::solid::csg_cylinder_solid(100.0, 400.0).unwrap();
        let mesh = crate::mesh::TriMesh::from_manifold(&solid);
        let triangles: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();
        assert!(snout_from_local_mesh(&mesh.vertices, &triangles).is_none());
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
}

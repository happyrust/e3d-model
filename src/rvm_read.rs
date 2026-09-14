//! AVEVA RVM 二进制读取——**基准侧与生成侧共用的那一份读法**。
//!
//! 存在的理由是对称：`rvm_compare` 今天一边吃 `*.rvm.json` 快照（rvm-rs 解出来的）、
//! 一边吃 `elements.json`（`gen_ams` 写的），两条路径的帧与单位各算各的。把两边都换成
//! 这里的 [`RvmFile`]，「基准 AABB 怎么算」就只剩一处实现，加判据也只改一处。
//! 写出端（`rvm_write`）复用同一套结构，读写共享一份格式定义。
//!
//! 不用 rvm-rs 的三个理由：① 它在本 crate 是 `dev-dependencies`，`src/` 里用不了；
//! ② 它对每条记录一律 `bbox.transform(&transform)`，对下面说的 rotated 帧会多转一次
//! （`examples/rvm_facets_dump.rs` 为此绕了 `y_up_to_record_frame` 一整套补偿）；
//! ③ 写出端本来就要有这份格式知识。
//!
//! # 格式（`tools/rvm_record_probe.py` 对六份 E3D 3.1 导出物实测）
//!
//! 大端到底。chunk = 16 字节 tag（4 个字符各占一个 big-endian u32）+ u32 **绝对**下一块偏移 + 负载。
//! 字串 = u32 字数（4 字节一字）+ 该字数的字节、NUL 补齐。
//!
//! ## 单位是三套，不是一套
//!
//! | 字段 | 单位 |
//! |---|---|
//! | `CNTB` 平移 | **mm**，绝对世界坐标（祖先链全 0，逐级累加会乘上嵌套深度） |
//! | `PRIM` 矩阵 3×3 | `R × 0.001` |
//! | `PRIM` 矩阵平移 | **m** |
//! | 顶点 / 记录 bbox / 图元参数 | **mm** |
//!
//! 于是 `世界 mm = (matrix3 · v + translation) × 1000`。
//!
//! ## 记录 bbox 有两个帧，而且不按 kind 分
//!
//! 432 条实测：解析图元 196/196 存**局部系**盒；FacetGroup 236 条里 228 条存
//! `AABB(matrix3 · v × 1000)`（**乘过旋转、未加平移**），另外 8 条
//! （`WALL 1..4` / `STWALL 1..4 of CWALL /1RS-WF03-W-C-RR001`）又回到局部系。
//! **帧由 E3D 写出端的代码路径决定，不由 kind 决定**——所以 [`RvmGeometry::bbox_frame`]
//! 是逐记录自证出来的，不是查表来的。
//!
//! 写出端（`expdri.dll`）的 `DRFACG` 无条件算 `记录 bbox = 宿主返回的包围盒 − 平移`，
//! 一条路径、没有分支：帧的差异发生在宿主的几何构建器那一侧，RVM 这边没有可查的表。
//! 所以这里的自证分支是**永久**的，不是等结论期间的权宜。
//!
//! 本模块给出的 [`RvmGeometry::world_aabb_mm`] 不依赖这个判定：它一律从几何本身
//! （面片顶点，或图元参数还原的规范盒）实算。`bbox_frame` 只用于记账与回归——
//! 它一旦变成 [`BboxFrame::Unknown`]，就说明 E3D 换了口径或出现了没建模的图元。

use std::collections::BTreeMap;
use std::path::Path;

use anyhow::{Context, Result, bail};

/// RVM 的长度基准是米，本仓一律 mm。
pub const M_TO_MM: f64 = 1000.0;

/// 记录 bbox 落在哪个帧（逐记录自证，见模块文档）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BboxFrame {
    /// `AABB(v)`：图元自身局部系。解析图元与 `WALL`/`STWALL` 族走这一档。
    Local,
    /// `AABB(matrix3 · v × 1000)`：乘过旋转、未加平移。多数 FacetGroup 走这一档。
    Rotated,
    /// 这条记录没有体，判帧无意义——`Line`（中心线）就是这一档，六份样本里有 377 条。
    /// 它**不是**异常：中心线本来就不参与空间判定。
    NotApplicable,
    /// 有体、但两个候选都对不上，或这个 kind 还没有规范盒模型。
    /// **这一档才是异常**：不得再拿记录字段当几何依据。
    Unknown,
}

/// 图元种类与它的参数（mm）。`kind_id` 与 RVM 的编号一致。
#[derive(Debug, Clone, PartialEq)]
pub enum Primitive {
    Pyramid([f64; 7]),
    Box([f64; 3]),
    RectangularTorus([f64; 4]),
    CircularTorus([f64; 3]),
    EllipticalDish([f64; 2]),
    SphericalDish([f64; 2]),
    /// `[rb, rt, height, offset_x, offset_y, bshear_x, bshear_y, tshear_x, tshear_y]`
    Snout([f64; 9]),
    Cylinder([f64; 2]),
    Sphere([f64; 1]),
    Line([f64; 2]),
    /// 多边形 → 轮廓 → 顶点（局部 mm）+ 逐顶点法线。
    FacetGroup(Vec<Polygon>),
    /// 读得出 kind_id 但本模块不认识，参数原样留着。
    Unknown {
        kind_id: u32,
        params: Vec<f64>,
    },
}

impl Primitive {
    pub fn kind_id(&self) -> u32 {
        match self {
            Self::Pyramid(_) => 1,
            Self::Box(_) => 2,
            Self::RectangularTorus(_) => 3,
            Self::CircularTorus(_) => 4,
            Self::EllipticalDish(_) => 5,
            Self::SphericalDish(_) => 6,
            Self::Snout(_) => 7,
            Self::Cylinder(_) => 8,
            Self::Sphere(_) => 9,
            Self::Line(_) => 10,
            Self::FacetGroup(_) => 11,
            Self::Unknown { kind_id, .. } => *kind_id,
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Self::Pyramid(_) => "Pyramid",
            Self::Box(_) => "Box",
            Self::RectangularTorus(_) => "RectangularTorus",
            Self::CircularTorus(_) => "CircularTorus",
            Self::EllipticalDish(_) => "EllipticalDish",
            Self::SphericalDish(_) => "SphericalDish",
            Self::Snout(_) => "Snout",
            Self::Cylinder(_) => "Cylinder",
            Self::Sphere(_) => "Sphere",
            Self::Line(_) => "Line",
            Self::FacetGroup(_) => "FacetGroup",
            Self::Unknown { .. } => "Unknown",
        }
    }

    /// 包住这个图元的**采样点云**（局部 mm）。没有体的（`Line`）与不认识的返回空。
    ///
    /// 为什么不是「规范盒的八个角」：盒角只在**盒本身**上是紧的。图元一旦带旋转，
    /// 八个角转出去的 AABB 会比真实几何大一圈——`1RCV0101` 的 `CircularTorus` 弯头
    /// 实测因此虚高 **56.7 mm**，把本来 `chord` 的件判成 `off`。曲面必须按面采样。
    ///
    /// **偏心 `Snout` 的记录 bbox 与这里给的真几何对不上是预期**：E3D 那只盒按另一条
    /// 公式写，见 [`declared_box_mm`](Self::declared_box_mm)，判帧由它兜住。
    pub fn local_hull_points_mm(&self) -> Vec<[f64; 3]> {
        // 圆周采样密度。采样点撑不到真实极值，AABB 会**偏小** r·(1−cos(π/n))，
        // 这个误差必须压到判档最细的一档 `EXACT_MM = 0.05 mm` 以下，否则本该 `exact`
        // 的件会被自己的采样误差顶成 `chord`。管道半径按 100 mm 算，n = 64 只到 0.12 mm，
        // 不够；n = 256 是 0.0075 mm，留了一个数量级的余量。这点采样量在这里不值一提。
        const STEPS: usize = 256;
        let ring = |radius: f64, z: f64| {
            (0..STEPS).map(move |i| {
                let th = std::f64::consts::TAU * i as f64 / STEPS as f64;
                [radius * th.cos(), radius * th.sin(), z]
            })
        };
        match self {
            Self::Box([x, y, z]) => {
                corners(&[-x / 2.0, -y / 2.0, -z / 2.0, x / 2.0, y / 2.0, z / 2.0]).to_vec()
            }
            Self::Cylinder([r, h]) => ring(*r, -h / 2.0).chain(ring(*r, h / 2.0)).collect(),
            Self::Sphere([r]) => {
                // 球的 AABB 与朝向无关，八个角就够紧。
                corners(&[-r, -r, -r, *r, *r, *r]).to_vec()
            }
            Self::Snout([rb, rt, h, ox, oy, ..]) => {
                // 偏心量两端各摊一半。
                ring(*rb, -h / 2.0)
                    .map(|p| [p[0] - ox / 2.0, p[1] - oy / 2.0, p[2]])
                    .chain(ring(*rt, h / 2.0).map(|p| [p[0] + ox / 2.0, p[1] + oy / 2.0, p[2]]))
                    .collect()
            }
            Self::EllipticalDish([r, h]) => {
                // 半椭球面（`(r·sinθ, h·cosθ)`，θ 从顶点扫到底圈）按纬度逐圈采到底圆。
                //
                // 与 [`Self::SphericalDish`] 同一个坑、同一条改法：碟一倾斜，AABB 的极值
                // 落在碟面中间某一圈上，光有底圈 + 顶点 + `(r/√2, h/√2)` 那一圈卡不住。
                // r 225 / h 100 的碟转 `rot_z(0.4)·rot_x(0.7)` 实测因此**偏小 11.7 mm**
                // （z 上界 156.6 对真值 168.3），生成侧的解析记录过不了自校闸。
                //
                // 这只是**采样**密度，不是形状口径：RVM 的 `EllipticalDish` 就是一只
                // z 向缩放的四分之一球（rvm-rs `impl Tessellate for EllipticalDish`），
                // E3D 自己的 `GM_EDish` 是冠球 + 折边环的 torispherical 头，两者差着
                // 一层导出损失——`expdri/DRDISH`（`0x10003fb8`）只写 (r, h)，折边半径
                // 只用来在 5（椭圆碟）与 6（球形碟）之间选一个，形状本身不落盘。
                const LATITUDES: usize = 64;
                if !(*r > 0.0 && *h > 0.0) {
                    return ring(*r, 0.0).chain(std::iter::once([0.0, 0.0, *h])).collect();
                }
                (1..LATITUDES)
                    .flat_map(move |j| {
                        let theta = std::f64::consts::FRAC_PI_2 * j as f64 / LATITUDES as f64;
                        ring(r * theta.sin(), h * theta.cos())
                    })
                    .chain(ring(*r, 0.0))
                    .chain(std::iter::once([0.0, 0.0, *h]))
                    .collect()
            }
            Self::SphericalDish([r, h]) => {
                // 球冠按纬度逐圈采到底圆：`R = (r² + h²) / 2h`，球心 (0, 0, h − R)。
                //
                // 碟一旦倾斜，AABB 的极值落在冠面中间某一圈上，光有底圈 + 顶点卡不住；
                // 而拿 (r/√2, h/√2) 那一圈顶替只对半球成立——浅碟的冠面在那个半径上要低得多，
                // r 225 / h 100 的 DISH 转 40° 时盒子因此虚高 8.9 mm，生成侧的解析记录
                // 过不了自校闸。纬向 64 圈：半球一档的采样误差 7.5e-5·R，与周向 256 段同量级
                //（`r·(1 − cos(π/256))` 也是 7.5e-5·r），不另开一档精度。
                const LATITUDES: usize = 64;
                if !(*r > 0.0 && *h > 0.0) {
                    return ring(*r, 0.0).chain(std::iter::once([0.0, 0.0, *h])).collect();
                }
                let sphere_radius = (r * r + h * h) / (2.0 * h);
                let phi_max = ((sphere_radius - h) / sphere_radius)
                    .clamp(-1.0, 1.0)
                    .acos();
                let centre_z = h - sphere_radius;
                (1..LATITUDES)
                    .flat_map(move |j| {
                        let phi = phi_max * j as f64 / LATITUDES as f64;
                        ring(sphere_radius * phi.sin(), centre_z + sphere_radius * phi.cos())
                    })
                    .chain(ring(*r, 0.0))
                    .chain(std::iter::once([0.0, 0.0, *h]))
                    .collect()
            }
            Self::Pyramid([bx, by, tx, ty, ox, oy, h]) => {
                // XOFF/YOFF 与 `Snout` 同一约定：两端各摊一半，底面心 (−ox/2, −oy/2)、
                // 顶面心 (+ox/2, +oy/2)。出处是 libgm.dll `GM_Pyramid::calcFacets`
                // （0x10094b10）：底面四点取 ±xbot/2 − xoff/2、±ybot/2 − yoff/2，
                // 顶面四点取 ±xtop/2 + xoff/2、±ytop/2 + yoff/2。
                //
                // 把偏心整个记到顶面会让盒子沿偏心方向整体涨 |off|/2：`/TEMP_EQUIP`
                // PIPE 2 的弯头（YOFF 50）因此虚高 25 mm，基准侧被判成 off。
                let quad = |sx: f64, sy: f64, cx: f64, cy: f64, z: f64| {
                    [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]]
                        .map(|[u, v]| [cx + u * sx / 2.0, cy + v * sy / 2.0, z])
                };
                quad(*bx, *by, -ox / 2.0, -oy / 2.0, -h / 2.0)
                    .into_iter()
                    .chain(quad(*tx, *ty, ox / 2.0, oy / 2.0, h / 2.0))
                    .collect()
            }
            Self::CircularTorus([offset, radius, angle]) => {
                swept_points(*angle, |th| {
                    // 管截面落在**径向/Z 平面**：`ring` 给的 (x, y) 要当成 (径向偏移, z)，
                    // 拿它的 y 当 z——写成 `[u, _, w]` 会把 z 取成恒 0，整条弯头压成一张片。
                    ring(*radius, 0.0)
                        .map(|[u, v, _]| {
                            let r = offset + u;
                            [r * th.cos(), r * th.sin(), v]
                        })
                        .collect()
                })
                .into_iter()
                .flatten()
                .collect()
            }
            Self::RectangularTorus([inner, outer, h, angle]) => swept_points(*angle, |th| {
                let mut out = Vec::with_capacity(4);
                for r in [*inner, *outer] {
                    for z in [-h / 2.0, h / 2.0] {
                        out.push([r * th.cos(), r * th.sin(), z]);
                    }
                }
                out
            })
            .into_iter()
            .flatten()
            .collect(),
            Self::Line(_) | Self::FacetGroup(_) | Self::Unknown { .. } => Vec::new(),
        }
    }

    /// 规范局部盒（mm）。认不出形状或没有体积（`Line`）时 None。
    pub fn local_aabb_mm(&self) -> Option<[f64; 6]> {
        aabb_of(self.local_hull_points_mm().into_iter())
    }

    /// E3D **自己往记录里写 bbox 时用的那只盒**（局部 mm），只在它与真几何不同的形状上给。
    ///
    /// 只服务 [`detect_bbox_frame`]：判帧问的是「记录里那 6 个数落在哪个帧」，
    /// 要拿 E3D 的口径去对；空间判定一律走 [`local_hull_points_mm`](Self::local_hull_points_mm)
    /// 的真几何，**不许**改用这里的盒——偏心 `Snout` 这一档它比真实体还小。
    ///
    /// 目前只有偏心 `Snout`（`XOFF`/`YOFF` 非零）。E3D 写的是一只**对称**盒，
    /// 半跨取两端「带符号盖心 + 该端半径」的较大者：
    ///
    /// ```text
    /// hx = max(rb − ox/2, rt + ox/2)      hy = max(rb − oy/2, rt + oy/2)
    /// 盒 = [−hx, −hy, −h/2] .. [hx, hy, h/2]
    /// ```
    ///
    /// 也就是说它只算了 `+` 那一侧、再镜像过去，`−` 侧的盖心偏出去的部分被吃掉了。
    /// 一只 `rb 136.5 / rt 109.55 / h 178 / ox 26.95` 的同心异径管接头（273.1×219.1 偏心大小头）
    /// 真几何是 `x ∈ [−149.975, 123.025]`，E3D 写的是 `x ∈ ±123.025`——**比实体还窄 26.95 mm**，
    /// 差值恰好等于 `XOFF`。所以它不是「放宽过的估计」，是一条算漏了的公式，凑不出、也不该凑。
    ///
    /// 证据是全量实测：`output/slyk-rvm/sweep` 的 80 份 E3D 3.1 导出物、5565 条 `Snout` 记录
    /// （其中 51 条偏心），上面这条公式**逐条**对得上（残差 ≤ 0.01 mm）；真几何那条只对上
    /// 非偏心的 5514 条，51 条偏心的残差恰好是 `|XOFF|`，大到顶穿容差时整份文件的判帧就塌了
    /// （`/1#罐组管线` 等 10 个 ZONE 的 REDUCER 因此被判 `Unknown`，严尺子对拍直接拒判）。
    pub fn declared_box_mm(&self) -> Option<[f64; 6]> {
        match self {
            Self::Snout([rb, rt, h, ox, oy, ..]) if *ox != 0.0 || *oy != 0.0 => {
                let hx = (rb - ox / 2.0).max(rt + ox / 2.0);
                let hy = (rb - oy / 2.0).max(rt + oy / 2.0);
                Some([-hx, -hy, -h / 2.0, hx, hy, h / 2.0])
            }
            _ => None,
        }
    }
}

/// 绕 Z 扫掠 `angle` 弧度，每一步给一圈截面点。
fn swept_points(angle: f64, section: impl Fn(f64) -> Vec<[f64; 3]>) -> Vec<Vec<[f64; 3]>> {
    // 扫掠角按 1° 一步采，够密；端面两步也保证被取到。
    let steps = (angle.abs().to_degrees().ceil() as usize).clamp(1, 720);
    (0..=steps)
        .map(|i| section(angle * i as f64 / steps as f64))
        .collect()
}

#[derive(Debug, Clone, PartialEq)]
pub struct Polygon {
    pub contours: Vec<Contour>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Contour {
    /// 局部 mm。
    pub vertices: Vec<[f64; 3]>,
    pub normals: Vec<[f64; 3]>,
}

/// 一条 `PRIM` / `OBST` / `INSU` 记录。
#[derive(Debug, Clone)]
pub struct RvmGeometry {
    /// `PRIM` / `OBST` / `INSU`。
    pub tag: String,
    pub flags: u32,
    pub primitive: Primitive,
    /// 列主序 3×3，**含 0.001 缩放**（原样，写回去不用换算）。
    pub matrix3: [f64; 9],
    /// 世界平移，**已换成 mm**（文件里存的是米）。
    pub translation_mm: [f64; 3],
    /// 记录里原样存的 6 个 f32（mm），帧见 [`bbox_frame`](Self::bbox_frame)。
    pub record_bbox: [f64; 6],
    pub bbox_frame: BboxFrame,
    /// `OBST` / `INSU` 在 bbox 之后、参数之前多带的那个等级字；`PRIM` 为 `None`。
    pub level: Option<u32>,
}

impl RvmGeometry {
    /// 3×3 的纯旋转部分（把 0.001 除掉）。
    pub fn rotation(&self) -> [f64; 9] {
        self.matrix3.map(|v| v * M_TO_MM)
    }

    /// 局部 mm → 世界 mm。
    pub fn to_world_mm(&self, v: [f64; 3]) -> [f64; 3] {
        let m = &self.matrix3;
        [
            (m[0] * v[0] + m[3] * v[1] + m[6] * v[2]) * M_TO_MM + self.translation_mm[0],
            (m[1] * v[0] + m[4] * v[1] + m[7] * v[2]) * M_TO_MM + self.translation_mm[1],
            (m[2] * v[0] + m[5] * v[1] + m[8] * v[2]) * M_TO_MM + self.translation_mm[2],
        ]
    }

    /// 面片顶点（局部 mm）。非 FacetGroup 为空。
    pub fn local_vertices_mm(&self) -> Vec<[f64; 3]> {
        match &self.primitive {
            Primitive::FacetGroup(polygons) => polygons
                .iter()
                .flat_map(|p| p.contours.iter())
                .flat_map(|c| c.vertices.iter().copied())
                .collect(),
            _ => Vec::new(),
        }
    }

    /// 够用来算 AABB 的**世界 mm** 点集：FacetGroup 给真顶点，解析图元给
    /// [`Primitive::local_hull_points_mm`] 的采样点。
    ///
    /// 判局部帧的一侧需要「这条几何在世界里占了哪些点」。只有 FacetGroup 才有真顶点，
    /// 解析图元若因此退回世界帧，同一份 RVM 里 Box 成员与 FacetGroup 成员就走了两套判据。
    /// `Line` 与不认识的 kind 仍然给空（没有体）。
    pub fn world_hull_points_mm(&self) -> Vec<[f64; 3]> {
        let local = match &self.primitive {
            Primitive::FacetGroup(_) => self.local_vertices_mm(),
            other => other.local_hull_points_mm(),
        };
        local.into_iter().map(|v| self.to_world_mm(v)).collect()
    }

    /// 世界 AABB（mm），**从几何本身实算**，不碰 `record_bbox`。
    ///
    /// `Line` 与不认识的 kind 返回 None（没有体，不参与空间判定）。
    pub fn world_aabb_mm(&self) -> Option<[f64; 6]> {
        aabb_of(self.world_hull_points_mm().into_iter())
    }
}

/// 一个 `CNTB` 组。
#[derive(Debug, Clone)]
pub struct RvmMember {
    /// 组名。命名元素是 E3D NAME（`/C-OR-1R345-C`）；无名元素是
    /// `<NOUN> <n> of <OWNER 显示名>`（`FTUBE 1 of BRANCH /C-OR-1R345-C`），NOUN 用全称。
    pub name: String,
    /// 从根到本组的完整路径。
    pub path: String,
    /// 本组在 `members` 里的父下标。
    pub parent: Option<usize>,
    /// `CNTB` 的平移（**mm**，绝对世界坐标，**不要逐级累加**）。
    pub translation_mm: [f64; 3],
    pub material: u32,
    pub geometries: Vec<RvmGeometry>,
}

impl RvmMember {
    /// 本组全部几何的世界 AABB 并集（mm）。
    pub fn world_aabb_mm(&self) -> Option<[f64; 6]> {
        self.geometries
            .iter()
            .filter_map(|g| g.world_aabb_mm())
            .reduce(merge)
    }

    /// 本组全部面片顶点，世界 mm。局部系判据由调用方乘 `world⁻¹` 得到。
    pub fn world_vertices_mm(&self) -> Vec<[f64; 3]> {
        self.geometries
            .iter()
            .flat_map(|g| {
                g.local_vertices_mm()
                    .into_iter()
                    .map(move |v| g.to_world_mm(v))
            })
            .collect()
    }

    /// 本组全部几何的世界 mm 点集，解析图元用规范盒的八个角顶上
    /// （见 [`RvmGeometry::world_hull_points_mm`]）。
    pub fn world_hull_points_mm(&self) -> Vec<[f64; 3]> {
        self.geometries
            .iter()
            .flat_map(|g| g.world_hull_points_mm())
            .collect()
    }
}

#[derive(Debug, Clone, Default)]
pub struct RvmHeader {
    pub banner: String,
    pub note: String,
    pub date: String,
    pub user: String,
    pub encoding: String,
    pub model: String,
    pub project: String,
}

#[derive(Debug, Clone)]
pub struct RvmFile {
    pub header: RvmHeader,
    /// 前序（父一定排在子之前），与文件里的 `CNTB` 顺序一致。
    pub members: Vec<RvmMember>,
}

impl RvmFile {
    pub fn read(path: impl AsRef<Path>) -> Result<Self> {
        let path = path.as_ref();
        let bytes =
            std::fs::read(path).with_context(|| format!("读 RVM 失败: {}", path.display()))?;
        Self::parse(&bytes).with_context(|| format!("解析 RVM 失败: {}", path.display()))
    }

    /// 按 `bbox_frame` 分桶计数，用于回归：`Unknown` 必须是 0。
    pub fn bbox_frame_tally(&self) -> BTreeMap<&'static str, usize> {
        let mut out = BTreeMap::new();
        for geometry in self.members.iter().flat_map(|m| m.geometries.iter()) {
            let key = match geometry.bbox_frame {
                BboxFrame::Local => "local",
                BboxFrame::Rotated => "rotated",
                BboxFrame::NotApplicable => "n/a",
                BboxFrame::Unknown => "unknown",
            };
            *out.entry(key).or_insert(0) += 1;
        }
        out
    }

    pub fn parse(bytes: &[u8]) -> Result<Self> {
        let mut header = RvmHeader::default();
        let mut members: Vec<RvmMember> = Vec::new();
        let mut stack: Vec<usize> = Vec::new();
        let mut offset = 0usize;

        while offset + 20 <= bytes.len() {
            if bytes[offset..offset + 16].iter().all(|b| *b == 0) {
                break;
            }
            let tag = read_tag(bytes, offset);
            let next = read_u32(bytes, offset + 16)? as usize;
            if next <= offset + 19 || next > bytes.len() {
                bail!("{tag:?} @ {offset:#x} 的下一块偏移 {next:#x} 越界");
            }
            let data = &bytes[offset + 20..next];

            match tag.as_str() {
                "HEAD" => {
                    let mut p = 8;
                    for slot in [
                        &mut header.banner,
                        &mut header.note,
                        &mut header.date,
                        &mut header.user,
                        &mut header.encoding,
                    ] {
                        if p + 4 > data.len() {
                            break;
                        }
                        let (s, q) = read_string(data, p)?;
                        *slot = s;
                        p = q;
                    }
                }
                "MODL" => {
                    let (model, p) = read_string(data, 8)?;
                    header.model = model;
                    if p + 4 <= data.len() {
                        header.project = read_string(data, p)?.0;
                    }
                }
                "CNTB" => {
                    let (name, p) = read_string(data, 8)?;
                    let parent = stack.last().copied();
                    // 命名元素的组名自带前导 `/`（`/C-OR-1R345-C`），拼接时别再加一个。
                    let path = match parent {
                        Some(i) => format!("{}/{}", members[i].path, name.trim_start_matches('/')),
                        None => format!("/{}", name.trim_start_matches('/')),
                    };
                    stack.push(members.len());
                    members.push(RvmMember {
                        name,
                        path,
                        parent,
                        translation_mm: [
                            read_f32(data, p)? as f64,
                            read_f32(data, p + 4)? as f64,
                            read_f32(data, p + 8)? as f64,
                        ],
                        material: read_u32(data, p + 12)?,
                        geometries: Vec::new(),
                    });
                }
                "CNTE" => {
                    stack.pop();
                }
                "PRIM" | "OBST" | "INSU" => {
                    let geometry = parse_geometry(&tag, data)?;
                    if let Some(&index) = stack.last() {
                        members[index].geometries.push(geometry);
                    }
                }
                // COLR 是收尾的一条全局色，本仓的判据不看颜色；END/未知块跳过。
                _ => {}
            }
            offset = next;
        }

        Ok(Self { header, members })
    }
}

/// bbox 之后第一个参数字节的偏移。
const PARAMS_OFFSET: usize = 84;
/// `OBST` / `INSU` 在参数前多带的等级字。
const LEVEL_WORD: usize = 4;
/// `flags == 2` 时记录尾巴上那 3 个 f64 的真实平移。
const FAR_FIELD_TAIL: usize = 24;

fn parse_geometry(tag: &str, data: &[u8]) -> Result<RvmGeometry> {
    if data.len() < PARAMS_OFFSET {
        bail!("{tag} 负载只有 {} 字节，不足 {PARAMS_OFFSET}", data.len());
    }
    let flags = read_u32(data, 4)?;
    let kind_id = read_u32(data, 8)?;

    let mut matrix3 = [0f64; 9];
    for (i, slot) in matrix3.iter_mut().enumerate() {
        *slot = read_f32(data, 12 + i * 4)? as f64;
    }
    let translation_mm = [
        read_f32(data, 48)? as f64 * M_TO_MM,
        read_f32(data, 52)? as f64 * M_TO_MM,
        read_f32(data, 56)? as f64 * M_TO_MM,
    ];
    let mut record_bbox = [0f64; 6];
    for (i, slot) in record_bbox.iter_mut().enumerate() {
        *slot = read_f32(data, 60 + i * 4)? as f64;
    }

    // `PRIM` 的参数紧跟在 bbox 后面；`OBST` / `INSU` 在这里多一个等级字，参数顺延 4 字节。
    // 八份基准样本一条 OBST/INSU 都没有，这条偏移验不了，所以不按 tag 硬编码：两个候选
    // 都试，取「正好吃到记录末尾」的那个。823 条实测全部零余量，这个自证是干净的。
    let candidates: &[usize] = if tag == "PRIM" {
        &[PARAMS_OFFSET]
    } else {
        &[PARAMS_OFFSET + LEVEL_WORD, PARAMS_OFFSET]
    };
    let mut exact = None;
    let mut first = None;
    for &start in candidates {
        let Some(tail) = data.get(start..) else {
            continue;
        };
        let Ok((primitive, used)) = parse_primitive(kind_id, tail) else {
            continue;
        };
        // flags=2 是写出端的远场逃逸档：平移超出 ±1e9 mm 时真值以 3 个 f64 附在记录尾巴上。
        if start + used == data.len() || (flags == 2 && start + used + FAR_FIELD_TAIL == data.len())
        {
            exact = Some((start, primitive));
            break;
        }
        first.get_or_insert((start, primitive));
    }
    let (params_at, primitive) = exact
        .or(first)
        .with_context(|| format!("{tag} 记录的参数段解析不出来（kind {kind_id}）"))?;

    let level = if params_at > PARAMS_OFFSET {
        Some(read_u32(data, PARAMS_OFFSET)?)
    } else {
        None
    };

    let mut geometry = RvmGeometry {
        tag: tag.to_string(),
        flags,
        primitive,
        matrix3,
        translation_mm,
        record_bbox,
        bbox_frame: BboxFrame::Unknown,
        level,
    };
    geometry.bbox_frame = detect_bbox_frame(&geometry);
    Ok(geometry)
}

/// 返回图元与它吃掉的字节数——调用侧拿这个数与记录长度对齐，自证参数段起点。
fn parse_primitive(kind_id: u32, tail: &[u8]) -> Result<(Primitive, usize)> {
    fn params<const N: usize>(tail: &[u8]) -> Result<([f64; N], usize)> {
        let mut out = [0f64; N];
        for (i, slot) in out.iter_mut().enumerate() {
            *slot = read_f32(tail, i * 4)? as f64;
        }
        Ok((out, N * 4))
    }
    Ok(match kind_id {
        1 => params(tail).map(|(p, n)| (Primitive::Pyramid(p), n))?,
        2 => params(tail).map(|(p, n)| (Primitive::Box(p), n))?,
        3 => params(tail).map(|(p, n)| (Primitive::RectangularTorus(p), n))?,
        4 => params(tail).map(|(p, n)| (Primitive::CircularTorus(p), n))?,
        5 => params(tail).map(|(p, n)| (Primitive::EllipticalDish(p), n))?,
        6 => params(tail).map(|(p, n)| (Primitive::SphericalDish(p), n))?,
        7 => params(tail).map(|(p, n)| (Primitive::Snout(p), n))?,
        8 => params(tail).map(|(p, n)| (Primitive::Cylinder(p), n))?,
        9 => params(tail).map(|(p, n)| (Primitive::Sphere(p), n))?,
        10 => params(tail).map(|(p, n)| (Primitive::Line(p), n))?,
        11 => parse_facet_group(tail).map(|(g, n)| (Primitive::FacetGroup(g), n))?,
        other => (
            Primitive::Unknown {
                kind_id: other,
                params: Vec::new(),
            },
            tail.len(),
        ),
    })
}

fn parse_facet_group(data: &[u8]) -> Result<(Vec<Polygon>, usize)> {
    let mut pos = 0usize;
    let polygon_count = read_u32(data, pos)? as usize;
    pos += 4;
    let mut polygons = Vec::with_capacity(polygon_count);
    for _ in 0..polygon_count {
        let contour_count = read_u32(data, pos)? as usize;
        pos += 4;
        let mut contours = Vec::with_capacity(contour_count);
        for _ in 0..contour_count {
            let vertex_count = read_u32(data, pos)? as usize;
            pos += 4;
            let mut vertices = Vec::with_capacity(vertex_count);
            let mut normals = Vec::with_capacity(vertex_count);
            for _ in 0..vertex_count {
                vertices.push([
                    read_f32(data, pos)? as f64,
                    read_f32(data, pos + 4)? as f64,
                    read_f32(data, pos + 8)? as f64,
                ]);
                normals.push([
                    read_f32(data, pos + 12)? as f64,
                    read_f32(data, pos + 16)? as f64,
                    read_f32(data, pos + 20)? as f64,
                ]);
                pos += 24;
            }
            contours.push(Contour { vertices, normals });
        }
        polygons.push(Polygon { contours });
    }
    Ok((polygons, pos))
}

/// 逐记录自证 bbox 的帧（见模块文档）。
fn detect_bbox_frame(geometry: &RvmGeometry) -> BboxFrame {
    // 中心线没有体，判帧无意义；不认识的 kind 才是真的说不出来。
    if matches!(geometry.primitive, Primitive::Line(_)) {
        return BboxFrame::NotApplicable;
    }
    let points = match &geometry.primitive {
        Primitive::FacetGroup(_) => geometry.local_vertices_mm(),
        other => other.local_hull_points_mm(),
    };
    if points.is_empty() {
        return BboxFrame::Unknown;
    }
    let m = &geometry.matrix3;
    let rotate = |v: &[f64; 3]| {
        [
            (m[0] * v[0] + m[3] * v[1] + m[6] * v[2]) * M_TO_MM,
            (m[1] * v[0] + m[4] * v[1] + m[7] * v[2]) * M_TO_MM,
            (m[2] * v[0] + m[5] * v[1] + m[8] * v[2]) * M_TO_MM,
        ]
    };

    let record = &geometry.record_bbox;
    let residual = |a: &[f64; 6]| {
        a.iter()
            .zip(record.iter())
            .fold(0.0f64, |acc, (x, y)| acc.max((x - y).abs()))
    };
    // 两套点云各判一次，取更贴的那一档：真几何，以及 E3D 写 bbox 时另算的那只盒
    // （见 `Primitive::declared_box_mm`，只有偏心 `Snout` 有）。E3D 哪天把那条公式
    // 补齐了，真几何这一套自己就会重新贴上，这里不用回来改。
    let mut best: Option<(f64, BboxFrame)> = None;
    let hulls = std::iter::once(points).chain(
        geometry
            .primitive
            .declared_box_mm()
            .map(|box_mm| corners(&box_mm).to_vec()),
    );
    for hull in hulls {
        for (aabb, frame) in [
            (aabb_of(hull.iter().copied()), BboxFrame::Local),
            (aabb_of(hull.iter().map(rotate)), BboxFrame::Rotated),
        ] {
            let Some(aabb) = aabb else { continue };
            let d = residual(&aabb);
            // 同样贴时取 Local：轴对齐的记录两个帧本就相等，别让它在两档之间来回跳。
            if best.is_none_or(|(best_d, _)| d < best_d) {
                best = Some((d, frame));
            }
        }
    }
    let Some((d_best, frame)) = best else {
        return BboxFrame::Unknown;
    };
    // 1mm + 该轴 8% 跨度：圆周按 1mm 弦高最少 8 段，极值角上没有顶点时最多缩进 7.6%。
    let span = (0..3).fold(0.0f64, |acc, a| acc.max((record[a + 3] - record[a]).abs()));
    let tolerance = 1.0 + 0.08 * span;
    if d_best <= tolerance {
        frame
    } else {
        BboxFrame::Unknown
    }
}

fn corners(aabb: &[f64; 6]) -> [[f64; 3]; 8] {
    let mut out = [[0.0; 3]; 8];
    for (i, slot) in out.iter_mut().enumerate() {
        *slot = [
            if i & 1 == 0 { aabb[0] } else { aabb[3] },
            if i & 2 == 0 { aabb[1] } else { aabb[4] },
            if i & 4 == 0 { aabb[2] } else { aabb[5] },
        ];
    }
    out
}

fn aabb_of(points: impl Iterator<Item = [f64; 3]>) -> Option<[f64; 6]> {
    let mut out: Option<[f64; 6]> = None;
    for p in points {
        out = Some(match out {
            None => [p[0], p[1], p[2], p[0], p[1], p[2]],
            Some(a) => [
                a[0].min(p[0]),
                a[1].min(p[1]),
                a[2].min(p[2]),
                a[3].max(p[0]),
                a[4].max(p[1]),
                a[5].max(p[2]),
            ],
        });
    }
    out
}

fn merge(a: [f64; 6], b: [f64; 6]) -> [f64; 6] {
    [
        a[0].min(b[0]),
        a[1].min(b[1]),
        a[2].min(b[2]),
        a[3].max(b[3]),
        a[4].max(b[4]),
        a[5].max(b[5]),
    ]
}

fn read_tag(bytes: &[u8], offset: usize) -> String {
    (0..4)
        .filter_map(|i| char::from_u32(read_u32(bytes, offset + i * 4).ok()?))
        .collect::<String>()
        .trim_end_matches('\0')
        .to_string()
}

fn read_u32(bytes: &[u8], offset: usize) -> Result<u32> {
    let slice = bytes
        .get(offset..offset + 4)
        .with_context(|| format!("读 u32 越界 @ {offset:#x}"))?;
    Ok(u32::from_be_bytes(slice.try_into().unwrap()))
}

fn read_f32(bytes: &[u8], offset: usize) -> Result<f32> {
    Ok(f32::from_bits(read_u32(bytes, offset)?))
}

fn read_string(data: &[u8], offset: usize) -> Result<(String, usize)> {
    let words = read_u32(data, offset)? as usize;
    let start = offset + 4;
    let end = start + words * 4;
    let slice = data
        .get(start..end)
        .with_context(|| format!("字串越界 @ {offset:#x}（{words} 字）"))?;
    let text = slice.split(|b| *b == 0).next().unwrap_or(&[]);
    Ok((String::from_utf8_lossy(text).into_owned(), end))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn swept_full_circle_reaches_the_outer_radius_on_both_axes() {
        let torus = Primitive::RectangularTorus([90.0, 110.0, 10.0, std::f64::consts::TAU]);
        let aabb = torus.local_aabb_mm().unwrap();
        for axis in 0..2 {
            assert!((aabb[axis] + 110.0).abs() < 0.2, "{aabb:?}");
            assert!((aabb[axis + 3] - 110.0).abs() < 0.2, "{aabb:?}");
        }
        assert!(
            (aabb[2] + 5.0).abs() < 1e-9 && (aabb[5] - 5.0).abs() < 1e-9,
            "{aabb:?}"
        );
    }

    #[test]
    fn quarter_torus_does_not_claim_the_far_quadrants() {
        // 0..90° 只扫到第一象限：X/Y 的下界是内半径侧，不是 −outer。
        let torus = Primitive::RectangularTorus([76.0, 106.0, 60.0, std::f64::consts::FRAC_PI_2]);
        let aabb = torus.local_aabb_mm().unwrap();
        assert!(aabb[0] >= -0.2, "X 下界不该跨到负半轴: {aabb:?}");
        assert!((aabb[3] - 106.0).abs() < 0.2, "{aabb:?}");
    }

    #[test]
    fn a_rotated_cylinder_is_not_bounded_by_its_rotated_box_corners() {
        // 这是 26229 那 56.7 mm 的成因：拿规范盒八角去转，曲面会虚高一圈。
        // 采样点云则贴着真实表面。绕 Z 转 45° 的圆柱，X/Y 半跨仍应是半径。
        let cylinder = Primitive::Cylinder([100.0, 400.0]);
        let hull = cylinder.local_hull_points_mm();
        let s = std::f64::consts::FRAC_1_SQRT_2;
        let rotated = aabb_of(
            hull.iter()
                .map(|v| [s * v[0] - s * v[1], s * v[0] + s * v[1], v[2]]),
        )
        .unwrap();
        assert!((rotated[3] - 100.0).abs() < 0.2, "{rotated:?}");
        // 八角近似会给出 100·√2 ≈ 141.4——差 41 mm，正是那一类假红灯。
        let corner_box = corners(&[-100.0, -100.0, -200.0, 100.0, 100.0, 200.0]);
        let naive = aabb_of(
            corner_box
                .iter()
                .map(|v| [s * v[0] - s * v[1], s * v[0] + s * v[1], v[2]]),
        )
        .unwrap();
        assert!(naive[3] > 140.0, "{naive:?}");
    }

    #[test]
    fn pyramid_splits_the_shift_between_the_two_faces() {
        // `/TEMP_EQUIP` PIPE 2 弯头的实测记录：bottom/top 都是 12×50、YOFF 50、高 50，
        // E3D 自己写进 RVM 的记录盒是 [−6, −50, −25, 6, 50, 25]。偏心整个记到顶面
        // 会得到 [−6, −25, −25, 6, 75, 25]，沿 Y 整体错 25 mm。
        let pyramid = Primitive::Pyramid([12.0, 50.0, 12.0, 50.0, 0.0, 50.0, 50.0]);
        let aabb = pyramid.local_aabb_mm().unwrap();
        assert_eq!(aabb, [-6.0, -50.0, -25.0, 6.0, 50.0, 25.0], "{aabb:?}");
    }

    #[test]
    fn eccentric_snout_declares_a_box_narrower_than_the_solid() {
        // `/装卸车汇管PI` 里 `/250-CGA-0311-B1/B1` 那只 REDUCER 的实测记录：
        // rb 136.5 / rt 109.55 / h 178 / XOFF 26.95，E3D 写的盒是 X ∈ ±123.025。
        let snout = Primitive::Snout([136.5, 109.55, 178.0, 26.95, 0.0, 0.0, 0.0, 0.0, 0.0]);
        let declared = snout.declared_box_mm().unwrap();
        assert_close(
            &declared,
            &[-123.025, -136.5, -89.0, 123.025, 136.5, 89.0],
            1e-9,
        );
        // 真几何在 −X 上比 E3D 那只盒宽出整整一个 XOFF——判帧拿 declared 去对，
        // 空间判定仍旧只认这一只。
        let hull = snout.local_aabb_mm().unwrap();
        assert_close(
            &hull,
            &[-149.975, -136.5, -89.0, 123.025, 136.5, 89.0],
            0.01,
        );
    }

    #[test]
    fn concentric_snout_has_no_separate_declared_box() {
        // 不偏心时 E3D 那条公式与真几何重合，5514 条实测记录都靠原路判帧，别多给一个候选。
        let snout = Primitive::Snout([136.5, 109.55, 178.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        assert_eq!(snout.declared_box_mm(), None);
    }

    #[test]
    fn circular_torus_matches_the_e3d_record_for_a_quarter_bend() {
        // ams7999 `/1RCV003VP` 的实测记录：offset 157.5 / radius 5 / angle π 时
        // 记录盒是 [0, 0, −5, 162.5, 162.5, 5]，与本模型逐轴吻合。
        let torus = Primitive::CircularTorus([157.5, 5.0, std::f64::consts::PI]);
        let aabb = torus.local_aabb_mm().unwrap();
        assert!((aabb[3] - 162.5).abs() < 0.1, "{aabb:?}");
        assert!((aabb[5] - 5.0).abs() < 1e-9, "{aabb:?}");
    }

    /// 手工拼一个最小 RVM：HEAD + MODL + CNTB + 一条 Box PRIM + CNTE + END。
    fn minimal_rvm() -> Vec<u8> {
        minimal_rvm_tagged("PRIM", None)
    }

    /// 同上，但几何记录的 tag 与等级字可换——用来钉 `OBST` / `INSU` 的参数偏移。
    fn minimal_rvm_tagged(geometry_tag: &str, level: Option<u32>) -> Vec<u8> {
        fn tag(out: &mut Vec<u8>, t: &str) {
            for c in t.chars() {
                out.extend_from_slice(&(c as u32).to_be_bytes());
            }
        }
        fn string(out: &mut Vec<u8>, s: &str) {
            let mut bytes = s.as_bytes().to_vec();
            while bytes.len() % 4 != 0 || bytes.len() == s.len() {
                bytes.push(0);
            }
            out.extend_from_slice(&((bytes.len() / 4) as u32).to_be_bytes());
            out.extend_from_slice(&bytes);
        }
        fn chunk(out: &mut Vec<u8>, t: &str, body: &[u8]) {
            tag(out, t);
            let next = out.len() + 4 + body.len();
            out.extend_from_slice(&(next as u32).to_be_bytes());
            out.extend_from_slice(body);
        }

        let mut head = vec![];
        head.extend_from_slice(&1u32.to_be_bytes());
        head.extend_from_slice(&2u32.to_be_bytes());
        for s in ["banner", "note", "date", "user", "Unicode UTF-8"] {
            string(&mut head, s);
        }

        let mut modl = vec![];
        modl.extend_from_slice(&1u32.to_be_bytes());
        modl.extend_from_slice(&1u32.to_be_bytes());
        string(&mut modl, "AMS");
        string(&mut modl, "/ALL");

        let mut cntb = vec![];
        cntb.extend_from_slice(&1u32.to_be_bytes());
        cntb.extend_from_slice(&2u32.to_be_bytes());
        string(&mut cntb, "BOX 1 of EQUIPMENT /E1");
        for v in [10.0f32, 20.0, 30.0] {
            cntb.extend_from_slice(&v.to_bits().to_be_bytes());
        }
        cntb.extend_from_slice(&1u32.to_be_bytes());

        let mut prim = vec![];
        prim.extend_from_slice(&1u32.to_be_bytes()); // version
        prim.extend_from_slice(&1u32.to_be_bytes()); // flags
        prim.extend_from_slice(&2u32.to_be_bytes()); // kind = Box
        // matrix3 = I × 0.001，平移 = (0.01, 0.02, 0.03) m = (10, 20, 30) mm
        for v in [
            0.001f32, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001, 0.01, 0.02, 0.03,
        ] {
            prim.extend_from_slice(&v.to_bits().to_be_bytes());
        }
        for v in [-50.0f32, -25.0, -15.0, 50.0, 25.0, 15.0] {
            prim.extend_from_slice(&v.to_bits().to_be_bytes());
        }
        if let Some(level) = level {
            prim.extend_from_slice(&level.to_be_bytes());
        }
        for v in [100.0f32, 50.0, 30.0] {
            prim.extend_from_slice(&v.to_bits().to_be_bytes());
        }

        let mut out = vec![];
        chunk(&mut out, "HEAD", &head);
        chunk(&mut out, "MODL", &modl);
        chunk(&mut out, "CNTB", &cntb);
        chunk(&mut out, geometry_tag, &prim);
        chunk(&mut out, "CNTE", &[0, 0, 0, 1, 0, 0, 0, 2]);
        chunk(&mut out, "END:", &[0, 0, 0, 1, 0, 0, 0, 1]);
        out.extend_from_slice(&[0u8; 16]);
        out
    }

    #[test]
    fn parses_the_minimal_file_with_all_three_unit_systems() {
        let file = RvmFile::parse(&minimal_rvm()).unwrap();
        assert_eq!(file.header.model, "AMS");
        assert_eq!(file.header.project, "/ALL");
        assert_eq!(file.header.encoding, "Unicode UTF-8");
        assert_eq!(file.members.len(), 1);

        let member = &file.members[0];
        assert_eq!(member.name, "BOX 1 of EQUIPMENT /E1");
        // CNTB 存的就是 mm，不换算。
        assert_close(&member.translation_mm, &[10.0, 20.0, 30.0], 1e-9);

        let geometry = &member.geometries[0];
        assert_eq!(geometry.primitive, Primitive::Box([100.0, 50.0, 30.0]));
        // 矩阵平移存的是米，读出来已经换成 mm。文件里是 f32，×1000 之后
        // 0.01f32 只能给到 9.9999998——这一档舍入是格式自带的，不是解析误差。
        assert_close(&geometry.translation_mm, &[10.0, 20.0, 30.0], 1e-5);
        // 3×3 原样带 0.001，rotation() 才是纯旋转。
        assert!((geometry.matrix3[0] - 0.001).abs() < 1e-9);
        assert!((geometry.rotation()[0] - 1.0).abs() < 1e-6);
    }

    /// `OBST` / `INSU` 在 bbox 与参数之间多一个等级字（expdri 的 `DRBOX`/`DRFACG` 里
    /// 那条与备用 tag 同一个判据的 `+= 4`）。八份基准样本一条都没有，所以读侧不按 tag
    /// 硬编码偏移，而是拿「正好吃到记录末尾」自证——这条测两个方向都钉住。
    #[test]
    fn obstruction_records_carry_a_level_word_before_the_params() {
        let file = RvmFile::parse(&minimal_rvm_tagged("OBST", Some(6))).unwrap();
        let geometry = &file.members[0].geometries[0];
        assert_eq!(geometry.tag, "OBST");
        assert_eq!(geometry.level, Some(6));
        // 等级字没跳过的话，参数会整体错一个 f32。
        assert_eq!(geometry.primitive, Primitive::Box([100.0, 50.0, 30.0]));

        // 反方向：没有等级字的 OBST 记录也得认出来，不能硬吃掉 4 个字节。
        let file = RvmFile::parse(&minimal_rvm_tagged("OBST", None)).unwrap();
        let geometry = &file.members[0].geometries[0];
        assert_eq!(geometry.level, None);
        assert_eq!(geometry.primitive, Primitive::Box([100.0, 50.0, 30.0]));
    }

    #[test]
    fn world_aabb_comes_from_the_geometry_not_the_record_field() {
        let file = RvmFile::parse(&minimal_rvm()).unwrap();
        let aabb = file.members[0].world_aabb_mm().unwrap();
        // 100×50×30 的盒，中心在 (10,20,30) mm。容差放到 1e-4：坐标经 f32 存过一轮。
        assert_close(&aabb, &[-40.0, -5.0, 15.0, 60.0, 45.0, 45.0], 1e-4);
    }

    #[track_caller]
    fn assert_close<const N: usize>(got: &[f64; N], want: &[f64; N], tolerance: f64) {
        let worst = got
            .iter()
            .zip(want.iter())
            .fold(0.0f64, |acc, (a, b)| acc.max((a - b).abs()));
        assert!(worst <= tolerance, "{got:?} vs {want:?}，最大差 {worst}");
    }

    #[test]
    fn axis_aligned_box_records_a_local_frame_bbox() {
        let file = RvmFile::parse(&minimal_rvm()).unwrap();
        assert_eq!(file.members[0].geometries[0].bbox_frame, BboxFrame::Local);
        assert_eq!(file.bbox_frame_tally().get("unknown"), None);
    }
}

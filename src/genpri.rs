//! 通用图元 GENPRI：几何是 `GENPRI → ARCHIV → ATTDAT.ELDATA` 里存的一份 SMLib B-rep，
//! 本模块把它解码、三角化成封闭的 [`Manifold`]。
//!
//! ## 出处
//!
//! - Core3D 3.1 `xba_drawgenericprimitive::getCSGTree@0x1077E190`：`xba_archive_reader::InitReading`
//!   打开元素名下的归档，`GeoGenericPrimitive::ReadFrom` 还原图元，交给
//!   `GeoGmlCreator(标签, max(CSG_TreeBuilderOptions + 32, 0.1), EOptions = 2)` 导出成 libgm 对象；
//!   B-rep 自带的变换不是单位阵时再包一层 `gm_CreateTransform`。
//! - XBaseDll 3.1 `GeoIwBrepGen::ReadFrom@0x102BEB00`：`GeoGenericPrimitiveImpl::ReadFrom`
//!   （`GeoTransf3d::ReadFrom`：类型字 1 = 单位阵，否则跟 16 个实数；归档版本 ≥ 3 才另有缓存面片与
//!   图元名）+ `GeoIwBrep::ReadFrom` → `sub_10248F30`：`GeoIwReader::ReadLong` 读 SMLib 数据版本
//!   （> 40 拒读）→ `IwBrepData::ReadFromDB` → `IwBrep::MakeTopologyFromData`。
//! - 三角化：`GeoGmlFunnel::Export(GeoGenericPrimitive)@0x102B5AE0` → `GeoIwBrepGen::ExportTo` →
//!   `GeoTessellationFunnel::Export(GeoIwBrep)@0x10261B10` → `GeoIwBrepImpl::Tessellate@0x102534F0`
//!   → SMLib `IwTess`，两把尺见 [`Tessellation`]。
//!
//! ## `ELDATA`
//!
//! `ARCHIV` 名下各 `ATTDAT` 的 `ELDATA` 按成员顺序首尾相接（单块至多 4096 字）。字是 u32；
//! 实数占两个字、低字在前，单位 mm；`99999999` 是「未设」占位。aps250162 的 15 种 GENPRI
//! （75 件）逐字解完、无剩余字：
//!
//! | 字 | 内容 |
//! |---|---|
//! | 0 | 归档版本 2 |
//! | 1 | 字节数 = 4 ×（字数 − 1） |
//! | 2–5 | UUID（= `ARCHIV.UUIDAR`） |
//! | 6 | `GeoTransf3d` 类型，1 = 单位阵 |
//! | 7 | SMLib 数据版本 32 |
//! | 8–13 | `3 0 1 2` + 模型容差 |
//! | 14–49 | 一个区域、一个壳，壳里 nF 张面（第 37、45、46 字是 nF） |
//!
//! 之后是一串「个数 + 记录」的表，记录以 3 个未设字开头（边用是 6 个）：
//!
//! | 表 | 记录 |
//! |---|---|
//! | 面用（2nF） | 面号、侧（前 nF 个 1，后 nF 个 0）、壳号 |
//! | 面（nF） | UV 范围 4 实数、容差、曲面号、首环、环数、0、0 |
//! | 环 | 0、边用数、首边用、0、0、0；表尾另有一个空表计数 0 |
//! | 边用 | 0、边、环、同向、参数曲线、两个带符号的配对引用、0、0 |
//! | 边 | 容差、三维曲线、起止顶点、首边用（带符号）、参数区间 2 实数、0 |
//! | 顶点 | 容差、坐标 3 实数、0 |
//! | 参数曲线、三维曲线 | 类码 14033（B 样条）：末号 n、次数、有理、维数，n+1 个控制点、n+次数+2 个节点、0 |
//! | 曲面 | 类码 15002（B 样条）：nu、nv、两向次数、有理，(nu+1)(nv+1) 个控制点（v 变化最快）、两向节点、0 |
//!
//! 有理控制点按齐次坐标存（x·w, y·w, z·w, w）。边用的参数曲线与边的三维曲线**同一套参数**：
//! aps250162 的 2093 条边用逐点核过，最大差 0.065 mm，与模型容差同一量级。
//!
//! 布局只认语料里见过的那一种（一区一壳、单位变换、数据版本 32、只有 B 样条）；别的一律
//! [`GenpriError::Layout`] 报出来，不猜。
//!
//! ## 归档版本 4：缓存面片
//!
//! aps250212 的地形模型（`TRNMOD → TRNCC → TRNCEL → GENPRI`，还有 TRNSLD / TRNSUR 下的）写的是版本 4：没有 B 样条、没有
//! SMLib 头，是 `GeoGenericPrimitiveImpl::ReadFrom` 在版本 ≥ 3 多读的那份**缓存面片**。E3D 的 TTY 导出写的就是它：6 格的多边形数、
//! 环数、点数、包围盒与导出逐一相同（格 6：103 个多边形逐环同点同序）。字节流（标志位占一字节，后面全部不对齐字界；
//! 整数 4 字节、实数 8 字节、小端）：
//!
//! | 字节 | 内容 |
//! |---|---|
//! | 0–3 | 归档版本 4 |
//! | 4–7 | 字节数（不含版本字，块尾按字补齐） |
//! | 8–23 | UUID |
//! | 24–27 | `GeoTransf3d` 类型，1 = 单位阵（TRNSUR 的是 2，后跟 16 个实数，没有可对拍的导出，不读） |
//! | 28 | 有缓存面片 = 1 |
//! | 29–32 | 1（面片组数） |
//! | 33… | 顶点号流：个数 n，然后 n 个整数——环是一串点号收于 `-点数`，面是几个环收于 `-2`，整流收于 `-1` |
//! | … | 顶点数 V，V × 3 实数（mm，GENPRI 局部系；地形链上 POS / ORI 全 0，即世界系） |
//! | … | 法向数 N，N × 3 实数；再一份同构的法向号流 |
//! | … | 每条边一组 `1 n 2`，没认，也用不上 |
//!
//! 一张面的几个环不全是「外环 + 孔」：E3D 导出时把与面法向反向的环当孔、同向的各写一个多边形（[`facet_polygons`]）。这些面片是面
//! 不是体（地形格是带一圈边界边的高程面；TRNSLD 的虽封闭却夹着零面积的折返环），建不成流形，照 E3D 只写 RVM（[`GenpriBuild::Surface`]）。

use std::collections::{HashMap, VecDeque};
use std::ops::Range;

use glam::{DVec2, DVec3};
use manifold_csg::Manifold;
use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation};

use e3d_io::db_element::{DbElement, DbElementError};
use e3d_io::record::descriptor::DescriptorValue;

const UNSET: u32 = 99_999_999;
const BSPLINE_CURVE: i32 = 14_033;
const BSPLINE_SURFACE: i32 = 15_002;
const SMLIB_VERSION: i32 = 32;
const MAX_DEGREE: usize = 7;
/// 细分树最深这么多层、一条边的二分最多走这么多步：坏数据（零长节点、NaN）不至于把三角化拖进死循环。
const MAX_TREE_DEPTH: usize = 16;
const MAX_BISECTION_STEPS: usize = 4096;
/// 一条边按 UV 尺寸最多再断这么多次（`IwTess::CheckAgainstSurfaceCache` 的段数上限）。
const MAX_UV_SPLITS: usize = 100;
/// 量等参线长度时每条取这么多段折线。
const ISO_LENGTH_SAMPLES: usize = 16;
/// 顶点按这个格距（mm）合并：退化边（球极）的各采样点、相邻边在顶点处的端点都落到同一个索引。
const WELD_MM: f64 = 1e-6;

#[derive(Debug, thiserror::Error)]
pub enum GenpriError {
    #[error(transparent)]
    Db(#[from] DbElementError),
    #[error("ELDATA 第 {at} 字：{reason}")]
    Layout { at: usize, reason: String },
    #[error("B-rep 引用越界：{0}")]
    Topology(String),
    #[error("三角化失败：{0}")]
    Tessellation(String),
    #[error("三角网格不成流形：{0}")]
    NotManifold(String),
}

/// 非均匀有理 B 样条曲线：边的三维曲线（`dim` 3）或边用的参数曲线（`dim` 2，UV）。
#[derive(Debug, Clone)]
pub struct BSplineCurve {
    pub degree: usize,
    pub rational: bool,
    pub dim: usize,
    /// 齐次控制点 (x·w, y·w, z·w, w)；非有理 w = 1，二维 z = 0。
    pub control: Vec<[f64; 4]>,
    pub knots: Vec<f64>,
}

impl BSplineCurve {
    pub fn point(&self, t: f64) -> DVec3 {
        let span = find_span(&self.knots, self.degree, self.control.len(), t);
        let mut weights = [0.0; MAX_DEGREE + 1];
        basis(&self.knots, self.degree, span, t, &mut weights);
        let mut acc = [0.0; 4];
        for (k, b) in weights[..=self.degree].iter().enumerate() {
            let cp = &self.control[span - self.degree + k];
            for (sum, x) in acc.iter_mut().zip(cp) {
                *sum += b * x;
            }
        }
        DVec3::new(acc[0], acc[1], acc[2]) / acc[3]
    }

    /// 参数曲线在 t 处的 UV。等参线（控制点某一分量全同）那一分量直接取控制点的值：求值的舍入
    /// 会把一条直边界抹成微微凹凸的折线。
    fn uv(&self, t: f64) -> DVec2 {
        let mut uv = self.point(t).truncate();
        for axis in 0..2 {
            let first = self.control[0][axis] / self.control[0][3];
            if self.control.iter().all(|c| c[axis] / c[3] == first) {
                uv[axis] = first;
            }
        }
        uv
    }

    /// 等参线（控制点某一分量全同）：那一分量是 u（0）还是 v（1）、值多少。两个分量都不变（退化成点）
    /// 不算。
    fn iso_axis(&self) -> Option<(usize, f64)> {
        let constant = |axis: usize| {
            let first = self.control[0][axis] / self.control[0][3];
            self.control
                .iter()
                .all(|c| c[axis] / c[3] == first)
                .then_some(first)
        };
        match (constant(0), constant(1)) {
            (Some(u), None) => Some((0, u)),
            (None, Some(v)) => Some((1, v)),
            _ => None,
        }
    }

    /// 一阶导：齐次曲线按基函数导数（Piegl & Tiller 式 2.9）求导，再按商法则除回 w。
    fn derivative(&self, t: f64) -> DVec3 {
        let p = self.degree;
        let span = find_span(&self.knots, p, self.control.len(), t);
        let mut value = [0.0; MAX_DEGREE + 1];
        basis(&self.knots, p, span, t, &mut value);
        let mut slope = [0.0; MAX_DEGREE + 1];
        if p > 0 {
            let mut lower = [0.0; MAX_DEGREE + 1];
            basis(&self.knots, p - 1, span, t, &mut lower);
            let over = |n: f64, i: usize, j: usize| {
                let d = self.knots[j] - self.knots[i];
                if d == 0.0 { 0.0 } else { n / d }
            };
            for (k, s) in slope[..=p].iter_mut().enumerate() {
                let i = span - p + k;
                let left = if k > 0 {
                    over(lower[k - 1], i, i + p)
                } else {
                    0.0
                };
                let right = if k < p {
                    over(lower[k], i + 1, i + p + 1)
                } else {
                    0.0
                };
                *s = p as f64 * (left - right);
            }
        }
        let mut a = [0.0; 4];
        let mut da = [0.0; 4];
        for (k, cp) in self.control[span - p..=span].iter().enumerate() {
            for ((sum, dsum), x) in a.iter_mut().zip(da.iter_mut()).zip(cp) {
                *sum += value[k] * x;
                *dsum += slope[k] * x;
            }
        }
        let point = DVec3::new(a[0], a[1], a[2]);
        let speed = DVec3::new(da[0], da[1], da[2]);
        (speed * a[3] - point * da[3]) / (a[3] * a[3])
    }
}

/// 非均匀有理 B 样条曲面，控制点下标 `i·count[1] + j`（v 变化最快）。
#[derive(Debug, Clone)]
pub struct BSplineSurface {
    pub degree: [usize; 2],
    pub rational: bool,
    pub count: [usize; 2],
    /// 齐次控制点 (x·w, y·w, z·w, w)。
    pub control: Vec<[f64; 4]>,
    pub knots: [Vec<f64>; 2],
}

impl BSplineSurface {
    pub fn point(&self, u: f64, v: f64) -> DVec3 {
        let [du, dv] = self.degree;
        let su = find_span(&self.knots[0], du, self.count[0], u);
        let sv = find_span(&self.knots[1], dv, self.count[1], v);
        let mut bu = [0.0; MAX_DEGREE + 1];
        let mut bv = [0.0; MAX_DEGREE + 1];
        basis(&self.knots[0], du, su, u, &mut bu);
        basis(&self.knots[1], dv, sv, v, &mut bv);
        let mut acc = [0.0; 4];
        for (a, wu) in bu[..=du].iter().enumerate() {
            for (b, wv) in bv[..=dv].iter().enumerate() {
                let cp = &self.control[(su - du + a) * self.count[1] + (sv - dv + b)];
                for (sum, x) in acc.iter_mut().zip(cp) {
                    *sum += wu * wv * x;
                }
            }
        }
        DVec3::new(acc[0], acc[1], acc[2]) / acc[3]
    }
}

#[derive(Debug, Clone)]
pub struct Face {
    /// u0, u1, v0, v1：修剪后的参数范围。
    pub uv_box: [f64; 4],
    pub surface: usize,
    /// 首环是外边界，其余是孔。
    pub loops: Range<usize>,
}

#[derive(Debug, Clone)]
pub struct Loop {
    pub uses: Range<usize>,
}

#[derive(Debug, Clone)]
pub struct EdgeUse {
    pub edge: usize,
    /// 沿边的参数增向走。
    pub same_sense: bool,
    pub pcurve: usize,
}

#[derive(Debug, Clone)]
pub struct Edge {
    pub curve: usize,
    pub vertices: [usize; 2],
    /// 边用到的三维曲线参数区间。
    pub range: [f64; 2],
}

/// 解码后的 B-rep（只留建体要用的那部分拓扑）。
#[derive(Debug, Clone)]
pub struct Brep {
    pub tolerance: f64,
    pub faces: Vec<Face>,
    pub loops: Vec<Loop>,
    pub uses: Vec<EdgeUse>,
    pub edges: Vec<Edge>,
    pub vertices: Vec<DVec3>,
    pub pcurves: Vec<BSplineCurve>,
    pub curves: Vec<BSplineCurve>,
    pub surfaces: Vec<BSplineSurface>,
}

/// GENPRI 的两种结局。
pub enum GenpriBuild {
    Solid(Manifold),
    /// 归档版本 4 的缓存面片：是面不是体（aps250212 的地形格是带边界环的高程面，地形体 TRNSLD 虽封闭但夹着零面积的
    /// 折返环），建不成流形；E3D 照样写成一条 `FacetGroup`，我们也只写 RVM（[`facet_polygons`]）。
    Surface(CachedFacets),
    /// 有依据地不出几何（没有归档；或小到 E3D 的 `IwTess` 门槛以下）。
    NoShape(String),
}

/// GENPRI 的局部实体（单位 mm，GENPRI 自己的 POS / ORI 系；E3D 里这两项全是 0）。
pub fn generic_primitive_local_solid(
    el: &DbElement,
    notes: &mut Vec<String>,
) -> Result<GenpriBuild, GenpriError> {
    let Some(words) = eldata_words(el)? else {
        return Ok(GenpriBuild::NoShape(
            "没有 ARCHIV → ATTDAT.ELDATA，GENPRI 无几何可读（E3D 同样不出）".to_string(),
        ));
    };
    match decode_archive(&words)? {
        Archive::Brep(brep) => {
            let Some(mesh) = tessellate(&brep, notes)? else {
                return Ok(GenpriBuild::NoShape(
                    "B-rep 包围盒次长边 < 1 mm 且最长边 < 20 mm，E3D IwTess 前就放弃（GeoIwBrepImpl::Tessellate）"
                        .to_string(),
                ));
            };
            Ok(GenpriBuild::Solid(mesh_solid(&mesh)?))
        }
        // 缓存面片不过 IwTess：E3D 照样写出去，没有尺寸门。
        Archive::Facets(facets) => Ok(GenpriBuild::Surface(facets)),
    }
}

/// 归档版本 4 的缓存面片（`GeoGenericPrimitiveImpl::ReadFrom` 在版本 ≥ 3 多读的那份）：E3D 的 TTY 导出写的就是它
/// （aps250212 地形 6 格的环数、点数、包围盒与 E3D 的面片逐一相同，见模块文档）。
#[derive(Debug, Clone)]
pub struct CachedFacets {
    pub vertices: Vec<DVec3>,
    /// 面 → 环 → 顶点号。一张面的几个环可能是外环 + 孔，也可能是同一张平面上互不嵌套的几块、甚至不共面的几块
    /// （E3D 导出时后两种各写一个多边形），分组见 [`facets_mesh`]。
    pub faces: Vec<Vec<Vec<u32>>>,
}

/// 一块 `ELDATA` 解出来的几何：版本 2 是 SMLib B-rep，版本 4 是缓存面片。
#[derive(Debug, Clone)]
pub enum Archive {
    Brep(Brep),
    Facets(CachedFacets),
}

/// 按第 0 字的归档版本分派：2 → [`decode`]，4 → 缓存面片；别的版本还没有语料，照例 [`GenpriError::Layout`]。
pub fn decode_archive(words: &[u32]) -> Result<Archive, GenpriError> {
    match words.first().map(|&w| w as i32) {
        Some(2) => Ok(Archive::Brep(decode(words)?)),
        Some(4) => Ok(Archive::Facets(decode_cached_facets(words)?)),
        Some(other) => Err(GenpriError::Layout {
            at: 0,
            reason: format!("归档版本 {other}，只认 2（SMLib B-rep）与 4（缓存面片）"),
        }),
        None => Err(GenpriError::Layout {
            at: 0,
            reason: "空块".to_string(),
        }),
    }
}

/// 版本 4 是字节流（标志位占一字节，后面的整数与实数都不对齐字界），按字节读；报错位置折成字号、原因里带字节号。
struct Bytes<'a> {
    data: &'a [u8],
    at: usize,
}

impl<'a> Bytes<'a> {
    fn fail<T>(&self, at: usize, reason: impl Into<String>) -> Result<T, GenpriError> {
        Err(GenpriError::Layout {
            at: at / 4,
            reason: format!("（版本 4 字节流第 {at} 字节）{}", reason.into()),
        })
    }

    fn take(&mut self, n: usize) -> Result<&'a [u8], GenpriError> {
        let Some(slice) = self.data.get(self.at..self.at + n) else {
            return self.fail(self.at, "数据提前结束");
        };
        self.at += n;
        Ok(slice)
    }

    fn byte(&mut self) -> Result<u8, GenpriError> {
        Ok(self.take(1)?[0])
    }

    fn int(&mut self) -> Result<i32, GenpriError> {
        let b = self.take(4)?;
        Ok(i32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    fn real(&mut self) -> Result<f64, GenpriError> {
        let b = self.take(8)?;
        Ok(f64::from_le_bytes([b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]]))
    }

    fn count(&mut self, what: &str) -> Result<usize, GenpriError> {
        let at = self.at;
        let value = self.int()?;
        match usize::try_from(value) {
            Ok(n) if n <= self.data.len() => Ok(n),
            _ => self.fail(at, format!("{what} = {value} 不合理")),
        }
    }
}

/// 版本 4 的整块 `ELDATA` → [`CachedFacets`]（布局见模块文档）。
fn decode_cached_facets(words: &[u32]) -> Result<CachedFacets, GenpriError> {
    let data: Vec<u8> = words.iter().flat_map(|w| w.to_le_bytes()).collect();
    let mut r = Bytes { data: &data, at: 0 };
    let version = r.int()?;
    if version != 4 {
        return r.fail(0, format!("归档版本 {version}，缓存面片只认 4"));
    }
    // 字节数不含版本字，块尾按字补齐（aps250212 的 8 块都多 3 字节）。
    let bytes = r.int()?;
    let payload = i64::try_from(data.len()).unwrap_or(i64::MAX) - 4;
    if !(i64::from(bytes) <= payload && i64::from(bytes) + 4 > payload) {
        return r.fail(
            4,
            format!("声明 {bytes} 字节，实有 {} 字（除版本字外 {payload} 字节）", words.len()),
        );
    }
    r.take(16)?; // UUID = `ARCHIV.UUIDAR`
    let transform = r.int()?;
    if transform != 1 {
        return r.fail(
            24,
            format!("GeoTransf3d 类型 {transform}（非单位阵）尚无可对拍的语料"),
        );
    }
    let at = r.at;
    let cached = r.byte()?;
    if cached != 1 {
        return r.fail(at, format!("缓存面片标志 {cached}，只认 1（有面片）"));
    }
    let at = r.at;
    let sets = r.int()?;
    if sets != 1 {
        return r.fail(at, format!("面片组数 {sets}，只认 1"));
    }
    let faces = read_ring_stream(&mut r, "顶点号流")?;
    let vertex_count = r.count("顶点数")?;
    let mut vertices = Vec::with_capacity(vertex_count);
    for _ in 0..vertex_count {
        vertices.push(DVec3::new(r.real()?, r.real()?, r.real()?));
    }
    // 后面是法向表与同构的法向号流（绕向由 [`orient_triangles`] 统一，这里只核结构）；再往后每条边一组 `1 n 2`
    // 的字节还没认，三角化用不上。
    let normal_count = r.count("法向数")?;
    r.take(24 * normal_count)?;
    let normal_rings = read_ring_stream(&mut r, "法向号流")?;
    if normal_rings.len() != faces.len() {
        return r.fail(
            at,
            format!("法向号流 {} 张面，顶点号流 {} 张", normal_rings.len(), faces.len()),
        );
    }
    for (fi, face) in faces.iter().enumerate() {
        for ring in face {
            if ring.len() < 3 {
                return Err(GenpriError::Topology(format!("面 {fi} 有不足 3 点的环")));
            }
            if let Some(&v) = ring.iter().find(|&&v| v as usize >= vertex_count) {
                return Err(GenpriError::Topology(format!(
                    "面 {fi} 引用顶点 {v}，只有 {vertex_count} 个"
                )));
            }
        }
    }
    Ok(CachedFacets { vertices, faces })
}

/// 「个数 + 流」：环是一串号码收于 `-点数`，面是几个环收于 `-2`，整流收于 `-1`；个数把这三种收尾都算上。
fn read_ring_stream(r: &mut Bytes, what: &str) -> Result<Vec<Vec<Vec<u32>>>, GenpriError> {
    let start = r.at;
    let n = r.count(what)?;
    let mut faces = Vec::new();
    let mut rings: Vec<Vec<u32>> = Vec::new();
    let mut ring: Vec<u32> = Vec::new();
    let mut read = 0usize;
    loop {
        if read >= n {
            return r.fail(start, format!("{what}：{n} 个字里没有结束符 -1"));
        }
        let at = r.at;
        let value = r.int()?;
        read += 1;
        match value {
            -1 => break,
            -2 => {
                if !ring.is_empty() || rings.is_empty() {
                    return r.fail(at, format!("{what}：-2 之前应是收完环的面"));
                }
                faces.push(std::mem::take(&mut rings));
            }
            v if v < 0 => {
                if ring.len() != v.unsigned_abs() as usize {
                    return r.fail(
                        at,
                        format!("{what}：环收尾 {v} 与环上 {} 个点不符", ring.len()),
                    );
                }
                rings.push(std::mem::take(&mut ring));
            }
            v => ring.push(v as u32),
        }
    }
    if read != n {
        return r.fail(start, format!("{what}：声明 {n} 个字，-1 在第 {read} 个"));
    }
    if !ring.is_empty() || !rings.is_empty() {
        return r.fail(start, format!("{what}：结束符前有没收完的环"));
    }
    Ok(faces)
}

/// 判两个环共面的距离门（mm）：平面面的点是 SMLib 算出来的双精度，共面的误差在 1e-9 量级；不共面的那些差的是整层高度。
const COPLANAR_MM: f64 = 0.05;

/// 缓存面片 → E3D 写进 RVM 的多边形（局部 mm；每个多边形第一个环是外环、其余是孔）：每张面的环按「共面且被包着」
/// 分组（[`group_rings`]），面的顺序与环内点的顺序照归档。aps250212 六格的环数、各环点数与 E3D TTY 导出逐一相同。
pub fn facet_polygons(facets: &CachedFacets) -> Vec<Vec<Vec<DVec3>>> {
    let points = &facets.vertices;
    facets
        .faces
        .iter()
        .flat_map(|face| group_rings(face, points))
        .map(|group| {
            group
                .into_iter()
                .map(|ring| ring.into_iter().map(|i| points[i as usize]).collect())
                .collect()
        })
        .collect()
}

/// 环的单位法向（Newell）、面积与顶点平均点。
fn ring_plane(ring: &[u32], points: &[DVec3]) -> (DVec3, f64, DVec3) {
    let mut normal = DVec3::ZERO;
    let mut centre = DVec3::ZERO;
    for k in 0..ring.len() {
        let a = points[ring[k] as usize];
        let b = points[ring[(k + 1) % ring.len()] as usize];
        normal += a.cross(b);
        centre += a;
    }
    (
        normal.normalize_or_zero(),
        0.5 * normal.length(),
        centre / ring.len().max(1) as f64,
    )
}

/// 平面内的两条正交轴。
fn plane_basis(normal: DVec3) -> (DVec3, DVec3) {
    let helper = if normal.x.abs() < 0.9 { DVec3::X } else { DVec3::Y };
    let u = normal.cross(helper).normalize();
    (u, normal.cross(u))
}

fn project_ring(ring: &[u32], points: &[DVec3], origin: DVec3, u: DVec3, v: DVec3) -> Vec<DVec2> {
    ring.iter()
        .map(|&i| {
            let d = points[i as usize] - origin;
            DVec2::new(d.dot(u), d.dot(v))
        })
        .collect()
}

/// 一张面的环分组，按环向（SMLib 的约定：外环与面法向同向、孔反向）：与最大环同向的各自是一个多边形（照归档顺序），
/// 反向的是孔，挂到共面且包着它最多顶点的外环上（同样多时取面积小的；谁都不包就挂最大的那个）——数顶点而不是只看质心，
/// 孔里套着岛时质心会落在岛里。这就是 E3D 写导出时的取舍：aps250212 TRNCEL 2
/// 的面 149 / 4410（外环 + 其内反向的 32 / 28 边两环）E3D 写成一个带两孔的多边形；面 0（同在 z −250 上、同向的 57 / 91
/// 边两块）、TRNCEL 3 的面 0（同向、互相交缠的 20 / 21 边两块）与 TRNCEL 1 的面 10（z 0 与 z −5000 各一块）都拆成两个
/// 多边形——只看包含关系会把 TRNCEL 3 那对误认成外环 + 孔。返回每组的环，第一个是外环。
fn group_rings(face: &[Vec<u32>], points: &[DVec3]) -> Vec<Vec<Vec<u32>>> {
    if face.len() <= 1 {
        return vec![face.to_vec()];
    }
    let planes: Vec<(DVec3, f64, DVec3)> = face.iter().map(|r| ring_plane(r, points)).collect();
    let largest = (0..face.len())
        .max_by(|&a, &b| planes[a].1.total_cmp(&planes[b].1))
        .unwrap_or(0);
    let reference = planes[largest].0;
    let outers: Vec<usize> = (0..face.len())
        .filter(|&k| planes[k].0.dot(reference) >= 0.0)
        .collect();
    let mut groups: Vec<Vec<Vec<u32>>> = outers.iter().map(|&k| vec![face[k].clone()]).collect();
    let largest_group = outers.iter().position(|&k| k == largest).unwrap_or(0);
    for j in (0..face.len()).filter(|&k| planes[k].0.dot(reference) < 0.0) {
        let cj = planes[j].2;
        // (组号, 包住的顶点数, 外环面积)：顶点数多者优先，同样多时面积小者优先。
        let mut best: Option<(usize, usize, f64)> = None;
        for (g, &i) in outers.iter().enumerate() {
            let (ni, ai, ci) = planes[i];
            if ni.dot(cj - ci).abs() > COPLANAR_MM {
                continue;
            }
            let (u, v) = plane_basis(ni);
            let ring_i = project_ring(&face[i], points, ci, u, v);
            let ring_j = project_ring(&face[j], points, ci, u, v);
            let votes = ring_j
                .iter()
                .filter(|&&p| inside(std::slice::from_ref(&ring_i), p))
                .count();
            if votes > 0
                && best.is_none_or(|(_, n, a)| votes > n || (votes == n && ai < a))
            {
                best = Some((g, votes, ai));
            }
        }
        groups[best.map_or(largest_group, |(g, _, _)| g)].push(face[j].clone());
    }
    groups
}

/// `GENPRI → ARCHIV → ATTDAT.ELDATA` 按成员顺序接起来的全部字；链不在时 `None`。
pub fn eldata_words(el: &DbElement) -> Result<Option<Vec<u32>>, GenpriError> {
    let Some(archive) = el.members_of_type("ARCHIV")?.into_iter().next() else {
        return Ok(None);
    };
    let mut words = Vec::new();
    for attdat in archive.members_of_type("ATTDAT")? {
        match attdat.attribute("ELDATA")? {
            Some(DescriptorValue::RawWords(chunk)) | Some(DescriptorValue::WordArray(chunk)) => {
                words.extend(chunk)
            }
            None | Some(DescriptorValue::Unset) => {}
            Some(other) => {
                return Err(GenpriError::Layout {
                    at: words.len(),
                    reason: format!("{} 的 ELDATA 不是字块：{other:?}", attdat.refno()),
                });
            }
        }
    }
    Ok((!words.is_empty()).then_some(words))
}

struct Words<'a> {
    words: &'a [u32],
    at: usize,
}

impl Words<'_> {
    fn fail<T>(&self, at: usize, reason: impl Into<String>) -> Result<T, GenpriError> {
        Err(GenpriError::Layout {
            at,
            reason: reason.into(),
        })
    }

    fn word(&mut self) -> Result<u32, GenpriError> {
        let Some(&word) = self.words.get(self.at) else {
            return self.fail(self.at, "数据提前结束");
        };
        self.at += 1;
        Ok(word)
    }

    fn int(&mut self) -> Result<i32, GenpriError> {
        Ok(self.word()? as i32)
    }

    /// 非负的个数或下标；顺手挡掉大过剩余字数的个数（坏数据别去分配几个 G）。
    fn count(&mut self, what: &str) -> Result<usize, GenpriError> {
        let at = self.at;
        let value = self.int()?;
        match usize::try_from(value) {
            Ok(n) if n <= self.words.len() => Ok(n),
            _ => self.fail(at, format!("{what} = {value} 不合理")),
        }
    }

    fn flag(&mut self, what: &str) -> Result<bool, GenpriError> {
        let at = self.at;
        match self.int()? {
            0 => Ok(false),
            1 => Ok(true),
            other => self.fail(at, format!("{what} 应为 0 / 1，是 {other}")),
        }
    }

    fn real(&mut self) -> Result<f64, GenpriError> {
        let low = u64::from(self.word()?);
        let high = u64::from(self.word()?);
        Ok(f64::from_bits((high << 32) | low))
    }

    fn unset(&mut self, n: usize) -> Result<(), GenpriError> {
        for _ in 0..n {
            let at = self.at;
            if self.word()? != UNSET {
                return self.fail(at, "应为未设占位 99999999");
            }
        }
        Ok(())
    }

    fn expect(&mut self, values: &[i32], what: &str) -> Result<(), GenpriError> {
        let at = self.at;
        for &value in values {
            if self.int()? != value {
                let got = &self.words[at..(at + values.len()).min(self.words.len())];
                return self.fail(at, format!("{what} 应为 {values:?}，是 {got:?}"));
            }
        }
        Ok(())
    }
}

/// 整块 `ELDATA` → [`Brep`]。
pub fn decode(words: &[u32]) -> Result<Brep, GenpriError> {
    let mut r = Words { words, at: 0 };
    let version = r.int()?;
    if version != 2 {
        return r.fail(0, format!("归档版本 {version}，只认 2"));
    }
    let bytes = r.int()?;
    if i64::from(bytes) != 4 * (words.len() as i64 - 1) {
        return r.fail(1, format!("声明 {bytes} 字节，实有 {} 字", words.len()));
    }
    r.at = 6;
    let transform = r.int()?;
    if transform != 1 {
        return r.fail(
            6,
            format!("GeoTransf3d 类型 {transform}（非单位阵）尚无语料"),
        );
    }
    let smlib = r.int()?;
    if smlib != SMLIB_VERSION {
        return r.fail(7, format!("SMLib 数据版本 {smlib}，只认过 {SMLIB_VERSION}"));
    }
    r.expect(&[3, 0, 1, 2], "IwBrepData 头")?;
    let tolerance = r.real()?;
    r.expect(&[1, 0, 2, 0, 0, 0, 0, 1, 0, 1, 0], "区域表")?;
    r.unset(3)?;
    r.expect(&[1, 1, 0, 0, 2], "区域")?;
    r.unset(3)?;
    let face_count = words.get(r.at + 1).map(|&w| w as i32).unwrap_or(-1);
    r.expect(
        &[
            0, face_count, 0, 1, 0, 0, 0, 0, 0, face_count, face_count, 1, 0, 0,
        ],
        "壳（只认一区一壳）",
    )?;
    let face_count = usize::try_from(face_count).unwrap_or(0);

    let at = r.at;
    let faceuses = r.count("面用数")?;
    if faceuses != 2 * face_count {
        return r.fail(
            at,
            format!("面用 {faceuses} 个，应为面数的两倍 {}", 2 * face_count),
        );
    }
    for k in 0..faceuses {
        r.unset(3)?;
        let side = i32::from(k < face_count);
        r.expect(&[(k % face_count.max(1)) as i32, side, 0], "面用")?;
    }

    let at = r.at;
    if r.count("面数")? != face_count {
        return r.fail(at, "面表个数与壳里的面数不一致");
    }
    let mut faces = Vec::with_capacity(face_count);
    for _ in 0..face_count {
        r.unset(3)?;
        let uv_box = [r.real()?, r.real()?, r.real()?, r.real()?];
        let _tolerance = r.real()?;
        let surface = r.count("曲面号")?;
        let first = r.count("首环")?;
        let count = r.count("环数")?;
        r.expect(&[0, 0], "面尾")?;
        faces.push(Face {
            uv_box,
            surface,
            loops: first..first + count,
        });
    }

    let mut loops = Vec::new();
    for _ in 0..r.count("环数")? {
        r.unset(3)?;
        r.expect(&[0], "环头")?;
        let count = r.count("边用数")?;
        let first = r.count("首边用")?;
        r.expect(&[0, 0, 0], "环尾")?;
        loops.push(Loop {
            uses: first..first + count,
        });
    }
    r.expect(&[0], "环表后的空表")?;

    let mut uses = Vec::new();
    for _ in 0..r.count("边用数")? {
        r.unset(6)?;
        r.expect(&[0], "边用头")?;
        let edge = r.count("边号")?;
        let _loop = r.count("环号")?;
        let same_sense = r.flag("同向")?;
        let pcurve = r.count("参数曲线号")?;
        let _pair = [r.int()?, r.int()?];
        r.expect(&[0, 0], "边用尾")?;
        uses.push(EdgeUse {
            edge,
            same_sense,
            pcurve,
        });
    }

    let mut edges = Vec::new();
    for _ in 0..r.count("边数")? {
        r.unset(3)?;
        let _tolerance = r.real()?;
        let curve = r.count("三维曲线号")?;
        let vertices = [r.count("起点")?, r.count("终点")?];
        let _first_use = r.int()?;
        let range = [r.real()?, r.real()?];
        r.expect(&[0], "边尾")?;
        edges.push(Edge {
            curve,
            vertices,
            range,
        });
    }

    let mut vertices = Vec::new();
    for _ in 0..r.count("顶点数")? {
        r.unset(3)?;
        let _tolerance = r.real()?;
        vertices.push(DVec3::new(r.real()?, r.real()?, r.real()?));
        r.expect(&[0], "顶点尾")?;
    }

    let mut pcurves = Vec::new();
    for _ in 0..r.count("参数曲线数")? {
        pcurves.push(read_curve(&mut r)?);
    }
    let mut curves = Vec::new();
    for _ in 0..r.count("三维曲线数")? {
        curves.push(read_curve(&mut r)?);
    }
    let mut surfaces = Vec::new();
    for _ in 0..r.count("曲面数")? {
        surfaces.push(read_surface(&mut r)?);
    }
    if r.at != words.len() {
        return r.fail(r.at, format!("曲面表之后还剩 {} 字", words.len() - r.at));
    }

    let brep = Brep {
        tolerance,
        faces,
        loops,
        uses,
        edges,
        vertices,
        pcurves,
        curves,
        surfaces,
    };
    check_references(&brep)?;
    Ok(brep)
}

fn read_knots(r: &mut Words, count: usize) -> Result<Vec<f64>, GenpriError> {
    let at = r.at;
    let knots = (0..count)
        .map(|_| r.real())
        .collect::<Result<Vec<_>, _>>()?;
    if knots.iter().any(|k| !k.is_finite()) || knots.windows(2).any(|w| w[1] < w[0]) {
        return r.fail(at, "节点向量不是有限的非降序列");
    }
    Ok(knots)
}

fn read_control(
    r: &mut Words,
    count: usize,
    dim: usize,
    rational: bool,
) -> Result<Vec<[f64; 4]>, GenpriError> {
    let at = r.at;
    if 2 * count * (dim + usize::from(rational)) > r.words.len() - r.at.min(r.words.len()) {
        return r.fail(at, format!("{count} 个控制点超出数据"));
    }
    let mut control = Vec::with_capacity(count);
    for _ in 0..count {
        let mut cp = [0.0, 0.0, 0.0, 1.0];
        for slot in cp.iter_mut().take(dim) {
            *slot = r.real()?;
        }
        if rational {
            cp[3] = r.real()?;
        }
        if cp.iter().any(|x| !x.is_finite()) || !(cp[3] > 0.0) {
            return r.fail(at, "控制点不是有限值或权不为正");
        }
        control.push(cp);
    }
    Ok(control)
}

fn read_curve(r: &mut Words) -> Result<BSplineCurve, GenpriError> {
    let at = r.at;
    let kind = r.int()?;
    if kind != BSPLINE_CURVE {
        return r.fail(at, format!("曲线类码 {kind}，只认 B 样条 {BSPLINE_CURVE}"));
    }
    let last = r.count("控制点末号")?;
    let degree = r.count("次数")?;
    let rational = r.flag("有理")?;
    let dim = r.count("维数")?;
    if !(1..=MAX_DEGREE).contains(&degree) || last < degree || !(2..=3).contains(&dim) {
        return r.fail(at, format!("曲线 n={last} 次数 {degree} 维数 {dim} 不合理"));
    }
    let control = read_control(r, last + 1, dim, rational)?;
    let knots = read_knots(r, last + degree + 2)?;
    r.expect(&[0], "曲线尾")?;
    Ok(BSplineCurve {
        degree,
        rational,
        dim,
        control,
        knots,
    })
}

fn read_surface(r: &mut Words) -> Result<BSplineSurface, GenpriError> {
    let at = r.at;
    let kind = r.int()?;
    if kind != BSPLINE_SURFACE {
        return r.fail(
            at,
            format!("曲面类码 {kind}，只认 B 样条 {BSPLINE_SURFACE}"),
        );
    }
    let last = [r.count("u 向末号")?, r.count("v 向末号")?];
    let degree = [r.count("u 向次数")?, r.count("v 向次数")?];
    let rational = r.flag("有理")?;
    if (0..2).any(|d| !(1..=MAX_DEGREE).contains(&degree[d]) || last[d] < degree[d]) {
        return r.fail(at, format!("曲面 n={last:?} 次数 {degree:?} 不合理"));
    }
    let count = [last[0] + 1, last[1] + 1];
    let control = read_control(r, count[0] * count[1], 3, rational)?;
    let ku = read_knots(r, last[0] + degree[0] + 2)?;
    let kv = read_knots(r, last[1] + degree[1] + 2)?;
    r.expect(&[0], "曲面尾")?;
    Ok(BSplineSurface {
        degree,
        rational,
        count,
        control,
        knots: [ku, kv],
    })
}

fn check_references(b: &Brep) -> Result<(), GenpriError> {
    let bad = |what: String| Err(GenpriError::Topology(what));
    for (i, f) in b.faces.iter().enumerate() {
        if f.surface >= b.surfaces.len() || f.loops.end > b.loops.len() || f.loops.is_empty() {
            return bad(format!("面 {i} 的曲面 / 环 {:?}", f.loops));
        }
    }
    for (i, l) in b.loops.iter().enumerate() {
        if l.uses.end > b.uses.len() || l.uses.is_empty() {
            return bad(format!("环 {i} 的边用 {:?}", l.uses));
        }
    }
    for (i, u) in b.uses.iter().enumerate() {
        if u.edge >= b.edges.len() || u.pcurve >= b.pcurves.len() || b.pcurves[u.pcurve].dim != 2 {
            return bad(format!("边用 {i} 的边 / 参数曲线"));
        }
    }
    for (i, e) in b.edges.iter().enumerate() {
        if e.curve >= b.curves.len()
            || b.curves[e.curve].dim != 3
            || e.vertices.iter().any(|&v| v >= b.vertices.len())
            || !(e.range[0] <= e.range[1])
        {
            return bad(format!("边 {i} 的曲线 / 顶点 / 参数区间"));
        }
    }
    Ok(())
}

fn find_span(knots: &[f64], degree: usize, count: usize, t: f64) -> usize {
    let n = count - 1;
    if t >= knots[n + 1] {
        return n;
    }
    if t <= knots[degree] {
        return degree;
    }
    let (mut lo, mut hi) = (degree, n + 1);
    let mut mid = (lo + hi) / 2;
    while t < knots[mid] || t >= knots[mid + 1] {
        if t < knots[mid] {
            hi = mid;
        } else {
            lo = mid;
        }
        mid = (lo + hi) / 2;
    }
    mid
}

/// Cox–de Boor（Piegl & Tiller A2.2）：`span` 上 `degree + 1` 个非零基函数。
fn basis(knots: &[f64], degree: usize, span: usize, t: f64, out: &mut [f64; MAX_DEGREE + 1]) {
    let mut left = [0.0; MAX_DEGREE + 1];
    let mut right = [0.0; MAX_DEGREE + 1];
    out[0] = 1.0;
    for j in 1..=degree {
        left[j] = t - knots[span + 1 - j];
        right[j] = knots[span + j] - t;
        let mut saved = 0.0;
        for r in 0..j {
            let denominator = right[r + 1] + left[j - r];
            let temp = if denominator == 0.0 {
                0.0
            } else {
                out[r] / denominator
            };
            out[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        out[j] = saved;
    }
}

/// `knots` 里严格落在 (lo, hi) 内的不同值。
fn interior_knots(knots: &[f64], lo: f64, hi: f64) -> Vec<f64> {
    let eps = 1e-12 * (hi - lo).abs().max(1.0);
    let mut out: Vec<f64> = Vec::new();
    for &k in knots {
        if k > lo + eps && k < hi - eps && out.last().is_none_or(|&last| k - last > eps) {
            out.push(k);
        }
    }
    out
}

/// E3D 交给 SMLib `IwTess` 的两把尺（`GeoIwBrepImpl::Tessellate@0x102534F0`）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tessellation {
    /// 弦高容差（mm）。
    pub chord: f64,
    /// 最大边长（mm）= 0.4 × B-rep 包围盒对角线（常量 `qword_10665338` = 0.4）。
    pub max_edge: f64,
}

/// 导出容差：`max(CSG_TreeBuilderOptions + 32, 0.1)`。RVM 导出时它取多少还没从调用侧坐实，
/// 这里取的是拿 aps250162 的 E3D 分段反推出来的那一档：单段整弧接受的弦高最大见过 5 mm
/// （R5 半圆一段），拒绝的最小见过 7.3 mm（R25 四分之一圆）。
const E3D_EXPORT_CHORD_MM: f64 = 6.0;

impl Tessellation {
    /// `GeoIwBrepImpl::Tessellate` 的取尺：B-rep 包围盒三边排序 `l0 ≤ l1 ≤ l2`；`l1 < 1 ∧ l2 < 20`
    /// 不三角化（返回 `None`）；细长件（`l2/l0 > 50 ∧ l1/l0 < 15 ∧ l0/弦高 < 5`）弦高收到
    /// `min(max(0.2·l0, 2), 弦高)`；最大边长 = 0.4 × 对角线。
    pub fn for_extent(size: DVec3, chord: f64) -> Option<Self> {
        let mut sides = [size.x.abs(), size.y.abs(), size.z.abs()];
        sides.sort_by(f64::total_cmp);
        let [l0, l1, l2] = sides;
        if l1 < 1.0 && l2 < 20.0 {
            return None;
        }
        let mut chord = chord;
        if l0 > 0.0 && l2 / l0 > 50.0 && l1 / l0 < 15.0 && l0 / chord < 5.0 {
            chord = (l0 * 0.2).max(2.0).min(chord);
        }
        Some(Self {
            chord,
            max_edge: 0.4 * size.length(),
        })
    }
}

/// 三角网格（局部 mm）。
#[derive(Debug, Clone, Default)]
pub struct TriangleMesh {
    pub points: Vec<DVec3>,
    pub triangles: Vec<[u32; 3]>,
    /// 与 `triangles` 逐个对齐：来自 B-rep 的哪张面。
    pub face_of_triangle: Vec<usize>,
}

#[derive(Default)]
struct Points {
    points: Vec<DVec3>,
    index: HashMap<[i64; 3], u32>,
}

impl Points {
    fn add(&mut self, p: DVec3) -> u32 {
        let key = [
            (p.x / WELD_MM).round() as i64,
            (p.y / WELD_MM).round() as i64,
            (p.z / WELD_MM).round() as i64,
        ];
        let points = &mut self.points;
        *self.index.entry(key).or_insert_with(|| {
            points.push(p);
            (points.len() - 1) as u32
        })
    }
}

fn distance_to_segment(p: DVec3, a: DVec3, b: DVec3) -> f64 {
    let ab = b - a;
    let len2 = ab.length_squared();
    if len2 == 0.0 {
        return p.distance(a);
    }
    let t = ((p - a).dot(ab) / len2).clamp(0.0, 1.0);
    p.distance(a + ab * t)
}

/// [lo, hi] 按节点断开、每个节点区间再等分；返回含两端的升序参数。
fn split_by_spans(
    knots: &[f64],
    lo: f64,
    hi: f64,
    mut per_span: impl FnMut(f64, f64) -> usize,
) -> Vec<f64> {
    let mut breaks = vec![lo];
    breaks.extend(interior_knots(knots, lo, hi));
    breaks.push(hi);
    let mut out = vec![lo];
    for pair in breaks.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        let n = per_span(a, b).max(1);
        for s in 1..=n {
            out.push(if s == n {
                b
            } else {
                a + (b - a) * s as f64 / n as f64
            });
        }
    }
    out
}

/// 一张面的细分树叶子 `[u0, u1, v0, v1]`（SMLib `IwTessSrfCache::BuildTreeWithSubdivision` →
/// `IwSurfaceCache::SubdivideToTolerances`，节点判据 `TestAgainstTolerances`）。根是曲面的整个参数域。
struct FaceTree {
    leaves: Vec<[f64; 4]>,
}

fn face_tree(surface: &BSplineSurface, tess: &Tessellation) -> FaceTree {
    let [du, dv] = surface.degree;
    let [nu, nv] = surface.count;
    let root = [
        surface.knots[0][du],
        surface.knots[0][nu],
        surface.knots[1][dv],
        surface.knots[1][nv],
    ];
    let mut leaves = Vec::new();
    let mut stack = vec![(root, 0usize)];
    while let Some((node, depth)) = stack.pop() {
        let dir = if depth < MAX_TREE_DEPTH {
            split_direction(surface, node, root, tess)
        } else {
            None
        };
        let Some(dir) = dir else {
            leaves.push(node);
            continue;
        };
        let at = split_param(&surface.knots[dir], node[2 * dir], node[2 * dir + 1]);
        let (mut low, mut high) = (node, node);
        low[2 * dir + 1] = at;
        high[2 * dir] = at;
        stack.push((high, depth + 1));
        stack.push((low, depth + 1));
    }
    FaceTree { leaves }
}

/// 节点要不要分、沿哪向分（0 = u，1 = v）：控制网厚度（`IwPseudoBox` 的第三个区间）超弦高，就沿控制网
/// 弯得多的那向分；否则尺寸向量（[`size_vector`]）的模超最大边长，就沿长的那向分。要分的那向已不到
/// 整个参数域的千分之一就不分，另一向还宽过百分之一就改分另一向。
fn split_direction(
    surface: &BSplineSurface,
    node: [f64; 4],
    root: [f64; 4],
    tess: &Tessellation,
) -> Option<usize> {
    let net = sub_net(surface, node);
    let thickness = net_thickness(&net);
    let dir = if thickness > tess.chord {
        let bend_u = (0..net[0].len())
            .map(|j| polyline_deviation(&net.iter().map(|row| row[j]).collect::<Vec<_>>()))
            .fold(0.0, f64::max);
        let bend_v = net
            .iter()
            .map(|row| polyline_deviation(row))
            .fold(0.0, f64::max);
        usize::from(bend_v > bend_u)
    } else {
        let planar = thickness <= 1e-9 * tess.max_edge.max(1.0);
        let size = size_vector(surface, node, planar);
        if size.length() <= tess.max_edge {
            return None;
        }
        usize::from(size.y >= size.x)
    };
    let width = |d: usize, b: [f64; 4]| b[2 * d + 1] - b[2 * d];
    if 0.001 * width(dir, root) <= width(dir, node) {
        Some(dir)
    } else if width(1 - dir, root) / 100.0 < width(1 - dir, node) {
        Some(1 - dir)
    } else {
        None
    }
}

/// 节点在 [lo, hi] 的中点分；离中点四分之一宽以内有节点值（knot）就分在离中点最近的那个上
/// （`IwSurfaceCache::SubdivideNode`）。
fn split_param(knots: &[f64], lo: f64, hi: f64) -> f64 {
    let mid = 0.5 * (lo + hi);
    interior_knots(knots, lo, hi)
        .into_iter()
        .filter(|k| (k - mid).abs() <= 0.25 * (hi - lo))
        .min_by(|a, b| (a - mid).abs().total_cmp(&(b - mid).abs()))
        .unwrap_or(mid)
}

/// 节点那块曲面的控制网（插节点截出来、除回 w）：`net[i][j]`，i 沿 u、j 沿 v。
fn sub_net(surface: &BSplineSurface, node: [f64; 4]) -> Vec<Vec<DVec3>> {
    let [nu, nv] = surface.count;
    let columns: Vec<Vec<[f64; 4]>> = (0..nv)
        .map(|j| (0..nu).map(|i| surface.control[i * nv + j]).collect())
        .collect();
    let columns = clip_polygons(
        &surface.knots[0],
        surface.degree[0],
        columns,
        node[0],
        node[1],
    );
    let rows: Vec<Vec<[f64; 4]>> = (0..columns[0].len())
        .map(|i| columns.iter().map(|column| column[i]).collect())
        .collect();
    clip_polygons(&surface.knots[1], surface.degree[1], rows, node[2], node[3])
        .into_iter()
        .map(|row| {
            row.into_iter()
                .map(|c| DVec3::new(c[0], c[1], c[2]) / c[3])
                .collect()
        })
        .collect()
}

/// 共用 `knots` 的几条齐次控制多边形：在 a、b 处插节点（Boehm）到 `degree` 重，截出 [a, b] 那一段。
fn clip_polygons(
    knots: &[f64],
    degree: usize,
    mut polygons: Vec<Vec<[f64; 4]>>,
    a: f64,
    b: f64,
) -> Vec<Vec<[f64; 4]>> {
    let mut knots = knots.to_vec();
    for t in [a, b] {
        if t <= knots[degree] || t >= knots[knots.len() - degree - 1] {
            continue;
        }
        while knots.iter().filter(|&&k| k == t).count() < degree {
            let n = polygons[0].len();
            let span = find_span(&knots, degree, n, t);
            for polygon in &mut polygons {
                let refined: Vec<[f64; 4]> = (0..=n)
                    .map(|i| {
                        if i + degree <= span {
                            polygon[i]
                        } else if i > span {
                            polygon[i - 1]
                        } else {
                            let d = knots[i + degree] - knots[i];
                            let s = if d == 0.0 { 0.0 } else { (t - knots[i]) / d };
                            std::array::from_fn(|c| {
                                (1.0 - s) * polygon[i - 1][c] + s * polygon[i][c]
                            })
                        }
                    })
                    .collect();
                *polygon = refined;
            }
            knots.insert(span + 1, t);
        }
    }
    let first = knots
        .iter()
        .rposition(|&k| k <= a)
        .unwrap_or(degree)
        .saturating_sub(degree);
    let last = knots
        .iter()
        .position(|&k| k >= b)
        .unwrap_or(knots.len() - degree - 1)
        .saturating_sub(1);
    polygons
        .into_iter()
        .map(|p| {
            let last = last.min(p.len() - 1);
            p[first.min(last)..=last].to_vec()
        })
        .collect()
}

/// 控制网沿法向的厚度（`IwPseudoBox` 的第三个区间）：法向取两向角点差之和的叉积。有一向的角点重合
/// （整圈闭合面、极点）时取包围盒最短边。
fn net_thickness(net: &[Vec<DVec3>]) -> f64 {
    let (last_i, last_j) = (net.len() - 1, net[0].len() - 1);
    let (c00, c10) = (net[0][0], net[last_i][0]);
    let (c01, c11) = (net[0][last_j], net[last_i][last_j]);
    let along_u = (c10 - c00) + (c11 - c01);
    let along_v = (c01 - c00) + (c11 - c10);
    let points = net.iter().flatten();
    if along_u.length() < 1e-9 || along_v.length() < 1e-9 {
        let lo = points.clone().fold(DVec3::INFINITY, |m, p| m.min(*p));
        let hi = points.fold(DVec3::NEG_INFINITY, |m, p| m.max(*p));
        return (hi - lo).min_element();
    }
    let normal = along_u.cross(along_v);
    if normal.length() < 1e-12 {
        return 0.0;
    }
    let normal = normal.normalize();
    let (lo, hi) = points.fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
        let d = (*p - c00).dot(normal);
        (lo.min(d), hi.max(d))
    });
    hi - lo
}

/// 点列离首尾连线的最大距离。
fn polyline_deviation(points: &[DVec3]) -> f64 {
    let (a, b) = (points[0], points[points.len() - 1]);
    points
        .iter()
        .map(|p| distance_to_segment(*p, a, b))
        .fold(0.0, f64::max)
}

/// `IwSurfaceCache::ComputeSizeTolerances`：u 向等参线在 v = 起 / 中 / 止三条的最大长度、v 向在
/// u = 起 / 中 / 止三条的最大长度；平的节点用四角弦长。
fn size_vector(surface: &BSplineSurface, node: [f64; 4], planar: bool) -> DVec2 {
    let [u0, u1, v0, v1] = node;
    if planar {
        let (p00, p10) = (surface.point(u0, v0), surface.point(u1, v0));
        let (p01, p11) = (surface.point(u0, v1), surface.point(u1, v1));
        return DVec2::new(
            p00.distance(p10).max(p01.distance(p11)),
            p00.distance(p01).max(p10.distance(p11)),
        );
    }
    let length = |at: &dyn Fn(f64) -> DVec3, lo: f64, hi: f64| {
        let mut previous = at(lo);
        (1..=ISO_LENGTH_SAMPLES)
            .map(|k| {
                let p = at(lo + (hi - lo) * k as f64 / ISO_LENGTH_SAMPLES as f64);
                let d = p.distance(previous);
                previous = p;
                d
            })
            .sum::<f64>()
    };
    let ends_and_middle = |lo: f64, hi: f64| [lo, 0.5 * (lo + hi), hi];
    let along_u = ends_and_middle(v0, v1)
        .into_iter()
        .map(|v| length(&|u| surface.point(u, v), u0, u1))
        .fold(0.0, f64::max);
    let along_v = ends_and_middle(u0, u1)
        .into_iter()
        .map(|u| length(&|v| surface.point(u, v), v0, v1))
        .fold(0.0, f64::max);
    DVec2::new(along_u, along_v)
}

/// `IwCurve::TessellateByBisection` 一步 [t, t + h] 的判定（TbAg.dll 2.10 `sub_1045DA70`）。
#[derive(Clone, Copy, PartialEq, Eq)]
enum Step {
    Reject,
    Accept,
    /// 接受，且弦高不到容差一半：同一步里把步长加倍再试。
    Grow,
}

/// 弦高取 Hermite 三次（两端点与切向）内侧两个 Bézier 点 `P0 + D0·h/3`、`P1 − D1·h/3` 到弦的较大距离
/// （`IwHermiteCurve::ComputeChordHeight`）；两端重合（整圈）时取控制多边形长度。
fn step_verdict(p0: DVec3, d0: DVec3, p1: DVec3, d1: DVec3, h: f64, chord: f64) -> Step {
    let b1 = p0 + d0 * (h / 3.0);
    let b2 = p1 - d1 * (h / 3.0);
    let height = if p0.distance_squared(p1) >= 1e-24 {
        distance_to_segment(b1, p0, p1).max(distance_to_segment(b2, p0, p1))
    } else {
        p0.distance(b1) + b1.distance(b2) + b2.distance(p1)
    };
    if height > chord {
        Step::Reject
    } else if height < 0.5 * chord {
        Step::Grow
    } else {
        Step::Accept
    }
}

/// 曲线在 (lo, hi) 内的 C1 断点：重数 ≥ 次数的节点值。
fn c1_breaks(curve: &BSplineCurve, lo: f64, hi: f64) -> Vec<f64> {
    curve
        .knots
        .chunk_by(|a, b| a == b)
        .filter(|run| run.len() >= curve.degree && run[0] > lo + 1e-12 && run[0] < hi - 1e-12)
        .map(|run| run[0])
        .collect()
}

/// SMLib `IwCurve::TessellateByBisection`（E3D 的曲线驱动：弦高 = `chord`，角度 0，长度不限）在
/// [t0, t1] 上的参数（含两端）。每步从上一步的步长起试：弦高超容差就减半；不到一半就加倍再试，
/// 加倍后超了退回上一个好步长；跨过 C1 断点的步还要以断点为终点再验一次；闭合曲线整段一步不收。
/// aps250162 的 793 条边，弦高 5.5–8 mm 时 727 条与 E3D 的段数相同、1 条多一段，其余是 E3D 另有
/// 面侧的分点。
fn bisection_params(curve: &BSplineCurve, t0: f64, t1: f64, chord: f64) -> Vec<f64> {
    let total = t1 - t0;
    if total.is_nan() || total <= 0.0 {
        return vec![t0, t1];
    }
    let min_step = total / 100_000.0;
    let tol = (t0.abs().max(t1.abs()) + 1.0) * 1e-12;
    let breaks = c1_breaks(curve, t0, t1);
    let at = |t: f64| (curve.point(t), curve.derivative(t));
    let mut params = vec![t0];
    let (mut t, mut h) = (t0, total);
    let (mut p0, mut d0) = at(t0);
    while t1 - t > tol && params.len() < MAX_BISECTION_STEPS {
        let mut grown: Option<f64> = None;
        for _ in 0..=100 {
            if t + h > t1 {
                h = t1 - t;
            }
            if t1 - tol > t + h && 2.0 * h + t > tol + t1 {
                h = t1 - t;
            }
            if h <= min_step - 1e-12 {
                break;
            }
            let (p1, d1) = at(t + h);
            let mut step = step_verdict(p0, d0, p1, d1, h, chord);
            if step != Step::Reject
                && breaks.iter().any(|&b| {
                    t < b && b < t + h && {
                        let (pb, db) = at(b);
                        step_verdict(p0, d0, pb, db, b - t, chord) == Step::Reject
                    }
                })
            {
                step = Step::Reject;
            }
            if step == Step::Grow
                && t == t0
                && (h - total).abs() <= tol
                && p0.distance(p1) < (p0.abs().max_element() + 1.0) * 1e-12
            {
                step = Step::Reject;
            }
            match step {
                Step::Accept => break,
                Step::Grow => {
                    grown = Some(h);
                    if t1 - tol < t + h {
                        h = t1 - t;
                        break;
                    }
                    h = (2.0 * h).min(t1 - t);
                }
                Step::Reject => {
                    if let Some(good) = grown {
                        h = good;
                        break;
                    }
                    h *= 0.5;
                    if h < min_step {
                        h = min_step;
                        break;
                    }
                }
            }
        }
        t += h;
        params.push(t);
        (p0, d0) = at(t);
        h = h.min(t1 - t);
    }
    if params.len() == 1 || t1 - t > tol {
        params.push(t1);
    } else if let Some(last) = params.last_mut() {
        *last = t1;
    }
    params
}

/// 升序、去重（相对 1e-10），两端钉回 t0、t1。
fn tidy(mut ts: Vec<f64>, t0: f64, t1: f64) -> Vec<f64> {
    ts.sort_by(f64::total_cmp);
    let eps = 1e-10 * (t1 - t0).abs().max(1e-300);
    ts.dedup_by(|b, a| (*b - *a).abs() <= eps);
    if ts.len() < 2 {
        return vec![t0, t1];
    }
    let n = ts.len();
    ts[0] = t0;
    ts[n - 1] = t1;
    ts
}

/// `IwTess::SplitFaceEdgesToUVSize` → `CheckAgainstSurfaceCache(…, 1.1, …)`：边用的一段，端点连线在 UV
/// 里（两头各去 1%）的包围盒比它压到的任一叶子宽出 1.1 倍，就从参数中点断开、两半再查；三维长度
/// 不到 `min_length` 的不断。
fn split_to_leaf_size(
    pc: &BSplineCurve,
    curve: &BSplineCurve,
    tree: &FaceTree,
    min_length: f64,
    ts: &mut Vec<f64>,
) {
    let mut pending: Vec<(f64, f64)> = ts.windows(2).map(|w| (w[0], w[1])).collect();
    let mut splits = 0;
    while let Some((a, b)) = pending.pop() {
        if splits >= MAX_UV_SPLITS {
            break;
        }
        let (pa, pb) = (pc.uv(a), pc.uv(b));
        let (qa, qb) = (pa.lerp(pb, 0.01), pa.lerp(pb, 0.99));
        let (lo, hi) = (qa.min(qb), qa.max(qb));
        let size = hi - lo;
        let too_wide = tree.leaves.iter().any(|leaf| {
            leaf[0] <= hi.x
                && lo.x <= leaf[1]
                && leaf[2] <= hi.y
                && lo.y <= leaf[3]
                && (size.x > 1.1 * (leaf[1] - leaf[0]) || size.y > 1.1 * (leaf[3] - leaf[2]))
        });
        if !too_wide {
            continue;
        }
        let m = 0.5 * (a + b);
        let (ca, cm, cb) = (curve.point(a), curve.point(m), curve.point(b));
        if ca.distance(cb) < min_length && ca.distance(cm) + cm.distance(cb) < min_length {
            continue;
        }
        insert_sorted(ts, m);
        splits += 1;
        pending.push((a, m));
        pending.push((m, b));
    }
}

/// 参数曲线在 [t0, t1] 上穿过等参线（第 `axis` 个分量 = `values` 里某值）的参数。
/// 沿等参线走的那一段不算穿过。
fn iso_crossings(
    pc: &BSplineCurve,
    t0: f64,
    t1: f64,
    axis: usize,
    values: &[f64],
    out: &mut Vec<f64>,
) {
    let spans = interior_knots(&pc.knots, t0, t1).len() + 1;
    let n = 32 * spans;
    let eps = 1e-12;
    let at = |t: f64| pc.uv(t)[axis];
    let mut prev_t = t0;
    let mut prev = at(t0);
    for s in 1..=n {
        let t = if s == n {
            t1
        } else {
            t0 + (t1 - t0) * s as f64 / n as f64
        };
        let cur = at(t);
        for &g in values {
            let (da, db) = (prev - g, cur - g);
            if da.abs() <= eps && db.abs() <= eps {
                continue;
            }
            if db.abs() <= eps {
                if s < n {
                    out.push(t);
                }
                continue;
            }
            if da.abs() <= eps || (da < 0.0) == (db < 0.0) {
                continue;
            }
            let (mut a, mut b, mut fa) = (prev_t, t, da);
            for _ in 0..60 {
                let m = 0.5 * (a + b);
                let fm = at(m) - g;
                if (fm < 0.0) == (fa < 0.0) {
                    a = m;
                    fa = fm;
                } else {
                    b = m;
                }
            }
            out.push(0.5 * (a + b));
        }
        prev_t = t;
        prev = cur;
    }
}

/// 一条边的采样：升序参数与对应的网格点号（两端是 B-rep 顶点本身）。
struct EdgeSamples {
    params: Vec<f64>,
    ids: Vec<u32>,
}

/// 修剪环自交时最多加密这么多轮：前几轮把相交的那几段二分，最后一轮还交着的就是曲线真相交，
/// 在交点处两边各加一个采样、共用同一个点（捏合点）。E3D 的网格里同样是这样：aps250162
/// GENPRI 13 端面上两段圆弧交叉 0.003 mm，E3D 在交点两侧各密密加了一串点，交点本身成了顶点，
/// 夹出来的那一小块单独剖成一个三角形（对应 `IwTess::CheckPolygonIntersections` /
/// `SplitSegmentOfLoop` / `TraceLaminaLoop`）。
const MAX_REFINE_ROUNDS: usize = 8;

/// 两条边在交点处共用一个点：`[(边, 参数), (边, 参数)]`。
type Pinch = [(usize, f64); 2];

/// 一张面的修剪环在（拉伸后的）UV 里的一段：两端、属于哪条边、两端在那条边上的参数。
struct RingSegment {
    a: DVec2,
    b: DVec2,
    edge: usize,
    ta: f64,
    tb: f64,
}

/// 修剪环上的一个点：拉伸后的 UV、边、边上第几个参数、参数值。
type RingPoint = (DVec2, usize, usize, f64);

/// 一张面的修剪环与全部段。捏合点两侧在这张面里都出现时，第二侧的 UV 取第一侧的，环在那里只是相碰。
fn face_rings(
    brep: &Brep,
    face: &Face,
    scale: DVec2,
    params: &[Vec<f64>],
    pinches: &[Pinch],
) -> (Vec<Vec<RingPoint>>, Vec<RingSegment>) {
    let mut rings: Vec<Vec<RingPoint>> = Vec::new();
    let mut segments = Vec::new();
    for l in &brep.loops[face.loops.clone()] {
        let mut ring = Vec::new();
        for ui in l.uses.clone() {
            let u = &brep.uses[ui];
            let pc = &brep.pcurves[u.pcurve];
            let mut along: Vec<RingPoint> = params[u.edge]
                .iter()
                .enumerate()
                .map(|(k, &t)| (pc.uv(t) * scale, u.edge, k, t))
                .collect();
            if !u.same_sense {
                along.reverse();
            }
            for pair in along.windows(2) {
                segments.push(RingSegment {
                    a: pair[0].0,
                    b: pair[1].0,
                    edge: u.edge,
                    ta: pair[0].3,
                    tb: pair[1].3,
                });
            }
            along.pop();
            ring.extend(along);
        }
        if ring.len() >= 2 {
            rings.push(ring);
        }
    }
    for [(ea, ta), (eb, tb)] in pinches {
        let find = |e: usize, t: f64| {
            rings
                .iter()
                .flatten()
                .find(|p| p.1 == e && p.3 == t)
                .map(|p| p.0)
        };
        if let (Some(uv), Some(_)) = (find(*ea, *ta), find(*eb, *tb)) {
            for p in rings.iter_mut().flatten() {
                if p.1 == *eb && p.3 == *tb {
                    p.0 = uv;
                }
            }
        }
    }
    (rings, segments)
}

/// 两条线段在各自内部的交点参数（0..1）。
fn crossing_params(p: &RingSegment, q: &RingSegment) -> (f64, f64) {
    let r = p.b - p.a;
    let s = q.b - q.a;
    let denominator = r.perp_dot(s);
    if denominator == 0.0 {
        return (0.5, 0.5);
    }
    let w = q.a - p.a;
    (
        (w.perp_dot(s) / denominator).clamp(0.0, 1.0),
        (w.perp_dot(r) / denominator).clamp(0.0, 1.0),
    )
}

fn insert_sorted(ts: &mut Vec<f64>, t: f64) {
    let at = ts.partition_point(|&x| x < t);
    if ts.get(at) != Some(&t) {
        ts.insert(at, t);
    }
}

/// 两段在各自内部相交（共端点、只在端点相碰都不算）。
fn segments_cross(p: &RingSegment, q: &RingSegment) -> bool {
    let orient = |a: DVec2, b: DVec2, c: DVec2| (b - a).perp_dot(c - a);
    let scale = (p.b - p.a).length().max((q.b - q.a).length());
    let eps = 1e-12 * scale * scale;
    let (d1, d2) = (orient(q.a, q.b, p.a), orient(q.a, q.b, p.b));
    let (d3, d4) = (orient(p.a, p.b, q.a), orient(p.a, p.b, q.b));
    if [d1, d2, d3, d4].iter().any(|d| d.abs() <= eps) {
        return false;
    }
    (d1 > 0.0) != (d2 > 0.0) && (d3 > 0.0) != (d4 > 0.0)
}

/// UV 拉伸：按三维长度把参数缩放再剖分，免得参数方向的比例把三角形压扁。
fn uv_scale(surface: &BSplineSurface, uv_box: [f64; 4]) -> DVec2 {
    let [u0, u1, v0, v1] = uv_box;
    let du = (u1 - u0).max(1e-12);
    let dv = (v1 - v0).max(1e-12);
    let (mut su, mut sv) = (0.0f64, 0.0f64);
    for a in [0.25, 0.5, 0.75] {
        let u = u0 + du * a;
        let v = v0 + dv * a;
        su = su.max(surface.point(u0, v).distance(surface.point(u1, v)) / du);
        sv = sv.max(surface.point(u, v0).distance(surface.point(u, v1)) / dv);
    }
    DVec2::new(su.max(1e-9), sv.max(1e-9))
}

/// B-rep → 三角网格。包围盒小到 E3D 不三角化时 `None`。
///
/// 顺序照 `IwTess`：先给各面建细分树（[`face_tree`]），再定边的采样——等参边在相邻面叶子的边界处断开
/// （`IsoEdgeSubdivide`），比所压叶子宽的段从中点断开（[`split_to_leaf_size`]），每段再按
/// `TessellateByBisection` 二分（[`bisection_params`]）；修剪环在 UV 里自交就把相交的那几段二分重来
/// （粗采样的窄槽两岸会交叉），最后逐面剖分。边的采样两侧面共用，故水密。没照搬的是 `BuildTree`
/// 之后的边界植入（`ImplantUVSegment`）与 `IwTess` 自己的剖分，顶点集与 E3D 并不全同。
pub fn tessellate(
    brep: &Brep,
    notes: &mut Vec<String>,
) -> Result<Option<TriangleMesh>, GenpriError> {
    let (lo, hi) = extent(brep);
    let Some(tess) = Tessellation::for_extent(hi - lo, E3D_EXPORT_CHORD_MM) else {
        return Ok(None);
    };
    let trees: Vec<FaceTree> = brep
        .faces
        .iter()
        .map(|f| face_tree(&brep.surfaces[f.surface], &tess))
        .collect();
    let scales: Vec<DVec2> = brep
        .faces
        .iter()
        .map(|f| uv_scale(&brep.surfaces[f.surface], f.uv_box))
        .collect();

    let mut face_of_use = vec![usize::MAX; brep.uses.len()];
    for (fi, f) in brep.faces.iter().enumerate() {
        for l in &brep.loops[f.loops.clone()] {
            for ui in l.uses.clone() {
                face_of_use[ui] = fi;
            }
        }
    }
    let mut uses_of_edge: Vec<Vec<usize>> = vec![Vec::new(); brep.edges.len()];
    for (ui, u) in brep.uses.iter().enumerate() {
        uses_of_edge[u.edge].push(ui);
    }

    // 闭合边（整圈）另在曲线节点处断开：E3D 把整圈的细圆柱切成参数 16 等分，这条规则在 `IwTess` 里
    // 还没找到出处；按节点断开至少留住象限点。
    let mut params: Vec<Vec<f64>> = Vec::with_capacity(brep.edges.len());
    for (ei, e) in brep.edges.iter().enumerate() {
        let curve = &brep.curves[e.curve];
        let [t0, t1] = e.range;
        let mut ts = vec![t0, t1];
        if curve.point(t0).distance(curve.point(t1)) <= WELD_MM {
            ts.extend(interior_knots(&curve.knots, t0, t1));
        }
        for &ui in &uses_of_edge[ei] {
            let fi = face_of_use[ui];
            if fi == usize::MAX {
                continue;
            }
            let pc = &brep.pcurves[brep.uses[ui].pcurve];
            if let Some((axis, value)) = pc.iso_axis() {
                let across = 1 - axis;
                let mut bounds: Vec<f64> = trees[fi]
                    .leaves
                    .iter()
                    .filter(|leaf| leaf[2 * axis] <= value && value <= leaf[2 * axis + 1])
                    .flat_map(|leaf| [leaf[2 * across], leaf[2 * across + 1]])
                    .collect();
                bounds.sort_by(f64::total_cmp);
                bounds.dedup();
                iso_crossings(pc, t0, t1, across, &bounds, &mut ts);
            }
        }
        params.push(tidy(ts, t0, t1));
    }
    for (fi, face) in brep.faces.iter().enumerate() {
        for l in &brep.loops[face.loops.clone()] {
            for u in &brep.uses[l.uses.clone()] {
                split_to_leaf_size(
                    &brep.pcurves[u.pcurve],
                    &brep.curves[brep.edges[u.edge].curve],
                    &trees[fi],
                    20.0 * brep.tolerance,
                    &mut params[u.edge],
                );
            }
        }
    }
    for (e, ts) in brep.edges.iter().zip(params.iter_mut()) {
        let curve = &brep.curves[e.curve];
        let pieces = std::mem::take(ts);
        let mut all = Vec::with_capacity(pieces.len());
        for pair in pieces.windows(2) {
            all.extend(bisection_params(curve, pair[0], pair[1], tess.chord));
        }
        *ts = tidy(all, e.range[0], e.range[1]);
    }

    let mut pinches: Vec<Pinch> = Vec::new();
    for round in 0..MAX_REFINE_ROUNDS {
        let last = round + 1 == MAX_REFINE_ROUNDS;
        let mut inserts: Vec<(usize, f64)> = Vec::new();
        for (fi, face) in brep.faces.iter().enumerate() {
            let (_, segments) = face_rings(brep, face, scales[fi], &params, &pinches);
            for (i, p) in segments.iter().enumerate() {
                for q in &segments[i + 1..] {
                    if !segments_cross(p, q) {
                        continue;
                    }
                    if last {
                        let (s, r) = crossing_params(p, q);
                        let tp = p.ta + (p.tb - p.ta) * s;
                        let tq = q.ta + (q.tb - q.ta) * r;
                        inserts.push((p.edge, tp));
                        inserts.push((q.edge, tq));
                        pinches.push([(p.edge, tp), (q.edge, tq)]);
                    } else {
                        inserts.push((p.edge, 0.5 * (p.ta + p.tb)));
                        inserts.push((q.edge, 0.5 * (q.ta + q.tb)));
                    }
                }
            }
        }
        if inserts.is_empty() {
            break;
        }
        for (edge, t) in inserts {
            insert_sorted(&mut params[edge], t);
        }
    }

    let mut points = Points::default();
    let mut pinned: HashMap<(usize, u64), u32> = HashMap::new();
    for [(ea, ta), (eb, tb)] in &pinches {
        let a = brep.curves[brep.edges[*ea].curve].point(*ta);
        let b = brep.curves[brep.edges[*eb].curve].point(*tb);
        let id = points.add(0.5 * (a + b));
        pinned.insert((*ea, ta.to_bits()), id);
        pinned.insert((*eb, tb.to_bits()), id);
    }
    let mut edges = Vec::with_capacity(brep.edges.len());
    for (ei, (e, ts)) in brep.edges.iter().zip(params).enumerate() {
        let curve = &brep.curves[e.curve];
        let n = ts.len();
        let ids = ts
            .iter()
            .enumerate()
            .map(|(k, &t)| {
                if let Some(&id) = pinned.get(&(ei, t.to_bits())) {
                    return id;
                }
                let p = if k == 0 {
                    brep.vertices[e.vertices[0]]
                } else if k + 1 == n {
                    brep.vertices[e.vertices[1]]
                } else {
                    curve.point(t)
                };
                points.add(p)
            })
            .collect();
        edges.push(EdgeSamples { params: ts, ids });
    }

    let mut triangles = Vec::new();
    let mut face_of_triangle = Vec::new();
    for (fi, face) in brep.faces.iter().enumerate() {
        match triangulate_face(
            brep,
            face,
            scales[fi],
            &trees[fi],
            &edges,
            &pinches,
            &mut points,
        ) {
            Ok(tris) => {
                face_of_triangle.extend(std::iter::repeat_n(fi, tris.len()));
                triangles.extend(tris);
            }
            Err(reason) => notes.push(format!("GENPRI 面 {fi} 三角化失败，跳过：{reason}")),
        }
    }
    if triangles.is_empty() {
        return Err(GenpriError::Tessellation("一张面也没剖出来".to_string()));
    }
    orient_triangles(&mut triangles, &points.points);
    Ok(Some(TriangleMesh {
        points: points.points,
        triangles,
        face_of_triangle,
    }))
}

/// B-rep 的真实外廓：顶点 + 每条边每个节点区间 16 点。
fn extent(brep: &Brep) -> (DVec3, DVec3) {
    let mut lo = DVec3::INFINITY;
    let mut hi = DVec3::NEG_INFINITY;
    for v in &brep.vertices {
        lo = lo.min(*v);
        hi = hi.max(*v);
    }
    for e in &brep.edges {
        let curve = &brep.curves[e.curve];
        for t in split_by_spans(&curve.knots, e.range[0], e.range[1], |_, _| 16) {
            let p = curve.point(t);
            lo = lo.min(p);
            hi = hi.max(p);
        }
    }
    (lo, hi)
}

/// 一张面：边界取各边的采样（点号共享，故水密），内部补细分树叶子的角点，UV 里约束 Delaunay 剖分，
/// 只留修剪区域内的三角形；三角形在 UV 里逆时针（法向 = ∂u × ∂v）。
fn triangulate_face(
    brep: &Brep,
    face: &Face,
    scale: DVec2,
    tree: &FaceTree,
    edges: &[EdgeSamples],
    pinches: &[Pinch],
    points: &mut Points,
) -> Result<Vec<[u32; 3]>, String> {
    let surface = &brep.surfaces[face.surface];
    let [u0, u1, v0, v1] = face.uv_box;
    let params: Vec<Vec<f64>> = edges.iter().map(|e| e.params.clone()).collect();
    let (rings, _) = face_rings(brep, face, scale, &params, pinches);
    if rings.is_empty() {
        return Err("没有边界环".to_string());
    }

    let mut cdt = ConstrainedDelaunayTriangulation::<Point2<f64>>::new();
    let mut id_of_vertex: Vec<Option<u32>> = Vec::new();
    let mut uv_of_vertex: Vec<DVec2> = Vec::new();
    let mut insert =
        |cdt: &mut ConstrainedDelaunayTriangulation<Point2<f64>>, uv: DVec2, id: Option<u32>| {
            let handle = cdt
                .insert(Point2::new(uv.x, uv.y))
                .map_err(|e| format!("UV 点 {uv} 插不进去：{e:?}"))?;
            let index = handle.index();
            if index >= id_of_vertex.len() {
                id_of_vertex.resize(index + 1, None);
                uv_of_vertex.resize(index + 1, DVec2::ZERO);
            }
            if id_of_vertex[index].is_none() {
                id_of_vertex[index] = id;
                uv_of_vertex[index] = uv;
            }
            Ok::<_, String>(handle)
        };
    let mut missing_constraints = 0usize;
    for ring in &rings {
        let handles = ring
            .iter()
            .map(|&(uv, edge, k, _)| insert(&mut cdt, uv, Some(edges[edge].ids[k])))
            .collect::<Result<Vec<_>, _>>()?;
        for k in 0..handles.len() {
            let (a, b) = (handles[k], handles[(k + 1) % handles.len()]);
            if a != b && cdt.try_add_constraint(a, b).is_empty() && !cdt.exists_constraint(a, b) {
                missing_constraints += 1;
            }
        }
    }

    let polygons: Vec<Vec<DVec2>> = rings
        .iter()
        .map(|r| r.iter().map(|&(uv, _, _, _)| uv).collect())
        .collect();
    let span = (DVec2::new(u1 - u0, v1 - v0) * scale).length().max(1e-9);
    let margin = 1e-6 * span;
    let mut corners: Vec<DVec2> = tree
        .leaves
        .iter()
        .flat_map(|l| {
            [
                DVec2::new(l[0], l[2]),
                DVec2::new(l[1], l[2]),
                DVec2::new(l[0], l[3]),
                DVec2::new(l[1], l[3]),
            ]
        })
        .collect();
    corners.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    corners.dedup();
    for corner in corners {
        let uv = corner * scale;
        if inside(&polygons, uv) && distance_to_rings(&polygons, uv) > margin {
            insert(&mut cdt, uv, None)?;
        }
    }

    // 哪些三角形在修剪区域里：从凸包外往里漫水，每跨一条约束边翻一次。不用「重心是否在环内」：
    // 边界上共线的采样点之间 CDT 会生出贴边的细三角形，重心离边界只有舍入误差那么远。
    // 约束没加全时奇偶不可信，退回重心判据。
    let mut inside_face: Vec<Option<bool>> = vec![None; cdt.num_all_faces()];
    if missing_constraints == 0 {
        let mut queue = std::collections::VecDeque::new();
        for f in cdt.inner_faces() {
            for e in f.adjacent_edges() {
                if e.rev().face().is_outer() && inside_face[f.fix().index()].is_none() {
                    inside_face[f.fix().index()] = Some(e.is_constraint_edge());
                    queue.push_back(f.fix());
                }
            }
        }
        while let Some(fixed) = queue.pop_front() {
            let state = inside_face[fixed.index()].unwrap_or(false);
            for e in cdt.face(fixed).adjacent_edges() {
                if let Some(next) = e.rev().face().as_inner()
                    && inside_face[next.fix().index()].is_none()
                {
                    inside_face[next.fix().index()] = Some(state ^ e.is_constraint_edge());
                    queue.push_back(next.fix());
                }
            }
        }
    }

    let mut tris = Vec::new();
    for f in cdt.inner_faces() {
        let vs = f.vertices();
        let keep = inside_face[f.fix().index()].unwrap_or_else(|| {
            let centroid = vs
                .iter()
                .map(|v| DVec2::new(v.position().x, v.position().y))
                .sum::<DVec2>()
                / 3.0;
            inside(&polygons, centroid)
        });
        if !keep {
            continue;
        }
        let mut tri = [0u32; 3];
        for (slot, v) in tri.iter_mut().zip(vs.iter()) {
            let index = v.fix().index();
            *slot = match id_of_vertex.get(index).copied().flatten() {
                Some(id) => id,
                None => {
                    let uv = uv_of_vertex[index] / scale;
                    let id = points.add(surface.point(uv.x, uv.y));
                    id_of_vertex[index] = Some(id);
                    id
                }
            };
        }
        if tri[0] != tri[1] && tri[1] != tri[2] && tri[0] != tri[2] {
            tris.push(tri);
        }
    }
    if missing_constraints > 0 && tris.is_empty() {
        return Err(format!(
            "{missing_constraints} 条边界约束加不上，剖不出三角形"
        ));
    }
    Ok(tris)
}

/// 统一绕向：沿「恰好两用」的边在三角形之间传播（共享边两侧方向必须相反），再按连通分量的有向
/// 体积翻正。按三角形而不按面传播：修剪环在捏合点两侧夹出来的那一小块绕向与同面其余部分相反。
fn orient_triangles(triangles: &mut [[u32; 3]], points: &[DVec3]) {
    let mut uses: HashMap<(u32, u32), Vec<(usize, bool)>> = HashMap::new();
    for (ti, t) in triangles.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            uses.entry((a.min(b), a.max(b)))
                .or_default()
                .push((ti, a < b));
        }
    }
    let mut adjacency: Vec<Vec<(usize, bool)>> = vec![Vec::new(); triangles.len()];
    for pair in uses.values() {
        if let [(t1, d1), (t2, d2)] = pair[..]
            && t1 != t2
        {
            adjacency[t1].push((t2, d1 == d2));
            adjacency[t2].push((t1, d1 == d2));
        }
    }
    let mut flip = vec![false; triangles.len()];
    let mut seen = vec![false; triangles.len()];
    for seed in 0..triangles.len() {
        if seen[seed] {
            continue;
        }
        seen[seed] = true;
        let mut component = vec![seed];
        let mut queue = VecDeque::from([seed]);
        while let Some(cur) = queue.pop_front() {
            for &(next, same_dir) in &adjacency[cur] {
                if !seen[next] {
                    seen[next] = true;
                    flip[next] = flip[cur] ^ same_dir;
                    component.push(next);
                    queue.push_back(next);
                }
            }
        }
        let mut volume6 = 0.0;
        for &ti in &component {
            let t = triangles[ti];
            let (a, b, c) = (
                points[t[0] as usize],
                points[t[1] as usize],
                points[t[2] as usize],
            );
            let v = a.cross(b).dot(c);
            volume6 += if flip[ti] { -v } else { v };
        }
        if volume6 < 0.0 {
            for &ti in &component {
                flip[ti] = !flip[ti];
            }
        }
    }
    for (t, f) in triangles.iter_mut().zip(flip) {
        if f {
            t.swap(1, 2);
        }
    }
}

/// 偶奇规则：落在奇数个环里即在修剪区域内（外环 + 孔）。
fn inside(rings: &[Vec<DVec2>], p: DVec2) -> bool {
    let mut odd = false;
    for ring in rings {
        for k in 0..ring.len() {
            let (a, b) = (ring[k], ring[(k + 1) % ring.len()]);
            if (a.y > p.y) != (b.y > p.y) {
                let x = a.x + (p.y - a.y) / (b.y - a.y) * (b.x - a.x);
                if p.x < x {
                    odd = !odd;
                }
            }
        }
    }
    odd
}

fn distance_to_rings(rings: &[Vec<DVec2>], p: DVec2) -> f64 {
    let mut best = f64::INFINITY;
    for ring in rings {
        for k in 0..ring.len() {
            let (a, b) = (ring[k], ring[(k + 1) % ring.len()]);
            let ab = b - a;
            let len2 = ab.length_squared();
            let t = if len2 == 0.0 {
                0.0
            } else {
                ((p - a).dot(ab) / len2).clamp(0.0, 1.0)
            };
            best = best.min(p.distance(a + ab * t));
        }
    }
    best
}

/// 三角网格 → [`Manifold`]（只收被三角形引用的点；非流形照实报错）。
pub fn mesh_solid(mesh: &TriangleMesh) -> Result<Manifold, GenpriError> {
    let mut compact: HashMap<u32, u64> = HashMap::new();
    let mut props: Vec<f64> = Vec::new();
    let mut indices: Vec<u64> = Vec::with_capacity(mesh.triangles.len() * 3);
    for t in &mesh.triangles {
        for &id in t {
            let next = compact.len() as u64;
            let index = *compact.entry(id).or_insert_with(|| {
                let p = mesh.points[id as usize];
                props.extend_from_slice(&[p.x, p.y, p.z]);
                next
            });
            indices.push(index);
        }
    }
    let solid = Manifold::from_mesh_f64(&props, 3, &indices)
        .map_err(|e| GenpriError::NotManifold(e.to_string()))?;
    if solid.is_empty() || solid.num_tri() == 0 {
        return Err(GenpriError::NotManifold("生成的流形为空".to_string()));
    }
    Ok(solid)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// aps250162 `/MEI-JCAN01` 名下四只 GENPRI 的整块 `ELDATA`（`tests/fixtures/genpri/`，十六进制字）。
    fn fixture(name: &str) -> Vec<u32> {
        let text = match name {
            "genpri2" => {
                include_str!("../tests/fixtures/genpri/aps250162-2013286706-1042-genpri2.eldata")
            }
            "genpri6" => {
                include_str!("../tests/fixtures/genpri/aps250162-2013286706-1078-genpri6.eldata")
            }
            "genpri10" => {
                include_str!("../tests/fixtures/genpri/aps250162-2013286706-1092-genpri10.eldata")
            }
            "genpri13" => {
                include_str!("../tests/fixtures/genpri/aps250162-2013286706-1103-genpri13.eldata")
            }
            // aps250212 地形 `GENPRI 1 of TRNCEL 6 of TRNCC 1 of TRNMOD /TerrainA/DEF01/MOD01`（归档版本 4）。
            "terrain6" => {
                include_str!("../tests/fixtures/genpri/aps250212-2013286756-1844-terrain-cell6.eldata")
            }
            other => panic!("no fixture {other}"),
        };
        text.split_whitespace()
            .map(|w| u32::from_str_radix(w, 16).unwrap())
            .collect()
    }

    fn mesh_of(name: &str) -> TriangleMesh {
        let brep = decode(&fixture(name)).unwrap();
        let mut notes = Vec::new();
        let mesh = tessellate(&brep, &mut notes)
            .unwrap()
            .expect("big enough to tessellate");
        assert!(notes.is_empty(), "{name}: {notes:?}");
        mesh
    }

    /// 被三角形引用的点的包围盒 `[x0, y0, z0, x1, y1, z1]`。
    fn mesh_box(mesh: &TriangleMesh) -> [f64; 6] {
        let mut lo = DVec3::INFINITY;
        let mut hi = DVec3::NEG_INFINITY;
        for t in &mesh.triangles {
            for &id in t {
                lo = lo.min(mesh.points[id as usize]);
                hi = hi.max(mesh.points[id as usize]);
            }
        }
        [lo.x, lo.y, lo.z, hi.x, hi.y, hi.z]
    }

    /// E3D TTY 导出（`aps_survey_lv6_2013286706_1037.rvm`）里同一件面片的包围盒，取到 0.01 mm。
    fn assert_box(got: [f64; 6], e3d: [f64; 6], tolerance: f64, what: &str) {
        for (axis, (g, e)) in got.iter().zip(e3d).enumerate() {
            assert!(
                (g - e).abs() <= tolerance,
                "{what}: 包围盒第 {axis} 分量 {g:.3}，E3D {e:.2}（全部 {got:?}）"
            );
        }
    }

    /// 控制点全在一张平面上（有理曲面的控制点也在，因为是齐次坐标除回去比的）。
    fn is_planar(surface: &BSplineSurface) -> bool {
        let points: Vec<DVec3> = surface
            .control
            .iter()
            .map(|c| DVec3::new(c[0], c[1], c[2]) / c[3])
            .collect();
        let lo = points.iter().fold(DVec3::INFINITY, |m, p| m.min(*p));
        let hi = points.iter().fold(DVec3::NEG_INFINITY, |m, p| m.max(*p));
        let scale = (hi - lo).length().max(1.0);
        let origin = points[0];
        let mut normal = DVec3::ZERO;
        for a in &points[1..] {
            for b in &points[1..] {
                let n = (*a - origin).cross(*b - origin);
                if n.length() > normal.length() {
                    normal = n;
                }
            }
        }
        if normal.length() <= 1e-12 * scale * scale {
            return true;
        }
        let normal = normal.normalize();
        points
            .iter()
            .all(|p| (*p - origin).dot(normal).abs() <= 1e-9 * scale)
    }

    #[test]
    fn decodes_the_cylinder_brep_word_for_word() {
        let brep = decode(&fixture("genpri6")).unwrap();
        assert_eq!(brep.faces.len(), 3);
        assert_eq!(brep.loops.len(), 3);
        assert_eq!(brep.uses.len(), 6);
        assert_eq!(brep.edges.len(), 3);
        assert_eq!(brep.vertices.len(), 2);
        assert_eq!(
            (brep.pcurves.len(), brep.curves.len(), brep.surfaces.len()),
            (6, 3, 3)
        );
        assert!(
            (brep.tolerance - 0.03377).abs() < 1e-5,
            "{}",
            brep.tolerance
        );
        let side = &brep.surfaces[brep.faces[0].surface];
        assert_eq!(
            (side.degree, side.rational, side.count),
            ([2, 1], true, [9, 2])
        );
        assert!(
            brep.faces[1..]
                .iter()
                .all(|f| is_planar(&brep.surfaces[f.surface]))
        );
        assert!(!is_planar(side));
        // 两个顶点是接缝线的两端，圆柱轴沿 X、长 80。
        let [a, b] = [brep.vertices[0], brep.vertices[1]];
        assert!(
            (b - a - DVec3::new(80.0, 0.0, 0.0)).length() < 1e-9,
            "{a} {b}"
        );
    }

    #[test]
    fn trim_curves_share_the_edge_parameter() {
        let brep = decode(&fixture("genpri6")).unwrap();
        for (fi, face) in brep.faces.iter().enumerate() {
            let surface = &brep.surfaces[face.surface];
            for l in &brep.loops[face.loops.clone()] {
                for u in &brep.uses[l.uses.clone()] {
                    let edge = &brep.edges[u.edge];
                    let curve = &brep.curves[edge.curve];
                    for k in 0..=8 {
                        let t = edge.range[0] + (edge.range[1] - edge.range[0]) * k as f64 / 8.0;
                        let uv = brep.pcurves[u.pcurve].uv(t);
                        let gap = surface.point(uv.x, uv.y).distance(curve.point(t));
                        assert!(gap < 1e-6, "face {fi} edge {} t {t}: {gap}", u.edge);
                    }
                }
            }
        }
    }

    #[test]
    fn cylinder_closes_with_the_e3d_box() {
        let mesh = mesh_of("genpri6");
        assert!(mesh_solid(&mesh).is_ok());
        assert_box(
            mesh_box(&mesh),
            [257.75, 60.17, 242.85, 337.75, 66.07, 248.75],
            0.06,
            "GENPRI 6",
        );
    }

    /// 短管：两端是同心两圈八边形之间的环面。圆柱面沿 v = 0 的采样点共线，CDT 会在边界外侧生出
    /// 贴边的细三角形——按重心判内外会把它们留下、边被用三次，按约束边奇偶漫水才对。
    #[test]
    fn pipe_with_collinear_boundary_samples_closes() {
        let mesh = mesh_of("genpri2");
        assert!(mesh_solid(&mesh).is_ok());
        assert_box(
            mesh_box(&mesh),
            [257.00, 64.19, 155.93, 338.50, 150.81, 219.07],
            0.06,
            "GENPRI 2",
        );
    }

    /// 端面上两段圆弧交叉 0.003 mm：不捏合就留洞、绕向冲突。
    #[test]
    fn crossing_trim_arcs_pinch_into_a_closed_solid() {
        let mesh = mesh_of("genpri13");
        assert!(mesh_solid(&mesh).is_ok());
        assert_box(
            mesh_box(&mesh),
            [259.75, 54.80, 214.26, 335.75, 83.82, 254.11],
            0.5,
            "GENPRI 13",
        );
    }

    /// 2 mm 厚的板边上一段 R5 半圆：E3D 的细分树对这条窄面不分，二分在 6 mm 弦高下也一步走完，最高点
    /// 只到半圆两端（y 119.28），不是半圆顶（119.61）。
    #[test]
    fn narrow_half_circle_strip_stays_as_coarse_as_e3d() {
        let mesh = mesh_of("genpri10");
        assert!(mesh_solid(&mesh).is_ok());
        assert_box(
            mesh_box(&mesh),
            [335.75, 55.03, 184.79, 337.75, 119.28, 253.62],
            0.06,
            "GENPRI 10",
        );
    }

    /// 圆心在原点、半径 r 的有理二次圆弧：从 +X 起逆时针 `quarters` 个四分之一圆，参数 [0, 1] 均分。
    fn arc(r: f64, quarters: usize) -> BSplineCurve {
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let mut control = vec![[r, 0.0, 0.0, 1.0]];
        for k in 0..quarters {
            let (s, c) = (k as f64 * std::f64::consts::FRAC_PI_2).sin_cos();
            control.push([r * (c - s) * w, r * (s + c) * w, 0.0, w]);
            control.push([-r * s, r * c, 0.0, 1.0]);
        }
        let mut knots = vec![0.0; 3];
        for k in 1..quarters {
            let t = k as f64 / quarters as f64;
            knots.extend([t, t]);
        }
        knots.extend([1.0; 3]);
        BSplineCurve {
            degree: 2,
            rational: true,
            dim: 3,
            control,
            knots,
        }
    }

    #[test]
    fn curve_derivative_matches_finite_differences() {
        for curve in [arc(25.0, 1), arc(5.0, 2)] {
            for t in [0.0f64, 0.3, 0.5, 0.9, 1.0] {
                let h = 1e-7;
                let (a, b) = ((t - h).max(0.0), (t + h).min(1.0));
                let numeric = (curve.point(b) - curve.point(a)) / (b - a);
                let analytic = curve.derivative(t);
                assert!(
                    (analytic - numeric).length() < 1e-4 * numeric.length(),
                    "t {t}: {analytic} vs {numeric}"
                );
            }
        }
    }

    /// 导出容差的两个锚：R5 半圆一步接受（Hermite 弦高 4.7 ≤ 6），R25 的四分之一圆整段 8.3 被拒、
    /// 半段 2.25 接受，拆成两步——E3D 对这两种弧的分段正是这样。
    #[test]
    fn bisection_takes_the_e3d_steps_on_circular_arcs() {
        assert_eq!(
            bisection_params(&arc(5.0, 2), 0.0, 1.0, 6.0),
            vec![0.0, 1.0]
        );
        assert_eq!(
            bisection_params(&arc(25.0, 1), 0.0, 1.0, 6.0),
            vec![0.0, 0.5, 1.0]
        );
    }

    #[test]
    fn rejects_a_truncated_or_foreign_blob() {
        let mut words = fixture("genpri6");
        words.truncate(500);
        assert!(matches!(
            decode(&words),
            Err(GenpriError::Layout { at: 1, .. })
        ));
        let mut words = fixture("genpri6");
        words[0] = 4;
        assert!(matches!(
            decode(&words),
            Err(GenpriError::Layout { at: 0, .. })
        ));
        let mut words = fixture("genpri6");
        words[6] = 2;
        assert!(matches!(
            decode(&words),
            Err(GenpriError::Layout { at: 6, .. })
        ));
    }

    #[test]
    fn tiny_breps_are_skipped_and_long_thin_ones_tighten_the_chord() {
        assert_eq!(
            Tessellation::for_extent(DVec3::new(0.5, 0.9, 19.0), 6.0),
            None
        );
        let thin = Tessellation::for_extent(DVec3::new(3.0, 3.0, 400.0), 6.0).unwrap();
        assert_eq!(thin.chord, 2.0);
        let plain = Tessellation::for_extent(DVec3::new(90.0, 85.0, 48.0), 6.0).unwrap();
        assert_eq!(plain.chord, 6.0);
        assert!((plain.max_edge - 0.4 * DVec3::new(90.0, 85.0, 48.0).length()).abs() < 1e-12);
    }

    /// 归档版本 4：aps250212 地形格 6 的缓存面片。E3D TTY 导出（`aps250212_lv6_2013286756_1588.rvm`）里同一件是 103 个多边形
    /// （41 个三角、56 个四边、两个六边、八 / 九 / 32 / 45 边各一个）、141 个不同的点，包围盒 [−4000, 10000, −4818.31] ..
    /// [0, 125000, 13500]；解出来的面片逐环相同。它是张带 33 条边界边的高程面，不是体；旧的 B-rep 解码器对它照例拒读。
    #[test]
    fn version_4_terrain_facets_decode_into_e3d_polygons() {
        let words = fixture("terrain6");
        assert!(matches!(
            decode(&words),
            Err(GenpriError::Layout { at: 0, .. })
        ));
        let Archive::Facets(facets) = decode_archive(&words).unwrap() else {
            panic!("版本 4 应解成缓存面片");
        };
        assert_eq!(facets.vertices.len(), 141);
        assert_eq!(facets.faces.len(), 103);
        assert!(facets.faces.iter().all(|face| face.len() == 1), "格 6 没有带孔的面");

        let polygons = facet_polygons(&facets);
        assert_eq!(polygons.len(), 103);
        let mut sizes: HashMap<usize, usize> = HashMap::new();
        for polygon in &polygons {
            assert_eq!(polygon.len(), 1);
            *sizes.entry(polygon[0].len()).or_insert(0) += 1;
        }
        let mut sizes: Vec<(usize, usize)> = sizes.into_iter().collect();
        sizes.sort_unstable();
        assert_eq!(
            sizes,
            [(3, 41), (4, 56), (6, 2), (8, 1), (9, 1), (32, 1), (45, 1)]
        );
        let mut lo = DVec3::INFINITY;
        let mut hi = DVec3::NEG_INFINITY;
        for p in polygons.iter().flatten().flatten() {
            lo = lo.min(*p);
            hi = hi.max(*p);
        }
        assert_box(
            [lo.x, lo.y, lo.z, hi.x, hi.y, hi.z],
            [-4000.0, 10000.0, -4818.31, 0.0, 125000.0, 13500.0],
            0.01,
            "terrain6",
        );
        // 边界边（只被一张面用到）：高程面的外沿一圈。
        let mut uses: HashMap<(u32, u32), usize> = HashMap::new();
        for ring in facets.faces.iter().flatten() {
            for k in 0..ring.len() {
                let (a, b) = (ring[k], ring[(k + 1) % ring.len()]);
                *uses.entry((a.min(b), a.max(b))).or_insert(0) += 1;
            }
        }
        assert_eq!(uses.values().filter(|&&n| n == 1).count(), 33);
    }

    /// 一张面的几个环怎么分组（E3D 导出时的取舍）：与最大环反向的是孔，挂在包着它的外环上；同向的——哪怕在外环里、
    /// 哪怕在另一张平面上——都是自己的多边形。
    #[test]
    fn rings_group_into_outer_rings_and_their_holes_the_way_e3d_splits_them() {
        let square = |x0: f64, y0: f64, x1: f64, y1: f64, z: f64| {
            vec![
                DVec3::new(x0, y0, z),
                DVec3::new(x1, y0, z),
                DVec3::new(x1, y1, z),
                DVec3::new(x0, y1, z),
            ]
        };
        let mut points = Vec::new();
        let mut ring = |pts: Vec<DVec3>| -> Vec<u32> {
            let first = points.len() as u32;
            points.extend(pts);
            (first..first + 4).collect()
        };
        let outer = ring(square(0.0, 0.0, 100.0, 100.0, 0.0));
        let mut reversed = square(20.0, 20.0, 80.0, 80.0, 0.0);
        reversed.reverse();
        let hole = ring(reversed);
        let island = ring(square(40.0, 40.0, 60.0, 60.0, 0.0));
        let beside = ring(square(200.0, 0.0, 300.0, 100.0, 0.0));
        let lifted = ring(square(20.0, 20.0, 80.0, 80.0, 50.0));
        let face = vec![
            lifted.clone(),
            hole.clone(),
            beside.clone(),
            island.clone(),
            outer.clone(),
        ];
        let groups = group_rings(&face, &points);
        assert_eq!(groups.len(), 4, "{groups:?}");
        assert!(groups.contains(&vec![outer.clone(), hole.clone()]), "{groups:?}");
        assert!(groups.contains(&vec![beside.clone()]));
        assert!(groups.contains(&vec![island.clone()]));
        assert!(groups.contains(&vec![lifted.clone()]));

        // 单环的面原样交出；两个共面互不嵌套的环各自成组。
        assert_eq!(group_rings(&[outer.clone()], &points), vec![vec![outer.clone()]]);
        assert_eq!(
            group_rings(&[outer.clone(), beside.clone()], &points).len(),
            2
        );
    }
}

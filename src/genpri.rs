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
/// 每个节点区间最多切这么多段：坏数据（零长节点、NaN）不至于把三角化拖进死循环。
const MAX_SEGMENTS_PER_SPAN: usize = 64;
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

    /// 控制点全在一张平面上（有理曲面的控制点也在，因为是齐次坐标除回去比的）。
    fn is_planar(&self) -> bool {
        let points: Vec<DVec3> = self
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
    let brep = decode(&words)?;
    let Some(mesh) = tessellate(&brep, notes)? else {
        return Ok(GenpriBuild::NoShape(
            "B-rep 包围盒次长边 < 1 mm 且最长边 < 20 mm，E3D IwTess 前就放弃（GeoIwBrepImpl::Tessellate）"
                .to_string(),
        ));
    };
    Ok(GenpriBuild::Solid(mesh_solid(&mesh)?))
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

/// 一条线在 [a, b] 上等分成几段才够：每段弦长 ≤ `max_edge`、弦中点到线的偏差 ≤ `chord`
/// （`chord` 为 `None` 不看弦高）。`line(k, t)` 给出第 k 条要量的线在 t 处的点。
fn segments_needed(
    a: f64,
    b: f64,
    lines: usize,
    line: &dyn Fn(usize, f64) -> DVec3,
    chord: Option<f64>,
    max_edge: f64,
) -> usize {
    let mut n = 1;
    while n < MAX_SEGMENTS_PER_SPAN {
        let fits = (0..lines).all(|k| {
            (0..n).all(|s| {
                let t0 = a + (b - a) * s as f64 / n as f64;
                let t1 = a + (b - a) * (s + 1) as f64 / n as f64;
                let (p0, p1) = (line(k, t0), line(k, t1));
                if p0.distance(p1) > max_edge {
                    return false;
                }
                chord.is_none_or(|chord| {
                    let mid = line(k, 0.5 * (t0 + t1));
                    distance_to_segment(mid, p0, p1) <= chord
                })
            })
        });
        if fits {
            break;
        }
        n *= 2;
    }
    n
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

/// 一张面的网格线（两向参数，含 UV 范围两端）。
struct Grid {
    lines: [Vec<f64>; 2],
}

fn face_grid(face: &Face, surface: &BSplineSurface, tess: &Tessellation) -> Grid {
    let [u0, u1, v0, v1] = face.uv_box;
    let planar = surface.is_planar();
    let ranges = [(u0, u1), (v0, v1)];
    let mut lines: [Vec<f64>; 2] = [Vec::new(), Vec::new()];
    for dir in 0..2 {
        let (lo, hi) = ranges[dir];
        let (olo, ohi) = ranges[1 - dir];
        // 另一向在哪几处量：断点与各区间中点。
        let mut across = vec![olo];
        across.extend(interior_knots(&surface.knots[1 - dir], olo, ohi));
        across.push(ohi);
        let mids: Vec<f64> = across.windows(2).map(|w| 0.5 * (w[0] + w[1])).collect();
        across.extend(mids);
        let point = |k: usize, t: f64| {
            let w = across[k];
            if dir == 0 {
                surface.point(t, w)
            } else {
                surface.point(w, t)
            }
        };
        let chord = (!planar).then_some(tess.chord);
        lines[dir] = split_by_spans(&surface.knots[dir], lo, hi, |a, b| {
            segments_needed(a, b, across.len(), &point, chord, tess.max_edge)
        });
    }
    Grid { lines }
}

/// 参数曲线在 [t0, t1] 上穿过网格线（第 `axis` 个分量 = `values` 里某值）的参数。
/// 沿网格线走的那一段不算穿过。
fn grid_crossings(
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
/// 顺序与 `IwTess` 同形：先定各面的网格线，再定边的采样（自身弦高 / 边长 + 相邻面网格线的穿越点），
/// 修剪环在 UV 里自交就把相交的那几段二分重来（粗采样的窄槽两岸会交叉），最后逐面剖分。
/// 边的采样两侧面共用，故水密。
pub fn tessellate(
    brep: &Brep,
    notes: &mut Vec<String>,
) -> Result<Option<TriangleMesh>, GenpriError> {
    let (lo, hi) = extent(brep);
    let Some(tess) = Tessellation::for_extent(hi - lo, E3D_EXPORT_CHORD_MM) else {
        return Ok(None);
    };
    let grids: Vec<Grid> = brep
        .faces
        .iter()
        .map(|f| face_grid(f, &brep.surfaces[f.surface], &tess))
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

    let mut params: Vec<Vec<f64>> = Vec::with_capacity(brep.edges.len());
    for (ei, e) in brep.edges.iter().enumerate() {
        let curve = &brep.curves[e.curve];
        let [t0, t1] = e.range;
        let mut ts = split_by_spans(&curve.knots, t0, t1, |a, b| {
            segments_needed(
                a,
                b,
                1,
                &|_, t| curve.point(t),
                Some(tess.chord),
                tess.max_edge,
            )
        });
        for &ui in &uses_of_edge[ei] {
            let fi = face_of_use[ui];
            if fi == usize::MAX {
                continue;
            }
            let pc = &brep.pcurves[brep.uses[ui].pcurve];
            for (axis, values) in grids[fi].lines.iter().enumerate() {
                grid_crossings(pc, t0, t1, axis, values, &mut ts);
            }
        }
        ts.sort_by(f64::total_cmp);
        let eps = 1e-10 * (t1 - t0).abs().max(1e-300);
        ts.dedup_by(|b, a| (*b - *a).abs() <= eps);
        if let Some(first) = ts.first_mut() {
            *first = t0;
        }
        if let Some(last) = ts.last_mut() {
            *last = t1;
        }
        params.push(ts);
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
            &grids[fi],
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

/// 一张面：边界取各边的采样（点号共享，故水密），内部补网格点，UV 里约束 Delaunay 剖分，
/// 只留修剪区域内的三角形；三角形在 UV 里逆时针（法向 = ∂u × ∂v）。
fn triangulate_face(
    brep: &Brep,
    face: &Face,
    scale: DVec2,
    grid: &Grid,
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
    for &u in &grid.lines[0] {
        for &v in &grid.lines[1] {
            let uv = DVec2::new(u, v) * scale;
            if inside(&polygons, uv) && distance_to_rings(&polygons, uv) > margin {
                insert(&mut cdt, uv, None)?;
            }
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

    /// aps250162 `/MEI-JCAN01` 名下三只 GENPRI 的整块 `ELDATA`（`tests/fixtures/genpri/`，十六进制字）。
    fn fixture(name: &str) -> Vec<u32> {
        let text = match name {
            "genpri2" => {
                include_str!("../tests/fixtures/genpri/aps250162-2013286706-1042-genpri2.eldata")
            }
            "genpri6" => {
                include_str!("../tests/fixtures/genpri/aps250162-2013286706-1078-genpri6.eldata")
            }
            "genpri13" => {
                include_str!("../tests/fixtures/genpri/aps250162-2013286706-1103-genpri13.eldata")
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
                .all(|f| brep.surfaces[f.surface].is_planar())
        );
        assert!(!side.is_planar());
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
}

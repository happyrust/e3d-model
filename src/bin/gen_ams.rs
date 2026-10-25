//! ams 库直读生成探针：开库 → 找根 → 生成 → OBJ + 不变量 + 总账落盘。
//!
//! 用法（路径全部显式给，不猜安装位置）：
//!
//! ```text
//! gen_ams --attlib <attlib.dat> --db <ams1112_0001> --template <模板文件或目录> \
//!         [--db-type DESI] [--sesno N] [--root w0/w1] [--out out/ams1112] \
//!         [--catalogue <ams5052_0001> ...] [--project-dir <库目录> ...] \
//!         [--route-only] [--no-instance-cache] [--no-foreign-projects]
//! ```
//!
//! `--catalogue` 可重复，把 `SPRE`/`CATR` 指向的目录库一并开进同一个 [`DbSet`]。
//! 不给也能跑，只是隐式管身的 P 点全落在 `ppoint_db_not_open` 那本账上。
//!
//! `--project-dir` 可重复：扫这些目录（只扫本层）建「库号 → 文件」表，引用跳到
//! 没开过的库时按需自动开，模板按库类型从 `--template` 的目录派生——一个目录顶
//! 一串 `--catalogue`。两者可并用：`add_db` 先登记的文件不会被 resolver 顶掉。
//! 目录仍然是显式给的；唯一的例外是 `--db` 所在项目 SYS 库登记的外项目库（AMS 的
//! MDB `/ALL` 挂着 ZDJ 的 db 7600）：给定目录解析不出时按 E3D 的 `<代号>000` 约定找到那个项目、
//! 只补登记的库号，逐条打印（[`DirectoryDbResolver::adopt_foreign_dbs`]）。`--no-foreign-projects` 关掉。
//!
//! 产物：
//! - `<out>/model.obj`：全部实体，一个 OBJ object 一个元素（`refno noun name`）。
//! - `<out>/elements.json`：逐元素不变量（体积/AABB/三角数/亏格/负体数）——
//!   RVM 对拍就吃这个。
//! - `<out>/report.json`：管线总账（skipped/failed/catalog/unsupported/unknown）。

use std::collections::BTreeMap;
use std::io::Write as _;
use std::path::{Path, PathBuf};

use anyhow::{Context, bail};
use e3d_io::db_element::{DbFilePin, DbSet, template_file_for};
use e3d_io::engine::{ReadOnlyEngine, ScanTier};
use e3d_io::refno::RefNo;
use e3d_model::catalogue::CatalogueMeshCache;
use e3d_model::db_discovery::DirectoryDbResolver;
use e3d_model::mesh::TriMesh;
use e3d_model::pipeline::{GenerationOutcome, Report, generate_subtree_with_cache};
use e3d_model::udet::UdetCatalog;

struct Args {
    attlib: PathBuf,
    db_file: PathBuf,
    template: PathBuf,
    db_type: String,
    sesno: Option<u32>,
    root: Option<RefNo>,
    out: PathBuf,
    catalogues: Vec<PathBuf>,
    project_dirs: Vec<PathBuf>,
    route_only: bool,
    no_instance_cache: bool,
    /// 给了就额外写一份 `<out>/generated.rvm`，见 [`e3d_model::rvm_write`]。
    rvm: bool,
    no_foreign_projects: bool,
}

fn parse_args() -> anyhow::Result<Args> {
    let mut attlib = None;
    let mut db_file = None;
    let mut template = None;
    let mut db_type = "DESI".to_string();
    let mut sesno = None;
    let mut root = None;
    let mut out = PathBuf::from("out/gen_ams");
    let mut catalogues = Vec::new();
    let mut project_dirs = Vec::new();
    let mut route_only = false;
    let mut no_instance_cache = false;
    let mut rvm = false;
    let mut no_foreign_projects = false;

    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--attlib" => attlib = Some(PathBuf::from(value()?)),
            "--db" => db_file = Some(PathBuf::from(value()?)),
            "--template" => template = Some(PathBuf::from(value()?)),
            "--db-type" => db_type = value()?,
            "--sesno" => sesno = Some(value()?.parse().context("--sesno 要无符号整数")?),
            "--root" => root = Some(parse_refno(&value()?)?),
            "--out" => out = PathBuf::from(value()?),
            "--catalogue" => catalogues.push(PathBuf::from(value()?)),
            "--project-dir" => project_dirs.push(PathBuf::from(value()?)),
            "--route-only" => route_only = true,
            "--no-instance-cache" => no_instance_cache = true,
            "--rvm" => rvm = true,
            "--no-foreign-projects" => no_foreign_projects = true,
            other => bail!("不认识的参数 {other}"),
        }
    }
    Ok(Args {
        attlib: attlib.context("缺 --attlib")?,
        db_file: db_file.context("缺 --db")?,
        template: template.context("缺 --template")?,
        db_type,
        sesno,
        root,
        out,
        catalogues,
        project_dirs,
        route_only,
        no_instance_cache,
        rvm,
        no_foreign_projects,
    })
}

/// 解析图元自校的容差（mm）：**只管 f32 落盘那一档**。两侧都是 f64 算到底，
/// 只差 RVM 那一次 f32 落盘，坐标量级 ~2e4 时是 ~2e-3 mm；给 0.01 留一个数量级。
///
/// 网格与解析面之间的离散化缺口不在这个数里——它按每块网格自己的段数算，见
/// [`e3d_model::rvm_write::chord_sagitta_mm`]。这条线 2026-09-06 之前是拿 0.01 mm
/// 一把尺量到底的，于是 `1RCV0101` 的 14 根真圆柱（8/16/28 段，弦高 0.06–0.35 mm）
/// 全被当成「约定写错了」退回面片：几何没错，只是文件大了，圆柱也没写成圆柱。
const ANALYTIC_TOL_MM: f64 = 0.01;

/// 一把网格识别器。`rvm_component_geometry` 按 [`RECOGNISER_ORDER`] 逐个试。
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Recogniser {
    Box,
    Cylinder,
    Snout,
    CircularTorus,
    /// **只由出身挑中**（目录 `SRTO`、设计 `RTOR`），不进 [`RECOGNISER_ORDER`]：矩形轮廓的
    /// `SREV` / 设计 `REVO` 回转出来的网格与矩形环一模一样，而 E3D 对那两族写的是烘好的
    /// `FacetGroup`——按长相轮到它，就把那些面片写成了 `RectangularTorus`。
    RectangularTorus,
    Pyramid,
    Sphere,
    SphericalDish,
    EllipticalDish,
}

/// 没有出身可依时的兜底次序。**次序有讲究**：等径不偏心的圆台让给圆柱、同形不偏心的
/// 棱台让给盒子，顶点恰好共球的正棱柱先被圆柱接走，球的识别器再靠内切比把剩下的多面体
/// 挡在外面；折边碟排在球形碟后面——球形碟的顶点全在冠球上，先被那一条认走。
/// `RectangularTorus` 不在这里，见它自己那条注释。
const RECOGNISER_ORDER: [Recogniser; 8] = [
    Recogniser::Box,
    Recogniser::Cylinder,
    Recogniser::Snout,
    Recogniser::CircularTorus,
    Recogniser::Pyramid,
    Recogniser::Sphere,
    Recogniser::SphericalDish,
    Recogniser::EllipticalDish,
];

/// 目录图元 → 该用哪把识别器。**记录种类由出身定，不由网格长相定。**
///
/// E3D 的 RVM 驱动按目录图元挑写出器（`LPYR`→DRPYR、`LSNO`/`SCON`→DRSNO、`SBOX`→DRBOX、
/// `SCYL`/`LCYL`→DRCYL、`SCTO`→DRCTO、`SSPH`→DRSPH），一只 PAAX/PBAX 退化成正方盒的
/// `LPYR` 照样写 `Pyramid`、一只等径不偏心的 `LSNO` 照样写 `Snout`。我们的识别器不知来历，
/// 先中先得就把它们写成了 `Box` 与 `Cylinder`——几何 exact、种类却不一致，80 根严尺子上
/// 是 302 件（300 只阀门的手轮座 + 2 只等径 LSNO）。
///
/// 名单只列**一个目录图元对一种记录**的那几族。`SDSH` 不在里面：`PRAD > 0` 才是折边碟
/// （见 [`e3d_model::elmodl::CataloguePartRecord::knuckled_dish`]），`PRAD ≤ 0` 是球形碟，
/// `PHEI = 0` 退化成圆盘。一个 noun 对不上一把识别器。折边那一支在调用处单独挑。
///
/// 挑中的那把**先试**，认不出来照旧走兜底次序——自校闸一道没少。`SRTO` 那把是例外中的
/// 例外：它**只有**出身这一条路上场（见 [`Recogniser::RectangularTorus`]），兜底次序里没有它。
fn recogniser_for_catalogue_noun(noun: &str) -> Option<Recogniser> {
    match noun {
        "SBOX" => Some(Recogniser::Box),
        "SCYL" | "LCYL" => Some(Recogniser::Cylinder),
        // TUBI 目录的图元（扁圆风管的隐式管身逐图元交出来，`route::TubeSection::Composite`）：E3D 照 DRBOX / DRCYL 写。
        "BOXI" => Some(Recogniser::Box),
        "TUBE" => Some(Recogniser::Cylinder),
        "LSNO" | "SCON" => Some(Recogniser::Snout),
        "SCTO" => Some(Recogniser::CircularTorus),
        // E3D 的 DRRTO：`/TT-0301` 那一族两只 180° 的半环各写一条 `RectangularTorus`
        // （inner 45 / outer 55 / height 25 / angle π），我们此前只能交面片——19 件。
        "SRTO" => Some(Recogniser::RectangularTorus),
        // `LPYR` 是目录图元；`SREC` 是型材截面的矩形成员，挤出来是一根棱柱，E3D 的
        // `DB_Sctn` 逐成员走 DRPYR 写它——**同样是看名词，不看长相**：`/CS001` 的截面也是
        // 矩形，只因为写成 `SPRO` + 4 个 `SPVE`，E3D 出的就是一条 `FacetGroup`。
        "LPYR" | "SREC" => Some(Recogniser::Pyramid),
        "SSPH" => Some(Recogniser::Sphere),
        _ => None,
    }
}

/// 设计图元 → 该用哪把识别器。**名单上只有 `RTOR`。**
///
/// 设计图元没有目录出身，一直是按长相认的；80 根严尺子（表现级 0）上 CYLI / BOX / DISH /
/// PYRA / CTOR / CONE 全部对得上——这些族按长相认与 E3D 按名词写同解，不必动。`RTOR` 是
/// 唯一的例外：它那把识别器**只由出身挑中**（见 [`Recogniser::RectangularTorus`]），兜底
/// 次序里没有它，不给出身就永远是面片；而 E3D 的 DRRTO 对 RTOR 写的正是 `RectangularTorus`。
///
/// 表现级 0 的基准里一件 RTOR 都没有：全 sweep 168 只 RTOR 的 LEVE 全是 [2,4]，两侧都不画，
/// 所以此前不在账上。取证是 `/盘梯底部梯子`（21909/51286）在表现级 3 的基准：两只 RTOR
/// 各一条 `RectangularTorus`，我们原本交的是面片。
///
/// `SNOU` / `CONE` / `PYRA` 的退化情形（等径圆台、同形棱台）按名词写才与 E3D 一致，但 80 根
/// 里一件没有，没有取证就不接；`ARTOR` 那一族同理。
fn recogniser_for_design_noun(noun: &str) -> Option<Recogniser> {
    match noun {
        "RTOR" => Some(Recogniser::RectangularTorus),
        _ => None,
    }
}

/// 这只目录图元 E3D 只写 `FacetGroup`，按长相的识别器一把都不许上场。
///
/// E3D 的拉伸写出器对目录 `SEXT` 一律写 `FacetGroup`（`DRWEXT`，记录种类常数 11）：截面是
/// 矩形不改走 `DRBOX`（`/1RX07-LCT` 的 20 只 90° 槽弯，网格是盒子），是整圆也不改走 `DRCYL`
/// （ams8000 竖梯 FTUB `/ACP1000-RFVL-R3B-500` 的圆板，每只 22 块，E3D TTY RVM 里全是面片）。
/// `SBOX` / `SCYL` 不走这里：它们由 [`recogniser_for_catalogue_noun`] 按出身挑中。出身不明时照旧按长相认。
fn catalogue_noun_writes_facets_only(catalogue_noun: Option<&str>) -> bool {
    catalogue_noun == Some("SEXT")
}

/// `export holes off` 时桥架 / 托盘的负体不进 RVM 记录。C-IY `FTUBE 1`（`24384/22405`）E3D 的
/// 三条 Box 就是未扣减的 SBOX：`1500×100×50`、`1500×103×50`、`1500×103×100`。
/// BOARD 那只网格被负体切成 28 个三角、体积只剩 4.5e5 mm³，包围盒仍是 1500×103×50。
/// 槽弯的 FacetGroup 也没有孔轮廓。NOZZ 相反：两个 SCYL 扣膛之后 E3D 仍写**一条**面片。
/// 所以只在可见块全是 `SBOX` / `SEXT` / `SREV` 时按 `parts` 拆开写，其余带负体的仍整只一条。
/// `export holes on` 不走这里（见 [`e3d_model::catalogue::export_holes_on`]）。
fn write_uncut_catalogue_parts(record_level_cuts: usize, nouns: &[String]) -> bool {
    record_level_cuts > 0
        && !nouns.is_empty()
        && nouns
            .iter()
            .all(|noun| matches!(noun.as_str(), "SBOX" | "SEXT" | "SREV"))
}

/// 小角度槽弯多出来的那一块。12 只 4° 弯我们有 4 个组件、E3D 只有 3 条 FacetGroup；
/// 多出来的最小边是 0.0004–0.26 mm，不到其余块中位最小边的一成。90° 弯 9 块的最小边
/// 都 ≥ 48.5 mm，一条都不扔。绝对 1 mm 会把整只已经缩到 0.03 mm 的弯（BEND 18）一起扔掉。
fn is_record_splinter(min_edge_mm: f64, sibling_min_edges_mm: &[f64]) -> bool {
    if !(min_edge_mm < 1.0) || sibling_min_edges_mm.is_empty() {
        return false;
    }
    let mut siblings = sibling_min_edges_mm.to_vec();
    siblings.sort_by(|a, b| a.total_cmp(b));
    let mid = siblings[siblings.len() / 2];
    min_edge_mm < mid * 0.1
}

fn mesh_min_edge_mm(vertices: &[[f64; 3]]) -> f64 {
    let Some(first) = vertices.first() else {
        return 0.0;
    };
    let mut lo = *first;
    let mut hi = *first;
    for vertex in vertices {
        for axis in 0..3 {
            lo[axis] = lo[axis].min(vertex[axis]);
            hi[axis] = hi[axis].max(vertex[axis]);
        }
    }
    (0..3)
        .map(|axis| hi[axis] - lo[axis])
        .fold(f64::MAX, f64::min)
}

/// `by_provenance` = 这把识别器是**出身**挑的，不是按长相轮到的。
///
/// 差别只在两处退化：按长相认时，长得像盒子的棱台让给 `Box`、两端等径的圆台让给
/// `Cylinder`（长成那样的多半**就是** SBOX / SCYL）；按出身认时不让——E3D 的 DRPYR /
/// DRSNO 不看退化成什么，LPYR 一律 `Pyramid`、LSNO 一律 `Snout`。
///
/// `frame` = 出身给的放置标架（元素局部系），只有目录图元有。有它时圆柱 / 圆台 / 四棱台 / 球 /
/// 圆环记录的矩阵照它写（Z 列 = 目录轴含符号、X/Y = ORIMAT 副轴、圆台底顶按 PBDM/PTDM），
/// 不照网格反推——E3D 的 DRCYL / DRSNO / DRPYR / DRCTO 写的就是这只标架（happyrust/e3d-model#1）。
/// 四棱台在目录 Z 与识别轴不共线、但上下同形不偏心时，把三条边长换到标架的 X/Y/Z 上
/// （`/1WCC096VN` 的 120 mm 高沿目录 X，不沿网格 −Z）。圆环的起扫 X 跟标架走：轴反号时起点
/// 换到另一头，只翻 Z 会扫到补弧。对不上就退回网格轴。尺寸仍由网格说了算，自校闸照过。
fn analytic_candidate(
    recogniser: Recogniser,
    world: &glam::DMat4,
    local_vertices: &[[f64; 3]],
    triangles: &[[u32; 3]],
    by_provenance: bool,
    frame: Option<&glam::DMat4>,
) -> Option<e3d_model::rvm_write::AnalyticCandidate> {
    use e3d_model::rvm_write as write;
    match recogniser {
        Recogniser::Box => {
            let (lo, hi) = aabb_of_points(local_vertices.iter().copied())?;
            let centre = [0, 1, 2].map(|axis| (lo[axis] + hi[axis]) / 2.0);
            let lengths = write::box_from_local_mesh(local_vertices, triangles).or_else(|| {
                // 出身是 SBOX 时，切过的网格不再是 12 个三角。E3D 写的仍是未扣减的盒，
                // 而这只盒子的包围盒还在（负体没削掉角）。按长相认时不走这条，免得把开孔盒写成实心盒。
                if !by_provenance {
                    return None;
                }
                let lengths = [0, 1, 2].map(|axis| hi[axis] - lo[axis]);
                lengths.iter().all(|len| *len > 0.0).then_some(lengths)
            })?;
            Some(write::box_geometry(world, lengths, centre))
        }
        Recogniser::Cylinder => {
            let cylinder = write::cylinder_from_local_mesh(local_vertices, triangles)?;
            Some(match frame {
                Some(frame) => write::cylinder_geometry_in_frame(world, &cylinder, frame),
                None => write::cylinder_geometry(world, &cylinder),
            })
        }
        Recogniser::Snout => {
            let snout = if by_provenance {
                write::snout_from_local_mesh_including_cylinders(local_vertices, triangles)?
            } else {
                write::snout_from_local_mesh(local_vertices, triangles)?
            };
            Some(match frame {
                Some(frame) => write::snout_geometry_in_frame(world, &snout, frame),
                None => write::snout_geometry(world, &snout),
            })
        }
        Recogniser::CircularTorus => {
            let torus = write::torus_from_local_mesh(local_vertices, triangles)?;
            Some(match frame {
                Some(frame) => write::circular_torus_geometry_in_frame(world, &torus, frame),
                None => write::circular_torus_geometry(world, &torus),
            })
        }
        Recogniser::RectangularTorus => {
            let torus = write::rect_torus_from_local_mesh(local_vertices, triangles)?;
            Some(match frame {
                Some(frame) => write::rectangular_torus_geometry_in_frame(world, &torus, frame),
                None => write::rectangular_torus_geometry(world, &torus),
            })
        }
        Recogniser::Pyramid => {
            let pyramid = if by_provenance {
                write::pyramid_from_local_mesh_including_boxes(local_vertices, triangles)?
            } else {
                write::pyramid_from_local_mesh(local_vertices, triangles)?
            };
            Some(match frame {
                Some(frame) => write::pyramid_geometry_in_frame(world, &pyramid, frame),
                None => write::pyramid_geometry(world, &pyramid),
            })
        }
        Recogniser::Sphere => {
            let sphere = write::sphere_from_local_mesh(local_vertices, triangles)?;
            Some(match frame {
                Some(frame) => write::sphere_geometry_in_frame(world, &sphere, frame),
                None => write::sphere_geometry(world, &sphere),
            })
        }
        Recogniser::SphericalDish => {
            let dish = write::spherical_dish_from_local_mesh(local_vertices, triangles)?;
            Some(write::spherical_dish_geometry(world, &dish))
        }
        // 折边碟写出去的是半椭球——照抄 E3D 导出时就做了的那层近似，好让两边 RVM 的图元
        // 种类对得上；形状损失算进候选自己的 `sagitta_mm`，仍由自校闸按 r/h 算出来的量放行。
        Recogniser::EllipticalDish => {
            // 按长相认时半球留给球形碟。出身已说明是折边碟（`PRAD > 0`）时，r = h 的
            // 半球也写成 `EllipticalDish`——`DRDISH` 只看折边开关。
            let dish = if by_provenance {
                write::elliptical_dish_from_local_mesh_including_hemispheres(
                    local_vertices,
                    triangles,
                )?
            } else {
                write::elliptical_dish_from_local_mesh(local_vertices, triangles)?
            };
            Some(match frame {
                Some(frame) => write::elliptical_dish_geometry_in_frame(world, &dish, frame),
                None => write::elliptical_dish_geometry(world, &dish),
            })
        }
    }
}

/// 一个组件 → 一条 RVM 记录。能认成解析图元就写解析图元，否则写 FacetGroup。
///
/// 有出身就先试出身挑的那把（`preferred`：目录图元走 [`recogniser_for_catalogue_noun`]，
/// 设计图元走 [`recogniser_for_design_noun`]），没有或认不出来再按
/// [`RECOGNISER_ORDER`] 逐个试，先认出来的算。`provenance_only` = 只许出身那一把上场
/// （拉伸体族：E3D 对它们只有「出身是 SREC 就 DRPYR，否则面片」这一条，按长相轮一遍
/// 会把矩形截面写成 `Box`，那正是 64f3f01 挡掉的）。
///
/// **自校闸**：解析候选算出来的世界 AABB 必须与这块网格实算的一致，否则退回面片。
/// 规范体约定写错了照样能写出一份解析得开但几何是错的 RVM，这种错不该靠人眼发现。
/// 「一致」的尺度由候选自己给（[`e3d_model::rvm_write::AnalyticCandidate::matches_mesh_aabb`]）。
#[allow(clippy::too_many_arguments)]
fn rvm_component_geometry(
    world: &glam::DMat4,
    rotation: [f64; 9],
    local_vertices: &[[f64; 3]],
    triangles: &[[u32; 3]],
    world_lo: [f64; 3],
    world_hi: [f64; 3],
    preferred: Option<Recogniser>,
    frame: Option<&glam::DMat4>,
    provenance_only: bool,
    analytic_count: &mut usize,
    rejected_count: &mut usize,
) -> e3d_model::rvm_read::RvmGeometry {
    let candidate = preferred
        .and_then(|recogniser| {
            analytic_candidate(recogniser, world, local_vertices, triangles, true, frame)
        })
        .or_else(|| {
            if provenance_only {
                return None;
            }
            RECOGNISER_ORDER
                .iter()
                .copied()
                .filter(|recogniser| Some(*recogniser) != preferred)
                .find_map(|recogniser| {
                    analytic_candidate(recogniser, world, local_vertices, triangles, false, frame)
                })
        });
    let had_candidate = candidate.is_some();
    let accepted = candidate
        .filter(|candidate| candidate.matches_mesh_aabb(world_lo, world_hi, ANALYTIC_TOL_MM))
        .map(|candidate| candidate.geometry);
    match &accepted {
        Some(_) => *analytic_count += 1,
        None if had_candidate => *rejected_count += 1,
        None => {}
    }
    accepted.unwrap_or_else(|| {
        e3d_model::rvm_write::facet_group_from_world_triangles(
            rotation,
            world.w_axis.truncate().to_array(),
            local_vertices,
            triangles,
        )
    })
}

/// E3D 对这些元素**只有面片写出器**，一律 `FacetGroup`——识别器再认得出盒子也不能写 `Box`。
///
/// **拉伸体族**的依据在 RVM 驱动里：`expdri/DRWEXT`（拉伸体写出器）写的记录种类是常数 11 =
/// FacetGroup（`unk_1000D394`），顶盖、N 张侧面、底盖各一条多边形；没有「截面恰好是矩形就
/// 改写 Box」这一条——盒子走的是另一条 `DRBOX`，只有真图元（设计 BOX / 目录 SBOX）才到得了那里。
/// 80 根严尺子印证（2026-09-14 实测）：名词认得出来的这一族 1 168 件，E3D 侧写的是
/// 1 223 条 `FacetGroup` 与 52 条 `Pyramid`（空心矩形 SCTN 的壁板），**一条 `Box` 都没有**；
/// 我们的网格识别器不知来历，把其中的矩形截面认成 `Box`、棱台形管墩认成 `Pyramid`——
/// 几何 exact、种类却不一致。接上这一条之后严尺子的 kind 不一致从 1 383 件降到 799 件。
///
/// **多面体**（POHE / POLYHE，显式 B-rep）同理：E3D 拿到的是顶点面表，没有图元可写，整只交
/// 面片写出器。取证是表现级 3 的探针（四根，15 只 POHE 各拆 6 块）：E3D 侧 90 条全是
/// `FacetGroup`，我们按长相认成了 30 条 `Box` + 48 条 `Pyramid`（其余 12 块本来就认不出来）。
/// 表现级 0 的 80 根里一只 POHE 都不画（全 sweep 336 只被跳过，LEVE 在 [2,4] / [4,5] / [5,7]），
/// 所以此前不在账上。
///
/// **通用图元**（GENPRI，SMLib B-rep，[`e3d_model::genpri`]）也一样：Core3D 把它交给
/// `GeoTessellationFunnel`，由 SMLib `IwTess` 三角化成面片集再进 libgm。aps250162 的 TTY 导出里
/// 75 只全是 `FacetGroup`；我们分段粗的圆柱会被认成 `Pyramid`。
///
/// **折弯板**（BPANEL，[`e3d_model::bpanel`]）：aps250110 的 52 只全是 `FacetGroup`——没有开孔的一段一条，
/// 有开孔的整只一条；直板那几段按长相会被认成 `Box`。
///
/// **布局式扶手**（HANDRA，[`e3d_model::handrail`]）：Core3D 把栏杆、立柱、踢脚板连同圆角弧合成一棵 CSG 树交给
/// 一次写出，aps250209 / aps250112 / aps250110 的 TTY 导出里每只 HANDRA 都是**一条** `FacetGroup`；我们的零件是
/// `compose` 起来的，圆角弧与直段不共顶点时会被按连通性拆开、环段被认成 `CircularTorus`、立柱认成 `Cylinder`
/// （aps250209 `/A01-AREA05-2ND-FL-HANDRAILS` 的 HANDRA 2 拆成 7 条）。
///
/// 名单 = 设计侧环拉伸（[`e3d_model::category::Category::LoopExtrusion`]：PANE / FLOOR / EXTR /
/// AEXTR / SCREED / GWALL）+ 设计侧回转体（[`e3d_model::category::Category::Revolution`]：REVO，E3D 的 DRWREV 一律面片；
/// 扫角超过 180° 的照 libgm 拆成两块各一条，由 `elmodl` 交 `parts`，半环不许被按长相认成 `RectangularTorus`）+
/// 型材扫掠（SCTN / GENSEC / STWALL / WALL / RAIL / KICKPL）+
/// 多面体（[`e3d_model::category::Category::Polyhedron`]：POHE / POLYHE）+ 通用图元 GENPRI + 折弯板 BPANEL + 布局式扶手 HANDRA
/// + 布局式梯段 STRFLT（[`e3d_model::stair_flight`]：斜梁、踏步、平台板在 aps250209 的 TTY 导出里是一条面片）。
/// 目录构件（FITT / FIXING / SBFI / PCOM…）**不在**名单里：它们由 GMSE 图元拼成，E3D 走
/// DRBOX / DRCYL 等图元写出器——`:SDS_Entity 3` 那块底板就是 E3D 自己写的 `Pyramid`。
///
/// 空心矩形 SCTN（E3D 拆成 4 条 `Pyramid` 壁板）不受影响：本来就是一条面片对 4 条 Pyramid，
/// 那是另一件事（拆板）。
fn e3d_exports_facets_only(noun: &str) -> bool {
    matches!(
        e3d_model::category::classify(noun),
        e3d_model::category::Category::LoopExtrusion
            | e3d_model::category::Category::Revolution
            | e3d_model::category::Category::Polyhedron
    ) || matches!(
        noun,
        "SCTN"
            | "GENSEC"
            | "STWALL"
            | "WALL"
            | "RAIL"
            | "KICKPL"
            | "GENPRI"
            | "BPANEL"
            | "HANDRA"
            | "STRFLT"
    )
}

/// E3D 给 RVM 组用的显示名：命名元素直接是 `/NAME`，无名元素是
/// `<NOUN> <n> of <OWNER 显示名>`。
///
/// noun 这里发的是库里的短名（`FTUB`），E3D 发的是全称（`FTUBE`）。
/// **对拍不受影响**——`rvm_compare` 两侧都做 `noun4()` 截断；人读时差一截尾巴。
/// 工程派生的子类型例外：两侧写的都是完整的 UDET 名（`:SDS_Pole`），截不得。
fn rvm_display_name(
    name: Option<&String>,
    noun: &str,
    owner_desc: Option<&String>,
    ordinal: usize,
) -> String {
    match (name, owner_desc) {
        (Some(name), _) => name.clone(),
        (None, Some(owner)) => format!("{noun} {ordinal} of {owner}"),
        (None, None) => format!("{noun} {ordinal}"),
    }
}

fn parse_refno(text: &str) -> anyhow::Result<RefNo> {
    let trimmed = text.trim_start_matches('=');
    let (w0, w1) = trimmed
        .split_once('/')
        .with_context(|| format!("--root 要 `w0/w1` 形式，得到 {text}"))?;
    Ok(RefNo::new(w0.trim().parse()?, w1.trim().parse()?))
}

/// 索引侧的独立普查：树根 + 元素总数。
///
/// 总数是给 [`Report::accounts_for`] 当判据的。它必须来自索引（`scan_elements`）
/// 而不是管线自己的遍历——同源就验不出「有子树没走到」。
struct IndexCensus {
    roots: Vec<RefNo>,
    total: usize,
}

/// 树根 = owner 指向自己、无效、或不在本库索引里的元素（WORL 语义）。
fn scan_index(db_file: &Path, sesno: Option<u32>) -> anyhow::Result<IndexCensus> {
    let mut engine = match sesno {
        Some(s) => ReadOnlyEngine::open_at(db_file, s),
        None => ReadOnlyEngine::open(db_file),
    }
    .with_context(|| format!("开库扫根 {}", db_file.display()))?;
    let mut owners = Vec::new();
    let mut indexed = std::collections::HashSet::new();
    for scanned in engine.scan_elements(ScanTier::Header)? {
        let scanned = scanned?;
        indexed.insert(scanned.refno);
        owners.push((scanned.refno, scanned.owner));
    }
    let roots = owners
        .iter()
        .filter(|(refno, owner)| owner == refno || !owner.is_valid() || !indexed.contains(owner))
        .map(|(refno, _)| *refno)
        .collect();
    Ok(IndexCensus {
        roots,
        total: indexed.len(),
    })
}

/// 独立扫出本库全部 BRAN/HANG 容器；仍由 [`generate_subtree`] 生成，避免审计探针
/// 另造一条比生产路径更容易“通过”的模型链。
fn scan_route_roots(
    db_file: &Path,
    sesno: Option<u32>,
    set: &std::sync::Arc<DbSet>,
) -> anyhow::Result<Vec<RefNo>> {
    let mut engine = match sesno {
        Some(s) => ReadOnlyEngine::open_at(db_file, s),
        None => ReadOnlyEngine::open(db_file),
    }
    .with_context(|| format!("开库扫 BRAN/HANG {}", db_file.display()))?;
    let mut roots = Vec::new();
    for scanned in engine.scan_elements(ScanTier::Header)? {
        let refno = scanned?.refno;
        if matches!(set.element(refno).element_type()?.as_str(), "BRAN" | "HANG") {
            roots.push(refno);
        }
    }
    Ok(roots)
}

#[derive(serde::Serialize)]
struct ElementInvariants {
    geometry_id: e3d_model::GeometryId,
    refno: String,
    /// 对拍用的名词：库里的短名，工程派生的子类型换成 UDET 名（`:SDS_Pole`），
    /// 与 E3D 写进 RVM 的那个一致。见 [`e3d_model::udet`]。
    noun: String,
    /// Diagnostic route segment shown dashed by Plant UI and intentionally
    /// absent from the E3D solid RVM export.
    is_invalid_tubi: bool,
    name: Option<String>,
    /// 属主的显示名（`NOUN /名`），属主无名时为 `None`。见 [`owner_keys`]。
    /// 管身（`TUBI`）的属主是容器本身，见 [`tube_pair_key`]。
    owner_desc: Option<String>,
    /// 本元素在属主**同 noun 兄弟**里的序号，文件原序 1 起；同 noun 按
    /// [`ElementInvariants::noun`] 那一档算（`:SDS_Plate` 与 `:SDS_Entity`
    /// 同是 `FIXING`，但各数各的）。管身：容器内建成管身的路由顺序，1 起。
    ordinal: usize,
    volume_mm3: f64,
    aabb_min: [f64; 3],
    aabb_max: [f64; 3],
    /// 元素局部系 AABB。RVM facet 必须先乘 `world^-1` 再与它比较；
    /// 旋转元素不能拿世界 AABB 的 X/Y 轴直接对拍。
    local_aabb_min: [f64; 3],
    local_aabb_max: [f64; 3],
    /// 元素局部系到世界系，列主序 `[c0,c1,c2,t]`。
    world: [[f64; 3]; 4],
    triangles: usize,
    genus: i32,
    /// 布尔运算后的连通体明细。只看总 AABB 会把主体和远端碎片混在一起。
    components: Vec<ComponentInvariants>,
    negatives_applied: usize,
    negatives_skipped: usize,
}

#[derive(serde::Serialize)]
struct ComponentInvariants {
    volume_mm3: f64,
    local_aabb_min: [f64; 3],
    local_aabb_max: [f64; 3],
    triangles: usize,
    genus: i32,
}

#[derive(serde::Serialize)]
struct RouteRootAudit {
    refno: String,
    noun: String,
    name: Option<String>,
    direct_members: usize,
    direct_member_nouns: BTreeMap<String, usize>,
    direct_route_members: usize,
    /// Route members whose SPRE resolves to a catalogue/specification element.
    direct_route_members_with_spre: usize,
    /// Route members whose SPRE is unset/nulref. E3D can legitimately export
    /// zero geometry for a HANG made only from such placeholder PCLA members.
    direct_route_members_without_spre: usize,
    generated_elements: usize,
    generated_real_elements: usize,
    generated_implied_tubes: usize,
    skipped: usize,
    failed: usize,
}

/// 对拍主键：属主显示名 + 同 noun 兄弟序号。
///
/// E3D 给无名元素的显示名是「NOUN n of <属主显示名>」，RVM 导出物里的
/// `owner_desc` / `ordinal` 正是这一对（实例：`GWALL 1 of CWALL /1RS-WF03-W-C-RR001`）。
/// 生成侧的元素大多无名，refno 又不进 RVM，所以这是两侧唯一对得上的身份。
///
/// 一次走完每个属主的成员表就把整组序号发完；逐元素回头数在 PANE 密集的 CWALL 上
/// 是几百万步的重复遍历。
///
/// 属主无名时必须递归复现 E3D 的默认显示名。例如 7997 的设备基本体位于无名
/// SUBE 下，RVM 键是 `CYLINDER 18 of SUBEQUIPMENT 20 of EQUIPMENT /X`；只看
/// 最近的命名祖先会把 20 个 SUBE 的同序号基本体错配在一起。
///
/// 名词一律走 [`UdetCatalog::display_noun`]：工程派生的子类型 E3D 写的是 UDET 名
/// （`:SDS_Pole`），序号也按 UDET 名数。见 [`e3d_model::udet`]。
fn owner_display_desc(
    set: &std::sync::Arc<DbSet>,
    udets: &UdetCatalog,
    refno: RefNo,
    cache: &mut std::collections::HashMap<RefNo, Option<String>>,
    visiting: &mut std::collections::HashSet<RefNo>,
) -> anyhow::Result<Option<String>> {
    if let Some(desc) = cache.get(&refno) {
        return Ok(desc.clone());
    }
    if !visiting.insert(refno) {
        bail!("owner 显示名链成环: {refno}");
    }

    let element = set.element(refno);
    let noun = udets.display_noun(&element)?;
    let desc = match element.stored_name()? {
        Some(name) => Some(format!("{noun} {name}")),
        None => match element.owner()? {
            None => None,
            Some(parent) => {
                match owner_display_desc(set, udets, parent.refno(), cache, visiting)? {
                    None => None,
                    Some(parent_desc) => {
                        let mut ordinal = 0usize;
                        let mut found = false;
                        for sibling in parent.members()? {
                            if udets.display_noun(&sibling)? == noun {
                                ordinal += 1;
                            }
                            if sibling.refno() == refno {
                                found = true;
                                break;
                            }
                        }
                        if !found {
                            bail!("{refno} 不在属主 {} 的成员表里", parent.refno());
                        }
                        Some(format!("{noun} {ordinal} of {parent_desc}"))
                    }
                }
            }
        },
    };
    visiting.remove(&refno);
    cache.insert(refno, desc.clone());
    Ok(desc)
}

fn owner_keys(
    set: &std::sync::Arc<DbSet>,
    udets: &UdetCatalog,
    owners: &std::collections::HashSet<RefNo>,
    desc_cache: &mut std::collections::HashMap<RefNo, Option<String>>,
) -> anyhow::Result<std::collections::HashMap<RefNo, (Option<String>, usize)>> {
    let mut keys = std::collections::HashMap::new();
    for owner_ref in owners {
        let owner = set.element(*owner_ref);
        let desc = owner_display_desc(
            set,
            udets,
            *owner_ref,
            desc_cache,
            &mut std::collections::HashSet::new(),
        )?;
        let mut seen: BTreeMap<String, usize> = BTreeMap::new();
        for member in owner.members()? {
            let noun = udets.display_noun(&member)?;
            let ordinal = seen.entry(noun).or_insert(0);
            *ordinal += 1;
            keys.insert(member.refno(), (desc.clone(), *ordinal));
        }
    }
    Ok(keys)
}

/// 管身的对拍主键：**属主 = 容器本身**（`BRAN /名` 或无名时的递归显示名），
/// **序号 = 该容器内建成管身的路由顺序**，1 起连续。
///
/// 管身没有库记录，`GeneratedElement::refno` 填的是容器；拿它去查 [`owner_keys`]
/// 得到的是「容器在其属主里的序号」（`PIPE /1RX-345` + 支管的兄弟序），一条支管的
/// 全部管身同键，`rvm_compare` 当场以「生成侧主键重复」退出（2026-09-01 记）。
/// E3D 把管身导成独立容器 `TUBE n of BRANCH /X`（`rvm_verify import` 记成 noun `TUBI`），
/// n 是**画出来的**管身沿路由的顺序；本函数按 [`e3d_model::pipeline::build_route_tubes`]
/// 的推入序——即路由槽位顺序——发号，零长 / 未建成的槽位不占号，与之同一口径。
/// 2026-09-02 用 ams7999 `/1RCV0101-60.3-VMB-R23-R342/B1`（`test_data/rvm/1RCV0101-B1.rvm`）
/// 核过：E3D `TUBE 1..3` 与我们的 `TUBI 1..3` 逐根配对且局部 AABB 差 ≤ 0.24 mm。
fn tube_pair_key(
    set: &std::sync::Arc<DbSet>,
    udets: &UdetCatalog,
    container: RefNo,
    desc_cache: &mut std::collections::HashMap<RefNo, Option<String>>,
    tube_ordinals: &mut std::collections::HashMap<RefNo, usize>,
) -> anyhow::Result<(Option<String>, usize)> {
    let desc = owner_display_desc(
        set,
        udets,
        container,
        desc_cache,
        &mut std::collections::HashSet::new(),
    )?;
    let ordinal = tube_ordinals.entry(container).or_insert(0);
    *ordinal += 1;
    Ok((desc, *ordinal))
}

fn main() -> anyhow::Result<()> {
    let args = parse_args()?;
    if args.no_instance_cache {
        // SAFETY: 参数刚解析完、任何工作线程都还没起，此刻进程里只有主线程能碰环境块。
        unsafe { std::env::set_var(e3d_model::primitive_instance::INSTANCE_CACHE_ENV, "1") };
    }
    let t0 = std::time::Instant::now();

    // 模板目录：`--template` 给目录就是它本身，给文件就是它躺的目录。
    // resolver 按库类型派生模板、显式 `--catalogue` 取 CATA 模板，都从这里出。
    let template_dir = if args.template.is_dir() {
        args.template.clone()
    } else {
        args.template
            .parent()
            .unwrap_or(Path::new("."))
            .to_path_buf()
    };
    let template_file = if args.template.is_dir() {
        template_file_for(&args.template, &args.db_type)?
    } else {
        args.template.clone()
    };

    // 池级元素缓存：一次生成是有界的一棵子树，而 owner 链上溯与目录链走查会把同一批
    // 元素反复解析（ams8000 实测每元素 145 遍）。长驻进程另说，见 `with_element_cache`。
    // DICT 库里的 UDET 定义决定 E3D 怎么叫工程派生出来的子类型；对拍主键带名词，
    // 不查这张表整族配不上（见 `e3d_model::udet`）。扫到哪些 DICT 库只有 resolver
    // 知道，所以在它被搬进 DbSet 之前先把库号记下来。
    let (set, dict_dbnos) = if args.project_dirs.is_empty() {
        (
            std::sync::Arc::new(DbSet::with_attlib_file(&args.attlib)?.with_element_cache()),
            Vec::new(),
        )
    } else {
        let mut resolver = DirectoryDbResolver::scan(&args.project_dirs, &template_dir)?;
        if !args.no_foreign_projects
            && let Some(main_dir) = args.db_file.parent()
        {
            for note in resolver.adopt_foreign_dbs(main_dir, &args.attlib, &template_dir) {
                println!("project-dir {note}");
            }
        }
        for skipped in resolver.skipped() {
            println!(
                "project-dir 诊断 {}: {}",
                skipped.file.display(),
                skipped.reason
            );
        }
        println!("{}", resolver.summary());
        let dict_dbnos = resolver.dbnos_of_type("DICT");
        // UDA 字典与 gen-model 生产侧一样挂上：目录表达式里按键读 UDA 的那一类
        // （风管 H 族头管的 `( :HXYsize[n] )`）不挂就求不出值，对拍量不到生产的行为。
        let attlib =
            std::sync::Arc::new(e3d_attlib::AttlibData::parse_file(&args.attlib).map_err(
                |error| anyhow::anyhow!("parse attlib {}: {error}", args.attlib.display()),
            )?);
        let (uda, unreadable) = resolver.uda_catalog(&dict_dbnos, &template_dir, &attlib);
        for (dbno, file, error) in &unreadable {
            println!(
                "DICT 库 {dbno}（{}）的 UDA 定义读不出：{error}",
                file.display()
            );
        }
        if !dict_dbnos.is_empty() {
            println!(
                "DICT 库 {dict_dbnos:?} 里读到 {} 条 UDA 定义",
                uda.definition_count()
            );
        }
        (
            std::sync::Arc::new(
                DbSet::with_resolver(attlib, Box::new(resolver))
                    .with_uda_catalog(std::sync::Arc::new(uda))
                    .with_element_cache(),
            ),
            dict_dbnos,
        )
    };
    let dbno = set.add_db(DbFilePin {
        file: args.db_file.clone(),
        template: template_file,
        db_type: Some(args.db_type.clone()),
        sesno: args.sesno,
    })?;
    let pinned = set.pinned_sesno(dbno);
    println!(
        "库 {} 打开为 dbno={dbno}，钉在会话 {pinned:?}",
        args.db_file.display()
    );

    // 目录库跟设计库一个池子：P 点在 SPRE→SCOM→PTSE 链的另一头，跨库。
    // 没给 --catalogue 就别去找 CATA 模板，免得模板目录里没有它时白报一个错。
    let catalogue_template = if args.catalogues.is_empty() {
        PathBuf::new()
    } else {
        template_file_for(&template_dir, "CATA")?
    };
    for file in &args.catalogues {
        let cat_dbno = set.add_db(DbFilePin {
            file: file.clone(),
            template: catalogue_template.clone(),
            db_type: Some("CATA".to_string()),
            sesno: None,
        })?;
        println!("目录库 {} 打开为 dbno={cat_dbno}", file.display());
    }

    let udets = UdetCatalog::scan(&set, &dict_dbnos)?;
    if !dict_dbnos.is_empty() {
        println!("DICT 库 {dict_dbnos:?} 里读到 {} 条 UDET 定义", udets.len());
    }

    // 只在从头整库生成时才拿索引总数当判据；`--root` 是子树，对不上是应该的。
    let (roots, index_total) = match args.root {
        Some(refno) => (vec![refno], None),
        None if args.route_only => {
            let roots = scan_route_roots(&args.db_file, args.sesno, &set)?;
            println!("route-only 扫到 {} 个 BRAN/HANG 根", roots.len());
            (roots, None)
        }
        None => {
            let census = scan_index(&args.db_file, args.sesno)?;
            println!(
                "索引 {} 个元素，扫到 {} 个根: {:?}",
                census.total,
                census.roots.len(),
                census.roots
            );
            (census.roots, Some(census.total))
        }
    };
    if roots.is_empty() && !args.route_only {
        bail!("没有可用的根元素（--root 也没给）");
    }

    std::fs::create_dir_all(&args.out)
        .with_context(|| format!("建输出目录 {}", args.out.display()))?;

    let mut merged = Report::default();
    let mut invariants = Vec::new();
    // 生成侧 RVM 的成员表。0 号是根容器（平移全 0，只提供层级），元素全挂在它下面。
    // E3D 会把 SITE/ZONE/PIPE 祖先一并导出来，我们暂时只给一层——`rvm_compare` 的
    // 主键只用成员名，不看祖先层数（见 `Key` / `owner_key`）。
    let mut rvm_members = vec![e3d_model::rvm_write::container_member(
        "/GENERATED",
        "/GENERATED",
        None,
    )];
    // 写成解析图元的件数 / 有可复用基本体但自校没过、退回 FacetGroup 的件数。
    let (mut rvm_analytic, mut rvm_analytic_rejected) = (0usize, 0usize);
    // 拉伸体族（SCTN / FLOOR / PANE…）不进识别器、照 E3D 直接写面片的组件数。
    let mut rvm_extrusion_facets = 0usize;
    // 打洞导出里扣过负体、照 E3D 整只写一条布尔结果面片的元素数。
    let mut rvm_boolean_facets = 0usize;
    let holes_on = e3d_model::catalogue::export_holes_on();
    // 目录图元退化成零尺寸、跟着 E3D 照写的空记录数。
    let mut rvm_zero_size = 0usize;
    // 点标记（SNOD）照 E3D 写成三条线的件数。
    let mut rvm_marks = 0usize;
    // 同一放置在 GMSE 里出现不止一次、跟着 E3D 多写的那几条记录数（体只有一块）。
    let mut rvm_repeated_records = 0usize;
    let obj_path = args.out.join("model.obj");
    let mut obj = std::io::BufWriter::new(
        std::fs::File::create(&obj_path).with_context(|| format!("建 {}", obj_path.display()))?,
    );
    let mut index_offset = 0u32;
    let mut noun_counts: BTreeMap<String, usize> = BTreeMap::new();
    let mut route_audit = Vec::new();
    let mut catalogue_mesh_cache = CatalogueMeshCache::default();
    // 分档计时：生成 / 三角化 / 落盘三段在同一个根循环里交错，只能各自累加。
    // 读取侧（索引查找 + 记录解析）另有全局计数器，取整段的差值。
    let mut gen_nanos = 0u128;
    let mut mesh_nanos = 0u128;
    let mut obj_nanos = 0u128;
    let reads_before = e3d_io::read_stats::snapshot();
    let phases_before = e3d_model::perf::snapshot();
    let loop_start = std::time::Instant::now();

    for root in roots {
        let route_element = args.route_only.then(|| set.element(root));
        let route_identity = route_element
            .as_ref()
            .map(|element| {
                let mut direct_member_nouns = BTreeMap::new();
                let mut direct_route_members = 0;
                let mut direct_route_members_with_spre = 0;
                let mut direct_route_members_without_spre = 0;
                for member in element.members()? {
                    let noun = member.element_type()?;
                    *direct_member_nouns.entry(noun.clone()).or_insert(0) += 1;
                    if matches!(
                        e3d_model::category::decision(&noun).category,
                        e3d_model::category::Category::RouteMember
                    ) {
                        direct_route_members += 1;
                        if member.get_element("SPRE")?.is_some() {
                            direct_route_members_with_spre += 1;
                        } else {
                            direct_route_members_without_spre += 1;
                        }
                    }
                }
                Ok::<_, e3d_io::db_element::DbElementError>((
                    element.element_type()?,
                    element.stored_name()?,
                    direct_member_nouns.values().sum(),
                    direct_member_nouns,
                    direct_route_members,
                    direct_route_members_with_spre,
                    direct_route_members_without_spre,
                ))
            })
            .transpose()?;
        let gen_start = std::time::Instant::now();
        let generated_root =
            generate_subtree_with_cache(set.element(root), &mut catalogue_mesh_cache);
        gen_nanos += gen_start.elapsed().as_nanos();
        let GenerationOutcome {
            elements,
            report,
            marks,
            bodyless,
            ..
        } = generated_root?;
        if let Some((
            noun,
            name,
            direct_members,
            direct_member_nouns,
            direct_route_members,
            direct_route_members_with_spre,
            direct_route_members_without_spre,
        )) = route_identity
        {
            route_audit.push(RouteRootAudit {
                refno: root.to_string(),
                noun,
                name,
                direct_members,
                direct_member_nouns,
                direct_route_members,
                direct_route_members_with_spre,
                direct_route_members_without_spre,
                generated_elements: elements.len(),
                generated_real_elements: elements
                    .iter()
                    .filter(|element| {
                        matches!(element.geometry_id, e3d_model::GeometryId::Element { .. })
                    })
                    .count(),
                generated_implied_tubes: elements
                    .iter()
                    .filter(|element| {
                        matches!(
                            element.geometry_id,
                            e3d_model::GeometryId::ImpliedTube { .. }
                        )
                    })
                    .count(),
                skipped: report.skipped.len(),
                failed: report.failed.len(),
            });
        }
        if args.route_only {
            merged.merge(report);
            continue;
        }
        let owners = elements
            .iter()
            .map(|generated| generated.refno)
            .chain(marks.iter().map(|mark| mark.refno))
            .chain(bodyless.iter().map(|record| record.refno))
            .map(|refno| set.element(refno).owner())
            .collect::<Result<Vec<_>, _>>()?
            .into_iter()
            .flatten()
            .map(|owner| owner.refno())
            .collect();
        let mut desc_cache = std::collections::HashMap::new();
        let keys = owner_keys(&set, &udets, &owners, &mut desc_cache)?;
        // 管身在 `elements` 里按容器的路由槽位顺序推入（`build_route_tubes` 顺序 push、
        // 生成物不重排），逐容器计数就是路由序。
        let mut tube_ordinals = std::collections::HashMap::new();
        for generated in &elements {
            let mesh_start = std::time::Instant::now();
            let mesh = generated.mesh();
            mesh_nanos += mesh_start.elapsed().as_nanos();
            let (lo, hi) = mesh.aabb().unwrap_or(([0.0; 3], [0.0; 3]));
            let inv_world = generated.world.inverse();
            let to_local = |vertex: &[f64; 3]| {
                (inv_world * glam::DVec3::from_array(*vertex).extend(1.0))
                    .truncate()
                    .to_array()
            };
            let local_vertices: Vec<[f64; 3]> = mesh.vertices.iter().map(to_local).collect();
            let (local_lo, local_hi) =
                aabb_of_points(local_vertices.iter().copied()).unwrap_or(([0.0; 3], [0.0; 3]));
            let mut components = generated
                .components()
                .into_iter()
                .map(|component| {
                    let component_mesh = TriMesh::from_manifold(&component);
                    let (component_lo, component_hi) =
                        aabb_of_points(component_mesh.vertices.iter().map(to_local))
                            .unwrap_or(([0.0; 3], [0.0; 3]));
                    ComponentInvariants {
                        volume_mm3: component.volume(),
                        local_aabb_min: component_lo,
                        local_aabb_max: component_hi,
                        triangles: component_mesh.triangle_count(),
                        genus: component.genus(),
                    }
                })
                .collect::<Vec<_>>();
            components.sort_by(|a, b| b.volume_mm3.total_cmp(&a.volume_mm3));
            let (owner_desc, ordinal) = match &generated.geometry_id {
                e3d_model::GeometryId::ImpliedTube { .. } => tube_pair_key(
                    &set,
                    &udets,
                    generated.refno,
                    &mut desc_cache,
                    &mut tube_ordinals,
                )?,
                e3d_model::GeometryId::Element { .. } => {
                    keys.get(&generated.refno).cloned().unwrap_or((None, 0))
                }
            };
            // 对拍身份用 E3D 的名词：工程派生的子类型两侧写的都是 UDET 名。
            // 管身没有库记录（`refno` 填的是容器），名词照旧是 `TUBI`。
            let identity_noun = match &generated.geometry_id {
                e3d_model::GeometryId::Element { .. } => {
                    udets.display_noun(&set.element(generated.refno))?
                }
                e3d_model::GeometryId::ImpliedTube { .. } => generated.noun.clone(),
            };
            if args.rvm && !mesh.indices.is_empty() {
                let world = &generated.world;
                let rotation = [
                    world.x_axis.x,
                    world.x_axis.y,
                    world.x_axis.z,
                    world.y_axis.x,
                    world.y_axis.y,
                    world.y_axis.z,
                    world.z_axis.x,
                    world.z_axis.y,
                    world.z_axis.z,
                ];
                // 角钢 FacetGroup 的记录原点改到目录轮廓原点。偏移是 JUSL 的 (PX, PY)；
                // 为零或不是角钢 SCTN 时不挪。GENSEC / CPAR 的 NA 落在目录原点，这里是 None。
                let angle_shift = match &generated.geometry_id {
                    e3d_model::GeometryId::Element { .. } => {
                        e3d_model::section::angle_facet_catalogue_shift(
                            &set.element(generated.refno),
                        )?
                    }
                    e3d_model::GeometryId::ImpliedTube { .. } => None,
                };
                // **逐组件一条记录**，不是逐元素一条。E3D 就是按目录件拆的
                // （`GeneratedElement::parts` 那条注释说的就是这件事：
                // 「E3D preserves touching catalogue primitives as separate display meshes」），
                // 合成一条会让 `Cylinder×2 → FacetGroup` 这类差异永远消不掉。
                // `components()` 有 `parts` 就用它，没有就按连通性拆——单块实体拆出来还是一块。
                //
                // **扣过负体的整只一条。** `export holes on`（TTY 取证的口径）下 E3D 写布尔结果：
                // NOZZ 的两个 SCYL 扣完膛是一条面片，MDS 关节 IJOINT1 两块开孔板是一条带孔面片，
                // aps7200 `/STAIR-STEEL-PRE-12/BASE` 被自己的两把 NSCY 开出 Ø20 栓孔也是一条带孔面片。
                // `export holes off`（C-IY 那份 addin 导出）时桥架 / 托盘的 SBOX / SEXT / SREV
                // 负体不进记录，每只可见图元各写一条未扣减的（C-IY 三条 Box 尺寸就是 SBOX 自己的
                // 1500×100×50 / 1500×103×50 / 1500×103×100），NGMS 自扣仍写扣过的面片。
                let split_uncut = !holes_on
                    && write_uncut_catalogue_parts(
                        generated
                            .negatives_applied
                            .saturating_sub(generated.own_ngms_cuts),
                        &generated.part_catalogue_nouns,
                    );
                // 打洞导出里扣过负体的件是布尔结果：整只一条面片，不认形状（见 [`e3d_model::catalogue::export_holes_on`]）。
                let boolean_result = holes_on && generated.negatives_applied > 0;
                // 拉伸体族与多面体照 E3D 的写法一律面片，不进识别器（见 [`e3d_exports_facets_only`]）。
                let facets_only = e3d_exports_facets_only(&generated.noun);
                // 没有目录出身的拉伸体不按连通性拆：E3D 的拉伸写出器一只元素写一条面片。MDS
                // `/SUPPORT/FRAMES/BS/FT08/4-BAR-1` 的网格是两个连通分量，E3D 只有一条 FacetGroup。
                // 多面体相反，**逐连通体一条**：aps7200 `/STAIRC.TOP` 的 `POHE 1 / 2` 各是 8 块互不相连的薄板，
                // E3D TTY 导出各 8 条（`aps7200_lv6_23584_8574.rvm`），我们此前合成一条。
                let polyhedron = matches!(
                    e3d_model::category::classify(&generated.noun),
                    e3d_model::category::Category::Polyhedron
                );
                let whole_extrusion =
                    facets_only && generated.part_catalogue_nouns.is_empty() && !polyhedron;
                let component_meshes: Vec<TriMesh> = if (generated.negatives_applied > 0
                    && !split_uncut)
                    || generated.noun == "CABLE"
                    || whole_extrusion
                {
                    vec![mesh.clone()]
                } else {
                    generated
                        .components()
                        .iter()
                        .map(TriMesh::from_manifold)
                        .collect()
                }
                .into_iter()
                .filter(|m| !m.indices.is_empty())
                .collect();
                let part_min_edges: Vec<f64> = if split_uncut {
                    component_meshes
                        .iter()
                        .map(|component| {
                            let local: Vec<[f64; 3]> =
                                component.vertices.iter().map(to_local).collect();
                            mesh_min_edge_mm(&local)
                        })
                        .collect()
                } else {
                    Vec::new()
                };
                // `PrimitiveInstance` / `AnalyticPrimitive` 描述的都是**整个元素**（在元素
                // 局部系里），所以只有单组件时它们才说得准；多组件的逐块自己认几何。
                // 后者是网格识别器认不出的那一族（SLCY → 带斜切角的 `Snout`），只能从属性直出，
                // 自校闸照过。
                let whole_element_instance =
                    (!facets_only && !boolean_result && component_meshes.len() == 1)
                        .then(|| {
                            generated
                                .primitive_instance
                                .as_ref()
                                .and_then(|instance| {
                                    e3d_model::rvm_write::analytic_from_instance(
                                        &generated.world,
                                        instance,
                                    )
                                })
                                .or_else(|| {
                                    generated.analytic_primitive.as_ref().and_then(|primitive| {
                                        e3d_model::rvm_write::analytic_from_primitive(
                                            &generated.world,
                                            primitive,
                                        )
                                    })
                                })
                        })
                        .flatten()
                        .filter(|candidate| candidate.matches_mesh_aabb(lo, hi, ANALYTIC_TOL_MM))
                        .map(|candidate| candidate.geometry);

                // 逐块的目录出身。对不上号（长度不等 = 这几块不是目录图元一块一条，
                // 比如按连通性拆出来的、或上面带负体那一支）就当出身不明，照旧靠长相认。
                let component_nouns: &[String] =
                    if component_meshes.len() == generated.part_catalogue_nouns.len() {
                        &generated.part_catalogue_nouns
                    } else {
                        &[]
                    };
                // 逐块的记录事实（份数 + 放置标架），同一条对齐规矩。E3D 逐 GMSE 成员写记录，
                // 两个成员算出同一只网格、同一个放置时它写两条字节一样的；我们体只放一块，
                // 记录照它的数补；矩阵照出身的标架写
                // （见 [`e3d_model::elmodl::GeneratedElement::part_record_repeats`]）。
                let component_records: &[e3d_model::elmodl::CataloguePartRecord] =
                    if component_meshes.len() == generated.part_record_repeats.len() {
                        &generated.part_record_repeats
                    } else {
                        &[]
                    };
                let repeats_of = |index: usize| {
                    component_records
                        .get(index)
                        .map(|record| record.repeats)
                        .unwrap_or(1)
                        .max(1)
                };
                // 设计图元的出身就是元素自己的名词（[`recogniser_for_design_noun`]）。只在
                // **单块、没扣负体、没有目录出身**时才说得准：多块是按连通性拆出来的，扣过
                // 负体的是布尔结果（E3D 也只能写面片），有目录出身的听目录的。
                // 折边的设计 DISH（`RADI ≠ 0`）写 `EllipticalDish`，见 `elmodl` 里给它记的 `knuckled_dish`。
                let design_recogniser = (component_meshes.len() == 1
                    && generated.negatives_applied == 0
                    && generated.part_catalogue_nouns.is_empty())
                .then(|| {
                    let knuckled = component_records
                        .first()
                        .is_some_and(|record| record.knuckled_dish);
                    if generated.noun == "DISH" && knuckled {
                        Some(Recogniser::EllipticalDish)
                    } else {
                        recogniser_for_design_noun(&generated.noun)
                    }
                })
                .flatten();
                let mut geometries = Vec::with_capacity(component_meshes.len());
                let mut push_repeated = |geometry: e3d_model::rvm_read::RvmGeometry,
                                         repeats: usize| {
                    rvm_repeated_records += repeats - 1;
                    geometries.extend(std::iter::repeat_n(geometry, repeats));
                };
                if let Some(geometry) = whole_element_instance {
                    rvm_analytic += 1;
                    push_repeated(geometry, repeats_of(0));
                } else {
                    for (index, component) in component_meshes.iter().enumerate() {
                        let local: Vec<[f64; 3]> =
                            component.vertices.iter().map(to_local).collect();
                        if split_uncut {
                            let siblings: Vec<f64> = part_min_edges
                                .iter()
                                .enumerate()
                                .filter(|(other, _)| *other != index)
                                .map(|(_, edge)| *edge)
                                .collect();
                            if is_record_splinter(part_min_edges[index], &siblings) {
                                continue;
                            }
                        }
                        let tris: Vec<[u32; 3]> = component.indices.as_chunks::<3>().0.to_vec();
                        // `SDSH` + `PRAD > 0`：种类由折边开关定，不由 r/h。半球（r = h）
                        // 按长相会被球形碟先认走，E3D 写的仍是 `EllipticalDish`。
                        let knuckled_dish = component_records
                            .get(index)
                            .is_some_and(|record| record.knuckled_dish);
                        let preferred = component_nouns
                            .get(index)
                            .and_then(|noun| {
                                if noun == "SDSH" && knuckled_dish {
                                    Some(Recogniser::EllipticalDish)
                                } else {
                                    recogniser_for_catalogue_noun(noun)
                                }
                            })
                            .or(design_recogniser);
                        // 出身的标架：圆柱 / 圆台 / 圆环记录的矩阵照它写（E3D 写的就是目录图元
                        // 自己的标架），见 [`e3d_model::rvm_write::cylinder_geometry_in_frame`]。
                        let frame = component_records
                            .get(index)
                            .and_then(|record| record.frame.as_ref());
                        // 拉伸体族一律面片，**除非这一块的出身另有说法**：SCTN 的 `SREC`
                        // 成员 E3D 走 DRPYR，不进拉伸体写出器（见 `e3d_exports_facets_only`）。
                        // 这一族只认出身挑的那一把，认不出来就是面片——按长相轮一遍正是
                        // 64f3f01 挡掉的那件事（矩形截面写成 `Box`）。
                        // 目录 `SEXT` 同理：矩形、整圆的挤出 E3D 都写 FacetGroup。
                        let catalogue_noun = component_nouns.get(index).map(String::as_str);
                        let provenance_only =
                            facets_only || catalogue_noun_writes_facets_only(catalogue_noun);
                        // 本件 `NGMR`（→ NGMS）自扣过的组件：E3D 写的是**扣过的**面片，
                        // 不是未扣减的图元（aps7200 `/STAIR-STEEL-PRE-12/BASE` 的栓孔板
                        // 就是一条带孔轮廓的 FacetGroup）。记录级负体照旧能按出身写
                        // 未扣减图元（桥架/托盘那族）。
                        let geometry = if boolean_result
                            || (provenance_only && preferred.is_none())
                            || generated.own_ngms_cuts > 0
                        {
                            if boolean_result {
                                rvm_boolean_facets += 1;
                            } else {
                                rvm_extrusion_facets += 1;
                            }
                            if let Some(shift) = angle_shift {
                                e3d_model::rvm_write::facet_group_at_catalogue_origin(
                                    rotation,
                                    world.w_axis.truncate().to_array(),
                                    &local,
                                    &tris,
                                    shift,
                                )
                            } else {
                                e3d_model::rvm_write::facet_group_from_world_triangles(
                                    rotation,
                                    world.w_axis.truncate().to_array(),
                                    &local,
                                    &tris,
                                )
                            }
                        } else {
                            let (clo, chi) = component.aabb().unwrap_or(([0.0; 3], [0.0; 3]));
                            // 网格认不出的目录图元（`SSLC` 的斜切节）照目录参数直出，自校闸照过。
                            let catalogue_analytic = component_records
                                .get(index)
                                .and_then(|record| Some((record.analytic.as_ref()?, record.frame?)))
                                .and_then(|(primitive, frame)| {
                                    e3d_model::rvm_write::analytic_from_primitive(
                                        &(*world * frame),
                                        primitive,
                                    )
                                });
                            match catalogue_analytic {
                                Some(candidate)
                                    if candidate.matches_mesh_aabb(clo, chi, ANALYTIC_TOL_MM) =>
                                {
                                    rvm_analytic += 1;
                                    candidate.geometry
                                }
                                rejected => {
                                    if rejected.is_some() {
                                        rvm_analytic_rejected += 1;
                                    }
                                    rvm_component_geometry(
                                        world,
                                        rotation,
                                        &local,
                                        &tris,
                                        clo,
                                        chi,
                                        preferred,
                                        frame,
                                        provenance_only,
                                        &mut rvm_analytic,
                                        &mut rvm_analytic_rejected,
                                    )
                                }
                            }
                        };
                        // 出身是 SEXT / SREV 且这条最终写成面片：记录帧改目录轮廓标架。
                        // SEXT 原点旋进标架，SREV 原点不旋，两只矩阵都在 CataloguePart.frame 里。
                        // 解析图元和自校没过退回的面片不进这一臂。
                        let geometry = if matches!(catalogue_noun, Some("SEXT" | "SREV"))
                            && matches!(
                                geometry.primitive,
                                e3d_model::rvm_read::Primitive::FacetGroup(_)
                            ) {
                            frame.map_or(geometry, |frame| {
                                e3d_model::rvm_write::facet_group_in_frame(
                                    world, frame, &local, &tris,
                                )
                            })
                        } else {
                            geometry
                        };
                        push_repeated(geometry, repeats_of(index));
                    }
                }
                // 没有体的目录图元：没有网格可认，记录却照写——E3D 就是这么写的
                // （见 [`e3d_model::elmodl::DegenerateCataloguePart`]）。它不过自校闸：
                // 闸门比的是解析候选与网格的 AABB，而这一条两样都没有。
                //
                // 布尔结果除外：E3D 写的是扣完的那一只体，没有体的图元不在里面，也不另写。aps250104
                // 23 只锚固件 ANCI（`/PS-01-00122.1` 等，目录 `/BT23-100` 里 `PDIA = DESP[11] = 0`、长 10 的
                // LCYL）扣过三把负体，E3D 只写一条 FacetGroup；没扣负体的 zone 66457 阀门
                // （`/1WCC216VN` 的零径 LCYL）照写 `Cylinder[0, 120]`。
                let bodyless_parts = if boolean_result {
                    &[][..]
                } else {
                    &generated.degenerate_parts[..]
                };
                for part in bodyless_parts {
                    rvm_zero_size += 1;
                    geometries.push(match part.shape {
                        e3d_model::elmodl::DegenerateCatalogueShape::Cylinder {
                            radius_mm,
                            height_mm,
                        } => e3d_model::rvm_write::bodyless_cylinder_geometry(
                            &generated.world,
                            &part.local_transform,
                            radius_mm,
                            height_mm,
                        ),
                        e3d_model::elmodl::DegenerateCatalogueShape::Snout {
                            bottom_radius_mm,
                            top_radius_mm,
                            height_mm,
                            offset_x_mm,
                        } => e3d_model::rvm_write::bodyless_snout_geometry(
                            &generated.world,
                            &part.local_transform,
                            bottom_radius_mm,
                            top_radius_mm,
                            height_mm,
                            offset_x_mm,
                        ),
                        e3d_model::elmodl::DegenerateCatalogueShape::Pyramid { parameters } => {
                            e3d_model::rvm_write::bodyless_pyramid_geometry(
                                &generated.world,
                                &part.local_transform,
                                parameters,
                            )
                        }
                        e3d_model::elmodl::DegenerateCatalogueShape::Line { length_mm } => {
                            e3d_model::rvm_write::bodyless_line_geometry(
                                &generated.world,
                                &part.local_transform,
                                length_mm,
                            )
                        }
                        e3d_model::elmodl::DegenerateCatalogueShape::Box { lengths_mm } => {
                            e3d_model::rvm_write::bodyless_box_geometry(
                                &generated.world,
                                &part.local_transform,
                                lengths_mm,
                            )
                        }
                        // 开放面片只有设计侧的 GENPRI 走（下面 `bodyless` 那一段），目录零尺寸件没有这一支。
                        e3d_model::elmodl::DegenerateCatalogueShape::OpenFacets { .. } => continue,
                    });
                }
                let name = rvm_display_name(
                    generated.name.as_ref(),
                    &identity_noun,
                    owner_desc.as_ref(),
                    ordinal,
                );
                rvm_members.push(e3d_model::rvm_read::RvmMember {
                    path: format!("/GENERATED/{name}"),
                    name,
                    parent: Some(0),
                    translation_mm: world.w_axis.truncate().to_array(),
                    material: 1,
                    geometries,
                });
            }
            invariants.push(ElementInvariants {
                geometry_id: generated.geometry_id.clone(),
                refno: generated.refno.to_string(),
                noun: identity_noun,
                is_invalid_tubi: generated.is_invalid_tubi,
                name: generated.name.clone(),
                owner_desc,
                ordinal,
                volume_mm3: generated.volume(),
                aabb_min: lo,
                aabb_max: hi,
                local_aabb_min: local_lo,
                local_aabb_max: local_hi,
                world: [
                    generated.world.x_axis.truncate().to_array(),
                    generated.world.y_axis.truncate().to_array(),
                    generated.world.z_axis.truncate().to_array(),
                    generated.world.w_axis.truncate().to_array(),
                ],
                triangles: mesh.triangle_count(),
                genus: generated.genus(),
                components,
                negatives_applied: generated.negatives_applied,
                negatives_skipped: generated.negatives_skipped.len(),
            });
            *noun_counts.entry(generated.noun.clone()).or_insert(0) += 1;
            let label = match &generated.name {
                Some(name) => format!("{} {} {}", generated.geometry_id, generated.noun, name),
                None => format!("{} {}", generated.geometry_id, generated.noun),
            };
            let obj_start = std::time::Instant::now();
            index_offset = mesh.write_obj_object(&mut obj, &label, index_offset)?;
            obj_nanos += obj_start.elapsed().as_nanos();
        }
        // 点标记（SNOD）：不出实体、不进 elements.json，RVM 里照 E3D 写过点的三条线。
        if args.rvm {
            for mark in &marks {
                let (owner_desc, ordinal) = keys.get(&mark.refno).cloned().unwrap_or((None, 0));
                let identity_noun = udets.display_noun(&set.element(mark.refno))?;
                let name = rvm_display_name(
                    mark.name.as_ref(),
                    &identity_noun,
                    owner_desc.as_ref(),
                    ordinal,
                );
                rvm_marks += 1;
                rvm_members.push(e3d_model::rvm_read::RvmMember {
                    path: format!("/GENERATED/{name}"),
                    name,
                    parent: Some(0),
                    translation_mm: mark.origin.to_array(),
                    material: 1,
                    geometries: e3d_model::rvm_write::mark_line_geometries(mark),
                });
            }
            // 设计侧的空记录：不出实体、不进 elements.json，RVM 里照 E3D 写——零尺寸的设计 CYLI
            // （[`e3d_model::elmodl::bodyless_design_record`]）与桥架路径点的 10 mm 记号（[`e3d_model::rpath::route_point_marks`]）。
            // 记录矩阵就是记录自带的世界标架。
            for record in &bodyless {
                let geometry = match record.shape {
                    e3d_model::elmodl::DegenerateCatalogueShape::Cylinder {
                        radius_mm,
                        height_mm,
                    } => e3d_model::rvm_write::bodyless_cylinder_geometry(
                        &record.world,
                        &glam::DMat4::IDENTITY,
                        radius_mm,
                        height_mm,
                    ),
                    e3d_model::elmodl::DegenerateCatalogueShape::Box { lengths_mm } => {
                        e3d_model::rvm_write::bodyless_box_geometry(
                            &record.world,
                            &glam::DMat4::IDENTITY,
                            lengths_mm,
                        )
                    }
                    // 归档版本 4 的 GENPRI（地形格的缓存面片）：是面不是体，照 E3D 写一条 FacetGroup。
                    e3d_model::elmodl::DegenerateCatalogueShape::OpenFacets { ref polygons } => {
                        e3d_model::rvm_write::bodyless_facet_group_geometry(&record.world, polygons)
                    }
                    _ => continue,
                };
                let (owner_desc, ordinal) = keys.get(&record.refno).cloned().unwrap_or((None, 0));
                let identity_noun = udets.display_noun(&set.element(record.refno))?;
                let name = rvm_display_name(
                    record.name.as_ref(),
                    &identity_noun,
                    owner_desc.as_ref(),
                    ordinal,
                );
                rvm_zero_size += 1;
                rvm_members.push(e3d_model::rvm_read::RvmMember {
                    path: format!("/GENERATED/{name}"),
                    name,
                    parent: Some(0),
                    translation_mm: record.world.w_axis.truncate().to_array(),
                    material: 1,
                    geometries: vec![geometry],
                });
            }
        }
        merged.merge(report);
    }
    let loop_nanos = loop_start.elapsed().as_nanos();
    let reads = e3d_io::read_stats::snapshot().delta_from(reads_before);
    let phases = e3d_model::perf::snapshot().delta_from(phases_before);
    debug_assert_eq!(
        merged.catalogue_mesh_cache,
        catalogue_mesh_cache.stats(),
        "merged per-root catalogue-cache deltas must equal final batch counters",
    );
    obj.flush()?;

    let elements_path = args.out.join("elements.json");
    std::fs::write(&elements_path, serde_json::to_vec_pretty(&invariants)?)?;
    if args.rvm {
        let rvm_path = args.out.join("generated.rvm");
        let file = e3d_model::rvm_read::RvmFile {
            header: e3d_model::rvm_write::generated_header(
                format!(
                    "generated by e3d-model from {} (root {})",
                    args.db_file.display(),
                    args.root
                        .map(|r| r.to_string())
                        .unwrap_or_else(|| "<whole db>".to_string()),
                ),
                "AMS",
                "/ALL",
            ),
            members: rvm_members,
        };
        file.write(&rvm_path)?;
        println!(
            "RVM: {} 个成员（解析图元 {rvm_analytic} / 自校未过退回面片 {rvm_analytic_rejected} / 拉伸体 / 多面体照 E3D 写面片 {rvm_extrusion_facets} / 布尔结果写面片 {rvm_boolean_facets}（export holes {}） / 零尺寸空记录 {rvm_zero_size} / 点标记 {rvm_marks} / 重复放置补写 {rvm_repeated_records}）-> {}",
            file.members.len() - 1,
            if holes_on { "on" } else { "off" },
            rvm_path.display()
        );
    }
    let report_path = args.out.join("report.json");
    std::fs::write(&report_path, serde_json::to_vec_pretty(&merged)?)?;
    let cache_path = args.out.join("primitive-cache.json");
    let cache_stats = e3d_model::primitive_cache_stats();
    std::fs::write(&cache_path, serde_json::to_vec_pretty(&cache_stats)?)?;
    if args.route_only {
        std::fs::write(
            args.out.join("routes.json"),
            serde_json::to_vec_pretty(&route_audit)?,
        )?;
    }

    println!("{}", merged.totals_line());
    println!("按 noun 生成: {noun_counts:?}");
    println!("基本体实例缓存: {cache_stats:?}");
    println!(
        "产物: {} / {} / {}（{} ms）",
        obj_path.display(),
        elements_path.display(),
        report_path.display(),
        t0.elapsed().as_millis()
    );
    // 分档：生成 / 三角化 / 写 OBJ 三段之和小于循环总时长，差额是审计与记账。
    // 读取一档**含在生成里**，不是并列的第四段，单列是为了看重复解析占多少。
    println!(
        "计时: 循环外 {}ms | 循环 {}ms = 生成 {}ms + 三角化 {}ms + 写OBJ {}ms + 其它 {}ms",
        (t0.elapsed().as_nanos() - loop_nanos) / 1_000_000,
        loop_nanos / 1_000_000,
        gen_nanos / 1_000_000,
        mesh_nanos / 1_000_000,
        obj_nanos / 1_000_000,
        loop_nanos.saturating_sub(gen_nanos + mesh_nanos + obj_nanos) / 1_000_000,
    );
    // `parse_misses / visited` 是「每个元素平均被真读了几遍」。句柄自己的 OnceLock
    // 只在同一个句柄内省，跨句柄（owner 链上溯、目录链重复引用）省不到——那部分
    // 只有给 DbSet 加共享缓存才能消掉，这个倍率就是它的收益上界。
    println!(
        "读取（含在生成里）: {} | 每元素真读 {:.2} 遍（visited {}）",
        reads.summary(),
        reads.parse_misses as f64 / merged.visited.max(1) as f64,
        merged.visited,
    );
    // 各档**嵌套**、不可相加：catalogue 里套着 mesh_build，二者产出的实体随后才被
    // tri_mesh 三角化。看绝对值和 ms/次，判断往哪一层下刀。
    println!("分档（嵌套，ms/次数）: {}", phases.summary());

    // 产物先落盘再判账：账不平时那三份文件正是查因用的。
    if let Some(total) = index_total {
        merged.accounts_for(total).map_err(|detail| {
            anyhow::anyhow!(
                "{detail}\n有元素既没被遍历也没被属主消费——先查 report.json 的 consumed/orphans"
            )
        })?;
        println!("账已平：visited + consumed = 索引 {total}");
    }
    Ok(())
}

fn aabb_of_points(points: impl IntoIterator<Item = [f64; 3]>) -> Option<([f64; 3], [f64; 3])> {
    let mut points = points.into_iter();
    let first = points.next()?;
    let mut lo = first;
    let mut hi = first;
    for point in points {
        for axis in 0..3 {
            lo[axis] = lo[axis].min(point[axis]);
            hi[axis] = hi[axis].max(point[axis]);
        }
    }
    Some((lo, hi))
}

#[cfg(test)]
mod tests {
    use super::{
        Recogniser, catalogue_noun_writes_facets_only, is_record_splinter,
        recogniser_for_catalogue_noun, write_uncut_catalogue_parts,
    };

    /// `/1RX07-LCT` 的 90° 槽弯（网格是盒子）与 ams8000 竖梯的圆板（网格是圆柱）都是 SEXT，
    /// E3D 写 FacetGroup。`SBOX` / `SCYL` 仍由出身先挑中。出身不明时不挡。
    #[test]
    fn a_catalogue_sext_is_offered_no_shape_recogniser() {
        assert!(catalogue_noun_writes_facets_only(Some("SEXT")));
        assert!(recogniser_for_catalogue_noun("SEXT").is_none());
        assert!(!catalogue_noun_writes_facets_only(Some("SBOX")));
        assert_eq!(recogniser_for_catalogue_noun("SBOX"), Some(Recogniser::Box));
        assert!(!catalogue_noun_writes_facets_only(Some("SCYL")));
        assert_eq!(
            recogniser_for_catalogue_noun("SCYL"),
            Some(Recogniser::Cylinder)
        );
        assert!(!catalogue_noun_writes_facets_only(None));
    }

    /// C-IY 带负体的托盘 / 槽弯按可见 SBOX / SEXT / SREV 拆开。NOZZ 的 SCYL 仍整只一条。
    /// 4° 弯多出来的薄片扔掉；整只都缩到 0.03 mm 时真块留下。
    #[test]
    fn negatives_on_tray_parts_split_unless_they_are_splinters() {
        assert!(write_uncut_catalogue_parts(
            1,
            &["SBOX".into(), "SBOX".into(), "SBOX".into()]
        ));
        assert!(write_uncut_catalogue_parts(
            1,
            &["SEXT".into(), "SREV".into()]
        ));
        assert!(!write_uncut_catalogue_parts(
            1,
            &["SCYL".into(), "SCYL".into()]
        ));
        assert!(!write_uncut_catalogue_parts(0, &["SBOX".into()]));
        assert!(!write_uncut_catalogue_parts(1, &[]));
        assert!(is_record_splinter(0.105, &[7.18, 7.18, 7.36]));
        assert!(!is_record_splinter(7.18, &[0.105, 7.18, 7.36]));
        assert!(is_record_splinter(0.0004, &[0.027, 0.027, 0.028]));
        assert!(!is_record_splinter(0.027, &[0.0004, 0.027, 0.028]));
        assert!(!is_record_splinter(48.5, &[50.0, 50.0, 100.0]));
    }
}

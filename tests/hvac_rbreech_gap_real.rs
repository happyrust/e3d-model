//! 真库门：裤形三通 `/RBREECH` 的 V 口——`GAP` 是**正体**（V 里的钣金填块），不是刀。
//!
//! 语料（2026-09-19，`_runs/ams-init-1112-7997-20260917-mc/e3d-hvac-tight-154700/p7 / p8 / p9_*`，
//! seg15 账 §26.12）：ams7997 唯一一只 RBREECH `THREEWAY 1 of BRANCH /-CUP-S-3-M-1405`（`24381/100890`），
//! DESP `RBRE 1200 510 820 710 1000 0 0 0 -352.5 100 100 265 …`。GMSET 全是 LPYR：
//!
//! - `/RBREECH-BODY1`（LPYR，P7→P6，1200×510 → 1905×710，高 900）名下 `NLPYRAMID 2`（**LEVE 0 10**，
//!   `Pbbtlength 0` / `Pbtplength DESP[13] = 265`，P8/P6）把 V 切出来；
//! - `/RBREECH-GAP`（LPYR **正体**，LEVE 0 10，`Pbbtlength 4`（写死）/ `Pbtplength DESP[13] + 10 = 275`，P8/P6）
//!   是 V 里的钣金填块，名下 `NGAP`（LEVE 0 10，`0 / 265`，y 向宽 100 好切穿）。
//!
//! 按图元算 (BODY1 − NLPY2) ∪ (GAP − NGAP)：V 面是 NLPY2 的面（元素局部系 x 0 → ±132.5，z 850 → 950，
//! 底是 x = 0 的尖点），GAP − NGAP 那两片 2 → 5 mm 的斜壁落在 BODY1 的实体里。E3D 的 CSG 面片
//! （`thre-100890-holeson.rvm`，`repre level hvac 6`）正是这个。引擎此前出的却是 **GAP 的外轮廓**
//! （x ±2 → ±137.5、底 4 mm 平台）——等于把 GAP 整只当刀从 BODY1 里挖掉，少了两片斜壁 + 4 mm 底。
//!
//! 缺 E3D 安装或影子资产**默认红**（与 `increment_real.rs` 同一口径）；确属无语料环境显式设
//! `E3D_SKIP_REAL_GATES=1` 换回跳过。

use std::path::{Path, PathBuf};
use std::sync::Arc;

use e3d_attlib::AttlibData;
use e3d_io::db_element::{DbFilePin, DbFileResolver, DbSet, template_file_for};
use e3d_io::refno::RefNo;
use e3d_model::catalogue::{CatalogueMeshCache, model_catalogue_member_with_cache};
use e3d_model::db_discovery::DirectoryDbResolver;
use manifold_csg::Manifold;

const AMS_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaMarineSample\ams000";
const ACP_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaCatalogue\acp000";
const DESIGN_DB: &str = "ams7997_0001";
const PROJECT_CATALOGUE: &str = "ams7000_0001";
const ATTLIB: &str = r"E:\reverse\e3d\shadow_e3d31_aps_all\attlib.dat";
const TEMPLATE_DIR: &str = r"E:\reverse\e3d\shadow_e3d31_aps_all";

fn fixtures_present() -> bool {
    let design = Path::new(AMS_DIR).join(DESIGN_DB);
    let missing: Vec<String> = [
        Path::new(AMS_DIR),
        design.as_path(),
        Path::new(ATTLIB),
        Path::new(TEMPLATE_DIR),
    ]
    .into_iter()
    .filter(|path| !path.exists())
    .map(|path| path.display().to_string())
    .collect();
    if missing.is_empty() {
        return true;
    }
    if std::env::var_os("E3D_SKIP_REAL_GATES").is_some() {
        eprintln!(
            "跳过（E3D_SKIP_REAL_GATES 已设）：缺 {}",
            missing.join("、")
        );
        return false;
    }
    panic!(
        "真库门语料缺失：{}。缺席不许静默变绿；确属无语料环境要跳过，显式设 E3D_SKIP_REAL_GATES=1。",
        missing.join("、"),
    );
}

struct SharedResolver(Arc<DirectoryDbResolver>);

impl DbFileResolver for SharedResolver {
    fn resolve(&self, dbno: u32) -> Option<DbFilePin> {
        self.0.resolve(dbno)
    }
}

/// 设计库开最新会话、目录库按需补开（与 `hvac_owned_negative_level_real.rs` 同款）。
fn open_design_set() -> Arc<DbSet> {
    let mut dirs = vec![PathBuf::from(AMS_DIR)];
    if Path::new(ACP_DIR).is_dir() {
        dirs.push(PathBuf::from(ACP_DIR));
    }
    let resolver = Arc::new(
        DirectoryDbResolver::scan(&dirs, Path::new(TEMPLATE_DIR))
            .expect("扫库目录建 dbno → 文件表"),
    );
    let attlib = Arc::new(AttlibData::parse_file(ATTLIB).expect("attlib 解析得开"));
    let set = Arc::new(DbSet::with_resolver(
        attlib,
        Box::new(SharedResolver(resolver)),
    ));
    let design = Path::new(AMS_DIR).join(DESIGN_DB);
    set.add_db(DbFilePin {
        file: design.clone(),
        template: template_file_for(Path::new(TEMPLATE_DIR), "DESI").expect("DESI 模板"),
        db_type: Some("DESI".to_string()),
        sesno: None,
    })
    .unwrap_or_else(|error| panic!("{} 开最新会话：{error}", design.display()));
    let catalogue = Path::new(AMS_DIR).join(PROJECT_CATALOGUE);
    if catalogue.is_file() {
        set.add_db(DbFilePin {
            file: catalogue.clone(),
            template: template_file_for(Path::new(TEMPLATE_DIR), "CATA").expect("CATA 模板"),
            db_type: Some("CATA".to_string()),
            sesno: None,
        })
        .unwrap_or_else(|error| panic!("{} 作目录库登记：{error}", catalogue.display()));
    }
    set
}

/// `24381/100890` 的世界放置（surreal `inst_relate.world_trans_d`，与 E3D `Q POS / ORI` 同一组数）：
/// 元素局部 → 世界。测试里的探针点按元素局部系写，好跟 RVM FacetGroup 与 TTY 探针对得上。
fn local_to_world(p: [f64; 3]) -> [f64; 3] {
    let q = glam::DQuat::from_xyzw(-0.65676755, -0.2620237, 0.2620237, 0.65676755);
    let t = glam::DVec3::new(17736.281, 8665.665, 5850.0);
    let w = q * glam::DVec3::from_array(p) + t;
    w.to_array()
}

fn world_to_local(w: [f64; 3]) -> [f64; 3] {
    let q = glam::DQuat::from_xyzw(-0.65676755, -0.2620237, 0.2620237, 0.65676755);
    let t = glam::DVec3::new(17736.281, 8665.665, 5850.0);
    (q.inverse() * (glam::DVec3::from_array(w) - t)).to_array()
}

/// 实体在元素局部点 `p` 处是不是有料：拿 1 mm 的小方块与实体求交，体积 > 0.5 算实。
fn solid_at(solid: &Manifold, p: [f64; 3]) -> bool {
    let [x, y, z] = local_to_world(p);
    let probe = Manifold::cube(1.0, 1.0, 1.0, true).translate(x, y, z);
    solid.intersection(&probe).volume() > 0.5
}

/// 网格顶点逐个折回元素局部系再取盒（世界盒的八个角折回去会虚大，元素是斜置的）。
fn local_bbox(part: &Manifold) -> Option<([f64; 3], [f64; 3])> {
    let (properties, stride, _triangles) = part.to_mesh_f64();
    if stride < 3 || properties.len() < 3 {
        return None;
    }
    let mut mn = [f64::INFINITY; 3];
    let mut mx = [f64::NEG_INFINITY; 3];
    for vertex in properties.chunks(stride) {
        let l = world_to_local([vertex[0], vertex[1], vertex[2]]);
        for k in 0..3 {
            mn[k] = mn[k].min(l[k]);
            mx[k] = mx[k].max(l[k]);
        }
    }
    Some((mn, mx))
}

#[test]
fn rbreech_gap_is_a_positive_filler_not_a_cutter() {
    if !fixtures_present() {
        return;
    }
    assert!(
        std::env::var_os("E3D_REPRE_LEVEL_HVAC").is_none(),
        "这道门按缺省 HVAC 6 级裁，别带着 E3D_REPRE_LEVEL_HVAC 跑"
    );
    let set = open_design_set();
    let mut cache = CatalogueMeshCache::default();
    let refno = RefNo::new(24381, 100890);
    let member = set.element(refno);
    let built = model_catalogue_member_with_cache(&member, &mut cache)
        .unwrap_or_else(|error| panic!("{refno} 建模失败：{error}"));
    let generated = built.generated.unwrap_or_else(|| {
        panic!(
            "{refno} 没出几何：{:?} / {:?}",
            built.no_shape, built.diagnostics
        )
    });
    println!(
        "{refno} THRE /RBREECH: solid volume {:.1} mm³, genus {}, diagnostics {:?}",
        generated.solid.volume(),
        generated.solid.genus(),
        built.diagnostics
    );
    if let Some(parts) = generated.parts.as_deref() {
        for (i, part) in parts.iter().enumerate() {
            let noun = generated
                .part_catalogue_nouns
                .get(i)
                .map(String::as_str)
                .unwrap_or("?");
            let bb = local_bbox(part)
                .map(|(mn, mx)| {
                    format!(
                        "x[{:.1},{:.1}] y[{:.1},{:.1}] z[{:.1},{:.1}]",
                        mn[0], mx[0], mn[1], mx[1], mn[2], mx[2]
                    )
                })
                .unwrap_or_else(|| "(empty)".to_string());
            println!(
                "  part {i:2} {noun:5} volume {:>14.1}  local bbox {bb}",
                part.volume()
            );
        }
    }
    // 标架自检：整体世界盒与 surreal `aabb_d` / E3D RVM 同一组数（元素局部 [-1002.5,-305,0]..[1002.5,505,1000]）。
    let (mn, mx) = local_bbox(&generated.solid).expect("有体");
    for (got, want) in mn
        .iter()
        .zip([-1002.5, -305.0, 0.0])
        .chain(mx.iter().zip([1002.5, 505.0, 1000.0]))
    {
        assert!(
            (got - want).abs() < 0.05,
            "元素局部盒应是 [-1002.5,-305,0]..[1002.5,505,1000]，得到 {mn:?}..{mx:?}"
        );
    }

    for p in [
        [30.0, 100.0, 900.0],
        [-30.0, 100.0, 900.0],
        [300.0, 100.0, 900.0],
        [68.0, 100.0, 900.0],
        [-68.0, 100.0, 900.0],
        [3.7, 100.0, 852.0],
        [0.0, 100.0, 840.0],
        [0.0, 100.0, 860.0],
        [68.0, -200.0, 900.0],
        [68.0, 400.0, 900.0],
        [80.0, 100.0, 900.0],
        [60.0, 100.0, 900.0],
    ] {
        println!("  probe {p:?} -> solid {}", solid_at(&generated.solid, p));
    }
    if let Some(parts) = generated.parts.as_deref() {
        for (i, part) in parts.iter().enumerate().filter(|(i, _)| *i >= 9) {
            for p in [
                [30.0, 100.0, 900.0],
                [68.0, 100.0, 900.0],
                [3.7, 100.0, 852.0],
                [100.0, 100.0, 925.0],
                [0.0, 100.0, 860.0],
                [0.0, -280.0, 900.0],
                [0.0, 480.0, 900.0],
            ] {
                println!("  part {i} probe {p:?} -> solid {}", solid_at(part, p));
            }
        }
    }
    // V 口横截面 z = 900（元素局部）：NLPY2 的面在 x = ±66.25，GAP 的外轮廓在 x = ±69.75。
    // (a) V 里面是空的；(b) GAP − NGAP 那片壁（66.25 < |x| < 69.75）是实的——E3D 的面片正是这样；
    // (c) 腿的本体当然是实的。以前引擎在 (b) 处是空的（V 面落在 GAP 的外轮廓上）。
    assert!(
        !solid_at(&generated.solid, [30.0, 100.0, 900.0]),
        "V 口里 (30, 100, 900) 该是空的"
    );
    assert!(
        !solid_at(&generated.solid, [-30.0, 100.0, 900.0]),
        "V 口里 (−30, 100, 900) 该是空的"
    );
    assert!(
        solid_at(&generated.solid, [300.0, 100.0, 900.0]),
        "腿本体 (300, 100, 900) 该是实的"
    );
    for x in [68.0, -68.0] {
        assert!(
            solid_at(&generated.solid, [x, 100.0, 900.0]),
            "GAP − NGAP 的斜壁 ({x}, 100, 900) 该是实的：V 面该落在 NLPY2 的面（x = ±66.25）上，不是 GAP 的外轮廓（±69.75）"
        );
    }
    // V 口底：E3D 是 x = 0 的尖点。z = 852 处 NLPY2 的刃口在 x = ±2.65、GAP 的外轮廓在 ±4.71，
    // (3.7, 100, 852) 落在那片壁里，该是实的——以前引擎这里是 4 mm 宽平台之上的空。
    assert!(
        solid_at(&generated.solid, [3.7, 100.0, 852.0]),
        "V 口底 (3.7, 100, 852) 在 GAP − NGAP 的壁里，该是实的"
    );
}

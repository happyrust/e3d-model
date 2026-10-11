//! 管支架鞍座 GMSE 的 SRTO 环：弧端 P 点的 `PX` / `PZ` 与 `PTCD` 角都写成
//! `180 * I / ( PI * ( OD + t ) )`，e3d-io 的 `catalogue_pml` 读得出 `PI`（操作码 111）
//! 两根轴才落到管周上，`torus_frame` 的一般支随即把环建出来——`torus_frame` 本身没改。
//!
//! 代表 `/1WCC-R220.003/ATTA-02`（24383/68277，GMSE-A 一条环）与
//! `/R710.026A/ATTA-05`（24383/66529，GMSE-C 四条环）。真值是 ZONE 66457 的 E3D RVM
//! 记录列（`plant-model-5/outputs/rvm-check-20261010/s2-frames/t23-{a,c}-e3d.json`）。
//! 修前两只都没有网格：三段程序因 `PI` 不可读整段被拒，`PX`/`PZ` 落 0、方向落 +Y，
//! 两轴重合在原点，`torus_frame` 判成「零高或退化」。

use std::path::{Path, PathBuf};
use std::sync::Arc;

use e3d_io::db_element::{DbFilePin, DbSet, template_file_for};
use e3d_io::refno::RefNo;
use e3d_model::catalogue::model_catalogue_member;
use e3d_model::db_discovery::DirectoryDbResolver;
use e3d_model::mesh::TriMesh;
use glam::{DMat4, DVec3};

const AMS_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\AvevaMarineSample\ams000";
const ZDJ_DIR: &str = r"D:\AVEVA\Projects\E3D3.1\ZDJ\ZDJ000";
const DESIGN_DB: &str = "ams7999_0001";
const TEMPLATE_DIR: &str = r"D:\work\plant-code\old\vendor\e3d-io\schema\e3d31\binaries";

fn fixtures_present() -> bool {
    let design = Path::new(AMS_DIR).join(DESIGN_DB);
    let attlib = Path::new(TEMPLATE_DIR).join("attlib.dat");
    if design.exists() && attlib.exists() {
        return true;
    }
    if std::env::var_os("E3D_SKIP_REAL_GATES").is_some() {
        return false;
    }
    panic!("真库门语料缺失。无语料环境要跳过，显式设 E3D_SKIP_REAL_GATES=1。");
}

fn ams_set() -> Arc<DbSet> {
    let template = Path::new(TEMPLATE_DIR);
    let resolver = DirectoryDbResolver::scan(
        &[PathBuf::from(AMS_DIR), PathBuf::from(ZDJ_DIR)],
        template,
    )
    .unwrap();
    let set = Arc::new(
        DbSet::with_attlib_file_and_resolver(&template.join("attlib.dat"), Box::new(resolver))
            .unwrap(),
    );
    set.add_db(DbFilePin {
        file: Path::new(AMS_DIR).join(DESIGN_DB),
        template: template_file_for(template, "DESI").unwrap(),
        db_type: Some("DESI".into()),
        sesno: None,
    })
    .unwrap();
    set
}

/// 每条 SRTO 记录的世界标架（`world × 图元标架`）和它那块网格。
fn saddle_records(set: &Arc<DbSet>, w1: u32) -> Vec<(DMat4, manifold_csg::Manifold)> {
    let element = set.element(RefNo::new(24383, w1));
    let built = model_catalogue_member(&element).unwrap();
    let generated = built.generated.expect("鞍座应出实体");
    let parts = generated.parts.clone().expect("目录成员按块保留");
    assert_eq!(parts.len(), generated.part_catalogue_nouns.len());
    assert_eq!(parts.len(), generated.part_record_repeats.len());
    assert!(
        generated
            .part_catalogue_nouns
            .iter()
            .all(|noun| noun == "SRTO"),
        "名词 {:?}",
        generated.part_catalogue_nouns
    );
    parts
        .into_iter()
        .zip(&generated.part_record_repeats)
        .map(|(part, record)| {
            assert_eq!(record.repeats, 1);
            (generated.world * record.frame.expect("SRTO 有标架"), part)
        })
        .collect()
}

fn columns(frame: &DMat4) -> [DVec3; 4] {
    [
        frame.x_axis.truncate(),
        frame.y_axis.truncate(),
        frame.z_axis.truncate(),
        frame.w_axis.truncate(),
    ]
}

/// 与一条 E3D 记录的四列（三根轴 + 原点）的最大逐列差。轴列按 1e-4 存，原点按 mm。
fn column_error(got: &DMat4, want: &[DVec3; 4]) -> f64 {
    columns(got)
        .iter()
        .zip(want)
        .map(|(g, w)| (*g - *w).length())
        .fold(0.0, f64::max)
}

fn local_aabb(frame: &DMat4, part: &manifold_csg::Manifold) -> (DVec3, DVec3) {
    let inverse = frame.inverse();
    let mut lo = DVec3::splat(f64::INFINITY);
    let mut hi = DVec3::splat(f64::NEG_INFINITY);
    for vertex in TriMesh::from_manifold(part).vertices {
        let local = inverse.transform_point3(DVec3::from_array(vertex));
        lo = lo.min(local);
        hi = hi.max(local);
    }
    (lo, hi)
}

/// GMSE-A：一条环，PAAX=P6 / PBAX=P7 在管周 `(OD + t) / 2 = 85.85` 上、对称分居 −Z
/// 两侧各半个张角，方向是弧端外切线。E3D 画 inner 84.15 / outer 87.55 / 高 150 /
/// 张角 0.52417 rad，记录原点 = 构件原点，X 列指向 P7、Y 列 = −PBAX、Z = X × Y。
#[test]
fn gmse_a_saddle_record_matches_e3d() {
    if !fixtures_present() {
        return;
    }
    let set = ams_set();
    let records = saddle_records(&set, 68277);
    assert_eq!(records.len(), 1, "GMSE-A 一条环");
    let (frame, part) = &records[0];
    let want = [
        DVec3::new(0.2319, 0.1156, -0.9659),
        DVec3::new(-0.8644, -0.4310, -0.2591),
        DVec3::new(-0.4462, 0.8949, 0.0),
        DVec3::new(9232.96, -19414.91, -2700.0),
    ];
    let err = column_error(frame, &want);
    assert!(err < 2.0e-3, "四列差 {err}：{:?}", columns(frame));

    // 记录系里的盒：弧从 +X 起沿 +Y 扫 0.52417 rad，内 84.15 外 87.55，高 ±75。
    // E3D 记录自带的 bbox 是 [72.852, 0, −75] – [87.55, 43.818, 75]（t23-a-e3d.json）。
    let (lo, hi) = local_aabb(frame, part);
    let angle: f64 = 2.0 * 45.0 / (168.3 + 3.4);
    let want_lo = DVec3::new(84.15 * angle.cos(), 0.0, -75.0);
    let want_hi = DVec3::new(87.55, 87.55 * angle.sin(), 75.0);
    assert!(lo.abs_diff_eq(want_lo, 0.05), "lo {lo} 应为 {want_lo}");
    assert!(hi.abs_diff_eq(want_hi, 0.05), "hi {hi} 应为 {want_hi}");
}

/// GMSE-C：同一原点上四条环（P4/P5、P6/P7、P8/P9、P13/P14），两两扫掠轴相反；
/// 四条记录逐一对上 E3D 的四条（四列全同），不能并成一条。
#[test]
fn gmse_c_saddle_writes_four_rings_matching_e3d() {
    if !fixtures_present() {
        return;
    }
    let set = ams_set();
    let records = saddle_records(&set, 66529);
    assert_eq!(records.len(), 4, "GMSE-C 四条环，got {}", records.len());
    let origin = DVec3::new(17638.97, 4754.66, 16679.0);
    let want = [
        [
            DVec3::new(0.2630, 0.1311, 0.9559),
            DVec3::new(-0.8554, -0.4265, 0.2939),
            DVec3::new(0.4462, -0.8949, 0.0),
            origin,
        ],
        [
            DVec3::new(0.2630, 0.1311, -0.9559),
            DVec3::new(-0.8554, -0.4265, -0.2939),
            DVec3::new(-0.4462, 0.8949, 0.0),
            origin,
        ],
        [
            DVec3::new(-0.8554, -0.4265, 0.2939),
            DVec3::new(-0.2630, -0.1311, -0.9559),
            DVec3::new(0.4462, -0.8949, 0.0),
            origin,
        ],
        [
            DVec3::new(0.8554, 0.4265, 0.2939),
            DVec3::new(0.2630, 0.1311, -0.9559),
            DVec3::new(-0.4462, 0.8949, 0.0),
            origin,
        ],
    ];
    let mut taken = [false; 4];
    for (frame, part) in &records {
        let (index, err) = want
            .iter()
            .enumerate()
            .map(|(index, want)| (index, column_error(frame, want)))
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .unwrap();
        assert!(err < 2.0e-3, "四列差 {err}：{:?}", columns(frame));
        assert!(!taken[index], "两条环对到了同一条 E3D 记录 {index}");
        taken[index] = true;

        let (lo, hi) = local_aabb(frame, part);
        let angle: f64 = 2.0 * 35.0 / (114.3 + 3.05);
        let want_lo = DVec3::new(57.15 * angle.cos(), 0.0, -75.0);
        let want_hi = DVec3::new(60.2, 60.2 * angle.sin(), 75.0);
        assert!(lo.abs_diff_eq(want_lo, 0.05), "lo {lo} 应为 {want_lo}");
        assert!(hi.abs_diff_eq(want_hi, 0.05), "hi {hi} 应为 {want_hi}");
    }
}

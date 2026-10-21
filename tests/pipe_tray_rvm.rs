//! AMS 管道式桥架与 E3D 原生 RVM 的局部包围盒回归。
//!
//! 基线是 GenModelRvmExport addin 的导出（`C-IY-1R330-B.rvm`），口径是管道 6 级、障碍体开、holes off。
//! 拼接板的 OUTA / OUTB 只在这一口径下算数：E3D 自己的 TTY 导出照画拼接板。
//!
//! 运行：
//! `$env:E3D_REPRE_LEVEL_PIPE='6'; $env:E3D_REPRE_OBST='on'; $env:E3D_EXPORT_HOLES='off'; cargo test --release --test pipe_tray_rvm -- --ignored --nocapture`

use std::{path::PathBuf, sync::Arc};

use e3d_io::{
    db_element::{DbFilePin, DbSet, template_file_for},
    refno::RefNo,
};
use e3d_model::{
    catalogue::model_catalogue_member, db_discovery::DirectoryDbResolver, mesh::TriMesh,
    pipeline::route_tubes_of,
};
use glam::DVec3;

fn pool() -> Arc<DbSet> {
    let projects = PathBuf::from(
        std::env::var("E3D_PIPE_TRAY_PROJECTS")
            .unwrap_or_else(|_| "D:/AVEVA/Projects/E3D3.1".into()),
    );
    let template = PathBuf::from(
        std::env::var("E3D_PIPE_TRAY_TEMPLATE")
            .unwrap_or_else(|_| "E:/reverse/e3d/shadow_e3d31_aps_all".into()),
    );
    let ams = projects.join("AvevaMarineSample/ams000");
    let resolver = DirectoryDbResolver::scan(
        &[ams.clone(), projects.join("AvevaCatalogue/acp000")],
        &template,
    )
    .unwrap();
    let set = Arc::new(
        DbSet::with_attlib_file_and_resolver(&template.join("attlib.dat"), Box::new(resolver))
            .unwrap(),
    );
    set.add_db(DbFilePin {
        file: ams.join("ams8000_0001"),
        template: template_file_for(&template, "DESI").unwrap(),
        db_type: Some("DESI".into()),
        // The native RVM fixture was exported from this frozen AMS snapshot.
        // Keep the differential regression independent from later SAVEWORKs.
        sesno: Some(281),
    })
    .unwrap();
    set
}

fn local_bounds(set: &Arc<DbSet>, word1: u32) -> (DVec3, DVec3, usize, Vec<String>) {
    let build = model_catalogue_member(&set.element(RefNo::new(24384, word1))).unwrap();
    let generated = build.generated.expect("BEND generated");
    let inverse = generated.world.inverse();
    let mut lo = DVec3::splat(f64::INFINITY);
    let mut hi = DVec3::splat(f64::NEG_INFINITY);
    for solid in generated.components() {
        for vertex in TriMesh::from_manifold(&solid).vertices {
            let local = inverse.transform_point3(DVec3::from_array(vertex));
            lo = lo.min(local);
            hi = hi.max(local);
        }
    }
    (lo, hi, generated.components().len(), build.diagnostics)
}

#[test]
#[ignore = "requires local AMS 8000 and ACP catalogue databases"]
fn tray_bends_match_native_e3d_rvm_and_application_filters() {
    let set = pool();
    let cases = [
        // BEND 4: rail + reserved + cable families; OBST=2 and COVR=0 are absent.
        (
            22412,
            DVec3::new(-51.501, -265.470, 0.0),
            DVec3::new(255.653, 177.336, 150.0),
            9,
        ),
        // BEND 1: OUTA/OUTB=1 suppress both splice plates.
        (
            22406,
            DVec3::new(-2.307, -52.802, 0.0),
            DVec3::new(6.901, 52.799, 150.0),
            // BOARD-1 is disconnected but belongs to the same native physical
            // facet group, hence four manifold components for three RVM groups.
            4,
        ),
    ];
    for (word1, expected_lo, expected_hi, expected_components) in cases {
        let (lo, hi, components, diagnostics) = local_bounds(&set, word1);
        if word1 == 22406 {
            assert!(
                diagnostics.iter().any(|line| line.contains("rpro-outa")),
                "OUTA filter missing: {diagnostics:?}"
            );
            assert!(
                diagnostics.iter().any(|line| line.contains("rpro-outb")),
                "OUTB filter missing: {diagnostics:?}"
            );
        }
        assert!(
            lo.abs_diff_eq(expected_lo, 0.60),
            "{word1}: lo={lo}, components={components}"
        );
        assert!(
            hi.abs_diff_eq(expected_hi, 0.60),
            "{word1}: hi={hi}, components={components}"
        );
        assert_eq!(components, expected_components, "{word1}: component count");
    }
}

#[test]
#[ignore = "requires local AMS 8000 and ACP catalogue databases"]
fn native_pipe_branch_suppresses_invalid_implied_tubes() {
    let set = pool();
    // /C-OR-1R345-C.  Its native E3D RVM contains seven FTUBE and two BEND
    // groups, with no implied TUBI.  The three route slots all fail the native
    // alignment/head gates and must therefore stay out of generated geometry.
    let tubes = route_tubes_of(&set.element(RefNo::new(24384, 23257))).unwrap();
    assert!(
        tubes.is_empty(),
        "native branch has no implied TUBI, generated: {:?}",
        tubes
            .iter()
            .map(|element| &element.notes)
            .collect::<Vec<_>>()
    );
}

//! Native E3D PRIM records, not the old Y-up RVM tessellator, are the oracle.
//! Run with the AMS/CATA installation: cargo test --release --test strt_rvm -- --ignored
use std::{collections::BTreeMap, path::PathBuf, sync::Arc};

use e3d_io::{
    db_element::{DbFilePin, DbSet, template_file_for},
    refno::RefNo,
};
use e3d_model::{
    catalogue::model_catalogue_member, db_discovery::DirectoryDbResolver, mesh::TriMesh,
};
use glam::{DMat4, DVec3};
use rvm_rs::{
    parse_rvm,
    store::{
        Store,
        geometry::GeometryKind,
        node::{NodeId, NodeKind},
    },
};

fn cylinders(store: &Store, id: NodeId, groups: &mut BTreeMap<String, Vec<(f64, f64, DMat4)>>) {
    let node = store.get_node(id).unwrap();
    if let NodeKind::Group(group) = &node.kind {
        let name = store.get_string(group.name).trim().to_owned();
        if name.starts_with("STRT ") {
            let mut link = group.first_geometry;
            let mut values = Vec::new();
            while let Some(id) = link {
                let geometry = store.get_geometry(id).unwrap();
                let GeometryKind::Cylinder(cylinder) = &geometry.kind else {
                    panic!("fixture must contain native cylinders: {name}");
                };
                let cols = geometry.transform.to_cols_array();
                let vector = |start| {
                    DVec3::new(
                        cols[start] as f64,
                        cols[start + 1] as f64,
                        cols[start + 2] as f64,
                    ) * 1000.0
                };
                let world = DMat4::from_cols(
                    vector(0).extend(0.0),
                    vector(3).extend(0.0),
                    vector(6).extend(0.0),
                    vector(9).extend(1.0),
                );
                values.push((cylinder.radius as f64, cylinder.height as f64, world));
                link = geometry.next;
            }
            groups.insert(name, values);
        }
    }
    let mut link = node.first_child;
    while let Some(id) = link {
        cylinders(store, id, groups);
        link = store.get_node(id).unwrap().next;
    }
}

fn pool() -> Arc<DbSet> {
    let projects = PathBuf::from(
        std::env::var("E3D_STRT_PROJECTS").unwrap_or_else(|_| "D:/AVEVA/Projects/E3D3.1".into()),
    );
    let template = PathBuf::from(
        std::env::var("E3D_STRT_TEMPLATE")
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
    for (file, kind, sesno) in [
        ("ams7997_0001", "DESI", Some(106)),
        ("ams7000_0001", "CATA", None),
    ] {
        set.add_db(DbFilePin {
            file: ams.join(file),
            template: template_file_for(&template, kind).unwrap(),
            db_type: Some(kind.into()),
            sesno,
        })
        .unwrap();
    }
    set
}

#[test]
#[ignore = "requires the local AMS 7997 session 106 and its catalogue files"]
fn cpv_square_round_transitions_evaluate_sqrt_and_atan() {
    let set = pool();
    for word1 in [76829, 76836] {
        let element = set.element(RefNo::new(24381, word1));
        let generated = model_catalogue_member(&element)
            .unwrap()
            .generated
            .expect("TRNS generated");
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
        // Native E3D RVM full body + end flanges. Previously +/-225 mm:
        // unresolved SQRT(125²+125²) fell back to zero and PTCD ATAN was lost.
        assert!(
            lo.abs_diff_eq(DVec3::new(-165.0, -165.0, 0.0), 0.51),
            "{word1}: {lo}"
        );
        assert!(
            hi.abs_diff_eq(DVec3::new(165.0, 165.0, 600.0), 0.51),
            "{word1}: {hi}"
        );
    }
}

#[test]
#[ignore = "requires the local AMS 7997 session 106 and its catalogue files"]
fn real_strt_bodies_and_each_flange_match_native_e3d_rvm() {
    let set = pool();
    let cases: &[(&[u8], &str, &[u32])] = &[
        (
            include_bytes!("fixtures/strt/cpv-hvac6.rvm"),
            "/-CPV-S-2-L-1301",
            &[76730, 76731, 76732],
        ),
        (
            include_bytes!("fixtures/strt/cam-hvac6.rvm"),
            "/Copy-of--CAM-S-1-M-5201",
            &[179334, 179342, 179348],
        ),
    ];
    for (bytes, branch, refs) in cases {
        let mut store = Store::new();
        parse_rvm(bytes, &mut store).unwrap();
        let mut groups = BTreeMap::new();
        for root in store.roots() {
            cylinders(&store, *root, &mut groups);
        }
        assert_eq!(groups.len(), refs.len());
        for (index, ref1) in refs.iter().enumerate() {
            let name = format!("STRT {} of BRANCH {branch}", index + 1);
            let generated = model_catalogue_member(&set.element(RefNo::new(24381, *ref1)))
                .unwrap()
                .generated
                .expect("STRT generated");
            let inverse = generated.world.inverse();
            let mut actual: Vec<_> = generated
                .components()
                .iter()
                .map(|solid| {
                    let mesh = TriMesh::from_manifold(solid);
                    let mut lo = DVec3::splat(f64::INFINITY);
                    let mut hi = DVec3::splat(f64::NEG_INFINITY);
                    for point in mesh.vertices {
                        let local = inverse.transform_point3(DVec3::from_array(point));
                        lo = lo.min(local);
                        hi = hi.max(local);
                    }
                    (lo, hi)
                })
                .collect();
            assert_eq!(actual.len(), 5, "{name}: body plus four flange cylinders");
            assert_eq!(groups[&name].len(), actual.len());
            for (radius, height, world) in &groups[&name] {
                let local = inverse * *world;
                let centre = local.w_axis.truncate();
                // Exact support extents of a centred Z-axis RVM cylinder.
                let x = local.x_axis.truncate();
                let y = local.y_axis.truncate();
                let radial = DVec3::from_array((x * x + y * y).to_array().map(f64::sqrt));
                let extent = radial * *radius + local.z_axis.truncate().abs() * (*height / 2.0);
                let expected = (centre - extent, centre + extent);
                let error = |bounds: &(DVec3, DVec3)| {
                    (bounds.0 - expected.0)
                        .abs()
                        .max_element()
                        .max((bounds.1 - expected.1).abs().max_element())
                };
                let best = (0..actual.len())
                    .min_by(|a, b| error(&actual[*a]).total_cmp(&error(&actual[*b])))
                    .unwrap();
                let difference = error(&actual.remove(best));
                assert!(
                    difference <= 0.51,
                    "{name}: RVM cylinder r={radius}, h={height}: {difference} mm error"
                );
            }
            assert!(actual.is_empty(), "no unmatched rendered components");
        }
    }
}

//! 实例化复用的前置判据：**把世界系顶点交给 Rust 算，与今天交给 manifold 算，是不是逐位相等？**
//!
//! 今天每个 BOX/CYLI 元素走的是
//!
//! ```text
//! canonical --placed(L)--> ensure_solid(物化) --placed(W)--> materialize --to_mesh_f64--> 顶点
//! ```
//!
//! 想省掉的是最后那条 `to_mesh_f64` 链（每件 11 µs 固定 + 0.42 µs/三角形）：只抽一次
//! 规范网格，逐件在 Rust 侧把顶点乘过去。能不能这么干，取决于两路算出来的 f64 位模式
//! 是否一致——差一个 ULP，`model.obj` / `aabb` / `local_aabb` 三处产物就都变了，
//! 「零行为变化」这条就不成立。
//!
//! 本探针不改任何生产路径，只回答这一个是非题。

use e3d_model::mesh::TriMesh;
use e3d_model::primitive_instance::{PrimitiveMeshKey, canonical_primitive_mesh};
use e3d_model::solid::{cylinder_segments, cylinder_tilt, placed};
use glam::{DMat4, DVec3};

/// 线性同余伪随机：不引依赖，且换机器结论可复现。
struct Rng(u64);

impl Rng {
    fn next_f64(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
    }

    /// `[lo, hi)` 上的均匀取值。
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.next_f64()
    }
}

/// 与 `transform::ori_matrix` 同族的任意姿态：绕三轴依次转。
/// 世界原点取到 10 万毫米量级——厂区坐标就是这个尺度，抵消误差在这里才显形。
fn random_world(rng: &mut Rng) -> DMat4 {
    DMat4::from_translation(DVec3::new(
        rng.range(-200_000.0, 200_000.0),
        rng.range(-200_000.0, 200_000.0),
        rng.range(-50_000.0, 50_000.0),
    )) * DMat4::from_rotation_z(rng.range(-std::f64::consts::PI, std::f64::consts::PI))
        * DMat4::from_rotation_y(rng.range(-std::f64::consts::PI, std::f64::consts::PI))
        * DMat4::from_rotation_x(rng.range(-std::f64::consts::PI, std::f64::consts::PI))
}

#[derive(Default)]
struct Tally {
    cases: usize,
    vertices: usize,
    /// 两步（先 L 后 W，与生产路径同序）在 Rust 侧重算，与 manifold 逐位相等的顶点数。
    two_step_exact: usize,
    /// 先在 Rust 侧折叠 `W*L` 再一次乘过去，与 manifold 逐位相等的顶点数。
    folded_exact: usize,
    two_step_max_ulp: u64,
    folded_max_ulp: u64,
    two_step_max_abs: f64,
    folded_max_abs: f64,
    index_mismatches: usize,
}

fn ulp_gap(a: f64, b: f64) -> u64 {
    if a == b {
        return 0;
    }
    if a.is_nan() || b.is_nan() {
        return u64::MAX;
    }
    // 单调映射到有序整数，跨零也能比。
    let key = |x: f64| {
        let bits = x.to_bits() as i64;
        if bits < 0 { i64::MIN - bits } else { bits }
    };
    key(a).abs_diff(key(b))
}

fn compare(tally: &mut Tally, canonical: &manifold_csg::Manifold, local: DMat4, world: DMat4) {
    // A 路：生产路径。`ensure_solid` 在局部那一步就 `is_empty()/num_tri()` 物化了一次，
    // 所以这里必须照样物化——不物化的话 manifold 会把 W*L 折成一次变换，是另一条路。
    let local_solid = placed(canonical, local);
    let _forced = local_solid.is_empty() || local_solid.num_tri() == 0;
    let world_solid = placed(&local_solid, world);
    let produced = TriMesh::from_manifold(&world_solid);

    // B 路：规范网格只抽一次，顶点在 Rust 侧乘过去。
    let base = TriMesh::from_manifold(canonical);
    if base.indices != produced.indices {
        tally.index_mismatches += 1;
    }
    if base.vertices.len() != produced.vertices.len() {
        tally.index_mismatches += 1;
        return;
    }

    let folded = world * local;
    for (source, expected) in base.vertices.iter().zip(&produced.vertices) {
        let v = DVec3::from_array(*source);
        let two_step = world.transform_point3(local.transform_point3(v)).to_array();
        let one_step = folded.transform_point3(v).to_array();
        let mut two_ok = true;
        let mut one_ok = true;
        for axis in 0..3 {
            let want = expected[axis];
            if two_step[axis].to_bits() != want.to_bits() {
                two_ok = false;
                tally.two_step_max_ulp = tally.two_step_max_ulp.max(ulp_gap(two_step[axis], want));
                tally.two_step_max_abs = tally.two_step_max_abs.max((two_step[axis] - want).abs());
            }
            if one_step[axis].to_bits() != want.to_bits() {
                one_ok = false;
                tally.folded_max_ulp = tally.folded_max_ulp.max(ulp_gap(one_step[axis], want));
                tally.folded_max_abs = tally.folded_max_abs.max((one_step[axis] - want).abs());
            }
        }
        tally.vertices += 1;
        tally.two_step_exact += usize::from(two_ok);
        tally.folded_exact += usize::from(one_ok);
    }
    tally.cases += 1;
}

fn report(name: &str, tally: &Tally) {
    println!(
        "{name}: 用例 {} 顶点 {} | 两步逐位相等 {} ({:.4}%) 最大 {} ULP / {:.3e} mm \
         | 折叠逐位相等 {} ({:.4}%) 最大 {} ULP / {:.3e} mm | 索引不一致 {}",
        tally.cases,
        tally.vertices,
        tally.two_step_exact,
        tally.two_step_exact as f64 * 100.0 / tally.vertices.max(1) as f64,
        tally.two_step_max_ulp,
        tally.two_step_max_abs,
        tally.folded_exact,
        tally.folded_exact as f64 * 100.0 / tally.vertices.max(1) as f64,
        tally.folded_max_ulp,
        tally.folded_max_abs,
        tally.index_mismatches,
    );
}

fn main() -> anyhow::Result<()> {
    let cases: usize = std::env::args()
        .nth(1)
        .map(|n| n.parse())
        .transpose()?
        .unwrap_or(2000);

    // BOX：`instantiate_box` 的 L 就是一个纯缩放。
    let cube = canonical_primitive_mesh(PrimitiveMeshKey::BoxV1)?;
    let mut rng = Rng(0x5eed_1234_abcd_0001);
    let mut box_tally = Tally::default();
    for _ in 0..cases {
        let local = DMat4::from_scale(DVec3::new(
            rng.range(1.0, 5000.0),
            rng.range(1.0, 5000.0),
            rng.range(1.0, 5000.0),
        ));
        compare(&mut box_tally, &cube, local, random_world(&mut rng));
    }
    report("BoxV1  ", &box_tally);

    // CYLI：`instantiate_cylinder` 的 L 是「绕 X 摆正 + 各向缩放」，段数进键。
    let mut cyl_tally = Tally::default();
    let mut rng = Rng(0x5eed_1234_abcd_0002);
    for _ in 0..cases {
        let diam = rng.range(10.0, 2000.0);
        let height = rng.range(-3000.0, 3000.0);
        let radius = diam / 2.0;
        let canonical = canonical_primitive_mesh(PrimitiveMeshKey::CylinderV3 {
            segments: cylinder_segments(radius),
        })?;
        let local = DMat4::from_rotation_x(cylinder_tilt(height))
            * DMat4::from_scale(DVec3::new(radius, radius, height.abs()));
        compare(&mut cyl_tally, &canonical, local, random_world(&mut rng));
    }
    report("CylV3  ", &cyl_tally);

    // 对照组：世界矩阵是恒等，只有局部缩放。用来分清「误差来自 W 的大平移」
    // 还是「两路本来就不同算」。
    let mut unit_tally = Tally::default();
    let mut rng = Rng(0x5eed_1234_abcd_0003);
    for _ in 0..cases {
        let local = DMat4::from_scale(DVec3::new(
            rng.range(1.0, 5000.0),
            rng.range(1.0, 5000.0),
            rng.range(1.0, 5000.0),
        ));
        compare(&mut unit_tally, &cube, local, DMat4::IDENTITY);
    }
    report("Box|W=I", &unit_tally);

    Ok(())
}

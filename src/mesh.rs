//! 三角网格产物（对标 `gm_CreateFacetStructure` 的输出面）。
//!
//! 只承载 RVM 对拍与目验要用的东西：f64 顶点 + 三角索引 + 不变量
//! （AABB / 体积 / 三角数）。渲染法线、光顺组是展示层的事，不进这里。

use manifold_csg::Manifold;
use std::io::{self, Write};

/// 共享顶点的三角网格，坐标 mm。
#[derive(Debug, Clone, Default)]
pub struct TriMesh {
    pub vertices: Vec<[f64; 3]>,
    /// 每三个一组，右手绕向朝外。
    pub indices: Vec<u32>,
}

impl TriMesh {
    pub fn from_manifold(solid: &Manifold) -> Self {
        let (props, n_props, tri) = solid.to_mesh_f64();
        if n_props < 3 || props.is_empty() {
            return Self::default();
        }
        let vert_count = props.len() / n_props;
        let mut vertices = Vec::with_capacity(vert_count);
        for i in 0..vert_count {
            let base = i * n_props;
            vertices.push([props[base], props[base + 1], props[base + 2]]);
        }
        let indices = tri.iter().map(|&i| i as u32).collect();
        Self { vertices, indices }
    }

    /// Concatenate closed parts without a boolean union.  E3D keeps CLFL
    /// catalogue primitives as separate RVM components even when they touch.
    pub fn from_manifolds(solids: &[Manifold]) -> Self {
        let mut out = Self::default();
        for solid in solids {
            let part = Self::from_manifold(solid);
            let base = out.vertices.len() as u32;
            out.vertices.extend(part.vertices);
            out.indices
                .extend(part.indices.into_iter().map(|index| index + base));
        }
        out
    }

    pub fn triangle_count(&self) -> usize {
        self.indices.len() / 3
    }

    pub fn aabb(&self) -> Option<([f64; 3], [f64; 3])> {
        let first = self.vertices.first()?;
        let mut lo = *first;
        let mut hi = *first;
        for v in &self.vertices {
            for axis in 0..3 {
                lo[axis] = lo[axis].min(v[axis]);
                hi[axis] = hi[axis].max(v[axis]);
            }
        }
        Some((lo, hi))
    }

    /// 以 OBJ object 的形式追加写出。`index_offset` 是此前已写出的顶点数
    /// （OBJ 索引全局连续、1 起），返回累加后的新偏移。
    pub fn write_obj_object(
        &self,
        w: &mut impl Write,
        name: &str,
        index_offset: u32,
    ) -> io::Result<u32> {
        writeln!(w, "o {name}")?;
        for v in &self.vertices {
            writeln!(w, "v {} {} {}", v[0], v[1], v[2])?;
        }
        for face in self.indices.as_chunks::<3>().0 {
            writeln!(
                w,
                "f {} {} {}",
                face[0] + 1 + index_offset,
                face[1] + 1 + index_offset,
                face[2] + 1 + index_offset
            )?;
        }
        Ok(index_offset + self.vertices.len() as u32)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cube_round_trips_with_invariants() {
        let solid = Manifold::cube(10.0, 20.0, 30.0, true);
        let mesh = TriMesh::from_manifold(&solid);
        assert_eq!(mesh.triangle_count(), 12);
        let (lo, hi) = mesh.aabb().unwrap();
        assert_eq!(lo, [-5.0, -10.0, -15.0]);
        assert_eq!(hi, [5.0, 10.0, 15.0]);
    }

    #[test]
    fn obj_output_is_one_based_and_offsets_accumulate() {
        let mesh = TriMesh {
            vertices: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            indices: vec![0, 1, 2],
        };
        let mut buf = Vec::new();
        let next = mesh.write_obj_object(&mut buf, "a", 0).unwrap();
        assert_eq!(next, 3);
        let text = String::from_utf8(buf).unwrap();
        assert!(text.contains("o a\n"));
        assert!(text.contains("f 1 2 3\n"));

        let mut buf2 = Vec::new();
        let next2 = mesh.write_obj_object(&mut buf2, "b", next).unwrap();
        assert_eq!(next2, 6);
        let text2 = String::from_utf8(buf2).unwrap();
        assert!(text2.contains("f 4 5 6\n"), "{text2}");
    }

    #[test]
    fn concatenation_preserves_overlapping_parts_without_boolean_union() {
        let a = Manifold::cube(10.0, 10.0, 10.0, true);
        let b = a.translate(5.0, 0.0, 0.0);
        let mesh = TriMesh::from_manifolds(&[a, b]);
        assert_eq!(mesh.triangle_count(), 24);
        assert_eq!(mesh.aabb(), Some(([-5.0, -5.0, -5.0], [10.0, 5.0, 5.0])));
    }
}

//! POLYHE/POHE（正）与 NPOLYH（负）多面体：显式 B-rep → 三角网格 → Manifold。
//!
//! 对标 Core3D `CSG_BasicPOL::getPrimGeom`(0x10726f10) → `sub_1071DCF0`（活桥反编译，
//! 7UW4 档案 §3.7.4）：两趟扫成员——趟 1 收顶点（`gm_AddVertexToPolyhedron` 按
//! DB_Ref 去重入库），趟 2 逐面 `gm_AddFacetToPolyhedron` + 逐边
//! `gm_AddSideToFacetOfPolyhedron`，最后 `gm_ValidateObject` 校验闭合。
//! 三种非标量图元（EXT/POL/REV）里唯一**不过 D2_Profile** 的一种。本 crate 的等价物：
//!
//! - 「顶点按 DB_Ref 去重」→ 全局顶点表按**坐标位模式**去重（现代变体共享 POIN
//!   本就同源；旧式变体同一角点的存储值逐位相同）。两张面「共享」顶点靠它拼水密。
//! - 「面法向反转标志 + 硬/软边类型」→ Core3D 用面上的反转标志统一各面绕向
//!   （§3.7.4 的 `rev`，属性码 10458597/725013 是现代路内联枚举、名字未映射）。
//!   本 crate 不依赖该标志，改用**几何一致化**：共享边绕向传播 + 分量级有向体积
//!   翻正——gen-model `tessellate_polyhedron`/`orient_outward` 只做体积翻正就够
//!   （渲染网格容忍混绕向），manifold 内核要求正确定向的封闭壳，必须补传播。
//! - 「gm_ValidateObject」→ [`Manifold::from_mesh_f64`] 的流形校验（非闭合/非流形报错）。
//!
//! 数据结构两变体（gen-model `fast_model/prim_model.rs` 的权威读法）：
//!
//! - **现代 POLYHE**：首成员 `POLPTL`（顶点表，`POIN.POS`），其余成员为面（POGON），
//!   面下 `POLOOP` 环（首环外边界、其余是孔），环下 `LOOPTS.VXREF`（顶点引用数组，
//!   同一环多条 LOOPTS 串接）。对应 §3.7.4 的 noun 码 183671242（顶点组）/44236870（面）。
//! - **旧式 POHE**：成员即面，面成员即顶点（读 `POS`），每面单环无孔。
//!
//! ⚠ ams1112 无 POLYHE/POHE/NPOLYH 元素，本模块目前只有解析用例兜底（立方/垫圈/
//! 翻面自愈/非闭合报错）；出现真数据后按 RVM 对拍再核（7UW4 §3.7.6 待补项）。

use std::collections::{HashMap, VecDeque};

use glam::DVec3;
use manifold_csg::Manifold;

use e3d_io::db_element::{DbElement, DbElementError};
use e3d_io::refno::RefNo;

/// 一张平面面片：`loops[0]` 外环，其余是孔（Core3D 面下多 POLOOP 同义）。
#[derive(Debug, Clone)]
pub struct PolyFace {
    pub loops: Vec<Vec<DVec3>>,
}

#[derive(Debug, thiserror::Error)]
pub enum PolyhedronError {
    #[error(transparent)]
    Db(#[from] DbElementError),
    #[error("{refno}（{noun}）下没有任何面，构不成多面体")]
    NoFaces { refno: String, noun: String },
    #[error("多面体 {faces} 张面一张都没剖出三角形")]
    NoTriangles { faces: usize },
    #[error("多面体面片拼不成封闭流形：{detail}")]
    NotManifold { detail: String },
}

/// 面积门（mm²）：与 §3.7 轮廓路的 `|area| ≥ 1e-6` 同一量级——低于它的面
/// 对实体没有贡献，跳过并记账。
const MIN_FACE_AREA_MM2: f64 = 1e-6;

/// `UnknownAttribute` 与未设一视同仁（`profile::loop_profile` 同一口径）。
fn position_or_none(el: &DbElement) -> Result<Option<[f64; 3]>, DbElementError> {
    match el.get_position("POS") {
        Ok(v) => Ok(v),
        Err(DbElementError::UnknownAttribute { .. }) => Ok(None),
        Err(e) => Err(e),
    }
}

fn vxrefs_or_empty(el: &DbElement) -> Result<Vec<RefNo>, DbElementError> {
    match el.get_ref_array("VXREF") {
        Ok(v) => Ok(v.unwrap_or_default()),
        Err(DbElementError::UnknownAttribute { .. }) => Ok(Vec::new()),
        Err(e) => Err(e),
    }
}

/// 从 POHE/POLYHE/NPOLYH 元素采集面片（两变体，见模块文档）。
///
/// 成员一律文件原序遍历（与 Core3D 的 NXTITM 迭代同序）。空环/空面丢弃；
/// 一张面都收不到是硬错误（数据坏，响亮）。
pub fn polyhedron_faces(el: &DbElement) -> Result<Vec<PolyFace>, PolyhedronError> {
    let members: Vec<DbElement> = el.members()?.collect();
    let modern = match members.first() {
        Some(first) => first.element_type()? == "POLPTL",
        None => false,
    };

    let mut faces = Vec::new();
    if modern {
        // 趟 1（对标 §3.7.4）：POLPTL 的 POIN 成员建 RefNo → 位置 索引。
        // POS 未设按 dabacon 缺省语义取 0（与 `loop_profile` 同款）。
        let mut verts: HashMap<RefNo, DVec3> = HashMap::new();
        for poin in members[0].members()? {
            if poin.element_type()? != "POIN" {
                continue;
            }
            let pos = position_or_none(&poin)?.unwrap_or([0.0; 3]);
            verts.insert(poin.refno(), DVec3::from_array(pos));
        }
        // 趟 2：其余成员即面（POGON），面下 POLOOP 环经 LOOPTS.VXREF 解引用。
        for face_el in &members[1..] {
            let mut loops = Vec::new();
            collect_poloop_rings(face_el, &verts, &mut loops)?;
            if !loops.is_empty() {
                faces.push(PolyFace { loops });
            }
        }
    } else {
        // 旧式 POHE：成员即面，面成员即顶点。没有 POS 的成员不是顶点，跳过。
        for face_el in &members {
            let mut ring = Vec::new();
            for vertex in face_el.members()? {
                if let Some(pos) = position_or_none(&vertex)? {
                    ring.push(DVec3::from_array(pos));
                }
            }
            if ring.len() >= 3 {
                faces.push(PolyFace { loops: vec![ring] });
            }
        }
    }

    if faces.is_empty() {
        return Err(PolyhedronError::NoFaces {
            refno: el.refno().to_string(),
            noun: el.element_type()?,
        });
    }
    Ok(faces)
}

/// 递归收面下的 POLOOP 环：环顶点 = 其 LOOPTS 成员的 VXREF 串接，经顶点表解引用。
/// gen-model 按 owner 分组的同一读法；递归是为容忍中间多一层包装元素。
fn collect_poloop_rings(
    el: &DbElement,
    verts: &HashMap<RefNo, DVec3>,
    out: &mut Vec<Vec<DVec3>>,
) -> Result<(), PolyhedronError> {
    for member in el.members()? {
        if member.element_type()? == "POLOOP" {
            let mut ring = Vec::new();
            for loopts in member.members()? {
                if loopts.element_type()? != "LOOPTS" {
                    continue;
                }
                for refno in vxrefs_or_empty(&loopts)? {
                    if let Some(p) = verts.get(&refno) {
                        ring.push(*p);
                    }
                }
            }
            if ring.len() >= 3 {
                out.push(ring);
            }
        } else {
            collect_poloop_rings(&member, verts, out)?;
        }
    }
    Ok(())
}

/// 一张面三角化后的产物：全局索引三角形 + 原始环的有向边（绕向传播用）。
struct FaceMesh {
    tris: Vec<[u32; 3]>,
    edges: Vec<(u32, u32)>,
}

/// 面片组 → 封闭实体。
///
/// 逐面：Newell 法向 → 面内 (u,v) 投影 → earcut 带孔剖分 → 出边绕向逐个对齐
/// 到 Newell 法向（gen-model `append_planar_face` 同款）。全局：坐标位模式去重
/// 顶点 → 共享边绕向传播 + 分量级有向体积翻正 → [`Manifold::from_mesh_f64`]
/// 流形校验（= `gm_ValidateObject` 的位置）。
///
/// 退化面（外环不足 3 点 / 面积低于 [`MIN_FACE_AREA_MM2`] / 剖分失败）跳过并记
/// `notes`，由调用方入账；面片拼不成封闭壳是硬错误。
pub fn polyhedron_solid(
    faces: &[PolyFace],
    notes: &mut Vec<String>,
) -> Result<Manifold, PolyhedronError> {
    let mut verts: Vec<DVec3> = Vec::new();
    let mut index_of: HashMap<[u64; 3], u32> = HashMap::new();
    let mut face_meshes: Vec<FaceMesh> = Vec::new();

    for (fi, face) in faces.iter().enumerate() {
        // 环 → 全局索引，去掉相邻重复与闭合重复点。
        let mut rings: Vec<Vec<u32>> = Vec::new();
        for loop_pts in &face.loops {
            let mut ring: Vec<u32> = Vec::new();
            for &p in loop_pts {
                let idx = intern(p, &mut verts, &mut index_of);
                if ring.last() == Some(&idx) {
                    continue;
                }
                ring.push(idx);
            }
            while ring.len() >= 2 && ring.first() == ring.last() {
                ring.pop();
            }
            rings.push(ring);
        }
        if rings.first().is_none_or(|r| r.len() < 3) {
            notes.push(format!("多面体面 {fi} 外环不足 3 个有效顶点，跳过"));
            continue;
        }
        let outer_pts: Vec<DVec3> = rings[0].iter().map(|&i| verts[i as usize]).collect();
        let Some(normal) = newell_normal(&outer_pts) else {
            notes.push(format!(
                "多面体面 {fi} 面积低于 {MIN_FACE_AREA_MM2}mm²（法向退化），跳过"
            ));
            continue;
        };
        // 面内二维基：拿一根与法向最不平行的坐标轴叉出来，避免退化
        // （gen-model `append_planar_face` 同款）。(u, v, normal) 右手系，
        // 绕法向逆时针的环投影后仍逆时针。
        let helper = if normal.x.abs() < 0.9 {
            DVec3::X
        } else {
            DVec3::Y
        };
        let u = helper.cross(normal).normalize();
        let v = normal.cross(u);
        let origin = verts[rings[0][0] as usize];

        let mut flat: Vec<f64> = Vec::new();
        let mut hole_starts: Vec<usize> = Vec::new();
        let mut local_to_global: Vec<u32> = Vec::new();
        for (ri, ring) in rings.iter().enumerate() {
            if ring.len() < 3 {
                if ri > 0 {
                    notes.push(format!("多面体面 {fi} 孔环 {ri} 不足 3 点，丢弃"));
                }
                continue;
            }
            if ri > 0 {
                hole_starts.push(flat.len() / 2);
            }
            for &gi in ring {
                let d = verts[gi as usize] - origin;
                flat.push(d.dot(u));
                flat.push(d.dot(v));
                local_to_global.push(gi);
            }
        }

        let Ok(tri_ids) = earcutr::earcut(&flat, &hole_starts, 2) else {
            notes.push(format!("多面体面 {fi} 剖分失败，跳过"));
            continue;
        };
        if tri_ids.len() < 3 {
            notes.push(format!("多面体面 {fi} 剖不出三角形，跳过"));
            continue;
        }

        let mut tris = Vec::with_capacity(tri_ids.len() / 3);
        for t in tri_ids.as_chunks::<3>().0 {
            let (ga, gb, gc) = (
                local_to_global[t[0]],
                local_to_global[t[1]],
                local_to_global[t[2]],
            );
            let (a, b, c) = (verts[ga as usize], verts[gb as usize], verts[gc as usize]);
            // earcut 的出边绕向跟着输入环走，逐个对齐到 Newell 法向才靠得住
            // （gen-model 同款）——对齐后整张面与它存储的外环绕向一致。
            if (b - a).cross(c - a).dot(normal) >= 0.0 {
                tris.push([ga, gb, gc]);
            } else {
                tris.push([ga, gc, gb]);
            }
        }
        let mut edges = Vec::new();
        for ring in &rings {
            if ring.len() < 3 {
                continue;
            }
            for k in 0..ring.len() {
                let a = ring[k];
                let b = ring[(k + 1) % ring.len()];
                if a != b {
                    edges.push((a, b));
                }
            }
        }
        face_meshes.push(FaceMesh { tris, edges });
    }

    if face_meshes.is_empty() {
        return Err(PolyhedronError::NoTriangles { faces: faces.len() });
    }

    // Core3D 靠面上的法向反转标志统一各面绕向；这里等价地从几何推：
    // 定向一致 ⇔ 相邻两面沿共享边方向相反。
    let flips = propagate_orientation(&face_meshes, &verts);

    // 汇总出网格（只收真正被三角形引用的顶点，未用点不进流形）。
    let mut compact: HashMap<u32, u64> = HashMap::new();
    let mut props: Vec<f64> = Vec::new();
    let mut tri_indices: Vec<u64> = Vec::new();
    for (fi, fm) in face_meshes.iter().enumerate() {
        for t in &fm.tris {
            for gi in tri_with_flip(t, flips[fi]) {
                let next = compact.len() as u64;
                let ci = *compact.entry(gi).or_insert_with(|| {
                    let p = verts[gi as usize];
                    props.extend_from_slice(&[p.x, p.y, p.z]);
                    next
                });
                tri_indices.push(ci);
            }
        }
    }

    let solid = Manifold::from_mesh_f64(&props, 3, &tri_indices).map_err(|e| {
        PolyhedronError::NotManifold {
            detail: e.to_string(),
        }
    })?;
    if solid.is_empty() || solid.num_tri() == 0 {
        return Err(PolyhedronError::NotManifold {
            detail: "生成的流形为空".into(),
        });
    }
    Ok(solid)
}

fn intern(p: DVec3, verts: &mut Vec<DVec3>, index_of: &mut HashMap<[u64; 3], u32>) -> u32 {
    *index_of
        .entry([p.x.to_bits(), p.y.to_bits(), p.z.to_bits()])
        .or_insert_with(|| {
            verts.push(p);
            (verts.len() - 1) as u32
        })
}

fn tri_with_flip(t: &[u32; 3], flip: bool) -> [u32; 3] {
    if flip { [t[0], t[2], t[1]] } else { *t }
}

/// Newell 法向（模 = 2×面积）；面积低于 [`MIN_FACE_AREA_MM2`] 判退化。
fn newell_normal(ring: &[DVec3]) -> Option<DVec3> {
    let mut n = DVec3::ZERO;
    for i in 0..ring.len() {
        let a = ring[i];
        let b = ring[(i + 1) % ring.len()];
        n += DVec3::new(
            (a.y - b.y) * (a.z + b.z),
            (a.z - b.z) * (a.x + b.x),
            (a.x - b.x) * (a.y + b.y),
        );
    }
    (n.length() > 2.0 * MIN_FACE_AREA_MM2).then(|| n.normalize())
}

/// 共享边绕向传播 + 分量级有向体积翻正，返回每张面要不要整面翻转。
///
/// - 邻接只信**恰好两用**的边（>2 用是非流形边，留给 `from_mesh_f64` 报）。
/// - 两面沿共享边方向**相同** ⇒ 绕向冲突 ⇒ 翻转关系取反（BFS 传播）。
/// - 每个连通分量各自按有向体积翻正（gen-model `orient_outward` 的思路按分量做，
///   多壳数据各壳自翻）。
fn propagate_orientation(faces: &[FaceMesh], verts: &[DVec3]) -> Vec<bool> {
    let mut edge_uses: HashMap<(u32, u32), Vec<(usize, bool)>> = HashMap::new();
    for (fi, f) in faces.iter().enumerate() {
        for &(a, b) in &f.edges {
            let key = (a.min(b), a.max(b));
            edge_uses.entry(key).or_default().push((fi, a < b));
        }
    }
    let mut adjacency: Vec<Vec<(usize, bool)>> = vec![Vec::new(); faces.len()];
    for uses in edge_uses.values() {
        if let [(f1, d1), (f2, d2)] = uses[..]
            && f1 != f2
        {
            let same_dir = d1 == d2;
            adjacency[f1].push((f2, same_dir));
            adjacency[f2].push((f1, same_dir));
        }
    }

    let mut flip = vec![false; faces.len()];
    let mut seen = vec![false; faces.len()];
    for seed in 0..faces.len() {
        if seen[seed] {
            continue;
        }
        seen[seed] = true;
        let mut component = vec![seed];
        let mut queue = VecDeque::from([seed]);
        while let Some(cur) = queue.pop_front() {
            for &(next, same_dir) in &adjacency[cur] {
                if seen[next] {
                    continue;
                }
                seen[next] = true;
                flip[next] = flip[cur] ^ same_dir;
                component.push(next);
                queue.push_back(next);
            }
        }
        let mut vol6 = 0.0;
        for &fi in &component {
            for t in &faces[fi].tris {
                let [ia, ib, ic] = tri_with_flip(t, flip[fi]);
                let (a, b, c) = (verts[ia as usize], verts[ib as usize], verts[ic as usize]);
                vol6 += a.cross(b).dot(c);
            }
        }
        if vol6 < 0.0 {
            for &fi in &component {
                flip[fi] = !flip[fi];
            }
        }
    }
    flip
}

#[cfg(test)]
mod tests {
    use super::*;

    fn face(points: &[[f64; 3]]) -> PolyFace {
        PolyFace {
            loops: vec![points.iter().map(|&p| DVec3::from_array(p)).collect()],
        }
    }

    /// 6 张面一致外向的立方 [0,s]³。
    fn cube(s: f64) -> Vec<PolyFace> {
        vec![
            face(&[[0., 0., 0.], [0., s, 0.], [s, s, 0.], [s, 0., 0.]]), // z=0，外向 −Z
            face(&[[0., 0., s], [s, 0., s], [s, s, s], [0., s, s]]),     // z=s，外向 +Z
            face(&[[0., 0., 0.], [s, 0., 0.], [s, 0., s], [0., 0., s]]), // y=0，外向 −Y
            face(&[[0., s, 0.], [0., s, s], [s, s, s], [s, s, 0.]]),     // y=s，外向 +Y
            face(&[[0., 0., 0.], [0., 0., s], [0., s, s], [0., s, 0.]]), // x=0，外向 −X
            face(&[[s, 0., 0.], [s, s, 0.], [s, s, s], [s, 0., s]]),     // x=s，外向 +X
        ]
    }

    #[test]
    fn a_six_face_cube_closes_into_the_exact_volume() {
        let mut notes = Vec::new();
        let solid = polyhedron_solid(&cube(2.0), &mut notes).unwrap();
        assert!(notes.is_empty(), "{notes:?}");
        assert!((solid.volume() - 8.0).abs() < 1e-9, "{}", solid.volume());
        assert_eq!(solid.genus(), 0);
        let bb = solid.bounding_box().unwrap();
        assert!(bb.min().iter().all(|&v| v.abs() < 1e-12));
        assert!(bb.max().iter().all(|&v| (v - 2.0).abs() < 1e-12));
    }

    /// 面绕向混乱的数据必须能自愈：Core3D 靠面上的反转标志纠绕向（§3.7.4），
    /// 本实现靠共享边传播——随便翻两张面，体积照旧。
    #[test]
    fn mixed_face_winding_is_healed_by_propagation() {
        let mut faces = cube(2.0);
        faces[1].loops[0].reverse();
        faces[4].loops[0].reverse();
        let mut notes = Vec::new();
        let solid = polyhedron_solid(&faces, &mut notes).unwrap();
        assert!((solid.volume() - 8.0).abs() < 1e-9, "{}", solid.volume());
        assert_eq!(solid.genus(), 0);
    }

    /// 带孔面（外环 + 孔环，= 面下多 POLOOP）：方垫圈，外 10×10、孔 4×4、高 5。
    /// 体积 (100 − 16) × 5 = 420，亏格 1。
    #[test]
    fn a_washer_with_hole_loops_builds_a_genus_one_solid() {
        let (s, hole0, hole1, h) = (10.0, 3.0, 7.0, 5.0);
        let bottom = PolyFace {
            loops: vec![
                vec![
                    DVec3::new(0., 0., 0.),
                    DVec3::new(0., s, 0.),
                    DVec3::new(s, s, 0.),
                    DVec3::new(s, 0., 0.),
                ],
                vec![
                    DVec3::new(hole0, hole0, 0.),
                    DVec3::new(hole1, hole0, 0.),
                    DVec3::new(hole1, hole1, 0.),
                    DVec3::new(hole0, hole1, 0.),
                ],
            ],
        };
        let top = PolyFace {
            loops: vec![
                vec![
                    DVec3::new(0., 0., h),
                    DVec3::new(s, 0., h),
                    DVec3::new(s, s, h),
                    DVec3::new(0., s, h),
                ],
                vec![
                    DVec3::new(hole0, hole0, h),
                    DVec3::new(hole0, hole1, h),
                    DVec3::new(hole1, hole1, h),
                    DVec3::new(hole1, hole0, h),
                ],
            ],
        };
        let mut faces = vec![bottom, top];
        // 4 张外侧壁。
        faces.push(face(&[[0., 0., 0.], [s, 0., 0.], [s, 0., h], [0., 0., h]]));
        faces.push(face(&[[s, 0., 0.], [s, s, 0.], [s, s, h], [s, 0., h]]));
        faces.push(face(&[[s, s, 0.], [0., s, 0.], [0., s, h], [s, s, h]]));
        faces.push(face(&[[0., s, 0.], [0., 0., 0.], [0., 0., h], [0., s, h]]));
        // 4 张孔道内壁。
        faces.push(face(&[
            [hole0, hole0, 0.],
            [hole0, hole1, 0.],
            [hole0, hole1, h],
            [hole0, hole0, h],
        ]));
        faces.push(face(&[
            [hole0, hole1, 0.],
            [hole1, hole1, 0.],
            [hole1, hole1, h],
            [hole0, hole1, h],
        ]));
        faces.push(face(&[
            [hole1, hole1, 0.],
            [hole1, hole0, 0.],
            [hole1, hole0, h],
            [hole1, hole1, h],
        ]));
        faces.push(face(&[
            [hole1, hole0, 0.],
            [hole0, hole0, 0.],
            [hole0, hole0, h],
            [hole1, hole0, h],
        ]));

        let mut notes = Vec::new();
        let solid = polyhedron_solid(&faces, &mut notes).unwrap();
        let expect = (s * s - (hole1 - hole0) * (hole1 - hole0)) * h;
        assert!(
            (solid.volume() - expect).abs() < 1e-9,
            "体积 {}，期望 {expect}",
            solid.volume()
        );
        assert_eq!(solid.genus(), 1, "带贯穿孔的垫圈应是亏格 1");
    }

    /// 缺一张面 = 壳不闭合，必须响亮报错（= `gm_ValidateObject` 不过就
    /// `gm_RemoveObject` 的口径），不许出一个悄悄漏水的体。
    #[test]
    fn an_open_shell_is_a_loud_error() {
        let mut faces = cube(2.0);
        faces.pop();
        let mut notes = Vec::new();
        let result = polyhedron_solid(&faces, &mut notes);
        assert!(
            matches!(result, Err(PolyhedronError::NotManifold { .. })),
            "开壳必须报 NotManifold"
        );
    }

    /// 全是退化面（共线环）→ NoTriangles，退化明细进 notes。
    #[test]
    fn all_degenerate_faces_report_no_triangles() {
        let degenerate = face(&[[0., 0., 0.], [1., 0., 0.], [2., 0., 0.]]);
        let mut notes = Vec::new();
        let result = polyhedron_solid(&[degenerate], &mut notes);
        assert!(matches!(
            result,
            Err(PolyhedronError::NoTriangles { faces: 1 })
        ));
        assert_eq!(notes.len(), 1, "{notes:?}");
    }
}

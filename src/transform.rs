//! POS/ORI → 矩阵折叠（对标 `MAT16`/`CONCAT`/`GLSMAT`）。
//!
//! 语义与 gen-model DB 模式的 `get_world_mat4_impl` 对齐：
//! - `POS`：毫米平移，缺省零。
//! - `ORI`：三个角（度），旋转 = `Rz(ori[2]) * Ry(ori[1]) * Rx(ori[0])`
//!   （aios-core `angles_to_ori` 同款，别自作聪明换序）。
//! - 世界矩阵 = 从根到自身逐级右乘局部矩阵。
//!
//! 本期不做的（记账在此，免得后人以为是漏）：GENSEC 的 BANG 隐藏方位、
//! SPINE/ZDIR 样条路径特判——1112 首里程碑的结构件不吃这两条。

use e3d_io::db_element::{DbElement, DbElementError};
use glam::{DMat3, DMat4, DVec3};

/// 上溯 owner 链的硬上限。dabacon 树深实测个位数，128 只可能是环。
const MAX_ANCESTOR_DEPTH: usize = 128;

/// 读三元组属性，属性不在该 noun 模板里视同缺省（None）；
/// 形状不对（TypeMismatch）照旧上浮——那是数据坏了，不是缺省。
fn triple_or_none(el: &DbElement, name: &str) -> Result<Option<[f64; 3]>, DbElementError> {
    match el.get_position(name) {
        Ok(v) => Ok(v),
        Err(DbElementError::UnknownAttribute { .. }) => Ok(None),
        Err(e) => Err(e),
    }
}

/// `(POS, ORI)` 原样三元组对，任一缺省时该项为 `None`。
pub type LocalPlacement = (Option<[f64; 3]>, Option<[f64; 3]>);

/// 一个元素进 [`local_matrix`] 的全部输入：`(POS, ORI)` 原样三元组。
///
/// 增量更新用它判「变换漂移」——两个会话读出的这一对不相等，子树里全部
/// 已建模元素的世界矩阵就都变了。判据必须与矩阵折叠读同一条路（同名、
/// 同缺省语义），否则会出现「矩阵变了但漂移检测说没变」的静默缺件。
pub fn local_placement(el: &DbElement) -> Result<LocalPlacement, DbElementError> {
    Ok((triple_or_none(el, "POS")?, triple_or_none(el, "ORI")?))
}

/// ORI 角三元组（度）→ 旋转矩阵。
pub fn ori_matrix(ori_deg: [f64; 3]) -> DMat3 {
    DMat3::from_rotation_z(ori_deg[2].to_radians())
        * DMat3::from_rotation_y(ori_deg[1].to_radians())
        * DMat3::from_rotation_x(ori_deg[0].to_radians())
}

/// 单个元素相对属主的局部矩阵：`T(POS) * R(ORI)`。两者都缺省时是单位阵。
pub fn local_matrix(el: &DbElement) -> Result<DMat4, DbElementError> {
    let (pos, ori) = local_placement(el)?;
    let rot = ori_matrix(ori.unwrap_or([0.0; 3]));
    Ok(DMat4::from_translation(DVec3::from_array(pos.unwrap_or([0.0; 3]))) * DMat4::from_mat3(rot))
}

/// 世界矩阵：owner 链折叠（根 → 自身逐级右乘）。
pub fn world_matrix(el: &DbElement) -> Result<DMat4, DbElementError> {
    let mut chain = vec![local_matrix(el)?];
    let mut cursor = el.owner()?;
    while let Some(current) = cursor {
        if chain.len() > MAX_ANCESTOR_DEPTH {
            return Err(DbElementError::OwnerCycle {
                start: el.refno(),
                at: current.refno(),
            });
        }
        chain.push(local_matrix(&current)?);
        cursor = current.owner()?;
    }
    let mut world = DMat4::IDENTITY;
    for local in chain.iter().rev() {
        world *= *local;
    }
    Ok(world)
}

/// `descendant` 相对 `ancestor` 局部系的矩阵：沿 owner 链从 ancestor 的下一级
/// 折叠到 descendant。descendant 不在 ancestor 子树里 = 数据错，响亮报。
pub fn matrix_relative_to(
    descendant: &DbElement,
    ancestor: &DbElement,
) -> Result<DMat4, DbElementError> {
    let mut chain = vec![local_matrix(descendant)?];
    let mut cursor = descendant.owner()?;
    loop {
        let Some(current) = cursor else {
            return Err(DbElementError::NotFound {
                refno: descendant.refno(),
                dbno: ancestor.db_no(),
                sesno: 0,
            });
        };
        if current.refno() == ancestor.refno() {
            break;
        }
        if chain.len() > MAX_ANCESTOR_DEPTH {
            return Err(DbElementError::OwnerCycle {
                start: descendant.refno(),
                at: current.refno(),
            });
        }
        chain.push(local_matrix(&current)?);
        cursor = current.owner()?;
    }
    let mut rel = DMat4::IDENTITY;
    for local in chain.iter().rev() {
        rel *= *local;
    }
    Ok(rel)
}

/// glam 列主序 4×4 → manifold-csg 的 4×3 仿射（列主序，末列平移）。
pub fn dmat4_to_affine4x3(m: DMat4) -> [f64; 12] {
    [
        m.x_axis.x, m.x_axis.y, m.x_axis.z, m.y_axis.x, m.y_axis.y, m.y_axis.z, m.z_axis.x,
        m.z_axis.y, m.z_axis.z, m.w_axis.x, m.w_axis.y, m.w_axis.z,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ori_rotation_order_is_z_then_y_then_x() {
        // ORI = [0, 0, 90]：绕 Z 转 90°，X 轴应指向 +Y。
        let m = ori_matrix([0.0, 0.0, 90.0]);
        let x = m * DVec3::X;
        assert!((x - DVec3::Y).length() < 1e-9, "{x:?}");

        // 复合序：矩阵右乘，Rx 先作用。ORI=[90,90,0] 作用在 Z 上：
        // Rx(90) 把 Z 转到 -Y；Ry(90) 不动 -Y；Rz(0) 不动。
        let m = ori_matrix([90.0, 90.0, 0.0]);
        let v = m * DVec3::Z;
        assert!((v - DVec3::NEG_Y).length() < 1e-9, "{v:?}");
    }

    #[test]
    fn affine4x3_translation_in_last_column() {
        let a = dmat4_to_affine4x3(DMat4::from_translation(DVec3::new(1.0, 2.0, 3.0)));
        assert_eq!(&a[9..], &[1.0, 2.0, 3.0]);
        assert_eq!(a[0], 1.0);
        assert_eq!(a[4], 1.0);
        assert_eq!(a[8], 1.0);
    }

    /// 权威对齐：`ori_matrix` 必须与 e3d-io 从 TTY 样本逆向的
    /// `record::orientation::axes`（Y/Z 轴向量）逐例一致。
    /// 那份实现是 ORI 语义的取证源，这里只是它的矩阵形态。
    #[test]
    fn ori_matrix_agrees_with_e3d_io_orientation_axes() {
        let samples: [[f64; 3]; 8] = [
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 32.0],
            [0.0, 0.0, -22.5209],
            [0.17651, 0.0, 0.0],
            [0.0, 90.0, 100.0],
            [180.0, 0.0, 180.0],
            [90.0, 90.0, 0.0],
            [12.3, -45.6, 78.9],
        ];
        for angles in samples {
            let (y_axis, z_axis) = e3d_io::record::orientation::axes(angles);
            let m = ori_matrix(angles);
            let y = m * DVec3::Y;
            let z = m * DVec3::Z;
            assert!(
                (y - DVec3::from_array(y_axis)).length() < 1e-9,
                "ORI {angles:?}: Y 轴 {y:?} != 权威 {y_axis:?}"
            );
            assert!(
                (z - DVec3::from_array(z_axis)).length() < 1e-9,
                "ORI {angles:?}: Z 轴 {z:?} != 权威 {z_axis:?}"
            );
        }
    }
}

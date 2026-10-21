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
//!
//! 唯一一处不读存储属性的局部矩阵：电缆桥架件（[`crate::cable_tray`] 那七个 noun）
//! 的模板里没有 POS/ORI，E3D 把它们注册成伪属性现算。[`local_matrix`] 对这些 noun
//! 改走 [`crate::cable_tray::placement_matrix`]，解不出来就带原因报错——托盘落在
//! 属主原点是静默缺件，不能用单位阵兜。

use e3d_io::db_element::{DbElement, DbElementError};
use e3d_io::refno::RefNo;
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

/// [`local_matrix`] 会读的**全部存储属性名**，按 [`LocalPlacement`] 的次序。
///
/// 增量的级联判据要回答「这个元素一动，子树里的世界矩阵变不变」，而它必须与
/// 这里读的是同一组名字：矩阵折叠多读一个属性、级联判据不知道，表现就是
/// 「矩阵变了但没人重建」的静默缺件。所以名字只有这一份，两边都从这里取。
///
/// 世界矩阵的输入不止这两个属性——还有 **owner 链本身**（[`world_matrix`] 逐级
/// 上溯右乘）。改挂属主同样让世界矩阵变，那一条不是属性名能表达的，由
/// [`crate::element_diff::ElementDiff::placement_input_changed`] 一并判。
///
/// **桥架件是例外**（[`crate::cable_tray`]）：它们的局部矩阵不读这两个名字，读的是
/// `POSREF/ZDIS/PKDI/ZOFF/BANG/GANGLE` 加 `CWBRAN`/`RPATH`/`POINTR` 那一片兄弟子树——
/// 跨元素的依赖，这张表表达不了。增量对它们另有一条腿：候选落在 `CWBRAN` 子树里就把
/// 整条分支级联重建（`increment::cable_tray_branch_of`），不靠这张表。
pub const PLACEMENT_ATTRIBUTES: [&str; 2] = ["POS", "ORI"];

/// 一个元素进 [`local_matrix`] 的全部输入：`(POS, ORI)` 原样三元组。
pub fn local_placement(el: &DbElement) -> Result<LocalPlacement, DbElementError> {
    let [pos, ori] = PLACEMENT_ATTRIBUTES;
    Ok((triple_or_none(el, pos)?, triple_or_none(el, ori)?))
}

/// ORI 角三元组（度）→ 旋转矩阵。
pub fn ori_matrix(ori_deg: [f64; 3]) -> DMat3 {
    DMat3::from_rotation_z(ori_deg[2].to_radians())
        * DMat3::from_rotation_y(ori_deg[1].to_radians())
        * DMat3::from_rotation_x(ori_deg[0].to_radians())
}

/// 单个元素相对属主的局部矩阵：`T(POS) * R(ORI)`。两者都缺省时是单位阵。
///
/// 桥架件例外：走 [`crate::cable_tray::placement_matrix`] 现算（见模块注释）。
pub fn local_matrix(el: &DbElement) -> Result<DMat4, DbElementError> {
    if crate::cable_tray::is_cable_tray_element(&el.element_type()?) {
        return crate::cable_tray::placement_matrix(el);
    }
    let (pos, ori) = local_placement(el)?;
    let rot = ori_matrix(ori.unwrap_or([0.0; 3]));
    Ok(DMat4::from_translation(DVec3::from_array(pos.unwrap_or([0.0; 3]))) * DMat4::from_mat3(rot))
}

/// 世界矩阵：owner 链折叠（根 → 自身逐级右乘）。
///
/// 链上遇到标架由属主现算的元素（[`ImplicitFrame`]）就在那一级截住，改用它现算的世界
/// 标架——这些元素没有 POS/ORI，逐级右乘只会乘进单位阵。
pub fn world_matrix(el: &DbElement) -> Result<DMat4, DbElementError> {
    let _timing = crate::perf::world_matrix_scope();
    let mut chain = Vec::new();
    let mut world = DMat4::IDENTITY;
    let mut cursor = Some(el.clone());
    while let Some(current) = cursor {
        if chain.len() > MAX_ANCESTOR_DEPTH {
            return Err(DbElementError::OwnerCycle {
                start: el.refno(),
                at: current.refno(),
            });
        }
        if let Some(frame) = implicit_world(&current)? {
            world = frame;
            break;
        }
        chain.push(local_matrix(&current)?);
        cursor = current.owner()?;
    }
    for local in chain.iter().rev() {
        world *= *local;
    }
    Ok(world)
}

/// 标架由属主现算、不读自己 POS/ORI 的元素。
enum ImplicitFrame {
    /// GENSEC 名下的 `JLDATU` / `PLDATU` / `ENDATU`（[`crate::datum`]）。
    GensecDatum,
    /// SCTN / STWALL 名下的 `FITT` / `CMFI`：`world(属主) * section_fitting_placement(属主, 件)`，
    /// 与 SCTN builder 装配件用的是同一份标架（E3D TTY 真值：SCTN 名下 521 只、STWALL 名下
    /// 126 只逐件对上）。CMFI 与 FITT 共用 `GFIORI` / `GFIORG`（recheck-3 §2.3 路由表），
    /// 属性面（POSL / ZDIS / DELP / BANG）相同。
    SectionFitting(DbElement),
    /// `CMFI` 名下的 `SBFI`：嵌套背板配件，原点沿用母件原点，成员自己的 POS/ORI 一样不读。
    /// 旋转见 [`sub_fitting_rotation`]（Z 取 `ZDIR`、Y 取母件 Y 去掉沿 Z 的分量、再绕 Z 转 `BANG`），
    /// 母件在 STWALL 上还是 SCTN 上都是这一条。
    SubFitting { cmfi: DbElement },
    /// SCTN 名下的 `SNOD`：结构节点，`ZDIS` 是沿梁轴（POSS→POSE）的距离。
    /// `world(属主 SCTN) * section_axis_frame(属主) * T(0,0,ZDIS)`——SCTN 自己没有 POS/ORI，
    /// 轴架现算（`section_axis_frame`，不含中性轴自旋：绕 Z 转不动沿 Z 的度量）。
    /// 只认 SCTN 属主；STWALL 名下 SNOD 的 ZDIS 是沿路径弧长，语义不同，不接。
    ///
    /// SNOD 名下的关节元素（`SJOI` 等）没有自己的隐式标架——`GJTORI` / `GJTORG` 内部
    /// 未翻，暂按普通 owner 链折进 SNOD 标架（它们的 `DELP` / `BANG` 在语料里全是 0）。
    SectionNode(DbElement),
    /// Detailed handrail posts have no POS/ORI.  Core3D places the post at
    /// its `POSREF` path point and applies `ZDIS`; template children inherit
    /// that frame through their owner chain.  Infill panels (`HRPANE` under
    /// `HRPNSE`) use the same station plus their `XOFF` / `ZOFF`.
    HandrailPost(DbElement),
}

/// 只看 noun 与属主，不求标架。
fn implicit_frame(el: &DbElement) -> Result<Option<ImplicitFrame>, DbElementError> {
    let noun = el.element_type()?;
    match noun.as_str() {
        "JLDATU" | "PLDATU" | "ENDATU" => Ok(crate::datum::is_gensec_datum(el)
            .map_err(|error| pseudo_attribute(el, &noun, error))?
            .then_some(ImplicitFrame::GensecDatum)),
        "FITT" | "CMFI" => match el.owner()? {
            Some(host) if matches!(host.element_type()?.as_str(), "SCTN" | "STWALL") => {
                Ok(Some(ImplicitFrame::SectionFitting(host)))
            }
            _ => Ok(None),
        },
        "SBFI" => match el.owner()? {
            Some(cmfi) if cmfi.element_type()?.as_str() == "CMFI" => match cmfi.owner()? {
                Some(host) if matches!(host.element_type()?.as_str(), "SCTN" | "STWALL") => {
                    Ok(Some(ImplicitFrame::SubFitting { cmfi }))
                }
                _ => Ok(None),
            },
            _ => Ok(None),
        },
        "SNOD" => match el.owner()? {
            Some(host) if host.element_type()?.as_str() == "SCTN" => {
                Ok(Some(ImplicitFrame::SectionNode(host)))
            }
            _ => Ok(None),
        },
        "HRPOST" | "HRGATE" | "HRTERM" | "HRPANE" => {
            Ok(Some(ImplicitFrame::HandrailPost(el.clone())))
        }
        _ => Ok(None),
    }
}

/// 现算标架的元素的世界标架；别的元素交 `None`。
///
/// 求不出来（宿主 SPINE 读坏、目录链断之类）按伪属性现算失败报，与桥架件同一种错。
fn implicit_world(el: &DbElement) -> Result<Option<DMat4>, DbElementError> {
    let Some(kind) = implicit_frame(el)? else {
        return Ok(None);
    };
    let noun = el.element_type()?;
    match kind {
        ImplicitFrame::GensecDatum => {
            crate::datum::gensec_datum_world(el).map_err(|error| pseudo_attribute(el, &noun, error))
        }
        ImplicitFrame::SectionFitting(host) => {
            let placement =
                crate::section::section_fitting_placement(&host, el).map_err(|error| {
                    pseudo_attribute(el, &noun, crate::elmodl::ModelError::from(error))
                })?;
            Ok(Some(world_matrix(&host)? * placement))
        }
        ImplicitFrame::SubFitting { cmfi } => {
            let parent = world_matrix(&cmfi)?;
            let origin = parent.transform_point3(DVec3::ZERO);
            let zdir = el.get_direction("ZDIR")?.unwrap_or([0.0, 0.0, 1.0]);
            let bang = el.get_double("BANG")?.unwrap_or(0.0);
            Ok(Some(match sub_fitting_rotation(&parent, zdir, bang) {
                Some(rotation) => DMat4::from_cols(
                    rotation.x_axis.extend(0.0),
                    rotation.y_axis.extend(0.0),
                    rotation.z_axis.extend(0.0),
                    origin.extend(1.0),
                ),
                // ZDIR 退化或与母件 Y 共线，投不出参考向：沿母件标架。
                None => parent,
            }))
        }
        ImplicitFrame::SectionNode(host) => {
            let frame = crate::section::section_axis_frame(&host).map_err(|error| {
                pseudo_attribute(el, &noun, crate::elmodl::ModelError::from(error))
            })?;
            let zdis = el.get_double("ZDIS")?.unwrap_or(0.0);
            Ok(Some(
                world_matrix(&host)? * frame * DMat4::from_translation(DVec3::new(0.0, 0.0, zdis)),
            ))
        }
        ImplicitFrame::HandrailPost(post) => handrail_post_world(&post),
    }
}

/// CMFI 名下 SBFI 的旋转（[`ImplicitFrame::SubFitting`]）：
///
/// - `Z` = 母件旋转 × `ZDIR`——`ZDIR` 就是 SBFI 的 Z 轴在母件系里的分量（原样，不翻 y）；
/// - `Y₀` = 母件 Y 去掉沿 Z 的分量后归一，`X₀ = Y₀ × Z`；
/// - 绕 Z 转 `BANG`：`X = X₀·cos + Y₀·sin`，`Y = Y₀·cos − X₀·sin`。
///
/// E3D TTY 真值（gen-model `verification/civil-tty-20261021/`）：
/// - 母件在 STWALL 上（`sbfi_truth.psv`）：ams1112 `17496/136127`（`BANG = 90`）与 `17496/140426`
///   （`BANG = 180`）逐轴对上；同一条规则套在 CMPF 名下的 `17496/137956`（`BANG = 0`）也对上。
/// - 母件在 SCTN 上（APS 项目副本，`aps7200_sbfi_truth.psv` / `aps7200_cmfi_truth.psv`）：aps7200
///   `23584/8625`、`23584/8629`（楼梯底板，`BANG = 0`）。E3D 的 Z 带 2.05e-6 的倾角，正是 `ZDIR` 存值
///   经母件旋转后的余量——此前按 RVM 拟合的「安装线水平投影 + 世界竖直」在这里只是近似。
///
/// `ZDIR` 退化或与母件 Y 共线时 `None`。
fn sub_fitting_rotation(parent: &DMat4, zdir: [f64; 3], bang_deg: f64) -> Option<DMat3> {
    let z = parent
        .transform_vector3(DVec3::from_array(zdir))
        .normalize_or_zero();
    if z.length_squared() <= f64::EPSILON {
        return None;
    }
    let line = parent.transform_vector3(DVec3::Y);
    let y0 = (line - z * line.dot(z)).normalize_or_zero();
    if y0.length_squared() <= f64::EPSILON {
        return None;
    }
    let x0 = y0.cross(z);
    let (sin, cos) = bang_deg.to_radians().sin_cos();
    Some(DMat3::from_cols(
        x0 * cos + y0 * sin,
        y0 * cos - x0 * sin,
        z,
    ))
}

/// World frame for a detailed handrail post (`POSTSE -> HANDRA`) or infill
/// panel (`HRPNSE -> HANDRA`).  `HRPOST` / `HRPANE` deliberately have no `POS`/`ORI`; its placement is the referenced RPATH
/// point plus the scalar distance along the selected path run.  Keeping
/// this as an implicit frame also makes every nested TMPL/primitive inherit
/// the same placement via the ordinary owner-chain fold.
fn handrail_post_world(post: &DbElement) -> Result<Option<DMat4>, DbElementError> {
    let Some(post_set) = post.owner()? else {
        return Ok(None);
    };
    let Some(handrail) = post_set.owner()? else {
        return Ok(None);
    };
    if handrail.element_type()?.as_str() != "HANDRA" {
        return Ok(None);
    }
    let Some(point) = post.get_element("POSREF")? else {
        return Ok(None);
    };
    let Some(position) = point.get_position("POS")? else {
        return Ok(None);
    };
    let zdis = post.get_double("ZDIS")?.unwrap_or(0.0);
    // `ZDIS` is the distance along the handrail path (despite the legacy
    // name), not a world-Z elevation.  Use the adjacent RPATH point to get
    // the local tangent; this is what places repeated posts between corners.
    let tangent = handrail_path_tangent(&handrail, &point, zdis)?;
    let path_tangent = tangent.unwrap_or(DVec3::Z);
    let distance = handrail_straight_run_distance(&handrail, &point, zdis)?;
    let path_position = handrail_curve_station(&handrail, &point, zdis)?
        .unwrap_or_else(|| DVec3::from_array(position) + path_tangent * distance);
    // HRPOST catalogue templates use their local +X axis back along the
    // handrail run.  The station itself is measured in the forward path
    // direction, but the template's profile origin is oriented by the
    // opposite tangent (verified on the +X and +Y straight runs in the live
    // APS sample).  Keeping this orientation on the implicit frame also
    // rotates every nested template primitive consistently.
    let post_kind = post.element_type()?;
    let justification = post.get_string("HRJUSL")?;
    let terminal_template = post.get_ref("SPRE")?;
    // The 36004 terminal catalogue is the transverse LEFT variant.  Other
    // LEFT terminals in the sample use the ordinary unrotated frame.
    let orient_term = post_kind == "HRTERM"
        && (justification.as_deref() != Some("LEFT")
            || terminal_template.is_some_and(|reference| reference.word1 == 36004));
    let local = if (post_kind == "HRPOST" || post_kind == "HRPANE" || orient_term)
        && let Some(run) = tangent.and_then(|value| value.try_normalize())
    {
        // LEFT and RIGH are mirror placements of the same catalogue post.
        // Their local +X axis points respectively with or against the path
        // tangent; treating both as RIGH mirrors an entire handrail run.
        let x_axis = match justification.as_deref() {
            // LEFT terminal templates are transverse to the path; their
            // catalogue +X axis follows the horizontal normal.
            Some("LEFT")
                if post_kind == "HRTERM"
                    && terminal_template.is_some_and(|reference| reference.word1 == 36004) =>
            {
                run.cross(DVec3::Z).normalize()
            }
            Some("LEFT") => run,
            // FORW posts use the path's left-hand horizontal normal rather
            // than the run axis (the 36007 catalogue in the live sample).
            Some("FORW") => run.cross(DVec3::Z).normalize(),
            _ => -run,
        };
        let y_axis = DVec3::Z.cross(x_axis).normalize();
        DMat4::from_cols(
            x_axis.extend(0.0),
            y_axis.extend(0.0),
            DVec3::Z.extend(0.0),
            path_position.extend(1.0),
        )
    } else {
        DMat4::from_translation(path_position)
    };
    if post_kind == "HRPANE" {
        // Infill panels sit at the same station, shifted by `ZOFF` up and by
        // `XOFF` along the station frame's -Y (the right-hand side of the run
        // for LEFT): aps250110 `/B01_LEVEL_03_ACCESS/HR_001`, both panels
        // (`XOFF 69 / ZOFF -170`) against the E3D TTY export.
        let xoff = post.get_double("XOFF")?.unwrap_or(0.0);
        let zoff = post.get_double("ZOFF")?.unwrap_or(0.0);
        let offset = DMat4::from_translation(DVec3::new(0.0, -xoff, zoff));
        return Ok(Some(world_matrix(&handrail)? * local * offset));
    }
    if post.element_type()?.as_str() == "HRGATE" {
        let width = post.get_double("WIDT")?.unwrap_or(0.0);
        let gate_frame = DMat4::from_translation(DVec3::new(0.0, width * 0.5, 0.0))
            * DMat4::from_rotation_z(-std::f64::consts::FRAC_PI_2);
        return Ok(Some(world_matrix(&handrail)? * local * gate_frame));
    }
    Ok(Some(world_matrix(&handrail)? * local))
}

/// Resolve a detailed post on the first point of a THRU RPATH curve to the
/// actual circular station.  Core3D places these posts on the arc itself;
/// using the through-point chord as a constant tangent loses the lateral
/// displacement that becomes large near the end of the arc.
fn handrail_curve_station(
    handrail: &DbElement,
    point: &DbElement,
    zdis: f64,
) -> Result<Option<DVec3>, DbElementError> {
    if zdis < 0.0 {
        return Ok(None);
    }
    let Some(rpath) = handrail
        .members()?
        .find(|member| member.element_type().is_ok_and(|noun| noun == "RPATH"))
    else {
        return Ok(None);
    };
    let points: Vec<DbElement> = rpath
        .members()?
        .filter(|member| member.element_type().is_ok_and(|noun| noun == "POINTR"))
        .collect();
    if points.first().map(|candidate| candidate.refno()) != Some(point.refno()) {
        return Ok(None);
    }
    let Some(next) = points.get(1) else {
        return Ok(None);
    };
    let Some(start_pos) = point.get_position("POS")? else {
        return Ok(None);
    };
    let Some(end_pos) = next.get_position("POS")? else {
        return Ok(None);
    };
    let Some(curve) = rpath
        .members()?
        .find(|member| member.element_type().is_ok_and(|noun| noun == "CURVE"))
    else {
        return Ok(None);
    };
    if curve.get_string("CURTYP")?.as_deref() != Some("THRU") {
        return Ok(None);
    }
    let Some(through_pos) = curve.get_position("POS")? else {
        return Ok(None);
    };
    let start = DVec3::from_array(start_pos);
    let through = DVec3::from_array(through_pos);
    let end = DVec3::from_array(end_pos);
    let normal = (through - start).cross(end - through);
    if normal.length_squared() <= 1e-12 {
        return Ok(None);
    }
    let centre = circumcentre(start, through, end, normal);
    let radius_vector = start - centre;
    let radius = radius_vector.length();
    if radius <= 1e-9 {
        return Ok(None);
    }
    let axis = normal.normalize();
    let mut sweep = axis
        .dot(radius_vector.cross(end - centre))
        .atan2(radius_vector.dot(end - centre));
    if sweep < 0.0 {
        sweep += std::f64::consts::TAU;
    }
    let arc_length = radius * sweep;
    if zdis > arc_length + 1e-6 {
        return Ok(None);
    }
    Ok(Some(
        centre + crate::section::rotate_about(radius_vector, axis, zdis / radius),
    ))
}

fn circumcentre(a: DVec3, b: DVec3, c: DVec3, normal: DVec3) -> DVec3 {
    let ab = b - a;
    let ac = c - a;
    let denom = 2.0 * normal.length_squared();
    a + (normal.cross(ab) * ac.length_squared() + ac.cross(normal) * ab.length_squared()) / denom
}

/// A detailed post anchored just past a straight path endpoint is clamped to
/// that endpoint by Core3D.  Keep the rule narrow: CURVE/RSECT paths have a
/// separate stationing model and must not be clamped using chord lengths.
fn handrail_straight_run_distance(
    handrail: &DbElement,
    point: &DbElement,
    zdis: f64,
) -> Result<f64, DbElementError> {
    let Some(rpath) = handrail
        .members()?
        .find(|member| member.element_type().is_ok_and(|noun| noun == "RPATH"))
    else {
        return Ok(zdis);
    };
    if rpath
        .members()?
        .any(|member| member.element_type().is_ok_and(|noun| noun != "POINTR"))
    {
        return Ok(zdis);
    }
    let points: Vec<DbElement> = rpath
        .members()?
        .filter(|member| member.element_type().is_ok_and(|noun| noun == "POINTR"))
        .collect();
    let Some(index) = points
        .iter()
        .position(|candidate| candidate.refno() == point.refno())
    else {
        return Ok(zdis);
    };
    let (a, b) = if let Some(next) = points.get(index + 1) {
        (point, next)
    } else if let Some(previous) = index.checked_sub(1).and_then(|i| points.get(i)) {
        // The terminal post's tangent looks back along the entering run,
        // but its station clamp is measured from that entering point to the
        // terminal point itself.
        (previous, point)
    } else {
        return Ok(zdis);
    };
    let (Some(a), Some(b)) = (a.get_position("POS")?, b.get_position("POS")?) else {
        return Ok(zdis);
    };
    Ok(zdis.min((DVec3::from_array(b) - DVec3::from_array(a)).length()))
}

fn handrail_path_tangent(
    handrail: &DbElement,
    point: &DbElement,
    zdis: f64,
) -> Result<Option<DVec3>, DbElementError> {
    let Some(rpath) = handrail
        .members()?
        .find(|member| member.element_type().is_ok_and(|noun| noun == "RPATH"))
    else {
        return Ok(None);
    };
    let points: Vec<DbElement> = rpath
        .members()?
        .filter(|member| member.element_type().is_ok_and(|noun| noun == "POINTR"))
        .collect();
    let Some(index) = points
        .iter()
        .position(|candidate| candidate.refno() == point.refno())
    else {
        return Ok(None);
    };
    // E3D measures an end-point post's ZDIS back along the entering run.  The
    // last POINTR can be a corner extension, so using `point -> previous`
    // would turn the post onto the wrong leg (the 8 m displacement is a useful
    // real-world example).  Interior and start points use the following run.
    let (from, to) = if index + 1 < points.len() {
        (point.clone(), points[index + 1].clone())
    } else if index >= 2 {
        (points[index - 1].clone(), points[index - 2].clone())
    } else if index == 1 {
        (point.clone(), points[0].clone())
    } else {
        return Ok(None);
    };
    let Some(from_pos) = from.get_position("POS")? else {
        return Ok(None);
    };
    let Some(to_pos) = to.get_position("POS")? else {
        return Ok(None);
    };
    // A CURVE member replaces the first straight edge.  Use the tangent of
    // the actual circular station, rather than the chord to its control point
    // (which is only correct at the start of the arc).
    if index == 0 {
        if handrail_curve_station(handrail, point, zdis)?.is_some() {
            let step = 0.1;
            let current = handrail_curve_station(handrail, point, zdis)?;
            let next = handrail_curve_station(handrail, point, zdis + step)?;
            let previous = handrail_curve_station(handrail, point, (zdis - step).max(0.0))?;
            let tangent = match (next, previous) {
                (Some(next), Some(previous)) => (next - previous).try_normalize(),
                // At an arc endpoint the forward probe is outside the arc.  A
                // one-sided derivative keeps the terminal post on the arc
                // tangent instead of falling back to the control-point chord.
                (None, Some(previous)) => {
                    current.and_then(|current| (current - previous).try_normalize())
                }
                (Some(next), None) => current.and_then(|current| (next - current).try_normalize()),
                (None, None) => None,
            };
            if tangent.is_some() {
                return Ok(tangent);
            }
        }
    }
    Ok((DVec3::from_array(to_pos) - DVec3::from_array(from_pos)).try_normalize())
}

fn pseudo_attribute(
    el: &DbElement,
    noun: &str,
    error: crate::elmodl::ModelError,
) -> DbElementError {
    match error {
        crate::elmodl::ModelError::Db(error) => error,
        other => DbElementError::PseudoAttribute {
            refno: el.refno(),
            noun: noun.to_string(),
            name: "POS".into(),
            detail: other.to_string(),
        },
    }
}

/// [`matrix_relative_to`] 的失败面。
///
/// 单开一个类型，是因为「上溯到根都没遇到 ancestor」根本不是一种
/// [`DbElementError`]：两个元素都读得好好的，错的是它们的**关系**。原先这里
/// 借 `DbElementError::NotFound` 报，还得凭空编一个 `sesno: 0`，报告里读出来
/// 是「db N 会话 0 的索引里没有 refno」——查的人会去翻索引，而索引没有任何问题。
#[derive(Debug, thiserror::Error)]
pub enum RelativeMatrixError {
    #[error("{descendant} 不在 {ancestor} 的 owner 子树里，求不出相对矩阵")]
    NotADescendant { descendant: RefNo, ancestor: RefNo },
    #[error(transparent)]
    Db(#[from] DbElementError),
}

/// `descendant` 相对 `ancestor` 局部系的矩阵：沿 owner 链从 ancestor 的下一级
/// 折叠到 descendant。descendant 不在 ancestor 子树里 = 数据错，响亮报。
pub fn matrix_relative_to(
    descendant: &DbElement,
    ancestor: &DbElement,
) -> Result<DMat4, RelativeMatrixError> {
    let mut chain = vec![local_matrix(descendant)?];
    let mut through_datum = implicit_frame(descendant)?.is_some();
    let mut cursor = descendant.owner()?;
    loop {
        let Some(current) = cursor else {
            return Err(RelativeMatrixError::NotADescendant {
                descendant: descendant.refno(),
                ancestor: ancestor.refno(),
            });
        };
        if current.refno() == ancestor.refno() {
            break;
        }
        if chain.len() > MAX_ANCESTOR_DEPTH {
            return Err(DbElementError::OwnerCycle {
                start: descendant.refno(),
                at: current.refno(),
            }
            .into());
        }
        through_datum |= implicit_frame(&current)?.is_some();
        chain.push(local_matrix(&current)?);
        cursor = current.owner()?;
    }
    // 中间隔着现算标架的元素时，逐级右乘不成立，改由两端的世界矩阵求。
    if through_datum {
        return Ok(world_matrix(ancestor)?.inverse() * world_matrix(descendant)?);
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

    /// STWALL 上 CMFI 名下 SBFI 的旋转对 E3D TTY 的 `ORI WRT WORLD`（ams1112，母件 `17496/136126`
    /// 与 `17496/140425` 在同一堵墙上、标架相同）。
    #[test]
    fn sub_fitting_rotation_matches_tty_on_stwall() {
        let parent = DMat4::from_cols(
            DVec3::new(0.0, 0.0, -1.0).extend(0.0),
            DVec3::new(-0.924199810, -0.381909298, 0.0).extend(0.0),
            DVec3::new(-0.381909298, 0.924199810, 0.0).extend(0.0),
            DVec3::new(2424.0, -7649.0, 2125.0).extend(1.0),
        );
        let cases = [
            (
                "17496/136127",
                [
                    -3.791130992464279e-10,
                    -0.38190929758561,
                    0.9241998097909705,
                ],
                90.0,
                [[-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]],
            ),
            (
                "17496/140426",
                [
                    4.082852644064673e-10,
                    -0.09667963406149882,
                    -0.995315552153052,
                ],
                180.0,
                [
                    [0.0, 0.0, -1.0],
                    [0.882947593, 0.469471563, 0.0],
                    [0.469471563, -0.882947593, 0.0],
                ],
            ),
        ];
        for (refno, zdir, bang, axes) in cases {
            let rotation = sub_fitting_rotation(&parent, zdir, bang).expect("ZDIR 不退化");
            for (got, want) in [rotation.x_axis, rotation.y_axis, rotation.z_axis]
                .into_iter()
                .zip(axes)
            {
                assert!(
                    (got - DVec3::from_array(want)).length() < 1e-6,
                    "{refno}: {got:?} != {want:?}"
                );
            }
        }
        assert!(
            sub_fitting_rotation(&parent, [0.0, 1.0, 0.0], 0.0).is_none(),
            "ZDIR 沿母件 Y 时投不出参考向"
        );
    }

    /// 母件在 SCTN 上：aps7200 `23584/8625` 的母件 `23584/8624` 取 E3D TTY 的 `ORI WRT WORLD`，结果对 SBFI
    /// 自己的 TTY 真值到 1e-8——连 Z 那 2.05e-6 的倾角也对上（拿世界竖直当 Z 对不上这一位）。
    #[test]
    fn sub_fitting_rotation_matches_tty_on_sctn() {
        let parent = DMat4::from_cols(
            DVec3::new(-1.0, 0.0, 0.0).extend(0.0),
            DVec3::new(0.0, -0.754593720, 0.656192287).extend(0.0),
            DVec3::new(0.0, 0.656192287, 0.754593720).extend(0.0),
            DVec3::new(12860.5, 10000.0, 1000.0).extend(1.0),
        );
        let zdir = [0.0, 0.6561907406682005, 0.7545950648270359];
        let rotation = sub_fitting_rotation(&parent, zdir, 0.0).expect("ZDIR 不退化");
        let axes = [
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.000002049],
            [0.0, 0.000002049, 1.0],
        ];
        for (got, want) in [rotation.x_axis, rotation.y_axis, rotation.z_axis]
            .into_iter()
            .zip(axes)
        {
            assert!(
                (got - DVec3::from_array(want)).length() < 1e-8,
                "{got:?} != {want:?}"
            );
        }
    }

    /// 「不在子树里」报出来要指向关系，不能把人引到索引上去。
    ///
    /// 旧写法借 `DbElementError::NotFound` 并编了个 `sesno: 0`，读报告的人看到
    /// 「db N 会话 0 的索引里没有 X」会去翻索引——而索引没有任何问题。
    #[test]
    fn not_a_descendant_names_the_relationship_not_the_index() {
        let text = RelativeMatrixError::NotADescendant {
            descendant: RefNo::new(9, 1234),
            ancestor: RefNo::new(9, 7),
        }
        .to_string();
        assert!(text.contains("9/1234") && text.contains("9/7"), "{text}");
        assert!(text.contains("子树"), "{text}");
        assert!(!text.contains("会话") && !text.contains("索引"), "{text}");
    }
}

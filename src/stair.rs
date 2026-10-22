//! 楼梯梯段 `STRFLT` 名下的踏步 `TREAD`：几何是目录件 GMRE 里的一只 `SBOX`，尺寸由 E3D 按梯段
//! 现算的两个伪属性给出。
//!
//! aps250110 `/B01_LEVEL_03_ACCESS/SF_001`、`SF_002` 的 25 只踏步：`SPRE → GPART
//! /SLH_TREADS/Simple_Catalogued_Tread --CATR--> SFIT /Simple_Catalogued_Tread`，GMRE 里一只 `SBOX`：
//! `PXLE = RPRO TWID`、`PYLE = RPRO TDEP`、`PZLE = RPRO THK`，中心 `(0, TDEP / 2, −THK / 2)`。DTSE 把
//! 三项接到设计侧：`TWID ← ATTRIB TWIDTH`、`TDEP ← ATTRIB TREADD`、`THK ← PARA[3]`（20）。踏步自己
//! 的 `TWIDTH` 存 0、`TREADD` 根本不存——两者都是随梯段现算的伪属性。E3D TTY RVM 给的盒是
//! 800 × 269.654 × 20 与 800 × 297.924 × 20。
//!
//! - `TWIDTH`：踏步自己的大于 0 就用它，否则取梯段的 `TWIDTH`。
//! - `TREADD`：梯段踏步宽（going）+ 前伸 `TREADO`。going 是 `OGOING`（大于 0 时），否则梯段 `RPATH`
//!   首末点的水平距离 ÷ 级数，级数 = 竖向距离 ÷ `RISE` 取整（两段梯 11 / 15 级）。这样算出 269.659 /
//!   297.919，与 E3D 差 ±0.005 mm；E3D 的确切算式没有取证。
//!
//! 梯段在踏步的模型单元之上：增量靠 `increment::implicit_frame_host_of` 的 STRFLT 支，梯段或它的
//! RPATH 改了就把整棵梯段子树级联重建。

use e3d_io::db_element::{DbElement, DbElementError};
use glam::DVec3;

/// 踏步的两个伪属性（mm）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TreadSize {
    /// `TWIDTH`
    pub width: f64,
    /// `TREADD`
    pub depth: f64,
}

/// `TREAD` 的 `TWIDTH` / `TREADD`。不是 TREAD、找不到属主梯段或梯段数据不全时 `None`，
/// 目录表达式照旧按未解报。
pub fn tread_size(tread: &DbElement) -> Result<Option<TreadSize>, DbElementError> {
    if tread.element_type()? != "TREAD" {
        return Ok(None);
    }
    let Some(flight) = owning_flight(tread)? else {
        return Ok(None);
    };
    let own = tread.get_double("TWIDTH")?.unwrap_or(0.0);
    let width = if own > 0.0 {
        own
    } else {
        flight.get_double("TWIDTH")?.unwrap_or(0.0)
    };
    let Some(going) = going(&flight)? else {
        return Ok(None);
    };
    let depth = going + flight.get_double("TREADO")?.unwrap_or(0.0);
    Ok((width > 0.0 && depth > 0.0).then_some(TreadSize { width, depth }))
}

/// `TREAD → TREADS → STRFLT`；中间隔几层不定死，最多往上找三层。
fn owning_flight(tread: &DbElement) -> Result<Option<DbElement>, DbElementError> {
    let mut current = tread.owner()?;
    for _ in 0..3 {
        let Some(owner) = current else {
            break;
        };
        if owner.element_type()? == "STRFLT" {
            return Ok(Some(owner));
        }
        current = owner.owner()?;
    }
    Ok(None)
}

/// 梯段的踏步宽（going）。
fn going(flight: &DbElement) -> Result<Option<f64>, DbElementError> {
    let set = flight.get_double("OGOING")?.unwrap_or(0.0);
    if set > 0.0 {
        return Ok(Some(set));
    }
    let rise = flight.get_double("RISE")?.unwrap_or(0.0);
    let Some(rpath) = crate::rpath::first_rpath(flight)? else {
        return Ok(None);
    };
    let mut points: Vec<DVec3> = Vec::new();
    for member in rpath.members()? {
        if member.element_type()? == "POINTR" {
            points.push(DVec3::from_array(
                member.get_position("POS")?.unwrap_or_default(),
            ));
        }
    }
    let (Some(first), Some(last)) = (points.first(), points.last()) else {
        return Ok(None);
    };
    let run = *last - *first;
    let steps = (run.z.abs() / rise).round();
    if !(rise > 0.0 && steps >= 1.0) {
        return Ok(None);
    }
    Ok(Some(run.truncate().length() / steps))
}

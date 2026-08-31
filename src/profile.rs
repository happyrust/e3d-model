//! 环剖面采集：PLOO/PAVE（板）与 LOOP/VERT（通用）→ 顶点环 + 高度 + 对齐。
//!
//! 对标 core.dll 取几何前的属性读（`GATPOS`/`DGETI` 一族）；语义按 gen-model
//! `fetch_loops_and_height` 的既有口径（SurrealQL 原文见 old-aios-core
//! `rs_surreal/geom.rs`）：
//!
//! - 环 = 元素的 PLOO/LOOP 成员（**文件原序**）；第一个环是外边界，其余是孔
//!   （「第二个开始都是负实体」）。
//! - 环顶点 = PLOO/LOOP 的 PAVE/VERT 成员，取 `POS[0], POS[1]`（局部 XY 平面）
//!   与 `FRAD`（倒圆半径，未设 = 0）。
//! - 高度 `HEIG`：第一个环上的值优先，环上没有再取元素自身（`PLOO.HEIG ?? PANE.HEIG`）。
//! - 对齐 `SJUS`：只在 PLOO 上读（LOOP 模板没有这个属性）；TOP 族下移一个高度、
//!   CEN 族下移半个高度（gen-model `cal_sjus_value` 同款）。
//!
//! 未设属性一律按 dabacon 缺省语义处理（缺省被省存，读回 `None` = 默认值），
//! 模板里根本没有该属性（`UnknownAttribute`）同样按未设算——SurrealQL 老口径的
//! `?? none` 就是这么落的。其余错误照直上抛。

use crate::discretise::FilletVertex;
use e3d_io::db_element::{DbElement, DbElementError};

/// 一个可拉伸元素的剖面数据。
#[derive(Debug, Clone)]
pub struct LoopProfile {
    /// 顶点环，文件原序：`rings[0]` 外边界，其余是孔。
    pub rings: Vec<Vec<FilletVertex>>,
    /// 拉伸高度（mm）。
    pub height: f64,
    /// SJUS 对齐产生的沿局部 −Z 下移量（mm）。无 SJUS 时为 0。
    pub sjus_drop: f64,
}

#[derive(Debug, thiserror::Error)]
pub enum ProfileError {
    #[error(transparent)]
    Db(#[from] DbElementError),
    #[error("{noun} {refno} 下没有任何 PLOO/LOOP 环，出不了剖面")]
    NoLoops { refno: String, noun: String },
    #[error("{refno} 的环 {ring_index} 只有 {points} 个顶点，构不成剖面")]
    DegenerateRing {
        refno: String,
        ring_index: usize,
        points: usize,
    },
}

/// `UnknownAttribute` 与未设一视同仁（见模块文档），其余错误上抛。
fn double_or_none(el: &DbElement, name: &str) -> Result<Option<f64>, DbElementError> {
    match el.get_double(name) {
        Ok(v) => Ok(v),
        Err(DbElementError::UnknownAttribute { .. }) => Ok(None),
        Err(e) => Err(e),
    }
}

fn string_or_none(el: &DbElement, name: &str) -> Result<Option<String>, DbElementError> {
    match el.get_string(name) {
        Ok(v) => Ok(v),
        Err(DbElementError::UnknownAttribute { .. }) => Ok(None),
        Err(e) => Err(e),
    }
}

/// SJUS 词 → 沿局部 −Z 的下移量（gen-model `cata_model::cal_sjus_value` 同款表）。
pub fn sjus_drop(sjus: &str, height: f64) -> f64 {
    match sjus {
        "UTOP" | "DTOP" | "TOP" => height,
        "UCEN" | "DCEN" | "CENT" => height / 2.0,
        _ => 0.0,
    }
}

/// 从环属主（PANE/FLOOR/GWALL/NXTR/EXTR…）采集剖面。
///
/// 对标 Core3D 共享轮廓装配器 `sub_10719900`（EXT/REV/板/墙共用，活桥反编译，
/// 7UW4 §3.7.1）：迭代环(PLOO/LOOP)的顶点(PAVE/VERT) 读 位置(ATT POS) + 倒圆
/// (ATT FRAD，即 libgm `D2_Profile` span 的 bulge)，第一环外边界、其余作孔。
/// §3.7.1 的绕向规范化（shoelace 有向面积 + `getApproxPolyLine`/`selfIntersecting`
/// 自交翻绕向）本 crate 收在 `solid::extrude_rings`/`revolve_rings`：外环有向面积
/// 翻正 + `FillRule::NonZero` 等价化解自交轮廓（见 `solid` 模块文档第 1、2 条）。
pub fn loop_profile(el: &DbElement) -> Result<LoopProfile, ProfileError> {
    let mut rings: Vec<Vec<FilletVertex>> = Vec::new();
    let mut first_loop_heig: Option<f64> = None;
    let mut first_ploo_sjus: Option<String> = None;

    for member in el.members()? {
        let noun = member.element_type()?;
        if noun != "PLOO" && noun != "LOOP" {
            continue;
        }
        if rings.is_empty() {
            first_loop_heig = double_or_none(&member, "HEIG")?;
            if noun == "PLOO" {
                first_ploo_sjus = string_or_none(&member, "SJUS")?;
            }
        }
        let mut ring = Vec::new();
        for vertex in member.members()? {
            let vnoun = vertex.element_type()?;
            if vnoun != "PAVE" && vnoun != "VERT" {
                continue;
            }
            let pos = vertex.get_position("POS")?.unwrap_or([0.0; 3]);
            let frad = double_or_none(&vertex, "FRAD")?.unwrap_or(0.0);
            ring.push(FilletVertex {
                x: pos[0],
                y: pos[1],
                frad,
            });
        }
        rings.push(ring);
    }

    if rings.is_empty() {
        return Err(ProfileError::NoLoops {
            refno: el.refno().to_string(),
            noun: el.element_type()?,
        });
    }
    for (ring_index, ring) in rings.iter().enumerate() {
        if ring.len() < 3 {
            return Err(ProfileError::DegenerateRing {
                refno: el.refno().to_string(),
                ring_index,
                points: ring.len(),
            });
        }
    }

    let height = match first_loop_heig {
        Some(h) => h,
        None => double_or_none(el, "HEIG")?.unwrap_or(0.0),
    };
    let drop = first_ploo_sjus
        .as_deref()
        .map(|sjus| sjus_drop(sjus, height))
        .unwrap_or(0.0);

    Ok(LoopProfile {
        rings,
        height,
        sjus_drop: drop,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sjus_table_matches_cata_model() {
        assert_eq!(sjus_drop("UTOP", 80.0), 80.0);
        assert_eq!(sjus_drop("DTOP", 80.0), 80.0);
        assert_eq!(sjus_drop("TOP", 80.0), 80.0);
        assert_eq!(sjus_drop("UCEN", 80.0), 40.0);
        assert_eq!(sjus_drop("DCEN", 80.0), 40.0);
        assert_eq!(sjus_drop("CENT", 80.0), 40.0);
        assert_eq!(sjus_drop("UBOT", 80.0), 0.0);
        assert_eq!(sjus_drop("", 80.0), 0.0);
    }
}

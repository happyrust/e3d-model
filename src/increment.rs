//! 模型增量更新：两个会话之间只重建受影响的元素。
//!
//! ## 管线分层（对标 core.dll / Core3D 的五层，取证见
//! `gen-model/docs/plans/2026-08-31-core-aligned-increment-architecture.md`）
//!
//! ```text
//! L0 会话定位   两端各钉一个 sesno 的 DbSet        ← switchToOldSession
//! L1 索引候选   collect_window → IndexCandidate    ← DB_IndexTableCompare【只是候选】
//! L2 逐元素差分 element_diff::diff_element         ← DB_Compare::checkEle【权威判据】
//! L3 语义账     ledger::ChangeLedger               ← DB_UserChanges
//! L4 单元上卷   plan_update → UpdatePlan           ← findTopLevelElement
//! ```
//!
//! **L1 只是候选过滤器，不是结论。** 索引说的 `Modified` 是「记录位置变了」＝
//! 记录被重写过，不等于内容变了。core 拿到候选一定回去做记录级、逐属性、按类型
//! 比值的差分（[`crate::element_diff`]），我们照做。少了这一层，「被重写」与
//! 「真的变了」分不开，而这两件事在真实语料里差着数量级。
//!
//! 三段式，段间只传数据：
//!
//! 1. **窗口收集** [`collect_window`]：e3d-io 的会话索引差分
//!    （对标 `core.dll::DB_IndexTableCompare`——删除 = 键集差、修改 = 记录位置变、
//!    flag 不参与判定）。先全量拿完 [`CompletedIndexDiff`] 再消费，差分中途失败
//!    整个窗口失败，没有半截结果可以拿去更新模型（ADR-017 整窗口口径）。
//! 2. **候选分类** [`plan_update`]：把索引级候选（refno + 增/删/改）折算成
//!    **模型单元**动作。模型单元的判据只有一份，在 [`crate::category::is_model_unit`]（落到元素上是
//!    [`is_model_unit_of`]：FITT 在 SCTN / STWALL 下自己成件、在别的属主下不成件）——全量管线按它
//!    产出、这里按它上卷，两边各写一份名单就会静默漏件（见该函数文档记的那次事故）。
//!    与 Core3D `PartialUpdateDesiMgr` 的 significant/primitive 上卷同构：
//!    - 候选自己是单元 → 单元就是它；
//!    - 候选是被消费的数据（剖面环 / 负体，含 CMPF 里的深层负体）→ 沿 owner 链
//!      上卷到最近的单元属主（`SignificantOwner` 语义：从自身起判、无深度上限、
//!      List 容器天然被跨过）；
//!    - 删除的单元不上卷，直接出 Remove（Core R15 同款）；
//!    - 上卷路上跨过路由容器（含候选自身）→ 容器进派生集，名下隐式管身按两端键
//!      重算（见下「派生单元」一节）；
//!    - 改动落在没有单元归属、也不在任何路由射程里的元素上（容器改名、NonGraphic、
//!      未建的目录件）→ 记 `no_model` 账，一个都不许蒸发；
//!    - 调用方经 [`plan_update_with`] 判为「只改了不影响模型的属性、无结构变化」的 Modified
//!      候选（gen-model 的两票：E3D 字典 DCHC=0 ∧ 生成器不读；2026-09-15 之前是 DataOnly ∧
//!      DCHC=0 的五个名字）→ 同样记 `no_model`、不上卷不级联，但账本照记、闭包种子照收
//!      （候选与它的祖先链照进闭包；闭包里能少的只有它本来会触发的粗级联扇出——桥架分支整条
//!      重建那片单元——那些根由调用方按两票前移）；调用方顺带说是哪一档（[`NoModelTier`]），
//!      只进两个横切计数。
//! 3. **执行** [`execute_plan`]：逐单元重建（与全量管线同一条 [`model_element`]），
//!    产出 Upsert / Remove 动作。重建结果为 Skipped / 失败时**发 Remove**——
//!    Core 的次序是先擦后画，画不出来那一行就该保持擦掉的状态，绝不留陈旧网格。
//!    派生容器同款走 [`route_tubes_of`]，旧槽位键做差发 Remove。
//!
//! ## 生产定位：谁消费这三段（gen-model ADR-057）
//!
//! 本 crate 是模型面的**纯函数层**：输入只有 `(库文件, sesno)` 与显式传入的配置 /
//! 缓存，不持 DB 句柄、不认识凭证 / 队列 / 水位。三段里**前两段是生产路**，
//! **第三段不是**：
//!
//! - gen-model 生产只消费 [`collect_window`] → [`plan_update`] →
//!   [`UpdatePlan`] / [`AffectedClosure`]（回答「S→T 之间哪些单元变了、哪些根被
//!   触到」），然后按**根**整根重算——走全量管线
//!   [`crate::pipeline::generate_subtree_with_cache`]，由 gen-model 侧做根级 CAS 发布、
//!   `ModelTarget` 指纹、manifest 去重、scoped delete，再写凭证（ADR-056 D3）。
//! - [`execute_plan`] / [`increment_update`] 是 L4 上卷「先擦后画」语义的**可执行规格**
//!   与离线工具（`update_ams` bin、`tests/increment_real.rs` 真库门、gen-model
//!   `increment_planner_parity` 对拍）。它产出的单元级 Upsert / Remove 在 gen-model
//!   持久层里**没有对应物**——凭证、cohort CAS、scoped delete 全是根级——所以不在
//!   生产接线。要把执行下沉到单元级，先另立 ADR 给这三样造出单元级对应物，
//!   不要绕过这一步直接接。
//!
//! 反过来，「谁是根」（MDU / significant owner 枚举）与「哪些根被触到」
//! （`touches_roots`）按 ADR-057 D4 / D5 都归本 crate：它们对 `DbSet` 是纯遍历，
//! 与 [`crate::category::is_model_unit`] 是同一棵树上的两层粒度（`BOX` 是单元不是根，`EQUI` 是根不是
//! 单元），并存、不互相替换。
//!
//! ## 与 Core3D 的一处有意分歧：变换级联
//!
//! 我们的产物是**世界系网格**（世界矩阵烘进顶点），Core3D 的图形段树在绘制时
//! 才 `CONCAT` 矩阵——所以容器（ZONE/STRU/…）POS/ORI 一动，Core 不用重画任何
//! 后代，我们却必须重建其子树内全部正体单元。而这些后代的**索引记录根本没变**
//! （世界位置是读取时沿 owner 链推导的），差分不会点名它们。
//!
//! **这一条不因为补齐 L2/L3 而消失，它是产物形态决定的。** core 的五层管线里
//! 没有任何一层对应 [`collect_unit_subtree`]，不是因为 core 更聪明，是因为
//! core 的产物是带矩阵的段树而我们的是烘死的三角形。照搬 core 把级联删掉，
//! `increment_real` 的 `Cascaded` 样本会当场证明它是静默缺件。
//!
//! L2 改的是**触发判据**，不是级联本身：从「两端 `(POS, ORI)` 不等」换成
//! [`ElementDiff::placement_input_changed`]。差别在于后者还认**改挂**——世界矩阵
//! 是沿 owner 链折叠出来的，链换了一条，`(POS, ORI)` 一个字没动，矩阵照样全变。
//! 那正是老实现漏掉的一半。
//!
//! ## 账本
//!
//! gen-model 宪法同款：静默缺件是最高级别缺陷。每个候选必须落进
//! `rolled_up` / `no_model` / `unresolved` 三本账之一（[`IncrementReport::accounts_for`]
//! 硬判据），执行侧 `regenerated + regen_skipped + regen_failed == units_to_regenerate`。
//!
//! ## 派生单元：隐式管身的增量走另一条腿
//!
//! 路由容器不是模型单元——它自己 refno 上没有「一件」几何。其中 RVM 坐实会补管
//! 的四族（[`crate::category::DERIVED_ROUTE_CONTAINER_NOUNS`]，HANG 除外：E3D
//! 不在 HANG 成员间补管；属主是 HVAC 的风管 BRAN 也除外：风管没有隐式管身，
//! 判据 [`derives_implied_tubes`] 与全量管线同一份）名下派生出 N 根按
//! [`crate::elmodl::GeometryId::ImpliedTube`]（两端站点 + 同站点对内出现序）
//! 索引的管身。它们的增量动作因此与模型单元不同：
//!
//! - **计划**：上卷路上跨过路由容器（[`derives_implied_tubes`]，含候选自身）、或级联
//!   子树里扫到路由容器，都进 [`UpdatePlan::regenerate_derived`]；同时在 **base 端**
//!   用 [`route_slot_keys`] 收这些容器的全部旧槽位键（[`UpdatePlan::derived_stale`]）。
//!   旧键收的是**全部槽位**而不是「上次真建成的那批」——计划期不建几何，无从知道
//!   哪些槽位当时退化了。多出来的 Remove 落在从未存在的键上，而 Remove 本就是
//!   幂等的「确保不存在」，宁多勿漏。
//! - **执行**：容器在 target 端还活着就 [`route_tubes_of`] 整链重算（与全量管线
//!   同一段串接代码），产出 upsert；已删或不再是路由容器则一根不建（retired）。
//!   旧键里没被这一轮重建接住的，统一发 Remove——与模型单元同一条
//!   「先擦后画，画不出保持擦掉」。
//!
//! 真库门今天在这条腿上没有判别力（门只开 DESI 库，P 点解不出、两端全量
//! `implied_tubes.built` 都是 0，管身键两侧都不出现）；等目录库进门它自动生效。
//! **目录件自身内容变了仍不在射程内**——那是跨库失效通道的活（设计文档 §3.4），
//! 不做进窗口差分。
//!
//! [`world_matrix`]: crate::transform::world_matrix

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;
use std::sync::Arc;

use e3d_io::db_element::{DbElement, DbElementError, DbSet};
use e3d_io::error::E3dReaderError;
use e3d_io::index::{CompletedIndexDiff, IndexCandidate, IndexDiff, IndexTreeError};
use e3d_io::page::PageId;
use e3d_io::refno::RefNo;
use e3d_io::session::DbView;

use crate::catalogue::CatalogueMeshCache;
use crate::category::{classify, derives_implied_tubes, graphics_ignored_below, is_model_unit_of};
use crate::element_diff::{ElementDiff, diff_element};
use crate::elmodl::{GeneratedElement, GeometryId, Modeled, model_element_with_cache};
use crate::ledger::{ChangeLedger, ChangeTally, Disposition};
use crate::pipeline::{Incident, route_tubes_of};
use crate::route::route_slot_keys;

/// 上卷 / 子树遍历的环保护。dabacon 树深实测个位数（与 `transform` 同一口径）。
const MAX_ANCESTOR_DEPTH: usize = 128;

#[derive(Debug, thiserror::Error)]
pub enum IncrementError {
    #[error(transparent)]
    Io(#[from] E3dReaderError),
    #[error("索引差分失败，窗口作废：{0}")]
    Index(#[from] IndexTreeError),
    #[error("会话 {sesno} 不在 {file} 的会话链上（链上有 {available:?}）")]
    UnknownSession {
        file: String,
        sesno: u32,
        available: Vec<u32>,
    },
    #[error("窗口不是正向的：base={base} 必须在 target={target} 之前（按链序，不是数字大小）")]
    WindowNotForward { base: u32, target: u32 },
}

/// 一个收集完成的增量窗口：两端会话 + 全量差分结果。
pub struct WindowDiff {
    pub base_sesno: u32,
    pub target_sesno: u32,
    pub base_root: PageId,
    pub target_root: PageId,
    pub diff: CompletedIndexDiff,
}

/// 收集 `base_sesno → target_sesno` 的索引差分窗口。
///
/// 两个端点都必须在文件的会话链上，且 base 在 target 之前（**链序**为准——
/// 链不保证连号，数字大小不算数）。差分任一步失败整体失败。
pub fn collect_window(
    db_file: &Path,
    base_sesno: u32,
    target_sesno: u32,
) -> Result<WindowDiff, IncrementError> {
    let mut view = DbView::open(db_file)?;
    let sessions = view.walk_sessions()?;
    let position = |sesno: u32| sessions.iter().position(|s| s.sesno == sesno);
    let missing = |sesno: u32| IncrementError::UnknownSession {
        file: db_file.display().to_string(),
        sesno,
        available: sessions.iter().map(|s| s.sesno).collect(),
    };
    let target_at = position(target_sesno).ok_or_else(|| missing(target_sesno))?;
    let base_at = position(base_sesno).ok_or_else(|| missing(base_sesno))?;
    // 链是新→旧，base 必须比 target 旧 = 位置更靠后。
    if base_at <= target_at {
        return Err(IncrementError::WindowNotForward {
            base: base_sesno,
            target: target_sesno,
        });
    }
    let base_root = sessions[base_at].index_root;
    let target_root = sessions[target_at].index_root;
    let diff = IndexDiff::new(view.pages_mut(), base_root, target_root).finish()?;
    Ok(WindowDiff {
        base_sesno,
        target_sesno,
        base_root,
        target_root,
        diff,
    })
}

/// 增量总账（可直接 serde_json 落盘）。
#[derive(Debug, Default, serde::Serialize)]
pub struct IncrementReport {
    pub base_sesno: u32,
    pub target_sesno: u32,
    /// 索引差分候选总数（inserted + modified + deleted）。
    pub candidates: usize,
    pub inserted: usize,
    pub modified: usize,
    pub deleted: usize,
    /// 位置相同仅 flag 不同的键数。真实语料上恒 0，非零值得人看（e3d-io R5 口径）。
    pub flag_only_drifts: u64,
    /// L3 语义账：这一窗的改动各是什么（对标 `DB_UserChanges` 的语义桶）。
    pub changes: ChangeTally,
    /// 索引点了名、L2 逐字节比下来内容一个字没动的候选数。
    ///
    /// 它们既没上卷也不是 `no_model`，单独一本账——这是 L1 与 L2 之间的落差，
    /// 与「这个改动落在没有模型归属的元素上」是两回事。老实现没有 L2，
    /// 这些候选一律当「变了」照常上卷重建。
    pub unchanged: usize,
    /// `no_model` 里由调用方的「不影响模型」谓词判掉、且谓词说是**老名单**那一档的候选
    /// （[`NoModelTier::MetadataOnly`]，[`plan_update_with`]）：记录真的变了，但改动全部落在业务
    /// 元数据上（NAME / DESC / PURP / FUNC / LCHKDA），成员 / 属主 / 类型都没动。
    /// 横切计数，已含在 `no_model` 里——它回答的是「这一窗省了多少次上卷」，不是第四本账。
    pub metadata_only: usize,
    /// `no_model` 里由同一条谓词判掉、但至少一个属性落在**新档**的候选（[`NoModelTier::UnreadAttribute`]，
    /// 2026-09-15 S6）：字典 DCHC=0 且生成器不读、却不在老名单里的属性。与 `metadata_only` 互斥、
    /// 同样已含在 `no_model` 里；两个数分开记，是让「门变宽以后多省了多少」在回执上读得出来。
    pub unread_attribute: usize,
    /// `no_model` 里由同一条谓词判掉、改动**只**落在戳上的候选（[`NoModelTier::CacheStamp`]，2026-09-17 Q6）：
    /// 整库缓存重建的扫写、或挪一件时被顺带盖戳的子树成员。与前两档互斥、同样已含在 `no_model` 里。
    pub cache_stamp: usize,
    /// 折算进了某个单元动作（重建 / 移除，含被去重掉的）的候选数。
    pub rolled_up: usize,
    /// `rolled_up` 里**只**靠变换级联折进去的候选数（自身与上方都没有单元 / 容器，
    /// 是子树里的后代被它带动）：典型是挪了位的 ZONE / EQUI。横切计数，不是第四本账。
    pub cascade_only: usize,
    /// 全部级联命中的单元 / 派生容器数之和（含已在计划集里的），与 `cascades.len()`
    /// 一起读：几处漂移、总共带动了多少件。
    pub cascade_hits: usize,
    /// 账本（L3）上出现过的不同 refno 数。候选除 `unchanged` 之外每个都得在账本上有一条
    /// （读不出的记 [`crate::ledger::ChangeKind::Unresolved`]），所以它必须等于
    /// `rolled_up + no_model + unresolved`——这是 L3 对 L4 的守恒，`accounts_for` 判。
    pub ledger_refnos: usize,
    /// 候选数超过 [`PlanLimits::max_candidates`]，规划器一个候选都没分类就退化成
    /// 「整库重建」的原因。非空时候选账不成立（什么都没分类），计划集全空，
    /// 调用方**必须**走全量生成而不是执行这份空计划。
    pub degraded_to_full: Option<String>,
    /// 没落到任何模型单元的候选：noun → 数量（容器改名、NonGraphic、目录件……）。
    pub no_model: BTreeMap<String, usize>,
    /// 计划重算派生几何的路由容器：容器 noun → 个数（按容器去重，不按候选）。
    ///
    /// 这不是第四本账——点名这些容器的候选已按各自去向记进 `rolled_up` /
    /// `no_model`，这里是横切的一笔，让「这一窗动了哪些链」一眼数得出来。
    pub derived_planned: BTreeMap<String, usize>,
    /// base 端旧槽位键读取失败的容器（容器在 base 端存在、链却走不通）。
    ///
    /// 出现即数据或读层问题，且后果与 `unresolved` 同级：说不出旧键就删不掉
    /// 旧管身，静默陈旧与静默缺件是同一级缺陷。不并进 `unresolved`——那本账
    /// 按候选计数、参与候选恒等式，这本按容器计。
    pub derived_stale_unreadable: Vec<Incident>,
    /// 连分类都做不了的候选（端点读不出 noun / owner 链断）。出现即数据或读层问题。
    pub unresolved: Vec<Incident>,
    /// 变换级联：哪个元素的世界矩阵输入变了、扇出了几个单元。
    ///
    /// 触发判据是 [`ElementDiff::placement_input_changed`]，含 `POS`/`ORI` 与**改挂**。
    pub cascades: Vec<Incident>,
    /// 计划出的单元数。
    pub units_to_regenerate: usize,
    pub units_to_remove: usize,
    /// 计划重算派生几何的容器数（= `derived_planned` 各项之和）。
    pub derived_to_regenerate: usize,
    /// 执行账：三者之和必须等于 `units_to_regenerate`。
    pub regenerated: usize,
    pub regen_skipped: Vec<Incident>,
    pub regen_failed: Vec<Incident>,
    /// 派生执行账：三者之和必须等于 `derived_to_regenerate`。
    ///
    /// `derived_regenerated` = 容器在 target 端活着、整链重算过（可能一根管身都
    /// 没建成——退化槽位不出几何，但旧键照样会被清）。
    pub derived_regenerated: usize,
    /// 容器在 target 端已删、或类型变得不再是路由容器：不重算，旧键全清。
    pub derived_retired: usize,
    /// 容器在 target 端读挂 / 链走不通：与模型单元的 `regen_failed` 同款，
    /// 旧键全清（先擦后画，画不出保持擦掉），且必须响。
    pub derived_failed: Vec<Incident>,
    /// 这一轮 upsert 的隐式管身根数（横切计数，已含在 `upserts` 里）。
    pub tube_upserts: usize,
    /// 这一轮 Remove 的管身键数（横切计数，已含在 `removals` 里）。
    pub tube_removals: usize,
    /// 逐容器的管段账：`regenerate_derived` 里每条在 target 端处理过的路由容器（pdms `a/b` 串）→
    /// 这一轮**真建成**的管身根数（[`route_tubes_of`] 返回多少就记多少；退休 / 类型已变的容器记 0，
    /// 读挂的不记——它已在 `derived_failed` 里响）。
    ///
    /// 与 `upserts` 分开记账，是给下游**局部**发布的管段完整性门用的（gen-model ADR-080 D6）：
    /// 那边一条 BRAN 的管身是整段重写，`upserts` 里少带一段就是静默丢管；门拿这本账对
    /// 真带上的段数，从 `upserts` 里数回来就是自证，抓不到烘焙 / 过滤 / 装配路上掉的那一段。
    /// 横切计数，各值之和 == `tube_upserts`。
    pub tube_segments: BTreeMap<String, usize>,
    /// 非致命观察（级联检查读挂但单元已入账、成员表疑似回环等）。
    pub notes: Vec<Incident>,
}

impl IncrementReport {
    /// 账要闭合：每个候选恰好落一本账；账本上每个 refno 恰好对应一本候选账；
    /// 每个待重建单元恰好有一个执行结论。
    ///
    /// 退化成整库重建（`degraded_to_full`）的报告没有分类过任何候选：候选账不成立、
    /// 三本账与账本都必须是空的——空计划被当成「没变」执行才是这里要拦的事。
    pub fn accounts_for(&self) -> Result<(), String> {
        let classified = self.rolled_up
            + self.unchanged
            + self.no_model.values().sum::<usize>()
            + self.unresolved.len();
        if let Some(reason) = &self.degraded_to_full {
            if classified != 0
                || self.ledger_refnos != 0
                || self.units_to_regenerate != 0
                || self.units_to_remove != 0
                || self.derived_to_regenerate != 0
            {
                return Err(format!(
                    "退化成整库重建（{reason}）的计划不该分类过任何候选：\
                     classified {classified} ledger {} units {}/{}/{}",
                    self.ledger_refnos,
                    self.units_to_regenerate,
                    self.units_to_remove,
                    self.derived_to_regenerate,
                ));
            }
            return Ok(());
        }
        if classified != self.candidates {
            return Err(format!(
                "候选账不平：rolled_up {} + unchanged {} + no_model {} + unresolved {} \
                 = {classified}，候选 {}",
                self.rolled_up,
                self.unchanged,
                self.no_model.values().sum::<usize>(),
                self.unresolved.len(),
                self.candidates,
            ));
        }
        // L3 对 L4：除 unchanged 之外每个候选都在账本上、且只有一个去向。
        let ledgered =
            self.rolled_up + self.no_model.values().sum::<usize>() + self.unresolved.len();
        if self.ledger_refnos != ledgered {
            return Err(format!(
                "账本不平：账本上 {} 个 refno，候选账 rolled_up {} + no_model {} + unresolved {} \
                 = {ledgered}",
                self.ledger_refnos,
                self.rolled_up,
                self.no_model.values().sum::<usize>(),
                self.unresolved.len(),
            ));
        }
        if self.cascade_only > self.rolled_up {
            return Err(format!(
                "cascade_only {} 是 rolled_up {} 的子集，不能比它大",
                self.cascade_only, self.rolled_up
            ));
        }
        let gated = self.metadata_only + self.unread_attribute + self.cache_stamp;
        let no_model = self.no_model.values().sum::<usize>();
        if gated > no_model {
            return Err(format!(
                "metadata_only {} + unread_attribute {} + cache_stamp {} = {gated} 是 no_model {no_model} 的子集，不能比它大",
                self.metadata_only, self.unread_attribute, self.cache_stamp
            ));
        }
        let executed = self.regenerated + self.regen_skipped.len() + self.regen_failed.len();
        if executed != self.units_to_regenerate {
            return Err(format!(
                "执行账不平：regenerated {} + skipped {} + failed {} = {executed}，计划 {}",
                self.regenerated,
                self.regen_skipped.len(),
                self.regen_failed.len(),
                self.units_to_regenerate,
            ));
        }
        let derived = self.derived_regenerated + self.derived_retired + self.derived_failed.len();
        if derived != self.derived_to_regenerate {
            return Err(format!(
                "派生执行账不平：regenerated {} + retired {} + failed {} = {derived}，计划 {}",
                self.derived_regenerated,
                self.derived_retired,
                self.derived_failed.len(),
                self.derived_to_regenerate,
            ));
        }
        // 逐容器管段账是 `tube_upserts` 的横切拆分：每条处理过（建成或退休）的容器一条，
        // 各值之和必须等于这一轮真 upsert 的管身根数。
        let segmented = self.tube_segments.values().sum::<usize>();
        if segmented != self.tube_upserts
            || self.tube_segments.len() != self.derived_regenerated + self.derived_retired
        {
            return Err(format!(
                "管段账不平：逐容器 {} 条共 {segmented} 段，tube_upserts {}，建成 + 退休容器 {}",
                self.tube_segments.len(),
                self.tube_upserts,
                self.derived_regenerated + self.derived_retired,
            ));
        }
        Ok(())
    }

    pub fn totals_line(&self) -> String {
        format!(
            "window {}→{}: candidates={} (+{} ~{} -{}) rolled_up={} unchanged={} no_model={} \
             (metadata_only={} unread_attribute={} cache_stamp={}) unresolved={} cascades={} units regen={} \
             remove={} derived={} | regenerated={} skipped={} failed={} | derived regen={} \
             retired={} failed={} tubes +{} -{}",
            self.base_sesno,
            self.target_sesno,
            self.candidates,
            self.inserted,
            self.modified,
            self.deleted,
            self.rolled_up,
            self.unchanged,
            self.no_model.values().sum::<usize>(),
            self.metadata_only,
            self.unread_attribute,
            self.cache_stamp,
            self.unresolved.len(),
            self.cascades.len(),
            self.units_to_regenerate,
            self.units_to_remove,
            self.derived_to_regenerate,
            self.regenerated,
            self.regen_skipped.len(),
            self.regen_failed.len(),
            self.derived_regenerated,
            self.derived_retired,
            self.derived_failed.len(),
            self.tube_upserts,
            self.tube_removals,
        )
    }

    /// L3 语义账那一行：这一窗到底发生了什么。
    pub fn changes_line(&self) -> String {
        let ChangeTally {
            created,
            deleted,
            type_changed,
            reparented,
            members_changed,
            attributes_modified,
            opaque_record_changes,
            unresolved,
            unchanged,
            created_subtree_roots,
            deleted_subtree_roots,
        } = self.changes;
        format!(
            "changes: created={created}（子树顶 {created_subtree_roots}） \
             deleted={deleted}（子树顶 {deleted_subtree_roots}） type_changed={type_changed} \
             reparented={reparented} members={members_changed} attrs={attributes_modified} \
             opaque={opaque_record_changes} unchanged={unchanged} unresolved={unresolved} \
             | ledger_refnos={} cascade_only={} cascade_hits={}",
            self.ledger_refnos, self.cascade_only, self.cascade_hits
        )
    }
}

/// [`plan_update_bounded`] 的护栏。`Default` = 不设限，即 [`plan_update`] 的行为。
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PlanLimits {
    /// 候选数**超过**它（严格大于）就不逐候选分类，直接退化成整库重建：
    /// 一窗动了大半个库时，逐候选走 owner 链比整库重算还贵（ams1112 721→722 的
    /// 24674 个候选 6.4 s，而全量生成本身也就这个量级），而且这种窗口多半是
    /// 合并 / 回退 / 重建，差分结论本来就不值得信。`None` = 不设限。
    ///
    /// 这里只认绝对数：`WindowDiff` 不知道 target 端一共有多少键。想按比例定
    /// （gen-model 用的是索引键数的 30%），调用方自己乘好再传进来。
    pub max_candidates: Option<usize>,
}

/// 分类完成、还没执行的更新计划。键用 `(word0, word1)`（`RefNo` 不带 `Ord`，
/// 这里只要一个稳定的去重序，不承载索引侧的无符号比较语义）。
pub struct UpdatePlan {
    pub regenerate: BTreeSet<(u32, u32)>,
    pub remove: BTreeSet<(u32, u32)>,
    /// 需要重算名下派生几何的容器（[`derives_implied_tubes`]，今天只有会补管身的
    /// 那四族路由容器，HANG 与风管 BRAN 不在内）。
    ///
    /// 与 `regenerate` 分开，因为**执行动作不同**：模型单元是「按 refno 删一件画
    /// 一件」，这些是「两端各算一遍链上的槽位键，新的 upsert、旧键里没被接住的
    /// remove」。混进 `regenerate` 会让 `execute_plan` 对着容器 refno 调
    /// [`model_element`]，那是一件根本不存在的东西。
    pub regenerate_derived: BTreeSet<(u32, u32)>,
    /// 上面那些容器在 **base 端**名下的全部槽位键。
    ///
    /// 在这里算而不是留给 `execute_plan`：`execute_plan` 按设计只读 target
    /// （文档里那句「与全量管线走同一条 `model_element`」），而旧键只有 base 说了算。
    /// [`route_slot_keys`] 只走成员表、不建几何，正好是 `plan_update`「只读两端
    /// DbSet」的活。
    pub derived_stale: BTreeSet<GeometryId>,
    /// L3 语义账。计数已经折进 [`IncrementReport::changes`]，这里保留逐条明细，
    /// 给探针与真库门查「这一窗到底改了什么」用。
    pub ledger: ChangeLedger,
    /// 非空 = 这份计划**不是**增量：候选数超过 [`PlanLimits::max_candidates`]，规划器
    /// 一个候选都没分类，四个集合全空。调用方要走整库重建；把它交给 [`execute_plan`]
    /// 只会得到一份零动作的产出（报告带 `degraded_to_full`），不是「没变」。
    pub full_rebuild: Option<String>,
    pub report: IncrementReport,
}

impl UpdatePlan {
    /// 计划期能对的账：候选账（报告）+ 账本守恒（L3 每个 refno 恰好一个去向，且去向
    /// 计数与报告的三本候选账逐项相等）。执行账要 [`execute_plan`] 之后才成立，
    /// 那一半在 [`IncrementReport::accounts_for`] 里、由产出方的报告去对。
    pub fn accounts_for(&self) -> Result<(), String> {
        let classified = self.report.rolled_up
            + self.report.unchanged
            + self.report.no_model.values().sum::<usize>()
            + self.report.unresolved.len();
        if self.full_rebuild.is_none() && classified != self.report.candidates {
            return Err(format!(
                "候选账不平：rolled_up {} + unchanged {} + no_model {} + unresolved {} \
                 = {classified}，候选 {}",
                self.report.rolled_up,
                self.report.unchanged,
                self.report.no_model.values().sum::<usize>(),
                self.report.unresolved.len(),
                self.report.candidates,
            ));
        }
        let tally = self.ledger.accounts_for()?;
        let no_model = self.report.no_model.values().sum::<usize>();
        if tally.rolled_up != self.report.rolled_up
            || tally.no_model != no_model
            || tally.unresolved != self.report.unresolved.len()
        {
            return Err(format!(
                "账本去向与候选账不一致：账本 rolled_up {} / no_model {} / unresolved {}，\
                 报告 {} / {no_model} / {}",
                tally.rolled_up,
                tally.no_model,
                tally.unresolved,
                self.report.rolled_up,
                self.report.unresolved.len(),
            ));
        }
        if self.report.ledger_refnos != self.ledger.distinct_refnos() {
            return Err(format!(
                "报告记的账本 refno 数 {} 与账本实际 {} 不一致",
                self.report.ledger_refnos,
                self.ledger.distinct_refnos()
            ));
        }
        Ok(())
    }
}

/// 这一窗的改动**波及了哪些元素**：每个受影响单元 / 派生容器 / 改挂・类型变更
/// 条目，自身连同它在两端图里的全部祖先（含自身，向上直到根）。
///
/// 它回答的是消费端的问题「根 r 名下的模型这一窗有没有可能变了」，而不是
/// `regenerate` 那个「重建哪一件」——两者粒度不同：gen-model 的生成根是 MDU
/// 交付单元（BRAN / EQUI / …），比网格单元粗一层；同一份闭包对任意一层的根都能
/// 直接查 `refnos.contains(r)`，不必各自再走一遍 owner 链。
///
/// **闭包是按祖先封闭的**：一件在里面，它上方的每一个属主都在里面。所以「根 r
/// 不在闭包里」＝「r 子树里没有任何一件受影响的东西」，正是凭证前移要的判据
/// （gen-model ADR-056 D3：`plan_update` 只用来选根与未变根凭证前移）。
///
/// # 为什么两端图都走
///
/// - `regenerate` 里的单元在 target 端活着，但**改挂**把它从旧根挪到了新根：
///   旧根的 manifest 里还留着它的一件，旧根也得重算。旧根只有 base 图认得。
/// - `remove` 里的单元 target 端已经没有了，只有 base 图能说出它曾挂在哪条链上。
/// - `Reparented` / `TypeChanged` 条目本身未必是单元（一根挪了位的 FTUB），
///   两端的链一起收，宁多勿漏。
///
/// 某一端不存在（新建的在 base 没有、删掉的在 target 没有）是正常形态，静默跳过；
/// 链走到一半读挂才进 [`AffectedClosure::unreadable`]——那是数据或读层问题，
/// 闭包可能不完整，消费端不得据此前移任何凭证。
#[derive(Debug, Default)]
pub struct AffectedClosure {
    /// 受影响元素及其全部祖先（含自身），`(word0, word1)` 键。
    pub refnos: BTreeSet<(u32, u32)>,
    /// 走 owner 链中途读挂的起点。非空 = 闭包可能不完整。
    pub unreadable: Vec<Incident>,
}

impl AffectedClosure {
    pub fn contains(&self, refno: RefNo) -> bool {
        self.refnos.contains(&key(refno))
    }

    /// 闭包是否完整可信：没有一条链是读挂的。
    pub fn is_complete(&self) -> bool {
        self.unreadable.is_empty()
    }
}

impl UpdatePlan {
    /// 受影响元素的祖先闭包，见 [`AffectedClosure`]。只读两端 `DbSet`，不建几何。
    ///
    /// 种子 = 计划集里的每个单元 / 派生容器 **∪ 账本上的每个候选**（除 L2 判没变的）。
    /// 后一半是 2026-09-02 加的：闭包服务的是凭证前移——「根 r 不在闭包里」要能推出
    /// 「r 子树里**什么都没动**」。只拿单元做种子时，落进 `no_model` 的改动（挪了位的
    /// RPATH/POINTR、改了 PKDI 的桥架件、任何我们**今天**不认识但**明天**会消费的数据）
    /// 不在闭包里，它上方的根就会被前移，几何却已经过期。多收的代价只是根级重算，
    /// 少收的代价是静默陈旧——按「宁多勿漏」取前者。
    pub fn affected_closure(&self, base: &Arc<DbSet>, target: &Arc<DbSet>) -> AffectedClosure {
        self.affected_closure_filtering_ledger(base, target, |_| true)
    }

    /// 与 [`Self::affected_closure`] 相同，但允许消费端只从账本种子中排除已被其模型入口门
    /// 明确判为无模型影响的条目。`regenerate` / `remove` / `regenerate_derived` 三个动作集
    /// 始终作为种子保留；过滤器不能撤销已经上卷出的几何或删除动作。
    pub fn affected_closure_filtering_ledger(
        &self,
        base: &Arc<DbSet>,
        target: &Arc<DbSet>,
        keep_ledger_entry: impl Fn(&crate::ledger::LedgerEntry) -> bool,
    ) -> AffectedClosure {
        let mut closure = AffectedClosure::default();
        let mut seeds: BTreeSet<(u32, u32)> = BTreeSet::new();
        seeds.extend(self.regenerate.iter().copied());
        seeds.extend(self.remove.iter().copied());
        seeds.extend(self.regenerate_derived.iter().copied());
        for entry in self
            .ledger
            .entries()
            .iter()
            .filter(|entry| keep_ledger_entry(entry))
        {
            seeds.insert(key(entry.refno));
        }
        // 「已收过就早退」的捷径只在**同一端图**内成立：同一个 refno 在 base 与 target
        // 上的祖先不是同一条链（改挂正是这样），所以每端各记一份 visited，闭包本身共享。
        // 曾经两端共用 `closure.refnos` 当 visited：先走 target 把元素收进去，再走 base 时
        // 第一步就早退，旧属主链整条丢掉——ams8000 45→46 的旧根 `24384/25801` 就这么漏的
        // （真库门 `the_affected_closure_reaches_both_owners_of_a_reparented_element`）。
        let mut visited_target: BTreeSet<(u32, u32)> = BTreeSet::new();
        let mut visited_base: BTreeSet<(u32, u32)> = BTreeSet::new();
        for seed in seeds {
            let refno = refno_of(seed);
            collect_ancestors_inclusive(target, refno, &mut visited_target, &mut closure);
            collect_ancestors_inclusive(base, refno, &mut visited_base, &mut closure);
        }
        closure
    }

    /// `roots` 里哪些根的子树这一窗被波及了。判据就是 [`AffectedClosure::contains`]。
    pub fn touches_roots(
        &self,
        roots: &[RefNo],
        base: &Arc<DbSet>,
        target: &Arc<DbSet>,
    ) -> BTreeSet<(u32, u32)> {
        let closure = self.affected_closure(base, target);
        roots
            .iter()
            .copied()
            .filter(|root| closure.contains(*root))
            .map(key)
            .collect()
    }
}

/// 把 `refno` 自身与它在 `set` 这一端的全部祖先并进闭包。
///
/// 起点不存在于这一端是正常形态（新建的在 base 没有、删掉的在 target 没有），
/// 静默返回；链中途读挂记进 `unreadable`。owner 指向自己是 dabacon 的根形态，
/// 与 [`nearest_unit`] 同口径；真正的环由 `MAX_ANCESTOR_DEPTH` 兜住。
///
/// `visited` 是**这一端图**的已走集：一件在这一端走过，它上方的链在这一端就收全了
/// （闭包按祖先封闭），不必再爬。它不能拿 `closure.refnos` 代替——那是两端合一的。
fn collect_ancestors_inclusive(
    set: &Arc<DbSet>,
    refno: RefNo,
    visited: &mut BTreeSet<(u32, u32)>,
    closure: &mut AffectedClosure,
) {
    let mut current = set.element(refno);
    match current.exists() {
        Ok(true) => {}
        Ok(false) => return,
        Err(DbElementError::NotFound { .. }) => return,
        Err(error) => {
            closure
                .unreadable
                .push(incident(refno, "?", format!("起点存在性读不出：{error}")));
            return;
        }
    }
    let mut depth = 0usize;
    loop {
        if !visited.insert(key(current.refno())) {
            return;
        }
        closure.refnos.insert(key(current.refno()));
        match current.owner() {
            Ok(Some(owner)) if owner.refno() == current.refno() => return,
            Ok(Some(owner)) => {
                depth += 1;
                if depth > MAX_ANCESTOR_DEPTH {
                    closure.unreadable.push(incident(
                        refno,
                        "?",
                        format!(
                            "owner 链超过 {MAX_ANCESTOR_DEPTH} 层，疑似成环，闭包截断在 {}",
                            owner.refno()
                        ),
                    ));
                    return;
                }
                current = owner;
            }
            Ok(None) => return,
            // 属主不在本库索引里（跨库 / 顶层根的另一种形态）：链在这里合法地结束。
            Err(DbElementError::NotFound { .. }) => return,
            Err(error) => {
                closure.unreadable.push(incident(
                    refno,
                    "?",
                    format!("owner 链在 {} 处读挂：{error}", current.refno()),
                ));
                return;
            }
        }
    }
}

/// 一次增量更新的完整产出：模型集上的动作 + 总账。
///
/// 两侧**同一把键**：`upserts` 里每件自带 [`GeneratedElement::geometry_id`]，
/// `removals` 也说 [`GeometryId`]。`removals` 是「确保不存在」（幂等，目标本来
/// 不在模型集里也不算错）。两个列表都已排序去重。
///
/// # 这里曾经是 `Vec<RefNo>`
///
/// 上行按 [`GeometryId`] 索引、下行按 refno 发 Remove，是同一条通路上的两种键。
/// 一个路由容器名下有 N 根隐式管身共享同一个 refno（`pipeline` 绝不制造伪 refno），
/// 于是**加得进一根管身，永远删不掉一根**。换成同一把键之后，派生单元那条腿
/// （模块文档「派生单元」一节）才接得通：管身的 Remove 说得出是哪一根。
pub struct IncrementOutcome {
    pub upserts: Vec<GeneratedElement>,
    pub removals: Vec<GeometryId>,
    pub report: IncrementReport,
}

fn key(refno: RefNo) -> (u32, u32) {
    (refno.word0, refno.word1)
}

fn refno_of(key: (u32, u32)) -> RefNo {
    RefNo::new(key.0, key.1)
}

/// 一件挂在自己 refno 上的实体的键。模型单元走这条。
fn element_id(unit: (u32, u32)) -> GeometryId {
    GeometryId::Element {
        refno: refno_of(unit).to_string(),
    }
}

/// 一根隐式管身的键：容器 + 同站点对内出现序 + 两个站点
/// （[`crate::route::route_slot_keys`] 给的那三元组）。
fn tube_id(container: RefNo, slot: (u32, RefNo, RefNo)) -> GeometryId {
    GeometryId::ImpliedTube {
        container_refno: container.to_string(),
        route_ordinal: slot.0,
        from_refno: slot.1.to_string(),
        to_refno: slot.2.to_string(),
    }
}

fn incident(refno: RefNo, noun: impl Into<String>, detail: impl Into<String>) -> Incident {
    Incident {
        refno: refno.to_string(),
        noun: noun.into(),
        detail: detail.into(),
    }
}

/// 一次上卷的结果。
struct Rollup {
    /// 起点元素自己的 noun。上卷本来就要读它，顺手交出来给账本用，省一次记录解析。
    noun: String,
    /// 起点元素的属主。L3 的祖先抢占要它（`isElementTopLevelCreate` 的判据就是
    /// 「我的属主在不在同一个桶里」）。
    owner: Option<RefNo>,
    /// 最近的模型单元（含自身）。`None` = 这条改动上方没有任何会被建模的东西。
    unit: Option<RefNo>,
    /// 路上跨过的最近一个路由容器（含自身）。它不会成为 `unit`——[`crate::category::is_model_unit`]
    /// 有意排除它，因为它自己 refno 上没有「一件」几何——而是走 [`derives_implied_tubes`]
    /// 那条路，让整条链上的隐式管身按两端键重算一遍。
    ///
    /// 带上 refno 而不只是 noun：光有 noun 只够记一笔「这一窗可能有管身陈旧了」，
    /// 点不出是哪条链，也就动不了手。
    route_container: Option<(RefNo, String)>,
}

/// 沿 owner 链同时找最近的模型单元与最近的路由容器（都含自身）。判据分别是
/// [`is_model_unit_of`]（与全量管线同一份；noun 层是 [`crate::category::is_model_unit`]，FITT 再看属主）
/// 与 [`derives_implied_tubes`]。
/// `SignificantOwner` 语义：从自身起判、跨过一切不命中的属主。
///
/// # 两条搜索必须解耦：命中单元不许提前收工
///
/// 这里一度是「碰到模型单元立即返回」。那对单元搜索没错，却把容器搜索一并砍了：
/// 链上**自身是模型单元**的成员（SBFI 是现成的例子——目录件里唯独它出几何；
/// SUPC 族名下挂型钢件同理）一挪位，早退让上层的 BRAN/SUPC 永远进不了
/// `regenerate_derived`，它两侧管身的世界坐标就静默过期了。级联也救不了——
/// [`collect_unit_subtree`] 只收候选**子树内**，不收它上方的容器。
///
/// # `graphicsBehaviour == 1` 的祖先让其下的一切都不算独立件
///
/// Core3D `DES_DrawList::isGraphicsIgnoredBetween`：从元素上溯，路上有一个中间祖先
/// 带 gb==1（[`graphics_ignored_below`]），这个元素的图形就被忽略——它是那个祖先
/// 属主几何的**数据**。所以上爬时每碰到一个 gb==1 节点，就把它**下方**找到的单元与
/// 容器整个作废（含候选自身），从它往上重找。这决定了这条链必须**走到根**：在中途
/// 收工，就不知道更上面还有没有一个 gb==1 把刚找到的单元打回数据。
fn nearest_unit(set: &Arc<DbSet>, refno: RefNo) -> Result<Rollup, DbElementError> {
    let mut current = set.element(refno);
    let start_noun = current.element_type()?;
    let start_owner = current.owner()?.map(|owner| owner.refno());
    let mut noun = start_noun.clone();
    let mut depth = 0usize;
    let mut unit: Option<RefNo> = None;
    let mut route_container: Option<(RefNo, String)> = None;
    loop {
        // 只看**祖先**：候选自己带 gb==1 说的是它的孩子，不是它自己。
        if depth > 0 && graphics_ignored_below(&noun) {
            unit = None;
            route_container = None;
        }
        // 按元素判（`is_model_unit_of`）：SCTN / STWALL 名下的 FITT 是单元，别的属主下的不是、往上卷。
        if unit.is_none() && is_model_unit_of(&current)? {
            unit = Some(current.refno());
        }
        if route_container.is_none() && derives_implied_tubes(&current)? {
            route_container = Some((current.refno(), noun.clone()));
        }
        match current.owner()? {
            // owner 指向自己是 dabacon 的另一种根形态，不是环——真库门的
            // `roots_at` 就是这个口径（「owner 指向自己、无效、或不在本库索引里」
            // 都算根）。判成 OwnerCycle 会把这类根下整棵子树的候选统统打进
            // unresolved，报的是「数据问题」而数据没有问题。
            // 真正的环（A→B→…→A）仍由 MAX_ANCESTOR_DEPTH 兜住。
            Some(owner) if owner.refno() == current.refno() => break,
            Some(owner) => {
                depth += 1;
                if depth > MAX_ANCESTOR_DEPTH {
                    return Err(DbElementError::OwnerCycle {
                        start: refno,
                        at: owner.refno(),
                    });
                }
                current = owner;
                noun = current.element_type()?;
            }
            None => break,
        }
    }
    Ok(Rollup {
        noun: start_noun,
        owner: start_owner,
        unit,
        route_container,
    })
}

/// 收集 `root` 子树内（不含 root 自身）全部模型单元与派生单元。
/// 返回子树内**命中**的单元总数（模型 + 派生，含已在计划集里的）。
///
/// # fanout 数的是命中，不是新插入
///
/// 返回值决定调用方的 `contributed`（这个候选折没折进单元动作）。这里一度返回
/// 「新插进集合的个数」：子树单元恰好已全部被同窗其它候选点名时返回 0，候选被
/// 记成 `no_model`——而那本账的语义是「改动落在没有模型归属的元素上」，它明明有。
/// 去重是计划集自己的事，不该改写「这个候选贡献了什么」；[`plan_update`] 里
/// `mark_derived` 对同一件事早已是这个口径（容器已在计划里，候选照样算 rolled_up）。
///
/// # 为什么派生单元也要一起收
///
/// 我们的产出是**世界坐标网格**。一个 ZONE 挪了位置，它名下每根隐式管身的顶点
/// 全都过期了，可路由容器不是模型单元、[`crate::category::is_model_unit`] 那一关根本拦不住它——
/// 只收 `out` 的话，级联会不声不响地把整片路由留在原地。真库门今天抓不到这个：
/// `implied_tubes.built` 还是 0，没有管身可对拍。等目录求值一落地它立刻变成
/// 「全量说管身动了、增量说没动」的静默分歧，所以现在就得收全。
///
/// 消费关系不影响收集：被消费的子树（剖面环、负体、CMPF）里本就没有单元，
/// 走一遍只是白读几页。成员表出现回环用 visited 兜住并记 note——那是数据坏，
/// 不该让级联把进程挂死。
///
/// # `visited` 是整份计划共用的（[`CascadeVisits`]）
///
/// 它一度每次调用新建：一窗里 ZONE 与它名下的 EQUI 都挪了位，EQUI 的子树就被读两遍；
/// 候选按 refno 序来，谁先谁后随机，大窗口里重叠子树重复读 N 遍是常态。现在一棵子树
/// 只走一次——后来的级联碰到**别的级联**已走过的节点就地停下，那片子树的单元早已在
/// 计划集里，记进 [`CascadeFanout::overlapped`] 而不是白读。同一次遍历里二次到达才是
/// 成员表回环，两者靠每次遍历的世代号分开。
///
/// # gb==1 节点不往下走
///
/// 它的成员是它属主几何的数据（[`graphics_ignored_below`]，与 `nearest_unit` 同口径），
/// 不是独立单元；节点自己按 [`is_model_unit_of`] / [`derives_implied_tubes`] 照常判（今天恒否）。
#[derive(Debug, Clone, Copy)]
struct CascadeFanout {
    /// 这次遍历新到达并命中的单元 / 派生容器数（含已在计划集里的）。
    hits: usize,
    /// 撞上别的级联已走过的节点数（含根自己）。非零 = 这个候选的子树（部分）
    /// 早已被本窗更早的级联覆盖，它照样算折进了计划。
    overlapped: usize,
}

/// 一份计划内全部级联遍历的到达记录：节点 → 第几次级联到的它。
#[derive(Default)]
struct CascadeVisits {
    seen: BTreeMap<(u32, u32), usize>,
    generation: usize,
    /// 本窗已经整条重建过的桥架分支（CWBRAN）：同一条分支里第二个候选起不再遍历、
    /// 也不再记一条 incident——一次导入 948 个桥架元素只该有一条「分支 X 级联」，
    /// 不是 812 条。
    tray_branches: BTreeSet<(u32, u32)>,
    /// 本窗已经整棵重建过的现算标架宿主（[`implicit_frame_host_of`]），同上去重。
    implicit_hosts: BTreeSet<(u32, u32)>,
}

fn collect_unit_subtree(
    set: &Arc<DbSet>,
    root: RefNo,
    out: &mut BTreeSet<(u32, u32)>,
    derived: &mut BTreeSet<(u32, u32)>,
    notes: &mut Vec<Incident>,
    visits: &mut CascadeVisits,
) -> Result<CascadeFanout, DbElementError> {
    visits.generation += 1;
    let generation = visits.generation;
    let mut fanout = CascadeFanout {
        hits: 0,
        overlapped: 0,
    };
    // 根自己已被更早的级联走过：整棵子树都在计划里了，一页都不必再读。
    if visits.seen.insert(key(root), generation).is_some() {
        fanout.overlapped += 1;
        return Ok(fanout);
    }
    let mut stack = vec![set.element(root)];
    while let Some(el) = stack.pop() {
        for member in el.members()? {
            match visits.seen.insert(key(member.refno()), generation) {
                Some(earlier) if earlier == generation => {
                    notes.push(incident(
                        member.refno(),
                        member.element_type().unwrap_or_else(|_| "?".into()),
                        format!("成员表回环（{root} 子树级联遍历中二次到达），已跳过"),
                    ));
                    continue;
                }
                Some(_) => {
                    fanout.overlapped += 1;
                    continue;
                }
                None => {}
            }
            let noun = member.element_type()?;
            if is_model_unit_of(&member)? {
                out.insert(key(member.refno()));
                fanout.hits += 1;
            }
            if derives_implied_tubes(&member)? {
                derived.insert(key(member.refno()));
                fanout.hits += 1;
            }
            if graphics_ignored_below(&noun) {
                continue;
            }
            stack.push(member);
        }
    }
    Ok(fanout)
}

/// 电缆桥架分支容器的 noun：桥架件放置的全部跨元素输入都在它名下。
const CABLE_TRAY_BRANCH_NOUN: &str = "CWBRAN";

/// 候选（含自身）是某个现算标架的输入时，那个标架的宿主：
///
/// - 名下有 `JLDATU` 或 `ENDATU` 的 GENSEC（[`crate::datum`]）：宿主自己（JUSL/BANG/LMIRR/SPRE）、
///   它的 SPINE 路径（SPINE 的 POS/ORI/YDIR，POINSP，CURVE），以及 datum 自己（JLDATU 的
///   ZDIS/PKDI/HPREF、PLDATU 的 POSL/YDIR、ENDATU 的 ZDIS/POSL）；
/// - 名下有 `FITT` 的 SCTN / STWALL（`section::section_fitting_placement`）：宿主自己
///   （POSS/POSE/BANG/JUSL/SPRE），以及 FITT 自己（POSL/ZDIS/BANG/DELP）；
/// - 带详图式立柱的 HANDRA（`transform::handrail_post_world`，名下有 POSTSE / HRPNSE / HRFEAT，或 RAILSE 里
///   有 HRTERM）：它的 RPATH 路径（POINTR / CURVE 的 POS、CURVE 的 CURTYP），以及立柱 HRPOST / HRTERM /
///   HRGATE 与面板 HRPANE 自己（POSREF / ZDIS / HRJUSL / SPRE / WIDT / XOFF / ZOFF）；
/// - 名下有 `JLDATU` 的 WALL（`elmodl::fixing_host_frame`，gen-model issue #37）：宿主自己（`JUSL` /
///   `LMIRR` 与 SPRE 目录截面的 p-line → PLDATU 标架），它的 SPINE 路径（POINSP / CURVE 的 POS），以及 datum
///   自己（JLDATU 的 ZDIS、PLDATU 的 POSL / YDIR）。墙 datum 下的 FIXING 自成单元，实例标架与开孔刀都由它现算，这些输入全在 FIXING 单元之上。
/// - 名下有 `PFIT` 的板（FLOOR / PANE / GWALL，`catalogue::panel_surface_z`）：板自己（第一个环上没有 `HEIG`
///   时的 `HEIG`）与它的 PLOO / LOOP（第一个环的 `HEIG` / `SJUS`）。PFIT 自成单元，贴在板 SJUS 指的那一面上；
///   板的顶点不在它的输入里（`profile::loop_extent` 不读顶点），改 PAVE / VERT 只上卷到板。
///
/// 从候选上溯只许穿过上面这些 noun；起点在这一端不存在也是 `None`。
///
/// # 为什么现算标架要单独一条腿
///
/// 这些元素没有 POS/ORI，[`crate::transform::world_matrix`] 按属主现算它们的标架，挂在
/// 下面的 FIXING / TMPL 件都吃这个标架。输入要么不是 POS/ORI，要么在宿主的**兄弟**子树
/// （SPINE 路径）里：改一只 POINSP，`nearest_unit` 上卷到 GENSEC 只重建了梁自己，
/// `placement_input_changed` 只认 POS/ORI，datum 底下的 FIXING 一个都不会重算；STWALL 的
/// POSS 挪了，墙上 FITT 模板里的件同样停在原处。详图式扶手同型：立柱的模板件自成单元，
/// 挪一只 POINTR 或改立柱的 ZDIS，`nearest_unit` 上卷到 HANDRA 只重建布局式那一件实体，
/// 立柱模板里的 EXTR / CYLI / GENSEC / PANE（aps250110 栏杆 ~1780 件）全停在旧站位。
///
/// 判据照桥架那条取最保守也最便宜的一档：命中就把整棵宿主子树级联重建。穿过 FIXING / TMPL
/// （或任何别的 noun）就不算——那以下的改动只影响它自己的子树，owner 链上的级联够得着。
/// 布局式 HANDRA（只有 RPATH，没有立柱）不算：它自己就是单元，RPATH 改了上卷到它就够。
fn implicit_frame_host_of(set: &Arc<DbSet>, refno: RefNo) -> Result<Option<RefNo>, DbElementError> {
    let mut current = set.element(refno);
    match current.exists() {
        Ok(true) => {}
        Ok(false) | Err(DbElementError::NotFound { .. }) => return Ok(None),
        Err(error) => return Err(error),
    }
    let owns = |host: &DbElement, noun: &str| -> Result<Option<RefNo>, DbElementError> {
        for member in host.members()? {
            if member.element_type()? == noun {
                return Ok(Some(host.refno()));
            }
        }
        Ok(None)
    };
    // 详图式扶手：立柱集 POSTSE、面板集 HRPNSE、门 HRFEAT，或 RAILSE 里夹着的端头 HRTERM，任一在场就算。
    let detailed_handrail = |host: &DbElement| -> Result<Option<RefNo>, DbElementError> {
        for member in host.members()? {
            match member.element_type()?.as_str() {
                "POSTSE" | "HRPNSE" | "HRFEAT" => return Ok(Some(host.refno())),
                "RAILSE" => {
                    for rail in member.members()? {
                        if rail.element_type()? == "HRTERM" {
                            return Ok(Some(host.refno()));
                        }
                    }
                }
                _ => {}
            }
        }
        Ok(None)
    };
    // POINSP → SPINE → GENSEC 与 PLDATU → JLDATU → GENSEC 都是两跳（WALL 同）；POINTR → RPATH → HANDRA、
    // HRPOST → POSTSE → HANDRA、HRTERM → RAILSE → HANDRA、HRGATE → HRFEAT → HANDRA 也是。
    for _ in 0..3 {
        match current.element_type()?.as_str() {
            // WALL 与 GENSEC 同型：datum（JLDATU → PLDATU）挂在宿主名下，FIXING 的标架沿宿主 spine 现算。
            // GENSEC 的端部 datum（ENDATU）不经 JLDATU，直接挂在宿主名下、同一条 spine 现算。
            "GENSEC" => return Ok(owns(&current, "JLDATU")?.or(owns(&current, "ENDATU")?)),
            "WALL" => return owns(&current, "JLDATU"),
            "SCTN" | "STWALL" => return owns(&current, "FITT"),
            "HANDRA" => return detailed_handrail(&current),
            // 板上配件 PFIT 贴在属主板 SJUS 指的那一面，吃板第一个环的 HEIG / SJUS。
            "FLOOR" | "PANE" | "GWALL" => return owns(&current, "PFIT"),
            "SPINE" | "POINSP" | "CURVE" | "JLDATU" | "PLDATU" | "ENDATU" | "FITT" | "PLOO"
            | "LOOP" => {}
            "RPATH" | "POINTR" | "POSTSE" | "HRPNSE" | "RAILSE" | "HRFEAT" | "HRPOST"
            | "HRPANE" | "HRTERM" | "HRGATE" => {}
            _ => return Ok(None),
        }
        match current.owner()? {
            Some(owner) if owner.refno() != current.refno() => current = owner,
            _ => return Ok(None),
        }
    }
    Ok(None)
}

/// [`joint_consumer_of`] 找到的那根梁。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum JointConsumer {
    /// 同库的 GENSEC，这一端的 JOIS / JOIE 指着这只 FIXING。
    Beam(RefNo),
    /// `CREF` 指到别的库：梁不在这个单库窗里，重建不了，只能点名。
    OtherDatabase(RefNo),
}

/// FIXING（在 `set` 这一端）是某根 GENSEC 的 `JOIS` / `JOIE` 关节时，那根梁。不是 FIXING、
/// 这一端不存在、`CREF` 空、或 `CREF` 指的梁这一端的 JOIS / JOIE 不指回它 → `None`。
///
/// # 为什么关节要单独一条腿
///
/// 梁从 JOIS / JOIE 指的 FIXING 那里取走它 TMPL 里的负体，扣自己的两端
/// （`elmodl::collect_joint_negatives`）。梁的几何因此吃 FIXING 的整棵子树（TMPL 负体的尺寸、
/// 位姿、NAPP、LEVE）与 FIXING 的世界标架——FIXING 挂在**另一根** GENSEC 的 datum 下，标架吃
/// 那根的 SPINE / JUSL / …（[`implicit_frame_host_of`]）。这些都不在梁的 owner 链上，也不在梁的
/// 子树里：改一只 NBOX，`nearest_unit` 上卷到 FIXING 只重建了端板，梁的端头停在旧刀口。
///
/// JOIS / JOIE 没有反向引用表（aps250110 的会话目录里只有 SPRE / HSTU / LSTU / CATR / … 这些），
/// 反查走 E3D 自己维护的回指：连接时 FIXING 的 `CREF` 写成那根梁（aps250110 的 522 个关节，
/// 522 个回指对上）。回指只当线索，认不认以梁这一端的 JOIS / JOIE 为准，陈旧的 CREF 只会漏认、
/// 不会错认。
fn joint_consumer_of(
    set: &Arc<DbSet>,
    refno: RefNo,
) -> Result<Option<JointConsumer>, DbElementError> {
    let fixing = set.element(refno);
    match fixing.exists() {
        Ok(true) => {}
        Ok(false) | Err(DbElementError::NotFound { .. }) => return Ok(None),
        Err(error) => return Err(error),
    }
    if fixing.element_type()? != "FIXING" {
        return Ok(None);
    }
    let Some(beam) = fixing.get_ref("CREF")? else {
        return Ok(None);
    };
    if beam.dbno() != refno.dbno() {
        return Ok(Some(JointConsumer::OtherDatabase(beam)));
    }
    let beam = set.element(beam);
    match beam.exists() {
        Ok(true) => {}
        Ok(false) | Err(DbElementError::NotFound { .. }) => return Ok(None),
        Err(error) => return Err(error),
    }
    if beam.element_type()? != "GENSEC" {
        return Ok(None);
    }
    if beam.get_ref("JOIS")? == Some(refno) || beam.get_ref("JOIE")? == Some(refno) {
        return Ok(Some(JointConsumer::Beam(beam.refno())));
    }
    Ok(None)
}

/// GENSEC（在 `set` 这一端）的 `JOIS` / `JOIE` 指着、`CREF` 又回指它的同库 FIXING——
/// [`joint_consumer_of`] 的反方向。不是 GENSEC、这一端不存在、关节空或不回指 → 空。
///
/// # 为什么梁变了关节 FIXING 也要重建
///
/// FIXING 的目录件求值经 `CREF` 读这根梁：`APAR` / `ADES`（梁的型材参数，e3d-io 参数环境）与
/// `AANGXZ` / `AANGYZ`（梁在关节那一端的轴向，`elmodl::attachment_angles`）。檐口加腋节点的刀与加腋板
/// 都吃它们。FIXING 挂在**柱**的 datum 下，不在梁的子树里：梁换了型材、SPINE 改了坡，
/// `nearest_unit` 上卷到梁为止，FIXING 与柱上按它扣的孔都停在旧处。
fn joint_fixings_of(set: &Arc<DbSet>, refno: RefNo) -> Result<Vec<RefNo>, DbElementError> {
    let beam = set.element(refno);
    match beam.exists() {
        Ok(true) => {}
        Ok(false) | Err(DbElementError::NotFound { .. }) => return Ok(Vec::new()),
        Err(error) => return Err(error),
    }
    if beam.element_type()? != "GENSEC" {
        return Ok(Vec::new());
    }
    let mut fixings = Vec::new();
    for joint in [beam.get_ref("JOIS")?, beam.get_ref("JOIE")?]
        .into_iter()
        .flatten()
    {
        if joint.dbno() != refno.dbno() {
            continue;
        }
        let fixing = set.element(joint);
        match fixing.exists() {
            Ok(true) => {}
            Ok(false) | Err(DbElementError::NotFound { .. }) => continue,
            Err(error) => return Err(error),
        }
        if fixing.element_type()? == "FIXING" && fixing.get_ref("CREF")? == Some(refno) {
            fixings.push(joint);
        }
    }
    Ok(fixings)
}

/// [`datum_fixing_host_of`] 从 FIXING 往上最多爬几层。aps250110 的 598 只全是
/// GENSEC → JLDATU → PLDATU → FIXING（三层）；8 只是防环。
const DATUM_FIXING_CLIMB: usize = 8;

/// FIXING（在 `set` 这一端）挂在某根 GENSEC 自己的 datum 树里时，那根 GENSEC。不是 FIXING、
/// 这一端不存在、上溯先碰到 `TMPL` / `SPINE`（FIXING 模板里的件、路径）或根本没有 GENSEC → `None`。
/// 与 `elmodl::collect_datum_fixings` 的下钻口径互逆：那边只不进 SPINE / TMPL 两种子树。
///
/// # 为什么宿主 FIXING 要单独一条腿
///
/// GENSEC 建模时扫自己 datum 树里的每只 FIXING，扣它 TMPL 里的螺栓孔、它目录件 `NGMR` 里的
/// 底板让位（`elmodl::collect_host_fixing_negatives`）。FIXING 是单元，改它、改它模板里的一只
/// NCYL，`nearest_unit` 上卷到 FIXING 为止，只重建了 FIXING 自己；它上面那根柱是另一个单元，
/// 孔与底面停在旧处。[`implicit_frame_host_of`] 管的是反方向（宿主变 → datum 下的件跟着变）。
fn datum_fixing_host_of(set: &Arc<DbSet>, refno: RefNo) -> Result<Option<RefNo>, DbElementError> {
    let fixing = set.element(refno);
    match fixing.exists() {
        Ok(true) => {}
        Ok(false) | Err(DbElementError::NotFound { .. }) => return Ok(None),
        Err(error) => return Err(error),
    }
    if fixing.element_type()? != "FIXING" {
        return Ok(None);
    }
    let mut current = fixing;
    for _ in 0..DATUM_FIXING_CLIMB {
        let owner = match current.owner()? {
            Some(owner) if owner.refno() != current.refno() => owner,
            _ => return Ok(None),
        };
        match owner.element_type()?.as_str() {
            "GENSEC" => return Ok(Some(owner.refno())),
            "TMPL" | "SPINE" => return Ok(None),
            _ => current = owner,
        }
    }
    Ok(None)
}

/// 候选（含自身）上溯碰到的最近 `CWBRAN`。不在任何桥架分支里 → `None`；
/// 起点在这一端不存在也是 `None`（Inserted 在 base、Deleted 在 target 是常态）。
///
/// # 为什么桥架要单独一条腿
///
/// 桥架件（CTSTRA/CTBEND/…）的模板里没有 POS/ORI，E3D 把放置注册成伪属性现算
/// （[`crate::cable_tray::placement`]）：输入是自己的 `POSREF/PKDI/ZDIS/ZOFF/BANG/GANGLE`、
/// `POSREF` 链上**别的桥架件**的 `PKDI/ZDIS`、以及 `CWBRAN` 名下 `RPATH → POINTR` 那一片
/// **兄弟**子树（`PPOS/PDIR/VERTDI/CURTYP/UBULG`）加 `CWBRAN` 自己的 `WTHICK/CLOSEF/VSHAPE`。
/// 这些边一条都不在 owner 链上：改一个 `POINTR`，`nearest_unit` 上卷到 RPATH/CWBRAN 都不是
/// 单元、`placement_input_changed` 只认 POS/ORI，于是落 `no_model`，而它旁边每一只
/// `CTRAY` 的世界矩阵都已经变了（`transform.rs` 记了两个月的「已知欠账」）。
///
/// 判据取最保守也最便宜的一档：**候选落在某个 `CWBRAN` 子树里（且自己不是 `CTRAY`），
/// 就把整个 `CWBRAN` 子树级联重建**。一条 `CWBRAN` 通常几十个成员，逐条依赖建图不值；
/// Core3D 那边伪属性根本没有变更检测，重画就是整条重算，我们只是把「整条」显式写出来。
/// `CTRAY` 自身的改动（SPRE 换目录件）只影响自己，上卷到自己就够，不触发整条。
fn cable_tray_branch_of(set: &Arc<DbSet>, refno: RefNo) -> Result<Option<RefNo>, DbElementError> {
    let mut current = set.element(refno);
    match current.exists() {
        Ok(true) => {}
        Ok(false) | Err(DbElementError::NotFound { .. }) => return Ok(None),
        Err(error) => return Err(error),
    }
    let mut depth = 0usize;
    loop {
        if current.element_type()? == CABLE_TRAY_BRANCH_NOUN {
            return Ok(Some(current.refno()));
        }
        match current.owner()? {
            Some(owner) if owner.refno() == current.refno() => return Ok(None),
            Some(owner) => {
                depth += 1;
                if depth > MAX_ANCESTOR_DEPTH {
                    return Err(DbElementError::OwnerCycle {
                        start: refno,
                        at: owner.refno(),
                    });
                }
                current = owner;
            }
            None => return Ok(None),
        }
    }
}

/// 把索引候选折算成模型单元动作。只读两端 `DbSet`，不建任何几何。不设候选上限
/// （[`PlanLimits::default`]）——真库门与探针要的就是「无论多大都逐候选给出结论」。
pub fn plan_update(base: &Arc<DbSet>, target: &Arc<DbSet>, window: &WindowDiff) -> UpdatePlan {
    plan_update_bounded(base, target, window, PlanLimits::default())
}

/// 带护栏的 [`plan_update`]：候选数超过 `limits.max_candidates` 就不逐候选分类，
/// 返回一份 [`UpdatePlan::full_rebuild`] 非空的空计划，报告记 `degraded_to_full`。
pub fn plan_update_bounded(
    base: &Arc<DbSet>,
    target: &Arc<DbSet>,
    window: &WindowDiff,
    limits: PlanLimits,
) -> UpdatePlan {
    plan_update_with(base, target, window, limits, |_| None)
}

/// 调用方「不影响模型」谓词对一个 Modified 候选的回答里带的档位（[`plan_update_with`]）。
///
/// 去向只有一个——记 `no_model`、不上卷、不级联；档位只决定横切计数落在
/// [`IncrementReport::metadata_only`]、[`IncrementReport::unread_attribute`] 还是
/// [`IncrementReport::cache_stamp`]。哪些属性算哪一档由调用方说（gen-model `model_impact`），本 crate 只数。
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum NoModelTier {
    /// 老名单（2026-09-08 立的门）：改动全部落在业务元数据 NAME / DESC / PURP / FUNC / LCHKDA 上。
    MetadataOnly,
    /// 新档（2026-09-15 S6）：至少一个属性是「E3D 字典 DCHC=0 ∧ 生成器不读」而不在老名单里。
    UnreadAttribute,
    /// 第三档（2026-09-17 Q6）：改动**只**落在戳上（[`crate::element_diff::PLACEMENT_STAMP_ATTRIBUTES`]，
    /// 今天只有 `CACHID`）——Design 应用随每次图形重建写下的缓存 `+1` 计数，写方不是设计变更，
    /// 生成器一处不读。戳混在别的放行属性里时不改档（跟着 `NAME` 记老档、跟着 `DUTY` 记新档），
    /// 只有一个字都没动、就戳跳了一下的才记这里。
    CacheStamp,
}

/// [`plan_update_bounded`] 外加一道调用方给的「不影响模型」谓词（gen-model 2026-09-08；
/// 2026-09-15 S6 起回答带档位）。
///
/// `metadata_only(diff)` 对一个 **Modified** 候选回 `Some(档)` = 这次改动全部落在不影响模型的
/// 属性上（NAME / DESC 之类的业务元数据，或 core 不重画、生成器也不读的属性，或只有戳 `CACHID`
/// 跳了一下），成员 / 属主 / 类型 / opaque 都没动：**不上卷、不级联**，记 `no_model`，档位进横切计数
/// [`IncrementReport::metadata_only`] / [`IncrementReport::unread_attribute`] /
/// [`IncrementReport::cache_stamp`]。哪些属性算不影响模型、
/// 算哪一档由调用方说——本 crate 不认识 gen-model 的效果表、E3D 字典的 DCHC 与生成读取集合，名单只写一份。
///
/// 三条不变量，调用方与本函数各守一半：
/// - 账本**照记**（`ledger.record` 在谓词之前）：这些候选仍是 [`UpdatePlan::affected_closure`]
///   的种子，「根不在闭包里 = 子树什么都没动」的凭证前移判据一字不改，只是不再把它们上卷成
///   `regenerate`、也不再从它们出发级联（桥架分支「子树里改了任何东西就整条重建」那类按 noun
///   不按属性判的扇出随之省掉——那片单元不进闭包、它们的根由调用方按两票前移，gen-model 的
///   全语料两态门钉「闭包只少不多、少掉的全是少重建的单元及其祖先」）；房间结构触发那类读账本的
///   消费者也照常看得见它们。
/// - 谓词只对 Modified 候选起作用：Inserted / Deleted 没有「只改了元数据」这一说。
/// - 谓词返回 `Some` 的候选必须真的没有结构变化——那是调用方谓词的责任，这里不复核；
///   `|_| None` 就是 [`plan_update_bounded`]。
pub fn plan_update_with(
    base: &Arc<DbSet>,
    target: &Arc<DbSet>,
    window: &WindowDiff,
    limits: PlanLimits,
    metadata_only: impl Fn(&ElementDiff) -> Option<NoModelTier>,
) -> UpdatePlan {
    let mut report = IncrementReport {
        base_sesno: window.base_sesno,
        target_sesno: window.target_sesno,
        candidates: window.diff.changes.len(),
        inserted: window.diff.tally.inserted as usize,
        modified: window.diff.tally.modified as usize,
        deleted: window.diff.tally.deleted as usize,
        flag_only_drifts: window.diff.diagnostics.flag_only_drifts,
        ..IncrementReport::default()
    };
    if let Some(limit) = limits.max_candidates
        && report.candidates > limit
    {
        let reason = format!(
            "candidates {} > max_candidates {limit}; window {}→{} degraded to a full rebuild",
            report.candidates, window.base_sesno, window.target_sesno
        );
        report.degraded_to_full = Some(reason.clone());
        return UpdatePlan {
            regenerate: BTreeSet::new(),
            remove: BTreeSet::new(),
            regenerate_derived: BTreeSet::new(),
            derived_stale: BTreeSet::new(),
            ledger: ChangeLedger::new(),
            full_rebuild: Some(reason),
            report,
        };
    }
    let mut ledger = ChangeLedger::new();
    let mut regenerate: BTreeSet<(u32, u32)> = BTreeSet::new();
    let mut remove: BTreeSet<(u32, u32)> = BTreeSet::new();
    let mut regenerate_derived: BTreeSet<(u32, u32)> = BTreeSet::new();
    let mut visits = CascadeVisits::default();
    // 删除候选上卷到的属主单元。属主自己也被删时要归并进那条 Remove，
    // 所以等 remove 集收齐了再定去向。
    let mut deferred: Vec<RefNo> = Vec::new();

    fn note_no_model(report: &mut IncrementReport, noun: &str) {
        *report.no_model.entry(noun.to_string()).or_insert(0) += 1;
    }

    /// 上卷跨过了路由容器就把容器记进派生集。返回值 = 这个候选折进了派生动作
    /// （去重不影响折算：容器已在计划里，候选照样算 rolled_up）。
    fn mark_derived(derived: &mut BTreeSet<(u32, u32)>, rollup: &Rollup) -> bool {
        match &rollup.route_container {
            Some((container, _)) => {
                derived.insert(key(*container));
                true
            }
            None => false,
        }
    }

    /// 一个候选的收尾：去向进账本，计数进报告。`no_model` 的 noun 用候选自己的。
    fn settle(
        report: &mut IncrementReport,
        ledger: &mut ChangeLedger,
        refno: RefNo,
        noun: &str,
        contributed: bool,
    ) {
        if contributed {
            report.rolled_up += 1;
            ledger.dispose(refno, Disposition::RolledUp);
        } else {
            note_no_model(report, noun);
            ledger.dispose(refno, Disposition::NoModel);
        }
    }

    /// 桥架分支级联（见 [`cable_tray_branch_of`]）：候选在 `lookup` 这一端落在某个 CWBRAN
    /// 子树里、且自己不是 CTRAY → 在 target 端把那个 CWBRAN 的子树整个收进计划。
    /// 返回这次级联折进了什么（`None` = 与桥架无关，或 CWBRAN 在 target 端已不存在
    /// ——那时它名下的托盘都是 Deleted 候选，各自出 Remove）。
    #[allow(clippy::too_many_arguments)]
    fn cascade_cable_tray_branch(
        lookup: &Arc<DbSet>,
        target: &Arc<DbSet>,
        refno: RefNo,
        noun: &str,
        regenerate: &mut BTreeSet<(u32, u32)>,
        regenerate_derived: &mut BTreeSet<(u32, u32)>,
        report: &mut IncrementReport,
        visits: &mut CascadeVisits,
    ) -> Result<Option<CascadeFanout>, DbElementError> {
        if noun.eq_ignore_ascii_case("CTRAY") {
            return Ok(None);
        }
        let Some(branch) = cable_tray_branch_of(lookup, refno)? else {
            return Ok(None);
        };
        match target.element(branch).exists() {
            Ok(true) => {}
            Ok(false) | Err(DbElementError::NotFound { .. }) => return Ok(None),
            Err(error) => return Err(error),
        }
        // 同一条分支本窗已经整条收过：这个候选照样算折进了计划，但不再遍历、不再记账。
        if !visits.tray_branches.insert(key(branch)) {
            return Ok(Some(CascadeFanout {
                hits: 0,
                overlapped: 1,
            }));
        }
        let fanout = collect_unit_subtree(
            target,
            branch,
            regenerate,
            regenerate_derived,
            &mut report.notes,
            visits,
        )?;
        report.cascades.push(incident(
            refno,
            noun,
            format!(
                "桥架放置输入变了（{noun} 落在 CWBRAN {branch} 子树里），级联重建整条分支 \
                 {} 个单元{}",
                fanout.hits,
                if fanout.overlapped > 0 {
                    format!("，另 {} 处已由本窗更早的级联覆盖", fanout.overlapped)
                } else {
                    String::new()
                }
            ),
        ));
        report.cascade_hits += fanout.hits;
        Ok(Some(fanout))
    }

    /// 现算标架级联（见 [`implicit_frame_host_of`]）：候选在 `lookup` 这一端是某个现算标架
    /// 的输入 → 在 target 端把整棵宿主子树收进计划。
    #[allow(clippy::too_many_arguments)]
    fn cascade_implicit_frames(
        lookup: &Arc<DbSet>,
        target: &Arc<DbSet>,
        refno: RefNo,
        noun: &str,
        regenerate: &mut BTreeSet<(u32, u32)>,
        regenerate_derived: &mut BTreeSet<(u32, u32)>,
        report: &mut IncrementReport,
        visits: &mut CascadeVisits,
    ) -> Result<Option<CascadeFanout>, DbElementError> {
        let Some(host) = implicit_frame_host_of(lookup, refno)? else {
            return Ok(None);
        };
        match target.element(host).exists() {
            Ok(true) => {}
            Ok(false) | Err(DbElementError::NotFound { .. }) => return Ok(None),
            Err(error) => return Err(error),
        }
        if !visits.implicit_hosts.insert(key(host)) {
            return Ok(Some(CascadeFanout {
                hits: 0,
                overlapped: 1,
            }));
        }
        let fanout = collect_unit_subtree(
            target,
            host,
            regenerate,
            regenerate_derived,
            &mut report.notes,
            visits,
        )?;
        report.cascades.push(incident(
            refno,
            noun,
            format!(
                "现算标架的输入变了（{noun} 落在宿主 {host} 名下），级联重建整棵宿主子树 {} 个单元{}",
                fanout.hits,
                if fanout.overlapped > 0 {
                    format!("，另 {} 处已由本窗更早的级联覆盖", fanout.overlapped)
                } else {
                    String::new()
                }
            ),
        ));
        report.cascade_hits += fanout.hits;
        Ok(Some(fanout))
    }

    /// 读不出来的候选：报告与账本各记一条，去向 `Unresolved`。账本上已经有它的
    /// 条目（L2 差分成功、L4 上卷才失败）就不再补 `Unresolved` 条目，只登记去向。
    fn give_up(
        report: &mut IncrementReport,
        ledger: &mut ChangeLedger,
        refno: RefNo,
        noun: &str,
        detail: String,
        already_ledgered: bool,
    ) {
        report
            .unresolved
            .push(incident(refno, noun, detail.clone()));
        if !already_ledgered {
            ledger.unresolved(refno, noun, detail);
        }
        ledger.dispose(refno, Disposition::Unresolved);
    }

    for change in &window.diff.changes {
        match *change {
            IndexCandidate::Inserted { refno, .. } => match nearest_unit(target, refno) {
                Ok(rollup) => {
                    ledger.created(refno, rollup.noun.clone(), rollup.owner);
                    let mut contributed = mark_derived(&mut regenerate_derived, &rollup);
                    if let Some(unit) = rollup.unit {
                        regenerate.insert(key(unit));
                        contributed = true;
                    }
                    // 新插进桥架分支的路径点 / 桥架件：整条分支的放置都变了。
                    // 新插进 GENSEC SPINE 的点、新的 JLDATU / PLDATU：datum 下的 FIXING 都挪了位。
                    let cascades = cascade_cable_tray_branch(
                        target,
                        target,
                        refno,
                        &rollup.noun,
                        &mut regenerate,
                        &mut regenerate_derived,
                        &mut report,
                        &mut visits,
                    )
                    .map_err(|error| format!("新增候选的桥架分支级联失败：{error}"))
                    .and_then(|tray| {
                        cascade_implicit_frames(
                            target,
                            target,
                            refno,
                            &rollup.noun,
                            &mut regenerate,
                            &mut regenerate_derived,
                            &mut report,
                            &mut visits,
                        )
                        .map(|datum| [tray, datum])
                        .map_err(|error| format!("新增候选的现算标架级联失败：{error}"))
                    });
                    match cascades {
                        Ok(fanouts) => {
                            let cascaded = fanouts
                                .iter()
                                .flatten()
                                .any(|fanout| fanout.hits > 0 || fanout.overlapped > 0);
                            if cascaded && !contributed {
                                report.cascade_only += 1;
                            }
                            contributed |= cascaded;
                            settle(&mut report, &mut ledger, refno, &rollup.noun, contributed);
                        }
                        Err(detail) => {
                            give_up(&mut report, &mut ledger, refno, &rollup.noun, detail, true)
                        }
                    }
                }
                Err(error) => give_up(
                    &mut report,
                    &mut ledger,
                    refno,
                    "?",
                    format!("新增候选上卷失败：{error}"),
                    false,
                ),
            },
            IndexCandidate::Deleted { refno, .. } => match nearest_unit(base, refno) {
                Ok(rollup) => {
                    ledger.deleted(refno, rollup.noun.clone(), rollup.owner);
                    // 容器记的是 base 端的链：被删的路由构件只有 base 认得它挂在哪条
                    // 链上；容器自己被删也走这条（执行侧 retired，旧键全清）。
                    let mut contributed = mark_derived(&mut regenerate_derived, &rollup);
                    match rollup.unit {
                        // 单元自己被删 → 直接 Remove（Core R15 同款，不再上卷）。
                        Some(unit) if unit == refno => {
                            remove.insert(key(unit));
                            contributed = true;
                        }
                        Some(unit) => {
                            deferred.push(unit);
                            contributed = true;
                        }
                        None => {}
                    }
                    // 从桥架分支里删掉一个路径点 / 桥架件：分支还在的话，剩下的托盘
                    // 全都挪了位。分支在 base 端找，级联在 target 端做。GENSEC 的
                    // SPINE 点 / datum 被删同理：宿主在 base 端找，级联在 target 端做。
                    let cascades = cascade_cable_tray_branch(
                        base,
                        target,
                        refno,
                        &rollup.noun,
                        &mut regenerate,
                        &mut regenerate_derived,
                        &mut report,
                        &mut visits,
                    )
                    .map_err(|error| format!("删除候选的桥架分支级联失败：{error}"))
                    .and_then(|tray| {
                        cascade_implicit_frames(
                            base,
                            target,
                            refno,
                            &rollup.noun,
                            &mut regenerate,
                            &mut regenerate_derived,
                            &mut report,
                            &mut visits,
                        )
                        .map(|datum| [tray, datum])
                        .map_err(|error| format!("删除候选的现算标架级联失败：{error}"))
                    });
                    match cascades {
                        Ok(fanouts) => {
                            let cascaded = fanouts
                                .iter()
                                .flatten()
                                .any(|fanout| fanout.hits > 0 || fanout.overlapped > 0);
                            if cascaded && !contributed {
                                report.cascade_only += 1;
                            }
                            contributed |= cascaded;
                            settle(&mut report, &mut ledger, refno, &rollup.noun, contributed);
                        }
                        Err(detail) => {
                            give_up(&mut report, &mut ledger, refno, &rollup.noun, detail, true)
                        }
                    }
                }
                Err(error) => give_up(
                    &mut report,
                    &mut ledger,
                    refno,
                    "?",
                    format!("删除候选上卷失败：{error}"),
                    false,
                ),
            },
            IndexCandidate::Modified { refno, .. } => {
                // L2：索引只说记录被重写过，这一步才回答到底哪里变了。
                let diff = match diff_element(base, target, refno) {
                    Ok(diff) => diff,
                    Err(error) => {
                        give_up(
                            &mut report,
                            &mut ledger,
                            refno,
                            "?",
                            format!("逐元素差分失败：{error}"),
                            false,
                        );
                        continue;
                    }
                };
                ledger.record(refno, &diff);
                if diff.is_unchanged() {
                    report.unchanged += 1;
                    continue;
                }
                // 调用方的「不影响模型」门（见 [`plan_update_with`]）：改动全部落在不影响模型的属性上、
                // 无结构变化 → 记 no_model，不上卷、不级联。账本上面已经记过，闭包种子照收。
                if let Some(tier) = metadata_only(&diff) {
                    match tier {
                        NoModelTier::MetadataOnly => report.metadata_only += 1,
                        NoModelTier::UnreadAttribute => report.unread_attribute += 1,
                        NoModelTier::CacheStamp => report.cache_stamp += 1,
                    }
                    settle(&mut report, &mut ledger, refno, &diff.noun, false);
                    continue;
                }

                let mut contributed = false;
                let mut resolved = true;
                // 级联之外有没有折进什么：自身 / 上方的单元或容器，或改挂 / 类型变更
                // 回 base 端找到的旧单元。级联单独记，才分得出「只靠级联」的那些。
                let mut direct_contribution = false;
                match nearest_unit(target, refno) {
                    Ok(rollup) => {
                        contributed = mark_derived(&mut regenerate_derived, &rollup);
                        if let Some(unit) = rollup.unit {
                            regenerate.insert(key(unit));
                            contributed = true;
                        }
                        direct_contribution = contributed;
                    }
                    Err(error) => {
                        resolved = false;
                        give_up(
                            &mut report,
                            &mut ledger,
                            refno,
                            &diff.noun,
                            format!("修改候选上卷失败：{error}"),
                            true,
                        );
                    }
                }

                // 改挂：旧属主那一侧的单元也要重建。core 的
                // `DB_UserChanges::elementIncluded(el, 旧属主)` 记的正是旧属主——
                // 一个 PLOO 从 PANE1 挪到 PANE2，PANE1 少了一个剖面环，同样得重画。
                // 只上卷 target 端就永远看不见 PANE1。
                //
                // 类型变更同款回 base 端：它从前是什么只有 base 说得出。BRAN 变成
                // 别的 noun，旧管身的键只有 base 端的链认得（容器进派生集，执行侧
                // retired 清旧键）；单元变成非单元，旧几何也得有人发 Remove
                // （base 端上卷进 regenerate，执行侧按 target 分类走 skip + Remove）。
                if resolved && (diff.owner_changed.is_some() || diff.type_changed.is_some()) {
                    match nearest_unit(base, refno) {
                        Ok(rollup) => {
                            contributed |= mark_derived(&mut regenerate_derived, &rollup);
                            if let Some(unit) = rollup.unit {
                                regenerate.insert(key(unit));
                                contributed = true;
                            }
                            direct_contribution |= contributed;
                        }
                        Err(error) => {
                            resolved = false;
                            give_up(
                                &mut report,
                                &mut ledger,
                                refno,
                                &diff.noun,
                                format!("改挂/类型变更后回溯 base 端失败：{error}"),
                                true,
                            );
                        }
                    }
                }

                // 变换级联（见模块文档「与 Core3D 的一处有意分歧」）。
                let mut cascade_contributed = false;
                if resolved && diff.placement_input_changed() {
                    match collect_unit_subtree(
                        target,
                        refno,
                        &mut regenerate,
                        &mut regenerate_derived,
                        &mut report.notes,
                        &mut visits,
                    ) {
                        Ok(fanout) => {
                            report.cascades.push(incident(
                                refno,
                                diff.noun.clone(),
                                format!(
                                    "{}，级联重建子树内 {} 个单元（含派生容器）{}",
                                    cascade_reason(&diff),
                                    fanout.hits,
                                    if fanout.overlapped > 0 {
                                        format!(
                                            "，另 {} 处已由本窗更早的级联覆盖",
                                            fanout.overlapped
                                        )
                                    } else {
                                        String::new()
                                    }
                                ),
                            ));
                            report.cascade_hits += fanout.hits;
                            // 子树里有单元、或子树早被别的级联收进计划，这个候选都算
                            // 折进了计划；两样都没有才是「改动落在没有模型归属的元素上」。
                            cascade_contributed |= fanout.hits > 0 || fanout.overlapped > 0;
                        }
                        Err(error) => {
                            // 级联收集失败 = 有一片子树的世界矩阵可能已过期而我们
                            // 点不出名字——这不是 note 级别的观察，必须亮成
                            // unresolved 才对得起「静默缺件」那条宪法。
                            resolved = false;
                            give_up(
                                &mut report,
                                &mut ledger,
                                refno,
                                &diff.noun,
                                format!("世界矩阵输入变了但级联收集失败：{error}"),
                                true,
                            );
                        }
                    }
                }

                // 桥架分支级联：候选在 CWBRAN 子树里（RPATH/POINTR/桥架件/CWBRAN 自己）
                // 改了任何东西，整条分支的托盘放置都可能变——这些输入不在 owner 链上，
                // 上面那两步都够不着（见 `cable_tray_branch_of`）。
                if resolved {
                    match cascade_cable_tray_branch(
                        target,
                        target,
                        refno,
                        &diff.noun,
                        &mut regenerate,
                        &mut regenerate_derived,
                        &mut report,
                        &mut visits,
                    ) {
                        Ok(fanout) => {
                            cascade_contributed |= fanout
                                .is_some_and(|fanout| fanout.hits > 0 || fanout.overlapped > 0);
                        }
                        Err(error) => {
                            resolved = false;
                            give_up(
                                &mut report,
                                &mut ledger,
                                refno,
                                &diff.noun,
                                format!("桥架分支级联失败：{error}"),
                                true,
                            );
                        }
                    }
                }

                // 现算标架级联：候选是宿主（GENSEC / WALL / SCTN / STWALL / 挂 PFIT 的板）、GENSEC / WALL 的
                // SPINE 路径、JLDATU / PLDATU、FITT 或板的 PLOO / LOOP，挂在下面的件标架都可能变了
                // （见 `implicit_frame_host_of`）。
                if resolved {
                    match cascade_implicit_frames(
                        target,
                        target,
                        refno,
                        &diff.noun,
                        &mut regenerate,
                        &mut regenerate_derived,
                        &mut report,
                        &mut visits,
                    ) {
                        Ok(fanout) => {
                            cascade_contributed |= fanout
                                .is_some_and(|fanout| fanout.hits > 0 || fanout.overlapped > 0);
                        }
                        Err(error) => {
                            resolved = false;
                            give_up(
                                &mut report,
                                &mut ledger,
                                refno,
                                &diff.noun,
                                format!("现算标架级联失败：{error}"),
                                true,
                            );
                        }
                    }
                }

                if resolved {
                    if cascade_contributed && !direct_contribution {
                        report.cascade_only += 1;
                    }
                    contributed = contributed || cascade_contributed;
                    settle(&mut report, &mut ledger, refno, &diff.noun, contributed);
                }
            }
        }
    }

    for unit in deferred {
        // 属主也被删了 → 它自己的 Deleted 候选已经出了 Remove；还活着 → 重建。
        if !remove.contains(&key(unit)) {
            regenerate.insert(key(unit));
        }
    }
    // 反向关节级联（见 [`joint_fixings_of`]）：本窗要重建或删除的梁，它 JOIS / JOIE 指着的 FIXING
    // 经 CREF 读它，一并重建。放在下面两条之前：这里补进来的 FIXING 还要再带出它挂的柱；而由
    // FIXING 带出来的梁只是被扣的刀变了、属性没动，不必再回头重建它另一端的 FIXING。两端各问一次：
    // 梁被删了只有 base 认得它的关节。
    let beam_sources: Vec<(u32, u32)> = regenerate.iter().chain(remove.iter()).copied().collect();
    for unit in beam_sources {
        let beam = refno_of(unit);
        for side in [target, base] {
            let fixings = match joint_fixings_of(side, beam) {
                Ok(fixings) => fixings,
                Err(error) => {
                    report.notes.push(incident(
                        beam,
                        "GENSEC",
                        format!("反向关节级联读挂，关节 FIXING 可能停在旧几何：{error}"),
                    ));
                    continue;
                }
            };
            for fixing in fixings {
                match target.element(fixing).exists() {
                    Ok(true) => {}
                    Ok(false) | Err(DbElementError::NotFound { .. }) => continue,
                    Err(error) => {
                        report.notes.push(incident(
                            fixing,
                            "FIXING",
                            format!(
                                "梁 {beam} 要重建或已删除，它的关节 FIXING 在 target 端读不出，\
                                 可能停在旧几何：{error}"
                            ),
                        ));
                        continue;
                    }
                }
                if !regenerate.insert(key(fixing)) {
                    continue;
                }
                report.cascades.push(incident(
                    beam,
                    "GENSEC",
                    format!(
                        "关节 FIXING {fixing} 经 CREF 读这根梁（APAR / AANGYZ），梁要重建或已删除，它一并重建"
                    ),
                ));
                report.cascade_hits += 1;
            }
        }
    }
    // 关节级联（见 [`joint_consumer_of`]）：本窗要重建或删除的 FIXING 若是某根 GENSEC 的
    // JOIS / JOIE 关节，那根梁扣的端头负体跟着变。放在计划集收齐之后做，FIXING 从哪条腿进来
    // （自己改了、TMPL 里的负体改了、宿主 GENSEC 的现算标架级联）都一样接得住。两端各问一次：
    // 删掉的 FIXING 只有 base 认得，改接到别的梁时新旧两根都要重建。
    let joint_sources: Vec<(u32, u32)> = regenerate.iter().chain(remove.iter()).copied().collect();
    let mut joint_beams: BTreeSet<(u32, u32)> = BTreeSet::new();
    let mut joint_notes: BTreeSet<((u32, u32), (u32, u32))> = BTreeSet::new();
    for unit in joint_sources {
        let fixing = refno_of(unit);
        for side in [target, base] {
            match joint_consumer_of(side, fixing) {
                Ok(None) => {}
                Ok(Some(JointConsumer::Beam(beam))) => {
                    // 梁这一窗被删了：它自己的 Deleted 候选出 Remove。
                    match target.element(beam).exists() {
                        Ok(true) => {}
                        Ok(false) | Err(DbElementError::NotFound { .. }) => continue,
                        Err(error) => {
                            report.notes.push(incident(
                                beam,
                                "GENSEC",
                                format!(
                                    "关节 FIXING {fixing} 要重建或已删除，这根梁在 target 端读不出，\
                                     端头负体可能停在旧刀口：{error}"
                                ),
                            ));
                            continue;
                        }
                    }
                    if regenerate.contains(&key(beam)) || !joint_beams.insert(key(beam)) {
                        continue;
                    }
                    regenerate.insert(key(beam));
                    report.cascades.push(incident(
                        fixing,
                        "FIXING",
                        format!(
                            "关节负体的来源 FIXING 要重建或已删除，JOIS / JOIE 指着它的 GENSEC {beam} 一并重建"
                        ),
                    ));
                    report.cascade_hits += 1;
                }
                Ok(Some(JointConsumer::OtherDatabase(beam))) => {
                    if joint_notes.insert((unit, key(beam))) {
                        report.notes.push(incident(
                            fixing,
                            "FIXING",
                            format!(
                                "关节 FIXING 的 CREF 指到别的库（{beam}），单库窗重建不了那根梁，\
                                 它的端头负体可能停在旧刀口"
                            ),
                        ));
                    }
                }
                Err(error) => report.notes.push(incident(
                    fixing,
                    "FIXING",
                    format!("关节级联读挂，JOIS / JOIE 指着它的梁可能停在旧刀口：{error}"),
                )),
            }
        }
    }
    // 宿主 FIXING 级联（见 [`datum_fixing_host_of`]）：本窗要重建或删除的 FIXING 挂在某根
    // GENSEC 的 datum 树里，那根构件扣的螺栓孔 / 底板让位跟着变。同样放在计划集收齐之后、两端
    // 各问一次：新挂上的 FIXING 只有 target 认得，删掉的只有 base 认得。
    let host_sources: Vec<(u32, u32)> = regenerate.iter().chain(remove.iter()).copied().collect();
    for unit in host_sources {
        let fixing = refno_of(unit);
        for side in [target, base] {
            let host = match datum_fixing_host_of(side, fixing) {
                Ok(Some(host)) => host,
                Ok(None) => continue,
                Err(error) => {
                    report.notes.push(incident(
                        fixing,
                        "FIXING",
                        format!("宿主 FIXING 级联读挂，挂着它的 GENSEC 可能停在旧刀口：{error}"),
                    ));
                    continue;
                }
            };
            // 宿主这一窗被删了：它自己的 Deleted 候选出 Remove。
            match target.element(host).exists() {
                Ok(true) => {}
                Ok(false) | Err(DbElementError::NotFound { .. }) => continue,
                Err(error) => {
                    report.notes.push(incident(
                        host,
                        "GENSEC",
                        format!(
                            "宿主 FIXING {fixing} 要重建或已删除，这根构件在 target 端读不出，\
                             螺栓孔与底板让位可能停在旧处：{error}"
                        ),
                    ));
                    continue;
                }
            }
            if !regenerate.insert(key(host)) {
                continue;
            }
            report.cascades.push(incident(
                fixing,
                "FIXING",
                format!(
                    "宿主扫描负体的来源 FIXING 要重建或已删除，名下挂着它的 GENSEC {host} 一并重建"
                ),
            ));
            report.cascade_hits += 1;
        }
    }
    // Remove 赢：同一单元既要删又要建说明分类端点混了，删除语义优先。
    for gone in &remove {
        regenerate.remove(gone);
    }

    // 派生容器的旧槽位键 + 计划账。旧键只有 base 说了算（`execute_plan` 按设计
    // 只读 target），且收的是**全部槽位**而非「上次真建成的那批」——计划期不建
    // 几何，无从知道哪些槽位当时退化；多出来的 Remove 落在从未存在的键上，
    // 幂等无害（见模块文档「派生单元」）。
    let mut derived_stale: BTreeSet<GeometryId> = BTreeSet::new();
    for &container in &regenerate_derived {
        let refno = refno_of(container);
        let noun = target
            .element(refno)
            .element_type()
            .or_else(|_| base.element(refno).element_type())
            .unwrap_or_else(|_| "?".into());
        *report.derived_planned.entry(noun.clone()).or_insert(0) += 1;
        match route_slot_keys(&base.element(refno)) {
            Ok(slots) => {
                for slot in slots {
                    derived_stale.insert(tube_id(refno, slot));
                }
            }
            // 容器自己在 base 端不存在：这一窗新建的链，没有旧键可清。
            // 只豁免容器本身——链走到一半某个成员读不出是数据问题，得响。
            Err(DbElementError::NotFound { refno: missing, .. }) if missing == refno => {}
            Err(error) => {
                report.derived_stale_unreadable.push(incident(
                    refno,
                    noun,
                    format!("base 端旧槽位键读不出，旧管身可能清不掉：{error}"),
                ));
            }
        }
    }

    ledger.finish();
    report.changes = ledger.tally();
    report.ledger_refnos = ledger.distinct_refnos();
    report.units_to_regenerate = regenerate.len();
    report.units_to_remove = remove.len();
    report.derived_to_regenerate = regenerate_derived.len();
    UpdatePlan {
        regenerate,
        remove,
        regenerate_derived,
        derived_stale,
        ledger,
        full_rebuild: None,
        report,
    }
}

/// 级联理由的可读形态。报告里要说得出是「谁动了」，不能只有一个 fanout 数字。
fn cascade_reason(diff: &ElementDiff) -> String {
    if diff.type_changed.is_some() {
        return "类型变更".into();
    }
    if diff.owner_changed.is_some() {
        return "改挂（owner 链换了一条，局部 POS/ORI 未必动）".into();
    }
    if diff.opaque {
        return "记录变了但指不出哪一项，保守级联".into();
    }
    let moved: Vec<&str> = crate::transform::PLACEMENT_ATTRIBUTES
        .into_iter()
        .filter(|name| diff.attributes.iter().any(|changed| changed == name))
        .collect();
    format!("{} 变了", moved.join("/"))
}

/// 执行计划：逐单元重建。与全量管线走同一条 [`model_element`]，产出动作集；
/// 派生容器走 [`route_tubes_of`]（与全量同一段串接代码），旧键做差发 Remove。
///
/// 退化成整库重建的计划（[`UpdatePlan::full_rebuild`]）这里**不执行**：它没有任何
/// 动作可执行，返回的产出是空的、报告带 `degraded_to_full` 与一条 note。调用方拿到它
/// 该走全量生成，把空产出当「这一窗没变」发布出去就是静默缺件。
pub fn execute_plan(target: &Arc<DbSet>, plan: UpdatePlan) -> IncrementOutcome {
    let UpdatePlan {
        regenerate,
        remove,
        regenerate_derived,
        derived_stale,
        ledger: _,
        full_rebuild,
        mut report,
    } = plan;
    if let Some(reason) = full_rebuild {
        report.notes.push(Incident {
            refno: String::new(),
            noun: String::new(),
            detail: format!("计划已退化成整库重建，未执行任何单元动作：{reason}"),
        });
        return IncrementOutcome {
            upserts: Vec::new(),
            removals: Vec::new(),
            report,
        };
    }
    let mut upserts: Vec<GeneratedElement> = Vec::new();
    let mut removals: BTreeSet<GeometryId> = remove.into_iter().map(element_id).collect();
    // 一次执行共享一个世代网格池：级联重建（容器一挪、整片 PANE 重来）里
    // 同形状只建一次网格；命中克隆与直建逐位一致，不改任何单元的产出。
    let mut mesh_cache = CatalogueMeshCache::default();

    for unit in regenerate {
        let refno = refno_of(unit);
        let el = target.element(refno);
        let noun = match el.element_type() {
            Ok(noun) => noun,
            Err(error) => {
                removals.insert(element_id(unit));
                report
                    .regen_failed
                    .push(incident(refno, "?", format!("读不出 noun：{error}")));
                continue;
            }
        };
        let category = classify(&noun);
        let is_unit = match is_model_unit_of(&el) {
            Ok(is_unit) => is_unit,
            Err(error) => {
                removals.insert(element_id(unit));
                report
                    .regen_failed
                    .push(incident(refno, noun, format!("读不出属主：{error}")));
                continue;
            }
        };
        if !is_unit {
            // 合法到达：类型变更把 base 端的单元送了进来，而它在 target 端的新 noun
            // 不再出几何——与 `Modeled::Skipped` 同款处置（先擦后画，画不出保持
            // 擦掉），不算失败。计划集没被手工改过的话，这里只有类型变更（或 FITT 改挂到
            // SCTN / STWALL 之外的属主下、没了现算标架）能到。
            removals.insert(element_id(unit));
            report.regen_skipped.push(incident(
                refno,
                noun,
                format!("target 端分类为 {category:?}，不成模型单元（类型变更后不再出几何）"),
            ));
            continue;
        }
        match model_element_with_cache(&el, category, &mut mesh_cache) {
            Ok(Modeled::Solid(generated)) => {
                report.regenerated += 1;
                upserts.push(*generated);
            }
            Ok(Modeled::Skipped { reason, .. }) => {
                // E3D 同样不出几何 → 模型集里不该有它的行（先擦后画，画不出保持擦掉）。
                removals.insert(element_id(unit));
                report.regen_skipped.push(incident(refno, noun, reason));
            }
            Err(error) => {
                removals.insert(element_id(unit));
                report
                    .regen_failed
                    .push(incident(refno, noun, error.to_string()));
            }
        }
    }

    // 派生容器：target 端整链重算，旧键里没被接住的发 Remove。
    // `kept` 只收这一轮真建成的管身键——退化槽位不出几何，它的旧键就该被清掉。
    let mut kept: BTreeSet<GeometryId> = BTreeSet::new();
    for container in regenerate_derived {
        let refno = refno_of(container);
        let el = target.element(refno);
        match el.element_type() {
            // 容器在 target 端已删：退休，旧键由下面的差集统一清。
            Err(DbElementError::NotFound { .. }) => {
                report.derived_retired += 1;
                report.tube_segments.insert(refno.to_string(), 0);
            }
            Err(error) => {
                report
                    .derived_failed
                    .push(incident(refno, "?", format!("读不出 noun：{error}")));
            }
            Ok(noun) => match derives_implied_tubes(&el) {
                Ok(false) => {
                    // 类型变更把容器变成了别的东西，或改挂到了风管（HVAC）下：名下不该
                    // 再有管身，旧键全清。
                    report.derived_retired += 1;
                    report.tube_segments.insert(refno.to_string(), 0);
                    report.notes.push(incident(
                        refno,
                        noun,
                        "计划时是派生管身的路由容器，target 端类型或属主学科已变，按退休清掉旧管身",
                    ));
                }
                Ok(true) => match route_tubes_of(&el) {
                    Ok(tubes) => {
                        report.derived_regenerated += 1;
                        // 逐容器管段账（gen-model ADR-080 D6 的门拿它对 upserts）：与下面的
                        // `upserts.push` 分开记，装配路上掉了一段这里才对不上。
                        report.tube_segments.insert(refno.to_string(), tubes.len());
                        for tube in tubes {
                            kept.insert(tube.geometry_id.clone());
                            report.tube_upserts += 1;
                            upserts.push(tube);
                        }
                    }
                    Err(error) => {
                        report
                            .derived_failed
                            .push(incident(refno, noun, error.to_string()));
                    }
                },
                Err(error) => {
                    report
                        .derived_failed
                        .push(incident(refno, noun, error.to_string()));
                }
            },
        }
    }
    for stale in derived_stale {
        if !kept.contains(&stale) && removals.insert(stale) {
            report.tube_removals += 1;
        }
    }

    upserts.sort_by(|left, right| left.geometry_id.cmp(&right.geometry_id));
    IncrementOutcome {
        upserts,
        removals: removals.into_iter().collect(),
        report,
    }
}

/// 分类 + 执行一步到位。
pub fn increment_update(
    base: &Arc<DbSet>,
    target: &Arc<DbSet>,
    window: &WindowDiff,
) -> IncrementOutcome {
    let plan = plan_update(base, target, window);
    execute_plan(target, plan)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `implicit_frame_host_of` 在 aps250110 真库上的形状（库或模板不在就跳过）：
    /// SPINE 点与 PLDATU 都上溯到名下有 JLDATU 的宿主；FIXING 以下不算；没有 JLDATU 的
    /// GENSEC（FIXING 模板里的那种）连同它的 SPINE 都不算；STWALL 名下的 FITT 上溯到墙。
    #[test]
    fn implicit_frame_inputs_climb_to_their_host() {
        let db = Path::new(r"D:\AVEVA\Projects\E3D3.1\AvevaPlantSample\aps000\aps250110_0001");
        let template = Path::new(r"E:\reverse\e3d\shadow_e3d31_aps_all");
        if !db.exists() || !template.join("attlib.dat").exists() {
            return;
        }
        let set = Arc::new(DbSet::with_attlib_file(&template.join("attlib.dat")).unwrap());
        set.add_db(e3d_io::db_element::DbFilePin {
            file: db.to_path_buf(),
            template: e3d_io::db_element::template_file_for(template, "DESI").unwrap(),
            db_type: Some("DESI".into()),
            sesno: None,
        })
        .unwrap();
        let host_of = |w1| implicit_frame_host_of(&set, RefNo::new(2013286654, w1)).unwrap();
        let host = Some(RefNo::new(2013286654, 2223));
        assert_eq!(host_of(2223), host, "GENSEC 3 of /AREA02_WS_AXIS_WS.1 自己");
        assert_eq!(host_of(24523), host, "POINSP → SPINE → GENSEC");
        assert_eq!(host_of(24525), host, "PLDATU → JLDATU → GENSEC");
        assert_eq!(
            host_of(3236),
            None,
            "datum 下的 FIXING 走 owner 链级联，不在这条腿上"
        );
        assert_eq!(
            host_of(31617),
            None,
            "FIXING 模板里的 GENSEC 名下没有 JLDATU"
        );
        assert_eq!(host_of(31618), None, "它的 SPINE 同样不算");
        let wall = Some(RefNo::new(2013286654, 39));
        assert_eq!(host_of(39), wall, "名下有 FITT 的 STWALL 自己");
        assert_eq!(host_of(40), wall, "FITT → STWALL");
    }

    /// `implicit_frame_host_of` 的 WALL 支在 ams1112 真库上的形状（库或模板不在就跳过；gen-model issue #37）：
    /// 墙上带 `DESP` 的 FIXING `17496/124135` 挂在 PLDATU → JLDATU → WALL 之下，它的实例标架与开孔刀都由
    /// `elmodl::fixing_host_frame` 沿墙现算。墙自己、JLDATU、PLDATU、墙的 SPINE 与 SPINE 上的点都上溯到墙；
    /// FIXING 自己不在这条腿上（它改了，owner 链上卷到它自己就够）。
    #[test]
    fn wall_datum_inputs_climb_to_their_wall() {
        let db = Path::new(r"D:\AVEVA\Projects\E3D3.1\AvevaMarineSample\ams000\ams1112_0001");
        let template = Path::new(r"E:\reverse\e3d\shadow_e3d31_aps_all");
        if !db.exists() || !template.join("attlib.dat").exists() {
            return;
        }
        let set = Arc::new(DbSet::with_attlib_file(&template.join("attlib.dat")).unwrap());
        set.add_db(e3d_io::db_element::DbFilePin {
            file: db.to_path_buf(),
            template: e3d_io::db_element::template_file_for(template, "DESI").unwrap(),
            db_type: Some("DESI".into()),
            sesno: None,
        })
        .unwrap();
        let fixing = set.element(RefNo::new(17496, 124135));
        assert_eq!(fixing.element_type().unwrap(), "FIXING");
        let pldatu = fixing.owner().unwrap().expect("FIXING 的属主");
        let jldatu = pldatu.owner().unwrap().expect("PLDATU 的属主");
        let wall = jldatu.owner().unwrap().expect("JLDATU 的属主");
        assert_eq!(pldatu.element_type().unwrap(), "PLDATU");
        assert_eq!(jldatu.element_type().unwrap(), "JLDATU");
        assert_eq!(wall.element_type().unwrap(), "WALL");
        let host = Some(wall.refno());
        let host_of = |refno| implicit_frame_host_of(&set, refno).unwrap();
        assert_eq!(host_of(wall.refno()), host, "名下有 JLDATU 的 WALL 自己");
        assert_eq!(host_of(jldatu.refno()), host, "JLDATU → WALL");
        assert_eq!(host_of(pldatu.refno()), host, "PLDATU → JLDATU → WALL");
        assert_eq!(
            host_of(fixing.refno()),
            None,
            "datum 下的 FIXING 走 owner 链级联，不在这条腿上"
        );
        let mut spine = None;
        for member in wall.members().unwrap() {
            if member.element_type().unwrap() == "SPINE" {
                spine = Some(member);
                break;
            }
        }
        let spine = spine.expect("WALL 名下的 SPINE");
        assert_eq!(host_of(spine.refno()), host, "SPINE → WALL");
        let mut points = 0usize;
        for point in spine.members().unwrap() {
            assert_eq!(
                host_of(point.refno()),
                host,
                "{} → SPINE → WALL",
                point.element_type().unwrap()
            );
            points += 1;
        }
        assert!(points > 0, "墙 spine 上至少有一个点");
    }

    /// `implicit_frame_host_of` 的板支在 ams1112 真库上的形状（库或模板不在就跳过）：FLOOR 上的 PFIT
    /// `17496/123446`（`/1RS03TT0073T`）贴在板 SJUS 指的那一面（`catalogue::panel_surface_z`）。板自己与它的
    /// PLOO 上溯到板；PLOO 的顶点与 PFIT 自己不在这条腿上。
    #[test]
    fn panel_loops_climb_to_the_panel_carrying_pfits() {
        let db = Path::new(r"D:\AVEVA\Projects\E3D3.1\AvevaMarineSample\ams000\ams1112_0001");
        let template = Path::new(r"E:\reverse\e3d\shadow_e3d31_aps_all");
        if !db.exists() || !template.join("attlib.dat").exists() {
            return;
        }
        let set = Arc::new(DbSet::with_attlib_file(&template.join("attlib.dat")).unwrap());
        set.add_db(e3d_io::db_element::DbFilePin {
            file: db.to_path_buf(),
            template: e3d_io::db_element::template_file_for(template, "DESI").unwrap(),
            db_type: Some("DESI".into()),
            sesno: None,
        })
        .unwrap();
        let pfit = set.element(RefNo::new(17496, 123446));
        assert_eq!(pfit.element_type().unwrap(), "PFIT");
        let floor = pfit.owner().unwrap().expect("PFIT 的属主");
        assert_eq!(floor.element_type().unwrap(), "FLOOR");
        let host = Some(floor.refno());
        let host_of = |refno| implicit_frame_host_of(&set, refno).unwrap();
        assert_eq!(host_of(floor.refno()), host, "名下有 PFIT 的 FLOOR 自己");
        assert_eq!(
            host_of(pfit.refno()),
            None,
            "PFIT 自己改了，owner 链上卷到它就够"
        );
        let mut loops = 0usize;
        for member in floor.members().unwrap() {
            if member.element_type().unwrap() != "PLOO" {
                continue;
            }
            assert_eq!(host_of(member.refno()), host, "PLOO → FLOOR");
            for vertex in member.members().unwrap() {
                assert_eq!(
                    host_of(vertex.refno()),
                    None,
                    "{} 不在 PFIT 的输入里",
                    vertex.element_type().unwrap()
                );
            }
            loops += 1;
        }
        assert!(loops > 0, "FLOOR 名下至少一个 PLOO");
    }

    /// `implicit_frame_host_of` 的 HANDRA 支在 aps250110 真库上的形状（库或模板不在就跳过）：
    /// 详图式扶手 `/BUILDING_02_LEVEL_04/HR_001`（POSTSE + RAILSE 里带 HRTERM）的 RPATH 点、立柱、
    /// 端头、各集合容器都上溯到它；`/BUILDING_01_LEVEL_03/HR_001` 的 HRGATE 经 HRFEAT、CURVE 经 RPATH
    /// 也到；立柱 TMPL 里的件不算（owner 链级联够得着）；只有 RPATH 的布局式 HANDRA 不算。
    #[test]
    fn handrail_post_inputs_climb_to_their_handrail() {
        let db = Path::new(r"D:\AVEVA\Projects\E3D3.1\AvevaPlantSample\aps000\aps250110_0001");
        let template = Path::new(r"E:\reverse\e3d\shadow_e3d31_aps_all");
        if !db.exists() || !template.join("attlib.dat").exists() {
            return;
        }
        let set = Arc::new(DbSet::with_attlib_file(&template.join("attlib.dat")).unwrap());
        set.add_db(e3d_io::db_element::DbFilePin {
            file: db.to_path_buf(),
            template: e3d_io::db_element::template_file_for(template, "DESI").unwrap(),
            db_type: Some("DESI".into()),
            sesno: None,
        })
        .unwrap();
        let host_of = |w1| implicit_frame_host_of(&set, RefNo::new(2013286654, w1)).unwrap();
        let handrail = Some(RefNo::new(2013286654, 3397));
        assert_eq!(
            host_of(3397),
            handrail,
            "HANDRA /BUILDING_02_LEVEL_04/HR_001 自己"
        );
        assert_eq!(host_of(3471), handrail, "RPATH → HANDRA");
        assert_eq!(host_of(3473), handrail, "POINTR → RPATH → HANDRA");
        assert_eq!(host_of(3398), handrail, "POSTSE → HANDRA");
        assert_eq!(host_of(3409), handrail, "HRPOST → POSTSE → HANDRA");
        assert_eq!(host_of(3466), handrail, "RAILSE → HANDRA");
        assert_eq!(host_of(13569), handrail, "HRTERM → RAILSE → HANDRA");
        assert_eq!(
            host_of(12337),
            None,
            "立柱 TMPL 里的 GENSEC 走 owner 链级联，不在这条腿上"
        );
        let gated = Some(RefNo::new(2013286654, 3335));
        assert_eq!(host_of(4715), gated, "HRGATE → HRFEAT → HANDRA");
        assert_eq!(host_of(3391), gated, "RPATH 里的 CURVE → HANDRA");
        assert_eq!(
            host_of(8599),
            None,
            "只有 RPATH 的布局式 HANDRA 自己就是单元，不级联"
        );
        assert_eq!(host_of(8601), None, "它的 POINTR 同样不算");
    }

    /// `joint_consumer_of` 在 aps250110 / aps250108 真库上的形状（库或模板不在就跳过）：梁
    /// GENSEC 2 of /B01_LEVEL_01_BEAMS（`/261`）两端的 FIXING 经 CREF 回指到它；梁自己、端板 TMPL、
    /// CREF 空的 FIXING、回指 STWALL 的 FIXING 都不算；aps250108 里那只关节 FIXING 的 CREF 指回
    /// aps250110 的梁，是跨库。
    #[test]
    fn joint_fixings_point_back_to_their_beam() {
        let plant = Path::new(r"D:\AVEVA\Projects\E3D3.1\AvevaPlantSample\aps000");
        let template = Path::new(r"E:\reverse\e3d\shadow_e3d31_aps_all");
        let dbs = [plant.join("aps250110_0001"), plant.join("aps250108_0001")];
        if dbs.iter().any(|db| !db.exists()) || !template.join("attlib.dat").exists() {
            return;
        }
        let set = Arc::new(DbSet::with_attlib_file(&template.join("attlib.dat")).unwrap());
        for db in dbs {
            set.add_db(e3d_io::db_element::DbFilePin {
                file: db,
                template: e3d_io::db_element::template_file_for(template, "DESI").unwrap(),
                db_type: Some("DESI".into()),
                sesno: None,
            })
            .unwrap();
        }
        let consumer = |w0, w1| joint_consumer_of(&set, RefNo::new(w0, w1)).unwrap();
        let beam = Some(JointConsumer::Beam(RefNo::new(2013286654, 261)));
        assert_eq!(consumer(2013286654, 349), beam, "JOIS 那只 FIXING");
        assert_eq!(consumer(2013286654, 376), beam, "JOIE 那只 FIXING");
        assert_eq!(consumer(2013286654, 261), None, "梁自己不是 FIXING");
        assert_eq!(consumer(2013286654, 26003), None, "端板 TMPL 不是 FIXING");
        assert_eq!(consumer(2013286654, 208), None, "CREF 空的 FIXING");
        assert_eq!(
            consumer(2013286654, 872),
            None,
            "CREF 回指 STWALL，墙不吃关节负体"
        );
        assert_eq!(
            consumer(2013286652, 390),
            Some(JointConsumer::OtherDatabase(RefNo::new(2013286654, 1819))),
            "aps250108 的关节 FIXING，梁在 aps250110"
        );
    }

    /// `joint_fixings_of` 在 aps250110 真库上的形状（库或模板不在就跳过）：梁 `/261` 两端的端板 FIXING、
    /// 檐口斜梁 `/2209` 终点的加腋 FIXING；柱（关节空）与 FIXING 自己都是空。
    #[test]
    fn beams_name_their_joint_fixings() {
        let db = Path::new(r"D:\AVEVA\Projects\E3D3.1\AvevaPlantSample\aps000\aps250110_0001");
        let template = Path::new(r"E:\reverse\e3d\shadow_e3d31_aps_all");
        if !db.exists() || !template.join("attlib.dat").exists() {
            return;
        }
        let set = Arc::new(DbSet::with_attlib_file(&template.join("attlib.dat")).unwrap());
        set.add_db(e3d_io::db_element::DbFilePin {
            file: db.to_path_buf(),
            template: e3d_io::db_element::template_file_for(template, "DESI").unwrap(),
            db_type: Some("DESI".into()),
            sesno: None,
        })
        .unwrap();
        let fixings_of = |w1| joint_fixings_of(&set, RefNo::new(2013286654, w1)).unwrap();
        let refnos = |w1s: &[u32]| {
            w1s.iter()
                .map(|w1| RefNo::new(2013286654, *w1))
                .collect::<Vec<_>>()
        };
        assert_eq!(fixings_of(261), refnos(&[349, 376]), "端板梁两端");
        assert_eq!(fixings_of(2209), refnos(&[2232]), "檐口斜梁终点");
        assert!(fixings_of(2223).is_empty(), "柱的关节空");
        assert!(fixings_of(2232).is_empty(), "FIXING 不是梁");
    }

    /// `datum_fixing_host_of` 在 aps250110 真库上的形状（库或模板不在就跳过）：柱 GENSEC 1 of
    /// /B01_LEVEL_01_COLUMNS（`/277`）datum 树里的底板 FIXING 与端板 FIXING 都上溯到它；柱自己、
    /// 端板 TMPL、挂在 WALL 上的 FIXING 都不算。
    #[test]
    fn datum_fixings_climb_to_their_host_gensec() {
        let db = Path::new(r"D:\AVEVA\Projects\E3D3.1\AvevaPlantSample\aps000\aps250110_0001");
        let template = Path::new(r"E:\reverse\e3d\shadow_e3d31_aps_all");
        if !db.exists() || !template.join("attlib.dat").exists() {
            return;
        }
        let set = Arc::new(DbSet::with_attlib_file(&template.join("attlib.dat")).unwrap());
        set.add_db(e3d_io::db_element::DbFilePin {
            file: db.to_path_buf(),
            template: e3d_io::db_element::template_file_for(template, "DESI").unwrap(),
            db_type: Some("DESI".into()),
            sesno: None,
        })
        .unwrap();
        let host_of = |w1| datum_fixing_host_of(&set, RefNo::new(2013286654, w1)).unwrap();
        let column = Some(RefNo::new(2013286654, 277));
        assert_eq!(host_of(25397), column, "底板 FIXING（SFIT）");
        assert_eq!(host_of(283), column, "端板 FIXING（TMPL）");
        assert_eq!(host_of(277), None, "柱自己不是 FIXING");
        assert_eq!(host_of(26003), None, "端板 TMPL 不是 FIXING");
        assert_eq!(host_of(208), None, "WALL 上的 FIXING");
    }

    fn incident_n(n: usize) -> Vec<Incident> {
        (0..n)
            .map(|i| Incident {
                refno: format!("1/{i}"),
                noun: "PANE".into(),
                detail: "test".into(),
            })
            .collect()
    }

    /// 候选账：三本账之和必须等于候选数，多一少一都要响。
    #[test]
    fn candidate_accounting_must_close() {
        let mut report = IncrementReport {
            candidates: 5,
            rolled_up: 3,
            unresolved: incident_n(1),
            ledger_refnos: 5,
            ..IncrementReport::default()
        };
        report.no_model.insert("ZONE".into(), 1);
        assert!(report.accounts_for().is_ok());

        report.rolled_up = 2;
        let err = report.accounts_for().unwrap_err();
        assert!(err.contains("候选账不平"), "{err}");
    }

    /// L2 判「内容没变」的候选自成一本账，也必须进恒等式。
    ///
    /// 它不能并进 `no_model`：那本账的语义是「改动落在没有模型归属的元素上」，
    /// 而这些候选压根就没有改动。混在一起，「索引虚报了多少」就再也读不出来。
    #[test]
    fn an_unchanged_candidate_has_its_own_account() {
        let mut report = IncrementReport {
            candidates: 4,
            rolled_up: 1,
            unchanged: 3,
            ledger_refnos: 1,
            ..IncrementReport::default()
        };
        assert!(report.accounts_for().is_ok());

        report.unchanged = 2;
        let err = report.accounts_for().unwrap_err();
        assert!(err.contains("unchanged 2"), "{err}");
    }

    /// L3 对 L4 的守恒：除 unchanged 之外每个候选都在账本上——账本 refno 数必须等于
    /// rolled_up + no_model + unresolved。少了就是有候选（多半是读不出的那些）没进账。
    #[test]
    fn the_ledger_must_name_every_classified_candidate() {
        let mut report = IncrementReport {
            candidates: 6,
            rolled_up: 2,
            unchanged: 2,
            unresolved: incident_n(1),
            ledger_refnos: 4,
            ..IncrementReport::default()
        };
        report.no_model.insert("ZONE".into(), 1);
        assert!(report.accounts_for().is_ok());

        // 读不出的那个候选没进账本：候选账照样平，账本账要响。
        report.ledger_refnos = 3;
        let err = report.accounts_for().unwrap_err();
        assert!(err.contains("账本不平"), "{err}");

        // cascade_only 是 rolled_up 的子集。
        report.ledger_refnos = 4;
        report.cascade_only = 3;
        let err = report.accounts_for().unwrap_err();
        assert!(err.contains("cascade_only"), "{err}");
    }

    /// 退化成整库重建的报告：什么都没分类，候选账不适用，但计划集与账本必须是空的
    /// ——一份带动作的「退化」计划是自相矛盾的，得响。
    #[test]
    fn a_degraded_plan_has_no_classification_and_no_actions() {
        let report = IncrementReport {
            candidates: 24674,
            degraded_to_full: Some("candidates 24674 > max_candidates 1000".into()),
            ..IncrementReport::default()
        };
        assert!(report.accounts_for().is_ok(), "{:?}", report.accounts_for());

        let report = IncrementReport {
            candidates: 24674,
            degraded_to_full: Some("candidates 24674 > max_candidates 1000".into()),
            units_to_regenerate: 1,
            ..IncrementReport::default()
        };
        let err = report.accounts_for().unwrap_err();
        assert!(err.contains("退化成整库重建"), "{err}");

        // 没有退化标记，候选却一个没分类：这是普通的候选账不平。
        let report = IncrementReport {
            candidates: 24674,
            ..IncrementReport::default()
        };
        assert!(report.accounts_for().unwrap_err().contains("候选账不平"));
    }

    /// `PlanLimits::default()` 不设限——`plan_update` 的行为一个字不变。
    #[test]
    fn default_limits_are_unbounded() {
        assert_eq!(PlanLimits::default().max_candidates, None);
    }

    /// 元数据门省下的三档（`metadata_only` / `unread_attribute` / `cache_stamp`）都是 `no_model` 的横切计数：
    /// 三数之和不能比 `no_model` 大，且三个都要在 `totals_line` 上读得到（2026-09-15 S6；2026-09-17 Q6 加第三档）。
    #[test]
    fn the_gate_tiers_are_a_subset_of_no_model_and_all_show_on_the_totals_line() {
        let mut report = IncrementReport {
            candidates: 6,
            rolled_up: 2,
            metadata_only: 1,
            unread_attribute: 2,
            cache_stamp: 1,
            ledger_refnos: 6,
            ..IncrementReport::default()
        };
        report.no_model.insert("ELBO".into(), 4);
        assert!(report.accounts_for().is_ok());
        let line = report.totals_line();
        assert!(
            line.contains("no_model=4 (metadata_only=1 unread_attribute=2 cache_stamp=1)"),
            "{line}"
        );

        report.unread_attribute = 3;
        let err = report.accounts_for().unwrap_err();
        assert!(err.contains("unread_attribute 3"), "{err}");
        report.unread_attribute = 2;
        report.cache_stamp = 2;
        let err = report.accounts_for().unwrap_err();
        assert!(err.contains("cache_stamp 2"), "{err}");
    }

    /// 派生执行账：regenerated + retired + failed 必须等于计划容器数。
    ///
    /// 单独一条恒等式而不是并进单元执行账：两边的「计划数」来源不同
    /// （模型单元 vs 派生容器），混在一起，哪边不平就再也读不出来。
    #[test]
    fn derived_execution_accounting_must_close() {
        let report = IncrementReport {
            derived_to_regenerate: 3,
            derived_regenerated: 1,
            derived_retired: 1,
            derived_failed: incident_n(1),
            tube_upserts: 2,
            tube_segments: BTreeMap::from([("1/10".to_string(), 2), ("1/11".to_string(), 0)]),
            ..IncrementReport::default()
        };
        assert!(report.accounts_for().is_ok());

        let report = IncrementReport {
            derived_to_regenerate: 3,
            derived_regenerated: 1,
            derived_retired: 1,
            tube_upserts: 2,
            tube_segments: BTreeMap::from([("1/10".to_string(), 2), ("1/11".to_string(), 0)]),
            ..IncrementReport::default()
        };
        let err = report.accounts_for().unwrap_err();
        assert!(err.contains("派生执行账不平"), "{err}");
    }

    /// 逐容器管段账是 `tube_upserts` 的横切拆分：建成 / 退休的容器每条一笔，各值之和等于
    /// 真 upsert 的管身根数。少一条容器、或段数对不上，下游拿它当门就核不出漏段。
    #[test]
    fn per_container_tube_segments_must_split_the_tube_upserts() {
        let short = IncrementReport {
            derived_to_regenerate: 2,
            derived_regenerated: 2,
            tube_upserts: 3,
            tube_segments: BTreeMap::from([("1/10".to_string(), 3)]),
            ..IncrementReport::default()
        };
        let err = short.accounts_for().unwrap_err();
        assert!(err.contains("管段账不平"), "{err}");

        let miscounted = IncrementReport {
            derived_to_regenerate: 2,
            derived_regenerated: 2,
            tube_upserts: 3,
            tube_segments: BTreeMap::from([("1/10".to_string(), 1), ("1/11".to_string(), 1)]),
            ..IncrementReport::default()
        };
        let err = miscounted.accounts_for().unwrap_err();
        assert!(err.contains("管段账不平"), "{err}");

        let closed = IncrementReport {
            derived_to_regenerate: 2,
            derived_regenerated: 2,
            tube_upserts: 3,
            tube_segments: BTreeMap::from([("1/10".to_string(), 1), ("1/11".to_string(), 2)]),
            ..IncrementReport::default()
        };
        assert!(closed.accounts_for().is_ok());
    }

    /// 执行账：regenerated + skipped + failed 必须等于计划单元数。
    #[test]
    fn execution_accounting_must_close() {
        let report = IncrementReport {
            units_to_regenerate: 4,
            regenerated: 2,
            regen_skipped: incident_n(1),
            regen_failed: incident_n(1),
            ..IncrementReport::default()
        };
        assert!(report.accounts_for().is_ok());

        let report = IncrementReport {
            units_to_regenerate: 4,
            regenerated: 2,
            ..IncrementReport::default()
        };
        let err = report.accounts_for().unwrap_err();
        assert!(err.contains("执行账不平"), "{err}");
    }

    /// refno 键往返无损（word1 高位置位的 refno 也一样）。
    #[test]
    fn refno_key_round_trips() {
        for (w0, w1) in [(17496u32, 1u32), (1u32, 0x7FFF_FFFFu32), (0x2001, 42)] {
            let refno = RefNo::new(w0, w1);
            assert_eq!(refno_of(key(refno)), refno);
        }
    }
}

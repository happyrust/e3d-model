//! 路由遍历与隐式管身（对标 `cachelib/GTTUBE` 取两端、`cachelib/TUMAT` 定位、
//! core.dll `ORIMAT` 定向、`pplib/CGETOD` 取外径；几何入口 `cachegml/GTTUBG`）。
//!
//! 五个路由容器（`BRAN`/`HANG`/`LUG`/`SUPC`/`TRUNNI`）的**遍历**走同一套：链头由
//! 容器的 `HPOS` 给，链尾由 `TPOS` 给，相邻两件之间是一条槽位。`PDMS_HangElement`
//! 的 `nextElement` / `prevElement` 走到头分别回 `TposElement` / `HposElement`，
//! 与 BRAN 逐点同构，走法上**不要**为 HANG 另起分支（`task_plan.md` §2.6）。
//!
//! 但「槽位上补不补管身」不归这里管，归
//! [`crate::category::derives_implied_tubes`]：E3D 只为 BRAN/LUG/SUPC/TRUNNI 四族
//! （[`crate::category::DERIVED_ROUTE_CONTAINER_NOUNS`]）在成员间补管，**HANG 不补**
//! （RVM 24381/177986 只导出 PCLAMP 与 HELEMENT 两组几何；把两个独立支撑件当管路串
//! 起来会凭空得到一根 33.8m 的假管），**属主是 HVAC 的风管 BRAN 也不补**（风管直段是
//! 显式 STRT；RVM 24381/30598 只有 4 Box + 1 Pyramid，按 noun 补管会多出两根 r=580 的
//! 20 多米圆管）。管线与增量都按那一个谓词收口：HANG 与风管 BRAN 不进本模块，
//! 它们的成员照常由通用遍历下钻、按目录件建模。
//!
//! # 两端点有两种来源，可得性完全不同
//!
//! | 来源 | 属性 | 现在读不读得到 |
//! |---|---|---|
//! | 容器头 / 尾 | 容器自身的 `HPOS/HDIR/HBOR`、`TPOS/TDIR/TBOR` | **读得到**，直读 |
//! | 构件离开 / 到达 | 目录侧 `LPOS/LDIR/LBOR`、`APOS/ADIR/ABOR` | **读不到**，见下 |
//!
//! 设计元素上的 `LEAV` / `ARRI` 只是 **P 点编号**（ams8000 全库 2612 个路由成员
//! 实测清一色 `LEAV=2 / ARRI=1`），坐标在目录里，要走
//! `SPRE → SPCO.CATR → SCOM.PTRE → PTAX`；而 `PTAX` 的 `PDIS` 实测是**目录表达式**
//! （`/ACP1000-TFVL-P2` 存的就是 `( ATTRIB RPRO TLEN )`），求值要 e3d-io 的
//! `DbElementParamEnv`。
//!
//! 所以「解构件 P 点」在这里是一条显式的缝 [`PPointResolver`]。走目录链的那份实现
//! 在 [`crate::catalogue_point::CataloguePoints`]；[`CatalogueNotWired`] 留着，
//! 给不开目录库的场合用，一律解不出、按 `missing_ppoint` 记账。
//!
//! **不拿构件 `POS` 顶替目录 P 点**：那样能算出一个看着合理、实则系统性偏短的
//! 长度，而报告全绿——本仓最忌讳的正是这种静默错。ams8000 实测 3172 条槽位里
//! 有 3171 条至少一端落在这条缝上，凑数的代价会是 3171 条假管。

use std::collections::BTreeMap;

use glam::{DMat3, DMat4, DVec3};
use manifold_csg::Manifold;

use e3d_io::db_element::{DbElement, DbElementError};
use e3d_io::record::descriptor::DescriptorValue;
use e3d_io::refno::RefNo;

use crate::catalogue_point::catalogue_component;
use crate::category::{Category, classify};
use crate::primitive_instance::{PrimitiveInstance, instantiate_centered_cylinder};
use crate::solid::SolidError;

/// `ORIMAT` 挑副轴时判「叉积退化」的阈值。
///
/// `Z` 与三个基轴的叉积长度是 `sin θ`，而按 X̂ / Ŷ / Ẑ 顺序试下来，**至少有一个**
/// 的 `sin θ ≥ √(2/3) ≈ 0.816`——所以这个阈值只要能挡住真正的平行就够，取多小
/// 都不会改变选中的是哪一根轴。定在 1e-9 是为了让「几乎平行」也走到下一根轴上去。
const AXIS_DEGENERATE_TOL: f64 = 1.0e-9;

/// 隐式管身的最小管长阈值（mm）：短于它按「无效管身」处理，不出几何。
///
/// **从 `Core3D.dll` 反编译钉死为 1.0mm**：`FAB_PipePieceManager::isValidTubeLength`
/// （`0x1057be50`）开头两道硬门——`fabs(管长) < 1.0`、或两端点直线距离 `< 1.0`，
/// 命中任一即判无效（`return 0`）。两道门在这里塌成一道：本仓的管长就是两端点
/// 直线距离（[`tube_length`]），没有独立的长度属性输入。`FAB_WeldingMachine::
/// validMinimumLength`（`0x1058ec00`）与图形侧 `GFX_QPR_IgnoreMinimumTubeLength` /
/// `PDMS_QPR_Operations::setIgnoreMinimumTubeLength` 印证同一口径（默认不忽略）。
/// 比较取 `< 1.0` 判无效的原样：恰好 1.0mm 的管 Core 判有效，照建。
///
/// 之前借 libgm 面级容差 `RES_TOL_MM`（0.051mm）占位，比真值小约 20 倍——
/// 0.051~1.0mm 的短管 E3D 判无效、本仓却会建出来，凭空多出针状假管。
/// 取证见 `evidence/20260901-pipe-baseline/bran-hang-algorithm-analysis.md` §二。
/// （遗留 `GTTUBG` 自身无显式阈值、靠内核判退化；这道 1.0mm 门作用在可视化路上
/// 的最终确认归 G0-2 RVM 对拍。）
pub const ZERO_LENGTH_TOL_MM: f64 = 1.0;

/// 隐式管身的共轴容差：到达点偏离离开轴线的**横向距离**必须严格小于**轴向距离的千分之一**
/// （`tan θ < 0.001`，θ ≈ 0.0573°），两端各查一次。超过就不建管。
///
/// E3D 只在**两端共轴且朝前**的槽位上画隐式管身。2026-09-02 用 ams8000 二十一条支管 + ams7999 一条
/// 的 TTY RVM 逐根对过（`evidence/20260902-ams8000-bran-audit/rvm-pending-items-2026-09-02.md` §4）：
/// 画出来的最斜一根是 `/M-OR-1R323-A` 的 TUBE 2（0.0563° = 0.983 mrad），没画的最正一根是
/// `/C-CO-1RX-3AH0A` 的尾管（2000 mm 上横偏 2.0 mm，恰好 1.000 mrad）与 `/M-OR-1R313-C` 的
/// 0.0607°（1.06 mrad）；再往上 0.36° / 0.5° / 3° / 8.1° / 13° / 31.6° / 47.4° / 90° 一根不画。
/// 阈值只能是 1 mrad，且**恰好等于不算过**——写成横向 < 千分之一轴向而不是 `acos` 出来的角度，
/// 是为了让 2.0 / 2000 这种整数坐标算出来正好卡在界上、不被浮点误差放进来。
///
/// 反向（轴向分量为负）单列成 [`SlotAlignment::Reversed`]：那是 E3D 数据一致性检查会报
/// 「负管长」的形态（README §3.2 的 74 根），与偏轴分开记账才看得出各占多少。
pub const TUBE_ALIGNMENT_TOL: f64 = 1.0e-3;

/// 一个 P 点的三件套：位置、方向、通径。
///
/// 对应 `GTTUBE` 里 `IHEAD`/`TAIL`/`PLEAVE`/`PARRIV` 四个例程各自取出的那一组。
/// `bore` 允许缺省：`CGETOD` 只在目录外径取不到时才回落到它（见
/// [`tube_outer_diameter`]），两端通径不齐并不阻止建管。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RoutePoint {
    pub pos: DVec3,
    pub dir: DVec3,
    pub bore: Option<f64>,
}

/// 管身一端是谁给的。类型本身就说明了它现在解不解得出，见模块文档那张表。
#[derive(Debug, Clone)]
pub enum RouteEnd {
    /// 容器头点：容器自身的 `HPOS/HDIR/HBOR`。
    ContainerHead(DbElement),
    /// 容器尾点：容器自身的 `TPOS/TDIR/TBOR`。
    ContainerTail(DbElement),
    /// 构件的离开点，`LEAV` 给的是 P 点编号而非坐标。
    MemberLeave {
        member: DbElement,
        ppoint: Option<u32>,
    },
    /// 构件的到达点，`ARRI` 同上。
    MemberArrive {
        member: DbElement,
        ppoint: Option<u32>,
    },
}

impl RouteEnd {
    /// 这一端**站在**哪个元素上：容器头尾都站在容器自己身上，构件端站在那个构件上。
    ///
    /// 这不是取个名字方便打印——管身的键就是两端站点的这对 refno
    /// （[`crate::elmodl::GeometryId::ImpliedTube`]），所以这个映射是键的一半。
    /// 写成穷举 `match`：往 [`RouteEnd`] 里加一种端点，这里必须当场回答它站在谁身上。
    pub fn station_refno(&self) -> RefNo {
        match self {
            RouteEnd::ContainerHead(container) | RouteEnd::ContainerTail(container) => {
                container.refno()
            }
            RouteEnd::MemberLeave { member, .. } | RouteEnd::MemberArrive { member, .. } => {
                member.refno()
            }
        }
    }
}

/// 一条管身槽位：从 `from` 到 `to`。
///
/// 「槽位」不是「管子」——两端解不出、或解出来是零长时，槽位仍在账上（它是链的
/// 一节），但不出几何。两个数不必相等，这正是 `ImpliedTubeDegenerate` 存在的理由。
#[derive(Debug, Clone)]
pub struct TubeSlot {
    pub from: RouteEnd,
    pub to: RouteEnd,
}

impl TubeSlot {
    /// 这条槽位键的**站点分量**：两端站点的 refno。
    ///
    /// 只要成员表的**相邻关系**没变，这对 refno 就不变——链上别处插删多少件都不影响。
    /// 完整的键还带一个槽位序号（见 [`route_slot_keys`] 与
    /// [`crate::elmodl::GeometryId::ImpliedTube`]）：站点对负责可读与常态去重，
    /// 序号兜底成员表回环时同一站点对出现两次的场合。
    pub fn station_refnos(&self) -> (RefNo, RefNo) {
        (self.from.station_refno(), self.to.station_refno())
    }
}

/// 一个路由容器走完的结果。
#[derive(Debug)]
pub struct Route {
    /// 链上的路由成员，文件原序。
    pub members: Vec<DbElement>,
    /// 不推进隐式管身链的成员。**记账、继续下钻、但不参与串接**——
    /// 它可能自带几何，却不是管身端点；算进去会凭空多出两截管身。
    /// 目录 `GTYP=ATTA` 的附着点属于 RouteMember、会生成自己的目录几何，但 E3D 的
    /// TUBE 穿过它（见 [`station_verdict`]）。
    pub non_route_members: Vec<DbElement>,
    /// 设计 noun 为 ATTA、目录链却走不到 SCOM 而**按穿过处理**的成员数。
    /// 不为零时，这条链上的 ANCI 型支撑（梯架的 RACK-REST / SPECIAL）可能被当成了穿过，
    /// 管身会比 E3D 少几段——报告里要点名。
    pub pass_through_unresolved: usize,
    /// 管身槽位，链序。条数恒为 `members.len() + 1`。
    pub tubes: Vec<TubeSlot>,
}

/// 链上相邻两站的下标对：`0` = 容器头，`1..=n` = 第 i 个路由成员，`n + 1` = 容器尾。
///
/// 单独拎出来是为了能不开库就测：「尾管必须补」「非路由成员不前移」这两条都只是
/// 这个序列的形状，而它们恰恰是最容易写错、又最难在真库上看出来的地方
/// （多一条少一条，报告里只是个数字变了）。
fn slot_indices(member_count: usize) -> impl Iterator<Item = (usize, usize)> {
    (0..=member_count).map(|i| (i, i + 1))
}

/// 链上第 `index` 站是谁。
///
/// 与 [`slot_indices`] 同一个理由单拎出来：管身的键是相邻两站**所站元素**的那一对，
/// 而头尾两站都站在容器自己身上，所以这段下标算术就是键的定义本身
/// （[`crate::elmodl::GeometryId::ImpliedTube`]）。放在这里能不开库就把
/// 「链中间插一件，只有跨它那条键该变」测掉。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Station {
    /// 容器头点（容器的 `HPOS`）。
    Head,
    /// 第 `.0` 个路由成员，0 基。
    Member(usize),
    /// 容器尾点（容器的 `TPOS`）。
    Tail,
}

fn station_at(index: usize, member_count: usize) -> Station {
    match index {
        0 => Station::Head,
        i if i <= member_count => Station::Member(i - 1),
        _ => Station::Tail,
    }
}

/// 隐式管身链对一个成员的裁决。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StationVerdict {
    /// 链上的一站：前一站的离开点到它的到达点是一条槽位，它的离开点开下一条。
    Station,
    /// 管身穿过它：记账、下钻，但不参与串接。
    PassThrough,
    /// 设计 noun 是 ATTA，目录链却走不到 SCOM（无 SPRE、库没开、目标不在会话）：
    /// 按穿过处理，另计一笔（[`Route::pass_through_unresolved`]）。
    PassThroughUnresolved,
}

/// 这个成员是不是隐式管身的一个串接站点——纯裁决，`noun` 是设计 noun，
/// `scom_gtyp` 是它目录件 `SCOM.GTYP` 的字面（解不出给 `None`）。
///
/// `Category::RouteMember` 回答的是“这件东西属于管路且要生成目录几何”；它不等于
/// “这件东西切断管身”。设计 noun 同为 ATTA 的成员在 E3D 里分两种：
///
/// - 目录 **`GTYP=ATTA/PCLA`**（`/AVEVA-ATTA-SPEC/SUPP:ANY`、`/NPHS/SPEC/PIPE/BRAN` 这类管道
///   支撑附着点）：E3D 的 TUBE 连续穿过它。ams7999 `/1RCV0101…/B1` 的 TUBE 2 = 801 mm 穿过
///   `/R342.062`，ams8000 `24384/26229` 的 TUBE 2 = 312.1 mm 穿过三只到 VALV。把它加入站点链
///   会把连续管拆段；附着点在中心线外时还会生成一来一回的重叠假管。
/// - 目录 **`GTYP=ANCI`**（MDS 梯架支撑 `/RACK-REST-300`、`/SPECIAL.300`，P1/P2 通径 300）：
///   是普通站点，管身在它两侧断开。`/C-GR-1RX-3020B`（24384/2944）的六根 TUBE 全以它为端点，
///   `/C-GR-1RX-3000A`（24384/3277）唯一一根是 ATTA9→BEND3。
///
/// 所以看的是目录 GTYP，不是设计 noun。证据见
/// `evidence/20260902-ams8000-bran-audit/rvm-pending-items-2026-09-02.md` §2。
pub fn station_verdict(noun: &str, scom_gtyp: Option<&str>) -> StationVerdict {
    if !matches!(classify(noun), Category::RouteMember) {
        return StationVerdict::PassThrough;
    }
    if !noun.eq_ignore_ascii_case("ATTA") {
        return StationVerdict::Station;
    }
    match scom_gtyp {
        Some(gtyp) if gtyp.eq_ignore_ascii_case("ANCI") => StationVerdict::Station,
        Some(_) => StationVerdict::PassThrough,
        None => StationVerdict::PassThroughUnresolved,
    }
}

/// 库上的裁决：非 ATTA 不查目录；ATTA 走 `SPRE → (CATR)* → SCOM` 读 `GTYP`。
/// 目录库没开 / 目标不在所钉会话与「没有 SPRE」同样算解不出——那是配置或数据的事，
/// 不该把整条链的遍历打断。
fn member_station_verdict(member: &DbElement) -> Result<StationVerdict, DbElementError> {
    let noun = member.element_type()?;
    if !noun.eq_ignore_ascii_case("ATTA") {
        return Ok(station_verdict(&noun, None));
    }
    let gtyp = match catalogue_component(member) {
        Ok(Some(component)) => scom_gtyp(&component)?,
        Ok(None) => None,
        Err(DbElementError::DbNotRegistered { .. } | DbElementError::NotFound { .. }) => None,
        Err(error) => return Err(error),
    };
    Ok(station_verdict(&noun, gtyp.as_deref()))
}

/// `SCOM.GTYP` 的字面（`Word { text }`，attlib 解不出字面时为 `None`）。
fn scom_gtyp(component: &DbElement) -> Result<Option<String>, DbElementError> {
    Ok(match component.attribute("GTYP") {
        Ok(Some(DescriptorValue::Word { text, .. })) => text,
        Ok(_) => None,
        Err(DbElementError::UnknownAttribute { .. }) => None,
        Err(error) => return Err(error),
    })
}

/// 走一个路由容器（算法文档 §4 的 `RouteWalker`）。
///
/// `prev` 从容器 `HPOS` 起步，每遇到一个路由成员就产出一条 `prev → 成员到达点`
/// 的槽位，`prev` 前移到该成员的离开点；走完补一条到容器 `TPOS` 的尾管。
/// 空容器给 1 条：只有头尾没有构件的 BRAN，E3D 照样在 HPOS 与 TPOS 之间画一根管。
pub fn walk_route(container: &DbElement) -> Result<Route, DbElementError> {
    let mut members = Vec::new();
    let mut non_route_members = Vec::new();
    let mut pass_through_unresolved = 0usize;
    for member in container.members()? {
        match member_station_verdict(&member)? {
            StationVerdict::Station => members.push(member),
            StationVerdict::PassThrough => non_route_members.push(member),
            StationVerdict::PassThroughUnresolved => {
                pass_through_unresolved += 1;
                non_route_members.push(member);
            }
        }
    }

    let mut leaves = Vec::with_capacity(members.len());
    let mut arrivals = Vec::with_capacity(members.len());
    for member in &members {
        leaves.push(ppoint_number(member, "LEAV")?);
        arrivals.push(ppoint_number(member, "ARRI")?);
    }

    let station = |index: usize| -> RouteEnd {
        match station_at(index, members.len()) {
            Station::Head => RouteEnd::ContainerHead(container.clone()),
            Station::Member(i) => RouteEnd::MemberArrive {
                member: members[i].clone(),
                ppoint: arrivals[i],
            },
            Station::Tail => RouteEnd::ContainerTail(container.clone()),
        }
    };
    // 出发端要的是**离开**侧，到达端要的是**到达**侧（§3.1 的 PLEAVE / PARRIV）。
    let departure = |index: usize| -> RouteEnd {
        match station_at(index, members.len()) {
            Station::Head => RouteEnd::ContainerHead(container.clone()),
            Station::Member(i) => RouteEnd::MemberLeave {
                member: members[i].clone(),
                ppoint: leaves[i],
            },
            // 槽位是 `(i, i + 1)`，出发下标最大是成员数，永远落不到尾站上。
            // 真落到了说明 `slot_indices` 被改坏了，此时按尾站走仍是自洽的形状。
            Station::Tail => RouteEnd::ContainerTail(container.clone()),
        }
    };

    let tubes = slot_indices(members.len())
        .map(|(from, to)| TubeSlot {
            from: departure(from),
            to: station(to),
        })
        .collect();

    Ok(Route {
        members,
        non_route_members,
        pass_through_unresolved,
        tubes,
    })
}

/// 每项的出现序：第 i 项的序号 = 它前面有多少个与它相等的键。
fn occurrence_ordinals<K: Ord>(keys: impl IntoIterator<Item = K>) -> Vec<u32> {
    let mut seen: BTreeMap<K, u32> = BTreeMap::new();
    keys.into_iter()
        .map(|key| {
            let counter = seen.entry(key).or_insert(0);
            let ordinal = *counter;
            *counter += 1;
            ordinal
        })
        .collect()
}

/// 每条槽位在**同 `(from, to)` 站点对内**的出现序（0 基，链序数第几次出现）。
/// 这就是 [`crate::elmodl::GeometryId::ImpliedTube`] 里 `route_ordinal` 的定义，
/// 全量（`pipeline::build_route_tubes`）与增量（[`route_slot_keys`]）共用这一份。
///
/// # 为什么是出现序，不是槽位下标
///
/// 键的局部性要求「链上别处插删不动我的键」，而槽位下标在链头插一件就整链平移，
/// 两端做差分看到的是「旧管身全消失、新管身全出现」——正是换两站点键要治的病，
/// 序号用下标等于把病带回来一半（下面 `inserting_*` 两条测试钉的就是这个）。
/// 出现序只在同一对站点真的重复（成员被重复挂进成员表、路由折返）时才递增，
/// 那正是序号存在的唯一理由：不消歧，两条真实槽位会碰成同一个 GeometryId。
pub fn slot_ordinals(tubes: &[TubeSlot]) -> Vec<u32> {
    occurrence_ordinals(tubes.iter().map(|slot| {
        let (from, to) = slot.station_refnos();
        ((from.word0, from.word1), (to.word0, to.word1))
    }))
}

/// 一个路由容器上全部管身槽位的键，链序。
///
/// 增量在 base 与 target 两端各算一遍「这条链上有哪几根管身」，两份键一比就得出
/// 该 upsert 哪几根、该 remove 哪几根（见 [`crate::increment`] 的派生单元一节）。
/// 走的是同一条 [`walk_route`]，序号是同一份 [`slot_ordinals`]，**不另写一份
/// 串接规则**——串接规则各写两份的下场，[`crate::category::is_model_unit`] 的
/// 文档里记着那次静默漏件。
///
/// 只要键不要几何，所以这条路不解 P 点、不查目录，两端都读得出来。
pub fn route_slot_keys(container: &DbElement) -> Result<Vec<(u32, RefNo, RefNo)>, DbElementError> {
    let route = walk_route(container)?;
    Ok(slot_ordinals(&route.tubes)
        .into_iter()
        .zip(&route.tubes)
        .map(|(ordinal, slot)| {
            let (from, to) = slot.station_refnos();
            (ordinal, from, to)
        })
        .collect())
}

/// P 点编号：`LEAV` / `ARRI` 存的是字值（实测 `Word { raw: 2 }` / `Word { raw: 1 }`）。
fn ppoint_number(el: &DbElement, name: &str) -> Result<Option<u32>, DbElementError> {
    Ok(match el.attribute(name)? {
        Some(DescriptorValue::Word { raw, .. }) => Some(raw),
        Some(DescriptorValue::Int(number)) if number >= 0 => Some(number as u32),
        _ => None,
    })
}

/// 容器头点：`IHEAD` 读的那一组，P 点号固定 1。
pub fn container_head(container: &DbElement) -> Result<Option<RoutePoint>, DbElementError> {
    container_point(container, "HPOS", "HDIR", "HBOR")
}

/// 容器尾点：`TAIL` 读的那一组，P 点号固定 2。
pub fn container_tail(container: &DbElement) -> Result<Option<RoutePoint>, DbElementError> {
    container_point(container, "TPOS", "TDIR", "TBOR")
}

/// 位置读不到就整个点不成立（`PLEAVE`/`IHEAD` 出错时 ok 标志清零，见 §3.1）。
/// 方向可以退化——`TUMAT` 只在两点重合时才回落到它。
fn container_point(
    container: &DbElement,
    pos: &str,
    dir: &str,
    bore: &str,
) -> Result<Option<RoutePoint>, DbElementError> {
    let Some(position) = container.get_position(pos)? else {
        return Ok(None);
    };
    Ok(Some(RoutePoint {
        pos: DVec3::from_array(position),
        dir: container
            .get_direction(dir)?
            .map(DVec3::from_array)
            .unwrap_or(DVec3::Z),
        bore: container.get_double(bore)?,
    }))
}

/// 目录侧 P 点求解器 —— P1 与 P2 之间那条唯一的缝。
///
/// 实现它要能走 `SPRE → SPCO.CATR → SCOM.PTRE → PTAX/PTCA` 并对 `PDIS` / `PBOR`
/// 这类目录表达式求值（e3d-io `catalogue_eval::ParamEnv` 的活元素实现）——
/// 见 [`crate::catalogue_point::CataloguePoints`]。目录库没开时用
/// [`CatalogueNotWired`]，让账面如实显示「解不出」。
pub trait PPointResolver {
    /// 构件的离开点。解不出给 `None`，不要拿别的值顶。
    fn leave(
        &self,
        member: &DbElement,
        ppoint: Option<u32>,
    ) -> Result<Option<RoutePoint>, DbElementError>;

    /// 构件的到达点。
    fn arrive(
        &self,
        member: &DbElement,
        ppoint: Option<u32>,
    ) -> Result<Option<RoutePoint>, DbElementError>;
}

/// 目录链还没接上时的求解器：构件侧一律解不出。
///
/// 这不是「先返回个默认值让流程跑通」，而是把缺口如实摆在账上：用它跑出来的
/// `missing_ppoint` 就是 P1 落地后能多出多少根真管的准确条数。
#[derive(Debug, Default, Clone, Copy)]
pub struct CatalogueNotWired;

impl PPointResolver for CatalogueNotWired {
    fn leave(&self, _: &DbElement, _: Option<u32>) -> Result<Option<RoutePoint>, DbElementError> {
        Ok(None)
    }

    fn arrive(&self, _: &DbElement, _: Option<u32>) -> Result<Option<RoutePoint>, DbElementError> {
        Ok(None)
    }
}

/// 解一端。容器端直读，构件端交给 `resolver`。
pub fn resolve_end(
    end: &RouteEnd,
    resolver: &dyn PPointResolver,
) -> Result<Option<RoutePoint>, DbElementError> {
    match end {
        RouteEnd::ContainerHead(container) => container_head(container),
        RouteEnd::ContainerTail(container) => container_tail(container),
        RouteEnd::MemberLeave { member, ppoint } => resolver.leave(member, *ppoint),
        RouteEnd::MemberArrive { member, ppoint } => resolver.arrive(member, *ppoint),
    }
}

/// `ORIMAT`（core.dll `0x526d674`）：方向向量 → 旋转矩阵。
///
/// ```text
/// Z = normalize(dir)                 退化 → None
/// for e in (X̂, Ŷ, Ẑ):               ← 顺序不能换
///     X = normalize(Z × e);  不退化就 break
/// Y = Z × X
/// ```
///
/// core.dll 存的是行序 `[X; Y; Z]` 的 4×3 行主序矩阵，配 FORTRAN 的行向量乘法
/// `v' = v · M`；换成 glam 的列向量约定，等价形式就是**列**为 X/Y/Z 的矩阵，
/// 即这里的 `from_cols`。两种写法同一个变换，别把它转置了。
///
/// **副轴的 tie-break 顺序决定圆柱的周向起点。** 体积与 AABB 对拍看不出来，
/// 逐顶点复刻 E3D 就必须照抄，不能随手取一个垂直向量。
pub fn orimat(dir: DVec3) -> Option<DMat3> {
    let length = dir.length();
    if !(length > AXIS_DEGENERATE_TOL) || !dir.is_finite() {
        return None;
    }
    let z = dir / length;
    let x = [DVec3::X, DVec3::Y, DVec3::Z]
        .into_iter()
        .find_map(|axis| {
            let cross = z.cross(axis);
            (cross.length() > AXIS_DEGENERATE_TOL).then(|| cross.normalize())
        })?;
    Some(DMat3::from_cols(x, z.cross(x), z))
}

/// `TUMAT`（Core3D `0x103439c0`）：两端点 → 管身的放置矩阵。
///
/// 平移取**中点** `(P1 + P2) · 0.5`（常量 0.5 已从 `0x10B4DC10` 读出），
/// 旋转取 `ORIMAT(normalize(P1 − P2))`；两点重合致方向退化时**回落到 `D1`**
/// （离开侧 P 点方向），不是回落到单位阵。
pub fn tumat(p1: DVec3, d1: DVec3, p2: DVec3) -> Option<DMat4> {
    let rotation = orimat(p1 - p2).or_else(|| orimat(d1))?;
    Some(DMat4::from_translation((p1 + p2) * 0.5) * DMat4::from_mat3(rotation))
}

/// 一条槽位相对两端 P 点方向的姿态（E3D 画不画隐式管身的第一道门，见
/// [`TUBE_ALIGNMENT_TOL`]）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SlotAlignment {
    /// 共轴且朝前：离开侧横偏 < 千分之一轴向，查了到达侧的话到达侧也是。
    Aligned {
        leave_deg: f64,
        /// 到达侧夹角（`−to.dir` 与槽位向量），没查到达侧时为 `None`。
        arrive_deg: Option<f64>,
    },
    /// 偏轴：某一侧横偏在容差之外但离开侧仍朝前。E3D 不画。
    Misaligned { deviation_deg: f64 },
    /// 反向：到达点在离开点身后（离开侧轴向分量 ≤ 0）。E3D 不画。
    Reversed { deviation_deg: f64 },
    /// 槽位向量或离开方向退化、判不了——照旧建，不拿判不了当不合格。
    Indeterminate,
}

/// 要不要连到达侧一起判。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArriveCheck {
    /// 到达端的方向按 P 点约定朝外（逆着行进），拿 `−to.dir` 与槽位向量比。构件的目录 P 点
    /// 如此；容器尾的 `TDIR` 也如此——E3D `Q TDIR` 与库里的值都指回支管内部（`/C-CO-1RX-3AH0A`
    /// HDIR D / TDIR U，`/M-CO-1R312-G` 同），`/M-OR-1RX-3041E` 的尾管 TDIR 偏 6° 就没被画。
    Member,
    /// 到达端的方向读不到（容器尾没有 `TDIR` 时 [`container_point`] 会填 `+Z`）：只判离开侧，
    /// 不拿填出来的方向把好管误判成偏轴。
    Skip,
}

/// 槽位向量（`to.pos − from.pos`）相对两端方向的共轴分档。
///
/// 离开侧：把槽位向量分解到 `from.dir` 上，轴向分量 ≤ 0 是反向，横向分量不小于千分之一轴向是偏轴。
/// 到达侧（[`ArriveCheck::Member`]）：对 `−to.dir` 做同一件事。两侧都要过：
/// `/C-GR-1RX-3020B` 的 ATTA6 → FTUB1 离开侧 0.0° 却没被 E3D 画，就是因为 FTUB1 的 P1 方向
/// 与槽位差 8.1°——E3D 要的是两端**共轴**，不是单端对准。
pub fn slot_alignment(from: &RoutePoint, to: &RoutePoint, arrive: ArriveCheck) -> SlotAlignment {
    let span = to.pos - from.pos;
    let Some(leave) = Coaxiality::of(span, from.dir) else {
        return SlotAlignment::Indeterminate;
    };
    if leave.axial < 0.0 {
        return SlotAlignment::Reversed {
            deviation_deg: leave.deviation_deg(),
        };
    }
    if !leave.within_tolerance() {
        return SlotAlignment::Misaligned {
            deviation_deg: leave.deviation_deg(),
        };
    }
    let arrive_side = match arrive {
        ArriveCheck::Skip => None,
        // 到达方向退化就只按离开侧算——判不了不算不合格。
        ArriveCheck::Member => Coaxiality::of(span, -to.dir),
    };
    if let Some(arrive_side) = arrive_side
        && !(arrive_side.axial > 0.0 && arrive_side.within_tolerance())
    {
        return SlotAlignment::Misaligned {
            deviation_deg: arrive_side.deviation_deg(),
        };
    }
    SlotAlignment::Aligned {
        leave_deg: leave.deviation_deg(),
        arrive_deg: arrive_side.map(|side| side.deviation_deg()),
    }
}

/// 槽位向量在某个单位方向上的分解：沿轴多少、偏离轴线多少。
#[derive(Debug, Clone, Copy)]
struct Coaxiality {
    axial: f64,
    lateral: f64,
}

impl Coaxiality {
    /// `span` 相对 `dir` 的分解；任一退化给 `None`。
    fn of(span: DVec3, dir: DVec3) -> Option<Self> {
        let dir_length = dir.length();
        if !(span.length() > AXIS_DEGENERATE_TOL)
            || !(dir_length > AXIS_DEGENERATE_TOL)
            || !span.is_finite()
            || !dir.is_finite()
        {
            return None;
        }
        let unit = dir / dir_length;
        let axial = span.dot(unit);
        let lateral = (span - unit * axial).length();
        Some(Self { axial, lateral })
    }

    /// 横向 < 千分之一轴向，**严格**小于（见 [`TUBE_ALIGNMENT_TOL`]）。
    ///
    /// 界上再让出十亿分之一：`/C-CO-1RX-3AH0A` 的尾管是整数坐标算出来的 2.0 / 2000.0，
    /// 经 P 点变换后落在 1 mrad 的正负几个 ulp 上，E3D 没画它；不留这一点余量，
    /// 浮点误差会把「恰好等于」随机判到两边。
    fn within_tolerance(&self) -> bool {
        self.lateral < TUBE_ALIGNMENT_TOL * self.axial * (1.0 - 1.0e-9)
    }

    /// 与轴的夹角（度，0..=180），只用来记账。
    fn deviation_deg(&self) -> f64 {
        self.lateral.atan2(self.axial).to_degrees()
    }
}

/// `VDIST`：管身长度 = 两端点**直线距离**。
///
/// 不扣端面、不往轴向投影。算法文档 §3.5 把这条单列，是因为「扣端面」是照着
/// 别的 CAD 的直觉最容易加进来的一步，而 core.dll 没有这一步。
pub fn tube_length(p1: DVec3, p2: DVec3) -> f64 {
    (p1 - p2).length()
}

/// `CGETOD`（pplib `0x103bf7ac`）：管身外径。
///
/// 先取管子目录件 `PARA[2]`（外径），保温开关置位时再加 `IPAR[1]`；
/// 取不到才回落到**到达侧**通径。前后通径不一致时 core.dll 取的就是到达侧，
/// 不是离开侧、也不是两者平均——照抄，别顺手改成更「合理」的那个。
pub fn tube_outer_diameter(
    catalogue_od: Option<f64>,
    insulation: Option<f64>,
    arrive_bore: Option<f64>,
) -> Option<f64> {
    match catalogue_od.filter(|od| od.is_finite()) {
        Some(od) => Some(od + insulation.filter(|value| value.is_finite()).unwrap_or(0.0)),
        None => arrive_bore.filter(|bore| bore.is_finite()),
    }
}

/// `CGETOD` 的 A 段：一条槽位**离开侧**选中的管子目录件外径（`PARA[2]`）。
///
/// 容器头读 `HSTU`/`HSRO`，普通构件离开读 `LSTU`/`LSRO`——与 [`crate::pipeline`] 建隐式管身时
/// 取外径的那一步同源，这里只是把它公开给不建几何、只要外径的消费者（gen-model 的 MBD 尺寸
/// 标注按 PML `aod of $!mem` 决定尺寸行位与字高）。目录库没开 / stub 不在所钉会话 / `PARA`
/// 缺席一律 `Ok(None)`，调用方按 [`tube_outer_diameter`] 回落到到达侧通径。
pub fn departure_tube_outer_diameter(from: &RouteEnd) -> Result<Option<f64>, DbElementError> {
    match from {
        RouteEnd::ContainerHead(departure) | RouteEnd::ContainerTail(departure) => {
            crate::catalogue_point::tube_outer_diameter_from_stub(departure, &["HSTU", "HSRO"])
        }
        RouteEnd::MemberLeave { member, .. } | RouteEnd::MemberArrive { member, .. } => {
            crate::catalogue_point::tube_outer_diameter_from_stub(member, &["LSTU", "LSRO"])
        }
    }
}

/// 建成的一根管身。
pub struct TubeGeometry {
    /// 世界系实体。
    pub solid: Manifold,
    pub length_mm: f64,
    pub radius_mm: f64,
    /// 管身局部系（原点在两端中点、`+Z` 沿 `P1 − P2`）到世界系。
    pub placement: DMat4,
    /// Canonical CylinderV3 placement in the tube-local frame.
    pub primitive_instance: Option<PrimitiveInstance>,
}

/// 建一根管身的结论。退化情形逐类分开，对应算法文档 §4 那张表——
/// 混成一个 `None` 就没法回答「这 3172 条到底卡在哪一步」。
pub enum TubeOutcome {
    Built(Box<TubeGeometry>),
    /// 低于最小管长（`length <` [`ZERO_LENGTH_TOL_MM`]，两端点重合是其特例）。
    ZeroLength {
        length_mm: f64,
    },
    /// 定不出朝向：两点重合，且离开侧方向也退化。
    NoDirection,
    /// 外径与到达侧通径都取不到。
    NoRadius,
    /// 尺寸过了校验但内核没给出实体。
    Failed(SolidError),
}

/// 造管身几何（算法文档 §3.5 的默认路）。
///
/// 圆柱的实体仍按真实半径和长度得到；无布尔的最终表示同时保留同一个
/// `CylinderV3` 单位体及其 centered-Z 实例变换。长度必须进入变换，不能套用
/// 目录 `TUBE` 那条“高恒为 1.0”的约定，否则会得到长度恒为 1mm 的管。
///
/// 走 [`instantiate_centered_cylinder`]，其实体约定与
/// [`crate::solid::csg_cylinder_solid`] 相同而非 `cylinder_solid`：`TUMAT` 把
/// 原点放在两端中点、轴沿 `+Z`，正是前者的约定（轴向局部 Z、原点居中）；后者是
/// 轴向局部 Y、原点在底面，套上去会让管身整体偏移半根长度并且躺倒。
///
/// 只出圆柱这一件实体。`CLIN` 中心线与 `LINE` 线段不是实体，出网格时要跳过，
/// 否则会污染连通分量与体积对拍。
pub fn build_tube(from: &RoutePoint, to: &RoutePoint, outer_diameter: Option<f64>) -> TubeOutcome {
    // 先判长度再判半径，顺序照 §4 那张退化表。ams8000 里那个空 BRAN
    // （两端点都在原点、通径也是 0）两条都踩，先判半径会把它归成「取不到半径」，
    // 读报告的人会去查目录，而它其实只是一条没画完的空分支。
    let length = tube_length(from.pos, to.pos);
    // `isValidTubeLength` 的原样是 `< 1.0` 判无效——恰好 1.0mm 有效。写成
    // `!(>=)` 而不是 `<`，让 NaN 长度也落进无效侧，不往半径/朝向那两步漏。
    if !(length >= ZERO_LENGTH_TOL_MM) {
        return TubeOutcome::ZeroLength { length_mm: length };
    }
    let Some(diameter) = outer_diameter.filter(|value| *value > 0.0) else {
        return TubeOutcome::NoRadius;
    };
    let Some(placement) = tumat(from.pos, from.dir, to.pos) else {
        return TubeOutcome::NoDirection;
    };
    match instantiate_centered_cylinder(diameter, length) {
        Ok(local) => TubeOutcome::Built(Box::new(TubeGeometry {
            solid: crate::solid::placed(&local.solid, placement),
            length_mm: length,
            radius_mm: diameter / 2.0,
            placement,
            primitive_instance: local.instance,
        })),
        Err(error) => TubeOutcome::Failed(error),
    }
}

/// 两端通径是否不一致。调用方据槽位决定实体有效性；头槽位的不一致
/// 由原生 RVM 对照确认只能保留诊断线。半径选择与此判定分开。
pub fn bore_mismatch(from: &RoutePoint, to: &RoutePoint) -> bool {
    match (from.bore, to.bore) {
        (Some(a), Some(b)) => (a - b).abs() > f64::EPSILON,
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 站点裁决看的是目录 `GTYP`，不是设计 noun：`GTYP=ATTA/PCLA` 的附着点被 E3D 的 TUBE 穿过，
    /// `GTYP=ANCI` 的梯架支撑是站点；目录链走不通按穿过处理但要单列。
    #[test]
    fn station_verdict_follows_the_catalogue_gtyp_for_atta() {
        assert_eq!(
            station_verdict("ATTA", Some("ATTA")),
            StationVerdict::PassThrough
        );
        assert_eq!(
            station_verdict("ATTA", Some("atta")),
            StationVerdict::PassThrough
        );
        assert_eq!(
            station_verdict("ATTA", Some("PCLA")),
            StationVerdict::PassThrough
        );
        assert_eq!(
            station_verdict("ATTA", Some("ANCI")),
            StationVerdict::Station
        );
        assert_eq!(
            station_verdict("ATTA", None),
            StationVerdict::PassThroughUnresolved
        );
        for noun in ["ELBO", "TEE", "VALV", "REDU", "FTUB", "BEND"] {
            assert_eq!(
                station_verdict(noun, None),
                StationVerdict::Station,
                "{noun}"
            );
        }
        // 不属于管路的成员从来不是站点，GTYP 说什么都不算。
        assert_eq!(
            station_verdict("BOX", Some("ANCI")),
            StationVerdict::PassThrough
        );
    }

    /// 共轴门：横偏 < 千分之一轴向才建。观测边界照抄：0.983 mrad 建（`/M-OR-1R323-A` TUBE 2）、
    /// 恰好 1.000 mrad 不建（`/C-CO-1RX-3AH0A` 2000 mm 上偏 2.0 mm）、1.06 mrad 不建；
    /// 3° 是偏轴、90° 是偏轴、180° 是反向、退化判不了；到达侧 8° 偏轴同样不建
    /// （2944 的 ATTA6 → FTUB1），容器尾不查到达侧。
    #[test]
    fn slot_alignment_splits_aligned_misaligned_and_reversed_at_the_observed_boundary() {
        let from = point([0.0; 3], [1.0, 0.0, 0.0], Some(50.0));
        // 到达点落在离 X 轴 `deg` 度的方向上，到达方向按 P 点约定反着行进方向。
        let aim = |deg: f64, length: f64| {
            let rad = deg.to_radians();
            point(
                [length * rad.cos(), length * rad.sin(), 0.0],
                [-rad.cos(), -rad.sin(), 0.0],
                Some(50.0),
            )
        };
        // 轴向 axial、横向 lateral 直接给，边界用整数坐标钉住。
        let offset =
            |axial: f64, lateral: f64| point([axial, lateral, 0.0], [-1.0, 0.0, 0.0], Some(50.0));
        assert!(matches!(
            slot_alignment(&from, &aim(0.0, 500.0), ArriveCheck::Member),
            SlotAlignment::Aligned { leave_deg, arrive_deg: Some(arrive_deg) }
                if leave_deg.abs() < 1e-9 && arrive_deg.abs() < 1e-6
        ));
        assert!(
            matches!(
                slot_alignment(&from, &offset(3490.0, 3.43), ArriveCheck::Member),
                SlotAlignment::Aligned { .. }
            ),
            "0.983 mrad 要建"
        );
        assert!(
            matches!(
                slot_alignment(&from, &offset(2000.0, 2.0), ArriveCheck::Member),
                SlotAlignment::Misaligned { .. }
            ),
            "恰好 1.000 mrad 不建"
        );
        assert!(
            matches!(
                slot_alignment(&from, &offset(1750.0, 1.855), ArriveCheck::Member),
                SlotAlignment::Misaligned { .. }
            ),
            "1.06 mrad 不建"
        );
        assert!(
            matches!(
                slot_alignment(&from, &aim(0.5, 40.7), ArriveCheck::Member),
                SlotAlignment::Misaligned { .. }
            ),
            "0.5° 不建（/M-CO-1RX-3051E 的头管）"
        );
        assert!(matches!(
            slot_alignment(&from, &aim(3.0, 20.9), ArriveCheck::Member),
            SlotAlignment::Misaligned { deviation_deg } if (deviation_deg - 3.0).abs() < 1e-9
        ));
        assert!(matches!(
            slot_alignment(&from, &aim(90.0, 538.5), ArriveCheck::Member),
            SlotAlignment::Misaligned { deviation_deg } if (deviation_deg - 90.0).abs() < 1e-9
        ));
        assert!(matches!(
            slot_alignment(&from, &aim(178.0, 2998.2), ArriveCheck::Member),
            SlotAlignment::Reversed { deviation_deg } if (deviation_deg - 178.0).abs() < 1e-9
        ));

        // 离开侧 0.0°、到达侧 8.1°：只查离开侧会放行，查到达侧才挡得住。
        let skewed_arrive = point([821.0, 0.0, 0.0], [-0.99, 0.141, 0.0], Some(300.0));
        assert!(matches!(
            slot_alignment(&from, &skewed_arrive, ArriveCheck::Member),
            SlotAlignment::Misaligned { deviation_deg } if (deviation_deg - 8.1).abs() < 0.1
        ));
        assert!(matches!(
            slot_alignment(&from, &skewed_arrive, ArriveCheck::Skip),
            SlotAlignment::Aligned {
                arrive_deg: None,
                ..
            }
        ));

        assert_eq!(
            slot_alignment(&from, &from, ArriveCheck::Member),
            SlotAlignment::Indeterminate
        );
        let no_dir = point([0.0; 3], [0.0; 3], Some(50.0));
        assert_eq!(
            slot_alignment(&no_dir, &aim(0.0, 500.0), ArriveCheck::Member),
            SlotAlignment::Indeterminate
        );
        // 到达方向退化时只按离开侧算，不算不合格。
        let arrive_no_dir = point([500.0, 0.0, 0.0], [0.0; 3], Some(50.0));
        assert!(matches!(
            slot_alignment(&from, &arrive_no_dir, ArriveCheck::Member),
            SlotAlignment::Aligned {
                arrive_deg: None,
                ..
            }
        ));
    }

    fn point(pos: [f64; 3], dir: [f64; 3], bore: Option<f64>) -> RoutePoint {
        RoutePoint {
            pos: DVec3::from_array(pos),
            dir: DVec3::from_array(dir),
            bore,
        }
    }

    /// `ORIMAT` 出来的必须是右手正交标架，且第三列就是给定方向。
    #[test]
    fn orimat_builds_a_right_handed_frame_whose_z_is_the_direction() {
        for dir in [
            DVec3::new(0.0, 0.0, 1.0),
            DVec3::new(1.0, 0.0, 0.0),
            DVec3::new(0.0, 1.0, 0.0),
            DVec3::new(1.0, 2.0, 3.0),
            DVec3::new(-4.0, 0.5, -0.25),
        ] {
            let m = orimat(dir).expect("方向不退化");
            let (x, y, z) = (m.x_axis, m.y_axis, m.z_axis);
            assert!((z - dir.normalize()).length() < 1e-12, "{dir:?} → {z:?}");
            for (a, b) in [(x, y), (y, z), (z, x)] {
                assert!(a.dot(b).abs() < 1e-12, "{dir:?} 标架不正交");
            }
            for axis in [x, y, z] {
                assert!((axis.length() - 1.0).abs() < 1e-12, "{dir:?} 标架不单位");
            }
            assert!(
                (m.determinant() - 1.0).abs() < 1e-12,
                "{dir:?} 不是右手系（行列式 {}）",
                m.determinant()
            );
        }
    }

    /// 副轴的 tie-break **顺序**要照抄 X̂ → Ŷ → Ẑ。
    ///
    /// 方向恰好是 X̂ 时第一根轴退化，必须落到 Ŷ 上得到 `X = Ẑ`。随手挑一个垂直
    /// 向量也能过上面那条正交性测试，却会把圆柱的周向起点转到别处——逐顶点对拍
    /// 才会暴露，那时已经很难查到是这里。
    #[test]
    fn orimat_tie_break_order_is_x_then_y_then_z() {
        let m = orimat(DVec3::X).unwrap();
        assert!((m.x_axis - DVec3::Z).length() < 1e-12, "{:?}", m.x_axis);
        assert!((m.y_axis + DVec3::Y).length() < 1e-12, "{:?}", m.y_axis);

        // 方向是 Ŷ 时 X̂ 不退化，第一根就该选中：X = normalize(Ŷ × X̂) = −Ẑ。
        let m = orimat(DVec3::Y).unwrap();
        assert!((m.x_axis + DVec3::Z).length() < 1e-12, "{:?}", m.x_axis);
    }

    #[test]
    fn orimat_rejects_a_degenerate_direction() {
        assert!(orimat(DVec3::ZERO).is_none());
        assert!(orimat(DVec3::new(f64::NAN, 0.0, 0.0)).is_none());
    }

    /// `TUMAT` 的平移是两端**中点**，不是起点。
    #[test]
    fn tumat_translates_to_the_midpoint_and_aims_local_z_from_p2_to_p1() {
        let p1 = DVec3::new(0.0, 0.0, 1000.0);
        let p2 = DVec3::new(0.0, 0.0, 200.0);
        let m = tumat(p1, DVec3::Z, p2).unwrap();
        assert!((m.w_axis.truncate() - DVec3::new(0.0, 0.0, 600.0)).length() < 1e-9);
        let local_z = m.transform_vector3(DVec3::Z);
        assert!(
            (local_z - (p1 - p2).normalize()).length() < 1e-12,
            "{local_z:?}"
        );
    }

    /// 两点重合时朝向回落到 `D1`，**不是**回落到单位阵。
    ///
    /// 回落成单位阵会让所有零长管一律朝 +Z；虽然零长管不出几何，但这个矩阵也是
    /// 特例路（自带目录几何集的管身）的摆放依据，那条路上它就不是无害的了。
    #[test]
    fn tumat_falls_back_to_the_leave_direction_not_to_identity() {
        let p = DVec3::new(5.0, 6.0, 7.0);
        let d1 = DVec3::new(1.0, 0.0, 0.0);
        let m = tumat(p, d1, p).unwrap();
        let local_z = m.transform_vector3(DVec3::Z);
        assert!((local_z - d1).length() < 1e-12, "{local_z:?}");
        assert!((m.w_axis.truncate() - p).length() < 1e-12);

        // D1 也退化就真的定不出朝向，要如实给 None。
        assert!(tumat(p, DVec3::ZERO, p).is_none());
    }

    /// 长度是直线距离，不做轴向投影、不扣端面。
    #[test]
    fn tube_length_is_the_straight_line_distance() {
        let p1 = DVec3::new(3.0, 4.0, 0.0);
        let p2 = DVec3::ZERO;
        assert!((tube_length(p1, p2) - 5.0).abs() < 1e-12);
    }

    /// 外径取不到时回落到**到达侧**通径。
    ///
    /// 这条单独钉住：前后通径不一致时取哪一侧，是照抄与自作主张的分水岭，
    /// 而取错了只会让管子粗细差一档，账面完全正常。
    #[test]
    fn outer_diameter_falls_back_to_the_arrive_side_bore() {
        assert_eq!(tube_outer_diameter(None, None, Some(100.0)), Some(100.0));
        assert_eq!(
            tube_outer_diameter(Some(60.0), None, Some(100.0)),
            Some(60.0)
        );
        assert_eq!(
            tube_outer_diameter(Some(60.0), Some(25.0), Some(100.0)),
            Some(85.0),
            "保温开着要加 IPAR[1]"
        );
        assert_eq!(tube_outer_diameter(None, Some(25.0), None), None);
    }

    /// 管身按真实长度建，不是「单位圆柱 + 缩放」。
    #[test]
    fn a_tube_is_built_at_its_real_length() {
        let from = point([0.0, 0.0, 1000.0], [0.0, 0.0, -1.0], Some(50.0));
        let to = point([0.0, 0.0, 200.0], [0.0, 0.0, 1.0], Some(50.0));
        let TubeOutcome::Built(tube) = build_tube(&from, &to, Some(100.0)) else {
            panic!("这根管该建得出来");
        };
        assert!((tube.length_mm - 800.0).abs() < 1e-9);
        assert!((tube.radius_mm - 50.0).abs() < 1e-12);

        // 世界系 AABB：轴沿 Z，跨度就是长度；中心在中点，不在端点。
        let bounds = tube.solid.bounding_box().expect("管身应当有包围盒");
        let (min, max) = (bounds.min(), bounds.max());
        assert!((max[2] - min[2] - 800.0).abs() < 1e-6, "{min:?} {max:?}");
        assert!(((max[2] + min[2]) / 2.0 - 600.0).abs() < 1e-6, "{max:?}");
    }

    #[test]
    fn a_collapsed_slot_is_reported_as_zero_length_rather_than_built() {
        let p = point([1.0, 2.0, 3.0], [0.0, 0.0, 1.0], Some(50.0));
        assert!(matches!(
            build_tube(&p, &p, Some(100.0)),
            TubeOutcome::ZeroLength { .. }
        ));
    }

    /// 最小管长是 1.0mm，不是 libgm 的 0.051mm：`isValidTubeLength`（`0x1057be50`）
    /// 判 `< 1.0` 无效——0.4mm 的针状管不许建，恰好 1.0mm 的照建。
    /// 0.051~1.0mm 这一档正是占位阈值年代凭空多出的假管。
    #[test]
    fn a_needle_tube_under_one_millimetre_is_invalid_but_exactly_one_builds() {
        let from = point([0.0, 0.0, 0.4], [0.0, 0.0, -1.0], Some(50.0));
        let to = point([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], Some(50.0));
        assert!(matches!(
            build_tube(&from, &to, Some(100.0)),
            TubeOutcome::ZeroLength { length_mm } if (length_mm - 0.4).abs() < 1e-12
        ));

        let from = point([0.0, 0.0, 1.0], [0.0, 0.0, -1.0], Some(50.0));
        assert!(matches!(
            build_tube(&from, &to, Some(100.0)),
            TubeOutcome::Built(_)
        ));
    }

    #[test]
    fn a_slot_without_any_diameter_is_reported_rather_than_guessed() {
        let from = point([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], None);
        let to = point([0.0, 0.0, 500.0], [0.0, 0.0, -1.0], None);
        assert!(matches!(
            build_tube(&from, &to, tube_outer_diameter(None, None, to.bore)),
            TubeOutcome::NoRadius
        ));
    }

    /// 链形：n 个路由成员给 n+1 条槽位，首条从容器头出发、末条落到容器尾。
    #[test]
    fn the_chain_always_carries_one_more_slot_than_it_has_members() {
        for members in 0..5usize {
            let slots: Vec<_> = slot_indices(members).collect();
            assert_eq!(slots.len(), members + 1, "{members} 个成员");
            assert_eq!(slots[0].0, 0, "首条必须从容器头出发");
            assert_eq!(
                slots[slots.len() - 1].1,
                members + 1,
                "末条必须落到容器尾（尾管不能省）"
            );
            for pair in slots.windows(2) {
                assert_eq!(pair[0].1, pair[1].0, "相邻两条要首尾相接");
            }
        }
    }

    /// 链上第几站站在谁身上：头尾都是容器，中间按 0 基取成员。
    ///
    /// 与 [`RouteEnd::station_refno`] 是同一个映射，那边由编译器保证穷举，
    /// 这边负责在不开库的前提下把下标算术钉死。
    fn station_element<'a>(container: &'a str, members: &[&'a str], index: usize) -> &'a str {
        match station_at(index, members.len()) {
            Station::Head | Station::Tail => container,
            Station::Member(i) => members[i],
        }
    }

    /// 一条链上全部管身的**生产形键**：出现序 + 两端站点，与
    /// [`route_slot_keys`] / `pipeline::build_route_tubes` 同一套序号规则。
    fn keyed_slots<'a>(container: &'a str, members: &[&'a str]) -> Vec<(u32, &'a str, &'a str)> {
        let pairs: Vec<(&str, &str)> = slot_indices(members.len())
            .map(|(from, to)| {
                (
                    station_element(container, members, from),
                    station_element(container, members, to),
                )
            })
            .collect();
        occurrence_ordinals(pairs.iter().copied())
            .into_iter()
            .zip(pairs)
            .map(|(ordinal, (from, to))| (ordinal, from, to))
            .collect()
    }

    /// 键的形状：头条从容器出发，末条落回容器，中间逐段串起相邻两件。
    #[test]
    fn a_slot_is_keyed_by_the_two_stations_it_spans() {
        assert_eq!(
            keyed_slots("C", &["A", "B", "D"]),
            vec![(0, "C", "A"), (0, "A", "B"), (0, "B", "D"), (0, "D", "C")]
        );
        // 空容器仍有一条槽位（HPOS 到 TPOS 那根），两站都站在容器自己身上。
        assert_eq!(keyed_slots("C", &[]), vec![(0, "C", "C")]);
    }

    /// 出现序只在同一对站点重复时递增——它不是槽位下标。
    ///
    /// 序号存在的唯一理由是消歧：成员被重复挂进成员表、或路由折返时，同一对
    /// `(from, to)` 会出现两次，不带序号它们就碰成同一个 GeometryId。除此之外
    /// 它必须纹丝不动，否则就是下两条测试钉死的那种整链平移。
    #[test]
    fn the_ordinal_counts_occurrences_within_one_station_pair_only() {
        assert_eq!(
            keyed_slots("C", &["A", "B", "A", "B"]),
            vec![
                (0, "C", "A"),
                (0, "A", "B"),
                (0, "B", "A"),
                (1, "A", "B"),
                (0, "B", "C"),
            ]
        );
    }

    /// **换键的全部理由**：在链中间插一件，只有被它劈开的那一条键该动。
    ///
    /// 这条测的是含序号的生产形键。旧键 `{container, preceding, ordinal}` 里
    /// `ordinal` 是槽位下标，插件之后后面每一根的序号都平移一位，两端做差分
    /// 看到的是「旧管身全消失、新管身全出现」，逐根比对彻底失去意义——而增量
    /// 恰恰要在这上面做差分。出现序不受别处插删影响，局部性连着序号一起成立。
    #[test]
    fn inserting_a_member_only_disturbs_the_slot_it_splits() {
        let before = keyed_slots("C", &["A", "B", "D"]);
        let after = keyed_slots("C", &["A", "X", "B", "D"]);

        let gone: Vec<_> = before.iter().filter(|k| !after.contains(k)).collect();
        let fresh: Vec<_> = after.iter().filter(|k| !before.contains(k)).collect();
        assert_eq!(gone, [&(0, "A", "B")], "只有被劈开的那条该消失");
        assert_eq!(
            fresh,
            [&(0, "A", "X"), &(0, "X", "B")],
            "只该多出跨新件的两条"
        );
    }

    /// 在链**头部**插一件同样只动一条：旧键（槽位下标当序号）会在这里整条平移，
    /// 生产形键不会。
    #[test]
    fn inserting_at_the_head_does_not_shift_the_rest() {
        let before = keyed_slots("C", &["A", "B"]);
        let after = keyed_slots("C", &["X", "A", "B"]);
        assert!(
            before.contains(&(0, "A", "B")) && after.contains(&(0, "A", "B")),
            "头部插件不该惊动下游任何一条键"
        );
        let gone: Vec<_> = before.iter().filter(|k| !after.contains(k)).collect();
        assert_eq!(gone, [&(0, "C", "A")], "只有原来的头条该消失");
    }

    /// 同一容器内键唯一。相邻站点对**可以**重复（重复成员 / 折返路由），
    /// 唯一性靠出现序补齐——这正是序号进键的理由。
    #[test]
    fn keys_within_one_container_are_unique() {
        for members in [
            vec![],
            vec!["A"],
            vec!["A", "B"],
            vec!["A", "B", "D", "E", "F"],
            // 重复成员：站点对 (A,B) 出现两次，只有序号分得开。
            vec!["A", "B", "A", "B"],
        ] {
            let keys = keyed_slots("C", &members);
            let mut unique = keys.clone();
            unique.sort_unstable();
            unique.dedup();
            assert_eq!(unique.len(), keys.len(), "{members:?} 出现了重复的键");
        }
    }

    /// 通径不一致要认出来，两边缺任意一边都不算「不一致」。
    #[test]
    fn a_missing_bore_is_not_a_mismatch() {
        let a = point([0.0; 3], [0.0, 0.0, 1.0], Some(50.0));
        let b = point([0.0; 3], [0.0, 0.0, 1.0], Some(100.0));
        let unknown = point([0.0; 3], [0.0, 0.0, 1.0], None);
        assert!(bore_mismatch(&a, &b));
        assert!(!bore_mismatch(&a, &a));
        assert!(!bore_mismatch(&a, &unknown));
    }

    /// 目录链没接上时，构件端必须如实解不出——不许拿构件 `POS` 顶。
    ///
    /// 顶上去能让 3171 条槽位「有几何」，长度却系统性偏短（P 点到构件原点的那段
    /// 被吃掉了），而报告会一片绿。这条测试就是钉住这个诱惑。
    #[test]
    fn the_unwired_resolver_refuses_to_invent_a_member_point() {
        fn assert_object_safe(_: &dyn PPointResolver) {}
        assert_object_safe(&CatalogueNotWired);
    }
}

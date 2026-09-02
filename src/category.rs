//! noun → 建模方式的分类（对标 `build/MODCMP` 的 `I*COM` 谓词大 switch）。
//!
//! 原则与 Core3D 相同：**先分类再取几何**，几何路径由类别决定。
//! 与 core.dll 不同的是我们把「认识但本期不做」显式区分出来
//! （[`Category::Catalog`] / [`Category::Unsupported`]），谁被跳过报告里必须有数，
//! 静默缺件是最高级别缺陷（gen-model 宪法同款）。
//!
//! 分类表来源：aios-core `pdms_types.rs` 的 noun 家族清单 + ams1112 实测 noun 分布。
//!
//! 路由族（管路与吊架）不写在上面那张手抄表里，而是从 `data/route-nouns.json`
//! 解析出来：那份清单是从活体 E3D 3.1 导出的 `noun_layout.json` 按**属性结构**
//! 筛出来的（容器 = 同时带 `HPOS`+`TPOS`，成员 = 同时带 `ARRI`+`LEAV`），
//! 有 104 个成员 noun。手抄 104 个短名进 Rust 迟早会跟 E3D 版本对不上，
//! 而对不上的表现是「某一族元素整族落 Unknown」——正是这张表最该防的事。

use std::collections::{BTreeMap, BTreeSet};
use std::sync::OnceLock;

use e3d_io::db_element::{DbElement, DbElementError};

/// 一个 noun 的建模路径。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Category {
    /// 纯组织节点（`LISTOP`）：遍历成员，不建模。SITE/ZONE/STRU/FRMW/CMPF…
    List,
    /// 正环拉伸体：PLOO/LOOP 边界环 + HEIG 沿局部 +Z 挤出。PANE/FLOOR/EXTR/AEXTR/SCREED/GWALL。
    LoopExtrusion,
    /// 负环拉伸体（NXTR）：几何同上，挂在属主上做 CSG 扣减（`LNEGIT`/`INGCOM` 语义）。
    NegExtrusion,
    /// 正回转体（REVO）：环剖面绕局部 X 轴转 ANGL 度。
    Revolution,
    /// 负回转体（NREV）。
    NegRevolution,
    /// 多面体（POHE/POLYHE）：显式 B-rep——POLPTL 顶点表 + 面(POLFAC/POGO)/POLOOP 环/
    /// LOOPTS.VXREF 顶点引用（旧式 POHE 则面下直存顶点）。三种非标量图元里唯一
    /// 不过 D2_Profile 的（Core3D `sub_1071DCF0`，7UW4 §3.7.4）。
    Polyhedron,
    /// 负多面体（NPOLYH），挂属主 CSG 扣减。
    NegPolyhedron,
    /// 正基本体，见 [`Prim`]。
    Primitive(Prim),
    /// 负基本体，见 [`Prim`]。
    NegPrimitive(Prim),
    /// 是负几何，但本期建不出来（NDIS/NSLC…）。
    ///
    /// 它必须留在负几何家族里而不是丢进 [`Category::Unsupported`]：负体的账要记在
    /// **属主**头上。落进 unsupported 只会得到一行「NCON: 15」，看不出是哪 15 个
    /// FLOOR 少挖了刀；留在这里，属主的 `negatives_skipped` 会点名到 refno，
    /// RVM 对拍看到体积偏大时能直接对上号。
    NegUnimplemented(&'static str),
    /// 几何集正体（`IGMCOM` 21 + `IG2COM` 5）：`SBOX`/`SCYL`/`TUBE`/`GMSE`…
    ///
    /// 几何集不是设计图元，是**另一条建模路**：core.dll 里由 `CGTCT2` + `sub_10714FC0`
    /// 处理，一个集合元素下挂一批子形状，整体作为一件几何交付。设计图元那条路
    /// （`GTGEOM` 的 `IECOMP`/`INCOMP` 分支）读的是元素自己的 XLEN/DIAM 之类标量。
    ///
    /// 单独立一格而不是塞进 [`Category::Unsupported`]：那 42 个 noun 落 Unknown
    /// 不是「实现没跟上」而是**建模概念缺失**——枚举里从来没有任何一项对应它们，
    /// 混进 unsupported 会让「本期不做的已知项」和「压根没想过的东西」同账。
    ///
    /// 本期记账不建（计划 N1 只关记账，几何留给后续阶段）。
    GeometrySet,
    /// 几何集负体（`INGCOM` 16）：`NSBO`/`NSCY`/`NLCY`/`NTUB`…挂属主做 CSG 扣减。
    ///
    /// 与 [`Category::NegUnimplemented`] 一样留在负几何家族里——它的账要记在**属主**
    /// 头上，落进别处就只剩一行「NSBO: 12」，看不出是哪些属主少挖了刀。
    NegGeometrySet,
    /// 目录件（SPRE/CATR 驱动，走目录库求值）：FITT/PFIT/SBFI/FIXING/CMFI/WALL 族，
    /// 名单以 aios-core `USE_CATE_NOUN_NAMES` 为准。
    /// 本期记账不建模（压 G4 目录表达式求值，二期）。
    Catalog,
    /// 路由容器：`BRAN` / `HANG` / `LUG` / `SUPC` / `TRUNNI`。自己不出几何，
    /// 成员串成一条有序路由，相邻两件之间由运行时派生出隐式管身。
    ///
    /// core.dll 判的是 noun 字典里的 `HNGT` 位（`HTCOMP` 谓词读它）。字典读不到，
    /// 改用等价的结构判据「同时拥有 `HPOS` 与 `TPOS`」——`cachelib/GTTUBE` 里
    /// 硬编码的三个 noun 码（`BRAN`=808220、`TRUNNI`=137403155、`LUG`=537123）
    /// 全部落在这 5 个里，两条独立证据对得上。
    ///
    /// **容器不止 BRAN 和 HANG。** 只认前两个会把 LUG/SUPC/TRUNNI 那一支支撑件
    /// 整片漏掉，而漏掉的表现只是报告里少几行，不报错。
    RouteContainer,
    /// 路由成员：管件与吊架构件，104 个 noun。目录驱动，几何 = 目录 GMRE 程序
    /// 求值（`crate::catalogue`），挂在成员自己的 refno 上，因此它是模型单元
    /// （[`is_model_unit`]）；目录解不到（库没开、SPRE 空）时有依据地不出几何。
    ///
    /// 判据同样是结构式的「同时拥有 `ARRI` 与 `LEAV`」——能进能出即在路由链上。
    /// 与 [`Category::Catalog`] 分开记：目录件是型材/墙族那一路，
    /// 路由成员另有隐式管身要配对，两本账混在一起就看不出管路缺了多少。
    RouteMember,
    /// 图元容器：自己不出几何，成员里的正体各自成件；成员里的负体扣的是**容器之外的
    /// 宿主**，由 `Report::host_negatives` 挂账，本期不做扣减。
    ///
    /// 目前只有 `TMPL`——目录模板在设计侧实例化出来的那层壳（`DESC` 形如
    /// "Flexible End Plate - 150mmx8mm"、"Fire Hose Wheel and Housing"）。出处是
    /// core.dll 的 `LPRMTV`：「属主是 `TMPL` 即算图元」，也就是 Core3D 把它的**孩子**
    /// 当图元逐个走，容器自己不是图元。
    ///
    /// 与 [`Category::List`] 只差一条，而那一条就是它存在的理由：**它消费成员**。
    /// 判 [`Category::Unknown`]（改判前）时那些负体没人认领，`aps250110` 里 1559 件
    /// 整片落进 `orphans`——而那本账的语义是「数据或分类有问题」，它们其实两样都不是。
    ///
    /// # 负体是并排挂的，但**不是**给兄弟准备的
    ///
    /// 成员表长这样（`TMPL 2013286654/25291`，负体在正体前后都有）：
    /// `DPSE DDSE NBOX NCYL NCYL PANE TMRPSE NCYL NCYL`。看起来像是该由那块 `PANE`
    /// 把 4 个 `NCYL` 螺栓孔和 1 个 `NBOX` 扣掉。**实测证否**，见
    /// `elmodl::collect_negatives` 的模块注释：照那条规则跑，5 个消防水带箱的
    /// 31 件零件被自己的墙龛扣得一件不剩，端板只剩两条 8.4mm 窄条。
    ///
    /// 所以这个类别只做两件事：把容器本身从欠账里摘出来，以及**把它的负体交给一本
    /// 名字对的账**。不做扣减——在拿到宿主判据（`NAPP`？连接关系？core.dll 另有机制？）
    /// 之前，任何扣减都是在猜。
    PrimitiveContainer,
    /// Core 的复合构件路径：元素本身是一个 builder 容器，成员分别建模后再聚合。
    Composite(CompositeFamily),
    /// 属主几何的**数据**而非独立实体：PLOO/PAVE/LOOP/VERT/SPINE/CURVE。
    /// 由属主的建模路径消费，遍历时不下钻。
    ProfileData,
    /// 无三维表示（`GMDRAW: Skipping non-3d` 同类）：基准/命名点/接头等。
    NonGraphic,
    /// 认识这个 noun、知道它有几何，但本期算法没实现——必须记账。
    Unsupported(&'static str),
    /// 分类表没认领的 noun——必须记账，宁可显式未知也不静默放行。
    Unknown,
}

/// 复合构件的业务家族。类别只表达走哪条 builder 路，不代表该路径已经实现。
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum CompositeFamily {
    /// `sub_107189A0` 返回 28：NOZZ / ELCONN / EQUCOM。
    CoreClass28,
    Equipment,
    Structural,
    BuildingAssembly,
    CableTray,
    Catalogue,
    PipeComponent,
    HangerComponent,
}

/// 分类之外的独立完成状态；防止“有类别”被统计成“已有模型”。
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum BuildDisposition {
    Built,
    EvidencePending,
    ProvenNonGraphic,
    OutOfCorpus,
}

/// 一个 NOUN 的可审计判定。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NounDecision {
    pub category: Category,
    pub disposition: BuildDisposition,
    pub evidence: &'static str,
}

impl Category {
    /// 是否是挂在属主上的负几何（`INGCOM` 谓词）。
    pub fn is_negative(&self) -> bool {
        matches!(
            self,
            Category::NegExtrusion
                | Category::NegRevolution
                | Category::NegPolyhedron
                | Category::NegPrimitive(_)
                | Category::NegUnimplemented(_)
                | Category::NegGeometrySet
        )
    }

    /// 是否自己产出正实体。
    pub fn is_positive_solid(&self) -> bool {
        matches!(
            self,
            Category::LoopExtrusion
                | Category::Revolution
                | Category::Polyhedron
                | Category::Primitive(_)
        )
    }

    /// 本期是否真的会产出几何（正实体，或能扣得出来的负体）。
    ///
    /// 覆盖率的「会建 / 记账不建」就按这个切。写成显式白名单而不是
    /// `is_positive_solid() || is_negative()` 再逐个减去建不出的负体：那种减法
    /// 每加一个「是负体但本期不建」的类别就要在每个调用点各改一次，漏改的表现是
    /// 覆盖率凭空变高——正是本 crate 的审计最该防的事。
    pub fn builds_geometry(&self) -> bool {
        matches!(
            self,
            Category::LoopExtrusion
                | Category::Revolution
                | Category::Polyhedron
                | Category::Primitive(_)
                | Category::NegExtrusion
                | Category::NegRevolution
                | Category::NegPolyhedron
                | Category::NegPrimitive(_)
        )
    }

    /// 当前类别对应的实际交付状态。
    pub fn build_disposition(&self) -> BuildDisposition {
        if self.builds_geometry() {
            BuildDisposition::Built
        } else {
            match self {
                // 图元容器自己不出几何是有出处的结论（`LPRMTV` 只把它的**孩子**
                // 算图元），不是「还没实现」，所以它跟 List 一样算已证无图形，
                // 不进 evidence_pending 那本待办账。
                Category::List
                | Category::ProfileData
                | Category::NonGraphic
                | Category::PrimitiveContainer => BuildDisposition::ProvenNonGraphic,
                Category::Unknown => BuildDisposition::OutOfCorpus,
                _ => BuildDisposition::EvidencePending,
            }
        }
    }

    /// 本元素的成员会不会被**它自己**消费掉（剖面数据 + 负几何）。
    ///
    /// 直接基本体消费自己的剖面/负体；目录实例与 Core 复合构件也消费挂在自身
    /// 下面的建模数据。后两类即使 builder 尚待补齐，属主关系仍是已确认语义：
    /// AMS1112 的 WALL → SPINE → CURVE/POINSP 就是目录墙的路径数据，不是四个
    /// 独立实体。
    ///
    /// [`Category::PrimitiveContainer`] 在这里，但它「消费」的含义与其余几项不同：
    /// 别人是把成员的几何吃进自己的实体，它只是**认领**——负体的宿主在容器之外，
    /// 本期一刀都不扣，认领的意思仅仅是「这些负体有主、别记成孤儿」。管线据此
    /// 把它们转进 `Report::host_negatives`。
    pub fn consumes_members(&self) -> bool {
        self.is_positive_solid()
            || matches!(
                self,
                Category::Catalog | Category::Composite(_) | Category::PrimitiveContainer
            )
    }
}

/// 基本体细分（libgm `gm_Create*` 一一对应的那批）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Prim {
    /// XLEN/YLEN/ZLEN，POS 居中（`gm_CreateBox`）。
    Box,
    /// DIAM/HEIG，轴向局部 Z，POS 居中（`gm_CreateCylinder`）。
    Cylinder,
    /// DBOT/DTOP/HEIG，局部 Z 轴、原点居中的同轴圆锥台。
    Cone,
    /// XBOT/YBOT/XTOP/YTOP/HEIG + XOFF/YOFF 的棱台（`gm_CreatePyramid`）。
    Pyramid,
    /// RINS/ROUT/HEIG/ANGL，矩形截面绕局部 Z 扫（`GM_RectTorus`）。
    RectTorus,
    /// RINS/ROUT/ANGL，圆管截面绕局部 Z 扫（`GM_CircTorus`）。
    CircTorus,
    /// DBOT/DTOP/HEIG + XOFF/YOFF，局部 Z 轴、原点居中的偏心圆锥台。
    Snout,
}

/// noun 名 → 类别。名字用 e3d-io 反哈希出的规范大写 noun 名。
pub fn classify(noun: &str) -> Category {
    use Category::*;
    match noun {
        // ---- 组织节点（遍历不建模；EQUI/SUBE 自身不带几何，几何在基本体成员上） ----
        "WORL" | "SITE" | "ZONE" | "STRU" | "FRMW" | "SBFR" | "CMPF" | "GPART" | "REST"
        | "SUBS" | "EQUI" | "SUBE" => List,

        // ---- 船体分段容器：自己不出几何，几何全在成员上 ----
        //
        // CWALL（曲面墙）/ CFLOOR（曲面楼板）两个 noun 在 aios-core 的两张权威名单里
        // **都不在**：既不是 `GNERAL_LOOP_OWNER_NOUN_NAMES`（它们不带 PLOO），
        // 也不是 `USE_CATE_NOUN_NAMES`（不吃目录），而 `generation_root` 把它们当
        // 生成根——生成根就是「一整棵子树一起重建」的那个容器。
        //
        // ams1112 实测坐实：CWALL → PANE 1995 / GWALL 122 / STWALL 76 / WALL 14，
        // CFLOOR → PANE 2280 / FLOOR 81，两者自己一个 PLOO 都不带。曾把它们判成
        // 目录件与未实现项，于是 DFS 在这一层整片截断，4275 个 PANE 一个都没走到。
        "CWALL" | "CFLOOR" => List,

        // ---- 管道容器：自己既不吃目录也不带几何，构件全在下面的 BRAN 上 ----
        //
        // 曾判成目录件，于是账上落一行「PIPE 47」，看着像有归属，其实是把一个纯
        // 组织节点记成了待办目录件——二期照着这本账去实现目录求值会扑空。
        "PIPE" | "HVAC" => List,

        // ---- 图元容器：几何在成员上，负体是正体的兄弟（见 Category::PrimitiveContainer） ----
        //
        // `TMPL` 是目录模板在设计侧实例化出来的那层壳（`DESC` 形如
        // "Flexible End Plate - 150mmx8mm"、"Railing Corner Section Size 8"），
        // 不是扶手专有件：`aps250110` 的 370 件挂在 FIXING 227 / HRPOST 124 /
        // HRTERM 6 / FITT 5 / HRPANE 5 / EQUI 2 / HRGATE 1 底下。
        "TMPL" => PrimitiveContainer,

        // ---- Core class 28 复合成员聚合器 ----
        // GTGEOM 强制三者进入 sub_10714FC0；sub_107189A0 映射为 28；类 28 构造器
        // sub_107210A0 逐成员调用 LegacyGeom_CreateBuilderForElement 并串成 builder 链。
        "NOZZ" | "ELCONN" | "EQUCOM" => Composite(CompositeFamily::CoreClass28),

        // ---- 环拉伸家族（aios-core GNERAL_LOOP_OWNER_NOUN_NAMES） ----
        "PANE" | "FLOOR" | "EXTR" | "AEXTR" | "SCREED" | "GWALL" => LoopExtrusion,
        "NXTR" => NegExtrusion,
        "REVO" => Revolution,
        "NREV" => NegRevolution,

        // ---- 多面体（显式 B-rep，7UW4 §3.7.4；结构读法同 gen-model prim_model.rs） ----
        "POHE" | "POLYHE" => Polyhedron,
        "NPOLYH" => NegPolyhedron,

        // ---- 基本体（正负成对；其余记账） ----
        "BOX" => Primitive(Prim::Box),
        "CYLI" => Primitive(Prim::Cylinder),
        "PYRA" => Primitive(Prim::Pyramid),
        "RTOR" => Primitive(Prim::RectTorus),
        "CTOR" => Primitive(Prim::CircTorus),
        "SNOU" => Primitive(Prim::Snout),
        "NBOX" => NegPrimitive(Prim::Box),
        "NCYL" => NegPrimitive(Prim::Cylinder),
        "NPYR" => NegPrimitive(Prim::Pyramid),
        "NRTO" => NegPrimitive(Prim::RectTorus),
        "NCTO" => NegPrimitive(Prim::CircTorus),
        "NSNO" => NegPrimitive(Prim::Snout),

        // ---- `A*` 设备图元族（IECOMP）：设计图元在目录侧的同形孪生 ----
        //
        // 取证（`scripts/a_family_attr_probe.py`，字典实读）：每个 `A*` 的属性集是对应
        // 设计图元的**真子集**，差的 7~8 个全是设计侧记账属性（LOCK/OWNER/ORRF/PURP/
        // REPCOU/TMRREF/TYPE/UDTYPE），**尺寸属性一个不差**——`ABOX` 同样是 XLEN/YLEN/
        // ZLEN，`APYRA` 同样是 XBOT/YBOT/XTOP/YTOP/XOFF/YOFF/HEIG。能力位也逐位相同。
        //
        // 方法本身有对照组：`AEXTR` 早就按同形接在 `LoopExtrusion` 上，它与 `EXTR` 的
        // 关系正是这套判据算出来的那种关系——判据能复现一个已知正确的答案。
        //
        // 曾经只认领了 `AEXTR` 一个，剩下 11 个兄弟整族落 Unknown：认领了一个，
        // 看着像认领了一族。
        "ABOX" => Primitive(Prim::Box),
        "ACYLI" => Primitive(Prim::Cylinder),
        "APYRA" => Primitive(Prim::Pyramid),
        "ARTOR" => Primitive(Prim::RectTorus),
        "ACTOR" => Primitive(Prim::CircTorus),
        "ASNOU" => Primitive(Prim::Snout),

        "CONE" => Primitive(Prim::Cone),
        "SLCY" | "DISH" => Unsupported("positive primitive not implemented in milestone 1"),
        // 正体自己就没实现，孪生体同理——等正体做出来这四个一起接。
        "ACONE" => Primitive(Prim::Cone),
        "ASLCY" | "ADISH" => {
            Unsupported("A* twin of a positive primitive not implemented in milestone 1")
        }
        // 这两个的标量属性同样与 POLYHE / REVO 全同，但它们的几何来自**成员结构**
        // （B-rep 顶点面表 / PLOO 剖面环），而属性表比对说不了成员的事。接线要先
        // 取一个真元素看成员 noun，取证前留在这里显式挂着，别按「看着像」就接。
        "APOLYH" | "AREVO" => Unsupported("A* twin whose member structure is not yet evidenced"),

        "NCON" => NegPrimitive(Prim::Cone),
        "NDIS" | "NSLC" => NegUnimplemented("negative primitive not implemented in milestone 1"),

        // ---- 几何集（core.dll 的另一条建模路，见 Category::GeometrySet） ----
        //
        // 三族共 42 个 noun，全部命中 GTGEOM 级联，此前一个都没认领。它们不是
        // 「实现没跟上」而是建模概念缺失，所以给的是自己的类别而不是 Unsupported。
        //
        // IGMCOM 21 —— 几何集成员（集合里的一个个形状）。
        "BOXI" | "GMSE" | "LCYL" | "LINE" | "LPYR" | "LSNO" | "SBOX" | "SCON" | "SCTO" | "SCYL"
        | "SDIS" | "SDSH" | "SEXT" | "SLINE" | "SLOO" | "SREV" | "SRTO" | "SSLC" | "SSPH"
        | "SVER" | "TUBE" => GeometrySet,
        // IG2COM 5 —— 二维几何集（截面/环/矩形一类的平面元素）。
        "GMSS" | "SANN" | "SPRO" | "SPVE" | "SREC" => GeometrySet,
        // INGCOM 16 —— 几何集负体。`NSBO`/`NSCY`/`NLCY` 曾被当成设计负图元
        // （`NegUnimplemented`），字典里它们 geomset=true 且属 INGCOM：是几何**集**
        // 负体，不是设计图元。既有计划一度据此写「NSBO/NLCY 根本不存在、删或降为
        // Unknown」——照做会抹掉一整族。
        "NBXI" | "NGMS" | "NLCY" | "NLPY" | "NLSN" | "NSBO" | "NSCO" | "NSCT" | "NSCY" | "NSDS"
        | "NSEX" | "NSRE" | "NSRT" | "NSSL" | "NSSP" | "NTUB" => NegGeometrySet,

        // ---- 目录件 / 型材（SPRE/CATR 驱动，二期） ----
        //
        // BRAN / HANG 曾经也在这一行。它们不是目录件而是路由容器，交给下面的
        // route_category 按结构判据认领；PIPE 则两头都不沾，见上面的组织节点。
        "FITT" | "PFIT" | "SBFI" | "CMFI" | "FIXING" | "WALL" | "STWALL" | "GENSEC" | "SCTN" => {
            Catalog
        }

        // ---- 属主几何的数据 ----
        // 后六个是多面体的 B-rep 内脏（顶点表/面/环/顶点引用），由 Polyhedron
        // 属主消费，遍历时不下钻。
        //
        // 面 noun 一度写作 `POGON`——**字典里没有这个 noun**。几何照样出得来
        // （`polyhedron.rs` 按成员位置走，不按名字），错的是账：面落 unknown、
        // 面下的 POLOOP/LOOPTS 落 orphans，而 `accounts_for` 是等式判据、元素没蒸发
        // 就照样平，没有任何门禁会因此变红。真名两个：现代 POLYHE 的面是
        // `POLFAC`(44236870)，旧式 POHE 的面是 `POGO`(832210)。
        "PLOO" | "PAVE" | "LOOP" | "VERT" | "SPINE" | "CURVE" | "POLPTL" | "POIN" | "POLFAC"
        | "POGO" | "POLOOP" | "LOOPTS" => ProfileData,

        // ---- 无三维表示 ----
        "JLDATU" | "PLDATU" | "POINSP" | "PNOD" | "SNOD" | "PJOI" | "SJOI" | "PALJ" | "SELJ"
        | "SCOJ" | "SUBJ" | "SEVE" | "RNODE" | "CNODE" | "APPDAR" | "APPLDA" | "APPLDW" => {
            NonGraphic
        }
        // TEXT 是注释数据；E3D 3.1 字典只有文字与引用属性，不带任何 GTGEOM 家族位。
        "TEXT" => NonGraphic,
        // 库务编号根（ams1112 实测 `17496/1`，名 `/**`，与 WORL `/*` 并列的第二个
        // 索引根）：无成员、无几何，只是 dabacon 的记账元素。
        "MNUM" => NonGraphic,

        // 上面那张手抄表没认领的，交给路由清单按结构判据认；再没有才算欠账。
        _ => route_category(noun).unwrap_or_else(|| dictionary_composite_category(noun)),
    }
}

/// 这个 noun 会不会**自己成为一件模型单元**——即全量管线会为它调用
/// [`crate::elmodl::model_element`]，并期待一件挂在它自己 refno 上的实体。
///
/// 全量产出与增量上卷必须共用这一个判据。两边各写一份名单的后果不是编译错误，
/// 是静默漏件：`SBFI` 一度只写在管线那一侧（目录件里唯独它按设计侧 `DESP` 出套管），
/// 而增量上卷只认 [`Category::is_positive_solid`]，于是**被删掉的 SBFI 永远上卷不到
/// 单元、永远不出 Remove**。真库门上的形状是「ams1112 721→722 全量说消失 3398 件，
/// 增量只移除 3356 件」——而两边各自的账都是平的，没有任何等式判据会因此变红。
///
/// **路由容器不在内，但那不等于增量管不着它。** 路由容器里 RVM 坐实会补管的四族
/// （[`DERIVED_ROUTE_CONTAINER_NOUNS`]）确实会让管线产出几何，可产出的是**挂在
/// 容器名下的 N 根隐式管身**，不是「一件挂在容器自己 refno 上的实体」——这个
/// 函数问的恰恰是后者，所以答案就该是否。
/// 它们由另一条正交判据 [`is_derived_unit`] 认领，走派生单元那条路。
///
/// # 为什么是穷举 `match` 而不是 `is_positive_solid() || ...`
///
/// 这份名单必须与 [`crate::elmodl::model_element`] 的产出臂**逐项对齐**。写成
/// `is_positive_solid()` 时对齐是**隐式**的——它今天恰好等于那四条产出臂，纯属巧合，
/// 没有任何东西钉住它。给 [`Category::GeometrySet`] 补一条产出臂而忘了改这里，
/// 后果不是编译错误：全量照常出几何，增量永远上卷不到它，**被删的那些永远不出
/// Remove**，两边的账各自还都是平的。这正是 `SBFI` 那次静默漏件的机理。
///
/// 所以这里**不留通配符臂**：往 [`Category`] 里加一个变体，这个函数就编译不过，
/// 加变体的人必须当场回答「它算不算一件模型单元」。把「记得同步」这件事从人
/// 交给编译器，是这个函数唯一能给出的真保证。
pub fn is_model_unit(noun: &str) -> bool {
    match classify(noun) {
        // 四条正体路径，与 `model_element` 的四条产出臂一一对应。
        Category::LoopExtrusion
        | Category::Revolution
        | Category::Polyhedron
        | Category::Primitive(_) => true,

        // 目录件本期整体压在二期的目录求值上，唯独 SBFI 的套管尺寸直接挂在设计侧
        // `DESP` 上，不查目录就能出几何（`elmodl::sbfitting_local_solid`）。
        Category::Catalog => matches!(noun, "SBFI" | "WALL"),

        // 路由成员的实体 = 目录 GMRE 程序求值（`crate::catalogue`），键正是
        // element:refno——这个函数问的「挂在自己 refno 上的一件」。全量在管线的
        // RouteMember 臂产出它，增量靠这里上卷到成员本身、走 `model_element` 的
        // RouteMember 臂重建；隐式管身与它无关，照旧走 [`is_derived_unit`]（容器）
        // 那条正交腿。2026-09-01 之前这里是 false：成员实体落在两条增量判据之外，
        // 被删/被改的成员既不重建也不出 Remove——SBFI 那次静默漏件的同型复发，
        // 目录库一进增量门就会咬（DESI-only 门上两边都建不出，所以一直没响）。
        Category::RouteMember => true,

        // 电缆桥架：托盘 `CTRAY` 的实体走目录链（SPRE→TABITE→PRTREF→GPART→CATR→SCOM→GMRE），
        // 键是 element:refno——全量管线的 `Composite(CableTray) if noun == "CTRAY"` 臂
        // 产出它，增量就得上卷到它、经 `model_element` 的同名臂重建。桥架**件**
        // （CTSTRA/CTBEND/…，[`crate::cable_tray::CABLE_TRAY_ELEMENT_NOUNS`]）自己不出
        // 实体，只是把没存的 POS/ORI 现算出来给下面的托盘用，不是单元。
        // 这是 SBFI / RouteMember 同型缺口的第三次现身：2026-09-02 桥架落地时只写了
        // 管线那一侧，增量对 CTRAY 的增 / 删 / 改一律 `no_model`。
        Category::Composite(CompositeFamily::CableTray) => {
            noun.trim().eq_ignore_ascii_case("CTRAY")
        }

        // 以下都不产出挂在自己 refno 上的实体。逐条列而不是一个 `_ => false`：
        // 见上面那段——通配符会把「新变体默认不是单元」这个危险假设变成沉默的默认值。
        Category::List
        | Category::PrimitiveContainer
        | Category::NegExtrusion
        | Category::NegRevolution
        | Category::NegPolyhedron
        | Category::NegPrimitive(_)
        | Category::NegUnimplemented(_)
        | Category::GeometrySet
        | Category::NegGeometrySet
        | Category::RouteContainer
        | Category::Composite(_)
        | Category::ProfileData
        | Category::NonGraphic
        | Category::Unsupported(_)
        | Category::Unknown => false,
    }
}

/// 这个 noun 会不会**在自己名下派生出一批几何**——即全量管线不为它本身出实体，
/// 却会为它产出 N 件挂在它名下、按别的键索引的东西。
///
/// 与 [`is_model_unit`] 正交：那条问「它自己算不算一件」，这条问「它名下会不会长
/// 出几件」。目前只有 E3D 确认会补管身的路由容器落在这一侧，它名下长出的是
/// [`crate::elmodl::GeometryId::ImpliedTube`] 那批隐式管身。
///
/// # 为什么必须单列一条，而不是把容器塞进 `is_model_unit`
///
/// 两者的**增量动作根本不同**。模型单元的增量是「按它的 refno 删掉一件、再画一件」；
/// 派生单元没有「一件挂在容器 refno 上的几何」可删，它的增量是**两端各算一遍这条链
/// 上的槽位键，新的一批 upsert、旧键里不在新批中的那些 remove**。混进 `is_model_unit`
/// 只会让增量对着容器 refno 发一条谁也对不上号的 Remove。
///
/// # 同样不留通配符
///
/// 理由与 [`is_model_unit`] 那段一字不差：往 [`Category`] 里加变体时，加的人必须当场
/// 回答「它名下会不会派生出几何」。让编译器逼问，而不是指望人记得。
///
/// # 这只是 noun 那一半；管线与增量的门是 [`derives_implied_tubes`]
///
/// `BRAN` 一个 noun 两个学科共用：管道分支属主是 `PIPE`，风管分支属主是 `HVAC`，
/// 而**风管没有隐式管身**。光看 noun 答不出来，所以真正放在派生门上的谓词是带元素的
/// [`derives_implied_tubes`]，这里的 noun 判据是它的前半段。
/// Route containers for which E3D actually synthesizes inter-member tube bodies.
///
/// HANG deliberately stays a [`Category::RouteContainer`] so its PCLA/HELE members are
/// traversed and modeled, but it is absent here: E3D RVM 24381/177986 exports only the
/// PCLAMP and HELEMENT groups. Treating those two support members as a pipe route creates
/// a spurious 33.8m cylinder between otherwise independent catalogue instances.
pub const DERIVED_ROUTE_CONTAINER_NOUNS: [&str; 4] = ["BRAN", "LUG", "SUPC", "TRUNNI"];

/// 属主 noun 是它的路由容器**不派生**隐式管身：风管。
///
/// E3D HVAC 的直段是显式 `STRT`，构件之间的间隙只在数据一致性检查里报错、不补画；
/// `GTTUBE` 的管身语义只属于管道那一支。唯一一份 HVAC RVM 参照
/// （`evidence/20260902-hvac-ctray-census/rvm-BRAN-24381-30598-primitives.json`，导出时
/// `repre tube on` + `export implied tube into separate containers`）里 `/-VES-E-4-L-2509`
/// 只有 4 Box（STRT）+ 1 Pyramid（BRCO），没有任何圆柱；按 noun 派生的话，同一条分支会在
/// HEAD→STRT1 与 STRT4→TAIL 各长出一根 r=580 的圆管（25.6 m + 17.5 m），ams7997 全库是
/// 245 根、4.46 km（`evidence/20260902-ams7997-bran-audit`）。
pub const NO_IMPLIED_TUBE_OWNER_NOUNS: [&str; 1] = ["HVAC"];

/// 这个容器名下会不会派生隐式管身——管线的路由容器臂、增量的三处上卷 / 级联 / 重算
/// **都问这一个**，两边各写一份门的下场见 [`is_model_unit`] 文档里那次 `SBFI` 漏件。
///
/// 两半：noun 在 [`DERIVED_ROUTE_CONTAINER_NOUNS`] 里（[`is_derived_unit`]），且属主不在
/// [`NO_IMPLIED_TUBE_OWNER_NOUNS`] 里。属主读不出（根、悬空）按「不是风管」处理——
/// 那是管道库的常态形态，不能因为它让管身整条链消失。
pub fn derives_implied_tubes(container: &DbElement) -> Result<bool, DbElementError> {
    if !is_derived_unit(&container.element_type()?) {
        return Ok(false);
    }
    let owner_noun = match container.owner()? {
        Some(owner) if owner.refno() != container.refno() => match owner.element_type() {
            Ok(noun) => noun,
            Err(DbElementError::NotFound { .. } | DbElementError::DbNotRegistered { .. }) => {
                return Ok(true);
            }
            Err(error) => return Err(error),
        },
        _ => return Ok(true),
    };
    Ok(!NO_IMPLIED_TUBE_OWNER_NOUNS.contains(&owner_noun.trim().to_ascii_uppercase().as_str()))
}

pub fn is_derived_unit(noun: &str) -> bool {
    let noun = noun.trim().to_ascii_uppercase();
    match classify(&noun) {
        Category::RouteContainer => DERIVED_ROUTE_CONTAINER_NOUNS.contains(&noun.as_str()),

        // 其余都不在自己名下派生几何。逐条列的理由见上。
        Category::LoopExtrusion
        | Category::Revolution
        | Category::Polyhedron
        | Category::Primitive(_)
        | Category::Catalog
        | Category::List
        | Category::PrimitiveContainer
        | Category::NegExtrusion
        | Category::NegRevolution
        | Category::NegPolyhedron
        | Category::NegPrimitive(_)
        | Category::NegUnimplemented(_)
        | Category::GeometrySet
        | Category::NegGeometrySet
        | Category::RouteMember
        | Category::Composite(_)
        | Category::ProfileData
        | Category::NonGraphic
        | Category::Unsupported(_)
        | Category::Unknown => false,
    }
}

/// 返回分类、实际完成状态和证据来源；报告按 NOUN 固化这三者。
pub fn decision(noun: &str) -> NounDecision {
    let category = classify(noun);
    let evidence = match category {
        Category::Composite(CompositeFamily::CoreClass28) => {
            "Core3D GTGEOM 0x10341d2e -> sub_10714FC0; sub_107189A0 class 28; sub_107210A0 member builder chain"
        }
        Category::Composite(_) => {
            "E3D 3.1 attlib family bit plus Core3D GTGEOM cascade; builder implementation pending"
        }
        Category::RouteContainer | Category::RouteMember => {
            "E3D 3.1 noun_layout structural attributes; Core route semantics"
        }
        Category::NonGraphic => "E3D 3.1 dictionary has no independent 3D builder path",
        Category::PrimitiveContainer => {
            "core.dll LPRMTV treats a member of TMPL as a primitive; the container carries none, and aps250110 shows its negatives cut a host outside the container"
        }
        Category::List | Category::ProfileData => "E3D 3.1 hierarchy/member-consumption semantics",
        Category::Unknown => "noun absent from the supported E3D 3.1 modelling routes",
        _ if category.builds_geometry() => {
            "implemented Rust builder with geometry regression tests"
        }
        _ => "classified modelling route; implementation or corpus evidence pending",
    };
    NounDecision {
        category,
        disposition: category.build_disposition(),
        evidence,
    }
}

/// 路由 noun 清单（`data/route-nouns.json`），与 crate 一起编译进二进制。
///
/// 内嵌而不是运行时读盘：分类表是纯函数，`classify` 不该因为工作目录不同就换答案。
const ROUTE_NOUNS_JSON: &str = include_str!("../data/route-nouns.json");

/// `route-nouns.json` 的结构（只取分类要用的两组短名，其余字段是取证留痕）。
#[derive(serde::Deserialize)]
struct RouteNounsFile {
    #[serde(rename = "routeContainers")]
    containers: Vec<RouteNounEntry>,
    #[serde(rename = "routeMembers")]
    members: Vec<RouteNounEntry>,
}

#[derive(serde::Deserialize)]
struct RouteNounEntry {
    short: String,
}

/// 路由容器与路由成员两个短名集合。
struct RouteNouns {
    containers: BTreeSet<String>,
    members: BTreeSet<String>,
}

/// 解析内嵌清单，只做一次。
///
/// 解析失败直接 panic：清单是编译期就定死的常量，解析不了说明这份构建从一开始
/// 就是坏的。让它安静地退化成「全库路由元素落 Unknown」远比崩掉更难查——那正是
/// 报告全绿、模型缺一整片的那种失败。
fn route_nouns() -> &'static RouteNouns {
    static ROUTE_NOUNS: OnceLock<RouteNouns> = OnceLock::new();
    ROUTE_NOUNS.get_or_init(|| {
        let parsed: RouteNounsFile =
            serde_json::from_str(ROUTE_NOUNS_JSON).expect("内嵌的 route-nouns.json 解析失败");
        RouteNouns {
            containers: parsed.containers.into_iter().map(|e| e.short).collect(),
            members: parsed.members.into_iter().map(|e| e.short).collect(),
        }
    })
}

/// 按路由清单认领 noun：先容器后成员，都不是才是分类表欠账。
fn route_category(noun: &str) -> Option<Category> {
    let route = route_nouns();
    if route.containers.contains(noun) {
        Some(Category::RouteContainer)
    } else if route.members.contains(noun) {
        Some(Category::RouteMember)
    } else {
        None
    }
}

const NOUN_FAMILY_MATRIX_JSON: &str = include_str!("../data/noun-family-matrix.json");

#[derive(serde::Deserialize)]
struct FamilyMatrix {
    nouns: Vec<FamilyMatrixRow>,
}

#[derive(serde::Deserialize)]
struct FamilyMatrixRow {
    noun_name: String,
    families: Vec<String>,
    /// dabacon 字典字段 `graphicsBehaviour`（#5099119）。矩阵里没有这一列的老版本
    /// 按 0 读——0 是「普通元素」，判 [`graphics_ignored_below`] 时不会误伤。
    #[serde(default)]
    gb: u8,
}

/// 字典给一个 noun 的两样东西：GTGEOM 级联家族位，与 `graphicsBehaviour`。
struct NounDictionaryRow {
    families: BTreeSet<String>,
    graphics_behaviour: u8,
}

fn noun_dictionary() -> &'static BTreeMap<String, NounDictionaryRow> {
    static ROWS: OnceLock<BTreeMap<String, NounDictionaryRow>> = OnceLock::new();
    ROWS.get_or_init(|| {
        let parsed: FamilyMatrix = serde_json::from_str(NOUN_FAMILY_MATRIX_JSON)
            .expect("内嵌的 E3D 3.1 noun-family-matrix.json 解析失败");
        parsed
            .nouns
            .into_iter()
            .map(|row| {
                (
                    row.noun_name,
                    NounDictionaryRow {
                        families: row.families.into_iter().collect(),
                        graphics_behaviour: row.gb,
                    },
                )
            })
            .collect()
    })
}

/// dabacon 字典的 `graphicsBehaviour` 字段（#5099119）。字典里没有这个 noun 时 `None`。
///
/// 取证（`gen-model/docs/plans/2026-09-02-increment-update-audit-and-next-plan.md` §1.3，
/// Core3D `DES_DrawList::isGraphicsIgnoredBetween` @ `0x1051c7d0`）：E3D 3.1 的 1931 个
/// noun 里 0 = 普通元素（1652）、**1 = 几何数据容器（109）**、2 = 层级容器
/// WORL/SITE/ZONE/…（98）、3 = 特殊画法 CABLE/HANDRA/ATTA…（72）。只有 `== 1` 的语义
/// 已从指令级坐实（见 [`graphics_ignored_below`]）；2 / 3 的含义未读，不要拿来分派。
pub fn graphics_behaviour(noun: &str) -> Option<u8> {
    let noun = noun.trim().to_ascii_uppercase();
    noun_dictionary()
        .get(&noun)
        .map(|row| row.graphics_behaviour)
}

/// 这个 noun **名下**的元素在 Core3D 里不作为独立图形项（`graphicsBehaviour == 1`）。
///
/// `isGraphicsIgnoredBetween(top, el)` 从 `el.owner()` 上溯到顶层单元，路上只要有一个
/// 中间祖先带这一位，`el` 的图形就被忽略——它是属主几何的**数据**（LOOP 里的 PAVE、
/// POLOOP 里的顶点引用、船体 `D*` 出图数据），不是一件可画的东西。判的是**祖先**，
/// 所以这个谓词问的是「它下面的东西要不要当独立件」，不是「它自己画不画」：
/// gb==1 的 noun 自己是否成件仍由 [`classify`] 说（今天 109 个没有一个是单元，见测试）。
///
/// 全量管线与增量上卷必须共用这一个判据：`pipeline::push_members` 据此把 gb==1 节点
/// 的成员整棵计进 `consumed`、不入栈；`increment::nearest_unit` 跨过 gb==1 祖先时丢掉
/// 其下找到的单元、归到上方；`collect_unit_subtree` 不进 gb==1 节点之下。少写任何一处，
/// 表现都是「core 不画、我们画了」的静默分歧——今天不炸只是因为语料里 gb==1 只有
/// LOOP / POLOOP / MNUM，都早已按 [`Category::ProfileData`] / [`Category::NonGraphic`] 收走。
///
/// 一处有意的近似：core 的上溯止于 `top`（drawlist 顶层元素）——gb==1 的元素自己被加进
/// drawlist 时，它的孩子照画。我们的「顶」是生成根 / 库根，而 gb==1 都是数据容器、不会被
/// 选成根，所以这里不分「它是不是顶」。
pub fn graphics_ignored_below(noun: &str) -> bool {
    graphics_behaviour(noun) == Some(1)
}

/// 字典里 `graphicsBehaviour == 1` 的全部 noun（给测试与报告点名用）。
pub fn graphics_ignored_nouns() -> impl Iterator<Item = &'static str> {
    noun_dictionary()
        .iter()
        .filter(|(_, row)| row.graphics_behaviour == 1)
        .map(|(noun, _)| noun.as_str())
}

/// 用 E3D 3.1 的真实家族位认领剩余级联 NOUN。这里只决定建模路径，状态仍为
/// `EvidencePending`；没有样本的专业族不会因此被伪报为已经建成。
fn dictionary_composite_category(noun: &str) -> Category {
    let Some(families) = noun_dictionary().get(noun).map(|row| &row.families) else {
        return Category::Unknown;
    };
    if families.contains("IGMCOM") || families.contains("IG2COM") {
        return Category::GeometrySet;
    }
    if families.contains("INGCOM") {
        return Category::NegGeometrySet;
    }
    if families.contains("INCOMP") {
        return Category::NegUnimplemented("E3D 3.1 negative design primitive pending");
    }
    // `ICABCO` 排在 `IPFCOM` 前面：电缆桥架是一条自带容器（`CWAY`/`CWBRAN`）与成员的
    // 独立专业线，带这个位就该记在桥架账上，哪怕它同时带 `IPFCOM` 这个最宽的板墙楼板族。
    // 全字典目前只有 `CTRAY` 一例同时带两位——2.10 时它只有 `IPFCOM`，3.1 才补上 `ICABCO`，
    // 顺序不动的话它会和 `CTSTRA`/`CTBEND`/`CWAY` 那批兄弟分家。
    // 这个顺序只决定**记账口径**：core.dll 里 `IPFCOM` 与 `ICABCO` 都进 `sub_10714FC0`，
    // 走哪个谓词进去都是同一条复合几何路，改顺序不改变几何产不产得出来。
    let family = if families.contains("ICABCO") {
        Some(CompositeFamily::CableTray)
    } else if families.contains("IPFCOM") {
        Some(CompositeFamily::Structural)
    } else if families.contains("IFCOMP") {
        Some(CompositeFamily::BuildingAssembly)
    } else if families.contains("IECOMP") {
        Some(CompositeFamily::Equipment)
    } else if families.contains("ICCOMP") {
        Some(CompositeFamily::Catalogue)
    } else if families.contains("IPCOMP") {
        Some(CompositeFamily::PipeComponent)
    } else if families.contains("IHCOMP") {
        Some(CompositeFamily::HangerComponent)
    } else {
        None
    };
    family.map(Category::Composite).unwrap_or(Category::Unknown)
}

/// 路由容器短名全集（`RouteWalker` 与报告分账都要按这 5 个来，不是 2 个）。
pub fn route_container_nouns() -> impl Iterator<Item = &'static str> {
    route_nouns().containers.iter().map(String::as_str)
}

/// 路由成员短名全集。
pub fn route_member_nouns() -> impl Iterator<Item = &'static str> {
    route_nouns().members.iter().map(String::as_str)
}

/// 表现级按学科分档——照 Core3D `build/MODCMP`（0x10251012）给 `build/ELMODL` 挑级的那张表。
///
/// `REPRESENTATION LEVEL …` 不是一个数，是六个全局（`build/DEDRRP` 0x1024D019 逐条写，
/// `Q REPRE` 逐行打印）：`LEVEL n`（通用，DrawList 键 19）、`LEVEL PIPE n`（键 20）、
/// `LEVEL NOZZ n`（21）、`LEVEL STRU n`（22）、`LEVEL CABLE n`（48）、`LEVEL HVAC n`（86）。
/// MODCMP 按被建元素的 noun 谓词挑一个交给 ELMODL → `build/LEVSET`，默认「画不画」回调
/// `csg_gml4/DR3DWA`（0x1008C279）再经 `build/LEVGET` 取回，对 `LEVE=[lo,hi]` 做闭区间
/// `lo <= L <= hi`。TTY 会话（`build/LEVINI`）六个都从 0 起；ams7997 基线宏只动了
/// `LEVEL 6` 与 `LEVEL HVAC 6`，所以管道构件在 0 级、风管在 6 级——COUP `/ABCF331DF` 那两片
/// LEVE (6,8) 的 LSNO 在 `LEVEL PIPE 6/8` 下 E3D 照画（`evidence/20260902-repre-level-tty-experiments/`）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum RepreDiscipline {
    /// `REPRESENTATION LEVEL n`——MODCMP 的兜底档：设计基本体、负体、面板等
    /// （`IECOMP` / `INCOMP` 都落这里）。
    General,
    /// `REPRESENTATION LEVEL PIPE n`：`IPCOMP`（管道构件）与 `IHCOMP`（吊架构件）。
    Pipe,
    /// `REPRESENTATION LEVEL HVAC n`：`IHVCOM`。core.dll `nounlib/IHVCOM`（0x561C010）=
    /// noun 的 `PRMF` 逻辑位 ∧ 从当前元素 `CLIMBS` 到 `HVAC` 属主——判的是**属主链**：
    /// 风管 STRT/BEND/TRNS 在 3.1 字典里与管道构件同带 `IPCOMP`，光看 noun 分不出来。
    Hvac,
    /// `REPRESENTATION LEVEL NOZZ n`：`NOZZ` / `ELCONN` / `EQUCOM`。
    Nozzle,
    /// `REPRESENTATION LEVEL STRU n`：`IFCOMP` / `IPFCOM` / `IASLCO` / `IPLCOM` / `IG2COM` /
    /// `IFICOM` / `IJOCOM` / `IPRCOM`。
    Structure,
    /// `REPRESENTATION LEVEL CABLE n`：`ICABCO`。
    Cable,
}

impl RepreDiscipline {
    pub const COUNT: usize = 6;
    pub const ALL: [RepreDiscipline; Self::COUNT] = [
        RepreDiscipline::General,
        RepreDiscipline::Pipe,
        RepreDiscipline::Hvac,
        RepreDiscipline::Nozzle,
        RepreDiscipline::Structure,
        RepreDiscipline::Cable,
    ];

    /// 在 [`Self::ALL`] 里的位置，给按学科缓存的表当下标。
    pub fn index(self) -> usize {
        match self {
            RepreDiscipline::General => 0,
            RepreDiscipline::Pipe => 1,
            RepreDiscipline::Hvac => 2,
            RepreDiscipline::Nozzle => 3,
            RepreDiscipline::Structure => 4,
            RepreDiscipline::Cable => 5,
        }
    }

    /// `Q REPRE` 里那一行的写法（`Representation level PIPE 0`），报告记账用。
    pub fn command(self) -> &'static str {
        match self {
            RepreDiscipline::General => "LEVEL",
            RepreDiscipline::Pipe => "LEVEL PIPE",
            RepreDiscipline::Hvac => "LEVEL HVAC",
            RepreDiscipline::Nozzle => "LEVEL NOZZ",
            RepreDiscipline::Structure => "LEVEL STRU",
            RepreDiscipline::Cable => "LEVEL CABLE",
        }
    }

    /// 覆盖本轮进程这一档级别的环境变量。
    pub fn env_var(self) -> &'static str {
        match self {
            RepreDiscipline::General => "E3D_REPRE_LEVEL",
            RepreDiscipline::Pipe => "E3D_REPRE_LEVEL_PIPE",
            RepreDiscipline::Hvac => "E3D_REPRE_LEVEL_HVAC",
            RepreDiscipline::Nozzle => "E3D_REPRE_LEVEL_NOZZ",
            RepreDiscipline::Structure => "E3D_REPRE_LEVEL_STRU",
            RepreDiscipline::Cable => "E3D_REPRE_LEVEL_CABLE",
        }
    }

    /// 缺省级 = ams7997 那批 TTY 基线导出时 `Q REPRE` 打出来的数：宏设了 `LEVEL 6` 与
    /// `LEVEL HVAC 6`，其余停在 `LEVINI` 的 0。GUI / addin 会话另有一套（管道通常 6），
    /// 对拍那类基线用 [`Self::env_var`] 覆盖。
    pub fn default_level(self) -> u32 {
        match self {
            RepreDiscipline::General | RepreDiscipline::Hvac => 6,
            RepreDiscipline::Pipe
            | RepreDiscipline::Nozzle
            | RepreDiscipline::Structure
            | RepreDiscipline::Cable => 0,
        }
    }
}

/// MODCMP 的 noun 分派。`under_hvac` 是 `IHVCOM` 的 `CLIMBS` 那一半——属主链上有 `HVAC`。
///
/// 顺序照 MODCMP：NOZZ 族 → `IHVCOM` → `IPCOMP` / `IHCOMP` → 结构族 → `ICABCO` → 几何集成员
/// （直接当元素建时按管道级）→ 兜底通用级。所以同时带 `IPFCOM` 与 `ICABCO` 的 `CTRAY` 在
/// 这里归结构级（[`dictionary_composite_category`] 把它记在桥架账上是另一回事）。`ICOCOM`
/// （SCOM）在 MODCMP 里给 -1 即不判级，SCOM 不会作为被建元素出现，这里落兜底。`IASLCO`
/// 不在家族矩阵里（矩阵里没有 noun 只带它），列出来只为与 MODCMP 逐位对照。
pub fn repre_discipline(noun: &str, under_hvac: bool) -> RepreDiscipline {
    let noun = noun.trim().to_ascii_uppercase();
    if matches!(noun.as_str(), "NOZZ" | "ELCONN" | "EQUCOM") {
        return RepreDiscipline::Nozzle;
    }
    let families = noun_dictionary().get(&noun).map(|row| &row.families);
    let has = |family: &str| families.is_some_and(|set| set.contains(family));
    let is_component = has("IPCOMP") || has("IHCOMP");
    if under_hvac && is_component {
        return RepreDiscipline::Hvac;
    }
    if is_component {
        return RepreDiscipline::Pipe;
    }
    if [
        "IFCOMP", "IPFCOM", "IASLCO", "IPLCOM", "IG2COM", "IFICOM", "IJOCOM", "IPRCOM",
    ]
    .iter()
    .any(|family| has(family))
    {
        return RepreDiscipline::Structure;
    }
    if has("ICABCO") {
        return RepreDiscipline::Cable;
    }
    if has("IGMCOM") || has("INGCOM") {
        return RepreDiscipline::Pipe;
    }
    RepreDiscipline::General
}

#[cfg(test)]
mod repre_discipline_tests {
    use super::{RepreDiscipline, repre_discipline};

    #[test]
    fn modcmp_dispatch_by_noun_and_hvac_ancestry() {
        assert_eq!(repre_discipline("VALV", false), RepreDiscipline::Pipe);
        assert_eq!(repre_discipline("COUP", false), RepreDiscipline::Pipe);
        // 风管 STRT/BEND 与管道构件同带 IPCOMP，靠属主链分档（IHVCOM = PRMF ∧ CLIMBS HVAC）。
        assert_eq!(repre_discipline("STRT", true), RepreDiscipline::Hvac);
        assert_eq!(repre_discipline("BEND", true), RepreDiscipline::Hvac);
        assert_eq!(repre_discipline("BEND", false), RepreDiscipline::Pipe);
        // 吊架构件（IHCOMP）与管道构件同一档。
        assert_eq!(repre_discipline("HELE", false), RepreDiscipline::Pipe);
        assert_eq!(repre_discipline("NOZZ", false), RepreDiscipline::Nozzle);
        assert_eq!(repre_discipline("FLOOR", false), RepreDiscipline::Structure);
        assert_eq!(repre_discipline("SCTN", false), RepreDiscipline::Structure);
        // MODCMP 先问 IPFCOM 再问 ICABCO：CTRAY 归结构级，只带 ICABCO 的 CWAY 才是桥架级。
        assert_eq!(repre_discipline("CTRAY", false), RepreDiscipline::Structure);
        assert_eq!(repre_discipline("CWAY", false), RepreDiscipline::Cable);
        assert_eq!(repre_discipline("BOX", false), RepreDiscipline::General);
        assert_eq!(repre_discipline("NCYL", false), RepreDiscipline::General);
        assert_eq!(repre_discipline("EQUI", false), RepreDiscipline::General);
        assert_eq!(repre_discipline("BOX", true), RepreDiscipline::General);
    }

    #[test]
    fn tty_baseline_defaults_only_lift_level_and_hvac() {
        for discipline in RepreDiscipline::ALL {
            let lifted = matches!(discipline, RepreDiscipline::General | RepreDiscipline::Hvac);
            assert_eq!(
                discipline.default_level(),
                if lifted { 6 } else { 0 },
                "{discipline:?}"
            );
        }
        let indices: Vec<usize> = RepreDiscipline::ALL.iter().map(|d| d.index()).collect();
        assert_eq!(indices, (0..RepreDiscipline::COUNT).collect::<Vec<_>>());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// ams1112 实测 noun 全集必须每个都有归属，一个 Unknown 都不许有——
    /// 分类表漏一个 noun，那个 noun 的元素就整族缺席，报告里只剩一行 unknown 计数。
    #[test]
    fn every_ams1112_noun_is_claimed() {
        const AMS1112_NOUNS: &[&str] = &[
            "CFLOOR", "CMFI", "CMPF", "CURVE", "CWALL", "FITT", "FIXING", "FLOOR", "FRMW", "GWALL",
            "JLDATU", "LOOP", "MNUM", "NBOX", "NCTO", "NCYL", "NPYR", "NREV", "NRTO", "NXTR",
            "PANE", "PAVE", "PFIT", "PJOI", "PLDATU", "PLOO", "PNOD", "POINSP", "SBFI", "SITE",
            "SJOI", "SNOD", "SPINE", "STRU", "STWALL", "VERT", "WALL", "ZONE",
        ];
        let unknown: Vec<_> = AMS1112_NOUNS
            .iter()
            .filter(|noun| matches!(classify(noun), Category::Unknown))
            .collect();
        assert!(unknown.is_empty(), "1112 的 noun 必须全部认领：{unknown:?}");
    }

    /// 多面体家族全链路接线（7UW4 §3.7.4 落地）：正体产实体并消费成员，
    /// 负体归负几何家族，B-rep 内脏 noun 全部算属主数据、不下钻。
    #[test]
    fn polyhedron_family_is_wired() {
        assert_eq!(classify("POHE"), Category::Polyhedron);
        assert_eq!(classify("POLYHE"), Category::Polyhedron);
        assert_eq!(classify("NPOLYH"), Category::NegPolyhedron);
        assert!(classify("NPOLYH").is_negative());
        assert!(classify("POLYHE").is_positive_solid());
        assert!(classify("POLYHE").consumes_members());
        for noun in ["POLPTL", "POIN", "POLFAC", "POGO", "POLOOP", "LOOPTS"] {
            assert_eq!(classify(noun), Category::ProfileData, "{noun}");
        }
    }

    /// 面 noun 的真名是 `POLFAC`（现代 POLYHE）与 `POGO`（旧式 POHE）。
    ///
    /// 分类表里曾写着字典中并不存在的 `POGON`。那一支永远匹配不上，几何却照出——
    /// 于是面落 unknown、面下的环与顶点引用落 orphans，而等式判据照样平账。
    /// 这条与 `tests/noun_coverage.rs` 的源码扫描一起把这类笔误钉死。
    #[test]
    fn the_polyhedron_face_nouns_are_polfac_and_pogo() {
        assert_eq!(classify("POLFAC"), Category::ProfileData);
        assert_eq!(classify("POGO"), Category::ProfileData);
        // 属主消费剖面数据，所以这两个不该被当成孤儿。
        assert!(classify("POLYHE").consumes_members());
        assert!(classify("POHE").consumes_members());
    }

    /// `A*` 设备图元族：与设计图元同形的那 5 个接进基本体，其余显式挂账。
    ///
    /// 判据是字典属性表实读（见 `classify` 里的取证注释）：`A*` 的属性集是对应
    /// 设计图元的真子集，尺寸属性一个不差。曾经只认领 `AEXTR` 一个，剩下 11 个
    /// 兄弟整族落 Unknown——认领了一个，看着像认领了一族。
    #[test]
    fn the_a_star_equipment_primitives_are_claimed() {
        assert_eq!(classify("ABOX"), Category::Primitive(Prim::Box));
        assert_eq!(classify("ACYLI"), Category::Primitive(Prim::Cylinder));
        assert_eq!(classify("APYRA"), Category::Primitive(Prim::Pyramid));
        assert_eq!(classify("ARTOR"), Category::Primitive(Prim::RectTorus));
        assert_eq!(classify("ACTOR"), Category::Primitive(Prim::CircTorus));
        assert_eq!(classify("ASNOU"), Category::Primitive(Prim::Snout));
        assert_eq!(classify("AEXTR"), Category::LoopExtrusion);
        // 正体没实现的、以及几何来自成员结构还没取证的，一律显式记账，不许落 Unknown。
        assert_eq!(classify("ACONE"), Category::Primitive(Prim::Cone));
        for noun in ["ASLCY", "ADISH", "APOLYH", "AREVO"] {
            assert!(
                matches!(classify(noun), Category::Unsupported(_)),
                "{noun} 应当显式记账，实得 {:?}",
                classify(noun)
            );
        }
    }

    /// 几何集三族 42 个 noun 全部认领，且**本期一个都不建**。
    ///
    /// 它们此前整族落 Unknown——不是实现没跟上，是 `Category` 里从来没有对应概念。
    #[test]
    fn the_geometry_set_families_are_claimed_but_not_built() {
        for noun in ["SBOX", "SCYL", "TUBE", "GMSE", "LINE", "SVER"] {
            assert_eq!(classify(noun), Category::GeometrySet, "{noun} 是几何集正体");
        }
        for noun in ["GMSS", "SANN", "SPRO", "SPVE", "SREC"] {
            assert_eq!(classify(noun), Category::GeometrySet, "{noun} 是二维几何集");
        }
        for noun in ["NSBO", "NSCY", "NLCY", "NTUB", "NGMS"] {
            assert_eq!(
                classify(noun),
                Category::NegGeometrySet,
                "{noun} 是几何集负体"
            );
        }
        for noun in ["SBOX", "GMSS", "NSBO", "NTUB"] {
            assert!(!classify(noun).builds_geometry(), "{noun} 本期不建几何");
        }
    }

    #[test]
    fn negative_predicates_match_the_ingcom_family() {
        assert!(classify("NXTR").is_negative());
        assert!(classify("NBOX").is_negative());
        assert!(classify("NREV").is_negative());
        assert!(!classify("PANE").is_negative());
        assert!(classify("PANE").is_positive_solid());
        assert!(!classify("PLOO").is_positive_solid());
    }

    /// ams1112 出现过的负体 noun 必须条条能建，不许再有「有依据地不出」。
    ///
    /// 这四种曾经是 `NegUnimplemented`：属主的正体照出、负体跳过，后果是那些
    /// FLOOR/GWALL 的体积整整多出一个负体——而 RVM 对拍比的正是体积。带着已知
    /// 原因的偏差去对拍，等于在结果里先撒一把噪声，真正的未知偏差就淹掉了。
    #[test]
    fn every_negative_in_ams1112_can_actually_be_built() {
        let expect = [
            ("NXTR", Category::NegExtrusion),
            ("NREV", Category::NegRevolution),
            ("NBOX", Category::NegPrimitive(Prim::Box)),
            ("NCYL", Category::NegPrimitive(Prim::Cylinder)),
            ("NPYR", Category::NegPrimitive(Prim::Pyramid)),
            ("NRTO", Category::NegPrimitive(Prim::RectTorus)),
            ("NCTO", Category::NegPrimitive(Prim::CircTorus)),
        ];
        for (noun, category) in expect {
            assert_eq!(classify(noun), category, "{noun}");
        }
        // 正体侧的同族必须走同一条几何路径，否则同一个形状会有两套实现。
        assert_eq!(classify("PYRA"), Category::Primitive(Prim::Pyramid));
        assert_eq!(classify("RTOR"), Category::Primitive(Prim::RectTorus));
        assert_eq!(classify("CTOR"), Category::Primitive(Prim::CircTorus));
        assert_eq!(classify("REVO"), Category::Revolution);
    }

    /// 建不出来的负体也算负体：它的账要记在属主头上，不能滑进 unsupported
    /// 变成一行「NCON: 15」——那样就看不出是哪些 FLOOR 少挖了刀。
    ///
    /// 两类都在这条里：设计负图元（`NDIS`/`NSLC`，正体也没实现）
    /// 与几何集负体（`NSBO`/`NSCY`/`NLCY`…，属 INGCOM 的另一条建模路）。
    #[test]
    fn unimplemented_negatives_stay_in_the_negative_family() {
        for noun in ["NSBO", "NCON", "NSNO", "NDIS", "NSCY", "NLCY", "NSLC"] {
            let category = classify(noun);
            assert!(
                category.is_negative(),
                "{noun} 应当仍算负几何，实得 {category:?}"
            );
            assert!(
                !matches!(category, Category::Unsupported(_)),
                "{noun} 落进 unsupported 就丢了属主"
            );
        }
        // 分家：NSLC 是设计负图元（primitive + INCOMP，正体 SLCY），此前整个漏掉；
        // NSBO 那三个字典里 geomset=true，属 INGCOM 的几何集那条路。
        assert!(matches!(classify("NSLC"), Category::NegUnimplemented(_)));
        assert_eq!(classify("NSBO"), Category::NegGeometrySet);
    }

    /// 正基本体、目录实例和复合构件消费自己的建模成员；组织节点仍须下钻。
    #[test]
    fn geometry_owners_consume_their_members() {
        assert!(classify("PANE").consumes_members());
        assert!(classify("BOX").consumes_members());
        assert!(classify("FITT").consumes_members());
        assert!(classify("WALL").consumes_members());
        assert!(classify("NOZZ").consumes_members());
        for noun in ["CWALL", "CFLOOR", "ZONE", "PLOO", "NXTR"] {
            assert!(!classify(noun).consumes_members(), "{noun} 不该消费成员");
        }
    }

    /// ams1112 的容器 noun 必须是可下钻的组织节点。
    ///
    /// 这条是一次真事故的回归：CWALL 曾被判成目录件、CFLOOR 被判成未实现项，
    /// 两者都不下钻，DFS 于是在 FRMW 下一层就停住——全库 30940 个元素只走到 160 个，
    /// 4275 个 PANE 一个都没进管线，而总账里只看得到「catalog: CWALL 121」这样
    /// 一行人畜无害的计数。
    #[test]
    fn hull_containers_are_traversable_lists() {
        for noun in ["CWALL", "CFLOOR"] {
            assert_eq!(
                classify(noun),
                Category::List,
                "{noun} 自己不带 PLOO、也不吃目录，是生成根容器，必须下钻"
            );
        }
        // 反面：真正吃目录的墙族仍然是目录件（aios-core USE_CATE_NOUN_NAMES）。
        for noun in ["WALL", "STWALL", "FITT", "SBFI"] {
            assert_eq!(classify(noun), Category::Catalog, "{noun} 是目录件");
        }
        // GWALL 自带 PLOO，是环拉伸而不是容器。
        assert_eq!(classify("GWALL"), Category::LoopExtrusion);
    }

    /// 路由清单必须真的是从 JSON 解析出来的，而且是那两个数目。
    ///
    /// 数字写死在这里是有意的：清单换了一版而没人注意到，这条会先响。
    #[test]
    fn the_route_lists_come_from_the_embedded_json() {
        assert_eq!(route_container_nouns().count(), 5, "路由容器应当恰好 5 个");
        assert_eq!(route_member_nouns().count(), 104, "路由成员应当恰好 104 个");
        for noun in route_container_nouns() {
            assert_eq!(classify(noun), Category::RouteContainer, "{noun}");
        }
        for noun in route_member_nouns() {
            assert_eq!(classify(noun), Category::RouteMember, "{noun}");
        }
    }

    /// 路由容器是 5 个不是 2 个。
    ///
    /// `GTTUBE` 硬编码的三个 noun 码里就有 LUG 和 TRUNNI，容器判据只写 BRAN+HANG
    /// 会把支撑件那一支整片漏掉——漏掉的表现只是报告少几行，不报错、账还平。
    #[test]
    fn all_five_route_containers_are_recognised() {
        for noun in ["BRAN", "HANG", "LUG", "SUPC", "TRUNNI"] {
            assert_eq!(
                classify(noun),
                Category::RouteContainer,
                "{noun} 是路由容器"
            );
        }
        // PIPE 是纯组织节点：不吃目录、不带几何，构件全在下面的 BRAN 上。
        assert_eq!(classify("PIPE"), Category::List);
    }

    /// `IHCOMP` 读的是 `HNGC`，是**吊架**构件不是 HVAC 构件。
    ///
    /// 这条钉的是全案最阴的一个坑：把 `IHCOMP` 当成「Is Hvac COMPonent」，整个吊架
    /// 族就走不进路由账——而 `visited + consumed` 仍然相等，`accounts_for` 照样通过，
    /// 报告一行都不会红，只是模型里少掉所有吊架。HVAC 另有谓词 `IHVCOM` 读 `HVAC`
    /// 字段，两者在 core.dll 里就是两个函数。
    #[test]
    fn the_hanger_family_is_hangers_not_hvac() {
        assert_eq!(
            classify("HANG"),
            Category::RouteContainer,
            "HANG 是吊架容器"
        );
        // 吊架/支撑构件条条要有名目，一个都不许落 Unknown。
        for noun in [
            "HELE", "HROD", "HNUT", "HPIN", "HANCI", "HSAD", "HACC", "CLEV", "EYNT", "EYRD",
            "UBOL", "BBLT", "SHOE", "PCLA", "PCLI", "SLUG", "SNUB", "WASH", "SWBR", "TRNB", "VSPR",
            "RCPL", "SPAC", "POST",
        ] {
            assert_eq!(
                classify(noun),
                Category::RouteMember,
                "{noun} 是吊架/支撑构件"
            );
        }
        // 反面：HVAC 不是路由 noun，不该被吊架判据顺手收编。E3D 3.1 字典把它的
        // hard base 设为 PIPE，实际是系统组织容器，不是一个独立吊架实体。
        assert_eq!(classify("HVAC"), Category::List);
    }

    /// ams8000 的 8 个管件族 unknown 必须条条被路由清单接住。
    ///
    /// 这 8 个 noun 共 2612 件，此前全部堆在 `unknown_nouns` 里——那本账的语义是
    /// 「分类表欠账」，2612 件欠账等于管道库整体没有归属。
    #[test]
    fn the_eight_ams8000_fitting_families_are_claimed() {
        for noun in [
            "FTUB", "BEND", "ATTA", "ELBO", "STRT", "REDU", "VALV", "WELD",
        ] {
            assert_eq!(
                classify(noun),
                Category::RouteMember,
                "{noun} 是路由成员，不该再落 Unknown"
            );
        }
    }

    /// 「什么算一件模型单元」只能有一份判据：全量按它产出，增量按它上卷。
    ///
    /// 这条钉的是一次真事故。`SBFI` 只写在管线那一侧，增量上卷仍只认正体，于是被删
    /// 的 SBFI 从来不出 Remove；真库门上是「全量说消失 3398 件，增量只移除 3356 件」，
    /// 而两边各自的账都平——`accounts_for` 那类等式判据一个都不会响。
    #[test]
    fn the_model_unit_predicate_covers_the_supported_catalogue_units_only() {
        // SBFI/WALL 是目录件、不是正体，却照样各产出一件挂在自己 refno 上的实体。
        assert_eq!(classify("SBFI"), Category::Catalog);
        assert!(!classify("SBFI").is_positive_solid());
        assert!(is_model_unit("SBFI"));
        assert_eq!(classify("WALL"), Category::Catalog);
        assert!(!classify("WALL").is_positive_solid());
        assert!(is_model_unit("WALL"));

        for noun in ["PANE", "FLOOR", "BOX", "CYLI", "REVO", "POLYHE", "AEXTR"] {
            assert!(is_model_unit(noun), "{noun} 是正体，必然是模型单元");
        }
        // 其余目录件本期不出几何，不能被顺手一起认成单元——那会让增量对着一批
        // 模型集里根本不存在的 refno 发 Remove。
        for noun in ["FITT", "PFIT", "CMFI", "FIXING", "STWALL", "SCTN"] {
            assert_eq!(classify(noun), Category::Catalog, "{noun}");
            assert!(!is_model_unit(noun), "{noun} 本期不产实体");
        }
        // 路由容器会产出隐式管身，但那些几何不按容器 refno 索引，见判据文档。
        for noun in ["BRAN", "HANG", "LUG", "SUPC", "TRUNNI"] {
            assert!(!is_model_unit(noun), "{noun} 的管身不按容器 refno 索引");
        }
        for noun in ["ZONE", "PLOO", "NBOX", "TEXT", "SBOX"] {
            assert!(!is_model_unit(noun), "{noun}");
        }
    }

    /// 路由成员是模型单元；它的容器不是，容器走派生腿。两条判据必须这样分工。
    ///
    /// 这条钉的是 SBFI 同型缺口的第二次现身（2026-09-01 审查轮查出）：管线为
    /// 路由成员产出 element:refno 键的目录实体，而当时两条增量判据都不认领它
    /// ——`is_model_unit(RouteMember)` 是 false，`is_derived_unit` 只认容器且
    /// 执行侧只重算管身。设计侧改/删一个 ELBO，它自己的实体既不重建也不出
    /// Remove；DESI-only 真库门上两边都建不出所以不响，目录库一进门就是
    /// 「全量说没了、增量没删」。修法与 SBFI 同款：判据翻真 +
    /// `model_element` 加 RouteMember 臂（委托目录求值，全量/增量同一份答案）。
    #[test]
    fn route_members_are_model_units_and_only_rvm_confirmed_containers_are_derived() {
        for noun in ["ELBO", "FTUB", "BEND", "ATTA", "GASK"] {
            assert_eq!(classify(noun), Category::RouteMember, "{noun}");
            assert!(is_model_unit(noun), "{noun} 的目录实体挂在自己 refno 上");
            assert!(!is_derived_unit(noun), "{noun} 名下不派生别的键");
        }
        for noun in DERIVED_ROUTE_CONTAINER_NOUNS {
            assert!(!is_model_unit(noun), "{noun} 自己 refno 上没有一件实体");
            assert!(is_derived_unit(noun), "{noun} 名下的管身按两端键派生");
        }
        assert!(!is_model_unit("HANG"));
        assert!(
            !is_derived_unit("HANG"),
            "HANG 只下钻并生成 PCLA/HELE；E3D RVM 不在二者之间补管身"
        );
    }

    /// 托盘 `CTRAY` 是模型单元（全量为它产出 element:refno 键的目录实体），桥架件不是。
    ///
    /// SBFI / RouteMember 同型缺口的第三次：`Composite(_) => false` 把 CTRAY 一并判了否，
    /// 而管线的 `Composite(CableTray) if noun == "CTRAY"` 臂照常出几何——增量对 CTRAY 的
    /// 增 / 删 / 改一律落 `no_model`，被删的托盘永远不出 Remove，账还是平的。
    #[test]
    fn the_cable_tray_itself_is_a_model_unit_and_its_carriers_are_not() {
        assert_eq!(
            classify("CTRAY"),
            Category::Composite(CompositeFamily::CableTray)
        );
        assert!(is_model_unit("CTRAY"), "CTRAY 的目录实体挂在自己 refno 上");
        assert!(!is_derived_unit("CTRAY"));
        // 七个桥架件 noun（`cable_tray::CABLE_TRAY_ELEMENT_NOUNS` 同一份名单）。
        for noun in [
            "CTSTRA", "CTBEND", "CTRISE", "CTTEE", "CTCROS", "CTREDU", "CTCOUP",
        ] {
            assert_eq!(
                classify(noun),
                Category::Composite(CompositeFamily::CableTray),
                "{noun}"
            );
            assert!(
                !is_model_unit(noun),
                "{noun} 只算放置，几何在它下面的 CTRAY 上"
            );
        }
        // 别的复合族照旧不是单元：它们的 builder 还没有，全量也不为它们出实体。
        for noun in ["NOZZ", "ELCONN", "EQUCOM"] {
            assert!(!is_model_unit(noun), "{noun}");
        }
    }

    /// `graphicsBehaviour == 1` 的 109 个 noun 是几何数据容器：**自己不是单元、名下不派生、
    /// 也不能是可下钻的 List**——List 会让全量管线把它们的成员当独立件建出来，而 Core3D
    /// 对这些成员一件都不画（`isGraphicsIgnoredBetween`，取证见 `graphics_ignored_below`）。
    ///
    /// 数字 109 写死是有意的：字典换版 gb 集合变了，这条先响，再去看增量上卷与
    /// `push_members` 的消费口径要不要跟。
    #[test]
    fn graphics_ignored_containers_are_never_units_nor_lists() {
        let ignored: Vec<&str> = graphics_ignored_nouns().collect();
        assert_eq!(ignored.len(), 109, "E3D 3.1 字典里 gb==1 应当恰好 109 个");
        for noun in &ignored {
            assert!(graphics_ignored_below(noun), "{noun}");
            assert!(!is_model_unit(noun), "{noun} 是数据容器，不该成件");
            assert!(!is_derived_unit(noun), "{noun} 名下不派生几何");
            assert_ne!(
                classify(noun),
                Category::List,
                "{noun} 判成 List 会让成员被当独立件下钻"
            );
        }
        // 语料里已知的三个都在集合里，且早已按数据 / 非图形收走。
        for noun in ["LOOP", "POLOOP", "MNUM"] {
            assert!(graphics_ignored_below(noun), "{noun} 是 gb==1");
        }
        assert_eq!(classify("LOOP"), Category::ProfileData);
        assert_eq!(classify("POLOOP"), Category::ProfileData);
        assert_eq!(classify("MNUM"), Category::NonGraphic);
        // 反面：普通元素 0、层级容器 2、特殊画法 3，一个都不许被这条谓词吞掉。
        for noun in ["BOX", "PANE", "BRAN", "PAVE", "PLOO"] {
            assert_eq!(graphics_behaviour(noun), Some(0), "{noun}");
            assert!(!graphics_ignored_below(noun), "{noun}");
        }
        for noun in ["WORL", "SITE", "ZONE"] {
            assert_eq!(graphics_behaviour(noun), Some(2), "{noun}");
        }
        assert_eq!(graphics_behaviour("CABLE"), Some(3));
        assert_eq!(graphics_behaviour("NOT-A-NOUN"), None);
    }

    /// 路由族既不产正体也不是负体，更不消费成员——成员是路由本身，必须继续下钻。
    #[test]
    fn route_nouns_neither_build_nor_consume() {
        for noun in ["BRAN", "HANG", "FTUB", "ELBO"] {
            let category = classify(noun);
            assert!(!category.is_positive_solid(), "{noun}");
            assert!(!category.is_negative(), "{noun}");
            assert!(
                !category.consumes_members(),
                "{noun} 的成员是路由，不能被吃掉"
            );
        }
    }

    /// `TMPL` 是图元容器：它认领自己那批负体，但**一刀都不扣**。
    ///
    /// 改判前它是 [`Category::Unknown`]，`aps250110` 里底下 1282 个 `NCYL`、232 个
    /// `NBOX`、45 个 `NXTR` 共 1559 件负体整片落进 `orphans`——而那本账的语义是
    /// 「数据或分类有问题」，这些负体两样都不是。
    ///
    /// **这条同时钉住一个被实测证否的改法**：负体在成员表里与正体并排
    /// （`DPSE DDSE NBOX NCYL NCYL PANE TMRPSE NCYL NCYL`），看着就该由正体兄弟扣掉。
    /// 真按那样跑，5 个 `TMPL "Fire Hose Wheel and Housing"` 的 31 件零件被自己的
    /// **墙龛**扣得一件不剩，端板只剩两条 8.4mm 窄条——那些负体扣的是容器之外的宿主。
    /// 所以 `is_positive_solid()` 在这里必须是假：`elmodl::collect_negatives` 只向下
    /// 圈定，容器不能有任何一条路径把自己的负体递给兄弟。
    #[test]
    fn tmpl_is_a_primitive_container_that_only_claims_its_negatives() {
        assert_eq!(classify("TMPL"), Category::PrimitiveContainer);
        assert!(
            classify("TMPL").consumes_members(),
            "负体要有主，否则整片落进语义不对的 orphans"
        );
        assert!(
            !classify("TMPL").is_positive_solid(),
            "容器自己不出几何，也不许成为负体的扣减方"
        );
        assert!(!classify("TMPL").builds_geometry());
        assert!(
            !is_model_unit("TMPL"),
            "几何在成员上，容器自己不是一件模型单元"
        );
        assert_eq!(
            decision("TMPL").disposition,
            BuildDisposition::ProvenNonGraphic,
            "LPRMTV 已证它不是图元，这不是「还没实现」"
        );
        // 那批负体仍然是负几何：账要记在属主头上，不许滑进别的本子。
        for noun in ["NCYL", "NBOX", "NXTR"] {
            assert!(classify(noun).is_negative(), "{noun}");
        }
        // 反面：真正的纯组织节点不消费成员，别被这个新类别顺手带走。
        for noun in ["ZONE", "STRU", "FRMW", "PIPE"] {
            assert_eq!(classify(noun), Category::List, "{noun}");
            assert!(!classify(noun).consumes_members(), "{noun} 不该消费成员");
        }
    }

    /// 桥架族要整族记在一起，`CTRAY` 不许被最宽的板墙族顺走。
    ///
    /// `CTRAY` 在 3.1 字典里同时带 `ICABCO` 和 `IPFCOM`，是全字典唯一一个同时带两个
    /// 复合族位的 noun。判据链原先先问 `IPFCOM`，于是这条桥架主干被记成结构件，跟
    /// `CTSTRA`/`CTBEND` 那批兄弟分在两本账上。这种分家不会让任何等式失衡——几何照出
    /// （两个族位在 core.dll 里都进同一个复合几何函数），错的只是「桥架这一族做完没有」
    /// 这个问题的答案，而那正是覆盖矩阵要回答的东西。
    #[test]
    fn the_cable_tray_family_stays_together() {
        for noun in ["CTRAY", "CTSTRA", "CTBEND", "CTREDU", "CTTEE"] {
            assert_eq!(
                classify(noun),
                Category::Composite(CompositeFamily::CableTray),
                "{noun} 是电缆桥架族"
            );
        }
        // 反面：IFCOMP+IPFCOM 都带的 STWELD 仍归结构。桥架优先只针对 ICABCO 这条
        // 独立专业线，不是「谁的族小谁优先」。
        assert_eq!(
            classify("STWELD"),
            Category::Composite(CompositeFamily::Structural),
            "STWELD 是结构焊缝"
        );
    }
}

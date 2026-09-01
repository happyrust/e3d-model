# e3d-model 下一步开发方案（结合 ida-bridge 复核）

> 日期：2026-09-01。范围：`vendor/e3d-model` 当前模型生成算法的审核结论 + 下一步方案。
> 复核手段：读全部源码/证据 + 对活体 IDB（`Core3D.dll` / `core.dll` / `libgeom.dll`）用
> ida-bridge 反编译交叉验证。四数覆盖基线沿用 `docs/noun-coverage-matrix.md`
> （1931 / 327 / 30 / 297 / 0，SNOU/NSNO/ASNOU 接入后 27→30，矩阵提交 1b16935）。
>
> **修订（2026-09-01 下午，913583b 落地后）**：§2.1 的 CRCATI 表初版是按 helper 形参
> 猜的 noun，全部 case 码经 `data/noun-family-matrix.json` 的 `noun_hash` 反查 + CRCATI
> switch 逐臂核对后**多处归属改写**（SREV/SEXT 两对码此前互相张冠李戴、702883 是 SDSH
> 不是 SCYL、599770 是 SSLC、三条"待定性"单臂是 2D 截面 SREC/SANN/SPRO，另捞出漏记的
> 第 20 臂 SDIS）。优先级同步改为按 ams8000+目录库**实跑残余计数**排序
> （evidence/20260901-model-reuse）。

---

## 一、现状：已经做扎实的部分（复核通过）

这些不是本轮要动的地方，列出来是为了把边界钉清楚，避免二次返工。

| 模块 | 现状 | 证据 |
|---|---|---|
| 遍历与总账 `pipeline.rs` | DFS 成员原序；`accounts_for` 硬判据（`visited + consumed == 索引总数`）；隐式管身/宿主负体独立成账不进等式 | 全库账平（ams1112 30940、ams8000 11695） |
| 分类 `category.rs` | 全 noun 有名目，0 落 Unknown（级联内）；路由 5 容器 + 104 成员从 `route-nouns.json` 结构判据认领；A* 图元族；几何集三族；`TMPL` 图元容器 | `tests/noun_coverage.rs` 7 条回归 |
| 直接图元 `elmodl`/`solid` | BOX/CYLI/PYRA/RTOR/CTOR/**SNOU** + A* 孪生；负体 NBOX/NCYL/NPYR/NRTO/NCTO；NXTR/NREV；POLYHE/POHE 显式 B-rep | `solid.rs` 全 builder 齐备 |
| 结构板件 | FLOOR/GWALL/PANE/SCREED 环拉伸，**RVM 逐面对拍已到位** | ams1112：4476/4476 配对，4474 精确、1 在 10mm 弦容差内、1 参考碎片 |
| 路由骨架 `route.rs` | ORIMAT/TUMAT/CGETOD/VDIST 均反编译落地；管身链走法 + 键 + 增量差分 | 对齐 core.dll `ORIMAT@0x526d674`、Core3D `TUMAT@0x103439c0`、pplib `CGETOD@0x103bf7ac` |
| 目录 P 点 `catalogue_point.rs` | `SPRE→SCOM→PTSE` 链；PTAX（轴+距离）/PTCA（坐标+PTCD 方向码）；目录轴 `Pn`/取负；经 e3d-io `DbElementParamEnv` 求目录表达式 | ams8000 516 个 PTCA 端点 100% 解出 |
| 目录件几何 `catalogue.rs` | GMRE 程序求值：SEXT/SBOX/LPYR/**LSNO/SCON**/SCYL/LCYL/SCTO/SRTO/SSPH（+SBOX 刀 NSCY/NSBO/NLCY）+ HVAC 矩形快路；成员局部网格缓存 | 与 CRCATI 各臂逐个对上（见 §二，反查修订版） |
| 设计侧网格复用 `extrusion_reuse.rs` | 环拉伸世代池（键=变体+grow+高度+环组位模式）；布尔参与者按 `baked` 边界直建；增量执行共享池 | 两库改动前后 model.obj/elements.json 逐字节一致；TTY RVM 九行 0 off（evidence/20260901-model-reuse） |
| 增量 `increment.rs`/`element_diff.rs` | 会话窗口差分 → 单元级重建；模型单元/派生单元两条正交判据；**RouteMember 已入射程**（is_model_unit=true，委托 model_catalogue_member_with_cache） | `tests/increment_real.rs` 真库主门 + route_members_are_model_units_… |

**工作树定性（修订）**：上述整摊已在 **913583b** 落地为一个提交（含 G4 目录几何线、
两站点管身键、CGETOD 真外径、WALL/SBFI、复用池与 RouteMember 增量修复）。
`evidence/20260831-pipe-model-generation/README.md` 的「管道零建模」结论已过时；
新基线见 `evidence/20260901-model-reuse/`：ams8000+ams5052/5053/5054 目录库全量
generated=2803（BEND 619 / FTUB 1533 / ELBO 158 / TUBI 334 等），
implied_tubes 3172 中 built 334、catalogue_od 2777、bore_fallback 50、missing_ppoint 6。

---

## 二、ida-bridge 复核的三条关键结论

### 2.1 目录基本体覆盖（修订版：case 码经 noun_hash 反查 + CRCATI switch 逐臂核对）

`CRCATI`（Core3D `sub_10345ACC`）是目录基本体的总 `switch`，按 noun 码派发到 libgm
`GMC*` 建体，共 **20 条派发臂**（初版表 19 条按形参猜 noun，多处张冠李戴，本表为
`data/noun-family-matrix.json` `noun_hash` 反查 + switch 原文核对后的定版）。
「实测需求」列 = ams8000+三目录库全量跑（evidence/20260901-model-reuse）中该 noun 落
`"is not implemented"` 诊断的次数——**优先级按它排，不按臂的多寡**。

| CRCATI case（正/负） | noun（正/负） | helper（MTRENT 名） | 建体语义 | 覆盖 | 实测需求 |
|---|---|---|---|---|---:|
| 1014841 / 828671 | SBOX / NSBO | `sub_103BFCEC` | 盒 | ✅ | — |
| 942751 / 1008005 | **SEXT / NSEX** | `sub_103B7FCC`→`sub_10715170`+`sub_104030F0` | 环组挤出（高度≥1e-6 才建；**无 +0.01 过切**） | ✅ 正体 / ❌ **负体 NSEX 缺** | **NSEX 1593** |
| 968617 / 643505 | **SREV / NSRE** | `sub_103B81CC`（`pplib/QSSREV`）→`sub_10715170`+`sub_10403110`+GMAMEM | 环组回转：PANG 缺省 360、钳 ±360、负角相位 +180；\|角\|>180 且 UQMODN 条件成立时劈两段回转 GMAMEM 组装 | ❌ **缺** | **SREV 425** |
| 785962 / 1026230 | SCYL / NSCY | `sub_103BF3FC` +0.01 | 圆柱（PDIS/PHEI） | ✅（NSCY 作 SBOX 刀） | — |
| 785955 / 1026041 | LCYL / NLCY | `sub_103C0374`（QLCYLI）+0.01 | 圆柱（两距 PTDI/PBDI） | ✅（NLCY 作 SBOX 刀） | — |
| 837417 / 821192 | LSNO / NLSN | `sub_103BFFC4` +0.01 | 锥台管嘴（两径两距+POFF） | ✅ 正体（913583b 经 snout 求值器） | — |
| 818038 / 829400 | SCON / NSCO | `sub_103BFE40` +0.01 | 圆锥（PDIA/PDIS） | ✅ 正体（snout 求值器 dtop=0 路） | — |
| 701101 / 860747 | SSPH / NSSP | `sub_103C2480`（QSSPHE） | 球 | ✅ 正体 | — |
| **912106** | **SDIS**（初版漏记的第 20 臂） | `sub_103C25E0`（`pplib/QSDISC`） | 圆盘：PDIA + PAXI + 沿轴 PDIS 平移 | ❌ **缺** | 0（本库） |
| 702883 / 908861 | **SDSH / NSDS** | `sub_103C0CBC`（`pplib/QSDISH`） | 碟形封头：带符号深度（≤0 翻轴取反）+ 两个绝对值参数 + PAXI/PDIS | ❌ **缺**（`solid::revolve_rings` 可复用） | 0（本库） |
| 599770 / 782015 | **SSLC / NSSL** | `sub_103C0514`（7 出参，4 角） | 斜切圆柱 | ❌ **缺** | 0（本库） |
| 841366 / 927815 | SCTO / NSCT | `sub_103C0E84` | 圆环段 | ✅ 正体 | — |
| 841771 / 938750 | SRTO / NSRT | `sub_103C0FA0` | 矩环段 | ✅ 正体 | — |
| 904404 / 1035518 | LPYR / NLPY | `sub_103C07B4`（QLPYRA） | 棱台 | ✅ 正体 | — |
| 594640 | **SREC** | `sub_103B88BC`（7 出参） | 2D 矩形截面（型材线，非实体臂） | ➖ 属 G1-1 | — |
| 817255 | **SANN** | `sub_103B8E70`（9 参） | 2D 圆环截面（型材线，非实体臂） | ➖ 属 G1-1 | — |
| 840259 | **SPRO** | `sub_103B8410` | 2D 任意截面（型材线，非实体臂） | ➖ 属 G1-1 | — |
| 726491 / 726152 | BOXI / NBXI | `sub_103BF6D8`→GMCBOX | 管身盒 stub（GTTUBG 线） | ➖ 属 G0-3 | — |
| 631901 / 586670 | TUBE / NTUB | `sub_103B8834` | 管身圆柱 stub（GTTUBG 线） | ➖ 属 G0-3 | — |
| 640317 / 3471112 | LINE / SLINE | （无 helper） | 中心线元数据 | ✅ 有意忽略（`ignored_lines`） | — |

**后果**：未实现的臂在 `catalogue.rs` 里落 `"catalogue primitive noun X is not implemented"`
诊断，对应成员**静默少一块几何**（诊断进 `notes`，账仍平，RVM 对拍才看得出体积缺失）。
ams8000 实测残余只有两族：**NSEX 1593（GMRE 内负挤出刀）与 SREV 425（回转体）**——
其余缺臂在本库目录程序里一次都没出现。负体孪生除 NSCY/NSBO/NLCY（作 SBOX 刀）外
**均未接**：NSEX 的量级说明 GMRE 需要**成员级负体集**（正体 compose 后统一扣减），
不是再多一种 SBOX 专用刀。

### 2.2 GTTUBG 的管身其实是「头段套管 + 逐成员目录几何 + 尾段」，且无显式零长阈值

`GTTUBG`（Core3D `0x10340f8e`）复核要点：

- 外径走 `sub_103BF7AC`（CGETOD）：取不到时回落**到达侧** bore（`if (v17) v60 = v57;`），
  与 `route::tube_outer_diameter` 一致。
- 遍历分支成员，对 noun 码 `631901`/`726491`/`640317` 逐个建目录几何（离开侧 stub），
  再在无成员时走**默认头段圆柱（631901）+ 尾段盒（813891）**。这比 Rust 当前「每槽位一根
  圆柱」更细：Core 的管身是**成员的目录 stub 几何**串起来的。
- **没有看到显式 `if (length < eps)` 零长判据**——Core 依赖几何内核（GMCTRA/GMAMEM）
  拒绝退化体。~~故 `route::ZERO_LENGTH_TOL_MM`（现占位 = 0.051mm）是 Rust 侧的安全阀~~
  **（2026-09-01 晚已钉死为 1.0mm）**：`FAB_PipePieceManager::isValidTubeLength`
  （`0x1057be50`）的两道硬门（`fabs(管长) < 1.0`、端点距离 `< 1.0`）即最小管长，
  ams8000 实测砍掉 16 根 0.051~1.0mm 针状假管（built 341→325，账目 +16 落
  `zero_length`）。这道门作用在可视化路上的最终确认仍归 G0-2 RVM 对拍。
  同轮落地：斜接弯头端点分支（`IsMitredBend@0x105773d0` = BEND 且伪属性
  `ACTNCU`≠0，存储侧展开 `NCUTS`/`CPAR[3]` 见 `catalogue_point::is_mitred_bend`；
  端点取构件原点、方向仍走目录 LDIR/ADIR）——ams8000/ams7997 共 3942 只 BEND
  实测 NCUTS 全 0（`examples/mitre_probe`），语料内暂无可对拍实例。

### 2.3 +0.01mm 轴向过切是**圆柱/锥族**的行为，不是全 CRCATI 通用

CRCATI 的圆柱/锥族负臂（SCON/NSCO、SCYL/NSCY、LSNO/NLSN、LCYL/NLCY）都有
`v48 = v48 + 0.01;`——与 `catalogue.rs` 的 `NSCY_AXIAL_OVERCUT_MM = 0.01` 同源。
但 **SEXT/NSEX 臂没有任何过切**（942751/1008005 反编译核实：读高度 → ≥1e-6 → 直接
`sub_104030F0` 挤出，正负同路）。补 NSEX 时**不要**照抄圆柱的 +0.01；共壁负挤出的
让刀策略要看扣减现场（LNEGIT 侧的 restol/nudge），对拍裁决，别先验加量。

---

## 三、差距清单（按优先级，2026-09-01 下午按实测残余重排）

**P0 —— 管道/目录件几何这条线（“继续”的主轴）**

- **G0-1a NSEX：GMRE 成员级负体集（实测 1593 次，最大单项）**。架构先行：正体
  compose 完成后统一扣减成员级负体（对标 Core 的 GMRE CSG 树），NSEX 走与 SEXT 同一
  求值器（环组+高度，§2.1 已核实无 +0.01 过切）；顺带把 NSCY/NSBO/NLCY 从「SBOX 专用
  刀」升级进同一负体集（行为对拍不变则保留现状亦可，以 RVM 裁决）。
- **G0-1b SREV/NSRE：目录回转体（实测 425 次）**。QSSREV 语义已反编译在案（§2.1）：
  PANG 缺省 360、钳 ±360、负角相位 +180、>180° 的双段劈分；`solid::revolve_rings` 与
  设计侧 REV 求值可复用，环组采集与 SEXT 同源（`sub_10715170` 同一个 profile 组装器）。
- **G0-1c 低频缺臂**（本库实测 0 次，等语料再排期）：SDSH/NSDS（QSDISH，翻轴语义在案）、
  SSLC/NSSL（斜切圆柱）、SDIS（QSDISC 圆盘，初版漏记的第 20 臂）。先在 aps 系列库上
  `dump_categories`/GMRE 扫描找实例，有实例才动手。
- **G0-2 管道端到端 RVM 验证**：生成侧基线已刷新到 c13f836 之后
  （evidence/20260901-g02-postfix-baseline）：ams8000 库注册缺口清零、TUBI built
  **553**（acp000 目录解活一批 P 点端）、missing_ppoint 11；ams7997 failed=0、
  HANG 325 条假槽位整体消失。**剩余就差参考侧**：导出 ams8000 管道区 / ams7997
  支撑区 TTY RVM（含 facets，WFxx 同款口径）后 `rvm_compare` 逐件裁决——
  `MDR_BranchVisualisationManager::getCSGTree` 语义的落地验收。本机无可编程
  E3D 会话，导出待人工执行。
- **G0-3 隐式管身对齐 GTTUBG**：核对头/尾 stub 语义（§2.2；TUBE/NTUB/BOXI/NBXI 四个
  stub noun 已定名），确认 Rust「每槽位一圆柱」与 Core「成员 stub 串接」在真库上产物
  一致。~~顺带钉死或文档化 `ZERO_LENGTH_TOL_MM`~~ ✅ 已钉 1.0mm（§2.2 修订注）。
- **G0-4 重做 ams8000 基线**：✅ **已完成**（evidence/20260901-model-reuse）：
  generated 153→**2803**，unknown 只剩 HVAC 2 / TEXT 24，orphans 0，账平。
- **G0-5 目录库注册收口**：✅ 机制已落（`db_discovery` + `--project-dir`，c13f836）。
  实测修正旧说法：跨库目标**不都在 ams000**——7002/7320 在 `AvevaCatalogue\acp000`、
  7600 在 `AvevaPlantSample\aps000`；**db 7000 两个项目各有一份常规命名主文件且内容
  不同**，resolver 按设计拒仲裁，要用 `--catalogue` 显式钉本项目那份。ams8000 库注册
  缺口已清零（残余 failed=2 是 db 7600 会话 12 索引里没有 `23984/105633` 的数据侧
  悬空引用），ams7997 failed=0。见 evidence/20260901-g02-postfix-baseline。

**P1 —— 目录件与复合构件**

- **G1-1 结构型材/目录件**：GENSEC/SCTN/FITT/PFIT/CMFI/FIXING 仍只记账
  （`catalog_pending`，除 SBFI/WALL）。型材要走目录截面求值，与 SEXT 同源——CRCATI 的
  三条 2D 截面臂 **SREC（594640）/SANN（817255）/SPRO（840259）** 已定名，就是这条线的
  目录侧输入（截面 + 沿 SPINE 扫掠），不是实体臂，别排进 G0-1。
- **G1-2 复合成员聚合 builder**：Composite(Structural 58 / CableTray 20 / BuildingAssembly 17
  / PipeComponent 12 / Equipment / Catalogue / CoreClass28)。对标 `sub_107210A0` 成员聚合链
  （逐成员 `LegacyGeom_CreateBuilderForElement` 串 builder）。class 28（NOZZ/ELCONN/EQUCOM）
  在现有语料里全是叶子、无成员可聚，先补有成员的 Structural/CableTray。

**P2 —— 收口项**

- **G2-1 宿主负体机制**：`TMPL` 名下 1559 件负体扣的是容器外宿主（墙/梁），现只记
  `host_negatives`、一刀未扣。要先能从负体定位宿主（`NAPP`？连接关系？core.dll 另有机制？），
  是 `TMPL` 之外的独立课题。
- **G2-2 未实现正基本体**：CONE/DISH/SLCY（+ 负体 NCON/NDIS/NSLC）。「ams8000 有
  CONE×4」出自 07-24 备份快照的旧账；**当前活库（会话 264）实测三者均零出现**
  （evidence/20260901-model-reuse），与 G0-1c 同列「等语料」，别再当对拍 8000 的前置。
- **G2-3 几何集**：GeometrySet/NegGeometrySet（core.dll `CGTCT2 + sub_10714FC0` 另一条建模路），
  现只记账，当前语料无实例，优先级最低。
- **G2-4 分类清扫 + 级联外定性**：以新 ams8000 账为准清扫残余 unknown（HANDRA/IASLCO 扶手线、
  级联外未知 noun），按 `docs/noun-coverage-matrix.md` §9/§11 的口径归并。

**P2' —— 复用线收尾（2026-09-01 审计遗留，独立于几何覆盖）**

- **G2-5 目录求值样板收口**：`model_catalogue_member_with_cache` 的「取网格→放置去重→
  摆放」块已手抄 8 份，抽成一个 `place_cached_part`。**排在 G0-1a/1b 落地之后**做——
  那两条臂还要往这个 match 里加分支，先抽会撞车。
- **G2-6 复用面扩展**：回转体（G0-1b 落地时顺带进 `extrusion_reuse` 同款池）、隐式管身
  （TUBI 3172 根，直径×长度族，`route::build_tube` 侧）、非 BOX/CYLI 基本体的真实尺寸
  记忆化。边界纪律照抄：布尔参与者烘焙直建（枚举顺序案例见 evidence/20260901-model-reuse
  README）。
- **G2-7 挤出实例化元数据透出**：`GeneratedElement.primitive_instance` 只覆盖 BOX/CYLI；
  同形 PANE 480 件在持久层仍是 480 张网格。把挤出池的键+局部变换透出（对标 inst_geo +
  geo_relate.trans），需要与消费端（plant-ui/rs-server）一起定格式。
- **G2-8 对拍配套刷新**：RR001/WF 区 facets 转储不含 SBFI/WALL（新覆盖 noun），
  rvm_compare 退回快照 bbox（已知不可信，最大差 5.4m）判出 49 off——重新导出含
  SBFITTING/WALL 的 facets 再裁决。

---

## 四、建议执行顺序与验收

1. ~~G0-4 先跑~~ ✅ 已完成（evidence/20260901-model-reuse，硬数字：NSEX 1593 / SREV 425 /
   其余 0；252 failed 全是目录库未注册）。
2. ~~G0-5 目录库注册收口~~ ✅ 已完成（c13f836 + 基线刷新，见上）：failed 只剩 2 件
   数据侧悬空引用。
3. **G0-1a NSEX 成员级负体集**（1-2 天）：最大单项；架构对标 GMRE CSG 树，RVM 对拍裁决
   让刀策略（§2.3：不许照抄 +0.01）。
4. **G0-1b SREV 回转臂**（1 天）：QSSREV 语义在案，builder 现成。
5. **G0-2 / G0-3 管道端到端对拍**（核心验收）：生成侧基线已就位
   （evidence/20260901-g02-postfix-baseline），剩 ams8000 管道区 / ams7997 支撑区的
   TTY RVM 导出（人工，E3D 侧）+ `rvm_compare` 逐件裁决。
6. P1 / P2 / P2' 按上表推进；G0-1c 低频臂等语料。

**验收判据（每一步都必须过）**：

- `accounts_for` 硬判据不破（`visited + consumed == 索引总数`）。
- RVM 局部系逐件对拍（`rvm_compare`）：新增几何 100% 配对，偏差落在已证容差内或标注为参考碎片。
- 只读库身份：跑前后 DB SHA-256 一致。
- `cargo test`（含 `noun_coverage` / `increment_real`）全绿；改覆盖面同步更新覆盖矩阵与断言，
  不放宽断言。
- 新补的目录臂：正/负孪生走同一几何路径；+0.01 过切**只对圆柱/锥族**照 CRCATI 带上
  （SEXT/NSEX 臂实测无过切，见 §2.3）。
- 触碰复用池/求值器的改动：两库（ams1112 全量、ams8000+目录库）改动前后
  model.obj / elements.json **逐字节一致**，或在证据里写明预期差异并逐件归因
  （方法见 evidence/20260901-model-reuse/README.md）。

---

## 五、复核用到的锚点（便于下一步直接接手）

- IDB：`Core3D.dll`（idalib-35724）、`core.dll`（idalib-41236/22484）、`libgeom.dll`（idalib-36012），
  ida-bridge 已在线，可直接 `exec ... --sql "SELECT decompile(0x...)"`。
- 关键函数：`GTGEOM@0x10341d2e`、`GTTUBG@0x10340f8e`、`CRCATI/sub_10345ACC`、
  `TUMAT@0x103439c0`、`ORIMAT@core.dll 0x526d674`、`CGETOD/sub_103BF7AC`、
  `QLPYRA/sub_103C07B4`、`QLCYLI/sub_103C0374`、`QSSPHE/sub_103C2480`。
- 本轮新增锚点：`QSSREV/sub_103B81CC`（回转参数）、`QSDISC/sub_103C25E0`（SDIS 圆盘）、
  `QSDISH/sub_103C0CBC`（SDSH 碟形）、`SSLC helper/sub_103C0514`、挤出构造
  `sub_104030F0`、回转构造 `sub_10403110`、GMRE 环组装配器 `sub_10715170`（SEXT/SREV
  共用）、组合装配 `GMAMEM`；case→noun 反查表 `data/noun-family-matrix.json`
  （`noun_hash` 字段，1931 全量）。
- 管道更上层语义：`MDR_BranchVisualisationManager::getCSGTree`（分支可视化 CSG 树）。

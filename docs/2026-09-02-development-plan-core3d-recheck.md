# e3d-model 开发计划（2026-09-02 晚 · Core3D.dll 复核版）

> 承接 `docs/2026-09-01-next-step-development-plan.md`（下称「09-01 计划」）。本轮复核手段：
> 读全部源码 / 证据 / 最新六份 `out/*/report.json` 账目 + 对活体 IDB 用 ida-bridge 反编译交叉验证
> （`Core3D.dll` = `idalib-9944`，`libgm.dll` 用 `exec-idb` 一次性探针）。
> 09-01 计划里已经 ✅ 的项（G0-1a/1b/1c、G0-4、G0-5）不再重述；**本文只写还欠的、
> 以及这一轮被 Core3D 反编译改写了口径的**。
>
> 复核规矩不变：以反编译原文为准，猜不出的写「待核」而不是先验拍板；每条差距都带
> **实测计数（哪个库、多少件）+ Core3D 锚点 + 验收判据**。

---

## 一、账目快照：现在还欠什么（六份最新报告，硬数字）

| 库 / 快照 | generated | catalog_pending（记账不建） | unsupported | 其它 |
|---|---:|---|---|---|
| ams7997（`out/notes-ams7997-full`） | 50678 | FITT 1819 / **SCTN 1762** / FIXING 180 / STWALL 170 / PFIT 57 / CMFI 4 | **DISH 68** | BRAN 830（HVAC 216 + PIPE 614）/ HANG 112；TMPL 17 |
| ams8000+目录库（`out/notes-ams8000-full`） | 2853 | **SCTN 528** / FITT 22 | — | BRAN 560；failed 2（数据侧悬空引用） |
| ams1112（`out/rvm-recheck-ams1112-0902`） | 4627 | FIXING 255 / PFIT 200 / FITT 86 / STWALL 76 / CMFI 3 | — | — |
| aps250110（`out/aps250110-tmpl-final`） | 1541 | **GENSEC 1505** / FIXING 604 / FITT 63 / STWALL 56 / SCTN 40 / PFIT 6 / WALL 2 | DISH 48 / CONE 5 | TMPL 370；**host_negatives NCYL 1282 / NBOX 232 / NXTR 45**；failed 11 |
| aps250209（`out/aps250209-class28`） | 1801 | **GENSEC 2698** / FIXING 140 | CONE 24 / DISH 1 | — |

合计还没建的大头：**SCTN 2330、GENSEC 4203、FITT 1990、FIXING 1179、STWALL 302、PFIT 263**
（结构线 ≈ 1 万件），设计基本体 **DISH 117 / CONE 29**（09-01 计划说的「当前活库零出现」只量了
ams8000，ams7997 / aps 两库都有），模板孔 1559 件（aps250110）。

**工作树（未提交）**：`route.rs` / `pipeline.rs` / `category.rs` / `catalogue.rs` / `elmodl.rs` /
`increment.rs` / `examples/bran_audit.rs` 七个文件——① 风管 BRAN（属主 HVAC）不再派生隐式管身
（`category::derives_implied_tubes` 取代 `is_derived_unit`，见 `evidence/20260902-ams7997-bran-audit` §2：
245 根 4.46 km 假风管，E3D 一根都不画）；② 路由成员「有意不画」的依据 `CatalogueMemberBuild.no_shape`
进 notes。**这是第一件要收的活**（§三 G0-6）。

---

## 二、Core3D 复核的新结论（本轮新增 / 对 09-01 计划的修订）

### 2.1 现代 CSG 路才是 E3D 3.1 画设计元素的主路，旧路是回落

`GTGEOM@0x10341d2e` 的调度顺序（函数头注释已在 IDB 里）：先 `GTGM2_TryModernCSGTree`，**现代
builder 未命中**或全局强制旧路时才进 `I*COM` 级联（`GTGEOM_CreateLegacyGeometryWithTransform@0x10714fc0`）。
现代路入口 `LegacyGeom_CreateBuilderForElement@0x1072c2e0` 按
`LegacyGeom_MapNounToBuilderType@0x107189a0` 把 noun 映到 47 个 builder 型，本轮用
`data/noun-family-matrix.json` 的 `noun_hash` 把 case 码全部反查成 noun（下表）。**这张表就是结构线
（G1）的施工图**：每个 noun 对应哪个 C++ 类、哪个构造函数。

| type | noun | builder 类（构造函数） | 备注 |
|---:|---|---|---|
| 1 / 2 / 3 / 4 | STRU / SUBS / FRMW / SBFR | `sub_10737BE0` / `C20` / `B30` / `B80`（0x70，纯聚合） | 容器：走基类 `DB_Item` 成员链 |
| **5** | **SCTN** | **`DB_Sctn`（`sub_107387F0`，0xE8）** | 型材直段，§2.4 |
| 6 / 8 | PNOD / SNOD | `sub_10737CB0` / `D00` | 节点 |
| 7 / 9 | PJOI / SJOI | `sub_10737F00` / `sub_10738010` | 关节 |
| 10 / 11 / 12 | FITT / CMFI / CMPF | `DB_Fitt`（`sub_1073A170`）/ `CmFitt` / `CmPfit` | 目录件（GMRE 求值，同 SBFI 已有路） |
| 13 | SBFI | `sub_1073B0A0` | ✅ 已建 |
| **14** | **PANE** | `DB_Pane`（`sub_10739990`） | ✅ 已建（环拉伸） |
| 15 / 16 | PFIT / COFI | `DB_Pfit` / `DB_Cofit`（0x78） | 目录件 |
| 17 / 18 / 19 / 20 / 21 | PCOJ / SCOJ / SUBJ / PALJ / SELJ | `sub_10737EA0` / `F50` / `sub_10738060` / `E40` / `FB0` | 关节族 |
| 22 | PLOO（+IECOMP 设备件） | `sub_1072CE20` | 设备族 |
| 23 | INCOMP 负体族 | `sub_1072CED0` | |
| 26 / 27 / 29 / 30 / 47 | SUBE / IPCOMP / IHCOMP / 缺省 | 基类 `sub_107210A0` | **管件 IPCOMP 只有基类聚合，无专门 builder** |
| 28 | NOZZ / ELCONN / EQUCOM | 基类，`v18=9` | class 28（09-01 G1-2） |
| 31 | PLIN | `sub_10739700`（0x78） | |
| 32 / 33 / 34 / 35 | ICCOMP / GMSE / GMSS / NGMS | 基类 | 几何集（G2-3） |
| 36 / 37 / 38 | IGMCOM / IG2COM / INGCOM | `sub_1072CDD0` / `sub_1072CE70` | 几何集成员 |
| **39** | **GENSEC** | **`DB_Gensec`（`sub_10730090`，0xE0）** | 型材 SPINE 扫掠，§2.4 |
| **40** | **SPINE** | `DB_Spine`（`sub_10739450`，0xA0） | GENSEC 的路径数据 |
| 41 | CURVE | `DB_Curve`（`sub_1071F510`，0x1D0） | 弧段 |
| 42 | POINSP | `DB_Poinsp`（`sub_10722EF0`，0x88） | 折点 |
| 43 / 44 / 45 | JLDATU / PLDATU / ENDATU | `sub_10739340` / `92D0` / `93E0` | 端部基准 |
| 46 | FIXING | `DB_Fixing`（`sub_1073A280`） | 目录件 + 模板孔宿主（§2.6） |

`CSG_PrimitiveUtilities::initialise@0x10727540` 另注册 **设计基本体 13 族**（BOX/CYL/CON/PYR/SNO/DIS/CTO/
RTO/POL/SLC/EXT/REV + 负孪生，`CSG_Basic*::getPrimGeom@0x10726a90…0x10727450`）与 **`CSG_TreeBuilderCat`
挂在 NOZZ / SUBCOM / HVBRCO / HVFLAN / HVHACC / HVSADD / HVSPLR / HVSTIF / HVTPPO / HVSKIR / HVIDAM**（HVAC 目录件）。

### 2.2 目录件 CSG 树：负体只扣「自己的正体」，顶层负体**不做差集**（改写 09-01 §四.3 的假设）

`CSG_TreeBuilderCat::getCSGTree@0x1072f5d0`（逐行核过）：

```
top = gm_CreateCombination(opt.byte31 ? 0 : 5)          // 5 = GM_ComplexCombination
for M in GMRE.members where isWanted(M):
    shape = CRCATI(M)                                     // sub_10345ACC，正/负 noun 同臂同形
    if M.firstMember == null || !opt.byte28:
        gm_AddMember(shape, T(M), top)                    // 直接聚合
    else:
        sub = gm_CreateCombination(opt.byte31 ? 0 : 3)    // 3 = GM_DIFFERENCE
        gm_AddMember(sub, I, top); gm_AddMember(shape, T(M), sub)
        addNegatives(sub, M)                              // 0x1072f480：M 的成员逐个 CRCATI 后加进 sub
```

`libgm.dll` 一次性探针核实 `GMCCOM/gm_CreateCombination`：**1/2/3 → `GM_SolidCombination`
（GM_UNION / GM_INTERSECTION / GM_DIFFERENCE，字符串表顺序），5 → `GM_ComplexCombination`，0/4 →
`GM_AggregateCombination`**；`GM_ComplexCombination::calcFacets@libgm 0x10050310` 只是逐操作数
`getOperandFacets` 后 `GM_CompFacets::aggregateWith`——**聚合，不做布尔**。

推论：**挂在 GMRE 顶层、不在任何正体名下的负体（ams7997 那 8 件 NSRT），现代 Cat 路把它当普通成员
聚合进 Complex(5)，既不扣兄弟也不扣装配**——要么以正形显示（若 `isWanted`），要么被表现旗过滤。
09-01 计划「负体扣装配而非扣兄弟」是猜的，不成立。管件（IPCOMP）没有现代 Cat 插件，走的是旧路
`GTGEOM_CreateLegacyGeometryWithTransform` → 那条路顶层负体怎么处理还要核 core.dll 的 `GTCATG`/`LNEGIT`
（§三 G0-7 第一步）。设计侧独立负体（NBOX/NCYL/…无正体宿主）走 `CSG_TreeBuilderNegativePrimitive@0x10726770`
→ `addStandAloneNegative@0x10726620` = `Combination(3)[Null, neg]`，**结果为空**——与我们「一刀不扣、也不画」一致。

### 2.3 修正：`MDR_BranchVisualisationManager` 不是管道 BRAN 的

`MDR_Manager::initialise@0x105d0f50` 把它注册给 **`RBRAN`**（`setBranchElement(NOUN_RBRAN)`，配 `RATTA` / `RSEG` /
`HATTA` / `TATTA`），`CAB_MDR_CableManager::initialise@0x109b61a0` 注册给 **`CABLE`**。管道 `BRAN` / `TUBI`
**没有任何现代插件**（全部 `addPlug` 调用点都列过：`MDR_CSGUtility` / `CSG_PrimitiveUtilities` / 结构辅助线 /
ACC / ASL / UDET 样例），**管身语义仍然只在旧路 `GTTUBG@0x10340f8e`**——09-01 计划 §五「管道更上层语义：
MDR_BranchVisualisationManager」删掉。`DB_RoutingBranch::createPathSegment@0x105B3B60`
（extrusion-group / REVO / RULED + mitre extension）是**路由线（RBRAN/CABLE）**的管身建法，桥架路由与电缆
以后能用，管道对拍（G0-2）别拿它当参照。

### 2.4 结构线（G1-1）的 Core3D 蓝图（本轮反编译落地的部分）

- **SCTN：`DB_Sctn::DB_makeCSG@0x10738C30`**（vtable 槽 1，含 `"DB_Sctn::DB_makeCSG"` 版本串）：
  ```
  profile   = vtbl[2](this, T, 0)                        // 截面沿 POSS→POSE 拉伸的实体
  xsec1     = cutPlane(profile, 平面A, 1); cutPlane(xsec1, 平面B, 0)   // 两端切平面，helper vtbl+112
  endPrep   = sub_10739770(MEML-ref builder)             // 777262=MEML 引用件，逆变换后 vtbl+76 组合
  joints    = sub_10722D80(JOIS builder) / (JOIE builder) // 912394=JOIS / 636832=JOIE；属主是 SUBJ(730315) 再加一层
  negatives = sub_10721600(mode 3)                       // byte_10D99C00 开时
  template  = owner==TMPL(779672) → sub_10722D80(mode 7)；TMPL.owner==FIXING(108608856) → sub_10721930
  ```
  构造函数 `sub_107387F0`：长度 `= |POSE(644128) − POSS(919690)|` 存 +128；两端切平面与标架来自
  **`dbboxlib/GTSTDT@0x1036CEC4`**（核到它按 noun 分路：SCTN 821683 / GENSEC 46619692 / PANE 640105 /
  WLJOIN 206501990，沿 PFIT 931840 / CMPF 661557 链上溯）。这是 G1-1a 要逐行翻的 Fortran 段。
- **GENSEC：`DB_Gensec::DB_makeCSG@0x107311D0`**（`"DB_Scan_to_GML::memberLines()"`）：构造函数 `sub_10730090`
  读 `LMIRR`(10458597) 翻转旗、`BANG`(679457)×0.01 存 +208，找 SPINE(3471220) 成员，成员按
  **POINSP(240493876) / CURVE(3635364) 交替**（诊断串 `"No POINSP between CURVEs"` / `"No SPINE for GENSEC"`）。
  段建法 **`sub_107318E0`**：截面 `D2_Profile` 按 BANG `rotateBy` → 直段 `gm_CreateExtrusion`（`" EXTR segment"`）
  / 弧段 `gm_CreateRevolution(0, r, axis, 180?, prof)`（`" REVO segment"`，`sweepAngle`/`pAxis` 打印）
  / 两端截面不同 `gm_CreateRuledSolid`（`" RULED segment"`）+ **首末斜接延伸**（`" Start-mitre extension"` /
  `" End-mitre extension"`，用 POINSP 的 `posFrom/posTo/vec1/vec2`）；LINE(640317) 特判。
  `STRU_GensecPline::Geometry@0x106274f0` 把 SPINE 变成 `vector<pair<mth3Line*, mth3Arc*>>`。
- **截面来源：`getProfileFromDB@0x10410b30 → sub_10410BB0`**（`"profileExtractFromDB/getProfileFromDB"`，
  `validLoop`，`D2_Profile::addSpan(bulge, point)`）——把目录截面（`SPRO` 的 PLOO/PAVE 环含 bulge、`SREC`/`SANN`
  参数臂）读成 `D2_Profile`。CRCATI 的三条 2D 臂 `SREC 594640 / SANN 817255 / SPRO 840259` 就是它的输入
  （09-01 §2.1 已定名）。
- **两端切面的 libgm 原语**：`gm_CreateSection(tree, GM_Operation, GM_CutType)` 由 `GMCXSE@0x104031b0` 包装，
  结构侧调用点 `sub_10735EF0`（gm_CreateSection + gm_CreateExtrusion + gm_CreateProfile）/ `sub_1072A570`
  / `sub_1072BA70`（`gm_CreateCutSurface` + `gm_AddCutProfilePoint`）。manifold-csg 侧对应半空间裁切。

### 2.5 复合聚合 builder（G1-2）的真身

`sub_107210A0` 是 **`DB_Item` 基类构造函数**：读 TYPE(642215) → `MapNounToBuilderType` → 遍历成员
（`sub_10716FC0(…,1)` 首 / `(…,0)` 次）逐个 `LegacyGeom_CreateBuilderForElement` 串成 `+12`（首子）/ `+16`（兄弟）
链；`TMPL`(779672) 属主特判（`+2` 指向模板）。**所有 0x70 大小的「纯聚合」builder（STRU/SUBS/FRMW/SBFR/
NOZZ/EQUCOM/IPCOMP…）就是这条链 + 缺省 `DB_makeCSG`**。Rust 侧 `Composite(_)` 的 builder 就按这个做：
成员各自建、按 `isWanted` 过滤、Complex(5) 聚合，不做布尔。

### 2.6 显示规则 / 模板孔（与已有结论一致，锚点补齐）

- `CSG_TreeBuilderOptions::isWanted@0x10716070`（私有重载）三模式：`mode 0` 只看 TUFL；`mode 1` 看 OBST(1/2)
  两位旗；`mode 2/3` LEVE 区间 + CLFL 分支（`byte12` 画 CL、`byte13` 再看 TUFL）。与 09-01 的
  `TUFL=false 一律不画` / DRWANT 口径一致。公开重载 `@0x107162c0` 在有外部回调时把
  `LEVE/OBST/CLFL/TUFL/noun hash` 交给回调（Design 的 draw-list 规则），不在 Core3D 内。
- 模板孔：`CSG_TreeBuilderPrimitive::getCSGTree@0x10726890` 建正体后 `addHolesBelowPrimitive`（空实现）→
  `owner==TMPL` → `addHolesBelowTemplate(…, mask 4)`；`TMPL.owner==FIXING` 时用 `DB_Iterator` 扫该 FIXING 下
  **全部 TMPL**。与 `evidence/20260902-template-holes-napp` 一致。

---

## 三、差距清单（按优先级；每条带计数 / 锚点 / 验收）

### P0 —— 先收口、再对拍

- ~~**G0-6 风管 BRAN 不派生管身 + `no_shape` 依据入库（工作树已改，未提交）**~~
  ✅ **已由并行会话入库**（`8d52639` no_shape 记账、`fae9e7e` HVAC 不派生管身、`41a550b` 表现级按学科拆分）。
  本会话独立复跑 `bran_audit` 核实：ams7997 HVAC **bad 123 → 0、built 245 → 0**（not_derived 2398）、PIPE 614 条
  逐位不变（built 4787 / 3323 m）；ams8000 PIPE 558 条逐位不变、HVAC 2 条 zero_length 6 → not_derived 6。
  **注意**：`41a550b` 往 `category.rs` 写进 `"IASLCO"/"IFICOM"/"IJOCOM"/"IPLCOM"/"IPRCOM"/"LEVEL"` 六个字面量，
  `tests/noun_coverage.rs::every_noun_named_in_the_classifier_actually_exists_in_the_dictionary` 在 HEAD 是红的
  （源码扫描把非级联家族名当成查无 noun）——要么白名单 `nonCascadeFamilies`，要么把家族名单挪出 `category.rs`。
- **G0-2 管道端到端 RVM 对拍**（不变）：参考侧 ams8000 管道区 / ams7997 支撑区 TTY RVM 导出待人工；
  生成侧留给 RVM 的三件（ATTA 处 TUBE 连续性、74+99 根反向管画不画、托架路由 r=12.5 细管）不变。
  **参照口径修正**：拿 `GTTUBG` 旧路对，不拿 `MDR_BranchVisualisationManager`（§2.3）。
- **G0-3 成员 stub 三 case**（`GTTUBG :137–:201`：`631901 TUBE → sub_103BF5C0 + sub_10402E10(半长, r) + GMCTRA 居中`
  / `726491 BOXI → sub_103BF6D8 + GMCBOX` / `640317 LINE → sub_10402D00 + GFX_Box::getType`）——
  语料里 ams8000/7997 目录程序是否出现 TUBE/BOXI stub 成员先普查（`examples/catalogue_probe`），出现再对拍。
- **G0-7 顶层负体 NSRT（ams7997 8 件）定性**：① 先查这 8 件的属主学科（IPCOMP 走旧路还是 SUBCOM/HVAC
  走现代 Cat 路）；② 现代路结论已定（§2.2：聚合不扣）；旧路核 core.dll `GTCATG`/`LNEGIT` 对顶层负 noun 的
  处理；③ 请人工在 E3D 里对这 8 件 `REPR` 一次看画不画。**没拍板前保持现状（不画、记账）。**

### P1 —— 结构线（最大的账，≈ 1 万件）

- **G1-1a SCTN 直段（2330 件：ams7997 1762 / ams8000 528 / aps 40）**
  按 §2.4 分四步，每步可单独验收：
  1. 截面：`SPRE → SPCO → CATR → 截面目录件` 求 `D2_Profile`（SPRO 的 PLOO/PAVE 环 + bulge、SREC/SANN 参数臂；
     GMRE 表达式仍经 e3d-io `DbElementParamEnv`）。锚点 `sub_10410BB0`。
  2. 直段实体：截面沿 `POSS→POSE` 拉伸、按 `BANG` 旋、`LMIRR` 镜像；标架用 **`GTSTDT@0x1036CEC4`** 的口径
     （别再用 ORIMAT 猜——SSLC 那次教训）。
  3. 两端切平面 `CUTB/CUTE`（GTSTDT 出的两组点+法向）→ manifold 半空间裁切；对应 `gm_CreateSection`。
  4. 关节 JOIS/JOIE、MEML 端部件、TMPL 孔：按 `DB_makeCSG` 的顺序追加，先只记账后补。
  验收：ams8000 `SCTN 528 → 0 pending`；RVM 局部系逐面对拍（WFxx 同款口径）新增几何 100% 配对；
  `accounts_for` 不破；两库改动前后其它 noun 的 `model.obj` 逐字节一致。
- **G1-1b GENSEC 扫掠（4203 件：aps250110 1505 / aps250209 2698）**
  SPINE 成员 POINSP/CURVE 交替 → 直段 EXTR / 弧段 REVO / 变截面 RULED + 首末斜接延伸（`sub_107318E0`）；
  `STRU_GensecPline::Geometry` 的线弧分段先在 `examples/spine_probe` 上对齐。**依赖 G1-1a 第 1 步的截面求值。**
  验收：aps250209 `GENSEC 2698 → 0 pending`，扶手 `aps250110-handrail` RVM 逐件对拍。
- **G1-1c FITT / PFIT / CMFI / FIXING / STWALL（FITT 1990 / FIXING 1179 / STWALL 302 / PFIT 263 / CMFI 7）**
  FITT/PFIT/CMFI/FIXING 是目录件（`DB_Fitt` 等 = GMRE 求值），与 SBFI 同一条 `model_catalogue_member_with_cache`
  路——**先把 G2-5 的 `place_cached_part` 收口做掉再接**，否则又是第 9 份手抄。STWALL **不在 47 型表里、
  也没有现代插件**（`ACC_MDR_CSGUtility::initialise@0x10a00520` 只挂 CTWALL / BPANEL / WLJOIN / WLPROF / WLPANE），
  与 WALL 同走旧路 `GTSTDT`——先核它与 WALL 的差别是不是只有 noun，是则直接复用 `wall_local_solid`。
  FIXING 同时是模板孔宿主（§2.6）。
  验收：五库 `catalog_pending` 只剩明确「无 GMRE / 表现过滤」的记账；ams1112 RR001/WF 区 facets 重导出
  含 FIXING/PFIT 后 `rvm_compare` 逐件裁决（G2-8 一并收）。
- **G1-2 复合聚合 builder**：按 §2.5——成员各建、`isWanted` 过滤、Complex(5) 聚合、不做布尔；
  class 28（NOZZ/ELCONN/EQUCOM）语料里全是叶子，先补 Structural / CableTray / BuildingAssembly 三族。

### P2 —— 收口项

- ~~**G2-2 设计基本体 CONE / DISH / SLCY（+ NCON / NDIS / NSLC）——解封，且便宜**
  计数：DISH 117（ams7997 68 / aps250110 48 / aps250209 1）、CONE 29（aps 5+24）。09-01 说「零出现」只量了 ams8000。~~
  ✅ **已完成（2026-09-02 晚，`evidence/20260902-g22-dish-slcy-design-primitives/`）**：CONE 其实早已接线（aps 两份
  报告是老快照）；DISH/SLCY + NDIS/NSLC + ADISH/ASLCY 按 `CSG_BasicDIS@0x10726d10` / `CSG_BasicSLC@0x107272d0`
  接进 `Prim::Dish` / `Prim::SlopedCylinder`。ams7997 68 只 DISH 对 TTY RVM **68/68 配对、65 件 ≤ 1 mm**；
  DISH 局部帧 = libgm（底圆 XY、顶 +Z，与 CYLI 的 Y 轴约定不同，RVM 逐列坐实）。顺带定下一条规则：**带负体的
  正体与负体同用 `negative_cylinder_segments` 网格**（E3D FacetGroup 实测 28 段），否则 R=1330 半球被 r=1335
  的 NCYL 切出 28 片刀缝碎片。遗留：SLCY 无语料未经 RVM 验证；`1RCS002BA` 两只 r≠h 折边碟与 RVM 半椭球图元差
  36 mm 是参考侧口径不够（RVM `EllipticalDish` 不存折边）。覆盖基线 built 33 → 39 / ledgered 294 → 288。
- **G2-1 模板孔 `NAPP & 4`**（aps250110 host_negatives NCYL 1282 / NBOX 232 / NXTR 45）：按
  `evidence/20260902-template-holes-napp` 的定案（只扣 `HOLC ∧ NAPP>0 ∧ NAPP&4 ∧ isWanted`，`NAPP=-1` 忠实不扣），
  账目改 `template_holes_applied / template_hole_definitions`。FIXING 下多 TMPL 的扫法照 §2.6。
- **G2-3 几何集**（GMSE/GMSS/NGMS，type 33/34/35 走基类聚合；成员 IGMCOM/IG2COM/INGCOM 走 `sub_1072CDD0/CE70`）：
  语料无实例，最低。
- **G2-4 分类清扫**：以 §一 的账为准。扶手线（HANDRA / RAIL / KICKPL / PLTFRM / STRLNG）在 Core3D 有专门插件
  （`ASL_MDR_CSGUtility::initialise@0x109f9d40` → `ASL_MDR_Handrail/Rail/Kickplate/PlatformVisualisationManager`），
  `aps250110-handrail` 那批要对拍时从这里进，不归 GENSEC。
- **G2-5 `place_cached_part` 收口**（8 份手抄 → 1）：**提到 G1-1c 之前**。
- **G2-6 / G2-7 / G2-8** 照 09-01 计划（复用面扩展 / 挤出实例化元数据 / 对拍配套刷新），G2-8 与 G1-1c 合并做。

---

## 四、建议执行顺序与里程碑

1. ~~**M0（半天）**：G0-6 入库（七文件一次提交 + evidence README 追记）~~ ✅（并行会话 `fae9e7e`/`8d52639`）；
   G0-7 ①③ 发出人工请求（8 件 NSRT 属主 + E3D REPR）——**待发**。
2. **M1（1 天）**：~~G2-2 CONE/DISH/SLCY~~ ✅（2026-09-02 晚）+ G2-5 `place_cached_part` 收口（**待做**；
   `catalogue.rs` 此刻有并行会话未提交改动，等它落地再动）。
3. **M2（2–3 天）**：G1-1a SCTN 四步；第 1 步（截面求值）单独验收，因为 G1-1b/1c 都吃它。
4. **M3（2 天）**：G1-1b GENSEC；同时 G1-1c FITT/PFIT/CMFI/FIXING 接进目录路。
5. **M4**：G1-2 复合聚合 → G2-1 模板孔 → G2-8 对拍刷新。
6. **贯穿**：G0-2 参考侧 RVM 一到就裁决；G0-3 stub 普查随 M2 的 catalogue_probe 顺手做。

**验收判据（每步都过，沿用 09-01 §四）**：`accounts_for` 硬判据不破；RVM 局部系逐件对拍新增几何 100% 配对、
偏差在已证容差内；只读库身份（DB SHA-256）前后一致；`cargo test` 全绿且不放宽断言；触碰复用池 / 求值器的改动
两库 `model.obj` / `elements.json` 逐字节一致或逐件归因；**新补的每条臂都先贴 Core3D 反编译锚点，再写代码**。

---

## 五、请拍板的几件事（标注在这里即可）

1. **G0-6 直接入库**（七文件一次提交），还是拆成「HVAC 不派生管身」与「no_shape 记账」两提交？
2. **G1 结构线的顺序**：SCTN 先（ams8000 有 RVM 参考、528 件）还是 GENSEC 先（aps 两库 4203 件、量最大但无 RVM）？
   本文按 SCTN 先排。
3. **G0-7 顶层 NSRT**：接受「现代路聚合不扣」的结论、维持现状记账，还是要我继续翻旧路 `GTCATG/LNEGIT`？
4. **G2-2 提前到 M1**（便宜、解封）是否同意？
5. 参考侧 RVM 导出（ams8000 管道区 / ams7997 支撑区 / ams1112 含 FIXING·PFIT 的 RR001·WF 区）能否本周排上人工？

---

## 六、本轮锚点表（便于直接接手）

| 用途 | 地址（Core3D.dll，除标注外） |
|---|---|
| 总调度 / 现代路入口 | `GTGEOM@0x10341d2e`、`LegacyGeom_CreateBuilderForElement@0x1072c2e0`、`LegacyGeom_MapNounToBuilderType@0x107189a0` |
| 基类 builder（成员链） | `DB_Item` ctor `sub_107210A0`；vtable `0x10b39eb4`（槽 1 `sub_107223D0` = 缺省 makeCSG，槽 5 `sub_10721F00` 成员几何） |
| SCTN | ctor `sub_107387F0`，`DB_makeCSG@0x10738C30`，端部件 `sub_10739770`，关节/模板挂接 `sub_10722D80` / `sub_10721930`，负体 `sub_10721600` |
| GENSEC / SPINE | ctor `sub_10730090`，`DB_makeCSG@0x107311D0`，段建法 `sub_107318E0`，`DB_Spine` ctor `sub_10739450`，`STRU_GensecPline::Geometry@0x106274f0` |
| 结构数据 / 切面 | `dbboxlib/GTSTDT@0x1036CEC4`，`sub_10717DA0`（把 GTSTDT 四组向量落到 builder），`GMCXSE@0x104031b0`→`gm_CreateSection` |
| 截面 | `getProfileFromDB@0x10410b30` → `sub_10410BB0`；CRCATI 2D 臂 `SREC 594640 / SANN 817255 / SPRO 840259` |
| 目录 CSG 树 | `CSG_TreeBuilderCat::getCSGTree@0x1072f5d0`、`addNegatives@0x1072f480`、`CRCATI/sub_10345ACC` |
| 设计基本体 | `CSG_PrimitiveUtilities::initialise@0x10727540`（注册表）、`CSG_Basic*::getPrimGeom@0x10726a90…0x10727450`、`CSG_TreeBuilderPrimitive::getCSGTree@0x10726890`、`NegativePrimitive@0x10726770`、`addStandAloneNegative@0x10726620`、`addHolesBelowTemplate@0x107263A0` |
| 显示规则 | `CSG_TreeBuilderOptions::isWanted@0x10716070`（私有）/ `@0x107162c0`（公开） |
| 路由线（非管道） | `MDR_Manager::initialise@0x105d0f50`（RBRAN）、`CAB_MDR_CableManager::initialise@0x109b61a0`（CABLE）、`MDR_BranchVisualisationManager::getCSGTree@0x105e9aa0`、`DB_RoutingBranch::createPathSegment@0x105B3B60` |
| 管身旧路 | `GTTUBG@0x10340f8e`（stub 三 case `:137–:201`）、`CGETOD/sub_103BF7AC` |
| libgm.dll | `gm_CreateCombination`（1/2/3 Solid=UNION/INTERSECTION/DIFFERENCE，5 Complex，0/4 Aggregate）、`GM_ComplexCombination::calcFacets@0x10050310` |
| 属性 / noun 码 | POSS 919690 · POSE 644128 · JOIS 912394 · JOIE 636832 · MEML 777262 · LMIRR 10458597 · BANG 679457 · TYPE 642215 · OWNER 10206636；TMPL 779672 · FIXING 108608856 · SUBJ 730315 · SPINE 3471220 · POINSP 240493876 · CURVE 3635364 · WLJOIN 206501990 |

> 运维备注：本轮把 `idalib-23568` 弄挂了一次（exec 环境里 `from ida_bridge import idb` 会覆盖运行时的 `idb` 全局，
> worker 循环随即崩掉）；已重启为 `idalib-9944`。**exec 代码里不要给 `idb` 这个名字赋值。**

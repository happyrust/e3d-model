# e3d-model 开发计划（2026-09-05 · Core3D.dll 复核第四版：builder 注册表普查）

> 承接 `docs/2026-09-04-development-plan-core3d-recheck-3.md`（下称 recheck-3）、
> `docs/2026-09-04-generation-algorithm.md`（总纲）、`docs/2026-09-04-step1-in-memory-plan.md`（第一步计划）。
>
> **前三版都是「逐条深挖」：拿到一个 noun，去反编译里找它的臂，核对我们写的对不对。**
> 这一版换了个方向——**先把 E3D 到底为哪些 noun 建几何这件事整张表数出来**，再拿它减去我们已经建的。
> 这不是又一批细节勘误，而是把总纲 §4 的 **D3 闸门**（「分清『E3D 也不画』与『我们还没建』」）
> 从一句判断变成一个**可机械求值的差集**。
>
> 本轮手段：ida-bridge 对活体 `Core3D.dll`（`idalib-49136`，原 `idalib-25128`，本轮中途重启过）
> 全映像枚举 `CSG_TreeBuilder::addPlug` 与 `DB_ModelBuilder::addPlug` 的**每一个调用点**，
> 逐点回溯取 `NOUN_*` 全局与 builder 虚表；再解出旧路 `CRDESI@0x10343b80` 的**全部 switch 臂**。
> 原始产物落在 `evidence/20260905-core3d-csg-registry/`（含复现脚本，可逐条重跑）。
>
> 复核规矩不变：以反编译原文为准，猜不出的写「待核」；每条差距带**锚点 + 验收判据**。

---

## 一、本轮最重要的一条：注册表比 recheck-3 记的大一个数量级

recheck-3 §八 的锚点表写「`addPlug@CSG_TreeBuilder@0x107158B0`，**13 个注册点**」。
全映像枚举的实数是 **80 个调用点**，分布在 **13 个注册函数**里——recheck-3 数的是**函数**，
不是注册点。这个差别不是记账口径问题：13 个注册点会让人以为现代 CSG 路只覆盖基本体，
于是「其余都走目录 / 旧路」；**80 个注册点覆盖了 65 个具名 noun，跨 8 个学科**，
其中整整四族是我们连分类都没认领的。

| 注册函数 | 点数 | 覆盖的族 |
|---|---:|---|
| `CSG_PrimitiveUtilities::initialise@0x10727540` | 19 | BOX/CYLI/CONE/PYRA + 四个负体 + NOZZ/SUBCOM + HVAC 目录九件 |
| `sub_109B55E0` | 11 | 电缆桥架件 + 路由节点（`CTSTRA…CTREDU`、`RNODE`、`CNODE`、`POINTR`、`CWBRAN`） |
| `sub_108F54F0` | 8 | 舱室装修（`FLRLAY`/`CLNTIL`/`BNDLIN`/`INSURQ` + 四个 `*FITT`） |
| `sub_1062CCE0` | 9 | 结构标注与辅助线（`AID*`/`LINDIM`/`MLABEL`/`STWELD`/`REFGLN`/`REFGAR`） |
| `MDR_CSGUtility::initialise@0x105E9DB0` | 6 | 分支 / 段 / 接管点（`HATTA`/`TATTA` + 4 个待核） |
| `ASL_MDR_CSGUtility::initialise` | 5 | 通道钢结构（`HANDRA`/`RAIL`/`KICKPL`/`PLTFRM`/`STRLNG`） |
| `ACC_MDR_CSGUtility::initialise` | 5 | 舱壁（`CTWALL`/`BPANEL`/`WLJOIN`/`WLPROF`/`WLPANE`） |
| `sub_108F89A0` | 4 | 梯 / 楼梯（`STRFLT`/`RLADDR`/`SLADDR`/`HRPOST`） |
| `sub_10728EC0` / `sub_107293D0` | 8 | `CSG_TreeBuilderMyBox` / `Cat`，**noun 由入参传入，待核** |
| `xba_base::Init` | 3 | 地形与通用图元（`TRNCUR`/`GENPRI`/`GENCUR`） |
| `sub_109B61A0` | 1 | `CABLE` |
| `DB_ModelBuilder::addPlug@0x108E79D0` | 1 | `DTPROF`（唯一调用者 `sub_1050B1D0`） |

**65 个 noun 由 `NOUN_*` 具名全局直接读出**，`BNDLIN` / `INSURQ` 两个由同函数内
`DB_AttPluggerHelper::addPlug(ATT_BNDARE, NOUN_BNDLIN, …)` 与 builder 类名（`CSG_TreeBuilderBNDLIN` /
`CSG_TreeBuilderINSURQ`）互证，**12 个调用点的 noun 仍是入参、待核**
（另 1 点是 `DB_ModelBuilder::addPlug` 转发进来的，不是独立注册）。

**每一个 builder 都不是桩。** 逐个读虚表第 0 槽（`getCSGTree(DB_Element const&, CSG_TreeBuilderOptions const&, D3_Transform&)`）
的函数体大小，最小 139 字节、最大 2261 字节，没有一个是 `return 0`：

```
CAB_CSGRouteNode::getCSGTree        0x109CB150  1909 bytes   ← RNODE
CAB_CSGControlNode::getCSGTree      0x109CABB0  1438 bytes   ← CNODE
ASL_CSGLadderVisualisationManager   0x1092D040  2261 bytes   ← RLADDR
CSG_TreeBuilderCLNTIL::getCSGTree   0x1090D0D0  1697 bytes   ← CLNTIL
CSG_TreeBuilderBNDLIN::getCSGTree   0x1090CC20  1187 bytes   ← BNDLIN
ASL_CSGStepLadderVisualisationMgr   0x1092D920  1088 bytes   ← SLADDR
MDR_BPanelVisualisationManager      0x109FFF10   792 bytes   ← BPANEL
ASL_CSGMultiflightStairsVisMgr      0x10922700   741 bytes   ← STRFLT
… 全表见 evidence/20260905-core3d-csg-registry/builder-vtable-slot0.txt
```

→ **「E3D 为这个 noun 建几何」这件事，现在有名有姓有字节数。**
这正是 D3 闸门缺的那半边判据。

---

## 二、旧路 `CRDESI@0x10343b80` 的全部 41 条臂（首次数全）

switch 的 41 个 case 立即数逐个反哈希（`hash = 27⁴ + Σ(ord(cᵢ)−64)·27ⁱ`）：

```
CYLI NCYL  DISH NDIS  CONE NCON  SNOU NSNO  BOX NBOX   PVOL GRDM  POHE POGO
PYRA NPYR  CTOR NCTO  RTOR NRTO  SLCY NSLC  RPLA DATU  BOUN DRAW  GRIDLN
PNOD SNOD  IPOI TPOI POIN   EXTR NXTR  RULS NRUL  PLOO  REVO NREV  POLYHE NPOLYH
```

对上 `classify()` 逐条比，**旧路有臂、我们连 noun 都不认识的只剩一对**：

- **`RULS`(914751) / `NRUL`(783446)** —— 直纹体（ruled solid）。共用一条臂，
  `DGETR(&unk_10B4DFE4, …)` 读一个实属性后调 `sub_10715170()` 造 builder，
  与 recheck-3 §2.6 勘误里认出的 **`gml_bkend/CRRSLD`（Create Ruled Solid）** 对上。
  这两个 noun **既不在 `category::classify()` 里，也不在 `data/noun-family-matrix.json` 里**
  （census 打印 `(not in census)`）——不是「判错了」，是**整族不存在**。

其余 39 条臂全部已认领或已点名记账（`PNOD`/`SNOD` 判 `NonGraphic`、`POIN`/`PLOO`/`POGO` 判
`ProfileData`、`PVOL`/`GRDM`/`RPLA`/`DATU`/`BOUN`/`DRAW`/`GRIDLN`/`IPOI`/`TPOI` 判 `Unknown`）。
**注意 `PNOD`/`SNOD` 判 `NonGraphic` 与旧路有臂是冲突的**，与下面 §3.1 的 `RNODE`/`CNODE` 同型，
一并进 W1 的复核清单。

---

## 三、差集：注册表有、我们没有

下表左列是 Core3D **有具名 builder** 的 noun，右列是本仓库 `category::classify()` 当前判据
（`Unknown` = 落到 `dictionary_composite_category` 兜底；判据来自 HEAD `c9443ee` + 工作树源码，
件数来自 `out/noun-categories.json` 这份**旧 census，M0 复跑后要刷新**）。

### 3.1 判 `NonGraphic`，但 Core3D 有上千字节的 builder（**最高优先级：判据反了**）

| noun | Core3D builder | 字节 | 我们判 |
|---|---|---:|---|
| `RNODE` | `CAB_CSGRouteNode` | 1909 | `NonGraphic`（`category.rs:413` 硬编码名单） |
| `CNODE` | `CAB_CSGControlNode` | 1438 | `NonGraphic`（同上） |
| `PNOD` / `SNOD` | 旧路 `CRDESI` case 621502 / 621505 | —— | `NonGraphic`（同上） |
| `POINTR` | `CAB_CSGRoutePoint` | 311 | `Unknown`（`aps250209` 单库 352 件） |

> **W2 探针已把这张表读细了一层，结论见 §五 W2**：`PNOD`/`SNOD` 判对（`gm_CreateMark`，
> 点标记）；`RNODE`/`CNODE` 只反了一半（有 `SPRE` 出实体、无 `SPRE` 画线框）；
> `POINTR` 那 352 件是扶手扫掠路径的载体，属 W4-2 而不是独立接线项。

`NonGraphic` 在本仓库的语义是「**E3D 也不画**」——它是 D3 闸门里那本「有名有姓且不欠账」的账。
把一个有 1909 字节 builder 的 noun 记进这本账，后果不是报错，是**这批件永远不会出现在任何欠账数里**：
`accounts_for` 照平，`not_wired` 照样是零，报告全绿。这是总纲 §六 红线 1「静默缺件」的最纯形态，
而且是我们**主动写进名单**造成的，不是漏掉的。

### 3.2 整族缺失：通道钢结构（ASL）

| noun | Core3D builder | 字节 | 我们判 |
|---|---|---:|---|
| `HANDRA` | `ASL_MDR_HandrailVisualisationManager` | 495 | **`Unknown`** |
| `RAIL` | `ASL_MDR_RailVisualisationManager` | 297 | **`Unknown`** |
| `KICKPL` | `ASL_MDR_KickplateVisualisationManager` | 297 | **`Unknown`** |
| `STRLNG` | `ASL_MDR_PlatformVisualisationManager` | 589 | **`Unknown`** |
| `PLTFRM` | `ASL_MDR_PlatformVisualisationManager` | 589 | `Composite(Structural)` |
| `STRFLT` | `ASL_CSGMultiflightStairsVisualisationManager` | 741 | `Composite(Structural)` |
| `RLADDR` | `ASL_CSGLadderVisualisationManager` | 2261 | `Composite(Structural)` |
| `SLADDR` | `ASL_CSGStepLadderVisualisationManager` | 1088 | `Composite(Structural)` |
| `HRPOST` | `MDR_HRPostVisualisationManager` | 577 | `Composite(Structural)` |

`docs/noun-coverage-matrix.md` 已经把 `HANDRA` 标成「**有几何、走别的线（真欠账）**」，
但依据是 `IASLCO` 名单与 `sub_10089BDC` 的调用者名单——**间接**。现在是直接证据：
九个 noun，九个具名 builder，最大的一个 2261 字节。`aps250209` 单库 `HANDRA` 81 件。
四个判 `Unknown` 的（`HANDRA`/`RAIL`/`KICKPL`/`STRLNG`）连账本都没进对。

### 3.3 整族缺失：舱室装修（Accommodation）

| noun | Core3D builder | 字节 | 我们判 |
|---|---|---:|---|
| `CLNTIL` | `CSG_TreeBuilderCLNTIL` | 1697 | **`Unknown`** |
| `BNDLIN` | `CSG_TreeBuilderBNDLIN` | 1187 | **`Unknown`**（noun 由 builder 类名互证） |
| `FLRLAY` | `CSG_TreeBuilderFLRLAY` | 598 | **`Unknown`** |
| `INSURQ` | `CSG_TreeBuilderINSURQ` | 143 | **`Unknown`**（同上） |
| `FPFITT` / `ELFITT` / `HVACFI` / `INFITT` | `CSG_TreeBuilderAccommodationFitting` | 528 | `Composite(Structural)` |

`sub_108F54F0` 同时注册了 40+ 条 `DB_AttPluggerHelper::addPlug`（`ATT_FLLARE`/`ATT_FLLVOL`/
`ATT_INSARE`/`ATT_BNDARE`/`ATT_FLRELE`/`ATT_CLNHEI`…），说明这一族有**自己的一套派生属性**，
几何来自属性求值而不是目录——与我们已有的任何一条路都不同形。

### 3.4 整族缺失：舱壁（ACC）

| noun | Core3D builder | 字节 | 我们判 |
|---|---|---:|---|
| `BPANEL` | `MDR_BPanelVisualisationManager` | 792 | `Composite(Structural)` |
| `CTWALL` | `MDR_WallVisualisationManager` | 474 | `Composite(Structural)` |
| `WLPROF` | `ACC_WallProfileVisualisationManager` | 297 | `Composite(Structural)` |
| `WLJOIN` | `MDR_WallJoinerVisualisationManager` | 268 | `Composite(Structural)` |
| `WLPANE` | `ACC_CSGWallPanelPlugger` | 139 | `Composite(Structural)` |

`BPANEL` 在总纲 §3.3 的 CSG 语义里已经出现过（「属主是 BPANEL 时清空自己的变换」），
但那是**作为负体的属主**；它自己有 792 字节的 builder 这件事之前没有记录。
`WLJOIN` 与 `SCTN` 在 `GTSTDT@0x1036CEC4` 里共用同一条规约分支（recheck-3 §2.2），
**说明它是一条按型材剖面扫掠的段**——`section.rs` 那条链可以直接复用，是这一族里最便宜的一个。

### 3.5 判 `Unknown` 的其余具名 builder（先归类，不一定要建）

`AIDLIN` / `AIDARC` / `AIDCIR` / `AIDPOI` / `LINDIM` / `MLABEL` / `REFGLN` / `REFGAR`（结构标注与辅助线）、
`TRNCUR`（地形曲线）、`SUBCOM`、`DTPROF`。

这一批与前三族**性质不同**：它们是**制图辅助**，在三维模型集里该不该出现是一个产品决定，
不是实现欠账。但它们现在判 `Unknown`——`Unknown` 的语义是「**还没搞清楚**」，
而我们现在搞清楚了。**它们应当从 `Unknown` 挪进一本明确的账**（见 W1）。

---

## 四、这张表怎么变成 D3 闸门的判据（本轮的真正交付物）

总纲 §4 与第一步计划 §1 W1 一直卡在同一个地方：`catalog_pending` 那 1 万件里，
「E3D 也不画」与「我们还没建」**没有可机械求值的分界**，所以 `not_wired` 这本账
三版计划都没落地——不是懒，是没有判据。

**现在有了**：

```
E3D 会为 noun N 建几何  ⟺  N ∈ CSG_TreeBuilder 注册表(65+13待核)
                          ∪ CRDESI switch(41)
                          ∪ 目录路（SPRE→CATR→GMRE 可解析）
```

前两项是**静态可枚举的常量集合**，已经落在 `evidence/20260905-core3d-csg-registry/`。
把它作为数据签进仓库，`not_wired` 就不再是人工分类，而是
「`classify()` 判 NonGraphic/Unknown ∧ noun ∈ 注册表」这个差集——**每次复跑自动求值，
不需要任何人再去反编译里翻一遍**。

---

## 五、工作项

编号供批注引用。工期是净开发时间。**W1 是全部后续项的前置**：它把差集变成一条会红的断言，
在那之前所有件数都是估的。

### W1 · 把注册表签进仓库，让 D3 的差集自己求值（1 天，零几何）✅ 已完成（2026-09-05）

- ✅ `data/core3d-builder-registry.json`——68 个 plug noun（含 builder 名、`getCSGTree` 地址与
  字节数、每个 `addPlug` 调用点）、41 条旧路臂、13 个未解析注册点。由
  `tools/core3d_registry_dump.py`（在 IDA 里跑）+ `tools/build_core3d_registry.py` 生成。
  **两个脚本放 `tools/` 不放 `evidence/`**：`evidence/` 在 `.gitignore` 里，
  搁那儿等于交付一份「注明可重跑、但重跑的东西没进仓库」的数据。
- ✅ `src/core3d_registry.rs`——`core3d_paths()` / `builds_geometry()` / `registered_nouns()`。
  注册表**内嵌**进二进制（与 `route-nouns.json` 同理：编译期定死的常量不该因工作目录换答案）。
- ✅ `tests/noun_coverage.rs::nouns_core3d_builds_that_we_have_not_accounted_for`——第二个分母。
  **当前基线 35 个 noun**；断言失败时逐条打印 builder 名、`getCSGTree` 地址与字节数。
- ✅ `core3d_registry::is_not_wired()`——判据本身，**全 crate 只有这一份**。
  上面那条基线数 noun、下面那本账数件，两处调同一个函数：各写一遍
  `matches!(…Unknown | NonGraphic) && builds_geometry(…)`，先改的那一处会让基线照绿、
  报告上的件数却已经换了口径，而这两个数分歧的时候谁都不会响。
- ✅ `Report::not_wired`（noun → 件数）+ `not_wired_total()`，进 `totals_line`
  （`unknown=` 之后一格）与 `report.json`。**交叉记账，不进 `accounts_for` 的等式**——
  每一件都已经在 `unknown_nouns` 或 `proven_non_graphic` 里数过一次，让它进等式，
  真丢了子树反而能靠它把差额补齐。回归钉：`pipeline::tests::not_wired_stays_out_of_the_balance_equation`
  与 `merge_carries_every_route_ledger`（多根合并漏并一本账 = 凭空少掉一批元素）。

  记账点放在 `match category` **外面**：这本账横跨 `Unknown` 与 `NonGraphic` 两个分支，
  而它们之所以要一起数，正是因为在报告上长得完全不一样——一个挂着欠账、一个记在
  「不欠账」那本。分头记就等于让判据跟着分支走，走着走着只剩一半。

**35 这个数怎么读**：它是 **noun 数不是件数**，也不等于「还要建 35 个 noun」——
§3.5 那 11 个制图辅助大概率该走豁免（§七-3 待拍板）。`Report::not_wired` 落地之后，
件数那一版才是能逐周下降的那个数。

**为什么做成特征化基线，而不是「会红的断言 + 豁免名单」**（与本节初稿的写法不同，
这是实现时改的）：豁免名单要填，前提是 §七-3 那个产品决定已经拍了。在那之前写豁免
等于**替产品先拍板**，而且会把「还没决定」伪装成「已经决定不建」——正是 §3.1 那类错误的成因。
所以按仓库已有的 `cascade_coverage_matches_the_recorded_baseline` 同一套路做：
`cargo test` 保持全绿，名单本身就是 W4 的工作队列，少一个删一行、多一个立刻红。

**这一项的价值不在它自己**：它让后面每一条接线都有一个自动会变的数，
而不是靠人再写一版 recheck 文档来告诉大家还欠多少。

### W2 · 修 §3.1 的判据反转 ✅ 探针已完成（2026-09-05），分类改动待件数

**验收（四个 noun 各一行带 Core3D 地址的依据）已满足**，依据写进了 `category.rs` 的
`"PNOD" | "SNOD"` 与 `"RNODE" | "CNODE"` 两条独立分支。结论比「实体 / 标架」二选一细一层：

| noun | Core3D 出处 | 产出的是什么 | 判 `NonGraphic` |
|---|---|---|---|
| `PNOD` / `SNOD` | `CRDESI@0x10343B80` case 621502 / 621505 → `sub_10402360@0x10402360` | `gm_CreateMark(point)`，**一个点标记** | **对** |
| `RNODE` / `CNODE` | `0x109CB150` / `0x109CABB0`，**无 SPRE** 那一支 | `gm_CreateCombination` + 十几条 `gm_CreateStraight`（CNODE 另有 6 段 `gm_CreateArc`）——**线框标记** | 对 |
| `RNODE` / `CNODE` | 同上，**有 `ATT_SPRE`** → `sub_109CAAD0@0x109CAAD0` | `gm_QueryIsOfType(g,14)` + `gm_CreateSolidTree` → **一件真实体，走目录几何** | **错** |
| `POINTR` | `CAB_CSGRoutePoint::getCSGTree@0x109CB8D0` | `getAtt(ATT_POS)` 之后 `return gm_CreateBox(10.0, 10.0, 10.0)` | 判 `Unknown`，见下 |

三件本轮才看清的事：

1. **`RNODE`/`CNODE` 的判据只反了一半。** 它不是「E3D 也不画」，也不是「整族都是实体」，
   而是**按元素分支**：带 SPRE 的路由节点走目录出实体，不带的只画线框。这与仓库里
   `SCTN` / `centreline_only` 那本账**完全同型**（500 只无 SPRE 的 MSUP 杆件，E3D 给一根 `Line`）。
2. **`RNODE` 的 builder 还有一道 noun 闸**：`sub_102B5530@0x102B5530` 里只有
   `HATTA`(0x1672B5) / `TATTA`(0x1672C1) / `RATTA`(0x1672BF) / `RNODE`(0x320775) 四个，
   且各挂一个运行期显示开关（`unk_10F84878`…）；不在表里的直接 `return 0`。
   三个 `*ATTA` 我们已判 `Composite(CableTray)`，与这道闸一致。
3. **`POINTR` 的 352 件不该排进接线队列。** `noun-coverage-matrix.md` §附录记的是
   `HANDRA → RPATH → POINTR` 是扶手的扫掠路径——它是 **W4-2 (ASL) 那一族的成员载体**，
   会被扶手的扫掠消费掉，不是 352 个各自要建的 10mm 立方体。那个 `gm_CreateBox(10,10,10)`
   是 E3D 给「孤立显示的路由点」画的固定记号，与件的物理尺寸无关。

**分类本身这一轮没动，是刻意的**：改它需要一个数——五库里到底有几只 `RNODE`/`CNODE` 带 SPRE。
那要等 recheck-3 的 N0-1 复跑（§六 的合并顺序正是这么排的）。**在那之前它不是静默的**：
`is_not_wired` 认得这两个 noun，每一只都会进 `Report::not_wired` 与覆盖率基线。
W1 先做的价值就在这里——判据留着是错的，但错得看得见。

**剩下的**：N0-1 复跑出件数之后，`RNODE`/`CNODE` 大概率该走 `SCTN` 的老路
（判 `Catalog`、无 SPRE 的落 `centreline_only`），而不是简单地移出 `NonGraphic`。

### W3 · 补 `RULS` / `NRUL` —— ❌ **撤销（2026-09-05）**

这一项按「先普查语料」那一步自己否掉了，两条都不成立：

- **六库 N0-1 复跑零件**（§九）。
- **这两个 noun 不在 E3D 3.1 字典里**：`out/noun-categories.json` 的 1931 条 verdict 查无此名。
  `CRDESI` 那条臂是 **PDMS 遗留**，不是 E3D 3.1 的建模路。

所以既不能补几何，**也不能补分类**——写进 `classify()` 会让
`every_noun_named_in_the_classifier_actually_exists_in_the_dictionary` 变红，而那条钉是对的。
它们留在 `not_wired` 基线里只是让注册表与分类表的差集闭合，理由已写进基线注释。

原计划里的 1 天省掉。

### W4 · 按族接线 —— **顺序被 §九 的普查推翻了，下表是改过的**

原表按「施工图完备度」排，把 `WLJOIN` 放第一、舱室装修放最后。六库复跑之后
**排第一的那一项是零件的，排最后那一族整族零件**：

| 序 | 族 | 六库件数 | 结论 | 工期 |
|---|---|---:|---|---|
| **W4-1** | **ASL 通道钢结构**（§3.2） | **105 欠账 + 145 已挂账** | 唯一有真件数的一族，**提到第一位** | 4 天 |
| W4-2 | `BPANEL`（原 ACC 五件里唯一有件的） | 52（`aps250110`） | 单件做，不必整族 | 1 天 |
| ~~W4-3~~ | ~~ACC 舱壁其余 4 个~~ | **0** | `WLJOIN`/`WLPROF`/`WLPANE`/`CTWALL` 六库全零，**砍掉** | ~~3 天~~ |
| ~~W4-4~~ | ~~舱室装修 8 个~~ | **0** | 八个 noun 六库全零，**整条砍掉** | ~~4 天~~ |
| W4-0 | `STWALL`（§十，不在注册表差集里，从 `catalog_pending` 里揪出来的） | 302 | ✅ **2026-09-06 已接**：300 出实体、2 件 DESP 退化，RVM 4/4 `exact`（§10.6） | 半天 |

ASL 那一族的件数摊开是：`HRPOST 124` / `HANDRA 92` / `STRFLT 20` / `RAIL 8` / `KICKPL 5` /
`RLADDR 1`，全部落在 `aps250110` 与 `aps250209`；`STRLNG` / `PLTFRM` / `SLADDR` 六库零件。
其中 `HANDRA`/`RAIL`/`KICKPL` 这 105 件判 `Unknown`（在 `not_wired` 上），
`HRPOST`/`STRFLT`/`RLADDR` 这 145 件判 `Composite(Structural)`（在 `evidence_pending` 上）
——**同一族横跨两本账**，这也是为什么单看任何一本都排不出优先级。

**验收不变**：该族 `not_wired` 归零或只剩点名例外；RVM 局部系逐件对拍 100% 配对；
**其它 noun 的 `model.obj` 逐字节不变**。

#### W4-1 施工图（2026-09-05 反编译 + `aps250110` 实测，动代码前先看这一段）

**一、语料里的树形**（`tree_census aps250110`，与下面的反编译逐条对得上）：

```
HANDRA 11
 ├─ RPATH  11 → POINTR 3366     扫掠路径与路径点
 ├─ POSTSE  4 → HRPOST 124 → TMPL 124
 ├─ RAILSE  4 → RAIL 8
 ├─ HRKPSE  3 → KICKPL 5
 ├─ HRPNSE  3 → HRPANE 5 → TMPL 5
 └─ HRFEAT  1
BPANEL 52 → RPATH 52            舱壁也走同一条路径机制
STRFLT 2 → HANDRA 4 / RPATH 2 / STRSTR 2 / TREADS 2
RLADDR 1 → LDRSTR 1 / RLCAGE 1 / RUNGSE 1
```

**`POINTR` 那 3366 件到此坐实是 `RPATH` 的成员**——不是 3366 件独立欠账（§九）。
`*SE` 结尾那一批（`POSTSE`/`RAILSE`/`HRKPSE`/`HRPNSE`）是**集合容器**，
注册表里没有它们，因为它们没有自己的 builder：属主的 builder 迭代它们。

**二、builder 是「薄壳 + 共享引擎」，不是九个独立算法**：

| noun | 壳 | 造上下文 | 共享引擎 |
|---|---|---|---|
| `HANDRA` | `0x109F96A0`(495B) | `sub_109F52B0` | **`sub_109F5560`** |
| `RAIL` | `0x109F9C10`(297B) | `sub_105A9420` | **`sub_105A97C0`** |
| `KICKPL` | `0x109F9890`(297B) | `sub_105B1130` | **`sub_105A97C0`** |

壳做三件事就走：`CSG_TreeBuilderOptions::isWanted` → `gm_SetDefaultLabel` →
（模式非 1/3 时）读 `ATT_LAYOUT`，为假直接 `return 0` → 造上下文 → 调引擎。
**九个 noun 不是九个算法，是两个引擎 + 各自的虚表钩子**（`*vtbl+36/40/52/64`）。

**三、`sub_109F5560` 的骨架**（HANDRA 引擎）：

1. 选项 `mode == 1 || mode == 3` → `sub_109F89B0`，另一条**简化表示**路（障碍体一类），不是主几何。
2. 建两个 `D2_Profile`（扫掠剖面），主 `Combination = gm_CreateCombination(1)`。
3. `DB_Iterator` 遍历成员，逐点读 **`ATT_POS` / `ATT_ORI` / `ATT_SWING` / `ATT_HEIG` / `ATT_WIDT`**。
4. 每根立柱 `sub_10924D80(ref, options…)` 造几何。
5. `ATT_SWING == "OPEN"` 且 `WIDT > 0` 时，造一个
   **`gm_CreateBox(WIDT+40, WIDT+40, HEIG*1.3+20)`**，沿轴偏 `HEIG*0.5 + 10`，加进**负体组** `v89`。
   这是开口处的让位块——常数 40 / 1.3 / 20 / 10 全部写死在代码里，照抄即可。
6. 收尾 `gm_CreateCombination(3)`：正体组 `v90` 与负体组 `v89` 各加一次
   ——**3 是差集**，即「立柱与扶手 − 开口让位块」，再并进主 `Combination`。

**下一步**（真正的 4 天在这里）：`sub_10924D80` 的立柱几何、两个 `D2_Profile` 从哪来、
`*vtbl+52/64` 三个钩子在 RAIL/KICKPL 上各是什么。**先出这三段的伪码再写 Rust**
（recheck-3 §六 约束 5）。**→ 三段 2026-09-06 全部拉出来了，见下面「四、」起；
上面「三、」里有两处读错，也在那里改正。**

#### W4-1 施工图·续（2026-09-06，伪码全部落在 `evidence/20260906-asl-builders/sub_*.c`）

**四、先改两处读错的**

1. `sub_10924D80` **不是立柱几何**，是 `ASL_CSGRungLadderGate::ASL_CSGRungLadderGate`
   （`MTR_Entry` 串里写着），只在 **`HRGATE`** 成员循环里调——扶手门的构造器。
   立柱在 `sub_109F60C0`，见「六、」。
2. 「造上下文 `sub_109F52B0`」其实是 **`ASL_MDR_CSGHandRail` 的构造器**，它的基类是
   **`DB_RoutingBranch`**（`sub_105B7140` = `DB_RoutingBranch::downloadDataFromDB`，
   `sub_105B3670` = `::createCurvesFromPath`，`sub_105B57E0` = `::do_solid_segments`，
   `sub_105B3B60` = `::createPathSegment`，`sub_105BC170` = `::setMitrePlanes`）。
   **扶手在 Core3D 里是一条「路由支线」**：RPATH 的点串变成一串 `DB_Curve`，剖面沿着它扫，
   拐角切斜接面——与 `route.rs` 那条隐式管身的骨架同型，只是剖面与拐角处理不同。

**五、语料有两种扶手，走的是两套 builder**（这一条决定 W4-1 到底要做几件事）

| 库 | HANDRA 件 | 树形 | 走哪套 |
|---|---:|---|---|
| `aps250209` | 81 | `HANDRA → RPATH → POINTR`，**没有** RAILSE/POSTSE/HRKPSE | **布局式**：HANDRA 自己的 builder 画 Ø40 栏杆 + Ø30 自动立柱 |
| `aps250110` | 11 | 另带 `RAILSE→RAIL(SPRE)` / `POSTSE→HRPOST(SPRE)→TMPL` / `HRKPSE→KICKPL(SPRE)` | **详图式**：HANDRA 自己**不画**，RAIL / KICKPL / HRPOST 各走自己的 builder |

闸门在壳 `0x109F96A0` 第 45–50 行：`v21 = 1; getAtt(ATT_LAYOUT, &v21); if (!v21) return 0;`
——`LAYOUT` 为假 HANDRA 就什么都不画。**`LAYOUT` 不在 attlib 里**（`section_probe` 把 HANDRA
的属性全打了一遍没有它，attlib 字节里也搜不到这个名字），是 core.dll 算出来的伪属性；
本机两库对得上的判据是「**有没有 RAILSE / POSTSE / HRKPSE / HRPNSE 成员**」，**待核**
（要核就去 `idalib-33060` 找 `LAYOUT` 的伪属性求值器）。

**六、布局式 HANDRA（`sub_109F5560`，`aps250209` 那 81 件）的完整流程**

属性 → 字段（`sub_109F7150`，vtbl+12 "downloadAttributes"）：

| 属性 | 字段 | 语料样本 `/BUILDING_01_LEVEL_03/HR_001` |
|---|---|---|
| `RAILHE` (real[]) | `+300..304` 栏杆高度表；`+328` = max | `[1075, 525]` |
| `POSTSP` | `+344` 立柱间距 | 1000 |
| `TANGOF` | `+312` 段首 / 段尾立柱退距（**推断**：第 36–40 行两个丢参的 `getAtt`，另一个是 `CLOSEF`） | 300 |
| `EPOSTO` | `+320` 开口支线**两端**的立柱退距（替代 TANGOF） | 0 |
| `POSTOF` | `+336` 立柱竖向起点偏移 | 0 |
| `SHOWKP` | `+352` 画不画踢脚板 | false |
| `CLOSEF` | `+28` 闭合 | true |
| 首个 `RPATH` 成员 | `+128`；其 `DRNS`/`DRNE` → `+32`/`+56`（两端切向，进斜接面） | |

路径（`createCurvesFromPath`）：RPATH 成员逐个走——`POINTR`/`POINSP` → vtbl+20 造直线段
（`FRAD > 0` 时 vtbl+28 在拐角补 `THRU` 圆弧）；`EXTGEO` → `sub_105D13B0` 外部几何段
（`ENDREF`/`STAREF`/`WORPOS`）；`RSECT`/`CURVE` → vtbl+24 圆弧。段号推进 `+260..264`
（`this[65..66]`，实体段表）与 `+288..292`（`this[72..73]`，**立柱段表**）。
段类型码：`LINE`(640317) / `THRU`(958142)。

主流程（`sub_109F5560`）：

```
mode ∈ {1,3}                       → sub_109F89B0（简化表示），不是主几何
vtbl+52 sub_109F7F10               → 剖面：sub_109F65B0 = **固定 Ø40 圆**
                                     （四段 bulge = tan22.5° 的圆弧过 (20,0)(0,20)(-20,0)(0,-20)，
                                      常量在 0x10B801C0..F0；RAILHE 为空直接不画）
vtbl+64 sub_109F9140 setMitrePlanes→ 拐角斜接面 = 相邻段切向的角平分面（sub_105BC170）；
                                     RPATH 的 DRNS/DRNE 做两端切向，垂直 → 平头，平行 → 不斜接；
                                     CLOSEF 时 vtbl+80 把首尾也接上
vtbl+40 sub_105B57E0 do_solid_segments
                                   → 每段 createPathSegment(sub_105B3B60)：LINE 段
                                     gm_CreateExtrusionGroup(圆, 段长) 按段标架摆；THRU 段
                                     gm_CreateProfile + 绕弧扫；→ 一根 z=0 的栏杆
for h in RAILHE: 上面那根栏杆平移 (0,0,h) 各放一份           → v90（栏杆组）
if POSTSP > 0 && TANGOF >= 0:
  for seg in 立柱段表: sub_109F60C0(seg)                    → v92（立柱组）
if SHOWKP: sub_105B0FC0 + sub_105B1370（踢脚板对象）加进 v90
for g in HRGATE 成员: ASL_CSGRungLadderGate(g) 几何绕 z 转 90° 放到 g 的 POS/ORI  → v94；
  g.WIDT > 0 && maxRAILHE > 0 → gm_CreateBox(WIDT+40, WIDT+40, maxRAILHE*1.3+20)
  沿 z 偏 maxRAILHE*0.5+10                                  → v89（门洞负体）
收尾：Combination(3) 差集 (v90 − v89)、(v92 − v89)，再并 v94，全部进顶层 Combination(1)
```

立柱（`sub_109F60C0(seg)`，逐段）：

```
seg 是 THRU → 不放立柱
L = 段长；H = maxRAILHE − 20 − POSTOF；柱 = gm_CreateCylinder(r=15, H)，
    轴心 z = H/2 + POSTOF + 路径点.z（柱顶停在顶栏杆轴线下 20，即 Ø40 栏杆的下缘）
s = e = TANGOF；开口支线（!CLOSEF）：首段 s = EPOSTO，末段 e = EPOSTO
前一段是 LINE 且方向相同(0.01) → s = 0；后一段是 LINE 且方向相同 → e = 0   （共线接头不重复放柱）
s + e > L → 这段一根都不放
n = floor((L − s − e) / POSTSP) − 1
若 L − e ≥ 60：在 s 处放一根
若 L − s < 60：到此为止
否则：若 L − e ≥ 60 且 n ≥ 0：第一根在 s + ((L−s−e) − n·POSTSP)/2，之后每 POSTSP 一根，共 n+1 根
      末根放在 L − e 处（无条件）
```

**七、详图式（`aps250110` 那 11 件的成员）**

| noun | 壳 → 上下文 → 引擎 | 剖面（vtbl+52） | 说明 |
|---|---|---|---|
| `RAIL` | `0x109F9C10` → `sub_105A9420`(ASL_MDR_CSGRail) → `sub_105A97C0` | **`sub_105B9C50`：`ATT_SPRE → ATT_GSTR → 成员按 ATT_LEVE 筛`**，即 SCTN 那条 `SPRE→SPRF→GSTR→GMSS` 目录截面链 | 沿 RPATH 从 `STAREF` 到 `ENDREF`，`ZOFF`=1075 / `XOFF` / `JUSL`=NA / `BANG`；`sub_105A97C0` = setMitrePlanes(vtbl+64 `sub_105B0920`) → do_solid_segments(vtbl+40 同一个 `sub_105B57E0`) → 失败退 vtbl+36 `sub_105AB0D0` |
| `KICKPL` | `0x109F9890` → `sub_105B1130`(ASL_MDR_CSGKickplate) → 同上 | 同上 | `JUSL`=LBOS，`XOFF`=7，`ZOFF`=−10 |
| `HRPOST` | `MDR_HRPostVisualisationManager`（577 B，未拉） | `SPRE`（db 7047）目录截面 | `POSREF` 指路径点，`LENG`=1100 竖直挤出，`ZDIS`/`HRJUSL`；成员 `TMPL` 是模板实例（`PrimitiveContainer`，已归类） |

`ASL_MDR_CSGRail` 与 `ASL_MDR_CSGKickplate` 的 **20 个虚表槽逐槽相同**（`evidence/…/vtbl_dump.py`），
差别全在属性（`JUSL`/`XOFF`/`ZOFF`）。三个上下文共用的槽：+40 `sub_105B57E0`、+44 `sub_105B74E0`、
+48 `sub_105B7410`、+60 `sub_105BBE20`、+68 `sub_105BC870`、+72 `sub_105BB350`。

**八、这张图对排期的含义**

- **布局式先做**：81 件全在 `aps250209`，剖面是常量、立柱是公式，**不碰目录**；差的只是
  「圆沿折线扫 + 拐角斜接」——`solid::cylinder_segments` + `cut_by_plane` 拼得出来，`FRAD>0`
  的圆弧段要一节圆环（`rect_torus_solid` 的圆剖面版）。估 1.5 天。
- 详图式的 RAIL / KICKPL 就是「`section.rs` 的截面链 + 布局式的路径扫掠」，等布局式的扫掠
  落地后再接，估 1 天；HRPOST 再半天。
- **对拍基准还没有**：`test_data/rvm/` 里没有任何一份含 HANDRA 的 E3D 导出。写 Rust 之前
  先从 E3D 导一份 `aps250209` 的扶手（比如 `STRFLT` 下那 36 件里任一件的属主）——
  没有它，布局式那 81 件写完只能自证，进不了 W4 的验收（RVM 100% 配对）。
- HRGATE、SHOWKP、mode 1/3 三条本机语料都没有件（HRGATE 只有 1 件且挂在 TMPL 下），
  先不接，判据留在上面的伪码里。

**顺带答了 §七-4**（原本要请你指认「哪几个库含 ASL / ACC / 舱室装修 的件」）：
含 ASL 的只有 `aps250110` 和 `aps250209`；ACC 只有 `BPANEL` 52 件在 `aps250110`；
舱室装修一件都没有。这个问题不用再问人了，`scripts/run_corpus_census.ps1` 每次复跑自己答。

### W5 · §3.5 的制图辅助族定性（半天，零几何）

11 个 noun 从 `Unknown` 挪进 `ProvenNonGraphic{ reason }` 或新账 `DraftingAid`。
**这是产品决定不是技术决定**——见 §七 请拍板第 3 条。

---

## 六、与 recheck-3 的关系（哪些改、哪些不改）

**不改**：recheck-3 §二 的全部反编译结论（`XSEXTR` / `GTSTDT` / `GFIORI`+`GFIORG` / `GPFORI`+`GPFORG`）、
§三 的 R1–R5、§四 P0 的 N0-1…N0-5。那些是**结构线内部**的精度问题，本轮一条都没碰。

**改**：

1. **§八 锚点表「13 个注册点」→ 80 个**，13 是注册**函数**数。
2. ~~**§2.5 STWALL 的结论加强了**：全表 80 个注册点里确实没有 `STWALL`，
   也不在 `CRDESI` 的 41 条臂里——recheck-3 的证伪判断由「扫立即数」升级为「枚举全注册表」，
   **可以直接按伪需求处理，不必再花那半天**。~~
   **这条 2026-09-05 被推翻了，见 §十。** 注册表里没有 `STWALL` 是**设计使然**，不是证伪。
3. **优先级重排**：recheck-3 的 M1–M4 全是结构线（FITT/PFIT/SCTN/GENSEC/FIXING），
   目标是把已接线那几族做**准**。本轮发现的是另一个维度——**四整族根本没接**，
   而且其中两族（§3.1、§3.2）**在账面上完全看不出来**。
   这两条线不冲突，但 **W1 必须插在最前面**：它零几何、一天，做完之后
   「结构线做准」与「四族没接」这两笔账第一次能同时看见。

**建议的合并顺序**：`W1 → recheck-3 的 N0-1 复跑（此时 not_wired 第一次有真数） → W2 → 视件数决定
先走 recheck-3 M1（结构线做准）还是本轮 W4（四族接线）`。

---

## 七、请拍板

1. **W1 插在 recheck-3 的 N0-1 之前**——同意否？（理由：复跑一次很贵，
   带着 `not_wired` 一起跑，一次拿两笔账。）
2. **W2 先探后改**——同意否？还是直接把 `RNODE`/`CNODE` 移出 `NonGraphic` 记进 `not_wired`、
   探针延后？（前者慢半天，后者会让 `not_wired` 短期虚高。）
3. **§3.5 那 11 个制图辅助 noun（`AID*`/`LINDIM`/`MLABEL`/`REFGLN`/`REFGAR`/`TRNCUR`…）
   进不进模型集？** 这是产品决定：E3D 会把它们画进三维视图，但它们不是物理实体。
   我的建议是**不建，但单开一本 `DraftingAid` 账**，理由写死在豁免名单里。
4. **W4 四族的顺序**（`WLJOIN` → ASL → ACC → 舱室装修）取决于**哪个库是真正要交付的**。
   现在的排法是按施工图完备度；若某一族对应的客户库根本不在交付范围内，直接砍掉比排后面好。
   **请指认：五库里哪几个库含 ASL / ACC / 舱室装修 的件？**
5. **12 个 noun 待核的注册点**（`sub_10728EC0` / `sub_107293D0` 的 `MyBox`/`Cat` 各 4 个、
   `MDR_CSGUtility` 的 4 个）要不要这轮收口？它们的 noun 由入参传入，
   要跟到调用者才能定，估半天。

---

## 八、本轮锚点表（Core3D.dll，重启后 `idalib-49136`）

| 用途 | 地址 |
|---|---|
| CSG 注册入口 | `CSG_TreeBuilder::addPlug@0x107158B0`（**80 个调用点**，13 个注册函数） |
| DB 模型注册入口 | `DB_ModelBuilder::addPlug@0x108E79D0`（1 个调用点：`DTPROF`） |
| 基本体注册 | `CSG_PrimitiveUtilities::initialise@0x10727540`（19 点） |
| 桥架 / 电缆注册 | `sub_109B55E0`（11 点）、`sub_109B61A0`（1 点 `CABLE`） |
| 舱室装修注册 | `sub_108F54F0`（8 点 CSG + 40+ 点 `DB_AttPluggerHelper`） |
| 结构标注注册 | `sub_1062CCE0`（9 点） |
| 通道钢结构注册 | `ASL_MDR_CSGUtility::initialise@0x109F9D40`（5 点）、`sub_108F89A0`（4 点） |
| 舱壁注册 | `ACC_MDR_CSGUtility::initialise@0x10A00520`（5 点） |
| 地形 / 通用图元注册 | `xba_base::Init@0x1077CC00`（3 点） |
| 待核注册点（12） | `sub_10728EC0`（4 点）、`sub_107293D0`（4 点）、`MDR_CSGUtility::initialise@0x105E9DB0`（4 点） |
| 旧路 switch | `CRDESI@0x10343B80`，41 条臂，立即数全表见 `evidence/…/crdesi-switch-decoded.txt` |
| 直纹体 | `RULS 914751` / `NRUL 783446` → `gml_bkend/CRRSLD@0x10349627` |
| 路由节点 builder | `CAB_CSGRouteNode@0x109CB150`(1909B) / `CAB_CSGControlNode@0x109CABB0`(1438B) / `CAB_CSGRoutePoint@0x109CB8D0`(311B) |
| 扶手（布局式）| 壳 `ASL_MDR_HandrailVisualisationManager::getCSGTree@0x109F96A0` → ctor `ASL_MDR_CSGHandRail@0x109F52B0`（vftable `0x10B8011C`）→ 引擎 `sub_109F5560`；属性下载 `sub_109F7150`；剖面 `sub_109F65B0`（Ø40 常量 `0x10B801C0..F0`）；立柱 `sub_109F60C0`；门 ctor `ASL_CSGRungLadderGate@0x10924D80` |
| `DB_RoutingBranch` 基类 | `downloadDataFromDB@0x105B7140` / `createCurvesFromPath@0x105B3670` / `do_solid_segments@0x105B57E0` / `createPathSegment@0x105B3B60` / `setMitrePlanes@0x105BC170`（HandRail 覆写 `0x109F9140`） |
| 扶手（详图式）| `ASL_MDR_CSGRail` vftable `0x10B39F70`（ctor `sub_105A9420`）/ `ASL_MDR_CSGKickplate` vftable `0x10B3A344`（ctor `sub_105B1130`），引擎 `sub_105A97C0`，目录剖面 `sub_105B9C50` |

> 复现：
> ```
> ida-bridge exec <client_id> -f tools/core3d_registry_dump.py \
>     > evidence/20260905-core3d-csg-registry/addplug-sites.txt
> python tools/build_core3d_registry.py      # → data/core3d-builder-registry.json
> python tools/noun_decode.py <code>…        # noun 哈希码反查
> ```
> 原始 dump 落在 `evidence/`（gitignored，本地产物）；脚本与生成出来的 JSON 进仓库。
>
> 运维备注：本轮枚举时用一条 `names JOIN xrefs` 的无界查询把 `idalib-25128` 卡死，
> 已 `supervisor stop` + `start-idalib` 重启为 `idalib-49136`（IDB 无写入，无数据损失）。
> **别再对 `names` 与 `xrefs` 做无界连接**——先把一侧收敛到具体地址。

---

## 九、N0-1 六库复跑：`not_wired` 的第一版真数（2026-09-05）

`scripts/run_corpus_census.ps1`（新增），六个库全量生成 + 收账，**总耗时 21 秒**。
`--sesno` 不给（取各库最新会话）：这个脚本量的是账不是几何回归。

> **2026-09-06 勘误**：下表的 `generated` 是**没钉目录库**跑出来的——`aps250110` / `aps250209`
> 有 4600 多件 GENSEC/SCTN 的 SPRE 链断在 db 7001 / 250701 / 250700 / 7047 没开上，落在
> `catalogue_chain_broken`，而这张表没有那一列。修正后的六库数与脚本改动见 §10.6；
> 本节 `not_wired` 那一半不受影响（它不吃目录）。

| 库 | visited | consumed | generated | catalog | unknown | **not_wired** | 账 |
|---|---:|---:|---:|---:|---:|---:|---|
| `ams1112` | 6059 | 24881 | 4627 | 620 | 0 | 3 | 平 |
| `ams7997` | 65491 | 104557 | 48329 | 2230 | 351 | 43 | 平 |
| `ams8000` | 6088 | 517 | 2505 | 22 | 0 | 637 | 平 |
| `aps250110` | 14333 | 11854 | 1594 | 729 | 8409 | 3449 | 平 |
| `aps250164` | 1048 | 3023 | 927 | 0 | 10 | 0 | 平 |
| `aps250209` | 7344 | 13650 | 1826 | 140 | 1424 | 433 | 平 |

**六库 `accounts_for` 全平**，`cargo test` 全绿（N0-1 的两条验收）。

### 4565 件里只有 105 件是真欠账

| noun | 件数 | 分布 | 这是什么 |
|---|---:|---|---|
| `POINTR` | 3718 | `aps250110` 3366 / `aps250209` 352 | **扫掠路径载体**，`HANDRA → RPATH → POINTR`。会被扶手扫掠消费掉，不是 3718 个 10mm 立方体（W2 §3 已考） |
| `SNOD` | 457 | 四库 | `gm_CreateMark` 点标记，**`NonGraphic` 判对**（W2 已证） |
| `PNOD` | 285 | 四库 | 同上 |
| `HANDRA` | 92 | `aps250209` 81 / `aps250110` 11 | **真欠账** |
| `RAIL` | 8 | `aps250110` | **真欠账** |
| `KICKPL` | 5 | `aps250110` | **真欠账** |

**判据过宽 40 倍**：4460 / 4565 是已经有反证的件。这不是把 `is_not_wired` 写错了——
它照定义算得没错，是**定义少了一维**：注册表只说「E3D 建不建」，没说「建的是实体还是记号」。
`AIDPOI` 画 `gm_CreateSphere`、`AIDLIN` 画 `gm_CreateStraight`、`POINTR` 画 `gm_CreateBox`，
在注册表里长得一模一样。

**没有当场给注册表加这一维**，因为直接扫 `gm_Create*` 调用**不可靠**：67 个 builder 里
32 个是 `indirect`（不直接调，转给别的函数），而 `RNODE` 正是被这样误判成「纯曲线」的
——它的实体在 `sub_109CAAD0` 那条 SPRE 支上。要做就得做传递闭包，那是独立一项。
在那之前 `not_wired` 该按 noun 读、不该按总数读。

### 这一跑顺手结掉的四件事

1. **`RULS`/`NRUL` 零件、且不在字典里** → W3 撤销（见 §五）。
2. **`RNODE`/`CNODE` 六库零件** → §3.1 标的「最高优先级：判据反了」在交付语料里**一件都不影响**。
   W2 的探针没白做（判据确实错了一半），但它的**优先级应当降到最低**。
3. **ACC 舱壁 4/5 零件、舱室装修 8/8 零件** → W4-3、W4-4 砍掉（见 §五 W4 改后的表）。
   原本排第一的 `WLJOIN` 也是零件。
4. **`STWALL` 302 件仍在 `catalog_pending`**（`ams1112` 76 / `ams7997` 170 / `aps250110` 56）。
   这一条随后单独查了，结论是伪需求那个判断错了——见 §十。

> 复现：
> ```
> cargo build --release --bin gen_ams
> pwsh -File scripts/run_corpus_census.ps1 -Tag n0-1-20260905
> python tools/census_nouns.py n0-1-20260905       # 逐 noun 落在哪本账上
> ```

---

## 十、STWALL 定性：伪需求那个判断是错的（2026-09-05）

recheck-3 §2.5 与本文 §一 都说 `STWALL` 是伪需求，依据是「它不在 80 个 CSG 注册点里，
也不在 `CRDESI` 的 41 条臂里」。**这个依据不成立**，三条独立证据：

### 10.1 注册表看不见它，是设计使然

`CSG_TreeBuilder::getCSGTree@0x10715B30` 的查表不是一次，是**三次**：

```
v8  = DB_Element::actualType(el);   查中 → 调
v16 = DB_Element::type(el);         != actualType 且查中 → 调
      DB_Element::hardType(el);     != type 且查中 → 调
return 0                            三次都不中才是「E3D 不画」
```

而 `ams1112` 的 `STWALL` 元素身上 **`GTYP = WALL`(776435)**，`TYPEX = [1, 179132626]`。
它按 **`WALL`** 那一格查 builder——**枚举注册表永远看不见 `STWALL`，无论 E3D 画不画它**。
`builds_geometry(noun)` 这个判据少了 `type` / `hardType` 这一层重定向。

### 10.2 「扫立即数」这个方法本身不成立

`STWALL` 的哈希 `179132626`(`0xAAD58D2`) 在 **Core3D.dll 与 core.dll 两个映像里都零命中**。
但**对照组 `WALL`(`0xBD8F3`) 同样零命中**——而 `WALL` 是确定有建模路的。
这些 noun 走 `NOUN_*` 全局与表，不是 switch 立即数。**零命中不是证据**，
recheck-3 §2.5 的证伪从第一步就不成立。

### 10.3 语料里它有完整的参数化剖面链

`section_probe --noun STWALL`（`ams1112`，76 件全是同一形态）：

```
17496/105701 STWALL   属主 CWALL /1RS-WF03-W-C-R3100
  GTYP = WALL          TYPEX = [1, STWALL]
  DESP = [1560.0, 3620.0]
  POSS = [-5930.0, 4097.15, -20.0]      DRNS = [-0.846, 0.533, 0]
  POSE = [-10635.36, 12085.27, -20.0]   DRNE = [0.669, -0.743, 0]
  SPRE = 15193/27717 @ db 7001
     └─ SPRO (LEVE [0,3] / [4,8] 两档表示) → 4 × SPVE
          PX = ( ATTRIB DESP[1] )   PY = ( ATTRIB DESP[2] )
```

**一个按元素自身 `DESP` 参数化的矩形剖面，沿 `POSS → POSE` 扫掠**，
带 `DRNS`/`DRNE` 端向与 `LEVE` 表示分档——与 `SCTN` / `WALL` **完全同型**。

### 10.4 结论与排期

- 判 `Catalog` **是对的**，`EvidencePending` 也是对的。302 件是**真欠账**，不是伪需求。
- 它可能是**目前最便宜的一族**：矩形剖面 + 起止点 + 端向，`section.rs` 那条链直接复用，
  比 §五 W4-1 的 ASL（4 天）便宜得多，而件数是 ASL 那 105 件的近三倍。
  **→ 2026-09-06 已接线，300/302 出实体、RVM 对拍 4/4 `exact`，见 §10.6。**
- 树形：`CWALL → STWALL 76 → FITT 86 / CMFI 3 / SNOD 2`。成员是配件，不是几何数据。
- **运维**：`db 7001` 必须显式 `--catalogue D:\AVEVA\Projects\E3D3.1\AvevaCatalogue\acp000\acp7001_0001`。
  它在 `AvevaCatalogue` 与 `YCYK-E3D` 两处同号，`--project-dir` 按命名约定分不出正身，整组不外交。

### 10.5 这条给 `not_wired` 留下的账

`is_not_wired` 现在只查 `actualType` 这一格。**同型的漏判可能不止 `STWALL` 一个**：
任何「自己没有 builder、靠 `GTYP` 重定向到别人那一格」的 noun，
在这本账上都长得跟「E3D 也不画」一模一样。要收口就得把 `GTYP` 也读进来——
那是一次语料侧的普查（每个 `Unknown`/`NonGraphic` 的 noun 抽几件看 `GTYP`），不是反编译。

### 10.6 接线结果（2026-09-06）：三处改动，300/302 出实体，RVM 对拍 4/4 `exact`

「`section.rs` 直接复用」这句在 10.4 里还是看剖面形态猜的，现在是接过线、对过拍的：

**改了什么**（只碰 noun 名单，截面链一行没动）：

| 处 | 改动 |
|---|---|
| `category::is_model_unit` | `Catalog` 臂加 `STWALL`——增量上卷与全量产出共用这一份名单，只加这里不加下面那处会落到 `other` 臂变 `Skipped(NoShape)`，账面上像「E3D 也不画」 |
| `elmodl::model_element` | `SCTN \| GENSEC` 臂改成 `is_section_like()`（`SCTN`/`GENSEC`/`STWALL`），三个 noun 走同一个 `section_body` |
| `elmodl::section_fitting_parts_of` | `STWALL` 的成员 FITT **不**并进本体（`SCTN` 照旧并）。依据是 RVM 基准：`STWALL n of CWALL /1RS-WF03-W-C-RR001` 每条都是 6 面 24 顶点的裸盒，配件没有随墙画出来 |

`GTSTDT` 的两端切平面这条线**不用分支**：RR001 那四件里两件是斜切的
（`17496/105812` 两端各偏轴 15.7°、`17496/105815` 各偏 11.9°），另两件平头；
SCTN 那套 `DRNS`/`DRNE` 切面把斜的、平的都切成了 `exact`。

**对拍**（`rvm_compare`，两种模式各跑一遍，`test_data/rvm/1RS-WF03-W-C-RR001.rvm[.json]`）：

```
STWA: 两侧都有 4 (exact 4 / chord 0 / off 0)，只在基准 0
  RVM↔RVM     raw ≤ 0.0015 mm    快照+facets↔elements.json   raw ≤ 0.0020 mm
```

差的是 RVM 的 f32 落盘，不是几何。这四件 2026-08-31 的 `compare-1RS-WF03-W-C-RR001.json`
里还是 `only_baseline`。

**六库件数**（`scripts/run_corpus_census.ps1 -Tag stwall-census-20260906`）：

| 库 | STWALL 出实体 | 没出的 | 为什么 |
|---|---:|---:|---|
| `ams1112` | 75 / 76 | 1 | `17496/106500` 的 `DESP = [100, 0]`——零高矩形，E3D 同样没东西可画；落 `centreline_only` |
| `ams7997` | 169 / 170 | 1 | `24381/42189` 的 `DESP = [0]`——`ATTRIB DESP[2]` 都取不到；同上 |
| `aps250110` | 56 / 56 | 0 | |

`catalog_pending` 六库合计 −302，`accounts_for` 六库全平。链形态全部是
`SPCO → SPRF(/Variable_thk_wall, GTYP=WALL) → GMSS(/STD-WALLS-SINGLE-GMSS) → SPRO ×2`
（`LEVE [0,3]` / `[4,8]` 两档，四个 `SPVE` 是 `(0,0) (DESP[1],0) (DESP[1],DESP[2]) (0,DESP[2])`），
ams7997 另有 2 件走 `SREC`，一样出。

**顺手揪出的一件事：N0-1 那张表（§九）少算了 4600 件几何，而且看不出来。**
`STWALL` 的 `SPRE` 全指 db 7001，它只在 `AvevaCatalogue\acp000` 里，`--project-dir` 给样例
工程扫不到，`acp000` 整目录又不能给（7000 与 `ams7000` 同号撞车）。第一版 census 一个目录库
都没钉，于是 `aps250110` 的 GENSEC 1466 / SCTN 40 / STWALL 56、`aps250209` 的 GENSEC 2698
全落在 `catalogue_chain_broken`——**而那张表没有这一列**。现在脚本显式钉四个文件
（7001 / 7047 / 250701 / 250700，链要走两跳）、表加 `Centreline` / `ChainBroken` 两列：

| 库 | generated（§九） | generated（现在） | ChainBroken | 备注 |
|---|---:|---:|---:|---|
| `ams1112` | 4627 | **4702** | 0 | +75 STWALL |
| `ams7997` | 48329 | **49769** | 481 | +169 STWALL，其余是 7001 解开的 SCTN；剩 481 只 SCTN 指 db 7600，本机只有 `aps7600`/`zdj7600`，海工工程没有——语料自身悬空，不钉 |
| `ams8000` | 2505 | **2828** | 2 | 全是 7001 解开的 SCTN |
| `aps250110` | 1594 | **3156** | 0 | GENSEC 1466 / SCTN 40 / STWALL 56 |
| `aps250164` | 927 | 927 | 0 | |
| `aps250209` | 1826 | **4524** | 0 | GENSEC 2698 |

`not_wired` 逐 noun 一个数没变（它不吃目录），§九那张 4565 件的表仍然成立。

**验证摘要**：`cargo test --lib` 245 条 + `noun_coverage` 8 条全绿（`the_model_unit_predicate_…`
那条钉从「STWALL 本期不产实体」翻成「是模型单元」）；`cargo fmt` 我碰的文件干净；
ams1112 非 STWALL 的 4627 件逐 refno 对比 N0-1 产物：refno 集合相同，FLOOR 有 3 件体积 /
三角数不同（rel ≤ 9.5e-4），那是并行会话今天在 `solid.rs` / `elmodl.rs` 上的 FLOOR 布尔清理
改动，不在本条路径上；真库门 `increment_real` 3 条全绿（1177 秒，`is_model_unit` 改名单
正是它盯的那类静默漏件——增量必须能上卷到新认领的 STWALL）。

# e3d-model 下一步开发方案（结合 ida-bridge 复核）

> 日期：2026-09-01。范围：`vendor/e3d-model` 当前模型生成算法的审核结论 + 下一步方案。
> 复核手段：读全部源码/证据 + 对活体 IDB（`Core3D.dll` / `core.dll` / `libgeom.dll`）用
> ida-bridge 反编译交叉验证。四数覆盖基线沿用 `docs/noun-coverage-matrix.md`
> （1931 / 327 / 30 / 297 / 0，SNOU/NSNO/ASNOU 接入后 27→30，矩阵提交 1b16935）。

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
| 目录件几何 `catalogue.rs`（**未提交/新增**） | GMRE 程序求值：SEXT/SBOX/LPYR/SCYL/LCYL/NSCY/NSBO/NLCY/SCTO/SRTO/SSPH + HVAC 矩形快路；成员局部网格缓存 | 与 CRCATI 各臂逐个对上（见 §二） |
| 增量 `increment.rs`/`element_diff.rs` | 会话窗口差分 → 单元级重建；模型单元/派生单元两条正交判据 | `tests/increment_real.rs` |

**未提交工作树的定性**：`git status` 显示 `src/catalogue.rs` 为**未跟踪新增**，
`route.rs`/`catalogue_point.rs`/`category.rs`/`pipeline.rs`/`elmodl.rs`/`solid.rs` 均有改动
未提交。这一整摊就是「二期 G4 / 管道与目录件几何」这条线的在建代码——也就是本次
「继续」的对象。`evidence/20260831-pipe-model-generation/README.md` 里「管道零建模、
FTUB/BEND/ELBO 落 unknown」的结论**已过时**：`category.rs` 现在把这 8 个管件族判为
`RouteMember` 并送进 `catalogue.rs`，需要一次新的实跑重新基线。

---

## 二、ida-bridge 复核的三条关键结论

### 2.1 目录基本体覆盖不全：CRCATI 有 ~19 条派发臂，`catalogue.rs` 只实现了 ~11 条

`CRCATI`（Core3D `sub_10345ACC`）是目录基本体的总 `switch`，按 noun 码派发到 libgm
`GMC*` 建体。逐臂对照 `catalogue.rs` 已实现的 noun：

| CRCATI case（正/负孪生） | 内部 helper | 建体 | `catalogue.rs` 是否覆盖 |
|---|---|---|---|
| 1014841 / 828671 | `sub_103BFCEC` | GMCBOX | ✅ SBOX / NSBO |
| 701101 / 860747 | `sub_103C2480`(QSSPHE) | 球 | ✅ SSPH |
| 785955 / 1026041 | `sub_103C0374`(QLCYLI) | 圆柱（两距）+0.01 | ✅ LCYL / NLCY |
| 841366 / 927815 | `sub_103C0E84` | 圆环 SCTO | ✅ SCTO |
| 841771 / 938750 | `sub_103C0FA0` | 矩环 SRTO | ✅ SRTO |
| 702883 / 908861 | `sub_103C0CBC`+ORIMAT | 圆柱（PDIS/PHEI） | ✅ SCYL |
| 904404 / 1035518 | `sub_103C07B4`(QLPYRA) | 棱台 | ✅ LPYR |
| 968617 / 643505 | `sub_103B81CC`→GMCCOM | 挤出/组合 | ✅ SEXT |
| 818038 / 829400 | `sub_103BFE40` +0.01 | 圆柱变体 | ❌ **缺** |
| 785962 / 1026230 | `sub_103BF3FC` +0.01 | 圆柱变体 | ❌ **缺** |
| 837417 / 821192 | `sub_103BFFC4` +0.01 | 圆柱/锥变体（两径） | ❌ **缺** |
| 599770 / 782015 | `sub_103C0514`+ORIMAT+CSRTAX（4 角） | 斜切/snout | ❌ **缺**（`solid::snout_solid` 已有，只差接线） |
| 942751 / 1008005 | `sub_103B7FCC`→`sub_104030F0` | 碟形/回转（单参） | ❌ **缺**（`solid::revolve_rings` 可复用） |
| 594640 | `sub_103B88BC`（7 参） | 单臂 | ❌ 待定性（疑 2D/截面 SPRO/SANN 类） |
| 817255 | `sub_103B8E70`（9 参） | 单臂 | ❌ 待定性（疑回转 SREV 类） |
| 840259 | `sub_103B8410` | 单臂（仅 &v5） | ❌ 疑 LINE/中心线元数据，可跳过 |
| 726491 / 726152 | `sub_103BF6D8`→GMCBOX | 盒变体 | ❌ 疑管身盒（走 GTTUBG，非 catalogue） |
| 631901 / 586670 | `sub_103B8834` | 圆柱（管身体） | ❌ 疑管身圆柱（走 GTTUBG，非 catalogue） |
| 640317 / 3471112 | `sub_103474C7` | GFX_Box 元数据 | ❌ 疑包围盒元数据 |

**后果**：未实现的臂在 `catalogue.rs` 里落 `"catalogue primitive noun X is not implemented"`
诊断，对应成员**静默少一块几何**（诊断进 `notes`，账仍平，RVM 对拍才看得出体积缺失）。
好消息是缺的正体多半有现成 builder（snout/revolve），主要工作是**写求值臂 + 抄参数映射**，
不是造新内核。

### 2.2 GTTUBG 的管身其实是「头段套管 + 逐成员目录几何 + 尾段」，且无显式零长阈值

`GTTUBG`（Core3D `0x10340f8e`）复核要点：

- 外径走 `sub_103BF7AC`（CGETOD）：取不到时回落**到达侧** bore（`if (v17) v60 = v57;`），
  与 `route::tube_outer_diameter` 一致。
- 遍历分支成员，对 noun 码 `631901`/`726491`/`640317` 逐个建目录几何（离开侧 stub），
  再在无成员时走**默认头段圆柱（631901）+ 尾段盒（813891）**。这比 Rust 当前「每槽位一根
  圆柱」更细：Core 的管身是**成员的目录 stub 几何**串起来的。
- **没有看到显式 `if (length < eps)` 零长判据**——Core 依赖几何内核（GMCTRA/GMAMEM）
  拒绝退化体。故 `route::ZERO_LENGTH_TOL_MM`（现占位 = 0.051mm）是 Rust 侧的安全阀，
  不是抄来的常量；对拍阶段要么按真库钉死、要么改成「交给内核判退化」并在文档里写明。

### 2.3 +0.01mm 轴向过切是 CRCATI 的通用行为

CRCATI 多条负圆柱臂（818038/829400、785962/1026230、837417/821192、821192）都有
`v48 = v48 + 0.01;`——与 `catalogue.rs` 的 `NSCY_AXIAL_OVERCUT_MM = 0.01` 同源。
补新臂时这条过切要一并带上，否则共壁负体会在 Manifold 里碎成薄片。

---

## 三、差距清单（按优先级）

**P0 —— 管道/目录件几何这条线（“继续”的主轴）**

- **G0-1 目录基本体补全**：把 §2.1 里缺的臂逐条实现并 RVM 对拍。先做有现成 builder 的
  snout（599770/782015）与碟形/回转（942751/1008005），再定性并实现三条圆柱变体
  （818038/829400、785962/1026230、837417/821192）。2D/截面单臂（594640、817255）先定性
  再决定是否属本线。
- **G0-2 管道端到端 RVM 验证**：在含 BRAN/FTUB/BEND/ELBO 的真库（ams8000 或 aps 管道库）
  上跑全量，用 `CataloguePoints` 解 LEAV/ARRI 成员 P 点，收敛 `missing_ppoint`，再对
  真 E3D 管道 RVM 逐件对拍。这是 `MDR_BranchVisualisationManager::getCSGTree` 语义的落地。
- **G0-3 隐式管身对齐 GTTUBG**：核对头/尾 stub 语义（§2.2），确认 Rust「每槽位一圆柱」
  与 Core「成员 stub 串接」在真库上产物一致；顺带钉死或文档化 `ZERO_LENGTH_TOL_MM`。
- **G0-4 重做 ams8000 基线**：`catalogue.rs` 落地后，`evidence/.../pipe-model-generation`
  的账已过时，需重跑刷新 unknown/generated/orphans 四数。

**P1 —— 目录件与复合构件**

- **G1-1 结构型材/目录件**：GENSEC/SCTN/FITT/PFIT/CMFI/FIXING 仍只记账
  （`catalog_pending`，除 SBFI/WALL）。型材要走目录截面（SPRO/section）求值，与 SEXT 同源。
- **G1-2 复合成员聚合 builder**：Composite(Structural 58 / CableTray 20 / BuildingAssembly 17
  / PipeComponent 12 / Equipment / Catalogue / CoreClass28)。对标 `sub_107210A0` 成员聚合链
  （逐成员 `LegacyGeom_CreateBuilderForElement` 串 builder）。class 28（NOZZ/ELCONN/EQUCOM）
  在现有语料里全是叶子、无成员可聚，先补有成员的 Structural/CableTray。

**P2 —— 收口项**

- **G2-1 宿主负体机制**：`TMPL` 名下 1559 件负体扣的是容器外宿主（墙/梁），现只记
  `host_negatives`、一刀未扣。要先能从负体定位宿主（`NAPP`？连接关系？core.dll 另有机制？），
  是 `TMPL` 之外的独立课题。
- **G2-2 未实现正基本体**：CONE/DISH/SLCY（+ 负体 NCON/NDIS/NSLC）。ams8000 有 CONE×4，
  对拍 8000 前需补；补正体时把成对孤儿负体一起接。
- **G2-3 几何集**：GeometrySet/NegGeometrySet（core.dll `CGTCT2 + sub_10714FC0` 另一条建模路），
  现只记账，当前语料无实例，优先级最低。
- **G2-4 分类清扫 + 级联外定性**：以新 ams8000 账为准清扫残余 unknown（HANDRA/IASLCO 扶手线、
  级联外未知 noun），按 `docs/noun-coverage-matrix.md` §9/§11 的口径归并。

---

## 四、建议执行顺序与验收

1. **G0-4 先跑**（半天）：刷新 ams8000/管道库基线，把「现在到底缺哪些几何」变成硬数字，
   作为后续每一步的对照底。
2. **G0-1 目录基本体补全**（数天，逐臂）：每补一条臂，`E3D_DEBUG_CATALOGUE_SBOX` 式取证 +
   RVM 逐件对拍，`notes` 里的 "not implemented" 计数逐条清零。
3. **G0-2 / G0-3 管道端到端**（核心）：这是覆盖率从 8.3% 往上走的主战场。
4. P1 / P2 按上表推进。

**验收判据（每一步都必须过）**：

- `accounts_for` 硬判据不破（`visited + consumed == 索引总数`）。
- RVM 局部系逐件对拍（`rvm_compare`）：新增几何 100% 配对，偏差落在已证容差内或标注为参考碎片。
- 只读库身份：跑前后 DB SHA-256 一致。
- `cargo test`（含 `noun_coverage` / `increment_real`）全绿；改覆盖面同步更新覆盖矩阵与断言，
  不放宽断言。
- 新补的目录臂：正/负孪生走同一几何路径，+0.01 过切按 CRCATI 带上。

---

## 五、复核用到的锚点（便于下一步直接接手）

- IDB：`Core3D.dll`（idalib-35724）、`core.dll`（idalib-41236/22484）、`libgeom.dll`（idalib-36012），
  ida-bridge 已在线，可直接 `exec ... --sql "SELECT decompile(0x...)"`。
- 关键函数：`GTGEOM@0x10341d2e`、`GTTUBG@0x10340f8e`、`CRCATI/sub_10345ACC`、
  `TUMAT@0x103439c0`、`ORIMAT@core.dll 0x526d674`、`CGETOD/sub_103BF7AC`、
  `QLPYRA/sub_103C07B4`、`QLCYLI/sub_103C0374`、`QSSPHE/sub_103C2480`。
- 管道更上层语义：`MDR_BranchVisualisationManager::getCSGTree`（分支可视化 CSG 树）。

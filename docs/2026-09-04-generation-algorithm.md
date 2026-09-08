# e3d-model 生成算法总纲（对标 core.dll / Core3D）

> **这份是「该怎么生成」的规范，不是「还欠什么」的计划。** 计划见
> `docs/2026-09-04-development-plan-core3d-recheck-2.md`（下称 recheck-2）与它承接的两份；
> 两者分工：总纲定形状与判据，计划定工期与差距。**两边冲突时以本文为准，并把计划改过来。**
>
> 依据：活桥 `core.dll = idalib-33060`（0x5 映像）、`Core3D.dll = idalib-25128`（0x10 映像）反编译，
> 加 gen-model `teach/learning-records/0009`、`上下文/会话-2026-08-31-…-7UW4.md`。
> 本文出现的地址都是这两个映像里的实址，改代码前照着复核。

---

## 一、契约

```
generate(refno, 已打开的库集合, 会话时点) -> (Vec<GeneratedElement>, Report)
```

- **输入是一个 refno**，不是一批：批量是外层对多个根反复调用（`Report::merge` 合账）。
- **输出全在内存**：世界系 `Manifold` + 稳定身份 `GeometryId` + 一本能自证完整的 `Report`。
- **不写任何东西**：不落盘、不写库、不碰 IDB。持久化、增量、房间是本层之外的事。
- **只读的时点是钉死的**：同一次生成里所有库读同一会话，不许中途换。

**这一层的成功定义不是「网格好看」，是「一个都没丢，丢了的每一个都有名有姓」。**

---

## 二、一个 refno 的「所有模型」是三重展开的乘积

`core.dll` 侧的入口是 `ADDELE(0x1021b4ec) → ADDDES(0x1021d005)`。不是「一个元素一个网格」：

| 段 | Core3D / core.dll | e3d-model | 展开出什么 |
|---|---|---|---|
| ① 子树展开 | `ADDDES` 的 `DBEFOR + DFIND` 走查 | `pipeline::generate_subtree` | refno → 该出模型的**元素集合** |
| ② 表示展开 | `ELMODL(0x1025277e)` 外层 `while` + `DRAWOP(0x10254579)` | `catalogue::repre_level_of` + `display_filtered` | 一个元素 → **1~2 个模型**（主体 / 绝热·障碍） |
| ③ 几何展开 | `GTGEOM(0x10341d2e)` → CSG builder 注册表，回退 `CRDESI(0x10343b80)` | `elmodl::model_element` / `catalogue` / `route` | 一个模型 → **一棵 CSG 树**（正体 ⊖ 负体） |
| ④ 落集 | `SGDRAW(0x102556de)` 递归几何集 → SGL 段；`maplib` 登记元素↔段 | `Vec<GeneratedElement>` + `GeometryId` | CSG 树 → **内存模型集** |

三段都可能「多出来」：子树里一个元素能带出一串隐式管身（库里根本没有记录），
一个元素能占两个段，一棵 CSG 树能是几十个图元的组合。**任何一段把「多」压成「一」，
丢的东西都不会报错。**

---

## 三、逐段规范

### 3.1 子树展开——遍历与剪枝

**core.dll 的游标语义**（`DFIND` 真身 `sub_5AB6CB0`，本轮逐行核过）：

- `DFIND(方向码, 步长)` **只在当前元素的成员表里挪一格**，它不是递归遍历器。
  方向码：`942746`=下一个、`968614`=上一个、`639379`=原地、`938991`=从末尾数、`11547609`=绝对下标。
- **下钻靠「不调 `DRETUR`」**，回溯靠 `DCLERR() + DRETUR()`，边界是走查前 `DDEST(-1)` 重置出来的那个指针栈
  的栈根（`DDEST` 的 trace 串是 `Destroying stack no %d`，栈根即 `DGOTO` 的落点）——弹不回去了，
  错误位不再被清，`while (!DGTERR())` 自然收口。*终止这一步是从代码形状推的，`sub_5AE9460` 未展开。*
- 逐元素读的就三样：`DGETI(642215)`=类型码、`DGETI(243798436)`=第二类型码、`DGETF(535968)`=refno。

**Rust 侧的等价形式**（已实现）：显式栈 + 反序压栈保文件原序。递归会在深树上爆栈，`DFIND` 那套
指针栈本来也是显式的。

**三道剪枝，一道都不许多、不许少**：

1. `graphicsBehaviour == 1`（字典字段码 `5099119`，经 `ATNINT` 读）→ **整棵子树不下钻**。
   ADDDES 在这里直接 `DRETUR`。对应 `category::graphics_ignored_below`，成员整棵计进 `consumed`。
2. `LISTOP` 真 → 交列表展开路，本元素不建模。
3. **被属主消费掉的成员**（自己的剖面环、自己的负体）不入栈，整棵计进 `consumed`。

**红线：「本元素不建模」≠「它下面没有能建模的东西」。** `Catalog` / `Unsupported` / `NonGraphic`
都必须继续下钻。这条反过来写过一次，代价是 30940 个元素只走到 160 个，报告全绿。

**唯一的硬判据**：`visited + consumed == 索引侧独立数出的子树总数`（`Report::accounts_for`）。
`tree_total` 必须来自 `scan_elements` 那样的独立来源；跟遍历同源就什么都验不出来。

### 3.2 表示展开——一个元素可能有两个模型

`ELMODL` 的外层 `while` 容易看漏。首遍 `v8 = -1`；跑完一遍后，若元素是管件 / 吊架 / 几何集
（`IHCOMP | IPCOMP | IGMCOM` 或 5 个特定 noun 码）且算出的透明度非 0，就：

```c
v8 = 0;  dword_10EB4004 &= ~8u;  dword_10EB4004 &= ~4u;   // 关掉两个表示位
v130 = -v130;  GLGOTO(ref);                                // 切到 -ref 那个段
// 整个函数体再跑一遍 → 第二个模型
```

- **`-ref`（refno 取负）就是第二个段的键**。`ADDDES` 里也是同一套：`IPCOMP | IHCOMP` 的元素
  除了 `+ref` 还要 `GLXIST(-ref)` / `SEGCRE(-ref)`。
- 透明度是位选出来的五档 **12 / 25 / 50 / 75 / 87**，存进 `qword_10E8C6A0` 再喂给 `GTGEOM`。
  Core3D 字符串侧的对应物是 `PML_DrawList::setInsulation / setInsulationTrans /
  setObstruction / setObstructionTrans / setProfileObst / setCablewayObst`——**`Trans` = 透明度**。
- 学科分流在 `DRAWOP`：`IFCOMP/IPFCOM/IASLCO`→`dword_10EB4000`、noun `3426330`→`…4008`、
  `ICABCO`→`…400C`、noun `710633`→`…4010`、其余（含管件）→`…4004`。位 1/2/4/8 推四个几何选项
  setter，再加 `LEVGET` 的绘图 level。**全部在 `GTGEOM` 之前**——闸门先于几何，别先建网格再丢。

**本 crate 的现状与待办**：表现级（`MODCMP` 那张学科表）已落在 `catalogue::repre_level_of`，
目录图元侧的 `OBST / TUFL / CLFL` 门（`DR3DWA` / `DRWANT`）已落在 `catalogue::display_filtered`，
开关 `E3D_REPRE_OBST`。**尚未坐实的是设计侧这条 `-ref` 第二遍与目录侧 OBST 门是不是同一件事**：
- 若是同一件事（管件的绝热/障碍全部来自目录 GMSE 的 OBST 图元），现状不欠，只欠在文档里写死；
- 若不是（设计侧另有一层绝热壳），模型集就少了一整类，而**账面上完全看不出来**。

→ 收口探针（P0，便宜）：取一只带绝热的管件，`REPRE INSU ON` 与 `OFF` 各导一次 TTY，
比对图元数；同时看 `GTGEOM` 在 `qword_10E8C6A0 != 0` 时走的是不是同一条目录路。
**在这个探针出结论之前，不要给 `GeometryId` 加表示维度**——键一旦发出去就改不动了。

### 3.3 几何展开——三条路，优先级固定

`GTGEOM` 开头就是路由：

```c
if (全局旗标强制旧路 || (现代路 GTGM2(0x10714e60) 失败))  → 旧 FORTRAN noun-switch
else                                                      → class = 4，CSG 树直接返回
```

1. **现代 C++ CSG 路（首选）**：`CSG_TreeBuilder::getCSGTree(0x10715b30)` 按
   `actualType → type → hardType` 三查 noun→builder 注册表，**查不到就返回 0，没有兜底**。
2. **旧 FORTRAN 路（回退）**：`CRDESI(0x10343b80)` 按 noun 大 switch 建基本体，
   `*a2 = 1/2` 标正负几何。独立的 DISH/SNOU/CTOR/RTOR/SLCY/POLYHE/EXTR/REVO 走的是这条。
3. **目录路**：`SPRE(643429) → SPCO → CATR(900345) → 目录件 → GMRE(643336)/GSTR(900835)/NGMR(895415)`，
   对应 `catalogue::model_catalogue_member_with_cache`。**FITT / PFIT / CMFI / SBFI / 路由成员
   走的是同一条**，差别只在标架。

**CSG 语义（三条，逐条都有反编译出处）**：

- 正图元 = 自身 ⊖ 其下负几何子（`addHolesBelowPrimitive`）⊖ 模板孔（`addHolesBelowTemplate`）；
- 独立负几何 = 登记为「待被属主减去」，属主是 BPANEL 时清空自己的变换（在板坐标系里做差）；
- 板 / 墙 = 边界环拉伸 ⊖ 各负几何成员，`gm_CreateCombination(3)` 差集，板做被减数。

**扣不扣由 `NAPP` 三档门决定**（recheck-2 §2.3 坐实）：bit0=扣自身、bit1=扣所属型材/邻件、
bit2=模板孔；`NAPP == -1` 时只有 builder mode ∈ {0,3,4,5,6} 放行。忠实实现，不要「一律扣」。

**尺寸怎么读**：按 noun 读固定几个具名设计属性再调对应 `gm_Create*`，全表在
7UW4 §3.6（BOX=XLEN/YLEN/ZLEN、CYL=DIAM/HEIG、SNO=7 参…，直径一律 `/2`）。
EXT/POL/REV 三个非标量图元的装配在 §3.7。

### 3.4 落内存——身份与账

- **身份是 `GeometryId`，不是 refno。** 一个容器 refno 名下会有 N 根隐式管身，只有那把
  「容器 + 两端站点 + 同站点对内出现序」的键区分得开。模型集、增量 Remove、真库门
  **一律按 `GeometryId` 索引**。
- **绝不制造伪 refno**：派生几何的 `refno` 字段固定填来源元素。
- **派生件不进 `accounts_for` 的等式**：隐式管身既不是 `visited` 也不是 `consumed`，
  单开一本 `implied_tubes`。塞进等式会让唯一的硬判据变松。
- **世界系交付**：`GeneratedElement::solid` 已是世界系、负体已扣。局部系留在 builder 内部。

---

## 四、第一步的完成判据：四道闸门

「模型都能正确生成到内存里」要能被判定，否则它只是一句感觉。四道闸门，**全绿才算第一步做完**：

| 闸门 | 判据 | 现在的状态 |
|---|---|---|
| **D1 账平** | `Report::accounts_for(tree_total)` 对每个根都 `Ok` | 已有实现且在跑，保持 |
| **D2 每个 noun 有归属** | `unknown_nouns` 为空；每个出现过的 noun 在 `noun_evidence` 里有一条稳定依据 | **红**：ams7997 新露头 DDAT 137 / DDSE 76 / DPSE 8 / LNKI 65 / LNKS 65 |
| **D3 没出几何的都有名有姓** | 每一件都落进 `skipped / failed / centreline_only / catalogue_chain_broken / catalog_pending / unsupported / geometry_sets / proven_non_graphic` 之一，且**能分清「E3D 也不画」与「我们还没建」** | **红**：`catalog_pending` ≈ 1 万件（SCTN 2330 / GENSEC 4203 / FITT 1990 / FIXING 1179 / STWALL 302 / PFIT 263）语义是后者，混在一本账上 |
| **D4 出了几何的合法** | 每个 `Manifold` 非空、流形、体积 > 0、在世界系；负体要么已扣、要么进 `negatives_skipped` 点名 | 基本绿，`host_negatives`（aps250110 的 1559 件）是已知缺机制项 |

**D3 的账目要拆**——这是第一步最该先动的一刀，且不需要写任何几何：
把「记账不建」拆成语义分明的两本，`catalog_pending` 只留「E3D 也不画 / 无 GMRE」，
「有建模路径但还没接线」单开一本 `not_wired`。现在这两类混着，报告上永远分不出
「第一步还差多少」。拆完，D3 的红点就是一个可以逐周下降的数。

---

## 五、执行顺序（只为让 D1–D4 变绿排，不按几何难度排）

1. **拆 D3 的账**（半天，零几何）：`catalog_pending` → `catalog_pending` + `not_wired`；
   `totals_line` 带上新账。之后每次复跑，第一步的进度就是 `not_wired` 这个数。
2. **补 D2 的 5 个 noun**（半天）：DDAT / DDSE / DPSE / LNKI / LNKS 归 `NonGraphic`
   （`noun-family-matrix.json` 里三项皆空、`gb = 0`），加 `noun_coverage` 断言钉住。
3. **§3.2 的表示探针**（半天）：settle「设计侧 `-ref` 第二遍」是不是目录 OBST 的同一件事。
   这条排在接线之前，因为它可能改 `GeometryId` 的形状。
4. **按件数接线**，每接一条 `not_wired` 就少一批：SCTN(2330) → FITT 族(3432) → GENSEC(4203) →
   STWALL(302)。每条的施工图在 recheck-2 §2、§3。
5. **D4 的 `host_negatives`**（1559 件）：宿主在容器之外这条机制整条都没有，排在结构线之后。

**每一步的收尾都一样**：`accounts_for` 不破、`cargo test` 全绿且不放宽断言、
两库 `model.obj` 逐字节不变或逐件归因。

---

## 六、红线（违反了就是缺陷，不是风格问题）

1. **静默缺件是最高级别缺陷。** 少一件必须有一行账；账对不上必须红。
2. **判据只许写一份。** 全量与增量共用同一个函数。`is_model_unit` 与短 `DESP` 那两次
   都是同一个形状：判据写了两份，两边各自的账还都是平的。
3. **先贴反编译锚点，再写代码。** 新补的每条臂在注释里带上 Core3D/core.dll 地址。
4. **有意与 libgm 分歧要写进 doc-comment**，并说明为什么等价（如 >180° 回转不拆、
   NonZero 顶替自交翻绕向）。
5. **闸门先于几何**。表现级、level、`NAPP`、`OBST` 都在建体之前判，别先建网格再丢。
6. **`!(x > 0.0)` 而不是 `x <= 0.0`**：NaN 必须落进拒绝分支。

---

## 七、待裁决

1. §3.2 的表示探针出结论前，`GeometryId` **不加**表示维度——同意否？
2. D3 拆账用 `not_wired` 这个名字，还是并进现有 `evidence_pending`？（后者语义已被
   「builder 或语料证据未闭环」占着，混进去会看不出「纯粹没接线」那批。）
3. 第一步的验收要不要加一条「五库全量复跑一次、`not_wired` 归零」作为出口条件，
   还是允许结构线分批出口？

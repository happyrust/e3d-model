# e3d-model 开发计划（2026-09-04 傍晚 · Core3D.dll 复核第三版）

> 承接 `docs/2026-09-04-development-plan-core3d-recheck-2.md`（下称 recheck-2）、
> `docs/2026-09-04-generation-algorithm.md`（总纲）与 `docs/2026-09-04-step1-in-memory-plan.md`（第一步计划）。
> 本轮手段：读 HEAD `1fc0ce0` + **未提交工作树**（G1-1a 第 2+3 步已经写完在里面），
> 对活体 IDB 用 ida-bridge 反编译（`Core3D.dll` = `idalib-25128`、`core.dll` = `idalib-33060`）。
>
> **本文只写三件事**：① 已写的代码对着反编译原文核下来对不对；② recheck-2 里标「待核」的几条这轮核清了什么；
> ③ 据此改过的差距清单与顺序。recheck-2 已经写死、这轮没有改动的施工图不再重述。
>
> 复核规矩不变：以反编译原文为准，猜不出的写「待核」；每条差距带**锚点 + 验收判据**。
    10|> 属性 / noun 码反查公式：`hash = 27^4 + Σ (ord(c_i) − 64) · 27^i`（`i` 从 0 起、首字母最低位；≥5 字母同式加高位）。
> 本文新钉的码：`DELP 855256`、`PLOO 837961`、`HEIG 675926`、`PAVE 645937`、`CENT 935445`、`UCEN 810750`、
> `TOP 543530`、`UTOP 857865`、`PJOI 719809`、`PCOJ 739303`、`PALJ 737062`（后两个名义待核）。

---

## 一、账目快照：这一轮的起点

HEAD 从 recheck-2 的 `d440af3` 走到 **`1fc0ce0`**（多了 `fix(catalogue): preserve ATTA display components`）。
真正的变化在**未提交的工作树**里（并行会话已经落地，12 个文件 +728 / −52）：

| recheck-2 的项 | 工作树状态 |
|---|---|
    20|| G0-8 `route.rs` 只有 `GTYP=ANCI` 是站点 | 已改（`station_verdict` 三分支翻转 + PCLA 单测），**证据行仍缺**，未提交 |
| G1-1a 第 2 步 接线 | 已改：`category::is_model_unit` 的 `Catalog` 臂加了 `SCTN`，`elmodl::model_element` 有 SCTN 臂（`section_body`），增量侧同步 |
| G1-1a 第 3 步 两端切面 | 已写：`section_cut_planes` / `SectionCutPlane` / `SectionCuts::extrusion_range` / `solid::cut_by_plane`，带 4 条单测 |
| 第一步计划 W1 拆账 | 已改，但拆法与 W1 写的不同：不是 `not_wired`，而是 `SkipKind::{NoShape, CentrelineOnly, CatalogueChainBroken}` 三档 + `Report::centreline_only` / `catalogue_chain_broken` |
| 第一步计划 W6 分段计时 | 已加 `src/perf.rs`，`elmodl` / `catalogue` / `route` / `mesh` / `transform` 都插了 scope |
| 顺手做的性能收口 | `catalogue.rs`：`mesh_tri_counts` 免掉局部系多物化一遍、单图元不再走 `Manifold::compose` |

**还没跑过复跑**：五库全量没有在工作树这版上复跑，所以下面所有件数仍是 recheck-2 的数
（SCTN 2330 / GENSEC 4203 / FITT 1990 / FIXING 1179 / STWALL 302 / PFIT 263 ≈ 1 万件）。
**M0 的第一件事就是复跑刷新这张表**——SCTN 那 2330 件现在应该已经分成
`generated / centreline_only / catalogue_chain_broken` 三档，账没刷之前谁也不知道接线接住了多少。

    30|---

## 二、本轮 Core3D 复核的结论

### 2.1 `gml_bkend/XSEXTR@0x1034A591`：已写的延伸区间**逐项等价**（正面结论）

recheck-2 §2.2 把这个函数记成「SPRO 臂 `sub_1034A591`，同名逻辑待逐行核」，并按 XSPYR 推广了一版。
这轮逐行核过了，它的真名是 **`gml_bkend/XSEXTR`**，是**通用剖面**的端部延伸（SREC 走 `XSPYR@0x10348685`，
其余走它），全文如下：

```c
TRAVCI(A.pt, T, a5);  MVMLTI(A.n, T, a6);      // 端平面进局部系
TRAVCI(B.pt, T, a7);  MVMLTI(B.n, T, a8);
if (A.n.z < 0.0 || B.n.z > 0.0)  swap(A, B);   // ★ 三分量整体互换
    40|z_min = 0.0;  z_max = L;
for (每个剖面点 (x, y)) {
    if (fabs(A.n.z) > 1e-4)
        z_min = fmin(z_min, ((A.pt.x−x)·A.n.x + (A.pt.y−y)·A.n.y + A.pt.z·A.n.z) / A.n.z);
    if (fabs(B.n.z) > 1e-4)
        z_max = fmax(z_max, ((B.pt.x−x)·B.n.x + (B.pt.y−y)·B.n.y + B.pt.z·B.n.z) / B.n.z);
}
if (z_min < 0.0 || z_max > L)  L = z_max − z_min;
if (z_min < 0.0)  T.origin += T.zaxis · z_min;   // 沿自己的局部 Z 平移，不是改实体
```

    50|把交点式展开就是 `z = pt.z − (n.x·(x − pt.x) + n.y·(y − pt.y)) / n.z`——**与
`SectionCutPlane::axis_crossing` 一个字不差**；`z_min = min(0, …)` / `z_max = max(L, …)`、
新长度 `z_max − z_min`、以及沿轴平移 `z_min`，也都与 `extrusion_range` +
`extrude_rings(…).translate(0, 0, z_min)` 一一对上。**第 3 步的延伸这一半可以判定为对的**，
不必等 RVM 样本。三处细节差别：

1. **缺 ★ 那一行 swap。** 对 SCTN 不可达（GTSTDT 已经把 A 规约成 `n.z ≥ 0`、B 规约成 `n.z ≤ 0`），
   但**对 GENSEC 是必需的**——见 §2.2。
2. 平移那一步 Core3D 加了 `if (z_min < 0)` 的门，我们无条件 `translate(0, 0, z_min)`。
   `z_min == 0` 时是恒等变换，行为一致，只是多一次矩阵。**不用改。**
3. **迭代的点集不同**：XSEXTR 走的是剖面点表，我们走的是 `flatten_ring` **折线化之后**的环。
    60|   PRAD 倒圆后弧点落在尖角内侧，斜切端算出来的延伸量会**偏短**（短的量级 ≈ 倒圆矢高 × |n_xy / n_z|）。
   平头梁上恒为零，只有斜切 + 大倒圆同时出现才看得见。→ G1-1a-v 的量测项。

### 2.2 `dbboxlib/GTSTDT@0x1036CEC4`：SCTN 与 GENSEC 是**两套口径**，不能共用一个 `cut_plane_normal`

逐行核过的两个分支：

```c
// SCTN(821683) 或 WLJOIN(206501990)
GATPS1(POSS) → p;  TRAVCI(A.pt, T, p);      GATPS1(POSE) → p;  TRAVCI(B.pt, T, p);
DGETRA(DRNS, 3) → d;  MVMLTI(A.n, T, d);    DGETRA(DRNE, 3) → d;  MVMLTI(B.n, T, d);
if (fabs(A.n.z) < 0.001) A.n = (0,0,1);     if (A.n.z < 0.0) A.n = −A.n;
    70|if (fabs(B.n.z) < 0.001) B.n = (0,0,1);     if (B.n.z > 0.0) B.n = −B.n;

// GENSEC(46619692)
GATPS1(POSS) → A.pt;                        GATPS1(POSE) → B.pt;          // 不变换
GATDIR(DRNS, 缺省 (0,0,1)) → A.n;           GATDIR(DRNE, 缺省) → B.n;      // 不变换、不判轴向、不定号
```

SCTN 那一半与 `section_cut_planes` / `cut_plane_normal` 完全一致（含 0.001 轴向门与两端定号）。
**GENSEC 那一半没有任何规约**：标架恒等、法向照抄属性，`A.n.z` 完全可能是负的。
所以 GENSEC 的端面顺序**只由 XSEXTR 的 swap 决定**——W4-3 直接复用 `cut_plane_normal` 会把 GENSEC 的
    80|两端切反（表现是段两头各多切 / 少切一块，不报错）。GENSEC 侧要走的是「原样取 → 交给带 swap 的延伸器」。

顺带核清的两条：
- **非 SCTN / GENSEC 的 noun**：`IFCOMP(noun) || noun == PANE(640105)` → `PICAOR` + `PICOPS` 求标架；
  否则 `MUNIT` + 缺省轴。STWALL 属前者而 PICAOR 无臂 → 210，与 recheck-2 一致。
- **TMPL / TMAR 前置分支**（recheck-2 §2.4 记为「低优先级、语料待普查」）现在有全形了：
  DGOTO(`OWNER`)，属主 `TYPE == TMAR(886835)` 时在 TMAR 成员里找 `ORRF` 指回本 TMPL 的那一条；
  然后沿属主链**跳过 `PFIT(931840)` 与 `CMPF(661557)`**，最后 `CSTRAM(T, 属主refno, 模板refno)` 出变换。

### 2.3 FITT / CMFI 的标架与位置：`GFIORI` + **`GFIORG`**（G1-1c 施工图，可直接照写）

recheck-2 只找到 `GCORG3` 一个位置函数。核清了：**`GCORG3@0x103AA08C` 只是路由器**，按 noun 分到各自的 ORG：

    90|| noun | 标架（PICAOR 分到） | 位置（GCORG3 分到） |
|---|---|---|
| `SCTN` 821683 | `GSCORI@0x103A306C` + `ROTNAX@0x103A3398` | 同两个 + `GJLORI@0x10399B54`，`MVMULT` 后 **原点减去** 旋转后的偏移 |
| `GENSEC` 46619692 | **错 210** | **错 210** |
| `PJOI/SJOI/PCOJ/SCOJ/SUBJ` | `GJTORI@0x103AB5EC` + `GNAORI@0x103AFF74` | 同两个 + `GJTORG@0x103AA848` |
| `FITT` 939930 / `CMFI` 713316 | **`GFIORI@0x103B27D4`** | **`GFIORG@0x103B370C`** |
| `PFIT` 931840 / `COFI` 713370 / `CMPF` 661557 | **`GPFORI@0x103B2CE4`** | **`GPFORG@0x103B3C1C`**（另收 `PAVE` 645937） |
| `SBFI` 713035 | `GSFORI@0x103B30AC` | `GSFORG@0x103B41BC` |
| `PLOO` 837961 | —— | 原点 `(0,0,0)` |
| 其它 | 210 | `CLIMB(STRU)` + `GATPOS(POS)`，失败 210 + 原点 |

   100|**`GFIORI`（标架）**：

```
POSL(781909) 读成整数 → k     // 读不到 → 错 211
DGOTO(OWNER(10206636))
k ∈ {DRNS 916114, DRNE 640552}:                    // 挂在属主的端向上
    d = DGETRA(k, 3)                               // 读的就是 POSL 指名的那条方向属性
    GTNAID → 缺省 p-line id;  GSPLNU(属主, id) → (点, 主向 u, 副向 v, …)
    if k == DRNS: v = −v
    if dot(d, v) < 0: d = −d
    n = cross(u, v);  v' = cross(d, n);  归一化成功 → u = v'
否则:                                              // 挂在具名 p-line 上
    GTPLID(k) → id;  NULREF(id) → GTNAID 兜底
    GSPLNU(属主, id) → (…, d, u, …)
ORNMAT(d, u) → M                                   // 两轴定姿
θ = BANG(679457) / 5729.577951308232               // 存的是百分之一度
MTMULT(out, M, ROTMAT(θ))；out 的第三轴照抄 M       // ★ BANG 绕局部第三轴（Z）
```

   110|**`GFIORG`（位置）**：

```
POSL → k;  读不到 → 210
k ∈ {DRNS, DRNE}:  GFIPOS@0x103B60AC → 位置，**直接返回，不加 DELP**
否则:
    GFIPOS → 基点 p
    DGOTO(OWNER);  GSCDIR@0x103B0354(属主, −1) → 轴向 u
    GTPLID(k) → id;  NULREF → GTNAID 兜底；仍空 → 位置 = p
    否则 GSPLNU → p-line 的点 q 与方向 w
         t = dot(p, u)
         ISPLLN(q, w, u, t) → 参数 s        // 平面 u·x = t 与直线 q + s·w 求交；失败走 MRMESS 报错
         PTONLN(q, w, s) → 位置
两条路合流:
   120|    δ = DGETRA(DELP(855256), 3)                 // ★ 是 DELP，不是 POS
    GFIORI → M;  位置 += M · δ
```

→ **recheck-2 §2.4 里「FITT 的偏移取 `POS`」要改成 `DELP`。** 这是 1990 件 FITT 会不会整体错位的那一条。

### 2.4 PFIT / COFI / CMPF / PAVE：`GPFORI` + `GPFORG`

**`GPFORI`（标架）**——比 FITT 简单得多：

```
TYPE == COFI(713370):
    M = I;  o = OWNER;  DGOTO(PFRE(643156) 指向的元素)
   130|    递归 GPFORI → M₀;  o' = 该元素的 OWNER
    CSTRAM(S, o', o);  M = S · M₀
否则（PFIT / CMPF）:
    M = I
    SJUS(921016) ∈ {DTOP 857848, DCEN 810733, DBOT 936094} → M = diag(1, −1, −1)   // 绕 X 转 180°
    θ = BANG / 5729.577951308232;  MTMULT(out, M, ROTMAT(θ))；第三轴照抄
```

**`GPFORG`（位置）**：

```
t = TYPE;  r = REF(535968);  t == PAVE(645937) → r = OWNER
   140|t == COFI: 同上递归 + CSTRAM，位置 = S · 递归位置 + 平移
否则:
    t ∈ {PFIT, PAVE, CMPF} → 位置 = DGETRA(POS(545713), 3)
    t == PAVE → DGOTO(r)
    DGOTO(OWNER);  DFINDI(FIRST, TYPE, PLOO(837961))     // 属主下的第一个 PLOO
    h = DGETR(HEIG(675926))
    j₁ = DGETI(SJUS)：CENT 935445 / UCEN 810750 / DCEN 810733 → off = −h/2
                      TOP  543530 / UTOP 857865 / DTOP 857848 → off = −h
    DGOTO(r);  j₂ = DGETI(SJUS)：同表，符号相反（+h/2 / +h）
    位置.z = off                                          // ★ 覆盖 POS 的 z，不是叠加
```

   150|`PLOO`（P-Line lOOp）是这轮新露头的 noun：型材剖面下挂的环元素，`HEIG` 是它的高度，
PFIT 的轴向站位就是拿它的高度按两端 `SJUS` 折算出来的。**`DBOT / BOT / UBOT` 一档的偏移是 0**（表里没有臂）。

### 2.5 STWALL（302 件）重新定性：**Core3D 里一行代码都没有**

- 全映像扫 `STWALL = 179132626` 这个 4 字节立即数：Core3D 只在 `.data` 的 noun 家族表里出现
  （`0x10DA6454` 起连着 8 条，另有 `0x10DA6E3C / 0x10DA6E94 / 0x10DA6EB4 / 0x10DA6F10` 各 2 条，
  邻居是 `SCTN / GENSEC / SBWALL / FIXING / COMFIX / PJOI`），**`.text` 里一处都没有**；
  `core.dll` 里也只有 `.data` 一处（`0x6001BA4`）。
- 对照组：`GENSEC` 在 Core3D 的 `.text` 里有（`PICAOR` 与 `GCORG3` 的 `case 46619692`）。
  取 `DB_Noun` 要用它的码，码不进 `.text` 就注册不进 `addPlug@CSG_TreeBuilder@0x107158B0`
   160|  的 13 个注册点里的任何一个 → **Core3D 的现代 CSG 路没有 STWALL builder。**
- 唯一带 STWALL 的字符串在自检函数 `sub_10492B90` 里：
  **`TMPLATE 1 of FITTING 1 of STWALL 3 of FRMWORK /WALLS`**——STWALL 底下挂 FITTING、FITTING 底下挂 TMPLATE。

→ **STWALL 大概率不是「还没接线的建体单元」，而是容器**：它的 302 件几何应当来自成员上的
FITTING / TMPL，走的是 G1-1c 与 G2-1 已经要做的那两条路。这把第一步计划的 W4-4
从「1 天、放置来源未知、排最后」改成「**先花半天证伪，证实了就并进 W4-2 顺带清账，不单独立项**」。
证伪只要一只样本：`examples/element_probe` 取 3 只 STWALL，看它有没有 `SPRE`、成员都是什么 noun。

### 2.6 勘误（recheck-2 的锚点表要改这几处）

   170|| recheck-2 写的 | 实际 |
|---|---|
| `SANN sub_10349627` | `sub_10349627` = **`gml_bkend/CRRSLD`**（Create Ruled Solid），不是 SANN 臂。SANN 的臂待重找 |
| `SPRO sub_1034A591`（待核） | **`gml_bkend/XSEXTR`**，通用剖面端部延伸，见 §2.1 |
| FITT 位置偏移取 `POS` | 取 **`DELP` 855256**，见 §2.3 |
| 只有 `GCORG3` 一个位置函数 | `GCORG3` 是路由器；FITT/PFIT/SBFI/关节各有 `GFIORG` / `GPFORG` / `GSFORG` / `GJTORG` |
| `GPFORI` 读「`PFRE`、`SJUS`、`BANG`、`DCEN/DTOP/DBOT`、`CSTRAM`」 | 标架只读 `SJUS` + `BANG`（COFI 才用 `PFRE` + `CSTRAM`）；`DCEN/DTOP/DBOT` 是 `SJUS` 的**取值**不是属性；`HEIG` + `PLOO` 在 `GPFORG` 那边 |
| STWALL「放置从哪来待核，先看缺省型 47」 | Core3D 无 STWALL 代码，见 §2.5 |

---

## 三、已写代码的复核意见（工作树，逐条带锚点）

   180|按「照着反编译能判定的」列，**不含**需要 RVM 才能裁决的。

### R1（P0，便宜）`section_cut_planes` 把端点硬写成 `(0,0,0)` / `(0,0,L)`

GTSTDT 是 `TRAVCI(T⁻¹, POSS)` / `TRAVCI(T⁻¹, POSE)` **真算**的，我们直接写死。
等价的前提有三条：`section_placement` 的原点恰在 `POSS`、局部 +Z 恰指向 `POSE`、`length == |POSE − POSS|`。
这三条现在没有任何一条被检查。**标架错了的表现不是报错，是体积对不上**——而体积对不上会先被赖到端切上。

- 改法：`section_cut_planes` 里真算一次 `T⁻¹ · POSS` / `T⁻¹ · POSE`，与 `(0,0,0)` / `(0,0,L)` 比，
  超过 1e-6 就 `notes.push` 一条（不改行为、不 fail）。
- 验收：五库复跑，这条 note 一条都不出；出了就是标架 bug，顺着 refno 查。

   190|### R2（P0，随 W4-3）XSEXTR 的 swap 没有实现

对 SCTN 不可达，**对 GENSEC 必需**（§2.2）。现在写在 `SectionCuts` 上的语义是「A 已朝 +Z、B 已朝 −Z」，
GENSEC 复用它就会切反。

- 改法：把 swap 放进 `SectionCuts` 的构造（`if a.n.z < 0 || b.n.z > 0 { swap }`），
  `section_cut_planes`（SCTN 路）照旧先规约、swap 恒不触发；GENSEC 路原样取属性直接构造。
- 验收：单测——两端法向都反着给，`extrusion_range` 与规约过的一致。

### R3（P1，先量再改）`SectionCutPlane::is_axial()` 用精确 `== 0.0`

`DB_makeCSG@0x10738C30` 那三处判据（伪码第 99 / 116 / 134 行）确实也是精确比较
（`*(double *)&v13 == 0.0 && *((double *)&v51 + 1) == 0.0`），**所以判据本身不是分歧**。
   200|差别在代价：libgm 拿一张解析平面切，空操作就是空操作；我们要按包围盒造一只方块做 `intersection`，
端盖共面布尔既费时、又可能在端面上留一层重合面（`a_flat_ended_beam_is_a_plain_extrusion`
这条单测断言 `num_tri == 12`，真机上一旦 `MVMLTI` 留下 1e-17 残差就会破）。

- 先量，别先改：复跑时把「`|n.x|` 或 `|n.y|` 非零但 < 1e-9」的 SCTN 件数、以及这些件切前切后的面数差记一行。
- 量出来普遍存在 → 给 `is_axial` 加一道 `hypot(n.x, n.y) <= 1e-9` 的门，并在 doc-comment 里写明
  「与 Core3D 的精确比较有意分歧，因为两边切面的代价不同」（总纲红线 4）。
- 量出来是零 → 一个字都不动，把这次量测写进 `evidence/`。

### R4（P2）折线化顶点参与延伸区间（§2.1 第 3 点）

斜切 + 大 `PRAD` 同时出现时延伸偏短。**先不改**——现在一个斜切样本都还没对拍过。
   210|记在这里，等 G1-1a 的 RVM 裁决时如果出现斜切件对不上，第一个查它。

### R5（P1）第一步计划 W1 的拆账名字与实现对不上了

W1 写的是 `catalog_pending` → `catalog_pending` + `not_wired`；工作树落的是
`SkipKind::{NoShape, CentrelineOnly, CatalogueChainBroken}` + 两本新账。两者解决的**不是同一个问题**：
- 已落地的这版分的是「**已经接线、但这一件没出几何**」的三种原因；
- W1 要的是「**这条 noun 整条还没接线**」与「E3D 也不画」的分野——`catalog_pending` 里那 1 万件
  现在还是混着的（FITT/GENSEC/FIXING/PFIT 是前者，真·无 GMRE 的是后者）。

→ 两本账都要，别互相顶替。`not_wired` 这一刀仍然欠着，且**它才是第一步进度的那个数**。

---
   220|
## 四、差距清单（改过顺序，替代 recheck-2 §三与第一步计划 §1 的 W4 表）

### P0 —— 先把账刷出来（做完才知道后面还欠多少）

- **N0-1 工作树复跑五库**。SCTN 那 2330 件按 `generated / centreline_only / catalogue_chain_broken` 落档；
  ams7997 管身基线刷新（recheck-2 G0-2 ③ 至今没做，那 4787 还是共轴门之前的数）。
  验收：`accounts_for` 不破；`cargo test` 全绿；两库其它 noun 的 `model.obj` 逐字节不变。
- **N0-2 R1 的标架自检**（半天，不改行为）。
- **N0-3 补 `not_wired`（= 原 W1 未完的那一半）**（半天，零几何）。
- **N0-4 G0-8 的证据行 + 提交**：哪条支管上 `PCLA` 被当站点拆过管，改后 E3D `TUBE` 数对上。
   230|- **N0-5 D2 的 5 个 noun**（DDAT / DDSE / DPSE / LNKI / LNKS → `NonGraphic` + `noun_coverage` 断言）。

### P1 —— 结构线，按「施工图完备度」重排

recheck-2 与第一步计划都按件数排（SCTN → FITT 族 → GENSEC → STWALL）。这轮把 STWALL 摘出去、
把 FITT 族的施工图补全之后，顺序改成：

1. **N1-1 FITT / CMFI（1990 件）** —— 施工图这轮已完备（§2.3）。目录路复用
   `model_catalogue_member_with_cache`，新活只有 `GFIORI` + `GFIORG` 两个函数，写进 `section.rs` 旁边的 `fitting.rs`。
   **`G2-5 place_cached_part` 收口排在它前面。**
   验收：两库 FITT 的 `catalog_pending` 归零或只剩点名例外；`DELP` 那条偏移有单测钉住（§2.3 ★）。
2. **N1-2 PFIT / COFI / CMPF / PAVE（263 件）** —— 施工图这轮已完备（§2.4），比 FITT 简单，紧跟着做。
   注意 `位置.z = off` 是**覆盖**不是叠加，以及 `PLOO` 的 `HEIG` 要能取到。
   240|3. **N1-3 SCTN 第 4 步负体（2330 件的端部精度）** —— 关节 `JOIS`/`JOIE`（mode 2）+ 成员链
   `FITT / CMFI(+SBFI) / SNOD→SJOI / SCOJ→SUBJ`，统一过 `NAPP & 2` 门。依赖 N1-1 的目录路。
   在它落地之前，现在生成的 SCTN 是**没扣关节与配件的毛坯**，端部偏胖——RVM 对拍要按这个预期读。
4. **N1-4 GENSEC（4203 件）** —— 件数最大但没有参考样本，且要先做 R2 的 swap。按 recheck-2 §2.5 的段建法。
5. **N1-5 FIXING（1179 件）** —— 几何源是它 `TMPL` 里的图元（recheck-2 §2.4 第 3 点），
   标架走 §2.2 新核出来的 TMPL/TMAR + `CSTRAM` 那条。与 G2-1 一起做。
6. **N1-6 STWALL（302 件）** —— **降级为半天的证伪项**（§2.5），排在最后，很可能不需要单独接线。

### P2 —— 收口

- **N2-1 `place_cached_part` 收口**（原 G2-5，`catalogue.rs` 现 15 处样板）——排在 N1-1 之前。
- **N2-2 `NAPP` 三档账目**（原 G2-1）：`template_holes_applied / neighbour_cuts_applied / hole_definitions`。
   250|- **N2-3 几何选项开关**（原 G2-9）：`byte_10D99C00` / `byte_10E5072C` / `byte_10E5072D` 对应的 `REPRE` 关键字**仍未核清**，
  先留 `E3D_REPRE_*` 环境变量占位、缺省全开（模式 0）。
- **N2-4 表示探针**（总纲 §3.2 / 第一步计划 W3）：设计侧 `-ref` 第二遍是不是目录 OBST 的同一件事。
  **这一条的排位要拍板**——它可能改 `GeometryId` 的形状，理应排在所有接线之前；但它卡在人工 TTY 导出上。
- **N2-5 R3 的量测**、**N2-6 `host_negatives` 1559 件**（原 W5）、**N2-7 SANN 臂重找**（§2.6）。

---

## 五、里程碑

| 里程碑 | 内容 | 净工期 |
|---|---|---|
| **M0** | N0-1 ~ N0-5（复跑刷账 + 标架自检 + `not_wired` + G0-8 收口 + 5 个 noun） | 1.5 天 |
   260|| **M1** | N2-1 收口 → N1-1 FITT/CMFI → N1-2 PFIT 族 | 3 天 |
| **M2** | N1-3 SCTN 负体 → R2 swap → N1-4 GENSEC | 3 天 |
| **M3** | N1-5 FIXING + N2-2 NAPP 三档；N1-6 STWALL 证伪 | 2 天 |
| **M4** | N2-4 表示探针裁决、N2-6 host_negatives、四门全绿复核 | 2 天 |

M0 做完，「第一步还差多少」才第一次变成报告上一个真数。在那之前，上面所有件数都是 09-02 的旧账。

---

## 六、贯穿约束（沿用，不重述理由）

1. `Report::accounts_for` 硬判据不破。
2. `cargo test` 全绿，且不许为了变绿放宽断言。
   270|3. 触碰复用池 / 求值器的改动，两库 `model.obj` / `elements.json` 逐字节一致或逐件归因。
4. 只读库身份（DB SHA-256）前后一致。
5. 新补的每条臂**先贴 Core3D 反编译锚点，再写代码**。
6. 判据只许写一份，全量与增量共用同一个函数。
7. 与 libgm 的**有意**分歧（如 R3）写进 doc-comment 并说明为什么可接受。

---

## 七、请拍板

1. **N1 的新顺序**（FITT → PFIT → SCTN 负体 → GENSEC → FIXING → STWALL）取代旧的按件数排——同意否？
   理由是施工图完备度：FITT / PFIT 这轮已经能照着写，GENSEC 还欠 swap 与段建法的对齐，STWALL 很可能是伪需求。
   280|2. **STWALL 降级为半天证伪项**（§2.5）——同意否？还是仍按 302 件的接线项排？
3. **R3 先量后改**（不动 `is_axial`，先数残差）——同意否？还是直接加容差门省一轮复跑？
4. **N2-4 表示探针**排在 M4 还是提到 M0？它可能改 `GeometryId` 的形状，但卡在人工 TTY 导出。
5. **`not_wired` 这本账**仍然要（§R5），还是就认工作树已落的三档、把「整条没接线」并进 `catalog_pending` 的 notes？
6. **人工依赖**：ams8000 管道区 / ams7997 支撑区 / ams1112 含 FIXING · PFIT 的 RR001 · WF 区的参考侧 RVM 全量导出，本周能否排上？
   N1-1 / N1-2 的验收卡在这里。

---

## 八、本轮新增锚点表（Core3D.dll = `idalib-25128`）

   290|| 用途 | 地址 / 码 |
|---|---|
| 端部延伸（通用剖面） | **`gml_bkend/XSEXTR@0x1034A591`**（SREC 走 `XSPYR@0x10348685`） |
| 端平面来源 | `dbboxlib/GTSTDT@0x1036CEC4`：SCTN/WLJOIN 规约分支、GENSEC 原样分支、`IFCOMP‖PANE → PICAOR+PICOPS` 分支、TMPL/TMAR 前置分支 |
| 端面跳过判据 | `DB_Sctn::DB_makeCSG@0x10738C30` 伪码第 99 / 116 / 134 行（精确 `== 0.0`） |
| 标架路由 | `pllib/PICAOR@0x103A59EC`（SCTN / 关节 / FITT·CMFI / PFIT·COFI·CMPF / SBFI，其余 210） |
| 位置路由 | `pllib/GCORG3@0x103AA08C`（同上 + `PLOO` 原点 + 缺省 `CLIMB(STRU)`+`GATPOS`） |
| FITT / CMFI | 标架 **`pllib/GFIORI@0x103B27D4`**、位置 **`pllib/GFIORG@0x103B370C`**；用到 `GFIPOS@0x103B60AC`、`GSCDIR@0x103B0354`、`GTPLID@0x103A89BC`、`GTNAID@0x103AC154`、`GSPLNU@0x1039C5C8`、`ISPLLN` / `PTONLN` / `ORNMAT` / `ROTMAT` |
| PFIT 族 | 标架 **`pllib/GPFORI@0x103B2CE4`**、位置 **`pllib/GPFORG@0x103B3C1C`**（`CSTRAM` 递归、`PLOO` + `HEIG` + 两端 `SJUS`） |
| SBFI / 关节 | `GSFORI@0x103B30AC` / `GSFORG@0x103B41BC`；`GJTORI@0x103AB5EC` / `GNAORI@0x103AFF74` / `GJTORG@0x103AA848`；SCTN 位置另用 `GJLORI@0x10399B54` |
   300|| CSG 注册表 | `addPlug@CSG_TreeBuilder@0x107158B0`，13 个注册点（`CSG_PrimitiveUtilities::initialise@0x10727540` 注 19 条最多）；STWALL 不在其中 |
| STWALL 证据 | `.data` 家族表 `0x10DA6454`（×8）/ `0x10DA6E3C` / `0x10DA6E94` / `0x10DA6EB4` / `0x10DA6F10`；`.text` 零处；自检串 `TMPLATE 1 of FITTING 1 of STWALL 3 of FRMWORK /WALLS`@`sub_10492B90` |
| 本轮新钉的码 | `DELP 855256` · `PLOO 837961` · `HEIG 675926` · `PAVE 645937` · `CENT 935445` · `UCEN 810750` · `TOP 543530` · `UTOP 857865` · `PJOI 719809` · `PCOJ 739303` · `PALJ 737062`（名义待核）· `COMFIX 349816161` · `SBWALL 179132140` |

> 运维备注：`Core3D.dll` = `idalib-25128`、`core.dll` = `idalib-33060`（本轮只用来验 STWALL 无代码）；
> exec 代码里仍**不要给 `idb` 这个名字赋值**。本轮反编译原文没有落盘，要留证据时按上表地址重跑。

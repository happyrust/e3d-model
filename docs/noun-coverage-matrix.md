# NOUN 覆盖矩阵终态表

> 计划：`gen-model/.planning/2026-08-31-noun-coverage-closure/task_plan.md` 完成判据最后一条。
> 本表**不是手写台账**，每个数字都由下面那两条命令机械导出，改了分类表就重跑。

| 项 | 值 |
|---|---|
| 字典 | `E:\reverse\e3d\shadow_e3d31_aps_all\attlib.dat`（Everything3D 3.1，5 840 896 bytes） |
| 字典里的 noun | 1931（其中 **1498 个不带任何 `I*COM` 家族位**） |
| 家族位字段号来源 | `scripts/icom_field_ids.py` 从 `core.dll` 的 22 个 `I*COM` 谓词逐个反编译解出，19 个解到字段号 |
| 家族矩阵 | `data/noun-family-matrix.json`，SHA-256 `13851F8024E6D4F4339F38FD453F2FB50E45999D07255D2E5A3E839048548EEF` |
| 快照时间 | 2026-08-31 18:2x（会话 T88W 复算，与落盘文件逐字节一致） |

## 一、总账

```
GTGEOM(0x10341d2e) 谓词级联判定「该出几何」的 noun        327   ← 覆盖率的分母
├─ 会建几何                                               30   9.2%
├─ 记账不建（有类别、有出处，本期不产几何）              297  90.8%
└─ 分类表欠账（Unknown，无名目）                           0   0.0%
```

分母只算 11 个**级联内**家族。另外 8 个家族（含船体 `IHLCOM` 96 个 noun）走别的模块，
不属这条几何路，落在 `unknown` 里也不算本线欠账——分母算错了覆盖率就没有意义。

## 二、19 个家族逐个终态

判据取自 `Category::build_disposition()`，四态含义见 §六。

### 级联内 11 族（327）

| 家族 | 含义 | noun | 已建 | 记账待建 | 无三维表示 | 欠账 | 终态说明 |
|---|---|---:|---:|---:|---:|---:|---|
| `IPCOMP` | 管件 | 74 | 0 | 74 | 0 | 0 | `RouteMember` 62 + `Composite(PipeComponent)` 12；几何压算法线 P4 |
| `IPFCOM` | 板墙楼板 / 建筑装配 | 69 | 4 | 62 | 3 | 0 | 已建 `FLOOR`/`GWALL`/`PANE`/`SCREED`；`Composite(Structural)` 58 待建；`CTRAY` 记在 `ICABCO` 账上 |
| `IFCOMP` | 结构 / 型材件 | 40 | 0 | 24 | 16 | 0 | `Composite(BuildingAssembly)` 17；`List` 5 靠 DFS 下钻等效 |
| `IECOMP` | 设备图元 | 32 | 17 | 15 | 0 | 0 | 覆盖率最高的一族；含 `Composite(CoreClass28)` 3 个 |
| `IHCOMP` | 吊架件 | 30 | 0 | 30 | 0 | 0 | 全 `RouteMember`，与 `IPCOMP` 同一条路由线 |
| `ICABCO` | 电缆桥架 | 21 | 0 | 20 | 1 | 0 | `Composite(CableTray)` 20（含 `CTRAY`）；`RNODE` 判无三维表示 |
| `IGMCOM` | GM 几何集 | 21 | 0 | 21 | 0 | 0 | 全 `GeometrySet`，**本期只记账不建**（语料里一件都没有） |
| `INGCOM` | 负几何集 | 16 | 0 | 16 | 0 | 0 | 全 `NegGeometrySet`，同上 |
| `INCOMP` | 设计负图元 | 13 | 9 | 4 | 0 | 0 | 9 个负体与正体成对可建；4 个 `NegUnimplemented` 与正体成对不建 |
| `IG2COM` | G2 几何集 | 7 | 0 | 7 | 0 | 0 | 全 `GeometrySet` |
| `ICCOMP` | 目录件 | 6 | 0 | 6 | 0 | 0 | 全 `Composite(Catalogue)` |

> 逐族相加是 329 而不是 327：`CTRAY` 与 `STWELD` 各带两个级联家族位，被数了两次。
> 分母 327 是**去重后**的 noun 数，两者都对，别拿一个去质疑另一个。

### 级联外 8 族（不在本线）

| 家族 | noun | 终态 |
|---|---:|---|
| `IHLCOM` | 96 | **不在本线**：船体族（`CPANEL`/`CPLATE`/`CHOLE`…）不走 GTGEOM，走船体模块 |
| `IPTCOM` | 9 | **不在本线**：`DPCA DPCY DPLI DPSP PTAX PTCA PTMI PTPOS PTSE`，目录 P 点定义 |
| `IPLCOM` | 2 | **不在本线**：`PLIN`、`PTSS` |
| `ICOCOM` | 1 | **不在本线**：`SCOM`（同时带 `ICCOMP`，已在级联内记账） |
| `IFICOM` | 1 | **不在本线**：`SFIT`（同上） |
| `IJOCOM` | 1 | **不在本线**：`JOIN`（同上） |
| `IPRCOM` | 1 | **不在本线**：`SPRF`（同上） |
| `IHVCOM` | 0 | ⚠ **本轮判不了，不是「无几何」**——见下 |

**`IHVCOM` 必须单独说清**：它的字段号 `591821`（从 `core.dll` 解出，来源无疑）
在 **E3D 2.10 和 3.1 两版字典的 `field_index` 里都不存在**，探针会打印
`⚠ 这些字段号在本版字典里不存在，对应家族本轮判不了：[591821]`。
所以「命中 0」的含义是**读不到这一位**，不是「没有 noun 属于 HVAC 家族」。
把它记成「无几何」就是拿读不到当读到了。`HVAC` noun 自身判 `Category::List`
（纯组织容器，`hard base` 是 `PIPE`），那是另一条独立证据，与本族判不判得了无关。

## 三、会建几何的 30 个

| 家族 | noun |
|---|---|
| `IECOMP`（17） | `ABOX ACTOR ACYLI AEXTR APYRA ARTOR ASNOU BOX CTOR CYLI EXTR POHE POLYHE PYRA REVO RTOR SNOU` |
| `INCOMP`（9） | `NBOX NCTO NCYL NPOLYH NPYR NREV NRTO NSNO NXTR` |
| `IPFCOM`（4） | `FLOOR GWALL PANE SCREED` |

`A*` 族的 5 个（`ABOX ACYLI APYRA ARTOR ACTOR`）是按属性表逐项同名同义接进正体 builder 的，
复算脚本 `scripts/a_family_attr_probe.py`；`AEXTR` 早已认领。
`SNOU`/`NSNO`/`ASNOU` 三个是 2026-09-01 随 `GM_Snout` builder（偏心圆锥台）一并接入的，
RVM 样本 `24381/130759,130763` 钉外形与 8 段 facet——已建因此 27 → 30。

## 四、记账不建的 297 个，按类别

| 类别 | 数量 | 这类账代表什么 |
|---|---:|---|
| `RouteMember` | 92 | 路由成员，几何在算法线 P4 |
| `Composite(Structural)` | 58 | Core 复合构件，走成员聚合 builder |
| `GeometrySet` | 28 | 几何集，本期只记账（`IGMCOM` 21 + `IG2COM` 7） |
| `Composite(CableTray)` | 20 | 电缆桥架族 |
| `Composite(BuildingAssembly)` | 17 | 建筑装配族 |
| `NegGeometrySet` | 16 | 几何集负体 |
| `Composite(PipeComponent)` | 12 | 管件复合 |
| `NonGraphic` | 12 | 无独立三维 builder 路径（基准 / 节点 / 接头） |
| `Unsupported` | 8 | 认识它、知道它有几何，本期没实现 |
| `Catalog` | 9 | 硬依赖目录求值 |
| `Composite(Catalogue)` | 6 | 目录复合件 |
| `NegUnimplemented` | 4 | 与未实现的正体成对不建 |
| `List` | 5 | 纯组织容器，靠 DFS 下钻等效（`CWALL CFLOOR CMPF FRMW SBFR`） |
| `Composite(Equipment)` | 4 | 设备复合 |
| `ProfileData` | 3 | 属主消费的剖面数据（`PAVE PLOO SPINE`） |
| `Composite(CoreClass28)` | 3 | `NOZZ`/`ELCONN`/`EQUCOM`，见 §五 |

## 五、每条终态的出处

`dump_categories` 给每个 noun 附一条 `evidence`。级联内 327 条的出处分布：

| 条数 | 出处 |
|---:|---|
| 117 | `E3D 3.1 attlib family bit plus Core3D GTGEOM cascade; builder implementation pending` |
| 92 | `E3D 3.1 noun_layout structural attributes; Core route semantics` |
| 65 | `classified modelling route; implementation or corpus evidence pending` |
| 30 | `implemented Rust builder with geometry regression tests` |
| 12 | `E3D 3.1 dictionary has no independent 3D builder path` |
| 8 | `E3D 3.1 hierarchy/member-consumption semantics` |
| 3 | `Core3D GTGEOM 0x10341d2e -> sub_10714FC0; sub_107189A0 class 28; sub_107210A0 member builder chain` |

最后 3 条是 `NOZZ`/`ELCONN`/`EQUCOM`。`sub_107189A0` 把这三个 noun 码映射为 class 28
（`IPCOMP`/`IHCOMP` 映射为 29/30），走成员聚合。**三者现已全部拿到实跑证据**
（2026-08-31，靶库见 §八）：

| noun | 件数 | 语料 |
|---|---:|---|
| `EQUCOM` | 438 | `aps250209_0001` |
| `NOZZ` | 50 | `aps250164_0001` |
| `ELCONN` | 8 + 34 | `aps250164_0001` + `ams8000_0001` |

三者在真库里**全是叶子元素、一个成员都没有**，所以 `sub_107210A0` 那条成员聚合链
在手上所有语料里都无物可聚。这正是它们停在 `EvidencePending` 而没有冒充已建的理由，
也说明这样停**没有丢任何几何**。

## 六、四种终态的确切含义（别混着说）

`Category::build_disposition()` 给出，与「类别」正交——**有类别不等于有模型**：

| 终态 | 含义 | 对应计划里的说法 |
|---|---|---|
| `Built` | 本期真的会产出几何（正实体或扣得出的负体） | 已建 |
| `EvidencePending` | 有分类、有 builder 路，但实现或语料证据还没到位 | 记账待建 |
| `ProvenNonGraphic` | 字典里没有独立三维 builder 路径（`NonGraphic`/`List`/`ProfileData`） | 有意不做 |
| `OutOfCorpus` | 落 `Unknown`，连名目都没有 | 欠账（当前为 0） |

这一层是专门为了防止「把 297 个记账数报成覆盖率」而立的。当前**几何覆盖率是 30/327 = 9.2%**，
不是 100%；100% 的只有「每个 noun 都有名目和出处」这件事。

## 七、版本差异明细（E3D 2.10 → 3.1）

两版矩阵由**同一个探针、同一套字段号**生成，唯一变量是字典，所以下面每一条差异都只能来自字典本身。
（`primitive`/`geomset`/`extrusion`/`gb` 取自 `noun_flags.json`，两次同一份，不参与 diff。）

```
2.10   noun 1384   级联 277        3.1   noun 1931   级联 327
```

### 7.1 noun 集合

新增 **548** 个；消失 **1** 个（`noun_hash 25899336`，`noun_flags.json` 里没有对应短名）；两版都有 1383 个。

### 7.2 分母 277 → 327 的完整拆解

| | 数量 | 说明 |
|---|---:|---|
| A. 3.1 新增 noun 且直接落在级联内 | 49 | 见下表 |
| B. 两版都有、3.1 里新加上级联家族位 | 1 | `TATTA`：`[]` → `[ICABCO]` |
| C. **退出级联（反向变化）** | **0** | 没有任何 noun 在 3.1 里掉出级联 |
| 合计 | +50 | 49 + 1 − 0 = 50 ✓ |

A 的 49 个按家族（`STWELD` 同时带 `IPFCOM`+`IFCOMP`，故逐族相加是 50）：

| 家族 | 新增 | noun |
|---|---:|---|
| `IPFCOM` | 17 | `IBNDRY ICLIP ICTOUT IEDGE IFCPLA IHOLE ILIM INODE INOTCH IPILLR IPLATE ISEAM ISTIFF ISTRU ISTRUR SDAFIX STWELD` |
| `IPCOMP` | 16 | `HVBRCO HVFLAN HVHACC HVIDAM HVSADD HVSKIR HVSPLR HVSTIF HVTPPO PEXSP PGBOX PIACT PIBLK PIGEN PWCHA PWMAN` |
| `IFCOMP` | 8 | `CNVCOM IPANE ISECT KSUSVE MFIX STCSET STWELD SUBCNV` |
| `ICABCO` | 4 | `BUNDLE CTCOUP JNODE PULLN` |
| `IG2COM` | 2 | `JWELD JWSET` |
| `ICCOMP` | 2 | `JWVERT WDET` |
| `INCOMP` | 1 | `DTHOLE` |

**这 50 个全部已被分类，无一落 `Unknown`**，终态清一色 `EvidencePending`：
`Composite(Structural)` 17、`Composite(PipeComponent)` 9、`Composite(BuildingAssembly)` 7、
`RouteMember` 7、`Composite(CableTray)` 5、`Composite(Catalogue)` 2、`GeometrySet` 2、
`NegUnimplemented` 1。

两条值得单独记的：

- 那 16 个新进 `IPCOMP` 的里有 **9 个 `HV*`**（`HVBRCO HVFLAN HVHACC HVIDAM HVSADD HVSKIR
  HVSPLR HVSTIF HVTPPO`），正是 Core3D 现代 CSG 已挂 plug 的那批 HVAC 部件。
  **HVAC 的几何是经 `IPCOMP` 分派的，与读不到的 `IHVCOM` 无关**——§二 那条「`IHVCOM` 判不了」
  因此不影响 HVAC 部件本身进入级联。
- 另外 7 个（`PEXSP PGBOX PIACT PIBLK PIGEN PWCHA PWMAN`）正是 §3.2 交叉验证里
  「`route-nouns.json` 有、2.10 字典没有」的那 7 个。换 3.1 后它们归位到 `IPCOMP`，
  那条残留就此闭合。

### 7.3 两版都有、家族位集合变了的 noun（共 3 个，全是「加」，无「减」）

| noun | 变化 | 现判 |
|---|---|---|
| `CTRAY` | `+ICABCO`（原只有 `IPFCOM`） | `Composite(CableTray)`（本轮改判，见下） |
| `TATTA` | `+ICABCO`（原无任何家族位） | `Composite(CableTray)` |
| `FEMODL` | `+IHLCOM`（级联外） | `Unknown`（船体线，不算本线欠账） |

**`CTRAY` 是这次 diff 查出的唯一一处记账不自洽，已改判。** `classify()` 的家族兜底是
一条有序 `if` 链，原先 `IPFCOM` 排在 `ICABCO` 前面，于是同时带两个家族位的 `CTRAY`
——全字典唯一这样的 noun——落进了 `Structural`，而同族的 `CTSTRA`/`CTBEND`/`CWAY`/
`CWBRAN`/`TATTA`/`HATTA`/`RATTA` 全是 `Composite(CableTray)`。桥架主干和它的弯头、
直段分在两本账上，「桥架这一族做完没有」就没法回答，而这正是本表要回答的问题。
2.10 下不存在这个矛盾（`CTRAY` 当时只有 `IPFCOM`），是**换 3.1 之后才出现的**；
计划 §3.1 也把 `CTRAY` 归在电缆桥架族。

改法是把 `ICABCO` 提到 `IPFCOM` 之前：电缆桥架是一条自带容器与成员的独立专业线，
带这个位就按桥架记账。**不是「谁的族小谁优先」**——`IFCOMP`/`IECOMP` 仍排在 `IPFCOM`
之后，同时带 `IFCOMP`+`IPFCOM` 的 `STWELD` 照旧归 `Structural`（结构焊缝，本就该在结构账上）。

要紧的是这次改判**不改变任何几何结论**：`core.dll` 里 `IPFCOM` 与 `ICABCO` 两个谓词
都进同一个 `sub_10714FC0`，走哪个位进去都是同一条复合几何路。变的只有记账口径，
四数（1931 / 327 / 30 / 297 / 0，按当前基线引）一位没动，位移只发生在 §四 的类别分布里：
`Composite(Structural)` 59→58、`Composite(CableTray)` 19→20。回归由
`category::tests::the_cable_tray_family_stays_together` 钉住（正面钉桥架五件整族在一起，
反面钉 `STWELD` 仍归结构）。

### 7.4 逐族命中数

| 家族 | | 2.10 | 3.1 | Δ |
|---|---|---:|---:|---:|
| `IPFCOM` | 级联内 | 52 | 69 | +17 |
| `IPCOMP` | 级联内 | 58 | 74 | +16 |
| `IFCOMP` | 级联内 | 32 | 40 | +8 |
| `ICABCO` | 级联内 | 15 | 21 | +6 |
| `IG2COM` | 级联内 | 5 | 7 | +2 |
| `ICCOMP` | 级联内 | 4 | 6 | +2 |
| `INCOMP` | 级联内 | 12 | 13 | +1 |
| `IHCOMP` `IECOMP` `IGMCOM` `INGCOM` | 级联内 | 30 / 32 / 21 / 16 | 同 | = |
| `IHLCOM` | 级联外 | 85 | 96 | +11 |
| `ICOCOM` `IFICOM` `IHVCOM` `IJOCOM` `IPLCOM` `IPRCOM` `IPTCOM` | 级联外 | 1/1/0/1/2/1/9 | 同 | = |

### 7.5 正负配对

`positive_equivalent` 字段**两版零变化**——正负体配对关系在 2.10 与 3.1 之间完全稳定，
`negative_primitives_and_their_positive_twins_agree` 这条回归不受换版本影响。

### 7.6 复算这张差异表

```powershell
python scripts/noun_family_probe.py "D:\AVEVA\Everything3D2.10\attlib.dat" --version "Everything3D 2.10" --out <旧矩阵.json>
python ../../gen-model/.scratch/t88w_matrix_diff.py <旧矩阵.json> data/noun-family-matrix.json
```

两版矩阵都可精确复现：2.10 重算得 198 332 bytes / 277 个级联，3.1 重算与落盘文件 SHA-256 逐字节一致。

## 八、APS 工厂语料实跑（2026-08-31）：给 class 28 补证据

`ams1112`/`ams8000` 都出自 `AvevaMarineSample`，设备族本就稀，`NOZZ` 与 `EQUCOM`
一件都没有。改到工厂样例 `AvevaPlantSample`（`aps000` 下 240 个库），用
`examples/tree_census` 逐库清点找靶（脚本 `gen-model/.scratch/t88w_find_nozz.ps1`，
234 个候选全扫 63 秒），选定两个库：

| 库 | 元素 | 总账 | class 28 落账 |
|---|---:|---|---|
| `aps250164_0001` | 4 071 | `visited=1048 consumed=3023 generated=905 failed=0 orphans=0`，账平 | `NOZZ 50`、`ELCONN 8` |
| `aps250209_0001` | 20 994 | `visited=7361 consumed=13633 generated=1801 failed=0`，账平 | `EQUCOM 438` |

三个 noun 全部落在 `evidence_pending` 且带 class 28 出处串，**没有一个进 `unknown_nouns`**；
`aps250209` 的 DFS 可达数是 `索引 20993 / DFS 20993`，一个没漏。

### 8.1 三者在真库里全是叶子

`tree_census` 的父子表（已确认未截断）里，`NOZZ`/`EQUCOM`/`ELCONN` 一次都没有作为父出现：

```
EQUI → SUBE 40 / NOZZ 15     SUBE → NOZZ 35     SUBE → ELCONN 8      （aps250164）
SUBE → EQUCOM 438                                                     （aps250209）
```

所以 `sub_107210A0` 那条成员聚合链在手上所有语料里都无物可聚。把三者停在
`EvidencePending` 而不硬做一个聚合 builder，**没有丢任何几何**。

### 8.2 新语料暴露的 14 个未知 noun —— 全在级联外

| 库 | `unknown_nouns` |
|---|---|
| `aps250164` | `STALNK 8`、`STLNKS 1`、`STLNKW 1` |
| `aps250209` | `VVALUE 603`、`POINTR 352`、`DDAT 224`、`RPATH 99`、`HANDRA 81`、`DDSE 21`、`DPCA 19`、`DPSE 18`、`TMPL 14`、`CONVEY 6`、`TMRPSE 1` |

逐个查家族位（`gen-model/.scratch/t88w_unknown_probe.py`）：13 个**不带任何家族位**，
`DPCA` 带的是级联外的 `IPTCOM`。级联内判 `Unknown` 的 noun 仍是 **0** 个，
四数基线 `1931 / 327 / 30 / 297 / 0` 不受影响，这些是 `OutOfCorpus` 终态。

⚠ **「级联外」不等于「没几何」**，这和 §二 里 `IHVCOM` 那条「读不到 ≠ 没有」是同一类错。
`HANDRA`（扶手）、`CONVEY`（输送机）看着就是有形的东西，只是不走 GTGEOM 这条线。
它们在本矩阵范围之外，但**需要单独定性**，不能拿「级联外」当「已收口」。
→ 定性已完成，见 **§九**；结论是这 14 个里确实有 1 个真欠账（`HANDRA`）、
1 个是图元容器（`TMPL`，已在 **§十一** 接进分类表），**而且这 14 个不是全集**。

### 8.3 17 个孤儿负体，根因是属主 `CONE` 本期没实现

`aps250209` 报 `orphans = {NCON 3, NCYL 12, NPYR 2}`，对上父子表正好：

```
CONE → NCYL 12    CONE → NCON 3    CONE → NPYR 2      合计 17
```

`CONE` 判 `Unsupported`（该库 24 件），属主没 builder，负体自然没人消费。
这不是分类缺陷，是孤儿账正常发挥作用 —— 它指出「将来实现 `CONE` 时这 17 个负体
必须一起接进去」。漏了的表现是该有洞的地方没洞，几何静悄悄地错。

## 九、级联外 noun 的单独定性（2026-08-31）

§8.2 那 14 个「级联外未知 noun」逐个定性的结果。取证走三路：
AVEVA 官方元素类型说明（`Aveva.Core.Database.xml`）、`core.dll` / `Core3D.dll`
活体反编译、真库父子结构 + 实跑账。

### 9.1 先说方法上的一个坑：立即数搜索必须搜 `Core3D.dll`

想用「noun 码有没有被硬编码进代码」来判有无几何时，**模块要选对**：

- `core.dll` 只装 `nounlib`（22 个 `I*COM` 谓词本体，`0x5xxxxxx` 段）；
- **几何分派逻辑整个在 `Core3D.dll`**（`GTGEOM` 在 `0x10341d2e`）。

只搜 `core.dll` 时，对照组 `PANE`（实跑建出 4275 件）、`CTRAY`、`ELCONN`、
`EQUCOM` **全部零命中**——手段没有分辨力。改搜 `Core3D.dll` 后对照组
9 个全中（`GENSEC` 195、`PANE` 114、`NOZZ` 76、`SPINE` 51、`ELCONN` 16、
`EQUCOM` 13、`CTRAY` 12、`CYLI` 5、`BOX` 4），零命中这才能当证据用。

**做这类判定一定要带一组已知答案的对照**，否则会拿「零命中」给有几何的 noun 结案。

### 9.2 两处反编译结论（AVEVA 自己的代码说的）

**`IASLCO`（`core.dll` `0x561b994`）是「扶手／平台总成族」谓词**，
全文就是一张写死的 10 个 noun 名单，不读字典：

```
STRFLT  PLTFRM  HRPOST  HRTERM  HRGATE  HRPANE   ← 带 IPFCOM，在级联内，判 Composite(Structural)
HANDRA  STRLNG  KICKPL  RAIL                     ← 无家族位，级联外，判 Unknown
```

同一张名单被字典的 `IPFCOM` 位劈成两半，**分界线没有语义**。
`IASLCO` 本身不在 `GTGEOM` 的级联里（`GTGEOM` 调的是 `IPCOMP`/`IHCOMP` →
`IFCOMP`/`IPFCOM`/`IECOMP`/`ICABCO`/`INCOMP` → `IG2COM`/`IGMCOM`/`INGCOM`/`ICCOMP`
→ `JSPCL`/`CRDESI`），但调 `IASLCO` 的 `sub_1025277E` 同时也调 `GTGEOM`，
是遍历总成成员再逐个出几何的上层驱动。

**`LPRMTV`（`core.dll` `0x561ce28`）明说「属主是 `TMPL` 的元素就算图元」**：

```c
primitive = ATNLOG(noun, 字段 659518 /* primitive */)
if (primitive && noun != WORL) return true
ownerType = <属主的类型>
return ownerType == TMPL && noun ∉ {DDSE, DPSE, PORS, LNKS}
```

一次给三个结论：`TMPL` 是几何容器（子图元该建）；`DDSE`/`DPSE` 即使挂在 `TMPL`
下也**显式不算图元**（非图形数据有了硬出处）；`TMPL` 自己不出形。

### 9.3 逐个结论

| noun | 官方说明 | 定性 | 关键出处 |
|---|---|---|---|
| `HANDRA` | Handrail | **有几何、走别的线（真欠账）** | `IASLCO` 名单与 `HRPOST`/`HRPANE`/`HRGATE` 同族；`sub_10089BDC`（`GTGEOM` 调用者）名单里与 `RLADDR`/`SLADDR` 并列 |
| `TMPL` | Template | **图元容器**（已接为 `PrimitiveContainer`，见 §十一） | `LPRMTV`；真库 `HRPOST → TMPL → 图元`；见 §9.4 与 §十一 |
| `RPATH` | Route path | 几何载体，无独立实体 | `HANDRA → RPATH → POINTR` 是扶手扫掠路径 |
| `POINTR` | Point on path | 几何载体，无独立实体 | `RPATH → POINTR`；`core.dll` `PPEVST`（P 点求值） |
| `CONVEY` | Conveyor | 组织容器，无独立几何 | `CONVEY → STRU / EQUI / DDSE`，自己不挂图元；两模块零命中 → 同 `HVAC`/`PIPE`，应归 `List` |
| `DDSE` | Design dataset | 非图形数据 | `LPRMTV` 显式排除；`DDSE → DDAT → VVALUE` 属性树 |
| `DDAT` | Design data | 非图形数据 | 同上 |
| `VVALUE` | Valid value | 非图形数据 | `DDAT → VVALUE` 叶子；两模块零命中 |
| `DPSE` | Design Ppoint/Pline set | 非图形数据（连接点集） | `LPRMTV` 显式排除 |
| `DPCA` | Cartesian design Ppoint | 非图形数据（连接点） | 家族位是级联外 `IPTCOM`；命中集中在 `ORIEN`/`STRAP` 等定位函数 |
| `TMRPSE` | Repeat set | 非图形数据 | `TMPL → TMRPSE → TMRREL`；两模块零命中 |
| `STALNK` | Status Link element | 非图形数据 | `STLNKW → STLNKS → STALNK`，独立挂 `WORL` 下 |
| `STLNKS` | Status Link elements | 非图形数据 | 同上；两模块零命中 |
| `STLNKW` | Status Link world | 非图形数据 | 同上；两模块零命中 |

### 9.4 定性过程查出的两笔账（实跑 `aps250110_0001`）

选带整套扶手立柱的库（`HRPOST 124` / `RAIL 8` / `KICKPL 5` / `HANDRA 11`）实跑
`out/aps250110-handrail`，账仍平（`visited 16122 + consumed 10065 = 索引 26187`）。

**（一）`TMPL` 的 1559 件负体无人认领**

```
TMPL → NCYL 1282    TMPL → NBOX 232    TMPL → NXTR 45
```

`TMPL` 底下正体（`CYLI 617` / `PANE 385` / `EXTR 291` / `CTOR 85` / `REVO 10` /
`BOX 8`）和负体是**并列挂着**的。`TMPL` 判 `Unknown` 不消费成员，于是正体被当
独立元素照常建出、**负体全部落进孤儿账**（连带 `LOOP 45` / `VERT 185` / `CURVE 1`，
共 1790 件）。

> **本节初稿的结论「该有洞的地方没有洞」已被实测推翻，见 §十一。**
> 那批负体扣的不是 `TMPL` 自己的正体成员，而是容器之外的宿主（墙 / 梁）。
> 真正的问题不是「孔没开在立柱上」，而是这 1559 件落在了一本**名字不对**的账上。

**（二）「级联外未知 noun」不止 14 个**

同一个库新冒出 9 个：`POSTSE HRKPSE HRPNSE HRFEAT`（扶手的四个集容器）、
`RAIL KICKPL`（横杆、踢脚板，都是实体件）、`RLCAGE CAGSEG HOOPSE`（梯子护笼）、
`LDRSTR STRSTR`（梯/楼梯结构串）、`DPSP TMRREL`。

§8.2 的「14 个」只是前两个语料碰到的那些，**不是全集**；要拿全集得按库遍历。

四数基线 `1931 / 327 / 30 / 297 / 0` 不受本节影响（这些 noun 全在级联外）。

## 十、复算

```powershell
cd vendor/e3d-model
# 换 E3D 版本时才需要重跑探针（会重写 data/noun-family-matrix.json）
python scripts/noun_family_probe.py "<attlib.dat>" --version "<版本名>"
# 日常复算：判词直连 category.rs 的 classify()，不存在会过期的影子表
cargo run --bin dump_categories -- data/noun-family-matrix.json out/noun-categories.json
python scripts/noun_coverage_report.py
cargo test --test noun_coverage
```

`tests/noun_coverage.rs` 的 7 条回归钉住本表：

| 测试 | 钉住什么 |
|---|---|
| `cascade_coverage_matches_the_recorded_baseline` | 分母 327 / 已建 30 / 记账 297 / 欠账 0 |
| `every_e3d31_cascade_family_has_a_modelling_route` | 欠账的家族分布为空 |
| `every_noun_named_in_the_classifier_actually_exists_in_the_dictionary` | 分类表里的 noun 名都在字典里查得到（`POGON` 那类静默失配） |
| `negative_primitives_and_their_positive_twins_agree` | 正负体走同一条几何路径 |
| `the_n1_defects_are_closed` | N1 三条缺陷修完的样子 |
| `the_n2_class28_fix_is_closed_and_spine_is_still_evidence_pending` | class 28 已关、`SPINE` 仍待证 |
| `a_noun_the_traversal_can_prune_is_never_a_model_unit` | 会被属主剪掉的 noun 不得同时是模型单元（**新类别必过这条**，见 §11.4） |

**改了覆盖面这些断言就会红**，那时要连同本表一起更新，而不是把断言放宽。

## 十一、`TMPL` 接为图元容器，以及一条被实测推翻的改法

### 11.1 原本要修的那笔账

§9.4（一）的结论是「`TMPL` 判 `Unknown` 不消费成员 → 1559 件负体成孤儿 →
该开的孔没开」，据此的改法是：让 `TMPL` 消费成员，负体扣到它的正体兄弟上。
判据看着很硬——`TMPL 2013286654/25291`（`DESC` = "Flexible End Plate - 150mmx8mm"）
的成员表是

```
DPSE  DDSE  NBOX  NCYL  NCYL  PANE  TMRPSE  NCYL  NCYL
```

一块 `PANE` 端板配 4 个 `NCYL`（`DIAM 22`，Z 向贯穿板厚）和 1 个 `NBOX`，
负体在正体前后都有。逐件量下来，孔的 XY 落在板轮廓内、Z 正好穿透板厚，
怎么看都是「照着这块板配的孔」。

### 11.2 实跑把它证伪了

照那条规则实现后在 `aps250110` 重跑：记账全部命中预测
（`unknown` 8779 → 8409、`orphans` 1790 → 1、账仍平），**但几何塌了**——
`generated` 1541 → 1386，`skipped` 0 → **155**，`REVO` 从 10 件掉到 0 件。

155 件全部来自 5 个 `TMPL`，每个整整 31 件，`DESC` 都是
**"Fire Hose Wheel and Housing"**（嵌在内墙里的消防水带箱）：

| 负体 | 尺寸 | 实际是什么 |
|---|---|---|
| `NXTR` | 环 1000 × 450，`HEIG` 700 | **墙上的凹龛** |
| `NBOX` | 1124 × 824 × 100，偏出 1292mm | 墙另一侧的开口 |

**水带箱正是装在那个凹龛里的**，拿凹龛去扣箱子，箱子一件不剩。
端板那一例同样翻案：新规则下 `PANE` 只剩 14782 mm³（原 ≈132000）、`genus = -1`
两个分离分量——那块 `NBOX` 的 `133.2 × 203.2` 是**英标 203×133 工字梁的截面**，
是给梁让位的梁廓开口，不是这块板上的洞。

### 11.3 结论：`TMPL` 的负体是宿主负体

> `TMPL` 底下的负体扣的是**容器之外的宿主**（墙 / 梁 / 被连接的构件），
> 不是 `TMPL` 自己的正体成员。

E3D 这条「构件在别处、洞开在宿主上」的路，本仓的负几何模型里整条都没有——
现有 `elmodl::collect_negatives` 只有「属主向下」一条圈定。补它要先解决
「怎么找到宿主」，目前唯一的线索是 `NAPP`：三件已查明扣外部的负体
`NAPP` 全是 `4294967295`，唯一那件螺栓孔 `NCYL` 是 `7`。样本太少，还不能当判据。

### 11.4 落地的改动

| 改动 | 说明 |
|---|---|
| 新类别 `Category::PrimitiveContainer` | 与 `RouteContainer` 同构：自己不出几何，成员才是内容。出处是 `LPRMTV`「属主是 `TMPL` 即算图元」 |
| `TMPL` 改判为它 | 370 件从 `Unknown` 下账；`consumes_members()` 为真，但「消费」只表示**认领**，一刀不扣 |
| 新账 `Report::host_negatives` | 1559 件负体从 `orphans`（语义「数据或分类有问题」）转进一本名字对的账，也不许无声混进 `consumed` |
| `collect_negatives` 保持只向下 | 模块注释里写死证否过程，防止有人照「负体是兄弟」的直觉再改一次 |

`PrimitiveContainer` 的 `consumes_members()` 为真意味着它的负体成员会被
`pipeline::push_members` **整棵剪掉**，因此必须满足 §十 那条
`a_noun_the_traversal_can_prune_is_never_a_model_unit`：`is_model_unit()`
对它返回假，全量剪掉的东西增量那侧也不会当单元发 Remove / Regen。

**三个语料实跑，几何逐件零变化**（按 `refno|noun|ordinal` 比体积 / AABB /
局部 AABB / 三角数 / 亏格 / 世界矩阵 / 负体数）：

| 语料 | 元素数 | 不变量有差异的 | `unknown` | `orphans` | `host_negatives` |
|---|---:|---:|---:|---:|---|
| `aps250110` | 1541 → 1541 | **0** | 8779 → **8409** | 1790 → **1** | `NCYL 1282 / NBOX 232 / NXTR 45` |
| `ams1112` | 4613 → 4613 | **0** | 0 | 0 | — |
| `ams8000` | 153 → 153 | **0** | 0 | 0 | — |

基本体实例缓存也逐项不变（`hits 678 / misses 9 / baked_operands 23 / unique_meshes 9`）。
四数基线 `1931 / 327 / 30 / 297 / 0` 不受影响：`TMPL` 在级联外。

### 11.5 留下的口子

宿主负体本身**还没有落地机制**：这 1559 件目前只是被记住，一件都没扣。
要真扣，得先能从负体定位到宿主（`NAPP`？连接关系？），那是 `TMPL` 之外的独立课题。

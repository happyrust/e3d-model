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
├─ 会建几何                                               27   8.3%
├─ 记账不建（有类别、有出处，本期不产几何）              300  91.7%
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
| `IECOMP` | 设备图元 | 32 | 15 | 17 | 0 | 0 | 覆盖率最高的一族；含 `Composite(CoreClass28)` 3 个 |
| `IHCOMP` | 吊架件 | 30 | 0 | 30 | 0 | 0 | 全 `RouteMember`，与 `IPCOMP` 同一条路由线 |
| `ICABCO` | 电缆桥架 | 21 | 0 | 20 | 1 | 0 | `Composite(CableTray)` 20（含 `CTRAY`）；`RNODE` 判无三维表示 |
| `IGMCOM` | GM 几何集 | 21 | 0 | 21 | 0 | 0 | 全 `GeometrySet`，**本期只记账不建**（语料里一件都没有） |
| `INGCOM` | 负几何集 | 16 | 0 | 16 | 0 | 0 | 全 `NegGeometrySet`，同上 |
| `INCOMP` | 设计负图元 | 13 | 8 | 5 | 0 | 0 | 8 个负体与正体成对可建；5 个 `NegUnimplemented` 与正体成对不建 |
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

## 三、会建几何的 27 个

| 家族 | noun |
|---|---|
| `IECOMP`（15） | `ABOX ACTOR ACYLI AEXTR APYRA ARTOR BOX CTOR CYLI EXTR POHE POLYHE PYRA REVO RTOR` |
| `INCOMP`（8） | `NBOX NCTO NCYL NPOLYH NPYR NREV NRTO NXTR` |
| `IPFCOM`（4） | `FLOOR GWALL PANE SCREED` |

`A*` 族的 5 个（`ABOX ACYLI APYRA ARTOR ACTOR`）是按属性表逐项同名同义接进正体 builder 的，
复算脚本 `scripts/a_family_attr_probe.py`；`AEXTR` 早已认领。

## 四、记账不建的 300 个，按类别

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
| `Unsupported` | 10 | 认识它、知道它有几何，本期没实现 |
| `Catalog` | 9 | 硬依赖目录求值 |
| `Composite(Catalogue)` | 6 | 目录复合件 |
| `NegUnimplemented` | 5 | 与未实现的正体成对不建 |
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
| 68 | `classified modelling route; implementation or corpus evidence pending` |
| 27 | `implemented Rust builder with geometry regression tests` |
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

这一层是专门为了防止「把 300 个记账数报成覆盖率」而立的。当前**几何覆盖率是 27/327 = 8.3%**，
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
四数（1931 / 327 / 27 / 300 / 0）一位没动，位移只发生在 §四 的类别分布里：
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
四数基线 `1931 / 327 / 27 / 300 / 0` 不受影响，这些是 `OutOfCorpus` 终态。

⚠ **「级联外」不等于「没几何」**，这和 §二 里 `IHVCOM` 那条「读不到 ≠ 没有」是同一类错。
`HANDRA`（扶手）、`CONVEY`（输送机）看着就是有形的东西，只是不走 GTGEOM 这条线。
它们在本矩阵范围之外，但**需要单独定性**，不能拿「级联外」当「已收口」。

### 8.3 17 个孤儿负体，根因是属主 `CONE` 本期没实现

`aps250209` 报 `orphans = {NCON 3, NCYL 12, NPYR 2}`，对上父子表正好：

```
CONE → NCYL 12    CONE → NCON 3    CONE → NPYR 2      合计 17
```

`CONE` 判 `Unsupported`（该库 24 件），属主没 builder，负体自然没人消费。
这不是分类缺陷，是孤儿账正常发挥作用 —— 它指出「将来实现 `CONE` 时这 17 个负体
必须一起接进去」。漏了的表现是该有洞的地方没洞，几何静悄悄地错。

## 九、复算

```powershell
cd vendor/e3d-model
# 换 E3D 版本时才需要重跑探针（会重写 data/noun-family-matrix.json）
python scripts/noun_family_probe.py "<attlib.dat>" --version "<版本名>"
# 日常复算：判词直连 category.rs 的 classify()，不存在会过期的影子表
cargo run --bin dump_categories -- data/noun-family-matrix.json out/noun-categories.json
python scripts/noun_coverage_report.py
cargo test --test noun_coverage
```

`tests/noun_coverage.rs` 的 6 条回归钉住本表：分母 327、已建 27、记账 300、欠账 0、
欠账家族分布为空、分类表里每个 noun 名都在字典里查得到。**改了覆盖面这些断言就会红**，
那时要连同本表一起更新，而不是把断言放宽。

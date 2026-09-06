# e3d-model 开发计划（2026-09-04 · 生成侧导出 RVM，与 E3D TTY RVM 直接对拍）

> 承接 `docs/2026-09-04-development-plan-core3d-recheck-3.md`、
> gen-model `docs/2026-08-04_rvm-baseline-verification-plan.md`（基准对拍总纲）与
> gen-model `docs/evidence/2026-09-03-db8000-bran-production-increment-tty-rvm.md` §3（现行 RVM 真值链）。
>
> **要做的事**：`gen_ams` 生成完模型之后，把生成物按 **AVEVA RVM 二进制格式**写出一份 `.rvm`，
> 使「E3D TTY 导出的 RVM」与「我们生成的 RVM」可以走**同一个读法、同一套判据**逐件对拍。
>
> **本轮手段**：① 读 e3d-model / gen-model 工作树；② 对 `test_data/rvm/` 下三份真实 E3D 导出物
> 做字节级实测（探针 `.scratch/rvm_chunk_probe.py` / `rvm_facet_probe.py` / `rvm_bbox_frame_probe.py`）；
> ③ ida-bridge 反编译 `Core3D.dll` / `core.dll` 复核写出端。
>
> **规矩**：格式的每一条都要有实测字节或反编译原文兜底；只靠推理得出的写「待核」。
>
> **2026-09-06 补记**：M4 之后又落了两件——M6「自校闸按网格自己的弦高定尺」（`73ebb5e`）与
> M7「Snout 识别 + 半圈弯头」（`19d1488`）。M4 正文里的数是它当轮（2026-09-05）的实测，
> **保留原样不改**；当天工作树上的复测数放在 §五 M7 末尾，两处对不上的地方那里逐条说明了成因。

---

## 一、为什么要做：现在的判据链是**不对称**的

```
基准侧   E3D TTY  →  a.rvm  →  rvm_verify import  →  a.rvm.json   ┐
                            └→  rvm_facets_dump   →  facets.json  ├→ rvm_compare
生成侧   gen_ams           →  elements.json                       ┘
```

两侧走的是两条完全不同的路径，代价都是实际踩过的（出处见 `src/bin/rvm_compare.rs` 模块文档）：

| # | 现象 | 现在的对策 | 代价 |
|---|---|---|---|
| 1 | 快照里的 `aabb_world_mm` 与同一条 PRIM 记录的面片顶点对不上（最大差 5.4 m），20 件顶点级全等的 GWALL 全部误判 `off` | 必须另跑一遍 `rvm_facets_dump` 出 `facets.json`，`rvm_compare` 才有可信基准 AABB | 多一个工具、多一个中间文件；缺 facet 的成员静默退回不可信字段（`aabb_source` 记账） |
| 2 | rvm-rs 的图元三角化曾用 Y-up 局部系，RVM 记录是 Z-up | `rvm_facets_dump` 写了 `y_up_to_record_frame` + `matches_record_bbox` 一整套**自适应探测** | 一个上游 bug 在我们这边长出 60 行补偿代码 |
| 3 | 两侧数据结构不同 | 只比 AABB 不变量 | 顶点级、拓扑级、几何种类全都比不了；「不比体积」是被迫的，不是选的 |
| 4 | 加一项判据 | 基准侧改 import/dump，生成侧改 `elements.json` schema，对拍侧改配对 | 三处同时改，schema 漂移就是静默错配 |

**导出 RVM 之后链路变成对称的**：

```
基准侧   E3D TTY  →  a.rvm  ┐
                            ├→  同一个 reader  →  同一份中间结构  →  rvm_compare
生成侧   gen_ams  →  b.rvm  ┘
```

四条代价一次性消掉：基准 AABB 与生成 AABB 用**同一段代码**从**同一种记录**里算出来，
不存在「一边可信一边不可信」；帧/单位换算只发生一次、在一个地方；判据加一项只改一处。

顺带得到两件本来要另立项目的东西：

- **生成物可以直接进 Navisworks / AVEVA Review** 等吃 RVM 的工具做目验，不必再走 OBJ。
- **写出端是读出端的判据**：能写出一份 E3D 自己读得回去的 RVM，等于把我们对格式的理解钉死了一次。

---

## 二、格式实测：施工图的地基

以下每一条都来自 `test_data/rvm/` 里**六份真实 E3D 3.1 导出物**的字节，不是从文档抄的。
图元普查（`.scratch/rvm_chunk_probe.py`）：

| 样本 | 图元构成 |
|---|---|
| `C-OR-1R345-C.rvm`（5 280 B） | Box 7 · FacetGroup 2 |
| `C-IY-1R330-B.rvm`（84 480 B） | Box 51 · FacetGroup 90 |
| `1RX07-LCT.rvm`（54 560 B） | Box 109 · FacetGroup 28 |
| `1RCV0101-B1.rvm`（17 600 B） | Cylinder 15 · Snout 6 · CircularTorus 5 · FacetGroup 3 · Box 3 |
| `1RS-WF03-W-C-RR001.rvm`（616 000 B） | FacetGroup 113 · Line 336 |
| `1RX03-MSUP-E0110.rvm`（12 320 B） | Line 41 |
| **合计** | **FacetGroup 236 · Box 170 · Line 377 · Cylinder 15 · Snout 6 · CircularTorus 5** |

> **样本集在本轮工作期间从 6 份长到 8 份**（另一条会话加了 `KX-CAM-S4001.rvm` 与
> `SCTN-HW150-C80.rvm`，共 14 条记录：FacetGroup 12 + Box 2）。
> 下面每张表都标了它是在几份上跑的。**新加的两份是独立加进来的，而下面每一条结论在它们上面照样成立**
> ——常量字段全中、`bbox_frame` 的 `unknown` 仍是 0，等于白捡了一次交叉验证。
> 全集现状：8 份 · CNTB/CNTE 各 609 · PRIM 823 · 判帧 446 条（local 216 / rotated 230 / unknown 0）。

`Line`（377 条）是 CENTRELINE，不是实体——生成侧不产出它，对拍时整族豁免
（`examples/rvm_facets_dump.rs:248` 已经这么处理）。

### 2.1 chunk 骨架

大端到底。每个 chunk = **16 字节 tag**（4 个字符各占一个 big-endian u32）+ **u32 绝对下一块偏移**，然后是负载。

`C-OR-1R345-C.rvm` 的完整骨架（`.scratch/rvm_chunk_probe.py` 输出，节选）：

```
0x000000  HEAD  next=0x00010c  v=1 n=2 strings=['AVEVA E3D Design Design Mk3.1.9.0[Z3190-108]  (WINDOWS-NT 0.0)  (7  Mar 2025 : 06:12)',
                                                'RVM baseline for BRAN /C-OR-1R345-C (dbnum 8000, head-tube ground truth)',
                                                'Wed Aug 12 17:11:31 2026', 'dpc@DESKTOP-NGJ2UUD', 'Unicode UTF-8']
0x00010c  MODL  next=0x000138  v=1 n=1 name='AMS' project='/ALL'
0x000138  CNTB  next=0x000174  v=1 n=2 name='/1RX03-EQUI'    translation=(0,0,0)                 material=1
0x000174    CNTB  ...          name='/1RX03-LCT'             translation=(0,0,0)                 material=1
0x0001b0      CNTB  ...        name='/1RX-345'               translation=(0,0,0)                 material=1
0x0001e8        CNTB  ...      name='/C-OR-1R345-C'          translation=(0,0,0)                 material=1
0x000228          CNTB  ...    name='FTUBE 1 of BRANCH /C-OR-1R345-C'  translation=(8550.54,14060.42,2900.00)  material=1
0x000278            PRIM  v=1 flags=1 kind=2(Box)        T=(8.9651,13.7808,2.9150)  bbox=(-500,-25,-15)..(500,25,15)  params=[1000,50,30]
0x0002ec          CNTE
0x000308          CNTB  ...    name='BEND 1 of BRANCH /C-OR-1R345-C'   translation=(9380.70,13500.47,2900.00)
0x000358            PRIM  v=1 flags=1 kind=11(FacetGroup)  T=(9.3807,13.5005,2.9000)  bbox=(-15.39,-21.58,0)..(16.50,22.36,30)
...
0x000f48  COLR  next=0x000f6c  payload=16B  color=0x82828200
0x000f6c  END:  next=0x000f88  payload=8B
0x000f88  <16 zero bytes> -> stop

chunk counts: {'HEAD':1, 'MODL':1, 'CNTB':13, 'PRIM':9, 'CNTE':13, 'COLR':1, 'END:':1}
```

**逐字段（实测）**：

**逐字段，附全部八份样本的出现次数**（常量项 100% 一致，没有一条例外）：

| chunk | 负载 | 实测 |
|---|---|---|
| `HEAD` | `u32 v`、`u32 n`、5 段**字串**：banner / filenote / date / user / encoding | `v=1 n=2` ×8 |
| `MODL` | `u32 v`、`u32 n`、2 段字串：model（`AMS`）、project（`/ALL`） | `v=1 n=1` ×8 |
| `CNTB` | `u32 v`、`u32 n`、1 段字串（组名）、`3×f32` 平移、`u32 material` | `v=1 n=2 material=1` ×609 |
| `CNTE` | 8 字节 | `00000001 00000002` ×609（与 CNTB 数相等，配平） |
| `COLR` | 16 字节：`u32 ×3` + `u32 0xRRGGBB00`（读侧 `& 0x00FFFFFF`） | `1,1,1, 0x82828200` ×8 |
| `END:` | tag 第 4 字是 `':'`(0x3a)，负载 8 字节 | `00000001 00000001` ×8 |
| `PRIM`/`OBST`/`INSU` | `u32 v`、`u32 flags`、`u32 kind`、**列主序 3×4 矩阵 12×f32**（m00 m10 m20 m01 m11 m21 m02 m12 m22 m03 m13 m23）、`6×f32` bbox、随后按 kind 的参数 | `PRIM v=1 flags=1` ×823；**`OBST`/`INSU` 一条都没有** |

> `CNTE` 的负载是 `1, 2` 不是 `1, 1`——`END:` 才是 `1, 1`。两个长得像，抄错了写出来的文件
> 读侧照样解得开（`parse_cnte` 根本不看负载），要等第三方工具才炸。

> 六份样本**全是 `PRIM`，没有 `OBST` / `INSU`**。其中 `C-IY-1R330-B` 是 gen-model
> `docs/2026-08-04_rvm-baseline-verification-plan.md` §6.1 记的**宽口径**（`repre obst on`）导的，
> 障碍体与预留净空照样写成普通 `PRIM`——这与那份文档「RVM 流里的预留体就是个普通 box，
> `GeometryType` 分不出来」的复核订正一致。**`repre obst on/off` 控制的是导不导，不是导成哪个 tag。**

**字串编码**：`u32 字数`（4 字节一字）+ 该字数的字节，NUL 补齐。
`'AMS'` → `words=1` + `41 4D 53 00`。

**尾部补零**：`END:` 之后文件补零到某个块边界。八份样本**全部**是 160 字节的整数倍
（`3976→5280`、`11124→12320`、`16804→17600`、`53008→54560`、`83660→84480`、`615812→616000`），
块大小的来源待核。读侧靠「连续 16 个 0 字节」停，补不补都能读回。

### 2.2 ★ 单位是**三套**，不是一套

这是全篇最容易写错、也最容易在对拍里表现成「几何全体偏了 1000 倍」的一条。

以 `BEND 1` 那条 FacetGroup 记录为例（`.scratch/rvm_facet_probe.py`）：

```
matrix cols   = [0.000829, -0.000559, 0.0,  0.000559, 0.000829, 0.0,  0.0, 0.0, 0.001]
matrix transl = [9.3807, 13.50047, 2.9]
bbox          = [-15.3939, -21.5815, 0.0, 16.5023, 22.3575, 30.0]
num_polygons  = 6
  poly0 cont0 v0: pos=[0.6939, 26.5, 30.0]  nrm=[0.0, 1.0, 0.0]
```

`|col0| = sqrt(0.000829² + 0.000559²) = 0.001`。**矩阵的 3×3 部分是 `R · 0.001`**，
即 mm→m 的换算被烘进了旋转里；平移是**米**；而顶点、bbox、图元参数（`Box params=[1000,50,30]`）
全是**毫米**。

| 字段 | 单位 | 说明 |
|---|---|---|
| `CNTB` 平移 | **mm** | 绝对世界坐标（见 §2.4） |
| `PRIM` 矩阵 3×3 | 无量纲 × 0.001 | `R · 0.001`，把局部 mm 送进世界 m |
| `PRIM` 矩阵平移 | **m** | 世界坐标 |
| `PRIM` bbox | **mm** | 帧见 §2.3 |
| kind 参数（Box lengths / Cylinder radius…） | **mm** | |
| FacetGroup 顶点 | **mm** | 局部系 |

于是 `世界 mm = (M3 · v_local + t) × 1000`，与 `examples/rvm_facets_dump.rs` 里
`(cols[..] · v + cols[9..]) * M_TO_MM` 的算法一致——那段代码是对的，只是当时把它记成了
「RVM 存米」，实际是「顶点存 mm、矩阵带 0.001、平移存 m」。

### 2.3 ★★ PRIM 记录 bbox 有**两个帧**，而且不是按 kind 分的

`rvm_compare` 的模块文档现在写着：

> `.rvm` 的 PRIM 记录自带的 bbox 与**同一条记录里的面片顶点**对不上（最大差 5.4m）……
> 拿它当判据，20 件顶点级全等的墙全部误判 `off`。

**观察是对的，结论要改**：bbox 没有错，它只是**不一定在顶点所在的那个帧里**。

`.scratch/rvm_bbox_frame_probe.py` 拿每条 FacetGroup 记录**自己的顶点**算候选 AABB 与记录字段比。
候选只有两个：`local = AABB(v)`，`rotated_mm = AABB(M3 · v × 1000)`（**只乘旋转、不加平移，mm**）。

```
1RCV0101-B1.rvm            3 条   rotated_mm 3
1RS-WF03-W-C-RR001.rvm   113 条   rotated_mm 105 · local 8      ← 唯一的例外群
1RX07-LCT.rvm             28 条   rotated_mm 28
C-IY-1R330-B.rvm          90 条   rotated_mm 90
C-OR-1R345-C.rvm           2 条   rotated_mm 2
                        ─────────────────────────────────────
                         236 条   rotated_mm 228 · local 8，各自帧下残差 ≤ 0.0011 mm
```

样例（`rotated_mm` 那一档，残差 0.0000）：

```
PRIM @ 0x358
  record     = [-15.3939, -21.5815,  0.0,  16.5023, 22.3575, 30.0]
  local      = [ -0.6939, -26.5000,  0.0,   2.0799, 26.5000, 30.0]   maxdiff=14.7000
  rotated_mm = [-15.3939, -21.5815,  0.0,  16.5023, 22.3575, 30.0]   maxdiff=0.0000   ← 命中
  world_mm   = [9365.31, 13478.89, 2900.00, 9397.20, 13522.83, 2930.00]
```

**那 8 个例外不是浮点抖动，是另一族构件**——把它们的属主打出来，边界干净得反常：

```
--- bbox 落在 local 帧的 8 条 ---
  PRIM | STWALL 1..4 of CWALL /1RS-WF03-W-C-RR001    (各 6 polys / 24 verts)
  PRIM | WALL   1..4 of CWALL /1RS-WF03-W-C-RR001    (48 / 194 / 144 / 35 polys)
--- 同一个文件里落在 rotated 帧的 ---
  PRIM | GWALL 1..3 of CWALL /1RS-WF03-W-C-RR001
  PRIM | SBFITTING 1 of CMPFITTING /1RS03KK1003T
```

`WALL` / `STWALL` 整族用 local 帧，同一个 CWALL 下的 `GWALL` 整族用 rotated 帧，
两者旋转矩阵都是正交的（`|col0| = 0.001`）、`dl` 与 `dr` 差着上千毫米，不存在「两个候选都对」的并列。
**所以 bbox 的帧由写出端的代码路径决定，不由 kind 决定。**

**解析图元这一侧则整齐得多**（`.scratch/rvm_prim_bbox_probe.py`）：

```
Box @0x278 |col0|=0.001 params=[1000,50,30]
  record  = [-500, -25, -15, 500, 25, 15]
  local   = [-500, -25, -15, 500, 25, 15]   ← 命中，d=0.000
  rotated = [-428.499, -300.322, -15, 428.499, 300.322, 15]   d=275.322

CircularTorus @0x2b0 |col0|=0.001 params=[offset 76, radius 30, angle 1.571]
  record  = [0, 0, -30, 106, 106, 30]     ← = [0,0,-r, offset+r, offset+r, r]，d=0.000
  rotated = [-26.572, -107.813, 0, 75.779, 13.927, 106]   d=107.813

analytic frame fit: {Box:local 170, Cylinder:local 15, Snout:local 6, CircularTorus:local 5}
                                                                       ← 196/196 全 local
```

**结论（236 条 FacetGroup + 196 条解析图元，共 432 条）**：

| | bbox 帧 | 计数 |
|---|---|---|
| 解析图元 Box / Cylinder / Snout / CircularTorus | 图元自身局部系 | 196 / 196 |
| FacetGroup，`WALL` / `STWALL` 族 | 局部系（同顶点） | 8 |
| FacetGroup，其余 | `AABB(M3 · v × 1000)`，旋转后未平移 | 228 |

rvm-rs 对所有记录一律 `bbox.transform(&transform)`（`parser/rvm.rs`），对 228 条 rotated 记录就
**多转了一次**——5.4 m 那个数就是这么来的。

**这个 bug 现在就在仓库里的快照上**。拿 `test_data/rvm/C-IY-1R330-B.rvm.json` 的
`aabb_world_mm` 与同一份 RVM 里面片顶点实算的世界 AABB 逐件比：

```
compared    : 18   (带 FacetGroup 的成员)
off by >1mm : 18   ← 全错
worst       : 158.4 mm  @ BEND 15 of BRANCH /C-IY-1R330-B
```

gen-model 那份方案文档记的首轮红灯是「L3 AABB 比较/超限 35/35，最大 310.236 mm」，
当时归因到导出口径（宽口径带进了 `GS-RESERVED` 预留净空）。**口径那一半是对的，但不是全部**
——BEND 这一族的 158 mm 是纯粹的读侧帧错误，与导出口径无关。两个成因得分开修。

**矩阵列序也顺带钉死了**：上面 228 条残差 0 是按**列主序**
（m00 m10 m20 | m01 m11 m21 | m02 m12 m22 | m03 m13 m23）算的。
换成行主序就是取了逆旋转，AABB 立刻对不上——所以列序不是「按惯例推测」，是被这 228 条证否过替代解的。

**读侧的可靠做法不是按 kind 查表，是逐记录自证**：拿该记录自己的几何（FacetGroup 用顶点，
解析图元用参数还原出的规范盒）分别按两个帧算 AABB，取与记录字段吻合的那个，两个都不吻合就记账。
`examples/rvm_facets_dump.rs:94 matches_record_bbox` 已经是这个套路，只是当时用来判 rvm-rs 的 Y-up/Z-up，
现在换成判帧。**这样 `--facets` 从必需降为可选**：世界 AABB 从记录字段就能算准。
（局部系判据仍需顶点，见 §5 M1。）

> 待核（IDA）：`WALL`/`STWALL` 与 `GWALL` 走的是不是两个不同的 bbox 计算点。
> 这个结论不影响我们**写**（写出端自己选一个帧并保持一致即可），只影响**读**的实现要不要保留自证分支
> ——在 IDA 给出确定答案之前，自证分支必须留着。

### 2.4 CNTB 平移是**绝对世界坐标**，祖先链全零

`/1RX03-EQUI` → `/1RX03-LCT` → `/1RX-345` → `/C-OR-1R345-C` 四层 CNTB 的平移**全是 `(0,0,0)`**，
成员那层直接是世界 mm（`FTUBE 1` = `(8550.54, 14060.42, 2900.00)`）。
**逐级累加会把坐标乘上嵌套深度**——gen-model `snapshot.rs:107` 早有这条注释，这里再由字节坐实一次。

另注：`CNTB` 平移与它内部 `PRIM` 的平移**不是同一个点**（`FTUBE 1`：CNTB `(8550.54,14060.42,2900.00)`
vs PRIM `(8965.1,13780.8,2915.0)`）。CNTB 给的是**元素原点**（管段起点），PRIM 给的是**图元自身原点**（盒心）。
写出端两个都要各自算，不能互相顶替。

### 2.5 命名：我们已经会了

无名元素的组名是 `<NOUN> <n> of <OWNER 显示名>`，NOUN 用**全称**（`FTUBE` / `BRANCH` / `SUBEQUIPMENT`），
不是库里的四字短名（`FTUB` / `BRAN` / `SUBE`）。

**这一半不用重做**：`src/bin/gen_ams.rs:233 owner_display_desc` 已经递归复现了 E3D 的默认显示名
（含 `CYLINDER 18 of SUBEQUIPMENT 20 of EQUIPMENT /X` 这种多层无名），
`tube_pair_key` 也已经把管身对齐到 `TUBE n of BRANCH /X`。写出端直接复用同一对 `(owner_desc, ordinal)`。

差的只有 **短名 → 全称** 一张表（`FTUB→FTUBE`、`BRAN→BRANCH`、`SUBE→SUBEQUIPMENT`…）。
`rvm_compare` 两侧都做 `noun4()` 截断，所以**对拍不需要这张表**；只有人读 RVM 时才需要。
→ 定为 M2 的可选项，先写短名，`noun_long_name()` 缺表时原样输出。

### 2.6 拿这些字节把 `C-IY-1R330-B` 的 35/35 红灯拆开

`docs/evidence/2026-09-04-model-correctness/` 里同一个样本有两份报告，差别只在基准 AABB 的来源：

```
rvm-compare.log          AABB 来源: 面片实算 0 / 快照字段 35
  off raw= 251.1561 mm  BEND 8      off raw= 246.0041 mm  BEND 4
  off raw= 241.7573 mm  BEND 15     off raw= 100.0000 mm  BEND 1/2/3/6
  Error: 35/35 件超出弦高容差

rvm-compare-facets.log   AABB 来源: 面片实算 35 / 快照字段 0
  off raw= 100.0000 mm  BEND 7 / FTUB 5 / BEND 1 / FTUB 1 / BEND 2 / BEND 5 / BEND 8 / BEND 9 / BEND 10 / BEND 11
  Error: 35/35 件超出弦高容差
```

**换成面片实算之后，35 件的偏差全部塌成同一个数：整整 100.0000 mm。** 这就把红灯拆成了两个独立成因：

**成因 A —— 读侧 bbox 帧错误（§2.3），贡献的是那 151 mm 的离散。**
`251.16 / 246.00 / 241.76 …` 这些参差不齐的数一个都不是真的，全是 rvm-rs 对 rotated 帧记录多乘一次
旋转的产物。`C-IY-1R330-B.rvm.json` 里 18 个带面片的成员**全部**被污染，最大 158.4 mm。

**成因 B —— 这份 RVM 根本不是 narrow 口径，剩下的 100.0 mm 是预留净空。**
`--scope narrow` 是命令行上贴的标签，字节不认这个标签。逐成员打开图元构成：

```
FTUBE 1 of BRANCH /C-IY-1R330-B     3 个图元
      Box  params=[1500.0, 100.0,  50.0]   Zspan=50    ← GS-CABLEWAY（LEVE=[0,0]，宽 100）
      Box  params=[1500.0, 103.0,  50.0]   Zspan=50    ← GS-BOARD    ← 生成侧只出这一个，是对的
      Box  params=[1500.0, 103.0, 100.0]   Zspan=100   ← GS-RESERVED（PZLE := RPRO CLEA = 100）
BEND 1 of BRANCH /C-IY-1R330-B      3 个 FacetGroup，Zspan 50 / 50 / 100   ← 同样三件套
```

签名归并后 **每一个成员都是三件套**，无一例外。`GS-RESERVED` 的 Z 高正好 100 mm——
**与 facets 那轮 35 件整齐划一的 100.0000 mm 完全对上**。

这两条与 gen-model `docs/2026-08-04_rvm-baseline-verification-plan.md` 的「复核订正」一节
（那里已经反解出 `[1] W=100 GS-CABLEWAY / [2] W=103 GS-BOARD / [3] W=103 GS-RESERVED`）
逐条吻合，只是当时没有把「读侧帧错误」这一半分出来，于是 151 mm 的离散被一起算进了口径账。

**结论：35/35 里没有一件是生成侧的几何缺陷。** A 归读侧（M0 已修，见 §五），B 归导出口径
（重导时 `repre obst off`，或让 `--scope` 从字节自证而不是靠人贴标签）。

---

## 三、写出端复核（ida-bridge）—— ✅ **已完成（2026-09-05）**

### 3.0 找错了地方：写出端不在 Core3D.dll，在 `expdri.dll`

`Core3D.dll` / `core.dll` 两个 IDB 里搜 `RVM`、搜 `CNTB`/`PRIM` 全是空的。真正的写出端由导出宏
自己点名：`rvm_export_*.mac` 第一行就是 **`export system /expdri.so`**，Windows 上落成
`E:\reverse\e3d\shadow_e3d31_aps_all\expdri.dll`（71 208 字节，2025-03-07 —— 与基准文件 banner 里那句
`(7 Mar 2025 : 06:12)` 是同一次构建）。整个 E3D 安装里只有它含 `Unicode UTF-8`，也就是 RVM `HEAD` 的
第五个字符串字段。

它小得反常，因为它只是**被动的字节水槽**：`expdriFunctionality(host)` 把宿主指针包成一个
`expdriRealClass`，此后所有 I/O 与几何查询都通过宿主的虚表回调出去。**驱动自己不做任何决策。**

模块内部 37 个例程各自带一句 `"expdri/XXX"` 的 trace 名，所以逐个可辨（IDB 里已改名为 `expdri_*`）：

| 例程 | 干什么 |
|---|---|
| `EXPHDR` / `EXPINI` / `EXPFIN` | `HEAD`、打开、收尾 |
| `EXPCST` / `EXPCND` | `CNTB` / `CNTE` |
| `DRBOX` `DRCYL` `DRSNOU` `DRSPHE` `DRCTOR` `DRRTOR` `DRDISH` `DRPYRA` `DRPOLY` `DRWLIN` `DRWEXT` | 各类解析图元 |
| `DRFACG` | FacetGroup |
| `DPMCHK` / `DPVCHK` / `BWRADM` / `BWRADV` | 远场坐标检测与逃逸（见 3.4） |
| `ZPRMRE` | 填 `version` / `flags` 两个字 |

### 3.1 ① bbox 的两个帧：**同一条代码路径，不是 kind 分支**

`DRFACG` 里算记录 bbox 的就是这四行（`dbl_1000F8A0` 是当前变换，`[0..8]` 旋转、`[9..11]` 平移 mm）：

```c
sub_1000BBF0(&v5, dbl_1000F8A0, v34, v35);   // 向宿主要这条几何的包围盒（虚表 +156）
v33[0..2] = v34[0..2] - dbl_1000F8A0[9..11]; // 减掉平移
v33[3..5] = v35[0..2] - dbl_1000F8A0[9..11];
```

**记录 bbox ≡ `宿主给的包围盒 − 平移`，无条件、无分支。** 所以 §2.3 那个「rotated 帧」不是刻意设计
的编码，而是这一句减法的副产品；而那 8 条 `WALL`/`STWALL` 落回 local，只可能是**宿主的包围盒提供者
对这一族返回了「加了平移、没加旋转」的盒**——帧的差异发生在虚表另一侧的几何构建器里，
`expdri` 这边看不见，也管不着。

这正面证实了 §2.3 的措辞「**帧由写出端的代码路径决定，不由 kind 决定**」，
并且把 §2.3 末尾那条「待核」结掉了：**读侧的自证分支必须留着**，因为帧由宿主的构建器决定，
写出端没有任何可查的表。M0 选择「一律从几何实算、`bbox_frame` 只做记账」是对的。

解析图元那一侧则是**写死的常量盒**，与宿主无关：

```c
DRBOX:  v9 = [-x/2, -y/2, -z/2, +x/2, +y/2, +z/2]
DRCYL:  v8 = [ -r,   -r,  -h/2,  +r,   +r,  +h/2 ]      ← 沿局部 Z 居中
```

`DRCYL` 这一行**独立复证了 M4 的圆柱约定**（`CylinderV3` 是 `Z ∈ [0,1]` 不居中，写 RVM 要挪 `h/2`）
——此前这条只有 AABB 自校闸兜着，现在有了写出端原文。

### 3.2 ② kind 选择规则：**不在写出端，问不到**

`expdri` 每个 kind 一个入口，宿主调哪个就写哪个；驱动里没有任何「退化成 FacetGroup」的判断。
**所以「E3D 何时写 Box、何时写面片」是宿主图形遍历的决定，从 RVM 这一侧永远推不出来。**

这把 M4 那条结论从经验判断升成了结构判断：**kind 一致率只能当诊断，不能当门禁**——
它度量的是「宿主对这一族选了哪种表示」，那是我们够不着的变量。

另外，原问题里「`level` 参数落在哪」问错了：**解析图元记录里没有 level 字段**，
所有 kind 的参数段都紧跟 bbox。`level` 是 `OBST`/`INSU` 才有的那个字，见下。

### 3.3 ③ `OBST` / `INSU`：一个全局闸 + 每条图元两个通道，**记录里多一个字**

每个 `DR*` 例程写 tag 之前都是同一段（以 `DRBOX` 为例，`a5` 是宿主传进来的 2 元等级数组）：

```c
if ( dword_10014090 <= 0 ) goto 默认;                    // 全局闸：没开就一律写默认 tag
if ( *a5 <= 0 ) {
    if ( a5[1] > 0 ) { 写 tag_B; dword_1000F9B0 += 4; }   // 通道 1
    else 默认: 写 tag_默认;
} else { 写 tag_A; dword_1000F9B0 += 4; }                 // 通道 0，优先于通道 1
```

而那多出来的 4 个字节，写在 **bbox 之后、参数之前**：

```c
v8 = 0;
if (通道命中) { unk_1000F13C = 等级; sub_1000BCF0(...,&v8); sub_1000B990(&v8, ...); }
sub_1000B9D0(a2); sub_1000B9D0(a3); sub_1000B9D0(a4);     // 参数在这之后才写
```

**结论**：`OBST` / `INSU` 记录 = `PRIM` 记录 + bbox 后插一个等级字，记录长 +4。
`dword_10014090` 是**与 `repre obst/insu on|off` 分开的另一个闸**——这解释了为什么
`C-IY-1R330-B` 明明是 `repre obst on` 导的，却一条 `OBST` 都没有：预留体被画了出来、
因而被导出了，但因为这个全局闸没开，写的是普通 `PRIM`。**§2 里「宽口径把 `GS-RESERVED`
带进来了、但在 RVM 流里它就是个普通 box」的复核订正，到此有了机制解释。**

（通道 0 / 通道 1 与 `OBST` / `INSU` 的对应按选项顺序推定；两个 tag 常量在 BSS 里，
静态读不出字面值。结构本身是确定的，对应关系是推定的。）

**这条挖出了读侧一个潜伏缺陷**，见 §5 M5。

### 3.4 ④ 远场逃逸档：实测覆盖不到的分支

`ZPRMRE` 只干一件事，填记录头两个字：

```c
*a2 = 1;                                   // version 恒为 1
a2[1] = (超界) ? 2 : 1;                     // flags
```

超界的判据是 `DPMCHK` → `DPVCHK`：**平移任一轴 `|t| ≥ 1e9 mm`（±1000 km）**。命中时
记录长 +24，末尾由 `BWRADM` → `BWRADV` 补写 3 个真实平移值。容器侧同理：`EXPCST` 里
`CNTB` 的第二个头字正常是 **2**，带障碍信息时变 **3**，远场时变 **4**。

八份基准 823 条记录全是 `flags=1`、609 条 `CNTB` 全是 `[1, 2]` —— **默认支**。
这是实测永远覆盖不到的分支，写出端只要坐标不越 ±1e9 mm 就恒走默认支。

### 3.5 顺带钉死的几条（与 §2 实测互证）

| 项 | 写出端原文 | 与实测 |
|---|---|---|
| kind 编号 | `DRBOX` 写常量 `2`、`DRFACG` 写 `11` | 与 §2.1 一致 |
| `CNTE` 负载 | `EXPCND` 写 `dword_1001001C` = `[1, 2]`，记录恒 28 字节 | 与实测 `00000001 00000002` 一致，**坐实了先前那次订正** |
| `CNTB` 记录长 | `4 × (名字字数 + 12)` = 头 5 + 版本/n 2 + 名字 (1+n) + 平移 3 + 材质 1 | 逐条对上 |
| FacetGroup 顶点帧 | 顶点与法线都过 `v37`，而 `v37` 在函数里被显式置成**单位阵** | 顶点存的就是局部系原值，与 M1/M2 的读写模型一致 |
| 记录长度 | `DRBOX` `+= 116`、`DRCYL` `+= 112` | = 20 头 + 96 / 92 负载，与实测逐字节相同 |

---

## 四、方案

### 4.1 落点：写出端进 `e3d-model`，不进 `gen-model`

| 候选 | 判断 |
|---|---|
| `e3d-model/src/rvm_write.rs`（新模块）+ `gen_ams --rvm <path>` | **采用**。生成物 `GeneratedElement` / `TriMesh` / `owner_display_desc` 全在这里，跨仓拿数据要先把它们序列化一遍——那就退回今天 `elements.json` 的老路 |
| `gen-model/src/rvm_baseline/export.rs` | 否。gen-model 只是消费方；且它的 `rvm_verify` 在 feature 门后，默认不编 |
| 复用 `gen-model/src/rvm/`（既有的 ASCII RVM 写出） | 否。那套写的是**文本 RVM**（`"HEAD\r\n     1     2\r\n..."`，`src/rvm/head.rs:31`），与 E3D 导出的二进制 RVM 不是一个东西；且它挂在 ArangoDB + MySQL + `AiosDBManager` 的旧数据栈上，`create_element_data_rvm_tree` 整个函数体已被注释成空壳 |

**依赖不变**：写出端是我们自己拼字节，**不依赖 rvm-rs**（rvm-rs 在 e3d-model 里是 `dev-dependencies`，
bin 本来也用不了）。回环测试放 `tests/`，那里可以用 rvm-rs。

### 4.2 写什么几何：先 FacetGroup，解析图元列为 M4

生成侧的可见产物是 `Manifold` → `TriMesh`（`src/mesh.rs`：f64 顶点 + u32 三角索引，mm）。
最直接的映射是 **FacetGroup(kind 11)**：一个三角形 = 一个 polygon（1 个 contour、3 个顶点 + 逐顶点法线）。

E3D 侧则是混的——同一份 `C-OR-1R345-C.rvm` 里直管段是 `Box`、弯头是 `FacetGroup`；
`C-IY-1R330-B.rvm` 是 `Box=51 + FacetGroup=90`。

**决定：M2 一律写 FacetGroup，几何种类差异记为信息项、不判红。** 理由：

1. 判据本来就落在 AABB / 顶点，不落在 kind——这与 `rvm_compare` 今天把 L2 降为信息项是同一个口径。
2. 我们手上**没有**与 E3D 对齐的解析图元。`primitive_instance.rs` 只有 `BoxV1` / `CylinderV3`
   两个规范体，且是为**网格缓存**设计的（`PrimitiveMeshKey` 刻意不带绝对尺寸），拿它去凑 RVM 的
   10 种图元是另一个课题。
3. 先把链路打通，再谈保真度。

M4 再做「有 `PrimitiveInstance` 就写 Box/Cylinder」，届时 kind 一致率本身变成一项判据。

### 4.3 写出口径（一张表，照着写）

| 项 | 写什么 |
|---|---|
| `HEAD` banner | `e3d-model <version> (<git rev>)` —— **不模仿 E3D 的横幅**，免得快照分不清来源 |
| `HEAD` filenote | `generated by e3d-model for <root> (dbnum <n>, sesno <n>)` |
| `HEAD` date / user | 本机时间 / 用户。**这两项与 banner 一起构成「不可能字节相同」的部分**，见 §七 |
| `MODL` | model = 项目码（`AMS`），project = MDB 名（`/ALL`） |
| 祖先 `CNTB` | 从 root 沿属主链上溯到 SITE，逐层一个 CNTB，**平移全 0**，material=1 |
| 成员 `CNTB` | name = §2.5 的显示名；平移 = 元素世界原点 **mm**（`GeneratedElement::world` 的平移列）；material=1 |
| `PRIM` 矩阵 | `M3 = R × 0.001`，`t = 世界平移 / 1000`（**m**）。`R` 取自 `GeneratedElement::world` 的 3×3 |
| `PRIM` 顶点 | `TriMesh` 顶点先乘 `world⁻¹` 落回**元素局部系 mm**（`gen_ams` 已经在算 `local_aabb`，同一段变换） |
| `PRIM` bbox | `AABB(M3 · v × 1000)`——§2.3 里 228/236 的那一档。**我们只用这一个帧**，不复现 `WALL`/`STWALL` 的例外；读侧的自证分支照样能认出来 |
| `PRIM` flags | `1`（六份样本 809/809 都是 1；其它取值待核） |
| 法线 | 按三角面法线写三份（E3D 也是逐顶点存，样例 `nrm=[0,1,0]` 三个顶点相同） |
| `CNTE` | 每个 `CNTB` 配一个，负载 `00000001 00000002`（**不是** `1,1`，见 §2.1 的提醒） |
| `COLR` | 收尾一条，颜色 `0x82828200`（与样本一致，暂不做逐元素配色） |
| `END:` | tag 第 4 字 `':'`，负载 8 字节 |
| 尾部补零 | 先不补。读侧靠 16 个 0 停，我们最后一块之后直接给 16 个 0 |

**为什么不偷懒把顶点直接写成世界 mm、矩阵写成 `I×0.001` + `t=0`**：那样合法、也能读回，
但**丢掉局部帧**。`rvm_compare` 的可信判据是「facet 顶点乘 `world⁻¹` 后在元素局部系比 AABB」
（模块文档「判据」一节：旋转墙的世界 X/Y AABB 不能代表墙自身的端头/厚度方向）。
写成 `R×0.001` + 局部顶点，两侧的局部帧才对得上。

### 4.4 对称对拍

`rvm_compare` 增一路输入：

```text
rvm_compare --baseline <a.rvm> --generated <b.rvm> [--out compare.json]      # 新：RVM ↔ RVM
rvm_compare --baseline <a.rvm.json> --elements <elements.json> [--facets …]  # 旧：保留不动
```

新路径两侧走**同一段** `read_rvm() -> Vec<RvmMember>`：同一个 key 提取（§2.5 的显示名 → 现有 `Key`）、
同一段帧换算（§2.2 / §2.3）、同一段 AABB 计算。`--facets` 在新路径下**不需要**。

判档沿用现有四档（`exact` / `chord` / `reference_artifact` / `off`）与现有阈值，不动判据口径
——这样第一轮就能拿新旧两条路径**对同一个样本互证**（见 M3 的验收判据）。

---

## 五、工作分解

每条带**锚点**（改哪里）与**验收判据**（怎么算做完）。

### M0 · 把 §2.3 的结论落回读侧 —— ✅ **已落地并验过**

**最后的改法比原计划小得多，因为「自证帧」这一步可以整个绕开。**

原打算逐记录判帧（`local` vs `rotated_mm`）。但把两类记录分开看就会发现根本不必判：

- **解析图元 196/196 存的是局部系盒**，而 rvm-rs 的 `bbox.transform(&transform)` 正好就是
  「把局部盒变换到世界」——**这条路径本来就是对的，一行都不用动。**
- **唯一错的是 FacetGroup**，而 FacetGroup **总是带顶点**。有顶点就不必猜帧：
  `世界 mm = (matrix3 · v + translation) × 1000` 恒成立，直接实算。

于是 `WALL`/`STWALL` 那 8 条例外也自动不成问题了——实算不关心记录字段在哪个帧。

- **落点**：gen-model `src/rvm_baseline/import.rs` 新增 `facet_group_world_aabb_mm()`，
  `build_geometry` 优先用它，非 FacetGroup 原样走 rvm-rs；
  `SnapshotMeta` 加 `facet_aabb_from_vertices`（`#[serde(default)]`，不动 `SNAPSHOT_VERSION`，
  旧快照读进来是 0，正好等于「这份快照还是老口径」的标记）。
- **实测**（`cargo build --features rvm_verify --bin rvm_verify` 通过，六份样本全跑）：

```
sample                    members    off>1mm       worst mm
1RCV0101-B1                     3          0         0.0002
1RS-WF03-W-C-RR001            113          0         0.0010     ← 含 WALL/STWALL 那 8 条
1RX03-MSUP-E0110                0          0         0.0000
1RX07-LCT                      28          0         0.0015
C-IY-1R330-B                   18          0         0.0009
C-OR-1R345-C                    2          0         0.0004
TOTAL                         164          0         0.0015

C-IY-1R330-B 单独对比：
  BEFORE (仓库里的快照)   compared=18  off>1mm=18  worst=158.3846 mm
  AFTER  (M0)             compared=18  off>1mm= 0  worst=  0.0009 mm
```

`AABB 面片实算` 的计数与 §2.1 的图元普查逐份对上（2 / 28 / 3 / 113 / 90 / 0，合计 236）。
残差 0.0015 mm 是 f32 舍入，量级符合坐标 ~2e4 × 1e-7。

- **`--facets` 对照已补齐**。生成侧不必重跑——`docs/evidence/2026-09-04-model-correctness/c-iy/elements.json`
  就是当轮的产物，直接拿它跑三路：

| 跑法 | 基准 AABB | `raw` 偏差的取值集合 |
|---|---|---|
| A 旧快照 + 不给 `--facets` | `snapshot` 35 | `{100.0, 241.7573, 246.0041, 251.1561}` ← 复现了当初记录的那一轮 |
| B **M0 快照** + 不给 `--facets` | `snapshot` 35 | **`{100.0}`** |
| C 旧快照 + `--facets` | `facets` 35 | **`{100.0}`** |

```
B vs C   verdict 不同的件数 : 0 / 35
B vs C   max_axis_delta 最大差 : 0.000017 mm
```

**判据达成：不给 `--facets` 与给了 `--facets` 逐件同档。**

> 一处必须说清、不能含糊过去的差别：B 走的是 **world** 帧（`comparison_frame=world`），
> C 走的是 **local** 帧。两者在这个样本上判得一样，但**这不等于 world 帧从此够用**
> ——`rvm_compare` 模块文档说得很清楚，旋转墙的世界 X/Y AABB 代表不了墙自身的端头/厚度方向。
> 所以 M0 的准确表述是：**`--facets` 从「不给就错」降为「不给就只有 world 帧判据」**，
> 而不是「不再需要」。要在旋转构件上也拿到 local 帧判据，仍得有顶点——这正是 M1
> 把顶点纳入统一读法的理由。

- **探针已转正**：四个 `.scratch/` 脚本合并成 `tools/rvm_record_probe.py`（四个子命令
  `skeleton` / `facet` / `frames` / `fields`，直接读 `.rvm` 字节）。复跑：

```bash
python tools/rvm_record_probe.py frames ../../gen-model/test_data/rvm/*.rvm
python tools/rvm_record_probe.py fields ../../gen-model/test_data/rvm/*.rvm
```

```
sample                          records   local  rotated  world  unknown   worst mm
1RCV0101-B1.rvm                      32      29        3      0        0     6.0300
1RS-WF03-W-C-RR001.rvm              113       8      105      0        0     0.0011
1RX03-MSUP-E0110.rvm                  0       0        0      0        0     0.0000
1RX07-LCT.rvm                       137     109       28      0        0     0.0000
C-IY-1R330-B.rvm                    141      51       90      0        0     0.0000
C-OR-1R345-C.rvm                       9       7        2      0        0     0.0000
TOTAL                               432     204      228      0        0     6.0300
```

`unknown = 0` 是这张表的判据：非 0 就是出现了没建模的图元或 E3D 换了帧口径，
读侧此时不得再拿记录字段当世界 AABB。

> 它**取代** `scripts/rvm_record_bbox_probe.py`。那个老探针问的是同一个问题，
> 但只比了 `world` 与 `local` 两个候选、且读的是 `rvm_facets_dump` 的产物（那条链自己带帧问题）。
> 两个都对不上，于是结论成了「记录 bbox 不可信」。补上第三个候选 `rotated` 就命中了。

> `1RCV0101-B1` 那 6.03 mm 不是帧判错，是**偏心 Snout 的记录 bbox 本来就不是紧盒**：
> `Snout[rb 33.165, rt 42.21, h 115.2, ox 12.06, oy 0]` 记录写的是 `x ∈ ±48.24`、
> `y ∈ [−42.21, +48.24]`（`oy = 0` 却上下不对称），紧盒应是 `x ∈ [−39.195, 48.24]`、`y ∈ ±39.195`。
> 「偏心全给顶 / 全给底 / 两端各半」三种建模都试过，残差只会从 6.03 变成 9.05。
> 结论：**E3D 这一支给的是放宽过的估计**，别去凑；残差 ≤ `|offset|/2`，判帧不受影响。
- **顺带**：`src/options.rs:451` 有一处并行会话留下的编译错（`store_mode` 少个 `.clone()`），
  整个 crate 编不过。补了这一个 `.clone()`，**不属于本计划范围，需与那条会话确认**。

### M1 · `rvm_read.rs`：一份两侧共用的读法 —— ✅ **已落地并验过**

- **落点**：`src/rvm_read.rs`（+ `pub mod rvm_read;`）、`examples/rvm_read_audit.rs`。
  `RvmFile { header, members }`、`RvmMember { name, path, parent, translation_mm, geometries }`、
  `RvmGeometry { primitive, matrix3, translation_mm, record_bbox, bbox_frame }`。
  **单位在解析时一次性归一**：`CNTB` 与 `PRIM` 平移统一出 mm，`matrix3` 原样保留 0.001
  （写回去不用换算），`rotation()` 给纯旋转。
- **`world_aabb_mm()` 不碰 `record_bbox`**：FacetGroup 走顶点，解析图元走
  `Primitive::local_aabb_mm()` 的八个角。`bbox_frame` 退化成**纯记账**——它变成
  `Unknown` 就说明 E3D 换了口径或出现没建模的图元，是回归信号，不是判据输入。
- **验收一：与独立实现逐项对上。** `examples/rvm_read_audit` 与
  `tools/rvm_record_probe.py`（另一套实现、另一种语言、直接读字节）在六份样本上：

```
sample                          members  geometry   local  rotated   n/a  unknown
1RCV0101-B1.rvm                      16        32      29        3     0        0
1RS-WF03-W-C-RR001.rvm              299       449       8      105   336        0
1RX03-MSUP-E0110.rvm                 51        41       0        0    41        0
1RX07-LCT.rvm                       159       137     109       28     0        0
C-IY-1R330-B.rvm                     40       141      51       90     0        0
C-OR-1R345-C.rvm                     13         9       7        2     0        0
TOTAL                               578       809     204      228   377        0

kind 分布: {Box 170, CircularTorus 5, Cylinder 15, FacetGroup 236, Line 377, Snout 6}
```

`members 578` = `CNTB` 数，`geometry 809` = `PRIM` 数，`local 204 / rotated 228` —— 与 §2.1、§2.3
的普查逐个相同。**两套独立实现同时错成一样的可能性，远低于一套自说自话。**

- **验收二：与 M0 的快照互校。** `rvm_read` 的逐成员世界 AABB vs `rvm_verify import`（rvm-rs + M0）：

```
sample                     compared   off>1e-3       worst mm
1RCV0101-B1                      12          0       0.000472
1RS-WF03-W-C-RR001              113          1       0.001018
1RX03-MSUP-E0110                  0          0       0.000000
1RX07-LCT                       137         31       0.001836
C-IY-1R330-B                     35          0       0.000939
C-OR-1R345-C                      9          0       0.000414
TOTAL                           306         32       0.001836
```

最大差 **0.0018 mm**，正是坐标 ~2e4 × f32 相对精度 1e-7 的量级；`off>1e-3` 那 32 件全落在
0.001–0.0018 之间，没有一件是真差异。

- **一处刻意的不一致**：另有 131 个成员快照给了世界 AABB 而 `rvm_read` 给 `None`。
  逐个查过，**131/131 都是只含 `Line` 的成员**——中心线没有体，不该贡献体积 AABB。
  这一档在 `rvm_read` 里是 `BboxFrame::NotApplicable`，与「判不出来」的 `Unknown` 分开计，
  否则 377 条中心线会把回归门永久染红。
- **单测 6 条**（`cargo test --lib rvm_read` 全过）：手工拼的最小 RVM 覆盖三套单位与帧判定，
  扫掠体 AABB 覆盖整圈/四分之一圈，`CircularTorus` 对着 ams7999 的真实记录钉了一条。

### M2 · `rvm_write.rs` + `gen_ams --rvm` —— ✅ **已落地并验过**

- **落点**：`src/rvm_write.rs`（`impl RvmFile { to_bytes / write }` +
  `facet_group_from_world_triangles` / `generated_header` / `container_member`）、
  `src/bin/gen_ams.rs` 加 `--rvm`。**读写共用 `rvm_read` 的结构**，所以
  「写出去再读回来必须一模一样」直接可判。
- **验收一：真实 E3D 文件过一遍我们的写出端。** 读六份 → 用 `rvm_write` 写回 → 再读：

```
sample                          members  geometry        名字不符   AABB 最大差 mm
1RCV0101-B1.rvm                      16        32           0      0.000000
1RS-WF03-W-C-RR001.rvm              299       449           0      0.000000
1RX03-MSUP-E0110.rvm                 51        41           0      0.000000
1RX07-LCT.rvm                       159       137           0      0.000000
C-IY-1R330-B.rvm                     40       141          0      0.000000
C-OR-1R345-C.rvm                     13         9           0      0.000000
```

578 个成员、809 条几何、六种图元全覆盖，名字 / 层级 / kind 全一致，世界 AABB 差 **0.000000 mm**
（值本来就来自 f32，重编码无损）。**写出端不是只在自造的玩具数据上跑通的。**

- **验收二：`gen_ams --rvm` 端到端。** 同 `/C-IY-1R330-B`、同 sesno 281：

```
RVM: 35 个成员 -> out/rvm-m2-c-iy/generated.rvm     (85 052 字节；E3D 同一条支管是 84 480)
generated.rvm    members 36   geometry 35   local 0   rotated 35   n/a 0   unknown 0
kind 分布: {FacetGroup 35}
```

`unknown = 0` 说明写出的每条记录都满足 §2.3 的 rotated 帧不变量——**E3D 那 228 条成立的那条，
我们自己也成立**；否则读侧会当场把它判成 `Unknown`。自己的文件再回环一遍同样 0.000000 mm。

- **验收三（这才是这件事的目的）：RVM ↔ RVM 直接对拍。** 两份 `.rvm` 进同一个读法：

```
生成侧 35 件 / 基准侧 35 件 / 主键配上 35 件
只在生成: []      只在基准: []
世界 AABB 逐轴最大差的取值分布:
     100.0 mm  x35
```

**35/35 配上、零单边、偏差整齐划一 100.0 mm。** 这与 §2.6 里 `--facets` 那一轮的数**完全一致**，
但这一次**没有 `elements.json`、没有 `rvm_facets_dump`、没有快照**——两份 `.rvm` 进去，一个数出来。
它同时又独立复证了一次 §2.6 的拆分：残差就是 `GS-RESERVED` 那 100 mm，没有别的。

- **本轮的简化，别当成已完成**：
  - 祖先只写一层 `/GENERATED` 容器，没有复现 SITE/ZONE/PIPE 链。
    `rvm_compare` 的主键只用成员名，不看祖先层数，所以不影响判定；要给人看还得补。
  - noun 发的是库里的短名（`FTUB`），E3D 是全称（`FTUBE`）。两侧都 `noun4()` 截断，
    对拍不受影响，人读时差一截尾巴。
  - 一律 FacetGroup（见 §4.2），解析图元留给 M4。
- **没做**：第三方工具（Navisworks / AVEVA Review）里打开一次做目验。

### M3 · `rvm_compare --baseline <a.rvm> --generated <b.rvm>` —— ✅ **已落地并验过**

```bash
# 新：RVM ↔ RVM，两侧同一个读法
rvm_compare --baseline a.rvm --generated b.rvm [--out compare.json]
# 旧：快照 ↔ elements.json，一行没动
rvm_compare --baseline a.rvm.json --elements elements.json [--facets f.json]
```

- **改法**：`load_rvm_side()` 把一份 `.rvm` 摊成本工具原有的两种形态
  （`SnapshotMember` + `FacetData` + `Generated`），于是 `Key` / `owner_key` /
  `pair_tubes_by_centre` / 四档判据 / 报告结构 **一行都没动**。
  `--generated` 与 `--elements` / `--facets` 互斥，给错了当场报错。
- **验收：新旧两条路径逐件同档。**

```
pairs  old=35  new=35   keys identical: True
aabb src  old: facets=35 snapshot=0 | new: facets=35 snapshot=0
frame     old: local×35             | new: local×35

verdict 不同         : 0 / 35
max_axis_delta 最大差 : 0.000041 mm
by_noun old: {BEND (18,18), FTUB (17,17)}   new: {BEND (18,18), FTUB (17,17)}
only_baseline/only_generated  old: 0/0   new: 0/0
```

**新路径不需要 `elements.json`、不需要 `rvm_facets_dump`、不需要快照 JSON**，
判档却与旧路径逐件相同。退出码语义不变（本例 `1`，因为确实有 100 mm 的口径差）。

- **顺带修的一处口径不一致**：第一版跑出来是「面片实算 18 / 快照字段 17」——
  17 个 FTUB 成员在基准侧是 `Box`，没有真顶点，于是退回**世界帧**，
  而 18 个 BEND 走**局部帧**。同一份文件里两类成员用两套判据是不能接受的。
  `RvmGeometry::world_hull_points_mm()` 给解析图元补上采样点云，现在是
  「面片实算 35 / 快照字段 0」，全部局部帧。

### M3 批量验收：21 条 BRAN（`scripts/run_rvm_symmetric_audit.ps1`）

单个样本上一致说明不了什么。`output/rvm-audit5` 那 21 条整批过一遍
（每条：`gen_ams --rvm` → 新路径 → 旧路径 → `tools/compare_reports.py` 逐件比）：

```
合计配对 126 件；判档不同 11 件；max_axis_delta 最大差 0.236569 mm
键不同 0（21 条全部）；only_baseline / only_generated 两侧都空；退出码 21/21 一致
```

**11 件判档不同，方向全部是 `chord → exact`**，没有一件变松（没有 `chord → off`、
没有 `exact → chord`）。成因是旧路径的基准 AABB 来自 `rvm_facets_dump` 按 **1 mm 弦高**
的三角化，曲面上会**内缩** `r·(1−cos(π/n))`；新路径直接对解析曲面采样，不吃这个亏。
**证据**：把新路径的采样密度从 64 段加到 256 段（自身误差 0.12 mm → 0.0075 mm），
上面这张表**一个数都没变**——说明 0.237 mm 全在旧路径那一侧，不是我们的采样误差。

批量跑之前它是红的，两个真缺陷都是这一批逼出来的：

1. **`TUBE` / `TUBI` 没归一。** E3D 把隐含管身导成 `TUBE n of BRANCH /X`，库里的 noun 是
   `TUBI`；`noun4()` 之后 `TUBE` ≠ `TUBI`，而 `pair_tubes_by_centre` 只认 `TUBI`，
   于是整族管身两边各成孤儿。`rvm_verify import` 早就在做这个归一，RVM ↔ RVM 这条路
   自己读文件就得自己做。修完 8 条样本的配对数上去了（`2944` 4→10、`3184` 3→7、`24049` 10→12…）。
2. **解析图元曾用「规范盒八个角」当点云。** 盒角只在盒自己身上是紧的；曲面一旦带旋转，
   八角转出去比真实几何大一圈——`26229` 的 `CircularTorus` 弯头因此虚高 **56.7 mm**，
   把本该 `chord` 的件判成 `off`，还让整条样本从绿翻红（退出码 0→1）。
   改成按面采样后回到 0.24 mm。这条有单测钉住（`a_rotated_cylinder_is_not_bounded_by_its_rotated_box_corners`）。

**一处已知限制，没修**：RVM 里**命名元素不带 noun**（E3D 只写 `/NAME`），
所以新路径给它们的 `Key::Name.noun` 是空的。两侧同样空，**配对不受影响**
（`26229` 那 5 个 `ATTA`/`VALV` 照样配上，`only_*` 两侧都空），
受影响的是 `by_noun` 的桶名和 `--noun` 过滤——在新模式下过滤不到命名元素。
要修得让写出端把 noun 也带进 RVM，那是改格式，本期不做。

### M4 · 解析图元 + kind 一致率 —— ✅ **已落地并验过**

- **落点**：`rvm_write::analytic_from_instance()`（`PrimitiveInstance` → `Box(2)` / `Cylinder(8)`）、
  `gen_ams` 里的自校闸、`rvm_compare` 的 `kind_match` / `baseline_kinds` / `generated_kinds`
  与 `NounTally::{kind_match, kind_differ}`。
- **规范体约定**（都是对着 E3D RVM 面片坐实过的，不是推的）：

| | 规范体 | RVM 要的 | 差在哪 |
|---|---|---|---|
| `BoxV1` | `Manifold::cube(1,1,1,true)`，**居中**单位立方 | `Box`：三边长，居中 | 无 |
| `CylinderV3` | `Manifold::cylinder(1,1,1,n,false)`，半径 1、**Z ∈ [0,1] 不居中** | `Cylinder`：半径 + 高，**沿 Z 居中** | 要沿自己的局部 Z 挪 h/2 |

- **自校闸**：约定写错了照样能写出一份**解析得开但几何是错的** RVM，这种错不该靠人眼发现。
  所以每写一个解析图元，都拿它算世界 AABB 与网格实算的比，对不上就退回 FacetGroup 并记账。
  本轮的尺子是一把 0.01 mm 的常数，21 条样本实测 **解析 63 件 / 自校未过 0 件**——约定是对的。
  **这把常数尺后来换掉了**：网格是内接的、解析面是外接的，两者天生差一个弦高，
  一把常数尺遇到段数少的圆柱就会误杀正确的候选，见 M6。

#### 目录件那 39 个 Box：不改契约，自己认几何

第一版只走 `PrimitiveInstance`，kind 一致率 59.5%，差的 39 件**全是 `FTUB`**：
E3D 写 `Box`，我们写面片。追下去发现 `catalogue.rs:1353` 是**刻意**把 `instance` 置 `None` 的：

> 内核对这只箱子施加了两次局部变换（缩放，然后居中平移），能记进 `PrimitiveInstance` 的只有一个矩阵。
> 折叠成 `centre * scale` 数学上等价、位模式上不等价……宁可少一族实例化，不要一个记错的变换。

**这个决定是对的，不该为 RVM 去改**：那 1.5e-11 mm 对持久化是真问题，
对我们写进 **f32** 的 RVM 低了 12 个数量级。所以 RVM 这一侧自己认几何——
`box_from_local_mesh()`：去重后正好 8 顶点、12 三角面、且顶点集恰好是自己 AABB 的八个角。
满足这三条的网格**就是**那只盒子，不是「像」；后面还有 AABB 自校闸再兜一道。

- **判档一个没变**：63 个成员从 FacetGroup 换成解析图元之后，
  126 件配对的判档与 M3 那一轮**逐件相同**（仍是 11 件 `chord → exact`，最大差 0.236569 mm）。
  表示变了、几何没变，正是想要的。
- **新判据：kind 一致率**

#### 逐组件一条记录，不是逐元素一条

E3D 按**目录件**拆记录，`GeneratedElement::parts` 的注释说的就是这件事
（「E3D preserves touching catalogue primitives as separate display meshes」）。
合成一条会让 `Cylinder×2 → FacetGroup` 这类差异永远消不掉，所以改成走 `components()`
（有 `parts` 用 `parts`，没有按连通性拆——单块实体拆出来还是一块）。
记录数当场对齐：`Cylinder×2 → FacetGroup×2`、`FacetGroup×66 → FacetGroup×6`。

#### 圆柱识别：认对了，但 kind 一致率反而掉了

组件拆开之后一个组件就是一个图元，圆柱就认得出来了：端盖法向即轴向，
而端盖法向一定在三角面法向里——逐个当轴试，满足「沿轴只有两个投影值」+「到轴线距离恒定」的那个就是。

结果**一致率 88.5% → 69.5%**，因为多出 **27 件 `FacetGroup → Cylinder`**：
E3D 把这一族 `FTUB` 圆管写成面片，我们写成解析圆柱。

**这 27 件的判档全部是 `exact`**（最大 0.0011 mm）——几何完全一致，差的只是表示。
而且文件小了一个数量级：`22803-M-BR-1R330-E1` 的生成侧 **85 712 → 4 180 字节**。

**所以保留识别，不为了指标回退。** 这正说明一致率**不能当门禁**：
把它当门禁，正确的做法（把真圆柱写成圆柱）会被判成回归，
人就会去迎合 E3D 的表示选择而不是把自己的模型描述准确。它是**诊断**，不是**目标**。

#### 弯头识别：走两个端盖

部分圆环的两头是两张平面圆盘，半径就是管截面半径 `r`。于是：
按面法向分组找出「法向相同、顶点共圆」的两组 → 扫掠轴 `a = n₁ × n₂` →
两个盖心各自沿盖内径向 `n × a` 走，交点就是扫掠中心 `O` → `offset = |O − c₁|`。

包角有个坑：**端盖本身分不出 90° 与 270°**（两者端盖完全一样）。所以拿实体自己的顶点定——
以某个端盖为起点绕 `+a` 量角，真起点会让所有顶点落在 `[0, 包角]` 内，两个候选取小的那个。
（第一版没做这一步，`circ_torus_solid(95,105,90°)` 认回来是 270°，单测当场拦下。）

单测直接拿 `circ_torus_solid` 造真值再认回来，不靠肉眼。

```
kind 一致率：92/131（70.2%）        ← 2026-09-05 当轮；复测见 M7
  不同的组合（基准 → 生成）：
    FacetGroup                                     → Cylinder     x27   ← 我们更解析，判档全 exact
    FacetGroup×3                                   → FacetGroup   x3
    CircularTorus                                  → FacetGroup   x3    ← 4 件里认出 1 件
    Cylinder×2                                     → FacetGroup×2 x3
    FacetGroup                                     → Box          x1
    FacetGroup×66                                  → FacetGroup×6 x1
    Cylinder×8 + Snout×6 + Box×3 + CircularTorus×2 → FacetGroup×3 + Box  x1
```

4 件弯头认出 1 件（`ELBO 1`，判档 `exact` 0.0006 mm）。**另外 3 件不是认错、是拒绝**——
它们的网格没通过「所有顶点到管心圆的距离都等于 r」那一关。判据严就是要这样：
这 3 件的判档本来就是 `exact` / `chord`，几何没问题，只是我们的网格不是一段纯圆环
（端头被路由裁过之类），写成 `CircularTorus` 会是**假的**。宁可写面片。

**126 件配对的判档自始至终一件没变**（11 件 `chord → exact`，最大差 0.236569 mm）——
Box 识别、逐组件拆分、圆柱识别、弯头识别，四次改动都只动表示、不动几何。

> 这句话到 M7 那轮**不再全对**：多出一件真的变松（`exact → off`）。几何仍然没动，
> 松的是判据里缺的一格，成因与修法见 M7 末尾。

### M5 · §3.3 挖出的读侧潜伏缺陷 —— ✅ **已修并验过**

`parse_geometry` 原来无条件把参数段起点当成偏移 84。按 §3.3，`OBST` / `INSU` 记录在 84 处
多一个等级字，参数顺延到 88。后果分两档，都不会报错：

- **解析图元**：参数整体错一个 f32 —— 尺寸静默变形。
- **FacetGroup**：多边形计数从等级字里读 —— 整条记录解析崩掉。

八份基准样本一条 `OBST`/`INSU` 都没有，所以这条**验不了**。因此没有按 tag 硬编码偏移，
而是拿一条自证判据代替：**参数段必须正好吃到记录末尾**。

这条判据先量过才敢用——823 条记录、6 种 kind，余量全是 0：

```
PRIM   Box            leftover=0  x172      PRIM   FacetGroup   leftover=0  x248
PRIM   CircularTorus  leftover=0  x5        PRIM   Line         leftover=0  x377
PRIM   Cylinder       leftover=0  x15       PRIM   Snout        leftover=0  x6
```

两个候选偏移都试（`PRIM` 只试 84；`OBST`/`INSU` 先试 88 再试 84），取正好吃到末尾的那个；
命中 88 时把等级字记进新字段 `RvmGeometry::level`，写出端原样写回去，回环因此仍然成立。
`flags == 2` 时额外允许 24 字节的远场尾巴（§3.4）。

**为什么这样比按 tag 写死好**：`OBST`/`INSU` 的偏移是反编译推出来的，没有样本兜底。
写死就是把一个未验证的假设焊进解析器；自证则是两个方向都不会错——真有等级字就跳过，
没有也认得出来，而且判据本身是被 823 条记录量过的。

**验收**：新增单测 `obstruction_records_carry_a_level_word_before_the_params`（两个方向各钉一次）；
19 条单测全过；8 份真实文件回环仍 0.000000 mm、`unknown` 仍为 0；
21 条批量 126 件配对判档一件没变（11 件 `chord → exact`、最大差 0.236569 mm、kind 92/131）。

### M6 · 自校闸的尺子改成候选自带的弦高 —— ✅ **已落地并验过**（`73ebb5e`）

M4 的闸门用一把 0.01 mm 的常数尺，**它会误杀正确的候选**。网格是内接多边形、解析面是外接曲面，
同一个形体两侧的 AABB 天生差一个弦高 `r·(1 − cos(π/n))`；`1RCV0101` 那 14 根**真**圆柱的差是
0.06–0.35 mm，全是它们自己那 8/16/28 段离散该有的量，闸门按 0.01 mm 判，14 根全退回面片。

- **落点**：`rvm_write::chord_sagitta_mm()` + 新类型 `AnalyticCandidate { geometry, sagitta_mm }`。
  **每个候选自带自己的弦高**，图元与容差绑在一起递出去、拆不开；`matches_mesh_aabb()` 放行
  `tol_mm + sagitta_mm`。段数从网格实算：`RecognisedCylinder::segments` 来自端盖顶点数，
  `RecognisedTorus::{tube_segments, sweep_steps}` 来自端盖与圈数，`PrimitiveMeshKey::CylinderV3`
  直接从键里取。`ANALYTIC_TOL_MM` 退回它本来的活：只管 f32 落盘那一档。
- **为什么不拿 `CHORD_TOL_MM` 当常数**：同一份网格可能出自曲面 0.5 mm、轮廓
  `PROFILE_CHORD_TOL_MM`、SNOU 5 mm 三种口径，而且段数规则有 45° 封顶，实际弦高常比容差还小。
  从网格自己的段数回算，不必猜它出自哪一档。
- **闸门没被削弱**：约定写错的量级是半径或半高（几十上百 mm），放行的是 0.01 mm 加一个实测
  ≤ 一个弦高的量，差两三个数量级。两条回归测试钉住这一点——把同一个候选的 `sagitta_mm`
  抹成 0（就是改动前的闸门）必须被拒。

### M7 · Snout 识别与半圈弯头 —— ✅ **已落地并验过**（`19d1488`）

M6 之后 `26229` 只剩一条 `kind_differ`：阀门 `/Copy-of-1RCV002VP`。E3D 写
`Cylinder×8 + Snout×6 + Box×3 + CircularTorus×2` 共 19 条记录，我们写 `FacetGroup×3 + Box`。
两个缺口，一个是缺识别器，一个是老 bug。

- **`snout_from_local_mesh()`**：认圆台与认圆柱同源——端盖法向即轴向、顶点沿轴只落在两个投影值上、
  每一档各自共圆。只是两个圆的半径可以不同、盖心可以横向错开，那个错位就是 RVM 的 `XOFF`；
  局部 +X 取在错位方向上，`YOFF` 因此恒为 0。三种形体**故意**认不出来、退回面片：两端等径又不偏心
  的是圆柱（`Cylinder` 更贴 E3D）；带剪切的端面不垂直于轴，第一条判据就过不去；锥顶退化成一点的
  圆锥只有一个顶点，撑不起「这些点共圆」。
- **半圈弯头的两处退化**（与 Snout 无关，是更早就在的 bug）。`torus_from_local_mesh` 原来拿
  `n₁ × n₂` 当扫掠轴：包角 180° 时起点盖背朝扫掠方向、终点盖顺着，转过半圈两个法向重合，
  叉乘退化成 0，弯头被判成「两盖平行 = 直管」丢掉。轴现在回退到 `n₁ × (c₂ − c₁)`
  ——**盖心连线与法向不共线时才有定义**，半圈正是这种情况，直管不是，所以这个回退没有放松
  它身后那条判据。更前面还有一关：180° 时两个盖**共面且同向**，落进同一个法向分组，
  16 个点分布在两个圆上当然不共圆，整组被当成「不是端盖」扔掉，弯头连轴那一步都走不到。
  现在每个法向分组先按**共享顶点**拆连通块再做共圆判定：两个盖不共顶点、拆得开，
  单个盖的三角面本来就连通、什么都不变。

**`26229` 实测**（`gen_ams --rvm`，ams8000 sesno 264，对 E3D 导出对拍；「M4 收尾 / M6 后」
两列引自 `73ebb5e` / `19d1488` 的提交信息，「M7 后」是 2026-09-06 17:18 本机复测）：

| | M4 收尾 | M6 后 | M7 后 |
|---|---|---|---|
| 解析图元 / 自校未过退回 | 14 / 14 | 28 / 0 | **36 / 0** |
| `rvm_compare` 判档 | 13 exact + 1 chord | 14/14 exact | **14/14 exact**（最大 0.0010 mm） |
| kind_match | — | 13 / 14 | **14 / 14** |
| 生成文件字节 | — | 114 936 | **6 344** |

阀门那一条现在两侧逐种同构：`Box×3 + CircularTorus×2 + Cylinder×8 + Snout×6`，19 条记录一条不差。

#### 21 条 BRAN 复测（2026-09-06 17:18）

同一批样本、同一条命令（`scripts/run_rvm_symmetric_audit.ps1` → `tools/compare_reports.py`）：

```
合计配对 126 件；判档不同 13 件；max_axis_delta 最大差 1.256116 mm
kind 一致率：126/131（96.2%）                       ← M4 当轮 92/131（70.2%）
解析图元 112 件 / 自校未过退回面片 1 件              ← M4 当轮 63 / 0
21 份生成文件：成员 153 · 几何 177；回环 0.000000 mm；bbox_frame unknown 0
kind 分布: {Box 47, Cylinder 53, FacetGroup 65, CircularTorus 6, Snout 6}
cargo test --lib 268 passed / 0 failed（其中 rvm 28 条）
```

**剩下 5 件 kind 不同，方向全部是「我们更解析」**：

```
FacetGroup×3   → Box×2 + FacetGroup             x1
FacetGroup×3   → Box + Cylinder + FacetGroup    x1
FacetGroup×3   → FacetGroup×2 + Box             x1
FacetGroup     → Box                            x1
FacetGroup×66  → Cylinder×12 + FacetGroup×6     x1
```

M4 当轮那三类**反向**的差一件不剩（`CircularTorus → FacetGroup` ×3、
`Cylinder×2 → FacetGroup×2` ×3、阀门那条 `Cylinder×8 + Snout×6 + Box×3 + CircularTorus×2
→ FacetGroup×3 + Box` ×1）：**没有一件是「E3D 认得出、我们认不出」。**

> **这张表不是纯 RVM 侧改动的功劳，读的时候要拆开算。** 同一棵工作树上还有未提交的几何改动
> （目录件负体扣减那条线），它也在动 kind 一致率。把 `19d1488` 单独 checkout 到隔离 worktree 跑：
> `22803` 的 17 个 FTUB 是实心圆柱，认成 `Cylinder`、4 180 字节；工作树上它们带内孔，
> 识别器在闸门之前就拒了、退回面片、91 152 字节——**而 E3D 也写面片，于是这 17 件反倒 kind_match**。
> `26229` 方向相反：单独 checkout 是解析 20 件 / 199 892 字节，工作树上是 36 / 6 344。
> **RVM 的种类保真度是模型保真度的下游**，这两件事各算各的。

#### 一件真的变松：`23198-C-OR-1R342-A` 的 `BEND 2` 从 `exact` 掉到 `off`

M3 那轮的措辞是「11 件判档不同，方向全部是 `chord → exact`，没有一件变松」。这一轮有一件变松了：

```
23198-C-OR-1R342-A  BEND 2   旧路径 exact (0.0003 mm)   新路径 off (1.2564 mm)
  基准 = FacetGroup×3
  生成 = Box + Cylinder(r=125, h=73.5) + FacetGroup
```

**几何没错，错的是两把尺子不是同一把。** 旧路径比的是我们的**网格**与 E3D 的面片：0.0003 mm
——两边的离散是同一份，这说明网格本身对得严丝合缝。新路径比的是我们写出去的**解析 `Cylinder`**
与 E3D 的面片：1.2564 mm，正好是那只圆盘自己的弦高（内接多边形的 AABB 比外接圆小这一个量）。
自校闸按候选自带的弦高放行了它，**这是对的**——解析面比面片更接近真实形体；
而 `rvm_compare` 给 `BEND` 的容差是按 noun 查表的 `CHORD_TOL_MM` = 0.5 mm，1.2564 > 0.5 → `off`。

**判据缺的那一格**：基准侧是 FacetGroup、生成侧是解析图元时，残差里含的是**基准自己的**离散缺口，
而 `chord_tolerance_mm()` 只按 noun 查表，不看基准的段数。修法是在这种配对上把基准面片的弦高
加进容差。**本期不改**——§七 写明这一轮不动判档口径与阈值；换判据要单独一轮，
否则说不清是谁的功劳。已记进 §八。

**顺序**：M0 → M1 → M2 → M3；M0 可以单独发。M4 视 M3 结论。M5 由 §三 反推出来，独立可发。
M6 是 M4 自校闸的修正、M7 依赖 M6（没有弦高这把尺子，认出来的圆台照样会被闸门退回去）。

---

## 六、验收标准（整件事）

1. `gen_ams --rvm out.rvm` 对样本 BRAN 产出非空 `.rvm`，rvm-rs 与 `rvm_read` 都解得开。
2. 写出记录满足 §2.3 的 bbox 不变量，残差 ≤ 1e-4 mm。
3. `rvm_compare --baseline a.rvm --generated b.rvm` 的逐件判档与现行路径**一致**（M3）。
   （M7 那轮出现一件例外，成因是判据缺一格、不是数据通路，见 M7 末尾与 §八。）
4. `--facets` 从必需降为可选，去掉它之后判档不变（M0）。
5. 退出码语义不变：0 = 全部在容差内，1 = 有超限。
6. 全部命令、样本路径写进 evidence，他人可独立复跑。

---

## 七、不做的事

- **不追求字节相同**。`HEAD` 带时间戳 / 用户名 / 产品横幅，f32 舍入两侧不会一致，
  E3D 的三角化是它自己的。判据是**结构与几何**，不是 `fc /b`。
- **不做 `.att`**。身份在生成侧本来就有 refno，不需要靠 ATT 反查。
- **不改 rvm-rs**。M1 之后我们不再依赖它的帧行为；上游修不修都不影响。
- **不动判档口径与阈值**。这一轮只换数据通路，换判据是另一件事——两件一起换就说不清是谁的功劳。
- **不复活 `gen-model/src/rvm/`**。那套文本 RVM + Arango/MySQL 栈按 §4.1 判死，
  本计划落地后应单独发一个删除 PR。

---

## 八、风险与未决

| 项 | 说明 | 处置 |
|---|---|---|
| `RectangularTorus` / `Sphere` / 两种 `Dish` / `Pyramid` 的 bbox 帧未验 | 6 份样本里一条都没有 | `local_box()` 的规范盒已按 rvmparser 口径写好（含 RTOR），等有样本直接跑。M0 的自证分支会把它们记成 `bbox_frame_unknown`，不会静默错 |
| ~~`WALL`/`STWALL` 的帧例外成因不明~~ | ✅ §3.1 结掉：记录 bbox ≡ `宿主包围盒 − 平移`，无分支；帧差异发生在宿主的几何构建器里，写出端看不见 | **读侧的自证分支永久保留**——帧不由任何可查的表决定 |
| ~~`flags` 的其它取值~~ | ✅ §3.4 结掉：`flags=2` = 平移越 ±1e9 mm 的远场逃逸，记录尾部多 3 个 f64；`CNTB` 第二个头字同理 2→4 | 写出端固定 1（坐标远不到 1e9 mm）；读侧允许并识别那 24 字节尾巴 |
| `OBST`/`INSU` 的通道↔tag 对应 | §3.3 的结构（多一个等级字）是确定的，但哪个通道对 `OBST`、哪个对 `INSU` 是按选项顺序推定的——两个 tag 常量在 BSS 里，静态读不出 | M5 的自证不依赖这个对应；真拿到带 `OBST` 的样本时一次就能定 |
| **解析 ↔ 面片配对时，基准自己的弦高没进容差** | 生成侧写解析图元、基准侧是 FacetGroup 时，AABB 残差里含的是基准的离散缺口，而 `chord_tolerance_mm()` 只按 noun 查表。`23198` 的 `BEND 2` 因此从 `exact` 掉到 `off`（1.2564 mm vs `CHORD_TOL_MM` 0.5 mm），见 M7 末尾 | 本期不改（§七：这一轮不动判档口径与阈值）。修法是这种配对上把基准面片的弦高加进容差，单独一轮做 |
| 尾部补零的块大小 | 样本长度都是 160 的整数倍，成因不明 | 不补零。若第三方工具（Navisworks）因此读不动，再按 M2 验收 3 的结果补 |
| 三角数膨胀 | 一个三角形一个 polygon，`C-IY-1R330-B` 这种 90 个 FacetGroup 的样本文件会比 E3D 大若干倍 | 先不管。真成问题时把共面三角合并成多边形（RVM 的 contour 本来就支持任意边数） |
| 祖先链取不到 | 生成侧从 root 往上走属主链，若 root 就是 SITE 则没有祖先 | 允许零祖先；`rvm_compare` 的 key 只用成员名，不依赖祖先层数 |
| 与增量的关系 | `increment.rs` 只重建受影响单元，导出的是全量还是增量子树 | 本期只做**全量子树导出**（与 `gen_ams --root` 同口径）。增量导出等 M3 之后再议 |

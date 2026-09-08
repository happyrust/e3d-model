# e3d-model 开发计划（2026-09-04 凌晨 · Core3D.dll 复核第二版）

> 承接 `docs/2026-09-02-development-plan-core3d-recheck.md`（下称「09-02 计划」）与 09-01 计划。
> 本轮复核手段：读 HEAD `d440af3` 全部源码 / 工作树 / 最新报告 + 对活体 IDB 用 ida-bridge 反编译
> （`Core3D.dll` = `idalib-25128`；`core.dll` = `idalib-33060` 本轮未动）。09-02 计划里已 ✅ 的项不再重述；
> **本文只写还欠的、以及这一轮被反编译改写了口径的**——重点是结构线（G1）的施工图：SCTN 两端切面的
> 精确算法、SCTN 负体的来源、FITT / PFIT / CMFI / FIXING 的建体路与标架、GENSEC 段建法。
>
> 复核规矩不变：以反编译原文为准，猜不出的写「待核」；每条差距都带**实测计数 + Core3D 锚点 + 验收判据**。
> 属性 / noun 码的反查公式（本轮钉死，可直接算）：`hash = 27^4 + Σ (ord(c_i) − 64) · 27^i`（`i` 从 0 起、首字母最低位）。
> 例：`POSS 919690`、`DRNS 916114`、`DRNE 640552`、`SPRE 643429`、`CATR 900345`、`GSTR 900835`、`GMRE 643336`、`NGMR 895415`。

---

## 一、账目快照：HEAD `d440af3` 还欠什么

| 库 / 快照 | generated | catalog_pending（记账不建） | 其它 |
|---|---:|---|---|
| ams7997（`out/base4c4-ams7997-full`，09-02 20:25，基线 `4c4ec21`） | 50501 | **FITT 1819 / SCTN 1762** / FIXING 180 / STWALL 170 / PFIT 57 / CMFI 4 | BRAN 830 / HANG 112；TMPL 17；**unknown_nouns DDAT 137 / DDSE 76 / DPSE 8 / LNKI 65 / LNKS 65**（新露头）；隐式管身 6433 → built 4787（**共轴门之前的数字**，见下） |
| ams8000+目录库（`out/audit-head-ams8000-full`，09-02 21:05） | 2626 | **SCTN 528** / FITT 22 | BRAN 560；隐式管身 3119 → **built 150**（zero_length 2564 / **misaligned 182 / reversed 217** / missing_ppoint 6 / bore_mismatch 24）；1 mrad 双端门后 built **129**（evidence §4）；failed 2（数据侧悬空） |
| ams1112 / aps250110 / aps250209 | 未复跑 | 同 09-02 计划 §一（GENSEC 4203 / FIXING 1179 / STWALL 302 / PFIT 263 …） | aps250110 host_negatives NCYL 1282 / NBOX 232 / NXTR 45 |

结构线仍是最大的账：**SCTN 2330 / GENSEC 4203 / FITT 1990 / FIXING 1179 / STWALL 302 / PFIT 263**（≈ 1 万件）。

**09-02 计划之后落地的**（都已入库）：`2846da1 / 8924f32 / 4e29cd5` 隐式管身**双端共轴 1 mrad 门 + 站点按目录 GTYP**
（evidence `20260902-ams8000-bran-audit/rvm-pending-items-2026-09-02.md`：21 条支管 + B1 里 19 条 TUBE 数与几何逐根同 E3D）；
`f5af760` `rvm_compare` 管身按几何配对（`TUBE n` 序号不可靠）；`5978e78` + `d440af3` **SCTN 截面求值（G1-1a 第 1 步）✅、
第 2 步的两个函数已写好（`section::section_placement` / `section_local_solid`）但尚未接进管线**——`category::is_model_unit`
对 `Catalog` 仍只认 `SBFI | WALL`，`elmodl::model_element` 没有 SCTN 臂，所以两库 SCTN 仍整数落 `catalog_pending`。
`d440af3` 还带了 `profile_rings` 零面积剖面回 `Ok(None)`（目录把图元参数化关掉）与增量侧 `ElementDiff::is_placement_only`
便宜路径判据。

**工作树（未提交，并行会话在改）**：`src/route.rs` 一处——`station_verdict` 从「`GTYP=ATTA` 穿过、其它为站点」改成
「**只有 `GTYP=ANCI` 是站点**，`ATTA / PCLA` 等一律穿过」（PCLA = 管夹）。方向对（E3D 只在 ANCI 两侧断管），但要补它的证据行
（哪条支管上 PCLA 被当站点拆了管）再收；本文记为 G0-8。

---

## 二、Core3D 复核的新结论

### 2.1 `DB_Sctn` 全流程（构造函数 `sub_107387F0` + `DB_makeCSG@0x10738C30`，逐行核过）

```
ctor:   length = |POSS − POSE| → +128
        sub_10717DA0(noun@+24, T, +136, +160, +184, +208)   // GTSTDT：切面 A(点 +136, 法向 +160)、B(点 +184, 法向 +208)
        JOIS(912394) → 引用件；若其 OWNER 的 TYPE == SUBJ(730315) 先建 SUBJ builder 再建 JOIS builder（父=SUBJ）→ +112
        JOIE(636832) 同上 → +116
        MEML(777262) → 引用件 builder（父=this）→ +120
makeCSG(T):
        if FIRST TYPE TMPL 找到成员 → return 0                      // sub_107169B0；带 TMPL 成员的 SCTN 自己不出几何（§2.4）
        profile = vtbl[2](this, T, 0)                                // 截面 → 直段实体（§2.2，含切面延伸）
        xsec1   = cut(profile, A, 新建 Section) ; cut(xsec1, B, 追加同一 Section)   // 法向 x=y=0 的那一面跳过
        MEML    → sub_10739770 → 两点 → gm_CreateStraight（一条线，见 §2.3）→ 逆变换后加进顶层组合
        withEndPrep = xsec1 − JOIS 关节负体 − JOIE 关节负体            // sub_10722D80(mode 0) → 负体 vtbl[2](joint, ·, 2)
        beam    = withEndPrep − mode-3 负体（FITT / CMFI+SBFI / SNOD→SJOI / SCOJ→SUBJ，§2.3） [byte_10D99C00]
                  − TMPL 属主负体（mode 7）；TMPL.owner==FIXING 时再扫 FIXING 节点
        top     = Combination{ MEML 线, beam ?: profile }
```

调试串就是四个阶段名：`profile` → `xsec1` → `withEndPrep` → `beam`（`"CSG id's for SCTN"`）。

**helper 类坐实**：所有 builder 的 `this[26]` 是 **`DB_Scan_to_GML`**（vtable `0x10b4e434`；线框孪生 `DB_Scan_to_Pline@0x10b4f954`，
ACC 版 `ACC_DB_Scan_to_GML@0x10b724d8`）。本轮用到的槽位：

| 字节偏移 | 槽 | 函数 | 语义 |
|---|---:|---|---|
| +32 / +36 | 8 / 9 | `sub_1072C010` / `sub_1072BEA0` | 空组合 / 新建组合 |
| +48 | 12 | `sub_1072BF60` | `gm_CreateStraight(p1, p2)`——**线**，受 p-line 开关 `byte_10E5072D` |
| +52 / +88 | 13 / 22 | `sub_1072C070` / `sub_1072A380` | 由树建差集节点 / 往差集加负操作数 |
| +68 / +72 / +76 | 17 / 18 / 19 | `sub_1072A120` / `sub_1072A1E0` / `sub_10729F50` | 加成员（+76 带变换） |
| +84 | 21 | `sub_1072A2A0` | 加图元（带变换；`byte_10D99C00` 关且组合非空时改为 `gm_RemoveTree`） |
| +100 / +104 | 25 / 26 | **`sub_1072A790`** / `sub_1072B780` | **目录库图元 → libgm** / 设计库图元 → libgm（按 `MDBTYP` 分路） |
| +112 | 28 | **`sub_1072BA70`** | **切平面**（§2.2） |
| +116 | 29 | `sub_1072A570` | 带 100 × L 矩形剖面的切面（`gm_AddCutProfilePoint` × 4），SCTN 未用 |

### 2.2 两端切平面的精确口径（G1-1a 第 3 步施工图）

**平面从哪来**（`dbboxlib/GTSTDT@0x1036CEC4`，SCTN / WLJOIN 分支）：

```
A.点  = TRAVCI(T⁻¹, POSS)          → 局部 (0, 0, 0)
B.点  = TRAVCI(T⁻¹, POSE)          → 局部 (0, 0, L)
A.法向 = MVMLTI(R⁻¹, DRNS)  ; |z| < 0.001 → (0,0,1)（unk_10B69110）; z < 0 → 取反   // 指向梁内(+Z)
B.法向 = MVMLTI(R⁻¹, DRNE)  ; |z| < 0.001 → (0,0,1)                ; z > 0 → 取反   // 指向梁内(−Z)
```
T 是 SCTN 局部标架（`pllib/PICAOR` + `pllib/PICOPS`，即 `section.rs::section_placement` 已实现的那一半）。
GENSEC 分支不变换（`MUNIT`，标架恒等）：点直接 `GATPS1(POSS/POSE)`，法向 `GATDIR(DRNS/DRNE, 缺省方向)`。

**怎么切**（`DB_Scan_to_GML` 槽 28 `sub_1072BA70(点, 法向, 树, 新建?)`）：
- 总开关 `byte_10D99C00 && !byte_10E5072C`（几何选项块 `sub_10715270/E0`：模式 `≠2` 开负体、`==1` 关切面，见 §2.6）；
  法向全零 → 返回 0（树被丢掉；GTSTDT 已保证不会全零）。
- 切面标架：`takeToZAxis(法向) · rotateY(−90°)` 取逆，再平移到点——切面局部 +X = −法向，切面 = 局部 YZ 平面，
  **去掉法向背面（梁外）那半空间**。
- `新建=1`：`Section = gm_CreateSection(树, op 3, cutType 0)` 再 `gm_AddMember(CutSurface, Section)`；`新建=0`：**第二面直接追加进同一个 Section**。
  `DB_makeCSG` 对 A 传 1、对 B 传 0——这个旗不是「留哪一侧」，是「新建 / 追加」。
- `DB_makeCSG` 在法向 `x == y == 0` 时跳过该面（垂直于轴的面对挤出体是空操作）。

**切之前先延伸**（这一步 09-02 计划漏了）：2D 截面成员进 libgm 时（槽 25 `sub_1072A790` 的 SCTN 臂 → `sub_107143D0(noun, T, &L, A.点, A.法向, B.点, B.法向)`
→ `gml_bkend/CRC2DI@0x103482A5`：SREC → `CRTPYR@0x103484EA`，SANN → `sub_10349627`，SPRO → `QSPROF` + `sub_1034A591`），
**先把直段拉长到两平面之外再切**——`gml_bkend/XSPYR@0x10348685`（SREC 臂，逐行核过）：
1. 对截面 4 个角点各作一条平行于轴的棱，与平面 A（`|A.法向.z| > 1e-4` 时）、平面 B 求交（`POSCAL`）：
   `z_min = min(0, 各棱交 A 的 z)`，`z_max = max(L, 各棱交 B 的 z)`；再对锥度轴线做同样夹取；
2. 若 `z_min < 0 || z_max > L`：新长度 `z_max − z_min`，尺寸 / 偏移按新区间重新插值，变换沿轴平移 `(z_min + z_max)/2` 修正；
3. 然后才交给 `gm_CreateSection` 切。

所以斜切面在截面的远侧**会长出材料**（梁端一直画到平面），不是只在 `[0, L]` 里削。SPRO 臂 `sub_1034A591` 同名逻辑待逐行核，
先按「对全部折线化顶点取 `z_A(x, y) = −(n_x x + n_y y)/n_z`、`z_B(x, y) = L − (n_x x + n_y y)/n_z` 的 min / max」推广。

**Rust 落法**（manifold-csg）：`extrude_rings(rings, z_max − z_min)` 平移到 `z_min`，再与两个半空间求交
（`(p − A.点)·A.法向 ≥ 0` 且 `(p − B.点)·B.法向 ≥ 0`）；`|n.z| ≤ 1e-4` 的面按 GTSTDT 口径已变成轴向 → 跳过。

### 2.3 SCTN 负体的来源重新定性（改写 09-02 计划 §2.4 第 4 步）

- **MEML 不是「端部件」**：`sub_10739770` 只算两点（`sub_10717620`）再 `gm_CreateStraight`——是一条**线**（member line），
  受 p-line 开关，属 `REPR CL` 类表现；实体建模里删掉这一步。
- **JOIS / JOIE 是负体源**：`sub_10722D80(joint, sctn, T, mode 0, profile, &out)` 把关节 builder 挂到 SCTN 的 helper 上，
  `byte_10D99C00` 开时调 `joint->vtbl[2](·, 2 − (mode≠0))` = **负体模式 2** 取关节的负几何，`out` 首次为空时由 profile 建差集节点，
  再把负体加进去。JOIS 的 TYPE 是 SUBJ 时其首个成员 builder 再挂一次。
- **mode 3（`sub_10721600`）扫 SCTN 自己的成员链**：`FITT(939930)` / `CMFI(713316)` → mode 3；CMFI 下的 `SBFI(713035)` → mode 6；
  `SNOD(621505)` 下 `SJOI(719812)` → mode 1、`SCOJ(739306)` → mode 2、SCOJ 下 `SUBJ(730315)` → mode 1。都是「从梁上扣」。
- **NAPP 位有了 Core3D 出处（改写 09-02 §2.6 的「bit 1/2 无出处」）**：槽 25 `sub_1072A790` 建图元前读一次 `NAPP`（`sub_107177A0`）：
  - `NAPP == −1`：只有 builder mode ∈ {0, 3, 4, 5, 6} 放行（关节 / 模板负体被拒，FITT-on-SCTN 放行）；
  - 否则 mode 0 → `NAPP & 1`；mode 1–6 / 8 / 9 → **`NAPP & 2`**；mode 7（模板）→ `NAPP & 4`；位不满足 → 不建。
  即 bit0 = 自身，**bit1 = 扣到所属型材 / 邻件**，bit2 = 模板孔。G2-1 的账目字段照此拆三档。
- 另有一个布尔属性 `174799030`（反查 `DESXCL`，待核名义）为真时该图元整个不建（槽 25 首判）——语料里若出现要进表现过滤。

### 2.4 FITT / PFIT / CMFI / FIXING：基类 builder + 目录路（G1-1c 施工图）

- 构造函数全是纯基类：`DB_Fitt::ctor sub_1073A170`、`DB_Fixing::ctor sub_1073A280` 只调 `DB_Item::ctor sub_107210A0` 再换 vtable；
  `DB_Fitt / DB_Pfit` 的 makeCSG 就是基类 `sub_107223D0`，**`DB_Fixing` 自带 makeCSG `sub_1073A560`**（自身正体 → 自身负体差集 →
  扫子树里的 FIXING 节点按 mode 7 挂负体 → 顶层组合；**不遍历成员链**）。
- **正体来源 = 槽 2 `sub_10722680`（所有 DB_Item 子类共用，SCTN 也是）**：
  1. 取几何源 `v41`：自身是 TMPL / 有 TMPL 成员（`FIRST TYPE TMPL`）→ TMPL；`ICCOMP` 族 → 自身；否则 `SPRE(643429)` → SPCO →
     `CATR(900345)`（或元素直接带 `CATR`）→ 目录件（SFIT / SCOM / SPRF…）；
  2. 槽 9（`sub_107385E0` → `sub_10718770` → **GTSTDT 的变换出参**）设 builder 变换；
  3. 槽 5 `sub_10721F00`：SCTN / GENSEC 走 `GSTR(900835)`（→ GMSS，2D 截面），其它正体走 `GMRE(643336)` → GMSE、负体走 `NGMR(895415)` → NGMS；
     逐成员 `sub_107213E0` → 槽 8 `sub_10721A30` → helper 槽 25 / 26 → `sub_10714510`（目录图元 → libgm，待核是否即 CRCATI 的 C++ 孪生）；
     源是 TMPL 时改为遍历 TMPL 成员，只取 `LPRMTV` 且 `INCOMP` 正负匹配的图元（**FIXING 的几何就是它的 TMPL 里的图元**）。
  4. 组合为空（`gm_QueryNumberOfMembers == 0`）→ `gm_RemoveObject`，返回 0。
  → **FITT / PFIT / CMFI 与 SBFI / 路由成员走的是同一条 GMRE 路**，Rust 侧就是 `model_catalogue_member_with_cache`；差别只在标架。
- **标架 = GTSTDT → `pllib/PICAOR@0x103A59EC`（方向）+ `pllib/PICOPS@0x103992EC → pllib/GCORG3@0x103AA08C`（位置）**，PICAOR 按 noun 分路：
  `SCTN` → `GSCORI@0x103A306C` + `ROTNAX@0x103A3398`（已实现）；`FITT / CMFI` → **`pllib/GFIORI@0x103B27D4`**（读 `POSL` p-line 键、`OWNER`、
  `BANG`、`DRNS/DRNE`，`ORNMAT / ROTMAT`）；`PFIT / COFI / CMPF` → **`pllib/GPFORI@0x103B2CE4`**（`PFRE`、`SJUS`、`BANG`、`DCEN/DTOP/DBOT`、`CSTRAM`）；
  `SBFI` → `sub_103B30AC`；关节族 → `sub_103AB5EC` + `sub_103AFF74`；`GENSEC` → 错 210；**其它 noun（含 STWALL）→ 错 210**。
  GCORG3 按同一组 noun 分路取位置（`CLIMB` 到 STRU、`GATPOS`、`POS`）。
- **STWALL**：IFCOMP 族但 PICAOR 没有它的臂 → GTSTDT 报 210 → 变换不由 GTSTDT 给。它与 WALL（IPFCOM）不是同一条路——
  09-02 计划「与 WALL 同走 GTSTDT」不成立；STWALL 的放置从哪来待核（先看基类成员链的变换 `sub_10716FC0` 那条，再看 `MapNounToBuilderType` 缺省型 47）。
- **模板实例机制**（低优先级，语料待普查）：几何源是 TMPL 且 `TMPL.OWNER` 的 TYPE 是 **`TMAR`(886835)** 时，在 TMAR 下找 `ORRF`(663162) 指回该 TMPL 的成员——
  ams7997 17 只 TMPL 的属主是 FIXING 还是 TMAR 先查。
- 基类 makeCSG 里的一条旁支：noun 是 **`BEND`(620516)** 且目录几何为空时走 `sub_1072D820 / sub_1072D8F0(AM_CoEdge)` 参数化回落——G0-2 对拍若遇到无 GMRE 的 BEND 从这里进。

### 2.5 GENSEC 段建法（`sub_107318E0`，845 行，与 09-02 §2.4 一致，补细节）

截面 `D2_Profile::rotateBy(builder+128)`（BANG 弧度）→ `gm_CreateProfile`；`LINE(640317)` 特判；弧段
`gm_CreateRevolution(0.0, 扫角, 轴点, 180.0, profile)`（`" REVO segment"`，`gm_AddCurve` 逐边）；末截面非空 → `gm_CreateRuledSolid(prof₁, T, prof₂)`
（`" RULED segment"`，变截面）；否则 `gm_CreateExtrusion(T, profile)`（`" EXTR segment"`）；首末 POINSP 的 `posFrom/posTo/vec1/vec2` 任一分量差 > 1e-6
时再挤一段 `" Start-mitre extension"` / `" End-mitre extension"`；`"!!! D3_Transform::map failed for spine segment"` 是它唯一的失败诊断。
槽 25 的 GENSEC 臂是 `sub_10732FF0`。GENSEC 在 GTSTDT 里**标架恒等、切面点直接取 POSS/POSE**。

### 2.6 几何选项块（负体 / 切面 / p-line 的总开关）

`sub_10715270(a1..a7)` / `sub_107152E0(a1..a6)`（Fortran 调用约定，`GTGEOM` 邻近的 `sub_10342683` 等 12 处调）：
`byte_10D99C00 = (a2 ≠ 2)`（负体 + 切面）、`byte_10E5072C = (a2 == 1)`（只关切面）、`byte_10E5072D = a3`（p-line / PLIN 画不画）。
模式 0 = 全 CSG、1 = 有负体无端切、2 = 都没有。**对应哪个 `REPRE` 关键字待核**（大概率是 holes / cutbacks 类开关）；
`elmodl` 现在按「负体一律扣」等价于模式 0，先加一个 `E3D_REPRE_HOLES`-型环境变量占位（缺省 0），别写死。

### 2.7 新露头的 5 个 unknown noun

ams7997 `unknown_nouns` **DDAT 137 / DDSE 76 / DPSE 8 / LNKI 65 / LNKS 65**：`data/noun-family-matrix.json` 里五个都是 `families = []`、`gb = 0`
——不在 GTGEOM 级联、无几何位，是数据 / 链接元素。归 `NonGraphic` 并在 `noun_coverage` 里钉住（G2-4）。

---

## 三、差距清单（按优先级；每条带计数 / 锚点 / 验收）

### P0 —— 先收口、再对拍

- **G0-8（新）工作树 `route.rs`：`GTYP` 只有 `ANCI` 是站点**。补证据（哪条支管的 PCLA 被当站点拆管、改后 E3D TUBE 数对上）后一次提交；
  `bran_audit` 与 21+B1 回归门复跑不得变差。
- **G0-2 管道端到端 RVM 对拍**：生成侧三条规则已坐实（双端共轴 1 mrad、ANCI 站点、C 撤回），21+B1 里 19 条逐根同 E3D。剩：
  ① `/M-OR-1R323-A` 1.75 mm / 0.68 mrad 尾管（E3D 不画，我们画）——留观；② ams8000 管道区 / ams7997 支撑区**全量** TTY 导出仍待人工；
  ③ ams7997 的管身账（built 4787）是共轴门之前的，**复跑一次刷新基线**并把 `misaligned / reversed` 写进 notes 归因。
- **G0-3 成员 stub 三 case**（`GTTUBG :137–:201`）不变。
- **G0-7 顶层 NSRT 8 件**：现代路结论已定（聚合不扣），旧路 `GTCATG / LNEGIT`（core.dll `idalib-33060`）与 E3D `REPR` 人工确认待做。**没拍板前维持现状。**

### P1 —— 结构线（≈ 1 万件）

- **G1-1a SCTN 直段（2330 件）**——第 1 步 ✅；剩三步，按 §2.1–2.3 的口径：
  2. **接线**：`category::is_model_unit` 的 `Catalog` 臂加 `SCTN`；`elmodl::model_element` 加 SCTN 臂（`section_catalogue → evaluate_section_profile →
     section_placement → section_local_solid`）；`pipeline` 的 `Category::Catalog` 走模型单元路；`NoSpre` / 空截面记 `centreline_only`（E3D 只画中心线），
     链断记 `catalogue_chain_broken`。增量侧 `is_model_unit` 同步生效（SBFI 那次静默漏件的教训）。
  3. **两端切面**：GTSTDT 口径求两平面（DRNS/DRNE 进局部系、z 门 0.001、正负号规约）→ 先按 XSPYR 延伸挤出区间 → 再两个半空间求交；`x=y=0` 的面跳过。
  4. **负体**：JOIS / JOIE 关节负体（mode 2）+ 成员链 FITT / CMFI(+SBFI) / SNOD→SJOI / SCOJ→SUBJ（mode 3 / 6 / 1 / 2）→ 统一过 `NAPP & 2` 门（`NAPP = −1` 只放 mode 3–6）；
     **MEML 删掉**（是线）。关节 / FITT 的负几何本身走 G1-1c 的目录路（NGMR → NGMS），所以第 4 步排在 G1-1c 之后。
  验收：ams8000 `SCTN 528 → generated 4 + centreline_only 500 + chain_broken 24`；ams7997 `1762 → 1761 + 1`；
  RVM：`/-KX-CAM-S4001-V1`（`evidence/20260902-g11a-section-profiles/rvm-S4001-groups-site46775.json`，72 顶点 / 14 面、bbox 387.5 × 40 × 40）逐顶点 ≤ 弦容差，
  L40X4 / HW150 / C80 三样本同；两库其它 noun 的 `model.obj` 逐字节不变；`accounts_for` 不破。
- **G1-1b GENSEC 扫掠（4203 件）**：按 §2.5；`examples/spine_probe` 先把 `STRU_GensecPline::Geometry@0x106274f0` 的线弧分段对齐；依赖第 1 步截面求值。
  验收：aps250209 `GENSEC 2698 → 0 pending`；`aps250110-handrail` 逐件 RVM。
- **G1-1c FITT / PFIT / CMFI / FIXING（FITT 1990 / FIXING 1179 / PFIT 263 / CMFI 7）**：目录路直接复用 `model_catalogue_member_with_cache`（§2.4 坐实同路），
  新活只有标架——先逐行翻 **`GFIORI@0x103B27D4`**（FITT / CMFI）与 **`GPFORI@0x103B2CE4`**（PFIT / COFI / CMPF）+ `GCORG3@0x103AA08C` 的对应臂，
  写进 `section.rs` 旁边的 `fitting.rs`；FIXING 的几何源是 TMPL 成员图元（§2.4 第 3 点），与 G2-1 一起做。**G2-5 `place_cached_part` 收口仍排在前面**。
  验收：五库 `catalog_pending` 只剩明确「无 GMRE / 表现过滤」的记账；ams1112 RR001 / WF 区含 FIXING · PFIT 的 facets 重导出后 `rvm_compare` 逐件裁决。
- **G1-1d（新）STWALL（302 件）**：先定放置来源（§2.4：不走 GTSTDT），`examples/element_probe` 取 3 只看 POS / ORI / SPRE 链；定了再接目录路。
- **G1-2 复合聚合 builder**：按 09-02 §2.5（成员各建、`isWanted` 过滤、Complex(5) 聚合、不做布尔）。

### P2 —— 收口项

- **G2-1 模板孔 / 邻件负体**：`NAPP & 4`（模板，`addHolesBelowTemplate`）+ **`NAPP & 2`（关节 / 配件扣型材，本轮新出处）**；账目改
  `template_holes_applied / neighbour_cuts_applied / hole_definitions` 三档，`NAPP = −1` 忠实不扣。
- **G2-4 分类清扫**：§2.7 的 5 个 noun → `NonGraphic`；扶手线走 `ASL_MDR_*` 插件（09-02 §G2-4 不变）。
- **G2-5 `place_cached_part` 收口**：`catalogue.rs` 现 15 处「取网格→放置去重→摆放」样板，抽成一份；**排在 G1-1c 之前**。
- **G2-9（新）几何选项开关**：§2.6 的三个字节对应的 `REPRE` 关键字核清后，`E3D_REPRE_*` 环境变量补一档，`elmodl` 的负体 / 端切都从它取。
- **G2-3 / G2-6 / G2-7 / G2-8** 照 09-01 / 09-02 计划。

---

## 四、建议执行顺序与里程碑

1. **M0（半天）**：G0-8 `route.rs` 收口入库；ams7997 全量复跑刷新管身基线（G0-2 ③）；G2-4 五个 noun 归类（一条提交 + `noun_coverage` 断言）。
2. **M1（1 天）**：G2-5 `place_cached_part` 收口（`catalogue.rs` 并行会话改动落地后再动）；G2-9 开关占位。
3. **M2（2 天）**：G1-1a 第 2 步接线 → 第 3 步端切（含 XSPYR 延伸）→ RVM 对 S4001 / L40X4 / HW150 / C80 逐顶点；第 4 步等 G1-1c。
4. **M3（2 天）**：G1-1c 标架三函数反编译落地 + FITT / PFIT / CMFI 接目录路；G1-1d STWALL 定性；回头补 G1-1a 第 4 步负体。
5. **M4（2 天）**：G1-1b GENSEC；G1-2 复合聚合；G2-1 三档账目。
6. **贯穿**：G0-2 参考侧 RVM 一到就裁决；G0-3 stub 普查随 M3 的 `catalogue_probe` 顺手做；G0-7 等人工。

**验收判据（每步都过，沿用 09-01 §四）**：`accounts_for` 硬判据不破；RVM 局部系逐件对拍新增几何 100% 配对、偏差在已证容差内；只读库身份（DB SHA-256）前后一致；
`cargo test` 全绿且不放宽断言；触碰复用池 / 求值器的改动两库 `model.obj` / `elements.json` 逐字节一致或逐件归因；**新补的每条臂都先贴 Core3D 反编译锚点，再写代码**。

---

## 五、请拍板的几件事（标注在这里即可）

1. **G1-1a 端切的延伸口径**：按 XSPYR「先拉长到平面外再切」（§2.2）做，还是先只在 `[0, L]` 内切、等 RVM 出现斜切样本再补延伸？本文按前者排。
2. **G1-1a 第 4 步的顺序**：负体（关节 / FITT 扣梁）等 G1-1c 目录路落地后再做（本文排法），还是先用现有 SBFI 路手接 FITT 负体？
3. **G1 结构线顺序**：SCTN（有 RVM 参考）→ FITT 族 → GENSEC（本文排法），还是 GENSEC 提前（aps 两库 4203 件、量最大）？
4. **G2-9 开关**：`REPRE` 关键字核清前，负体 / 端切缺省全开（模式 0）是否接受？
5. **G0-7 顶层 NSRT** 与 **参考侧全量 RVM 导出**（ams8000 管道区 / ams7997 支撑区 / ams1112 含 FIXING · PFIT 的 RR001 · WF 区）能否本周排上人工？

---

## 六、本轮锚点表（Core3D.dll = `idalib-25128`，便于直接接手）

| 用途 | 地址 |
|---|---|
| SCTN | ctor `sub_107387F0`（POSS/POSE → 长度 +128，切面 +136/+160/+184/+208，JOIS +112 / JOIE +116 / MEML +120）、`DB_makeCSG@0x10738C30`、槽 5 `sub_10738C00` |
| 切面 | `dbboxlib/GTSTDT@0x1036CEC4`（POSS/POSE/DRNS/DRNE，z 门 0.001，常量表 `0x10B690C4…0x10B6910C`）、桥接 `sub_10717DA0` / `sub_10718770`、`DB_Scan_to_GML` 槽 28 `sub_1072BA70`（`gm_CreateSection(·,3,0)` + `gm_CreateCutSurface`）、槽 29 `sub_1072A570` |
| 延伸 | 槽 25 `sub_1072A790` SCTN 臂 → `sub_107143D0` → `gml_bkend/CRC2DI@0x103482A5` → SREC `CRTPYR@0x103484EA` → **`gml_bkend/XSPYR@0x10348685`**；SANN `sub_10349627`；SPRO `sub_1034A591` |
| SCTN 负体 | 关节 / 模板挂接 `sub_10722D80`（mode → 负体模式 `2 − (mode≠0)`）、mode-3 扫描 `sub_10721600`（FITT 939930 / CMFI 713316 / SBFI 713035 / SNOD 621505 / SJOI 719812 / SCOJ 739306 / SUBJ 730315）、FIXING 扫描 `sub_10721930`、TMPL 成员探测 `sub_107169B0`（`DFINDI(FIRST, TYPE, TMPL)`） |
| MEML 线 | `sub_10739770` → `sub_10717620` → 槽 12 `sub_1072BF60`（`gm_CreateStraight`）；p-line 表现 `sub_10721CC0`（`PSTR 900844 / PLIN 813904 / PKEY 1027474`） |
| 基类 builder | ctor `sub_107210A0`、makeCSG `sub_107223D0`（BEND 620516 回落 `sub_1072D820 / sub_1072D8F0`）、正体 `sub_10722680`（SPRE / CATR / TMPL / TMAR 886835 / ORRF 663162 / SFIT 931843）、成员几何 `sub_10721F00`（GSTR / GMRE / NGMR）、单图元 `sub_107213E0` → `sub_10721A30` → 槽 25/26 → `sub_10714510` |
| FITT 族 | `DB_Fitt::ctor sub_1073A170`、`DB_Fixing::ctor sub_1073A280` / makeCSG `sub_1073A560`、`DB_Pfit` vtable `0x10b50548`；标架 `pllib/PICAOR@0x103A59EC`、`pllib/PICOPS@0x103992EC`、`pllib/GCORG3@0x103AA08C`、**`pllib/GFIORI@0x103B27D4`**（POSL 781909 / BANG 679457 / DRNS / DRNE）、**`pllib/GPFORI@0x103B2CE4`**（PFRE 643156 / SJUS 921016 / DCEN 810733 / DTOP 857848 / DBOT 936094 / `CSTRAM@0x108e9888`） |
| GENSEC | `DB_Gensec` vtable `0x10b4faa0`（makeCSG `sub_107311D0`，槽 5 `sub_10731180`，槽 9 `sub_10731670`）、段建法 `sub_107318E0`、槽 25 臂 `sub_10732FF0` |
| 选项开关 | `sub_10715270` / `sub_107152E0`（`byte_10D99C00` 负体+切面、`byte_10E5072C` 关切面、`byte_10E5072D` p-line）、写 1 的 `sub_107297C0` |
| vtable | `DB_Item 0x10b39eb4`、`DB_Sctn 0x10b5022c`、`DB_Fitt 0x10b50494`、`DB_Fixing 0x10b5050c`、`DB_Scan_to_GML 0x10b4e434`、`DB_Scan_to_Pline 0x10b4f954`、`ACC_DB_Scan_to_GML 0x10b724d8` |
| 属性 / noun 码 | POSS 919690 · POSE 644128 · DRNS 916114 · DRNE 640552 · JOIS 912394 · JOIE 636832 · MEML 777262 · BANG 679457 · SPRE 643429 · CATR 900345 · GSTR 900835 · GMRE 643336 · NGMR 895415 · OWNER 10206636 · TYPE 642215 · REF 535968 · FIRST 11547609 · NEXT 942746 · POS 545713；TMPL 779672 · TMAR 886835 · FIXING 108608856 · SCTN 821683 · GENSEC 46619692 · WLJOIN 206501990 · PANE 640105 · PFIT 931840 · COFI 713370 · CMPF 661557 · STWALL 179132626 · ENDATU 312510290 |

> 运维备注：`idalib-9944` 已不在，Core3D 现为 `idalib-25128`，`core.dll` 为 `idalib-33060`；exec 代码里仍**不要给 `idb` 这个名字赋值**。
> 本轮反编译原文暂存 `%TEMP%\core3d-recheck-0904\*.txt|.c`（临时目录，不入库；要留证据时挪进 `evidence/`）。

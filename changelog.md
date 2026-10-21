# Changelog

## 2026-10-22 db_discovery 按主项目 SYS 库的外项目表补外项目库：不给 ZDJ000 也能解析 MDB `/ALL` 挂的 db 7600（happyrust/e3d-model#5）

- `DirectoryDbResolver::adopt_foreign_dbs`：读主项目目录里的 `<代号>sys`（`ams000/amssys`），取 `PROJ` 非空的 `DB`
  元素（`PROJ` 存代号的字符码，`RawWords([90, 68, 74])` = ZDJ；`DBNO` 存成 Word），给定目录解析不出的库号到
  外项目目录里取同号正身。外项目目录按 E3D 的约定找：环境变量 `<代号>000` 优先，否则主项目同根下一层
  唯一一个 `<代号>000`（`custom_evars.bat` 调的 `evars<代号>.bat` 设的就是它）；找不到或不唯一只记说明。
- 只补登记的库号，不把整个外项目目录挂进来：ZDJ000 自己也有 7009 / 7011 / 7015（与 ACP 同号），整目录挂进来会
  把 ACP 的正身挤成「分不出」——S1 的整库生成给了 ZDJ000 + acp000，这三个库一直被扣着，现在按 SYS 库登记
  从 ACP 补回。给定目录已有、但 SYS 库登记在别的项目的（ams000 的 `ams7355_0001`，登记为 APS 的
  `*MDU/CATA`）照旧用给定目录，只记一条说明。
- `gen_ams`：给了 `--project-dir` 时按 `--db` 所在项目补外项目库并逐条打印，`--no-foreign-projects` 关掉。
- 验收：只给 ams000 + acp000，ams7999 `24383/66515` / `66585` 两只 ATTA 由 failed 变为生成，RVM 与
  `elements.json` 与显式给 ZDJ000 时逐字节相同；ams7997 `/1PTU-INST23` 6 个 STRU 20/20 exact。土建、`/SSC测试`、
  MDS、S1 逐件不变（SCB 6000–6003 随之补进）。
- 真库门 `saddle_srto_real` 新增 `the_zdj_catalogue_is_adopted_from_the_ams_system_db`；单测
  `proj_words_spell_the_foreign_project_code`、`a_foreign_project_dir_is_found_beside_the_main_project_or_refused`。

## 2026-10-22 截面成员按出身与元素标架写记录：单 SREC 截面的 STWALL 写 E3D 那条 `Pyramid`

- `elmodl`：SCTN / STWALL 截面只有一个成员时也把目录出身交给写出端（此前要 ≥ 2 个成员才交），每个成员
  另带一只元素局部系的标架（单位阵）：成员建在元素局部系里、沿局部 Z 挤出，DRPYR 把 SREC 的高写在挤出
  方向、底边写在截面 X / Y 上。ams7997 `/1SL-WF08-W-C-S8038`、`/1SL-WF08-W-C-S8019` 的 STWA 1 截面就是
  一只 `SREC`：E3D 写 `Pyramid[150, 700, 150, 700, 0, 0, 1835]` / `[200, 200, 200, 200, 0, 0, 1200]`，记录矩阵与
  元素标架逐列相同；此前我们写拉伸体面片。只交出身不给标架时，识别器对盒子随便挑轴
  （`[150, 1835, 150, 1835, 0, 0, 700]`）。
- 语料里没有带 SREC 成员的 SCTN（S1 基准、MDS、`/SSC测试` 的 `Pyramid` 记录逐条普查过），SCTN 走的是
  同一条截面路径；标架对不上识别出来的轴时 `pyramid_geometry_in_frame` 照旧退回网格轴。
- 对拍（连同上一条）：ams7997 `/SSC测试` 整站记录种类不同 4 → 0，4522 exact、1 film 不变；`Pyramid` 记录
  逐条普查 35 只里 33 只与 E3D 逐位相同，剩风管 OFST 1 / TAPER 1 两只偏心棱台换了一种等价写法（几何一致，
  另案）。土建 9982 件、MDS BS 80 份 ZONE、S1 基准、FIXING / datum 真值逐件不变；两只 STWALL 的轴差
  0.0023 / 0.0008 → 0。
- 真库门 `rvm_record_kinds_real` 新增 `single_srec_walls_and_flat_duct_plates_write_e3d_pyramids`：两只 STWALL 与
  BEND 1 / THRE 1 的记录种类顺序、`Pyramid` 七个参数照 E3D 原样。

## 2026-10-22 高为 0 的目录棱台照写一条 `Pyramid`：风管 THRE / BEND 的无体板对上 E3D

- `catalogue`：高为 0、底顶仍有边长的正体 LPYR，Core 不出体，E3D 的 DRPYR 照写一条 `Pyramid`，高是 `-0`
  （ams7997 `/-VES-E-4-L-2501` 风管 BEND 1 带 2 块、THRE 1 带 4 块 `Pyramid[500, 500, 500, 500, 0, ±0, -0]`）。
  `flat_lpyr_geometry` 与正常棱台同一套标架与轴向翻转，经新的 `DegenerateCatalogueShape::Pyramid` 交给
  `rvm_write::bodyless_pyramid_geometry` 写记录；七个参数与 E3D 逐位相同（含 `-0`）。
- 这两只没有负体：本体那条 FacetGroup 两侧本来一致，差的就是这几块无体的板。「E3D 只把被切的图元合成
  面片」那条假设用不上，布尔结果的写法不动。
- 单测 `a_flat_lpyr_has_no_body_but_keeps_its_record`。

## 2026-10-22 rvm_read 认 E3D 给斜切大小头写的倒着的声明盒：e3d-model#6 的三个暖通区不再拒判

- `rvm_read::detect_bbox_frame`：斜切大到 z 上界算到下界以下时，E3D 照公式写 min > max 的记录盒
  （`/-CAM-E-2-H-5302` BEND 2 / 3，XTSH 或 YTSH −7.5°，z ∈ [−34.56, −438.99]）。角点 AABB 会把它摆正、对不上，
  声明盒再拿原样的六个数在局部帧里对一次。
- happyrust/e3d-model#6 的三个区（plant-model-6 `outputs/t43-*`，`LEVEL HVAC 6` 的 TTY 导出）此前整份拒判，
  现在正式对拍：`/-KX-CAM-E01` 96 对、`/-KX-CAM-S01` 34 对、`/-RX-CAV-E01` 82 对，全部 exact；只在 E3D 的
  一条是 `/-CAM-E-2-H-5302` 那根 `PDIA` 求值失败的零半径 TUBE。
- 单测 `an_inverted_declared_box_still_frames_the_record`。

## 2026-10-21 RVM 记录种类跟 E3D 走：扣过负体的件写布尔结果面片（`export holes on`），MDS 关节不再丢孔

- `gen_ams` 新增 `E3D_EXPORT_HOLES=on|off`（缺省 on，即 TTY 取证宏的 `export holes on`）。打洞导出里扣过负体的件是
  一条布尔结果 FacetGroup：不按图元拆、不认形状、不写整元素解析图元。MDS 关节 `IJOINT1`（JOIN
  `/HCS-VT41-68/2`，两只 SBOX 各带一只图元级 NSCY）原先按桥架口径写成两条未扣减的 Box，孔在 RVM 里丢了；
  FT48B 被 NBOX 切过的设计 BOX 原先按长相认成 `Pyramid`。
- 桥架那条「可见图元各写一条未扣减的」口径是 `export holes off` 的：同一支 C-IY-1R330-B 在
  `export holes on` 的 TTY 重导里 FTUBE 是一条 FacetGroup，在 addin 导出里是三条 Box。对拍那份 addin 基线时设
  `E3D_EXPORT_HOLES=off`，结果不变（35 条 exact、种类一致）。
- 拉伸体族没有目录出身时一只元素写一条面片，不再按连通分量拆（MDS `FT08/4-BAR-1` 等 15 根杆、
  ams1112 `/1RS-WF04-F-C-F002` 的 3 块楼板、`/1RX-RM03-R312-VOLU` PANE 1）。
- `rvm_write::pyramid_geometry_in_frame`：目录标架的 X 落在识别出来的 y 上（绕轴转 90°）时底 / 顶边长对调；
  斜着落时退回网格轴。此前偏心照标架、边长照网格，自校闸不过，风管 TRNS / TAPER / BRCO 的偏心棱台
  退成了面片。
- 对拍：MDS BS 80 份 ZONE 记录种类不同 58 → 0；ams7997 `/SSC测试` 整站 9 → 4；土建 6 → 2；几何判档
  全部不变。剩下的是两只 STWALL（E3D 写 Pyramid）与 `/-VES-E-4-L-2501` 的 THRE / BEND（E3D 只把被切的
  图元合成面片，其余写 Pyramid）。
- 真库门 `rvm_record_kinds_real`：IJOINT1 在 holes on 下是一条亏格 2 的 FacetGroup，holes off 下是两条 Box。
  单测 `pyramid_in_frame_swaps_the_cap_sides_when_the_frame_turns_a_quarter`。

## 2026-10-21 rvm_compare 新增 `film` 判档：连在主体上的零厚残膜 / 发丝不再算 off

- `reference_artifact` 只认断开的薄片；负体切剩的膜与发丝连在主体上，按包围盒差一律 off。新判法：对每一侧，
  取落在对侧局部 AABB 加弦高之外的面片顶点，连同面片边穿过那只盒面的交点（只拿盒外顶点会把「真的
  长出一截」的端面量成 0 厚），量它在三个主方向上的最小跨度；每一侧多出来的部分都 ≤ 0.05 mm、且去掉
  以后两侧 AABB 在弦高内一致，记 `film`，膜厚与所在侧写进 `pairs[]`。
- 只走 RVM ↔ RVM、局部帧那条路；只为 off 的那几件把两侧面片轮廓再读一遍，不多留点云。
- 土建 3 件 off 全部转为 film：ams1112 `/1RS-WF03-W-C-RR001` GWALL 13（E3D 侧，膜厚 0.010 mm）、ams7997
  `/1LR-WF05-S-C` FLOOR 6（E3D 侧，0.010 mm）、`/1RX-FN01-F001` FLOOR 12（生成侧发丝，0.00005 mm）；去掉膜以后
  主体差 ≤ 0.0014 mm。其余土建 9979 件、S1 基准（`/MDS/FT01/1-BAR-1` 那根同名杆仍是 off）、MDS、
  `/SSC测试` 整站判档不变。
- 单测 `a_thin_fin_on_top_is_a_film_but_a_longer_block_is_not`、
  `a_zero_thickness_hair_on_the_generated_side_is_a_film`、`principal_span_finds_the_thickness_of_a_tilted_sheet`。

## 2026-10-21 rvm_read 认 E3D 给偏心棱台、偏心 / 斜切 Snout 写的声明盒：FT48B 与 `/SSC测试` 整站不再拒判

- `rvm_read::Primitive::declared_box_mm` 给偏心 `Pyramid`（XOFF / YOFF 非零）补上与 Snout 同形的声明盒：每轴半宽
  `max(底/2 − off/2, 顶/2 + off/2)`、z = ±h/2。MDS `/SUPPORT/FRAMES/BS/FT48B/6-SUBS1` 的四只楔块记录盒
  y ∈ ±12.75，真几何 y ∈ [−38.25, 12.75]（TTY `Q PPOS 1 / 2` 实取两端面心）。生成器与
  `local_hull_points_mm` 本来就对，只是判帧对不上声明盒，整份 RVM 被 `rvm_compare` 拒读。
- 同一次普查补齐 Snout 声明盒的两处：`hx > hy` 时 y 上界 E3D 写的是 `hx`（ams7997 风管 TRNS 的圆锥段、
  zone-66457 的偏心阀体）；X 向斜切与 Y 向一样按「度当弧度」计入 z 两端（hvac-5201 的风管斜切段；
  XBSH 没有样本，按对称写）。
- 普查盘上 350 份 E3D 3.1 导出物：402 条 `Pyramid` 里 140 条偏心、243 条偏心 / 斜切 `Snout` 逐条命中
  （残差 ≤ 0.01 mm）；判帧 `unknown` 的记录从 226 条（12 份文件）降到 0。
- 对拍（缺省表现级生成，对 `LEVEL 6` / `LEVEL HVAC 6` 的 TTY 导出）：MDS BS 80 份 ZONE 全部读进，FT48B 36 对
  exact（其中 20 对记录种类不同，另案），合计 1477 对：1473 exact、4 chord、0 off；`/SUPPORT/FRAMES/BS/DATA/FT`
  整份同样结果。ams7997 `/SSC测试` 整站（土建 + 暖通）第一次能整站对拍：4523 对，4522 exact，1 off 是已知的
  FLOOR 6 残膜。S1 基准与土建逐件不变。
- `examples/rvm_facets_dump`：记录盒对上声明盒时改拿真外形挑三角化候选，偏心棱台不再报「口径不合」。
- 单测 `eccentric_pyramid_declares_the_snout_style_box`、`snout_declared_box_tops_y_with_hx_and_counts_x_shear`。

## 2026-10-21 只有负体集的目录件也算链终点：MDS 关节 JOIN 截杆（ams7324 E3D TTY 实证）

- `catalogue_point::follow_catalogue_component`：目录链上的节点有 GMRE 或 NGMR 就是组件终点（原先只认
  SCOM 与 GMRE）。MDS 内部关节的 `JOIN`（`/E3D/Joint-for-FT55/JOIN`）GMRE 未设、NGMR 一把 NSCY（Ø `DESP[1]` ×
  `DESP[2]`），把它接的杆从端面截掉 100 mm；链在这里断掉时宿主 FIXING 扫描拿不到这把刀。
- MDS BS 支架 80 个 ZONE 逐个用 E3D TTY 导 RVM 对拍（1441 对）：FT54A–D、FT55A–D 的 36 根杆原先长 100 mm，
  现在 exact；按导出会话的表现级（`LEVEL 0`）生成时 1437 exact、4 chord（FT44 圆管弦高）、0 off；FT48B 那份
  RVM 因 4 只 PYRAMID 记录 bbox 帧认不出未对。FIXING 真值、土建与 S1 基准逐件不变。
- `gensec_end_datum_real` 新增 `joint_catalogue_negatives_trim_mds_bars`。

## 2026-10-21 SBFI 标架合成一条：SCTN 上的 CMFI 名下也按 ZDIR / BANG（aps7200 E3D TTY 实证）

- `transform::ImplicitFrame::SubFitting`：母件 CMFI 在 SCTN 上的 SBFI 不再走「安装线水平投影 + 世界竖直」，
  与 STWALL 上的同用 `sub_fitting_rotation`（Z = 母件旋转 × `ZDIR`、Y = 母件 Y 去掉沿 Z 的分量、再绕 Z 转
  `BANG`）。
- 真值取自 APS 项目副本（10-19 已把 aps7200 挂进 `/ALL`，`run_copy_tty.ps1` 只读宏）：aps7200
  `23584/8625`、`23584/8629`（楼梯底板）的 `POS/ORI WRT WORLD`，母件 `23584/8624`、`23584/8628` 也一并取了。
  E3D 的 Z 带 2.05e-6 的倾角，正是 `ZDIR` 存值经母件旋转后的余量；旧规则差 2.05e-6，现在 2e-10。
- 语料里 CMFI 在 SCTN 上的 SBFI 只有这两只（APS 6 只 SBFI、AMS 156 只普查）；土建对拍逐件不变。
- 单测 `sub_fitting_rotation_matches_tty_on_sctn`（到 1e-8）。

## 2026-10-21 零长度 SPINE 上的 PLDATU 照 GPLDAT 出标架（ams7330 E3D TTY 实证）

- `datum::solve`：GENSEC 的 SPINE 长度为 0 时（两只 POINSP 重合），E3D 求不出站位——TTY 对 JLDATU 报
  `pos;xdir;ydir;zdir` 求值失败——PLDATU 却照样有标架：站位在 GENSEC 原点，截面标架取单位阵（切向
  +Z、Ydir +Y），不转 `BANG` 与中性轴角，其余照 GPLDAT。JLDATU / ENDATU 照旧不给标架。
- ams7330 `/SUPPORT/SPECIALS/BS/PIPE/3-BAR-2`、`4-BAR-2` 的两只 PowerClick 接头 FIXING 原先退成单位阵，
  包围盒差 22.4 mm；现在与 TTY 逐件对上。FIXING 真值对拍出体的 910 只全部一致；GENSEC datum 真值
  3678 个对上 3588 个，剩下 90 个是 MDS FT44 斜撑上 `POSL=VV` 的 PLDATU（圆管「圆周 315°」p-line，
  目录里 PX / PY 未填），另案。
- `gensec_end_datum_real` 新增 `stationless_pline_datums_sit_where_e3d_puts_them`（两只 PLDATU + 两只 FIXING）。

## 2026-10-21 墙上 PLDATU 按 POSL p-line 落标架：IBOW 门窗对上 E3D TTY（ams7997 / aps250110 实证）

- `elmodl::wall_pline_datum`：PLDATU 在 JLDATU 站位系里的标架与 GENSEC 的 PLDATU 同一套 GPLDAT——原点是墙截面
  里 `POSL` 那条 p-line 减 `JUSL` 线（`LMIRR` 时 x 取负），Z 是那条线的 PLAX，Y 是 `YDIR` 去掉 Z 分量，X = Y × Z；
  p-line 由新的 `datum::section_pline` 从 SPRE 目录截面求。`OBOW` 在 (0, 0)、PLAX −X，与原来写死的公式逐位
  相同；`IBOW` 在 (墙厚, 0)、PLAX +X。
- 删掉按 aps250110 RVM 拟合的 IBOW 规则（PLDATU 抬高 `(DESP[0]−600).max(3400)` + 清 FIXING `POS[1]/POS[2]`）：
  E3D TTY 里 IBOW 的 FIXING 同样按全 POS 从 PLDATU 量。ams7997 `/Copy-of-R211-M01`（`24381/2067`）原先放偏
  1.9 m、X 轴反向；它和 aps250110 的 6 只门窗现在对 TTY `POS/ORI WRT WORLD` 逐件 ≤ 0.00004 mm / 4.3e-10。
- R211 的两块门扇另靠 e3d-io 3bed2ba（FIXING 上 `PTYP DIST` 的 `ATTRIB RPRO` 转角求值失败、方向回 +Y）：P8 / P12
  的 PTCD 回到 FIXING +Y，门扇贴回墙面。土建对拍 9982 件里只有 R211 一件变化（off 1496 mm → exact），
  `/电气测试模型` 整区 663/663 exact。
- `wall_fixing_datum_real` 新增两条 IBOW 真库门：R211（标架 + RVM 世界包围盒）与 aps250110 的 6 只门窗。

## 2026-10-21 rvm_compare：基准侧断开薄片按凸包宽度判窄

- `rvm_compare`：reference_artifact 判档里「窄于一个弦高的断开薄片」，基准侧改按分量在局部 XY 的凸包
  最小宽度量（生成侧只有 AABB，仍按短边）。斜放的细条 AABB 并不窄：ams7997 `/1RX-FN09-F001` FLOOR 2
  的 E3D 面片里有一块只有两个不同 XY 点的零厚片，AABB 584 × 103 mm，宽度是 0。主体与生成侧差
  0.002 mm，判档 off → reference_artifact；其余各站不变。

## 2026-10-21 重接的倒圆弧按理论切点铺相位点（ams7997 E3D TTY 实证）

- `discretise::flatten_ring`：环里有 Core3D 跳过的退化线段、倒圆两端都越界时，弧的两个端点重接到相邻
  PAVE，相位点仍铺在理论切点之间的整段弧上（只换端点，不换角度范围）。原先按两个 PAVE 的角度取
  格点，切点远在 90 km 外的 `FRAD 31441`（ams7997 NXTR `24381/42463`）只剩一个格点。
- ams7997 `/1RX-FN09-F001` FLOOR 4：两把负体交界处的 10×4×70 mm 碎片消失，与 E3D TTY RVM 一致
  （0.004 mm）；其余各站逐件不变。

## 2026-10-21 圆弧墙截面走 SPRE 目录截面 + JUSL（ams7997 E3D TTY 实证）

- `elmodl::wall_section`：圆弧墙的截面与 SCTN / GENSEC 同一条截面链——SPRE 目录截面减去 `JUSL` 那条
  p-line，`LMIRR` 时沿 x 镜像；`circular_wall_radii` 按截面 x 范围（朝行进方向左手侧）给内外径，斜接
  端标架就是截面系，替掉上一条的「扫掠符号 × LMIRR」口径。语料 62 堵圆弧墙的 SPRE 都是 DESP
  参数化的矩形：`NA` / `OBOW` 在 x = 0，`IBOW` 在 x = 厚。
- ams7997 `/1SL-WF01-W` 的两堵逆时针 `IBOW` 墙（`24381/31100`、`24381/31106`）原先放在 spine 内侧，
  差 736 / 752 mm，现在在外侧 [28100, 28900]；ams1112、ams7997 其余各站逐件不变。
- 对拍范围扩到 `/SSC测试` 里混着 HVAC 的三个区：只导土建 STRU，每个 STRU 一份 RVM（12 份、
  3679 件），exact 3678，余 `/1LR-WF05-S-C` FLOOR 6 一件另案。

## 2026-10-21 圆弧墙 LMIRR 翻侧 + 扫掠段数取截面外径（ams7997 E3D TTY 实证）

- `elmodl::circular_wall_radii` / `wall_end_frame` 改收 `side` = 扫掠角符号 ×（`LMIRR` 时 −1）：镜像的墙落在
  行进方向的右手侧。ams7997 `24381/1597`（顺时针、`LMIRR`）回到 spine 内侧 `[r−厚, r]`，原先差一个
  墙厚 600 mm；斜接端标架仍是右手系，体侧符号跟着 `LMIRR` 翻。
- 扫掠段数按截面外径算（原先按 spine 半径）：落在外侧的 `24381/1932`、`24381/4302` 原先差 4.1 / 0.77 mm。
- ams7997 `/电气测试模型`（`24381/777`）整区 RVM 对拍：WALL 18/21 → 21/21，整区 662/663（剩下的
  `/Copy-of-R211-M01` 另案）；`/1RB-CIVI`、`/1RC-CIVI`、`/1RX-ARCHHB`、ams1112 逐件不变。

## 2026-10-21 挤出截面改用 Positive 填充（ams7997 E3D TTY 实证）

- `solid::extrude_rings`：`FillRule::NonZero` → `FillRule::Positive`。单端越界的倒圆倒着发出去的那段线与弧
  围出的反向耳朵、几乎共线的折返尖刺、伸到外环外的孔环，绕数都是 −1，E3D 不算实心。
- ams7997 `/1RX-ARCHHB`：PANE 311 exact + 13 off → 325/325 exact；ams1112 与 ams7997 另两站不变。

## 2026-10-21 负扫掠圆弧墙落在 spine 外侧（ams7997 E3D TTY 实证）

- `elmodl::circular_wall_radii` / `wall_end_frame`：墙截面落在行进方向的左手侧——逆时针扫掠在
  spine 内侧 `[r−厚, r]`（原口径），顺时针在外侧 `[r, r+厚]`；斜接端标架跟着翻，仍是右手系。
- ams7997 `/1RB-CIVI`、`/1RC-CIVI` 整站 RVM 对拍：WALL 1/13 → 13/13、0/12 → 12/12（原先各差一个墙厚
  1500 / 1300 mm），FLOOR / GWALL 本来就对；ams1112 的 14 堵正扫掠墙不变。

## 2026-10-21 SBFI：DESP 套管两头接长 + STWALL 上 CMFI 名下的标架（E3D TTY 实证）；ams1112 全库收口

- `elmodl::sbfitting_axial_span`：DESP 套管占 `Z−LENGTH−DESP[5] .. Z+DESP[4]`（目录件 SFIT `/GW-HOLE` 的
  两只 LCYL 就是这么写的），开孔刀同步。`/1RS03CC0028T`（`DESP[4]=50`）对上。
- `transform::ImplicitFrame::SubFitting`：STWALL 上 CMFI 名下的 SBFI，Z = 母件旋转 × `ZDIR`（SBFI 的 Z
  在母件系里的分量，原样）、Y = 母件 Y 去掉沿 Z 的分量、X = Y × Z，再绕 Z 转 `BANG`
  （`sub_fitting_rotation`）。ams1112 两只（`BANG` 90 / 180）与 TTY 的 `ORI WRT WORLD` 逐轴对上
  （≤ 4e-10）。SCTN 上的照旧走平放规则（aps7200 两只不变），两条要合并得先拿到 APS 的 TTY 真值。
- `rvm_compare`：RVM ↔ RVM 生成侧补连通分量账，reference_artifact 两侧同一口径。
- ams1112 全库 RVM 对拍（5135 件）：exact 5132、reference_artifact 2（FLOOR 9 / 12：E3D 面片里窄于弦高的
  断开残片）、off 1（GWALL 13：E3D 在两把相邻 NXTR 之间留了一条 0.01 mm 宽、26 mm 高的连体薄膜，
  2026-09-20 起我们的亚容差薄膜清理会把它清掉）。

## 2026-10-21 PFIT 板上配件成件：目录几何贴属主板 SJUS 那一面（E3D TTY 实证）

- `category`：`PFIT` 进模型单元，走目录构件 GMRE 求值（与 `FIXING` / `FITT` 同一条路），不挑属主。
- `catalogue::panel_fitting_world`：实例标架 = 属主板标架 × `T(POS.x, POS.y, z板面)` × `R(ORI)`。
  E3D TTY 里 PFIT 自己的 `POS/ORI WRT WORLD` 含 `POS.z`，RVM 里的目录几何却落在板面上；
  `z板面` 由 `panel_surface_z` 按 PFIT 的 `SJUS` 取板的顶 / 中 / 底面（板 `[0, HEIG]` 挤出、
  再按第一个环的 `SJUS` 下移）。FLOOR 上的 PFIT 全是 UTOP：ams1112 `/16` 那块 DBOT 板顶面在
  z = 1340，其余 UTOP 板在 z = 0；ams7999 PANE 上的 194 只 SJUS 未设，落在板底面（板厚 16–30 mm）。
  CEN 族还没有语料。
- `profile::loop_extent`：只读第一个环（与板自己）的 `HEIG` / `SJUS`，不读顶点；`loop_profile` 共用它。
- `increment::implicit_frame_host_of` 加板支：FLOOR / PANE / GWALL 名下有 PFIT 时，板自己或它的
  PLOO / LOOP 改了，整棵板子树级联重建（`panel_loops_climb_to_the_panel_carrying_pfits`）。
- 实证（gen-model `verification/civil-tty-20261021/`）：ams1112 全库 RVM 对拍 PFIT 199/199 exact；
  全项目 456 只 PFIT 里出体 434 只（FLOOR 240 + PANE 194，PANE 需把 ZDJ000 加进 `--project-dir`
  才开得到 db 7600），元素标架对 TTY ≤ 5e-5 mm、几何对 RVM AABB ≤ 0.62 mm；17 只无形状两边都空。

## 2026-10-19 SBFI 平放标架：嵌套配件的宿主刀与正体归位（7200 原生 RVM 实证）

- `transform::ImplicitFrame::SubFitting`：`SBFI`（属主 `CMFI`、再上 `SCTN`/`STWALL`）
  的世界标架改为——原点沿用母配件标架原点，旋转取安装线的**水平投影**（y'=
  normalize(y−Ẑ·(y·Ẑ))、z'=Ẑ、x'=y'×z'）。原生 RVM 里 `SBFITTING` 与 `FITTING`
  同原点但平放（aps7200 `23584/8625`、`23584/8629` 逐列对上）；安装线竖直时退回
  母件标架。
- 影响两处：SBFI 的 mode-6 NGMR 宿主刀（NGMS 的 NSBO 被 NAPP=-1 放行，原先落到
  母件斜标架的位置、切不中目标）现在落到正确位置——8622/8626 宿主的下角残差
  （z 867..962、y 9953..10000）由此全部消失，宿主双向采样差 0.0014 mm。
- `elmodl`：无 `DESP` 但有 `SPRE` 的 `SBFI` 从 skip 改走
  `model_catalogue_member_with_cache`（与 FITT 同一条目录求值路），原生
  `SBFITTING` 组（带孔平板）对上（双向 0.0004 mm）。7200 全库 4 件
  （23584/6548、8625、8629、10004）。DESP 套管路径不变。
- P5 顺带拿到原生实证：8622/8626 是 `JUSL=FOC` 样本，回退 NA 后宿主与原生
  逐点对上。

## 2026-10-18 P5：JUSL 缺失对齐线回退——按 GTNAID 证据回退中性轴

- 反编译钉死（`idapro` idalib 直开 `D:\ida_scratch\plant3\Core3D.dll.i64`，绕过坏掉的
  ida-bridge venv）：SCTN 对齐线解析 `pllib/GJLORI@0x10399B54 → pllib/GTJLID@0x103AC744`，
  `GTPLID@0x103A89BC` 在 PSTR 成员里 `DFINDI` JUSL 的 p-key 编码；**落空调
  `pllib/GTNAID@0x103AC154` 取中性轴线，不是回原点**；连中性轴（PKEY 编码
  531482/726125/726137）都没有才 NULIFY → 调用方得零偏移。
- `section::justification_offset` 加第二层：`PKEY==jusl` 找不到时回退 `PKEY==NA` 的
  PLIN（`member_frame` 的 `(PX,PY)`），PSTR 也没有 NA 才按原点并记 note。
  `SectionProfile.justification_line` 改报实际解析到的线名（回退后 `NA`，全失空串）。
- 语料验证：`census_sctn_joint_classes_and_missing_jusl` 重跑，66/67 回退命中 NA
  （aps7200×25、7324×16、7329×5、7350×18、7356×2），唯一例外 aps7329 `23713/4134`
  （`JUSL=NA` 且 PSTR 空）按原点——与 GTNAID 的 NULIFY 支一致。
- 几何影响实测：7200 的 25 根 `JUSL=FOC` 里 23 根 NA 在 `[10,0]`，截面沿局部 x 平移
  10 mm（体积不变）；另 2 根 NA 在 (0,0) 几何不动但 note 记回退。

## 2026-10-18 gen-model #34 D 线：SCTN 关节负体记账分型

- `cbfffb2`（已提交）：`fixing_negatives` 改记三项账（有无几何源 / 交出刀数 / NAPP 挡下刀数），
  `negatives_skipped` 只收真正没有几何源的条目；「有源但源内无负体」改记 note。
- `Report` / `GeneratedElement` 新增 `joint_cuts_explained`：JOIS/JOIE 指向**另一根** SCTN
  名下 SNOD 里 `CREF` 回指宿主的 SJOI（J-A 拓扑）且没有几何源时，按 §12① 的原生证据
  （250110 /68 双向最大差 0.007071 mm、/70 RECONNECT END 幂等）登记为「已解释：端面已
  覆盖」，不再记 skipped；caveat 一并写进 detail。
- 无源关节 / 成员按拓扑打类标签：`[J-A]`（已解释，§12①）、`[J-B]`（已解释，
  sctn_250110_jb-20261018 两样本 /78 /84：宿主正体双向 0.0063 mm、SJOINT 零图元）、
  `[J-C]`（PJOI）、`[M]`（成员 FITT/CMFI/SBFI）、`[未分型]`；J-C/M 留在
  `negatives_skipped` 待原生证据。
- `Report` 新增 centreline-only 逐件明细：每个只画中心线的构件记 refno、noun 与原因
  （无 SPRE / 目录截面成员全被显示筛或空占位），聚合 noun 计数保留。
- 测试：语料守卫 `no_snod_sjoi_in_the_corpus_carries_a_geometry_source`（一旦 SNOD→SJOI
  真交出负体就红——P7 的 `ImplicitFrame::SectionNode` 暂定标架须先处理）；`#[ignore]`d
  普查 `census_sctn_joint_classes_and_missing_jusl` 出 `classes.json`（J-C 242、J-D 65、
  JUSL 缺失 67 根、缺失线名真不在 PSTR PKEY 里）。

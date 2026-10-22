# Changelog

## 2026-10-22 详图式扶手成员按一条规则站位、转向——ZDIS 从圆角切点量，端头与立柱同按 HRJUSL

- 立柱 `HRPOST`、端头 `HRTERM`、面板 `HRPANE`、门 `HRGATE` 没有 POS / ORI，标架现算（`transform::handrail_post_world`）。
  站位改由 `handrail::member_station` 给：从 `POSREF` 点出发、沿离开它的那条边走 `ZDIS`，起量点是这条边直段的起点
  ——拐点上 `FRAD` 圆角的切长（`FRAD·tan(θ/2)`，与栏杆的段表同一份 `path_layout`）先让掉；闭合路径末点那条边
  绕回首点；开口路径末点沿进入段往回量；边上夹着 `CURTYP THRU` 的 `CURVE` 时站位在过三点的圆弧上；越过终点的
  夹在终点。
- 朝向一条规则：局部 +X 顺路径（`HRJUSL LEFT`）、逆路径（`RIGH` 与其余）或取 `路径 × Z`（`FORW`），端头也一样。
  此前末点的路径方向取「倒数第二条边反向」，再靠 36004 / `LEFT` 两条特例补，aps250112 两只端头转反 180°。`XOFF`
  偏在路径右侧（`路径 × Z`）、`ZOFF` 向上，立柱 / 端头 / 面板同一口径。门照旧不转。
- aps250112 `/EL(+)100000/HR_001`、`HR_002`：5 根挂在 `FRAD 100` 直角拐点上的立柱（差 100 mm）与 2 只端头，模板件
  共 102 件 off 归零；aps250110 3425 件 exact 不变。新真库门 `aps_handrail_real`。
- 祖先读护栏：`handrail_post_world` 不再读 `POS` / `SPRE`，消费者换成 `handrail::member_station`（`CLOSEF`）
  与 `through_arc`（`CURTYP` / `POS`）。读取集合不变。

## 2026-10-22 折弯板 BPANEL 沿 RPATH 中线建体，没开孔的逐段一条 FacetGroup——aps250110 52 只全出、exact

- aps250110 的 52 只折弯板（`/AREA02_WS_CLADDING*` 墙板 40 只、`/AREA02_WS_ROOFING` 屋面板 12 只）此前整族
  落在 `Composite(Structural)` 待建账上。截面中线是名下 `RPATH` 的 `POINTR` 点串，拐角按 `FRAD` 倒圆；板厚
  `WTHICK` 以中线对称（`WJUSL = CENT`，语料里全是），沿 `RPATH.YDIR` 从 0 拉到板宽 `WIDT`。
- 新模块 `bpanel`：段表与扶手同一套（`handrail::path_layout`）；直段是矩形板（`FRAD = 0` 的拐角先加长再按
  角平分面切），拐角是一节矩形截面圆环。`BPANEL` 与 `HANDRA` 同列为 `Composite(Structural)` 里的模型单元，
  开孔负体照旧由 `model_element` 扣。
- E3D TTY RVM 有两种写法，弯的分段也不同：名下没有负体时逐段一条 FacetGroup（N 个点 = 2N − 3 条，弯按曲面
  图元口径分段、至少 2 段）；有开孔时扣完整只一条，弯按布尔操作数的粗口径（10 mm 弦高的整圈段数按扫角缩、
  至少 1 段）——屋面板 9 / 10 / 11 的 R25 弯只剩一条弦，板的最低点就在弦上，按细口径分会差 0.76 mm。
  `model_element` 没扣负体时把各段交给写出端（出身名词留空，只为逐块一条记录）；`gen_ams` 把 BPANEL 列进
  只写面片的名单（直板段按长相会被认成 `Box`）。
- 读取集合不变（157：`WIDT` / `WTHICK` / `WJUSL` / `FRAD` / `CLOSEF` / `YDIR` 早已在内）。
- aps250110：两侧都有 3375 → 3427，exact 3373 → 3425，只在 E3D 54 → 2。真库门 `aps250110_structures_real`
  加六只板（有孔 / 无孔，墙板、屋面板、收边）的世界包围盒与逐段记录条数。

## 2026-10-22 梯段输入改了级联整棵梯段子树：楼梯踏步的尺寸随梯段现算，增量要跟上

- `f21e7a1` 让 `stair::owning_flight` 沿 owner 链上溯到梯段，读它的 `TWIDTH` / `TREADO` / `OGOING` / `RISE` 与
  `RPATH` 首末点，却没登记进祖先读护栏（`ancestor_reads_guard` 红；当时只跑了相关的几组测试）。踏步自成
  单元、梯段在它之上：单元级规划只重建 `nearest_unit` 上卷到的单元，梯段改了，踏步会停在旧尺寸。
- `increment::implicit_frame_host_of` 加 STRFLT 支：名下（三层内）有 `TREAD` 的梯段，它自己、它的 RPATH /
  POINTR 改了就把整棵梯段子树级联进计划，与详图式扶手、挂 PFIT 的板同型。真库单测
  `stair_flight_inputs_climb_to_the_flight_carrying_treads`（aps250110 `/B01_LEVEL_03_ACCESS/SF_001`）。
- 护栏登记：`stair.rs::owning_flight`（只核 noun），消费者 `tread_size`（`TWIDTH` / `TREADO`）、`going`
  （`OGOING` / `POS` / `RISE`）；`rpath::first_rpath` 的理由补上梯段。生成结果不变。

## 2026-10-22 楼梯踏步 TREAD 走目录构件求值，尺寸取随梯段现算的 TWIDTH / TREADD——aps250110 25 只全出、exact

- aps250110 两段梯 `/B01_LEVEL_03_ACCESS/SF_001`、`SF_002` 的 25 只踏步此前整族落在 `Composite(Structural)`
  待建账上。链是 `SPRE → GPART /SLH_TREADS/Simple_Catalogued_Tread --CATR--> SFIT`，GMRE 一只 SBOX：
  `PXLE / PYLE / PZLE = RPRO TWID / TDEP / THK`，中心 `(0, TDEP / 2, −THK / 2)`；DTSE 把 `TWID` 接到
  `ATTRIB TWIDTH`、`TDEP` 接到 `ATTRIB TREADD`、`THK` 接到 `PARA[3]`（20）。
- `TREAD` 归 `Catalog`、成件，走与 FIXING / FITT 同一个目录构件求值器。踏步自己的 `TWIDTH` 存 0、`TREADD`
  不存：新模块 `stair` 按属主梯段现算——`TWIDTH` 取梯段的（800），`TREADD` = going + `TREADO`，going 是
  `OGOING` 或梯段 `RPATH` 首末水平距离 ÷ 级数（竖向距离 ÷ `RISE` 取整），经 `member_env` 的伪属性交进求值
  环境。算出 269.659 / 297.919，E3D 的盒深 269.654 / 297.924。
- 读取集合 152 → 157（`TWIDTH` / `TREADD` / `TREADO` / `OGOING` / `RISE`，都不是 DCHC=0）。
- aps250110：两侧都有 3350 → 3375，exact 3348 → 3373，只在 E3D 79 → 54。真库门
  `aps250110_structures_real` 加两段梯各一只踏步的世界包围盒与伪属性。

## 2026-10-22 单 DESP 的圆弧墙按 SPRE 目录截面求值：aps250110 两堵外墙出件

- `CWALL /B01_LEV01_EXT_WALLS`、`/B01_LEV02_EXT_WALLS` 的 WALL 用 `/100_thk_wall`：目录截面是
  `[0, CPAR[1]] × [0, DESP[1]]`，厚 100 在目录 `PARA`，DESP 只有高一项（4000 / 3800）。`wall_local_solid`
  先前要求 DESP 至少两项，整件报「DESP length 1 < 2」。
- 截面本来就从目录求（与 SCTN / GENSEC 同一条截面链），DESP 的「厚、高」只是链断时的退路矩形：
  改为 DESP 满两项才有退路，目录求不出又没有退路时才报错。
- aps250110：两堵墙对 E3D exact，failed 13 → 11。真库门 `aps250110_structures_real`（世界包围盒到 0.06 mm）。

## 2026-10-22 GENPRI 剖分照 SMLib `IwTess`：面细分树 + 等参边断开 + 曲线二分——aps250162 exact 451 → 461

- 原来各面按节点区间一律画网格线：GENPRI 10 / 14 板边那段 R5 半圆（两段有理二次）在节点处多了一个采样，
  正落在半圆顶上，包围盒比 E3D 高 0.33 mm；E3D 的面树对 2 mm 宽的窄面根本不分。
- 改照 TbAg.dll（E3D 2.10）反编译的 `IwTess`：
  - 面：`BuildTreeWithSubdivision` 从曲面整个参数域起建树。节点控制网厚度（插节点截出）超弦高就沿弯的那向
    分；否则尺寸向量（`ComputeSizeTolerances`：u / v 向各三条等参线的最大长度，平面用四角弦长）超最大
    边长就沿长的那向分。分点取中点，四分之一宽内有节点值就吸到节点上；内部点取叶子角点。
  - 边：等参边在相邻面叶子边界处断开（`IsoEdgeSubdivide`）；UV 里比所压叶子宽 1.1 倍的段从中点断开
    （`SplitFaceEdgesToUVSize`）；每段再按 `IwCurve::TessellateByBisection` 二分（Hermite 内侧 Bézier 点的弦高，
    超容差减半、不到一半加倍，C1 断点复验，闭合曲线整段不收）。闭合边另在节点处断开：E3D 把整圈的
    细圆柱切成参数 16 等分，出处还没找到，按节点断开留住象限点。
- 对拍 aps250162：exact 451 → 461，chord 15 → 5，off 0。只剩 GENPRI 13（0.49 mm）：E3D 在 76 mm 长的条上把
  R5 圆弧切得更细，来自 `BuildTree` 之后的边界植入（`ImplantUVSegment`）或面剖分，未反编译。其余库没有
  GENPRI，只动 `genpri`。
- 单测 +3（GENPRI 10 包围盒到 0.06 mm、曲线解析导数、两种圆弧的二分步数）；真库门局部包围盒加 GENPRI 10。

## 2026-10-22 通用图元 GENPRI 建体：解码归档里的 SMLib B-rep，UV 约束剖分——aps250162 75 只全出，60 exact / 15 chord

- GENPRI 的几何不在属性里，在 `GENPRI → ARCHIV → ATTDAT.ELDATA`：E3D 外壳（归档版本 2、字节数、UUID、
  `GeoTransf3d` 类型）+ SMLib 原生 B-rep（`IwBrepData`，数据版本 32），曲线、曲面全是 B 样条（类码 14033 /
  15002）。XBaseDll 3.1 坐实读取链 `GeoIwBrepGen::ReadFrom → GeoIwBrep::ReadFrom → IwBrepData::ReadFromDB`；
  aps250162 的 15 种 75 件逐字解完，布局表写在模块文档里。
- 新模块 `genpri`：解码、NURBS 求值、三角化。E3D 这一步是 `GeoIwBrepImpl::Tessellate` → SMLib `IwTess`：
  最大边长 = 0.4 × B-rep 包围盒对角线；次长边 < 1 mm 且最长边 < 20 mm 不出；细长件收弦高。
  各面先定网格线，边的采样（自身弦高 / 边长 + 相邻面网格线的穿越点）两侧面共用，水密；面在 UV 里按三维长度
  拉伸后做约束 Delaunay（新依赖 `spade`，与 rvm-rs 同版），按约束边奇偶漫水取修剪区域；修剪环自交先二分
  加密，曲线真相交就在交点捏合（GENPRI 13 端面两段圆弧交叉 0.003 mm，E3D 在那里同样多出一个顶点）；
  绕向按三角形传播、按分量体积翻正。导出弦高暂取 6 mm（E3D 分段反推）。
- 分类：GENPRI 仍走 `Composite(Equipment)`（字典 IECOMP 位），`is_model_unit` 认它；ARCHIV / ATTDAT 是属主
  消费的成员。`gen_ams` 对 GENPRI 只写 `FacetGroup`（E3D 75 只全是；我们分段粗的圆柱会被认成 `Pyramid`）。
- 读取集合加 `ELDATA`（字典 DCHC = 0：归档变了几何就变，不许当元数据放行）。
- 对拍 aps250162：两侧都有 391 → 466，exact 391 → 451，chord 15（GENPRI 10 / 14 包围盒差 0.33 mm、
  GENPRI 13 差 0.49 mm），off 0，只在 E3D 75 → 0。土建、S1、MDS 四套框、`/SSC测试`、FIXING / datum 逐件不变。
- 单测 7 条（`tests/fixtures/genpri/` 三块真 `ELDATA`）；真库门 `genpri_real`（75 只全成封闭实体、三只局部包围盒
  对 E3D）。

## 2026-10-22 深于半球的 `SphericalDish` 按 E3D 声明盒判帧：aps250128 不再整份拒判，190 对全 exact

- aps250128（APS 管道 AREA02 车间）的 E3D TTY 导出里，`/100-FW-201` 等 6 只阀门的阀帽是 r 44.1 / h 80.6 的
  `SphericalDish`：深于半球，真几何在球心那一圈宽到 R = (r² + h²) / 2h = 52.365；E3D 写进记录的盒却只到
  底圆半径 `[−44.1, −44.1, 0] .. [44.1, 44.1, 80.6]`。差 8.265 mm 顶穿判帧容差（8.056），`rvm_compare` 整份拒判。
- `Primitive::declared_box_mm`：h > r 的 `SphericalDish` 给声明盒 `[−r, −r, 0] .. [r, r, h]`，判帧多一个候选；
  h ≤ r 时它就是真几何，不另给。空间判定照旧只认真几何，写出端不变。
- 普查盘上 546 份 E3D 3.1 导出物（gen-model `output/rvm_dish_census.py`）：107 条 `SphericalDish` 逐条与这只盒
  相同（残差 0），其中 24 条 h > r；除上面 6 条，aps250132 的 18 条（r 17.358 / h 26）此前只是靠容差混过去。
  改后 546 份判帧 unknown 0 条。
- aps250128 第一次能对拍：190 对全 exact，记录种类全部一致。土建、S1、MDS 四套框、`/SSC测试`、FIXING / datum、
  ams8000、APS 其余逐件不变。
- 单测 `deep_spherical_dish_frames_by_the_base_radius_box`。

## 2026-10-22 圆管圆周 p-line（`PX` / `PY` 空程序）落在管外圆上：GENSEC datum 真值 3588 → 3678 / 3678

- MDS 支架 FT44 斜撑（ams7324 / 7329 / 7350 各 30 个）的 PLDATU，`POSL` 是 CTUB p-line 集 `/GTYPE_CTUB_PTSS` 里的圆周线
  VV（315°，30 个）与 PP（225°，60 个）。这一集 24 条圆周线里只有 0° / 90° / 180° / 270° 四条写了 `PX` / `PY`
  （`ATTRIB RPRO DPTH / 2`），其余 20 条两个表达式程序都空着（原始字全 0；常量 0 存 `[0, 1, 0, 0]`）。E3D TTY 真值里
  这 90 个 PLDATU 离 JLDATU 站位一个管外半径、沿 PLAX（CHS 88.9 / 114.3 / 114.0 → 44.45 / 57.15 / 57.0 mm），我们当
  常量 0 放在圆心。
- `section::pline_offset`：`PX` / `PY` 都是空程序时，取截面里最大的 SANN（以它自己的 `PX` / `PY` 为圆心）沿 PLAX 的
  半径点；没有 SANN 的截面照旧取程序值。`datum::solve` 的 `POSL` 与 `datum::section_pline`（墙上 FIXING 的
  JUSL / POSL）都走它。core.dll 里的求值没有反查；别的截面上的空程序线没有语料，照旧按 0。
- 真值：datum 3678 / 3678；名下 90 只 JOINT1 FIXING（没有 SPRE、不出实体）的世界标架随之对上。土建、`/SSC测试`、
  MDS 四套框、ams8000、APS、S1、其余 FIXING 逐件不变。
- 真库门 `gensec_end_datum_real::circumference_pline_datums_sit_on_the_tube_wall`。

## 2026-10-22 扶手面板 `HRPANE` 按立柱站位现算标架、PFIT 目录实例转 `BANG`：aps250110 off 56 → 1

- aps250110 `/B01_LEVEL_03_ACCESS/HR_001` 的两块扶手面板 `HRPANE`（`HRPNSE` 名下）没有 POS / ORI，模板里的
  PANE / GENSEC / DISH 55 件按 HANDRA 的标架摆，偏 3.3 m。对 E3D TTY 导出反算：面板与立柱同一套站位（`POSREF`
  路径点 + `ZDIS` 沿路径，`HRJUSL LEFT` 时 X 顺路径），再在站位标架里加 `(0, −XOFF, ZOFF)`。
  `transform::ImplicitFrame::HandrailPost` 认 `HRPANE`，`increment::implicit_frame_host_of` 把 `HRPNSE` / `HRPANE`
  算进详图式扶手的输入。
- 同库墙板 `/B02_LEVEL_01_WALLS` PANE 1 上 `BANG = 90` 的 PFIT：目录实例绕板法向转 `BANG`（不进 PFIT 自己的 ORI），
  此前 1600 × 300 的板长短边对调。`catalogue::panel_fitting_world` 右乘 `Rz(BANG)`。
- 对拍：aps250110 off 56 → 1（剩 `/AREA02_PR_AXIS_P2.2` 截面宽 1.07 mm）。土建、`/SSC测试`、MDS 四套框、
  ams8000、S1、FIXING / datum 真值逐件不变。
- 真库门 `rvm_record_kinds_real::handrail_panels_and_panel_fittings_sit_where_e3d_draws_them`；
  `ancestor_reads_guard` 里 `handrail_post_world` 加读 `XOFF` / `ZOFF`、`panel_fitting_world` 加读 `BANG`。

## 2026-10-22 端头斜切的 SREC 截面成员写面片、折边设计 DISH 写 `EllipticalDish`：aps250110 记录种类不同 40 → 10

- APS aps250110 整库对 E3D TTY 只读导出（gen-model `verification/civil-tty-20261021/aps250110_lv6_*.rvm`）：门框模板
  （`FITT 1 of STWALL n of CWALL …`）里 45° 斜接的 FBAR 20 只，E3D 写面片、我们写 `Pyramid`。截面就是一只 SREC，
  可端头斜切就不是棱台；包围盒自校拦不住——斜接前后包围盒一样。这是 happyrust/e3d-model@9d08daa 把单 SREC
  截面也交出身之后才有的（那一批语料里的 STWALL 都是平头）。
- `section::SectionCutPlane::is_square`（法向偏离轴 ≤ 1e-9，BANG 旋出的余量不算斜切）；`elmodl` 的截面本体
  带 `square_ends`，两端不都是平头时 SREC 成员不交出身，走拉伸体族的面片。
- 扶手立柱模板里 10 只 `DIAM 60.3 / HEIG 30.15 / RADI 25` 的设计 DISH：E3D 的 DRDISH 只看折边半径，写
  `EllipticalDish`；半球按长相先被球形碟认走。`elmodl` 给 `RADI ≠ 0` 的 DISH 记 `knuckled_dish`，`gen_ams`
  的设计出身按它挑 `EllipticalDish`。
- 对拍：aps250110 记录种类不同 40 → 10（剩整圈设计 REVO 的记录条数），几何判档不变。土建、`/SSC测试`、
  MDS 四套框、ams8000、S1、FIXING / datum 真值逐件不变。
- 真库门 `rvm_record_kinds_real::mitred_flat_bars_and_knuckled_dishes_follow_e3d_writers`。

## 2026-10-22 目录 `SEXT` 一律写 `FacetGroup`：整圆截面的挤出也不认成 `Cylinder`

- ams8000 竖梯 FTUB（目录 `/ACP1000-RFVL-R3B-500`，14 只）的圆板是整圆截面的 SEXT，E3D TTY 导出里全是
  `FacetGroup`；我们的长相识别器把它们认成了 `Cylinder`（每只 22–55 条）。同库 3 只 `ELCONN` 的矩形 SEXT
  认成了 `Pyramid`。
- E3D 的拉伸写出器（`DRWEXT`，记录种类常数 11）对目录 SEXT 只写面片。此前只挡了 `Box`
  （`allow_box_recogniser`），现在 `gen_ams` 的 `catalogue_noun_writes_facets_only` 让目录 SEXT 只走出身
  （与拉伸体族同一条），按长相的识别器一把都不上场；`rvm_component_geometry` 的 `allow_box` 参数随之去掉。
- 对拍：ams8000 记录种类不同 45 → 28（剩下的是整圈 SREV 的记录条数），几何判档不变。土建、
  `/SSC测试`、MDS、S1、FIXING / datum 真值逐件不变。
- 真库门 `rvm_record_kinds_real::catalogue_extrusions_stay_facet_groups_even_when_round`；单测
  `a_catalogue_sext_is_offered_no_shape_recogniser`。

## 2026-10-22 电气连接件 `ELCONN` 按 `CATR` 进目录链：ams8000 34 只接线盒 / 底座对上 E3D

- ams8000 整库按 ZONE 的 E3D TTY 导出里，`/1RX03-CASE-DQ`、`/1RX03-CASE-YK` 两个区的设备名下有 34 只
  `ELCONN`（如 `24384/24874`），E3D 每只一组面片，我们一只不出：类 28 里只有 `NOZZ` 接了目录求值，
  `ELCONN` / `EQUCOM` 当时没有语料。
- `category::enters_catalogue_by_catref` 加上 `ELCONN`：它与 `NOZZ` 一样只有 `CATR`（这里是
  `CATR → TABITE → PRTREF → GPART → SCOM`，`catalogue_point::follow_catalogue_component` 本来就跟得通），
  `POS` / `ORI` 是自己的，模型单元与管线产出同进同退。`EQUCOM` 照旧没有语料、不接。
- 对拍：ams8000 10 个 ZONE 只在 E3D 的 34 → 0，新增 34 对全 exact。其中 28 只接线盒 E3D 把整圈 SREV 拆成
  两半写两条面片、我们写一条（几何相同，记录条数差一），另起一条跟。土建、`/SSC测试`、MDS、S1、
  FIXING / datum 真值逐件不变。
- 真库门 `rvm_record_kinds_real::electrical_connections_enter_their_catalogue_by_catr`；单测
  `the_nozzle_enters_its_catalogue_by_catref_and_is_a_unit` 改钉 ELCONN 进、EQUCOM 不进。

## 2026-10-22 截面 `SPRE` 悬空的角钢上，FITT 的径向落在截面 x：ams8000 暖通支架 22 只端板对上 E3D

- ams8000 整库按 ZONE 用 E3D TTY 只读导出（gen-model `verification/civil-tty-20261021/ams8000_lv6_*.rvm`，
  10 个 ZONE）对拍：`/1CSV-HVACHB` 三个支架区的 22 只 FITT（`/-SR-CSV-S5001-V3/F1` 等，SFIT `/M8XP1`
  端板）全是 off，11–110 mm。板的尺寸与目录偏移都对，只是整个绕梁轴转了 90°。
- 这些竖杆的角钢 `SPRE`（`13244/271598` 等）在 db 5052 里已经不存在：E3D 自己也没有截面，梁只画一条
  `Line`。`section::section_fitting_placement` 原先对「截面目录读不到」一律按径向 π/2 兜底（那是
  ams7997 `/-KX-CAM-S4001` 缺 db 7600 时定的，那里 E3D 手上有截面）。现在分开：`SPRE` 在开着的库里
  查无此号（`DbElementError::NotFound`）时径向取 0（截面 x），库没注册时照旧 π/2。
- 对拍：ams8000 10 个 ZONE 2528 对全 exact（此前 off 22）。土建、`/SSC测试`、MDS、S1、FIXING / datum
  真值逐件不变。
- 真库门 `section_fitting_real::fittings_on_a_section_with_a_dangling_spre_turn_like_e3d`（270 × 110 与
  270 × 270 两块板，世界包围盒对 E3D TTY RVM 的 Box 到 0.1 mm）。

## 2026-10-22 暖通分段弯头的 SSLC 斜切节写带斜切角的 `Snout`，零径目录圆台照写记录：e3d-model#6 三个区记录种类全部一致

- happyrust/e3d-model#6 的三个暖通区（ams7997 `/-KX-CAM-E01`、`/-KX-CAM-S01`、`/-RX-CAV-E01`）正式对拍后几何
  212 对全 exact，记录种类不同 62 件：分段弯头（BEND 33、ELBO 28）的斜切节是目录 `SSLC`，E3D 走 DRSNO
  写带斜切角的 `Snout`，我们写面片；软接头 `/1CAM501FL`（目录 `/CFLEX`）少 27 条 `Snout[0, 0, 2, …]`。
- `CataloguePartRecord::analytic`：目录件逐块带上网格认不出的那族的参数——`SSLC` 的半径、高与四个
  斜切角本来就在网格键里（`catalogue::mesh_key_analytic`）。`gen_ams` 用设计 SLCY 的同一个写法
  （`rvm_write::analytic_from_primitive`，记录帧 = 元素世界帧 × 目录标架）直出，自校闸照过；被自己的
  NGMS 切过的块不给。
- `catalogue::evaluate_catalogue_snout`：两端直径都是 0、高还在的 LSNO 不再丢弃，和高为 0 的法兰盘一样走
  `SnoutBuild::Bodyless` 记一条无体 `Snout`（原 `Flat`，现在带高）。
- 对拍：三个区 96 / 34 / 82 对全 exact、记录种类不同 62 → 0，`Snout` 记录逐条与 E3D 相同（斜切角 ±11.25°）。土建、
  `/SSC测试`、MDS、S1、FIXING / datum 真值逐件不变。
- 真库门 `rvm_record_kinds_real::hvac_mitred_bends_and_flex_extensions_write_e3d_snouts`；单测
  `a_slope_cylinder_key_carries_its_snout_parameters`。

## 2026-10-22 桥架拼接板的 OUTA / OUTB 只在 addin 口径下算数：C-IY 对 E3D TTY 重导（holes on）35/35 exact

- 用当前 ams8000 把 `/C-IY-1R330-B` 按 addin 基线的表现口径（LEVEL PIPE 6、REPRE OBST ON）TTY 重导两份
  （gen-model `verification/ciy-tty-20261022/`）：`export holes on` 与 `off` 两份里 12 只 2.85°–10° 的 TBR-100
  槽弯都画着 L / R 拼接板（`DESP[4] = DESP[5] = 1`，目录说明「0: Have, 1: No」）；只有 GenModelRvmExport addin
  那份基线（`C-IY-1R330-B.rvm`）没有。`catalogue::application_filtered` 原先一律按 OUTA / OUTB 藏拼接板，
  TTY 口径下这 12 只差约 100 mm。
- `catalogue::export_holes_on`（从 `gen_ams` 挪进库，`E3D_EXPORT_HOLES` 缺省 on）：OUTA / OUTB 只在 `off`
  （addin 口径）时算数；`COVR = 0` 的盖板两种口径都不画。
- 对拍：C-IY 对今日 TTY holes on 导出 35/35 exact（此前 23/35），对 addin 基线（holes off）仍 35/35；
  TTY holes off 那份另有 addin 没有的盒（FTUBE 多两条 Box），不当基线。土建、`/SSC测试`、
  MDS、S1（含 1RX07-LCT 的槽弯）、FIXING / datum 真值逐件不变。
- `pipe_tray_rvm`（`#[ignore]`，对 addin 基线）的运行口径补上 `E3D_EXPORT_HOLES=off`。

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

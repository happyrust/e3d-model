# Changelog

## 2026-10-25 带 FRAD 圆角的详图式 RAIL 逐节写面片——aps250112 10 根栏杆的记录条数对上

- E3D 把详图式 RAIL 沿 RPATH 逐节写：直段一条（整张截面一次挤出，管截面的两个 SANN 半环在同一条里）、`FRAD` 圆角弧**每个 GMSS 成员
  一条**（各走一次 `gm_CreateRevolution`）：aps250112 `RAIL 1 / 2 of RAILSET 1 of /EL(+)104880_TOS_PLATE/HR_003`（3 直段 + 2 弧）各 7 条，
  `/EL(+)107820_TOS/HR_001` 的 16 条、`/EL(+)102470_TOS_PLATE/HR_001` 的 13 条同款（`aps_survey_lv6_2013286656_2.rvm`）；我们整根一条
  （W7a 时记为种类不同 10，几何 exact / chord ≤ 0.15）。
- 只有直段的折线（拐角斜接）E3D 仍是整根一条：aps250110 `/B01_LEVEL_01_HANDRAILS/HR_001` 3 节、`/BUILDING_01_LEVEL_03/HR_001` 6 节、
  aps250112 `RAIL 1 of RAILSET 1 of /EL(+)102470_TOS_PLATE/HR_001`（一节直段 + 末端一只弧长 0.085 mm 的退化圆角，E3D 不写那只弧）都是
  一条，带 `CURTYP THRU` 弧的路径也是一条（aps250110 `/BUILDING_01_LEVEL_03/HR_001`：4 直段 + 1 段 R≈14 m 的 THRU 弧，E3D 一条 326 个多边形）；
  第一版把它们也拆了（`gates-w7v4` aps250110 种类不同 0 → 8），现在只在路径带弧长 ≥ 1 mm 的 **FRAD** 圆角弧时才逐节写。
- `handrail::detailed_item_local_solid` 本来就是逐节建再 `compose`：现在弧逐成员各扫一块（单成员的 `SectionProfile` 走 `spine_swept_solid`），
  `RailBuild::Solid` 把逐节的块（含 `ZOFF`）一并交出；`elmodl` 对 RAIL（不含 KICKPL，没有分条的样本）且没有负体时把它们交 `parts`
  （出身不交，与 BPANEL 同款），写出端逐块一条 `FacetGroup`。几何不动。
- 真库门 `rvm_record_kinds_real::a_detailed_rail_with_fillets_writes_one_facet_group_per_piece_like_e3d`（HR_003 两根各 7 条、世界盒照旧；
  102470 HR_001 的 RAILSET 1 一条、RAILSET 2 13 条）。aps250112 种类不同 10 → 0，其余回归门 0 变化。

## 2026-10-25 变截面型材按两端截面放样——aps7200 `/TAPER-SCTN1..4` 的锥管不再画成直管，记录条数也对上

- SPRF `/OD-CONE`（`GTYP VTUB`）的两个 `PANG=180` SANN 带着 `DRAD = 0.5·DESP[2] − 0.5·DESP[1]`、`DWID`、`DX` / `DY`：半径 /
  壁厚 / 圆心从 POSS 到 POSE 的增量。E3D 在两端各求一张截面放样（`/TAPER-SCTN1` 外径 1000 → 500、壁 200 → 200），弧按 10 mm
  口径、**起点**半径定段（外弧 24 / 内弧 20 段，半圈各 12 / 10 步），整只写一条面片（两个半壳在同一条里，径向的内面不写），
  面片体积 1.625157e9 mm³；我们不读这四个属性，按等截面挤成 2.26e9 的直管、逐成员两条记录——包围盒恰好相同，对拍一直是
  exact，只在种类不同里露头（4 件）。
- `section::sann_rings` 读 `DRAD` / `DWID` / `DX` / `DY`，有增量时成员带 `taper`（POSE 端的环，与起点环逐点对应：两端同一组段数，
  `tapered_annulus_rings`）；`section_local_solid` 对带 `taper` 的成员在挤出区间两端各插一张截面放样（新的 `solid::loft_rings`：侧面四边形、
  两端带孔三角化封口），端切照旧；`SectionSolid::tapered` 走到 `elmodl`，变截面的型材不逐成员交 `parts`，整只一条。等截面的
  SANN 照旧走 0.5 mm 曲面口径（ams7997 DTUB 那 68 根不动）；沿 SPINE 的扫掠遇到变截面成员只记 note、按起点截面画（没有样本）。
  读取集合 169 → 171（`DRAD` / `DWID`；`DX` / `DY` 本就在），gen-model 那份一起重生成。
- 单元 `solid::loft_between_two_rings_is_the_frustum`、`section::a_tapered_tube_lofts_both_half_annuli_between_its_two_end_sections`；真库门
  `rvm_record_kinds_real::a_tapered_section_lofts_between_its_end_sections_in_one_facet_group`（世界盒不变、体积对 E3D 面片 0.1%、一条）。
  aps7200 种类不同 4 → 0，其余回归门 0 变化。

## 2026-10-25 多面体逐连通体写一条面片——aps7200 `/STAIRC.TOP` 两只 POHE 的记录条数对上

- `POHE 1 / 2 of SUBS 1 of /STAIRC.TOP`（`23584/9700`、`9753`）各是 8 块互不相连的薄板（六边形板 2 + 矩形板 6，厚 0.99），
  E3D TTY 导出逐连通体写、各 8 条 `FacetGroup`；我们把多面体按拉伸体的规矩（MDS `4-BAR-1` 两个连通分量 E3D 只写一条）
  合成一条。`gen_ams` 的「没有目录出身的面片族整只一条」给 `Category::Polyhedron` 开例外，按连通性逐块一条；几何不动。
- 真库门 `rvm_record_kinds_real::a_polyhedron_writes_one_facet_group_per_body_like_e3d`（世界盒对 E3D 0.05 mm、8 条）。
  aps7200 种类不同 6 → 4（余 `/TAPER-SCTN1..4`：E3D 把变截面型材的两个 SANN 半环写在一条里，且半径按 `DRAD` 从 1000 收到 500，
  我们还不读 `DRAD` / `DWID` / `DX` / `DY`，包围盒恰好相同）。回归门 `w7v2`（与回转图元那条同一轮）其余 0 变化。

## 2026-10-25 扫角超过 180° 的回转图元照 libgm 拆两段写两条面片——aps250209 的 REVO、ams8000 28 只接线盒的整圈 SREV、aps7200 / aps250110 的整圈 REVO 对上

- libgm `gm_CreateRevolution` 一次只扫 ≤ 180°：超过的拆成两段各扫 ANGL/2、**各按自己的扫角定段数**，再 `gm_CreateCombination(1)`
  并起来；E3D 的 RVM 驱动照两段写两条 `FacetGroup`（记录矩阵相同）。取证：aps250209 `/AREA07-CV_0001/Belt` REVO 2（`ANGL 190`）
  E3D 95° + 95° 各 5 步（10 mm 口径 R 409.75 整圈 16 段），我们整段一次算 9 步 21.1°，包围盒差 1.18 mm（这就是 W7t 收口时剩的
  那 1 件 off）；`/AREA05-CV_0001/Belt` REVO 4（210°）105° + 105° 各 5 步；同库 180° 整的 REVO 4 of `/AREA07-CV_0001/Belt` 一条；
  ams8000 接线盒 `ELCONN 1 of /1-WCC-151-BC` 的目录 SREV（`PANG = −360`）180° + 180° 各 4 步，E3D 先写朝 +Z 的那半；
  aps7200 `REVO 1 / 2 of TMPL 1 of FITT 1 of STWA 3 of /WALLS`、aps250110 10 只整圈设计 REVO 同款。
- `solid::revolution_is_split`（|ANGL| > 180°）；`revolve_rings_primitive` 建整体、角度格按两段之和铺（190° → 10 步，与拆开建的逐点相同；
  整圈的格不变）；`revolve_rings_profile_faceted_parts` 交两块记录单位（第二块 = 第一块绕局部 X 再转 ANGL/2；负角两块对调）。
  设计 REVO：没有负体时 `elmodl` 把两块交 `parts`（出身不交，与 BPANEL 同款），有负体照旧布尔结果整只一条；NREV 刀同一张格。
  目录 SREV：名下没有刀时两块各一个网格键（`SrevMeshKey.part` 1 / 2，`SREV_TESSELLATION_VERSION` 2 → 3）、同一个放置，
  各一条记录；挂着刀的照旧整只（没有样本）。`gen_ams` 的 `e3d_exports_facets_only` 加 `Category::Revolution`：E3D 的 DRWREV
  一律面片，半环不许被按长相认成 `RectangularTorus`。
- **只给回转图元**：型材沿弧 SPINE 的扫掠与目录 NSRE 刀照旧 `revolve_rings_profile_faceted`（整段一次定段数）——第一轮
  （`gates-w7v1`）把两段的格透进了这条路，aps250110 / aps250112 的 18 只钢筋箍 GENSEC（整圈弧 SPINE）偏 5.7–6.2 mm，E3D 的
  `DB_Gensec` 扫掠自己按整段定段数；收回后全部 exact。单元 `solid::a_revolution_over_180_degrees_is_built_on_the_two_half_grid`
  把两张格都钉住。
- 真库门 `rvm_record_kinds_real::a_revolution_over_180_degrees_writes_two_facet_groups_like_e3d`：REVO 2 的世界盒对 E3D 0.05 mm、
  两条，REVO 4（180°）一条；`ELCONN 1 of /1-WCC-151-BC` 7 条 + 世界盒。对拍：aps250209 4623 对 4622 exact、off 1、种类不同 2 →
  4623 exact、off 0、种类不同 0；ams8000 种类不同 28 → 0；aps7200 种类不同 8 → 6；aps250110 种类不同 10 → 0；其余回归门 0 变化。

## 2026-10-24 ATTA 断不断隐式管身按它自己的 `SPKBRK`，不再按目录 GTYP、也不分学科——ams8000 `/M-OR-1RX-3081D` 多出的两根归零，ams8000 2614 对全 exact

- ams8000 `/M-OR-1RX-3081D` 挂三对 MDS 托架支撑的 DATUM / SREF ATTA，E3D 写三根隐式管（SREF → BEND 820、两只弯头间 4.17 / 4.18），
  我们按「SCOM `GTYP=ANCI` 是站点」多写 DATUM → SREF 240、FTUB → DATUM 140 两根（off 603、种类不同）。逐只对属性：
  `/1RX03-MSUP-E0017`、`E0018` 两对 ATTA 是梯架支撑里仅有的没存 `SPKBRK` 的（`E0015` 那对与 ams8000 其余 233 只全真；另两只在
  `/M-OR-1RX-3AF0A`，槽位全零长）；E3D 自己记在 `SPAMAP` 第二个字里的隐式管空间键也只在画了管的成员上有值。目录链、LSTU
  （E0017 / E0018 的是 `/ACP1000-Ladder` 那套，别的支管同样混用照画）、PCON、成对正反向槽位的形态逐一排除。
- 旁证：ams7997 `/Copy-of-1RCS0620-1R80025` 的 `/Copy-of-NP-R322003-DAB` 是 `/AVEVA-ATTA-SPEC` 那族（`GTYP=ATTA`，此前一律穿过）里
  唯一 `SPKBRK` 真的，只读 TTY 导出（gen-model `verification/civil-tty-20261021/spkbrk_lv6_24381_145993.rvm`）E3D 把 WELD 5 → WELD 6
  那根在它那里切成 124.0 + 300.0；管道支撑附着点（ams8000 43 + 2、ams7997 1848、aps7200 60 只）与风管链上的 MDS 支撑
  （ams7997 572、ams8000 2 只，`GTYP=ANCI`）全没存，与「GTYP=ATTA 穿过」「风管 ATTA 一律穿过」两条经验判据的结论一致——它们只是
  和 `SPKBRK` 相关。旧 fast_model（`cata_model.rs`）也是按 `SPKBRK` 跳过不断管的 ATTA。
- `route::station_verdict` 第三参改为 ATTA 的 `SPKBRK`（存了值给 `Some`，没存按布尔缺省 `Some(false)`，模板里没有才 `None` →
  `PassThroughUnresolved`）；`member_station_verdict` 不再走 `SPRE → SCOM.GTYP`，风管学科不再特判 ATTA（SPLR 照旧穿过）。STIF / BRCO /
  SPLR 的模板里没有 `SPKBRK`，不走这条。读取集合 168 → 169（`SPKBRK`；gen-model 的 `generation_read_set.json` 同步）。
- 单元 `route::station_verdict_follows_spkbrk_for_atta`、`an_atta_on_a_duct_route_follows_the_same_spkbrk_rule`；真库门
  `rvm_record_kinds_real::an_atta_without_spkbrk_does_not_end_a_tube`（`/M-OR-1RX-3081D` 三根 `Cylinder[12.5, 819.995 / 4.169 / 4.176]`）。
  ams8000 对拍 2614 对 2613 exact、off 1、种类不同 29 → 2614 exact、off 0、种类不同 28（余 28 是整圈 SREV 的写法）。

## 2026-10-24 扁圆风管的隐式管身按管身目录件的几何集逐图元建——aps250142 `/SUPPLY_LEVEL02-001/02` 对上，aps250142 521 对全 exact

- 风管隐式直段的断面此前只认管身目录件 GMSE 里**第一只**可见的 BOXING / TUBE。扁圆风管 `LSTU /AVEVAHVACSPEC/FOTUBEB` → `/FOTUBE2` 可见的是五只：
  `-BOXI`（`PXLE = LEAWID − LEAHEI`、`PZLE = LEAHEI`）、`-TUB1` / `-TUB2`（`PDIA = LEAHEI`，轴是 PTSE 的 P4 / P5，`PX = ±(LEAWID − LEAHEI)/2`）与
  `-INS1` / `-INS2`（`OBST 1` 但 `TUFL` 真、`LEVE [0,10]`，`REPRE TUBE ON` 下 E3D 照画；`PDIA = LEAHEI + IPAR[1]`，保温为 0 时与 TUB1 / TUB2 重合）。
  aps250142 `/SUPPLY_LEVEL02-001/02` STIF → DAMP 那 4.41 mm 的槽位（LEAWID 500 / LEAHEI 100）E3D 写 `Box[100, 400, 4.41]` + 四条 `Cylinder[50, 4.41]`
  （圆柱在 ±200，两对字节一样），我们只按 TUB1 写一条居中的 `Cylinder[50, 4.41]`（off 200）。
- `hvac_tube::duct_departure_section` 现在收集全部可见的 BOXING / TUBE（仍是 `display_filtered` 那一道）：一只照旧；多只交 `TubeSection::Composite`
  ——BOXING 居中，TUBE 的 `PAXI = Pn` 取那个 P 点的 `PX` / `PZ`（同一套 `OF PREV` 环境求，求不出的分量是 0）作断面内的偏移，同形同位的成员并成
  一块记份数；`route::build_tube` 逐块按真尺寸建在滑转后的管身局部系里（滑转与矩形断面同一格 `z_axis`），`parts` / `part_nouns`（`BOXI` / `TUBE`）/
  `part_records`（份数 + 偏移标架）交给隐式管身的 `GeneratedElement`，`gen_ams` 照目录 noun 写 `Box` / `Cylinder`。`TubeSection` 不再 `Copy`。
  目录局部 X（`PXLE` 方向）与管身局部 X 的正向是否一致没有样本：语料里的偏移成对对称，符号不影响结果。
- 单元测试 `route::a_flat_oval_duct_body_builds_its_box_and_tubes_as_parts`；真库门 `rvm_record_kinds_real::a_flat_oval_duct_body_writes_its_box_and_tubes_like_e3d`
  （五条记录、世界盒对 E3D 0.05 mm、模型集一件三块）。aps250142 对拍 521 对 520 exact、off 1、种类不同 1 → 521 exact、off 0、种类不同 0。

## 2026-10-24 风管头槽位按目录写下的键在宿主 BRAN 上精确取 `:HXYsize`、`gen_ams` 挂 UDA 字典——aps250142 `/RETURN_LEVEL02-002/01` 的头管对上（并回 `w7j-uda`）

- 分支 `w7j-uda`（2026-10-23，`5e1649e`、`e11d359`，基于 `b758b28`）并回 main，自动合并无冲突。
- `gen_ams` 此前从不挂 UDA 字典，目录表达式里按键读 UDA 的那一类（风管 H 族头管的 `( :HXYsize[n] )`）在对拍里永远求不出值，而 gen-model 生产
  一直挂着（`UdaCatalogProvider`）。新增 `DirectoryDbResolver::uda_catalog`（给定 DICT 库号逐本读 `UdaCatalog`、按给出的顺序合并，库号由调用方给），
  `gen_ams` 用 UDET 那份 DICT 库号挂字典（APS 一套 5184 条定义）。
- 容器头 / 尾槽位的求值环境只挂目录件 SCOM，挂了字典也问不到 BRAN 上的 UDA。`hvac_tube::PreviousElementEnv` 加 `uda_host`（容器头 / 尾 = 宿主 BRAN）：
  按目录写下的键 `0x17EF4B61`（acp7006 `/UDA/HVAC/HXYsize` 的 `OLDKEY`）在 BRAN 上精确取（`DbElement::uda_by_key`），不走同名回退；字典不认这把键才交给
  目录件。aps250142 的 BRAN 在这把键下存着 450，头管（Cylinder r 225、2374.5 mm）与 E3D 对上；ams7997 的 BRAN 把值存在 ams5100 那把键
  `0x2652AB52` 下，精确取拿不到、仍 NoRadius——与 E3D 在那里求值失败一致（同名回退会画出 675 × 600）。矩形 H 族头槽位的断面现在读得出，但 BRAN
  没有 `LEAV`、滚转没有样本：`leave_frame_z_axis` 对缺 `LEAV` 按解不出记账。读取集合不动（按键读的是 UDA）。
- 真库门 `hvac_tube::a_head_slot_reads_the_branch_uda_by_the_catalogue_key_only`。分支上的回归门 `uda3` / `uda4`（对 `w7q`）：V1–V5、V7、V8
  0 变化，V9 只有 aps250142 变（exact 519 → 520、off 2 → 1、种类不同 2 → 1）；并回后的门见计划 §9。

## 2026-10-24 地形格的 GENPRI（归档版本 4，缓存面片）照 E3D 写成面片——aps250212 6 只对上，只在 E3D 归零

- aps250212 地形模型 `TRNMOD /TerrainA/DEF01/MOD01 → TRNCC 1 → TRNCEL 1..6` 每格一只 `GENPRI 1`，`ELDATA` 的归档版本是 4：
  没有 SMLib B-rep，是 `GeoGenericPrimitiveImpl::ReadFrom` 在版本 ≥ 3 多读的那份缓存面片（字节流：顶点号流 + 顶点表 + 法向表 + 法向号流，
  布局见 `genpri` 模块文档）。解码器此前只认版本 2，6 只全挂 `failed`（只在 E3D 6）。E3D 的 TTY 导出写的就是这份缓存：6 格的多边形数、
  环数、点数、包围盒逐一相同（格 6 的 103 个多边形逐环同点同序）。现在 `genpri::decode_archive` 按第 0 字分派：2 → B-rep、
  4 → `CachedFacets`；一张面的几个环按环向分组（与面法向反向的是孔、同向的各写一个多边形，这是 E3D 的取舍：格 2 的两张带孔面
  写成带两孔的多边形，格 3 里同向交缠的 20 / 21 边两环拆成两个）。
- 这些面片是面不是体：地形格是带一圈边界边的高程面（格 6 有 33 条只被一张面用的边），TRNSLD 的虽封闭却夹着零面积的折返环，
  建不成流形。照 E3D 只写 RVM：建模判 `Skipped(NoShape)`（理由写明是面），管线在复合单元的 `NoShape` 后也问一遍
  `bodyless_design_record`（与零尺寸 CYLI 同款），新的 `DegenerateCatalogueShape::OpenFacets` 走 [`GenerationOutcome::bodyless`]，
  `gen_ams` 用 `rvm_write::bodyless_facet_group_geometry` 写一条 `FacetGroup`（多边形 / 孔照归档，法线取外环面法线）；模型集、
  `elements.json` 与增量不动。同库 TRNSUR 的 GENPRI（`2013286756/1507`）变换类型是 2（非单位阵，跟 16 个实数），不在导出里、没有可对拍的
  语料，照例 `Layout` 拒读。
- 单元测试加格 6 的整块 `ELDATA`（`tests/fixtures/genpri/aps250212-2013286756-1844-terrain-cell6.eldata`）：103 面 / 141 点 / 尺寸直方图 /
  包围盒对 E3D，环分组规则另钩一条；真库门 `rvm_record_kinds_real` 加 `/TERRAIN-A-ZONE`：6 条 `FacetGroup`，格 1 / 2 / 6 的多边形数与环数、
  格 1 / 6 的世界盒对 E3D，`elements.json` 里没有它们。aps250212 对拍 0 对、只在 E3D 6 → 6 对全 exact。

## 2026-10-24 进了布尔的正 CTOR / RTOR 按 10 mm 粗格建——aps250209 设备模板的两只半圈管对上

- E3D 把 LNEGIT 的最终结果写成一条粗面片，**正体也按 10 mm 口径离散**，与负体同一张格（CYLI / CONE / DISH / 斜切圆柱早已这么做）；
  正环管、正矩形环此前仍按 0.5 mm 曲面口径建。aps250209 `CTORUS 1 / 2 of TMPLATE 1 of EQUIPMENT /A02-AREA05-PRIMARY-CRUSHER`
  （`RINS 1558.44 / ROUT 2377.56 / ANGL 180`，名下 NCTO 掉空、NRTO 切外半、NRTO 在 z −290 切下段）：E3D 的面片扫 18 段、管 16 段，
  正是 `negative_part_rev_segments(ROUT, ANGL)` × `negative_cylinder_segments(管半径)`；切外半的 NRTO `ROUT` 与正体相同（2377.56），
  同一张格下两者的外弦重合、切得干净。我们 78 × 64 的细格正体，外赤道的弦在 NRTO 的 18 条粗弦之间探出去，差完留下一圈 22 片碎块，
  包围盒比 E3D 大一个管半径（off 409.56）。现在 `primitive_local_solid` 的 `RectTorus` / `CircTorus` 在有负体（`allow_instance == false`）时
  走 `*_with_segments` 的粗格。同类证据：aps250164 `CTORUS 1 of SUBEQUIPMENT /C1101-SKIRT`（93 / 207 / 90 + NCTO）E3D 扫 3 段、管 8 段；
  aps7200 `RTORUS 1 of SUBSTRUCTURE /C1101-PL1-BASE`（ROUT 1850 / ANGL 165 + NCYL + NBOX）E3D 扫 15 段、每段 11°——这两件包围盒本来就对，
  只是面片比 E3D 密十来倍。语料里带负体的环共 347 只（ams7997 340 只 RTOR 不在 TTY 导出里），包围盒不随格数变，回归门不受影响。
- 结果与 E3D 同形：体积 8.9620e7 对 E3D 面片的 8.9621e7 mm³，连 E3D 自己留下的两处碎片（内赤道上正体的弦探进 NRTO 内圆 1555.62
  以内的 12 片、r = 1968 处 NRTO 顶点圈探出另一只 16 段 NRTO 粗弦的 4 处底部薄片）都一样有，z 范围同样是 ±409.56。
- 真库门 `rvm_record_kinds_real` 加这对：世界盒对 E3D 0.05 mm、体积 ±0.2%、每只一条 `FacetGroup`。aps250209 对拍 4623 对 4620 exact、
  off 3 → 4622 exact、off 1（整圈 REVO，ISSUE_09 既有类）、种类不同 2。

## 2026-10-24 布局式梯段 STRFLT 自己的斜梁、踏步与平台板——aps250209 18 只对上，只在 E3D 归零

- `INSTOP` 以 `LAYOUT` 开头的梯段没有 TREADS / STRSTR 成员，E3D 给 STRFLT 自己写一条 `FacetGroup`；我们把 STRFLT 当纯容器，一件不画
  （aps250209 只在 E3D 18）。新模块 `stair_flight`，照 TTY 导出的面片还原：两条宽 50 的斜梁贴在 `TWIDTH` 之外（中心 ±(TWIDTH/2 + 25)，
  正是 HANDRA 的 `POS`），上边是 RPATH、沿路径法向往下 `FTHICK`，拐角按过拐点的角平分面斜接、开口端按 RPATH 的 `DRNS` / `DRNE`
  （这些梯段 `DRNS = +X` 垂直于路径 → 平头，`DRNE = +Y` → 顶端竖切）；斜段上 `n = round(升高 / RISE)` 级踏步盒，进深 `g = 水平投影 / n`，
  每级从第 k 条立板到第 k+1 条再探 `TREADO`，厚 `g / 10`，第 0 级只在前面有平台段时画、第 n 级（上层平台）不画；水平段是宽
  `TWIDTH`、厚 `g / 10` 的平台板，拐角同样斜接。踏步厚的三条证据（26.28 / 24.37 / 24.91 对 g 262.75 / 243.7 / 249.09）里 `FTHICK` 恰好都等于 g，
  是 g/10 还是 FTHICK/10 还要一条两者不等的样本；`ORISE` / `OGOING` / `UEXTNS` / `LEXTNS` 语料里全是 0，非零时记 note。
- `category::is_model_unit` 把 STRFLT 认成单元（名下的 HANDRA 仍各自成件）；gb==2 容器的护栏给它开了唯一的例外（我们对的是 RVM 导出，
  它给 LAYOUT 梯段写了面片）。详图式（`INSTOP` 以 DETAIL 开头）照旧 `Skipped(NoShape)`，几何在成员上。`gen_ams` 的
  `e3d_exports_facets_only` 加 STRFLT，整只一条面片。踢脚板那套「沿直段扫的带」抽成 `handrail::band_segment_solid`，斜梁与平台板共用。
  读取集合多 `INSTOP` / `FTHICK` / `ORISE` / `UEXTNS` / `LEXTNS`（163 → 168），gen-model 那份一起重生成。
- 真库门 `rvm_record_kinds_real` 加 `/A01-AREA05-BASE-FL-STAIRS` 的两段：世界盒对 E3D 0.05 mm（平台 + 10 级的那段长到末级踢边、高到路径顶；
  从斜段起步的那段底端平头最低点 −194.67），每只一条 `FacetGroup`，四只扶手照旧各自成件。aps250209 对拍 4605 对 → 4623 对 4620 exact、
  off 3（CTORUS 负体布尔 2 + 整圈 REVO 1）、只在 E3D 0、种类不同 2。

## 2026-10-24 布局式扶手的两端按 RPATH 端向切、踢脚板在圆角拐角按拐点斜接、整只写一条面片——aps250209 38 只 HANDRA 对上

- **开口端**：`RPATH` 的 `DRNS` / `DRNE` 非零且与该段既不平行也不垂直时，栏杆与踢脚板的开口端是过端点、法向 = 端向的平面
  （`handrail::apply_end_directions`，复用 GENSEC SPINE 的 `spine_mitre_plane` 门），其余情形仍是垂直于轴的平头。aps250209
  `/A01-AREA05-BASE-FL-STAIRS` 等 8 段梯段的 16 只 HANDRA（平台段 + 斜段、`DRNE = +Y`）在 E3D TTY 导出里斜段顶端是 `y = 端点` 的竖直面
  （端柱还在端点上），我们按平头画多伸 24.07 − 15 = 9.07 mm；aps250110 `SF_002` 的端向是零、aps250112 `SF_001` 两端是平台段，
  两条规则在那里给出同一张面。off 16 → 0。
- **圆角拐角的踢脚板**：此前 `FRAD > 0` 的拐角踢脚板拒做（`layout kickplate does not support arc segments`），整只扶手都不出
  （aps250209 22 只）。E3D 的踢脚板不跟弧：直板一直画到原拐点、按角平分面斜接（`/A01-AREA05-4TH-FLOOR-HANDRAILS` FRAD 100 的水平拐角、
  `/A02-AREA07-HANDRAILS` 竖直面里的内弯拐角，板顶 / 板底都在过拐点的角平分面上相交），现在踢脚板按没有倒角的路径重排直段，栏杆照旧走弧。
  只在 E3D 22 → 0，全 exact。
- **记录种类**：E3D 给每只布局式 HANDRA 写一条 `FacetGroup`（aps250209 / aps250112 / aps250110 的导出都是）；我们的零件是 `compose`
  起来的，弧与直段不共顶点时会被按连通性拆开、环段被认成 `CircularTorus`、立柱认成 `Cylinder`（一只拆成 7 条）。`gen_ams`
  的 `e3d_exports_facets_only` 加 HANDRA，整只一条面片。
- 真库门 `rvm_record_kinds_real` 加两条（aps250209 在副本 SYS 库登记后导出）：梯段扶手 `2013286753/40206` / `40211` 的世界盒止于端柱
  36445.0，圆角踢脚板扶手 `40177` / `40183` 的世界盒对 E3D 0.05 mm，两条都钉每只 HANDRA 恰好一条 `FacetGroup`。祖先读守卫登记 RPATH
  的 `DRNS` / `DRNE`。aps250209 对拍 4583 对 4564 exact、off 19、只在 E3D 40 → 4605 对 4602 exact、off 3（CTORUS 负体布尔 2 + 整圈 REVO 1）、
  只在 E3D 18（全是 STRFLT 自身的面片）、种类不同 2。

## 2026-10-23 桥架分支的路径点照 E3D 写 10 mm 记号——aps7200 8 条空分支的 20 个 POINTR 对上

- E3D 给 `CWBRAN` 的 RPATH 每个 `POINTR` 写一只 10 × 10 × 10、沿世界轴、以点为中心的 `Box`（Core3D
  `CAB_CSGRoutePoint::getCSGTree`）：aps7200 zone `=23584/10278` 的 8 条没有托盘构件的分支、共 20 点，TTY 导出全是这样；
  我们把 POINTR 当路径数据，一条不写。记号不是实物，走只写 RVM 的通道 `GenerationOutcome::bodyless`（`rpath::route_point_marks`，
  新增 `DegenerateCatalogueShape::Box` 与 `rvm_write::bodyless_box_geometry`）：模型集、`elements.json` 与增量都不动。
- 口径：有 RPATH 就画。语料里只有这 8 条空分支有 CWBRAN 导出（ams8000 的桥架是 BRAN + FTUB，APS 其余 10 本没有 CWBRAN），
  挂着托盘件的分支 E3D 画不画记号没有样本，碰到反例再收窄；路径里的 CURVE / EXTGEO 成员不是点，跳过。
- 真库门 `rvm_record_kinds_real` 加这个 zone：20 只 `Box[10, 10, 10]`，名字、原点与三轴对 E3D，`elements.json` 里没有它们。
  aps7200 对拍只在 E3D 20 → 0、exact 2326 → 2346（2352 对）。

## 2026-10-23 管道容器下的矩形 TUBI 目录件照矩形画——aps7200 `/HVAC/RECT/M1/B2` 的支管管身对上

- 路由容器的学科按属主 noun 定（`PIPE` → 管道、`HVAC` → 风管），管道学科的隐式管身只读离开侧 stub 目录件的
  `PARA[2]` 外径。aps7200 `BRAN /HVAC/RECT/M1/B2` 挂在 PIPE `/HVAC/RECT/M1` 下，`OLET → ELBO` 那段的 `LSTU = /RDUCT/XYDUCT`：
  SCOM `/XYDUCT` 的 `PARA[2]` 是 1，断面写在 GMSE 的 BOXING 里（`PXLE` 100 × `PZLE` 50，目录元组里的常数），E3D 照矩形画
  （TTY `Box[50, 100, 2150]`），我们画成 1 mm 的圆管（off 49.5）。现在管道学科先按风管那条路（`hvac_tube::duct_departure_section`）
  问一遍：解得出**矩形**断面就用它；圆的、解不出的、读挂的照旧走 `PARA[2]` / 到达侧通径。记录标架按离开侧构件的滚转写
  （`Box[100, 50, 2150]`），与 E3D 按世界轴写的是同一只盒子。
- 真库门 `rvm_record_kinds_real` 加这段管身：世界包围盒对 E3D 0.05 mm。aps7200 off 4 → 3、种类不同 9 → 8。

## 2026-10-23 零尺寸的设计 CYLI 照 E3D 写一条空记录——aps7200 `/EQRACK.5M.HR5` 那只 `Cylinder[0, 0]` 对上

- 设计侧 CYLI `DIAM` / `HEIG` 为 0 时 Core 不出体，我们按尺寸退化跳过（账在 `skipped`），E3D 的 RVM 驱动却照走 DRCYL：
  aps7200 `CYLINDER 3 of SUBSTRUCTURE 1 of STRUCTURE /EQRACK.5M.HR5`（`23584/9172`，`LEVE [0,10]`）TTY 导出里是一条
  `Cylinder[0, 0]`，矩阵就是元素的世界标架。现在这种元素走新的只写 RVM 的通道 `GenerationOutcome::bodyless`
  （`elmodl::bodyless_design_record`，与点标记 `marks` 同款）：`gen_ams` 照 E3D 写空记录，模型集、`elements.json` 与增量都不动，
  跳过的账照记。表现级门与正体同一道。只认 CYLI：别的设计基本体零尺寸时 E3D 写不写，语料里还没有样本。
- 真库门 `rvm_record_kinds_real` 加这只 CYLI：记录、三轴与原点对 E3D，`elements.json` 里没有它。aps7200 对拍
  只在 E3D 21 → 20、exact 2324 → 2325。

## 2026-10-23 圆转方风管变径的偏心 Snout 按 E3D 的方向偏——aps7200 `/HVAC2-B1/ITEM8` 对上（修在 e3d-io）

- acp7002 `/FORD`（圆转方风管变径）本体的两只 LSNO 把偏心方向 `PBAX` 取自 P7 / P8，它们的 `PTCD` 是
  `AXIS -Y ( ATANT ( ±DESP[10] - (DESP[4] - DESP[5]) / 2 , 0 - DESP[11] ) ) ∓X`。e3d-io 的 PML 求值器此前对 `ATANT`
  一律拒答（两个参数谁是纵坐标没有对过 E3D），方向回到模板缺省 +Y，`TRNS /HVAC2-B1/ITEM8`（`23584/7410`）的两只 Snout
  往世界 +Z 偏 450 / 50 mm，E3D 往 −Y / +Y，对拍 off 375 mm。现在 `ATANT ( a , b )` = `atan2(a, b)`（度，纵坐标在前，保象限），
  按 E3D TTY 导出钉的：另一种次序给 180°、两只都留在 −Y。这样读，两条链对任意偏移都指向跑道形两个半圆心
  `(DESP[10] ± (DESP[4] - DESP[5]) / 2, DESP[11])`，正是目录作者写它的用意。
- 真库门 `rvm_record_kinds_real` 加这只变径：两只 Snout 的偏心量、记录 X 轴（偏心方向）与原点对 E3D 记录。
  aps7200 对拍 off 5 → 4、exact 2323 → 2324；其余语料里没有任何 `ATANT` 被求值（生成 ams8000 全库时追过）。

## 2026-10-23 挂着关节的弧墙两端不按 SPINE 端向斜切——aps7200 `/C1101-BASE` 的 165° 弧墙对上

- WALL 的一端挂着 PJOI（`JOIS` / `JOIE`，关节的 `CREF` 回指墙自己）时，端部归关节管，E3D 不按 SPINE 的 `DRNS` / `DRNE`
  斜切那一端。aps7200 `WALL 1 of FRMW /C1101-BASE`（`23584/6980`：165° 弧、`RADI` 1850、`JUSL OBOW`）两端各一只 PJOI，
  SPINE 却写着与切向成 44° 的 `DRNS` / `DRNE`（两只关节的 `CUTP` 就是它们翻了 Y 的向量）：此前按它们过端点切半空间，
  弧绕回来被两刀削得只剩 565 mm 一小块（对拍 off 1742 mm）；E3D TTY 导出里两端都是径向平头。现在只跳过挂着关节的
  那一端，没有关节的弧墙（ams1112 `/1RS-WF05-W-C-RR001` WALL 4，`DRNS = E`）E3D 照 `DRNS` 斜切、照旧。
- 语料（ams1112、ams7997、aps250110、aps7200）里只有这一堵 WALL 挂着关节，直墙挂关节的没有样本：直墙的端切不动，
  PJOI 的 `CUTP` 也没有拿来切。
- 真库门 `rvm_record_kinds_real` 加这堵弧墙的世界包围盒（对 E3D 面片，0.05 mm）。aps7200 对拍 off 6 → 5、exact 2322 → 2323。

## 2026-10-23 结构节点 SNOD 照 E3D 写点标记的三条线——aps7200 191 只节点补齐

- E3D 把 SNOD 导成 RVM 是三条 `Line[5, 20]`（Core3D `gm_CreateMark`）：以 SNOD 的站位为中点，按 X / Y / Z 的顺序
  沿宿主 SCTN 所在属主系的三轴——不随杆件轴，也不随 `BANG`。aps7200 TTY 导出的 191 只全是这样：SBFR 下的
  竖柱、反向梁、斜撑一律沿世界三轴，转过 −15° 的 STRU `/C1101-PL3`、墙上 FITT 模板、转过 −90° 的楼梯结构里的
  跟着属主系转。记录标架与阀杆线同一口径（`rvm_write::bodyless_line_geometry`）。此前一条都不写。
- 点标记走新的只写记录的通道 `GenerationOutcome::marks`（`pipeline::MarkLines`）：`gen_ams` 把它们写进 RVM，
  模型集（`elements`）、`elements.json` 与增量都不动——gen-model 的烘焙要实体，空体会把整根标成失败。
  `Report::centreline_only` 的账照旧记（模型集里仍没有它）。STWALL 名下的 SNOD 标架没接，不出标记。
- 真库门 `rvm_record_kinds_real` 加两只节点（世界三轴的 `/B-3-SNOD-7`、转过 −15° 的 `/WEST-KICKPLATE` 那只）。

## 2026-10-23 SCTN 名下 SNOD 里带目录正体的关节 SJOI 照目录件建、法兰端板切梁端——aps7200 34 只关节与 2 根梁对上

- SJOI 没有 POS / ORI，标架由 E3D 现算（`transform::ImplicitFrame::SectionJoint` → `datum::section_joint_world`）：
  Z 取 `CUTP`（属主系）；Y₀ 取宿主 SCTN 的轴去掉沿 Z 的分量，X₀ = Y₀ × Z，再绕 Z 转关节自己的 `BANG`。原点从
  SNOD 的站位沿宿主截面挪到 `POSL` 那条 p-line（减宿主 `JUSL`），再沿宿主轴挪到连接梁（`CREF`）`JLIN` 那条线上。
  APS 项目副本上只读导出的 34 只 TTY 世界位姿逐只对上（≤ 0.01 mm）。
- Z 的符号跟连接梁的滚转走（`datum::section_joint_cut_sign`）：aps7200 的连接梁全是 `BANG` 0，Z = `CUTP`；MDS 的
  4 只样本（ams7324 `/MDS/FT09` 的两只 `IJOINT1`、ams7330 PowerClick 的两只 `J1`）连接梁是 `BANG` 180，E3D 的 Z 与
  `CUTP` 反向，角板落在梁那一侧。连接梁转 ±90°、连接的就是宿主自己的关节没有样本：不出正体、不切梁端，标架
  仍沿 SNOD，与以前一样。第一版只按 `CUTP` 放，回归门拦下了 ams7324 的两只（翻到柱的另一侧，差 62 mm）与
  ams7330 那只自连关节（差 235 mm）。
- `SPRE` 走得到带 `GMRE` 的目录件（JOIN）的 SJOI 在元素层改判目录件（`category::element_category`，全量管线
  与增量重建同一个判据）；目录链读挂时照样按目录件走，失败记在建模那一步。正体照常按表现级过滤：aps7200 的
  32 只螺栓连接角钢与 2 只法兰端板各 26 条记录，种类、参数与 TTY RVM 一致；62 只焊接托座的图元全在 `LEVE`
  6–10，整只 NoShape，E3D 同样不画。没有 `SPRE` 的 SJOI（MDS 的全部）照旧记「无三维表示」。
- 目录 NAPP 门在 mode 2（SCTN 端关节）放行 `-1`：法兰端板 NGMS 里的 NSBO 把 `/B-BC-1A`、`/C-BC-1A` 的梁端退到
  端板内侧面，梁比不切短 11.95 mm，与 TTY 一致（此前 off 11.95）。
- 增量：关节级联从 FIXING ↔ GENSEC 推广到 SJOI ↔ SCTN（`increment::joint_consumer_of` / `joint_fixings_of`：
  SJOI 的 `CREF` 回指梁、梁的 `JOIS` / `JOIE` 指回它）；SNOD 里挂着目录关节的 SCTN 进现算标架级联
  （`implicit_frame_host_of`，可穿过 SNOD 上溯），柱或 SNOD 改了整棵柱子树重建。级联记账的 noun 改按库里读。
- aps7200 对拍：2297 对 / 2286 exact / off 8 / 只在 E3D 55 → 2331 / 2322 / 6 / 21。读取集合多 `CUTP`、`JLIN`（163）。
- 真库门：`aps7200_frames_real` 加 5 只 SJOI 的世界位姿；`rvm_record_kinds_real` 加法兰端板的记录种类与梁端
  包围盒、MDS `IJOINT1` 的世界包围盒；`increment` 单测钉 aps7200 关节的两头回指与柱的级联。

## 2026-10-22 PANE 上组合配件 CMPF 名下的 SBFI 贴板面——aps7200 `/FLR-PLT1` 的套管对上

- 板上 PFIT 的目录实例贴在 `SJUS` 指的那一面；PANE 上 CMPF 名下的 SBFI 同一口径，按 CMPF 自己的 `SJUS`
  定板面，CMPF 的标架不动。aps7200 `/FLR-PLT1`（30 mm 楼板，CMPF `SJUS UTOP`）的 SBFI 此前在板底、off 30 mm，
  现与 TTY 真值一致。
- 只认 PANE：ams1112 / ams7997 的 149 只 CMPF 都在 GWALL 上（`SJUS DBOT`），`POS.z` 是套管沿墙高的位置，
  按板面摆会把套管挑开几米（第一版就是这样被回归门拦下的），照旧沿 CMPF 原点。增量侧 PANE 的环与厚度
  改动级联到名下 CMPF。
- 真库门 `aps7200_frames_real` 加 CMPF / SBFI 一条。

## 2026-10-22 踢脚板的 datum 按 PKDI 比例站位、圆心式弧端点在圆内照样成弧——aps7200 3 只 FIXING 对上

- JLDATU 的站位 = `HPREF` 起点 + `ZDIS` + `PKDI × 路径全长`。`PKDI` 此前读了没用：aps7200
  `/EAST-KICKPLATE`（直梁 1020 mm，`PKDI = 0.5`）与 `/OUTER-KICKPLATE`（165° 三点圆弧）的 FIXING 偏 510 /
  1885 mm，现与 E3D TTY 真值差 ≤ 0.01 mm。AMS 的 3678 只 datum 真值对拍不变。
- GENSEC SPINE 的 `CURVE`（`CURTYP CENT`）两端都落在 `RADI` 圆内时没有切线，此前退成直段；E3D 沿以
  `POS` 为圆心、半径取终点距离的劣弧走（`/INNER-KICKPLATE`：`RADI` 1050，两端距圆心约 850）。改后它的
  JLDATU（`ZDIS 1531`）与 TTY 真值差 < 0.05 mm、FIXING exact；梁身还差 5.6 mm，是圆弧切分点数不同。
- 真库门 `aps7200_frames_real`（新）：三只 JLDATU 对 TTY 世界位姿（APS 项目副本上只读导出）。

## 2026-10-22 矩形风管弯头按设计角转——aps7200 11 只弯头的本体补齐（修在 e3d-io）

- 根因在 e3d-io `ca949ef`：命名角轴 `[26, 64]` 的角度字 64 落在 51..=100 段，是 `DESP 14`（acp7021 矩形
  风管弯头 `/DBS` 的弯角），此前按 `PARAM 64` 读、出界得 0。`P2` / `P6` 不转，弯头是直的，本体
  `RectangularTorus` 整只没写，对拍 off 125 mm。现在与目录元组的操作数同一套解码，`PARAM` 段读法不变。
- 真库门 `rvm_record_kinds_real` 加 aps7200 `23584/7314`：本体是一条 `RectangularTorus[100, 300, 400, 90°]`。
  aps7200 不在 APS 的 `/ALL` 里，E3D 只能在项目副本上导，设计库文件照样在 aps000。

## 2026-10-22 目录 LINE（TUFL 为真、两点）照 E3D 写 `Line` 记录——aps250132 / aps250160 26 只阀门的阀杆线补齐

- LINE 不出体，但过得了表现级（`TUFL` 为真）的那几条 E3D 照写一条 `Line[5, 长]`：aps250132 / aps250160 的
  26 只阀门，目录 `LINE`（`PTS 0 3` 一类）各一条，线段从 P0 到 P3、原点在中点。此前一律只记账，
  26 组记成种类不同（几何本来 exact）。
- 记录标架只看线段的世界方向：Y 取世界 −Z 在垂直于线段那张面上的投影，竖直时取世界 +Y，X = Y × Z。
  APS 普查 TTY 导出里 2504 条 `Line`（26 条阀杆线、2478 条 GENSEC 基准线）无一例外；与构件自己的
  X / Y 无关（26 只阀门里 16 只不同）。
- `TUFL` 假的 LINE（acp7000 里 3000 来条、ams 的阀门全是）照旧不写；多点线没有样本，照旧只记账。
  扣过负体的布尔结果与其他无体图元一样不另写。
- 读取集合多一个 `PTS`（160 → 161）。单测 `a_line_record_takes_its_frame_from_the_world_direction_alone`；
  真库门 `rvm_record_kinds_real` 加 `/V118`。

## 2026-10-22 风管链上的分流片 SPLR 不当管身端点——aps250142 `/SUPPLY_WORKSHOP-001` 多出的 25 mm 风管去掉

- 分流片 SPLR 装在风管里面，E3D 不拿它当管身端点，与风管链上的 ATTA 同路穿过。aps250142 的 19 只 SPLR 有
  18 只坐在两件的接缝上，当不当站点都一样；`/SUPPLY_WORKSHOP-001` 的 `2013286686/18` 伸进下一件 25 mm，
  当站点多画一根 25 mm 的矩形管，E3D 那里没有管。
- 零长构件不一概穿过：同一条支管的 BRCO `2013286686/56` 也是零长、也错开 144.7 mm，E3D 照样画了它到
  下一件的那一根。
- 单测 `a_splitter_on_a_duct_route_is_passed_through`；真库门 `rvm_record_kinds_real` 加这条支管：只出那一根
  144.7 mm 的 Box。

## 2026-10-22 设了坡度方向（SLODIR）的 P 点按实际方向判共轴——aps250124 放坡 TEE 两侧的管身补齐

- E3D 的坡度工具（E3D 2.1 `PMLLIB/design/objects/pipeSlopeComponent.pmlobj`：`PSLDIR $!componentRef.leave …`、
  `psldir 3 …`）给目录里带偏差容差的 P 点另设实际方向，存在设计构件的 `SLODIR` 里：第 n 个
  三元组是 Pn，全零是没设；P 点位置不动。管身两端判共轴看的是这个方向。
- aps250124 `/100-LAYOUT300-307/B1` 按 1:75 放坡：两只弯头转了坡度角，中间那只 TEE（`2013286668/683`）
  没转，`SLODIR` 把 P1 / P2 各压低 0.7639°。按目录方向判，TEE 两侧的槽位各偏 0.764°、不建；E3D
  两根都画了（1042.81、711.16 mm）。现在这条支管 10 根管身与 E3D 一致。
- 方向按属主系解（与 `POS` / `ORI` / `HDIR` 同口径）。全语料只有这一只 TEE 设了 `SLODIR`，它的
  `ORI` 为零，属主系与构件系分不出来。
- 读取集合多一个 `SLODIR`（159 → 160），gen-model 的两份产物一起重生成。真库门
  `rvm_record_kinds_real` 加 aps250124 这条支管。

## 2026-10-22 rvm_compare：E3D 写在容器那一组里的隐式管并成一件、按容器名配对

- APS 那批 TTY 普查（`aps_survey_lv6.mac`）与 ams8000 / `/SSC测试` 的导出都没开「隐式管导成单独容器」：E3D 把
  路由容器的隐式管直接写在容器自己那一组里，组名就是容器名；`gen_ams` 照单独容器的口径写
  `TUBI n of <容器>`。两边各成孤儿——E3D 那组发不出主键，我们的管身落进只在生成侧（TOTAL 不显示它）。
  aps250104 的 16 只 TRUNNI、aps250110 泵房 4 条 BRAN 就是这么被记成「我们多出隐式管」的，实际与 E3D
  一根不差。
- RVM ↔ RVM 对拍时，基准里有同名带几何的成员、生成侧又没有同名元素的容器，把它名下的管身并成一件
  （世界 AABB 取并、记录种类拼接、判档走世界帧）交给同名配对；开了单独容器的导出里容器组没有几何，
  不受影响。报告新增 `container_tube_groups`。
- gen-model 回归门 w7d：新配上的 256 只容器（APS 199、ams8000 整库 52、`/SSC测试` 5）251 exact、
  5 off。那 5 条支管的管身条数与 E3D 不同（我们多或少 1–4 根），此前藏在只在生成侧里，另行取证。

## 2026-10-22 布尔结果整只一条面片时不另写无体的目录图元——aps250104 23 只锚固件记录种类与 E3D 一致

- aps250104 的锚固件 ANCI（`/PS-01-00122.1` … `/PS-01-00147.1`，目录 `/BT23-100`）扣过三把负体，E3D 写
  一条布尔结果 FacetGroup；目录里 `PDIA = DESP[11] = 0`、长 10 的 LCYL 没有体，不在布尔结果里，E3D 也不
  另写。我们照「无体图元照写 DRCYL」多写一条 `Cylinder[0, 10]`，23 只记成种类不同。
- 没扣负体的件照旧写：zone 66457 的 12 只阀门（`/1WCC216VN` 等）零径 LCYL 写 `Cylinder[0, 120]`、
  aps250142 风管的零高圆柱，与 E3D 一致。对着回归门里全部带零尺寸圆柱、又配得上记录种类的 74 组：
  走图元的 51 组 E3D 都写了，布尔结果的 23 组 E3D 都没写。
- 真库门 `rvm_record_kinds_real` 加 aps250104：锚固件只写一条 FacetGroup、SUPC 名下不出 TUBI、
  TRUNNI `/PS-01-00111/B1` 照出 `Cylinder[44.45, 524.56]`。

## 2026-10-22 SUPC 不补隐式管身——派生管身的路由容器只剩 GTTUBE 认的 BRAN / TRUNNI / LUG

- `cachelib/GTTUBE` 硬编码的 noun 码只有 BRAN、TRUNNI、LUG 三个；SUPC 此前跟着「路由容器」一起进了
  `DERIVED_ROUTE_CONTAINER_NOUNS`，没有 E3D 依据。aps250104 的 TTY 导出（gen-model
  `verification/civil-tty-20261021/aps_survey_lv6_2013286648_2.rvm`）里 16 只 TRUNNI 都有管身，7 只 SUPC
  （`/PS-01-00117/B1`、`/PS-01-00122/B1` 等）一根都没有，我们多出 7 根。
- SUPC 与 HANG 同路：只下钻成员、按目录件建模，不派生管身；增量的派生腿随同一个谓词收口。
  已部署的库里旧的 SUPC 管身要靠一次整库重生成清掉。

## 2026-10-22 布局式扶手的 RAILHE 是栏杆顶高，踢脚板照 E3D 建——梯段上 10 只扶手全 exact

- E3D TTY 导出（aps250112 `/EQUIP_SUPPORT/SF_001..003`、aps250110 `/B01_LEVEL_03_ACCESS/SF_001`、`SF_002` 的梯段扶手）：
  平台段栏杆截面环中心在 `RAILHE − 20`、顶在 `RAILHE`；斜段端环垂直于轴、顶点正好在端点上方 `RAILHE` 处。
  即 Ø40 的截面圆挂在抬起的路径下面（圆心沿截面「上」向往下 20），两端平头、拐角斜接。此前轴线放在
  `RAILHE`，斜段两端靠水平切面把包围盒补齐，平台段就高出 20 mm（aps250112 6 只 off 20）。
- 踢脚板：板厚 4、离路径 10 起高 100（截面「上」向），拐角斜接、开口端竖切（此前 8 × 110、从路径起）。
- 相邻两节栏杆共用的斜接面在布尔并之后留下零体积的皮，每片单写一条记录；HANDRA 也走
  `drop_subresolution_films`，与 E3D 一样整只一条面片。
- aps250112：off 6 → 0、exact 761 → 767、种类不同 16 → 10（余 10 根带 `FRAD` 圆角的栏杆，E3D 逐段写）；
  aps250110：chord 1 → 0、exact 3427 → 3428。真库门 `aps_handrail_real` 加两只梯段扶手的包围盒与记录条数。

## 2026-10-22 详图式栏杆按 SZDI / EZDI 截段、扫过 CURTYP THRU 圆弧——aps250112 10 根、aps250110 2 根全出、exact

- `handrail::run_segments`：`STAREF` + `SZDI` → `ENDREF` + `EZDI` 两端与立柱 `ZDIS` 同一口径取站位，截口不切斜接面。
  aps250112 `/EL(+)104880_TOS*/HR_00x` 的 10 根 `STAREF == ENDREF`、`SZDI 450 → EZDI 2196.4` 之类，此前判成「取不出
  一段」。距离都是 0 的走原来那串边，结果不变。
- `CURTYP THRU` 的 `CURVE` 把那条边换成过三点的圆弧（`swap_in_through_arc`），两邻直段的斜接面改用弧的切向，
  弧两端的斜接面经 `DRNS` / `DRNE` 交给 `spine_swept_solid`。aps250110 `/BUILDING_01_LEVEL_03/HR_001`（闭合路径
  首段 R ≈ 14828 的弧）的 `RAIL 1` / `RAIL 2` 此前整根不画；拐点带 `FRAD` 的弧本期不认，照旧整根不画。
- 读取集合 +2（`SZDI` / `EZDI`，159）。aps250112：两侧都有 678 → 771、exact 564 → 761、只在 E3D 93 → 0
  （含上两条）；aps250110：两侧都有 3427 → 3429、exact 3425 → 3427、只在 E3D 2 → 0。

## 2026-10-22 目录构件式立柱 / 端头 HRPOST / HRTERM 走 SFIT 求值——aps250112 68 根立柱、15 只端头全出、exact

- `HRPOST` / `HRTERM` 归 `Catalog`，与 FIXING / FITT / TREAD 同一个目录构件求值器（`SPRE → GPART --CATR--> SFIT` 的
  GMRE；`RPRO` 经 DTSE 接设计侧的 `LENG` / `ZOFF` 与目录 `PARA`），标架用上一条的规则。GPART 有 CATR 的
  才自己成件（`category::is_model_unit_of`）；没有 CATR 的是模板件，几何在设计侧 TMPL 成员上，照旧由成员
  各自成件，也不记 `catalog_pending`。
- aps250112：`/Lionweld_Kennedy_Solid/...` 立柱 68 根、`.../Return` 回环 10 只、`.../Pads` 5 只此前一件不出，
  现在全出、exact。`Return` 回环起点 `LEFT`、末点 `RIGH`，两只都朝扶手里——「端头朝路径外」那种读法在这里就转反了。
  aps250110 的 130 只模板件不变。

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

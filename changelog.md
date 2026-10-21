# Changelog

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

# SigMap Query Context
Generated: 2026-10-07T18:46:37.697Z

## src/section.rs
```
# module: ! 型材截面（G1-1a 第 1 步）：把 SCTN / GENSEC 的目录截面求成一个二维环组。 ! ! 对标 Core3D getProfileFromDB@0x10410b30 → sub10410BB0（profileExtractFromDB）： ! ! 1. 截面元素从哪来：SCTN.SPRE → SPCO --CATR--> SPRF --GSTR--> GMSS（SPRE 也可以 ! 直接指到 SPRF，见 [sectioncatalogue]），GMSS 的成员是 ! 二维几何集元素 SPRO（任意截面，顶点 SPVE）/ SREC（矩形）/ SANN（圆环）。 ! 实测（examples/sectionprobe --census）：ams7997 1762 只 SCTN 里 1761 只走这条链 ! （SPRO 173 / SPRO+SREC 1554 /
pub struct SectionCatalogue  :96-102  # 一条走通的截面目录链。
pub struct SectionMember  :121-127  # GMSS 里一个过了显示筛、且真有形状的成员。
pub struct SectionProfile  :135-166  # 求好的二维截面：环在截面自己的平面里（x 右、y 上），单位 mm。
pub struct MemberFrame  :322-326  # 成员在 GMSS 平面里的标架：平移 `(PX, PY)` + `PLAX` 给的旋转（`pplib/QSPROF`）。
pub struct SectionPlacement  :679-687  # SCTN 在属主坐标系里的标架与长度（`dbboxlib/GTSTDT@0x1036CEC4` 里 `pllib/PIC
pub struct SectionCutPlane  :924-927  # 一端的切平面，坐标在 [`SectionPlacement::local`] 的局部系里。
pub struct SectionCuts  :950-955  # SCTN 两端的切平面。
pub struct SpinePath  :1107-1119  # GENSEC 的路径：`SPINE` 下的 `POINSP` 点串，点之间可夹 `CURVE` 弧。
pub struct SectionSolid  :1886-1896  # [`section_local_solid`] 的成品：可见实体，加上并集**之前**的那几块。
pub enum SectionError  :58-81
pub enum SectionChain  :85-92  # `SPRE` 链走到的地方。
pub enum SectionSource  :110-117  # 截面来自 GMSS 的哪种成员。
pub enum SpineSegment  :1039-1053  # 一段路径。坐标已折算到 GENSEC 自身的局部系。
impl SectionProfile  :168-173
  pub fn is_empty(&self) → bool  :170-170  # 没有任何成员出得了形状（全被筛掉、或只剩空占位 SPRO）：E3D 同样只画中心线。
impl MemberFrame  :328-337
  pub fn apply(&self, x: f64, y: f64) → [f64  :330-330  # 把成员局部 (x, y) 放进 GMSS 平面。
impl SectionCutPlane  :929-946
  pub fn is_axial(&self) → bool  :933-933  # 法向 `x = y = 0`：平面垂直于轴，与挤出体的端盖重合，切了是空操作。
```

## src\section.rs
```
pub struct SectionCatalogue  :96-102  # 一条走通的截面目录链。
pub struct SectionMember  :121-127  # GMSS 里一个过了显示筛、且真有形状的成员。
pub struct SectionProfile  :135-166  # 求好的二维截面：环在截面自己的平面里（x 右、y 上），单位 mm。
pub struct MemberFrame  :322-326  # 成员在 GMSS 平面里的标架：平移 `(PX, PY)` + `PLAX` 给的旋转（`pplib/QSPROF`）。
pub struct SectionPlacement  :679-687  # SCTN 在属主坐标系里的标架与长度（`dbboxlib/GTSTDT@0x1036CEC4` 里 `pllib/PIC
pub struct SectionCutPlane  :924-927  # 一端的切平面，坐标在 [`SectionPlacement::local`] 的局部系里。
pub struct SectionCuts  :950-955  # SCTN 两端的切平面。
pub struct SpinePath  :1107-1119  # GENSEC 的路径：`SPINE` 下的 `POINSP` 点串，点之间可夹 `CURVE` 弧。
pub struct SectionSolid  :1886-1896  # [`section_local_solid`] 的成品：可见实体，加上并集**之前**的那几块。
pub enum SectionError  :58-81
pub enum SectionChain  :85-92  # `SPRE` 链走到的地方。
pub enum SectionSource  :110-117  # 截面来自 GMSS 的哪种成员。
pub enum SpineSegment  :1039-1053  # 一段路径。坐标已折算到 GENSEC 自身的局部系。
impl SectionProfile  :168-173
pub fn is_empty(&self) → bool  :170-170  # 没有任何成员出得了形状（全被筛掉、或只剩空占位 SPRO）：E3D 同样只画中心线。
impl MemberFrame  :328-337
pub fn apply(&self, x: f64, y: f64) → [f64  :330-330  # 把成员局部 (x, y) 放进 GMSS 平面。
impl SectionCutPlane  :929-946
pub fn is_axial(&self) → bool  :933-933  # 法向 `x = y = 0`：平面垂直于轴，与挤出体的端盖重合，切了是空操作。
impl SectionCuts  :957-975
```

## src\datum.rs
```
pub struct DatumSolve  :31-53  # 解一个 GENSEC 名下 datum 用到的全部中间量；取证探针直接看这些。
impl DatumSolve  :55-83
pub fn section_rotation(&self) → DMat3  :57-57  # 站位的截面标架（不含 spin）：列 = (Ydir × 切向, Ydir, 切向)。
pub fn profile_rotation(&self) → DMat3  :63-63  # 截面平面（放置后）→ GENSEC 局部系的旋转：截面标架再绕 Z 转 spin。
pub fn placed_pline_offset(&self) → Option<[f64  :68-68  # p-line 在放置后截面平面里的位置：减对齐、镜像。
pub fn placed_pline_axis(&self) → Option<DVec3>  :78-78  # p-line 的 PLAX 方向（放置后截面平面里）。
pub fn is_gensec_datum(el: &DbElement) → Result<bool, ModelError>  :102-104  # 这只元素是不是 GENSEC 名下、标架要现算的 JLDATU / PLDATU（只看属主链，不求标架）。
pub fn solve(datum: &DbElement) → Result<Option<DatumSolve>, ...  :106-204
pub fn gensec_datum_world(datum: &DbElement) → Result<Option<DMat4>, Model...  :344-349  # JLDATU / PLDATU（宿主是 GENSEC）的世界标架；别的元素交 `None`。
pub fn datum_local(solve: &DatumSolve) → DMat4  :352-376  # datum 在宿主 GENSEC 局部系里的标架。
```

## src/datum.rs
```
# module: ! GENSEC 名下的隐式标架：JLDATU（沿 SPINE 的站位）与 PLDATU（站位上某条截面 p-line）。 ! ! 这两级在模板里都没有 POS/ORI，[crate::transform::localmatrix] 在它们身上给的是 ! 单位阵，于是挂在 GENSEC 上的 FIXING——连同它 TMPL 里的一切——全落在 GENSEC 原点 ! 附近。E3D 现算（pllib/GPLDOP@0x103ADC48 沿路径定站位，pllib/GPLDAT@0x103AD778 ! 定 p-line 标架）： ! ! - JLDATU：从 HPREF 指的 POINSP（未设 = SPINE 起点）沿路径走 ZDIS，落在两端之外 ! 时沿端切向直线外推。站位标架 = 该处的截面标架（X = Ydir × 切向，Y = 沿路径传递到 ! 此处的 SPINE Ydir，Z
pub struct DatumSolve  :31-53  # 解一个 GENSEC 名下 datum 用到的全部中间量；取证探针直接看这些。
impl DatumSolve  :55-83
  pub fn section_rotation(&self) → DMat3  :57-57  # 站位的截面标架（不含 spin）：列 = (Ydir × 切向, Ydir, 切向)。
  pub fn profile_rotation(&self) → DMat3  :63-63  # 截面平面（放置后）→ GENSEC 局部系的旋转：截面标架再绕 Z 转 spin。
  pub fn placed_pline_offset(&self) → Option<[f64  :68-68  # p-line 在放置后截面平面里的位置：减对齐、镜像。
  pub fn placed_pline_axis(&self) → Option<DVec3>  :78-78  # p-line 的 PLAX 方向（放置后截面平面里）。
pub fn is_gensec_datum(el: &DbElement) → Result<bool, ModelError>  :102-104  # 这只元素是不是 GENSEC 名下、标架要现算的 JLDATU / PLDATU（只看属主链，不求标架）。
pub fn solve(datum: &DbElement) → Result<Option<DatumSolve>, ...  :106-204
pub fn gensec_datum_world(datum: &DbElement) → Result<Option<DMat4>, Model...  :344-349  # JLDATU / PLDATU（宿主是 GENSEC）的世界标架；别的元素交 `None`。
pub fn datum_local(solve: &DatumSolve) → DMat4  :352-376  # datum 在宿主 GENSEC 局部系里的标架。
```

## evidence/20260902-g11a-section-profiles/README.md
```
h1 G1-1a 第 1 步：SCTN 截面求值（`src/section.rs` + `examples/section_probe.rs`）
h2 1. Core3D 口径（ida-bridge，`Core3D.dll` = `idalib-9944`）
h2 2. 库里的链形态（`section_probe --census`）
h2 3. 求值结果（`section_probe --evaluate`）
h2 4. 代码
h2 5. 验证
h2 6. 留给第 2 步
```

## evidence\20260902-g11a-section-profiles\README.md
```
h1 G1-1a 第 1 步：SCTN 截面求值（`src/section.rs` + `examples/section_probe.rs`）
h2 1. Core3D 口径（ida-bridge，`Core3D.dll` = `idalib-9944`）
h2 2. 库里的链形态（`section_probe --census`）
h2 3. 求值结果（`section_probe --evaluate`）
h2 4. 代码
h2 5. 验证
h2 6. 留给第 2 步
```

## evidence/20260831-043624-rvm-compare-local-frame/rvm-local-frame.patch
```
let Some(polygons) = &geometry.polygons_world_mm else {
let keep = |noun: &str| {
let key = (owner_key(desc), noun4(noun), ordinal);
let tally = by_noun.entry(key.1.clone()).or_default();
let Some(element) = ours.get(key) else {
let delta_min_mm = std::array::from_fn(|i| generated_aabb[i] - baseline_aabb[i]);
let delta_max_mm = std::array::from_fn(|i| generated_aabb[3 + i] - baseline_aabb[3 + i]);
let max_axis_delta_mm = delta_min_mm
let mesh = TriMesh::from_manifold(&generated.solid);
```

## src\rvm_read.rs
```
pub struct Polygon  :397-399
pub struct Contour  :402-406
pub struct RvmGeometry  :410-424  # 一条 `PRIM` / `OBST` / `INSU` 记录。
pub struct RvmMember  :478-490  # 一个 `CNTB` 组。
pub struct RvmHeader  :524-532
pub struct RvmFile  :535-539
pub enum BboxFrame  :55-66  # 记录 bbox 落在哪个帧（逐记录自证，见模块文档）。
pub enum Primitive  :70-89  # 图元种类与它的参数（mm）。`kind_id` 与 RVM 的编号一致。
impl Primitive  :91-385
pub fn kind_id(&self) → u32  :92-92
pub fn name(&self) → &'static str  :109-109
pub fn local_hull_points_mm(&self) → Vec<[f64  :134-134  # 包住这个图元的**采样点云**（局部 mm）。没有体的（`Line`）与不认识的返回空。
pub fn local_aabb_mm(&self) → Option<[f64  :326-326  # 规范局部盒（mm）。认不出形状或没有体积（`Line`）时 None。
pub fn declared_box_mm(&self) → Option<[f64  :369-369  # E3D **自己往记录里写 bbox 时用的那只盒**（局部 mm），只在它与真几何不同的形状上给。
impl RvmGeometry  :426-474
pub fn rotation(&self) → [f64  :428-428  # 3×3 的纯旋转部分（把 0
pub fn to_world_mm(&self, v: [f64; 3]) → [f64  :433-433  # 局部 mm → 世界 mm。
pub fn local_vertices_mm(&self) → Vec<[f64  :443-443  # 面片顶点（局部 mm）。非 FacetGroup 为空。
pub fn world_hull_points_mm(&self) → Vec<[f64  :460-460  # 够用来算 AABB 的**世界 mm** 点集：FacetGroup 给真顶点，解析图元给
pub fn world_aabb_mm(&self) → Option<[f64  :471-471  # 世界 AABB（mm），**从几何本身实算**，不碰 `record_bbox`。
```

## evidence/20260831-043624-rvm-compare-local-frame/rvm_compare.rs.original
```
fn owner_key(desc: &str) -> String {
fn noun4(noun: &str) -> String {
fn main() -> anyhow::Result<()> {
let Some(polygons) = &geometry.polygons_world_mm else {
let keep = |noun: &str| {
let facet_aabb = member
let key = (owner_key(desc), noun4(noun), ordinal);
let Some(desc) = &element.owner_desc else {
let key = (owner_key(desc), noun4(&element.noun), element.ordinal);
let tally = by_noun.entry(key.1.clone()).or_default();
let Some(element) = ours.get(key) else {
let generated_aabb = [
let delta_min_mm = std::array::from_fn(|i| generated_aabb[i] - baseline_aabb[i]);
let delta_max_mm = std::array::from_fn(|i| generated_aabb[3 + i] - baseline_aabb[3 + i]);
let max_axis_delta_mm = delta_min_mm
```

## src/rvm_read.rs
```
# module: ! AVEVA RVM 二进制读取——基准侧与生成侧共用的那一份读法。 ! ! 存在的理由是对称：rvmcompare 今天一边吃 .rvm.json 快照（rvm-rs 解出来的）、 ! 一边吃 elements.json（genams 写的），两条路径的帧与单位各算各的。把两边都换成 ! 这里的 [RvmFile]，「基准 AABB 怎么算」就只剩一处实现，加判据也只改一处。 ! 写出端（rvmwrite）复用同一套结构，读写共享一份格式定义。 ! ! 不用 rvm-rs 的三个理由：① 它在本 crate 是 dev-dependencies，src/ 里用不了； ! ② 它对每条记录一律 bbox.transform(&transform)，对下面说的 rotated 帧会多转一次 ! （examples/rvmfacetsdump.rs 为此绕了
pub struct Polygon  :397-399
pub struct Contour  :402-406
pub struct RvmGeometry  :410-424  # 一条 `PRIM` / `OBST` / `INSU` 记录。
pub struct RvmMember  :478-490  # 一个 `CNTB` 组。
pub struct RvmHeader  :524-532
pub struct RvmFile  :535-539
pub enum BboxFrame  :55-66  # 记录 bbox 落在哪个帧（逐记录自证，见模块文档）。
pub enum Primitive  :70-89  # 图元种类与它的参数（mm）。`kind_id` 与 RVM 的编号一致。
impl Primitive  :91-385
  pub fn kind_id(&self) → u32  :92-92
  pub fn name(&self) → &'static str  :109-109
  pub fn local_hull_points_mm(&self) → Vec<[f64  :134-134  # 包住这个图元的**采样点云**（局部 mm）。没有体的（`Line`）与不认识的返回空。
  pub fn local_aabb_mm(&self) → Option<[f64  :326-326  # 规范局部盒（mm）。认不出形状或没有体积（`Line`）时 None。
  pub fn declared_box_mm(&self) → Option<[f64  :369-369  # E3D **自己往记录里写 bbox 时用的那只盒**（局部 mm），只在它与真几何不同的形状上给。
impl RvmGeometry  :426-474
  pub fn rotation(&self) → [f64  :428-428  # 3×3 的纯旋转部分（把 0
  pub fn to_world_mm(&self, v: [f64; 3]) → [f64  :433-433  # 局部 mm → 世界 mm。
  pub fn local_vertices_mm(&self) → Vec<[f64  :443-443  # 面片顶点（局部 mm）。非 FacetGroup 为空。
  pub fn world_hull_points_mm(&self) → Vec<[f64  :460-460  # 够用来算 AABB 的**世界 mm** 点集：FacetGroup 给真顶点，解析图元给
```

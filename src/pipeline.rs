//! 设计树遍历与总账（对标 `add/ADDDES` 的 DFS + `Core3D` 的批任务收口）。
//!
//! 遍历规则（成员一律文件原序）：
//!
//! | 类别 | 动作 |
//! |---|---|
//! | `List` | 无 |
//! | `LoopExtrusion` / `Primitive` / `Revolution` / `Polyhedron` | [`model_element`] |
//! | `Catalog` / `Unsupported` / `Unknown` | 记账 |
//! | `RouteContainer` | 记账，并按 `RouteWalker` 的串接方式数出隐式管身 |
//! | `RouteMember` | 记账 |
//! | `NonGraphic` | 记账（判「无三维表示」的放行要留名，见 [`Report::non_graphic`]） |
//! | `Neg*` / `ProfileData` | 属主消费掉了就不会走到这里；走到了就是孤儿，记账 |
//!
//! **下钻是无条件的**，只有一个例外：被本元素消费掉的成员（自己的剖面环与负体）
//! 不再入栈，改为整棵子树计入 [`Report::consumed`]。
//!
//! 「本元素不建模」**不等于**「它下面没有能建模的东西」——这条曾经反过来写过一次：
//! `Catalog` / `Unsupported` / `NonGraphic` 都不下钻，而 ams1112 的 4275 个 PANE
//! 全挂在被判成目录件的 CWALL 与未实现项的 CFLOOR 底下，于是全库 30940 个元素
//! 只走到 160 个，总账上却只有一行「catalog: CWALL 121」，看不出丢了一整片船体。
//!
//! 账本原则（gen-model 宪法同款）：静默缺件是最高级别缺陷。
//! [`Report::accounts_for`] 就是这条的硬判据——`visited + consumed` 必须等于
//! 子树的元素总数，一个都不许蒸发。

use std::collections::BTreeMap;

use e3d_io::db_element::{DbElement, DbElementError};

use crate::catalogue_point::CataloguePoints;
use crate::category::{BuildDisposition, Category, decision, is_model_unit};
use crate::elmodl::{GeneratedElement, GeometryId, Modeled, model_element};
use crate::route::{
    PPointResolver, Route, TubeOutcome, bore_mismatch, build_tube, resolve_end,
    tube_outer_diameter, walk_route,
};

/// 一条有名有姓的记录。
#[derive(Debug, Clone, serde::Serialize)]
pub struct Incident {
    pub refno: String,
    pub noun: String,
    pub detail: String,
}

/// 隐式管身账（算法文档 §2.3 的字段口径）。
///
/// 管身是运行时派生件：库里没有 TUBI 记录，它**既不是 `visited` 也不是 `consumed`**。
/// 塞进那两本账会让 [`Report::accounts_for`] 失真——那是全 crate 唯一的硬判据，
/// 让它变松比少一本账严重得多。所以单开这一本，与主账互不相干。
#[derive(Debug, Default, serde::Serialize)]
pub struct ImpliedTubes {
    /// 管身条数，按 `RouteWalker` 的串接方式数出来：每个路由成员前面一条，
    /// 末尾到容器 `TPOS` 再补一条，即 `路由成员数 + 1`。
    pub count: usize,
    /// 已实际生成圆柱实体的管身数。
    pub built: usize,
    /// 管身总长（mm）。**只有 `measured == count` 时才是 `Some`**——量了一部分就
    /// 报总长，写出来的数看着跟真的一样，却少了没量到的那些。宁可给 `None`。
    pub total_length: Option<f64>,
    /// 已经量出长度的槽位数。`count - measured` 就是还卡在
    /// [`ImpliedTubeDegenerate`] 各项上的条数。
    pub measured: usize,
    /// 已量出的那部分长度之和（mm）。名字不叫 `total_*`，因为在
    /// `measured == count` 之前它只是个部分和。
    pub measured_length_mm: f64,
    /// 按属主容器 noun 分账：BRAN 一支和 HANG 一支的管身要能分开看。
    pub by_owner_noun: BTreeMap<String, usize>,
    /// 退化情形明细，P2 出几何时才填得上。
    pub degenerate: ImpliedTubeDegenerate,
}

impl ImpliedTubes {
    /// 全部槽位都量过了才给总长，否则留 `None`。
    ///
    /// 分成一步而不是边量边写，是因为「量全了没有」只有走完整棵子树才知道；
    /// 中途写进去的部分和一旦被下游读走，就再也没有地方能看出它是残的。
    pub fn settle_total_length(&mut self) {
        self.total_length = (self.measured == self.count).then_some(self.measured_length_mm);
    }
}

/// 建不出几何的管身分类计数（算法文档 §4 的退化表）。
#[derive(Debug, Default, serde::Serialize)]
pub struct ImpliedTubeDegenerate {
    /// 两端点重合（`length <` [`crate::route::ZERO_LENGTH_TOL_MM`]）。
    pub zero_length: usize,
    /// P 点取不到（`GTTUBE` 的 ok 标志为 0）：链走通了但目录里没这个 P 点，
    /// 或轴向/距离表达式求不出值。
    pub missing_ppoint: usize,
    /// P 点所在的库根本没打开——`SPRE`/`CATR` 指向未注册的 dbno。
    ///
    /// 与 `missing_ppoint` 分开记：那条是数据缺，这条是**跑的时候少给了库**，
    /// 补个 `--catalogue` 就能变成真管。混在一起会把配置问题读成数据问题。
    pub ppoint_db_not_open: usize,
    /// 未打开目录库按 dbno 分账，供启动器补齐 `--catalogue`。
    pub ppoint_db_not_open_by_dbno: BTreeMap<String, usize>,
    /// 库已打开但引用的历史目录元素不在所钉会话中。
    pub ppoint_target_not_found: usize,
    pub ppoint_target_not_found_by_dbno: BTreeMap<String, usize>,
    /// 两端通径不一致，按到达侧取值照常建，但要留痕。
    pub bore_mismatch: usize,
    /// 两端都解出来了，却定不出朝向或取不到半径。
    ///
    /// 与 `missing_ppoint` 分开：那条是「还没接上」，这条是「接上了但数据坏」，
    /// 混在一起会让 P1 的进度看起来比实际好。
    pub unbuildable: usize,
}

/// 生成总账（可直接 serde_json 落盘）。
#[derive(Debug, Default, serde::Serialize)]
pub struct Report {
    /// 遍历到的元素总数。
    pub visited: usize,
    /// 建成实体数。
    pub generated: usize,
    /// 正体被有依据跳过（零高度 / 里程碑边界）。
    pub skipped: Vec<Incident>,
    /// 数据坏建不出来（响亮账）。
    pub failed: Vec<Incident>,
    /// 目录件记账：noun → 数量（二期 G4）。
    pub catalog_pending: BTreeMap<String, usize>,
    /// 路由容器记账：noun → 数量。BRAN/HANG/LUG/SUPC/TRUNNI 五族。
    pub route_containers: BTreeMap<String, usize>,
    /// 路由成员记账：noun → 数量。目录驱动，几何压在 P1。
    ///
    /// 与 [`Report::catalog_pending`] 分开记：目录件是型材与墙族那一路，路由成员
    /// 另有隐式管身要配对，两本混在一起就看不出管路到底缺了多少件。
    pub route_members: BTreeMap<String, usize>,
    /// 隐式管身账，见 [`ImpliedTubes`]。
    pub implied_tubes: ImpliedTubes,
    /// 几何集正体记账：noun → 数量。[`Category::GeometrySet`] 那 26 个 noun。
    ///
    /// 与 [`Report::unsupported`] 分开：几何集是 core.dll 里**另一条建模路**，
    /// 不是「实现没跟上」而是建模概念缺失。混进 unsupported 会让「本期不做的
    /// 已知项」和「压根没想过的东西」同账。
    pub geometry_sets: BTreeMap<String, usize>,
    /// 几何集负体记账：noun → 数量。[`Category::NegGeometrySet`] 那 16 个 INGCOM noun。
    ///
    /// 不并进 [`Report::orphans`]：孤儿那本账的语义是「数据或分类有问题」，而一个
    /// `NSBO` 挂在几何集属主底下是完全正常的数据——只是几何集这条路本期整条不建，
    /// 属主自己都在 [`Report::geometry_sets`] 里记着。混进孤儿账会让它天天有值，
    /// 真正的孤儿（挂错属主的 PLOO）就淹在里面了。
    pub geometry_set_negatives: BTreeMap<String, usize>,
    /// 认识但本期不做：noun → 数量。
    pub unsupported: BTreeMap<String, usize>,
    /// 分类表没认领：noun → 数量。出现即分类表欠账。
    pub unknown_nouns: BTreeMap<String, usize>,
    /// 判「无三维表示」而放过的：noun → 数量。[`Category::NonGraphic`]。
    ///
    /// 这本账立出来之前，`NonGraphic` 与 [`Category::List`] 共用一个空分支，元素只进
    /// `visited` 总数、按 noun 一笔不记——于是「判它无几何」这个结论**在实跑里无法证伪**：
    /// 报告上看不出哪些 noun 是以这个理由被放过的，更看不出放过了多少件。而
    /// `NOZZ`/`ELCONN` 恰恰被 `GTGEOM` 硬编码送进复合几何路，判据方向是反的。
    ///
    /// 记了账才谈得上复核：先让每一次「无三维表示」的放行都在报告里留名。
    pub proven_non_graphic: BTreeMap<String, usize>,
    /// 已有明确建模路径、但 builder 或目标语料证据尚未闭环：noun → 数量。
    pub evidence_pending: BTreeMap<String, usize>,
    /// 已实际完成成员聚合并产出实体的复合构件：noun → 数量。
    pub composites_built: BTreeMap<String, usize>,
    /// 每个实际出现 NOUN 的分类依据；同一 NOUN 只保留稳定的一条依据。
    pub noun_evidence: BTreeMap<String, String>,
    /// 被属主消费掉、因而没单独遍历的子树：**被剪掉的那个成员的 noun** → 整棵子树的
    /// 元素数。它与 [`Report::visited`] 相加必须等于子树总元素数，见
    /// [`Report::accounts_for`]。
    pub consumed: BTreeMap<String, usize>,
    /// 没有属主消费的剖面环 / 负几何：noun → 数量。
    /// 出现即数据或分类有问题——一个 PLOO 挂在 ZONE 底下，它的 PAVE 就没人读。
    pub orphans: BTreeMap<String, usize>,
    /// 负几何被跳过的明细（正体照出，体积会偏大，对拍时按这本账归因）。
    pub negatives_skipped: Vec<Incident>,
    /// 非致命观察（倒圆越界、SJUS 未生效、孔环退化等）。
    pub notes: Vec<Incident>,
}

/// 一次生成的完整产出。
pub struct GenerationOutcome {
    pub elements: Vec<GeneratedElement>,
    pub report: Report,
}

/// 从 `root` 起 DFS 生成整棵子树。遍历中的 I/O 错误（读不到成员表）直接上抛——
/// 那是库文件层面的问题，不是单元素账。
pub fn generate_subtree(root: DbElement) -> Result<GenerationOutcome, DbElementError> {
    let mut elements = Vec::new();
    let mut report = Report::default();
    // 显式栈，避免深树递归爆栈；压栈顺序反转以保持文件原序遍历。
    let mut stack: Vec<DbElement> = vec![root];

    while let Some(el) = stack.pop() {
        report.visited += 1;
        let noun = el.element_type()?;
        let noun_decision = decision(&noun);
        let category = noun_decision.category;
        let refno = el.refno().to_string();
        report
            .noun_evidence
            .entry(noun.clone())
            .or_insert_with(|| noun_decision.evidence.to_owned());
        if noun_decision.disposition == BuildDisposition::EvidencePending {
            *report.evidence_pending.entry(noun.clone()).or_insert(0) += 1;
        }

        match category {
            Category::List => {}
            Category::NonGraphic => {
                *report.proven_non_graphic.entry(noun.clone()).or_insert(0) += 1;
            }
            Category::LoopExtrusion
            | Category::Primitive(_)
            | Category::Revolution
            | Category::Polyhedron => match model_element(&el, category) {
                Ok(Modeled::Solid(generated)) => {
                    for skip in &generated.negatives_skipped {
                        report.negatives_skipped.push(Incident {
                            refno: skip.refno.to_string(),
                            noun: skip.noun.clone(),
                            detail: format!("属主 {refno}（{noun}）: {}", skip.reason),
                        });
                    }
                    for note in &generated.notes {
                        report.notes.push(Incident {
                            refno: refno.clone(),
                            noun: noun.clone(),
                            detail: note.clone(),
                        });
                    }
                    report.generated += 1;
                    elements.push(*generated);
                }
                Ok(Modeled::Skipped { reason }) => report.skipped.push(Incident {
                    refno,
                    noun,
                    detail: reason,
                }),
                Err(error) => report.failed.push(Incident {
                    refno,
                    noun,
                    detail: error.to_string(),
                }),
            },
            // 目录件里唯独 SBFI 出实体。判据取自 `category::is_model_unit`，增量的
            // 上卷读的是同一个函数——这份名单一旦两处各写一份，被删的 SBFI 就会在
            // 增量里永远上卷不到单元、永远不出 Remove，而两边的账各自还都是平的。
            //
            // 「DESP 够不够长」那道门也曾写在这里，同样只写了这一份：增量直接调
            // `model_element`，短 DESP 在那边成了 `regen_failed`，在这边却是没这件事。
            // 现在门在 `elmodl` 里，短 DESP 两边统一落 `Skipped`。
            Category::Catalog if is_model_unit(&noun) => match model_element(&el, category) {
                Ok(Modeled::Solid(generated)) => {
                    for note in &generated.notes {
                        report.notes.push(Incident {
                            refno: refno.clone(),
                            noun: noun.clone(),
                            detail: note.clone(),
                        });
                    }
                    report.generated += 1;
                    decrement(&mut report.evidence_pending, &noun);
                    elements.push(*generated);
                }
                Ok(Modeled::Skipped { reason }) => report.skipped.push(Incident {
                    refno,
                    noun,
                    detail: reason,
                }),
                Err(error) => report.failed.push(Incident {
                    refno,
                    noun,
                    detail: error.to_string(),
                }),
            },
            Category::Catalog => {
                *report.catalog_pending.entry(noun.clone()).or_insert(0) += 1;
            }
            Category::RouteContainer => {
                *report.route_containers.entry(noun.clone()).or_insert(0) += 1;
                let route = walk_route(&el)?;
                let tubes = route.tubes.len();
                report.implied_tubes.count += tubes;
                *report
                    .implied_tubes
                    .by_owner_noun
                    .entry(noun.clone())
                    .or_insert(0) += tubes;
                let built = build_route_tubes(
                    &el,
                    &route,
                    &CataloguePoints,
                    &mut report.implied_tubes,
                    &mut elements,
                )?;
                report.generated += built;
            }
            Category::RouteMember => {
                *report.route_members.entry(noun.clone()).or_insert(0) += 1;
            }
            Category::Composite(_) => {
                // 类 28 与其余复合族必须经真实成员聚合 builder 才能转入 composites_built。
                // 在此之前只进独立的 evidence_pending 账，绝不把“已分类”冒充“已建”。
            }
            Category::GeometrySet => {
                *report.geometry_sets.entry(noun.clone()).or_insert(0) += 1;
            }
            Category::NegGeometrySet => {
                *report
                    .geometry_set_negatives
                    .entry(noun.clone())
                    .or_insert(0) += 1;
            }
            Category::Unsupported(_) => {
                *report.unsupported.entry(noun.clone()).or_insert(0) += 1;
            }
            Category::Unknown => {
                *report.unknown_nouns.entry(noun.clone()).or_insert(0) += 1;
            }
            // 属主消费掉的剖面与负体根本不入栈，能走到这里的都没人认领。
            Category::ProfileData
            | Category::NegExtrusion
            | Category::NegRevolution
            | Category::NegPolyhedron
            | Category::NegPrimitive(_)
            | Category::NegUnimplemented(_) => {
                *report.orphans.entry(noun.clone()).or_insert(0) += 1;
            }
        }

        push_members(&el, category, &mut stack, &mut report)?;
    }

    report.implied_tubes.settle_total_length();
    Ok(GenerationOutcome { elements, report })
}

fn decrement(ledger: &mut BTreeMap<String, usize>, noun: &str) {
    if let Some(count) = ledger.get_mut(noun) {
        *count -= 1;
        if *count == 0 {
            ledger.remove(noun);
        }
    }
}

/// 量一个容器上的全部管身槽位，逐类记账（算法文档 §4 的退化表）。
///
/// 这里**只量不建**：`GeneratedElement` 是按 refno 索引的，而管身没有库记录，
/// 塞进去要先定死伪 refno 的形状，那会连带动到增量更新与 RVM 对拍的主键。
/// 那两处一动就牵到增量更新与 RVM 对拍，等长度先在真库上对平了再改建几何。
fn build_route_tubes(
    container: &DbElement,
    route: &Route,
    resolver: &dyn PPointResolver,
    ledger: &mut ImpliedTubes,
    elements: &mut Vec<GeneratedElement>,
) -> Result<usize, DbElementError> {
    let mut built_count = 0usize;
    for (ordinal, slot) in route.tubes.iter().enumerate() {
        let ends = resolve_end(&slot.from, resolver)
            .and_then(|from| Ok((from, resolve_end(&slot.to, resolver)?)));
        let (Some(from), Some(to)) = (match ends {
            Ok(ends) => ends,
            // 库没开是配置问题，不是这棵子树坏了：记一笔接着量下一根。
            Err(DbElementError::DbNotRegistered { dbno }) => {
                ledger.degenerate.ppoint_db_not_open += 1;
                *ledger
                    .degenerate
                    .ppoint_db_not_open_by_dbno
                    .entry(dbno.to_string())
                    .or_insert(0) += 1;
                continue;
            }
            Err(DbElementError::NotFound { dbno, .. }) => {
                ledger.degenerate.missing_ppoint += 1;
                ledger.degenerate.ppoint_target_not_found += 1;
                *ledger
                    .degenerate
                    .ppoint_target_not_found_by_dbno
                    .entry(dbno.to_string())
                    .or_insert(0) += 1;
                continue;
            }
            Err(error) => return Err(error),
        }) else {
            ledger.degenerate.missing_ppoint += 1;
            continue;
        };
        if bore_mismatch(&from, &to) {
            ledger.degenerate.bore_mismatch += 1;
        }
        // 目录外径要等 P1 的目录求值，现在只有回落的到达侧通径这一路。
        let diameter = tube_outer_diameter(None, None, to.bore);
        match build_tube(&from, &to, diameter) {
            TubeOutcome::Built(tube) => {
                ledger.measured += 1;
                ledger.measured_length_mm += tube.length_mm;
                ledger.built += 1;
                built_count += 1;
                let preceding_refno = match &slot.from {
                    crate::route::RouteEnd::ContainerHead(element)
                    | crate::route::RouteEnd::ContainerTail(element) => element.refno().to_string(),
                    crate::route::RouteEnd::MemberLeave { member, .. }
                    | crate::route::RouteEnd::MemberArrive { member, .. } => {
                        member.refno().to_string()
                    }
                };
                elements.push(GeneratedElement {
                    geometry_id: GeometryId::ImpliedTube {
                        container_refno: container.refno().to_string(),
                        preceding_refno,
                        ordinal: u32::try_from(ordinal).unwrap_or(u32::MAX),
                    },
                    // 管身没有库记录；来源元素固定保存容器，绝不制造伪 refno。
                    refno: container.refno(),
                    noun: "TUBI".into(),
                    name: None,
                    solid: tube.solid,
                    world: tube.placement,
                    primitive_instance: None,
                    negatives_applied: 0,
                    negatives_skipped: Vec::new(),
                    notes: vec![format!(
                        "implied tube length={}mm radius={}mm",
                        tube.length_mm, tube.radius_mm
                    )],
                });
            }
            TubeOutcome::ZeroLength { .. } => ledger.degenerate.zero_length += 1,
            TubeOutcome::NoDirection | TubeOutcome::NoRadius | TubeOutcome::Failed(_) => {
                ledger.degenerate.unbuildable += 1
            }
        }
    }
    Ok(built_count)
}

/// 成员入栈。被本元素消费掉的（自己的剖面环与负体）不入栈，整棵子树计进
/// [`Report::consumed`]——不计的话它们就从账上蒸发了。
fn push_members(
    el: &DbElement,
    category: Category,
    stack: &mut Vec<DbElement>,
    report: &mut Report,
) -> Result<(), DbElementError> {
    let consumes = category.consumes_members();
    let mut batch = Vec::new();
    for member in el.members()? {
        let noun = member.element_type()?;
        let member_category = decision(&noun).category;
        let consumed = consumes
            && (matches!(member_category, Category::ProfileData) || member_category.is_negative());
        if consumed {
            *report.consumed.entry(noun).or_insert(0) += subtree_size(&member)?;
        } else {
            batch.push(member);
        }
    }
    // 栈是后进先出：反着压，弹出来才是文件原序。
    stack.extend(batch.into_iter().rev());
    Ok(())
}

/// 一棵子树的元素数（含自身）。只用来记账，不读属性。
fn subtree_size(root: &DbElement) -> Result<usize, DbElementError> {
    let mut count = 0usize;
    let mut stack = vec![root.clone()];
    while let Some(el) = stack.pop() {
        count += 1;
        stack.extend(el.members()?);
    }
    Ok(count)
}

impl Report {
    /// 并进另一棵子树的账（多根库逐根生成时用）。
    ///
    /// 放在结构体自己身上而不是调用方：漏并一本账就等于凭空少掉一批元素，
    /// 而调用方那边看不出来——`consumed` 刚加上时就是漏在 `gen_ams` 里，
    /// 靠 [`Report::accounts_for`] 才逮住。
    pub fn merge(&mut self, other: Report) {
        let Report {
            visited,
            generated,
            skipped,
            failed,
            catalog_pending,
            route_containers,
            route_members,
            implied_tubes,
            geometry_sets,
            geometry_set_negatives,
            unsupported,
            unknown_nouns,
            proven_non_graphic,
            evidence_pending,
            composites_built,
            noun_evidence,
            consumed,
            orphans,
            negatives_skipped,
            notes,
        } = other;
        self.visited += visited;
        self.generated += generated;
        self.skipped.extend(skipped);
        self.failed.extend(failed);
        self.negatives_skipped.extend(negatives_skipped);
        self.notes.extend(notes);
        self.implied_tubes.count += implied_tubes.count;
        self.implied_tubes.built += implied_tubes.built;
        self.implied_tubes.measured += implied_tubes.measured;
        self.implied_tubes.measured_length_mm += implied_tubes.measured_length_mm;
        self.implied_tubes.total_length =
            match (self.implied_tubes.total_length, implied_tubes.total_length) {
                // 一边还没量过就整体算没量过：把 None 当 0 加进去会得到一个偏小的
                // 「总长」，而它看起来跟真数一模一样。
                (Some(a), Some(b)) => Some(a + b),
                _ => None,
            };
        let degenerate = &mut self.implied_tubes.degenerate;
        degenerate.zero_length += implied_tubes.degenerate.zero_length;
        degenerate.missing_ppoint += implied_tubes.degenerate.missing_ppoint;
        degenerate.ppoint_db_not_open += implied_tubes.degenerate.ppoint_db_not_open;
        degenerate.bore_mismatch += implied_tubes.degenerate.bore_mismatch;
        degenerate.unbuildable += implied_tubes.degenerate.unbuildable;
        degenerate.ppoint_target_not_found += implied_tubes.degenerate.ppoint_target_not_found;
        for (dbno, count) in implied_tubes.degenerate.ppoint_db_not_open_by_dbno {
            *degenerate
                .ppoint_db_not_open_by_dbno
                .entry(dbno)
                .or_insert(0) += count;
        }
        for (dbno, count) in implied_tubes.degenerate.ppoint_target_not_found_by_dbno {
            *degenerate
                .ppoint_target_not_found_by_dbno
                .entry(dbno)
                .or_insert(0) += count;
        }
        for (target, source) in [
            (&mut self.catalog_pending, catalog_pending),
            (&mut self.route_containers, route_containers),
            (&mut self.route_members, route_members),
            (
                &mut self.implied_tubes.by_owner_noun,
                implied_tubes.by_owner_noun,
            ),
            (&mut self.geometry_sets, geometry_sets),
            (&mut self.geometry_set_negatives, geometry_set_negatives),
            (&mut self.unsupported, unsupported),
            (&mut self.unknown_nouns, unknown_nouns),
            (&mut self.proven_non_graphic, proven_non_graphic),
            (&mut self.evidence_pending, evidence_pending),
            (&mut self.composites_built, composites_built),
            (&mut self.consumed, consumed),
            (&mut self.orphans, orphans),
        ] {
            for (noun, count) in source {
                *target.entry(noun).or_insert(0) += count;
            }
        }
        for (noun, evidence) in noun_evidence {
            self.noun_evidence.entry(noun).or_insert(evidence);
        }
    }

    /// 被属主消费掉、没单独遍历的元素总数。
    pub fn consumed_total(&self) -> usize {
        self.consumed.values().sum()
    }

    /// 遍历有没有漏人：`visited + consumed` 应当等于子树的元素总数。
    ///
    /// `tree_total` 从索引侧独立数出来（`scan_elements`），两边同源就验不出东西。
    /// 差值为正 = 有子树被静默跳过了，那正是「CWALL 不下钻」那次事故的形状：
    /// 报告全绿，30940 个元素只走到 160 个。
    pub fn accounts_for(&self, tree_total: usize) -> Result<(), String> {
        let seen = self.visited + self.consumed_total();
        if seen == tree_total {
            return Ok(());
        }
        Err(format!(
            "账对不上：visited {} + consumed {} = {seen}，索引侧 {tree_total}，差 {}",
            self.visited,
            self.consumed_total(),
            tree_total as i64 - seen as i64,
        ))
    }

    /// 隐式管身总数（派生件，不进 [`Report::accounts_for`] 的等式）。
    pub fn implied_tube_total(&self) -> usize {
        self.implied_tubes.count
    }

    /// 各本账 + 生成数是否把 visited 解释干净。对拍前先跑这个自检，
    /// 账对不上就别谈几何。
    pub fn totals_line(&self) -> String {
        format!(
            "visited={} consumed={} generated={} skipped={} failed={} catalog={} \
             route_containers={} route_members={} implied_tubes={} tubes_measured={} \
             tubes_built={} tubes_no_ppoint={} unsupported={} unknown={} orphans={} neg_skipped={}",
            self.visited,
            self.consumed_total(),
            self.generated,
            self.skipped.len(),
            self.failed.len(),
            self.catalog_pending.values().sum::<usize>(),
            self.route_containers.values().sum::<usize>(),
            self.route_members.values().sum::<usize>(),
            self.implied_tubes.count,
            self.implied_tubes.measured,
            self.implied_tubes.built,
            self.implied_tubes.degenerate.missing_ppoint,
            self.unsupported.values().sum::<usize>(),
            self.unknown_nouns.values().sum::<usize>(),
            self.orphans.values().sum::<usize>(),
            self.negatives_skipped.len(),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 隐式管身是派生件，不许挤进 `visited + consumed` 那道等式。
    ///
    /// 把管身算成 visited 会让 `accounts_for` 变成「索引数 + 管身数」才平——判据一松，
    /// 真丢了子树也照样过。这是这本账必须独立的全部理由。
    #[test]
    fn implied_tubes_stay_out_of_the_balance_equation() {
        let mut report = Report {
            visited: 10,
            ..Default::default()
        };
        report.consumed.insert("PLOO".into(), 4);
        assert!(report.accounts_for(14).is_ok());

        report.implied_tubes.count = 561;
        report
            .implied_tubes
            .by_owner_noun
            .insert("BRAN".into(), 561);
        assert!(
            report.accounts_for(14).is_ok(),
            "管身进账后 14 这个总数不该改变"
        );
        assert_eq!(report.implied_tube_total(), 561);
    }

    /// 新开的三本账都要并进 `merge`，漏一本就等于凭空少掉一批元素。
    ///
    /// 这条是回归钉：`consumed` 当年就是漏在 `gen_ams` 的多根合并里，报告看着正常，
    /// 靠 `accounts_for` 才逮住——而路由三本账没有那样的等式兜底，只能靠这条测试。
    #[test]
    fn merge_carries_every_route_ledger() {
        let mut left = Report::default();
        left.route_containers.insert("BRAN".into(), 2);
        left.route_members.insert("ELBO".into(), 3);
        left.implied_tubes.count = 5;
        left.implied_tubes.built = 3;
        left.implied_tubes.by_owner_noun.insert("BRAN".into(), 5);
        left.implied_tubes.degenerate.zero_length = 1;
        left.proven_non_graphic.insert("TEXT".into(), 2);
        left.evidence_pending.insert("NOZZ".into(), 2);
        left.composites_built.insert("NOZZ".into(), 1);
        left.noun_evidence.insert("NOZZ".into(), "ida".into());

        let mut right = Report::default();
        right.route_containers.insert("HANG".into(), 1);
        right.route_members.insert("ELBO".into(), 4);
        right.implied_tubes.count = 2;
        right.implied_tubes.built = 2;
        right.implied_tubes.by_owner_noun.insert("HANG".into(), 2);
        right.implied_tubes.degenerate.bore_mismatch = 3;
        right.implied_tubes.degenerate.ppoint_db_not_open = 4;
        right
            .implied_tubes
            .degenerate
            .ppoint_db_not_open_by_dbno
            .insert("5053".into(), 4);
        right.proven_non_graphic.insert("TEXT".into(), 5);
        right.evidence_pending.insert("NOZZ".into(), 3);
        right.composites_built.insert("NOZZ".into(), 4);
        right.noun_evidence.insert("NOZZ".into(), "newer".into());

        left.merge(right);

        assert_eq!(left.route_containers["BRAN"], 2);
        assert_eq!(left.route_containers["HANG"], 1);
        assert_eq!(left.route_members["ELBO"], 7, "同 noun 要相加而不是覆盖");
        assert_eq!(left.implied_tubes.count, 7);
        assert_eq!(left.implied_tubes.built, 5);
        assert_eq!(left.implied_tubes.by_owner_noun["BRAN"], 5);
        assert_eq!(left.implied_tubes.by_owner_noun["HANG"], 2);
        assert_eq!(left.implied_tubes.degenerate.zero_length, 1);
        assert_eq!(left.implied_tubes.degenerate.bore_mismatch, 3);
        assert_eq!(left.implied_tubes.degenerate.ppoint_db_not_open, 4);
        assert_eq!(
            left.implied_tubes.degenerate.ppoint_db_not_open_by_dbno["5053"],
            4
        );
        assert_eq!(left.proven_non_graphic["TEXT"], 7);
        assert_eq!(left.evidence_pending["NOZZ"], 5);
        assert_eq!(left.composites_built["NOZZ"], 5);
        assert_eq!(left.noun_evidence["NOZZ"], "ida");
    }

    /// 总长只要有一边没量过，合并结果就必须是 `None`。
    ///
    /// 把 `None` 当 0 加进去会得到一个偏小的总长，而它长得跟真数一模一样——
    /// 这类假数据比缺字段难查得多。
    #[test]
    fn an_unmeasured_total_length_poisons_the_sum_instead_of_counting_as_zero() {
        let mut measured = Report::default();
        measured.implied_tubes.total_length = Some(1200.0);
        let mut also_measured = Report::default();
        also_measured.implied_tubes.total_length = Some(300.0);
        let mut unmeasured = Report::default();

        let mut both = Report::default();
        both.implied_tubes.total_length = Some(1200.0);
        both.merge(also_measured);
        assert_eq!(both.implied_tubes.total_length, Some(1500.0));

        measured.merge(Report::default());
        assert_eq!(
            measured.implied_tubes.total_length, None,
            "对方没量过，合出来的总长就不能当成量过的"
        );

        unmeasured.merge(both);
        assert_eq!(unmeasured.implied_tubes.total_length, None);
    }
}

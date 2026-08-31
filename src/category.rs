//! noun → 建模方式的分类（对标 `build/MODCMP` 的 `I*COM` 谓词大 switch）。
//!
//! 原则与 Core3D 相同：**先分类再取几何**，几何路径由类别决定。
//! 与 core.dll 不同的是我们把「认识但本期不做」显式区分出来
//! （[`Category::Catalog`] / [`Category::Unsupported`]），谁被跳过报告里必须有数，
//! 静默缺件是最高级别缺陷（gen-model 宪法同款）。
//!
//! 分类表来源：aios-core `pdms_types.rs` 的 noun 家族清单 + ams1112 实测 noun 分布。

/// 一个 noun 的建模路径。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Category {
    /// 纯组织节点（`LISTOP`）：遍历成员，不建模。SITE/ZONE/STRU/FRMW/CMPF…
    List,
    /// 正环拉伸体：PLOO/LOOP 边界环 + HEIG 沿局部 +Z 挤出。PANE/FLOOR/EXTR/AEXTR/SCREED/GWALL。
    LoopExtrusion,
    /// 负环拉伸体（NXTR）：几何同上，挂在属主上做 CSG 扣减（`LNEGIT`/`INGCOM` 语义）。
    NegExtrusion,
    /// 正回转体（REVO）：环剖面绕局部 X 轴转 ANGL 度。
    Revolution,
    /// 负回转体（NREV）。
    NegRevolution,
    /// 多面体（POHE/POLYHE）：显式 B-rep——POLPTL 顶点表 + 面(POGON)/POLOOP 环/
    /// LOOPTS.VXREF 顶点引用（旧式 POHE 则面下直存顶点）。三种非标量图元里唯一
    /// 不过 D2_Profile 的（Core3D `sub_1071DCF0`，7UW4 §3.7.4）。
    Polyhedron,
    /// 负多面体（NPOLYH），挂属主 CSG 扣减。
    NegPolyhedron,
    /// 正基本体，见 [`Prim`]。
    Primitive(Prim),
    /// 负基本体，见 [`Prim`]。
    NegPrimitive(Prim),
    /// 是负几何，但本期建不出来（NCON/NDIS/NSNO…）。
    ///
    /// 它必须留在负几何家族里而不是丢进 [`Category::Unsupported`]：负体的账要记在
    /// **属主**头上。落进 unsupported 只会得到一行「NCON: 15」，看不出是哪 15 个
    /// FLOOR 少挖了刀；留在这里，属主的 `negatives_skipped` 会点名到 refno，
    /// RVM 对拍看到体积偏大时能直接对上号。
    NegUnimplemented(&'static str),
    /// 目录件（SPRE/CATR 驱动，走目录库求值）：FITT/PFIT/SBFI/FIXING/CMFI/WALL 族，
    /// 名单以 aios-core `USE_CATE_NOUN_NAMES` 为准。
    /// 本期记账不建模（压 G4 目录表达式求值，二期）。
    Catalog,
    /// 属主几何的**数据**而非独立实体：PLOO/PAVE/LOOP/VERT/SPINE/CURVE。
    /// 由属主的建模路径消费，遍历时不下钻。
    ProfileData,
    /// 无三维表示（`GMDRAW: Skipping non-3d` 同类）：基准/命名点/接头等。
    NonGraphic,
    /// 认识这个 noun、知道它有几何，但本期算法没实现——必须记账。
    Unsupported(&'static str),
    /// 分类表没认领的 noun——必须记账，宁可显式未知也不静默放行。
    Unknown,
}

impl Category {
    /// 是否是挂在属主上的负几何（`INGCOM` 谓词）。
    pub fn is_negative(&self) -> bool {
        matches!(
            self,
            Category::NegExtrusion
                | Category::NegRevolution
                | Category::NegPolyhedron
                | Category::NegPrimitive(_)
                | Category::NegUnimplemented(_)
        )
    }

    /// 是否自己产出正实体。
    pub fn is_positive_solid(&self) -> bool {
        matches!(
            self,
            Category::LoopExtrusion
                | Category::Revolution
                | Category::Polyhedron
                | Category::Primitive(_)
        )
    }

    /// 本元素的成员会不会被**它自己**消费掉（剖面数据 + 负几何）。
    ///
    /// 只有产正实体的元素消费成员；别处遇到 PLOO 或负体就是孤儿，要留名。
    pub fn consumes_members(&self) -> bool {
        self.is_positive_solid()
    }
}

/// 基本体细分（libgm `gm_Create*` 一一对应的那批）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Prim {
    /// XLEN/YLEN/ZLEN，POS 居中（`gm_CreateBox`）。
    Box,
    /// DIAM/HEIG，轴向局部 Z，POS 居中（`gm_CreateCylinder`）。
    Cylinder,
    /// XBOT/YBOT/XTOP/YTOP/HEIG + XOFF/YOFF 的棱台（`gm_CreatePyramid`）。
    Pyramid,
    /// RINS/ROUT/HEIG/ANGL，矩形截面绕局部 Z 扫（`GM_RectTorus`）。
    RectTorus,
    /// RINS/ROUT/ANGL，圆管截面绕局部 Z 扫（`GM_CircTorus`）。
    CircTorus,
}

/// noun 名 → 类别。名字用 e3d-io 反哈希出的规范大写 noun 名。
pub fn classify(noun: &str) -> Category {
    use Category::*;
    match noun {
        // ---- 组织节点（遍历不建模；EQUI/SUBE 自身不带几何，几何在基本体成员上） ----
        "WORL" | "SITE" | "ZONE" | "STRU" | "FRMW" | "SBFR" | "CMPF" | "GPART" | "REST"
        | "SUBS" | "EQUI" | "SUBE" => List,

        // ---- 船体分段容器：自己不出几何，几何全在成员上 ----
        //
        // CWALL（曲面墙）/ CFLOOR（曲面楼板）两个 noun 在 aios-core 的两张权威名单里
        // **都不在**：既不是 `GNERAL_LOOP_OWNER_NOUN_NAMES`（它们不带 PLOO），
        // 也不是 `USE_CATE_NOUN_NAMES`（不吃目录），而 `generation_root` 把它们当
        // 生成根——生成根就是「一整棵子树一起重建」的那个容器。
        //
        // ams1112 实测坐实：CWALL → PANE 1995 / GWALL 122 / STWALL 76 / WALL 14，
        // CFLOOR → PANE 2280 / FLOOR 81，两者自己一个 PLOO 都不带。曾把它们判成
        // 目录件与未实现项，于是 DFS 在这一层整片截断，4275 个 PANE 一个都没走到。
        "CWALL" | "CFLOOR" => List,

        // ---- 环拉伸家族（aios-core GNERAL_LOOP_OWNER_NOUN_NAMES） ----
        "PANE" | "FLOOR" | "EXTR" | "AEXTR" | "SCREED" | "GWALL" => LoopExtrusion,
        "NXTR" => NegExtrusion,
        "REVO" => Revolution,
        "NREV" => NegRevolution,

        // ---- 多面体（显式 B-rep，7UW4 §3.7.4；结构读法同 gen-model prim_model.rs） ----
        "POHE" | "POLYHE" => Polyhedron,
        "NPOLYH" => NegPolyhedron,

        // ---- 基本体（正负成对；其余记账） ----
        "BOX" => Primitive(Prim::Box),
        "CYLI" => Primitive(Prim::Cylinder),
        "PYRA" => Primitive(Prim::Pyramid),
        "RTOR" => Primitive(Prim::RectTorus),
        "CTOR" => Primitive(Prim::CircTorus),
        "NBOX" => NegPrimitive(Prim::Box),
        "NCYL" => NegPrimitive(Prim::Cylinder),
        "NPYR" => NegPrimitive(Prim::Pyramid),
        "NRTO" => NegPrimitive(Prim::RectTorus),
        "NCTO" => NegPrimitive(Prim::CircTorus),
        "SLCY" | "CONE" | "DISH" | "SNOU" => {
            Unsupported("positive primitive not implemented in milestone 1")
        }
        "NLCY" | "NSBO" | "NCON" | "NSNO" | "NDIS" | "NSCY" => {
            NegUnimplemented("negative primitive not implemented in milestone 1")
        }

        // ---- 目录件 / 型材（SPRE/CATR 驱动，二期） ----
        "FITT" | "PFIT" | "SBFI" | "CMFI" | "FIXING" | "WALL" | "STWALL" | "GENSEC" | "SCTN"
        | "PIPE" | "BRAN" | "HANG" => Catalog,

        // ---- 属主几何的数据 ----
        // 后五个是多面体的 B-rep 内脏（顶点表/面/环/顶点引用），由 Polyhedron
        // 属主消费，遍历时不下钻。
        "PLOO" | "PAVE" | "LOOP" | "VERT" | "SPINE" | "CURVE" | "POLPTL" | "POIN" | "POGON"
        | "POLOOP" | "LOOPTS" => ProfileData,

        // ---- 无三维表示 ----
        "JLDATU" | "PLDATU" | "POINSP" | "PNOD" | "SNOD" | "PJOI" | "SJOI" | "PALJ" | "SELJ"
        | "SCOJ" | "SUBJ" | "SEVE" | "RNODE" | "CNODE" | "NOZZ" | "ELCONN" | "APPDAR"
        | "APPLDA" | "APPLDW" => NonGraphic,
        // 库务编号根（ams1112 实测 `17496/1`，名 `/**`，与 WORL `/*` 并列的第二个
        // 索引根）：无成员、无几何，只是 dabacon 的记账元素。
        "MNUM" => NonGraphic,

        _ => Unknown,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// ams1112 实测 noun 全集必须每个都有归属，一个 Unknown 都不许有——
    /// 分类表漏一个 noun，那个 noun 的元素就整族缺席，报告里只剩一行 unknown 计数。
    #[test]
    fn every_ams1112_noun_is_claimed() {
        const AMS1112_NOUNS: &[&str] = &[
            "CFLOOR", "CMFI", "CMPF", "CURVE", "CWALL", "FITT", "FIXING", "FLOOR", "FRMW", "GWALL",
            "JLDATU", "LOOP", "MNUM", "NBOX", "NCTO", "NCYL", "NPYR", "NREV", "NRTO", "NXTR",
            "PANE", "PAVE", "PFIT", "PJOI", "PLDATU", "PLOO", "PNOD", "POINSP", "SBFI", "SITE",
            "SJOI", "SNOD", "SPINE", "STRU", "STWALL", "VERT", "WALL", "ZONE",
        ];
        let unknown: Vec<_> = AMS1112_NOUNS
            .iter()
            .filter(|noun| matches!(classify(noun), Category::Unknown))
            .collect();
        assert!(unknown.is_empty(), "1112 的 noun 必须全部认领：{unknown:?}");
    }

    /// 多面体家族全链路接线（7UW4 §3.7.4 落地）：正体产实体并消费成员，
    /// 负体归负几何家族，B-rep 内脏 noun 全部算属主数据、不下钻。
    #[test]
    fn polyhedron_family_is_wired() {
        assert_eq!(classify("POHE"), Category::Polyhedron);
        assert_eq!(classify("POLYHE"), Category::Polyhedron);
        assert_eq!(classify("NPOLYH"), Category::NegPolyhedron);
        assert!(classify("NPOLYH").is_negative());
        assert!(classify("POLYHE").is_positive_solid());
        assert!(classify("POLYHE").consumes_members());
        for noun in ["POLPTL", "POIN", "POGON", "POLOOP", "LOOPTS"] {
            assert_eq!(classify(noun), Category::ProfileData, "{noun}");
        }
    }

    #[test]
    fn negative_predicates_match_the_ingcom_family() {
        assert!(classify("NXTR").is_negative());
        assert!(classify("NBOX").is_negative());
        assert!(classify("NREV").is_negative());
        assert!(!classify("PANE").is_negative());
        assert!(classify("PANE").is_positive_solid());
        assert!(!classify("PLOO").is_positive_solid());
    }

    /// ams1112 出现过的负体 noun 必须条条能建，不许再有「有依据地不出」。
    ///
    /// 这四种曾经是 `NegUnimplemented`：属主的正体照出、负体跳过，后果是那些
    /// FLOOR/GWALL 的体积整整多出一个负体——而 RVM 对拍比的正是体积。带着已知
    /// 原因的偏差去对拍，等于在结果里先撒一把噪声，真正的未知偏差就淹掉了。
    #[test]
    fn every_negative_in_ams1112_can_actually_be_built() {
        let expect = [
            ("NXTR", Category::NegExtrusion),
            ("NREV", Category::NegRevolution),
            ("NBOX", Category::NegPrimitive(Prim::Box)),
            ("NCYL", Category::NegPrimitive(Prim::Cylinder)),
            ("NPYR", Category::NegPrimitive(Prim::Pyramid)),
            ("NRTO", Category::NegPrimitive(Prim::RectTorus)),
            ("NCTO", Category::NegPrimitive(Prim::CircTorus)),
        ];
        for (noun, category) in expect {
            assert_eq!(classify(noun), category, "{noun}");
        }
        // 正体侧的同族必须走同一条几何路径，否则同一个形状会有两套实现。
        assert_eq!(classify("PYRA"), Category::Primitive(Prim::Pyramid));
        assert_eq!(classify("RTOR"), Category::Primitive(Prim::RectTorus));
        assert_eq!(classify("CTOR"), Category::Primitive(Prim::CircTorus));
        assert_eq!(classify("REVO"), Category::Revolution);
    }

    /// 建不出来的负体也算负体：它的账要记在属主头上，不能滑进 unsupported
    /// 变成一行「NCON: 15」——那样就看不出是哪些 FLOOR 少挖了刀。
    #[test]
    fn unimplemented_negatives_stay_in_the_negative_family() {
        for noun in ["NSBO", "NCON", "NSNO", "NDIS", "NSCY", "NLCY"] {
            let category = classify(noun);
            assert!(
                category.is_negative(),
                "{noun} 应当仍算负几何，实得 {category:?}"
            );
            assert!(
                !matches!(category, Category::Unsupported(_)),
                "{noun} 落进 unsupported 就丢了属主"
            );
        }
    }

    /// 只有产正实体的元素才消费成员（剖面 + 负体）。别处遇到 PLOO / 负体是孤儿，
    /// 得留名而不是被当作「已被消费」抹掉。
    #[test]
    fn only_positive_solids_consume_their_members() {
        assert!(classify("PANE").consumes_members());
        assert!(classify("BOX").consumes_members());
        for noun in ["CWALL", "CFLOOR", "ZONE", "FITT", "PLOO", "NXTR"] {
            assert!(!classify(noun).consumes_members(), "{noun} 不该消费成员");
        }
    }

    /// ams1112 的容器 noun 必须是可下钻的组织节点。
    ///
    /// 这条是一次真事故的回归：CWALL 曾被判成目录件、CFLOOR 被判成未实现项，
    /// 两者都不下钻，DFS 于是在 FRMW 下一层就停住——全库 30940 个元素只走到 160 个，
    /// 4275 个 PANE 一个都没进管线，而总账里只看得到「catalog: CWALL 121」这样
    /// 一行人畜无害的计数。
    #[test]
    fn hull_containers_are_traversable_lists() {
        for noun in ["CWALL", "CFLOOR"] {
            assert_eq!(
                classify(noun),
                Category::List,
                "{noun} 自己不带 PLOO、也不吃目录，是生成根容器，必须下钻"
            );
        }
        // 反面：真正吃目录的墙族仍然是目录件（aios-core USE_CATE_NOUN_NAMES）。
        for noun in ["WALL", "STWALL", "FITT", "SBFI"] {
            assert_eq!(classify(noun), Category::Catalog, "{noun} 是目录件");
        }
        // GWALL 自带 PLOO，是环拉伸而不是容器。
        assert_eq!(classify("GWALL"), Category::LoopExtrusion);
    }
}

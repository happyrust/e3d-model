//! 生成器**读到的属性名集合**（gen-model 开发计划 2026-09-15 D2 / S5）。
//!
//! 它是增量「不影响模型」判定的第二票：一个 DESI 属性的变化只在 **字典 DCHC == 0** 且
//! **生成器不读这个名字** 时才允许跳过（gen-model `model_impact::attribute_is_metadata_only`，S6 起）。
//! 「读」按最保守的口径估：本 crate `src/**/*.rs`（`src/bin` 除外，每个文件截到它第一个
//! cfg(test) 标记——所以这份注释不能把那个标记原样写出来）里出现的、是 E3D 属性名
//! （字典 4270 ∪ 运行库 schema 4478）的全部大写字面量
//! `"[A-Z][A-Z0-9]{1,7}"`。多收一个名字只是少跳一次；漏一个名字就是一张停在旧形状上的网格，
//! 所以宁宽勿漏——`PARA` 之类经变量传进 `get_double(name)` 的间接读点，名字仍来自某处的字面量，
//! 这一扫就收得到。
//!
//! 产物 `data/attribute-read-set.json` 由 gen-model `output/gen_read_set.py` 生成（它手上有字典与
//! schema，本 crate 没有），随源码入库、编译期嵌入；`tests/attribute_reads_guard.rs` 用同一条规则
//! 重扫源码，与嵌入的产物**逐名相等**才绿——源码里多出一个大写字面量，要么重跑生成脚本把它归进
//! `names`（是属性）或 `non_attribute_literals`（不是），要么它是字典 noun 名（自动放行），
//! 要么它是**戳**（[`AttributeReadSet::stamp_literals`]：是属性名，但源码提它只为把它排除掉）。
//!
//! 边界：这里只收**本 crate** 的读点。e3d-io 的目录表达式求值环境（`db_param_env.rs`）替生成器读的
//! 设计侧 `ANGL RADI HEIG DESP UNIPAR` 与目录侧 `DATA.PURP/DKEY/NUMB`、`SELE.QUES/ANSW/MAXA/TANS`
//! 由 gen-model 那份 `generation_read_set.json` 单列（只作文档，不进那一票——它们要么 DCHC ≠ 0、
//! 要么只在目录元素上读，而元数据门只跑 DESI 窗口）。

use std::collections::BTreeSet;
use std::sync::OnceLock;

const ATTRIBUTE_READ_SET_JSON: &str = include_str!("../data/attribute-read-set.json");

/// `data/attribute-read-set.json` 的形状。
#[derive(Debug, serde::Deserialize)]
pub struct AttributeReadSet {
    /// 是 E3D 属性名的大写字面量——生成器读到的属性名，升序、无重复。
    pub names: Vec<String>,
    /// **戳**：是属性名（在运行库 schema 里），但源码提它只为**按名字把它排除掉**——
    /// [`crate::element_diff::PLACEMENT_STAMP_ATTRIBUTES`] 就是这张表，生成脚本的
    /// `STAMP_LITERALS` 与它逐名相等（守卫钉）。写一句名字不是读，所以它们**不进** [`Self::names`]：
    /// 收进去就等于让「只动了摆放」这条判据自己写的排除名单反过来把它挡住。
    /// 真被传进 e3d-io 的按名读取口照旧算读、照旧回到 `names`（直接读点无条件算属性名），守卫随即红。
    pub stamp_literals: Vec<String>,
    /// 既不是属性名也不是字典 noun 的大写字面量（家族位 `IPFCOM`、方位词 `LEFT` / `TOP`、
    /// 设计表键 `TLEN`…），升序、无重复。列出来是为了让守卫分得清「新冒出来的没归类的字面量」
    /// 与「早就知道不是属性的字串」。
    pub non_attribute_literals: Vec<String>,
}

/// 嵌入的产物，懒解析一次。
pub fn attribute_read_set_file() -> &'static AttributeReadSet {
    static FILE: OnceLock<AttributeReadSet> = OnceLock::new();
    FILE.get_or_init(|| {
        serde_json::from_str(ATTRIBUTE_READ_SET_JSON)
            .expect("data/attribute-read-set.json 随源码入库，必须是合法 JSON")
    })
}

/// 生成器读到的属性名集合（D2 的第二票要问的那一份）。
pub fn attribute_read_set() -> &'static BTreeSet<&'static str> {
    static SET: OnceLock<BTreeSet<&'static str>> = OnceLock::new();
    SET.get_or_init(|| {
        attribute_read_set_file()
            .names
            .iter()
            .map(String::as_str)
            .collect()
    })
}

/// 生成器是否读这个属性名（大小写 / 空白不敏感）。
pub fn reads_attribute(name: &str) -> bool {
    attribute_read_set().contains(name.trim().to_ascii_uppercase().as_str())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_embedded_read_set_is_sorted_unique_and_answers_the_known_reads() {
        let file = attribute_read_set_file();
        for side in [&file.names, &file.non_attribute_literals] {
            assert!(
                side.windows(2).all(|pair| pair[0] < pair[1]),
                "产物两侧必须严格升序：{side:?}"
            );
        }
        assert!(
            file.names
                .iter()
                .all(|n| !file.non_attribute_literals.contains(n)),
            "同一个字面量不能既是属性名又是非属性"
        );
        // 摆放、几何、目录链、多面体顶点引用：生成器确实读。
        for name in [
            "POS", "ORI", "DIAM", "HEIG", "SPRE", "CATR", "VXREF", "pos", " diam ",
        ] {
            assert!(reads_attribute(name), "{name}");
        }
        // 业务元数据：一个都不读——这是元数据门第二票的前提。戳同理：差分层按名字排除它，不是读它。
        for name in ["NAME", "DESC", "PURP", "FUNC", "LCHKDA", "CACHID"] {
            assert!(!reads_attribute(name), "{name} 不该被生成器读到");
        }
    }
}

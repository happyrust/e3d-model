//! 护栏：生成器源码里出现的**每一个属性名字面量**都在嵌入的读取集合 `data/attribute-read-set.json`
//! 里，集合里的每一个名字也真的还在源码里——两边逐名相等（gen-model 开发计划 2026-09-15 D2 / S5）。
//!
//! 读取集合是元数据门的第二票（DCHC == 0 ∧ 生成器不读 → 才许跳过），它漏一个名字的后果是
//! **一张停在旧形状上的网格，而且没有任何东西会报**；所以这里按最宽的口径扫：源码里出现的、是 E3D
//! 属性名的全部大写字面量，不管它是直接传进 `get_double("HEIG")` 还是经变量 / 常量 / 名单绕了几手。
//!
//! 扫描规则与 gen-model `output/gen_read_set.py` **逐字相同**（改一处必须同步改另一处）：
//! - 字面量 = `"` `[A-Z][A-Z0-9]{1,7}` `"`，不分析字串语境；
//! - `src/**/*.rs`，`src/bin` 除外；每个文件截到它第一个 `#[cfg(test)]`（测试模块都在文件尾）；
//! - 属性名 / 非属性的划分由生成脚本按字典 ∪ 运行库 schema 决定并写进产物；本 crate 没有那两份数据，
//!   只能核对「每个字面量都被产物归了类」：`names`（属性名）∪ `stamp_literals`（是属性名，但源码提它
//!   只为把它排除掉）∪ `non_attribute_literals`（不是属性）∪ 字典 noun 名
//!   （`category::dictionary_noun_names`，自动放行）。
//!
//! 红了怎么办：源码新增了大写字面量 → 在 gen-model 仓跑 `python output/gen_read_set.py`，它会按字典
//! 把新字面量归到该去的一侧，再把两份产物一起入库；产物里有名字源码已经不读 → 同样重跑脚本。

use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Path, PathBuf};

const TEST_MARKER: &str = "#[cfg(test)]";

/// 与 `gen_read_set.py::rust_sources(skip_bin=True)` 同口径：`src` 递归，跳过 `src/bin`。
fn rust_sources(root: &Path, dir: &Path, out: &mut Vec<PathBuf>) {
    for entry in fs::read_dir(dir).expect("read src dir") {
        let path = entry.expect("dir entry").path();
        if path.is_dir() {
            if path.parent() == Some(root) && path.file_name().is_some_and(|n| n == "bin") {
                continue;
            }
            rust_sources(root, &path, out);
        } else if path.extension().is_some_and(|ext| ext == "rs") {
            out.push(path);
        }
    }
}

/// 与 `gen_read_set.py::strip_tests` 同口径：截到第一个 `#[cfg(test)]`。
fn strip_tests(source: &str) -> &str {
    match source.find(TEST_MARKER) {
        Some(cut) => &source[..cut],
        None => source,
    }
}

/// 与正则 `"([A-Z][A-Z0-9]{1,7})"` 同口径：引号、2–8 个大写字母数字（首字母）、引号；不看字串语境。
fn uppercase_literals(source: &str, out: &mut BTreeSet<String>) {
    let bytes = source.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] != b'"' {
            i += 1;
            continue;
        }
        let start = i + 1;
        let mut end = start;
        while end < bytes.len() && end - start < 8 {
            let c = bytes[end];
            let ok = if end == start {
                c.is_ascii_uppercase()
            } else {
                c.is_ascii_uppercase() || c.is_ascii_digit()
            };
            if !ok {
                break;
            }
            end += 1;
        }
        if end - start >= 2 && end < bytes.len() && bytes[end] == b'"' {
            out.insert(source[start..end].to_string());
            // 正则是非重叠匹配：收尾引号被消费掉，从它后面继续。
            i = end + 1;
        } else {
            i += 1;
        }
    }
}

/// 全部字面量 → 出现的文件（给报错点名用）。
fn collect_literals() -> BTreeMap<String, BTreeSet<String>> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    let mut files = Vec::new();
    rust_sources(&root, &root, &mut files);
    files.sort();
    let mut by_literal: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    for path in files {
        let source = fs::read_to_string(&path).expect("read source");
        let mut literals = BTreeSet::new();
        uppercase_literals(strip_tests(&source), &mut literals);
        let file = path
            .strip_prefix(&root)
            .unwrap()
            .to_string_lossy()
            .replace('\\', "/");
        for literal in literals {
            by_literal.entry(literal).or_default().insert(file.clone());
        }
    }
    by_literal
}

/// 源码里出现的每个大写字面量都被产物归了类；产物里的每个名字也都还在源码里。
#[test]
fn every_attribute_literal_in_the_generator_is_in_the_read_set() {
    let file = e3d_model::attribute_reads::attribute_read_set_file();
    let names: BTreeSet<&str> = file.names.iter().map(String::as_str).collect();
    let stamps: BTreeSet<&str> = file.stamp_literals.iter().map(String::as_str).collect();
    let non_attribute: BTreeSet<&str> = file
        .non_attribute_literals
        .iter()
        .map(String::as_str)
        .collect();
    let nouns: BTreeSet<&str> = e3d_model::category::dictionary_noun_names().collect();
    assert_eq!(names.len(), file.names.len(), "names 里有重复");
    assert_eq!(
        non_attribute.len(),
        file.non_attribute_literals.len(),
        "non_attribute_literals 里有重复"
    );

    let by_literal = collect_literals();
    let unclassified: Vec<String> = by_literal
        .iter()
        .filter(|(literal, _)| {
            let literal = literal.as_str();
            !names.contains(literal)
                && !stamps.contains(literal)
                && !non_attribute.contains(literal)
                && !nouns.contains(literal)
        })
        .map(|(literal, files)| {
            format!(
                "{literal} ({})",
                files.iter().cloned().collect::<Vec<_>>().join(", ")
            )
        })
        .collect();
    assert!(
        unclassified.is_empty(),
        "这些大写字面量既不在读取集合、也不在非属性名单、也不是 noun：{unclassified:?}。\
         在 gen-model 仓跑 `python output/gen_read_set.py` 重生成 data/attribute-read-set.json（它按字典判它们是不是属性）。"
    );

    let literals: BTreeSet<&str> = by_literal.keys().map(String::as_str).collect();
    let stale_names: Vec<&str> = names.difference(&literals).copied().collect();
    assert!(
        stale_names.is_empty(),
        "读取集合里这些名字源码已经不读了，重跑生成脚本：{stale_names:?}"
    );
    let stale_other: Vec<&str> = non_attribute.difference(&literals).copied().collect();
    assert!(
        stale_other.is_empty(),
        "非属性名单里这些字面量源码里已经没有了，重跑生成脚本：{stale_other:?}"
    );

    // 两侧互斥；noun 名不该被记成非属性（生成脚本会先把 noun 摘掉）。
    let both: Vec<&str> = names.intersection(&non_attribute).copied().collect();
    assert!(both.is_empty(), "同一个字面量两侧都有：{both:?}");
    let noun_as_other: Vec<&str> = non_attribute.intersection(&nouns).copied().collect();
    assert!(
        noun_as_other.is_empty(),
        "这些是字典 noun 名，不该出现在 non_attribute_literals：{noun_as_other:?}"
    );

    // 戳（2026-09-16 Q5）：生成脚本的 STAMP_LITERALS 与差分层那张表必须是同一份名单。
    // 这条一红就说明两仓的口径漂开了——重跑生成脚本，或者先弄清那个名字到底该不该算戳。
    assert_eq!(
        file.stamp_literals,
        e3d_model::element_diff::PLACEMENT_STAMP_ATTRIBUTES,
        "产物的 stamp_literals 与 element_diff::PLACEMENT_STAMP_ATTRIBUTES 漂开了"
    );
    // 戳不许混进读取集合：它一旦进 `names`，元数据门与便宜路径判据就被自己写的排除名单挡住了。
    // 会让它进去的只有一种写法——把它传进 e3d-io 的按名读取口（直接读点无条件算属性名）；
    // 真到那一步，先回答「那处读是不是几何输入」，再决定改判据还是改这张表。
    let stamp_as_name: Vec<&str> = names.intersection(&stamps).copied().collect();
    assert!(
        stamp_as_name.is_empty(),
        "这些戳出现在读取集合里（多半是新加了一处按名读取口）：{stamp_as_name:?}"
    );
    let stamp_as_other: Vec<&str> = non_attribute.intersection(&stamps).copied().collect();
    assert!(
        stamp_as_other.is_empty(),
        "戳不是非属性：{stamp_as_other:?}"
    );
    // 反过来，排除名单也不许养闲人：每个戳都得真在源码里出现（`PLACEMENT_STAMP_ATTRIBUTES`
    // 自己就是那处出现），不然这条排除口径已经没有对象了。
    let dead_stamps: Vec<&str> = stamps.difference(&literals).copied().collect();
    assert!(
        dead_stamps.is_empty(),
        "这些戳源码里已经不提了，从 STAMP_LITERALS / PLACEMENT_STAMP_ATTRIBUTES 里摘掉：{dead_stamps:?}"
    );

    // 今天的形状：142 个属性名（141 个字典 / schema 名 + 伪属性 `CPAR`；关节负体带进 JOIS / JOIE /
    // NAPP / CREF，宿主 FIXING 扫描带进 NGMR，详图式扶手立柱的现算标架带进 HRJUSL / WIDT）；
    // 变了不是错，是提醒把 gen-model 那份 generation_read_set.json 一起重生成。
    assert_eq!(names.len(), 142, "读取集合条数漂移：两份产物要一起重生成");
}

/// 直接读点的字面量必然在集合里——`get_*("X")` / `attribute("X")` 一个都不许漏，这是宽口径之下
/// 最不该出错的那一层，单独钉一次，报错时能直接点到调用形状。
#[test]
fn every_direct_reader_call_literal_is_in_the_read_set() {
    const READERS: &[&str] = &[
        ".get_string(\"",
        ".get_integer(\"",
        ".get_double(\"",
        ".get_bool(\"",
        ".get_ref(\"",
        ".get_integer_array(\"",
        ".get_double_array(\"",
        ".get_ref_array(\"",
        ".get_position(\"",
        ".get_direction(\"",
        ".get_orientation(\"",
        ".get_element(\"",
        ".get_element_array(\"",
        ".attribute(\"",
        ".has_attribute(\"",
    ];
    let names = e3d_model::attribute_read_set();
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    let mut files = Vec::new();
    rust_sources(&root, &root, &mut files);
    let mut direct = BTreeSet::new();
    for path in files {
        let source = fs::read_to_string(&path).expect("read source");
        let source = strip_tests(&source);
        for reader in READERS {
            let mut from = 0;
            while let Some(at) = source[from..].find(reader) {
                let open = from + at + reader.len();
                let close = source[open..]
                    .find('"')
                    .map(|o| open + o)
                    .expect("closing quote");
                direct.insert(source[open..close].to_string());
                from = close;
            }
        }
    }
    // 2026-09-15 是 45 个不同的名字；门槛只为识破扫描器空转，不追口径。
    assert!(
        direct.len() > 30,
        "直接读点太少（{}），扫描器坏了？",
        direct.len()
    );
    let missing: Vec<&str> = direct
        .iter()
        .map(String::as_str)
        .filter(|name| !names.contains(name))
        .collect();
    assert!(
        missing.is_empty(),
        "这些直接读点的属性名不在读取集合里：{missing:?}"
    );
}

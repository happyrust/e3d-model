//! 护栏：几何管线读到的**每一个元素引用属性**都必须落在三档之一，三档之外一律红。
//!
//! | 档 | 含义 | 增量怎么覆盖「被引用的那一端变了」 |
//! |---|---|---|
//! | (a) `FILE_TABLE_REFERENCES` | dab 文件里有反向引用表（属性描述符 kind 16，核心在 `db_put_reference` 写时同步维护） | 按目标 refno 在表里反查引用者（e3d-io `Engine::referrers_of`） |
//! | (b) `INTRA_UNIT_REFERENCES` | 没有表，但目标与引用者在同一个模型单元里 | 单元整体重建，owner 链上卷已覆盖 |
//! | (c) `ACCEPTED_NO_TABLE_REFERENCES` | 没有表、目标跨单元 | **在案缺口**，必须在这里点名并写明为什么可以接受 |
//!
//! 新增一条跨单元引用读而不归档 = 一个静默不重生成的来源，这正是本 crate 最高级别的缺陷
//! （见 `increment.rs` 模块文档「账本」一节）。
//!
//! 三张名单的来源：gen-model `docs/evidence/2026-09-06-ref-rev-vs-file-table-coverage.md`
//! （783 个 E3D 3.1 库文件实测：哪些属性名有反向表、各在哪类库里）。
//! 名单**不许**凭字典的 ELEMENT 类型推——`TYPEX` 同形不同义，`HREF`/`TREF`/`CREF` 是 ELEMENT 却没有表。

use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Path, PathBuf};

/// (a) 语料里实测有反向引用表的属性名（含 CATA 侧链：SPCO→SCOM→GMSE/PTSE/…）。
const FILE_TABLE_REFERENCES: &[&str] = &[
    "SPRE", "CATR", "PRTREF", "PTRE", "GMRE", "GSTR", "PSTR", "HSTU", "LSTU", "POSREF", "STAREF",
    "ENDREF",
];

/// (b) 无表、目标在同一单元内。
const INTRA_UNIT_REFERENCES: &[(&str, &str)] = &[(
    "VXREF",
    "POHE 面片引用同一 POHE 下 POLPTL 的 POIN；面与点都是这个单元的成员，单元整体重建",
)];

/// (c) 无表、目标跨单元——在案缺口，逐条写理由。
const ACCEPTED_NO_TABLE_REFERENCES: &[(&str, &str)] = &[
    (
        "HSRO",
        "容器头 stub 的备选引用，只在 HSTU 为空时才读；目标是 CATA 的 SPCO。783 库语料里没有一张 HSRO 表，\
         即 E3D 自己也不为它维护反向索引；SPCO 变了走 SPRE/HSTU 表反查的引用者集合基本覆盖同一批根",
    ),
    (
        "LSRO",
        "成员离开侧 stub 的备选引用，只在 LSTU 为空时才读；理由同 HSRO",
    ),
];

/// 间接读点：属性名不是字面量、经变量传进 `get_element(...)` 的那几处。**每一处都要在这里
/// 登记它最终会读到哪些名字**；出现没登记的间接读点就红——那是一条没归档的引用。
///
/// `(文件名后缀, 变量名, 该文件里必须能找到的字面量)`。
const INDIRECT_READ_SITES: &[(&str, &str, &[&str])] = &[
    // `via` 在 SPRE → CATR/PRTREF 的目录链上逐跳切换。
    ("section.rs", "via", &["\"SPRE\"", "\"CATR\"", "\"PRTREF\""]),
    // `tube_outer_diameter_from_stub(departure, &[...])` 的 stub 名单由 pipeline.rs 传入。
    ("catalogue_point.rs", "name", &[]),
    (
        "pipeline.rs",
        "",
        &["&[\"HSTU\", \"HSRO\"]", "&[\"LSTU\", \"LSRO\"]"],
    ),
    // `point_index(name)` 闭包：STAREF / ENDREF 指向属主 HANDRA 的 POINTR。
    (
        "handrail.rs",
        "name",
        &["point_index(\"STAREF\")", "point_index(\"ENDREF\")"],
    ),
    // `reference(el, name)` 是 `get_element` 的薄包装，调用点都是字面量。
    (
        "cable_tray.rs",
        "name",
        &[
            "reference(el, \"POSREF\")",
            "reference(current, \"POSREF\")",
        ],
    ),
];

/// 经间接读点最终读到的名字（与上表配套，参与三档归属校验）。
const INDIRECT_READ_NAMES: &[&str] = &[
    "SPRE", "CATR", "PRTREF", "HSTU", "HSRO", "LSTU", "LSRO", "STAREF", "ENDREF", "POSREF",
];

const READ_CALLS: &[&str] = &[".get_element(", ".get_ref(", ".get_ref_array("];

fn rust_sources(dir: &Path, out: &mut Vec<PathBuf>) {
    for entry in fs::read_dir(dir).expect("read src dir") {
        let path = entry.expect("dir entry").path();
        if path.is_dir() {
            rust_sources(&path, out);
        } else if path.extension().is_some_and(|ext| ext == "rs") {
            out.push(path);
        }
    }
}

#[derive(Debug, Default)]
struct Reads {
    /// 字面量或 `const NAME: &str = "..."` 解出来的名字。
    direct: BTreeSet<String>,
    /// `(文件名, 变量名)` —— 经变量传入的读点。
    indirect: BTreeSet<(String, String)>,
}

fn resolve_const(source: &str, ident: &str) -> Option<String> {
    let needle = format!("const {ident}: &str = \"");
    let start = source.find(&needle)? + needle.len();
    let end = source[start..].find('"')? + start;
    Some(source[start..end].to_string())
}

fn scan(path: &Path, source: &str, reads: &mut Reads) {
    let file = path
        .file_name()
        .and_then(|name| name.to_str())
        .expect("utf-8 file name")
        .to_string();
    for call in READ_CALLS {
        let mut from = 0;
        while let Some(at) = source[from..].find(call) {
            let open = from + at + call.len();
            let close = source[open..]
                .find(')')
                .map(|offset| open + offset)
                .unwrap_or_else(|| panic!("{file}: unterminated {call}"));
            let arg = source[open..close].trim();
            from = close;
            if let Some(literal) = arg.strip_prefix('"') {
                let name = literal.trim_end_matches('"');
                reads.direct.insert(name.to_string());
            } else if arg.chars().all(|c| c.is_ascii_uppercase() || c == '_') {
                let resolved = resolve_const(source, arg)
                    .unwrap_or_else(|| panic!("{file}: {call}{arg}) 的常量在本文件里解不出来"));
                reads.direct.insert(resolved);
            } else {
                reads.indirect.insert((file.clone(), arg.to_string()));
            }
        }
    }
}

fn collect_reads() -> (Reads, BTreeMap<String, String>) {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    let mut files = Vec::new();
    rust_sources(&root, &mut files);
    files.sort();
    let mut reads = Reads::default();
    let mut sources = BTreeMap::new();
    for path in files {
        let source = fs::read_to_string(&path).expect("read source");
        scan(&path, &source, &mut reads);
        let file = path.file_name().unwrap().to_str().unwrap().to_string();
        // 同名文件（如多个 mod.rs）拼在一起查字面量即可。
        sources
            .entry(file)
            .and_modify(|joined: &mut String| joined.push_str(&source))
            .or_insert(source);
    }
    (reads, sources)
}

/// 每一个读到的引用属性都归进三档之一；三档里的每一条都真的被读（名单不许过期）。
#[test]
fn every_reference_read_is_filed_under_one_of_three_categories() {
    let (reads, _) = collect_reads();
    let mut read: BTreeSet<&str> = reads.direct.iter().map(String::as_str).collect();
    read.extend(INDIRECT_READ_NAMES.iter().copied());

    let mut filed: BTreeMap<&str, &str> = BTreeMap::new();
    for name in FILE_TABLE_REFERENCES {
        assert!(
            filed.insert(name, "file-table").is_none(),
            "{name} 重复归档"
        );
    }
    for (name, _) in INTRA_UNIT_REFERENCES {
        assert!(
            filed.insert(name, "intra-unit").is_none(),
            "{name} 重复归档"
        );
    }
    for (name, why) in ACCEPTED_NO_TABLE_REFERENCES {
        assert!(!why.trim().is_empty(), "{name} 的接受理由不能为空");
        assert!(
            filed.insert(name, "accepted-no-table").is_none(),
            "{name} 重复归档"
        );
    }

    let unfiled: Vec<&str> = read
        .iter()
        .copied()
        .filter(|n| !filed.contains_key(n))
        .collect();
    assert!(
        unfiled.is_empty(),
        "这些引用属性被几何读到了，却没归进任何一档（有反向表 / 单元内 / 在案缺口）：{unfiled:?}。\
         先查 gen-model docs/evidence/2026-09-06-ref-rev-vs-file-table-coverage.md 看它有没有表，再归档。"
    );
    let stale: Vec<&str> = filed
        .keys()
        .copied()
        .filter(|n| !read.contains(n))
        .collect();
    assert!(
        stale.is_empty(),
        "名单里这些属性已经没人读了，删掉它们：{stale:?}"
    );
}

/// 间接读点（属性名经变量传入）必须逐处登记，并且登记的字面量真的在那个文件里。
#[test]
fn indirect_reference_reads_are_each_registered_with_their_names() {
    let (reads, sources) = collect_reads();
    let expected: BTreeSet<(String, String)> = INDIRECT_READ_SITES
        .iter()
        .filter(|(_, var, _)| !var.is_empty())
        .map(|(file, var, _)| (file.to_string(), var.to_string()))
        .collect();
    assert_eq!(
        reads.indirect, expected,
        "间接读点集合变了：多出来的每一处都要在 INDIRECT_READ_SITES 登记它最终读到哪些名字；\
         消失的那一处从表里删掉"
    );
    for (file, _, literals) in INDIRECT_READ_SITES {
        let source = sources
            .get(*file)
            .unwrap_or_else(|| panic!("src 里找不到 {file}"));
        for literal in *literals {
            assert!(
                source.contains(literal),
                "{file} 里找不到登记的字面量 {literal}——间接读点读到的名字变了，更新 INDIRECT_READ_NAMES"
            );
        }
    }
}

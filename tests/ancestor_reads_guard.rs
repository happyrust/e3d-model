//! 护栏：生成器**沿 owner 链向上读到的东西**只能是三样——`POS` / `ORI`（位姿，增量按锚点前移覆盖）、
//! noun（`element_type()`，改 noun 就是删 + 建）、refno——此外每一处都得在这里**登记**它读的是谁、
//! 为什么增量能覆盖（gen-model issue #39，2026-09-22 审阅 C-1）。
//!
//! 为什么单独钉这一层：读取集合守卫（`attribute_reads_guard.rs`）只钉「读了哪个属性名」，不钉
//! 「从谁身上读」。增量选根按**候选元素的祖先闭包**上卷——根之上的祖先属性变了，不会落进任何一个
//! 根的闭包（只有 `POS` / `ORI` 走锚点那条路）。所以生成器一旦在 builder 里 `owner().get_double("LEVE")`，
//! 那就是一张停在旧形状上的网格，而且读取集合守卫是绿的。
//!
//! 怎么钉（纯源码扫描，与 `reference_reads_guard.rs` 的间接读点登记同型）：
//! - 找出 `src/**/*.rs`（`src/bin` 除外，截掉 `#[cfg(test)]` 之后）里**每一处** `.owner()` 调用，
//!   定位到它所在的 fn；这些 `(文件, fn)` 必须与 [`OWNER_WALK_SITES`] 逐条相等——新开一处 owner 上溯
//!   就得来登记；
//! - 每个登记的 fn，它函数体里出现的属性名字面量（大写字面量 ∩ 嵌入读取集合，noun 名除外）必须 ⊆
//!   登记的 `reads`——在 `world_matrix` 里多读一个 `LEVE` 就红；
//! - 上溯拿到的祖先常被**传给别的函数**去读（`first_rpath(handrail)`、`fixing_host_frame(wall, ..)`），
//!   这些消费者按 [`ANCESTOR_CONSUMERS`] 同样逐条登记、同样只许读登记的名字；
//! - 位姿那条路 `local_matrix` / `local_placement` 只许读 `POS` / `ORI`，桥架伪属性分支必须挡在
//!   `is_cable_tray_element` 之后。
//!
//! 口径是**函数级**的近似：同一个 fn 里读自身与读祖先分不开，所以 `reads` 是「这个 fn 一共读了什么」，
//! 每条登记的 `why` 要把「哪些是读祖先的、增量怎么覆盖」写清楚。四档：
//!
//! | 档 | 含义 | 增量怎么覆盖 |
//! |---|---|---|
//! | `Transform` | 只为折世界矩阵读 `POS` / `ORI` | 锚点 / 刚体前移（ADR-066）与 regen 同口径 |
//! | `NounOnly` | 只读祖先的 noun / refno | 改 noun = 删 + 建，两端都是候选 |
//! | `WithinUnit` | 读的祖先与读者在**同一个模型单元**里 | 单元整体重建，owner 链上卷覆盖 |
//! | `AboveUnit` | 读的祖先在读者的单元**之上** | **在案缺口**：只有整棵子树重算的根口径才覆盖，单元级发布（ADR-080）不覆盖——理由逐条写 |

use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Path, PathBuf};

const TEST_MARKER: &str = "#[cfg(test)]";

/// 上溯拿到的祖先被怎么用。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Climb {
    Transform,
    NounOnly,
    WithinUnit,
    AboveUnit,
}

/// 一处 owner 上溯：`(文件名, fn 名, 档, 这个 fn 一共读到的属性名, 理由)`。
struct Site {
    file: &'static str,
    func: &'static str,
    climb: Climb,
    reads: &'static [&'static str],
    why: &'static str,
}

/// 每一处调用 `.owner()` 的 fn。多一处没登记、少一处已删掉，都红。
const OWNER_WALK_SITES: &[Site] = &[
    Site {
        file: "transform.rs",
        func: "world_matrix",
        climb: Climb::Transform,
        reads: &[],
        why: "根 → 自身逐级右乘 local_matrix；祖先身上只经 local_matrix 读 POS / ORI（见 transform_climb_reads_only_pos_and_ori）",
    },
    Site {
        file: "transform.rs",
        func: "matrix_relative_to",
        climb: Climb::Transform,
        reads: &[],
        why: "同 world_matrix，只折到指定祖先为止；比 refno 判「到了没」",
    },
    Site {
        file: "catalogue.rs",
        func: "has_hvac_ancestor",
        climb: Climb::NounOnly,
        reads: &[],
        why: "IHVCOM 的 CLIMBS(HVAC)：沿 owner 链只看 element_type 是不是 HVAC",
    },
    Site {
        file: "category.rs",
        func: "route_discipline",
        climb: Climb::NounOnly,
        reads: &[],
        why: "路由容器的学科：只看 owner 的 noun 是不是 HVAC",
    },
    Site {
        file: "handrail.rs",
        func: "owning_handrail",
        climb: Climb::NounOnly,
        reads: &[],
        why: "RAIL → RAILSE → HANDRA 连取两次 owner，只核 noun；拿到的 HANDRA 交给 first_rpath / path_points 读（见 ANCESTOR_CONSUMERS）",
    },
    Site {
        file: "elmodl.rs",
        func: "fixing_host_wall",
        climb: Climb::NounOnly,
        reads: &[],
        why: "FIXING 按 noun 上溯找宿主 WALL，只读 element_type",
    },
    Site {
        file: "elmodl.rs",
        func: "fixing_host_frame",
        climb: Climb::AboveUnit,
        reads: &["OWNER", "ZDIS"],
        why: "FIXING 自成模型单元（category::is_model_unit），它的实例标架却读 owner²=JLDATU 的 ZDIS（OWNER 是报错文案里的名字），再经 \
              wall_spine_points / world_matrix 读宿主 WALL——两者都在 FIXING 单元之上。整棵子树重算的根口径下 \
              （JLDATU 改了 → 祖先闭包命中 WALL 所在的 STRU / FRMW / SBFR 根 → 整棵重算）FIXING 跟着重生成；\
              ADR-080 单元级发布只重建 plan_update 点名的单元，FIXING 不在 JLDATU 的子树里 → 不点名 → 实例停在旧站位。\
              开 model_unit_level_publish 之前要把这条边补进单元规划（gen-model issue #39 / #37）",
    },
    Site {
        file: "cable_tray.rs",
        func: "placement",
        climb: Climb::WithinUnit,
        reads: &["PKDI", "POS", "POSREF", "ZDIS", "ZOFF"],
        why: "桥架件 POS / ORI 是伪属性，按 Core3D 现算：CTELE → CTMTRL → CWBRAN 无条件上溯两级，读 CWBRAN 的 WTHICK \
              （justification_offset）与它的 RPATH 成员（rpath::first_rpath）。CWBRAN 就是这些件的路由容器 / 单元根 \
              （increment::cable_tray_branch_of），读的都在单元之内。POS 字面量是报错文案里的名字",
    },
    Site {
        file: "rpath.rs",
        func: "resolve_anchor",
        climb: Climb::WithinUnit,
        reads: &["CLOSEF", "PKDI", "POS", "POSREF", "ZDIS"],
        why: "祖先读只有 container（CWBRAN，单元根）的 CLOSEF；anchor_point.owner() 取 RPATH、再 owner() 只比 refno 是否等于 \
              container。PKDI / ZDIS / POSREF 读在 POSREF 链的每一跳上——那是引用边不是 owner 边，跨单元时由 \
              reference_reads_guard 的 POSREF 文件表 + 设计引用级联（gen-model issue #32）覆盖；POS 是报错文案里的名字",
    },
    Site {
        file: "increment.rs",
        func: "collect_ancestors_inclusive",
        climb: Climb::NounOnly,
        reads: &[],
        why: "增量规划器自己的祖先闭包，只收 refno",
    },
    Site {
        file: "increment.rs",
        func: "nearest_unit",
        climb: Climb::NounOnly,
        reads: &[],
        why: "增量规划器的上卷：沿 owner 链看 noun 是不是模型单元 / 路由容器 / gb==1 容器",
    },
    Site {
        file: "increment.rs",
        func: "cable_tray_branch_of",
        climb: Climb::NounOnly,
        reads: &[],
        why: "桥架件上溯到 CWBRAN：只看 noun",
    },
    Site {
        file: "increment.rs",
        func: "implicit_frame_host_of",
        climb: Climb::NounOnly,
        reads: &[],
        why: "现算标架的输入上溯到宿主（GENSEC / SCTN / STWALL）：只看 noun，再看宿主名下有没有 JLDATU / FITT",
    },
    Site {
        file: "transform.rs",
        func: "implicit_frame",
        climb: Climb::NounOnly,
        reads: &[],
        why: "判一个元素的标架是不是由属主现算：FITT 取 owner 只核 noun 是不是 SCTN / STWALL；拿到的宿主交给 \
              section_fitting_placement 读（见 ANCESTOR_CONSUMERS）。JLDATU / PLDATU 那支转给 datum::host_and_joint",
    },
    Site {
        file: "datum.rs",
        func: "host_and_joint",
        climb: Climb::NounOnly,
        reads: &[],
        why: "PLDATU → JLDATU → GENSEC 连取 owner，只核 noun；拿到的宿主 GENSEC 与 JLDATU 交给 datum::solve 读\
              （见 ANCESTOR_CONSUMERS）",
    },
    Site {
        file: "datum.rs",
        func: "point_on_host",
        climb: Climb::AboveUnit,
        reads: &["POS"],
        why: "JLDATU.HPREF 指的 POINSP：取它的 owner（SPINE）与 owner²（只比 refno 是不是宿主 GENSEC），读 POINSP 的 POS \
              并经 local_matrix 折 SPINE 的 POS / ORI。POINSP 在宿主 GENSEC 的 SPINE 下，读者是挂在 datum 下、自成单元的 \
              FIXING 及其 TMPL 件——在它们的单元之上。根口径下（POINSP 改了 → 祖先闭包命中 GENSEC 所在的 FRMW / STRU 根 \
              → 整棵重算）覆盖；ADR-080 单元级发布下由 increment::implicit_frame_host_of 的整棵 GENSEC 子树级联覆盖——\
              与 fixing_host_frame 那条（gen-model issue #39）同型，这一条的单元规划已经补上",
    },
];

/// 接过一个祖先元素去读它的函数：`(文件名, fn 名, 一共读到的属性名, 理由)`。
/// 这些 fn 自己不调 `.owner()`，但它们读的对象是别处上溯拿到的祖先。
const ANCESTOR_CONSUMERS: &[(&str, &str, &[&str], &str)] = &[
    (
        "handrail.rs",
        "first_rpath",
        &[],
        "HANDRA（RAIL 的 owner²，同一扶手单元）的第一个 RPATH 成员：只看成员 noun",
    ),
    (
        "handrail.rs",
        "path_points",
        &["FRAD", "POS"],
        "读 HANDRA 名下 RPATH 的 POINTR 位置与倒角：扶手单元内部的成员，单元整体重建覆盖",
    ),
    (
        "rpath.rs",
        "first_rpath",
        &[],
        "CWBRAN（桥架件的 owner²，同一路由单元）的第一个 RPATH 成员：只看成员 noun",
    ),
    (
        "cable_tray.rs",
        "justification_offset",
        &["WJUSL", "WTHICK", "XOFF"],
        "桥架件的对正偏移读 CWBRAN.WTHICK：CWBRAN 是这些件的单元根，单元内",
    ),
    (
        "datum.rs",
        "solve",
        &[
            "BANG", "HPREF", "JUSL", "LMIRR", "PKDI", "POSL", "YDIR", "ZDIS",
        ],
        "GENSEC 名下 JLDATU / PLDATU 的现算标架：读 datum 自己（PLDATU 的 POSL / YDIR）、它的 JLDATU（ZDIS / PKDI / \
         HPREF）与宿主 GENSEC（BANG / LMIRR / JUSL，SPRE 经 section_catalogue、SPINE 经 spine_path）。读者是 datum 下的 \
         FIXING 与 TMPL 件，这些都在它们的单元之上：根口径下整棵重算覆盖；单元级规划由 increment::implicit_frame_host_of \
         把整棵 GENSEC 子树级联进计划覆盖（datum 输入改了、宿主 GENSEC 或 SPINE 路径改了都命中）",
    ),
    (
        "section.rs",
        "section_fitting_placement",
        &[
            "BANG", "DELP", "DX", "DY", "JUSL", "PKEY", "POSE", "POSL", "POSS", "PSTR", "ZDIS",
        ],
        "SCTN / STWALL 名下 FITT 的现算标架：读宿主（POSS / POSE / BANG / JUSL，SPRE 目录的 PSTR 线 PKEY / DX / DY）与 FITT \
         自己（POSL / ZDIS / BANG / DELP）。SCTN 名下的 FITT 本就并进 SCTN 单元（单元内）；STWALL 名下 FITT 模板里的件自成单元，\
         宿主在它们之上：根口径下整棵重算覆盖；单元级规划由 increment::implicit_frame_host_of 把整棵宿主子树级联进计划覆盖",
    ),
    (
        "elmodl.rs",
        "wall_spine_points",
        &["POS"],
        "宿主 WALL 的 SPINE 成员点位：被 fixing_host_frame 用在 FIXING 单元之上（AboveUnit，理由见那一条）；\
         WALL 自己的生成也用它，那一侧在单元内",
    ),
];

// ---------------------------------------------------------------- 源码扫描

/// `src` 递归，跳过 `src/bin`（与 attribute_reads_guard 同口径）。
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

fn strip_tests(source: &str) -> &str {
    match source.find(TEST_MARKER) {
        Some(cut) => &source[..cut],
        None => source,
    }
}

/// 把注释的内容换成空格（长度不变、位置对齐）；`blank_strings` 为真时连字串 / 字符字面量的
/// **内容**也换掉，只留结构——找 `fn`、配大括号、找 `.owner()` 都在那份上做，不会被文案里的
/// 括号和引号带偏；提取字面量则用只去注释的那份：注释里提到的名字不算读，报错文案里的算。
fn mask_with(source: &str, blank_strings: bool) -> String {
    let bytes = source.as_bytes();
    let mut out = bytes.to_vec();
    let mut i = 0;
    let blank = |out: &mut Vec<u8>, from: usize, to: usize| {
        for b in &mut out[from..to] {
            if *b != b'\n' {
                *b = b' ';
            }
        }
    };
    while i < bytes.len() {
        match bytes[i] {
            b'/' if bytes.get(i + 1) == Some(&b'/') => {
                let end = source[i..].find('\n').map_or(bytes.len(), |o| i + o);
                blank(&mut out, i, end);
                i = end;
            }
            b'/' if bytes.get(i + 1) == Some(&b'*') => {
                let end = source[i + 2..]
                    .find("*/")
                    .map_or(bytes.len(), |o| i + 2 + o + 2);
                blank(&mut out, i, end);
                i = end;
            }
            // 原始字串 `r"…"` / `r#"…"#`：收尾是 `"` 加同样多的 `#`。
            b'r' if (i == 0 || !is_ident(bytes[i - 1]))
                && bytes[i + 1..].iter().take_while(|b| **b == b'#').count() + i + 1
                    < bytes.len()
                && bytes[i + 1 + bytes[i + 1..].iter().take_while(|b| **b == b'#').count()]
                    == b'"' =>
            {
                let hashes = bytes[i + 1..].iter().take_while(|b| **b == b'#').count();
                let open = i + 1 + hashes;
                let closer = format!("\"{}", "#".repeat(hashes));
                let end = source[open + 1..]
                    .find(&closer)
                    .map_or(bytes.len(), |o| open + 1 + o + closer.len());
                if blank_strings {
                    blank(
                        &mut out,
                        open + 1,
                        end.saturating_sub(closer.len()).max(open + 1),
                    );
                }
                i = end;
            }
            b'"' => {
                // 普通字串：找未转义的收尾引号。
                let mut j = i + 1;
                while j < bytes.len() {
                    if bytes[j] == b'\\' {
                        j += 2;
                        continue;
                    }
                    if bytes[j] == b'"' {
                        break;
                    }
                    j += 1;
                }
                let end = (j + 1).min(bytes.len());
                if blank_strings {
                    blank(&mut out, i + 1, end.saturating_sub(1).max(i + 1));
                }
                i = end;
            }
            b'\'' => {
                // 字符字面量 `'x'` / `'\n'`；生命周期 `'a` 不是。
                if bytes.get(i + 2) == Some(&b'\'') {
                    if blank_strings {
                        blank(&mut out, i + 1, i + 2);
                    }
                    i += 3;
                } else if bytes.get(i + 1) == Some(&b'\\') && bytes.get(i + 3) == Some(&b'\'') {
                    if blank_strings {
                        blank(&mut out, i + 1, i + 3);
                    }
                    i += 4;
                } else {
                    i += 1;
                }
            }
            _ => i += 1,
        }
    }
    String::from_utf8(out).expect("mask keeps utf-8 (only ascii bytes are replaced)")
}

/// 只留结构（注释、字串、字符字面量内容全部换成空格）。
fn mask(source: &str) -> String {
    mask_with(source, true)
}

/// 只去注释，字串保留——提字面量用。
fn strip_comments(source: &str) -> String {
    mask_with(source, false)
}

/// 一个 fn：名字与函数体在源码里的区间（含大括号）。
#[derive(Debug, Clone)]
struct Func {
    name: String,
    body: std::ops::Range<usize>,
}

fn is_ident(b: u8) -> bool {
    b.is_ascii_alphanumeric() || b == b'_'
}

/// 在 mask 过的源码里找每个 `fn name(...) ... {` 与它的函数体。
fn functions(masked: &str) -> Vec<Func> {
    let bytes = masked.as_bytes();
    let mut out = Vec::new();
    let mut from = 0;
    while let Some(at) = masked[from..].find("fn ") {
        let start = from + at;
        from = start + 3;
        if start > 0 && is_ident(bytes[start - 1]) {
            continue;
        }
        let mut j = start + 3;
        while j < bytes.len() && bytes[j] == b' ' {
            j += 1;
        }
        let name_start = j;
        while j < bytes.len() && is_ident(bytes[j]) {
            j += 1;
        }
        if j == name_start {
            continue;
        }
        let name = masked[name_start..j].to_string();
        // 签名到函数体：第一个 `{`（trait 里的 `fn x();` 声明碰到 `;` 就不是定义）。
        let mut k = j;
        let mut open = None;
        while k < bytes.len() {
            match bytes[k] {
                b'{' => {
                    open = Some(k);
                    break;
                }
                b';' => break,
                _ => k += 1,
            }
        }
        let Some(open) = open else { continue };
        let mut depth = 0usize;
        let mut close = None;
        for (idx, b) in bytes.iter().enumerate().skip(open) {
            match b {
                b'{' => depth += 1,
                b'}' => {
                    depth -= 1;
                    if depth == 0 {
                        close = Some(idx);
                        break;
                    }
                }
                _ => {}
            }
        }
        let Some(close) = close else { continue };
        out.push(Func {
            name,
            body: open..close + 1,
        });
    }
    out
}

/// 包住 `pos` 的最内层 fn。
fn enclosing<'a>(funcs: &'a [Func], pos: usize) -> Option<&'a Func> {
    funcs
        .iter()
        .filter(|f| f.body.contains(&pos))
        .min_by_key(|f| f.body.len())
}

/// 与 attribute_reads_guard 同口径的大写字面量 `"[A-Z][A-Z0-9]{1,7}"`。
fn uppercase_literals(source: &str) -> BTreeSet<String> {
    let bytes = source.as_bytes();
    let mut out = BTreeSet::new();
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
            i = end + 1;
        } else {
            i += 1;
        }
    }
    out
}

struct Scan {
    /// `(文件名, fn 名)` → 该 fn 函数体里读到的属性名（大写字面量 ∩ 读取集合，去掉 noun 名）。
    reads_by_fn: BTreeMap<(String, String), BTreeSet<String>>,
    /// 调过 `.owner()` 的 `(文件名, fn 名)`。
    owner_walks: BTreeSet<(String, String)>,
    /// 每个文件的函数体原文，给位姿那条路的顺序检查用。
    bodies: BTreeMap<(String, String), String>,
}

fn scan_sources() -> Scan {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    let mut files = Vec::new();
    rust_sources(&root, &root, &mut files);
    files.sort();

    let read_set: BTreeSet<&str> = e3d_model::attribute_reads::attribute_read_set_file()
        .names
        .iter()
        .map(String::as_str)
        .collect();
    let nouns: BTreeSet<&str> = e3d_model::category::dictionary_noun_names().collect();

    let mut scan = Scan {
        reads_by_fn: BTreeMap::new(),
        owner_walks: BTreeSet::new(),
        bodies: BTreeMap::new(),
    };
    for path in files {
        let file = path.file_name().unwrap().to_str().unwrap().to_string();
        let source = fs::read_to_string(&path).expect("read source");
        let source = strip_tests(&source);
        let masked = mask(source);
        let commentless = strip_comments(source);
        assert_eq!(masked.len(), source.len(), "{file}: mask 改变了长度");
        assert_eq!(
            commentless.len(),
            source.len(),
            "{file}: strip_comments 改变了长度"
        );
        let funcs = functions(&masked);

        let mut from = 0;
        while let Some(at) = masked[from..].find(".owner()") {
            let pos = from + at;
            from = pos + ".owner()".len();
            let func = enclosing(&funcs, pos)
                .unwrap_or_else(|| panic!("{file}: 第 {} 字节的 .owner() 不在任何 fn 里", pos));
            scan.owner_walks.insert((file.clone(), func.name.clone()));
        }
        for func in &funcs {
            let body = &commentless[func.body.clone()];
            let attrs: BTreeSet<String> = uppercase_literals(body)
                .into_iter()
                .filter(|l| read_set.contains(l.as_str()) && !nouns.contains(l.as_str()))
                .collect();
            let key = (file.clone(), func.name.clone());
            // 同名 fn（不同 impl 里的同名方法）合并读到的名字。
            scan.reads_by_fn
                .entry(key.clone())
                .or_default()
                .extend(attrs);
            scan.bodies
                .entry(key)
                .and_modify(|joined| {
                    joined.push('\n');
                    joined.push_str(body);
                })
                .or_insert_with(|| body.to_string());
        }
    }
    scan
}

fn fmt_set(set: &BTreeSet<String>) -> String {
    set.iter().cloned().collect::<Vec<_>>().join(", ")
}

// ---------------------------------------------------------------- 断言

/// 每一处 `.owner()` 上溯都登记在案，且那个 fn 只读它登记的属性名。
#[test]
fn every_owner_walk_is_registered_and_reads_only_what_it_declares() {
    let scan = scan_sources();

    let registered: BTreeSet<(String, String)> = OWNER_WALK_SITES
        .iter()
        .map(|s| (s.file.to_string(), s.func.to_string()))
        .collect();
    assert_eq!(
        registered.len(),
        OWNER_WALK_SITES.len(),
        "OWNER_WALK_SITES 里有重复登记"
    );

    let unregistered: Vec<String> = scan
        .owner_walks
        .difference(&registered)
        .map(|(f, n)| {
            format!(
                "{f}::{n} 读到 [{}]",
                fmt_set(
                    scan.reads_by_fn
                        .get(&(f.clone(), n.clone()))
                        .unwrap_or(&BTreeSet::new())
                )
            )
        })
        .collect();
    assert!(
        unregistered.is_empty(),
        "这些 fn 沿 owner 链上溯了，却没在 OWNER_WALK_SITES 登记它读的是谁、增量怎么覆盖：{unregistered:?}。\
         祖先属性变了不会进任何根的闭包（只有 POS / ORI 走锚点）——先回答这一处读的是不是根之上的东西，再登记。"
    );
    let stale: Vec<String> = registered
        .difference(&scan.owner_walks)
        .map(|(f, n)| format!("{f}::{n}"))
        .collect();
    assert!(
        stale.is_empty(),
        "登记里这些 fn 已经不调 .owner() 了（或改名了），把它们从 OWNER_WALK_SITES 删掉：{stale:?}"
    );

    for site in OWNER_WALK_SITES {
        assert!(
            !site.why.trim().is_empty(),
            "{}::{} 的理由不能为空",
            site.file,
            site.func
        );
        let key = (site.file.to_string(), site.func.to_string());
        let actual = scan.reads_by_fn.get(&key).cloned().unwrap_or_default();
        let declared: BTreeSet<String> = site.reads.iter().map(|s| s.to_string()).collect();
        let extra: Vec<&String> = actual.difference(&declared).collect();
        assert!(
            extra.is_empty(),
            "{}::{}（{:?}）读到了没登记的属性 {extra:?}。若它是从祖先身上读的：POS / ORI 之外的祖先属性变了\
             不会让这个单元重算——要么改成不读祖先，要么登记进 reads 并在 why 里写清增量怎么覆盖",
            site.file,
            site.func,
            site.climb
        );
        let dead: Vec<&String> = declared.difference(&actual).collect();
        assert!(
            dead.is_empty(),
            "{}::{} 登记的 {dead:?} 源码里已经不读了，从 reads 里删掉",
            site.file,
            site.func
        );
        match site.climb {
            Climb::Transform | Climb::NounOnly => assert!(
                actual.is_empty(),
                "{}::{} 标成 {:?}，函数体里却直接读了属性 {actual:?}",
                site.file,
                site.func,
                site.climb
            ),
            Climb::AboveUnit => assert!(
                site.why.contains("ADR-080") && site.why.contains("issue #"),
                "{}::{} 是在案缺口，why 里必须点名 ADR-080（单元级发布不覆盖）与跟踪的 issue",
                site.file,
                site.func
            ),
            Climb::WithinUnit => {}
        }
    }
}

/// 接过祖先去读的函数同样逐条登记，只读登记的名字；登记的 fn 必须还存在。
#[test]
fn ancestor_consumers_read_only_declared_attributes() {
    let scan = scan_sources();
    let mut seen = BTreeSet::new();
    for (file, func, reads, why) in ANCESTOR_CONSUMERS {
        assert!(!why.trim().is_empty(), "{file}::{func} 的理由不能为空");
        assert!(seen.insert((*file, *func)), "{file}::{func} 重复登记");
        let key = (file.to_string(), func.to_string());
        let actual = scan.reads_by_fn.get(&key).unwrap_or_else(|| {
            panic!("ANCESTOR_CONSUMERS 登记的 {file}::{func} 在源码里找不到（删了或改名了）")
        });
        let declared: BTreeSet<String> = reads.iter().map(|s| s.to_string()).collect();
        let extra: Vec<&String> = actual.difference(&declared).collect();
        assert!(
            extra.is_empty(),
            "{file}::{func} 读到了没登记的属性 {extra:?}：它读的对象是别处上溯拿到的祖先，多读一个名字就多一条\
             祖先属性 → 几何的边，登记进 reads 并在 why 里写清增量怎么覆盖"
        );
        let dead: Vec<&String> = declared.difference(actual).collect();
        assert!(
            dead.is_empty(),
            "{file}::{func} 登记的 {dead:?} 源码里已经不读了"
        );
    }
}

/// 位姿那条路：祖先身上经 `local_matrix` / `local_placement` 只读 `POS` / `ORI`；
/// 桥架伪属性分支（读 PKDI / ZDIS / POSREF…）必须挡在 `is_cable_tray_element` 之后——
/// 桥架件只出现在 CWBRAN 之下、从不在任何单元根之上，所以不构成「根之上的属性读」。
#[test]
fn transform_climb_reads_only_pos_and_ori() {
    let scan = scan_sources();
    let allowed: BTreeSet<String> = ["POS", "ORI"].iter().map(|s| s.to_string()).collect();
    for func in ["local_matrix", "local_placement"] {
        let key = ("transform.rs".to_string(), func.to_string());
        let actual = scan
            .reads_by_fn
            .get(&key)
            .unwrap_or_else(|| panic!("transform.rs::{func} 找不到——位姿读点搬家了，更新本守卫"));
        let extra: Vec<&String> = actual.difference(&allowed).collect();
        assert!(
            extra.is_empty(),
            "transform.rs::{func} 读了 POS / ORI 之外的属性 {extra:?}：世界矩阵沿 owner 链对每个祖先都调它，\
             多读一个名字 = 根之上那个属性变了却没人重算"
        );
    }
    let body = scan
        .bodies
        .get(&("transform.rs".to_string(), "local_matrix".to_string()))
        .expect("local_matrix body");
    let gate = body
        .find("is_cable_tray_element(")
        .expect("local_matrix 必须先问 is_cable_tray_element");
    let branch = body
        .find("placement_matrix(")
        .expect("local_matrix 的桥架分支必须经 placement_matrix");
    assert!(
        gate < branch,
        "local_matrix 里桥架伪属性分支必须挡在 is_cable_tray_element 之后"
    );
}

/// 扫描器自检：mask 不吃结构、fn 定位与字面量提取在已知形状上给出已知答案。
#[test]
fn the_scanner_finds_functions_and_literals_in_a_known_shape() {
    let source = r#"
/// doc with "quotes" and { braces }
fn outer(el: &DbElement) -> Result<(), E> {
    let s = "text with } brace"; // comment with { brace
    let c = '{';
    let owner = el.owner()?; // "POS" in a comment must not count
    let _ = owner.get_double("LEVE")?;
    Ok(())
}
fn other() { let _ = "ORI"; }
"#;
    let masked = mask(source);
    assert_eq!(masked.len(), source.len());
    let funcs = functions(&masked);
    assert_eq!(
        funcs.iter().map(|f| f.name.as_str()).collect::<Vec<_>>(),
        ["outer", "other"]
    );
    let pos = masked
        .find(".owner()")
        .expect("owner call survives masking");
    assert_eq!(
        enclosing(&funcs, pos).map(|f| f.name.as_str()),
        Some("outer")
    );
    let commentless = strip_comments(source);
    assert_eq!(commentless.len(), source.len());
    let outer = &commentless[funcs[0].body.clone()];
    let literals = uppercase_literals(outer);
    assert!(literals.contains("LEVE"), "{literals:?}");
    // 注释里的 "POS" 不算读；字串（报错文案）里的名字才算。
    assert!(!literals.contains("POS"), "{literals:?}");
    assert!(uppercase_literals(&commentless[funcs[1].body.clone()]).contains("ORI"));
    assert!(!uppercase_literals(&commentless[funcs[1].body.clone()]).contains("LEVE"));

    // 原始字串里的引号与括号也不许带偏结构。
    let raw = "fn a() { let s = r#\"has \" quote and } brace\"#; s.owner() }\nfn b() {}";
    let funcs = functions(&mask(raw));
    assert_eq!(
        funcs.iter().map(|f| f.name.as_str()).collect::<Vec<_>>(),
        ["a", "b"]
    );
    assert_eq!(
        enclosing(&funcs, raw.find(".owner()").unwrap()).map(|f| f.name.as_str()),
        Some("a")
    );
}

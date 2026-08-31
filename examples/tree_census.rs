//! 库内元素普查：谁是根、谁从根走得到、走不到的挂在哪。
//!
//! `gen_ams` 只走 DFS，它看不见「索引里有、树上却够不着」的元素。这个探针从
//! 索引侧（`scan_elements`）拿全集，再按 owner 链算每个元素的终极祖先，
//! 两边一对就知道 DFS 少走了哪一片。
//!
//! ```text
//! cargo run --release --example tree_census -- --attlib <attlib.dat> \
//!     --db <ams1112_0001> --template <desvir.dat> [--db-type DESI] [--sesno N]
//! ```

use std::collections::{BTreeMap, HashMap, HashSet};
use std::path::PathBuf;
use std::sync::Arc;

use anyhow::{Context, bail};
use e3d_io::db_element::{DbFilePin, DbSet};
use e3d_io::engine::{ReadOnlyEngine, ScanTier};
use e3d_io::refno::RefNo;

fn main() -> anyhow::Result<()> {
    let mut attlib: Option<PathBuf> = None;
    let mut db_file: Option<PathBuf> = None;
    let mut template: Option<PathBuf> = None;
    let mut db_type = "DESI".to_string();
    let mut sesno: Option<u32> = None;

    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--attlib" => attlib = Some(PathBuf::from(value()?)),
            "--db" => db_file = Some(PathBuf::from(value()?)),
            "--template" => template = Some(PathBuf::from(value()?)),
            "--db-type" => db_type = value()?,
            "--sesno" => sesno = Some(value()?.parse()?),
            other => bail!("不认识的参数 {other}"),
        }
    }
    let attlib = attlib.context("缺 --attlib")?;
    let db_file = db_file.context("缺 --db")?;
    let template = template.context("缺 --template")?;

    let set = Arc::new(DbSet::with_attlib_file(&attlib)?);
    let dbno = set.add_db(DbFilePin {
        file: db_file.clone(),
        template,
        db_type: Some(db_type),
        sesno,
    })?;
    let pinned = set.pinned_sesno(dbno);
    println!("dbno={dbno} 钉在会话 {pinned:?}");

    // 索引侧全集：refno → owner。
    let mut engine = match pinned {
        Some(s) => ReadOnlyEngine::open_at(&db_file, s),
        None => ReadOnlyEngine::open(&db_file),
    }?;
    let mut owner_of: HashMap<RefNo, RefNo> = HashMap::new();
    let mut order: Vec<RefNo> = Vec::new();
    for scanned in engine.scan_elements(ScanTier::Header)? {
        let scanned = scanned?;
        owner_of.insert(scanned.refno, scanned.owner);
        order.push(scanned.refno);
    }
    println!("索引元素总数 = {}", order.len());

    // 终极祖先（owner 走不动为止）。自指、指向索引外、无效，都算走不动。
    let mut terminal: HashMap<RefNo, RefNo> = HashMap::new();
    for &start in &order {
        if terminal.contains_key(&start) {
            continue;
        }
        let mut path = Vec::new();
        let mut cursor = start;
        let end = loop {
            if let Some(&known) = terminal.get(&cursor) {
                break known;
            }
            path.push(cursor);
            let Some(&owner) = owner_of.get(&cursor) else {
                break cursor;
            };
            if owner == cursor || !owner.is_valid() || !owner_of.contains_key(&owner) {
                break cursor;
            }
            if path.len() > 256 {
                break cursor;
            }
            cursor = owner;
        };
        for node in path {
            terminal.insert(node, end);
        }
    }

    let mut by_terminal: BTreeMap<String, usize> = BTreeMap::new();
    let mut terminals: HashSet<RefNo> = HashSet::new();
    for &refno in &order {
        let end = terminal[&refno];
        terminals.insert(end);
        *by_terminal.entry(end.to_string()).or_insert(0) += 1;
    }

    println!("\n=== 终极祖先（子树规模倒序）===");
    let mut rows: Vec<(&String, &usize)> = by_terminal.iter().collect();
    rows.sort_by(|a, b| b.1.cmp(a.1));
    for (refno_text, count) in rows.iter().take(30) {
        let refno = terminals
            .iter()
            .find(|r| r.to_string() == **refno_text)
            .copied()
            .expect("terminal set holds it");
        let element = set.element(refno);
        let noun = element.element_type().unwrap_or_else(|e| format!("<{e}>"));
        let name = element.stored_name().ok().flatten().unwrap_or_default();
        let owner = owner_of.get(&refno).copied();
        let owner_indexed = owner.map(|o| owner_of.contains_key(&o)).unwrap_or(false);
        println!(
            "{refno_text:>24}  {count:>6} 元素  noun={noun:<8} name={name:<24} \
             owner={owner:?} owner_in_index={owner_indexed}"
        );
    }

    // noun 分布 + 父子 noun 边：几何元素到底挂在谁下面。
    let mut noun_of: HashMap<RefNo, String> = HashMap::with_capacity(order.len());
    for &refno in &order {
        let noun = set
            .element(refno)
            .element_type()
            .unwrap_or_else(|e| format!("<{e}>"));
        noun_of.insert(refno, noun);
    }
    let mut noun_counts: BTreeMap<&str, usize> = BTreeMap::new();
    let mut edges: BTreeMap<(&str, &str), usize> = BTreeMap::new();
    for &refno in &order {
        let child = noun_of[&refno].as_str();
        *noun_counts.entry(child).or_insert(0) += 1;
        let parent = owner_of
            .get(&refno)
            .and_then(|o| noun_of.get(o))
            .map(String::as_str)
            .unwrap_or("<无 owner>");
        *edges.entry((parent, child)).or_insert(0) += 1;
    }

    println!("\n=== noun 分布 ===");
    let mut nouns: Vec<(&&str, &usize)> = noun_counts.iter().collect();
    nouns.sort_by(|a, b| b.1.cmp(a.1));
    for (noun, count) in nouns {
        println!("{noun:<10} {count:>6}");
    }

    println!("\n=== 父 noun → 子 noun ===");
    let mut edge_rows: Vec<(&(&str, &str), &usize)> = edges.iter().collect();
    edge_rows.sort_by(|a, b| b.1.cmp(a.1));
    for ((parent, child), count) in edge_rows {
        println!("{parent:<10} → {child:<10} {count:>6}");
    }

    // 成员表侧的对照：从每个终极祖先 DFS，看能走到几个。
    println!("\n=== 从终极祖先 DFS 的可达数（成员表口径）===");
    for (refno_text, index_count) in rows.iter().take(30) {
        let refno = terminals
            .iter()
            .find(|r| r.to_string() == **refno_text)
            .copied()
            .expect("terminal set holds it");
        let mut seen = 0usize;
        let mut stack = vec![set.element(refno)];
        let mut broke = None;
        while let Some(element) = stack.pop() {
            seen += 1;
            if seen > 200_000 {
                broke = Some("超过 20 万，疑似成环".to_string());
                break;
            }
            match element.members() {
                Ok(members) => stack.extend(members),
                Err(error) => {
                    broke = Some(format!("{} 读成员失败: {error}", element.refno()));
                    break;
                }
            }
        }
        let flag = if seen == **index_count {
            ""
        } else {
            "  ← 对不上"
        };
        println!("{refno_text:>24}  索引 {index_count:>6} / DFS {seen:>6}{flag} {broke:?}");
    }

    Ok(())
}

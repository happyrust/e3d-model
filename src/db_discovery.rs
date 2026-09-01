//! 项目目录扫描：把 e3d-io 留的那条跨库缝（[`DbFileResolver`]）用最朴素的方式接上。
//!
//! e3d-io 的立场是「没注册的库报错，不猜」（`e3d_io::db_element` 模块头），发现依赖
//! 数据是消费方的活。这里的实现只回答一个问题：**给定目录里躺着哪些 dabacon 库文件**
//! ——逐文件读页 0 描述符，拿文件自报的库号（`db_mark`，与 `RefNo::dbno` 同口径）与
//! 库类型（+0x20 的 db1 哈希），按库号建表。引用跳到没开过的库时
//! [`e3d_io::db_element::DbSet`] 问一次 [`DirectoryDbResolver::resolve`]，模板文件按
//! 库类型经 [`template_file_for`] 从模板目录派生。
//!
//! 这**不是** SYS 库 MDB 成员表那条正统项目发现（core.dll 的 CURD 语义，即
//! `db_element` 模块头说的 S4）：目录里躺着的库不等于工程挂载的库。它给的是
//! `gen_ams` 这类直读探针「一个 `--project-dir` 顶一串 `--catalogue`」的便利，
//! 选哪套目录仍然是调用方的显式决定。
//!
//! 几条边界：
//!
//! - **只扫给定目录本层，不递归。** 备份副本、嵌套工程一律不猜；要多个目录就多给几个。
//! - 多文件库的 extent 文件（`ext_no > 1`）登记进 [`discovered`] 但不外交：
//!   `ReadOnlyEngine` 打开主文件（`ext_no == 1`）时会按 `<stem>_<ext:04>`
//!   命名约定自己附挂兄弟文件（见 `e3d_io::page::io` 的 `ExtentSet`）。
//! - 同号多候选按 **dabacon 命名约定认正身**：`<stem>_<数字>` 且数字等于文件自报
//!   `ext_no` 的是正身，唯一正身进表，落选的进 [`skipped`] 诊断。真实工程目录里
//!   就躺着陈年副本与手工备份（ams000 实测：`ams7355`（2015-04）挨着
//!   `ams7355_0001`（2015-09）、`ams8000_0001.codex-before-*`），硬错误会把整次
//!   扫描搞死在一个从不被引用的库上。认不出唯一正身的组**整组不外交**——真被
//!   引用到时 `DbNotRegistered` 会出声，开场诊断里躺着原因；这仍然是不猜，只是
//!   把「拒绝」从扫描期挪到了引用点。
//! - 认不出类型（+0x20 反哈希不出名字）的库文件进 [`skipped`] 诊断、不进表：
//!   类型派生不出模板文件，静默配一个错模板读出来的是别人的 schema。
//!
//! [`discovered`]: DirectoryDbResolver::discovered
//! [`skipped`]: DirectoryDbResolver::skipped

use std::collections::{BTreeMap, HashMap, HashSet};
use std::io::{Read as _, Seek as _, SeekFrom};
use std::path::{Path, PathBuf};

use e3d_io::db_element::{DbFilePin, DbFileResolver, template_file_for};
use e3d_io::meta::{DbDescriptor, FileInfo};

/// 目录里认出来的一个库文件（含 extent 文件）。
#[derive(Debug, Clone)]
pub struct DiscoveredDb {
    pub file: PathBuf,
    /// 文件自报的库号（页 0 `db_mark`，与 `RefNo::dbno` 同口径）。
    pub dbno: u32,
    /// 文件自报的 extent 号；`<= 1` 是主文件。
    pub ext_no: u32,
    /// 库类型名（`DESI`/`CATA`/…），页 0 +0x20 反哈希。
    pub db_type: String,
}

/// 长得像库却没进表的文件与原因。诊断口径，不是错误——一个锁住的文件、
/// 一个反哈希不出类型的库，都不该搞垮整次扫描，但也不许无声消失。
#[derive(Debug, Clone)]
pub struct SkippedFile {
    pub file: PathBuf,
    pub reason: String,
}

#[derive(Debug, thiserror::Error)]
pub enum DiscoveryError {
    #[error("{} 不是目录", dir.display())]
    NotADirectory { dir: PathBuf },
    #[error("扫描 {}: {source}", dir.display())]
    ReadDir {
        dir: PathBuf,
        source: std::io::Error,
    },
}

/// 按目录扫描建表的 [`DbFileResolver`]：dbno → 主文件 + 按类型派生的模板。
#[derive(Debug)]
pub struct DirectoryDbResolver {
    pins: HashMap<u32, DbFilePin>,
    discovered: Vec<DiscoveredDb>,
    skipped: Vec<SkippedFile>,
    non_db_files: usize,
}

impl DirectoryDbResolver {
    /// 扫一批目录，建 dbno → 主文件表。
    ///
    /// 模板路径按库类型从 `template_dir` 派生（`DESI` → `desvir.dat`），只派生
    /// 不查存在性：目录里常躺着永远不会被引用的库类型（SYST/DICT/…），它们缺
    /// 模板不该拦住扫描；真被引用到的库缺模板，开库时 `DbSet` 会带着完整路径报。
    pub fn scan(dirs: &[PathBuf], template_dir: &Path) -> Result<Self, DiscoveryError> {
        let mut discovered: Vec<DiscoveredDb> = Vec::new();
        let mut skipped: Vec<SkippedFile> = Vec::new();
        let mut non_db_files = 0usize;
        // 同一个文件经两个目录参数给进来只算一次，别让它伪装成重复主文件。
        let mut seen: HashSet<PathBuf> = HashSet::new();

        for dir in dirs {
            if !dir.is_dir() {
                return Err(DiscoveryError::NotADirectory { dir: dir.clone() });
            }
            let entries = std::fs::read_dir(dir).map_err(|source| DiscoveryError::ReadDir {
                dir: dir.clone(),
                source,
            })?;
            for entry in entries {
                let entry = entry.map_err(|source| DiscoveryError::ReadDir {
                    dir: dir.clone(),
                    source,
                })?;
                let path = entry.path();
                if !path.is_file() {
                    continue;
                }
                let identity = std::fs::canonicalize(&path).unwrap_or_else(|_| path.clone());
                if !seen.insert(identity) {
                    continue;
                }
                match probe_descriptor(&path) {
                    Err(error) => skipped.push(SkippedFile {
                        file: path,
                        reason: format!("读文件头失败: {error}"),
                    }),
                    Ok(None) => non_db_files += 1,
                    Ok(Some(descriptor)) => {
                        if descriptor.db_mark == 0 {
                            skipped.push(SkippedFile {
                                file: path,
                                reason: "页 0 自报库号 0".to_string(),
                            });
                            continue;
                        }
                        let Some(db_type) = descriptor.db_type_name() else {
                            skipped.push(SkippedFile {
                                file: path,
                                reason: format!(
                                    "页 0 +0x20 的类型字 0x{:08X} 反哈希不出库类型名，派生不出模板文件",
                                    descriptor.db_type_hash
                                ),
                            });
                            continue;
                        };
                        discovered.push(DiscoveredDb {
                            file: path,
                            dbno: descriptor.db_mark,
                            ext_no: descriptor.ext_no,
                            db_type,
                        });
                    }
                }
            }
        }

        // 稳定输出：库号、extent、路径。诊断和 summary 都靠这份顺序可复读。
        discovered.sort_by(|a, b| (a.dbno, a.ext_no, &a.file).cmp(&(b.dbno, b.ext_no, &b.file)));

        // 同号分组，按命名约定认正身。选择权在 resolver（`with_attlib_file_and_resolver`
        // 的文档把 selection 明确划给它），但这里的「选」只认文件自己的话：
        // 名字里的 extent 数字与页 0 自报的 ext_no 对上才算正身。
        let mut by_dbno: BTreeMap<u32, Vec<&DiscoveredDb>> = BTreeMap::new();
        for db in &discovered {
            if primary_ext(db.ext_no) == 1 {
                by_dbno.entry(db.dbno).or_default().push(db);
            }
        }

        let mut pins: HashMap<u32, DbFilePin> = HashMap::new();
        for (dbno, group) in &by_dbno {
            let canonical: Vec<&&DiscoveredDb> = group
                .iter()
                .filter(|db| canonically_named(&db.file, primary_ext(db.ext_no)))
                .collect();
            let winner = match (canonical.len(), group.len()) {
                (1, _) => *canonical[0],
                (0, 1) => group[0],
                _ => {
                    // 分不出正身：整组进诊断、不外交。真被引用到时 DbNotRegistered
                    // 会出声，这几条诊断就是答案。
                    let candidates = group
                        .iter()
                        .map(|db| db.file.display().to_string())
                        .collect::<Vec<_>>()
                        .join("、");
                    for db in group {
                        skipped.push(SkippedFile {
                            file: db.file.clone(),
                            reason: format!(
                                "库 {dbno} 有 {} 个候选主文件（{candidates}），按命名约定分不出正身，整组不外交",
                                group.len()
                            ),
                        });
                    }
                    continue;
                }
            };
            for db in group {
                if !std::ptr::eq(*db, winner) {
                    skipped.push(SkippedFile {
                        file: db.file.clone(),
                        reason: format!(
                            "同号库 {dbno} 的非常规命名副本，让位给 {}",
                            winner.file.display()
                        ),
                    });
                }
            }
            let template = match template_file_for(template_dir, &winner.db_type) {
                Ok(template) => template,
                Err(error) => {
                    skipped.push(SkippedFile {
                        file: winner.file.clone(),
                        reason: format!("库类型 {} 派生不出模板文件名: {error}", winner.db_type),
                    });
                    continue;
                }
            };
            pins.insert(
                *dbno,
                DbFilePin {
                    file: winner.file.clone(),
                    template,
                    db_type: Some(winner.db_type.clone()),
                    sesno: None,
                },
            );
        }

        // 只见 extent 文件、没扫到主文件的库进诊断：它 resolve 不出来，
        // 之后报 DbNotRegistered 时这条就是答案。（主文件组分不出正身的
        // 已经整组进过诊断，这里不再重复。）
        for db in &discovered {
            if db.ext_no > 1 && !pins.contains_key(&db.dbno) && !by_dbno.contains_key(&db.dbno) {
                skipped.push(SkippedFile {
                    file: db.file.clone(),
                    reason: format!(
                        "库 {} 只见 extent 文件（ext_no={}），没扫到主文件",
                        db.dbno, db.ext_no
                    ),
                });
            }
        }

        Ok(Self {
            pins,
            discovered,
            skipped,
            non_db_files,
        })
    }

    /// 全部认出来的库文件（含 extent 文件），按库号排。
    pub fn discovered(&self) -> &[DiscoveredDb] {
        &self.discovered
    }

    /// 长得像库却没进表的文件与原因。
    pub fn skipped(&self) -> &[SkippedFile] {
        &self.skipped
    }

    /// 可解析的库号，升序。
    pub fn dbnos(&self) -> Vec<u32> {
        let mut out: Vec<u32> = self.pins.keys().copied().collect();
        out.sort_unstable();
        out
    }

    /// 一行人话总结，给 CLI 开场打印。
    pub fn summary(&self) -> String {
        let mut by_type: BTreeMap<&str, usize> = BTreeMap::new();
        for pin in self.pins.values() {
            let db_type = pin.db_type.as_deref().unwrap_or("?");
            *by_type.entry(db_type).or_insert(0) += 1;
        }
        let types = by_type
            .iter()
            .map(|(db_type, count)| format!("{db_type}×{count}"))
            .collect::<Vec<_>>()
            .join(" ");
        format!(
            "project-dir 扫到 {} 个库（{}），{} 个可疑文件进诊断，{} 个非库文件略过",
            self.pins.len(),
            types,
            self.skipped.len(),
            self.non_db_files
        )
    }
}

impl DbFileResolver for DirectoryDbResolver {
    fn resolve(&self, dbno: u32) -> Option<DbFilePin> {
        self.pins.get(&dbno).cloned()
    }
}

/// 描述符自报 0 号 extent 时按主文件算（`DbDescriptor::extent` 的同一条回退，
/// 普查里没有库真报 0）。
fn primary_ext(ext_no: u32) -> u32 {
    if ext_no == 0 { 1 } else { ext_no }
}

/// 文件名是否符合 dabacon 命名约定：`<stem>_<数字>` 且数字与文件自报 extent 号一致。
///
/// `ams7355_0001`（ext_no 1）是正身；`ams7355`（无后缀的陈年副本）、
/// `ams8000_0001.codex-before-move-20260724`（备份，后缀不是纯数字）都不是。
fn canonically_named(path: &Path, ext_no: u32) -> bool {
    let Some(name) = path.file_name().and_then(|name| name.to_str()) else {
        return false;
    };
    let Some((_, suffix)) = name.rsplit_once('_') else {
        return false;
    };
    !suffix.is_empty()
        && suffix.bytes().all(|byte| byte.is_ascii_digit())
        && suffix.parse::<u32>() == Ok(ext_no)
}

/// 读一个文件的头两页判断它是不是 dabacon 库。
///
/// 判据两道：页 0 解析成描述符（页型 0、页大小合法），页 1 的签名字是
/// `FileInfo::SIGNATURE`——后者按 `e3d_io::meta` 的普查是把库文件和 `*dflts`
/// 目录里无扩展名文本文件分开的唯一字段。`Ok(None)` = 不是库文件；
/// `Err` = 文件本身读不了（锁住、无权限），交给调用方进诊断。
fn probe_descriptor(path: &Path) -> std::io::Result<Option<DbDescriptor>> {
    let eof_is_not_a_db = |error: std::io::Error| {
        if error.kind() == std::io::ErrorKind::UnexpectedEof {
            Ok(None)
        } else {
            Err(error)
        }
    };

    let mut file = std::fs::File::open(path)?;
    let mut header = [0u8; 0x40];
    if let Err(error) = file.read_exact(&mut header) {
        return eof_is_not_a_db(error);
    }
    let Ok(descriptor) = DbDescriptor::parse(&header) else {
        return Ok(None);
    };

    let mut page1_head = [0u8; 8];
    file.seek(SeekFrom::Start(u64::from(descriptor.page_size)))?;
    if let Err(error) = file.read_exact(&mut page1_head) {
        return eof_is_not_a_db(error);
    }
    let signature = u32::from_be_bytes(page1_head[4..8].try_into().expect("8-byte buffer"));
    if signature != FileInfo::SIGNATURE {
        return Ok(None);
    }
    Ok(Some(descriptor))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 一页 512 词（2048 字节）的字表，尾部补零。
    fn page(words: &[u32]) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(2048);
        for word in words {
            bytes.extend_from_slice(&word.to_be_bytes());
        }
        bytes.resize(2048, 0);
        bytes
    }

    /// 一个最小可信的 dabacon 文件：页 0 描述符 + 页 1 签名。
    /// 字表布局与 `e3d_io::meta` 的 `ams8000_page0` 同源。
    fn write_db(dir: &Path, name: &str, dbno: u32, ext_no: u32, type_hash: u32) -> PathBuf {
        let mut bytes = page(&[
            0,
            2,
            dbno,
            0,
            0,
            0,
            0,
            0,
            type_hash,
            0,
            4,
            ext_no,
            4,
            512,
            dbno + 0x2000,
        ]);
        bytes.extend(page(&[1, u32::MAX, 2]));
        let path = dir.join(name);
        std::fs::write(&path, bytes).unwrap();
        path
    }

    struct TempDir(PathBuf);

    impl TempDir {
        fn new(tag: &str) -> Self {
            let dir = std::env::temp_dir().join(format!(
                "e3d-model-db-discovery-{tag}-{}",
                std::process::id()
            ));
            // 上一跑的残留先清掉，别让旧文件混进断言。
            let _ = std::fs::remove_dir_all(&dir);
            std::fs::create_dir_all(&dir).unwrap();
            Self(dir)
        }

        fn path(&self) -> &Path {
            &self.0
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    #[test]
    fn scan_builds_the_table_and_derives_templates_by_type() {
        let dir = TempDir::new("basic");
        let desi = write_db(
            dir.path(),
            "ams8000_0001",
            8000,
            1,
            e3d_attlib::db1_hash("DESI"),
        );
        write_db(
            dir.path(),
            "ams8000_0002",
            8000,
            2,
            e3d_attlib::db1_hash("DESI"),
        );
        let cata = write_db(
            dir.path(),
            "ams5052_0001",
            5052,
            1,
            e3d_attlib::db1_hash("CATA"),
        );
        std::fs::write(dir.path().join("MDS-REPORT"), b"### not a database\r\n").unwrap();

        let template_dir = Path::new("templates");
        let resolver =
            DirectoryDbResolver::scan(&[dir.path().to_path_buf()], template_dir).unwrap();

        assert_eq!(resolver.dbnos(), vec![5052, 8000]);
        assert_eq!(
            resolver.discovered().len(),
            3,
            "两个主文件 + 一个 extent 文件"
        );
        assert_eq!(resolver.non_db_files, 1);
        assert!(resolver.skipped().is_empty(), "{:?}", resolver.skipped());

        let pin = resolver.resolve(8000).expect("8000 在表里");
        assert_eq!(pin.file, desi);
        assert_eq!(pin.template, template_dir.join("desvir.dat"));
        assert_eq!(pin.db_type.as_deref(), Some("DESI"));
        assert_eq!(pin.sesno, None);

        let pin = resolver.resolve(5052).expect("5052 在表里");
        assert_eq!(pin.file, cata);
        assert_eq!(pin.template, template_dir.join("catvir.dat"));

        assert!(resolver.resolve(4242).is_none(), "没扫到的库不猜");
    }

    /// extent 文件不单独外交：它由主文件按命名约定附挂。
    #[test]
    fn an_extent_file_is_recorded_but_never_handed_out() {
        let dir = TempDir::new("extent-only");
        write_db(
            dir.path(),
            "ams7999_0002",
            7999,
            2,
            e3d_attlib::db1_hash("DESI"),
        );

        let resolver =
            DirectoryDbResolver::scan(&[dir.path().to_path_buf()], Path::new("t")).unwrap();

        assert!(resolver.resolve(7999).is_none());
        assert_eq!(resolver.discovered().len(), 1);
        assert_eq!(resolver.skipped().len(), 1, "缺主文件要在诊断里出声");
        assert!(resolver.skipped()[0].reason.contains("没扫到主文件"));
    }

    /// ams000 实测形状：`ams7355`（陈年副本）挨着 `ams7355_0001`（正身）、
    /// `ams8000_0001.codex-before-*`（备份）。命名约定认得出正身就让副本让位。
    #[test]
    fn a_canonically_named_primary_beats_its_stray_copies() {
        let dir = TempDir::new("stray-copies");
        let stale = write_db(dir.path(), "ams7355", 7355, 1, e3d_attlib::db1_hash("CATA"));
        let live = write_db(
            dir.path(),
            "ams7355_0001",
            7355,
            1,
            e3d_attlib::db1_hash("CATA"),
        );
        write_db(
            dir.path(),
            "ams7355_0001.codex-before-move",
            7355,
            1,
            e3d_attlib::db1_hash("CATA"),
        );

        let resolver =
            DirectoryDbResolver::scan(&[dir.path().to_path_buf()], Path::new("t")).unwrap();

        let pin = resolver.resolve(7355).expect("正身在表里");
        assert_eq!(pin.file, live);
        assert_eq!(resolver.skipped().len(), 2, "{:?}", resolver.skipped());
        assert!(resolver.skipped().iter().any(|s| s.file == stale));
        assert!(
            resolver.skipped().iter().all(|s| s.reason.contains("让位")),
            "{:?}",
            resolver.skipped()
        );
    }

    /// 分不出正身（两个都不合命名约定，如 `amscom` 与 `amscom.backup`）：
    /// 整组不外交、进诊断——拒绝从扫描期挪到引用点，但仍然不猜。
    #[test]
    fn an_undecidable_group_refuses_to_resolve_instead_of_guessing() {
        let dir = TempDir::new("undecidable");
        write_db(dir.path(), "amscom", 3000, 1, e3d_attlib::db1_hash("COMM"));
        write_db(
            dir.path(),
            "amscom.backup",
            3000,
            1,
            e3d_attlib::db1_hash("COMM"),
        );

        let resolver =
            DirectoryDbResolver::scan(&[dir.path().to_path_buf()], Path::new("t")).unwrap();

        assert!(resolver.resolve(3000).is_none(), "分不出正身不许猜");
        assert_eq!(resolver.skipped().len(), 2);
        assert!(
            resolver
                .skipped()
                .iter()
                .all(|s| s.reason.contains("分不出正身")),
            "{:?}",
            resolver.skipped()
        );
    }

    /// 无后缀但同号只有它一份（`amssys` 这类系统库）：孤本照常进表。
    #[test]
    fn a_sole_unconventionally_named_db_still_resolves() {
        let dir = TempDir::new("sole");
        let file = write_db(dir.path(), "amssys", 6001, 1, e3d_attlib::db1_hash("SYST"));

        let resolver =
            DirectoryDbResolver::scan(&[dir.path().to_path_buf()], Path::new("t")).unwrap();

        assert_eq!(resolver.resolve(6001).expect("孤本进表").file, file);
    }

    /// 反哈希不出类型 → 进诊断不进表：配错模板读出来的是别人的 schema。
    #[test]
    fn an_untyped_db_lands_in_diagnostics_not_in_the_table() {
        let dir = TempDir::new("untyped");
        write_db(dir.path(), "odd_0001", 4242, 1, 0);

        let resolver =
            DirectoryDbResolver::scan(&[dir.path().to_path_buf()], Path::new("t")).unwrap();

        assert!(resolver.resolve(4242).is_none());
        assert_eq!(resolver.skipped().len(), 1);
        assert!(resolver.skipped()[0].reason.contains("反哈希不出库类型名"));
    }

    /// 页 0 长得像描述符、页 1 没有签名 → 不是库，静默略过（计数即可）。
    #[test]
    fn a_file_without_the_page1_signature_is_not_a_database() {
        let dir = TempDir::new("no-signature");
        let mut bytes = page(&[0, 2, 8000, 0, 0, 0, 0, 0, 722_578, 0, 4, 1, 4, 512, 16_192]);
        bytes.extend(page(&[1, 0, 2])); // 签名字不是 0xFFFFFFFF
        std::fs::write(dir.path().join("fake_0001"), bytes).unwrap();

        let resolver =
            DirectoryDbResolver::scan(&[dir.path().to_path_buf()], Path::new("t")).unwrap();

        assert!(resolver.discovered().is_empty());
        assert_eq!(resolver.non_db_files, 1);
    }

    /// 同一个文件经两个目录参数给进来只算一次，不伪装成重复主文件。
    #[test]
    fn the_same_directory_given_twice_is_not_a_duplicate() {
        let dir = TempDir::new("same-dir-twice");
        write_db(
            dir.path(),
            "ams8000_0001",
            8000,
            1,
            e3d_attlib::db1_hash("DESI"),
        );

        let dirs = vec![dir.path().to_path_buf(), dir.path().to_path_buf()];
        let resolver = DirectoryDbResolver::scan(&dirs, Path::new("t")).unwrap();
        assert_eq!(resolver.dbnos(), vec![8000]);
    }
}

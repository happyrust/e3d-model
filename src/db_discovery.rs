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
//! 外项目库是唯一一处不靠目录定的：E3D 的 MDB 把别的项目的库当外项目库挂进来（AMS 的 `/ALL`
//! 挂着 ZDJ 的 db 7600），登记在主项目 SYS 库的 `DB` 元素上（`PROJ` = 外项目代号）。
//! [`DirectoryDbResolver::adopt_foreign_dbs`] 只补表里登记、给定目录又解析不出的那几个库号，
//! 不把整个外项目目录挂进来：外项目里没登记的库会拿同号把给定目录里的正身挤成「分不出」。
//!
//! [`discovered`]: DirectoryDbResolver::discovered
//! [`skipped`]: DirectoryDbResolver::skipped

use std::collections::{BTreeMap, HashMap, HashSet};
use std::io::{Read as _, Seek as _, SeekFrom};
use std::path::{Path, PathBuf};
use std::sync::Arc;

use e3d_attlib::AttlibData;
use e3d_io::db_element::{DbFilePin, DbFileResolver, DbSet, template_file_for};
use e3d_io::engine::{ReadOnlyEngine, ScanTier};
use e3d_io::meta::{DbDescriptor, FileInfo};
use e3d_io::record::descriptor::DescriptorValue;
use e3d_io::uda_catalog::UdaCatalog;

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
    /// 经 [`Self::adopt_foreign_dbs`] 从外项目目录补进来的库数。
    adopted: usize,
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
            adopted: 0,
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

    /// 可解析的库里类型是 `db_type` 的那些，升序。
    ///
    /// 消费方按类型挑库用：`gen_ams` 的 UDET 名字表要先找出目录里的 DICT 库
    /// （见 [`crate::udet::UdetCatalog::scan`]），而工程目录里躺着哪些库是这里才知道的。
    pub fn dbnos_of_type(&self, db_type: &str) -> Vec<u32> {
        let mut out: Vec<u32> = self
            .pins
            .iter()
            .filter(|(_, pin)| {
                pin.db_type
                    .as_deref()
                    .is_some_and(|actual| actual.eq_ignore_ascii_case(db_type))
            })
            .map(|(dbno, _)| *dbno)
            .collect();
        out.sort_unstable();
        out
    }

    /// 给定字典库里的 UDA 定义，按给出的库号顺序合并（同一把键先到先得，
    /// [`UdaCatalog::merge_in_order`]），连同读不开的库（库号、文件、原因）。表里没有的库号跳过。
    ///
    /// 给 `gen_ams` 挂字典用（[`DbSet::with_uda_catalog`]），库号与它的 UDET 名字表是同一份
    /// （[`Self::dbnos_of_type`] 问 DICT，升序）。生产 gen-model 按 MDB 的 DICT 名单（SYS 库 `CURD`
    /// 序）读同一种表；这里只有目录，序按库号——两本字典给出同一把键时才分得出先后。
    /// 读不开的库交给调用方报，不拦整张表：一本坏字典不该让别的字典里的定义一起消失。
    pub fn uda_catalog(
        &self,
        dictionaries: &[u32],
        template_dir: &Path,
        attlib: &AttlibData,
    ) -> (UdaCatalog, Vec<(u32, PathBuf, String)>) {
        let dicvir = template_dir.join("dicvir.dat");
        let mut merged = UdaCatalog::default();
        let mut unreadable = Vec::new();
        for &dbno in dictionaries {
            let Some(pin) = self.pins.get(&dbno) else {
                continue;
            };
            match UdaCatalog::read(&pin.file, &dicvir, attlib) {
                Ok(catalog) => merged.merge_in_order(catalog),
                Err(error) => unreadable.push((dbno, pin.file.clone(), error)),
            }
        }
        (merged, unreadable)
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
        let adopted = if self.adopted > 0 {
            format!("（其中 {} 个是 SYS 库登记的外项目库）", self.adopted)
        } else {
            String::new()
        };
        format!(
            "project-dir 扫到 {} 个库（{}）{adopted}，{} 个可疑文件进诊断，{} 个非库文件略过",
            self.pins.len(),
            types,
            self.skipped.len(),
            self.non_db_files
        )
    }

    /// 补上主项目 SYS 库登记的外项目库，返回逐条说明（补了哪个、哪个补不上、哪个与登记不符）。
    ///
    /// `main_dir` 是主项目目录（`ams000`），系统库取它里面的 `<代号>sys`。外项目目录按
    /// [`foreign_project_dir`] 找。只补给定目录解析不出的库号：给定目录里已有的照旧用给定目录，
    /// 与登记不符时只记一条说明（ams000 里的 `ams7355_0001` 在 SYS 库里登记为 APS 的库）。
    pub fn adopt_foreign_dbs(
        &mut self,
        main_dir: &Path,
        attlib: &Path,
        template_dir: &Path,
    ) -> Vec<String> {
        let mut notes = Vec::new();
        let main_dir = std::path::absolute(main_dir).unwrap_or_else(|_| main_dir.to_path_buf());
        let Some(own) = project_code_of(&main_dir) else {
            notes.push(format!(
                "{} 不是 `<代号>000` 形的项目目录，不补外项目库",
                main_dir.display()
            ));
            return notes;
        };
        let Some(sys_file) = project_system_db(&main_dir, &own) else {
            notes.push(format!(
                "{} 里没有系统库 {own}sys，不补外项目库",
                main_dir.display()
            ));
            return notes;
        };
        let foreign = match read_foreign_dbs(&sys_file, attlib, template_dir) {
            Ok(foreign) => foreign,
            Err(error) => {
                notes.push(format!("读 {} 的外项目表失败：{error}", sys_file.display()));
                return notes;
            }
        };
        let mut projects: HashMap<String, Result<(PathBuf, DirectoryDbResolver), String>> =
            HashMap::new();
        for db in foreign {
            if db.project.eq_ignore_ascii_case(&own) {
                continue;
            }
            let project = projects
                .entry(db.project.to_ascii_uppercase())
                .or_insert_with(|| {
                    let dir = foreign_project_dir(&db.project, &main_dir)?;
                    let resolver =
                        DirectoryDbResolver::scan(std::slice::from_ref(&dir), template_dir)
                            .map_err(|error| error.to_string())?;
                    Ok((dir, resolver))
                });
            let label = format!("外项目库 {} {}（{}）", db.dbno, db.name, db.project);
            if let Some(pin) = self.pins.get(&db.dbno) {
                if let Ok((dir, _)) = project
                    && pin
                        .file
                        .parent()
                        .is_none_or(|parent| !same_dir(parent, dir))
                {
                    notes.push(format!(
                        "{label}：给定目录已有 {}，按给定目录（SYS 库登记在 {}）",
                        pin.file.display(),
                        dir.display()
                    ));
                }
                continue;
            }
            match project {
                Ok((dir, resolver)) => match resolver.pins.get(&db.dbno) {
                    Some(pin) => {
                        notes.push(format!("{label} ← {}", pin.file.display()));
                        self.pins.insert(db.dbno, pin.clone());
                        self.adopted += 1;
                    }
                    None => notes.push(format!("{label}：{} 里解析不出这个库号", dir.display())),
                },
                Err(error) => notes.push(format!("{label}：{error}")),
            }
        }
        notes
    }
}

impl DbFileResolver for DirectoryDbResolver {
    fn resolve(&self, dbno: u32) -> Option<DbFilePin> {
        self.pins.get(&dbno).cloned()
    }
}

/// 主项目 SYS 库里登记的一只外项目库。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ForeignDb {
    pub dbno: u32,
    /// 外项目代号（`ZDJ`、`ACP`…）。
    pub project: String,
    /// 库名（`*SAMPLE/PPROJCATA`），只进说明。
    pub name: String,
}

/// 读一份 SYS 库里的外项目表：`PROJ` 非空的 `DB` 元素，按库号排。
///
/// 包括主项目自己的代号（如果它给自己的库也填了 `PROJ`），由调用方排除。
pub fn read_foreign_dbs(
    sys_file: &Path,
    attlib: &Path,
    template_dir: &Path,
) -> Result<Vec<ForeignDb>, String> {
    let set = Arc::new(
        DbSet::with_attlib_file(attlib)
            .map_err(|error| format!("attlib {}: {error}", attlib.display()))?,
    );
    let template =
        template_file_for(template_dir, "SYST").map_err(|error| format!("SYST 模板: {error}"))?;
    let dbno = set
        .add_db(DbFilePin {
            file: sys_file.to_path_buf(),
            template,
            db_type: Some("SYST".into()),
            sesno: None,
        })
        .map_err(|error| format!("开库: {error}"))?;
    let mut engine = match set.pinned_sesno(dbno) {
        Some(sesno) => ReadOnlyEngine::open_at(sys_file, sesno),
        None => ReadOnlyEngine::open(sys_file),
    }
    .map_err(|error| format!("开库: {error}"))?;
    let mut foreign = Vec::new();
    for scanned in engine
        .scan_elements(ScanTier::Header)
        .map_err(|error| format!("扫元素: {error}"))?
    {
        let scanned = scanned.map_err(|error| format!("扫元素: {error}"))?;
        let element = set.element(scanned.refno);
        if !element.element_type().is_ok_and(|noun| noun == "DB") {
            continue;
        }
        let Some(project) = element
            .attribute("PROJ")
            .ok()
            .flatten()
            .as_ref()
            .and_then(project_code)
        else {
            continue;
        };
        // SYS 库里的 DBNO 存成 Word（`Word { raw: 7600 }`），不是 Int。
        let dbno = match element.attribute("DBNO").ok().flatten() {
            Some(DescriptorValue::Word { raw, .. }) => Some(raw),
            Some(DescriptorValue::Int(dbno)) => u32::try_from(dbno).ok(),
            _ => None,
        };
        let Some(dbno) = dbno else {
            continue;
        };
        let name = element.stored_name().ok().flatten().unwrap_or_default();
        foreign.push(ForeignDb {
            dbno,
            project,
            name,
        });
    }
    foreign.sort_by_key(|db| db.dbno);
    foreign.dedup_by_key(|db| db.dbno);
    Ok(foreign)
}

/// `PROJ` 存的是代号的字符码（`RawWords([90, 68, 74])` = `ZDJ`），空 = 主项目自己的库。
fn project_code(value: &DescriptorValue) -> Option<String> {
    let code = match value {
        DescriptorValue::Text(text) => text.trim().to_string(),
        DescriptorValue::RawWords(words) | DescriptorValue::WordArray(words) => words
            .iter()
            .filter_map(|word| char::from_u32(*word))
            .collect::<String>()
            .trim()
            .to_string(),
        _ => return None,
    };
    (!code.is_empty() && code.chars().all(|c| c.is_ascii_alphanumeric())).then_some(code)
}

/// 项目目录名去掉 `000` 就是项目代号（`ams000` → `ams`、`ZDJ000` → `ZDJ`）。
fn project_code_of(dir: &Path) -> Option<String> {
    let code = dir.file_name()?.to_str()?.strip_suffix("000")?;
    (!code.is_empty()).then(|| code.to_string())
}

/// 项目的系统库文件 `<代号>sys`（`ams000/amssys`），文件名不分大小写。
fn project_system_db(dir: &Path, code: &str) -> Option<PathBuf> {
    let wanted = format!("{code}sys");
    std::fs::read_dir(dir)
        .ok()?
        .flatten()
        .map(|entry| entry.path())
        .find(|path| {
            path.is_file()
                && path
                    .file_name()
                    .and_then(|name| name.to_str())
                    .is_some_and(|name| name.eq_ignore_ascii_case(&wanted))
        })
}

/// 外项目代号 → 项目目录。E3D 按环境变量 `<代号>000` 找（`custom_evars.bat` 调的
/// `evars<代号>.bat` 设的就是 `%projects_dir%<文件夹>\<代号>000`）；没设时在主项目同根
/// （`main_dir` 的上两级）下一层里找名为 `<代号>000` 的目录，不分大小写。找不到或不唯一都不猜。
pub fn foreign_project_dir(code: &str, main_dir: &Path) -> Result<PathBuf, String> {
    let var = format!("{}000", code.to_ascii_uppercase());
    if let Some(value) = std::env::var_os(&var) {
        let dir = PathBuf::from(value);
        return if dir.is_dir() {
            Ok(dir)
        } else {
            Err(format!("环境变量 {var}={} 不是目录", dir.display()))
        };
    }
    let root = main_dir
        .parent()
        .and_then(Path::parent)
        .ok_or_else(|| format!("{} 没有上两级目录，找不到项目根", main_dir.display()))?;
    let wanted = format!("{code}000");
    let mut hits: Vec<PathBuf> = std::fs::read_dir(root)
        .map_err(|error| format!("读 {}: {error}", root.display()))?
        .flatten()
        .filter_map(|folder| std::fs::read_dir(folder.path()).ok())
        .flat_map(|children| children.flatten())
        .map(|child| child.path())
        .filter(|path| {
            path.is_dir()
                && path
                    .file_name()
                    .and_then(|name| name.to_str())
                    .is_some_and(|name| name.eq_ignore_ascii_case(&wanted))
        })
        .collect();
    hits.sort();
    match hits.len() {
        1 => Ok(hits.remove(0)),
        0 => Err(format!(
            "没设环境变量 {var}，{} 下一层也没有 {wanted} 目录",
            root.display()
        )),
        _ => Err(format!(
            "{} 下有 {} 个 {wanted} 目录（{}），不猜",
            root.display(),
            hits.len(),
            hits.iter()
                .map(|path| path.display().to_string())
                .collect::<Vec<_>>()
                .join("、")
        )),
    }
}

fn same_dir(a: &Path, b: &Path) -> bool {
    let canonical =
        |path: &Path| std::fs::canonicalize(path).unwrap_or_else(|_| path.to_path_buf());
    canonical(a) == canonical(b)
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

        assert_eq!(resolver.dbnos_of_type("CATA"), vec![5052]);
        assert_eq!(
            resolver.dbnos_of_type("desi"),
            vec![8000],
            "类型名不分大小写"
        );
        assert!(resolver.dbnos_of_type("DICT").is_empty());
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

    /// amssys 里 `PROJ` 的存法：代号的字符码一字一个，空表是主项目自己的库。
    #[test]
    fn proj_words_spell_the_foreign_project_code() {
        assert_eq!(
            project_code(&DescriptorValue::RawWords(vec![90, 68, 74])).as_deref(),
            Some("ZDJ")
        );
        assert_eq!(
            project_code(&DescriptorValue::Text(" ACP ".into())).as_deref(),
            Some("ACP")
        );
        assert_eq!(project_code(&DescriptorValue::RawWords(Vec::new())), None);
        assert_eq!(
            project_code_of(Path::new("projects/AvevaMarineSample/ams000")).as_deref(),
            Some("ams")
        );
        assert_eq!(project_code_of(Path::new("ZDJ000")).as_deref(), Some("ZDJ"));
        assert_eq!(project_code_of(Path::new("000")), None);
        assert_eq!(project_code_of(Path::new("projects")), None);
    }

    /// 没设 `<代号>000` 时在主项目同根的下一层找：唯一一个才认，两个就不猜。
    #[test]
    fn a_foreign_project_dir_is_found_beside_the_main_project_or_refused() {
        let root = TempDir::new("foreign-root");
        let main = root.path().join("AvevaMarineSample").join("ams000");
        std::fs::create_dir_all(&main).unwrap();
        let qzx = root.path().join("Sample").join("qzx000");
        std::fs::create_dir_all(&qzx).unwrap();
        assert_eq!(foreign_project_dir("QZX", &main).unwrap(), qzx);

        let error = foreign_project_dir("QZY", &main).unwrap_err();
        assert!(error.contains("QZY000"), "{error}");

        std::fs::create_dir_all(root.path().join("Copy").join("QZX000")).unwrap();
        let error = foreign_project_dir("QZX", &main).unwrap_err();
        assert!(error.contains("不猜"), "{error}");
    }
}

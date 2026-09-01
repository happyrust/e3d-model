//! 模型增量更新探针：对同一库文件的两个会话做索引差分，只重建受影响的单元。
//!
//! 用法（路径全部显式给，不猜安装位置）：
//!
//! ```text
//! update_ams --attlib <attlib.dat> --db <ams1112_0001> --template <模板文件或目录> \
//!            [--db-type DESI] [--base-sesno N] [--target-sesno M] [--out out/update_ams]
//! ```
//!
//! 两个会话号都缺省时取「最新窗口」：target = 链头（最新），base = 它的前一个。
//!
//! 产物：
//! - `<out>/update-report.json`：增量总账（候选分类 / 级联 / 执行结论）。
//! - `<out>/upserts.obj` + `<out>/upserts.json`：重建单元的网格与不变量
//!   （按 refno 覆盖旧网格）。
//! - `<out>/removals.json`：要从模型集移除的 refno（幂等「确保不存在」）。
//!
//! 账不平（[`IncrementReport::accounts_for`] 不过）以非零码退出——账本原则
//! 与 gen_ams 相同：产物先落盘再判账，查因就用那几份文件。

use std::io::Write as _;
use std::path::PathBuf;

use anyhow::{Context, bail};
use e3d_io::db_element::{DbFilePin, DbSet, template_file_for};
use e3d_model::increment::{collect_window, increment_update};
use e3d_model::mesh::TriMesh;

struct Args {
    attlib: PathBuf,
    db_file: PathBuf,
    template: PathBuf,
    db_type: String,
    base_sesno: Option<u32>,
    target_sesno: Option<u32>,
    out: PathBuf,
    no_instance_cache: bool,
}

fn parse_args() -> anyhow::Result<Args> {
    let mut attlib = None;
    let mut db_file = None;
    let mut template = None;
    let mut db_type = "DESI".to_string();
    let mut base_sesno = None;
    let mut target_sesno = None;
    let mut out = PathBuf::from("out/update_ams");
    let mut no_instance_cache = false;

    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().with_context(|| format!("参数 {flag} 缺值"));
        match flag.as_str() {
            "--attlib" => attlib = Some(PathBuf::from(value()?)),
            "--db" => db_file = Some(PathBuf::from(value()?)),
            "--template" => template = Some(PathBuf::from(value()?)),
            "--db-type" => db_type = value()?,
            "--base-sesno" => {
                base_sesno = Some(value()?.parse().context("--base-sesno 要无符号整数")?)
            }
            "--target-sesno" => {
                target_sesno = Some(value()?.parse().context("--target-sesno 要无符号整数")?)
            }
            "--out" => out = PathBuf::from(value()?),
            "--no-instance-cache" => no_instance_cache = true,
            other => bail!("不认识的参数 {other}"),
        }
    }
    Ok(Args {
        attlib: attlib.context("缺 --attlib")?,
        db_file: db_file.context("缺 --db")?,
        template: template.context("缺 --template")?,
        db_type,
        base_sesno,
        target_sesno,
        out,
        no_instance_cache,
    })
}

/// 会话号缺省解析：target = 最新，base = 前一个。链上不足两个会话时要求显式给。
fn resolve_window(args: &Args) -> anyhow::Result<(u32, u32)> {
    if let (Some(base), Some(target)) = (args.base_sesno, args.target_sesno) {
        return Ok((base, target));
    }
    let mut view = e3d_io::session::DbView::open(&args.db_file)
        .with_context(|| format!("开库读会话链 {}", args.db_file.display()))?;
    let sessions = view.walk_sessions()?;
    let target = match args.target_sesno {
        Some(t) => t,
        None => sessions.first().context("会话链是空的")?.sesno,
    };
    let base = match args.base_sesno {
        Some(b) => b,
        None => {
            // target 的前一个（链序）。
            let at = sessions
                .iter()
                .position(|s| s.sesno == target)
                .with_context(|| format!("会话 {target} 不在链上"))?;
            sessions
                .get(at + 1)
                .with_context(|| format!("会话 {target} 已是链上最旧的，没有可缺省的 base"))?
                .sesno
        }
    };
    Ok((base, target))
}

#[derive(serde::Serialize)]
struct UpsertInvariants {
    geometry_id: e3d_model::GeometryId,
    refno: String,
    noun: String,
    name: Option<String>,
    volume_mm3: f64,
    aabb_min: [f64; 3],
    aabb_max: [f64; 3],
    /// 元素局部系到世界系，列主序 `[c0,c1,c2,t]`（与 gen_ams elements.json 同约定）。
    world: [[f64; 3]; 4],
    triangles: usize,
    genus: i32,
    negatives_applied: usize,
    negatives_skipped: usize,
}

fn main() -> anyhow::Result<()> {
    let args = parse_args()?;
    if args.no_instance_cache {
        // SAFETY: 参数刚解析完、任何工作线程都还没起，此刻进程里只有主线程能碰环境块。
        unsafe { std::env::set_var(e3d_model::primitive_instance::INSTANCE_CACHE_ENV, "1") };
    }
    let t0 = std::time::Instant::now();

    let (base_sesno, target_sesno) = resolve_window(&args)?;
    println!(
        "库 {}：增量窗口 {base_sesno} → {target_sesno}",
        args.db_file.display()
    );

    let window = collect_window(&args.db_file, base_sesno, target_sesno)?;
    println!(
        "索引差分：+{} ~{} -{}（flag 漂移 {}，读页 {}）",
        window.diff.tally.inserted,
        window.diff.tally.modified,
        window.diff.tally.deleted,
        window.diff.diagnostics.flag_only_drifts,
        window.diff.io.pages_read,
    );

    let template_file = if args.template.is_dir() {
        template_file_for(&args.template, &args.db_type)?
    } else {
        args.template.clone()
    };
    // 两个端点各开一个池：差分归差分（DbView），读数归读数（DbSet）。
    let open_endpoint = |sesno: u32| -> anyhow::Result<std::sync::Arc<DbSet>> {
        let set = std::sync::Arc::new(DbSet::with_attlib_file(&args.attlib)?);
        set.add_db(DbFilePin {
            file: args.db_file.clone(),
            template: template_file.clone(),
            db_type: Some(args.db_type.clone()),
            sesno: Some(sesno),
        })?;
        Ok(set)
    };
    let base_set = open_endpoint(base_sesno)?;
    let target_set = open_endpoint(target_sesno)?;

    let outcome = increment_update(&base_set, &target_set, &window);

    std::fs::create_dir_all(&args.out)
        .with_context(|| format!("建输出目录 {}", args.out.display()))?;

    let obj_path = args.out.join("upserts.obj");
    let mut obj = std::io::BufWriter::new(
        std::fs::File::create(&obj_path).with_context(|| format!("建 {}", obj_path.display()))?,
    );
    let mut index_offset = 0u32;
    let mut invariants = Vec::with_capacity(outcome.upserts.len());
    for generated in &outcome.upserts {
        let mesh = TriMesh::from_manifold(&generated.solid);
        let (lo, hi) = mesh.aabb().unwrap_or(([0.0; 3], [0.0; 3]));
        invariants.push(UpsertInvariants {
            geometry_id: generated.geometry_id.clone(),
            refno: generated.refno.to_string(),
            noun: generated.noun.clone(),
            name: generated.name.clone(),
            volume_mm3: generated.solid.volume(),
            aabb_min: lo,
            aabb_max: hi,
            world: [
                generated.world.x_axis.truncate().to_array(),
                generated.world.y_axis.truncate().to_array(),
                generated.world.z_axis.truncate().to_array(),
                generated.world.w_axis.truncate().to_array(),
            ],
            triangles: mesh.triangle_count(),
            genus: generated.solid.genus(),
            negatives_applied: generated.negatives_applied,
            negatives_skipped: generated.negatives_skipped.len(),
        });
        let label = match &generated.name {
            Some(name) => format!("{} {} {}", generated.geometry_id, generated.noun, name),
            None => format!("{} {}", generated.geometry_id, generated.noun),
        };
        index_offset = mesh.write_obj_object(&mut obj, &label, index_offset)?;
    }
    obj.flush()?;

    std::fs::write(
        args.out.join("upserts.json"),
        serde_json::to_vec_pretty(&invariants)?,
    )?;
    // 按 GeometryId 落盘，与 upserts.json 同一把键：一个路由容器 refno 名下可能有
    // 多根管身，只写 refno 的话消费端分不出该擦哪一根。
    std::fs::write(
        args.out.join("removals.json"),
        serde_json::to_vec_pretty(&outcome.removals)?,
    )?;
    let report_path = args.out.join("update-report.json");
    std::fs::write(&report_path, serde_json::to_vec_pretty(&outcome.report)?)?;
    let cache_stats = e3d_model::primitive_cache_stats();
    std::fs::write(
        args.out.join("primitive-cache.json"),
        serde_json::to_vec_pretty(&cache_stats)?,
    )?;

    println!("{}", outcome.report.totals_line());
    println!("基本体实例缓存: {cache_stats:?}");
    println!(
        "动作：upsert {} 个 / remove {} 个；产物 {}（{} ms）",
        outcome.upserts.len(),
        outcome.removals.len(),
        args.out.display(),
        t0.elapsed().as_millis()
    );

    // 产物先落盘再判账（同 gen_ams）：账不平时那几份文件正是查因用的。
    outcome.report.accounts_for().map_err(|detail| {
        anyhow::anyhow!("{detail}\n先看 update-report.json 的 unresolved/no_model")
    })?;
    println!("账已平：候选与执行两本账闭合");
    Ok(())
}

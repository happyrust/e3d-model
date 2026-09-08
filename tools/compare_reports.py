r"""把 `run_rvm_symmetric_audit.ps1` 跑出来的两份报告逐件对照。

新路径（`rvm_compare --generated`，RVM ↔ RVM）必须与旧路径
（`--elements` + `--facets`，快照 ↔ elements.json）**逐件同档**。
不同的每一件都要能解释；「新的看起来更合理」不算解释。

    python tools/compare_reports.py out/rvm-symmetric-20260904
"""

import json
import pathlib
import sys


def load(path):
    data = json.loads(pathlib.Path(path).read_text(encoding="utf-8"))
    return {p["key"]: p for p in data["pairs"]}, data


def main(root):
    root = pathlib.Path(root)
    runs = json.loads((root / "runs.json").read_text(encoding="utf-8"))
    if isinstance(runs, dict):
        runs = [runs]

    print(f"{'sample':<30}{'pairs':>7}{'键不同':>8}{'判档不同':>10}{'Δdelta mm':>12}{'退出码':>10}")
    total = differing = missing = 0
    worst = 0.0
    offenders = []
    for run in runs:
        stem = run.get("Stem", "?")
        if not run.get("NewJson") or not run.get("OldJson"):
            print(f"{stem:<30}  {run.get('Status', '缺报告')}")
            missing += 1
            continue
        new, dnew = load(run["NewJson"])
        old, dold = load(run["OldJson"])
        key_diff = len(set(new) ^ set(old))
        shared = sorted(set(new) & set(old))
        verdict_diff = [k for k in shared if new[k]["verdict"] != old[k]["verdict"]]
        delta = max(
            (abs(new[k]["max_axis_delta_mm"] - old[k]["max_axis_delta_mm"]) for k in shared),
            default=0.0,
        )
        total += len(shared)
        differing += len(verdict_diff)
        worst = max(worst, delta)
        for k in verdict_diff[:3]:
            offenders.append(f"{stem}: {k}  old={old[k]['verdict']} new={new[k]['verdict']}")
        print(
            f"{stem:<30}{len(shared):>7}{key_diff:>8}{len(verdict_diff):>10}{delta:>12.6f}"
            f"{str(run.get('OldExit')) + '/' + str(run.get('NewExit')):>10}"
        )

    print(f"\n合计配对 {total} 件；判档不同 {differing} 件；max_axis_delta 最大差 {worst:.6f} mm")
    for line in offenders[:12]:
        print(f"  {line}")
    if missing:
        print(f"{missing} 条样本没有报告")

    # kind 一致率只有 RVM ↔ RVM 那一侧有；它衡量的是「我们的几何分解与 E3D 是否同构」。
    same = differ = 0
    by_pair = {}
    for run in runs:
        if not run.get("NewJson"):
            continue
        new, _ = load(run["NewJson"])
        for pair in new.values():
            if pair.get("kind_match") is None:
                continue
            if pair["kind_match"]:
                same += 1
            else:
                differ += 1
                combo = (summarise(pair.get("baseline_kinds")), summarise(pair.get("generated_kinds")))
                by_pair[combo] = by_pair.get(combo, 0) + 1
    if same or differ:
        total_kind = same + differ
        print(f"\nkind 一致率：{same}/{total_kind}（{100.0 * same / total_kind:.1f}%）")
        print("  不同的组合（基准 → 生成）：")
        for (base, gen), count in sorted(by_pair.items(), key=lambda kv: -kv[1])[:10]:
            print(f"    {base:<38} → {gen:<20} x{count}")


def summarise(kinds):
    """`['Box','Box','Cylinder']` → `Box×2 + Cylinder`。逐个列出来在几十片的成员上没法看。"""
    if not kinds:
        return "-"
    counts = {}
    for k in kinds:
        counts[k] = counts.get(k, 0) + 1
    return " + ".join(
        k if n == 1 else f"{k}×{n}" for k, n in sorted(counts.items(), key=lambda kv: (-kv[1], kv[0]))
    )
    return 1 if (differing or missing) else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "out/rvm-symmetric-20260904"))

"""把 N0-1 census 的六份 report.json 按 noun 横过来，看每个 noun 落在哪本账上。

用法： python .scratch/census_nouns.py [tag]
"""

import collections
import json
import os
import sys

TAG = sys.argv[1] if len(sys.argv) > 1 else "n0-1-20260905"
LIBS = ["ams1112", "ams7997", "ams8000", "aps250110", "aps250164", "aps250209"]
WANT = [
    # W4-3 ACC 舱壁
    "WLJOIN", "WLPROF", "WLPANE", "BPANEL", "CTWALL",
    # W4-4 舱室装修
    "CLNTIL", "BNDLIN", "FLRLAY", "INSURQ", "FPFITT", "ELFITT", "HVACFI", "INFITT",
    # W3 直纹体
    "RULS", "NRUL",
    # W2 路由节点
    "RNODE", "CNODE", "POINTR", "PNOD", "SNOD",
    # W4-2 ASL 通道钢结构
    "HANDRA", "RAIL", "KICKPL", "STRLNG", "PLTFRM", "STRFLT", "RLADDR", "SLADDR", "HRPOST",
    # recheck-3 §2.5 的伪需求
    "STWALL",
]
LEDGERS = [
    "catalog_pending", "unknown_nouns", "not_wired", "proven_non_graphic",
    "evidence_pending", "composites_built", "route_members", "consumed",
    "centreline_only", "geometry_sets", "unsupported", "orphans",
]

hits = collections.defaultdict(lambda: collections.defaultdict(dict))
for lib in LIBS:
    path = os.path.join("out", TAG, lib, "report.json")
    if not os.path.exists(path):
        continue
    report = json.load(open(path, encoding="utf-8"))
    for ledger in LEDGERS:
        for noun, count in (report.get(ledger) or {}).items():
            if noun in WANT:
                hits[noun][ledger][lib] = count

for noun in WANT:
    if noun not in hits:
        print("%-8s  --  六库全零" % noun)
        continue
    for ledger, by_lib in hits[noun].items():
        total = sum(by_lib.values())
        detail = " ".join("%s=%d" % kv for kv in sorted(by_lib.items()))
        print("%-8s  %-18s %6d   %s" % (noun, ledger, total, detail))

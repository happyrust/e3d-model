"""Generate `data/core3d-builder-registry.json` from the raw ida-bridge dump.

Input: the stdout of `tools/core3d_registry_dump.py` run against a live Core3D.dll
IDB, saved to `evidence/20260905-core3d-csg-registry/addplug-sites.txt`.

    ida-bridge exec <client_id> -f tools/core3d_registry_dump.py \
        > evidence/20260905-core3d-csg-registry/addplug-sites.txt
    python tools/build_core3d_registry.py

The CRDESI switch arms are carried here as decoded literals because the decode is
a pure function of the case immediates (`tools/noun_decode.py`), not of the dump.
`evidence/` is gitignored, so the raw dump is a local artifact and this script plus
the checked-in JSON are what make it reproducible.
"""

import json
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HERE = os.path.join(ROOT, "evidence", "20260905-core3d-csg-registry")
OUT = os.path.join(ROOT, "data", "core3d-builder-registry.json")

CSG_PLUG_ENTRY = "0x107158b0"
DB_MODEL_PLUG_ENTRY = "0x108e79d0"
CRDESI = "0x10343b80"

# Nouns whose addPlug site passes the DB_Noun in through a register, so
# `csg_registry_dump.py` cannot read a NOUN_* global at the site. These two are
# pinned by the builder class name plus the DB_AttPluggerHelper::addPlug lines in
# the same function (`ATT_BNDARE, NOUN_BNDLIN` / `ATT_INSARE, NOUN_INSURQ`).
INFERRED_NOUN_BY_BUILDER = {
    "CSG_TreeBuilderBNDLIN": "BNDLIN",
    "CSG_TreeBuilderINSURQ": "INSURQ",
}

# The single DB_ModelBuilder::addPlug registration (caller sub_1050B1D0).
DB_MODEL_PLUGS = [{"noun": "DTPROF", "builder": "DES_ModelDTPROF", "registrar": "sub_1050B1D0",
                   "sites": ["0x1055633d"]}]

# CRDESI@0x10343b80 switch arms, `case` immediates reverse-hashed by noun_decode.py.
CRDESI_ARMS = {
    718014: "CYLI", 785957: "NCYL", 703003: "DISH", 912101: "NDIS", 640470: "CONE",
    818033: "NCON", 956116: "SNOU", 837419: "NSNO", 549344: "BOX", 1014836: "NBOX",
    779182: "PVOL", 790729: "GRDM", 636109: "POHE", 832210: "POGO", 564937: "PYRA",
    904406: "NPYR", 897213: "CTOR", 841361: "NCTO", 897228: "RTOR", 841766: "NRTO",
    1026046: "SLCY", 599765: "NSLC", 560322: "RPLA", 959395: "DATU", 822719: "BOUN",
    985369: "DRAW", 207879217: "GRIDLN", 621502: "PNOD", 621505: "SNOD", 719964: "IPOI",
    719975: "TPOI", 813985: "POIN", 900968: "EXTR", 900977: "NXTR", 914751: "RULS",
    783446: "NRUL", 837961: "PLOO", 842877: "REVO", 968612: "NREV", 77028748: "POLYHE",
    128856299: "NPOLYH",
}

RE_SITE = re.compile(r"^(0x[0-9a-f]+) \| (.+?) \| (\S+) \| (\S+) \| (\S+) \| (\d+)$")


def read_sites():
    path = os.path.join(HERE, "addplug-sites.txt")
    rows = []
    with open(path, encoding="utf-8-sig") as fh:
        for line in fh:
            m = RE_SITE.match(line.strip())
            if m:
                site, owner, noun, builder, slot, size = m.groups()
                rows.append(
                    {"site": site, "registrar": owner, "noun": noun, "builder": builder,
                     "getCsgTree": None if slot == "?" else slot,
                     "getCsgTreeBytes": int(size) or None}
                )
    if not rows:
        raise SystemExit(
            "addplug-sites.txt 里一行都没解析出来——它是不是旧格式（没有 getCSGTree 两列）？"
            " 重跑 csg_registry_dump.py。"
        )
    return rows


def short_registrar(owner):
    owner = owner.split("(")[0].strip()
    return owner or "?"


def main():
    sites = read_sites()

    by_noun = {}
    unresolved = []
    for row in sites:
        noun, builder = row["noun"], row["builder"]
        if noun == "?":
            noun = INFERRED_NOUN_BY_BUILDER.get(builder, "?")
        if noun == "?" or builder == "?":
            why = "DB_Noun arrives in a register; follow the caller to pin it"
            if row["site"] == "0x108e7a5e":
                why = ("not a registration: DB_ModelBuilder::addPlug forwarding into "
                       "CSG_TreeBuilder::addPlug")
            unresolved.append(
                {"site": row["site"], "registrar": short_registrar(row["registrar"]),
                 "builder": None if builder == "?" else builder, "why": why}
            )
            continue
        entry = by_noun.setdefault(
            noun,
            {"noun": noun, "source": "csg_plug", "builder": builder,
             "registrar": short_registrar(row["registrar"]), "sites": []},
        )
        entry["sites"].append(row["site"])
        if row["getCsgTree"]:
            entry["getCsgTree"] = row["getCsgTree"]
        if row["getCsgTreeBytes"]:
            entry["getCsgTreeBytes"] = row["getCsgTreeBytes"]
        if builder in INFERRED_NOUN_BY_BUILDER:
            entry["nounInferredFrom"] = "builder class name + DB_AttPluggerHelper::addPlug"

    for plug in DB_MODEL_PLUGS:
        by_noun.setdefault(plug["noun"], dict(plug, source="db_model_plug"))

    legacy = [
        {"noun": noun, "source": "crdesi", "code": code, "switch": CRDESI}
        for code, noun in sorted(CRDESI_ARMS.items(), key=lambda kv: kv[1])
    ]

    doc = {
        "$comment": (
            "Nouns for which Core3D has a geometry-building path, enumerated from the "
            "live IDB. This is the objective half of the D3 gate: a noun in here that "
            "we classify NonGraphic/Unknown is an unaccounted gap, not a proven "
            "non-graphic element. Regenerate with tools/core3d_registry_dump.py "
            "(inside IDA) plus tools/build_core3d_registry.py."
        ),
        "image": "Core3D.dll",
        "capturedOn": "2026-09-05",
        "evidenceDir": "evidence/20260905-core3d-csg-registry/",
        "entryPoints": {
            "csgPlug": CSG_PLUG_ENTRY,
            "dbModelPlug": DB_MODEL_PLUG_ENTRY,
            "legacySwitch": CRDESI,
        },
        "plugSiteCount": len(sites),
        "nouns": sorted(by_noun.values(), key=lambda e: e["noun"]),
        "legacySwitchArms": legacy,
        "unresolvedSites": sorted(unresolved, key=lambda e: e["site"]),
    }

    with open(OUT, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(doc, fh, ensure_ascii=False, indent=2)
        fh.write("\n")
    print("wrote %s: %d plug nouns, %d legacy arms, %d unresolved sites"
          % (OUT, len(doc["nouns"]), len(legacy), len(unresolved)))


if __name__ == "__main__":
    main()

"""解出 core.dll 里每个 I*COM 谓词读的 noun 字典字段号。

I*COM 的形状统一是 ATNINT/ATNLOG(noun, &field_id, &value, &err)——字段号是一个
静态 dword。把它读出来，就能把「谓词」对上 DB_Noun 的能力位（字段号表来自
gm_noun_caps_probe.py，那份是从 core.dll 的 DB_Noun 访问器里逆出来的）。
"""

import json

import ida_bytes
import ida_funcs
import ida_name
import idautils
import idc

FIELD_NAMES = {
    659518: "primitive",
    859903: "geomset",
    663225: "extrusion",
    290555737: "isPointsetPoint",
    5099119: "graphicsBehaviour",
    661624: "point",
    778791: "positiveEquivalent",
    837586: "synonym",
    297853135: "primaryList",
    76272573: "changeType",
    847458: "spatialMap",
    65664829: "secondaryHierarchy",
    722704: "visible",
    661628: "toplevel",
    750400: "pickable",
    621476: "modifiable",
    843594: "world",
    713101: "defined",
    45889870: "validc",
}

TARGETS = [
    "IASLCO", "ICABCO", "ICCOMP", "ICOCOM", "IECOMP", "IFCOMP", "IFICOM",
    "IG2COM", "IGMCOM", "IHCOMP", "IHLCOM", "IHVCOM", "IJOCOM", "INCOMP",
    "INGCOM", "IPCOMP", "IPFCOM", "IPLCOM", "IPRCOM", "IPTCOM", "ISUBCO",
    "IUPCOM",
]

out = []
for name in TARGETS:
    ea = ida_name.get_name_ea(idc.BADADDR, name)
    if ea == idc.BADADDR:
        out.append({"name": name, "error": "symbol not found"})
        continue
    func = ida_funcs.get_func(ea)
    if not func:
        out.append({"name": name, "error": "no function"})
        continue

    callees, candidates = [], []
    for head in idautils.Heads(func.start_ea, func.end_ea):
        mnem = idc.print_insn_mnem(head)
        if mnem == "call":
            target = idc.get_operand_value(head, 0)
            callees.append(ida_name.get_name(target) or hex(target))
        # 字段号以 `push offset <data>` 传进 ATNINT/ATNLOG，取该地址处的 dword。
        for op in range(2):
            if idc.get_operand_type(head, op) != idc.o_imm:
                continue
            addr = idc.get_operand_value(head, op)
            if not ida_bytes.is_loaded(addr):
                continue
            value = ida_bytes.get_dword(addr)
            if value in FIELD_NAMES:
                candidates.append(
                    {"at": hex(addr), "field": value, "meaning": FIELD_NAMES[value]}
                )
            elif 500000 <= value <= 400000000:
                candidates.append({"at": hex(addr), "field": value, "meaning": "?"})

    out.append(
        {
            "name": name,
            "ea": hex(func.start_ea),
            "calls": sorted(set(callees)),
            "fields": candidates,
        }
    )

print(json.dumps(out, indent=1))

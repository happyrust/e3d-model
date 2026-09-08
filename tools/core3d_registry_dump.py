"""Enumerate the Core3D CSG_TreeBuilder::addPlug registry from a live IDB.

Shape at every call site:

    push 4 / operator new / mov [eax], offset ??_7<Builder>@@6B@
    push eax                                  ; CSG_BaseCSGTree*
    mov  eax, ds:?NOUN_<NAME>@@3QBVDB_Noun@@B ; the noun global
    push [eax]                                ; DB_Noun*
    call CSG_TreeBuilder::instance / mov ecx, eax
    call addPlug

So walk back from the call, pick up the nearest NOUN_ symbol and the nearest
vftable offset written into the freshly-newed object, then resolve that vftable's
slot 0 (`getCSGTree`) so a stub is distinguishable from a real builder.

Run:  ida-bridge exec <client_id> -f tools/core3d_registry_dump.py
Output line format (consumed by tools/build_core3d_registry.py):

    <site> | <registrar> | <noun> | <builder> | <getCSGTree ea> | <bytes>
"""

import re

import ida_bytes
import ida_funcs
import ida_name
import idautils
import idc

ADDPLUG = 0x107158B0

RE_NOUN = re.compile(r"\?NOUN_([A-Za-z0-9_]+)@@")
RE_VFT = re.compile(r"\?\?_7([A-Za-z0-9_]+)@@6B@")


def _demangle(nm):
    if not nm:
        return ""
    return ida_name.demangle_name(nm, 0) or nm


def _slot0(builder):
    """Resolve <builder>::getCSGTree — vftable slot 0 — to (ea, size)."""
    ea = ida_name.get_name_ea(0, "??_7%s@@6B@" % builder)
    if ea in (None, idc.BADADDR):
        return ("", 0)
    target = ida_bytes.get_dword(ea)
    f = ida_funcs.get_func(target)
    if not f:
        return (hex(target), 0)
    return (hex(target), f.end_ea - f.start_ea)


rows = []
for xref in idautils.XrefsTo(ADDPLUG, 0):
    ea = xref.frm
    if idc.print_insn_mnem(ea) != "call":
        continue
    f = ida_funcs.get_func(ea)
    owner = _demangle(ida_name.get_name(f.start_ea)) if f else "?"
    owner = owner.replace("public: static void __cdecl ", "").replace(
        "public: void __thiscall ", ""
    )
    owner = owner.split("(")[0].strip() or "?"
    noun = ""
    builder = ""
    cur = ea
    for _ in range(30):
        prev = idc.prev_head(cur)
        if prev == idc.BADADDR or prev >= cur:
            break
        cur = prev
        # Stop at the previous registration: several of these functions register
        # back-to-back, and without this guard the scan happily attributes the
        # previous site's NOUN_* global to a site whose noun arrives in a register.
        if idc.print_insn_mnem(cur) == "call" and idc.get_operand_value(cur, 0) == ADDPLUG:
            break
        raw = idc.GetDisasm(cur) or ""
        for opn in range(3):
            v = idc.get_operand_value(cur, opn)
            if v and v != idc.BADADDR:
                raw += " " + (ida_name.get_name(v) or "")
        if not noun:
            m = RE_NOUN.search(raw)
            if m:
                noun = m.group(1)
        if not builder:
            m = RE_VFT.search(raw)
            if m:
                builder = m.group(1)
        if noun and builder:
            break
    slot, size = _slot0(builder) if builder else ("", 0)
    rows.append((hex(ea), owner, noun or "?", builder or "?", slot or "?", size))

rows.sort()
for r in rows:
    print("%s | %s | %s | %s | %s | %d" % r)
_result_ = {"count": len(rows)}

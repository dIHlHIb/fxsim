"""Prelookup and Interpolation Using Prelookup for supported fixed-point configurations.

- Prelookup: the input is clipped to [first, last breakpoint]; index = the interval holding it (from 0; on the last
  breakpoint, the last interval with fraction 1); fraction = (x - bp[k]) / (bp[k+1] - bp[k]) computed exactly and
  rounded once into the fraction type by the block's RndMeth (library default Floor; Floor, Nearest, Zero recorded).
  The fraction type must hold 1.0 (Simulink:blocks:PreLookupFractionOverflow otherwise); ExtrapMethod
  Clip (Linear, also the library default when it is left out: Simulink:blocks:PreLookupExtrapMethodError; recordings -- the Interpolation block likewise, CannotExtrapData).
- Interpolation Using Prelookup, one table dimension, fed by one Prelookup: the table is quantised to its type (nearest);
  Linear point-slope: y = T[k] + f * (T[k+1] - T[k]) rounded once into the output type by RndMeth (default Floor;
  Floor, Nearest recorded); Flat: T[k] (the last value when the index is the last interval and the fraction 1 --
  a fraction rounded up to 1 inside the table stays T[k]); Nearest: T[k+1] when f >= 0.5, else T[k].

Accepted only as recorded: the input in the breakpoints' own type (sfix16_En8 recorded), breakpoints exact in it,
index uint8 / uint32, the table type equal to the output type with every table value inside it, the intermediate type
the internal rule or fixdt(1,32,20). Two dimensions (17 models): Linear rounds along dimension 1
into the intermediate type (fraction length 10 for a fixdt(1,16,10) table with the internal rule, 20 with
fixdt(1,32,20); other internal-rule tables refused), then along dimension 2 into the output; Flat / Nearest pick a
corner. Three dimensions are refused. The lowering keeps the Prelookup's two outputs as two internal blocks (index, fraction) and the
Interpolation block as one; both run on the Python path.
"""
import json
import re
from fractions import Fraction

import fxgraph_lib as G

INTERNAL = "\x1d"
PL_DEFAULTS = {"BreakpointsSpecification": "Explicit values", "BreakpointsData": "[10:10:110]",
               "BreakpointsFirstPoint": "10", "BreakpointsSpacing": "10", "BreakpointsNumPoints": "11",
               "BreakpointDataTypeStr": "Inherit: Same as input", "IndexDataTypeStr": "uint32",
               "FractionDataTypeStr": None, "ExtrapMethod": "Linear", "UseLastBreakpoint": "off", "RndMeth": "Floor"}
IP_DEFAULTS = {"NumberOfTableDimensions": "2", "Table": None, "InterpMethod": "Linear point-slope",
               "ExtrapMethod": "Linear", "TableDataTypeStr": None, "OutDataTypeStr": None,
               "IntermediateResultsDataTypeStr": "Inherit: Inherit via internal rule", "RndMeth": "Floor",
               "InternalRulePriority": "Speed", "SaturateOnIntegerOverflow": "off"}
INDEX_TYPES = {"uint8": (0, 8, 0), "uint32": (0, 32, 0)}
ROUNDINGS = ("Floor", "Nearest", "Zero")
# Supported input, fraction and rounding configurations for explicit breakpoints.
MAPB_BP = "fixdt(1,8,4)"
MAPB_INPUTS = ((1, 12, 6), (1, 16, 12))
MAPB_FT = ((0, 16, 4), (0, 8, 2), (0, 8, 4), (0, 16, 15))
MAPB_RND = ("Nearest", "Floor", "Round")


def has(spec):
    return isinstance(spec, dict) and isinstance(spec.get("blocks"), dict) and any(
        isinstance(b, dict) and b.get("type") in ("PreLookup", "Interpolation_n-D") for b in spec["blocks"].values())


def _refuse(where, msg, code="UNSUPPORTED"):
    raise G.GraphError(code, where, msg)


def _params(n, b, defaults):
    p = dict(defaults)
    for k, v in b.items():
        if k in ("type", "InputPortMap"):
            continue
        if k not in p:
            _refuse("design.blocks.%s.%s" % (n, k), "unknown parameter %s for %s (accepted: %s)"
                    % (k, b["type"], ", ".join(sorted(defaults))), "BAD_PARAM")
        p[k] = str(v).strip()
    return p


def _nums(text, where):
    v = str(text).strip()
    if not (v.startswith("[") and v.endswith("]")):
        _refuse(where, "a row of numeric literals, e.g. '[0 1 2 4]'", "BAD_PARAM")
    out = []
    for e in v[1:-1].replace(",", " ").replace(";", " ").split():
        try:
            out.append(Fraction(e))
        except (ValueError, ZeroDivisionError):
            _refuse(where, "%r is not a numeric literal (expressions are not evaluated)" % e)
    return out


def _rows(text, where):
    """Read a matrix literal or a column-major reshape as exact fractions."""
    v = str(text).strip()
    m = re.match(r"^reshape\(\s*(\[[^\]]*\])\s*,\s*\[\s*([0-9.]+)[\s,]+([0-9.]+)\s*\]\s*\)$", v)
    if m:
        vals = _nums(m.group(1), where)
        n1, n2 = Fraction(m.group(2)), Fraction(m.group(3))
        if n1.denominator != 1 or n2.denominator != 1 or n1 * n2 != len(vals):
            _refuse(where, "reshape: %d values do not fill %s x %s" % (len(vals), m.group(2), m.group(3)), "BAD_PARAM")
        n1, n2 = int(n1), int(n2)
        return [[vals[i + n1 * j] for j in range(n2)] for i in range(n1)]
    if not (v.startswith("[") and v.endswith("]")):
        _refuse(where, "a matrix of numeric literals, e.g. '[0 1; 2 3]'", "BAD_PARAM")
    return [_nums("[%s]" % r, where) for r in v[1:-1].split(";") if r.strip()]


def _round(v, mode):
    """v (Fraction) to an integer by the rounding method."""
    if mode == "Floor":
        return v.numerator // v.denominator
    if mode == "Zero":
        return int(v)   # toward zero
    if mode == "Round" and v < 0:   # half away from zero (the fractions are nonnegative)
        return -_round(-v, "Nearest")
    if mode == "Ceiling":   
        return -((-v).numerator // (-v).denominator)
    return (v + Fraction(1, 2)).numerator // (v + Fraction(1, 2)).denominator   # Nearest: half up


def _breakpoints(n, p):
    where = "design.blocks.%s" % n
    if p["BreakpointsSpecification"] == "Explicit values":
        bp = _nums(p["BreakpointsData"], where + ".BreakpointsData")
    elif p["BreakpointsSpecification"] == "Even spacing":
        try:
            f0, sp, k = Fraction(p["BreakpointsFirstPoint"]), Fraction(p["BreakpointsSpacing"]), int(p["BreakpointsNumPoints"])
        except (ValueError, ZeroDivisionError):
            _refuse(where, "Even spacing: numeric first point, spacing and number of points", "BAD_PARAM")
        bp = [f0 + i * sp for i in range(k)]
    else:
        _refuse(where + ".BreakpointsSpecification", "Explicit values or Even spacing (recorded)")
    if len(bp) < 2 or any(b2 <= b1 for b1, b2 in zip(bp, bp[1:])):
        _refuse(where, "breakpoints must be at least two strictly increasing values", "BAD_PARAM")
    return bp


def rewrite(spec, info):
    """spec with each PreLookup split into its index and fraction blocks and each 1-D Interpolation Using Prelookup
    an internal block reading them; queued type checks in info["prelookup_checks"]."""
    blocks = dict(spec["blocks"])
    lines = [dict(ln) for ln in spec.get("lines", [])]
    checks = info.setdefault("prelookup_checks", [])
    into = {(ln["dst"], int(ln.get("dport", 1))): ln for ln in lines}
    pls = {}
    for n, b in list(blocks.items()):
        if b.get("type") != "PreLookup":
            continue
        where = "design.blocks." + n
        p = _params(n, b, PL_DEFAULTS)
        bp = _breakpoints(n, p)
        if p["ExtrapMethod"] != "Clip":
            _refuse(where + ".ExtrapMethod", "Prelookup with ExtrapMethod %r%s: Simulink refuses linear extrapolation "
                    "of fixed-point data (Simulink:blocks:PreLookupExtrapMethodError); write 'Clip'"
                    % (p["ExtrapMethod"], " (the library default, left out)" if "ExtrapMethod" not in b else ""),
                    "TYPE_MISMATCH")
        if p["UseLastBreakpoint"] != "off":
            _refuse(where + ".UseLastBreakpoint", "UseLastBreakpoint on was not recorded")
        mapb = p["BreakpointDataTypeStr"] == MAPB_BP
        if p["RndMeth"] not in (MAPB_RND if mapb else ROUNDINGS):
            _refuse(where + ".RndMeth", "RndMeth %r: supported modes for these breakpoints are %s" % (
                p["RndMeth"], ", ".join(MAPB_RND if mapb else ROUNDINGS)))
        if p["IndexDataTypeStr"] not in INDEX_TYPES:
            _refuse(where + ".IndexDataTypeStr", "index type %r: uint8 or uint32 recorded" % p["IndexDataTypeStr"])
        if not p["FractionDataTypeStr"]:
            _refuse(where + ".FractionDataTypeStr", "give the fraction type explicitly, e.g. fixdt(0,16,15) (its "
                    "library default was not recorded)")
        ft = G._fixdt(p["FractionDataTypeStr"], where + ".FractionDataTypeStr")
        if not isinstance(ft, tuple):
            _refuse(where + ".FractionDataTypeStr", "a fixdt fraction type")
        if ft[2] > ft[1] - (2 if ft[0] else 1):
            _refuse(where + ".FractionDataTypeStr", "fraction type %s cannot hold 1.0 (the fraction on the last "
                    "breakpoint): Simulink:blocks:PreLookupFractionOverflow -- use fraction length <= word length - %d"
                    % (p["FractionDataTypeStr"], 2 if ft[0] else 1), "TYPE_MISMATCH")
        if p["BreakpointDataTypeStr"] != "Inherit: Same as input" and not mapb:
            _refuse(where + ".BreakpointDataTypeStr", "Use the input type or %s for breakpoints." % MAPB_BP)
        if mapb:
            if tuple(ft) not in MAPB_FT or p["IndexDataTypeStr"] != "uint32":
                _refuse(where, "Prelookup with breakpoints %s, fraction %s and index %s: use a uint32 index "
                        "and one of these fraction types: %s" % (MAPB_BP, p["FractionDataTypeStr"],
                        p["IndexDataTypeStr"], ", ".join(G._tstr(t) for t in MAPB_FT)))
            if any((x * 16).denominator != 1 or not -128 <= x * 16 <= 127 for x in bp):
                _refuse(where + ".BreakpointsData", "breakpoints not exact in %s (not recorded)" % MAPB_BP)
        src = into.get((n, 1))
        if src is None:
            _refuse(where, "input port 1 is not connected", "BAD_CONNECTION")
        par = {"bp": [str(x) for x in bp], "rnd": p["RndMeth"], "it": INDEX_TYPES[p["IndexDataTypeStr"]], "ft": list(ft)}
        if mapb:
            par["mapb"] = True
        # the index keeps the Prelookup's own name (its compiled type is the index type, as Simulink's port 1); the
        # fraction (port 2) is an internal block, recorded in parameter_bounds.PORT_KEY for the export's port types
        X = INTERNAL + n + "/"
        blocks[n] = {"type": "PLCore", "Params": json.dumps(dict(par, part="index"), sort_keys=True)}
        blocks[X + "fraction"] = {"type": "PLCore", "Params": json.dumps(dict(par, part="fraction"), sort_keys=True)}
        lines.append({"src": src["src"], "sport": int(src.get("sport", 1)), "dst": X + "fraction", "dport": 1})
        checks.append(("prelookup", n, (src["src"], int(src.get("sport", 1))), [str(x) for x in bp], mapb,
                       p["BreakpointsSpecification"]))
        pls[n] = (n, X + "fraction", len(bp), par)
    for ln in lines:
        if ln["src"] in pls:
            k = int(ln.get("sport", 1))
            if k not in (1, 2):
                _refuse("design.lines", "Prelookup %s has output ports 1 (index) and 2 (fraction)" % ln["src"],
                        "BAD_CONNECTION")
            if k == 2:
                ln["src"] = pls[ln["src"]][1]
            ln.pop("sport", None)
    for n, b in list(blocks.items()):
        if b.get("type") != "Interpolation_n-D":
            continue
        where = "design.blocks." + n
        p = _params(n, b, IP_DEFAULTS)
        dims = p["NumberOfTableDimensions"]
        if dims not in ("1", "2"):
            _refuse(where + ".NumberOfTableDimensions", "Interpolation Using Prelookup with %s table dimensions: one or "
                    "two are answered" % dims)
        if p["ExtrapMethod"] != "Clip":
            _refuse(where + ".ExtrapMethod", "Interpolation Using Prelookup with ExtrapMethod %r%s: Simulink refuses "
                    "linear extrapolation of fixed-point data (Simulink:blocks:CannotExtrapData); write 'Clip'"
                    % (p["ExtrapMethod"], " (the library default, left out)" if "ExtrapMethod" not in b else ""),
                    "TYPE_MISMATCH")
        if p["InterpMethod"] not in ("Linear point-slope", "Flat", "Nearest"):
            _refuse(where + ".InterpMethod", "Linear point-slope, Flat or Nearest")
        if p["RndMeth"] not in ("Floor", "Nearest", "Round", "Zero", "Ceiling"):
            _refuse(where + ".RndMeth", "RndMeth %r: use Floor or Nearest. Round requires the supported "
                    "fixed-point table configuration. Zero and Ceiling require an unsigned table." % p["RndMeth"])
        if p["IntermediateResultsDataTypeStr"] == "Inherit: Same as output" and \
                str(p["TableDataTypeStr"]).replace(" ", "").startswith("fixdt(0,"):
            pass   # recordings (unsigned tables): checked with the table type below
        elif p["IntermediateResultsDataTypeStr"] not in ("Inherit: Inherit via internal rule", "fixdt(1,32,20)"):
            _refuse(where + ".IntermediateResultsDataTypeStr", "the internal rule or fixdt(1,32,20) (recorded)")
        if p["InternalRulePriority"] != "Speed" or p["SaturateOnIntegerOverflow"] != "off":
            _refuse(where, "InternalRulePriority Speed and SaturateOnIntegerOverflow off recorded")
        if not p["TableDataTypeStr"] or not p["OutDataTypeStr"] or p["TableDataTypeStr"] != p["OutDataTypeStr"]:
            _refuse(where + ".OutDataTypeStr", "give the table and output types explicitly and equal (recorded)")
        tt = G._fixdt(p["TableDataTypeStr"], where + ".TableDataTypeStr")
        umode = None
        if isinstance(tt, tuple) and tt[0] == 0:
            # recordings (52 models, every step): an unsigned table fixdt(0,8,4) / fixdt(0,16,8), 1-D,
            # Linear point-slope. With an unsigned intermediate (Inherit: Same as output, or the internal rule on the
            # 16-bit table) a falling segment subtracts the rounded magnitude: T[k] - rnd(f * (T[k] - T[k+1])); the
            # internal rule on the 8-bit table rounds T[k] + f * (T[k+1] - T[k]) once
            if tt not in ((0, 8, 4), (0, 16, 8)) or dims != "1" or p["InterpMethod"] != "Linear point-slope":
                _refuse(where + ".TableDataTypeStr", "an unsigned table %s (%s, %s-D): recorded with fixdt(0,8,4) / "
                        "fixdt(0,16,8), 1-D, Linear point-slope; record this shape to open it"
                        % (p["TableDataTypeStr"], p["InterpMethod"], dims))
            same_ = p["IntermediateResultsDataTypeStr"] == "Inherit: Same as output"
            if not same_ and p["IntermediateResultsDataTypeStr"] != "Inherit: Inherit via internal rule":
                _refuse(where + ".IntermediateResultsDataTypeStr", "an unsigned table with intermediate type %s: the "
                        "internal rule or 'Inherit: Same as output' recorded"
                        % p["IntermediateResultsDataTypeStr"])
            umode = "mag" if (same_ or tt[1] == 16) else "once"
        elif p["RndMeth"] in ("Zero", "Ceiling") or p["IntermediateResultsDataTypeStr"] == "Inherit: Same as output":
            _refuse(where, 'RndMeth %s / intermediate %s with a signed table: recorded with unsigned tables only'
                    "" % (p["RndMeth"], p["IntermediateResultsDataTypeStr"]))
        if dims == "2":
            # recordings 2-D (17 models): Linear interpolates along dimension 1 on both rows, each rounded into the
            # intermediate type, then along dimension 2 into the output -- the intermediate's fraction length is 10
            # for a fixdt(1,16,10) table with the internal rule, 20 with fixdt(1,32,20); for fixdt(1,8,4) with the
            # internal rule it is not determined by the records (refused). Flat and Nearest pick a corner.
            if p["InterpMethod"] == "Linear point-slope":
                if p["IntermediateResultsDataTypeStr"] == "fixdt(1,32,20)":
                    stage = 20
                elif p["TableDataTypeStr"] == "fixdt(1,16,10)":
                    stage = 10
                else:
                    stage = "undetermined"   # refused below unless the table is constant
            else:
                stage = None
        table = _nums(p["Table"] or "", where + ".Table") if dims == "1" else \
            [v for row in _rows(p["Table"] or "", where + ".Table") for v in row]
        lo, hi = (-(1 << (tt[1] - 1)), (1 << (tt[1] - 1)) - 1) if tt[0] else (0, (1 << tt[1]) - 1)
        tq = []
        for v in table:
            # within one LSB outside the type Simulink saturates the value and runs; beyond, it stops
            # (fxpParameterOverflow, checked with the model's ParameterOverflowMsg in post_compile)
            q = min(max(_round(v * (Fraction(2) ** tt[2]), "Nearest"), lo), hi)
            info.setdefault("prelookup_overflow", []).append((v, tuple(tt), where + ".Table", "table value"))
            tq.append(q)
        owners = []
        for d_ in range(int(dims)):
            a, f = into.get((n, 2 * d_ + 1)), into.get((n, 2 * d_ + 2))
            if a is None or f is None:
                _refuse(where, "inputs %d (index) and %d (fraction) must be connected" % (2 * d_ + 1, 2 * d_ + 2),
                        "BAD_CONNECTION")
            ow = [o for o, v in pls.items() if v[0] == a["src"] and v[1] == f["src"]]
            if not ow:
                _refuse(where, "inputs %d and %d must be the index and fraction outputs of one Prelookup (recorded)"
                        % (2 * d_ + 1, 2 * d_ + 2))
            owners.append(ow[0])
        sizes = [pls[o][2] for o in owners]
        if dims == "2":
            rows = _rows(p["Table"] or "", where + ".Table")
            if len(rows) != sizes[0] or any(len(r_) != sizes[1] for r_ in rows):
                _refuse(where + ".Table", "a %d x %d table is needed for %d and %d breakpoints"
                        % (sizes[0], sizes[1], sizes[0], sizes[1]), "BAD_PARAM")
        elif len(tq) != sizes[0]:
            _refuse(where + ".Table", "table of %d values for %d breakpoints" % (len(tq), sizes[0]), "BAD_PARAM")
        if len({json.dumps(pls[o][3]["ft"]) for o in owners}) != 1:
            _refuse(where, "Prelookups with different fraction types (not recorded)")
        pp = pls[owners[0]][3]
        if stage == "undetermined" if dims == "2" else False:
            if all(pls[o][3].get("mapb") for o in owners):
                stage = tt[2]   # use the table fraction length with explicit breakpoints
            else:
                _refuse(where + ".IntermediateResultsDataTypeStr", "2-D linear interpolation of a %s table with "
                        "the internal-rule intermediate type: not determined by the records -- give "
                        "IntermediateResultsDataTypeStr fixdt(1,32,20)" % p["TableDataTypeStr"])
        if any(pls[o][3].get("mapb") for o in owners) or (p["RndMeth"] == "Round" and umode is None):
            # Explicit breakpoints require the supported table and intermediate types.
            if tuple(tt) != (1, 16, 8) or not all(pls[o][3].get("mapb") for o in owners) or \
                    p["IntermediateResultsDataTypeStr"] != "Inherit: Inherit via internal rule":
                _refuse(where, "For breakpoints %s or Round interpolation, use explicit %s breakpoints on "
                        "every input, a sfix16_En8 table and output, and the internal-rule intermediate type."
                        % (MAPB_BP, MAPB_BP))
        if dims == "2" and p["InterpMethod"] == "Flat" and any(pls[o][3]["rnd"] == "Nearest" for o in owners) and \
                not all(pls[o][3].get("mapb") for o in owners):
            # a Nearest fraction can round up to 1 inside the table (Simulink then keeps T[k], 1-D
            # four designs); not recorded in two dimensions
            _refuse(where, "2-D Flat interpolation fed by a Prelookup with RndMeth Nearest: not recorded (1-D is); "
                    "use Floor on the Prelookup or record this shape to open it")
        blocks[n] = {"type": "PLInterp", "Params": json.dumps(
            {"tq": tq, "tt": list(tt), "method": p["InterpMethod"], "rnd": p["RndMeth"], "ft": pp["ft"],
             "it": pp["it"], "dims": int(dims), "cols": sizes[-1], "stage": stage if dims == "2" else None,
             "umode": umode},
            sort_keys=True)}
    for ln in lines:
        if ln.get("sport") == 1:
            ln.pop("sport")
    import parameter_bounds
    pk = dict(spec.get(parameter_bounds.PORT_KEY) or {})
    for n_, v_ in pls.items():
        pk["%s|2" % n_] = [v_[1], 1]
    out = dict(spec, blocks=blocks, lines=lines)
    if pk:
        out[parameter_bounds.PORT_KEY] = pk
    return out


def _bp_range(c, bt, mode):
    """recordings (silent wrong): the breakpoints against their type -- explicit ones through the parameter overflow
    rule (within one LSB above the maximum Simulink saturates: 128 on sfix16_En8 is 127.99609375); evenly spaced ones
    may not leave the type (PreLookupBreakpointsValueOutOfRange); saturated breakpoints must stay increasing."""
    where = "design.blocks." + c[1]
    lo, hi = (-(1 << (bt[1] - 1)), (1 << (bt[1] - 1)) - 1) if bt[0] else (0, (1 << bt[1]) - 1)
    bp = [Fraction(x) for x in c[3]]
    sc = Fraction(2) ** bt[2]
    even = len(c) > 5 and c[5] == "Even spacing"
    for i, x in enumerate(bp):
        if even:
            if i == 0:
                G._param_overflow(x, bt, where + ".BreakpointsFirstPoint", "Prelookup %s first breakpoint" % c[1],
                                  mode=mode)
            if not lo <= x * sc <= hi:
                _refuse(where, "evenly spaced breakpoint %s outside the breakpoint type %s: Simulink rejects it "
                        "(PreLookupBreakpointsValueOutOfRange)" % (x, G._tstr(bt)), "PORT_TYPE_INVALID")
        else:
            G._param_overflow(x, bt, where + ".BreakpointsData", "Prelookup %s breakpoint" % c[1], mode=mode)
    sat = [min(max(x * sc, lo), hi) / sc for x in bp]
    if any(b2 <= b1 for b1, b2 in zip(sat, sat[1:])):
        _refuse(where + ".BreakpointsData", "breakpoints %s saturated into %s are no longer increasing (not "
                "recorded)" % ([str(x) for x in bp], G._tstr(bt)))
    return sat


def post_compile(d, info, port_block):
    mode = (getattr(d, "hw", None) or {}).get("ParameterOverflowMsg", G.PARAM_OVERFLOW_DEFAULT)
    for v, tt, where, what in info.get("prelookup_overflow", []):
        G._param_overflow(v, tt, where, what, mode=mode)
    for c in info.get("prelookup_checks", []):
        s = port_block(info, c[2][0], c[2][1], "design.blocks." + c[1])
        t = d.otype.get(s)
        if len(c) > 4 and c[4]:
            if t not in MAPB_INPUTS:
                _refuse("design.blocks." + c[1], "Prelookup input of type %s with breakpoints %s: use "
                        "sfix12_En6 or sfix16_En12." % (d.type_str(s) if s in d.otype else "?", MAPB_BP))
            _bp_range(c, (1, 8, 4), mode)
            continue
        if isinstance(t, tuple):
            _bp_range(c, t, mode)   # the breakpoints are in the input's type here
        if t != (1, 16, 8):
            _refuse("design.blocks." + c[1], 'Prelookup input of type %s: recorded with sfix16_En8 only'
                    "" % (d.type_str(s) if s in d.otype else "?"))
        for x in c[3]:
            if (Fraction(x) * (Fraction(2) ** t[2])).denominator != 1:
                _refuse("design.blocks." + c[1], "breakpoint %s is not exact in the input type %s (not recorded)"
                        % (x, d.type_str(s)))


def out_type(bt, p, ts, where):
    par = json.loads(p["Params"])
    if bt == "PLCore":
        return tuple(par["it"] if par["part"] == "index" else par["ft"])
    return tuple(par["tt"])


def kernel(bt, p, ts):
    par = json.loads(p["Params"])
    if bt == "PLCore":
        t = ts[0]
        sc = Fraction(2) ** t[2]
        bp = [Fraction(x) for x in par["bp"]]
        ff = Fraction(2) ** par["ft"][2]
        index = par["part"] == "index"
        rnd = par["rnd"]

        # a breakpoint up to one LSB above its type's maximum is saturated (Simulink), as checked in
        # post_compile; the breakpoint type is the input's, or fixdt(1,8,4) for explicitly typed breakpoints
        bt = (1, 8, 4) if par.get("mapb") else t
        blo, bhi = (-(1 << (bt[1] - 1)), (1 << (bt[1] - 1)) - 1) if bt[0] else (0, (1 << bt[1]) - 1)
        bsc = Fraction(2) ** bt[2]
        bp = [min(max(x * bsc, blo), bhi) / bsc for x in bp]

        def k_pl(v):
            x = Fraction(v[0]) / sc
            x = min(max(x, bp[0]), bp[-1])
            k = max(i for i in range(len(bp) - 1) if bp[i] <= x)
            if index:
                return k
            return _round((x - bp[k]) / (bp[k + 1] - bp[k]) * ff, rnd)
        return k_pl
    tq = par["tq"]
    one = 1 << par["ft"][2]
    method, rnd = par["method"], par["rnd"]
    if par.get("dims") == 2:
        cols = par["cols"]
        T = [tq[r * cols:(r + 1) * cols] for r in range(len(tq) // cols)]
        sc = Fraction(2) ** ((par["stage"] or par["tt"][2]) - par["tt"][2])

        def k_ip2(v):
            i, f1, j, f2 = int(v[0]), Fraction(int(v[1]), one), int(v[2]), Fraction(int(v[3]), one)
            i, j = min(max(i, 0), len(T) - 2), min(max(j, 0), cols - 2)
            if method == "Flat":
                return T[i + (1 if f1 >= 1 and i == len(T) - 2 else 0)][j + (1 if f2 >= 1 and j == cols - 2 else 0)]
            if method == "Nearest":
                return T[i + (1 if 2 * f1 >= 1 else 0)][j + (1 if 2 * f2 >= 1 else 0)]
            if rnd == "Round":
                # With Round the product f * (T[k+1] - T[k]) is
                # rounded on its own and added to T[k] (a tie below zero goes away from zero there); the table sits
                # on the intermediate grid (stage = table fraction length, the recorded shape)
                a0 = Fraction(T[i][j]) * sc + _round(f1 * (T[i + 1][j] - T[i][j]) * sc, rnd)
                a1 = T[i][j + 1] * sc + _round(f1 * (T[i + 1][j + 1] - T[i][j + 1]) * sc, rnd)
                y0 = a0 / sc   # on the output grid: Round is answered with the intermediate at the table's length
                return int(y0) + _round(f2 * (a1 - a0) / sc, rnd) if y0.denominator == 1 else \
                    _round(y0 + f2 * (a1 - a0) / sc, rnd)
            a0 = _round((T[i][j] + f1 * (T[i + 1][j] - T[i][j])) * sc, rnd)
            a1 = _round((T[i][j + 1] + f1 * (T[i + 1][j + 1] - T[i][j + 1])) * sc, rnd)
            return _round((a0 + f2 * (a1 - a0)) / sc, rnd)
        return k_ip2

    def k_ip(v):
        k, f = int(v[0]), int(v[1])
        k = min(max(k, 0), len(tq) - 2)
        if method == "Flat":
            # a fraction rounded up to 1 inside the table is still T[k]; only the last interval's
            # fraction 1 (the input on or past the last breakpoint) gives the last table value
            return tq[k + 1] if f >= one and k == len(tq) - 2 else tq[k]
        if method == "Nearest":
            return tq[k + 1] if 2 * f >= one else tq[k]
        if par.get("umode") == "mag" and tq[k + 1] < tq[k]:   # an unsigned intermediate
            return tq[k] - _round(Fraction(f, one) * (tq[k] - tq[k + 1]), rnd)
        if par.get("umode") == "once":   # the 8-bit unsigned table, rounded once (Round too)
            return _round(tq[k] + Fraction(f, one) * (tq[k + 1] - tq[k]), rnd)
        if rnd == "Round":   # The product rounded on its own, then added (see the 2-D kernel)
            return tq[k] + _round(Fraction(f, one) * (tq[k + 1] - tq[k]), rnd)
        return _round(tq[k] + Fraction(f, one) * (tq[k + 1] - tq[k]), rnd)
    return k_ip

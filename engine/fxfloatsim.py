"""Simulation of designs with floating-point (double / single) signals -- the path of recordings,
Python only. Every rule here is checked against recordings (702 models with IEEE bit patterns,
the regression check); the accept predicate (engine/fxgraph_lib) opens only the recorded shapes.

A signal value is a Python float for 'double' / 'single' (a single is a float that to_single leaves unchanged), an int
(the stored integer) for a fixed-point type (s, w, f), and 0 / 1 for boolean."""
import json
import math
import re
from fractions import Fraction

import fxfloat as F

DOUBLE, SINGLE, BOOL = "double", "single", "boolean"
FLOATS = (DOUBLE, SINGLE)
INT_NAMES = {"int8": (1, 8, 0), "uint8": (0, 8, 0), "int16": (1, 16, 0), "uint16": (0, 16, 0), "int32": (1, 32, 0),
             "uint32": (0, 32, 0)}
INHERIT_RULE = "Inherit: Inherit via internal rule"


class FloatError(Exception):
    def __init__(self, code, where, message):
        Exception.__init__(self, message)
        self.code, self.where, self.message = code, where, message


def parse_type(text):
    """'double' / 'single' / 'boolean' / an integer name / fixdt(s,w,f) -> DOUBLE / SINGLE / BOOL / (s, w, f); an
    inherit rule -> the text itself (resolved by the block's rule)."""
    t = str(text).strip()
    if t in FLOATS or t == BOOL:
        return t
    if t in INT_NAMES:
        return INT_NAMES[t]
    m = re.match(r"^fixdt\(\s*([01])\s*,\s*(\d+)\s*,\s*(-?\d+)\s*\)$", t)
    if m:
        return (int(m.group(1)), int(m.group(2)), int(m.group(3)))
    if t.startswith("Inherit"):
        return t
    raise FloatError("UNSUPPORTED", None, "data type %r is not supported with floating-point signals" % t)


def tname(t):
    if t in FLOATS or t == BOOL:
        return t
    return "fixdt(%d,%d,%d)" % t


def to_float(v, t, ft):
    """value v of type t as a float of type ft (double / single): fixed-point exactly then rounded once, boolean 0 / 1,
    double -> single rounded to nearest even, single -> double exact."""
    if t in FLOATS:
        return F.to_single(v) if ft == SINGLE else v
    if t == BOOL:
        return float(v)
    return F.fixed_to_float(v, t[2], ft)


def float_to_fixed(x, t, rnd, sat):
    """recordings (4,704 finite conversions equal): the exact value rounded by RndMeth
    ('Simplest' rounds toward zero for a float input), saturated or wrapped; NaN gives 0; +-Inf the type's limit with
    saturation on, 0 with it off."""
    s, w, f = t
    if math.isnan(x):
        return 0
    if math.isinf(x):
        if not sat:
            return 0
        lo, hi = (-(1 << (w - 1)), (1 << (w - 1)) - 1) if s else (0, (1 << w) - 1)
        return hi if x > 0 else lo
    return F.float_to_fixed(x, s, w, f, "Zero" if rnd == "Simplest" else rnd, sat)


def convert(v, t, to, rnd="Floor", sat=False):
    """v of type t into type to."""
    if to in FLOATS:
        return to_float(v, t, to)
    if to == BOOL:
        if t in FLOATS:
            return 0 if (v == 0 or math.isnan(v)) and not math.isnan(v) or v == 0 else 1
        return 1 if v != 0 else 0
    if t in FLOATS:
        return float_to_fixed(v, to, rnd, sat)
    raise FloatError("UNSUPPORTED", None, "fixed-point to fixed-point conversion is not on the float path")


def bits(v, t):
    return F.bits(v, t)


def _const_value(text):
    import fxnum
    v = fxnum.canonical(str(text), "scalar")
    if isinstance(v, fxnum.NumError):
        raise FloatError(v.code, None, v.message)
    return float(str(v))


def out_type(bt, p, ts, where):
    """Output type of a block from its inputs -- recordings (every recorded compiled
    type): explicit double / single / fixdt / integer name; the internal rule (and an inherited Abs / Unary Minus /
    MinMax default) gives double if any input is double, else single if any is single; 'Same as first input' the
    first input's type (a Gain rejects it, UdtInvInherit)."""
    ot = str(p.get("OutDataTypeStr", INHERIT_RULE)).strip()
    if bt in ("Abs", "UnaryMinus"):
        ot = str(p.get("OutDataTypeStr", "Inherit: Same as input")).strip()
        if ot in ("Inherit: Same as input", INHERIT_RULE):
            return ts[0]
    t = parse_type(ot)
    if not isinstance(t, str) or t in FLOATS or t == BOOL:
        return t
    if t == "Inherit: Same as first input":
        if bt == "Gain":
            raise FloatError("BAD_TYPE", where + ".OutDataTypeStr", "Gain has no output rule 'Inherit: Same as first "
                             "input' (Simulink UdtInvInherit)")
        return ts[0]
    if t in (INHERIT_RULE, "Inherit: Same as input") or (bt == "Switch" and t.startswith("Inherit")):
        data = [ts[0], ts[2]] if bt == "Switch" else ts
        if DOUBLE in data:
            return DOUBLE
        if SINGLE in data:
            return SINGLE
        raise FloatError("UNSUPPORTED", where, "%s with no floating-point input is not on the float path" % bt)
    raise FloatError("UNSUPPORTED", where + ".OutDataTypeStr", "%s output rule %r with floating-point signals is "
                     "not recorded" % (bt, ot))


def _arith(vals, ts, ops, to, op_fn, rnd, sat):
    """A Sum / Product into type to: in a float type every input is converted to it, then combined left to right in it;
    into a fixed-point type the inputs are combined in double and the result converted (rnd, sat)."""
    # with any double input the arithmetic is in double and rounded once into a
    # single output (FS: an all-single chain rounds every step)
    work = DOUBLE if (to not in FLOATS or DOUBLE in ts) else to
    acc = None
    for v, t, o in zip(vals, ts, ops):
        x = to_float(v, t, work)
        if acc is None:
            acc = op_fn(None, x, o, work)
        else:
            acc = op_fn(acc, x, o, work)
    if to in FLOATS:
        return F.to_single(acc) if to == SINGLE else acc
    return float_to_fixed(acc, to, rnd, sat)


def canon_nan(x):
    """A NaN result is the positive quiet NaN in Simulink's output (0x7ff8... / 0x7fc00000; recordings
    FA: Inf - Inf, 0 * Inf, 0 / 0) -- not the sign the host's arithmetic leaves."""
    return math.nan if math.isnan(x) else x


def _sum_op(acc, x, o, t):
    if acc is None:
        return x if o == "+" else F.neg(x)
    return F.add(acc, x, t) if o == "+" else F.sub(acc, x, t)


def _prod_op(acc, x, o, t):
    if acc is None:
        return x if o == "*" else F.div(1.0, x, t)
    return F.mul(acc, x, t) if o == "*" else F.div(acc, x, t)


def simulate(design, inputs, nsteps):
    """design: {blocks, lines}; inputs: {Inport: (type, [values])}. Returns {Outport: (type, [values])} and the
    compiled types {block: type}."""
    design = _lower_pl(design)
    blocks, lines = design["blocks"], design["lines"]
    src = {n: {} for n in blocks}
    for ln in lines:
        src[ln["dst"]][int(ln.get("dport", 1))] = ln["src"]
    dests = {n: [ln["dst"] for ln in lines if ln["src"] == n] for n in blocks}
    types, vals, done = {}, {}, set()
    for n, (t, v) in inputs.items():
        types[n], vals[n] = t, v
        done.add(n)
    order = []
    pending = [n for n in blocks if n not in done]
    while pending:
        progress = False
        for n in list(pending):
            ins = [src[n][k] for k in sorted(src[n])]
            if all(i in done for i in ins):
                order.append(n)
                done.add(n)
                pending.remove(n)
                progress = True
                b = blocks[n]
                bt, where = b["type"], "design.blocks." + n
                ts = [types[i] for i in ins]
                xs = [vals[i] for i in ins]
                b = dict(lib_defaults(b["type"]), **b)   # parameters left out take the library default
                bp = _backprop(n, b, blocks, dests[n], where)
                types[n], vals[n] = _block(bt, dict(b, OutDataTypeStr=bp) if bp else b, ts, xs, nsteps, where)
        if not progress:
            raise FloatError("UNSUPPORTED", "design", "a feedback loop with floating-point signals is not recorded")
    outs = {n: (types[n], vals[n]) for n in blocks if blocks[n]["type"] == "Outport"}
    return outs, types


BACKPROP = "Inherit: Inherit via back propagation"


def _backprop(n, b, blocks, dst, where):
    """recordings (40 models): 'Inherit via back propagation' into Outports -- a typed Outport among
    the destinations gives its type; only untyped Outports: a Constant or a Data Type Conversion gives double (a Gain
    or Sum the input's type, the fixed-point path). Returns the resolved OutDataTypeStr, or None if not back
    propagated."""
    if str(b.get("OutDataTypeStr", "")).strip() != BACKPROP and not (
            b["type"] == "DataTypeConversion" and "OutDataTypeStr" not in b):
        return None
    if b["type"] == "Lookup1D" and dst and all(
            (blocks[d]["type"] == "Outport" and str(blocks[d].get("OutDataTypeStr", "Inherit: auto")).strip() in
             ("Inherit: auto", BACKPROP)) or
            (blocks[d]["type"] == "DataTypeConversion" and str(blocks[d].get("OutDataTypeStr", "")).strip().startswith(
                "fixdt(")) for d in dst):
        # backprop / recordings (45 designs run, host): a 1-D Lookup Table whose output
        # is back-propagated from untyped Outports or Data Type Conversions with an explicit type compiles to double
        # (neither passes a type back); the table and breakpoints keep their own types (see _lookup, form fixed_in)
        return "double"
    if b["type"] == "InterpolationUsingPrelookup" and dst and all(
            blocks[d]["type"] == "Outport" and str(blocks[d].get("OutDataTypeStr", "Inherit: auto")).strip() in
            ("Inherit: auto", BACKPROP) for d in dst):
        # recordings (96 designs run, host and ASIC): an Interpolation Using Prelookup whose output
        # is back-propagated from untyped Outports compiles to double (see _block, InterpolationUsingPrelookup)
        return "double"
    if not dst or any(blocks[d]["type"] != "Outport" for d in dst):
        raise FloatError("UNSUPPORTED", where + ".OutDataTypeStr", "back propagation on the float path is recorded "
                         "into Outports only")
    typed = sorted({str(blocks[d].get("OutDataTypeStr", "Inherit: auto")).strip() for d in dst} -
                   {"Inherit: auto", BACKPROP})
    if len(typed) > 1:
        raise FloatError("UNSUPPORTED", where, "back propagation into Outports of different types is not recorded")
    if typed:
        return typed[0]
    if b["type"] in ("Constant", "DataTypeConversion"):
        return "double"
    raise FloatError("UNSUPPORTED", where, "%s back-propagated into an untyped Outport takes its input's type "
                     "(fixed-point path)" % b["type"])


_HW = {"asic": False}   # the hardware of the request being simulated (set by in_domain / handle)
PL_F = "\x1df"   # the Prelookup's fraction port, an internal block (the fixed-point engine's lowering)


def _lower_pl(design):
    """A Prelookup's two outputs (k port 1, f port 2) as two blocks, as the fixed-point engine lowers it
    (fxgraph_lib._lower_prelookup: port 2 is the internal block 'P\x1df' of type PrelookupF)."""
    if not any(isinstance(b, dict) and b.get("type") == "Prelookup" for b in design.get("blocks", {}).values()):
        return design
    import fxgraph_lib as _G
    blocks, lines = _G._lower_prelookup(design["blocks"], design["lines"])
    return dict(design, blocks=blocks, lines=lines)


def _prelookup_kf(b, t, v):
    """(k, the fraction's stored integer, the fraction type) of a fixed-point Prelookup for one stored input value --
    the fixed-point engine's kernel (ref/fxcore._prelookup_kf, recordings:
    every k / f equal), so both paths give the same index and fraction."""
    import fxgraph_lib as _G
    s_, w_, f_ = t
    n = int(v) & ((1 << w_) - 1)
    case = {"params": b, "ins": [{"s": s_, "wl": w_, "fl": f_, "bin": format(n, "0%db" % w_)}],
            "hw": {"ProdHWDeviceType": "ASIC/FPGA->ASIC/FPGA"} if _HW["asic"] else None}
    return _G.fc._prelookup_kf(case)


def _block(bt, b, ts, xs, nsteps, where):
    rnd = str(b.get("RndMeth", "Floor"))
    sat = str(b.get("SaturateOnIntegerOverflow", "off")) == "on"
    if bt in ("Prelookup", "PrelookupF"):
        t = ts[0]
        if not isinstance(t, tuple):
            raise FloatError("UNSUPPORTED", where, "a Prelookup on a %s signal is not on the float path" % tname(t))
        try:
            kfs = [_prelookup_kf(b, t, v) for v in xs[0]] or [_prelookup_kf(b, t, 0)]
        except ValueError as e:
            raise FloatError("UNSUPPORTED", where, str(e))
        if bt == "Prelookup":
            return (0, 32, 0), [k for k, _f, _t in kfs][:len(xs[0])]
        return tuple(kfs[0][2]), [f for _k, f, _t in kfs][:len(xs[0])]
    if bt == "InterpolationUsingPrelookup":
        return _interp_pl_double(b, ts, xs, where)
    if bt == "Outport":
        ot = str(b.get("OutDataTypeStr", "Inherit: auto")).strip()
        if ot not in ("Inherit: auto", "Inherit: Inherit via back propagation"):
            t = parse_type(ot)
            if t != ts[0]:
                raise FloatError("PORT_TYPE_INVALID", where, "Outport typed %s on a %s signal" % (tname(t), tname(ts[0])))
        return ts[0], xs[0]
    if bt == "Constant":
        v = _const_value(b.get("Value", "1"))
        ot = str(b.get("OutDataTypeStr", "Inherit: Inherit from 'Constant value'")).strip()
        t = DOUBLE if ot in ("Inherit: Inherit from 'Constant value'", "Inherit: Inherit via back propagation") \
            else parse_type(ot)
        if t in FLOATS:
            return t, [F.to_single(v) if t == SINGLE else v] * nsteps
        raise FloatError("UNSUPPORTED", where, "a fixed-point Constant is not on the float path")
    if bt == "DataTypeConversion":
        ot = str(b.get("OutDataTypeStr", "Inherit: Inherit via back propagation")).strip()
        t = DOUBLE if ot == "Inherit: Inherit via back propagation" else parse_type(ot)
        if isinstance(t, str) and t.startswith("Inherit"):
            raise FloatError("UNSUPPORTED", where + ".OutDataTypeStr", "DTC output rule %r is not recorded" % ot)
        si = str(b.get("ConvertRealWorld", "Real World Value (RWV)")).startswith("Stored")
        out = []
        for v in xs[0]:
            if si:
                out.append(_convert_si(v, ts[0], t, rnd, sat))
            else:
                out.append(_convert(v, ts[0], t, rnd, sat))
        return t, out
    if bt == "Gain":
        to = out_type(bt, b, ts, where)
        K = _const_value(b.get("Gain", "1"))
        pt = str(b.get("ParamDataTypeStr", INHERIT_RULE)).strip()
        work = DOUBLE if (to not in FLOATS or ts[0] == DOUBLE) else to
        Kp = to_float(K, DOUBLE, work) if pt.startswith("Inherit") else to_float(K, DOUBLE, parse_type(pt)) \
            if parse_type(pt) in FLOATS else None
        if Kp is None:
            raise FloatError("UNSUPPORTED", where + ".ParamDataTypeStr", "a fixed-point gain parameter with "
                             "floating-point signals is not recorded")
        out = []
        for v in xs[0]:
            y = F.mul(to_float(v, ts[0], work), Kp, work)
            out.append((F.to_single(y) if to == SINGLE else y) if to in FLOATS else
                       float_to_fixed(y, to, rnd, sat))
        return to, out
    if bt in ("Sum", "Product"):
        to = out_type(bt, b, ts, where)
        ops = str(b.get("Inputs", "++" if bt == "Sum" else "**")).replace("|", "").strip()
        if ops.isdigit():
            ops = ("+" if bt == "Sum" else "*") * int(ops)
        if bt == "Product" and any(o == "/" and t not in FLOATS for o, t in zip(ops, ts)):
            # NaN divided by a fixed-point zero gave the negative NaN -- not the
            # IEEE propagation; one witness does not settle a rule
            raise FloatError("UNSUPPORTED", where, "Product dividing by a fixed-point input on the float path is not "
                             "settled by the recordings; convert the divisor to double first")
        fn = _sum_op if bt == "Sum" else _prod_op
        return to, [_arith([x[k] for x in xs], ts, ops, to, fn, rnd, sat) for k in range(nsteps)]
    if bt in ("Abs", "UnaryMinus"):
        to = out_type(bt, b, ts, where)
        out = []
        for v in xs[0]:
            x = to_float(v, ts[0], to) if to in FLOATS else None
            if x is None:
                raise FloatError("UNSUPPORTED", where, "%s into a fixed-point type on the float path" % bt)
            out.append((x if math.isnan(x) else abs(x)) if bt == "Abs" else F.neg(x))
        return to, out
    if bt == "MinMax":
        to = out_type(bt, b, ts, where)
        fn = str(b.get("Function", "min"))
        out = []
        for k in range(nsteps):
            cand = [to_float(x[k], t, to) for x, t in zip(xs, ts)]
            best = cand[0]
            for c in cand[1:]:
                if math.isnan(best) or ((c < best) if fn == "min" else (c > best)):
                    best = c
            out.append(best)
        return to, out
    if bt == "Switch":
        to = out_type(bt, b, ts, where)
        crit = str(b.get("Criteria", "u2 >= Threshold"))
        th = _const_value(b.get("Threshold", "0"))
        out = []
        for k in range(nsteps):
            c = to_float(xs[1][k], ts[1], DOUBLE)
            take1 = c >= th if crit == "u2 >= Threshold" else c > th if crit == "u2 > Threshold" else c != 0
            j = 0 if take1 else 2
            out.append(to_float(xs[j][k], ts[j], to) if to in FLOATS else _convert(xs[j][k], ts[j], to, rnd, sat))
        return to, out
    if bt in LOOKUPS:
        return _lookup(bt, b, ts, xs, nsteps, where)
    raise FloatError("UNSUPPORTED", where, "%s with floating-point signals is not recorded" % bt)


# ---- 1-D / 2-D / n-D Lookup Table on floating-point signals: recordings (425
# models, every output as IEEE hex; the regression check). The rules, each separated from its
# alternatives by recorded designs:
#  - the index search is the generated one: Linear extrapolation takes the edge interval (u <= bp[0]: the first,
#    u >= bp[-1]: the last) and f = (u - bp[i]) / (bp[i+1] - bp[i]); Clip gives f = 0 / 1 at the ends, and with
#    UseLastTableValue on the last table value at or above the last breakpoint (ignored under Linear extrapolation);
#    a NaN input falls to the last interval
#  - point-slope y = (y1 - y0) * f + y0; Lagrange y = (1 - f) * y0 + f * y1 (point-slope's form fails Lagrange designs);
#    n-D interpolates dimension 1 first, then 2, 3 (the other order fails; Lagrange as a sum of weight products fails)
#  - Flat takes the table value of the interval (at or above the last breakpoint the last value); Flat never
#    extrapolates
#  - single signals round every operation to single (computing in double and rounding once fails 17 / 17 designs)
#  - a fixed-point input (breakpoints quantised to the input's type) into a double table and output: the input and
#    breakpoints converted exactly to double, computed in double; double into a fixed-point output (Flat, Lagrange):
#    computed in double, converted once with RndMeth Simplest as toward zero
#  - Nearest: the upper value unless u - bp[i] < bp[i+1] - u in the working
#    precision (f >= 0.5 fails 11 designs, f > 0.5 27, the three midpoint forms 24, 2 (u - bp[i]) >= span 11, a strict
#    distance 27); like Flat it does not extrapolate
#  - point-slope into a fixed-point output with an explicit intermediate double / fixdt(1,32,20) / fixdt(1,40,30)
#    (64 designs, tables double / fixdt(1,16,8), inputs double / single): computed in
#    the input's float type, the table at its run-time values, cast once into the output by RndMeth, saturating or
#    wrapping (exact real arithmetic fails 3, a cast at each step 2, the product cast alone 2)
# Refused: point-slope into a fixed-point output with the intermediate 'Inherit: Same as output' (the default; no
# candidate reproduces it: recordings 28 designs, recordings), Cubic / Akima,
# non-default index search / diagnostics / types, tables given other than as a list or reshape([...],[...]).
LOOKUPS = ("Lookup1D", "Lookup2D", "LookupND")
LOOKUP_PS_IT = ("double", "fixdt(1,32,20)", "fixdt(1,40,30)")   # the explicit intermediates LS recorded
LOOKUP_KEY_PARAMS = ("InterpMethod", "ExtrapMethod", "UseLastTableValue", "TableDataTypeStr", "RndMeth",
                     "SaturateOnIntegerOverflow")
LOOKUP_FREE = {"Table", "NumberOfTableDimensions", "InterpMethod", "ExtrapMethod", "UseLastTableValue",
               "TableDataTypeStr", "OutDataTypeStr", "RndMeth", "SaturateOnIntegerOverflow", "SampleTime", "type"} | \
    {"BreakpointsForDimension%d" % k for k in range(1, 31)}
LOOKUP_FIXED = {"BreakpointsSpecification": "Explicit values", "IndexSearchMethod": "Binary search",
                "BeginIndexSearchUsingPreviousIndexResult": "off", "DiagnosticForOutOfRangeInput": "None",
                "RemoveProtectionInput": "off", "ApplyFullPrecisionForLinearInterpolation": "off",
                "IntermediateResultsDataTypeStr": "Inherit: Same as output", "TableSource": "Dialog",
                "DataSpecification": "Table and breakpoints", "UseOneInputPortForAllInputData": "off",
                "SupportTunableTableSize": "off", "TableMin": "[]", "TableMax": "[]", "OutMin": "[]", "OutMax": "[]",
                "LockScale": "off", "InputSameDT": "on"}


def _interp_pl_double(b, ts, xs, where):
    """In a sweep of 96 designs checked at every step: an Interpolation Using
    Prelookup (1-D, Clip) whose output is back-propagated to double takes the table quantised into its own type
    (nearest, ties toward +inf) and the fraction f = the fraction port's stored value / 2^fl; Linear point-slope gives
    y0 + f * (y1 - y0) in double (y0, y1 the table at k and k + 1, the last point repeated); Flat gives y1 at f = 1,
    else y0. Its span is checked in in_domain (_ipl_double_ok)."""
    if str(b.get("OutDataTypeStr", "")).strip() != "double":
        raise FloatError("UNSUPPORTED", where + ".OutDataTypeStr", "Interpolation Using Prelookup on the float path is "
                         "recorded with a back-propagated (double) output only")
    if str(b.get("NumberOfTableDimensions", "1")).strip() != "1" or len(ts) != 2:
        raise FloatError("UNSUPPORTED", where, "Interpolation Using Prelookup on the float path: 1-D only")
    tk, tf = ts
    if tk != (0, 32, 0) or not isinstance(tf, tuple):
        raise FloatError("UNSUPPORTED", where, "Interpolation Using Prelookup on the float path: inputs from a Prelookup")
    tt = parse_type(str(b.get("TableDataTypeStr", "")))
    if not isinstance(tt, tuple):
        raise FloatError("UNSUPPORTED", where + ".TableDataTypeStr", "a fixed-point table type is required here")
    T = [Fraction(_q_param(v, tt, where + ".Table", tie_up=True)) / Fraction(2) ** tt[2]
         for v in _lookup_nums(b["Table"], where + ".Table")]
    interp = str(b.get("InterpMethod", "Linear point-slope")).strip()
    out = []
    for k, fn in zip(xs[0], xs[1]):
        k = int(k)
        if not 0 <= k < len(T):
            raise FloatError("UNSUPPORTED", where, "index %d outside the table" % k)
        t0, t1 = float(T[k]), float(T[min(k + 1, len(T) - 1)])
        f = float(Fraction(int(fn)) / Fraction(2) ** tf[2])
        if interp == "Linear point-slope":
            out.append(t0 + f * (t1 - t0))
        elif interp == "Flat":
            out.append(t1 if f >= 1 else t0)
        else:
            raise FloatError("UNSUPPORTED", where + ".InterpMethod", "InterpMethod %s is not recorded here" % interp)
    return DOUBLE, out


def _lookup_nums(text, where):
    t = str(text).strip()
    if not (t.startswith("[") and t.endswith("]")) or "[" in t[1:-1] or ";" in t:
        raise FloatError("UNSUPPORTED", where, "%r: on the float path only a bracketed list of numbers is recorded" % t[:60])
    try:
        return [float(v) for v in re.split(r"[\s,]+", t[1:-1].strip()) if v]
    except ValueError:
        raise FloatError("UNSUPPORTED", where, "%r: only numeric literals are taken on the float path" % t[:60])


def _lookup_table(text, nd, where):
    t = str(text).strip()
    m = re.match(r"^reshape\(\s*(\[[^\[\]]*\])\s*,\s*(\[[^\[\]]*\])\s*\)$", t)
    if m:
        vals = _lookup_nums(m.group(1).replace(",", " "), where)
        shape = [int(v) for v in _lookup_nums(m.group(2).replace(",", " "), where)]
    elif nd == 1:
        vals = _lookup_nums(text, where)
        shape = [len(vals)]
    else:
        raise FloatError("UNSUPPORTED", where, "an n-D table on the float path is recorded as reshape([...],[...]) only")
    n = 1
    for d in shape:
        n *= d
    if len(shape) != nd or n != len(vals) or any(d < 2 for d in shape):
        raise FloatError("BAD_PARAM", where, "the table's shape %s does not fit %d dimensions" % (shape, nd))
    return vals, shape


def _q_param(v, t, where, tie_up=False):
    """A breakpoint or table value into a fixed-point type: nearest with ties toward +inf, an overflowing value refused
    (table -0.482421875 into sfix16_En8 is stored -123;
    breakpoints -122.5 / -384.5 LSB stored -122 / -384, +378.5 stored 379 -- the
    fixed-point path's rule, ref/fxcore._quant_param). tie_up is kept for the callers; both give the same rule."""
    s, w, f = t
    q = Fraction(v) * (Fraction(2) ** f) if f >= 0 else Fraction(v) / (Fraction(2) ** -f)
    fl = q.numerator // q.denominator
    d = q - fl
    n = fl + 1 if d >= Fraction(1, 2) else fl
    lo, hi = (-(1 << (w - 1)), (1 << (w - 1)) - 1) if s else (0, (1 << w) - 1)
    if not lo <= n <= hi:
        raise FloatError("BAD_PARAM", where, "breakpoint %r overflows %s (Simulink stops with fxpParameterOverflow)"
                         % (v, tname(t)))
    return n


def _lk_pre(u, bp, extrap, ultv, r):
    n = len(bp)
    if extrap == "Linear":
        if u <= bp[0]:
            i = 0
        elif u < bp[-1]:
            i = _lk_bs(u, bp)
        else:
            i = n - 2
        return i, r(r(u - bp[i]) / r(bp[i + 1] - bp[i])), False
    if u <= bp[0]:
        return 0, 0.0, False
    if u < bp[-1]:
        i = _lk_bs(u, bp)
        return i, r(r(u - bp[i]) / r(bp[i + 1] - bp[i])), False
    if ultv == "on":
        return n - 1, 0.0, True
    return n - 2, 1.0, False


def _lk_bs(u, bp):
    lo, hi = 0, len(bp) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if u < bp[mid]:
            hi = mid
        else:
            lo = mid
    return lo


# The recorded span of the 'ps_same_out' rule (recordings
# LS, recordings: all 194 designs reproduced; 3-D only in its recorded settings)
PS_SAME_TABLES = ("double", (1, 16, 2), (1, 16, 8))
PS_SAME_RND = ("Floor", "Ceiling", "Nearest", "Zero", "Convergent", "Simplest", "Round")   # Round: LS3a (18 designs)
# ASIC / FPGA (16 designs, every step equal): the same
# rule, recorded with Floor / Nearest / Simplest and outputs whose word is 8, 16 or 32 bits
PS_SAME_ASIC_RND = ("Floor", "Nearest", "Simplest")


def _ps_same_out_outside(bt, ti, tt, to, rnd, extrap, ultv, b, nd):
    """None inside the recorded span of the point-slope 'Same as output' rule; else the first field outside, as text."""
    sat_ = str(b.get("SaturateOnIntegerOverflow", "off")).strip()
    nd3_old = nd == 3 and ti == DOUBLE and tt == DOUBLE and to == (1, 16, 6) and rnd == "Simplest" and \
        extrap == "Clip" and sat_ == "off"
    # 3-D, LS3e (36 designs): single input, fixdt(1,16,8) table, fixdt(1,32,20) / fixdt(0,8,4), Floor / Nearest /
    # Convergent, saturation on and off, 3 .. 5 breakpoints per dimension, UseLastTableValue off, host
    nd3_new = nd == 3 and ti == SINGLE and tt == (1, 16, 8) and to in ((1, 32, 20), (0, 8, 4)) and \
        rnd in ("Floor", "Nearest", "Convergent") and extrap == "Clip" and ultv == "off"
    if bt not in ("Lookup1D", "Lookup2D") and not (nd3_old or nd3_new):
        # 3-D: recorded with double input and table into fixdt(1,16,6), Simplest, saturation off, Clip
        # (4 designs) and the LS3e set above
        return "a table of %d dimensions with these types (recorded: 1-D and 2-D; 3-D only double into fixdt(1,16,6) " \
               "with Simplest and saturation off, or single with a fixdt(1,16,8) table into fixdt(1,32,20) / " \
               "fixdt(0,8,4) with Floor / Nearest / Convergent)" % nd
    if tt not in PS_SAME_TABLES:
        return "table type %s (recorded: double, fixdt(1,16,2), fixdt(1,16,8))" % tname(tt)
    s, w, f = to
    if not ((s, w) == (1, 16) and 2 <= f <= 12 or to in ((0, 8, 4), (0, 12, 4), (1, 32, 20))):
        return "output type %s (recorded: fixdt(1,16,2..12), fixdt(0,8,4), fixdt(0,12,4), fixdt(1,32,20))" % tname(to)
    if rnd not in PS_SAME_RND or (nd == 3 and rnd == "Round"):
        return "RndMeth %s" % rnd
    if extrap != "Clip":
        return "ExtrapMethod %s" % extrap
    # UseLastTableValue on: recorded with every input / table type in 1-D and 2-D (LS3a / b / c), 3-D as above
    for d in range(nd):
        n = len(_lookup_nums(b.get("BreakpointsForDimension%d" % (d + 1), ""), "design"))
        if not (2 <= n <= 9 if nd == 1 else 2 <= n <= 9 if nd == 2 else (n in (3, 5) if nd3_old else 3 <= n <= 5)):
            return "%d breakpoints in dimension %d (recorded: %s)" % (n, d + 1, "2..9" if nd < 3 else "3 or 5" if
                                                                       nd3_old else "3..5")
    return None


def _fit(n, t, sat):
    s, w, _f = t
    lo, hi = (-(1 << (w - 1)), (1 << (w - 1)) - 1) if s else (0, (1 << w) - 1)
    if lo <= n <= hi:
        return n
    if sat:
        return min(max(n, lo), hi)
    n &= (1 << w) - 1
    return n - (1 << w) if s and n >= 1 << (w - 1) else n


def _lk_ps_same_out(us, bps, tb, shape, ultv, r, wk, to, rnd, sat, same, tfix=False):
    """Linear point-slope from floating-point inputs into a fixed-point output with the intermediate type 'Inherit:
    Same as output' (recordings 150, recordings 28,
    recordings). Each interpolation, dimension 1 first:
    y = fit_T(y0_T + p_T), y0_T = y0 cast into the output type T by RndMeth (saturating or wrapping), p = (y1 - y0) * f
    in floating point -- the table's type when the table is double, the input's when it is fixed point -- cast into T
    the same way, except that when the table's type is T itself p is only rounded onto T's grid (no range limit before
    the sum). f comes from the input's type; Clip: f = 0 / 1 at the ends (at or above the last breakpoint, and NaN, the
    last value directly with UseLastTableValue on); a later dimension interpolates the earlier results as values of
    T."""
    nd = len(bps)
    pre = [_lk_pre(u, bp, "Clip", ultv, r) for u, bp in zip(us, bps)]
    strides = [1]
    for d in range(1, nd):
        strides.append(strides[-1] * shape[d - 1])
    sc = 2.0 ** to[2]
    wide = (1, 256, to[2])
    # with saturation off the stages wrap in T's container word (8 / 16 / 32 / 64 bits) and only the result is wrapped
    # into T's own word length: a 12-bit output carries a 16-bit value from one dimension to the next
    # (a double table above fixdt(0,12,4)'s
    # range); the outputs recorded so far with a word of their own container width see no difference
    tc = to if sat else (to[0], next(c for c in (8, 16, 32, 64, 128) if c >= to[1]), to[2])

    def rec(d, idx):
        if d < 0:
            v = tb[sum(i * s_ for i, s_ in zip(idx, strides))]
            # a fixed-point table value into T is a fixed-to-fixed cast: Simplest is Floor there (LS2 recordings); from a
            # double table it is Zero
            return v, float_to_fixed(v, tc, "Floor" if (tfix and rnd == "Simplest") else rnd, sat)
        i, f, end = pre[d]
        i0 = list(idx)
        i0[d] = i
        if end:
            # the last value taken directly (UseLastTableValue): a later dimension still interpolates it as a value of
            # T (recordings 3-D recordings)
            v_, vT_ = rec(d - 1, i0)
            return vT_ / sc, vT_
        i1 = list(idx)
        i1[d] = i + 1
        (a, aT), (b_, _bT) = rec(d - 1, i0), rec(d - 1, i1)
        # the operands go into the working type first (a 32-bit T value is not exact in single: LS2 recordings)
        pf = wk(wk(wk(b_) - wk(a)) * f)
        pT = float_to_fixed(pf, wide, rnd, False) if same else float_to_fixed(pf, tc, rnd, sat)
        yT = _fit(aT + pT, tc, sat)
        return yT / sc, yT
    return _fit(rec(nd - 1, [0] * nd)[1], to, sat)


def _lk_eval(us, bps, tb, shape, interp, extrap, ultv, r):
    nd = len(bps)
    pre = [_lk_pre(u, bp, extrap if interp not in ("Flat", "Nearest") else "Clip", ultv, r) for u, bp in zip(us, bps)]
    strides = [1]
    for d in range(1, nd):
        strides.append(strides[-1] * shape[d - 1])

    def at(idx):
        return tb[sum(i * s for i, s in zip(idx, strides))]

    if interp == "Flat":
        return at([len(bp) - 1 if end else (i + 1 if f == 1.0 else i) for (i, f, end), bp in zip(pre, bps)])
    if interp == "Nearest":
        # recordings (36 designs: midpoints, +-1 ulp, breakpoints; dyadic and non-dyadic, double /
        # single, 1-D / 2-D / 3-D): the upper value unless u - bp[i] < bp[i+1] - u, both in the working precision --
        # a tie goes up, a NaN goes up (103 more designs, NaN at step 12)
        return at([len(bp) - 1 if end else (i if r(u - bp[i]) < r(bp[i + 1] - u) else i + 1)
                   for (i, f, end), bp, u in zip(pre, bps, us)])
    base = [len(bp) - 1 if end else i for (i, f, end), bp in zip(pre, bps)]

    def rec(d, idx):
        if d < 0:
            return at(idx)
        i, f, end = pre[d]
        i0 = list(idx)
        i0[d] = base[d]
        if end:
            return rec(d - 1, i0)
        i1 = list(idx)
        i1[d] = base[d] + 1
        a, b_ = rec(d - 1, i0), rec(d - 1, i1)
        if interp == "Linear point-slope":
            return r(r(r(b_ - a) * f) + a)
        return r(r(r(1.0 - f) * a) + r(f * b_))
    return rec(nd - 1, [0] * nd)


def _lookup(bt, b, ts, xs, nsteps, where):
    nd = 1 if bt == "Lookup1D" else 2 if bt == "Lookup2D" else int(str(b.get("NumberOfTableDimensions", "3")).strip())
    if len(ts) != nd:
        raise FloatError("BAD_CONNECTION", where, "%s with %d table dimensions needs %d inputs" % (bt, nd, nd))
    for k, v in b.items():
        if k in LOOKUP_FREE or k.startswith("_") or (k.startswith("BreakpointsForDimension") and
                                                       (k.endswith("DataTypeStr") and str(v).strip() ==
                                                        "Inherit: Same as corresponding input" or
                                                        k.endswith(("FirstPoint", "Spacing", "Source", "Min", "Max"))
                                                        and (not k.endswith("Source") or str(v).strip() == "Dialog"))):
            continue
        if k in LOOKUP_FIXED and str(v).strip() == LOOKUP_FIXED[k]:
            continue
        if k == "IntermediateResultsDataTypeStr" and str(v).strip() in LOOKUP_PS_IT:
            continue   # point-slope into a fixed-point output (below); the support domain keys it
        if k in ("InternalRulePriority", "FractionDataTypeStr"):
            continue   # used by fixed-point intermediate rules only; the float rules below do not read them
        raise FloatError("UNSUPPORTED", where + "." + k, "%s %s = %r on the float path is not recorded" % (bt, k, v))
    interp = str(b.get("InterpMethod", "Linear point-slope")).strip()
    extrap = str(b.get("ExtrapMethod", "Linear")).strip()
    ultv = str(b.get("UseLastTableValue", "off")).strip()
    rnd = str(b.get("RndMeth", "Simplest")).strip()
    sat = str(b.get("SaturateOnIntegerOverflow", "off")).strip()
    if interp not in ("Linear point-slope", "Linear Lagrange", "Flat", "Nearest"):
        raise FloatError("UNSUPPORTED", where + ".InterpMethod", "%s InterpMethod %r on the float path is not "
                         "recorded" % (bt, interp))
    if extrap not in ("Linear", "Clip") or ultv not in ("on", "off"):
        raise FloatError("UNSUPPORTED", where + ".ExtrapMethod", "%s ExtrapMethod %r is not recorded" % (bt, extrap))
    ot = str(b.get("OutDataTypeStr", "Inherit: Same as first input")).strip()
    to = ts[0] if ot == "Inherit: Same as first input" else parse_type(ot)
    if isinstance(to, str) and to not in FLOATS:
        raise FloatError("UNSUPPORTED", where + ".OutDataTypeStr", "%s output rule %r on the float path is not "
                         "recorded" % (bt, ot))
    tt_ = str(b.get("TableDataTypeStr", "Inherit: Same as output")).strip()
    tt = to if tt_ == "Inherit: Same as output" else parse_type(tt_)
    if any(t not in FLOATS for t in ts) and not (all(t == ts[0] for t in ts)):
        raise FloatError("UNSUPPORTED", where, "%s with inputs of different types is not recorded" % bt)
    linear = interp not in ("Flat", "Nearest")
    if linear and extrap == "Linear" and not (all(t in FLOATS for t in ts) and tt in FLOATS and to in FLOATS):
        raise FloatError("PORT_TYPE_INVALID", where + ".ExtrapMethod", "Simulink rejects linear extrapolation unless "
                         "the input, table and output are all floating point (Simulink:blocks:OnlyExtrapFloatAndHalf); "
                         "set ExtrapMethod to Clip")
    ti = ts[0]
    if ti in FLOATS and tt == ti and to == ti:
        form, work = "float", ti
    elif ti not in FLOATS and ti != BOOL and tt == DOUBLE and to == DOUBLE:
        form, work = "fixed_in", DOUBLE
    elif bt == "Lookup1D" and ti not in FLOATS and ti != BOOL and to == DOUBLE and tt not in FLOATS and tt != BOOL:
        # backprop (the output back-propagated to double): the table quantised into its
        # own fixed-point type, the breakpoints into the input's, then the double rule (all 45 recorded designs at
        # every step; a double table / unquantised breakpoints / (1-f)*a + f*b each fail some)
        why = _fixed_table_double_outside(ti, tt, interp, extrap, ultv)
        if why:
            raise FloatError("UNSUPPORTED", where, "Lookup1D from %s with table %s into double: %s is not recorded"
                             % (tname(ti), tname(tt), why))
        form, work = "fixed_in", DOUBLE
    elif ti == DOUBLE and tt == DOUBLE and to not in FLOATS and to != BOOL and interp != "Linear point-slope" and \
            rnd == "Simplest" and sat == "off":
        form, work = "fixed_out", DOUBLE
    elif ti in FLOATS and to not in FLOATS and to != BOOL and interp == "Linear point-slope" and \
            str(b.get("IntermediateResultsDataTypeStr", "Inherit: Same as output")).strip() in LOOKUP_PS_IT and \
            (tt == DOUBLE or (tt not in FLOATS and tt != BOOL)):
        form, work = "ps_fixed_out", ti
    elif ti in FLOATS and to not in FLOATS and to != BOOL and interp == "Linear point-slope" and \
            str(b.get("IntermediateResultsDataTypeStr", "Inherit: Same as output")).strip() == "Inherit: Same as output" \
            and (tt == DOUBLE or (tt not in FLOATS and tt != BOOL)) and all(t == ti for t in ts):
        form, work = "ps_same_out", ti
    else:
        raise FloatError("UNSUPPORTED", where, "%s with input %s, table %s, output %s and InterpMethod %s is not "
                         "recorded on the float path" % (bt, tname(ti), tname(tt), tname(to), interp))
    if form == "ps_same_out":
        why = _ps_same_out_outside(bt, ti, tt, to, rnd, extrap, ultv, b, nd)
        if why:
            raise FloatError("UNSUPPORTED", where, "%s Linear point-slope from %s into %s with the intermediate type "
                             "'Inherit: Same as output': %s is not recorded" % (bt, tname(ti), tname(to), why))
    r = (lambda v: v) if work == DOUBLE else F.to_single
    bps = []
    for d in range(nd):
        raw = _lookup_nums(b.get("BreakpointsForDimension%d" % (d + 1), ""), where + ".BreakpointsForDimension%d" % (d + 1))
        if form == "fixed_in":
            s_, w_, f_ = ti
            vs = [_q_param(v, ti, where) for v in raw]
            raw = [float(Fraction(n) / (Fraction(2) ** f_)) if f_ >= 0 else float(n * 2 ** -f_) for n in vs]
        else:
            raw = [r(v) for v in raw]
        if len(raw) < 2 or any(raw[i] >= raw[i + 1] for i in range(len(raw) - 1)):
            raise FloatError("BAD_PARAM", where + ".BreakpointsForDimension%d" % (d + 1), "breakpoints must be "
                             "strictly increasing (after quantisation)")
        bps.append(raw)
    vals, shape = _lookup_table(b.get("Table", ""), nd, where + ".Table")
    if shape != [len(bp) for bp in bps]:
        raise FloatError("BAD_PARAM", where + ".Table", "the table's shape %s does not match the breakpoints %s"
                         % (shape, [len(bp) for bp in bps]))
    if form in ("ps_fixed_out", "ps_same_out") or (form == "fixed_in" and tt != DOUBLE):
        # the table at its run-time value: a fixed-point table type quantises it (nearest), a double one keeps it
        tb = [float(Fraction(_q_param(v, tt, where + ".Table", tie_up=True)) / (Fraction(2) ** tt[2])) if tt != DOUBLE
              else v for v in vals]
    else:
        tb = [r(v) for v in vals]
    out = []
    for k in range(nsteps):
        us = [to_float(x[k], t, DOUBLE) if form == "fixed_in" else x[k] for x, t in zip(xs, ts)]
        if form == "ps_same_out":
            out.append(_lk_ps_same_out(us, bps, tb, shape, ultv, r, F.to_single if (tt != DOUBLE and ti == SINGLE)
                                       else (lambda v: v), to, rnd, sat == "on", tt == to, tt != DOUBLE))
            continue
        y = _lk_eval(us, bps, tb, shape, interp, extrap, ultv, r)
        if form == "fixed_out":
            y = float_to_fixed(y, to, "Zero", False)
        elif form == "ps_fixed_out":
            y = float_to_fixed(y, to, rnd, sat == "on")
        out.append(y)
    return to, out


def _convert(v, t, to, rnd, sat):
    if to in FLOATS:
        return to_float(v, t, to)
    if to == BOOL:
        return 0 if (t not in FLOATS and v == 0) or (t in FLOATS and v == 0) else 1
    if t in FLOATS:
        return float_to_fixed(v, to, rnd, sat)
    raise FloatError("UNSUPPORTED", None, "fixed-point to fixed-point conversion is not on the float path")


def _convert_si(v, t, to, rnd, sat):
    """ConvertRealWorld 'Stored Integer (SI)': the stored integer carried over -- a fixed-point input's stored integer
    as a float, a float input rounded into the output's stored integer."""
    if t in FLOATS and to not in FLOATS and to != BOOL:
        return float_to_fixed(v, (to[0], to[1], 0), rnd, sat)
    if t not in FLOATS and t != BOOL and to in FLOATS:
        return to_float(v, (t[0], t[1], 0), to)
    return _convert(v, t, to, rnd, sat)


# ---------------------------------------------------------------- the support domain
KEY_PARAMS = ("OutDataTypeStr", "Inputs", "RndMeth", "SaturateOnIntegerOverflow", "ParamDataTypeStr",
              "ConvertRealWorld", "Criteria", "Function")


KEY_DEFAULTS = {   # the library defaults of the rule-selecting parameters, so a parameter left out keys as its default
    "Sum": {"OutDataTypeStr": INHERIT_RULE, "Inputs": "++", "RndMeth": "Floor", "SaturateOnIntegerOverflow": "off"},
    "Product": {"OutDataTypeStr": INHERIT_RULE, "Inputs": "**", "RndMeth": "Zero", "SaturateOnIntegerOverflow": "off"},
    "Gain": {"OutDataTypeStr": INHERIT_RULE, "RndMeth": "Floor", "SaturateOnIntegerOverflow": "off",
             "ParamDataTypeStr": INHERIT_RULE},
    "Abs": {"OutDataTypeStr": "Inherit: Same as input", "RndMeth": "Floor", "SaturateOnIntegerOverflow": "on"},
    "UnaryMinus": {"SaturateOnIntegerOverflow": "on"},
    "MinMax": {"OutDataTypeStr": "Inherit: Same as first input", "Inputs": "1", "RndMeth": "Floor",
               "SaturateOnIntegerOverflow": "off", "Function": "min"},
    "Switch": {"OutDataTypeStr": "Inherit: Inherit via internal rule", "RndMeth": "Floor",
               "SaturateOnIntegerOverflow": "off", "Criteria": "u2 >= Threshold"},
    "Constant": {"OutDataTypeStr": "Inherit: Inherit from 'Constant value'"},
    "DataTypeConversion": {"OutDataTypeStr": BACKPROP, "RndMeth": "Zero", "SaturateOnIntegerOverflow": "off",
                           "ConvertRealWorld": "Real World Value (RWV)"},
    "Outport": {"OutDataTypeStr": "Inherit: auto"}}


def lib_defaults(bt):
    """The block's library defaults from the engine's registry (blockspec DEFAULTS + JSON_LIBRARY_DEFAULTS, recorded in
    recordings), then KEY_DEFAULTS for what the registry does not list."""
    import blockspec as _bs
    d = dict(KEY_DEFAULTS.get(bt, {}))
    d.update(_bs.DEFAULTS.get(bt, {}))
    d.update(_bs.JSON_LIBRARY_DEFAULTS.get(bt, {}))
    return d


def _norm(bt, k, v):
    v = str(v).strip()
    if k == "Inputs":
        v = v.replace("|", "")
        if v.isdigit():
            v = ("+" if bt == "Sum" else "*") * int(v) if bt in ("Sum", "Product") else v
    return v


def block_key(bt, b, ts, asic, dst_types):
    """The tuple a block is opened by: its type, its input types, the parameters that select a rule (with the library
    default where left out; values such as Gain / Constant / Threshold are free), the hardware and -- for back
    propagation -- its destinations' types."""
    dflt = lib_defaults(bt)
    key = [bt, [tname(t) for t in ts], [_norm(bt, k, b.get(k, dflt.get(k, ""))) for k in KEY_PARAMS], bool(asic),
           dst_types]
    if bt in LOOKUPS:   # the Lookup rule selectors (kept apart so the other blocks' keys stay as recorded)
        key.append([_norm(bt, k, b.get(k, dflt.get(k, ""))) for k in LOOKUP_KEY_PARAMS] +
                   [str(b.get("NumberOfTableDimensions", dflt.get("NumberOfTableDimensions", ""))).strip()
                    if bt == "LookupND" else ""])
        it_ = str(b.get("IntermediateResultsDataTypeStr", "Inherit: Same as output")).strip()
        if it_ != "Inherit: Same as output":   # only then, so every earlier Lookup key stays as recorded
            key.append(["IntermediateResultsDataTypeStr", it_])
    return key


def design_keys(design, inputs, asic):
    """[(block, key)] of every block of a design, from one type pass (one step of zeros)."""
    _HW["asic"] = bool(asic)
    design = _lower_pl(design)
    blocks = design["blocks"]
    zeros = {n: (t, [0.0 if t in FLOATS else 0]) for n, (t, _v) in inputs.items()}
    _outs, types = simulate(design, zeros, 1)
    src = {n: {} for n in blocks}
    for ln in design["lines"]:
        src[ln["dst"]][int(ln.get("dport", 1))] = ln["src"]
    keys = []
    for n, b in blocks.items():
        if b["type"] in ("Inport",):
            continue
        ins = [src[n][k] for k in sorted(src[n])]
        dst = sorted(str(blocks[ln["dst"]].get("OutDataTypeStr", "Inherit: auto"))
                     for ln in design["lines"] if ln["src"] == n and blocks[ln["dst"]]["type"] == "Outport")
        keys.append((n, json.dumps(block_key(b["type"], b, [types[i] for i in ins], asic, dst))))
    return keys


_DOMAIN = None


def domain():
    """engine/float_domain.json (the regression check): the block keys of every recordings design
    the simulator reproduced bit for bit."""
    global _DOMAIN
    if _DOMAIN is None:
        import os
        p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "float_domain.json")
        _DOMAIN = {json.dumps(k) for k in json.load(open(p))} if os.path.exists(p) else set()
    return _DOMAIN


def in_domain(design, inputs, asic):
    """(True, None) when every block of the design is a recorded tuple; else (False, the first block that is not)."""
    try:
        keys = design_keys(design, inputs, asic)
    except FloatError as e:
        return False, (e.where, e.message, e.code)
    ps = {n for n, k in keys if k not in domain() and _ps_same_out_key(k) and (not asic or _ps_same_asic_ok(k))}
    ps |= _ipl_double_blocks(design, inputs, asic)
    ps |= {n for n, k in keys if k not in domain() and _mixed_double_key(k)}
    ftd = {n for n, k in keys if k not in domain() and not asic and _fixed_table_double_key(k)}
    ps |= ftd
    src = {ln["dst"]: ln["src"] for ln in design["lines"]}
    # recordings (17 designs): the Data Type Conversion after such a Lookup, as recorded
    dtc_ = {n for n, k in keys if k not in domain() and src.get(n) in ftd and k in FTD_DTC_KEYS}
    ps |= dtc_
    # and the untyped Outport after that conversion (it takes the conversion's type)
    ps |= {n for n, k in keys if k not in domain() and src.get(n) in dtc_ and design["blocks"][n]["type"] == "Outport"
           and str(design["blocks"][n].get("OutDataTypeStr", "Inherit: auto")).strip() == "Inherit: auto"}
    for n, k in keys:
        if k in domain() or n in ps:
            continue
        b = design["blocks"][n]
        if b["type"] == "Outport" and src.get(n) in ps and str(b.get("OutDataTypeStr", "Inherit: auto")).strip() == \
                "Inherit: auto":
            continue   # the Outport of a point-slope 'Same as output' Lookup takes its output type unchanged
        return False, ("design.blocks." + n, "this block's floating-point shape is not recorded: " + k[:160])
    return True, None


_IPL_DOMAIN = None


def _ipl_domain():
    """engine/ipl_double_domain.json (the regression check): the recorded (InterpMethod, input type,
    hardware, number of breakpoints, table type) tuples."""
    global _IPL_DOMAIN
    if _IPL_DOMAIN is None:
        import os
        p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ipl_double_domain.json")
        _IPL_DOMAIN = {json.dumps(k) for k in json.load(open(p))} if os.path.exists(p) else set()
    return _IPL_DOMAIN


IPL_OWN = {"NumberOfTableDimensions", "Table", "TableDataTypeStr", "OutDataTypeStr", "InterpMethod", "ExtrapMethod"}
PL_OWN = {"BreakpointsData", "ExtrapMethod"}


def _ipl_double_blocks(design, inputs, asic):
    """The blocks of a back-propagated (double) Interpolation Using Prelookup fed by a fixed-point Prelookup inside the
    recorded span (exact recorded tuples): the Interpolation (1-D, Clip, table
    and InterpMethod as recorded, every other parameter at its library default), its Prelookup (dialog breakpoints,
    Clip, every other parameter at its default; fed straight by an Inport of a recorded type), the hardware and the
    number of breakpoints of a recorded tuple, and every destination an untyped Outport. Returns those block names and
    the Outports they feed; empty when anything is outside."""
    blocks, lines = design["blocks"], design["lines"]
    acc = set()
    for n, b in blocks.items():
        if not isinstance(b, dict) or b.get("type") != "InterpolationUsingPrelookup" or \
                str(b.get("OutDataTypeStr", "")).strip() != BACKPROP:
            continue
        srcs = {int(ln.get("dport", 1)): (ln["src"], str(ln.get("sport", 1)).strip()) for ln in lines if ln["dst"] == n}
        if set(srcs) != {1, 2} or srcs[1][0] != srcs[2][0] or srcs[1][1] not in ("1", "") or srcs[2][1] != "2":
            continue
        pn = srcs[1][0]
        pb = blocks.get(pn) or {}
        if pb.get("type") != "Prelookup":
            continue
        dl = lib_defaults("InterpolationUsingPrelookup")
        if any(k not in IPL_OWN and k != "type" and str(v).strip() != str(dl.get(k, v)).strip() for k, v in b.items()):
            continue
        dp = lib_defaults("Prelookup")
        if any(k not in PL_OWN and k != "type" and str(v).strip() != str(dp.get(k, v)).strip() for k, v in pb.items()):
            continue
        if str(b.get("NumberOfTableDimensions", "")).strip() != "1" or str(b.get("ExtrapMethod", "Linear")) != "Clip" \
                or str(pb.get("ExtrapMethod", "Linear")) != "Clip":
            continue
        pin = [ln["src"] for ln in lines if ln["dst"] == pn]
        if len(pin) != 1 or (blocks.get(pin[0]) or {}).get("type") != "Inport" or pin[0] not in inputs:
            continue
        t = inputs[pin[0]][0]
        if not isinstance(t, tuple):
            continue
        try:
            npts = len(_lookup_nums(pb["BreakpointsData"], "bp"))
            ntab = len(_lookup_nums(b["Table"], "table"))
        except (FloatError, KeyError):
            continue
        key = json.dumps([str(b.get("InterpMethod", "Linear point-slope")).strip(), list(t), bool(asic), npts,
                          str(b.get("TableDataTypeStr", "")).strip()])
        if npts != ntab or key not in _ipl_domain():
            continue
        outs = [ln["dst"] for ln in lines if ln["src"] in (n, pn) and ln["dst"] != n]
        if not outs or any((blocks.get(d) or {}).get("type") != "Outport" or str(
                blocks[d].get("OutDataTypeStr", "Inherit: auto")).strip() not in ("Inherit: auto", BACKPROP) for d in outs):
            continue
        acc |= {n, pn, pn + PL_F} | set(outs)
    return acc


def _fixed_table_double_outside(ti, tt, interp, extrap, ultv):
    """The recorded span of the 1-D fixed-input, fixed-table, double-output form (recordings backprop,
    45 designs, host): input fixdt 8..18 bits with a fraction of 0 up to the word length, table fixdt 10..32 bits with a
    fraction of 0 up to the word length, Flat / Nearest / Linear point-slope, ExtrapMethod Clip (Linear only with Flat
    / Nearest, where it has no effect), UseLastTableValue off. None inside, else the field outside."""
    if not (8 <= ti[1] <= 18 and 0 <= ti[2] <= ti[1]):
        return "input %s (recorded: 8 to 18 bits, fraction 0 to the word length)" % tname(ti)
    if not (10 <= tt[1] <= 32 and 0 <= tt[2] <= tt[1]):
        return "table %s (recorded: 10 to 32 bits, fraction 0 to the word length)" % tname(tt)
    if interp not in ("Flat", "Nearest", "Linear point-slope"):
        return "InterpMethod %s" % interp
    if extrap != "Clip" and interp == "Linear point-slope":
        return "ExtrapMethod %s" % extrap
    if ultv not in ("off", "on"):
        # UseLastTableValue on recorded too
        return "UseLastTableValue %s" % ultv
    return None


FTD_DTC_KEYS = {json.dumps(["DataTypeConversion", ["double"], [t_, "", "Floor", "off", "", "Real World Value (RWV)", "",
                                                               ""], False, ["Inherit: auto"]])
                for t_ in ("fixdt(1,24,12)", "fixdt(1,16,14)")}   # the two recorded (Floor, saturation off)
# DTC fixdt(0,16,8) / fixdt(1,32,16) after the double Lookup,
# Floor / Nearest / Zero, saturation off / on -- every one recorded
FTD_DTC_KEYS |= {json.dumps(["DataTypeConversion", ["double"], [t_, "", r_, s_, "", "Real World Value (RWV)", "", ""], False,
                             ["Inherit: auto"]])
                 for t_ in ("fixdt(0,16,8)", "fixdt(1,32,16)") for r_ in ("Floor", "Nearest", "Zero") for s_ in ("off", "on")}


def _mixed_double_key(k):
    """recordings (24 designs, host and ASIC): a Sum / Product / MinMax / Switch with a double
    operand (a Constant left at its library default) and fixed-point operands of 8 to 16 bits compiles to double; each
    fixed-point operand enters as its exact double value and the block computes in double (the float path's rule)."""
    kk = json.loads(k)
    if kk[0] not in ("Sum", "Product", "MinMax", "Switch") or "double" not in kk[1] or "single" in kk[1] or \
            all(t == "double" for t in kk[1]):
        return False   # only a mix: an all-double block keeps the recorded keys
    if kk[0] == "Switch" and (len(kk[1]) != 3 or "double" not in (kk[1][0], kk[1][2]) or
                              all(t == "double" for t in (kk[1][0], kk[1][2]))):
        return False   # recorded: one double and one fixed-point data port (the control port fixed point)
    if str(kk[2][0]) != INHERIT_RULE:
        return False
    for t in kk[1]:
        if t == "double":
            continue
        m = re.match(r"^fixdt\(([01]),(\d+),(-?\d+)\)$", str(t))
        if not m or not (6 <= int(m.group(2)) <= 32 and 0 <= int(m.group(3)) <= int(m.group(2))):
            # recorded: 8..16 bits and fixdt(1,6,2) / fixdt(1,24,12) / fixdt(0,32,0)
            #
            return False
    return True


def _fixed_table_double_key(k):
    """A Lookup1D key of the fixed-input, fixed-table, double-output form (span checked when simulated)."""
    kk = json.loads(k)
    if not (kk[0] == "Lookup1D" and len(kk) == 6 and len(kk[1]) == 1 and str(kk[1][0]).startswith("fixdt(") and
            kk[2][0] in (BACKPROP, "double") and (str(kk[5][3]).startswith("fixdt(") or kk[5][3] in (
                "Inherit: Same as output", "double"))):
        return False
    ti = parse_type(kk[1][0])
    tt = parse_type(kk[5][3]) if str(kk[5][3]).startswith("fixdt(") else (1, 16, 8)   # double table: span of the input
    # linear extrapolation is left to the block, which rejects it as Simulink does (OnlyExtrapFloatAndHalf)
    return _fixed_table_double_outside(ti, tt, kk[5][0], "Clip", str(kk[5][2] or "off")) is None


def _ps_same_asic_ok(k):
    """ASIC / FPGA for the point-slope 'Same as output' form: the recorded RndMeth and outputs of 8 / 16 / 32 bits, 1-D
    and 2-D (LS3b)."""
    kk = json.loads(k)
    m = re.match(r"^fixdt\(([01]),(\d+),(-?\d+)\)$", str(kk[2][0]).replace(" ", ""))
    return kk[0] in ("Lookup1D", "Lookup2D") and m is not None and int(m.group(2)) in (8, 16, 32) and \
        str(kk[5][4]) in PS_SAME_ASIC_RND


def _ps_same_out_key(k):
    """A Lookup key of the point-slope 'Same as output' form (its span is checked by _ps_same_out_outside when the
    block is simulated): float inputs, a fixed-point output, Linear point-slope, the default intermediate type."""
    kk = json.loads(k)
    if kk[0] not in ("Lookup1D", "Lookup2D", "LookupND") or len(kk) != 6:
        return False
    return all(t in FLOATS for t in kk[1]) and str(kk[2][0]).startswith("fixdt(") and kk[5][0] == "Linear point-slope"


def _float_inputs(req, design):
    """The request's inputs as {Inport: (type, values)}: a float input {"type": "double" | "single", "vals": [...] or
    "hex": [...]}, a fixed-point one {s, wl, fl, ints} (or {"type": {s, wl, fl}, "ints"}), a boolean
    {"type": "boolean", "ints": [0 / 1 ...]}."""
    import struct as _st
    out = {}
    for n, b in design["blocks"].items():
        if b.get("type") != "Inport":
            continue
        v = (req.get("inputs") or {}).get(n)
        if not isinstance(v, dict):
            raise FloatError("BAD_INPUT", "inputs." + n, "input %s is required" % n)
        t = v.get("type")
        if t in FLOATS:
            if "hex" in v:
                fmt = ">d" if t == DOUBLE else ">f"
                xs = [_st.unpack(fmt, bytes.fromhex(str(h)))[0] for h in v["hex"]]
            else:
                xs = [float(x) for x in v.get("vals", [])]
            out[n] = (t, [F.to_single(x) if t == SINGLE else x for x in xs])
        elif t == BOOL:
            out[n] = (BOOL, [1 if x else 0 for x in v.get("ints", [])])
        else:
            tt = t if isinstance(t, dict) else v
            out[n] = ((int(tt["s"]), int(tt["wl"]), int(tt["fl"])), [int(x) for x in v.get("ints", [])])
    return out


def looks_float(req, design):
    """True when the request has a floating-point input or a block names double / single (or a Constant / a Data Type
    Conversion takes its library default, which can be double): only then is the float path tried."""
    for v in (req.get("inputs") or {}).values():
        if isinstance(v, dict) and v.get("type") in FLOATS:
            return True
    for b in design.get("blocks", {}).values():
        if not isinstance(b, dict):
            continue
        for k in ("OutDataTypeStr", "ParamDataTypeStr"):
            if str(b.get(k, "")).strip() in FLOATS:
                return True
        if b.get("type") in ("Constant", "DataTypeConversion") and "OutDataTypeStr" not in b:
            return True
        if str(b.get("OutDataTypeStr", "")).strip() == BACKPROP:
            return True
    return False


def handle(req, design, asic):
    """The float path of fxgraph: None when the design is not inside the recorded domain (the fixed-point engine then
    answers or refuses it as before), else the result."""
    try:
        ins = _float_inputs(req, design)
    except (FloatError, KeyError, TypeError, ValueError):
        return None
    ok, why = in_domain(design, ins, asic)
    if not ok and len(why) > 2 and why[2] not in ("UNSUPPORTED", None) and str(why[0] or "").startswith(
            "design.blocks.") and (design.get("blocks", {}).get(str(why[0]).split(".")[2]) or {}).get("type") == \
            "Lookup1D" and str(design["blocks"][str(why[0]).split(".")[2]].get("OutDataTypeStr", "")).strip() == BACKPROP:
        # a back-propagated 1-D Lookup (it compiles to double, see _backprop): Simulink's own rejection there
        # (OnlyExtrapFloatAndHalf, recordings backprop) keeps its code
        raise FloatError(why[2], why[0], why[1])
    if not ok:
        named = any(str(b.get(k, "")).strip() in FLOATS for b in design.get("blocks", {}).values()
                    if isinstance(b, dict) for k in ("OutDataTypeStr", "ParamDataTypeStr"))
        if named or any(t in FLOATS for t, _v in ins.values()):
            # also a block that names double / single (the native replay in the recorded configuration: a DTC to double outside the
            # domain was refused with "DTC needs an explicit fixdt type")
            # a double / single input: the fixed-point engine cannot take it either, so the honest reason is the
            # Report the block and signal shape that limit floating-point support.
            if len(why) > 2 and why[2] not in ("UNSUPPORTED", None):
                # a recorded Simulink rejection (linear extrapolation with a
                # fixed-point input, table or output is OnlyExtrapFloatAndHalf) keeps its own code and reason
                raise FloatError(why[2], why[0] or "design", why[1])
            raise FloatError("UNSUPPORTED", why[0] or "design", "%s; floating-point signals are answered only on "
                             "the recorded shapes" % why[1])
        return None
    lens = {len(v) for _t, v in ins.values()}
    n = req.get("nsteps") or (min(lens) if lens else None)
    if not isinstance(n, int) or n < 1:
        raise FloatError("MISSING_FIELD", "nsteps", "nsteps is required when the design has no Inport")
    float_in = any(t in FLOATS for t, _v in ins.values())
    try:
        outs, types = simulate(design, ins, n)
    except FloatError:
        if not float_in:
            return None
        raise
    if not any(t in FLOATS for t in types.values()):
        # no double / single signal anywhere (the native FO back-propagation slice, item 1: a fixed-point Gain
        # back-propagated into Outports was answered here, past the fixed-point engine's parameter-overflow check):
        # a fixed-point design is the fixed-point engine's, whatever made it look float
        return None
    if req.get("engine") == "c":
        raise FloatError("UNSUPPORTED_WIDTH", "engine", "floating-point signals run on the Python engine only (no "
                         "C-core form); use engine auto or python")
    res = {}
    for o, (t, vs) in sorted(outs.items()):
        if t in FLOATS:
            res[o] = {"type": t, "vals": [None if (math.isnan(x) or math.isinf(x)) else x for x in vs],
                      "hex": [bits(x, t) for x in vs]}
        elif t == BOOL:
            res[o] = {"type": "boolean", "bool": vs}
        else:
            res[o] = {"type": tname(t), "s": t[0], "wl": t[1], "fl": t[2], "ints": vs}
    return {"status": "ok", "compiled_types": {b: tname(t) for b, t in sorted(types.items()) if PL_F not in b},
            "outputs": res,
            "metrics": {"nsteps": n, "nblocks": len(design["blocks"]), "engine": "python-float"},
            "note": "floating-point signals: values in vals (null for NaN / Inf), exact IEEE bit patterns in hex"}

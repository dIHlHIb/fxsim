"""hardware cost models (fpga_xc7, asic_gates) from a structural inventory of the datapath.

Inventory = what a straightforward HDL mapping of the fixed-point design contains (one multiplier per Gain /
FIR tap / Product, adders at the accumulator width, one register per delayed bit, comparators/muxes):
  mults: [(a_w, b_w, const)]   const = the constant operand's integer value (None for signal x signal);
                               a_w is the signal width, b_w the constant's (or second signal's) width
  adders: [w]   regs: [w]   cmps: [w]   muxes: [w]
Two sources: verilog_inventory() reads HDL Coder Verilog (used for calibration), design_inventory() builds the
same from a compiled engine design (used by the optimiser). Coefficients live in hwcost_model.json, fitted by
the regression check on the calibration families only."""
import json
import math
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(HERE, "hwcost_model.json")

DECL = re.compile(r"^\s*(wire|reg|input|output)\s+(signed\s+)?(?:\[(\d+):(\d+)\]\s+)?([A-Za-z_][A-Za-z0-9_]*)\s*;")
PORT = re.compile(r"^\s*(input|output)\s+(signed\s+)?(?:\[(\d+):(\d+)\]\s+)?([A-Za-z_][A-Za-z0-9_]*)\s*;")
CONST = re.compile(r"^(\d+)'(s?)([bdh])([0-9a-fA-F_]+)$")
ASSIGN = re.compile(r"^\s*assign\s+([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.*);\s*$")
NBA = re.compile(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*<=")


def _const(tok):
    m = CONST.match(tok.strip())
    if not m:
        return None
    w, s, base, digits = int(m.group(1)), m.group(2), m.group(3), m.group(4).replace("_", "")
    v = int(digits, {"b": 2, "d": 10, "h": 16}[base])
    if s and v >= 1 << (w - 1):
        v -= 1 << w
    return w, v


def verilog_inventory(path):
    width, regs = {}, set()
    inv = {"mults": [], "adders": [], "regs": [], "cmps": [], "muxes": []}
    lines = open(path).read().splitlines()
    for ln in lines:
        m = DECL.match(ln) or PORT.match(ln)
        if m:
            hi, lo = m.group(3), m.group(4)
            width[m.group(5)] = (int(hi) - int(lo) + 1) if hi is not None else 1
            if m.group(1) == "reg":
                regs.add(m.group(5))
    driven = set()
    for ln in lines:
        m = NBA.match(ln)
        if m and m.group(1) in regs:
            driven.add(m.group(1))
    body = [ln for ln in lines if not DECL.match(ln) and not PORT.match(ln)]
    for r in sorted(driven):
        pat = re.compile(r"\b%s\b" % re.escape(r))
        uses = [ln for ln in body if pat.search(ln) and not re.match(r"\s*%s\s*<=" % re.escape(r), ln)
                and not re.match(r"\s*if\b|\s*end|\s*else", ln)]
        mult_only = bool(uses) and all("*" in u for u in uses)
        inv["regs"].append((width[r], mult_only))
    stmts = " ".join(ln.split("//")[0] for ln in lines).split(";")
    for st in stmts:
        m = re.search(r"\bassign\s+([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.*)$", st, re.S)
        if not m:
            continue
        dst, rhs = m.group(1), " ".join(m.group(2).split())
        w = width.get(dst, 0)
        mm = re.match(r"^(\S+)\s*\*\s*(\S+)$", rhs)
        if mm:
            a, b = mm.group(1), mm.group(2)
            ca, cb = _const(a), _const(b)
            if ca:
                inv["mults"].append((width.get(b, 0), ca[0], ca[1], b))
            elif cb:
                inv["mults"].append((width.get(a, 0), cb[0], cb[1], a))
            else:
                inv["mults"].append((width.get(a, 0), width.get(b, 0), None, a + "*" + b))
            continue
        if re.match(r"^[A-Za-z_][A-Za-z0-9_]*\s*[-+]\s*[A-Za-z_][A-Za-z0-9_]*$", rhs):
            inv["adders"].append(w)
            continue
        for cm in re.finditer(r"([A-Za-z_][A-Za-z0-9_]*)\s*(<=|>=|==|<|>)\s*([A-Za-z0-9_']+)", rhs):
            if cm.group(2) == "==" and _const(cm.group(3)) and _const(cm.group(3))[0] == 1:
                continue  # select on a boolean: part of the mux
            inv["cmps"].append(width.get(cm.group(1), w))
        inv["muxes"] += [w] * rhs.count("?")
    return inv


def csd_digits(v):
    """Number of non-zero digits of v in canonical signed-digit form."""
    v, n = abs(v), 0
    while v:
        if v & 1:
            n += 1
            v -= 2 - (v & 3) if (v & 3) == 3 else 1
            v = -v if v < 0 else v
        v >>= 1
    return n


def _min_signed_width(v):
    w = 1
    while not (-(1 << (w - 1)) <= v <= (1 << (w - 1)) - 1):
        w += 1
    return w


def dsp_tiles(a_w, b_w, const, rule):
    """DSP48E1 count of one multiplier under the fitted mapping rule (None: maps to LUTs)."""
    if const is not None:
        v = const
        tz = (v & -v).bit_length() - 1 if v else 0
        b_eff = _min_signed_width(v >> tz) if v else 0  # trailing zeros become a shift
        if v == 0 or b_eff <= rule["const_min_width"]:
            return None
        b_w = b_eff
    if a_w < rule["min_width"] or b_w < rule["min_width"] or a_w + b_w < rule["min_out_width"]:
        return None
    A, B = rule["a_max"], rule["b_max"]

    def tiles(x, y):
        nx = 1 if x <= A else 1 + -(-(x - A) // (A - 1))
        ny = 1 if y <= B else 1 + -(-(y - B) // (B - 1))
        return nx * ny
    return min(tiles(a_w, b_w), tiles(b_w, a_w))


def _unique_mults(mults):
    """Multipliers that synthesis keeps: the same signal times constants with the same odd part are one
    multiplier plus shifts (merged by the synthesiser), so they are counted once."""
    seen, out = set(), []
    for a_w, b_w, c, src in mults:
        key = (src, c >> ((c & -c).bit_length() - 1) if c else c) if c is not None else (src, None, len(out))
        if key in seen:
            continue
        seen.add(key)
        out.append((a_w, b_w, c, src))
    return out


def features(inv, rule):
    f = {"adder_bits": sum(inv["adders"]), "reg_bits": sum(w for w, _ in inv["regs"]),
         "mult_only_reg_bits": sum(w for w, mo in inv["regs"] if mo),
         "misc_bits": sum(inv["cmps"]) + sum(inv["muxes"]), "dsp": 0, "lut_mult": 0.0, "all_mult": 0.0,
         "const_mult": 0.0, "var_mult": 0.0,
         # declared widths where no effective width is known (Verilog inventories)
         "adder_bits_eff": sum(inv["adders_eff"]) if "adders_eff" in inv else sum(inv["adders"]),
         # output conversions (experimental cost estimate, not calibrated): a saturating conversion that drops integer bits is
         # a comparator over the dropped bits plus a selector of the output word; a rounding mode other than
         # Floor / Simplest on dropped fraction bits is an adder of the output word
         "sat_cast_bits": sum(w_ + di for w_, di, df, sat, rm in inv.get("casts", ()) if sat and di),
         "rnd_cast_bits": sum(w_ for w_, di, df, sat, rm in inv.get("casts", ()) if df and rm not in ("Floor", "Simplest"))}
    for a_w, b_w, c, _ in _unique_mults(inv["mults"]):
        # partial products of an array multiplier: one row per 1 bit of the constant as written in HDL
        # (two's complement at its literal width); signal x signal: a_w x b_w
        size = bin(c % (1 << b_w)).count("1") * a_w if c is not None else a_w * b_w
        f["all_mult"] += size
        f["const_mult" if c is not None else "var_mult"] += size   # v3: signal x signal costs differ
        t = dsp_tiles(a_w, b_w, c, rule)
        if t is None:
            f["lut_mult"] += size
        else:
            f["dsp"] += t
    return f


def load_model():
    with open(MODEL) as fh:
        return json.load(fh)


def predict(inv, model=None, absorbed_bits=0):
    """absorbed_bits: adder bits the DSP48E1 pre-adders take (preadd_absorbed); they are not fabric adders for the
    DSP-mapped LUT prediction (lut_dsp). The no-DSP and gate predictions keep them."""
    model = model or load_model()
    f = features(inv, model["dsp_rule"])
    f["preadd_absorbed_bits"] = absorbed_bits
    out = {"features": f, "dsp": f["dsp"],
           # registers that only feed multipliers are taken into the DSP48E1 input registers together with the
           # register before them (empirical rule of the calibration set: FF = reg_bits - 2 x mult-only bits)
           "ff_dsp": max(0, f["reg_bits"] - 2 * f["mult_only_reg_bits"]) if f["dsp"] else f["reg_bits"]}
    for target, coef in model["linear"].items():
        g = dict(f, adder_bits=f["adder_bits"] - absorbed_bits) if target == "lut_dsp" else f
        out[target] = max(0.0, coef["c"] + sum(coef[k] * g[k] for k in coef if k != "c"))
    return out


def design_inventory(d):
    """Inventory of a compiled engine design (Design.compile done), mirroring the HDL Coder mapping that
    verilog_inventory reads: Gain = constant x signal, Discrete FIR = one constant multiplier per tap + adders
    at the accumulator width + tap registers, Sum with m inputs = m - 1 adders at the accumulator width,
    delays = registers, Product = signal x signal, comparisons / selections = comparators and muxes."""
    import fxgraph_lib as G
    fc = G.fc
    inv = {"mults": [], "adders": [], "regs": [], "cmps": [], "muxes": [], "casts": []}
    users = {n: [] for n in d.names}
    for n in d.names:
        for s in d.src[n]:
            users[s].append(n)

    def w(n):
        t = d.otype[n]
        return 1 if t == G.BOOL else t[1]
    for n in d.names:
        bt, p = d.btype[n], d.params[n]
        ts = [d.otype[s] for s in d.src[n]]
        if bt == "Gain" and ts[0] != G.BOOL:
            rec = d.gain_record(n)   # the compiled record (engine/fxrecords.GainRecord)
            pt, npar = rec.param_type, rec.param_stored
            if npar != 0 and abs(npar) & (abs(npar) - 1):  # a power of two is a shift in HDL, not a multiplier
                # an unsigned parameter times a signed signal is written as a signed constant one bit wider
                inv["mults"].append((ts[0][1], pt[1] + (1 if not pt[0] and ts[0][0] else 0), npar, d.src[n][0]))
        elif bt == "Sum":
            at = getattr(d, "acc_types", {}).get(n) or d.otype[n]
            inv["adders"] += [at[1]] * (len(ts) - 1)
            # effective width (experimental cost estimate): bits above the widest input's integer range + ceil(log2 inputs) are
            # sign-extension copies and fraction bits below every input's LSB are zeros; synthesis prunes both
            # (30- and 50-bit accumulators give the same LUT count)
            if ts and all(t != G.BOOL for t in ts):
                ib = max(t[1] - t[2] for t in ts)
                fb = min(at[2], max(t[2] for t in ts))
                weff = min(at[1], ib + int(math.ceil(math.log2(max(2, len(ts))))) + fb)
                inv.setdefault("adders_eff", []).extend([weff] * (len(ts) - 1))
            inv.setdefault("adder_roles", []).append((n, at[1], len(ts) - 1))
            if len(ts) == 2 and all(t != G.BOOL for t in ts):
                inv.setdefault("sum2", []).append((n, at, d.otype[n], ts))
        elif bt == "DiscreteFir":
            xt = ts[0]
            rec = d.fir_record(n)   # the compiled record (engine/fxrecords.FirRecord)
            coefs, ctp, at = list(d.param_record(n).coefs), rec.coef_type, rec.acc_type
            for k, c in enumerate(coefs):
                q = G._quant_param(c, *ctp)
                if q and abs(q) & (abs(q) - 1):
                    inv["mults"].append((xt[1], ctp[1] + (1 if not ctp[0] and xt[0] else 0), q, n + "/tap%d" % k))
            inv["adders"] += [at[1]] * (len(coefs) - 1)
            inv["regs"] += [(xt[1], False)] * (len(coefs) - 2) + ([(xt[1], True)] if len(coefs) > 1 else [])
        elif bt in G.DELAYS:
            k = int(p["DelayLength"]) if bt == "Delay" else 1
            mult_only = bool(users[n]) and all(d.btype[u] in ("Gain", "Product") for u in users[n])
            inv["regs"] += [(w(n), False)] * (k - 1) + [(w(n), mult_only)]
        elif bt == "Product":
            if len(ts) == 2 and all(t != G.BOOL for t in ts):
                inv["mults"].append((ts[0][1], ts[1][1], None, "*".join(d.src[n])))
        elif bt == "Saturation":
            inv["cmps"] += [w(n)] * 2
            inv["muxes"] += [w(n)] * 2
        elif bt == "Abs":
            inv["cmps"].append(ts[0][1])
            inv["muxes"].append(w(n))
        elif bt == "MinMax":
            inv["cmps"] += [w(n)] * (len(ts) - 1)
            inv["muxes"] += [w(n)] * (len(ts) - 1)
        elif bt in ("Switch", "MultiportSwitch"):
            inv["muxes"] += [w(n)] * (len(ts) - 2 if bt == "MultiportSwitch" else 1)
        elif bt in ("RelationalOperator", "CompareToConstant"):
            inv["cmps"].append(max(t[1] for t in ts if t != G.BOOL) if any(t != G.BOOL for t in ts) else 1)
        if bt == "Gain" and ts and ts[0] != G.BOOL and inv["mults"] and inv["mults"][-1][3] == d.src[n][0]:
            inv.setdefault("gain_mult", {})[n] = inv["mults"][-1]
        inner = _inner_type(d, n, bt, ts)
        if inner is not None and d.otype[n] != G.BOOL:
            inv["casts"].append(_cast_item(inner, d.otype[n], p))
    return inv


def full_precision_sum(ts):
    """Exact type of the sum of two inputs: every fraction bit of both and one more integer bit than the wider."""
    s = 1 if any(t[0] for t in ts) else 0
    f = max(t[2] for t in ts)
    ib = max(t[1] - t[2] + (1 if s and not t[0] else 0) for t in ts) + 1
    return (s, ib + f, f)


def keeps(t, need):
    """Type t holds every value of type need (no fraction bit dropped, no integer bit dropped)."""
    if t[0] < need[0]:
        return False
    return t[2] >= need[2] and (t[1] - t[2] - t[0]) >= (need[1] - need[2] - need[0])


def preadd_absorbed(d, inv, rule):
    """Estimate the adder bits absorbed by DSP48E1 pre-adders feeding mapped Gains."""
    if not inv.get("sum2"):
        return {}
    users = {x: [] for x in d.names}
    for x in d.names:
        for s_ in d.src[x]:
            users[s_].append(x)
    gm = inv.get("gain_mult", {})
    out = {}
    for n, at, ot, ts in inv.get("sum2", ()):
        need = full_precision_sum(ts)
        m_ok = users[n] and all(d.btype[u] == "Gain" and gm.get(u) is not None
                                and dsp_tiles(gm[u][0], gm[u][1], gm[u][2], rule) not in (None, 0) for u in users[n])
        # This mapping requires equal accumulator and output types and a coefficient
        # narrower than the pre-add after removing trailing zero bits.
        if m_ok and at[1] <= rule.get("a_max", 25) and tuple(at) == tuple(ot) and \
                all(_const_eff_width(gm[u]) < at[1] for u in users[n]):
            out[n] = at[1]
    return out


def _const_eff_width(m):
    """Effective width of a constant multiplier's constant: its HDL width less its trailing zero bits."""
    q = abs(m[2] or 0)
    tz = (q & -q).bit_length() - 1 if q else 0
    return m[1] - tz


def adder_roles(d, inv, rule):
    """Role of each Sum's adders for the DSP48E1 mapping (experimental cost estimate, not calibrated): 'preadd' when the Sum
    only feeds Gains mapped to DSP48E1 and fits the 25-bit pre-adder, 'postadd' when one of its inputs is such a
    Gain and it fits the 48-bit post-adder, else 'fabric'. Returns {role: adder bits}."""
    users = {x: [] for x in d.names}
    for x in d.names:
        for s_ in d.src[x]:
            users[s_].append(x)
    gm = inv.get("gain_mult", {})

    def dsp_gain(g):
        m = gm.get(g)
        return m is not None and dsp_tiles(m[0], m[1], m[2], rule) not in (None, 0)
    out = {"preadd": 0, "postadd": 0, "fabric": 0}
    for n, w_, k in inv.get("adder_roles", ()):
        if users[n] and all(d.btype[u] == "Gain" and dsp_gain(u) for u in users[n]) and w_ <= 25:
            out["preadd"] += w_ * k
        elif any(d.btype[s_] == "Gain" and dsp_gain(s_) for s_ in d.src[n]) and w_ <= 48:
            out["postadd"] += w_ * k
        else:
            out["fabric"] += w_ * k
    return out


def _inner_type(d, n, bt, ts):
    """Type a block computes in before its output conversion (None: no conversion logic of its own)."""
    import fxgraph_lib as G
    if any(t == G.BOOL for t in ts) or not ts:
        return None
    if bt == "Sum":
        return getattr(d, "acc_types", {}).get(n)
    if bt == "Gain":
        pt = d.gain_record(n).param_type
        return (1 if (pt[0] or ts[0][0]) else 0, ts[0][1] + pt[1], ts[0][2] + pt[2])
    if bt == "Product" and len(ts) == 2:
        return (1 if (ts[0][0] or ts[1][0]) else 0, ts[0][1] + ts[1][1], ts[0][2] + ts[1][2])
    if bt == "DataTypeConversion":
        return ts[0]
    return None


def _cast_item(inner, out, p):
    """(output word, integer bits dropped, fraction bits dropped, saturate, rounding) of one output conversion."""
    di = max(0, (inner[1] - inner[2] - inner[0]) - (out[1] - out[2] - out[0]))
    df = max(0, inner[2] - out[2])
    return (out[1], di, df, str(p.get("SaturateOnIntegerOverflow", "off")) == "on", str(p.get("RndMeth", "Floor")))


# ---- structural terms, measured on the synthesised pre-add FIR stand-ins -------------------------
# recordings. predict() above is the calibrated model and is unchanged; predict_struct() keeps
# its coefficients and replaces three inventory quantities by structural ones:
#   * Sum adders: the width of each stage of the HDL adder chain after aligning the operands to the accumulator
#     (bits below both operands' LSB are a copy of one operand, bits above the accumulator MSB are discarded)
#     instead of (inputs - 1) x accumulator word;
#   * pre-adds the DSP48E1 takes (preadd_absorbed) leave the fabric with their aligned width;
#   * registers: only the bits some consumer reads (bit liveness along the delay chain) count; a register whose
#     consumers are all absorbed pre-adds goes into the DSP input register, mult-only registers as before.
# No coefficient is fitted to the stand-in measurements.

def _bit_span(t):
    """(lsb, msb) bit positions of a fixed-point type (msb = sign bit position for signed types)."""
    return -t[2], t[1] - t[2] - 1


def sum_stage_widths(ts, at):
    """Adder width of every stage of a chain that adds the inputs ts one by one in accumulator type at: an input's
    bits below the accumulator LSB are floored away, bits above its MSB wrap; a stage adds from the lower of the two
    operands' LSBs only where both have bits (below that the result is a copy) up to one bit over the wider operand
    (capped at the accumulator MSB)."""
    alsb, amsb = _bit_span(at)

    def span(t):
        lo, hi = _bit_span(t)
        return max(lo, alsb), min(hi, amsb)
    plo, phi = span(ts[0])
    out = []
    for t in ts[1:]:
        ilo, ihi = span(t)
        top = min(amsb, max(phi, ihi) + 1)
        out.append(max(1, top - max(plo, ilo) + 1))
        plo, phi = min(plo, ilo), top
    return out


def live_register_bits(d):
    """{register: bits some consumer reads}. A Sum reads an input between the accumulator's LSB and MSB, a delay
    reads what its own consumers read, other blocks read the whole word."""
    import fxgraph_lib as G
    users = {n: [] for n in d.names}
    for n in d.names:
        for s_ in d.src[n]:
            users[s_].append(n)
    memo = {}

    def need(n, stack=()):
        if n in memo:
            return memo[n]
        t = d.otype[n]
        if t == G.BOOL or n in stack:
            return _bit_span(t) if t != G.BOOL else (0, 0)
        lo, hi = None, None
        for u in users[n]:
            bt = d.btype[u]
            if bt == "Sum" and (getattr(d, "acc_types", {}).get(u) or d.otype[u]) != G.BOOL:
                at = getattr(d, "acc_types", {}).get(u) or d.otype[u]
                ulo = max(_bit_span(t)[0], _bit_span(at)[0])
                uhi = min(_bit_span(t)[1], _bit_span(at)[1])
            elif bt in G.DELAYS:
                ulo, uhi = need(u, stack + (n,))
            else:
                ulo, uhi = _bit_span(t)
            lo = ulo if lo is None else min(lo, ulo)
            hi = uhi if hi is None else max(hi, uhi)
        memo[n] = _bit_span(t) if lo is None else (lo, hi)
        return memo[n]
    out = {}
    for n in d.names:
        if d.btype[n] in G.DELAYS:
            lo, hi = need(n)
            k = int(d.params[n].get("DelayLength", 1)) if d.btype[n] == "Delay" else 1
            out[n] = k * max(0, hi - lo + 1) if d.otype[n] != G.BOOL else k
    return out


def predict_struct(d, inv=None, model=None):
    """predict() with the structural terms above. Returns the same keys plus 'struct' (the replaced quantities)."""
    import fxgraph_lib as G
    model = model or load_model()
    inv = inv or design_inventory(d)
    rule = model["dsp_rule"]
    absorbed = preadd_absorbed(d, inv, rule)
    stage = {}
    for n in d.names:
        if d.btype[n] == "Sum":
            ts = [d.otype[s_] for s_ in d.src[n]]
            at = getattr(d, "acc_types", {}).get(n) or d.otype[n]
            if len(ts) > 1 and at != G.BOOL and all(t != G.BOOL for t in ts):
                stage[n] = sum_stage_widths(ts, at)
    inv2 = dict(inv)
    declared = {n: (w_, k) for n, w_, k in inv.get("adder_roles", ())}
    inv2["adders"] = list(inv["adders"])
    for n, ws in stage.items():   # replace the declared adders of these Sums by their stage widths
        w_, k = declared[n]
        for _ in range(k):
            inv2["adders"].remove(w_)
        inv2["adders"] += ws
    users = {n: [] for n in d.names}
    for n in d.names:
        for s_ in d.src[n]:
            users[s_].append(n)
    live = live_register_bits(d)
    gm = inv.get("gain_mult", {})

    def dsp_gain(g):
        m = gm.get(g)
        return m is not None and dsp_tiles(m[0], m[1], m[2], rule) not in (None, 0)
    into_dsp = {n: live[n] for n in live if users[n] and all(u in absorbed for u in users[n])}
    def stages(n):   # register stages of a delay (a Delay of length k holds k words)
        return int(d.params[n].get("DelayLength", 1)) if d.btype[n] == "Delay" else 1
    # an input pipeline holds at most two stages a multiplier port (AREG / BREG <= 2)
    mult = {n for n in live if users[n] and all(d.btype[u] in ("Gain", "Product") and dsp_gain(u) for u in users[n])}
    mult_only = {n: live[n] * min(stages(n), 2) // stages(n) for n in mult}
    # two-stage input register: a delay whose readers are
    # DSP multipliers and the next delay of a chain that itself only feeds DSP multipliers is copied into the
    # second multiplier's input pipeline (AREG / BREG = 2) -- no logic flip-flop is left; whatever of the two stages
    # the next delay leaves free
    chain2 = {}
    for n in live:
        nxt = [u for u in users[n] if u in mult]
        if n in mult or not nxt or not all(u in mult or (d.btype[u] in ("Gain", "Product") and dsp_gain(u))
                                           for u in users[n]):
            continue
        free = min(2 - min(stages(u), 2) for u in nxt)
        if free > 0:
            chain2[n] = live[n] * min(stages(n), free) // stages(n)
    absorbed_bits = sum(sum(stage.get(n, [w_])) for n, w_ in absorbed.items())
    out = predict(inv2, model, absorbed_bits=absorbed_bits)
    reg_live = sum(live.values())
    # each absorbed register counted once: the multiplier-only delays (the old 2x proxy stood in for the chain's first
    # stage) and the first stage of two-stage chains; same predictions on the 22 stand-in candidates, biquad FF 0
    out["ff_dsp"] = max(0, reg_live - sum(into_dsp.values()) - sum(mult_only.values()) - sum(chain2.values())) \
        if out["dsp"] \
        else reg_live
    out["ff_nodsp"] = max(0.0, model["linear"]["ff_nodsp"]["c"] + model["linear"]["ff_nodsp"]["reg_bits"] * reg_live) \
        if "ff_nodsp" in model["linear"] else reg_live
    out["struct"] = {"adder_bits_declared": sum(inv["adders"]), "adder_bits_aligned": sum(inv2["adders"]),
                     "preadd_absorbed": sorted(absorbed), "preadd_absorbed_bits": absorbed_bits,
                     "reg_bits_declared": sum(w_ for w_, _ in inv["regs"]), "reg_bits_live": reg_live,
                     "reg_bits_into_dsp": sum(into_dsp.values()), "reg_bits_mult_only": sum(mult_only.values()),
                     "reg_bits_chain2": sum(chain2.values())}
    return out

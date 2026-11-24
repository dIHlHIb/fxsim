"""lower a multirate design to an equivalent single-rate design at the base step, so that the optimiser,
the C core, code generation and the range proof all apply unchanged.

Every signal of the lowered design is the multirate signal held between its hits (zero-order hold), sampled at
every base step. Stateless blocks need no change (their inputs are held, so recomputing gives the held value).
What changes (k = base-step index, E(P, o) = "k is a hit of period P steps, offset o steps", built from a Counter
Limited and a Compare To Constant; all added blocks are internal and never decision variables):
  * Inport with period P > 1 or offset o: the caller's samples are expanded to the base step (hold; 0 before the
    first hit) -- see expand_inputs();
  * Unit Delay / Delay(N) in a domain with period P > 1: each one-sample stage becomes
        H = Hold(x, E);  y = Hold(UnitDelay(H, ic), E, ic)
    (at a hit, UnitDelay(H) still holds the input of the previous hit);
  * Downsample K, offset o (input period Pi, output period Po = K Pi):
        o = 0: y = Hold(x, E(Po, 0), ic);  o > 0: L = Hold(x, E(Po, o Pi), ic), y = Hold(L, E(Po, 0), ic);
        single-rate: y = Hold(x, E(K Pi, o Pi), ic);
  * Upsample L, offset o (output period Po = Pi / L): y = Hold(Gate(x, E(Pi, o Po)), E(Po, 0)) (the Hold is
    dropped when Po is the base step);
  * Rate Transition: fast -> slow y = Hold(x, E(Po, 0)); slow -> fast: Integrity + Deterministic
    y = Hold(UnitDelay(Hold(x, E(Pi)), ic), E(Pi), ic); Integrity only y = Hold(UnitDelay(Hold(x, E(Po)), ic), E(Po), ic);
    Integrity off y = x.
FIR / CIC multirate filters are not lowered yet (UNSUPPORTED for the optimiser; fxgraph simulates them).
The lowered design is checked against engine/fxmultirate.py by the regression check."""
from fractions import Fraction

import fxgraph_lib as G
import fxmultirate as MR

INTERNAL = "\x1e"   # prefix of every block the lowering adds (cannot appear in model block names)


def _steps(x, base):
    v = x / base
    if v.denominator != 1:
        raise G.GraphError("UNSUPPORTED_RATE", "design", "period not a multiple of the base step")
    return int(v)


def lower(spec, hw=None, in_types=None):
    """Returns (lowered spec, info) with info = {"base", "period_steps", "offset_steps", "internal", "carrier",
    "filters"}. in_types ({Inport: (s, w, f)}) is needed when the design has FIR / CIC multirate filters (their
    internal types are resolved from the compiled input type and written explicitly into the lowering)."""
    d = G.Design(spec, hw)
    # declared port types: only those sitting on an Inport are carried into the lowered design (the Inport keeps
    # its name and type there); one on a computed signal is refused
    port_types = [e for e in spec.get("port_types") or [] if isinstance(e, dict)]
    if d.multirate and any(d.btype.get(e.get("block")) != "Inport" for e in port_types):
        raise G.GraphError("UNSUPPORTED", "design.port_types", "typed subsystem ports on a computed signal in a "
                           "multirate design")
    if not d.multirate:
        return spec, {"base": Fraction(1), "period_steps": {}, "offset_steps": {}, "internal": []}
    per = MR.rates(d)
    offs = d.mr_offsets
    import fxhdl
    fxhdl.check_control_rates(spec, per, offs)
    filters = [n for n in d.names if d.btype[n] in ("FIRDecimation", "FIRInterpolation", "CICDecimation",
                                                    "CICInterpolation")]
    if filters:
        if in_types is None:
            raise G.GraphError("MISSING_FIELD", "inputs", "input types are needed to lower multirate filters")
        dbl = [(b, k) for b, p in spec["blocks"].items() for k, v in p.items()
               if k.endswith("DataTypeStr") and str(v).strip() in ("double", "single")]
        if dbl:
            # fxopt's start types 'double' downstream of the filters do not change the filters' inner
            # types; the filters are resolved on a copy with a placeholder fixed type there (the lowering itself copies
            # the user's blocks, line below). Upstream of a filter they would: refused with the reason
            ups, todo = set(), list(filters)
            while todo:
                x = todo.pop()
                for y in d.src[x]:
                    if y not in ups:
                        ups.add(y)
                        todo.append(y)
            bad = [(b, k) for b, k in dbl if b in ups or b in filters]
            if bad:
                raise G.GraphError("START_TYPE_NEEDED", "design.blocks.%s.%s" % bad[0], "start type %r upstream of the "
                                   "multirate filter(s) %s: their inner word lengths follow this type -- give it a "
                                   "fixed-point start type (fixdt)" % (spec["blocks"][bad[0][0]][bad[0][1]],
                                                                       ", ".join(sorted(filters))))
            blks = {b: dict(p, **{k: "fixdt(1,32,16)" for b2, k in dbl if b2 == b}) for b, p in spec["blocks"].items()}
            dc = G.Design(dict(spec, blocks=blks), hw)
            MR.rates(dc)
            dc.compile(in_types)
            d = dc
        else:
            d.compile(in_types)
    vals = [p for p in per.values() if p is not None] + [o for o in offs.values() if o]
    base = vals[0]
    for v in vals[1:]:
        base = MR._gcd(base, v)
    ps = {n: (_steps(per[n], base) if per[n] is not None else 1) for n in d.names}
    os_ = {n: _steps(offs[n], base) for n in d.names}
    blocks = {n: dict(d.params[n], type=d.btype[n]) for n in d.names}
    for n in d.names:   # user blocks keep only what the user gave (defaults re-applied by Design)
        blocks[n] = dict(spec["blocks"][n])
    lines = []
    internal = []
    enables = {}

    def add(name, blk):
        blocks[name] = blk
        internal.append(name)
        return name

    def enable(P, o):
        key = (P, o)
        if key in enables:
            return enables[key]
        if P > (1 << 32):   # the enable counter would need more than 32 bits
            raise G.GraphError("UNSUPPORTED_RATE", "design", "a sample period of %d base steps needs an enable counter "
                               "wider than 32 bits (rates too far apart for the single-rate lowering)" % P)
        c = add(INTERNAL + "cnt%d_%d" % key, {"type": "CounterLimited", "uplimit": str(P - 1)})
        e = add(INTERNAL + "en%d_%d" % key, {"type": "CompareToConstant", "relop": "==", "const": str(o)})
        lines.append({"src": c, "dst": e, "dport": 1})
        enables[key] = e
        return e

    def hold(tag, x, en, ic="0"):
        h = add(INTERNAL + tag, {"type": "Hold", "InitialCondition": str(ic)})
        lines.append({"src": x, "dst": h, "dport": 1})
        lines.append({"src": en, "dst": h, "dport": 2})
        return h

    def udelay(tag, x, ic):
        u = add(INTERNAL + tag, {"type": "UnitDelay", "InitialCondition": str(ic)})
        lines.append({"src": x, "dst": u, "dport": 1})
        return u

    # every original edge, possibly redirected through the lowered structure of its destination
    replace_src = {}   # original block -> name that now carries its output
    pending = []       # (dst, port, src) edges into blocks that stay
    for n in d.names:
        bt, p = d.btype[n], d.params[n]
        src = d.src[n]
        where = "design.blocks." + n
        if bt in ("FIRDecimation", "FIRInterpolation"):
            y = _lower_fir(n, d, ps, ("SRC", src[0]), add, lines, enable, hold, udelay, where)
            blocks.pop(n)
            replace_src[n] = y
            continue
        if bt in ("CICDecimation", "CICInterpolation"):
            y = _lower_cic(n, d, ps, ("SRC", src[0]), add, lines, enable, hold, udelay, where)
            blocks.pop(n)
            replace_src[n] = y
            continue
        if ps.get(n, 1) > 1 and bt in ("DiscreteFir", "CounterLimited", "CounterFreeRunning", "DetectChange",
                                       "DetectIncrease", "DetectDecrease", "DiscreteIntegrator"):
            # internal state advances only on the block's hits: the engine gates its state updates
            blocks[n]["_GateSteps"] = "%d,%d" % (ps[n], os_[n])
            blocks[n]["_GateSlot"] = enable(ps[n], os_[n])
            # its output must also hold between hits (the state already moved on after the hit)
            replace_src[n] = hold(n + "_hold", ("RAW", n), blocks[n]["_GateSlot"])
        if bt in G.DELAYS and ps[n] > 1:
            L = int(p["DelayLength"]) if bt == "Delay" else 1
            ic = p["InitialCondition"]
            if os_[n]:
                raise G.GraphError("UNSUPPORTED_RATE", where, "delay in an offset domain")
            en = enable(ps[n], 0)
            x = ("SRC", src[0])
            for k in range(L):
                h = hold("%s_in%d" % (n, k), x, en)
                u = udelay("%s_z%d" % (n, k), h, ic)
                x = hold("%s_out%d" % (n, k), u, en, ic)
            blocks.pop(n)
            replace_src[n] = x
            continue
        if bt == "Downsample":
            K, o = int(p["DownsamplingFactor"]), int(p["SampleOffset"])
            ic = p["InitialConditions"]
            pi = ps[src[0]]
            x = ("SRC", src[0])
            if p["RateOptions"] != "Allow multirate processing":
                # offset 0: enabled at step 0, the initial condition is never output (graph 19 of
                # ran with one beyond the type) -- the Hold starts at 0
                y = hold(n + "_y", x, enable(K * pi, o * pi), "0" if o == 0 else ic)
            elif o == 0:
                # the hold is enabled at step 0, so the initial condition is never output -- and Simulink does not
                # check it; recordings (069197963, two cases): this lowering carried it into
                # the Hold, whose overflow check refused what fxgraph answers (fxcodegen PORT_TYPE_INVALID)
                y = hold(n + "_y", x, enable(ps[n], 0), "0")
            else:
                lt = hold(n + "_latch", x, enable(ps[n], o * pi), ic)
                y = hold(n + "_y", lt, enable(ps[n], 0), ic)
            blocks.pop(n)
            replace_src[n] = y
            continue
        if bt == "Upsample":
            L, o = int(p["UpsamplingFactor"]), int(p["SampleOffset"])
            po, pi = ps[n], ps[src[0]]
            g = add(INTERNAL + n + "_gate", {"type": "Gate"})
            lines.append({"src": ("SRC", src[0]), "dst": g, "dport": 1})
            lines.append({"src": enable(pi, o * po), "dst": g, "dport": 2})
            y = hold(n + "_y", g, enable(po, 0)) if po > 1 else g
            blocks.pop(n)
            replace_src[n] = y
            continue
        if bt == "RateTransition":
            po, pi = ps[n], ps[src[0]]
            ic = p["InitialCondition"]
            if os_.get(src[0], 0) and p.get("Integrity") == "on":
                # Simulink outputs InitialCondition until an offset input's first hit; the lowering's
                # Hold latches the not-yet-sampled 0 -- refused here (fxgraph simulates it)
                try:
                    ic_zero = float(str(ic)) == 0.0
                except ValueError:
                    ic_zero = False
                if not ic_zero:
                    raise G.GraphError("UNSUPPORTED_RATE", "design.blocks." + n, "a Rate Transition with Integrity "
                                       "on, a non-zero InitialCondition and an input with a sample-time offset is "
                                       "simulated by fxgraph but not lowered for fxopt / fxcodegen")
            x = ("SRC", src[0])
            if po >= pi:
                # fast to slow: the Hold is enabled at step 0 -- with no input offset the initial condition is
                # never output (Simulink runs one beyond the type), so it starts at 0
                y = hold(n + "_y", x, enable(po, 0), "0" if not os_.get(src[0], 0) else ic)
            elif p["Integrity"] != "on":
                y = x
            else:
                pe = pi if p["Deterministic"] == "on" else po
                en = enable(pe, 0)
                h = hold(n + "_in", x, en, ic)
                u = udelay(n + "_z", h, ic)
                y = hold(n + "_y", u, en, ic)
            blocks.pop(n)
            replace_src[n] = y
            continue
        if bt == "Inport":
            blocks[n] = {k: v for k, v in blocks[n].items() if k != "SampleTime"}
            continue
        for k, s in enumerate(src):
            pending.append((n, k + 1, s))
        if os_.get(n, 0) and ps.get(n) and bt != "Outport" and n not in replace_src:
            # a block of an offset domain has no output before its first hit (Simulink: 0);
            # recomputing it every base step would already combine constants with the not-yet-sampled inputs
            replace_src[n] = hold(n + "_first", ("RAW", n), enable(ps[n], os_[n]))
    # resolve sources (an original block may now be carried by another name, possibly through chains)

    def resolve(x):
        seen = set()
        while isinstance(x, tuple) or x in replace_src:
            if isinstance(x, tuple):
                if x[0] == "RAW":
                    return x[1]
                x = x[1]
                continue
            if x in seen:
                raise G.GraphError("ALGEBRAIC_LOOP", "design", "lowering loop through %s" % x)
            seen.add(x)
            x = replace_src[x]
        return x
    out_lines = [{"src": resolve(s), "dst": dst, "dport": port} for dst, port, s in pending]
    for ln in lines:
        out_lines.append({"src": resolve(ln["src"]), "dst": ln["dst"], "dport": ln["dport"]})
    for b in blocks:   # everything runs at the base step now
        blocks[b].pop("SampleTime", None)
    carrier = {n: resolve(n) for n in d.names}
    lowered = {"blocks": blocks, "lines": out_lines}
    if port_types:
        lowered["port_types"] = port_types
    return lowered, {"base": base, "period_steps": ps, "offset_steps": os_,
                                                     "internal": internal, "carrier": carrier, "filters": filters}


def _fx(t):
    return "fixdt(%d,%d,%d)" % t


def _delay(tag, x, P, ic, enable, hold, udelay):
    """One sample of delay at period P steps (gated construction when P > 1)."""
    if P == 1:
        return udelay(tag, x, ic)
    en = enable(P, 0)
    h = hold(tag + "_in", x, en)
    u = udelay(tag + "_z", h, ic)
    return hold(tag + "_out", u, en, ic)


def _lower_fir(n, d, ps, x, add, lines, enable, hold, udelay, where):
    p = d.params[n]
    xt = d.otype[d.src[n][0]]
    ct, pt, at, ot = MR.fir_types(p, xt, where, G.fc._asic(d.hw))
    h = MR._coefs(p, where)
    rm = MR.FIR_RM[p["roundingMode"]]
    sat = "on" if p["overflowMode"] == "on" else "off"
    dec = d.btype[n] == "FIRDecimation"
    R = int(p["D"] if dec else p["L"])
    pin, pout = ps[d.src[n][0]], ps[n]
    taps = [x]
    ntap = len(h) if dec else (len(h) + R - 1) // R
    for k in range(1, ntap):
        taps.append(_delay("%s_x%d" % (n, k), taps[-1], pin, "0", enable, hold, udelay))

    def term(j, src, tag):
        g = add("\x1e%s_g%s" % (n, tag), {"type": "Gain", "Gain": repr(h[j]), "ParamDataTypeStr": _fx(ct),
                                            "OutDataTypeStr": _fx(pt), "RndMeth": rm, "SaturateOnIntegerOverflow": sat})
        lines.append({"src": src, "dst": g, "dport": 1})
        a = add("\x1e%s_a%s" % (n, tag), {"type": "DataTypeConversion", "OutDataTypeStr": _fx(at), "RndMeth": rm,
                                            "SaturateOnIntegerOverflow": sat})
        lines.append({"src": g, "dst": a, "dport": 1})
        return a

    def total(terms, tag):
        if len(terms) == 1:
            acc = terms[0]
        else:
            acc = add("\x1e%s_s%s" % (n, tag), {"type": "Sum", "Inputs": "+" * len(terms), "AccumDataTypeStr": _fx(at),
                                                  "OutDataTypeStr": _fx(at), "RndMeth": rm,
                                                  "SaturateOnIntegerOverflow": sat})
            for k, t in enumerate(terms):
                lines.append({"src": t, "dst": acc, "dport": k + 1})
        o = add("\x1e%s_o%s" % (n, tag), {"type": "DataTypeConversion", "OutDataTypeStr": _fx(ot), "RndMeth": rm,
                                            "SaturateOnIntegerOverflow": sat})
        lines.append({"src": acc, "dst": o, "dport": 1})
        return o
    if dec:
        order = [j for ph in range(R - 1, -1, -1) for j in range(ph, len(h), R)]
        y0 = total([term(j, taps[j], "%d" % j) for j in order], "")
        return hold(n + "_y", y0, enable(pout, 0)) if pout > 1 else y0
    phases = []
    for ph in range(R):
        js = list(range(ph, len(h), R))
        if not js:
            continue
        yp = total([term(j, taps[(j - ph) // R], "%d" % j) for j in js], "p%d" % ph)
        gt = add("\x1e%s_gate%d" % (n, ph), {"type": "Gate"})
        lines.append({"src": yp, "dst": gt, "dport": 1})
        lines.append({"src": enable(pin, ph * pout), "dst": gt, "dport": 2})
        phases.append(gt)
    if len(phases) == 1:
        y = phases[0]
    else:
        y = add("\x1e%s_mux" % n, {"type": "Sum", "Inputs": "+" * len(phases), "AccumDataTypeStr": _fx(ot),
                                    "OutDataTypeStr": _fx(ot)})
        for k, t in enumerate(phases):
            lines.append({"src": t, "dst": y, "dport": k + 1})
    return hold(n + "_y", y, enable(pout, 0)) if pout > 1 else y


def _lower_cic(n, d, ps, x, add, lines, enable, hold, udelay, where):
    p = d.params[n]
    xt = d.otype[d.src[n][0]]
    dec = d.btype[n] == "CICDecimation"
    secs, ot = MR.cic_types(p, xt, dec, where)
    R, M, N = int(p["R"]), int(p["M"]), int(p["N"])
    pin, pout = ps[d.src[n][0]], ps[n]

    def cast(tag, src, t):
        c = add("\x1e%s_c%s" % (n, tag), {"type": "DataTypeConversion", "OutDataTypeStr": _fx(t), "RndMeth": "Floor",
                                            "SaturateOnIntegerOverflow": "off"})
        lines.append({"src": src, "dst": c, "dport": 1})
        return c

    def integrator(tag, v, t, P):
        c = cast(tag, v, t)
        s = add("\x1e%s_is%s" % (n, tag), {"type": "Sum", "Inputs": "++", "AccumDataTypeStr": _fx(t),
                                             "OutDataTypeStr": _fx(t), "RndMeth": "Floor",
                                             "SaturateOnIntegerOverflow": "off"})
        z = _delay("%s_iz%s" % (n, tag), s, P, "0", enable, hold, udelay)
        lines.append({"src": z, "dst": s, "dport": 1})
        lines.append({"src": c, "dst": s, "dport": 2})
        return z   # registered: the integrator outputs its state

    def comb(tag, v, t, P):
        c = cast(tag, v, t)
        z = c
        for k in range(M):
            z = _delay("%s_cz%s_%d" % (n, tag, k), z, P, "0", enable, hold, udelay)
        s = add("\x1e%s_cs%s" % (n, tag), {"type": "Sum", "Inputs": "+-", "AccumDataTypeStr": _fx(t),
                                             "OutDataTypeStr": _fx(t), "RndMeth": "Floor",
                                             "SaturateOnIntegerOverflow": "off"})
        lines.append({"src": c, "dst": s, "dport": 1})
        lines.append({"src": z, "dst": s, "dport": 2})
        return s
    v = x
    if dec:
        for k in range(N):
            v = integrator("i%d" % k, v, (1,) + tuple(secs[k]), pin)
        v = hold(n + "_smp", v, enable(pout, 0)) if pout > 1 else v
        for k in range(N):
            v = comb("c%d" % k, v, (1,) + tuple(secs[N + k]), pout)
    else:
        for k in range(N):
            v = comb("c%d" % k, v, (1,) + tuple(secs[k]), pin)
        g = add("\x1e%s_zs" % n, {"type": "Gate"})
        lines.append({"src": v, "dst": g, "dport": 1})
        lines.append({"src": enable(pin, 0), "dst": g, "dport": 2})
        v = g
        for k in range(N):
            v = integrator("j%d" % k, v, (1,) + tuple(secs[N + k]), pout)
    y = cast("out", v, ot)
    return hold(n + "_y", y, enable(pout, 0)) if pout > 1 else y


def expand_inputs(info, inputs, nsteps=None):
    """Samples at each Inport's own rate -> samples at every base step (held, 0 before the first hit)."""
    out = {}
    for n, xs in inputs.items():
        P, o = info["period_steps"].get(n, 1), info["offset_steps"].get(n, 0)
        total = nsteps if nsteps is not None else o + P * len(xs)
        seq = []
        for k in range(total):
            i = (k - o) // P if k >= o else -1
            seq.append(xs[min(i, len(xs) - 1)] if i >= 0 else 0)
        out[n] = seq
    return out

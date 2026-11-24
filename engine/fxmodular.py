"""Modular (wrap-around) regions for fxopt: integrator-comb chains (CIC / Hogenauer) built from basic blocks.

A region is a connected set of Sums whose ports only add or subtract (wrapping: saturation off) and delays (Unit
Delay / Delay, initial condition 0, no reset or enable), containing a feedback loop. When every input into it is a
fixed-point Inport and every signal leaving it has a finite impulse response from every input, two's-complement
arithmetic is exact there: with one type fixdt(1, W, f) for every Sum output and accumulator -- f the finest input
fraction (no rounding inside), W wide enough for the proven range of every signal leaving the region -- each internal
value is congruent to the ideal one modulo 2^(W - f), and the leaving values, which fit W bits, equal it. The
integrators wrap by design.

The range proof: the region is linear and time-invariant on integers (units of 2^-f); the impulse response of each
leaving signal from each input is simulated exactly until it has been zero for n consecutive steps, n the number of
delay states -- then it is zero for ever (Cayley-Hamilton: an output of an n-state autonomous linear system that is
zero for n consecutive steps stays zero). The bound of a leaving signal is the sum over inputs and taps of the tap
times the input's extreme (its input_ranges entry, else its type's full range). Regions that fail any test are left
to the normal optimisation."""
from fractions import Fraction
import re

MAX_STEPS = 200000
SIGNS = re.compile(r"^\|?[+-]+$")


def _int(text):
    """Evaluate an integer parameter with the engine expression parser."""
    import fxnum
    c = fxnum.canonical(text, "int")
    if isinstance(c, fxnum.NumError):
        return None
    try:
        return int(c)
    except (TypeError, ValueError):
        return None


def _candidate(b):
    """(kind, signs or delay length) of a block that can be inside a region, else None."""
    t = b.get("type")
    if t == "Sum":
        sg = str(b.get("Inputs", "++")).strip()
        if not SIGNS.match(sg) or str(b.get("SaturateOnIntegerOverflow", "off")) == "on":
            return None
        sg = sg.replace("|", "")
        return ("sum", sg) if len(sg) >= 2 else None
    if t in ("UnitDelay", "Delay"):
        if _int(b.get("InitialCondition", "0")) != 0:
            return None
        if t == "Delay":
            if any(str(b.get(k, "off")).strip() not in ("off", "None", "") for k in ("ShowEnablePort", "ExternalReset")):
                return None
            if str(b.get("DelayLengthSource", "Dialog")).strip() != "Dialog":
                return None
            n = _int(b.get("DelayLength", "2"))
            return ("delay", n) if n is not None and n >= 1 else None
        return ("delay", 1)
    if t == "Downsample":
        # a CIC from basic blocks decimates between its integrators and its combs; offset 0, sample based
        if str(b.get("SampleOffset", "0")).strip() != "0" or \
                str(b.get("RateOptions", "Allow multirate processing")).strip() != "Allow multirate processing" or \
                str(b.get("InputProcessing", "Elements as channels (sample based)")).strip() != \
                "Elements as channels (sample based)":
            return None
        k = _int(b.get("DownsamplingFactor", "2"))
        return ("down", k) if k is not None and k >= 2 else None
    return None


def _input_type(req, x):
    for src in [req.get("inputs") or {}] + list(req.get("input_sets") or []):
        if isinstance(src, dict) and "tolerance_scale" in src:
            src = src.get("inputs") or {}   # {"inputs": {...}, "tolerance_scale": k} (fxopt)
        v = src.get(x) if isinstance(src, dict) else None
        if isinstance(v, dict):
            t = v.get("type", v)
            if all(k in t for k in ("s", "wl", "fl")):
                return int(t["s"]), int(t["wl"]), int(t["fl"])
    r = (req.get("input_ranges") or {}).get(x)
    if isinstance(r, dict) and isinstance(r.get("type"), dict):
        t = r["type"]
        return int(t["s"]), int(t["wl"]), int(t["fl"])
    return None


def _input_extremes(req, x, t, f):
    """[lo, hi] of input x in units of 2^-f."""
    s, wl, fl = t
    r = (req.get("input_ranges") or {}).get(x)
    if isinstance(r, dict) and "min" in r and "max" in r:
        lo, hi = Fraction(r["min"]), Fraction(r["max"])
    else:
        lo = Fraction(-(1 << (wl - 1)) if s else 0, 1) * Fraction(2) ** -fl
        hi = Fraction(((1 << (wl - 1)) - 1) if s else ((1 << wl) - 1), 1) * Fraction(2) ** -fl
    if _offset(req, x):
        # an Inport with a sample-time offset reads 0 until its first hit (as fxrange and the
        # lowering do): 0 is one of its values inside the region too
        lo, hi = min(lo, Fraction(0)), max(hi, Fraction(0))
    return lo * 2 ** f, hi * 2 ** f


def _offset(req, x):
    """Whether Inport x has a sample-time offset ('[T o]' with o != 0); an unreadable one counts as an offset."""
    st = str(((req.get("design") or {}).get("blocks") or {}).get(x, {}).get("SampleTime", "-1")).strip()
    m = re.match(r"^\[\s*([^\s,\]]+)\s*[ ,]\s*([^\s,\]]+)\s*\]$", st)
    if not m:
        return False
    try:
        return float(m.group(2)) != 0
    except ValueError:
        return True


def regions(req):
    """[{"blocks", "inputs", "exits", "s", "wl", "fl", "bounds", "fir_length"}] of the request's modular regions."""
    d = req.get("design") or {}
    blocks, lines = d.get("blocks") or {}, d.get("lines") or []
    if not isinstance(blocks, dict) or not isinstance(lines, list):
        return []
    fixed = set(req.get("fixed_blocks") or [])
    kind = {n: _candidate(b) for n, b in blocks.items() if isinstance(b, dict) and n not in fixed}
    kind = {n: k for n, k in kind.items() if k}
    src = {}   # dst -> {port: src}
    users = {}
    for ln in lines:
        if not isinstance(ln, dict) or ln.get("src") not in blocks or ln.get("dst") not in blocks:
            return []
        src.setdefault(ln["dst"], {})[int(ln.get("dport", 1))] = ln["src"]
        users.setdefault(ln["src"], set()).add(ln["dst"])
    # connected components of candidate blocks (lines between two candidates)
    comp, out = {}, []
    for n in kind:
        if n in comp:
            continue
        stack, members = [n], set()
        while stack:
            m = stack.pop()
            if m in members:
                continue
            members.add(m)
            for v in list(src.get(m, {}).values()) + list(users.get(m, ())):
                if v in kind and v not in members:
                    stack.append(v)
        for m in members:
            comp[m] = members
        r = _region(req, blocks, kind, src, users, members)
        if r:
            out.append(r)
    return out


def across_rate(req):
    """integrator chains (a candidate component with a feedback loop) that leave through a rate change
    (Downsample, FIR / CIC Decimation) -- a CIC built from basic blocks with its combs after the Downsample. Such a
    chain is not recognised as a modular region (its leaving signal, the integrators' sum, has no finite response
    before the combs), so the optimiser keeps the integrators from wrapping and may answer INFEASIBLE although a
    hand plan with wrapping integrators is exact. Returns the names of those integrator Sums ([] when none)."""
    d = req.get("design") or {}
    blocks, lines = d.get("blocks") or {}, d.get("lines") or []
    if not isinstance(blocks, dict) or not isinstance(lines, list):
        return []
    kind = {n: _candidate(b) for n, b in blocks.items() if isinstance(b, dict)}
    kind = {n: k for n, k in kind.items() if k}
    src, users = {}, {}
    for ln in lines:
        if isinstance(ln, dict) and ln.get("src") in blocks and ln.get("dst") in blocks:
            src.setdefault(ln["dst"], {})[int(ln.get("dport", 1))] = ln["src"]
            users.setdefault(ln["src"], set()).add(ln["dst"])
    seen, out = set(), []
    for n in kind:
        if n in seen:
            continue
        stack, members = [n], set()
        while stack:
            m = stack.pop()
            if m in members:
                continue
            members.add(m)
            for v in list(src.get(m, {}).values()) + list(users.get(m, ())):
                if v in kind and v not in members:
                    stack.append(v)
        seen |= members
        if not _has_cycle(members, {m: src.get(m, {}) for m in members}):
            continue
        if any(blocks.get(u, {}).get("type") in ("Downsample", "FIRDecimation", "CICDecimation")
               for m in members for u in users.get(m, ()) if u not in members):
            out += sorted(m for m in members if kind[m][0] == "sum")
    return out


def _region(req, blocks, kind, src, users, members):
    # a Sum needs every port connected; a delay one input
    for m in members:
        k = kind[m]
        ports = src.get(m, {})
        want = len(k[1]) if k[0] == "sum" else 1
        if sorted(ports) != list(range(1, want + 1)):
            return None
    # a feedback loop inside (else the normal optimisation handles it)
    if not _has_cycle(members, src):
        return None
    ins = sorted({s for m in members for s in src[m].values() if s not in members})
    types = {}
    for x in ins:
        if blocks[x].get("type") != "Inport":
            return None
        t = _input_type(req, x)
        if t is None:
            return None
        types[x] = t
    if not ins:
        return None
    exits = sorted(m for m in members if any(u not in members for u in users.get(m, ())))
    if not exits or any(kind[e][0] != "sum" for e in exits):
        return None
    f = max(t[2] for t in types.values())
    order = _order(members, kind, src)
    if order is None:
        return None
    nstate = sum(kind[m][1] for m in members if kind[m][0] == "delay")
    downs = [m for m in members if kind[m][0] == "down"]
    mr = None
    if downs:
        # integrators -> Downsample -> combs. One rate change; everything after it runs at the slow rate,
        # nothing slow feeds back into the fast part, and every leaving signal is slow
        if len(downs) > 1:
            return None
        ds = downs[0]
        slow, todo = set(), [ds]
        while todo:
            m = todo.pop()
            for u in users.get(m, ()):
                if u in members and u not in slow:
                    slow.add(u)
                    todo.append(u)
        if ds in slow or any(s_ in slow or s_ == ds for m in members if m not in slow and m != ds
                             for s_ in src[m].values()):
            return None
        if any(e not in slow for e in exits):
            return None
        mr = {"down": ds, "slow": slow, "K": kind[ds][1]}
    sims = order
    order = [m for m in sims if kind[m][0] == "sum"]
    # the system is at rest before step 0, so the value at step N sums the taps 0 .. N only; with an
    # input range that excludes 0 a prefix can reach further than the whole response (the start-up transient) --
    # the bound is the envelope over every prefix, all inputs at the same step
    resp, fir_len = {}, 0
    for x in ins:
        h = _impulse(members, kind, src, order, x, exits, nstate) if mr is None else \
            _impulse_mr(members, kind, src, sims, x, exits, nstate, mr)
        if h is None:
            return None
        resp[x] = h
        fir_len = max([fir_len] + [len(h[e]) for e in exits])
    bounds = {}
    for e in exits:
        lo_run = {x: Fraction(0) for x in ins}
        hi_run = {x: Fraction(0) for x in ins}
        best_lo = best_hi = None
        for k in range(max(1, fir_len)):
            for x in ins:
                lo_x, hi_x = _input_extremes(req, x, types[x], f)
                v = resp[x][e][k] if k < len(resp[x][e]) else 0
                a, b = v * lo_x, v * hi_x
                lo_run[x] += min(a, b)
                hi_run[x] += max(a, b)
            lo_n, hi_n = sum(lo_run.values()), sum(hi_run.values())
            best_lo = lo_n if best_lo is None else min(best_lo, lo_n)
            best_hi = hi_n if best_hi is None else max(best_hi, hi_n)
        bounds[e] = [best_lo, best_hi]
    w = 2
    while any(b[0] < -(1 << (w - 1)) or b[1] > (1 << (w - 1)) - 1 for b in bounds.values()):
        w += 1
        if w > 128:
            return None
    # rounding inside the region (pruned fraction lengths): the response of every leaving signal to an error at
    # each Sum's output; None (no pruning allowed there) when it does not end
    gains = {}
    for m in order:
        h = _impulse(members, kind, src, order, None, exits, nstate, inject=m) if mr is None else \
            _impulse_mr(members, kind, src, sims, None, exits, nstate, mr, inject=m)
        gains[m] = None if h is None else {e: sum(abs(v) for v in h[e]) for e in exits}
    return {"blocks": sorted(members), "inputs": ins, "exits": exits, "s": 1, "wl": w, "fl": f,
            "bounds": {e: [float(b[0] / 2 ** f), float(b[1] / 2 ** f)] for e, b in bounds.items()},
            "exact_bounds": {e: (b[0] / 2 ** f, b[1] / 2 ** f) for e, b in bounds.items()},
            "fir_length": fir_len, "gains": gains, "src": {m: dict(src[m]) for m in members},
            "sums": order, "input_types": types}


def _has_cycle(members, src):
    color = {}

    def visit(n):
        color[n] = 1
        for s in src.get(n, {}).values():
            if s in members:
                if color.get(s) == 1 or (s not in color and visit(s)):
                    return True
        color[n] = 2
        return False
    return any(n not in color and visit(n) for n in members)


def _order(members, kind, src):
    """Evaluation order of the Sums (delays read their state)."""
    sums = [m for m in members if kind[m][0] in ("sum", "down")]
    done, order = set(), []
    for _ in range(len(sums) + 1):
        progress = False
        for m in sums:
            if m in done:
                continue
            if all(s not in members or kind[s][0] == "delay" or s in done for s in src[m].values()):
                done.add(m)
                order.append(m)
                progress = True
        if not progress:
            break
    return order if len(order) == len(sums) else None   # an algebraic loop without a delay: not handled


def _impulse(members, kind, src, order, x, exits, nstate, inject=None):
    """{exit: [h0, h1, ...]} integer impulse response from input x (1 unit at step 0) -- or, with inject = a Sum of
    the region, from one unit added to that Sum's output at step 0 (a rounding error there) -- trailing zeros
    dropped; None when it does not end within MAX_STEPS."""
    state = {m: [0] * kind[m][1] for m in members if kind[m][0] == "delay"}
    h = {e: [] for e in exits}
    zeros = 0
    for k in range(MAX_STEPS):
        val = {m: state[m][-1] for m in state}
        if x is not None:
            val[x] = 1 if k == 0 else 0
        for s_ in {s for m in members for s in src[m].values() if s not in members}:
            val.setdefault(s_, 0)
        for m in order:
            sg = kind[m][1]
            val[m] = sum((1 if sg[p - 1] == "+" else -1) * val[s] for p, s in src[m].items())
            if m == inject and k == 0:
                val[m] += 1
        for e in exits:
            h[e].append(val[e])
        for m in state:
            state[m] = [val[src[m][1]]] + state[m][:-1]
        if k > 0 and all(val[e] == 0 for e in exits):
            zeros += 1
            if zeros >= max(1, nstate):
                for e in exits:
                    while h[e] and h[e][-1] == 0:
                        h[e].pop()
                return h
        else:
            zeros = 0
    return None


def _impulse_mr(members, kind, src, sims, x, exits, nstate, mr, inject=None):
    """_impulse for a region with one Downsample (factor K, offset 0): the leaving (slow) signals at the Downsample's
    hits t = 0, K, 2K, ... answer a unit at input x (or added to Sum inject) at fast step p; c[d] is the response at a
    hit d fast steps after the unit, d = t - p -- every d is covered by one phase p in 0 .. K-1 (a unit at a slow Sum
    only at a hit, p = 0). The slow system is autonomous after the unit: zero leaving values for nstate consecutive
    hits mean zero for ever (Cayley-Hamilton on the slow-rate state map). {exit: [c0, c1, ...]} or None."""
    ds, slow, K = mr["down"], mr["slow"], mr["K"]
    c = {e: {} for e in exits}
    ext = {s for m in members for s in src[m].values() if s not in members}
    phases = [0] if inject is not None and inject in slow else range(K)
    for p in phases:
        state = {m: [0] * kind[m][1] for m in members if kind[m][0] == "delay"}
        held = {m: 0 for m in slow | {ds}}
        zeros, ended = 0, False
        for t in range(MAX_STEPS):
            hit = t % K == 0
            val = {m: state[m][-1] for m in state}
            for s_ in ext:
                val[s_] = 0
            if x is not None:
                val[x] = 1 if t == p else 0
            for m in sims:
                if m in slow or m == ds:
                    if not hit:
                        val[m] = held[m]
                        continue
                    if m == ds:
                        val[m] = val[src[m][1]]
                    else:
                        sg = kind[m][1]
                        val[m] = sum((1 if sg[q - 1] == "+" else -1) * val[s_] for q, s_ in src[m].items())
                        if m == inject and t == p:
                            val[m] += 1
                    held[m] = val[m]
                else:
                    sg = kind[m][1]
                    val[m] = sum((1 if sg[q - 1] == "+" else -1) * val[s_] for q, s_ in src[m].items())
                    if m == inject and t == p:
                        val[m] += 1
            if hit and t >= p:
                for e in exits:
                    c[e][t - p] = val[e]
            for m in state:
                if m in slow and not hit:
                    continue   # a slow delay advances at the hits only
                state[m] = [val[src[m][1]]] + state[m][:-1]
            if hit and t > p:
                if all(val[e] == 0 for e in exits):
                    zeros += 1
                    if zeros >= max(1, nstate):
                        ended = True
                        break
                else:
                    zeros = 0
        if not ended:
            return None
    h = {}
    for e in exits:
        n = max(c[e]) + 1 if c[e] else 0
        h[e] = [c[e].get(d, 0) for d in range(n)]
        while h[e] and h[e][-1] == 0:
            h[e].pop()
    return h


def _pinned(req):
    """Blocks whose output type a typed port fixes (an Outport with an explicit type, design.port_types), followed
    back through delays to the block that sets the type (as fxgopt_lib.GraphProblem._pin_typed_ports does): a
    region containing one cannot keep the common integer width."""
    d = req.get("design") or {}
    blocks, lines = d.get("blocks") or {}, d.get("lines") or []
    want = set()
    for n, b in blocks.items():
        if isinstance(b, dict) and b.get("type") == "Outport" and \
                not str(b.get("OutDataTypeStr", "Inherit: auto")).strip().startswith("Inherit"):
            want |= {ln.get("src") for ln in lines if ln.get("dst") == n}
    for e in d.get("port_types") or []:
        if isinstance(e, dict):
            want.add(e.get("block"))
    srcs = {}
    for ln in lines:
        srcs.setdefault(ln.get("dst"), []).append(ln.get("src"))
    out = set()
    for b in want:
        seen = set()
        while isinstance(blocks.get(b), dict) and blocks[b].get("type") in ("UnitDelay", "Delay") and srcs.get(b) \
                and b not in seen:
            seen.add(b)
            out.add(b)
            b = srcs[b][0]
        out.add(b)
    return out


def types_keep_width(r, blocks, ib):
    """Every Sum of the region signed with at least ib integer bits in its output and accumulator types -- the wraps
    then subtract multiples of 2^ib and the leaving values stay congruent (a pinned fixdt(1,8,10)
    among fixdt(1,12,10) broke it)."""
    for m in r["sums"]:
        for k in ("OutDataTypeStr", "AccumDataTypeStr"):
            t = _fixdt((blocks.get(m) or {}).get(k))
            if t is None or not t[0] or t[1] - t[2] < ib:
                return False
    return True


def error_bound(r, blocks):
    """Largest deviation of each leaving signal caused by the rounding inside the region with the Sum types of
    blocks ({name: params}): each input cast into a coarser accumulator, then the accumulator into a coarser output,
    loses less than one LSB, times the leaving signal's response to an error at that Sum. None if a needed response
    does not end."""
    def fl_of(b):
        while True:
            if b in r["input_types"]:
                return r["input_types"][b][2]
            if b in r["sums"]:
                return _fixdt(blocks[b]["OutDataTypeStr"])[2]
            b = r["src"][b][1]   # a delay: its source's type
    e_ = {e: Fraction(0) for e in r["exits"]}
    for m in r["sums"]:
        fa = _fixdt(blocks[m]["AccumDataTypeStr"])[2]
        fo = _fixdt(blocks[m]["OutDataTypeStr"])[2]
        q = Fraction(0)
        for s_ in r["src"][m].values():
            if fl_of(s_) > fa:
                q += Fraction(2) ** -fa
        if fa > fo:
            q += Fraction(2) ** -fo
        if q:
            g = r["gains"].get(m)
            if g is None:
                return None
            for e in e_:
                e_[e] += q * g[e]
    return e_


def _fixdt(t):
    m = re.match(r"^fixdt\((\d+),\s*(\d+),\s*(-?\d+)\)$", str(t).strip())
    return (int(m.group(1)), int(m.group(2)), int(m.group(3))) if m else None


def fits(r, blocks, ib):
    """(ok, error bound) -- every leaving signal, with the rounding inside, stays within ib integer bits."""
    if not types_keep_width(r, blocks, ib):
        return False, None
    eb = error_bound(r, blocks)
    if eb is None:
        return False, None
    for e, (lo, hi) in r["exact_bounds"].items():
        fo = _fixdt(blocks[e]["OutDataTypeStr"])[2]
        top = Fraction(2) ** (ib - 1)
        if lo - eb[e] < -top or hi + eb[e] > top - Fraction(2) ** -fo:
            return False, eb
    return True, eb


def apply(req, extra_bits=None):
    """(request carrying "_modular": [{"blocks", "ib"}] for fxgopt_lib (each region's Sum types keep ib integer
    bits, the search picks their fraction lengths), regions). extra_bits: {region index: bits added to ib}."""
    if req.get("modular", True) is False:
        return req, []
    if req.get("initial_state", "design") == "any":
        # the proof assumes the chain starts at rest (initial conditions 0); any initial state
        # makes the integrators' homogeneous response part of the leaving values -- not covered
        return req, []
    regs = [r for r in regions(req) if not set(r["blocks"]) & _pinned(req)]
    if not regs:
        return req, []
    extra_bits = extra_bits or {}
    kept = []
    for k, r in enumerate(regs):
        r = dict(r, ib=r["wl"] - r["fl"] + extra_bits.get(k, 0))
        kept.append(r)
    out = dict(req, _modular=[jsonable(r) for r in kept])
    return out, kept


def jsonable(r):
    """The region as the optimiser, fxrange and fxprove read it from the request (JSON values only)."""
    return {"blocks": r["blocks"], "ib": r["ib"], "exits": r["exits"], "sums": r["sums"],
            "src": {m: {str(p): s for p, s in d.items()} for m, d in r["src"].items()},
            "gains": r["gains"], "input_types": {x: list(t) for x, t in r["input_types"].items()},
            "exact_bounds": {e: [str(b[0]), str(b[1])] for e, b in r["exact_bounds"].items()}}


def from_json(j):
    return dict(j, src={m: {int(p): s for p, s in d.items()} for m, d in j["src"].items()},
                input_types={x: tuple(t) for x, t in j["input_types"].items()},
                exact_bounds={e: (Fraction(b[0]), Fraction(b[1])) for e, b in j["exact_bounds"].items()})


def exit_ranges(regions_json, blocks):
    """{exit block: (lo, hi)} real ranges of the values leaving each region under the types of blocks: the proven
    range widened by the rounding error bound (None for a region whose bound does not exist)."""
    out = {}
    for j in regions_json or []:
        r = from_json(j)
        if not types_keep_width(r, blocks, j["ib"]):
            return None
        eb = error_bound(r, blocks)
        if eb is None:
            return None
        for e, (lo, hi) in r["exact_bounds"].items():
            out[e] = (lo - eb[e], hi + eb[e])
    return out

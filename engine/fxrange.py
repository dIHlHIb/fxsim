"""estimated worst-case value ranges of the ideal signals (a SEARCH HEURISTIC that sizes the integer bits; it
is not a guarantee -- the guarantee is engine/fxprove.py on the final candidate), and the stress input sets
used to check the error bound in range mode.

Bounds are on the ideal (double) signals:
  * blocks outside feedback loops: interval arithmetic per block type;
  * a feedback loop made only of linear blocks (Gain, Sum, delays, Discrete FIR, real-world conversion, Shift
    Arithmetic): superposition of impulse responses -- for an input in [lo, hi], max y = sum_k (h[k] > 0 ? h[k]*hi
: h[k]*lo) (min likewise); the response must decay, else the loop is unstable or marginal -> UNSUPPORTED;
  * any other feedback loop: interval fixed-point iteration; no convergence -> UNSUPPORTED.
Blocks that act on bit patterns (Bitwise Operator, Extract Bits) and Stored Integer conversions have no value
bound and are refused in range mode. See docs/en/OPTIMIZER.md (range mode) for what is and is not guaranteed."""
import math
import random

import fxgraph_lib as G
import fxcore as fc
from fxopt_lib import OptError

LINEAR = ("Gain", "Sum", "UnitDelay", "Delay", "DiscreteFir", "DataTypeConversion", "ShiftArithmetic", "Outport")
MAX_IMPULSE = 1 << 18
INF = math.inf


def is_linear(d, n):
    if d.btype[n] == "DiscreteIntegrator":
        # y[k+1] = y[k] + K u[k]: linear without output limits (loops closed through it, PI / leaky loops)
        # Backward Euler / Trapezoidal are not modelled here: interval rule only
        return d.params[n].get("LimitOutput") != "on" and G.fc.dti_method(d.params[n]) == "FE"
    if d.btype[n] == "DataTypeConversion":
        return not str(d.params[n].get("ConvertRealWorld", "")).startswith("Stored Integer")
    return d.btype[n] in LINEAR


def _hull(*iv):
    iv = [x for x in iv if x is not None]
    return (min(a for a, _ in iv), max(b for _, b in iv)) if iv else None


def _scale(iv, k):
    a, b = iv[0] * k, iv[1] * k
    return (min(a, b), max(a, b))


def _mul(x, y):
    p = [x[0] * y[0], x[0] * y[1], x[1] * y[0], x[1] * y[1]]
    return (min(p), max(p))


def _div(x, y, where):
    if y[0] <= 0 <= y[1]:
        raise OptError("UNSUPPORTED", where, "range mode: a divisor's range contains 0, the quotient is unbounded")
    return _mul(x, (1.0 / y[1], 1.0 / y[0]) if y[0] > 0 else (1.0 / y[1], 1.0 / y[0]))


def _block_interval(d, n, ins, where):
    """Interval of block n's output from its input intervals (non-delay, non-loop evaluation)."""
    bt, p = d.btype[n], d.params[n]
    if bt == "Constant":
        v = G._const_double(p)
        return (v, v)
    if bt in ("Outport", "DataTypeConversion"):
        if bt == "DataTypeConversion" and str(p.get("ConvertRealWorld", "")).startswith("Stored Integer"):
            raise OptError("UNSUPPORTED", where, "range mode: Stored Integer conversion has no value bound")
        return ins[0]
    if bt == "Gain":
        return _scale(ins[0], float(p["Gain"]))
    if bt == "Sum":
        signs = G._sum_signs(p, d.nin[n])
        lo = hi = 0.0
        for iv, sg in zip(ins, signs):
            if sg == "+":
                lo, hi = lo + iv[0], hi + iv[1]
            else:
                lo, hi = lo - iv[1], hi - iv[0]
        return (lo, hi)
    if bt == "Product":
        ops = str(p["Inputs"]).strip()
        ops = "*" * int(ops) if ops.isdigit() else ops
        acc = (1.0, 1.0)
        for iv, o in zip(ins, ops):
            acc = _mul(acc, iv) if o == "*" else _div(acc, iv, where)
        return acc
    if bt == "Saturation":
        lo, hi = float(p["LowerLimit"]), float(p["UpperLimit"])
        return (min(max(ins[0][0], lo), hi), max(min(ins[0][1], hi), lo))
    if bt == "Abs":
        a, b = ins[0]
        return (0.0 if a <= 0 <= b else min(abs(a), abs(b)), max(abs(a), abs(b)))
    if bt == "UnaryMinus":
        return (-ins[0][1], -ins[0][0])
    if bt == "MinMax":
        f = min if p["Function"] == "min" else max
        return (f(iv[0] for iv in ins), f(iv[1] for iv in ins))
    if bt == "Switch":
        return _hull(ins[0], ins[2])
    if bt == "Hold":   # multirate lowering: a held copy of its input (or its initial value)
        ic = float(p["InitialCondition"])
        return _hull(ins[0], (ic, ic))
    if bt == "Gate":
        return _hull(ins[0], (0.0, 0.0))
    if bt == "MultiportSwitch":
        return _hull(*ins[1:])
    if bt == "ShiftArithmetic":
        return _scale(ins[0], G._shift_factor(p))
    if bt in ("RelationalOperator", "LogicalOperator", "CompareToConstant", "DetectChange", "DetectIncrease",
              "DetectDecrease", "CompareToZero"):
        return (0.0, 1.0)
    if bt in ("CounterLimited", "CounterFreeRunning"):
        return (0.0, float(G._counter_limit(d, n)))
    if bt == "Signum":
        return ((-1.0 if ins[0][0] < 0 else 0.0), (1.0 if ins[0][1] > 0 else 0.0))
    if bt == "Rounding":
        return (math.floor(ins[0][0]), math.ceil(ins[0][1]))
    if bt == "Lookup1D":
        table = G._lut_raw(p)[1]
        return (min(table), max(table))
    if bt == "DiscreteFir" and p.get("FilterStructure") == "Direct form transposed":
        # y[n] = sum_{k<=n} c_k x[n-k] + z_(n+1)(0): the initial states are partial sums added to the output
        init = G._fir_init_double(p, len(fc._dfir_coefs(p)))
        terms = [_scale(ins[0], c) for c in fc._dfir_coefs(p)]
        out, lo, hi = None, 0.0, 0.0
        for k, (a, b) in enumerate(terms):
            lo, hi = lo + a, hi + b
            z = init[k] if k < len(init) else 0.0
            out = _hull(out, (lo + z, hi + z)) if out else (lo + z, hi + z)
        return out
    if bt == "DiscreteFir":
        x = _hull(ins[0], *[(v, v) for v in G._fir_init_double(p, len(fc._dfir_coefs(p)))])
        lo = hi = 0.0
        for c in fc._dfir_coefs(p):
            a, b = _scale(x, c)
            lo, hi = lo + a, hi + b
        return (lo, hi)
    if bt == "DiscreteIntegrator":
        if p.get("LimitOutput") == "on":
            lo, hi = float(p["LowerSaturationLimit"]), float(p["UpperSaturationLimit"])
            ic = float(p["InitialCondition"])
            return (min(lo, max(ic, lo)), max(hi, min(ic, hi)))
        raise OptError("UNSUPPORTED", where, "range mode: an integrator without output limits is unbounded")
    if bt in ("BitwiseOperator", "ExtractBits"):
        raise OptError("UNSUPPORTED", where, "range mode: %s acts on bit patterns, no value bound" % bt)
    raise OptError("UNSUPPORTED", where, "range mode: no range rule for %s" % bt)


def _sccs(d):
    """Strongly connected components (all edges, delays included), in dependency order."""
    index, low, on, stack, out = {}, {}, set(), [], []
    users = {n: [] for n in d.names}
    for n in d.names:
        for s in d.src[n]:
            users[s].append(n)
    counter = [0]

    def visit(v):  # iterative Tarjan
        work = [(v, iter(users[v]))]
        index[v] = low[v] = counter[0]
        counter[0] += 1
        stack.append(v)
        on.add(v)
        while work:
            node, it = work[-1]
            nxt = next(it, None)
            if nxt is None:
                work.pop()
                if work:
                    low[work[-1][0]] = min(low[work[-1][0]], low[node])
                if low[node] == index[node]:
                    comp = []
                    while True:
                        w = stack.pop()
                        on.discard(w)
                        comp.append(w)
                        if w == node:
                            break
                    out.append(comp)
                continue
            if nxt not in index:
                index[nxt] = low[nxt] = counter[0]
                counter[0] += 1
                stack.append(nxt)
                on.add(nxt)
                work.append((nxt, iter(users[nxt])))
            elif nxt in on:
                low[node] = min(low[node], index[nxt])
    for n in d.names:
        if n not in index:
            visit(n)
    return out[::-1]  # Tarjan emits sinks first


def _impulse_bounds(d, comp, ext, where):
    """Linear loop: bound of every member for inputs ext[source] = (lo, hi), plus the zero-input response of
    the initial conditions. Returns {member: (lo, hi)}."""
    members = set(comp)
    sources = sorted(ext)
    blocks, lines = {}, []
    for s in sources:
        blocks["__in_" + s] = {"type": "Inport"}
    for n in comp:
        blocks[n] = dict(d.params[n], type=d.btype[n])
        for k, s in enumerate(d.src[n]):
            lines.append({"src": s if s in members else "__in_" + s, "dst": n, "dport": k + 1})
    sub0 = G.Design({"blocks": blocks, "lines": lines})  # with the initial conditions (zero-input response)
    for n in comp:  # impulse responses need a system at rest
        if d.btype[n] in G.DELAYS or d.btype[n] == "DiscreteIntegrator":
            blocks[n]["InitialCondition"] = "0"
        elif d.btype[n] == "DiscreteFir":
            blocks[n]["InitialStates"] = "0"
    sub = G.Design({"blocks": blocks, "lines": lines})
    lo = {n: 0.0 for n in comp}
    hi = {n: 0.0 for n in comp}
    # simulate well past the longest pure-delay path, so delayed responses are not cut off; the
    # truncation and the tail term remain heuristics
    span = sum(int(d.params[n].get("DelayLength", 1)) if d.btype[n] in G.DELAYS else
               (len(fc._dfir_coefs(d.params[n])) if d.btype[n] == "DiscreteFir" else 0) for n in comp)
    L = max(4096, 1 << (4 * span + 4).bit_length())
    def settled(h):
        peak = max(abs(v) for v in h) or 1.0
        return all(math.isfinite(v) for v in h) and max(abs(v) for v in h[3 * len(h) // 4:]) <= 1e-12 * peak

    while True:
        resp = {}
        for s in sources:
            zero = {("__in_" + t): [0.0] * L for t in sources}
            zero["__in_" + s] = [1.0] + [0.0] * (L - 1)
            resp[s] = sub.run_double(zero, L, comp)
        zi = sub0.run_double({("__in_" + t): [0.0] * L for t in sources}, L, comp)
        ok = all(settled(resp[s][n]) for s in sources for n in comp) and all(settled(zi[n]) for n in comp)
        if ok or L >= MAX_IMPULSE:
            break
        L *= 4
    if not ok:
        # name the blocks that lie on a cycle (the component also holds blocks fed by the loop, so the
        # first four names in alphabetical order could all be off the loop) and the responses that do not settle
        def on_cycle(n):
            seen_, todo_ = set(), [y for y in d.src[n] if y in comp]
            while todo_:
                y = todo_.pop()
                if y == n:
                    return True
                if y not in seen_:
                    seen_.add(y)
                    todo_ += [z for z in d.src[y] if z in comp]
            return False
        cyc = sorted(n for n in comp if on_cycle(n)) or sorted(comp)
        bad_ = sorted({n for s in sources for n in comp if not settled(resp[s][n])} |
                      {n for n in comp if not settled(zi[n])})
        raise OptError("UNSUPPORTED", where, "range mode: the feedback loop through %s does not settle (unstable "
                       "or marginally stable), so no input range bounds it. An integrator that relies on wrap-around "
                       "(a CIC / Hogenauer filter, a phase accumulator) cannot be range-proven: give the request "
                       "\"ranges_from_inputs\": false and no input_ranges -- the plan is then checked on the given "
                       "input sets only (guarantees sampled); signals that do not settle: %s"
                       % (", ".join(cyc[:16]), ", ".join(bad_[:16])))
    # y[N] = zi[N] + sum_s sum_{k<=N} h_s[k] x_s[N-k]; the system is at rest before step 0, so the bound is the
    # extreme over N of the prefix sums (0 need not lie inside an input range)
    for n in comp:
        flo, fhi, tail = [0.0] * L, [0.0] * L, 0.0
        for s in sources:
            a, b = ext[s]
            h = resp[s][n]
            clo = chi = 0.0
            for k, v in enumerate(h):
                if v > 0:
                    clo, chi = clo + v * a, chi + v * b
                elif v < 0:
                    clo, chi = clo + v * b, chi + v * a
                flo[k] += clo
                fhi[k] += chi
            # the response beyond the L simulated samples: below 1e-12 of its peak and decaying; charge it
            # generously as L more samples at the largest late value
            tail += L * max(abs(v) for v in h[3 * L // 4:]) * max(abs(a), abs(b))
        z = zi[n]
        lo[n] = min(flo[k] + z[k] for k in range(L)) - tail
        hi[n] = max(fhi[k] + z[k] for k in range(L)) + tail
    return {n: (lo[n], hi[n]) for n in comp}, {s: resp[s] for s in sources}


def bounds(d, in_ranges, modular=None):
    """{slot: (lo, hi)} for every block (and the Discrete FIR product slots), and a method summary.
    modular: the request's modular regions (engine/fxmodular.py): their signals leaving are bounded by the region's
    proof (impulse responses from the input ranges), the wrapping signals inside by their shared integer width; the
    rest of the design is bounded as usual from those."""
    val = {n: in_ranges[n] for n in d.inports}
    methods = set()
    inner = [n for n in d.names if d.btype[n] != "Inport"]
    mod_blocks = set()
    if modular:
        from fractions import Fraction
        for r in modular:
            top = float(2 ** (r["ib"] - 1))
            for m in r["blocks"]:
                val[m] = (-top, top)
            for e, (lo, hi) in r["exact_bounds"].items():
                val[e] = (float(Fraction(lo)), float(Fraction(hi)))
            mod_blocks |= set(r["blocks"])
        methods.add("modular regions (proven leaving ranges)")
        inner = [n for n in inner if n not in mod_blocks]
    if all(is_linear(d, n) for n in inner) and not mod_blocks:
        # wholly linear design: superposition is exact per signal (interval arithmetic would count an input
        # reaching a signal along several paths once per path)
        got, _ = _impulse_bounds(d, inner, dict(in_ranges), "design")
        val.update(got)
        methods.add("impulse-response bound of linear systems")
        comps = []
    else:
        comps = _sccs(d)
    for comp in comps:
        if all(n in mod_blocks for n in comp):
            continue
        cyclic = len(comp) > 1 or comp[0] in d.src[comp[0]]
        if not cyclic:
            n = comp[0]
            if d.btype[n] == "Inport":
                continue
            ins = [val[s] for s in d.src[n]]
            if d.btype[n] in G.DELAYS:
                ic = float(d.params[n]["InitialCondition"])
                val[n] = _hull(ins[0], (ic, ic))
            else:
                val[n] = _block_interval(d, n, ins, "design.blocks." + n)
            methods.add("interval arithmetic")
            continue
        where = "design.blocks." + sorted(comp)[0]
        ext = {s: val[s] for n in comp for s in d.src[n] if s not in comp}
        if all(is_linear(d, n) for n in comp):
            got, _ = _impulse_bounds(d, comp, ext, where)
            val.update(got)
            methods.add("impulse-response bound of linear feedback loops")
            continue
        # non-linear loop: interval fixed point inside the component
        cur = {n: None for n in comp}
        for n in comp:
            if d.btype[n] in G.DELAYS:
                ic = float(d.params[n]["InitialCondition"])
                cur[n] = (ic, ic)
            elif d.btype[n] == "DiscreteIntegrator" and d.params[n].get("LimitOutput") == "on":
                cur[n] = _block_interval(d, n, [None], "design.blocks." + n)   # the limits hold it
        for _ in range(2000):
            changed = False
            for n in comp:
                ins = [val[s] if s not in comp else cur[s] for s in d.src[n]]
                if any(iv is None for iv in ins):
                    continue
                if d.btype[n] in G.DELAYS:
                    new = _hull(cur[n], ins[0])
                else:
                    new = _block_interval(d, n, ins, "design.blocks." + n)
                    new = _hull(cur[n], new)
                if new != cur[n]:
                    if max(abs(new[0]), abs(new[1])) > 1e30:
                        raise OptError("UNSUPPORTED", where, "range mode: the bound of the feedback loop through %s "
                                       "diverges" % ", ".join(sorted(comp)[:4]))
                    cur[n] = new
                    changed = True
            if not changed:
                break
        else:
            raise OptError("UNSUPPORTED", where, "range mode: no finite bound proven for the non-linear feedback loop "
                           "through %s" % ", ".join(sorted(comp)[:4]))
        if any(v is None for v in cur.values()):
            raise OptError("UNSUPPORTED", where, "range mode: loop without a defined start value")
        val.update(cur)
        methods.add("interval fixed point of non-linear feedback loops")
    # hidden slots the optimiser sizes: Discrete FIR products
    for n in d.names:
        if d.btype[n] == "DiscreteFir":
            p = d.params[n]
            x = val[d.src[n][0]]
            if p.get("FilterStructure") != "Direct form transposed":   # transposed products see the input only
                x = _hull(x, *[(v, v) for v in G._fir_init_double(p, len(fc._dfir_coefs(p)))])
            coefs = fc._dfir_coefs(p)
            fs = p.get("FilterStructure")
            if fs in ("Direct form symmetric", "Direct form antisymmetric"):
                # products of tap sums x_k +- x_(N-1-k) (sized as input x coefficient they had one integer
                # bit too few and wrapped). The design is not compiled here, so the tap-sum type is not known: the
                # bound assumes the tap sum fits it; a tap sum that wraps is an overflow of its own, caught by the
                # bit-exact evaluation and by the overflow prover on the compiled design
                ts = (2 * x[0], 2 * x[1]) if fs == "Direct form symmetric" else (x[0] - x[1], x[1] - x[0])
                for k in range(len(coefs) // 2):
                    val[G._hid(n, "p%d" % k)] = _scale(ts, coefs[k])
                if len(coefs) % 2:
                    val[G._hid(n, "p%d" % (len(coefs) // 2))] = _scale(x, coefs[len(coefs) // 2])
            else:
                for k, c in enumerate(coefs):
                    val[G._hid(n, "p%d" % k)] = _scale(x, c)
    return val, ", ".join(sorted(methods)) or "inputs only"


def worst_case_sets(d, in_ranges, length):
    """Linear designs: for every judged output and both extremes, the input sequence that reaches the bound
    (each input at hi or lo following the sign of its impulse response). Returns [(label, {inport: values})]."""
    if not all(d.btype[n] == "Inport" or is_linear(d, n) for n in d.names):
        return []
    try:
        _, resp = _impulse_bounds(d, [n for n in d.names if d.btype[n] != "Inport"],
                                  {n: in_ranges[n] for n in d.inports}, "design")
    except OptError:
        return []
    out = []
    for o in d.outports:
        for sign, tag in ((1, "max"), (-1, "min")):
            seqs = {}
            for x in d.inports:
                h = resp[x][o][:length]
                lo, hi = in_ranges[x]
                # y[N] = sum_k h[k] x[N-k]: x[N-k] = hi where sign*h[k] > 0
                pat = [hi if sign * h[k] > 0 else lo for k in range(len(h))][::-1]
                seqs[x] = (pat + [pat[-1]] * length)[:length]
            out.append(("worst_case_%s_%s" % (o, tag), seqs))
    return out


def add_range_scenarios(pb, req):
    """Parse input_ranges, prove the bounds and add the stress input sets (range mode)."""
    spec = pb.range_spec
    d = pb.d0
    if not isinstance(spec, dict) or set(spec) != set(d.inports):
        raise OptError("BAD_PARAM", "input_ranges", "object with one entry per Inport: {\"type\": {s, wl, fl}, "
                       "\"min\": lo, \"max\": hi}")
    rt = req.get("range_test", {})
    if not isinstance(rt, dict) or set(rt) - {"length", "seed"}:
        raise OptError("BAD_PARAM", "range_test", "{\"length\": n, \"seed\": k}")
    import fxopt_lib as L
    length = L.strict_number(rt, "length", "range_test", integer=True, default=4096)
    seed = L.strict_number(rt, "seed", "range_test", integer=True, default=0)
    if not (64 <= length <= 1_000_000):
        raise OptError("BAD_PARAM", "range_test.length", "64..1000000 samples")
    types, ints_rng, real_rng = {}, {}, {}
    for x in d.inports:
        e = spec[x]
        where = "input_ranges." + x
        if not isinstance(e, dict) or set(e) - {"type", "min", "max"}:
            raise OptError("BAD_PARAM", where, "{\"type\": {s, wl, fl}, \"min\": lo, \"max\": hi}")
        t = L.strict_type(e.get("type"), where + ".type")
        lo, hi = L.strict_number(e, "min", where), L.strict_number(e, "max", where)
        if not lo <= hi:
            raise OptError("BAD_PARAM", where, "min must not exceed max")
        s, w, f = t
        ilo, ihi = L.stored_range(lo, hi, f)
        tlo, thi = (-(1 << (w - 1)), (1 << (w - 1)) - 1) if s else (0, (1 << w) - 1)
        if ilo > ihi or ilo < tlo or ihi > thi:
            raise OptError("BAD_PARAM", where, "range [%g, %g] does not fit the input type fixdt(%d,%d,%d)" % (lo, hi, s, w, f))
        types[x], ints_rng[x], real_rng[x] = t, (ilo, ihi), (ilo * 2.0 ** -f, ihi * 2.0 ** -f)
        # an Inport with a sample-time offset reads 0 until its first hit (the lowering
        # feeds 0, as fxgraph does), so the proven range of that signal includes 0 -- the proof ran on the caller's
        # range alone and called a design bounded that wraps at t = 0
        if (getattr(pb, "mr_info", None) or {}).get("offset_steps", {}).get(x, 0) > 0 and not ilo <= 0 <= ihi:
            ilo, ihi = min(ilo, 0), max(ihi, 0)
            ints_rng[x], real_rng[x] = (ilo, ihi), (ilo * 2.0 ** -f, ihi * 2.0 ** -f)
    if pb.in_types is None:
        pb.in_types = types
    elif pb.in_types != types:
        raise OptError("BAD_PARAM", "input_ranges", "Inport types must match those of the input sets")
    init = req.get("initial_state", "design")
    if init not in ("design", "any"):
        raise OptError("BAD_PARAM", "initial_state", '"design" (the initial conditions of the design, reset before '
                       'every sequence) or "any" (every delay state anywhere in its type range)')
    pb.range_ints, pb.initial_state = ints_rng, init
    pb.bounds, pb.bound_method = bounds(d, real_rng, req.get("_modular"))
    pb.range_summary = {x: {"min": real_rng[x][0], "max": real_rng[x][1],
                            "type": "fixdt(%d,%d,%d)" % types[x]} for x in d.inports}
    rng = random.Random(seed)
    names = list(d.inports)

    def const(v):
        return [v] * length
    sets = [("range: all inputs at max", {x: const(ints_rng[x][1]) for x in names}),
            ("range: all inputs at min", {x: const(ints_rng[x][0]) for x in names}),
            ("range: step min to max", {x: [ints_rng[x][0]] * (length // 2) + [ints_rng[x][1]] * (length - length // 2)
                                        for x in names}),
            ("range: step max to min", {x: [ints_rng[x][1]] * (length // 2) + [ints_rng[x][0]] * (length - length // 2)
                                        for x in names}),
            ("range: alternating min/max", {x: [ints_rng[x][k % 2] for k in range(length)] for x in names}),
            ("range: square wave, period 64", {x: [ints_rng[x][(k // 32) % 2] for k in range(length)] for x in names}),
            ("range: random extremes", {x: [ints_rng[x][rng.getrandbits(1)] for _ in range(length)] for x in names}),
            ("range: random uniform", {x: [rng.randint(*ints_rng[x]) for _ in range(length)] for x in names})]
    if len(names) > 1:
        sets.append(("range: first input max, others min", {x: const(ints_rng[x][1] if i == 0 else ints_rng[x][0])
                                                           for i, x in enumerate(names)}))
    for label, seqs in worst_case_sets(d, real_rng, length):
        sets.append(("range: " + label, {x: [max(ints_rng[x][0], min(ints_rng[x][1], round(v * 2.0 ** types[x][2])))
                                              for v in seqs[x]] for x in names}))
    info = getattr(pb, "mr_info", None)
    for label, ints in sets:
        if info is not None:
            # multirate: a stress sequence is a sequence of Inport samples, held over the Inport's period
            import fxlower
            short = {x: v[:max(1, length // info["period_steps"].get(x, 1))] for x, v in ints.items()}
            ints = {x: v[:length] for x, v in fxlower.expand_inputs(info, short, length).items()}
        pb.scenarios.append({"label": label, "ints": ints, "nsteps": length,
                             "scale": getattr(pb, "stress_scale", 1.0)})   # stress_tolerance_scale

"""Search for input sequences that exceed an output tolerance inside the given ranges.

Evaluate windows separated by quiet gaps and score each by its largest relative
error. A coordinate search varies one sample at a time. Re-simulate a violating
window on its own before returning it. The search is heuristic.
"""
import math
import os
import subprocess
import random
import time

import fxgopt_lib as GL
import fxgraph_lib as G


DC_STEPS = 4096   # length of the constant-input probes (drift reached 1.46 x tolerance at 3000 steps)


def integrating(spec):
    """Whether the design can accumulate a small input over time: a feedback loop, or a Discrete-Time Integrator."""
    blocks, lines = spec["blocks"], spec["lines"]
    if any(b.get("type") == "DiscreteIntegrator" for b in blocks.values()):
        return True
    succ = {}
    for ln in lines:
        succ.setdefault(ln["src"], []).append(ln["dst"])
    state, stack = {}, []
    for root in blocks:
        if state.get(root):
            continue
        stack.append((root, iter(succ.get(root, ()))))
        state[root] = 1
        while stack:
            n, it = stack[-1]
            nxt = next(it, None)
            if nxt is None:
                state[n] = 2
                stack.pop()
            elif state.get(nxt) == 1:
                return True
            elif not state.get(nxt):
                state[nxt] = 1
                stack.append((nxt, iter(succ.get(nxt, ()))))
    return False


def memory_estimate(spec):
    """Samples after which an input stops mattering, for FIR / delay structures; loops get a fixed extra."""
    m, loops = 1, False
    for b in spec["blocks"].values():
        t = b.get("type")
        if t == "DiscreteFir":
            m += len(str(b.get("Coefficients", "")).strip("[]").replace(",", " ").split())
        elif t in ("UnitDelay", "Memory"):
            m += 1
        elif t == "Delay":
            try:
                m += int(float(b.get("DelayLength", 1)))
            except ValueError:
                m += 1
        elif t in ("Downsample", "Upsample"):
            try:
                m *= max(1, int(float(b.get("N", b.get("K", 1)))))
            except ValueError:
                pass
        elif t in ("DiscreteTransferFcn", "DiscreteIntegrator", "DiscreteFilter"):
            loops = True
    return min(256, m + (32 if loops else 0))


def _problem(req, sol_blocks, ins):
    q = {k: v for k, v in req.items() if k in ("hw", "error", "wordlengths", "reference")}
    blocks = {b: dict(v, **sol_blocks.get(b, {})) for b, v in req["design"]["blocks"].items()}
    q["design"] = {"blocks": blocks, "lines": req["design"]["lines"]}
    q["inputs"] = ins
    return GL.GraphProblem(q)


class Evaluator:
    """The solution compiled once; errors() swaps the inputs only."""

    def __init__(self, req, sol_blocks, in_types):
        self.in_types = in_types
        dummy = {x: {"type": {"s": t[0], "wl": t[1], "fl": t[2]}, "ints": [0] * 8} for x, t in in_types.items()}
        self.pb = pb = _problem(req, sol_blocks, dummy)
        self.d = G.Design(pb.spec, pb.hw)
        self.d.compile(pb.in_types)
        self.judged = list(pb.tols)

    def errors(self, seqs):
        """Per input sample, the largest |fixed - ideal| / tolerance over the judged outports."""
        pb, d = self.pb, self.d
        ints = dict(seqs)
        if pb.mr_info is not None:
            import fxlower
            ints = fxlower.expand_inputs(pb.mr_info, ints)
        n = min(len(v) for v in ints.values())
        ints = {k: v[:n] for k, v in ints.items()}
        if pb.mr_info is None and d.vm_ok and not os.environ.get("FXERRSEARCH_NO_CP"):
            out = self._errors_core(ints, n)
            if out is not None:
                return out
        logs = G.run_fast(d, ints, n, self.judged)[0] if d.vm_ok else d.run(ints, n, self.judged)
        # ideal reference: the C core's double program (bit-identical to run_double, the regression check;
        # ~69 % of the search time was the Python double simulation); Python when the core declines
        ref = G.run_double_core(pb.d0, ints, {x: self.in_types[x][2] for x in pb.d0.inports}, n, self.judged)
        if ref is None:
            ref = pb.d0.run_double({x: [math.ldexp(v, -self.in_types[x][2]) for v in ints[x]] for x in pb.d0.inports},
                                   n, self.judged)
        n_in = len(next(iter(seqs.values())))
        out = [0.0] * n_in
        for o, tl in pb.tols.items():
            fl = d.otype[o][2]
            for t, (a, b) in enumerate(zip(logs[o], ref[o])):
                k = min(n_in - 1, t * n_in // max(1, n))   # base step -> input sample (multirate)
                # a non-finite ideal value (x / x at x = 0) was skipped as NaN -- and the search then
                # promised "exhaustive"; it is an infinite error, so the input becomes a counterexample and the
                # re-optimisation meets REFERENCE_NOT_FINITE on it
                if not math.isfinite(b):
                    e = math.inf
                elif -(1 << 53) < a < (1 << 53):
                    e = abs(math.ldexp(a, -fl) - b) / tl
                else:   # a wider output compared exactly, not as a rounded double
                    from fractions import Fraction
                    e = float(abs(Fraction(a) / (Fraction(2) ** fl) - Fraction(b))) / tl
                if e > out[k]:
                    out[k] = e
        return out

    def _errors_core(self, ints, n):
        """The same per-step measure from one `fxgraph_core -s` session (IN + DREF + CP): the fixed program and the
        double reference run side by side in C (two subprocesses and the Python loop per score were most of
        the search time). None when the core cannot take it (the caller then uses the two-run path above)."""
        pb, d = self.pb, self.d
        if not os.path.exists(G.CORE) or n < 1 or any(len(v) != n for v in ints.values()):
            return None
        try:
            slot, ins, dly = G.vm_instructions(d)
            dslot, dins, ddly = G.vm_double_instructions(pb.d0)
        except G.GraphError:
            return None
        judged = list(pb.tols)
        if any(x not in slot or dslot.get(x) != slot[x] for x in list(pb.d0.inports) + judged):
            return None
        head = ["IN %d %d %d %s" % (dslot[x], self.in_types[x][2], n, " ".join(str(int(v)) for v in ints[x]))
                for x in pb.d0.inports]
        head.append("DREF %d %d %s %d %s %d %s" % (len(pb.d0.slots), len(dins), " ".join(dins), len(ddly),
                                                   " ".join(ddly), len(judged), " ".join(str(dslot[o]) for o in judged)))
        wide, k = [], 0
        for b in d.order:
            if b in d.vm:
                c = len(G._flat(d.vm[b]))
                if b in d.vm_wide:
                    wide.extend(range(k, k + c))
                k += c
        fast = bool(d.vm_fast) and not os.environ.get("FXGRAPH_NO_FAST")
        head.append("CP %d %d %d %d %s %s %d %s %d %s %s" % (
            len(slot), len(ins), 1 if fast else 0, len(wide) if fast else 0, " ".join(map(str, wide)) if fast else "",
            " ".join(ins), len(dly), " ".join(dly), len(judged),
            " ".join("%d %d" % (slot[o], d.otype[o][2]) for o in judged), " ".join(repr(float(pb.tols[o])) for o in judged)))
        head.append("END")
        try:
            p = subprocess.run([G.CORE, "-s"], input=("\n".join(head) + "\n").encode(), capture_output=True, timeout=600)
        except (OSError, subprocess.TimeoutExpired):
            return None
        per = None
        for line in p.stdout.decode(errors="replace").splitlines():
            parts = line.split()
            if not parts:
                continue
            if parts[0] == "E":
                if len(parts) >= 4 and parts[1] == "UNRECORDED":
                    name = next((k for k, i in slot.items() if str(i) == parts[2]), "?")
                    raise G.GraphError("UNSUPPORTED", "design.blocks." + name, "at step %s: %s"
                                       % (parts[3], G.fc.LUT_ROUND1_MSG % "for this block"))
                if len(parts) >= 4 and parts[1] == "SIM_DIAGNOSTIC":
                    name = next((k for k, i in slot.items() if str(i) == parts[2]), "?")
                    raise G.GraphError("SIM_DIAGNOSTIC", "design.blocks." + name, "at step %s: Multiport Switch control "
                                       "is out of range of its data ports: Simulink stops with an error "
                                       "(DiagnosticForDefault Error, or a single data port)" % parts[3])
                return None
            if parts[0] == "P":
                per = [float(v) for v in parts[2:]]
        if p.returncode != 0 or per is None or len(per) != n:
            return None
        return per


CAP_S = 1800.0   # safety stop only; the search itself is a fixed amount of work (deterministic)


STATEFUL = ("UnitDelay", "Delay", "DiscreteFir", "DiscreteIntegrator", "CounterLimited", "CounterFreeRunning",
            "DetectChange", "DetectIncrease", "DetectDecrease", "Memory", "Downsample", "Upsample", "RateTransition",
            "FIRDecimation", "FIRInterpolation", "CICDecimation", "CICInterpolation")
EXHAUSTIVE_MAX = 1 << 20


def memoryless(spec):
    return not any(b.get("type") in STATEFUL for b in spec["blocks"].values())


COUNTERS = ("CounterLimited", "CounterFreeRunning")
EXHAUSTIVE_PERIODIC_MAX = 1 << 23


def counter_period(spec):
    """The design's only state is counters that no input drives (an NCO-like phase): their common period in steps
    (None otherwise). Their state then runs through the same cycle whatever the input is."""
    blocks = spec["blocks"]
    if any(b.get("type") in STATEFUL and b.get("type") not in COUNTERS for b in blocks.values()):
        return None
    fed = {ln["dst"] for ln in spec["lines"]}
    period = 1
    for n, b in blocks.items():
        if b.get("type") in COUNTERS:
            if n in fed:
                return None
            try:
                p = int(float(b.get("uplimit", "7"))) + 1 if b["type"] == "CounterLimited" else 1 << int(b.get("NumBits", "16"))
            except ValueError:
                return None
            period = period * p // math.gcd(period, p)
    return period


def exhaustive_periodic(req, sol_blocks, in_types, ranges, period, keep=256):
    """One input, state = input-independent counters of period P: each stored input value held for P steps covers
    every (counter state, input value) pair exactly once -- the exact worst error over the range. The counter-
    example keeps the worst input values, each held for a full period (so their bad phases are revisited)."""
    (x, (lo, hi)), = ranges.items()
    ev = Evaluator(req, sol_blocks, in_types)
    vals = list(range(lo, hi + 1))
    seq = [v for v in vals for _ in range(period)]
    e = ev.errors({x: seq})
    per_val = [max(e[i * period:(i + 1) * period]) for i in range(len(vals))]
    worst = max(per_val) if per_val else 0.0
    if worst <= 1.0:
        return worst, None
    idx = sorted(range(len(vals)), key=lambda i: -per_val[i])[:max(1, keep // period)]
    return worst, {x: [vals[i] for i in sorted(idx) for _ in range(period)]}


def exhaustive(req, sol_blocks, in_types, ranges, keep=256):
    """A memoryless design with one input: every stored input value of the range, once. Returns (worst ratio over
    the whole range, the `keep` worst values as a counterexample sequence or None if none is over the tolerance).
    Exact for the range (not a search)."""
    (x, (lo, hi)), = ranges.items()
    ev = Evaluator(req, sol_blocks, in_types)
    vals = list(range(lo, hi + 1))
    e = ev.errors({x: vals})
    worst = max(e) if e else 0.0
    if worst <= 1.0:
        return worst, None
    idx = sorted(range(len(vals)), key=lambda i: -e[i])[:keep]
    return worst, {x: [vals[i] for i in sorted(idx)]}


_START_FN = None


def _start_worker(s):
    return _START_FN(s)


def _map_starts(fn, starts, workers):
    """fn(s) for s in range(starts), in order; in `workers` forked processes when more than one (fn and what it
    closes over are inherited by fork, nothing is pickled but the start index and the result)."""
    global _START_FN
    n = max(1, min(int(workers or 1), starts))
    if n == 1:
        return [fn(s) for s in range(starts)]
    import multiprocessing as mp
    _START_FN = fn
    try:
        with mp.get_context("fork").Pool(n) as pool:
            return pool.map(_start_worker, range(starts), chunksize=1)
    finally:
        _START_FN = None


def search(req, sol_blocks, in_types, ranges, seed=0, rounds=16, starts=24, stats=None, workers=1):
    """ranges: {Inport: (lo, hi)} stored integers. A fixed number of starts and hill-climb rounds, so the result
    does not depend on machine load (fxopt's results must be the same for the same request). Returns (worst ratio
    found, sequence or None if <= 1); stats (dict) receives starts_done and cap_hit (True if the safety stop cut
    the search short, which makes the result load-dependent and is reported)."""
    t_end = time.monotonic() + CAP_S
    if len(ranges) == 1 and memoryless(req["design"]):
        (lo, hi), = ranges.values()
        if hi - lo + 1 <= EXHAUSTIVE_MAX:
            if stats is not None:
                stats.update(exhaustive=True, starts_done=0, cap_hit=False)
            return exhaustive(req, sol_blocks, in_types, ranges)
    if len(ranges) == 1:
        per = counter_period(req["design"])
        (lo, hi), = ranges.values()
        if per is not None and per * (hi - lo + 1) <= EXHAUSTIVE_PERIODIC_MAX:
            if stats is not None:
                stats.update(exhaustive=True, starts_done=0, cap_hit=False)
            return exhaustive_periodic(req, sol_blocks, in_types, ranges, per)
    ev = Evaluator(req, sol_blocks, in_types)
    M = memory_estimate(req["design"])
    W = max(4, min(64, M))
    xs = sorted(ranges)
    stride = W + M + 4

    def score(cands):
        seqs = {x: [] for x in xs}
        for c in cands:
            for x in xs:
                seqs[x] += [0 if ranges[x][0] <= 0 <= ranges[x][1] else ranges[x][0]] * (M + 4) + c[x]
        for x in xs:
            seqs[x] += [seqs[x][-1]] * M
        e = ev.errors(seqs)
        return [max(e[j * stride + M + 4: j * stride + M + 4 + W + M] or [0.0]) for j in range(len(cands))]

    best, best_c = 0.0, None
    done = 0
    # slow drift in integrating designs (a clamped accumulator, a limited integrator) shows only after
    # thousands of steps of a small constant input -- the windows below are far too short. A fixed set of constant
    # inputs (powers of two LSB of either sign, and the range ends) is tried over DC_STEPS steps first.
    dc_best, dc_seq = 0.0, None
    if integrating(req["design"]):
        levels = set()
        for x in xs:
            lo, hi = ranges[x]
            k = 1
            while k <= max(abs(lo), abs(hi)):
                levels |= {k, -k}
                k *= 2
        for lv in sorted(levels, key=lambda v: (abs(v), v)):
            if time.monotonic() > t_end:
                break
            seq = {}
            for x in xs:
                lo, hi = ranges[x]
                seq[x] = [min(hi, max(lo, lv))] * DC_STEPS
            e = max(ev.errors(seq) or [0.0])
            if e > dc_best:
                dc_best, dc_seq = e, seq
    def one_start(s):
        # recordings (fxopt default mode 0.72 x fxpopt's speed on a 259-block chain; the search was 275 s
        # of 765): every start has its own generator, so the starts run in parallel and the result does not depend
        # on the number of processes
        rng_s = random.Random("%d:%d" % (seed, s))
        cur = {x: [rng_s.randint(*ranges[x]) for _ in range(W)] for x in xs}
        if s == 1:
            cur = {x: [ranges[x][(k % 2)] for k in range(W)] for x in xs}
        val = score([cur])[0]
        for _ in range(rounds):
            if time.monotonic() > t_end:
                break
            cands = []
            for x in xs:
                lo, hi = ranges[x]
                for k in range(W):
                    vals = {lo, hi, 0 if lo <= 0 <= hi else lo} | {rng_s.randint(lo, hi) for _ in range(3)} | \
                           {min(hi, max(lo, cur[x][k] + d)) for d in (-2, -1, 1, 2)}
                    for v in vals - {cur[x][k]}:
                        c = {y: list(cur[y]) for y in xs}
                        c[x][k] = v
                        cands.append(c)
            if not cands:   # every input range is a single value: no neighbour to try (
                break       # an all-zero input crashed here with IndexError)
            r = score(cands)
            i = max(range(len(r)), key=r.__getitem__)
            if r[i] <= val:
                break
            val, cur = r[i], cands[i]
        return val, cur
    results = _map_starts(one_start, starts, workers)
    for val, cur in results:   # in start order: the earliest of equal values wins, whatever ran first
        done += 1
        if val > best:
            best, best_c = val, cur
    if stats is not None:
        stats["starts_done"] = done
        stats["cap_hit"] = time.monotonic() > t_end
    if dc_best > 1.0 and dc_best >= best:
        return dc_best, dc_seq
    if best_c is None or best <= 1.0:
        if stats is not None and best_c is not None and best >= dc_best:
            # recordings (the error reporting contract): the worst input found, even within the tolerance, so an agent can
            # replay the search's closest call (summary.max_error_over_tolerance comes from it)
            stats["worst_seq"] = {x: [0 if ranges[x][0] <= 0 <= ranges[x][1] else ranges[x][0]] * (M + 4) +
                                  best_c[x] + [best_c[x][-1]] * M for x in xs}
        elif stats is not None and dc_seq is not None:
            stats["worst_seq"] = dc_seq
        return max(best, dc_best), None
    # confirm on its own: quiet lead-in, the window, quiet tail
    seq = {x: [0 if ranges[x][0] <= 0 <= ranges[x][1] else ranges[x][0]] * (M + 4) + best_c[x] +
           [best_c[x][-1]] * M for x in xs}
    alone = max(ev.errors(seq))
    return alone, (seq if alone > 1.0 else None)

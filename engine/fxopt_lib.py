"""fxopt library: word-length search for cascaded fixed-point FIR chains (see docs/en/OPTIMIZER.md).

Pieces:
  * Problem      -- design, input, reference ranges, cost weights, type derivation from word lengths
  * plan_tokens  -- candidate -> fxchain_core token line (same protocol and semantics as engine/fxchain)
  * Model        -- analytic error model (Floor bias + noise + coefficient phasor) for a fast greedy
  * Pool         -- persistent `fxchain_core -s` workers (bit-exact evaluation, per-stage reuse)
  * optimise     -- model greedy -> bit-exact calibration -> bit-exact greedy refinement
Everything is deterministic for a given request (no wall-clock dependence unless the time budget is hit).
"""
import cmath
import json
import math
import os
import subprocess
import threading
import time
from fractions import Fraction

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CORE = os.path.join(HERE, "fxchain_core")


def is_set_wrapper(e, inports):
    """Recognise a wrapped input set with an optional tolerance scale. A port named
    inputs takes precedence if no scale is supplied."""
    if not isinstance(e, dict):
        return False
    if "tolerance_scale" in e:
        # an Inport named tolerance_scale, given without "inputs", is a plain set
        return "inputs" in e or "tolerance_scale" not in inports
    return set(e) == {"inputs"} and isinstance(e["inputs"], dict) and "inputs" not in inports


class OptError(Exception):
    def __init__(self, code, where, message):
        Exception.__init__(self, message)
        self.code, self.where, self.message = code, where, message


def strict_type(t, where):
    """{s, wl, fl} with integer fields, s in {0, 1} (no truthiness, no truncation)."""
    if not isinstance(t, dict):
        raise OptError("BAD_PARAM", where, "{s, wl, fl} required")
    extra = sorted(set(t) - {"s", "wl", "fl"})
    if extra:
        raise OptError("BAD_PARAM", where + "." + extra[0], "unknown field %s" % extra[0])
    for k in ("s", "wl", "fl"):
        if k not in t:
            raise OptError("BAD_PARAM", where + "." + k, "{s, wl, fl} required")
        if not isinstance(t[k], int) or isinstance(t[k], bool):
            raise OptError("BAD_TYPE", where + "." + k, "%s must be an integer, got %r" % (k, t[k]))
    if t["s"] not in (0, 1):
        raise OptError("BAD_TYPE", where + ".s", "s must be 0 or 1")
    return t["s"], t["wl"], t["fl"]


def stored_range(lo, hi, f):
    """Stored-integer range [ceil(lo * 2^f), floor(hi * 2^f)] computed exactly (in doubles a 64-bit
    bound such as 2^63 - 1001 rounded to 2^63 - 1024, narrowing the proven input range)."""
    from fractions import Fraction
    sc = Fraction(2) ** f
    return math.ceil(Fraction(lo) * sc), math.floor(Fraction(hi) * sc)


def strict_number(spec, key, where, integer=False, default=None):
    if key not in spec:
        if default is not None:
            return default
        raise OptError("BAD_PARAM", where + "." + key, "%s is required" % key)
    v = spec[key]
    ok = isinstance(v, int) if integer else isinstance(v, (int, float))
    if not ok or isinstance(v, bool) or (isinstance(v, float) and not math.isfinite(v)):
        raise OptError("BAD_PARAM", where + "." + key, "%s must be a %s number, got %r"
                       % (key, "an integer" if integer else "finite", v))
    return v


def strict_ints(ints, where):
    if not isinstance(ints, list) or not ints:
        raise OptError("MISSING_FIELD", where, "stored integers required")
    if not all(isinstance(v, int) and not isinstance(v, bool) for v in ints):
        raise OptError("BAD_INPUT", where, "integers required (no floats or booleans)")
    return ints


def check_fields(obj, allowed, where):
    extra = sorted(set(obj) - set(allowed))
    if extra:
        raise OptError("BAD_PARAM", (where + "." if where else "") + extra[0], "unknown field %s" % extra[0])


# ---------------------------------------------------------------- exact helpers (same rules as ref/fxcore.py)
def _round_nearest(num, den):
    """round(num/den) with ties toward +inf (MATLAB 'Nearest'); den > 0."""
    return (2 * num + den) // (2 * den)


def quant_param(g, s, w, f):
    x = Fraction(g) * (Fraction(2) ** f)
    n = _round_nearest(x.numerator, x.denominator)
    lo, hi = (-(1 << (w - 1)), (1 << (w - 1)) - 1) if s else (0, (1 << w) - 1)
    return max(lo, min(hi, n))


def best_fl(g, s, w):
    """Largest fraction length whose nearest-rounded value fits (s, w) (best-precision scaling)."""
    gf = Fraction(g)
    lo, hi = (-(1 << (w - 1)), (1 << (w - 1)) - 1) if s else (0, (1 << w) - 1)
    a = abs(gf)
    f = w + 2 - (a.numerator.bit_length() - a.denominator.bit_length())
    while True:
        x = gf * (Fraction(2) ** f)
        n = _round_nearest(x.numerator, x.denominator)
        if lo <= n <= hi:
            return f
        f -= 1


def int_bits(m):
    """Integer bits (incl. sign) of a signed type that holds magnitudes up to m: smallest I with 2^(I-1) > m."""
    if m <= 0:
        return -60
    return math.floor(math.log2(m)) + 2


def tz(n):
    n = abs(n)
    return (n & -n).bit_length() - 1 if n else 0


# ---------------------------------------------------------------- problem definition
class Problem:
    def __init__(self, req):
        d = req.get("design")
        if not isinstance(d, dict):
            raise OptError("MISSING_FIELD", "design", "design object is required")
        try:
            self.ns, self.nt = int(d["nstages"]), int(d["ntaps"])
        except (KeyError, TypeError, ValueError):
            raise OptError("MISSING_FIELD", "design", "design.nstages and design.ntaps are required")
        if not (1 <= self.ns <= 16 and 1 <= self.nt <= 256):
            raise OptError("LIMIT_EXCEEDED", "design", "nstages 1..16 and ntaps 1..256 supported by fxopt")
        self.gstr = self._gains(req)
        self.g = [[float(v) for v in row] for row in self.gstr]
        for s in range(self.ns):
            for k in range(self.nt):
                if not math.isfinite(self.g[s][k]):
                    raise OptError("BAD_PARAM", "design.gains[%d][%d]" % (s, k), "gain must be finite")
        self.rnd = req.get("rounding", "Floor")
        self.sat = req.get("saturate", "off")
        if self.rnd != "Floor" or self.sat != "off":
            raise OptError("UNSUPPORTED", "rounding", "fxopt currently supports RndMeth Floor with saturation off only")
        wls = req.get("wordlengths", list(range(6, 33, 2)))
        if not (isinstance(wls, list) and wls and all(isinstance(w, int) and 2 <= w <= 32 for w in wls)):
            raise OptError("BAD_PARAM", "wordlengths", "list of integers 2..32 required")
        self.wls = sorted(set(wls))
        err = req.get("error", {})
        try:
            self.tol = float(err.get("max_abs", 2.0 ** -10))
        except (TypeError, ValueError, AttributeError):
            raise OptError("BAD_PARAM", "error.max_abs", "number required")
        if not (self.tol > 0):
            raise OptError("BAD_PARAM", "error.max_abs", "must be positive")
        self._input(req.get("input"))
        self.nvar_stage = 2 * self.nt + 2
        self.nvar = self.ns * self.nvar_stage
        # cost weights: gain out, param, acc: 1; stage out: 1 + (nt-1) unit delays of the next stage
        self.weight = []
        for s in range(self.ns):
            self.weight += [1] * (2 * self.nt) + [1, 1 + ((self.nt - 1) if s < self.ns - 1 else 0)]
        self.const_cost = (self.nt - 1) * self.xt[1] + self.xt[1]  # stage-1 delays + subsystem input port

    def _gains(self, req):
        d = req["design"]
        if "gains" in d:
            gs = d["gains"]
            if isinstance(gs, list) and gs and not isinstance(gs[0], list):
                gs = [gs] * self.ns
            if not (isinstance(gs, list) and len(gs) == self.ns and all(isinstance(r, list) and len(r) == self.nt for r in gs)):
                raise OptError("BAD_PARAM", "design.gains", "list of ntaps gains (or nstages lists of them) required")
            return [[str(v) for v in r] for r in gs]
        b = req.get("blocks")
        if not isinstance(b, dict):
            raise OptError("MISSING_FIELD", "design.gains", "design.gains or blocks (with Gain strings) required")
        out = []
        for s in range(1, self.ns + 1):
            row = []
            for k in range(1, self.nt + 1):
                p = b.get("G%d_%d" % (s, k))
                if not isinstance(p, dict) or "Gain" not in p:
                    raise OptError("MISSING_FIELD", "blocks.G%d_%d.Gain" % (s, k), "gain missing")
                row.append(str(p["Gain"]))
            out.append(row)
            a = b.get("Acc%d" % s, {})
            # the one port-count parser (fxnum.port_count, as fxgraph / fxchain); isdigit() / int()
            # read '\u0661\u0666' as 16 and crashed INTERNAL on '\u00b3'
            import fxnum
            pc = fxnum.port_count(a.get("Inputs", "+" * self.nt), "+-|")
            if isinstance(pc, fxnum.NumError):
                raise OptError(pc.code, "blocks.Acc%d.Inputs" % s, pc.message)
            sg = pc.replace("|", "")
            if not set(sg) <= set("+-"):
                sg = "+" * int(sg)
            if sg != "+" * self.nt:
                raise OptError("UNSUPPORTED", "blocks.Acc%d.Inputs" % s, "fxopt supports all-'+' sums only")
        return out

    def _input(self, spec):
        if not isinstance(spec, dict):
            raise OptError("MISSING_FIELD", "input", "input object is required")
        check_fields(spec, ("kind", "type", "amplitude", "omega", "n", "ints"), "input")
        self.xt = strict_type(spec.get("type", {"s": 1, "wl": 16, "fl": 14}), "input.type")
        s, w, f = self.xt
        if not (2 <= w <= 32):
            raise OptError("BAD_PARAM", "input.type.wl", "2..32 required")
        lo, hi = (-(1 << (w - 1)), (1 << (w - 1)) - 1) if s else (0, (1 << w) - 1)
        kind = spec.get("kind", "ints")
        self.sine = None
        if kind == "sine":
            amp, om = float(strict_number(spec, "amplitude", "input")), float(strict_number(spec, "omega", "input"))
            n = strict_number(spec, "n", "input", integer=True)
            if not (1 <= n <= 2_000_000):
                raise OptError("LIMIT_EXCEEDED", "input.n", "1..2000000 samples")
            ints = [int(math.floor(amp * math.sin(om * i) * (1 << f))) if f >= 0 else
                    int(math.floor(amp * math.sin(om * i) / (1 << -f))) for i in range(n)]
            self.sine = (amp, om)
        elif kind == "ints":
            ints = strict_ints(spec.get("ints"), "input.ints")
        else:
            raise OptError("BAD_PARAM", "input.kind", "sine or ints")
        if any(v < lo or v > hi for v in ints):
            raise OptError("BAD_INPUT", "input.ints", "stored integer out of range for input type")
        self.x = ints

    # ---- reference (double) and ranges
    def reference(self):
        """Double-precision cascade on the quantised x (same summation order as the core's DREF)."""
        f = self.xt[2]
        u = [math.ldexp(v, -f) for v in self.x]
        n = len(u)
        stages = []
        for s in range(self.ns):
            y = [0.0] * n
            for k, c in enumerate(self.g[s]):
                if k >= n:
                    break
                y[k:] = [a + c * b for a, b in zip(y[k:], u)]
            stages.append(y)
            u = y
        return stages

    def set_ranges(self, stages):
        f = self.xt[2]
        self.umax = [max(abs(math.ldexp(v, -f)) for v in self.x)] + [max(map(abs, y)) for y in stages[:-1]]
        self.ymax = [max(map(abs, y)) for y in stages]
        slack = self.tol * 2  # fixed-point values deviate from the double ones by about the error budget
        self.IBy = [int_bits(m + slack) for m in self.ymax]
        self._ibg = {}
        self._par = {}

    # ---- types from word lengths
    def param(self, s, k, w):
        key = (s, k, w)
        r = self._par.get(key)
        if r is None:
            g = self.g[s][k]
            if g == 0:
                ps, pf = 1, w - 1
            else:
                ps = 1 if g < 0 else 0
                pf = best_fl(self.gstr[s][k], ps, w)
            npar = quant_param(self.gstr[s][k], ps, w, pf)
            r = (ps, w, pf, npar, math.ldexp(npar, -pf))
            self._par[key] = r
        return r

    def IBg(self, s, k, pw):
        key = (s, k, pw)
        r = self._ibg.get(key)
        if r is None:
            gq = abs(self.param(s, k, pw)[4])
            r = int_bits(gq * (self.umax[s] + 2 * self.tol) + 2.0 ** -40)
            self._ibg[key] = r
        return r

    def types(self, wl):
        """wl vector -> per stage: [(param(s,w,f), npar), ...], [gain out (s,w,f)], acc, out."""
        res = []
        for s in range(self.ns):
            b = s * self.nvar_stage
            nt = self.nt
            pars, gouts = [], []
            for k in range(nt):
                pw, gw = wl[b + k], wl[b + nt + k]
                ps, _, pf, npar, _ = self.param(s, k, pw)
                pars.append(((ps, pw, pf), npar))
                gouts.append((1, gw, gw - self.IBg(s, k, pw)))
            aw, ow = wl[b + 2 * nt], wl[b + 2 * nt + 1]
            res.append((pars, gouts, (1, aw, aw - self.IBy[s]), (1, ow, ow - self.IBy[s])))
        return res

    def hwbits(self, wl):
        return self.const_cost + sum(w * c for w, c in zip(self.weight, wl))

    def blocks(self, wl):
        """Simulink block parameter strings for a solution."""
        out = {}
        for s, (pars, gouts, acc, o) in enumerate(self.types(wl)):
            for k in range(self.nt):
                (ps, pw, pf), _ = pars[k]
                out["G%d_%d" % (s + 1, k + 1)] = {
                    "Gain": self.gstr[s][k], "OutDataTypeStr": "fixdt(%d,%d,%d)" % gouts[k],
                    "ParamDataTypeStr": "fixdt(%d,%d,%d)" % (ps, pw, pf),
                    "RndMeth": self.rnd, "SaturateOnIntegerOverflow": self.sat}
            out["Acc%d" % (s + 1)] = {
                "Inputs": "+" * self.nt, "OutDataTypeStr": "fixdt(%d,%d,%d)" % o,
                "AccumDataTypeStr": "fixdt(%d,%d,%d)" % acc, "RndMeth": self.rnd,
                "SaturateOnIntegerOverflow": self.sat}
        return out


# ---------------------------------------------------------------- core token planning (mirrors engine/fxchain)
FAST_BITS = 62


def plan_tokens(pb, wl):
    """Candidate -> (core 'C' line, final fraction length). Floor (rm 0) and wrap (sat 0) only."""
    toks = []
    fast = pb.xt[1] <= FAST_BITS
    cur = pb.xt
    ofl = None
    for (pars, gouts, acc, out) in pb.types(wl):
        si, wi, fi = cur
        for (pt, npar), ot in zip(pars, gouts):
            full_s, full_w, full_f = (1 if (si or pt[0]) else 0), wi + pt[1], fi + pt[2]
            sh = full_f - ot[2]
            ash = ot[2] - acc[2]
            fast = fast and wi + pt[1] <= FAST_BITS and full_w + max(0, -sh) <= FAST_BITS and sh <= 60 \
                and ot[1] + max(0, -ash) <= FAST_BITS and ash <= 60
            toks.append("%d %d 0 %d %d 0 %d 0 %d %d 0 1" % (npar, sh, ot[0], ot[1], ash, acc[0], acc[1]))
        osh = acc[2] - out[2]
        fast = fast and acc[1] + max(0, -osh) <= FAST_BITS and osh <= 60
        toks.append("0 0 0 %d %d 0 %d 0 %d %d 0" % (acc[0], acc[1], osh, out[0], out[1]))
        cur = out
        ofl = out[2]
    return "C %d %d 0 %d %d %s" % (pb.ns, pb.nt, ofl, 1 if fast else 0, " ".join(toks)), ofl


# ---------------------------------------------------------------- analytic error model
class Model:
    """E = |B| + A + z*sqrt(V):
    B  = DC sum of Floor biases propagated through downstream DC gains,
    V  = variance of Floor noise propagated through downstream impulse-response energy,
    A  = amplitude of the coefficient-quantisation error (phasor at the sine frequency, or a
         worst-case bound sum|dh|*peak for arbitrary inputs)."""

    Z = 4.0

    def __init__(self, pb):
        self.pb = pb
        ns, nt = pb.ns, pb.nt
        self.ddc = []
        self.den = []
        for s in range(ns):
            dc, h = 1.0, [1.0]
            for t in range(s + 1, ns):
                dc *= sum(pb.g[t])
                h = _conv(h, pb.g[t])
            self.ddc.append(dc)
            self.den.append(sum(v * v for v in h))
        if pb.sine:
            amp, om = pb.sine
            hw = [sum(c * cmath.exp(-1j * om * k) for k, c in enumerate(pb.g[s])) for s in range(ns)]
            self.zk = []
            for s in range(ns):
                pre = amp
                for t in range(ns):
                    if t != s:
                        pre *= hw[t]
                self.zk.append([pre * cmath.exp(-1j * om * k) for k in range(nt)])
            self.phasor = True
            self._build_peak(len(pb.x))
        else:
            self.zk = [[pb.umax[s] * abs(self.ddc[s]) * (1 + 0j)] * nt for s in range(ns)]
            self.phasor = False

    def ufl(self, wl, s):
        pb = self.pb
        if s == 0:
            return pb.xt[2]
        return wl[(s - 1) * pb.nvar_stage + 2 * pb.nt + 1] - pb.IBy[s - 1]

    def tap(self, s, k, pw, gw, af, ufl):
        """(bias, var, coefficient error term) of one tap."""
        pb = self.pb
        ps, _, pf, npar, gq = pb.param(s, k, pw)
        dh = gq - pb.g[s][k]
        cterm = dh * self.zk[s][k] if self.phasor else abs(dh) * self.zk[s][k]
        if npar == 0:
            return 0.0, 0.0, cterm
        ep = -(ufl + pf - tz(npar))
        eff = min(gw - pb.IBg(s, k, pw), af)
        if -eff > ep:
            q, qp = 2.0 ** -eff, 2.0 ** ep
            return -(q - qp) / 2, (q * q - qp * qp) / 12, cterm
        return 0.0, 0.0, cterm

    @staticmethod
    def outterm(af, of):
        if of < af:
            q, qa = 2.0 ** -of, 2.0 ** -af
            return -(q - qa) / 2, (q * q - qa * qa) / 12
        return 0.0, 0.0

    def stage_terms(self, wl, s, ufl=None):
        pb = self.pb
        b = s * pb.nvar_stage
        nt = pb.nt
        af = wl[b + 2 * nt] - pb.IBy[s]
        of = wl[b + 2 * nt + 1] - pb.IBy[s]
        if ufl is None:
            ufl = self.ufl(wl, s)
        taps = [self.tap(s, k, wl[b + k], wl[b + nt + k], af, ufl) for k in range(nt)]
        return taps, self.outterm(af, of)

    def combine(self, B, V, C):
        sig = math.sqrt(max(V, 0.0))
        if not self.phasor:
            return abs(B) + C.real + self.Z * sig
        A = abs(C)
        if sig <= 0.0:
            return abs(B) + A
        return abs(B) + sig * self.peak(A / sig)

    def peak(self, a):
        """Median of max_n (a*sin(theta_n) + g_n), g_n ~ N(0,1), over the record length (tabulated)."""
        tab = self._peak_tab
        if a >= tab[-1][0]:
            return a + (tab[-1][1] - tab[-1][0])
        i = min(int(a / self._peak_step), len(tab) - 2)
        (a0, f0), (a1, f1) = tab[i], tab[i + 1]
        return f0 + (f1 - f0) * (a - a0) / (a1 - a0)

    def _build_peak(self, n):
        th = [math.sin(-math.pi / 2 + math.pi * (j + 0.5) / 48) for j in range(48)]  # sin is symmetric

        def logphi(x):
            return math.log1p(-0.5 * math.erfc(x / math.sqrt(2.0)))

        def med(a):
            lo, hi = -2.0 + 0.0 * a, a + 8.0
            target = math.log(0.5) / n
            for _ in range(40):
                t = 0.5 * (lo + hi)
                if sum(logphi(t - a * s) for s in th) / len(th) < target:
                    lo = t
                else:
                    hi = t
            return 0.5 * (lo + hi)

        self._peak_step = 0.1
        self._peak_tab = [(k * self._peak_step, med(k * self._peak_step)) for k in range(161)]

    def evaluate(self, wl):
        B = V = 0.0
        C = 0j
        for s in range(self.pb.ns):
            taps, (ob, ov) = self.stage_terms(wl, s)
            B += self.ddc[s] * (sum(t[0] for t in taps) + ob)
            V += self.den[s] * (sum(t[1] for t in taps) + ov)
            C += sum(t[2] for t in taps)
        return self.combine(B, V, C), (B, V, C)


def _conv(a, b):
    out = [0.0] * (len(a) + len(b) - 1)
    for i, x in enumerate(a):
        for j, y in enumerate(b):
            out[i + j] += x * y
    return out


def model_greedy(pb, model, start, cap, levels, maxstep=2):
    """Greedy descent on the analytic model: repeatedly apply the single 'next smaller word length'
    move with the smallest (model error increase) / (cost decrease) while the model error stays <= cap.
    Returns snapshots {level: wl} = last configuration whose model error <= level."""
    ns, nt, nvs = pb.ns, pb.nt, pb.nvar_stage
    wl = list(start)
    wls = pb.wls
    lower = {w: (wls[i - 1] if i > 0 else None) for i, w in enumerate(wls)}
    # per-stage cached terms
    taps, outs = [], []
    for s in range(ns):
        t, o = model.stage_terms(wl, s)
        taps.append(t)
        outs.append(o)

    def totals():
        B = V = 0.0
        C = 0j
        for s in range(ns):
            B += model.ddc[s] * (sum(t[0] for t in taps[s]) + outs[s][0])
            V += model.den[s] * (sum(t[1] for t in taps[s]) + outs[s][1])
            C += sum(t[2] for t in taps[s])
        return B, V, C

    B, V, C = totals()
    E = model.combine(B, V, C)
    snaps = {}
    for lv in levels:
        if E <= lv:
            snaps[lv] = list(wl)
    deltas = [None] * pb.nvar

    def options(i):
        """Moves of variable i one or more allowed word lengths down: [(dB, dV, dC, local state, new wl)]."""
        res = []
        nw = wl[i]
        for _ in range(maxstep):
            nw = lower[nw]
            if nw is None:
                break
            res.append(delta(i, nw) + (nw,))
        return res

    def delta(i, nw):
        """(dB, dV, dC, new-local-state) for setting variable i to word length nw."""
        s, j = divmod(i, nvs)
        b = s * nvs
        if j < 2 * nt:
            k = j % nt
            old = taps[s][k]
            pw, gw = wl[b + k], wl[b + nt + k]
            if j < nt:
                pw = nw
            else:
                gw = nw
            af = wl[b + 2 * nt] - pb.IBy[s]
            new = model.tap(s, k, pw, gw, af, model.ufl(wl, s))
            return (model.ddc[s] * (new[0] - old[0]), model.den[s] * (new[1] - old[1]), new[2] - old[2],
                    ("tap", s, k, new))
        if j == 2 * nt:  # accumulator: all taps of the stage + output term
            ufl = model.ufl(wl, s)
            af = nw - pb.IBy[s]
            of = wl[b + 2 * nt + 1] - pb.IBy[s]
            nt_ = [model.tap(s, k, wl[b + k], wl[b + nt + k], af, ufl) for k in range(nt)]
            no = model.outterm(af, of)
            dB = model.ddc[s] * (sum(t[0] for t in nt_) + no[0] - sum(t[0] for t in taps[s]) - outs[s][0])
            dV = model.den[s] * (sum(t[1] for t in nt_) + no[1] - sum(t[1] for t in taps[s]) - outs[s][1])
            return dB, dV, 0j, ("acc", s, nt_, no)
        # stage output: own output term + next stage's taps (their input grid changes)
        af = wl[b + 2 * nt] - pb.IBy[s]
        of = nw - pb.IBy[s]
        no = model.outterm(af, of)
        dB = model.ddc[s] * (no[0] - outs[s][0])
        dV = model.den[s] * (no[1] - outs[s][1])
        nxt = None
        if s + 1 < ns:
            b2 = (s + 1) * nvs
            af2 = wl[b2 + 2 * nt] - pb.IBy[s + 1]
            nxt = [model.tap(s + 1, k, wl[b2 + k], wl[b2 + nt + k], af2, of) for k in range(nt)]
            dB += model.ddc[s + 1] * (sum(t[0] for t in nxt) - sum(t[0] for t in taps[s + 1]))
            dV += model.den[s + 1] * (sum(t[1] for t in nxt) - sum(t[1] for t in taps[s + 1]))
        return dB, dV, 0j, ("out", s, no, nxt)

    for i in range(pb.nvar):
        deltas[i] = options(i)
    steps = 0
    while True:
        best, bi, bE, bd = None, -1, None, None
        for i in range(pb.nvar):
            for d in deltas[i]:
                En = model.combine(B + d[0], V + d[1], C + d[2])
                if En > cap:
                    continue
                dc = pb.weight[i] * (wl[i] - d[4])
                r = (En - E) / dc
                if best is None or r < best:
                    best, bi, bE, bd = r, i, En, d
        if bi < 0:
            break
        d = bd
        st = d[3]
        B, V, C = B + d[0], V + d[1], C + d[2]
        wl[bi] = d[4]
        s = bi // nvs
        if st[0] == "tap":
            taps[s][st[2]] = st[3]
        elif st[0] == "acc":
            taps[s] = st[2]
            outs[s] = st[3]
        else:
            outs[s] = st[2]
            if st[3] is not None:
                taps[s + 1] = st[3]
        E = bE
        steps += 1
        # refresh deltas of every variable whose local terms may have changed
        aff = set()
        b = s * nvs
        aff.update(range(b, b + nvs))
        if s > 0:
            aff.add(b - 1)  # previous stage output feeds this stage's taps
        if s + 1 < ns:
            aff.update(range(b + nvs, b + 2 * nvs))
        for i in aff:
            deltas[i] = options(i)
        if steps % 64 == 0:  # re-sync accumulated floating sums
            B, V, C = totals()
            E = model.combine(B, V, C)
        for lv in levels:
            if E <= lv:
                snaps[lv] = list(wl)
    return snaps, steps


# ---------------------------------------------------------------- worker pool (bit-exact evaluation)
class Pool:
    def __init__(self, pb, nworkers, timeout_s):
        self.pb = pb
        self.n = nworkers
        self.timeout = timeout_s
        self.procs = []
        if not os.path.exists(CORE):
            raise OptError("CORE_UNAVAILABLE", "engine/fxchain_core", "core binary missing (run engine/build.sh)")
        s, w, f = pb.xt
        xline = "X %d %d %d %d %s\n" % (len(pb.x), s, w, f, " ".join(map(str, pb.x)))
        cs = " ".join(repr(c) for row in pb.g for c in row)
        dref = "DREF %d %d %s\n" % (pb.ns, pb.nt, cs)
        for _ in range(nworkers):
            p = subprocess.Popen([CORE, "-s"], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                 stderr=subprocess.DEVNULL, bufsize=1 << 16)
            p.stdin.write((xline + dref).encode())
            p.stdin.flush()
            self.procs.append(p)
        self.evals = 0
        self.core_ns = 0

    def close(self):
        for p in self.procs:
            try:
                p.stdin.write(b"END\n")
                p.stdin.close()
            except OSError:
                pass
        for p in self.procs:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()

    def _run(self, wi, lines, out):
        p = self.procs[wi]
        try:
            p.stdin.write("".join(lines).encode())
            p.stdin.flush()
            res = []
            for _ in range(sum(1 for l in lines if l.startswith("C") or l.startswith("BASE"))):
                r = self._read(p)
                res.append(r)
            out[wi] = res
        except Exception as e:  # noqa: BLE001 -- reported as CORE_FAILED below
            out[wi] = e

    @staticmethod
    def _read(p):
        rec = {}
        stages = []
        while True:
            line = p.stdout.readline().decode()
            if not line:
                raise OptError("CORE_FAILED", "core", "worker exited")
            parts = line.split()
            if parts[0] == "E":
                raise OptError(parts[1] if len(parts) > 1 else "CORE_FAILED", "core", " ".join(parts[2:]))
            if parts[0] == "R":
                rec["core_ns"] = int(parts[1])
                rec["nx"] = int(parts[2])
                rec["start"] = int(parts[4])
            elif parts[0] == "S":
                stages.append((int(parts[2]), int(parts[3])))
            elif parts[0] == "M" and "nx" in rec and "err" not in rec:
                md, F, sm = int(parts[1]), int(parts[2]), int(parts[3])
                rec["err"] = math.ldexp(md, -F)
                rec["mean"] = math.ldexp(sm, -F) / rec["nx"]
            elif parts[0] == "D":
                if "err" not in rec or "core_ns" not in rec:  # an incomplete answer is an error
                    raise OptError("CORE_FAILED", "core", "worker result incomplete")
                rec["stages"] = stages
                return rec
            else:
                raise OptError("CORE_FAILED", "core", "unexpected worker line %r" % line[:60])

    def evaluate(self, base, cands):
        """Evaluate candidates (wl vectors) bit-exactly. `base` (wl or None) is sent first to every
        worker as the reuse base. Returns list of records (err, mean, stages, core_ns) in input order."""
        lines_b = None
        if base is not None:
            lines_b = "BASE " + plan_tokens(self.pb, base)[0] + "\n"
        assign = [[] for _ in range(self.n)]
        for i, c in enumerate(cands):
            assign[i % self.n].append(i)
        out = [None] * self.n
        ths = []
        for wi in range(self.n):
            if not assign[wi]:
                continue
            lines = []
            if lines_b is not None:
                lines.append(lines_b)
            for i in assign[wi]:
                lines.append(plan_tokens(self.pb, cands[i])[0] + "\n")
            th = threading.Thread(target=self._run, args=(wi, lines, out))
            th.start()
            ths.append(th)
        for th in ths:
            th.join()
        res = [None] * len(cands)
        for wi in range(self.n):
            if not assign[wi]:
                continue
            r = out[wi]
            if isinstance(r, OptError):
                raise r
            if isinstance(r, Exception):
                raise OptError("CORE_FAILED", "core", str(r))
            if lines_b is not None:
                self.core_ns += r[0]["core_ns"]
                r = r[1:]
            for i, rec in zip(assign[wi], r):
                res[i] = rec
                self.core_ns += rec["core_ns"]
        self.evals += len(cands)
        return res


# ---------------------------------------------------------------- search driver
def _cost(pb, wl):
    """Objective of the search: the problem's weighted cost when it has one, else hwbits."""
    f = getattr(pb, "cost", None)
    return f(wl) if f is not None else pb.hwbits(wl)


def _single_moves(pb, pool, log, cur, e0, m0, step, phase):
    """Measure every single one-step move (step = lower or upper map). Returns [(d_err, d_cost, i, rec)]
    with d_cost = cost saved (negative for increases)."""
    idx = [i for i in range(pb.nvar) if step[cur[i]] is not None]
    cands = []
    for i in idx:
        c = list(cur)
        c[i] = step[c[i]]
        cands.append(c)
    res = pool.evaluate(cur, cands) if cands else []
    out = []
    c0 = _cost(pb, cur)
    for i, c, r in zip(idx, cands, res):
        log.add(phase, pb, c, r)
        dc = c0 - _cost(pb, c)
        if dc == 0:
            continue  # neither saves nor costs anything under this objective
        out.append((r["err"] - e0, dc, i, r))
    # knapsack weight: the bias (mean error) is the additive part; peak decreases do not add up
    for j, (de, dc, i, r) in enumerate(out):
        out[j] = (max(de, abs(r["mean"]) - abs(m0), 0.0), dc, i, r, de)
    return out


# Pair exchange is optional. It can reduce hardware cost on some designs
# and increase it on others. Enable it with FXOPT_PAIR_EXCHANGE=1.


PAIR_EXCHANGE = os.environ.get("FXOPT_PAIR_EXCHANGE", "0") != "0"


def _pair_exchange(pb, pool, log, cur, lower, upper):
    """Try an upward step on one variable and a downward step on one or all of its
    structural neighbours. Evaluate cheaper candidates in a batch and return the
    cheapest feasible (wl, rec), or None."""
    groups = getattr(pb, "exchange_groups", None)
    if not groups:
        return None
    c0 = _cost(pb, cur)
    cands, seen = [], set()
    for i, js in groups:
        if upper[cur[i]] is None:
            continue
        js = [j for j in js if lower[cur[j]] is not None]
        moves = [[j] for j in js] + ([js] if len(js) > 1 else [])
        for m in moves:
            c = list(cur)
            c[i] = upper[c[i]]
            for j in m:
                c[j] = lower[c[j]]
            t = tuple(c)
            if t in seen or _cost(pb, c) >= c0:
                continue
            seen.add(t)
            cands.append(c)
    if not cands:
        return None
    best = None
    for c, r in zip(cands, pool.evaluate(cur, cands)):
        log.add("pair-exchange", pb, c, r)
        if r["err"] <= pb.tol and (best is None or _cost(pb, c) < _cost(pb, best[0])):
            best = (c, r)
    return best


def _knapsack(start, downs, lower, budget, skip, tol):
    """Apply reductions in order of measured error increase per bit saved while the summed positive
    increments stay within budget (moves that reduce the error are always taken)."""
    if budget < 0:
        return None
    c = list(start)
    used = 0.0
    for de, dc, i, r, _ in sorted(downs, key=lambda t: (t[0] / t[1], t[4], t[2])):
        if i == skip or lower[c[i]] is None or r["err"] > tol:
            continue
        if used + max(de, 0.0) > budget:
            continue
        used += max(de, 0.0)
        c[i] = lower[c[i]]
    return c

class Log:
    """Append-only JSONL record of every bit-exact candidate evaluation."""

    def __init__(self, path):
        self.path = path
        self.f = open(path, "x") if path else None  # a fresh file per attempt, never appended to
        self.n = 0

    def add(self, phase, pb, wl, rec):
        self.n += 1
        if self.f:
            self.f.write(json.dumps({"id": self.n, "phase": phase, "hwbits": pb.hwbits(wl), "cost": _cost(pb, wl),
                                     "max_abs_diff": rec["err"], "feasible": rec["err"] <= pb.tol,
                                     "first_recomputed_stage": rec["start"], "core_ms": rec["core_ns"] / 1e6,
                                     "wl": "".join("%02d" % w for w in wl)}) + "\n")

    def close(self):
        if self.f:
            self.f.close()


def optimise(pb, pool, log, deadline, max_rounds=1000, progress=None):
    """Returns dict with the best feasible solution and search statistics."""
    stats = {"model_steps": 0, "rounds": 0, "phase_s": {}}
    t0 = time.monotonic()
    model = Model(pb)
    top = [max(pb.wls)] * pb.nvar
    levels = sorted(set([pb.tol * f / 20 for f in range(2, 41)] + [pb.tol * f / 200 for f in range(180, 221)]))
    snaps, steps = model_greedy(pb, model, top, 2 * pb.tol, levels, 1)
    stats["model_steps"] = steps
    stats["phase_s"]["model_greedy"] = round(time.monotonic() - t0, 3)
    # calibration: evaluate every distinct snapshot bit-exactly, keep the cheapest feasible one
    t1 = time.monotonic()
    uniq = []
    for lv in sorted(snaps):
        if not uniq or snaps[lv] != uniq[-1]:
            uniq.append(snaps[lv])
    res = pool.evaluate(None, uniq)
    best = None
    for wl, r in zip(uniq, res):
        log.add("calibrate", pb, wl, r)
        if r["err"] <= pb.tol and (best is None or _cost(pb, wl) < _cost(pb, best[0])):
            best = (wl, r)
    if best is None:
        r = pool.evaluate(None, [top])[0]
        log.add("calibrate", pb, top, r)
        if r["err"] > pb.tol:
            raise OptError("INFEASIBLE", "error.max_abs",
                           "even the widest allowed word lengths give max_abs_diff %.6g > %.6g" % (r["err"], pb.tol))
        best = (top, r)
    stats["phase_s"]["calibrate"] = round(time.monotonic() - t1, 3)
    stats["after_calibration"] = {"hwbits": pb.hwbits(best[0]), "max_abs_diff": best[1]["err"]}
    return refine(pb, pool, log, best, deadline, max_rounds, stats, progress)


WIDE_EXCHANGE = os.environ.get("FXOPT_WIDE_EXCHANGE", "1") != "0"


def refine(pb, pool, log, best, deadline, max_rounds, stats, progress=None):
    """Bit-exact refinement (problem-independent: needs pb.nvar, pb.wls, pb.tol, pb.hwbits and a pool
    whose evaluate(base, cands) returns records with err and mean). Each round measures every single
    one-step reduction (and, when those are exhausted, every one-step increase), then applies a knapsack
    of moves whose summed measured error increments fit fractions of the remaining slack; every
    combination is verified bit-exactly."""
    stats.setdefault("rounds", 0)
    t2 = time.monotonic()
    cur, cur_r = best
    lower = {w: (pb.wls[i - 1] if i > 0 else None) for i, w in enumerate(pb.wls)}
    upper = {w: (pb.wls[i + 1] if i + 1 < len(pb.wls) else None) for i, w in enumerate(pb.wls)}
    timed_out = False
    fracs = (1.0, 0.85, 0.7, 0.55, 0.4, 0.25, 0.1)
    exchange_tries = 0
    pair_tried = None   # the solution the pair exchange last failed on

    def pair_step():
        nonlocal pair_tried, timed_out
        if not PAIR_EXCHANGE or pair_tried == tuple(cur):
            return None
        if time.monotonic() > deadline:
            # a search skipped at the deadline is a budget cut, not a deterministic end
            timed_out = True
            return None
        got = _pair_exchange(pb, pool, log, cur, lower, upper)
        if got is None:
            pair_tried = tuple(cur)
        else:
            stats["pair_exchanges"] = stats.get("pair_exchanges", 0) + 1
        return got
    while stats["rounds"] < max_rounds:
        if time.monotonic() > deadline:
            timed_out = True
            break
        stats["rounds"] += 1
        c0 = _cost(pb, cur)
        e0 = cur_r["err"]
        slack = pb.tol - e0
        downs = _single_moves(pb, pool, log, cur, e0, cur_r["mean"], lower, "refine")
        feas = [(raw, dc, i) for w, dc, i, r, raw in downs if r["err"] <= pb.tol]
        combos = []
        if feas:
            for f in fracs:
                combos.append(_knapsack(cur, downs, lower, f * slack, None, pb.tol))
        else:
            # exchange: one increase frees error for several reductions
            if exchange_tries >= 2:
                got = pair_step()
                if got is None:
                    break
                cur, cur_r = got
                exchange_tries = 0
                continue
            exchange_tries += 1
            ups = _single_moves(pb, pool, log, cur, e0, cur_r["mean"], upper, "exchange")
            ups = [u for u in ups if u[4] < 0 and u[1] < 0]  # frees error and costs something
            ups.sort(key=lambda u: (u[4] / -u[1], u[2]))  # most error freed per bit spent first
            wide = exchange_tries == 2 and WIDE_EXCHANGE  # last attempt: every increase, also by two steps
            for _, dc, i, r, de in (ups if wide else ups[:2 * len(fracs)]):
                steps = [upper[cur[i]]]
                if wide and steps[0] is not None and upper.get(steps[0]) is not None:
                    steps.append(upper[steps[0]])
                for w2 in steps:
                    c = list(cur)
                    c[i] = w2
                    for f in (0.9, 0.6):
                        combos.append(_knapsack(c, downs, lower, f * (slack - de), i, pb.tol))
        combos = [c for c in combos if c is not None and _cost(pb, c) < c0]
        if os.environ.get("FXOPT_DEBUG"):
            print("dbg round", stats["rounds"], "feas", len(feas), "neg", sum(1 for d in feas if d[0] < 0),
                  "combos", len(combos), "slack", slack / pb.tol, flush=True)
        uniq = []
        for c in combos:
            if c not in uniq:
                uniq.append(c)
        chosen = None
        if uniq:
            cres = pool.evaluate(cur, uniq)
            for c, r in zip(uniq, cres):
                log.add("refine-combo", pb, c, r)
                if os.environ.get("FXOPT_DEBUG"):
                    print("   combo", c0 - _cost(pb, c), r["err"] / pb.tol, flush=True)
                if r["err"] <= pb.tol and (chosen is None or _cost(pb, c) < _cost(pb, chosen[0])):
                    chosen = (c, r)
        if chosen is None and feas:
            de, dc, i = min(feas, key=lambda t: (t[0] / t[1], t[2]))
            c = list(cur)
            c[i] = lower[c[i]]
            chosen = (c, [d[3] for d in downs if d[2] == i][0])
        if chosen is None:
            if exchange_tries >= 2:
                got = pair_step()
                if got is None:
                    break
                cur, cur_r = got
                exchange_tries = 0
            continue
        if feas:
            exchange_tries = 0
        cur, cur_r = chosen
        if progress:
            progress(stats["rounds"], _cost(pb, cur), cur_r["err"])
    stats["phase_s"]["refine"] = round(time.monotonic() - t2, 3)
    return {"wl": cur, "rec": cur_r, "stats": stats, "timed_out": timed_out}

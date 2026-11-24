"""proof that a compiled fixed-point design never overflows for any input sequence inside given input
ranges, at any time step (arbitrary horizon), starting from its initial state.

The proof runs on the C core's integer program (the same instructions that are simulated bit-exactly), so it is
about the fixed-point design itself, rounding included, not about the ideal signals. All arithmetic is exact
(Python integers and Fractions). Every quantisation point is checked BEFORE wrapping or saturating: a cast's
value after rounding, every partial sum of an accumulator, every step of a product chain, every left shift.
A point "fits" when that value lies inside its target type, so neither wrap-around nor saturation can occur
there (contract: saturation at a quantisation point counts as an overflow; an explicit Saturation block's
clamp is a value operation, not an overflow).

Structure: the slot dependency graph (delays included) is split into strongly connected components.
  * acyclic parts: interval arithmetic per instruction;
  * a feedback loop made only of linear instructions (gain, cast, sum, shift, negation, copy, constant):
    every cast is "x / 2^sh + e" with e in the interval of its rounding mode -- exact as long as nothing
    overflows, which is what is being proven (induction over time). With the one-step map s' = A s + B u,
    the states of the first K steps are enclosed exactly (prefix sums of |row . A^j B| over the input,
    rounding and constant terms), and a box S with s_0..s_{K-1} in S and A^K S + W_K in S is an inductive
    invariant: s_{n+K} = A^K s_n + w, w in W_K. Signals at step n >= K are A^K-terms on S plus the last K
    steps' contributions;
  * any other loop: interval iteration of the delay states until F(S) is in S (a post-fixed point, checked
    exactly), with one widening step to the states' type ranges; no post-fixed point -> unknown.
A failed proof only means "not proven": the bounds are conservative, it does not show an overflow happens.
"""
import hashlib
import json
from fractions import Fraction

import fxgraph_lib as G

LINEAR_OPS = {"CONST", "GAIN", "CAST", "COPY", "SUM", "SHIFT", "UN-neg"}
K_START, K_MAX = 64, 4096
MAX_STATES = 24
ITER = 200


class Unsupported(Exception):
    pass


def trange(s, w):
    return (-(1 << (w - 1)), (1 << (w - 1)) - 1) if s else (0, (1 << w) - 1)


def parse_ct(tok):
    sh, rm, s, w, sat = (int(v) for v in tok.split())
    return sh, rm, s, w, sat


def err_range(rm):
    """Interval of e in y = x / 2^sh + e for a rounding mode code (sh > 0)."""
    if rm == 0:  # Floor
        return (Fraction(-1), Fraction(0))
    if rm == 1:  # Ceiling
        return (Fraction(0), Fraction(1))
    if rm == 2:  # Zero
        return (Fraction(-1), Fraction(1))
    return (Fraction(-1, 2), Fraction(1, 2))  # nearest variants


def fdiv(a, sh):
    """floor(a / 2^sh) for an integer or Fraction a."""
    q = Fraction(a) / (1 << sh) if sh > 0 else Fraction(a) * (1 << -sh)
    return q.numerator // q.denominator


def cdiv(a, sh):
    return -fdiv(-Fraction(a), sh)


def rnd(a, sh, rm):
    """The C core's rounding of a / 2^sh (integer or Fraction a; same rules as rdiv128). Monotone in a, so
    rounding the ends of an interval gives the exact interval of rounded values."""
    q = Fraction(a) / (1 << sh) if sh > 0 else Fraction(a) * (1 << -sh)
    fl = q.numerator // q.denominator
    if fl == q:
        return fl
    if rm == 0:
        return fl
    if rm == 1:
        return fl + 1
    if rm == 2:
        return fl + 1 if q < 0 else fl
    c = 2 * (q - fl) - 1
    if c > 0:
        return fl + 1
    if c < 0:
        return fl
    if rm == 3:
        return fl + 1
    if rm == 4:
        return fl + 1 if q > 0 else fl
    return fl + (fl & 1)


def fmt(x):
    try:
        return "%.3g" % float(x)
    except OverflowError:
        return ">1e308"


def hull(*iv):
    return (min(a for a, _ in iv), max(b for _, b in iv))


def imul(x, y):
    p = (x[0] * y[0], x[0] * y[1], x[1] * y[0], x[1] * y[1])
    return (min(p), max(p))


def _ins_list(d):
    """[(op, dst, srcs, args)] in execution order; op 'UN' split into UN-neg / UN-abs."""
    out = []

    def add(e):
        op, sl, rest = e
        if op == "LOGIC":
            out.append(("LOGIC", sl[0], list(rest[1]), rest))
        elif op == "MPSW":
            out.append(("MPSW", sl[0], [sl[1]] + [x for x, _ in rest[3]], rest))
        elif op == "PRODN":
            out.append(("PRODN", sl[0], [x for x, _, _ in rest[7]], rest))
        elif op == "BITW":
            out.append(("BITW", sl[0], list(rest[5]), rest))
        elif op == "SUM":
            out.append(("SUM", sl[0], [x for x, _, _ in rest[4]], rest))
        elif op == "MINMAX":
            out.append(("MINMAX", sl[0], [x for x, _ in rest[3]], rest))
        elif op == "LUT":
            out.append(("LUT", sl[0], [sl[1]], rest))
        elif op == "LUTL":
            out.append(("LUTL", sl[0], [sl[1]], rest))
        elif op == "UN":
            neg = rest.split()[0] == "1"
            out.append(("UN-neg" if neg else "UN-abs", sl[0], sl[1:], rest))
        else:
            out.append((op, sl[0], list(sl[1:]), rest))
    for n in d.order:
        if n in d.vm:
            e = d.vm[n]
            for x in (e if isinstance(e, list) else [e]):
                add(x)
    return out


class Prover:
    def __init__(self, d, in_ints, initial="design", modular=None):
        """d: compiled Design; in_ints: {Inport: (lo, hi)} stored-integer ranges; initial: 'design' or 'any'.
        modular: (blocks, {exit block: (lo, hi) real range}) of the request's modular regions (engine/fxmodular.py):
        their Sums wrap by design and are not overflow points; the values leaving them have the proven range."""
        self.mod_blocks, self.mod_exits = (set(modular[0]), dict(modular[1])) if modular else (set(), {})
        if not getattr(d, "vm_ok", False):
            raise Unsupported("the design needs the Python evaluation path (too wide for the C core)")
        self.d = d
        # a Saturation's cast before its clamp (hidden slot <n>/c) with both limits inside the
        # output type is not an overflow (Simulink counts none there; fxgraph diagnostics skip it the same way)
        self.quiet_slots = {G._hid(n, "c") for n in (d.sat_cast_quiet() if hasattr(d, "sat_cast_quiet") else ())}
        self.ins = _ins_list(d)
        self.by_dst = {}
        for k, it in enumerate(self.ins):
            self.by_dst[it[1]] = k
        self.dly = {x: (s, ic) for n in d.names for (x, s, ic) in d.vm_dly.get(n, ())}
        self.in_ints = dict(in_ints)
        self.initial = initial
        self.val = {}
        self.points = {}
        self.notes = []
        self.loops = []
        self.status = "bounded"
        self.merge = False
        for k, it in enumerate(self.ins):
            if it[0] in ("BITW", "EXTR"):
                raise Unsupported("%s of block %s acts on bit patterns; no value bound" % (it[0], self._blk(it[1])))
            if it[0] == "PRODN" and (it[3][0] or any(dv for _, _, dv in it[3][7])):
                raise Unsupported("division in block %s: no range proof for quotients yet" % self._blk(it[1]))

    # ---- helpers
    @staticmethod
    def _blk(slot):
        return slot.split("\x1f")[0]

    def slot_range(self, x, seen=()):
        """Type range of a slot (for widening and the 'any' initial state)."""
        if x in self.in_ints:
            t = self.d.otype[x]
            return trange(t[0], t[1])
        if x in self.dly:
            if x in seen:
                raise Unsupported("cannot type delay slot %s" % x)
            return self.slot_range(self.dly[x][0], seen + (x,))
        op, _, srcs, a = self.ins[self.by_dst[x]]
        if op in ("REL", "LOGIC"):
            return (0, 1)
        if op == "CONST":
            v = int(a)
            return (v, v)
        if op == "COPY":
            return self.slot_range(srcs[0], seen + (x,))
        if op == "SAT":
            lo, hi = (int(v) for v in a.split())
            return (lo, hi)
        if op == "SHIFT":
            _, _, s, w = (int(v) for v in a.split())
            return trange(s, w)
        if op == "LUT":
            return (min(a.outs), max(a.outs))
        if op == "LUTL":
            return trange(*parse_ct(a.out_cast)[2:4])
        if op == "LUT2":
            return (min(a.tab), max(a.tab)) if not a.linear else trange(*parse_ct(a.out_cast)[2:4])
        if op == "PRODN":
            return trange(a[4], a[5])
        if op in ("GAIN", "UN-neg", "UN-abs"):
            ct = parse_ct(a.split(None, 1)[1])
        elif op in ("CAST", "PROD"):
            ct = parse_ct(a)
        elif op == "SUM":
            ct = parse_ct(a[3])
        elif op == "MINMAX":
            ct = parse_ct(a[2])
        elif op == "SWITCH":
            ct = parse_ct(" ".join(a.split()[2:7]))
        elif op == "MPSW":
            ct = parse_ct(a[3][0][1])
        else:
            raise Unsupported("no type for slot %s" % x)
        return trange(ct[2], ct[3])

    def _point(self, key, desc, lo, hi, s, w, sat, record):
        tmin, tmax = trange(s, w)
        ok = tmin <= lo and hi <= tmax
        if sat and self.ins[key[0]][1] in self.quiet_slots:
            return (min(max(lo, tmin), tmax), max(min(hi, tmax), tmin))
        if record:
            k, part = key
            op, dst = self.ins[k][0], self.ins[k][1]
            # which declared type this point is stored in: the accumulator for every step of a sum, else the
            # instruction's output type
            role = "accumulator" if op == "SUM" and part != 0 else \
                ("intermediate" if op in ("LUTL", "LUT2") and part != 0 else "output")
            new = {"where": desc, "slot": dst.replace("\x1f", "/"), "role": role, "min": lo, "max": hi,
                   "type_min": tmin, "type_max": tmax, "fits": ok, "on_overflow": "saturate" if sat else "wrap"}
            old = self.points.get(key)
            if old is not None and self.merge:  # two sound enclosures of the same point: keep the intersection
                new["min"], new["max"] = max(old["min"], lo), min(old["max"], hi)
                new["fits"] = tmin <= new["min"] and new["max"] <= tmax
            self.points[key] = new
        if ok:
            return (lo, hi)
        if sat:
            return (min(max(lo, tmin), tmax), max(min(hi, tmax), tmin))
        return (tmin, tmax)

    def _table_point(self, k, a, t, desc, record):
        """recordings: the conversion of the table values into the output / intermediate type is a quantisation
        point like any other: a table value that only fits after saturating (or wrapping) is an overflow."""
        conv = getattr(a, "table_conv", None)
        if conv is None:
            raise Unsupported("lookup record without its table conversion range")
        self._point((k, 9), desc, conv[0], conv[1], t[0], t[1], 1 if getattr(a, "saturate", False) else 0, record)

    def _frac_guard(self, a, tag):
        """recordings: the interpolation bound assumes the quantised fraction lies in [0, 1]; a fraction type with a
        negative fraction length can round it above 1 (Ceiling: 0.5 -> 2), so such a lookup is not proven."""
        ft = getattr(a, "frac_type", None)
        if ft is not None and ft[2] < 0:
            raise Unsupported("%s: fraction type %r with a negative fraction length (the fraction may exceed 1)"
                              % (tag, ft))

    def _cast(self, iv, ct, key, desc, record):
        sh, rm, s, w, sat = ct
        lo, hi = iv
        if sh > 0:
            lo, hi = rnd(lo, sh, rm), rnd(hi, sh, rm)
        elif sh < 0:
            lo, hi = lo << -sh, hi << -sh
        return self._point(key, desc, lo, hi, s, w, sat, record)

    # ---- interval semantics of one instruction
    def _eval(self, k, v, record):
        op, dst, srcs, a = self.ins[k]
        b = self._blk(dst)
        tag = dst.replace("\x1f", "/")
        if op == "CONST":
            x = int(a)
            return (x, x)
        if op in ("REL", "LOGIC"):
            return (0, 1)
        if op == "COPY":
            return v[srcs[0]]
        if op == "SAT":
            lo, hi = (int(t) for t in a.split())
            x = v[srcs[0]]
            return (min(max(x[0], lo), hi), max(min(x[1], hi), lo))
        if op == "CAST":
            return self._cast(v[srcs[0]], parse_ct(a), (k, 0), "%s: conversion" % tag, record)
        if op == "GAIN":
            g, ct = a.split(None, 1)
            g = int(g)
            return self._cast(imul(v[srcs[0]], (g, g)), parse_ct(ct), (k, 0), "%s: gain product" % tag, record)
        if op == "PROD":
            return self._cast(imul(v[srcs[0]], v[srcs[1]]), parse_ct(a), (k, 0), "%s: product" % tag, record)
        if op in ("UN-neg", "UN-abs"):
            x = v[srcs[0]]
            if op == "UN-neg":
                y = (-x[1], -x[0])
            else:
                y = (0 if x[0] <= 0 <= x[1] else min(abs(x[0]), abs(x[1])), max(abs(x[0]), abs(x[1])))
            return self._cast(y, parse_ct(a.split(None, 1)[1]), (k, 0), "%s: %s" % (tag, op[3:]), record)
        if op == "SHIFT":
            left, kk, s, w = (int(t) for t in a.split())
            x = v[srcs[0]]
            if left:
                return self._point((k, 0), "%s: left shift" % tag, x[0] << kk, x[1] << kk, s, w, 0, record)
            return (x[0] >> kk, x[1] >> kk)
        if op == "SUM":
            m, first, acc_ct, out_ct, parts = a
            act = parse_ct(acc_ct)
            acc = (0, 0)
            order = ([first] if first >= 0 else []) + [j for j in range(m) if j != first]
            for step, j in enumerate(order):
                x, ct, sg = parts[j]
                xv = self._cast(v[x], parse_ct(ct), (k, 1 + j), "%s: input %d into the accumulator" % (tag, j + 1),
                                record)
                if step == 0 and j == first:
                    acc = xv
                    continue
                acc = (acc[0] + xv[0], acc[1] + xv[1]) if sg > 0 else (acc[0] - xv[1], acc[1] - xv[0])
                acc = self._point((k, 1000 + step), "%s: accumulator after input %d" % (tag, j + 1),
                                  acc[0], acc[1], act[2], act[3], act[4], record)
            return self._cast(acc, parse_ct(out_ct), (k, 0), "%s: output conversion" % tag, record)
        if op == "MINMAX":
            m, ismax, out_ct, parts = a
            xs = [self._cast(v[x], parse_ct(ct), (k, 1 + j), "%s: input %d" % (tag, j + 1), record)
                  for j, (x, ct) in enumerate(parts)]
            f = max if ismax else min
            y = (f(x[0] for x in xs), f(x[1] for x in xs))
            return self._cast(y, parse_ct(out_ct), (k, 0), "%s: output conversion" % tag, record)
        if op == "SWITCH":
            t = a.split()
            c1, c3 = parse_ct(" ".join(t[2:7])), parse_ct(" ".join(t[7:12]))
            return hull(self._cast(v[srcs[1 - 1]], c1, (k, 1), "%s: input 1" % tag, record),
                        self._cast(v[srcs[2]], c3, (k, 3), "%s: input 3" % tag, record))
        if op == "MPSW":
            parts = a[3]
            return hull(*[self._cast(v[x], parse_ct(ct), (k, 1 + j), "%s: data input %d" % (tag, j + 1), record)
                          for j, (x, ct) in enumerate(parts)])
        if op == "LUT":
            self._table_point(k, a, a.out_type, "%s: table value -> output conversion" % tag, record)
            return (min(a.outs), max(a.outs))
        if op == "LUTL":
            self._table_point(k, a, a.intermediate, "%s: table value -> intermediate conversion" % tag, record)
            self._frac_guard(a, tag)
            # y = t0 + pr, pr = round(frac * D / 2^ff) with D the table step at the table's precision (in
            # intermediate units D / 2^dsh); the fraction saturates into [0, 2^ff], so pr lies between 0 and D rounded
            # outward, per segment; y is checked in the intermediate type, then cast
            its, itw, _itf = a.intermediate
            tms, tmw, _tmf = a.term_type
            sat, outc, dsh, slopes = 1 if a.saturate else 0, a.out_cast, a.dsh, a.slopes
            tv = list(a.tab)
            seg = []
            for k_, dv in enumerate(slopes[:len(tv) - 1]):
                lo_, hi_ = min(0, dv >> dsh), max(0, -((-dv) >> dsh))
                seg.append((tv[k_], lo_, hi_))
            plo, phi = min(x[1] for x in seg), max(x[2] for x in seg)
            pr = self._point((k, 1), "%s: interpolation step (signed term type)" % tag, plo, phi, tms, tmw, sat,
                             record)
            if pr == (plo, phi):
                y = (min(min(t + lo_ for t, lo_, _ in seg), tv[-1]), max(max(t + hi_ for t, _, hi_ in seg), tv[-1]))
            else:   # the step overflowed: only the sum bound holds
                y = (min(tv) + pr[0], max(tv) + pr[1])
            y = self._point((k, 2), "%s: interpolated value (intermediate type)" % tag, y[0], y[1], its, itw, sat,
                            record)
            return self._cast(y, parse_ct(outc), (k, 0), "%s: output conversion" % tag, record)
        if op == "LUT2":
            tab, bints = a.tab, a.bps
            if not a.linear:
                self._table_point(k, a, a.out_type, "%s: table value -> output conversion" % tag, record)
                return (min(tab), max(tab))
            self._table_point(k, a, a.intermediate, "%s: table value -> intermediate conversion" % tag, record)
            self._frac_guard(a, tag)
            (_ff, _fs, _fw, _rm, sat, its, itw, _ul, _itf, dsh), outc, tabT = a.core_fields()
            # dimension 1 first between table values (step from the table's precision, as LUTL), then dimension 2
            # between the two intermediate results; each stage checked in the intermediate type
            n2 = len(bints[1])
            rows = [tab[i:i + n2] for i in range(0, len(tab), n2)]
            rowsT = [tabT[i:i + n2] for i in range(0, len(tabT), n2)]
            seg = []
            for r in range(len(rows) - 1):
                for j in range(n2):
                    dv = rowsT[r + 1][j] - rowsT[r][j]
                    seg.append((rows[r][j], min(0, dv >> dsh), max(0, -((-dv) >> dsh))))
            plo, phi = min(x[1] for x in seg), max(x[2] for x in seg)
            pr = self._point((k, 1), "%s: interpolation step 1 (intermediate type)" % tag, plo, phi, its, itw, sat,
                             record)
            if pr == (plo, phi):
                y = (min(min(t + lo_ for t, lo_, _ in seg), min(tab)), max(max(t + hi_ for t, _, hi_ in seg), max(tab)))
            else:
                y = (min(tab) + pr[0], max(tab) + pr[1])
            y = self._point((k, 2), "%s: interpolated value 1 (intermediate type)" % tag, y[0], y[1], its, itw, sat,
                            record)
            lo, hi = y
            dif = (min(0, lo - hi), max(0, hi - lo))
            pr = self._point((k, 3), "%s: interpolation step 2 (intermediate type)" % tag, dif[0], dif[1], its, itw,
                             sat, record)
            if pr != dif:
                y = (y[0] + pr[0], y[1] + pr[1])
            y = self._point((k, 4), "%s: interpolated value 2 (intermediate type)" % tag, y[0], y[1], its, itw, sat,
                            record)
            return self._cast(y, parse_ct(outc), (k, 0), "%s: output conversion" % tag, record)
        if op == "PRODN":
            _, fo, rmm, _, s, w, sat, operands = a
            x0, fl0, _ = operands[0]
            acc, afl = v[x0], fl0
            if len(operands) == 1:
                sh = afl - fo
                y = (fdiv(acc[0], sh), cdiv(acc[1], sh)) if sh > 0 else (acc[0] << -sh, acc[1] << -sh)
                return self._point((k, 0), "%s: product" % tag, y[0], y[1], s, w, sat, record)
            for j in range(1, len(operands)):
                x, fl, _ = operands[j]
                p = imul(acc, v[x])
                sh = afl + fl - fo
                y = (fdiv(p[0], sh), cdiv(p[1], sh)) if sh > 0 else (p[0] << -sh, p[1] << -sh)
                acc = self._point((k, j), "%s: product step %d" % (tag, j), y[0], y[1], s, w, sat, record)
                afl = fo
            return acc
        raise Unsupported("no range rule for instruction %s of block %s" % (op, b))

    # ---- graph structure
    def _deps(self):
        dep = {}
        for k, (op, dst, srcs, a) in enumerate(self.ins):
            dep[dst] = list(srcs)
        for x, (s, _) in self.dly.items():
            dep[x] = [s]
        for x in self.in_ints:
            dep[x] = []
        return dep

    def _sccs(self, dep):
        users = {x: [] for x in dep}
        for x, ss in dep.items():
            for s in ss:
                users.setdefault(s, []).append(x)
                if s not in dep:
                    raise Unsupported("slot %s has no definition" % s)
        index, low, on, stack, out = {}, {}, set(), [], []
        cnt = [0]
        for v0 in dep:
            if v0 in index:
                continue
            index[v0] = low[v0] = cnt[0]
            cnt[0] += 1
            stack.append(v0)
            on.add(v0)
            work = [(v0, iter(users[v0]))]
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
                    index[nxt] = low[nxt] = cnt[0]
                    cnt[0] += 1
                    stack.append(nxt)
                    on.add(nxt)
                    work.append((nxt, iter(users[nxt])))
                elif nxt in on:
                    low[node] = min(low[node], index[nxt])
        return out[::-1]

    def storage_range(self, x, seen=()):
        """Full range of the type a delay state is STORED in: the compiled output type of the delay
        block, or for a hidden state the type of the signal it holds -- never the values its source can reach."""
        blk = x.split("\x1f")[0]
        if x == blk:
            t = self.d.otype[x]
            return (0, 1) if t == G.BOOL else trange(t[0], t[1])
        st = getattr(self.d, "slot_types", {}).get(x)
        if st is not None:   # a hidden slot with its own storage type (FIR taps, transposed partial sums in the
            return trange(st[0], st[1])   # accumulator type; recordings: the output type was used)
        src = self.dly[x][0] if x in self.dly else None
        if src is None or x in seen:
            t = self.d.otype[blk]
            return (0, 1) if t == G.BOOL else trange(t[0], t[1])
        if src.split("\x1f")[0] == src:
            t = self.d.otype[src]
            return (0, 1) if t == G.BOOL else trange(t[0], t[1])
        if src in self.dly:
            return self.storage_range(src, seen + (x,))
        t = self.d.otype[blk]  # a state fed by a hidden slot of its own block (counter, integrator)
        return (0, 1) if t == G.BOOL else trange(t[0], t[1])

    def _init_state(self, x):
        if self.initial == "any":
            return self.storage_range(x)
        ic = self.dly[x][1]
        return (ic, ic)

    # ---- main
    def _counters(self, dep):
        """Counter blocks are exact by construction (state in [0, limit]; the increment's cast is only taken
        below the limit, a free-running counter wraps by definition): their slots become sources with known
        ranges and their internal points are not quantisation points of the design."""
        known = {}
        for n in self.d.names:
            bt = self.d.btype[n]
            if bt not in ("CounterLimited", "CounterFreeRunning"):
                continue
            lim = G._counter_limit(self.d, n)
            st = (0, lim)
            if self.initial == "any":
                # any storable state, also above the limit (it then counts up to the type's wrap,
                # which a counter does by definition)
                t = self.d.otype[n]
                st = trange(t[0], t[1])
            known[n] = st
            for t, rng in (("one", (1, 1)), ("lim", (lim, lim)), ("zero", (0, 0)), ("eq", (0, 1)),
                           ("inc", (st[0] + 1, st[1] + 1)), ("nxt", st)):
                known[G._hid(n, t)] = rng
        for x, r in known.items():
            dep[x] = []
            self.val[x] = r
        return set(known)

    def run(self):
        dep = self._deps()
        fixed = self._counters(dep)
        mod_slots = {x for x in dep if self._blk(x) in self.mod_blocks}
        for x in mod_slots:   # the wrapping signals: anything in their type
            self.val[x] = self.slot_range(x)
        for e, (lo, hi) in self.mod_exits.items():
            import math
            f = self.d.otype[e][2]
            ilo, ihi = math.floor(lo * 2 ** f), math.ceil(hi * 2 ** f)
            tmin, tmax = self.slot_range(e)
            if not (tmin <= ilo and ihi <= tmax):
                self.status = "unknown"
                self.notes.append("modular region: the range leaving %s (%g .. %g) exceeds its type" % (e, lo, hi))
            else:
                self.val[e] = (ilo, ihi)
        if mod_slots:
            self.loops.append({"blocks": ", ".join(sorted(self.mod_blocks)), "method": "modular region: + / - and "
                               "delays only, wraps intended; the leaving values bounded by the impulse responses "
                               "from the input ranges plus the rounding error bound (engine/fxmodular.py)",
                               "proven": self.status == "bounded"})
        for comp in self._sccs(dep):
            if len(comp) == 1 and comp[0] in fixed:
                continue
            if all(x in mod_slots for x in comp):
                continue
            cyc = len(comp) > 1 or comp[0] in dep[comp[0]]
            if not cyc:
                x = comp[0]
                if x in self.in_ints:
                    self.val[x] = self.in_ints[x]
                elif x in self.dly:
                    self.val[x] = hull(self._init_state(x), self.val[self.dly[x][0]])
                else:
                    self.val[x] = self._eval(self.by_dst[x], self.val, True)
                continue
            members = set(comp)
            ks = sorted(self.by_dst[x] for x in comp if x in self.by_dst)
            states = sorted(x for x in comp if x in self.dly)
            ops = {self.ins[k][0] for k in ks}
            name = ", ".join(sorted({self._blk(x) for x in comp}))
            if ops <= LINEAR_OPS and len(states) <= MAX_STATES:
                region = self._region(members, dep)
                ok = False
                if region != members:  # the loop together with the linear signal path feeding it (cascades)
                    rk = sorted(self.by_dst[x] for x in region if x in self.by_dst)
                    rs = sorted(x for x in region if x in self.dly)
                    snap_val, snap_pts = dict(self.val), dict(self.points)
                    self.merge = True
                    ok, how = self._linear_loop(rk, rs, region)
                    self.merge = False
                    rset = set(rk)
                    if ok and any(not p_["fits"] for key, p_ in self.points.items() if key[0] in rset):
                        ok = False  # the linear model assumes no overflow anywhere in the region
                    if ok:
                        how = "joint linear region of %d signals: %s" % (len(region), how)
                    else:
                        self.val, self.points = snap_val, snap_pts
                if not ok:
                    ok, how = self._linear_loop(ks, states, members)
                    kset = set(ks)
                    inside = [p_ for key, p_ in self.points.items() if key[0] in kset and not p_["fits"]] if ok else []
                    if inside:  # the linear model assumes no overflow; keep its point bounds as diagnostics only
                        how += ("; but %d quantisation point(s) inside the loop exceed their type, so the linear "
                                "model (and the loop's invariant) is not valid" % len(inside))
                        self.loops.append({"blocks": name, "method": how, "proven": False})
                        self.status = "unknown"
                        self.notes.append("feedback loop through %s: %s" % (name, how))
                        for x in comp:
                            self.val[x] = self._fallback_range(x)
                        continue
                if ok:  # the interval invariant, when it exists, can only tighten (e.g. sign information)
                    keep = {x: self.val[x] for x in members}
                    self.merge = True
                    ok2, how2 = self._interval_loop(ks, states, members, merge_vals=keep)
                    self.merge = False
                    if ok2:
                        how += "; intersected with an " + how2
                    else:
                        self.val.update(keep)
            else:
                ok, how = self._interval_loop(ks, states, members)
            self.loops.append({"blocks": name, "method": how, "proven": ok})
            if not ok:
                self.status = "unknown"
                self.notes.append("feedback loop through %s: %s" % (name, how))
                for x in comp:  # whatever happens inside, every signal stays in its type range
                    self.val[x] = self._fallback_range(x)
                for k in ks:
                    self._eval(k, self.val, True)
        bad = [p for p in self.points.values() if not p["fits"]]
        if bad and self.status == "bounded":
            self.status = "unknown"
        return self

    def _lin_slot(self, x):
        if x in self.dly:
            return True
        k = self.by_dst.get(x)
        return k is not None and self.ins[k][0] in LINEAR_OPS

    def _region(self, members, dep):
        """The loop plus the linear signals upstream of it (already bounded on their own), as long as the
        region stays small and no external input of the region depends on a region member."""
        region = set(members)
        stack = [s for x in members for s in dep[x] if s not in region]
        while stack:
            x = stack.pop()
            if x in region or not self._lin_slot(x):
                continue
            if x in self.dly and sum(1 for y in region if y in self.dly) >= MAX_STATES:
                continue
            region.add(x)
            stack.extend(dep[x])
        ext = {s for x in region for s in dep[x] if s not in region}
        seen, stack = set(), list(ext)
        while stack:
            x = stack.pop()
            if x in seen:
                continue
            if x in region:
                return set(members)
            seen.add(x)
            stack.extend(dep[x])
        return region

    def _fallback_range(self, x):
        """slot_range, and for a delay state also its initial state (slot_range can be the source's value range,
        which need not contain the initial condition)."""
        v = self.slot_range(x)
        return hull(v, self._init_state(x)) if x in self.dly else v

    def _interval_loop(self, ks, states, members, merge_vals=None):
        S = {x: self._init_state(x) for x in states}
        widened = False
        for it in range(2 * ITER):
            v = dict(self.val)
            v.update(S)
            for k in ks:
                v[self.ins[k][1]] = self._eval(k, v, False)
            new = {x: hull(S[x], v[self.dly[x][0]]) for x in states}
            if new == S:
                v = dict(self.val)
                v.update(S)
                for k in ks:
                    v[self.ins[k][1]] = self._eval(k, v, True)
                for x in members:
                    self.val[x] = v[x]
                    if merge_vals is not None:
                        a, b = merge_vals[x]
                        self.val[x] = (max(a, v[x][0]), min(b, v[x][1]))
                return True, ("interval post-fixed point F(S) in S after %d iterations%s"
                              % (it + 1, " with widening to type ranges" if widened else ""))
            S = new
            if it == ITER - 1:  # widening: jump to the states' type ranges, then look for a post-fixed point again
                widened = True
                # hulled with the states reached so far (which include the initial state): slot_range is a value
                # range for some sources (a Saturation's limits, a table's values) and can exclude the initial
                # condition -- the box then passed F(S) in S without containing step 0 (
                # "bounded" with a wrap at step 0)
                S = {x: hull(S[x], self.slot_range(x)) for x in states}
        return False, "no interval post-fixed point within the iteration budget (not proven; no claim of instability)"

    # ---- linear loops: affine forms over states, external inputs, rounding errors and the constant 1
    def _linear_loop(self, ks, states, members):
        sidx = {x: i for i, x in enumerate(states)}
        m = len(states)
        syms = []  # (kind, interval) of the non-state symbols
        ext_sym = {}

        def sym(kind, iv):
            syms.append((kind, (Fraction(iv[0]), Fraction(iv[1]))))
            return m + len(syms) - 1
        one = sym("const", (1, 1))
        form = {}
        obs = {}  # observable name -> (form, check) ; check = (s, w, sh, key, desc) for points

        def get(x):
            if x in members:
                if x in sidx:
                    return {sidx[x]: Fraction(1)}
                return form[x]
            if x not in ext_sym:
                ext_sym[x] = sym("input " + x, self.val[x])
            return {ext_sym[x]: Fraction(1)}

        def scale(f, c):
            return {i: c * v for i, v in f.items() if c * v != 0}

        def add(f, g, sg=1):
            out = dict(f)
            for i, v in g.items():
                out[i] = out.get(i, 0) + sg * v
                if out[i] == 0:
                    del out[i]
            return out

        def cast(f, ct, key, desc):
            sh, rm, s, w, sat = ct
            obs[key] = (f, (s, w, sh, desc, sat, rm))
            if sh <= 0:
                return scale(f, Fraction(1 << -sh))
            g = scale(f, Fraction(1, 1 << sh))
            g[sym("rounding", err_range(rm))] = Fraction(1)
            return g

        for k in ks:
            op, dst, srcs, a = self.ins[k]
            tag = dst.replace("\x1f", "/")
            if op == "CONST":
                f = {one: Fraction(int(a))}
            elif op == "COPY":
                f = get(srcs[0])
            elif op == "CAST":
                f = cast(get(srcs[0]), parse_ct(a), (k, 0), "%s: conversion" % tag)
            elif op == "GAIN":
                g, ct = a.split(None, 1)
                f = cast(scale(get(srcs[0]), Fraction(int(g))), parse_ct(ct), (k, 0), "%s: gain product" % tag)
            elif op == "UN-neg":
                f = cast(scale(get(srcs[0]), Fraction(-1)), parse_ct(a.split(None, 1)[1]), (k, 0), "%s: neg" % tag)
            elif op == "SHIFT":
                left, kk, s, w = (int(t) for t in a.split())
                if left:
                    f = scale(get(srcs[0]), Fraction(1 << kk))
                    obs[(k, 0)] = (f, (s, w, 0, "%s: left shift" % tag, 0, 0))
                else:
                    f = scale(get(srcs[0]), Fraction(1, 1 << kk))
                    f[sym("rounding", (-1, 0))] = Fraction(1)
            elif op == "SUM":
                m_, first, acc_ct, out_ct, parts = a
                act = parse_ct(acc_ct)
                order = ([first] if first >= 0 else []) + [j for j in range(m_) if j != first]
                acc = {}
                for step, j in enumerate(order):
                    x, ct, sg = parts[j]
                    xv = cast(get(x), parse_ct(ct), (k, 1 + j), "%s: input %d into the accumulator" % (tag, j + 1))
                    if step == 0 and j == first:
                        acc = xv
                        continue
                    acc = add(acc, xv, sg)
                    obs[(k, 1000 + step)] = (acc, (act[2], act[3], 0, "%s: accumulator after input %d"
                                                   % (tag, j + 1), act[4], 0))
                f = cast(acc, parse_ct(out_ct), (k, 0), "%s: output conversion" % tag)
            else:
                return False, "instruction %s not linear" % op
            form[dst] = f
            obs[("slot", dst)] = (f, None)
        for x in states:
            obs[("slot", x)] = ({sidx[x]: Fraction(1)}, None)
        nxt = [get(self.dly[x][0]) for x in states]
        p = len(syms)
        A = [[nxt[i].get(j, Fraction(0)) for j in range(m)] for i in range(m)]
        B = [[nxt[i].get(m + q, Fraction(0)) for q in range(p)] for i in range(m)]
        lo_u = [iv[0] for _, iv in syms]
        hi_u = [iv[1] for _, iv in syms]
        s0 = [self._init_state(x) for x in states]
        s0 = [(Fraction(a_), Fraction(b_)) for a_, b_ in s0]

        def rng_row(row):  # row over symbols (length p)
            lo = hi = Fraction(0)
            for c, a_, b_ in zip(row, lo_u, hi_u):
                if c > 0:
                    lo, hi = lo + c * a_, hi + c * b_
                elif c < 0:
                    lo, hi = lo + c * b_, hi + c * a_
            return lo, hi

        def rng_state(row, box):  # row over states, box of states
            lo = hi = Fraction(0)
            for c, (a_, b_) in zip(row, box):
                if c > 0:
                    lo, hi = lo + c * a_, hi + c * b_
                elif c < 0:
                    lo, hi = lo + c * b_, hi + c * a_
            return lo, hi

        names = list(obs)
        Cs = {o: [obs[o][0].get(j, Fraction(0)) for j in range(m)] for o in names}
        Ds = {o: [obs[o][0].get(m + q, Fraction(0)) for q in range(p)] for o in names}
        Drng = {o: rng_row(Ds[o]) for o in names}
        # n = 0 .. K-1: v_n = C A^n s0 + sum_{j<n} C A^j B u + D u
        Mj = [row[:] for row in B]          # A^j B
        Pj = [[Fraction(int(i == j)) for j in range(m)] for i in range(m)]  # A^j
        pre = {o: (Fraction(0), Fraction(0)) for o in names}   # sum_{j<n} rng(C A^j B u)
        env = {o: None for o in names}
        Wst = [(Fraction(0), Fraction(0))] * m
        K = 0

        def matmul(X, Y):
            return [[sum((X[i][t] * Y[t][j] for t in range(len(Y)) if X[i][t]), Fraction(0))
                     for j in range(len(Y[0]))] for i in range(len(X))]

        # float pre-scan (a heuristic only, it picks K): first power of two with ||A^K|| clearly below 1/2
        Af = [[float(v) for v in row] for row in A]
        Pf = [[float(i == j) for j in range(m)] for i in range(m)]
        Kt, kf = None, 0
        while kf < K_MAX:
            Pf = [[sum(Af[i][t] * Pf[t][j] for t in range(m)) for j in range(m)] for i in range(m)]
            kf += 1
            nf = max((sum(abs(v) for v in row) for row in Pf), default=0.0)
            if not nf < 1e300:
                break
            if kf >= K_START and kf & (kf - 1) == 0 and nf <= 0.4:
                Kt = kf
                break
        if Kt is None:
            return False, ("linear loop: ||A^K|| does not fall below 1/2 within K = %d steps (not proven; the loop "
                           "is unstable, marginal or too slowly decaying for the step budget)" % K_MAX)
        while True:
            while K < Kt:
                for o in names:
                    C = Cs[o]
                    lo, hi = rng_state([sum((C[t] * Pj[t][j] for t in range(m) if C[t]), Fraction(0))
                                        for j in range(m)], s0)
                    v = (lo + pre[o][0] + Drng[o][0], hi + pre[o][1] + Drng[o][1])
                    env[o] = v if env[o] is None else (min(env[o][0], v[0]), max(env[o][1], v[1]))
                for o in names:
                    C = Cs[o]
                    if any(C):
                        r = rng_row([sum((C[t] * Mj[t][q] for t in range(m) if C[t]), Fraction(0)) for q in range(p)])
                        pre[o] = (pre[o][0] + r[0], pre[o][1] + r[1])
                Wst = [(Wst[i][0] + rng_row(Mj[i])[0], Wst[i][1] + rng_row(Mj[i])[1]) for i in range(m)]
                Mj = matmul(A, Mj)
                Pj = matmul(A, Pj)
                K += 1
            norm = max((sum(abs(v) for v in row) for row in Pj), default=Fraction(0))
            if norm <= Fraction(1, 2) or Kt >= K_MAX:
                break
            Kt *= 2
        if norm >= 1:
            return False, ("linear loop: ||A^K|| = %s >= 1 at K = %d (not proven; the loop is unstable, marginal "
                           "or too slowly decaying for the step budget)" % (fmt(norm), K))
        # invariant box S: contains s_0..s_{K-1} and A^K S + W in S
        E = [env[("slot", x)] for x in states]
        S = list(E)
        proven = False
        for it in range(60):
            img = [rng_state(Pj[i], S) for i in range(m)]
            img = [(img[i][0] + Wst[i][0], img[i][1] + Wst[i][1]) for i in range(m)]
            if all(S[i][0] <= img[i][0] and img[i][1] <= S[i][1] for i in range(m)):
                proven = True
                break
            S = [(min(S[i][0], img[i][0]), max(S[i][1], img[i][1])) for i in range(m)]
            if it >= 3:  # inflate a little so that a contraction lands strictly inside
                S = [(a_ - (b_ - a_) / 64 - 1, b_ + (b_ - a_) / 64 + 1) for a_, b_ in S]
        if not proven:
            return False, "linear loop: no inductive invariant box found (not proven)"
        # bounds of every observable at n >= K: C A^K s_{n-K} + sum_{j<K} C A^j B u + D u
        for o in names:
            C = Cs[o]
            CA = [sum((C[t] * Pj[t][j] for t in range(m) if C[t]), Fraction(0)) for j in range(m)]
            a_, b_ = rng_state(CA, S)
            v = (a_ + pre[o][0] + Drng[o][0], b_ + pre[o][1] + Drng[o][1])
            env[o] = (min(env[o][0], v[0]), max(env[o][1], v[1]))
        for o in names:
            lo, hi = env[o]
            if o[0] == "slot":
                v = (fdiv(lo, 0), cdiv(hi, 0))
                if self.merge and o[1] in self.val:
                    v = (max(v[0], self.val[o[1]][0]), min(v[1], self.val[o[1]][1]))
                self.val[o[1]] = v
                continue
            s, w, sh, desc, sat, rm = obs[o][1]
            self._point(o, desc, rnd(lo, sh, rm), rnd(hi, sh, rm), s, w, sat, True)
        return True, "linear K-step invariant (K = %d, ||A^K|| = %s, %d states, %d symbols)" % (K, fmt(norm), m, p)


def fingerprint(spec, in_ints, contract):
    blob = json.dumps({"design": spec, "inputs": {k: list(v) for k, v in sorted(in_ints.items())},
                       "contract": contract}, sort_keys=True)
    return hashlib.sha256(blob.encode()).hexdigest()


def prove(spec, hw, in_types, in_ints, initial="design", witness=True, modular=None):
    """Overflow proof of a design given as a spec with explicit types. Returns a JSON-able report:
    status bounded | violated (a concrete counterexample sequence was found, see witness) | unknown |
    unsupported, the method, every quantisation point with its bound, and the reasons."""
    contract = {"inputs": {x: {"type": "fixdt(%d,%d,%d)" % in_types[x], "stored_min": in_ints[x][0],
                               "stored_max": in_ints[x][1]} for x in sorted(in_ints)},
                "initial_state": initial, "horizon": "unbounded",
                "overflow_definition": "any quantisation point (cast after rounding, accumulator partial sum, product "
                                       "step, left shift) outside its type, wrapping or saturating",
                "explicit_saturation_blocks": "value operation, not an overflow",
                "lookup_table_fraction": "the interpolation fraction always saturates by the block's definition "
                                         "(it reaches 1 at the last breakpoint); not an overflow",
                # what the certificate does not cover, stated
                "parameters": "constants, gains, coefficients, table values, limits and initial values are quantised "
                              "to their types once, when the design is compiled (nearest, saturating, as Simulink "
                              "does); the certificate is about the design so compiled -- a parameter that "
                              "saturated there is not reported as an overflow",
                "counters": "a counter wraps or resets by definition; not an overflow"}
    rep = {"contract": contract, "fingerprint": fingerprint(spec, in_ints, contract)}
    try:
        d = G.Design(spec, hw)
        d.compile(in_types)
        mod = None
        if modular:
            import fxmodular
            ex = fxmodular.exit_ranges(modular, spec["blocks"])
            if ex is None:
                raise Unsupported("modular region: the rounding error bound of its fraction lengths does not exist")
            mod = (sorted(b for r in modular for b in r["blocks"]), ex)
        pr = Prover(d, in_ints, initial, mod).run()
    except Unsupported as e:
        rep.update(status="unsupported", reason=str(e))
        return rep
    except G.GraphError as e:
        rep.update(status="unsupported", reason="%s: %s" % (e.code, e.message))
        return rep
    bad = [dict(p) for p in pr.points.values() if not p["fits"]]
    rep.update(status=pr.status, loops=pr.loops, points_checked=len(pr.points), points_not_proven=bad[:50],
               reasons=pr.notes + ["%d quantisation point(s) whose bound exceeds the type" % len(bad)] if bad
               else pr.notes)
    rep["slot_bounds"] = {x.replace("\x1f", "/"): list(v) for x, v in sorted(pr.val.items())}
    if rep["status"] == "unknown" and witness:
        # initial "any": the design's own initial state is one of the allowed states, so its witnesses count
        # too; then other storable initial states are tried (find_witness_any)
        w = find_witness(d, in_ints)
        if w is not None and initial == "any":
            w["initial_states"] = "the design's own initial state"
        if w is None and initial == "any":
            w = find_witness_any(d, in_ints, pr)
        if w is not None:
            # a concrete input sequence inside the ranges, simulated bit-exactly, overflows: a counterexample
            rep["status"] = "violated"
            rep["witness"] = w
    return rep


def find_witness(d, in_ints, max_runs=40, seed=0):
    """Search for an input sequence inside the ranges that makes a quantisation point leave its type, by running
    an instrumented copy of the design's integer program (engine/fxcodegen.py, the same instructions as the C
    core) on extreme, alternating, stepped and random sequences. Returns None or {inputs, step, block}."""
    import os
    import random
    import subprocess
    import tempfile
    import fxcodegen as CG
    if not d.vm_ok:
        return None
    names = []
    try:
        src = CG.generate(d, list(d.outports) or [d.names[0]], names, allow_strict=True)
    except CG.CodegenError:
        return None
    root = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "runs", "witness")
    os.makedirs(root, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=root) as tmp:
        c, exe = os.path.join(tmp, "w.c"), os.path.join(tmp, "w")
        with open(c, "w") as f:
            f.write(src)
        if subprocess.run(["clang", "-O1", "-std=c99", "-w", "-o", exe, c], capture_output=True).returncode:
            return None
        L = min(512, 64 + 8 * sum(len(v) for v in d.vm_dly.values()))
        rng = random.Random(seed)
        ins = list(d.inports)
        cands = []
        for pick in ("lo", "hi"):
            cands.append({x: [in_ints[x][0 if pick == "lo" else 1]] * L for x in ins})
        for per in (1, 2, 3, 4, 8, 16):
            cands.append({x: [in_ints[x][(k // per) % 2] for k in range(L)] for x in ins})
            cands.append({x: [in_ints[x][1 - (k // per) % 2] for k in range(L)] for x in ins})
        cands.append({x: [in_ints[x][0]] * (L // 2) + [in_ints[x][1]] * (L - L // 2) for x in ins})
        cands.append({x: [in_ints[x][1]] * (L // 2) + [in_ints[x][0]] * (L - L // 2) for x in ins})
        while len(cands) < max_runs:
            if len(cands) % 2:
                cands.append({x: [in_ints[x][rng.getrandbits(1)] for _ in range(L)] for x in ins})
            else:
                cands.append({x: [rng.randint(*in_ints[x]) for _ in range(L)] for x in ins})
        for cand in cands[:max_runs]:
            text = "%d\n" % L + "\n".join(" ".join(map(str, cand[x])) for x in ins) + "\n"
            p = subprocess.run([exe], input=text.encode(), capture_output=True, timeout=60)
            for ln in p.stdout.decode().splitlines():
                if ln.startswith("OVERFLOW "):
                    _, st, pt = ln.split()
                    st = int(st)
                    return {"inputs": {x: cand[x][:st + 1] for x in ins}, "step": st,
                            "block": names[int(pt)].replace("\x1f", "/") if int(pt) >= 0 else None,
                            "note": "bit-exact simulation of this design's integer program; reproduce with fxgraph"}
    return None


def _witness_inputs(in_ints, ins, L, rng, max_runs):
    """Extreme, alternating, stepped and random input sequences of length L inside the ranges."""
    cands = []
    for pick in (0, 1):
        cands.append({x: [in_ints[x][pick]] * L for x in ins})
    for per in (1, 2, 3, 4, 8, 16):
        cands.append({x: [in_ints[x][(k // per) % 2] for k in range(L)] for x in ins})
        cands.append({x: [in_ints[x][1 - (k // per) % 2] for k in range(L)] for x in ins})
    cands.append({x: [in_ints[x][0]] * (L // 2) + [in_ints[x][1]] * (L - L // 2) for x in ins})
    cands.append({x: [in_ints[x][1]] * (L // 2) + [in_ints[x][0]] * (L - L // 2) for x in ins})
    cands.append({x: [0] * L if in_ints[x][0] <= 0 <= in_ints[x][1] else [in_ints[x][0]] * L for x in ins})
    while len(cands) < max_runs:
        if len(cands) % 2:
            cands.append({x: [in_ints[x][rng.getrandbits(1)] for _ in range(L)] for x in ins})
        else:
            cands.append({x: [rng.randint(*in_ints[x]) for _ in range(L)] for x in ins})
    return cands[:max_runs]


def find_witness_any(d, in_ints, pr, n_states=12, n_inputs=24, seed=0):
    """Counterexample search for initial state "any": start from storable initial states (every delay at the
    bottom / top / zero of its storage range, alternating, random) and drive the inputs with the sequences of
    find_witness, on the batch form of the instrumented integer program (engine/fxcodegen.py). Any storable
    state is allowed by the contract, so an overflow found this way is a counterexample.
    Returns None or {inputs, initial_states, step, block}."""
    import os
    import random
    import subprocess
    import tempfile
    import fxcodegen as CG
    if not d.vm_ok:
        return None
    dly = [x for n in d.names for (x, _s, _ic) in d.vm_dly.get(n, ())]
    if not dly:
        return None
    names = []
    try:
        src = CG.generate(d, list(d.outports) or [d.names[0]], names, batch=True, allow_strict=True)
    except CG.CodegenError:
        return None
    rng = random.Random(seed)
    # every storable state (storage_range): slot_range is the source's value range for a delay behind a Saturation /
    # table / constant, so states outside it were never tried (unknown instead of violated)
    box = [pr.storage_range(x) for x in dly]
    states = [[a for a, b in box], [b for a, b in box], [min(max(0, a), b) for a, b in box],
              [(a if k % 2 else b) for k, (a, b) in enumerate(box)], [(b if k % 2 else a) for k, (a, b) in enumerate(box)]]
    while len(states) < n_states:
        states.append([rng.choice((a, b)) if len(states) % 2 else rng.randint(a, b) for a, b in box])
    ins = list(d.inports)
    seqs = _witness_inputs(in_ints, ins, 64, rng, n_inputs) if ins else [{}]
    cases = [(st, q) for st in states for q in seqs]
    root = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "runs", "witness")
    os.makedirs(root, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=root) as tmp:
        c, exe = os.path.join(tmp, "w.c"), os.path.join(tmp, "w")
        with open(c, "w") as f:
            f.write(src)
        if subprocess.run(["clang", "-O1", "-std=c99", "-w", "-o", exe, c], capture_output=True).returncode:
            return None
        parts = ["%d" % len(cases)]
        for st, q in cases:
            parts.append(" ".join(map(str, st)) + " %d " % (64 if ins else 1) + " ".join(" ".join(map(str, q[x])) for x in ins))
        p = subprocess.run([exe], input=("\n".join(parts) + "\n").encode(), capture_output=True, timeout=120)
        for ln in p.stdout.decode().splitlines():
            f = ln.split()
            if len(f) >= 5 and f[2] == "OVERFLOW":
                st_, q = cases[int(f[1])]
                step, pt = int(f[3]), int(f[4])
                return {"inputs": {x: q[x][:step + 1] for x in ins},
                        "initial_states": {x.replace("\x1f", "/"): v for x, v in zip(dly, st_)}, "step": step,
                        "block": names[pt].replace("\x1f", "/") if pt >= 0 else None,
                        "note": "bit-exact simulation of this design's integer program from the given storable "
                                "initial delay states (allowed by initial_state any)"}
    return None

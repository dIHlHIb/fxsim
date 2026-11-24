"""fixed-point C code generation.

A compiled design (types resolved by the engine) becomes one self-contained C file: integer arithmetic only
(no floating point, no library beyond stdio/stdlib), one function `step` that executes the same integer
instructions as the C core (engine/fxgraph_core.c), and a `main` that reads input samples and prints the
recorded outputs. Values are __int128, as in the core (clang and gcc support it as an extension of C99).

I/O of the generated program (text, whitespace separated):
  stdin : N, then for every Inport in the order of `inports` N stored integers
  stdout: for every recorded output, one line "name v0 v1 ... v(N-1)"
Unsupported designs (the engine's Python path only, i.e. vm_ok false) are refused with UNSUPPORTED."""
import fxgraph_lib as G

RUNTIME = r"""
#include <stdio.h>
#include <stdlib.h>
typedef __int128 i128;
typedef unsigned __int128 u128;

static long STEP = 0, OV_STEP = -1;
static int PT = -1, OV_PT = -1;   /* first quantisation point outside its type (instrumented builds) */

static i128 ovf(i128 n, int s, int wl, int sat) {
    i128 lo = s ? -((i128)1 << (wl - 1)) : 0;
    i128 hi = s ? ((i128)1 << (wl - 1)) - 1 : (i128)(((u128)1 << wl) - 1);
    if (n >= lo && n <= hi) return n;
    if (OV_STEP < 0 && PT != -2) { OV_STEP = STEP; OV_PT = PT; }   /* -2: a counter, wraps by definition */
    if (sat) return n < lo ? lo : hi;
    u128 m = (u128)n & (((u128)1 << wl) - 1);
    if (s && (m >> (wl - 1))) return (i128)m - ((i128)1 << wl);
    return (i128)m;
}

/* interpolation fraction of a linear lookup table: always saturates by the block's definition (it reaches 1 at
   the last breakpoint, which the fraction type cannot hold) -- clamped, not reported as an overflow */
static i128 fsat(i128 n, int s, int wl) {
    i128 lo = s ? -((i128)1 << (wl - 1)) : 0;
    i128 hi = s ? ((i128)1 << (wl - 1)) - 1 : (i128)(((u128)1 << wl) - 1);
    return n < lo ? lo : (n > hi ? hi : n);
}

/* rounding modes: 0 floor 1 ceiling 2 zero 3 nearest 4 round 5 convergent */
static i128 cst(i128 n, int sh, int rm, int s, int wl, int sat) {
    if (sh <= 0) return ovf((i128)((u128)n << (-sh)), s, wl, sat);
    i128 q = n >> sh;
    u128 r = (u128)n & (((u128)1 << sh) - 1);
    if (r != 0) {
        u128 half = (u128)1 << (sh - 1);
        switch (rm) {
        case 0: break;
        case 1: q += 1; break;
        case 2: if (n < 0) q += 1; break;
        case 3: if (r >= half) q += 1; break;
        default:
            if (r > half) q += 1;
            else if (r == half) { if (rm == 4) { if (n > 0) q += 1; } else q += (q & 1); }
        }
    }
    return ovf(q, s, wl, sat);
}

static i128 rdiv(i128 num, i128 den, int rm) {
    if (den < 0) { num = -num; den = -den; }
    i128 q = num / den, r = num % den;
    if (r < 0) { q -= 1; r += den; }
    if (r == 0) return q;
    switch (rm) {
    case 0: return q;
    case 1: return q + 1;
    case 2: return num < 0 ? q + 1 : q;
    default: {
        i128 c = 2 * r - den;
        if (c > 0) return q + 1;
        if (c < 0) return q;
        if (rm == 3) return q + 1;
        if (rm == 4) return num > 0 ? q + 1 : q;
        return q + (q & 1);
    }
    }
}

static i128 sdiv(i128 num, int e, i128 den, int rm) {
    if (e >= 0) return rdiv(num * ((i128)1 << e), den, rm);
    return rdiv(num, den * ((i128)1 << (-e)), rm);
}

static i128 rdint(void) {
    char t[64];
    if (scanf("%63s", t) != 1) { fprintf(stderr, "input truncated\n"); exit(2); }
    char *s = t; int neg = 0; u128 v = 0;
    if (*s == '-') { neg = 1; s++; }
    if (*s < '0' || *s > '9') { fprintf(stderr, "bad integer %s\n", t); exit(2); }
    while (*s >= '0' && *s <= '9') v = v * 10 + (unsigned)(*s++ - '0');
    if (*s) { fprintf(stderr, "bad integer %s\n", t); exit(2); }
    return neg ? -(i128)v : (i128)v;
}

static void pr(i128 v) {
    char t[48]; int n = 0, neg = v < 0;
    u128 u = neg ? -(u128)v : (u128)v;
    if (!u) t[n++] = '0';
    while (u) { t[n++] = (char)('0' + (int)(u % 10)); u /= 10; }
    if (neg) putchar('-');
    while (n) putchar(t[--n]);
}
"""


class CodegenError(Exception):
    def __init__(self, code, where, message):
        Exception.__init__(self, message)
        self.code, self.where, self.message = code, where, message


def _ct(tok):
    sh, rm, s, w, sat = (int(v) for v in tok.split())
    return "%d, %d, %d, %d, %d" % (sh, rm, s, w, sat)


def _lit(v):
    v = int(v)
    if -(1 << 62) < v < (1 << 62):
        return "((i128)%dLL)" % v
    # no negative literal or left shift of a negative value -- the 128-bit two's-complement
    # pattern from two unsigned halves (the conversion to i128 wraps on clang / gcc)
    m = v & ((1 << 128) - 1)
    return "((i128)(((u128)0x%xULL << 64) | (u128)0x%xULL))" % (m >> 64, m & ((1 << 64) - 1))


def _emit(op, sl, rest, S):
    """C statements for one instruction (same semantics as exec_int / op_new of fxgraph_core.c)."""
    d = S(sl[0])
    if op == "CONST":
        return ["%s = %s;" % (d, _lit(rest))]
    if op == "COPY":
        return ["%s = %s;" % (d, S(sl[1]))]
    if op == "GAIN":
        g, ct = rest.split(None, 1)
        return ["%s = cst(%s * %s, %s);" % (d, S(sl[1]), _lit(g), _ct(ct))]
    if op == "PROD":
        return ["%s = cst(%s * %s, %s);" % (d, S(sl[1]), S(sl[2]), _ct(rest))]
    if op == "CAST":
        return ["%s = cst(%s, %s);" % (d, S(sl[1]), _ct(rest))]
    if op == "SAT":
        lo, hi = rest.split()
        return ["{ i128 v = %s; if (v < %s) v = %s; if (v > %s) v = %s; %s = v; }"
                % (S(sl[1]), _lit(lo), _lit(lo), _lit(hi), _lit(hi), d)]
    if op == "UN":
        neg, ct = rest.split(None, 1)
        a = S(sl[1])
        expr = "-%s" % a if neg == "1" else "(%s < 0 ? -%s : %s)" % (a, a, a)
        return ["%s = cst(%s, %s);" % (d, expr, _ct(ct))]
    if op == "REL":
        sa, sb, code = (int(v) for v in rest.split())
        o = ["==", "!=", "<", "<=", ">", ">="][code]
        return ["%s = (%s * ((i128)1 << %d)) %s (%s * ((i128)1 << %d));" % (d, S(sl[1]), sa, o, S(sl[2]), sb)]
    if op == "SUM":
        m, first, acc_ct, out_ct, parts = rest
        lines = ["{ i128 acc = 0, xv;"]
        if first >= 0:
            x, ct, _ = parts[first]
            lines.append("acc = cst(%s, %s);" % (S(x), _ct(ct)))
        a_sh, a_rm, a_s, a_w, a_sat = (int(v) for v in acc_ct.split())
        for j, (x, ct, sg) in enumerate(parts):
            if j == first:
                continue
            lines.append("xv = cst(%s, %s); acc = ovf(acc %s xv, %d, %d, %d);"
                         % (S(x), _ct(ct), "+" if sg > 0 else "-", a_s, a_w, a_sat))
        lines.append("%s = cst(acc, %s); }" % (d, _ct(out_ct)))
        return lines
    if op == "MINMAX":
        m, ismax, out_ct, parts = rest
        lines = ["{ i128 best = cst(%s, %s), xv;" % (S(parts[0][0]), _ct(parts[0][1]))]
        for x, ct in parts[1:]:
            lines.append("xv = cst(%s, %s); if (xv %s best) best = xv;" % (S(x), _ct(ct), ">" if ismax else "<"))
        lines.append("%s = cst(best, %s); }" % (d, _ct(out_ct)))
        return lines
    if op == "SHIFT":
        left, k, s, w = (int(v) for v in rest.split())
        a = S(sl[1])
        if left:
            return ["%s = ovf(%s, %d, %d, 0);" % (d, "0" if k >= 127 else "(i128)((u128)%s << %d)" % (a, k), s, w)]
        return ["%s = %s;" % (d, ("(%s < 0 ? -1 : 0)" % a) if k >= 127 else "%s >> %d" % (a, k))]
    if op == "LOGIC":
        code, srcs = rest
        ins = ["(%s != 0)" % S(x) for x in srcs]
        if code == 6:
            return ["%s = %s == 0;" % (d, S(srcs[0]))]
        alls = " && ".join(ins)
        anys = " || ".join(ins)
        cnt = " + ".join(ins)
        expr = {0: alls, 1: anys, 2: "!(%s)" % alls, 3: "!(%s)" % anys, 4: "(%s) & 1" % cnt,
                5: "!((%s) & 1)" % cnt}[code]
        return ["%s = %s;" % (d, expr)]
    if op == "SWITCH":
        t = rest.split()
        mode, q = int(t[0]), t[1]
        c1, c3 = " ".join(t[2:7]), " ".join(t[7:12])
        cond = {0: "%s != 0" % S(sl[2]), 1: "%s >= %s" % (S(sl[2]), _lit(q)), 2: "%s > %s" % (S(sl[2]), _lit(q))}[mode]
        return ["%s = (%s) ? cst(%s, %s) : cst(%s, %s);" % (d, cond, S(sl[1]), _ct(c1), S(sl[3]), _ct(c3))]
    if op == "MPSW":
        cfl, base, ncontig, parts = rest
        c = S(sl[1])
        nd = len(parts)
        lines = ["{ i128 c = %s, k; int integral = 1, idx = %d;" % (c, nd - 1)]
        if cfl <= 0:
            lines.append("k = c * ((i128)1 << %d);" % (-cfl))
        else:
            lines.append("integral = (c & (((i128)1 << %d) - 1)) == 0; k = c >> %d;" % (cfl, cfl))
        lines.append("if (integral) { i128 i = k - %d; if (i >= 0 && i < %d) idx = (int)i; }" % (base, ncontig))
        lines.append("switch (idx) {")
        for j, (x, ct) in enumerate(parts):
            lines.append("case %d: %s = cst(%s, %s); break;" % (j, d, S(x), _ct(ct)))
        lines.append("} }")
        return lines
    if op == "BITW":
        code, s, w, um, mask, srcs = rest
        mm = "(((i128)1 << %d) - 1)" % w
        lines = ["{ i128 mm = %s, acc = %s & mm, v;" % (mm, S(srcs[0]))]
        opch = {0: "&", 2: "&", 1: "|", 3: "|", 4: "^"}.get(code)
        for x in srcs[1:]:
            if opch:
                lines.append("v = %s & mm; acc = acc %s v;" % (S(x), opch))
        if um and opch:
            lines.append("v = %s & mm; acc = acc %s v;" % (_lit(mask), opch))
        if code in (2, 3, 5):
            lines.append("acc = ~acc & mm;")
        if s:
            lines.append("if ((acc >> %d) & 1) acc -= (i128)1 << %d;" % (w - 1, w))
        lines.append("%s = acc; }" % d)
        return lines
    if op == "EXTR":
        lo, nb, s = (int(v) for v in rest.split())
        lines = ["{ i128 v = (%s >> %d) & (((i128)1 << %d) - 1);" % (S(sl[1]), lo, nb)]
        if s:
            lines.append("if ((v >> %d) & 1) v -= (i128)1 << %d;" % (nb - 1, nb))
        lines.append("%s = v; }" % d)
        return lines
    if op == "LUT":
        r = rest
        mode, sa, sb, last = r.mode, r.sa, r.sb, len(r.bps) - 1
        bp = "{%s}" % ", ".join(_lit(b) for b in r.bps)
        ov = "{%s}" % ", ".join(_lit(o) for o in r.outs)
        return ["{ static const i128 bp[] = %s, ov[] = %s;" % (bp, ov),
                "i128 u = %s * ((i128)1 << %d), b0 = bp[0] * ((i128)1 << %d), bl = bp[%d] * ((i128)1 << %d);"
                % (S(sl[1]), sa, sb, last, sb),
                "if (u < b0) u = b0; if (u > bl) u = bl; int k = 0;",
                "while (k < %d && u >= bp[k + 1] * ((i128)1 << %d)) k++;" % (last - 1, sb),
                "i128 bk = bp[k] * ((i128)1 << %d), bk1 = bp[k + 1] * ((i128)1 << %d);" % (sb, sb),
                "int up = %s; %s = ov[k + up]; }" % ("u >= bk1" if mode == 0 else "(u - bk) >= (bk1 - u)", d)]
    if op == "LUTL":
        r = rest
        if getattr(r, "frac_round1_stop", False):   # the engine stops at that event
            raise CodegenError("UNSUPPORTED", "design.blocks." + r.block, "1-D lookup with UseLastTableValue on whose fraction "
                     "may round to 1 where Simulink's behaviour is not recorded: no generated code")
        if r.magnitude_slope and any(v < 0 for v in r.slopes):   # not implemented in generated code
            raise CodegenError("UNSUPPORTED", "design.blocks." + r.block, "1-D lookup with a 32-bit table type and a "
                               "falling segment on this hardware (magnitude slope term): no C generation")
        sa, sb, dsh, rm, sat, ulast = r.sa, r.sb, r.dsh, r.rm, 1 if r.saturate else 0, 1 if r.use_last else 0
        fs, fw, ff = r.frac_type
        its, itw, _itf = r.intermediate
        tms, tmw, _tmf = r.term_type
        last = len(r.bps) - 1
        bp = "{%s}" % ", ".join(_lit(b) for b in r.bps)
        tv = "{%s}" % ", ".join(_lit(t) for t in r.tab)
        sl_ = "{%s}" % ", ".join(_lit(v) for v in r.slopes)
        return ["{ static const i128 bp[] = %s, tv[] = %s, sl[] = %s;" % (bp, tv, sl_),
                "i128 u = %s * ((i128)1 << %d), b0 = bp[0] * ((i128)1 << %d), bl = bp[%d] * ((i128)1 << %d), y;"
                % (S(sl[1]), sa, sb, last, sb),
                "if (u < b0) u = b0; if (u > bl) u = bl;",
                "if (%d && u >= bl) y = tv[%d]; else {" % (ulast, last),
                "  int k = 0; while (k < %d && u >= bp[k + 1] * ((i128)1 << %d)) k++;" % (last - 1, sb),
                "  i128 bk = bp[k] * ((i128)1 << %d), bk1 = bp[k + 1] * ((i128)1 << %d);" % (sb, sb),
                ("  i128 fr = rdiv((u - bk) * ((i128)1 << %d), bk1 - bk, %d), "
                 "fq = fr >= ((i128)1 << %d) ? ((i128)1 << %d) : fsat(fr, %d, %d);" % (ff, rm, ff, ff, fs, fw))
                if getattr(r, "frac_carry", False) else   # a fraction rounding to 1 is 1
                "  i128 fq = fsat(rdiv((u - bk) * ((i128)1 << %d), bk1 - bk, %d), %d, %d);" % (ff, rm, fs, fw),
                # the slope term in the record's term type (signed; fxlutplan.LutLinear.term_type)
                "  i128 pr = ovf(rdiv(fq * sl[k], (i128)1 << %d, %d), %d, %d, %d);" % (ff + dsh, rm, tms, tmw, sat),
                "  y = ovf(tv[k] + pr, %d, %d, %d); }" % (its, itw, sat),
                "%s = cst(y, %s); }" % (d, _ct(r.out_cast))]
    if op == "LUT2":
        r = rest
        mode, sh, bints, tab = r.mode, r.shifts, r.bps, r.tab
        (ff, fs, fw, rm, sat, its, itw, ulast, _itf, dsh), outc, tabT = r.core_fields()
        n1, n2 = len(bints[0]), len(bints[1])
        L = ["{ static const i128 b1[] = {%s}, b2[] = {%s}, tb[] = {%s}, tT[] = {%s};"
             % (", ".join(_lit(v) for v in bints[0]), ", ".join(_lit(v) for v in bints[1]),
                ", ".join(_lit(v) for v in tab), ", ".join(_lit(v) for v in tabT)),
             "const i128 *bp[2] = {b1, b2}; int nn[2] = {%d, %d}, sa[2] = {%d, %d}, sb[2] = {%d, %d}, k[2], nx[2];"
             % (n1, n2, sh[0], sh[2], sh[1], sh[3]),
             "i128 in2[2] = {%s, %s}, f[2], yy[2];" % (S(sl[1]), S(sl[2])),
             "for (int q = 0; q < 2; q++) { int last = nn[q] - 1, kk = 0;",
             "  i128 u = in2[q] * ((i128)1 << sa[q]), b0 = bp[q][0] * ((i128)1 << sb[q]), bl = bp[q][last] * ((i128)1 << sb[q]);",
             "  if (u < b0) u = b0; if (u > bl) u = bl;",
             "  while (kk < last - 1 && u >= bp[q][kk + 1] * ((i128)1 << sb[q])) kk++;",
             "  i128 bk = bp[q][kk] * ((i128)1 << sb[q]), bk1 = bp[q][kk + 1] * ((i128)1 << sb[q]);"]
        if mode < 2:
            up = "u >= bk1" if mode == 0 else "(u - bk) >= (bk1 - u)"
            L += ["  k[q] = kk + ((%s) ? 1 : 0); f[q] = 0; nx[q] = 0; }" % up,
                  "(void)yy; %s = tb[(size_t)k[0] * %d + k[1]]; }" % (d, n2)]
            return L
        L += ["  if (%d && u >= bl) { kk = last; f[q] = 0; }" % ulast,
              "  else f[q] = fsat(rdiv((u - bk) * ((i128)1 << %d), bk1 - bk, %d), %d, %d);" % (ff, rm, fs, fw),
              "  k[q] = kk; nx[q] = kk + 1 < last ? kk + 1 : last; }",
              "for (int c = 0; c < 2; c++) { int col = c ? nx[1] : k[1];",
              "  i128 y0 = tb[(size_t)k[0] * %d + col], dT = tT[(size_t)nx[0] * %d + col] - tT[(size_t)k[0] * %d + col];"
              % (n2, n2, n2),
              "  yy[c] = ovf(y0 + ovf(rdiv(f[0] * dT, (i128)1 << %d, %d), %d, %d, %d), %d, %d, %d); }"
              % (ff + dsh, rm, its, itw, sat, its, itw, sat),
              "%s = cst(ovf(yy[0] + ovf(rdiv(f[1] * (yy[1] - yy[0]), (i128)1 << %d, %d), %d, %d, %d), %d, %d, %d), %s); }"
              % (d, ff, rm, its, itw, sat, its, itw, sat, _ct(outc))]
        return L
    if op == "PRODN":
        special, fo, rmm, rmd, s, w, sat, operands = rest
        lo = "(%s ? -((i128)1 << (%d - 1)) : 0)" % (s, w)
        hi = "(%s ? ((i128)1 << (%d - 1)) - 1 : (i128)(((u128)1 << %d) - 1))" % (s, w, w)
        x = [S(o[0]) for o in operands]
        fl = [o[1] for o in operands]
        dv = [o[2] for o in operands]
        L = ["do { i128 acc; int afl;"]
        if special:
            L.append("if (%s == 0) { %s = %s < 0 ? %s : %s; break; }" % (x[0], d, x[1], lo, hi))
            L.append("%s = ovf(sdiv(%s, %d, %s, %d), %d, %d, %d); break;"
                     % (d, x[1], fl[0] + fo - fl[1], x[0], rmd, s, w, sat))
        else:
            if not dv[0]:
                L.append("acc = %s; afl = %d;" % (x[0], fl[0]))
                if len(operands) == 1:
                    sh = fl[0] - fo
                    L.append("%s = ovf(rdiv(acc, %s, %d) * %s, %d, %d, %d); break;"
                             % (d, _lit(1 << sh) if sh > 0 else "1", rmm, _lit(1 << (-sh)) if sh < 0 else "1", s, w, sat))
            else:
                # a zero divisor gives that step the output type's extreme and the chain goes on
                L.append("if (%s == 0) acc = %s; else acc = ovf(sdiv(1, %d, %s, %d), %d, %d, %d); afl = %d;"
                         % (x[0], hi, fl[0] + fo, x[0], rmd, s, w, sat, fo))
            for j in range(1, len(operands)):
                if not dv[j]:
                    L.append("{ i128 v = acc * %s; int sh = afl + %d - %d;" % (x[j], fl[j], fo))
                    L.append("acc = ovf(sh > 0 ? rdiv(v, (i128)1 << sh, %d) : v * ((i128)1 << (-sh)), %d, %d, %d); }"
                             % (rmm, s, w, sat))
                else:
                    L.append("if (%s == 0) acc = acc < 0 ? %s : %s; else acc = ovf(sdiv(acc, %d + %d - afl, %s, %d), %d, %d, %d);"
                             % (x[j], lo, hi, fl[j], fo, x[j], rmd, s, w, sat))
                L.append("afl = %d;" % fo)
            L.append("%s = acc;" % d)
        L.append("(void)afl; } while (0);")
        return L
    raise CodegenError("UNSUPPORTED", "design", "no C generation for instruction %s" % op)


def label(name):
    """Printed name of an output: printable, no whitespace (internal names carry control characters)."""
    return "".join(c if c.isprintable() and not c.isspace() and c not in '"\\' else "~" for c in name)


def _refuse_strict(d, exc):
    """recordings (an engine refusal previously accepted by code generation): generated code cannot
    stop a run, so a design whose Multiport Switch stops Simulink on an out-of-range control -- which the engine
    stops on too -- is refused, in range or not."""
    rd = getattr(d, "runtime_diag", None)
    if rd:
        raise exc("UNSUPPORTED", "design.blocks.%s.DiagnosticForDefault" % rd[0],
                  "Multiport Switch %s stops the simulation on an out-of-range control (DiagnosticForDefault 'Error' "
                  "-- the default when left out -- or a single data port); generated code cannot stop: set "
                  "DiagnosticForDefault to 'None' with at least two data ports" % ", ".join(rd[:4]))


def generate(d, outputs, point_names=None, batch=False, allow_strict=False):
    """C source for compiled design d recording `outputs` (block names). The program also reports the first
    quantisation point that left its type ("OVERFLOW step block_index", blocks listed in point_names).
    batch=True (verification, engine/verify_*.py): stdin K, then K cases "D-delay initial values, n, inputs";
    stdout one line per case "CASE k OK" or "CASE k OVERFLOW step block_index", no output values."""
    if any(stop for _, _, stop in (getattr(d, "hdl_info", None) or {}).get("assertions") or []):
        raise CodegenError("UNSUPPORTED", "design", "an Assertion that stops the simulation (StopWhenAssertionFail "
                           "on) has no generated-C form")
    if not d.vm_ok:
        raise CodegenError("UNSUPPORTED", "design", "design needs the engine's Python path (wider than 126 bits or "
                           "blocks without integer instructions); no C generation")
    if not allow_strict:   # allow_strict: the prover's own overflow-witness programs (not a design answer)
        _refuse_strict(d, CodegenError)
    slot = {n: i for i, n in enumerate(d.slots)}

    def S(x):
        return "V[%d]" % slot[x]
    body = []
    names_pt = point_names if point_names is not None else []
    for n in d.order:
        if n not in d.vm:
            continue
        e = d.vm[n]
        body.append("/* %s (%s) */" % (n.replace("*/", "* /"), d.btype[n]))
        # counters wrap by definition and are excluded from the overflow contract (fxprove._counters): their
        # wrap is not a counterexample
        body.append("PT = %d;" % (-2 if d.btype[n] in ("CounterLimited", "CounterFreeRunning") else len(names_pt)))
        names_pt.append(n)
        for op, sl, rest in (e if isinstance(e, list) else [e]):
            body += _emit(op, sl, rest, S)
    dly = [(x, s_, ic) for n in d.names for (x, s_, ic) in d.vm_dly.get(n, ())]
    ins = list(d.inports)
    src = [RUNTIME, "#define NS %d" % max(1, len(d.slots)), "static i128 V[NS];", "",
           "/* inports: %s; outputs: %s */" % (", ".join(ins), ", ".join(outputs)),
           "static void init(void) {"]
    src += ["    V[%d] = %s;" % (slot[x], _lit(ic)) for x, _, ic in dly]
    src += ["}", "", "static void step(void) {"]
    src += ["    " + ln for ln in body]
    src += ["}", "", "static void update(void) {"]
    if dly:
        src.append("    i128 nv[%d];" % len(dly))
        src += ["    nv[%d] = V[%d];" % (k, slot[s_]) for k, (x, s_, ic) in enumerate(dly)]
        src += ["    V[%d] = nv[%d];" % (slot[x], k) for k, (x, s_, ic) in enumerate(dly)]
    if batch:
        src += ["}", "", "int main(void) {",
                "    long K = (long)rdint();",
                "    for (long c = 0; c < K; c++) {"]
        src += ["        V[%d] = rdint();" % slot[x] for x, _, ic in dly]
        src += ["        long n = (long)rdint();",
                "        if (n < 0 || n > 100000) return 2;",
                "        i128 in[%d * 64];" % max(1, len(ins)),
                "        if (n > 64) return 2;",
                "        for (int p = 0; p < %d; p++) for (long t = 0; t < n; t++) in[p * 64 + t] = rdint();" % len(ins),
                "        OV_STEP = -1; OV_PT = -1;",
                "        for (long t = 0; t < n; t++) {", "            STEP = t;"]
        src += ["            V[%d] = in[%d * 64 + t];" % (slot[x], k) for k, x in enumerate(ins)]
        src += ["            step();", "            update();", "        }",
                '        if (OV_STEP >= 0) printf("CASE %ld OVERFLOW %ld %d\\n", c, OV_STEP, OV_PT);',
                '        else printf("CASE %ld OK\\n", c);', "    }", "    return 0;", "}", ""]
        return "\n".join(src)
    src += ["}", "", "int main(void) {",
            "    long n = (long)rdint();",
            "    if (n < 0) return 2;",
            "    i128 *in = malloc(sizeof(i128) * (size_t)(n ? n : 1) * %d);" % max(1, len(ins)),
            "    i128 *out = malloc(sizeof(i128) * (size_t)(n ? n : 1) * %d);" % max(1, len(outputs)),
            "    if (!in || !out) return 3;",
            "    for (int p = 0; p < %d; p++) for (long t = 0; t < n; t++) in[p * n + t] = rdint();" % len(ins)]
    for k, x in enumerate(ins):   # input samples are checked against their declared types
        t = d.otype[x]
        lo, hi = (0, 1) if t == G.BOOL else ((-(1 << (t[1] - 1)), (1 << (t[1] - 1)) - 1) if t[0] else (0, (1 << t[1]) - 1))
        src += ["    for (long t = 0; t < n; t++) if (in[%d * n + t] < %s || in[%d * n + t] > %s) {" % (k, _lit(lo), k, _lit(hi)),
                '        fprintf(stderr, "input %s: sample %%ld outside its type\\n", t); return 4; }' % label(x)]
    src += ["    init();",
            "    for (long t = 0; t < n; t++) {", "        STEP = t;"]
    src += ["        V[%d] = in[%d * n + t];" % (slot[x], k) for k, x in enumerate(ins)]
    src += ["        step();"]
    src += ["        out[%d * n + t] = V[%d];" % (k, slot[o]) for k, o in enumerate(outputs)]
    src += ["        update();", "    }"]
    for k, o in enumerate(outputs):
        src += ['    fputs("%s", stdout);' % label(o),
                "    for (long t = 0; t < n; t++) { putchar(' '); pr(out[%d * n + t]); }" % k,
                "    putchar('\\n');"]
    src += ['    if (OV_STEP >= 0) printf("OVERFLOW %ld %d\\n", OV_STEP, OV_PT);',
            "    free(in); free(out);", "    return 0;", "}", ""]
    return "\n".join(src)

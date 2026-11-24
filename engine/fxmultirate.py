"""multirate simulation of a compiled design (Python reference path).

Timing rules: recordings, fitted to recordings (recordings*_dev):
  * every signal has a discrete period (a Fraction of seconds) and offset 0; the base step is the gcd of all
    periods; a block executes on the base steps that are multiples of its period and holds its output between;
  * single-input blocks inherit their input's period, multi-input blocks the gcd of their inputs' periods;
  * within a base step blocks execute in the data-dependency order; delays advance on their own hits;
  * Downsample K, offset o (multirate): y[m] = x[mK] for o = 0, else y[0] = IC and y[m] = x[(m-1)K + o];
    single-rate: output at the input rate = the latest x[i] with i mod K = o (IC before the first);
  * Upsample L, offset o: output period P/L; y[j] = x[j div L] when j mod L = o, else 0;
  * Rate Transition fast -> slow: the current input; slow -> fast (out = in / K): Integrity on and Deterministic on:
    y[j] = IC for j < K, else x[j div K - 1]; Integrity on, Deterministic off: y[0] = IC, y[j] = x[(j-1) div K];
    Integrity off: y[j] = x[j div K]; a period ratio that is not an integer is refused (as Simulink does).
Anything else (non-zero offsets, Enforce single-rate Upsample, explicit periods that clash) is refused."""
import os
import re
import collections
import math
from fractions import Fraction

import fxgraph_lib as G


def _frac(text, where):
    t = str(text).strip()
    try:
        if "/" in t:
            a, b = t.split("/")
            return Fraction(int(a), int(b))
        return Fraction(t)
    except (ValueError, ZeroDivisionError):
        raise G.GraphError("BAD_PARAM", where, "rational sample-time value required, got %r" % text)


def _gcd(a, b):
    return Fraction(math.gcd(a.numerator * b.denominator, b.numerator * a.denominator), a.denominator * b.denominator)


def _int_param(p, key, where, lo=1):
    try:
        v = int(str(p[key]).strip())
    except (ValueError, KeyError):
        raise G.GraphError("BAD_PARAM", where + "." + key, "integer required")
    if v < lo:
        raise G.GraphError("BAD_PARAM", where + "." + key, "must be >= %d" % lo)
    return v


def _sample_time(p, where, with_offset=False):
    v = str(p.get("SampleTime", "-1")).strip()
    off = Fraction(0)
    if v in ("-1", "inf", "Inf"):
        return (None, off) if with_offset else None
    if v.startswith("["):
        parts = v.strip("[]").replace(",", " ").split()
        if len(parts) not in (1, 2):
            # '[2 1 3]' was read as period 2 (Simulink:SampleTime:InvTsParamSetting_Vector)
            raise G.GraphError("BAD_PARAM", where + ".SampleTime", "a sample time is a period or [period offset]; "
                               "%r is rejected by Simulink" % v)
        if len(parts) == 2:
            off = _frac(parts[1], where + ".SampleTime")
            if off != 0 and not with_offset:
                raise G.GraphError("UNSUPPORTED_RATE", where + ".SampleTime", "sample time offset only on Inports")
        v = parts[0]
    t = _frac(v, where + ".SampleTime")
    if t <= 0:
        raise G.GraphError("UNSUPPORTED_RATE", where + ".SampleTime", "only discrete positive periods")
    if not (0 <= off < t):
        # recordings (72 records, both solver modes): [P P], [P P+1], [P -1] -- Simulink rejects
        # them before compiling (Simulink:SampleTime:InvTsParamSetting_Vector)
        raise G.GraphError("BAD_PARAM", where + ".SampleTime", "sample time offset %s outside [0, %s): Simulink "
                           "rejects it (InvTsParamSetting_Vector)" % (off, t))
    return (t, off) if with_offset else t


# ---------------------------------------------------------------- FIR Decimation / Interpolation (dspmlti4)
# Rules fitted to recordings:
#   coefficients: explicit fixdt, or 'Same word length as input' = signed, best precision; quantised to nearest,
#   saturated. Inherited product / accumulator / output types are signed; 'Same as input' = signed version of the
#   input type; product 'Inherit via internal rule' = full precision (w_x + w_c, also for an unsigned input),
#   widened to the next native word (8 / 16 / 32 / 64) -- an ideal product above 32 bits is rejected like Simulink
#   (dspshared:block:outProdWdLenUnsupHdwr); accumulator internal rule = ideal product width + ceil(log2(taps per
#   phase)), widened to a native word. Every product is cast to the product type, then to the accumulator type,
#   summed in the accumulator (wrap or saturate), the sum cast to the output type, all with the block's rounding
#   ('Simplest' behaves as Floor). Decimation sums the polyphase branches from the last phase to the first;
#   interpolation sums the taps of the active phase in coefficient order. The transposed structure with saturation
#   is refused (its partial-sum order is not established).
FIR_RM = {"Floor": "Floor", "Ceiling": "Ceiling", "Zero": "Zero", "Nearest": "Nearest", "Round": "Round",
          "Convergent": "Convergent", "Simplest": "Floor"}


def _native(w):
    for n in (8, 16, 32, 64):
        if w <= n:
            return n
    return w


def _coefs(p, where):
    try:
        return [float(x) for x in str(p["h"]).strip().strip("[]").replace(",", " ").split()]
    except ValueError:   # the shared rule: a MATLAB expression is UNSUPPORTED, anything else BAD_PARAM
        raise G.not_a_number(p["h"], where + ".h", "numeric coefficient vector")


def _fixdt_param(text, where):
    t = G._fixdt(text, where)
    if t is None:
        raise G.GraphError("UNSUPPORTED", where, "fixdt(...) required, got %r" % (text,))
    return t


def _fir_coef_fl(c, w):
    """Best-precision fraction length of one FIR Decimation / Interpolation coefficient in a signed w-bit word: the
    largest f with -2^(w-1) <= c 2^f < 2^(w-1) (before rounding)."""
    x = Fraction(c)
    f = w - 1 - (math.floor(math.log2(abs(c))) if c else 0)
    lim = Fraction(1 << (w - 1))
    while not (-lim <= x * Fraction(2) ** f < lim):
        f -= 1
    while -lim <= x * Fraction(2) ** (f + 1) < lim:
        f += 1
    return f


def _factor(p, dec, where):
    """The decimation (D) or interpolation (L) factor: a positive integer (0 divided by zero)."""
    k = "D" if dec else "L"
    try:
        R = int(str(p[k]).strip())
    except ValueError:
        raise G.GraphError("BAD_PARAM", "%s.%s" % (where, k), "an integer factor is required")
    if R < 1:
        raise G.GraphError("BAD_PARAM", "%s.%s" % (where, k), "%s factor %d: a positive integer is required"
                           % ("decimation" if dec else "interpolation", R))
    return R


def fir_types(p, xt, where, asic=False):
    """(coef, product, accumulator, output) types of a FIR Decimation / Interpolation for input type xt; asic: the
    hardware is ASIC/FPGA (the internal-rule product / accumulator words differ)."""
    if xt == G.BOOL:
        raise G.GraphError("UNSUPPORTED", where, "boolean input to a multirate FIR")
    # only the recorded multirate settings are modelled (the .slx path checks them in the regression check)
    for k, ok in (("FilterSource", "Dialog parameters"), ("framing", "Allow multirate processing")):
        if str(p.get(k, ok)) != ok:
            raise G.GraphError("UNSUPPORTED", "%s.%s" % (where, k), "%s=%r is not supported (only %r)" % (k, p[k], ok))
    h = _coefs(p, where)
    dec = "D" in p and "L" not in p
    R = _factor(p, dec, where)
    if p["roundingMode"] not in FIR_RM:
        raise G.GraphError("UNSUPPORTED", where + ".roundingMode", "rounding %r" % p["roundingMode"])
    if dec and p.get("filtStruct") == "Direct form transposed" and p["overflowMode"] == "on":
        raise G.GraphError("UNSUPPORTED", where + ".filtStruct", "FIR Decimation: transposed structure with saturation "
                           "is not supported (direct form, or wrap-around)")
    if dec and p.get("filtStruct", "Direct form") not in ("Direct form", "Direct form transposed"):
        raise G.GraphError("UNSUPPORTED", where + ".filtStruct", "%r" % p["filtStruct"])
    if p["firstCoeffMode"] == "Binary point scaling":
        ct = _fixdt_param(p["firstCoeffDataTypeStr"], where + ".firstCoeffDataTypeStr")
    elif p["firstCoeffMode"] == "Same word length as input" and all(c == 0 for c in h):
        # all-zero coefficients make Simulink's inherited coefficient type its own (fraction ~1000)
        raise G.GraphError("UNSUPPORTED", where + ".h", "every coefficient is 0 with the inherited coefficient type, "
                           "whose type Simulink chooses by a rule not modelled; give an explicit coefficient type")
    elif p["firstCoeffMode"] == "Same word length as input":
        # the Gain parameter's rule per coefficient, as Discrete FIR -- recordings: -1 fits En15
        # on sfix16 (the |h|-symmetric _best_fl gave En14)
        # recordings (neighbourhood probe): the fraction is the largest at
        # which each coefficient lies inside [-2^(w-1), 2^(w-1)) LSB before rounding -- 0.9476 on 4 bits is sfix4_En3
        # (saturated to 0.875, a precision-loss warning, inside the one LSB Simulink accepts), where the Gain
        # parameter's rule (nearest-rounded value must fit) gave En2: 122 of 125 values wrong. the recorded
        # -1 -> En15 on sfix16 holds under both
        fls = [_fir_coef_fl(c, xt[1]) for c in h if c != 0]
        ct = (1, xt[1], min(fls) if fls else xt[1] - 1)
        if xt[1] < 2:
            # a one-bit input gives a signed one-bit coefficient type, which Simulink does not allow
            # (as the Sum); what it picks instead is not recorded
            raise G.GraphError("UNSUPPORTED_WIDTH", where + ".firstCoeffMode", "the coefficient type 'Same word length "
                               "as input' of a one-bit input is a signed one-bit type, which Simulink does not allow; "
                               "its choice here is not recorded")
    else:
        raise G.GraphError("UNSUPPORTED", where + ".firstCoeffMode", "%r" % p["firstCoeffMode"])
    xs = (1, xt[1], xt[2])
    pm = p["prodOutputMode"]
    pw = xt[1] + ct[1]   # an unsigned w_x by signed w_c product fits in w_x + w_c signed bits
    if pm == "Binary point scaling":
        pt = _fixdt_param(p["prodOutputDataTypeStr"], where + ".prodOutputDataTypeStr")
        pw_ideal = pt[1]
    elif pm == "Same as input":
        pt, pw_ideal = xs, xs[1]
    elif pm == "Inherit via internal rule":
        if pw > 32 and not asic:
            raise G.GraphError("PORT_TYPE_INVALID", where, "full-precision product of %d bits exceeds the 32-bit host "
                               "word (Simulink dspshared:block:outProdWdLenUnsupHdwr)" % pw)
        # on ASIC/FPGA the product word is exact (w_x + w_c, and
        # Simulink accepts it above 32 bits: recordings, 11 of 11); on the host the native container (recorded clean)
        pt, pw_ideal = (1, pw if asic else _native(pw), xt[2] + ct[2]), pw
    else:
        raise G.GraphError("UNSUPPORTED", where + ".prodOutputMode", "%r" % pm)
    nphase = (len(h) + R - 1) // R
    am = p["accumMode"]
    if am == "Binary point scaling":
        at = _fixdt_param(p["accumDataTypeStr"], where + ".accumDataTypeStr")
    elif am == "Same as input":
        at = xs
    elif am == "Same as product output":
        at = pt
    elif am == "Inherit via internal rule":
        # recordings (b06 recordings, the re-check of recordings): ASIC/FPGA
        # the exact word, product + ceil(log2(taps per phase)); host one tap per phase the exact product word, more
        # taps the native container of product + guard bits, at most 32 (recordings 4032 / 4035 / 4075
        # / 4109: 11..12 bits -> sfix16; recordings: 5..8 bits -> sfix8, sfix7 x 4 taps -> sfix16;
        # above 32 the engine took 64 -- silent wrong values)
        guard = math.ceil(math.log2(nphase)) if nphase > 1 else 0
        if asic:
            at = (1, pw_ideal + guard, pt[2])
        elif nphase == 1:
            at = (1, pw_ideal, pt[2])
        else:
            at = (1, min(_native(pw_ideal + guard), 32), pt[2])
    else:
        raise G.GraphError("UNSUPPORTED", where + ".accumMode", "%r" % am)
    om = p["outputMode"]
    if om == "Binary point scaling":
        ot = _fixdt_param(p["outputDataTypeStr"], where + ".outputDataTypeStr")
    elif om == "Same as input":
        ot = xs
    elif om == "Same as product output":
        ot = pt
    elif om == "Same as accumulator":
        ot = at
    else:
        raise G.GraphError("UNSUPPORTED", where + ".outputMode", "%r" % om)
    for t in (ct, pt, at, ot):
        if t[1] > 64:
            raise G.GraphError("UNSUPPORTED", where, "internal word length above 64 bits")
    return ct, pt, at, ot


def _best_fl(h, w):
    m = max(abs(x) for x in h) if h else 0
    if m == 0:
        return w - 1
    f = w - 2 - math.floor(math.log2(m))
    hi = (1 << (w - 1)) - 1
    while round(m * 2 ** f) > hi:
        f -= 1
    while round(m * 2 ** (f + 1)) <= hi:
        f += 1
    return f


def _fir_setup(d, n):
    p = d.params[n]
    where = "design.blocks." + n
    xt = d.otype[d.src[n][0]]
    ct, pt, at, ot = fir_types(p, xt, where, G.fc._asic(d.hw))
    h = _coefs(p, where)
    # recordingsb (b02 ri085 / ri089): a coefficient
    # outside the coefficient type, or outputBufInitCond outside the output type, stops Simulink (fxpParameterOverflow,
    # default diagnostics); the engine had saturated them. Same line as every other parameter (fxgraph_lib)
    pom, notes = getattr(d, "pom", "error"), getattr(d, "param_notes", None)
    for j, c in enumerate(h):   # (element j of a vector h as h[j], as every other list parameter)
        G._param_overflow(c, ct, where + ".h", "%s %s: a coefficient" % (d.btype[n], n), mode=pom, notes=notes,
                          note_where=G._elem_where(where + ".h", j, len(h)))
    try:
        ic_ = float(str(p.get("outputBufInitCond", "0")).strip())
    except ValueError:
        ic_ = None
    if ic_ is not None and isinstance(ot, tuple):
        # Decimation: the value itself. Interpolation: Simulink checks outputBufInitCond / L against the output type on
        # the same line (20 / 20 beyond the line and recordings
        # stop with fxpParameterOverflow, 104 inside run); the recorded three designs were inside that line (IC / L in the
        # type although IC itself was not) -- an earlier version omitted the Interpolation check altogether
        L_ = 1
        if d.btype[n] == "FIRInterpolation":
            try:
                L_ = int(str(p.get("L", "1")).strip())
            except ValueError:
                L_ = 1
        G._param_overflow(ic_ / L_, ot, where + ".outputBufInitCond", "%s %s: outputBufInitCond%s" % (
            d.btype[n], n, " / L (%d)" % L_ if L_ != 1 else ""), mode=pom, notes=notes)
    cq = [G._quant_param(c, *ct) for c in h]
    fm = {"rm": FIR_RM[p["roundingMode"]], "om": "Saturate" if p["overflowMode"] == "on" else "Wrap"}
    dec = d.btype[n] == "FIRDecimation"
    R = _factor(p, dec, where)
    return {"cq": cq, "ct": ct, "pt": pt, "at": at, "ot": ot, "xt": xt, "fm": fm, "dec": dec, "R": R, "hist": []}


def _fx_cast(v, f_from, t, fm):
    return G.fc._cast_int(v, f_from, t, fm) if hasattr(G.fc, "_cast_int") else _cast(v, f_from, t, fm)


def _cast(v, f_from, t, fm):
    s, w, f = t
    sh = f_from - f
    if sh <= 0:
        q = v << (-sh)
    else:
        q = v >> sh
        r = v - (q << sh)
        if r:
            half = 1 << (sh - 1)
            rm = fm["rm"]
            if rm == "Ceiling":
                q += 1
            elif rm == "Zero":
                q += 1 if v < 0 else 0
            elif rm == "Nearest":
                q += 1 if r >= half else 0
            elif rm in ("Round", "Convergent"):
                if r > half:
                    q += 1
                elif r == half:
                    q += (1 if v > 0 else 0) if rm == "Round" else (q & 1)
    return _ovf(q, t, fm["om"] == "Saturate")


def _ovf(v, t, sat):
    s, w, _ = t
    lo, hi = (-(1 << (w - 1)), (1 << (w - 1)) - 1) if s else (0, (1 << w) - 1)
    if lo <= v <= hi:
        return v
    if sat:
        return lo if v < lo else hi
    m = v & ((1 << w) - 1)
    return m - (1 << w) if s and m >> (w - 1) else m


def _fir_output(st, m):
    """Output sample m (decimation: at input index m*R; interpolation: fast index m)."""
    cq, xt, pt, at, ot, fm, R = st["cq"], st["xt"], st["pt"], st["at"], st["ot"], st["fm"], st["R"]
    X = st["hist"]
    n = len(cq)
    if st["dec"]:
        taps = [(j, m * R - j) for p in range(R - 1, -1, -1) for j in range(p, n, R)]
    else:
        ph, base = m % R, m // R
        taps = [(j, base - (j - ph) // R) for j in range(ph, n, R)]
    acc = None
    sat = fm["om"] == "Saturate"
    for j, i in taps:
        xv = X[i] if 0 <= i < len(X) else 0
        a = _cast(_cast(cq[j] * xv, xt[2] + st["ct"][2], pt, fm), pt[2], at, fm)
        acc = a if acc is None else _ovf(acc + a, at, sat)
    return _cast(acc if acc is not None else 0, at[2], ot, fm)


# ---------------------------------------------------------------- CIC Decimation / Interpolation (dspmlti4)
# Rules fitted to recordings: Hogenauer structure with registered integrators (each integrator outputs
# its state before adding), wrap-around arithmetic, Floor when a section drops LSBs.
#   decimation: N integrators at the input rate, sample every R-th input (offset 0), N combs (delay M) at the output
#     rate; full precision B = w_in + ceil(N log2(RM)) in every section, fraction length of the input.
#   interpolation: N combs at the input rate, zero-stuffing (sample on phase 0), N integrators at the output rate;
#     full-precision section j: w_in + ceil(log2 G_j), G_j = 2^j (combs), 2^(2N-j) (RM)^(j-N) / R (integrators).
#   'Specify word lengths': section j keeps its full-precision integer bits (fl_j = w_j - (B_j - f_in)), likewise the
#     output; 'Binary point scaling': explicit (w, f) per section and output; 'Minimum section word lengths':
#     decimation prunes LSBs by Hogenauer's rule for the given output word length, interpolation keeps full
#     precision sections. Signed input only (Simulink rejects unsigned); zero-latency CIC is refused (removed in R2026b).


def _ivec(text, where):
    try:
        return [int(float(v)) for v in str(text).strip().strip("[]").replace(",", " ").split()]
    except ValueError:
        raise G.GraphError("BAD_PARAM", where, "integer vector required")


def _hogenauer_dec(win, R, M, N, bout):
    from math import comb
    B = win + math.ceil(N * math.log2(R * M))
    f2 = []
    for j in range(1, 2 * N + 1):
        if j <= N:
            poly = [1]
            for _ in range(N - j + 1):
                q = [0] * (len(poly) + R * M - 1)
                for a, v in enumerate(poly):
                    for k in range(R * M):
                        q[a + k] += v
                poly = q
            for _ in range(j - 1):
                q = [0] * (len(poly) + R * M)
                for a, v in enumerate(poly):
                    q[a] += v
                    q[a + R * M] -= v
                poly = q
            f2.append(sum(v * v for v in poly))
        else:
            n2 = 2 * N + 1 - j
            f2.append(comb(2 * n2, n2))
    sig2 = (2.0 ** (2 * (B - bout))) / 12.0
    return B, [B - max(0, math.floor(-0.5 * math.log2(f) + 0.5 * math.log2(sig2) + 0.5 * math.log2(6.0 / N)))
               for f in f2]


def cic_types(p, xt, dec, where):
    """(sections [(w, f)] * 2N, output type); every word 2..128 bits."""
    secs, ot = _cic_types(p, xt, dec, where)
    _cic_words_ok(secs, ot, where)
    return secs, ot


def _cic_types(p, xt, dec, where):
    if xt == G.BOOL or not xt[0]:
        raise G.GraphError("PORT_TYPE_INVALID", where, "CIC needs a signed input (Simulink "
                           "dspshared:block:inDataTypeNotSupported2)")
    if "ero" in str(p.get("ftype", "")).lower() and "latency" in str(p.get("ftype", "")).lower():
        raise G.GraphError("UNSUPPORTED", where + ".ftype", "zero-latency CIC is not available in R2026b "
                           "(dsp:block:zeroLatCICNotSupport)")
    if dec and str(p.get("RateOptions", "Allow multirate processing")) != "Allow multirate processing":
        # recordings (recordings as given, without the setting) and the library default
        # (Enforce single-rate processing): Simulink rejects a scalar sample-based input
        # (dsp:block:inputNotMultDecFac); only multirate processing is modelled
        raise G.GraphError("UNSUPPORTED", where + ".RateOptions", "CIC Decimation with RateOptions=%r is not "
                           "supported (only 'Allow multirate processing'; Simulink rejects a scalar sample-based "
                           "input with single-rate processing)" % p["RateOptions"])
    R, M, N = (_int_param(p, k, where) for k in ("R", "M", "N"))
    if R < 2:
        # Simulink rejects R = 1 ("the resample factor (R) parameter
        # must be an integer greater than 1")
        raise G.GraphError("BAD_PARAM", where + ".R", "CIC resample factor R = %d: Simulink requires an integer "
                           "greater than 1" % R)
    win, fin = xt[1], xt[2]
    if dec:
        full = [win + math.ceil(N * math.log2(R * M))] * (2 * N)
    else:
        full = []
        for j in range(1, 2 * N + 1):
            g = Fraction(2 ** j) if j <= N else Fraction(2 ** (2 * N - j) * (R * M) ** (j - N), R)
            full.append(win + (math.ceil(math.log2(g)) if g > 1 else 0))
    mode = p["filterInternals"]
    if mode == "Full precision":
        return [(w, fin) for w in full], (1, full[-1], fin)
    ow = _int_param(p, "outputWordLength", where, lo=2)
    if mode == "Minimum section word lengths":
        if dec:
            B, wls = _hogenauer_dec(win, R, M, N, ow)
            return [(w, w - (B - fin)) for w in wls], (1, ow, ow - (B - fin))
        return [(w, fin) for w in full], (1, ow, ow - (full[-1] - fin))
    bps = _ivec(p["BPS"], where + ".BPS")
    if len(bps) != 2 * N:
        raise G.GraphError("BAD_PARAM", where + ".BPS", "%d section word lengths required" % (2 * N))
    if mode == "Specify word lengths":
        if not dec and any(w < fw for w, fw in zip(bps, full)):
            # interpolation with a section narrower than its full-precision word: recordings (targeted
            # recording) 18 of 21 such graphs match, 3 do not and no rule
            # explains which -- refused. Every section at least full precision:
            # 5 of 5 match.
            raise G.GraphError("UNSUPPORTED", where + ".BPS",
                               "CIC interpolation with 'Specify word lengths' and a section narrower than full "
                               "precision: how Simulink drops the bits is not established (recorded counterexamples); use "
                               "full-precision sections")
        return [(w, w - (fw - fin)) for w, fw in zip(bps, full)], (1, ow, ow - (full[-1] - fin))
    if mode == "Binary point scaling":
        fls = _ivec(p["FLPS"], where + ".FLPS")
        if len(fls) != 2 * N:
            raise G.GraphError("BAD_PARAM", where + ".FLPS", "%d section fraction lengths required" % (2 * N))
        return list(zip(bps, fls)), (1, ow, int(p["outputFracLength"]))
    raise G.GraphError("UNSUPPORTED", where + ".filterInternals", "%r" % mode)


def _cic_words_ok(secs, ot, where):
    """recordings: Simulink takes section and output word lengths of 2..128 bits; a design
    that needs more (R 4096, N 6 on a 53-bit input: 131 bits) is rejected at data-type propagation."""
    for w in [x[0] for x in secs] + [ot[1]]:
        if not 2 <= w <= 128:
            raise G.GraphError("BAD_PARAM", where, "CIC word length %d: Simulink takes 2..128 bits (it rejects the "
                               "model at data-type propagation)" % w)


def _cic_setup(d, n):
    p = d.params[n]
    dec = d.btype[n] == "CICDecimation"
    xt = d.otype[d.src[n][0]]
    secs, ot = cic_types(p, xt, dec, "design.blocks." + n)
    R, M, N = int(p["R"]), int(p["M"]), int(p["N"])
    return {"dec": dec, "R": R, "M": M, "N": N, "secs": secs, "ot": ot, "xt": xt, "I": [0] * N,
            "hist": [[0] * M for _ in range(N)], "cv": 0, "cf": xt[2], "i": 0}


def _to(v, f_from, w, f):
    return _cast(v, f_from, (1, w, f), {"rm": "Floor", "om": "Wrap"})


def _cic_integrators(st, v, f, secs):
    for k in range(st["N"]):
        w, fl = secs[k]
        v = _to(v, f, w, fl)
        f = fl
        old = st["I"][k]
        st["I"][k] = _ovf(old + v, (1, w, 0), False)
        v = old
    return v, f


def _cic_combs(st, v, f, secs):
    for k in range(st["N"]):
        w, fl = secs[k]
        v = _to(v, f, w, fl)
        f = fl
        h = st["hist"][k]
        c = _ovf(v - h[0], (1, w, 0), False)
        st["hist"][k] = h[1:] + [v]
        v = c
    return v, f


MAX_BASE_STEPS = 2000000


_OFFSET_DOMAIN = None


def _offset_domain():
    """engine/offset_domain.json (the regression check): the recorded [relation, block type, P, o] of explicit
    sample times on non-Inport blocks (SingleTasking)."""
    global _OFFSET_DOMAIN
    if _OFFSET_DOMAIN is None:
        import json as _json
        p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "offset_domain.json")
        _OFFSET_DOMAIN = {tuple(t) for t in _json.load(open(p))} if os.path.exists(p) else set()
    return _OFFSET_DOMAIN


def _explicit_rate(d, n, bt, pe, oe, new, new_off, where):
    """An explicit SampleTime (pe, oe) on block n against the inherited (new, new_off), SingleTasking: the written
    sample time when it is one of the recorded relations (engine/offset_domain.json), else None."""
    if oe and any(ch in str(n) for ch in G.RESERVED_CHARS):
        # recorded lowerings (enabled / resettable Unit Delays, ...) hand the user block's SampleTime to hidden plain
        # blocks; the offset relations were recorded on the plain blocks themselves only (an
        # UnitDelayEnabledSynchronous with SampleTime [2 1] answered wrongly) -- refused
        return False
    if (pe, oe) == (new, new_off):
        rel = "same" if oe else None
    elif new is not None and pe == new and new_off and oe == 0:
        rel = "to0"   # same period, the source on an offset grid: the block reads its last hit
    elif (new, new_off) == (1, 0) and pe > 1 and pe.denominator == 1:
        rel = "base"  # a base-rate source sampled by a slower block at o + kP (recordings direct, recordings Outport, ST)
    else:
        rel = None
    if rel is None:
        return None
    if (rel, bt, str(pe), str(oe)) in _offset_domain():
        return (pe, oe)
    return False


_AUTO_DOMAIN = None


def _auto_domain():
    global _AUTO_DOMAIN
    if _AUTO_DOMAIN is None:
        import json as _json
        p_ = os.path.join(os.path.dirname(os.path.abspath(__file__)), "auto_domain.json")
        _AUTO_DOMAIN = _json.load(open(p_)) if os.path.exists(p_) else {"blocks": [], "upsample": []}
    return _AUTO_DOMAIN


AUTO_RATE_CHANGERS = ("Downsample", "Upsample", "FIRDecimation", "FIRInterpolation", "CICDecimation",
                      "CICInterpolation", "RateTransition")


def rates(d):
    """{block: period (Fraction) or None for constant-only signals}."""
    mode = str((getattr(d, "hw", None) or {}).get("SolverMode", "SingleTasking")).strip()
    if mode not in ("SingleTasking", "Auto"):
        # recordings (80 designs, both modes): the engine simulates Simulink's SingleTasking mode;
        # under Auto / MultiTasking (Auto is multitasking for a multirate model) Simulink delays an Upsample's output
        # by one slow period (25 of 28 designs differ) and rejects loops with two different explicit periods
        # (IllegalIPortRateTrans); MultiTasking is not simulated -- refused rather than answered in the wrong mode
        # SingleTasking and Auto are
        # recorded (Auto below: an Upsample's one-slow-period delay, direct connections between rates rejected);
        # MultiTasking is not recorded
        raise G.GraphError("UNSUPPORTED", "hw.SolverMode", "SolverMode %r in a multirate design is not recorded "
                           "(SingleTasking and Auto are)" % mode)
    # parameter checks shared by simulation, lowering (fxopt, fxcodegen) and Verilog (an offset outside
    # 0 .. factor-1 used to be checked only when simulating, so the lowering accepted it)
    for n in d.names:
        # an infinite sample time on anything but a Constant is refused in multirate designs too
        # (Simulink rejects it at set_param, InvTsParamSetting_Constant; the single-rate check had it, this path not)
        if str(d.params[n].get("SampleTime", "-1")).strip() in ("inf", "Inf") and d.btype[n] != "Constant":
            raise G.GraphError("BAD_PARAM", "design.blocks.%s.SampleTime" % n, "infinite sample time on %s: Simulink "
                               "accepts inf on constant blocks only (InvTsParamSetting_Constant)" % d.btype[n])
    for n in d.names:
        bt, p = d.btype[n], d.params[n]
        if bt in ("Downsample", "Upsample"):
            key = "DownsamplingFactor" if bt == "Downsample" else "UpsamplingFactor"
            K = _int_param(p, key, "design.blocks." + n)
            if K < 1:
                raise G.GraphError("BAD_PARAM", "design.blocks.%s.%s" % (n, key), "factor must be >= 1")
            o = _int_param(p, "SampleOffset", "design.blocks." + n, lo=0)
            if not 0 <= o < K:
                raise G.GraphError("BAD_PARAM", "design.blocks.%s.SampleOffset" % n, "0 .. factor-1")
    per, off = {}, {}
    for n in d.inports:
        if str(d.params[n].get("SampleTime", "-1")).strip() in ("inf", "Inf"):
            # an Inport with sample time inf was run at period 1; Simulink rejects it
            # (Simulink:SampleTime:InvTsParamSetting_Constant)
            raise G.GraphError("BAD_PARAM", "design.blocks.%s.SampleTime" % n, "an Inport cannot have sample time "
                               "inf (Simulink rejects it)")
        t, o = _sample_time(d.params[n], "design.blocks." + n, with_offset=True)
        per[n], off[n] = (t or Fraction(1)), o
    for n in d.names:
        if d.btype[n] == "Constant":
            st_ = str(d.params[n].get("SampleTime", "inf")).strip()
            if st_ not in ("inf", "Inf", "-1"):
                # Constant (SingleTasking): a Constant with its own [P o] outputs at
                # o + kP -- answered for the recorded (P, o) where nothing reads it but Outports
                t_, o_ = _sample_time(d.params[n], "design.blocks." + n, with_offset=True)
                users_ = [x for x in d.names if n in d.src[x]]
                if ("const", "Constant", str(t_), str(o_)) in _offset_domain() and \
                        all(d.btype[x] == "Outport" for x in users_):
                    per[n], off[n] = t_, o_
                    continue
            # a Constant with a period of its own would clock the stateful blocks it
            # feeds; the engine gives every Constant no period -- an explicit one is refused (not recorded)
            if st_ not in ("inf", "Inf", "-1"):
                raise G.GraphError("UNSUPPORTED_RATE", "design.blocks.%s.SampleTime" % n, "a Constant with its own "
                                   "sample time %r in a multirate design is not supported (inf / inherited only)"
                                   % d.params[n].get("SampleTime"))
            per[n] = None
    for _pass in range(2):
        for _ in range(len(d.names) + 2):
            changed = False
            for n in d.order:
                bt, p = d.btype[n], d.params[n]
                where = "design.blocks." + n
                if bt in ("Inport", "Constant"):
                    continue
                ins = [per.get(s, "?") for s in d.src[n]]
                known = [x for x in ins if x not in ("?", None)]
                offs = [off.get(s, Fraction(0)) for s in d.src[n] if per.get(s, "?") not in ("?", None)]
                o_in = offs[0] if offs else Fraction(0)
                if bt in ("Downsample", "Upsample", "FIRDecimation", "FIRInterpolation", "CICDecimation",
                          "CICInterpolation") and any(offs):
                    # Simulink dspshared:block:sampleTimeOffsetNotZero
                    raise G.GraphError("UNSUPPORTED_RATE", where, "%s does not accept a sample-time offset" % bt)
                new_off = o_in
                if not known:
                    if all(x is None for x in ins) and ins:
                        if bt in ("Downsample", "Upsample", "FIRDecimation", "FIRInterpolation", "CICDecimation",
                                  "CICInterpolation", "RateTransition"):
                            # a rate-changing block fed only by constants has no period to
                            # change; its behaviour is not established -- refused on every path
                            raise G.GraphError("UNSUPPORTED_RATE", where, "%s fed by a constant (sample time inf) "
                                               "is not supported" % bt)
                        new = None
                    else:
                        continue
                else:
                    pin = known[0]
                    for x in known[1:]:
                        pin = _gcd(pin, x)
                    if len(set(zip(known, offs))) > 1 and len(set(offs)) == 1 and offs[0]:
                        # inputs with the same non-zero offset
                        # and different periods ([2 1] + [4 1], [4 1] + [6 1]) run at [gcd period, offset] in Simulink
                        # -- the engine had put them on the base step with offset 0, so state behind them advanced
                        # twice as often
                        new_off = offs[0]
                        if not new_off < pin:
                            raise G.GraphError("UNSUPPORTED_RATE", where, "inputs with offset %s and periods whose "
                                               "gcd is %s: not recorded" % (new_off, pin))
                    elif len(set(zip(known, offs))) > 1:
                        # inputs on different grids: the block runs on their common base period, offset 0
                        for o in offs:
                            if o:
                                pin = _gcd(pin, o)
                        new_off = Fraction(0)
                    if bt == "Downsample":
                        K = _int_param(p, "DownsamplingFactor", where)
                        multi = p["RateOptions"] == "Allow multirate processing"
                        new = pin * K if multi else pin
                    elif bt == "CICDecimation":
                        new = pin * _int_param(p, "R", where)
                    elif bt == "CICInterpolation":
                        new = pin / _int_param(p, "R", where)
                    elif bt == "FIRDecimation":
                        new = pin * _int_param(p, "D", where)
                    elif bt == "FIRInterpolation":
                        new = pin / _int_param(p, "L", where)
                    elif bt == "Upsample":
                        L = _int_param(p, "UpsamplingFactor", where)
                        if p["RateOptions"] != "Allow multirate processing" and \
                                str(p.get("InputProcessing", "")).strip() != "Elements as channels (sample based)":
                            # recordings (findings/recordings = 2 / 3 / 4): sample based, Simulink gives the multirate
                            # result whatever RateOptions says; frame based single rate stays refused
                            raise G.GraphError("UNSUPPORTED_RATE", where + ".RateOptions",
                                               "Upsample frame based with single-rate processing is not supported")
                        new = pin / L
                    elif bt == "RateTransition":
                        if o_in and p["Deterministic"] == "on":
                            # Simulink:blocks:SampleTimeOffsetMustZero
                            raise G.GraphError("UNSUPPORTED_RATE", where, "deterministic Rate Transition from a signal "
                                               "with a sample-time offset")
                        new_off = Fraction(0)
                        def _from_dst():
                            # recordings (findings/recordings): OutPortSampleTime -1 (the library default) or the Inherit
                            # option -- Simulink takes the period of the block the output feeds (a Unit Delay /
                            # Gain with its own SampleTime; equal to an explicit Rate Transition of that period).
                            # One destination with an explicit period only; otherwise still refused
                            uk = getattr(d, "user_keys", {})
                            dsts = [m for m in d.names if n in d.src[m]]
                            ts_ = {_sample_time(d.params[m], "design.blocks." + m) for m in dsts
                                   if "SampleTime" in uk.get(m, set())}
                            ts_.discard(None)
                            if len(dsts) != 1 or len(ts_) != 1:
                                raise G.GraphError("UNSUPPORTED_RATE", where, "Rate Transition with an inherited output "
                                                   "period needs exactly one destination with an explicit SampleTime "
                                                   "(destinations %s); give OutPortSampleTime" % sorted(dsts))
                            return ts_.pop()
                        if p["OutPortSampleTimeOpt"] == "Multiple of input port sample time":
                            m_ = _frac(p["OutPortSampleTimeMultiple"], where + ".OutPortSampleTimeMultiple")
                            if m_ <= 0:
                                # a multiple of 0 / -2 ended INTERNAL (modulo by zero); Simulink
                                # refuses a non-positive multiple
                                raise G.GraphError("BAD_PARAM", where + ".OutPortSampleTimeMultiple", "Rate Transition "
                                                   "OutPortSampleTimeMultiple %s: Simulink needs a positive multiple"
                                                   % p["OutPortSampleTimeMultiple"])
                            new = pin * m_
                        elif p["OutPortSampleTimeOpt"] == "Specify":
                            new = _sample_time({"SampleTime": p["OutPortSampleTime"]}, where)
                            if new is None:
                                new = _from_dst()
                        elif p["OutPortSampleTimeOpt"] == "Inherit" and mode == "SingleTasking" and not o_in and \
                                all(d.btype[m] == "Outport" for m in d.names if n in d.src[m]) and \
                                any(n in d.src[m] for m in d.names) and \
                                not any("SampleTime" in getattr(d, "user_keys", {}).get(m, set())
                                        for m in d.names if n in d.src[m]):
                            # recordings (16 designs, SingleTasking): a Rate
                            # Transition with an inherited output sample time feeding Outports only runs at its input's
                            # period and passes the input through (every Integrity / Deterministic setting, input
                            # periods 1 / 2 / 3); other destinations and SolverMode Auto are not answered here (an
                            # Outport with its own SampleTime goes to the recordings rule below)
                            new = pin
                        elif p["OutPortSampleTimeOpt"] == "Inherit":
                            new = _from_dst()
                        else:
                            raise G.GraphError("UNSUPPORTED_RATE", where + ".OutPortSampleTimeOpt", "%r not supported"
                                               % p["OutPortSampleTimeOpt"])
                        r = new / pin
                        e_ic = getattr(d, "rt_ic_err", {}).get(n)
                        if e_ic is not None and not r > 1:
                            raise e_ic   # an initial value beyond the type stops Simulink unless fast-to-slow
                        if o_in and r.denominator != 1:
                            raise G.GraphError("UNSUPPORTED_RATE", where, "slow-to-fast Rate Transition from a signal with "
                                               "a sample-time offset is not supported")
                        if not (r.denominator == 1 or r.numerator == 1):
                            # Simulink:blocks:RateTransMultipleIptsOpts
                            raise G.GraphError("UNSUPPORTED_RATE", where, "Rate Transition between periods %s and %s: "
                                               "not an integer multiple" % (pin, new))
                    else:
                        new = pin
                        user = getattr(d, "user_keys", {}).get(n, set(p))
                        pe, oe = _sample_time(p, where, with_offset=True) if "SampleTime" in user else (None, 0)
                        if pe is not None and (pe, oe) != (new, new_off) or (pe is not None and oe):
                            got = _explicit_rate(d, n, bt, pe, oe, new, new_off, where)
                            if got:
                                new, new_off = got
                            elif oe or new_off or (pe, oe) != (new, new_off):
                                raise G.GraphError("UNSUPPORTED_RATE", where + ".SampleTime", "explicit sample time "
                                                   "[%s %s] with the inherited [%s %s]: %s" % (
                                                       pe, oe, new, new_off,
                                                       "this combination is not recorded (insert a Rate Transition, or "
                                                       "use a recorded sample time)" if got is False else
                                                       "not supported (insert a Rate Transition)"))
                if per.get(n, "?") != new or off.get(n, Fraction(0)) != new_off:
                    per[n] = new
                    off[n] = new_off
                    changed = True
            if not changed:
                break
        else:
            # a feedback loop through a rate-increasing block
            # (Upsample, FIR / CIC interpolation) halves the period on every pass; the unconverged periods depended
            # on the number of blocks in the design (base step 1/64 with 5 blocks, 1/1024 with 9) -- refused on
            # every path instead
            loop = sorted(n for n in d.names if d.btype[n] in ("Upsample", "FIRInterpolation", "CICInterpolation"))
            raise G.GraphError("UNSUPPORTED_RATE", "design.blocks." + (loop[0] if loop else d.order[0]),
                               "sample time does not converge (feedback loop through a rate-increasing block: %s)"
                               % (", ".join(loop) if loop else "?"))
        # source blocks without inputs (counters...) that nothing fixed: the model's fastest rate
        srcs = [n for n in d.names if n not in per and not d.src[n] and d.btype[n] not in ("Inport", "Constant")]
        known_p = [x for x in per.values() if x is not None]
        if not known_p:
            # recorded / recordings (a counter -> Downsample, no Inport): with no Inport no rate was known, so even a
            # counter with an explicit tsamp / SampleTime stayed unresolved ("sample time could not be resolved");
            # those explicit periods are now taken first, and propagation continues from them. The same for a block
            # with inputs and an explicit SampleTime (recorded HDL Counter lowering: a Unit Delay with SampleTime 1 in
            # a loop with inf-period Constants): its period is the one written; the propagation pass then checks
            # every other explicit period against what it inherits
            got = False
            seeds = srcs + [n for n in d.order if n not in per and d.src[n] and
                            d.btype[n] not in ("Inport", "Constant", "RateTransition") and
                            "SampleTime" in getattr(d, "user_keys", {}).get(n, set())]
            for n in seeds:
                user = getattr(d, "user_keys", {}).get(n, set())
                pn = d.params[n]
                if d.btype[n] in ("CounterLimited", "CounterFreeRunning") and "SampleTime" not in user and "tsamp" in user:
                    pn, user = dict(pn, SampleTime=str(pn["tsamp"])), user | {"SampleTime"}
                if "SampleTime" in user:
                    t_ = _sample_time(pn, "design.blocks." + n)
                    if t_ is not None:
                        per[n], off[n], got = t_, Fraction(0), True
            if got:
                continue
        else:
            # with an Inport fixing some rates, a block with inputs and an explicit SampleTime that no
            # propagation reached (the HDL Counter's lowered Unit Delay loop with inf-period Constants) stayed unresolved
            # and the design was refused; its written period seeds it too (Simulink runs these: findings/recordings.json)
            got = False
            for n in d.order:
                if n in per or not d.src[n] or d.btype[n] in ("Inport", "Constant", "RateTransition"):
                    continue
                if "SampleTime" in getattr(d, "user_keys", {}).get(n, set()):
                    t_ = _sample_time(d.params[n], "design.blocks." + n)
                    if t_ is not None:
                        per[n], off[n], got = t_, Fraction(0), True
            if got:
                continue
        if not srcs or not known_p:
            break
        fast = known_p[0]
        for x in known_p[1:]:
            fast = _gcd(fast, x)
        for n in srcs:
            user = getattr(d, "user_keys", {}).get(n, set())
            pn = d.params[n]
            if d.btype[n] in ("CounterLimited", "CounterFreeRunning") and "SampleTime" not in user and "tsamp" in user:
                # a counter's period is its tsamp (the single-rate path reads it so);
                # here it was ignored and the counter ran at the model's fastest rate
                pn = dict(pn, SampleTime=str(pn["tsamp"]))
                user = user | {"SampleTime"}
            if "SampleTime" not in user and d.btype[n] in ("CounterLimited", "CounterFreeRunning"):
                # a counter without tsamp in a multirate design -- the engine took the gcd of every
                # period, Simulink's default (-1) differs in three recorded shapes; refused, give tsamp
                raise G.GraphError("UNSUPPORTED_RATE", "design.blocks.%s.tsamp" % n, "a counter without an explicit "
                                   "sample time (tsamp) in a multirate design: Simulink's inherited rate here is not "
                                   "recorded; give tsamp")
            per[n] = (_sample_time(pn, "design.blocks." + n) if "SampleTime" in user else None) or fast
            off[n] = Fraction(0)
    missing = [n for n in d.names if n not in per]
    if missing:
        raise G.GraphError("UNSUPPORTED_RATE", "design.blocks." + missing[0], "sample time could not be resolved")
    d.mr_offsets = {n: off.get(n, Fraction(0)) for n in d.names}
    # a block with state driven only by constants takes its sample time backwards from its consumers (Simulink,
    # Detect Change and Unit Delay after a Constant both compiled to the downstream Sum's [3 0]);
    # the consumers are found through constant-rate stateless blocks. Several different consumer rates, or none,
    # are not observed and stay refused.
    users = {n: [] for n in d.names}
    for n in d.names:
        for s_ in d.src[n]:
            users[s_].append(n)
    for n in d.names:
        bt = d.btype[n]
        if per.get(n, "?") is None and (bt in G.DSTATEFUL or bt in G.DELAYS or bt == "DiscreteFir") and d.src[n]:
            seen, todo, rates_ = set(), list(users[n]), set()
            while todo:
                u = todo.pop()
                if u in seen:
                    continue
                seen.add(u)
                if per.get(u) is None:
                    todo += users[u]
                else:
                    rates_.add((per[u], off.get(u, Fraction(0))))
            if len(rates_) != 1:
                raise G.GraphError("UNSUPPORTED_RATE", "design.blocks." + n, "%s driven only by constants: its sample "
                                   "time comes from its consumers, which here have %s" % (
                                       bt, "no rate" if not rates_ else "different rates"))
            per[n], o_ = rates_.pop()
            off[n] = o_
            d.mr_offsets[n] = o_
    for n in d.names:   # see fxgraph_lib.dti_period_check
        if d.btype[n] == "DiscreteIntegrator" and per.get(n) not in (None, Fraction(1)):
            raise G.GraphError("UNSUPPORTED_RATE", "design.blocks." + n, G.DTI_PERIOD_MSG % per[n])
    # recordings (silent wrong, recorded vector pass): a Mux of signals at different periods is one signal Simulink
    # runs at the fastest of them (a Unit Delay after it steps at 1); the element copies took each its own rate. A
    # vector whose elements run at different periods is refused
    import re as _re
    groups = {}
    for n in d.names:
        m_ = _re.match(r"^(.+)\[(\d+)\]$", n)
        if m_ and "\x1d" not in n:
            groups.setdefault(m_.group(1), set()).add(per.get(n))
    mixed = sorted(k for k, v in groups.items() if len(v) > 1)
    if mixed:
        raise G.GraphError("UNSUPPORTED", "design.blocks." + mixed[0], "a vector signal whose elements run at "
                           "different sample periods (a Mux of signals at different rates): Simulink runs it at the "
                           "fastest period; not supported -- bring the signals to one rate before the Mux (Rate "
                           "Transition / Zero-Order Hold)")
    # recordings (class: complex signals in a multirate design): a Real-Imag to Complex whose parts run at
    # different periods -- Simulink runs it (and the blocks after it) at the fastest period, one sample per base step;
    # the parts kept their own periods here
    import fxhdl as _H
    info_ = getattr(d, "hdl_info", None) or {}
    for n_, re_, im_ in info_.get("cplx_pairs", []):
        try:
            pr_ = per.get(_H._port_block(info_, re_[0], re_[1], "design.blocks." + n_))
            pi_ = per.get(_H._port_block(info_, im_[0], im_[1], "design.blocks." + n_))
        except G.GraphError:
            continue
        if pr_ is not None and pi_ is not None and pr_ != pi_:
            raise G.GraphError("UNSUPPORTED", "design.blocks." + n_, "Real-Imag to Complex %s with the real part at "
                               "period %s and the imaginary part at %s: Simulink runs the complex signal at the "
                               "fastest period; not supported -- bring both parts to one rate first" % (n_, pr_, pi_))
    if mode == "Auto":
        # the Auto support domain (engine/auto_domain.json, the regression check): the block types and Upsample
        # (factor, offset) pairs of the Auto recordings Simulink ran
        dom = _auto_domain()
        for n in d.names:
            bt = d.btype[n]
            if any(ch in n for ch in G.RESERVED_CHARS):
                continue
            if bt not in dom["blocks"]:
                raise G.GraphError("UNSUPPORTED_RATE", "design.blocks." + n, "SolverMode Auto with a %s in a multirate "
                                   "design is not recorded (recorded with Auto: %s)" % (bt, ", ".join(dom["blocks"])))
            if "modes" in dom:
                import auto_rules as _mad
                if [bt, _mad.mode_sig(bt, d.params[n])] not in dom["modes"]:
                    raise G.GraphError("UNSUPPORTED_RATE", "design.blocks." + n, "SolverMode Auto with this %s "
                                       "parameter setting in a multirate design is not recorded" % bt)
            if bt == "RateTransition":
                # under Auto only the recorded (Integrity,
                # Deterministic, direction, factor) combinations (auto_domain.json rate_transitions), and the
                # fast-to-slow Integrity on / Deterministic on setting of recordings
                pin_ = [per.get(s_) for s_ in d.src[n] if per.get(s_) is not None]
                import auto_rules as _mad
                t_ = _mad.rt_tuple(d.params[n], pin_[0] if pin_ else None, per.get(n))
                # the recordings designs: fast to slow by 2 / 4 / 16 only (native tests: K = 5 was
                # answered beyond the declaration); a Rate Transition fed by another one was not recorded
                ok_ = t_ is not None and (t_ in dom.get("rate_transitions", []) or
                                          (t_[:2] == ["on", "on"] and t_[2] == "f2s" and t_[3] in (2, 4, 16)))
                if ok_ and any(d.btype.get(s_) == "RateTransition" for s_ in d.src[n] if s_ is not None):
                    raise G.GraphError("UNSUPPORTED_RATE", "design.blocks." + n, "SolverMode Auto with a Rate "
                                       "Transition fed by another Rate Transition is not recorded")
                if not ok_:
                    raise G.GraphError("UNSUPPORTED_RATE", "design.blocks." + n, "SolverMode Auto with this Rate "
                                       "Transition (Integrity %s, Deterministic %s, %s) is not recorded" % (
                                           d.params[n].get("Integrity"), d.params[n].get("Deterministic"),
                                           "%s by %s" % (t_[2], t_[3]) if t_ else "periods not known"))
            if bt == "Upsample" and [int(d.params[n]["UpsamplingFactor"]), int(d.params[n]["SampleOffset"])] \
                    not in dom["upsample"]:
                raise G.GraphError("UNSUPPORTED_RATE", "design.blocks." + n, "SolverMode Auto: an Upsample with "
                                   "factor %s and offset %s is not recorded" % (d.params[n]["UpsamplingFactor"],
                                                                               d.params[n]["SampleOffset"]))
        # SolverMode Auto (multitasking for a multirate model; recordings, 26 designs, and
        # recordings): a line joining two blocks of different sample times (period or offset), the
        # destination not a rate-changing block, is rejected -- Simulink:DataTransfer:IllegalIPortRateTrans
        for n in d.names:
            if d.btype[n] in AUTO_RATE_CHANGERS or per.get(n) is None:
                continue
            for s_ in d.src[n]:
                if s_ is None or per.get(s_) is None:
                    continue
                if (per[s_], d.mr_offsets.get(s_, Fraction(0))) != (per[n], d.mr_offsets.get(n, Fraction(0))):
                    raise G.GraphError("UNSUPPORTED_RATE", "design.blocks." + n, "SolverMode Auto (multitasking): "
                                       "%s (sample time [%s %s]) is driven directly by %s ([%s %s]); Simulink rejects "
                                       "this without a Rate Transition (Simulink:DataTransfer:IllegalIPortRateTrans)"
                                       % (n, per[n], d.mr_offsets.get(n, 0), s_, per[s_], d.mr_offsets.get(s_, 0)))
    return per


def precheck(d, inputs, t_stop):
    """The refusals run() makes before simulating (the integrator loop, the base-step limit, the input lengths, the
    resampler offsets, the multirate filters' types and parameter overflow): the C path through the single-rate
    lowering (fxgraph _mr_c) must refuse exactly what this path refuses."""
    if getattr(d, "dti_split", None):
        raise G.GraphError("UNSUPPORTED", "design.blocks." + d.dti_split[0], "a feedback loop closed through a "
                           "Discrete-Time Integrator in a multirate design is not supported (single-rate designs "
                           "run it); put a Unit Delay in the loop")
    per = rates(d)
    offs = d.mr_offsets
    periods = [p for p in per.values() if p is not None] + [o for o in offs.values() if o]
    base = periods[0]
    for p in periods[1:]:
        base = _gcd(base, p)
    step = {n: (int(p / base) if p is not None else 1) for n, p in per.items()}
    ost = {n: int(offs[n] / base) for n in d.names}
    nsteps = int(Fraction(t_stop) / base) + 1
    if nsteps > MAX_BASE_STEPS:
        raise G.GraphError("LIMIT_EXCEEDED", "stop_time", "%d base steps (base step %s) exceed the limit of %d for one "
                           "multirate simulation" % (nsteps, base, MAX_BASE_STEPS))
    for n in d.inports:
        need = max(0, (nsteps - 1 - ost[n]) // step[n] + 1)
        if len(inputs[n]) < need:
            raise G.GraphError("BAD_INPUT", "inputs." + n, "%d samples needed up to t = %s, got %d"
                               % (need, t_stop, len(inputs[n])))
    for n in d.names:
        bt, p = d.btype[n], d.params[n]
        if bt == "Downsample" and not 0 <= int(p["SampleOffset"]) < int(p["DownsamplingFactor"]):
            raise G.GraphError("BAD_PARAM", "design.blocks.%s.SampleOffset" % n, "0 .. factor-1")
        if bt == "Upsample" and not 0 <= int(p["SampleOffset"]) < int(p["UpsamplingFactor"]):
            raise G.GraphError("BAD_PARAM", "design.blocks.%s.SampleOffset" % n, "0 .. factor-1")
        if bt in ("FIRDecimation", "FIRInterpolation"):
            _fir_setup(d, n)
        elif bt in ("CICDecimation", "CICInterpolation"):
            _cic_setup(d, n)


def run(d, inputs, t_stop, record, hook=None):
    """inputs: {Inport: stored ints at the Inport's own rate}; returns {name: (times, values)} at each block's hits
    for t <= t_stop (times as Fractions). hook (run_diagnostics): a dict told the block and time being evaluated."""
    auto_ = str((getattr(d, "hw", None) or {}).get("SolverMode", "SingleTasking")).strip() == "Auto"
    if getattr(d, "dti_split", None):
        # follow-up: this simulator evaluates a Discrete-Time Integrator with its input of the same step,
        # so a loop closed through one gave other values than the single-rate engine -- refused until the
        # multirate path reads the integrator's state first
        raise G.GraphError("UNSUPPORTED", "design.blocks." + d.dti_split[0], "a feedback loop closed through a "
                           "Discrete-Time Integrator in a multirate design is not supported (single-rate designs "
                           "run it); put a Unit Delay in the loop")
    per = rates(d)
    offs = d.mr_offsets
    periods = [p for p in per.values() if p is not None] + [o for o in offs.values() if o]
    base = periods[0]
    for p in periods[1:]:
        base = _gcd(base, p)
    step = {n: (int(p / base) if p is not None else 1) for n, p in per.items()}
    ost = {n: int(offs[n] / base) for n in d.names}
    nsteps = int(Fraction(t_stop) / base) + 1
    if nsteps > MAX_BASE_STEPS:   # a 2^-22 base step made a short run take hours (stall, no answer)
        raise G.GraphError("LIMIT_EXCEEDED", "stop_time", "%d base steps (base step %s) exceed the limit of %d for one "
                           "multirate simulation" % (nsteps, base, MAX_BASE_STEPS))
    for n in d.inports:
        need = max(0, (nsteps - 1 - ost[n]) // step[n] + 1)
        if len(inputs[n]) < need:
            raise G.GraphError("BAD_INPUT", "inputs." + n, "%d samples needed up to t = %s, got %d"
                               % (need, t_stop, len(inputs[n])))
    ic = getattr(d, "ic", {})
    dlen = getattr(d, "dlen", {})
    mric = getattr(d, "mr_ic", {})
    state = {n: collections.deque([ic[n]] * dlen[n]) for n in d.delays}
    for k in d.kern.values():
        if isinstance(k, G._FxKernel):
            k.reset()
    val = {}
    logs = {n: ([], []) for n in record}
    mr = {}
    for n in d.names:
        bt, p = d.btype[n], d.params[n]
        if bt == "Downsample":
            mr[n] = {"K": int(p["DownsamplingFactor"]), "o": int(p["SampleOffset"]), "latch": mric[n], "m": 0,
                     "i": 0, "multi": p["RateOptions"] == "Allow multirate processing"}
            if not 0 <= mr[n]["o"] < mr[n]["K"]:
                raise G.GraphError("BAD_PARAM", "design.blocks.%s.SampleOffset" % n, "0 .. factor-1")
        elif bt == "Upsample":
            mr[n] = {"L": int(p["UpsamplingFactor"]), "o": int(p["SampleOffset"]), "j": 0}
            if not 0 <= mr[n]["o"] < mr[n]["L"]:
                raise G.GraphError("BAD_PARAM", "design.blocks.%s.SampleOffset" % n, "0 .. factor-1")
        elif bt == "RateTransition":
            mr[n] = {"hist": [], "j": 0, "integ": p["Integrity"] == "on", "det": p["Deterministic"] == "on",
                     "auto": str((getattr(d, "hw", None) or {}).get("SolverMode", "SingleTasking")).strip() == "Auto"}
        elif bt in ("FIRDecimation", "FIRInterpolation"):
            mr[n] = _fir_setup(d, n)
        elif bt in ("CICDecimation", "CICInterpolation"):
            mr[n] = _cic_setup(d, n)
    for n in d.names:  # every signal starts at its initial value (constants are evaluated every step)
        val[n] = mric.get(n, 0)
    for k in range(d_steps(nsteps)):
        hit = {n: k >= ost[n] and (k - ost[n]) % step[n] == 0 for n in d.names}
        idx = {n: (k - ost[n]) // step[n] for n in d.names}
        for n in d.order:
            bt = d.btype[n]
            src = d.src[n]
            if hook is not None:
                hook["n"], hook["t"] = n, k * base
            if bt == "Inport":
                if hit[n]:
                    val[n] = inputs[n][idx[n]]
                continue
            if bt in G.DELAYS:
                if hit[n]:
                    val[n] = state[n][0]
                continue
            if bt == "Downsample":
                st = mr[n]
                x = val[src[0]]
                if hit[src[0]] if per[src[0]] is not None else True:
                    i = idx[src[0]] if per[src[0]] is not None else k
                    if i % st["K"] == st["o"]:
                        st["latch"] = x
                if hit[n]:
                    if not st["multi"]:
                        val[n] = st["latch"]
                    elif st["o"] == 0:
                        val[n] = x
                    else:
                        val[n] = mric[n] if st["m"] == 0 else st["prev"]
                    st["m"] += 1
                # value captured at index (m-1)K + o is what the next output hit returns
                if st["multi"] and st["o"] > 0 and (hit[src[0]] if per[src[0]] is not None else True):
                    i = idx[src[0]] if per[src[0]] is not None else k
                    if i % st["K"] == st["o"]:
                        st["prev"] = x
                continue
            if bt == "Upsample":
                st = mr[n]
                if hit[n]:
                    j = idx[n]
                    if auto_:
                        # SolverMode Auto (28 Upsample designs): the output at
                        # o + kL is the PREVIOUS slow input sample (one slow period later than SingleTasking), the
                        # initial condition before the first
                        if (hit[src[0]] if per[src[0]] is not None else j % st["L"] == 0):
                            st["prev"] = st.get("cur", mric.get(n, 0))
                            st["cur"] = val[src[0]]
                        val[n] = st.get("prev", mric.get(n, 0)) if j % st["L"] == st["o"] else 0
                    else:
                        val[n] = val[src[0]] if j % st["L"] == st["o"] else 0
                continue
            if bt in ("FIRDecimation", "FIRInterpolation"):
                st = mr[n]
                s0 = src[0]
                if hit[s0]:
                    st["hist"].append(val[s0])
                if hit[n]:
                    val[n] = _fir_output(st, idx[n])
                continue
            if bt in ("CICDecimation", "CICInterpolation"):
                st = mr[n]
                s0 = src[0]
                N, secs, ot = st["N"], st["secs"], st["ot"]
                if st["dec"]:
                    if hit[s0]:
                        v, f = _cic_integrators(st, val[s0], st["xt"][2], secs[:N])
                        if st["i"] % st["R"] == 0:
                            v, f = _cic_combs(st, v, f, secs[N:])
                            st["out"] = _to(v, f, ot[1], ot[2])
                        st["i"] += 1
                    if hit[n]:
                        val[n] = st["out"]
                else:
                    if hit[n]:
                        j = idx[n]
                        if j % st["R"] == 0:
                            st["cv"], st["cf"] = _cic_combs(st, val[s0], st["xt"][2], secs[:N])
                        v = st["cv"] if j % st["R"] == 0 else 0
                        v, f = _cic_integrators(st, v, st["cf"], secs[N:])
                        val[n] = _to(v, f, ot[1], ot[2])
                continue
            if bt == "RateTransition":
                st = mr[n]
                s0 = src[0]
                if per[s0] is not None and hit[s0]:
                    st["hist"].append(val[s0])
                if hit[n]:
                    j = idx[n]
                    if per[s0] is None or per[n] >= per[s0]:      # fast -> slow (or equal): current input
                        # with Integrity on, before the input's first hit (an input with an offset)
                        # Simulink outputs the block's InitialCondition (the engine gave 0)
                        val[n] = mric[n] if (st["integ"] and per[s0] is not None and not st["hist"]) else val[s0]
                    else:
                        K = int(per[s0] / per[n])
                        # Keep the rate-transition index separate from per-block hit counters.
                        if not st["integ"] and st["auto"]:
                            # recordings (SolverMode Auto, Integrity off,
                            # Deterministic either way, K 2..4): the fast output runs one fast step behind its
                            # SingleTasking timing and starts from 0, not the InitialCondition
                            if j == 0:
                                val[n] = 0
                                continue
                            h = (j - 1) // K
                        elif not st["integ"]:
                            h = j // K
                        elif st["det"]:
                            h = j // K - 1
                        else:
                            h = (j - 1) // K if j >= 1 else -1
                        val[n] = st["hist"][h] if 0 <= h < len(st["hist"]) else mric[n]
                continue
            if hit[n] or per[n] is None:
                val[n] = d.kern[n]([val[s] for s in src])
        for n, (ts, vs) in logs.items():
            if per[n] is None or hit[n]:
                ts.append(k * base)
                vs.append(val[n])
        for n in d.delays:
            if hit[n]:
                q = state[n]
                q.popleft()
                q.append(val[d.src[n][0]])
    return logs


def run_diagnostics(d, inputs, t_stop, record):
    """run() that also counts, per block, the values that left a quantisation point's type -- wrapped or saturated --
    with the first time (seconds) of each, and every block's output range: the single-rate diagnostics
    (Design.run_diagnostics, the error reporting contract) for multirate designs. A CIC filter's integrator and section wraps are
    its design (modular arithmetic), not events, like a counter's. Returns (logs, {block: record})."""
    cur = {"n": None, "t": 0}
    ev = {}
    # a Saturation's cast into its output type with both limits inside it is not an event
    quiet = d.sat_cast_quiet() if hasattr(d, "sat_cast_quiet") else set()
    EMPTY = {"wraps": 0, "saturations": 0, "first_wrap_time": None, "first_saturation_time": None,
             "divisions_by_zero": 0, "first_division_by_zero_time": None}
    orig_ovf, orig_fc, orig_div0 = _ovf, G.fc._overflow, G.fc._div0

    def note(kind):
        n = cur["n"]
        if n is None or d.btype[n] in ("CICDecimation", "CICInterpolation") or n in quiet:
            return
        e = ev.setdefault(n, dict(EMPTY))
        e[kind + "s"] += 1
        if e["first_%s_time" % kind] is None:
            e["first_%s_time" % kind] = str(cur["t"])

    def ovf(v, t, sat):
        s_, w_, _ = t
        lo, hi = (-(1 << (w_ - 1)), (1 << (w_ - 1)) - 1) if s_ else (0, (1 << w_) - 1)
        if not lo <= v <= hi:
            note("saturation" if sat else "wrap")
        return orig_ovf(v, t, sat)

    def fc_overflow(n_, s_, wl, om):
        lo, hi = (-(1 << (wl - 1)), (1 << (wl - 1)) - 1) if s_ else (0, (1 << wl) - 1)
        if not lo <= n_ <= hi:
            note("saturation" if om == "Saturate" else "wrap")
        return orig_fc(n_, s_, wl, om)

    def div0(*a, **k):
        n = cur["n"]
        if n is not None:
            e = ev.setdefault(n, dict(EMPTY))
            e["divisions_by_zero"] += 1
            if e["first_division_by_zero_time"] is None:
                e["first_division_by_zero_time"] = str(cur["t"])
        return orig_div0(*a, **k)
    names = [n for n in d.order if d.btype[n] not in ("Inport", "Outport")]
    wanted = list(dict.fromkeys(list(record) + names))
    g = globals()
    g["_ovf"], G.fc._overflow, G.fc._div0 = ovf, fc_overflow, div0
    try:
        logs = run(d, inputs, t_stop, wanted, hook=cur)
    finally:
        g["_ovf"], G.fc._overflow, G.fc._div0 = orig_ovf, orig_fc, orig_div0
    out = {}
    for n in names:
        e = dict(ev.get(n, EMPTY))
        t = d.otype.get(n)
        vs = logs[n][1]
        e["type"] = d.type_str(n)
        if vs and isinstance(t, tuple):
            e.update(min=float(Fraction(min(vs)) / 2 ** t[2]), max=float(Fraction(max(vs)) / 2 ** t[2]),
                     stored_min=min(vs), stored_max=max(vs))
        elif vs:
            e.update(min=min(vs), max=max(vs))
        out[n] = e
    return {n: logs[n] for n in record}, out


def d_steps(n):
    return n

"""Exact fixed-point core: models MATLAB fi quantize/cast/add/sub/mul/neg with integer arithmetic."""
import math
import os
import re
import struct
from fractions import Fraction

ERR_MINWL = "fixed:numerictype:invalidMinWordLengthCodegen"


class FxInputError(ValueError):
    """Malformed input record or inconsistent operands: stable code, where, message."""

    def __init__(self, code, where, message):
        ValueError.__init__(self, "%s: %s" % (code, message))
        self.code, self.where, self.message = code, where, message


_INT_CLASSES = {"int8": (1, 8), "uint8": (0, 8), "int16": (1, 16), "uint16": (0, 16), "int32": (1, 32),
                "uint32": (0, 32), "int64": (1, 64), "uint64": (0, 64)}


def _check_type(v, where):
    s, wl, fl = v.get("s"), v.get("wl"), v.get("fl")
    for k, x in (("s", s), ("wl", wl), ("fl", fl)):
        if not isinstance(x, int) or isinstance(x, bool):
            raise FxInputError("BAD_TYPE", where + "." + k, "integer required, got %r" % (x,))
    if s not in (0, 1) or wl < 1:
        raise FxInputError("BAD_TYPE", where, "s must be 0 or 1 and wl >= 1, got s=%r wl=%r" % (s, wl))
    return s, wl, fl


def _check_bin(b, wl, where):
    if not isinstance(b, str) or len(b) != wl or set(b) - set("01"):
        raise FxInputError("BAD_VALUE", where, "bit string of exactly %d characters 0/1 required" % wl)


def _decode(v, where="value"):
    """Return (signed, wl, fl, int value) from a value record."""
    if "cls" in v:  # built-in integer recorded as {"cls": "int16"|"uint8"|..., "num": "..."}
        cls = v["cls"]
        if cls not in _INT_CLASSES:
            raise FxInputError("BAD_TYPE", where + ".cls", "unknown integer class %r" % (cls,))
        s, w = _INT_CLASSES[cls]
        try:
            n = int(v["num"])
        except (TypeError, ValueError, KeyError):
            raise FxInputError("BAD_VALUE", where + ".num", "integer required")
        lo, hi = (-(1 << (w - 1)), (1 << (w - 1)) - 1) if s else (0, (1 << w) - 1)
        if not lo <= n <= hi:
            raise FxInputError("BAD_VALUE", where + ".num", "%d outside %s" % (n, cls))
        return s, w, 0, n
    s, wl, fl = _check_type(v, where)
    _check_bin(v.get("bin"), wl, where + ".bin")
    n = int(v["bin"], 2)
    if s and n >> (wl - 1):
        n -= 1 << wl
    return s, wl, fl, n


def _round_div(n, sh, rm):
    """Divide integer n by 2**sh (sh>0) with the given rounding method."""
    q = n >> sh  # floor
    r = n - (q << sh)  # remainder in [0, 2**sh)
    if r == 0:
        return q
    half = 1 << (sh - 1)
    if rm == "Floor":
        return q
    if rm == "Ceiling":
        return q + 1
    if rm == "Zero":
        return q + 1 if n < 0 else q
    if rm == "Nearest":  # ties toward +inf
        return q + 1 if r >= half else q
    if rm == "Round":  # ties away from zero
        if r > half:
            return q + 1
        if r < half:
            return q
        return q + 1 if n > 0 else q
    if rm == "Convergent":  # ties to even
        if r > half:
            return q + 1
        if r < half:
            return q
        return q + (q & 1)
    raise ValueError(rm)


def _shift_round(n, fl_from, fl_to, rm):
    sh = fl_from - fl_to
    if sh <= 0:
        return n << (-sh)
    return _round_div(n, sh, rm)


def _div0():
    """Called where a division by zero saturates by the dividend's sign (no-op; engine run_diagnostics counts it)."""


def _overflow(n, s, wl, om):
    lo, hi = (-(1 << (wl - 1)), (1 << (wl - 1)) - 1) if s else (0, (1 << wl) - 1)
    if lo <= n <= hi:
        return n
    if om == "Saturate":
        return lo if n < lo else hi
    n &= (1 << wl) - 1
    if s and n >> (wl - 1):
        n -= 1 << wl
    return n


# the same function under a name engine run_diagnostics does not patch: for quantisations that are not run-time
# events of the current step (a recomputed history, parameter quantisation) -- recordings
_overflow_plain = _overflow


def _cast(n, fl_from, s, wl, fl, fm):
    return _overflow(_shift_round(n, fl_from, fl, fm["rm"]), s, wl, fm["om"])


def _out(s, wl, fl, n):
    return {"s": int(s), "wl": wl, "fl": fl, "bin": format(n & ((1 << wl) - 1), "0%db" % wl)}


def _x_to_fraction(hexstr):
    d = struct.unpack(">d", bytes.fromhex(hexstr))[0]
    return Fraction(d)


def _mode_type(mode, wl_spec, fl_spec, full_wl, full_fl):
    """Result type (wl, fl) for a non-FullPrecision mode, given the full-precision type."""
    if mode == "FullPrecision":
        return full_wl, full_fl
    if mode == "KeepLSB":
        return wl_spec, full_fl
    if mode == "KeepMSB":
        return wl_spec, wl_spec - (full_wl - full_fl)
    return wl_spec, fl_spec


def _round_sig(n, p):
    """Round integer n to p significant bits (nearest, ties to even); returns an integer."""
    drop = abs(n).bit_length() - p
    if drop <= 0:
        return n
    sign = -1 if n < 0 else 1
    return sign * (_round_div(abs(n), drop, "Convergent") << drop)


def _add_sub(case, sub):
    fm = case["fm"]
    sa, wa, fa, na = _decode(case["a"])
    sb, wb, fb, nb = _decode(case["b"])
    s = 1 if (sa or sb) else 0
    # full precision type
    fl = max(fa, fb)
    ia = wa - fa
    ib = wb - fb
    full_wl = max(ia, ib) + 1 + (1 if sa != sb else 0) + fl
    tw, tf = _mode_type(fm["sm"], fm["swl"], fm["sfl"], full_wl, fl)
    if fm["cbs"] and fm["sm"] != "FullPrecision":
        na = _cast(na, fa, s, tw, tf, fm)
        nb = _cast(nb, fb, s, tw, tf, fm)
        fa = fb = tf
    f = max(fa, fb)
    total = (na << (f - fa)) + ((-nb if sub else nb) << (f - fb))
    if fm["sm"] != "FullPrecision" and not fm["cbs"]:
        # observed: the exact sum is first rounded to a limited number of significant bits
        mw = max(wa, wb, fm["swl"])
        total = _round_sig(total, 53 if mw <= 52 else max(mw, 64) + 1)
    return _out(s, tw, tf, _cast(total, f, s, tw, tf, fm))


def _mul(case):
    fm = case["fm"]
    sa, wa, fa, na = _decode(case["a"])
    sb, wb, fb, nb = _decode(case["b"])
    s = 1 if (sa or sb) else 0
    full_wl, full_fl = wa + wb, fa + fb
    tw, tf = _mode_type(fm["pm"], fm["pwl"], fm["pfl"], full_wl, full_fl)
    return _out(s, tw, tf, _cast(na * nb, full_fl, s, tw, tf, fm))


def _cmp(case):
    sa, wa, fa, na = _decode(case["a"])
    sb, wb, fb, nb = _decode(case["b"])
    f = max(fa, fb)
    x, y = na << (f - fa), nb << (f - fb)
    r = {"lt": x < y, "le": x <= y, "gt": x > y, "ge": x >= y, "eq": x == y, "ne": x != y}[case["cmp"]]
    return {"bool": int(r)}


def _bitshift(case):
    fm = case["fm"]
    s, w, f, n = _decode(case["a"])
    k = case["k"]
    n = n << k if k >= 0 else n >> (-k)
    return _out(s, w, f, _overflow(n, s, w, fm["om"]))


def _abs(case):
    fm = case["fm"]
    s, w, f, n = _decode(case["a"])
    return _out(s, w, f, _overflow(abs(n), s, w, fm["om"]))


def _divide(case):
    fm = case["fm"]
    t = case["t"]
    sa, wa, fa, na = _decode(case["a"])
    sb, wb, fb, nb = _decode(case["b"])
    if nb == 0:
        # division by zero: saturates by the sign of the dividend, whatever the overflow action
        lo, hi = (-(1 << (t["wl"] - 1)), (1 << (t["wl"] - 1)) - 1) if t["s"] else (0, (1 << t["wl"]) - 1)
        _div0()
        return _out(t["s"], t["wl"], t["fl"], lo if na < 0 else hi)
    e = t["fl"] + fb - fa
    num = na << e if e >= 0 else na
    den = nb if e >= 0 else nb << (-e)
    if den < 0:
        num, den = -num, -den
    # observed: the quotient is first rounded (nearest, ties to even) to P significant bits
    # recordings (99840 cases): 53 significant bits when the target has <= 22 bits and both operands
    # have <= 32 bits (any rounding mode); otherwise max(129, T + 11). T = 23 is not in the data.
    if t["wl"] <= 22 and max(wa, wb) <= 32:
        prec = 53
    else:
        prec = max(129, t["wl"] + 11)  # recordings: 129 up to T ~ 118 bits, 138 at T=127, 139 at T=128
    num, den = _round_sig_ratio(num, den, prec)
    n = _round_quot(num, den, fm["rm"])
    return _out(t["s"], t["wl"], t["fl"], _overflow(n, t["s"], t["wl"], fm["om"]))


def _round_sig_ratio(num, den, p):
    """Round num/den (den > 0) to p significant bits, nearest with ties to even; returns (num', den')."""
    if num == 0:
        return num, den
    a = abs(num)
    e = a.bit_length() - den.bit_length()  # first guess; corrected below
    # exact binade: find e with 2**(e-1) <= |x| < 2**e
    while True:
        lo_ok = (a << max(1 - e, 0)) >= (den << max(e - 1, 0))
        hi_ok = (a << max(-e, 0)) < (den << max(e, 0))
        if not lo_ok:
            e -= 1
        elif not hi_ok:
            e += 1
        else:
            break
    k = p - e  # scale so that the integer part has p bits
    if k >= 0:
        q = _round_quot(a << k, den, "Convergent")
        out_num, out_den = q, 1 << k
    else:
        q = _round_quot(a, den << (-k), "Convergent")
        out_num, out_den = q << (-k), 1
    return (-out_num if num < 0 else out_num), out_den


def _round_quot(num, den, rm):
    """Round the rational num/den (den > 0) to an integer with the given rounding method."""
    q, r = divmod(num, den)
    if r == 0:
        return q
    if rm == "Floor":
        return q
    if rm == "Ceiling":
        return q + 1
    if rm == "Zero":
        return q + 1 if num < 0 else q
    c = 2 * r - den  # compare the remainder with half
    if c > 0:
        return q + 1
    if c < 0:
        return q
    if rm == "Nearest":
        return q + 1
    if rm == "Round":
        return q + 1 if num > 0 else q
    return q + (q & 1)


def _decode_vec(v, where="vector"):
    s, wl, fl = _check_type(v, where)
    bins = v.get("bins")
    if not isinstance(bins, list) or not bins:
        raise FxInputError("BAD_DIMENSION", where + ".bins", "non-empty list of bit strings required")
    ns = []
    for i, b in enumerate(bins):
        _check_bin(b, wl, "%s.bins[%d]" % (where, i))
        n = int(b, 2)
        if s and n >> (wl - 1):
            n -= 1 << wl
        ns.append(n)
    return s, wl, fl, ns


def _accumulate(items, f_in, s, tw, tf, fm, cast_each):
    """Sequentially accumulate integers (fraction length f_in) into the sum type (tw, tf)."""
    acc = 0
    for x in items:
        if cast_each:
            x = _cast(x, f_in, s, tw, tf, fm)
            fx = tf
        else:
            fx = f_in
        f = max(tf, fx)
        total = (acc << (f - tf)) + (x << (f - fx))
        if not cast_each:
            # observed: without cast-before-sum the exact running sum is first limited in significance
            m = fm["swl"]
            total = _round_sig(total, 53 if m <= 52 else max(m, 64) + 1)
        acc = _cast(total, f, s, tw, tf, fm)
    return acc


def _sum_type(fm, wl_in, fl_in, n):
    full_wl = wl_in + (n - 1).bit_length()  # growth by ceil(log2(n)) bits
    return _mode_type(fm["sm"], fm["swl"], fm["sfl"], full_wl, fl_in)


def _vsum(case):
    fm = case["fm"]
    s, wl, fl, ns = _decode_vec(case["av"])
    tw, tf = _sum_type(fm, wl, fl, len(ns))
    if fm["sm"] == "FullPrecision":
        return _out(s, tw, tf, _cast(sum(ns), fl, s, tw, tf, fm))
    return _out(s, tw, tf, _accumulate(ns, fl, s, tw, tf, fm, bool(fm["cbs"])))


def _dot(case):
    fm = case["fm"]
    s1, w1, f1, xs = _decode_vec(case["av"], "av")
    s2, w2, f2, ys = _decode_vec(case["bv"], "bv")
    if len(xs) != len(ys):
        raise FxInputError("BAD_DIMENSION", "bv.bins", "dot product of vectors of lengths %d and %d" % (len(xs), len(ys)))
    s = 1 if (s1 or s2) else 0
    full_fl = f1 + f2
    pw, pf = _mode_type(fm["pm"], fm["pwl"], fm["pfl"], w1 + w2, full_fl)
    prods = [_cast(x * y, full_fl, s, pw, pf, fm) for x, y in zip(xs, ys)]
    tw, tf = _sum_type(fm, pw, pf, len(prods))
    if fm["sm"] == "FullPrecision":
        return _out(s, tw, tf, _cast(sum(prods), pf, s, tw, tf, fm))
    cast_each = bool(fm["cbs"])
    if w1 + w2 <= 52 and tw <= 52:
        cast_each = False  # observed: small operands ignore CastBeforeSum (acts like the double path)
    return _out(s, tw, tf, _accumulate(prods, pf, s, tw, tf, fm, cast_each))


# ---------------------------------------------------------------------------
# Simulink block level . One case = one time step of one block.
# Rules are derived from the recorded
# behaviour of individual blocks.
# ---------------------------------------------------------------------------

_INHERIT_RULE = "Inherit: Inherit via internal rule"
_SAME_FIRST = "Inherit: Same as first input"
_SAME_INPUT = "Inherit: Same as input"
# Observed: a parameter whose value cannot be represented at all in an unsigned
# best-precision type (negative value) gets this fraction length.
_BEST_PREC_FL_UNREPRESENTABLE = 1000


def parse_type(text):
    """The one reading of a fixed-point data type string: 'fixdt(s,w,f)' or a built-in integer name
    int8 .. uint32 -> (s, w, f); None for anything else ('Inherit: ...', double, ...). Every resolver of an explicit
    type uses it, so an accumulator written int8 means what fixdt(1,8,0) means (ASIC Sum read it as unset)."""
    t = _parse_fixdt(text)
    if t is None and isinstance(text, str) and text.strip() in _NAMED_INT:
        t = _NAMED_INT[text.strip()]
    return t


def _parse_fixdt(text):
    """'fixdt(s,w,f)' -> (s, w, f); None for anything else."""
    text = text.strip()
    if not text.startswith("fixdt(") or not text.endswith(")"):
        return None
    m_ = re.match(r"^fixdt\(\s*'([su])fix([0-9]+)(?:_E(n?)([0-9]+))?'\s*\)$", text)
    if m_:
        # the type named by its compiled name (type-string survey: fixdt('ufix1'), fixdt('ufix17') in real models):
        # sfixW_EnF is fraction length F, sfixW_EF fraction length -F, no suffix 0
        f = int(m_.group(4)) if m_.group(4) else 0
        return (1 if m_.group(1) == "s" else 0), int(m_.group(2)), (f if m_.group(3) == "n" else -f)
    parts = [p.strip() for p in text[6:-1].split(",")]
    # coverage per block: the signedness may be written true / false, and the slope-bias
    # form fixdt(s, w, slope, bias) with a power-of-two slope and zero bias is the binary-point type itself
    if parts and parts[0] in ("true", "false"):
        parts[0] = "1" if parts[0] == "true" else "0"
    if len(parts) == 4 and re.match(r"^[+-]?0*(\.0*)?$", parts[3]):
        m = re.match(r"^2\s*\^\s*\(?\s*([+-]?[0-9]+)\s*\)?$", parts[2])
        if not m:
            return None
        parts = [parts[0], parts[1], str(-int(m.group(1)))]
    if len(parts) != 3:
        return None
    # ASCII integers only (int() read 'fixdt(1,1_6,8)' as sfix16_En8 and accepts non-ASCII digits);
    # anything else, e.g. 'fixdt(1,8,2^3)', is not a fixdt the engine reads (was an uncaught error)
    if not all(re.match(r"^[+-]?[0-9]+$", x) for x in parts):
        return None
    return int(parts[0]), int(parts[1]), int(parts[2])


def _hw_words(case):
    """Native integer sizes of the production hardware, smallest first; [] for ASIC/FPGA (no native sizes:
    internal rules keep the full-precision word)."""
    hw = case.get("hw") or {}
    if _asic(hw):
        return []
    sizes = {8, 16, int(hw.get("ProdBitPerInt", 32)), int(hw.get("ProdBitPerLong", 32))}
    if hw.get("ProdLongLongMode") == "on":
        sizes.add(int(hw.get("ProdBitPerLongLong", 64)))
    return sorted(sizes)


def _asic(hw):
    """Production hardware ASIC/FPGA: no native integer sizes. Evidence so far (recordings
    recordings): an inherited Gain output keeps the full-precision word (12 + 12 -> 24 bits, not 32).
    Other internal rules on ASIC/FPGA are not observed and keep the native-size behaviour."""
    return str((hw or {}).get("ProdHWDeviceType", "")).startswith("ASIC/FPGA")


def _native_container(w, words):
    """Round a word length up to the next native size; wider types are left as they are."""
    for n in words:
        if w <= n:
            return n
    return w


def _blk_fm(params):
    """fimath-like dict for a block. 'Simplest' behaves as Floor in every observed block."""
    rm = params.get("RndMeth", "Floor")
    if rm == "Simplest":
        rm = "Floor"
    om = "Saturate" if params.get("SaturateOnIntegerOverflow") == "on" else "Wrap"
    return {"rm": rm, "om": om}


def _ins(case):
    return [_decode(v) for v in case["ins"]]


def _int_bits(s, w, f, s_target):
    """Integer bits (sign bit included) a value of type (s, w, f) needs in a type of signedness s_target."""
    return w - f + (1 if (s_target and not s) else 0)


def _quant_param(value, s, w, f):
    """Quantise a double block parameter to (s, w, f): round to nearest, saturate."""
    x = Fraction(value) * (Fraction(2) ** f)
    n = _round_quot(x.numerator, x.denominator, "Nearest")
    lo, hi = (-(1 << (w - 1)), (1 << (w - 1)) - 1) if s else (0, (1 << w) - 1)
    return max(lo, min(hi, n))


def _best_precision_fl(value, s, w):
    """Largest fraction length whose nearest-rounded value still fits in (s, w)."""
    g = Fraction(value)
    if g == 0 or (not s and g < 0):
        return _BEST_PREC_FL_UNREPRESENTABLE
    lo, hi = (-(1 << (w - 1)), (1 << (w - 1)) - 1) if s else (0, (1 << w) - 1)
    a = abs(g)
    f = w + 2 - (a.numerator.bit_length() - a.denominator.bit_length())
    while True:
        x = g * (Fraction(2) ** f)
        n = _round_quot(x.numerator, x.denominator, "Nearest")
        if lo <= n <= hi:
            return f
        f -= 1


def gain_param_fl(value, s, w, asic, out_rule=True):
    """Best-precision fraction length of a Gain parameter under the internal rule.
    A positive value: the largest fraction whose nearest-rounded value still fits (_best_precision_fl; recordings,
    other recorded ASIC configurations). A negative value must fit as it is, before rounding: recordings (targeted
    recording, 3,200 Gains on x86-64 and 3,200 on ASIC/FPGA: a value just beyond the most negative code takes the
    coarser scale; 6,400 of 6,400 parameter types, while rounding-fits is wrong on all 3,200 such values; strict
    for positive values would be wrong on all 1,600 positive controls). recordings asic / out_rule: kept for the
    callers' signature (the rule is the same on both hardware classes and does not depend on the output rule)."""
    f = _best_precision_fl(value, s, w)
    if s and value < 0 and f != _BEST_PREC_FL_UNREPRESENTABLE:
        g = Fraction(value)
        while g * (Fraction(2) ** f) < -(1 << (w - 1)):
            f -= 1
    return f


def bool_gain_param_type(value, asic, out_rule=True):
    """Parameter type of a Gain fed by a boolean with ParamDataTypeStr 'Inherit: Inherit via internal rule'
    (26 gains on the host and on ASIC/FPGA): unsigned for a
    positive gain, signed for a negative one, at best precision in the SHORTEST word that holds the gain exactly --
    on the host the first of 8 / 16 / 32 bits (32 when none is exact: 0.3 -> ufix32_En33, recorded with
    recordings), on ASIC/FPGA any word length (3 -> ufix2, 1000 -> ufix7_E3, -129 -> sfix9, -128 ->
    sfix2_E6, a double that no shorter word holds: 0.3 -> ufix53_En54). A signed word has at least two bits.
    With the internal-rule output the output type is the parameter type (all 52 recordings but ASIC gain 1,
    whose output is ufix9 -- refused by the caller). Raises ValueError for a gain the rule does not cover (0)."""
    g = Fraction(value)
    if g == 0:
        raise ValueError("gain 0 with a boolean input is not recorded")
    s = 1 if g < 0 else 0
    words = (8, 16, 32) if not asic else range(2 if s else 1, 65)
    for w in words:
        f = gain_param_fl(value, s, w, asic, out_rule)
        if f != _BEST_PREC_FL_UNREPRESENTABLE and (g * Fraction(2) ** f).denominator == 1:
            return (s, w, f)
    if not asic:
        return (s, 32, gain_param_fl(value, s, 32, asic, out_rule))
    raise ValueError("gain %r is not exactly representable in 64 bits" % value)


UFIX1 = (0, 1, 0)


def one_bit_product(ta, tb, words):
    """recordings: on a host target, a product of two factors of which exactly
    one is ufix1 with fraction 0 (a plain 0 / 1) takes the other factor's type as its internal-rule type, no bits
    added. Both ufix1: the general rule (uint8 on a host, ufix2 on ASIC/FPGA). Returns None when the rule does not
    apply. ASIC/FPGA the same (17 recordings)."""
    if (tuple(ta) == UFIX1) == (tuple(tb) == UFIX1):   # on ASIC/FPGA too
        return None
    return tuple(tb) if tuple(ta) == UFIX1 else tuple(ta)


def gain_inherit_out(si, wi, fi_, pt, words):
    """Output type of a Gain with the internal rule: the full-precision product, padded to a native size when it fits
    one -- except for a one-bit factor (one_bit_product): a ufix1 input gives the parameter type (sfix11_E26, not
    sfix16), a ufix1 parameter gives the input type (ufix17_En8; recordings), both
    ufix1 give uint8."""
    t = one_bit_product((si, wi, fi_), tuple(pt), words)
    if t is not None:
        return t
    return ((1 if (si or pt[0]) else 0), _native_container(wi + pt[1], words), fi_ + pt[2])


def _blk_gain(case):
    p = case["params"]
    words = _hw_words(case)
    si, wi, fi_, ni = _ins(case)[0]
    g = float(p["Gain"])
    pt = parse_type(p["ParamDataTypeStr"])
    if pt is None:  # internal rule: input signedness and word length, best precision
        pt = (si, wi, gain_param_fl(g, si, wi, _asic(case.get("hw")), p["OutDataTypeStr"] == _INHERIT_RULE))
    npar = _quant_param(g, *pt)
    full_s, full_w, full_f = (1 if (si or pt[0]) else 0), wi + pt[1], fi_ + pt[2]
    ot = p["OutDataTypeStr"]
    t = parse_type(ot)
    if t is None:
        if ot == _SAME_INPUT:
            t = (si, wi, fi_)
        else:  # internal rule (gain_inherit_out)
            t = gain_inherit_out(si, wi, fi_, pt, words)
    s, w, f = t
    return _out(s, w, f, _cast(ni * npar, full_f, s, w, f, _blk_fm(p)))


def _blk_dtc(case):
    p = case["params"]
    s, w, f = parse_type(p["OutDataTypeStr"])
    si, wi, fi_, ni = _ins(case)[0]
    fm = _blk_fm(p)
    if p.get("ConvertRealWorld", "").startswith("Stored Integer"):
        return _out(s, w, f, _overflow(ni, s, w, fm["om"]))
    return _out(s, w, f, _cast(ni, fi_, s, w, f, fm))


def _container32(w, words):
    """Native container up to the long word; wider words round up to a multiple of it (
    33..64 -> 64, 65..96 -> 96, ... 260 -> 288)."""
    if not words:
        return w
    big = words[-1]
    return _native_container(w, words) if w <= big else -(-w // big) * big


def _sum_out_rule(types, words):
    """Sum output 'internal rule': the widest input word (native container; wider than the long word: next
    multiple of it), range of the widest-range input."""
    s = 1 if any(t[0] for t in types) else 0
    ib = max(_int_bits(t[0], t[1], t[2], s) for t in types)
    w = _container32(max(t[1] for t in types), words)
    return s, w, min(max(t[2] for t in types), w - ib)


SUM_ACC_RULE = "readout"  # "readout": fitted to the direct accumulator readout; "legacy": §25.3


def _sum_range_bits(types, signs, s, f):
    """Total bits (at fraction length f) of the smallest type of signedness s that holds every possible sum."""
    lo = hi = 0
    for (ts, tw, tf), sg in zip(types, signs):
        a, b = ((-(1 << (tw - 1)), (1 << (tw - 1)) - 1) if ts else (0, (1 << tw) - 1))
        a, b = a << (f - tf), b << (f - tf)
        lo, hi = (lo + a, hi + b) if sg == "+" else (lo - b, hi - a)
    if s:
        return max(hi.bit_length(), (-lo - 1).bit_length() if lo < 0 else 0) + 1
    return max(hi.bit_length(), 1)


def _sum_acc_container(n, big):
    """Container of the exact sum: its width rounded up to a multiple of the long word, at least two long
    words (inputs <= 32 bits: always 64; recordings has wider ones)."""
    return max(2 * big, -(-n // big) * big)


def _sum_acc_rule(types, signs, out_t, sat, words):
    """Sum accumulator 'internal rule' (fitted to recordings,
    1512/1512 types). I = integer bits of the exact range of the sum."""
    if SUM_ACC_RULE == "legacy":
        return _sum_acc_rule_legacy(types, signs, out_t, sat, words)
    s = 1 if (any(t[0] for t in types) or "-" in signs) else 0
    f = max(t[2] for t in types)
    if "+" not in signs:  # no '+' input: the accumulator starts from 0 and the output LSB counts too
        f = max(f, out_t[2])
    n = _sum_range_bits(types, signs, s, f)
    if not words:   # ASIC/FPGA: the exact range of the sum
        if len(types) == 1 and n < 2:
            # recordings (108 cells): a one-input Sum keeps its input's type, except that a one-bit
            # input becomes two bits (ufix1_En100 -> ufix2_En100, recordings; ufix1 / ufix1_En10 -> ufix2 / _En10)
            n = 2
        return s, n, f
    big = words[-1]
    if n <= big:
        # the exact type fits the long word: the accumulator IS the long word (one-input
        # '-' of ufix1_E2 -> sfix32_En5, '+-' of sfix4_En2 / ufix8 -> sfix32_En2; every recorded host accumulator is
        # 32 bits or wider). The smallest native container (8 / 16 bits) was never recorded: recordings's -127
        return s, big, f
    I = n - f
    W = out_t[1] if out_t[1] > big else big
    fl_o = min(out_t[2], f) if "+" in signs else out_t[2]
    C = _sum_acc_container(n, big)
    if sat:
        other = f - (C - W)
    elif out_t[1] > big:
        # wrapping, output wider than the long word: the accumulator is the output-sized container (two long words
        # at least) with fraction min(f, W - I) shifted down by the container bits beyond W. At W = 48 this is
        # min(f - 16, big - I), the form fitted to the readouts (W = 33 .. 47: no readout where it beats the output
        # fraction). recordings: W = 33 .. 64 (recordings and 3).
        # W > 2 long words (sum_acc_b3, recordings graphs): an exact sum that fits two long words keeps its
        # fraction f; a wider one takes the same form; an integer range wider than two words is refused where it
        # matters (sum_acc_unobserved)
        # the container: two long words, else W rounded up to a long word -- up to three long words; beyond that
        # (97 .. 128 bits) the output word itself (with W = 100 .. 124
        # the 128-bit container's shift kept too few bits -- 1 LSB under Zero rounding -- while 68 .. 96 need 96)
        # above three long words the input fraction f is kept unshifted
        # when it is the smaller term (f < W - I, sum_acc_b5); when the range term decides (W - I <= f) the
        # 32-bit-rounded container's shift applies as below 96 bits (W = 100 with I = 40: 32, not 60)
        C32 = max(2 * big, -(-W // big) * big)
        if W > 2 * big and n <= 2 * big:
            other = f
        elif W <= 3 * big:
            other = min(f, W - I) - (C32 - W)
        else:
            # recordings (recordings with the feasible accumulator fractions of sum_acc_b7 / b5): over
            # three long words the input fraction is kept while the exact sum fits three long words (n <= 96: 7-14
            # needs >= 43 of f = 52, 6-09 >= 47 of 64); a wider exact sum takes the same shifted form as below 96 bits
            # (7-06 needs >= 48 = 64 - 16, shift-w100 <= 32 = 60 - 28, edge-w108 <= 46: 44; the earlier "f < W - I"
            # split gave edge-w108 En64, which Simulink contradicts)
            other = f if n <= 3 * big else min(f, W - I) - (C32 - W)
    else:
        other = W - I
    fl = fl_o if other is None else max(fl_o, other)
    out_wins = fl == out_t[2] and (other is None or out_t[2] > other)
    sign = 0 if s == 0 else (out_t[0] if (out_t[1] >= big and out_wins) else 1)
    return sign, W, fl


def sum_acc_unobserved(types, signs, out_t, sat, words, rm=None):
    """Wrapping Sum, accumulator by the internal rule, output wider than two long words, exact sum wider than two long
    words: the regions the recordings do not support are refused rather than guessed.
      W = 65 .. 96 (container 96, recordings): refused when the integer range of the
        sum exceeds two long words and the rule keeps fewer fraction bits than the inputs (sum_acc_b3: Simulink keeps
        more there).
      W > 96 (sum_acc_b5, sum_acc_b7): with an integer range beyond two long words the earlier
        refusal stands (the fraction computed with the 32-bit-rounded container throughout, as in earlier versions: 672
        such cells had been opened without a recording)."""
    if not words or not isinstance(out_t, tuple) or out_t[1] <= 2 * words[-1]:
        return None
    big = words[-1]
    W = out_t[1]
    sg = "".join(signs)
    if sat and not (sg == "++" and len(types) == 2 and rm == "Zero" and W <= 128):
        # saturating Sums are recorded for '++' of two inputs, Zero rounding, W = 72 ..
        # 128 (the rule's fraction is in every feasible interval, exactly 36 at W = 100 and 48 at W = 112); other
        # saturating shapes over two long words are not recorded
        return ("Sum accumulator by the internal rule with a %d-bit saturating output (over two long words, signs %s, "
                "rounding %s): only '++' of two inputs with Zero rounding up to 128 bits is recorded"
                "; not supported" % (W, sg, rm))
    s = 1 if (any(t[0] for t in types) or "-" in signs) else 0
    f = max(t[2] for t in types)
    fl_o = min(out_t[2], f) if "+" in signs else out_t[2]
    if "+" not in signs:
        f = max(f, out_t[2])
    n = _sum_range_bits(types, signs, s, f)
    if n <= 2 * big:
        return None
    I = n - f
    where = ("Sum accumulator by the internal rule with a %d-bit %s output, a %d-bit integer range and input "
             "fraction %d" % (W, "saturating" if sat else "wrapping", I, f))
    # wrapping over three long words the input fraction
    # is kept at n = 92 (W = 108) and shifted at n = 100 (W = 108) and 104 (W = 112, which rules out "keep f while
    # n + 4 < W"); no recorded wrapping cell lies between, so the rule's "n <= 96" is not pinned for n = 93 .. 99 --
    # refused there when keeping f would give another fraction (recordings pins the saturating form
    # on both sides of the container step)
    if not sat and W > 3 * big and 93 <= n <= 99 and \
            max(fl_o, f) != max(fl_o, min(f, W - I) - (-(-W // big) * big - W)):
        return where + ": with a %d-bit exact sum the recordings do not pin whether the input fraction is kept " \
                       "(kept at 92, shifted at 100); not supported" % n
    if W > 3 * big:
        # over three long words only '++' of two inputs, Zero / Floor rounding, wrapping is recorded
        # (sum_acc_b5, sum_acc_b7, sum_wide_acc); recordings has four other shapes
        # ('+-', '-++', '+-+'), which still match with the accumulator fraction moved by +-4 -- they pin nothing
        # recordings (recordings = 100 / 128): '+-', '-+', '++-', '+++' with Zero rounding, and '++' of two
        # inputs with Ceiling / Convergent / Round, all wrapping -- the rule's fraction in every feasible interval
        # (Round exactly 32 at W = 100); Nearest could not be pinned, other combinations are not recorded
        recorded = (sg == "++" and len(types) == 2 and rm in ("Zero", "Floor")) or \
            (W <= 128 and not sat and rm == "Zero" and sg in ("+-", "-+", "++-", "+++")) or \
            (W <= 128 and not sat and sg == "++" and len(types) == 2 and rm in ("Ceiling", "Convergent", "Round"))
        if not recorded:
            return where + ": over three long words the recorded shapes are '++' of two inputs (Zero / Floor; up to " \
                           "128 bits also Ceiling / Convergent / Round) and up to 128 bits '+-', '-+', '++-', '+++' " \
                           "with Zero rounding (signs %s, rounding %s); not supported" % (sg, rm)
        if I > 2 * big:
            C = -(-W // big) * big
            if max(fl_o, min(f, W - I) - (C - W)) < f:
                return where + ": an integer range beyond two long words with an output this wide is not recorded; " \
                               "not supported"
        return None
    if I > 2 * big and _sum_acc_rule(types, signs, out_t, sat, words)[2] < f:
        return where + ': the rule keeps fewer fraction bits than the inputs, Simulink keeps more' \
                       "; not supported"
    return None


def _sum_acc_rule_legacy(types, signs, out_t, sat, words):
    """Sum accumulator 'internal rule' (fitted to recordings; the >native branch is only partly pinned down)."""
    s = 1 if (any(t[0] for t in types) or "-" in signs) else 0
    ib = max(_int_bits(t[0], t[1], t[2], s) for t in types) + (len(types) - 1).bit_length()
    f = max(t[2] for t in types)
    if not words:
        return s, ib + f, f
    big = words[-1]
    if ib + f <= big:  # ideal type fits a native word
        return s, _native_container(ib + f, words), f
    keep_out = min(out_t[2], f)  # never finer than the inputs, align to the output LSB
    ow = out_t[1]
    if ow > big:  # output itself wider than the native word
        return s, -(-ow // big) * big, keep_out
    if sat:
        return s, big, keep_out
    return s, big, max(big - ib, keep_out)


def sum_one_input_unobserved(p, ntypes, words, vec_reduce=False, max_wl=0):
    """A one-input Sum (Inputs '+', '1', '|+', or a single vector input summed over its elements) with an output or
    accumulator by the internal rule on ASIC/FPGA: recordings --
    Simulink types ufix1_En100 into ufix2_En100, the exact-range rule gives ufix1_En100; not recorded systematically
    (recordings requests recorded-sum1-*). Recorded and allowed since: '+' / '|+' with inputs up to 32 bits, vector sums of
    up to 8 elements of up to 32 bits; a one-input '-' and wider cases stay refused.
    Returns the reason or None."""
    if words or not (ntypes == 1 or vec_reduce):
        return None
    sg = str(p.get("Inputs", "+")).replace("|", "").strip()
    if not vec_reduce and sg in ("+", "1") and max_wl <= 32:
        # recordings (Inputs '+' and '|+', 1 .. 32-bit inputs, signed and unsigned): recorded; the
        # rule is _sum_acc_rule's exact range with a one-bit input widened to two bits
        return None
    if not vec_reduce and sg == "-" and max_wl <= 8:
        # recordings: one-input '-' of ufix1_E2 / ufix4_E1 / ufix8 / sfix8_En2
        # on ASIC/FPGA -- accumulator and output types (Fixed-Point Tool readouts) and values equal to the
        # exact-range rule; opened within the recorded input widths
        return None
    if vec_reduce and ntypes <= 8 and max_wl <= 32:
        # recordings (12 ASIC cells: widths 2 / 3 / 8, ufix1_En100 .. sfix32_En10): the exact range of
        # the elements' sum, as for a multi-input Sum -- types and values equal; opened within the recorded sizes
        return None
    if _INHERIT_RULE not in (p.get("OutDataTypeStr"), p.get("AccumDataTypeStr")):
        return None
    return ("one-input Sum%s with a type by the internal rule on ASIC/FPGA: Simulink adds a bit the exact-range rule "
            "does not, not recorded; not supported" % (" over a vector" if vec_reduce else ""))


def _sum_types(p, types, signs, fm, words):
    """(output, accumulator) types of a Sum. ASIC/FPGA (words == []; recordings outputs and 71/71
    accumulator readouts): the accumulator rule keeps the exact range of the sum, and the output internal rule is
    the accumulator type."""
    def resolve(text):
        t = parse_type(text)
        if t is not None:
            return t
        if text == _SAME_FIRST:
            return tuple(types[0])
        return None
    sat = fm["om"] == "Saturate"
    if not words and p["OutDataTypeStr"] == _INHERIT_RULE:
        placeholder = (1, 64, max(t[2] for t in types))   # only its fraction length is read (no '+' input)
        acc_t = resolve(p["AccumDataTypeStr"]) or _sum_acc_rule(types, signs, placeholder, sat, words)
        return acc_t, acc_t
    out_t = resolve(p["OutDataTypeStr"]) or _sum_out_rule(types, words)
    acc_t = resolve(p["AccumDataTypeStr"]) or _sum_acc_rule(types, signs, out_t, sat, words)
    return out_t, acc_t


def _blk_sum(case, n):
    p = case["params"]
    words = _hw_words(case)
    vals = _ins(case)
    types = [(v[0], v[1], v[2]) for v in vals]
    signs = str(p.get("Inputs", "+" * n)).replace("|", "").strip()
    if signs.isdigit():  # Simulink also accepts a port count, meaning all '+'
        signs = "+" * int(signs)
    fm = _blk_fm(p)

    def resolve(text):
        t = parse_type(text)
        if t is not None:
            return t
        if text == _SAME_FIRST:
            return types[0]
        return None

    out_t, acc_t = _sum_types(p, types, signs, fm, words)
    s, w, f = acc_t
    xs = [_cast(n_, f_, s, w, f, fm) for (_s, _w, f_, n_) in vals]
    # observed order: start from the first '+' input, then the others in port order
    first = signs.index("+") if "+" in signs else None
    acc = xs[first] if first is not None else 0
    for i, (sg, x) in enumerate(zip(signs, xs)):
        if i == first:
            continue
        acc = _overflow(acc + x if sg == "+" else acc - x, s, w, fm["om"])
    os_, ow, of = out_t
    return _out(os_, ow, of, _cast(acc, f, os_, ow, of, fm))


def _blk_product2(case):
    p = case["params"]
    words = _hw_words(case)
    (sa, wa, fa, na), (sb, wb, fb, nb) = _ins(case)
    full_s, full_w, full_f = (1 if (sa or sb) else 0), wa + wb, fa + fb
    ot = p["OutDataTypeStr"]
    t = parse_type(ot)
    if t is None:
        if ot == _SAME_FIRST:
            t = (sa, wa, fa)
        else:  # internal rule: same as Gain, one-bit factor included (one_bit_product)
            t = one_bit_product((sa, wa, fa), (sb, wb, fb), words) or (full_s, _native_container(full_w, words), full_f)
    s, w, f = t
    return _out(s, w, f, _cast(na * nb, full_f, s, w, f, _blk_fm(p)))


def _blk_saturation(case):
    p = case["params"]
    s, w, f, n = _ins(case)[0]
    hi = _quant_param(float(p["UpperLimit"]), s, w, f)
    lo = _quant_param(float(p["LowerLimit"]), s, w, f)
    return _out(s, w, f, min(max(n, lo), hi))


_RELOPS = {"==": lambda a, b: a == b, "~=": lambda a, b: a != b, "<": lambda a, b: a < b,
           "<=": lambda a, b: a <= b, ">": lambda a, b: a > b, ">=": lambda a, b: a >= b}


RELOP_SPAN_LIMIT = 31  # fraction lengths further apart than this (both inputs <= 32 bits) -> lossy compare


def relop_lossy_type(ta, tb, words):
    """The type both inputs of a Relational Operator are cast to first (relop_cast_fm), or None for an exact
    comparison. On the host, when both inputs fit 32 bits and their fraction lengths differ by
    32 or more, the input with the greater range widened to 32 bits with its MSB kept -- an unsigned such type
    clamps negative values to 0. ASIC/FPGA (words == []): exact (recordings system grid,
    5 designs with spans of 34 .. 36 compare exactly)."""
    if not words or max(ta[1], tb[1]) > 32:
        return None
    span = abs(ta[2] - tb[2])
    fine = max((ta, tb), key=lambda t: t[2])
    # recordings (recordings, recordings): a span
    # of exactly 31 is lossy too when both inputs are signed and the finer one is 32 bits (int8 vs sfix32_En31 '=='
    # gives [1, 0] for [1, -1]); ufix32_En31 or sfix31_En31 against int8 stay exact (21 / 21 such pairs lossy,
    # 658 other span <= 31 pairs exact)
    if span <= RELOP_SPAN_LIMIT and not (span == 31 and ta[0] and tb[0] and fine[1] == 32):
        return None
    g = max((ta, tb), key=lambda t: t[1] - t[2])
    return (g[0], 32, 32 - (g[1] - g[2]))


def relop_cast_fm(p):
    """Rounding of the lossy Relational Operator cast (
    recordings): the block's RndMeth, Simplest as Floor, saturating (the engine had always used Floor:
    Ceiling, Convergent, Nearest, Round and Zero compared wrongly)."""
    rm = str(p.get("RndMeth", "Floor"))
    return {"rm": "Floor" if rm == "Simplest" else rm, "om": "Saturate"}


RELOP_ASIC_IDEAL_MAX = 248   # exact at 184 / 187 / 248


def relop_wide_unobserved(ta, tb, words):
    """Return a refusal reason above the recorded 248-bit ASIC/FPGA comparison width, or None."""
    if words:
        return None
    s = 1 if (ta[0] or tb[0]) else 0
    f = max(ta[2], tb[2])
    ideal = max(t[1] - t[2] + (1 if s and not t[0] else 0) for t in (ta, tb)) + f
    if ideal > RELOP_ASIC_IDEAL_MAX:
        return ("Relational Operator on ASIC/FPGA with an ideal common type of %d bits: exact comparison is recorded up "
                "to %d bits; not supported" % (ideal, RELOP_ASIC_IDEAL_MAX))
    return None


def _blk_relop(case):
    """Exact comparison, except where relop_lossy_type gives a cast type (host, fraction spans of 32 or more)."""
    (sa, wa, fa, na), (sb, wb, fb, nb) = _ins(case)
    op = _RELOPS[case["params"]["Operator"]]
    c = relop_lossy_type((sa, wa, fa), (sb, wb, fb), _hw_words(case))
    if c is not None:
        cs, cw, cf = c
        fm = relop_cast_fm(case["params"])
        return {"bool": int(op(_cast(na, fa, cs, cw, cf, fm), _cast(nb, fb, cs, cw, cf, fm)))}
    f = max(fa, fb)
    return {"bool": int(op(na << (f - fa), nb << (f - fb)))}


def _blk_abs(case):
    s, w, f, n = _ins(case)[0]
    return _out(s, w, f, _overflow(abs(n), s, w, _blk_fm(case["params"])["om"]))


def _blk_uminus(case):
    s, w, f, n = _ins(case)[0]
    return _out(s, w, f, _overflow(-n, s, w, _blk_fm(case["params"])["om"]))


def _minmax_compare_type(types, out_t, words):
    """Type the MinMax inputs are converted to before comparing (fitted to recordings).
    Inputs wider than the long word: the container is the widest input word rounded up to
    a multiple of the long word; if the ideal common type fits it the comparison is exact, else the
    integer bits are kept (34 graphs). Inputs within the long word: the branches below."""
    s = 1 if any(t[0] for t in types) else 0
    ib = max(_int_bits(t[0], t[1], t[2], s) for t in types)
    f = max(t[2] for t in types)
    if not words:
        return s, ib + f, f
    big = words[-1]
    if ib + f <= big:  # ideal common type fits a native word
        return s, _native_container(ib + f, words), f
    wc = _container32(max(t[1] for t in types), words)
    if wc > big:
        if ib + f <= wc:
            return (s, wc, f)
        # Simulink compares exactly while the ideal common
        # type is at most 128 bits (sweep 57 .. 128 bits, every fraction bit the output shows is kept; the old
        # "keep the integer bits in the input container" cut one LSB: sfix53_En6 vs ufix48_En32 -> ufix53_En18);
        # cannot tell the two apart (the cut was invisible in all 35 graphs)
        if ib + f <= 128:
            return (s, ib + f, f)
        # Wider common types are screened by minmax_wide_unobserved.
        return (s, wc, min(f, wc - ib))
    if out_t[1] > big:
        # inputs within the long word, output wider: Simulink
        # compares exactly; the output-sized container lost the integer bits when the ideal type was wider than it
        # (sfix32_En31 vs sfix32_E20 into fixdt(1,64,20) wrapped). Rounding is monotone, so where nothing overflows the
        # exact comparison and the old container give the same result (recordings unchanged)
        if ib + f <= 128:
            return s, ib + f, f
        return s, -(-out_t[1] // big) * big, min(out_t[2], f)   # refused where it can overflow (minmax_wide_unobserved)
    # otherwise: the input with the greatest range, widened to the native word (keep its MSB);
    # a signed one is additionally not made finer than the output
    g = max(types, key=lambda t: (t[1] - t[2], -t[0]))  # range tie: the unsigned input (port-independent)
    fl = big - (g[1] - g[2])
    if s == 0 and g[1] < big and out_t[2] < fl:
        # recordings (recordings mmdr-a / c): every input unsigned and the
        # output coarser than this comparison type -- Simulink rounds the selected input once, from its own value to
        # the output (Convergent and Nearest recorded); rounding into the comparison type first and then into the
        # output gave other values. The exact ideal common type gives that single rounding; recordings
        # (the 97 modules of this class, Round / Nearest formerly refused as contradicting the rule) and recordings
        # agree with it (all modes). Whether Simulink's selection is exact where inputs tie only in the narrower type
        # is not visible in any recording (both choices give the same values there)
        return (0, ib + f, f)
    if g[1] >= big:
        # recordingsb (ufix13_En13 vs sfix32_En2 into sfix5,
        # Convergent -- Simulink 0, the capped type gave 1): an input that already fills the long word is the
        # comparison type as it is, not made coarser down to the output; recordings
        # agree (64 + 256 cases newly equal, none lost)
        return g[0], big, fl
    return g[0], big, (min(fl, out_t[2]) if g[0] else fl)


def minmax_mixed_sign_many(types, out_t, words, rm, explicit_out=True):
    """a host MinMax with
    three or more inputs mixing unsigned and signed types was answered with one rounding from the exact minimum, while
    Simulink rounds into the running pair's type first (fixdt(1,12,6), fixdt(0,10,6), fixdt(1,16,10) into
    fixdt(1,16,4), Nearest: Simulink 42, engine 41). Two Floor roundings equal one, so only a rounding other than Floor
    is refused -- the same gate as the validated domain; recordings
    (mixed signs, Floor) stay answered. A reason string, or None."""
    if not words or len(types) < 3 or any(len(t) != 3 for t in types):
        return None
    if any(t[0] == 0 for t in types) and any(t[0] == 1 for t in types) and str(rm) != "Floor":
        return ("MinMax with %d inputs mixing unsigned and signed types (%s) and RndMeth %s: the comparison order and "
                "roundings are not recorded; not supported" % (len(types), ", ".join(_tname(t) for t in types), rm))
    return None


def minmax_capped_wrap(types, out_t, words, om):
    """recordings in Simulink R2026b: a signed widest-range input makes the comparison type
    the long word with the fraction cut to the output's (the last branch of _minmax_compare_type); there, with
    saturation off, an input whose range exceeds the output type's (ufix32_En17 near 2^15 into int16, Convergent)
    was compared without wrapping while Simulink's minimum shows the input wrapped into the output type first
    (-32768 at every step). Which comparison Simulink uses where an input wraps on the way to the output is not
    recorded for this branch -- refused there. A reason string, or None."""
    if om == "Saturate" or not words or not isinstance(out_t, tuple) or any(len(t) != 3 for t in types):
        return None
    try:
        cs, cw, cf = _minmax_compare_type(types, out_t, words)
    except (ValueError, TypeError, IndexError):
        return None
    big = words[-1]
    g = max(types, key=lambda t: (t[1] - t[2], -t[0]))
    if not (g[0] and cw == big and cf == out_t[2] and cf < big - (g[1] - g[2]) and cw > out_t[1]):
        return None

    def rng(t):
        sc = Fraction(1, 2 ** t[2]) if t[2] >= 0 else Fraction(2 ** -t[2])
        return ((-(1 << (t[1] - 1)) if t[0] else 0) * sc, ((1 << (t[1] - t[0])) - 1) * sc)
    # the recorded designs of this branch
    # have every input inside the range of the widest-range input g; the finding's ufix32_En17 reaches 32767.99,
    # above g = int16 -- only that is refused (an input exceeding the output alone is recorded equal)
    # also has an sfix25_En20 input beyond g = sfix4_E1 that matches unrounded (fraction = the
    # comparison fraction); the finding's input is rounded into the comparison type (17 -> 0 fraction bits) --
    # only an input that is rounded on the way in and can exceed g's range is refused
    glo, ghi = rng(g)
    for t in types:
        lo, hi = rng(t)
        if t[2] > cf and (hi > ghi or lo < glo):
            return (('MinMax with saturation off whose input %s can exceed the range of the '
                "widest-range input %s (comparison type %s, output %s): Simulink's comparison "
                'there is not recorded (in the recorded case the input wrapped as in a %d-bit '
                'type); set SaturateOnIntegerOverflow on or give the inputs types whose ranges nest')
                    % (_tstr_short(t), _tstr_short(g), _tstr_short((cs, cw, cf)), _tstr_short(out_t), g[1]))
    return None


def minmax_wide_unobserved(types, out_t, words):
    """MinMax supports input, output and ideal common types up to 128 bits on both profiles."""
    if any(len(t) != 3 for t in types) or not isinstance(out_t, tuple):   # host and ASIC/FPGA alike
        return None
    s = 1 if any(t[0] for t in types) else 0
    ib = max(_int_bits(t[0], t[1], t[2], s) for t in types)
    f = max(t[2] for t in types)
    w = max([t[1] for t in types] + [out_t[1]])
    if w > 128:
        return "MinMax supports input and output types up to 128 bits (requested %d bits)" % w
    if ib + f > 128:
        return "MinMax supports an ideal common type up to 128 bits (requested %d bits)" % (ib + f)
    return None


def minmax_unresolved(types, out_t, words, rm):
    """Formerly refused two classes of MinMax (22 modules contradicted the
    comparison-type rule, Round / Nearest, ideal common type over the long word, inputs and output within it):
      (a) every input unsigned, output coarser than the comparison type (20 wrong) -- explained by a single rounding
          of the selected input, computed in _minmax_compare_type;
      (b) every input rounded in the comparison cast, output fraction equal to the comparison type's (2 wrong) --
          those 2 were unsigned modules of (a); with (a) fixed all 40 modules of (b) match.
    recordings now matches 69120 / 69120 with nothing refused. Kept as the hook callers check;
    returns None."""
    return None


# recordings (78 models, R2026b): host MinMax with 3 .. 5 inputs where one input
# has fewer fraction bits than the output. Simulink compares left to right in pairs and keeps the running result in
# an intermediate type: the first pair in the ideal common type (largest integer bits + largest fraction bits); every
# later pair in the running result's own type when its integer bits strictly exceed the next input's (the next
# input is rounded into it with the block's rounding mode), else in the ideal common type of the two. The selected
# value is cast to the output at the end. Fits every recorded step of the 76 host designs there and of every
# all-signed host design of recordings (the
# "Floor to 2^-8" of s16E8, s12E2, s20E12); designs with an unsigned input do not all fit, so the rule is used only
# on the recorded span below.
MM3C_OUT = (1, 16, 8)
MM3C_OTHERS = frozenset([(1, 16, 8), (1, 16, 10), (1, 16, 12), (1, 20, 12), (1, 12, 9)])
MM3C_COARSE = frozenset([(1, 16, 7), (1, 16, 4), (1, 16, 0)])
MM3C_RND = ("Floor", "Nearest", "Zero")
MM3C_WORDS = [8, 16, 32]


def minmax_chain_types(types):
    """Intermediate type of each pairwise step (input 2 .. n) of the host MinMax chain (see above)."""
    a = tuple(types[0])
    out = []
    for k, b in enumerate(types[1:], 1):
        s = 1 if (a[0] or b[0]) else 0
        ia, ib = _int_bits(a[0], a[1], a[2], s), _int_bits(b[0], b[1], b[2], s)
        if k > 1 and ia > ib:
            t = (s, ia + a[2], a[2])
        else:
            f = max(a[2], b[2])
            t = (s, max(ia, ib) + f, f)
        out.append(t)
        a = t
    return out


def minmax_coarse_span(types, out_t, words, rnd):
    """True when a host MinMax lies in the recorded span of recordings: 3 .. 5 inputs, output
    fixdt(1,16,8), exactly one input coarser than the output (fixdt(1,16,7), (1,16,4) or (1,16,0)) placed first,
    in the middle (index n // 2) or last, the others fixdt(1,16,8), (1,16,10), (1,16,12), (1,20,12) or (1,12,9),
    RndMeth Floor / Nearest / Zero (as written), saturation on or off, Intel x86-64 words (8, 16, 32)."""
    if list(words) != MM3C_WORDS or tuple(out_t) != MM3C_OUT or rnd not in MM3C_RND:
        return False
    ts = [tuple(t) for t in types]
    n = len(ts)
    if not 3 <= n <= 5:
        return False
    coarse = [k for k, t in enumerate(ts) if t in MM3C_COARSE]
    if len(coarse) != 1 or coarse[0] not in (0, n // 2, n - 1):
        return False
    return all(t in MM3C_OTHERS for k, t in enumerate(ts) if k != coarse[0])


def minmax_chain_eval(vals, fls, chain, fm, pick):
    """Run the pairwise chain on stored integers vals (fraction lengths fls); returns (value, fraction length)."""
    acc, af = vals[0], fls[0]
    for x, fx, t in zip(vals[1:], fls[1:], chain):
        acc = pick(_cast(acc, af, t[0], t[1], t[2], fm), _cast(x, fx, t[0], t[1], t[2], fm))
        af = t[2]
    return acc, af


def _blk_minmax(case):
    p = case["params"]
    words = _hw_words(case)
    vals = _ins(case)
    types = [(v[0], v[1], v[2]) for v in vals]
    ot = p["OutDataTypeStr"]
    out_t = parse_type(ot)
    if out_t is None:
        if ot == _SAME_FIRST:
            out_t = types[0]
        else:  # internal rule: widest input word (no padding), range of the widest-range input
            s = 1 if any(t[0] for t in types) else 0
            ib = max(_int_bits(t[0], t[1], t[2], s) for t in types)
            w = max(t[1] for t in types)
            out_t = (s, w, min(max(t[2] for t in types), w - ib))
    fm = _blk_fm(p)
    why = minmax_unresolved(types, out_t, words, fm["rm"]) or minmax_wide_unobserved(types, out_t, words) or \
        minmax_capped_wrap(types, out_t, words, fm["om"]) or minmax_mixed_sign_many(
        types, out_t, words, fm["rm"], not str(p.get("OutDataTypeStr", "Inherit")).strip().startswith("Inherit"))
    if why:
        raise ValueError(why)
    if words and minmax_coarse_span(types, out_t, words, p.get("RndMeth", "Floor")):
        chain = minmax_chain_types(types)
        y, yf = minmax_chain_eval([v[3] for v in vals], [v[2] for v in vals], chain, fm,
                                  min if p["Function"] == "min" else max)
        s, w, f = out_t
        return _out(s, w, f, _cast(y, yf, s, w, f, fm))
    cs, cw, cf = _minmax_compare_type(types, out_t, words)
    xs = [_cast(n, f, cs, cw, cf, fm) for (_s, _w, f, n) in vals]
    y = min(xs) if p["Function"] == "min" else max(xs)
    s, w, f = out_t
    return _out(s, w, f, _cast(y, cf, s, w, f, fm))


# ---------------------------------------------------------------- blocks
_NAMED_INT = {"int8": (1, 8, 0), "uint8": (0, 8, 0), "int16": (1, 16, 0), "uint16": (0, 16, 0),
              "int32": (1, 32, 0), "uint32": (0, 32, 0)}
_FI_RE = None


def _named_type(text):
    """'fixdt(s,w,f)' / built-in integer name -> (s, w, f); 'double' / 'single' / 'boolean' as strings; else None."""
    text = text.strip()
    t = _parse_fixdt(text)
    if t is not None:
        return t
    if text in _NAMED_INT:
        return _NAMED_INT[text]
    if text in ("double", "single", "boolean"):
        return text
    return None


def _parse_fi(text):
    """'fi(v,s,w,f)' -> (v, s, w, f) or None."""
    global _FI_RE
    if _FI_RE is None:
        import re
        _FI_RE = re.compile(r"^\s*fi\(\s*([^,]+?)\s*,\s*([01])\s*,\s*(\d+)\s*,\s*(-?\d+)\s*\)\s*$")
    m = _FI_RE.match(text)
    if not m:
        return None
    try:
        v = float(m.group(1))
    except ValueError:
        v = _double_expr(m.group(1))
        if v is None:
            raise
    return v, int(m.group(2)), int(m.group(3)), int(m.group(4))


def _double_expr(text):
    """The double MATLAB computes for plain arithmetic with sqrt and pi (for example, fi(1/sqrt(2),1,16,15)): numbers,
    + - * /, parentheses, sqrt(..), pi -- each operation correctly rounded in IEEE double, as in MATLAB. None for
    anything else (powers included: MATLAB's ^ is not guaranteed to round like Python's)."""
    import ast
    import math
    try:
        tree = ast.parse(text.strip(), mode="eval")
    except SyntaxError:
        return None

    def ev(n):
        if isinstance(n, ast.Expression):
            return ev(n.body)
        if isinstance(n, ast.Constant) and isinstance(n.value, (int, float)) and not isinstance(n.value, bool):
            return float(n.value)
        if isinstance(n, ast.Name) and n.id == "pi":
            return math.pi
        if isinstance(n, ast.UnaryOp) and isinstance(n.op, (ast.USub, ast.UAdd)):
            v = ev(n.operand)
            return -v if isinstance(n.op, ast.USub) else v
        if isinstance(n, ast.BinOp) and isinstance(n.op, (ast.Add, ast.Sub, ast.Mult, ast.Div)):
            a, b = ev(n.left), ev(n.right)
            if isinstance(n.op, ast.Div) and b == 0:
                raise ValueError
            return {ast.Add: a + b, ast.Sub: a - b, ast.Mult: a * b}.get(type(n.op)) if not \
                isinstance(n.op, ast.Div) else a / b
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == "sqrt" and len(n.args) == 1 \
                and not n.keywords:
            v = ev(n.args[0])
            if v < 0:
                raise ValueError
            return math.sqrt(v)
        raise ValueError
    try:
        return ev(tree)
    except (ValueError, RecursionError):
        return None


def _fmt_double(x):
    """MATLAB double rendered with 17 significant digits, trailing zeros dropped (C %.17g)."""
    return "%.17g" % x


_NEAREST_SAT = {"rm": "Nearest", "om": "Saturate"}


def _blk_constant(case):
    p = case["params"]
    v = p["Value"].strip()
    if str(p.get("OutDataTypeStr", "")).strip() == "boolean":   # v != 0
        return {"bool": 1 if float(v) != 0 else 0}
    fiv = _parse_fi(v)
    ot = p.get("OutDataTypeStr", "Inherit: Inherit from 'Constant value'")
    if fiv is not None:
        fs, fw, ff = fiv[1:]
        fn = _quant_param(fiv[0], fs, fw, ff)  # fi() construction: nearest, saturate
    if ot.startswith("Inherit"):
        if fiv is not None:
            return _out(fs, fw, ff, fn)
        return {"cls": "double", "num": _fmt_double(float(v))}
    t = _named_type(ot)
    if t == "double":
        return {"cls": "double", "num": _fmt_double(fn * 2.0 ** -ff if fiv is not None else float(v))}
    s, w, f = t
    if fiv is not None:
        return _out(s, w, f, _cast(fn, ff, s, w, f, _NEAREST_SAT))
    return _out(s, w, f, _quant_param(float(v), s, w, f))


# Stateful blocks (Delay, Discrete FIR): recordings show one case per time step and the harness
# evaluates cases one at a time in file order, so inputs of earlier steps of the same module are kept
# here (keyed by op/batch/block; a module restarts at step 0). Cases of a module must arrive in step order.
_HIST = {}


def _history(case):
    key = (case["op"], case["batch"], case["block"])
    h = _HIST.get(key)
    if case["step"] == 0 or h is None:
        h = _HIST[key] = []
    if len(h) != case["step"]:
        raise ValueError("stateful block %s: step %d arrived after %d steps" % (key, case["step"], len(h)))
    h.append([(0, 1, 0, int(v["bool"])) if isinstance(v, dict) and "bool" in v else _decode(v) for v in case["ins"]])
    return h


def _blk_delay(case):
    p = case["params"]
    h = _history(case)
    L = int(p.get("DelayLength", "1")) if p["BlockType"] == "Delay" else 1
    s, w, f, _ = h[-1][0]
    k = case["step"]
    if k >= L:
        return _out(s, w, f, h[k - L][0][3])
    return _out(s, w, f, _quant_param(float(p["InitialCondition"]), s, w, f))


def _port_shift(n, k, d, bp, s, w):
    """Shift Arithmetic, shift amount k from the port (
    recordings: every recorded step). The shift is
    saturated to int16; the right-shift amount a (Right: k > 0; Left: -k for k > 0; Bidirectional: k) is combined with
    the binary-point shift as a - BinPtShiftNumber in 16-bit two's complement. Where that wraps (or is -32768, whose
    negation wraps) the direction follows the wrapped sign: >= 0 or -32768 shifts right, else left. The stored
    integer is then shifted by |a| (left: wrapped to the word; right: arithmetic)."""
    k = min(max(k, -32768), 32767)
    a = (k if k > 0 else 0) if d == "Right" else (-k if k > 0 else 0) if d == "Left" else k
    if a == 0:
        return n
    m = a - bp
    if not -32768 < m <= 32767:
        wm = ((m + 32768) & 0xFFFF) - 32768
        a = abs(a) if (wm >= 0 or wm == -32768) else -abs(a)
    if a > 0:
        return n >> min(a, w + 1)
    return _overflow(n << min(-a, w + 1), s, w, "Wrap")   # every bit is out beyond the word


def _blk_shift(case):
    """Shift Arithmetic: stored integer shifted (left: wrap to the word, right: arithmetic/floor);
    the binary point moves by BinPtShiftNumber (fl_out = fl_in - BinPtShiftNumber).
    Bidirectional: positive BitShiftNumber shifts right, negative shifts left."""
    p = case["params"]
    s, w, f, n = _ins(case)[0]
    d = p["BitShiftDirection"]
    if str(p.get("BitShiftNumberSource", "Dialog")) == "Input port":
        # recordings (10 designs, 10 / 10): the shift from port 2 each step; Left / Right shift by k > 0
        # and pass the value for k <= 0; Bidirectional: k > 0 right, k < 0 left
        k = _ins(case)[1][3]
        if k > 2147483647:
            # recordings (uint32 port): Simulink
            # computes every earlier step and stops at the first port value of 2^31 or more (ShiftNonIntParam)
            raise SimDiagnostic("the shift value %d from the port is not an integer between -2147483648 and 2147483647: "
                                "Simulink stops the simulation (ShiftNonIntParam)" % k)
        if str(p.get("CheckOORBitShift", "off")).strip() == "on" and \
                str(p.get("DiagnosticForOORShift", "Warning")).strip() == "Error":
            # recordings (14 stops, 6 in-range runs): the net shift must
            # be 0 .. 31 (Left / Right) or -31 .. 31 (Bidirectional), for every data and shift-port type recorded;
            # outside, Simulink stops at that step (Simulink:blocks:ShiftOutOfRange)
            lo_ = -31 if d == "Bidirectional" else 0
            if not lo_ <= k <= 31:
                raise SimDiagnostic("the net shift value %d is out of range (%d .. 31): Simulink stops the simulation "
                                    "(DiagnosticForOORShift 'Error', ShiftOutOfRange)" % (k, lo_))
        rng = str(p.get("_ShiftRange", "")).split()
        if p.get("_ShiftGeneral"):
            pass
        elif rng:   # the recorded shift range of this setting (engine/msh_domain.json)
            if not int(rng[0]) <= k <= int(rng[1]):
                raise LutUnrecorded("Shift Arithmetic: a shift of %d from the port is outside the range answered "
                                    "for this setting (%s .. %s: the shifts whose recorded results follow this rule)"
                                    % (k, rng[0], rng[1]))
        elif abs(k) > 7 or abs(k) >= w:   # recorded shifts are -7 .. 7
            raise LutUnrecorded("Shift Arithmetic: a shift of %d from the port on a %d-bit word is not recorded "
                                "(recorded: -7 .. 7); record a design with this shift to open it" % (k, w))
        if p.get("_ShiftGeneral"):
            v = _port_shift(n, k, d, int(p.get("BinPtShiftNumber", "0")), s, w)
        elif (d == "Left" and k > 0) or (d == "Bidirectional" and k < 0):
            v = _overflow(n << min(abs(k), w + 1), s, w, "Wrap")   # every bit is out beyond the word
        elif (d == "Right" and k > 0) or (d == "Bidirectional" and k > 0):
            v = n >> min(k, w + 1)
        else:
            v = n
        return _out(s, w, f - int(p.get("BinPtShiftNumber", "0")), v)
    k = int(p["BitShiftNumber"])
    if d == "Left" or (d == "Bidirectional" and k < 0):
        v = _overflow(n << abs(k), s, w, "Wrap")
    else:
        v = n >> abs(k)
    return _out(s, w, f - int(p["BinPtShiftNumber"]), v)


def _truth(v):
    """Logical value of an input record: boolean as given, numbers nonzero = true."""
    if "bool" in v:
        return 1 if int(v["bool"]) else 0
    return 1 if _decode(v)[3] != 0 else 0


def _blk_logic(case):
    op = case["params"]["Operator"]
    xs = [_truth(v) for v in case["ins"]]
    if op == "AND":
        r = all(xs)
    elif op == "OR":
        r = any(xs)
    elif op == "NAND":
        r = not all(xs)
    elif op == "NOR":
        r = not any(xs)
    elif op == "XOR":
        r = sum(xs) % 2 == 1
    elif op == "NXOR":
        r = sum(xs) % 2 == 0
    elif op == "NOT":
        r = not xs[0]
    else:
        raise ValueError("logical operator %r" % op)
    return _bool_or_named(case["params"].get("OutDataTypeStr", "boolean"), r)


def _const_value(text):
    """Numeric literal, a built-in integer cast such as 'int8(-21)', or fi(v, s, w, f) (its quantised value:
    nearest, saturating; recordings, Compare To Constant fi(3,1,16,0)) -> float."""
    import re
    fiv = _parse_fi(text)
    if fiv is not None:
        return float(Fraction(_quant_param(fiv[0], *fiv[1:])) / Fraction(2) ** fiv[3])
    m = re.match(r"^\s*(u?int(?:8|16|32))\((.*)\)\s*$", text)
    if m:
        lo_hi = {"int8": (-128, 127), "uint8": (0, 255), "int16": (-32768, 32767), "uint16": (0, 65535),
                 "int32": (-2 ** 31, 2 ** 31 - 1), "uint32": (0, 2 ** 32 - 1)}[m.group(1)]
        v = _round_quot(*Fraction(float(m.group(2))).as_integer_ratio(), "Nearest")
        return float(max(lo_hi[0], min(lo_hi[1], v)))
    return float(text)


def _bool_or_named(out_text, r):
    """Logical result in the block's output type: 'boolean' -> {bool}, an integer type -> 0/1 in it."""
    t = _named_type(out_text)
    if t in (None, "boolean"):
        return {"bool": 1 if r else 0}
    if t == "double":
        return {"cls": "double", "num": "1" if r else "0"}
    s, w, f = t
    if f < 0:  # 1 is not representable; never observed (recordings only has boolean / uint8)
        raise ValueError("logical output type %s not supported" % out_text)
    return _out(s, w, f, (1 << f) if r else 0)


def _blk_cmpconst(case):
    """Compare To Constant: the constant is quantised to the input type (nearest, saturate) and the
    stored integers are compared."""
    p = case["params"]
    s, w, f, n = _ins(case)[0]
    c = _quant_param(_const_value(p["const"]), s, w, f)
    return _bool_or_named(p.get("OutDataTypeStr", "boolean"), _RELOPS[p["relop"]](n, c))


class SimDiagnostic(Exception):
    """A run-time condition on which Simulink stops the simulation with an error."""


def _switch_pick_type(types):
    """Switch / Multiport Switch output 'internal rule' (fitted to recordings): the data input with the
    greatest range (integer bits excluding the sign bit); among equal ranges the most precise one."""
    return max(types, key=lambda t: (t[1] - t[2] - t[0], t[2]))


def _bool_select(p, data, idx):
    """recordings (host and ASIC/FPGA alike): boolean data ports of Switch / Multiport Switch.
    All data ports boolean: the internal rule (and, for Switch, an explicit 'boolean') gives a boolean output that
    is the selected value. Boolean mixed with fixed point is recorded for one fixed-point type only, and boolean data
    into an explicit fixed-point output not at all: refused.
    recordings (70 designs, host and ASIC/FPGA): boolean data mixed
    with fixed-point data. Internal rule: the boolean counts as uint8 in _switch_pick_type (greatest integer range,
    then precision); when it wins the output is boolean (selected value != 0), else the fixed-point type (a boolean
    selected gives 0 / 1 cast into it). 'Inherit: Same as first input' (Switch): the first data port's type,
    boolean or not. Equal ranges between the boolean and a fixed-point type were recorded once only: refused."""
    ot = p["OutDataTypeStr"]
    if all("bool" in v for v in data):
        if ot not in (_INHERIT_RULE, "boolean", _SAME_FIRST):
            raise ValueError("boolean data ports into output type %s (not recorded)" % ot)
        return {"bool": 1 if int(data[idx]["bool"]) else 0}
    types = [(0, 8, 0) if "bool" in v else _decode(v)[:3] for v in data]
    if ot == _SAME_FIRST:
        win = 0
    elif ot == _INHERIT_RULE:
        keys = [(t[1] - t[2] - t[0], t[2]) for t in types]
        if keys.count(max(keys)) > 1:
            # a Switch with the boolean on u1 and a uint8-range type on u3 takes u3's type (as for fixed-point ties,
            # u3 first): recordings; the other way round
            # is not recorded
            if p.get("BlockType") == "Switch" or (len(data) == 2 and "bool" in data[0] and "bool" not in data[1]
                                                    and p.get("Criteria") is not None):
                win = 1
            else:
                raise ValueError("boolean and fixed-point data of equal range on a selection block (not recorded)")
        else:
            win = keys.index(max(keys))
    else:
        raise ValueError("boolean and fixed-point data ports into output type %s (not recorded)" % ot)
    v = data[idx]
    if "bool" in data[win]:
        return {"bool": int((int(v["bool"]) if "bool" in v else _decode(v)[3]) != 0)}
    s, w, f = types[win]
    if "bool" in v:
        return _out(s, w, f, _cast(1 if int(v["bool"]) else 0, 0, s, w, f, _blk_fm(p)))
    ds, dw, df, dn = _decode(v)
    return _out(s, w, f, _cast(dn, df, s, w, f, _blk_fm(p)))


def _blk_switch(case):
    p = case["params"]
    u1, u2, u3 = case["ins"]
    crit = p["Criteria"]
    if "bool" in u1 or "bool" in u3:
        if "bool" in u2:
            take = int(u2["bool"]) != 0
        elif crit == "u2 ~= 0":
            take = _decode(u2)[3] != 0
        else:
            # the threshold comparison does not depend on the data types
            s2, w2, f2, n2 = _decode(u2)
            q = _quant_param(float(p["Threshold"]), s2, w2, f2)
            take = n2 >= q if crit == "u2 >= Threshold" else n2 > q
        return _bool_select(p, [u1, u3], 0 if take else 1)
    t1, t3 = _decode(u1), _decode(u3)
    if "bool" in u2:
        take = int(u2["bool"]) != 0  # boolean control: passes u1 when true, whatever the criterion
    else:
        s2, w2, f2, n2 = _decode(u2)
        if crit == "u2 ~= 0":
            take = n2 != 0
        else:
            q = _quant_param(float(p["Threshold"]), s2, w2, f2)
            take = n2 >= q if crit == "u2 >= Threshold" else n2 > q
    ot = p["OutDataTypeStr"]
    if ot == _SAME_FIRST:
        t = t1[:3]
    elif ot == _INHERIT_RULE:
        # recordings (168 recorded tie blocks): equal range and fraction length but
        # different signedness (sfix(w+1) against ufix(w)) -- Simulink takes u3's type (a Multiport Switch keeps the
        # first data port); max() keeps the first of equals, so u3 goes first
        t = _switch_pick_type([t3[:3], t1[:3]])
    else:
        t = _named_type(ot)
    s, w, f = t
    src = t1 if take else t3
    return _out(s, w, f, _cast(src[3], src[2], s, w, f, _blk_fm(p)))


def _real_value(v):
    """Exact value of an input record: fixed point / integers as Fraction, double/single from the text."""
    if "bool" in v:
        return Fraction(int(v["bool"]))
    if "cls" in v and v["cls"] in ("double", "single"):
        return Fraction(float(v["num"]))
    s, w, f, n = _decode(v)
    return Fraction(n) / (Fraction(2) ** f)


def mps_indices(p):
    """The DataPortIndices of a Multiport Switch in 'Specify indices' order, as integers ({6,4,5} or [6 4 5])."""
    return [int(x) for x in str(p.get("DataPortIndices", "")).strip().strip("{}[]").replace(",", " ").split()]


def _blk_mpswitch(case):
    p = case["params"]
    ctrl, data = case["ins"][0], case["ins"][1:]
    spec = p["DataPortOrder"] == "Specify indices"
    n = int(p["Inputs"]) if not spec else len(mps_indices(p))
    extra = p["DataPortForDefault"] == "Additional data port"
    c = _real_value(ctrl)
    base = 0 if p["DataPortOrder"].startswith("Zero") else 1
    k = c - base  # 0-based data port
    if spec:
        # recordings (24 designs, 24 / 24 bit-exact): the port whose index equals the control, else the
        # default port (the last data port, or the additional one)
        ix = mps_indices(p)
        idx = ix.index(int(c)) if c.denominator == 1 and int(c) in ix else None
    else:
        idx = int(k) if (k.denominator == 1 and 0 <= k < n) else None
    if idx is not None:
        pass
    else:
        if (p.get("DiagnosticForDefault", "None") == "Error" or n == 1) and \
                not str(case.get("op", "")).startswith("probe:"):   # the engine's compile-time type probe (zeros)
            # Simulink stops here (MPSwitchControlInputOORIndexValue / ...RangeError)
            raise SimDiagnostic("Multiport Switch control %s is out of range of the %d data port(s): Simulink stops "
                                "with an error (DiagnosticForDefault Error, or a single data port)" % (c, n))
        idx = len(data) - 1  # last data port, or the additional default port
    if any("bool" in v for v in data):
        return _bool_select(p, data, idx)
    types = [_decode(v)[:3] for v in data]
    ot = p["OutDataTypeStr"]
    if ot == _INHERIT_RULE:
        t = _switch_pick_type(types)
    elif ot == _SAME_FIRST:
        t = types[0]
    else:
        t = _named_type(ot)
    s, w, f = t
    ds, dw, df, dn = _decode(data[idx])
    return _out(s, w, f, _cast(dn, df, s, w, f, _blk_fm(p)))


def _productn_type(ops, types, words):
    """Product/Divide output 'internal rule' (fitted to recordings): decided by the first two operands only (after the
    one-bit rule below).
    '**': full-precision product, padded to a native word if it fits one. One '/' among the first two:
    fl = fl(numerator) - fl(denominator), word = widest of the two padded to a native word.
    Single '/' (reciprocal): same signedness and word, fl = wl - fl_in - 1 - s."""
    if len(types) == 1:
        s, w, f = types[0]
        if ops == "/":
            return s, w, w - f - 1 - s
        return types[0]
    if set(ops) == {"*"}:
        # multiplications only: the ufix1 factors drop out, then the first two
        # remaining factors decide; with one left, its type; with none left, the general rule (uint8)
        rest = [t for t in types if tuple(t) != UFIX1]
        if len(rest) == 1:
            return tuple(rest[0])
        if rest:
            types = rest
    elif ops[0] == "*" and ops[1] == "*":
        # mixed operators: the first two operands by position, with the one-bit rule (U*A/B -> A)
        t = one_bit_product(types[0], types[1], words)
        if t is not None:
            return t
    (sa, wa, fa), (sb, wb, fb) = types[0], types[1]
    s = 1 if (sa or sb) else 0
    if ops[0] == "*" and ops[1] == "*":
        return s, _native_container(wa + wb, words), fa + fb
    if ops[0] == "*":  # a / b
        return s, _native_container(max(wa, wb), words), fa - fb
    if ops[1] == "*":  # b / a
        return s, _native_container(max(wa, wb), words), fb - fa
    raise ValueError("Product inputs %r: '//' not observed" % ops)


def _quant_fraction(x, s, w, f, fm):
    """Round an exact rational to (s, w, f) with the block's rounding and overflow."""
    num, den = x.numerator, x.denominator
    if f >= 0:
        num <<= f
    else:
        den <<= -f
    return _overflow(_round_quot(num, den, fm["rm"]), s, w, fm["om"])


PRODUCTN_STEPWISE = True  # True: every partial product/quotient is rounded to the output type


def _blk_productn(case):
    p = case["params"]
    words = _hw_words(case)
    vals = _ins(case)
    ops = str(p["Inputs"]).strip()
    if ops.isdigit():
        ops = "*" * int(ops)
    types = [v[:3] for v in vals]
    ot = p["OutDataTypeStr"]
    if ot == _INHERIT_RULE:
        t = _productn_type(ops, types, words)
    elif ot == _SAME_FIRST:
        t = types[0]
    else:
        t = _named_type(ot)
    s, w, f = t
    fm = _blk_fm(p)
    # 'Simplest' rounds a product down (Floor) but a quotient toward zero (observed in recordings)
    fmq = dict(fm, rm="Zero") if p.get("RndMeth") == "Simplest" else fm
    lo, hi = (-(1 << (w - 1)), (1 << (w - 1)) - 1) if s else (0, (1 << w) - 1)
    exact = [Fraction(v[3]) / (Fraction(2) ** v[2]) for v in vals]

    def divide(a, b):
        if b == 0:  # observed (1618 recordings cases): saturates by the sign of the dividend
            return None
        return a / b

    if ops == "/*":  # computed as u2 / u1 (no intermediate reciprocal)
        q = divide(exact[1], exact[0])
        if q is None:
            _div0()
            return _out(s, w, f, lo if exact[1] < 0 else hi)
        return _out(s, w, f, _quant_fraction(q, s, w, f, fmq))
    acc = exact[0] if ops[0] == "*" else None
    last = fm
    # a zero divisor gives the output type's extreme -- max, or min
    # for a negative dividend (0 / 0 is max) -- as that step's value, and the chain goes on with it; it used to end the
    # whole product there (the later operands were ignored)
    if ops[0] == "/":
        acc = divide(Fraction(1), exact[0])
        if acc is None:
            acc = Fraction(hi) / (Fraction(2) ** f)
        last = fmq
        if PRODUCTN_STEPWISE and len(ops) > 1:
            acc = Fraction(_quant_fraction(acc, s, w, f, fmq)) / (Fraction(2) ** f)
    for i in range(1, len(ops)):
        if ops[i] == "*":
            acc = acc * exact[i]
            last = fm
        else:
            q = divide(acc, exact[i])
            if q is None:
                _div0()
            acc = (Fraction(lo if acc < 0 else hi) / (Fraction(2) ** f)) if q is None else q
            last = fmq
        if PRODUCTN_STEPWISE and i < len(ops) - 1:
            acc = Fraction(_quant_fraction(acc, s, w, f, last)) / (Fraction(2) ** f)
    return _out(s, w, f, _quant_fraction(acc, s, w, f, last))


def _blk_bool2arith(case):
    """Boolean (Relational Operator output) into an arithmetic block (fitted to recordings):
    Sum and division: the boolean is a uint8 0/1. Multiplication: the boolean selects (b ? x: 0) and the
    'internal rule' output type is the other operand's type. Gain: output = b ? gain: 0; an inherited
    parameter type is 32 bits, best precision, signed only for a negative gain; the inherited output
    type is the parameter type."""
    p = case["params"]
    a, b = case["ins"][0], case["ins"][1]
    r = _blk_relop({"ins": [a, b], "params": {"Operator": p["relop_Operator"]}})["bool"]
    ins = list(case["ins"][2:])
    ins.insert(int(p["bool_port"]) - 1, {"bool": r})
    tp = {k: v for k, v in p.items() if k not in ("target", "bool_port", "relop_Operator", "relop_OutDataTypeStr")}
    return _arith_with_bool(p["target"], dict(case, ins=ins, params=tp))


def _blk_booldiv(case):
    """Boolean (Relational Operator output) as a Product divisor."""
    p = case["params"]
    extra = ("relop_pattern", "other_operand")
    return _blk_bool2arith(dict(case, params=dict({k: v for k, v in p.items() if k not in extra}, target="Product")))


def _arith_with_bool(tgt, case):
    """Gain / Sum / Product / DTC ('DTC') whose inputs include exactly one boolean record {"bool": r};
    rules of _blk_bool2arith."""
    tp = case["params"]
    bi = [i for i, v in enumerate(case["ins"]) if "bool" in v]
    if len(bi) != 1:
        raise ValueError("exactly one boolean input supported")
    r = int(case["ins"][bi[0]]["bool"])
    rest = [v for v in case["ins"] if "bool" not in v]
    ins = list(case["ins"])
    ins[bi[0]] = {"cls": "uint8", "num": str(r)}
    sub = dict(case, ins=ins, params=tp)
    if tgt == "Gain":
        g = float(tp["Gain"])
        pt = _named_type(tp.get("ParamDataTypeStr", _INHERIT_RULE))
        if pt is None:
            pt = bool_gain_param_type(g, _asic(case.get("hw")),
                                      tp.get("OutDataTypeStr", _INHERIT_RULE) == _INHERIT_RULE)
        npar = _quant_param(g, *pt)
        ot = tp.get("OutDataTypeStr", _INHERIT_RULE)
        t = pt if ot == _INHERIT_RULE else _named_type(ot)
        s_, w_, f_ = t
        return _out(s_, w_, f_, _cast(npar if r else 0, pt[2], s_, w_, f_, _blk_fm(tp)))
    if tgt == "Sum" and _asic(case.get("hw")) and tp.get("OutDataTypeStr", _INHERIT_RULE) == _INHERIT_RULE:
        # recordings (45 Sums, 60 / 60 with the Products): on ASIC/FPGA the internal rule counts the
        # boolean as uint8's range [0, 255]; the output holds the exact range of the signed sum at fraction length
        # max(fl, 0), signed when the fixed-point operand is or a '-' is present, in the shortest word
        signs = [c for c in str(tp["Inputs"]).replace("|", "") if c in "+-"]
        xs, xw, xf, xn = _decode(rest[0])
        fo = max(xf, 0)
        lo_x = Fraction(-(1 << (xw - 1)) if xs else 0, 1 << xf) if xf >= 0 else Fraction(-(1 << (xw - 1)) if xs else 0) * 2 ** -xf
        hi_x = Fraction((1 << (xw - xs)) - 1, 1 << xf) if xf >= 0 else Fraction((1 << (xw - xs)) - 1) * 2 ** -xf
        rngs = []
        for k, v in enumerate(case["ins"]):
            lo_, hi_ = (Fraction(0), Fraction(255)) if "bool" in v else (lo_x, hi_x)
            rngs.append((lo_, hi_) if signs[k] == "+" else (-hi_, -lo_))
        lo, hi = sum(r_[0] for r_ in rngs), sum(r_[1] for r_ in rngs)
        so = 1 if (xs or "-" in signs) else 0
        lo_i, hi_i = lo * 2 ** fo, hi * 2 ** fo
        wo = 1
        while not ((-(1 << (wo - 1)) <= lo_i and hi_i <= (1 << (wo - 1)) - 1) if so else hi_i <= (1 << wo) - 1):
            wo += 1
        val = sum((Fraction(r) if "bool" in v else Fraction(xn) / Fraction(2) ** xf) * (1 if signs[k] == "+" else -1)
                  for k, v in enumerate(case["ins"]))
        n = val * 2 ** fo
        return _out(so, wo, fo, int(n))
    if tgt == "Sum":
        return _blk_sum(sub, len(ins))
    if tgt == "Product":
        ops = str(tp["Inputs"]).strip()
        if ops == "**":
            x = _decode(rest[0])
            ot = tp["OutDataTypeStr"]
            if ot == _INHERIT_RULE:
                t = x[:3]
            elif ot == _SAME_FIRST:
                t = _decode(ins[0])[:3]
            else:
                t = _named_type(ot)
            s_, w_, f_ = t
            return _out(s_, w_, f_, _cast(x[3] if r else 0, x[2], s_, w_, f_, _blk_fm(tp)))
        return _blk_productn(sub)
    if tgt == "DTC":
        t = _named_type(tp["OutDataTypeStr"])
        if t in ("double", "single"):
            return {"cls": t, "num": _fmt_double(float(r))}
        s_, w_, f_ = t
        sub["params"] = dict(tp, OutDataTypeStr="fixdt(%d,%d,%d)" % t)
        return _blk_dtc(sub)
    raise ValueError("bool2arith target %r" % tgt)


def _dfir_coefs(p):
    return [float(x) for x in p["Coefficients"].strip().strip("[]").replace(",", " ").split()]


DFIR_STATE_CAST = False  # True would pass delayed inputs through the state type (they are not)


def _dfir_types(p, xt, coefs, words):
    """Discrete FIR (direct form) data types (fitted to recordings).
    coef  'Same word length as input': input signedness and word, the best precision every coefficient fits;
    prod  internal rule: full precision, padded to a native word if it fits one; 'Same as input';
    accum internal rule: product signedness and fraction length, word rounded up to a multiple of 32;
          'Same as product output' / 'Same as input'; state 'Same as accumulator'; output
          'Same as accumulator' / 'Same as input'."""
    ct = _named_type(p["CoefDataTypeStr"])
    if not isinstance(ct, tuple):
        # best precision that every coefficient fits (signed ranges are asymmetric, so per coefficient);
        # coefficients an unsigned type cannot represent at all do not constrain it
        s, w = xt[0], xt[1]
        # the Gain parameter's rule, negative edge included (a coefficient -1.00001 on sfix16 is
        # sfix16_En14 in Simulink -- it must fit before rounding -- not En15)
        fls = [gain_param_fl(c, s, w, not words) for c in coefs if c != 0]
        fls = [f for f in fls if f != _BEST_PREC_FL_UNREPRESENTABLE]
        ct = (s, w, min(fls) if fls else w - s)
    pt = _named_type(p["ProductDataTypeStr"])
    if not isinstance(pt, tuple):
        if p["ProductDataTypeStr"] == "Inherit: Same as input":
            pt = xt
        else:   # one-bit factor rule as Gain / Product (ufix1 input, Simulink on host and ASIC)
            pt = one_bit_product(xt, ct, words) or \
                (1 if (xt[0] or ct[0]) else 0, _native_container(xt[1] + ct[1], words), xt[2] + ct[2])
    at = _named_type(p["AccumDataTypeStr"])
    if not isinstance(at, tuple):
        a = p["AccumDataTypeStr"]
        if a == "Inherit: Same as product output":
            at = pt
        elif a == "Inherit: Same as input":
            at = xt
        elif not words:   # ASIC/FPGA (direct form): product word + ceil(log2(taps)) guard bits
            at = (pt[0], pt[1] + (len(coefs) - 1).bit_length(), pt[2])
        else:
            at = (pt[0], -(-pt[1] // 32) * 32, pt[2])
    st = _named_type(p["StateDataTypeStr"])
    if not isinstance(st, tuple):
        st = at if p["StateDataTypeStr"] == "Inherit: Same as accumulator" else xt
    ot = _named_type(p["OutDataTypeStr"])
    if not isinstance(ot, tuple):
        ot = at if p["OutDataTypeStr"] == "Inherit: Same as accumulator" else xt
    return ct, pt, at, st, ot


def _dfir_initial_states(p, xt, ntaps):
    """Initial delay-line contents as stored integers of the INPUT type (quantised
    like a parameter -- nearest, saturate; not to the state type), most recent first. A scalar fills every
    state; otherwise exactly ntaps - 1 values."""
    vals = _vec(p.get("InitialStates", "0"))
    if len(vals) == 1:
        vals = vals * max(ntaps - 1, 0)
    if len(vals) != max(ntaps - 1, 0):
        raise ValueError("Discrete FIR: InitialStates needs 1 or %d values" % (ntaps - 1))
    return [_quant_param(v, *xt) for v in vals]


# Discrete FIR transposed / symmetric / antisymmetric structures
# Fitted (355 dev modules): transposed partial sums live in the ACCUMULATOR type (the State type parameter does
# not apply to them; 103/103 explicit modules only match this way); tap-sum internal rule = input word + 1, capped
# at 32 bits; symmetric product internal rule = input word + 1 + coefficient word (native container) at the tap-sum
# fraction length; accumulator internal rule as the direct form (product word rounded up to a multiple of 32).
# recordings (input 40 bits; recordings: 33 / 40 / 48 bits): the cap applies up to 32-bit
# inputs only -- a wider input gets a tap sum one bit wider (no cap)
DFIR2 = {"tap_w": lambda w: min(w + 1, 32) if w <= 32 else w + 1, "acc_rule": lambda pt, xt: (pt[0], -(-pt[1] // 32) * 32, pt[2]),
         "t_state": "acc", "t_init": "state", "t_z_to_acc": True, "s_order": "pairs_then_centre", "s_centre_tapsum": False,
         "s_line_cast": False}


_DFIR2_STATE = {}   # (op, batch, block) -> (last step, partial sums after it)


def _dfir2_types(p, xt, coefs, words):
    """Coefficient / product / accumulator / state / output types (direct-form rules for the inherited
    ones), plus the tap-sum type of the symmetric structures."""
    ct, pt, at, st, ot = _dfir_types(p, xt, coefs, words)
    tt = _named_type(p.get("TapSumDataTypeStr", "Inherit: Same as input"))
    if not isinstance(tt, tuple):
        if p.get("TapSumDataTypeStr") == "Inherit: Same as input":
            tt = xt
        else:   # internal rule: one more integer bit than the input
            tt = (xt[0], DFIR2["tap_w"](xt[1]), xt[2])
    sym = p.get("FilterStructure") in ("Direct form symmetric", "Direct form antisymmetric")
    if sym and p["ProductDataTypeStr"] == _INHERIT_RULE:
        # symmetric structures: full precision of the tap sum times the coefficient, at the tap sum's fraction
        # length. With an inherited tap sum the word is input word + 1 + coefficient word (
        # also where the tap sum itself is capped at 32 bits); recordings: with an
        # explicit tap-sum type it is the tap sum's word + coefficient word
        tw = tt[1] if isinstance(_named_type(p.get("TapSumDataTypeStr", "Inherit: Same as input")), tuple) \
            else xt[1] + 1
        pt = (1 if (tt[0] or ct[0]) else 0, _native_container(tw + ct[1], words), tt[2] + ct[2])
        at, st, ot = _dfir_inherit_rest(p, xt, pt, at, st, ot)
    elif sym and p["AccumDataTypeStr"] == _INHERIT_RULE:
        at, st, ot = _dfir_inherit_rest(p, xt, pt, None, st, ot)
    return ct, pt, at, st, ot, tt


def _dfir_inherit_rest(p, xt, pt, at, st, ot):
    """Accumulator / state / output types that follow the product type (inherited)."""
    a = p["AccumDataTypeStr"]
    if a == "Inherit: Same as product output":
        at = pt
    elif a == _INHERIT_RULE:
        at = DFIR2["acc_rule"](pt, xt)
    if p["StateDataTypeStr"] == "Inherit: Same as accumulator":
        st = at
    if p["OutDataTypeStr"] == "Inherit: Same as accumulator":
        ot = at
    return at, st, ot


def _blk_dfir2(case):
    p = case["params"]
    struct = p["FilterStructure"]
    if struct == "Direct form":
        return _blk_dfir(case)
    h = _history(case)
    words = _hw_words(case)
    xt = h[-1][0][:3]
    coefs = _dfir_coefs(p)
    N = len(coefs)
    ct, pt, at, st, ot, tt = _dfir2_types(p, xt, coefs, words)
    fm = _blk_fm(p)
    cq = [_quant_param(c, *ct) for c in coefs]

    def prod_acc(n, f, c):
        return _cast(_cast(n * c, f + ct[2], pt[0], pt[1], pt[2], fm), pt[2], at[0], at[1], at[2], fm)

    def add(a, b):
        return _overflow(a + b, at[0], at[1], fm["om"])
    if struct == "Direct form transposed":
        if DFIR2["t_state"] == "acc":
            st = at
        vals = _vec(p.get("InitialStates", "0"))
        if len(vals) == 1:
            vals = vals * max(N - 1, 0)
        if len(vals) != max(N - 1, 0):
            raise ValueError("Discrete FIR: InitialStates needs 1 or %d values" % (N - 1))
        if DFIR2["t_init"] == "state":
            z = [_quant_param(v, *st) for v in vals]
        else:
            z = [_cast(_quant_param(v, *xt), xt[2], st[0], st[1], st[2], fm) for v in vals]
        y = None
        key = (case["op"], case["batch"], case["block"])
        k0 = case["step"]
        first = 0
        memo = _DFIR2_STATE.get(key)
        if k0 > 0 and memo is not None and memo[0] == k0 - 1:   # carry the partial sums from the previous step
            z, first = memo[1], k0
        for step in range(first, k0 + 1):
            _, _, fx, nx = h[step][0]
            a = [prod_acc(nx, fx, c) for c in cq]
            zt = [_cast(v, st[2], at[0], at[1], at[2], fm) for v in z]
            if N > 1:
                y = _cast(add(a[0], zt[0]), at[2], ot[0], ot[1], ot[2], fm)
            else:   # one coefficient -- the product goes to the output without the accumulator type
                y = _cast(_cast(nx * cq[0], fx + ct[2], pt[0], pt[1], pt[2], fm), pt[2], ot[0], ot[1], ot[2], fm)
            nz = []
            for k in range(1, N):
                v = add(a[k], zt[k]) if k < N - 1 else a[k]
                nz.append(_cast(v, at[2], st[0], st[1], st[2], fm))
            z = nz
        _DFIR2_STATE[key] = (k0, z)
        return _out(ot[0], ot[1], ot[2], y)
    # symmetric / antisymmetric: delay line of inputs, tap sums of mirrored taps, one product per pair
    sign = 1 if struct == "Direct form symmetric" else -1
    init = _dfir_initial_states(p, xt, N)
    k0 = case["step"]

    def tap(k):
        if k0 - k >= 0:
            sx, wx, fx, nx = h[k0 - k][0]
        else:
            nx, fx = init[k - k0 - 1], xt[2]
        if k > 0 and DFIR2["s_line_cast"]:
            nx, fx = _cast(nx, fx, st[0], st[1], st[2], fm), st[2]
        return nx, fx
    terms = []
    for k in range(N // 2):
        (n1, f1), (n2, f2) = tap(k), tap(N - 1 - k)
        v1 = _cast(n1, f1, tt[0], tt[1], tt[2], fm)
        v2 = _cast(n2, f2, tt[0], tt[1], tt[2], fm)
        ts = _overflow(v1 + sign * v2, tt[0], tt[1], fm["om"])
        terms.append(prod_acc(ts, tt[2], cq[k]))
    if N % 2:
        nm, fmid = tap(N // 2)
        if DFIR2["s_centre_tapsum"]:
            nm, fmid = _cast(nm, fmid, tt[0], tt[1], tt[2], fm), tt[2]
        centre = prod_acc(nm, fmid, cq[N // 2])
        terms = [centre] + terms if DFIR2["s_order"] == "centre_first" else terms + [centre]
    acc = terms[0]
    for v in terms[1:]:
        acc = add(acc, v)
    return _out(ot[0], ot[1], ot[2], _cast(acc, at[2], ot[0], ot[1], ot[2], fm))


def _blk_dfir(case):
    p = case["params"]
    h = _history(case)
    words = _hw_words(case)
    x0 = h[-1][0]
    xt = x0[:3]
    coefs = _dfir_coefs(p)
    ct, pt, at, st, ot = _dfir_types(p, xt, coefs, words)
    fm = _blk_fm(p)
    cq = [_quant_param(c, *ct) for c in coefs]
    init = _dfir_initial_states(p, xt, len(cq))
    k0 = case["step"]
    acc = None
    for k, c in enumerate(cq):
        if k0 - k >= 0:
            sx, wx, fx, nx = h[k0 - k][0]
            if k > 0 and DFIR_STATE_CAST:
                nx = _cast(nx, fx, st[0], st[1], st[2], fm)
                fx = st[2]
        else:
            nx, fx = init[k - k0 - 1], xt[2]
        prod = _cast(nx * c, fx + ct[2], pt[0], pt[1], pt[2], fm)
        if len(cq) == 1:
            # with one coefficient Simulink casts the product straight
            # to the output -- the accumulator type is not applied
            return _out(ot[0], ot[1], ot[2], _cast(prod, pt[2], ot[0], ot[1], ot[2], fm))
        v = _cast(prod, pt[2], at[0], at[1], at[2], fm)
        acc = v if acc is None else _overflow(acc + v, at[0], at[1], fm["om"])
    return _out(ot[0], ot[1], ot[2], _cast(acc, at[2], ot[0], ot[1], ot[2], fm))


# ---------------------------------------------------------------- recorded configurations 
def _blk_counter(case):
    """Counter Limited: 0..uplimit then back to 0, smallest unsigned integer holding uplimit;
    Counter Free-Running: modulo 2^NumBits in ufix(NumBits)."""
    p = case["params"]
    k = case["step"]
    if "Limited" in p["library_block"]:
        up = int(p["uplimit"])
        # uplimit 2^32 .. 2^53 gives uint64 (both hardware); above 2^53 (not exactly a
        # double) is not recorded (was a StopIteration)
        if not 0 <= up <= (1 << 53):
            raise ValueError("Counter Limited: upper limit outside 0 .. 2^53 is not supported (not recorded)")
        w = next(x for x in (8, 16, 32, 64) if up < (1 << x))
        return _out(0, w, 0, k % (up + 1))
    n = int(p["NumBits"])
    return _out(0, n, 0, k % (1 << n))


def _blk_detect(case):
    """Detect Change / Increase / Decrease: u[k] against u[k-1]; at k = 0 against the initial value
    quantised to the input type (nearest, saturate). Output boolean or uint8 0/1."""
    p = case["params"]
    h = _history(case)
    s, w, f, n = h[-1][0]
    k = case["step"]
    prev = h[k - 1][0][3] if k > 0 else _quant_param(float(p["vinit"]), s, w, f)
    if "Increase" in p["library_block"]:
        r = n > prev
    elif "Decrease" in p["library_block"]:
        r = n < prev
    else:
        r = n != prev
    return _bool_or_named(p.get("OutDataTypeStr", "boolean"), r)


def _blk_signround(case):
    """Sign: -1/0/+1 as a signed integer of the input word length (fl 0; built-in integer inputs keep their
    class), saturating to the type. Rounding Function (R2026b accepts double/single and integers only):
    floor / ceil / fix / round (half away from zero) on the value, same class as the input."""
    import math
    p = case["params"]
    v = case["ins"][0]
    if p["BlockType"] == "Rounding":
        op = p["Operator"]
        if "cls" in v and v["cls"] in ("double", "single"):
            x = float(v["num"])
            r = {"floor": math.floor, "ceil": math.ceil, "fix": math.trunc,
                 "round": lambda t: math.copysign(math.floor(abs(t) + 0.5), t)}[op](x)
            r = float(r)
            if r == 0:
                r = math.copysign(0.0, x)  # MATLAB keeps the sign of zero (round(-0.11) -> -0)
            return {"cls": v["cls"], "num": _fmt_double(r)}
        s, w, f, n = _decode(v)
        return _out(s, w, f, n)  # integers are already integral
    if "cls" in v and v["cls"] in ("double", "single"):
        x = float(v["num"])
        return {"cls": v["cls"], "num": _fmt_double(float((x > 0) - (x < 0)))}
    s, w, f, n = _decode(v)
    sgn = (n > 0) - (n < 0)
    if "cls" in v and v["cls"].startswith("u"):
        return _out(0, w, 0, sgn)
    lo, hi = -(1 << (w - 1)), (1 << (w - 1)) - 1
    return _out(1, w, 0, max(lo, min(hi, sgn)))


class ParamUnrecorded(ValueError):
    """A parameter written in a valid MATLAB form whose effect here is not recorded (refused, UNSUPPORTED)."""


_INT_CLASSES = {"int8": (1, 8), "uint8": (0, 8), "int16": (1, 16), "uint16": (0, 16), "int32": (1, 32),
                "uint32": (0, 32), "int64": (1, 64), "uint64": (0, 64)}


def matlab_int_expr(text):
    """a numeric parameter in the MATLAB forms real models use -- decimal / 0x.. / 0b.. literals, + - *
    / ^ and parentheses, true / false, int8(..) .. uint64(..) casts (round half away, saturate), intmax / intmin('cls'),
    bitshift(a, k), bitcmp(x) of an integer class, pow2(k), hex2dec / bin2dec('..'), a trailing ';'. Returns
    (exact Fraction value, integer class name or None for double); None when the text is not of this form. Arithmetic
    with an integer-class operand rounds and saturates into that class, as MATLAB does; two different classes: None."""
    import re
    # the native expression lane, MATLAB eval semantics: trailing
    # white space (VT / FF included: '3\x0b' is 3) ends the statement harmlessly; a line break before or inside the
    # value outside brackets is a MATLAB error ('\n8', '-\n8': m_unexpected_sep; '2\n+1': m_missing_variable)
    text = str(text).rstrip(" \t\r\n\x0b\x0c")
    depth = 0
    for ch in text:
        if ch in "[(":
            depth += 1
        elif ch in "])":
            depth = max(depth - 1, 0)
        elif ch in "\r\n" and depth == 0:
            return None
    if any(ord(ch) > 126 or (ord(ch) < 32 and ch not in "\t\n\r") for ch in str(text)):
        return None   # non-ASCII / control whitespace is a MATLAB error (InvalidWhitespace)
    t = text.strip()
    while t.endswith(";"):
        t = t[:-1].strip()
    # ASCII digits only (other scripts' digits are not MATLAB numbers)
    tok_re = re.compile(r"\s*(?:(0[xX][0-9A-Fa-f]+|0[bB][01]+)|((?:[0-9]+\.?[0-9]*|\.[0-9]+)(?:[eE][-+]?[0-9]+)?)|"
                        r"([A-Za-z_][A-Za-z0-9_]*)|('[^']*')|(\S))", re.A)
    toks, pos = [], 0
    while pos < len(t):
        m = tok_re.match(t, pos)
        if not m or m.end() == pos:
            return None
        pos = m.end()
        if m.group(1):
            digits = m.group(1)[2:]
            v = int(digits, 16 if m.group(1)[1] in "xX" else 2)
            # recordings (silent wrong): MATLAB gives a hex / binary literal the class of its digit count, leading
            # zeros included (0x00FF is uint16: 0x00FF+1-253 = 3; by value it was uint8 and saturated)
            bits = 4 * len(digits) if m.group(1)[1] in "xX" else len(digits)
            cls = next((c for c in ("uint8", "uint16", "uint32", "uint64") if bits <= _INT_CLASSES[c][1]), None)
            if cls == "uint64" and v >= 1 << 53:
                return None   # 64-bit values at or beyond 2^53: not modelled (as in cast)
            if cls is None:
                return None
            toks.append(("n", (Fraction(v), cls)))
        elif m.group(2):
            fv = float(m.group(2))
            if fv != fv or fv in (float("inf"), float("-inf")):
                return None   # '1e400' (inf in double) crashed here with INTERNAL
            toks.append(("n", (Fraction(fv), None)))
        elif m.group(3):
            toks.append(("id", m.group(3)))
        elif m.group(4):
            toks.append(("s", m.group(4)[1:-1]))
        elif m.group(5):
            toks.append(("op", m.group(5)))
    i = [0]

    def peek():
        return toks[i[0]] if i[0] < len(toks) else (None, None)

    def take(kind=None, val=None):
        k, v = peek()
        if (kind and k != kind) or (val is not None and v != val):
            raise ValueError
        i[0] += 1
        return v

    def cast(v, cls):
        if cls is None:
            return v
        sg, w = _INT_CLASSES[cls]
        if w == 64 and abs(v) >= 1 << 53:
            # recordings (int64(9007199254740993)*3): beyond 2^53 MATLAB's 64-bit integer literals and arithmetic do
            # not follow the double path; not modelled -- refused
            raise ValueError
        r = _round_quot(v.numerator, v.denominator, "Round")
        lo, hi = (-(1 << (w - 1)), (1 << (w - 1)) - 1) if sg else (0, (1 << w) - 1)
        return Fraction(max(lo, min(hi, r)))

    def combine(a, b, fn, op=None):
        (va, ca), (vb, cb) = a, b
        if "logical" in (ca, cb) and ({ca, cb} - {"logical", None}):
            # MATLAB rejects a logical combined with an integer class (true*int16(3))
            raise ValueError
        ca, cb = (None if ca == "logical" else ca), (None if cb == "logical" else cb)
        if ca and cb and ca != cb:
            raise ValueError
        c = ca or cb
        r = fn(va, vb)
        nonint = va.denominator != 1 or vb.denominator != 1
        if c is not None and nonint and (op in ("+", "-") or _INT_CLASSES[c][1] == 64):
            # recordings (silent wrong) / D2 / D5: + and - of an integer class with a non-integer double are not
            # double-then-round in MATLAB / Simulink (uint8(3)+(1.4-0.9) is 3, not 4: the sum is not rounded to a
            # double first), and 64-bit classes have quirks (int64(3)-(0.7-0.2) is 4; double ^ int64) -- refused
            raise ValueError
        if c is not None and INT_DOUBLE_ROUND and _INT_CLASSES[c][1] <= 32 and \
                (va.denominator != 1 or vb.denominator != 1 or r.denominator != 1):
            # MATLAB: integer-class arithmetic is computed in double, then rounded (half away) and saturated
            try:
                rd = fn(Fraction(float(va)), Fraction(float(vb)))
                rd = Fraction(float(rd))
            except (OverflowError, ZeroDivisionError):
                raise ValueError
            return cast(rd, c), c
        if c is not None and INT_DOUBLE_ROUND and _INT_CLASSES[c][1] == 64 and va.denominator == 1 or \
                c is not None and INT_DOUBLE_ROUND and _INT_CLASSES[c][1] == 64 and vb.denominator == 1:
            # 64-bit classes: MATLAB keeps the double operand's exact value (int64(5)*0.3 = 1: 5 x 0.2999... rounds
            # to 1), so the exact rational is right; rounded half away and saturated
            return cast(r, c), c
        if c is not None and (va.denominator != 1 or vb.denominator != 1 or r.denominator != 1):
            # recordings (silent wrong): MATLAB computes integer-class arithmetic in double and rounds that
            # (0.3*int8(5) = 2; the exact rational gave 1) -- an integer class with a non-integer operand or result
            # is not modelled (refused), only exact integer arithmetic is
            raise ValueError
        if c is None and not _dyadic(r):
            # in double, 1/49*49 is 0.99999999999999989 (Simulink stops), not the exact rational 1;
            # an intermediate not exact in binary is left to the double evaluator (engine/fxnum.py)
            raise ValueError
        return cast(r, c), c

    def expr():
        v = term()
        while peek() in (("op", "+"), ("op", "-")):
            op = take()
            w_ = term()
            v = combine(v, w_, (lambda x, y: x + y) if op == "+" else (lambda x, y: x - y), op)
        return v

    def term():
        v = unary()
        while peek() in (("op", "*"), ("op", "/")):
            op = take()
            w_ = unary()
            if op == "/" and w_[0] == 0:
                raise ValueError
            v = combine(v, w_, (lambda x, y: x * y) if op == "*" else (lambda x, y: x / y), op)
        return v

    def unary():
        if peek() in (("op", "-"), ("op", "+")):
            op = take()
            v, c = unary()
            if c == "logical":   # a sign on a logical gives a double (+true, -true)
                c = None
            return (cast(-v, c), c) if op == "-" else (v, c)
        return power()

    def power():
        # recordings (silent wrongs): MATLAB's power chain, as recorded in recordings
        # (29,052 / 29,055 chains): unsigned exponents fold left (2^1^3 = 8); a signed exponent takes the rest of the
        # chain up to -- not including -- its final exponent, and inside that span a signed exponent takes the rest
        # of the span (2^-1^-1^3 = (2^-(1^-1))^3 = 0.125). Two signs on one exponent are not modelled (refused)
        v = atom()
        items = []
        while peek() == ("op", "^"):
            take()
            neg, nsign = False, 0
            while peek() in (("op", "-"), ("op", "+")):
                neg ^= take() == "-"
                nsign += 1
            if nsign > 1:
                raise ValueError
            items.append((nsign == 1, neg, atom()))

        def pw(x, e):
            if e[0].denominator != 1 or abs(e[0]) > 64:
                raise ValueError
            if (x[1] not in (None, "logical") or e[1] not in (None, "logical")) and e[0] < 0:
                # a negative power with an integer class on either side is a MATLAB error
                raise ValueError
            if 64 in (_INT_CLASSES.get(x[1], (0, 0))[1], _INT_CLASSES.get(e[1], (0, 0))[1]) and x[0].denominator != 1:
                raise ValueError   # double ^ int64 / uint64 -- MATLAB rounds the double power; refused
            return combine(x, e, lambda a_, b_: a_ ** int(b_), "^")

        def signed(e, neg):
            return (cast(-e[0], e[1]), e[1]) if neg else e

        def fold(x, its, end):
            i_ = 0
            while i_ < len(its):
                sg, neg, a = its[i_]
                if not sg or i_ >= end:
                    x = pw(x, signed(a, neg))
                    i_ += 1
                    continue
                operand = fold(a, its[i_ + 1:end], end - i_ - 1) if end > i_ + 1 else a
                x = pw(x, signed(operand, neg))
                i_ = end
            return x
        return fold(v, items, len(items) - 1)

    def atom():
        k, v = peek()
        if k == "n":
            take()
            return v
        if k == "op" and v == "(":
            take()
            r = expr()
            take("op", ")")
            return r
        if k == "id":
            take()
            if v in ("true", "false"):
                return Fraction(int(v == "true")), "logical"
            take("op", "(")
            if v in ("intmax", "intmin"):
                cls = take("s")
                take("op", ")")
                sg, w = _INT_CLASSES[cls]
                val = ((1 << (w - 1)) - 1 if sg else (1 << w) - 1) if v == "intmax" else (-(1 << (w - 1)) if sg else 0)
                return Fraction(val), cls
            if v in ("hex2dec", "bin2dec"):
                txt = take("s")
                take("op", ")")
                # recordings (silent wrong): the result is a double -- exact only below 2^53 (hex2dec of
                # '20000000000001' rounds to 2^53) -- and characters outside the digit set are an error
                if not re.match(r"^[0-9A-Fa-f]{1,16}$" if v == "hex2dec" else r"^[01]{1,64}$", txt):
                    raise ValueError   # (more than 16 / 64 digits is MATLAB's TooManyDigits)
                val = int(txt, 16 if v == "hex2dec" else 2)
                if val >= 1 << 53:
                    raise ValueError
                return Fraction(val), None
            args = [expr()]
            while peek() == ("op", ","):
                take()
                args.append(expr())
            take("op", ")")
            if v in _INT_CLASSES and len(args) == 1:
                return cast(args[0][0], v), v
            if v == "pow2" and args[0][1] not in (None, "logical"):
                raise ValueError   # pow2 of an integer class is a MATLAB error (mustBeFloat)
            if v == "bitshift" and args[0][1] in (None, "logical") and args[0][0] < 0:
                raise ValueError   # bitshift of a negative double is a MATLAB error
            if v in ("pow2", "bitshift") and any(x[1] == "logical" for x in args):
                raise ValueError   # a logical in bitshift / pow2 is a MATLAB error
            if v == "pow2" and len(args) == 1:
                k_ = args[0][0]
                if k_.denominator != 1 or not -1074 <= k_ <= 1023:
                    raise ValueError   # pow2 beyond the double range (Inf / 0 in MATLAB)
                return Fraction(2) ** int(k_), args[0][1]
            if v == "bitshift" and len(args) == 2 and args[1][0].denominator == 1 and args[0][0].denominator == 1:
                a_, c_ = args[0]
                if abs(args[1][0]) > 64:
                    raise ValueError   # the shift was built before any range check (GBs of memory)
                k_ = int(args[1][0])
                if c_ is None and not 0 <= a_ < 1 << 53:
                    raise ValueError   # bitshift of a double outside [0, 2^53) -- MATLAB errors past 2^64
                r_ = int(a_) << k_ if k_ >= 0 else int(a_) >> -k_
                if c_ in (None, "logical") and abs(r_) >= 1 << 53:
                    # recordings (silent wrong): MATLAB's bitshift of a double drops bits past 64 and loses
                    # exactness past 2^53 (bitshift(1,64) = 0); not modelled -- refused
                    raise ValueError
                if c_ not in (None, "logical"):
                    sg, w = _INT_CLASSES[c_]
                    r_ &= (1 << w) - 1   # MATLAB drops the bits shifted out of the class
                    if sg and r_ >> (w - 1):
                        r_ -= 1 << w
                return Fraction(r_), c_
            if v == "bitcmp" and len(args) == 1 and args[0][1] and not _INT_CLASSES[args[0][1]][0]:
                w = _INT_CLASSES[args[0][1]][1]
                r_ = ~int(args[0][0]) & ((1 << w) - 1)
                if w == 64 and r_ >= 1 << 53:
                    raise ValueError   # 64-bit values at or beyond 2^53: not modelled (as in cast)
                return Fraction(r_), args[0][1]
        raise ValueError
    try:
        r = expr()
        if i[0] != len(toks) or r[0] is None:
            return None
        return r
    except (ValueError, KeyError, TypeError, ZeroDivisionError, OverflowError):
        return None


INT_DOUBLE_ROUND = True   # MATLAB computes in double for <= 32-bit classes (exact for 64-bit), 7,577 equal, 0 differ


def _dyadic(x):
    """x (a Fraction) is exactly a double (MATLAB rounds every double operation; 3*0.1 is not 3/10's double)."""
    try:
        return Fraction(float(x)) == x
    except OverflowError:
        return False


def bitmask_stored(p, s, w, f):
    """The Bitwise Operator mask as a stored-integer pattern of the input word (both mask modes)."""
    mv = _mask_value(p["BitMask"])
    typed = None
    if not isinstance(mv, Fraction):
        mv, typed = mv
    # recordings (120 designs): an integer-class mask acts by its value exactly as a plain
    # number does -- real-world scaling, rounding down, then modulo 2^wl (30 / 30 out-of-range and real-world ones)
    if p.get("BitMaskRealWorld", "").startswith("Real"):
        mv = mv * (Fraction(2) ** f)
    n = _round_quot(mv.numerator, mv.denominator, "Floor")
    return n & ((1 << w) - 1)


def _mask_value(text):
    """Bitwise Operator mask: decimal, hex2dec('..') or bin2dec('..') -> exact number."""
    import re
    t = text.strip()
    m = re.match(r"^bin2dec\('([01]+)'\)$", t)
    if m:
        v = Fraction(int(m.group(1), 2))
    else:
        mh = re.match(r"^hex2dec\('([0-9A-Fa-f]+)'\)$", t)
        if mh:
            v = Fraction(int(mh.group(1), 16))
        elif re.fullmatch(r"[-+]?(?:[0-9]+\.?[0-9]*|\.[0-9]+)(?:[eE][-+]?[0-9]+)?", t, re.A):
            v = Fraction(float(t))   # a plain decimal literal only ("1_0" was read as 10)
        else:
            r = matlab_int_expr(t)   # 0xFF, 0b1010, 2^4-1, uint8(200), intmax('uint16') ...
            if r is None:
                raise ValueError("Bitwise Operator BitMask %r: not a numeric MATLAB expression the engine evaluates "
                                 "(literals, + - * / ^, int casts, intmax / intmin, bitshift, bitcmp, pow2, hex2dec, "
                                 "bin2dec)" % (text,))
            if abs(r[0]) >= 2 ** 52:
                raise ValueError("Bitwise Operator BitMask %s: magnitude 2^52 or more cannot be set in Simulink" % t)
            return r
    if abs(v) >= 2 ** 52:
        # Simulink cannot set a mask of magnitude 2^52 or more
        # (mask initialisation fails); the engine answered modulo 2^wl
        raise ValueError("Bitwise Operator BitMask %s: magnitude 2^52 or more cannot be set in Simulink" % t)
    return v


def _blk_cmpzero(case):
    """Compare To Zero (30 designs): the input's value against 0 -- a boolean as 0 / 1 --
    output boolean or uint8."""
    p = case["params"]
    v = case["ins"][0]
    n = int(v["bool"]) if "bool" in v else _decode(v)[3]
    return _bool_or_named(p.get("OutDataTypeStr", "boolean"), _RELOPS[p["relop"]](n, 0))


def _blk_sqrt(case):
    """Sqrt, Operator sqrt, AlgorithmType Exact, explicit output type: the exact square root of
    the input's real value in the output's units, rounded by the block's RndMeth (Floor / Zero / Ceiling / Nearest,
    half up) and saturated into the output type. A negative input stops the run at that step, as Simulink's
    simulation stops (fxpSqrt_negativeinputnotsupported: every one of the 13 recorded designs with a negative input,
    recordings)."""
    p = case["params"]
    s, w, f, n = _decode(case["ins"][0])
    os_, ow, of = _named_type(p["OutDataTypeStr"])
    if n < 0:
        raise SimDiagnostic("Sqrt of a negative input (%g): Simulink stops the simulation here "
                            "(fxpSqrt_negativeinputnotsupported)" % (n / 2.0 ** f))
    sh = 2 * of - f   # y = sqrt(n * 2^-f) * 2^of = sqrt(n * 2^(2 of - f))
    rm = p.get("RndMeth", "Floor")
    if rm in ("Convergent", "Round"):
        rm = "Nearest"   # no ties when sh >= 0 (the engine refuses these for sh < 0)
    if sh >= 0:
        N = n << sh
        k = math.isqrt(N)
        if rm in ("Ceiling",) and k * k != N:
            k += 1
        elif rm == "Nearest" and N >= k * k + k + 1:   # N integer: k + 0.5 squared is k^2 + k + 0.25
            k += 1
    else:
        # input finer than twice the output fraction: y = sqrt(n / 2^m); floor(sqrt(x)) = isqrt(floor(x)) for x >= 0
        # (Floor / Zero / Ceiling are the correctly rounded root; Nearest is refused)
        if rm == "Nearest":
            raise LutUnrecorded("Sqrt with RndMeth Nearest and an input finer than twice the output fraction is not "
                                "determined")
        m = -sh
        k = math.isqrt(n >> m)
        if rm == "Ceiling" and (k * k) << m != n:
            k += 1
    # the result saturates to the output type whatever SaturateOnIntegerOverflow says (recordings /
    # with it off: 255 / 65535 where the root exceeds the type)
    return _out(os_, ow, of, _overflow(k, os_, ow, "Saturate"))


def _nl_param(p, k, s, w, f):
    """A Dead Zone / Rate Limiter / Wrap To Zero parameter in the input type: nearest, saturated
    (0.5 in ufix12 is 1, -0.25 in ufix12 is 0, both with a precision-loss warning; an
    overflow beyond one LSB is refused by the engine's parameter check first)."""
    return _quant_param(Fraction(_const_value(p[k])), s, w, f)


def _blk_deadzone(case):
    """Dead Zone: output type = input type; the
    Start / End of the dead zone quantised to it (nearest, ties up, saturated within one LSB); 0 where
    Start <= u <= End, u - End above, u - Start below, the exact difference then saturated
    (SaturateOnIntegerOverflow on) or wrapped (off) into the input type. A Start above 0 or an End below 0 makes the
    difference leave the type in both overflow modes; a Start above the End is not recorded."""
    p = case["params"]
    s, w, f, n = _decode(case["ins"][0])
    lo, up = _nl_param(p, "LowerValue", s, w, f), _nl_param(p, "UpperValue", s, w, f)
    if lo > up:   # the engine refuses it at compile time (fxgraph_lib _check_validated)
        raise ParamUnrecorded("Dead Zone LowerValue above UpperValue in the input type is not recorded")
    om = "Saturate" if str(p.get("SaturateOnIntegerOverflow", "on")).strip() == "on" else "Wrap"
    return _out(s, w, f, _overflow(n - up if n > up else n - lo if n < lo else 0, s, w, om))


def _blk_wraptozero(case):
    """Wrap To Zero: output type = input type; 0 where u > Threshold (quantised to
    the input type, as the mask's Switch threshold), else u."""
    p = case["params"]
    s, w, f, n = _decode(case["ins"][0])
    return _out(s, w, f, 0 if n > _nl_param(p, "Threshold", s, w, f) else n)


_RL_STATE = {}   # (op, batch, block) -> (step, previous output)


def _blk_ratelimit(case):
    """Rate Limiter, sample period 1 or 2: output
    type = input type; the rising and falling slew limits times the period (Simulink's weighted limits) and the initial condition quantised to it; y = y_prev + R where
    u - y_prev > R, y_prev + F where u - y_prev < F, else u (exact differences; the results cannot overflow)."""
    p = case["params"]
    s, w, f, n = _decode(case["ins"][0])
    T = Fraction(str(p.get("_RlT", "1")))   # the sample period (1 and 2 recorded)
    R = _quant_param(Fraction(_const_value(p["RisingSlewLimit"])) * T, s, w, f)
    # the falling limit is not held in the input type (-1 with a ufix12 input runs); its magnitude is
    # quantised (the engine refuses ties and magnitudes beyond the input type)
    Fl = -_quant_param(-Fraction(_const_value(p["FallingSlewLimit"])) * T, 0, w + 1, f)
    if R < 0 or Fl > 0:
        raise ParamUnrecorded("Rate Limiter with a negative rising or a positive falling slew limit (in the input "
                              "type) is not recorded")
    key = (case["op"], case["batch"], case["block"])
    k = case["step"]
    if k == 0 or key not in _RL_STATE:
        yp = _nl_param(p, "InitialCondition", s, w, f)
    else:
        k0, yp = _RL_STATE[key]
        if k0 != k - 1:
            raise ValueError("stateful block %s: step %d arrived after %d" % (key, k, k0))
    d = n - yp
    y = yp + R if d > R else yp + Fl if d < Fl else n
    _RL_STATE[key] = (k, y)
    return _out(s, w, f, y)


CORDIC_BEYOND = False   # CORDIC sin/cos supports angles within +-pi/2.
CORDIC_SC_MARGIN = 4   # steps of 2^-(w-2) around k pi/2 refused (0 of 412 steps differ outside)


def _cordic_sincos(case):
    """Trigonometric Function sin / cos, ApproximationMethod CORDIC: output sfix(w, w-2). Only angles within +-pi/2
    are answered (no reduction rule found reproduces Simulink beyond); the angle is floored to w-2
    fraction bits. Rotation CORDIC: x0 = 1/K_N rounded to nearest at w-2 bits, y0 = 0; i = 0 .. N-1:
    where z >= 0 x -= y >> i, y += x >> i, z -= atan(2^-i); else the opposite (arctangents rounded to nearest at w-2
    bits, >> a floor shift). sin is y, cos is x. Angles within CORDIC_SC_MARGIN steps of a multiple of pi/2 are
    refused (the reduction there is not reproduced: the constants' precision is not determined), and an angle
    outside [-2 pi, 2 pi] stops Simulink (CordicAngleOutOfRange)."""
    p = case["params"]
    op = str(p.get("Operator", "sin"))
    s, w, f, n = _decode(case["ins"][0])
    N = int(str(p.get("NumberOfIterations", "11")).strip())
    fz = w - 2
    th = Fraction(n, 1) / Fraction(2) ** f
    if abs(th) > Fraction(2 * math.pi):
        raise SimDiagnostic("CORDIC input angle %s is outside [-2 pi, 2 pi] (Simulink:blocks:CordicAngleOutOfRange "
                            "stops the simulation)" % float(th))
    hp = Fraction(math.pi) / 2
    neg = False
    if abs(th) > hp:
        if not CORDIC_BEYOND:
            # Wider angles need an independently verified quadrant reduction.
            raise LutUnrecorded("CORDIC %s: the angle %s is beyond +-pi/2, where quadrant reduction is "
                                "outside the supported range. Reduce the angle to "
                                "[-pi/2, pi/2] before the block and apply the sign yourself" % (op, float(th)))
        # candidate (0 of 76,254 steps beyond +-pi/2 wrong): the reduction works on
        # fixdt(1, W, W-4), W = max(w, 16): the angle floored to G = W-4 fraction bits, pi and pi/2 rounded to nearest
        # there, 2 pi = twice that pi; beyond +-pi by 2 pi, then beyond +-pi/2 by pi with both results negated
        G = max(w, 16) - 4
        grid = Fraction(1, 2 ** min(G, fz))
        kq = round(th / hp)
        if abs(th - kq * hp) < CORDIC_SC_MARGIN * grid:
            raise LutUnrecorded("CORDIC %s: the angle %s is within %d steps of %d x pi/2, where Simulink's quadrant "
                                "reduction is not reproduced" % (op, float(th), CORDIC_SC_MARGIN, kq))
        if f > G:
            v = th * Fraction(2) ** G
            th = Fraction(v.numerator // v.denominator, 2 ** G)

        def qg(c):
            v = Fraction(c) * Fraction(2) ** G
            return Fraction(_round_quot(v.numerator, v.denominator, "Nearest")) / Fraction(2) ** G
        PI, HP = qg(math.pi), qg(math.pi / 2)
        TP = 2 * PI
        if th > PI:
            th -= TP
        elif th < -PI:
            th += TP
        if th > HP:
            th, neg = th - PI, True
        elif th < -HP:
            th, neg = th + PI, True
    else:
        kq = round(th / hp)
        if kq != 0 and abs(th - kq * hp) * Fraction(2) ** fz < CORDIC_SC_MARGIN:
            raise LutUnrecorded("CORDIC %s: the angle %s is within %d steps of %d x pi/2, where Simulink's quadrant "
                                "reduction is not reproduced" % (op, float(th), CORDIC_SC_MARGIN, kq))
    zv = th * Fraction(2) ** fz
    z = zv.numerator // zv.denominator
    K = 1.0
    for i in range(N):
        K *= 1 / math.sqrt(1 + 2 ** (-2 * i))
    kv = Fraction(K) * Fraction(2) ** fz
    x, y = _round_quot(kv.numerator, kv.denominator, "Nearest"), 0
    for i in range(N):
        av = Fraction(math.atan(2.0 ** -i)) * Fraction(2) ** fz
        a = _round_quot(av.numerator, av.denominator, "Nearest")
        if z >= 0:
            x, y, z = x - (y >> i), y + (x >> i), z - a
        else:
            x, y, z = x + (y >> i), y - (x >> i), z + a
    if neg:
        x, y = -x, -y
    return _out(1, w, fz, y if op == "sin" else x)


def _blk_trig(case):
    """Trigonometric Function, Operator atan2, ApproximationMethod CORDIC (recordings
    888 of 912 steps bit-exact, the rest at 27 iterations on 28 bits, which the engine refuses). Output
    sfix(w, w-3) for two signed inputs of one type, sfix(w, w-2) for unsigned ones. On the x axis (y = 0) the exact
    values: 0 for x >= 0, pi rounded to nearest in the output type for x < 0. Otherwise a point with x < 0 is negated
    (both coordinates) and +pi (y >= 0) or -pi (y < 0) is added at the end; then a point with y < 0 is mirrored to
    y > 0 and the angle negated. Vectoring CORDIC on the stored integers, unbounded: i = 0 .. N-1, where y < 0:
    x -= y >> i, y += x >> i, z -= atan(2^-i); else x += y >> i, y -= x >> i, z += atan(2^-i); the arctangents rounded
    to nearest in the output type, >> an arithmetic (floor) shift. (Mirroring matters where y reaches 0 during the
    iterations: the next step then turns away from the original half plane.)"""
    p = case["params"]
    if str(p.get("ApproximationMethod")) == "CORDIC" and str(p.get("Operator", "sin")) in ("sin", "cos"):
        return _cordic_sincos(case)
    if str(p.get("Operator", "sin")) != "atan2" or str(p.get("ApproximationMethod")) != "CORDIC":
        raise ParamUnrecorded("Trigonometric Function: only atan2 / sin / cos with CORDIC are recorded bit-exactly")
    (s1, w, f1, y), (s2, w2, f2, x) = (_decode(v) for v in case["ins"])
    N = int(str(p.get("NumberOfIterations", "11")).strip())
    fz = w - 3 if s1 else w - 2
    pi_q = _round_quot(*Fraction(math.pi * 2 ** fz).as_integer_ratio(), "Nearest")
    if y == 0:
        return _out(1, w, fz, 0 if x >= 0 else pi_q)
    add = 0
    if x < 0:
        add = pi_q if y >= 0 else -pi_q
        x, y = -x, -y
    flip = y < 0
    if flip:
        y = -y
    z = 0
    for i in range(N):
        a = _round_quot(*Fraction(math.atan(2.0 ** -i) * 2 ** fz).as_integer_ratio(), "Nearest")
        if y < 0:
            x, y, z = x - (y >> i), y + (x >> i), z - a
        else:
            x, y, z = x + (y >> i), y - (x >> i), z + a
    return _out(1, w, fz, (-z if flip else z) + add)


def _dti_statefb(case, want_gain):
    """Discrete-Time Integrator whose state port feeds, through a Gain, its own external initial condition
    (24 + 24 designs bit-exact): s_k = the state before any reset at step k (0 at the
    start), ic_k = floor(g * s_k) wrapped into the output type (the Gain's output type, which must equal it); on a
    reset x_k = ic_k; y_k = x_k; x_{k+1} = x_k + floor(K u_k). Returns y (want_gain False) or the Gain's output ic."""
    p = case["params"]
    h = _history(case)
    s, w, f, _sl = _dti_type(p, h[0][0][:3])
    K, g = dti_gain(p), Fraction(str(p["_SFGain"]))
    mode = str(p.get("ExternalReset", "none"))

    def wrap(v):
        return _overflow_plain(v, s, w, "Wrap")
    x, rp = 0, 0
    for k in range(case["step"] + 1):
        us, uw, uf, un = h[k][0]
        sk = x
        v = g * sk
        ic = wrap(v.numerator // v.denominator)
        if mode != "none":
            r = h[k][1][3]
            hit = (rp <= 0 < r) if mode == "rising" else (rp > 0 >= r) if mode == "falling" else \
                (rp <= 0 < r or rp > 0 >= r) if mode == "either" else (r != 0 or rp != 0)
            if hit:
                x = ic
            rp = r
        if k == case["step"]:
            return _out(s, w, f, ic if want_gain else x)
        inc = K * Fraction(un) / (Fraction(2) ** uf) * (Fraction(2) ** f)
        x = wrap(x + inc.numerator // inc.denominator)


def _blk_dtistate(case):
    return _dti_statefb(case, False)


def _blk_dtistategain(case):
    return _dti_statefb(case, True)


def delayport_source(k, nk, lim, pdf, mode, rsts, last=None):
    """The port-length Delay at step k with run-time length nk: the step whose input is output, or None for the
    initial condition. nk is clamped into 0 .. lim, into 1 .. lim with
    PreventDirectFeedthrough 'on' (a length 0 or -1 reads
    the previous step's input). ExternalReset 'Rising' (a boolean reset): at a step where the
    reset changes from 0 the history is filled with the initial condition before the output, so a read from before
    the last such step gives the initial condition; rsts are the reset values of steps 0 .. k (or, when last is given,
    of steps k - 1 .. k: last is then the last step <= k where the reset changed from 0, computed by the caller)."""
    nk = min(max(nk, 1 if pdf else 0), lim)
    if mode == "Rising" and last is not None:
        rsts = ([0] * (k + 1 - len(rsts))) + list(rsts)   # only rsts[k - 1], rsts[k] are read below
    elif mode == "Rising":
        last = 0
        prev = 0
        for t, r in enumerate(rsts[:k + 1]):
            if r != 0 and prev == 0:
                last = t
            prev = r
    else:
        last = 0
    if mode == "Rising":
        if last == k and nk == 0 and rsts[k] != 0 and (k == 0 or rsts[k - 1] == 0):
            raise LutUnrecorded("Delay length 0 from the port at a reset step: not recorded (the recorded resets "
                                "met lengths 1 .. 4); record a design with this step to open it")
    elif mode not in ("None", "none"):
        raise LutUnrecorded("ExternalReset %r with the length from a port: only 'Rising' is recorded" % mode)
    return k - nk if k - nk >= last else None


_DLYEN_STATE = {}   # (op, batch, block) -> (step, state, held output, reset one / two steps earlier)
_DLYPORT_LAST = {}  # (op, batch, block) -> (step, last step where the reset changed from 0)


def _blk_delayport(case):
    """Delay with the length or the initial condition from port 2 (20 designs bit-exact):
    output type = input type. Length from the port (uint8): y_k = u_{k - n_k}, the initial condition (0) before the
    start; n_k above DelayLengthUpperLimit is not recorded. Initial condition from the port: y_k = x0_0 for k < L
    (the port's value at the first step), then u_{k-L}."""
    p = case["params"]
    h = _history(case)
    k = case["step"]
    s, w, f, _ = h[k][0]
    enab = str(p.get("ShowEnablePort", "off")).strip() == "on"
    mode = str(p.get("ExternalReset", "None")).strip()
    if (enab or mode not in ("None", "none")) and str(p.get("DelayLengthSource", "Dialog")) != "Input port":
        # the Delay with an enable port and / or an external reset, dialog length and initial condition (the engine's
        # recordings; recorded
        # recordings -- every step equal):
        # - an enabled step (enable > 0; for a boolean / unsigned enable: nonzero) outputs the oldest state and shifts
        #   u in; a disabled step holds the previous output and the state. Before the first enabled step the held
        #   output is 0, not the initial condition;
        # - a reset acts on enabled steps (every step without an enable port), before the output: it fills the state
        #   with the initial condition, which is then output. Level: the reset is nonzero now or at the previous
        #   step; Level hold: nonzero now; Rising / Falling / Either: an edge of the reset's sign -- from negative to
        #   zero or positive, or from zero to positive unless the sample before that zero was negative (Falling
        #   mirrored), with zero before the start. The reset's history advances on every step, enabled or not.
        N = int(str(p.get("DelayLength", "2")).strip())
        if N < 1:
            # DelayLength 0 with an enable / reset port runs
            # in Simulink; its rule is not settled here -- refused, not a crash (the type probe hit an IndexError)
            raise LutUnrecorded("a Delay with an enable or external reset port and DelayLength %d is not supported" % N)
        ic = _quant_param(float(_const_value(p.get("InitialCondition", "0"))), s, w, f)
        ri = 2 if enab else 1
        key = (case["op"], case["batch"], case["block"])
        memo = _DLYEN_STATE.get(key)
        if k > 0 and memo is not None and memo[0] == k - 1:
            # the state after step k - 1 (this kernel used to replay every step from 0: quadratic in the run length)
            _, state, y, r1, r2 = memo
            state = list(state)
            t0 = k
        else:
            state = [ic] * N
            y = 0 if enab else ic
            r1 = r2 = 0   # the reset one and two steps earlier
            t0 = 0
        for t in range(t0, k + 1):
            en = (h[t][1][3] > 0) if enab else True
            rt = h[t][ri][3] if mode not in ("None", "none") else 0
            sg, s1, s2 = (rt > 0) - (rt < 0), (r1 > 0) - (r1 < 0), (r2 > 0) - (r2 < 0)
            rise = (s1 < 0 and sg >= 0) or (s1 == 0 and sg > 0 and s2 >= 0)
            fall = (s1 > 0 and sg <= 0) or (s1 == 0 and sg < 0 and s2 <= 0)
            rst = {"Level": rt != 0 or r1 != 0, "Level hold": rt != 0, "Rising": rise, "Falling": fall,
                   "Either": rise or fall}.get(mode, False)
            if en:
                if rst:
                    state = [ic] * N
                y = state[0]
                state = state[1:] + [h[t][0][3]]
            r1, r2 = rt, r1
        _DLYEN_STATE[key] = (k, state, y, r1, r2)
        return _out(s, w, f, y)
    if str(p.get("DelayLengthSource", "Dialog")) == "Input port":
        nk = h[k][1][3]
        lim_t = str(p.get("DelayLengthUpperLimit", "100")).strip()
        if not lim_t.isdigit():
            raise LutUnrecorded("DelayLengthUpperLimit %r: only integer upper limits are recorded" % lim_t)
        lim = int(lim_t)
        cap = min(lim, 64)   # recorded lengths 0 .. 16 and 0 .. 32, with limits up to 64
        # a run-time length below 0 or above DelayLengthUpperLimit is clamped into
        # 0 .. limit (every such recorded step equal); it had been refused
        dg = str(p.get("DiagnosticForDelayLength", "None")).strip()
        if not 0 <= nk <= lim and dg != "None":
            # with DiagnosticForDelayLength 'Error' an
            # out-of-range length stops Simulink (ParameterDelayLengthIsLessThanLowerLimitOrLargerThanUpperLimit);
            # the clamp below is recorded with 'None' only (every design)
            if dg == "Error":
                raise SimDiagnostic("Delay length %d from the port is outside 0 .. %d: Simulink stops the simulation "
                                    "(DiagnosticForDelayLength 'Error', ParameterDelayLengthIsLessThanLowerLimitOr"
                                    "LargerThanUpperLimit)" % (nk, lim))
            raise LutUnrecorded("Delay length %d from the port is outside 0 .. %d with DiagnosticForDelayLength %r: "
                                "only 'None' (clamped) and 'Error' (stops) are recorded" % (nk, lim, dg))
        pdf = str(p.get("PreventDirectFeedthrough", "off")).strip() == "on"
        mode = str(p.get("ExternalReset", "None")).strip()
        if pdf or mode not in ("None", "none"):
            # recordings (every step equal)
            if str(p.get("InitialConditionSource", "Dialog")) == "Input port" or lim > cap:
                raise LutUnrecorded("PreventDirectFeedthrough / ExternalReset with an initial condition from a port "
                                    "or DelayLengthUpperLimit above %d: not recorded" % cap)
            last_ = None
            if mode == "Rising":
                # the last step where the reset changed from 0, kept per block (was a scan of every step: quadratic)
                key = (case["op"], case["batch"], case["block"])
                memo = _DLYPORT_LAST.get(key)
                if k > 0 and memo is not None and memo[0] == k - 1:
                    last_ = memo[1]
                else:
                    last_ = 0
                    for t in range(k):
                        if h[t][2][3] != 0 and (t == 0 or h[t - 1][2][3] == 0):
                            last_ = t
                if h[k][2][3] != 0 and (k == 0 or h[k - 1][2][3] == 0):
                    last_ = k
                _DLYPORT_LAST[key] = (k, last_)
                rs_ = [h[t][2][3] for t in range(max(k - 1, 0), k + 1)]
            else:
                rs_ = [h[t][2][3] for t in range(k + 1)] if mode not in ("None", "none") else []
            src = delayport_source(k, nk, lim, pdf, mode, rs_, last=last_)
            return _out(s, w, f, h[src][0][3] if src is not None else
                        _quant_param(float(_const_value(p.get("InitialCondition", "0"))), s, w, f))
        nk = min(max(nk, 0), lim)
        if nk < 0 or nk > cap:
            raise LutUnrecorded("Delay length %d from the port is outside the recorded 0 .. %d (DelayLengthUpperLimit "
                                "%d); record a design with this length to open it" % (nk, cap, lim))
        if k - nk >= 0:
            return _out(s, w, f, h[k - nk][0][3])
        # before the start: the initial value -- port 3's value at the first step when it also comes from a port,
        # else the dialog value quantised to the output type (0.438 in sfix16_En8 is
        # 112; the earlier recordings had 0 only)
        if str(p.get("InitialConditionSource", "Dialog")) == "Input port":
            return _out(s, w, f, h[0][2][3])
        return _out(s, w, f, _quant_param(float(_const_value(p.get("InitialCondition", "0"))), s, w, f))
    L = int(str(p["DelayLength"]).strip())
    return _out(s, w, f, h[k - L][0][3] if k >= L else h[0][1][3])


def _blk_bitwise(case):
    """Bitwise Operator on the stored integers' two's-complement bit patterns (word length of the inputs,
    output type = input type). Mask: 'Stored Integer' as given; 'Real World Value' scaled into the input's
    stored units (value * 2^fl). Both taken modulo 2^wl."""
    p = case["params"]
    vals = [_decode(v) for v in case["ins"]]
    s, w, f, _ = vals[0]
    m = (1 << w) - 1
    xs = [n & m for (_s, _w, _f, n) in vals]
    if p.get("UseBitMask") == "on":
        # a real-world mask that is not a whole number of LSBs is rounded down (160 recordings cases, fl < 0)
        xs.append(bitmask_stored(p, s, w, f))
    op = p["logicop"]
    acc = xs[0]
    if op in ("AND", "NAND"):
        for x in xs[1:]:
            acc &= x
    elif op in ("OR", "NOR"):
        for x in xs[1:]:
            acc |= x
    elif op in ("XOR",):
        for x in xs[1:]:
            acc ^= x
    if op in ("NAND", "NOR", "NOT"):
        acc = ~acc & m
    n = acc - (1 << w) if (s and acc >> (w - 1)) else acc
    return _out(s, w, f, n)


def _blk_extract(case):
    """Extract Bits: a bit field of the input's two's-complement pattern; the output keeps the input's
    signedness; 'Preserve fixed-point scaling' -> fl_out = fl_in - (index of the field's lowest bit),
    'Treat bit field as an integer' -> fl_out = 0. Upper and lower half are ceil(w/2) bits."""
    p = case["params"]
    s, w, f, n = _decode(case["ins"][0])
    lo, nb = _extract_field(p, s, w)
    # arithmetic shift: bits above the word are sign extension; a field starting below bit 0 (a negative index, or
    # more bits from the MSB than the word has) shifts the pattern up
    pat = n >> lo if lo >= 0 else n << -lo
    v = pat & ((1 << nb) - 1)
    if s and v >> (nb - 1):
        v -= 1 << nb
    fl = f - lo if p["outScalingMode"].startswith("Preserve") else 0
    return _out(s, nb, fl, v)


def _num_bits(p):
    """Extract Bits numBits: 0 acts as 1; a negative count is rejected by
    Simulink (ParameterValiditySignFinitenessErrorMessage, recordings ...; recorded a previous version)."""
    n = int(p["numBits"])
    if n < 0:
        raise ValueError("Extract Bits numBits %d: Simulink rejects a negative number of bits" % n)
    return max(n, 1)


def _index_vector(text):
    """Integer elements of a MATLAB row vector parameter written with expressions ('[1+1 2]', '[int8(1), 3];'), each
    element by matlab_int_expr; None when an element is not an integer or the text is not a plain row."""
    t = text.strip()
    while t.endswith(";"):
        t = t[:-1].strip()
    if t.startswith("[") and t.endswith("]"):
        t = t[1:-1]
    import re as _re
    if _re.search(r",\s*,", t):   # (a leading comma is valid MATLAB)
        return None   # a doubled / leading comma is a MATLAB syntax error
    elems, depth, cur = [], 0, ""
    for ch in t:
        if ch in "([":
            depth += 1
        elif ch in ")]":
            depth -= 1
        if depth == 0 and (ch in ", " or ch == "\t"):
            if cur.strip():
                elems.append(cur.strip())
            cur = ""
            continue
        cur += ch
    if cur.strip():
        elems.append(cur.strip())
    out = []
    for e in elems:
        while e.startswith("[") and e.endswith("]"):
            e = e[1:-1].strip()
        r = matlab_int_expr(e)
        if r is None or r[0].denominator != 1:
            return None
        out.append(int(r[0]))
    return out or None


def _extract_field(p, s, w):
    """(lowest bit index, number of bits) of the Extract Bits field for an input of signedness s, width w."""
    import re
    mode = p["bitsToExtract"]
    if mode == "Upper half":
        nb = -(-w // 2)
        lo = w - nb
    elif mode == "Lower half":
        nb = -(-w // 2)
        lo = 0
    elif mode == "Range starting with most significant bit":
        nb = _num_bits(p)   # numBits 0 acts as 1
        lo = w - nb                      # numBits > wl: lo < 0, the field extends below bit 0 (xl, 8 / 8)
    elif mode == "Range ending with least significant bit":
        nb = _num_bits(p)
        lo = 0
    elif mode != "Range of bits":
        # recordings: 'Range starting with least significant bit' (not a value of the block) was read as the
        # MSB range; Simulink warns at load, falls back to 'Upper half' and compiles another type
        raise ValueError("bitsToExtract %r is not one of 'Upper half', 'Lower half', 'Range starting with most "
                         "significant bit', 'Range ending with least significant bit', 'Range of bits'" % mode)
    else:
        # exactly "[a b]" with ASCII integers ("[x1 y7]", "[0:7]", other digits were read as numbers)
        m_ = re.fullmatch(r"\s*\[\s*(-?[0-9]+)\s*[ ,]\s*(-?[0-9]+)\s*\]\s*", str(p["bitIdxRange"]), re.A)
        # one index, '[n]' or 'n', is the range [n n] (types and values equal)
        m1 = re.fullmatch(r"\s*(\[\s*)?(-?[0-9]+)(?(1)\s*\])\s*", str(p["bitIdxRange"]), re.A)
        if not m_ and not m1:
            # recordings (36 / 36): MATLAB forms evaluate element by element -- '[1.0 2]', '[[1] 2]',
            # '[int8(1) 2]', '[1+1 2]', '[0x1 2]', '[1, 1+2]', '[2 3];', '[true 2]' (true is 1, false 0)
            v_ = _index_vector(str(p["bitIdxRange"]))
            if v_ is None or len(v_) not in (1, 2):
                raise ValueError("Extract Bits bitIdxRange %r: \"[a b]\" or one index \"[n]\" / \"n\" required"
                                 % (p["bitIdxRange"],))
            a, b = (v_[0], v_[-1])
        else:
            a, b = (int(m_.group(1)), int(m_.group(2))) if m_ else (int(m1.group(2)), int(m1.group(2)))
        if a > b:
            # and recordings (4 / 4):
            # Simulink does not reorder a descending range -- it extracts the single bit a
            b = a
        lo, hi = a, b
        nb = hi - lo + 1
    if nb < 1:
        raise ParamUnrecorded("Extract Bits field of %d bit(s) is not recorded" % nb)
    if s and nb == 1:
        # a signed field needs 2 bits: widened downwards when anchored at the MSB, else upwards
        # (sign-extending past the top of the word)
        nb = 2
        if mode in ("Upper half", "Range starting with most significant bit"):
            lo = max(lo - 1, 0)
    return lo, nb


def _prop_eval(expr, env):
    """Evaluate a Data Type Propagation parameter expression (names NumBits1/2, Slope1/2, IsSigned1/2,
    Bias1/2; max([..]), min([..]); + - * / and or; TRUE/FALSE). Restricted parser, no eval of Python."""
    import ast
    import re

    def vec(m):  # MATLAB vector [a b -c] / [a, b] -> Python list [a, b, -c]
        items = [x for x in re.split(r"[\s,]+", m.group(1).strip()) if x]
        return "[" + ", ".join(items) + "]"
    t = re.sub(r"\[([^\[\]]*)\]", vec, expr.strip())
    t = t.replace("TRUE", "True").replace("FALSE", "False")
    tree = ast.parse(t, mode="eval")

    def ev(n):
        if isinstance(n, ast.Expression):
            return ev(n.body)
        if isinstance(n, ast.Constant) and isinstance(n.value, (int, float, bool)):
            return Fraction(n.value) if not isinstance(n.value, bool) else n.value
        if isinstance(n, ast.Name) and n.id in env:
            return env[n.id]
        if isinstance(n, (ast.Tuple, ast.List)):
            return [ev(e) for e in n.elts]
        if isinstance(n, ast.BoolOp):
            vals = [bool(ev(v)) for v in n.values]
            return all(vals) if isinstance(n.op, ast.And) else any(vals)
        if isinstance(n, ast.BinOp):
            a, b = ev(n.left), ev(n.right)
            ops = {ast.Add: lambda x, y: x + y, ast.Sub: lambda x, y: x - y, ast.Mult: lambda x, y: x * y,
                   ast.Div: lambda x, y: Fraction(x) / Fraction(y), ast.Pow: lambda x, y: Fraction(x) ** int(y)}
            return ops[type(n.op)](a, b)
        if isinstance(n, ast.UnaryOp) and isinstance(n.op, ast.USub):
            return -ev(n.operand)
        if isinstance(n, ast.UnaryOp) and isinstance(n.op, ast.Not):
            return not ev(n.operand)
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id in ("max", "min"):
            args = [ev(a) for a in n.args]
            if len(args) == 1 and isinstance(args[0], list):
                args = args[0]
            return (max if n.func.id == "max" else min)(args)
        raise ValueError("unsupported expression %r" % expr)
    return ev(tree)


def _ref_props(v):
    """(is signed, number of bits, slope) of a reference signal record."""
    s, w, f, _ = _decode(v)
    return bool(s), w, Fraction(2) ** (-f)


def _blk_dtprop(case):
    """Data Type Conversion with 'Inherit via back propagation' whose type comes from a Data Type Duplicate
    (= the other signal's type) or a Data Type Propagation block (signedness / word length / slope from the
    reference signals by the block's documented parameters, or best precision for given values)."""
    p = case["params"]
    src = case["ins"][0]
    if p["variant"] == "duplicate":
        t = _decode(case["ins"][1])[:3]
    else:
        (s1, n1, sl1), (s2, n2, sl2) = [_ref_props(v) for v in case["ins"][1:3]]
        env = {"IsSigned1": s1, "IsSigned2": s2, "NumBits1": Fraction(n1), "NumBits2": Fraction(n2),
               "Slope1": sl1, "Slope2": sl2, "Bias1": Fraction(0), "Bias2": Fraction(0)}
        sg = 1 if _prop_eval(p["ref_IsSigned"], env) else 0
        nb = _prop_eval(p["ref_NumBitsBase"], env) * _prop_eval(p["ref_NumBitsMult"], env) + \
            _prop_eval(p["ref_NumBitsAdd"], env)
        nb = max(int(nb), 2 if sg else 1)  # a signed type has at least 2 bits (computed 1 -> 2)
        if p["ref_PropScalingMode"].startswith("Obtain via best"):
            vals = _prop_eval(p["ref_ValuesUsedBestPrec"], {})
            vals = [Fraction(x) for x in (vals if isinstance(vals, list) else [vals])]
            fls = [_best_precision_fl(float(x), sg, nb) for x in vals if x != 0]
            fls = [f for f in fls if f != _BEST_PREC_FL_UNREPRESENTABLE]
            fl = min(fls)
        else:
            slope = _prop_eval(p["ref_SlopeBase"], env) * _prop_eval(p["ref_SlopeMult"], env) + \
                _prop_eval(p["ref_SlopeAdd"], env)
            slope = Fraction(slope)
            k = slope.denominator.bit_length() - 1 if slope.numerator == 1 else None
            if slope.numerator == 1 and slope.denominator == 1 << k:
                fl = k
            elif slope.denominator == 1 and slope.numerator & (slope.numerator - 1) == 0:
                fl = -(slope.numerator.bit_length() - 1)
            else:
                raise ValueError("non power-of-two slope %s" % slope)
        t = (sg, nb, fl)
    s, w, f = t
    x = _real_value(src)
    fm = _blk_fm(p)
    if p.get("RndMeth") == "Simplest" and "cls" in src and src["cls"] in ("double", "single"):
        fm = dict(fm, rm="Zero")  # 'Simplest' from floating point rounds toward zero
    return _out(s, w, f, _quant_fraction(x, s, w, f, fm))


def dti_gain(p):
    """The constant a Discrete-Time Integrator multiplies its input by each step (Forward Euler): gainval * T, T the
    sample period (the internal-rule output slope is the input slope times K*T; the engine
    passes T as _DtiT, absent = 1)."""
    T = Fraction(1) if str(p.get("IntegratorMethod", "")).startswith("Accumulation") else \
        Fraction(str(p.get("_DtiT", "1")))   # accumulation methods have no T
    return Fraction(float(p["gainval"])) * T


def dti_method(p):
    """'FE', 'BE' or 'TR' (Integration or Accumulation: the same steps at T = 1)."""
    m = str(p.get("IntegratorMethod", "Integration: Forward Euler"))
    return "BE" if m.endswith("Backward Euler") else "TR" if m.endswith("Trapezoidal") else "FE"


def _dti_type(p, it):
    """Discrete-Time Integrator output/state type: explicit fixdt, or the internal rule = the input type with
    its slope multiplied by the step constant K*T (dti_gain). Returns (s, w, recorded fl, exact slope)."""
    K = dti_gain(p)
    t = _named_type(p["OutDataTypeStr"])
    if isinstance(t, tuple):
        s, w, f = t
        return s, w, f, Fraction(2) ** (-f)
    s, w, f = it
    if dti_method(p) == "TR":
        K = K / 2   # recordings (19 trapezoidal designs): one fraction bit more, the step K/2
    slope = K * Fraction(2) ** (-f)
    # recorded fraction length of a slope type: -floor(log2(slope)) (slope adjustment factor in [1, 2))
    e = slope.numerator.bit_length() - slope.denominator.bit_length()
    if Fraction(2) ** e > slope:
        e -= 1
    return s, w, -e, slope


DTI_IC_RM = "Nearest"


def dti_explicit_refusal(p, it):
    """Why an explicit Discrete-Time Integrator output type is not answered, or None. recordings: the gain quantisation
    rule is not determined -- except for a power-of-two gain, which every quantisation keeps exact. Recorded: 159
    power-of-two modules of recordings (all equal apart from four with saturation on, an output coarser
    than the input and Ceiling / Zero / Convergent rounding -- that shape stays refused) and the loops of
    recordings"""
    t = _named_type(p["OutDataTypeStr"])
    if not isinstance(t, tuple):
        return None
    K = dti_gain(p)
    if K <= 0 or K.numerator & (K.numerator - 1) or K.denominator & (K.denominator - 1):
        return "an explicit output type with a gain that is not a power of two (the gain quantisation is not " \
               "determined)"
    if p.get("LimitOutput") == "on":
        try:
            ic, lo, hi = (float(p["InitialCondition"]), float(p["LowerSaturationLimit"]),
                          float(p["UpperSaturationLimit"]))
        except (KeyError, ValueError):
            ic = lo = hi = 0.0
        if ic < lo or ic > hi:   # cover the internal rule only
            return "an explicit output type with an initial condition outside the output limits (not recorded)"
    if p.get("SaturateOnIntegerOverflow") == "on" and t[2] < it[2] and \
            p.get("RndMeth", "Floor") not in ("Floor", "Nearest", "Round", "Simplest"):
        return "an explicit output type coarser than the input with saturation on and %s rounding (recorded " \
               "modules disagree)" % p.get("RndMeth")
    return None


def _blk_dti(case):
    """Discrete-Time Integrator, Forward Euler (integration or accumulation, T = 1): y[k] = x[k];
    x[k+1] = x[k] + round(K u[k] / slope) in the output type (block rounding / overflow); x[0] = IC
    quantised to the type; with LimitOutput the state is clamped to the quantised limits."""
    p = case["params"]
    h = _history(case)
    it = h[0][0][:3]
    s, w, frec, slope = _dti_type(p, it)
    fm = _blk_fm(p)
    K = dti_gain(p)
    lo, hi = (-(1 << (w - 1)), (1 << (w - 1)) - 1) if s else (0, (1 << w) - 1)

    def q(x, rm, om):
        v = Fraction(x) / slope
        return _overflow_plain(_round_quot(v.numerator, v.denominator, rm), s, w, om)
    limit = p.get("LimitOutput") == "on"
    if limit:
        ul, ll = p["UpperSaturationLimit"], p["LowerSaturationLimit"]
        up = hi if ul.strip() in ("inf", "Inf") else q(float(ul), "Nearest", "Saturate")
        dn = lo if ll.strip() in ("-inf", "-Inf") else q(float(ll), "Nearest", "Saturate")

    def clamp(x):
        return max(dn, min(up, x)) if limit else x
    x = clamp(q(float(p["InitialCondition"]), DTI_IC_RM, "Saturate"))
    x0_int = x
    explicit = isinstance(_named_type(p["OutDataTypeStr"]), tuple)
    if explicit and not os.environ.get("FXCORE_DTI_EXPLICIT_RECORD") and dti_explicit_refusal(p, it) is not None:
        # the engine refuses an explicit DTI output type (the gain quantisation is not
        # determined by the recorded data); the reference refuses it the same way instead of computing a value from a
        # rule that 218 of 229 development modules follow (the formula below is kept as a record, behind the switch)
        raise ValueError("Discrete-Time Integrator with an explicit output type: the gain quantisation is not "
                         "determined by the recorded data")
    if explicit:
        # explicit output type (recordings/§56d): the gain is double(K) rounded (nearest)
        # to the fraction length G = f_out + w_in - f_in - s_in (s_in = 1 signed input), the product gain * u is
        # formed in double, then cast; 218 of 229 recordings modules (the engine still refuses explicit types)
        us0, uw0, uf0, _ = h[0][0]
        G = frec + uw0 - uf0 - (1 if us0 else 0)
        gx = dti_gain(p) * (Fraction(2) ** G)
        kq = float(Fraction(_round_quot(gx.numerator, gx.denominator, "Nearest")) / (Fraction(2) ** G))
    meth = dti_method(p)
    rmode = str(p.get("ExternalReset", "none"))
    ext = str(p.get("InitialConditionSource", "internal")) == "external"
    if meth == "FE" and (rmode != "none" or ext):
        # recordings (25 designs, 25 / 25 bit-exact): Forward Euler with an external reset and /
        # or an external initial condition. The reset input (port 2 when present) triggers -- rising: from <= 0 to
        # > 0; falling: from > 0 to <= 0; either: both; level: non-zero, or a change from non-zero to zero; sampled
        # level: non-zero (the previous value starts at 0). On a trigger the state becomes the initial condition
        # (the external one at this step) before the output; then x += round(K u) as usual. The external initial
        # condition (the last port) is in the output type and sets the state at step 0.
        ri = 1 if rmode != "none" else None
        xi = (2 if ri else 1) if ext else None

        def trig(rp, r):
            if rmode == "rising":
                return rp <= 0 < r
            if rmode == "falling":
                return rp > 0 >= r
            if rmode == "either":
                return rp <= 0 < r or rp > 0 >= r
            if rmode == "level":
                return r != 0 or rp != 0
            return r != 0   # sampled level
        x, rp = None, 0
        for k in range(case["step"] + 1):
            us, uw, uf, un = h[k][0]
            ic = h[k][xi][3] if ext else x0_int
            if x is None:
                x = h[0][xi][3] if ext else x0_int
            if ri is not None:
                r = h[k][ri][3]
                if trig(rp, r):
                    x = ic
                rp = r
            if k == case["step"]:
                return _out(s, w, frec, x)
            inc = K * Fraction(un) / (Fraction(2) ** uf) / slope
            ov = _overflow if k == case["step"] - 1 else _overflow_plain
            x = ov(x + ov(_round_quot(inc.numerator, inc.denominator, fm["rm"]), s, w, fm["om"]), s, w, fm["om"])
    if meth != "FE":
        # recordings (48 designs, 48 / 48 bit-exact): with h_k = round(K u_k / slope) (K/2 for
        # trapezoidal), Backward Euler x_k = clamp(x_{k-1} + h_k), y_k = x_k; Trapezoidal y_k = clamp(x_{k-1} + h_k),
        # x_k = clamp(y_k + h_k); x_{-1} = the initial condition
        if explicit:
            raise ValueError("Discrete-Time Integrator %s with an explicit output type is not recorded" % meth)
        Kh = K / 2 if meth == "TR" else K
        y = x
        for k in range(case["step"] + 1):
            us, uw, uf, un = h[k][0]
            inc = Kh * Fraction(un) / (Fraction(2) ** uf) / slope
            ov = _overflow if k == case["step"] else _overflow_plain
            inc_n = ov(_round_quot(inc.numerator, inc.denominator, fm["rm"]), s, w, fm["om"])
            if meth == "BE":
                x = y = clamp(ov(x + inc_n, s, w, fm["om"]))
            else:
                y = clamp(ov(x + inc_n, s, w, fm["om"]))
                x = clamp(ov(y + inc_n, s, w, fm["om"]))
        return _out(s, w, frec, y)
    for k in range(case["step"]):
        us, uw, uf, un = h[k][0]
        if explicit:
            inc = Fraction(kq * math.ldexp(un, -uf)) / slope
        else:
            inc = K * Fraction(un) / (Fraction(2) ** uf) / slope
        # the history up to the previous step is recomputed every step: only the newest update is this step's
        ov = _overflow if k == case["step"] - 1 else _overflow_plain
        inc_n = ov(_round_quot(inc.numerator, inc.denominator, fm["rm"]), s, w, fm["om"])
        x = clamp(ov(x + inc_n, s, w, fm["om"]))
    return _out(s, w, frec, x)


# ---------------------------------------------------------------- 1-D Lookup Table
def _vec(text):
    return [float(x) for x in str(text).strip().strip("[]").replace(",", " ").split()]


def _lut_type(text, default):
    if text.startswith("Inherit"):
        return default
    t = _named_type(text)
    return t


def lut_flat_fm(p, tt, fm):
    """Rounding of a table value taken as it is (Flat / Nearest) into the output type. recordings
: with a double table type, 'Simplest' rounds toward zero
    (table -1.7 into fixdt(1,16,0) gives -1); the other six modes and fixed-point table types are as the block says."""
    # (_blk_fm has already turned 'Simplest' into Floor: the block's own parameter is read here)
    if tt == "double" and p.get("RndMeth") == "Simplest" and p.get("InterpMethod") in ("Flat", "Nearest"):
        return dict(fm, rm="Zero")
    return fm


_SETUP_CACHE = {}   # the multirate path ran the table setup on every step


def _setup_key(name, p, types):
    return (name, tuple(sorted((k, v) for k, v in p.items() if isinstance(v, (str, int, float)))),
            tuple(tuple(t) if isinstance(t, (list, tuple)) else t for t in types))


def _memo_setup(name, fn, p, types):
    """A table block's setup (pure in its parameters and input types) computed once per (parameters, types) and
    handed out as fresh lists, so a caller that changes one cannot change the cached copy. 442 s -> fast for an
    HDL NCO's 4096-entry sine table on the multirate path (recorded)."""
    key = _setup_key(name, p, types)
    hit = _SETUP_CACHE.get(key)
    if hit is None:
        hit = fn(p, *types)
        if len(_SETUP_CACHE) > 512:
            _SETUP_CACHE.clear()
        _SETUP_CACHE[key] = hit
    def fresh(x):   # lists copied at every level (values are immutable Fractions / numbers)
        return [fresh(y) for y in x] if isinstance(x, list) else x
    return tuple(fresh(x) for x in hit)


def _lut_setup(p, it):
    """Quantised 1-D table: (breakpoints as Fractions, their type, table values as Fractions, output type,
    table type, raw breakpoints, raw table). it = input type (s, w, f)."""
    return _memo_setup("lut1d", _lut_setup_raw, p, (it,))


def _lut_setup_raw(p, it):
    table = _vec(p["Table"])
    if p["BreakpointsSpecification"] == "Even spacing":
        x0, dx = float(p["BreakpointsForDimension1FirstPoint"]), float(p["BreakpointsForDimension1Spacing"])
        bps = [x0 + k * dx for k in range(len(table))]
    else:
        bps = _vec(p["BreakpointsForDimension1"])
    if len(bps) != len(table) or len(table) < 2:
        raise ValueError("1-D Lookup Table: breakpoints and table need the same length (>= 2)")
    bt = _lut_type(p["BreakpointsForDimension1DataTypeStr"], tuple(it))
    ot = p["OutDataTypeStr"]
    if ot == _SAME_FIRST:
        out_t = tuple(it)
    elif ot.startswith("Inherit"):  # back propagation: the Outport's default (double) in the recordings
        out_t = "double"
    else:
        out_t = _named_type(ot)
    tt = _lut_type(p["TableDataTypeStr"], out_t)
    bq = [Fraction(_quant_param(b, *bt)) / (Fraction(2) ** bt[2]) for b in bps]
    if any(b1 <= b0 for b0, b1 in zip(bq, bq[1:])):
        raise ValueError("1-D Lookup Table: breakpoints must be strictly increasing in their data type %s" % (bt,))
    if tt == "double":
        tq = [Fraction(v) for v in table]
    else:
        tq = [Fraction(_quant_param(v, *tt)) / (Fraction(2) ** tt[2]) for v in table]
    return bq, bt, tq, out_t, tt, bps, table


LUT1D_TABLE_RM = None
LUT1D_DIFF_EXACT = True   # slope from the table values as stored (fixes 2 modules, breaks none; recordings)


_LUT_SPEED_DOMAIN = None


def _lut_speed_domain():
    """engine/lut_speed_domain.json: the exact (input type, TableDataTypeStr, OutDataTypeStr) tuples of recordings
    recordings where x86-64 Speed matched the Precision form on every recorded step."""
    global _LUT_SPEED_DOMAIN
    if _LUT_SPEED_DOMAIN is None:
        import json as _json
        import os as _os
        f = _os.path.join(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))), "engine",
                          "lut_speed_domain.json")
        _LUT_SPEED_DOMAIN = {_json.dumps(t) for t in _json.load(open(f))} if _os.path.exists(f) else set()
    return _LUT_SPEED_DOMAIN


def lut1d_intermediate(p, out_t, tt, asic, in_t=None):
    """Intermediate type of the 1-D linear point-slope table as the arithmetic uses it.
    Explicit / 'Same as output': as written, on x86-64 widened to a native word 8/16/32 (recordings types
    350/350); ASIC/FPGA as written. 'Inherit via internal rule' (116 of 118 modules,
    and the Fixed-Point Tool read-out; form first proposed by recordings): signed; with Wt, Ft the table type and
    F = 32 (UseLastTableValue on) or 31 (off): ASIC/FPGA Precision (Wt + 1 + F, Ft + F), ASIC/FPGA Speed (Wt + 1, Ft),
    x86-64 Precision (32, Ft + 31 - Wt). x86-64 Speed is refused (2 of 35 development modules contradict every form
    tried), as is an explicit fraction type with this rule (not recorded)."""
    txt = str(p["IntermediateResultsDataTypeStr"])
    if "internal rule" not in txt:
        it = _lut_type(txt, out_t)
        if it != "double" and not asic:
            it = (it[0], _container32(it[1], (8, 16, 32)), it[2])
        return it
    if not str(p.get("FractionDataTypeStr", _INHERIT_RULE)).startswith("Inherit"):
        raise ValueError("intermediate type 'Inherit via internal rule' with an explicit fraction type is not recorded")
    if not isinstance(tt, tuple):
        raise ValueError("intermediate type 'Inherit via internal rule' with a double table type is not recorded")
    F = 32 if p.get("UseLastTableValue") == "on" else 31
    prio = p.get("InternalRulePriority", "Speed")
    if asic:
        return (1, tt[1] + 1 + F, tt[2] + F) if prio == "Precision" else (1, tt[1] + 1, tt[2])
    if prio == "Precision":
        if tt[1] > 32:
            # a table word over 32 bits -- (32, Ft + 31 - Wt)
            # drops the table's low bits where Simulink stays exact; the rule there is not recorded
            raise ValueError("intermediate type 'Inherit via internal rule' on x86-64 with a table word of %d bits "
                             "(over 32) is not determined by the recorded data" % tt[1])
        return (1, 32, tt[2] + 31 - tt[1])
    import json as _json
    if lut1d_speed_signed_ok(p, in_t, tt):
        # recordings (Speed paired with Precision) with
        # x86-64 Speed with a signed 16- or 32-bit table takes the signed 32-bit
        # word with the table's fraction, one bit less where the table has 32 bits: fraction Ft + min(0, 31 - Wt)
        # (63 designs, Floor / Nearest / Zero, UseLastTableValue off / on: every step equal; the Precision form
        # (32, Ft + 31 - Wt) is 1 LSB off under Zero)
        return (1, 32, tt[2] + min(0, 31 - tt[1]))
    # (the native minimal counterexamples, recordings: RndMeth Zero was
    # one LSB off; recordings show these tuples with RndMeth Simplest, the library default
    # -- Simplest / Floor only, Zero / Nearest / Ceiling / Round / Convergent refused)
    if in_t is not None and tt[1] <= 32 and p.get("UseLastTableValue", "off") == "off" and \
            str(p.get("RndMeth", "Floor")).strip() in ("Simplest", "Floor") and _json.dumps(
            [list(in_t), str(p.get("TableDataTypeStr", "")).strip(),
             str(p.get("OutDataTypeStr", "Inherit: Same as first input")).strip()]) in _lut_speed_domain():
        # recordings (every input value of each recorded design): on x86-64 Speed takes the
        # Precision form for these exact tuples; elsewhere the recordings contradict every single form tried
        # (unsigned tables, an 8-bit table, sfix16_En10 into fixdt(1,20,14)) -- still refused there. UseLastTableValue
        # off only: the recording had it off; with it on the native slice found 4 of 10 wrong.
        # RndMeth Simplest (the library default the recording ran with: dut_params) or Floor only: with Zero the
        # native testing found 1-LSB differences
        return (1, 32, tt[2] + 31 - tt[1])
    raise ValueError("intermediate type 'Inherit via internal rule' with InternalRulePriority Speed on x86-64 is not "
                     "determined by the recorded data (2 of 35 development modules contradict it)")


# the recorded span of the x86-64 Speed rule for signed 16- / 32-bit tables (
# recordings): inputs 8..16 bits, table fixdt(1,16,10..14) or fixdt(1,32,0), outputs as
# recorded, inherited breakpoint / fraction types, Clip, saturation off, Floor / Nearest / Zero
LUT1D_SPEED_SIGNED_OUTS = ("Inherit: Same as first input", "fixdt(1,16,8)", "fixdt(1,20,14)")


def lut1d_speed_signed_ok(p, in_t, tt):
    if not (isinstance(in_t, tuple) and isinstance(tt, tuple) and tt[0] == 1):
        return False
    if not ((tt[1] == 16 and 10 <= tt[2] <= 14) or
            (tt[1] == 32 and tt[2] == 0 and str(p.get("UseLastTableValue", "off")).strip() == "off")):
        # a 32-bit table with UseLastTableValue on differs on 3 of 3 recorded designs
        return False
    if not (8 <= in_t[1] <= 16 and 0 <= in_t[2] <= in_t[1]):
        return False
    return str(p.get("OutDataTypeStr", "Inherit: Same as first input")).strip() in LUT1D_SPEED_SIGNED_OUTS and \
        str(p.get("RndMeth", "Floor")).strip() in ("Floor", "Nearest", "Zero", "Simplest") and \
        str(p.get("SaturateOnIntegerOverflow", "off")).strip() == "off" and \
        str(p.get("ExtrapMethod", "Clip")).strip() == "Clip" and \
        str(p.get("FractionDataTypeStr", _INHERIT_RULE)).strip() == _INHERIT_RULE and \
        str(p.get("BreakpointsForDimension1DataTypeStr", "Inherit: Same as corresponding input")).strip() == \
        "Inherit: Same as corresponding input"


LUT1D_UNSIGNED_FLOOR_BASE = 32   # P >= 32 - max(Wt, C) with unsigned breakpoints


def lut1d_inherited_fraction(p, tq, it, out_t, tt, asic=False, bt=None, in_t=None):
    """FractionDataTypeStr = 'Inherit: Inherit via internal rule' of the 1-D linear point-slope table
. Where the data decide it
    -- every table value and every step between neighbouring table values representable in the intermediate type
    (lut1d_intermediate) -- the fraction is unsigned with F fraction bits:
      Precision: F = 32 with UseLastTableValue on, 31 with it off (word 32);
      Speed:     F = max(0, Wt + Fi - Ft) (table word / fraction Wt, Ft; intermediate fraction Fi), word F + 1 with
                 UseLastTableValue off (1 must be representable), max(1, F) with it on; on x86-64 the word is at
                 most 32 bits (ASIC/FPGA: no limit seen); with the internal intermediate rule Fi = Ft.
    bit-exact where computed, the rest refused; the one development counterexample (recordings
    8009/24, Speed + Ceiling on x86-64) put that class in the refused range.
    The F rule was first proposed by an independent derivation and then checked here against
    the recorded outputs. Elsewhere a ValueError names the condition."""
    internal = "internal rule" in str(p.get("IntermediateResultsDataTypeStr", ""))
    if it == "double" or out_t == "double" or not isinstance(tt, tuple):
        raise ValueError("inherited fraction type with a double intermediate, table or output type is not recorded")
    lo = (-(Fraction(2) ** (it[1] - 1)) if it[0] else Fraction(0)) / (Fraction(2) ** it[2])
    hi = (Fraction(2) ** (it[1] - it[0]) - 1) / (Fraction(2) ** it[2])
    if not all(lo <= v <= hi for v in tq):
        raise ValueError("inherited fraction type: a table value lies outside the intermediate type %s; this case is "
                         "not determined by the recorded data" % _tstr_short(it))
    ult = p.get("UseLastTableValue") == "on"
    if not asic and tuple(it) == tuple(tt) and it[0] == 0 and it[1] in (8, 16) and \
            any(tq[j + 1] < tq[j] for j in range(len(tq) - 1)):
        # recordings (19 models, 16-bit inputs): a falling table whose table type is the
        # unsigned intermediate type rounds the fraction to 2^-16 (the falling term uses the step's magnitude,
        # lut1d_magnitude_slope); the rising steps must still fit. Speed priority only (the recorded default)
        if not all(tq[j + 1] - tq[j] <= hi for j in range(len(tq) - 1)):
            raise ValueError("inherited fraction type: a rising step lies outside the intermediate type %s"
                             % _tstr_short(it))
        if p.get("InternalRulePriority", "Speed") != "Speed":
            raise ValueError("inherited fraction type of a falling unsigned table: recorded with InternalRulePriority "
                             "Speed only")
        return "fixdt(0,16,16)" if ult else "fixdt(0,17,16)"
    if not all(lo <= tq[j + 1] - tq[j] <= hi for j in range(len(tq) - 1)):
        raise ValueError("inherited fraction type: a step between neighbouring table values lies outside the "
                         "intermediate type %s; this case is not determined by the recorded data" % _tstr_short(it))
    prio = p.get("InternalRulePriority", "Speed")
    if prio == "Precision":
        return "fixdt(0,32,32)" if ult else "fixdt(0,32,31)"
    if prio != "Speed":
        raise ValueError("InternalRulePriority %r not recorded" % prio)
    if not asic and p.get("RndMeth") == "Ceiling" and not (isinstance(bt, tuple) and bt[0] == 1):
        # signed breakpoint types: recordings show Ceiling with P = Wt + Fi - Ft (14 models,
        # sfix16_En8 and fixdt(1,32,32) breakpoints); the counterexample below has an unsigned input
        # recordings (re-recorded): the rule gives F = 10 where Simulink needs F >= 11, and no tested
        # variant explains it without breaking matching modules; the class with that counterexample is refused
        raise ValueError("inherited fraction type with InternalRulePriority Speed and Ceiling rounding on x86-64 is "
                         "not determined by the recorded data (1 of 26 development modules contradicts the rule)")
    F = max(0, tt[1] + (tt[2] if internal else it[2]) - tt[2])   # internal intermediate rule: Fi = Ft
    if not asic and isinstance(bt, tuple) and bt[0] == 0:
        # recordings (50 models): with an unsigned breakpoint type Simulink's fraction
        # precision has a floor, P = max(Wt + Fi - Ft, 32 - max(Wt, C)), C the input's container width (8 / 16 / 32;
        # the breakpoints inherit the input's type there). Where the floor does not raise F and F >= 16 the earlier
        # rule holds (recorded exhaustive Simulink checks);
        # a raised P is answered only in the recorded scope
        cw = (in_t or bt)[1]   # the input's container width (the breakpoint type where the input is not given)
        C = 8 if cw <= 8 else 16 if cw <= 16 else 32
        P = max(F, LUT1D_UNSIGNED_FLOOR_BASE - max(tt[1], C))
        if not (P == F and F >= 16):
            same_in = str(p.get("BreakpointsForDimension1DataTypeStr", "Inherit: Same as corresponding input")) \
                .startswith("Inherit: Same as corresponding input")
            why = None
            if not same_in:
                why = "an explicit unsigned breakpoint type (recorded: breakpoints inheriting the input type)"
            elif bt[1] > 24:
                why = "a %d-bit input (recorded up to 24 bits)" % bt[1]
            elif not 8 <= tt[1] <= 16:
                why = "a %d-bit table (recorded 8..16)" % tt[1]
            elif p.get("RndMeth", "Floor") not in ("Floor", "Nearest"):
                why = "rounding %s (recorded Floor / Nearest)" % p.get("RndMeth")
            elif tuple(it) != tuple(out_t):
                why = "an intermediate type other than the output type (recorded: intermediate = output)"
            if why:
                raise ValueError("inherited fraction type with InternalRulePriority Speed on x86-64 and an unsigned "
                                 "breakpoint type %s: Wt + Fi - Ft = %d, floor 32 - max(Wt, C) = %d; not recorded "
                                 "with %s"
                                 % (_tstr_short(bt), F, 32 - max(tt[1], C), why))
            F = P
    if not asic:   # x86-64: the fraction word is at most 32 bits (recordings fit only F = 31)
        F = min(F, 32 if ult else 31)
    return "fixdt(0,%d,%d)" % ((max(1, F), F) if ult else (F + 1, F))


class LutUnrecorded(ValueError):
    """A run reaches a lookup event whose Simulink behaviour is not recorded (the run stops there, naming the step)."""


LUT_ROUND1_MSG = ("1-D Lookup Table, UseLastTableValue on: the interpolation fraction rounds to 1 here; whether "
                  "Simulink then saturates it below 1 or takes the next table value is recorded on the host for "
                  "fraction words up to 32 bits (Nearest / Round / Ceiling; Convergent at 8 / 16 / 32) and on ASIC / "
                  "FPGA for Nearest / Round / Ceiling up to 32 bits, not %s. Use "
                  "Floor rounding (the fraction never rounds up) or UseLastTableValue off")


def lut1d_frac_carry(p, ff=None, bt=None, asic=False):
    """What a fraction that rounds to 1 does: True = the next table value (carry), False = saturates below 1,
    "refuse" = not recorded: the run stops at the first step where it happens (LutUnrecorded / the C core's
    E UNRECORDED) -- a static refusal would also refuse the many tables that never round a fraction to 1.
    Inherited fraction types only. UseLastTableValue off: carry. On: see the rule below (explicit or
    even spacing makes no difference); Convergent / Simplest outside the saturating words, host words over 32 and
    ASIC words over 32 are not recorded. Floor / Zero
    never round a fraction in [0, 1) up to 1; Round is Nearest for a fraction >= 0."""
    if not (LUT1D_FRAC_CARRY and str(p.get("FractionDataTypeStr", _INHERIT_RULE)).startswith("Inherit")):
        return False
    if p.get("UseLastTableValue") != "on":
        return True
    rm = p.get("RndMeth", "Floor")
    if rm in ("Floor", "Zero") or ff is None:
        return False
    # recordings (F = 4..24, Nearest / Round, explicit and
    # even spacing alike) with recordings (F = 4..32 and 64, Nearest /
    # Round / Ceiling, host and ASIC): on the host a fraction word of 8, 16 or 32 saturates (32) and
    # every other word up to 31 carries; ASIC / FPGA saturates for every word up to 32 (64 not distinguishable)
    if asic:
        return False if rm in ("Nearest", "Round", "Ceiling") and ff <= 32 else "refuse"
    if ff in (8, 16, 32):
        return False if rm in ("Nearest", "Round", "Ceiling", "Convergent") else "refuse"
    if ff <= 31 and rm in ("Nearest", "Round", "Ceiling"):
        return True
    return "refuse"


def lut1d_round1_why(p, ff, bt, asic):
    rm = p.get("RndMeth", "Floor")
    return ("on ASIC / FPGA" if asic else "with rounding %s" % rm if rm not in ("Nearest", "Round", "Ceiling") else
            "with a %d-bit fraction and breakpoint type %s" % (ff, _tstr_short(bt) if isinstance(bt, tuple) else bt))


def lut1d_fraction_by_gap(ft, inherited, gap, p, tq, in_signed, asic, it_w=None):
    """The fraction type actually used when the input is finer than the breakpoints (gap = input fraction length -
    breakpoint fraction length). ft is the fraction type the block would use otherwise (explicit, or
    lut1d_inherited_fraction's), F its fraction length. Recorded (F = 8, gaps 7..16;
    recordings: F = 12 / 15, gaps 7..16; recordings: F = 16 / 17, gaps
    8 / 9): the fraction is rounded to 2^-P with P = max(F, gap) for gap <= 15 and P = F at gap 16. Answered:
      gap <= 0, or gap <= 7 and <= F: F (unchanged; the earlier recordings);
      8 <= gap <= 16 and (gap < F or gap = 16): F -- host, inherited fraction type, Floor; signed inputs on rising
                                      tables, unsigned inputs at gaps 8 / 9 (recorded packages, rising
                                      and falling);
      8 <= gap <= 15 and gap >= F:    P = gap -- host, signed input, Floor / Nearest, inherited, rising table;
      P = gap holds only for an intermediate word of at most 16
      bits; above 16 (17 / 24 / 32 seen) Simulink keeps P = F at gaps 8..15 (9 blocks fitted, 49 block-settings in
      recordings, Floor and Nearest) -- answered as F there (signed input, rising table); an unknown
      intermediate word (it_w None) is refused where the two differ;
      anything else refused (ValueError naming the condition)."""
    fs, fw, ff = _named_type(ft)
    if gap <= 0 or (gap <= 7 and gap <= ff):
        return ft
    rm = p.get("RndMeth", "Floor")
    falling = any(b < a for a, b in zip(tq, tq[1:]))
    wide_it = it_w is not None and it_w > 16
    keep = gap < ff or gap == 16 or wide_it
    why = None
    if it_w is None and not (gap < ff or gap == 16):
        why = "with an unknown intermediate word (P = gap holds only up to 16 bits)"
    if why:
        pass
    elif gap > 16:
        why = "outside the recorded gaps (up to 16)"
    elif gap <= 7:
        why = "below 8 but above the fraction type's %d bits (not recorded)" % ff
    elif not inherited:
        why = "with an explicit fraction type (recorded with the inherited fraction type only)"
    elif asic:
        why = "on ASIC / FPGA (recorded on the host only)"
    elif keep and wide_it and not (gap < ff or gap == 16):
        if not in_signed:
            why = "with an unsigned input and an intermediate word over 16 bits (signed inputs only)"
        elif rm not in ("Floor", "Nearest"):
            why = "with rounding %s and an intermediate word over 16 bits (Floor / Nearest)" % rm
        elif falling:
            why = "on a table with a falling segment (rising tables only)"
    elif keep:
        if rm != "Floor":
            why = "with rounding %s below the fraction type's precision (recorded with Floor only)" % rm
        elif not in_signed and gap not in (8, 9):
            why = "with an unsigned input at gap %d (recorded for unsigned inputs at gaps 8 / 9 only)" % gap
        elif in_signed and falling:
            why = "on a table with a falling segment (recorded on rising tables only for signed inputs)"
    else:
        if not in_signed:
            why = "with an unsigned input (recorded with signed inputs only)"
        elif rm not in ("Floor", "Nearest"):
            why = "with rounding %s (recorded with Floor and Nearest only)" % rm
        elif falling:
            why = "on a table with a falling segment (recorded on rising tables only)"
    if why:
        # testing: say what is answered and how to get there -- every shape answers a gap of at most
        # min(7, F) bits; the wider gaps only in the recorded host / inherited-fraction shapes
        lim = min(7, ff)
        raise ValueError("the input has %d fraction bits more than the breakpoints, and Simulink's interpolation "
                         'fraction for that gap is not recorded %s'
                         '. Answered in every shape: a gap of at most %d bits '
                         "(7, and not above the fraction type's %d bits) -- give the breakpoints %d more fraction "
                         "bits, or convert the input to %d fewer, before the table"
                         % (gap, why, lim, ff, gap - lim, gap - lim))
    if keep:
        return ft
    return "fixdt(0,%d,%d)" % ((gap, gap) if p.get("UseLastTableValue") == "on" else (gap + 1, gap))


def lut2d_gaps(p, its):
    """Input fraction length minus breakpoint fraction length, per dimension of a 2-D table."""
    out = []
    for k in (1, 2):
        b = _lut_type(str(p.get("BreakpointsForDimension%dDataTypeStr" % k, "Inherit: Same as corresponding input")),
                      tuple(its[k - 1]))
        out.append(its[k - 1][2] - b[2] if isinstance(b, tuple) else 0)
    return out


def lut2d_fraction_gap_ok(ft, gaps):
    """2-D: only the gaps where the fraction type's precision is used unchanged (gap <= 7 and <= its fraction length);
    there is one 2-D model (gap 12, P = 12) -- too little to tell the 1-D rules apart."""
    ff = _named_type(ft)[2]
    for k, g in enumerate(gaps):
        if not (g <= 7 and g <= ff):
            raise ValueError("input %d has %d fraction bits more than its breakpoints; Simulink's fraction precision "
                             "for that gap is not determined for 2-D tables (there is one "
                             "2-D model)" % (k + 1, g))
    return ft


def _tstr_short(t):
    return "%sfix%d_En%d" % ("s" if t[0] else "u", t[1], t[2])


def lut_linear_types_recorded(p):
    """Linear interpolation: the intermediate and fraction type forms the recorded data cover; a ValueError names any
    other (signed explicit fraction types reached the compiler; no recordings module has one, and the
    earlier signed-saturation arithmetic for them was never checked against Simulink).
      intermediate: explicit fixdt / integer name, 'Inherit: Same as output', 'Inherit: Inherit via internal rule';
      fraction:     the inherited rule, or an explicit unsigned fixdt / unsigned integer name."""
    itxt = str(p.get("IntermediateResultsDataTypeStr", "Inherit: Same as output"))
    if itxt not in ("Inherit: Same as output", _INHERIT_RULE) and not isinstance(_named_type(itxt), tuple):
        raise ValueError("intermediate type %r is not one of the recorded forms (explicit fixdt, 'Inherit: Same as "
                         "output', 'Inherit: Inherit via internal rule')" % itxt)
    ftxt = str(p.get("FractionDataTypeStr", _INHERIT_RULE))
    if ftxt != _INHERIT_RULE:
        ft = _named_type(ftxt)
        if not isinstance(ft, tuple) or ft[0] != 0:
            raise ValueError("fraction type %r: only the inherited rule or an unsigned fixed-point type is recorded "
                             "(no recordings module has a signed or non-fixed-point fraction type)" % ftxt)


LUT1D_FRAC_CARRY = True   # a fraction that rounds to 1 is 1 (inherited fraction type)


def lut1d_magnitude_slope(tt, it, asic):
    """recordings: on the host, with a 32-bit table type, Simulink forms
    the slope term of a falling segment from the magnitude, y0 - round(f * (y0 - y1)) -- the signed difference of two
    32-bit values does not fit the long word. ASIC/FPGA (no native words) uses the signed term (516 / 516 tables)."""
    # recordings (19 models, 968659 steps): also an unsigned 8 / 16-bit table type that is
    # the intermediate type (the library-default shape of a falling table): y0 - R(f * |y1 - y0|), f rounded first
    return (not asic) and isinstance(tt, tuple) and (tt[1] == 32 or (
        tt[0] == 0 and tt[1] in (8, 16) and isinstance(it, tuple) and tuple(it) == tuple(tt)))


def _blk_lut1d(case):
    p = case["params"]
    s, w, f, n = _ins(case)[0]
    bq, bt, tq, out_t, tt, _bps, _table = _lut_setup(p, (s, w, f))
    fm = _blk_fm(p)
    u = Fraction(n) / (Fraction(2) ** f)
    meth = p["InterpMethod"]
    last = len(bq) - 1
    if meth == "Linear point-slope" and p["ExtrapMethod"] != "Clip":
        raise ValueError("linear extrapolation with linear interpolation not recorded")
    if meth == "Linear point-slope":
        lut_linear_types_recorded(p)
    uu = min(max(u, bq[0]), bq[last])  # Clip (Flat / Nearest: 'Linear' extrapolation behaves the same)
    k = 0
    while k < last - 1 and uu >= bq[k + 1]:
        k += 1
    if meth == "Flat":
        y = tq[k + 1] if uu >= bq[k + 1] else tq[k]
    elif meth == "Nearest":
        y = tq[k + 1] if (uu - bq[k]) >= (bq[k + 1] - uu) else tq[k]
    else:  # point-slope in the intermediate type: y0 + round(frac * (y1 - y0)), then round the sum
        it = lut1d_intermediate(p, out_t, tt, _asic(case.get("hw")), (s, w, f))
        frac = (uu - bq[k]) / (bq[k + 1] - bq[k])

        def qi(x):
            return Fraction(_quant_fraction(x, it[0], it[1], it[2], fm)) / (Fraction(2) ** it[2])

        def qs(x):   # the slope term frac * (y1 - y0): signed, one bit wider than an unsigned intermediate type
            return Fraction(_quant_fraction(x, 1, it[1] + (0 if it[0] else 1), it[2], fm)) / (Fraction(2) ** it[2])

        def qt(x):   # table value into the intermediate type (rounding: LUT1D_TABLE_RM, None = the block's)
            f2 = fm if LUT1D_TABLE_RM is None else dict(fm, rm=LUT1D_TABLE_RM)
            return Fraction(_quant_fraction(x, it[0], it[1], it[2], f2)) / (Fraction(2) ** it[2])
        ft = p.get("FractionDataTypeStr", _INHERIT_RULE)
        if it == "double":  # all in double: frac first, then y0 + frac * (y1 - y0)
            fd = float(uu - bq[k]) / float(bq[k + 1] - bq[k])
            y = Fraction(float(tq[k]) + fd * (float(tq[k + 1]) - float(tq[k])))
        elif p.get("UseLastTableValue") == "on" and uu >= bq[last]:
            y = qi(tq[last])  # at / beyond the last breakpoint the last table value itself
        else:
            inh_ = str(ft).startswith("Inherit")
            if inh_:
                ft = lut1d_inherited_fraction(p, tq, it, out_t, tt, _asic(case.get("hw")), bt, (s, w, f))  # ValueError outside
            ft = lut1d_fraction_by_gap(ft, inh_, f - bt[2], p, tq, bool(s), _asic(case.get("hw")),
                                       it[1] if isinstance(it, tuple) else None)
            if True:  # the fraction type: quantised with the block's rounding
                fs, fw, ff = _named_type(ft)
                fq = _quant_fraction(frac, fs, fw, ff, {"rm": fm["rm"], "om": "Saturate"})
                mode_ = lut1d_frac_carry(p, ff, bt, _asic(case.get("hw")))
                if mode_ and _round_quot((frac * (Fraction(2) ** ff)).numerator,
                                         (frac * (Fraction(2) ** ff)).denominator, fm["rm"]) >= (1 << ff):
                    if mode_ == "refuse":
                        raise LutUnrecorded(LUT_ROUND1_MSG % lut1d_round1_why(p, ff, bt, _asic(case.get("hw"))))
                    # an input one LSB below
                    # a breakpoint rounds the fraction to 1 -- Simulink then gives the next table value (y1), the
                    # engine had saturated the fraction to the largest value below 1
                    fq = 1 << ff
                frac = Fraction(fq) / (Fraction(2) ** ff)
            if LUT1D_DIFF_EXACT:   # slope from the table values as stored (table type), then the product rounded
                y0 = qt(tq[k])
                dstep = tq[k + 1] - tq[k]
                if dstep < 0 and lut1d_magnitude_slope(tt, it, _asic(case.get("hw"))):
                    y = qi(y0 - qs(frac * (-dstep)))
                elif dstep < 0 and isinstance(it, tuple) and it[0] == 0:
                    # with an unsigned intermediate type that
                    # is not the table type (or on ASIC / FPGA), the slope term of a falling segment is cast into the
                    # intermediate type itself with the block's overflow mode -- wrapped, or 0 with saturation on
                    # -- then added to y0
                    y = qi(y0 + qi(frac * dstep))
                else:
                    y = qi(y0 + qs(frac * dstep))
            else:
                y0, y1 = qt(tq[k]), qt(tq[k + 1])
                y = qi(y0 + qs(frac * (y1 - y0)))
    if out_t == "double":
        return {"cls": "double", "num": _fmt_double(float(y))}
    os_, ow, of = out_t
    return _out(os_, ow, of, _quant_fraction(y, os_, ow, of, lut_flat_fm(p, tt, fm)))


def _mat(text):
    rows = [r for r in str(text).strip().strip("[]").split(";")]
    return [[float(x) for x in r.replace(",", " ").split()] for r in rows if r.strip()]


ORDER_2D = "dim1_first"
# inherited fraction type of 2-D linear interpolation: lut2d_inherited_fraction; the earlier
# guess "unsigned, intermediate word, one integer bit" (1747 / 1760 steps of recordings) is superseded


LUT2D_DIFF_EXACT = True


def _lut2d_setup(p, its):
    """2-D Lookup Table: (quantised breakpoints per dimension, table quantised to the table type, output type)
    for input types its = [(s, w, f), (s, w, f)]."""
    return _memo_setup("lut2d", lambda p_, its_: _lut2d_setup_raw(p_, list(its_)), p, (tuple(tuple(t) for t in its),))


def _lut2d_setup_raw(p, its):
    table = _mat(p["Table"])
    out_t_txt = p["OutDataTypeStr"]
    if out_t_txt == _SAME_FIRST:
        out_t = tuple(its[0])
    elif out_t_txt.startswith("Inherit"):
        out_t = "double"
    else:
        out_t = _named_type(out_t_txt)
    tt = _lut_type(p["TableDataTypeStr"], out_t)
    bqs = []
    for dim in (1, 2):
        if p["BreakpointsSpecification"] == "Even spacing":
            x0 = float(p["BreakpointsForDimension%dFirstPoint" % dim])
            dx = float(p["BreakpointsForDimension%dSpacing" % dim])
            cnt = len(table) if dim == 1 else len(table[0])
            bps = [x0 + k * dx for k in range(cnt)]
        else:
            bps = _vec(p["BreakpointsForDimension%d" % dim])
        bt = _lut_type(p["BreakpointsForDimension%dDataTypeStr" % dim], tuple(its[dim - 1]))
        bq = [Fraction(_quant_param(b, *bt)) / (Fraction(2) ** bt[2]) for b in bps]
        if any(b1 <= b0 for b0, b1 in zip(bq, bq[1:])):   # require strictly increasing quantized breakpoints
            raise ValueError("2-D Lookup Table: breakpoints of dimension %d must be strictly increasing in their "
                             "data type %s" % (dim, bt))
        bqs.append(bq)
    if len(bqs[0]) != len(table) or any(len(r) != len(bqs[1]) for r in table):
        raise ValueError("2-D Lookup Table: table size does not match the breakpoints")
    tq = [[Fraction(v) if tt == "double" else Fraction(_quant_param(v, *tt)) / (Fraction(2) ** tt[2]) for v in r]
          for r in table]
    return bqs, tq, out_t


def _lut2d_itype(p, out_t, asic=False):
    """Intermediate type of a linear 2-D table. Internal rule (recordings.types, 113 linear modules):
    Precision -> 32 bits with one more integer bit than the output; Speed -> 16 bits with one more integer bit
    when that fits, else the output type when it fits 16 bits, else 32 bits with the output's fraction length."""
    itxt = p["IntermediateResultsDataTypeStr"]
    if itxt == _INHERIT_RULE and out_t != "double" and not asic and \
            p.get("InternalRulePriority", "Speed") == "Precision":
        # the audited class (recordings is Precision; the Speed rule below is the recorded one)
        tt_ = _lut_type(p["TableDataTypeStr"], out_t)
        if tt_ != out_t or (isinstance(tt_, tuple) and tt_[1] > 32):
            # on x86-64 Simulink follows the table type here,
            # not the output type this rule uses (int16 table, int32 output: -12345 vs -12346), and a table word
            # over 32 bits keeps the table exact; only table type = output type of at most 32 bits is answered
            raise ValueError("2-D intermediate type 'Inherit via internal rule' on x86-64 with a table type %s other "
                             'than the output type or wider than 32 bits is not determined by the recorded data'
                             "" % (tt_,))
    if itxt == _INHERIT_RULE and out_t != "double":
        if not str(p.get("FractionDataTypeStr", _INHERIT_RULE)).startswith("Inherit"):
            # recordings (4 of 48 tables differ; ASIC 13 of 48): the
            # internal-rule intermediate type with an explicit fraction type is not what this rule gives -- refused,
            # as the 1-D table refuses the same class
            raise ValueError("2-D intermediate type 'Inherit via internal rule' with an explicit fraction type is not "
                             "determined by the recorded data")
        os0, ow0, of0 = out_t
        oi = ow0 - of0
        if p.get("InternalRulePriority", "Speed") == "Precision":
            return (1, 32, 32 - oi - 1)
        if ow0 + 1 <= 16:
            return (1, 16, 16 - oi - 1)
        if ow0 <= 16:
            return out_t
        return (1, 32, of0)
    it = _lut_type(itxt, out_t)
    if it != "double" and not asic:   # ASIC/FPGA: the word as written (as 1-D)
        it = (it[0], _container32(it[1], (8, 16, 32)), it[2])
    return it


def lut2d_inherited_fraction(p, tq, it, asic, out_t):
    """FractionDataTypeStr = 'Inherit: Inherit via internal rule' of the 2-D linear table, both dimensions
: where the data decide it -- InternalRulePriority Precision, every table
    value and every step between neighbouring table values along either dimension representable in the intermediate
    type, and an intermediate type that is explicit / 'Same as output', or the internal rule on x86-64 -- the
    fraction is fixdt(0,32,32) with UseLastTableValue on, fixdt(0,32,31) with it off (77 of 77
    modules; recordings: 28 of 28). Speed: on ASIC/FPGA only, explicit /
    'Same as output' intermediate (Wi, Fi) as written, same table-range condition, and Wt + Fi - Ft <= Wi: F = Wi
    fraction bits, word F (UseLastTableValue on) or F + 1 (29 of 29; recordings 3 more).
    Elsewhere a ValueError names the condition."""
    prio = p.get("InternalRulePriority", "Speed")
    if prio not in ("Precision", "Speed"):
        raise ValueError("InternalRulePriority %r not recorded" % prio)
    if prio == "Speed" and not asic:
        raise ValueError("2-D inherited fraction type with InternalRulePriority Speed on x86-64 is not determined by "
                         "the recorded data (2 of 47 modules contradict every form tried)")
    if it == "double":
        raise ValueError("2-D inherited fraction type with a double intermediate type is not recorded")
    tt = _lut_type(p["TableDataTypeStr"], out_t)
    if out_t == "double" or not isinstance(tt, tuple):
        # no recordings module has a double table or output with a fixed-point intermediate type (1-D refuses
        # the same class)
        raise ValueError("2-D inherited fraction type with a double table or output type is not recorded")
    if asic and str(p["IntermediateResultsDataTypeStr"]) == _INHERIT_RULE:
        raise ValueError("2-D inherited fraction type with the internal intermediate rule on ASIC/FPGA is not "
                         "determined by the recorded data (%s)" % ("12 of 14 modules" if prio == "Precision" else
                                                                    "Speed: not recorded within the table range"))
    lo = (-(Fraction(2) ** (it[1] - 1)) if it[0] else Fraction(0)) / (Fraction(2) ** it[2])
    hi = (Fraction(2) ** (it[1] - it[0]) - 1) / (Fraction(2) ** it[2])
    vals = [v for row in tq for v in row]
    steps = [tq[i + 1][j] - tq[i][j] for i in range(len(tq) - 1) for j in range(len(tq[0]))] + \
            [tq[i][j + 1] - tq[i][j] for i in range(len(tq)) for j in range(len(tq[0]) - 1)]
    if not all(lo <= v <= hi for v in vals + steps):
        raise ValueError("2-D inherited fraction type: a table value or a step between neighbouring table values lies "
                         "outside the intermediate type %s; this case is not determined by the recorded data"
                         % _tstr_short(it))
    ult = p.get("UseLastTableValue") == "on"
    if prio == "Speed":
        # ASIC/FPGA, explicit / 'Same as output' intermediate (Wi, Fi) as written: F = Wi fraction bits in both
        # dimensions (29 of 29 such modules; 5 of them contradict the 1-D form Wt + Fi - Ft,
        # which is smaller there). Where Wt + Fi - Ft > Wi the two forms differ and no recorded module decides it.
        F, F1d = it[1], tt[1] + it[2] - tt[2]
        if F1d > F:
            raise ValueError("2-D inherited fraction type with InternalRulePriority Speed: table word + intermediate "
                             "fraction - table fraction (%d) exceeds the intermediate word (%d); not determined by "
                             "the recorded data" % (F1d, F))
        return "fixdt(0,%d,%d)" % ((F, F) if ult else (F + 1, F))
    return "fixdt(0,32,32)" if ult else "fixdt(0,32,31)"


def _blk_lut2d(case):
    """2-D Lookup Table: table rows follow dimension 1, columns dimension 2.
    Flat / Nearest pick an index per dimension like the 1-D table; Linear point-slope interpolates along
    dimension 1 at the two neighbouring columns, then along dimension 2, every step in the intermediate type."""
    p = case["params"]
    ins = _ins(case)
    bqs, tq, out_t = _lut2d_setup(p, [x[:3] for x in ins])
    us = [Fraction(n) / (Fraction(2) ** f) for s, w, f, n in ins[:2]]
    fm = _blk_fm(p)
    meth = p["InterpMethod"]
    idx, clipped = [], []
    for bq, u in zip(bqs, us):
        last = len(bq) - 1
        uu = min(max(u, bq[0]), bq[last])
        k = 0
        while k < last - 1 and uu >= bq[k + 1]:
            k += 1
        idx.append(k)
        clipped.append(uu)

    def pick(dim):
        k, uu, bq = idx[dim], clipped[dim], bqs[dim]
        if meth == "Flat":
            return k + 1 if uu >= bq[k + 1] else k
        return k + 1 if (uu - bq[k]) >= (bq[k + 1] - uu) else k
    if meth in ("Flat", "Nearest"):
        y = tq[pick(0)][pick(1)]
    else:
        lut_linear_types_recorded(p)
        it = _lut2d_itype(p, out_t, _asic(case.get("hw")))

        def qi(x):
            return Fraction(_quant_fraction(x, it[0], it[1], it[2], fm)) / (Fraction(2) ** it[2])
        ft = p.get("FractionDataTypeStr", _INHERIT_RULE)
        if str(ft).startswith("Inherit"):
            ft = lut2d_inherited_fraction(p, tq, it, _asic(case.get("hw")), out_t)   # ValueError outside the envelope
        lut2d_fraction_gap_ok(ft, lut2d_gaps(p, [tuple(x[:3]) for x in _ins(case)[:2]]))   

        def fr(dim):
            k, uu, bq = idx[dim], clipped[dim], bqs[dim]
            f_ = (uu - bq[k]) / (bq[k + 1] - bq[k])
            if not str(ft).startswith("Inherit"):
                fs, fw, ff = _named_type(ft)
                f_ = Fraction(_quant_fraction(f_, fs, fw, ff, {"rm": fm["rm"], "om": "Saturate"})) / (Fraction(2) ** ff)
            return f_

        def lin(y0, y1, f_):
            if LUT2D_DIFF_EXACT:   # the slope from the values as they are (table type / intermediate), product rounded
                return qi(qi(y0) + qi(f_ * (y1 - y0)))
            y0, y1 = qi(y0), qi(y1)
            return qi(y0 + qi(f_ * (y1 - y0)))
        k1, k2 = idx
        f1, f2 = fr(0), fr(1)
        # UseLastTableValue: at / beyond the last breakpoint of a dimension that dimension uses the last index
        # with a zero fraction (the fraction type cannot hold 1)
        if p.get("UseLastTableValue") == "on":
            if clipped[0] >= bqs[0][-1]:
                k1, f1 = len(bqs[0]) - 1, Fraction(0)
            if clipped[1] >= bqs[1][-1]:
                k2, f2 = len(bqs[1]) - 1, Fraction(0)
        n1, n2 = min(k1 + 1, len(bqs[0]) - 1), min(k2 + 1, len(bqs[1]) - 1)
        if ORDER_2D == "dim1_first":
            a = lin(tq[k1][k2], tq[n1][k2], f1)
            b = lin(tq[k1][n2], tq[n1][n2], f1)
            y = lin(a, b, f2)
        else:
            a = lin(tq[k1][k2], tq[k1][n2], f2)
            b = lin(tq[n1][k2], tq[n1][n2], f2)
            y = lin(a, b, f1)
    if out_t == "double":
        return {"cls": "double", "num": _fmt_double(float(y))}
    os_, ow, of = out_t
    return _out(os_, ow, of, _quant_fraction(y, os_, ow, of, lut_flat_fm(p, _lut_type(p["TableDataTypeStr"], out_t), fm)))


def _blk_sat_x(case):
    """Saturation with an explicit output type: the input is cast to the output type
    (block rounding, saturating), then clamped to the limits quantised to the output type (nearest, saturate)."""
    p = case["params"]
    s, w, f, n = _ins(case)[0]
    os_, ow, of = _named_type(p["OutDataTypeStr"])
    fm = _blk_fm(p)
    v = _cast(n, f, os_, ow, of, {"rm": fm["rm"], "om": "Saturate"})
    hi = _quant_param(float(p["UpperLimit"]), os_, ow, of)
    lo = _quant_param(float(p["LowerLimit"]), os_, ow, of)
    return _out(os_, ow, of, min(max(v, lo), hi))


def _blk_abs_x(case):
    """Abs with an explicit output type: |u| exactly, then cast to the output type
    with the block's rounding and overflow mode."""
    p = case["params"]
    s, w, f, n = _ins(case)[0]
    os_, ow, of = _named_type(p["OutDataTypeStr"])
    return _out(os_, ow, of, _cast(abs(n), f, os_, ow, of, _blk_fm(p)))


def lutnd_table(text):
    """n-D Lookup Table values written as reshape([v1 v2 ...], [n1 n2 ...]) (column-major, as MATLAB): {index tuple:
    value} and the shape. Other spellings raise ValueError."""
    import re
    m = re.match(r"^\s*reshape\(\s*\[([^\]]*)\]\s*,\s*\[([^\]]*)\]\s*\)\s*$", str(text))
    if not m:
        raise ValueError("an n-D table is written as reshape([values], [n1 n2 ...])")
    vals = [Fraction(str(float(v))) if not re.match(r"^-?\d+(\.\d+)?$", v) else Fraction(v)
            for v in m.group(1).replace(",", " ").split()]
    shape = [int(float(v)) for v in m.group(2).replace(",", " ").split()]
    size = 1
    for d in shape:
        size *= d
    if size != len(vals):
        raise ValueError("reshape: %d values do not fill the shape %s" % (len(vals), shape))
    if any(d < 2 for d in shape):
        # reshape([1 2 3], [1 3]) is a valid 1-D table in Simulink -- a spelling not taken here
        raise ValueError("an n-D table is written as reshape([values], [n1 n2 ...]) with every dimension at least 2, "
                         "got shape %s" % shape)
    table = {}
    for lin, v in enumerate(vals):
        idx, r = [], lin
        for d in shape:
            idx.append(r % d)
            r //= d
        table[tuple(idx)] = v
    return table, shape


def lutnd_magnitude(tt, it):
    """The falling term from the magnitude (as the 1-D rule lut1d_magnitude_slope): an unsigned 8 / 16-bit table type
    that is the intermediate type (0 of 1240 steps wrong with it, 175 without)."""
    return isinstance(tt, tuple) and tt[0] == 0 and tt[1] in (8, 16) and tuple(it) == tuple(tt)


def _blk_lutnd_v1(case):
    """n-D Lookup Table, 3 or 4 dimensions (host, InternalRulePriority
    Speed, inherited intermediate / fraction types, Clip; 74,936 steps equal). Breakpoints and table quantised to their
    types (nearest); Flat / Nearest take a table value per dimension as the 1-D table and cast it to the output (floor);
    Linear point-slope interpolates dimension 1 first, then 2, ...: every step y0 + floor(f * (y1 - y0)) in the
    intermediate type (= the output type), floor and wrap, f floored to F fraction bits -- F = the intermediate word,
    16 where the falling term comes from the magnitude (lutnd_magnitude: y0 - floor(f * (y0 - y1)))."""
    p = case["params"]
    ins = _ins(case)
    dims = int(str(p["NumberOfTableDimensions"]).strip())
    its_ = [x[:3] for x in ins[:dims]]
    ot_txt = str(p.get("OutDataTypeStr", _SAME_FIRST))
    out_t = tuple(its_[0]) if ot_txt == _SAME_FIRST else _named_type(ot_txt)
    tt = _lut_type(str(p.get("TableDataTypeStr", "Inherit: Same as output")), out_t)
    table, shape = lutnd_table(p["Table"])
    if len(shape) != dims:
        raise ValueError("the table has %d dimensions, NumberOfTableDimensions is %d" % (len(shape), dims))

    def qv(v, t):
        return Fraction(_quant_param(v, *t)) / (Fraction(2) ** t[2])
    bqs = []
    for d in range(1, dims + 1):
        bt = _lut_type(str(p.get("BreakpointsForDimension%dDataTypeStr" % d, "Inherit: Same as corresponding input")),
                       tuple(its_[d - 1]))
        bq = [qv(b, bt) for b in _vec(p["BreakpointsForDimension%d" % d])]
        if len(bq) != shape[d - 1] or any(b1 <= b0 for b0, b1 in zip(bq, bq[1:])):
            raise ValueError("n-D Lookup Table: breakpoints of dimension %d must be strictly increasing and match the "
                             "table size %d" % (d, shape[d - 1]))
        bqs.append(bq)
    tq = {k: qv(v, tt) for k, v in table.items()}
    s_, w_, f_ = out_t

    def wrap_floor(v, t):
        s, w, f = t
        n = (v * (Fraction(2) ** f)).__floor__()
        n &= (1 << w) - 1
        if s and n >= 1 << (w - 1):
            n -= 1 << w
        return Fraction(n) / (Fraction(2) ** f)
    us = [Fraction(n) / (Fraction(2) ** f) for s, w, f, n in ins[:dims]]
    loc = []
    for u, bq in zip(us, bqs):
        last = len(bq) - 1
        uu = min(max(u, bq[0]), bq[last])
        k = 0
        while k < last - 1 and uu >= bq[k + 1]:
            k += 1
        loc.append((k, uu))
    meth = p["InterpMethod"]
    if meth in ("Flat", "Nearest"):
        idx = []
        for (k, uu), bq in zip(loc, bqs):
            if meth == "Flat":
                idx.append(k + 1 if uu >= bq[k + 1] else k)
            else:
                idx.append(k + 1 if (uu - bq[k]) >= (bq[k + 1] - uu) else k)
        v = tq[tuple(idx)] * (Fraction(2) ** f_)
        return _out(s_, w_, f_, _overflow(v.__floor__(), s_, w_, "Wrap"))
    it = out_t
    mag = lutnd_magnitude(tt, it)
    F = 16 if mag else it[1]
    fs = [Fraction(((uu - bq[k]) / (bq[k + 1] - bq[k]) * (Fraction(2) ** F)).__floor__()) / (Fraction(2) ** F)
          for (k, uu), bq in zip(loc, bqs)]
    ks = [k for k, _ in loc]

    def rec(d, base):   # dimension dims-1 at the top: dimension 1 is interpolated first
        if d < 0:
            return tq[tuple(base)]
        b0, b1 = list(base), list(base)
        b0[d], b1[d] = ks[d], min(ks[d] + 1, len(bqs[d]) - 1)
        y0, y1 = wrap_floor(rec(d - 1, b0), it), wrap_floor(rec(d - 1, b1), it)
        if mag and y1 < y0:
            return wrap_floor(y0 - wrap_floor(fs[d] * (y0 - y1), it), it)
        return wrap_floor(y0 + wrap_floor(fs[d] * (y1 - y0), it), it)
    y = wrap_floor(rec(dims - 1, [0] * dims), out_t)
    return _out(s_, w_, f_, int(y * (Fraction(2) ** f_)))




def lutnd_setup(p, its):
    """n-D Lookup Table: (breakpoints per dimension quantised to their types, the table {index: value} quantised to
    the table type, its shape, the output type, the table type)."""
    dims = int(str(p["NumberOfTableDimensions"]).strip())
    ot_txt = str(p.get("OutDataTypeStr", _SAME_FIRST))
    if ot_txt == _SAME_FIRST:
        out_t = tuple(its[0])
    elif ot_txt.startswith("Inherit"):
        out_t = "double"
    else:
        out_t = _named_type(ot_txt)
    tt = _lut_type(str(p.get("TableDataTypeStr", "Inherit: Same as output")), out_t)
    table, shape = lutnd_table(p["Table"])
    if len(shape) != dims:
        raise ValueError("the table has %d dimensions, NumberOfTableDimensions is %d" % (len(shape), dims))
    bqs = []
    for d in range(1, dims + 1):
        bt = _lut_type(str(p.get("BreakpointsForDimension%dDataTypeStr" % d, "Inherit: Same as corresponding input")),
                       tuple(its[d - 1]))
        bq = [Fraction(_quant_param(b, *bt)) / (Fraction(2) ** bt[2]) for b in _vec(p["BreakpointsForDimension%d" % d])]
        if len(bq) != shape[d - 1] or any(b1 <= b0 for b0, b1 in zip(bq, bq[1:])):
            raise ValueError("n-D Lookup Table: breakpoints of dimension %d must be strictly increasing and match the "
                             "table size %d" % (d, shape[d - 1]))
        bqs.append(bq)
    tq = {k: (Fraction(v) if tt == "double" else Fraction(_quant_param(v, *tt)) / (Fraction(2) ** tt[2]))
          for k, v in table.items()}
    return bqs, tq, shape, out_t, tt


def _blk_lutnd(case):
    """n-D Lookup Table: the dimension-by-dimension rule (_blk_lutnd_dims) where the engine's gate chose it
    (_NDRule 'dims', recordings), else the earlier rule of recordings"""
    if case["params"].get("_NDRule") == "dims":
        return _blk_lutnd_dims(case)
    return _blk_lutnd_v1(case)


def _blk_lutnd_dims(case):
    """n-D Lookup Table, dimension by dimension (
    every step of the 100 recorded designs Simulink runs). Breakpoints quantised to their (inherited) types and the
    table to the table type, nearest with ties toward +inf. Flat / Nearest pick an index per dimension and cast the
    table value into the output (block rounding and overflow, lut_flat_fm). Linear point-slope (internal-rule
    intermediate, fixed-point table): intermediate type (1, 32, Ft + min(0, 31 - Wt)) from the table type (Wt, Ft);
    the fraction of each dimension rounded by the block's RndMeth (Simplest: Floor) into Wt fraction bits; dimension
    1 interpolated first, each step qi(qi(y0) + qi(f * (y1 - y0))) in the intermediate type with the block's rounding
    and overflow; UseLastTableValue on: at or above the last breakpoint of a dimension that dimension takes the last
    index with fraction 0. Other linear forms are refused by the gate (not in this kernel's recorded span)."""
    p = case["params"]
    ins = _ins(case)
    asic = _asic(case.get("hw"))
    dims = int(str(p["NumberOfTableDimensions"]).strip())
    its_ = [tuple(x[:3]) for x in ins[:dims]]
    bqs, tq, shape, out_t, tt = lutnd_setup(p, its_)
    fm = _blk_fm(p)
    us = [Fraction(n) / (Fraction(2) ** f) for s, w, f, n in ins[:dims]]
    meth = p["InterpMethod"]
    ks, cl = [], []
    for bq, u in zip(bqs, us):
        last = len(bq) - 1
        uu = min(max(u, bq[0]), bq[last])
        k = 0
        while k < last - 1 and uu >= bq[k + 1]:
            k += 1
        ks.append(k)
        cl.append(uu)
    if meth in ("Flat", "Nearest"):
        idx = []
        for k, uu, bq in zip(ks, cl, bqs):
            if meth == "Flat":
                idx.append(k + 1 if uu >= bq[k + 1] else k)
            else:
                idx.append(k + 1 if (uu - bq[k]) >= (bq[k + 1] - uu) else k)
        y = tq[tuple(idx)]
    else:
        if not isinstance(tt, tuple) or str(p.get("IntermediateResultsDataTypeStr")) != _INHERIT_RULE or \
                not str(p.get("FractionDataTypeStr", _INHERIT_RULE)).startswith("Inherit"):
            raise ValueError("n-D linear interpolation is answered with the internal-rule intermediate type, an "
                             "inherited fraction type and a fixed-point table only")
        it = (1, 32, tt[2] + min(0, 31 - tt[1]))
        F = tt[1]

        def qi(x):
            return Fraction(_quant_fraction(x, it[0], it[1], it[2], fm)) / (Fraction(2) ** it[2])
        fs = []
        for d in range(dims):
            k, uu, bq = ks[d], cl[d], bqs[d]
            f_ = (uu - bq[k]) / (bq[k + 1] - bq[k])
            fs.append(Fraction(_quant_fraction(f_, 0, F + 1, F, {"rm": fm["rm"], "om": "Saturate"})) / (Fraction(2) ** F))
        kk = list(ks)
        if p.get("UseLastTableValue") == "on":
            for d in range(dims):
                if cl[d] >= bqs[d][-1]:
                    kk[d], fs[d] = len(bqs[d]) - 1, Fraction(0)

        def rec(d, base):   # dimension 1 interpolated first
            if d < 0:
                return tq[tuple(base)]
            b0, b1 = list(base), list(base)
            b0[d], b1[d] = kk[d], min(kk[d] + 1, len(bqs[d]) - 1)
            y0, y1 = rec(d - 1, b0), rec(d - 1, b1)
            return qi(qi(y0) + qi(fs[d] * (y1 - y0)))
        y = rec(dims - 1, [0] * dims)
    if out_t == "double":
        return {"cls": "double", "num": _fmt_double(float(y))}
    os_, ow, of = out_t
    return _out(os_, ow, of, _quant_fraction(y, os_, ow, of, lut_flat_fm(p, tt, fm)))


def prelookup_fraction_type(p, xt, asic):
    """The fraction output type: an explicit fixdt, or the internal rule (the library default) -- 32 bits on the host,
    the input's word on ASIC / FPGA, unsigned and all fraction (ufix32_En31 on the host;
    ufix8_En7 / ufix16_En15 / ufix32_En31 for 8 / 16 / 32-bit inputs on ASIC)."""
    txt = str(p.get("FractionDataTypeStr", "fixdt(0,32,31)")).strip()
    if txt == _INHERIT_RULE:
        w = xt[1] if asic else 32
        if w not in (8, 16, 32):
            raise ValueError("Prelookup: the inherited fraction type for a %d-bit input is not recorded" % w)
        return (0, w, w - 1)
    return _named_type(txt)


def _prelookup_setup(p, xt, asic=False):
    bt = _lut_type(str(p.get("BreakpointDataTypeStr", "Inherit: Same as input")), tuple(xt))
    bq = [Fraction(_quant_param(b, *bt)) / (Fraction(2) ** bt[2]) for b in _vec(p["BreakpointsData"])]
    if len(bq) < 2 or any(b1 <= b0 for b0, b1 in zip(bq, bq[1:])):
        raise ValueError("Prelookup: breakpoints must be at least two and strictly increasing in their type %s" % (bt,))
    ft = prelookup_fraction_type(p, xt, asic)
    return bq, ft


def _prelookup_kf(case):
    """Prelookup, Clip (every k / f equal):
    breakpoints rounded to nearest into their type; k = the interval of the clipped input (0 .. n-2), f = (u - b_k) /
    (b_k+1 - b_k) floored into the fraction type (1 at the top end)."""
    p = case["params"]
    s, w, f, n = _ins(case)[0]
    bq, ft = _prelookup_setup(p, (s, w, f), _asic(case.get("hw")))
    u = Fraction(n) / (Fraction(2) ** f)
    last = len(bq) - 1
    uu = min(max(u, bq[0]), bq[last])
    k = 0
    while k < last - 1 and uu >= bq[k + 1]:
        k += 1
    fr = (uu - bq[k]) / (bq[k + 1] - bq[k])
    return k, (fr * (Fraction(2) ** ft[2])).__floor__(), ft


def _blk_prelookup_k(case):
    k, _fn, _ft = _prelookup_kf(case)
    return _out(0, 32, 0, k)


def _blk_prelookup_f(case):
    _k, fn, ft = _prelookup_kf(case)
    return _out(ft[0], ft[1], ft[2], fn)


def interp_pl_itype(tt, ft):
    """Intermediate type of Interpolation Using Prelookup, linear, 'Inherit via internal rule', Speed
    (56 of 60 models, every step): the table type itself when the fraction type is a
    32-bit word; otherwise 16 bits with one integer bit more than the table, 32 bits when the table word does not fit
    that (table word + 1 > 16). A 16-bit table with a 16 / 8-bit fraction is not settled (recordings s16En10 takes
    the table type, recordings tu16En8 fits neither): the engine refuses it."""
    if ft[1] == 32 or (tt[1] == 16 and ft[1] in (16, 8)):
        # (a 16-bit table with a 16 / 8-bit fraction: the table type -- recordings on ASIC, every
        # recorded design of both table / output pairs, and recordings s16En10. recordings
        # tu16En8 on the host fits neither rule, so the accept predicate opens this only for the recorded ASIC pairs)
        return tuple(tt)
    if tt[0] and tt[1] == 16:
        # recordings (ASIC / FPGA: fractions ufix16_En15 / ufix8_En7, tables sfix16_En8 / int16 -- 48 models,
        # every step on the table's grid) and recordings s16En10: a signed 16-bit table interpolates in the table
        # type (recordings ts16En8 fits it too); an unsigned 16-bit table fits neither (refused)
        return tuple(tt)
    ti = tt[1] - tt[2]
    return (1, 16, 16 - ti - 1) if tt[1] + 1 <= 16 else (1, 32, 32 - ti - 1)


def _blk_interp_pl(case):
    """Interpolation Using Prelookup, 1 / 2 dimensions, Clip: inputs k1, f1, k2, f2 ...; Flat / Nearest take the table
    value at k (+1 where f = 1, or f >= 1/2 for Nearest) and cast it to the output (floor); Linear point-slope
    interpolates dimension 1 first, every step y0 + floor(f (y1 - y0)) in interp_pl_itype with wrap, then floor into
    the output type."""
    p = case["params"]
    ins = _ins(case)
    dims = int(str(p["NumberOfTableDimensions"]).strip())
    out_t = _named_type(str(p["OutDataTypeStr"]))
    tt = _lut_type(str(p.get("TableDataTypeStr", "Inherit: Same as output")), out_t)
    if dims == 1:
        vals = _vec(p["Table"])
        table, shape = {(i,): Fraction(str(v)) for i, v in enumerate(vals)}, [len(vals)]
    else:
        table, shape = lutnd_table(p["Table"])
    tq = {k: Fraction(_quant_param(v, *tt)) / (Fraction(2) ** tt[2]) for k, v in table.items()}
    ks, fs, ft = [], [], None
    for d in range(dims):
        ks.append(min(max(int(ins[2 * d][3]), 0), shape[d] - 2))
        fs_, fw_, ff_, fn_ = ins[2 * d + 1]
        ft = (fs_, fw_, ff_)
        fs.append(Fraction(fn_) / (Fraction(2) ** ff_))

    def wrap_floor(v, t):
        s, w, f = t
        n = (v * (Fraction(2) ** f)).__floor__()
        n &= (1 << w) - 1
        if s and n >= 1 << (w - 1):
            n -= 1 << w
        return Fraction(n) / (Fraction(2) ** f)
    s_, w_, f_ = out_t
    meth = p.get("InterpMethod", "Linear point-slope")
    if meth in ("Flat", "Nearest"):
        idx = [k + (1 if (f >= 1 or (meth == "Nearest" and f >= Fraction(1, 2))) else 0) for k, f in zip(ks, fs)]
        v = tq[tuple(min(i, shape[d] - 1) for d, i in enumerate(idx))]
        return _out(s_, w_, f_, int(wrap_floor(v, out_t) * (Fraction(2) ** f_)))
    it = interp_pl_itype(tt, ft)
    host_asic = _asic(case.get("hw"))

    def rec(d, base):
        if d < 0:
            return tq[tuple(base)]
        b0, b1 = list(base), list(base)
        b0[d], b1[d] = ks[d], min(ks[d] + 1, shape[d] - 1)
        y0, y1 = wrap_floor(rec(d - 1, b0), it), wrap_floor(rec(d - 1, b1), it)
        if not it[0] and y1 < y0 and ft[1] < 32 and not host_asic:
            # recordings (host, an unsigned 16-bit table, a 16 / 8-bit fraction): on a falling
            # segment Simulink takes y0 - floor(f (y0 - y1)), 1 LSB above y0 + floor(f (y1 - y0)). With a 32-bit
            # fraction and on ASIC the recordings take the latter
            return wrap_floor(y0 - wrap_floor(fs[d] * (y0 - y1), it), it)
        return wrap_floor(y0 + wrap_floor(fs[d] * (y1 - y0), it), it)
    y = wrap_floor(rec(dims - 1, [0] * dims), out_t)
    return _out(s_, w_, f_, int(y * (Fraction(2) ** f_)))


_BLOCKS = {
    "blk_prelookup_k": _blk_prelookup_k,
    "blk_prelookup_f": _blk_prelookup_f,
    "blk_interp_pl": _blk_interp_pl,
    "blk_lutnd": _blk_lutnd,
    "blk_minmax": _blk_minmax,
    "blk_product2": _blk_product2,
    "blk_saturation": _blk_saturation,
    "blk_relop": _blk_relop,
    "blk_abs": _blk_abs,
    "blk_uminus": _blk_uminus,
    "blk_gain": _blk_gain,
    "blk_dtc": _blk_dtc,
    "blk_sum2": lambda c: _blk_sum(c, 2),
    "blk_sum3": lambda c: _blk_sum(c, 3),
    "blk_sum4": lambda c: _blk_sum(c, 4),
    "blk_constant": _blk_constant,
    "blk_minmaxw": _blk_minmax,                                  # targeted wide-type recordings
    "blk_sumacc": lambda c: _blk_sum(c, len(c["ins"])),
    "blk_sumacc_t6": lambda c: _blk_sum(c, len(c["ins"])),
    "blk_minmaxw_t6": lambda c: _blk_minmax(c),
    "blk_delay": _blk_delay,
    "blk_shift": _blk_shift,
    "blk_logic": _blk_logic,
    "blk_cmpconst": _blk_cmpconst,
    "blk_switch": _blk_switch,
    "blk_sqrt": _blk_sqrt,
    "blk_cmpzero": _blk_cmpzero,
    "blk_deadzone": _blk_deadzone,
    "blk_wraptozero": _blk_wraptozero,
    "blk_ratelimit": _blk_ratelimit,
    "blk_trig": _blk_trig,
    "blk_delayport": _blk_delayport,
    "blk_dtistate": _blk_dtistate,
    "blk_dtistategain": _blk_dtistategain,
    "blk_mpswitch": _blk_mpswitch,
    "blk_productn": _blk_productn,
    "blk_bool2arith": _blk_bool2arith,
    "blk_booldiv": _blk_booldiv,
    "blk_lut1d": _blk_lut1d,
    "blk_lut_lps": _blk_lut1d,
    "blk_lut_inh": _blk_lut1d,
    "blk_lut2d": _blk_lut2d,
    "blk_lut2d_inh": _blk_lut2d,
    "blk_sat_x": _blk_sat_x,
    "blk_abs_x": _blk_abs_x,
    "blk_dfir": _blk_dfir,
    "blk_dfir2": _blk_dfir2,
    "blk_counter": _blk_counter,
    "blk_detect": _blk_detect,
    "blk_signround": _blk_signround,
    "blk_bitwise": _blk_bitwise,
    "blk_extract": _blk_extract,
    "blk_dtprop": _blk_dtprop,
    "blk_dti": _blk_dti,
}


def eval_case(case):
    op = case["op"]
    if op in _BLOCKS:
        return _BLOCKS[op](case)
    fm = case["fm"]
    if op == "sum":
        return _vsum(case)
    if op == "dot":
        return _dot(case)
    if op == "divide":
        return _divide(case)
    if op == "cmp":
        return _cmp(case)
    if op == "bitshift":
        return _bitshift(case)
    if op == "abs":
        return _abs(case)
    if op == "quantize":
        t = case["t"]
        if t["s"] and t["wl"] == 1:
            return {"err": ERR_MINWL}
        x = _x_to_fraction(case["x"])
        scaled = x * (Fraction(2) ** t["fl"])
        # exact rounding on a rational: scale by 2**k to get an integer, then round
        num, den = scaled.numerator, scaled.denominator
        sh = den.bit_length() - 1  # den is a power of two
        n = _round_div(num, sh, fm["rm"]) if sh > 0 else num
        n = _overflow(n, t["s"], t["wl"], fm["om"])
        return _out(t["s"], t["wl"], t["fl"], n)
    if op == "cast":
        t = case["t"]
        if t["s"] and t["wl"] == 1:
            return {"err": ERR_MINWL}
        sa, wa, fa, na = _decode(case["a"])
        return _out(t["s"], t["wl"], t["fl"], _cast(na, fa, t["s"], t["wl"], t["fl"], fm))
    if op == "add":
        return _add_sub(case, False)
    if op == "sub":
        return _add_sub(case, True)
    if op == "mul":
        return _mul(case)
    if op == "neg":
        sa, wa, fa, na = _decode(case["a"])
        return _out(sa, wa, fa, _cast(-na, fa, sa, wa, fa, fm))
    raise ValueError(op)


def _tname(t):
    return "%sfix%d_En%d" % ("s" if t[0] else "u", t[1], t[2])

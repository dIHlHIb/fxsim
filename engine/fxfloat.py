"""IEEE binary32, binary64 and exact-rational helpers for supported floating-point signal paths."""
import math
import struct
from fractions import Fraction

DOUBLE = "double"
SINGLE = "single"
FLOAT_TYPES = (DOUBLE, SINGLE)


def to_single(x):
    """x rounded to IEEE binary32 (round to nearest, ties to even), returned as a Python float."""
    if math.isnan(x) or math.isinf(x):
        return x
    try:
        return struct.unpack("<f", struct.pack("<f", x))[0]
    except OverflowError:   # beyond the largest single: +-Inf, as IEEE round-to-nearest gives
        return math.copysign(math.inf, x)


def bits(x, t):
    """IEEE bit pattern of x in type t as hex ('double': 16 digits, 'single': 8) -- results are compared on these. A NaN
    keeps its sign and payload (x86-64 semantics: an invalid operation gives the negative default NaN, a NaN operand
    propagates)."""
    if t == DOUBLE:
        return "%016x" % struct.unpack("<Q", struct.pack("<d", x))[0]
    return "%08x" % struct.unpack("<I", struct.pack("<f", to_single(x)))[0]


def fixed_to_float(n, f, t):
    """The stored integer n with fraction length f as a float of type t: n * 2^-f rounded once (to nearest even) into
    the float format -- exact while the value has at most 53 (double) / 24 (single) significant bits."""
    q = Fraction(n) / (Fraction(2) ** f) if f >= 0 else Fraction(n) * (Fraction(2) ** -f)
    if t == SINGLE:
        return _fraction_to_single(q)
    return float(q)   # Python's float(Fraction) is correctly rounded (nearest, ties to even)


def _fraction_to_single(q):
    """q rounded once (nearest, ties to even) to binary32 -- not via double, which would round twice."""
    if q == 0:
        return 0.0
    sign = -1 if q < 0 else 1
    a = abs(q)
    e = a.numerator.bit_length() - a.denominator.bit_length()
    if Fraction(2) ** e > a:
        e -= 1
    e = max(e, -126)                                  # subnormals keep the minimum exponent
    scale = Fraction(2) ** (23 - e)
    m = a * scale
    mi = _round(m, "Convergent")
    if mi >= (1 << 24):                               # rounding carried into the next binade
        mi //= 2
        e += 1
    if e > 127:
        return sign * math.inf
    return sign * math.ldexp(float(mi), e - 23)


ROUNDINGS = ("Floor", "Ceiling", "Zero", "Nearest", "Convergent", "Round", "Simplest")


def _round(q, rnd):
    """q (a Fraction) rounded to an integer by the Simulink rounding mode rnd (Simplest: not defined here -- its
    meaning depends on the block, so it is refused by the caller until recorded)."""
    fl = q.numerator // q.denominator
    if q == fl:
        return fl
    if rnd == "Floor":
        return fl
    if rnd == "Ceiling":
        return fl + 1
    if rnd == "Zero":
        return fl if q > 0 else fl + 1
    half = Fraction(1, 2)
    d = q - fl
    if rnd == "Nearest":            # ties toward +inf
        return fl + 1 if d >= half else fl
    if rnd == "Round":              # ties away from zero
        if d == half:
            return fl + 1 if q > 0 else fl
        return fl + 1 if d > half else fl
    if rnd == "Convergent":         # ties to even
        if d == half:
            return fl if fl % 2 == 0 else fl + 1
        return fl + 1 if d > half else fl
    raise ValueError("rounding %r is not defined for a float input" % rnd)


def float_to_fixed(x, s, w, f, rnd, saturate):
    """A float x into the fixed-point type (s, w, f): the exact value of x rounded to the 2^-f grid by rnd, then
    saturated (saturate) or wrapped into w bits. Returns the stored integer. NaN / Inf raise ValueError: what Simulink
    gives for them is decided by the recordings, not here."""
    if math.isnan(x) or math.isinf(x):
        raise ValueError("a non-finite value (%r) into a fixed-point type: not recorded" % x)
    q = Fraction(x) * (Fraction(2) ** f) if f >= 0 else Fraction(x) / (Fraction(2) ** -f)
    n = _round(q, rnd)
    lo, hi = (-(1 << (w - 1)), (1 << (w - 1)) - 1) if s else (0, (1 << w) - 1)
    if lo <= n <= hi:
        return n
    if saturate:
        return min(max(n, lo), hi)
    n &= (1 << w) - 1
    return n - (1 << w) if s and n >= 1 << (w - 1) else n


POS_NAN = struct.unpack("<d", struct.pack("<Q", 0x7FF8000000000000))[0]


def nan_in(*xs):
    """recordings: NaN follows x86-64 -- a NaN operand propagates with its own sign (the first NaN
    operand), an invalid operation on non-NaN operands (Inf - Inf, 0 * Inf, 0 / 0) gives the negative default NaN.
    Returns the first NaN operand, else None."""
    for x in xs:
        if math.isnan(x):
            return x
    return None


def neg(x):
    """Negation (Unary Minus, a leading '-' in a Sum): the sign flips, -0 included, but a NaN is left as it is
    (-(+NaN) is +NaN, -(+0) is -0)."""
    return x if math.isnan(x) else -x


def sub(a, b, t):
    """a - b in type t (a subtraction, not a + (-b): a NaN operand keeps its sign)."""
    n = nan_in(a, b)
    r = n if n is not None else a - b
    return to_single(r) if t == SINGLE else r


def add(a, b, t):
    """a + b in type t (one IEEE rounding; single: the double sum rounded to single -- exact for single operands since
    the double sum of two singles rounds the same as a direct single addition)."""
    n = nan_in(a, b)
    r = n if n is not None else a + b
    return to_single(r) if t == SINGLE else r


def mul(a, b, t):
    """a * b in type t (as add: the double product of two singles is exact, rounding it to single is the single
    product)."""
    n = nan_in(a, b)
    r = n if n is not None else a * b
    return to_single(r) if t == SINGLE else r


def div(a, b, t):
    """a / b in type t, IEEE (x / 0 = +-Inf, 0 / 0 = NaN); single: the double quotient rounded to single (correct
    rounding: double has more than 2 * 24 + 2 bits)."""
    n = nan_in(a, b)
    if n is not None:
        r = n
    elif b == 0:
        r = -POS_NAN if a == 0 else math.copysign(math.inf, a) * math.copysign(1.0, b)
    else:
        r = a / b
    return to_single(r) if t == SINGLE else r

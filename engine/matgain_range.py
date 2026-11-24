"""The range predicate D for Gain Matrix(K*u): inside D the block is
answered by the one rule every recording fits. That rule:
- K is quantised with the whole matrix's best precision (or the explicit parameter type);
- each product is rounded by RndMeth into the output type, unless the output is the internal rule;
- the row is accumulated left to right, saturating or wrapping at every addition.
Seven alternative rules are each refuted by recorded designs (section 2). The bounds are what the recordings span:
recordings (612 designs
run, 4- to 8-bit inputs, host and ASIC) and the matrix maps. Outside D the exact tuples and families keep their
answers; everything else is refused with the field that is outside."""
import re

INH = "Inherit: Inherit via internal rule"
SAME = "Inherit: Same as input"
# explicit parameter types recorded with Gain Matrix: the dense, branched and lossy matrix configurations, and
# recordings (fixdt(1,8,0) / fixdt(1,16,8) 45 designs each, fixdt(1,16,15) 18, all equal;
# fixdt(1,8,8) / fixdt(1,16,16) only overflowed there -- not opened)
PARAM_TYPES = {(1, 8, 5), (1, 8, 6), (1, 8, 7), (1, 16, 12), (1, 16, 14), (1, 8, 0), (1, 16, 8), (1, 16, 15)}
# shapes recorded with a vector input: every m x n with m 1..4, n 2..4 (recordings added
# 1x3 1x4 2x4 4x2 3x4 4x3, 18 to 36 designs each, all equal); 2x1 on a scalar input is the vector-parameter path
SHAPES = {"%dx%d" % (m, n) for m in range(1, 5) for n in range(2, 5)}
RNDS = {"Floor", "Ceiling", "Convergent", "Nearest", "Round", "Simplest", "Zero"}
IN_WL = (4, 32)    # recorded: 4 (witness s4En2) .. 32 (G, A)
OUT_WL = (6, 32)   # recorded: 6 (witness fixdt(1,6,2)) .. 32 (G, A)


def _fixdt(text):
    m = re.match(r"^fixdt\(\s*([01])\s*,\s*(\d+)\s*,\s*(-?\d+)\s*\)$", str(text).strip())
    return tuple(int(g) for g in m.groups()) if m else None


# recordings (every
# design with integer products into fixdt(0,32,0), 136 designs, all steps): the parameter and input types recorded
U32_PARAMS = {(0, 8, 0), (1, 4, 0), (1, 6, 0), (1, 8, 0), (1, 12, 0), (1, 16, 0)}
U32_IN_SIGNED = (4, 32)     # recorded: 4, 8, 12, 16, 17, 20, 23, 24, 25, 31, 32
U32_IN_UNSIGNED = (4, 16)   # recorded: 4, 8, 12, 16
U32_PW_MAX = 40             # recorded products: 8 .. 40 bits


def u32_exact_sum(it, pt):
    """Integer products into fixdt(0,32,0): Simulink sums the row exactly and saturates / wraps once when the
    parameter type and the input type are both built-in integers (8, 16 or 32 bits, fraction 0) and the products
    have at most 24 bits; otherwise each product is cast into the output on its own (the per-product rule). Every
    recorded design fits (section 11); the per-product rule fails on 24 of the exact-sum ones and the exact sum on
    104 of the others."""
    return it[1] in (8, 16, 32) and pt[1] in (8, 16, 32) and it[2] == 0 and pt[2] == 0 and it[1] + pt[1] <= 24


def _u32_integer(it, pt, shape, rnd, sat, asic):
    """The recorded span of integer products into fixdt(0,32,0) (section 11): parameter one of U32_PARAMS, input
    signed 4..32 or unsigned 4..16 bits with fraction 0, products of at most 40 bits, any m x n of SHAPES, every
    RndMeth (an integer product into an integer output is never rounded), saturation on or off, host or ASIC."""
    s, wl, fl = it
    if pt not in U32_PARAMS:
        return False, "ParamDataTypeStr fixdt(%d,%d,%d) with an unsigned 32-bit output (recorded there: %s)" % (
            pt + (", ".join("fixdt(%d,%d,%d)" % t for t in sorted(U32_PARAMS)),))
    lo, hi = U32_IN_SIGNED if s else U32_IN_UNSIGNED
    if not lo <= wl <= hi:
        return False, "input fixdt(%d,%d,0) with an unsigned 32-bit output (recorded there: signed %d..%d and " \
                      "unsigned %d..%d bits)" % (s, wl, U32_IN_SIGNED[0], U32_IN_SIGNED[1], U32_IN_UNSIGNED[0],
                                                 U32_IN_UNSIGNED[1])
    if wl + pt[1] > U32_PW_MAX:
        return False, "products of %d bits into an unsigned 32-bit output (recorded: up to %d)" % (wl + pt[1], U32_PW_MAX)
    if str(rnd) not in RNDS or str(sat) not in ("on", "off"):
        return False, "RndMeth %s / saturation %s" % (rnd, sat)
    return True, None


_REGIONS = None


def _regions():
    """engine/matgain_regions.json (the regression check): per region A-E and hardware, the recorded value
    set of every axis."""
    global _REGIONS
    if _REGIONS is None:
        import json as _json
        import os as _os
        p_ = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "matgain_regions.json")
        _REGIONS = _json.load(open(p_)) if _os.path.exists(p_) else {}
    return _REGIONS


def region_of(it, param, out, shape):
    """The region a setting belongs to by its defining condition, or None:
    A  unsigned 32-bit output, explicit parameter, products to round into the output (product fraction > fo);
    B  unsigned 32-bit output, product fraction = fo, fo not 0 / 4 (or 4 with products under 24 bits);
    C  unsigned 32-bit output fixdt(0,32,0), internal-rule parameter;
    D  integer products into fixdt(0,32,0);
    E  5 or more rows or columns."""
    m_, n_ = (int(x) for x in str(shape).split("x"))
    if m_ >= 5 or n_ >= 5:
        return "E"
    o = _fixdt(out)
    p = str(param).strip()
    if o is None or o[:2] != (0, 32):
        return None
    if p == INH:
        return "C" if o[2] == 0 else None
    pt = _fixdt(p)
    if pt is None:
        return None
    fr = pt[2] + it[2]
    if o[2] == 0 and fr == 0:
        return "D"
    if fr > o[2]:
        return "A"
    if fr == o[2] and (o[2] not in (0, 4) or (o[2] == 4 and it[1] + pt[1] < 24)):
        return "B"
    return None


def in_regions(it, param, out, rnd, sat, shape, asic=False):
    """True when every axis value of the setting was recorded in its region on the same hardware (section 13)."""
    reg = region_of(it, param, out, shape)
    a = (_regions().get(reg) or {}).get("asic" if asic else "host") if reg else None
    if not a:
        return False
    import json as _json
    return (_json.dumps(list(it)) in a["in"] and str(param).strip() in a["param"] and str(out).strip() in a["out"] and
            str(rnd) in a["rnd"] and str(sat) in a["sat"] and str(shape) in a["shape"])


def in_range(it, param, out, rnd, sat, shape, asic=False):
    """(True, None) when the Gain Matrix setting is inside D; else (False, the first field outside, as text).
    it: the input type (s, wl, fl); param / out: ParamDataTypeStr / OutDataTypeStr as written; shape 'mxn'."""
    if not (isinstance(it, tuple) and len(it) == 3):
        return False, "input type %r" % (it,)
    if in_regions(it, param, out, rnd, sat, shape, asic):
        return True, None
    s, wl, fl = it
    if not (IN_WL[0] <= wl <= IN_WL[1] and 0 <= fl <= wl):
        return False, "input fixdt(%d,%d,%d) (recorded: %d..%d bits, fraction 0..word length)" % (s, wl, fl, *IN_WL)
    if shape not in SHAPES:
        return False, "a %s matrix (recorded: %s)" % (shape, ", ".join(sorted(SHAPES)))
    p = str(param).strip()
    o_ = str(out).strip()
    if _fixdt(o_) == (0, 32, 0) and _fixdt(p) is not None and _fixdt(p)[2] == 0 and fl == 0:
        # section 11: integer products into fixdt(0,32,0) -- exact sum or per-product rule by u32_exact_sum
        return _u32_integer(it, _fixdt(p), shape, rnd, sat, asic)
    if _fixdt(p) == (1, 16, 0) and _fixdt(o_) is not None and _fixdt(o_)[:2] == (0, 32):
        # fixdt(1,16,0) is recorded only with an unsigned 32-bit output, signed
        # 16..24-bit inputs with fraction 0 or 4 equal to the output's, 1x2 / 2x3 / 3x3, Floor, saturation on, host --
        # 11 designs, all the per-product rule; answered there only
        m_, n_ = (int(x) for x in shape.split("x"))
        if not (s == 1 and 16 <= wl <= 24 and fl in (0, 4) and _fixdt(o_)[2] == fl and 1 <= m_ <= 3 and
                2 <= n_ <= 3 and str(rnd) == "Floor" and str(sat) == "on" and not asic):
            return False, "ParamDataTypeStr fixdt(1,16,0) with input fixdt(%d,%d,%d), output %s, a %s matrix, " \
                          "RndMeth %s, saturation %s%s (recorded only with signed 16..24-bit inputs, an unsigned " \
                          "32-bit output of the input's fraction 0 or 4, 1..3 x 2..3, Floor, saturation on, host)" % (
                              s, wl, fl, o_, shape, rnd, sat, ", ASIC/FPGA" if asic else "")
        return True, None
    if p != INH and _fixdt(p) not in PARAM_TYPES:
        return False, "ParamDataTypeStr %s (recorded: the internal rule, %s)" % (
            p, ", ".join("fixdt(%d,%d,%d)" % t for t in sorted(PARAM_TYPES)))
    o = str(out).strip()
    if o not in (INH, SAME):
        t = _fixdt(o)
        if t is None or not (OUT_WL[0] <= t[1] <= OUT_WL[1] and 0 <= t[2] <= t[1]):
            return False, "OutDataTypeStr %s (recorded: the internal rule, 'Same as input', fixdt with %d..%d bits " \
                          "and fraction 0..word length)" % (o, *OUT_WL)
    if o not in (INH, SAME) and _fixdt(o)[0] == 0 and _fixdt(o)[1] == 32:
        # recordings (host and ASIC): with an unsigned
        # 32-bit output Simulink sums a row exactly and saturates once when the products' fraction equals the
        # output's (10 designs, all integer products into fixdt(0,32,0)); when the products need a shift into the
        # output (product fraction below it: 31 designs) the per-product rule holds, as for 25..31-bit outputs.
        # Answered only there, with an explicit parameter type (an internal-rule parameter's fraction is not known
        # here and was not recorded with such an output)
        fo = _fixdt(o)[2]
        pt = _fixdt(p) if p != INH else None
        if pt is None:
            return False, "an unsigned 32-bit output type %s with an internal-rule parameter type (not recorded)" % o
        if pt[2] + fl >= fo:
            # recordings (38 models) with recordings: unshifted products (fraction = the output's)
            # follow the per-product rule when they are wider than 24 bits (fraction 0: 25..40 bits recorded) or have
            # a fraction (4: 24..40 bits recorded); integer products of 16..24 bits are summed exactly and saturated
            # once (13 designs), which this engine does not lower -- refused, as are unrecorded widths / fractions
            pw = it[1] + pt[1]
            per_product = (pt[2] + fl == fo) and ((fo == 0 and 25 <= pw <= 40) or (fo == 4 and 24 <= pw <= 40))
            # section 10: integer products of at most 24 bits are summed exactly and saturated / wrapped once
            # (fxgraph_lib Design._matgain_exact_rows). Recorded: parameter fixdt(1,8,0); unsigned 8..16-bit or
            # signed 16-bit integer inputs (16..24-bit products); 1..3 x 2..4; Floor, Nearest, Ceiling, Round, Zero;
            # saturation on and off; host and ASIC (GU 10, GU2 5, GP 3, GR 3 designs -- all the exact sum; the per-product rule fails on 13)
            m_, n_ = (int(x) for x in shape.split("x"))
            exact_sum = pt == (1, 8, 0) and fl == 0 and fo == 0 and pw <= 24 and \
                ((s == 0 and 8 <= wl <= 16) or (s == 1 and wl == 16)) and 1 <= m_ <= 3 and 2 <= n_ <= 4 and \
                str(rnd) in ("Floor", "Nearest", "Ceiling", "Round", "Zero")
            if not (per_product or exact_sum):
                return False, "an unsigned 32-bit output type %s with products of %d bits and fraction %d (Simulink " \
                              "sums such rows exactly where the products have at most 24 bits and fraction 0; this " \
                              "combination of types, shape and rounding is not recorded)" % (o, pw, pt[2] + fl)
    if str(rnd) not in RNDS or str(sat) not in ("on", "off"):
        return False, "RndMeth %s / saturation %s" % (rnd, sat)
    return True, None

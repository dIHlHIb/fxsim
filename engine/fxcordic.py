"""Complex to Magnitude-Angle and Magnitude-Angle to Complex with ApproximationMethod CORDIC (high-frequency partition
recordings). Rules from recordings (32 models, every stored input pair of
sfix8_En5 and 4096 pairs of sfix12_En9 / sfix16_En13, NumberOfIterations 3 / 6 / 11, ScaleReciprocalGainFactor on /
off; every step equal):

- Complex to Magnitude-Angle (u = re + j im, both parts of one type (1, w, f)):
  - angle = the engine's CORDIC atan2(im, re) (ref/fxcore._blk_trig, output (1, w, w - 3)) -- equal at every step;
  - magnitude: the same vectoring CORDIC on the stored integers (a point with re < 0 negated, im < 0 mirrored, then
    i = 0 .. N-1: im < 0 -> re -= im >> i, im += re >> i, else re += im >> i, im -= re >> i) gives x; with
    ScaleReciprocalGainFactor on, x times 1/K_N rounded to nearest in (1, w, w - 1), floored to f; the result in
    (1, w + 2, f), wrapping.
- Magnitude-Angle to Complex (magnitude and angle of one type (1, w, f)): the rotation CORDIC of ref/fxcore's sin /
  cos from (magnitude, 0) instead of (1/K, 0) -- the angle floored to w - 2 fraction bits, arctangents rounded to
  nearest there; beyond +-pi/2 the reduction of fxcore's candidate (the angle floored to G = max(w, 16) - 4 fraction
  bits, pi and pi/2 rounded to nearest there: beyond +-pi by 2 pi, then beyond +-pi/2 by pi with both results
  negated); then the gain correction as above; re and im in (1, w + 2, f), wrapping. (The candidate that main keeps
  off for sin / cos is equal here on every recorded point, angles to +-4 included.)

Answered only on the recorded input types, iteration counts and options; ApproximationMethod 'None' is refused as
Simulink refuses it on fixed-point inputs, and NumberOfIterations 11 on an 8-bit input as Simulink refuses it
(CordicIterationsGreaterThanInputWL).
"""
import json
import math
from fractions import Fraction

import fxgraph_lib as G

REC_IN = ((1, 8, 5), (1, 12, 9), (1, 16, 13))
REC_N = {8: (3, 6), 12: (3, 6, 11), 16: (3, 6, 11)}
TYPES = ("ComplexToMagnitudeAngle", "MagnitudeAngleToComplex")
PARAMS = {"ComplexToMagnitudeAngle": {"Output": "Magnitude and angle", "ApproximationMethod": "None",
                                      "NumberOfIterations": "11", "ScaleReciprocalGainFactor": "on",
                                      "SampleTime": "-1"},
          "MagnitudeAngleToComplex": {"Input": "Magnitude and angle", "ApproximationMethod": "None",
                                      "NumberOfIterations": "11", "ScaleReciprocalGainFactor": "on",
                                      "SampleTime": "-1"}}
REC = "recordings"


def _refuse(where, msg, code="UNSUPPORTED"):
    raise G.GraphError(code, where, msg)


def params(n, b):
    """The block's checked parameters (a dict) -- before any type is known."""
    t = b["type"]
    where = "design.blocks." + n
    extra = sorted(set(b) - {"type"} - set(PARAMS[t]))
    if extra:
        _refuse(where + "." + extra[0], "unknown parameter %s for %s (accepted: %s)"
                % (extra[0], t, ", ".join(sorted(PARAMS[t]))), "BAD_PARAM")
    p = dict(PARAMS[t], **{k: str(v).strip() for k, v in b.items() if k != "type"})
    io = "Output" if t == "ComplexToMagnitudeAngle" else "Input"
    if p[io] != "Magnitude and angle":
        _refuse(where + "." + io, "%s %s %r: 'Magnitude and angle' was recorded (%s)" % (t, io, p[io], REC))
    if p["ApproximationMethod"] == "None":
        _refuse(where + ".ApproximationMethod", "%s ApproximationMethod 'None' takes double / single inputs only "
                "(Simulink rejects fixed-point inputs: PortValidityDataTypeErrorMessage, %s); use 'CORDIC'"
                % (t, REC), "PORT_TYPE_INVALID")
    if p["ApproximationMethod"] != "CORDIC":
        _refuse(where + ".ApproximationMethod", "None or CORDIC", "BAD_PARAM")
    if p["ScaleReciprocalGainFactor"] not in ("on", "off"):
        _refuse(where + ".ScaleReciprocalGainFactor", "on or off", "BAD_PARAM")
    if p["SampleTime"] not in ("-1", ""):
        _refuse(where + ".SampleTime", "%s SampleTime %s: recorded inherited (-1) only" % (t, p["SampleTime"]))
    try:
        p["N"] = int(p["NumberOfIterations"])
    except ValueError:
        _refuse(where + ".NumberOfIterations", "a positive integer", "BAD_PARAM")
    return p


def core(op, part, p):
    """A CordicCore block (two inputs: re, im for 'c2ma'; magnitude, angle for 'ma2c')."""
    return {"type": "CordicCore", "Params": json.dumps({"op": op, "part": part, "N": p["N"],
                                                        "scale": p["ScaleReciprocalGainFactor"] == "on"})}


def out_type(p, ts, where):
    par = json.loads(p["Params"])
    t0, t1 = ts[0], ts[1] if len(ts) > 1 else None
    if not (isinstance(t0, tuple) and t0 == t1 and t0 in REC_IN):
        _refuse(where, "Complex to Magnitude-Angle / Magnitude-Angle to Complex on inputs %s: recorded with both "
                "parts (or magnitude and angle) of one of sfix8_En5, sfix12_En9, sfix16_En13 (%s); record this type "
                "to open it" % (", ".join(G._tstr(t) if isinstance(t, tuple) else str(t) for t in ts), REC))
    s, w, f = t0
    if par["N"] not in REC_N[w]:
        if w == 8 and par["N"] > 6:
            _refuse(where, "NumberOfIterations %d on an 8-bit input: Simulink rejects it "
                    "(CordicIterationsGreaterThanInputWL, %s)" % (par["N"], REC), "BAD_PARAM")
        _refuse(where, "NumberOfIterations %d on a %d-bit input: recorded %s (%s); record this count to open it"
                % (par["N"], w, ", ".join(map(str, REC_N[w])), REC))
    if par["op"] == "c2ma" and par["part"] == "ang":
        return (1, w, w - 3)
    return (1, w + 2, f)


def _rq(v):
    v = Fraction(v)
    return (v + Fraction(1, 2)).numerator // (v + Fraction(1, 2)).denominator


def _wrap(v, w):
    lo = -(1 << (w - 1))
    return (v - lo) % (1 << w) + lo


def _invk(N, w):
    k = 1.0
    for i in range(N):
        k *= 1 / math.sqrt(1 + 2.0 ** (-2 * i))
    return Fraction(_rq(Fraction(k) * 2 ** (w - 1)), 2 ** (w - 1))


def kernel(p, ts):
    par = json.loads(p["Params"])
    s, w, f = ts[0]
    N, scale, op, part = par["N"], par["scale"], par["op"], par["part"]
    ik = _invk(N, w)

    def gain(v):
        return (v * ik).__floor__() if scale else v

    if op == "c2ma" and part == "ang":
        return _atan2_kernel(N, w, f)
    if op == "c2ma":
        def k_mag(v):
            x, y = int(v[0]), int(v[1])
            if x < 0:
                x, y = -x, -y
            if y < 0:
                y = -y
            for i in range(N):
                if y < 0:
                    x, y = x - (y >> i), y + (x >> i)
                else:
                    x, y = x + (y >> i), y - (x >> i)
            return _wrap(gain(x), w + 2)
        return k_mag
    fz = w - 2
    hp = Fraction(math.pi) / 2
    G_ = max(w, 16) - 4

    def qg(c):
        return Fraction(_rq(Fraction(c) * 2 ** G_), 2 ** G_)
    PI, HP = qg(math.pi), qg(math.pi / 2)
    atans = [_rq(Fraction(math.atan(2.0 ** -i)) * 2 ** fz) for i in range(N)]

    def k_rot(v):
        mag, th = int(v[0]), Fraction(int(v[1]), 2 ** f)
        neg = False
        if abs(th) > hp:
            if f > G_:
                th = Fraction((th * 2 ** G_).__floor__(), 2 ** G_)
            if th > PI:
                th -= 2 * PI
            elif th < -PI:
                th += 2 * PI
            if th > HP:
                th, neg = th - PI, True
            elif th < -HP:
                th, neg = th + PI, True
        z = (th * 2 ** fz).__floor__()
        x, y = mag, 0
        for i in range(N):
            if z >= 0:
                x, y, z = x - (y >> i), y + (x >> i), z - atans[i]
            else:
                x, y, z = x + (y >> i), y - (x >> i), z + atans[i]
        if neg:
            x, y = -x, -y
        return _wrap(gain(x if part == "re" else y), w + 2)
    return k_rot


def _atan2_kernel(N, w, f):
    """ref/fxcore._blk_trig's atan2 CORDIC on stored integers (inputs re, im), output stored at w - 3 bits."""
    fz = w - 3
    pi_q = _rq(Fraction(math.pi) * 2 ** fz)
    atans = [_rq(Fraction(math.atan(2.0 ** -i)) * 2 ** fz) for i in range(N)]

    def k_ang(v):
        x, y = int(v[0]), int(v[1])
        if y == 0:
            return 0 if x >= 0 else pi_q
        add = 0
        if x < 0:
            add = pi_q if y >= 0 else -pi_q
            x, y = -x, -y
        flip = y < 0
        if flip:
            y = -y
        z = 0
        for i in range(N):
            if y < 0:
                x, y, z = x - (y >> i), y + (x >> i), z - atans[i]
            else:
                x, y, z = x + (y >> i), y - (x >> i), z + atans[i]
        return _wrap((-z if flip else z) + add, w)
    return k_ang

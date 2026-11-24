"""DSP HDL Toolbox NCO (dsphdlsigops2/NCO; dsphdlsrcs2/NCO is a library link to it), from recordings
(15 settings, 128 steps) and recordings (the same 8 settings at 64 steps, plus the phase increment from an
input port). Rules in recordings. Python path only.

Behaviour (every recorded step):
- phase accumulator, AccumulatorWL bits, wraps; it starts at 0 and adds the increment once per step
- the phase is quantised by dropping the low AccumulatorWL - NumQuantizerAccumulatorBits bits (q, Q bits)
- the output is round-to-nearest(sin(2 pi q / 2^Q) * 2^14) (cos likewise) in sfix16_En14
- the output for the phase after k increments appears at step k + 6, valid high from step 6; before that 0
- the increment from the dialog (PhaseIncrementSource Property), or from an input port (int16, accumulator 16 bits):
  the phase at output step t is the sum of the increments of steps 0 .. t - 7

the accumulator advances only on valid (gaps), PhaseOffset is added to the accumulator before the
quantiser, output fixdt(1, WL, WL - 2) for WL 12 / 16 / 18: round-half-up(sin * 2^FL).

Refused (not recorded): every other parameter value (dither -- deterministic but not reproduced --, LUT compression,
frame size, accumulator / quantiser / output types other than the recorded sets), Q > AccumulatorWL, increment-port
types other than INC_PORTS (int8 / int16 / int32 / uint16, sign-extended, into 16 / 20 / 32-bit
accumulators) and gaps in valid with an increment port other than int16 into 16 bits."""
import math

DEFAULTS = {"PhaseIncrementSource": "Property", "PhaseIncrement": "100", "Frequency": "510", "FreqResolution": "0.05",
            "SamplingFreq": "4000", "PhaseOffsetSource": "Property", "PhaseOffset": "0", "PhaseOffsetR": "pi/3",
            "DitherSource": "None", "NumDitherBits": "4", "SamplesPerFrame": "1", "LUTCompress": "off",
            "ResetAction": "off", "Waveform": "Sine", "PhasePort": "off", "RoundingMethod": "Floor",
            "OverflowAction": "Wrap", "AccumulatorDataType": "Binary point scaling", "AccumulatorSigned": "yes",
            "AccumulatorWL": "16", "AccumulatorFL": "0", "PhaseQuantization": "on", "NumQuantizerAccumulatorBits": "12",
            "OutputDataType": "Binary point scaling", "OutputSigned": "yes", "OutputWL": "16", "OutputFL": "14",
            "SimulateUsing": "Code generation"}
# the parameters the recorded models vary; every other one must keep the value above
FREE = {"PhaseIncrementSource": ("Property", "Input port"), "PhaseIncrement": None,
        # the quantised phase as an output, the phase offset from an input port
        "PhasePort": ("off", "on"), "PhaseOffsetSource": ("Property", "Input port"),
        # the accumulator reset port
        "ResetAction": ("off", "on"),
        # dither from the block's own PN generator
        "DitherSource": ("None", "Property"),
        "Waveform": ("Sine", "Cosine", "Sine and cosine", "Complex exponential"), "AccumulatorWL": None,
        # recordings (96 models): the compressed table, in H's setting only (see lut_value)
        "LUTCompress": ("off", "on"),
        "NumQuantizerAccumulatorBits": None, "SimulateUsing": ("Code generation", "Interpreted execution"),
        # phase offset from the dialog, output types 18/16, 12/10
        "PhaseOffset": None, "OutputWL": None, "OutputFL": None,
        # unused by the recorded configuration (dither off, increment from the dialog / port)
        "Frequency": None, "FreqResolution": None, "SamplingFreq": None, "NumDitherBits": None, "PhaseOffsetR": None}
WAVE_PORTS = {"Sine": ["sin"], "Cosine": ["cos"], "Sine and cosine": ["sin", "cos"], "Complex exponential": ["cplx"]}
LATENCY = 6
# the phase increment port: (port type, AccumulatorWL, NumQuantizerAccumulatorBits) recorded (recordings int16
# 16/12; recordings int8, uint16 (also ufix16, compiled uint16), int32 32/12 and 32/16, int16 and uint16 into
# a 20-bit accumulator). The increment is the port's value (signed ports sign-extended), added modulo 2^AccumulatorWL
# on valid samples; valid gaps recorded with int16 16/12 only
INC_PORTS = {((1, 16, 0), 16, 12), ((1, 8, 0), 16, 12), ((0, 16, 0), 16, 12), ((1, 32, 0), 32, 12),
             ((1, 32, 0), 32, 16), ((1, 16, 0), 20, 12), ((0, 16, 0), 20, 12)}
INC_PORT_GAPS = {((1, 16, 0), 16, 12)}
# (Waveform, OutputWL, OutputFL, PhasePort) recorded with an increment port: every combination above with the
# defaults, and per combination the further settings of recordings
INC_PORT_SETTINGS_ALL = {("Sine and cosine", 16, 14, "off")}
INC_PORT_SETTINGS = {((1, 16, 0), 16, 12): {("Sine", 16, 14, "off")},
                     ((1, 8, 0), 16, 12): {("Sine and cosine", 12, 10, "off"), ("Sine and cosine", 18, 16, "off"),
                                           ("Sine", 16, 14, "off"), ("Cosine", 16, 14, "off"),
                                           ("Complex exponential", 16, 14, "off")},
                     ((1, 32, 0), 32, 12): {("Sine and cosine", 16, 14, "on")},
                     ((0, 16, 0), 20, 12): {("Sine", 16, 14, "off")}}
# the recorded values
ACC_WLS = (12, 16, 20, 24, 32)
QUANT_BITS = (8, 10, 12, 14, 16, 18)
OUTPUT_TYPES = ((12, 10), (16, 14), (18, 16))
# recordings (192 models, LUTCompress off every step equal): output types with more integer bits,
# accumulators 16 / 24, dither none or 4 bits, every waveform -- accepted in either setting
KP_OUT_TYPES = ((16, 10), (16, 8), (12, 4), (20, 12), (16, 14), (8, 2))
# recordings (50 accepted models equal): with the increment from the dialog, dither off, no phase offset
# port and no reset port (the sweep's setting) -- AccumulatorWL 8 .. 64, NumQuantizerAccumulatorBits 4 .. 18 and not
# above AccumulatorWL (Simulink refuses more than 19: dsphdl:NCO:NumQuantAccumBitsTooLarge; more than the accumulator:
# dsphdl:NCO:accWordLenNotGtQuanWordLen), outputs fixdt(1, WL, FL) with WL 8 .. 32, FL >= 6, WL - FL 1 .. 4 (recorded
# 8/6 .. 32/30). The output saturates symmetrically to +-(2^(WL-1) - 1) (FL = WL - 1: sin -1 gives -32767 in 16/15).
# AccumulatorWL 6, 7 and 65, NumQuantizerAccumulatorBits 19 recorded equal beyond the recorded sweep
SWEPT_ACC, SWEPT_Q, SWEPT_OUT_WL, SWEPT_OUT_FL_MIN, SWEPT_OUT_IBITS = (6, 65), (4, 19), (8, 32), 6, (1, 4)
# recordings (56 models): exactly one of dither (NumDitherBits 1 / 4 / 6 / 8), a phase increment port,
# the phase output, a phase offset (dialog value or a ufix(AccumulatorWL) port) on, no reset port -- AccumulatorWL
# 8 .. 48, quantisers 6 .. 16 (not above the accumulator), the recordings output envelope; increment ports int8 / 16 /
# 32 and uint16 not wider than the accumulator, or the accumulator's own width (signed or unsigned); valid gaps too
L63_ACC, L63_Q, L63_DITHER = (8, 48), (6, 16), (1, 4, 6, 8)
L63_INC = {(1, 8, 0), (1, 16, 0), (1, 32, 0), (0, 16, 0)}
# recordings (216 models, all run; the native the regression check): no dither / reset / phase offset
# (port or dialog value), code generation, valid always 1; AccumulatorWL 16 / 24 / 32, NumQuantizerAccumulatorBits
# 8 / 12 / 16 and below the accumulator, every waveform, the increment from the dialog or a ufix(AccumulatorWL) port,
# the phase output off / on, LUTCompress off / on, outputs fixdt(1, WL, FL) of NC_OUT_TYPES (the generator's list)
NC_ACC, NC_Q = (16, 24, 32), (8, 12, 16)
NC_OUT_TYPES = ((8, 7), (12, 10), (12, 7), (16, 15), (16, 13), (16, 11), (16, 10), (16, 8), (18, 16), (18, 12),
                (24, 20), (24, 16))


def nc_setting(p, a, q, wl, fl):
    """whether the NCO's settings lie in recordings's recorded domain (the increment port's type is
    checked by the kernel: ufix(AccumulatorWL) only)"""
    return (p.get("DitherSource") == "None" and p.get("ResetAction") == "off"
            and p.get("PhaseOffsetSource") == "Property" and p.get("PhaseOffset", "0").strip() in ("0", "0.0")
            and p.get("SimulateUsing") == "Code generation" and a in NC_ACC and q in NC_Q and q < a
            and (wl, fl) in NC_OUT_TYPES)


def l63_setting(p):
    """whether the NCO's settings are those recordings show (one feature on at a time)"""
    off_dialog = p.get("PhaseOffsetSource") == "Property" and str(p.get("PhaseOffset", "0")).strip() not in ("0", "0.0")
    dither = p.get("DitherSource") == "Property"
    feats = [dither, p["PhaseIncrementSource"] == "Input port", p.get("PhasePort") == "on",
             p.get("PhaseOffsetSource") == "Input port", off_dialog]
    if sum(feats) != 1 or p.get("ResetAction", "off") != "off":
        return False
    if dither:
        try:
            return int(str(p.get("NumDitherBits", "4")).strip()) in L63_DITHER
        except ValueError:
            return False
    return True


def l63_inc_ok(t, a):
    """an increment port type recordings show with an a-bit accumulator"""
    return isinstance(t, tuple) and t[2] == 0 and ((t in L63_INC and t[1] <= a) or t[1] == a)
# the dither is one PN bit stream, s(n) = s(n-1) ^ s(n-2) ^ s(n-5) ^ s(n-19), started
# from 1 followed by 18 zeros; each sample takes the next NumDitherBits bits (first bit most significant) and adds
# them to the accumulator before quantisation -- the accumulator itself does not keep them (506 / 506 steps of
# NumDitherBits 1, 4, 6 with a zero increment, and of NumDitherBits 4 with increment 100, phase and sine); the
# generator advances on valid samples only (advancing every step fails)
DITHER_TAPS = (1, 2, 5, 19)
DITHER_SEED = (1,) + (0,) * 18
DITHER_BITS = (1, 4, 6)


class _Pn:
    def __init__(self):
        self.bits = list(DITHER_SEED)
        self.pos = 0

    def take(self, nb):
        while len(self.bits) < self.pos + nb:
            k = len(self.bits)
            x = 0
            for i in DITHER_TAPS:
                x ^= self.bits[k - i]
            self.bits.append(x)
        v = 0
        for b in self.bits[self.pos:self.pos + nb]:
            v = (v << 1) | b
        self.pos += nb
        if self.pos > 4096:   # keep the list short: only the last 19 bits feed the recurrence
            del self.bits[:self.pos - 19]
            self.pos = 19
        return v


# recordings.
# Every recorded output step and compiled type equals the rules above (phase accumulator, dither PN stream, plain or
# compressed table, latency 6) in these settings; each set is the generator's axis, sampled jointly:
# - 'lutc' (g3, 76 designs): LUTCompress on, no dither / phase output / reset / offset, code generation; quantiser
#   10 / 12 / 14 / 16 / 18 (every one of the 60 (q, WL, WL - FL) triples recorded), outputs fixdt(1, WL, FL) with WL
#   10 / 14 / 20 / 32 and WL - FL 1 / 2 / 4, AccumulatorWL 16 .. 48 (16 / 24 / 32 / 48 recorded) above q, every
#   waveform, the increment from the dialog or a ufix(AccumulatorWL) port (valid always 1 with the port)
# - 'lutc_nl1' (candidate N-L1): the (q, output) pairs of recordings
#   M in the same setting with any AccumulatorWL 16 .. 48 -- g3 shows the table does not depend on the accumulator
# - 'multi' (g4, 78 designs): exactly two of a phase increment port, dither, the phase output, LUTCompress off;
#   AccumulatorWL 16 / 20 / 24 / 32 / 40 / 48, quantiser 8 / 12 / 16 below it, NumDitherBits 1 / 2 / 4 / 5 / 8,
#   ports int16 / uint16 / int32 (accumulators of 32 bits or more) / ufix(AccumulatorWL), the six outputs of L116_G4_OUT,
#   every waveform, valid gaps recorded
# - 'q19' (g5, 24 designs): exactly one feature (a ufix(AccumulatorWL) increment port, 4 dither bits or the phase
#   output), quantiser 16 .. 19, AccumulatorWL 20 / 24 / 32 / 48, outputs 16/14, 18/16, 24/22, Sine / Sine and
#   cosine, valid always 1
# - 'wide' (g6, 24 designs): an increment port wider than the accumulator -- int32 / uint32 / sfix(AccumulatorWL + 8)
#   into 16 / 20 / 24 / 28 bits, quantiser 12, Sine and cosine 16/14, the phase output off / on, valid always 1: the
#   port's value (signed ports sign-extended) is added modulo 2^AccumulatorWL
# Simulink errors recorded there: quantiser above 19 with LUTCompress off (g5: validatePropertiesImpl), quantiser
# above the accumulator (g3), NumDitherBits 0 (g4: SetPropGeneralError), an increment port with fraction bits (g6:
# validateInputsImpl). Recorded runs still refused: LUTCompress on with quantiser 20 (its compressed table differs from
# lut_value at 2 of 2 designs: a 4-bit fine word, not fine_wl), LUTCompress with 'Interpreted execution' (2
# designs), three features at once (1 design), dither with a reset and a phase offset port (1 design), a double
# increment port (the engine has no double signals).
L116_G3_Q, L116_G3_WL, L116_G3_IBITS, L116_ACC_RANGE = (10, 12, 14, 16, 18), (10, 14, 20, 32), (1, 2, 4), (16, 48)
L116_M_OUT, L116_M_WAVES = ((16, 14), (12, 10), (16, 10), (18, 16)), ("Sine", "Complex exponential", "Sine and cosine")
L116_G4_ACC, L116_G4_Q, L116_G4_NB = (16, 20, 24, 32, 40, 48), (8, 12, 16), (1, 2, 4, 5, 8)
L116_G4_OUT = ((16, 15), (24, 22), (16, 14), (18, 16), (20, 18), (14, 12))
L116_G5_ACC, L116_G5_Q, L116_G5_OUT = (20, 24, 32, 48), (16, 17, 18, 19), ((16, 14), (18, 16), (24, 22))
L116_G5_WAVES = ("Sine", "Sine and cosine")
L116_G6_ACC = (16, 20, 24, 28)
# the settings whose increment-port recordings have valid gaps. Not 'np1': on the INC_PORTS tuples the recordings rule
# (gaps on every type; recorded with u48 / s32 / u16 into 48 / 32 / 18 bits) and INC_PORT_GAPS (int16 16/12, Sine
# and cosine only) disagree -- opened only where they agree, valid always 1
L116_GAPS = ("multi",)
Q_MAX = 19   # recordings (NumQuantAccumBitsTooLarge), recordings: more quantiser bits stop Simulink


def _l116_base(p):
    return (p.get("ResetAction") == "off" and p.get("PhaseOffsetSource") == "Property"
            and str(p.get("PhaseOffset", "0")).strip() in ("0", "0.0") and p.get("SimulateUsing") == "Code generation"
            and str(p.get("SamplesPerFrame", "1")).strip() == "1")


def l116_tags(p, a, q, wl, fl):
    """the recordings settings (see above) the NCO's parameters lie in; the increment port's type is checked by
    l116_port_ok"""
    if not _l116_base(p):
        return set()
    inc_port = p.get("PhaseIncrementSource") == "Input port"
    dith = p.get("DitherSource") == "Property"
    ph = p.get("PhasePort") == "on"
    wave = p.get("Waveform")
    try:
        nb = int(str(p.get("NumDitherBits", "4")).strip())
    except ValueError:
        return set()
    tags = set()
    if p.get("LUTCompress") == "on":
        if not dith and not ph and L116_ACC_RANGE[0] <= a <= L116_ACC_RANGE[1] and q < a:
            if q in L116_G3_Q and wl in L116_G3_WL and wl - fl in L116_G3_IBITS:
                tags.add("lutc")
            if (q in NC_Q and (wl, fl) in NC_OUT_TYPES) or (q in NC_Q and (wl, fl) in L116_M_OUT and wave in
                                                            L116_M_WAVES):
                tags.add("lutc_nl1")
        return tags
    if p.get("LUTCompress") != "off":
        return tags
    nfeat = int(inc_port) + int(dith) + int(ph)
    if nfeat == 2 and a in L116_G4_ACC and q in L116_G4_Q and q < a and (not dith or nb in L116_G4_NB) and \
            (wl, fl) in L116_G4_OUT:
        tags.add("multi")
    if nfeat == 1 and a in L116_G5_ACC and q in L116_G5_Q and (wl, fl) in L116_G5_OUT and wave in L116_G5_WAVES and \
            (not dith or nb == 4):
        tags.add("q19")
    if inc_port and not dith and a in L116_G6_ACC and q == 12 and wave == "Sine and cosine" and (wl, fl) == (16, 14):
        tags.add("wide")
    return tags


def l116_port_ok(tag, t, a):
    """the increment port types recorded in a recordings setting"""
    if not isinstance(t, tuple) or t[2] != 0:
        return False
    if tag in ("lutc", "lutc_nl1", "q19"):
        return t == (0, a, 0)
    if tag == "multi":
        return t in ((1, 16, 0), (0, 16, 0), (0, a, 0)) or (t == (1, 32, 0) and a >= 32)
    if tag == "wide":
        return t in ((1, 32, 0), (0, 32, 0), (1, a + 8, 0))
    return False


def _l116_port_tags(p, a, q, t, wl, fl):
    """the recordings settings, and the single-feature configurations ('np1': the recordings one-feature setting and port rule, also on the port
    tuples of INC_PORTS that the old kernel left to the narrower INC_PORT_SETTINGS), accepting increment port t"""
    ok = [tg for tg in sorted(l116_tags(p, a, q, wl, fl)) if l116_port_ok(tg, t, a)]
    if l63_setting(p) and l63_inc_ok(t, a) and p.get("PhaseIncrementSource") == "Input port" and \
            L63_ACC[0] <= a <= L63_ACC[1] and L63_Q[0] <= q <= min(L63_Q[1], a) and \
            p.get("LUTCompress") == "off" and _l116_base(p) and SWEPT_OUT_WL[0] <= wl <= SWEPT_OUT_WL[1] and \
            fl >= SWEPT_OUT_FL_MIN and SWEPT_OUT_IBITS[0] <= wl - fl <= SWEPT_OUT_IBITS[1]:
        ok.append("np1")
    return ok


def _params_116(b, where):
    """params() for a design the earlier domains refuse: Simulink's recorded errors, then the recordings settings
    (l116_tags); None when neither applies (the earlier refusal stands)"""
    import parameter_bounds as _lb
    p = _read(b, where)

    def num(k):
        try:
            return _lb.pint(p[k], where + "." + k)
        except Exception:   # noqa: BLE001 -- the earlier refusal reports it
            return None
    a, q, wl, fl = num("AccumulatorWL"), num("NumQuantizerAccumulatorBits"), num("OutputWL"), num("OutputFL")
    if None in (a, q, wl, fl):
        return None
    if p.get("DitherSource") == "Property":
        nb = num("NumDitherBits")
        if nb is not None and nb < 1:
            _refuse(where + ".NumDitherBits", ('NumDitherBits=%d: Simulink rejects it when the parameter '
                'is set (SystemBlock:MATLABSystem:SetPropGeneralError)') % nb, "BAD_PARAM")
    if p.get("LUTCompress") == "off" and q > Q_MAX:
        _refuse(where + ".NumQuantizerAccumulatorBits", ("NumQuantizerAccumulatorBits=%d with LUTCompress 'off': Simulink "
            'rejects more than %d in validatePropertiesImpl '
            '(SystemBlock:MATLABSystem:MethodInvokeErrorWithStack; dsphdl:NCO:NumQuantAccumBitsTooLarge)')
                % (q, Q_MAX), "BAD_PARAM")
    if q > a:
        _refuse(where + ".NumQuantizerAccumulatorBits", ('more quantiser bits (%d) than accumulator bits (%d): '
            'Simulink refuses it (dsphdl:NCO:accWordLenNotGtQuanWordLen)') % (q, a), "BAD_PARAM")
    if p.get("LUTCompress") == "on" and q > Q_MAX:
        _refuse(where + ".NumQuantizerAccumulatorBits", ("NumQuantizerAccumulatorBits=%d with LUTCompress 'on': Simulink "
            'runs it, but its compressed table was not reproduced (q = 20 differs '
            'from the implemented compressed-table rule); record it to open') % q)
    if not l116_tags(p, a, q, wl, fl):
        nfeat = sum((p.get("PhaseIncrementSource") == "Input port", p.get("DitherSource") == "Property",
                     p.get("PhasePort") == "on", p.get("ResetAction") == "on", p.get("PhaseOffsetSource") == "Input port"))
        if nfeat >= 3 and p.get("LUTCompress") == "off":
            # Simulink runs them; one design each
            _refuse(where, ('three or more of the phase increment port, dither, the phase '
                'output, the reset port and the phase offset port at once: '
                'recorded once per combination, not opened -- two at a time are'))
        return None
    for k, lo, hi in (("AccumulatorWL", 16, 48), ("NumQuantizerAccumulatorBits", 8, Q_MAX), ("OutputWL", 8, 32),
                      ("OutputFL", 2, 31), ("PhaseOffset", 0, 0)):
        _int(p, k, where, lo, hi)
    if p.get("DitherSource") == "Property":
        _int(p, "NumDitherBits", where, 1, 8)
    inc = None
    if p["PhaseIncrementSource"] == "Property":
        inc = int(float(_int(p, "PhaseIncrement", where, -(1 << 64), 1 << 64)))
    return p, a, q, inc


def out_type(p):
    return (1, int(p["OutputWL"]), int(p["OutputFL"]))


def _refuse(where, msg, code="UNSUPPORTED"):
    from fxgraph_lib import GraphError
    raise GraphError(code, where, msg)


def _int(p, k, where, lo, hi):
    import parameter_bounds
    v = parameter_bounds.pint(p[k], where + "." + k)   # fxnum's parser, not int()
    if not lo <= v <= hi:
        _refuse(where + "." + k, "%s=%d outside the recorded range %d..%d" % (k, v, lo, hi))
    # the value as read here ('2^10' -> 1024) for every later reader (int() of the text crashed)
    p[k] = str(v)
    return v


# LUTCompress on (96 models, 12,679 distinct (phase, output) samples; recordings: three
# full tables; recordings: 396 models, 46,187 samples over quantisers 8 / 12 / 16 and outputs
# 12/10, 16/10, 16/14, 18/16 -- every one equal). The q-bit phase is a quadrant (2 bits) and a quarter-wave index x
# (n = q - 2 bits) split A | B | C with A = B = ceil(n / 3) bits and C the rest (q 8 / 10 / 12 / 16: 2|2|2, 3|3|2,
# 4|4|2, 5|5|4); a coarse table holds sin at x - C rounded to nearest at the output fraction, a fine table
# sin(C) * cos(A) (A alone, B dropped) in a signed word of fine_wl(WL) = ceil(WL / 4) + 1 bits at the best precision for its largest
# value (fraction PF, rounded to nearest); their sum is floored to the output fraction. The second and fourth quadrants
# mirror the index as 2^n - x (the peak, x = 0 there, is +-1 saturated as the plain table); the third and fourth negate.


def lutc_split(q, wl):
    """(A bits, C bits, PF) of the compressed table for a q-bit phase into a wl-bit output."""
    n = q - 2
    ab = -(-n // 3)
    cb = n - 2 * ab
    mx = math.sin(math.pi / 2 / (1 << n) * ((1 << cb) - 1))
    # the fine word: ceil(WL / 4) + 1 bits (WL 8 -> 3, 24 -> 7 bits; floor(WL / 3)
    # agreed on every earlier recording, WL 12 / 16 / 18 / 20, and was wrong at WL 8 with q 8 and at WL 24)
    return ab, cb, fine_wl(wl) - 1 - (math.floor(math.log2(mx)) + 1)


def fine_wl(wl):
    """the fine table's word length for a wl-bit output (recorded WL 8, 12, 16, 18, 20, 24)"""
    return -(-wl // 4) + 1


def _lutc_quarter(x, n, wl, fl, pf, ab, cb):
    h = math.pi / 2 / (1 << n)
    c = x & ((1 << cb) - 1)
    a = (x >> (n - ab)) << (n - ab)
    co = math.floor(math.sin(h * (x - c)) * (1 << fl) + 0.5)                      # at 2^-fl
    fi = math.floor(math.sin(h * c) * math.cos(h * a) * (1 << pf) + 0.5)        # at 2^-pf
    # floor(co * 2^-fl + fi * 2^-pf) at 2^-fl, exactly
    if pf >= fl:
        return (co * (1 << (pf - fl)) + fi) >> (pf - fl)
    return co + fi * (1 << (fl - pf))


def lut_value(ph, q, sel, wl, fl, compress=False):
    """The output stored integer of the NCO at quantised phase ph (q bits): round-half-up(sin / cos(2 pi ph / 2^q) *
    2^fl), saturated to +-(2^(wl-1) - 1); with LUTCompress on the compressed table above."""
    top = (1 << (wl - 1)) - 1
    if not compress:
        x = 2 * math.pi * ph / (1 << q)
        v = math.sin(x) if sel == "sin" else math.cos(x)
        # saturated symmetrically, to +-(2^(WL-1) - 1) (with FL = WL - 1, sin -1 gives -32767 in 16/15)
        return max(-top, min(top, math.floor(v * (1 << fl) + 0.5)))
    ab, cb, pf = lutc_split(q, wl)
    n = q - 2
    if sel != "sin":
        ph = (ph + (1 << n)) % (1 << q)
    qd, x = ph >> n, ph & ((1 << n) - 1)
    if qd in (1, 3):
        v = (1 << fl) if x == 0 else _lutc_quarter((1 << n) - x, n, wl, fl, pf, ab, cb)
    else:
        v = _lutc_quarter(x, n, wl, fl, pf, ab, cb)
    if qd >= 2:
        v = -v
    return max(-top, min(top, v))


def _read(b, where):
    """the block's parameters (unknown ones BAD_PARAM, fixed ones at their only recorded value) with the defaults"""
    p = {}
    for k, v in b.items():
        if k == "type":
            continue
        if k not in DEFAULTS:
            _refuse(where + "." + k, "unknown parameter %s for HDLNCO" % k, "BAD_PARAM")
        if not isinstance(v, str):
            _refuse(where + "." + k, "text value required", "BAD_PARAM")
        v = v.strip()
        allowed = FREE.get(k, ())
        if k not in FREE and v != DEFAULTS[k]:
            _refuse(where + "." + k, "%s=%r is not supported (only %r was recorded)" % (k, v, DEFAULTS[k]))
        if allowed and v not in allowed:
            _refuse(where + "." + k, "%s=%r (recorded: %s)" % (k, v, ", ".join(repr(x) for x in allowed)))
        p[k] = v
    for k, v in DEFAULTS.items():
        p.setdefault(k, v)
    if p.get("DitherSource") != "Property":
        # Simulink rejects NumDitherBits 0 when it is set (with dither on); with
        # dither off a value below 1 was never recorded -- refused rather than assumed ignored
        try:
            nb0 = int(str(p.get("NumDitherBits", "4")).strip())
        except ValueError:
            nb0 = None
        if nb0 is not None and nb0 < 1:
            _refuse(where + ".NumDitherBits", "NumDitherBits=%d with DitherSource 'None': not recorded (with dither "
                    "on Simulink rejects it: SystemBlock:MATLABSystem:SetPropGeneralError)" % nb0)
    return p


def params(b, where):
    """(parameters, AccumulatorWL, NumQuantizerAccumulatorBits, the dialog increment or None for the port). The
    domains signed before recordings first; a design they refuse (UNSUPPORTED) is tried against the recordings domain
, so nothing they answer changes."""
    from fxgraph_lib import GraphError
    try:
        return _params_old(b, where)
    except GraphError as e:
        if e.code != "UNSUPPORTED":
            raise
        r = _params_116(b, where)
        if r is None:
            raise
        return r


def _params_old(b, where):
    p = _read(b, where)
    # the sweep's setting: increment from the dialog, no dither, no phase offset (port or dialog value) /
    # reset port, no phase output
    swept = (p["PhaseIncrementSource"] == "Property" and p.get("DitherSource") == "None"
             and p.get("PhaseOffsetSource") == "Property" and p.get("ResetAction") == "off"
             and p.get("PhasePort") == "off" and p.get("PhaseOffset", "0").strip() in ("0", "0.0"))
    l63 = not swept and l63_setting(p)
    a = _int(p, "AccumulatorWL", where, *(SWEPT_ACC if swept else (L63_ACC if l63 else (12, 32))))
    q = _int(p, "NumQuantizerAccumulatorBits", where, *(SWEPT_Q if swept else (L63_Q if l63 else (8, 18))))
    # the recorded sets, not the ranges between them (the ranges of
    # for the setting it swept)
    if a not in ACC_WLS and not (swept or l63):
        _refuse(where + ".AccumulatorWL", 'AccumulatorWL %d: recorded %s (and %d.. %d with the increment from the '
                "dialog, no dither and no phase offset / reset / phase ports)" % (
                    a, ", ".join(map(str, ACC_WLS)), SWEPT_ACC[0], SWEPT_ACC[1]))
    if q not in QUANT_BITS and not (swept or l63):
        _refuse(where + ".NumQuantizerAccumulatorBits", ('NumQuantizerAccumulatorBits %d: recorded %s '
            '(and %d.. %d for the recorded configuration)') % (q, ", ".join(map(str, QUANT_BITS)), SWEPT_Q[0], SWEPT_Q[1]))
    if q > a:
        _refuse(where + ".NumQuantizerAccumulatorBits", "more quantiser bits (%d) than accumulator bits (%d): Simulink "
                "refuses it (dsphdl:NCO:accWordLenNotGtQuanWordLen)" % (q, a), "BAD_PARAM")
    import parameter_bounds as _lb
    kp = (_lb.pint(p["OutputWL"], where + ".OutputWL"), _lb.pint(p["OutputFL"], where + ".OutputFL")) in KP_OUT_TYPES \
        and p["PhaseIncrementSource"] == "Property" and p.get("PhaseOffsetSource") == "Property" \
        and p.get("ResetAction") == "off" and p.get("PhasePort") == "off" \
        and p.get("PhaseOffset", "0").strip() in ("0", "0.0") and a in (16, 24) \
        and (p.get("DitherSource") == "None" or (p.get("DitherSource") == "Property" and
                                                 str(p.get("NumDitherBits", "4")).strip() == "4"))   # (H's setting)
    # recordings (396 models, LUTCompress on, every step equal): no dither / reset / phase output,
    # the offset from the dialog, the increment from the dialog or a port of the accumulator's width; accumulators
    # 16 / 20 / 24 / 32, quantisers 8 / 12 / 16 (not above the accumulator), outputs 16/14, 12/10, 16/10, 18/16, Sine /
    # Complex exponential / Sine and cosine
    m_set = p.get("LUTCompress") == "on" and p.get("DitherSource") == "None" and p.get("ResetAction") == "off" \
        and p.get("PhasePort") == "off" and p.get("PhaseOffsetSource") == "Property" and a in (16, 20, 24, 32) \
        and q in (8, 12, 16) and q <= a and p["Waveform"] in ("Sine", "Complex exponential", "Sine and cosine") \
        and (_lb.pint(p["OutputWL"], where + ".OutputWL"), _lb.pint(p["OutputFL"], where + ".OutputFL")) in \
        ((16, 14), (12, 10), (16, 10), (18, 16))
    nc = nc_setting(p, a, q, _lb.pint(p["OutputWL"], where + ".OutputWL"), _lb.pint(p["OutputFL"], where + ".OutputFL"))
    if p.get("LUTCompress") == "on":
        # LUTCompress on was recorded (every sample equal to lut_value, simulated with code generation) in
        # recordings's setting with 12 quantiser bits and the output types of KP_OUT_TYPES, and in
        # the recorded (the phase output on, Sine and cosine, a 16-bit accumulator, no dither / ports / offset)
        # with q 12 into 16/14 and 12/10 and q 10 into 16/14
        l22 = (p["PhaseIncrementSource"] == "Property" and p.get("PhaseOffsetSource") == "Property"
               and p.get("ResetAction") == "off" and p.get("PhasePort") == "on" and p["Waveform"] == "Sine and cosine"
               and p.get("PhaseOffset", "0").strip() in ("0", "0.0") and a == 16 and p.get("DitherSource") == "None"
               and (q, _lb.pint(p["OutputWL"], where + ".OutputWL"), _lb.pint(p["OutputFL"], where + ".OutputFL"))
               in ((12, 16, 14), (12, 12, 10), (10, 16, 14)))
        if not ((kp and q == 12) or l22 or m_set or nc) or p.get("SimulateUsing") != "Code generation":
            _refuse(where + ".LUTCompress", "LUTCompress 'on' was recorded with NumQuantizerAccumulatorBits 12, "
                    "AccumulatorWL 16 / 24, the increment from the dialog, dither off or 4 bits, no phase offset / "
                    "reset / phase ports and outputs %s, and with the phase output, Sine "
                    "and cosine, a 16-bit accumulator: quantiser 12 into fixdt(1,16,14) / fixdt(1,12,10), quantiser 10 "
                    "into fixdt(1,16,14); and without dither / ports, accumulators 16 / 20 / 24 / 32, quantisers "
                    "8 / 12 / 16, outputs 16/14, 12/10, 16/10, 18/16; record this "
                    "setting to open it"
                    % ", ".join("fixdt(1,%d,%d)" % t for t in KP_OUT_TYPES))
    wl = _int(p, "OutputWL", where, *((8, 24) if nc else (8, 20) if (kp or m_set) else SWEPT_OUT_WL if (swept or l63) else (12, 18)))
    fl = _int(p, "OutputFL", where, *((7, 20) if nc else (2, 16) if (kp or m_set) else (SWEPT_OUT_FL_MIN, SWEPT_OUT_WL[1]) if
                                      (swept or l63) else (8, 16)))
    if kp or m_set or nc:
        pass
    elif swept or l63:
        if not SWEPT_OUT_IBITS[0] <= wl - fl <= SWEPT_OUT_IBITS[1]:
            _refuse(where + ".OutputFL", 'output fixdt(1,%d,%d): recorded with %d.. %d integer bits (word length - '
                    "fraction length)" % (wl, fl, SWEPT_OUT_IBITS[0], SWEPT_OUT_IBITS[1]))
    else:
        if fl > wl - 2:
            _refuse(where + ".OutputFL", "output fixdt(1,%d,%d): 1.0 does not fit (recorded: fraction length = word "
                    "length - 2)" % (wl, fl))
        if (wl, fl) not in OUTPUT_TYPES:
            _refuse(where + ".OutputWL", "output fixdt(1,%d,%d): recorded %s" % (
                wl, fl, ", ".join("fixdt(1,%d,%d)" % x for x in OUTPUT_TYPES)))
    # (offsets up to the accumulator's range on accumulators up to 48 bits, e.g. 211567209935712 on 48)
    # recordings: +-70000 on a 16-bit accumulator runs and is taken modulo 2^16 (equal)
    _ob = max(1 << a, 70000) if a == 16 else 1 << a
    _int(p, "PhaseOffset", where, *((-_ob, _ob) if l63 else (-(1 << 40), 1 << 40)))
    if p.get("DitherSource") == "Property":
        nb = _int(p, "NumDitherBits", where, 1, 32)
        # 1 / 4 / 6 bits with a 16-bit accumulator; recordings: 4 bits into 20 bits, quantiser 12
        # (32 bits recorded too, but its dither never reached the quantised phase: not shown, refused)
        if not ((nb in DITHER_BITS and a == 16) or (nb == 4 and a == 20 and q == 12) or l63):
            _refuse(where + ".NumDitherBits", "dither with %d bits and a %d-bit accumulator: recorded %s bits with a "
                    "16-bit accumulator, 4 bits with a 20-bit one and 12 quantiser bits"
                    % (nb, a, ", ".join(map(str, DITHER_BITS))))
        if p.get("PhaseOffsetSource") != "Property" or \
                (p.get("ResetAction") == "on" and not (nb == 4 and a == 16 and p["PhaseIncrementSource"] == "Property")) \
                or (p["PhaseIncrementSource"] != "Property" and a != 16):
            _refuse(where + ".DitherSource", "dither was recorded without a phase offset port or reset port, and "
                    "with an increment port only into 16 bits")
    if p.get("ResetAction") == "on" and p.get("PhaseOffsetSource") != "Property":
        _refuse(where + ".ResetAction", "the reset port was recorded without a phase offset port and without the "
                "phase output only")
    if p.get("PhaseOffsetSource") == "Input port" and a != 16 and not l63:
        _refuse(where + ".PhaseOffsetSource", 'a phase offset port was recorded with a 16-bit accumulator only'
                "")
    if p["PhaseIncrementSource"] == "Property":
        # Simulink evaluates the dialog value as a double -- an integer above 2^53 (2^63 - 1,
        # 4194639225976689342) is first rounded to the nearest double (the 64-bit accumulator records)
        inc = _int(p, "PhaseIncrement", where, -(1 << 64), 1 << 64)
        inc = int(float(inc))
    else:
        if a not in {k[1] for k in INC_PORTS} and not (l63 or nc):
            _refuse(where + ".AccumulatorWL", "a phase increment port with a %d-bit accumulator: recorded %s" % (
                a, ", ".join(map(str, sorted({k[1] for k in INC_PORTS})))))
        inc = None
    return p, a, q, inc


def ports(p):
    """Output ports in order: 'sin', 'cos', 'cplx' (complex: re cos, im sin), ['phase',] then 'valid'."""
    return WAVE_PORTS[p["Waveform"]] + (["phase"] if p.get("PhasePort") == "on" else []) + ["valid"]


def inputs(p):
    """Input ports in order: ['inc',] ['off',] 'valid'."""
    # the block's port labels: resetAccum before valid; with an increment port inc, resetAccum,
    # valid and inc, offset, valid
    return (["inc"] if p["PhaseIncrementSource"] == "Input port" else []) + \
        (["rst"] if p.get("ResetAction") == "on" else []) + \
        (["off"] if p.get("PhaseOffsetSource") == "Input port" else []) + ["valid"]


def phase_type(p):
    """the phase output: the quantised accumulator, unsigned, NumQuantizerAccumulatorBits wide (ufix12 recorded)"""
    return (0, int(p["NumQuantizerAccumulatorBits"]), 0)


class Kernel:
    """One real output of the NCO: sel in 'sin', 'cos', 'valid'."""

    def __init__(self, p, a, q, inc, sel, ts, where):
        # the domains signed before recordings first; a design they refuse (UNSUPPORTED) is tried against the recordings
        # domain (_init_116), so nothing they answer changes
        from fxgraph_lib import GraphError
        try:
            self._init_old(p, a, q, inc, sel, ts, where)
        except GraphError as e:
            if e.code != "UNSUPPORTED" or not self._init_116(p, a, q, inc, sel, ts, where):
                raise
            return
        if not self.gaps_ok and inc is None and isinstance(ts[0], tuple) and \
                any(tg in L116_GAPS for tg in _l116_port_tags(p, a, q, ts[0], self.wl, self.fl)):
            # answered before with valid always 1 only (e.g. recordings's ufix(A) ports):
            # show this setting and port with valid gaps
            self.gaps_ok = True

    def _init_116(self, p, a, q, inc, sel, ts, where):
        """recordings (l116_tags), and the single-feature configurations: the recordings one-feature increment-port rule also for the port tuples of
        INC_PORTS (the old kernel left them to the narrower INC_PORT_SETTINGS). True when the design is accepted."""
        from fxgraph_lib import BOOL
        _, wl, fl = out_type(p)
        tags = l116_tags(p, a, q, wl, fl)
        gaps = True
        if inc is None:
            t = ts[0]
            if isinstance(t, tuple) and t[2] > 0:
                # an increment port with fraction bits stops Simulink in
                # validateInputsImpl
                _refuse(where, "phase increment port of type %s: the increment must be an integer type (fraction "
                        "length 0); Simulink rejects it in validateInputsImpl (SystemBlock:MATLABSystem:"
                        "MethodInvokeErrorWithStack)" % (t,), "PORT_TYPE_INVALID")
            ok = _l116_port_tags(p, a, q, t, wl, fl)
            if not ok:
                return False
            gaps = any(tg in L116_GAPS for tg in ok)
        elif not tags:
            return False
        if ts[-1] != BOOL:
            _refuse(where, "the valid input must be boolean", "TYPE_MISMATCH")
        self.a, self.q, self.inc, self.sel, self.where = a, q, inc, sel, where
        self.off, self.wl, self.fl = 0, wl, fl
        self.lutc = p.get("LUTCompress") == "on"
        self.ix = {nm: i for i, nm in enumerate(inputs(p))}
        self.l63 = False
        self.gaps_ok = gaps
        self.offport = self.rstport = False
        self.dither = int(p["NumDitherBits"]) if p.get("DitherSource") == "Property" else 0
        self.reset()
        return True

    def _init_old(self, p, a, q, inc, sel, ts, where):
        from fxgraph_lib import BOOL
        self.a, self.q, self.inc, self.sel, self.where = a, q, inc, sel, where
        try:   # recordings: '(0)' / '0*1' ended in INTERNAL (int())
            self.off = int(str(p["PhaseOffset"]).strip())
        except ValueError:
            _refuse(where + ".PhaseOffset", "PhaseOffset %r: an integer literal is required (MATLAB expressions are not "
                    "evaluated)" % p["PhaseOffset"], "UNSUPPORTED")
        _, self.wl, self.fl = out_type(p)
        self.lutc = p.get("LUTCompress") == "on"
        if ts[-1] != BOOL:
            _refuse(where, "the valid input must be boolean (from a design input: through a logic block, e.g. NOT(NOT x))", "TYPE_MISMATCH")
        names = inputs(p)
        self.ix = {nm: i for i, nm in enumerate(names)}
        self.l63 = l63_setting(p) and not (a in ACC_WLS and q in QUANT_BITS and inc is None
                                           and (ts[0], a, q) in INC_PORTS)
        # increment ports ufix(AccumulatorWL), with valid always 1 (no gaps)
        nc_inc = inc is None and nc_setting(p, a, q, self.wl, self.fl) and ts[0] == (0, a, 0)
        if inc is None and (ts[0], a, q) not in INC_PORTS and not (self.l63 and l63_inc_ok(ts[0], a)) and not nc_inc:
            _refuse(where, "phase increment port of type %s with AccumulatorWL %d, NumQuantizerAccumulatorBits %d: "
                    "recorded %s" % (ts[0], a, q, "; ".join("%s %d/%d" % k for k in sorted(INC_PORTS, key=str))))
        if self.lutc and inc is None and ts[0] != (0, a, 0):
            # LUTCompress on with an increment port: recorded with ufix(AccumulatorWL) ports only
            #
            _refuse(where + ".LUTCompress", "LUTCompress 'on' with a phase increment port of type %s: recorded with "
                    "ufix%d (the accumulator's width) only"
                    % (ts[0], a))
        # gaps with an increment port were recorded with Sine and cosine at the default output only
        self.gaps_ok = inc is not None or ((ts[0], a, q) in INC_PORT_GAPS and p["Waveform"] == "Sine and cosine"
                                           and (p["OutputWL"], p["OutputFL"]) == ("16", "14")
                                           and p.get("PhasePort", "off") == "off")
        if inc is None and self.l63:
            self.gaps_ok = True   # increment ports with valid gaps recorded on every type
        elif inc is None and not nc_inc:
            # every increment-port recording has these settings; the waveform per INC_PORT_WAVES
            fixed = {"PhaseOffset": "0", "SimulateUsing": "Code generation"}
            odd = sorted(k for k, v in fixed.items() if str(p.get(k, DEFAULTS[k])).strip() != v)
            if odd:
                _refuse(where + "." + odd[0], "%s=%r with a phase increment port: recorded %r only" % (
                    odd[0], p.get(odd[0]), fixed[odd[0]]))
            got = (p["Waveform"], int(p["OutputWL"]), int(p["OutputFL"]), p.get("PhasePort", "off"))
            sets = INC_PORT_SETTINGS_ALL | INC_PORT_SETTINGS.get((ts[0], a, q), set())
            if got not in sets:
                _refuse(where + ".Waveform", "Waveform %r, output fixdt(1,%d,%d), PhasePort %s with this phase "
                        "increment port: recorded %s" % (got + ("; ".join("%s %d/%d phase %s" % s for s in
                                                                         sorted(sets)),)))
        self.offport = p.get("PhaseOffsetSource") == "Input port"
        self.rstport = p.get("ResetAction") == "on"
        self.dither = int(p["NumDitherBits"]) if p.get("DitherSource") == "Property" else 0
        if self.rstport and ts[self.ix["rst"]] != BOOL:
            _refuse(where, "the reset input must be boolean", "TYPE_MISMATCH")
        if self.offport and ts[self.ix["off"]] != (1, 16, 0) and not (self.l63 and ts[self.ix["off"]] == (0, a, 0)):
            _refuse(where, "phase offset port of type %s: recorded int16 (16-bit accumulator) and ufix%d (the "
                    "accumulator's width) only" % (ts[self.ix["off"]], a))
        if inc is None and (self.rstport or self.offport or self.dither):
            # each together with an int16 increment port into 16 bits, quantiser 12, the defaults
            got = (ts[0], a, q, p["Waveform"], int(p["OutputWL"]), int(p["OutputFL"]), p.get("PhasePort", "off"))
            if got != ((1, 16, 0), 16, 12, "Sine and cosine", 16, 14, "off") or \
                    (self.dither and self.dither != 4) or (self.rstport + bool(self.offport) + bool(self.dither)) > 1:
                _refuse(where, "a reset port, a phase offset port or dither together with a phase increment port "
                        "was recorded one at a time, with an int16 increment into a 16-bit accumulator, quantiser 12, "
                        "Sine and cosine, output 16/14, 4 dither bits")
            self.gaps_ok = False   # show these with valid always high only (review of recordings)
        if inc is not None and self.dither and a == 20 and not self.l63:
            # one setting (review of recordings)
            got20 = (p["PhaseIncrement"].strip(), p["PhaseOffset"].strip(), p.get("PhasePort", "off"), p["Waveform"],
                     int(p["OutputWL"]), int(p["OutputFL"]))
            if got20 != ("100", "0", "on", "Sine and cosine", 16, 14):
                _refuse(where + ".AccumulatorWL", "dither with a 20-bit accumulator was recorded with PhaseIncrement "
                        "100, PhaseOffset 0, the phase port, Sine and cosine, output 16/14 only")
            self.gaps_ok = False
        self.reset()

    def reset(self):
        self.t = 0
        self.acc = 0
        self.pn = _Pn()
        self.hist = []   # phases waiting for their output step

    def value(self, acc):
        ph = ((acc + self.off) % (1 << self.a)) >> (self.a - self.q)
        if self.sel == "phase":
            return ph
        return lut_value(ph, self.q, self.sel, self.wl, self.fl, self.lutc)

    def __call__(self, v):
        """recordings: the accumulator advances only on a valid input; each valid input gives its sample
        LATENCY steps later (valid high then), data 0 otherwise. With the increment port, a valid that is not always
        high was not recorded."""
        t = self.t
        self.t += 1
        if not v[-1] and not self.gaps_ok:
            _refuse(self.where, "valid is 0 at step %d: valid gaps with the phase increment port were not recorded" % t)
        rst = v[self.ix["rst"]] if self.rstport else 0
        if self.rstport and rst and not v[-1]:
            # a reset while valid is low still restarts the accumulator
            # (ignoring it differs from step 29)
            self.acc = 0
        if v[-1]:
            # the phase offset port adds its value at the step the sample is taken; the dither of
            # this sample is added the same way
            extra = (v[self.ix["off"]] - self.off if self.offport else 0) + \
                (self.pn.take(self.dither) if self.dither else 0)
            self.hist.append((t + LATENCY, self.acc + extra))
            self.acc = (self.acc + (self.inc if self.inc is not None else v[self.ix["inc"]])) % (1 << self.a)
            if self.rstport and rst:
                # the sample of the reset step still uses the old phase; the accumulator restarts from 0 after it
                # (reset at step 20, the sine is 0 again at step 27 = 21 + the latency)
                self.acc = 0
        if self.hist and self.hist[0][0] == t:
            acc = self.hist.pop(0)[1]
            return 1 if self.sel == "valid" else self.value(acc)
        return 0

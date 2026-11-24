"""DSP HDL Toolbox Discrete FIR Filter and FIR Decimator (dsphdlfiltering2), with configuration-specific pipeline timing. Python path only: an exact kernel, the C core / fxcodegen / Verilog
refuse (vm_ok False).

Blocks (JSON design): HDLFIRFilter and HDLFIRDecimator, inputs (data, valid), outputs (data, valid), parameters as in
the dialog. Each is lowered by engine/fxhdl.py into two internal HDLFIRCore blocks (Port 1 data, Port 2 valid) that
run the same kernel.

Behaviour (all 127 records):
- coefficients: the input's word length, best precision; the given values must be exact in that type
- output type (Inherit via internal rule): signed, fraction length = input fl + coefficient fl, integer bits the fewest
  that hold the exact range of sum(c_k x_k) over the input type's range
- value: the full-precision convolution of the valid input samples; the decimator keeps the samples D-1, 2D-1, ...
- the output (with valid high) appears a fixed number of steps after the sample it belongs to; data is 0 while
  valid is low. The latency: Direct form systolic N + 6 (asymmetric) / ceil(N/2) + 7 (symmetric), Direct form
  transposed 6, Partly serial systolic and the decimator from the recorded tables.

Inputs: signed 12 / 16 / 18 / 24 bits on every structure, signed 14 / 20 bits on Direct form
systolic, unsigned ufix16_En16 / uint8 on Direct form systolic and transposed (signed full-precision output), these
exact types only, with the recorded coefficients, every other parameter at its default and valid always high.

Refused (not recorded): other parameter values, other input types, antisymmetric coefficients treated as
anything but asymmetric (refused), tap counts outside the recorded ones for the table-driven structures, and at run
time a valid pattern other than the recorded one (always high; every NumCycles steps from step 0 for partly serial;
gaps only where recordings show them, see GAPS)."""
import math
import re
from fractions import Fraction

# structures where a set antisymmetric after
# quantisation was recorded -- the symmetric latency, valid always 1
ANTI_RECORDED = {("Direct form systolic", None, 4), ("Direct form systolic", None, 6),
                 ("Partly serial systolic", "2", 4), ("decim", "2", 8)}
INPUT_WLS = (12, 16, 18, 24)
# further input types, each recorded with exactly this type, these structures, the coefficients
# below, every other parameter at its default and valid always high (the output: signed, full precision)
# more coefficient sets on Direct form systolic for the unsigned inputs, and valid gaps for two of the
# configurations. Keyed (input type, structure) -> the recorded coefficient sets
_C26 = ("0.1", "-0.2", "0.35", "0.35", "-0.2", "0.1")
_C27_U8 = (("0.1", "0.2", "0.2", "0.1"), ("0.4",) * 6, ("0.9", "-0.7", "0.6", "0.6", "-0.7", "0.9"),
           ("-0.1", "-0.2", "-0.2", "-0.1"))
NEW_INPUTS = {((1, 14, 13), "Direct form systolic"): (_C26,), ((1, 20, 19), "Direct form systolic"): (_C26,),
              ((0, 16, 16), "Direct form systolic"): (_C26, ("0.1", "0.2", "0.2", "0.1")),
              ((0, 16, 16), "Direct form transposed"): (_C26,),
              ((0, 8, 0), "Direct form systolic"): (_C26,) + _C27_U8, ((0, 8, 0), "Direct form transposed"): (_C26,)}
# the partly serial reset: NumCycles -> the recorded coefficient sets; after a reset the
# first len(c) valid outputs are 0 (4, 6, 8 taps recorded, whatever NumCycles)
SERIAL_RESET = {"2": (_C26, ("0.05", "0.1", "-0.15", "0.3", "0.25", "-0.1", "0.08", "0.02"), ("0.1", "0.4", "0.4", "0.1")),
                "4": (_C26,)}
NEW_INPUTS_GAPS = {((1, 14, 13), "Direct form systolic", _C26), ((0, 8, 0), "Direct form systolic", _C26)}

FIR_DEFAULTS = {"NumeratorSource": "Property", "Numerator": None, "NumeratorPrototype": "[]",
                "FilterStructure": "Direct form systolic",
                "SerializationOption": "Minimum number of cycles between valid input samples", "NumCycles": "2",
                "NumberOfMultipliers": "2", "roundingMode": "Floor", "overflowMode": "off",
                "CoefficientsDataTypeStr": "Inherit: Same word length as input",
                "OutputDataTypeStr": "Inherit: Inherit via internal rule", "LockScale": "off", "ResetInputPort": "off",
                "HDLGlobalReset": "off", "CoeffMultiplier": "Multiplier", "CoeffMultiplierSystolic": "Multiplier",
                "UseMultForPow2Coeff": "on", "FrameInputSymmOptim": "on", "SimulateUsing": "Code generation"}
DEC_DEFAULTS = {"NumeratorSource": "Property", "Numerator": None, "NumeratorPrototype": "[]",
                "FilterStructure": "Direct form systolic", "DecimationFactor": "2", "NumCycles": "1",
                "OptimizeSymmCoeff": "on", "roundingMode": "Floor", "overflowMode": "off",
                "CoefficientsDataTypeStr": "Inherit: Same word length as input",
                "OutputDataTypeStr": "Inherit: Inherit via internal rule", "LockScale": "off", "ResetInputPort": "off",
                "HDLGlobalReset": "off", "SimulateUsing": "Code generation"}
# values other than the default that were recorded (anything else is refused)
def _fixdt(txt):
    """(1, w, f) for 'fixdt(1,w,f)' (signed binary-point scaling, the recorded form), else None"""
    m = re.match(r"^fixdt\(\s*1\s*,\s*(\d+)\s*,\s*(-?\d+)\s*\)$", txt or "")
    return (1, int(m.group(1)), int(m.group(2))) if m and 2 <= int(m.group(1)) <= 64 else None


FIR_ALLOWED = {"FilterStructure": ("Direct form systolic", "Direct form transposed", "Partly serial systolic"),
               "NumCycles": None, "SimulateUsing": ("Code generation", "Interpreted execution"),
               # recordings (Direct form systolic with an explicit output type)
               "roundingMode": ("Nearest", "Convergent", "Zero"), "overflowMode": ("on",), "ResetInputPort": ("on",)}
DEC_ALLOWED = {"DecimationFactor": ("2", "3", "4", "8"), "SimulateUsing": ("Code generation", "Interpreted execution"),
               "roundingMode": ("Nearest",),   # Nearest with an explicit output type
               "overflowMode": ("on",),        # saturation, factor 2 with an explicit type
               "ResetInputPort": ("on",),      # factor 2, the 6-tap set, defaults
               "NumCycles": None}              # filled from DEC_NC below
NS = (4, 5, 8, 9, 12, 16, 24, 32)
# latency (steps from the input sample to its output); partly serial: {NumCycles: {(N, symmetric): latency}}
PS = {2: dict(zip([(n, s) for n in NS for s in (False, True)],
                  [11, 9, 12, 12, 13, 12, 14, 13, 15, 13, 17, 14, 21, 16, 25, 18])),
      4: dict(zip([(n, s) for n in NS for s in (False, True)],
                  [10, 9, 12, 10, 13, 11, 13, 14, 14, 13, 15, 14, 17, 15, 19, 16]))}
PS[2][(6, True)] = 12   # recordings: 6 symmetric taps, valid every other step
# NumCycles 3 and 8 for 6 symmetric, 8 asymmetric and 16 symmetric taps, NumCycles 4 for
# 6 symmetric taps -- each recorded with valid exactly every NumCycles steps from step 0 only (PS_EXACT)
PS[3] = {(6, True): 10, (8, False): 13, (16, True): 14}
PS[8] = {(6, True): 10, (8, False): 14, (16, True): 15}
PS[4][(6, True)] = 10
PS_EXACT = {(3, 6, True), (3, 8, False), (3, 16, True), (8, 6, True), (8, 8, False), (8, 16, True), (4, 6, True)}
# recordings (the NumCycles sweep): NumCycles 2 .. 19 (also equal to and above the tap count) on input
# sfix16_En15 with every other parameter at its library default, valid every NumCycles steps from step 0 -- each
# (NumCycles, taps, symmetric) one recorded latency. Where the record had a sparse or irregular valid (spacing >=
# NumCycles) the latency held for every sample: those keys take any spacing >= NumCycles (L54_SPACED), the others
# the exact rhythm only (PS_EXACT). The NumCycles 2 / 4 entries it re-recorded agreed with the tables above.
L54 = {3: {(5, False): 12, (12, True): 13, (32, False): 21, (12, False): 14},
       5: {(9, False): 14, (24, True): 15, (5, False): 11},
       6: {(12, True): 13, (32, False): 19, (8, True): 11},
       7: {(16, False): 16, (4, True): 9, (9, False): 14, (4, False): 10},
       8: {(24, True): 16, (5, False): 11, (12, True): 13, (8, True): 11},
       9: {(9, True): 12}, 11: {(8, False): 14},
       12: {(32, False): 21, (8, True): 11, (16, False): 17, (9, False): 15},
       16: {(4, True): 9, (9, False): 15, (24, True): 19, (16, True): 15}, 19: {(16, False): 22}}
# the same model on 17 of the keys above with 10
# coefficient sets each; the latency and the ready pattern depend only on (NumCycles, taps, symmetric), never on the
# values, and the 90 cases on the keys above agreed with them. The sets of the opposite symmetry give these 16 keys
# (4 to 6 sets each, all the same value; exact valid rhythm only)
L70 = {3: {(32, True): 17}, 5: {(24, False): 17, (9, True): 12}, 6: {(12, False): 15, (32, True): 17},
       7: {(16, True): 14, (9, True): 12}, 8: {(12, False): 15, (24, False): 18}, 9: {(9, False): 15},
       11: {(8, True): 11}, 12: {(16, True): 15, (32, True): 18}, 16: {(24, False): 21, (9, True): 12},
       19: {(16, True): 15}}
for _nc, _tab in L70.items():
    assert not set(_tab) & set(L54.get(_nc, {}))
    L54.setdefault(_nc, {}).update(_tab)
L54_SPACED = {(3, 12, False), (3, 12, True)}
L54_KEYS = {(nc, n, s) for nc, tab in L54.items() for (n, s) in tab}
for _nc, _tab in L54.items():
    PS.setdefault(_nc, {}).update(_tab)
PS_EXACT |= L54_KEYS - L54_SPACED
# Ready (third output) is low for min(NumCycles, taps) - 1 steps after each valid sample.
# valid always high -- with N = min(NumCycles, taps) and P = 2N, ready is 1 only at steps
# 0, P, 2P, ...; the inputs of steps jP and jP + 1 are taken, the others dropped; the first of the pair comes out the
# table latency later, the second N steps after it. Recorded on input sfix16_En15, inherited coefficient type:
PS_ALWAYS = {(2, 6, True), (2, 8, False), (3, 6, True), (3, 8, False), (4, 6, True), (4, 8, False), (8, 6, True),
             (8, 8, False)}
# decimator: from the D-th input sample (the first kept convolution sample) to the first valid output
DEC_NS = (8, 12, 16, 24, 32)
DEC = {2: dict(list(zip([(n, s) for n in DEC_NS for s in (False, True)], [15, 13, 19, 16, 23, 16, 31, 17, 39, 18])) +
              [((6, True), 13)]),   # recordings: 6 symmetric taps, first output step 14
       3: dict(zip([(n, s) for n in DEC_NS for s in (False, True)], [15, 15, 18, 14, 24, 15, 30, 17, 39, 19])),
       4: dict(zip([(n, s) for n in DEC_NS for s in (False, True)], [13, 13, 17, 17, 21, 21, 29, 15, 37, 16]))}
# factor 8 with 16 symmetric / 32 asymmetric taps; 6 symmetric taps on factors 3 and 4
DEC[8] = {(16, True): 17, (32, False): 33}
DEC[8][(8, True)] = 9   # recordings: 8 symmetric taps, valid always high
# the decimator with NumCycles > 1 -- {(factor, NumCycles): {(taps, symmetric): latency from the kept input
# sample to its output}}, valid every NumCycles steps from step 0, input sfix16_En15, every other parameter at its
# default (one record each)
DEC_NC = {(2, 2): {(32, True): 18}, (2, 5): {(8, True): 14},
          (3, 2): {(8, True): 14}, (3, 3): {(16, True): 17}, (3, 6): {(24, True): 19},
          (4, 2): {(16, True): 15}, (4, 4): {(24, True): 17}, (4, 7): {(32, True): 19},
          (8, 2): {(16, True): 14}, (8, 8): {(24, True): 15}, (8, 11): {(32, True): 16}}
# FIR Decimator with NumCycles > 1 when every polyphase branch
# (taps k, k+D, ...) reads the same forwards and backwards -- the first valid output moves by a fixed step count per
# (factor, NumCycles, taps), the same for one palindrome in every branch, distinct palindromes, sign-alternated and
# proportional branches (60 sets, 11 keys); the offset is added to the symmetric DEC_NC latency. (8, 2, 16) moved by
# +75: not answered
DEC_NC_PAL = {(2, 2, 32): 0, (2, 5, 8): -1, (3, 2, 8): 0, (3, 3, 16): 0, (3, 6, 24): -3, (4, 2, 16): -1,
              (4, 4, 24): -2, (4, 7, 32): -3, (8, 8, 24): 0, (8, 11, 32): -1}
DEC[3][(6, True)] = 12
DEC[4][(6, True)] = 13
FIR_ALLOWED["NumCycles"] = tuple(str(k) for k in sorted(PS))
DEC_ALLOWED["NumCycles"] = tuple(str(k) for k in sorted({nc for (_d, nc) in DEC_NC}))
# with gaps in valid (the only gap records): the output of the kept sample with valid index
# i appears b steps after the input with valid index i + a (a + b = the latency above). Fitted per configuration
# (one fit each): transposed (0, 6) for every tap count (its latency does not depend on the taps); the others only
# for the recorded configuration (16 symmetric taps); elsewhere a valid with gaps is refused at run time.
GAPS = {("decim", 2, 16, True): (0, 16), ("decim", 4, 16, True): (16, 5),        
        ("decim", 2, 8, True): (0, 13), ("decim", 2, 24, True): (0, 17),         # 
        ("decim", 4, 8, True): (8, 5), ("decim", 4, 24, True): (0, 15)}
_NUM = re.compile(r"^[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?$")


def _refuse(where, msg, code="UNSUPPORTED"):
    from fxgraph_lib import GraphError
    raise GraphError(code, where, msg)


def params(bt, b, where):
    """Checked parameters of a user block (HDLFIRFilter / HDLFIRDecimator)."""
    dflt, allowed = (FIR_DEFAULTS, FIR_ALLOWED) if bt == "HDLFIRFilter" else (DEC_DEFAULTS, DEC_ALLOWED)
    p = {}
    for k, v in b.items():
        if k == "type":
            continue
        if k not in dflt:
            _refuse(where + "." + k, "unknown parameter %s for %s" % (k, bt), "BAD_PARAM")
        if not isinstance(v, str):
            _refuse(where + "." + k, "text value required", "BAD_PARAM")
        v = v.strip()
        ok = (dflt[k] is None) or v == dflt[k] or v in allowed.get(k, ())
        if k in ("CoefficientsDataTypeStr", "OutputDataTypeStr") and _fixdt(v):
            ok = True   # an explicit signed binary-point type
        if not ok:
            _refuse(where + "." + k, "%s=%r is not supported (recorded: %s)" % (
                k, v, ", ".join(repr(x) for x in (dflt[k],) + tuple(x for x in allowed.get(k, ()) if x != dflt[k]))))
        p[k] = v
    if "Numerator" not in p:
        _refuse(where + ".Numerator", "the coefficients are required", "MISSING_FIELD")
    for k, v in dflt.items():
        p.setdefault(k, v)
    return p


def coefficients(p, where):
    txt = p["Numerator"].strip()
    if not (txt.startswith("[") and txt.endswith("]")):
        _refuse(where + ".Numerator", "a literal vector [c0 c1 ...] is required", "BAD_PARAM")
    items = [x for x in re.split(r"[\s,]+", txt[1:-1].strip()) if x]
    if not all(_NUM.match(x) for x in items):
        _refuse(where + ".Numerator", "only numeric literals are supported", "BAD_PARAM")
    if not all(math.isfinite(float(x)) for x in items):
        _refuse(where + ".Numerator", "a coefficient outside the double range", "BAD_PARAM")
    return [dbl(x) for x in items]


def dbl(x):
    """Simulink quantises the double a literal denotes, not its
    decimal text -- e.g. -5.960464477539063e-08 is -2^-24 exactly as a double (a tie that rounds half up to 0), and a
    17-digit literal just below a tie can be the tie itself as a double. Exact value of the nearest double."""
    return Fraction(float(x))


class Plan:
    def __init__(self, bt, p, ts, where):
        x, v = ts[0], ts[1]
        from fxgraph_lib import BOOL
        self.reset = p.get("ResetInputPort", "off") == "on"
        if self.reset:
            # Direct form systolic; recordings: Direct form transposed (valid always high) and
            # Partly serial systolic with NumCycles 2 (the recorded 6 symmetric taps, full precision, valid every 2
            # steps from step 0)
            st_r = p["FilterStructure"]
            # show transposed and serial resets with one setting each: these coefficients, input
            # sfix16_En15, inherited coefficient and output types (review of recordings)
            cs_r = coefficients(p, where)
            plain = (tuple(x) == (1, 16, 15) and _fixdt(p.get("OutputDataTypeStr", "")) is None
                     and p.get("CoefficientsDataTypeStr", FIR_DEFAULTS["CoefficientsDataTypeStr"])
                     == FIR_DEFAULTS["CoefficientsDataTypeStr"]
                     and p.get("roundingMode", "Floor") == "Floor" and p.get("overflowMode", "off") == "off")
            serial_sets = [[dbl(x_) for x_ in s] for s in SERIAL_RESET.get(p.get("NumCycles"), ())]
            ok_r = (bt == "HDLFIRDecimator" and p.get("DecimationFactor") == "2" and plain
                    and cs_r == [dbl(x_) for x_ in _C26]) or bt == "HDLFIRFilter" and (
                st_r == "Direct form systolic" or
                (st_r == "Direct form transposed" and plain and cs_r == [dbl(x_) for x_ in _C26]) or
                (st_r == "Partly serial systolic" and plain and cs_r in serial_sets))
            if not ok_r:
                _refuse(where + ".ResetInputPort", "the reset input port was recorded for Direct form systolic, for "
                        "Direct form transposed with the coefficients [%s], and for Partly serial systolic with %s; "
                        "each with input sfix16_En15 and the inherited types only" % (
                            " ".join(_C26), "; ".join("NumCycles %s [%s]" % (k, " ".join(s))
                                                      for k, v in sorted(SERIAL_RESET.items()) for s in v)))
            if len(ts) < 3 or ts[2] != BOOL:
                _refuse(where, "the reset input must be boolean", "TYPE_MISMATCH")
        if v != BOOL:
            _refuse(where, "the valid input must be boolean (from a design input: through a logic block, e.g. NOT(NOT x))", "TYPE_MISMATCH")
        # the recorded set of input word lengths, not a range
        if x == BOOL or not (x[0] and x[1] in INPUT_WLS):
            # signed 14 / 20 bits on Direct form systolic, unsigned 8 / 16 bits on Direct form
            # systolic and transposed -- each with every other parameter at its library default
            st0 = p["FilterStructure"]
            ok_in = x != BOOL and bt == "HDLFIRFilter" and (tuple(x), st0) in NEW_INPUTS
            if ok_in:
                sets = NEW_INPUTS[(tuple(x), st0)]
                cs = coefficients(p, where)
                hit = [s for s in sets if [dbl(v) for v in s] == cs]
                if not hit:
                    _refuse(where + ".Numerator", 'input type %s on %s was recorded with the coefficients %s only'
                            "" % (x, st0, " or ".join("[%s]" % " ".join(s) for s in sets)))
                self.new_gaps = (tuple(x), st0, hit[0]) in NEW_INPUTS_GAPS
                odd = sorted(k for k, dv in FIR_DEFAULTS.items() if dv is not None and k not in ("FilterStructure",)
                             and p.get(k, dv) != dv)
                if odd:
                    _refuse(where + "." + odd[0], 'input type %s was recorded with %s at its library default only'
                            "" % (x, odd[0]))
            else:
                _refuse(where, "input type %s: recorded signed %s bits (every structure), and %s with the "
                        "structures given" % (
                            x, ", ".join(map(str, INPUT_WLS)),
                            "; ".join(sorted({"%s %s" % k for k in NEW_INPUTS}))))
        self.new_input = x != BOOL and not (x[0] and x[1] in INPUT_WLS)
        c = coefficients(p, where)
        n = len(c)
        if n < 2 or all(ci == 0 for ci in c):
            _refuse(where + ".Numerator", "at least two coefficients, not all zero", "BAD_PARAM")
        wl = x[1]
        # coefficients: the input's word length, best precision, rounded to nearest (half up;
        # Floor / Fix / Convergent excluded by the records; a negative exact half, where Nearest and Round differ, is
        # refused)
        lo_, hi_ = -(1 << (wl - 1)), (1 << (wl - 1)) - 1

        def rq(v_):
            return (v_ + Fraction(1, 2)).numerator // (v_ + Fraction(1, 2)).denominator

        cfl = next(fl for fl in range(200, -200, -1) if all(lo_ <= rq(ci * (Fraction(2) ** fl)) <= hi_ for ci in c))
        exact_fl = next(fl for fl in range(200, -200, -1)
                        if all(lo_ <= ci * (Fraction(2) ** fl) <= hi_ for ci in c))
        if exact_fl != cfl:
            _refuse(where + ".Numerator", "a coefficient rounds across the range of %d bits (best precision is not "
                    "determined by the records)" % wl)
        ct = _fixdt(p.get("CoefficientsDataTypeStr", ""))
        if ct is not None and (ct != (1, 16, 15) or tuple(x) != (1, 16, 15)):
            # recordings (swap probe): an explicit coefficient type was recorded as fixdt(1,16,15) on input
            # sfix16_En15 only
            _refuse(where + ".CoefficientsDataTypeStr", "an explicit coefficient type was recorded as fixdt(1,16,15) "
                    "with input sfix16_En15 only")
        if ct is not None:   # the given type, rounded to nearest like the inherited one
            cfl, lo_, hi_ = ct[2], -(1 << (ct[1] - 1)), (1 << (ct[1] - 1)) - 1
            if not all(lo_ <= rq(ci * (Fraction(2) ** cfl)) <= hi_ for ci in c):
                _refuse(where + ".CoefficientsDataTypeStr", "a coefficient does not fit %s (saturation of "
                        "coefficients was not recorded)" % p["CoefficientsDataTypeStr"])
        q = [ci * (Fraction(2) ** cfl) for ci in c]
        # recordings (Partly serial systolic, input sfix16_En15):
        # -2^-16 at fraction length 15 is a negative exact half; Simulink's output matched rounding it half up to 0
        # (Nearest), not away from zero to -1 (Round, one step differs). recordings: Direct form systolic too
        # (recordings equal with
        # Nearest; Round away differs on 108 of 120 steps). recordings: Direct form
        # transposed too (round away differs from step 7); the other structures stay undecided
        if any(v_ < 0 and v_.denominator == 2 for v_ in q) and not (
                bt == "HDLFIRFilter" and p.get("FilterStructure") in ("Partly serial systolic", "Direct form systolic",
                                                              "Direct form transposed")):
            _refuse(where + ".Numerator", "a negative coefficient exactly half-way at fraction length %d: Nearest and "
                    "Round differ there and the records do not decide" % cfl)
        self.c = [rq(v_) for v_ in q]
        if all(ci == 0 for ci in self.c):   # review of recordings: the output type of an all-zero filter is unrecorded
            _refuse(where + ".Numerator", "every coefficient rounds to 0 in its type (not recorded)")
        # Simulink decides symmetry on the quantised
        # coefficients (self.c), not on the literals -- [0.1 0.2 0.2 0.1000001] quantises symmetric and takes the
        # symmetric latency (it took the asymmetric one: valid 1-2 steps late, silently)
        qc = self.c
        sym = qc == qc[::-1]
        self.anti = not sym and qc == [-v_ for v_ in qc[::-1]]
        if not sym and not self.anti and len(qc) > 2 and len(qc) % 2 and qc[1:] == qc[1:][::-1] and \
                bt == "HDLFIRFilter" and str(p.get("FilterStructure", "")).strip() == "Direct form systolic":
            # recordings (silent wrong): 7 taps [a b c d d c b] -- palindromic after the first tap -- came
            # out at Simulink's symmetric latency (step 11 = ceil(7/2) + 7), the engine took the asymmetric one (13).
            # Which sets Simulink treats as symmetric beyond a true palindrome is not recorded: this class (odd tap
            # count, Direct form systolic -- the recorded partly serial one-hot / alternating sets of this kind
            # took the asymmetric latency as the engine does) is refused
            _refuse(where + ".Numerator", "coefficients that read the same both ways except the first tap (after "
                    "quantisation): Simulink took the symmetric latency for one such set, the rule "
                    "it uses for them is not recorded; make the set symmetric or clearly asymmetric")
        if self.anti:
            # recordings (ii): a set antisymmetric after quantisation takes the symmetric latency, recorded (valid
            # always 1) for Direct form systolic 4 / 6 taps, Partly serial NumCycles 2 with 4 taps and the FIR
            # Decimator by 2 with 8 taps; elsewhere refused
            key_ = ("decim", p.get("DecimationFactor"), len(qc)) if bt == "HDLFIRDecimator" else (
                p.get("FilterStructure"), p.get("NumCycles") if p.get("FilterStructure") == "Partly serial systolic"
                else None, len(qc))
            if key_ not in ANTI_RECORDED:
                _refuse(where + ".Numerator", "antisymmetric coefficients (after quantisation): the latency was "
                        "recorded for %s only" % "; ".join("%s %s %d taps" % (a, b or "", n_) for a, b, n_ in
                                                           sorted(ANTI_RECORDED, key=str)))
            sym = True
        fl = x[2] + cfl
        xmin, xmax = (-(1 << (wl - 1)), (1 << (wl - 1)) - 1) if x[0] else (0, (1 << wl) - 1)
        ymax = sum(max(ci * xmin, ci * xmax) for ci in self.c)
        ymin = sum(min(ci * xmin, ci * xmax) for ci in self.c)
        k = 1   # total bits including sign, smallest that holds [ymin, ymax] as stored integers
        while not (-(1 << (k - 1)) <= ymin and ymax <= (1 << (k - 1)) - 1):
            k += 1
        self.otype = (1, k, fl)
        if ymin >= 0 and not x[0]:
            # every sum non-negative (an unsigned input, non-negative coefficients) -- an unsigned
            # output of the fewest bits holding ymax (uint8 [0.1 0.2 0.2 0.1]: ufix17_En9)
            k = 1
            while ymax > (1 << k) - 1:
                k += 1
            self.otype = (0, k, fl)
        self.cast = None
        ot = _fixdt(p.get("OutputDataTypeStr", ""))
        rm, ov = p.get("roundingMode", "Floor"), p.get("overflowMode", "off")
        if ot is None and (rm != "Floor" or ov != "off") and not (bt == "HDLFIRDecimator" and ov == "off"
                                                                 and p.get("DecimationFactor") == "2"):
            # (the FIR Decimator's full-precision output with Nearest was recorded equal to the default)
            _refuse(where + ".roundingMode", "roundingMode / overflowMode with the inherited (full-precision) output: "
                    "nothing is rounded, recorded only with an explicit output type")
        if ot is not None and tuple(x) != (1, 16, 15):
            # recordings (swap probe): every explicit output type was recorded on input sfix16_En15
            _refuse(where + ".OutputDataTypeStr", 'an explicit output type was recorded with input sfix16_En15 only'
                    "")
        if ot is not None:
            # recordings (Direct form systolic) and recordings (Direct form transposed, Partly serial
            # systolic with NumCycles 2, the FIR Decimator with factor 2 -- without saturation)
            ok_st = (bt == "HDLFIRFilter" and (p["FilterStructure"] in ("Direct form systolic", "Direct form transposed")
                                               or (p["FilterStructure"] == "Partly serial systolic"
                                                   and p.get("NumCycles") in ("2", "4")))) or \
                (bt == "HDLFIRDecimator" and p.get("DecimationFactor") in ("2", "3", "4") and
                 (ov != "on" or p.get("DecimationFactor") == "2"))   # factors 3 / 4, saturation on 2
            if not ok_st:
                _refuse(where + ".OutputDataTypeStr", "an explicit output type was recorded for Direct form systolic "
                        "and transposed, Partly serial systolic with NumCycles 2 and 4, and the FIR Decimator by 2, 3 "
                        "and 4 (saturation by 2 only)")
            if bt == "HDLFIRFilter" and p["FilterStructure"] in ("Direct form transposed", "Partly serial systolic") \
                    and (rm, ov) not in (("Floor", "off"), ("Nearest", "off"), ("Floor", "on")):
                # an explicit output type on these structures was recorded with Floor, Nearest and
                # Floor with saturation only; Convergent / Zero on Direct form systolic only
                _refuse(where + ".roundingMode", "roundingMode %s with overflowMode %s on %s: recorded Floor, Nearest "
                        "and Floor with saturation only" % (rm, ov, p["FilterStructure"]))
            self.cast = (fl, ot, rm, ov == "on")
            self.otype = ot
        self.dec = 1
        st = p["FilterStructure"]
        self.dec_nc = 1
        self.l54 = False
        if bt == "HDLFIRDecimator":
            self.dec = int(p["DecimationFactor"])
            self.dec_nc = int(p.get("NumCycles", "1"))
            tab = DEC[self.dec] if self.dec_nc == 1 else DEC_NC.get((self.dec, self.dec_nc), {})
            pal_lat = None
            if self.dec_nc > 1:
                br = [range(k_, n, self.dec) for k_ in range(self.dec)]
                allpal = all([self.c[i_] for i_ in b_] == [self.c[i_] for i_ in b_][::-1] for b_ in br)
                zero_br = any(all(self.c[i_] == 0 for i_ in b_) for b_ in br)
                few_mag = any(len({min(i_, n - 1 - i_) for i_ in b_}) > 1
                              and len({abs(self.c[i_]) for i_ in b_ if self.c[i_]}) < 2 for b_ in br)
                if allpal and not zero_br and not few_mag:
                    key_ = (self.dec, self.dec_nc, n)
                    if key_ not in DEC_NC_PAL or (n, True) not in tab:
                        _refuse(where + ".Numerator", "FIR Decimator by %d with NumCycles %d and %d taps whose every "
                                'polyphase branch is palindromic (taps k, k+%d,... read the same both ways): the '
                                "latency of that class was recorded on %s only (by 8 with "
                                "NumCycles 2 and 16 taps it moved by +75 steps)" % (
                                    self.dec, self.dec_nc, n, self.dec, ", ".join(
                                        "(%d, %d, %d)" % k for k in sorted(DEC_NC_PAL))))
                    pal_lat = tab[(n, True)] + DEC_NC_PAL[key_]
            if pal_lat is None and (n, sym) not in tab:
                if self.dec_nc == 1:
                    _refuse(where + ".Numerator", "FIR Decimator with %d taps: latency recorded for %s taps only"
                            % (n, ", ".join(map(str, DEC_NS))))
                _refuse(where + ".NumCycles", "FIR Decimator by %d with NumCycles %d and %d %s taps: not recorded "
                        "(recorded with NumCycles > 1: %s)" % (
                            self.dec, self.dec_nc, n, "symmetric" if sym else "asymmetric", "; ".join(
                                "factor %d NumCycles %d %d %s taps" % (d_, nc_, n_, "symmetric" if s_ else
                                                                        "asymmetric")
                                for (d_, nc_), t_ in sorted(DEC_NC.items()) for (n_, s_) in sorted(t_))))
            if self.dec_nc > 1 and pal_lat is None:
                # recordings (110 models on the 11 keys): each NumCycles > 1
                # latency is one recording with random symmetric coefficients, and the latency moves with the values.
                # Moved (README table): all taps equal or equal within each polyphase branch (branch k = taps k, k+D,
                # ...; alt2 with even D), an all-zero branch (halfband with even D, one-hot), and asymmetric sets.
                # Random symmetric sets agree on all 11 keys, and so do symmetric sets with two zeros at each end
                #. Accepted: truly symmetric (not antisymmetric) sets whose every branch
                # holds a nonzero tap and, over two or more free coefficients, two distinct nonzero magnitudes
                br = [range(k_, n, self.dec) for k_ in range(self.dec)]
                why = None
                if self.anti:
                    why = "antisymmetric coefficients"
                elif any(all(self.c[i_] == 0 for i_ in b_) for b_ in br):
                    why = "a polyphase branch whose taps are all 0 (e.g. halfband or one-hot)"
                elif any(len({min(i_, n - 1 - i_) for i_ in b_}) > 1
                         and len({abs(self.c[i_]) for i_ in b_ if self.c[i_]}) < 2 for b_ in br):
                    # a branch of two or more free coefficients (a mirror pair, taps i and n-1-i, is one: the recorded
                    # randsym by 3 with 8 taps) needs two distinct nonzero magnitudes. Moved without: all taps equal,
                    # one value per branch (phaseeq, recordings: -3 .. +75), few nonzero taps (centrepair -1 / -2).
                    # Zero taps are fine otherwise: symzero (two zeros at each end) matched on all 11 keys
                    why = "fewer than two distinct nonzero magnitudes in a polyphase branch (all taps equal, one " \
                          "value per branch, or few nonzero taps)"
                elif allpal:
                    # an all-palindromic set the class rule above does not take (unreachable while the
                    # zero-branch and magnitude rules come first; kept as a guard)
                    why = "every polyphase branch palindromic (taps k, k+D, ... read the same both ways)"
                if why:
                    _refuse(where + ".Numerator", ('FIR Decimator by %d with NumCycles %d and %s (branch k = taps k, '
                        'k+%d,...): the latency of NumCycles > 1 moves with the coefficient '
                        'values (16 equal taps by 4 one step later in the engine; in native '
                        'runs, equal-per-branch, zero-branch and asymmetric sets moved it by '
                        '-3 to +75 steps); it was recorded to hold for symmetric sets with two '
                        'distinct nonzero magnitudes in every branch; use those, or NumCycles 1') % (self.dec, self.dec_nc, why, self.dec))
            self.lat = pal_lat if pal_lat is not None else tab[(n, sym)]
            self.period = self.dec_nc
            self.gap = GAPS.get(("decim", self.dec, n, sym)) if self.dec_nc == 1 else (0, self.lat)
            self.l54 = self.dec_nc > 1 or (self.dec, n, sym) == (8, 8, True)
            if self.gap is None and not sym and self.dec in (2, 4) and self.dec_nc == 1:
                # asymmetric D2 a = N + 2, D4 a = N, b = 5 (a + b = the latency table)
                self.gap = (n + 2, 5) if self.dec == 2 else (n, 5)
        elif st == "Direct form systolic":
            if not 4 <= n <= 32:
                _refuse(where + ".Numerator", "%d taps: the systolic latency was recorded for 4 to 32 taps" % n)
            self.lat = (n + 6) if not sym else (math.ceil(n / 2) + 7)
            self.period = 1
            # asymmetric (N + 3, 3), symmetric (N/2 + 4, 3) -- even N only (odd N not recorded)
            self.gap = (n + 3, 3) if not sym else ((n // 2 + 4, 3) if n % 2 == 0 else None)
        elif st == "Direct form transposed":
            if not 4 <= n <= 32:
                _refuse(where + ".Numerator", "%d taps: the transposed latency was recorded for 4 to 32 taps" % n)
            self.lat = 6
            self.period = 1
            self.gap = (0, 6)
        else:
            nc = int(p["NumCycles"])
            if (n, sym) not in PS[nc]:
                _refuse(where + ".Numerator", "Partly serial systolic with %d taps: latency recorded for %s taps only"
                        % (n, ", ".join(map(str, NS))))
            self.lat = PS[nc][(n, sym)]
            self.period = nc
            self.l54 = (nc, n, sym) in L54_KEYS
            # every valid input gives its output the table latency later, whatever the spacing
            # (>= NumCycles); ready (third output) is low for NumCycles - 1 steps after each valid input
            self.gap = (0, self.lat)
        if self.new_input and not getattr(self, "new_gaps", False) or self.anti and st != "Partly serial systolic":
            # (an antisymmetric set: valid always 1 recorded, or the Partly serial rhythm, recordings)
            self.gap = None   # recorded with valid always high only: a gap is refused at run time
        self.serial = bt == "HDLFIRFilter" and st == "Partly serial systolic"
        self._pin_lab2_28(bt, p, n, sym, ot, rm, ov, where)
        # after a reset the partly serial filter gives its first len(c) valid outputs as 0; the
        # transposed filter was recorded with valid always high only
        self.reset_zero = len(self.c) if (self.reset and self.serial) else 0
        if self.reset and bt == "HDLFIRDecimator":
            # after a reset the decimator by 2 with the 6-tap set gives its first 3 valid outputs as
            # 0 (3 = taps / factor is a guess, not recorded: only this setting is accepted); valid always high
            self.reset_zero = 3
            self.gap = None
        if self.reset and st == "Direct form transposed":
            self.gap = None
        key = (getattr(self, "period", 1), n, sym)
        self.exact = self.serial and (key in PS_EXACT or self.reset) or self.dec_nc > 1
        if self.l54:
            # show these latencies on input sfix16_En15 with every other parameter at its library
            # default (inherited coefficient and output types, Floor, no saturation, no reset port)
            odd = sorted(k for k, dv in (FIR_DEFAULTS if bt == "HDLFIRFilter" else DEC_DEFAULTS).items()
                         if dv is not None and k not in ("FilterStructure", "NumCycles", "DecimationFactor",
                                                         "SimulateUsing") and p.get(k, dv) != dv)
            if tuple(x) != (1, 16, 15) or odd:
                _refuse(where + ("." + odd[0] if odd else ""), "this latency (%s) was recorded on input sfix16_En15 "
                        "with every other parameter at its library default only%s" % (
                            "NumCycles %d, %d %s taps" % (self.period, n, "symmetric" if sym else "asymmetric"),
                            ": %s=%r" % (odd[0], p.get(odd[0])) if odd else ": input %s" % (tuple(x),)))
        # (NumCycles 2, 6 taps, coefficient type fixdt(1,16,15) and an
        # explicit output type, valid always high -- recorded too)
        self.ah_ok = (self.serial and key in PS_ALWAYS and not self.reset and tuple(x) == (1, 16, 15)
                      and (ct is None or (ct == (1, 16, 15) and key == (2, 6, True))))
        self.ah_period = 2 * min(self.period, n) if self.serial else 0
        lab28 = (self.serial and key in PS_EXACT) or (bt == "HDLFIRDecimator" and (
            self.dec == 8 or (self.dec in (3, 4) and (n, sym) == (6, True))))
        if lab28 and (tuple(x) != (1, 16, 15) or ct is not None):
            # recordings (swap probe): the recordings latencies were recorded on input sfix16_En15 with the inherited
            # coefficient type only
            _refuse(where, 'this latency was recorded with input sfix16_En15 and the inherited coefficient type only'
                    "")
        self.ready_period = min(self.period, n) if self.serial else self.period
        self.where = where


    def _pin_lab2_28(self, bt, p, n, sym, ot, rm, ov, where):
        """recordings show its new latencies and explicit types in one setting each (review of
        recordings): Partly serial NumCycles 4 with 6 symmetric taps and the FIR Decimator by 3 / 4 with 6
        symmetric taps only with fixdt(1,18,15) (Nearest; Floor or Nearest for the decimator), an explicit type on
        those structures with these taps only, saturation on the decimator by 2 with 6 symmetric taps,
        fixdt(1,16,15) and valid always high only."""
        six = (n, sym) == (6, True)
        self.ah_only = False
        if self.serial and p.get("NumCycles") == "4" and (six or ot is not None):
            # also at full precision together with the reset port;
            # at full precision with valid always high (checked at run time: ah_only)
            if six and ot is None and not self.reset and rm == "Floor" and ov == "off":
                self.ah_only = True
            elif not (six and ot == (1, 18, 15) and rm == "Nearest" and ov == "off") and \
                    not (six and ot is None and self.reset and rm == "Floor" and ov == "off"):
                _refuse(where + ".OutputDataTypeStr", "Partly serial NumCycles 4: 6 symmetric taps and an explicit "
                        'output type were recorded together only, with fixdt(1,18,15), Nearest, no saturation'
                        "")
        if bt == "HDLFIRDecimator" and self.dec in (3, 4) and (six or ot is not None):
            if not (six and ot == (1, 18, 15) and rm in ("Floor", "Nearest") and ov == "off"):
                _refuse(where + ".OutputDataTypeStr", "FIR Decimator by %d: 6 symmetric taps and an explicit output "
                        'type were recorded together only, with fixdt(1,18,15), Floor or Nearest, no saturation'
                        "" % self.dec)
        if bt == "HDLFIRDecimator" and ov == "on":
            if not (self.dec == 2 and six and ot == (1, 16, 15)):
                _refuse(where + ".overflowMode", "saturation on the FIR Decimator was recorded by 2 with 6 symmetric "
                        "taps and fixdt(1,16,15) only")
            self.gap = None   # recorded with valid always high only


def _cast(v, fl, ot, rm, sat):
    """the full-precision stored integer v (fraction length fl) in the output type ot = (1, w, f): rounded by rm,
    then wrapped or saturated"""
    sh = fl - ot[2]
    if sh > 0:
        q, r = divmod(v, 1 << sh)   # floor
        half = 1 << (sh - 1)
        if rm == "Nearest":
            q += 1 if r >= half else 0
        elif rm == "Convergent":
            q += 1 if (r > half or (r == half and q & 1)) else 0
        elif rm == "Zero":
            q += 1 if (v < 0 and r) else 0
    else:
        q = v << -sh
    w = ot[1]
    lo, hi = -(1 << (w - 1)), (1 << (w - 1)) - 1
    if sat:
        return max(lo, min(hi, q))
    q &= (1 << w) - 1
    return q - (1 << w) if q > hi else q


class Kernel:
    """One output port (0 data, 1 valid, 2 ready) of the filter; the whole filter is evaluated per step."""

    def __init__(self, plan, port):
        self.pl, self.port = plan, port
        self.reset()

    def reset(self):
        """The start of a run (fxgraph calls it before every run)."""
        self._hw_reset()
        self.zero_left = 0
        self.reset_at = None
        self.ah = False

    def _always_high(self, pl, t, x, valid):
        """valid high every step -- ready at t % P == 0 only; the inputs of t % P in (0, 1) taken."""
        self.ah = True
        if not valid:
            _refuse(pl.where, 'valid 0 at step %d after an always-high start: recorded with valid always high only'
                    "" % t)
        P = pl.ah_period
        if self.port == 2:
            return 1 if t % P == 0 else 0
        if t % P in (0, 1):
            self.hist.insert(0, x)
            del self.hist[len(pl.c):]
            s = sum(ci * xi for ci, xi in zip(pl.c, self.hist))
            due = t + pl.lat if t % P == 0 else t - 1 + pl.lat + P // 2
            self.pending[due] = s
        out = self.pending.pop(t, None)
        if self.port == 1:
            return 1 if out is not None else 0
        if out is not None and pl.cast is not None:
            out = _cast(out, *pl.cast)
        return out if out is not None else 0

    def _hw_reset(self):
        """The filter's own reset input: registers clear (the run's step count and reset bookkeeping stay)."""
        self.t = 0
        self.last_valid = None
        self.hist = []
        self.count = 0
        self.pending = {}
        self.kept = {}

    def __call__(self, v):
        pl = self.pl
        x, valid = v[0], v[1]
        if pl.reset and v[2]:
            # the output due at a reset step still comes out, then
            # every register clears and that step's input is not taken
            t = self.t
            # review of recordings: valid at a reset step as recorded (the regular rhythm / always high), and serial
            # resets only as recorded: from step 12, 13 or more steps apart, not while the zeros are coming out
            if pl.exact and bool(valid) != (t % pl.period == 0) or pl.gap is None and not pl.exact and not valid:
                _refuse(pl.where, "valid %d at the reset step %d: not recorded" % (int(bool(valid)), t))
            # (a reset while the zeroed outputs of the previous one come out restarts the count --
            # 4 taps reset at 12 / 25, 8 taps at 25 / 50)
            if pl.reset_zero and not pl.serial and (t < 13 or (self.reset_at is not None and t - self.reset_at < 27)
                                                    or self.zero_left):
                _refuse(pl.where, "reset at step %d: decimator resets were recorded from step 13 on, 27 or more steps "
                        "apart, not while the zeroed outputs of an earlier reset are due" % t)
            if pl.serial and (t < 12 or (self.reset_at is not None and t - self.reset_at < 13)):
                _refuse(pl.where, "reset at step %d: partly serial resets were recorded from step 12 on and 13 or more "
                        "steps apart only" % t)
            ready_r = 1 if (self.last_valid is None or t - self.last_valid >= pl.ready_period) else 0
            out = self.pending.pop(t, None)
            if pl.reset_zero and out is not None and self.zero_left:
                out, self.zero_left = 0, self.zero_left - 1
            self._hw_reset()
            self.t = t + 1
            if pl.reset_zero and not pl.serial:
                self.zero_left = pl.reset_zero   # the decimator: the grouping restarts after t
                self.reset_at = t
            if pl.serial:
                # the reset step restarts the serial cycle (ready low at the next step after a reset
                # on an odd step); the valid input arriving then is still taken (its output is one of the zeros)
                self.last_valid, self.reset_at = t, t
                self.zero_left = pl.reset_zero
            if self.port == 1:
                return 1 if out is not None else 0
            if self.port == 2:
                return ready_r if pl.serial else 1
            if out is not None and pl.cast is not None:
                out = _cast(out, *pl.cast)
            return out if out is not None else 0
        t = self.t
        self.t += 1
        if pl.ah_ok and (self.ah or (t == 1 and valid and self.last_valid == 0)):
            return self._always_high(pl, t, x, valid)
        if pl.ah_only and t >= 1:
            _refuse(pl.where, "this configuration (Partly serial NumCycles 4, 6 symmetric taps, full precision) was "
                    "recorded with valid always high only")
        ready = 1 if (self.last_valid is None or t - self.last_valid >= pl.ready_period) else 0
        if pl.serial and self.last_valid is not None and self.last_valid == getattr(self, "reset_at", None):
            # after a reset ready is high again 2 steps on, whatever NumCycles (NumCycles 4,
            # reset at 12: 1 0 1 1 1 ...)
            ready = 1 if t - self.last_valid >= 2 else 0
        if pl.exact and bool(valid) != (t % pl.period == 0):   # reset steps are handled above
            _refuse(pl.where, "valid %d at step %d: this configuration was recorded with valid exactly every %d steps "
                    "from step 0 only" % (int(bool(valid)), t, pl.period))
        if pl.gap is None and not valid:
            _refuse(pl.where, "valid is 0 at step %d: only an always-high valid was recorded for this configuration"
                    % t)
        if valid and not (self.last_valid is None or t - self.last_valid >= pl.period or
                          self.last_valid == getattr(self, "reset_at", None)):
            _refuse(pl.where, "valid at step %d, %d steps after the previous one: a %s with NumCycles %d was "
                    "recorded taking a sample every %d steps or more" % (
                        t, t - self.last_valid, "partly serial filter" if pl.serial else "FIR Decimator", pl.period,
                        pl.period))
        if valid:
            self.last_valid = t
        if self.port == 2:
            return ready
        if valid:
            self.hist.insert(0, x)
            del self.hist[len(pl.c):]
            s = sum(ci * xi for ci, xi in zip(pl.c, self.hist))
            j = self.count
            if j % pl.dec == pl.dec - 1:
                self.kept[j] = s
            if pl.gap is None:   # always-high valid: the table latency
                if j in self.kept:
                    self.pending[t + pl.lat] = self.kept.pop(j)
            else:
                a, b = pl.gap
                if j - a in self.kept:
                    self.pending[t + b] = self.kept.pop(j - a)
            self.count += 1
        out = self.pending.pop(t, None)
        if out is not None and self.zero_left:
            out, self.zero_left = 0, self.zero_left - 1
        if self.port == 1:
            return 1 if out is not None else 0
        if out is not None and pl.cast is not None:
            out = _cast(out, *pl.cast)
        return out if out is not None else 0

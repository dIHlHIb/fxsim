"""Recorded bounds for numeric block parameters. The limits include library defaults from 924 saved models
and values observed in native tests. User parameters outside these bounds are refused before lowering.
Blocks generated during lowering use validated parameters and are exempt from this check.

ENFORCE is for the internal-consistency tests only (the regression check compares paths beyond the recorded
widths); every production path runs with it on."""

ENFORCE = [True]
INTERNAL = "\x1d"
# the vector pass replaces Tapped Delay, Concatenate and Bus Assignment by element blocks; it records them here, with
# the element signals that carry their inputs and outputs
VEC_KEY = "\x1dlab2_widths"
# the lowerings that move a line off output port k of a block record where it went: "block|k" -> [block, port]
# (an NCO's cos, a Complex to Real-Imag's outputs)
PORT_KEY = "\x1dlab2_ports"
# the BOOL_SHAPES rows the vector pass found fed by a vector (their names and element copies)
VEC_ROWS_KEY = "\x1dlab2_vec_rows"
# parameters a typed lowering uses other than as written (a Bias value rounded / saturated into the
# input type): {location: note} in fxgraph_lib._param_note's form, reported in parameter_quantization
NOTES_KEY = "\x1dlab2_param_notes"
# {"user": {name: type}, "helpers": [internal names]} the type-dependent recorded rewrites (fxhdl.typed_rewrite)
# added -- the cost counts them under their user block
HELPERS_KEY = "\x1dlab2_helpers"

# (block type, parameter): (lowest, highest, evidence)
BOUNDS = {
    ("BitConcat", "numInputs"): (1, 128, '2, 3, 4; 1, 5, 6, 8; 1.. 128'),
    ("BitShift", "N"): (0, 95, '0, 1, 11, 12, 15, 16; 0.. 95 (word length - 1 on 96 bits)'),
    ("BitRotate", "N"): (0, 64, '1, 3, 9; 0.. 64'),
    ("BitSet", "iBit"): (-1, 51, '0, 3, 7, 12; 0.. 32; -1 and the word length leave the input unchanged, 32.. 51 inside the word set the bit (Simulink refuses a bit index of 52 and above: Simulink:Masking:Bad_Init_Commands; all 16 trials at index 52 also failed)'),
    ("BitClear", "iBit"): (-1, 51, '0, 3, 7; 0.. 31; -1 and the word length leave the input unchanged, up to 51 inside the word clear the bit (Simulink refuses a bit index of 52 and above)'),
    ("BitSlice", "lidx"): (0, 200, '0.. 200'),
    ("BitSlice", "ridx"): (0, 127, '0.. 127'),
    ("BusCreator", "Inputs"): (2, 4, '3, 4; 2 (before a Bus Assignment)'),
    ("Concatenate", "NumInputs"): (2, 4, '2, 3; 4'),
    ("DataTypeDuplicate", "NumInputPorts"): (2, 3, "2, 3"),
    ("Repeat", "N"): (2, 5, '2, 3, 4; 5 (the library default)'),
    # recordings: 1, 2, 5 and 8 equal to Simulink as well; within the
    # range only the recorded counts are taken (fxvector TAPPED_COUNTS: 6 and 7 not recorded)
    ("TappedDelay", "NumDelays"): (1, 8, '3, 4; 1, 2, 5, 8'),
    ("TappedDelayEnabledSynchronous", "NumDelays"): (1, 8, '3, 4; 1, 2, 5, 8'),
    ("HDLCounter", "CountWordLen"): (4, 16, "4, 6, 8, 10, 16"),
    ("HDLCounter", "CountFracLen"): (0, 2, "0, 2"),
    # 5..7 stay refused by fxhdlfir.DEC_ALLOWED (the recorded set 2, 3, 4, 8)
    ("HDLFIRDecimator", "DecimationFactor"): (2, 8, '2, 3, 4; 8'),
    # HDL Counter values (defaults included): a count limit, step or start outside these was
    # never recorded -- in particular none near the count type's limits
    ("HDLCounter", "CountMax"): (5, 25, "5, 20, 25"),
    ("HDLCounter", "CountStep"): (-1, 5, "-1, 0.25, 1, 2, 3, 5", "real"),
    ("HDLCounter", "CountInit"): (-3, 10, "-3, 0, 3, 5, 10"),
    ("HDLCounter", "CountFrom"): (-3, 10, "-3, 0, 3, 5, 10"),
}


def pint(v, where):
    """An integer parameter as Simulink reads it: an ASCII integer literal (fxnum's numeric parser first; recordings: Python's int() took '2_0' as 20 and other digit scripts). BAD_PARAM otherwise."""
    import re
    import fxnum
    import fxgraph_lib as G
    if not re.fullmatch(r"\s*[+-]?[0-9]+\s*", str(v)):
        # coverage per block: plain arithmetic of numeric literals (2^4, 16/2) through the engine's evaluator
        # (fxnum._literal_arith); its value must be an exact integer -- names and functions stay refused
        lit = fxnum._literal_arith(str(v).strip()) if str(v).strip() else None
        from fractions import Fraction
        try:
            q = Fraction(lit) if lit is not None else None
        except (ValueError, ZeroDivisionError):
            q = None
        if q is not None and q.denominator != 1 and not re.fullmatch(r"\s*[+-]?[0-9.]+([eE][+-]?[0-9]+)?\s*", str(v)):
            # MATLAB's double value of the arithmetic is not an integer (1/49*49 is
            # 0.99999999999999989); Simulink runs it -- how it rounds there is not modelled (it was a request error)
            raise G.GraphError("UNSUPPORTED", where, "%r evaluates to %s in MATLAB's double arithmetic, not an "
                               "integer; how Simulink uses a non-integer value here is not modelled -- write the "
                               "integer" % (v, lit))
        if q is None:
            # 'int8(3)', '[3]' -- Simulink takes them; only these plain forms (an integer literal, in
            # brackets or one integer-class cast of it) are read here: arithmetic with an integer class follows
            # MATLAB's rounding rules (the engine's), not taken until merged
            m_ = re.fullmatch(r"\s*(?:\[\s*([+-]?[0-9]+)\s*\]|(u?)int(8|16|32)\(\s*([+-]?[0-9]+)\s*\))\s*", str(v))
            if m_ and m_.group(1) is not None:
                return int(m_.group(1))
            if m_:
                k_, w_ = int(m_.group(4)), int(m_.group(3))
                lo_, hi_ = (0, (1 << w_) - 1) if m_.group(2) else (-(1 << (w_ - 1)), (1 << (w_ - 1)) - 1)
                if lo_ <= k_ <= hi_:   # (a cast that saturates is left to the refusal below)
                    return k_
        if q is None and re.search(r"\b[A-Za-z]\w*\s*\(|\b0[xXbB][0-9A-Fa-f]|\b(true|false)\b", str(v)):
            # (any function call -- floor, hex2dec, bitcmp, double --, hex / binary literals (0x03, whose
            # class MATLAB takes from the digit count) and true / false are MATLAB expressions too)
            # recordings (a previous version, Q1 / Q7): an expression MATLAB evaluates (0.3*int8(5) is 2) -- integer-class
            # arithmetic with a non-integer operand is refused as a class (not fitted); Simulink runs it, so it is not
            # supported here rather than a malformed request
            raise G.GraphError("UNSUPPORTED", where, "%r: an expression Simulink evaluates; only integer literals, "
                               "exact integer arithmetic and an integer-class cast of an integer are read here -- "
                               "write the integer value" % (v,))
        if q is None or q.denominator != 1:   # (a non-integer literal such as 4.5: the request is wrong)
            raise G.GraphError("BAD_PARAM", where, "integer literal required, got %r" % (v,))
        return int(q)
    c = fxnum.canonical(v, "scalar")
    if isinstance(c, fxnum.NumError) or not re.fullmatch(r"[+-]?[0-9]+", str(c)):
        raise G.GraphError("BAD_PARAM", where, "integer literal required, got %r" % (v,))
    return int(c)


def number(v, where):
    """A finite numeric literal through fxnum (Fraction() took '14/2' and '1_0'): its canonical text, else BAD_PARAM."""
    import re
    import fxnum
    import fxgraph_lib as G
    c = fxnum.canonical(v, "scalar")
    if isinstance(c, fxnum.NumError):
        # recordings (the sweep after recordings): fxnum's own kind -- 'int8(8)' is not supported here (UNSUPPORTED), not
        # a malformed request; the message said "not supported" under BAD_PARAM
        # (a bare name such as 'abc' stays a request error, as in pint: no workspace variables here)
        code = c.code if c.code and not re.fullmatch(r"\s*[A-Za-z_]\w*\s*", str(v)) else "BAD_PARAM"
        raise G.GraphError(code, where, c.message)
    if str(c).lower().lstrip("+-") in ("inf", "nan"):
        raise G.GraphError("BAD_PARAM", where, "a finite number required, got %r" % (v,))
    return c


# the engine's value when a bounded parameter is omitted (engine/blockspec.py, fxhdl.py, fxvector.py, block_lowering.py,
# fxhdlfir.py defaults)
DEFAULTS = {("BitConcat", "numInputs"): "2", ("BitShift", "N"): "0", ("BitSlice", "lidx"): "7",
            ("BitRotate", "N"): "1", ("BitSet", "iBit"): "0", ("BitClear", "iBit"): "0",
            ("BitSlice", "ridx"): "0", ("BusCreator", "Inputs"): "2", ("Concatenate", "NumInputs"): "2",
            ("DataTypeDuplicate", "NumInputPorts"): "2", ("Repeat", "N"): "5", ("TappedDelay", "NumDelays"): "4",
            ("TappedDelayEnabledSynchronous", "NumDelays"): "4", ("HDLCounter", "CountWordLen"): "8",
            ("HDLCounter", "CountFracLen"): "0", ("HDLFIRDecimator", "DecimationFactor"): "2",
            ("HDLCounter", "CountMax"): "25", ("HDLCounter", "CountStep"): "1", ("HDLCounter", "CountInit"): "0",
            ("HDLCounter", "CountFrom"): "0"}


# HDL Counters recorded outside the ranges above, accepted as these exact settings only --
# (CountType, CountWordLen, CountMax (None: Free running), CountStep, CountInit, (reset, load, enable, direction ports))
_P0, _P4 = ("off",) * 4, ("on",) * 4
_PE, _PRE = ("off", "off", "on", "off"), ("on", "off", "on", "off")   # enable; reset + enable
# recorded block parameters that the .slx reader resolves when they name enclosing mask parameters or saved model
# workspace variables (a previous version, recordings; e.g. CountWordLen = addrWL, CountMax =
# 2^addrWL-1 in the example models). The value is then checked like a
# literal (pint, the recorded ranges / whitelists).
NAMED_NUMERIC = {"HDLCounter": ("CountWordLen", "CountFracLen", "CountMax", "CountInit", "CountStep", "CountFrom"),
                 "BitSlice": ("lidx", "ridx"), "BitConcat": ("numInputs",), "BitShift": ("N",),
                 "HDLRAM": ("AddressBits", "ram_size"),
                 # In the example models, 140 Extract Bits write bitIdxRange as
                 # '[0 NumBits]' / '[0 NumBits-1]' / '[NumBits 2*NumBits]' with NumBits a parameter of the enclosing
                 # mask; read as the values like the names above
                 "ExtractBits": ("bitIdxRange", "numBits")}

COUNTER_RECORDED = {
    ("Free running", 24, None, 1, 0, _P0), ("Free running", 32, None, 1, 0, _P0),
    ("Free running", 24, None, 1000, 0, _P0), ("Free running", 24, None, 100000, 0, _P0),
    ("Free running", 32, None, 50000000, 0, _P0), ("Free running", 32, None, 1, 0, _P4),
    ("Count limited", 8, 255, 1, 0, _P0), ("Count limited", 10, 1023, 1, 0, _P0),
    ("Count limited", 16, 65535, 1, 0, _P0), ("Count limited", 8, 200, 1, 0, _P0),
    ("Count limited", 10, 1023, 1, 923, _P0), ("Count limited", 16, 65535, 1, 65435, _P0),
    ("Count limited", 16, 65535, 1, 0, _P4),
    ("Modulo", 10, 1000, 1, 0, _P0), ("Modulo", 10, 1000, 1, 900, _P0),
    # CountMax 200 with the direction port alone
    ("Count limited", 8, 200, 1, 0, ("off", "off", "off", "on")),
    # word lengths 3 and 20, and the 12-bit Free running counter with step 100
    ("Count limited", 3, 7, 1, 0, _P0), ("Free running", 3, None, 1, 0, _P0),
    ("Count limited", 20, 25, 1, 0, _P0), ("Free running", 20, None, 1, 0, _P0),
    ("Free running", 20, None, 30000, 0, _P0), ("Free running", 12, None, 100, 0, _P0),
    # the most frequent literal settings in the example models (ports: reset, load, enable, dir)
    ("Free running", 32, None, 1, 0, _PE), ("Free running", 3, None, 1, 0, _PRE),
    ("Count limited", 2, 3, 1, 0, _PE), ("Count limited", 3, 7, 1, 0, _PE),
    ("Free running", 3, None, -1, 0, ("off", "on", "on", "off")), ("Free running", 2, None, 1, 0, _PRE),
    ("Count limited", 1, 1, 1, 0, _PRE), ("Count limited", 16, 65535, 1, 65500, _PRE),
    ("Count limited", 8, 239, 1, 200, _PRE), ("Free running", 9, None, 1, 480, _PRE),
    ("Modulo", 24, 4915199, 1, 0, _PE), ("Modulo", 24, 4915199, 1, 4915190, _PE)}


# Bit Slice ranges recorded on 40 / 48 / 64-bit inputs (fraction length 0, signed and
# unsigned; bits beyond the input word read 0): these (lidx, ridx) skip the bounds; fxhdl.bit_out_type pins the input
# (lidx, ridx) -> the input word lengths it was recorded on; recordings: 18-bit slices of 108-bit inputs
# (signed and unsigned, the bits read unsigned) and 47..40
BITSLICE_WIDE = {(39, 0): (40, 48, 64), (47, 16): (40, 48, 64), (63, 32): (40, 48, 64), (63, 63): (40, 48, 64),
                 (35, 18): (108,), (53, 36): (108,), (71, 54): (108,), (89, 72): (108,), (107, 90): (108,),
                 (47, 40): (108,)}
BITSLICE_WIDE_INPUTS = (40, 48, 64, 108)
# Bit Concat with these input counts (inputs uint8 and boolean alternating; one input:
# uint8, passed through) beyond the recorded 2..4
BITCONCAT_RECORDED = ()   # (recordings 1 / 5 / 6 / 8: inside the recordings range 1 .. 128 now)


def bitslice_wide(b):
    try:
        return (pint(str(b.get("lidx", "7")).strip(), "lidx"), pint(str(b.get("ridx", "0")).strip(), "ridx")) \
            in BITSLICE_WIDE
    except Exception:   # noqa: BLE001 -- the bound check reports a bad spelling
        return False


def counter_recorded(b):
    """Whether an HDL Counter block is one of the recordings show settings (unsigned, fraction length 0, the other
    parameters at their defaults)."""
    import fxhdl
    def g(k):
        return str(b.get(k, fxhdl.COUNTER_DEFAULTS.get(k, ""))).strip()
    # review of recordings: the block's own defaults (CountType Count limited), no count_hit output, CountSampTime 1
    if g("CountDataType") != "Unsigned" or g("CountFracLen") != "0" or g("CountHitOutputPort") != "off" \
            or g("CountSampTime") != "1":
        return False
    import fxgraph_lib as G
    def num(k):   # the strict integer reader (Python's int() takes '2_00' as 200)
        return pint(g(k), "design.blocks.HDLCounter." + k)
    try:
        ct = g("CountType")
        key = (ct, num("CountWordLen"), None if ct == "Free running" else num("CountMax"), num("CountStep"),
               num("CountInit"), tuple(g(k) for k in
                                         ("CountResetPort", "CountLoadPort", "CountEnbPort", "CountDirPort")))
    except (ValueError, G.GraphError):
        return False
    return key in COUNTER_RECORDED


# recordings (108 counters Simulink ran, 256 steps each, equal step by step on both paths): the word
# lengths swept for each (CountType, signed); within them every count limit, step and start the type holds is
# accepted (a systematic sweep of each parameter over its whole range, not single settings)
# an unsigned Count limited counter of 33 / 34 bits and a fraction length 4 (ufix8_En4, step 0.25)
# recorded equal beyond the recorded sweep
COUNTER_SWEPT_WL = {("Count limited", 0): (1, 34), ("Count limited", 1): (2, 32), ("Free running", 0): (1, 33),
                    ("Free running", 1): (5, 32), ("Modulo", 0): (1, 32), ("Modulo", 1): (2, 32)}
COUNTER_SWEPT_FL = {"Count limited": (0, 4), "Free running": (0, 3), "Modulo": (0, 0)}


def counter_swept(b):
    """Whether an HDL Counter lies inside the recorded sweep: its word length in the swept interval of its type and
    signedness, its fraction length in the swept one; the values themselves are checked by the block (they must fit
    the type, Simulink's own rules)."""
    import fxhdl
    def g(k):
        return str(b.get(k, fxhdl.COUNTER_DEFAULTS.get(k, ""))).strip()
    if g("CountSampTime") != "1" or g("CountFromType") != "Initial value":
        return False
    sg = {"Unsigned": 0, "Signed": 1}.get(g("CountDataType"))
    rng = COUNTER_SWEPT_WL.get((g("CountType"), sg))
    if rng is None:
        return False
    try:
        w, f = pint(g("CountWordLen"), "CountWordLen"), pint(g("CountFracLen"), "CountFracLen")
    except Exception:   # noqa: BLE001 -- the bound check reports a bad spelling
        return False
    flo, fhi = COUNTER_SWEPT_FL[g("CountType")]
    return rng[0] <= w <= rng[1] and flo <= f <= fhi


BIT_INT_PARAMS = {("BitSet", "iBit"), ("BitClear", "iBit"), ("BitShift", "N"), ("BitRotate", "N"),
                  ("BitSlice", "lidx"), ("BitSlice", "ridx")}


def normalize_ints(spec):
    """recordings (recordings bx*): the bit blocks' integer parameters written in MATLAB forms Simulink evaluates --
    int8(4), uint8(51), '2+3', '3.0', '1e1' -- are rewritten as the integer they evaluate to (each with a Simulink record,
    equal); anything that is not an exact integer is left as written (the checks refuse it with its reason)."""
    if not isinstance(spec, dict) or not isinstance(spec.get("blocks"), dict):
        return spec
    import re
    import fxnum
    import fxcore
    from fractions import Fraction
    out = None
    for n, b in spec["blocks"].items():
        if not isinstance(b, dict):
            continue
        for (t, k) in BIT_INT_PARAMS:
            if b.get("type") != t or k not in b or re.fullmatch(r"\s*[+-]?[0-9]+\s*", str(b[k])):
                continue
            v = str(b[k]).strip()
            q = None
            try:
                r_ = fxcore.matlab_int_expr(v)
                q = r_[0] if r_ is not None else None
            except Exception:   # noqa: BLE001
                q = None
            if q is None:
                lit = fxnum._literal_arith(v)
                try:
                    q = Fraction(lit) if lit is not None else None
                except (ValueError, ZeroDivisionError):
                    q = None
            if q is not None and Fraction(q).denominator == 1:
                if out is None:
                    out = dict(spec, blocks=dict(spec["blocks"]))
                out["blocks"][n] = dict(out["blocks"].get(n, b), **{k: str(int(q))})
    return out if out is not None else spec


def check(spec):
    """Raise GraphError UNSUPPORTED for the first user block parameter outside its recorded range."""
    if not ENFORCE[0] or not isinstance(spec, dict) or not isinstance(spec.get("blocks"), dict):
        return
    import fxgraph_lib as G
    for n, b in spec["blocks"].items():
        if INTERNAL in n or not isinstance(b, dict):
            continue
        t = b.get("type")
        # a recorded wide counter skips only the bounds of the parameters it is keyed on (review of recordings: CountFrom
        # and, for Free running, CountMax are still checked)
        skip = ()
        if t == "BitSlice" and bitslice_wide(b):
            skip = ("lidx", "ridx")
        if t == "BitConcat":
            try:
                if pint(str(b.get("numInputs", "2")).strip(), "numInputs") in BITCONCAT_RECORDED:
                    skip = ("numInputs",)   # recordings (fxhdl pins the input types)
            except Exception:   # noqa: BLE001 -- the bound check reports a bad spelling
                pass
        if t == "HDLCounter" and counter_swept(b):
            # CountFrom (only mirroring CountInit) stays bounded. recordings (28 pairs): a Free
            # running counter does not use CountMax and Simulink does not check its range (1, 3, 71, 100, 300 on 8
            # bits, 864 on 9 bits gave the default's run step for step) -- only its spelling is checked
            skip = ("CountWordLen", "CountFracLen", "CountStep", "CountInit", "CountMax")
            if str(b.get("CountFromType", "Initial value")).strip() == "Initial value" and \
                    str(b.get("CountFrom", b.get("CountInit", "0"))).strip() == str(b.get("CountInit", "0")).strip():
                skip += ("CountFrom",)
        elif t == "HDLCounter" and counter_recorded(b):
            skip = ("CountWordLen", "CountStep", "CountInit") + (
                ("CountMax",) if str(b.get("CountType", "Count limited")).strip() != "Free running" else ())
            # with CountFromType 'Initial value' CountFrom only mirrors CountInit (the reader writes it so)
            if str(b.get("CountFromType", "Initial value")).strip() == "Initial value" and \
                    str(b.get("CountFrom", b.get("CountInit", "0"))).strip() == str(b.get("CountInit", "0")).strip():
                skip += ("CountFrom",)
        for (bt, p), row in BOUNDS.items():
            lo, hi, src = row[:3]
            real = row[3:] == ("real",)
            if bt != t:
                continue
            # an omitted parameter takes the engine's default, which is checked like a written one
            raw = b[p] if p in b else DEFAULTS[(bt, p)]
            where = "design.blocks.%s.%s" % (n, p)
            # a spelling fxnum does not read ('2_00', '400/2', other digits), Inf or NaN is
            # BAD_PARAM here -- the block code must never read it on its own
            from fractions import Fraction
            # a skipped (recorded / swept) parameter is only read as a finite number here -- a fraction-length
            # counter's CountMax 187.5; the block checks its grid and integers
            v = Fraction(number(raw, where)) if (real or p in skip) else pint(raw, where)
            if p in skip:
                continue   # a recorded / swept setting: its spelling is checked above, not the range
            if not lo <= v <= hi:
                raise G.GraphError("UNSUPPORTED", where,
                                   "%s %s = %s%s is outside the recorded range %d..%d (%s); not computed until "
                                   "Simulink is recorded there" % (t, p, raw, "" if p in b else " (the default)",
                                                                   lo, hi, src))


# compiled word lengths (largest of the block's output and its inputs): the largest seen in recordings
# and native probes of each block (recordings and the records' types)
WIDTHS = {
    # (output bound, evidence[, input bound]): recordings -- the inputs separately where they were narrower
    "BitConcat": (326, 'outputs 1.. 326 bits, inputs 1.. 64', 64),
    "BitSlice": (201, 'outputs 1.. 201 bits, inputs 1.. 128', 128),
    "BitShift": (96, '12 / 16 bits; 17.. 96 bits'),
    "BitRotate": (128, '8 / 12 bits; 17.. 128 bits'),
    "BitSet": (128, '8 / 12 bits; 18.. 65 bits; 80 / 96 / 125 / 127 bits'),
    "BitClear": (128, '8 / 12 bits; 17.. 128 bits'),
    "UnitDelayEnabledSynchronous": (40, '8.. 40 bits'),
    "UnitDelayEnabledResettableSynchronous": (40, '8.. 40 bits'),
    "UnitDelayResettableSynchronous": (40, '8.. 40 bits'),
    "HDLCounter": (16, '4.. 16 bits'),
    "DetectRisePositive": (128, 'up to 12 bits; 18.. 128 bits, outputs boolean'),
    # recordings #8: the recorded "the 20-bit model was refused by Simulink" does not hold in general (24 .. 64 bits ran,
    # 11 / 11); the bound stays because wider words were not recorded
    "Repeat": (64, '16 bits; 24.. 64 bits'),
    "DataTypeDuplicate": (128, '16 bits; 17.. 128 bits'),
    "Concatenate": (65, '16 bits; 18.. 65 bits'),
    "DirectLookupND": (32, '8 / 16 bits; 17.. 32 bits (wider indices: the engine refuses, 32 integer bits)'),
    "Bias": (128, '8 / 16 bits; 17.. 53 bits; up to 125.. 128 bits'),
    "IncrementRealWorld": (128, '20.. 53 bits; up to 125.. 128 bits'),
    "DecrementRealWorld": (128, '17.. 48 bits; up to 125.. 128 bits'),
    "IncrementStoredInteger": (128, '18.. 53 bits; up to 125.. 128 bits'),
    "DecrementStoredInteger": (128, '17.. 40 bits; up to 125.. 128 bits'),
    "BitReduce": (33, "inputs up to 33 bits"),
    # (Compare To Zero is the engine's CompareToConstant, whose widths rest on the main recordings sets: not a row here)
    "ManualSwitch": (128, 'up to 16 bits; 18.. 128 bits'),
    "BusAssignment": (128, 'up to 16 bits; 17.. 128 bits'),
    "TappedDelay": (65, '16 bits; 20.. 65 bits'),
    "TappedDelayEnabledSynchronous": (128, '16 bits; 17.. 128 bits'),
    "Reshape": (65, '16 bits; 18.. 65 bits'),
    "SignalSpecification": (108, "up to 108 bits"),
    "Assertion": (128, 'up to 16 bits; 17.. 128 bits'),
    "HDLRAM": (32, 'data 8 / 12 / 16 / 18 / 24 / 32 bits or boolean, addresses 2.. 5, '
                   "8 bits (32-bit data only in the recorded types, fxhdl), write enables boolean"),
}


def user_blocks(spec):
    """{name: (type, [source names])} of the user's blocks WIDTHS covers (before any lowering)."""
    if not isinstance(spec, dict) or not isinstance(spec.get("blocks"), dict):
        return {}
    out = {}
    for n, b in spec["blocks"].items():
        if INTERNAL in n or not isinstance(b, dict) or b.get("type") not in WIDTHS:
            continue
        srcs = [(ln.get("src"), _port(ln.get("sport", 1))) for ln in spec.get("lines", [])
                if isinstance(ln, dict) and ln.get("dst") == n]
        out[n] = (b["type"], srcs)
    for n, row in (spec.get(VEC_KEY) or {}).items():
        t, names = row[0], row[1]
        if t in WIDTHS and n not in out:   # it adds the vector pass's blocks, never replaces a row
            # each element with its output port (a MATLAB Function's output k >= 2)
            out[n] = (t, [tuple(x) if isinstance(x, (list, tuple)) else (x, 1) for x in names[:]])
    return out


def _port(v):
    try:
        return int(v)
    except (TypeError, ValueError):
        return 1


def _driver_or_none(d, s, sp):
    """The compiled block carrying output port sp of design block s: through the recorded lowering's aliases (an
    identity block such as a Signal Specification removed) and its port maps (output port 2 of a MATLAB Function, a
    counter's hit); recordings -- the line's own source name is not always a compiled block of that type."""
    ports = getattr(d, "_lab2_ports", None) or {}
    for _ in range(64):
        nxt = ports.get("%s|%d" % (s, sp))
        if not nxt:
            break
        s, sp = nxt[0], int(nxt[1])
    if sp == 1 and s in getattr(d, "otype", {}):
        return s
    info = getattr(d, "hdl_info", None)
    lowered = INTERNAL + "%s/m/y%d" % (s, sp)   # a MATLAB Function compiled into blocks (fxhdl: output port k)
    if sp > 1 and lowered in getattr(d, "otype", {}):
        return lowered
    if not info:
        return s if sp == 1 else None
    import fxhdl
    for _ in range(64):
        try:
            m = fxhdl._port_block(info, s, sp, "design")
        except Exception:   # noqa: BLE001 -- a port that does not exist is the design check's to report
            return None
        if m == s or m not in info.get("alias", {}):
            return m
        s, sp = m, 1
    return None


def _driver(d, s, sp, n):
    """The compiled driver of (s, sp); recordings: one that cannot be found is refused, not skipped."""
    m = _driver_or_none(d, s, sp)
    if m is None or m not in getattr(d, "otype", {}):
        import fxgraph_lib as G
        raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "the compiled source of input %s port %d of this "
                           "%s could not be traced, so its recorded word length cannot be checked"
                           % (s, sp, (getattr(d, "_lab2_user", {}).get(n) or ("block",))[0]))
    return m


# a boolean into Concatenate, Data Type Duplicate, Reshape and Tapped Delay is accepted in the
# one shape recordings show for each (parameter: accepted values, the engine default first);
# any other shape with a boolean is refused until recorded
NO_BOOLEAN = ()
BOOL_SHAPES = {
    "Concatenate": {"NumInputs": ("2",), "Mode": ("Vector",)},
    "DataTypeDuplicate": {"NumInputPorts": ("2",)},
    "Reshape": {"OutputDimensionality": ("1-D array",)},
    "TappedDelay": {"NumDelays": ("3",), "vinit": ("0", "0.0"), "DelayOrder": ("Oldest",), "includeCurrent": ("off",)},
}
BOOL_DEFAULTS = {"Concatenate": {"NumInputs": "2", "Mode": "Vector"}, "DataTypeDuplicate": {"NumInputPorts": "2"},
                 "Reshape": {"OutputDimensionality": "1-D array"},
                 "TappedDelay": {"NumDelays": "4", "vinit": "0.0", "DelayOrder": "Oldest", "includeCurrent": "off"}}


BOOL_VECTOR_ROWS = ("Concatenate", "DataTypeDuplicate", "Reshape")


def _bool_shape_ok(t, params):
    rows = BOOL_SHAPES.get(t)
    if rows is None:
        return True
    vin = (params or {}).get("_vector_inputs")
    if vin:
        # show boolean 3-vectors into Concatenate(2), DTD(2) and
        # Reshape 1-D (equal widths); any other width or a mix of widths (scalar + vector) is not recorded
        ws = [int(x) for x in str(vin).split(",")]
        if t not in BOOL_VECTOR_ROWS or len(set(ws)) != 1 or ws[0] not in (1, 3):
            return False
    for k, ok in rows.items():
        v = str((params or {}).get(k, BOOL_DEFAULTS[t][k])).strip()
        if v not in ok:
            return False
    return True


def user_params(spec):
    """{name: parameters} of the user blocks BOOL_SHAPES covers, HDL Counters and Bit Slices (vector-pass blocks from VEC_KEY's
    third field)."""
    out = {}
    if not isinstance(spec, dict) or not isinstance(spec.get("blocks"), dict):
        return out
    for n, b in spec["blocks"].items():
        # HDL Counters and Bit Slices too: check_widths admits their recorded wide settings by the parameters
        if INTERNAL not in n and isinstance(b, dict) and (b.get("type") in BOOL_SHAPES or
                                                          b.get("type") in ("HDLCounter", "BitSlice")):
            out[n] = {k: v for k, v in b.items() if k != "type"}
    for n, row in (spec.get(VEC_KEY) or {}).items():
        if len(row) > 2 and n not in out:
            out[n] = dict(row[2])
    for x in spec.get(VEC_ROWS_KEY) or []:
        n, ws = (x[0], x[1]) if isinstance(x, (list, tuple)) else (x, "0")
        if n in out:
            out[n] = dict(out[n], _vector_inputs=ws)
    return out


def check_widths(d):
    """Raise GraphError UNSUPPORTED when a user the extended blocks's compiled output or input is wider than recorded. Also
    (whether or not the ranges are enforced): a the extended blocks with a word wider than the C core's 126 bits keeps the
    design on the Python path (Manual Switch, Tapped Delay Enabled, Assertion, Bus Assignment, Data Type
    Duplicate, Bit Clear on 128-bit words stopped the C core with CORE_FAILED / PROTOCOL)."""
    import fxgraph_lib as G
    for n, (t, srcs) in (getattr(d, "_lab2_user", None) or {}).items():
        for m in [n] + [_driver(d, x, sp, n) for x, sp in srcs]:
            ty = d.otype.get(m)
            if isinstance(ty, tuple) and len(ty) == 3 and ty[1] > G.VM_BITS:
                d.vm_ok = False
    if not ENFORCE[0]:
        return
    for n, (t, srcs) in (getattr(d, "_lab2_user", None) or {}).items():
        row = WIDTHS[t]
        src = row[1]
        for m in [n] + [_driver(d, x, sp, n) for x, sp in srcs]:
            ty = d.otype.get(m)
            if ty == G.BOOL and not _bool_shape_ok(t, (getattr(d, "_lab2_params", None) or {}).get(n)):
                raise G.GraphError("UNSUPPORTED", "design.blocks." + n,
                                   "%s with a boolean %s in a shape not recorded (recorded with a scalar boolean "
                                   'or, for Concatenate / DTD / Reshape, equal 3-element boolean vectors: %s'
                                   ')' % (t, "output" if m == n else "input",
                                                                  ", ".join("%s %s" % (k, v[0]) for k, v in
                                                                            BOOL_SHAPES[t].items())))
            if ty == G.BOOL and t in NO_BOOLEAN:
                raise G.GraphError("UNSUPPORTED", "design.blocks." + n,
                                   "%s with a boolean %s: recorded with fixed-point signals only ("
                                   "not computed until Simulink is recorded there)"
                                   % (t, "output" if m == n else "input"))
            hi = row[0] if m == n or len(row) < 3 else row[2]
            if t == "HDLCounter" and (counter_recorded((getattr(d, "_lab2_params", None) or {}).get(n) or {}) or
                                      counter_swept((getattr(d, "_lab2_params", None) or {}).get(n) or {})):
                hi = 34   # the recorded wide counters (24 / 32 bits); recordings: up to 33 bits;
                #           recordings
            if t == "BitSlice" and bitslice_wide((getattr(d, "_lab2_params", None) or {}).get(n) or {}):
                hi = 108  # inputs of 40 / 48 / 64 / 108 bits (fxhdl.bit_out_type pins
                #           the exact input per range)
            if isinstance(ty, tuple) and len(ty) == 3 and ty[1] > hi:
                raise G.GraphError("UNSUPPORTED", "design.blocks." + n,
                                   "%s with a %d-bit %s is outside the recorded word lengths (at most %d bits: %s); "
                                   "not computed until Simulink is recorded there"
                                   % (t, ty[1], "output" if m == n else "input", hi, src))

"""Discrete FIR Filter, FilterStructure 'Lattice MA' (28 models run,
the 24 on signed inputs reproduced at every step by this rule), lowered to the engine's own blocks.

Stage m = 1 .. M with reflection coefficient k_m (f_0 = g_0 = the input u):
    f_m[n] = f_{m-1}[n] + k_m * g_{m-1}[n-1]
    g_m[n] = k_m * f_{m-1}[n] + g_{m-1}[n-1]
    y[n]   = f_M[n]
Every product is rounded (Floor) and wrapped into the product type, every sum is made in the accumulator type (each
operand cast into it), the output is f_M cast (Floor, wrap) into the output type. Types:
  * written: CoefDataTypeStr / ProductDataTypeStr / AccumDataTypeStr / OutDataTypeStr -- recorded with exactly
    fixdt(1,16,15) / fixdt(1,32,28) / fixdt(1,36,28) / fixdt(1,24,16);
  * left to the library: coefficients at the best precision of the whole vector at the input's word length, the
    product and accumulator 32 bits with the coefficient's plus the input's fraction length, the output the input's
    type (dut_rtp: the states compile to that product type).
Answered only as recorded: signed inputs sfix16_En14 / sfix16_En8 / sfix12_En10, 2 to 8 coefficients, RndMeth Floor,
no saturation, zero initial states, single rate. An unsigned input is refused (Simulink rejects it with inherited
types, fxpParameterOverflow; with the written types its result differs from this rule)."""
import json
import re
from fractions import Fraction

INTERNAL = "\x1d"
LATTICE = "Lattice MA"
REC_IN = {(1, 16, 14), (1, 16, 8), (1, 12, 10)}
# ufix16_En8 ran with the explicit types only (inherited: the coefficients take the
# unsigned input's type and the negative ones overflow -- fxpParameterOverflow, 4 of 4)
REC_IN_UNSIGNED = {(0, 16, 8)}
# with the library types also on sfix8_En6 and sfix24_En12
REC_IN_LIB = {(1, 8, 6), (1, 24, 12)}
REC_EXPLICIT = {"CoefDataTypeStr": "fixdt(1,16,15)", "ProductDataTypeStr": "fixdt(1,32,28)",
                "AccumDataTypeStr": "fixdt(1,36,28)", "OutDataTypeStr": "fixdt(1,24,16)"}
TYPE_KEYS = tuple(REC_EXPLICIT)
DEFAULTS = {"CoefDataTypeStr": "Inherit: Same word length as input",
            "ProductDataTypeStr": "Inherit: Inherit via internal rule",
            "AccumDataTypeStr": "Inherit: Inherit via internal rule", "OutDataTypeStr": "Inherit: Same as input"}
# parameters a lattice takes here (as recorded); the rest must stay at their library values (ExternalReset 'None' is
# the library value every recording shows in dut_params; a reset port is not answered)
ACCEPTED = {"type", "FilterStructure", "Coefficients", "CoefSource", "InputProcessing", "RndMeth",
            "SaturateOnIntegerOverflow", "InitialStates"} | set(TYPE_KEYS)
LIB_VALUES = {"ExternalReset": "None"}
OUT_MODES = {"Inherit: Same as input": "input", "Inherit: Same as accumulator": "acc"}
_DOM = None


def lattice_domain():
    """the declared domain of the stage rule, engine/lattice_domain.json (the regression check writes it
    from the recordings the stage model reproduces; recordings):
    {"lib": {hardware: {input word: axes}}, "explicit": {"full" | "acc_only": axes}, "rules": ...}"""
    global _DOM
    if _DOM is None:
        import os
        _DOM = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "lattice_domain.json")))
    return _DOM


NONDIRECTED = ("Nearest", "Convergent", "Round")


def host_product_word(w):
    """the host's inherited product word: 2w rounded up to 8 / 16 / 32 when
    2w <= 32, exactly 2w above (8 -> 8, 10 .. 14 -> 16,
    18 .. 30 -> 32, then 34, 36, 38 .. 64 exactly)"""
    return 2 * w if 2 * w > 32 else (8 if 2 * w <= 8 else 16 if 2 * w <= 16 else 32)


def stage_types(w, f, ks, asic):
    """(coefficient, product, accumulator = state) types of the library rule; None for all-zero coefficients"""
    import fxcore as fc
    nz = [v for v in ks if v != 0]
    if not nz:
        return None
    cf = min(fc._best_precision_fl(v, 1, w) for v in nz)
    if asic:
        F = f + cf
        # integer bits: one input-sized term plus M terms of the coefficient type's bound times the input
        # (ceil(log2(1 + M 2^(w - cf))), exact in fractions)
        x, g = 1 + len(ks) * Fraction(2) ** (w - cf), 0
        while Fraction(2) ** g < x:
            g += 1
        A = (1, F + (w - f) + g, F)
        P = (1, 2 * w, F)
    else:
        W = 32 if 2 * w <= 32 else 64
        F = f + min(cf, W - w)
        A = (1, W, F)
        P = (1, host_product_word(w), F)
    return (1, w, cf), P, A


def _refuse(where, msg, code="UNSUPPORTED"):
    import fxgraph_lib as G
    raise G.GraphError(code, where, msg)


def _is(b):
    return isinstance(b, dict) and b.get("type") == "DiscreteFir" and \
        str(b.get("FilterStructure", "")).strip() == LATTICE


def has(spec):
    return any(_is(b) for b in (spec.get("blocks") or {}).values())


def provisional(spec):
    """Each lattice stands as a Gain 1 (the input's type) until its input type is known (the probe)."""
    if not has(spec):
        return spec
    blocks = {n: ({"type": "Gain", "Gain": "1", "OutDataTypeStr": "Inherit: Same as input"} if _is(b) else b)
              for n, b in spec["blocks"].items()}
    return dict(spec, blocks=blocks)


def _fix(t):
    return "fixdt(%d,%d,%d)" % t


def _parse(t, where):
    m = re.fullmatch(r"\s*fixdt\(\s*([01])\s*,\s*(\d+)\s*,\s*(-?\d+)\s*\)\s*", str(t))
    if not m:
        _refuse(where, "a fixdt type required")
    return int(m.group(1)), int(m.group(2)), int(m.group(3))


def _lower_one(spec, n, t_in):
    import fxcore as fc
    import fxnum
    b = spec["blocks"][n]
    where = "design.blocks." + n
    import blockspec as BS
    lib = dict(BS.DEFAULTS.get("DiscreteFir", {}), **BS.JSON_LIBRARY_DEFAULTS.get("DiscreteFir", {}))
    # (a parameter at its library value is what the recordings ran -- a model read back writes them all)
    extra = sorted(k for k in b if k not in ACCEPTED and not k.startswith("_") and
                   str(b[k]).strip() != str(lib.get(k, LIB_VALUES.get(k, "\x00"))).strip())
    if extra:
        _refuse(where + "." + extra[0], ("Discrete FIR 'Lattice MA' with %s: not recorded (recorded "
            'settings used coefficients and the four type parameters only)') % extra[0])
    rm = str(b.get("RndMeth", "Floor")).strip()
    sat = str(b.get("SaturateOnIntegerOverflow", "off")).strip()
    dom_, why = _in_domain(b, t_in, where)
    xdom = None
    if dom_ is None:
        xdom, xwhy = _in_explicit(b, t_in, where)
        why = why or xwhy
    whys = (": " + why) if why else ""
    if str(b.get("CoefSource", "Dialog parameters")).strip() not in ("Dialog parameters", "Dialog"):
        _refuse(where + ".CoefSource", "Discrete FIR 'Lattice MA': coefficients from the dialog only")
    ip = str(b.get("InputProcessing", "Elements as channels (sample based)")).strip()
    if ip != "Elements as channels (sample based)":
        _refuse(where + ".InputProcessing", "Discrete FIR 'Lattice MA': sample-based processing only")
    ic = str(b.get("InitialStates", "0")).strip()
    if ic not in ("0", "0.0", "[0]"):
        # (Simulink runs nonzero initial states; not modelled here)
        _refuse(where + ".InitialStates", "Discrete FIR 'Lattice MA' with InitialStates %s: zero initial states only "
                "(recorded); nonzero initial states are not modelled" % ic)
    if dom_ is None and xdom is None and isinstance(t_in, tuple) and len(t_in) == 3 and not t_in[0]:
        if _lib_types(b):
            _unsigned_lib(b, t_in, where)
        _unsigned_explicit(b, t_in, where)
    if dom_ is None and xdom is None and (rm != "Floor" or sat != "off"):
        _refuse(where, "Discrete FIR 'Lattice MA' with RndMeth %s and saturation %s: outside the declared stage-rule "
                "domain of engine/lattice_domain.json%s; record it to "
                "open it" % (rm, sat, whys))
    # recordings (fixed by class: back to what recordings show) -- G2: the block's own
    # SampleTime ran at the input's rate here (Simulink runs the filter at it); G4: ASIC / FPGA grows the inner words
    st_ = str(b.get("SampleTime", "-1")).strip()
    if st_ not in ("-1", ""):
        _refuse(where + ".SampleTime", "Discrete FIR 'Lattice MA' with its own SampleTime %s: not recorded (it "
                "inherits the input's rate there); leave it -1 and set the rate upstream" % st_)
    import fxcore as fc_
    if dom_ is not None:
        return _lower_stage(spec, n, t_in, dom_, rm, sat)
    if xdom is not None:
        tc, tp, ta = xdom["types"]
        return _build(spec, n, xdom["ks"], 1, t_in[1], t_in[2], tc, tp, ta, xdom["out"],
                      {"RndMeth": rm, "SaturateOnIntegerOverflow": sat}, cast_in=True)
    if fc_._asic(getattr(_HW, "v", None) or {}):
        _refuse(where, ("Discrete FIR 'Lattice MA' on ASIC / FPGA hardware with "
            'these settings: outside the declared stage-rule domain '
            '(engine/lattice_domain.json)%s; record it to open it') % whys)
    if isinstance(t_in, tuple) and tuple(t_in) in REC_IN_UNSIGNED and \
            {k: str(b[k]).strip() for k in TYPE_KEYS if k in b} == REC_EXPLICIT:
        pass   # u16En8 (4 designs): the explicit (signed) types on ufix16_En8
    elif not isinstance(t_in, tuple) or tuple(t_in) not in (REC_IN if not _lib_types(b) else REC_IN | REC_IN_LIB):
        if isinstance(t_in, tuple) and not t_in[0]:
            # (with non-negative coefficients Simulink runs an unsigned input -- not recorded here)
            _refuse(where, ("Discrete FIR 'Lattice MA' on an unsigned input %s: not recorded "
                '(recordings cover signed inputs sfix16_En14 / sfix16_En8 / sfix12_En10)') % _fix(t_in))
        _refuse(where, "Discrete FIR 'Lattice MA' on input %s: outside the declared stage-rule domain "
                "(engine/lattice_domain.json)%s; the earlier host path takes sfix16_En14 / sfix16_En8 / sfix12_En10 "
                "only" % (_fix(t_in) if isinstance(t_in, tuple) else t_in, whys))
    raw = str(b.get("Coefficients", "")).strip()
    if ";" in raw or "'" in raw:
        # recordings: a column ([0.5;0.25], [0.5 -0.25]') is one filter per row in Simulink, which a
        # lattice does not take (Simulink:blocks:DiscreteFirSIMONotAllowed)
        _refuse(where + ".Coefficients", "Discrete FIR 'Lattice MA' coefficients %r as a column: Simulink rejects it "
                "(DiscreteFirSIMONotAllowed, a lattice takes one row)" % raw, "BAD_PARAM")
    c = fxnum.canonical(raw, "vector")
    if isinstance(c, fxnum.NumError):
        _refuse(where + ".Coefficients", c.message, c.code or "BAD_PARAM")
    try:
        ks = [float(v) for v in str(c).strip("[]").replace(",", " ").split()]
    except ValueError:
        ks = []
    if any(v != v or v in (float("inf"), float("-inf")) for v in ks):
        # NaN / Inf ended INTERNAL (Simulink: InvParamSetting)
        _refuse(where + ".Coefficients", "Discrete FIR coefficients %r: not finite (Simulink rejects them)" % raw,
                "BAD_PARAM")
    lib_ = _lib_types(b)
    if not (2 <= len(ks) <= 8 or (lib_ and ks == [0.0])):
        # (one zero coefficient with the library types, the input passed through)
        _refuse(where + ".Coefficients", "Discrete FIR 'Lattice MA' with %d coefficients: 2 to 8 recorded (one, "
                "zero, with the library types)%s" % (len(ks), whys))
    if isinstance(t_in, tuple) and tuple(t_in) in REC_IN_LIB and len(ks) > 5:
        # recordings (silent wrong): sfix8_En6 / sfix24_En12 were recorded with
        # up to 5 coefficients. Extending the 16-bit input rule to 8 coefficients gave a different output.
        _refuse(where + ".Coefficients", "Discrete FIR 'Lattice MA' with %d coefficients on %s: 1 to 5 recorded on "
                "this input; record it to open it" % (len(ks), _fix(tuple(t_in))))
    mx = max(abs(v) for v in ks)
    if mx >= 1 or (mx < 0.25 and not lib_):
        # recordings (silent wrong): recorded coefficient sets have their largest magnitude in
        # [0.25, 1); below it the inherited coefficient fraction makes the accumulator too narrow for the input
        # (Simulink keeps the input's range), all-zero passes the input through, very large ones differ
        # with the library types, small (|k| < 0.25), tiny (0.001 .. 0.002), all-zero
        # and partly zero sets are recorded too -- the state type keeps the input's integer bits (see below)
        _refuse(where + ".Coefficients", "Discrete FIR 'Lattice MA' with the largest coefficient magnitude %s: "
                "recorded below 1 (and with the explicit types from 0.25)" % ("%g" % mx))
    given = {k: str(b[k]).strip() for k in TYPE_KEYS if k in b}
    if given and not (given == REC_EXPLICIT):
        if any(v != DEFAULTS[k] for k, v in given.items()):
            _refuse(where, "Discrete FIR 'Lattice MA' with the types %s: recorded with all four left to the library "
                    "or exactly %s" % (given, REC_EXPLICIT))
    s, w, f = t_in
    if given != REC_EXPLICIT:
        nz_ = [v for v in ks if v != 0] or [0.0]
        cf_ = min(fc._best_precision_fl(v, 1, w) for v in nz_)
        if False and cf_ > w:   # The state type below covers these fractions.
            # recordings (after the retraction): [-0.25 0.1] -- 0.25 fits at w + 1 fraction bits, the
            # inherited coefficient fraction then exceeds the word (as below 0.25): the recordings have w - 1 / w only
            _refuse(where + ".Coefficients", "Discrete FIR 'Lattice MA' with the library types and coefficients whose "
                    "best-precision fraction length (%d) exceeds the input word (%d): not recorded (the recorded "
                    "sets need %d or %d)" % (cf_, w, w - 1, w))
    if given == REC_EXPLICIT:
        tc, tp, ta, to = (_parse(REC_EXPLICIT[k], where + "." + k) for k in TYPE_KEYS)
    else:
        nz = [v for v in ks if v != 0]
        cf = min(fc._best_precision_fl(v, 1, w) for v in nz) if nz else None
        # recordings (host): the lattice's state / product type is a 32-bit word (64 when the
        # input word doubled exceeds 32) with the coefficient + input fraction, but never fewer integer bits than the
        # input's (s16En8 with |k| 0.11: sfix32_En24, not En26; all-zero coefficients: W - input integer bits)
        W_ = 32 if 2 * w <= 32 else 64
        F_ = W_ - (w - f) if cf is None else min(cf + f, W_ - (w - f))
        tc = (1, w, cf if cf is not None else w - 1)
        # the products wrap at the host's product word, not the state word (recordings /
        # 3.1: 'products wrap at the state word W' refuted by 92 recorded designs; equal for 12 / 16-bit inputs)
        tp = (1, W_ if cf is None else host_product_word(w), F_)
        ta, to = (1, W_, F_), (s, w, f)
    return _build(spec, n, ks, s, w, f, tc, tp, ta, to, {"RndMeth": "Floor", "SaturateOnIntegerOverflow": "off"})


def _unsigned_explicit(b, t_in, where):
    """An unsigned input with the coefficient, product and accumulator types written signed and the output left
    'Same as input' (so unsigned): Simulink stops with Simulink:blocks:DiscreteFirAllDataTypeMustBeSigned
    (default diagnostics). With a signed written output it runs
    (recordings u16En8, answered below); other unsigned shapes are refused there."""
    import fxgraph_lib as G
    txt = {k: str(b.get(k, DEFAULTS[k])).strip().replace(" ", "") for k in TYPE_KEYS}
    mode = str((getattr(_HW, "v", None) or {}).get("ParameterOverflowMsg", G.PARAM_OVERFLOW_DEFAULT)).strip()
    if all(txt[k].startswith("fixdt(1,") for k in ("CoefDataTypeStr", "ProductDataTypeStr", "AccumDataTypeStr")) and \
            txt["OutDataTypeStr"] == DEFAULTS["OutDataTypeStr"].replace(" ", "") and mode == "error":
        _refuse(where + ".OutDataTypeStr", "Discrete FIR 'Lattice MA' on the unsigned input %s with signed written "
                "coefficient / product / accumulator types and the output 'Inherit: Same as input' (unsigned): "
                "Simulink stops with Simulink:blocks:DiscreteFirAllDataTypeMustBeSigned; write a signed output type "
                "or give the input a signed type" % _fix(tuple(t_in)), "PORT_TYPE_INVALID")


def _unsigned_lib(b, t_in, where):
    """An unsigned input with the library types (recordings): the
    coefficients take an unsigned type of the input's word, so a negative coefficient of one LSB or more below zero
    stops Simulink under its default ParameterOverflowMsg (SimulinkFixedPoint:util:fxpParameterOverflow). The LSB is
    taken at the smallest fraction the vector can get (the signed best precision of its largest magnitude; the
    unsigned one is never smaller), so the error is certain; anything else is refused as not modelled (with
    non-negative coefficients Simulink runs it on unsigned coefficient / accumulator types, G2N_08 / 09)."""
    import fxcore as fc
    import fxgraph_lib as G
    ks = _coefs(b, where)
    if ks is None:
        return   # (a column, NaN or unreadable coefficients: the checks below say what Simulink does)
    mode = str((getattr(_HW, "v", None) or {}).get("ParameterOverflowMsg", G.PARAM_OVERFLOW_DEFAULT)).strip()
    if ks and mode == "error" and any(ks):
        w = t_in[1]
        cf_lo = fc._best_precision_fl(max(abs(v) for v in ks), 1, w)
        neg = [v for v in ks if v < 0 and Fraction(v) * Fraction(2) ** cf_lo <= -1]
        if neg:
            _refuse(where + ".Coefficients", "Discrete FIR 'Lattice MA' on the unsigned input %s with the library "
                    "types: the coefficient %s does not fit the unsigned coefficient type (Simulink stops with "
                    "SimulinkFixedPoint:util:fxpParameterOverflow under its default diagnostics); give the input a "
                    "signed type or write the coefficient type" % (_fix(tuple(t_in)), "%g" % neg[0]),
                    "PORT_TYPE_INVALID")
    _refuse(where, ("Discrete FIR 'Lattice MA' on the unsigned input %s with the library "
        'types: not recorded as an opened shape (Simulink runs non-negative '
        'coefficients on unsigned coefficient / accumulator types; not modelled here)') % _fix(tuple(t_in)))


def _coefs(b, where):
    import fxnum
    raw = str(b.get("Coefficients", "")).strip()
    if ";" in raw or "'" in raw:
        return None
    c = fxnum.canonical(raw, "vector")
    if isinstance(c, fxnum.NumError):
        return None
    try:
        ks = [float(v) for v in str(c).strip("[]").replace(",", " ").split()]
    except ValueError:
        return None
    if not ks or any(v != v or v in (float("inf"), float("-inf")) for v in ks):
        return None
    return ks


def _plain(b):
    """InitialStates 0, sample based, the block's SampleTime -1, coefficients from the dialog (every recording of
    the declared domain)"""
    return str(b.get("InitialStates", "0")).strip() in ("0", "0.0", "[0]") and \
        str(b.get("InputProcessing", "Elements as channels (sample based)")).strip() == \
        "Elements as channels (sample based)" and str(b.get("SampleTime", "-1")).strip() in ("-1", "") and \
        str(b.get("CoefSource", "Dialog parameters")).strip() in ("Dialog parameters", "Dialog")


def _in_domain(b, t_in, where):
    """(lowering, None) when the block lies in the declared library-type domain of engine/lattice_domain.json, else
    (None, the reason; None when the block does not have the library types): a signed input sfix(w, f) with
    0 <= f <= w - 1 (the rule's integer run does not depend on f), the input word recorded on this hardware, 1 .. M
    coefficients, the coefficient integer bits Ic = w - cf and every |k| within what was recorded on (hardware, w),
    the (RndMeth, saturation) pair and the output mode recorded there, and no product cast dropping more than 32 bits
    under a rounding to nearest"""
    import fxcore as fc
    om = OUT_MODES.get(str(b.get("OutDataTypeStr", DEFAULTS["OutDataTypeStr"])).strip())
    if any(str(b.get(k, DEFAULTS[k])).strip() != DEFAULTS[k] for k in TYPE_KEYS if k != "OutDataTypeStr"):
        return None, None
    if om is None:
        return None, "OutDataTypeStr %r with the library's coefficient / product / accumulator types: recorded with " \
            "'Inherit: Same as input' / 'Inherit: Same as accumulator' only" % str(b.get("OutDataTypeStr"))
    if not isinstance(t_in, tuple) or len(t_in) != 3 or t_in[0] != 1 or not _plain(b):
        return None, None
    ks = _coefs(b, where)
    if ks is None:
        return None, None
    asic = bool(fc._asic(getattr(_HW, "v", None) or {}))
    hw = "ASIC / FPGA" if asic else "the host"
    ty = stage_types(t_in[1], t_in[2], ks, asic)
    if ty is None:
        return None, "all-zero coefficients: the stage rule's types do not cover them"
    w, f, M, cf = t_in[1], t_in[2], len(ks), ty[0][2]
    rm = str(b.get("RndMeth", "Floor")).strip()
    sat = str(b.get("SaturateOnIntegerOverflow", "off")).strip()
    a = lattice_domain()["lib"].get("asic" if asic else "host", {}).get(str(w))
    why = None
    if a is None:
        why = "input word %d not recorded on %s (recorded: %s)" % (w, hw, ", ".join(
            lattice_domain()["lib"].get("asic" if asic else "host", {})))
    elif not 0 <= f <= w - 1:
        why = "input fraction %d outside 0 .. %d" % (f, w - 1)
    elif M > a["M"]:
        why = "%d coefficients: 1 .. %d recorded on %d-bit inputs on %s" % (M, a["M"], w, hw)
    elif not a["Ic"][0] <= w - cf <= a["Ic"][1]:
        why = "coefficient integer bits %d (best-precision fraction %d at word %d): %d .. %d recorded on %d-bit " \
            "inputs on %s" % (w - cf, cf, w, a["Ic"][0], a["Ic"][1], w, hw)
    elif asic and not any(c[0] == [w, f, M, cf, rm, sat, om] and max(abs(Fraction(v)) for v in ks) <= Fraction(c[1])
                          for c in a.get("cells", [])):
        # recordings (ASIC, library types, 12 coefficients, sfix10_En8,
        # 1067 / 2048 steps 1 LSB low, inside the M / Ic / |k| range but in no recorded cell): on ASIC / FPGA only the
        # recorded cells (w, f, M, cf, RndMeth, saturation, output mode) with each cell's largest |k| are answered
        why = "library-default types on %s: the cell (input sfix%d_En%d, %d coefficients, coefficient fraction %d, %s / " \
            "saturation %s) with these coefficients is not recorded there (the range rule is answered on the host " \
            "only); write the coefficient, product and accumulator types, or record this cell" % (hw, w, f, M, cf, rm, sat)
    elif max(abs(Fraction(v)) for v in ks) > Fraction(a["kmax"]):
        why = "largest |k| %s beyond the recorded %s on %d-bit inputs on %s" % (
            "%g" % max(abs(v) for v in ks), a["kmax"], w, hw)
    elif [rm, sat] not in a["rs"]:
        why = "RndMeth %s with saturation %s not recorded on %d-bit inputs on %s (recorded: %s)" % (
            rm, sat, w, hw, ", ".join("%s / %s" % tuple(x) for x in a["rs"]))
    elif om not in a["out"]:
        why = "output 'Same as %s' not recorded on %d-bit inputs on %s" % (
            "accumulator" if om == "acc" else "input", w, hw)
    elif rm in NONDIRECTED and cf + ty[2][2] - ty[1][2] > 32:
        why = _drop_why(cf + ty[2][2] - ty[1][2], rm)
    if why:
        return None, why
    return dict(ks=ks, types=ty, out=om), None


def _drop_why(drop, rm):
    # one exact rounding differs from Simulink by one LSB; the readings that
    # match it are not separated by the recordings
    return "the product cast drops %d bits (more than 32) under RndMeth %s: Simulink's rounding of such products is " \
        "not separated by the recordings" % (drop, rm)


def _explicit_types(b, t_in, ks):
    """(coefficient, product, accumulator, output) of written types: all of coefficient / product / accumulator
    written ('full'), or only the accumulator ('acc_only': coefficients at the best precision of the input word,
    product (2w, f + cf) -- recordings); (group, types) or (None, None)"""
    import fxcore as fc
    w, f = t_in[1], t_in[2]
    txt = {k: str(b.get(k, DEFAULTS[k])).strip() for k in TYPE_KEYS}
    fx = {k: re.fullmatch(r"fixdt\(\s*([01])\s*,\s*(\d+)\s*,\s*(-?\d+)\s*\)", v) for k, v in txt.items()}
    fx = {k: (int(m.group(1)), int(m.group(2)), int(m.group(3))) if m else None for k, m in fx.items()}
    if any(txt[k] != DEFAULTS[k] and fx[k] is None for k in TYPE_KEYS if k != "OutDataTypeStr"):
        return None, None
    ta = fx["AccumDataTypeStr"]
    if ta is None:
        return None, None
    if fx["CoefDataTypeStr"] and fx["ProductDataTypeStr"]:
        grp, tc, tp = "full", fx["CoefDataTypeStr"], fx["ProductDataTypeStr"]
    elif fx["CoefDataTypeStr"] is None and fx["ProductDataTypeStr"] is None:
        nz = [v for v in ks if v != 0]
        if not nz:
            return None, None
        cf = min(fc._best_precision_fl(v, 1, w) for v in nz)
        grp, tc, tp = "acc_only", (1, w, cf), (1, 2 * w, f + cf)
    else:
        return None, None
    o = txt["OutDataTypeStr"]
    if o in OUT_MODES:
        om, to = OUT_MODES[o], ((1, w, f) if OUT_MODES[o] == "input" else ta)
    elif fx["OutDataTypeStr"]:
        om, to = "written", fx["OutDataTypeStr"]
    else:
        return None, None
    return grp, (tc, tp, ta, to, om)


def _in_explicit(b, t_in, where):
    """(lowering, None) when a block with written types lies in the declared domain (engine/lattice_domain.json
    "explicit"), else (None, reason; None when the block has the library types): signed types, the input word,
    hardware, (RndMeth, saturation) and output mode as recorded in the group, the coefficient count, the word and
    integer bits of every type and the bits each cast drops (input into the accumulator, product, product into the
    accumulator, output) within their recorded ranges, every coefficient fitting its type, no product cast dropping
    more than 32 bits under a rounding to nearest"""
    import fxcore as fc
    if _lib_types(b) or not isinstance(t_in, tuple) or len(t_in) != 3 or not _plain(b):
        return None, None
    ks = _coefs(b, where)
    if ks is None:
        return None, None
    grp, ty = _explicit_types(b, t_in, ks)
    if grp is None:
        return None, "these written types (%s): recorded with coefficient, product and accumulator types all " \
            "written, or the accumulator type alone" % ", ".join(
                "%s %s" % (k, str(b[k]).strip()) for k in TYPE_KEYS if k in b and str(b[k]).strip() != DEFAULTS[k])
    if t_in[0] != 1:
        return None, None
    tc, tp, ta, to, om = ty
    g = lattice_domain()["explicit"].get(grp)
    asic = bool(fc._asic(getattr(_HW, "v", None) or {}))
    rm = str(b.get("RndMeth", "Floor")).strip()
    sat = str(b.get("SaturateOnIntegerOverflow", "off")).strip()
    w, f, M = t_in[1], t_in[2], len(ks)
    q = {"M": M, "AW": ta[1], "ai": ta[1] - ta[2], "s_in": f - ta[2], "s_p": tc[2] + ta[2] - tp[2],
         "s_pa": tp[2] - ta[2], "s_out": ta[2] - to[2]}
    if grp == "full":
        q.update(cw=tc[1], ci=tc[1] - tc[2], PW=tp[1], pi=tp[1] - tp[2])
    else:
        q["Ic"] = w - tc[2]
    if om == "written":
        q.update(ow=to[1], oi=to[1] - to[2])
    names = {"M": "coefficient count", "AW": "accumulator word", "ai": "accumulator integer bits",
             "s_in": "bits dropped casting the input into the accumulator", "s_p": "bits dropped by the product cast",
             "s_pa": "bits dropped casting the product into the accumulator",
             "s_out": "bits dropped by the output cast", "cw": "coefficient word", "ci": "coefficient integer bits",
             "PW": "product word", "pi": "product integer bits", "Ic": "coefficient integer bits",
             "ow": "output word", "oi": "output integer bits"}
    why = None
    if not ks or not any(ks):
        why = "all-zero coefficients"
    elif any(t[0] != 1 for t in (tc, tp, ta, to)):
        why = "an unsigned written type"
    elif g is None:
        why = "group not recorded"
    elif w not in g["words"]:
        why = "input word %d (recorded: %s)" % (w, ", ".join(map(str, g["words"])))
    elif ("asic" if asic else "host") not in g["hw"]:
        why = "this hardware"
    elif [rm, sat] not in g["rs"]:
        why = "RndMeth %s with saturation %s (recorded: %s)" % (rm, sat, ", ".join("%s / %s" % tuple(x)
                                                                                  for x in g["rs"]))
    elif om not in g["out"]:
        why = "output %s (recorded: %s)" % (str(b.get("OutDataTypeStr")).strip(), ", ".join(g["out"]))
    else:
        for k, v in q.items():
            lo, hi = g["ranges"][k]
            if not lo <= v <= hi:
                why = "%s %d outside the recorded %d .. %d" % (names[k], v, lo, hi)
                break
    if why is None and max(abs(Fraction(v)) for v in ks) > Fraction(g["kmax"]):
        why = "largest |k| %g beyond the recorded %s" % (max(abs(v) for v in ks), g["kmax"])
    if why is None:
        lim = Fraction(2) ** (tc[1] - 1)
        for v in ks:
            t = Fraction(v) * Fraction(2) ** tc[2]
            n = (t.numerator * 2 + t.denominator) // (2 * t.denominator)
            if not -lim <= n < lim:
                why = "the coefficient %g does not fit %s (Simulink's parameter overflow is not recorded here)" % (
                    v, _fix(tc))
                break
    if why is None and rm in NONDIRECTED and q["s_p"] > 32:
        why = _drop_why(q["s_p"], rm)
    if why:
        return None, "with written types (%s group): %s" % ("all three" if grp == "full" else "accumulator only", why)
    return dict(ks=ks, types=(tc, tp, ta), out=to), None


def _lower_stage(spec, n, t_in, dom_, rm, sat):
    """recordings on a recorded cell: products in P, sums and states in A, the
    block's RndMeth and saturation on every cast, the output in the input type or A"""
    tc, tp, ta = dom_["types"]
    to = tuple(t_in) if dom_["out"] == "input" else ta
    return _build(spec, n, dom_["ks"], 1, t_in[1], t_in[2], tc, tp, ta, to,
                  {"RndMeth": rm, "SaturateOnIntegerOverflow": sat})


def _build(spec, n, ks, s, w, f, tc, tp, ta, to, R, cast_in=False):
    where = "design.blocks." + n
    blocks = dict(spec["blocks"])
    del blocks[n]
    lines = [dict(ln) for ln in spec.get("lines", [])]
    src = [ln for ln in lines if ln.get("dst") == n]
    if len(src) != 1:
        _refuse(where, "Discrete FIR takes exactly one input", "BAD_CONNECTION")
    lines = [ln for ln in lines if ln.get("dst") != n]
    X = INTERNAL + n + "/"

    def gain(nm, k):
        blocks[nm] = dict(R, type="Gain", Gain=repr(k), ParamDataTypeStr=_fix(tc), OutDataTypeStr=_fix(tp))

    def add(nm, a_, b_):
        blocks[nm] = dict(R, type="Sum", Inputs="++", AccumDataTypeStr=_fix(ta), OutDataTypeStr=_fix(ta))
        lines.append({"src": a_[0], "sport": a_[1], "dst": nm, "dport": 1})
        lines.append({"src": b_[0], "sport": b_[1], "dst": nm, "dport": 2})

    u = (src[0]["src"], int(src[0].get("sport", 1)))
    if not s:
        # u16En8: Simulink takes an unsigned input as the signed type of its
        # word and fraction, wrapping (ufix16_En8 65535 acts as -1 LSB: y -256 then -34 ..., every step equal)
        blocks[X + "in"] = dict(R, type="DataTypeConversion", OutDataTypeStr=_fix((1, w, f)))
        lines.append({"src": u[0], "sport": u[1], "dst": X + "in", "dport": 1})
        u = (X + "in", 1)
    elif cast_in:
        # written types: the input is cast into the accumulator first (f_0 = g_0 = cast_A(u), the block's RndMeth and
        # saturation; recordings) -- with the library types that cast is exact
        blocks[X + "in"] = dict(R, type="DataTypeConversion", OutDataTypeStr=_fix(ta))
        lines.append({"src": u[0], "sport": u[1], "dst": X + "in", "dport": 1})
        u = (X + "in", 1)
    f_, g_ = u, u
    for m, k in enumerate(ks, start=1):
        z = X + "z%d" % m
        blocks[z] = {"type": "UnitDelay", "InitialCondition": "0"}
        lines.append({"src": g_[0], "sport": g_[1], "dst": z, "dport": 1})
        gk, fk = X + "kg%d" % m, X + "kf%d" % m
        gain(gk, k)
        lines.append({"src": z, "sport": 1, "dst": gk, "dport": 1})
        gain(fk, k)
        lines.append({"src": f_[0], "sport": f_[1], "dst": fk, "dport": 1})
        fn, gn = X + "f%d" % m, X + "g%d" % m
        add(fn, f_, (gk, 1))
        add(gn, (fk, 1), (z, 1))
        f_, g_ = (fn, 1), (gn, 1)
    blocks[n] = dict(R, type="DataTypeConversion", OutDataTypeStr=_fix(to))
    lines.append({"src": f_[0], "sport": f_[1], "dst": n, "dport": 1})
    lines = [{k: v for k, v in ln.items() if not (k == "sport" and v == 1)} for ln in lines]
    # recordings (parameter_quantization): each reflection coefficient is quantised into tc by its two internal Gains, whose
    # notes carry internal names; it is reported once under the user's location, Coefficients[j] (a scalar unindexed)
    import fxgraph_lib as G
    import parameter_bounds
    notes = dict(spec.get(parameter_bounds.NOTES_KEY) or {})
    for j, k in enumerate(ks):
        G._param_note(k, tuple(tc), G._elem_where(where + ".Coefficients", j, len(ks)), notes)
    out = dict(spec, blocks=blocks, lines=lines)
    if notes:
        out[parameter_bounds.NOTES_KEY] = notes
    return out


def _lib_types(b):
    """every type parameter left out or written as the library's default (the export writes them out)"""
    return all(str(b[k]).strip() == DEFAULTS[k] for k in TYPE_KEYS if k in b)


class _HW:
    v = None


def typed(spec, probe, hw=None):
    """spec with every Lattice MA Discrete FIR lowered (its input type from probe(spec, [(block, 1)]))."""
    _HW.v = hw
    while has(spec):
        n = next(x for x, b in spec["blocks"].items() if _is(b))
        pr = probe(spec, [(n, 1)])
        if pr is None:
            # the probe also returns None without a loop (an input whose type the engine works out only
            # later: an unsigned input behind a port-IC Delay, a counter chain without an Inport, a library-default
            # Constant into a Sum) -- the reason says what is unknown, not that there is a loop
            _refuse("design.blocks." + n, "Discrete FIR 'Lattice MA' whose input type is not known before the block "
                    "is lowered (it depends on a loop through it, or on types the engine resolves only later): not "
                    "supported here; give the input a fixed type upstream (a Data Type Conversion)")
        spec = _lower_one(spec, n, tuple(pr[0]) if isinstance(pr[0], (tuple, list)) else pr[0])
    return spec

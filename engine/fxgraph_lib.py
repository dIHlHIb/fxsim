"""fxgraph: bit-exact simulation of general discrete fixed-step fixed-point block diagrams .

A design is a set of blocks plus lines (src block/port -> dst block/port). Unit Delays break
algebraic dependencies, so any directed graph is allowed as long as every cycle passes through a
Unit Delay. Simulation is two-phase:
  compile  -- resolve every output data type (explicit types, inheritance rules), quantise block
              parameters and build a per-block integer kernel; done once per design;
  run      -- for every time step evaluate the blocks in schedule order on stored integers, then
              update Unit Delay states.
Block arithmetic reuses the rules in ref/fxcore.py (validated against recordings);
the regression check re-checks every block recordings case and both chain recordings through this
engine. Rules without recording coverage are listed in docs/en/GRAPH.md ("unverified").
"""
import json
import os
import re
from fractions import Fraction
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "ref"))
import fxcore as fc  # noqa: E402
import fxlutplan as LP  # noqa: E402 -- compiled lookup records

import functools
from blockspec import DEFAULTS, JSON_LIBRARY_DEFAULTS, JSON_LIBRARY_UNSUPPORTED, JSON_LIBRARY_UNRECORDED, LIB_DIALOG_PARAMS, RECORDED_VALID_PARAMS, REQUIRED_LIBRARY_TEXT, META_VALUES_BY_TYPE, REQUIRED, SEMANTIC_EXTRA, META_ANY, META_VALUES, NUMERIC_VALUES, PORT_COUNTS, _UNUSED_DIM2, _UNUSED_DIM  # noqa: E402,F401 -- the block-support registry
import fxnum  # noqa: E402  (the one numeric-parameter parser)

# pure helpers from ref/fxcore.py, memoised (a search compiles the same parameters thousands of times)
_quant_param = functools.lru_cache(maxsize=1 << 16)(fc._quant_param)


def _param_overflow(v, t, where, what, mode="error", notes=None, note_where=None):
    """One check for every parameter quantised into a fixed-point type (the error reporting contract: what
    Simulink stops with fxpParameterOverflow is refused, no longer saturated per the old recordings convention).
    recordings (Gain parameter, Compare To Constant, Switch threshold, FIR coefficients and initial
    states, Rate Transition / Downsample / Upsample initial values, Saturation limits) and recordings (delays, Detect
    vinit, integrator initial value and limits) draw one line: a value one LSB or more below the type's minimum, or
    more than one LSB above its maximum, stops Simulink under its default diagnostics; in between it saturates to the
    bound. mode is the model's ParameterOverflowMsg (hw): only 'error' (the default) stops; 'warning' / 'none'
    saturate, as the engine always did. note_where: the parameter_quantization key when it is not where (element j of a
    list parameter, recordings; the refusal keeps naming the parameter)."""
    _param_note(v, t, note_where or where, notes)
    if mode != "error":
        return
    s_, w_, f_ = t
    x = Fraction(v) * Fraction(2) ** f_   # unrounded, in LSBs
    lo, hi = (-(1 << (w_ - 1)), (1 << (w_ - 1)) - 1) if s_ else (0, (1 << w_) - 1)
    if x <= lo - 1 or x > hi + 1:
        rng = "[%s, %s]" % (_fmt_num(Fraction(lo) / 2 ** f_), _fmt_num(Fraction(hi) / 2 ** f_))
        raise GraphError("PORT_TYPE_INVALID", where, "%s %s does not fit %s, range %s (Simulink stops with "
                         "fxpParameterOverflow under its default diagnostics; up to one LSB above the maximum it "
                         "saturates); write a value within the range or use a wider type, or set "
                         "hw.ParameterOverflowMsg 'warning' / 'none' to saturate it as Simulink then does"
                         % (what, _fmt_num(v), _tstr(tuple(t)), rng))


# Simulink's default ParameterOverflowMsg. Test harnesses that replay recordings made with 'none' (or random-design
# generators that test other properties) may set it in-process; the command-line tools never change it.
PARAM_OVERFLOW_DEFAULT = "error"


def saturating_params(req, tool="fxgraph"):
    """A request for fxgraph / fxcodegen / fxchain with hw.ParameterOverflowMsg 'none' (see PARAM_OVERFLOW_DEFAULT)."""
    if tool not in ("fxgraph", "fxcodegen", "fxchain") or not isinstance(req, dict) or "catalogue" in req:
        return req
    return dict(req, hw=dict(req.get("hw") or {}, ParameterOverflowMsg="none"))


def golden_hw(hw):
    """hw for replaying a recording that ran: the recorders set ParameterOverflowMsg 'none' (34 recorder
    scripts write it; a recording with values for an overflowing parameter proves it for the rest -- under the
    default 'error' Simulink stops). The product default stays 'error'."""
    return dict(hw or {}, ParameterOverflowMsg="none")


def _param_note(v, t, where, notes):
    """the error reporting contract (complete information): a parameter whose value the engine does not use as written --
    rounded to its type's precision (Simulink's ParameterPrecisionLossMsg, default warning) or saturated to the
    type's bound (within the one LSB Simulink accepts, or under ParameterOverflowMsg warning / none). notes maps the
    parameter's location to what was written, the type, the value used and why."""
    if notes is None or not isinstance(t, tuple):
        return
    try:
        x = Fraction(v)
    except (TypeError, ValueError):
        return
    s_, w_, f_ = t
    q = Fraction(fc._quant_param(float(x), s_, w_, f_)) / Fraction(2) ** f_
    if q == x:
        return
    lo, hi = (-(1 << (w_ - 1)), (1 << (w_ - 1)) - 1) if s_ else (0, (1 << w_) - 1)
    out = x * Fraction(2) ** f_ < lo or x * Fraction(2) ** f_ > hi
    notes[where] = {"written": _fmt_num(x), "type": _tstr(tuple(t)), "used": _fmt_num(q),
                    "why": "saturated to the type's bound" if out else "rounded to the type's precision"}


def _elem_where(where, j, n):
    """The location of element j (0-based) of a list parameter of n elements: where[j + 1]; a scalar: where (
    every coefficient of a vector wrote the same key, so only the last quantised one was reported)."""
    return where if n == 1 else "%s[%d]" % (where, j + 1)


def _fmt_num(v):
    v = Fraction(v)
    return str(v.numerator) if v.denominator == 1 else repr(float(v))


_best_precision_fl = functools.lru_cache(maxsize=1 << 16)(fc._best_precision_fl)
_parse_fixdt = functools.lru_cache(maxsize=1 << 16)(fc._parse_fixdt)

BOOL = "bool"  # marker type for logical signals (Relational Operator output)
MAX_BLOCKS = 100000
MAX_STEPS = 10_000_000
MAX_WL = 4096  # Python path; the C core takes 126 bits (vm_ok)

# block types evaluated through the reference functions of ref/fxcore.py (op name of recordings)
FXCORE_OPS = {"Constant": "blk_constant", "ShiftArithmetic": "blk_shift", "LogicalOperator": "blk_logic",
              "CompareToConstant": "blk_cmpconst", "Switch": "blk_switch", "MultiportSwitch": "blk_mpswitch",
              "DiscreteFir": "blk_dfir2",
              "CounterLimited": "blk_counter", "CounterFreeRunning": "blk_counter", "DetectChange": "blk_detect",
              "DetectIncrease": "blk_detect", "DetectDecrease": "blk_detect", "Signum": "blk_signround",
              "Rounding": "blk_signround", "BitwiseOperator": "blk_bitwise", "ExtractBits": "blk_extract",
              "DiscreteIntegrator": "blk_dti", "Lookup1D": "blk_lut1d", "Sqrt": "blk_sqrt", "CompareToZero": "blk_cmpzero",
              "DeadZone": "blk_deadzone", "WrapToZero": "blk_wraptozero", "RateLimiter": "blk_ratelimit",
              "Trigonometry": "blk_trig", "DelayPort": "blk_delayport",
              "DTIStateFB": "blk_dtistate", "DTIStateFBGain": "blk_dtistategain",
              "Lookup2D": "blk_lut2d", "LookupND": "blk_lutnd", "Prelookup": "blk_prelookup_k",
              "PrelookupF": "blk_prelookup_f", "InterpolationUsingPrelookup": "blk_interp_pl"}
_NUMLIT = re.compile(r"^\s*[-+]?(\d+(\.\d*)?|\.\d+)([eE][-+]?\d+)?\s*$")


_MATGAIN_DOMAIN = None


_MSH_DOMAIN = None
_MATRIX_DOMAIN = None


def _matrix_domain():
    """engine/matrix_domain.json (the regression check): the exact matrix-signal tuples recordings
    recordings settles -- (block, shape, parameter shapes, collapse) with the input types, the output
    type and the hardware Simulink ran it with."""
    global _MATRIX_DOMAIN
    if _MATRIX_DOMAIN is None:
        p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "matrix_domain.json")
        _MATRIX_DOMAIN = set(json.load(open(p))) if os.path.exists(p) else set()
    return _MATRIX_DOMAIN


def matrix_key(mkey, in_types, out_type, asic):
    """The support-domain key of a lowered matrix element (shared with the regression check)."""
    def tn(t):
        return "boolean" if t == BOOL else list(t) if isinstance(t, (tuple, list)) else str(t)
    return json.dumps([json.loads(mkey) if isinstance(mkey, str) else mkey,
                       sorted({json.dumps(tn(t)) for t in in_types}), tn(out_type), bool(asic)], sort_keys=True)


def _msh_domain():
    """engine/msh_domain.json (the regression check): the exact Multiport Switch 'Specify indices' and port-shift
    Shift Arithmetic tuples recordings settles, each with its recorded index / shift range."""
    global _MSH_DOMAIN
    if _MSH_DOMAIN is None:
        p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "msh_domain.json")
        _MSH_DOMAIN = json.load(open(p)) if os.path.exists(p) else {"MS": {}, "SH": {}}
    return _MSH_DOMAIN


_LUTPORT_DOMAIN = None


def _lutport_domain():
    """engine/lutport_domain.json (the regression check): the exact 1-D Lookup Table settings with the table /
    breakpoints from ports that recordings show -- the support domain of that opening."""
    global _LUTPORT_DOMAIN
    if _LUTPORT_DOMAIN is None:
        p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lutport_domain.json")
        _LUTPORT_DOMAIN = set(json.dumps(t) for t in json.load(open(p))) if os.path.exists(p) else set()
    return _LUTPORT_DOMAIN


_DELAYEN_DOMAIN = None


def _delay_enable_reset(p):
    """A Delay with an enable port, or with an external reset and its length from the dialog (the port-length Delay
    with a reset is checked against engine/delaymode_domain.json instead)."""
    return str(p.get("ShowEnablePort", "off")).strip() == "on" or (
        str(p.get("ExternalReset", "None")).strip() not in ("None", "none") and
        str(p.get("DelayLengthSource", "Dialog")) != "Input port")


def _delayen_domain():
    """engine/delayen_domain.json (the regression check): the enabled Delay's recorded domain."""
    global _DELAYEN_DOMAIN
    if _DELAYEN_DOMAIN is None:
        p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "delayen_domain.json")
        _DELAYEN_DOMAIN = json.load(open(p)) if os.path.exists(p) else {"enable_types": [], "reset_types": [],
                                                                         "modes": [], "delay_length": [0, -1],
                                                                         "asic": [], "data_wl": [0, -1]}
    return _DELAYEN_DOMAIN


def _matgain_domain():
    """engine/matgain_domain.json (the regression check): the exact Gain Matrix(K*u) tuples recordings
    recorded in Simulink -- the support domain of that opening."""
    global _MATGAIN_DOMAIN
    if _MATGAIN_DOMAIN is None:
        import json as _json
        p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "matgain_domain.json")
        _MATGAIN_DOMAIN = {_json.dumps(t) for t in _json.load(open(p))} if os.path.exists(p) else set()
    return _MATGAIN_DOMAIN


_DELAYPORT_DOMAIN = None


def _delayport_domain():
    """engine/delayport_domain.json (the regression check): the exact port-sourced Delay tuples
    recorded in Simulink."""
    global _DELAYPORT_DOMAIN
    if _DELAYPORT_DOMAIN is None:
        p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "delayport_domain.json")
        _DELAYPORT_DOMAIN = {json.dumps(t) for t in json.load(open(p))} if os.path.exists(p) else set()
    return _DELAYPORT_DOMAIN


_SUMBOOL3_DOMAIN = None
_DELAYMODE_DOMAIN = None


def _delaymode_domain():
    """engine/delaymode_domain.json (the regression check): the exact port-length Delay tuples with
    PreventDirectFeedthrough 'on' or an external reset that recordings show Simulink
    running."""
    global _DELAYMODE_DOMAIN
    if _DELAYMODE_DOMAIN is None:
        p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "delaymode_domain.json")
        _DELAYMODE_DOMAIN = {json.dumps(t) for t in json.load(open(p))} if os.path.exists(p) else set()
    return _DELAYMODE_DOMAIN


_LU_DOMAIN = None


def _lu_recorded(p, ts, dims, it, tt, asic):
    """A falling segment on an unsigned intermediate type inside the recorded span (engine/lu_domain.json,
    engine/make_lu_domain.py; recordings): the explicit fraction type fixdt(0,16,15), inputs of
    8..16 bits, Clip, and a recorded [dimensions, table-to-intermediate relation, intermediate word, ASIC, saturation]."""
    global _LU_DOMAIN
    if _LU_DOMAIN is None:
        pth = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lu_domain.json")
        _LU_DOMAIN = json.load(open(pth)) if os.path.exists(pth) else []
    if str(p.get("FractionDataTypeStr", "")).strip() != "fixdt(0,16,15)" or \
            str(p.get("ExtrapMethod", "Clip")).strip() != "Clip":
        return False
    if any(not isinstance(t, (tuple, list)) or not (8 <= t[1] <= 16) for t in ts[:dims]):
        return False
    if not (isinstance(it, tuple) and isinstance(tt, tuple)) or it[1] not in (8, 16, 32):
        return False
    import make_lu_domain as _mlu
    return _mlu.key(dims, it, tt, asic, str(p.get("SaturateOnIntegerOverflow", "off")).strip()) in _LU_DOMAIN


def _sumbool3_in_range(ts, p, asic):
    """The three-operand Sum with one boolean operand inside the ranges the recorded tuples span (recorded item d:
    generalised over recorded ranges, not tuples; recordings):
    on the host, one boolean at a recorded position; a recorded Inputs sign string; a signed fixed-point operand of a
    word length within the recorded span with a fraction 0 .. wl-1; an unsigned operand only of a recorded type (that
    axis was not varied); the internal-rule output, an explicit signed fixdt within the recorded word-length span
    with fraction 0 .. wl-1, or an unsigned output only of a recorded type (one recorded); a recorded saturation
    setting. On ASIC / FPGA only the exact recorded tuples (one design recorded there)."""
    dom = _sumbool3_domain()
    if not dom:
        return False
    sig = ["bool" if t_ == BOOL else (list(t_) if isinstance(t_, (tuple, list)) else None) for t_ in ts]
    if sig.count("bool") != 1 or None in sig:
        return False
    ot = str(p.get("OutDataTypeStr", fc._INHERIT_RULE)).strip()
    sat = str(p.get("SaturateOnIntegerOverflow", "off")).strip()
    inp = str(p.get("Inputs", "")).strip()
    # recordings (recordings side note): RndMeth and AccumDataTypeStr are checked against the recorded values
    # -- Floor and Nearest rounding with an internal-rule accumulator
    # (RndMeth Zero / Convergent / Ceiling, accumulators fixdt(1,32,16) and fixdt(1,12,4); all equal with the existing
    # rule). 'Inherit: Same as first input' with a boolean first operand is Simulink's PropForwardDataTypeError
    rnd_, acc_ = str(p.get("RndMeth", "Floor")).strip(), str(p.get("AccumDataTypeStr", fc._INHERIT_RULE)).strip()
    if rnd_ not in ("Floor", "Nearest", "Zero", "Convergent", "Ceiling") or \
            acc_ not in (fc._INHERIT_RULE, "fixdt(1,32,16)", "fixdt(1,12,4)"):
        return False
    if acc_ != fc._INHERIT_RULE and rnd_ != "Floor":
        # native tests: the explicit accumulators were
        # recorded with Floor only; another rounding with them is a combination not recorded
        return False
    if asic:
        return json.dumps([sig, inp, ot, sat, True]) in dom
    tup = [x for x in (json.loads(t) for t in dom) if not x[4]]
    if not tup or sig.index("bool") not in {x[0].index("bool") for x in tup}:
        return False
    if inp not in {x[1] for x in tup} or sat not in {x[3] for x in tup}:
        return False
    fx = [tuple(t_) for x in tup for t_ in x[0] if t_ != "bool"]
    sw = [t_[1] for t_ in fx if t_[0] == 1]
    us = {t_ for t_ in fx if t_[0] == 0}
    for t_ in sig:
        if t_ == "bool":
            continue
        t_ = tuple(t_)
        if t_[0] == 1:
            if not (sw and min(sw) <= t_[1] <= max(sw) and 0 <= t_[2] <= t_[1] - 1):
                return False
        elif t_ not in us:
            return False
    if ot != fc._INHERIT_RULE:
        outs = [fc._named_type(x[2]) for x in tup if x[2] != fc._INHERIT_RULE]
        t_ = fc._named_type(ot)
        if not isinstance(t_, tuple):
            return False
        if t_[0] == 0:
            return t_ in {o for o in outs if isinstance(o, tuple) and o[0] == 0}
        so = [o[1] for o in outs if isinstance(o, tuple) and o[0] == 1]
        if not (so and min(so) <= t_[1] <= max(so) and 0 <= t_[2] <= t_[1] - 1):
            return False
    return True


def _sumbool3_domain():
    """engine/sumbool3_domain.json (the regression check): the exact three-operand Sum-with-a-boolean tuples
    recordings show in Simulink."""
    global _SUMBOOL3_DOMAIN
    if _SUMBOOL3_DOMAIN is None:
        p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sumbool3_domain.json")
        _SUMBOOL3_DOMAIN = {json.dumps(t) for t in json.load(open(p))} if os.path.exists(p) else set()
    return _SUMBOOL3_DOMAIN


def user_block_of(name, names):
    """The user block a compiled block's cost belongs to (hardware cost by the user's
    design): a lowering's helper ('<name>\x1d...', '\x1e', '\x1f') and an element of a vector block ('G[1]' when
    'G[1]' is not itself a block of the compiled design ... -- the element copies) are reported under the user block."""
    o = re.split("[\x1d\x1e\x1f]", name)[0] or name
    m = re.match(r"^(.*)\[\d+\]$", o)
    if m and m.group(1) not in names:
        o = m.group(1)
    return o


def _param_text(v, where):
    """Block parameters are text (as in a model file); a finite JSON number is taken as its text form. Anything else
    -- true / false, lists, objects, null, a non-finite number -- is BAD_PARAM (such values crashed deep
    inside the evaluation)."""
    if isinstance(v, str):
        return v
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise GraphError("BAD_PARAM", where, "parameter values are strings (or numbers), got %s"
                         % type(v).__name__)
    if isinstance(v, float) and v != v or v in (float("inf"), float("-inf")):
        raise GraphError("BAD_PARAM", where, "finite value required")
    return repr(v) if isinstance(v, float) else str(v)


def _guard_block(method):
    """A parameter value the block's rules cannot interpret is a structured BAD_PARAM naming the block, never an
    uncaught exception (fuzzed values ended in KeyError / OverflowError / TypeError)."""
    @functools.wraps(method)
    def run(self, n, *a, **k):
        try:
            return method(self, n, *a, **k)
        except GraphError:
            raise
        except (ValueError, TypeError, AttributeError, KeyError, IndexError, OverflowError, ZeroDivisionError) as e:
            raise GraphError("BAD_PARAM", "design.blocks.%s" % n, "a parameter value of this %s block could not be "
                             "interpreted (%s: %s)" % (self.btype.get(n, "?"), type(e).__name__, str(e)[:160]))
    return run


def _check_params(bt, p_user, where):
    """Unknown parameters and unsupported values are errors. Returns the accepted parameters that the
    engine does not use (reported back as ignored)."""
    sem = set(DEFAULTS.get(bt, {})) | set(REQUIRED.get(bt, ())) | set(SEMANTIC_EXTRA.get(bt, ()))
    ignored = []
    for k, v in p_user.items():
        if isinstance(v, str) and k not in COSMETIC_COMMON and not k.startswith("_") and \
                any(ord(ch) > 126 or (ord(ch) < 32 and ch not in "\t\n\r") for ch in v):
            # at the one reading point of every block parameter (recordings in the recorded configuration: BitMask
            # '3' + U+00A0 was answered -- its reader does not go through fxnum.canonical; Simulink's set_param
            # rejects it). Cosmetic text (Description, Tag ...) may hold any character; '_' keys are the engine's own
            # lowering parameters (hidden slot names hold control characters)
            raise GraphError("BAD_PARAM", "%s.%s" % (where, k), "%s %r has a character outside printable ASCII "
                             "(a no-break space, a control character ...): Simulink rejects the value" % (k, v[:60]))
        if bt in ("Lookup1D", "Lookup2D", "LookupND", "Prelookup", "InterpolationUsingPrelookup") and \
                (k == "Table" or k == "BreakpointsData" or re.match(r"^BreakpointsForDimension\d+$", k)) and \
                isinstance(v, str) and re.search(r"(?i)(?<![A-Za-z_])(nan|inf)(?![A-Za-z_0-9])", v):
            # recordings (and the same class swept: 1-D Lookup table / breakpoints, Prelookup breakpoints): NaN / Inf
            # in a table or breakpoint stored in fixed point ended INTERNAL; Simulink stops (AttemptconvertNaN2Fix /
            # fxpParameterOverflow)
            raise GraphError("PORT_TYPE_INVALID", "%s.%s" % (where, k), "%s %s %r: NaN / Inf cannot be stored in a "
                             "fixed-point table (Simulink stops: AttemptconvertNaN2Fix / fxpParameterOverflow)"
                             % (bt, k, v[:60]))
        if DESIGN_RANGE_RE.match(k) and str(v).strip() == "":
            # an empty design range is not "unset" -- Simulink rejects it (InvParamSetting)
            raise GraphError("BAD_PARAM", "%s.%s" % (where, k), "%s is empty; Simulink rejects an empty value "
                             "(write '[]' for no range)" % k)
        if k in STOPPING_VALUES and str(v).strip() in STOPPING_VALUES[k][0]:
            # recordings: "ignored" parameters whose value stops Simulink in these designs
            raise GraphError(STOPPING_VALUES[k][1], "%s.%s" % (where, k), "%s %s=%r: %s" % (bt, k, str(v).strip(),
                                                                                         STOPPING_VALUES[k][2]))
        if k in COSMETIC_COMMON:
            # block-layout and annotation parameters every Simulink block has
            ignored.append(k)
            continue
        if k in CODEGEN_STATE and bt in STATEFUL:
            # code-generation storage of a block's state (RTWStateStorageTypeQualifier ...)
            ignored.append(k)
            continue
        if DESIGN_RANGE_RE.match(k) and str(v).strip() not in ("[]", ""):
            # recordings (silent wrong, both trees): a design range is not "ignored" by Simulink -- ParamMax widens
            # a Gain's best-precision parameter type (Gain 1.3, ParamMax 100: sfix12_En4, 1.3125), and an
            # inconsistent range stops the model (PrmDesignMaxIsOutOfDTRange, PrmQuantizedValueLessThanDesignMin ...)
            raise GraphError("UNSUPPORTED", "%s.%s" % (where, k), "design range %s = %r is not supported: Simulink "
                             "uses design minimum / maximum values to choose types and to check parameters, which "
                             "this engine does not model; leave it '[]'" % (k, str(v).strip()))
        if bt == "LookupND" and k == "IndexSearchMethod" and str(v).strip() not in ("Binary search", ""):
            # recordings: 'Evenly spaced points' needs evenly spaced breakpoints (LookupEvenSpacingMsg); the
            # search method is not recorded for n-D tables
            raise GraphError("UNSUPPORTED", "%s.%s" % (where, k), "LookupND IndexSearchMethod %r is not recorded "
                             "(only 'Binary search')" % str(v).strip())
        if k == "Inputs" and bt in ("Sum", "Product", "MinMax", "MultiportSwitch") and isinstance(v, str) and \
                not re.match(r"^\s*(\d+(\.0*)?|[|+\-*/]+)\s*$", v) and v.strip() and \
                not v.strip().startswith("[") and not re.match(r"^\s*[+\-*/|]+\d", v) and \
                not (isinstance(fxnum.canonical(v, "int"), fxnum.NumError) and
                     fxnum.canonical(v, "int").code == "BAD_PARAM"):   # (malformed text stays BAD_PARAM)
            # a port count written as an expression ('2^-1^-1^3*16', 'int8(2)/0.8') was evaluated and
            # answered; Simulink rejected those two at set_param -- other expressions are not recorded, so not supported
            raise GraphError("UNSUPPORTED", "%s.%s" % (where, k), "%s Inputs %r: a port count written as an "
                             "expression is not supported (Simulink rejects some, e.g. '2^-1^-1^3*16'); write the "
                             "number or the sign string" % (bt, v))
        if bt in ("Delay", "DelayPort") and k == "DelayLength" and isinstance(v, str):
            r_dl = fc.matlab_int_expr(v)
            if r_dl is not None and r_dl[1] in ("int64", "uint64", "logical"):
                # Simulink rejects a delay length of class int64 / uint64 / logical
                # (DelayLengthDataTypeIsNotSupported); it was read as its value
                raise GraphError("BAD_PARAM", "%s.%s" % (where, k), "Delay DelayLength %r is of class %s, which Simulink "
                                 "rejects (DelayLengthDataTypeIsNotSupported)" % (v, r_dl[1]))
        if bt == "BitwiseOperator" and k == "BitMask" and isinstance(v, str):
            vb_ = v.strip().rstrip(";").strip()
            r_mask = fc.matlab_int_expr(vb_)
            if vb_.lower() in ("true",) or vb_.lower().startswith("logical(") or (r_mask is not None and r_mask[1] == "logical"):
                # (by the parsed class, not the text -- '(true)', '((true))' were still mask 1)
                # recordings (silent wrong) / Q7c: a logical mask is 0 in Simulink (AND gives 0), not 1 -- not modelled
                raise GraphError("UNSUPPORTED", "%s.%s" % (where, k), "a logical BitMask %r is not supported (Simulink "
                                 "does not read a logical mask as its number); write the number" % v)
            vb1_ = vb_[1:-1].strip() if vb_.startswith("[") and vb_.endswith("]") and \
                len(re.split(r"[\s,;]+", vb_.strip("[]").strip())) == 1 else vb_
            if vb1_ != vb_ and not re.match(r"^\s*-?[0-9.]+(e[-+]?[0-9]+)?\s*$", vb1_):
                # '[int8(3)]' runs in Simulink; a bracketed expression is not read here
                raise GraphError("UNSUPPORTED", "%s.%s" % (where, k), "BitMask %r: a bracketed expression is not "
                                 "supported here; write the mask as a number" % v)
            if not vb1_.startswith("[") and fc.matlab_int_expr(vb1_) is None and \
                    isinstance(fxnum.canonical(vb1_, "scalar"), fxnum.NumError) and \
                    fxnum.canonical(vb1_, "scalar").code in ("UNSUPPORTED", "BAD_PARAM") and \
                    re.search(r"[A-Za-z(]", vb1_):
                # a mask written as an expression the engine does not evaluate exactly as MATLAB
                # does is not supported here (it was "block evaluation failed", a request error)
                raise GraphError("UNSUPPORTED", "%s.%s" % (where, k), "BitMask %r: this expression is not evaluated "
                                 "here; write the mask as a number" % v)
            if vb_.startswith("[") and len(re.split(r"[\s,;]+", vb_.strip("[]").strip())) > 1:
                # a vector mask runs in Simulink (element-wise); not supported here
                raise GraphError("UNSUPPORTED", "%s.%s" % (where, k), "a vector BitMask %r is not supported here; "
                                 "write one value" % v)
        if bt == "ExtractBits" and k == "bitIdxRange" and isinstance(v, str) and ";" in v.strip().rstrip(";"):
            # a column '[7;4]' runs in Simulink; not supported here (it was a request error)
            raise GraphError("UNSUPPORTED", "%s.%s" % (where, k), "Extract Bits bitIdxRange %r written as a column is "
                             "not supported here; write '[a b]'" % v)
        if bt == "CounterFreeRunning" and k == "NumBits" and re.match(r"^\s*\d+\.\d*[1-9]\d*\s*$", str(v)):
            # a non-integer bit count (2.5) runs in Simulink -- not modelled, not a malformed request
            raise GraphError("UNSUPPORTED", "%s.%s" % (where, k), "Counter Free-Running NumBits %r is not an integer; "
                             "how Simulink rounds it is not recorded" % str(v).strip())
        if bt == "ShiftArithmetic" and k == "DiagnosticForOORShift" and str(v).strip().lower() == "error":
            # 'Error' stops Simulink on an out-of-range dialog shift (40 / 100 on a 16-bit input);
            # the recorded diagnostic cases used a shift of 10 on int8. The
            # limit lies between and is not recorded: accepted for |shift| <= 10 only
            c_ = fxnum.canonical(p_user.get("BitShiftNumber", "8"), "int")
            if isinstance(c_, fxnum.NumError) or abs(int(c_)) > 10:
                raise GraphError("UNSUPPORTED", "%s.%s" % (where, k), "Shift Arithmetic DiagnosticForOORShift 'Error' "
                                 "with shift %s is not supported (Simulink stops on an out-of-range shift; recorded "
                                 "to run up to 10); use 'None' or 'Warning'" % p_user.get("BitShiftNumber"))
        if bt in ("Lookup1D", "Lookup2D") and k == "DiagnosticForOutOfRangeInput" and str(v).strip() == "Error":
            # recordings (the same Simulink block, Lookup_n-D): 'Error' stops Simulink at the first input outside
            # the breakpoints; the engine answered it
            raise GraphError("UNSUPPORTED", "%s.%s" % (where, k), "%s DiagnosticForOutOfRangeInput 'Error' is not "
                             "supported: Simulink stops the run at the first input outside the breakpoints, which this "
                             "engine does not model; use 'None' or 'Warning'" % bt)
        if bt == "LookupND" and k == "DiagnosticForOutOfRangeInput" and str(v).strip() not in ("None", ""):
            # recordings: 'Error' stops Simulink at the first input outside the breakpoints (and 'Warning' warns);
            # not modelled for n-D tables
            raise GraphError("UNSUPPORTED", "%s.%s" % (where, k), "LookupND DiagnosticForOutOfRangeInput %r is not "
                             "supported (only 'None')" % str(v).strip())
        if k == "type" or k in sem:
            continue
        if bt == "MLFn" and k.startswith("_Out"):
            continue  # the other output ports' declared types (engine/fxhdl.py _mlfn)
        if (bt == "Lookup1D" and k in ("TableSource", "BreakpointsForDimension1Source") or
                bt == "Lookup2D" and k in ("TableSource", "BreakpointsForDimension1Source",
                                           "BreakpointsForDimension2Source")) and str(v).strip() == "Input port":
            continue   # lowered by fxvector (_LutPorts); refused there if not vectors
        if k in ("_GateSteps", "_GateSlot", "_Tie", "_GainVec", "_DtiT", "_RlT", "_VecReduce", "_SFGain", "_SFGainOut", "_MatGain", "_MatGainOut", "_FracDefault", "_MatGainKey", "_ShiftRange", "_ShiftGeneral", "_MatKey", "_LutPorts", "_NDRule", "_ColGain") or (k == "_HalfGridIC" and bt == "UnitDelay") \
                or (k == "_SelIdx" and bt == "MultiportSwitch") \
                or (k == "_Clamp" and bt == "Saturation"):
            continue  # set by the multirate lowering / the vector pass only (engine/fxlower.py, fxvector.py)
        if k == "SampleTime" and bt == "CompareToConstant":
            # recordings (recorded): Simulink set_param stops (ParamUnknown: the mask has no
            # SampleTime); other masks are not recorded either way
            raise GraphError("BAD_PARAM", "%s.%s" % (where, k), "%s has no parameter SampleTime (Simulink rejects "
                             "it); it inherits its rate" % bt)
        if k == "SampleTime" or (k == "tsamp" and bt in ("CounterLimited", "CounterFreeRunning")):
            continue  # checked across the design (single rate)
        dialog = LIB_DIALOG_PARAMS.get(bt)
        m5_ = bt == "LookupND" and re.match(r"^BreakpointsForDimension([5-9]|[12]\d|30)"
                                           r"(DataTypeStr|FirstPoint|Spacing|Min|Max|Source)?$", k)
        if m5_:
            nd5 = str(p_user.get("NumberOfTableDimensions", "3")).strip()
            if nd5 not in ("3", "4"):
                # a 5-D table's own breakpoints were "unknown parameter" (request)
                raise GraphError("UNSUPPORTED", "%s.NumberOfTableDimensions" % where, "LookupND with %s dimensions "
                                 "is not recorded (3 or 4)" % nd5)
            ignored.append(k)
            continue
        if (bt == "Lookup1D" and _UNUSED_DIM.match(k)) or (bt == "Lookup2D" and _UNUSED_DIM2.match(k)):
            if k.endswith("DataTypeStr") and str(v).strip() in ("boolean", "half"):
                # recordings (native: BreakpointsForDimension21DataTypeStr 'boolean' rejected at set_param,
                # UdtInvBuiltin, even for a dimension the table does not use)
                raise GraphError("PORT_TYPE_INVALID", "%s.%s" % (where, k), "%s %s: Simulink rejects it even for an "
                                 "unused dimension (UdtInvBuiltin)" % (k, v))
            ignored.append(k)
            continue
        if (k in META_ANY or k in META_VALUES) and dialog is not None and k not in dialog:
            # names were matched across all block types (a Gain took "framing")
            raise GraphError("BAD_PARAM", "%s.%s" % (where, k), "%s is not a parameter of %s" % (k, bt))
        if k in META_ANY:
            ignored.append(k)
            continue
        if k in META_VALUES and bt == "DelayPort" and k in ("ShowEnablePort", "ExternalReset") and \
                _delay_enable_reset(p_user):
            continue   # the enabled / resettable Delay: checked against its recorded domain (Design._delayen_check)
        if k in META_VALUES and bt == "DelayPort" and k in ("ExternalReset", "PreventDirectFeedthrough") and \
                str(p_user.get("ShowEnablePort", "off")).strip() != "on" and \
                str(p_user.get("DelayLengthSource", "Dialog")) == "Input port":
            continue   # the port-length Delay: checked against engine/delaymode_domain.json (Design._refuse_unrecorded)
        if k in META_VALUES:
            allowed = META_VALUES_BY_TYPE.get(bt, {}).get(k, META_VALUES[k])
            if str(v) not in allowed:
                raise GraphError("UNSUPPORTED", "%s.%s" % (where, k), "%s=%r is not supported for %s (only %s)"
                                 % (k, v, bt, " / ".join(repr(x) for x in allowed)))
            if not (k == "UseLastTableValue" and bt in ("Lookup1D", "Lookup2D")):
                # UseLastTableValue influences the lookup fraction type.
                ignored.append(k)
            continue
        if bt == "Inport" and k == "OutDataTypeStr":
            raise GraphError("UNSUPPORTED", "%s.%s" % (where, k), "an Inport's OutDataTypeStr is not read in a JSON "
                             "design; give the input's type in the request's inputs ({\"s\", \"wl\", \"fl\"}) instead "
                             "(an .slx Inport's type is read by fxslx)")
        known_ = sorted(sem | set(dialog or ()) | set(RECORDED_VALID_PARAMS.get(bt, ())))
        alt_ = [x for x in known_ if x.lower() == k.lower() and x != k]
        if alt_:
            # Simulink matches parameter names case-insensitively ('gain' is Gain) -- valid, not a
            # malformed request; this engine reads the exact name
            raise GraphError("UNSUPPORTED", "%s.%s" % (where, k), "parameter %s of %s is written in another case; "
                             "this engine reads parameter names exactly -- write %s" % (k, bt, alt_[0]))
        if k in known_:
            raise GraphError("UNSUPPORTED", "%s.%s" % (where, k), "parameter %s of %s is a valid Simulink parameter "
                             "this engine does not read; leave it out (or at its default) to run the design" % (k, bt))
        raise GraphError("BAD_PARAM", "%s.%s" % (where, k), "unknown parameter %s for %s" % (k, bt))
    # value syntax, the same for every block and every evaluation route (the routes through
    # ref/fxcore.py had no check, so an unknown rounding / overflow value or a malformed fixdt was taken as a default
    # or crashed)
    for k, v in p_user.items():
        loc = "%s.%s" % (where, k)
        if bt == "DiscreteFir" and k == "Coefficients" and (";" in str(v) or str(v).strip().endswith("'")):
            # recordings (native: [0.5;0.25] on a scalar input is two 1-tap channels; the engine ran one 2-tap
            # filter -- silently wrong): a column / matrix of coefficients (also a transposed row, [..]') is not supported
            raise GraphError("UNSUPPORTED", loc, "Discrete FIR Filter Coefficients %r: a column or matrix gives one "
                             "filter per row in Simulink (several channels), which is not supported; write the "
                             "coefficients as a row [b0 b1 ...]" % str(v)[:60])
        if k.endswith("DataTypeStr"):
            if not isinstance(v, str):
                raise GraphError("BAD_TYPE", loc, "data type must be a string")
            txt = v.strip()
            if re.match(r"^fixdt\(\s*[01]\s*,\s*\d+\s*\)$", txt) and not (
                    bt in ("RelationalOperator", "LogicalOperator") and k == "OutDataTypeStr" and
                    txt.replace(" ", "") in LOGIC_OUT_TYPES):
                # fixdt(s, w) -- a word length with best-precision scaling (FIR Decimation's 'Specify
                # word length') is a valid Simulink type; how each block picks its scaling is not recorded
                raise GraphError("UNSUPPORTED", loc, "%r gives a word length only (best-precision scaling chosen by the "
                                 "block), which is not supported; give fixdt(s, w, f) with the fraction length" % v)
            if not (txt.startswith("Inherit:") or txt in ("double", "single", "boolean") or txt in fc._NAMED_INT
                    or fc._parse_fixdt(txt) is not None or (bt in ("RelationalOperator", "LogicalOperator") and
                                                            txt.replace(" ", "") in LOGIC_OUT_TYPES)):
                raise _type_text_error(loc, v)
            t = fc._parse_fixdt(txt)
            if t is not None and (t[0] not in (0, 1) or not 1 <= t[1] <= MAX_WL or abs(t[2]) > 4096):
                raise GraphError("BAD_TYPE", loc, "fixdt out of supported range: %s" % v)
        elif k == "RndMeth" and v not in ("Floor", "Ceiling", "Zero", "Nearest", "Round", "Convergent", "Simplest"):
            raise GraphError("UNSUPPORTED_ROUNDING", loc, "unsupported rounding method %r" % (v,))
        elif k == "SaturateOnIntegerOverflow" and v not in ("on", "off"):
            raise GraphError("BAD_PARAM", loc, "on or off")
    for k, allowed in (("OutDataTypeStr", ("boolean",) + tuple(LOGIC_OUT_TYPES)),):
        if bt in ("RelationalOperator", "LogicalOperator") and k in p_user and \
                str(p_user[k]).replace(" ", "") not in allowed:
            if str(p_user[k]).strip() == "Inherit: Logical":
                # recordings (14 designs): SimulinkBlock:Foundation:UdtInvalidValue
                raise GraphError("PORT_TYPE_INVALID", "%s.%s" % (where, k), "%s output type 'Inherit: Logical' is "
                                 "not a valid value in Simulink (UdtInvalidValue); use boolean or a recorded integer "
                                 "type" % bt)
            raise GraphError("UNSUPPORTED", "%s.%s" % (where, k), "%s output type %r: recorded are boolean and %s"
                             % (bt, p_user[k], ", ".join(sorted(LOGIC_OUT_TYPES))))
    # an Outport's own data type is a constraint, not a conversion (recorded, recordings: an exported design needs it
    # on a DUT's Outport): it must be the compiled type of the signal that drives it -- checked after compile
    return ignored


# recordings (34 designs host and ASIC): Relational / Logical Operator
# output types Simulink runs with the result 0 / 1 in them; 'fixdt(1,16)' (no fraction length) compiles to int16
LOGIC_OUT_TYPES = {"fixdt(1,16,0)": (1, 16, 0), "fixdt(1,16)": (1, 16, 0), "fixdt(0,8,0)": (0, 8, 0),
                   "fixdt(0,1,0)": (0, 1, 0), "fixdt(1,32,0)": (1, 32, 0)}


# Production hardware settings that recordings has recorded (parameter sweep after recordings): Intel x86-64 (Windows64)
# with int / long 32 bits, long long off, and ASIC/FPGA. Any other value of a setting the engine reads (native word
# sizes) is refused; settings the engine does not read are accepted only with their recorded values.
HW_RECORDED = {"ProdHWDeviceType": None, "ProdBitPerInt": ("32",), "ProdBitPerLong": ("32",),
               "ProdBitPerLongLong": ("64",), "ProdLongLongMode": ("off",), "ProdBitPerChar": ("8",),
               "ProdBitPerShort": ("16",), "ProdWordSize": ("64",), "ProdIntDivRoundTo": ("Zero",),
               "ProdShiftRightIntArith": ("on",), "ProdEqTarget": ("on",), "FixedStep": None,
               # multirate values differ by mode; MultiTasking / Auto refused for
               # multirate designs in fxmultirate.rates
               "SolverMode": ("SingleTasking", "Auto", "MultiTasking"),
               # Simulink's Diagnostics > Data Validity > Parameters > "Detect overflow" (default error: the model stops
               # with fxpParameterOverflow; warning / none: the parameter saturates -- recordings,
               # and most recordings recorders set none)
               "ParameterOverflowMsg": ("error", "warning", "none")}


def _check_hw(hw):
    if hw is None:
        return
    if not isinstance(hw, dict):
        raise GraphError("BAD_PARAM", "hw", "hardware settings object required")
    # recordings (int and long 16 bits on a custom processor): Simulink stops when none of char / short /
    # int / long is 32 bits (Simulink:ConfigSet:Hardware_Prod_No32BitType)
    sizes_ = [str(hw.get(k_, d_)).strip() for k_, d_ in (("ProdBitPerChar", "8"), ("ProdBitPerShort", "16"),
                                                       ("ProdBitPerInt", "32"), ("ProdBitPerLong", "32"))]
    if "32" not in sizes_:
        raise GraphError("PORT_TYPE_INVALID", "hw.ProdBitPerInt", ('char / short / int / long of %s bits: Simulink requires one '
            'of them to be 32 bits and stops (Simulink:ConfigSet:Hardware_Prod_No32BitType); '
            'set ProdBitPerInt or ProdBitPerLong to 32') % " / ".join(sizes_))
    for k, v in hw.items():
        where = "hw." + k
        if k not in HW_RECORDED:
            raise GraphError("UNSUPPORTED", where, "hardware setting %s has never been recorded" % k)
        if k == "ProdHWDeviceType":
            if str(v) != "Intel->x86-64 (Windows64)" and not str(v).startswith("ASIC/FPGA"):
                raise GraphError("UNSUPPORTED", where, "production hardware %r: only Intel->x86-64 (Windows64) and "
                                 "ASIC/FPGA have been recorded" % v)
        elif HW_RECORDED[k] is not None and str(v).strip() not in HW_RECORDED[k]:
            raise GraphError("UNSUPPORTED", where, "%s = %r: only %s has been recorded" % (k, v, HW_RECORDED[k][0]))


# the error reporting contract ("on a refusal: where, why, and what would make it supported"): per error code, what kind of
# refusal it is and what the caller can do next (added to every fxgraph error result)
_NOT_VERIFIED = ("refused, not answered: this configuration has not been verified bit-exact against Simulink. Change "
                 "the parameter or block named in where / message to a supported form (docs/en/GRAPH.md, "
                 "docs/en/BLOCKS.md); if the design needs it, report where + message so it can be recorded")
NEXT_STEP = {
    "SIM_DIAGNOSTIC": ("simulink_rejects", "Simulink stops this run with an error at the step named in the message "
                       "(a block diagnostic set to Error): change the input, or the block's diagnostic setting"),
    "UNSUPPORTED": ("not_supported", _NOT_VERIFIED),
    "UNSUPPORTED_RATE": ("not_supported", "the rate structure is not supported: integer rate ratios, sample-based "
                         "multirate blocks with their rate options written out (docs/en/GRAPH.md, multirate designs)"),
    "UNSUPPORTED_WIDTH": ("not_supported", "an intermediate is wider than the C core holds: use engine auto / python, "
                          "or narrower types"),
    "UNSUPPORTED_ROUNDING": ("not_supported", "use one of the rounding methods Floor, Ceiling, Zero, Nearest, Round, "
                             "Convergent, Simplest, spelled exactly"),
    "PORT_TYPE_INVALID": ("simulink_rejects", "Simulink rejects this data-type combination too (the message names "
                          "its error id where recorded): change the types named in the message"),
    "TYPE_MISMATCH": ("simulink_rejects", "the inherited types disagree as they would in Simulink: give one of the "
                      "blocks named in the message an explicit output type"),
    "TYPE_UNRESOLVED": ("not_supported", "the inherited types around a feedback loop cannot be resolved here: give "
                        "a block in the loop an explicit output type"),
    "ALGEBRAIC_LOOP": ("request", "a feedback path without delay: put a Unit Delay (or Delay) in the loop"),
    "LIMIT_EXCEEDED": ("request", "the run is larger than the limits: fewer steps or a smaller design"),
    "INTERNAL": ("defect", "an engine defect, not a property of the design: report input_sha256 and the request"),
    "CORE_FAILED": ("defect", "the C core failed: retry with engine python and report input_sha256"),
    "TIMEOUT": ("request", "the run took longer than FXGRAPH_TIMEOUT_S: fewer steps, a smaller design, or a larger "
                "limit"),
    "OOM": ("request", "the run needed more memory than allowed: fewer steps or a smaller design"),
}
# recorded: an Assertion block with StopWhenAssertionFail on stopped the run, as Simulink stops it
# an internal signal of the recorded lowering (Design.compile lowers the design again and never lets it out)
NEXT_STEP["RELOWER"] = ("defect", "an internal step of the engine reached the caller: report it with the input_sha256")
NEXT_STEP["ASSERTION_FAILED"] = ("simulink_rejects", "Simulink stops the simulation at the same step: the assertion's "
                                 "input was zero there (the message names the block and the step); change the inputs "
                                 "or the design, or set StopWhenAssertionFail off to record the failures instead")
for _c in ("BAD_PARAM", "BAD_TYPE", "MISSING_FIELD", "BAD_INPUT", "BAD_JSON", "BAD_CONNECTION", "UNKNOWN_BLOCK_TYPE"):
    NEXT_STEP[_c] = ("request", "the request itself is malformed at where: fix that field (the message says what "
                     "is accepted)")
# the codes of fxopt, fxcert and fxslx (their errors carried no refusal_kind / next_step)
NEXT_STEP.update({
    "UNSUPPORTED_BLOCK": ("not_supported", _NOT_VERIFIED),
    "UNSUPPORTED_PARAM": ("not_supported", _NOT_VERIFIED),
    "UNSUPPORTED_MATLAB": ("not_supported", "the MATLAB Function script uses a construct outside the supported "
                           "subset (the message names the line): rewrite it with the supported constructs"),
    "BAD_MATLAB": ("request", "the MATLAB Function script is malformed at the line named in the message: fix it"),
    "BAD_SLX": ("request", "the file is not a readable Simulink .slx model (the message names the part): give an "
                ".slx saved by Simulink"),
    "FILE_NOT_FOUND": ("request", "the file named in path cannot be read: give an existing path (an absolute path, "
                       "or one relative to the fxsim tree -- paths resolve against the tree from any working "
                       "directory)"),
    "INFEASIBLE": ("request", "even the widest allowed word lengths miss the error tolerance: widen wordlengths / "
                   "block_wordlengths or loosen the tolerance"),
    "NO_SOLUTION": ("request", "no plan met the constraints within the search limits: loosen the tolerance or the "
                    "word-length limits, or raise the time limit"),
    "REFERENCE_NOT_FINITE": ("request", "the ideal double-precision reference is not finite on these inputs "
                             "(for example a division by zero): change the inputs or the design"),
    "LOG_DIR_EXISTS": ("request", "log_dir must be a new directory: give another name (a run is never resumed)"),
    "CORE_UNAVAILABLE": ("defect", "the C core is missing or does not start: run sh engine/build.sh once, or use "
                         "engine python"),
    "VERIFY_FAILED": ("defect", "an independent re-computation disagreed with the result: a defect, do not use the "
                      "result; report input_sha256 and the request"),
    "UNRELATED_FAILURE": ("defect", "an evaluation failed for a reason unrelated to the plan: report input_sha256"),
    "UNSUPPORTED_EXPORT": ("not_supported", _NOT_VERIFIED),
    # a double start type where the optimiser needs a fixed one (upstream of a multirate filter); the
    # generic "not verified against Simulink" next step contradicted the message
    "START_TYPE_NEEDED": ("request", "give the block named in where a fixed-point start type (fixdt): the optimiser "
                          "then chooses its word length; double start types are accepted elsewhere"),
})


_SPEC_REF = re.compile(r"\s*\((?:see\s+)?(?:docs/SPEC/[^()]*|SPEC\s*§[^()]*)\)|\s*[,;]?\s*docs/SPEC/[A-Za-z0-9_./]+(?:\s+R\d+)?")


# parameter values Simulink runs that the numeric parser refuses
SIMULINK_ACCEPTS_NUMERIC = {("CounterLimited", "uplimit"), ("DiscreteIntegrator", "gainval"), ("CompareToConstant", "const")}

# left-out settings the engine reads with an in-code fallback equal to the library dialog value
#, listed in assumed_defaults like the DEFAULTS entries
REPORTED_LIBRARY_DEFAULTS = {"RelationalOperator": {"OutDataTypeStr": "boolean", "RndMeth": "Simplest"},   # recordings;
                             # Simplest acts as Floor in the lossy comparison cast (fc.relop_cast_fm)
                             "Abs": {"OutDataTypeStr": "Inherit: Same as input", "RndMeth": "Floor"},
                             "Saturation": {"OutDataTypeStr": "Inherit: Same as input"},
                             "Outport": {"OutDataTypeStr": "Inherit: auto"}}

# A Constant at its library type (double): refused with its own next step
CONST_DOUBLE_MSG = ("Constant %s has the %s output type (the library default when OutDataTypeStr is left out): "
                    "double signals are simulated only on the recorded floating-point shapes (docs GRAPH.md, "
                    "Floating-point signals), and this design is not one of them; in Simulink the blocks it feeds "
                    "would compute in double too")
NEXT_STEP_BY_PREFIX = (
    ("Constant ", " output type (the library default when OutDataTypeStr is left out)",
     ("not_supported", "set the Constant's OutDataTypeStr to a fixed-point type (e.g. fixdt(1,16,8)), or to 'Inherit: "
                       "Inherit via back propagation' to take the type of the input it feeds")),)


def with_next_step(rec):
    """Attach refusal guidance and normalize the diagnostic text."""
    if isinstance(rec, dict) and isinstance(rec.get("message"), str):
        clean = _SPEC_REF.sub("", rec["message"]).strip()
        if clean and clean != rec["message"]:
            rec["message"] = clean
    if isinstance(rec, dict) and rec.get("status") == "error" and "refusal_kind" not in rec:
        msg = rec.get("message") if isinstance(rec.get("message"), str) else ""
        for pre, mid, ks in NEXT_STEP_BY_PREFIX:
            if msg.startswith(pre) and mid in msg:
                rec["refusal_kind"], rec["next_step"] = ks
                return rec
        rec["refusal_kind"], rec["next_step"] = NEXT_STEP.get(rec.get("code"), ("unknown", "see message"))
    return rec


def refuse_runtime_diag(d, tool):
    """For tools that cannot stop a run (the code generators): a design whose Multiport Switch stops Simulink on an
    out-of-range control is refused. fxgraph (Python path and C core) stops the run; the
    optimiser treats an input that stops Simulink as infeasible."""
    rd = getattr(d, "runtime_diag", None)
    if rd:
        raise GraphError("UNSUPPORTED", "design.blocks.%s.DiagnosticForDefault" % rd[0],
                         "%s: Multiport Switch %s stops the simulation on an out-of-range control (DiagnosticForDefault "
                         "'Error' -- the library default when left out -- or a single data port); set "
                         "DiagnosticForDefault to 'None' (the default port is taken) with at least two data ports"
                         % (tool, ", ".join(rd[:4])))


def public_block_types():
    return sorted(t for t in DEFAULTS if t not in INTERNAL_TYPES)


def catalogue():
    """the block types a design may use, each with its parameter names and the value a design that
    leaves one out gets (the Simulink library default for a JSON design); "required" parameters have no default."""
    out = {}
    for t in public_block_types():
        lib = dict(DEFAULTS[t])
        lib.update(JSON_LIBRARY_DEFAULTS.get(t, {}))
        req = list(REQUIRED.get(t, ())) + list(JSON_LIBRARY_UNRECORDED.get(t, ()))   # no library value
        for k in JSON_LIBRARY_UNRECORDED.get(t, ()):
            lib.pop(k, None)
        sem = set(DEFAULTS.get(t, {})) | set(req) | set(SEMANTIC_EXTRA.get(t, ())) | \
            set(JSON_LIBRARY_UNSUPPORTED.get(t, {}))
        dialog = LIB_DIALOG_PARAMS.get(t)
        restricted = {k: sorted(META_VALUES_BY_TYPE.get(t, {}).get(k, META_VALUES[k])) for k in META_VALUES
                      if dialog is None or k in dialog} if dialog is not None else \
            {k: sorted(v) for k, v in META_VALUES_BY_TYPE.get(t, {}).items()}
        sem |= set(restricted)
        if t != "Outport":
            sem.add("SampleTime")
        out[t] = {"parameters": sorted(k for k in sem if not k.startswith("_")),
                  "defaults": {k: v for k, v in sorted(lib.items()) if not k.startswith("_")},
                  "required": req,
                  "must_write": sorted(JSON_LIBRARY_UNSUPPORTED.get(t, {})),
                  # parameters accepted with these values only
                  "values": restricted}
    return out


def check_request_fields(req, allowed, where=""):
    """The request-field rule of every CLI (fxchain ignored unknown fields): an unknown field is a
    BAD_PARAM, never silently dropped."""
    extra = sorted(set(req) - set(allowed))
    if extra:
        raise GraphError("BAD_PARAM", where + extra[0], "unknown request field %s" % extra[0])


def hw_words(hw):
    """Validated hardware settings -> native word sizes (smallest first; [] on ASIC/FPGA). One function for every
    path, so that hw takes effect everywhere (fxchain used the host sizes whatever hw said)."""
    _check_hw(hw)
    return fc._hw_words({"hw": hw or {}})


def _block_rates_differ(btype, blocks, names):
    """a block other than an Inport with an explicit sample-time offset ([P o], o != 0),
    or explicit periods that differ between blocks, make a multirate design (engine/fxmultirate.py decides which of
    them are supported); a single explicit period with offset 0 stays single-rate."""
    seen = set()
    for n in names:
        v = str(blocks[n].get("SampleTime", "-1")).strip()
        if v in ("-1", "inf", "Inf", ""):
            continue
        parts = v.strip("[]").replace(",", " ").split()
        if not parts or not all(_NUMLIT.match(x) for x in parts):
            continue   # the single-rate check reports the malformed text
        if len(parts) == 2 and float(parts[1]) != 0 and btype[n] != "Inport":
            return True
        seen.add(float(parts[0]))
    return len(seen) > 1


def _check_rates(btype, params):
    """Single-rate check over the design: explicit sample times must be -1, inf (Constant only) or one common
    positive period with zero offset."""
    periods = set()
    for n, p in params.items():
        for k in ("SampleTime", "tsamp"):
            if k not in p:
                continue
            v = str(p[k]).strip()
            where = "design.blocks.%s.%s" % (n, k)
            if v in ("inf", "Inf"):
                if btype[n] != "Constant":
                    raise GraphError("UNSUPPORTED", where, "infinite sample time on %s" % btype[n])
                continue
            m = re.match(r"^\[?\s*([^\s,\]]+)\s*(?:[ ,]\s*([^\s\]]+))?\s*\]?$", v)
            if not m or not _NUMLIT.match(m.group(1)) or (m.group(2) is not None and not _NUMLIT.match(m.group(2))):
                raise GraphError("UNSUPPORTED", where, "sample time %r is not a numeric literal" % v)
            if m.group(2) is not None and float(m.group(2)) != 0:
                raise GraphError("UNSUPPORTED", where, "sample time offset %s (only 0)" % m.group(2))
            t = float(m.group(1))
            if t == -1:
                continue
            if t <= 0:
                raise GraphError("UNSUPPORTED", where, "sample time %s: only discrete (positive) periods" % v)
            periods.add(t)
    if len(periods) > 1:
        raise GraphError("UNSUPPORTED", "design.blocks", "several sample times %s: multi-rate designs are not "
                         "supported" % sorted(periods))
    return periods.pop() if periods else None


DTI_PERIOD_MSG = ("Discrete-Time Integrator at sample period %s: only period 1 is supported (the integration step "
                  "T enters the update; other periods have not been recorded)")


# hidden slots of the C-core expansions (counter: next state via compare + switch; detect: one-step delay;
# integrator: increment and next state; signum: two compares)
DSTATEFUL = ("CounterLimited", "CounterFreeRunning", "DetectChange", "DetectIncrease", "DetectDecrease",
             "DiscreteIntegrator")
HIDDEN_P4B2 = {"Hold": ("hd",), "Gate": ("zero",),
               "CounterLimited": ("one", "lim", "zero", "eq", "inc", "nxt"),
               "CounterFreeRunning": ("one", "lim", "zero", "eq", "inc", "nxt"),
               "DetectChange": ("prev",), "DetectIncrease": ("prev",), "DetectDecrease": ("prev",),
               "DiscreteIntegrator": ("inc", "sum", "nxt", "prev", "sum2", "first", "zero", "ic", "rp", "ta", "tb",
                                      "trig", "xe"),
               "Signum": ("zero", "pos", "neg")}
# parameters the reference functions use to tell library blocks apart (filled in from the engine block type)
FXCORE_INJECT = {"CounterLimited": {"library_block": "simulink/Sources/Counter Limited"},
                 "CounterFreeRunning": {"library_block": "simulink/Sources/Counter Free-Running"},
                 "DetectChange": {"library_block": "Detect Change"},
                 "DetectIncrease": {"library_block": "Detect Increase"},
                 "DetectDecrease": {"library_block": "Detect Decrease"},
                 "Signum": {"BlockType": "Signum"}, "Rounding": {"BlockType": "Rounding"},
                 "Lookup1D": {"BlockType": "Lookup_n-D"}, "Lookup2D": {"BlockType": "Lookup_n-D"}}
# parameters quantised to the input type of Dead Zone / Rate Limiter / Wrap To Zero
NL_PARAMS_ALL = {"DeadZone": ("LowerValue", "UpperValue"), "WrapToZero": ("Threshold",),
                 "RateLimiter": ("RisingSlewLimit", "FallingSlewLimit", "InitialCondition")}
NL_PARAMS = {"DeadZone": ("LowerValue", "UpperValue"), "WrapToZero": ("Threshold",),
             "RateLimiter": ("RisingSlewLimit", "InitialCondition")}   # the falling limit: see _check_validated
DELAYS = ("UnitDelay", "Delay")  # output depends on past inputs only (breaks algebraic loops)
# recordings (recordings a .. d): multirate filters fed
# by an inherited sfix8_En1 loop that do not start it -- the loop keeps its own type. Only these configurations are
# recorded; every other one fed by a loop is refused (_mrfilter_recorded). Keys: the parameters that set the output.
# recordings (recordings a .. e) adds five more: FIR Decimation fixdt(1,4,0) /
# fixdt(1,16,4), FIR Interpolation fixdt(1,8,0), CIC Decimation binary point scaling (sections 16 / 1, output 16 / 4)
# and minimum section word lengths with a 6-bit output.
MRFILTER_ON_LOOP = {
    "FIRDecimation": [{"h": "[0.5 0.5]", "D": "2", "outputMode": "Binary point scaling", "outputDataTypeStr": t}
                      for t in ("fixdt(1,32,0)", "fixdt(1,8,0)", "fixdt(1,4,0)", "fixdt(1,16,4)")],
    "FIRInterpolation": [{"h": "[0.5 0.5]", "L": "2", "outputMode": "Binary point scaling", "outputDataTypeStr": t}
                         for t in ("fixdt(1,32,0)", "fixdt(1,8,0)")],
    "CICDecimation": [{"R": "2", "M": "1", "N": "2", "filterInternals": "Minimum section word lengths",
                       "outputWordLength": w, "outputFracLength": "0"} for w in ("32", "6")] +
                     [{"R": "2", "M": "1", "N": "2", "filterInternals": "Binary point scaling", "BPS": "[16 16 16 16]",
                       "FLPS": "[1 1 1 1]", "outputWordLength": "16", "outputFracLength": "4"}],
    "CICInterpolation": [],
}
MRFILTER_LOOP_TYPE = (1, 8, 1)


# the default of InputSameDT ("Require all inputs to have the same data type") when a design leaves it
# out, per block type. Recorded: 2-D Lookup Table 'on' (recordings a .. d: inputs
# ufix14_En3 / sfix32_En25 rejected with InputPortDataTypeMismatch); Sum, Product, MinMax and Relational Operator
# 'off' (recordings show-isdt-*: inputs ufix8_En1 / sfix12_En3 compile); Switch and Multiport Switch
# 'off' (the same requests compile, read back 'off'). None would mark a block that
# takes the parameter with an unrecorded default (refused when left out and the data inputs differ); none is left.
# Block types without the parameter are not listed.
INPUT_SAME_DT_DEFAULT = {"Lookup2D": "on", "LookupND": "on", "Sum": "off", "Product": "off", "MinMax": "off",
                         "RelationalOperator": "off", "Switch": "off", "MultiportSwitch": "off"}
# the same-type parameter of each multi-input block type and its default (recordings
# dialog values; recordings for InputSameDT): Logical Operator's is AllPortsSameDT ('off';
# 'on' with differing inputs: the native 29 recordings rejections, recordings). Bitwise Operator has
# none and always requires one type (handled in _kernel_core); any other multi-input type is refused when its inputs
# differ (its rule is not recorded).
SAME_DT_PARAM = dict({bt: ("InputSameDT", v) for bt, v in INPUT_SAME_DT_DEFAULT.items()},
                     LogicalOperator=("AllPortsSameDT", "off"))


def _out_rule_error(where, ot):
    """An output data type string the block does not take here: 'Inherit: Inherit via back propagation', which
    Simulink offers for Gain / Product / Sum / MinMax but the engine does not implement, is UNSUPPORTED
    (Simulink compiles such a MinMax); anything else -- including
    rules the block does not offer -- stays BAD_TYPE, as on the fxchain path."""
    if str(ot).strip() == "Inherit: Inherit via back propagation":
        return GraphError("UNSUPPORTED", where + ".OutDataTypeStr", "output data type rule %r is not implemented" % ot)
    return _type_text_error(where + ".OutDataTypeStr", ot)


TYPE_CALL_RE = re.compile(r"^\s*(fixdt|float|numerictype)\s*\(\s*'[A-Za-z0-9]+'\s*\)\s*$")   # fixdt('double') ...
# fixdt with slope and bias (fixdt(1,16,0.1,0), fixdt(1,16,0.25,1)) or slope-adjustment / exponent / bias
# is a valid Simulink type the engine does not take (binary-point scaling only)
TYPE_SLOPE_RE = re.compile(r"^\s*fixdt\(\s*[01]\s*,\s*\d+\s*(,\s*[^,()]+\s*){2,3}\)\s*$")


# type strings Simulink runs (recordings-checked by the reviewer) that the engine does not parse -- named types
# it does not take on this block, other type constructors, fixdt with numeric expressions / decimals, a trailing ';'
TYPE_NAME_UNSUP = ("int64", "uint64", "half", "double", "single")
# built-in names in another letter case ('UINT8', 'BOOLEAN') -- Simulink normalises them; not here
TYPE_NAMES_CI = ("int8", "uint8", "int16", "uint16", "int32", "uint32", "boolean", "int64", "uint64", "half", "double",
                 "single")
TYPE_CTOR_RE = re.compile(r"^\s*(numerictype|sfrac|ufrac|sint|uint|sfix|ufix)\s*\([^()]*\)\s*;?\s*$")
TYPE_FIXDT_EXPR_RE = re.compile(r"^\s*fixdt\s*\(\s*([0-9.eE+\-*/^ ]+),([0-9.eE+\-*/^ ]+),([0-9.eE+\-*/^ ]+)\)\s*(;?)\s*$")


# recordings (silent wrong): enumerated parameters were compared with one exact string, and any other
# text ran as some default ('u2 >= threshold' as '>', 'stored integer (si)' as real-world value, 'left' as a right
# shift). Simulink reads an option case-insensitively, an exact match first, else a unique prefix ('u2 >=' is
# 'u2 >= Threshold', but 'Stored' is rejected -- abbreviations are refused here as not supported), and rejects anything
# else (BadSetParamValue).
ENUM_OPTIONS = {
    # recordings (recorded blocks; Simulink takes other letter cases: 'newest', 'Held', 'Reset' recorded valid)
    ("TappedDelay", "DelayOrder"): ("Oldest", "Newest"), ("TappedDelayEnabledSynchronous", "DelayOrder"): ("Oldest", "Newest"),
    ("EnablePort", "StatesWhenEnabling"): ("held", "reset"), ("EnabledOutport", "OutputWhenDisabled"): ("held", "reset"),
    ("ComplexToRealImag", "Output"): ("Real and imag", "Real", "Imag"),
    # recorded popup sweep: a port-indexed Selector read any IndexMode but 'Zero-based' as one-based
    ("Selector", "IndexMode"): ("One-based", "Zero-based"),
    ("Selector", "IndexOptions"): ("Select all", "Index vector (dialog)", "Index vector (port)", "Starting index (dialog)",
                                   "Starting index (port)"),
    ("Switch", "Criteria"): ("u2 >= Threshold", "u2 > Threshold", "u2 ~= 0"),
    ("DataTypeConversion", "ConvertRealWorld"): ("Real World Value (RWV)", "Stored Integer (SI)"),
    ("ShiftArithmetic", "BitShiftDirection"): ("Left", "Right", "Bidirectional"),
    ("LogicalOperator", "Operator"): ("AND", "OR", "NAND", "NOR", "XOR", "NXOR", "NOT"),
    (None, "RndMeth"): ("Ceiling", "Convergent", "Floor", "Nearest", "Round", "Simplest", "Zero"),

    # the same class, probed after recordings (case variants were refused as request errors; IconShape 'bogus' was answered)
    ("MinMax", "Function"): ("min", "max"),
    ("RelationalOperator", "Operator"): ("==", "~=", "<", "<=", ">=", ">", "isInf", "isNaN", "isFinite"),
    ("CompareToConstant", "relop"): ("==", "~=", "<", "<=", ">=", ">"),
    ("CompareToZero", "relop"): ("==", "~=", "<", "<=", ">=", ">"),
    ("Lookup1D", "InterpMethod"): ("Flat", "Nearest", "Linear point-slope", "Linear Lagrange", "Cubic spline", "Akima spline"), ("Lookup2D", "InterpMethod"): ("Flat", "Nearest", "Linear point-slope", "Linear Lagrange", "Cubic spline", "Akima spline"), ("LookupND", "InterpMethod"): ("Flat", "Nearest", "Linear point-slope", "Linear Lagrange", "Cubic spline", "Akima spline"),
    ("InterpolationUsingPrelookup", "InterpMethod"): ("Flat", "Nearest", "Linear point-slope", "Linear Lagrange",
                                                      "Cubic spline", "Akima spline"),
    ("Lookup1D", "ExtrapMethod"): ("Clip", "Linear", "Cubic spline", "Akima spline"), ("Lookup2D", "ExtrapMethod"): ("Clip", "Linear", "Cubic spline", "Akima spline"), ("LookupND", "ExtrapMethod"): ("Clip", "Linear", "Cubic spline", "Akima spline"),
    ("Prelookup", "ExtrapMethod"): ("Clip", "Linear"), ("InterpolationUsingPrelookup", "ExtrapMethod"): ("Clip", "Linear"),
    ("DiscreteIntegrator", "IntegratorMethod"): ("Integration: Forward Euler", "Integration: Backward Euler",
                                                 "Integration: Trapezoidal", "Accumulation: Forward Euler",
                                                 "Accumulation: Backward Euler", "Accumulation: Trapezoidal"),
    ("DiscreteIntegrator", "ExternalReset"): ("none", "rising", "falling", "either", "level", "sampled level"),
    ("Delay", "ExternalReset"): ("None", "Rising", "Falling", "Either", "Level", "Level hold"),
    ("Gain", "Multiplication"): ("Element-wise(K.*u)", "Matrix(K*u)", "Matrix(u*K)", "Matrix(K*u) (u vector)"),
    ("Product", "Multiplication"): ("Element-wise(.*)", "Matrix(*)"),
    ("BitwiseOperator", "logicop"): ("AND", "OR", "NAND", "NOR", "XOR", "NOT"),
    ("Sum", "IconShape"): ("rectangular", "round"),
    # recordings (silent wrong on Bitwise mask mode and 1-D Lookup breakpoint specification; option text
    # answered on 20+ parameters): every enumerated parameter the engine reads goes through the same resolver
    ("BitwiseOperator", "BitMaskRealWorld"): ("Real World Value", "Stored Integer"),
    ("LogicalOperator", "IconShape"): ("rectangular", "distinctive"),
    ("Lookup1D", "BreakpointsSpecification"): ("Explicit values", "Even spacing", "Breakpoint object"),
    ("Lookup1D", "IndexSearchMethod"): ("Evenly spaced points", "Linear search", "Binary search"),
    ("Lookup1D", "DiagnosticForOutOfRangeInput"): ("None", "Warning", "Error"),
    ("Lookup1D", "TableSource"): ("Dialog", "Input port"),
    ("Lookup1D", "BreakpointsDataSource"): ("Dialog", "Input port"),
    ("Lookup1D", "InternalRulePriority"): ("Speed", "Precision"),
    ("Lookup2D", "BreakpointsSpecification"): ("Explicit values", "Even spacing", "Breakpoint object"),
    ("Lookup2D", "IndexSearchMethod"): ("Evenly spaced points", "Linear search", "Binary search"),
    ("Lookup2D", "DiagnosticForOutOfRangeInput"): ("None", "Warning", "Error"),
    ("Lookup2D", "TableSource"): ("Dialog", "Input port"),
    ("Lookup2D", "BreakpointsDataSource"): ("Dialog", "Input port"),
    ("Lookup2D", "InternalRulePriority"): ("Speed", "Precision"),
    ("LookupND", "BreakpointsSpecification"): ("Explicit values", "Even spacing", "Breakpoint object"),
    ("LookupND", "IndexSearchMethod"): ("Evenly spaced points", "Linear search", "Binary search"),
    ("LookupND", "DiagnosticForOutOfRangeInput"): ("None", "Warning", "Error"),
    ("LookupND", "TableSource"): ("Dialog", "Input port"),
    ("LookupND", "BreakpointsDataSource"): ("Dialog", "Input port"),
    ("LookupND", "InternalRulePriority"): ("Speed", "Precision"),
    ("Prelookup", "BreakpointsSpecification"): ("Explicit values", "Even spacing", "Breakpoint object"),
    ("Prelookup", "IndexSearchMethod"): ("Evenly spaced points", "Linear search", "Binary search"),
    ("Prelookup", "DiagnosticForOutOfRangeInput"): ("None", "Warning", "Error"),
    ("Prelookup", "TableSource"): ("Dialog", "Input port"),
    ("Prelookup", "BreakpointsDataSource"): ("Dialog", "Input port"),
    ("Prelookup", "InternalRulePriority"): ("Speed", "Precision"),
    ("PrelookupF", "BreakpointsSpecification"): ("Explicit values", "Even spacing", "Breakpoint object"),
    ("PrelookupF", "IndexSearchMethod"): ("Evenly spaced points", "Linear search", "Binary search"),
    ("PrelookupF", "DiagnosticForOutOfRangeInput"): ("None", "Warning", "Error"),
    ("PrelookupF", "TableSource"): ("Dialog", "Input port"),
    ("PrelookupF", "BreakpointsDataSource"): ("Dialog", "Input port"),
    ("PrelookupF", "InternalRulePriority"): ("Speed", "Precision"),
    ("InterpolationUsingPrelookup", "BreakpointsSpecification"): ("Explicit values", "Even spacing", "Breakpoint object"),
    ("InterpolationUsingPrelookup", "IndexSearchMethod"): ("Evenly spaced points", "Linear search", "Binary search"),
    ("InterpolationUsingPrelookup", "DiagnosticForOutOfRangeInput"): ("None", "Warning", "Error"),
    ("InterpolationUsingPrelookup", "TableSource"): ("Dialog", "Input port"),
    ("InterpolationUsingPrelookup", "BreakpointsDataSource"): ("Dialog", "Input port"),
    ("InterpolationUsingPrelookup", "InternalRulePriority"): ("Speed", "Precision"),
    ("Delay", "DiagnosticForDelayLength"): ("None", "Warning", "Error"),
    ("Delay", "DelayLengthSource"): ("Dialog", "Input port"),
    ("Delay", "InitialConditionSource"): ("Dialog", "Input port"),
    ("DiscreteIntegrator", "InitialConditionSource"): ("internal", "external"),
    ("ShiftArithmetic", "DiagnosticForOORShift"): ("None", "Warning", "Error"),
    ("ShiftArithmetic", "BitShiftNumberSource"): ("Dialog", "Input port"),
    ("ExtractBits", "outScalingMode"): ("Preserve fixed-point scaling", "Treat bit field as an integer"),
    # (ExtractBits bitsToExtract keeps its own exact check: a masked popup falls back to 'Upper half' for other
    #  text -- recordings -- which the engine refuses rather than model)
    ("Downsample", "RateOptions"): ("Allow multirate processing", "Enforce single-rate processing"),
    ("Upsample", "RateOptions"): ("Allow multirate processing", "Enforce single-rate processing"),
    ("MultiportSwitch", "DataPortOrder"): ("Zero-based contiguous", "One-based contiguous", "Specify indices"),
    ("MultiportSwitch", "DataPortForDefault"): ("Last data port", "Additional data port"),
    ("MultiportSwitch", "DiagnosticForDefault"): ("None", "Warning", "Error"),
    ("Mux", "DisplayOption"): ("bar", "signals", "none"),
    # CIC filter type, Demux display, Selector run-time range checks
    ("CICDecimation", "ftype"): ("Decimator", "Zero-latency decimator"),
    ("Demux", "DisplayOption"): ("bar", "none"),
    ("FIRDecimation", "roundingMode"): ("Ceiling", "Convergent", "Floor", "Nearest", "Round", "Simplest", "Zero"),
    ("FIRInterpolation", "roundingMode"): ("Ceiling", "Convergent", "Floor", "Nearest", "Round", "Simplest", "Zero")}
# option lists not known to be complete in Simulink: other text is refused as not supported (it may be valid)
INCOMPLETE_ENUMS = {("Downsample", "RateOptions"), ("Upsample", "RateOptions"), ("Mux", "DisplayOption"),
                    ("Selector", "IndexOptions")}
# on / off flags among the "no effect" parameters -- any other value stops Simulink at set_param
ON_OFF_FLAGS = ("includeCurrent", "ShowOutputPort", "SaturateOnIntegerOverflow", "ZeroCross", "LockScale", "UseCircularBuffer", "StateMustResolveToSignalObject",
                "MustResolveToSignalObject", "ShowName", "DropShadow", "HideAutomaticName",
                # every on / off parameter the engine reads or lists as ignored
                "BeginIndexSearchUsingPreviousIndexResult", "Deterministic", "Integrity", "LimitOutput",
                "RemoveProtectionIndex", "RemoveProtectionInput", "RequireIndexFractionAsBus", "UseBitMask",
                "UseLastBreakpoint", "UseLastTableValue", "ValidIndexMayReachLast", "overflowMode", "VectorParams1D",
                "AllowDiffInputSizes", "AllPortsSameDT", "ShowEnablePort", "PreventDirectFeedthrough", "IgnoreLimit",
                "ShowSaturationPort", "ShowStatePort", "UseOneInputPortForAllInputData",
                "ApplyFullPrecisionForLinearInterpolation", "ApplyAccurateEfficientRounding",
                "SupportTunableTableSize", "SupportTunableSize", "InputSameDT", "RemoveDelayLengthCheckInGeneratedCode",
                "CheckOORBitShift", "LinearizeAsGain", "RuntimeRangeChecks", "OutputSaturationStatus")


LOOKUP_TYPES_ENUM = ("Lookup1D", "Lookup2D", "LookupND", "Prelookup", "PrelookupF", "InterpolationUsingPrelookup")


def resolve_option(text, options):
    """Simulink's reading of an enumerated parameter: the canonical option, or None (rejected)."""
    t = str(text).strip().lower()
    exact = [o for o in options if o.lower() == t]
    if exact:
        return exact[0]
    pref = [o for o in options if t and o.lower().startswith(t)]
    return pref[0] if len(pref) == 1 else None



_ROW_RUN = re.compile(r"[ \t,;\x00]*\x00[ \t,;\x00]*")


def param_text_rules(bt, k, v, where):
    """recordings (S1 / N1 / F1 / U2), the one reading of a block parameter's text, before any reader strips or
    parses it (canonicalize_design for the lowerings that read the raw block, and Design):
      * a character outside printable ASCII (other than tab / CR / LF / VT / FF) -> BAD_PARAM (Simulink rejects it,
        recordings);
      * a '%' comment or a double-quoted string -> UNSUPPORTED (MATLAB evaluates them; not read here -- they were
        refused as a malformed request, U2);
      * CR / LF at either end are trimmed, VT / FF at the end too (Simulink runs '\n3', '3\r\n', '3\x0b': N1 / H2);
      * a CR / LF inside the text: outside brackets it ends the MATLAB statement -> BAD_PARAM (Simulink
        BlkParamEvalErr: '2\n+1', '(\n3)', N1); directly inside [...] it separates rows, so it is written as ';'
        ('[1\n2\n3]' is the column [1;2;3], S1: a Discrete FIR read it as a row); elsewhere -> UNSUPPORTED.
    Enumerated options keep their own rules (resolve_enums); cosmetic text (Description ...) and the engine's own
    '_' parameters are not touched. Returns the text to read."""
    if not isinstance(v, str) or k.startswith("_") or k in COSMETIC_COMMON:
        return v
    import option_validation
    if k in option_validation.TEXT_PARAMS:
        # recorded: program text, not a parameter value (a MATLAB Function's Script: lines, '%' comments, quotes are its
        # own syntax, read by engine/fxmlfn.py; callbacks)
        return v
    loc = "%s.%s" % (where, k)
    stack, out, prev, i, n = [], [], "", 0, len(v)
    in_str = False
    while i < n:
        ch = v[i]
        o = ord(ch)
        if in_str:
            if ch == "'":
                if i + 1 < n and v[i + 1] == "'":
                    out.append("''")
                    i += 2
                    continue
                in_str = False
            elif ch in "\r\n":
                raise GraphError("UNSUPPORTED", loc, "%s %r: a line break inside a quoted text is not supported" % (k, v[:60]))
            elif o > 126 or (o < 32 and ch != "\t"):
                raise GraphError("BAD_PARAM", loc, "%s %r has a character outside printable ASCII (a no-break space, "
                                 "a control character ...): Simulink rejects the value" % (k, v[:60]))
            out.append(ch)
            prev = ch
            i += 1
            continue
        if ch == "%":
            raise GraphError("UNSUPPORTED", loc, "%s %r: a MATLAB comment ('%%') in a parameter is not read here; "
                             "write the value without it" % (k, v[:60]))
        if ch == '"':
            raise GraphError("UNSUPPORTED", loc, "%s %r: double-quoted text is not read here; write it with single "
                             "quotes" % (k, v[:60]))
        if o > 126 or (o < 32 and ch not in "\t\r\n\x0b\x0c"):
            if not "".join(out).strip() and ch.isspace():
                # a no-break space BEFORE the value runs in Simulink (BitMask '\xa03'); after or inside
                # it Simulink rejects the value -- the leading form is not recorded broadly: not supported
                raise GraphError("UNSUPPORTED", loc, "%s %r: a non-ASCII blank before the value is not supported "
                                 "here; remove it" % (k, v[:60]))
            raise GraphError("BAD_PARAM", loc, "%s %r has a character outside printable ASCII (a no-break space, a "
                             "control character ...): Simulink rejects the value" % (k, v[:60]))
        if ch == "'":
            # a quote after a value is the transpose; otherwise a string starts
            if not (prev and (prev.isalnum() or prev in ")]}'_.")):
                in_str = True
        elif ch in "([{":
            stack.append(ch)
        elif ch in ")]}":
            if stack:
                stack.pop()
        out.append(ch)
        if not ch.isspace():
            prev = ch
        i += 1
    t = "".join(out)
    # ends: CR / LF trimmed (with the blanks next to them), VT / FF trimmed at the end
    if t.lstrip(" \t\r\n")[:1] in ("\x0b", "\x0c") and t.strip(" \t\r\n"):
        # a leading VT / FF: recordings runs '\x0b3' / '\x0c3' on Delay / Gain / Saturation / Constant,
        # show an error for BitMask '\x0bint8(3)' -- not one rule, so not taken (a trailing one runs)
        raise GraphError("UNSUPPORTED", loc, "%s %r: a leading vertical tab / form feed is not supported here (Simulink "
                         "runs some such values and rejects others); remove it" % (k, v[:60]))
    lead_ = t.lstrip(" \t")
    if lead_[:1] in ("\r", "\n"):
        body_ = t.strip(" \t\r\n\x0b\x0c")
        if re.match(r"^\d+(\.\d*)?$", body_ or "x") is None:
            # a leading line break is trimmed by Simulink before a plain number only ('\n3' runs);
            # before an expression, a parenthesis or a bracket ('\n2+1', '\n(3)', '\n[3]', FIR '\n[1 2 3]')
            # Simulink stops (BlkParamEvalErr); a signed number after it is not recorded
            if re.match(r"^[+-]?\d+(\.\d*)?([eE][+-]?\d+)?$", body_):
                raise GraphError("UNSUPPORTED", loc, "%s %r: a line break before a signed number or an exponent is "
                                 "not recorded; remove it" % (k, v[:60]))
            raise GraphError("BAD_PARAM", loc, "%s %r: a line break before an expression -- Simulink cannot evaluate "
                             "it (BlkParamEvalErr); only a plain number may follow a leading line break" % (k, v[:60]))
    ws_ = " \t\r\n\x0b\x0c"
    t2 = t.strip(ws_) if t and (t.strip(" \t")[:1] in "\r\n\x0b\x0c" or t.strip(" \t")[-1:] in "\r\n\x0b\x0c") else t
    if "\x0b" in t2 or "\x0c" in t2:
        raise GraphError("UNSUPPORTED", loc, "%s %r: a vertical tab / form feed inside the value is not supported"
                         % (k, v[:60]))
    if "\r" not in t2 and "\n" not in t2:
        return t2
    # inner line breaks: where are they?
    res, stack = [], []
    j, m = 0, len(t2)
    in_str = False
    prev = ""
    while j < m:
        ch = t2[j]
        if in_str:
            if ch == "'":
                in_str = False
            res.append(ch)
            j += 1
            continue
        if ch == "'" and not (prev and (prev.isalnum() or prev in ")]}'_.")):
            in_str = True
        elif ch in "([{":
            stack.append(ch)
        elif ch in ")]}" and stack:
            stack.pop()
        if ch in "\r\n":
            if ch == "\r" and j + 1 < m and t2[j + 1] == "\n":
                j += 1
            if "".join(res).rstrip().endswith("..."):
                # '2 ...\n+1' is a MATLAB line continuation (Simulink gives 3) -- not read here
                raise GraphError("UNSUPPORTED", loc, "%s %r: a line continuation ('...') is not supported here; write "
                                 "the value on one line" % (k, v[:60]))
            if "[" not in stack and "{" not in stack:
                # outside brackets (also inside parentheses: '(\n3)', 'int8(\n3)' -- recordings error)
                raise GraphError("BAD_PARAM", loc, "%s %r: a line break inside the expression ends the MATLAB "
                                 "statement; Simulink cannot evaluate it (BlkParamEvalErr)" % (k, v[:60]))
            if stack[-1] != "[":
                raise GraphError("UNSUPPORTED", loc, "%s %r: a line break inside %s ... is not supported"
                                 % (k, v[:60], stack[-1]))
            res.append("\x00")
            prev = ";"
            j += 1
            continue
        res.append(ch)
        if not ch.isspace():
            prev = ch
        j += 1
    t3 = "".join(res)

    def row(mo):
        a, b = mo.start(), mo.end()
        if "," in mo.group(0) and mo.group(0).index(",") > mo.group(0).index("\x00"):
            # a comma after the line break ('[1\n,2]'): not recorded. Before it ('[1,\n2,\n3]') recordings
            # reads a column, the same as '[1\n2\n3]' (Discrete FIR, Gain, 1-D Lookup table / breakpoints)
            raise GraphError("UNSUPPORTED", loc, "%s %r: a comma after a line break inside [...] is not supported"
                             % (k, v[:60]))
        before = t3[:a].rstrip()[-1:]
        after = t3[b:].lstrip()[:1]
        if before == "[" or after == "]":
            return " "     # a line break next to a bracket adds no row ('[\n1 2\n]' is a row)
        return ";"
    return _ROW_RUN.sub(row, t3)

def canonicalize_design(design):
    """Enumerated parameters of every block in their canonical spelling, before any lowering reads them (the DTI
    state-port and reset lowerings read the raw block)."""
    if not (isinstance(design, dict) and isinstance(design.get("blocks"), dict)):
        return
    for n, b in design["blocks"].items():
        if isinstance(b, dict) and b.get("type") == "MultiportSwitch" and "zeroidx" in b and "DataPortOrder" not in b:
            # the legacy Multiport Switch parameter zeroidx ('on' / 'off') is a
            # spelling of DataPortOrder (zero- / one-based contiguous data ports); it was "unknown parameter"
            z_ = str(b.pop("zeroidx")).strip().lower()
            if z_ not in ("on", "off"):
                raise GraphError("BAD_PARAM", "design.blocks.%s.zeroidx" % n, "zeroidx must be 'on' or 'off'")
            b["DataPortOrder"] = "Zero-based contiguous" if z_ == "on" else "One-based contiguous"
        if isinstance(b, dict) and isinstance(b.get("type"), str):
            w_ = "design.blocks." + str(n)
            body = {k: (v if (ENUM_OPTIONS.get((b["type"], k)) or ENUM_OPTIONS.get((None, k)) or
                              k in ON_OFF_FLAGS) else param_text_rules(b["type"], k, v, w_))
                    for k, v in b.items() if k != "type"}
            b.update(resolve_enums(b["type"], body, w_))


def resolve_enums(bt, user, where):
    out = dict(user)
    for k, v in user.items():
        opts = ENUM_OPTIONS.get((bt, k)) or ENUM_OPTIONS.get((None, k))
        if opts is None and re.match(r"^BreakpointsForDimension\d+Source$", k) and bt in LOOKUP_TYPES_ENUM:
            opts = ("Dialog", "Input port")
        if opts is not None and isinstance(v, str):
            t = v.strip().lower()
            exact = [o for o in opts if o.lower() == v.lower()]
            if exact:
                out[k] = exact[0]
                continue
            if [o for o in opts if o.lower() == t]:
                # surrounding spaces ('<= '): whether Simulink trims them is not recorded
                raise GraphError("UNSUPPORTED", "%s.%s" % (where, k), "%s %s=%r: surrounding spaces are not "
                                 "supported here; write the option exactly" % (bt, k, v))
            if t and any(o.lower().startswith(t) for o in opts):
                # Simulink takes some abbreviations ('u2 >=') and rejects others ('Stored'): not modelled
                raise GraphError("UNSUPPORTED", "%s.%s" % (where, k), "%s %s=%r: an abbreviated option is not "
                                 "supported here; write it in full (%s)" % (bt, k, v, " / ".join(opts)))
            if (bt, k) in INCOMPLETE_ENUMS:
                raise GraphError("UNSUPPORTED", "%s.%s" % (where, k), "%s %s=%r is not supported here (known: %s)"
                                 % (bt, k, v, " / ".join(opts)))
            raise GraphError("BAD_PARAM", "%s.%s" % (where, k), "%s %s=%r: Simulink rejects it (not one of %s)"
                             % (bt, k, v, " / ".join(opts)))
        elif k in ON_OFF_FLAGS and isinstance(v, str):
            if v.lower() in ("on", "off"):
                out[k] = v.lower()
            elif v.strip().lower() in ("on", "off", "yes", "no") or v.strip().lower() == "of":
                # ('of' -- recordings: Simulink takes the unique prefix as off)
                # surrounding spaces, or 'yes' / 'no' (Simulink reads 'yes' as on) -- not modelled
                raise GraphError("UNSUPPORTED", "%s.%s" % (where, k), "%s %s=%r is not supported here; write 'on' "
                                 "or 'off'" % (bt, k, v))
            else:
                raise GraphError("BAD_PARAM", "%s.%s" % (where, k), "%s %s=%r: Simulink rejects it (on / off)"
                                 % (bt, k, v))
    return out


COSMETIC_COMMON = frozenset(("Position", "ZOrder", "BackgroundColor", "ForegroundColor", "ShowName", "NamePlacement",
                             "FontName", "FontSize", "FontWeight", "FontAngle", "Orientation", "DropShadow",
                             "HideAutomaticName", "Description", "Tag", "AttributesFormatString", "BlockMirror",
                             "BlockRotation", "Priority"))
CODEGEN_STATE = frozenset(("RTWStateStorageTypeQualifier", "CodeGenStateStorageTypeQualifier", "StateName",
                           "StateStorageClass", "StateSignalObject"))
STATEFUL = ("UnitDelay", "Delay", "DiscreteIntegrator", "DiscreteFir", "DiscreteTransferFcn", "DiscreteFilter",
            "Memory", "TappedDelay")
# recordings (Simulink R2026b, the reviewer's designs): values of "no effect" parameters that stop the model
STOPPING_VALUES = {
    "VarSizeSig": (("Yes",), "UNSUPPORTED", "variable-size signals are not supported (Simulink cannot propagate a "
                   "variable size from a fixed-size source)"),
    "MustResolveToSignalObject": (("on",), "BAD_PARAM", "Simulink rejects it: there is no signal object to resolve "
                                  "to (SigPropPrmsMustResolveError)"),
    "StateMustResolveToSignalObject": (("on",), "BAD_PARAM", "Simulink rejects it: there is no signal object to "
                                       "resolve to (SigPropPrmsMustResolveError)"),
    "DataMode": (("message",), "BAD_PARAM", "Simulink rejects it on this port (BusElPorts:ParameterNotSupported)"),
    "BusVirtuality": (("nonvirtual",), "BAD_PARAM", "Simulink rejects it on this port "
                      "(BusElPorts:ParameterNotSupported)")}
DESIGN_RANGE_RE = re.compile(r"^(OutMin|OutMax|ParamMin|ParamMax|CoefMin|CoefMax|TableMin|TableMax|OutputMin|"
                             r"OutputMax|BreakpointMin|BreakpointMax|BreakpointsMin|BreakpointsMax|"
                             r"BreakpointsForDimension\d+(Min|Max))$")


# recordings: 'Inherit: Keep LSB' on a Sum is run by Simulink; rules a block does not offer stay BAD_TYPE
# ('Inherit: Same as first input' on a Gain), so only the Sum / Product word-length rules are listed
TYPE_INHERIT_RULES = ("Inherit: Keep MSB", "Inherit: Keep LSB", "Inherit: Match scaling")


def _type_text_error(loc, v):
    """recordings: a data type written as a MATLAB type expression Simulink evaluates (fixdt('double'),
    float('single'), numerictype('double'), fixdt('int8') ...) is a valid design the engine does not take
    (UNSUPPORTED, not_supported); other strings stay BAD_TYPE (a malformed request)."""
    if isinstance(v, str) and TYPE_SLOPE_RE.match(v):
        return GraphError("UNSUPPORTED", loc, "data type %r uses slope / bias scaling, which is not supported here; "
                          "write a binary-point type fixdt(s, w, f)" % v)
    if isinstance(v, str) and TYPE_CALL_RE.match(v):
        return GraphError("UNSUPPORTED", loc, "data type expression %r is not supported here; write it as fixdt(s, w, "
                          "f), a built-in integer name (int8 ...), boolean, or double / single" % v)
    if isinstance(v, str):
        m = TYPE_FIXDT_EXPR_RE.match(v)
        plain = m and not m.group(4) and all(re.match(r"^\s*\d+\s*$", g) for g in m.groups()[:3])
        vs = v.strip()
        # further spellings Simulink evaluates -- any fixdt(...) that is not three integer literals
        # ('(fixdt(1,16,4))', 'fixdt(1,16,pi)', 'fixdt (1,8,2)', 'fixdt(1,16,4) % c', ...) and the named inherit rules
        fixdt_expr = "fixdt" in vs and "_" not in vs and not plain and not re.match(r"^fixdt\(\s*-?\d+\s*,\s*-?\d+\s*,\s*-?\d+\s*\)$", vs)
        if re.match(r"^fixdt\s+\(", vs):
            fixdt_expr = True   # 'fixdt (1,8,2)': MATLAB allows the space
        if vs.lower() in TYPE_NAMES_CI and vs not in TYPE_NAMES_CI:
            fixdt_expr = True
        if vs in TYPE_INHERIT_RULES or fixdt_expr or \
                vs in TYPE_NAME_UNSUP or TYPE_CTOR_RE.match(v) or (m and not plain):
            return GraphError("UNSUPPORTED", loc, "data type %r is a valid Simulink type this engine does not take "
                              "here; write fixdt(s, w, f) with integer literals, a built-in integer name up to 32 "
                              "bits (int8 ...), or boolean" % v)
    return GraphError("BAD_TYPE", loc, "unsupported data type string %r" % v)


def _mrfilter_recorded(bt, p, in_types):
    """Whether a multirate filter fed by a loop is one of the recorded configurations, on the recorded loop type."""
    if any(t != MRFILTER_LOOP_TYPE for t in in_types):
        return False
    return any(all(str(p.get(k, "")).strip() == v for k, v in cfg.items()) for cfg in MRFILTER_ON_LOOP.get(bt, ()))


MR_BLOCKS = ("Downsample", "Upsample", "RateTransition", "FIRDecimation", "FIRInterpolation", "CICDecimation",
             "CICInterpolation")
def word_class(w):
    """The smallest native word holding w bits (8 / 16 / 32 / 64)."""
    return next((c for c in (8, 16, 32, 64) if w <= c), w)


SUM_RULE_FL_MAX = 300   # ASIC/FPGA internal-rule Sums: fraction lengths beyond this are not recorded
DIV_NET_EXP = (-128, 426)   # net scaling exponents of the recorded divisions (recordings etc.)
BACKPROP = "Inherit: Inherit via back propagation"
SAME_ACC = "Inherit: Same as accumulator"
SEED_TRIALS = 64  # choice sequences tried when several blocks could start an inherited-type cycle
CYCLE_MODE = "check"  # "check": seed once, assumed loop-back types must hold (Simulink-like); "iterate": fixed point


def _lower_prelookup(blocks, lines):
    """A Prelookup has two outputs, k (port 1) and f (port 2): port 2 is an internal block PrelookupF ("P\x1df") fed
    by the same input; lines from port 2 leave it instead."""
    pls = [n_ for n_, b_ in blocks.items() if isinstance(b_, dict) and b_.get("type") == "Prelookup"]
    if not pls:
        return blocks, lines
    blocks, out = dict(blocks), []
    for n_ in pls:
        if str(blocks[n_].get("FractionDataTypeStr", "Inherit")).strip().startswith("Inherit"):
            # the library's fraction type follows the hardware (host ufix32_En31; ASIC / FPGA
            # ufix<w>_En<w-1> of the input's word length) -- resolved in _prelookup_check, where both are known
            # (written as the rule text: the engine's check and kernel read it, fxcore.prelookup_fraction_type)
            blocks[n_] = dict(blocks[n_], FractionDataTypeStr="Inherit: Inherit via internal rule", _FracDefault="1")
        blocks[n_ + "\x1df"] = dict(blocks[n_], type="PrelookupF")
    for ln in lines:
        if isinstance(ln, dict) and ln.get("src") in pls and str(ln.get("sport", 1)).strip() == "2":
            out.append(dict({k: v for k, v in ln.items() if k != "sport"}, src=ln["src"] + "\x1df"))
        elif isinstance(ln, dict) and ln.get("src") in pls and str(ln.get("sport", 1)).strip() not in ("1", ""):
            raise GraphError("BAD_CONNECTION", "design.blocks." + ln["src"], "Prelookup %s has two outputs (k, f); a "
                             "line leaves its port %s" % (ln["src"], ln.get("sport")))
        else:
            out.append(ln)
        if isinstance(ln, dict) and ln.get("dst") in pls:
            out.append(dict(ln, dst=ln["dst"] + "\x1df"))
    return blocks, out


def state_port_refusal(src):
    """recordings (refuse this shape first): the evaluation ignored the integrator's
    SaturateOnIntegerOverflow, the Gain's saturation, rounding mode and sample time. Also raised before the vector
    lowering reads the line (a Mux elsewhere in the design gave INTERNAL on sport 'state')."""
    return GraphError("UNSUPPORTED", "design.blocks.%s.ShowStatePort" % src, "the integrator state port is not "
                      "supported (the state-port feedback through a Gain was not reproduced for every "
                      "parameter of the integrator and the Gain); take the integrator output instead, or record the "
                      "shape with each of those parameters to open it")


def _lower_dti_statefb(blocks, lines):
    """a Discrete-Time Integrator (Forward Euler, reset, external initial condition, state
    port) whose state port feeds a Gain that feeds its own initial-condition port is evaluated as one block
    (DTIStateFB: y) and, when the Gain has other destinations, a second one for the Gain (DTIStateFBGain) -- the loop
    through the state port has no algebraic part. Other state-port uses are refused."""
    sp_lines = [ln for ln in lines if isinstance(ln, dict) and str(ln.get("sport", "1")).strip().lower() == "state"]
    if not sp_lines:
        return blocks, lines
    if not STATEPORT_FEEDBACK:
        raise state_port_refusal(sp_lines[0].get("src"))
    blocks, lines = dict(blocks), list(lines)
    for ln in sp_lines:
        d, g = ln.get("src"), ln.get("dst")
        bd, bg = blocks.get(d), blocks.get(g)
        where = "design.blocks.%s" % d
        ok = isinstance(bd, dict) and bd.get("type") == "DiscreteIntegrator" and isinstance(bg, dict) and \
            bg.get("type") == "Gain" and str(bd.get("ShowStatePort", "off")) == "on" and \
            str(bd.get("InitialConditionSource", "internal")) == "external" and \
            str(bd.get("ExternalReset", "none")) in ("rising", "falling", "either", "level") and \
            str(bd.get("IntegratorMethod", "Integration: Forward Euler")).endswith("Forward Euler") and \
            str(bd.get("LimitOutput", "off")) != "on"
        icp = 3
        into = [x for x in lines if isinstance(x, dict) and x.get("src") == g]
        ok = ok and any(x.get("dst") == d and int(x.get("dport", 1)) == icp for x in into)
        try:
            gv = Fraction(str(bg.get("Gain", "1")).strip())
        except (ValueError, ZeroDivisionError):
            gv = None
        ok = ok and gv is not None and gv != 0 and (abs(gv).numerator & (abs(gv).numerator - 1)) == 0 and \
            (abs(gv).denominator & (abs(gv).denominator - 1)) == 0
        if not ok:
            raise GraphError("UNSUPPORTED", where + ".ShowStatePort", "the integrator state port is supported only "
                             "as recorded: Forward Euler with a reset (rising / falling / either / level) and an external "
                             "initial condition, the state port through a Gain of a power of two (+-) into this "
                             "integrator's initial-condition port; other uses are not")
        params = {k: v for k, v in bd.items() if k not in ("type", "ShowStatePort", "InitialConditionSource")}
        params.update(type="DTIStateFB", _SFGain=str(gv), _SFGainOut=str(bg.get("OutDataTypeStr", fc._INHERIT_RULE)))
        blocks[d] = params
        others = [x for x in into if not (x.get("dst") == d)]
        lines = [x for x in lines if x is not ln and not (x.get("src") == g and x.get("dst") == d)]
        ins_d = [x for x in lines if isinstance(x, dict) and x.get("dst") == d]
        # the Gain stays a block of its own (its compiled type and any other destination), computed from the same
        # inputs; an unused Gain output is simply not read
        blocks[g] = dict(params, type="DTIStateFBGain")
        lines += [dict(x, dst=g) for x in ins_d]
    return blocks, lines


class GraphError(Exception):
    def __init__(self, code, where, message):
        Exception.__init__(self, message)
        self.code, self.where, self.message = code, where, message


def _ninputs(btype, p, where):
    for k in REQUIRED.get(btype, ()):
        if btype == "ShiftArithmetic" and k == "BitShiftNumber" and \
                str(p.get("BitShiftNumberSource", "Dialog")).strip() == "Input port":
            continue   # the shift comes from port 2
        if btype in ("Lookup1D", "Lookup2D") and k == "Table" and ("T" in lut_ports(p) or
                                                                   str(p.get("TableSource", "Dialog")).strip() == "Input port"):
            continue   # the table comes from a port
        if k not in p:
            lib_ = REQUIRED_LIBRARY_TEXT.get((btype, k))
            if lib_ is not None:
                # "required" was untrue -- Simulink runs the library block with its default
                raise GraphError("UNSUPPORTED", "%s.%s" % (where, k), "%s left out: the library default %s is not "
                                 "supported here; give %s" % (k, lib_, k))
            raise GraphError("MISSING_FIELD", "%s.%s" % (where, k), "parameter %s is required for %s" % (k, btype))
    if btype in MR_BLOCKS:
        return 1
    if btype in ("BitSlice", "BitConcat", "BitShift", "BitRotate", "BitSet", "BitClear"):   # recordings (fxhdl.py)
        import fxhdl
        return fxhdl.bit_ninputs(btype, p, where)
    if btype in ("Hold", "Gate"):
        return 2
    if btype in ("PLCore", "PLInterp"):   # recordings (engine/fxprelookup.py): x; index, fraction
        return 1 if btype == "PLCore" else (4 if '"dims": 2' in str(p.get("Params", "")) else 2)
    if btype == "HDLFIRCore":   # recordings (engine/fxhdlfir.py): data, valid [, reset]
        return 3 if '"ResetInputPort": "on"' in str(p.get("Params", "")) else 2
    if btype == "RAMCore":   # recordings (engine/fxhdl.py _ram_deep): din, write address, write enable, port address
        return 4
    if btype == "CordicCore":   # recordings (engine/fxcordic.py): re, im -- or magnitude, angle
        return 2
    if btype == "NCOCore":   # recordings (engine/fxhdlnco.py): [increment,] valid
        import fxhdl
        return fxhdl.nco_ninputs(p)
    if btype == "MLFn":   # recordings (engine/fxhdl.py lowers a MATLAB Function block into one MLFn per output)
        import fxhdl
        return len(fxhdl.ml_inputs(p, where))
    if btype == "Trigonometry":
        return 2 if str(p.get("Operator", "sin")).strip() == "atan2" else 1
    if btype in ("Prelookup", "PrelookupF"):
        return 1
    if btype == "InterpolationUsingPrelookup":
        nd_ = str(p.get("NumberOfTableDimensions", "2")).strip()
        if nd_ not in ("1", "2"):
            raise GraphError("UNSUPPORTED", where + ".NumberOfTableDimensions", "Interpolation Using Prelookup is "
                             "recorded with 1 or 2 dimensions, got %r" % nd_)
        return 2 * int(nd_)
    if btype == "LookupND":
        nd_ = str(p.get("NumberOfTableDimensions", "3")).strip()
        if nd_ not in ("3", "4"):
            raise GraphError("UNSUPPORTED", where + ".NumberOfTableDimensions", "LookupND answers 3 or 4 dimensions "
                             "(use Lookup1D / Lookup2D for 1 / 2), got %r" % nd_)
        return int(nd_)
    if btype == "DelayPort":
        # length and initial condition both from ports is u, d, x0 (Simulink runs it; refused later as
        # not recorded, not as a missing port)
        # Simulink's port order u, d, enable, reset, x0: count each one present (refused later as not recorded)
        return 1 + (str(p.get("DelayLengthSource", "Dialog")) == "Input port") + \
            (str(p.get("ShowEnablePort", "off")).strip() == "on") + \
            (str(p.get("ExternalReset", "None")).strip() not in ("None", "none", "")) + \
            (str(p.get("InitialConditionSource", "Dialog")) == "Input port")
    if btype in ("DTIStateFB", "DTIStateFBGain"):
        return 2   # u and the reset (the initial condition is the fed-back state)
    if btype == "ShiftArithmetic" and str(p.get("BitShiftNumberSource", "Dialog")).strip() == "Input port":
        return 2
    if btype in ("Lookup1D", "Lookup2D") and lut_ports(p):
        # u (2-D: u1, u2), the table elements, the breakpoint elements
        return (2 if btype == "Lookup2D" else 1) + sum(lut_ports(p).values())
    if btype == "DiscreteIntegrator":   # u, then the reset and the external initial condition when present
        return 1 + (str(p.get("ExternalReset", "none")).strip() != "none") + \
            (str(p.get("InitialConditionSource", "internal")).strip() == "external")
    if btype in ("Gain", "DataTypeConversion", "Saturation", "Abs", "UnaryMinus", "UnitDelay", "Outport",
                 "Delay", "ShiftArithmetic", "CompareToConstant", "DiscreteFir", "DetectChange", "DetectIncrease",
                 "DetectDecrease", "Signum", "Rounding", "ExtractBits", "DiscreteIntegrator", "Lookup1D", "Sqrt",
                 "CompareToZero", "DeadZone", "WrapToZero", "RateLimiter"):
        return 1
    if btype == "Lookup2D":
        return 2
    if btype in ("Inport", "Constant", "CounterLimited", "CounterFreeRunning"):
        return 0
    if btype == "BitwiseOperator":
        try:
            n = int(p["NumInputPorts"])
        except ValueError:
            raise GraphError("BAD_PARAM", where + ".NumInputPorts", "number of inputs required")
        if not (1 <= n <= 1024):
            # recordings (one native run): NumInputPorts 0 with the mask on -- Simulink refused it
            # at set_param, the engine used one port and answered
            raise GraphError("BAD_PARAM", where + ".NumInputPorts", "1..1024 inputs (NumInputPorts %d)" % n)
        if p.get("UseBitMask") == "on":
            n = 1
        if not (1 <= n <= 1024):
            raise GraphError("BAD_PARAM", where + ".NumInputPorts", "1..1024 inputs")
        if str(p.get("logicop", "")).strip() == "NOT" and n != 1:
            # NOT has one port in Simulink (build error with two
            # wires); the engine accepted and ignored input 2
            raise GraphError("BAD_PARAM", where + ".NumInputPorts", "Bitwise NOT has one input (NumInputPorts %d)" % n)
        return n
    if btype == "Switch":
        return 3
    if btype == "LogicalOperator":
        if p["Operator"] == "NOT":
            return 1
        try:
            n = int(p["Inputs"])
        except ValueError:
            raise GraphError("BAD_PARAM", where + ".Inputs", "number of inputs required")
        if n == 1:
            # Inputs '1' is valid in Simulink (the operator over the elements of one input; on a
            # vector it is answered by the vector pass); on a scalar signal it is not supported here, not a
            # malformed request
            raise GraphError("UNSUPPORTED", where + ".Inputs", "a Logical Operator with one input on a scalar signal "
                             "is not supported")
        if not (2 <= n <= 1024):
            raise GraphError("BAD_PARAM", where + ".Inputs", "2..1024 inputs")
        return n
    if btype == "MultiportSwitch":
        if p["DataPortOrder"] not in ("One-based contiguous", "Zero-based contiguous", "Specify indices"):
            raise GraphError("BAD_PARAM", where + ".DataPortOrder", "One-based contiguous, Zero-based contiguous or "
                             "Specify indices")
        if p["DataPortOrder"] == "Specify indices":
            # distinct integer indices, one data port each (+ the additional default port)
            try:
                ix = fc.mps_indices(p)
            except ValueError:
                txt_ = str(p.get("DataPortIndices", "")).strip()
                if txt_.startswith("{") and txt_.endswith("}"):
                    # cells Simulink runs ({[1 2],3}: several indices for one port, {1;3}, {1:2,3})
                    raise GraphError("UNSUPPORTED", where + ".DataPortIndices", "DataPortIndices %r: only a cell of "
                                     "single integers, one per data port ({0,3,5}), is supported" % txt_)
                raise GraphError("BAD_PARAM", where + ".DataPortIndices", "a list of integer indices, e.g. {0,3,5}")
            if not str(p.get("DataPortIndices", "")).strip().startswith("{"):
                # recordings (18 / 18): a numeric vector ([7 9]) is rejected (InvCaseConditions); the
                # cell form {7,9} runs
                raise GraphError("PORT_TYPE_INVALID", where + ".DataPortIndices", "DataPortIndices %r: Simulink "
                                 "rejects a numeric vector here (Simulink:blocks:InvCaseConditions); write the "
                                 "indices as a cell, e.g. {%s}" % (p.get("DataPortIndices"), ",".join(map(str, ix))))
            if not ix or len(set(ix)) != len(ix):
                raise GraphError("UNSUPPORTED", where + ".DataPortIndices", "distinct indices are recorded")
            if str(p.get("Inputs", len(ix))).strip() != str(len(ix)):
                raise GraphError("UNSUPPORTED", where + ".Inputs", "Inputs %s with %d DataPortIndices is not recorded "
                                 "(set Inputs to the number of indices)" % (p.get("Inputs"), len(ix)))
            return 1 + len(ix) + (1 if p.get("DataPortForDefault") == "Additional data port" else 0)
        try:
            n = int(p["Inputs"])
        except ValueError:
            raise GraphError("BAD_PARAM", where + ".Inputs", "number of data ports required")
        if not (1 <= n <= 1024):   # unbounded, a huge count exhausted memory
            raise GraphError("BAD_PARAM", where + ".Inputs", "1..1024 data ports")
        return 1 + n + (1 if p["DataPortForDefault"] == "Additional data port" else 0)
    if btype == "Product":
        if "|" in str(p["Inputs"]):
            # '|' (a Sum icon spacer) in a Product's operator string reached the compiled record unstripped
            # and failed there (INTERNAL); how Simulink reads it is not recorded -- refused here, once, for every path
            raise GraphError("UNSUPPORTED", where + ".Inputs", "Product Inputs %r: '|' in a Product's operator string "
                             "is not recorded; not supported" % p["Inputs"])
        txt = str(p["Inputs"]).strip()
        if txt and not txt.isdigit() and not set(txt) - set("*/"):
            if "/" in txt[:2] and txt[:2] == "//":
                raise GraphError("UNSUPPORTED", where + ".Inputs", "'//' has no recordings coverage")
            if txt[0] == "/" and len(txt) > 2:
                # with a leading division Simulink starts from u2 / u1 and rounds to
                # the output type at every step; the engine computes 1 / u1 first (575 vs 154) -- refused
                raise GraphError("UNSUPPORTED", where + ".Inputs", "Product %r: a leading division with more than "
                                 "two operands is evaluated differently; not supported" % txt)
            return len(txt)
    if btype == "RelationalOperator":
        op = str(p.get("Operator", "")).strip()
        if op == "isInf":
            # recordings (rox3): this one-input operator was refused as a malformed request ("input port
            # 2 is not connected"); Simulink runs it -- isInf on fixed-point input is not recorded (isNaN / isFinite
            # are, recordings: rewritten to a Constant in Design.__init__)
            raise GraphError("UNSUPPORTED", where + ".Operator", "Relational Operator %r (one input) is not supported: "
                             "Simulink's output for it has not been recorded; a fixed-point or boolean signal is never "
                             "Inf, so use a boolean Constant 0 instead" % op)
        return 2
    if btype in ("Sum", "Product", "MinMax"):
        txt = str(p["Inputs"]).replace("|", "").strip()
        if txt.isdigit():
            n = int(txt)
        elif btype == "Sum" and txt and not set(txt) - set("+-"):
            n = len(txt)
        elif btype == "Product" and txt and not set(txt) - set("*"):
            n = len(txt)
        else:
            raise GraphError("UNSUPPORTED", where + ".Inputs", "Inputs %r not supported for %s" % (p["Inputs"], btype))
        if not (1 <= n <= 1024):
            raise GraphError("BAD_PARAM", where + ".Inputs", "1..1024 inputs")
        if btype == "Product" and n < 1:
            raise GraphError("BAD_PARAM", where + ".Inputs", "at least one input")
        if btype == "MinMax" and n < 2:
            raise GraphError("UNSUPPORTED", where + ".Inputs", "MinMax: one-input (vector) form not supported")
        return n
    raise GraphError("UNKNOWN_BLOCK_TYPE", where + ".type", "unknown block type %r" % btype)


def _sum_signs(p, n):
    txt = str(p["Inputs"]).replace("|", "").strip()
    return "+" * n if txt.isdigit() else txt


def _fm(p, where):
    rm = p.get("RndMeth", "Floor")
    if rm not in ("Floor", "Ceiling", "Zero", "Nearest", "Round", "Convergent", "Simplest"):
        raise GraphError("UNSUPPORTED_ROUNDING", where + ".RndMeth", "unsupported rounding method %r" % (rm,))
    sat = p.get("SaturateOnIntegerOverflow", "off")
    if sat not in ("on", "off"):
        raise GraphError("BAD_PARAM", where + ".SaturateOnIntegerOverflow", "on or off")
    return fc._blk_fm(p)


def _fixdt(text, where):
    if not isinstance(text, str):
        raise GraphError("BAD_TYPE", where, "data type must be a string")
    t = fc.parse_type(text)   # the one reading: fixdt and int8 .. uint32
    if t is not None:
        s, w, f = t
        if s and w == 1:
            # recordings (F6): Simulink rejects a signed one-bit type (IllegalFixedPointDataType_signed_one_bit)
            raise GraphError("BAD_TYPE", where, "%s is a signed type of one bit, which Simulink does not allow; use "
                             "fixdt(1,2,...) or more bits, or an unsigned fixdt(0,1,...)" % text)
        if s not in (0, 1) or not (1 <= w <= MAX_WL) or (s and w < 2) or abs(f) > 4096:
            raise GraphError("BAD_TYPE", where, "fixdt out of supported range: %s" % text)
    return t


def not_a_number(text, where, what="numeric literal"):
    """The error for a parameter that is not a number: engine/fxnum.py's rule (a MATLAB expression UNSUPPORTED,
    anything else BAD_PARAM), for the parameters read outside the NUMERIC registry."""
    c = fxnum.canonical(text, "any")
    if isinstance(c, fxnum.NumError):
        return GraphError(c.code, where, c.message)
    if c.startswith("["):   # a list where this path reads one value (a vector Gain on a scalar signal)
        return GraphError("UNSUPPORTED", where, "a list %r where one value is used (vector parameters need a vector "
                          "signal)" % c[:60])
    return GraphError("BAD_PARAM", where, "%s required" % what)


def _real(text, where):
    try:
        v = float(text)
    except (TypeError, ValueError):
        raise not_a_number(text, where)
    if v != v or v in (float("inf"), float("-inf")):
        raise GraphError("BAD_PARAM", where, "finite value required")
    return v


def _chk(t, where):
    if t != BOOL and not (1 <= t[1] <= MAX_WL):
        raise GraphError("UNSUPPORTED_WIDTH", where, "resolved word length %d outside 1..%d" % (t[1], MAX_WL))
    return t


RMCODE = {"Floor": 0, "Ceiling": 1, "Zero": 2, "Nearest": 3, "Round": 4, "Convergent": 5}
RELCODE = {"==": 0, "~=": 1, "<": 2, "<=": 3, ">": 4, ">=": 5}
VM_BITS = 126


FAST_BITS = 62  # same bound as engine/fxchain: every intermediate fits int64 with headroom


def _ctok(fl_from, t, fm, wl_from):
    """Cast token 'sh rm s wl sat' for the C core, whether it fits the 126-bit core, and whether it
    fits the 64-bit fast path."""
    s, w, f = t
    sh = fl_from - f
    ok = wl_from + max(0, -sh) <= VM_BITS and w <= VM_BITS and sh <= VM_BITS
    f62 = wl_from + max(0, -sh) <= FAST_BITS and w + 1 <= FAST_BITS and sh <= 60
    return "%d %d %d %d %d" % (sh, RMCODE[fm["rm"]], s, w, 1 if fm["om"] == "Saturate" else 0), ok, f62


# recorded item c: the recorded span of the accumulator Sum with typed exits, field by field (the values each
# field was recorded with; recordings)
SF_IN = {1: ({8, 12, 16}, {2, 4, 8}), 0: ({16}, {8})}          # sfix8_En4; SF sfix8_En2 sfix12_En4 sfix16_En8 ufix16_En8
SF_EXIT1 = {1: (set(range(7, 18)) | {24, 32}, {2, 4, 8, 13, 16}), 0: ({16}, {8})}   # bits; SF 8 .. 32
SF_EXIT2 = {1: ({8, 12, 16, 24, 32}, {2, 4, 8, 16}), 0: ({16}, {8})}                # SF shape 2 / the conversion
SF_CONFLICT_FL = {1, 2}                                         # sfix10_En1 against sfix10_En2


def _sf_span(shape, te, typed, extra, p, words):
    """None inside the recorded span of the item-c accumulator, else the first field outside, as text."""
    def fits(t, span):
        return isinstance(t, tuple) and t[0] in span and t[1] in span[t[0]][0] and t[2] in span[t[0]][1] and \
            0 <= t[2] < t[1]
    if not fits(tuple(te) if isinstance(te, tuple) else te, SF_IN):
        return "external input %s (recorded: fixdt(1,8|12|16, 2|4|8), fixdt(0,16,8))" % (
            _tstr(te) if isinstance(te, tuple) else te)
    signs = str(p.get("Inputs", "++")).replace("|", "").strip()
    if signs not in ("++", "+-"):
        return "Inputs %r (recorded: '++', '+-')" % signs
    if str(p.get("RndMeth", "Floor")).strip() != "Floor" or \
            str(p.get("AccumDataTypeStr", fc._INHERIT_RULE)).strip() != fc._INHERIT_RULE:
        return "RndMeth / accumulator type (recorded: Floor, the internal rule)"
    if words not in ([], [8, 16, 32]):
        return "hardware with native words %s (recorded: 32-bit host, ASIC/FPGA)" % words
    if not typed:
        return None   # only the conversion (recorded with shape 1): the Sum takes the external input's type
    if len(set(typed)) > 1:
        if shape != 1 or len(typed) != 2 or extra or len({t[1] for t in typed}) != 1 or \
                any(t[0] != 1 or t[1] not in SF_EXIT1[1][0] or t[2] not in SF_CONFLICT_FL for t in typed):
            return "typed Outports %s (recorded with different types: two, one word length, fractions 1 and 2)" % \
                ", ".join(_tstr(t) for t in typed)
        return None
    span = SF_EXIT1 if shape == 1 else SF_EXIT2
    if not fits(typed[0], span):
        return "typed Outport %s (recorded: signed %s bits with fraction %s; fixdt(0,16,8))" % (
            _tstr(typed[0]), "7..17, 24, 32" if shape == 1 else "8, 12, 16, 24, 32",
            "2, 4, 8, 13, 16" if shape == 1 else "2, 4, 8, 16")
    if shape == 2 and (len(typed) != 1):
        return "%d typed Outports (recorded: one)" % len(typed)
    if shape == 1 and not (len(typed) == 1 or (len(typed) <= 3 and not extra)):
        return "%d typed Outports with a further branch (recorded: one typed Outport with a branch, or 2..3 alone)" % \
            len(typed)
    return None


def _tstr(t):
    return "boolean" if t == BOOL else "fixdt(%d,%d,%d)" % t


def lut_ports(p):
    """{'T': n, 'B': m} for a 1-D Lookup Table whose table / breakpoints come from ports (the vector lowering's
    _LutPorts, e.g. 'recordings'), in port order (u, then the table, then the breakpoints); {} otherwise."""
    out = {}
    for tok in str(p.get("_LutPorts", "")).split():
        if tok[0] in "TBC":   # 'M' / 'N': a 2-D table's dimension sizes (lut_dims)
            out[tok[0]] = int(tok[1:])
    return out


def lut_dims(p):
    """(m, n) of a 2-D Lookup Table whose data come from ports (the lowering's 'M3 N3'), else None."""
    d = {tok[0]: int(tok[1:]) for tok in str(p.get("_LutPorts", "")).split() if tok[0] in "MN"}
    return (d["M"], d["N"]) if "M" in d and "N" in d else None


def _lut_port_values(p, ts, recs, nu=1):
    """The parameters of one step of a port-sourced 1-D Lookup Table: the port elements'
    exact values written as the table / breakpoints, typed with the ports' own types (Simulink's run-time table and
    breakpoint types are the port signal types: dut_rtp of every recorded design). recs None: a placeholder for type
    inference (zero table, breakpoints 0, 1, 2 ... LSBs)."""
    lp = lut_ports(p)
    dims = lut_dims(p)
    q = {k: v for k, v in p.items() if k != "_LutPorts"}
    port = nu
    for c, key, tkey in (("T", "Table", "TableDataTypeStr"), ("B", "BreakpointsForDimension1",
                                                              "BreakpointsForDimension1DataTypeStr"),
                         ("C", "BreakpointsForDimension2", "BreakpointsForDimension2DataTypeStr")):
        if c not in lp:
            continue
        t = ts[port]
        if recs is None:
            ints = [0] * lp[c] if c == "T" else list(range(lp[c]))
        else:
            ints = [int(recs[port + j]["bin"], 2) - ((1 << t[1]) if t[0] and recs[port + j]["bin"][0] == "1" else 0)
                    for j in range(lp[c])]
        vals = [Fraction(v) / (Fraction(2) ** t[2]) if t[2] >= 0 else Fraction(v) * (Fraction(2) ** -t[2])
                for v in ints]
        if c == "T" and dims:
            # a 2-D table from a matrix signal: elements column-major, rows for the text
            m_, n_ = dims
            q[key] = "[%s]" % ";".join(" ".join(repr(float(vals[j * m_ + i])) for j in range(n_)) for i in range(m_))
        else:
            q[key] = "[%s]" % " ".join(repr(float(v)) for v in vals)
        q[tkey] = "fixdt(%d,%d,%d)" % t
        port += lp[c]
    return q


def _rec(t, n):
    """Stored integer -> recordings-style value record."""
    if t == BOOL:
        return {"bool": int(n)}
    s, w, f = t
    return {"s": s, "wl": w, "fl": f, "bin": format(n & ((1 << w) - 1), "0%db" % w)}


def _unrec_type(out, where):
    if "bool" in out:
        return BOOL
    if "cls" in out:
        if out["cls"] in fc._NAMED_INT:
            return fc._NAMED_INT[out["cls"]]
        hint = ("; a Constant without an output type is double in Simulink, and the Sum / Product / Gain / MinMax / "
                "Switch / Delay it feeds then compute in double too -- give the "
                "Constant a fixed-point OutDataTypeStr or 'Inherit: Inherit via back propagation'"
                if out["cls"] == "double" else "")
        raise GraphError("UNSUPPORTED", where, "%s signals are not supported (fixed point and boolean only)%s"
                         % (out["cls"], hint))
    return (out["s"], out["wl"], out["fl"])


def _unrec(out):
    if "bool" in out:
        return int(out["bool"])
    return fc._decode(out)[3]


def _named_param_type(p, g, hw=None):
    """Gain parameter type with a boolean input: explicit, else fc.bool_gain_param_type (the shortest word
    holding the gain exactly at best precision)."""
    t = fc._named_type(p.get("ParamDataTypeStr", fc._INHERIT_RULE))
    if isinstance(t, tuple):
        return t
    try:
        # the shortest word holding the gain exactly (host: 8 / 16 / 32)
        return fc.bool_gain_param_type(g, fc._asic(hw), p.get("OutDataTypeStr", fc._INHERIT_RULE) == fc._INHERIT_RULE)
    except ValueError as e:
        raise GraphError("UNSUPPORTED", "ParamDataTypeStr", str(e))


# blocks with recordings data but no bit-exact model yet: refused explicitly (the
# Trigonometric Function is answered for CORDIC atan2 only, _trig_check)
PENDING_TYPES = {
    "NCO": "NCO is not supported yet ",
    # real Simulink blocks were answered as UNKNOWN_BLOCK_TYPE (a request error); they are blocks
    # this engine does not support yet (recorded supports Ground, Bias, Math, Tapped Delay, Signal Specification:
    # engine/block_lowering.py -- removed here at the merge of a previous version)
    "Terminator": "Terminator is not supported yet on this engine: leave the signal unconnected or end it in an "
                  "Outport",
    # further real Simulink blocks that were answered as UNKNOWN_BLOCK_TYPE (a request error)
    "Scope": "Scope is not supported (no display here): log the signal with an Outport",
    "Display": "Display is not supported (no display here): log the signal with an Outport",
    "Quantizer": "Quantizer is not supported yet on this engine: use a Data Type Conversion to the quantised type",
    "Relay": "Relay is not supported yet on this engine",
    "Backlash": "Backlash is not supported yet on this engine",
    "Step": "Step is not supported yet on this engine: feed the step as Inport data",
    "Clock": "Clock is not supported (continuous time): feed the time as Inport data",
    "Integrator": "Integrator is a continuous-time block (not supported): use a Discrete-Time Integrator",
    "Lookup_n-D": "Lookup_n-D is the .slx block type: in a JSON design write Lookup1D or Lookup2D",
    "Fcn": "Fcn (an expression block) is not supported yet on this engine: build the expression from blocks",
    "SineCosineLookup": "Lookup Tables Sine / Cosine is not supported yet ",
    # recorded next partitions, refused until recorded (recordings; Discrete Filter / Discrete
    # Transfer Fcn: engine/fxiir.py)
    "SignalFromWorkspace": "Signal From Workspace is not supported in a JSON design: give the data as an Inport's "
                           "input (a saved .slx with it is read as an Inport)"}
HIDDEN_SEP = "\x1f"
DTI_NEW_TYPES = ((1, 16, 8), (1, 12, 4), (1, 8, 2), (0, 12, 6), (1, 20, 12))   # inputs
STATEPORT_FEEDBACK = False   # the state-port feedback lowering is switched off (refused)
PORT_PARAM_TYPES = ((1, 16, 8), (1, 12, 4), (0, 12, 6), (1, 8, 2), (1, 20, 12), (1, 16, 0))   # port-sourced
# parameters: recordings (the first two, host) and recordings (host and ASIC)
DTI_WIDEN_TYPES = ((1, 32, 16), (0, 8, 4), (1, 24, 0), (1, 8, 0))   # inputs
RESERVED_CHARS = ("\x1d", "\x1e", "\x1f")   # vector copies, multirate lowering, hidden C-core slots
INTERNAL_PARAMS = ("_NDRule", "_Clamp", "_GateSteps", "_GateSlot", "_Tie", "_GainVec", "_ICSteps", "_DtiT", "_InWidths", "_OutWidths",
                   "_HalfGridIC", "_RlT", "_VecReduce", "_SelIdx", "_SFGain", "_SFGainOut", "_Pat", "_In", "_InCls", "_MatGain", "_MatGainOut", "_FracDefault", "_MatGainKey", "_ShiftRange", "_ShiftGeneral", "_MatKey", "_LutPorts", "_ColGain")
INTERNAL_TYPES = ("Hold", "Gate", "MLFn", "HDLFIRCore", "NCOCore", "CordicCore", "RAMCore", "PLCore", "PLInterp", "DelayPort",
                  "DTIStateFB", "DTIStateFBGain", "PrelookupF")   # created only by the multirate lowering (engine/fxlower.py) and the recorded lowering (engine/fxhdl.py)


def check_user_design(spec, where="design"):
    """User-facing entry points (CLIs): block names with the reserved control characters and the internal
    parameters that only the engine's own passes may set are refused."""
    if not isinstance(spec, dict) or not isinstance(spec.get("blocks"), dict):
        return
    for key in spec:   # an engine-internal design key (parameter_bounds.VEC_KEY) is not user input
        if not isinstance(key, str) or any(ch in key for ch in RESERVED_CHARS):
            raise GraphError("BAD_PARAM", where, "design key %r contains a reserved control character "
                             "(U+001D..U+001F)" % (key,))
    # unknown keys of the design and of its lines were silently accepted
    extra = sorted(set(spec) - {"blocks", "lines", "port_types"})
    if extra:
        raise GraphError("BAD_PARAM", "%s.%s" % (where, extra[0]), "unknown design field %s (a design has blocks, "
                         "lines and, from fxslx, port_types)" % extra[0])
    for k, ln in enumerate(spec.get("lines") or []):
        if isinstance(ln, dict):
            bad = sorted(set(ln) - {"src", "dst", "sport", "dport"})
            if bad:
                raise GraphError("BAD_PARAM", "%s.lines[%d].%s" % (where, k, bad[0]), "unknown line field %s (a line "
                                 "has src, dst and optional sport, dport)" % bad[0])
    for name, b in spec["blocks"].items():
        if isinstance(b, dict) and b.get("type") == "Delay" and \
                str(b.get("DelayLengthSource", "Dialog")).strip() == "Input port" and \
                str(b.get("DiagnosticForDelayLength", "None")).strip() != "None":
            # the native minimal counterexample: with
            # DiagnosticForDelayLength 'Error' Simulink stops when the port length leaves 0 .. limit, the engine
            # clamped and answered; 'Error' / 'Warning' are not recorded here -- refused
            raise GraphError("UNSUPPORTED", "%s.blocks.%s.DiagnosticForDelayLength" % (where, name), "Delay %s with "
                             "the length from a port and DiagnosticForDelayLength %r: Simulink stops (Error) or "
                             "warns (Warning) when the port length leaves 0 .. DelayLengthUpperLimit; not recorded "
                             "here -- leave it 'None'" % (name, b.get("DiagnosticForDelayLength")))
        if not isinstance(b, dict):
            # "B": 3 ended in INTERNAL (AttributeError)
            raise GraphError("BAD_PARAM", "%s.blocks.%s" % (where, name), "a block is an object with a type and its "
                             "parameters, got %s" % type(b).__name__)
        if not isinstance(name, str) or any(ch in name for ch in RESERVED_CHARS):
            raise GraphError("BAD_PARAM", where + ".blocks", "block name %r contains a reserved control character "
                             "(U+001D..U+001F)" % (name,))
        if isinstance(b, dict):
            if b.get("type") in INTERNAL_TYPES:
                raise GraphError("UNKNOWN_BLOCK_TYPE", "%s.blocks.%s.type" % (where, name), "%s is an internal block "
                                 "of the multirate lowering, not a model block" % b["type"])
            for k in b:
                if k in INTERNAL_PARAMS:
                    raise GraphError("BAD_PARAM", "%s.blocks.%s.%s" % (where, name, k), "internal parameter; "
                                     "only the engine's own passes set it")
    for k, ln in enumerate(spec.get("lines") or []):
        if isinstance(ln, dict):
            for key in ("src", "dst"):
                v = ln.get(key)
                if isinstance(v, str) and any(ch in v for ch in RESERVED_CHARS):
                    raise GraphError("BAD_PARAM", "%s.lines[%d].%s" % (where, k, key), "reserved control character")
    # enumerated options in their canonical spelling for every user-facing tool (fxgraph, fxopt, fxcodegen, fxcert):
    # the lowerings read the raw block (it ran at the fxgraph entry only)
    canonicalize_design(spec)
    import option_validation   # recorded popup sweep: popup options with surrounding spaces (not recorded)
    option_validation.check(spec)


def _lut_slopes(rows, tt, it):
    """Table steps at the table's own precision (Simulink forms y1 - y0 from the stored table values,
    not from their intermediate-type copies). rows: list of table rows (Fractions). Returns (per-row lists of
    integer steps at fraction F = max(table, intermediate), dsh = F - intermediate fraction)."""
    tf = tt[2] if isinstance(tt, tuple) else it[2]
    F = max(tf, it[2])
    out = [[int((r[k + 1] - r[k]) * (Fraction(2) ** F)) for k in range(len(r) - 1)] + [0] for r in rows]
    return out, F - it[2]


def _lut_table_ints(tq, tt, it):
    """2-D table values at fraction F = max(table, intermediate) as integers, row-major; and dsh."""
    tf = tt[2] if isinstance(tt, tuple) else it[2]
    F = max(tf, it[2])
    return [int(v * (Fraction(2) ** F)) for r in tq for v in r], F - it[2]


def _prodn_bits(tb, ops, ot):
    """Largest intermediate (in bits, sign included) of the C core's PRODN instruction for input types tb, ops
    ("*" / "/") and output type ot -- every product, scaled numerator / denominator and rounding divisor 2^sh,
    following engine/fxgraph_core.c step by step, including the accumulated product of several inputs
    and the scaling of a quotient."""
    fo, ow = ot[2], ot[1] + 1
    need = [ow + 1]

    def sdiv(num_w, e, den_w):   # scaled_div(num, e, den): num * 2^e (e >= 0) or den * 2^-e
        need.append(num_w + max(e, 0) + 1)
        need.append(den_w + max(-e, 0) + 1)
    W = [t[1] + 1 for t in tb]   # an unsigned word needs one more bit as a signed integer
    F = [t[2] for t in tb]
    if ops == "/*":
        sdiv(W[1], F[0] + fo - F[1], W[0])
        return max(need)
    if ops[0] == "*":
        acc_w, afl = W[0], F[0]
        if len(ops) == 1:
            sh = afl - fo
            need.append(sh + 2 if sh > 0 else acc_w - sh + 1)
            return max(need)
    else:
        sdiv(2, F[0] + fo, W[0])
        acc_w, afl = ow, fo
    for j in range(1, len(ops)):
        if ops[j] == "*":
            v = acc_w + W[j]
            sh = afl + F[j] - fo
            need.append(v + 1)
            need.append(sh + 2 if sh > 0 else v - sh + 1)
        else:
            sdiv(acc_w, F[j] + fo - afl, W[j])
        acc_w, afl = ow, fo
    return max(need)


def _hid(n, tag):
    """Name of a hidden C-core slot of block n (never a user-visible block name)."""
    return n + "\x1f" + tag


class _GatedKernel:
    """Python path of a stateful block in a slow domain: evaluated only on its hits (k = P j + o), output held."""

    def __init__(self, k, P, o):
        self.k, self.P, self.o = k, P, o
        self.t, self.y = 0, 0

    def reset(self):
        self.t, self.y = 0, 0
        if hasattr(self.k, "reset"):
            self.k.reset()

    def __call__(self, v):
        t = self.t
        self.t += 1
        if t >= self.o and (t - self.o) % self.P == 0:
            self.y = self.k(v)
        return self.y


class _HoldKernel:
    """Python path of the internal Hold block: enable ? x : previous output (starts at the initial value)."""

    def __init__(self, ic):
        self.ic = self.y = ic

    def reset(self):
        self.y = self.ic

    def __call__(self, v):
        if v[1]:
            self.y = v[0]
        return self.y


class _FxKernel:
    """Per-step evaluation through ref/fxcore.py (exact reference rules); stateful blocks (Discrete FIR)
    keep their history inside fxcore, keyed by this design and block name."""

    def __init__(self, d, n, route, ts, ot):
        self.d, self.n, self.route, self.ts, self.ot = d, n, route, list(ts), ot
        self.key = (route[1] if route[0] == "op" else "bool:" + route[1], "%x" % id(d))
        self.t = 0

    def reset(self):
        self.t = 0

    def __call__(self, v):
        case = self.d._fx_case(self.n, [_rec(t, x) for t, x in zip(self.ts, v)], self.t, self.key)
        self.t += 1
        out = self.d._fx_call(self.route, case)
        return _unrec(out)

    def peek(self):
        """This step's output of a block without direct feedthrough (a Forward Euler integrator: the state), before
        its input of this step is known -- evaluated on a placeholder input that is then dropped from the history."""
        case = self.d._fx_case(self.n, [_rec(t, 0) for t in self.ts], self.t, self.key)
        out = self.d._fx_call(self.route, case)
        h = fc._HIST.get((self.key[0], self.key[1], self.n))
        if h is None or len(h) != self.t + 1:
            raise GraphError("INTERNAL", "design.blocks." + self.n, "integrator state read out of step")
        h.pop()
        return _unrec(out)


class _TransposedDouble:
    """Ideal transposed FIR (same operation order as the C core's DREF program): y = p0 + z1,
    z_k <- p_k + z_(k+1), z_(N-1) <- p_(N-1); the initial states are the partial sums."""
    def __init__(self, coefs, init):
        self.c, self.z = list(coefs), [0.0] + list(init)

    def step(self, x):
        c, z, N = self.c, self.z, len(self.c)
        p = [ck * x for ck in c]
        y = 0.0 + p[0] + (z[1] if N > 1 else 0.0)
        self.z = [0.0] + [(0.0 + p[k] + z[k + 1]) if k < N - 1 else (0.0 + p[k]) for k in range(1, N)]
        return y


def _dfir_sym_check(p, xt, where, words):
    """Symmetric structures: type combinations Simulink rejects (recordings with the
    same identifier; all 355 recorded modules accepted). The tap sum must be signed when the input or the
    coefficients are; with an unsigned input and unsigned coefficients, a product by internal rule of negative
    coefficients is refused first (all types must be unsigned), then an antisymmetric tap sum by internal rule
    (a difference is signed) or any signed tap-sum / product / accumulator / state / output type."""
    coefs = fc._dfir_coefs(p)
    if len(coefs) == 1:
        # recordings (the recorded sym2 / recordings): Simulink refuses a symmetric or
        # antisymmetric structure with one coefficient (Simulink:Parameters:InvParamSetting --
        # INDEX; recordings: the id was cited as Simulink:blocks:...)
        raise GraphError("PORT_TYPE_INVALID", where + ".FilterStructure", "%s with one coefficient: Simulink "
                         "rejects it (Simulink:Parameters:InvParamSetting); use Direct form or give at least two "
                         "coefficients" % p["FilterStructure"])
    ct_, pt, at, st, ot, tt = fc._dfir2_types(p, xt, coefs, words)
    if not xt[0] and not ct_[0] and p["ProductDataTypeStr"] == fc._INHERIT_RULE:
        # unsigned input and coefficients with the product by
        # internal rule -- Simulink refuses it with an inherited tap sum (DiscreteFirAllDataTypeMustBeUnsigned)
        # although the engine's types were all unsigned; with an explicit tap sum not recorded
        if str(p.get("TapSumDataTypeStr", "")).startswith("Inherit"):
            raise GraphError("PORT_TYPE_INVALID", where, "all data types must be unsigned: unsigned input and "
                             "coefficients with the product by internal rule and an inherited tap sum "
                             "(Simulink:blocks:DiscreteFirAllDataTypeMustBeUnsigned)")
        raise GraphError("UNSUPPORTED", where + ".ProductDataTypeStr", "symmetric structure with unsigned input and "
                         "coefficients and the product by internal rule (not recorded with an explicit tap sum): give "
                         "the product an explicit type")
    if (xt[0] or ct_[0]) and not tt[0]:
        raise GraphError("PORT_TYPE_INVALID", where + ".TapSumDataTypeStr", "tap-sum type must be signed "
                         "(Simulink:blocks:DiscreteFirTapSumDataTypeMustBeSigned)")
    if not xt[0] and not ct_[0]:
        if p["ProductDataTypeStr"] == fc._INHERIT_RULE and any(c < 0 for c in coefs):
            raise GraphError("PORT_TYPE_INVALID", where, "all data types must be unsigned: negative coefficients "
                             "with an unsigned input (Simulink:blocks:DiscreteFirAllDataTypeMustBeUnsigned)")
        if tt[0] or (p["FilterStructure"] == "Direct form antisymmetric"
                     and p["TapSumDataTypeStr"] == fc._INHERIT_RULE):
            raise GraphError("PORT_TYPE_INVALID", where + ".TapSumDataTypeStr", "tap-sum type must be unsigned "
                             "(Simulink:blocks:DiscreteFirTapSumDataTypeMustBeUnsigned)")
        if any(t[0] for t in (pt, at, st, ot)):
            raise GraphError("PORT_TYPE_INVALID", where, "all data types must be unsigned "
                             "(Simulink:blocks:DiscreteFirAllDataTypeMustBeUnsigned)")


def _dfir_sign_check(p, xt, where, words):
    """Every filter structure (direct form: recordingsrecordings
    .dropped: 84 / 84 with the same identifier, all 576 recorded direct-form modules accepted; transposed, symmetric
    and antisymmetric: recordings, 9 / 9 rejections, all 355
    recorded modules of those forms fit): with S = input or coefficient type signed, the
    product, accumulator and output types must all have signedness S (an input and coefficients of different
    signedness are fine, and the state type is not checked). Of the forms tried this is the only one with no false
    refusal."""
    coefs = fc._dfir_coefs(p)
    ct_, pt, at, st, ot = fc._dfir_types(p, xt, coefs, words)
    S = bool(xt[0] or ct_[0])
    if any(bool(t[0]) != S for t in (pt, at, ot)):
        raise GraphError("PORT_TYPE_INVALID", where, "all data types must be %s (product %s, accumulator %s, output "
                         "%s; Simulink:blocks:DiscreteFirAllDataTypeMustBe%s)"
                         % ("signed" if S else "unsigned", _tstr(pt), _tstr(at), _tstr(ot),
                            "Signed" if S else "Unsigned"))


def _fir_init_double(p, ntaps):
    """Unquantised initial states of a Discrete FIR (ideal reference), most recent first."""
    vals = fc._vec(p.get("InitialStates", "0"))
    return vals * max(ntaps - 1, 0) if len(vals) == 1 else vals


def _table_conv(vals, t, rm):
    """(min, max) of table values rounded into type t (rounding rm) before any wrap or saturation."""
    f = t[2]
    qs = []
    for v in vals:
        x = Fraction(v) * (Fraction(2) ** f)
        qs.append(fc._round_quot(x.numerator, x.denominator, rm))
    return (min(qs), max(qs)) if qs else (0, 0)


def _counter_limit(d, n):
    """The count's wrap limit, from the parameter record (also on designs that are never compiled)."""
    return d.param_record(n).limit


def _dti_ic_double(p):
    """Ideal initial state: the initial condition, clamped to the limits when limiting is on."""
    x0 = float(p["InitialCondition"])
    if p.get("LimitOutput") == "on":
        x0 = max(float(p["LowerSaturationLimit"]), min(float(p["UpperSaturationLimit"]), x0))
    return x0


def _lut_raw(p):
    """Unquantised breakpoints and table of a 1-D Lookup Table (the double reference)."""
    table = fc._vec(p["Table"])
    if p["BreakpointsSpecification"] == "Even spacing":
        x0, dx = float(p["BreakpointsForDimension1FirstPoint"]), float(p["BreakpointsForDimension1Spacing"])
        return [x0 + k * dx for k in range(len(table))], table
    return fc._vec(p["BreakpointsForDimension1"]), table


def _lut2d_raw(p):
    """Unquantised breakpoints (per dimension) and table rows of a 2-D Lookup Table (the double reference)."""
    table = [[float(v) for v in r] for r in fc._mat(p["Table"])]
    bps = []
    for dim, cnt in ((1, len(table)), (2, len(table[0]))):
        if p["BreakpointsSpecification"] == "Even spacing":
            x0 = float(p["BreakpointsForDimension%dFirstPoint" % dim])
            dx = float(p["BreakpointsForDimension%dSpacing" % dim])
            bps.append([x0 + k * dx for k in range(cnt)])
        else:
            bps.append([float(v) for v in fc._vec(p["BreakpointsForDimension%d" % dim])])
    return bps, table


def _dlut2(bps, table, mode, u1, u2):
    """Same operations as the C core's double LUT2 instruction (bilinear, dimension 1 first)."""
    k, f = [], []
    for bp, u in zip(bps, (u1, u2)):
        last = len(bp) - 1
        u = min(max(u, bp[0]), bp[last])
        kk = 0
        while kk < last - 1 and u >= bp[kk + 1]:
            kk += 1
        bk, bk1 = bp[kk], bp[kk + 1]
        if mode < 2:
            k.append(kk + (1 if (u >= bk1 if mode == 0 else (u - bk) >= (bk1 - u)) else 0))
            f.append(0.0)
        else:
            k.append(kk)
            f.append((u - bk) / (bk1 - bk))
    if mode < 2:
        return table[k[0]][k[1]]
    t00, t10 = table[k[0]][k[1]], table[k[0] + 1][k[1]]
    t01, t11 = table[k[0]][k[1] + 1], table[k[0] + 1][k[1] + 1]
    a0, a1 = t00 + f[0] * (t10 - t00), t01 + f[0] * (t11 - t01)
    return a0 + f[1] * (a1 - a0)


def _dlut(bps, table, mode, u):
    """Same operations as the C core's double LUT instruction. mode 0 Flat, 1 Nearest, 2 linear point-slope."""
    last = len(bps) - 1
    if u < bps[0]:
        u = bps[0]
    if u > bps[last]:
        u = bps[last]
    k = 0
    while k < last - 1 and u >= bps[k + 1]:
        k += 1
    if mode == 2:
        return table[k] + ((u - bps[k]) / (bps[k + 1] - bps[k])) * (table[k + 1] - table[k])
    up = u >= bps[k + 1] if mode == 0 else (u - bps[k]) >= (bps[k + 1] - u)
    return table[k + 1] if up else table[k]


def _plan(cls, **kw):
    """A compiled lookup record; a broken invariant is a compiler defect, reported as a structured INTERNAL error
    (never an uncaught exception)."""
    try:
        return cls(**kw)
    except LP.PlanError as e:
        raise GraphError("INTERNAL", "design.blocks.%s" % kw.get("block"), str(e))


def _lut_types_recorded(p, where, dim):
    """UNSUPPORTED for intermediate / fraction type forms outside the recorded data (ref/fxcore.py
    lut_linear_types_recorded)."""
    try:
        fc.lut_linear_types_recorded(p)
    except ValueError as e:
        raise GraphError("UNSUPPORTED", where + ".FractionDataTypeStr" if "fraction" in str(e) else
                         where + ".IntermediateResultsDataTypeStr", "%s Lookup Table: %s" % (dim, e))


def _lut_declared(p, key):
    """A lookup's type parameter as declared (fixdt / named integer), None for inherited / rule types: the cost
    counter counts declared intermediate and fraction words only."""
    return _fixdt(str(p.get(key, "")), "")


def _lut_it_source(p):
    txt = str(p.get("IntermediateResultsDataTypeStr", ""))
    if _lut_declared(p, "IntermediateResultsDataTypeStr") is not None:
        return "explicit"
    if txt == fc._INHERIT_RULE:
        return "internal rule"
    if "Same as output" in txt:
        return "same as output"
    raise GraphError("INTERNAL", "IntermediateResultsDataTypeStr", "no compile source for %r" % txt)


def _lut_frac_source(p):
    return "inherited rule" if str(p.get("FractionDataTypeStr", fc._INHERIT_RULE)).startswith("Inherit") else "explicit"


def _lut_mode(p):
    return {"Flat": 0, "Nearest": 1, "Linear point-slope": 2}[p["InterpMethod"]]


def _dti_ints(p, it, where="design.blocks", mode="error", notes=None):
    """Discrete-Time Integrator (internal rule): initial state and clamp limits as stored integers of the
    state type (same quantisation as ref/fxcore.py _blk_dti). Returns (x0, (lo, hi) or None)."""
    s, w, frec, slope = fc._dti_type(p, it)
    if not slope:
        # gain value 0 (K x T = 0) gave Fraction(1, 0) here (INTERNAL)
        raise GraphError("UNSUPPORTED", where + ".gainval", "Discrete-Time Integrator with gain value %r (K x T = 0) "
                         "is not supported; the internal-rule output slope is the input slope times K x T"
                         % p.get("gainval"))
    lo, hi = (-(1 << (w - 1)), (1 << (w - 1)) - 1) if s else (0, (1 << w) - 1)

    def q(x, rm, om, key):
        v = Fraction(x) / slope
        r = fc._round_quot(v.numerator, v.denominator, rm)
        if mode == "error" and (v <= lo - 1 or v > hi + 1):   # unrounded (0.6 LSB below the minimum: ok)
            # an initial condition or limit outside the state type stops Simulink (fxpParameterOverflow)
            raise GraphError("PORT_TYPE_INVALID", where + "." + key, "Discrete-Time Integrator %s: %s %s does not fit "
                             "the state type %s, range [%s, %s] (Simulink stops with fxpParameterOverflow under its "
                             "default diagnostics); write a value within the range, widen the integrator's input "
                             "type, or set hw.ParameterOverflowMsg 'warning' / 'none' to saturate it"
                             % (where.split(".")[-1], key, _fmt_num(x), "fixdt(%d,%d,%s)" % (s, w, frec),
                                _fmt_num(lo * slope), _fmt_num(hi * slope)))
        r = fc._overflow(r, s, w, om)
        if notes is not None and Fraction(x) != r * slope:
            # the initial condition and limits are quantised to the state type -- reported like every
            # other parameter the engine does not use as written
            notes[where + "." + key] = {"written": _fmt_num(x), "type": "fixdt(%d,%d,%s)" % (s, w, frec),
                                        "used": _fmt_num(r * slope),
                                        "why": "saturated to the type's bound" if r in (lo, hi) and
                                        not (lo * slope <= Fraction(x) <= hi * slope) else
                                        "rounded to the type's precision"}
        return r
    lims = None
    if p.get("LimitOutput") == "on":
        ul, ll = p["UpperSaturationLimit"], p["LowerSaturationLimit"]
        if mode == "error" and any(x_.strip().lstrip("+-") in ("inf", "Inf") for x_ in (ul, ll)):
            # with LimitOutput on, an infinite limit is an overflowing parameter for a fixed-point
            # state (46 of 46 recorded units: fxpParameterOverflow); the engine took the type's bound
            k_ = "UpperSaturationLimit" if ul.strip().lstrip("+-") in ("inf", "Inf") else "LowerSaturationLimit"
            raise GraphError("PORT_TYPE_INVALID", where + "." + k_, "Discrete-Time Integrator %s: an infinite %s with a "
                             "fixed-point state (Simulink stops with fxpParameterOverflow under its default "
                             "diagnostics); write the limits as numbers inside the state type or turn LimitOutput off"
                             % (where.split(".")[-1], k_))
        up = hi if ul.strip() in ("inf", "Inf") else q(float(ul), "Nearest", "Saturate", "UpperSaturationLimit")
        dn = lo if ll.strip() in ("-inf", "-Inf") else q(float(ll), "Nearest", "Saturate", "LowerSaturationLimit")
        lims = (dn, up)
    x0 = q(float(p["InitialCondition"]), fc.DTI_IC_RM, "Saturate", "InitialCondition")
    if lims:
        x0 = max(lims[0], min(lims[1], x0))
    return x0, lims


_GAIN_SAME_DOMAIN = None


def _gain_same_domain():
    global _GAIN_SAME_DOMAIN
    if _GAIN_SAME_DOMAIN is None:
        p_ = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gain_same_domain.json")
        _GAIN_SAME_DOMAIN = set(json.load(open(p_))) if os.path.exists(p_) else set()
    return _GAIN_SAME_DOMAIN


class Design:
    """Compiled design. Build with Design(spec, hw); then run(inputs)."""

    def __init__(self, spec, hw=None):
        self.acc_types, self.fir_types = {}, {}   # compiled internal types (filled by the kernels; cost counter)
        if not isinstance(spec, dict):
            raise GraphError("MISSING_FIELD", "design", "design object is required")
        # a the extended blocks's left-out parameters are the library block's defaults (engine/fxhdl.py)
        import fxhdl as _fxh
        spec = _fxh.with_library_defaults(spec)
        # recorded numeric parameters outside their recorded range are refused first (engine/parameter_bounds.py)
        import parameter_bounds
        spec = parameter_bounds.normalize_ints(spec)   # int8(4), '2+3', '3.0' ... in the bit blocks' indices
        parameter_bounds.check(spec)
        if not hasattr(self, "_lab2_user") or self._lab2_user is None:
            self._lab2_user = parameter_bounds.user_blocks(spec)   # the user's the extended blocks, before any lowering
            self._lab2_params = parameter_bounds.user_params(spec)   # their parameters
        # HDL library blocks: lowered into engine blocks every path supports (engine/fxhdl.py)
        import fxhdl
        self.hdl_info = None
        self._ram_auto = None
        self._lab2_ports = dict(getattr(self, "_lab2_ports", None) or {})
        self._lab2_ports.update(spec.get(parameter_bounds.PORT_KEY) or {})
        self._lab2_pnotes = dict(spec.get(parameter_bounds.NOTES_KEY) or {})   # recordings (engine/block_lowering.py)
        if fxhdl.has_hdl(spec):
            self.hw = hw
            # System RAMs whose address width is the address signal's (AddressBits 'Inherit: from address'): a
            # provisional width until compile() knows the address type (engine/fxhdl.py recordings)
            self._ram_auto, spec = fxhdl.ram_auto_provisional(spec)
            keep_ = {k: spec[k] for k in ("same_types",) if k in spec}   # (not port_types: lowering renames ports)
            spec, self.hdl_info = fxhdl.lower(spec)
            # the recorded lowering returned the design without the vector pass's same_types (a Mux of
            # different types was answered whenever a the extended blocks, e.g. a Zero-Order Hold, was in the design)
            if "same_types" in keep_ and "same_types" not in spec and isinstance(keep_["same_types"], list):
                # only groups whose blocks the lowering kept (recordings named
                # removed blocks -> BAD_PARAM)
                bl_ = spec.get("blocks") or {}
                g_ = [x for x in keep_["same_types"] if isinstance(x, dict) and
                      all(isinstance(y, str) and y in bl_ for y in x.get("blocks") or [])]
                if g_:
                    spec = dict(spec, same_types=g_)
            # where the lowerings moved output ports (parameter_bounds.PORT_KEY)
            self._lab2_ports.update(spec.get(parameter_bounds.PORT_KEY) or {})
            # the lowerings' notes on the coefficients they used other than as written (engine/fxiir.py)
            self._lab2_pnotes.update(spec.get(parameter_bounds.NOTES_KEY) or {})
        blocks = spec.get("blocks")
        lines = spec.get("lines", [])
        if not isinstance(blocks, dict) or not blocks:
            raise GraphError("MISSING_FIELD", "design.blocks", "non-empty blocks object is required")
        if len(blocks) > MAX_BLOCKS:
            raise GraphError("LIMIT_EXCEEDED", "design.blocks", "at most %d blocks" % MAX_BLOCKS)
        if not isinstance(lines, list):
            raise GraphError("MISSING_FIELD", "design.lines", "lines list is required")
        _check_hw(hw)
        def _delay_own_block(b_):
            # a Delay whose length / initial condition comes from a port, or with an enable port (recordings
            # recordings): its own block
            return isinstance(b_, dict) and b_.get("type") == "Delay" and (
                str(b_.get("DelayLengthSource", "Dialog")) == "Input port" or
                str(b_.get("InitialConditionSource", "Dialog")) == "Input port" or
                str(b_.get("ShowEnablePort", "off")).strip() == "on" or
                str(b_.get("ExternalReset", "None")).strip() not in ("None", "none"))
        if any(_delay_own_block(b_) for b_ in blocks.values()):
            # a Delay whose length or initial condition comes from port 2 is not
            # a fixed delay (length 0 feeds through); evaluated as its own block on the Python path
            blocks = {n_: (dict(b_, type="DelayPort") if _delay_own_block(b_) else b_) for n_, b_ in blocks.items()}
        blocks, lines = _lower_dti_statefb(blocks, lines)
        blocks, lines = _lower_prelookup(blocks, lines)
        DRF = {"DetectRisePositive": ">", "DetectRiseNonnegative": ">=", "DetectFallNegative": "<",
               "DetectFallNonpositive": "<="}
        if any(isinstance(b_, dict) and b_.get("type") in DRF for b_ in blocks.values()):
            # recordings (40 designs, 37 run: every step recomputed): Detect Rise / Fall keeps
            # the previous comparison result u REL 0 in a delay of the output type whose initial value is vinit, and
            # outputs (u REL 0) AND NOT previous. Lowered here to Compare To Zero -> Unit Delay -> NOT -> AND; a vinit
            # outside the output type (uint8, -1) overflows the delay's initial condition as in Simulink
            blocks, lines = dict(blocks), list(lines)
            for n_, b_ in list(blocks.items()):
                if not (isinstance(b_, dict) and b_.get("type") in DRF):
                    continue
                ip_ = str(b_.get("InputProcessing", "Elements as channels (sample based)")).strip()
                if ip_ not in ("Elements as channels (sample based)", "Inherited"):
                    raise GraphError("UNSUPPORTED", "design.blocks.%s.InputProcessing" % n_, "%s InputProcessing %r is "
                                     "not supported (sample based only)" % (b_["type"], ip_))
                extra_ = sorted(set(b_) - {"type", "vinit", "OutDataTypeStr", "SampleTime", "InputProcessing"})
                if extra_:
                    raise GraphError("BAD_PARAM", "design.blocks.%s.%s" % (n_, extra_[0]), "unknown parameter %s for %s"
                                     % (extra_[0], b_["type"]))
                ot_ = str(b_.get("OutDataTypeStr", "boolean")).strip()
                if ot_ not in ("boolean", "uint8"):
                    raise GraphError("UNSUPPORTED", "design.blocks.%s.OutDataTypeStr" % n_, "%s output type %s: only "
                                     "boolean and uint8 have been recorded" % (b_["type"], ot_))
                if ot_ == "uint8":
                    # vinit is held in the uint8 output; an overflow
                    # (-1, -128, 300) stops Simulink -- reported on this block, not on the lowering's delay
                    try:
                        v_ = fc._const_value(str(b_.get("vinit", "0")))
                    except (ValueError, KeyError, TypeError):
                        v_ = None
                    if v_ is not None:
                        _param_overflow(v_, (0, 8, 0), "design.blocks.%s.vinit" % n_, "%s %s: vinit in the uint8 "
                                        "output" % (b_["type"], n_), mode=str((hw or {}).get("ParameterOverflowMsg", PARAM_OVERFLOW_DEFAULT)).strip())
                c_, d_, q_ = n_ + "\x1dcmp", n_ + "\x1dprev", n_ + "\x1dnot"
                blocks[c_] = {"type": "CompareToZero", "relop": DRF[b_["type"]], "OutDataTypeStr": ot_}
                blocks[d_] = {"type": "UnitDelay", "InitialCondition": str(b_.get("vinit", "0"))}
                blocks[q_] = {"type": "LogicalOperator", "Operator": "NOT",
                              **({"OutDataTypeStr": "fixdt(0,8,0)"} if ot_ == "uint8" else {})}
                blocks[n_] = {"type": "LogicalOperator", "Operator": "AND", "Inputs": "2",
                              **({"OutDataTypeStr": "fixdt(0,8,0)"} if ot_ == "uint8" else {})}
                lines = [dict(ln, dst=c_) if isinstance(ln, dict) and ln.get("dst") == n_ else ln for ln in lines]
                lines += [{"src": c_, "dst": d_}, {"src": d_, "dst": q_}, {"src": c_, "dst": n_, "dport": 1},
                          {"src": q_, "dst": n_, "dport": 2}]
            spec = dict(spec, blocks=blocks, lines=lines)

        def _neg_shift(b_):
            # recordings (host) and recordings (32 designs, host and ASIC,
            # '-2', '-(3)', '-1*2', 'int8(-2)', '-2;', '[-1]'): a negative BitShiftNumber with direction Left / Right
            # leaves the input unchanged (same type, same values)
            if not (isinstance(b_, dict) and b_.get("type") == "ShiftArithmetic" and
                    str(b_.get("BitShiftDirection", "Bidirectional")).strip() in ("Left", "Right")):   # (left out: the library's Bidirectional)
                return False
            t_ = str(b_.get("BitShiftNumber", "0")).strip()
            while t_.endswith(";"):
                t_ = t_[:-1].strip()
            if t_.startswith("[") and t_.endswith("]") and t_.count("[") == 1 and " " not in t_.strip("[] ") \
                    and "," not in t_:
                t_ = t_[1:-1].strip()
            r_ = fc.matlab_int_expr(t_)
            return r_ is not None and r_[0].denominator == 1 and r_[0] < 0
        if any(_neg_shift(b_) for b_ in blocks.values()):
            blocks = {n_: (dict(b_, BitShiftNumber="0") if _neg_shift(b_) else b_) for n_, b_ in blocks.items()}
            spec = dict(spec, blocks=blocks)
        if any(isinstance(b_, dict) and b_.get("type") == "RelationalOperator" and
               str(b_.get("Operator", "")).strip() in ("isNaN", "isFinite") for b_ in blocks.values()):
            # recordings (12 designs host and ASIC, fixed-point and boolean
            # inputs): isNaN is always false and isFinite always true, boolean output, no warning -- the block becomes
            # a boolean Constant (its input is not read). isInf was not drawn in the recording: still refused
            blocks = dict(blocks)
            for n_, b_ in list(blocks.items()):
                if isinstance(b_, dict) and b_.get("type") == "RelationalOperator" and \
                        str(b_.get("Operator", "")).strip() in ("isNaN", "isFinite"):
                    # the unary block has one input port -- a line into port 2 cannot be built
                    for ln in lines:
                        if isinstance(ln, dict) and ln.get("dst") == n_ and str(ln.get("dport", 1)).strip() != "1":
                            raise GraphError("BAD_CONNECTION", "design.blocks." + n_, "%s (Operator %s) has one input "
                                             "port; a line goes into port %s" % (n_, b_["Operator"], ln.get("dport")))
                    # the Constant it becomes would drop the block's own sample time
                    if str(b_.get("SampleTime", "-1")).strip() not in ("-1", ""):
                        raise GraphError("UNSUPPORTED", "design.blocks.%s.SampleTime" % n_, "Relational Operator %s "
                                         "with SampleTime %s is not supported (an inherited sample time is)"
                                         % (b_["Operator"], b_["SampleTime"]))
                if isinstance(b_, dict) and b_.get("type") == "RelationalOperator" and \
                        str(b_.get("Operator", "")).strip() in ("isNaN", "isFinite") and \
                        str(b_.get("OutDataTypeStr", "boolean")).strip() == "boolean":
                    blocks[n_] = {"type": "Constant", "OutDataTypeStr": "boolean",
                                  "Value": "1" if str(b_["Operator"]).strip() == "isFinite" else "0"}
                    lines = [ln for ln in lines if not (isinstance(ln, dict) and ln.get("dst") == n_)]
            spec = dict(spec, blocks=blocks, lines=lines)
        # "port_types": [{"block": name, "type": "fixdt(..)" | "boolean" | intN, "where": text}] -- data types that
        # ports of a flattened subsystem declare for the signal a block drives (written by the .slx reader); checked
        # against the compiled type, as an Outport's own OutDataTypeStr is
        self.port_types = []
        for k, e in enumerate(spec.get("port_types") or []):
            if not (isinstance(e, dict) and isinstance(e.get("block"), str) and e["block"] in blocks and
                    isinstance(e.get("type"), str)):
                raise GraphError("BAD_PARAM", "design.port_types[%d]" % k, "{\"block\": name, \"type\": type string} "
                                 "with a block of the design required")
            self.port_types.append((e["block"], e["type"], str(e.get("where", "design.port_types[%d]" % k))))
        # "same_types": [{"blocks": [names], "where": text}] -- signals that must compile to one type (the elements of
        # a Mux, written by the vector lowering, engine/fxvector.py)
        self.same_types = []
        for k, e in enumerate(spec.get("same_types") or []):
            if not (isinstance(e, dict) and isinstance(e.get("blocks"), list) and
                    all(isinstance(x, str) and x in blocks for x in e["blocks"])):
                raise GraphError("BAD_PARAM", "design.same_types[%d]" % k, "{\"blocks\": [names of the design]} required")
            self.same_types.append((list(e["blocks"]), str(e.get("where", "design.same_types[%d]" % k))))
            if e.get("bus_outport"):
                self.bus_outport_groups = getattr(self, "bus_outport_groups", set()) | {len(self.same_types) - 1}
        self.words = fc._hw_words({"hw": hw or {}})
        self.hw = hw
        self.pom = str((hw or {}).get("ParameterOverflowMsg", PARAM_OVERFLOW_DEFAULT)).strip()
        self.slot_types = {}   # types of hidden delay slots (FIR taps, partial sums); filled by compile
        self.names = sorted(blocks)
        self.btype, self.params, self.nin = {}, {}, {}
        self.ignored_params = {}
        self.runtime_diag = []
        self.assumed_defaults = {}
        for name in self.names:
            b = blocks[name]
            where = "design.blocks." + name
            if not isinstance(name, str) or HIDDEN_SEP in name:
                # the C-core's hidden slots are named <block>\x1f<tag>: a user name with \x1f could alias one
                raise GraphError("BAD_PARAM", "design.blocks", "block name %r contains a reserved control "
                                 "character (U+001F)" % (name,))
            if not isinstance(b, dict) or "type" not in b:
                raise GraphError("MISSING_FIELD", where + ".type", "block type is required")
            bt = b["type"]
            if bt in PENDING_TYPES:
                raise GraphError("UNSUPPORTED", where + ".type", PENDING_TYPES[bt])
            if bt not in DEFAULTS:
                # the next step promised "the message says what is accepted"
                import difflib
                known = public_block_types()
                close = difflib.get_close_matches(str(bt), known, n=3, cutoff=0.5)
                raise GraphError("UNKNOWN_BLOCK_TYPE", where + ".type", "unknown block type %r%s; accepted types: %s "
                                 "(parameters of each: fxgraph request {\"catalogue\": true})"
                                 % (bt, " (did you mean %s?)" % " / ".join(close) if close else "", ", ".join(known)))
            p = dict(DEFAULTS[bt])
            if not any(ch in name for ch in RESERVED_CHARS):
                # a left-out parameter of the user's block is the library's default; a block the engine's
                # own lowerings create (an internal name) means the engine's default (recorded: the Repeat lowering's
                # Upsample, the Bit Concat lowering's blocks)
                p.update(JSON_LIBRARY_DEFAULTS.get(bt, {}))
            user = {k: _param_text(v, "%s.%s" % (where, k)) if not k.startswith("_") else v
                    for k, v in b.items() if k != "type"}
            if not any(ch in name for ch in RESERVED_CHARS) and bt not in INTERNAL_TYPES:
                # (a block a lowering created carries the engine's own text -- recorded NCO core keeps its settings as
                # JSON, double quotes and all; the user's block was read by these rules in canonicalize_design)
                user = {k: v if (ENUM_OPTIONS.get((bt, k)) or ENUM_OPTIONS.get((None, k)) or k in ON_OFF_FLAGS)
                        else param_text_rules(bt, k, v, where) for k, v in user.items()}   
            if bt == "MultiportSwitch" and "zeroidx" in user and "DataPortOrder" not in user:
                # (the legacy zeroidx, as in canonicalize_design -- recordings)
                z_ = str(user.pop("zeroidx")).strip().lower()
                if z_ not in ("on", "off"):
                    raise GraphError("BAD_PARAM", where + ".zeroidx", "zeroidx must be 'on' or 'off'")
                user["DataPortOrder"] = "Zero-based contiguous" if z_ == "on" else "One-based contiguous"
            user = resolve_enums(bt, user, where)
            if bt in ("Inport", "Outport") and str(user.get("SignalType", "auto")).strip() not in ("auto", "real"):
                # every engine signal is real; an Outport declaring
                # 'complex' on a real signal is a Simulink error (InputPortComplexSignalMismatch), and a complex Inport
                # was simulated as real -- both had been answered with SignalType listed as ignored
                if bt == "Outport":
                    raise GraphError("PORT_TYPE_INVALID", where + ".SignalType", "the port declares SignalType %r but "
                                     "its signal is real (Simulink:DataType:InputPortComplexSignalMismatch); use 'auto' or "
                                     "'real'" % user["SignalType"])
                raise GraphError("UNSUPPORTED", where + ".SignalType", "SignalType %r: the engine simulates real "
                                 "signals only; use 'auto' or 'real'" % user["SignalType"])
            self.ignored_params[name] = sorted(_check_params(bt, user, where))
            # the error reporting contract: the parameters the design left out and the values assumed for them (the Simulink
            # library defaults for a JSON design)
            self.assumed_defaults[name] = {k: p[k] for k in sorted(p) if k not in user and not k.startswith("_")}
            # inherited settings the result relies on but the engine reads with an in-code fallback
            # (recordings dialog values)
            for k, lib in REPORTED_LIBRARY_DEFAULTS.get(bt, {}).items():
                if k not in user and k not in self.assumed_defaults[name]:
                    self.assumed_defaults[name][k] = lib
            for k, lib in JSON_LIBRARY_UNSUPPORTED.get(bt, {}).items():   
                if k not in user:
                    raise GraphError("UNSUPPORTED", "%s.%s" % (where, k), "%s left out takes the library default %r, "
                                     "which is not supported (write %s out)" % (k, lib, k))
            for k in JSON_LIBRARY_UNRECORDED.get(bt, ()) if not any(ch in name for ch in RESERVED_CHARS) else ():
                if k not in user:   # the library value is not recorded
                    raise GraphError("MISSING_FIELD", "%s.%s" % (where, k), "%s left out would take the library "
                                     "block's value, which is not recorded; write %s out" % (k, k))
            if bt == "Constant" and str(user.get("OutDataTypeStr", "")).strip() in ("", "Inherit: Inherit from 'Constant value'"):
                # recorded / recordings: a typed literal 'int8(-3)' / 'uint16(7)' gives the
                # Constant that integer class (MATLAB casts: round half away, saturate); recording requested
                #
                m_ = re.match(r"^\s*(u?int(?:8|16|32))\s*\(.*\)\s*$", str(user.get("Value", "")))
                r_ = fc.matlab_int_expr(str(user["Value"])) if m_ else None
                if r_ is not None and r_[1] == m_.group(1):
                    user["OutDataTypeStr"] = m_.group(1)
                    user["Value"] = str(int(r_[0]))
            if bt == "Constant" and str(user.get("Value", "")).strip() in ("true", "false") and \
                    str(user.get("OutDataTypeStr", "")).strip() in ("", "Inherit: Inherit from 'Constant value'"):
                # In 67 recorded Constant blocks, a MATLAB logical literal makes an inherited-type Constant
                # logical, i.e. boolean (recording requested: recordings)
                user["OutDataTypeStr"] = "boolean"
            if bt == "Constant" and str(user.get("OutDataTypeStr", "")).strip() == "boolean" and \
                    str(user.get("Value", "")).strip() in ("true", "false"):
                # a boolean Constant written true / false is 1 / 0
                user["Value"] = "1" if str(user["Value"]).strip() == "true" else "0"
            for k, kind in NUMERIC_VALUES.get(bt, {}).items():   # one numeric parser for every block
                if k in user and isinstance(user[k], str):
                    if (bt == "LookupND" or (bt == "InterpolationUsingPrelookup" and
                                             str(user[k]).strip().startswith("reshape"))) and k == "Table":
                        # an n-D table is reshape([values], [n1 n2 ...]) (fxcore.lutnd_table parses it)
                        try:
                            fc.lutnd_table(user[k])
                        except ValueError as e:
                            if "written as" in str(e):
                                # reshape(0:7, [2 2 2]) is run by Simulink -- an
                                # expression the engine does not evaluate, not a malformed request
                                raise GraphError("UNSUPPORTED", "%s.%s" % (where, k), "%s: this table spelling is "
                                                 "not evaluated here; write reshape([v1 v2 ...], [n1 n2 ...]) with "
                                                 "numeric literals" % e)
                            raise GraphError("BAD_PARAM", "%s.%s" % (where, k), str(e))
                        continue
                    c = fxnum.canonical(user[k], kind)
                    if isinstance(c, fxnum.NumError):
                        if c.code == "BAD_PARAM" and (bt, k) in SIMULINK_ACCEPTS_NUMERIC:
                            # Simulink runs these (a non-integer counter limit, a vector gain);
                            # not supported here -- not a malformed request
                            raise GraphError("UNSUPPORTED", "%s.%s" % (where, k), "%s (Simulink accepts this value; "
                                             "the engine does not support it)" % c.message)
                        raise GraphError(c.code, "%s.%s" % (where, k), c.message)
                    user[k] = c
            if bt in PORT_COUNTS and PORT_COUNTS[bt][0] in user and isinstance(user[PORT_COUNTS[bt][0]], str):
                k, ops = PORT_COUNTS[bt]
                c = fxnum.port_count(user[k], ops)
                if isinstance(c, fxnum.NumError):
                    raise GraphError(c.code, "%s.%s" % (where, k), c.message)
                user[k] = c
            p.update(user)
            if bt in ("FIRDecimation", "FIRInterpolation"):
                # recordings (21 of 21): a *DataTypeStr written without
                # its *Mode sets the mode in Simulink (a fixdt / integer name -> 'Binary point scaling', 'Inherit: X'
                # -> 'X'); the engine kept the mode's library default and ignored the string -- silent wrong values
                for mk, tk in (("firstCoeffMode", "firstCoeffDataTypeStr"), ("prodOutputMode", "prodOutputDataTypeStr"),
                               ("accumMode", "accumDataTypeStr"), ("outputMode", "outputDataTypeStr")):
                    if tk in user:
                        ts_ = str(user[tk]).strip()
                        if re.match(r"^u?int(8|16|32)$", ts_) or re.match(r"^fixdt\(\s*0\s*,", ts_):
                            # recordings (25 cases): an unsigned fixdt or a
                            # built-in integer name is not a valid FIR Decimation / Interpolation type -- Simulink
                            # stops at set_param (UdtEvalsToUnsupportedFixdtSignMode / UdtInvBuiltin)
                            raise GraphError("PORT_TYPE_INVALID", "%s.%s" % (where, tk), "%s %s: %s %r is not "
                                             "accepted -- these types are signed fixdt only (Simulink: "
                                             "UdtEvalsToUnsupportedFixdtSignMode / UdtInvBuiltin); write fixdt(1,w,f)"
                                             % (bt, name, tk, user[tk]))
                        derived = ts_[len("Inherit: "):] if ts_.startswith("Inherit: ") else \
                            "Specify word length" if re.match(r"^fixdt\(\s*[01]\s*,\s*\d+\s*\)$", ts_) else \
                            "Binary point scaling"
                        if mk in user and str(user[mk]).strip() != derived:
                            # written both ways and disagreeing, Simulink keeps whichever was set last;
                            # a JSON object has no order the engine can rely on -- refused (the engine used the mode)
                            raise GraphError("UNSUPPORTED", "%s.%s" % (where, mk), "%s %s: %s %r and %s %r disagree "
                                             "(%r means %r); Simulink keeps the one set last, which a request cannot "
                                             "say -- write only one, or make them agree" % (
                                                 bt, name, mk, user[mk], tk, user[tk], user[tk], derived))
                        if mk not in user:
                            p[mk] = derived
                            self.assumed_defaults.get(name, {}).pop(mk, None)
            self.btype[name], self.params[name] = bt, p
            self.user_keys = getattr(self, "user_keys", {})
            self.user_keys[name] = set(user)
            self.nin[name] = _ninputs(bt, p, where)
            if bt == "CounterFreeRunning":   # 1 << NumBits was computed before any width check
                try:
                    nb = int(str(p["NumBits"]).strip())
                except ValueError:
                    raise GraphError("BAD_PARAM", where + ".NumBits", "integer number of bits required")
                if not 1 <= nb <= MAX_WL:
                    raise GraphError("UNSUPPORTED_WIDTH", where + ".NumBits", "NumBits %d outside 1..%d" % (nb, MAX_WL))
            if bt == "ShiftArithmetic":   # '1e10' became readable; a shift beyond the type range is not
                for k in ("BinPtShiftNumber", "BitShiftNumber"):
                    try:
                        sh = int(str(p.get(k, "0")).strip())
                    except ValueError:
                        continue   # the block's own check reports it
                    if abs(sh) > 4096:
                        raise GraphError("UNSUPPORTED", "%s.%s" % (where, k), "a shift of %d is outside the "
                                         "supported fraction-length range (|shift| <= 4096)" % sh)
            if bt == "CounterLimited":   # uplimit >= 2^32 crashed the reference (INTERNAL)
                try:
                    up = int(str(p["uplimit"]).strip())
                except ValueError:
                    raise GraphError("BAD_PARAM", where + ".uplimit", "integer upper limit required")
                # up to 2^53 recorded (uint64 from 2^32); beyond that not recorded
                if not 0 <= up <= (1 << 53):
                    raise GraphError("UNSUPPORTED", where + ".uplimit", "Counter Limited: upper limit outside "
                                     "0 .. 2^53 is not supported (not recorded)")
        # every period an Inport writes ("1" counts: Inports at 1 and 2 are two rates -- they were refused as
        # "several sample times ... not supported" while 2 and 4 ran)
        ins_ts = {str(blocks[n].get("SampleTime", "-1")).replace(" ", "") for n in self.names
                  if self.btype[n] == "Inport" and str(blocks[n].get("SampleTime", "-1")).strip() != "-1"}
        self.multirate = any(self.btype[n] in MR_BLOCKS for n in self.names) or \
            any(t.startswith("[") and not t.endswith(",0]") for t in ins_ts) or len(ins_ts) > 1 or \
            _block_rates_differ(self.btype, blocks, self.names)
        if not self.multirate:  # multirate designs: rates resolved by engine/fxmultirate.py
            period = _check_rates(self.btype, {n: {k: v for k, v in blocks[n].items() if k in ("SampleTime", "tsamp")}
                                               for n in self.names})
            # the Discrete-Time Integrator's step T is the sample period; the
            # engine carries it as _DtiT and integrates with the constant K*T. The common period of a
            # single-rate design is its explicit sample time (1 when none is given).
            if period is not None and period != 1:
                # note: a block at SampleTime 2 fed
                # by an Inport that inherits its rate -- Simulink reads the input data at the block's period (5
                # samples of 10), the engine took every value as one sample; the inputs' period must be stated
                loose = [n for n in self.names if self.btype[n] == "Inport" and
                         str(blocks[n].get("SampleTime", "-1")).strip() in ("-1", "")]
                if loose:
                    raise GraphError("UNSUPPORTED", "design.blocks.%s.SampleTime" % loose[0],
                                     "the design runs at period %g (a block's SampleTime) but Inport %s inherits its "
                                     "sample time: Simulink would read its data at that period, not one value per "
                                     "step; set SampleTime %g on every Inport (then each input value is one sample "
                                     "at that period)" % (period, loose[0], period))
                for n in self.names:
                    if self.btype[n] == "DiscreteIntegrator":
                        self.params[n]["_DtiT"] = repr(period)
                    if self.btype[n] == "RateLimiter":
                        # Scale the slew limit by the sample period.
                        if period != 2:
                            raise GraphError("UNSUPPORTED", "design.blocks." + n + ".SampleTime", "Rate Limiter at "
                                             "sample period %g: only periods 1 and 2 are recorded (Simulink takes the "
                                             "slew limits times the period, then into the input type); run the "
                                             "design at period 1 or 2, or ask for a recording at this period"
                                             % period)
                        self.params[n]["_RlT"] = "2"
        elif any(self.btype[n] in ("RateLimiter", "DeadZone", "WrapToZero") for n in self.names) and \
                not any(self.btype[n] == "RateLimiter" for n in self.names):
            raise GraphError("UNSUPPORTED", "design.blocks." + next(n for n in self.names if self.btype[n] in (
                "DeadZone", "WrapToZero")), "Dead Zone / Wrap To Zero in a multirate design are not recorded "
                             "(single-rate designs at periods 1 and 2 are)")
        elif any(self.btype[n] == "RateLimiter" for n in self.names):
            raise GraphError("UNSUPPORTED", "design.blocks." + next(n for n in self.names if self.btype[n] ==
                                                                    "RateLimiter"),
                             "Rate Limiter in a multirate design is not recorded (only sample period 1)")
        # connectivity: every input port driven exactly once; output port 1 only (single-output blocks)
        self.src = {n: [None] * self.nin[n] for n in self.names}
        for j, ln in enumerate(lines):
            where = "design.lines[%d]" % j
            try:
                s, sp, d, dp = ln["src"], int(ln.get("sport", 1)), ln["dst"], int(ln.get("dport", 1))
            except (KeyError, TypeError, ValueError):
                raise GraphError("BAD_CONNECTION", where, "src, dst (and optional sport, dport) required")
            if s not in self.btype or d not in self.btype:
                raise GraphError("BAD_CONNECTION", where, "unknown block %r" % (s if s not in self.btype else d))
            if self.btype[s] == "Outport" or sp != 1:
                raise GraphError("BAD_CONNECTION", where, "%s has no output port %d" % (s, sp))
            if not (1 <= dp <= self.nin[d]):
                raise GraphError("BAD_CONNECTION", where, "%s has no input port %d" % (d, dp))
            if self.src[d][dp - 1] is not None:
                raise GraphError("BAD_CONNECTION", where, "%s input %d is driven twice" % (d, dp))
            self.src[d][dp - 1] = s
        for n in self.names:
            for k, s in enumerate(self.src[n]):
                if s is None:
                    # Simulink grounds an unconnected input (0) and runs the model; it is a
                    # design the engine does not take, not a malformed request
                    raise GraphError("UNSUPPORTED", "design.blocks." + n, "input port %d is not connected; Simulink "
                                     "grounds an unconnected input (0) and runs, which is not supported here -- "
                                     "connect it (a Constant 0 of the right type where a zero is meant)" % (k + 1))
        self.inports = [n for n in self.names if self.btype[n] == "Inport"]
        self.outports = [n for n in self.names if self.btype[n] == "Outport"]
        self.delays = [n for n in self.names if self.btype[n] in DELAYS]
        self._schedule()
        # hidden C-core slots of blocks the front-end expands (structure only, independent of types)
        self.hidden = []
        for n in self.names:
            bt, p = self.btype[n], self.params[n]
            if bt == "DiscreteFir":
                try:
                    nc = len(fc._dfir_coefs(p))
                except ValueError:
                    raise GraphError("BAD_PARAM", "design.blocks.%s.Coefficients" % n, "numeric vector required")
                if not (1 <= nc <= 4096):
                    raise GraphError("LIMIT_EXCEEDED", "design.blocks.%s.Coefficients" % n, "1..4096 coefficients")
                self.hidden += [_hid(n, "z%d" % k) for k in range(1, nc)] + [_hid(n, "p%d" % k) for k in range(nc)]
                fs_ = p.get("FilterStructure", "Direct form")
                if fs_ == "Direct form transposed":
                    self.hidden += [_hid(n, "nz%d" % k) for k in range(1, nc)]
                elif fs_ in ("Direct form symmetric", "Direct form antisymmetric"):
                    self.hidden += [_hid(n, "t%d" % k) for k in range(nc // 2)]
            elif bt == "Delay":
                try:
                    L = int(p["DelayLength"])
                except ValueError:
                    raise GraphError("BAD_PARAM", "design.blocks.%s.DelayLength" % n, "integer required")
                if not (1 <= L <= 65536):
                    raise GraphError("UNSUPPORTED", "design.blocks.%s.DelayLength" % n, "1..65536 supported")
                self.hidden += [_hid(n, "d%d" % k) for k in range(1, L)]
            elif bt in ("CompareToConstant", "CompareToZero"):
                self.hidden.append(_hid(n, "c"))
            elif bt == "DataTypeConversion" and str(p.get("OutDataTypeStr", "")).strip() == "boolean":
                self.hidden.append(_hid(n, "z"))   # the zero compared against
            elif bt == "Saturation":  # cast before the clamp when the output type is explicit
                self.hidden.append(_hid(n, "c"))
            elif bt == "RelationalOperator":  # cast inputs of a lossy comparison
                self.hidden += [_hid(n, "ra"), _hid(n, "rb")]
            elif bt == "MinMax" and 3 <= len(self.src.get(n, ())) <= 5:
                # the pairwise chain of recordings (fxcore.minmax_chain_types): one running result
                # per step but the last; unused unless the block is in that recorded span
                self.hidden += [_hid(n, "mm%d" % k) for k in range(1, len(self.src[n]) - 1)]
            elif bt in HIDDEN_P4B2:  # C-core expansions of these block types
                self.hidden += [_hid(n, t) for t in HIDDEN_P4B2[bt]]
            elif bt in ("Switch", "MultiportSwitch"):   # boolean data ports on the C core: 0 and (u ~= 0) slots
                self.hidden += [_hid(n, "bz"), _hid(n, "nz1"), _hid(n, "nz2"), _hid(n, "nz3")]
        # multirate lowering: stateful blocks of a slow domain advance their state only on their hits
        self.gated = {}
        for n in self.names:
            g = self.params[n].get("_GateSteps")
            if g is None:
                continue
            P, o = (int(v) for v in str(g).split(","))
            bt = self.btype[n]
            if bt == "DiscreteFir":
                k = max(len(fc._dfir_coefs(self.params[n])) - 1, 0)
            elif bt in DSTATEFUL:
                k = 1
            else:
                raise GraphError("BAD_PARAM", "design.blocks.%s._GateSteps" % n, "only for stateful blocks")
            self.gated[n] = (P, o, self.params[n]["_GateSlot"], k)
            self.hidden += [_hid(n, "g%d" % i) for i in range(k)]
        self.slots = self.names + self.hidden

    # ---- execution order: Unit Delay outputs do not depend on their input within a step
    def _schedule(self, split=None):
        """split: Discrete-Time Integrators (Forward Euler: the output is the state, no direct feedthrough) whose
        readers need not wait for them -- used only when the plain order has a cycle through an integrator.
        Simulink runs PI / leaky-integrator loops closed through the integrator alone. Such an integrator is
        still scheduled after its input (its update needs it); the step starts by reading its state."""
        split = set(split or ())
        indeg = {n: 0 for n in self.names}
        users = {n: [] for n in self.names}
        for n in self.names:
            if self.btype[n] in DELAYS:
                continue
            deps = [s for s in self.src[n] if s not in split]
            if self.params[n].get("_GateSlot") in self.btype:
                deps.append(self.params[n]["_GateSlot"])   # a gated block reads its enable in the same step
            for s in deps:
                indeg[n] += 1
                users[s].append(n)
        ready = sorted(n for n in self.names if indeg[n] == 0)
        order = []
        import heapq
        heapq.heapify(ready)
        while ready:
            n = heapq.heappop(ready)
            order.append(n)
            for u in users[n]:
                indeg[u] -= 1
                if indeg[u] == 0:
                    heapq.heappush(ready, u)
        if len(order) != len(self.names):
            stuck = {n for n in self.names if indeg[n] > 0}
            # the blocks merely downstream of the loop were listed too -- keep only those that also
            # feed a stuck block (peeling the ones that feed none leaves the cycles and the paths between them)
            loop = set(stuck)
            while True:
                tail = {n for n in loop if not any(u in loop for u in users[n])}
                if not tail:
                    break
                loop -= tail
            cyc = sorted(loop or stuck)
            dti = [x for x in cyc if self.btype[x] == "DiscreteIntegrator" and fc.dti_method(self.params[x]) == "FE"]
            if dti and not split and not any(self.params[x].get("_GateSlot") or self.params[x].get("_GateSteps")
                                             for x in self.names):
                # every Forward Euler integrator (the only method the engine accepts) breaks the loop like a delay
                return self._schedule([x for x in self.names if self.btype[x] == "DiscreteIntegrator"
                                       and fc.dti_method(self.params[x]) == "FE"])
            if dti:
                # only when every cycle passes through an integrator (a genuine algebraic loop next to an
                # integrator loop was reported as the integrator case)
                rest = set(cyc) - set(dti)
                deg = {x: sum(1 for s_ in self.src[x] if s_ in rest) for x in rest}
                q_ = [x for x in rest if deg[x] == 0]
                seen_ = 0
                while q_:
                    x = q_.pop()
                    seen_ += 1
                    for u in users[x]:
                        if u in rest:
                            deg[u] -= 1
                            if deg[u] == 0:
                                q_.append(u)
                if seen_ != len(rest):
                    dti = []
            if dti:
                # Simulink runs a loop closed through a Forward Euler integrator (no direct
                # feedthrough); the engine evaluates the integrator with its input of the same step -- not yet
                # supported (the recorded 11 loop recordings are requested)
                raise GraphError("UNSUPPORTED", "design.blocks." + dti[0], "a feedback loop closed only through a "
                                 "Discrete-Time Integrator (%s) is not supported yet: put a Unit Delay in the loop, "
                                 "or use an Accumulator form with a Unit Delay (loop: %s)"
                                 % (", ".join(dti[:3]), ", ".join(cyc[:10])))
            dp_ = [x for x in cyc if self.btype[x] == "DelayPort" and
                   str(self.params[x].get("DelayLengthSource", "Dialog")) != "Input port"]
            if dp_ and _delay_enable_reset(self.params[dp_[0]]):
                # the enabled / resettable Delay (engine/delayen_domain.json): recorded outside feedback loops only
                raise GraphError("UNSUPPORTED", "design.blocks." + dp_[0], "a feedback loop through a Delay with an "
                                 "enable or external reset port (%s) is not supported (not recorded) (loop: %s)"
                                 % (dp_[0], ", ".join(cyc[:10])))
            if dp_:
                # a Delay whose initial condition comes from a port still delays its data input, so
                # Simulink runs the loop; the port-sourced Delay is not recorded inside loops -- not supported, and not
                # an algebraic loop (with the LENGTH from a port Simulink itself reports one: kept below)
                raise GraphError("UNSUPPORTED", "design.blocks." + dp_[0], "a feedback loop through a Delay with its "
                                 "initial condition from a port (%s) is not supported (not recorded); use a dialog "
                                 "initial condition there, or keep the port-sourced Delay outside the loop (loop: %s)"
                                 % (dp_[0], ", ".join(cyc[:10])))
            raise GraphError("ALGEBRAIC_LOOP", "design.blocks." + cyc[0],
                             "cycle without a Unit Delay through: %s" % ", ".join(cyc[:10]) +
                             (" (and %d more)" % (len(cyc) - 10) if len(cyc) > 10 else ""))
        self.order = order
        self.dti_split = sorted(split)
        self._build_param_records()

    def _build_param_records(self):
        """Parameter records (engine/fxrecords.py): what the parameters alone fix, available without compiling."""
        import fxrecords as R
        self.param_recs = {}
        for n in self.names:
            bt, p = self.btype[n], self.params[n]
            try:
                if bt == "CounterLimited":
                    self.param_recs[n] = R.CounterParams(limit=int(str(p["uplimit"]).strip()))
                elif bt == "CounterFreeRunning":
                    self.param_recs[n] = R.CounterParams(limit=(1 << int(str(p["NumBits"]).strip())) - 1)
                elif bt in R.DETECT_OPS:
                    self.param_recs[n] = R.DetectParams(relop=R.DETECT_OPS[bt], vinit=float(p["vinit"]))
                elif bt == "Hold":
                    self.param_recs[n] = R.HoldParams(ic=float(p["InitialCondition"]))
                elif bt == "DiscreteFir":
                    cs = fc._dfir_coefs(p)
                    self.param_recs[n] = R.FirParams(coefs=tuple(cs), init=tuple(_fir_init_double(p, len(cs))),
                                                     structure=str(p.get("FilterStructure")))
                elif bt == "DiscreteIntegrator":
                    lim = (float(p["LowerSaturationLimit"]), float(p["UpperSaturationLimit"])) \
                        if p.get("LimitOutput") == "on" else None
                    self.param_recs[n] = R.DtiParams(gain=fc.dti_gain(p), ic=_dti_ic_double(p), limits=lim)
            except (ValueError, KeyError, R.RecordError):
                # a malformed or inconsistent value (a FIR's initial-state count, empty coefficients):
                # no record; the block's own checks report it with their code where it is used, in their order
                continue

    def param_record(self, n):
        rec = getattr(self, "param_recs", {}).get(n)
        if rec is None:
            # the parameters could not be interpreted when the design was built: the same answer the kernels' guard
            # gives for an unreadable value (BAD_PARAM), not a compiler defect
            raise GraphError("BAD_PARAM", "design.blocks." + n, "a parameter value of this %s block could not be "
                             "interpreted" % self.btype[n])
        return rec

    # ---- type resolution (needs Inport types)
    def compile(self, in_types):
        try:
            return self._compile_once(in_types)
        except GraphError as e:
            if e.code != "RELOWER" or not getattr(self, "_ram_auto", None) or getattr(self, "_relowered", False):
                if e.code == "RELOWER":   # (no original spec to lower again: never expected)
                    raise GraphError("INTERNAL", "design", "a signed enable could not be lowered again")
                raise
            # the enables whose compiled type is signed, marked for the > 0 lowering, once
            import fxhdl
            orig = self._ram_auto
            blocks = dict(orig["blocks"])
            for n in e.relower:
                if n in blocks:
                    blocks[n] = dict(blocks[n], _EnSigned=fxhdl.MARK)
                    if n in getattr(e, "relower_types", {}):
                        blocks[n]["_EnType"] = e.relower_types[n]
            orig2 = dict(orig, blocks=blocks)
            hw = self.hw
            self.__init__(orig2, hw)
            self._ram_auto = self._ram_auto or orig2
            self._relowered = True
            return self._compile_once(in_types)

    def _compile_once(self, in_types):
        if getattr(self, "_ram_auto", None):
            import fxhdl
            orig = self._ram_auto
            hw = self.hw
            self.__init__(fxhdl.typed_rewrite(orig, hw, in_types), hw)
            self._ram_auto = orig   # a later compile with other input types resolves the width again
            if any(isinstance(b, dict) and b.get("type") == "MATLABFunction" for b in orig.get("blocks", {}).values()):
                # a MATLAB Function was compiled into engine blocks: if that form does not compile, the interpreter
                # cores run it instead (the compiled form refused programs the interpreter runs)
                try:
                    return self._compile(in_types)
                except GraphError as e:
                    if os.environ.get("FX_NO_MLFN_FALLBACK"):
                        raise   # the compiler's tests: a compiled form that does not compile must show
                    first = e
                self.__init__(fxhdl.typed_rewrite(orig, hw, in_types, mlfn=False), hw)
                self._ram_auto = orig
                try:
                    return self._compile(in_types)
                except GraphError:
                    raise self._note_default(first)
        try:
            return self._compile(in_types)
        except GraphError as e:
            raise self._note_default(e)

    def _note_default(self, e):
        """a refusal of a parameter the design left out said only what is accepted ("Clip only"); it
        now says the value came from the library default and that the parameter should be written out."""
        parts = str(e.where or "").split(".")
        if e.code in ("UNSUPPORTED", "PORT_TYPE_INVALID") and len(parts) == 4 and parts[:2] == ["design", "blocks"]:
            blk, k = parts[2], parts[3]
            lib = (getattr(self, "assumed_defaults", {}).get(blk) or {}).get(k)
            if lib is not None and "left out" not in e.message:
                e = GraphError(e.code, e.where, "%s -- note: %s was left out, so the library default %r was used"
                               % (e.message, k, lib))
        return e

    def _compile(self, in_types):
        r = self._compile_inner(in_types)
        import parameter_bounds
        parameter_bounds.check_widths(self)   # recorded word lengths of recorded blocks (recordings policy)
        return r

    def _compile_inner(self, in_types):
        for n, orig in getattr(self, "_bp_orig", {}).items():   # back propagation: types are chosen per compile
            self.params[n] = orig
        self._bp_orig = {}
        self.otype = {}
        for n in self.inports:
            if n not in in_types:
                raise GraphError("MISSING_FIELD", "inputs." + n, "no data for Inport %s" % n)
            self.otype[n] = _chk(in_types[n], "inputs." + n)
        if getattr(self, "hdl_info", None):
            import fxhdl
            fxhdl.pre_compile_checks(self, self.hdl_info, in_types)
        self._resolve_types()   # loop seeds: every choice is tried there (recordings name independence)
        if self._matgain_exact_rows():
            # the rows summed exactly: their element types changed -- resolve again from the Inports
            self.otype = {n: self.otype[n] for n in self.inports}
            self._resolve_types()
        if getattr(self, "hdl_info", None):
            import fxhdl
            fxhdl.post_compile_checks(self, self.hdl_info)
        self._refuse_unrecorded()
        self.param_notes = dict(getattr(self, "_lab2_pnotes", None) or {})   # the error reporting contract (_param_note)
        self._check_param_ranges()
        self._matrix_gate()   # after the parameter checks: an overflowing matrix parameter is Simulink's own error
        # compiled records (engine/fxrecords.py) are built by each block's kernel from its current types, so the
        # incremental compiler (which calls _kernel directly, one type assignment after another) gets fresh ones
        self.compiled_recs = {}
        self.runtime_diag = []   # blocks on which Simulink can stop the run
        self.vm, self.vm_ok, self.vm_dly = {}, True, {}
        self.vm_why = None
        self.acc_types, self.fir_types = {}, {}   # compiled internal types per block (cost counter); reset per compile
        self.slot_types = {}   # types of hidden delay slots (FIR taps, partial sums)
        # 64-bit fast path: every stored value must fit int64; blocks whose intermediates are wider
        # (vm_wide) are computed with 128-bit arithmetic inside the 64-bit path
        store_fast = all(t == BOOL or t[1] + 1 <= FAST_BITS for t in self.otype.values())
        self.kern, self.vm_wide = {}, set()
        for n in self.names:
            self.vm_fast = True
            self.kern[n] = self._kernel(n)
            if not self.vm_fast:
                self.vm_wide.add(n)
        self.vm_fast = store_fast
        self._check_port_types()

    def _check_port_types(self):
        """A port's declared data type must be the compiled type of its signal (Simulink reports a data type
        mismatch otherwise; a port never converts)."""
        for gi_, (bs, where) in enumerate(getattr(self, "same_types", [])):
            ts_ = sorted({self.type_str(b) for b in bs})
            if len(ts_) > 1 and gi_ in getattr(self, "bus_outport_groups", set()):
                raise GraphError("PORT_TYPE_INVALID", where, "a virtual bus with elements of different data types (%s) "
                                 "into an Outport: Simulink rejects it (InputPortCannotAcceptMixedDataTypeVirtualBus); "
                                 "select the elements with a Bus Selector, one Outport each" % ", ".join(ts_))
            if len(ts_) > 1:
                raise GraphError("UNSUPPORTED", where, "a Mux of signals with different data types (%s): Simulink "
                                 "rejected every recorded case (InputPortDataTypeMismatch); convert the "
                                 "inputs to one type first" % ", ".join(ts_))
        todo = list(self.port_types)
        for n in self.outports:
            v = str(self.params[n].get("OutDataTypeStr", "Inherit: auto")).strip()
            if not v.startswith("Inherit"):
                todo.append((n, v, "design.blocks.%s.OutDataTypeStr" % n))
        for b, v, where in todo:
            try:
                want = fc._named_type(v)
            except (ValueError, KeyError):
                want = None
            if want == "boolean":
                want = BOOL
            if not (isinstance(want, tuple) or want == BOOL):
                raise GraphError("UNSUPPORTED", where, "port data type %r (a fixed-point / integer type or boolean)" % v)
            got = self.otype[b]
            if (tuple(got) if isinstance(got, (tuple, list)) else got) != want:
                raise GraphError("TYPE_MISMATCH", where, "the port declares %s but its signal compiles to %s (a port "
                                 "does not convert: Simulink reports a data type mismatch)"
                                 % (v, self.type_str(b)))

    def _matrix_gate(self):
        """a matrix-signal element (or a new vector collapse) is answered only
        for a recorded tuple -- block, shape, parameter shapes, collapse, input types, output type, hardware
        (engine/matrix_domain.json, the regression check)."""
        for n in self.names:
            bt, p = self.btype[n], self.params[n]
            if "_MatKey" in p:
                its_ = [self.otype.get(x) for x in self.src[n] if x is not None]
                k_ = matrix_key(p["_MatKey"], its_, self.otype.get(n), fc._asic(self.hw))
                if k_ not in _matrix_domain() and bt == "UnitDelay" and isinstance(self.otype.get(n), tuple):
                    # an initial value outside the signal's type is Simulink's own error (fxpParameterOverflow,
                    # UnitDelay uint8): that, not "not recorded"
                    try:
                        ic_ = float(str(p.get("InitialCondition", "0")))
                    except ValueError:
                        ic_ = None
                    if ic_ is not None:
                        _param_overflow(ic_, self.otype[n], "design.blocks.%s.InitialCondition" % n,
                                        "Unit Delay %s: InitialCondition" % n, mode=self.pom)
                if k_ not in _matrix_domain():
                    raise GraphError("UNSUPPORTED", "design.blocks." + n.split("[")[0], "%s on a matrix signal (or a "
                                     "collapse over a vector) with this shape, parameter shape, collapse, types and "
                                     "hardware is not recorded (engine/matrix_domain.json); record it to open it"
                                     % bt)

    def _check_param_ranges(self):
        """Parameters quantised into a fixed-point type that overflow it (_param_overflow):
        Constant -- recorded Simulink error; Gain with an explicit parameter type, Saturation limits,
        Compare To Constant, Switch threshold -- not recorded, refused. Delays / Detect vinit / integrators / FIR /
        multirate initial values are checked where they are quantised."""
        for n in self.names:
            bt, p, ot = self.btype[n], self.params[n], self.otype.get(n)
            where = "design.blocks." + n
            try:
                fiv_ = fc._parse_fi(str(p.get("Value", ""))) if bt == "Constant" else None
                if bt == "Constant" and isinstance(ot, tuple) and fiv_ is not None:
                    # a fi(...) value going into another type is checked by its
                    # value -- fi(300,1,16,0) into int8 stops Simulink (fxpParameterOverflow, not a downcast error);
                    # fi(3,1,16,0) into int8 and fi(0.25,1,16,8) into fixdt(1,8,4) compile
                    val_ = Fraction(fc._quant_param(fiv_[0], *fiv_[1:])) / Fraction(2) ** fiv_[3]
                    _param_overflow(val_, ot, where + ".Value", "Constant %s: Value" % n, mode=self.pom,
                                    notes=getattr(self, "param_notes", None))
                elif bt == "Constant" and isinstance(ot, tuple):
                    v = fxnum.canonical(p.get("Value", "0"), "scalar")
                    if not isinstance(v, fxnum.NumError):
                        _param_overflow(float(str(v)), ot, where + ".Value", "Constant %s: Value" % n, mode=self.pom, notes=getattr(self, "param_notes", None))
                elif bt == "Gain":
                    pt = _fixdt(p.get("ParamDataTypeStr"), where + ".ParamDataTypeStr") \
                        if not str(p.get("ParamDataTypeStr", "")).startswith("Inherit") else None
                    v = fxnum.canonical(p.get("Gain", "1"), "scalar")
                    if pt is not None and not isinstance(v, fxnum.NumError):
                        _param_overflow(float(str(v)), pt, where + ".Gain", "Gain %s: Gain in ParamDataTypeStr" % n, mode=self.pom, notes=getattr(self, "param_notes", None))
                    it_ = self.otype.get(self.src[n][0]) if self.src[n] else None
                    if str(p.get("ParamDataTypeStr", "")).strip() == fc._SAME_INPUT and isinstance(it_, tuple) and \
                            not isinstance(v, fxnum.NumError):
                        # the parameter takes the input type (ufix8_En4 x 20, x -2 stop)
                        _param_overflow(float(str(v)), tuple(it_), where + ".Gain", "Gain %s: Gain in the input type "
                                        "('Inherit: Same as input')" % n, mode=self.pom,
                                        notes=getattr(self, "param_notes", None))
                    if pt is None and isinstance(it_, tuple) and not isinstance(v, fxnum.NumError) and \
                            not (float(str(v)) < 0 and not it_[0]):
                        try:
                            _param_note(float(str(v)), tuple(self._gain_param_type(n, it_)), where + ".Gain",
                                        self.param_notes)
                        except GraphError:
                            pass   # refused by the kernel with its own message
                    if pt is None and self.pom == "error" and isinstance(it_, tuple) and not it_[0] and \
                            not isinstance(v, fxnum.NumError) and float(str(v)) < 0 and \
                            str(p.get("ParamDataTypeStr", fc._INHERIT_RULE)).strip() == fc._INHERIT_RULE:
                        # The inferred parameter inherits input signedness. A negative gain
                        # cannot be represented when that input is unsigned.
                        code_ = "PORT_TYPE_INVALID"   # ASIC/FPGA too (13 of 13)
                        raise GraphError(code_, where + ".Gain", "Gain %s: Gain %s is negative, but with "
                                         "ParamDataTypeStr 'Inherit: Inherit via internal rule' the parameter takes "
                                         "the input's signedness and the input %s is unsigned%s; use a signed input or "
                                         "an explicit signed ParamDataTypeStr" % (
                                             n, _fmt_num(float(str(v))), _tstr(tuple(it_)),
                                             " (Simulink stops with fxpParameterOverflow under its default diagnostics)"))
                elif bt == "Saturation" and isinstance(ot, tuple) and not p.get("_Clamp"):
                    # (_Clamp: the index clamp of recorded Direct Lookup lowering -- an engine construct, not a user
                    # parameter; past the index type's range it is a no-op at the bound)
                    for k in ("UpperLimit", "LowerLimit"):
                        v = str(p.get(k, "")).strip()
                        if v.lstrip("+-") in ("inf", "Inf"):
                            if self.pom == "error":
                                # an infinite limit is an overflow too (UpperLimit inf,
                                # LowerLimit -inf, both: fxpParameterOverflow)
                                raise GraphError("PORT_TYPE_INVALID", where + "." + k, "Saturation %s: %s %s is an "
                                                 "overflow of %s (Simulink stops with fxpParameterOverflow under its "
                                                 "default diagnostics); write a finite limit within the type, or set "
                                                 "hw.ParameterOverflowMsg 'warning' / 'none' to saturate it to the "
                                                 "type's bound" % (n, k, v, _tstr(tuple(ot))))
                        elif v:
                            _param_overflow(float(v), ot, where + "." + k, "Saturation %s: %s" % (n, k), mode=self.pom, notes=getattr(self, "param_notes", None))
                elif bt == "CompareToConstant" and self.src[n] and isinstance(self.otype.get(self.src[n][0]), tuple):
                    _param_overflow(fc._const_value(p["const"]), self.otype[self.src[n][0]], where + ".const",
                                    "Compare To Constant %s: const in the input type" % n, mode=self.pom, notes=getattr(self, "param_notes", None))
                elif bt in NL_PARAMS and self.src[n] and isinstance(self.otype.get(self.src[n][0]), tuple):
                    # the parameters are quantised to the input type; dz / rl / wz
                    # overflows (a negative Start with an unsigned input, Threshold 10 in sfix8_En4, a rising slew 4
                    # in sfix8_En8) stop Simulink with fxpParameterOverflow
                    for k in NL_PARAMS[bt]:
                        v_, what_ = self._nl_param_value(n, k)
                        _param_overflow(v_, self.otype[self.src[n][0]], where + "." + k, what_, mode=self.pom,
                                        notes=getattr(self, "param_notes", None))
                elif bt == "Switch" and len(self.src[n]) > 1 and isinstance(self.otype.get(self.src[n][1]), tuple) \
                        and str(p.get("Criteria", "")).strip() != "u2 ~= 0":
                    _param_overflow(float(p["Threshold"]), self.otype[self.src[n][1]], where + ".Threshold",
                                    "Switch %s: Threshold in the control type" % n, mode=self.pom, notes=getattr(self, "param_notes", None))
            except (TypeError, ValueError, KeyError):
                continue   # unparsable values are refused by the block's own checks

    def _on_cycle(self, n):
        """Whether block n lies on a (delayed) feedback loop."""
        users = {}
        for x in self.names:
            for y in self.src[x]:
                users.setdefault(y, set()).add(x)
        seen, todo = set(), [n]
        while todo:
            z = todo.pop()
            for u in users.get(z, ()):
                if u == n:
                    return True
                if u not in seen:
                    seen.add(u)
                    todo.append(u)
        return False

    def _nl_param_value(self, n, k):
        """(value, text) of a Dead Zone / Wrap To Zero / Rate Limiter parameter as Simulink quantises it into the
        input type: the rising slew limit times the sample period (Simulink's
        overflow names the 'Weighted Rise Limit (internal variable)'), the others as written."""
        bt, p = self.btype[n], self.params[n]
        v = fc._const_value(p[k])
        if bt == "RateLimiter" and k == "RisingSlewLimit":
            t = str(p.get("_RlT", "1"))
            return (v * float(t), "Rate Limiter %s: RisingSlewLimit%s in the input type (Simulink's Weighted Rise "
                    "Limit)" % (n, "" if t == "1" else " times the sample period " + t))
        return v, "%s %s: %s in the input type" % (bt, n, k)

    def _matgain_exact_rows(self):
        """Gain Matrix(K*u) into an unsigned 32-bit output whose products are integers (no fraction, so no shift into
        the output fraction 0) of at most 24 bits: Simulink sums the row exactly and saturates / wraps once
        (recordings,
        recordings -- every such design; the per-product rule fails on
        them). The lowered row (fxvector._matgain_lower: element Gains rounded into the output, a chain of Sums in
        the output type) is re-typed for this compile: each product kept exact, the partial sums in a signed type
        wide enough for the whole row, one cast into the output at the last Sum. The range predicate
        (engine/matgain_range.py) decides where this is answered. Returns True when a row was re-typed."""
        orig = getattr(self, "_mgx_orig", {})
        for n, o in orig.items():   # per compile: start from the lowered parameters
            self.params[n] = o
        self._mgx_orig = {}
        rows = {}
        for g in self.names:
            p = self.params[g]
            if self.btype[g] != "Gain" or "_MatGain" not in p:
                continue
            ot = _fixdt(str(p.get("OutDataTypeStr", "")).strip(), "design.blocks.%s.OutDataTypeStr" % g) \
                if str(p.get("OutDataTypeStr", "")).strip().startswith("fixdt") else None
            pt = _fixdt(str(p.get("ParamDataTypeStr", "")).strip(), "design.blocks.%s.ParamDataTypeStr" % g) \
                if str(p.get("ParamDataTypeStr", "")).strip().startswith("fixdt") else None
            it = self.otype.get(self.src[g][0]) if self.src.get(g) else None
            if ot is None or pt is None or not (isinstance(it, tuple) and len(it) == 3):
                continue
            import matgain_range as _mgr
            if not (ot == (0, 32, 0) and _mgr.u32_exact_sum(it, pt)):
                continue   # section 11: only built-in integer operands with products of at most 24 bits
            rows.setdefault(p["_MatGain"], []).append((g, it, pt))
        changed = False
        for own, gs in rows.items():
            sums = [x for x in self.names if self.btype[x] == "Sum" and self.params[x].get("_MatGain") == own]
            gset = {g for g, _it, _pt in gs}
            all_g = [x for x in self.names if self.btype[x] == "Gain" and self.params[x].get("_MatGain") == own]
            if set(all_g) != gset or not sums:
                continue   # every product of the matrix must be of this kind (one input type, one parameter type)
            it, pt = gs[0][1], gs[0][2]
            sp_, pw = (1 if (it[0] or pt[0]) else 0), it[1] + pt[1]
            ncol = len(all_g) // len([x for x in sums if not any(x in self.src.get(y, ()) for y in sums)])
            wide = "fixdt(1,%d,0)" % (pw + (ncol - 1).bit_length() + 1)
            for g in all_g:
                self._mgx_orig[g] = self.params[g]
                self.params[g] = dict(self.params[g], OutDataTypeStr="fixdt(%d,%d,0)" % (sp_, pw))
            for x in sums:
                self._mgx_orig[x] = self.params[x]
                last = not any(x in self.src.get(y, ()) for y in sums)
                q = dict(self.params[x], AccumDataTypeStr=wide, _MatGainOut=self.params[x]["OutDataTypeStr"])
                if not last:
                    q["OutDataTypeStr"] = wide
                self.params[x] = q
            changed = True
        return changed

    def _refuse_unrecorded(self):
        """Parameter values the engine used to accept although no recording shows the behaviour (parameter sweep
        after recordings): refused until recorded."""
        for n in self.names:
            bt, p, where = self.btype[n], self.params[n], "design.blocks." + n
            if "_MatGain" in p and bt == "Sum":
                # Gain Matrix(K*u): only the recorded types, rounding,
                # overflow and hardware (fxvector.MATGAIN_SHAPES for the shapes)
                own = p["_MatGain"]
                gs = [x for x in self.src[n] if x is not None and self.btype.get(x) == "Gain"]
                its = {self.otype.get(self.src[g][0]) for g in gs if self.src.get(g)}
                pts = {str(self.params[g].get("ParamDataTypeStr", fc._INHERIT_RULE)).strip() for g in gs}
                why = None
                ot_mg = str(p.get("_MatGainOut", p.get("OutDataTypeStr"))).strip()
                sat_mg = str(p.get("SaturateOnIntegerOverflow", "off"))
                rnd_mg = str(p.get("RndMeth", "Floor"))
                user_ot = p.get("_MatGainOut")
                n_gains = sum(1 for x in self.names if self.btype.get(x) == "Gain" and
                              self.params[x].get("_MatGain") == own)

                def _outs_ok(outs_):
                    if not callable(outs_):
                        return ot_mg in outs_
                    try:
                        return outs_(its, ot_mg, sat_mg, user=user_ot)
                    except TypeError:
                        return outs_(its, ot_mg, sat_mg)
                fam = [k for k, (ins_, pts_, outs_, rnds_, sats_) in self.MATGAIN_FAMILIES.items()
                       if its <= ins_ and pts <= pts_ and rnds_ is not None and rnd_mg in rnds_ and sat_mg in sats_ and
                       _outs_ok(outs_) and (user_ot != "Inherit: Same as input" or k in ("lab2_105", "keypath"))]
                if n_gains not in (4, 9):   # show 2x2 and 3x3 only
                    fam = [k for k in fam if k != "lab2_105"]
                if n_gains == 9:
                    fam = [k for k in fam if k in self.MATGAIN_33]
                    if not fam:
                        why = "a 3x3 matrix and these types (3x3 is recorded in the two supported type families only)"
                if n_gains == 16:   # 4x4: the recorded configuration only
                    fam = [k for k in fam if k == "keypath"]
                    if not fam:
                        why = 'a 4x4 matrix and these types (only one 4x4 configuration was recorded)'
                key_ = p.get("_MatGainKey")
                if key_ and len(its) == 1 and isinstance(next(iter(its)), tuple):
                    pt_, ot0_, shp_ = json.loads(key_)
                    if json.dumps([list(next(iter(its))), str(pt_).strip(), str(ot0_).strip(), rnd_mg, sat_mg,
                                   bool(fc._asic(self.hw)), shp_]) in _matgain_domain():
                        fam, why = ["basic_fill"], None   # this exact recorded tuple (main)
                range_why = None
                asic_ok_ = set(fam) & (self.MATGAIN_ASIC | {"basic_fill"})
                if (not fam or (fc._asic(self.hw) and not asic_ok_)) and key_ and len(its) == 1 and \
                        isinstance(next(iter(its)), tuple):
                    # the range predicate D (engine/matgain_range.py), host and ASIC
                    import matgain_range as _mgr
                    pt_, ot0_, shp_ = json.loads(key_)
                    ok_, range_why = _mgr.in_range(tuple(next(iter(its))), pt_, ot0_, rnd_mg, sat_mg, shp_,
                                                     bool(fc._asic(self.hw)))
                    if ok_:
                        fam, why = ["range"], None
                    elif range_why and not fam:
                        why = range_why   # (the range predicate's reason is the specific one: the engine's wording)
                if why:
                    pass
                elif fc._asic(self.hw) and fam and not set(fam) & (self.MATGAIN_ASIC | {"basic_fill", "range"}):
                    why = "ASIC / FPGA hardware"
                elif not fam:
                    why = range_why or "input %s, ParamDataTypeStr %s, OutDataTypeStr %s, RndMeth %s, saturation %s" % (
                        ", ".join(_tstr(t) if isinstance(t, tuple) else str(t) for t in its), ", ".join(sorted(pts)),
                        ot_mg, rnd_mg, sat_mg)
                if why:
                    raise GraphError("UNSUPPORTED", "design.blocks." + own, "Gain Matrix(K*u) with %s is not recorded "
                                     "(the recorded families: engine/fxgraph_lib.py MATGAIN_FAMILIES -- the "
                                     "lookup / matrix dense map, the branch map and the precision-loss map); record "
                                     "this shape to open it" % why)
            if bt in ("CompareToConstant", "DetectChange", "DetectIncrease", "DetectDecrease", "CompareToZero"):
                ot = str(p.get("OutDataTypeStr", "boolean")).strip()
                if ot not in ("boolean", "uint8"):
                    raise GraphError("UNSUPPORTED", where + ".OutDataTypeStr", "%s output type %s: only boolean and "
                                     "uint8 have been recorded" % (bt, ot))
            elif bt == "Product" and "/" in str(p.get("Inputs", "")) and self.otype.get(n) not in (None, BOOL):
                # Refuse net division exponents outside the supported range.
                ops = str(p["Inputs"]).replace("|", "").strip()
                ts_ = [self.otype.get(x) for x in self.src[n]]
                if ops and not ops.isdigit() and len(ops) == len(ts_) and all(t not in (None, BOOL) for t in ts_):
                    E = self.otype[n][2] - sum(t[2] for t, op in zip(ts_, ops) if op == "*") + \
                        sum(t[2] for t, op in zip(ts_, ops) if op == "/")
                    if not DIV_NET_EXP[0] <= E <= DIV_NET_EXP[1]:
                        raise GraphError("UNSUPPORTED", where, "division with a net scaling of 2^%d (output fraction "
                                         'length + divisor fractions - dividend fractions): only %d.. %d have been '
                                         "recorded; Simulink's result beyond is its own "
                                         '-- rescale the operands' % (E, DIV_NET_EXP[0], DIV_NET_EXP[1]))
            elif bt == "MultiportSwitch":
                ct = self.otype.get(self.src[n][0])
                if ct == BOOL and p.get("_SelIdx"):
                    # a boolean index into a Selector's index port -- Simulink refuses it
                    raise GraphError("UNSUPPORTED", "design.blocks." + str(p["_SelIdx"]), "Selector index port fed "
                                     "by a boolean signal: Simulink refuses it; give an integer index")
                if ct not in (None, BOOL) and ct[2] > 0:
                    raise GraphError("UNSUPPORTED", where, "Multiport Switch control with fraction bits (%s): only "
                                     "integer-scaled controls have been recorded" % (ct,))
                if p.get("DataPortOrder") == "Specify indices":
                    # recordings (accept only the recorded shapes; recordings,
                    # recordings): a uint8 / uint16 control with 2 .. 6 indices in 0 .. 15, or an
                    # int8 control with indices in -4 .. 15; data ports sfix16_En8 / sfix12_En4 / uint8 (mixed);
                    # DiagnosticForDefault None or Warning; host or ASIC; outside feedback loops
                    ix_ = fc.mps_indices(p)
                    dts_ = {self.otype.get(x) for x in self.src[n][1:]}
                    lo_ = -4 if ct == (1, 8, 0) else 0
                    old_ok_ = ct in ((0, 8, 0), (0, 16, 0), (1, 8, 0)) and 2 <= len(ix_) <= 6 and \
                        all(lo_ <= x <= 15 for x in ix_) and dts_ <= {(1, 16, 8), (1, 12, 4), (0, 8, 0)} and \
                        str(p.get("DiagnosticForDefault", "None")) in ("None", "Warning")
                    # recordings (200 models): the exact recorded tuples (engine/msh_domain.json)
                    key_ = json.dumps([list(ct) if isinstance(ct, tuple) else ct,
                                       [list(self.otype.get(x)) if isinstance(self.otype.get(x), tuple) else
                                        self.otype.get(x) for x in self.src[n][1:]],
                                       str(p.get("OutDataTypeStr", fc._INHERIT_RULE)).strip(),
                                       str(p.get("DataPortForDefault", "Last data port")),
                                       str(p.get("DiagnosticForDefault", "None")), ix_ == sorted(ix_), len(ix_),
                                       bool(fc._asic(self.hw))])
                    rng_ = _msh_domain()["MS"].get(key_)
                    new_ok_ = rng_ is not None and all(rng_[0] <= x <= rng_[1] for x in ix_)
                    if not (old_ok_ or new_ok_) or self._on_cycle(n):
                        raise GraphError("UNSUPPORTED", where, "Multiport Switch 'Specify indices': this setting (control "
                                         "type, data port types, output type, default port, diagnostic, index order "
                                         "and range, hardware) is not one of the recorded ones (engine/msh_domain.json; "
                                         "or a uint8 / uint16 control with 2 to 6 indices in 0 .. 15, int8: -4 .. 15, "
                                         "data ports sfix16_En8, sfix12_En4 or uint8), or it is inside a feedback "
                                         "loop; record a design of this shape to open it")
            elif bt == "MinMax" and len(self.src[n]) >= 3 and not fc._asic(self.hw):
                # on Intel, MinMax with 3+ inputs of different fraction lengths sometimes
                # selects on a grid coarser than the output (2^-8 with s16E8, s12E2, s20E12 -> sfix20_En10; rule not
                # determined), and an input outside the output's range wraps in an intermediate container
                #. Every recorded deviation coarsened to the grid of one of the
                # inputs, and Floor commutes with the selection, so the exact select-then-Floor answer is accepted
                # only when no input is coarser than the output (min input fraction length >= output fraction
                # length) and every input's range lies inside the output type (recordings
                # mm000 / 006 / 010 / 012 / 014 / 015, recordings show). ASIC/FPGA is exact and stays accepted.
                ins = [self.otype.get(sn) for sn in self.src[n]]
                ot = self.otype.get(n)
                why = None
                if ot in (None, BOOL) or any(t in (None, BOOL) for t in ins):
                    why = "a boolean or unresolved type"
                elif fc.minmax_coarse_span(ins, ot, self.words, str(p.get("RndMeth", "Floor"))):
                    # the recorded span of one input coarser than the output, answered
                    # by the pairwise chain (fxcore.minmax_chain_types); anything outside it keeps the refusal below
                    pass
                elif not ins[0][0] and not ins[1][0] and any(t[0] for t in ins[2:]):
                    # recordings (silent wrong before): with the first two inputs
                    # unsigned and a later one signed, Simulink compares in an unsigned intermediate -- a negative
                    # later input is not selected (min of (0, 0, -1) gives 0)
                    why = "the first two inputs unsigned and a later one signed (%s): Simulink compares in an " \
                          "unsigned intermediate type" % ", ".join(_tstr(t) for t in ins)
                elif fc.minmax_mixed_sign_many(ins, ot, self.words, str(self.params[n].get("RndMeth", "Floor")).strip()):
                    # the native minimal counterexample (
                    # sfix12_En6, ufix10_En6, sfix16_En10 into sfix16_En4,
                    # Nearest, step 31 -- Simulink 42, the engine 41): with an unsigned input among signed ones Simulink
                    # rounds the later input into the pairwise result type first (two roundings); under Floor the two
                    # roundings equal one (recordings 39 / recordings equal). One
                    # implementation (ref/fxcore.minmax_mixed_sign_many) here and in the C-core compile / _blk_minmax
                    why = fc.minmax_mixed_sign_many(ins, ot, self.words, str(self.params[n].get("RndMeth", "Floor")).strip())
                elif min(t[2] for t in ins) < ot[2]:
                    why = "an input with fewer fraction bits than the output %s (%s)" % (
                        _tstr(ot), ", ".join(_tstr(t) for t in ins if t[2] < ot[2]))
                else:
                    bad = [t for t in ins if not ((t[0] == 0 or ot[0]) and t[1] - t[0] - t[2] <= ot[1] - ot[0] - ot[2])]
                    if bad:
                        why = "an input whose range exceeds the output %s (%s)" % (
                            _tstr(ot), ", ".join(_tstr(t) for t in bad))
                if why:
                    raise GraphError("UNSUPPORTED", where, "MinMax with %d inputs on this hardware and %s: Simulink "
                                     "may select on a coarser grid or wrap in an intermediate type (rule not "
                                     "determined); give every input "
                                     "at least the output's fraction bits and a range inside it, use two-input "
                                     "MinMax blocks, or ASIC/FPGA hardware" % (len(self.src[n]), why))
            elif bt == "ShiftArithmetic" and str(p.get("BitShiftNumberSource", "Dialog")) == "Input port":
                # an int8 shift port, no binary-point shift, outside feedback loops
                st_ = self.otype.get(self.src[n][1]) if len(self.src[n]) > 1 else None
                dt_ = self.otype.get(self.src[n][0])
                old_ok_ = st_ == (1, 8, 0) and str(p.get("BinPtShiftNumber", "0")).strip() == "0" and \
                    dt_ in PORT_PARAM_TYPES and str(p.get("CheckOORBitShift", "off")) == "off"
                # recordings (180 models): the exact recorded tuples with their run-time shift
                # range (engine/msh_domain.json); CheckOORBitShift only off, or on with DiagnosticForOORShift None
                oor_ = str(p.get("CheckOORBitShift", "off"))
                rng_ = None
                if oor_ == "off" or str(p.get("DiagnosticForOORShift", "Warning")) == "None":
                    rng_ = _msh_domain()["SH"].get(json.dumps([
                        list(dt_) if isinstance(dt_, tuple) else dt_, list(st_) if isinstance(st_, tuple) else st_,
                        str(p.get("BitShiftDirection", "Bidirectional")), str(p.get("BinPtShiftNumber", "0")).strip(),
                        oor_, bool(fc._asic(self.hw))]))
                # the general rule over the recorded span (SH 180 + SH2 110 models,
                # recordings): any stored value shifted by the port value (left: wrapped into the word,
                # right: arithmetic), for data fixdt 8..32 bits with fraction 0..wl, an integer shift port (signed 8..32
                # bits or unsigned 4..16 bits), every direction, BinPtShiftNumber -3 .. 2, host or ASIC, the
                # out-of-range check off or on with each diagnostic (Error stops at the first net shift outside
                # 0 .. 31 / -31 .. 31; Warning is reported in simulink_warnings); shifts of any size follow
                # fxcore._port_shift (the int16 net shift with the binary-point shift, SH3)
                try:
                    bp_ = int(str(p.get("BinPtShiftNumber", "0")).strip())
                except ValueError:
                    bp_ = None
                if isinstance(st_, tuple) and len(st_) == 3 and st_[2] != 0:
                    # recordings (8 designs, sfix8_En2 / ufix8_En3
                    # ports): Simulink accepts only built-in types and fixed point with slope 1, bias 0 for the
                    # shift port (ShiftInvalidBitShiftOrBinPtShiftType)
                    raise GraphError("PORT_TYPE_INVALID", where, "Shift Arithmetic: the shift port's type %s has a "
                                     "fraction; Simulink accepts built-in types and fixed point with slope 1 "
                                     "(ShiftInvalidBitShiftOrBinPtShiftType)" % _tstr(st_))
                # uint32 shift ports (24
                # models): Simulink runs them while every port value is below 2^31 and stops at the first step with a
                # value of 2^31 or more (ShiftNonIntParam, a run-time error; fxcore._blk_shift)
                # SH3: unsigned ports recorded to 20 bits
                # (ufix20), signed int12 / int32 among the signed 8 .. 32
                gen_ok_ = isinstance(dt_, tuple) and len(dt_) == 3 and dt_[0] in (0, 1) and 8 <= dt_[1] <= 32 and \
                    0 <= dt_[2] <= dt_[1] and isinstance(st_, tuple) and len(st_) == 3 and st_[2] == 0 and \
                    ((st_[0] == 1 and 8 <= st_[1] <= 32) or (st_[0] == 0 and (4 <= st_[1] <= 20 or st_[1] == 32))) and \
                    bp_ is not None and -3 <= bp_ <= 2 and \
                    str(p.get("BitShiftDirection", "Bidirectional")) in ("Left", "Right", "Bidirectional") and \
                    (oor_ == "off" or (oor_ == "on" and str(p.get("DiagnosticForOORShift", "Warning")) in
                                       ("None", "Warning", "Error")))
                if self._on_cycle(n) or not (old_ok_ or rng_ or gen_ok_):
                    raise GraphError("UNSUPPORTED", where, "Shift Arithmetic with the shift from an input port: "
                                     "inside a feedback loop, or a setting outside the recorded span (data fixdt with "
                                     "8 .. 32 bits and fraction 0 .. word length, an integer shift port of 8 .. 32 "
                                     "signed or 4 .. 20 or 32 unsigned bits, BinPtShiftNumber -3 .. 2): not supported")
                if rng_:   # the recorded range of this setting (wider than the old -7 .. 7 where recorded so)
                    self.params[n] = dict(self.params[n], _ShiftRange="%d %d" % tuple(rng_))
                if gen_ok_:
                    self.params[n] = dict(self.params[n], _ShiftGeneral="1")
            elif bt in ("DTIStateFB", "DTIStateFBGain"):
                # the Gain's output type must be the integrator's (Simulink does not convert
                # at the initial-condition port: recordings InputPortDataTypeMismatch); boolean reset
                it_ = self.otype.get(self.src[n][0])
                rt_ = self.otype.get(self.src[n][1])
                if isinstance(it_, tuple):
                    s_, w_, f_, _sl = fc._dti_type(p, tuple(it_))
                    if fc._named_type(str(p.get("_SFGainOut", ""))) != (s_, w_, f_):
                        raise GraphError("PORT_TYPE_INVALID", where, "the Gain on the state port must output the "
                                         "integrator's type %s (Simulink does not convert at the initial-condition "
                                         "port: InputPortDataTypeMismatch)" % _tstr((s_, w_, f_)))
                if rt_ != BOOL or str(p.get("OutDataTypeStr", fc._INHERIT_RULE)) != fc._INHERIT_RULE:
                    raise GraphError("UNSUPPORTED", where, "integrator state-port feedback: a boolean reset and the "
                                     "internal-rule output type are recorded")
                # recordings (accept only the recorded shapes): input
                # sfix16_En8 / sfix12_En4, gain value 1 or 0.5, Forward Euler, no saturation, outside other loops
                try:
                    gv_ = float(str(p.get("gainval", "1.0")).strip())
                except ValueError:
                    gv_ = None
                if it_ not in ((1, 16, 8), (1, 12, 4)) or gv_ not in (1.0, 0.5) or fc.dti_method(p) != "FE" or \
                        str(p.get("LimitOutput", "off")) != "off":
                    raise GraphError("UNSUPPORTED", where, "integrator state-port feedback: recorded with an input "
                                     "sfix16_En8 or sfix12_En4, gain value 1 or 0.5, Forward Euler (Integration or "
                                     "Accumulation) and no output limits; record a design of this shape to open it")
            elif bt == "DelayPort" and _delay_enable_reset(p):
                self._delayen_check(n, p, where)
            elif bt == "DelayPort":
                # recordings, and recordings (
                # foundation completeness; 360 models, every combination below recorded on host and ASIC):
                #   length from a port: uint8 / uint16 / int8 / ufix4 length, DelayLengthUpperLimit 3 / 4 / 8 / 16 / 32
                #     (ufix4 with 16 / 32: Simulink DelayLengthUpperLimitValueOutOfRange), any dialog initial value;
                #   initial value from a port of the input's type: DialogLength 1 .. 6 / 8 / 16 / 32;
                #   both from ports (port 2 the length, port 3 the initial value), dialog initial value unused;
                #   data u8En4 / sfix12_En4 / sfix16_En8 and PORT_PARAM_TYPES; no feedback loop
                lp_ = str(p.get("DelayLengthSource", "Dialog")) == "Input port"
                ip_ = str(p.get("InitialConditionSource", "Dialog")) == "Input port"
                ts_ = [self.otype.get(s_) for s_ in self.src[n]]
                t0_ = ts_[0]
                lt_ = ts_[1] if lp_ else None
                it_ = ts_[2] if (lp_ and ip_) else (ts_[1] if ip_ else None)
                lim_ = str(p.get("DelayLengthUpperLimit", "100")).strip()
                if lp_ and not ip_ and isinstance(t0_, tuple):
                    # the dialog initial value is output before the history fills; outside
                    # the data type Simulink stops (fxpParameterOverflow under default diagnostics) -- it was quantised
                    try:
                        icv_ = float(fc._const_value(p.get("InitialCondition", "0")))
                    except (ValueError, TypeError):
                        raise GraphError("UNSUPPORTED", where + ".InitialCondition", "Delay %s: initial value %r with "
                                         "the length from a port: one scalar value is recorded"
                                         % (n, p.get("InitialCondition")))
                    if icv_ != icv_ or icv_ in (float("inf"), float("-inf")):
                        # (recordings NaN -> AttemptconvertNaN2Fix, Inf -> fxpParameterOverflow; both were INTERNAL)
                        raise GraphError("PORT_TYPE_INVALID", where + ".InitialCondition", "Delay %s: InitialCondition "
                                         "%s cannot be stored in %s (Simulink stops: %s)" % (
                                             n, p.get("InitialCondition"), _tstr(t0_), "AttemptconvertNaN2Fix"
                                             if icv_ != icv_ else "fxpParameterOverflow"))
                    _param_overflow(icv_, t0_, where + ".InitialCondition", "Delay %s: InitialCondition" % n,
                                    self.pom, getattr(self, "param_notes", None))
                if lp_ and lt_ == (0, 4, 0) and lim_ in ("16", "32"):
                    raise GraphError("BAD_PARAM", where + ".DelayLengthUpperLimit", "DelayLengthUpperLimit %s does not "
                                     "fit the ufix4 length port (Simulink DelayLengthUpperLimitValueOutOfRange)" % lim_)
                # recorded item b (recordings 23:2x: generalise over the recorded ranges, not tuples): with the
                # whitelist lifted, every recorded port-length / port-initial-condition Delay is equal to Simulink
                # (recordings 384 legs, recordings) -- answered over
                # the recorded spans: data fixdt 8..32 bits, fraction 0..wl; the length port one of the recorded
                # integer types; DelayLengthUpperLimit an integer 1..64 that the port can hold; an initial-condition
                # port of the data type; a dialog length 1..32 with a port initial condition
                def _int_(x):
                    try:
                        v_ = int(str(x).strip())
                    except ValueError:
                        return None
                    return v_ if str(v_) == str(x).strip() else None
                LEN_PORTS_ = ((0, 4, 0), (0, 8, 0), (0, 16, 0), (0, 32, 0), (1, 8, 0), (1, 16, 0))
                kp_ = isinstance(t0_, tuple) and 8 <= t0_[1] <= 32 and 0 <= t0_[2] <= t0_[1]
                lim_i_ = _int_(lim_) if lp_ else None
                # (a feedback loop through the Delay: only the recorded loop tuples below -- every recorded loop is a
                # Sum accumulator)
                bad_ = self._on_cycle(n) or not kp_ or \
                    (lp_ and (lt_ not in LEN_PORTS_ or lim_i_ is None or not 1 <= lim_i_ <= 64 or
                              lim_i_ > ((1 << (lt_[1] - lt_[0])) - 1))) or \
                    (ip_ and it_ != t0_) or \
                    (ip_ and not lp_ and not 1 <= (_int_(p.get("DelayLength", "2")) or 0) <= 32)
                if bad_ and isinstance(t0_, tuple) and json.dumps(
                        [list(t0_), list(lt_) if lp_ and isinstance(lt_, tuple) else None,
                         list(it_) if ip_ and isinstance(it_, tuple) else None,
                         lim_ if lp_ else str(p.get("DelayLength", "2")).strip(), bool(fc._asic(self.hw)),
                         bool(self._on_cycle(n))]) in _delayport_domain():
                    bad_ = False   # this exact recorded tuple (engine/delayport_domain.json)
                pdf_ = str(p.get("PreventDirectFeedthrough", "off")).strip()
                rm_ = str(p.get("ExternalReset", "None")).strip()
                if lp_ and (pdf_ != "off" or rm_ not in ("None", "none")):
                    # recordings (every step
                    # equal): PreventDirectFeedthrough 'on' reads a length below 1 as 1; ExternalReset 'Rising' fills
                    # the history with the initial condition at a step where the boolean reset changes from 0,
                    # before the output (ref/fxcore.py _blk_delayport). Recorded at one data / length type and limit
                    # each: the exact tuples of engine/delaymode_domain.json (the regression check --check)
                    rt_ = ts_[2] if len(ts_) > 2 else None
                    key_ = json.dumps([list(t0_) if isinstance(t0_, tuple) else t0_,
                                       list(lt_) if isinstance(lt_, tuple) else lt_, lim_, pdf_,
                                       "None" if rm_ == "none" else rm_,
                                       None if rm_ in ("None", "none") else
                                       ("boolean" if rt_ == BOOL else list(rt_) if isinstance(rt_, tuple) else rt_),
                                       bool(fc._asic(self.hw))])
                    if ip_ or self._on_cycle(n) or \
                            str(p.get("DiagnosticForDelayLength", "None")).strip() != "None" or \
                            key_ not in _delaymode_domain():
                        raise GraphError("UNSUPPORTED", where, "Delay %s with the length from a port and "
                                         "PreventDirectFeedthrough %r / ExternalReset %r: recorded only on the host with "
                                         "int8 data, an int8 length port, DelayLengthUpperLimit 4, a dialog initial "
                                         "condition, DiagnosticForDelayLength 'None', outside feedback loops, and "
                                         "either PreventDirectFeedthrough 'on' without a reset or ExternalReset "
                                         "'Rising' with a boolean reset port (engine/delaymode_domain.json); record a "
                                         "design of this shape to open it" % (n, pdf_, rm_))
                if bad_:
                    raise GraphError("UNSUPPORTED", where, "Delay with the length or initial condition from an input "
                                     "port: recorded with data fixdt of 8..32 bits (fraction 0..word length), a "
                                     "uint8 / uint16 / uint32 / int8 / int16 / ufix4 length port and an integer "
                                     "DelayLengthUpperLimit 1..64 it can hold, an initial-condition port of the data "
                                     "type (dialog length 1..32); record a design of this shape to open it")
            elif bt == "ShiftArithmetic":
                try:
                    k = int(str(p.get("BitShiftNumber", "0")).strip())
                except ValueError:
                    continue
                if k < 0 and str(p.get("BitShiftDirection", "Bidirectional")) != "Bidirectional":
                    raise GraphError("UNSUPPORTED", where + ".BitShiftNumber", "negative shift %d with direction %s: "
                                     "only Bidirectional has been recorded with negative counts"
                                     % (k, p.get("BitShiftDirection")))

    def _propagate(self):
        changed = True
        while changed:
            changed = False
            for n in self.names:
                if n in self.otype:
                    continue
                t = self._out_type(n)
                if t is not None:
                    self.otype[n] = _chk(t, "design.blocks." + n)
                    changed = True

    def _resolve_types(self):
        self._bp_sw_data = set()
        self._resolve_types_inner()
        self._loop_minmax_reader_check()
        for w in sorted(self._bp_sw_data):
            # answers an internal-rule Sum chain into a Switch data port only
            # where the other data input already has the type the chain arrives with (host int32 both); with another
            # type Simulink back-propagates that type into the Sum and rejects it (
            # InvDisagreeInternalRuleDType); recordings (160 seeded designs) confirms the
            # equal-type answers for many Sum types (82 / 82) and shows two refused designs Simulink runs with the
            # other data input's type throughout  -- left refused, not separated from the rejections
            a, b = (self.otype.get(self.src[w][j]) for j in (0, 2))
            if a is None or a != b:
                raise GraphError("UNSUPPORTED", "design.blocks.%s" % w,
                                 ('Switch %s: a Sum with an internal-rule output, fed by a source typed '
                                     "'Inherit via back propagation', reaches data port 1 or 3 while the two "
                                     'data inputs have different types (%s, %s); Simulink then back-propagates '
                                     "the other data input's type into the chain: it rejects the model where "
                                     'that disagrees with the internal rule and in some designs gives the '
                                     'source, the Sum and the chain that type instead; which one applies '
                                     'is not separated -- give the source or the Sum an explicit output type') % (w, _tstr(a) if a else "untyped", _tstr(b) if b else "untyped"))

    def _loop_minmax_reader_check(self):
        """Reject a MinMax reader that imposes a conflicting type on its feedback loop."""
        if not any(self.btype[x] == "MinMax" and not str(self.params[x].get("OutDataTypeStr", fc._INHERIT_RULE)).strip()
                   .startswith("Inherit") for x in self.names):
            return   # cheap exit: the shape needs a MinMax with an explicit type (this runs on every compile)
        users = {}
        for x in self.names:
            for y in self.src[x]:
                users.setdefault(y, set()).add(x)

        def reach(a, b):   # a path a -> ... -> b along signal flow
            seen_, todo_ = set(), [a]
            while todo_:
                z = todo_.pop()
                for u in users.get(z, ()):
                    if u == b:
                        return True
                    if u not in seen_:
                        seen_.add(u)
                        todo_.append(u)
            return False
        delays = [x for x in self.names if self.btype[x] in ("UnitDelay", "Delay")]
        loop = set()
        for dl in delays:
            if not reach(dl, dl):
                continue
            members = {x for x in self.names if x == dl or (reach(dl, x) and reach(x, dl))}
            if any(self.btype[x] == "Sum" and str(self.params[x].get("OutDataTypeStr", fc._INHERIT_RULE)).strip()
                   in ("", fc._INHERIT_RULE) for x in members):
                loop |= members
        if not loop:
            return
        for r_ in self.names:
            if self.btype[r_] not in ("Product", "Sum", "MinMax") or len(self.src[r_]) < 2 or r_ in loop:
                continue
            ins = self.src[r_]
            mms = [x for x in ins if self.btype.get(x) == "MinMax" and x not in loop and
                   not str(self.params[x].get("OutDataTypeStr", fc._INHERIT_RULE)).strip().startswith("Inherit") and
                   any(y in loop for y in self.src[x])]
            lps = [x for x in ins if x in loop]
            for m in mms:
                for l_ in lps:
                    if self.otype.get(m) is not None and self.otype.get(l_) is not None and self.otype[m] != self.otype[l_]:
                        raise GraphError("TYPE_MISMATCH", "design.blocks." + r_, "%s %s reads MinMax %s (explicit type %s) "
                                         "and %s of the feedback loop through %s (type %s): Simulink back-propagates "
                                         '%s into the loop and rejects the model (InvDisagreeInternalRuleDType'
                                         ") -- give the loop's delay or Sum an explicit type, or "
                                         "make the MinMax's type the loop's" % (
                                             self.btype[r_], r_, m, _tstr(self.otype[m]), l_,
                                             ", ".join(sorted(loop)[:6]), _tstr(self.otype[l_]),
                                             _tstr(self.otype[m])))

    def _resolve_types_inner(self):
        """Propagate explicit and inherited types, then resolve remaining cycles.
        Try up to SEED_TRIALS seed sequences. Accept when all outcomes agree on
        the types, reject a consistently invalid cycle, and refuse ambiguous outcomes."""
        self._propagate()
        if self._sf_loop_from_exit():
            self._propagate()
        for n in self.names:
            if n not in self.otype and self._backprop(n):
                raise GraphError("UNSUPPORTED", "design.blocks.%s.OutDataTypeStr" % n, "back propagation from a "
                                 "block whose destination's other input is not typed before it (a feedback loop or a "
                                 "chain of inherited types): not recorded; give %s an "
                                 "explicit type" % n)
        base = dict(self.otype)
        # a CIC Interpolation with a specified (not full-precision) output fed by a loop
        # signal: whether its output type starts the loop is not recorded and its type is not a plain type string --
        # refused wherever the design would be accepted (_downstream_decide)
        # copy d: a CIC Decimation (minimum section word lengths, 32-bit output) does not
        # start the loop -- recorded; CIC Interpolation is not
        self._cic_on_loop = [c for c in self.names if self.btype[c] == "CICInterpolation" and
                             str(self.params[c].get("filterInternals", "Full precision")) != "Full precision" and
                             any(x not in base for x in self.src[c])]
        # the FIR Decimation / Interpolation and CIC Decimation release covers only the native four
        # recorded configurations on an sfix8_En1 loop (MRFILTER_ON_LOOP); any other one fed by the loop is refused
        # where the design would be accepted
        self._mrfilter_on_loop = [c for c in self.names if self.btype[c] in MRFILTER_ON_LOOP and
                                  any(x not in base for x in self.src[c])]
        downstream_bad = self._downstream_start_check(base)   # applied only where the design would be accepted
        todo, outcomes, tried, direct = [[]], [], [], []
        while todo:
            if len(outcomes) >= SEED_TRIALS:
                raise GraphError("UNSUPPORTED", "design.blocks." + (getattr(self, "_seed_cands", None) or ["?"])[0],
                                 "inherited data types around feedback loops: more than %d ways to start the type "
                                 "resolution; give one block in each loop an explicit output type"
                                 % SEED_TRIALS)
            choices = todo.pop()
            self.otype = dict(base)
            amb = []
            self._seed_direct = False
            try:
                self._seed_cycles(choices, amb)
                outcomes.append(("ok", {n: self.otype[n] for n in self.names}))
            except GraphError as e:
                outcomes.append(("err", e))
            direct.append(self._seed_direct)
            tried.append(choices + [0] * len(amb))
            for off, k in enumerate(amb):   # the choice points this run met first: queue the other branches
                for c in range(1, k):
                    todo.append(choices + [0] * off + [c])
        # a start refused only for its multi-port loop entry gives way to another start's error, so the error does
        # not depend on which start was tried first (the regression check graph 424: TYPE_MISMATCH by one start)
        errs = [o[1] for o in outcomes if o[0] == "err"]
        first_err = next((e for e in errs if not getattr(e, "multi_port_loop", False)), errs[0] if errs else None)
        if len(outcomes) == 1:
            if outcomes[0][0] == "err":
                raise outcomes[0][1]
            self._downstream_decide(downstream_bad + self._late_bad(base, outcomes[0][1]), outcomes[0][1])
            self.otype = dict(base)
            self._seed_cycles(tried[0], [])   # the late-start check reused the state: rerun the accepted typing
            return
        oks = [(c, o[1]) for c, o in zip(tried, outcomes) if o[0] == "ok"]
        if not oks:
            raise first_err   # rejected whichever block starts the loop
        plain = [o for o, dr in zip(outcomes, direct) if not dr]
        if plain and all(o[0] == "err" for o in plain):
            # every start inside the inherited part rejects it (recordings group: Simulink rejects these);
            # only a start at an explicit block would type it -- rejected as before
            raise next((o[1] for o in plain if not getattr(o[1], "multi_port_loop", False)), plain[0][1])
        if len(oks) == len(outcomes) and all(t == oks[0][1] for _c, t in oks):
            # every choice gives the same typing: the result does not depend on the choice
            self._downstream_decide(downstream_bad + self._late_bad(base, oks[0][1]), oks[0][1])
            self.otype = dict(base)
            self._seed_cycles(oks[0][0], [])   # rerun it so every derived state belongs to it
            return
        cands = getattr(self, "_seed_cands", None) or ["?"]
        raise GraphError("UNSUPPORTED", "design.blocks." + cands[0],
                         "inherited data types around feedback loops: the result depends on which of %s starts the "
                         'type resolution (%d ways tried: %s); give one block in the loop an explicit output type'
                         "" % (", ".join(cands[:6]), len(outcomes),
                                         ", ".join(sorted({"accepted" if o[0] == "ok" else o[1].code
                                                           for o in outcomes}))))

    def _downstream_starts(self, base):
        """(signal, type) pairs Simulink may start an inherited loop from, from outside the loop (
        recordings): a Sum / Product / MinMax with three or more inputs, two or more of them
        typed, gives an untyped signal on another of its inputs one of those typed inputs' types; with fewer inputs or
        typed inputs, a typed input's type, and an explicit output type, are starts when wider than the loop signal's
        own type (the flag True marks those width-gated starts)."""
        out = []
        for c in self.names:
            ot_ = fc.parse_type(str(self.params[c].get("OutDataTypeStr", ""))) \
                if self.btype[c] not in INTERNAL_TYPES else None
            # (FIR Decimation / Interpolation and CIC filters have no OutDataTypeStr, so ot_ is None for them: their
            # outputs are never starts; recordings pins the recorded configurations instead, MRFILTER_ON_LOOP)
            rec = self.btype[c] in ("Sum", "Product", "MinMax") and len(self.src[c]) >= 2
            if ot_ is not None:
                # and its recording recordings:
                # an explicit output type of a block the loop feeds starts the loop -- int32 / int16 / sfix32_En8 /
                # ufix10_En1 do, int8 / uint8 against a ufix8_En1 loop do not (recorded at Sum / Product / MinMax of
                # two or three inputs: "wider"). recordings 
                # copies a / b / d / e (and recordings: a one-input
                # Product '/', narrower and wider): an int16 output of a one-input Sum '+' or Product '*', of a Gain and of a Data
                # Type Conversion does not start a ufix8_En1 loop (as recordings 578's DTC to sfix29_En14 in
                # recordings) -- not starts. Other blocks with an explicit output (a one-input
                # MinMax, Switch, ...) are not recorded: an output that is wider, or as wide but another type, counts
                # ("differs"; _counts)
                if not rec and (self.btype[c] in ("Gain", "DataTypeConversion") or
                                (self.btype[c] in ("Sum", "Product") and len(self.src[c]) == 1
                                 and str(self.params[c].get("Inputs", "")).strip() in ("+", "|+", "*", "1", "/"))):
                    ot_ = None
            # the explicit type itself, not only blocks typed before the loop (base) -- a Lookup1D /
            # DiscreteFir with an explicit int16 output on the loop was not typed there and slipped through
            tc = base.get(c, ot_) if ot_ is not None else None
            if isinstance(tc, tuple):
                for x in dict.fromkeys(x for x in self.src[c] if x not in base):
                    if self.btype.get(x) in INTERNAL_TYPES or not self.src.get(x):
                        continue
                    # recordings (recordings a .. f): narrower explicit
                    # outputs of Abs, Saturation and a one-input Sum '-' do not start the loop ("differs": wider or as
                    # wide but another type counts); for every other kind nothing is recorded ("any": a type other
                    # than the loop's counts, narrower too)
                    sg = str(self.params[c].get("Inputs", "")).strip()
                    narrow_rec = self.btype[c] in ("Abs", "Saturation") or \
                        (self.btype[c] == "Sum" and len(self.src[c]) == 1 and sg in ("-", "|-"))
                    out.append((x, tc, "wider" if rec else ("differs" if narrow_rec else "any")))
            if self.btype[c] == "RelationalOperator" and len(self.src[c]) == 2:
                # recordings (recordings copy c): a Relational Operator's
                # typed input (DTC to sfix16_En4) gives the loop signal on its other input its type as a two-input
                # Product's does: Simulink typed the loop sfix16_En4. recordings
                # (recordings a .. d): a narrower typed input (sfix6_En2,
                # ufix4_En1, sfix7_E2; either port) starts it too -- Simulink then rejects the model
                # (InvDisagreeInternalRuleDType): a start of any width (flag False).
                # recordings (24 probes, one change each): the start stays
                # whatever the relop's loop input (Sum / Unit Delay / Abs), DTC or Inport, operator, port or width;
                # it goes away only when the loop holds a MinMax by the internal rule with an input typed outside
                # the loop (varying those inputs one at a time preserved their respective
                # types) -- such a loop is typed from that input, not from the comparison
                for x in dict.fromkeys(x for x in self.src[c] if x not in base):
                    if self.btype.get(x) in INTERNAL_TYPES or not self.src.get(x):
                        continue
                    if self._loop_has_typed_minmax(x, base):
                        continue
                    for t_ in dict.fromkeys(base[y] for y in self.src[c] if y in base):
                        if t_ != BOOL:
                            out.append((x, t_, False))
            if self.btype[c] not in ("Sum", "Product", "MinMax") or len(self.src[c]) < 2:
                continue
            typed = [x for x in self.src[c] if x in base]
            if len(self.src[c]) < 3 or len(typed) < 2:
                # a two-input Product whose other input is sfix16_En4 (wider
                # than the loop's ufix8_En1) gives the loop that type; the same Product with an ufix8_En1 other
                # input does not (V0, V2), nor do narrower typed inputs (recordings's two-input and single-typed
                # cells) -- a typed input is then a start like a wider explicit output (_counts: wider only)
                for x in dict.fromkeys(x for x in self.src[c] if x not in base):
                    if self.btype.get(x) in INTERNAL_TYPES or not self.src.get(x):
                        continue
                    for t_ in dict.fromkeys(base[y] for y in typed):
                        if t_ != BOOL:
                            # V0 / V2's other
                            # input was the loop's own type; an as-wide input of another type (V1 with fixdt(1,8,-4))
                            # starts the loop -- Simulink typed it fixdt(1,8,-4): it counts
                            # recordings:
                            # a two-input MinMax's NARROWER typed input (ufix1_En1 against a uint32 loop) started the
                            # loop in Simulink, which then rejected the model -- for MinMax any other type counts
                            out.append((x, t_, "any" if self.btype[c] == "MinMax" else "differs"))
                continue
            for x in dict.fromkeys(x for x in self.src[c] if x not in base):
                if self.btype.get(x) in INTERNAL_TYPES or not self.src.get(x):
                    continue
                for t_ in dict.fromkeys(base[y] for y in typed):
                    if t_ != BOOL:
                        out.append((x, t_, False))
        return list(dict.fromkeys(out))

    def _loop_has_typed_minmax(self, x, base):
        """Whether the cycle(s) through the untyped signal x (its strongly connected component among the blocks not
        typed in base) hold a MinMax with an internal-rule output and an input typed in base (recorded-relst-*)."""
        names = [n for n in self.names if n not in base]
        fwd = {n: [m for m in names if n in self.src.get(m, ())] for n in names}

        def reach(start, nxt):
            seen, todo = {start}, [start]
            while todo:
                for m in nxt(todo.pop()):
                    if m not in seen:
                        seen.add(m)
                        todo.append(m)
            return seen
        scc = reach(x, lambda n: fwd[n]) & reach(x, lambda n: [m for m in self.src.get(n, ()) if m not in base])
        return any(self.btype[m] == "MinMax" and len(self.src[m]) >= 2 and
                   str(self.params[m].get("OutDataTypeStr", "")) == fc._INHERIT_RULE and
                   any(y in base for y in self.src[m]) for m in scc if len(scc) > 1)

    def _downstream_start_check(self, base, starts=None, keep=False):
        """Try every downstream start (the loop started from the type a downstream block gives it). A start whose
        signal's own rule does not reproduce the type it was started with is "bad": Simulink may reject the model
        (InvDisagreeInternalRuleDType: 18 of the 25 recordings ufix10_En1) or compile
        it (recordings 402 / 567, starting elsewhere). Consistent starts are kept with the typing they give.
        Returns the bad starts [(signal, type, own type or None, explicit)]; _downstream_decide judges them against
        the engine's own typing."""
        starts = self._downstream_starts(base) if starts is None else starts
        if not keep:
            self._downstream_typings = []
        bad = []
        for x, t_, expl in starts:
            self.otype = dict(base)
            self.otype[x] = t_
            try:
                self._propagate()
                self._seed_cycles([], [])
                got = self._out_type(x)
            except GraphError:
                got = None
            if got == t_:
                self._downstream_typings.append((x, t_, {n: self.otype.get(n) for n in self.names}, expl))
            else:
                bad.append((x, t_, got, expl))
        self.otype = dict(base)
        return bad

    def _late_starts(self, base, typing):
        """recordings (recordings a .. f): the typed side of a start may be
        the output of another inherited loop, typed only once that loop is solved -- Simulink starts the first loop
        from it as from a DTC (a / b / c / d / f compile with that type, e is rejected). For each loop (strongly
        connected component of blocks untyped in base), every block that does not depend on it takes the engine's
        typing, and the starts on that loop's signals are collected again."""
        fwd = {n: [m for m in self.names if n in self.src.get(m, ())] for n in self.names}

        def reach(start, nxt):
            seen, todo = {start}, [start]
            while todo:
                for m in nxt(todo.pop()):
                    if m not in seen:
                        seen.add(m)
                        todo.append(m)
            return seen
        free = [n for n in self.names if n not in base]
        done, out = set(), []
        for x in free:
            if x in done:
                continue
            down = reach(x, lambda n: fwd[n])
            scc = down & reach(x, lambda n: [m for m in self.src.get(n, ()) if m not in base])
            done |= scc
            if len(scc) < 2:
                continue
            ext = dict(base)
            ext.update({n: typing[n] for n in free if n not in down and typing.get(n) is not None})
            if len(ext) == len(base):
                continue
            # recordings (recordings a .. d): a start whose loop side reaches the
            # comparison through blocks outside the loop counts as one inside it does -- the same terms as the early
            # starts (any signal that depends on the loop)
            out += [t for t in self._downstream_starts(ext) if t[0] in down]
        return list(dict.fromkeys(out))

    def _late_bad(self, base, typing):
        """The late starts (_late_starts) not already among the early ones, checked like them (appended to the kept
        consistent typings); returns their bad starts."""
        early = set(self._downstream_starts(base))
        late = [t for t in self._late_starts(base, typing) if t not in early]
        if not late:
            return []
        return self._downstream_start_check(base, late, keep=True)

    def _counts(self, x, t_, expl, typing):
        """Whether a downstream start applies. An explicit output type applies when it is wider than the loop signal's
        own type: recordings -- 10 .. 32-bit outputs start a ufix8_En1 loop, int8 / uint8 do not;
        recordings , b5loop2-03 -- a 10-bit output narrower than the loop compiles with the
        loop's own types. Where the loop signal comes from makes no exemption (refuse what the
        recordings do not cover): recordings 416 / 462 / 578, wider outputs fed by a Unit Delay, Unary Minus, Abs
        or a 'Same as input' Gain that Simulink compiles with the loop's own types, are refused."""
        if not expl:
            return True
        own = typing.get(x)
        if not (isinstance(own, tuple) and isinstance(t_, tuple)):
            return True
        if expl == "any":
            return t_ != own
        return t_[1] > own[1] or (expl == "differs" and t_[1] == own[1] and t_ != own)

    def _downstream_decide(self, bad, typing):
        """The engine would accept the loop with typing: refuse it (UNSUPPORTED) when a downstream start that applies
        is inconsistent (never accepted on the engine's own start; a rejection stays a rejection) or gives other
        types (_downstream_agree)."""
        cic = getattr(self, "_cic_on_loop", None)
        if cic:
            raise GraphError("UNSUPPORTED", "design.blocks." + cic[0], "inherited data types around feedback loops: "
                             "a CIC filter with a specified output type fed by the loop -- whether it starts the loop "
                             "is not recorded; give the loop an explicit type")
        for c in getattr(self, "_mrfilter_on_loop", None) or ():
            if not _mrfilter_recorded(self.btype[c], self.params[c], [typing.get(x) for x in self.src[c]]):
                raise GraphError("UNSUPPORTED", "design.blocks." + c, "inherited data types around feedback loops: a "
                                 '%s fed by the loop in a configuration that is not recorded ('
                                 "four configurations on an sfix8_En1 loop) -- whether its output "
                                 "starts the loop is not known; give the loop an explicit type"
                                 % self.btype[c])
        bad = [b for b in bad if self._counts(b[0], b[1], b[3], typing)]
        if bad:
            x, t_, got, _e = bad[0]
            raise GraphError("UNSUPPORTED", "design.blocks." + x, "inherited data types around feedback loops: "
                             "Simulink may start this loop from a downstream block (started as %s, %s's own rule "
                             "gives %s) and whether it does is not recorded; give the loop an explicit type "
                             "" % (_tstr(t_), x, _tstr(got) if got else "no type"))
        self._downstream_agree(typing)

    def _downstream_agree(self, typing):
        """recordings (Simulink recordings: InvDisagreeInternalRuleDType): every downstream start may be
        consistent and still give the loop a different type than the engine's own start -- which one Simulink takes
        is not recorded, so a design whose typing depends on it is refused rather than given the engine's choice."""
        for x, t_, other, expl in getattr(self, "_downstream_typings", []):
            if not self._counts(x, t_, expl, typing):
                continue
            if any(other.get(n) != typing.get(n) for n in self.names):
                raise GraphError("UNSUPPORTED", "design.blocks." + x, "inherited data types around feedback loops: "
                                 "started from a downstream block (as %s) the loop gets "
                                 "other types than from its own blocks; which start Simulink takes is not recorded; "
                                 "give the loop an explicit type" % _tstr(t_))

    def _feeds(self, a, b):
        """Whether a signal path leads from block a to block b."""
        users = getattr(self, "_users_cache", None)
        if users is None:
            users = {n: [] for n in self.names}
            for n in self.names:
                for s_ in self.src[n]:
                    users[s_].append(n)
            self._users_cache = users
        seen, todo = set(), [a]
        while todo:
            n = todo.pop()
            for u in users.get(n, ()):
                if u == b:
                    return True
                if u not in seen:
                    seen.add(u)
                    todo.append(u)
        return False

    def _seed_cycles(self, choices, amb):
        """One resolution of the remaining cycles: at the k-th point where several blocks could be seeded, seed
        choices[k] (0 beyond the list; the number of candidates is appended to amb for those)."""
        seeded, assumed = [], []
        point = 0
        for _ in range(len(self.names) + 1):
            missing = [n for n in self.names if n not in self.otype]
            if not missing:
                break
            # the enable input of an internal Hold/Gate (multirate lowering) never carries the data type: such a
            # block is not a seed (a lowered slow-domain Delay in a loop was
            # seeded with the boolean enable's type and the loop became boolean)
            # the control port of a Switch / Multiport Switch never gives the data type -- a loop
            # with no typed DATA input was typed from the control (Simulink: double), and a hold register whose
            # control type differs from the data type was refused with a false TYPE_MISMATCH
            def data_srcs(n):
                if self.btype[n] == "Switch":
                    return [x for k, x in enumerate(self.src[n]) if k != 1]
                if self.btype[n] == "MultiportSwitch":
                    return list(self.src[n][1:])
                return list(self.src[n])
            cand = [n for n in missing if self.src[n] and self.btype[n] not in INTERNAL_TYPES
                    and any(x in self.otype for x in data_srcs(n))]
            if not cand:
                break
            if not seeded:
                self._seed_cands = list(cand)
            # a choice is (block, assumed loop-back type): the type of one of its typed inputs. With one block and
            # one distinct input type there is nothing to choose (the directed-loop recordings: one typed input per
            # head); otherwise every choice is tried by _resolve_types (recordings 487: a three-input Sum
            # "Same as first input", loop-back on port 1, typed inputs of two types -- the first typed one was not
            # Simulink's). recordings
            opts = []
            for c in cand:
                for t_ in dict.fromkeys(self.otype[x] for x in data_srcs(c) if x in self.otype):
                    opts.append((c, t_))
            # Simulink may also start the loop at a block whose
            # output type is explicit -- its untyped loop-back source takes the type of that block's typed input
            # (a Unit Delay feeding an explicit Product took the Product's other input's type, and the loop did not
            # close). Offered as a choice too, so a loop whose result depends on it is refused, not typed one way
            if os.environ.get("FXGRAPH_SEED_EXPLICIT", "1") == "1":
                for e in self.names:
                    if e in self.otype and self.btype[e] not in INTERNAL_TYPES and self.btype[e] != "Inport":
                        ds = data_srcs(e)
                        unt = [x for x in ds if x not in self.otype and x in missing]
                        if unt and self.btype[e] in ("Product", "Sum", "MinMax", "Switch", "MultiportSwitch"):
                            for t_ in dict.fromkeys(self.otype[x] for x in ds if x in self.otype):
                                for x in unt:
                                    # only a block on the loop itself (its output comes back to the delay);
                                    # an explicit block merely fed by the
                                    # loop does not start it in Simulink
                                    if self.btype[x] in DELAYS and self._feeds(e, x):
                                        opts.append((("direct", x), t_))
            pick = 0
            if len(opts) > 1:
                if point < len(choices):
                    pick = choices[point]
                else:
                    amb.append(len(opts))
                point += 1
            n, known = opts[pick]
            if isinstance(n, tuple):   # a delay typed directly from the explicit block it feeds (see above)
                self._seed_direct = True
                x = n[1]
                self.otype[x] = known
                seeded.append(x)
                assumed.append((x, x, known))
                self._propagate()
                continue
            tmp = sorted(set(x for x in data_srcs(n) if x not in self.otype))  # a source may feed several ports
            if any(x not in self.otype for x in self.src[n] if x not in tmp):
                break   # an untyped control: the block cannot be seeded yet
            nloop = sum(1 for x in data_srcs(n) if x not in self.otype)
            if nloop > 1 and not (nloop == 2 and (self.btype[n] == "MinMax" or (
                    self.btype[n] == "Sum" and str(self.params[n].get("AccumDataTypeStr", fc._INHERIT_RULE)).strip()
                    == fc._INHERIT_RULE))):
                # Only the supported two-port feedback shapes may be seeded here.
                e_ = GraphError("UNSUPPORTED", "design.blocks." + n, "a feedback loop enters %s %s on %d ports; how "
                                "Simulink types such a loop is not recorded: give the loop's delay or %s an explicit "
                                "type" % (self.btype[n], n, nloop, n))
                e_.multi_port_loop = True   # _resolve_types: another start's error comes first (name independence)
                raise e_
            for x in tmp:
                self.otype[x] = known
            try:
                t = self._out_type(n)
            finally:
                for x in tmp:
                    del self.otype[x]
            if t is None:
                break
            self.otype[n] = _chk(t, "design.blocks." + n)
            seeded.append(n)
            assumed.extend((n, x, known) for x in tmp)
            self._propagate()
        missing = [n for n in self.names if n not in self.otype]
        if missing:
            # a loop with no typed input of its own that a downstream
            # block can start (a Relational Operator's typed input) compiles in Simulink with that block's type --
            # not implemented, so UNSUPPORTED rather than a claim that the design cannot be typed
            try:
                startable = [t[0] for t in self._downstream_starts(dict(self.otype)) if t[0] in missing]
            except Exception:  # noqa: BLE001  (only refines the error code)
                startable = []
            if startable:
                raise GraphError("UNSUPPORTED", "design.blocks." + startable[0],
                                 "inherited data types around feedback loops: the loop through %s has no typed input "
                                 'of its own and Simulink may start it from a downstream block; not implemented'
                                 '' % ", ".join(sorted(set(startable))[:6]))
            raise GraphError("TYPE_UNRESOLVED", "design.blocks." + missing[0],
                             "data types could not be resolved (inheritance cycle?) for: %s" % ", ".join(missing[:10]))
        if seeded and CYCLE_MODE == "iterate":
            for _ in range(64):
                changed = False
                for n in self.names:
                    if self.btype[n] == "Inport":
                        continue
                    t = self._out_type(n)
                    if t is not None and t != self.otype[n]:
                        self.otype[n] = _chk(t, "design.blocks." + n)
                        changed = True
                if not changed:
                    break
            else:
                raise GraphError("TYPE_UNRESOLVED", "design.blocks." + seeded[0],
                                 "inherited types around a cycle do not settle to a fixed point")
        elif seeded:
            # Simulink-like (recordings error messages): the loop-back input of the seeded block was
            # assumed to have the type of its first typed input; the type that actually arrives there must be
            # exactly that, otherwise the cycle is inconsistent (OutputPortDataTypeMismatch /
            # InputPortDataTypeMismatch: "input port 2 needs sfix30_En11 but is driven by sfix32_En11")
            for n, x, want in assumed:
                if self.otype[x] != want and self.btype.get(n) in ("Switch", "MultiportSwitch") and \
                        str(self.params[n].get("InputSameDT", INPUT_SAME_DT_DEFAULT[self.btype[n]])).strip() == "off" \
                        and x in [y for k_, y in enumerate(self.src[n])
                                  if k_ != (1 if self.btype[n] == "Switch" else 0)]:
                    # a Switch with differing data types
                    # is legal (InputSameDT off) -- Simulink compiled such a loop (N1 data sfix32_En18 / sfix9_En9 ->
                    # sfix9_En9); which data port's type it takes around a loop is not recorded, so this is a
                    # refusal, not the "Simulink rejects" the inconsistency below claims
                    raise GraphError("UNSUPPORTED", "design.blocks.%s" % n, "inherited data types around a feedback "
                                     "loop through %s %s, whose data inputs differ (%s from %s, the loop assumed %s): "
                                     "Simulink accepts differing data types here, but which one the loop takes is not "
                                     "recorded; give the loop's delay or %s an explicit type"
                                     % (self.btype[n], n, _tstr(self.otype[x]), x, _tstr(want), n))
                if self.otype[x] != want and getattr(self, "hdl_info", None):
                    import block_lowering
                    ss = block_lowering.declared_on_cycle(self.src, self.hdl_info)
                    if ss:
                        # an explicitly typed Signal Specification on the loop is
                        # where Simulink takes the loop's type from (it compiled the model); the engine reads the
                        # block as an identity, so this inconsistency is not a Simulink rejection but a refusal
                        raise GraphError("UNSUPPORTED", "design.blocks.%s" % ss, "a feedback loop whose inherited "
                                         "types are set by the explicitly typed Signal Specification %s: Simulink "
                                         "takes the loop's type from it, which is not recorded; give the loop's delay "
                                         "or %s an explicit type, or put a Data Type Conversion in place of %s"
                                         % (ss, n, ss))
                if self.otype[x] != want:
                    raise GraphError("TYPE_MISMATCH", "design.blocks.%s" % n,
                                     "inherited data types around a cycle are inconsistent: an input of %s was "
                                     "resolved as %s but is driven by %s from %s" % (n, _tstr(want),
                                                                                    _tstr(self.otype[x]), x))

    def _in_types(self, n):
        ts = [self.otype.get(s) for s in self.src[n]]
        return None if any(t is None for t in ts) else ts

    def _no_bool(self, n, ts):
        for k, t in enumerate(ts):
            if t == BOOL:
                raise GraphError("UNSUPPORTED", "design.blocks.%s" % n,
                                 "input %d is boolean; boolean into arithmetic blocks is not validated yet" % (k + 1))

    def _fx_route(self, n, ts):
        """(kind, name) when block n is evaluated through ref/fxcore.py, else None.
        kind 'op': a recordings block op; kind 'bool': an arithmetic block with one boolean input."""
        bt, p = self.btype[n], self.params[n]
        if bt in ("Switch", "MultiportSwitch") and str(p.get("OutDataTypeStr", "")).startswith("Inherit") and \
                p.get("OutDataTypeStr") not in (fc._INHERIT_RULE, fc._SAME_FIRST):
            # back propagation / 'Inherit: auto' reached the evaluator as a None type (INTERNAL)
            raise _out_rule_error("design.blocks." + n, p["OutDataTypeStr"])
        if bt == "MultiportSwitch" and str(p.get("OutDataTypeStr", "")).strip() == "boolean":
            # Simulink refuses the value when the parameter is set
            # (SimulinkBlock:Foundation:UdtInvalidValue / UdtInvBuiltin); a boolean output comes from the internal rule
            raise GraphError("PORT_TYPE_INVALID", "design.blocks.%s.OutDataTypeStr" % n, "Multiport Switch does not "
                             "take 'boolean' as its output type (Simulink SimulinkBlock:Foundation:UdtInvBuiltin); "
                             "use 'Inherit: Inherit via internal rule' with boolean data ports")
        data_ts = [t for k, t in enumerate(ts or []) if not ((bt == "Switch" and k == 1) or
                                                            (bt == "MultiportSwitch" and k == 0))]
        if bt == "Switch" and str(p.get("OutDataTypeStr", "")).strip() == "boolean" and \
                not all(t == BOOL for t in data_ts):
            # was BAD_PARAM "block evaluation failed: too many values to unpack": a boolean output is recorded for
            # all-boolean data only
            raise GraphError("UNSUPPORTED", "design.blocks.%s.OutDataTypeStr" % n, "Switch output type 'boolean' with "
                             "fixed-point data is not recorded (only all-boolean data); use 'Inherit: Inherit via "
                             "internal rule' or compare the output with Compare To Constant ~= 0")
        if bt in ("Switch", "MultiportSwitch") and any(t == BOOL for t in data_ts):
            # all-boolean data ports with the internal rule (Switch also an explicit
            # 'boolean') give a boolean output, the selected value -- answered; the rest is refused
            # recordings (70 designs): boolean mixed with fixed point
            # (two data ports, fixed-point data up to 32 bits) under the internal rule or, for Switch, 'Same as first
            # input'; all-boolean data also under 'Same as first input' and with a threshold criterion -- answered
            # (ref/fxcore.py _bool_select); equal ranges of the boolean (as uint8) and a fixed-point type refused
            why = None
            ot_ = p.get("OutDataTypeStr")
            if not all(t == BOOL for t in data_ts):
                fx_ = [t for t in data_ts if t != BOOL]
                keys = [(t[1] - t[2] - t[0], t[2]) for t in [(0, 8, 0) if t == BOOL else t for t in data_ts]]
                if len(data_ts) != 2:
                    why = "boolean and fixed-point data ports mixed over %d data ports (recorded with two)" % len(data_ts)
                elif any(t[1] > 32 for t in fx_):
                    why = "boolean mixed with fixed-point data wider than 32 bits (not recorded)"
                elif ot_ not in ((fc._INHERIT_RULE, fc._SAME_FIRST) if bt == "Switch" else (fc._INHERIT_RULE,)):
                    why = "boolean and fixed-point data ports mixed into the output type %s (not recorded)" % ot_
                elif ot_ == fc._INHERIT_RULE and keys.count(max(keys)) > 1 and \
                        not (bt == "Switch" and data_ts[0] == BOOL and data_ts[1] != BOOL):
                    why = "boolean data mixed with %s, whose range equals the boolean's (counted as uint8) in the " \
                          "internal rule (not recorded)" % _tstr(fx_[0])
            elif ot_ not in ((fc._INHERIT_RULE, "boolean", fc._SAME_FIRST) if bt == "Switch" else (fc._INHERIT_RULE,)):
                why = "boolean data ports into the output type %s (not recorded)" % ot_
            if why:
                raise GraphError("UNSUPPORTED", "design.blocks." + n, "%s with %s: convert the data to uint8 with a "
                                 "Data Type Conversion before the switch and compare the result with Compare To "
                                 "Constant ~= 0" % (bt, why))
        # recordings (needless refusal list) / recordings (8 / 8): an MSB-anchored field wider
        # than the input word extends below bit 0 (the input shifted up); answered in ref/fxcore.py _blk_extract
        if bt == "Product" and str(p.get("Multiplication", "")).startswith("Matrix") and "/" in str(p.get("Inputs", "")):
            # a matrix inverse needs all-double or
            # all-single types (Simulink:blocks:MatMultDivOnlyDblAndSgl); engine signals are fixed point
            raise GraphError("PORT_TYPE_INVALID", "design.blocks.%s.Multiplication" % n, "Product: matrix division "
                             "(Multiplication %s with '/' in Inputs) needs every type double or single "
                             "(Simulink:blocks:MatMultDivOnlyDblAndSgl)" % p.get("Multiplication"))
        if bt == "Saturation":
            try:
                lo_, hi_ = float(str(fxnum.canonical(p["LowerLimit"], "scalar"))), \
                    float(str(fxnum.canonical(p["UpperLimit"], "scalar")))
            except (KeyError, TypeError, ValueError):
                lo_ = hi_ = None
            if lo_ is not None and lo_ > hi_:
                # Simulink refuses the parameters; the engine gave the upper limit (and RTL INTERNAL)
                raise GraphError("BAD_PARAM", "design.blocks.%s.LowerLimit" % n, "Saturation: LowerLimit %s is above "
                                 "UpperLimit %s (Simulink refuses these parameters)" % (p["LowerLimit"], p["UpperLimit"]))
        if bt in ("DetectChange", "DetectIncrease", "DetectDecrease", "Signum", "CompareToConstant",
                  "BitwiseOperator", "DiscreteIntegrator", "ShiftArithmetic", "ArithShift",
                  "Rounding", "DiscreteFir", "DeadZone", "WrapToZero", "RateLimiter") and ts and ts[0] == BOOL:
            # recordings (d) and recordings (E1, Signum / Compare To Constant / Bitwise Operator / Discrete-Time
            # Integrator / Shift / Rounding / FIR): Simulink accepts boolean data here; not recorded -- a clean
            # refusal instead of the evaluator's BAD_PARAM "BAD_TYPE: integer required, got None"
            raise GraphError("UNSUPPORTED", "design.blocks." + n, "boolean input to %s (not recorded): convert it "
                             "with a Data Type Conversion to uint8 first" % bt)
        if bt in FXCORE_OPS:
            self._check_validated(n, ts)
            return ("op", FXCORE_OPS[bt])
        nb = sum(1 for t in ts if t == BOOL)
        if nb and bt in ("Gain", "Sum", "Product", "DataTypeConversion"):
            if nb > 1 and not (bt == "Sum" and len(ts) == 3):
                raise GraphError("UNSUPPORTED", "design.blocks." + n, "more than one boolean input")
            # the same type forms as the native kernels accept (the boolean route took any string)
            where = "design.blocks." + n
            # only the recorded boolean shapes are answered -- host, Gain / Data
            # Type Conversion of the boolean, Sum / Product of the boolean and one fixed-point operand
            #; the rest gave wrong types or values
            if bt == "Product" and len(ts) == 1 and str(p.get("Inputs", "")).strip() in ("1", "*"):
                raise GraphError("PORT_TYPE_INVALID", where, "a one-input Product of a boolean "
                                 "(Simulink:DataType:InvFixptBoolOutPortDType)")
            why = None
            ir_gain = bt == "Gain" and str(p.get("OutDataTypeStr", fc._INHERIT_RULE)).strip() == fc._INHERIT_RULE \
                and fc._named_type(p.get("ParamDataTypeStr", fc._INHERIT_RULE)) is None
            try:
                g1 = ir_gain and float(str(fxnum.canonical(p.get("Gain", "1"), "scalar"))) == 1
            except (TypeError, ValueError):
                g1 = False
            if fc._asic(self.hw) and ir_gain and g1:
                # on ASIC/FPGA gain 1 alone gave parameter ufix1 but output ufix9 (every
                # other gain: output = parameter type); one recording, no rule -- refused
                why = "with gain 1 and the internal rules on ASIC/FPGA (Simulink's output type there, ufix9, follows no " \
                      "recorded rule; write the output type)"
            elif fc._asic(self.hw) and not ir_gain and (bt != "DataTypeConversion" and
                                                          str(p.get("OutDataTypeStr", fc._INHERIT_RULE)).strip().startswith("Inherit")) \
                    and not (bt in ("Sum", "Product") and len(ts) == 2 and
                             str(p.get("OutDataTypeStr", fc._INHERIT_RULE)).strip() == fc._INHERIT_RULE and
                             str(p.get("AccumDataTypeStr", fc._INHERIT_RULE)).strip() == fc._INHERIT_RULE and
                             str(p.get("Inputs", "")).replace("|", "").strip() in
                             (("++", "+-") if bt == "Sum" else ("**",))):
                # recordings (60 / 60): a two-operand Sum ('++' / '+-') or Product ('**') of the
                # boolean and a fixed-point operand with the internal rule is answered on ASIC/FPGA too
                # (ref/fxcore.py _arith_with_bool)
                # the inherited output rule differs on ASIC (all 6 recorded Gain types); an explicit output
                # type (and a conversion, always explicit) does not depend on it (recordings designs)
                why = "with an inherited output type on ASIC/FPGA (the host's rule gives other types there; write " \
                      "the output type)"
            elif bt == "Sum" and len(ts) == 3 and ts[0] == BOOL and \
                    str(p.get("AccumDataTypeStr", fc._INHERIT_RULE)).strip() == "Inherit: Same as first input":
                # Simulink cannot take the accumulator type from a
                # boolean first operand (Simulink:DataType:PropForwardDataTypeError)
                raise GraphError("TYPE_MISMATCH", where + ".AccumDataTypeStr", "AccumDataTypeStr 'Inherit: Same as "
                                 "first input' with a boolean first operand: Simulink cannot propagate the boolean type "
                                 "to the accumulator and rejects the model (PropForwardDataTypeError)")
            elif bt == "Sum" and len(ts) == 3 and _sumbool3_in_range(ts, p, fc._asic(self.hw)):
                # recorded item d: over the ranges recordings spans (engine/sumbool3_domain.json,
                # the regression check --check; recordings too): the boolean counts as uint8 0 / 1 in the Sum's own rule
                # (ref/fxcore.py _arith_with_bool; the ufix1 alternative differs on recordings)
                pass
            elif bt == "Sum" and len(ts) == 3 and sum(t == BOOL for t in ts) > 1:
                why = "with %d boolean operands (the rule here takes one boolean operand)" % sum(t == BOOL for t in ts)
            elif bt == "Sum" and len(ts) == 3:
                why = "with 3 operands outside the ranges recorded for a three-operand Sum with a boolean operand " \
                      "(engine/sumbool3_domain.json)"
            elif bt in ("Sum", "Product") and len(ts) != 2:
                why = "with %d operands (recorded with two: the boolean and one fixed-point operand)" % len(ts)
            elif bt == "Product" and any(t != BOOL and tuple(t) == fc.UFIX1 for t in ts):
                why = "with a ufix1 operand (the one-bit factor rule meets the boolean; not recorded)"
            elif bt == "Gain":
                try:
                    zero = float(str(fxnum.canonical(p.get("Gain", "1"), "scalar"))) == 0
                except (TypeError, ValueError):
                    zero = False
                if zero:
                    why = "with gain 0 (not recorded)"
            if why:
                raise GraphError("UNSUPPORTED", where, "boolean input to %s %s: convert the boolean with a Data Type "
                                 "Conversion (e.g. to uint8) first" % (bt, why))
            if bt == "Sum" and _fixdt(p["AccumDataTypeStr"], where + ".AccumDataTypeStr") is None and \
                    p["AccumDataTypeStr"] not in (fc._SAME_FIRST, fc._INHERIT_RULE):
                raise _type_text_error(where + ".AccumDataTypeStr", p["AccumDataTypeStr"])
            if bt == "Gain" and _fixdt(p["ParamDataTypeStr"], where + ".ParamDataTypeStr") is None and \
                    p["ParamDataTypeStr"] != fc._INHERIT_RULE:
                raise _type_text_error(where + ".ParamDataTypeStr", p["ParamDataTypeStr"])
            ot_b = str(p.get("OutDataTypeStr", fc._INHERIT_RULE)).strip()
            if bt in ("Sum", "Product") and ot_b.startswith("Inherit") and ot_b not in (
                    fc._INHERIT_RULE, fc._SAME_FIRST, "Inherit: Inherit via back propagation"):
                # (Keep MSB / Keep LSB / Match scaling are refused here too: the fixed-point route never takes them
                # -- recorded version let the boolean route answer them)
                # the native minimal counterexample: a Sum with 'Inherit: Same
                # as input' and a boolean input was answered (Simulink: UdtInvInherit, the rule is not one of the
                # block's) -- the boolean route had skipped the output-rule check the fixed-point route makes
                raise _type_text_error(where + ".OutDataTypeStr", ot_b)
            if bt == "Gain" and ot_b.startswith("Inherit") and ot_b not in (fc._INHERIT_RULE, fc._SAME_INPUT):
                # (the same class on a Gain: any other rule crashed INTERNAL here; refused as the fixed-point route does)
                raise _out_rule_error(where, ot_b) if ot_b == "Inherit: Inherit via back propagation" else \
                    _type_text_error(where + ".OutDataTypeStr", ot_b)
            if bt == "Gain" and p.get("OutDataTypeStr") == fc._SAME_INPUT:
                # crashed the evaluator ("cannot unpack NoneType"); the boolean-input Gain is
                # recorded with the internal rule and explicit outputs only
                raise GraphError("UNSUPPORTED", where + ".OutDataTypeStr", "boolean input to a Gain with 'Inherit: "
                                 "Same as input' (not recorded): use the internal rule or an explicit output type")
            if ts and ts[0] == BOOL and bt in ("Sum", "Product") and \
                    (p.get("OutDataTypeStr") == fc._SAME_FIRST or (bt == "Sum" and p["AccumDataTypeStr"] == fc._SAME_FIRST)):
                # recordings (108 records): a boolean first
                # input with 'Inherit: Same as first input' for the output or the accumulator is a Simulink error
                # (InvFixptBoolOutPortDType / InvFixptBoolAccumDType / SumBlockOutputDataTypeIsBool)
                k = "AccumDataTypeStr" if p.get("OutDataTypeStr") != fc._SAME_FIRST else "OutDataTypeStr"
                raise GraphError("PORT_TYPE_INVALID", where + "." + k, "a boolean first input with %s 'Inherit: Same "
                                 "as first input' (Simulink:DataType:InvFixptBool%sDType)"
                                 % (k, "OutPort" if k == "OutDataTypeStr" else "Accum"))
            return ("bool", {"DataTypeConversion": "DTC"}.get(bt, bt))
        if bt == "Product" and str(p["Inputs"]).strip() not in ("2", "**"):
            if str(p.get("OutDataTypeStr", "")).startswith("Inherit") and \
                    p.get("OutDataTypeStr") not in (fc._INHERIT_RULE, fc._SAME_FIRST):
                # back propagation (or another inherit rule) reached the evaluator as a None type and crashed it
                # (the regression check seed 1 graph 269, once TypeErrors were reported as INTERNAL)
                raise _out_rule_error("design.blocks." + n, p["OutDataTypeStr"])
            return ("op", "blk_productn")
        return None

    def _lutport_check(self, n, ts, recs, step=None):
        """a 1-D Lookup Table with its table / breakpoints from ports is checked as the dialog block it is at
        each step -- the same checks, on the step's values typed with the ports' types (recs None: placeholder values,
        for the compile-time checks); plus the recorded support domain (engine/lutport_domain.json)."""
        p0 = self.params[n]
        where = "design.blocks." + n
        two = self.btype[n] == "Lookup2D"
        nu = 2 if two else 1
        if any(t == BOOL or not isinstance(t, (tuple, list)) for t in ts):
            raise GraphError("UNSUPPORTED", where, "Lookup Table with its table / breakpoints from ports: a "
                             "boolean input is not recorded")
        lp = lut_ports(p0)
        tps_, at_ = [], nu
        for c in ("T", "B", "C"):
            if c in lp:
                if len({tuple(t) for t in ts[at_:at_ + lp[c]]}) != 1:
                    raise GraphError("UNSUPPORTED", where, "Lookup Table with its %s from a port: elements of "
                                     "different types are not recorded" % {"T": "table", "B": "breakpoints",
                                                                            "C": "breakpoints"}[c])
                tps_.append(list(ts[at_]))
                at_ += lp[c]
        # 2-D (recordings 2-D designs): the sources carry a '2D' marker, the inputs both types, the
        # number of points m x n
        key_ = json.dumps([("2D " if two else "") + " ".join(c for c in ("T", "B", "C") if c in lp),
                           [list(t) for t in ts[:nu]] if two else list(ts[0]), tps_,
                           str(p0.get("TableDataTypeStr", "Inherit: Same as output")).strip(),
                           str(p0.get("OutDataTypeStr", fc._SAME_FIRST)).strip(),
                           str(p0.get("InterpMethod", "Linear point-slope")), str(p0.get("ExtrapMethod", "Clip")),
                           str(p0.get("InternalRulePriority", "Speed")), bool(fc._asic(self.hw)),
                           "%dx%d" % lut_dims(p0) if two else max(lp.values())])
        q = _lut_port_values(p0, ts, recs, nu)
        self.params[n] = q
        try:
            self._check_validated(n, list(ts[:nu]))
        except GraphError as e:
            if step is None:
                raise
            raise GraphError(e.code, e.where, "at step %d (table / breakpoints from the ports): %s" % (step, e.message))
        finally:
            self.params[n] = p0
        if key_ not in _lutport_domain():
            raise GraphError("UNSUPPORTED", where, "Lookup Table with its table / breakpoints from ports: this "
                             "setting (sources, input / port types, table and output types, interpolation, extrapolation, "
                             "internal rule priority, hardware, number of points) is not one of the recorded ones "
                             "(engine/lutport_domain.json); record a design of this shape to open it")
        return q

    def _delayen_check(self, n, p, where):
        """The Delay with an enable port and / or an external reset, dialog length and initial condition
        (engine/delayen_domain.json, the regression check): answered per recorded (enable port, ExternalReset)
        combination on its recorded lengths (a span where the lengths were swept), enable / reset port types and
        hardware; data fixdt 8..32 bits (fraction 0..wl); outside feedback loops. ref/fxcore.py _blk_delayport has the
        rule."""
        dom = _delayen_domain()
        ts_ = [self.otype.get(s_) for s_ in self.src[n]]
        enab = str(p.get("ShowEnablePort", "off")).strip() == "on"
        mode = str(p.get("ExternalReset", "None")).strip()
        mode = "None" if mode == "none" else mode
        def tn(t):
            return "boolean" if t == BOOL else list(t) if isinstance(t, tuple) else t
        def ts(t):
            return _tstr(t) if isinstance(t, tuple) else str(t)
        cb = dom.get("combos", {}).get("%s|%s" % ("on" if enab else "off", mode))
        why = None
        if str(p.get("DelayLengthSource", "Dialog")) != "Dialog" or \
                str(p.get("InitialConditionSource", "Dialog")) != "Dialog":
            why = "the length or initial condition from a port as well"
        elif cb is None:
            why = "ShowEnablePort %s with ExternalReset %s" % ("on" if enab else "off", mode)
        elif self._on_cycle(n):
            why = "a feedback loop through it"
        elif not (isinstance(ts_[0], tuple) and dom["data_wl"][0] <= ts_[0][1] <= dom["data_wl"][1] and
                  0 <= ts_[0][2] <= ts_[0][1]):
            why = "data %s" % ts(ts_[0])
        elif enab and tn(ts_[1]) not in cb["enable_types"]:
            why = "an enable port of type %s" % ts(ts_[1])
        elif mode != "None" and tn(ts_[2 if enab else 1]) not in cb["reset_types"]:
            why = "a reset port of type %s" % ts(ts_[2 if enab else 1])
        elif bool(fc._asic(self.hw)) not in cb["asic"]:
            why = "this hardware"
        else:
            try:
                L_ = int(str(p.get("DelayLength", "2")).strip())
            except ValueError:
                L_ = None
            sp_ = cb.get("span")
            if L_ is None or not (L_ in cb["lengths"] or (sp_ and sp_[0] <= L_ <= sp_[1])):
                why = "DelayLength %s" % p.get("DelayLength", "2")
        if why:
            raise GraphError("UNSUPPORTED", where, "Delay %s with an enable / external reset port: %s is not recorded "
                             "(engine/delayen_domain.json lists the recorded combinations, lengths and port types)"
                             % (n, why))

    def _check_validated(self, n, ts):
        """Refuse configurations whose rules are not bit-exact against recordings yet."""
        bt, p = self.btype[n], self.params[n]
        where = "design.blocks." + n
        if bt in ("Lookup1D", "Lookup2D") and lut_ports(p):
            self._lutport_check(n, ts, None)
            return
        if bt in ("Lookup1D", "Lookup2D") and any(str(p.get(k, "Dialog")).strip() == "Input port"
                                                  for k in ("TableSource", "BreakpointsForDimension1Source",
                                                            "BreakpointsForDimension2Source")):
            raise GraphError("UNSUPPORTED", where, "1-D Lookup Table with its table / breakpoints from ports: give them "
                             "as vector signals (an Inport with PortDimensions n); a scalar port is not recorded")
        if bt == "Rounding" and ts and ts[0] != BOOL:
            # recordings (107 Rounding Function records with integer and
            # fixed-point inputs, all "Expected data types for the port must be double or single"; no recording
            # accepts one) -- the earlier "integer inputs are accepted" was wrong
            raise GraphError("PORT_TYPE_INVALID", where, "Rounding Function: Simulink R2026b takes double or single "
                             "inputs only, got %s (Simulink PortValidationInputError)" % _tstr(ts[0]))
        if bt == "Sqrt":
            self._sqrt_check(n, ts)
        if bt == "Trigonometry":
            self._trig_check(n, ts)
        if bt == "LookupND":
            self._lutnd_check(n, ts)
        if bt in ("Prelookup", "PrelookupF"):
            self._prelookup_check(n, ts)
        if bt == "InterpolationUsingPrelookup":
            self._interp_pl_check(n, ts)
        if bt == "DiscreteFir":
            # recordings (native: StateDataTypeStr 'double' rejected at set_param, UdtInvBuiltin): the built-in
            # floating / boolean names are not accepted for the FIR's internal types either
            for k_ in ("StateDataTypeStr", "ProductDataTypeStr", "AccumDataTypeStr", "CoefDataTypeStr",
                       "TapSumDataTypeStr"):
                if str(p.get(k_, "")).strip() in ("double", "single", "half", "boolean"):
                    raise GraphError("PORT_TYPE_INVALID", where + "." + k_, "Discrete FIR Filter %s %s: Simulink "
                                     "rejects it (UdtInvBuiltin); use an Inherit rule or a fixdt type" % (k_, p[k_]))
        if bt == "DiscreteFir" and str(p.get("OutDataTypeStr", "")).strip() in ("double", "single", "half", "boolean"):
            # Simulink rejects these output types for the FIR block (SimulinkBlock:Foundation:
            # UdtInvalidValue); the engine answered as if 'Inherit: Same as input'
            raise GraphError("PORT_TYPE_INVALID", where + ".OutDataTypeStr", "Discrete FIR Filter output type %s: "
                             "Simulink rejects it (UdtInvalidValue); use an Inherit rule or a fixdt type"
                             % p["OutDataTypeStr"])
        if bt in NL_PARAMS:
            # fixed-point / integer inputs only; Dead Zone saturates (its default, the
            # only value recorded); Rate Limiter in its inherited sample-time mode
            if ts and not (isinstance(ts[0], tuple) and len(ts[0]) == 3):
                raise GraphError("UNSUPPORTED", where, "%s with a %s input is not recorded (fixed-point and integer "
                                 "inputs are)" % (bt, ts[0]))
            for k_ in NL_PARAMS_ALL[bt]:
                try:
                    v_ = float(fc._const_value(p[k_])) if k_ in p else 0.0
                except (ValueError, KeyError, TypeError, OverflowError):
                    continue   # reported by the block's own evaluation
                if v_ != v_ or v_ in (float("inf"), float("-inf")):
                    raise GraphError("UNSUPPORTED", where + "." + k_, "%s %s %s is not finite: not recorded (Simulink "
                                     "quantises the parameter into the input type); write a finite value" % (bt, k_,
                                                                                                          p[k_]))
            if bt in ("DeadZone", "WrapToZero") and self._on_cycle(n):
                raise GraphError("UNSUPPORTED", where, "%s inside a feedback loop is not recorded (Rate Limiter loops "
                                 "are); move it out of the loop or ask for a recording" % bt)
            if ts and isinstance(ts[0], tuple) and ts[0][2] == ts[0][1] and 4 <= ts[0][1] <= 48:
                # fraction length = word length is recorded only where Simulink stops on a parameter overflow
                # Check for parameter overflow before the width refusal below.
                for k_ in NL_PARAMS[bt]:
                    v_, what_ = self._nl_param_value(n, k_)
                    _param_overflow(v_, ts[0], where + "." + k_, what_, mode=self.pom)
            if ts and isinstance(ts[0], tuple) and not (4 <= ts[0][1] <= 48 and -2 <= ts[0][2] <= ts[0][1] - 1):
                raise GraphError("UNSUPPORTED", where, "%s with input %s is not recorded: word lengths 4 to 48 bits "
                                 "and fraction lengths -2 to the word length - 1 are; convert the signal into such a type "
                                 "first (Data Type Conversion), or ask for a recording of this type"
                                 % (bt, _tstr(ts[0])))
            if bt == "DeadZone" and str(p.get("SaturateOnIntegerOverflow", "on")).strip() not in ("on", "off"):
                raise GraphError("BAD_PARAM", where + ".SaturateOnIntegerOverflow", "Dead Zone "
                                 "SaturateOnIntegerOverflow is 'on' or 'off'")
            if bt == "DeadZone" and ts and isinstance(ts[0], tuple):
                for k_ in NL_PARAMS[bt]:   # Simulink's parameter overflow comes first
                    v_, what_ = self._nl_param_value(n, k_)
                    _param_overflow(v_, ts[0], where + "." + k_, what_, mode=self.pom)
                # Compare the written bounds before quantization.
                q_ = [Fraction(fc._const_value(p[k_])) for k_ in NL_PARAMS[bt]]
                if q_[0] > q_[1]:
                    raise GraphError("UNSUPPORTED", where + ".LowerValue", "Dead Zone LowerValue %s above UpperValue "
                                     "%s in the input type %s is not recorded (only Start <= End is); write "
                                     "LowerValue <= UpperValue" % (p["LowerValue"], p["UpperValue"], _tstr(ts[0])))
            if bt == "RateLimiter":
                # recordings (native: Simulink rejects RisingSlewLimit -0.5 at set_param, ParamValiditySign):
                # the rising limit must be positive; a positive falling limit is not recorded
                try:
                    r_, f__ = (Fraction(fc._const_value(p[k_])) for k_ in ("RisingSlewLimit", "FallingSlewLimit"))
                except (ValueError, KeyError, TypeError, ZeroDivisionError):
                    r_ = f__ = None
                if r_ is not None and r_ < 0:
                    raise GraphError("BAD_PARAM", where + ".RisingSlewLimit", "Rate Limiter RisingSlewLimit %s: "
                                     "Simulink accepts positive values only (ParamValiditySign)" % p["RisingSlewLimit"])
                if f__ is not None and f__ > 0:
                    raise GraphError("UNSUPPORTED", where + ".FallingSlewLimit", "Rate Limiter FallingSlewLimit %s > 0 "
                                     "is not recorded; write a negative (or zero) falling limit" % p["FallingSlewLimit"])
            if bt == "RateLimiter" and ts and isinstance(ts[0], tuple):
                for k_ in NL_PARAMS[bt]:   # Simulink's parameter overflow comes first
                    v_, what_ = self._nl_param_value(n, k_)
                    _param_overflow(v_, ts[0], where + "." + k_, what_, mode=self.pom)
                # the falling limit is not held in the input type (-1 with a ufix12 input runs); its
                # magnitude within the input type's maximum (plus one LSB) and off a rounding tie is what was recorded
                s_, w_, f_ = ts[0]
                T_ = Fraction(str(p.get("_RlT", "1")))
                x_ = -Fraction(fc._const_value(p["FallingSlewLimit"])) * T_ * Fraction(2) ** f_
                hi_ = (1 << (w_ - s_)) - 1
                # fraction of an LSB: only below one half is recorded (-0.25 LSB: rounds to 0, which
                # nearest and toward-zero both give); half and above (where they part) are not
                if x_ > hi_ + 1 and T_ != 1:
                    # the Simulink check (falling -100 on int8 at period 2): Simulink stops with a parameter
                    # overflow of the "Weighted Fall Limit"
                    raise GraphError("PORT_TYPE_INVALID", where + ".FallingSlewLimit", "Rate Limiter %s: "
                                     "FallingSlewLimit %s times the sample period %s does not fit the input type %s "
                                     "(Simulink stops with fxpParameterOverflow on the Weighted Fall Limit under its "
                                     "default diagnostics); write a smaller limit or use a wider type"
                                     % (n, p["FallingSlewLimit"], _fmt_num(T_), _tstr(ts[0])))
                if x_ < 0 or x_ > hi_ + 1 or x_ - (x_.numerator // x_.denominator) >= Fraction(1, 2):
                    raise GraphError("UNSUPPORTED", where + ".FallingSlewLimit", "Rate Limiter FallingSlewLimit %s "
                                     "with input %s%s: only a non-positive limit whose magnitude (times the sample "
                                     "period) fits the input type's maximum and whose fraction of an LSB is below "
                                     "one half is recorded (Simulink does not hold the falling limit in the input "
                                     "type, and how it rounds half an LSB or more is not recorded); write a whole "
                                     "number of input LSBs" % (
                                         p["FallingSlewLimit"], _tstr(ts[0]),
                                         "" if T_ == 1 else " at sample period %s" % _fmt_num(T_)))
                if T_ != 1:
                    # at period 2 every recorded limit times the period is a whole number
                    # of input LSBs within the input type; whether Simulink rounds before or after the product (and
                    # what an overflowing product does) is not recorded
                    for k_ in ("RisingSlewLimit", "FallingSlewLimit"):
                        y_ = abs(Fraction(fc._const_value(p[k_]))) * T_ * Fraction(2) ** f_
                        if y_.denominator != 1 or y_ > hi_:
                            raise GraphError("UNSUPPORTED", where + "." + k_, "Rate Limiter %s %s at sample period %s "
                                             "with input %s: the limit times the period (%s LSB) must be a whole "
                                             "number of input LSBs within the input type's maximum %d LSB -- only "
                                             "that is recorded at a period other than 1; run at period 1 or choose "
                                             "a limit that is a multiple of %s" % (
                                                 k_, p[k_], _fmt_num(T_), _tstr(ts[0]), _fmt_num(y_), hi_,
                                                 _fmt_num(Fraction(2) ** -f_ / T_)))
            if bt == "RateLimiter" and str(p.get("SampleTimeMode", "inherited")) != "inherited":
                raise GraphError("UNSUPPORTED", where + ".SampleTimeMode", "Rate Limiter SampleTimeMode %r: only "
                                 "'inherited' is recorded" % p.get("SampleTimeMode"))
        if bt == "Signum" and ts and ts[0] != BOOL and not ts[0][0] and \
                (ts[0][2] == 0 and ts[0][1] in (8, 16, 32, 64) or ts[0][1] < 2):
            # recordings (65 designs): Simulink makes ufixN_Ek signed (sfixN,
            # the value 0 / 1) but keeps a built-in uintN; only fraction 0 with N in 8 / 16 / 32 / 64 is ambiguous
            # for engine types (and ufix1 would give an invalid sfix1) -- those stay refused
            raise GraphError("UNSUPPORTED", where, "Signum with input %s: Simulink keeps a built-in uintN type but "
                             "makes ufixN signed, and engine types do not tell the two apart here; give the "
                             "input a fraction length or a signed type" % _tstr(ts[0]))
        if bt == "MultiportSwitch" and p.get("OutDataTypeStr") == fc._SAME_FIRST:
            # recordings (3 records): Simulink refuses the rule for this block
            # (SimulinkBlock:Foundation:UdtInvInherit); the engine used to accept it (found by the recorded shared
            # 'rejected alike' rule -- the old check counted a wiring artefact as the refusal)
            raise GraphError("BAD_TYPE", where + ".OutDataTypeStr", "Multiport Switch: 'Inherit: Same as first "
                             "input' is not a supported inheritance rule for this block (Simulink refuses it)")
        if bt == "DiscreteIntegrator":
            # the parameter cross-check comes first, as in Simulink (set_param), before what the engine supports
            # (backward Euler / trapezoidal with such a value were refused as UNSUPPORTED) -- but only in
            # the recorded configuration (internal initial condition, no reset, internal-rule
            # output); with an external source / reset / explicit output type the order is unrecorded
            if p.get("LimitOutput") == "on" and p["InitialConditionSource"] == "internal" \
                    and p["ExternalReset"] == "none" and p["OutDataTypeStr"] == fc._INHERIT_RULE \
                    and str(p["IntegratorMethod"]).split(":")[0] in ("Integration", "Accumulation"):
                # recorded methods (Accumulation Trapezoidal / Integration Backward
                # Euler get the same ParameterCrossValidation error)
                # final: an initial condition outside the output limits -- whether the first output is
                # the clamped value or the initial condition itself -- has never been recorded; refused
                try:
                    ic, lo_, hi_ = (float(p["InitialCondition"]), float(p["LowerSaturationLimit"]),
                                    float(p["UpperSaturationLimit"]))
                except ValueError:
                    ic, lo_, hi_ = 0.0, 0.0, 0.0
                if ic < lo_ or ic > hi_:
                    # Simulink refuses the parameters themselves for every integration method
                    # (Simulink:dialog:ParameterCrossValidationTripleError, LowerSaturationLimit <= InitialCondition
                    # <= UpperSaturationLimit), before compiling
                    raise GraphError("BAD_PARAM", where + ".InitialCondition", "Discrete-Time Integrator: initial "
                                     "condition %r outside the output limits [%r, %r] (Simulink requires "
                                     "LowerSaturationLimit <= InitialCondition <= UpperSaturationLimit)" % (ic, lo_, hi_))
            ot_dti = str(p["OutDataTypeStr"]).strip()
            if ot_dti in ("double", "single", "half"):
                # recordings (silent wrong, the independent check): a floating-point output type was
                # answered by the internal rule; Simulink integrates in that type -- not simulated here
                raise GraphError("UNSUPPORTED", where + ".OutDataTypeStr", "Discrete-Time Integrator with a %s output "
                                 "type: floating-point signals are not simulated here; use the internal rule or a "
                                 "fixdt type" % ot_dti)
            if ot_dti == "boolean":
                raise GraphError("PORT_TYPE_INVALID", where + ".OutDataTypeStr", "Discrete-Time Integrator: boolean is "
                                 "not an output type Simulink allows (SimulinkBlock:Foundation:UdtInvalidValue)")
            meth_ = fc.dti_method(p)
            if ts and isinstance(ts[0], tuple) and (meth_ != "FE" or p["ExternalReset"] != "none" or
                                                    p["InitialConditionSource"] != "internal"):
                # these run on the Python path, which never reached the Forward Euler parameter-overflow
                # check of the initial condition and limits in the state type (Simulink fxpParameterOverflow)
                try:
                    _dti_ints(p, tuple(ts[0]), where, self.pom, getattr(self, "param_notes", None))
                except (ValueError, KeyError, TypeError):
                    pass   # malformed values are reported by the block's own evaluation
            if p["IntegratorMethod"] not in ("Integration: Forward Euler", "Accumulation: Forward Euler",
                                             "Integration: Backward Euler", "Accumulation: Backward Euler",
                                             "Integration: Trapezoidal", "Accumulation: Trapezoidal"):
                raise GraphError("BAD_PARAM", where + ".IntegratorMethod", "Integration / Accumulation: Forward "
                                 "Euler, Backward Euler or Trapezoidal")
            if p["ExternalReset"] != "none" or p["InitialConditionSource"] != "internal":
                # Forward Euler, no output limits, internal-rule output type,
                # outside feedback loops; reset port boolean / int8 / uint8; the external initial condition in the
                # output type (Simulink rejects another type: InputPortDataTypeMismatch)
                if p["ExternalReset"] not in ("none", "rising", "falling", "either", "level", "sampled level"):
                    raise GraphError("BAD_PARAM", where + ".ExternalReset", "none, rising, falling, either, level or "
                                     "sampled level")
                if p["InitialConditionSource"] not in ("internal", "external"):
                    raise GraphError("BAD_PARAM", where + ".InitialConditionSource", "internal or external")
                if meth_ != "FE" or p.get("LimitOutput") == "on" or \
                        isinstance(fc._named_type(p["OutDataTypeStr"]), tuple) or self._on_cycle(n):
                    raise GraphError("UNSUPPORTED", where, "Discrete-Time Integrator with an external reset or initial "
                                     "condition: Forward Euler, no output limits, the internal-rule output type and no "
                                     "feedback loop are recorded")
                k_ = 1
                if p["ExternalReset"] != "none":
                    rt_ = ts[k_] if len(ts) > k_ else None
                    if rt_ not in (BOOL, (1, 8, 0), (0, 8, 0)):
                        raise GraphError("UNSUPPORTED", where, "Discrete-Time Integrator reset input %s: boolean, int8 "
                                         "and uint8 are recorded" % (_tstr(rt_) if rt_ else rt_))
                    k_ += 1
                if p["InitialConditionSource"] == "external" and len(ts) > k_ and isinstance(ts[0], tuple):
                    s_, w_, f_, _sl = fc._dti_type(p, tuple(ts[0]))
                    if tuple(ts[k_]) != (s_, w_, f_) if isinstance(ts[k_], tuple) else True:
                        raise GraphError("PORT_TYPE_INVALID", where, "Discrete-Time Integrator %s: the external "
                                         "initial condition is %s but must be the output type %s (Simulink "
                                         "InputPortDataTypeMismatch); convert it with a Data Type Conversion"
                                         % (n, _tstr(ts[k_]) if ts[k_] != BOOL else "boolean", _tstr((s_, w_, f_))))
            ext_ = p["ExternalReset"] != "none" or p["InitialConditionSource"] != "internal"
            if (meth_ != "FE" or ext_) and ts and \
                    isinstance(ts[0], tuple):
                # recordings (accept only the recorded shapes): the input
                # types and gain values recorded for these methods / ports
                try:
                    gv_ = float(str(p.get("gainval", "1.0")).strip())
                except ValueError:
                    gv_ = None
                gs_ = (1.0, 0.5) if ext_ else (1.0, 0.5, 0.25, 2.0)
                # Backward Euler / Trapezoidal on sfix8_En4 with gains 0.1 .. 3 at
                # sample periods 0.25 .. 3
                t49_ = not ext_ and tuple(ts[0]) == (1, 8, 4) and gv_ in (0.1, 0.5, 1.0, 3.0)
                # bt (Backward Euler / Trapezoidal) on DTI_WIDEN_TYPES with gains 0.125 / 3 /
                # 1.5; rs: a boolean reset alone on sfix32_En16 / ufix8_En4 with gain 0.25, an external initial
                # condition alone on sfix24_En0 / int8 with gain 2 (Forward Euler)
                rst_, xic_ = p["ExternalReset"] != "none", p["InitialConditionSource"] != "internal"
                w6_ = (not ext_ and tuple(ts[0]) in DTI_WIDEN_TYPES and gv_ in (0.125, 3.0, 1.5)) or \
                    (rst_ and not xic_ and tuple(ts[0]) in ((1, 32, 16), (0, 8, 4)) and gv_ == 0.25 and
                     len(ts) > 1 and ts[1] == BOOL) or \
                    (xic_ and not rst_ and tuple(ts[0]) in ((1, 24, 0), (1, 8, 0)) and gv_ == 2.0)
                if not t49_ and not w6_ and (tuple(ts[0]) not in DTI_NEW_TYPES or gv_ not in gs_):
                    raise GraphError("UNSUPPORTED", where, "Discrete-Time Integrator %s%s: recorded with inputs %s and "
                                     "gain values %s; record a design with this input type and gain to open it"
                                     % (p["IntegratorMethod"], " with an external reset / initial condition" if ext_
                                        else "", ", ".join(_tstr(t) for t in DTI_NEW_TYPES),
                                        " / ".join("%g" % g for g in gs_)))
            if meth_ != "FE":
                # Backward Euler / Trapezoidal with the internal-rule output type,
                # outside feedback loops (they feed the input straight through; the recorded designs have no loop)
                if fc._named_type(p["OutDataTypeStr"]) is not None and \
                        isinstance(fc._named_type(p["OutDataTypeStr"]), tuple):
                    raise GraphError("UNSUPPORTED", where + ".OutDataTypeStr", "Discrete-Time Integrator %s with an "
                                     "explicit output type is not recorded; use the internal rule"
                                     % p["IntegratorMethod"])
                if self._on_cycle(n):
                    raise GraphError("UNSUPPORTED", where, "Discrete-Time Integrator %s inside a feedback loop is not "
                                     "recorded (Forward Euler is)" % p["IntegratorMethod"])
            if str(p["OutDataTypeStr"]).strip().startswith("Inherit") and p["OutDataTypeStr"] != fc._INHERIT_RULE:
                # recordings (10 designs): with back propagation Simulink compiles
                # the integrator as double; the engine answered every 'Inherit: ...' string like the internal rule
                raise GraphError("UNSUPPORTED", where + ".OutDataTypeStr", "Discrete-Time Integrator %s: "
                                 "OutDataTypeStr %r is not supported (with back propagation Simulink compiles the "
                                 "integrator as double; the other inherit rules are not recorded); use 'Inherit: "
                                 "Inherit via internal rule' or an explicit fixdt type" % (n, p["OutDataTypeStr"]))
            if p["OutDataTypeStr"] != fc._INHERIT_RULE:
                # refused unless the gain is a power of two (every gain quantisation keeps it exact;
                # recordings 159 modules), see fc.dti_explicit_refusal
                why_ = fc.dti_explicit_refusal(p, ts[0]) if ts and ts[0] != BOOL else "a boolean input"
                if why_:
                    raise GraphError("UNSUPPORTED", where + ".OutDataTypeStr", "Discrete-Time Integrator: %s; use the "
                                     "internal rule" % why_)
            try:
                K = fc.dti_gain(p)
            except ValueError:
                raise not_a_number(p.get("gainval"), where + ".gainval", "numeric gain")
            if K <= 0 or K.numerator & (K.numerator - 1) or K.denominator & (K.denominator - 1):
                raise GraphError("UNSUPPORTED", where + ".gainval", "Discrete-Time Integrator: the internal-rule "
                                 "output has slope = input slope * K * T, which is a binary-point "
                                 "type only when K * T is a power of two; engine signals are binary-point only")
        if bt in ("Lookup1D", "Lookup2D"):
            self._lut_guard(p, where, 1 if bt == "Lookup1D" else 2)
        if bt == "Lookup2D":
            if p["OutDataTypeStr"].startswith("Inherit") and p["OutDataTypeStr"] != fc._SAME_FIRST:
                raise GraphError("UNSUPPORTED", where + ".OutDataTypeStr", "2-D Lookup Table: explicit fixdt or "
                                 "'Inherit: Same as first input' only")
            if p["InterpMethod"] not in ("Flat", "Nearest", "Linear point-slope"):
                raise GraphError("UNSUPPORTED", where + ".InterpMethod", "Flat, Nearest or Linear point-slope")
            if p["ExtrapMethod"] != "Clip":
                raise GraphError("PORT_TYPE_INVALID" if p["ExtrapMethod"] == "Linear" else "UNSUPPORTED",
                                 where + ".ExtrapMethod", "Clip only (Simulink refuses fixed-point "
                                 "linear extrapolation, Simulink:blocks:OnlyExtrapFloatAndHalf)")
            if p["InterpMethod"] == "Linear point-slope":
                _lut_types_recorded(p, where, "2-D")
            if p["InterpMethod"] == "Linear point-slope" and ts and BOOL not in ts[:2] and \
                    str(p.get("FractionDataTypeStr", fc._INHERIT_RULE)).startswith("Inherit"):
                try:   # the rule where recordings decides it; elsewhere a refusal naming the condition
                    self._lut2d_frac(p, [tuple(t) for t in ts[:2]])
                except ValueError as e:
                    raise GraphError("UNSUPPORTED", where + ".FractionDataTypeStr", '2-D Lookup Table: %s'
                                     "" % e)
            if ts and BOOL not in ts[:2]:
                try:
                    fc._lut2d_setup(p, [tuple(t) for t in ts[:2]])
                except ValueError as e:
                    raise GraphError("BAD_PARAM", where, str(e))
            if p["InterpMethod"] == "Nearest" and any(
                    str(p["BreakpointsForDimension%dDataTypeStr" % k]) != "Inherit: Same as corresponding input"
                    and ts and ts[k - 1] != BOOL and fc._named_type(p["BreakpointsForDimension%dDataTypeStr" % k])
                    != tuple(ts[k - 1]) for k in (1, 2)):
                raise GraphError("PORT_TYPE_INVALID", where, "Nearest interpolation needs breakpoints of the input's "
                                 "type (Simulink:blocks:PreLookupNearestUAndBpDataTypeMismatch)")
        if bt in ("Lookup1D", "Lookup2D"):
            # recordings (argued): a boolean output type crashed (INTERNAL), a boolean input gave BAD_PARAM "integer
            # required, got None" -- neither is recorded
            if str(p.get("OutDataTypeStr", "")).strip() == "boolean":
                raise GraphError("UNSUPPORTED", where + ".OutDataTypeStr", "a boolean lookup output is not supported "
                                 "(not recorded)")
            if ts and BOOL in ts[:(1 if bt == "Lookup1D" else 2)]:
                raise GraphError("UNSUPPORTED", where, "a boolean lookup input is not supported (not recorded); "
                                 "convert it to a numeric type first (Data Type Conversion)")
            self._lut_static_checks(p, ts, where, 1 if bt == "Lookup1D" else 2)
            self._lut_fine_input_check(p, ts, where, 1 if bt == "Lookup1D" else 2)
            self._lut_a4_checks(p, ts, where, 1 if bt == "Lookup1D" else 2)
        if bt == "Lookup1D":
            ot = p["OutDataTypeStr"]
            if ot.startswith("Inherit") and ot != fc._SAME_FIRST:
                raise GraphError("UNSUPPORTED", where + ".OutDataTypeStr", "1-D Lookup Table: explicit fixdt or "
                                 "'Inherit: Same as first input' only (back propagation gives double)")
            if str(p["InterpMethod"]).startswith("Linear") and p["ExtrapMethod"] == "Linear" and ts and ts[0] != BOOL:
                # recordings (20 designs) and recordings: Simulink rejects linear extrapolation on
                # fixed point (Simulink:blocks:OnlyExtrapFloatAndHalf) -- the library default, so write Clip
                raise GraphError("PORT_TYPE_INVALID", where + ".ExtrapMethod", "Simulink rejects linear extrapolation "
                                 "on fixed-point signals (Simulink:blocks:OnlyExtrapFloatAndHalf); set ExtrapMethod "
                                 "to Clip")
            if p["InterpMethod"] not in ("Flat", "Nearest", "Linear point-slope"):
                raise GraphError("UNSUPPORTED", where + ".InterpMethod", "Flat, Nearest or Linear point-slope")
            if p["InterpMethod"] == "Linear point-slope":
                if p["ExtrapMethod"] != "Clip":
                    # recordings (60 / 60): Simulink refuses linear extrapolation of a fixed-point table
                    raise GraphError("UNSUPPORTED", where + ".ExtrapMethod", "linear interpolation: Clip only -- "
                                     "Simulink refuses fixed-point linear extrapolation "
                                     "(Simulink:blocks:OnlyExtrapFloatAndHalf); write ExtrapMethod "
                                     "'Clip'")
                _lut_types_recorded(p, where, "1-D")
                if ts and ts[0] != BOOL:
                    try:   # breakpoints / table that cannot be set up (e.g. not strictly increasing once quantised)
                        fc._lut_setup(p, tuple(ts[0]))
                    except ValueError as e:
                        raise GraphError("BAD_PARAM", where, str(e))
                    try:   # the rule where recordings decides it; elsewhere a refusal naming the condition
                        self._lut1d_frac(p, tuple(ts[0]))
                    except ValueError as e:
                        raise GraphError("UNSUPPORTED", where + ".FractionDataTypeStr", '1-D Lookup Table: %s'
                                         "" % e)

    def _lut_fine_input_check(self, p, ts, where, dims):
        """with linear interpolation, an input much finer than its
        breakpoints rounds the fraction to 2^-gap where the recordings decide it. 1-D: ref/fxcore.py
        lut1d_fraction_by_gap, applied in _lut1d_frac (refusals there name the condition). 2-D: one recording only, so
        only the gaps where the fraction type is used unchanged are answered (lut2d_fraction_gap_ok)."""
        if p.get("InterpMethod") != "Linear point-slope" or dims != 2 or not ts or len(ts) < 2 or \
                any(t == BOOL or not isinstance(t, (list, tuple)) for t in ts[:2]):
            return
        try:
            ft = self._lut2d_frac(p, [tuple(t) for t in ts[:2]])
            gaps = fc.lut2d_gaps(p, [tuple(t) for t in ts[:2]])
        except (ValueError, KeyError):
            return   # refused by the other checks, with their own reason
        if not isinstance(fc._named_type(str(ft)), tuple):
            return
        try:
            fc.lut2d_fraction_gap_ok(ft, gaps)
        except ValueError as e:
            raise GraphError("UNSUPPORTED", where + ".BreakpointsForDimension1DataTypeStr", "2-D Lookup Table, linear "
                             "interpolation: %s" % e)

    # the recorded type combinations (input, breakpoints,
    # table, output) -- anything else refused with the design to record
    LUTND_TYPES = {((1, 8, 4), None, None, None), ((1, 8, 4), (1, 12, 6), (1, 8, 4), None),
                   ((0, 8, 4), None, (0, 8, 4), None), ((1, 12, 6), (1, 8, 4), (1, 12, 6), (1, 16, 8)),
                   ((1, 12, 6), None, (0, 8, 4), (1, 12, 6)), ((0, 8, 4), (1, 12, 6), (1, 8, 4), None)}

    # Gain Matrix(K*u): one row per recording -- (inputs, parameter types, output types, RndMeth, saturation);
    # every row fits one rule (each product rounded into the output type, then the row accumulated in it, wrapping
    # or saturating at every addition)
    _S8, _S12, _I8 = (1, 8, 4), (1, 12, 8), (1, 8, 0)
    MATGAIN_FAMILIES = {
        # recordings (18 models)
        "dense": ({_S8, _S12, _I8}, {fc._INHERIT_RULE, "fixdt(1,8,5)"}, {"Inherit: Same as first input", "fixdt(1,24,10)"},
                  {"Floor"}, {"off"}),
        # recordings (120 models)
        "branch": ({_S8, _S12}, {"fixdt(1,8,6)"},
                   lambda its, ot, sat: len(its) == 1 and (next(iter(its)), ot, sat) in Design.MATGAIN_BRANCH,
                   {"Floor", "Nearest", "Zero", "Convergent"}, {"off", "on"}),
        # recordings (60 models)
        "loss_a": ({_S12}, {fc._INHERIT_RULE, "fixdt(1,8,7)"}, {"fixdt(1,24,10)", "fixdt(1,16,4)", "fixdt(1,8,2)"},
                   {"Floor", "Nearest", "Zero", "Convergent", "Ceiling"}, {"off", "on"}),
        # recordings (81 models)
        "loss_b": ({_S8, _S12, _I8}, {fc._INHERIT_RULE, "fixdt(1,8,7)", "fixdt(1,16,12)"},
                   {"fixdt(1,24,10)", "fixdt(1,16,4)", "fixdt(0,16,8)"}, {"Floor"}, {"off"}),
        # Square matrices with inherited or explicit output types on both hardware profiles.
        "lab2_105": ({(1, 16, 8), (1, 16, 14), (1, 12, 4), (1, 16, 0), (0, 8, 4)}, {fc._INHERIT_RULE, "fixdt(1,16,14)"},
                     lambda its, ot, sat, user=None: user in ("Inherit: Same as input", "Inherit: Inherit via internal "
                                                              "rule", "fixdt(1,32,16)"),
                     {"Floor", "Nearest"}, {"off", "on"}),
        # recordings (420 models, host and ASIC; 2x2 / 2x3 / 3x2 / 3x3 / 4x4): the Sum rows carry
        # 'Same as first input' for both the internal rule and 'Same as input' (the lowering, fxvector._matgain_lower)
        "keypath": ({(1, 16, 0), (1, 16, 15), (1, 20, 10), (1, 24, 12), (1, 32, 16), (0, 16, 8), (1, 12, 4)},
                    {fc._INHERIT_RULE, "fixdt(1,16,14)"}, {"Inherit: Same as first input", "fixdt(1,32,16)"},
                    {"Floor", "Nearest", "Zero", "Convergent", "Ceiling"}, {"off", "on"})}
    MATGAIN_ASIC = {"lab2_105", "keypath"}   # the families recorded on ASIC / FPGA hardware too
    MATGAIN_33 = {"lab2_105", "keypath"}     # the families recorded with a 3x3 matrix
    MATGAIN_BRANCH = {((1, 12, 8), "fixdt(1,16,12)", "off"), ((1, 12, 8), "fixdt(1,16,2)", "off"),
                      ((1, 12, 8), "fixdt(1,16,8)", "off"), ((1, 12, 8), "fixdt(1,8,12)", "off"),
                      ((1, 12, 8), "fixdt(1,8,12)", "on"), ((1, 8, 4), "fixdt(1,16,-2)", "off"),
                      ((1, 8, 4), "fixdt(1,16,4)", "off"), ((1, 8, 4), "fixdt(1,16,8)", "off"),
                      ((1, 8, 4), "fixdt(1,8,8)", "off"), ((1, 8, 4), "fixdt(1,8,8)", "on")}

    # recorded shapes
    PRELOOKUP_IN = {(1, 8, 4), (1, 12, 6), (0, 8, 4),
                    # recordings (host; breakpoints inherited, fraction 32 bits, 3 / 5 / 9 points)
                    (1, 8, 0), (1, 16, 0), (1, 16, 8), (1, 16, 14), (0, 16, 8), (1, 32, 16)}
    PRELOOKUP_NBP = {2, 3, 4, 5, 9}
    SQRT_SAME_FIRST = {(0, 10, 4, "Ceiling", True), (0, 24, 12, "Ceiling", False), (1, 12, 10, "Ceiling", False),
                       (0, 10, 4, "Nearest", True), (1, 12, 10, "Nearest", False), (0, 10, 4, "Zero", True),
                       (0, 24, 12, "Zero", False), (0, 8, 0, "Floor", False), (0, 12, 12, "Floor", False),
                       (0, 16, 8, "Floor", True)}   # 
    INTERP16_HOST_TABLES = {(1, 16, 8), (1, 16, 12), (0, 16, 8), (0, 16, 12), (1, 16, 4), (1, 16, 14)}
    PRELOOKUP_ASIC_IN = {(1, 8, 0), (1, 16, 0), (1, 8, 4), (1, 16, 8), (1, 16, 14), (0, 8, 4), (0, 16, 8), (1, 32, 16)}
    PRELOOKUP_BP = {None, (1, 12, 6), (1, 8, 4)}
    PRELOOKUP_FR = {(0, 32, 31), (0, 16, 15), (0, 8, 7)}
    PRELOOKUP_IN_105 = {(1, 16, 0), (1, 16, 8), (1, 16, 14), (0, 8, 4)}   # recordings (library fraction type)
    INTERP_TO_105 = {((1, 16, 8), (1, 32, 16)), ((1, 16, 0), (1, 16, 0))}   # recordings, Linear / Flat
    INTERP_TO = {((1, 12, 6), (1, 16, 8)), ((0, 8, 4), (1, 12, 6)), ((1, 8, 4), (1, 8, 4)), ((1, 8, 4), (1, 16, 8)),
                 ((0, 8, 4), (0, 12, 6)), ((0, 8, 4), (1, 16, 10)), ((1, 12, 6), (1, 12, 6)), ((1, 12, 6), (1, 12, 4)),
                 ((0, 16, 8), (1, 12, 6)), ((1, 16, 8), (1, 8, 4)),
                 ((1, 16, 8), (1, 32, 16)), ((0, 16, 12), (0, 16, 12))}   # recordings (host)
    INTERP_FLAT = {((1, 12, 6), (1, 16, 8)), ((0, 8, 4), (1, 12, 6)),
                   ((1, 16, 8), (1, 32, 16)), ((0, 16, 12), (0, 16, 12))}   # recordings (host)

    def _prelookup_check(self, n, ts):
        p, where = self.params[n], "design.blocks." + n.split("\x1d")[0]
        if ts and (ts[0] == BOOL or not isinstance(ts[0], tuple)):
            raise GraphError("UNSUPPORTED", where, "Prelookup with a boolean or non-fixed-point input is not recorded")
        if p["ExtrapMethod"] != "Clip":
            # recordings (the library default Linear): PreLookupExtrapMethodError
            raise GraphError("PORT_TYPE_INVALID", where + ".ExtrapMethod", "Simulink rejects linear extrapolation in a "
                             "fixed-point Prelookup (Simulink:blocks:PreLookupExtrapMethodError); set ExtrapMethod to "
                             "Clip")
        for k, v in DEFAULTS["Prelookup"].items():
            if k not in ("BreakpointDataTypeStr", "FractionDataTypeStr", "ExtrapMethod") and str(p.get(k, v)).strip() != v:
                raise GraphError("UNSUPPORTED", where + "." + k, "Prelookup %s %r is not recorded (only %r)"
                                 % (k, p.get(k), v))
        bt_ = str(p.get("BreakpointDataTypeStr")).strip()
        bt_ = None if bt_.startswith("Inherit") else fc._named_type(bt_)
        xt = tuple(ts[0]) if ts else None
        inh_fr = str(p.get("FractionDataTypeStr")).strip() == fc._INHERIT_RULE
        try:
            ft_ = fc.prelookup_fraction_type(p, xt or (1, 16, 0), fc._asic(self.hw))
        except ValueError as e:
            raise GraphError("UNSUPPORTED", where + ".FractionDataTypeStr", str(e))
        if xt not in self.PRELOOKUP_IN or bt_ not in self.PRELOOKUP_BP or ft_ not in self.PRELOOKUP_FR:
            raise GraphError("UNSUPPORTED", where, "Prelookup with input %s, breakpoints %s, fraction %s is not recorded "
                             "(inputs sfix8_En4 / sfix12_En6 / ufix8_En4, and int8 / int16 / sfix16_En8 / sfix16_En14 / ufix16_En8 / "
                             "sfix32_En16 with inherited breakpoints; breakpoints inherited, fixdt(1,12,6) or "
                             "fixdt(1,8,4); fraction fixdt(0,32,31) / (0,16,15) / (0,8,7)); record it to open it"
                             % (_tstr(xt) if xt else "?", bt_ or "inherited", ft_))
        if fc._asic(self.hw) and not (inh_fr and bt_ is None and xt in self.PRELOOKUP_ASIC_IN):
            # on ASIC recorded with the inherited fraction (the input's word) and
            # inherited breakpoints, for these inputs only
            raise GraphError("UNSUPPORTED", where, "Prelookup on ASIC / FPGA is recorded with the inherited fraction "
                             "and breakpoint types and an input of %s only"
                             % ", ".join(_tstr(t) for t in sorted(self.PRELOOKUP_ASIC_IN)))
        try:
            bq, _ft = fc._prelookup_setup(p, xt, fc._asic(self.hw))
        except ValueError as e:
            raise GraphError("BAD_PARAM", where + ".BreakpointsData", str(e))
        for v in fc._vec(p["BreakpointsData"]):   # recordings (as the n-D table): fxpParameterOverflow
            _param_overflow(v, bt_ or xt, where + ".BreakpointsData", "breakpoint", self.pom,
                            getattr(self, "param_notes", None))
        lab2_105 = xt in self.PRELOOKUP_IN_105 and inh_fr and bt_ is None
        if len(bq) not in self.PRELOOKUP_NBP and not (lab2_105 and len(bq) == 17):
            raise GraphError("UNSUPPORTED", where + ".BreakpointsData", "Prelookup with %d breakpoints is not recorded "
                             "(2 to 5, or 9; 17 with the recorded input types and the inherited types)" % len(bq))

    def _interp_pl_check(self, n, ts):
        p, where = self.params[n], "design.blocks." + n
        dims = int(str(p["NumberOfTableDimensions"]).strip())
        for d in range(dims):   # the recorded wiring: k / f of one Prelookup per dimension
            sk, sf = self.src[n][2 * d], self.src[n][2 * d + 1]
            if self.btype.get(sk) != "Prelookup" or sf != sk + "\x1df":
                raise GraphError("UNSUPPORTED", where, "Interpolation Using Prelookup: inputs %d / %d must be the k / f "
                                 "outputs of one Prelookup (the recorded wiring)" % (2 * d + 1, 2 * d + 2))
        ot = str(p.get("OutDataTypeStr")).strip()
        if ot.startswith("Inherit"):
            # the inherited output compiles to double
            raise GraphError("UNSUPPORTED", where + ".OutDataTypeStr", "Interpolation Using Prelookup with an inherited "
                             "output type compiles to double in Simulink (not simulated); give an explicit fixdt")
        if p["ExtrapMethod"] != "Clip":
            raise GraphError("PORT_TYPE_INVALID", where + ".ExtrapMethod", "Simulink rejects linear extrapolation in a "
                             "fixed-point Interpolation Using Prelookup (Simulink:blocks:CannotExtrapData); set "
                             "ExtrapMethod to Clip")
        for k, v in DEFAULTS["InterpolationUsingPrelookup"].items():
            if k not in ("NumberOfTableDimensions", "InterpMethod", "ExtrapMethod", "OutDataTypeStr", "TableDataTypeStr") \
                    and str(p.get(k, v)).strip() != v:
                raise GraphError("UNSUPPORTED", where + "." + k, "Interpolation Using Prelookup %s %r is not recorded "
                                 "(only %r)" % (k, p.get(k), v))
        out_t = fc._named_type(ot)
        tt_ = str(p.get("TableDataTypeStr")).strip()
        tt_ = out_t if tt_.startswith("Inherit") else fc._named_type(tt_)
        ft_ = tuple(self.otype.get(self.src[n][1]) or ())
        meth = p["InterpMethod"]
        combos = self.INTERP_FLAT if meth in ("Flat", "Nearest") else self.INTERP_TO
        # on ASIC (a 16-bit table with the 16 / 8-bit inherited fraction): both recorded
        # table / output pairs, Flat and Linear, equal Simulink with the table type as the intermediate
        # (fxcore.interp_pl_itype); on the host this stays refused (recordings tu16En8 fits no rule)
        # (both recorded 1-D only -- recordings: 2-D tables passed this opening, 46 / 50 answered, unrecorded)
        kp16_ = dims == 1 and fc._asic(self.hw) and (tt_, out_t) in (((0, 16, 12), (0, 16, 12)), ((1, 16, 8), (1, 32, 16)))
        # recordings (192 models with a fixed-point output, host and ASIC / FPGA, every step equal): 1-D, fed by
        # a Prelookup on its inputs with the library fraction type, tables / outputs INTERP_TO_105, Linear and Flat;
        # a signed 16-bit table with a 16 / 8-bit fraction (ASIC) interpolates in the table type (interp_pl_itype)
        pl_src = [self.src[n][2 * d] for d in range(dims)]
        ok_105 = dims == 1 and meth in ("Flat", "Linear point-slope") and (tt_, out_t) in self.INTERP_TO_105 and \
            all(tuple(self.otype.get(self.src[k_][0]) or ()) in self.PRELOOKUP_IN_105 and
                str(self.params[k_].get("FractionDataTypeStr")).strip() == fc._INHERIT_RULE for k_ in pl_src)
        # recordings (host, 108 models: 16-bit tables x a 16 / 8-bit fraction x three outputs, every
        # step equal with fxcore.interp_pl_itype and its unsigned falling-segment rule)
        h16_ = dims == 1 and meth == "Linear point-slope" and not fc._asic(self.hw) and \
            tt_ in self.INTERP16_HOST_TABLES and \
            out_t in (tt_, (1, 32, 16), (1, 16, 12)) and ft_ and ft_[1] in (16, 8)
        kp16_ = kp16_ or h16_
        if not ok_105 and (meth not in ("Flat", "Nearest", "Linear point-slope") or
                           ((tt_, out_t) not in combos and not h16_) or
                           (tt_[1] == 16 and ft_ and ft_[1] != 32 and not kp16_)):
            # a 16-bit table with a 16 / 8-bit fraction: recordings ts16En8 fits two intermediate
            # rules (32-bit, or the table type), recordings s16En10 only the table type, tu16En8 neither
            raise GraphError("UNSUPPORTED", where, "Interpolation Using Prelookup %s with table %s, output %s%s is not "
                             "recorded (10 table / output combinations for Linear "
                             "point-slope, 2 for Flat / Nearest; a 16-bit table only with the 32-bit fraction, "
                             "its intermediate type is not settled otherwise); record it to open it"
                             % (meth, tt_, out_t, ", fraction %s" % (ft_,) if ft_ else ""))
        kp_asic = fc._asic(self.hw) and (tt_, out_t) in {((1, 16, 8), (1, 32, 16)), ((0, 16, 12), (0, 16, 12))}
        if (fc._asic(self.hw) and not kp_asic and not ok_105) or self._on_cycle(n):
            # (ASIC: recordings's two explicit table / output pairs; recordings's)
            raise GraphError("UNSUPPORTED", where, "Interpolation Using Prelookup is recorded on the host, outside loops "
                             "(on ASIC / FPGA only fixdt(1,16,8) -> fixdt(1,32,16) and fixdt(0,16,12) -> "
                             "fixdt(0,16,12); int16 -> int16 after a Prelookup on the recorded input types)")
        if str(p["Table"]).strip().startswith("reshape") and dims == 1:
            raise GraphError("UNSUPPORTED", where + ".Table", "Interpolation Using Prelookup: a 1-D table written "
                             "with reshape is not supported here; write a row [v1 v2 ...]")
        try:
            if dims == 1:
                vals = fc._vec(p["Table"])
                # negative table values: recorded with a signed fixdt(1,16,8) table into fixdt(1,32,16)
                # (host, Linear point-slope and Flat)
                neg_ok_ = (tt_, out_t) in {((1, 16, 8), (1, 32, 16))} or (h16_ and tt_[0] == 1)
                if len(vals) < 2 or (any(v < 0 for v in vals) and not neg_ok_ and not ok_105):   # (signed)
                    raise ValueError("a table of at least two non-negative values is recorded (negative values only "
                                     "with a fixdt(1,16,8) table into fixdt(1,32,16))")
            else:
                table, shape = fc.lutnd_table(p["Table"])
                if any(v < 0 for v in table.values()):
                    raise ValueError("a table of non-negative values is recorded")
        except ValueError as e:
            raise GraphError("UNSUPPORTED", where + ".Table", "Interpolation Using Prelookup: %s" % e)
        sizes = [len(vals)] if dims == 1 else list(shape)
        for d in range(dims):
            nbp = len(fc._vec(self.params[self.src[n][2 * d]]["BreakpointsData"]))
            if d >= len(sizes) or sizes[d] != nbp:
                # Simulink:blocks:IndexRangeMismatch -- the table's dimension d must have as many
                # points as the Prelookup feeding k_d has breakpoints
                raise GraphError("PORT_TYPE_INVALID", where + ".Table", "Interpolation Using Prelookup: table "
                                 "dimension %d has %s points but Prelookup %s has %d breakpoints (Simulink: "
                                 "IndexRangeMismatch)" % (d + 1, sizes[d] if d < len(sizes) else 0,
                                                          self.src[n][2 * d], nbp))
        for v in (vals if dims == 1 else list(table.values())):   # fxpParameterOverflow
            _param_overflow(v, tt_, where + ".Table", "table value", self.pom, getattr(self, "param_notes", None))

    # the recorded span of the dimension-by-dimension
    # rule (fxcore._blk_lutnd_dims; there is the axis list)
    LUTND_DIMS_IN = {(0, 16, 8), (0, 8, 4), (1, 12, 6), (1, 16, 0), (1, 16, 8), (1, 8, 4)}
    LUTND_DIMS_TABLE = {"fixdt(0,16,12)", "fixdt(1,16,14)", "fixdt(1,16,8)", "double"}
    LUTND_DIMS_OUT = {"Inherit: Same as first input", "fixdt(1,16,8)", "fixdt(1,32,20)"}
    LUTND_DIMS_IT = {"Inherit: Inherit via internal rule", "Inherit: Same as output", "fixdt(1,32,16)"}

    def _lutnd_dims_outside(self, n, ts):
        """The first axis of a LookupND outside the recorded span of the dimension-by-dimension rule, or None."""
        p = self.params[n]
        dims = int(str(p["NumberOfTableDimensions"]).strip())
        if dims != 3:
            return "%d dimensions (recorded: 3)" % dims
        if any(not isinstance(t, tuple) or tuple(t) not in self.LUTND_DIMS_IN for t in ts[:dims]):
            return "input types %s (recorded: %s)" % (", ".join(_tstr(t) for t in ts[:dims]),
                                                      ", ".join(_tstr(t) for t in sorted(self.LUTND_DIMS_IN)))
        rec = {"InterpMethod": ("Flat", "Nearest", "Linear point-slope"), "ExtrapMethod": ("Clip",),
               "BreakpointsSpecification": ("Explicit values",), "RndMeth": ("Floor", "Nearest", "Zero", "Simplest"),
               "SaturateOnIntegerOverflow": ("on", "off"), "UseLastTableValue": ("on", "off"),
               "InternalRulePriority": ("Speed",), "FractionDataTypeStr": (fc._INHERIT_RULE,),
               "IndexSearchMethod": ("Binary search",), "DiagnosticForOutOfRangeInput": ("None",)}
        for d in range(1, dims + 1):
            rec["BreakpointsForDimension%dDataTypeStr" % d] = ("Inherit: Same as corresponding input",)
        dflt = DEFAULTS.get("LookupND", {})
        for k, vals in rec.items():
            v = str(p.get(k, dflt.get(k, vals[0]))).strip()
            if v not in vals:
                return "%s %r (recorded: %s)" % (k, v, ", ".join(repr(x) for x in vals))
        tt_ = str(p.get("TableDataTypeStr", "Inherit: Same as output")).strip()
        ot_ = str(p.get("OutDataTypeStr", fc._SAME_FIRST)).strip()
        it_ = str(p.get("IntermediateResultsDataTypeStr", "Inherit: Same as output")).strip()
        if tt_ not in self.LUTND_DIMS_TABLE or ot_ not in self.LUTND_DIMS_OUT or it_ not in self.LUTND_DIMS_IT:
            return "table type %s, output type %s, intermediate type %s (not among the recorded ones)" % (tt_, ot_, it_)
        if p["InterpMethod"] == "Linear point-slope" and tt_.replace(" ", "").startswith("fixdt(0,"):
            # native tests: with an unsigned table, Linear differed by
            # 1 LSB in 4 of 46 answered designs (Simplest x3, Floor x1) -- refused until recorded
            return "linear interpolation with the unsigned table type %s (not settled by the recordings)" % tt_
        if p["InterpMethod"] == "Linear point-slope" and (it_ != fc._INHERIT_RULE or tt_ == "double"):
            return ("linear interpolation with the intermediate type %s and the table type %s (recorded to the step: "
                    "the internal-rule intermediate type with a fixed-point table)" % (it_, tt_))
        try:
            table, shape = fc.lutnd_table(p["Table"])
        except ValueError as e:
            return str(e)
        if len(shape) != dims or any(not 2 <= d_ <= 5 for d_ in shape):
            return "table shape %s (recorded: 2 to 5 points per dimension)" % (shape,)
        if self._on_cycle(n):
            return "a feedback loop through the table"
        return None

    def _lutnd_check(self, n, ts):
        p = self.params[n]
        p.pop("_NDRule", None)
        try:
            self._lutnd_check_v1(n, ts)
            return
        except GraphError as e:
            if e.code != "UNSUPPORTED":
                raise
            why = self._lutnd_dims_outside(n, ts)
            if why is not None:
                raise GraphError("UNSUPPORTED", e.where, "%s; the n-D table answered dimension by dimension is "
                                 "recorded for a narrower span: %s" % (e.message, why))
        where = "design.blocks." + n
        tt_ = str(p.get("TableDataTypeStr", "Inherit: Same as output")).strip()
        table, shape = fc.lutnd_table(p["Table"])
        dims = int(str(p["NumberOfTableDimensions"]).strip())
        ot_ = fc._named_type(str(p.get("OutDataTypeStr")).strip()) if str(p.get("OutDataTypeStr")).strip() != \
            fc._SAME_FIRST else tuple(ts[0])
        ttq = fc._named_type(tt_) if not tt_.startswith("Inherit") and tt_ != "double" else (None if tt_ == "double" else ot_)
        if ttq is not None:
            for v in table.values():   # fxpParameterOverflow, as the earlier rule
                _param_overflow(v, ttq, where + ".Table", "table value", self.pom, getattr(self, "param_notes", None))
        for k in range(1, dims + 1):
            for v in fc._vec(p["BreakpointsForDimension%d" % k]):
                _param_overflow(v, tuple(ts[k - 1]), where + ".BreakpointsForDimension%d" % k, "breakpoint", self.pom,
                                getattr(self, "param_notes", None))
        p["_NDRule"] = "dims"
        try:
            fc._blk_lutnd({"params": p, "hw": self.hw, "ins": [{"s": t[0], "wl": t[1], "fl": t[2], "bin": "0" * t[1]}
                                                               for t in ts[:dims]]})
        except ValueError as e:
            p.pop("_NDRule", None)
            raise GraphError("BAD_PARAM", where, str(e))

    def _lutnd_check_v1(self, n, ts):
        p, where = self.params[n], "design.blocks." + n
        dims = int(str(p["NumberOfTableDimensions"]).strip())
        self._lut_guard(p, where, dims)
        if ts and any(t == BOOL or not isinstance(t, tuple) for t in ts[:dims]):
            raise GraphError("UNSUPPORTED", where, "LookupND with a boolean or non-fixed-point input is not recorded")
        if p["InterpMethod"] not in ("Flat", "Nearest", "Linear point-slope"):
            raise GraphError("UNSUPPORTED", where + ".InterpMethod", "LookupND: Flat, Nearest or Linear point-slope")
        if p["ExtrapMethod"] != "Clip":
            # recordings (the library default Linear): OnlyExtrapFloatAndHalf
            raise GraphError("PORT_TYPE_INVALID", where + ".ExtrapMethod", "Simulink rejects linear extrapolation on "
                             "fixed-point signals (Simulink:blocks:OnlyExtrapFloatAndHalf); set ExtrapMethod to Clip")
        if p.get("BreakpointsSpecification", "Explicit values") != "Explicit values":
            raise GraphError("UNSUPPORTED", where + ".BreakpointsSpecification", "LookupND: explicit breakpoints are "
                             "recorded only")
        if len({tuple(t) for t in ts[:dims]}) != 1:
            raise GraphError("UNSUPPORTED", where, "LookupND with inputs of different types is not recorded")
        xt = tuple(ts[0])

        def typ(k, inh):
            v = str(p.get(k, inh)).strip()
            return None if v == inh else fc._named_type(v)
        bts = {typ("BreakpointsForDimension%dDataTypeStr" % k, "Inherit: Same as corresponding input")
               for k in range(1, dims + 1)}
        tt_, ot_ = typ("TableDataTypeStr", "Inherit: Same as output"), typ("OutDataTypeStr", fc._SAME_FIRST)
        combo = (xt, bts.pop() if len(bts) == 1 else "mixed", tt_, ot_)
        if combo not in self.LUTND_TYPES:
            raise GraphError("UNSUPPORTED", where, "LookupND with input %s, breakpoints %s, table %s, output %s is not "
                             "recorded (six type combinations of sfix8_En4 / "
                             "ufix8_En4 / sfix12_En6 inputs); record this combination to open it"
                             % (_tstr(xt), combo[1] or "inherited", tt_ or "inherited", ot_ or "inherited"))
        if p["InterpMethod"] == "Nearest" and combo[1] not in (None, xt):
            raise GraphError("PORT_TYPE_INVALID", where, "Nearest interpolation needs breakpoints of the input's type "
                             "(Simulink:blocks:PreLookupNearestUAndBpDataTypeMismatch)")
        for k, want in (("IntermediateResultsDataTypeStr", "Inherit: Same as output"),
                        ("FractionDataTypeStr", fc._INHERIT_RULE), ("InternalRulePriority", "Speed"),
                        ("UseLastTableValue", "off"), ("RndMeth", "Simplest"), ("SaturateOnIntegerOverflow", "off")):
            if str(p.get(k, want)).strip() != want:
                raise GraphError("UNSUPPORTED", where + "." + k, "LookupND %s %r is not recorded (only %r)"
                                 % (k, p.get(k), want))
        if fc._asic(self.hw) or self._on_cycle(n):
            raise GraphError("UNSUPPORTED", where, "LookupND is recorded on the host, outside feedback loops")
        try:
            table, shape = fc.lutnd_table(p["Table"])
        except ValueError as e:
            raise GraphError("UNSUPPORTED", where + ".Table", "LookupND: %s" % e)
        if len(shape) != dims or any(not 2 <= d_ <= 4 for d_ in shape):
            raise GraphError("UNSUPPORTED", where + ".Table", "LookupND: a table of %d dimensions with 2 to 4 points "
                             "each is recorded, got shape %s" % (dims, shape))
        # a table value or breakpoint outside its type by more than one LSB stops Simulink
        # (fxpParameterOverflow, default diagnostics); the engine had saturated it
        for v in table.values():
            _param_overflow(v, tt_ or ot_ or xt, where + ".Table", "table value", self.pom, getattr(self, "param_notes", None))
        for k in range(1, dims + 1):
            bt_k = typ("BreakpointsForDimension%dDataTypeStr" % k, "Inherit: Same as corresponding input") or xt
            for v in fc._vec(p["BreakpointsForDimension%d" % k]):
                _param_overflow(v, bt_k, where + ".BreakpointsForDimension%d" % k, "breakpoint", self.pom,
                                getattr(self, "param_notes", None))
        if any(v < 0 for v in table.values()):
            raise GraphError("UNSUPPORTED", where + ".Table", "LookupND with negative table values is not recorded")
        try:
            fc._blk_lutnd({"params": p, "ins": [{"s": t[0], "wl": t[1], "fl": t[2], "bin": "0" * t[1]}
                                                 for t in ts[:dims]]})
        except ValueError as e:
            raise GraphError("BAD_PARAM", where, str(e))

    def _lut_guard(self, p, where, dims):
        """recordings: shapes that crashed the lookup setup (INTERNAL) -- floating-point types anywhere, a
        left-out breakpoint vector -- refused before any rule reads them."""
        keys = ["OutDataTypeStr", "TableDataTypeStr", "IntermediateResultsDataTypeStr", "FractionDataTypeStr"] + \
            ["BreakpointsForDimension%dDataTypeStr" % k for k in range(1, dims + 1)]
        for key in keys:
            v = str(p.get(key, "")).strip()
            # 'double' tables / breakpoints are answered where recorded (
            # recordings): only a double output, and single / half anywhere, are refused here
            if v in ("single", "half") or (v == "double" and key == "OutDataTypeStr"):
                raise GraphError("UNSUPPORTED", where + "." + key, "%s %s is not supported (floating-point signals are "
                                 "not simulated here)" % (v, key))
        explicit = str(p.get("BreakpointsSpecification", "Explicit values")).strip() == "Explicit values"
        for k in range(1, dims + 1):
            if explicit and not str(p.get("BreakpointsForDimension%d" % k, "")).strip():   # (even spacing: no list)
                raise GraphError("UNSUPPORTED", where + ".BreakpointsForDimension%d" % k, "BreakpointsForDimension%d "
                                 "was left out; the library default breakpoints are not supported -- give them" % k)

    def _lut_a4_checks(self, p, ts, where, dims):
        """linear interpolation where Simulink does not form the slope term
        y0 + round(f * (y1 - y0)) as the engine does, on tables with a falling segment. The 1-D case with a 32-bit
        table type on the host is implemented (magnitude term); the others are refused -- their
        recordings were not delivered and the rule is mapped empirically only (rare shapes: no rule chased):
          recordings: an unsigned intermediate type that differs from the table type (Simulink saturates / wraps the
                negative step in the intermediate type); a step that does not fit the intermediate type at all;
          recordings: host: table type = intermediate type, unsigned 8 / 16 bits; host, 2-D: a 32-bit table type under
                Floor / Ceiling / Simplest / Nearest."""
        # a single-precision table / breakpoint / intermediate / fraction type crashed the evaluator
        # (_quant_param() takes 4 positional arguments); recordings: only the 'Inherit: ...' strings each parameter offers
        # (any text starting with "Inherit" was read as the default rule)
        known = {"TableDataTypeStr": ("Inherit: Same as output",),
                 "IntermediateResultsDataTypeStr": ("Inherit: Same as output", fc._INHERIT_RULE),
                 "FractionDataTypeStr": (fc._INHERIT_RULE,)}
        for k in range(1, dims + 1):
            known["BreakpointsForDimension%dDataTypeStr" % k] = ("Inherit: Same as corresponding input",)
        for key, inh in known.items():
            v = str(p.get(key, inh[0])).strip()
            if v.startswith("Inherit") and v not in inh:
                raise GraphError("UNSUPPORTED", where + "." + key, "%r is not supported (%s)"
                                 % (v, " / ".join(repr(x) for x in inh)))
            if v == "single":
                raise GraphError("UNSUPPORTED", where + "." + key, "single-precision %s is not supported" % key)
        self._lut_guard(p, where, dims)
        # 1-D Nearest needs breakpoints of the input's type, as the 2-D check above (Simulink
        # PreLookupNearestUAndBpDataTypeMismatch); 'Evenly spaced points' index search needs evenly spaced breakpoints
        if dims == 1 and p.get("InterpMethod") == "Nearest" and ts and ts[0] != BOOL:
            bv = str(p["BreakpointsForDimension1DataTypeStr"])
            if bv != "Inherit: Same as corresponding input" and fc._named_type(bv) != tuple(ts[0]):
                raise GraphError("PORT_TYPE_INVALID", where, "Nearest interpolation needs breakpoints of the input's "
                                 "type (Simulink:blocks:PreLookupNearestUAndBpDataTypeMismatch)")
        if str(p.get("IndexSearchMethod", "")).startswith("Evenly") and p.get("BreakpointsSpecification") != "Even spacing":
            for k in range(1, dims + 1):
                try:
                    bv = fc._vec(p["BreakpointsForDimension%d" % k])
                except (ValueError, KeyError):
                    continue
                ds = [bv[i + 1] - bv[i] for i in range(len(bv) - 1)]
                if ds and max(ds) - min(ds) > 1e-12 * max(1.0, abs(max(ds))):
                    raise GraphError("PORT_TYPE_INVALID", where + ".IndexSearchMethod", "'Evenly spaced points' with "
                                     "unevenly spaced breakpoints of dimension %d (Simulink LookupEvenSpacingMsg)" % k)
        if p.get("InterpMethod") != "Linear point-slope" or not ts or any(t == BOOL for t in ts[:dims]):
            return
        if str(p.get("TableDataTypeStr", "")).strip() == "double" and p.get("RndMeth") == "Simplest" and \
                str(p.get("OutDataTypeStr", "")).strip() != "double":
            # recordings (3 of 170 steps, -23 against -24): with a
            # double table 'Simplest' is not Floor under linear interpolation either; only Flat / Nearest are
            # recorded -- refused
            raise GraphError("UNSUPPORTED", where + ".RndMeth", "linear interpolation on a double table with "
                             "'Simplest' rounding into a fixed-point output (Simulink's rounding there is not "
                             "recorded; use Floor or Zero explicitly)")
        asic = fc._asic(self.hw)
        try:
            if dims == 1:
                _bq, _bt, tq, out_t, tt, _b, _t = fc._lut_setup(p, tuple(ts[0]))
                it = fc.lut1d_intermediate(p, out_t, tt, asic, tuple(ts[0]))
                steps = [tq[k + 1] - tq[k] for k in range(len(tq) - 1)]
            else:
                _bqs, tq, out_t = fc._lut2d_setup(p, [tuple(t) for t in ts[:2]])
                it = fc._lut2d_itype(p, out_t, asic)
                tt = fc._lut_type(p["TableDataTypeStr"], out_t)
                steps = [tq[i + 1][j] - tq[i][j] for i in range(len(tq) - 1) for j in range(len(tq[0]))] + \
                        [tq[i][j + 1] - tq[i][j] for i in range(len(tq)) for j in range(len(tq[0]) - 1)]
        except (ValueError, KeyError, TypeError, IndexError):
            return   # reported by the block's own evaluation
        if not isinstance(it, tuple) or not isinstance(tt, tuple) or not steps:
            return
        falling = any(v < 0 for v in steps)
        if dims == 1 and not asic and isinstance(ts[0], (tuple, list)) and it[0] == 0 and it[1] == 8 and ts[0][1] > 12 and \
                not (ts[0][0] == 0 and ts[0][1] == 16 and 14 <= ts[0][2] <= 16 and tuple(it) == (0, 8, 7) and
                     isinstance(out_t, tuple) and out_t[0] == 0):
            # recordings (a new-seed slice of 60 + 30): with the intermediate =
            # table = unsigned 8 bits, 18 of 30 differ for 16-bit inputs of either sign (sfix16_En4 / En8 / En12,
            # ufix16_En8, int16), under Floor / Nearest / Zero and every table shape; the 16-bit cell was all equal --
            # the 8-bit cell with inputs over 12 bits is refused, except the unsigned 16-bit inputs with fraction
            # 14..16 that recordings show equal
            # xcont: an unsigned 8-bit intermediate type
            # with a signed 16-bit input differs (recorded equal: signed 10- / 12-bit inputs, recordings
            # LU recordings): by 1 LSB of the table on rising and falling segments alike. Signed inputs above 12
            # bits with that intermediate are not answered by any recording -- refused
            raise GraphError("UNSUPPORTED", where, "linear interpolation with an unsigned 8-bit intermediate type %s "
                             "and an input %s (over 12 bits) is not supported" % (_tstr(it), _tstr(tuple(ts[0]))))
        cap = ((1 << (it[1] - 1)) - 1 if it[0] else (1 << it[1]) - 1) * Fraction(1, 2 ** it[2]) if it[2] >= 0 else \
            ((1 << (it[1] - 1)) - 1 if it[0] else (1 << it[1]) - 1) * (2 ** -it[2])
        why = None
        if max(abs(v) for v in steps) > cap:
            why = "a step between neighbouring table values does not fit the intermediate type %s" % _tstr(it)
        elif falling and it[0] == 0 and tuple(it) != tuple(tt):
            why = "a falling segment with an unsigned intermediate type %s other than the table type %s" \
                % (_tstr(it), _tstr(tt))
        elif falling and not asic and it[0] == 0 and tuple(it) == tuple(tt) and it[1] in (8, 16) and (
                dims != 1 or it[1] != 16 or ts[0] == BOOL or ts[0][1] != 16 or ts[0][0] != 0
                or not (isinstance(out_t, tuple) and out_t[0] == 0)   # recorded with unsigned outputs only (recorded)
                or p.get("RndMeth", "Floor") not in ("Floor", "Nearest", "Zero")
                or not str(p.get("FractionDataTypeStr", fc._INHERIT_RULE)).startswith("Inherit")
                or p.get("UseLastTableValue", "off") != "off"):
            # the 1-D case is recorded with unsigned 16-bit inputs and unsigned
            # outputs (the native example: a signed sfix16_En8 input / signed output is up to 4 LSB off; also
            # xcont) and only for the 16-bit table = intermediate type: the
            # the native fresh-seed slice has 18 of 30 wrong for the 8-bit cell across
            # table fractions and shapes, also with unsigned inputs, and asks for the whole 8-bit cell to be refused;
            # the engine's fixdt(0,8,7) exception rests on two recorded tables only (exp8 / sin8, ufix16_En16 / En14,
            # Floor / Nearest) -- recorded keeps the whole 8-bit cell refused (merge of a previous version); under
            # Floor / Nearest /
            # Zero, inherited fraction type, UseLastTableValue off (the magnitude slope term, ref/fxcore.py
            # lut1d_magnitude_slope); other input widths, roundings, explicit fraction types (recordings
            # fixdt(0,32,31) differs) and 2-D tables are not
            why = "a falling segment with table = intermediate type %s on this hardware: answered only for an " \
                "unsigned 16-bit table = intermediate type (the 8-bit one does not follow the recorded rule), " \
                "recorded for 1-D tables with unsigned 16-bit inputs and unsigned outputs under Floor / Nearest / " \
                "Zero, an inherited fraction " \
                "type and UseLastTableValue off only" % _tstr(it)
        elif falling and it[0] == 0 and p.get("SaturateOnIntegerOverflow") == "on":
            # recordings (10 tables): also when the unsigned
            # intermediate type IS the table type, Simulink saturates the negative slope term to 0 (the output
            # stays at y0) -- any hardware, any width; no recordings module of this shape is answered
            why = "a falling segment with an unsigned intermediate type %s and SaturateOnIntegerOverflow on" % _tstr(it)
        elif falling and not asic and dims == 2 and self._lut_declared_wl(p, out_t) == 32 and \
                p.get("RndMeth", "Floor") in ("Floor", "Ceiling", "Simplest", "Nearest"):
            # recordings (8 tables): a declared 32-bit intermediate type with a table type
            # of any other width is wrong under Floor (Zero is equal; declared 24 / 33 / 40 / 64 bits are equal).
            # One recordings module of this shape was equal and is now refused with them
            why = "a falling segment of a 2-D table with a 32-bit intermediate type under %s rounding on this " \
                "hardware" % p.get("RndMeth", "Floor")
        elif falling and not asic and dims == 2 and tt[1] == 32 and \
                p.get("RndMeth", "Floor") in ("Floor", "Ceiling", "Simplest", "Nearest"):
            # the recorded rule specifies "table or intermediate type 32 bits"; recordings hold 55
            # modules with a narrower table type and a 32-bit intermediate type on falling segments, all equal with
            # the signed term -- so only the 32-bit table type (never recorded in 2-D) is refused
            why = "a falling segment of a 2-D table with a 32-bit table type under %s rounding on this hardware" \
                % p.get("RndMeth", "Floor")
        if why and falling and it[0] == 0 and "falling segment" in why and _lu_recorded(p, ts, dims, it, tt, asic):
            # a falling segment with an unsigned intermediate type inside the
            # recorded span (engine/lu_domain.json): host with intermediate = table type uses the magnitude term,
            # otherwise the slope term is cast into the intermediate type (ref/fxcore.py _blk_lut1d; 2-D: _blk_lut2d)
            why = None
        if why:
            raise GraphError("UNSUPPORTED", where, "linear interpolation: %s -- Simulink's slope term there is not "
                             "determined by the recorded data" % why)

    @staticmethod
    def _lut_declared_wl(p, out_t):
        """Word length of the intermediate type as the block declares it (an explicit type, or the output's under
        'Same as output'); None for the internal rule."""
        v = str(p.get("IntermediateResultsDataTypeStr", "Inherit: Same as output")).strip()
        if v == "Inherit: Same as output":
            return out_t[1] if isinstance(out_t, tuple) else None
        if v.startswith("Inherit"):
            return None
        try:
            t = fc._named_type(v)
        except (ValueError, KeyError):
            return None
        return t[1] if isinstance(t, tuple) else None

    def _lut_static_checks(self, p, ts, where, dims):
        """recordings (recordingsrecordings): configurations Simulink
        rejects when the model is updated -- an explicit fraction type that cannot hold the interpolation fraction
        (UseLastTableValue off: fraction length <= word - 2 signed, <= word - 1 unsigned; on: one more; the error text
        of PreLookupFractionOverflow / OnlyExtrapFloatAndHalf), and a breakpoint the breakpoint data type cannot
        represent (LookupNdBreakpointsValueOutOfRange)."""
        if p["InterpMethod"] == "Linear point-slope":
            ft = fc._named_type(str(p.get("FractionDataTypeStr", fc._INHERIT_RULE)))
            if isinstance(ft, tuple):
                fs_, fw_, ff_ = ft
                lim = fw_ - fs_ - (0 if p.get("UseLastTableValue") == "on" else 1)
                if ff_ > lim:
                    raise GraphError("PORT_TYPE_INVALID", where + ".FractionDataTypeStr", "the fraction type %s cannot "
                                     "hold the interpolation fraction with UseLastTableValue %s (fraction length must "
                                     "be <= %d; Simulink:blocks:PreLookupFractionOverflow)"
                                     % (p["FractionDataTypeStr"], p.get("UseLastTableValue", "off"), lim))
        for k in range(1, dims + 1):
            if not ts or ts[k - 1] == BOOL:
                continue
            bt_ = fc._lut_type(p["BreakpointsForDimension%dDataTypeStr" % k], tuple(ts[k - 1]))
            if not isinstance(bt_, tuple):
                continue
            if p["BreakpointsSpecification"] == "Even spacing":
                tbl = fc._vec(p["Table"]) if dims == 1 else (fc._mat(p["Table"]) if k == 1 else fc._mat(p["Table"])[0])
                x0 = float(p["BreakpointsForDimension%dFirstPoint" % k])
                dx = float(p["BreakpointsForDimension%dSpacing" % k])
                bps = [x0 + j * dx for j in range(len(tbl))]
            else:
                if "BreakpointsForDimension%d" % k not in p:   # param fuzz: was an uncaught KeyError
                    raise GraphError("MISSING_FIELD", "%s.BreakpointsForDimension%d" % (where, k), "parameter "
                                     "BreakpointsForDimension%d is required with explicit breakpoint values" % k)
                bps = fc._vec(p["BreakpointsForDimension%d" % k])
            bs, bw, bf = bt_
            lo, hi = (-(1 << (bw - 1)), (1 << (bw - 1)) - 1) if bs else (0, (1 << bw) - 1)
            if p["BreakpointsSpecification"] == "Even spacing":
                # the recorded tolerance is recorded for explicit
                # breakpoints only -- an even-spacing point rounding outside the type gave wrong values (or Simulink
                # LookupNdBreakpointsSpacingDoesNotFitInDerivedDatatype); refused as before recordings (a point that
                # rounds back inside is recorded equal: b-even-e1), as is a Spacing the breakpoint type cannot
                # represent exactly (rare shape: no exact rule chased)
                # exact (2.0 ** bf overflowed -- INTERNAL -- for fraction lengths beyond ~1023)
                sc = Fraction(2) ** bf
                dxs = Fraction(float(p["BreakpointsForDimension%dSpacing" % k])) * sc
                if dxs.denominator != 1 or any(not lo <= round(Fraction(b) * sc) <= hi for b in bps):
                    raise GraphError("PORT_TYPE_INVALID", where + ".BreakpointsForDimension%dSpacing" % k,
                                     "even spacing of dimension %d: every point must round inside the "
                                     "breakpoint data type %s and the spacing must be representable in it (Simulink:"
                                     "blocks:LookupNdBreakpointsSpacingDoesNotFitInDerivedDatatype where recorded)"
                                     % (k, _tstr(bt_)))
            # Explicit breakpoints less than one LSB outside the range: Simulink accepts and saturates the breakpoint
            # onto the range end (+0.68 / -0.32 LSB), giving a
            # table whose quantised breakpoints are all in range. One LSB or more outside is refused; the recorded
            # rejections (LookupNdBreakpointsValueOutOfRange) are far outside, the edge between is not recorded.
            # exactly one LSB above the maximum Simulink accepts
            # and saturates, as for every parameter; below the minimum the line stays
            if any(not lo - 1 < Fraction(b) * Fraction(2) ** bf <= hi + 1 for b in bps):
                raise GraphError("PORT_TYPE_INVALID", where + ".BreakpointsForDimension%d" % k, ('a breakpoint of dimension %d is one LSB or more below, or '
                    'more than one LSB above, its data type %s (Simulink stops '
                    'on it; the diagnostic identifier is not '
                    'LookupNdBreakpointsValueOutOfRange, which was recorded for even '
                    'spacing only); write the breakpoints within the type or widen it') % (k, _tstr(bt_)))
            if p["BreakpointsSpecification"] != "Even spacing":
                # breakpoints quantised to their type are reported (rounded, or saturated within the
                # one LSB Simulink accepts)
                for j_, b_ in enumerate(bps):
                    _param_note(b_, tuple(bt_), where + ".BreakpointsForDimension%d[%d]" % (k, j_ + 1),
                                getattr(self, "param_notes", None))
        # recordings (N3, 24 of 24): table data outside the table type stops Simulink (fxpParameterOverflow, default
        # diagnostics) on the same line as every other parameter; the engine had saturated it
        if ts and not any(t == BOOL for t in ts[:dims]):
            try:
                if dims == 1:
                    _bq, _bt, _tq, out_t, tt_, _b, _t = fc._lut_setup(p, tuple(ts[0]))
                    vals_ = fc._vec(p["Table"])
                else:
                    _bqs, _tq, out_t = fc._lut2d_setup(p, [tuple(t) for t in ts[:2]])
                    tt_ = fc._lut_type(p["TableDataTypeStr"], out_t)
                    vals_ = [v for r in fc._mat(p["Table"]) for v in r]
            except (ValueError, KeyError, TypeError, IndexError):
                tt_ = None   # reported by the block's own evaluation
            if isinstance(tt_, tuple):
                for v_ in vals_:
                    _param_overflow(v_, tuple(tt_), where + ".Table", "Lookup %s: a table value"
                                    % where.split(".")[-1], mode=self.pom,
                                    notes=getattr(self, "param_notes", None))

    def _lut2d_frac(self, p, xts):
        """Fraction type of a 2-D linear table (explicit, or the inherited rule of ref/fxcore.py)."""
        ft = p.get("FractionDataTypeStr", fc._INHERIT_RULE)
        if not str(ft).startswith("Inherit"):
            return ft
        bqs, tq, out_t = fc._lut2d_setup(p, xts)
        it = fc._lut2d_itype(p, out_t, fc._asic(self.hw))
        return fc.lut2d_inherited_fraction(p, tq, it, fc._asic(self.hw), out_t)

    def _lut1d_frac(self, p, xt):
        """Fraction type of a 1-D linear point-slope table (explicit, or the inherited rule of ref/fxcore.py)."""
        ft = p.get("FractionDataTypeStr", fc._INHERIT_RULE)
        bq, bt_, tq, out_t, tt, _b, _t = fc._lut_setup(p, xt)
        it = fc.lut1d_intermediate(p, out_t, tt, fc._asic(self.hw), tuple(xt))   # refusals of the intermediate rule first
        inh = str(ft).startswith("Inherit")
        if inh:
            ft = fc.lut1d_inherited_fraction(p, tq, it, out_t, tt, fc._asic(self.hw), bt_, tuple(xt))
        # an input much finer than the breakpoints
        return fc.lut1d_fraction_by_gap(ft, inh, xt[2] - bt_[2], p, tq, bool(xt[0]), fc._asic(self.hw),
                                        it[1] if isinstance(it, tuple) else None)

    def _fx_case(self, n, recs, step, key):
        if self.btype[n] in ("Lookup1D", "Lookup2D") and lut_ports(self.params[n]):
            # the step's port values are the table / breakpoints, used in the same
            # step; the breakpoints must be strictly increasing, else Simulink stops the run
            # (SimulinkBlock:Foundation:LookupNonMonotonicErrRuntimeParam, every recorded design with a non-increasing
            # step)
            ts_ = [BOOL if "bool" in r else (r["s"], r["wl"], r["fl"]) for r in recs]
            probe_ = str(key[0]).startswith("probe:")
            lp = lut_ports(self.params[n])
            nu_ = 2 if self.btype[n] == "Lookup2D" else 1
            for c_, at_ in (("B", nu_ + lp.get("T", 0)), ("C", nu_ + lp.get("T", 0) + lp.get("B", 0))):
                if probe_ or c_ not in lp:
                    continue
                t_ = ts_[at_]
                bi = [int(recs[at_ + j]["bin"], 2) - ((1 << t_[1]) if t_[0] and recs[at_ + j]["bin"][0] == "1" else 0)
                      for j in range(lp[c_])]
                if any(bi[j] >= bi[j + 1] for j in range(len(bi) - 1)):
                    raise GraphError("SIM_DIAGNOSTIC", "design.blocks." + n, "at step %d: the breakpoints from the port "
                                     "are not strictly increasing; Simulink stops the simulation "
                                     "(LookupNonMonotonicErrRuntimeParam)" % step)
            if probe_:
                q = _lut_port_values(self.params[n], ts_, None, nu_)
            else:
                ck_ = (n, tuple(r.get("bin") for r in recs[nu_:]))
                cache_ = self.__dict__.setdefault("_lutport_seen", {})
                if ck_ not in cache_:
                    cache_[ck_] = self._lutport_check(n, ts_, recs, step)
                q = cache_[ck_]
            q = dict(q)
            q.update(FXCORE_INJECT.get(self.btype[n], {}))
            return {"op": key[0], "batch": key[1], "block": n, "step": step, "params": q,
                    "hw": self.hw or {}, "ins": recs[:nu_]}
        p = dict(self.params[n])
        p.update(FXCORE_INJECT.get(self.btype[n], {}))
        if self.btype[n] == "Product" and str(p["Inputs"]).strip().isdigit():
            p["Inputs"] = "*" * int(p["Inputs"])
        return {"op": key[0], "batch": key[1], "block": n, "step": step, "params": p,
                "hw": self.hw or {}, "ins": recs}

    def _fx_call(self, route, case):
        kind, name = route
        try:
            if kind == "op":
                return fc._BLOCKS[name](case)
            return fc._arith_with_bool(name, case)
        except fc.ParamUnrecorded as e:
            raise GraphError("UNSUPPORTED", "design.blocks." + case["block"], str(e))
        except fc.LutUnrecorded as e:   # an unrecorded event at this step
            raise GraphError("UNSUPPORTED", "design.blocks." + case["block"], "at step %s: %s" % (case.get("step"), e))
        except fc.SimDiagnostic as e:   # Simulink stops the run here
            raise GraphError("SIM_DIAGNOSTIC", "design.blocks." + case["block"], "at step %s: %s" % (case.get("step"), e))
        except (KeyError, ValueError, ZeroDivisionError) as e:
            raise GraphError("BAD_PARAM", "design.blocks." + case["block"], "block evaluation failed: %s" % e)
        except OverflowError:   # a Constant 'Inf' into int8 stopped with an internal error
            raise GraphError("UNSUPPORTED", "design.blocks." + case["block"], "a parameter value is not finite "
                             "(Inf / NaN) or too large to quantise: not supported")
        except TypeError as e:   # a defect in the evaluator, not a property of the request
            raise GraphError("INTERNAL", "design.blocks." + case["block"], "block evaluation failed: %s" % e)

    def _probe(self, n, ts, route):
        """Output type of an fxcore-evaluated block: evaluate it once on zero-valued inputs."""
        if self.btype[n] == "ExtractBits" and BOOL in ts:
            # reported as a malformed request ("integer required, got None")
            raise GraphError("UNSUPPORTED", "design.blocks." + n, "Extract Bits of a boolean input is not recorded")
        recs = [_rec(t, 0) for t in ts]
        out = self._fx_call(route, self._fx_case(n, recs, 0, ("probe:" + route[1], id(self))))
        if self.btype[n] == "Constant" and out.get("cls") in ("double", "single"):
            # double signals are not simulated (they would span the whole
            # engine); a Constant left at its library type is refused at its OutDataTypeStr with the fix
            raise GraphError("UNSUPPORTED", "design.blocks.%s.OutDataTypeStr" % n, CONST_DOUBLE_MSG
                             % (n, out["cls"]))
        return _unrec_type(out, "design.blocks." + n)

    def _emit_route(self, n, route, ts, ot):
        """C-core instructions for a block evaluated through ref/fxcore.py on the Python path; the two paths
        are checked against each other and against recordings (the regression check --fast)."""
        bt, p = self.btype[n], self.params[n]
        src = self.src[n]
        fm = fc._blk_fm(p) if "RndMeth" in p or "SaturateOnIntegerOverflow" in p else {"rm": "Floor", "om": "Wrap"}
        U8 = (0, 8, 0)
        tb = [U8 if t == BOOL else t for t in ts]  # a boolean behaves as uint8 0/1

        def ct(fl_from, t, wl_from, f=fm):
            tok, ok, f62 = _ctok(fl_from, t, f, wl_from)
            self._vm_need(n, ok, "width")
            self.vm_fast &= f62
            return tok

        def width(bits):
            self._vm_need(n, bits <= VM_BITS, "width")
            self.vm_fast &= bits <= FAST_BITS
        if route == ("op", "blk_constant"):
            v = _unrec(fc._blk_constant({"params": p}))
            width(2 if ot == BOOL else ot[1] + 1)
            self.vm[n] = ("CONST", [n], str(v))
        elif bt == "DelayPort" and _delay_enable_reset(p) and \
                str(p.get("DelayLengthSource", "Dialog")) != "Input port" and \
                str(p.get("InitialConditionSource", "Dialog")) != "Input port":
            # the Delay with an enable port and / or an external reset on the C core (DEN, engine/fxgraph_core.c):
            # the state machine of ref/fxcore.py _blk_delayport, step by step in the core's one-shot run
            enab = str(p.get("ShowEnablePort", "off")).strip() == "on"
            mode = str(p.get("ExternalReset", "None")).strip()
            mcode = {"None": 0, "none": 0, "Level": 1, "Level hold": 2, "Rising": 3, "Falling": 4, "Either": 5}.get(mode)
            dt = ts[0] if ts else None
            ok_ = mcode is not None and isinstance(dt, tuple) and dt[1] <= 64 and ot == dt
            self._vm_need(n, ok_, "no_form")
            if ok_:
                N = int(str(p.get("DelayLength", "2")).strip())
                ic = fc._quant_param(float(fc._const_value(p.get("InitialCondition", "0"))), dt[0], dt[1], dt[2])
                en_src = src[1] if enab else src[0]
                r_src = src[2 if enab else 1] if mcode else src[0]
                width(dt[1] + 1)
                self.vm[n] = ("DEN", [n, src[0], en_src, r_src], "%d %d %d %d" % (N, ic, 1 if enab else 0, mcode))
        elif bt == "DelayPort" and not _delay_enable_reset(p) and (
                str(p.get("DelayLengthSource", "Dialog")) == "Input port" or
                str(p.get("InitialConditionSource", "Dialog")) == "Input port"):
            # the Delay with the length or the initial condition from a port on the C core (DPORT,
            # engine/fxgraph_core.c): ref/fxcore.py _blk_delayport / delayport_source, a ring buffer of the input
            dt = ts[0] if ts else None
            lport = str(p.get("DelayLengthSource", "Dialog")) == "Input port"
            icport = str(p.get("InitialConditionSource", "Dialog")) == "Input port"
            lim_t = str(p.get("DelayLengthUpperLimit", "100")).strip()
            mode = str(p.get("ExternalReset", "None")).strip()
            pdf = str(p.get("PreventDirectFeedthrough", "off")).strip() == "on"
            rising = mode == "Rising"
            dg = 0 if str(p.get("DiagnosticForDelayLength", "None")).strip() == "None" else 1
            ok_ = isinstance(dt, tuple) and dt[1] <= 64 and ot == dt
            if lport:
                ok_ = ok_ and lim_t.isdigit() and isinstance(ts[1], tuple) and ts[1][2] == 0 and \
                    mode in ("None", "none", "Rising")
                if ok_ and (pdf or rising):
                    ok_ = not icport and int(lim_t) <= 64
            self._vm_need(n, ok_, "no_form")
            if ok_:
                ic = fc._quant_param(float(fc._const_value(p.get("InitialCondition", "0"))), dt[0], dt[1], dt[2])
                if lport:
                    lim = int(lim_t)
                    cap = min(lim, 64)
                    xs = src[2] if icport else src[0]
                    rs = src[2] if rising else src[0]
                    rest = "0 %d %d %d %d %d %d %d 0" % (lim, cap, dg, 1 if pdf else 0, 1 if rising else 0,
                                                         1 if icport else 0, ic)
                    self.vm[n] = ("DPORT", [n, src[0], src[1], xs, rs], rest)
                else:
                    L = int(str(p["DelayLength"]).strip())
                    self.vm[n] = ("DPORT", [n, src[0], src[0], src[1], src[0]], "1 0 0 0 0 0 1 %d %d" % (ic, L))
                width(dt[1] + 1)
        elif bt == "ShiftArithmetic" and str(p.get("BitShiftNumberSource", "Dialog")) == "Input port":
            # the shift from a port on the C core (PSHIFT, engine/fxgraph_core.c): the same rule as ref/fxcore.py
            # _blk_shift / _port_shift; a step on which the reference stops or refuses (a shift value of 2^31 or
            # more, DiagnosticForOORShift 'Error' outside the range, a shift outside the answered range) is handed
            # back to the Python path (E FALLBACK), which reports it. A Warning check runs on the Python path (the
            # warning is built from the shift port's log there as well, so either path gives it)
            d_ = str(p.get("BitShiftDirection", "Bidirectional"))
            kt = ts[1] if len(ts) > 1 else None
            ok_ = isinstance(kt, tuple) and kt[2] == 0 and kt[1] <= 32 and isinstance(ts[0], tuple) and \
                ts[0][1] <= 32 and d_ in ("Left", "Right", "Bidirectional")
            self._vm_need(n, ok_, "no_form")
            if ok_:
                rng = str(p.get("_ShiftRange", "")).split()
                mode = 2 if p.get("_ShiftGeneral") else (1 if rng else 0)
                lo_, hi_ = (int(rng[0]), int(rng[1])) if mode == 1 else (0, 0)
                chk = 1 if (str(p.get("CheckOORBitShift", "off")).strip() == "on" and
                            str(p.get("DiagnosticForOORShift", "Warning")).strip() == "Error") else 0
                width(ts[0][1] + 1)
                self.vm[n] = ("PSHIFT", [n, src[0], src[1]], "%d %d %d %d %d %d %d %d %d" % (
                    {"Left": 0, "Right": 1, "Bidirectional": 2}[d_], mode, int(p.get("BinPtShiftNumber", "0")),
                    ts[0][0], ts[0][1], lo_, hi_, -31 if d_ == "Bidirectional" else 0, chk))
        elif bt == "ShiftArithmetic":
            k = int(p["BitShiftNumber"])
            d = p["BitShiftDirection"]
            left = 1 if (d == "Left" or (d == "Bidirectional" and k < 0)) else 0
            width(ts[0][1] + 1)
            self.vm[n] = ("SHIFT", [n, src[0]], "%d %d %d %d" % (left, abs(k), ts[0][0], ts[0][1]))
        elif bt == "LogicalOperator":
            code = {"AND": 0, "OR": 1, "NAND": 2, "NOR": 3, "XOR": 4, "NXOR": 5, "NOT": 6}.get(p["Operator"])
            if code is None:
                raise GraphError("BAD_PARAM", "design.blocks.%s.Operator" % n, "unsupported logical operator")
            if len(src) > 1024:
                self._vm_need(n, False, "ports")   # the C core's LOGIC takes 1 .. 1024 inputs
                return
            self.vm[n] = ("LOGIC", [n], (code, list(src)))
        elif bt == "CompareToConstant":
            if ts[0] == BOOL:
                raise GraphError("UNSUPPORTED", "design.blocks." + n, "boolean input to Compare To Constant")
            q = _quant_param(fc._const_value(p["const"]), *ts[0])
            c = _hid(n, "c")
            width(ts[0][1] + 1)
            self.vm[n] = [("CONST", [c], str(q)), ("REL", [n, src[0], c], "0 0 %d" % RELCODE[p["relop"]])]
        elif bt == "CompareToZero":
            # recorded note: Compare To Zero (and the Detect Rise / Fall lowering built on it) had no C-core form, so
            # every Detect design ran on the Python path only; the value against a stored 0 (a boolean input stays
            # on the Python path)
            # a boolean input is its stored 0 / 1 against 0 (ref/fxcore.py _blk_cmpzero)
            c = _hid(n, "c")
            width(2 if ts[0] == BOOL else ts[0][1] + 1)
            self.vm[n] = [("CONST", [c], "0"), ("REL", [n, src[0], c], "0 0 %d" % RELCODE[p["relop"]])]
        elif bt == "Switch":
            crit = p["Criteria"]
            pre = []
            if ts[0] == BOOL or ts[2] == BOOL:
                # boolean data (ref/fxcore.py _bool_select):
                # a boolean output is the selected value ~= 0, a fixed-point output the selected value cast into it
                (u1c, ca), (u3c, cb), pre = self._vm_bool_data(n, [(src[0], ts[0], "nz1"), (src[2], ts[2], "nz3")],
                                                               ot, ct)
            else:
                (u1c, ca), (u3c, cb) = (src[0], ct(ts[0][2], ot, ts[0][1])), (src[2], ct(ts[2][2], ot, ts[2][1]))
            if ts[1] != BOOL:
                # the control word was never checked against the core's width, so an unsigned 128-bit
                # control >= 2^127 compared as negative on the C path (Compare To Constant already checked it)
                width(ts[1][1] + 1)
            if ts[1] == BOOL or crit == "u2 ~= 0":
                mode, q = 0, 0
            else:
                mode = 1 if crit == "u2 >= Threshold" else 2
                q = _quant_param(float(p["Threshold"]), *ts[1])
            sw = ("SWITCH", [n, u1c, src[1], u3c], "%d %d %s %s" % (mode, q, ca, cb))
            self.vm[n] = pre + [sw] if pre else sw
        elif bt == "MultiportSwitch":
            cty = U8 if ts[0] == BOOL else ts[0]
            if len(ts) - 1 > 1023:
                # 1024 data ports reached
                # the C core and failed there (PROTOCOL "engine: MPSW"); the Python path takes any number
                self._vm_need(n, False, "ports")
                return
            boolparts = None
            if any(t == BOOL for t in ts[1:]):
                # boolean data (ref/fxcore.py _bool_select): all-boolean data any number of
                # ports, mixed with fixed point two data ports (the recorded shape)
                if not all(t == BOOL for t in ts[1:]) and len(ts) - 1 != 2:
                    self._vm_need(n, False, "bool_switch")
                    return
                boolparts = self._vm_bool_data(n, [(x, t, "nz%d" % (k + 1)) for k, (x, t) in enumerate(zip(src[1:], ts[1:]))],
                                               ot, ct)
            if p["DataPortOrder"] == "Specify indices":
                # the port whose index equals the control, else the default port (the last
                # data port, or the additional one). On the C core when the indices are distinct and contiguous and
                # the run cannot stop (DiagnosticForDefault None / Warning, two or more indices): an MPSW with the
                # parts reordered by index and the default port appended last
                try:
                    ix = fc.mps_indices(p)
                except ValueError:
                    ix = []
                extra_ = p.get("DataPortForDefault") == "Additional data port"
                nd_ = len(src) - 1
                strict_ = p.get("DiagnosticForDefault", "None") == "Error"
                # gaps between the indices select the default port too (filled with it); with a stopping run only
                # contiguous indices (a gap value must stop, the core would take the filled part)
                if boolparts is not None or len(ix) < 2 or len(set(ix)) != len(ix) or \
                        max(ix) - min(ix) + 1 > 1022 or \
                        (strict_ and sorted(ix) != list(range(min(ix), min(ix) + len(ix)))) or \
                        p.get("DiagnosticForDefault", "None") not in ("None", "Warning", "Error") or \
                        nd_ != len(ix) + (1 if extra_ else 0) or ts[0] == BOOL or \
                        cty[1] - min(cty[2], 0) > 32 or not -64 <= cty[2] <= 64:
                    self._vm_need(n, False, "no_form")
                    return
                width(cty[1] + max(0, -cty[2]) + 2)
                if p.get("DiagnosticForDefault", "None") == "Error" and n not in self.runtime_diag:
                    self.runtime_diag.append(n)   # the C core stops too: the instruction is emitted strict
                lo_ = min(ix)
                span_ = max(ix) - lo_ + 1
                dflt = nd_ - 1   # the last data port, or the additional default port
                order = [dflt] * span_
                for j, v in enumerate(ix):
                    order[v - lo_] = j
                parts = [(src[1 + j], ct(ts[1 + j][2], ot, ts[1 + j][1])) for j in order]
                parts.append((src[1 + dflt], ct(ts[1 + dflt][2], ot, ts[1 + dflt][1])))
                self.vm[n] = ("MPSW", [n, src[0]], (cty[2], lo_, span_, parts))
                return
            base = 0 if p["DataPortOrder"].startswith("Zero") else 1
            # on the host Simulink wraps a control wider than 32 bits modulo
            # 2^32 before indexing (the engine indexed with the exact value: silent wrong); ASIC/FPGA uses the exact
            # value for wide words (recorded: ufix40) but a negative fraction length is reported to wrap there too
            # -- controls whose integer span exceeds 32 bits are refused, except the recorded ASIC form
            # ASIC/FPGA wraps too ((2^32 + 1) selects port 1 with ufix40 /
            # sfix40 controls) -- against the recorded one ASIC recording, where the exact value was used; refused on
            # every hardware
            if ts[0] != BOOL and cty[1] - min(cty[2], 0) > 32:
                raise GraphError("UNSUPPORTED", "design.blocks." + n, "Multiport Switch control %s spans more than "
                                 "32 integer bits: Simulink wraps it modulo 2^32 on this hardware (rare shape, "
                                 "refused); narrow the control" % _tstr(cty))
            # DiagnosticForDefault 'Error' (the library default) stops Simulink at the first
            # out-of-range control; one data port stops it whatever the diagnostic. Honoured at run time on the
            # Python path (the C core, the optimiser, the prover and the code generators cannot stop: they refuse)
            diag = p.get("DiagnosticForDefault", "None")
            extra_port = p["DataPortForDefault"] == "Additional data port"
            if diag not in ("None", "Warning", "Error"):
                raise GraphError("UNSUPPORTED", "design.blocks." + n + ".DiagnosticForDefault",
                                 "%r is not recorded (None, Warning or Error)" % diag)
            # 'Warning' = 'None' plus a Simulink warning (identical outputs)
            if int(p["Inputs"]) == 1 and extra_port:
                raise GraphError("PORT_TYPE_INVALID", "design.blocks." + n, "Multiport Switch with one data port "
                                 "and an additional default port: Simulink does not build it")
            if (diag == "Error" or int(p["Inputs"]) == 1) and n not in self.runtime_diag:
                self.runtime_diag.append(n)   # the C core stops too: the instruction is emitted strict
            width(cty[1] + max(0, -cty[2]) + 2)
            # the control is shifted by its fraction length; outside +-64 the C core's shift
            # would be undefined -- such designs stay on the Python path
            self._vm_need(n, -64 <= cty[2] <= 64, "width")
            if boolparts is not None:
                *bp_, pre = boolparts
                self.vm[n] = pre + [("MPSW", [n, src[0]], (cty[2], base, int(p["Inputs"]), list(bp_)))]
            else:
                parts = [(x, ct(t[2], ot, t[1])) for x, t in zip(src[1:], ts[1:])]
                self.vm[n] = ("MPSW", [n, src[0]], (cty[2], base, int(p["Inputs"]), parts))
        elif bt == "Product" or (route[0] == "bool" and route[1] == "Product"):
            rec = self._product_rec(n, p, tb, ot, fm)
            ops = rec.ops
            fmq = dict(fm, rm=rec.rounding_quotient)
            if ops == "**":  # boolean selects: b * x then cast
                full = tb[0][2] + tb[1][2]
                width(tb[0][1] + tb[1][1])
                self.vm[n] = ("PROD", [n, src[0], src[1]], ct(full, ot, tb[0][1] + tb[1][1]))
            else:
                width(_prodn_bits(tb, ops, ot))
                special = 1 if ops == "/*" else 0
                operands = [(x, t[2], 1 if o == "/" else 0) for x, t, o in zip(src, tb, ops)]
                # the C core reads fraction lengths within +-126 (ufix52_En127 gave PROTOCOL)
                self._vm_need(n, all(-126 <= t[2] <= 126 for t in tb) and -126 <= ot[2] <= 126, "width")
                self.vm[n] = ("PRODN", [n], (special, ot[2], RMCODE[fm["rm"]], RMCODE[fmq["rm"]], ot[0], ot[1],
                                             1 if fm["om"] == "Saturate" else 0, operands))
        elif route == ("bool", "Gain"):
            g = float(p["Gain"])
            rec = self._gain_rec(n, g, _named_param_type(p, g, self.hw), BOOL, ot)
            pt, npar = rec.param_type, rec.param_stored
            width(8 + pt[1])
            self.vm[n] = ("GAIN", [n, src[0]], "%d %s" % (npar, ct(pt[2], ot, 8 + pt[1])))
        elif route == ("bool", "Sum"):
            rec = self._sum_rec(n, p, tb, ot, fm, strict=False)
            signs, at, first = list(rec.signs), rec.acc_type, rec.first
            parts = [(x, ct(t[2], at, t[1]), 1 if sg == "+" else -1) for x, t, sg in zip(src, tb, signs)]
            self.vm[n] = ("SUM", [n], (len(tb), -1 if first is None else first, ct(at[2], at, at[1] + 1),
                                      ct(at[2], ot, at[1]), parts))
        elif route == ("bool", "DTC"):
            if str(p.get("ConvertRealWorld", "")).startswith("Stored Integer"):
                self.vm[n] = ("CAST", [n, src[0]], ct(0, (ot[0], ot[1], 0), 8))
            else:
                self.vm[n] = ("CAST", [n, src[0]], ct(0, ot, 8))
        elif bt == "DiscreteFir":
            xt = ts[0]
            if xt == BOOL:
                raise GraphError("UNSUPPORTED", "design.blocks." + n, "boolean input to Discrete FIR")
            coefs = list(self.param_record(n).coefs)
            rec = self._fir_rec(n, p, xt, coefs)
            ctp, pt, at, st, ot2, tt = (rec.coef_type, rec.product_type, rec.acc_type, rec.state_type, rec.out_type,
                                        rec.tapsum_type)
            cq = list(rec.coefs_stored)
            nc = len(coefs)
            if p.get("FilterStructure") == "Direct form transposed":
                # recordings (b01 recordings): the
                # transposed structure keeps its initial states in the accumulator type and Simulink checks them
                # against it -- also with one coefficient (no delay line), where the engine returned before any check
                for v_ in fc._vec(p.get("InitialStates", "0")):
                    _param_overflow(v_, tuple(at), "design.blocks.%s.InitialStates" % n, "Discrete FIR %s: an initial "
                                    "state (in the accumulator type)" % n, mode=self.pom,
                                    notes=getattr(self, "param_notes", None))
            if nc == 1:
                # one coefficient skips the accumulator type (ref/fxcore.py): the product cast to the
                # product type, then straight to the output (any structure; no delay line)
                pk = _hid(n, "p0")
                width(max(xt[1] + ctp[1], pt[1]) + 2)
                self.vm[n] = [("GAIN", [pk, src[0]], "%d %s" % (cq[0], ct(xt[2] + ctp[2], pt, xt[1] + ctp[1]))),
                              ("CAST", [n, pk], ct(pt[2], ot, pt[1]))]
                return
            entries = []
            fs_ = p["FilterStructure"]
            if fs_ == "Direct form transposed":
                # partial sums z_k in the accumulator type: y = p0 + z1,
                # z_k <- p_k + z_(k+1), z_(N-1) <- p_(N-1); products cast to the product type, then to the accumulator
                z = [None] + [_hid(n, "z%d" % k) for k in range(1, nc)]
                nz = [None] + [_hid(n, "nz%d" % k) for k in range(1, nc)]
                vals = fc._vec(p.get("InitialStates", "0"))
                vals = vals * max(nc - 1, 0) if len(vals) == 1 else vals
                zi = [_quant_param(v, *at) for v in vals
                      if _param_overflow(v, at, "design.blocks.%s.InitialStates" % n, "Discrete FIR %s: an initial "
                                         "state" % n, mode=self.pom, notes=getattr(self, "param_notes", None)) is None]
                self.vm_dly[n] = [(z[k], nz[k], zi[k - 1]) for k in range(1, nc)]
                self.slot_types.update({x: at for x in z[1:] + nz[1:]})
                width(max(xt[1] + ctp[1], at[1] + 2))
                for k in range(nc):
                    entries.append(("GAIN", [_hid(n, "p%d" % k), src[0]],
                                    "%d %s" % (cq[k], ct(xt[2] + ctp[2], pt, xt[1] + ctp[1]))))
                pa = [(_hid(n, "p%d" % k), ct(pt[2], at, pt[1]), 1) for k in range(nc)]
                ident = ct(at[2], at, at[1])
                for k in range(1, nc):
                    parts = [pa[k]] + ([(z[k + 1], ident, 1)] if k < nc - 1 else [])
                    entries.append(("SUM", [nz[k]], (len(parts), 0, ct(at[2], at, at[1] + 1), ident, parts)))
                parts = [pa[0]] + ([(z[1], ident, 1)] if nc > 1 else [])
                entries.append(("SUM", [n], (len(parts), 0, ct(at[2], at, at[1] + 1), ct(at[2], ot, at[1]), parts)))
                self.vm[n] = entries
                return
            if fs_ in ("Direct form symmetric", "Direct form antisymmetric"):
                sg = 1 if fs_ == "Direct form symmetric" else -1
                taps = [src[0]] + [_hid(n, "z%d" % k) for k in range(1, nc)]
                iq = fc._dfir_initial_states(p, xt, nc)
                self.vm_dly[n] = [(taps[k], taps[k - 1], iq[k - 1]) for k in range(1, nc)]
                self.slot_types.update({x: xt for x in taps[1:]})
                width(max(tt[1] + ctp[1], xt[1] + ctp[1], tt[1] + 2))
                parts = []
                for k in range(nc // 2):
                    tk = _hid(n, "t%d" % k)
                    tp = [(taps[k], ct(xt[2], tt, xt[1]), 1), (taps[nc - 1 - k], ct(xt[2], tt, xt[1]), sg)]
                    entries.append(("SUM", [tk], (2, 0, ct(tt[2], tt, tt[1] + 1), ct(tt[2], tt, tt[1]), tp)))
                    pk = _hid(n, "p%d" % k)
                    entries.append(("GAIN", [pk, tk], "%d %s" % (cq[k], ct(tt[2] + ctp[2], pt, tt[1] + ctp[1]))))
                    parts.append((pk, ct(pt[2], at, pt[1]), 1))
                if nc % 2:
                    pk = _hid(n, "p%d" % (nc // 2))
                    entries.append(("GAIN", [pk, taps[nc // 2]],
                                    "%d %s" % (cq[nc // 2], ct(xt[2] + ctp[2], pt, xt[1] + ctp[1]))))
                    parts.append((pk, ct(pt[2], at, pt[1]), 1))
                entries.append(("SUM", [n], (len(parts), 0, ct(at[2], at, at[1] + 1), ct(at[2], ot, at[1]), parts)))
                self.vm[n] = entries
                return
            taps = [src[0]] + [_hid(n, "z%d" % k) for k in range(1, nc)]
            iq = fc._dfir_initial_states(p, xt, nc)  # input-type stored integers
            self.vm_dly[n] = [(taps[k], taps[k - 1], iq[k - 1]) for k in range(1, nc)]
            self.slot_types.update({x: xt for x in taps[1:]})
            width(xt[1] + ctp[1])
            for k in range(nc):
                pk = _hid(n, "p%d" % k)
                entries.append(("GAIN", [pk, taps[k]], "%d %s" % (cq[k], ct(xt[2] + ctp[2], pt, xt[1] + ctp[1]))))
            parts = [(_hid(n, "p%d" % k), ct(pt[2], at, pt[1]), 1) for k in range(nc)]
            entries.append(("SUM", [n], (nc, 0, ct(at[2], at, at[1] + 1), ct(at[2], ot, at[1]), parts)))
            self.vm[n] = entries
        elif bt in ("CounterLimited", "CounterFreeRunning"):
            # state n (a one-step delay of nxt, initial 0); nxt = (n == lim) ? 0 : n + 1
            rec = self._counter_rec(n, bt, p, ot)
            w = rec.out_type[1]
            lim = rec.limit
            h = {t: _hid(n, t) for t in HIDDEN_P4B2[bt]}
            at = (0, w + 1, 0)
            width(w + 2)
            self.vm[n] = [("CONST", [h["one"]], "1"), ("CONST", [h["lim"]], str(lim)), ("CONST", [h["zero"]], "0"),
                          ("REL", [h["eq"], n, h["lim"]], "0 0 %d" % RELCODE["=="]),
                          ("SUM", [h["inc"]], (2, 0, ct(0, at, at[1] + 1), ct(0, ot, at[1]),
                                               [(n, ct(0, at, w), 1), (h["one"], ct(0, at, w), 1)])),
                          ("SWITCH", [h["nxt"], h["zero"], h["eq"], h["inc"]], "0 0 %s %s" % (ct(0, ot, w), ct(0, ot, w)))]
            self.vm_dly[n] = [(n, h["nxt"], 0)]
        elif bt in ("DetectChange", "DetectIncrease", "DetectDecrease"):
            rec = self._detect_rec(n)
            xt = rec.in_type
            prev = _hid(n, "prev")
            ic = rec.vinit_stored
            op = rec.relop
            width(xt[1] + 1)
            self.vm[n] = ("REL", [n, src[0], prev], "0 0 %d" % RELCODE[op])
            self.vm_dly[n] = [(prev, src[0], ic)]
        elif bt == "DiscreteIntegrator" and (fc.dti_method(p) != "FE" or p.get("ExternalReset", "none") != "none" or
                                             p.get("InitialConditionSource", "internal") != "internal"):
            self._vm_dti_ext(n, p, ts, ot, ct, width)
        elif bt == "DiscreteIntegrator":
            # internal rule with K = 2^e: state slope = input slope * K, so the increment is the input's stored
            # integer; y = x; x_next = clamp(ovf(x + u))
            rec = self._dti_rec(n, p, ts[0], ot)
            xt = rec.in_type
            x0, lims = rec.x0_stored, rec.limits_stored
            h = {t: _hid(n, t) for t in HIDDEN_P4B2[bt]}
            # the increment K * u in the output type: K = 2^e makes it the input's stored integer at fraction length
            # xt.fl - e (the internal rule's output has exactly that fraction length; an explicit one rounds into it)
            K_ = fc.dti_gain(p)
            e_ = K_.numerator.bit_length() - 1 if K_.denominator == 1 else -(K_.denominator.bit_length() - 1)
            same = ct(xt[2] - e_, ot, xt[1])
            width(ot[1] + 2)
            ent = [("CAST", [h["inc"], src[0]], same),
                   ("SUM", [h["sum"]], (2, 0, ct(ot[2], ot, ot[1] + 1), ct(ot[2], ot, ot[1]),
                                        [(n, ct(ot[2], ot, ot[1]), 1), (h["inc"], ct(ot[2], ot, ot[1]), 1)]))]
            if lims is None:
                ent.append(("COPY", [h["nxt"], h["sum"]], ""))
            else:
                ent.append(("SAT", [h["nxt"], h["sum"]], "%d %d" % lims))
            self.vm[n] = ent
            self.vm_dly[n] = [(n, h["nxt"], x0)]
        elif bt == "Signum":
            xt = ts[0]
            h = {t: _hid(n, t) for t in HIDDEN_P4B2[bt]}
            sat = {"rm": "Floor", "om": "Saturate"}
            width(xt[1] + 2)
            self.vm[n] = [("CONST", [h["zero"]], "0"),
                          ("REL", [h["pos"], src[0], h["zero"]], "0 0 %d" % RELCODE[">"]),
                          ("REL", [h["neg"], src[0], h["zero"]], "0 0 %d" % RELCODE["<"]),
                          ("SUM", [n], (2, 0, ct(0, (1, 3, 0), 4, sat), ct(0, ot, 3, sat),
                                        [(h["pos"], ct(0, (1, 3, 0), 1, sat), 1), (h["neg"], ct(0, (1, 3, 0), 1, sat), -1)]))]
        elif bt == "Rounding":
            self.vm[n] = ("COPY", [n, src[0]], "")  # integer / fixed-point inputs pass unchanged
        elif bt == "BitwiseOperator":
            xt = ts[0]
            if any(t == BOOL for t in ts):
                raise GraphError("UNSUPPORTED", "design.blocks." + n, "boolean input to Bitwise Operator")
            code = {"AND": 0, "OR": 1, "NAND": 2, "NOR": 3, "XOR": 4, "NOT": 5}.get(p["logicop"])
            if code is None:
                raise GraphError("BAD_PARAM", "design.blocks.%s.logicop" % n, "AND, OR, NAND, NOR, XOR or NOT")
            um = 1 if p.get("UseBitMask") == "on" else 0
            mask = 0
            if um:
                try:
                    mask = fc.bitmask_stored(p, *xt)
                except fc.ParamUnrecorded as e:
                    raise GraphError("UNSUPPORTED", "design.blocks.%s.BitMask" % n, str(e))
                except ValueError as e:
                    raise GraphError("BAD_PARAM", "design.blocks.%s.BitMask" % n, str(e))
            width(xt[1] + 1)
            self.vm[n] = ("BITW", [n], (code, xt[0], xt[1], um, mask, list(src)))
        elif bt == "ExtractBits":
            xt = ts[0]
            try:
                lo, nb = fc._extract_field(p, xt[0], xt[1])
            except fc.ParamUnrecorded as e:
                raise GraphError("UNSUPPORTED", "design.blocks." + n, str(e))
            if lo < 0:
                self._vm_need(n, False, "extract_below0")   # a field below bit 0: Python path only
                return
            width(max(xt[1], lo + nb) + 1)
            self.vm[n] = ("EXTR", [n, src[0]], "%d %d %d" % (lo, nb, xt[0]))
        elif bt in ("Lookup1D", "Lookup2D") and lut_ports(p):
            self._vm_need(n, False, "no_form")   # table / breakpoints from ports -- Python path only
        elif bt == "Lookup1D" and p["InterpMethod"] == "Linear point-slope":
            # integer point-slope instruction (same steps as ref/fxcore.py _blk_lut1d); a double intermediate
            # type stays on the Python path
            xt = ts[0]
            bq, bt_, tq, out_t, tt, _b, _t = fc._lut_setup(p, xt)
            it = fc.lut1d_intermediate(p, out_t, tt, fc._asic(self.hw), tuple(xt) if isinstance(xt, tuple) else None)
            if it == "double" or out_t == "double":
                self._vm_need(n, False, "lut_double")
            else:
                ft = self._lut1d_frac(p, tuple(xt))
                fs_, fw_, ff_ = fc._named_type(ft)
                F = max(xt[2], bt_[2])
                sa, sb = F - xt[2], F - bt_[2]
                tab = [fc._quant_fraction(v, it[0], it[1], it[2], fm) for v in tq]
                bints = [int(b * (Fraction(2) ** bt_[2])) for b in bq]
                outc = ct(it[2], ot, it[1])
                # slope from the table values as stored: at max(table, intermediate) fraction, the product
                # then divided by 2^(ff + dsh)
                self._vm_need(n, isinstance(tt, tuple), "lut_double_table")   # a double table type has no integer steps: Python path
                slopes, dsh = _lut_slopes([tq], tt if isinstance(tt, tuple) else it, it)
                swid = max(abs(v) for v in slopes[0]).bit_length() + 2 if slopes[0] else 2
                width(max(xt[1] + sa, bt_[1] + sb, swid + fw_ + 2, ff_ + max(xt[1] + sa, bt_[1] + sb)) + 2)
                self._vm_need(n, ff_ <= 62 and fw_ <= 64 and ff_ + dsh <= 120, "width")
                # a falling segment on an unsigned intermediate type outside the magnitude
                # case casts the slope term into the intermediate type (ref/fxcore.py _blk_lut1d) -- not in the C core
                self._vm_need(n, not (isinstance(it, tuple) and it[0] == 0 and any(v < 0 for v in slopes[0]) and
                                      not fc.lut1d_magnitude_slope(tt, it, fc._asic(self.hw))), "lut_unsigned_falling")
                self.vm[n] = ("LUTL", [n, src[0]], _plan(LP.LutLinear, 
                    block=n, in_type=tuple(xt), bp_type=tuple(bt_), table_type=tt, intermediate=tuple(it),
                    intermediate_source=_lut_it_source(p), intermediate_declared=_lut_declared(p, "IntermediateResultsDataTypeStr"),
                    frac_type=(fs_, fw_, ff_), frac_source=_lut_frac_source(p), frac_declared=_lut_declared(p, "FractionDataTypeStr"),
                    out_type=tuple(ot), out_cast=outc, rounding=fm["rm"], saturate=fm["om"] == "Saturate",
                    magnitude_slope=fc.lut1d_magnitude_slope(tt, it, fc._asic(self.hw)),
                    frac_carry=fc.lut1d_frac_carry(p, ff_, bt_, fc._asic(self.hw)) is True,   
                    frac_round1_stop=fc.lut1d_frac_carry(p, ff_, bt_, fc._asic(self.hw)) == "refuse",
                    use_last=p.get("UseLastTableValue") == "on", sa=sa, sb=sb, bps=bints, tab=tab, slopes=slopes[0],
                    dsh=dsh, table_conv=_table_conv(tq, it, fm["rm"])))
        elif bt == "Lookup2D":
            # integer 2-D table instruction (same steps as ref/fxcore.py _blk_lut2d), inherited fraction types
            # included (_lut2d_frac); only a double intermediate or output type stays on the Python path
            if BOOL in ts[:2] or ot == BOOL:
                self._vm_need(n, False, "lut2d_bool")
                return
            bqs, tq, out_t = fc._lut2d_setup(p, [tuple(t) for t in ts[:2]])
            mode = _lut_mode(p)
            bpts = tuple(tuple(fc._lut_type(p["BreakpointsForDimension%dDataTypeStr" % dim], tuple(ts[dim - 1])))
                         for dim in (1, 2))
            sh, bints = [], []
            for dim in (1, 2):
                xt, bt_ = ts[dim - 1], bpts[dim - 1]
                F = max(xt[2], bt_[2])
                sh += [F - xt[2], F - bt_[2]]
                bints.append([int(b * (Fraction(2) ** bt_[2])) for b in bqs[dim - 1]])
                width(max(xt[1] + F - xt[2], bt_[1] + F - bt_[2]) + 2)
            ttype = fc._lut_type(p["TableDataTypeStr"], out_t)
            common = dict(block=n, method=p["InterpMethod"], in_types=(tuple(ts[0]), tuple(ts[1])), bp_types=bpts,
                          table_type=ttype, out_type=tuple(ot), shifts=tuple(sh), bps=tuple(bints))
            if mode < 2:
                ffm = fc.lut_flat_fm(p, ttype, fm)   # a double table under Simplest rounds toward zero
                tab = [fc._quant_fraction(v, ot[0], ot[1], ot[2], ffm) for r in tq for v in r]
                rec = _plan(LP.Lut2D, tab=tab, table_conv=_table_conv([v for r in tq for v in r], ot, ffm["rm"]), **common)
            else:
                it = fc._lut2d_itype(p, out_t, fc._asic(self.hw))
                if it == "double" or out_t == "double":
                    self._vm_need(n, False, "lut_double")
                    return
                ft = self._lut2d_frac(p, [tuple(t) for t in ts[:2]])
                fs_, fw_, ff_ = fc._named_type(ft)
                tab = [fc._quant_fraction(v, it[0], it[1], it[2], fm) for r in tq for v in r]
                self._vm_need(n, isinstance(ttype, tuple), "lut_double_table")
                tabT, dsh2 = _lut_table_ints(tq, ttype if isinstance(ttype, tuple) else it, it)
                self._vm_need(n, ff_ + dsh2 <= 120, "width")
                width(fw_ + max(abs(v) for v in tabT).bit_length() + 4)
                outc = ct(it[2], ot, it[1])
                for d_ in (0, 1):
                    xt = ts[d_]
                    width(ff_ + max(xt[1] + sh[2 * d_], bpts[d_][1] + sh[2 * d_ + 1]) + 4)
                width(fw_ + it[1] + 3)
                self._vm_need(n, ff_ <= 62 and fw_ <= 64, "width")
                rec = _plan(LP.Lut2D, tab=tab, intermediate=tuple(it), intermediate_source=_lut_it_source(p),
                               intermediate_declared=_lut_declared(p, "IntermediateResultsDataTypeStr"),
                               frac_type=(fs_, fw_, ff_), frac_source=_lut_frac_source(p),
                               frac_declared=_lut_declared(p, "FractionDataTypeStr"), out_cast=outc,
                               rounding=fm["rm"], saturate=fm["om"] == "Saturate",
                               use_last=p.get("UseLastTableValue") == "on", tab_steps=tabT, dsh=dsh2,
                               table_conv=_table_conv([v for r in tq for v in r], it, fm["rm"]), **common)
            self.vm[n] = ("LUT2", [n, src[0], src[1]], rec)
        elif bt == "Lookup1D":
            xt = ts[0]
            bq, bt_, tq, out_t, tt, _b, _t = fc._lut_setup(p, xt)
            F = max(xt[2], bt_[2])
            sa, sb = F - xt[2], F - bt_[2]
            outs = [fc._quant_fraction(v, ot[0], ot[1], ot[2], fc.lut_flat_fm(p, tt, fm)) for v in tq]   
            bints = [int(b * (Fraction(2) ** bt_[2])) for b in bq]
            width(max(xt[1] + sa, bt_[1] + sb) + 2)
            self.vm[n] = ("LUT", [n, src[0]], _plan(LP.LutIndex, block=n, method=p["InterpMethod"], in_type=tuple(xt),
                                                          bp_type=tuple(bt_), table_type=tt, out_type=tuple(ot),
                                                          sa=sa, sb=sb, bps=bints, outs=outs,
                                                          table_conv=_table_conv(tq, ot, fm["rm"])))
        elif bt == "InterpolationUsingPrelookup":
            self._vm_ipl(n, p, ts, ot, width)
        elif bt in ("Prelookup", "PrelookupF") and ts[0] != BOOL:
            # C-core form of Prelookup (instruction PLK): ref/fxcore.py _prelookup_kf -- the clipped input against the
            # breakpoints at a common scale; k = the interval, f = (u - b_k) / (b_k+1 - b_k) floored into the fraction type
            xt = tuple(ts[0])
            bq, ft = fc._prelookup_setup(p, xt, fc._asic(self.hw))
            bt_ = fc._lut_type(str(p.get("BreakpointDataTypeStr", "Inherit: Same as input")), xt)
            F = max(xt[2], bt_[2])
            bints = [int(b * (Fraction(2) ** F)) for b in bq]
            if F < 0 or F - xt[2] > 120 or ft[2] < 0 or ft[2] > 62 or len(bints) < 2:
                self._vm_need(n, False, "width")
                return
            width(max(xt[1] + F - xt[2], max(abs(b).bit_length() for b in bints) + 1) + ft[2] + 3)
            self.vm[n] = ("PLK", [n, src[0]], (0 if bt == "Prelookup" else 1, F - xt[2], ft[2], bints))
        else:
            self._vm_need(n, False, "no_form")  # no C-core form (Python path only)

    def _vm_bool_data(self, n, ports, ot, ct):
        """Data ports of a Switch / Multiport Switch with boolean data, for the C core (ref/fxcore.py _bool_select):
        a boolean output (the boolean won the internal rule, or all data are boolean) is the selected value ~= 0 -- a
        fixed-point port goes through a hidden (u ~= 0) slot, a boolean port as it is, both passed unchanged (0 / 1);
        a fixed-point output casts a boolean as the integer 0 / 1 and a fixed-point port as usual.
        Returns [(slot, cast) per port] + [the instructions to run before the switch]."""
        pre, out = [], []
        z = _hid(n, "bz")
        if ot == BOOL:
            one = (0, 1, 0)
            ident = ct(0, one, 1)
            for x, t, tag in ports:
                if t == BOOL:
                    out.append((x, ident))
                    continue
                if not pre:
                    pre.append(("CONST", [z], "0"))
                nz = _hid(n, tag)
                self._vm_need(n, t[1] + 1 <= VM_BITS, "width")
                pre.append(("REL", [nz, x, z], "0 0 %d" % RELCODE["~="]))
                out.append((nz, ident))
        else:
            for x, t, tag in ports:
                out.append((x, ct(0, ot, 8) if t == BOOL else ct(t[2], ot, t[1])))
        return out + [pre]

    def _vm_dti_ext(self, n, p, ts, ot, ct, width):
        """C-core form of the Discrete-Time Integrator's Backward Euler / Trapezoidal methods and of Forward Euler with
        an external reset and / or an external initial condition: the same steps as ref/fxcore.py _blk_dti, from the
        existing instructions. The increment round(K u / slope) is a cast of the input when K u / slope is the input's
        stored integer times a power of two (otherwise: Python path only)."""
        src = self.src[n]
        meth = fc.dti_method(p)
        rmode = str(p.get("ExternalReset", "none"))
        ext = str(p.get("InitialConditionSource", "internal")) == "external"
        xt = tuple(ts[0])
        if xt == BOOL or not isinstance(ot, tuple):
            self._vm_need(n, False, "no_form")
            return
        s_, w_, frec, slope = fc._dti_type(p, xt)
        if (s_, w_, frec) != tuple(ot) or slope != Fraction(2) ** (-frec):
            self._vm_need(n, False, "no_form")   # a slope that is not a power of two
            return
        K = fc.dti_gain(p) / (2 if meth == "TR" else 1)
        r = K * (Fraction(2) ** (-xt[2])) / slope   # the increment per stored input LSB
        if r <= 0 or r.numerator & (r.numerator - 1) or r.denominator & (r.denominator - 1):
            self._vm_need(n, False, "no_form")
            return
        g = r.numerator.bit_length() - 1 if r.denominator == 1 else -(r.denominator.bit_length() - 1)
        rec = self._dti_rec(n, p, xt, ot)
        x0, lims = rec.x0_stored, rec.limits_stored
        h = {t: _hid(n, t) for t in HIDDEN_P4B2["DiscreteIntegrator"]}
        same = ct(ot[2] - g, ot, xt[1])
        idt = ct(ot[2], ot, ot[1])
        width(ot[1] + 2)
        width(xt[1] + max(0, g) + 2)

        def add(dst, a, b):
            return ("SUM", [dst], (2, 0, ct(ot[2], ot, ot[1] + 1), ct(ot[2], ot, ot[1]),
                                   [(a, ct(ot[2], ot, ot[1]), 1), (b, ct(ot[2], ot, ot[1]), 1)]))

        def lim(dst, a):
            return ("COPY", [dst, a], "") if lims is None else ("SAT", [dst, a], "%d %d" % lims)
        ent = [("CAST", [h["inc"], src[0]], same)]
        self.slot_types[h["prev"]] = tuple(ot)
        if meth == "BE" and rmode == "none" and not ext:
            ent += [add(h["sum"], h["prev"], h["inc"]), lim(n, h["sum"])]
            self.vm[n] = ent
            self.vm_dly[n] = [(h["prev"], n, x0)]
            return
        if meth == "TR" and rmode == "none" and not ext:
            ent += [add(h["sum"], h["prev"], h["inc"]), lim(n, h["sum"]), add(h["sum2"], n, h["inc"]),
                    lim(h["nxt"], h["sum2"])]
            self.vm[n] = ent
            self.vm_dly[n] = [(h["prev"], h["nxt"], x0)]
            return
        if meth != "FE":
            self._vm_need(n, False, "no_form")   # BE / TR with a reset or an external initial condition (not recorded)
            return
        ri = 1 if rmode != "none" else None
        xi = (2 if ri else 1) if ext else None
        if ext and tuple(ts[xi]) != tuple(ot):
            self._vm_need(n, False, "no_form")
            return
        if ri is not None and (ts[ri] == BOOL or not isinstance(ts[ri], tuple)) and ts[ri] != BOOL:
            self._vm_need(n, False, "no_form")
            return
        dl = []
        if ext:
            # the state at step 0 is the external initial condition at step 0: a first-step flag selects it
            ent += [("CONST", [h["zero"]], "0"),
                    ("SWITCH", [h["xe"], src[xi], h["first"], h["prev"]], "0 0 %s %s" % (idt, idt))]
            dl.append((h["first"], h["zero"], 1))
            self.slot_types[h["first"]] = (0, 8, 0)
            icv, xcur = src[xi], h["xe"]
        else:
            ent.append(("CONST", [h["ic"]], str(x0)))
            icv, xcur = h["ic"], h["prev"]
        if ri is not None:
            rt = (0, 8, 0) if ts[ri] == BOOL else tuple(ts[ri])
            if not ext:
                ent.append(("CONST", [h["zero"]], "0"))
            width(rt[1] + 1)
            r_, rp = src[ri], h["rp"]
            gt = lambda d, a: ("REL", [d, a, h["zero"]], "0 0 %d" % RELCODE[">"])
            le = lambda d, a: ("REL", [d, a, h["zero"]], "0 0 %d" % RELCODE["<="])
            ne = lambda d, a: ("REL", [d, a, h["zero"]], "0 0 %d" % RELCODE["~="])
            if rmode == "rising":
                ent += [le(h["ta"], rp), gt(h["tb"], r_), ("LOGIC", [h["trig"]], (0, [h["ta"], h["tb"]]))]
            elif rmode == "falling":
                ent += [gt(h["ta"], rp), le(h["tb"], r_), ("LOGIC", [h["trig"]], (0, [h["ta"], h["tb"]]))]
            elif rmode == "either":
                # (rp <= 0 < r) or (rp > 0 >= r) == (rp > 0) xor (r > 0)
                ent += [gt(h["ta"], rp), gt(h["tb"], r_), ("LOGIC", [h["trig"]], (4, [h["ta"], h["tb"]]))]
            elif rmode == "level":
                ent += [ne(h["ta"], rp), ne(h["tb"], r_), ("LOGIC", [h["trig"]], (1, [h["ta"], h["tb"]]))]
            elif rmode == "sampled level":
                ent += [ne(h["trig"], r_)]
            else:
                self._vm_need(n, False, "no_form")
                return
            ent.append(("SWITCH", [n, icv, h["trig"], xcur], "0 0 %s %s" % (idt, idt)))
            dl.append((rp, r_, 0))
            self.slot_types[rp] = rt
        else:
            ent.append(("COPY", [n, xcur], ""))
        ent.append(add(h["nxt"], n, h["inc"]))
        self.vm[n] = ent
        dl.append((h["prev"], h["nxt"], x0))
        self.vm_dly[n] = dl

    def _vm_ipl(self, n, p, ts, ot, width):
        """C-core form of Interpolation Using Prelookup (instruction IPL): the same rule as ref/fxcore.py
        _blk_interp_pl, with the table precomputed in the type each step reads it in -- Flat / Nearest: the table value
        floored and wrapped into the output type; linear point-slope: the table value floored and wrapped into the
        intermediate type (interp_pl_itype), every step y0 + floor(f (y1 - y0)) wrapped into it (y0 - floor(f (y0 -
        y1)) on a falling segment where the Python kernel takes it), then floored and wrapped into the output."""
        src = self.src[n]
        dims = int(str(p["NumberOfTableDimensions"]).strip())
        out_t = fc._named_type(str(p["OutDataTypeStr"]))
        tt = fc._lut_type(str(p.get("TableDataTypeStr", "Inherit: Same as output")), out_t)
        if dims == 1:
            vals = fc._vec(p["Table"])
            table, shape = {(i,): Fraction(str(v)) for i, v in enumerate(vals)}, [len(vals)]
        else:
            table, shape = fc.lutnd_table(p["Table"])
        if dims not in (1, 2) or len(shape) != dims or tuple(out_t) != tuple(ot) or any(t == BOOL for t in ts):
            self._vm_need(n, False, "no_form")
            return
        fts = [tuple(ts[2 * d + 1]) for d in range(dims)]
        tq = {k: Fraction(fc._quant_param(v, *tt)) / (Fraction(2) ** tt[2]) for k, v in table.items()}

        def wf(v, t):   # floor into t, wrap into its word: the stored integer
            s_, w_, f_ = t
            q = (v * (Fraction(2) ** f_)).__floor__() & ((1 << w_) - 1)
            return q - (1 << w_) if s_ and q >= 1 << (w_ - 1) else q
        meth = p.get("InterpMethod", "Linear point-slope")
        mode = {"Flat": 0, "Nearest": 1}.get(meth, 2)
        idx = [tuple(i) for i in (sorted(table) if dims == 1 else
                                  [(a, b) for b in range(shape[1]) for a in range(shape[0])])]
        if mode < 2:
            it = tuple(ot)
            tab = [wf(tq[k], out_t) for k in idx]
            mag = 0
        else:
            it = fc.interp_pl_itype(tt, fts[-1])
            tab = [wf(tq[k], it) for k in idx]
            mag = int(not it[0] and fts[-1][1] < 32 and not fc._asic(self.hw))
        if any(f_[0] != 0 or f_[2] < 0 or f_[2] > 62 for f_ in fts) or it[1] > 64 or ot[1] > 64 or \
                abs(it[2] - ot[2]) > 120:
            self._vm_need(n, False, "width")
            return
        width(max(it[1], ot[1]) + max(f_[2] for f_ in fts) + 4)
        shp = list(shape) + [1] * (2 - dims)
        self.vm[n] = ("IPL", [n] + list(src), (dims, mode, shp, [f_[2] for f_ in fts] + [0] * (2 - dims), it,
                                              tuple(ot), mag, tab))

    def _counter_rec(self, n, bt, p, ot):
        """Build (once per compile) the counter's record; consumers read it through counter_record."""
        import fxrecords as R
        lim = self.param_record(n).limit
        try:
            rec = R.CounterRecord(block=n, kind=bt, limit=lim, out_type=tuple(ot))
        except R.RecordError as e:
            raise GraphError("INTERNAL", "design.blocks." + n, str(e))
        self.compiled_recs = getattr(self, "compiled_recs", {})
        self.compiled_recs[n] = rec
        return rec

    def _detect_rec(self, n):
        import fxrecords as R
        bt, p = self.btype[n], self.params[n]
        xt = self.otype[self.src[n][0]]
        if xt == BOOL:
            raise GraphError("UNSUPPORTED", "design.blocks." + n, "boolean input to %s" % bt)
        try:
            pr = self.param_record(n)
            _param_overflow(pr.vinit, xt, "design.blocks.%s.vinit" % n, "%s %s: vinit" % (bt, n), mode=self.pom, notes=getattr(self, "param_notes", None))
            rec = R.DetectRecord(block=n, kind=bt, relop=pr.relop, vinit=pr.vinit,
                                 vinit_stored=_quant_param(pr.vinit, *xt), in_type=tuple(xt),
                                 out_type=self.otype[n] if self.otype[n] == BOOL else tuple(self.otype[n]))
        except R.RecordError as e:
            raise GraphError("INTERNAL", "design.blocks." + n, str(e))
        self.compiled_recs = getattr(self, "compiled_recs", {})
        self.compiled_recs[n] = rec   # one table of compiled records per design
        return rec

    def _fir_rec(self, n, p, xt, coefs):
        import fxrecords as R
        ctp, pt, at, st, ot2, tt = fc._dfir2_types(p, xt, coefs, self.words)
        # recordings (the native recheck): Simulink refuses a Discrete FIR any of whose types is wider than
        # 128 bits (input 65 bits -> inherited product 130; product, accumulator, output, tap sum 129:
        # SimulinkFixedPoint:util:IllegalFixedPointDataType_too_many_bits); 128-bit types compile
        for what, t_ in (("input", xt), ("coefficient", ctp), ("product", pt), ("accumulator", at), ("state", st),
                         ("output", ot2), ("tap sum", tt)):
            if t_ is not None and t_ != BOOL and t_[1] > 128:
                raise GraphError("PORT_TYPE_INVALID", "design.blocks." + n, "the %s type %s is wider than 128 bits: "
                                 "Simulink refuses the Discrete FIR "
                                 "(SimulinkFixedPoint:util:IllegalFixedPointDataType_too_many_bits)" % (what, _tstr(t_)))
        try:
            rec = R.FirRecord(block=n, in_type=tuple(xt), coef_type=tuple(ctp), product_type=tuple(pt),
                              acc_type=tuple(at), state_type=tuple(st), out_type=tuple(ot2), tapsum_type=tuple(tt),
                              coefs_stored=tuple(_quant_param(c, *ctp) for j_, c in enumerate(coefs)
                                                 if _param_overflow(c, ctp, "design.blocks.%s.Coefficients" % n,
                                                                    "Discrete FIR %s: a coefficient" % n, mode=self.pom, notes=getattr(self, "param_notes", None),
                                                                    note_where=_elem_where("design.blocks.%s.Coefficients"
                                                                                           % n, j_, len(coefs))) is None))
        except R.RecordError as e:
            raise GraphError("INTERNAL", "design.blocks." + n, str(e))
        self.compiled_recs = getattr(self, "compiled_recs", {})
        self.compiled_recs[n] = rec
        # the cost counter's view (fxgopt_lib.inner_wl), derived from the record
        self.fir_types = getattr(self, "fir_types", {})
        self.fir_types[n] = (rec.coef_type, rec.product_type, rec.acc_type, rec.state_type)
        return rec

    def fir_record(self, n):
        rec = getattr(self, "compiled_recs", {}).get(n)
        if rec is None or rec.__class__.__name__ != "FirRecord":
            raise GraphError("INTERNAL", "design.blocks." + n, "FIR record requested before compile")
        return rec

    def _product_rec(self, n, p, ts, ot, fm):
        import fxrecords as R
        ops = str(p["Inputs"]).strip()
        if ops.isdigit():
            ops = "*" * int(ops)
        try:
            rec = R.ProductRecord(block=n, ops=ops, in_types=tuple(tuple(t) for t in ts),
                                  out_type=ot if ot == BOOL else tuple(ot), rounding=fm["rm"],
                                  rounding_quotient="Zero" if p.get("RndMeth") == "Simplest" else fm["rm"],
                                  saturate=fm["om"] == "Saturate")
        except R.RecordError as e:
            raise GraphError("INTERNAL", "design.blocks." + n, str(e))
        self.compiled_recs = getattr(self, "compiled_recs", {})
        self.compiled_recs[n] = rec
        return rec

    def product_record(self, n):
        rec = getattr(self, "compiled_recs", {}).get(n)
        if rec is None or rec.__class__.__name__ != "ProductRecord":
            raise GraphError("INTERNAL", "design.blocks." + n, "product record requested before compile")
        return rec

    def _gain_rec(self, n, g, pt, it, ot):
        import fxrecords as R
        try:
            rec = R.GainRecord(block=n, gain=g, param_type=tuple(pt), param_stored=_quant_param(g, *pt),
                               in_type=it if it == BOOL else tuple(it), out_type=ot if ot == BOOL else tuple(ot))
        except R.RecordError as e:
            raise GraphError("INTERNAL", "design.blocks." + n, str(e))
        self.compiled_recs = getattr(self, "compiled_recs", {})
        self.compiled_recs[n] = rec
        return rec

    def gain_record(self, n):
        rec = getattr(self, "compiled_recs", {}).get(n)
        if rec is None or rec.__class__.__name__ != "GainRecord":
            raise GraphError("INTERNAL", "design.blocks." + n, "gain record requested before compile")
        return rec

    def _sum_rec(self, n, p, ts, ot, fm, strict):
        """The Sum's compiled record (built by its kernel from the current types). strict: the main route, where an
        Inherit rule other than "Same as first input" / the internal rule is a BAD_TYPE; the boolean-input route
        takes the internal rule for anything else (as it always did)."""
        import fxrecords as R
        where = "design.blocks." + n
        signs = _sum_signs(p, self.nin[n])
        why1 = fc.sum_one_input_unobserved(p, len(ts), self.words, p.get("_VecReduce") == "1",
                                           max((t[1] for t in ts if isinstance(t, tuple)), default=0))
        if why1:
            raise GraphError("UNSUPPORTED", where, why1)
        at = _fixdt(p["AccumDataTypeStr"], where + ".AccumDataTypeStr")
        src_ = "explicit"
        if at is None:
            if p["AccumDataTypeStr"] == fc._SAME_FIRST:
                at, src_ = ts[0], "same_first"
            elif not strict or p["AccumDataTypeStr"] == fc._INHERIT_RULE:
                why = fc.sum_acc_unobserved(ts, signs, ot, fm["om"] == "Saturate", self.words, fm["rm"]) \
                    if ot != BOOL else None
                if why:
                    raise GraphError("UNSUPPORTED", where + ".AccumDataTypeStr", why + "")
                at, src_ = fc._sum_acc_rule(ts, signs, ot, fm["om"] == "Saturate", self.words), "internal_rule"
            else:
                raise _type_text_error(where + ".AccumDataTypeStr", p.get("AccumDataTypeStr"))
        if strict:
            _chk(at, where + ".AccumDataTypeStr")
        for t_, k in ((at, "AccumDataTypeStr"), (ot, "OutDataTypeStr")):
            if isinstance(t_, tuple) and t_[0] and t_[1] < 2:
                # ASIC's exact-range rule gives a signed one-bit type for '-' of a ufix1 input; Simulink
                # forbids signed one-bit types and what it picks instead is not recorded (fxchain refused it already)
                raise GraphError("UNSUPPORTED_WIDTH", where + "." + k, "the internal rule gives a signed one-bit type "
                                 "%s, which Simulink does not allow; its choice here is not recorded" % _tstr(t_))
        try:
            rec = R.SumRecord(block=n, signs=tuple(signs), in_types=tuple(tuple(t) for t in ts), acc_type=tuple(at),
                              acc_source=src_, out_type=tuple(ot) if ot != BOOL else ot,
                              first=signs.index("+") if "+" in signs else None)
        except R.RecordError as e:
            raise GraphError("INTERNAL", where, str(e))
        self.compiled_recs = getattr(self, "compiled_recs", {})
        self.compiled_recs[n] = rec
        # the cost counter's view (hwcost, fxgopt_lib.inner_wl): derived from the record, never computed apart
        self.acc_types = getattr(self, "acc_types", {})
        self.acc_types[n] = rec.acc_type
        return rec

    def sum_record(self, n):
        rec = getattr(self, "compiled_recs", {}).get(n)
        if rec is None or rec.__class__.__name__ != "SumRecord":
            raise GraphError("INTERNAL", "design.blocks." + n, "sum record requested before compile")
        return rec

    def _dti_rec(self, n, p, xt, ot):
        """Built when the integrator's program is compiled (after its configuration was validated)."""
        import fxrecords as R
        x0, lims = _dti_ints(p, xt, "design.blocks." + n, self.pom, getattr(self, "param_notes", None))
        pr = self.param_record(n)
        try:
            rec = R.DtiRecord(block=n, gain=pr.gain, ic=pr.ic, limits=pr.limits, x0_stored=x0,
                              limits_stored=lims, in_type=tuple(xt), out_type=tuple(ot))
        except R.RecordError as e:
            raise GraphError("INTERNAL", "design.blocks." + n, str(e))
        self.compiled_recs = getattr(self, "compiled_recs", {})
        self.compiled_recs[n] = rec
        return rec

    def dti_record(self, n):
        rec = getattr(self, "compiled_recs", {}).get(n)
        if rec is None or rec.__class__.__name__ != "DtiRecord":
            raise GraphError("INTERNAL", "design.blocks." + n, "integrator record requested before compile")
        return rec

    def detect_record(self, n):
        rec = getattr(self, "compiled_recs", {}).get(n)
        if rec is None or rec.__class__.__name__ != "DetectRecord":
            raise GraphError("INTERNAL", "design.blocks." + n, "detect record requested before compile")
        return rec

    def counter_record(self, n):
        """The compiled CounterRecord of a counter block (built by compile)."""
        rec = getattr(self, "compiled_recs", {}).get(n)
        if rec is None or rec.__class__.__name__ != "CounterRecord":
            raise GraphError("INTERNAL", "design.blocks." + n, "counter record requested before compile")
        return rec

    def lut_record(self, n):
        """The compiled lookup record of block n (engine/fxlutplan.py), or None (not a lookup, or a lookup that has
        no integer form and runs on the Python path)."""
        e = self.vm.get(n)
        for x in (e if isinstance(e, list) else [e] if e else []):
            if x[0] in ("LUT", "LUTL", "LUT2"):
                return x[2]
        return None

    @_guard_block
    def _backprop(self, n):
        if str(self.params[n].get("OutDataTypeStr", "")).strip() != BACKPROP:
            return False
        if self.btype[n] == "Gain":
            # a Gain back-propagated into Outports only (its other destinations are
            # not recorded -- they keep the "not implemented" refusal)
            ds_ = [c for c in self.names for y in self.src[c] if y == n]
            return bool(ds_) and all(self.btype[c] == "Outport" for c in ds_)
        return self.btype[n] in ("Constant", "DataTypeConversion", "Sum", "Product")

    def _accumulator_exits(self, n):
        """The item-c accumulator shape (_backprop_type shape 1, _sf_loop_from_exit shape 2), or None: a two-input Sum,
        one input a Unit Delay that the Sum drives and that drives only the Sum, the other input external; the Sum's
        other destinations are explicit-type Outports and at most one internal-rule Gain / explicit Data Type
        Conversion, or a single explicit Data Type Conversion. Returns (delay, external source, the Outports' types,
        the extra branch's block type or None, the external input's type or None)."""
        if self.btype[n] != "Sum" or self.nin.get(n) != 2 or self.params[n].get("_Tie") is not None or \
                "\x1d" in n:
            return None
        dst = [c for c in self.names for y in self.src[c] if y == n]
        fb = [c for c in dst if self.btype[c] == "UnitDelay" and c in self.src[n]]
        if len(fb) != 1 or dst.count(fb[0]) != 1 or list(self.src[n]).count(fb[0]) != 1 or \
                [c for c in self.names for y in self.src[c] if y == fb[0]] != [n]:
            return None
        ext = [x for x in self.src[n] if x != fb[0]]
        rest = [c for c in dst if c != fb[0]]
        typed, extra = [], []
        for c in rest:
            t_ = str(self.params[c].get("OutDataTypeStr", "")).strip()
            if self.btype[c] == "Outport" and t_.startswith("fixdt"):
                typed.append(_fixdt(t_, "design.blocks.%s.OutDataTypeStr" % c))
            elif self.btype[c] == "Gain" and t_ in ("", fc._INHERIT_RULE):
                extra.append("Gain")
            elif self.btype[c] == "DataTypeConversion" and t_.startswith("fixdt"):
                extra.append("DataTypeConversion")
            else:
                return None
        if len(ext) != 1 or len(extra) > 1 or not (typed or extra == ["DataTypeConversion"]):
            return None
        te = self.otype.get(ext[0])
        return fb[0], ext[0], typed, (extra[0] if extra else None), te

    def _sf_loop_from_exit(self):
        """recorded item c, shape 2 (recordings shape 2 with a typed Outport, 60 designs:
        23 run, 37 rejected, separated by this rule without exception): an internal-rule accumulator Sum (the
        _accumulator_exits shape with typed Outports of one type T). Simulink starts the loop's type resolution from
        the typed exit: the Sum and its delay compile to T where the Sum's own internal rule on (external input, T)
        gives T again; otherwise it rejects the model (InvDisagreeInternalRuleDType / PropForwardDataTypeError).
        Only inside the recorded span (_sf_span); elsewhere the general loop resolution applies as before. Returns
        whether a Sum was typed here."""
        done = False
        for n in self.names:
            if n in self.otype or self.btype[n] != "Sum" or \
                    str(self.params[n].get("OutDataTypeStr", fc._INHERIT_RULE)).strip() != fc._INHERIT_RULE:
                continue
            acc_ = self._accumulator_exits(n)
            if acc_ is None:
                continue
            fb_, ext_, typed_t, extra_, te_ = acc_
            if te_ is None or not typed_t or len(set(typed_t)) != 1 or _sf_span(2, te_, typed_t, extra_,
                                                                              self.params[n], self.words):
                continue
            t_ = typed_t[0]
            p = self.params[n]
            signs = str(p.get("Inputs", "++")).replace("|", "").strip()
            types = [tuple(te_) if x == ext_ else t_ for x in self.src[n]]
            own, _acc = fc._sum_types(dict(p, OutDataTypeStr=fc._INHERIT_RULE,
                                           AccumDataTypeStr=p.get("AccumDataTypeStr", fc._INHERIT_RULE)),
                                      types, signs, fc._blk_fm(p), self.words)
            if tuple(own) != tuple(t_):
                raise GraphError("TYPE_MISMATCH", "design.blocks.%s.OutDataTypeStr" % n, "the feedback loop through %s "
                                 "and %s starts from the typed Outport (%s) and the Sum's internal rule gives %s: "
                                 'Simulink rejects the model (InvDisagreeInternalRuleDType'
                                 ')' % (n, fb_, _tstr(t_), _tstr(tuple(own))))
            self._bp_orig[n] = p
            self.params[n] = dict(p, OutDataTypeStr="fixdt(%d,%d,%d)" % tuple(t_))
            self.otype[n] = tuple(t_)
            done = True
        return done

    def _backprop_type(self, n):
        """recordings: a Constant or Data Type
        Conversion typed 'Inherit via back propagation' takes its type from the one block it drives, by a rule that
        depends on that block:
          Sum (two or three inputs, explicit or internal-rule output), MinMax, Relational Operator (two inputs),
            Switch data port (internal-rule output): the other input's type -- of a three-input Sum, the other
            input in the larger word class, on a tie the lower port;
          Gain and Product (two or three inputs, element-wise) with an explicit output type: that output type;
          an Outport with an explicit type: that type, also through a Unit Delay that drives only that Outport;
          Gain with the internal rule, an untyped Outport, a Switch / Multiport Switch control port: double (not
            simulated, refused).
        The value is then quantised as for an explicit type (Constant: nearest, half up, saturating -- Simulink's
        default diagnostics stop on an overflowing Constant, the engine saturates as for every parameter, recordings;
        Data Type Conversion: its own rounding and overflow mode). Several destinations are refused: the type then
        came from one of them by a rule that depends on the block names only (K1 / K2 -> K2, A / Z -> A; not line
        or block order) and is not determined. Other destinations and fi() values are
        refused too. Returns the type,
        or None while the other input is not typed yet."""
        bt, p = self.btype[n], self.params[n]
        where = "design.blocks.%s.OutDataTypeStr" % n

        def dests(x):
            return [(c, k) for c in self.names for k, y in enumerate(self.src[c]) if y == x]
        dst = dests(n)

        def refuse(why):
            raise GraphError("UNSUPPORTED", where, 'back propagation into %s: %s'
                             '; give %s an explicit type'
                             % ("%s %s" % (self.btype[dst[0][0]], dst[0][0]) if dst else "nothing", why, n))
        outs_all = dst and all(self.btype[cc] == "Outport" for cc, _ in dst)
        typed_ = sorted({str(self.params[cc].get("OutDataTypeStr", "Inherit: auto")).strip() for cc, _ in dst} -
                        {"Inherit: auto", "Inherit: Inherit via back propagation"}) if outs_all else []
        if outs_all and len(dst) > 1 and len(typed_) == 1 and bt in ("Gain", "Sum", "DataTypeConversion", "Constant"):
            # Outports, one of them typed -- the block takes that type
            t_ = _fixdt(typed_[0], where)
            if t_ is None:
                refuse("a typed Outport of type %r among the destinations; not recorded on this path" % typed_[0])
            self._bp_orig[n] = p
            self.params[n] = dict(p, OutDataTypeStr="fixdt(%d,%d,%d)" % t_)
            return t_
        if outs_all and not typed_ and bt in ("Gain", "Sum") and (bt == "Gain" or len(dst) > 1):
            # recordings (Simulink's compiled types): back propagation into untyped Outports only --
            # a Gain takes its input's type, a Sum its inputs' type (recorded with equal input types); it had been
            # refused as "an untyped Outport gives double" (untrue: that is a Constant / Data Type Conversion)
            ins_ = [self.otype.get(x) for x in self.src[n]]
            if any(t_ is None for t_ in ins_):
                return None
            if any(t_ == BOOL or not isinstance(t_, tuple) for t_ in ins_):
                refuse("a boolean or non-fixed-point input; not recorded")
            if bt == "Sum" and len({tuple(t_) for t_ in ins_}) != 1:
                refuse("a Sum with inputs of different types into untyped Outports; not recorded")
            self._bp_orig[n] = p
            self.params[n] = dict(p, OutDataTypeStr="fixdt(%d,%d,%d)" % tuple(ins_[0]))
            return tuple(ins_[0])
        acc_ = self._accumulator_exits(n) if bt == "Sum" else None
        if acc_ is not None:
            # recorded item c, shape 1 (13 designs; recordings
            # shape 1: 80 / 80 run): a back-propagated accumulator -- the Sum feeds a Unit
            # Delay that feeds it back -- whose other destinations are typed Outports (with one typed Outport also an
            # internal-rule Gain or an explicit Data Type Conversion branch), or only an explicit Data Type Conversion
            # before an untyped Outport. Typed Outports of one type: the Sum and its delay compile to that type; of
            # different types Simulink rejects the model (Input/OutputPortDataTypeMismatch); only the conversion: the
            # external input's type. Answered inside the recorded span (_sf_span) only.
            fb_, ext_, typed_t, extra_, te_ = acc_
            if te_ is None:
                return None
            why_ = _sf_span(1, te_, typed_t, extra_, p, self.words)
            if why_:
                refuse(('an accumulator Sum with typed '
                    'exits outside the recorded span: %s') % why_)
            if not typed_t:
                return tuple(te_)
            if len(set(typed_t)) > 1:
                raise GraphError("TYPE_MISMATCH", where, ('the accumulator %s drives typed Outports '
                    'of different types (%s); Simulink rejects '
                    'this (Input/OutputPortDataTypeMismatch)') % (n, ", ".join(_tstr(t_) for t_ in typed_t)))
            return typed_t[0]
        if len(dst) > 1 and bt == "Sum" and self.nin[n] == 2 and \
                str(p.get("Inputs", "++")).replace("|", "").strip() in ("++", "+-"):
            # recordings (99 designs, host and ASIC): an accumulator -- the Sum feeds a Unit Delay
            # that feeds it back -- with an untyped Outport and at most one further destination: an explicit Data
            # Type Conversion, an internal-rule Gain, a second Unit Delay, or a typed Outport. With the typed Outport
            # the Sum takes its type; otherwise its external input's type. (Two typed sinks: a name-order rule,
            # recordings -- refused below.)
            def _expl_(blk_, text_):
                return _fixdt(text_, "design.blocks.%s.OutDataTypeStr" % blk_) if text_ and \
                    not text_.startswith("Inherit") else None
            fb_ = [cc for cc, _ in dst if self.btype[cc] == "UnitDelay" and
                   any(self.src[n][j] == cc for j in range(2))]
            ext_ = [self.src[n][j] for j in range(2) if self.src[n][j] not in fb_]
            rest_ = [cc for cc, _ in dst if cc not in fb_[:1]]
            outs_u = [cc for cc in rest_ if self.btype[cc] == "Outport" and
                      _expl_(cc, str(self.params[cc].get("OutDataTypeStr", "")).strip()) is None]
            outs_t = [cc for cc in rest_ if self.btype[cc] == "Outport" and cc not in outs_u]
            others = [cc for cc in rest_ if cc not in outs_u and cc not in outs_t]
            other_ok = all(
                (self.btype[cc] == "DataTypeConversion" and
                 _expl_(cc, str(self.params[cc].get("OutDataTypeStr", "")).strip()) is not None) or
                (self.btype[cc] == "Gain" and str(self.params[cc].get("OutDataTypeStr", fc._INHERIT_RULE)).strip()
                 == fc._INHERIT_RULE) or
                (self.btype[cc] == "UnitDelay" and not any(self.src[n][j] == cc for j in range(2)))
                for cc in others)
            if len(fb_) == 1 and len(ext_) == 1 and len(outs_u) == 1 and len(outs_t) + len(others) <= 1 and \
                    other_ok and self.params[n].get("_Tie") is None:
                if outs_t:
                    return _expl_(outs_t[0], str(self.params[outs_t[0]].get("OutDataTypeStr", "")).strip())
                te_ = self.otype.get(ext_[0])
                if te_ is None:
                    return None
                if te_ == BOOL or not isinstance(te_, tuple):
                    refuse("a boolean or non-fixed-point accumulator input; not recorded")
                return tuple(te_)
        if len(dst) != 1:
            refuse("%d destinations; which one gives the type is not determined" % len(dst) if dst else
                   "no destination")
        c, k = dst[0]
        if "\x1d" in n or "\x1d" in c or self.params[n].get("_Tie") or self.params[c].get("_Tie"):
            # every back-propagation recording is scalar; a vector source / destination (lowered to
            # element copies) is refused rather than answered element by element
            refuse("a vector signal; the back-propagation rules are recorded for scalar signals only")
        cb, cp = self.btype[c], self.params[c]
        other = [x for j, x in enumerate(self.src[c]) if j != k]
        cot = str(cp.get("OutDataTypeStr", "")).strip()
        if (cb == "MinMax" or (cb == "Sum" and cot in ("", fc._INHERIT_RULE))) and self._bp_reaches_switch(c, dests):
            # recordings (17 + 25 designs, host and ASIC): when the
            # internal-rule Sum / MinMax a back-propagated source feeds drives a Switch or Multiport Switch, Simulink
            # compiles source and block as double (control port) or back-propagates the other data port's type and
            # rejects a disagreement with the internal rule (InvDisagreeInternalRuleDType, data port); the engine
            # gave the other input's type -- wrong values. Not separated: refused
            # what Simulink does differs per shape (compiles the chain as double, back-propagates the
            # other data port's type, or rejects it); the message no longer claims one of them for every design
            refuse("the %s drives a Switch / Multiport Switch (directly or through blocks with inherited types); there "
                   "Simulink may compile the chain as double, take another data port's type or reject the model, and "
                   "which one applies is not recorded for this shape -- give %s or the %s an explicit output type"
                   % (cb, n, cb))

        if cb == "MinMax" and cot in ("", fc._INHERIT_RULE):
            # a back-propagated
            # limit into an internal-rule MinMax whose output feeds back to its own input (a saturating accumulator:
            # Unit Delay -> Sum -> MinMax -> Unit Delay) -- Simulink rejected 5 of 15 such loops
            # (InvDisagreeInternalRuleDType: it back-propagates the delay's type around the loop) while the engine
            # answered; the 10 it ran are not separated from the 5 by a recorded rule -- refused
            todo, seen_ = [c], set()
            while todo:
                x = todo.pop()
                for d_, _k in dests(x):
                    if d_ == c:
                        refuse("the MinMax's output feeds back to its own input (a saturating loop); there Simulink "
                               "back-propagates the loop's type and rejects some of these designs "
                               "(InvDisagreeInternalRuleDType) by a rule that is not recorded -- give "
                               "%s an explicit type (for example the loop's type)" % n)
                    if d_ not in seen_ and self.btype[d_] not in ("Outport",):
                        seen_.add(d_)
                        todo.append(d_)

        def explicit(blk, text):
            return _fixdt(text, "design.blocks.%s.OutDataTypeStr" % blk) if text and not text.startswith("Inherit") \
                else None

        def other_type(xs):
            ts = [self.otype.get(x) for x in xs]
            if any(t is None for t in ts):
                return None
            if any(t == BOOL for t in ts):
                refuse("the other input is boolean; not recorded")
            return tuple(ts[0])
        if bt in ("Sum", "Product"):
            # recordings (two-input Sum / Product): a typed Outport (also through
            # a Unit Delay) and an explicit Gain give their type as for a Constant, but a Product input gives the
            # Product's OTHER input's type (a Constant took the Product's output type); an untyped Outport gave
            # the first input's type, rule not separated -- refused, as is every other destination
            if self.nin[n] != 2 or (bt == "Product" and (str(p.get("Inputs", "2")).strip() not in ("2", "**") or
                                                         str(p.get("Multiplication", "Element-wise(.*)")).startswith("Matrix"))):
                refuse("only a two-input %s has been recorded with this output rule" % bt)
            typed_out = cb == "Outport" and explicit(c, cot) is not None
            unit_typed = cb == "UnitDelay"
            if not (typed_out or unit_typed):
                if cb not in ("Gain", "Product", "Outport"):
                    refuse("an arithmetic block's back propagation into %s is not recorded" % cb)
                if str(p.get("RndMeth", "Floor")).strip() != "Floor" or \
                        str(p.get("SaturateOnIntegerOverflow", "off")).strip() != "off":
                    # the native FO replay in the recorded configuration: every
                    # recorded design of this rule (Sum / Product into a Gain, a Product input or one untyped Outport:
                    # recordings,
                    # recordings -- 325 designs) has RndMeth Floor and saturation off
                    refuse("this rule is recorded with RndMeth Floor and saturation off only; record the setting")
                # grid 2 (18 models) with recordings:
                # into a Gain, a Product input or an untyped Outport the Sum / Product takes its FIRST input's type,
                # unless the second input's word class (the smallest native word, 8 / 16 / 32) is larger -- then the
                # second input's. Host, inputs up to 32 bits
                ins_ = [self.otype.get(x) for x in self.src[n]]
                if any(t_ is None for t_ in ins_):
                    return None
                if any(t_ == BOOL for t_ in ins_) or any(t_[1] > 32 for t_ in ins_) or not self.words:
                    refuse("recorded on the host with fixed-point inputs up to 32 bits only")
                a_, b_ = (tuple(t_) for t_ in ins_)
                t = b_ if word_class(b_[1]) > word_class(a_[1]) else a_
                self._bp_orig[n] = p
                self.params[n] = dict(p, OutDataTypeStr="fixdt(%d,%d,%d)" % t)
                return t
        if cb == "Outport":
            t = explicit(c, cot)
            if t is None:
                refuse("an Outport without an explicit type gives double, which is not simulated")
        elif cb == "UnitDelay":
            nd = dests(c)

            def ty_(x):
                return explicit(x, str(self.params[x].get("OutDataTypeStr", "")).strip())
            # the Unit Delay also feeds back into this Sum (an accumulator); its other two destinations
            fb_ = [x for x, _ in nd if x == n]
            nd2 = [(x, k_) for x, k_ in nd if x != n]
            outs_ = [x for x, _ in nd2 if self.btype[x] == "Outport"]
            gains_ = [x for x, _ in nd2 if self.btype[x] == "Gain"]
            t = None
            if len(nd) == 1 and outs_:
                t = ty_(outs_[0])
            elif len(fb_) == 1 and len(nd2) == 2 and len(outs_) == 2 and all(ty_(x) is not None for x in outs_):
                # recordings (104 models, host and ASIC): two typed Outports of the same type
                # give that type; of different types Simulink stops (Input / OutputPortDataTypeMismatch)
                t1, t2 = ty_(outs_[0]), ty_(outs_[1])
                if t1 != t2:
                    raise GraphError("PORT_TYPE_INVALID", "design.blocks." + c, "Unit Delay %s drives two Outports "
                                     "typed %s and %s; back propagation from the Sum %s cannot satisfy both "
                                     "(Simulink InputPortDataTypeMismatch)" % (c, _tstr(t1), _tstr(t2), n))
                t = t1
            elif len(fb_) == 1 and len(nd2) == 2 and len(outs_) == 1 and len(gains_) == 1 and ty_(outs_[0]) is not None and \
                    ty_(gains_[0]) is not None:
                # (a typed Outport and a Gain with an explicit output type -- the Outport's type, for
                # every Gain type recorded)
                t = ty_(outs_[0])
            if t is None:
                refuse("recorded: a Unit Delay driving one typed Outport, two Outports of one type, or a typed "
                       "Outport and a Gain with an explicit output type")
        elif cb in ("Product", "Gain"):
            t = explicit(c, cot)
            if cb == "Gain" and t is None:
                refuse("a Gain without an explicit output type gives double (internal rule) or was not recorded")
            if cb == "Product" and (t is None or self.nin[c] not in (2, 3) or
                                    str(cp.get("Multiplication", "Element-wise(.*)")).startswith("Matrix") or
                                    "/" in str(cp.get("Inputs", ""))):
                refuse("only a two- or three-input element-wise Product with an explicit output type has been "
                       "recorded")
            if cb == "Product":
                # recordings (834 recorded sources, host and ASIC, two and
                # three inputs): the Sum's word-class rule -- of the other inputs the one in the larger word class
                # (a boolean counts as 8), the lower port on a tie; the explicit output type only when its word
                # class is larger. The engine took the output type (recordings's Product recordings had a class-32
                # output over class-16 inputs, where both agree): recordings, sfix16_En8 for int16, y -103 / 102
                ot_ = [self.otype.get(x) for x in other]
                if any(o_ is None for o_ in ot_):
                    return None
                cls = [8 if o_ == BOOL else word_class(o_[1]) for o_ in ot_]
                if any(o_ != BOOL and o_[1] > 128 for o_ in ot_) or t[1] > 128:
                    refuse("a word over 128 bits: the word-class rule is recorded up to 128 bits")
                k_ = max(range(len(ot_)), key=lambda j: (cls[j], -j))
                wide = [c_ for c_ in cls + [word_class(t[1])] if c_ > 64]
                if len(set(wide)) > 1:
                    # above 64 bits the "class" is the width itself; two different wide candidates
                    # were never compared in a recording (recordings would) -- refused
                    refuse("two candidates wider than 64 bits (%s): their comparison is not recorded"
                           % ", ".join(str(c_) for c_ in sorted(set(wide))))
                if word_class(t[1]) <= cls[k_]:
                    t = ot_[k_]
        elif cb == "Sum":
            if self.nin[c] < 2:
                # a one-input Sum has no other input to take the type from (it was an IndexError)
                refuse("a one-input Sum has no other input; not recorded")
            if cot not in (fc._INHERIT_RULE,) and explicit(c, cot) is None:
                refuse("only an explicit or internal-rule Sum output has been recorded")
            if self.nin[c] > 3:
                refuse("a Sum with %d inputs; up to three have been recorded" % self.nin[c])
            t = other_type(other)
            if t is None:
                return None
            if self.nin[c] == 3:
                # recordings (120 host models: Constant / DTC at port 1 / 2 / 3, other inputs
                # differing in word class, sign and fraction length, both orders, internal-rule and explicit output;
                # the recordings cell on ASIC as well): of the other inputs the one in the larger word class (8 / 16 / 32),
                # on a tie the lower port -- sign and fraction length do not matter; an explicit output in a larger
                # word class than that one gives its type (below, as for two inputs). the recordings cell
                # (ufix7_En1, sfix10_En4 -> sfix10_En4) is where "the lowest port" had been wrong. Recorded other
                # inputs up to 20 bits, outputs up to 30
                ot = [tuple(self.otype.get(x)) for x in other]
                if any(o_[1] > 32 for o_ in ot):
                    refuse("a three-input Sum with an other input over 32 bits is not recorded")
                t = ot[1] if word_class(ot[1][1]) > word_class(ot[0][1]) else ot[0]
            if explicit(c, cot) is not None and t != explicit(c, cot):
                t = self._bp_explicit_out(t, explicit(c, cot), refuse)
            if cot == fc._INHERIT_RULE and not self.words and t != BOOL:
                # recordings (i): on ASIC/FPGA an internal-rule Sum whose output reaches an Outport with an explicit
                # type (directly or through Unit Delays) in a larger word class than the type it gives its
                # back-propagated input: Simulink rejects the model (InvDisagreeInternalRuleDType; recordings,
                # recordings); within the class it is accepted -- the host accepts both
                x_, seen_ = c, set()
                while x_ not in seen_:
                    seen_.add(x_)
                    nd_ = dests(x_)
                    if len(nd_) != 1:
                        break
                    x_ = nd_[0][0]
                    if self.btype[x_] == "Outport":
                        ty_ = explicit(x_, str(self.params[x_].get("OutDataTypeStr", "")).strip())
                        if ty_ is not None and word_class(ty_[1]) > word_class(t[1]):
                            raise GraphError("PORT_TYPE_INVALID", "design.blocks.%s.OutDataTypeStr" % x_,
                                             "the internal-rule Sum %s drives this %s port through back "
                                             "propagation from %s (%s): Simulink rejects the model on ASIC/FPGA "
                                             "(Simulink:DataType:InvDisagreeInternalRuleDType)"
                                             % (c, _tstr(ty_), n, _tstr(t)))
                        break
                    if self.btype[x_] != "UnitDelay":
                        break
        elif cb in ("RelationalOperator", "MinMax"):
            if self.nin[c] != 2:
                refuse("only two inputs have been recorded")
            t = other_type(other)
            if cb == "MinMax" and t is not None and explicit(c, cot) is not None:
                t = self._bp_explicit_out(t, explicit(c, cot), refuse)
        elif cb == "Switch":
            if k == 1:
                refuse("the control port compiles as double, which is not simulated")
            cp0 = getattr(self, "_bp_orig", {}).get(c, cp)
            if str(cp0.get("OutDataTypeStr", fc._INHERIT_RULE)).strip() != fc._INHERIT_RULE:
                refuse("only a Switch with the internal-rule output has been recorded")
            ra = self._reset_acc_input(c, 2 - k)
            if ra is not None:
                # recordings (48 reset accumulators, host and ASIC: Unit Delay -> internal-rule Sum with
                # an input x -> Switch data port, the other data port a back-propagated Constant, Switch -> Unit
                # Delay): Simulink gives the Constant, the Switch and the Unit Delay x's type (39 / 39 run; the Sum
                # keeps its internal rule on (delay, x); 9 stop on the Constant's parameter overflow) -- the Switch's
                # internal rule would have taken the wider Sum type
                t = self.otype.get(ra)
                if t is not None and t != BOOL:
                    self._bp_orig.setdefault(c, cp0)
                    self.params[c] = dict(cp0, OutDataTypeStr="fixdt(%d,%d,%d)" % tuple(t))
            else:
                t = other_type([self.src[c][2 - k]])
        elif cb == "BitwiseOperator":
            # recordings (24 designs, host and ASIC): a back-propagated Constant as one
            # operand of a two-input Bitwise Operator takes the other operand's type
            if self.nin[c] != 2 or str(cp.get("UseBitMask", "off")).strip() == "on":
                refuse("only two-input Bitwise Operators without a mask have been recorded")
            t = other_type(other)
        elif cb == "MultiportSwitch" and k == 0:
            refuse("the control port compiles as double, which is not simulated")
        else:
            refuse("not recorded")
        if t is None:
            return None
        if t == BOOL:
            # recordings (pbool): a boolean other input of a Product wins at word class 8; the source compiles
            # boolean (Constant: value ~= 0; DTC: input ~= 0, as written with 'boolean')
            self._bp_orig[n] = p
            self.params[n] = dict(p, OutDataTypeStr="boolean")
            return BOOL
        t = tuple(t)
        if bt == "Constant":
            v = str(p.get("Value", "")).strip()
            if fc._parse_fi(v) is not None:
                refuse("an fi() value has its own type; not recorded with back propagation")
        elif bt == "DataTypeConversion":
            it = self.otype.get(self.src[n][0])
            if it is None:
                return None
            if it == BOOL or not str(p.get("ConvertRealWorld", "Real World Value (RWV)")).startswith("Real World"):
                refuse("only a real-world-value conversion of a fixed-point input has been recorded")
        self._bp_orig[n] = p
        self.params[n] = dict(p, OutDataTypeStr="fixdt(%d,%d,%d)" % t)
        return t

    def _reset_acc_input(self, w, j):
        """The x of a reset accumulator around Switch w (data port index j holds an internal-rule two-input Sum whose
        inputs are a Unit Delay fed directly by w and x), else None."""
        sm = self.src[w][j]
        if self.btype.get(sm) != "Sum" or self.nin.get(sm) != 2 or \
                str(self.params[sm].get("OutDataTypeStr", fc._INHERIT_RULE)).strip() not in ("", fc._INHERIT_RULE) or \
                "\x1d" in sm or self.params[sm].get("_Tie"):
            return None
        d_, x_ = self.src[sm]
        # recorded: the delay on Sum port 1, Inputs '++' or '+-'
        if self.btype.get(d_) != "UnitDelay" or not self.src[d_] or self.src[d_][0] != w or x_ == d_ or \
                str(self.params[sm].get("Inputs", "++")).replace("|", "").strip() not in ("++", "+-"):
            return None
        return x_

    def _sqrt_check(self, n, ts):
        """recordings (60 designs, host and ASIC): what Simulink rejects is rejected; the exact square
        root into an explicit fixed-point type is answered (190 / 192 steps recomputed exactly; the 2 that differ need
        an input fraction length more than twice the output's); everything else -- Newton-Raphson, rSqrt, the internal
        output rule -- is recorded only in part and refused."""
        p, where = self.params[n], "design.blocks." + n
        op = str(p.get("Operator", "sqrt")).strip()
        # the dialog names SqrtAlgorithmType / RsqrtAlgorithmType; AlgorithmType is a hidden alias
        algs = {str(p[k]).strip() for k in ("AlgorithmType", "SqrtAlgorithmType", "RsqrtAlgorithmType") if k in p}
        if len(algs) > 1:
            raise GraphError("BAD_PARAM", where + ".AlgorithmType", "AlgorithmType and RsqrtAlgorithmType disagree")
        alg = algs.pop() if algs else "Exact"
        ot_ = str(p.get("OutDataTypeStr", "Inherit: Inherit via internal rule")).strip()
        if ot_ == "Inherit: Same as input":
            raise GraphError("PORT_TYPE_INVALID", where + ".OutDataTypeStr", "Sqrt has no 'Inherit: Same as input' "
                             "output option (Simulink SimulinkBlock:Foundation:UdtInvalidValue)")
        if op == "signedSqrt" and ts and ts[0] != BOOL:
            raise GraphError("PORT_TYPE_INVALID", where + ".Operator", "signedSqrt needs a floating-point input or "
                             "output (Simulink BMathSignedSqrtIntermDtpMustBeFloat)")
        if op == "rSqrt" and alg == "Exact":
            raise GraphError("PORT_TYPE_INVALID", where + ".AlgorithmType", "rSqrt does not support the Exact method for "
                             "fixed-point types (Simulink BMathUnsupportedMethod)")
        if op == "rSqrt" and not str(p.get("IntermediateResultsDataTypeStr", "Inherit: Inherit from input")).strip() \
                .startswith("Inherit") and str(p.get("IntermediateResultsDataTypeStr")).strip() not in ("double", "single"):
            raise GraphError("PORT_TYPE_INVALID", where + ".IntermediateResultsDataTypeStr", "rSqrt takes an inherited, "
                             "double or single intermediate type only (Simulink SqrtInverseInvParamSetting)")
        if ts and ts[0] == BOOL:
            raise GraphError("UNSUPPORTED", where, "Sqrt of a boolean input is not recorded")
        if op != "sqrt" or alg != "Exact":
            raise GraphError("UNSUPPORTED", where, "Sqrt with Operator %r and AlgorithmType %r is not supported yet "
                             "(recorded in part only); use Operator sqrt with AlgorithmType Exact" % (op, alg))
        itv = str(p.get("IntermediateResultsDataTypeStr", "Inherit: Inherit via internal rule")).strip()
        if itv != "Inherit: Inherit via internal rule":
            # recordings (Simulink refused a double intermediate type the engine answered), and
            # recorded behaviour (recordings /
            # nr008, read back as SqrtAlgorithmType Exact): with 'Inherit: Inherit from output' / 'from input' Simulink
            # does not give the exactly rounded root (recordings x = 1: 63 against 32) -- only the default internal rule
            # (what every default design reads back) is answered
            raise GraphError("UNSUPPORTED", where + ".IntermediateResultsDataTypeStr", "Sqrt with intermediate type %r is "
                             "not supported (answered with the default 'Inherit: Inherit via internal rule' only; "
                             "Simulink's result with other intermediate types is not the exactly rounded root)" % itv)
        t = _fixdt(ot_, where + ".OutDataTypeStr") if not ot_.startswith("Inherit") else None
        if t is None and ot_ == "Inherit: Same as first input" and ts and isinstance(ts[0], tuple) and \
                tuple(ts[0]) + (str(p.get("RndMeth", "Floor")), bool(fc._asic(self.hw))) in self.SQRT_SAME_FIRST:
            # recordings (the library default output 'Same as first input'): the output is the input
            # type; opened for the exact recorded (input type, rounding, hardware) tuples
            t = tuple(ts[0])
            self.params[n] = dict(p, OutDataTypeStr="fixdt(%d,%d,%d)" % t)
        if t is None:
            raise GraphError("UNSUPPORTED", where + ".OutDataTypeStr", "Sqrt output type %r: only an explicit fixed-point "
                             "type is supported so far (the internal rule is recorded for five types only)" % ot_)
        if ts and ts[0][2] > 2 * t[2] and str(p.get("RndMeth", "Floor")) in ("Nearest", "Convergent", "Round"):
            # with Floor / Zero / Ceiling the answer is the correctly rounded square root
            # also here; with Nearest 15 of the recorded steps differ and no rule explains all of them.
            raise GraphError("UNSUPPORTED", where, ('Sqrt with RndMeth Nearest and an input fraction length '
                "%d above twice the output's (%d): Simulink's rounding "
                'there is not determined; use Floor, Zero or Ceiling, or '
                "give the output at least half the input's fraction bits") % (ts[0][2], t[2]))
        if str(p.get("RndMeth", "Floor")) not in ("Floor", "Zero", "Ceiling", "Nearest", "Convergent", "Round"):
            # Convergent and Round (low): with an input fraction length at most twice the output's the
            # root is never exactly half way between two output steps ((k + 1/2)^2 is not an integer number of
            # 2^-2fo), so both give the Nearest answer recorded in recordings; Simplest is not recorded
            raise GraphError("UNSUPPORTED", where + ".RndMeth", "Sqrt rounding %r is not recorded (Floor, Zero, Ceiling, "
                             "Nearest, and Convergent / Round, which equal Nearest here, are)" % p.get("RndMeth"))

    def _trig_check(self, n, ts):
        """atan2 with CORDIC on two inputs of one signed fixed-point type is
        answered (7 designs bit-exact); Simulink rejects NumberOfIterations equal to the word length
        (CordicWordLengthEqualToNumberIterations). sin / cos / sincos with CORDIC are refused: the recorded steps
        that need a quadrant correction are not reproduced."""
        p, where = self.params[n], "design.blocks." + n
        op, am = str(p.get("Operator", "sin")).strip(), str(p.get("ApproximationMethod", "None")).strip()
        if am == "CORDIC" and op in ("sin", "cos"):
            # recordings (implement from the recordings):
            # one signed fixed-point input of 8 .. 32 bits, 1 .. min(11, w - 2) iterations; iterations >= the word
            # length are rejected by Simulink (CordicIterationsGreaterThanInputWL); angles near a multiple of pi/2
            # are refused at their step, and beyond +-pi/2 outside the recorded fraction lengths w-3 / w-4 or near the
            # type's extremes (fxcore._cordic_sincos, recordings refit)
            if len(ts) != 1 or not isinstance(ts[0], tuple) or not ts[0][0]:
                raise GraphError("UNSUPPORTED", where, "CORDIC %s: one signed fixed-point input is recorded, got %s"
                                 % (op, ", ".join(str(t) for t in ts)))
            try:
                it = int(str(p.get("NumberOfIterations", "11")).strip())
            except ValueError:
                raise GraphError("BAD_PARAM", where + ".NumberOfIterations", "a positive integer")
            w = ts[0][1]
            if it > w:
                raise GraphError("PORT_TYPE_INVALID", where + ".NumberOfIterations", "CORDIC %s with %d iterations on "
                                 "a %d-bit input: Simulink rejects iterations above the word length "
                                 "(CordicIterationsGreaterThanInputWL; 16 on 12 bits recorded, 16 on 16 bits runs)"
                                 % (op, it, w))
            if not 1 <= it <= min(11, w - 2) or not 8 <= w <= 32:
                raise GraphError("UNSUPPORTED", where + ".NumberOfIterations", "CORDIC %s with %d iterations on a "
                                 "%d-bit input: only 8- to 32-bit inputs with 1 .. min(11, word length - 2) "
                                 "iterations are reproduced bit-exactly" % (op, it, w))
            return
        if op != "atan2" or am != "CORDIC":
            raise GraphError("UNSUPPORTED", where, "Trigonometric Function %s with ApproximationMethod %s: atan2, sin "
                             "and cos with CORDIC are supported on fixed point (sincos, the other operators and "
                             "approximations are not)" % (op, am))
        if len(ts) != 2 or not all(isinstance(t, tuple) for t in ts):
            raise GraphError("UNSUPPORTED", where, "Trigonometric Function atan2: two fixed-point inputs are needed, "
                             "got %s" % ", ".join(str(t) for t in ts))
        if ts[0] != ts[1]:
            # recordings (3 designs): Simulink rejects two inputs of different types
            raise GraphError("PORT_TYPE_INVALID", where, "Trigonometric Function atan2: the two inputs have different "
                             "types (%s, %s); Simulink rejects this -- convert them to one type"
                             % (_tstr(ts[0]), _tstr(ts[1])))
        try:
            it = int(str(p.get("NumberOfIterations", "11")).strip())
        except ValueError:
            raise GraphError("BAD_PARAM", where + ".NumberOfIterations", "a positive integer")
        w = ts[0][1]
        if it == w:
            raise GraphError("PORT_TYPE_INVALID", where + ".NumberOfIterations", "CORDIC atan2 with %d iterations "
                             "on a %d-bit input: Simulink rejects iterations equal to the word length "
                             "(CordicWordLengthEqualToNumberIterations)" % (it, w))
        if not 1 <= it <= min(w - 1, 24) or not 8 <= w <= 32:
            # 8- to 32-bit inputs; 27 iterations on 28 bits differ by
            # one LSB on 14 steps (rule not found), 24 on 28 bits and fewer are exact
            raise GraphError("UNSUPPORTED", where + ".NumberOfIterations", "CORDIC atan2 with %d iterations on a "
                             "%d-bit input: only 8- to 32-bit inputs with 1 .. min(word length - 1, 24) iterations "
                             "are reproduced bit-exactly" % (it, w))

    def _bp_reaches_switch(self, c, dests):
        """Whether c's output reaches a Switch / Multiport Switch input, through blocks whose output type is inherited,
        in a shape Simulink does not run in fixed point with the engine's rules (compiled double
        or rejected). recordings (216 designs) answers two shapes: a Data Type Conversion
        with an inherited type ends the chain (the source keeps the other input's type); and on the host an
        internal-rule Sum whose chain to a Switch DATA port passes only Unit Delays, Abs, Saturation, Unary Minus or
        internal-rule Sums (source = the other input's type, the Sum's own rule). Everything else reaching a Switch /
        Multiport Switch stays refused. A block with an explicit output type ends the search."""
        start_sum = self.btype[c] == "Sum"
        todo, seen = [(c, True)], set()
        while todo:
            x, plain = todo.pop()
            if (x, plain) in seen:
                continue
            seen.add((x, plain))
            for d, port in dests(x):
                bd = self.btype[d]
                if bd in ("Switch", "MultiportSwitch"):
                    data_port = bd == "Switch" and port != 1
                    if not (start_sum and plain and data_port and self.words):
                        return True
                    self.__dict__.setdefault("_bp_sw_data", set()).add(d)   # checked once every type is resolved (_resolve_types)
                    continue
                if bd in ("Outport", "Inport"):
                    continue
                ot_ = str(self.params[d].get("OutDataTypeStr", "Inherit")).strip()
                if bd == "DataTypeConversion" and ot_.startswith("Inherit"):
                    continue   # recorded: the conversion ends the back-propagation chain
                if bd in ("UnitDelay", "Delay", "UnaryMinus") or ot_.startswith("Inherit"):
                    keeps = bd in ("UnitDelay", "Delay", "UnaryMinus", "Abs", "Saturation") or \
                        (bd == "Sum" and ot_ == fc._INHERIT_RULE)
                    todo.append((d, plain and keeps))
        return False

    def _bp_explicit_out(self, other_t, out_t, refuse):
        """recordings grid 1 (88 models, with recordings): a
        Constant / DTC into a Sum or MinMax with an explicit output type takes that output type when the output's word
        class (the smallest native word: 8 / 16 / 32) is larger than the other input's, else the other input's type.
        Recorded on the host with signed types of 12 .. 30 bits; the other input winning also with an unsigned other input
        and on ASIC/FPGA; anything else is refused."""
        if tuple(other_t) == tuple(out_t):
            return tuple(out_t)
        if other_t[1] <= 32 and out_t[1] <= 32 and word_class(out_t[1]) <= word_class(other_t[1]):
            # the other input wins also when it is unsigned (ufix12_En4 against
            # fixdt(1,16,8)) and on ASIC/FPGA (sfix12_En4 against fixdt(1,16,8))
            return tuple(other_t)
        if not self.words or not other_t[0] or not out_t[0] or other_t[1] > 32 or out_t[1] > 32:
            refuse("an explicit output type %s unlike the other input's %s: the word-class rule is recorded on the "
                   "host for signed types up to 32 bits only" % (_tstr(out_t), _tstr(other_t)))
        return tuple(out_t) if word_class(out_t[1]) > word_class(other_t[1]) else tuple(other_t)

    def _sum_same_acc(self, n, ts, where):
        """recordings table 1: a Sum's output 'Inherit: Same as accumulator' is the accumulator type
        (internal rule: on the host 32 bits, on ASIC/FPGA the exact range; explicit; 'Same as first input')."""
        p = self.params[n]
        if any(t == BOOL for t in ts):
            raise GraphError("UNSUPPORTED", where + ".OutDataTypeStr", "'Same as accumulator' with a boolean input")
        acc = str(p.get("AccumDataTypeStr", fc._INHERIT_RULE)).strip()
        at = _fixdt(acc, where + ".AccumDataTypeStr") if not acc.startswith("Inherit") else None
        if at is not None:
            return at
        if acc == fc._SAME_FIRST:
            return tuple(ts[0])
        if acc != fc._INHERIT_RULE:
            raise _type_text_error(where + ".AccumDataTypeStr", acc)
        signs = _sum_signs(p, self.nin[n])
        fm = _fm(p, where)
        at = tuple(fc._sum_types(dict(p, OutDataTypeStr=fc._INHERIT_RULE), ts, signs, fm, self.words)[1])
        if tuple(fc._sum_acc_rule(ts, signs, at, fm["om"] == "Saturate", self.words)) != at:
            raise GraphError("UNSUPPORTED", where + ".OutDataTypeStr", "'Same as accumulator' with an internal-rule "
                             "accumulator that depends on the output type here; not recorded")
        return at

    def _double_relop_const(self, n):
        """recordings (120 designs, host and ASIC): a Constant of the default
        type (double) whose only destination is a two-input Relational Operator against a fixed-point signal --
        Simulink compiles the Constant as double and compares the input's real value with it exactly (1440 / 1440
        steps; representable, unrepresentable and out-of-range values alike, no warning). The engine gives the Constant
        an internal type that holds its double value exactly (at least 33 bits, so the comparison is exact on the
        host too) and reports it as double; the other input is checked in the Relational Operator. None when the
        shape does not apply (then the Constant is refused as double, as before)."""
        import re
        p = self.params[n]
        ot_ = str(p.get("OutDataTypeStr", "")).strip()
        if ot_ not in ("", "Inherit: Inherit from 'Constant value'"):
            return None
        txt = str(p.get("Value", "")).strip()
        if not re.match(r"^[-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?$", txt):
            return None
        dst = [(c, k) for c in self.names for k, y in enumerate(self.src[c]) if y == n]
        if len(dst) != 1 or "\x1d" in n:
            return None
        c, k = dst[0]
        if self.btype[c] != "RelationalOperator" or len(self.src[c]) != 2 or self.src[c][1 - k] == n or \
                str(self.params[c].get("OutDataTypeStr", "boolean")).strip() not in ("boolean", "Inherit: Logical"):
            return None
        v = float(txt)
        if v != v or v in (float("inf"), float("-inf")):
            return None
        num, den = v.as_integer_ratio()
        f = den.bit_length() - 1
        w = max(33, abs(num).bit_length() + 1)
        if w > 128:
            raise GraphError("UNSUPPORTED", "design.blocks.%s.Value" % n, "a double Constant %s compared in Relational "
                             "Operator %s needs more than 128 bits to hold exactly; recorded values are within the "
                             "compared signal's range or near it -- give the Constant an explicit fixdt type" % (txt, c))
        self.__dict__.setdefault("dbl_const", set()).add(n)
        # the kernel evaluates the Constant in that type (restored before every compile, as for back propagation)
        self.__dict__.setdefault("_bp_orig", {})[n] = p
        self.params[n] = dict(p, OutDataTypeStr="fixdt(1,%d,%d)" % (w, f))
        return (1, w, f)

    def _out_type(self, n):
        bt, p = self.btype[n], self.params[n]
        where = "design.blocks." + n
        if bt in ("RelationalOperator", "LogicalOperator"):
            t_ = LOGIC_OUT_TYPES.get(str(p.get("OutDataTypeStr", "boolean")).replace(" ", ""))
            if t_ is not None and bt == "LogicalOperator":
                self.params[n] = dict(p, OutDataTypeStr="fixdt(%d,%d,%d)" % t_)   # evaluated by ref/fxcore.py
            return t_ or BOOL
        if self._backprop(n):
            return self._backprop_type(n)
        if bt == "Constant":
            ot_ = str(p.get("OutDataTypeStr", "")).strip()
            if ot_ == "boolean":
                # recordings (24 models, host and ASIC/FPGA): a boolean Constant is true
                # for every non-zero value (2, -1, 0.5 too), false for 0
                if str(p.get("Value", "")).strip().startswith("[") or fc._parse_fi(str(p.get("Value", ""))):
                    raise GraphError("UNSUPPORTED", where + ".Value", "a boolean Constant takes one numeric value")
                return BOOL
            if ot_ in ("single", "half"):
                # 'single' crashed the evaluator (BAD_PARAM "too many values to unpack")
                raise GraphError("UNSUPPORTED", where + ".OutDataTypeStr", "floating-point Constant (%s): fxgraph "
                                 "simulates fixed-point and boolean signals only" % ot_)
            dt = self._double_relop_const(n)
            if dt is not None:
                return dt
            if str(p.get("Value", "")).strip().startswith("["):
                # a list value makes a vector Constant, which the engine does not
                # simulate (scalar signals); this was BAD_PARAM "could not convert string to float"
                raise GraphError("UNSUPPORTED", where + ".Value", "a vector Constant (Value %r) is not supported; "
                                 "use one Constant per element" % str(p.get("Value"))[:60])
            if isinstance(fc._named_type(ot_), tuple):
                # recordings (F6): a Constant typed fixdt(1,1,0) was answered; Simulink rejects a signed one-bit type
                # (IllegalFixedPointDataType_signed_one_bit), as the engine already did for DTC / Sum -- the same
                # range check now runs for the Constant
                _fixdt(ot_, where + ".OutDataTypeStr")
            return self._probe(n, [], self._fx_route(n, []))
        if bt in ("Gain", "Sum", "Product", "MinMax", "DataTypeConversion"):
            t = _fixdt(p.get("OutDataTypeStr"), where + ".OutDataTypeStr") if bt != "DataTypeConversion" \
                else None
            if bt == "DataTypeConversion" and str(p.get("OutDataTypeStr", "")).strip() == "boolean":
                # recordings (40 models): the output is (input != 0) whatever
                # ConvertRealWorld, RndMeth and SaturateOnIntegerOverflow say -- one LSB is true
                return BOOL
            if bt == "DataTypeConversion":
                t = _fixdt(p.get("OutDataTypeStr"), where + ".OutDataTypeStr")
                if t is None:
                    raise GraphError("UNSUPPORTED", where + ".OutDataTypeStr", "DTC needs an explicit fixdt type")
                return t
            if t is not None:
                return t
        # recordings (8 hold loops through a Multiport Switch with an
        # explicit output type): Simulink compiles the loop with that type; the engine waited for the loop-back input
        # and refused the differing data types. A Switch / Multiport Switch with an explicit type starts it as well
        if bt in ("Abs", "Saturation", "Switch", "MultiportSwitch") and \
                not str(p.get("OutDataTypeStr", fc._SAME_INPUT)).strip().startswith("Inherit"):
            # recordings (C3, 20 recorded loops): an explicit output type is known before the input, so it starts
            # an inherited loop as a typed block does (Simulink compiles them); the engine waited for the input and
            # then called the loop inconsistent. The kernel still checks the type against the input once known
            ot_ = str(p.get("OutDataTypeStr")).strip()
            if ot_ in ("double", "single", "half") or (ot_ == "boolean" and bt in ("Abs", "Saturation")):
                # a floating-point output (and boolean outside the recorded all-boolean Switch) ended in BAD_PARAM
                # "block evaluation failed: too many values to unpack" -- a defect message for a valid setting
                raise GraphError("UNSUPPORTED", where + ".OutDataTypeStr", "%s output type %r is not supported: the "
                                 "engine simulates fixed point (and boolean where recorded); give a fixdt type or an "
                                 "'Inherit: ...' rule" % (bt, ot_))
            t = _fixdt(p.get("OutDataTypeStr"), where + ".OutDataTypeStr")
            if t is not None:
                return t
        # rules that only look at the first input resolve as soon as that one is known; this is what lets
        # Simulink break type cycles at a 'Same as first input' block (recordings group C)
        if bt in ("Sum", "Product", "MinMax") and p.get("OutDataTypeStr") == fc._SAME_FIRST and self.src[n]:
            t0 = self.otype.get(self.src[n][0])
            if t0 is not None and t0 != BOOL:
                return t0
        ts = self._in_types(n)
        if ts is None:
            return None
        if bt in ("FIRDecimation", "FIRInterpolation"):
            import fxmultirate
            return fxmultirate.fir_types(p, ts[0], where, fc._asic(self.hw))[3]
        if bt in ("CICDecimation", "CICInterpolation"):
            import fxmultirate
            return fxmultirate.cic_types(p, ts[0], bt == "CICDecimation", where)[1]
        if bt in ("Hold", "Gate"):
            if ts[1] != BOOL:
                raise GraphError("TYPE_MISMATCH", where, "%s needs a boolean enable" % bt)
            return ts[0]
        if bt in ("UnitDelay", "Delay", "Outport") + MR_BLOCKS:
            return ts[0]
        if bt in ("BitSlice", "BitConcat", "BitShift", "BitRotate", "BitSet", "BitClear"):   # recordings (fxhdl)
            import fxhdl
            return fxhdl.bit_out_type(bt, p, ts, where)
        if bt == "MLFn":   
            import fxhdl
            return fxhdl.ml_out_type(p, ts, where)
        if bt in ("PLCore", "PLInterp"):   
            import fxprelookup
            return fxprelookup.out_type(bt, p, ts, where)
        if bt == "HDLFIRCore":   
            import fxhdl
            return fxhdl.fir_out_type(p, ts, where)
        if bt == "CordicCore":   
            import fxcordic
            return fxcordic.out_type(p, ts, where)
        if bt == "NCOCore":   
            import fxhdl
            return fxhdl.nco_kernel(p, ts, where) and fxhdl.nco_out_type(p)
        if bt == "RAMCore":   
            import fxhdl
            return fxhdl.ram_kernel(p, ts, where) and fxhdl.ram_out_type(p, ts, where)
        route = self._fx_route(n, ts)
        if route is not None:
            return self._probe(n, ts, route)
        self._no_bool(n, ts)
        if bt == "UnaryMinus" and not ts[0][0]:
            # Simulink rejects an unsigned input (Simulink:blocks:PortValidationFixptError, recordings)
            raise GraphError("PORT_TYPE_INVALID", where, "Unary Minus needs a signed input, got %s" % _tstr(ts[0]))
        if bt in ("Saturation", "Abs", "UnaryMinus"):
            ot = p.get("OutDataTypeStr", fc._SAME_INPUT)
            if ot == fc._SAME_INPUT:
                return ts[0]
            if bt == "UnaryMinus":  # R2026b's Unary Minus has no output data type parameter
                raise GraphError("UNSUPPORTED", where + ".OutDataTypeStr", "Unary Minus: the output type follows the input")
            t = _fixdt(ot, where + ".OutDataTypeStr")
            if t is None:
                raise GraphError("UNSUPPORTED", where + ".OutDataTypeStr", "%s: 'Same as input' or an explicit fixdt" % bt)
            return t  # explicit output type
        ot = p["OutDataTypeStr"]
        if bt == "Gain":
            if ot == fc._SAME_INPUT:
                return ts[0]
            if ot != fc._INHERIT_RULE:
                raise _out_rule_error(where, ot)
            si, wi, fi_ = ts[0]
            pt = self._gain_param_type(n, ts[0])
            return fc.gain_inherit_out(si, wi, fi_, tuple(pt), self.words)
        if bt == "Product":
            if ot == fc._SAME_FIRST:
                return ts[0]
            if ot != fc._INHERIT_RULE:
                raise _out_rule_error(where, ot)
            t = fc.one_bit_product(ts[0], ts[1], self.words)
            if t is not None:  # a ufix1 factor adds no bits
                return t
            (sa, wa, fa), (sb, wb, fb) = ts
            return (1 if (sa or sb) else 0, fc._native_container(wa + wb, self.words), fa + fb)
        if bt == "Sum":
            if ot == fc._SAME_FIRST:
                return ts[0]
            if ot == SAME_ACC:
                return self._sum_same_acc(n, ts, where)
            if ot != fc._INHERIT_RULE:
                raise _out_rule_error(where, ot)
            # Bound the input fraction length for internal-rule Sums on ASIC/FPGA.
            if not self.words and any(t_ != BOOL and abs(t_[2]) > SUM_RULE_FL_MAX for t_ in ts):
                raise GraphError("UNSUPPORTED", where + ".OutDataTypeStr", "the internal rule on ASIC/FPGA with an input "
                                 "fraction length beyond +-%d is not recorded (Simulink's result differs there); give "
                                 "an explicit output type" % SUM_RULE_FL_MAX)
            # ASIC/FPGA: the output internal rule is the accumulator type (ref/fxcore.py _sum_types)
            return fc._sum_types(p, ts, _sum_signs(p, self.nin[n]), _fm(p, where), self.words)[0]
        if bt == "MinMax":
            if ot == fc._SAME_FIRST:
                return ts[0]
            if ot != fc._INHERIT_RULE:
                raise _out_rule_error(where, ot)
            s = 1 if any(t[0] for t in ts) else 0
            ib = max(fc._int_bits(t[0], t[1], t[2], s) for t in ts)
            w = max(t[1] for t in ts)
            return (s, w, min(max(t[2] for t in ts), w - ib))
        raise GraphError("UNKNOWN_BLOCK_TYPE", where, bt)

    def _gain_param_type(self, n, it):
        p = self.params[n]
        where = "design.blocks.%s.ParamDataTypeStr" % n
        g = _real(p.get("Gain"), "design.blocks.%s.Gain" % n)
        pt = _fixdt(p["ParamDataTypeStr"], where)
        if pt is None:
            if str(p["ParamDataTypeStr"]).strip() == fc._SAME_INPUT:
                # recordings (42 models, host = ASIC): the
                # parameter takes the input type; with the internal-rule output the output is the full-precision
                # product (uint8 -> uint16, sfix8_En4 -> sfix16_En8, sfix16_En8 -> sfix32_En16, uint16 -> uint32).
                # Recorded for 8- and 16-bit inputs; other widths refused. An overflowing gain first, as Simulink's
                # fxpParameterOverflow
                if isinstance(it, (tuple, list)) and len(it) == 3:
                    _param_overflow(g, tuple(it), "design.blocks.%s.Gain" % n, "Gain %s: Gain in the input type "
                                    "('Inherit: Same as input')" % n, mode=self.pom)
                    # recordings (8 more widths: 12- and 32-bit inputs, host and ASIC): the output is the
                    # full-precision product as for 8 / 16 bits -- exact on ASIC, the native container on the host --
                    # which is the internal-rule output with this parameter type
                    if str(p.get("OutDataTypeStr", fc._INHERIT_RULE)).strip() == fc._INHERIT_RULE:
                        return tuple(it)
                    ot_s = str(p.get("OutDataTypeStr", "")).strip()
                    # only a lowered Gain Matrix(K*u) (an element-wise vector Gain also carries _GainVec but is not
                    # recorded with this parameter type; a scalar Gain is refused below)
                    if ("_MatGain" in p or "_ColGain" in p) and json.dumps([list(it), ot_s,
                                                       len(str(p.get("_GainVec", p.get("Gain", ""))).strip().strip("[]")
                                                           .replace(";", " ").replace(",", " ").split()),
                                                       bool(fc._asic(self.hw))]) in _gain_same_domain() and (
                            "_MatGain" in p or json.dumps([list(it), fc._SAME_INPUT, ot_s, str(p.get("RndMeth", "Floor")),
                                                           str(p.get("SaturateOnIntegerOverflow", "off")),
                                                           bool(fc._asic(self.hw)), str(p["_ColGain"])])
                            in _matgain_domain()):
                        # recordings (Gain Matrix(K*u), 40 designs Simulink ran): the parameter takes
                        # the input type and the per-product rule casts into the explicit output -- the engine's rule
                        # equals every recorded step; only the recorded tuples (engine/gain_same_domain.json). A
                        # matrix with rows is held to the exact tuple (rounding, saturation, shape) at its row Sums
                        # (_refuse_unrecorded, engine/matgain_domain.json); an m x 1 matrix on a scalar input has no
                        # row Sum, so its exact tuple is checked here
                        return tuple(it)
                raise GraphError("UNSUPPORTED", where, "Gain %s: ParamDataTypeStr 'Inherit: Same as input' is recorded "
                                 "with the internal-rule output only; use the internal rule or an explicit parameter type"
                                 % n)
            if p["ParamDataTypeStr"] != fc._INHERIT_RULE:
                raise _type_text_error(where + ".ParamDataTypeStr", p["ParamDataTypeStr"])
            gs = [g]
            if "_GainVec" in p:   # an element of a vector Gain: best precision of the whole vector
                gs = [float(v) for v in str(p["_GainVec"]).strip().strip("[]").replace(",", " ").replace(";", " ").split()]
            try:
                pt = (it[0], it[1], min(fc.gain_param_fl(x, it[0], it[1], fc._asic(self.hw),
                                                         p["OutDataTypeStr"] == fc._INHERIT_RULE) for x in gs))
            except ValueError as e:
                raise GraphError("UNSUPPORTED", where, str(e))
        return pt

    # ---- per-block integer kernels: f(list of input stored ints) -> output stored int (or 0/1)
    def tie_rank(self):
        """Order in which Simulink reports events of the same step (12 recorded designs):
        depth first from the Outports in port order, each block after its inputs taken in input-port order -- a
        branch feeding a lower-numbered Outport or Sum input comes first; delays do not pull in their inputs.
        Blocks feeding no Outport follow in execution order. {block: rank}"""
        def port(o):
            try:
                return int(str(self.params[o].get("Port", "")).strip())
            except ValueError:
                return None
        outs = sorted(self.outports, key=lambda o: (port(o) is None, port(o) or 0, o))
        rank, seen = {}, set()
        for o in outs:
            stack = [(o, False)]
            while stack:
                x, done = stack.pop()
                if done:
                    rank.setdefault(x, len(rank))
                    continue
                if x in seen:
                    continue
                seen.add(x)
                stack.append((x, True))
                if self.btype[x] not in DELAYS:
                    for s_ in reversed([s for s in self.src[x] if s is not None]):
                        if s_ not in seen:
                            stack.append((s_, False))
        for x in (getattr(self, "order", None) or self.names):
            rank.setdefault(x, len(rank))
        return rank

    # why a block keeps the design off the C core (fxgraph engine "c"; the error reporting contract: one cause, one block)
    VM_WHY = {"width": "needs > 126-bit intermediates (or fraction lengths beyond the C core's range)",
              "bool_switch": "boolean data on a Switch / Multiport Switch, which the C core does not take",
              "extract_below0": "an Extract Bits field below bit 0, which the C core does not take",
              "lut_double": "a double intermediate or output type in the lookup (no integer steps for the C core)",
              "lut_double_table": "a double table type (no integer steps for the C core)",
              "lut2d_bool": "a boolean input or output of a 2-D Lookup, which the C core does not take",
              "no_form": "a block type without a C-core form (Python path only)",
              "mr_filter": "a multirate filter block (multirate designs run on the Python path)",
              "multirate": "a multirate block (multirate designs run on the Python path)",
              "ports": "more inputs than the C core holds (a Multiport Switch: 1023 data ports, a Logical Operator: "
                       "1024 inputs)"}

    def _vm_need(self, n, ok, why):
        """self.vm_ok &= ok, remembering the first block and cause that turned the C core off."""
        if not ok and self.vm_ok:
            self.vm_why = (n, why)
        self.vm_ok = self.vm_ok and bool(ok)

    @_guard_block
    def _kernel(self, n):
        k = self._kernel_core(n)
        if n in getattr(self, "gated", {}):
            P, o, en, _cnt = self.gated[n]
            new = []
            extra = []
            for i, (x, src, ic) in enumerate(self.vm_dly.get(n, ())):
                tmp = _hid(n, "g%d" % i)
                # the delayed value keeps its own type: a hidden slot (FIR taps, partial sums) has its own entry in
                # slot_types to keep delayed inputs at their own type
                t = self.otype[src] if src in self.otype else getattr(self, 'slot_types', {}).get(src, self.otype[n])
                t = (0, 1, 0) if t == BOOL else t
                ident, _ok, _f = _ctok(t[2], t, {"rm": "Floor", "om": "Wrap"}, t[1])
                extra.append(("SWITCH", [tmp, src, en, x], "0 0 %s %s" % (ident, ident)))
                new.append((x, tmp, ic))
            if extra:
                cur = self.vm.get(n)
                self.vm[n] = (cur if isinstance(cur, list) else ([cur] if cur else [])) + extra
                self.vm_dly[n] = new
            if k is not None and not isinstance(k, _GatedKernel):
                k = _GatedKernel(k, P, o)
        return k

    def _kernel_core(self, n):
        bt, p = self.btype[n], self.params[n]
        where = "design.blocks." + n
        ts = self._in_types(n) or []
        ot = self.otype[n]
        cast = fc._cast
        data_ts = [t for k, t in enumerate(ts) if not ((bt == "Switch" and k == 1) or (bt == "MultiportSwitch" and k == 0)
                                                     or bt in ("DiscreteIntegrator", "DelayPort", "DTIStateFB",
                                                               "DTIStateFBGain", "InterpolationUsingPrelookup") or
                                                     (bt == "ShiftArithmetic" and k == 1) or
                                                     (bt == "Lookup1D" and k >= 1 and lut_ports(p)) or
                                                     (bt == "Lookup2D" and k >= 2 and lut_ports(p)))]   # ports checked apart
        if bt == "LogicalOperator" and str(p.get("AllPortsSameDT", "off")) == "on" and any(t != ot for t in ts):
            # AllPortsSameDT covers the output too -- a NOT of an sfix27_E1 with a boolean output is
            # rejected (InputPortDataTypeMismatch)
            raise GraphError("PORT_TYPE_INVALID", where, "AllPortsSameDT is on but the ports have different types "
                             "(inputs %s, output %s)" % (", ".join(_tstr(t) for t in ts), _tstr(ot)))
        if len(set(data_ts)) > 1 and bt not in INTERNAL_TYPES + ("BitConcat",):
            # the labs' internal blocks (Hold / Gate; recorded MLFn / HDLFIRCore / NCOCore cores, whose ports carry data
            # and boolean valid / enable by design), and Bit Concat, whose inputs of different types are its recorded
            # use (inputs of 1 .. 40 bits, boolean among them, in one block)
            pname, dflt = SAME_DT_PARAM.get(bt, (None, None))
            if bt == "BitwiseOperator":
                # no
                # parameter -- Simulink always requires one input type (SimulinkFixedPoint:util:fxpBitOpInTypeIncompatible)
                raise GraphError("PORT_TYPE_INVALID", where, "Bitwise Operator inputs of different types %s "
                                 "(SimulinkFixedPoint:util:fxpBitOpInTypeIncompatible)" % ", ".join(_tstr(t) for t in data_ts))
            if pname is None:
                # a multi-input block type whose same-type rule is not recorded
                raise GraphError("UNSUPPORTED", where, "%s with data inputs of different types %s: whether Simulink "
                                 "accepts that is not recorded for this block type" % (bt, ", ".join(_tstr(t) for t in data_ts)))
            isd = p.get(pname, dflt)
            if str(isd) == "on":
                # Simulink rejects the model when the inputs of such a block differ in type (InputPortDataTypeMismatch)
                raise GraphError("PORT_TYPE_INVALID", where, "%s is on but the data inputs have different types %s"
                                 % (pname, ", ".join(_tstr(t) for t in data_ts)))
        if bt == "Inport":
            return None
        if bt == "DataTypeConversion" and ot == BOOL:
            # output = (input != 0), conversion settings play no part
            if ts[0] == BOOL:
                self.vm[n] = ("COPY", [n, self.src[n][0]], "")
                return lambda v: v[0]
            c = _hid(n, "z")
            self._vm_need(n, ts[0][1] + 1 <= VM_BITS, "width")
            self.vm_fast &= ts[0][1] + 1 <= FAST_BITS
            self.vm[n] = [("CONST", [c], "0"), ("REL", [n, self.src[n][0], c], "0 0 %d" % RELCODE["~="])]
            return lambda v: 1 if v[0] != 0 else 0
        if bt in ("FIRDecimation", "FIRInterpolation", "CICDecimation", "CICInterpolation"):
            self._vm_need(n, False, "mr_filter")
            return None
        if bt in ("BitSlice", "BitConcat", "BitShift", "BitRotate", "BitSet", "BitClear"):
            # recordings (Bit Set / Clear: BITW; Bit Rotate: Python path only): Python kernel; Bit Shift and Bit Slice also as existing C-core instructions (SHIFT wraps a left
            # shift into the type and shifts right arithmetically; EXTR takes a bit field of the sign-extended
            # stored integer), Bit Concat stays on the Python path
            import fxhdl
            ins = fxhdl.bit_vm(bt, p, ts, n, self.src[n])
            if ins is None:
                self.vm_ok = False
            else:
                self.vm[n] = ins
            return fxhdl.bit_kernel(bt, p, ts)
        if bt in ("PLCore", "PLInterp"):   # Python path only (engine/fxprelookup.py)
            import fxprelookup
            self.vm_ok = False
            return fxprelookup.kernel(bt, p, ts)
        if bt == "CordicCore":   # Python path only (engine/fxcordic.py)
            import fxcordic
            self.vm_ok = False
            return fxcordic.kernel(p, ts)
        if bt == "NCOCore":   # Python path only (engine/fxhdlnco.py)
            import fxhdl
            self.vm_ok = False
            return fxhdl.nco_kernel(p, ts, where)
        if bt == "RAMCore":   # Python path only (engine/fxhdl.py RamKernel)
            import fxhdl
            self.vm_ok = False
            return fxhdl.ram_kernel(p, ts, where)
        if bt == "HDLFIRCore":   # Python path only (engine/fxhdlfir.py)
            import fxhdl
            self.vm_ok = False
            return fxhdl.fir_kernel(p, ts, where)
        if bt == "MLFn":   # Python path only (exact interpreter, engine/fxmlsem.py)
            import fxhdl
            self.vm_ok = False
            return fxhdl.MlKernel(p, ts, where)
        if bt in ("Hold", "Gate"):
            tt = (0, 1, 0) if ot == BOOL else ot
            ident, _ok, _f = _ctok(tt[2], tt, {"rm": "Floor", "om": "Wrap"}, tt[1])
            x, en = self.src[n]
            if bt == "Hold":
                import fxrecords as R
                icv = _real(p["InitialCondition"], where + ".InitialCondition")
                if ot != BOOL:
                    _param_overflow(icv, ot, where + ".InitialCondition", "Hold %s: InitialCondition" % n, mode=self.pom, notes=getattr(self, "param_notes", None))
                rec = R.HoldRecord(block=n, ic_stored=(1 if icv else 0) if ot == BOOL else _quant_param(icv, *ot),
                                   out_type=ot if ot == BOOL else tuple(ot))
                self.compiled_recs = getattr(self, "compiled_recs", {})
                self.compiled_recs[n] = rec
                ic = rec.ic_stored
                hd = _hid(n, "hd")
                self.vm[n] = ("SWITCH", [n, x, en, hd], "0 0 %s %s" % (ident, ident))
                self.vm_dly[n] = [(hd, n, ic)]
                return _HoldKernel(ic)
            zero = _hid(n, "zero")
            self.vm[n] = [("CONST", [zero], "0"), ("SWITCH", [n, x, en, zero], "0 0 %s %s" % (ident, ident))]
            return lambda v: v[0] if v[1] else 0
        if bt in MR_BLOCKS:
            # initial output quantised to the signal type like a delay's initial condition
            key = "InitialCondition" if bt == "RateTransition" else "InitialConditions"
            try:
                ic = _real(p[key], where + "." + key)
            except GraphError:
                # a multirate offset-0 Downsample never outputs its initial condition -- Simulink runs
                # Inf / NaN / 1e400 there; it was refused as a malformed request
                if bt == "Downsample" and str(p.get("SampleOffset", "0")).strip() in ("0", "0.0") and \
                        str(p.get("RateOptions", "Allow multirate processing")).strip() == "Allow multirate processing":
                    ic = 0.0
                else:
                    raise
            # recordings (default diagnostics): a multirate Downsample with SampleOffset 0 never outputs its
            # initial condition and Simulink does not check it. recordings: in single-rate mode Simulink does stop
            # (fxpParameterOverflow, 82 / 82) -- recordings graph 19 had been recorded with ParameterOverflowMsg
            # 'none' (the regression check.m), so it saturated there; the normal check (which honours that setting) applies
            unused_ic = bt == "Downsample" and str(p.get("SampleOffset", "0")).strip() in ("0", "0.0") and \
                (str(p.get("RateOptions", "Allow multirate processing")).strip() == "Allow multirate processing" or
                 # factor 1, single-rate, offset 0, IC -129 on int8 -- Simulink runs
                 # (the output is the input); it had been refused as "Simulink stops"
                 str(fxnum.canonical(str(p.get("DownsamplingFactor", "2")), "int")) == "1")
            if bt == "RateTransition" and not any("[" in str(self.params[m].get("SampleTime", "")) for m in self.names):
                # recordings (recorded): fast to slow, an input without
                # an offset -- the initial condition is never output and Simulink runs a value beyond the type
                # (Deterministic on or off); slow to fast (rtic05) uses it and Simulink stops. Answered fast to slow
                try:
                    import fxmultirate as _M
                    per_ = _M.rates(self)
                    s0_ = self.src[n][0]
                    unused_ic = per_.get(s0_) is not None and per_.get(n) is not None and per_[n] >= per_[s0_]
                except GraphError:
                    unused_ic = False
            if ot != BOOL and not unused_ic:
                _param_overflow(ic, ot, where + "." + key, "%s %s: %s" % (bt, n, key), mode=self.pom,
                                notes=getattr(self, "param_notes", None))
            self.mr_ic = getattr(self, "mr_ic", {})
            self.mr_ic[n] = (1 if ic != 0 else 0) if ot == BOOL else _quant_param(ic, *ot)
            self._vm_need(n, False, "multirate")  # multirate: Python path (engine/fxmultirate.py)
            return None
        if bt in ("Outport",):
            if ts and ts[0] != BOOL:
                # recorded (recordings show, ufix128 through an Outport): a word the C core cannot hold failed with
                # CORE_FAILED "value outside its type" on the default path; such designs now run on the Python path
                self._vm_need(n, ts[0][1] + 1 <= VM_BITS, "width")
                self.vm_fast &= ts[0][1] + 1 <= FAST_BITS
            self.vm[n] = ("COPY", [n] + self.src[n], "")
            return lambda v: v[0]
        if bt in DELAYS:
            # initial condition quantised to the input type like a block parameter (nearest, saturate):
            # recordings (Delay / Unit Delay)
            ic = _real(p["InitialCondition"], where + ".InitialCondition")
            if ot != BOOL:
                _param_overflow(ic, ot, where + ".InitialCondition", "%s %s: InitialCondition" % (bt, n), mode=self.pom, notes=getattr(self, "param_notes", None))
            # a Tapped Delay's delays (engine/fxvector.py, _HalfGridIC): an initial value half-way between two
            # values of the type rounds up like any parameter -- +1.5 LSB starts at 2 (24
            # Tapped Delays), -2.5 LSB at -2 (-0.15625 in sfix12_En4 starts at -0.125); the negative
            # tie was refused while unrecorded
            self.ic = getattr(self, "ic", {})
            self.dlen = getattr(self, "dlen", {})
            self.ic[n] = (1 if ic != 0 else 0) if ot == BOOL else _quant_param(ic, *ot)
            L = int(p["DelayLength"]) if bt == "Delay" else 1
            self.dlen[n] = L
            # C core: a chain of one-sample delays (hidden slots d1..d(L-1), the block's own slot last)
            chain = [_hid(n, "d%d" % k) for k in range(1, L)] + [n]
            src = self.src[n][0]
            self.vm_dly[n] = []
            for x in chain:
                self.vm_dly[n].append((x, src, self.ic[n]))
                src = x
            return None
        route = self._fx_route(n, ts)
        if route is not None:
            if bt == "DiscreteFir":
                fs_ = p["FilterStructure"]
                if fs_ not in ("Direct form", "Direct form transposed", "Direct form symmetric",
                               "Direct form antisymmetric"):
                    raise GraphError("UNSUPPORTED", where + ".FilterStructure", "Discrete FIR: %r" % fs_)
                # an inheritance string of another block or parameter ('Inherit: Same as first input',
                # 'Inherit: auto', ...) was computed through an else-branch -- only the ones each parameter offers
                for k_, ok_ in (("CoefDataTypeStr", ("Inherit: Same word length as input",)),
                                ("ProductDataTypeStr", (fc._INHERIT_RULE, "Inherit: Same as input")),
                                ("AccumDataTypeStr", (fc._INHERIT_RULE, "Inherit: Same as input",
                                                      "Inherit: Same as product output")),
                                ("StateDataTypeStr", ("Inherit: Same as accumulator", "Inherit: Same as input")),
                                ("OutDataTypeStr", ("Inherit: Same as accumulator", "Inherit: Same as input")),
                                ("TapSumDataTypeStr", (fc._INHERIT_RULE, "Inherit: Same as input"))):
                    v_ = str(p.get(k_, ok_[0]))   # not stripped: the evaluator compares the raw text
                    if v_.strip().startswith("Inherit") and v_ not in ok_:
                        raise GraphError("BAD_TYPE", where + "." + k_, "%r is not an inheritance rule of the Discrete "
                                         "FIR %s (%s)" % (v_, k_, " / ".join(repr(x) for x in ok_)))
                if fc._asic(self.hw) and ts[0] != BOOL:
                    # review of the recordings fixes: both rules were recorded on the host only; the
                    # ASIC sets hold neither shape -- refused on ASIC/FPGA
                    nco = len(fc._dfir_coefs(p))
                    if nco == 1:
                        raise GraphError("UNSUPPORTED", where + ".Coefficients", "Discrete FIR with one coefficient on "
                                         "ASIC/FPGA (recorded on the host only)")
                    if fs_ in ("Direct form symmetric", "Direct form antisymmetric") and \
                            p.get("ProductDataTypeStr") == fc._INHERIT_RULE and \
                            not str(p.get("TapSumDataTypeStr", "Inherit")).strip().startswith("Inherit"):
                        raise GraphError("UNSUPPORTED", where + ".ProductDataTypeStr", "symmetric FIR with an explicit "
                                         "tap sum and the product by internal rule on ASIC/FPGA (recorded on the host "
                                         "only): give the product an explicit type")
                if ts[0] != BOOL and fs_ in ("Direct form symmetric", "Direct form antisymmetric"):
                    _dfir_sym_check(p, ts[0], where, self.words)
                if ts[0] != BOOL:
                    _dfir_sign_check(p, ts[0], where, self.words)
                if fc._asic(self.hw) and fs_ != "Direct form" and (
                        p["AccumDataTypeStr"] == fc._INHERIT_RULE or
                        (fs_ != "Direct form transposed" and p["TapSumDataTypeStr"] == fc._INHERIT_RULE)):
                    raise GraphError("UNSUPPORTED", where, "Discrete FIR %s on ASIC/FPGA hardware: the inherited "
                                     "accumulator / tap-sum rules are only observed for the direct form there; use "
                                     "explicit types" % fs_)
                try:
                    fc._dfir_initial_states(p, ts[0] if ts[0] != BOOL else (0, 8, 0), len(fc._dfir_coefs(p)))
                except ValueError as e:
                    raise GraphError("BAD_PARAM", where + ".InitialStates", str(e))
                if str(p.get("CoefDataTypeStr", "Inherit")).strip().startswith("Inherit") and \
                        all(c == 0 for c in fc._dfir_coefs(p)):
                    # recordings (8 of 8): with every
                    # coefficient 0 the inherited coefficient type is Simulink's own (sfix16_En15 input -> output
                    # sfix32_En1015), not the engine's -- a cold shape, refused
                    raise GraphError("UNSUPPORTED", where + ".Coefficients", "Discrete FIR %s: every coefficient is 0 with "
                                     "an inherited coefficient type, whose type Simulink chooses by a rule not modelled; "
                                     "give CoefDataTypeStr explicitly (the output is 0 anyway)" % n)
                if ts[0] != BOOL and fs_ != "Direct form transposed":
                    # the initial states are quantised into the input type
                    # and overflow it like any parameter; the transposed structure's states are
                    # in the accumulator type, checked in the kernel (9.5 / -20 on sfix8_En4 run there)
                    for v_ in fc._vec(p.get("InitialStates", "0")):
                        _param_overflow(v_, ts[0], where + ".InitialStates", "Discrete FIR %s: an initial state" % n,
                                        mode=self.pom, notes=getattr(self, "param_notes", None))
            self._emit_route(n, route, ts, ot)
            return _FxKernel(self, n, route, ts, ot)
        if bt == "RelationalOperator" and BOOL in ts:
            # recordings (40 designs, host and ASIC): a boolean input is
            # compared as the number 0 / 1, exactly, against a boolean or a fixed-point signal of up to 32 bits (no
            # lossy cast, whatever the fraction lengths and RndMeth; 480 / 480 steps)
            o_ = [t for t in ts if t != BOOL]
            if o_ and o_[0][1] > 32:
                raise GraphError("UNSUPPORTED", where, "Relational Operator %s: a boolean compared with %s; recorded "
                                 "against fixed-point signals of up to 32 bits only -- convert the boolean with a Data "
                                 "Type Conversion first" % (n, _tstr(o_[0])))
            ts = [(0, 8, 0) if t == BOOL else t for t in ts]
            op = p["Operator"]
            if op not in fc._RELOPS:
                raise GraphError("BAD_PARAM", where + ".Operator", "unsupported operator %r" % op)
            rel = fc._RELOPS[op]
            f = max(ts[0][2], ts[1][2])
            sa, sb = f - ts[0][2], f - ts[1][2]
            self._vm_need(n, ts[0][1] + sa <= VM_BITS and ts[1][1] + sb <= VM_BITS, "width")
            self.vm_fast &= ts[0][1] + sa <= FAST_BITS and ts[1][1] + sb <= FAST_BITS
            self.vm[n] = ("REL", [n, self.src[n][0], self.src[n][1]], "%d %d %d" % (sa, sb, RELCODE[op]))
            return lambda v: 1 if rel(v[0] << sa, v[1] << sb) else 0
        self._no_bool(n, ts)
        if bt == "RelationalOperator":
            op = p["Operator"]
            if op not in fc._RELOPS:
                raise GraphError("BAD_PARAM", where + ".Operator", "unsupported operator %r" % op)
            dbl = [k for k, x in enumerate(self.src[n]) if x in getattr(self, "dbl_const", ())]
            if dbl:
                o_ = ts[1 - dbl[0]]
                if len(dbl) > 1 or o_ == BOOL or o_[1] > 40:
                    raise GraphError("UNSUPPORTED", where, "Relational Operator %s compares a double Constant with %s: "
                                     "recorded against fixed-point signals of up to 40 bits only (beyond 53 bits a "
                                     "double cannot hold the signal exactly) -- give the Constant an explicit type"
                                     % (n, _tstr(o_) if o_ != BOOL else "a boolean"))
            fa, fb = ts[0][2], ts[1][2]
            rel = fc._RELOPS[op]
            why = fc.relop_wide_unobserved(tuple(ts[0]), tuple(ts[1]), self.words)
            if why:
                raise GraphError("UNSUPPORTED", where, why)
            c = fc.relop_lossy_type(tuple(ts[0]), tuple(ts[1]), self.words)   # exact on ASIC/FPGA
            if c is not None:
                cfm = fc.relop_cast_fm(p)   # the block's RndMeth (Simplest as Floor), saturating
                ra, rb = _hid(n, "ra"), _hid(n, "rb")
                cta, oka, fa62 = _ctok(fa, c, cfm, ts[0][1])
                ctb, okb, fb62 = _ctok(fb, c, cfm, ts[1][1])
                self._vm_need(n, oka and okb, "width")
                self.vm_fast &= fa62 and fb62
                self.vm[n] = [("CAST", [ra, self.src[n][0]], cta), ("CAST", [rb, self.src[n][1]], ctb),
                              ("REL", [n, ra, rb], "0 0 %d" % RELCODE[op])]
                return lambda v: 1 if rel(cast(v[0], fa, *c, cfm), cast(v[1], fb, *c, cfm)) else 0
            f = max(fa, fb)
            sa, sb = f - fa, f - fb
            self._vm_need(n, ts[0][1] + sa <= VM_BITS and ts[1][1] + sb <= VM_BITS, "width")
            self.vm_fast &= ts[0][1] + sa <= FAST_BITS and ts[1][1] + sb <= FAST_BITS
            self.vm[n] = ("REL", [n, self.src[n][0], self.src[n][1]], "%d %d %d" % (sa, sb, RELCODE[op]))
            return lambda v: 1 if rel(v[0] << sa, v[1] << sb) else 0
        fm = _fm(p, where)
        s, w, f = ot
        if bt == "Gain":
            si, wi, fi_ = ts[0]
            rec = self._gain_rec(n, _real(p["Gain"], where + ".Gain"), self._gain_param_type(n, ts[0]), ts[0], ot)
            pt, npar = rec.param_type, rec.param_stored
            ff = fi_ + pt[2]
            ct, ok, _f62 = _ctok(ff, ot, fm, wi + pt[1])
            self.vm_fast &= _f62
            self._vm_need(n, ok and wi + pt[1] <= VM_BITS, "width")
            self.vm_fast &= wi + pt[1] <= FAST_BITS and pt[1] <= FAST_BITS
            self.vm[n] = ("GAIN", [n, self.src[n][0]], "%d %s" % (npar, ct))
            return lambda v: cast(v[0] * npar, ff, s, w, f, fm)
        if bt == "Product":
            self._product_rec(n, p, ts, ot, fm)
            ff = ts[0][2] + ts[1][2]
            ct, ok, _f62 = _ctok(ff, ot, fm, ts[0][1] + ts[1][1])
            self.vm_fast &= _f62
            self._vm_need(n, ok and ts[0][1] + ts[1][1] <= VM_BITS, "width")
            self.vm_fast &= ts[0][1] + ts[1][1] <= FAST_BITS
            self.vm[n] = ("PROD", [n, self.src[n][0], self.src[n][1]], ct)
            return lambda v: cast(v[0] * v[1], ff, s, w, f, fm)
        if bt == "DataTypeConversion":
            fi_ = ts[0][2]
            if str(p.get("ConvertRealWorld", "")).startswith("Stored Integer"):
                om = fm["om"]
                ct, ok, _f62 = _ctok(0, (s, w, 0), fm, ts[0][1])
                self.vm_fast &= _f62
                self._vm_need(n, ok, "width")
                self.vm[n] = ("CAST", [n, self.src[n][0]], ct)
                return lambda v: fc._overflow(v[0], s, w, om)
            ct, ok, _f62 = _ctok(fi_, ot, fm, ts[0][1])
            self.vm_fast &= _f62
            self._vm_need(n, ok, "width")
            self.vm[n] = ("CAST", [n, self.src[n][0]], ct)
            return lambda v: cast(v[0], fi_, s, w, f, fm)
        if bt == "Saturation":
            hi = _quant_param(_real(p["UpperLimit"], where + ".UpperLimit"), s, w, f)
            lo = _quant_param(_real(p["LowerLimit"], where + ".LowerLimit"), s, w, f)
            if ts[0] != ot:  # explicit output type: cast (block rounding, saturating), then clamp
                si, wi, fi_ = ts[0]
                cfm = {"rm": fm["rm"], "om": "Saturate"}
                ct, ok, _f62 = _ctok(fi_, ot, cfm, wi)
                self.vm_fast &= _f62
                self._vm_need(n, ok, "width")
                c = _hid(n, "c")
                self.vm[n] = [("CAST", [c, self.src[n][0]], ct), ("SAT", [n, c], "%d %d" % (lo, hi))]
                return lambda v: min(max(cast(v[0], fi_, s, w, f, cfm), lo), hi)
            self.vm[n] = ("SAT", [n, self.src[n][0]], "%d %d" % (lo, hi))
            return lambda v: min(max(v[0], lo), hi)
        if bt == "Abs":
            om = fm["om"]
            if ts[0] != ot:  # explicit output type: |u| exactly, then cast with the block's rounding/overflow
                si, wi, fi_ = ts[0]
                ct, ok, _f62 = _ctok(fi_, ot, fm, wi + 1)
                self.vm_fast &= _f62
                self._vm_need(n, ok, "width")
                self.vm[n] = ("UN", [n, self.src[n][0]], "0 " + ct)
                return lambda v: cast(abs(v[0]), fi_, s, w, f, fm)
            ct, ok, _f62 = _ctok(f, ot, fm, w + 1)
            self.vm_fast &= _f62
            self._vm_need(n, ok, "width")
            self.vm[n] = ("UN", [n, self.src[n][0]], "0 " + ct)
            return lambda v: fc._overflow(abs(v[0]), s, w, om)
        if bt == "UnaryMinus":
            om = fm["om"]
            ct, ok, _f62 = _ctok(f, ot, fm, w + 1)
            self.vm_fast &= _f62
            self._vm_need(n, ok, "width")
            self.vm[n] = ("UN", [n, self.src[n][0]], "1 " + ct)
            return lambda v: fc._overflow(-v[0], s, w, om)
        if bt == "Sum":
            rec = self._sum_rec(n, p, ts, ot, fm, strict=True)
            signs, at, first = list(rec.signs), rec.acc_type, rec.first
            as_, aw, af = at
            fls = [t[2] for t in ts]
            om = fm["om"]
            rest = [(i, signs[i] == "+") for i in range(len(signs)) if i != first]
            def k_sum(v):
                xs = [cast(x, fl_, as_, aw, af, fm) for x, fl_ in zip(v, fls)]
                acc = xs[first] if first is not None else 0
                for i, plus in rest:   # fc._overflow looked up per call: run_diagnostics counts through it
                    acc = fc._overflow(acc + xs[i] if plus else acc - xs[i], as_, aw, om)
                return cast(acc, af, s, w, f, fm)
            parts = []
            for i, t in enumerate(ts):
                ct, ok, _f62 = _ctok(t[2], at, fm, t[1])
                self.vm_fast &= _f62
                self._vm_need(n, ok, "width")
                parts.append((self.src[n][i], ct, 1 if signs[i] == "+" else -1))
            acc_ct, ok1, _f62 = _ctok(af, at, fm, aw + 1)
            self.vm_fast &= _f62
            out_ct, ok2, _f62 = _ctok(af, ot, fm, aw)
            self.vm_fast &= _f62
            self._vm_need(n, ok1 and ok2, "width")
            self.vm[n] = ("SUM", [n], (len(ts), -1 if first is None else first, acc_ct, out_ct, parts))
            return k_sum
        if bt == "MinMax":
            why = fc.minmax_unresolved(ts, ot, self.words, fm["rm"]) or fc.minmax_wide_unobserved(ts, ot, self.words) \
                or fc.minmax_capped_wrap(ts, ot, self.words, fm["om"]) or fc.minmax_mixed_sign_many(
                    ts, ot, self.words, fm["rm"], not str(p.get("OutDataTypeStr", "Inherit")).strip().startswith("Inherit"))
            if why:
                raise GraphError("UNSUPPORTED", where, why + "")
            cs, cw, cf = fc._minmax_compare_type(ts, ot, self.words)
            fls = [t[2] for t in ts]
            if p["Function"] not in ("min", "max"):
                raise GraphError("BAD_PARAM", where + ".Function", "min or max")
            pick = min if p["Function"] == "min" else max
            if not fc._asic(self.hw) and fc.minmax_coarse_span(ts, ot, self.words, str(p.get("RndMeth", "Floor"))):
                # pairwise chain with intermediate types (fxcore.minmax_chain_types);
                # in the C core a two-input MINMAX per step into a hidden slot
                chain = fc.minmax_chain_types(ts)
                ops = []
                prev, pt = self.src[n][0], ts[0]
                for k, t in enumerate(chain, 1):
                    dst = n if k == len(chain) else _hid(n, "mm%d" % k)
                    ca, oka, fa62 = _ctok(pt[2], t, fm, pt[1])
                    cb, okb, fb62 = _ctok(ts[k][2], t, fm, ts[k][1])
                    oc, okc, fc62 = _ctok(t[2], ot if dst == n else t, fm, t[1])
                    self.vm_fast &= fa62 and fb62 and fc62
                    self._vm_need(n, oka and okb and okc, "width")
                    ops.append(("MINMAX", [dst], (2, 1 if p["Function"] == "max" else 0, oc,
                                                  [(prev, ca), (self.src[n][k], cb)])))
                    prev, pt = dst, t
                self.vm[n] = ops
                pk2 = lambda a, b: pick(a, b)
                return lambda v: cast(*fc.minmax_chain_eval(list(v), fls, chain, fm, pk2), s, w, f, fm)
            parts = []
            for i, t in enumerate(ts):
                ct, ok, _f62 = _ctok(t[2], (cs, cw, cf), fm, t[1])
                self.vm_fast &= _f62
                self._vm_need(n, ok, "width")
                parts.append((self.src[n][i], ct))
            out_ct, ok, _f62 = _ctok(cf, ot, fm, cw)
            self.vm_fast &= _f62
            self._vm_need(n, ok, "width")
            self.vm[n] = ("MINMAX", [n], (len(ts), 1 if p["Function"] == "max" else 0, out_ct, parts))
            return lambda v: cast(pick(cast(x, fl_, cs, cw, cf, fm) for x, fl_ in zip(v, fls)), cf, s, w, f, fm)
        raise GraphError("UNKNOWN_BLOCK_TYPE", where, bt)

    def run(self, inputs, nsteps, record):
        """inputs: {Inport: list of stored ints}; record: block names to log. Returns {name: [ints]}. Assertion blocks
 are checked after the run (_assertions)."""
        if (getattr(self, "hdl_info", None) or {}).get("es_kernels"):
            import fxhdl
            fxhdl.es_kernel_check(self.hdl_info)   # recordings (recorded)
        asserts = (getattr(self, "hdl_info", None) or {}).get("assertions") or []
        if not asserts:
            return self._run(inputs, nsteps, record)
        logs = self._run(inputs, nsteps, list(record) + [o for _, o, _ in asserts])
        return _assertions(self, logs, record)

    def _run(self, inputs, nsteps, record):
        import collections
        val = {}
        ic = getattr(self, "ic", {})
        dlen = getattr(self, "dlen", {})
        state = {d: collections.deque([ic[d]] * dlen[d]) for d in self.delays}
        for k in self.kern.values():
            if hasattr(k, "reset"):   # every stateful kernel, also those added by recorded (RAM, FIR, NCO)
                k.reset()
        logs = {n: [] for n in record}
        plan = []
        for n in self.order:
            bt = self.btype[n]
            if bt == "Inport":
                plan.append((n, 0, None, None))
            elif bt in DELAYS:
                plan.append((n, 1, None, None))
            else:
                plan.append((n, 2, self.kern[n], self.src[n]))
        dsrc = [(d, self.src[d][0]) for d in self.delays]
        rec = [(n, logs[n]) for n in dict.fromkeys(record)]  # a name recorded twice is logged once
        split = [(n, self.kern[n]) for n in getattr(self, "dti_split", [])]
        for t in range(nsteps):
            for n, k in split:   # integrators breaking a loop: their state first
                val[n] = k.peek()
            for n, kind, k, srcs in plan:
                if kind == 0:
                    val[n] = inputs[n][t]
                elif kind == 1:
                    val[n] = state[n][0]
                else:
                    val[n] = k([val[s] for s in srcs])
            for n, lg in rec:
                lg.append(val[n])
            new = [val[s] for d, s in dsrc]
            for (d, s), v in zip(dsrc, new):
                q = state[d]
                q.popleft()
                q.append(v)
        return logs

    def sat_cast_quiet(self):
        """recordings: Saturation blocks whose cast into the explicit output type is not an overflow event.
        The engine casts before the clamp; with both limits inside the output type the clamp leaves a
        value of the type and Simulink counts no saturation there (recordings-free native evidence: recordings,
        0 events against the engine's 768). With a limit outside the type the cast is still counted (not recorded)."""
        quiet = set()
        for n in self.names:
            t = self.otype.get(n)
            if self.btype[n] != "Saturation" or not isinstance(t, tuple):
                continue
            s_, w_, f_ = t[0], t[1], t[2]
            lo_t = Fraction(-(1 << (w_ - 1)) if s_ else 0, 1) / (Fraction(2) ** f_)
            hi_t = Fraction((1 << (w_ - 1)) - 1 if s_ else (1 << w_) - 1, 1) / (Fraction(2) ** f_)
            try:
                lims = [Fraction(float(str(fxnum.canonical(self.params[n][k], "scalar"))))
                        for k in ("LowerLimit", "UpperLimit")]
            except (KeyError, TypeError, ValueError, OverflowError):
                continue
            if all(lo_t <= v <= hi_t for v in lims):
                quiet.add(n)
        return quiet

    def run_diagnostics(self, inputs, nsteps, record):
        """run() that also counts, per block, the values that left a quantisation point's type -- its output and its
        internal ones (accumulator, product, cast) -- wrapped or saturated, with the first step of each, and the
        block's output range. Python path only. Returns (logs, {block: record})."""
        cur = {"n": None, "t": 0}
        ev = {}
        quiet = self.sat_cast_quiet()
        orig, orig_div0 = fc._overflow, fc._div0
        EMPTY = {"wraps": 0, "saturations": 0, "first_wrap_step": None, "first_saturation_step": None,
                 "divisions_by_zero": 0, "first_division_by_zero_step": None}

        def div0():
            if cur["n"] is not None:
                e = ev.setdefault(cur["n"], dict(EMPTY))
                e["divisions_by_zero"] += 1
                if e["first_division_by_zero_step"] is None:
                    e["first_division_by_zero_step"] = cur["t"]

        def counting(n_, s_, wl, om):
            lo, hi = (-(1 << (wl - 1)), (1 << (wl - 1)) - 1) if s_ else (0, (1 << wl) - 1)
            if not lo <= n_ <= hi and cur["n"] is not None and cur["n"] not in quiet:
                e = ev.setdefault(cur["n"], dict(EMPTY))
                kind = "saturation" if om == "Saturate" else "wrap"
                e[kind + "s"] += 1
                if e["first_%s_step" % kind] is None:
                    # a Discrete-Time Integrator computes at step t the state its update of step t - 1 produced
                    e["first_%s_step" % kind] = cur["t"] - (1 if self.btype[cur["n"]] == "DiscreteIntegrator" and
                                                            fc.dti_method(self.params[cur["n"]]) == "FE" else 0)
            return orig(n_, s_, wl, om)

        rng = {}
        names = [n for n in self.order if self.btype[n] not in ("Inport", "Outport")]
        wanted = list(dict.fromkeys(list(record) + names))
        fc._overflow, fc._div0 = counting, div0
        try:
            logs = self._run_traced(inputs, nsteps, wanted, cur)
        finally:
            fc._overflow, fc._div0 = orig, orig_div0
        return {n: logs[n] for n in record}, self._diag_records(names, logs, ev, EMPTY)

    def _diag_records(self, names, logs, ev, EMPTY):
        """The per-block diagnostics records from the event counts ev and every block's logged output (shared by the
        Python reference run_diagnostics and the C path run_diagnostics_c)."""
        diag = {}
        for n in names:
            t = self.otype.get(n)
            vs = logs[n]
            r = dict(ev.get(n, EMPTY), type=self.type_str(n))
            if isinstance(t, tuple) and vs:
                def real(x, f_=t[2]):   # exact scaling first: a huge stored integer overflows float
                    try:
                        return float(Fraction(x) / (Fraction(2) ** f_)) if f_ >= 0 else float(x * (1 << -f_))
                    except OverflowError:
                        return 1.7976931348623157e308 if x > 0 else -1.7976931348623157e308
                r["min"], r["max"] = real(min(vs)), real(max(vs))
                r["stored_min"], r["stored_max"] = min(vs), max(vs)
            elif t == BOOL and vs:
                # a boolean block's range was null (0 / 1 as the Fixed-Point Tool reports it)
                r["min"], r["max"] = float(min(int(bool(v)) for v in vs)), float(max(int(bool(v)) for v in vs))
            diag[n] = r
        return diag

    def run_diagnostics_c(self, inputs, nsteps, record, timeout_s=600):
        """run_diagnostics on the C core (engine/fxgraph_core DIAG): the same events -- a value leaving a quantisation
        point's type while the block's instruction runs, wrapped or saturated, with its first step; divisions by zero --
        and the same ranges, from every block's logged output. Returns (logs, {block: record})."""
        EMPTY = {"wraps": 0, "saturations": 0, "first_wrap_step": None, "first_saturation_step": None,
                 "divisions_by_zero": 0, "first_division_by_zero_step": None}
        quiet = self.sat_cast_quiet()
        names = [n for n in self.order if self.btype[n] not in ("Inport", "Outport")]
        wanted = list(dict.fromkeys(list(record) + names))
        if any(isinstance(v, tuple) and v and v[0] in CORE_PY_DIAG_OPS for v in self.vm.values()):
            # the instructions moved to the C core for speed (fork CA) keep diagnostics on the Python reference
            self.c_fallback = True
            return self.run_diagnostics(inputs, nsteps, record)
        try:
            logs, _ns, counts, byslot = run_fast(self, inputs, nsteps, wanted, timeout_s, diag=True)
        except CoreFallback:
            return self.run_diagnostics(inputs, nsteps, record)
        # lookup blocks: the C core folds some of their casts into compile-time tables (a Flat / Nearest lookup selects an entry
        # already cast into the output type, so its run-time cast never executes there): their events are counted by
        # the Python kernel itself on the inputs the C run logged -- the reference's own count by construction
        # (also the linear forms: the C kernel clamps a fraction that rounds to 1 by hand, not through a counted cast --
        # in the regression check)
        # also the blocks whose event rules are the Python kernel's own: a Discrete-Time Integrator (events at the step
        # of the state update) and the counters (a designed wrap is not an event) -- replayed in step order, state
        # reset first (the regression check recordings)
        folded = [n for n in names if self.btype[n] in ("Lookup1D", "Lookup2D", "LookupND", "DiscreteIntegrator",
                                                        "CounterFreeRunning", "CounterLimited", "Prelookup",
                                                        "PrelookupF", "InterpolationUsingPrelookup")]
        ev = {}
        if folded:
            cur = {"n": None, "t": 0}
            orig = fc._overflow

            def counting(n_, s_, wl, om):
                lo, hi = (-(1 << (wl - 1)), (1 << (wl - 1)) - 1) if s_ else (0, (1 << wl) - 1)
                if not lo <= n_ <= hi and cur["n"] is not None and cur["n"] not in quiet:
                    e = ev.setdefault(cur["n"], dict(EMPTY))
                    kind = "saturation" if om == "Saturate" else "wrap"
                    e[kind + "s"] += 1
                    if e["first_%s_step" % kind] is None:
                        e["first_%s_step" % kind] = cur["t"] - (1 if self.btype[cur["n"]] == "DiscreteIntegrator" and
                                                                fc.dti_method(self.params[cur["n"]]) == "FE" else 0)
                return orig(n_, s_, wl, om)

            def val_of(x, t):
                return inputs[x][t] if self.btype[x] == "Inport" else logs[x][t]
            fc._overflow = counting
            try:
                for n in folded:
                    cur["n"] = n
                    if hasattr(self.kern[n], "reset"):   # stateless lookups; a cache, if any, starts clean
                        self.kern[n].reset()
                    for t in range(nsteps):
                        cur["t"] = t
                        self.kern[n]([val_of(x, t) for x in self.src[n]])
            finally:
                fc._overflow = orig
                cur["n"] = None
            counts = {sl: c for sl, c in counts.items() if _slot_owner(self, byslot.get(sl)) not in folded}
        for sl, (w, sa, fw, fs, dz, fdz) in counts.items():
            n = _slot_owner(self, byslot.get(sl))
            if n is None:
                raise GraphError("CORE_FAILED", "core", "internal error: diagnostics for an unknown slot")
            e = ev.setdefault(n, dict(EMPTY))
            dti_fe = self.btype.get(n) == "DiscreteIntegrator" and fc.dti_method(self.params[n]) == "FE"
            if n not in quiet:
                for kind, cnt, first in (("wrap", w, fw), ("saturation", sa, fs)):
                    if cnt:
                        e[kind + "s"] += cnt
                        f_ = first - (1 if dti_fe else 0)
                        if e["first_%s_step" % kind] is None or f_ < e["first_%s_step" % kind]:
                            e["first_%s_step" % kind] = f_
            if dz:
                e["divisions_by_zero"] += dz
                if e["first_division_by_zero_step"] is None or fdz < e["first_division_by_zero_step"]:
                    e["first_division_by_zero_step"] = fdz
        return {n: logs[n] for n in record}, self._diag_records(names, logs, ev, EMPTY)

    def _run_traced(self, inputs, nsteps, record, cur):
        """run() with cur["n"] / cur["t"] set to the block and step being evaluated."""
        import collections
        val = {}
        ic = getattr(self, "ic", {})
        dlen = getattr(self, "dlen", {})
        state = {d: collections.deque([ic[d]] * dlen[d]) for d in self.delays}
        for k in self.kern.values():
            if hasattr(k, "reset"):   # every stateful kernel, also those added by recorded (RAM, FIR, NCO)
                k.reset()
        logs = {n: [] for n in record}
        dsrc = [(d, self.src[d][0]) for d in self.delays]
        split = [(n, self.kern[n]) for n in getattr(self, "dti_split", [])]
        for t in range(nsteps):
            cur["t"] = t
            for n, k in split:   # integrators breaking a loop: their state first, not counted twice
                val[n] = k.peek()
            for n in self.order:
                bt = self.btype[n]
                if bt == "Inport":
                    val[n] = inputs[n][t]
                elif bt in DELAYS:
                    val[n] = state[n][0]
                else:
                    cur["n"] = n
                    val[n] = self.kern[n]([val[s] for s in self.src[n]])
                    cur["n"] = None
            for n in record:
                logs[n].append(val[n])
            new = [val[s] for d, s in dsrc]
            for (d, s), v in zip(dsrc, new):
                q = state[d]
                q.popleft()
                q.append(v)
        return logs

    def is_bool_signal(self, n, inport_bool=()):
        """Structural test (no compile needed): does block n output a boolean?"""
        seen = set()
        while n not in seen:
            seen.add(n)
            bt = self.btype[n]
            if bt in ("RelationalOperator", "LogicalOperator"):
                return str(self.params[n].get("OutDataTypeStr", "boolean")).replace(" ", "") not in LOGIC_OUT_TYPES
            if bt in ("CompareToConstant", "DetectChange", "DetectIncrease", "DetectDecrease", "CompareToZero"):
                return fc._named_type(self.params[n].get("OutDataTypeStr", "boolean")) in (None, "boolean")
            if bt == "Inport":
                return n in inport_bool
            if bt in DELAYS + ("Outport",):
                n = self.src[n][0]
                continue
            return False
        return False

    def run_double(self, inputs, nsteps, record):
        """Double-precision reference of the same graph (casts are identities, unquantised parameters).
        inputs: {Inport: list of floats}. Whole-array evaluation when the graph has no cycle at all
        (delays become shifts), step-by-step otherwise. Same summation order as the C core's DREF."""
        import math
        order = None if getattr(self, "gated", None) else self._array_order()
        if order is not None:
            arr = {}
            for n in order:
                bt, p = self.btype[n], self.params[n]
                xs = [arr[s] for s in self.src[n]] if bt != "Inport" else None
                if bt == "Inport":
                    arr[n] = list(inputs[n][:nsteps])
                elif bt == "Constant":
                    arr[n] = [_const_double(p)] * nsteps
                elif bt in DELAYS:
                    L = min(int(p["DelayLength"]) if bt == "Delay" else 1, nsteps)
                    arr[n] = [float(p["InitialCondition"])] * L + xs[0][:nsteps - L]
                elif bt == "DiscreteFir":
                    acc = [0.0] * nsteps
                    fp = self.param_record(n)
                    init = list(fp.init)
                    if fp.structure == "Direct form transposed":
                        st_ = _TransposedDouble(list(fp.coefs), init)
                        arr[n] = [st_.step(v) for v in xs[0][:nsteps]]
                        continue
                    for k, c in enumerate(fp.coefs):
                        sh = [init[k - 1 - t] for t in range(min(k, nsteps))] + xs[0][:nsteps - k]
                        acc = [a + c * v for a, v in zip(acc, sh)]
                    arr[n] = acc
                elif bt in DSTATEFUL:
                    st = self._dinit(n)
                    arr[n] = [self._dstep(n, st, ([x_[t] for x_ in xs] if len(xs) > 1 else xs[0][t]) if xs else None)
                              for t in range(nsteps)]
                elif bt == "Hold":
                    y, out = self.param_record(n).ic, []
                    for a, e in zip(xs[0], xs[1]):
                        if e:
                            y = a
                        out.append(y)
                    arr[n] = out
                elif bt == "Gate":
                    arr[n] = [a if e else 0.0 for a, e in zip(xs[0], xs[1])]
                else:
                    arr[n] = self._dvec(n, xs, math)
            return {n: arr[n] for n in record}
        import collections
        val = {}
        state = {x: collections.deque([float(self.params[x]["InitialCondition"])] *
                                      (int(self.params[x]["DelayLength"]) if self.btype[x] == "Delay" else 1))
                 for x in self.delays}
        firs = {n: (_TransposedDouble(list(self.param_record(n).coefs), list(self.param_record(n).init))
                    if self.param_record(n).structure == "Direct form transposed" else
                    collections.deque(list(self.param_record(n).init) + [0.0]))
                for n in self.names if self.btype[n] == "DiscreteFir"}
        logs = {n: [] for n in record}
        dst = {n: self._dinit(n) for n in self.names if self.btype[n] in DSTATEFUL}
        holds = {n: self.param_record(n).ic for n in self.names if self.btype[n] == "Hold"}
        rls = {}   # Rate Limiter: previous output
        split = list(getattr(self, "dti_split", []))
        for t in range(nsteps):
            for n in split:   # integrators breaking a loop: their state is this step's output
                val[n] = dst[n]["x"]
            for n in self.order:
                bt = self.btype[n]
                if bt == "Inport":
                    val[n] = inputs[n][t]
                elif bt == "Hold":
                    if val[self.src[n][1]]:
                        holds[n] = val[self.src[n][0]]
                    val[n] = holds[n]
                elif bt == "Gate":
                    val[n] = val[self.src[n][0]] if val[self.src[n][1]] else 0.0
                elif n in self.gated and not (t >= self.gated[n][1] and (t - self.gated[n][1]) % self.gated[n][0] == 0):
                    pass   # slow-domain stateful block between its hits: output held, state unchanged
                elif bt in DSTATEFUL:
                    val[n] = self._dstep(n, dst[n], ([val[s_] for s_ in self.src[n]] if len(self.src[n]) > 1 else
                                                     val[self.src[n][0]]) if self.src[n] else None)
                elif bt in DELAYS:
                    val[n] = state[n][0]
                elif bt == "Constant":
                    val[n] = _const_double(self.params[n])
                elif bt == "DiscreteFir":
                    h = firs[n]
                    if isinstance(h, _TransposedDouble):
                        val[n] = h.step(val[self.src[n][0]])
                        continue
                    h.appendleft(val[self.src[n][0]])
                    h.pop()
                    acc = 0.0
                    for c, v in zip(self.param_record(n).coefs, h):
                        acc = acc + c * v
                    val[n] = acc
                elif bt == "DelayPort":
                    hist_ = rls.setdefault(("dp", n), {"u": [], "x0": None})
                    hist_["u"].append(val[self.src[n][0]])
                    if hist_["x0"] is None:
                        # the initial-value port's first value: port 3 when the length is from port 2
                        hist_["x0"] = val[self.src[n][-1]]
                    t_ = len(hist_["u"]) - 1
                    p_ = self.params[n]
                    if str(p_.get("DelayLengthSource", "Dialog")) == "Input port":
                        hist_.setdefault("d", []).append(int(val[self.src[n][1]]))
                        rm_ = str(p_.get("ExternalReset", "None")).strip()
                        if rm_ not in ("None", "none"):
                            hist_.setdefault("r", []).append(val[self.src[n][2]])
                        # (clamped, D; PreventDirectFeedthrough / Rising reset: fxcore.delayport_source, recordings)
                        rh_ = hist_.get("r", [])
                        last_ = None
                        if rm_ == "Rising":
                            # the last step where the reset changed from 0, carried along (was a scan per step)
                            last_ = hist_.get("last", 0)
                            if rh_[t_] != 0 and (t_ == 0 or rh_[t_ - 1] == 0):
                                last_ = t_
                            hist_["last"] = last_
                            rh_ = rh_[max(t_ - 1, 0):t_ + 1]
                        src_ = fc.delayport_source(t_, hist_["d"][-1], int(str(p_.get("DelayLengthUpperLimit",
                                                                                       "100")).strip()),
                                                   str(p_.get("PreventDirectFeedthrough", "off")).strip() == "on",
                                                   rm_, rh_, last=last_)
                        k_ = src_ if src_ is not None else -1
                        ic_ = hist_["x0"] if str(p_.get("InitialConditionSource", "Dialog")) == "Input port" else \
                            float(fc._const_value(p_.get("InitialCondition", "0")))
                        val[n] = hist_["u"][k_] if k_ >= 0 else ic_
                    else:
                        L_ = int(p_["DelayLength"])
                        val[n] = hist_["u"][t_ - L_] if t_ >= L_ else hist_["x0"]
                elif bt == "RateLimiter":
                    v_, y_ = val[self.src[n][0]], rls.get(n, float(fc._const_value(self.params[n]["InitialCondition"])))
                    t_ = float(self.params[n].get("_RlT", "1"))   # limits times the period
                    r_ = float(fc._const_value(self.params[n]["RisingSlewLimit"])) * t_
                    f_ = float(fc._const_value(self.params[n]["FallingSlewLimit"])) * t_
                    val[n] = rls[n] = y_ + r_ if v_ - y_ > r_ else y_ + f_ if v_ - y_ < f_ else v_
                else:
                    val[n] = self._dvec(n, [[val[s]] for s in self.src[n]], math)[0]
            for n in record:
                logs[n].append(val[n])
            new = [val[self.src[x][0]] for x in self.delays]
            for x, v in zip(self.delays, new):
                state[x].popleft()
                state[x].append(v)
        return logs

    def _array_order(self):
        """Topological order including delay edges, or None if the graph has a (delayed) cycle."""
        indeg = {n: len(self.src[n]) for n in self.names}
        users = {n: [] for n in self.names}
        for n in self.names:
            for s in self.src[n]:
                users[s].append(n)
        ready = sorted(n for n in self.names if indeg[n] == 0)
        out = []
        while ready:
            n = ready.pop()
            out.append(n)
            for u in users[n]:
                indeg[u] -= 1
                if indeg[u] == 0:
                    ready.append(u)
        return out if len(out) == len(self.names) else None

    def _dinit(self, n):
        bt, p = self.btype[n], self.params[n]
        if bt in ("CounterLimited", "CounterFreeRunning"):
            return {"x": 0.0, "lim": float(_counter_limit(self, n))}
        if bt == "DiscreteIntegrator":
            rec = self.param_record(n)
            return {"x": rec.ic, "k": float(rec.gain), "lim": rec.limits, "m": fc.dti_method(p),
                    "reset": str(p.get("ExternalReset", "none")), "ext": p.get("InitialConditionSource") == "external",
                    "rp": 0.0, "started": False}
        rec = self.param_record(n)
        return {"x": rec.vinit, "rel": fc._RELOPS[rec.relop]}

    def _dstep(self, n, st, u):
        """One step of a stateful block in double (same operations as the C core's double program)."""
        bt = self.btype[n]
        y = st["x"]
        if bt in ("CounterLimited", "CounterFreeRunning"):
            st["x"] = 0.0 if y == st["lim"] else (0.0 + y) + 1.0
            return y
        if bt == "DiscreteIntegrator" and isinstance(u, list):
            # Forward Euler with an external reset / initial condition (fxcore._blk_dti for the rule)
            u_, rest = u[0], u[1:]
            r_ = rest[0] if st["reset"] != "none" else None
            ic = rest[-1] if st["ext"] else st.get("ic0", st["x"])
            st.setdefault("ic0", st["x"])
            if not st["started"]:
                st["started"] = True
                if st["ext"]:
                    st["x"] = ic
            if r_ is not None:
                rp, m_ = st["rp"], st["reset"]
                hit = (rp <= 0 < r_) if m_ == "rising" else (rp > 0 >= r_) if m_ == "falling" else \
                    (rp <= 0 < r_ or rp > 0 >= r_) if m_ == "either" else (r_ != 0 or rp != 0) if m_ == "level" \
                    else r_ != 0
                if hit:
                    st["x"] = ic
                st["rp"] = r_
            y_ = st["x"]
            st["x"] = (0.0 + y_) + u_ * st["k"]
            return y_
        if bt == "DiscreteIntegrator" and st.get("m", "FE") != "FE":
            lim = st["lim"]

            def cl(v):
                return v if lim is None else min(lim[1], max(lim[0], v))
            if st["m"] == "BE":
                st["x"] = cl((0.0 + y) + u * st["k"])
                return st["x"]
            yy = cl((0.0 + y) + u * st["k"] / 2)
            st["x"] = cl(yy + u * st["k"] / 2)
            return yy
        if bt == "DiscreteIntegrator":
            v = (0.0 + y) + u * st["k"]
            if st["lim"] is not None:
                if v < st["lim"][0]:
                    v = st["lim"][0]
                if v > st["lim"][1]:
                    v = st["lim"][1]
            st["x"] = v
            return y
        r = 1.0 if st["rel"](u, y) else 0.0  # detect: u against the previous input
        st["x"] = u
        return r

    def _dvec(self, n, xs, math):
        bt, p = self.btype[n], self.params[n]
        if bt == "Signum":
            return [(0.0 + (1.0 if v > 0 else 0.0)) - (1.0 if v < 0 else 0.0) for v in xs[0]]
        if bt == "Sqrt":
            return [math.sqrt(v) if v >= 0 else float("nan") for v in xs[0]]
        if bt == "Trigonometry":   # atan2 / sin / cos (_trig_check)
            op_ = str(p.get("Operator", "sin"))
            if op_ in ("sin", "cos"):
                return [(math.sin if op_ == "sin" else math.cos)(a) for a in xs[0]]
            return [math.atan2(a, b) for a, b in zip(xs[0], xs[1])]
        if bt == "DeadZone":
            lo, up = float(fc._const_value(p["LowerValue"])), float(fc._const_value(p["UpperValue"]))
            return [v - up if v > up else v - lo if v < lo else 0.0 for v in xs[0]]
        if bt == "WrapToZero":
            th = float(fc._const_value(p["Threshold"]))
            return [0.0 if v > th else v for v in xs[0]]
        if bt == "RateLimiter":   # a whole series (the per-step loop keeps its own state, run_double)
            t_ = float(p.get("_RlT", "1"))   # limits times the period
            r, fl_ = float(fc._const_value(p["RisingSlewLimit"])) * t_, float(fc._const_value(p["FallingSlewLimit"])) * t_
            y, out = float(fc._const_value(p["InitialCondition"])), []
            for v in xs[0]:
                y = y + r if v - y > r else y + fl_ if v - y < fl_ else v
                out.append(y)
            return out
        if bt == "Rounding":
            return list(xs[0])
        if bt == "Lookup1D":
            if lut_ports(p):
                raise GraphError("UNSUPPORTED", "design.blocks." + n, "the double reference of a 1-D Lookup Table with "
                                 "its table / breakpoints from ports is not supported")
            bps, table = _lut_raw(p)
            mode = _lut_mode(p)
            return [_dlut(bps, table, mode, v) for v in xs[0]]
        if bt == "Lookup2D":
            if lut_ports(p):
                raise GraphError("UNSUPPORTED", "design.blocks." + n, "the double reference of a 2-D Lookup Table with "
                                 "its table / breakpoints from ports is not supported")
            bps, table = _lut2d_raw(p)
            mode = _lut_mode(p)
            return [_dlut2(bps, table, mode, a, b) for a, b in zip(xs[0], xs[1])]
        if bt == "Gain":
            g = float(p["Gain"])
            return [g * v for v in xs[0]]
        if bt == "Product":
            ops = str(p["Inputs"]).strip()
            ops = "*" * int(ops) if ops.isdigit() else ops
            if ops == "**":
                return [a * b for a, b in zip(xs[0], xs[1])]

            def dv(a, b):
                if b != 0:
                    return a / b
                # IEEE 754 (as the C core and MATLAB doubles): the sign of a zero divisor counts
                return math.nan if (a == 0 or a != a) else math.copysign(math.inf, a) * math.copysign(1.0, b)
            out = []
            for vals in zip(*xs):
                if ops == "/*":
                    out.append(dv(vals[1], vals[0]))
                    continue
                acc = dv(1.0, vals[0]) if ops[0] == "/" else vals[0]
                for o, v in zip(ops[1:], vals[1:]):
                    acc = dv(acc, v) if o == "/" else acc * v
                out.append(acc)
            return out
        if bt == "ShiftArithmetic" and str(p.get("BitShiftNumberSource", "Dialog")) == "Input port":
            d_ = p["BitShiftDirection"]
            out = []
            for v, k in zip(xs[0], xs[1]):
                e = (k if (d_ == "Left" and k > 0) else -k if (d_ in ("Right", "Bidirectional") and k > 0) else
                     -k if (d_ == "Bidirectional" and k < 0) else 0)
                out.append(v * 2.0 ** e)
            return out
        if bt == "DelayPort":   # a whole series (the per-step loop keeps its own history, run_double)
            lp_ = str(p.get("DelayLengthSource", "Dialog")) == "Input port"
            if lp_:
                # the initial value before the start: port 3 at the first step when it is from a port too
                ic = xs[2][0] if str(p.get("InitialConditionSource", "Dialog")) == "Input port" else \
                    float(fc._const_value(p.get("InitialCondition", "0")))
                lim_d = int(str(p.get("DelayLengthUpperLimit", "100")).strip())
                rm_ = str(p.get("ExternalReset", "None")).strip()
                rs_ = xs[2] if rm_ not in ("None", "none") else []
                # clamped into 0 .. limit; PreventDirectFeedthrough / Rising reset:
                # fxcore.delayport_source
                pdf_ = str(p.get("PreventDirectFeedthrough", "off")).strip() == "on"
                src_, last_ = [], 0
                for t in range(len(xs[0])):
                    if rm_ == "Rising":
                        # the last step where the reset changed from 0, carried along (was a scan per step)
                        if rs_[t] != 0 and (t == 0 or rs_[t - 1] == 0):
                            last_ = t
                        src_.append(fc.delayport_source(t, int(xs[1][t]), lim_d, pdf_, rm_,
                                                        rs_[max(t - 1, 0):t + 1], last=last_))
                    else:
                        src_.append(fc.delayport_source(t, int(xs[1][t]), lim_d, pdf_, rm_, rs_))
                return [xs[0][k_] if k_ is not None else ic for k_ in src_]
            L = int(p["DelayLength"])
            return [xs[0][t - L] if t >= L else xs[1][0] for t in range(len(xs[0]))]
        if bt == "ShiftArithmetic":
            f = _shift_factor(p)
            return [f * v for v in xs[0]]
        if bt == "LogicalOperator":
            op = p["Operator"]
            out = []
            for vals in zip(*xs):
                t = [1 if v != 0 else 0 for v in vals]
                r = {"AND": all(t), "OR": any(t), "NAND": not all(t), "NOR": not any(t), "XOR": sum(t) % 2 == 1,
                     "NXOR": sum(t) % 2 == 0, "NOT": not t[0]}[op]
                out.append(1.0 if r else 0.0)
            return out
        if bt == "CompareToConstant":
            rel, c = fc._RELOPS[p["relop"]], fc._const_value(p["const"])
            return [1.0 if rel(v, c) else 0.0 for v in xs[0]]
        if bt == "CompareToZero":
            rel = fc._RELOPS[p["relop"]]
            return [1.0 if rel(v, 0) else 0.0 for v in xs[0]]
        if bt == "Switch":
            crit = p["Criteria"]
            boolc = self.is_bool_signal(self.src[n][1])
            thr = float(p["Threshold"])
            out = []
            for a, u2, b in zip(*xs):
                take = u2 != 0 if (crit == "u2 ~= 0" or boolc) else (u2 >= thr if crit == "u2 >= Threshold" else u2 > thr)
                out.append(a if take else b)
            return out
        if bt == "MultiportSwitch" and p["DataPortOrder"] == "Specify indices":
            ix = fc.mps_indices(p)
            return [vals[1 + ix.index(int(vals[0]))] if (math.isfinite(vals[0]) and vals[0] == math.floor(vals[0])
                                                         and int(vals[0]) in ix) else vals[-1] for vals in zip(*xs)]
        if bt == "MultiportSwitch":
            base = 0 if p["DataPortOrder"].startswith("Zero") else 1
            nc = int(p["Inputs"])
            out = []
            for vals in zip(*xs):
                k = vals[0] - base
                idx = int(k) if (math.isfinite(k) and k == math.floor(k) and 0 <= k < nc) else len(vals) - 2
                out.append(vals[1 + idx])
            return out
        if bt in ("DataTypeConversion", "Outport"):
            if bt == "DataTypeConversion" and str(p.get("ConvertRealWorld", "")).startswith("Stored Integer"):
                raise GraphError("UNSUPPORTED", "design.blocks.%s.ConvertRealWorld" % n,
                                 "Stored Integer conversion has no double-precision reference")
            return list(xs[0])
        if bt == "Saturation":
            lo, hi = float(p["LowerLimit"]), float(p["UpperLimit"])
            return [hi if v > hi else (lo if v < lo else v) for v in xs[0]]
        if bt == "Abs":
            return [math.fabs(v) for v in xs[0]]
        if bt == "UnaryMinus":
            return [-v for v in xs[0]]
        if bt == "RelationalOperator":
            rel = fc._RELOPS[p["Operator"]]
            return [1.0 if rel(a, b) else 0.0 for a, b in zip(xs[0], xs[1])]
        if bt == "Sum":
            signs = _sum_signs(p, self.nin[n])
            acc = [0.0] * len(xs[0])
            for x, sg in zip(xs, signs):
                acc = [a + v for a, v in zip(acc, x)] if sg == "+" else [a - v for a, v in zip(acc, x)]
            return acc
        if bt == "MinMax":
            f = min if p["Function"] == "min" else max
            out = list(xs[0])
            for x in xs[1:]:
                out = [f(a, b) if (b > a if p["Function"] == "max" else b < a) else a for a, b in zip(out, x)]
            return out
        if bt in ("Prelookup", "PrelookupF", "InterpolationUsingPrelookup"):
            raise GraphError("UNSUPPORTED", "design.blocks." + n.split("\x1d")[0], "%s has no ideal (double) reference yet: "
                             "it can be simulated (fxgraph) but not word-length optimised (fxopt)" % bt)
        if bt == "LookupND":
            raise GraphError("UNSUPPORTED", "design.blocks." + n, "LookupND has no ideal (double) reference yet: it can "
                             "be simulated (fxgraph) but not word-length optimised (fxopt)")
        if bt in ("BitwiseOperator", "ExtractBits"):
            raise GraphError("UNSUPPORTED", "design.blocks." + n, "%s acts on bit patterns and has no ideal "
                             "(double) reference: it can be simulated (fxgraph) but not word-length optimised "
                             "(fxopt)" % bt)
        raise GraphError("UNKNOWN_BLOCK_TYPE", "design.blocks." + n, bt)

    def type_str(self, n):
        t = self.otype[n]
        if t == BOOL:
            return "boolean"
        if n in getattr(self, "dbl_const", ()):
            return "double"   # a default-typed Constant compared exactly (_double_relop_const)
        return "fixdt(%d,%d,%d)" % t


# ---------------------------------------------------------------- C fast path (engine/fxgraph_core)
CORE = os.path.join(HERE, "fxgraph_core")
OPCODE = {"GAIN": 1, "PROD": 2, "CAST": 3, "SAT": 4, "UN": 5, "REL": 6, "SUM": 7, "MINMAX": 8, "COPY": 9,
          "CONST": 10, "SHIFT": 11, "LOGIC": 12, "SWITCH": 13, "MPSW": 14, "PRODN": 15, "BITW": 16, "EXTR": 17,
          "LUT": 18, "LUTL": 19, "LUT2": 20, "PSHIFT": 21, "DEN": 22, "DPORT": 23, "IPL": 30, "PLK": 31}


def _fmt_ins(n, entry, slot):
    if isinstance(entry, list):
        return [_fmt_ins(n, e, slot) for e in entry]
    op, sl, rest = entry
    if op == "LOGIC":
        code, srcs = rest
        return "%d %d %d %d %s" % (OPCODE[op], slot[sl[0]], code, len(srcs), " ".join(str(slot[x]) for x in srcs))
    if op == "MPSW":
        cfl, base, ncontig, parts = rest
        return "%d %d %d %d %d %d %d %s" % (OPCODE[op], slot[sl[0]], slot[sl[1]], cfl, base, ncontig, len(parts),
                                            " ".join("%d %s" % (slot[x], c) for x, c in parts))
    if op == "PRODN":
        special, fo, rmm, rmd, s_, w_, sat, operands = rest
        return "%d %d %d %d %d %d %d %d %d %d %s" % (
            OPCODE[op], slot[sl[0]], len(operands), special, fo, rmm, rmd, s_, w_, sat,
            " ".join("%d %d %d" % (slot[x], fl, dv) for x, fl, dv in operands))
    if op == "BITW":
        code, s_, w_, um, mask, srcs = rest
        return "%d %d %d %d %d %d %d %d %s" % (OPCODE[op], slot[sl[0]], len(srcs), code, s_, w_, um, mask,
                                               " ".join(str(slot[x]) for x in srcs))
    if op == "LUT":
        r = rest
        return "%d %d %d %d %d %d %d %s" % (OPCODE[op], slot[sl[0]], slot[sl[1]], r.mode, len(r.bps), r.sa, r.sb,
                                            " ".join("%d %d" % pr for pr in zip(r.bps, r.outs)))
    if op == "LUTL":
        r = rest
        it, ft = r.intermediate, r.frac_type
        return "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %s %s %s" % (
            OPCODE[op], slot[sl[0]], slot[sl[1]], len(r.bps), r.sa, r.sb, ft[2], ft[0], ft[1], r.rm,
            1 if r.saturate else 0, it[0], it[1], it[2],
            (1 if r.use_last else 0) | (2 if r.magnitude_slope else 0) |   # bit 1: magnitude slope term
            (4 if getattr(r, "frac_carry", False) else 0) |   # bit 2: a fraction rounding to 1 is 1
            (8 if getattr(r, "frac_round1_stop", False) else 0),   # bit 3: ... is not recorded: stop the run
            r.dsh, r.out_cast,
            " ".join("%d %d" % pr for pr in zip(r.bps, r.tab)), " ".join(map(str, r.slopes)))
    if op == "LUT2":
        r = rest
        (ff, fs, fw, rm, sat, its, itw, ulast, _itf, dsh), outc, tabT = r.core_fields()
        sh, bints = r.shifts, r.bps
        return "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %s %s %s %s %s" % (
            OPCODE[op], slot[sl[0]], slot[sl[1]], slot[sl[2]], r.mode, len(bints[0]), len(bints[1]), sh[0], sh[1],
            sh[2], sh[3], ff, fs, fw, rm, sat, its, itw, ulast, dsh, outc, " ".join(map(str, bints[0])),
            " ".join(map(str, bints[1])), " ".join(map(str, r.tab)), " ".join(map(str, tabT)))
    if op == "PLK":
        mode, sa, ffl, bints = rest
        return "%d %d %d %d %d %d %d %s" % (OPCODE[op], slot[sl[0]], slot[sl[1]], mode, sa, ffl, len(bints),
                                            " ".join(map(str, bints)))
    if op == "IPL":
        dims, mode, shp, ffs, it, ot, mag, tab = rest
        return "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %s %s" % (
            OPCODE[op], slot[sl[0]], dims, mode, shp[0], shp[1], ffs[0], ffs[1], it[0], it[1], it[2], ot[0], ot[1],
            ot[2], mag, " ".join(str(slot[x]) for x in sl[1:]), " ".join(map(str, tab)))
    if op == "SUM":
        m, first, acc_ct, out_ct, parts = rest
        head = "%d %d %d %s %s" % (OPCODE[op], slot[sl[0]], m, first, acc_ct + " " + out_ct)
        return head + " " + " ".join("%d %s %d" % (slot[src], ct, sg) for src, ct, sg in parts)
    if op == "MINMAX":
        m, ismax, out_ct, parts = rest
        head = "%d %d %d %d %s" % (OPCODE[op], slot[sl[0]], m, ismax, out_ct)
        return head + " " + " ".join("%d %s" % (slot[src], ct) for src, ct in parts)
    return "%d %s %s" % (OPCODE[op], " ".join(str(slot[x]) for x in sl), rest)


def _flat(x):
    return x if isinstance(x, list) else [x]


def _fmt_ins_d(d, n, slot):
    """_fmt_ins, with a strict Multiport Switch (Simulink stops on an out-of-range control) marked
    for the core by its negated count of contiguous data ports."""
    x = _fmt_ins(n, d.vm[n], slot)
    if n in getattr(d, "runtime_diag", ()):
        def strict(e, txt):
            if e[0] != "MPSW":
                return txt
            t = txt.split(" ")
            t[5] = str(-int(t[5]))
            return " ".join(t)
        if isinstance(d.vm[n], list):   # a Multiport Switch with boolean data: hidden (u ~= 0) slots first
            x = [strict(e, txt) for e, txt in zip(d.vm[n], x)]
        else:
            x = strict(d.vm[n], x)
    return x


def vm_instructions(d):
    """Compiled design -> (slot map, list of integer instruction strings, delay strings 'dslot src ic')."""
    slot = {n: i for i, n in enumerate(d.slots)}
    ins = []
    for n in d.order:
        if n in d.vm:
            ins += _flat(_fmt_ins_d(d, n, slot))
    dly = ["%d %d %d" % (slot[x], slot[s_], ic) for n in d.names for (x, s_, ic) in d.vm_dly.get(n, ())]
    return slot, ins, dly


def _shift_factor(p):
    """Real-valued effect of Shift Arithmetic: y = u * 2^e."""
    k = int(p["BitShiftNumber"])
    d = p["BitShiftDirection"]
    left = d == "Left" or (d == "Bidirectional" and k < 0)
    try:
        return 2.0 ** (int(p["BinPtShiftNumber"]) + (abs(k) if left else -abs(k)))
    except OverflowError:   # beyond double: the reference is not finite (reported as such, not INTERNAL)
        return float("inf")


def _const_double(p):
    """The ideal (double) reference value of a Constant. A value written fi(v, s, w, f) is the fi object's value --
    v quantised (nearest, saturating) -- as a double-override run of the model would hold it (error review
    the reference used v itself, so fi(0.1,1,8,4) was 0.1 against the design's 0.125 and the difference
    0.025 was charged to every plan as an error no word length can remove)."""
    v = p["Value"].strip()
    fiv = fc._parse_fi(v)
    if str(p.get("OutDataTypeStr", "")).strip() == "boolean":
        return 1.0 if float(v) != 0 else 0.0
    if fiv is None:
        return float(v)
    val, s_, w_, f_ = fiv
    return float(Fraction(fc._quant_param(val, s_, w_, f_)) / (Fraction(2) ** f_))


def vm_double_instructions(d):
    """Double-precision program of the same graph for the core's DREF: casts are identities, gains,
    limits, thresholds and constants are the unquantised parameter values, delays start from the given
    initial conditions, booleans are 0/1 (the analogue of a double data-type override)."""
    slot = {n: i for i, n in enumerate(d.slots)}
    ins, dly = [], []
    for n in d.order:
        mark = len(dly)
        _vm_double_one(d, n, slot, ins, dly)
        if n in getattr(d, "gated", {}):
            en = slot[d.gated[n][2]]
            for i in range(mark, len(dly)):
                x, src, ic = dly[i].split(" ", 2)
                tmp = slot[_hid(n, "g%d" % (i - mark))]
                ins.append("13 %d %s %d %s 0 0.0" % (tmp, src, en, x))
                dly[i] = "%s %d %s" % (x, tmp, ic)
    return slot, ins, dly


def _vm_double_one(d, n, slot, ins, dly):
    if True:
        bt, p, src = d.btype[n], d.params[n], [slot[x] for x in d.src[n]]
        me = slot[n]
        if bt == "Inport":
            return
        if bt in DELAYS:
            L = int(p["DelayLength"]) if bt == "Delay" else 1
            ic = float(p["InitialCondition"])
            chain = [_hid(n, "d%d" % k) for k in range(1, L)] + [n]
            prev = d.src[n][0]
            for x in chain:
                dly.append("%d %d %r" % (slot[x], slot[prev], ic))
                prev = x
            return
        if bt == "Gain":
            ins.append("1 %d %d %r" % (me, src[0], float(p["Gain"])))
        elif bt == "Product":
            ops = str(p["Inputs"]).strip()
            ops = "*" * int(ops) if ops.isdigit() else ops
            if ops == "**":
                ins.append("2 %d %d %d" % (me, src[0], src[1]))
            else:
                ins.append("15 %d %d %d %s" % (me, len(src), 1 if ops == "/*" else 0,
                                                " ".join("%d %d" % (x, 1 if o == "/" else 0) for x, o in zip(src, ops))))
        elif bt == "DataTypeConversion":
            if str(p.get("ConvertRealWorld", "")).startswith("Stored Integer"):
                raise GraphError("UNSUPPORTED", "design.blocks.%s.ConvertRealWorld" % n,
                                 "Stored Integer conversion has no double-precision reference")
            ins.append("3 %d %d" % (me, src[0]))
        elif bt == "Saturation":
            ins.append("4 %d %d %r %r" % (me, src[0], float(p["LowerLimit"]), float(p["UpperLimit"])))
        elif bt in ("Abs", "UnaryMinus"):
            ins.append("5 %d %d %d" % (me, src[0], 1 if bt == "UnaryMinus" else 0))
        elif bt == "RelationalOperator":
            ins.append("6 %d %d %d %d" % (me, src[0], src[1], RELCODE[p["Operator"]]))
        elif bt == "Sum":
            signs = _sum_signs(p, d.nin[n])
            ins.append("7 %d %d %s" % (me, len(src), " ".join("%d %d" % (x, 1 if sg == "+" else -1)
                                                              for x, sg in zip(src, signs))))
        elif bt == "MinMax":
            ins.append("8 %d %d %d %s" % (me, len(src), 1 if p["Function"] == "max" else 0, " ".join(map(str, src))))
        elif bt == "Constant":
            ins.append("10 %d %r" % (me, _const_double(p)))
        elif bt in ("DelayPort",) or (bt == "ShiftArithmetic" and
                                         str(p.get("BitShiftNumberSource", "Dialog")) == "Input port"):
            raise GraphError("UNSUPPORTED", "design.blocks." + n, "no C double form (Python path)")
        elif bt == "ShiftArithmetic":
            ins.append("1 %d %d %r" % (me, src[0], _shift_factor(p)))
        elif bt == "LogicalOperator":
            code = {"AND": 0, "OR": 1, "NAND": 2, "NOR": 3, "XOR": 4, "NXOR": 5, "NOT": 6}[p["Operator"]]
            ins.append("12 %d %d %d %s" % (me, code, len(src), " ".join(map(str, src))))
        elif bt in ("CompareToConstant", "CompareToZero"):
            c = slot[_hid(n, "c")]
            ins.append("10 %d %r" % (c, fc._const_value(p["const"]) if bt == "CompareToConstant" else 0.0))
            ins.append("6 %d %d %d %d" % (me, src[0], c, RELCODE[p["relop"]]))
        elif bt == "Hold":
            hd = slot[_hid(n, "hd")]
            dly.append("%d %d %r" % (hd, me, d.param_record(n).ic))
            ins.append("13 %d %d %d %d 0 0.0" % (me, src[0], src[1], hd))
        elif bt == "Gate":
            z = slot[_hid(n, "zero")]
            ins.append("10 %d 0.0" % z)
            ins.append("13 %d %d %d %d 0 0.0" % (me, src[0], src[1], z))
        elif bt == "Switch":
            crit = p["Criteria"]
            boolc = d.is_bool_signal(d.src[n][1])
            mode = 0 if (crit == "u2 ~= 0" or boolc) else (1 if crit == "u2 >= Threshold" else 2)
            ins.append("13 %d %d %d %d %d %r" % (me, src[0], src[1], src[2], mode, float(p["Threshold"])))
        elif bt == "MultiportSwitch" and p["DataPortOrder"] == "Specify indices":
            raise GraphError("UNSUPPORTED", "design.blocks." + n, "no C double form (Python path)")
        elif bt == "MultiportSwitch":
            base = 0 if p["DataPortOrder"].startswith("Zero") else 1
            ins.append("14 %d %d %d %d %d %s" % (me, src[0], base, int(p["Inputs"]), len(src) - 1,
                                                  " ".join(map(str, src[1:]))))
        elif bt == "DiscreteFir" and d.param_record(n).structure == "Direct form transposed":
            coefs = list(d.param_record(n).coefs)
            N = len(coefs)
            init = list(d.param_record(n).init)
            for k, c in enumerate(coefs):
                ins.append("1 %d %d %r" % (slot[_hid(n, "p%d" % k)], src[0], c))
            for k in range(1, N):
                z, nz = slot[_hid(n, "z%d" % k)], slot[_hid(n, "nz%d" % k)]
                dly.append("%d %d %r" % (z, nz, init[k - 1]))
                if k < N - 1:
                    ins.append("7 %d 2 %d 1 %d 1" % (nz, slot[_hid(n, "p%d" % k)], slot[_hid(n, "z%d" % (k + 1))]))
                else:
                    ins.append("7 %d 1 %d 1" % (nz, slot[_hid(n, "p%d" % k)]))
            if N > 1:
                ins.append("7 %d 2 %d 1 %d 1" % (me, slot[_hid(n, "p0")], slot[_hid(n, "z1")]))
            else:
                ins.append("7 %d 1 %d 1" % (me, slot[_hid(n, "p0")]))
        elif bt == "DiscreteFir":
            coefs = list(d.param_record(n).coefs)
            taps = [d.src[n][0]] + [_hid(n, "z%d" % k) for k in range(1, len(coefs))]
            init = list(d.param_record(n).init)
            for k in range(1, len(coefs)):
                dly.append("%d %d %r" % (slot[taps[k]], slot[taps[k - 1]], init[k - 1]))
            for k, c in enumerate(coefs):
                ins.append("1 %d %d %r" % (slot[_hid(n, "p%d" % k)], slot[taps[k]], c))
            ins.append("7 %d %d %s" % (me, len(coefs), " ".join("%d 1" % slot[_hid(n, "p%d" % k)]
                                                               for k in range(len(coefs)))))
        elif bt in ("CounterLimited", "CounterFreeRunning"):
            h = {t: slot[_hid(n, t)] for t in HIDDEN_P4B2[bt]}
            ins += ["10 %d 1.0" % h["one"], "10 %d %r" % (h["lim"], float(_counter_limit(d, n))), "10 %d 0.0" % h["zero"],
                    "6 %d %d %d %d" % (h["eq"], me, h["lim"], RELCODE["=="]),
                    "7 %d 2 %d 1 %d 1" % (h["inc"], me, h["one"]),
                    "13 %d %d %d %d 0 0.0" % (h["nxt"], h["zero"], h["eq"], h["inc"])]
            dly.append("%d %d 0.0" % (me, h["nxt"]))
        elif bt in ("DetectChange", "DetectIncrease", "DetectDecrease"):
            prev = slot[_hid(n, "prev")]
            rec = d.param_record(n)
            ins.append("6 %d %d %d %d" % (me, src[0], prev, RELCODE[rec.relop]))
            dly.append("%d %d %r" % (prev, src[0], rec.vinit))
        elif bt == "DiscreteIntegrator" and (fc.dti_method(p) != "FE" or p.get("ExternalReset", "none") != "none" or
                                             p.get("InitialConditionSource", "internal") != "internal"):
            raise GraphError("UNSUPPORTED", "design.blocks." + n, "no C double form (Python path)")
        elif bt == "DiscreteIntegrator":
            h = {t: slot[_hid(n, t)] for t in HIDDEN_P4B2[bt]}
            rec = d.param_record(n)
            ins += ["1 %d %d %r" % (h["inc"], src[0], float(rec.gain)), "7 %d 2 %d 1 %d 1" % (h["sum"], me, h["inc"])]
            if rec.limits is not None:
                ins.append("4 %d %d %r %r" % (h["nxt"], h["sum"], rec.limits[0], rec.limits[1]))
            else:
                ins.append("9 %d %d" % (h["nxt"], h["sum"]))
            dly.append("%d %d %r" % (me, h["nxt"], rec.ic))
        elif bt == "Signum":
            h = {t: slot[_hid(n, t)] for t in HIDDEN_P4B2[bt]}
            ins += ["10 %d 0.0" % h["zero"], "6 %d %d %d %d" % (h["pos"], src[0], h["zero"], RELCODE[">"]),
                    "6 %d %d %d %d" % (h["neg"], src[0], h["zero"], RELCODE["<"]),
                    "7 %d 2 %d 1 %d -1" % (me, h["pos"], h["neg"])]
        elif bt == "Rounding":
            ins.append("9 %d %d" % (me, src[0]))
        elif bt == "Lookup1D":
            if lut_ports(p):
                raise GraphError("UNSUPPORTED", "design.blocks." + n, "the double reference of a 1-D Lookup Table with "
                                 "its table / breakpoints from ports is not supported")
            bps, table = _lut_raw(p)
            ins.append("18 %d %d %d %d %s" % (me, src[0], _lut_mode(p), len(table),
                                              " ".join("%r %r" % (b, t) for b, t in zip(bps, table))))
        elif bt == "Lookup2D":
            if lut_ports(p):
                raise GraphError("UNSUPPORTED", "design.blocks." + n, "the double reference of a 2-D Lookup Table with "
                                 "its table / breakpoints from ports is not supported")
            bps, table = _lut2d_raw(p)
            ins.append("20 %d %d %d %d %d %d %s %s %s" % (me, src[0], src[1], _lut_mode(p), len(bps[0]), len(bps[1]),
                                                       " ".join(map(repr, bps[0])), " ".join(map(repr, bps[1])),
                                                       " ".join(repr(v) for r in table for v in r)))
        elif bt in ("Outport", "UnitDelay"):
            ins.append("9 %d %d" % (me, src[0]))
        else:
            raise GraphError("UNSUPPORTED", "design.blocks." + n, "%s acts on bit patterns or has no ideal (double) "
                             "reference: it can be simulated (fxgraph) but not word-length optimised (fxopt)" % bt)


def _assertions(d, logs, record):
    """Assertion blocks: the first step whose input is zero stops the run when
    StopWhenAssertionFail is on -- ASSERTION_FAILED, with the outputs of the steps before it in e.partial (Simulink
    records up to the step before) -- or is counted (d.assert_events: {block: [failing steps]})."""
    d.assert_events = {}
    first = {}   # stopping Assertion -> its first failing step
    for n, o, stop in (getattr(d, "hdl_info", None) or {}).get("assertions") or []:
        bad = [k for k, v in enumerate(logs[o]) if v == 0]
        d.assert_events[n] = bad
        if bad and stop:
            first[n] = bad[0]
    if first:
        # the earliest failure of any stopping Assertion stops the run, whatever the blocks' order (
        # the first block in the list decided); blocks failing at that same step are named in a fixed order
        k = min(first.values())
        at = sorted(n for n, s_ in first.items() if s_ == k)
        # the run ends at step k: failures after it never happen
        d.assert_events = {n: [s_ for s_ in ev if s_ <= k] for n, ev in d.assert_events.items()}
        e = GraphError("ASSERTION_FAILED", "design.blocks." + at[0], "assertion failed at step %d%s (Simulink: "
                       "Simulink:blocks:AssertionAssert; the run stops)"
                       % (k, " (also %s)" % ", ".join(at[1:]) if len(at) > 1 else ""))
        e.partial = {r: logs[r][:k] for r in record}
        e.step = k
        raise e
    return {r: logs[r] for r in record}


def run_double_core(d, ints, in_fl, nsteps, record, timeout_s=600):
    """The ideal double-precision reference of d (same as d.run_double) computed by the C core's double program
    (DREF + DOUT): {block: [float] * nsteps} for the record blocks. ints: {Inport: stored integers}, in_fl: {Inport:
    fraction length}. Returns None when the core cannot take the design (no binary, a block without a double
    instruction, any core error) -- the caller then uses the Python path. Bit-identical to run_double
    (the regression check)."""
    import subprocess
    if not os.path.exists(CORE) or nsteps < 1:
        return None
    try:
        slot, dins, ddly = vm_double_instructions(d)
    except GraphError:
        return None
    if any(r not in slot for r in record) or any(x not in slot for x in d.inports):
        return None
    head = ["IN %d %d %d %s" % (slot[x], in_fl[x], nsteps, " ".join(str(int(v)) for v in ints[x][:nsteps]))
            for x in d.inports]
    head.append("DREF %d %d %s %d %s %d %s" % (len(d.slots), len(dins), " ".join(dins), len(ddly), " ".join(ddly),
                                               len(record), " ".join(str(slot[r]) for r in record)))
    head += ["DOUT", "END"]
    try:
        p = subprocess.run([CORE, "-s"], input=("\n".join(head) + "\n").encode(), capture_output=True,
                           timeout=timeout_s)
    except (OSError, subprocess.TimeoutExpired):
        return None
    if p.returncode != 0:
        return None
    by_slot = {}
    for line in p.stdout.decode().splitlines():
        parts = line.split()
        if parts and parts[0] == "E":
            return None
        if parts and parts[0] == "W":
            vals = [float(v) for v in parts[3:]]
            if len(vals) != nsteps:
                return None
            by_slot[int(parts[1])] = vals
    out = {}
    for r in record:
        if slot[r] not in by_slot:
            return None
        out[r] = by_slot[slot[r]]
    return out


def _slot_owner(d, name):
    """The design block a C-core slot belongs to: the block itself, or for a hidden slot (a reserved-character name)
    the block whose lowering made it."""
    if name is None:
        return None
    if name in d.btype:
        return name   # (recorded hidden blocks are blocks of the lowered design: engine/fxgraph folds them by owner)
    for ch in RESERVED_CHARS:
        for part in str(name).split(ch):
            if part in d.btype:
                return part
    return None


CORE_PY_DIAG_OPS = ("PSHIFT", "DEN", "DPORT")   # C-core instructions whose diagnostics stay on the Python reference


class CoreFallback(Exception):
    """The C core handed a run back to the Python path (E FALLBACK)."""


def run_fast(d, inputs, nsteps, record, timeout_s=600, diag=False):
    """Same contract as Design.run, executed by the C interpreter. Returns (logs, core_ns); with diag, also the C
    core's per-slot event counts {slot: (wraps, saturations, first wrap, first saturation, div0, first div0)} and
    {slot: name} -- (logs, core_ns, counts, byslot). (recorded: HDL assertions are recorded alongside and folded in.)"""
    asserts = (getattr(d, "hdl_info", None) or {}).get("assertions") or []
    if asserts:
        res = _run_fast(d, inputs, nsteps, list(record) + [o for _, o, _ in asserts], timeout_s, diag)
        return (_assertions(d, res[0], record),) + tuple(res[1:])
    return _run_fast(d, inputs, nsteps, record, timeout_s, diag)


def _run_fast(d, inputs, nsteps, record, timeout_s=600, diag=False):
    import subprocess
    record = list(dict.fromkeys(record))
    slot, ins, dly = vm_instructions(d)
    out = ["P %d %d %d" % (len(d.slots), len(ins), nsteps)] + ins
    for n in d.inports:
        out.append("IN %d %s" % (slot[n], " ".join(map(str, inputs[n][:nsteps]))))
    pairs = [x.split() for x in dly]  # 'dslot src ic' for every (possibly hidden) one-sample delay
    for a, _, v in pairs:
        out.append("INIT %s %s" % (a, v))
    out.append("DLY %d %s" % (len(pairs), " ".join("%s %s" % (a, b) for a, b, _ in pairs)))
    out.append("REC %d %s" % (len(record), " ".join(str(slot[x]) for x in record)))
    if diag:
        out.append("DIAG")
    out.append("END")
    p = subprocess.run([CORE], input="\n".join(out).encode(), capture_output=True, timeout=timeout_s)
    first = p.stdout.decode(errors="replace").split("\n", 1)[0]
    if first.startswith("E FALLBACK "):
        # a step the Python reference stops or refuses on (a shift from a port of 2^31 or more, ...): the run is
        # repeated on the Python path, which gives the same answer or reports that step as the reference does
        d.c_fallback = True
        if diag:
            raise CoreFallback(first)
        return d.run(inputs, nsteps, record), 0
    if first.startswith("E UNRECORDED "):   # a lookup fraction rounding to 1, unrecorded
        _e, _c, sl_, st_ = (first.split() + ["?", "?"])[:4]
        name = next((n for n, i in slot.items() if str(i) == sl_), "?")
        raise GraphError("UNSUPPORTED", "design.blocks." + name, "at step %s: %s" % (st_, unrecorded_round1_msg(d, name)))
    if first.startswith("E SIM_DIAGNOSTIC "):   # a strict Multiport Switch out of range
        _e, _c, sl_, st_ = (first.split() + ["?", "?"])[:4]
        name = next((n for n, i in slot.items() if str(i) == sl_), "?")
        raise GraphError("SIM_DIAGNOSTIC", "design.blocks." + name, "at step %s: Multiport Switch control is out of "
                         "range of its data ports: Simulink stops with an error (DiagnosticForDefault Error, or a "
                         "single data port)" % st_)
    text = p.stdout.decode()
    counts = {}
    if diag:
        keep = []
        for ln in text.split("\n"):
            if ln.startswith("X "):
                f_ = [int(v) for v in ln.split()[1:]]
                counts[f_[0]] = tuple(f_[1:])
            else:
                keep.append(ln)
        text = "\n".join(keep)
    logs, core_ns = parse_core_output(p.returncode, text, {slot[x]: x for x in record},
                                      nsteps, {x: d.otype[x] for x in record})
    if diag:
        return logs, core_ns, counts, {i: n for n, i in slot.items()}
    return logs, core_ns


def unrecorded_round1_msg(d, name):
    """The message of a lookup fraction rounding to 1 where it is not recorded (the C core's E UNRECORDED)."""
    try:
        p = d.params[name]
        xt = d.otype.get(d.src[name][0])
        bt_ = fc._lut_setup(p, tuple(xt))[1] if xt else None
        ff = fc._named_type(d._lut1d_frac(p, tuple(xt)))[2] if xt else None
        return fc.LUT_ROUND1_MSG % fc.lut1d_round1_why(p, ff, bt_, fc._asic(d.hw))
    except Exception:  # noqa: BLE001 -- the message only
        return fc.LUT_ROUND1_MSG % "for this block"


def parse_core_output(rc, text, byslot, nsteps, types):
    """Strict reader of the C core's run output: exit code 0, an 'E' line becomes its error, every
    recorded slot exactly once with exactly nsteps integers inside its type's range, one 'T' line, the 'D'
    terminator last, nothing else. byslot: {slot: block}; types: {block: (s, wl, fl) or BOOL}."""
    lines = text.split("\n")
    if lines and lines[-1] == "":
        lines.pop()
    if lines and lines[0].startswith("E "):
        parts = lines[0].split(" ", 2)
        if len(parts) > 1 and parts[1] == "PROTOCOL":
            # the C core rejected the program the engine compiled for it -- an engine defect, not a property of the
            # design ("engine: MPSW" with refusal_kind unknown)
            raise GraphError("CORE_FAILED", "core", "internal error: the C core could not run this design")
        raise GraphError(parts[1] if len(parts) > 1 else "CORE_FAILED", "core", parts[2] if len(parts) > 2 else "")
    if rc != 0:
        raise GraphError("CORE_FAILED", "core", "core exited with code %d" % rc)
    if not lines or lines[-1] != "D":
        raise GraphError("CORE_FAILED", "core", "core output has no end marker")
    logs, core_ns, seen_t = {}, 0, False
    for ln in lines[:-1]:
        parts = ln.split(" ")
        if parts[0] == "O" and len(parts) >= 2:
            try:
                sl = int(parts[1])
                vals = [int(v) for v in parts[2:]]
            except ValueError:
                raise GraphError("CORE_FAILED", "core", "malformed output record")
            if sl not in byslot:
                raise GraphError("CORE_FAILED", "core", "output record for an unrequested slot %d" % sl)
            name = byslot[sl]
            if name in logs:
                raise GraphError("CORE_FAILED", "core", "duplicate output record for %s" % name)
            if len(vals) != nsteps:
                raise GraphError("CORE_FAILED", "core", "%s: %d samples, expected %d" % (name, len(vals), nsteps))
            t = types[name]
            lo, hi = (0, 1) if t == BOOL else ((-(1 << (t[1] - 1)), (1 << (t[1] - 1)) - 1) if t[0] else (0, (1 << t[1]) - 1))
            if any(v < lo or v > hi for v in vals):
                raise GraphError("CORE_FAILED", "core", "%s: value outside its type %s" % (name, _tstr(t)))
            logs[name] = vals
        elif parts[0] == "T" and len(parts) == 2 and not seen_t:
            try:
                core_ns = int(parts[1])
            except ValueError:
                raise GraphError("CORE_FAILED", "core", "malformed timing record")
            seen_t = True
        else:
            raise GraphError("CORE_FAILED", "core", "unexpected core output line %r" % ln[:60])
    missing = [x for x in byslot.values() if x not in logs]
    if missing:
        raise GraphError("CORE_FAILED", "core", "core output incomplete: no record for %s" % ", ".join(sorted(missing)[:5]))
    return logs, core_ns


class IncrementalCompiler:
    """Builds core programs for many type assignments of one design structure. Each block's compiled
    instruction is cached by (block, its type overrides, its input types, its output type); misses are
    compiled by the same Design._kernel as a full compile, so the program text is identical to
    vm_instructions(full compile) (checked in the regression check)."""

    def __init__(self, spec, hw, in_types):
        self.d = Design(spec, hw)
        self.base_params = {n: dict(p) for n, p in self.d.params.items()}
        self.in_types = in_types
        self.slot = {n: i for i, n in enumerate(self.d.slots)}
        self.cache = {}

    def program(self, overrides):
        """overrides: {block: {param: type string}}. Returns (instrs, delays, otype, ok126, fast62, wide)
        where wide lists the instruction indices that need 128-bit arithmetic on the 64-bit path."""
        d = self.d
        for n, ov in overrides.items():
            p = dict(self.base_params[n])
            p.update(ov)
            d.params[n] = p
        d.otype = {}
        for n in d.inports:
            d.otype[n] = _chk(self.in_types[n], "inputs." + n)
        d._resolve_types()
        ins, ok, fast, dlys, wide = [], True, True, {}, []
        for n in d.order:
            ov = overrides.get(n)
            key = (n, tuple(sorted(ov.items())) if ov else (), tuple(d.otype[s] for s in d.src[n]), d.otype[n])
            hit = self.cache.get(key)
            if hit is None:
                d.vm, d.vm_ok, d.vm_fast, d.ic, d.vm_dly, d.dlen = {}, True, True, {}, {}, {}
                d.vm_why = None
                d._kernel(n)
                t = d.otype[n]
                txt = _flat(_fmt_ins_d(d, n, self.slot)) if n in d.vm else []
                dl = ["%d %d %d" % (self.slot[x], self.slot[s_], ic) for (x, s_, ic) in d.vm_dly.get(n, ())]
                hit = (txt, d.vm_ok, t == BOOL or t[1] + 1 <= FAST_BITS, d.vm_fast, dl)
                self.cache[key] = hit
            txt, o, store, comp, dl = hit
            if not comp:
                wide.extend(range(len(ins), len(ins) + len(txt)))
            ins.extend(txt)
            ok &= o
            fast &= store
            dlys[n] = dl
        dly = [x for n in d.names for x in dlys.get(n, ())]
        return ins, dly, dict(d.otype), ok, fast, wide

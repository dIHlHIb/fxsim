"""Lower supported block configurations. Each block is rewritten
into engine blocks that every path supports, so the Python path, the C core, fxcodegen and fxverilog all run them:

  untyped (engine/fxhdl.py lower, before types):
    SignalSpecification  identity; a declared type must equal the input's (TYPE_MISMATCH), Dimensions checked by the
                         vector pass
    Reshape              identity (OutputDimensionality 1-D / column / row: the engine's vectors carry no orientation)
    ManualSwitch         the selected input (sw / CurrentSetting '1': input 1, '0': input 2); the two inputs must have
                         one type (different types were recorded with an undetermined output rule: refused)
    Math conj            a real signed input unchanged (unsigned: PORT_TYPE_INVALID); complex: engine/fxhdl.py _complex
    Assertion            a hidden Outport; after every run the first zero sample stops the run (StopWhenAssertionFail
                         on: ASSERTION_FAILED with the outputs before that step) or is counted (off); Enabled off: none
  typed (compile time, the input types probed: engine/fxhdl.py typed_rewrite):
    Bias                 Sum(x, Constant(bias rounded to nearest in x's type)), output x's type, wrap / saturate
    Increment / Decrement Real World / Stored Integer   Sum(x, +-1 or +-1 LSB in x's type), wrap
    BitReduce            AND / OR: a comparison of the bit pattern; XOR: the parity of the bits; output ufix1
    Ground               into a Sum: a zero of the Sum's other input type (anything else: refused)
    Math mod             integer classes, the divisor a Constant: x - d * floor(x / d) (d = 0: x)
  structural (engine/fxvector.py): TappedDelay (and the Enabled Synchronous one), BusAssignment."""
import math
import json
import re
from fractions import Fraction

INTERNAL = "\x1d"
UNTYPED = ("SignalSpecification", "Reshape", "ManualSwitch", "Assertion")
TYPED = ("Bias", "IncrementRealWorld", "DecrementRealWorld", "IncrementStoredInteger", "DecrementStoredInteger",
         "BitReduce", "Ground")
ALL = UNTYPED + TYPED + ("Math",)
DEFAULTS = {"SignalSpecification": {"Dimensions": "-1", "OutDataTypeStr": "Inherit: auto"},
            "Reshape": {"OutputDimensionality": "1-D array", "OutputDimensions": "[1,1]"},
            "ManualSwitch": {"sw": "1"},
            "Assertion": {"Enabled": "on", "AssertionFailFcn": "", "StopWhenAssertionFail": "on"},
            "Math": {"Operator": "exp", "OutDataTypeStr": "Inherit: Same as first input",
                     "SaturateOnIntegerOverflow": "on", "RndMeth": "Floor"},
            "Bias": {"Bias": "0.0", "SaturateOnIntegerOverflow": "off"},
            "IncrementRealWorld": {}, "DecrementRealWorld": {}, "IncrementStoredInteger": {},
            "DecrementStoredInteger": {},
            "BitReduce": {"mode": "AND"},
            "Ground": {},
            "TappedDelay": {"vinit": "0.0", "NumDelays": "4", "DelayOrder": "Oldest", "includeCurrent": "off",
                            "samptime": "-1"},
            "TappedDelayEnabledSynchronous": {"vinit": "0.0", "NumDelays": "4", "DelayOrder": "Oldest",
                                              "includeCurrent": "off", "samptime": "-1"},
            "BusAssignment": {"AssignedSignals": "signal1"}}   # the library defaults
ALLOWED = {"Reshape.OutputDimensionality": ("1-D array", "Column vector (2-D)", "Row vector (2-D)",
                                           "Customize"),   # 
           "ManualSwitch.sw": ("0", "1"), "Assertion.Enabled": ("on", "off"),
           "Assertion.StopWhenAssertionFail": ("on", "off"),
           "Math.Operator": ("conj", "mod", "square", "magnitude^2", "reciprocal", "rem", "transpose", "hermitian"),
           "Math.SaturateOnIntegerOverflow": ("on", "off"), "Bias.SaturateOnIntegerOverflow": ("on", "off"),
           "BitReduce.mode": ("AND", "OR", "XOR"), "TappedDelay.DelayOrder": ("Oldest", "Newest"),
           "TappedDelay.includeCurrent": ("on", "off"), "TappedDelayEnabledSynchronous.DelayOrder": ("Oldest", "Newest"),
           "TappedDelayEnabledSynchronous.includeCurrent": ("on", "off")}
# Additional Math Function operators:
MATH_IDENTITY = ("conj", "transpose", "hermitian")             # untyped: the input unchanged
MATH_TYPED = ("mod", "square", "magnitude^2", "reciprocal", "rem")   # typed rewrites (the input types probed)
# rejected by Simulink R2026b on every recorded fixed-point / integer input (PortValidationInputError); the engine
# has no double signals, so they are refused on every input
MATH_REJECTED = ("exp", "log", "10^u", "log10", "pow", "hypot")
# the recorded input types and OutDataTypeStr values of the recordings operators (strict: nothing else is accepted)
MATH_L18_IN = {"square": ((1, 16, 8), (0, 8, 0)), "magnitude^2": ((1, 16, 8), (0, 8, 0)),
               "reciprocal": ((1, 16, 8), (0, 8, 0)), "transpose": ((1, 16, 8), (0, 8, 0)),
               "hermitian": ((1, 16, 8),), "rem": ((0, 8, 0),)}
MATH_L18_OUT = ("Inherit: Same as first input", "fixdt(1,24,12)")
# recorded Simulink rejections among the recordings operators by input type (Simulink:blocks:PortValidationInputError)
MATH_L18_REJECTED = {("hermitian", (0, 8, 0)), ("rem", (1, 16, 8))}


def math_op(b):
    return str(b.get("Operator", "exp")).strip()


NIN = {"SignalSpecification": 1, "Reshape": 1, "ManualSwitch": 2, "Assertion": 1, "Bias": 1, "IncrementRealWorld": 1,
       "DecrementRealWorld": 1, "IncrementStoredInteger": 1, "DecrementStoredInteger": 1, "BitReduce": 1, "Ground": 0}


def _refuse(where, msg, code="UNSUPPORTED"):
    from fxgraph_lib import GraphError
    raise GraphError(code, where, msg)


def dims_of(text, where):
    """Signal Specification Dimensions: None for -1 (inherited), else the list of sizes ("[1 1]", "[1,1]", "[11]",
    "4"); anything else is BAD_PARAM."""
    t = str(text).strip()
    if t == "-1":
        return None
    body = t[1:-1] if t.startswith("[") and t.endswith("]") else t
    parts = body.replace(",", " ").split()
    if not parts or not all(x.isdigit() and int(x) >= 1 for x in parts):
        _refuse(where, "Dimensions %r: -1, or positive integer sizes" % text, "BAD_PARAM")
    return [int(x) for x in parts]


def params(n, b):
    """Checked parameters (defaults filled) of block n."""
    if b.get("type") == "BitReduce" and "mode" not in b:
        # the library default is now recorded -- recordings (a freshly placed Bit Reduce:
        # mode 'AND') and a recording of library blocks; AND itself is recorded
        b = dict(b, mode="AND")
    if b.get("type") == "ManualSwitch" and "sw" not in b:
        # the library block's table has no position (varsize / SampleTime only) and every
        # recording sets sw, so a left-out position is not assumed (the export would write the guess)
        _refuse("design.blocks.%s.sw" % n, "Manual Switch without sw: give '1' (input 1) or '0' (input 2); the "
                "library block's position is not recorded", "MISSING_FIELD")
    t = b["type"]
    where = "design.blocks." + n
    p = dict(DEFAULTS[t])
    for k, v in b.items():
        if k == "type" or k.startswith("_"):   # internal markers of the vector pass (_Tie)
            continue
        if k == "SampleTime":
            # listed by the catalogue for every block; these blocks inherit their rate, so the
            # library's '-1' is taken and any other sample time refused
            if str(v).strip() != "-1":
                _refuse(where + ".SampleTime", "%s with SampleTime %r: only '-1' (inherited, the library value) is "
                        "supported" % (t, v))
            continue
        if t == "Math" and k == "OutputSignalType" and str(b.get("Operator", "")).strip() == "conj":
            # a dialog parameter of Math (accepted on a complex conj); on a real input 'complex'
            # makes a complex output with a zero imaginary part -- Simulink runs it, not supported here
            if str(v).strip() not in ("auto", "real"):
                _refuse(where + "." + k, "conj of a real signal with OutputSignalType %r (a complex output) is not "
                        "supported; leave it 'auto'" % v)
            continue
        if k not in DEFAULTS[t] and __import__("fxgraph_lib").DESIGN_RANGE_RE.match(k):
            # recordings (3): OutMin / OutMax of a Signal Specification ... are real parameters
            if str(v).strip() not in ("[]", ""):
                _refuse(where + "." + k, "design range %s = %r is not supported: Simulink uses design minimum / "
                        "maximum values to choose types and to check parameters, which this engine does not model; "
                        "leave it '[]'" % (k, str(v).strip()))
            continue
        if k not in DEFAULTS[t]:
            _refuse(where + "." + k, "unknown parameter %s for %s" % (k, t), "BAD_PARAM")
        p[k] = str(v).strip()
    if t == "Math" and math_op(b) == "conj" and p["OutDataTypeStr"] not in (DEFAULTS["Math"]["OutDataTypeStr"],
                                                                          "Inherit: Inherit via internal rule"):
        # (the internal rule is recorded too -- the input's type, values equal)
        # recordings (README, conj): an explicit fixdt(1,17,8) has no effect (the output keeps the input's type);
        # class: any other text ('bogus', another inherit rule) was answered the same way
        import fxcore
        ot_ = p["OutDataTypeStr"]
        if fxcore.parse_type(ot_) is None:
            G_ = __import__("fxgraph_lib")
            if re.match(r"^\s*(Inherit:|boolean\s*$|double\s*$|single\s*$)", ot_):
                _refuse(where + ".OutDataTypeStr", "conj with OutDataTypeStr %r: only the library rule, the internal "
                        "rule and an explicit fixed-point type (none has an effect) are recorded" % ot_)
            raise G_._type_text_error(where + ".OutDataTypeStr", ot_)
    for k, v in p.items():
        al = ALLOWED.get("%s.%s" % (t, k))
        if al and v not in al:
            _refuse(where + "." + k, "%s=%r (recorded: %s)" % (k, v, ", ".join(repr(x) for x in al)))
    if t == "Reshape" and p["OutputDimensionality"] == "Customize":
        # OutputDimensions [1 1 0] was answered (the scalar path is an identity); Simulink
        # rejects any dimension that is not a positive integer at set_param (Simulink:Engine:InvSignalDims). The
        # element count against the input is checked by the vector pass (fxvector) and below for a scalar
        import fxvector
        dims_ = fxvector._ints(p["OutputDimensions"], where + ".OutputDimensions")
        if not dims_ or any(d_ <= 0 for d_ in dims_):
            _refuse(where + ".OutputDimensions", "Reshape OutputDimensions %s: Simulink rejects dimensions that are "
                    "not positive integers (Simulink:Engine:InvSignalDims)" % p["OutputDimensions"], "BAD_PARAM")
        if len(dims_) != 2:
            _refuse(where + ".OutputDimensions", "Reshape Customize with %d dimension(s) %s: only two dimensions were "
                    "recorded" % (len(dims_), dims_))
    if t == "Assertion" and p["AssertionFailFcn"]:
        _refuse(where + ".AssertionFailFcn", "an AssertionFailFcn (MATLAB code run on each failure) cannot run here")
    return p


def _from_complex(spec, n, seen=None):
    """True when a signal into block n comes, through any blocks but a Complex to Real-Imag, from a Real-Imag to
    Complex (a complex signal)."""
    seen = seen if seen is not None else set()
    for ln in spec.get("lines", []):
        if ln.get("dst") != n or ln.get("src") in seen:
            continue
        s_ = ln["src"]
        seen.add(s_)
        t_ = (spec["blocks"].get(s_) or {}).get("type")
        if t_ == "RealImagToComplex":
            return True
        if t_ != "ComplexToRealImag" and _from_complex(spec, s_, seen):
            return True
    return False


def has(spec):
    return any(isinstance(b, dict) and b.get("type") in ALL for b in spec.get("blocks", {}).values())


def _num(v, where):
    # through fxnum first (Fraction() read '1_0', other digit scripts and '20/2' as 10)
    import parameter_bounds
    c = parameter_bounds.number(v, where)
    try:
        # the value is the double MATLAB evaluates, not the decimal
        # text -- an exact half LSB or an overflow-line value written with many digits rounds to another double
        return Fraction(float(str(c)))
    except OverflowError:
        _refuse(where, "a finite number required, got %r (a double overflows)" % (v,), "BAD_PARAM")
    except (ValueError, ZeroDivisionError):
        _refuse(where, "numeric literal required, got %r" % (v,), "BAD_PARAM")


# ---------------------------------------------------------------- untyped (fxhdl.lower)
def _on_cycle(n, lines):
    succ = {}
    for l in lines:
        succ.setdefault(l["src"], set()).add(l["dst"])
    seen, todo = set(), list(succ.get(n, ()))
    while todo:
        m = todo.pop()
        if m == n:
            return True
        if m not in seen:
            seen.add(m)
            todo.extend(succ.get(m, ()))
    return False


def lower_untyped(spec, info):
    """Identity blocks removed (their consumers read the source), Assertions turned into hidden Outports; returns the
    new spec. info gets "declared" (block, source, type text), "equal_types" (block, a, b), "assertions"."""
    blocks = dict(spec["blocks"])
    lines = list(spec.get("lines", []))
    src = {(l["dst"], int(l.get("dport", 1))): (l["src"], int(l.get("sport", 1))) for l in lines}
    alias = {}
    removed = set()
    replaced = False
    for n, b in spec["blocks"].items():
        if not isinstance(b, dict) or b.get("type") not in UNTYPED + ("Math",):
            continue
        t = b["type"]
        if t == "Math" and math_op(b) in MATH_REJECTED:
            _refuse("design.blocks.%s.Operator" % n, "Math Function %s takes floating-point inputs only: Simulink "
                    'R2026b rejects it on fixed-point and integer inputs (Simulink:blocks:PortValidationInputError'
                    '), and the engine has no double signals' % math_op(b), "PORT_TYPE_INVALID")
        if t == "Math" and math_op(b) in MATH_TYPED:
            continue   # mod, square, magnitude^2, reciprocal, rem: typed rewrites
        p = params(n, b)
        where = "design.blocks." + n
        nin = NIN.get(t, 1)
        for k in range(1, nin + 1):
            if (n, k) not in src:
                _refuse(where, "input %d is not connected" % k, "BAD_CONNECTION")
        for (d_, dp), _s in src.items():
            if d_ == n and dp > nin:
                _refuse(where, "input port %d does not exist" % dp, "BAD_CONNECTION")
        for l in lines:
            if l["src"] == n and int(l.get("sport", 1)) != 1:
                _refuse(where, "output port %s does not exist" % l.get("sport"), "BAD_CONNECTION")
        del blocks[n]
        removed.add(n)
        if t == "SignalSpecification":
            # parsed as numbers (the text with its spaces removed read "[11]" as [1 1])
            dims = dims_of(p["Dimensions"], where + ".Dimensions")
            if dims is not None and math.prod(dims) != 1:
                _refuse(where + ".Dimensions", "Dimensions %s on a scalar signal (Simulink: PortDimsMismatch)"
                        % p["Dimensions"], "TYPE_MISMATCH")
            if p["OutDataTypeStr"] != "Inherit: auto":
                info.setdefault("declared", []).append((n, src[(n, 1)], p["OutDataTypeStr"]))
                if _on_cycle(n, lines):
                    # on a feedback loop Simulink takes the loop's type from the
                    # specification (Sum -> Unit Delay -> Signal Specification fixdt(1,32,6) -> Sum: all sfix32_En6);
                    # a fixed-type identity in its place, and the declared check still requires its input to have
                    # that type (fixdt(1,24,6) there: InputPortDataTypeMismatch, as Simulink)
                    blocks[n] = {"type": "DataTypeConversion", "OutDataTypeStr": p["OutDataTypeStr"],
                                 "RndMeth": "Floor", "SaturateOnIntegerOverflow": "off"}
                    removed.discard(n)
                    replaced = True
                    continue
            alias[n] = src[(n, 1)]
        elif t == "Reshape":
            alias[n] = src[(n, 1)]
        elif t == "ManualSwitch":
            info.setdefault("equal_types", []).append((n, src[(n, 1)], src[(n, 2)]))
            alias[n] = src[(n, 1)] if p["sw"] == "1" else src[(n, 2)]
        elif t == "Math" and math_op(b) == "conj":   # conj of a real signal (the complex pass took the complex ones)
            info.setdefault("conj_real", []).append((n, src[(n, 1)]))
            alias[n] = src[(n, 1)]
        elif t == "Math":   # transpose / hermitian of a scalar: the input unchanged
            # (RndMeth and saturation do not matter for this identity; replayed equal)
            if p["OutDataTypeStr"] not in MATH_L18_OUT and math_op(b) != "transpose":
                # (transpose keeps the input's type whatever OutDataTypeStr says -- internal rule,
                # fixdt(0,24,27), fixdt(1,8,0) ... all gave the input type, values unchanged; 16 / 16 equal)
                _refuse(where + ".OutDataTypeStr", "Math Function %s with OutDataTypeStr %r: recorded with %s only "
                        "(it keeps the input's type)" % (math_op(b), p["OutDataTypeStr"], " / ".join(MATH_L18_OUT)))
            info.setdefault("math_identity", []).append((n, src[(n, 1)], math_op(b)))
            alias[n] = src[(n, 1)]
        elif t == "Assertion":
            if any(l["src"] == n for l in lines):
                _refuse(where, "an Assertion has no output port", "BAD_CONNECTION")
            if p["Enabled"] == "on":
                o = INTERNAL + "assert/" + n
                blocks[o] = {"type": "Outport"}
                lines.append(dict({"src": src[(n, 1)][0], "dst": o}, **({"sport": src[(n, 1)][1]}
                                                                         if src[(n, 1)][1] != 1 else {})))
                info.setdefault("assertions", []).append((n, o, p["StopWhenAssertionFail"] == "on"))
    if not removed and not replaced:
        return spec

    def res(s):
        seen = 0
        while s[0] in alias:
            s = alias[s[0]]
            seen += 1
            if seen > 10000:
                _refuse("design.lines", "a loop of identity blocks", "ALGEBRAIC_LOOP")
        return s
    out = []
    for l in lines:
        if l["dst"] in removed:
            continue
        if l["src"] in alias:
            s_, sp = res((l["src"], int(l.get("sport", 1))))
            l = {k: v for k, v in l.items() if k != "sport"}
            l["src"] = s_
            if sp != 1:
                l["sport"] = sp
        out.append(l)
    info.setdefault("alias", {}).update({k: res(v) for k, v in alias.items()})
    for key in ("declared", "equal_types", "conj_real", "math_identity"):
        info[key] = [tuple(res(x) if isinstance(x, tuple) else x for x in e) for e in info.get(key, [])]
    return dict(spec, blocks=blocks, lines=out)


def declared_on_cycle(src, info):
    """The first explicitly typed Signal Specification (info "declared") whose source block lies on a cycle of the
    compiled graph (src: block -> list of (source block, port) or source names), or None."""
    def preds(n):
        out = []
        for x in src.get(n) or []:
            if x is not None:
                out.append(x[0] if isinstance(x, tuple) else x)
        return out
    for n, s, _txt in info.get("declared", []):
        start = s[0]
        seen, todo = set(), list(preds(start))
        while todo:
            m = todo.pop()
            if m == start:
                return n
            if m in seen:
                continue
            seen.add(m)
            todo.extend(preds(m))
    return None


def post_compile(d, info, port_block):
    import fxgraph_lib as G
    for n, s, txt in info.get("declared", []):
        t = d.otype.get(port_block(s[0], s[1], "design.blocks." + n))
        want = G.BOOL if txt == "boolean" else G._fixdt(txt, "design.blocks.%s.OutDataTypeStr" % n)
        if want is None:
            _refuse("design.blocks.%s.OutDataTypeStr" % n, "Signal Specification type %r (fixdt, an integer type or "
                    "boolean)" % txt)
        if t != want:
            _refuse("design.blocks." + n, "Signal Specification declares %s, the input is %s (Simulink: "
                    "InputPortDataTypeMismatch)" % (txt, d.type_str(port_block(s[0], s[1], "design.blocks." + n))),
                    "TYPE_MISMATCH")
    for n, a, b in info.get("equal_types", []):
        ta = d.otype.get(port_block(a[0], a[1], "design.blocks." + n))
        tb = d.otype.get(port_block(b[0], b[1], "design.blocks." + n))
        if ta != tb:
            _refuse("design.blocks." + n, "Manual Switch inputs of the types %s and %s: the output type rule for "
                    "different input types is not determined by the recordings"
                    % (G._tstr(ta) if ta not in (None, G.BOOL) else ta, G._tstr(tb) if tb not in (None, G.BOOL) else tb))
    for n, s in info.get("conj_real", []):
        t = d.otype.get(port_block(s[0], s[1], "design.blocks." + n))
        if t == G.BOOL or (t is not None and not t[0]):
            _refuse("design.blocks." + n, "Math Function conj of an unsigned or boolean input (Simulink: "
                    "PortValidationInputError)", "PORT_TYPE_INVALID")
    for n, s, op in info.get("math_identity", []):
        t = d.otype.get(port_block(s[0], s[1], "design.blocks." + n))
        _math_l18_type(n, op, t)


def _math_l18_type(n, op, t):
    """The recordings operators take the recorded input types only; the recorded Simulink rejections are
    PORT_TYPE_INVALID."""
    import fxgraph_lib as G
    where = "design.blocks." + n
    if t is None:
        _refuse(where, "the input type of Math Function %s is not known" % op)
    if (op, t) in MATH_L18_REJECTED or (op == "hermitian" and (t == G.BOOL or not t[0])):
        _refuse(where, "Math Function %s of %s: Simulink R2026b rejects this input type "
                "(Simulink:blocks:PortValidationInputError)"
                % (op, "boolean" if t == G.BOOL else G._tstr(t)), "PORT_TYPE_INVALID")
    if op == "transpose":
        return   # any fixed-point input and boolean: the input unchanged
    if op in ("square", "magnitude^2", "reciprocal") and t != G.BOOL:
        return   # any fixed-point input (replayed equal, 0 differences; transpose: sfix16_En1 too)
    if op == "rem":
        # Simulink runs rem on the integer classes int8 .. uint32 only (both signs, 15 / 15 equal)
        # and rejects every other type (fixdt(1,32,-3), sfix16_En1 ...: 13 / 13)
        if t == G.BOOL or t[2] != 0 or t[1] not in (8, 16, 32):
            _refuse(where, "Math Function rem of %s: Simulink accepts the integer classes int8 .. uint32 only "
                    "(PortValidationInputError)" % ("boolean" if t == G.BOOL else G._tstr(t)), "PORT_TYPE_INVALID")
        return
    if t not in MATH_L18_IN[op]:
        _refuse(where, "Math Function %s of %s: recorded for %s only"
                % (op, "boolean" if t == G.BOOL else G._tstr(t), ", ".join(G._tstr(x) for x in MATH_L18_IN[op])))


# ---------------------------------------------------------------- typed (fxhdl.typed_rewrite)
def typed_blocks(spec):
    return [n for n, b in spec.get("blocks", {}).items() if isinstance(b, dict) and (
        b.get("type") in TYPED or (b.get("type") == "Math" and math_op(b) in MATH_TYPED))]


def provisional(spec):
    """Placeholders with the right ports for the typed blocks until compile() knows their input types. Each has the
    output type of the block it stands for (Bias, Increment / Decrement and mod: their input's type; Bit Reduce:
    ufix1), so a loop through one can be typed with the placeholder in place (typed_rewrite's probe) -- a
    placeholder is never left in a compiled design (a Bias in a loop
    stayed a Gain 1 of an inherited type and computed 0, 1, 2, 3 for 0, 2, 4, 6)."""
    blocks = dict(spec["blocks"])
    for n in typed_blocks(spec):
        t = blocks[n]["type"]
        if t == "Ground":
            blocks[n] = {"type": "Constant", "Value": "0", "OutDataTypeStr": "fixdt(1,8,0)"}
        elif t == "Math" and math_op(blocks[n]) in ("mod", "rem"):   # two inputs, the first input's type
            blocks[n] = {"type": "Sum", "Inputs": "++", "OutDataTypeStr": "Inherit: Same as first input"}
        elif t == "Math":   # square / magnitude^2 / reciprocal: one input, the type they will have
            od = str(blocks[n].get("OutDataTypeStr", "Inherit: Same as first input")).strip()
            blocks[n] = {"type": "Gain", "Gain": "1", "OutDataTypeStr": "Inherit: Same as input"
                         if od == "Inherit: Same as first input" else od}
        elif t == "BitReduce":   # ufix1, as recorded; a Compare To Constant may only
            # output boolean / uint8, so it could not stand for it -- every probe through a Bit Reduce loop failed
            blocks[n] = {"type": "DataTypeConversion", "OutDataTypeStr": "fixdt(0,1,0)", "RndMeth": "Floor",
                         "SaturateOnIntegerOverflow": "off"}
        else:
            blocks[n] = {"type": "Gain", "Gain": "1", "OutDataTypeStr": "Inherit: Same as input"}
    return dict(spec, blocks=blocks)


def _fx(t):
    return "fixdt(%d,%d,%d)" % t


def _const(nm, v, t):
    """A Constant of type t holding the exact value v. The literal is written as m*2^e (m odd), which MATLAB and the
    engine evaluate exactly as long as m fits a double's 53 bits -- whatever t's word length (Bias /
    Increment / Decrement on 63 .. 128-bit words, refused before for the word length alone); a value that needs more
    significant bits is refused."""
    v = Fraction(v)
    if v == 0:
        return {"type": "Constant", "Value": "0", "OutDataTypeStr": _fx(t)}
    e = 0
    while v.denominator != 1:
        v *= 2
        e -= 1
    m = v.numerator
    while m % 2 == 0:
        m //= 2
        e += 1
    if abs(m) >= 1 << 53:
        _refuse("design.blocks." + nm, "a constant with more than 53 significant bits (its literal would not be "
                "exact)")
    return {"type": "Constant", "Value": "%d*2^%d" % (m, e) if e else str(m), "OutDataTypeStr": _fx(t)}


def _loop_unanchored(blocks, lines, n):
    """True when block n sits on a feedback loop in which no block has an explicit (non-inherited) output type and
    which is not the recorded shape: the loop's types then come from the engine's loop seed, not from anything
    Simulink was seen doing. The recorded shape is accepted (below)."""
    fwd, bwd = {}, {}
    for l in lines:
        fwd.setdefault(l["src"], set()).add(l["dst"])
        bwd.setdefault(l["dst"], set()).add(l["src"])

    def reach(start, nxt):
        seen, stack = set(), list(nxt.get(start, ()))
        while stack:
            y = stack.pop()
            if y in seen:
                continue
            seen.add(y)
            stack += list(nxt.get(y, ()))
        return seen
    loop = reach(n, fwd) & reach(n, bwd)
    if n not in loop:
        return False
    for y in loop - {n}:
        od = str((blocks.get(y) or {}).get("OutDataTypeStr", "Inherit")).strip()
        if od and not od.startswith("Inherit"):
            return False
    # the recorded shape (19 / 19 PASS in Simulink, no warnings), wired exactly so:
    # outside -> Sum port 1, Sum -> block port 1, block -> Unit Delay, Unit Delay -> Sum port 2; the Sum "++" of the
    # internal rule or 'Inherit: Same as first input'. Its types and parameters are checked after the probe
    # (_recorded_loop; recordings: the structure alone let unrecorded types, parameters and wirings through)
    others = sorted(loop - {n}, key=lambda y: (blocks.get(y) or {}).get("type", ""))
    kinds = [(blocks.get(y) or {}).get("type") for y in others]
    if kinds == ["Sum", "UnitDelay"]:
        sm, dl = others
        sb = blocks[sm]
        od = str(sb.get("OutDataTypeStr", "Inherit: Inherit via internal rule")).strip()
        into = {(l["dst"], int(l.get("dport", 1))): l["src"] for l in lines}
        wired = (str(sb.get("Inputs", "++")).strip() == "++" and into.get((sm, 2)) == dl and
                 into.get((sm, 1)) not in loop and into.get((n, 1)) == sm and into.get((dl, 1)) == n)
        # the Sum and the Unit Delay as recorded: no accumulator type, rounding or saturation of their own, an
        # initial value 0 (those were not restricted)
        db_ = blocks[dl]
        # a parameter written out at the engine's default value (a saved .slx carries AccumDataTypeStr,
        # RndMeth and SaturateOnIntegerOverflow) is the recorded Sum too
        import fxgraph_lib as G
        sdef = dict(G.DEFAULTS["Sum"], InputSameDT="off")   # InputSameDT: the .slx reader keeps it (off)
        extra = {k for k in sb if k not in ("type", "Inputs", "OutDataTypeStr")}
        wired = wired and all(k in sdef and str(sb[k]).strip() == sdef[k] for k in extra) and \
            set(db_) <= {"type", "InitialCondition", "SampleTime"} and \
            str(db_.get("InitialCondition", "0")).strip() in ("0", "0.0") and \
            str(db_.get("SampleTime", "-1")).strip() in ("-1", "1")
        if wired and od in ("Inherit: Inherit via internal rule", "Inherit: Same as first input"):
            return od
    return True


# the configurations recorded in inheritance-typed loops: (block type, Sum output rule, input type, parameter) --
# parameter: (Bias value, saturation) for a Bias, the Constant divisor for mod, None otherwise.
# recordings (19 / 19 PASS) and recordings (25 / 25 PASS), both in Simulink without warnings
_IR, _SF = "Inherit: Inherit via internal rule", "Inherit: Same as first input"
_INT8, _S12E4, _INT16 = (1, 8, 0), (1, 12, 4), (1, 16, 0)
_LOOP_RECORDED = set()
for _t in ("Bias", "IncrementRealWorld", "IncrementStoredInteger", "DecrementRealWorld", "DecrementStoredInteger"):
    _pp = (Fraction(1), "off") if _t == "Bias" else None
    for _m in (_IR, _SF):
        _LOOP_RECORDED.add((_t, _m, _INT8, _pp))                                     # 
    _LOOP_RECORDED.add((_t, _SF, _S12E4, _pp))                                       # 
for _m in (_IR, _SF):
    _LOOP_RECORDED.update({("Math", _m, _INT8, Fraction(5)), ("Math", _m, _INT16, Fraction(5))})   # 
for _ty in ((0, 8, 0), (0, 16, 0), (1, 16, 0), (1, 16, 8)):                          # 
    for _m in (_IR, _SF):
        _LOOP_RECORDED.update({("Bias", _m, _ty, (Fraction(1), "off")), ("IncrementStoredInteger", _m, _ty, None)})
_LOOP_RECORDED.update({("Bias", _SF, (1, 10, 3), (Fraction(1), "off")), ("IncrementStoredInteger", _SF, (1, 10, 3), None)})
for _bv in (Fraction(-3), Fraction(2), Fraction(1, 2)):
    _LOOP_RECORDED.add(("Bias", _IR, _INT8, (_bv, "off")))
_LOOP_RECORDED.add(("Bias", _IR, _INT8, (Fraction(1), "on")))
_LOOP_RECORDED.update({("Math", _IR, (0, 8, 0), Fraction(5)), ("Math", _IR, _INT8, Fraction(3)),
                       ("Math", _IR, _INT16, Fraction(7))})


def _recorded_loop(where, t, b, ty, mode, blocks, lines, n):
    """Refuses an inheritance-typed loop whose block, parameters and input type were not among the recorded ones."""
    if t == "Bias":
        par = (_num(b.get("Bias", "0"), where + ".Bias"), str(b.get("SaturateOnIntegerOverflow", "off")))
    elif t == "Math":
        dsrc = next((l["src"] for l in lines if l["dst"] == n and int(l.get("dport", 1)) == 2), None)
        db = blocks.get(dsrc, {}) if dsrc else {}
        par = _num(db.get("Value", ""), where) if db.get("type") == "Constant" else None
    else:
        par = None
    ok_ = (t, mode, ty, par) in _LOOP_RECORDED
    if not ok_:
        import fxgraph_lib as G
        _refuse(where, ('%s of %s in a loop typed by inheritance only '
            '(Sum %s): this configuration was not recorded; '
            'give a block in the loop an explicit output type') % (t, G._tstr(ty), mode))


def typed_rewrite(spec, probe, hw=None):
    """probe(spec, [(block, dport)]) -> input types. Returns the spec with the typed blocks rewritten. hw: the
    hardware settings (the reciprocal's internal rule rounds to the host word)."""
    import fxgraph_lib as G
    tb = typed_blocks(spec)
    if not tb:
        return spec
    blocks = dict(spec["blocks"])
    lines = list(spec.get("lines", []))
    import parameter_bounds
    pnotes = dict(spec.get(parameter_bounds.NOTES_KEY) or {})
    for n in tb:
        b = blocks[n]
        t = b["type"]
        where = "design.blocks." + n
        cur = dict(spec, blocks=blocks, lines=lines)
        src = {int(l.get("dport", 1)): (l["src"], int(l.get("sport", 1))) for l in lines if l["dst"] == n}
        x = INTERNAL + n + "/b/"
        new, nl = {}, []
        probed = []   # (input port, probed type) the rewrite relies on
        shape = _loop_unanchored(blocks, lines, n) if t != "Ground" else False
        if shape is True:
            _refuse(where, "%s inside a feedback loop whose types come from inheritance only (no block in the loop "
                    "has an explicit output type): not recorded; give a block in the loop an "
                    "explicit output type" % t)

        def w(s_, d_, port=1):
            if isinstance(s_, tuple):
                nl.append(dict({"src": s_[0], "dst": d_, "dport": port}, **({"sport": s_[1]} if s_[1] != 1 else {})))
            else:
                nl.append({"src": s_, "dst": d_, "dport": port})
        if t == "Ground":
            users = [l for l in lines if l["src"] == n]
            ok_ = len(users) == 1 and blocks.get(users[0]["dst"], {}).get("type") == "Sum"
            if not ok_:
                _refuse(where, "Ground feeding anything but one Sum input: its type comes from back-propagation, "
                        "recorded only into a Sum")
            s_ = users[0]["dst"]
            others = sorted(int(l.get("dport", 1)) for l in lines if l["dst"] == s_ and l["src"] != n)
            if not others:
                _refuse(where, "Ground into a Sum without another input")
            sb = blocks[s_]
            sins = str(sb.get("Inputs", "++")).strip()
            if (len(sins) if not sins.isdigit() else int(sins)) != 2 or \
                    str(sb.get("OutDataTypeStr", "Inherit: Inherit via internal rule")).strip() != \
                    "Inherit: Inherit via internal rule":
                # recorded: a two-input Sum of inherited output type; its compiled type in a
                # wider or explicitly typed Sum was not
                _refuse(where, ('Ground into a Sum other than two-input with an inherited '
                    'output type (only that configuration was recorded)'))
            up, stack = set(), [l["src"] for l in lines if l["dst"] == s_ and l["src"] != n]
            while stack:
                y = stack.pop()
                if y in up:
                    continue
                up.add(y)
                stack += [l["src"] for l in lines if l["dst"] == y]
            if s_ in up:
                _refuse(where, "Ground into a Sum inside a feedback loop (not recorded)")
            ts = probe(cur, [(s_, others[0])])
            if not ts or ts[0] is None or ts[0] == G.BOOL:
                _refuse(where, "the type of the Sum's other input is not known")
            new[n] = _const(n, 0, ts[0])
        elif t == "Math" and math_op(b) == "mod":
            ts = probe(cur, [(n, 1), (n, 2)])
            if not ts or None in ts:
                _refuse(where, "the input types of Math mod are not known before its rewrite (a source typed by "
                        "back propagation or an internal rule resolved later, or a loop whose "
                        "types depend on the order of resolution); give the source an explicit output type")
            for tt in ts:
                if tt == G.BOOL or tt[2] != 0 or tt[1] not in (8, 16, 32):
                    _refuse(where, "Math Function mod of %s: only the integer classes int8 .. uint32 are accepted "
                            "(Simulink: PortValidationInputError)" % (tt,), "PORT_TYPE_INVALID")
            if ts[0] != ts[1]:
                _refuse(where, "Math Function mod of %s and %s (one integer class recorded)"
                        % (G._tstr(ts[0]), G._tstr(ts[1])), "TYPE_MISMATCH")
            probed = [(1, ts[0]), (2, ts[1])]
            if isinstance(shape, str):
                _recorded_loop(where, t, b, ts[0], shape, blocks, lines, n)
            dsrc = src.get(2)
            db = blocks.get(dsrc[0], {}) if dsrc else {}
            if db.get("type") != "Constant":
                # a signal divisor, int8 .. int32 / uint8 .. uint16, 12 / 12 equal -- u - floor(u /
                # v) * v, and mod(u, 0) = u
                dv = None
            else:
                dv = _num(db.get("Value", ""), "design.blocks.%s.Value" % dsrc[0])
                if dv.denominator != 1:
                    _refuse(where, "a non-integer divisor")
            p = params(n, b)
            # recordings (recordings xmm*): Simulink ignores OutDataTypeStr for mod -- fixdt(1,16,0) or the internal
            # rule, the output keeps the input's integer class (as rem does); every one of those records equal
            ty = ts[0]
            s, wl = ty[0], ty[1]
            if dv == 0:
                new[n] = {"type": "DataTypeConversion", "OutDataTypeStr": _fx(ty), "RndMeth": "Floor",
                          "SaturateOnIntegerOverflow": "off"}   # mod(x, 0) = x
                w(src[1], n)
            elif dv is None:
                q, pr, r, rc, nz = x + "q", x + "p", x + "r", x + "rc", x + "nz"
                new[q] = {"type": "Product", "Inputs": "*/", "OutDataTypeStr": _fx((1, wl + 2, 0)), "RndMeth": "Floor",
                          "SaturateOnIntegerOverflow": "off"}
                w(src[1], q, 1)
                w(dsrc, q, 2)
                new[pr] = {"type": "Product", "Inputs": "**", "OutDataTypeStr": _fx((1, 2 * wl + 3, 0)),
                           "RndMeth": "Floor", "SaturateOnIntegerOverflow": "off"}
                w(dsrc, pr, 1)
                w(q, pr, 2)
                new[r] = {"type": "Sum", "Inputs": "+-", "OutDataTypeStr": _fx((1, 2 * wl + 4, 0)),
                          "AccumDataTypeStr": _fx((1, 2 * wl + 4, 0)), "RndMeth": "Floor",
                          "SaturateOnIntegerOverflow": "off"}
                w(src[1], r, 1)
                w(pr, r, 2)
                new[rc] = {"type": "DataTypeConversion", "OutDataTypeStr": _fx(ty), "RndMeth": "Floor",
                           "SaturateOnIntegerOverflow": "off"}   # exact: |r| < |v|, the sign of v
                w(r, rc)
                new[nz] = {"type": "CompareToConstant", "relop": "~=", "const": "0", "OutDataTypeStr": "boolean"}
                w(dsrc, nz)
                new[n] = {"type": "Switch", "Criteria": "u2 ~= 0", "Threshold": "0"}   # mod(u, 0) = u
                w(rc, n, 1)
                w(nz, n, 2)
                w(src[1], n, 3)
            else:
                q, pr, r = x + "q", x + "p", x + "r"
                new[q] = {"type": "Product", "Inputs": "*/", "OutDataTypeStr": _fx((1, wl + 2, 0)), "RndMeth": "Floor",
                          "SaturateOnIntegerOverflow": "off"}
                w(src[1], q, 1)
                w(dsrc, q, 2)
                new[pr] = {"type": "Product", "Inputs": "**", "OutDataTypeStr": _fx((1, 2 * wl + 3, 0)),
                           "RndMeth": "Floor", "SaturateOnIntegerOverflow": "off"}
                w(dsrc, pr, 1)
                w(q, pr, 2)
                new[r] = {"type": "Sum", "Inputs": "+-", "OutDataTypeStr": _fx((1, 2 * wl + 4, 0)),
                          "AccumDataTypeStr": _fx((1, 2 * wl + 4, 0)), "RndMeth": "Floor",
                          "SaturateOnIntegerOverflow": "off"}
                w(src[1], r, 1)
                w(pr, r, 2)
                new[n] = {"type": "DataTypeConversion", "OutDataTypeStr": _fx(ty), "RndMeth": "Floor",
                          "SaturateOnIntegerOverflow": "off"}   # exact: 0 <= r < |d| or |d| < r <= 0
                w(r, n)
                _ = s
        elif t == "Math":   # square, magnitude^2, reciprocal, rem
            op = math_op(b)
            nin = 2 if op == "rem" else 1
            for k in range(1, nin + 1):
                if k not in src:
                    _refuse(where, "input %d is not connected" % k, "BAD_CONNECTION")
            ts = probe(cur, [(n, k) for k in range(1, nin + 1)])
            if (not ts or None in ts) and _from_complex(cur, n):
                # Math square / magnitude^2 of a complex signal reached an internal error (a probe of the
                # complex signal has no real type); complex Math is not modelled (Simulink accepts it)
                _refuse(where, "Math Function %s of a complex signal is not supported (complex signals: Sum, Product, "
                        "Gain, Unit Delay, conj and the Real-Imag blocks)" % op)
            if not ts or None in ts:
                _refuse(where, "the input types of Math Function %s are not known before its rewrite (a source "
                        "typed by back propagation or an internal rule resolved later, or a loop "
                        "whose types depend on the order of resolution); give the source an explicit output type" % op)
            for tt in ts:
                _math_l18_type(n, op, tt)
            if op == "rem" and ts[1] != ts[0]:
                _refuse(where, "Math Function rem of %s and %s (recorded with one type on both inputs)"
                        % (G._tstr(ts[0]), G._tstr(ts[1])))
            probed = [(k + 1, tt) for k, tt in enumerate(ts)]
            if isinstance(shape, str):
                _recorded_loop(where, t, b, ts[0], shape, blocks, lines, n)
            p = params(n, b)
            rnd, sat = p["RndMeth"], p["SaturateOnIntegerOverflow"]
            # recordings (recordings xmm*): rem of integers is exact -- RndMeth and saturation do not matter (Simulink
            # ran Nearest / Round / Simplest and saturation off, equal)
            od = p["OutDataTypeStr"]
            ty = ts[0]
            internal = od == "Inherit: Inherit via internal rule"
            if od not in MATH_L18_OUT and not internal and G._fixdt(od, where + ".OutDataTypeStr") is None:
                _refuse(where + ".OutDataTypeStr", "Math Function %s with OutDataTypeStr %r: recorded with %s, an "
                        "explicit fixdt or the internal rule" % (op, od, " / ".join(MATH_L18_OUT)))
            # rem keeps the input type whatever OutDataTypeStr says (recorded: fixdt(1,24,12) gave uint8)
            out = ty if od == "Inherit: Same as first input" or op == "rem" else None if internal else \
                G._fixdt(od, where + ".OutDataTypeStr")
            if internal and op == "reciprocal":
                # reciprocal's internal rule keeps the input's signedness and word length with
                # fraction length wl - 1 - s - fl (sfix16_En8 -> sfix16_En6, ufix8 -> ufix8_En7, sfix32_En3 ->
                # sfix32_En27, ufix16_En15 -> uint16, sfix8_En8 -> sfix8_E2, sfix8_E7 -> sfix8_En13)
                out = (ty[0], ty[1], ty[1] - 1 - ty[0] - ty[2])
                # recordings (28 / 28 host odd widths; recordings re-check: sfix12 -> sfix16_En10): on host
                # hardware the word length rounds up to the next native word (8 / 16 / 32), the fraction length stays
                # wl - 1 - s - fl of the input's word; ASIC / FPGA (39 / 39), native widths and 33 bits keep the input's
                words = [w_ for w_ in G.hw_words(hw) if w_ >= ty[1]]
                if words:
                    out = (ty[0], min(words), out[2])
            if op in ("square", "magnitude^2"):
                # the exact square, rounded to floor in the output type and saturated (magnitude^2 of a real
                # signal is its square); recorded 22 / 22 steps
                new[n] = {"type": "Product", "Inputs": "**", "OutDataTypeStr": "Inherit: Inherit via internal rule"
                          if out is None else _fx(out), "RndMeth": rnd, "SaturateOnIntegerOverflow": sat}
                w(src[1], n, 1)
                w(src[1], n, 2)
            elif op == "reciprocal":
                # 1 / u exact, floor, saturated; 1 / 0 the output type's maximum (recorded) -- the engine's division
                # by zero saturates by the dividend's sign, here +1
                one = x + "one"
                new[one] = _const(one, 1, (0, 1, 0))
                new[n] = {"type": "Product", "Inputs": "*/", "OutDataTypeStr": "Inherit: Inherit via internal rule"
                          if out is None else _fx(out), "RndMeth": rnd, "SaturateOnIntegerOverflow": sat}
                w(one, n, 1)
                w(src[1], n, 2)
            else:   # rem of integers: u - fix(u / v) * v; rem(u, 0) = 0 (signed too)
                wl = ty[1]
                q, pr, r, rc, z, nz = x + "q", x + "p", x + "r", x + "rc", x + "zero", x + "nz"
                # the quotient exact before truncation toward zero, one bit wider (int16 -32768 / -1 = 32768)
                new[q] = {"type": "Product", "Inputs": "*/", "OutDataTypeStr": _fx((1, wl + 2, 0)), "RndMeth": "Zero",
                          "SaturateOnIntegerOverflow": "off"}
                w(src[1], q, 1)
                w(src[2], q, 2)
                new[pr] = {"type": "Product", "Inputs": "**", "OutDataTypeStr": _fx((1, 2 * wl + 3, 0)),
                           "RndMeth": "Floor", "SaturateOnIntegerOverflow": "off"}
                w(src[2], pr, 1)
                w(q, pr, 2)
                new[r] = {"type": "Sum", "Inputs": "+-", "OutDataTypeStr": _fx((1, 2 * wl + 4, 0)),
                          "AccumDataTypeStr": _fx((1, 2 * wl + 4, 0)), "RndMeth": "Floor",
                          "SaturateOnIntegerOverflow": "off"}
                w(src[1], r, 1)
                w(pr, r, 2)
                new[rc] = {"type": "DataTypeConversion", "OutDataTypeStr": _fx(ty), "RndMeth": "Floor",
                           "SaturateOnIntegerOverflow": "off"}   # exact: |r| < |v|, the sign of u
                w(r, rc)
                new[z] = _const(z, 0, ty)
                new[nz] = {"type": "CompareToConstant", "relop": "~=", "const": "0", "OutDataTypeStr": "boolean"}
                w(src[2], nz)
                new[n] = {"type": "Switch", "Criteria": "u2 ~= 0", "Threshold": "0"}
                w(rc, n, 1)
                w(nz, n, 2)
                w(z, n, 3)
        else:
            ts = probe(cur, [(n, 1)])
            if not ts or ts[0] is None:
                # recordings #4: the message named a loop where there was none (a back-propagated DTC or an
                # internal-rule Sum feeding the block)
                _refuse(where, "the input type of %s is not known before its rewrite: its source takes its type from "
                        "back propagation or an internal rule resolved later (or a loop whose types depend on the order "
                        "of resolution); give the source an explicit output type" % t)
            ty = ts[0]
            if ty == G.BOOL:
                _refuse(where, "%s of a boolean input (not recorded)" % t)
            probed = [(1, ty)]
            if isinstance(shape, str):
                _recorded_loop(where, t, b, ty, shape, blocks, lines, n)
            s, wl, fl = ty
            lsb = Fraction(1, 2 ** fl) if fl >= 0 else Fraction(2 ** -fl)
            lo, hi = (-(1 << (wl - 1)), (1 << (wl - 1)) - 1) if s else (0, (1 << wl) - 1)
            p = params(n, b)
            if t == "BitReduce":
                mode = p["mode"]
                cmp = x + "c"
                if mode in ("AND", "OR"):
                    allones = -1 if s else (1 << wl) - 1
                    cv = (allones if mode == "AND" else 0) * lsb
                    new[cmp] = {"type": "CompareToConstant", "relop": "==" if mode == "AND" else "~=",
                                "const": repr(float(cv)), "OutDataTypeStr": "boolean"}
                    w(src[1], cmp)
                else:   # XOR: the parity of the wl bits of the pattern
                    # the pattern as an unsigned integer first (a signed one-bit field would widen to two bits)
                    pat = x + "pat"
                    new[pat] = {"type": "DataTypeConversion", "OutDataTypeStr": "fixdt(0,%d,0)" % wl,
                                "ConvertRealWorld": "Stored Integer (SI)", "RndMeth": "Floor",
                                "SaturateOnIntegerOverflow": "off"}
                    w(src[1], pat)
                    bits = []
                    for k in range(wl):
                        e, z = x + "e%d" % k, x + "z%d" % k
                        new[e] = {"type": "ExtractBits", "bitsToExtract": "Range of bits",
                                  "bitIdxRange": "[%d %d]" % (k, k), "outScalingMode": "Treat bit field as an integer"}
                        w(pat, e)
                        new[z] = {"type": "CompareToConstant", "relop": "~=", "const": "0",
                                  "OutDataTypeStr": "boolean"}
                        w(e, z)
                        bits.append(z)
                    if len(bits) == 1:
                        cmp = bits[0]
                    else:
                        new[cmp] = {"type": "LogicalOperator", "Operator": "XOR", "Inputs": str(len(bits))}
                        for k, z in enumerate(bits):
                            w(z, cmp, k + 1)
                new[n] = {"type": "DataTypeConversion", "OutDataTypeStr": "fixdt(0,1,0)", "RndMeth": "Floor",
                          "SaturateOnIntegerOverflow": "off"}
                w(cmp, n)
            else:
                sat = "off"
                if t == "Bias":
                    bv = _num(p["Bias"], where + ".Bias")
                    q_ = bv / lsb
                    # a tie rounds up on both sides: the positive half was recorded, the negative
                    # half too (31 cases, 372 / 372 samples half up) -- no refusal for it any more
                    qi = (q_.numerator * 2 + q_.denominator) // (2 * q_.denominator)   # nearest, half up
                    # recordings (max + 0.5 / + 1 LSB run in Simulink, 4 cases;
                    # its 960-case grid: every parameter overflows iff <= min - 1 LSB or > max + 1 LSB)
                    # the one parameter-overflow rule (PORT_TYPE_INVALID, simulink_rejects), and only
                    # under ParameterOverflowMsg 'error' -- with 'warning' / 'none' Simulink saturates (it was
                    # BAD_PARAM and refused whatever the setting)
                    G._param_overflow(bv, ty, where + ".Bias", "Bias %s: Bias" % n,
                                      mode=(hw or {}).get("ParameterOverflowMsg", G.PARAM_OVERFLOW_DEFAULT))
                    # recordings (24 models): inside the line Simulink runs with the parameter saturated
                    # to the input type's limit (runtime parameter = input type, precision-loss warning) -- max +
                    # 0.5 / + 1 LSB gives max, min - 0.5 LSB gives min (also its negative half: both roundings)
                    out_ = qi < lo or qi > hi
                    qi = min(max(qi, lo), hi)
                    delta = qi * lsb
                    if delta != bv:   # the value used is reported (parameter_quantization)
                        pnotes[where + ".Bias"] = {"written": G._fmt_num(bv), "type": G._tstr(tuple(ty)),
                                                   "used": G._fmt_num(delta),
                                                   "why": "saturated to the type's bound" if out_ else
                                                   "rounded to the type's precision"}
                    sat = p["SaturateOnIntegerOverflow"]
                    sign = "+"
                elif t in ("IncrementRealWorld", "DecrementRealWorld"):
                    # show (f13 / w3 / px, 23 single-block designs and 15 Simulink stops):
                    # 1.0 is put in the input type like a Bias -- nearest, half up (LSB 2: one LSB; LSB 4 and more:
                    # 0, the input unchanged), saturated to the type's maximum (no integer bits: 1.0 = max + 1 LSB,
                    # max added / subtracted); 1.0 above max + 1 LSB (negative integer bits) stops Simulink
                    q_ = Fraction(1) / lsb
                    if q_ > hi + 1 and (hw or {}).get("ParameterOverflowMsg", G.PARAM_OVERFLOW_DEFAULT) == "error":
                        _refuse(where, ("%s of %s: 1.0 is beyond the input type's maximum + 1 "
                            'LSB, where Simulink stops (fxpParameterOverflow under '
                            'the default diagnostics); give the input integer bits') % (t, G._tstr(ty)), "PORT_TYPE_INVALID")
                    qi = min((q_.numerator * 2 + q_.denominator) // (2 * q_.denominator), hi)
                    delta = qi * lsb
                    sign = "+" if t.startswith("Increment") else "-"
                else:
                    delta = lsb
                    sign = "+" if t.startswith("Increment") else "-"
                k_ = x + "k"
                new[k_] = _const(n, delta, ty)
                new[n] = {"type": "Sum", "Inputs": "+" + sign, "OutDataTypeStr": _fx(ty), "RndMeth": "Floor",
                          "SaturateOnIntegerOverflow": sat}
                w(src[1], n, 1)
                w(k_, n, 2)
        # the probed input types must be the ones the rewritten design compiles to: a Data Type Duplicate ties each
        # input to a constant of its probed type (TYPE_MISMATCH otherwise) -- the rewrite was built for them
        for port, pt in probed:
            if pt is None or pt == G.BOOL or port not in src:
                continue
            chk, ref = x + "chk%d" % port, x + "ref%d" % port
            new[ref] = _const(ref, 0, pt)
            new[chk] = {"type": "DataTypeDuplicate", "NumInputPorts": "2"}
            w(src[port], chk, 1)
            w(ref, chk, 2)
        lines = [l for l in lines if l["dst"] != n] + nl
        blocks.update(new)
    return dict(spec, blocks=blocks, lines=lines, **({parameter_bounds.NOTES_KEY: pnotes} if pnotes else {}))

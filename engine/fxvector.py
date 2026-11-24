"""v1 structural blocks: vector signals, Mux, Demux, Selector -- by scalarisation.

A design may carry vector signals built by Mux (and split by Demux / Selector). This module rewrites it into a
scalar design: every block that carries a vector of width w becomes w scalar copies named "<block>[k]" (k from
1), each wired to element k of its inputs (a scalar input is broadcast to every element); Mux, Demux and Selector
become pure wiring. Elementwise semantics are assumed for the whitelisted blocks; a block whose vector behaviour
is not elementwise (a one-input Sum or MinMax over a vector, vector Gain parameters, ...) is refused.
The copies of one block share its parameters, so their data types stay equal: info["ties"] maps every copy to
its first copy, and fxopt keeps one decision variable per original block (GraphProblem honours "_tie").
Multiple output ports are only allowed on Demux (sport); everything else is single-output as in the engine."""
import json  # noqa: E402
import fxgraph_lib as G
import fxnum  # noqa: E402
import blockspec  # noqa: E402

_CONST_KIND = blockspec.NUMERIC_VALUES["Constant"]["Value"]

INTERNAL = "\x1d"
ELEMENTWISE = ("ExtractBits", "Gain", "Sum", "Product", "UnitDelay", "Delay", "DataTypeConversion", "Saturation", "Abs",
               "UnaryMinus", "MinMax", "Switch", "RelationalOperator", "LogicalOperator", "Outport", "Inport",
               "Constant", "ShiftArithmetic", "CompareToConstant", "Rounding", "Signum", "DiscreteFir",
               "DataTypeDuplicate",   # a sink: one copy per element, each checking its elements' types
               # recordings (engine/block_lowering.py): element by element
               "SignalSpecification", "Reshape", "ManualSwitch", "Assertion", "Bias", "Math", "BitReduce",
               "IncrementRealWorld", "DecrementRealWorld", "IncrementStoredInteger", "DecrementStoredInteger",
               "MultiportSwitch",   # a scalar control picks the same port for every element
               "EnabledOutport",    # element by element, InitialOutput per element
               # element by element in Simulink
               "BitSet", "BitClear", "BitwiseOperator", "Repeat")
FOREACH_RECORDED = ("Inport", "Outport", "Gain", "Sum", "UnitDelay", "Saturation", "Constant", "ForEach")
VEC_WIDTH_TYPES = ("TappedDelay", "TappedDelayEnabledSynchronous", "Concatenate", "BusAssignment")
TAPPED = ("TappedDelay", "TappedDelayEnabledSynchronous")   # a scalar in, a vector out
DEMUX_UNEVEN = ((4, 3),)   # recordings: 2 + 1 + 1
TAPPED_COUNTS = (1, 2, 3, 4, 5, 8)   # Tapped Delay NumDelays recorded equal (recordings 3 / 4; recordings)
STRUCT = ("Mux", "Demux", "Selector", "BusCreator", "BusSelector")
# virtual buses: a Bus Creator is a Mux whose elements have names (ElementNames, the signal
# names on its input lines in a model); a Bus Selector picks elements by name -- one output port per element, or one
# sub-bus (OutputAsBus on). A bus may pass through Unit Delays (every element delayed) and subsystem boundaries;
# any other block on a bus is refused (Simulink accepts buses only on routing and bus-capable blocks)
BUS_THROUGH = ("UnitDelay", "Outport")
# parameters that may be vectors: element k of the block takes element k
VECPARAM = {"Gain": ("Gain",), "UnitDelay": ("InitialCondition",), "EnabledOutport": ("InitialOutput",),
            "Saturation": ("UpperLimit", "LowerLimit"), "Constant": ("Value",),
            "ShiftArithmetic": ("BitShiftNumber",),   # per element
            "BitwiseOperator": ("BitMask",)}   # a vector mask, element k on element k
# one-input blocks over a vector that reduce it to a scalar (Simulink: sum / extremum along the elements)
COLLAPSE = ("Sum", "MinMax", "LogicalOperator", "BitwiseOperator")   # LogicalOperator: recordings (Inputs 1
# over a vector), recordings; BitwiseOperator: recordings (one input without a
# mask: reduced)


def _checked(text, where, kind="any"):
    """The one numeric parser (engine/fxnum.py): '[1_0 2 3]', '[1 2 3', non-ASCII digits were
    read by float() here and accepted, an expression crashed INTERNAL. Returns the canonical text."""
    c = fxnum.canonical(text, kind)
    if isinstance(c, fxnum.NumError):
        raise G.GraphError(c.code, where, c.message)
    return c


def _int1(text, where):
    """One integer structural parameter (NumDelays, Inputs, PortDimensions, DelayLength, NumInputs) through the one
    numeric parser (int() accepted '2_0' as 20 and non-ASCII digits)."""
    v = _ints(text, where)
    if len(v) != 1:
        raise G.GraphError("BAD_PARAM", where, "one integer required, got %r" % str(text)[:60])
    return v[0]


def _ints(text, where="design"):
    c = _checked(text, where)
    rows = _expand(c, where)
    if len(rows) > 1 and all(len(r) == 1 for r in rows):
        # a column ('[2;1]', '[1;2]') is run by Simulink -- valid, not modelled (as the
        # transposed spelling "[1 2]'")
        raise G.GraphError("UNSUPPORTED", where, "a column of integers %r is not supported here; write a row "
                           "('[a b ...]')" % str(text)[:60])
    if len(rows) > 1 or not rows or not rows[0]:
        # recordings: '[]' and matrices ('[1 1; 2 2]') are rejected by Simulink at set_param; the engine had
        # answered (no outputs / flattened)
        raise G.GraphError("BAD_PARAM", where, "a non-empty row of integers is required, got %r" % str(text)[:60])
    vals = [float(v) for r in rows for v in r]
    if any(v != v or v in (float("inf"), float("-inf")) for v in vals):
        # NaN / inf gave INTERNAL (Simulink rejects them too)
        raise G.GraphError("BAD_PARAM", where, "finite integer values required, got %r" % str(text)[:60])
    if any(v != int(v) for v in vals):
        raise G.GraphError("BAD_PARAM", where, "integer values required, got %r" % str(text)[:60])
    return [int(v) for v in vals]


FOREACH_OK = {"InputPartition": "on", "InputPartitionDimension": "1", "InputPartitionWidth": "1",
              "InputPartitionOffset": "0", "OutputConcatenationDimension": "1"}


def _drop_foreach(spec):
    """For Each subsystem: with every input partitioned element by element (dimension 1,
    width 1) and the outputs concatenated along dimension 1, its blocks act on each element with their own state --
    exactly the element-wise copies this pass makes of a vector signal. The ForEach marker block is checked and
    removed; a partitioned mask parameter is written as the vector parameter it stands for (Gain [k1 ... kn])."""
    fe = [n for n, b in spec["blocks"].items() if b.get("type") == "ForEach"]
    if not fe:
        return spec
    for n in fe:
        p = {k: str(v) for k, v in spec["blocks"][n].items() if k != "type"}
        for k, v in p.items():
            if k not in FOREACH_OK:
                raise G.GraphError("BAD_PARAM", "design.blocks.%s.%s" % (n, k), "unknown parameter %s for ForEach" % k)
            if v != FOREACH_OK[k]:
                raise G.GraphError("UNSUPPORTED", "design.blocks.%s.%s" % (n, k), "%s=%r (only %r: element-by-element "
                                   "partitioning is supported)" % (k, v, FOREACH_OK[k]))
        if "/" not in n:
            raise G.GraphError("BAD_PARAM", "design.blocks." + n, "a ForEach block must sit inside a subsystem level")
    return {"blocks": {k: v for k, v in spec["blocks"].items() if k not in fe},
            "lines": [l for l in spec.get("lines", []) if l.get("src") not in fe and l.get("dst") not in fe]}


# 'Multidimensional array' along dimension 1 of scalars / 1-D
# vectors gives an n x 1 matrix whose values and types equal Vector mode's through these downstream blocks (the
# matrix carries on through the first group, ends in a scalar at the second); anything else downstream is refused
_MD_CARRY = {"Gain": lambda b: str(b.get("Multiplication", "Element-wise(K.*u)")).startswith("Element-wise"),
             "Sum": lambda b: str(b.get("Inputs", "++")).strip() == "++",
             "UnitDelay": lambda b: True,
             "DataTypeConversion": lambda b: True,
             "Mux": lambda b: str(b.get("Inputs", "2")).strip() == "2"}
_MD_END = {"Outport": lambda b: True,
           "Sum": lambda b: str(b.get("Inputs", "")).strip() == "+",
           "Selector": lambda b: str(b.get("Indices", "")).strip().isdigit()}


def _md_dim1_cone(spec, n, where):
    seen, todo = set(), [n]
    while todo:
        x = todo.pop()
        for ln in spec.get("lines", []):
            if ln.get("src") != x:
                continue
            d = ln.get("dst")
            b = spec["blocks"].get(d) or {}
            t = b.get("type")
            if t in _MD_END and _MD_END[t](b):
                continue
            if t in _MD_CARRY and _MD_CARRY[t](b):
                if d not in seen:
                    seen.add(d)
                    todo.append(d)
                continue
            raise G.GraphError("UNSUPPORTED", where + ".Mode", "Concatenate in 'Multidimensional array' mode builds an "
                               "n x 1 matrix; it reaches %s (%s), which was not recorded with a matrix input "
                               "(Outport, element-wise Gain, Sum, Unit Delay, Data Type Conversion, "
                               "Mux of 2, Selector of one element)" % (d, t))


def _concat_to_mux(spec):
    """Vector Concatenate: the inputs side by side like a Mux, and every input must have
    the same data type (Simulink:DataType:InputPortDataTypeMismatch on sfix16_En8 + sfix12_En4 / + ufix8) -- an
    internal Data Type Duplicate on the inputs carries that check (engine/fxhdl.py). Multidimensional-array mode is
    refused (matrix signals)."""
    if not any(b.get("type") == "Concatenate" for b in spec["blocks"].values()):
        return spec
    blocks = dict(spec["blocks"])
    lines = list(spec.get("lines", []))
    for n, b in spec["blocks"].items():
        if b.get("type") != "Concatenate":
            continue
        where = "design.blocks." + n
        p = {k: str(v) for k, v in b.items() if k != "type"}
        extra = sorted(set(p) - {"NumInputs", "Mode", "ConcatenateDimension"})
        if extra:
            raise G.GraphError("BAD_PARAM", where + "." + extra[0], "unknown parameter %s for Concatenate" % extra[0])
        if p.get("Mode", "Vector") != "Vector":
            if p.get("Mode") != "Multidimensional array" or str(p.get("ConcatenateDimension", "1")).strip() != "1":
                raise G.GraphError("UNSUPPORTED", where + ".Mode", "Concatenate in %r mode, dimension %s, builds "
                                   "matrices (supported: 'Vector' mode, and 'Multidimensional array' along dimension "
                                   "1 into the recorded downstream blocks)" % (p["Mode"], p.get("ConcatenateDimension",
                                                                                                 "1")))
            _md_dim1_cone(spec, n, where)
        try:
            k = _int1(p.get("NumInputs", "2"), where + ".NumInputs")
        except ValueError:
            raise G.GraphError("BAD_PARAM", where + ".NumInputs", "integer number of inputs required")
        blocks[n] = {"type": "Mux", "Inputs": str(k)}
        dtd = INTERNAL + n + "/dtd"
        blocks[dtd] = {"type": "DataTypeDuplicate", "NumInputPorts": str(k)}
        for ln in spec.get("lines", []):
            if ln.get("dst") == n:
                lines.append(dict(ln, dst=dtd))
    return {"blocks": blocks, "lines": lines}


def _num(c, where):
    if isinstance(c, fxnum.NumError):
        raise G.GraphError(c.code, where, c.message)
    return c


def _width(b, n):
    """An Inport's signal width from PortDimensions (int() took '1_0' as 10, '3.0' / '[3 1]' crashed):
    one integer, [n], [n 1] or [1 n]; -1 (inherited) is one element."""
    w = int(_num(fxnum.dims(b.get("PortDimensions", "1")), "design.blocks.%s.PortDimensions" % n))
    return 1 if w == -1 else w


# matrix signals on element-wise blocks. An m x n signal is
# lowered to m*n element copies in column-major order (MATLAB's x(:)), the vector convention x[1] .. x[m*n]; matrix
# parameters are flattened the same way. Only these blocks were recorded with matrices; the support domain is checked
# after typing (fxgraph_lib, engine/matrix_domain.json) on the key each lowered element carries (_MatKey)
MATRIX_BLOCKS = ("Gain", "Sum", "Product", "Abs", "Saturation", "DataTypeConversion", "UnitDelay", "Constant", "Outport")


def _dims2(b, n):
    """(width, (m, n) for a matrix signal or None): PortDimensions '[m n]' with m, n > 1 is a matrix; everything
    else as _width."""
    t = str(b.get("PortDimensions", "1")).strip()
    if t.startswith("["):
        c = fxnum.canonical(t, "vector")
        if not isinstance(c, fxnum.NumError):
            els = c.strip("[]").split()
            if len(els) == 2:
                try:
                    m_, n_ = int(els[0]), int(els[1])
                except ValueError:
                    m_ = n_ = 0
                if m_ > 1 and n_ > 1:
                    return m_ * n_, (m_, n_)
    return _width(b, n), None


def _pmat(text):
    """The shape (rows, cols) of a parameter text that is a true matrix (more than one row and column), else None."""
    try:
        rows = _expand(_checked(text, "design"), "design") if str(text).strip().startswith("[") else []
    except G.GraphError:
        return None
    if len(rows) > 1 and len({len(r) for r in rows}) == 1 and len(rows[0]) > 1:
        return (len(rows), len(rows[0]))
    return None


def _collapse_form(b, t):
    """True for the one-input forms of Sum ('+', '1') and Product ('*', '1') that collapse their input."""
    if t == "Sum":
        return str(b.get("Inputs", "+")).replace("|", "").strip() in ("+", "1")
    return str(b.get("Inputs", "2")).strip() in ("*", "1")


def _collapse_mode(b, n):
    """(CollapseMode, CollapseDim as an int) of a collapsing Sum / Product."""
    mode = str(b.get("CollapseMode", "All dimensions")).strip()
    if mode not in ("All dimensions", "Specified dimension"):
        raise G.GraphError("BAD_PARAM", "design.blocks.%s.CollapseMode" % n, "CollapseMode %r is not a Simulink value "
                           "('All dimensions' or 'Specified dimension')" % mode)
    d = _num(fxnum.canonical(b.get("CollapseDim", "1"), "int"), "design.blocks.%s.CollapseDim" % n)
    return mode, int(d)


def _matrix_w(n, b, t, ins, ws, ish, psh, shape, key, mkey, mcollapse):
    """The width of block n when a matrix signal or a matrix parameter reaches it:
    element-wise blocks need every non-scalar operand to have the same matrix shape (Simulink rejects a 1-D vector
    or another shape: PortDimsMismatch; a Unit Delay's initial value: InvalidICForUnitDelaySampleBased...); a
    collapsing Sum / Product reduces all elements, the columns (CollapseDim 1: a 1 x n result) or the rows (2: m x 1)."""
    where = "design.blocks." + n
    if t not in MATRIX_BLOCKS:
        raise G.GraphError("UNSUPPORTED", where, "%s %s on a matrix signal or with a matrix parameter is not recorded "
                           "(recorded with matrices: %s)" % (t, n, ", ".join(MATRIX_BLOCKS)))
    if t in ("Gain", "Product"):
        mult = str(b.get("Multiplication", "Element-wise(K.*u)" if t == "Gain" else "Element-wise(.*)")).strip()
        if not mult.startswith("Element-wise"):
            raise G.GraphError("UNSUPPORTED", where + ".Multiplication", "%s %s with Multiplication %r on a matrix: "
                               "only element-wise multiplication is recorded" % (t, n, mult))
    if t in ("Sum", "Product") and len(ins) == 1 and _collapse_form(b, t):
        sh = ish[0]
        if sh is None:
            raise G.GraphError("UNSUPPORTED", where, "a collapsing %s with a matrix parameter is not recorded" % t)
        mode, d = _collapse_mode(b, n)
        if mode == "All dimensions":
            w, out = 1, None
        elif d == 1:
            w, out = sh[1], (1, sh[1])
        elif d == 2:
            w, out = sh[0], (sh[0], 1)
        else:
            raise G.GraphError("UNSUPPORTED", where + ".CollapseDim", "%s %s: CollapseDim %d on a %d x %d matrix is "
                               "not recorded (recorded: 1 and 2)" % (t, n, d, sh[0], sh[1]))
        mcollapse[n] = ("matrix", mode, d, sh)
        mkey[n] = {"block": t, "shape": list(sh), "params": {}, "collapse": [mode, d]}
        if out is not None:
            shape[key] = out
        return w
    ops = [("input %d" % (j + 1), sh, w_) for j, (w_, sh) in enumerate(zip(ws, ish)) if w_ > 1]
    pkind = {}
    for k_ in VECPARAM.get(t, ()):
        if k_ in b and t != "Constant":
            cnt = len(_ints_or_vals(b[k_], "%s.%s" % (where, k_), count_only=True))
            pkind[k_] = "matrix" if psh.get(k_) else ("scalar" if cnt == 1 else "vector")
            if cnt > 1:
                ops.append((k_, psh.get(k_), cnt))
    shapes = {sh for _, sh, _ in ops}
    if any(not lab.startswith("input") and sh is None for lab, sh, _ in ops) and any(ish):
        # a 1-D vector parameter on a matrix signal: not recorded (what Simulink does with it is not known)
        raise G.GraphError("UNSUPPORTED", where, "%s %s: a 1-D vector parameter on a matrix signal is not recorded; "
                           "write the parameter as a matrix of the signal's shape or as a scalar" % (t, n))
    if len(shapes) != 1 or None in shapes:
        desc = ", ".join("%s %s" % (lab, "%d x %d" % sh if sh else "a 1-D vector of %d" % cnt) for lab, sh, cnt in ops)
        if t == "UnitDelay" and "InitialCondition" in [lab for lab, _, _ in ops]:
            raise G.GraphError("BAD_PARAM", where + ".InitialCondition", "Unit Delay %s: the initial condition's "
                               "dimensions do not match the signal's (%s; Simulink: InvalidICForUnitDelaySampleBased)"
                               % (n, desc))
        raise G.GraphError("TYPE_MISMATCH", where, "%s %s: operands of different dimensions (%s; Simulink reports a "
                           "port dimension mismatch)" % (t, n, desc))
    if any(w_ == 1 for w_ in ws) and t != "Outport":
        raise G.GraphError("UNSUPPORTED", where, "%s %s: a scalar signal with a matrix operand is not recorded"
                           % (t, n))
    sh = shapes.pop()
    shape[key] = sh
    if t != "Outport":
        mkey[n] = {"block": t, "shape": list(sh), "params": pkind, "collapse": None}
    return sh[0] * sh[1]


def _mux_inputs(b, n):
    """A Mux's input count (a list of input widths is valid Simulink, not modelled: UNSUPPORTED)."""
    where = "design.blocks.%s.Inputs" % n
    v = _num(fxnum.canonical(b.get("Inputs", "2"), "vector"), where)
    if str(b.get("Inputs", "2")).strip().startswith("[") and not v.startswith("["):
        return 1   # recordings: '[2]' is ONE input of width 2 (a one-element width list), not two inputs
    if v.startswith("["):
        # a list of input widths ([2 1 1], [1 2 1], [-1 2]) -- one input per entry; each
        # width (-1: any) is checked against the connected signal (fxvector widths)
        return len(_ints(v, where))
    return int(_num(fxnum.canonical(v, "int"), where))


def _mux_widths(b, n):
    """The declared input widths of a Mux whose Inputs is a list (None for a count)."""
    v = _num(fxnum.canonical(b.get("Inputs", "2"), "vector"), "design.blocks.%s.Inputs" % n)
    if str(b.get("Inputs", "2")).strip().startswith("[") and not v.startswith("["):
        return _ints(v, "design.blocks.%s.Inputs" % n)   # a one-element width list
    return _ints(v, "design.blocks.%s.Inputs" % n) if v.startswith("[") else None


def _expand(text, where):
    """Rows of element texts of a canonical vector / matrix text, colon ranges of binary-exact literals expanded
    ('[1:3]' reached float() and crashed INTERNAL)."""
    rows = []
    for r in text.strip().strip("[]").split(";"):
        els = []
        for e in r.replace(",", " ").split():
            if ":" in e:
                v = fxnum._range_values(e)
                if v is None:
                    raise G.GraphError("UNSUPPORTED", where, "colon range %r with steps not exact in binary is not "
                                       "evaluated; write the values out" % e)
                els += v
            else:
                els.append(e)
        if els:
            rows.append(els)
    return rows


STRUCT_PARAMS = {"Selector": {"NumberOfDimensions", "IndexMode", "IndexOptions", "IndexOptionArray", "IndexParamArray",
                              "Indices", "OutputSizes", "OutputSizeArray", "InputPortWidth", "SampleTime",
                              "RuntimeRangeChecks"},
                 "Demux": {"Outputs", "DisplayOption", "BusSelectionMode"},
                 "Mux": {"Inputs", "DisplayOption", "UseBusObject", "BusObject", "NonVirtualBus"}}


def check_ports(spec):
    """recordings: Outport numbers must run 1..N (Simulink BadOutportNum), and an Outport declaring more than one
    element on a design without vectors is a dimension mismatch (PortDimsMismatch); both were answered."""
    outs = [(n, b) for n, b in spec["blocks"].items() if isinstance(b, dict) and b.get("type") == "Outport"]
    nums = []
    for n, b in outs:
        if "Port" in b:
            try:
                v = float(str(b["Port"]).strip())
            except ValueError:
                raise G.GraphError("BAD_PARAM", "design.blocks.%s.Port" % n, "Outport Port must be a number")
            if v != int(v) or v < 1:
                raise G.GraphError("BAD_PARAM", "design.blocks.%s.Port" % n, "Outport Port must be a positive integer")
            nums.append(int(v))
    if nums and sorted(nums) != list(range(1, len(nums) + 1)) or (nums and len(nums) != len(outs)):
        raise G.GraphError("BAD_PARAM", "design.blocks.%s.Port" % outs[0][0], "Outport numbers %s: Simulink needs "
                           "them numbered 1..%d without gaps (Simulink:Engine:BadOutportNum)" % (sorted(nums), len(outs)))
    other = set().union(*G.LIB_DIALOG_PARAMS.values()) | {k for d in G.DEFAULTS.values() for k in d}
    for n, b in spec["blocks"].items():
        own = STRUCT_PARAMS.get(b.get("type")) if isinstance(b, dict) else None
        if own is None:
            continue
        for k in b:
            if k != "type" and not k.startswith("_") and k not in own and k in other and \
                    k not in ("Description", "Tag", "Priority"):
                # Selector OutDataTypeStr / Demux RndMeth were answered (Simulink ParamUnknown)
                raise G.GraphError("BAD_PARAM", "design.blocks.%s.%s" % (n, k), "%s has no parameter %s (Simulink "
                                   "rejects it)" % (b["type"], k))
    if not has_vectors(spec):
        for n, b in outs:
            if str(b.get("PortDimensions", "-1")).strip() not in ("-1", "") and _width(b, n) != 1:
                raise G.GraphError("TYPE_MISMATCH", "design.blocks.%s.PortDimensions" % n, "Outport %s: "
                                   "PortDimensions %s but the signal is a scalar (Simulink reports a port dimension "
                                   "mismatch)" % (n, b["PortDimensions"]))


def has_vectors(spec):
    """True when the design needs this pass: vector signals, structural blocks, or a Delay whose initial
    condition gives one value per step (expanded into a chain of Unit Delays). (Main recordings also counts a matrix
    parameter on a scalar-only design; recorded does not -- that design stays refused as before, "a list where one value
    is used", which is what main answers too: a scalar signal with a matrix operand is not recorded.)"""
    if any(b.get("type") in ("Concatenate", "ForEach", "BusAssignment") + TAPPED for b in spec["blocks"].values()):
        return True
    for b in spec["blocks"].values():
        if b.get("type") == "MATLABFunction":
            # a MATLAB Function whose output is a vector built inside (y = fi(zeros(1, 4), ...))
            import fxmlfn
            import fxmlsem
            try:
                fn = fxmlfn.parse(b.get("Script", ""))
                nin = len(fn["functions"][0]["inputs"])
                if any(w > 1 for w in fxmlsem.widths(b.get("Script", ""), [1] * nin)):
                    return True
            except Exception:   # noqa: BLE001 -- the block's own compile reports a bad script
                pass
    return any(b.get("type") in STRUCT or _dims2(b, n)[0] > 1
               for n, b in spec["blocks"].items() if b.get("type") in STRUCT + ("Inport",)) or \
        any(b.get("type") == "Delay" and str(b.get("InitialCondition", "0")).strip().startswith("[")
            for b in spec["blocks"].values()) or \
        any(b.get("type") == "ShiftArithmetic" and _vec_shift(b) for b in spec["blocks"].values()) or \
        any(b.get("type") == "Constant" and _vec_const(b) for b in spec["blocks"].values()) or \
        any(b.get("type") == "Gain" and _vec_gain(b) for b in spec["blocks"].values())


def _vec_const(b):
    """a Constant whose Value is a vector or matrix literal is a vector source, element k
    the k-th value in column-major order (any VectorParams1D: on gives 1-D, off keeps 1 x n / n x 1)."""
    v = str(b.get("Value", "")).strip()
    if not v.startswith("["):
        return False
    try:
        return len(_ints_or_vals(v)) > 1
    except G.GraphError:
        return True   # let the pass report what it cannot read


def _vec_gain(b):
    """A Gain whose K holds several values (a row or column): on a scalar input Simulink expands the input and the
    output is a vector of numel(K) (with
    Multiplication Matrix(K*u) an m x 1 K, recordings 2x1 and recordings), so the
    design needs this pass."""
    k_ = str(b.get("Gain", "1")).strip()
    rows_ = [r_ for r_ in k_.strip("[]").replace("\n", ";").split(";") if r_.strip()]
    if len(rows_) > 1 and any(len(r_.replace(",", " ").split()) > 1 for r_ in rows_):
        # a K with rows and columns (a 2 x 2 matrix on a scalar input) --
        # a matrix output; not taken by this pass
        return False
    try:
        return len(_ints_or_vals(b.get("Gain", "1"), "design", count_only=True)) > 1
    except G.GraphError:
        return False


def _vec_shift(b):
    """recordings (and recordings): a BitShiftNumber with several values makes the
    output a vector of that width (a scalar input is expanded), so the design needs this pass."""
    t = str(b.get("BitShiftNumber", "0")).strip().rstrip(";").strip()
    if not (t.startswith("[") and t.endswith("]")):
        return False
    try:
        return len(_ints_or_vals(t, "design")) > 1
    except Exception:   # noqa: BLE001 -- the engine reports a malformed value itself
        return False


def refuse_vectors(spec, tool):
    """a tool without this pass (fxcodegen, fxcert) must refuse a design that needs it -- built as a
    scalar design, a vector Inport carried one element and the rest of the design was typed and generated wrongly
    (status ok, no warning)."""
    if isinstance(spec, dict) and isinstance(spec.get("blocks"), dict) and has_vectors(spec):
        raise G.GraphError("UNSUPPORTED", "design", "%s does not handle vector signals, Mux / Demux / Selector or "
                           "per-step Delay initial conditions yet (fxgraph and fxopt do)" % tool)


def _matrix(text, where="design"):
    return [[float(v) for v in r] for r in _expand(_checked(text, where), where)]


# a Delay that leaves DelayLength out has the library's 2 (blockspec
# JSON_LIBRARY_DEFAULTS, the value the run uses); the vector pass read the initial condition as for length 1
_DL_LIB = __import__("blockspec").JSON_LIBRARY_DEFAULTS["Delay"]["DelayLength"]


def _delay_ic(n, b, w):
    """Per-element initial values of a Delay of width w: a list of w scalars, or of w per-step lists (length N).
    Simulink: a non-scalar IC must have the delay length as its last dimension --
    a w x N matrix gives row k to element k; a length-N vector is for a scalar signal only."""
    N = int(_num(fxnum.canonical(b.get("DelayLength", _DL_LIB), "int"), "design.blocks.%s.DelayLength" % n))
    m = _matrix(b.get("InitialCondition", "0"), "design.blocks.%s.InitialCondition" % n)
    flat = [v for r in m for v in r]
    if len(flat) == 1:
        return [flat[0]] * w
    if str(b.get("DelayLengthSource", "Dialog")).strip() == "Input port":
        # with the length from a port the dialog DelayLength is not the delay -- [1 2 3 4] with upper
        # limit 4 runs in Simulink; checking it against DelayLength gave an untrue "Simulink rejects"
        raise G.GraphError("UNSUPPORTED", "design.blocks.%s.InitialCondition" % n, "a non-scalar initial condition "
                           "with the delay length from a port is not supported here; give one scalar value")
    if w == 1 and (N == 1 or (len(m) == N and all(len(r) == 1 for r in m))):
        # recordings (c): on a scalar signal Simulink accepts a Delay(1) with several initial values and a Delay(N)
        # with a column [1;2;3]; what it does with them is not recorded -- not answered, and not called a Simulink
        # error as before (the recorded refusals are on vector signals)
        raise G.GraphError("UNSUPPORTED", "design.blocks.%s.InitialCondition" % n, "initial condition %r on a scalar "
                           "signal with delay length %d: Simulink accepts this shape but its meaning is not recorded; "
                           "write a scalar or a row of %d values" % (b.get("InitialCondition"), N, N))
    # (Simulink runs a row of one value per element on a Delay of length 1 -- 8 of 8 recorded; the
    # recorded case that looked like a refusal had DelayLength left out, i.e. the library's 2)
    if N == 1:   # one value per element (a w x 1 column, or a matrix read column by column)
        # A one-step delay reads one initial value per signal element.
        # Matrix initial conditions are stored in column-major order.
        if len(m) > 1 and all(len(r) == len(m[0]) for r in m):
            flat = [m[i][j] for j in range(len(m[0])) for i in range(len(m))]
        if len(flat) != w:
            # Simulink refuses it as a parameter error for Delay length 1 and
            # Unit Delay alike (InvalidICForUnitDelaySampleBasedNonScalar1DInput): BAD_PARAM, as the other IC shapes
            raise G.GraphError("BAD_PARAM", "design.blocks.%s.InitialCondition" % n, "%d initial values for width %d "
                               "(a unit delay needs a scalar or one value per element)" % (len(flat), w))
        return flat
    if len(m) == 1 and len(m[0]) == N:
        if w > 1:
            # Simulink refuses a length-N vector on
            # a vector signal (SimulinkBlocks:Delay:InvalidICForSampleBasedNonScalarInput); the engine gave every
            # element the same per-step values. Only the width x N matrix is recorded (slx_struct_vecparams)
            raise G.GraphError("BAD_PARAM", "design.blocks.%s.InitialCondition" % n, "a length-%d initial condition "
                               "vector on a signal of width %d: Simulink needs a scalar or a %d x %d matrix (one row "
                               "per element; SimulinkBlocks:Delay:InvalidICForSampleBasedNonScalarInput)"
                               % (N, w, w, N))
        return [list(m[0]) for _ in range(w)]
    if len(m) == w and all(len(r) == N for r in m):
        return [list(r) for r in m]
    if w == 1 and len(m) > 1 and all(len(r) == N for r in m):
        # on a scalar signal Simulink runs it, widening the output to one element per row -- not
        # answered (it was refused as a Simulink error)
        raise G.GraphError("UNSUPPORTED", "design.blocks.%s.InitialCondition" % n, "a %d x %d initial condition on a "
                           "scalar Delay of length %d makes Simulink's output %d wide: not supported"
                           % (len(m), N, N, len(m)))
    # a parameter inconsistency (it was reported as a port type error): BAD_PARAM
    raise G.GraphError("BAD_PARAM", "design.blocks.%s.InitialCondition" % n,
                       "the last dimension of a non-scalar initial condition must equal the delay length %d "
                       "(SimulinkBlocks:Delay:ICDimensionAndDelayLengthMismatch)" % N)


# Selector library defaults (the library block selects [1 3] of a width-3 input; the
# independent model gave y = [10 12] where the engine answered the scalar 10). Reported as assumed_defaults.
SEL_DEFAULTS = {"NumberOfDimensions": "1", "IndexMode": "One-based", "IndexOptions": "Index vector (dialog)",
                "Indices": "[1 3]", "OutputSizes": "1", "InputPortWidth": "3"}


def structural_assumed(spec):
    """{block: {parameter: library value}} for the Selector parameters a design leaves out."""
    out = {}
    for n, b in (spec.get("blocks") or {}).items():
        if isinstance(b, dict) and b.get("type") == "Selector":
            miss = {k: v for k, v in SEL_DEFAULTS.items() if k not in b}
            if str(b.get("IndexOptions", SEL_DEFAULTS["IndexOptions"])) != "Starting index (dialog)":
                miss.pop("OutputSizes", None)
            if miss:
                out[n] = miss
    return out


def _demux_parts(n, b):
    """('count', k) for Outputs k, or ('widths', [w1, ...]) for a bracketed list -- also a one-element list: '[4]'
    is one output of width 4 (it was split into four of width 1, silently wrong). Simulink rejects
    a 0 / negative element (MuxOrDemuxBadVectorParam), Outputs below 1 (MuxOrDemuxBadNumPortsParam) and bus
    selection mode on a vector (DemuxSplittingInBusMode); -1 (sizes taken from the signal) is not supported here."""
    where = "design.blocks.%s.Outputs" % n
    if str(b.get("BusSelectionMode", "off")).strip() == "on":
        if str(b.get("Outputs", "2")).strip() == "1":
            # with one output Simulink runs it (nothing is split) -- not supported, not a rejection
            raise G.GraphError("UNSUPPORTED", "design.blocks.%s.BusSelectionMode" % n, "Demux in bus selection mode is "
                               "not supported; set BusSelectionMode off")
        raise G.GraphError("PORT_TYPE_INVALID", "design.blocks.%s.BusSelectionMode" % n, "Demux in bus selection mode "
                           "cannot split a vector signal (Simulink DemuxSplittingInBusMode); set BusSelectionMode off")
    text = str(b.get("Outputs", "2")).strip()
    vals = _ints(text, where)
    if text.startswith("["):
        if any(v == -1 for v in vals):
            raise G.GraphError("UNSUPPORTED", where, "Demux Outputs %s: -1 (a width taken from the signal) is not "
                               "supported; give every width" % text)
        if not vals or any(v < 1 for v in vals):
            raise G.GraphError("PORT_TYPE_INVALID", where, "Demux Outputs %s: every width must be positive (Simulink "
                               "MuxOrDemuxBadVectorParam)" % text)
        return ("widths", vals)
    if len(vals) != 1 or vals[0] < 1:
        raise G.GraphError("PORT_TYPE_INVALID", where, "Demux Outputs %s: the number of outputs must be at least 1 "
                           "(Simulink MuxOrDemuxBadNumPortsParam)" % text)
    return ("count", vals[0])


def _sel_port(b):
    """recordings: a one-dimensional Selector whose indices come from its second input port."""
    return str(b.get("IndexOptions", "")) == "Index vector (port)" and str(b.get("NumberOfDimensions", "1")) == "1"


def _sel_indices(n, b, in_width=None):
    """One-based element indices a one-dimensional Selector outputs: 'Index vector (dialog)' lists them;
    'Starting index (dialog)' takes OutputSizes consecutive elements from the starting index. Parameters left out take
    the library values (SEL_DEFAULTS)."""
    if "IndexOptionArray" in b:
        # IndexOptionArray was ignored ("{'Select all'}" and "{'bogus'}" answered from Indices);
        # its forms are not recorded -- refused, without claiming which spellings Simulink takes
        raise G.GraphError("UNSUPPORTED", "design.blocks.%s.IndexOptionArray" % n, "Selector: IndexOptionArray is not "
                           "read by this engine (which of its spellings Simulink accepts is not recorded); give the "
                           "option in IndexOptions and the indices in Indices")
    if "IndexParamArray" in b:
        # the string form "{'2'}" was silently ignored (and Simulink rejects it); Indices is the parameter
        raise G.GraphError("BAD_PARAM", "design.blocks.%s.IndexParamArray" % n, "Selector: IndexParamArray is not "
                           "supported; give the indices in Indices (e.g. \"[2]\")")
    if "OutputSizeArray" in b:
        # recordings (native: '["1"]' rejected at set_param, InvSelBlkMxArrayParam; the engine ignored it): the
        # array form of OutputSizes is not supported, and Simulink accepts only a cell array of character vectors
        txt = str(b["OutputSizeArray"]).strip()
        raise G.GraphError("UNSUPPORTED" if txt.startswith("{") else "BAD_PARAM",
                           "design.blocks.%s.OutputSizeArray" % n, "Selector OutputSizeArray %r: %s; give the size in "
                           "OutputSizes" % (txt[:40], "not supported" if txt.startswith("{") else
                                            "Simulink needs a cell array of character vectors (InvSelBlkMxArrayParam)"))
    given = set(b)   # the parameters the design wrote (the Starting-index check needs them)
    b = dict(SEL_DEFAULTS, **b)
    if str(b.get("NumberOfDimensions", "1")) != "1":
        raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "Selector: one dimension only")
    mode = str(b.get("IndexMode", "One-based"))
    if mode not in ("One-based", "Zero-based"):
        raise G.GraphError("BAD_PARAM", "design.blocks.%s.IndexMode" % n, "One-based or Zero-based")
    # a zero-based index k is element k + 1
    base = 1 if mode == "Zero-based" else 0
    if "IndexParamArray" in b:
        # recordings (helper recordings): the cell-array index form was ignored silently (Simulink rejects
        # its string form) -- write Indices
        raise G.GraphError("UNSUPPORTED", "design.blocks.%s.IndexParamArray" % n, "Selector IndexParamArray is not "
                           "read; write the indices as Indices")
    opt = str(b.get("IndexOptions", "Index vector (dialog)"))
    if opt == "Index vector (dialog)":
        # recordings (silent wrong) / recordings: a parameter a design leaves out is the library block's value --
        # a library Selector placed new has Indices [1 3] (output [x1 x3]; the model and recordings alike;
        # main does the same). A saved .slx without Indices means the built-in 1: the .slx reader writes that in
        return [i + base for i in _ints(b.get("Indices", "[1 3]"), "design.blocks.%s.Indices" % n)]
    if opt == "Starting index (dialog)":
        if "Indices" not in given:   # (was tested after the defaults were merged in: never reached)
            raise G.GraphError("TYPE_MISMATCH", "design.blocks.%s.Indices" % n, "Selector 'Starting index (dialog)' with "
                               "Indices left out: the library value [1 3] is not a scalar starting index (Simulink: "
                               "Simulink:blocks:SelInvNumElements) -- write the starting index")
        st, size = _ints(b.get("Indices", "1"), "design.blocks.%s.Indices" % n), \
            _ints(b.get("OutputSizes", "1"), "design.blocks.%s.OutputSizes" % n)
        if len(st) != 1 or len(size) != 1 or size[0] < 1:
            raise G.GraphError("BAD_PARAM", "design.blocks." + n, "Selector starting index: one start and one output size")
        return [st[0] + base + j for j in range(size[0])]
    if opt == "Select all" and in_width is not None:
        return list(range(1, in_width + 1))   # every element, in order
    raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "Selector: 'Index vector (dialog)' or 'Starting index "
                       "(dialog)' only (index ports are not supported)")


# blocks that keep their input's dimensions (element-wise, delays); any other block driven by a 2-D signal gives an
# orientation the engine does not track ("?")
_KEEPS_DIMS = ("Gain", "Sum", "MinMax", "Abs", "UnaryMinus", "Signum", "DataTypeConversion", "Saturation", "Switch",
               "Product", "Bias", "UnitDelay", "Delay", "Memory", "UnitDelayEnabledSynchronous",
               "UnitDelayResettableSynchronous", "UnitDelayEnabledResettableSynchronous", "ShiftArithmetic",
               "RelationalOperator", "LogicalOperator", "BitwiseOperator", "Math", "SignalSpecification")


def _orientation_of_dims(text):
    v = str(text or "").strip().replace(",", " ")
    if not (v.startswith("[") and v.endswith("]")):
        return None
    parts = v.strip("[]").split()
    if len(parts) != 2:
        return None
    if parts[1] == "1" and parts[0] != "1":
        return "column"
    if parts[0] == "1" and parts[1] != "1":
        return "row"
    return None


def _ic_orientation(text):
    """'row' for a literal row [a b ...], 'column' for [a; b; ...], 'matrix' for more, None for a scalar / other."""
    v = str(text or "").strip()
    if not (v.startswith("[") and v.endswith("]")):
        return None
    rows = [r.replace(",", " ").split() for r in v[1:-1].split(";")]
    rows = [r for r in rows if r]
    if len(rows) == 1:
        return "row" if len(rows[0]) > 1 else None
    if all(len(r) == 1 for r in rows):
        return "column"
    return "matrix"


def _lit_shape(text):
    """The shape a parameter literal has: ("s",) scalar, ("row", n), ("col", n), ("mat", r, c); None if not a literal."""
    v = str(text if text is not None else "").strip()
    if not (v.startswith("[") and v.endswith("]")):
        return ("s",)
    rows = [r.replace(",", " ").split() for r in v[1:-1].split(";")]
    rows = [r for r in rows if r]
    if not rows:
        return None
    if len(rows) == 1:
        return ("row", len(rows[0])) if len(rows[0]) > 1 else ("s",)
    if all(len(r) == 1 for r in rows):
        return ("col", len(rows))
    if any(len(r) != len(rows[0]) for r in rows):
        return None   # ragged rows: the value's own check reports it (BAD_PARAM)
    return ("mat", len(rows), len(rows[0]))


def _dims_shape(text):
    """Shape of dimensions written [r c] (PortDimensions, Reshape OutputDimensions); None for one number / other."""
    v = str(text or "").strip().replace(",", " ")
    if not (v.startswith("[") and v.endswith("]")):
        return None
    parts = v.strip("[]").split()
    if len(parts) != 2:
        return None
    try:
        r, c = int(parts[0]), int(parts[1])
    except ValueError:
        return None
    if r == 1 and c == 1:
        return ("s",)
    if c == 1:
        return ("col", r)
    if r == 1:
        return ("row", c)
    return ("mat", r, c)


# parameters whose literal carries a shape that must agree with the block's input
_SHAPED_PARAMS = {"Gain": ("Gain",), "Saturation": ("UpperLimit", "LowerLimit"), "BitwiseOperator": ("BitMask",),
                  "ShiftArithmetic": ("BitShiftNumber",), "UnitDelay": ("InitialCondition",)}
# (a Delay's initial condition may be a w x N matrix of per-step values -- _delay_ic checks that form)
_DELAYS_IC = ("UnitDelay", "Memory", "UnitDelayEnabledSynchronous", "UnitDelayResettableSynchronous",
              "UnitDelayEnabledResettableSynchronous")


def _check_delay_ic_orientation(spec):
    """the 2-D shape of every signal (scalar, 1-D vector, row 1 x n, column n x 1, matrix r x c),
    from a Reshape, an Inport with dimensions [r c] or a Constant's literal (VectorParams1D on: rows and columns are
    1-D, matrices stay), carried through blocks that keep their input's dimensions. Simulink refuses (PortDimsMismatch;
    recordings 22 fresh designs): a matrix meeting a vector, a column meeting a row, a matrix parameter (gain, limits, mask,
    shift, initial condition) on another shape, a row parameter on a column or matrix, a matrix into a reduction /
    Demux / Selector / Mux. Shapes the engine does not follow ("?") never receive a 2-D parameter. Only 1-D vectors,
    rows, columns of one class and same-shaped matrices are answered (recordings show)."""
    blocks = spec.get("blocks", {})
    shape = {}
    for n, b in blocks.items():
        if not isinstance(b, dict):
            continue
        t = b.get("type")
        if t == "Reshape":
            od = str(b.get("OutputDimensionality", "")).strip()
            shape[n] = {"Column vector (2-D)": ("col", 0), "Row vector (2-D)": ("row", 0),
                        "1-D array": ("1d",)}.get(od) or (_dims_shape(b.get("OutputDimensions")) or ("?",)
                                                          if od == "Customize" else ("?",))
        elif t == "Inport":
            d = _dims_shape(b.get("PortDimensions"))
            if d is not None:
                shape[n] = d
            elif str(b.get("PortDimensions", "1")).strip() not in ("1", "-1", ""):
                shape[n] = ("1d",)
        elif t == "Constant":
            ls = _lit_shape(b.get("Value"))
            if ls is not None and ls[0] != "s":
                on = str(b.get("VectorParams1D", "on")).strip() == "on"
                shape[n] = ("1d",) if (on and ls[0] in ("row", "col")) else ls
    srcs, data_in = {}, {}
    for ln in spec.get("lines", []):
        srcs.setdefault(ln.get("dst"), []).append((int(ln.get("dport", 1)), ln.get("src")))
        if int(ln.get("dport", 1)) == 1:
            data_in[ln.get("dst")] = ln.get("src")
    for n, b in blocks.items():
        if isinstance(b, dict) and b.get("type") == "Concatenate" and \
                str(b.get("Mode", "Vector")).strip().startswith("Multidimensional"):
            # a multidimensional concatenation of 1-D vectors is 2-D in Simulink (a row IC after it
            # is refused); its shape is not followed -- 2-D parameters after it are refused as not recorded
            shape[n] = ("?",)
    mux1 = set()   # a one-input Mux keeps a row / column / matrix (Simulink then refuses a
    for n, b in blocks.items():   # mismatched IC or gain after it); it was taken as a 1-D vector
        if isinstance(b, dict) and b.get("type") == "Mux":
            if len(srcs.get(n, [])) == 1 and str(b.get("Inputs", "2")).strip() == "1":
                mux1.add(n)
            else:
                shape[n] = ("1d",)
    seeded = {}   # a matrix parameter on a scalar input makes the output that matrix (Simulink:
    for n, b in blocks.items():   # a 2 x 2 Gain / limit / IC / shift on a scalar is a 2 x 2 signal)
        if not isinstance(b, dict) or n in shape:
            continue
        t = b.get("type")
        ks = _SHAPED_PARAMS.get(t, ()) + (("InitialCondition",) if t in _DELAYS_IC + ("Delay",) and t not in
                                          _SHAPED_PARAMS else ())
        if t == "Delay" and str(b.get("DelayLength", _DL_LIB)).strip() != "1":
            ks = ()
        if t == "BitwiseOperator" and str(b.get("UseBitMask", "on")).strip() != "on":
            ks = ()
        if t == "Gain" and not str(b.get("Multiplication", "Element-wise(K.*u)")).startswith("Element-wise"):
            ks = ()
        for k in ks:
            ps = _lit_shape(b.get(k)) if k in b else None
            if ps is not None and ps[0] == "mat":
                seeded[n] = ps
                break

    def cls(x):
        return x if x[0] == "mat" else (x[0],)

    def where_(n):
        return "design.blocks." + n

    changed = True
    while changed:
        changed = False
        for n, b in blocks.items():
            if isinstance(b, dict) and b.get("type") == "Concatenate" and shape.get(n) == ("?",):
                continue
            if isinstance(b, dict) and b.get("type") == "Mux" and n not in mux1:
                # a several-input Mux of a 2-D signal keeps a 2-D shape in Simulink: not followed
                if shape.get(n) != ("?",) and any(shape.get(x, ("s",))[0] not in ("s", "1d")
                                                  for _, x in srcs.get(n, [])):
                    shape[n] = ("?",)
                    changed = True
                continue
            if not isinstance(b, dict) or b.get("type") in ("Reshape", "Inport", "Constant") or \
                    (b.get("type") == "Mux" and n not in mux1):
                continue
            ins = [(p_, shape[x]) for p_, x in srcs.get(n, []) if x in shape]
            ins = [(p_, x) for p_, x in ins if x[0] != "s"]
            t = b.get("type")
            if t == "Product" and str(b.get("Multiplication", "Element-wise(.*)")).startswith("Matrix") and ins:
                # a matrix product of 2-D signals is not refused by Simulink as mismatched
                # dimensions; not recorded here
                raise G.GraphError("UNSUPPORTED", where_(n) + ".Multiplication", "Product Matrix(*) of 2-D signals: "
                                   "not supported")
            if n in seeded and not [x for p_, x in ins if p_ == 1]:
                ins = [(1, seeded[n])] + [(p_, x) for p_, x in ins if p_ != 1]
            if not ins:
                continue
            if t == "Switch":   # the control port's shape does not reach the output
                ins = [(p_, x) for p_, x in ins if p_ != 2] or ins
            kinds = {cls(x) for _, x in ins}
            if t not in _KEEPS_DIMS and t not in ("ExtractBits", "CompareToConstant", "Bias", "Rounding") and \
                    n not in mux1:
                if kinds == {("1d",)}:
                    continue   # 1-D vectors only: no 2-D shape to follow (Goto, Selector, ... keep 1-D)
                o = ("?",)
            elif len(kinds) == 1:
                o = ins[0][1] if ins[0][1][0] != "row" and ins[0][1][0] != "col" else (ins[0][1][0], 0)
            elif any(k[0] == "?" for k in kinds):
                o = ("?",)
            else:
                ks = sorted(kinds)
                if any(k[0] == "mat" for k in ks) or {k[0] for k in ks} == {"row", "col"}:
                    raise G.GraphError("TYPE_MISMATCH", where_(n), "%s combines signals of the shapes %s: Simulink "
                                       "refuses mismatched dimensions (PortDimsMismatch)"
                                       % (t, ", ".join(_shape_text(k) for k in ks)))
                if t in ("Sum", "Product") and {k[0] for k in ks} in ({"1d", "row"}, {"1d", "col"}):
                    # Simulink runs a Sum of a 1-D vector and a row, a Product of a 1-D vector
                    # and a column, element by element (equal); the output's 2-D shape is not followed
                    o = ("?",)
                    if shape.get(n) != o:
                        shape[n] = o
                        changed = True
                    continue
                raise G.GraphError("UNSUPPORTED", where_(n), "%s combines a 1-D vector with a 2-D %s: not recorded "
                                   "(give the inputs one shape)" % (t, "/".join(k[0] for k in ks if k[0] != "1d")))
            if shape.get(n) != o:
                shape[n] = o
                changed = True
    for n, b in blocks.items():
        # recordings (silent wrong) / recordings: a row, column or matrix into a several-input Mux or a
        # Concatenate -- Simulink keeps or refuses the 2-D shape (a1 b1 a2 b2 for two rows along dimension 1; a row
        # with a column is refused); the engine flattened it. Not recorded: refused
        if isinstance(b, dict) and (b.get("type") == "Concatenate" or (b.get("type") == "Mux" and n not in mux1)):
            twod = sorted({shape[x][0] for _, x in srcs.get(n, []) if x in shape and shape[x][0] not in ("s", "1d")})
            multidim = str(b.get("Mode", "Vector")).strip().startswith("Multidimensional")
            # a Mux of 2-D signals runs in Simulink (recordings) but keeps a 2-D shape the
            # engine does not follow: its output is '?' (2-D parameters after it are refused, recordings); a row
            # with a column and a multidimensional concatenation of 2-D signals (a1 b1 a2 b2)
            # are refused
            # (a matrix into a vector Concatenate or a Mux is refused by Simulink; a row / column into
            # a vector Concatenate is not recorded -- only a row or column into a Mux is followed, as '?')
            if twod and b.get("type") == "Mux" and not ({"row", "col"} <= set(twod)) and "mat" not in twod:
                continue
            if twod:
                raise G.GraphError("UNSUPPORTED", where_(n), "%s of 2-D signals (%s): not supported -- give 1-D "
                                   "vectors (Reshape to a 1-D array first)" % (b.get("type"), ", ".join(
                                       {"row": "row", "col": "column", "mat": "matrix", "?": "2-D"}[k] for k in twod)))
    for n, b in blocks.items():
        if not isinstance(b, dict):
            continue
        t = b.get("type")
        x = shape.get(data_in.get(n))
        if x is None or x[0] == "s":
            # a scalar input: a delay's vector IC makes the vector (handled by the vector pass); other parameters
            # keep their own rules
            continue
        # a one-input Sum / MinMax / Bitwise reduction over a matrix runs in Simulink (D4a-g) -- it was
        # refused here with an untrue reason; it reduces over all elements like a vector
        # (a one-input Logical Operator over a matrix stays refused: Simulink refuses it, recordings -- true per recordings)
        if x[0] == "mat" and (t in ("Demux", "Selector") or (t == "Mux" and n not in mux1) or
                              (t == "LogicalOperator" and len(srcs.get(n, [])) == 1 and _one_input(b, t))):
            raise G.GraphError("TYPE_MISMATCH", where_(n), '%s of a %s matrix: Simulink refuses it (%s'
                               ')' % (t, _shape_text(x), "DemuxCannotSplitMatrices" if t == "Demux" else
                                         "PortDimsMismatch"))
        if t == "Delay" and x[0] in ("row", "col", "mat") and "InitialCondition" in b:
            ps_ = _lit_shape(b.get("InitialCondition"))
            if ps_ is not None and ps_[0] != "s" and str(b.get("DelayLength", _DL_LIB)).strip() != "1":
                # a non-scalar IC of a Delay(N > 1) on a 2-D signal (Simulink refused the
                # recorded ones: InvalidICFor...); its layout for 2-D signals is not recorded
                raise G.GraphError("UNSUPPORTED", where_(n) + ".InitialCondition", "a non-scalar initial condition "
                                   "of a Delay of length %s on a %s signal: not recorded (Simulink refused the "
                                   "probed shapes); use a scalar initial condition or a 1-D signal"
                                   % (str(b.get("DelayLength")).strip(), _shape_text(x)))
        delay1 = t == "Delay" and str(b.get("DelayLength", _DL_LIB)).strip() == "1"
        for k in _SHAPED_PARAMS.get(t, ()) + (("InitialCondition",) if (t in _DELAYS_IC or delay1) and
                                               t not in _SHAPED_PARAMS else ()):
            if k not in b:
                continue
            if t == "BitwiseOperator" and str(b.get("UseBitMask", "on")).strip() != "on":
                continue
            if t == "Gain" and not str(b.get("Multiplication", "Element-wise(K.*u)")).startswith("Element-wise"):
                continue   # a matrix product (K*u, u*K): its own rules (refused as not recorded), not element shapes
            ps = _lit_shape(b.get(k))
            if ps is None or ps[0] == "s":
                continue
            ok = None
            if ps[0] == "mat":
                ok = x[0] == "mat" and x[1:] == ps[1:]
            elif ps[0] == "row":
                ok = x[0] in ("1d", "row") or (x[0] == "?" and None)
                if x[0] in ("col", "mat"):
                    ok = False
            elif ps[0] == "col":
                # 1-D + column: recorded for a delay IC and an element-wise Gain (recordings
                # recordings*_Kcol_elementwiseku, equal)
                col_1d = _DELAYS_IC + ("Gain",)
                ok = x[0] == "col" or (x[0] == "1d" and t in col_1d)
                if x[0] in ("row", "mat"):
                    ok = False
                elif x[0] == "1d" and t not in col_1d:
                    ok = None
            if x[0] == "?":
                ok = None
            if ok is False:
                raise G.GraphError("TYPE_MISMATCH", where_(n) + "." + k, "%s %s of the shape %s on a %s input: "
                                   "Simulink refuses mismatched dimensions (PortDimsMismatch)%s"
                                   % (t, k, _shape_text(ps), _shape_text(x),
                                      "; write it as a %s" % ("column ([a; b])" if x[0] == "col" else
                                                              "row ([a b])" if x[0] in ("row", "1d") else
                                                              "matrix of the signal's size") if x[0] != "?" else ""))
            if ok is None:
                raise G.GraphError("UNSUPPORTED", where_(n) + "." + k, "%s %s of the shape %s on a %s input: not "
                                   "recorded -- not computed" % (t, k, _shape_text(ps), _shape_text(x)))


def _shape_text(x):
    return {"s": "scalar", "1d": "1-D vector", "row": "row (1 x n)", "col": "column (n x 1)",
            "?": "2-D (not followed)"}.get(x[0]) if x[0] != "mat" else "%d x %d" % (x[1], x[2])


def named_port_check(spec):
    """recordings: a line on a named port (the integrator's sport 'state') reached the recorded passes, which take ports as
    numbers (INTERNAL ValueError); the state port is refused (main, recordings) -- said here first."""
    for ln in (spec.get("lines") or []) if isinstance(spec, dict) else []:
        if not isinstance(ln, dict):
            continue
        for k in ("sport", "dport"):
            v = ln.get(k)
            if v is not None and not str(v).strip().lstrip("-").isdigit():
                if str(v).strip() == "state":
                    raise G.GraphError("UNSUPPORTED", "design.lines", 'the integrator state port is not supported'
                                       "; take the output port instead")
                raise G.GraphError("BAD_CONNECTION", "design.lines", "port %s %r: a port number is required" % (k, v))


_ONE_OUTPUT = {"Inport", "Constant", "Gain", "Sum", "Product", "DataTypeConversion", "UnitDelay", "Delay", "Saturation",
               "Abs", "UnaryMinus", "Mux", "Concatenate", "Selector", "Switch", "MinMax", "Reshape",
               "RelationalOperator", "LogicalOperator", "MultiportSwitch", "BitwiseOperator", "ShiftArithmetic",
               "CompareToConstant", "SignalSpecification", "DataTypeDuplicate"}


def scalarise(spec):
    """Returns (scalar spec, info{"width", "element", "ties", "outports"})."""
    named_port_check(spec)
    import parameter_bounds
    parameter_bounds.check(spec)   # recorded ranges first
    spec = {k: v for k, v in spec.items() if k != parameter_bounds.VEC_KEY}   # only this pass writes it
    # the user blocks this pass replaces (their compiled widths are checked on their elements)
    # a 2-D column / row vector reaching a delay
    # whose initial condition is the other orientation ([2 6] on a column, [2; 6] on a row) stops Simulink
    # (PortDimsMismatch52 / 21), also through a Gain / Sum / MinMax / another delay; the engine counts elements only.
    _check_delay_ic_orientation(spec)
    vec_user = {n: b.get("type") for n, b in spec["blocks"].items() if isinstance(b, dict) and INTERNAL not in n
                and b.get("type") in VEC_WIDTH_TYPES}
    vec_params = {n: {k: str(v) for k, v in spec["blocks"][n].items() if k != "type"} for n in vec_user}
    # the rows recorded with a scalar boolean only (parameter_bounds.BOOL_SHAPES)
    bool_rows = [n for n, b in spec["blocks"].items() if isinstance(b, dict) and INTERNAL not in n
                 and b.get("type") in parameter_bounds.BOOL_SHAPES]
    # For Each subsystems: inside a partition every signal is one element, so a block there acts on
    # each element on its own -- a one-input Sum / MinMax is not a reduction across the vector, and only element-wise
    # blocks are accepted (anything that would see the whole vector is refused)
    fe_pre = tuple(sorted({n.rsplit("/", 1)[0] + "/" for n, b in spec["blocks"].items()
                           if isinstance(b, dict) and b.get("type") == "ForEach" and "/" in n}))
    for n_, b_ in spec["blocks"].items():
        if fe_pre and n_.startswith(fe_pre) and isinstance(b_, dict) and b_.get("type") not in ELEMENTWISE + (
                "ForEach",):
            raise G.GraphError("UNSUPPORTED", "design.blocks." + n_, "%s inside a For Each subsystem: only "
                               "element-wise blocks are supported there (each partition is one element)"
                               % b_.get("type"))
        # For Each was recorded with these blocks inside only
        # (Gain, Sum '++', Unit Delay; recordings: Saturation, Constant)
        if fe_pre and n_.startswith(fe_pre) and isinstance(b_, dict):
            t_ = b_.get("type")
            if t_ not in FOREACH_RECORDED:
                raise G.GraphError("UNSUPPORTED", "design.blocks." + n_, "%s inside a For Each subsystem is not "
                                   "recorded (recorded: %s)"
                                   % (t_, ", ".join(FOREACH_RECORDED)))
            if t_ == "Sum" and str(b_.get("Inputs", "++")).strip() not in ("++", "2"):
                raise G.GraphError("UNSUPPORTED", "design.blocks.%s.Inputs" % n_, "a Sum %s inside a For Each "
                                   "subsystem is not recorded (only '++')" % b_.get("Inputs"))
    spec = _concat_to_mux(_drop_foreach(spec))
    blocks = spec["blocks"]
    # declared port types: only those sitting on an Inport are carried over (onto each of its elements)
    port_types = [e for e in spec.get("port_types") or [] if isinstance(e, dict)]
    if any(not isinstance(blocks.get(e.get("block")), dict) or blocks[e["block"]].get("type") != "Inport"
           for e in port_types):
        raise G.GraphError("UNSUPPORTED", "design.port_types", "typed subsystem ports on a computed signal in a "
                           "design with vector signals")
    srcs = {}
    for ln in spec["lines"]:
        if str(ln.get("sport", 1)).strip().lower() == "state":
            # a state-port line in a design with vector signals reached int() -- INTERNAL
            if not G.STATEPORT_FEEDBACK:
                raise G.state_port_refusal(ln.get("src"))
            raise G.GraphError("UNSUPPORTED", "design.blocks.%s.ShowStatePort" % ln.get("src"), "the integrator "
                               "state port in a design with vector signals is not supported")
        try:
            sp_, dp_ = int(str(ln.get("sport", 1)).strip()), int(str(ln.get("dport", 1)).strip())
        except ValueError:
            # a previous version: a text port in a vector design gave INTERNAL
            raise G.GraphError("BAD_CONNECTION", "design.lines", "line %s -> %s: sport / dport must be integers"
                               % (ln.get("src"), ln.get("dst")))
        if sp_ != 1 and (blocks.get(ln["src"]) or {}).get("type") in _ONE_OUTPUT:
            # a previous version: a line from port 2 of a single-output block in a vector design was answered (recorded: only
            # for the basic one-output blocks -- the vector pass has multi-output the extended blocks besides the Demux)
            raise G.GraphError("BAD_CONNECTION", "design.blocks." + str(ln["src"]), "%s has no output port %d"
                               % (ln["src"], sp_))
        srcs.setdefault(ln["dst"], {})[dp_] = (ln["src"], sp_)
    nin = {}
    for n, b in blocks.items():
        if b.get("type") in TAPPED:
            _tapped_check(n, b, srcs, spec.get("lines", []))
    lab211_vector = {"Assertion", "ManualSwitch"}
    for n, b in blocks.items():
        t = b["type"]
        if t == "Mux":
            nin[n] = _mux_inputs(b, n)
            hi = max(srcs.get(n, {}).keys(), default=0)
            if hi > nin[n]:
                # recordings (sim/b05): a line into port 3 of a Mux whose Inputs is 2 (the library's value
                # when left out) -- Simulink has no such port (InvSimulinkObjectName); the engine sized the Mux from
                # its lines
                raise G.GraphError("BAD_CONNECTION", "design.blocks.%s.Inputs" % n, "a line enters input %d of a Mux "
                                   "with %d inputs (Inputs %s%s); write Inputs as the number of inputs"
                                   % (hi, nin[n], b.get("Inputs", "2"), "" if "Inputs" in b else ", left out"))
        elif t == "BusCreator":
            nin[n] = _int1(b.get("Inputs", "2"), "design.blocks.%s.Inputs" % n)
        elif t == "Selector" and _sel_port(b):
            nin[n] = 2
        elif t in ("Demux", "Selector", "BusSelector"):
            nin[n] = 1
        else:
            nin[n] = max(srcs.get(n, {}).keys(), default=0)
    width = {}   # (block, out port) -> width
    layouts = {}
    shape = {}   # (block, out port) -> (m, n): a matrix signal, elements column-major:
    # only a matrix Inport (PortDimensions [m n]) and what the engine's matrix path builds from it -- recorded own
    # matrices (Constants, Reshape) keep recorded rules
    lut_tshape = {}   # a 2-D Lookup Table's table port shape, from the width pass for the lowering
    col_gain = set()   # Gain Matrix(K*u) with an m x 1 K on a scalar input: lowered element-wise
    mkey, mcollapse = {}, {}   # block -> its matrix support-domain key / its collapse plan
    cshape = {}   # (Constant, 1) -> (m, n): a matrix Constant, an operand shape for the engine's matrix path only

    def bus_names(n):
        b = blocks[n]
        k = _int1(b.get("Inputs", "2"), "design.blocks.%s.Inputs" % n)
        # a propagated signal name is written <name>: the element is name
        names = [x.strip().strip("<>") for x in str(b.get("ElementNames", "")).split(",")] if b.get("ElementNames") else \
            ["signal%d" % (j + 1) for j in range(k)]
        if len(names) != k or len(set(names)) != k or any(not x or "." in x for x in names):
            raise G.GraphError("BAD_PARAM", "design.blocks.%s.ElementNames" % n, "%d distinct element names without "
                               "dots required (nested buses are not supported)" % k)
        return names

    def selected(n):
        sel = [x.strip() for x in str(blocks[n].get("OutputSignals", "")).split(",") if x.strip()]
        if not sel:
            raise G.GraphError("BAD_PARAM", "design.blocks.%s.OutputSignals" % n, "names of the selected elements")
        if str(blocks[n].get("OutputAsBus", "off")) not in ("on", "off"):
            raise G.GraphError("BAD_PARAM", "design.blocks.%s.OutputAsBus" % n, "on or off")
        return sel

    def check_creator_names(n):
        """recordings: a Bus Selector output keeps the selected element's name; a Bus Creator cannot
        give it another (Simulink: SigHierPropBusSelOutputDoesNotMatchInput, or BusSelectorCantChangeSignalLabel when no
        Bus Selector reads the bus). Checked for every Bus Creator, read or not (D2: only a read bus was checked)."""
        ins = [srcs[n][k] for k in sorted(srcs.get(n, {}))]
        nms_ = bus_names(n)
        for k_, (s_, sp_) in enumerate(ins):
            sb_ = blocks.get(s_, {})
            if sb_.get("type") == "BusSelector" and str(sb_.get("OutputAsBus", "off")) != "on":
                sel_ = selected(s_)
                want_ = sel_[sp_ - 1].split(".")[-1] if sp_ - 1 < len(sel_) else None
                # a propagated name is written <name> on the line (Simulink shows the carried name that way)
                if want_ is not None and k_ < len(nms_) and nms_[k_].strip("<>") != want_:
                    raise G.GraphError("BAD_PARAM", "design.blocks.%s.ElementNames" % n, "element %d is named %r, "
                                       "but it is the Bus Selector %s output %r, which keeps its name (Simulink: "
                                       "SigHierPropBusSelOutputDoesNotMatchInput / BusSelectorCantChangeSignalLabel)"
                                       % (k_ + 1, nms_[k_], s_, want_))

    for n_bc in [x for x, b_ in blocks.items() if isinstance(b_, dict) and b_.get("type") == "BusCreator"]:
        check_creator_names(n_bc)
    # a line has one name in Simulink -- one signal into two Bus Creators under two element names is a
    # design no model holds as written (the name a Bus Selector looks for is the line's); it was answered
    named_ = {}
    for n_bc in sorted(x for x, b_ in blocks.items() if isinstance(b_, dict) and b_.get("type") == "BusCreator"):
        for k_, sig_ in enumerate([srcs[n_bc][k] for k in sorted(srcs.get(n_bc, {}))]):
            nms_ = bus_names(n_bc)
            if k_ >= len(nms_) or blocks.get(sig_[0], {}).get("type") in ("BusCreator", "BusSelector"):
                continue
            prev_ = named_.setdefault(tuple(sig_), (nms_[k_], n_bc))
            if prev_[0].strip("<>") != nms_[k_].strip("<>"):
                raise G.GraphError("BAD_PARAM", "design.blocks.%s.ElementNames" % n_bc, "the signal %s is element %r "
                                   "of Bus Creator %s and %r of %s: a Simulink line has one name, so no model holds "
                                   "this design -- name it the same in both" % (sig_[0], prev_[0], prev_[1],
                                                                               nms_[k_], n_bc))

    def layout(n, sp=1, depth=0):
        """[(element name, width)] of the bus on output sp of block n, or None if it is not a bus."""
        key = (n, sp)
        if key in layouts:
            return layouts[key]
        b = blocks[n]
        t = b["type"]
        if t in G.PENDING_TYPES:
            # a block the engine does not take yet (Tapped Delay: its output is a vector) was taken
            # as one element wide, and the Selector after it refused with an untrue width mismatch -- its own reason
            raise G.GraphError("UNSUPPORTED", "design.blocks.%s.type" % n, G.PENDING_TYPES[t])
        ins = [srcs[n][k] for k in sorted(srcs.get(n, {}))]
        lay = None
        if depth > 1000:
            raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "bus loop")
        if t == "BusCreator":
            nms_ = bus_names(n)
            check_creator_names(n)
            # an input that is itself a bus contributes its elements under dotted names
            # (signal1.signal1); the whole inner bus is still selectable by its own name (bus_offset)
            lay = []
            for nm_, x in zip(nms_, ins):
                sub = layout(*x, depth=depth + 1)
                if sub:
                    lay += [("%s.%s" % (nm_, cn), cw) for cn, cw in sub]
                else:
                    lay.append((nm_, wof(*x)))
        elif t == "BusSelector" and str(b.get("OutputAsBus", "off")) == "on":
            src = dict(layout(*ins[0], depth=depth + 1) or [])
            lay = []
            for nm in selected(n):
                if nm not in src:
                    raise G.GraphError("BAD_PARAM", "design.blocks.%s.OutputSignals" % n, "no bus element %r" % nm)
                lay.append((nm, src[nm]))
        elif (t in BUS_THROUGH or t == "BusAssignment") and ins:
            lay = layout(*ins[0], depth=depth + 1)
            if lay and t == "UnitDelay" and len(str(b.get("InitialCondition", "0")).strip().strip("[]").replace(
                    ",", " ").replace(";", " ").split()) > 1:
                # with a vector initial condition the Unit Delay no longer passes the bus (Simulink:
                # BusSelectorRequiresBusSignal after it); its output is a plain vector
                lay = None
        layouts[key] = lay
        return lay

    def bus_offset(n, nm):
        """(offset, width) of element nm in the bus feeding Bus Selector n."""
        src = srcs[n][1]
        lay = layout(*src)
        if lay is None and blocks[src[0]]["type"] == "BusSelector":
            sel_ = selected(src[0])
            up_ = layout(*srcs[src[0]][1]) or []
            if int(src[1]) <= len(sel_) and any(x.startswith(sel_[int(src[1]) - 1] + ".") for x, _ in up_):
                # the earlier Bus Selector output a nested bus -- Simulink runs the two-stage
                # selection; not supported here (it was called a Simulink rejection)
                raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "selecting from the nested bus %r that Bus "
                                   "Selector %s takes out is not supported; select %s.<element> directly"
                                   % (sel_[int(src[1]) - 1], src[0], sel_[int(src[1]) - 1]))
        if lay is None and blocks[src[0]]["type"] not in BUS_THROUGH + ("BusAssignment", "BusSelector") and \
                any(layout(*x) for x in (srcs.get(src[0], {}).get(k) for k in sorted(srcs.get(src[0], {}))) if x):
            # the bus went through a block that does not pass buses here (a Switch between two
            # buses: Simulink runs it) -- read from a saved model this check came first and said Simulink rejects it
            raise G.GraphError("UNSUPPORTED", "design.blocks." + src[0], "a bus into a %s block (buses may only pass "
                               "through Unit Delays, subsystem boundaries and Bus Selectors here)"
                               % blocks[src[0]]["type"])
        if lay is None:
            raise G.GraphError("TYPE_MISMATCH", "design.blocks." + n, "a Bus Selector needs a bus input")
        off = 0
        for name, w in lay:
            if name == nm:
                return off, w
            off += w
        off = 0
        span = None
        for name, w in lay:   # a whole inner bus: its elements are contiguous
            if name.startswith(nm + "."):
                span = (off, (span[1] if span else 0) + w) if span is None else (span[0], span[1] + w)
            off += w
        if span:
            return span
        raise G.GraphError("BAD_PARAM", "design.blocks.%s.OutputSignals" % n, "no bus element %r (the bus has %s)"
                           % (nm, ", ".join(x for x, _ in lay)))

    def tapped(n):
        """The elements of a Tapped Delay's output, oldest first (DelayOrder Oldest) or newest first: ('d', k) the
        input delayed k steps (k = 1 .. NumDelays), ('x', 0) the current input (includeCurrent on, at the newest
        end) -- recordings."""
        b = blocks[n]
        try:
            nd = _int1(b.get("NumDelays", "4"), "design.blocks.%s.NumDelays" % n)
        except (ValueError, G.GraphError) as e_:
            # Simulink runs NumDelays 'int8(3)' / '[3]' -- the integer expression reader (the engine's
            # fc.matlab_int_expr: exact integer arithmetic only), else the original refusal
            r_ = G.fc.matlab_int_expr(str(b.get("NumDelays", "4")).strip().strip("[]").strip())
            if r_ is None or r_[0].denominator != 1:
                if isinstance(e_, G.GraphError):
                    raise
                raise G.GraphError("BAD_PARAM", "design.blocks.%s.NumDelays" % n, "integer required")
            nd = int(r_[0])
        if not 1 <= nd <= 1024:
            raise G.GraphError("UNSUPPORTED", "design.blocks.%s.NumDelays" % n, "1 .. 1024 delays")
        if nd not in TAPPED_COUNTS:
            raise G.GraphError("UNSUPPORTED", "design.blocks.%s.NumDelays" % n, 'NumDelays %d: recorded %s'
                               "" % (nd, ", ".join(map(str, sorted(TAPPED_COUNTS)))))
        inc = str(b.get("includeCurrent", "off")).strip()
        order = str(b.get("DelayOrder", "Oldest")).strip()
        if inc not in ("on", "off") or order not in ("Oldest", "Newest"):
            raise G.GraphError("BAD_PARAM", "design.blocks." + n, "includeCurrent on / off, DelayOrder Oldest / Newest")
        newest_first = [("x", 0)] * (inc == "on") + [("d", k) for k in range(1, nd + 1)]
        return newest_first if order == "Newest" else newest_first[::-1]

    def bus_assign(n):
        """[(element name, offset, width, new-value source or None)] of Bus Assignment n."""
        b = blocks[n]
        import block_lowering
        # an omitted AssignedSignals takes the registered default signal1, not ''
        names = [x.strip() for x in str(b.get("AssignedSignals", block_lowering.DEFAULTS["BusAssignment"]["AssignedSignals"]))
                 .split(",") if x.strip()]
        lay = layout(*srcs[n][1])
        if lay is None:
            raise G.GraphError("TYPE_MISMATCH", "design.blocks." + n, "a Bus Assignment needs a bus on input 1")
        if not names or len(set(names)) != len(names) or any(x not in dict(lay) for x in names):
            raise G.GraphError("BAD_PARAM", "design.blocks.%s.AssignedSignals" % n, "distinct element names of the "
                               "bus (%s)" % ", ".join(x for x, _ in lay))
        if sorted(srcs.get(n, {})) != list(range(1, len(names) + 2)):
            raise G.GraphError("BAD_CONNECTION", "design.blocks." + n, "a bus and %d new values" % len(names))
        out, off = [], 0
        for nm, w_ in lay:
            s_ = srcs[n][names.index(nm) + 2] if nm in names else None
            if s_ is not None and wof(*s_) != w_:
                raise G.GraphError("TYPE_MISMATCH", "design.blocks." + n, "element %s has width %d, its new value %d"
                                   % (nm, w_, wof(*s_)))
            out.append((nm, off, w_, s_))
            off += w_
        return out

    busy = set()
    back_edges = set()   # signals a feedback loop came back to while their width was being worked out
    unresolved = [0]   # how many times a width still being computed was asked for
    mlw = {}

    def mlfn_widths(n):
        """(input widths, output widths) of MATLAB Function n (engine/fxmlsem.py widths)."""
        if n not in mlw:
            import fxmlfn
            import fxmlsem
            ins_ = [srcs[n][k] for k in sorted(srcs.get(n, {}))]
            iw = [wof(*x) for x in ins_]
            if any(x is None for x in iw):
                raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "a vector feedback loop through a MATLAB "
                                   "Function")
            try:
                ow = fxmlsem.widths(blocks[n].get("Script", ""), iw)
            except fxmlfn.MlError as e:
                raise G.GraphError(e.code, "design.blocks.%s.Script" % n, "line %d: %s" % (e.line, e.message))
            mlw[n] = (iw, ow)
        return mlw[n]

    def wof(n, sp=1):
        """Width of output sp of block n; None while it is being computed (a feedback loop: the loop's width comes
        from its other inputs). A width worked out while part of its loop was unresolved is not cached: it is worked
        out again once the loop's entry has its width (a vector accumulator loop)."""
        key = (n, sp)
        if key in width:
            return width[key]
        if key in busy:
            unresolved[0] += 1
            back_edges.add(key)
            return None
        before = unresolved[0]
        busy.add(key)
        try:
            w = _wof(n, sp)
        finally:
            busy.discard(key)
        if w is None:
            raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "signal width undetermined (a loop with no input "
                               "of known width)")
        if key in back_edges and w != 1 and not (fe_pre and n.startswith(fe_pre)):
            # recordings (main): a feedback loop that carries a vector was not recorded; recorded
            # recorded vector loop is the accumulator inside a For Each, which stays accepted
            raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "feedback loop through a vector signal (width %d) "
                               "is not supported outside a For Each subsystem: write the loop per element" % w)
        if unresolved[0] == before or not busy:
            width[key] = w
        return w

    def _wof(n, sp=1):
        key = (n, sp)
        b = blocks[n]
        t = b["type"]
        ins = [srcs[n][k] for k in sorted(srcs.get(n, {}))]
        need = nin.get(n) if t == "Mux" else (1 if t == "Demux" else (2 if _sel_port(b) else 1) if t == "Selector"
                                              else None)   # (recorded: a Selector with an index port has two inputs)
        if t == "Selector" and _sel_port(b) and 2 not in srcs.get(n, {}) and 1 in srcs.get(n, {}) and \
                str(b.get("IndexMode", "One-based")).strip() == "One-based":
            # the grounded index port reads 0, outside the one-based indices -- Simulink stops the
            # run at the first step (it does not run it, as the message below said)
            raise G.GraphError("SIM_DIAGNOSTIC", "design.blocks." + n, "Selector %s: the index port (input 2) is not "
                               "connected; Simulink grounds it (index 0), outside the one-based range 1..%s, and stops "
                               "the run at step 0 -- connect the index signal" % (n, b.get("InputPortWidth", "N")))
        extra_ = [k for k in sorted(srcs.get(n, {})) if need is not None and k > need]
        if extra_:
            raise G.GraphError("BAD_CONNECTION", "design.blocks." + n, "%s %s has %d input port(s) but a line goes into "
                               "port %d; Simulink cannot connect it" % (t, n, need, extra_[0]))
        if need is not None and sorted(srcs.get(n, {})) != list(range(1, need + 1)):
            # recordings (silently wrong: a Mux of Inputs 3 with port 3 unwired was answered 2 wide; Simulink
            # grounds the port and outputs 3 elements) and F8 (a Demux / Selector without its input gave INTERNAL)
            raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "%s %s: input port(s) %s not connected; Simulink "
                               "grounds an unconnected input (0), which is not supported here -- connect "
                               "every input (a Constant 0 of the right type where a zero is meant)"
                               % (t, n, ", ".join(str(k) for k in range(1, need + 1) if k not in srcs.get(n, {}))))
            # (Simulink then runs the model, or stops on another check -- recordings MuxInValidPortWidths -- either way
            # the engine does not take it)
        if t == "Inport":
            w, sh_ = _dims2(b, n)
            if sh_:
                shape[key] = sh_
        elif t == "Constant":
            w = len(_ints_or_vals(b.get("Value", "0"), "design.blocks.%s.Value" % n, _CONST_KIND))
            sh_ = _pmat(b.get("Value", "0"))
            if sh_:
                cshape[key] = sh_   # an operand shape for the engine's matrix path (no _MatKey: recorded answers the Constant)
        elif t in ("Mux", "BusCreator"):
            dw = _mux_widths(b, n) if t == "Mux" else None
            if dw is not None:
                got = [wof(*s) for s in ins]
                if len(got) < len(dw):
                    # an input of the width list without a line -- Simulink runs it, that element
                    # is 0; it was called a dimension mismatch Simulink rejects
                    raise G.GraphError("UNSUPPORTED", "design.blocks.%s" % n, "Mux %s Inputs %s with %d lines: an "
                                       "unconnected Mux input (Simulink feeds 0 there) is not supported; connect every "
                                       "input (e.g. a Constant 0)" % (n, b.get("Inputs"), len(got)))
                if len(dw) != len(got) or any(d != -1 and d != g for d, g in zip(dw, got)):
                    raise G.GraphError("TYPE_MISMATCH", "design.blocks.%s.Inputs" % n, "Mux input widths %s, the "
                                       "signals have %s (Simulink rejects the dimensions)" % (dw, got))
            # an input still being worked out is a feedback loop's back edge; it is taken as a
            # scalar here and the back edge's own width is checked to be 1 when it is known (a vector loop refused)
            w = sum(1 if x is None else x for x in (wof(*s) for s in ins))
        elif t == "BusSelector":
            sel = selected(n)
            if str(b.get("OutputAsBus", "off")) == "on":
                w = sum(bus_offset(n, nm)[1] for nm in sel)
            else:
                if sp > len(sel):
                    raise G.GraphError("BAD_CONNECTION", "design.blocks." + n, "output port %d of %d" % (sp, len(sel)))
                w = bus_offset(n, sel[sp - 1])[1]
        elif t == "Demux":
            total = wof(*ins[0])
            if total is None:   # inside a loop still being resolved (was INTERNAL)
                return None
            outs = b.get("Outputs", "2")
            if str(b.get("BusSelectionMode", "off")).strip() != "off":
                raise G.GraphError("UNSUPPORTED", "design.blocks.%s.BusSelectionMode" % n, "Demux BusSelectionMode %r: "
                                   "not supported (it was ignored)" % b.get("BusSelectionMode"))
            parts = _ints(outs, "design.blocks.%s.Outputs" % n)
            if str(outs).strip().startswith("[") and len(parts) == 1:
                # recordings (silent wrong): [4] is ONE output of width 4 (a width list), not four outputs
                raise G.GraphError("UNSUPPORTED", "design.blocks.%s.Outputs" % n, "Demux Outputs %s: a one-element "
                                   "width list (one output of that width) is not supported; write the number of "
                                   "outputs without brackets" % str(outs).strip())
            if len(parts) > 1 and (0 in parts or parts.count(-1) > 1 or any(x < -1 for x in parts)):
                # a zero width (Simulink refuses), several -1 (Simulink runs them) -- not
                # recorded either way, so not a request error
                raise G.GraphError("UNSUPPORTED", "design.blocks.%s.Outputs" % n, "Demux width list %s: only positive "
                                   "widths with at most one -1 were recorded" % parts)
            if len(parts) == 1:
                k = parts[0]
                if k < 1 or k > total:
                    raise G.GraphError("BAD_PARAM", "design.blocks." + n + ".Outputs", "Demux: %d outputs of a width %d"
                                       % (k, total))
                # a width the count does not divide is split with the larger parts first
                # (4 into 3: 2 + 1 + 1; Simulink warns "Nonuniform distribution", the export refuses that warning)
                q, r_ = divmod(total, k)
                if r_ and (total, k) not in DEMUX_UNEVEN:
                    raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "Demux: width %d not divisible by %d (only "
                                       "%s recorded)" % (total, k, ", ".join("%d into %d" % x for x in DEMUX_UNEVEN)))
                parts = [q + 1] * r_ + [q] * (k - r_)
            elif parts.count(-1) == 1 and all(x > 0 for x in parts if x != -1):
                # one -1 takes the rest of the width
                rest = total - sum(x for x in parts if x != -1)
                if rest < 1:
                    raise G.GraphError("BAD_PARAM", "design.blocks." + n + ".Outputs", "widths %s leave nothing for -1"
                                       % parts)
                parts = [rest if x == -1 else x for x in parts]
            if sum(parts) != total:
                raise G.GraphError("BAD_PARAM", "design.blocks." + n + ".Outputs", "widths %s do not sum to %d" % (parts, total))
            if not 1 <= sp <= len(parts):
                # a previous version: a line from a port the Demux does not have
                # crashed (INTERNAL); Simulink cannot build the model either
                raise G.GraphError("BAD_CONNECTION", "design.blocks." + n, "Demux %s has %d output port(s) (Outputs "
                                   "%s); a line leaves its port %d" % (n, len(parts), b.get("Outputs", "2"), sp))
            w = parts[sp - 1]
        elif t == "Selector" and _sel_port(b):
            # one output element per index element; index k picks input element k (one-based) or
            # k + 1 (zero-based), step by step -- a Multiport Switch per output element
            if len(ins) != 2:
                raise G.GraphError("BAD_CONNECTION", "design.blocks." + n, "Selector with an index port: 2 inputs")
            ipw = str(b.get("InputPortWidth", SEL_DEFAULTS["InputPortWidth"])).strip()
            win_ = wof(*ins[0])   # None inside a feedback loop still being resolved
            if win_ is not None and ipw not in ("-1", "") and ipw != str(win_):
                # with an index port the InputPortWidth (library 3) must also match the input
                # width -- Simulink stops (PortDimsMismatch42); it was not checked on this path
                raise G.GraphError("TYPE_MISMATCH", "design.blocks." + n + ".InputPortWidth",
                                   "Selector %s: InputPortWidth %s but the input signal has width %d (Simulink reports "
                                   "a port dimension mismatch); set InputPortWidth to %d or -1 (inherited)"
                                   % (n, ipw, wof(*ins[0]), wof(*ins[0])))
            w = wof(*ins[1])
        elif t == "Selector":
            u0_ = unresolved[0]
            win_ = wof(*ins[0])
            in_loop_ = unresolved[0] > u0_ or tuple(ins[0]) in back_edges
            if win_ is None:
                # recordings (was INTERNAL): inside a feedback loop the input width is not known yet; the output
                # width is the number of indices -- the checks run again once the loop is resolved (not cached)
                return len(_sel_indices(n, b, None))
            idx = _sel_indices(n, b, win_)
            ipw = _ipw_text(n, b)   # recordings (main): '3.0' is 3
            if in_loop_ and ((ipw not in ("-1", "") and ipw != str(win_)) or any(i > win_ for i in idx)):
                # a previous version: the input is a loop's back edge whose width was taken as a scalar
                # -- a loop through a vector Selector, not a dimension mismatch Simulink reports
                raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "a feedback loop through Selector %s (a vector "
                                   "signal) is not supported: write the loop per element" % n)
            if ipw not in ("-1", "") and ipw != str(wof(*ins[0])):
                # recordings (F7): InputPortWidth 4 on a width-3 signal -- Simulink rejects the model
                # (PortDimsMismatch); the engine ignored the parameter
                raise G.GraphError("TYPE_MISMATCH", "design.blocks." + n + ".InputPortWidth",
                                   "Selector %s: InputPortWidth %s but the input signal has width %d (Simulink reports "
                                   "a port dimension mismatch); set InputPortWidth to %d or -1 (inherited)"
                                   % (n, ipw, wof(*ins[0]), wof(*ins[0])))
            if any(i < 1 or i > wof(*ins[0]) for i in idx):
                raise G.GraphError("BAD_PARAM", "design.blocks." + n + ".Indices",
                                   "index outside the input width %d" % wof(*ins[0]))
            w = len(idx)
        elif t in TAPPED:
            if t == "TappedDelayEnabledSynchronous" and len(ins) > 1:
                if (wof(*ins[1]) or 1) > 1:
                    # Simulink stops (StateEnableSignalMustBeScalar); a tap vector per element was
                    # answered
                    raise G.GraphError("PORT_TYPE_INVALID", "design.blocks." + n, "Tapped Delay Enabled Synchronous "
                                       "%s with a %d-wide enable: Simulink requires a scalar enable "
                                       "(StateEnableSignalMustBeScalar)" % (n, wof(*ins[1])))
                # an enable computed from the block's own output with no delay on the way is an
                # algebraic loop for Simulink (UnableToSolveAlgLoop through its enabled subsystem); the same loop with
                # a Unit Delay runs (and on a Unit Delay Enabled Synchronous it runs: recorded M2b / M2c)
                nofb_ = ("UnitDelay", "Delay", "Memory", "DiscreteIntegrator", "TappedDelay")
                seen_, stack_ = set(), [ins[1][0]]
                while stack_:
                    x_ = stack_.pop()
                    if x_ == n:
                        raise G.GraphError("PORT_TYPE_INVALID", "design.blocks." + n, "the enable of Tapped Delay "
                                           "Enabled Synchronous %s is computed from its own output with no delay on "
                                           "the way: Simulink stops with an algebraic loop (UnableToSolveAlgLoop)" % n)
                    if x_ in seen_ or blocks.get(x_, {}).get("type") in nofb_:
                        continue
                    seen_.add(x_)
                    stack_ += [s_[0] for s_ in srcs.get(x_, {}).values()]
            # in a feedback loop the input's width is still being worked out (None, a back edge) --
            # taken as a scalar here and checked when known (it was refused as "a vector input")
            if (wof(*ins[0]) or 1) != 1:
                raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "Tapped Delay of a vector input (scalar inputs "
                                   "recorded)")
            w = len(tapped(n))
        elif t == "BusAssignment":
            w = wof(*ins[0])
            bus_assign(n)
        elif t == "SignalSpecification":
            w = wof(*ins[0]) if ins else 1
            import block_lowering
            dims = block_lowering.dims_of(b.get("Dimensions", "-1"), "design.blocks." + n + ".Dimensions")
            # recordings (sim/b01 A5_06 / 12 / 14): a row or higher-dimension shape with one non-1 dimension equal to
            # the width ([1 2], [1 3], [2 1 1]) is accepted by Simulink on a 1-D signal of that width
            big = [x for x in (dims or []) if x != 1]
            want = None if dims is None else (dims[0] if len(dims) == 1 else
                                              ((big[0] if big else 1) if len(big) <= 1 else -2))
            if want == -2 or (want is not None and want != w):
                raise G.GraphError("TYPE_MISMATCH", "design.blocks." + n + ".Dimensions", "Signal Specification "
                                   "Dimensions %s, the input has width %d (Simulink: PortDimsMismatch42)"
                                   % (b.get("Dimensions"), w))
        elif t == "MATLABFunction":
            # the function sees its vector arguments whole; output widths from the script
            w = mlfn_widths(n)[1][sp - 1] if sp <= len(mlfn_widths(n)[1]) else 1
        else:
            if t not in BUS_THROUGH and any(layout(*x) is not None for x in ins):
                raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "a bus into a %s block (buses may only pass "
                                   "through Unit Delays, subsystem boundaries and Bus Selectors)" % t)
            if fe_pre and n.startswith(fe_pre) and t not in ELEMENTWISE:
                raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "%s inside a For Each subsystem: only "
                                   "element-wise blocks are supported there (each partition is one element)" % t)
            if t == "MultiportSwitch" and ins and (wof(*ins[0]) or 1) > 1:
                raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "Multiport Switch with a vector control (a "
                                   "scalar control over vector data is recorded)")
            if t == "MultiportSwitch" and len(ins) == 2 and (wof(*ins[1]) or 1) > 1 and \
                    str(b.get("DataPortOrder", "One-based contiguous")).strip() not in (
                        "One-based contiguous", "Zero-based contiguous"):
                raise G.GraphError("UNSUPPORTED", "design.blocks.%s.DataPortOrder" % n, "Multiport Switch with one "
                                   "data port on a vector and DataPortOrder %r: recorded with One-based / Zero-based "
                                   "contiguous only" % b.get("DataPortOrder"))
            if t == "Repeat" and (wof(*ins[0]) or 1) > 1 and \
                    str(b.get("InputProcessing", "Columns as channels (frame based)")).strip() != \
                    "Elements as channels (sample based)":
                # a frame-based vector is one channel of several samples, not elements (width 2,
                # frame based -- not the element-wise repeat)
                raise G.GraphError("UNSUPPORTED", "design.blocks." + n + ".InputProcessing", "Repeat frame based on a "
                                   "vector (width %d): a frame, not separate elements -- not supported; use sample "
                                   "based (Elements as channels)" % wof(*ins[0]))
            if t == "Reshape" and str(b.get("OutputDimensionality", "")).strip() == "Customize":
                # recordings: [4 1], [1 4], [2 2] of a width-4 signal -- the elements in MATLAB's
                # column-major order, which is the engine's element order; the dimensions must hold the width
                dims_ = _ints(str(b.get("OutputDimensions", "[1,1]")), "design.blocks.%s.OutputDimensions" % n)
                if not dims_ or any(d_ <= 0 for d_ in dims_):   # recordings (as block_lowering.params)
                    raise G.GraphError("BAD_PARAM", "design.blocks.%s.OutputDimensions" % n, "Reshape OutputDimensions "
                                       "%s: Simulink rejects dimensions that are not positive integers "
                                       "(Simulink:Engine:InvSignalDims)" % b.get("OutputDimensions"))
                pr_ = 1
                for d_ in dims_:
                    pr_ *= d_
                if pr_ == (wof(*ins[0]) or 1) and len(dims_) != 2:
                    # Customize [4] of a width-4 signal runs in Simulink -- not recorded here
                    raise G.GraphError("UNSUPPORTED", "design.blocks.%s.OutputDimensions" % n, "Reshape Customize "
                                       "with %d dimension(s) %s: only two dimensions were recorded" % (len(dims_), dims_))
                if len(dims_) != 2 or pr_ != (wof(*ins[0]) or 1):
                    raise G.GraphError("TYPE_MISMATCH", "design.blocks.%s.OutputDimensions" % n, "Reshape to %s of a "
                                       "width-%d signal (Simulink: the element counts must agree)" % (dims_, wof(*ins[0]) or 1))
            if t in ("Lookup1D", "Lookup2D") and _lut_port_srcs(b, t):
                # the table and / or the breakpoints from input ports -- the inputs
                # scalar on the first ports, then the table (if from a port; 2-D: a matrix signal), then the
                # breakpoints (if from ports), each a vector
                u0_ = unresolved[0]
                ws = [wof(*s) for s in ins]
                if unresolved[0] > u0_ or key in back_edges or any(x is None for x in ws) or \
                        any(tuple(x) in back_edges for x in ins):
                    # (the engine's 'assumed' check; recorded tracks loops with unresolved / back_edges)
                    raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "a feedback loop through a 1-D Lookup "
                                       "Table with its table / breakpoints from ports is not supported")
                tsh_ = shape.get(tuple(ins[len(ins) - len(_lut_port_srcs(b, t))])) if "T" in _lut_port_srcs(b, t) \
                    else None
                _lut_port_widths(n, b, ws, t, tsh_)
                lut_tshape[n] = tsh_
                return 1
            if t not in ELEMENTWISE:
                ws = [wof(*s) for s in ins]
                if any(x is not None and x > 1 for x in ws):
                    raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "%s on a vector signal" % t)
                w = 1
            else:
                u0_ = unresolved[0]
                ws_raw_ = [wof(*s) for s in ins]
                in_loop0_ = unresolved[0] > u0_ or any(tuple(x) in back_edges for x in ins)
                ws = [x for x in ws_raw_ if x is not None] or [1]
                # (the engine's two Multiport Switch refusals are not taken: recorded answers one vector data port as recorded,
                # recordings, and refuses a vector control above)
                if any(shape.get(tuple(s)) for s in ins):
                    # main recordings: a signal from a matrix Inport -- the engine's matrix rules
                    # and support domain (engine/matrix_domain.json, checked after typing on _MatKey)
                    if in_loop0_ or any(x is None for x in ws_raw_):
                        raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "a feedback loop through %s %s (a "
                                           "matrix signal) is not supported" % (t, n))
                    ish = [shape.get(tuple(s)) or cshape.get(tuple(s)) for s in ins]
                    psh = {k_: _pmat(b[k_]) for k_ in VECPARAM.get(t, ()) if k_ in b and t != "Constant"}
                    if t == "Gain" and str(b.get("Multiplication", "")).strip().startswith("Matrix"):
                        psh = {}   # Gain Matrix(K*u) / (u*K): K is a matrix product's operand, not element-wise
                    return _matrix_w(n, b, t, ins, ws_raw_, ish, psh, shape, key, mkey, mcollapse)
                if len(ins) == 1 and ws[0] > 1 and t in ("Sum", "Product") and _collapse_form(b, t) and \
                        str(b.get("CollapseMode", "All dimensions")).strip() == "Specified dimension":
                    # main recordings (recordings SumCollapse / ProdCollapse on a 1-D vector): 'Specified
                    # dimension' 1 reduces every element; dimension 2 Simulink rejects (InvalidOperationDimExceedsBound).
                    # Checked against the recorded tuples after typing (_MatKey). (recorded keeps its own 'All
                    # dimensions' reductions, recordings.)
                    if in_loop0_:
                        raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "a feedback loop through %s %s (a "
                                           "vector collapse) is not supported" % (t, n))
                    mode_, d_ = _collapse_mode(b, n)
                    if d_ != 1:
                        raise G.GraphError("BAD_PARAM", "design.blocks.%s.CollapseDim" % n, "%s %s: CollapseDim %d on "
                                           "a 1-D vector of width %d exceeds its one dimension (Simulink: "
                                           "InvalidOperationDimExceedsBound)" % (t, n, d_, ws[0]))
                    mcollapse[n] = ("vector", mode_, d_, (ws[0],))
                    mkey[n] = {"block": t, "shape": [ws[0]], "params": {}, "collapse": [mode_, d_]}
                    return 1
                port_src = (t == "Delay" and (str(b.get("DelayLengthSource", "Dialog")) == "Input port" or
                                              str(b.get("InitialConditionSource", "Dialog")) == "Input port")) or \
                    (t == "ShiftArithmetic" and str(b.get("BitShiftNumberSource", "Dialog")) == "Input port")
                if port_src and max(ws) > 1:
                    if t == "Delay" and len(ws) > 1 and ws[1] > 1:
                        # a vector on the length port (PortDimsMismatch) or on the initial-
                        # condition port of a sample-based vector (InvalidICForSampleBasedNonScalarInput) is rejected
                        raise G.GraphError("PORT_TYPE_INVALID", "design.blocks." + n, "Delay %s with its length or "
                                           "initial condition from port 2: Simulink rejects a %d-wide signal there "
                                           "(the length port is scalar; a vector initial condition is not accepted "
                                           "for sample-based input)" % (n, ws[1]))
                    # port-sourced parameters are recorded on scalar signals only
                    raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "%s with a parameter from an input port on "
                                       "a vector signal is not recorded; use scalar signals, or record this shape to "
                                       "open it" % t)
                # a parameter written '[]' is the library's empty default (an Outport's InitialOutput),
                # not a width-0 vector
                pw = [_vp_len(b[k], "design.blocks.%s.%s" % (n, k)) for k in _vecparams(t, b)
                      if k in b and t != "Constant" and str(b[k]).strip() not in ("[]", "")]
                w = max(ws + pw)
                if t == "MultiportSwitch" and len(ins) == 2 and (wof(*ins[1]) or 1) > 1:
                    w = 1   # one data port on a vector -- the control picks one element
                if t == "Gain" and str(b.get("Multiplication", "")).strip() == "Matrix(K*u)":
                    # recordings (18 models, 45,056 element-steps equal): K an m x n
                    # matrix, u a vector of n -> a vector of m; lowered to n element Gains (the whole matrix's best
                    # precision) and a Sum per row (_matgain_lower); recorded shapes only (MATGAIN_SHAPES)
                    K = _matrix(b.get("Gain", "1"), "design.blocks.%s.Gain" % n)
                    if any(str(v).strip().lstrip("+-").lower() in ("inf", "nan") for r in K for v in r):
                        # inf in the matrix gave INTERNAL; Simulink rejects it (InvParamSetting)
                        raise G.GraphError("BAD_PARAM", "design.blocks.%s.Gain" % n, "Gain values must be finite "
                                           "(Simulink rejects inf / NaN), got %r" % b.get("Gain"))
                    mm, nn = len(K), len(K[0]) if K else 0
                    src0_ = ins[0][0] if ins and isinstance(ins[0], (tuple, list)) else None
                    sb_ = blocks.get(src0_) if src0_ is not None else None
                    if isinstance(sb_, dict) and sb_.get("type") == "Inport":
                        sh_ = _dims_shape(sb_.get("PortDimensions", "1"))
                        if sh_ is not None and sh_ != ("s",) and not (sh_[0] == "col" and sh_[1] == nn):
                            # K = [2 3] times an explicit [1 2] row input --
                            # Simulink stops with MatGainInvInDimsKu (the inner dimensions do not agree); the
                            # input was flattened to a vector of 2 and answered. A [r c] input must be n x 1
                            raise G.GraphError("TYPE_MISMATCH", "design.blocks." + n, "Gain Matrix(K*u): K is %d x %d "
                                               "but the input %s is %s; K*u needs an input of %d x 1 (a vector of %d or "
                                               "a column) -- Simulink stops (MatGainInvInDimsKu)"
                                               % (mm, nn, src0_, str(sb_.get("PortDimensions")).strip(), nn, nn))
                    if nn == 1 and mm > 1 and ws == [1]:
                        # an m x 1 K on a scalar input: K*u is the element-wise product (recordings
                        # 2x1, recordings -- every such design equal), lowered as element Gains
                        col_gain.add(n)
                        width[key] = mm
                        return mm
                    if (mm, nn) not in MATGAIN_SHAPES and (mm, nn) not in _matgain_tuple_shapes():
                        raise G.GraphError("UNSUPPORTED", "design.blocks.%s.Gain" % n, "Gain Matrix(K*u) with a %d x %d "
                                           "matrix is not recorded (recorded: %s); record this shape to open it"
                                           % (mm, nn, ", ".join("%dx%d" % x for x in sorted(MATGAIN_SHAPES))))
                    if (len(ws) != 1 or ws[0] != nn) and \
                            in_loop0_:
                        # recordings: the width met a loop's back edge (assumed 1)
                        raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "a feedback loop through Gain "
                                           "Matrix(K*u) %s (a vector signal) is not supported" % n)
                    if len(ws) != 1 or ws[0] != nn:
                        raise G.GraphError("TYPE_MISMATCH", "design.blocks." + n, "Gain Matrix(K*u): K has %d "
                                           "columns but the input has width %s (Simulink: dimension mismatch)"
                                           % (nn, ws))
                    # (the engine's fix: a matrix gain that is itself the loop's back edge -- refused here by
                    # wof's back-edge check, a vector width on a back edge; the engine's 'assumed' set is not this pass's)
                    return mm
                # the matrix modes of Gain are a matrix product (K = [2 3], u = [1 5] gives 17, not
                # [2 15]); only element-wise multiplication has been observed (element-wise or scalar), so a
                # matrix mode on a vector signal or a vector gain is refused until it is recorded
                if t in ("Gain", "Product") and w > 1 and \
                        not str(b.get("Multiplication", "Element-wise(K.*u)")).startswith("Element-wise"):
                    raise G.GraphError("UNSUPPORTED", "design.blocks." + n + ".Multiplication",
                                       t + " %s on a vector (input widths %s, gain width %s): only element-wise "
                                       "multiplication is supported (matrix products not yet observed)"
                                       % (b.get("Multiplication"), ws, pw or [1]))
                if t == "Math" and w > 1 and str(b.get("Operator", "exp")).strip() not in (
                        "conj", "mod", "rem", "square", "reciprocal", "magnitude^2", "transpose"):   # rem: recordings
                    # element-wise, equal (transpose of a 1-D vector: the elements unchanged, ma032 /
                    # ma120 / ma121 / ma125 / ma172 / ma173 / ma176)
                    # the other operators were recorded on scalars only; transpose /
                    # hermitian turn a vector into a matrix, which the engine does not carry
                    raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "Math Function %s on a vector (width %d): "
                                       "recorded on scalars only" % (str(b.get("Operator")).strip(), w))
                if t == "Delay" and w > 1 and (str(b.get("ShowEnablePort", "off")).strip() != "off" or
                                               str(b.get("ExternalReset", "None")).strip() != "None"):
                    # the enable / reset ports were recorded on scalar data only
                    raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "Delay with an enable or reset port on a "
                                       "vector signal (widths %s): only scalar data was recorded" % ws)
                if t == "DataTypeDuplicate" and any(x not in (1, w) for x in ws):
                    # two different vector widths are not recorded for Data Type Duplicate, nor for
                    # Concatenate (lowered through an internal one, named back to the user's block; a scalar next to
                    # a vector is recorded, recordings); Vector Concatenate is meant to
                    # join any widths, so this is "not supported yet", not "Simulink rejects it"
                    user = n[len(INTERNAL):-len("/dtd")] if n.startswith(INTERNAL) and n.endswith("/dtd") else n
                    kind = "Concatenate" if user != n else "Data Type Duplicate"
                if t == "DataTypeDuplicate" and any(x not in (1, w) for x in ws) and kind == "Concatenate":
                    # a Vector Concatenate of widths 1, 2, 3 is the
                    # inputs side by side; its type check compares one element of each input (a vector has one type)
                    w = 1
                elif t == "DataTypeDuplicate" and any(x not in (1, w) for x in ws):
                    raise G.GraphError("UNSUPPORTED", "design.blocks." + user, "%s of inputs with different widths %s "
                                       "is not supported (not recorded; only equal widths, or scalars next to a vector)" % (kind, ws))
                if w > 1 and any(x not in (1, w) for x in ws + pw):
                    raise G.GraphError("TYPE_MISMATCH", "design.blocks." + n, "input / parameter widths %s differ"
                                       % (ws + pw))
                np_ = _declared_ports(b, t)
                if np_ is not None and np_ > len(ins):
                    # a Sum with Inputs left out (library '|++') and one line was taken as a
                    # one-input reduction (width 1); Simulink grounds port 2 and adds element-wise (width 3)
                    raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "%s %s has %d input ports%s but %d "
                                       "connected; Simulink grounds an unconnected input (0), which is not "
                                       "supported here -- connect every port (a Constant 0 where a zero is meant), or "
                                       "set Inputs for the ports you use" % (t, n, np_, " (Inputs left out: the "
                                       "library default %s)" % ("'|++'" if t == "Sum" else "'2'") if "Inputs" not in b
                                       else "", len(ins)))
                if w > 1 and len(ins) == 1 and t in COLLAPSE and _one_input(b, t) and not \
                        (fe_pre and n.startswith(fe_pre)):
                    w = 1   # reduced along the elements: becomes a w-input scalar block
                elif w > 1 and t == "Product" and len(ins) == 1 and str(b.get("Inputs", "2")).strip() in ("1", "*"):
                    w = 1   # the product of the elements, built as a chain below
                elif w > 1 and t == "Product" and len(ins) == 1 and str(b.get("Inputs", "2")).strip() == "/":
                    raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "one-input Product '/' over a vector "
                                       "(not recorded)")
                elif w > 1 and t == "Sum" and len(ins) == 1 and str(b.get("Inputs", "+")).replace("|", "").strip() \
                        not in ("+", "1", "-"):
                    # parameter sweep: a one-input Sum "-" over a vector was negated element
                    # by element; Simulink presumably reduces it like "+", which has never been recorded with "-"
                    raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "one-input Sum %r over a vector: only "
                                       "\"+\" (sum of the elements) has been recorded" % b.get("Inputs"))
        if any(shape.get(tuple(s)) for s in ins):
            # main recordings: a matrix (from a matrix Inport) reaching a block the engine's matrix path does not take (Mux /
            # Demux / Selector / buses, Tapped Delay, ...) -- not recorded
            raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "%s %s on a matrix signal is not supported "
                               "(recorded with matrices: %s)" % (t, n, ", ".join(MATRIX_BLOCKS)))
        return w

    for n in blocks:
        if blocks[n]["type"] == "BusSelector" and str(blocks[n].get("OutputAsBus", "off")) != "on":
            for sp in range(1, len(selected(n)) + 1):
                wof(n, sp)
            continue
        if blocks[n]["type"] == "Demux":
            # (recorded keeps its own width rules -- one -1, recordings -- in wof; the engine's _demux_parts
            # refuses -1)
            parts = len(_ints(blocks[n].get("Outputs", "2")))
            k = _ints(blocks[n].get("Outputs", "2"))[0] if parts == 1 else parts
            for sp in range(1, k + 1):
                wof(n, sp)
        else:
            wof(n)

    for n, b in blocks.items():
        if b["type"] == "Outport" and str(b.get("PortDimensions", "-1")).strip() not in ("-1", ""):
            w_o = _width(b, n)
            if (n, 1) in width and width[(n, 1)] != w_o:
                # Simulink PortDimsMismatch
                raise G.GraphError("TYPE_MISMATCH", "design.blocks.%s.PortDimensions" % n, "Outport %s: "
                                   "PortDimensions %s but the signal has width %d (Simulink reports a port dimension "
                                   "mismatch)" % (n, b["PortDimensions"], width[(n, 1)]))

    def elem(n, sp, k):
        """(block, output port) of the scalar signal carrying element k (1-based) of output sp of block n. The port
        is kept: a multi-output block (HDL Counter count_hit, RAM, the DSP HDL blocks' valid, a MATLAB Function)
        in a design with vectors used to be rewired to its port 1."""
        b = blocks[n]
        t = b["type"]
        ins = [srcs[n][j] for j in sorted(srcs.get(n, {}))]
        if t in ("Mux", "BusCreator"):
            for s in ins:
                ws = wof(*s)
                if k <= ws:
                    return elem(s[0], s[1], k)
                k -= ws
            raise G.GraphError("INTERNAL", "design.blocks." + n, "Mux element out of range")
        if t == "BusSelector":
            names = selected(n) if str(b.get("OutputAsBus", "off")) == "on" else [selected(n)[sp - 1]]
            for nm in names:
                off, w = bus_offset(n, nm)
                if k <= w:
                    return elem(ins[0][0], ins[0][1], off + k)
                k -= w
            raise G.GraphError("INTERNAL", "design.blocks." + n, "Bus Selector element out of range")
        if t == "Demux":
            off = sum(width[(n, j)] for j in range(1, sp))
            return elem(ins[0][0], ins[0][1], off + k)
        if t == "Selector" and _sel_port(b):
            return (n if width[(n, 1)] == 1 else "%s[%d]" % (n, k)), 1
        if t == "Selector":
            idx = _sel_indices(n, b, wof(*ins[0]))
            return elem(ins[0][0], ins[0][1], idx[k - 1])
        if t in TAPPED:
            kind, kk = tapped(n)[k - 1]
            return elem(ins[0][0], ins[0][1], 1) if kind == "x" else ("%s\x1dd%d" % (n, kk), 1)
        if t == "BusAssignment":
            for nm, off, w_, s_ in bus_assign(n):
                if off < k <= off + w_:
                    return elem(s_[0], s_[1], k - off) if s_ is not None else elem(ins[0][0], ins[0][1], k)
            raise G.GraphError("INTERNAL", "design.blocks." + n, "Bus Assignment element out of range")
        if t == "MATLABFunction" and n in mlv:
            ow = mlfn_widths(n)[1]
            return n, sum(ow[:sp - 1]) + k   # the flat output port of element k of output sp
        return (n if width[(n, 1)] == 1 else "%s[%d]" % (n, k)), sp

    for n in blocks:
        t = blocks[n].get("type")
        if t in lab211_vector and n in srcs:
            ws = [wof(*srcs[n][k]) for k in sorted(srcs[n])]
            if t == "Assertion" and any(w_ and w_ > 1 for w_ in ws):
                # recordings (8 models, widths 2 / 4, boolean and sfix16_En8): a vector fails at a step when any
                # element is 0, once per step (one stop / one warning, not one per element) -- the assertion of the
                # one-input AND of the elements (the reduction recorded in recordings); the design is rewritten
                # so and this pass runs again
                src_ = srcs[n][1]
                nm_ = INTERNAL + n + "/all"
                b2 = dict(spec["blocks"], **{nm_: {"type": "LogicalOperator", "Operator": "AND", "Inputs": "1"}})
                l2 = [ln for ln in spec.get("lines", []) if ln.get("dst") != n]
                l2 += [dict({"src": src_[0], "dst": nm_}, **({"sport": src_[1]} if src_[1] != 1 else {})),
                       {"src": nm_, "dst": n}]
                return scalarise(dict(spec, blocks=b2, lines=l2))
            if t == "ManualSwitch" and len(set(w_ for w_ in ws if w_)) > 1:
                # a scalar and a vector would be broadcast here; Simulink's rule was not recorded
                raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "Manual Switch of inputs of widths %s "
                                   "(recorded with equal widths)" % ws)
    # a For Each was recorded partitioning 4-element vectors only
    #
    for ln in spec["lines"]:
        if fe_pre and ln["dst"].startswith(fe_pre) and not ln["src"].startswith(fe_pre):
            w_in = wof(ln["src"], int(ln.get("sport", 1)))
            if w_in != 4:
                raise G.GraphError("UNSUPPORTED", "design.blocks." + ln["dst"], ('a For Each subsystem fed a '
                    '%d-element signal: recorded '
                    'with 4-element inputs only') % w_in)
    out_blocks, out_lines, ties, element = {}, [], {}, {}
    import json
    mlv = {n for n, b in blocks.items() if b["type"] == "MATLABFunction"
           and any(w > 1 for w in mlfn_widths(n)[0] + mlfn_widths(n)[1])}
    for n, b in blocks.items():
        t = b["type"]
        if t == "Selector" and _sel_port(b):
            ins_ = [srcs[n][j] for j in sorted(srcs.get(n, {}))]
            win = wof(*ins_[0])
            zb = str(b.get("IndexMode", "One-based")) == "Zero-based"
            for k in range(1, width[(n, 1)] + 1):
                name = n if width[(n, 1)] == 1 else "%s[%d]" % (n, k)
                out_blocks[name] = {"type": "MultiportSwitch", "Inputs": str(win),
                                    "DataPortOrder": "Zero-based contiguous" if zb else "One-based contiguous",
                                    "_SelIdx": n}
                element[name] = (n, k)
                out_lines.append(_line(elem(ins_[1][0], ins_[1][1], k if wof(*ins_[1]) > 1 else 1), name, 1))
                for j in range(1, win + 1):
                    out_lines.append(_line(elem(ins_[0][0], ins_[0][1], j), name, j + 1))
            continue
        if t == "MultiportSwitch" and len(srcs.get(n, {})) == 2 and (wof(*srcs[n][2]) or 1) > 1:
            # recordings (12 models): Inputs 1 on a width-w vector picks element c (One-based 1..w,
            # Zero-based 0..w-1) into a scalar; an out-of-range control stops Simulink whatever DiagnosticForDefault
            # (MPSwitchControlInputRangeError) -- a w-port switch on the elements, with the error diagnostic
            win = wof(*srcs[n][2])
            out_blocks[n] = dict({k: v for k, v in b.items() if k not in ("Inputs", "DiagnosticForDefault")},
                                 Inputs=str(win), DiagnosticForDefault="Error")
            element[n] = (n, 1)
            out_lines.append(_line(elem(srcs[n][1][0], srcs[n][1][1], 1), n, 1))
            for j in range(1, win + 1):
                out_lines.append(_line(elem(srcs[n][2][0], srcs[n][2][1], j), n, j + 1))
            continue
        if t in STRUCT:
            continue
        if t in TAPPED:
            ins_ = [srcs[n][j] for j in sorted(srcs.get(n, {}))]
            nd = max(k for kind, k in tapped(n) if kind == "d")
            prev = elem(ins_[0][0], ins_[0][1], 1)
            # a vector vinit gives output element j its own initial value -- the delay
            # stage that element shows starts at vinit(j)
            vv = str(b.get("vinit", "0")).strip()
            ic_of = {}
            if vv.startswith("["):
                vals = _ints_or_vals(vv, "design.blocks.%s.vinit" % n)
                outs_ = [x for x in tapped(n) if x[0] == "d"]   # includeCurrent's current input has no stage
                if len(vals) != len(outs_):
                    raise G.GraphError("BAD_PARAM", "design.blocks.%s.vinit" % n, "vinit has %d values for %d delays"
                                       % (len(vals), len(outs_)))
                ic_of = {k_: repr(float(v_)) for (kind_, k_), v_ in zip(outs_, vals)}
            for k in range(1, nd + 1):
                nm = "%s\x1dd%d" % (n, k)
                if t == "TappedDelay":
                    out_blocks[nm] = {"type": "UnitDelay", "InitialCondition": ic_of.get(k, str(b.get("vinit", "0"))),
                                      "_HalfGridIC": "refuse"}   # 
                    out_lines.append(_line(prev, nm, 1))
                else:
                    if len(ins_) != 2:
                        raise G.GraphError("BAD_CONNECTION", "design.blocks." + n, "data and enable inputs")
                    out_blocks[nm] = {"type": "UnitDelayEnabledSynchronous",
                                      "InitialCondition": ic_of.get(k, str(b.get("vinit", "0")))}
                    out_lines.append(_line(prev, nm, 1))
                    out_lines.append(_line(elem(ins_[1][0], ins_[1][1], 1), nm, 2))
                element[nm] = (n, k)
                prev = (nm, 1)
            continue
        if t == "BusAssignment":
            # no block of its own: its elements are the bus's or the new values (elem); each new value must have its
            # element's type (Simulink: BusComp_DataTypeError) -- a Data Type Duplicate per element
            for nm, off, w_, s_ in bus_assign(n):
                if s_ is None:
                    continue
                bs = srcs[n][1]
                for k in range(1, w_ + 1):
                    chk = "%s\x1dchk_%s_%d" % (n, nm, k)
                    out_blocks[chk] = {"type": "DataTypeDuplicate", "NumInputPorts": "2"}
                    out_lines.append(_line(elem(s_[0], s_[1], k), chk, 1))
                    out_lines.append(_line(elem(bs[0], bs[1], off + k), chk, 2))
            continue
        if n in mlv:
            # one block, its vector arguments flattened element by element into its input ports; the lowering
            # (engine/fxhdl.py _mlfn) gives every output element its own port
            iw, ow = mlfn_widths(n)
            out_blocks[n] = dict(b, _InWidths=json.dumps(iw), _OutWidths=json.dumps(ow))
            element[n] = (n, 1)
            port = 0
            for (s, sp), w in zip([srcs[n][j] for j in sorted(srcs.get(n, {}))], iw):
                for kk in range(1, w + 1):
                    port += 1
                    out_lines.append(_line(elem(s, sp, kk), n, port))
            continue
        w = width[(n, 1)]
        ins = [srcs[n][j] for j in sorted(srcs.get(n, {}))]
        if t == "Gain" and str(b.get("Multiplication", "")).strip() == "Matrix(K*u)" and n not in col_gain:
            _matgain_lower(n, b, ins, wof, elem, out_blocks, out_lines, element)
            continue
        if t in ("Lookup1D", "Lookup2D") and _lut_port_srcs(b, t):
            tsh_ = lut_tshape.get(n)
            _lutport_lower(n, b, ins, wof, elem, out_blocks, out_lines, element, t, tsh_)
            continue
        if n in mcollapse:
            kind_, mode_, d_, sh_ = mcollapse[n]
            s0, sp0 = ins[0]
            if kind_ == "vector" or mode_ == "All dimensions":
                groups = [list(range(1, (sh_[0] if kind_ == "vector" else sh_[0] * sh_[1]) + 1))]
            elif d_ == 1:   # down each column: a 1 x n result
                groups = [[j * sh_[0] + i + 1 for i in range(sh_[0])] for j in range(sh_[1])]
            else:           # along each row: an m x 1 result
                groups = [[j * sh_[0] + i + 1 for j in range(sh_[1])] for i in range(sh_[0])]
            for gi, grp in enumerate(groups, start=1):
                name = n if len(groups) == 1 else "%s[%d]" % (n, gi)
                nb = {key: v for key, v in b.items() if key not in ("PortDimensions", "CollapseMode", "CollapseDim")}
                if t == "Sum":
                    nb["Inputs"] = "+" * len(grp)
                    nb["_VecReduce"] = "1"
                else:
                    nb["Inputs"] = "*" * len(grp)
                nb["_MatKey"] = json.dumps(mkey[n], sort_keys=True)
                if len(groups) > 1 and gi > 1:
                    nb["_Tie"] = "%s[1]" % n
                    ties[name] = "%s[1]" % n
                out_blocks[name] = nb
                element[name] = (n, gi)
                for port, e in enumerate(grp, start=1):
                    out_lines.append(_line(elem(s0, sp0, e), name, port))
            continue
        if w == 1 and len(ins) == 1 and wof(*ins[0]) > 1 and (t in COLLAPSE and _one_input(b, t) or (
                t == "Product" and str(b.get("Inputs", "2")).strip() in ("1", "*"))):
            m = wof(*ins[0])
            nb = {key: v for key, v in b.items() if key != "PortDimensions"}
            if t == "Product" and str(b.get("Inputs", "2")).strip() in ("1", "*"):
                # the elements multiplied one after another;
                # the first product takes the block's output rule (inherited: the two-element product's type,
                # sfix32_En16 for sfix16_En8), every later one keeps that type
                s0, sp0 = ins[0]
                prev = elem(s0, sp0, 1)
                for kk in range(2, m + 1):
                    cn = n if kk == m else "%s\x1dpr%d" % (n, kk)
                    blk = dict(nb, Inputs="2")
                    if kk > 2 and str(b.get("OutDataTypeStr", "Inherit: Inherit via internal rule")).startswith("Inherit"):
                        blk["OutDataTypeStr"] = "Inherit: Same as first input"
                    out_blocks[cn] = blk
                    out_lines.append(_line(prev, cn, 1))
                    out_lines.append(_line(elem(s0, sp0, kk), cn, 2))
                    prev = (cn, 1)
                element[n] = (n, 1)
                continue
            if t == "MinMax" and m > 2:
                # the elements of a vector share one type, so a chain of two-input
                # MinMax blocks (each keeping that type) gives the vector's min / max -- the N-input scalar MinMax
                # is not determined on every hardware
                s0, sp0 = ins[0]
                prev = elem(s0, sp0, 1)
                for kk in range(2, m + 1):
                    cn = n if kk == m else "%s\x1dmm%d" % (n, kk)
                    out_blocks[cn] = dict(nb, Inputs="2")
                    if kk < m:
                        # recordings (silent wrong): the links kept the block's output type, so a narrow output (int8)
                        # wrapped the running max before the last element; Simulink takes the max of the vector and
                        # converts once (-1 for [-27 2046 1] / 8 into int8) -- the links keep the elements' type
                        out_blocks[cn]["OutDataTypeStr"] = "Inherit: Same as first input"
                    out_lines.append(_line(prev, cn, 1))
                    out_lines.append(_line(elem(s0, sp0, kk), cn, 2))
                    prev = (cn, 1)
                element[n] = (n, 1)
                continue
            if t == "Sum":
                neg = str(b.get("Inputs", "+")).replace("|", "").strip() == "-"
                nb["Inputs"] = ("-" if neg else "+") * m
                nb["_VecReduce"] = "1"   # a one-input Sum over a vector (fxcore.sum_one_input_unobserved)
            elif t == "BitwiseOperator":
                nb["NumInputPorts"] = str(m)
            else:
                nb["Inputs"] = str(m)
            out_blocks[n] = nb
            element[n] = (n, 1)
            s0, sp0 = ins[0]
            for kk in range(1, m + 1):
                out_lines.append(_line(elem(s0, sp0, kk), n, kk))
            continue
        for k in range(1, w + 1):
            name = n if w == 1 else "%s[%d]" % (n, k)
            nb = {key: v for key, v in b.items() if key != "PortDimensions"}
            if t == "SignalSpecification":
                nb["Dimensions"] = "-1"   # checked against the vector's width above
            mat_ = (n, 1) in shape
            if n in col_gain:   # an m x 1 Matrix(K*u) on a scalar input: each element is K[i] * u
                nb.pop("Multiplication", None)
                # its shape, for the support-domain check of the 'Same as input' parameter (fxgraph_lib
                # _gain_param_type: the exact recordings tuple, rounding and saturation included)
                nb["_ColGain"] = "%dx1" % w
            if n in mkey:
                nb["_MatKey"] = json.dumps(mkey[n], sort_keys=True)
            if t == "Constant" and w > 1:
                nb["Value"] = repr(float(_ints_or_vals(b["Value"], "design.blocks.%s.Value" % n, _CONST_KIND,
                                                       matrix=mat_)[k - 1]))
            if t == "Delay":
                ic = _delay_ic(n, b, w)[k - 1]
                if isinstance(ic, list) and (str(b.get("ExternalReset", "None")) not in ("None", "none") or
                                             str(b.get("ShowEnablePort", "off")) == "on"):
                    # the per-step lowering has no reset / enable port, so the line into port 2
                    # was reported as a malformed request (BAD_CONNECTION); Simulink runs these -- not supported here
                    raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "Delay with one initial value per step and "
                                       "an enable or reset port is not supported here")
                if isinstance(ic, list):
                    nb["_ICSteps"] = ic
                else:
                    nb["InitialCondition"] = repr(float(ic))
            for key in _vecparams(t, b):
                if t != "Constant" and key in b and _vp_len(b[key], "design.blocks.%s.%s" % (n, key)) > 1:
                    nb[key] = repr(float(_ints_or_vals(b[key], "design.blocks.%s.%s" % (n, key), matrix=mat_)[k - 1]))
                    if t == "Gain":
                        # inherited parameter type: best precision over the vector (its elements, colon ranges
                        # expanded)
                        nb["_GainVec"] = "[%s]" % " ".join(
                            repr(v) for v in _ints_or_vals(b[key], "design.blocks.%s.%s" % (n, key), matrix=mat_))
            if w > 1 and k > 1:
                nb["_Tie"] = "%s[1]" % n
                ties[name] = "%s[1]" % n
            out_blocks[name] = nb
            element[name] = (n, k)
            for port, (s, sp) in enumerate(ins, start=1):
                kk = k if wof(s, sp) > 1 else 1
                out_lines.append(_line(elem(s, sp, kk), name, port))
    out_blocks, out_lines = _expand_step_ic(out_blocks, out_lines)
    # a Mux / Selector / Tapped Delay has no block of its own here; its elements are these scalar signals, so its
    # compiled type can be reported when they all share one type (fxgraph _compiled_types)
    struct = {n: [elem(n, 1, k) for k in range(1, width[(n, 1)] + 1)] for n, b in blocks.items()
              if (b.get("type") in ("Mux", "Selector") or b.get("type") in TAPPED) and (n, 1) in width}
    # a Demux carries its input's elements on all its outputs (one element type: a vector signal has one
    # type), so its compiled type -- and every further output port's -- is theirs; a Bus Selector's first output is
    # the selected element (279 / 279 packages with these blocks were INCOMPLETE for want of a type)
    for n, b in blocks.items():
        if b.get("type") == "Demux" and srcs.get(n, {}).get(1) is not None:
            s_, sp_ = srcs[n][1]
            if (s_, sp_) in width:
                struct[n] = [elem(s_, sp_, k) for k in range(1, width[(s_, sp_)] + 1)]
        elif b.get("type") == "BusSelector" and str(b.get("OutputAsBus", "off")) != "on" and (n, 1) in width:
            struct[n] = [elem(n, 1, k) for k in range(1, width[(n, 1)] + 1)]
    widths = {}
    for n, t in vec_user.items():
        names = [elem(n, 1, k) for k in range(1, width[(n, 1)] + 1)] if (n, 1) in width else []
        for j in sorted(srcs.get(n, {})):
            s_, sp_ = srcs[n][j]
            names += [elem(s_, sp_, k) for k in range(1, wof(s_, sp_) + 1)]
        # (block, output port) -- a MATLAB Function's output k >= 2 is not its output 1
        widths[n] = (t, sorted({(x[0], int(x[1])) for x in names}), vec_params[n])   # its parameters
    out = {"blocks": out_blocks, "lines": out_lines}
    vec_rows = []
    for n in bool_rows:
        win = [wof(*srcs[n][j]) for j in sorted(srcs.get(n, {}))]
        if any(w_ > 1 for w_ in win):
            vec_rows += [(x, ",".join(str(w_) for w_ in win))
                         for x in [n] + ["%s[%d]" % (n, k) for k in range(1, max(win) + 1)]]
    if vec_rows:
        out[parameter_bounds.VEC_ROWS_KEY] = vec_rows
    if widths:
        out[parameter_bounds.VEC_KEY] = widths
    # recordings (a previous version; recordings, 8 designs: sfix12_En3 + sfix16_En5, boolean +
    # fixed point): Simulink rejected every Mux of elements with different types (InputPortDataTypeMismatch); the
    # lowering made them separate wires, so the engine answered. The type pass checks these groups (fxgraph_lib
    # same_types)
    # this branch's elem() gives (block, output port); the type pass compares the blocks' first outputs, so an
    # element on a further output port (a multi-output block) is left out of its group
    groups = [{"blocks": sorted({e_[0] for e_ in (elem(n, 1, k) for k in range(1, width[(n, 1)] + 1))
                                 if int(e_[1]) == 1}), "where": "design.blocks." + n}
              for n, b in blocks.items() if b["type"] == "Mux"]
    groups = [g for g in groups if len(g["blocks"]) > 1]
    for n, b in blocks.items():
        # a virtual bus into an Outport -- Simulink requires one data type for all its elements
        # (InputPortCannotAcceptMixedDataTypeVirtualBus); the type pass checks it
        if b["type"] == "Outport" and srcs.get(n, {}).get(1) and layout(*srcs[n][1]):
            bl_ = sorted({e_[0] for e_ in (elem(n, 1, k) for k in range(1, width[(n, 1)] + 1)) if int(e_[1]) == 1})
            if len(bl_) > 1:
                groups.append({"blocks": bl_, "where": "design.blocks." + n, "bus_outport": True})
    if groups:
        out["same_types"] = groups
    if port_types:   # a previous version: a port type constraint follows its block onto the block's elements
        out["port_types"] = [dict(e, block=x) for e in port_types
                             for x in ([e["block"]] if e["block"] in out_blocks else
                                       sorted(k for k in out_blocks if k.startswith(e["block"] + "[")))]
    return out, {"width": {f"{n}:{p}": w for (n, p), w in width.items()},
                                                       "element": element, "ties": ties, "struct": struct}


def _tapped_check(n, b, srcs, lines):
    """recordings: a Tapped Delay's whole parameter set and ports before its rewrite -- the registered parameters
    only, the one recorded sample time (-1, inherited), a numeric scalar initial value, exactly its inputs (1; 2 for
    the enabled one) all connected, and output port 1 only."""
    import block_lowering
    where = "design.blocks." + n
    t = b["type"]
    allowed = set(block_lowering.DEFAULTS[t])
    for k in b:
        if k != "type" and k not in allowed and not k.startswith("_"):
            raise G.GraphError("BAD_PARAM", where + "." + k, "unknown parameter %s for %s" % (k, t))
    st = str(b.get("samptime", "-1")).strip()
    if st != "-1":
        try:
            float(st)
        except ValueError:
            raise G.GraphError("BAD_PARAM", where + ".samptime", "sample time %r is not a number" % st)
        raise G.GraphError("UNSUPPORTED_RATE", where + ".samptime", ('sample time %s (only -1, '
            'inherited, was recorded)') % st)
    vv = str(b.get("vinit", "0.0")).strip()
    if not (t == "TappedDelay" and vv.startswith("[")):   # a vector vinit on the Tapped Delay
        try:
            float(vv)
        except ValueError:
            raise G.GraphError("UNSUPPORTED", where + ".vinit", "initial value %r (a numeric scalar, or a vector on "
                               "the Tapped Delay, was recorded)" % b.get("vinit"))
    nin = 2 if t == "TappedDelayEnabledSynchronous" else 1
    got = sorted(srcs.get(n, {}))
    if got != list(range(1, nin + 1)):
        raise G.GraphError("BAD_CONNECTION", where, "%s takes exactly %d input%s (connected: %s)"
                           % (t, nin, "s" if nin > 1 else "", got))
    for ln in lines:
        if ln.get("src") == n and int(ln.get("sport", 1)) != 1:
            raise G.GraphError("BAD_CONNECTION", where, "output port %s of %s does not exist" % (ln.get("sport"), t))


def _line(src, dst, dport):
    s_, sp = src
    return dict({"src": s_, "dst": dst, "dport": dport}, **({"sport": sp} if sp != 1 else {}))


_MG_TUPLE_SHAPES = None


def _matgain_tuple_shapes():
    """Shapes outside MATGAIN_SHAPES recorded as exact tuples (engine/matgain_domain.json, from
    recordings: 1x5, 2x5, 5x2, 1x6): passed here, answered only for the exact recorded
    tuple (fxgraph_lib checks the tuple; matgain_range.in_range refuses these shapes)."""
    global _MG_TUPLE_SHAPES
    if _MG_TUPLE_SHAPES is None:
        import json as _json
        import os as _os
        p_ = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "matgain_domain.json")
        sh = set()
        if _os.path.exists(p_):
            for t in _json.load(open(p_)):
                m_, n_ = (int(x) for x in str(t[6]).split("x"))
                sh.add((m_, n_))
        r_ = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "matgain_regions.json")
        if _os.path.exists(r_):   # the shapes of the widened regions (matgain_range.in_regions checks the rest)
            for hws in _json.load(open(r_)).values():
                for a in hws.values():
                    for t in a["shape"]:
                        m_, n_ = (int(x) for x in t.split("x"))
                        sh.add((m_, n_))
        _MG_TUPLE_SHAPES = {x for x in sh - MATGAIN_SHAPES if x[1] >= 2}   # m x 1 K: the element-wise path
    return _MG_TUPLE_SHAPES


MATGAIN_SHAPES = {(m, n) for m in range(1, 5) for n in range(2, 5)}   # m 1..4, n 2..4: recordings added 1x3 1x4 2x4 4x2 3x4 4x3   # 
# recordings (2x2 / 3x3, its own type family)


def _ipw_text(n, b):
    """Selector InputPortWidth as an integer text ('3.0' is 3 in Simulink, compared as text it gave
    an untrue width mismatch); a non-integer width is not recorded."""
    t = str(b.get("InputPortWidth", SEL_DEFAULTS["InputPortWidth"])).strip()
    try:
        v = float(t) if t else -1.0
    except ValueError:
        return t
    if v != v or v in (float("inf"), float("-inf")) or v != int(v):
        raise G.GraphError("UNSUPPORTED", "design.blocks.%s.InputPortWidth" % n, "Selector %s: InputPortWidth %r is "
                           "not an integer; how Simulink reads it is not recorded" % (n, t))
    return str(int(v))


LUT_PORT_PARAMS = ("TableSource", "BreakpointsForDimension1Source", "BreakpointsForDimension2Source")


def _lut_port_srcs(b, t="Lookup1D"):
    """A 1-D / 2-D Lookup Table's port-sourced data, in port order after the inputs: 'T' (table), then 'B'
    (breakpoints of dimension 1), then 'C' (dimension 2; 2-D only) -- recordings: u..., T, bp1, bp2."""
    items = [("T", "TableSource"), ("B", "BreakpointsForDimension1Source")]
    if t == "Lookup2D":
        items.append(("C", "BreakpointsForDimension2Source"))
    return [c for c, k in items if str(b.get(k, "Dialog")).strip() == "Input port"]


def _lut_port_widths(n, b, ws, t="Lookup1D", tshape=None):
    """Widths of a Lookup Table with port-sourced data: the inputs scalar, one vector per port-sourced item.
    1-D: table and breakpoints of one length (3 / 5 points), {'T': n, 'B': n}. 2-D: the table
    an m x n matrix signal (column-major elements) or the dialog matrix, breakpoints of m and n,
    {'T': m*n, 'B': m, 'C': n}."""
    if t == "Lookup2D":
        return _lut2_port_widths(n, b, ws, tshape)
    srcs = _lut_port_srcs(b)
    if len(ws) != 1 + len(srcs):
        raise G.GraphError("BAD_CONNECTION", "design.blocks." + n, "1-D Lookup Table %s with %s from input ports has "
                           "%d input ports (u, then %s); %d connected" % (n, " and ".join(
                               {"T": "the table", "B": "the breakpoints"}[c] for c in srcs), 1 + len(srcs),
                               ", ".join({"T": "table", "B": "breakpoints"}[c] for c in srcs), len(ws)))
    if ws[0] != 1:
        raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "1-D Lookup Table %s with ports: a vector input u "
                           "(width %d) is not recorded" % (n, ws[0]))
    out = dict(zip(srcs, ws[1:]))
    for c, k in (("T", "Table"), ("B", "BreakpointsForDimension1")):
        if c not in out:
            out[c] = len(_ints_or_vals(b.get(k, "[]"), "design.blocks.%s.%s" % (n, k), count_only=True))
    if out["T"] != out["B"]:
        raise G.GraphError("TYPE_MISMATCH", "design.blocks." + n, "1-D Lookup Table %s: the table has %d values and "
                           "the breakpoints %d (Simulink reports a dimension mismatch)" % (n, out["T"], out["B"]))
    return out


def _lut2_port_widths(n, b, ws, tshape):
    srcs = _lut_port_srcs(b, "Lookup2D")
    where = "design.blocks." + n
    if len(ws) != 2 + len(srcs):
        raise G.GraphError("BAD_CONNECTION", where, "2-D Lookup Table %s with data from input ports has %d input ports "
                           "(u1, u2, then %s); %d connected" % (n, 2 + len(srcs), ", ".join(
                               {"T": "table", "B": "breakpoints 1", "C": "breakpoints 2"}[c] for c in srcs), len(ws)))
    if ws[0] != 1 or ws[1] != 1:
        raise G.GraphError("UNSUPPORTED", where, "2-D Lookup Table %s with ports: vector inputs are not recorded" % n)
    got = dict(zip(srcs, ws[2:]))
    m_ = got.get("B")
    n_ = got.get("C")
    if m_ is None:
        m_ = len(_ints_or_vals(b.get("BreakpointsForDimension1", "[]"), where + ".BreakpointsForDimension1",
                               count_only=True))
    if n_ is None:
        n_ = len(_ints_or_vals(b.get("BreakpointsForDimension2", "[]"), where + ".BreakpointsForDimension2",
                               count_only=True))
    if "T" in got:
        if tshape is None or tuple(tshape) != (m_, n_):
            raise G.GraphError("TYPE_MISMATCH", where, "2-D Lookup Table %s: the table port must be a %d x %d matrix "
                               "signal (the breakpoints' lengths); it is %s (Simulink reports a dimension mismatch)"
                               % (n, m_, n_, "%d x %d" % tuple(tshape) if tshape else "width %d" % got["T"]))
    else:
        tsh = _pmat(b.get("Table", ""))
        if tsh is not None and tuple(tsh) != (m_, n_):   # an unread form (reshape(...)) is refused later, truly
            raise G.GraphError("TYPE_MISMATCH", where, "2-D Lookup Table %s: the table must be a %d x %d matrix (the "
                               "breakpoints' lengths)" % (n, m_, n_))
    return {"T": m_ * n_, "B": m_, "C": n_}


def _lutport_lower(n, b, ins, wof, elem, out_blocks, out_lines, element, t="Lookup1D", tshape=None):
    """one scalar block with u on port 1 and the elements of each port-sourced vector on the ports after it
    (table first, then breakpoints); _LutPorts tells ref/fxcore.py / fxgraph_lib how to read them each step."""
    ws = [wof(*s) for s in ins]
    wid = _lut_port_widths(n, b, ws, t, tshape)
    srcs = _lut_port_srcs(b, t)
    nu = 2 if t == "Lookup2D" else 1
    nb = {k: v for k, v in b.items() if k not in LUT_PORT_PARAMS and k != "PortDimensions"}
    # 2-D: the dimension sizes too ('M3 N3'), so the kernel can rebuild the table matrix from its column-major elements
    nb["_LutPorts"] = " ".join("%s%d" % (c, wid[c]) for c in srcs) + \
        (" M%d N%d" % (wid["B"], wid["C"]) if t == "Lookup2D" else "")
    out_blocks[n] = nb
    element[n] = (n, 1)
    for j in range(nu):
        s0, sp0 = ins[j]
        out_lines.append(_line(elem(s0, sp0, 1), n, j + 1))
    port = nu + 1
    for j, c in enumerate(srcs, start=nu):
        s_, sp_ = ins[j]
        for kk in range(1, wid[c] + 1):
            out_lines.append(_line(elem(s_, sp_, kk), n, port))
            port += 1


def _matgain_lower(n, b, ins, wof, elem, out_blocks, out_lines, element):
    """Gain Matrix(K*u): row i of the output is the Sum of the n element products K[i][j] * u[j], each an element Gain
    with the whole matrix's inherited parameter type (_GainVec) and the internal-rule (exact) product; the Sum takes
    the block's output type (internal rule: the product type, 'Inherit: Same as first input') with Floor and wrap --
    recordings (2x2 / 2x3 / 3x2, sfix8_En4 / sfix12_En8 / int8, parameter internal rule
    or fixdt(1,8,5), output internal rule or fixdt(1,24,10), host): 45,056 element-steps equal. With an explicit output
    type each product is first rounded into it."""
    import blockspec as _bs
    known = set(_bs.LIB_DIALOG_PARAMS.get("Gain", ())) | {"type", "_Tie"}
    bad = sorted(k for k in b if k not in known and not k.startswith("_"))
    if bad:
        # the lowering rebuilds the block, so a parameter Gain does not have (Simulink
        # set_param: ParamUnknown) was dropped and the design answered
        raise G.GraphError("BAD_PARAM", "design.blocks.%s.%s" % (n, bad[0]), "Gain has no parameter %s (Simulink "
                           "rejects it)" % bad[0])
    honoured = {"type", "Gain", "Multiplication", "OutDataTypeStr", "ParamDataTypeStr", "RndMeth",
                "SaturateOnIntegerOverflow", "OutMin", "OutMax", "ParamMin", "ParamMax", "LockScale"}
    rng_ = sorted(k for k in ("OutMin", "OutMax", "ParamMin", "ParamMax") if str(b.get(k, "[]")).strip() not in ("[]", ""))
    if rng_:
        # the lowering rebuilt the block and dropped the design range silently (Simulink uses it)
        raise G.GraphError("UNSUPPORTED", "design.blocks.%s.%s" % (n, rng_[0]), "design range %s = %r is not "
                           "supported: Simulink uses design minimum / maximum values to choose types and to check "
                           "parameters, which this engine does not model; leave it '[]'" % (rng_[0], b[rng_[0]]))
    other = sorted(k for k in b if k not in honoured and not k.startswith("_") and
                   not (k == "SampleTime" and str(b[k]).strip() in ("-1", "")))
    if other:
        raise G.GraphError("UNSUPPORTED", "design.blocks.%s.%s" % (n, other[0]), "Gain Matrix(K*u) with %s %r is not "
                           "recorded" % (other[0], b[other[0]]))
    ot_ = str(b.get("OutDataTypeStr", "Inherit: Inherit via internal rule")).strip()
    # recordings (672 models run, every step equal): 'Inherit: Same as input' -- every product rounded into the
    # input's type (the Gains' own rule), the row accumulated in it like an explicit output type
    if ot_.startswith("Inherit") and ot_ not in ("Inherit: Inherit via internal rule", "Inherit: Same as input"):
        # a valid inherit rule the recording does not cover (was BAD_TYPE / request)
        raise G.GraphError("UNSUPPORTED", "design.blocks.%s.OutDataTypeStr" % n, "Gain Matrix(K*u) with output type "
                           "%r is not recorded (internal rule or an explicit fixdt)" % ot_)
    K = _expand(_checked(b.get("Gain", "1"), "design.blocks.%s.Gain" % n), "design.blocks.%s.Gain" % n)
    mm, nn = len(K), len(K[0])
    allk = "[%s]" % " ".join(v for r in K for v in r)
    s0, sp0 = ins[0]
    ot = str(b.get("OutDataTypeStr", "Inherit: Inherit via internal rule")).strip()
    for i in range(mm):
        row = n if mm == 1 else "%s[%d]" % (n, i + 1)
        out_blocks[row] = {"type": "Sum", "Inputs": "+" * nn, "_MatGain": n,
                           # the user block's own settings, for the support-domain check (fxgraph_lib MATGAIN_DOMAIN)
                           "_MatGainKey": json.dumps([b.get("ParamDataTypeStr", "Inherit: Inherit via internal rule"),
                                                      ot, "%dx%d" % (mm, nn)]),
                           # 'Same as input': every product rounded into the input
                           # type, the row accumulated in it -- as an explicit output type that is the input's
                           "OutDataTypeStr": "Inherit: Same as first input" if ot in (
                               "Inherit: Inherit via internal rule", "Inherit: Same as input") else ot,
                           "_MatGainOut": ot, "RndMeth": b.get("RndMeth", "Floor"),
                           "SaturateOnIntegerOverflow": b.get("SaturateOnIntegerOverflow", "off")}
        element[row] = (n, i + 1)
        for j in range(nn):
            g = "%s%s/g%d_%d" % (INTERNAL, n, i + 1, j + 1)
            out_blocks[g] = {"type": "Gain", "Gain": K[i][j], "_GainVec": allk, "_MatGain": n}
            if ot != "Inherit: Inherit via internal rule":
                # recordings (silent wrong; recordings rechecked natively): with an explicit output type Simulink
                # rounds every product into the output type first, then adds (the Sum of same-typed inputs is exact)
                out_blocks[g].update(OutDataTypeStr=ot, RndMeth=b.get("RndMeth", "Floor"),
                                     SaturateOnIntegerOverflow=b.get("SaturateOnIntegerOverflow", "off"))
            if "ParamDataTypeStr" in b:
                out_blocks[g]["ParamDataTypeStr"] = b["ParamDataTypeStr"]
            out_lines.append(_line(elem(s0, sp0, j + 1), g, 1))   # (recorded: elem gives a (block, port) pair)
            if nn == 2:
                out_lines.append({"src": g, "dst": row, "dport": j + 1})
        if nn > 2:
            # (also with the internal rule: recordings, saturation on --
            # 0.75u + 1.5u saturates before -0.875u is added; the earlier internal-rule recordings wrapped, where one
            # n-input Sum and the chain agree)
            # recordings (286,720 element-steps): the row accumulates in the output type,
            # wrapping or saturating at every addition -- a chain of two-input Sums (one n-input Sum saturates once)
            gs_ = ["%s%s/g%d_%d" % (INTERNAL, n, i + 1, j + 1) for j in range(nn)]
            prev = gs_[0]
            for j in range(1, nn):
                dst = row if j == nn - 1 else "%s%s/s%d_%d" % (INTERNAL, n, i + 1, j)
                if dst != row:
                    out_blocks[dst] = dict(out_blocks[row])
                    out_blocks[dst]["Inputs"] = "++"
                out_lines.append({"src": prev, "dst": dst, "dport": 1})
                out_lines.append({"src": gs_[j], "dst": dst, "dport": 2})
                prev = dst
            out_blocks[row]["Inputs"] = "++"


def _expand_step_ic(blocks, lines):
    """A Delay(N) with one initial value per step becomes N Unit Delays: src -> d1 -> ... -> d(N-1) -> name, where
    the output (name) starts at step 0's value and stage d_j at step (N - j)'s value."""
    out_b, out_l = {}, list(lines)
    for n, b in blocks.items():
        if b.get("type") != "Delay" or "_ICSteps" not in b:
            out_b[n] = b
            continue
        # the lowering kept only the initial values -- a frame-based Delay
        # (Simulink: ICFrameLengthNotMatchMaxDelayLengthInFrameMode) was answered; every other parameter must be
        # one the lowering honours
        ip = str(b.get("InputProcessing", "Elements as channels (sample based)")).strip()
        if ip not in ("Elements as channels (sample based)", "Inherited"):
            raise G.GraphError("UNSUPPORTED", "design.blocks.%s.InputProcessing" % n, "Delay %s with InputProcessing "
                               "%r is not supported (frame-based processing is not modelled); use 'Elements as "
                               "channels (sample based)'" % (n, ip))
        # (a parameter at its library default is not extra: a model read back writes ExternalReset 'None',
        # ShowEnablePort 'off', the Dialog sources)
        dflt_ = {"ExternalReset": ("none", ""), "ShowEnablePort": ("off",), "DelayLengthSource": ("dialog",),
                 "InitialConditionSource": ("dialog",), "PreventDirectFeedthrough": ("off",)}
        extra = sorted(k for k in b if k not in ("type", "DelayLength", "InitialCondition", "InputProcessing",
                                                 "_ICSteps", "_Tie", "SampleTime") and
                       str(b[k]).strip().lower() not in dflt_.get(k, ()))
        if extra or str(b.get("SampleTime", "-1")).strip() not in ("-1", ""):
            raise G.GraphError("UNSUPPORTED", "design.blocks.%s" % n, "Delay %s with one initial value per step: %s "
                               "is not supported with this form; give a scalar initial condition"
                               % (n, ", ".join(extra or ["SampleTime"])))
        ic = b["_ICSteps"]
        N = len(ic)
        # the expansion below keeps none of the Delay's other parameters, so
        # their checks never ran (a frame-based Delay was answered; Simulink refuses that IC there,
        # ICFrameLengthNotMatchMaxDelayLengthInFrameMode); check them on the user's block first
        G._check_params("Delay", {k: v for k, v in b.items() if not k.startswith("_")}, "design.blocks." + n)
        if str(b.get("ShowEnablePort", "off")).strip() == "on" or str(b.get("ExternalReset", "None")).strip() not in \
                ("None", "none", ""):
            # Simulink runs a per-step initial condition with an enable / reset port; the
            # per-step expansion has no ports (it was refused as a connection error)
            raise G.GraphError("UNSUPPORTED", "design.blocks.%s.InitialCondition" % n, "a per-step initial condition on "
                               "a Delay with an enable or reset port: not supported; give one initial value")
        stages = ["%s%s/d%d" % (INTERNAL, n, j) for j in range(1, N)] + [n]
        for j, x in enumerate(stages, start=1):
            out_b[x] = {"type": "UnitDelay", "InitialCondition": repr(float(ic[N - j]))}
            if "SampleTime" in b:   # (was dropped: a Delay with SampleTime 2 ran every step)
                out_b[x]["SampleTime"] = b["SampleTime"]
            if "_Tie" in b:
                out_b[x]["_Tie"] = b["_Tie"] if x == n else "%s%s/d%d" % (INTERNAL, b["_Tie"], j)
        out_l = [dict(ln, dst=stages[0]) if ln["dst"] == n else ln for ln in out_l]
        out_l += [{"src": stages[j], "dst": stages[j + 1], "dport": 1} for j in range(N - 1)]
    return out_b, out_l


def _vp_len(text, where):
    """The element count of a vector parameter. A text that is not a [...] list is one value, read by the block
    itself (a scalar BitMask "bin2dec('1')" / "hex2dec('58')" is the block's own mask syntax; for example,
    recordings, 9 designs Simulink runs, were refused once the vector pass parsed the mask)."""
    if not str(text).strip().startswith("["):
        return 1
    return len(_ints_or_vals(text, where))


def _vecparams(t, b):
    """The vector parameters of block b: a Bitwise Operator's BitMask only when the mask is used (its library
    default bin2dec('11011001') is not read otherwise -- recordings)."""
    ks = VECPARAM.get(t, ())
    if t == "BitwiseOperator" and str(b.get("UseBitMask", "on")).strip() != "on":
        return ()
    return ks


def _ints_or_vals(text, where="design", kind="any", count_only=False, matrix=False):
    # count_only (a previous version) / matrix (main recordings): the engine refuses matrix values unless only the element count is wanted; recorded reads
    # them column by column either way, so the flags change nothing here
    """Element texts of a vector parameter. kind: the parameter's declared kind (blockspec.NUMERIC_VALUES) -- recordings: the Constant's Value is 'typed_any' (fi(...) is read by the block), which 'any' refused as UNSUPPORTED
    in any design with a vector signal."""
    t = _checked(text, where, kind)
    if not t.startswith("["):
        return [t]
    rows = _expand(t, where)
    if len(rows) > 1 and len(rows[0]) > 1:
        # Matrix constants use column-major order.
        # Preserve that order when pairing each constant with an input element.
        if any(len(r) != len(rows[0]) for r in rows):
            raise G.GraphError("BAD_PARAM", where, "matrix rows of different lengths")
        return [float(rows[i][j]) for j in range(len(rows[0])) for i in range(len(rows))]
    return [float(v) for r in rows for v in r]


def _declared_ports(b, t):
    """Input ports a Sum / Product declares (Inputs as a count or a sign / operator string; left out: the library
    block's '|++' / '2'); None for other blocks or a spelling not read here (checked elsewhere)."""
    if t not in ("Sum", "Product"):
        return None
    v = str(b.get("Inputs", "|++" if t == "Sum" else "2")).strip()
    if v.isdigit():
        return int(v)
    ops = v.replace("|", "")
    chars = "+-" if t == "Sum" else "*/"
    if ops and all(c in chars for c in ops):
        return len(ops)
    return None


def _one_input(b, t):
    """True when a Sum / MinMax block has a single input port with the recorded reduction (Sum "+" or Inputs = 1;
    a one-input "-" is refused by the caller). Called only for
    blocks with exactly one connected input, so a missing Inputs (a model saved with Simulink's default of one
    port) also means one input."""
    if t == "BitwiseOperator":
        # recordings (16 designs): one input, no mask, AND / OR / XOR / NAND / NOR over a
        # vector reduce its elements to one value (with a mask: element-wise)
        return str(b.get("NumInputPorts", "1")).strip() == "1" and str(b.get("UseBitMask", "on")).strip() == "off" \
            and str(b.get("logicop", "AND")).strip() != "NOT"
    if t == "LogicalOperator":
        # recordings (32 designs): AND / OR / NAND / NOR / XOR / NXOR with one input over a vector
        # reduce its elements to one boolean (NOT stays element-wise)
        return str(b.get("Operator", "AND")).strip() != "NOT" and str(b.get("Inputs", "")).strip() == "1"
    if "Inputs" not in b:
        # the library Sum is '|++' and the library Product '2' (two ports; an unconnected
        # second port is grounded) -- only MinMax's library block has one input
        return t == "MinMax"
    v = str(b["Inputs"]).strip()
    if t == "Sum":
        v = v.replace("|", "")
        return v in ("+", "1", "-")   # recordings: '-' reduces too (negated sum)
    if t == "LogicalOperator":
        # NOT is element-wise on a vector (one output per element), never a reduction -- with Inputs '1'
        # written out it was rewired as one and refused ("G has no input port 2")
        return v == "1" and str(b.get("Operator", "AND")).strip() != "NOT"
    return v == "1"

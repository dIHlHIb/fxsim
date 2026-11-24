"""HDL library blocks by lowering into engine blocks that every path already supports.

HDL Counter and the three synchronous delays (Unit Delay Enabled / Enabled Resettable / Resettable Synchronous) are
rewritten, before compilation, into Constant, Sum, Relational Operator, Switch, Data Type Conversion, Unit Delay and
the internal Hold block. The rewritten design runs on the Python path, the C core, fxcodegen and Verilog unchanged,
and the user's block name stays the block's (first) output. Rules (every one from
recordings):

  synchronous delays: y = state; next state = R ? IC: (E ? u: state) -- reset wins even when not enabled; the
      output type is the input type; top level and inside a subsystem behave alike.
  HDL Counter (output type fixdt from CountDataType / CountWordLen / CountFracLen; c = the count, starts at CountInit):
      priority rst > load > enb; rst: c = init; load: c = load_val; enb (or no enable port):
      Free running   c = wrap(c +- step)
      Count limited  up: c = init if c == max else wrap(c + step); down: c = init if c == max
                     else wrap(c - step)
      Modulo         up: n = c + step, n - (max + 1) if n > max; down: n = c - step, n + (max + 1) if n < 0; wrap
      the direction port: true = up. count_hit (second output) = (c == max), observed for Count limited only.

Anything not observed is refused with UNSUPPORTED (not guessed): see _refuse below and Design-side checks in
post_compile_checks (control inputs must be boolean, load_val must have the count's type, initial conditions must lie
on the input type's grid)."""
from fractions import Fraction

import os
import re
import fxgraph_lib as G

INTERNAL = "\x1d"
SYNC = ("UnitDelayEnabledSynchronous", "UnitDelayEnabledResettableSynchronous", "UnitDelayResettableSynchronous")
# lowered the same way
LAB23 = ("DirectLookupND", "Repeat", "DataTypeDuplicate")   # DetectRisePositive: the engine's core lowering
TYPES = SYNC + ("HDLCounter",) + LAB23 + ("HDLRAM", "MATLABFunction", "HDLFIRFilter", "HDLFIRDecimator", "HDLFIFO")
RAM_PORTS = {"Simple dual port": ["wr_din", "wr_addr", "wr_en", "rd_addr"],
             "Dual port": ["wr_din", "wr_addr", "wr_en", "rd_addr"], "Single port": ["din", "addr", "we"]}
# also recorded 10 and 16 address bits (sfix16_En8 data); the lowering here builds blocks per address
# (10 bits overflows the Multiport Switch, 16 bits the block limit), so they stay refused until a native kernel
RAM_ADDR_BITS = tuple(range(2, 17))   # recordings (2..4), recordings (5, 8), recordings (10, 16), recordings (12),
# recordings (6, 7, 9, 11, 13, 14, 15; Simulink refuses 1: hdlsllib:hdlsllib:addrportwidth, 'between 2 and 29')
# 10 / 16 address bits: a native memory core (RAMCore) instead of blocks per address; recorded with sfix16_En8 data
# and initial value 0 only
RAM_DEEP_ADDR = (9, 10, 11, 12, 13, 14, 15, 16)   # 9 .. 15 on the same core
# the deep settings recorded: (address bits, data type or 'boolean') with initial value 0;
# a non-zero initial value: (10, sfix16_En8, Simple dual port) only
RAM_DEEP_DATA = {(10, (1, 16, 8)), (16, (1, 16, 8)), (12, (1, 16, 8)), (10, (1, 32, 0)), (10, (0, 8, 0)),
                 (10, "boolean"),
                 # sfix16_En8 on 9, 11, 13, 14, 15 bits; ufix8 on 11 / 14, boolean on
                 # 9 / 13 / 15 (Simple / Dual / Single port alike)
                 (9, (1, 16, 8)), (11, (1, 16, 8)), (13, (1, 16, 8)), (14, (1, 16, 8)), (15, (1, 16, 8)),
                 (11, (0, 8, 0)), (14, (0, 8, 0)), (9, "boolean"), (13, "boolean"), (15, "boolean"),
                 # recordings (recordings probes): int16 on 10, uint16 on 9, int8 on 12 (Dual port),
                 # sfix16_En15 on 11 bits
                 (10, (1, 16, 0)), (9, (0, 16, 0)), (12, (1, 8, 0)), (11, (1, 16, 15))}
RAM_DEEP_INIT = {(10, (1, 16, 8), "Simple dual port")}
RAM_DEEP_OLD = {(9, "Single port"), (13, "Single port"), (10, "Single port")}   # (10: recordings)   # WriteOutputValue Old data
RAM_DATA_BITS = (3, 8, 12, 16, 18, 24, 32)   # recordings (8, 16), recordings (12, 18, 24; boolean data too), recordings;
# 3: recordings (ufix3 data, 4 address bits)
# show the new widths in pairs only: 32-bit data (these types) with 8 address bits, and
# sfix16_En8 data with 10 / 16 address bits
RAM_DATA32_TYPES = {(1, 32, 0), (1, 32, 16), (0, 32, 0)}
RAM_DEEP_ADDR_DATA = (1, 16, 8)
RAM_DEFAULTS = {"RAMType": "Simple dual port", "AddressBits": "", "WriteOutputValue": "New data",
                "RAMInitialValue": "0"}
COUNTER_PORTS = (("CountResetPort", "rst"), ("CountLoadPort", "load"), ("CountLoadPort", "load_val"),
                 ("CountEnbPort", "enb"), ("CountDirPort", "dir"))

COUNTER_DEFAULTS = {"CountType": "Count limited", "CountInit": "0", "CountStep": "1", "CountMax": "25",
                    "CountFromType": "Initial value", "CountFrom": "0", "CountResetPort": "off",
                    "CountLoadPort": "off", "CountEnbPort": "off", "CountDirPort": "off", "CountHitOutputPort": "off",
                    "CountDataType": "Unsigned", "CountWordLen": "8", "CountFracLen": "0", "CountSampTime": "1"}
SYNC_DEFAULTS = {"InitialCondition": "0.0", "SampleTime": "-1"}

# the core Delay with its enable port and / or external reset, lowered here; a Delay
# without them stays the engine's own block. Port order u, enable, reset (the recorded PortConnectivity).
PDELAY_DEFAULTS = {"DelayLength": None, "InitialCondition": "0", "DelayLengthSource": "Dialog",
                   "InitialConditionSource": "Dialog", "ExternalReset": "None", "ShowEnablePort": "off",
                   "SampleTime": "-1"}
PDELAY_RESETS = ("None", "Rising", "Falling", "Either", "Level", "Level hold")
# the recorded port combinations (ShowEnablePort, ExternalReset); the others are refused
PDELAY_RECORDED = {("on", "None"), ("off", "Rising"), ("off", "Falling"), ("off", "Either"), ("off", "Level"),
                   ("off", "Level hold"), ("on", "Level"),
                   ("on", "Rising")}   # recordings (DelayLength 2 / 4 / 8)
PDELAY_MAX = 64
PDELAY_LENGTHS = (1, 2, 3, 4, 8)   # recordings (1, 3) and recordings (2, 4, 8): every recorded port combination
# recordings (the length sweep; 81 models equal to the engine's rule): the enable port alone, the enable
# port with a Level reset and a Rising reset alone recorded on 18 lengths spread over 5 .. 64 (with 1 .. 4, 8 above):
# lengths 1 .. PDELAY_MAX open for these three. The other recorded combinations (Falling / Either / Level / Level
# hold alone, the enable port with a Rising reset) only on the lengths recorded for them (recordings 16 / 32, the engine's
# recordings 16, recordings 6 / 16 / 24 / 32 / 64).
PDELAY_SWEPT = {("on", "None"), ("on", "Level"), ("off", "Rising")}
PDELAY_SPARSE_LENGTHS = (6, 16, 24, 32, 64)
# recordings (every step equal on the lowering): the enable port with a Falling / Either / Level
# hold reset at lengths 2 / 5 / 33 (only those), and lengths 5 / 33 for the other recorded combinations
PDELAY_A26_NEW = {("on", "Falling"), ("on", "Either"), ("on", "Level hold")}
PDELAY_A26_LENGTHS = {c: (2, 5, 33) for c in PDELAY_A26_NEW}
PDELAY_A26_LENGTHS.update({c: (5, 33) for c in (("off", "Falling"), ("off", "Either"), ("off", "Level"),
                                                ("off", "Level hold"), ("on", "Rising"))})
PDELAY_RECORDED = PDELAY_RECORDED | PDELAY_A26_NEW
# the shapes recorded on ASIC/FPGA and inside a feedback loop (
# the enable with a Falling / Either / Level hold reset); the others are refused
# there (post_compile_checks) -- found by the a previous version merge (the engine's the regression check)
PDELAY_ASIC = {("on", "None"), ("on", "Level"), ("on", "Rising")}
PDELAY_LOOP = {("off", "Level"), ("on", "Rising"), ("on", "Falling"), ("on", "Either"), ("on", "Level hold")}


def port_delay(b):
    """A Delay block with its enable port or external reset shown (lowered by this module)."""
    return isinstance(b, dict) and b.get("type") == "Delay" and (
        str(b.get("ShowEnablePort", "off")).strip() != "off" or str(b.get("ExternalReset", "None")).strip() != "None")


def _lowered(b):
    return isinstance(b, dict) and (b.get("type") in TYPES or port_delay(b))


ES_TYPES = ("EnablePort", "EnabledOutport", "StateControl")
CPLX_TYPES = ("RealImagToComplex", "ComplexToRealImag")
CORDIC_TYPES = ("ComplexToMagnitudeAngle", "MagnitudeAngleToComplex")   # recordings (engine/fxcordic.py), via _complex
# Complex to Real-Imag Output -> the parts its output ports carry (0 real, 1 imaginary)
C2RI_PARTS = {"Real and imag": (0, 1), "Real": (0,), "Imag": (1,)}


def _c2ri_parts(b):
    return C2RI_PARTS.get(str(b.get("Output", "Real and imag")), (0, 1))


def has_hdl(spec):
    import parameter_bounds as _lb
    if isinstance(spec, dict) and spec.get(_lb.HELPERS_KEY):
        return True   # a design the type-dependent recorded rewrites lowered (its helpers' owners)
    if isinstance(spec, dict) and isinstance(spec.get("blocks"), dict) and any(
            isinstance(b, dict) and b.get("type") in ("HDLNCO", "BitConcat", "ComplexToMagnitudeAngle", "MagnitudeAngleToComplex") for b in spec["blocks"].values()):
        return True
    if isinstance(spec, dict) and isinstance(spec.get("blocks"), dict):
        import block_lowering
        import fxaction
        import fxprelookup
        import fxiir
        import fxzoh
        import fxtrigger
        import fxlattice
        if block_lowering.has(spec) or fxaction.has(spec) or fxprelookup.has(spec) or fxiir.has(spec) or fxzoh.has(spec) \
                or fxtrigger.has(spec) or fxlattice.has(spec):
            return True
        if any(isinstance(b, dict) and b.get("type") == "Constant" and isinstance(b.get("Value"), str)
               and _cplx_value(b["Value"]) is not None for b in spec["blocks"].values()):
            return True   # complex Constant (lowered by _cplx_const)
    return isinstance(spec, dict) and isinstance(spec.get("blocks"), dict) and \
        any(isinstance(b, dict) and (b.get("type") in TYPES + ES_TYPES + CPLX_TYPES or port_delay(b))
            for b in spec["blocks"].values())


def _refuse(where, msg, code="UNSUPPORTED"):
    raise G.GraphError(code, where, msg)


def _repointed(spec, pairs):
    """a lowering that moves a line off output port k of a block records where that port went
    (parameter_bounds.PORT_KEY: "block|port" -> [block, port]), so the recorded-width check finds the compiled driver."""
    import parameter_bounds
    m = dict(spec.get(parameter_bounds.PORT_KEY) or {})
    for (s_, k), (t_, kp) in pairs:
        m["%s|%d" % (s_, k)] = [t_, kp]
    return m


def _pint(v, where):
    """An integer block parameter: an ASCII integer literal through fxnum, else BAD_PARAM."""
    import parameter_bounds
    return parameter_bounds.pint(v, where)


def _num(v, where):
    # through fxnum first (Fraction() took '14/2' and '1_0'); Inf / NaN are not numbers here
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


def n_inputs(b, where="design.blocks"):
    t = b["type"]
    if t == "MATLABFunction":
        return ml_inputs(b, where)
    if t in ("HDLFIRFilter", "HDLFIRDecimator"):
        # the reset input port is the third
        return ["data", "valid"] + (["reset"] if str(b.get("ResetInputPort", "off")).strip() == "on" else [])
    if t == "HDLFIFO":
        import fxhdlfifo
        return list(fxhdlfifo.INPUTS)
    if t == "HDLRAM":
        rt = str(b.get("RAMType", RAM_DEFAULTS["RAMType"]))
        if rt not in RAM_PORTS:
            _refuse("design.blocks", "RAMType %r (Simple dual port, Dual port, Single port)" % rt, "BAD_PARAM")
        return RAM_PORTS[rt]
    if t in ("DetectRisePositive", "Repeat"):
        return ["u"]
    if t == "DirectLookupND":
        try:
            nd = _pint(b.get("NumberOfTableDimensions", "1"), where + ".NumberOfTableDimensions")
        except ValueError:
            nd = 0
        # Simulink's block has one index port per table dimension the selection does not cover --
        # Element all of them, Column one fewer, 2-D Matrix two fewer (a 2-D Column table: 1 port; 3-D Element: 3);
        # counted that way, the block's own checks then give the true reason (only Element / 1-2 dimensions answered)
        sel = str(b.get("InputsSelectThisObjectFromTable", "Element")).strip()
        nports = nd - {"Column": 1, "2-D Matrix": 2}.get(sel, 0)
        return ["idx%d" % k for k in range(1, nports + 1)] if 1 <= nports <= 30 else ["idx1"]
    if t == "DataTypeDuplicate":
        try:
            n = _pint(b.get("NumInputPorts", "2"), where + ".NumInputPorts")
        except ValueError:
            n = 0
        if not 2 <= n <= 1024:
            _refuse("design.blocks", "Data Type Duplicate needs 2..1024 inputs", "BAD_PARAM")
        return ["in%d" % k for k in range(1, n + 1)]
    if t == "HDLCounter":
        p = dict(COUNTER_DEFAULTS, **{k: str(v) for k, v in b.items() if k != "type"})
        return [nm for flag, nm in COUNTER_PORTS if p.get(flag) == "on"]
    if port_delay(b):
        # Simulink's order u, d, enable, reset, x0 (a length / initial condition port is counted, then
        # refused by port_delay_params as not recorded rather than as a missing port)
        return ["u"] + (["d"] if str(b.get("DelayLengthSource", "Dialog")).strip() == "Input port" else []) + \
            (["E"] if str(b.get("ShowEnablePort", "off")).strip() == "on" else []) + \
            (["R"] if str(b.get("ExternalReset", "None")).strip() != "None" else []) + \
            (["x0"] if str(b.get("InitialConditionSource", "Dialog")).strip() == "Input port" else [])
    return {"UnitDelayEnabledSynchronous": ["u", "E"], "UnitDelayEnabledResettableSynchronous": ["u", "E", "R"],
            "UnitDelayResettableSynchronous": ["u", "R"]}[t]


# a parameter a JSON design leaves out of a the extended blocks
# is the Simulink library block's default -- the HDL NCO dithers and takes its increment from a port, the Direct
# Lookup Table (n-D) is two-dimensional -- not the engine's table value (the package passed with the engine's value
# written, while the design meant something else). A saved .slx carries these values itself (InstanceData / the
# reader's built-in defaults); internal blocks of the lowerings keep the engine's.
LIBRARY_DEFAULTS = {t: G.JSON_LIBRARY_DEFAULTS[t] for t in ("HDLNCO", "DirectLookupND")}   # engine/blockspec.py


def with_library_defaults(spec):
    """spec with every left-out LIBRARY_DEFAULTS parameter of a user block written out (a copy when any)."""
    if not isinstance(spec, dict) or not isinstance(spec.get("blocks"), dict):
        return spec
    blocks = None
    for n, b in spec["blocks"].items():
        lib = LIBRARY_DEFAULTS.get(b.get("type")) if isinstance(b, dict) else None
        if not lib or any(ch in n for ch in G.RESERVED_CHARS) or all(k in b for k in lib):
            continue
        blocks = blocks or dict(spec["blocks"])
        blocks[n] = dict({k: v for k, v in lib.items() if k not in b}, **b)
    return spec if blocks is None else dict(spec, blocks=blocks)


BP_TEXT = "Inherit: Inherit via back propagation"


def bp_from_declared(spec):
    """recordings: a back-propagated source (a Constant or Data Type
    Conversion with OutDataTypeStr 'Inherit: Inherit via back propagation') driving a Signal Specification with an
    explicit type, or a Data Type Duplicate whose other input comes straight from a block with an explicit type, takes
    that type in Simulink (15 / 15 audited cases accepted, types and values as the native runs show). Written into the
    design before compiling; a source two such blocks would give different types is left alone (the type check then
    refuses it as before). Returns spec (a copy when changed)."""
    if not isinstance(spec, dict) or not isinstance(spec.get("blocks"), dict):
        return spec
    blocks, lines = spec["blocks"], spec.get("lines", [])

    def bp(n):
        b = blocks.get(n)
        return isinstance(b, dict) and b.get("type") in ("Constant", "DataTypeConversion") and \
            str(b.get("OutDataTypeStr", "")).strip() == BP_TEXT

    def explicit(n):
        b = blocks.get(n)
        if not isinstance(b, dict) or b.get("type") not in ("Constant", "DataTypeConversion", "SignalSpecification"):
            return None
        t = str(b.get("OutDataTypeStr", "")).strip()
        return None if not t or t.startswith("Inherit") or t == "auto" else t
    want = {}
    for n, b in blocks.items():
        if not isinstance(b, dict):
            continue
        ins = [ln for ln in lines if ln.get("dst") == n]
        if b.get("type") == "SignalSpecification":
            t = explicit(n)
            for ln in ins:
                if t and bp(ln["src"]) and int(ln.get("sport", 1)) == 1:
                    want.setdefault(ln["src"], set()).add(t)
        elif b.get("type") == "DataTypeDuplicate":
            typed = {explicit(ln["src"]) for ln in ins if not bp(ln["src"])}
            typed.discard(None)
            if len(typed) == 1:
                for ln in ins:
                    if bp(ln["src"]) and int(ln.get("sport", 1)) == 1:
                        want.setdefault(ln["src"], set()).add(next(iter(typed)))
    new = {n: next(iter(ts)) for n, ts in want.items() if len(ts) == 1}
    if not new:
        return spec
    return dict(spec, blocks=dict(blocks, **{n: dict(blocks[n], OutDataTypeStr=t) for n, t in new.items()}))


def _nco_cplx_direct(spec):
    """The HDL NCOs of spec whose Complex exponential output goes only to Outports."""
    bl = spec.get("blocks") or {} if isinstance(spec, dict) else {}
    return {n_ for n_, b_ in bl.items() if isinstance(b_, dict) and b_.get("type") == "HDLNCO" and
            str(b_.get("Waveform", "")).strip() == "Complex exponential" and
            all(isinstance(bl.get(l_.get("dst")), dict) and bl[l_["dst"]].get("type") == "Outport"
                for l_ in spec.get("lines") or [] if isinstance(l_, dict) and l_.get("src") == n_ and
                str(l_.get("sport", 1)).strip() == "1")}


def lower(spec):
    """(lowered spec, info). info: {"bool_ctrl": [(block, port name, source)], "same_type": [(block, source, type)],
    "ic_grid": [(block, value, source)], "blocks": {user block: [internal names]}}."""
    import fxvector
    fxvector.named_port_check(spec)   # the recorded lowerings read ports as numbers
    for n_, b_ in (spec.get("blocks") or {}).items():
        if not isinstance(b_, dict):
            continue
        if b_.get("type") == "Math" and str(b_.get("OutDataTypeStr", "")).strip() == "boolean":
            # Simulink rejects a boolean output type at set_param
            raise G.GraphError("BAD_PARAM", "design.blocks.%s.OutDataTypeStr" % n_, "Math OutDataTypeStr 'boolean' "
                               "is rejected by Simulink at set_param; leave the output type inherited")
        if b_.get("type") == "Math" and re.match(r"^fixdt\(\s*[01]\s*,\s*\d+\s*\)$", str(b_.get("OutDataTypeStr", "")).strip()):
            # best-precision fixdt(s, w) without OutMin / OutMax stops Simulink at compile
            # (Simulink:DataType:SpecifyParamsForBestPrecisionMode); it was answered as fixdt(s, w, input fraction)
            raise G.GraphError("BAD_PARAM", "design.blocks.%s.OutDataTypeStr" % n_, "Math OutDataTypeStr %r "
                               "(best precision) without output limits: Simulink stops at compile "
                               "(SpecifyParamsForBestPrecisionMode); give the fraction length" % b_["OutDataTypeStr"])
        if b_.get("type") in CPLX_TYPES and str(b_.get("SampleTime", "-1")).strip() not in ("-1", "", "inf", "Inf"):
            # recordings (silent wrong): the block's own sample time was ignored -- Simulink samples and holds at it
            raise G.GraphError("UNSUPPORTED", "design.blocks.%s.SampleTime" % n_, "%s SampleTime %s: the block's "
                               "own sample time (sample and hold) is not modelled; leave it -1 (inherited) and set the "
                               "rate upstream" % (b_["type"], b_.get("SampleTime")))
        if b_.get("type") in CPLX_TYPES and str(b_.get("SampleTime", "-1")).strip().lower() == "inf":
            # Simulink rejects SampleTime inf on these blocks at set_param
            raise G.GraphError("BAD_PARAM", "design.blocks.%s.SampleTime" % n_, "%s SampleTime inf is rejected by "
                               "Simulink at set_param; use -1 (inherited)" % b_["type"])
    cinfo = None
    # the user's blocks and the internal names before any recorded pass -- the cost
    # counts the user's blocks, the helpers a recorded lowering adds are claimed by their owner (cost_owner)
    import parameter_bounds as _lb
    prior = spec.get(_lb.HELPERS_KEY) or {}
    user0 = dict(prior.get("user") or {})
    user0.update({n_: b_.get("type") for n_, b_ in (spec.get("blocks") or {}).items()
                  if isinstance(b_, dict) and INTERNAL not in n_ and n_ not in user0})
    internal0 = {n_ for n_ in (spec.get("blocks") or {}) if INTERNAL in n_} - set(prior.get("helpers") or ())
    # the HDL NCOs whose Complex exponential output goes only to the model's Outports in the user's design (the
    # recorded shape for parts wider than CPLX_MAX_WL, see post_compile_checks)
    nco_direct = _nco_cplx_direct(spec) | set(prior.get("nco_direct") or ())
    spec = bp_from_declared(spec)
    spec = _cplx_const(spec)   # a complex Constant -> two real Constants + Real-Imag to Complex
    if any(isinstance(b, dict) and b.get("type") == "HDLNCO" for b in spec["blocks"].values()):
        spec = _nco(spec)   # before the complex pass: a complex-exponential NCO feeds a Real-Imag to Complex
    if any(isinstance(b, dict) and b.get("type") in CPLX_TYPES + CORDIC_TYPES for b in spec["blocks"].values()):
        spec, cinfo = _complex(spec)
    blocks = dict(spec["blocks"])
    lines = list(spec.get("lines", []))
    info = {"bool_ctrl": [], "same_type": [], "ic_grid": [], "blocks": {}, "_spec": spec,
            "cplx_exact": (cinfo or {}).get("exact", []), "cplx_pairs": (cinfo or {}).get("pairs", []),
            "cplx_parts": (cinfo or {}).get("parts", {}),
            "cplx_region": (cinfo or {}).get("region", {}), "cplx_sums": (cinfo or {}).get("sums", []),
            "cplx_post": (cinfo or {}).get("post", [])}
    for n_, (m_, a_) in ((cinfo or {}).get("c2ma") or {}).items():
        # the magnitude and angle outputs are compiled types Simulink checks -- port 1 is
        # carried by the magnitude core, port 2 by the angle core
        info.setdefault("first", {})[n_] = m_[0]
        info.setdefault("ports", {})[n_] = {2: a_[0]}
    import fxtrigger
    if fxtrigger.has(spec):   # triggered / resettable subsystems
        spec = fxtrigger.rewrite(spec, info)
        info["_spec"] = spec
    import fxaction
    if fxaction.has(spec):   # Switch Case action subsystems, Merge, Data Store
        spec = fxaction.rewrite(spec, info)
        info["_spec"] = spec
    import fxprelookup
    if fxprelookup.has(spec):   # Prelookup, 1-D Interpolation Using Prelookup
        spec = fxprelookup.rewrite(spec, info)
        info["_spec"] = spec
    import fxzoh
    if fxzoh.has(spec):   # Zero-Order Hold (first: it can feed the blocks below)
        spec = fxzoh.rewrite(spec, info)
        info["_spec"] = spec
    import fxiir
    if fxiir.has(spec):   # Discrete Filter, Discrete Transfer Fcn
        spec = fxiir.rewrite(spec, info)
        info["_spec"] = spec
    blocks = dict(spec["blocks"])
    lines = list(spec.get("lines", []))
    import block_lowering
    if block_lowering.has(spec):   # recorded configurations: identity blocks, Assertions
        spec = block_lowering.lower_untyped(spec, info)
        blocks = dict(spec["blocks"])
        lines = list(spec.get("lines", []))
    pd_ = [n for n, b in blocks.items() if port_delay(b)]
    if pd_:
        # the Delays with an enable / reset port that sit on a cycle of the design (checked against
        # the recorded feedback shapes in post_compile_checks)
        succ_ = {}
        for ln in lines:
            succ_.setdefault(ln.get("src"), set()).add(ln.get("dst"))
        for n in pd_:
            seen_, todo_ = set(), list(succ_.get(n, ()))
            while todo_:
                x_ = todo_.pop()
                if x_ not in seen_:
                    seen_.add(x_)
                    todo_.extend(succ_.get(x_, ()))
            if n in seen_:
                info.setdefault("pdelay_loop", set()).add(n)
    new_lines = []
    feeds = {}   # (block, dport) -> line
    for ln in lines:
        if _lowered(blocks.get(ln.get("dst"), {})):
            feeds[(ln["dst"], int(ln.get("dport", 1)))] = ln
    for n, b in spec["blocks"].items():
        if not _lowered(b):
            continue
        where = "design.blocks." + n
        X = INTERNAL + n + "/"
        if port_delay(b):
            for k_ in ("DelayLengthSource", "InitialConditionSource"):
                if str(b.get(k_, "Dialog")).strip() != "Dialog":
                    # before the port check: the shape is not recorded however it is wired
                    _refuse(where + "." + k_, "%s=%r: only 'Dialog' (a delay length or initial condition from an "
                            "input port was not recorded with an enable or reset port)" % (k_, b.get(k_)))
        ports = n_inputs(b, where)
        src = {}
        for k, nm in enumerate(ports):
            ln = feeds.get((n, k + 1))
            if ln is None:
                if nm in ("E", "R"):
                    # Simulink grounds an unwired enable / reset port and runs (a request error
                    # was untrue); other ports stay request errors (a bare 2-D Direct Lookup Table's index port 2
                    # stops Simulink: recordings)
                    _refuse(where, "input port %d (%s) is not connected; Simulink grounds an unconnected input (0) "
                            "and runs, which is not supported here -- connect it" % (k + 1, nm))
                _refuse(where, "input port %d (%s) is not connected" % (k + 1, nm), "BAD_CONNECTION")
            src[nm] = (ln["src"], int(ln.get("sport", 1)))
        extra = [k for (d, k) in feeds if d == n and k > len(ports)]
        if extra:
            _refuse(where, "input port %d does not exist (%s has %d)" % (extra[0], b["type"], len(ports)),
                    "BAD_CONNECTION")
        add, wires = {}, []

        def wire(s, d, port=1):
            if isinstance(s, tuple):
                wires.append({"src": s[0], "sport": s[1], "dst": d, "dport": port} if s[1] != 1 else
                             {"src": s[0], "dst": d, "dport": port})
            else:
                wires.append({"src": s, "dst": d, "dport": port})
        if port_delay(b):
            _port_delay(n, b, X, src, add, wire, info, where)
        elif b["type"] in SYNC:
            _sync(n, b, X, src, add, wire, info, where)
        elif b["type"] == "HDLCounter":
            _counter(n, b, X, src, add, wire, info, where)
        elif b["type"] == "DetectRisePositive":
            _drp(n, b, X, src, add, wire, info, where)
        elif b["type"] == "DirectLookupND":
            _dlut(n, b, X, src, add, wire, info, where)
        elif b["type"] == "Repeat":
            _repeat(n, b, X, src, add, wire, info, where)
        elif b["type"] == "HDLRAM":
            _ram(n, b, X, src, add, wire, info, where)
        elif b["type"] == "HDLFIFO":
            import fxhdlfifo
            fxhdlfifo.lower(n, b, X, src, add, wire, info, where)
        elif b["type"] == "MATLABFunction":
            _mlfn(n, b, X, src, add, wire, info, where, ports)
        elif b["type"] in ("HDLFIRFilter", "HDLFIRDecimator"):
            _hdlfir(n, b, X, src, add, wire, info, where)
        else:
            # Data Type Duplicate: no output; its inputs must end up with one type (post_compile_checks)
            info.setdefault("dtd", []).append((n, [src[k] for k in sorted(src, key=lambda x: int(x[2:]))]))
        for k in add:
            if k in blocks and k != n:
                _refuse(where, "internal name clash %r" % k, "INTERNAL")
        blocks.update(add)
        if b["type"] == "DataTypeDuplicate":
            del blocks[n]
        new_lines += wires
        info["blocks"][n] = sorted(add)
    # lines into lowered blocks are replaced by the internal wiring; lines out of a counter's port 2 come from the
    # internal hit comparison
    out = []
    for ln in lines:
        if ln.get("dst") in info["blocks"]:
            continue
        if spec["blocks"].get(ln.get("src"), {}).get("type") == "DataTypeDuplicate":
            _refuse("design.lines", "Data Type Duplicate %s has no output port" % ln["src"], "BAD_CONNECTION")
        s = ln.get("src")
        if s in info["blocks"] and int(ln.get("sport", 1)) != 1:
            p2 = info.get("port2", {}).get(s) if int(ln["sport"]) == 2 else None
            p2 = info.get("ports", {}).get(s, {}).get(int(ln["sport"]), p2)
            if p2 is None:
                _refuse("design.lines", "output port %s of %s does not exist" % (ln["sport"], s), "BAD_CONNECTION")
            ln = {k: v for k, v in ln.items() if k != "sport"}
            ln["src"] = p2
        out.append(ln)
    # the internal wiring of a lowered block fed from another lowered block's port 2 (count_hit into a
    # synchronous delay's enable / reset) is re-pointed the same way
    nl = []
    for ln in new_lines:
        s = ln.get("src")
        if s in info["blocks"] and int(ln.get("sport", 1)) != 1:
            p2 = info.get("port2", {}).get(s) if int(ln["sport"]) == 2 else None
            p2 = info.get("ports", {}).get(s, {}).get(int(ln["sport"]), p2)
            if p2 is None:
                _refuse("design.lines", "output port %s of %s does not exist" % (ln["sport"], s), "BAD_CONNECTION")
            ln = dict({k: v for k, v in ln.items() if k != "sport"}, src=p2)
        nl.append(ln)
    low = {"blocks": blocks, "lines": out + nl}
    import parameter_bounds
    if spec.get(parameter_bounds.PORT_KEY):
        low[parameter_bounds.PORT_KEY] = spec[parameter_bounds.PORT_KEY]
    if spec.get(parameter_bounds.NOTES_KEY):   # the filter lowerings' coefficient notes (engine/fxiir.py)
        low[parameter_bounds.NOTES_KEY] = spec[parameter_bounds.NOTES_KEY]
    if any(b.get("type") in ES_TYPES for b in blocks.values()):
        pk, nk = low.get(parameter_bounds.PORT_KEY), low.get(parameter_bounds.NOTES_KEY)
        low = _enabled_subsystems(low, info)
        if pk:
            low[parameter_bounds.PORT_KEY] = pk
        if nk:
            low[parameter_bounds.NOTES_KEY] = nk
    info["user_types"] = user0
    info["nco_cplx_direct"] = sorted(nco_direct)
    info["lab2_helpers"] = sorted(n_ for n_ in low["blocks"] if INTERNAL in n_ and n_ not in internal0)
    return low, info


# the typed passes' markers on a block are this object, which no JSON design can carry -- a user's
# "_EnSigned": "off" selected the pre-recordings enable rule; any other value is an unknown parameter now
MARK = ("lab2 internal marker",)


def b_type_of(b):
    return b.get("type", "?") if isinstance(b, dict) else "?"


def _marked(b, k):
    return b.get(k) is MARK


def _sync(n, b, X, src, add, wire, info, where):
    en_signed, bool_data = _marked(b, "_EnSigned"), _marked(b, "_BoolData")
    p = dict(SYNC_DEFAULTS, **{k: str(v) for k, v in b.items() if k != "type" and k != "_EnType" and not (
        k in ("_EnSigned", "_BoolData") and b[k] is MARK)})
    extra = sorted(set(p) - set(SYNC_DEFAULTS))
    if extra:
        _refuse(where + "." + extra[0], "unknown parameter %s for %s" % (extra[0], b["type"]), "BAD_PARAM")
    ic, st = p["InitialCondition"], p["SampleTime"]
    _num(ic, where + ".InitialCondition")
    info["ic_grid"].append((n, ic, src["u"][0] if src["u"][1] == 1 else None))
    # an initial value exactly half-way rounds half up here too (38 / 38 equal: the three
    # synchronous unit delays and the Enabled Synchronous Tapped Delay built from them), as the port Delay's
    info.setdefault("ic_ties_nearest", set()).add(n)
    if bool_data:
        if str(ic).strip() not in ("0", "1", "0.0", "1.0"):
            _refuse(where + ".InitialCondition", 'boolean data with the initial value %r (recorded 0 and 1'
                    ')' % ic)
        info.setdefault("ic_bool_ok", set()).add(n)
    # a control input of any type: non-zero is true (recordings ), made boolean by
    # NOT(NOT x) -- the identity for a boolean
    for c in ("E", "R"):
        if c in src:
            # the controls' compiled types, checked once known (post_compile_checks) -- the typed pass
            # cannot probe a control that depends on the delay's own output
            info.setdefault("sync_ctrl", []).append((n, c, src[c], en_signed))
        if c == "E" and c in src and en_signed:
            # recordings (recorded): a signed enable enables only when E > 0 (int8 -1 is disabled); the reset stays
            # non-zero (negative resets recorded)
            a = X + "E_pos"
            add[a] = {"type": "CompareToConstant", "relop": ">", "const": "0", "OutDataTypeStr": "boolean"}
            if isinstance(b.get("_EnType"), str):
                # lowered again with the enable's compiled type (a loop through the delay's output):
                # an identity conversion gives the comparison its type source
                add[X + "E_ty"] = {"type": "DataTypeConversion", "OutDataTypeStr": b["_EnType"], "RndMeth": "Floor",
                                   "SaturateOnIntegerOverflow": "off"}
                wire(src[c], X + "E_ty", 1)
                wire((X + "E_ty", 1), a, 1)
            else:
                wire(src[c], a, 1)
            src[c] = a
            continue
        if c in src:
            a, b2 = X + c + "_n1", X + c + "_n2"
            add[a] = {"type": "LogicalOperator", "Operator": "NOT", "Inputs": "1"}
            add[b2] = {"type": "LogicalOperator", "Operator": "NOT", "Inputs": "1"}
            wire(src[c], a, 1)
            wire(a, b2, 1)
            src[c] = b2
    u = src["u"]
    _st = str(st).strip().strip("[]").replace(",", " ").split()
    if len(_st) == 2:
        try:
            _off = float(_st[1])
        except ValueError:
            _off = None
        if _off != 0:
            # recordings (silent wrong after a previous version opened [P o] offsets on plain blocks): the
            # synchronous delays with a sample-time offset are not recorded -- refused
            _refuse(where + ".SampleTime", "%s with the sample-time offset %s: offsets were recorded on plain blocks "
                    "only (engine/offset_domain.json); leave SampleTime -1 or [P 0]" % (b["type"], st),
                    "UNSUPPORTED_RATE")
    dly = {"type": "UnitDelay", "InitialCondition": ic, "SampleTime": st}
    if "R" in src and bool_data:
        # R ? ic: u for boolean data -- OR(R, u) for ic 1, AND(NOT R, u) for ic 0; the reset wins
        # over the enable (the hold below loads on E OR R)
        m = X + "rst"
        if float(str(ic).strip()) == 1:
            add[m] = {"type": "LogicalOperator", "Operator": "OR", "Inputs": "2"}
            wire(src["R"], m, 1)
        else:
            nr = X + "R_not"
            add[nr] = {"type": "LogicalOperator", "Operator": "NOT", "Inputs": "1"}
            wire(src["R"], nr, 1)
            add[m] = {"type": "LogicalOperator", "Operator": "AND", "Inputs": "2"}
            wire(nr, m, 1)
        wire(u, m, 2)
        if "E" in src:
            en = X + "en"
            add[en] = {"type": "LogicalOperator", "Operator": "OR", "Inputs": "2"}
            wire(src["E"], en, 1)
            wire(src["R"], en, 2)
            h = X + "hold"
            add[h] = {"type": "Hold", "InitialCondition": ic}
            wire(m, h, 1)
            wire(en, h, 2)
            nxt = h
        else:
            nxt = m
    elif "R" in src:
        f = X + "false"
        add[f] = {"type": "RelationalOperator", "Operator": "~="}       # always false, boolean
        wire(u, f, 1)
        wire(u, f, 2)
        icsig = X + "ic"
        add[icsig] = {"type": "Hold", "InitialCondition": ic}           # never enabled: IC in the input's type
        wire(u, icsig, 1)
        wire(f, icsig, 2)
        m = X + "rst"
        add[m] = {"type": "Switch", "Criteria": "u2 ~= 0", "OutDataTypeStr": G.fc._SAME_FIRST}
        wire(icsig, m, 1)
        wire(src["R"], m, 2)
        wire(u, m, 3)
        if "E" in src:
            en = X + "en"
            add[en] = {"type": "LogicalOperator", "Operator": "OR", "Inputs": "2"}
            wire(src["E"], en, 1)
            wire(src["R"], en, 2)
            h = X + "hold"
            add[h] = {"type": "Hold", "InitialCondition": ic}
            wire(m, h, 1)
            wire(en, h, 2)
            nxt = h
        else:
            nxt = m
    else:
        h = X + "hold"
        add[h] = {"type": "Hold", "InitialCondition": ic}
        wire(u, h, 1)
        wire(src["E"], h, 2)
        nxt = h
    add[n] = dly
    wire(nxt, n, 1)


def _port_delay(n, b, X, src, add, wire, info, where):
    """recordings (14 recorded configurations, every step reproduced): the core Delay with an
    enable port and / or an external reset.
    - an enabled step outputs the oldest stage and shifts u in; a disabled step holds the previous output and the
      state;
    - a reset step (enabled) outputs the initial condition and resets every stage to it, then shifts u in;
    - the trigger: Rising r and not r', Falling r' and not r, Either r != r', Level r or r' (non-zero, or just fell
      to zero), Level hold r -- r' = the reset input one step earlier, kept on disabled steps too (the recorded
      enable + Level case: a disabled step neither resets nor writes)."""
    for k_ in ("DelayLengthSource", "InitialConditionSource"):
        if str(b.get(k_, "Dialog")).strip() != "Dialog":
            # main answers a length / initial condition from an input port without an enable /
            # reset port; the two together were not recorded
            raise G.GraphError("UNSUPPORTED", where + "." + k_, "a Delay with an enable or reset port and %s %r: not "
                               "recorded; record such a Delay to open it" % (k_, b.get(k_)))
    p = dict({k: v for k, v in PDELAY_DEFAULTS.items() if v is not None},
             **{k: str(v).strip() for k, v in b.items() if k != "type"})
    extra = sorted(set(p) - set(PDELAY_DEFAULTS))
    if extra:
        _refuse(where + "." + extra[0], "unknown parameter %s for Delay" % extra[0], "BAD_PARAM")
    if "DelayLength" not in p:
        _refuse(where + ".DelayLength", "DelayLength is required", "MISSING_FIELD")
    for k in ("DelayLengthSource", "InitialConditionSource"):
        if p[k] != "Dialog":
            _refuse(where + "." + k, "%s=%r: only 'Dialog' (a delay length or initial condition from an input "
                    "port was not recorded)" % (k, p[k]))
    if p["ShowEnablePort"] not in ("on", "off"):
        _refuse(where + ".ShowEnablePort", "on or off", "BAD_PARAM")
    if p["ExternalReset"] not in PDELAY_RESETS:
        _refuse(where + ".ExternalReset", "one of %s" % ", ".join(PDELAY_RESETS), "BAD_PARAM")
    if (p["ShowEnablePort"], p["ExternalReset"]) not in PDELAY_RECORDED:
        _refuse(where, "Delay with ShowEnablePort %s and ExternalReset %r is not supported (recorded: the enable port "
                "alone; each external reset without the enable port; the enable port with each reset)"
                % (p["ShowEnablePort"], p["ExternalReset"]))
    if p["SampleTime"] not in ("-1", "1"):
        _refuse(where + ".SampleTime", "SampleTime %r: inherited (-1) or 1 only" % p["SampleTime"])
    N = _pint(p["DelayLength"], where + ".DelayLength")
    combo = (p["ShowEnablePort"], p["ExternalReset"])
    if combo in PDELAY_A26_NEW:
        ok_len = N in PDELAY_A26_LENGTHS[combo]
    else:
        ok_len = N in PDELAY_LENGTHS or (combo in PDELAY_SWEPT and 1 <= N <= PDELAY_MAX) or \
            N in PDELAY_SPARSE_LENGTHS or N in PDELAY_A26_LENGTHS.get(combo, ())
    if not ok_len:
        # only the recorded lengths / ranges
        if combo in PDELAY_A26_NEW:
            rec = ", ".join(map(str, PDELAY_A26_LENGTHS[combo]))
        elif combo in PDELAY_SWEPT:
            rec = "1 .. %d" % PDELAY_MAX
        else:
            rec = ", ".join(map(str, sorted(set(PDELAY_LENGTHS + PDELAY_SPARSE_LENGTHS +
                                                PDELAY_A26_LENGTHS.get(combo, ())))))
        _refuse(where + ".DelayLength", "DelayLength %d with ShowEnablePort %s and ExternalReset %r: recorded lengths "
                "for this combination: %s"
                % (N, combo[0], combo[1], rec))
    ic = p["InitialCondition"]
    if ic.startswith("["):
        _refuse(where + ".InitialCondition", "a vector initial condition with an enable / reset port (not recorded)")
    _num(ic, where + ".InitialCondition")
    u = src["u"]
    info.setdefault("pdelay_meta", []).append((n, combo, u))   # hardware / data / loop gate
    # the initial condition is rounded to the data type and must fit it, as for the synchronous delays; boolean
    # data is refused there too
    info["ic_grid"].append((n, ic, None))   # under the user name: boolean data is refused there
    info.setdefault("ic_ties_nearest", set()).add(n)   # a tie rounds Nearest
    sign = {}
    for c in ("E", "R"):
        if c in src:
            # the type check (boolean only) is on the source after compile; inside, NOT(NOT x) -- the identity for
            # a boolean -- so a wrong type reaches that check instead of failing on an internal block
            info.setdefault("bool_port", []).append((n, {"E": "enable", "R": "external reset"}[c], src[c]))
            raw = src[c]
            a, b2 = X + c + "_n1", X + c + "_n2"
            add[a] = {"type": "LogicalOperator", "Operator": "NOT", "Inputs": "1"}
            add[b2] = {"type": "LogicalOperator", "Operator": "NOT", "Inputs": "1"}
            wire(src[c], a, 1)
            wire(a, b2, 1)
            src[c] = b2
            # recordings (int8 / int32 controls, 300+ steps each): a signed control acts by its
            # sign -- an enable is on when > 0; edges follow the sign (below). x > 0 and x < 0 from x and a zero of
            # its type (a never-enabled Hold of x); for a boolean or unsigned control x < 0 is never true and x > 0
            # is x ~= 0, so these give the boolean rule unchanged
            fz, zz = X + c + "_f", X + c + "_z"
            add[fz] = {"type": "RelationalOperator", "Operator": "~="}   # always false
            wire(raw, fz, 1)
            wire(raw, fz, 2)
            add[zz] = {"type": "Hold", "InitialCondition": "0"}
            wire(raw, zz, 1)
            wire(fz, zz, 2)
            for nm_, op_ in (("pos", ">"), ("neg", "<")):
                add[X + c + "_" + nm_] = {"type": "RelationalOperator", "Operator": op_}
                wire(raw, X + c + "_" + nm_, 1)
                wire(zz, X + c + "_" + nm_, 2)
            sign[c] = (X + c + "_pos", X + c + "_neg")
    if "E" in src:
        src["E"] = sign["E"][0]   # enabled when > 0 (for booleans the same as ~= 0)
    icsig = X + "ic"
    # the always-false control and the initial-condition source take u's type only: they read u one step late
    # (a Unit Delay keeps the type), so a feedback loop through the Delay is not an algebraic loop (
    # a resettable accumulator and a hold register run in Simulink)
    ud = X + "u_late"
    add[ud] = {"type": "UnitDelay", "InitialCondition": "0"}
    wire(u, ud, 1)
    add[X + "false"] = {"type": "RelationalOperator", "Operator": "~="}   # always false, boolean
    wire(ud, X + "false", 1)
    wire(ud, X + "false", 2)
    add[icsig] = {"type": "Hold", "InitialCondition": ic}                # never enabled: IC in the input's type
    wire(ud, icsig, 1)
    wire(X + "false", icsig, 2)
    trig = None
    if "R" in src:
        r = src["R"]
        rp = X + "rprev"
        add[rp] = {"type": "UnitDelay", "InitialCondition": "0", "SampleTime": "-1"}   # every step, enabled or not
        wire(r, rp, 1)
        mode = p["ExternalReset"]
        trig = X + "trig"
        if mode == "Level hold":
            add[trig] = {"type": "LogicalOperator", "Operator": "OR", "Inputs": "2"}   # r OR r = r, boolean
            wire(r, trig, 1)
            wire(r, trig, 2)
        elif mode == "Level":
            add[trig] = {"type": "LogicalOperator", "Operator": "OR", "Inputs": "2"}
            wire(r, trig, 1)
            wire(rp, trig, 2)
        else:
            # recordings (int8 / int32 resets, 12 units, every step equal): edges follow the
            # control's sign. Rising: from negative to zero or positive, or from zero to positive unless the sample
            # before that zero was negative (-5, 0, 5 is one rising edge; -5, 0, 0, 5 two); Falling mirrored; Either
            # both. For a boolean / unsigned control this is r AND NOT r' (Rising) and r' AND NOT r (Falling)
            pos, neg = sign["R"]

            def lop(nm_, op_, *xs):
                add[X + nm_] = {"type": "LogicalOperator", "Operator": op_, "Inputs": str(len(xs))}
                for k_, x_ in enumerate(xs):
                    wire(x_, X + nm_, k_ + 1)
                return X + nm_

            def late(nm_, x_):
                add[X + nm_] = {"type": "UnitDelay", "InitialCondition": "0", "SampleTime": "-1"}
                wire(x_, X + nm_, 1)
                return X + nm_
            pp1, np1 = late("pos_p", pos), late("neg_p", neg)
            pp2, np2 = late("pos_pp", pp1), late("neg_pp", np1)
            zero_p = lop("zero_p", "NOR", pp1, np1)
            edges = []
            if mode in ("Rising", "Either"):
                edges.append(lop("rise", "OR", lop("rise1", "AND", np1, lop("nneg", "NOT", neg)),
                                 lop("rise2", "AND", zero_p, pos, lop("nnegpp", "NOT", np2))))
            if mode in ("Falling", "Either"):
                edges.append(lop("fall", "OR", lop("fall1", "AND", pp1, lop("npos", "NOT", pos)),
                                 lop("fall2", "AND", zero_p, neg, lop("npospp", "NOT", pp2))))
            if len(edges) == 1:
                add[trig] = {"type": "LogicalOperator", "Operator": "OR", "Inputs": "2"}
                wire(edges[0], trig, 1)
                wire(edges[0], trig, 2)
            else:
                add[trig] = {"type": "LogicalOperator", "Operator": "OR", "Inputs": "2"}
                wire(edges[0], trig, 1)
                wire(edges[1], trig, 2)
    en = src.get("E")

    def gated(name, d_in):
        """a stage register: d_in written on enabled steps, held on disabled ones"""
        if en is None:
            add[name] = {"type": "UnitDelay", "InitialCondition": ic, "SampleTime": "-1"}
            wire(d_in, name, 1)
            return
        add[name + "_h"] = {"type": "Hold", "InitialCondition": ic}
        wire(d_in, name + "_h", 1)
        wire(en, name + "_h", 2)
        add[name] = {"type": "UnitDelay", "InitialCondition": ic, "SampleTime": "-1"}
        wire(name + "_h", name, 1)

    def reset_or(name, x):
        """the initial condition on a reset step, else x (typed as the data: Same as first input on the IC)"""
        if trig is None:
            return x
        add[name] = {"type": "Switch", "Criteria": "u2 ~= 0", "OutDataTypeStr": G.fc._SAME_FIRST}
        wire(icsig, name, 1)
        wire(trig, name, 2)
        wire(x, name, 3)
        return name
    stages = [X + "s%d" % k for k in range(1, N + 1)]
    for k, s in enumerate(stages):
        gated(s, u if k == 0 else reset_or(X + "rs%d" % (k + 1), stages[k - 1]))
    out = reset_or(X + "ro", stages[-1])
    if en is None:
        # the output block keeps the user's name: the last value through a Switch on an always-false control
        add[n] = {"type": "Switch", "Criteria": "u2 ~= 0", "OutDataTypeStr": G.fc._SAME_FIRST}
        wire(icsig, n, 1)
        wire(X + "false", n, 2)
        wire(out, n, 3)
    else:
        # a disabled step holds the previous output; before the first enabled step that is 0, not the initial
        # condition (Simulink 0 at step 0 with the enable low, recordings)
        add[n] = {"type": "Hold", "InitialCondition": "0"}
        wire(out, n, 1)
        wire(en, n, 2)


def _counter(n, b, X, src, add, wire, info, where):
    p = dict(COUNTER_DEFAULTS, **{k: str(v) for k, v in b.items() if k != "type"})
    extra = sorted(set(p) - set(COUNTER_DEFAULTS))
    if extra:
        _refuse(where + "." + extra[0], "unknown parameter %s for HDLCounter" % extra[0], "BAD_PARAM")
    for k in ("CountResetPort", "CountLoadPort", "CountEnbPort", "CountDirPort", "CountHitOutputPort"):
        if p[k] not in ("on", "off"):
            _refuse(where + "." + k, "on or off", "BAD_PARAM")
    ports_ = [k for k in ("CountResetPort", "CountLoadPort", "CountEnbPort", "CountDirPort") if p[k] == "on"]
    if p["CountSampTime"].strip() != "1" and not (p["CountSampTime"].strip() in ("2", "3", "4")):
        # review of recordings: every recorded counter had CountSampTime 1; another
        # period ran every base step here. recordings: CountSampTime 2 / 3 / 4 without input ports
        # (Count limited / Free running / Modulo, 4 / 8 bits) -- the counter's register runs at that period
        _refuse(where + ".CountSampTime", "CountSampTime %s%s: recorded 1, and 2 / 3 / 4 without input ports, only"
                % (p["CountSampTime"], " with input ports" if ports_ else ""))
    typ = p["CountType"]
    if typ not in ("Free running", "Count limited", "Modulo"):
        _refuse(where + ".CountType", "Free running, Count limited or Modulo", "BAD_PARAM")
    if p["CountFromType"] != "Initial value":
        # Simulink refused CountFromType Specify in every attempt
        _refuse(where + ".CountFromType", "CountFromType %r (Simulink refused 'Specify' in every recorded "
                "attempt; only 'Initial value')" % p["CountFromType"])
    if p["CountDataType"] not in ("Unsigned", "Signed"):
        _refuse(where + ".CountDataType", "Unsigned or Signed", "BAD_PARAM")
    s = 1 if p["CountDataType"] == "Signed" else 0
    cf_, ci_ = p["CountFrom"].strip(), p["CountInit"].strip()
    if "CountFrom" in b and cf_ != ci_:
        # a written CountFrom that differs from CountInit was ignored ('Initial value'
        # mirrors CountInit) -- Simulink checks it at set_param
        try:
            cfv, civ, cmv = float(cf_), float(ci_), float(p["CountMax"])
        except ValueError:
            cfv = civ = cmv = None
        if cfv is not None and cfv < 0 and not s:
            _refuse(where + ".CountFrom", "CountFrom %s on an unsigned counter: Simulink rejects it "
                    "(hdlsllib:hdlsllib:countvalrange)" % cf_, "BAD_PARAM")
        if cfv is not None and p["CountType"] == "Count limited" and not cfv <= civ <= cmv:
            _refuse(where + ".CountFrom", "CountFrom %s with CountInit %s outside [%s, %s]: Simulink rejects it "
                    "(hdlsllib:hdlsllib:initoutofrange)" % (cf_, ci_, cf_, p["CountMax"]), "BAD_PARAM")
        _refuse(where + ".CountFrom", "CountFrom %s different from CountInit %s with CountFromType 'Initial value': "
                "not recorded; leave CountFrom out (it follows CountInit)" % (cf_, ci_))
    try:
        w, f = _pint(p["CountWordLen"], where + ".CountWordLen"), _pint(p["CountFracLen"], where + ".CountFracLen")
    except ValueError:
        _refuse(where + ".CountWordLen", "integer word / fraction length", "BAD_PARAM")
    if not (1 + s <= w <= G.MAX_WL):
        _refuse(where + ".CountWordLen", "word length %d (Simulink refused 0; %d..%d)" % (w, 1 + s, G.MAX_WL),
                "BAD_PARAM")
    if (f > w) if not s else (f >= w):
        # recordings (7 recordings): Simulink refuses it --
        # hdlsllib:fraclengthunsigned / hdlsllib:fraclengthsigned; an unsigned fraction length equal to the word
        # length is accepted (ufix1_En1 .. ufix3_En3, recorded equal)
        _refuse(where + ".CountFracLen", "fraction length %d with a %d-bit %s counter: the fraction length must be "
                "%s the word length (Simulink: hdlsllib:fraclength%s)" % (
                    f, w, "signed" if s else "unsigned", "less than" if s else "at most",
                    "signed" if s else "unsigned"), "BAD_PARAM")
    lo, hi = (-(1 << (w - 1)), (1 << (w - 1)) - 1) if s else (0, (1 << w) - 1)

    def q(k, check_range=True):
        try:
            v = Fraction(float(_num(p[k], where + "." + k))) * (Fraction(2) ** f)   # MATLAB evaluates a double
        except (OverflowError, ValueError):
            # '1e400' overflows a double (Inf in MATLAB): a parameter error, not an INTERNAL crash
            raise G.GraphError("BAD_PARAM", where + "." + k, "%s=%s is not a finite number" % (k, p[k]))
        if v.denominator != 1:
            _refuse(where + "." + k, "%s=%s is not on the counter's grid 2^-%d (quantisation not observed)"
                    % (k, p[k], f))
        v = int(v)
        if check_range and not lo <= v <= hi:
            # Simulink:Masking:Bad_Init_Commands
            _refuse(where + "." + k, "%s=%s does not fit the counter type" % (k, p[k]), "BAD_PARAM")
        return v
    init, step = q("CountInit"), q("CountStep", False)
    mx = q("CountMax", typ != "Free running")
    if typ == "Free running":
        mx = max(lo, min(hi, mx))   # unused by a free-running counter
    # a negative step counts the other way (unsigned, step -1); the constant holds |step|
    neg = step < 0
    step = abs(step)
    if step > hi:
        _refuse(where + ".CountStep", "|CountStep| does not fit the counter type (not observed)")
    if step == 0:
        _refuse(where + ".CountStep", "CountStep 0 (Simulink refuses it)", "BAD_PARAM")
    # a count type with no integer bits and a non-zero CountInit was
    # refused after recordings -- but Simulink's refusals there came from the recording writing CountInit while
    # CountMax / CountStep still held the library defaults (the mask checks each pair in order); set in a safe order
    # Simulink accepted 54 / 54 such counters and they equal the engine every step. The settings that are illegal
    # in themselves stay refused below (recordings 10 / 10)
    if typ != "Free running" and mx == init:
        _refuse(where + ".CountMax", "CountMax equal to CountInit (Simulink refuses it)", "BAD_PARAM")
    hit = p["CountHitOutputPort"] == "on"
    if hit and p["CountDirPort"] == "on":
        _refuse(where + ".CountHitOutputPort", "count_hit together with the direction port (Simulink refuses it)",
                "BAD_PARAM")
    if typ == "Modulo" and mx <= init:
        # Simulink cycles between CountMax and CountInit (7 3 4 5 6 7 3 ...);
        # one recording, not modelled -- refused as a rare shape
        _refuse(where + ".CountMax", "Modulo counter with CountMax below CountInit: Simulink then cycles between the "
                "two, which is not modelled; use CountMax above CountInit")
    if typ == "Modulo" and f != 0:
        # recordings (recorded: hdlsllib:fraclengthInModuloMode)
        _refuse(where + ".CountFracLen", "a Modulo counter with a fraction length (Simulink refuses it)", "BAD_PARAM")
    T = "fixdt(%d,%d,%d)" % (s, w, f)
    # the wide intermediate is signed, so an unsigned counter counting down below CountInit sees it
    # (it wrapped the word: uint8 Modulo, CountMax 5, 0 - 1 gave 255, Simulink 5)
    W = "fixdt(1,%d,%d)" % (w + 2, f)

    def val(v, fl=f):
        if abs(v) >= 1 << 53:
            _refuse(where, "counter values beyond 2^53 stored integers are not exported exactly")
        return repr(float(Fraction(v, 1 << fl))) if fl >= 0 else str(v * (1 << -fl))

    def const(nm, v, ty):
        add[X + nm] = {"type": "Constant", "Value": val(v), "OutDataTypeStr": ty, "SampleTime": "inf"}
        return X + nm

    def sum_(nm, signs, a, c, ty):
        add[X + nm] = {"type": "Sum", "Inputs": signs, "OutDataTypeStr": ty, "AccumDataTypeStr": ty,
                       "SaturateOnIntegerOverflow": "off", "RndMeth": "Floor"}
        wire(a, X + nm, 1)
        wire(c, X + nm, 2)
        return X + nm

    def rel(nm, op, a, c):
        add[X + nm] = {"type": "RelationalOperator", "Operator": op}
        wire(a, X + nm, 1)
        wire(c, X + nm, 2)
        return X + nm

    def switch(nm, a, ctrl, c, ty):
        add[X + nm] = {"type": "Switch", "Criteria": "u2 ~= 0", "OutDataTypeStr": ty,
                       "SaturateOnIntegerOverflow": "off", "RndMeth": "Floor"}
        wire(a, X + nm, 1)
        wire(ctrl, X + nm, 2)
        wire(c, X + nm, 3)
        return X + nm

    def dtc(nm, a, ty):
        add[X + nm] = {"type": "DataTypeConversion", "OutDataTypeStr": ty, "RndMeth": "Floor",
                       "SaturateOnIntegerOverflow": "off"}
        wire(a, X + nm, 1)
        return X + nm
    c = n
    k_init, k_step, k_max = const("init", init, T), const("step", step, T), const("max", mx, T)
    # recordings (108 counters): a negative step counts the other way in every type, with or without the
    # direction port (dir false negates the step: d*_lim_stm*, c021 / c045 / c069, recordings, *_mod_stm*)
    plus, minus = ("+-", "++") if neg else ("++", "+-")
    if typ == "Free running":
        up, down = sum_("up", plus, c, k_step, T), sum_("down", minus, c, k_step, T)
    elif typ == "Count limited":
        eq = rel("eq", "==", c, k_max)
        up = switch("upsel", k_init, eq, sum_("up", plus, c, k_step, T), T)
        # recordings (28 steps counting down at the maximum, every one back to
        # the initial value): at CountMax the next step is CountInit whatever the direction; otherwise down wraps
        # (0 - 1 -> 255 with CountMax 25)
        down = switch("downmax", k_init, eq, sum_("down", minus, c, k_step, T), T)
    else:
        # the range is CountInit..CountMax (19 + 2 -> 3, 3 - 2 -> 19)
        kw_max, kw_m, kw_zero = const("maxw", mx, W), const("modw", mx - init + 1, W), const("zerow", init, W)
        upw = sum_("upw", "++", c, k_step, W)
        upsel = switch("upsel", sum_("upm", "+-", upw, kw_m, W), rel("gt", ">", upw, kw_max), upw, W)
        up = dtc("upc", upsel, T)
        dnw = sum_("downw", "+-", c, k_step, W)
        dnsel = switch("downsel", sum_("downm", "++", dnw, kw_m, W), rel("lt", "<", dnw, kw_zero), dnw, W)
        down = dtc("downc", dnsel, T)
        if neg:   # a negative step moves down when counting "up" (the range wraps once at CountInit)
            up, down = down, up
    v = switch("dir", up, src["dir"], down, T) if "dir" in src else up
    if "enb" in src:
        v = switch("enb", v, src["enb"], c, T)
    if "load" in src:
        v = switch("load", src["load_val"], src["load"], v, T)
        info["same_type"].append((n, src["load_val"], (s, w, f)))
    if "rst" in src:
        v = switch("rst", k_init, src["rst"], v, T)
    for nm in ("rst", "load", "enb", "dir"):
        if nm in src:
            info["bool_ctrl"].append((n, nm, src[nm]))
    v = dtc("next", v, T)   # an explicit type in the loop, so the state's type never waits on the loop
    # a counter with input ports runs at its ports' rate (CountSampTime 1, ports at period
    # 2 / 3: the count steps once per port sample); CountSampTime is the period of a counter without ports
    cst_ = p["CountSampTime"].strip()
    add[c] = {"type": "UnitDelay", "InitialCondition": val(init), "SampleTime": "-1" if src else cst_}
    if src and cst_ != "1":
        # recordings (102 designs, every step equal): ports with CountSampTime 2 / 3 / 4 were
        # recorded with the ports sampled at the counter's own period (the register runs at its ports' rate, as
        # show for CountSampTime 1) -- a design input at any other period is refused in
        # post_compile_checks
        info.setdefault("counter_st_ports", []).append((n, cst_))
    wire(v, c, 1)
    if hit:
        # count_hit (13 + 11 counters with it, every type, negative
        # steps): with e the step this step would take (the signed step, negated when dir is false),
        #   Count limited  count == CountMax, or count + e leaves the word;
        #   Modulo         count + e > CountMax (e > 0), count + e < CountInit (e < 0);
        #   Free running   count + e leaves the word.
        info.setdefault("port2", {})[n] = X + "hit"
        # recordings (51 recorded hit counters): hit = condition AND NOT rst AND NOT load (enable does not
        # gate it)
        gate = [g for g in ("rst", "load") if g in src]
        cn = "hitc" if gate else "hit"
        k_top, k_bot = const("topw", hi, W), const("botw", lo, W)

        def cond(tag, sgn):
            # sgn "++": count + |step|, "+-": count - |step|; which one is e depends on the step's sign and dir
            nx = sum_("hitw" + tag, sgn, c, k_step, W)
            if typ == "Modulo":
                if sgn == "++":
                    return rel("hitmx" + tag, ">", nx, X + "maxw")
                return rel("hitmn" + tag, "<", nx, X + "zerow")
            ov = X + "hitov" + tag
            add[ov] = {"type": "LogicalOperator", "Operator": "OR", "Inputs": "2"}
            wire(rel("hithi" + tag, ">", nx, k_top), ov, 1)
            wire(rel("hitlo" + tag, "<", nx, k_bot), ov, 2)
            if typ != "Count limited":
                return ov
            o = X + "hitcl" + tag
            add[o] = {"type": "LogicalOperator", "Operator": "OR", "Inputs": "2"}
            wire(rel("hiteq" + tag, "==", c, k_max), o, 1)
            wire(ov, o, 2)
            return o
        h_up = cond("u", "+-" if neg else "++")
        h = h_up
        if "dir" in src:   # never with count_hit (Simulink: mutually exclusive, refused above), kept for safety
            h = switch("hitdir", h_up, src["dir"], cond("d", "++" if neg else "+-"), "boolean")
        add[X + cn] = {"type": "DataTypeConversion", "OutDataTypeStr": "boolean", "RndMeth": "Floor",
                       "SaturateOnIntegerOverflow": "off"}
        wire(h, X + cn)
        if gate:
            add[X + "hit"] = {"type": "LogicalOperator", "Operator": "AND", "Inputs": str(1 + len(gate))}
            wire(X + cn, X + "hit", 1)
            for k, g in enumerate(gate):
                ng = X + "hitn_" + g
                add[ng] = {"type": "LogicalOperator", "Operator": "NOT", "Inputs": "1"}
                wire(src[g], ng, 1)
                wire(ng, X + "hit", k + 2)


# recordings (class: the popup parameters of the library (masked) blocks recorded lowers -- a mask popup takes only
# its exact option text in Simulink, BadSetParamValue otherwise): the option lists of the popups read here
_PROC = ("Columns as channels (frame based)", "Elements as channels (sample based)")
POPUP_OPTIONS = {("Repeat", "InputProcessing"): _PROC,
                 ("Repeat", "RateOptions"): ("Enforce single-rate processing", "Allow multirate processing"),
                 ("Repeat", "FactorSource"): ("Dialog parameter", "Input port"),
                 # recordings: 'Maintain input frame rate' runs in Simulink (frame based) -- the block's
                 # two frame modes are options too (refused as not supported where the lowering does not take them)
                 ("Repeat", "mode"): ("Unused parameter value", "Maintain input frame size", "Maintain input frame rate")}
# (Detect Rise Positive is lowered by the engine's code, fxgraph_lib, which reads its InputProcessing)


# options Simulink takes that the engine does not model: refused as not supported (after the spelling check)
POPUP_UNMODELLED = {("Repeat", "mode"): {
    "Maintain input frame size": "frame-based repetition (recordings show 'Unused parameter value' only)",
    "Maintain input frame rate": ('frame-based repetition keeping the frame rate '
        '(Simulink outputs N elements per step; not modelled)')}}


def _popup_check(t, p, where):
    for (t_, k), why in POPUP_UNMODELLED.items():
        if t_ == t and str(p.get(k, "")) in why:
            _refuse(where + "." + k, "%s %s %r is not supported: %s" % (t, k, p[k], why[p[k]]))
    for (t_, k), opts in POPUP_OPTIONS.items():
        if t_ != t or k not in p or p[k] in opts:
            continue
        v = p[k]
        near = [o for o in opts if o.lower() == " ".join(v.split()).lower() or
                (v.strip() and o.lower().startswith(v.strip().lower())) or o.lower() in v.lower()]
        if near:
            _refuse(where + "." + k, "%s %s %r: a mask popup takes only its exact option text in Simulink "
                    "(BadSetParamValue); write %r" % (t, k, v, near[0]), "BAD_PARAM")
        _refuse(where + "." + k, "%s %s %r is not one of the options %s" % (t, k, v, ", ".join(repr(o) for o in opts)),
                "BAD_PARAM")


def _params(b, defaults, where, extra_ok=()):
    p = dict(defaults, **{k: str(v) for k, v in b.items() if k != "type"})
    # the vector pass's element tie (_Tie) is internal, not a block parameter (Repeat over a vector)
    bad = sorted(set(p) - set(defaults) - set(extra_ok) - {"_Tie"})
    if bad:
        _refuse(where + "." + bad[0], "unknown parameter %s for %s" % (bad[0], b["type"]), "BAD_PARAM")
    _popup_check(b["type"], p, where)
    return p


DRP_DEFAULTS = {"vinit": "0", "InputProcessing": "Elements as channels (sample based)", "OutDataTypeStr": "boolean"}


BOOL_OUT = ("RelationalOperator", "LogicalOperator", "DetectRisePositive")


def _bool_source(spec, n, depth=0):
    """Structural test: is block n's output boolean whatever the input types? (a design Inport takes its type from
    the request: assumed numeric here and confirmed after compilation)."""
    b = spec["blocks"].get(n, {})
    t = b.get("type")
    if depth > 64 or t is None:
        return False
    if t in BOOL_OUT:
        return t != "DetectRisePositive" or b.get("OutDataTypeStr", "boolean") == "boolean"
    if t in ("CompareToConstant", "DetectChange", "DetectIncrease", "DetectDecrease"):
        return b.get("OutDataTypeStr", "boolean") == "boolean"
    if t in ("UnitDelay", "Delay", "Hold") + SYNC:
        srcs = [l["src"] for l in spec.get("lines", []) if l.get("dst") == n and int(l.get("dport", 1)) == 1]
        return bool(srcs) and _bool_source(spec, srcs[0], depth + 1)
    return False


def _drp(n, b, X, src, add, wire, info, where):
    """Detect Rise Positive: y = (previous input <= 0) and (input > 0); the previous input
    before step 0 is vinit; output boolean or uint8."""
    p = _params(b, DRP_DEFAULTS, where)
    if p["InputProcessing"] != "Elements as channels (sample based)":
        _refuse(where + ".InputProcessing", "%r (sample based only)" % p["InputProcessing"])
    if p["OutDataTypeStr"] not in ("boolean", "uint8"):
        _refuse(where + ".OutDataTypeStr", "boolean or uint8 (observed)", "BAD_PARAM")
    _num(p["vinit"], where + ".vinit")
    # recordings (14 units): Simulink converts vinit to the OUTPUT type first --
    # boolean: any non-zero is true; uint8: quantised to an integer -- and the step-0 test "previous <= 0" sees that
    # value; only its sign matters, so the delay starts at 1 or 0. recordings (
    # show): uint8 follows the parameter-overflow line of every other parameter -- Simulink stops
    # (fxpParameterOverflow) iff vinit <= -1 or > 256 (min - 1 LSB / max + 1 LSB); -0.3, -0.002, -0.006 run with a
    # precision-loss warning (quantised to 0), 15.875 / 127.99 run (positive)
    vi = Fraction(float(p["vinit"]))
    if p["OutDataTypeStr"] == "boolean":
        v0 = 1 if vi != 0 else 0
    else:
        if vi <= -1 or vi > 256:
            _refuse(where + ".vinit", "vinit %s does not fit the uint8 output: at or below min - 1 LSB (-1) or above "
                    "max + 1 LSB (256), Simulink stops (fxpParameterOverflow); give a value in -0.5 .. 256"
                    % p["vinit"], "BAD_PARAM")
        # the stored initial value is vinit rounded to the nearest uint8 (-0.7 and
        # -0.5 -> 0, 0.5 and 0.7 -> 1, precision-loss warning)
        v0 = 1 if vi >= Fraction(1, 2) else 0
    ic0 = str(v0)
    info["ic_grid"].append((X + "prev", ic0, None))
    u = src["u"]
    add[X + "prev"] = {"type": "UnitDelay", "InitialCondition": ic0, "SampleTime": "-1"}
    wire(u, X + "prev")
    boolean = _bool_source(info["_spec"], u[0])
    info.setdefault("drp", []).append((n, u, boolean))
    if boolean:
        # on a boolean signal "previous <= 0" is NOT previous and "input > 0" is the input itself
        add[X + "le"] = {"type": "LogicalOperator", "Operator": "NOT", "Inputs": "1"}
        wire(X + "prev", X + "le")
        gt = u
    else:
        add[X + "le"] = {"type": "CompareToConstant", "relop": "<=", "const": "0", "OutDataTypeStr": "boolean"}
        wire(X + "prev", X + "le")
        add[X + "gt"] = {"type": "CompareToConstant", "relop": ">", "const": "0", "OutDataTypeStr": "boolean"}
        wire(u, X + "gt")
        gt = X + "gt"
    last = n if p["OutDataTypeStr"] == "boolean" else X + "and"
    add[last] = {"type": "LogicalOperator", "Operator": "AND", "Inputs": "2"}
    wire(X + "le", last, 1)
    wire(gt, last, 2)
    if last != n:
        add[n] = {"type": "DataTypeConversion", "OutDataTypeStr": "uint8", "RndMeth": "Floor",
                  "SaturateOnIntegerOverflow": "off"}
        wire(last, n)


DLUT_DEFAULTS = {"NumberOfTableDimensions": "1", "InputsSelectThisObjectFromTable": "Element", "Table": "",
                 "TableDataTypeStr": "Inherit: Inherit from 'Table data'", "DiagnosticForOutOfRangeInput": "Warning",
                 "RemoveProtectionInput": "off", "SampleTime": "-1"}


def _dlut(n, b, X, src, add, wire, info, where):
    """Direct Lookup Table (n-D), Element (recordings dlut*): the zero-based index is clamped into the table
    (Warning / None: the index runs past the end -> the last element, a negative one -> the first); 1-D or 2-D
    (port 1 the row, port 2 the column); the index must be an integer type (fixed point with fraction bits is a
    Simulink error); the table type must be an explicit fixdt (an inherited one is double)."""
    p = _params(b, DLUT_DEFAULTS, where)
    if p["InputsSelectThisObjectFromTable"] != "Element":
        _refuse(where + ".InputsSelectThisObjectFromTable", "only Element is supported")
    if p["DiagnosticForOutOfRangeInput"] not in ("None", "Warning"):
        _refuse(where + ".DiagnosticForOutOfRangeInput", "%r: an out-of-range index stops the simulation in "
                "Simulink; only None / Warning (clamp) are supported" % p["DiagnosticForOutOfRangeInput"])
    if p["RemoveProtectionInput"] != "off":
        _refuse(where + ".RemoveProtectionInput", "only off (clamping) is observed")
    t = G._parse_fixdt(p["TableDataTypeStr"]) if not p["TableDataTypeStr"].startswith("Inherit") else None
    if t is None:
        named = G.fc._NAMED_INT.get(p["TableDataTypeStr"])
        t = named[:3] if named else None
    if t is None:
        _refuse(where + ".TableDataTypeStr", "%r: an inherited table type is double (no double signals); give an "
                "explicit fixdt" % p["TableDataTypeStr"])
    nd = _pint(p["NumberOfTableDimensions"], where + ".NumberOfTableDimensions")
    if nd not in (1, 2):
        # recordings (a 3-D Element table runs in Simulink): the reason is the dimension count
        _refuse(where + ".NumberOfTableDimensions", "a %d-D Direct Lookup Table is not supported here (1-D and 2-D "
                "tables were recorded)" % nd)
    txt = p["Table"].strip()
    if not (txt.startswith("[") and txt.endswith("]")):
        _refuse(where + ".Table", "numeric matrix literal required", "UNSUPPORTED_PARAM")
    rows = [[_num(v, where + ".Table") for v in r.replace(",", " ").split()] for r in txt[1:-1].split(";")]
    rows = [r for r in rows if r]
    if nd == 1:
        if len(rows) != 1 and not all(len(r) == 1 for r in rows):
            _refuse(where + ".Table", "a 1-D table must be a vector", "BAD_PARAM")
        rows = [[v for r in rows for v in r]]
    elif any(len(r) != len(rows[0]) for r in rows):
        _refuse(where + ".Table", "rows of different lengths", "BAD_PARAM")
    s_, w_, f_ = t
    lo, hi = (-(1 << (w_ - 1)), (1 << (w_ - 1)) - 1) if s_ else (0, (1 << w_) - 1)
    T = "fixdt(%d,%d,%d)" % t
    for r in rows:
        for v in r:
            q = Fraction(float(v)) * (Fraction(2) ** f_)
            if q.denominator != 1 or not lo <= q <= hi:
                _refuse(where + ".Table", "table value %s is not a value of %s (quantisation not observed)" % (v, T))

    def clamp(k, size):
        nm = X + "clamp%d" % k
        add[nm] = {"type": "Saturation", "UpperLimit": str(size - 1), "LowerLimit": "0", "RndMeth": "Floor",
                   "_Clamp": "on"}
        wire(src["idx%d" % k], nm)
        info.setdefault("int_index", []).append((n, "idx%d" % k, src["idx%d" % k]))
        return nm

    def mpsw(nm, ctrl, data):
        add[nm] = {"type": "MultiportSwitch", "DataPortOrder": "Zero-based contiguous", "Inputs": str(len(data)),
                   "DataPortForDefault": "Last data port", "OutDataTypeStr": T, "RndMeth": "Floor",
                   "SaturateOnIntegerOverflow": "off",
                   # the clamp past the end is the default port: no stop (recordings made a left-out one 'Error')
                   "DiagnosticForDefault": "None"}
        wire(ctrl, nm, 1)
        for j, d in enumerate(data):
            wire(d, nm, j + 2)
        return nm

    def const(nm, v):
        add[nm] = {"type": "Constant", "Value": repr(float(v)), "OutDataTypeStr": T, "SampleTime": "inf"}
        return nm
    if nd == 1:
        vals = rows[0]
        c1 = clamp(1, len(vals))
        if len(vals) == 1:
            _refuse(where + ".Table", "a one-element table (not observed)")
        mpsw(n, c1, [const(X + "t%d" % j, v) for j, v in enumerate(vals)])
    else:
        R, C = len(rows), len(rows[0])
        if R < 2 or C < 2:
            _refuse(where + ".Table", "a 2-D table with a single row or column (not observed)")
        cr, cc = clamp(1, R), clamp(2, C)
        cols = [mpsw(X + "col%d" % c, cr, [const(X + "t%d_%d" % (r, c), rows[r][c]) for r in range(R)])
                for c in range(C)]
        mpsw(n, cc, cols)


REPEAT_DEFAULTS = {"N": "5", "ic": "0", "InputProcessing": "Columns as channels (frame based)",
                   "RateOptions": "Enforce single-rate processing", "FactorSource": "Dialog parameter", "Nmax": "16",
                   "mode": "Unused parameter value"}


# the lowering's Upsample is sample based (the engine's long-standing meaning); written out, since recordings
# refuses an Upsample that leaves InputProcessing out (the library default is frame based)
_SAMPLE_BASED = "Elements as channels (sample based)"


def _repeat(n, b, X, src, add, wire, info, where):
    """Repeat, sample based with Allow multirate processing: output period P/N, each
    input value repeated N times starting at t = 0 with u[0]; the initial condition does not appear (single
    tasking). Frame-based processing (the library default) gives a vector output and is refused."""
    p = _params(b, REPEAT_DEFAULTS, where)
    # show: sample based, RateOptions has no effect (Enforce single-rate also runs at P/N, 5 /
    # 5 equal); frame based with Allow multirate processing on a scalar is the same as sample based (the vector pass
    # refuses it on a vector); frame based single rate (the library default) outputs a vector of N -- refused
    frame = p["InputProcessing"] != "Elements as channels (sample based)"
    if frame and p["RateOptions"] != "Allow multirate processing":
        _refuse(where + ".InputProcessing", "Repeat frame based with 'Enforce single-rate processing' (the library "
                "default) outputs a vector of N samples per input sample, which is not supported; use sample based "
                "(Elements as channels) or Allow multirate processing")
    if p["FactorSource"] != "Dialog parameter":
        _refuse(where + ".FactorSource", "repeat count from the dialog only")
    k = int(_num(p["N"], where + ".N"))
    if str(k) != p["N"].strip() or not 2 <= k <= 1 << 16:
        _refuse(where + ".N", "integer repeat count 2.. required", "BAD_PARAM")
    # Upsample inserts zeros between the input samples; a Hold enabled by an upsampled 'true' keeps each input
    # value for N output samples, starting with u[0] at t = 0 (a Rate Transition would delay by one input period)
    u = src["u"]
    # the IC is not used (single tasking) but Simulink still quantises it into the input type and stops on
    # an overflow (ic -1 on uint8: fxpParameterOverflow) -- checked once the input type is known (post_compile)
    info.setdefault("repeat_ic", []).append((n, p["ic"], u))
    add[X + "false"] = {"type": "RelationalOperator", "Operator": "~="}
    wire(u, X + "false", 1)
    wire(u, X + "false", 2)
    add[X + "true"] = {"type": "LogicalOperator", "Operator": "NOT", "Inputs": "1"}
    wire(X + "false", X + "true")
    add[X + "upx"] = {"type": "Upsample", "UpsamplingFactor": str(k), "InputProcessing": _SAMPLE_BASED}
    wire(u, X + "upx")
    add[X + "upen"] = {"type": "Upsample", "UpsamplingFactor": str(k), "InputProcessing": _SAMPLE_BASED}
    wire(X + "true", X + "upen")
    add[n] = {"type": "Hold", "InitialCondition": "0"}
    wire(X + "upx", n, 1)
    wire(X + "upen", n, 2)


def pre_compile_checks(d, info, in_types):
    """Before type resolution: a Detect Rise Positive fed straight from a design input whose type contradicts the
    structural guess gets a clear refusal (the lowering is chosen before the input types are known)."""
    for n, p, x, y in info.get("cplx_exact", []):
        ta, tb = d.otype.get(_port_block(info, x[0], x[1], "design.blocks." + n)), \
            d.otype.get(_port_block(info, y[0], y[1], "design.blocks." + n))
        tp = d.otype.get(p)
        if None in (ta, tb, tp) or G.BOOL in (ta, tb):
            continue
        if tp[2] != ta[2] + tb[2] or tp[1] - tp[0] < (ta[1] - ta[0]) + (tb[1] - tb[0]) + (ta[0] or tb[0]):
            _refuse("design.blocks." + n, "complex Product whose partial products %s x %s are rounded or cut to %s "
                    "(only exact partial products are observed)" % (G._tstr(ta), G._tstr(tb), G._tstr(tp)))
    for n, (s, sp), boolean in info.get("drp", []):
        if s in in_types and (in_types[s] == G.BOOL) != boolean:
            _refuse("design.blocks." + n, "Detect Rise Positive fed straight from the boolean design input %s is not "
                    "supported; route it through a logic block (e.g. NOT(NOT x)), which makes it structurally "
                    "boolean" % s)


def _ram(n, b, X, src, add, wire, info, where):
    """HDL RAMs (the masked and the System versions behave alike): 2^AddressBits words; a word
    is written when the write enable is true; every output is registered (the read address of step t gives data at
    t + 1; RAMInitialValue before that and in unwritten words). Simple dual port / Dual port read port: the old value
    on a same-step read and write of one address; Dual port write-port output (port 1) and Single port: New data or
    Old data as set (WriteOutputValue; recordings show Old data on the Dual port)."""
    p = _params(b, RAM_DEFAULTS, where, extra_ok=("_BoolData",))
    booldata = p.pop("_BoolData", "off") == "on"
    rt = p["RAMType"]
    if p["AddressBits"].strip() == RAM_AUTO:
        _refuse(where + ".AddressBits", "inherited address width not resolved", "INTERNAL")
    try:
        A = _pint(p["AddressBits"], where + ".AddressBits")
    except ValueError:
        _refuse(where + ".AddressBits", "integer number of address bits required", "BAD_PARAM")
    if A not in RAM_ADDR_BITS:
        # recordings, refuse what is not recorded: recordings show AddressBits 2, 3 and 4, recordings
        # 5 and 8
        _refuse(where + ".AddressBits", 'address width %d: recorded %d.. %d only ('
                "Simulink refuses fewer than 2 bits)" % (A, RAM_ADDR_BITS[0], RAM_ADDR_BITS[-1]),
                "BAD_PARAM" if A < 2 else "UNSUPPORTED")
    if p["WriteOutputValue"] not in ("New data", "Old data"):
        _refuse(where + ".WriteOutputValue", "New data or Old data", "BAD_PARAM")
    init = p["RAMInitialValue"]
    _num(init, where + ".RAMInitialValue")
    W = 1 << A
    if rt == "Single port":
        din, waddr, we, raddr = src["din"], src["addr"], src["we"], src["addr"]
    else:
        din, waddr, we, raddr = src["wr_din"], src["wr_addr"], src["wr_en"], src["rd_addr"]
    info.setdefault("ram", []).append((n, A, din, [waddr] + ([raddr] if raddr != waddr else []), we))
    if A in RAM_DEEP_ADDR:
        if booldata and (A, "boolean") not in RAM_DEEP_DATA:
            _refuse(where, 'a RAM with %d address bits and boolean data: recorded with %s address bits only'
                    "" % (A, ", ".join(str(a) for a, t in sorted(RAM_DEEP_DATA, key=str)
                                                               if t == "boolean")))
        if booldata and _norm_num(init) != 0:
            _refuse(where + ".RAMInitialValue", "a deep RAM of boolean data was recorded with initial value 0 only")
        # the data type and the initial value are checked once the types are known (typed check below)
        info.setdefault("ram_deep", {})[n] = (A, init, rt, booldata)
        if booldata:   # stored as uint8 0 / 1, as the per-address lowering does
            add[X + "b2u"] = {"type": "DataTypeConversion", "OutDataTypeStr": "uint8", "RndMeth": "Floor",
                              "SaturateOnIntegerOverflow": "off"}
            wire(din, X + "b2u")
            din = X + "b2u"
        _ram_deep(n, p, X, A, din, waddr, we, raddr, add, wire, info, init, booldata)
        return
    if booldata:   # stored as uint8 0 / 1 (ram_bool_mark)
        add[X + "b2u"] = {"type": "DataTypeConversion", "OutDataTypeStr": "uint8", "RndMeth": "Floor",
                          "SaturateOnIntegerOverflow": "off"}
        wire(din, X + "b2u")
        din = X + "b2u"
    info["ic_grid"].append((X + "w0", init, None))
    info.setdefault("ram_ic", set()).add(X + "w0")
    regs = []
    for k in range(W):
        eq, sel, wsel, reg = X + "eq%d" % k, X + "sel%d" % k, X + "wsel%d" % k, X + "w%d" % k
        add[eq] = {"type": "CompareToConstant", "relop": "==", "const": str(k), "OutDataTypeStr": "boolean"}
        wire(waddr, eq)
        add[wsel] = {"type": "LogicalOperator", "Operator": "AND", "Inputs": "2"}
        wire(we, wsel, 1)
        wire(eq, wsel, 2)
        # the word keeps the last value written to it: a Hold of the data on its write select (typed by the data,
        # so the register loop needs no inherited-type seed)
        add[sel] = {"type": "Hold", "InitialCondition": init}
        wire(din, sel, 1)
        wire(wsel, sel, 2)
        add[reg] = {"type": "UnitDelay", "InitialCondition": init, "SampleTime": "-1"}
        wire(sel, reg)
        regs.append(reg)

    def read(tag, addr):
        nm = X + "mux_" + tag
        add[nm] = {"type": "MultiportSwitch", "DataPortOrder": "Zero-based contiguous", "Inputs": str(W),
                   "DataPortForDefault": "Last data port", "OutDataTypeStr": G.fc._INHERIT_RULE, "RndMeth": "Floor",
                   "SaturateOnIntegerOverflow": "off", "DiagnosticForDefault": "None"}   # the address is in range
        wire(addr, nm, 1)
        for k, r in enumerate(regs):
            wire(r, nm, k + 2)
        return nm

    def newdata(tag, addr):
        nm = X + "new_" + tag
        add[nm] = {"type": "Switch", "Criteria": "u2 ~= 0", "OutDataTypeStr": G.fc._SAME_FIRST}
        wire(din, nm, 1)
        wire(we, nm, 2)
        wire(read(tag, addr), nm, 3)
        return nm

    def out(name, v):
        if booldata:   # back to boolean: the stored 0 / 1 compared with 0
            reg = name + "/u8" if name.startswith(INTERNAL) else X + "u8_" + name.split("/")[-1]
            add[reg] = {"type": "UnitDelay", "InitialCondition": init, "SampleTime": "-1"}
            wire(v, reg)
            add[name] = {"type": "CompareToConstant", "relop": "~=", "const": "0", "OutDataTypeStr": "boolean"}
            wire(reg, name)
            return
        add[name] = {"type": "UnitDelay", "InitialCondition": init, "SampleTime": "-1"}
        wire(v, name)
    if rt == "Simple dual port":
        out(n, read("rd", raddr))
    elif rt == "Dual port":
        # the write port's output follows WriteOutputValue (Old
        # data gives the address's previous value; the masked Dual Port RAM, without the parameter, gives new data)
        out(n, newdata("wr", waddr) if p["WriteOutputValue"] == "New data" else read("wr", waddr))
        out(X + "rd_dout", read("rd", raddr))
        info.setdefault("port2", {})[n] = X + "rd_dout"
    else:
        out(n, newdata("sp", waddr) if p["WriteOutputValue"] == "New data" else read("sp", waddr))


def _norm_num(v):
    try:
        return float(str(v).strip())
    except ValueError:
        return None


def _ram_deep(n, p, X, A, din, waddr, we, raddr, add, wire, info, init="0", booldata=False):
    """A deep RAM (10 / 16 address bits) as one internal RAMCore per output, each followed by the
    output register: the core gives, at step t, the value the port shows at t + 1 (read port: the word before this
    step's write; write port: New data / Old data as set) and then writes; the UnitDelay after it is the registered
    output, so a feedback loop through the RAM still passes a Unit Delay."""
    rt = p["RAMType"]

    def core(tag, mode, addr, name):
        c = X + "core_" + tag
        add[c] = {"type": "RAMCore", "Mode": mode, "A": str(A), "Init": str(init).strip(), "Owner": n}
        wire(din, c, 1)
        wire(waddr, c, 2)
        wire(we, c, 3)
        wire(addr, c, 4)
        if booldata:   # back to boolean: the stored 0 / 1 compared with 0
            reg = name + "/u8" if name.startswith(INTERNAL) else X + "u8_" + name.split("/")[-1]
            add[reg] = {"type": "UnitDelay", "InitialCondition": "0", "SampleTime": "-1"}
            wire(c, reg)
            add[name] = {"type": "CompareToConstant", "relop": "~=", "const": "0", "OutDataTypeStr": "boolean"}
            wire(reg, name)
            return
        add[name] = {"type": "UnitDelay", "InitialCondition": str(init).strip(), "SampleTime": "-1"}
        wire(c, name)
    if p["WriteOutputValue"] != "New data" and rt != "Simple dual port" and (A, rt) not in RAM_DEEP_OLD:
        # review (deep RAM): recordings show the deep RAMs with New data only; recordings show Old data
        # on the Single Port RAM with 9 and 13 address bits
        _refuse("design.blocks." + n + ".WriteOutputValue", "a RAM with %d address bits was recorded with "
                "WriteOutputValue New data only%s" % (
                    A, "; Old data on a Single port RAM with %s bits" % " / ".join(
                        str(a) for a, _ in sorted(RAM_DEEP_OLD))))
    wmode = "new" if p["WriteOutputValue"] == "New data" else "old"
    if rt == "Simple dual port":
        core("rd", "read", raddr, n)
    elif rt == "Dual port":
        core("wr", wmode, waddr, n)
        core("rd", "read", raddr, X + "rd_dout")
        info.setdefault("port2", {})[n] = X + "rd_dout"
    else:
        core("sp", wmode, waddr, n)


class RamKernel:
    """RAMCore: inputs din, write address, write enable, port address; output the value the port shows next step."""

    def __init__(self, p, ts, where):
        from fxgraph_lib import BOOL
        from fractions import Fraction
        self.mode, self.A, self.where = p.get("Mode", "read"), int(p.get("A", "10")), where
        if ts[2] != BOOL:
            _refuse(where, "RAM write enable of type %s (boolean observed)" % (ts[2],))
        # the initial value as a stored integer of the data type; exact only (3 in sfix16_En8 = 768)
        q = Fraction(float(str(p.get("Init", "0")).strip() or "0")) * (Fraction(2) ** ts[0][2])   # a double
        if q.denominator != 1:
            _refuse(where, "RAM initial value %s is not exact in the data type (not recorded)" % p.get("Init"))
        self.init = int(q)
        s_, w_, _f = ts[0]
        lo_, hi_ = (-(1 << (w_ - 1)), (1 << (w_ - 1)) - 1) if s_ else (0, (1 << w_) - 1)
        if not lo_ <= self.init <= hi_:
            # Simulink accepts it and saturates silently (a System RAM); not computed here
            _refuse(where + ".RAMInitialValue", "RAM initial value %s is outside the data type %s: Simulink saturates "
                    "it silently in a System RAM where the parameter-overflow diagnostic allows; not "
                    "computed here -- write a value the type holds" % (p.get("Init"), ts[0]))
        self.reset()

    def reset(self):
        self.mem = {}

    def __call__(self, v):
        din, wa, we, pa = v[0], v[1], v[2], v[3]
        old = self.mem.get(pa, self.init)
        out = old if self.mode in ("read", "old") else (din if we else old)
        if we:
            self.mem[wa] = din
        return out


def ram_out_type(p, ts, where):
    return ts[0]


def ram_kernel(p, ts, where):
    return RamKernel(p, ts, _owner_where(p, where))


# ---------------------------------------------------------------- complex signals
def _cnum(v, where):
    """A complex literal 'a+bi' / 'a-bi' / 'bi' / 'a' (i or j) -> (re, im) texts."""
    import re
    t = str(v).strip().replace(" ", "")
    m = re.match(r"^([-+]?[0-9.]+(?:[eE][-+]?\d+)?)?(?:([-+][0-9.]*(?:[eE][-+]?\d+)?)[ij])?$", t)
    if not m or not t:
        m2 = re.match(r"^([-+]?[0-9.]*(?:[eE][-+]?\d+)?)[ij]$", t)
        if m2:
            im = m2.group(1) or "1"
            return "0", ("1" if im == "+" else "-1" if im == "-" else im)
        raise G.GraphError("UNSUPPORTED_PARAM", where, "numeric (complex) literal required, got %r" % (v,))
    re_ = m.group(1) or "0"
    im = m.group(2)
    if im in ("+", "-"):
        im += "1"
    return re_, (im or "0")


_CPLX_CALL = re.compile(r"^\s*complex\s*\(\s*([^,()]+?)\s*(?:,\s*([^,()]+?)\s*)?\)\s*$")


def _cplx_value(v):
    """(re, im) texts of a complex Constant value -- complex(a), complex(a, b), a+bi, bi -- or None for a real one."""
    t = str(v).strip()
    m = _CPLX_CALL.match(t)
    if m:
        return m.group(1), (m.group(2) or "0")
    if re.search(r"[0-9.]\s*[ij]\s*$", t) or re.match(r"^\s*[-+]?[ij]\s*$", t):
        try:
            return _cnum(t, "Value")
        except G.GraphError:
            return None
    return None


def _cplx_const(spec):
    """recordings (27 models): a complex Constant (complex(0), complex(1), complex(a, b), a+bi, bi) is a complex
    signal -- complex(0) too -- whose parts take the block's type; with an explicit fixdt it is two real Constants
    into a Real-Imag to Complex (named as the Constant); with an inherited type Simulink makes it complex double,
    which the engine does not simulate (refused)."""
    blocks = spec.get("blocks") or {}
    hits = [n for n, b in blocks.items() if isinstance(b, dict) and b.get("type") == "Constant"
            and _cplx_value(b.get("Value", "")) is not None]
    if not hits:
        return spec
    blocks = dict(blocks)
    for n in hits:
        b = blocks[n]
        re_, im_ = _cplx_value(b["Value"])
        od = str(b.get("OutDataTypeStr", "Inherit: Inherit from 'Constant value'")).strip()
        if G._parse_fixdt(od) is None:
            _refuse("design.blocks.%s.OutDataTypeStr" % n, "a complex Constant with OutDataTypeStr %r: Simulink "
                    "makes it complex double, which is not simulated; give it an explicit fixdt" % od)
        X = INTERNAL + n + "/cplx/"
        extra = {k: v for k, v in b.items() if k in ("SampleTime",)}
        blocks[X + "re"] = dict({"type": "Constant", "Value": re_, "OutDataTypeStr": od}, **extra)
        blocks[X + "im"] = dict({"type": "Constant", "Value": im_, "OutDataTypeStr": od}, **extra)
        blocks[n] = {"type": "RealImagToComplex", "Input": "Real and imag"}
    lines = list(spec.get("lines", []))
    for n in hits:
        X = INTERNAL + n + "/cplx/"
        lines += [{"src": X + "re", "dst": n, "dport": 1}, {"src": X + "im", "dst": n, "dport": 2}]
    return dict(spec, blocks=blocks, lines=lines)


def _cplx_recorded(b, where, rows, src):
    """a block on a complex signal only with the settings recorded."""
    for k, default, ok in rows:
        v = str(b.get(k, default)).strip()
        if v not in ok:
            _refuse(where + "." + k, "%s %s on a complex signal: only %s is recorded (%s)"
                    % (k, v, " / ".join(ok), src))


# complex part types recorded (recordings sfix16_En8, recordings NCO sfix16_En14, recordings conj sfix12_En4 / sfix16_En8)
CPLX_MAX_WL = 16


def complex_consumer_check(spec):
    """recordings (class: a complex signal into a block the complex pass does not take): the refusal used to come
    from a later pass with a reason that is not the cause (Bias: 'its type is not known before its rewrite'; Tapped
    Delay / Bus Assignment: 'the compiled source could not be traced'). Refused here, saying so."""
    blocks = spec.get("blocks") if isinstance(spec, dict) else None
    if not isinstance(blocks, dict) or not any(isinstance(b, dict) and b.get("type") in (
            "RealImagToComplex", "MagnitudeAngleToComplex") for b in blocks.values()):
        return
    OK = ("Gain", "Sum", "Product", "UnitDelay", "Math", "Mux", "Concatenate")
    TAKES = OK + ("ComplexToRealImag", "ComplexToMagnitudeAngle", "Outport", "Demux", "Selector", "BusCreator",
                  "BusSelector", "RealImagToComplex", "MagnitudeAngleToComplex")
    lines = [l for l in spec.get("lines", []) if isinstance(l, dict)]
    src = {}
    for l in lines:
        try:
            src[(l["dst"], int(l.get("dport", 1)))] = l["src"]
        except (KeyError, TypeError, ValueError):
            return
    cplx = {n for n, b in blocks.items() if isinstance(b, dict) and b.get("type") in (
        "RealImagToComplex", "MagnitudeAngleToComplex")}
    changed = True
    while changed:
        changed = False
        for n, b in blocks.items():
            if n in cplx or not isinstance(b, dict) or b.get("type") not in OK + ("BusCreator",):
                continue
            if b.get("type") == "Math" and str(b.get("Operator", "exp")).strip() != "conj":
                continue
            if any(src.get((n, k)) in cplx for k in range(1, 65)):
                cplx.add(n)
                changed = True
    for (d, k), s_ in sorted(src.items()):
        t = (blocks.get(d) or {}).get("type") if isinstance(blocks.get(d), dict) else None
        if s_ in cplx and t is not None and t not in TAKES:
            raise G.GraphError("UNSUPPORTED", "design.blocks." + d, "a complex signal (from %s) into a %s block: complex "
                               "signals are supported through Gain, Sum, Product, Unit Delay, conj, Mux / Concatenate and "
                               "the complex conversion blocks only" % (s_, t))


def complex_mix_check(spec):
    """recordings: a complex and a real signal into one Mux / Vector Concatenate -- Simulink stops
    (InputPortComplexSignalMismatch); the vector pass ran first and answered element by element. Called before it."""
    blocks = spec.get("blocks") if isinstance(spec, dict) else None
    if not isinstance(blocks, dict) or not any(isinstance(b, dict) and b.get("type") in (
            "RealImagToComplex", "MagnitudeAngleToComplex") for b in blocks.values()):
        return
    lines = [l for l in spec.get("lines", []) if isinstance(l, dict)]
    src = {}
    for l in lines:
        try:
            src[(l["dst"], int(l.get("dport", 1)))] = l["src"]
        except (KeyError, TypeError, ValueError):
            return
    OK = ("Gain", "Sum", "Product", "UnitDelay", "Math", "Mux", "Concatenate")
    cplx = {n for n, b in blocks.items() if isinstance(b, dict) and b.get("type") in (
        "RealImagToComplex", "MagnitudeAngleToComplex")}
    changed = True
    while changed:
        changed = False
        for n, b in blocks.items():
            if n in cplx or not isinstance(b, dict) or b.get("type") not in OK:
                continue
            if b.get("type") == "Math" and str(b.get("Operator", "exp")).strip() != "conj":
                continue
            if any(src.get((n, k)) in cplx for k in range(1, 65)):
                cplx.add(n)
                changed = True
    for n, b in blocks.items():
        if isinstance(b, dict) and b.get("type") in ("Mux", "Concatenate"):
            ins = [src[(n, k)] for k in range(1, 65) if (n, k) in src]
            kinds = {x in cplx for x in ins}
            if len(kinds) == 2:
                raise G.GraphError("PORT_TYPE_INVALID", "design.blocks." + n, "%s %s mixes complex (%s) and real (%s) "
                                   "inputs; Simulink rejects it (InputPortComplexSignalMismatch)" % (
                                       b["type"], n, ", ".join(x for x in ins if x in cplx),
                                       ", ".join(x for x in ins if x not in cplx)))


def _complex(spec):
    """Complex signals are carried as a real part and an imaginary part (two real signals). Rules (
    every recorded step): Real-Imag to Complex / Complex to Real-Imag are wiring; a Unit Delay delays both parts; a
    real Gain multiplies both parts; a complex Gain gr+gi*i forms the four partial products, each rounded to the
    output type, then re = a*gr - b*gi, im = a*gi + b*gr; a complex Sum adds part by part; a complex Product of two
    complex inputs is re = a*c - b*d, im = a*d + b*c (observed with products that are exact in the output type, which
    is checked). Anything else on a complex signal is refused (a real block cannot take a complex input)."""
    blocks = spec["blocks"]
    lines = spec.get("lines", [])
    src = {(l["dst"], int(l.get("dport", 1))): (l["src"], int(l.get("sport", 1))) for l in lines}
    users = {}
    for l in lines:
        users.setdefault(l["src"], []).append(l)
    OK = ("Gain", "Sum", "Product", "UnitDelay", "Math")
    cplx = {n for n, b in blocks.items() if b.get("type") in ("RealImagToComplex", "MagnitudeAngleToComplex")}
    changed = True
    while changed:
        changed = False
        for n, b in blocks.items():
            if n in cplx or b.get("type") not in OK:
                continue
            if b.get("type") == "Math" and str(b.get("Operator", "exp")).strip() != "conj":
                continue
            if any(src.get((n, k), (None,))[0] in cplx for k in range(1, 9)):
                cplx.add(n)
                changed = True
    # recordings (class: the own SampleTime of every block that carries a complex signal): conj / Gain / Sum /
    # Product / Unit Delay / the CORDIC blocks were run at the input's period (Simulink runs them at their own)
    for n in sorted(cplx) + [x for x, b in blocks.items() if b.get("type") in ("ComplexToRealImag",
                                                                               "ComplexToMagnitudeAngle")]:
        st_ = str(blocks[n].get("SampleTime", "-1")).strip()
        if blocks[n].get("type") == "UnitDelay":
            # recordings (G4 ud_ic / G4C chains at SampleTime 1, recordings at 2 into a complex Sum): each
            # part is a real Unit Delay with the block's parameters, its own sample time included (the real
            # multirate rules)
            continue
        if st_ not in ("-1", ""):
            _refuse("design.blocks.%s.SampleTime" % n, "%s %s carries a complex signal with its own SampleTime %s: the "
                    "block's own sample time is not modelled for complex signals; leave it -1 (inherited) and set the "
                    "rate upstream" % (blocks[n].get("type"), n, st_))
    # shapes Simulink refuses that the engine had answered -- an Outport whose SignalType says the other
    # complexity (InputPortComplexSignalMismatch), a complex conj with OutputSignalType real
    # (PortComplexSignalMismatch); and a complex Inport, which the engine cannot take (it has no complex inputs)
    for n, b in blocks.items():
        t_ = b.get("type")
        st_ = str(b.get("SignalType", "auto")).strip()
        if t_ == "Outport" and st_ in ("real", "complex"):
            fed = src.get((n, 1), (None,))[0] in cplx
            if (st_ == "complex") != fed:
                _refuse("design.blocks.%s.SignalType" % n, "Outport SignalType %s on a %s signal (Simulink: "
                        "InputPortComplexSignalMismatch)" % (st_, "complex" if fed else "real"), "TYPE_MISMATCH")
        if t_ == "Inport" and st_ == "complex":
            _refuse("design.blocks.%s.SignalType" % n, "a complex Inport (complex signals are formed inside the design "
                    "with Real-Imag to Complex)")
        if t_ == "Math" and str(b.get("Operator", "")).strip() == "conj" and n in cplx and \
                str(b.get("OutputSignalType", "auto")).strip() == "real":
            _refuse("design.blocks.%s.OutputSignalType" % n, "conj of a complex signal with OutputSignalType real "
                    "(Simulink: PortComplexSignalMismatch)", "TYPE_MISMATCH")
    # a complex signal into an Outport: two real Outports <name>.re / <name>.im (engine Outports are real)
    split_out = {}
    for n in cplx:
        for l in list(users.get(n, [])):
            y = l["dst"]
            if blocks.get(y, {}).get("type") == "Outport" and y not in split_out:
                for k in (".re", ".im"):
                    if y + k in blocks:
                        _refuse("design.blocks." + y, "complex Outport %s: the name %s is taken" % (y, y + k),
                                "BAD_PARAM")
                c2 = INTERNAL + y + "/c2ri"
                blocks = dict(blocks)
                del blocks[y]
                blocks[c2] = {"type": "ComplexToRealImag"}
                blocks[y + ".re"] = {"type": "Outport"}
                blocks[y + ".im"] = {"type": "Outport"}
                lines = [dict(x, dst=c2) if x is l else x for x in lines] + [
                    {"src": c2, "dst": y + ".re"}, {"src": c2, "sport": 2, "dst": y + ".im"}]
                split_out[y] = [y + ".re", y + ".im"]
    if split_out:
        spec = dict(spec, blocks=blocks, lines=lines)
        src = {(l["dst"], int(l.get("dport", 1))): (l["src"], int(l.get("sport", 1))) for l in lines}
        users = {}
        for l in lines:
            users.setdefault(l["src"], []).append(l)
    for n in cplx:
        for l in users.get(n, []):
            t = blocks[l["dst"]].get("type")
            if l["dst"] not in cplx and t not in ("ComplexToRealImag", "ComplexToMagnitudeAngle"):
                raise G.GraphError("UNSUPPORTED", "design.blocks." + l["dst"], "a complex signal (%s) into a %s block "
                                   "(only Gain, Sum, Product, Unit Delay and Complex to Real-Imag are supported)" % (n, t))
    for n, b in blocks.items():
        t_ = b.get("type")
        if t_ == "RealImagToComplex" and str(b.get("Input", "Real and imag")) != "Real and imag":
            _refuse("design.blocks.%s.Input" % n, "Real-Imag to Complex %r (only 'Real and imag')" % b.get("Input"))
        if t_ == "ComplexToRealImag" and str(b.get("Output", "Real and imag")) not in C2RI_PARTS:
            # 'Real' / 'Imag' give one output port, the part as it is
            _refuse("design.blocks.%s.Output" % n, "Complex to Real-Imag %r ('Real and imag', 'Real' or 'Imag')"
                    % b.get("Output"), "BAD_PARAM")
        for k in b:
            if t_ in CPLX_TYPES and k not in ("type", "Input", "Output", "ConstantPart", "SampleTime"):
                _refuse("design.blocks.%s.%s" % (n, k), "unknown parameter %s for %s" % (k, t_), "BAD_PARAM")
    for n, b in blocks.items():
        if b.get("type") == "ComplexToRealImag" and src.get((n, 1), (None,))[0] not in cplx:
            raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "Complex to Real-Imag of a real signal")
    # every line must use a port the block has -- Real-Imag to Complex: inputs 1 and 2, output 1;
    # Complex to Real-Imag: input 1, outputs 1 and 2; any other complex block: output 1
    for ln in lines:
        s_, sp = ln.get("src"), int(ln.get("sport", 1))
        d_, dp = ln.get("dst"), int(ln.get("dport", 1))
        ts_, td_ = blocks.get(s_, {}).get("type"), blocks.get(d_, {}).get("type")
        if (ts_ == "ComplexToRealImag" and not 1 <= sp <= len(_c2ri_parts(blocks[s_]))) or (s_ in cplx and sp != 1):
            raise G.GraphError("BAD_CONNECTION", "design.lines", "output port %d of %s does not exist" % (sp, s_))
        if (td_ == "RealImagToComplex" and dp not in (1, 2)) or (td_ == "ComplexToRealImag" and dp != 1):
            raise G.GraphError("BAD_CONNECTION", "design.lines", "input port %d of %s does not exist" % (dp, d_))
    for n in cplx:
        if blocks[n].get("type") == "RealImagToComplex" and not ((n, 1) in src and (n, 2) in src):
            # Simulink grounds an unwired input and runs (it was called a request error)
            raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "Real-Imag to Complex %s: input port %d not "
                               "connected; Simulink grounds an unconnected input (0) and runs, which is not supported "
                               "here -- connect both inputs (a Constant 0 where a zero part is meant)"
                               % (n, 1 if (n, 1) not in src else 2))
    add, new_lines, parts, exact = {}, [], {}, []
    pairs = []   # (Real-Imag to Complex, real source, imaginary source): one complex type (checked after compile)
    sums = []    # (complex Sum outside the recordings shape, its inputs' real parts): recordings (after compile)
    post = []    # checks after the compile (_cplx_114_post): Unit Delay ICs, saturating Gains

    def W(a, b2, port=1):
        new_lines.append({"src": a[0], "sport": a[1], "dst": b2, "dport": port} if a[1] != 1 else
                         {"src": a[0], "dst": b2, "dport": port})

    def part(s_, k):
        """(source, port) of part k (0 re, 1 im) of the complex signal from (block, port) s_; a real one as is (a
        Complex to Real-Imag output is the part of its input it carries: recordings straight into RI2C)."""
        if s_[0] in cplx:
            return parts[s_[0]][k]
        if blocks.get(s_[0], {}).get("type") == "ComplexToMagnitudeAngle" and s_[1] in (1, 2):
            # a Complex to Magnitude-Angle output straight into a Magnitude-Angle to Complex --
            # its magnitude / angle core (made below under the block's internal prefix)
            return (INTERNAL + s_[0] + "/" + ("mag", "ang")[s_[1] - 1], 1) if k == 0 else None
        if blocks.get(s_[0], {}).get("type") == "ComplexToRealImag" and (s_[0], 1) in src:
            r_ = part(src[(s_[0], 1)], _c2ri_parts(blocks[s_[0]])[s_[1] - 1])
            return r_ if k == 0 else None
        return s_ if k == 0 else None

    def ready(x):
        if x[0] in cplx:
            return x[0] in parts
        if blocks.get(x[0], {}).get("type") == "ComplexToRealImag" and (x[0], 1) in src:
            return ready(src[(x[0], 1)])
        return True
    order = []
    pending = set(cplx)
    # Unit Delays first (they may sit in loops), then the rest in dependency order
    for n in sorted(cplx):
        if blocks[n]["type"] == "UnitDelay":
            X = INTERNAL + n + "/"
            parts[n] = ((X + "re", 1), (X + "im", 1))
    guard = 0
    while pending and guard < 10 * len(cplx) + 10:
        guard += 1
        for n in sorted(pending):
            b = blocks[n]
            t = b["type"]
            sin_ = str(b.get("Inputs", "++")).replace("|", "").strip()
            # a Sum's Inputs may be a count ("2"), not only a sign list
            nin = 2 if t in ("RealImagToComplex", "Product", "MagnitudeAngleToComplex") else (
                (int(sin_) if sin_.isdigit() else len(sin_)) if t == "Sum" else 1)
            if t == "MagnitudeAngleToComplex":
                import fxcordic
                fxcordic.params(n, b)   # recordings (a): Input 'Magnitude' / 'Angle' refused as not recorded
            ins = [src.get((n, k)) for k in range(1, nin + 1)]
            if any(x is None for x in ins):
                if t == "MagnitudeAngleToComplex" or (t == "RealImagToComplex" and ins[0] is not None):
                    # recordings (b): Simulink grounds the unwired port and runs
                    raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "%s %s: input port %d not connected; "
                                       "Simulink grounds an unconnected input (0) and runs, which is not supported "
                                       "here -- connect every input" % (t, n, ins.index(None) + 1))
                raise G.GraphError("BAD_CONNECTION", "design.blocks." + n, "an input is not connected")
            if not all(ready(x) for x in ins):
                continue
            pending.discard(n)
            X = INTERNAL + n + "/"
            where = "design.blocks." + n
            if t == "RealImagToComplex":
                for k_, x_ in enumerate(ins):
                    if x_[0] in cplx:
                        # recordings (Simulink stops, the engine took the real part): the inputs of
                        # Real-Imag to Complex are real
                        raise G.GraphError("PORT_TYPE_INVALID", where, "Real-Imag to Complex %s input %d is a complex "
                                           "signal (from %s); Simulink rejects it (InputPortComplexSignalMismatch: the "
                                           "inputs must be real)" % (n, k_ + 1, x_[0]))
                re_, im_ = part(ins[0], 0), part(ins[1], 0)
                parts[n] = (re_, im_)
                pairs.append((n, re_, im_))
            elif t == "MagnitudeAngleToComplex":   # recordings (engine/fxcordic.py): two CORDIC rotations
                import fxcordic
                p_ = fxcordic.params(n, b)
                for k_ in ("re", "im"):
                    add[X + k_] = fxcordic.core("ma2c", k_, p_)
                    W(part(ins[0], 0), X + k_, 1)
                    W(part(ins[1], 0), X + k_, 2)
                parts[n] = ((X + "re", 1), (X + "im", 1))
            elif t == "Math":   # conj: the imaginary part negated, its type kept,
                # saturating or wrapping by SaturateOnIntegerOverflow (default on); OutDataTypeStr has no effect
                # recordings (class: the complex pass read only the parameters it uses): every
                # parameter goes through the real Math path's check -- an unknown parameter, IntermediateResultsDataTypeStr
                # included (Simulink rejects fixdt(1,4,0) at set_param), was ignored
                __import__("block_lowering").params(n, {k_: v_ for k_, v_ in b.items() if k_ not in ("OutputSignalType",
                                                                                              "SampleTime")})
                sat = str(b.get("SaturateOnIntegerOverflow", "on")).strip()
                if sat not in ("on", "off"):
                    _refuse(where + ".SaturateOnIntegerOverflow", "on or off", "BAD_PARAM")
                add[X + "im"] = {"type": "UnaryMinus", "SaturateOnIntegerOverflow": sat}
                W(part(ins[0], 1), X + "im")
                parts[n] = (part(ins[0], 0), (X + "im", 1))
            elif t == "UnitDelay":
                if ins[0][0] not in cplx:
                    _refuse(where, "internal: a complex Unit Delay of a real signal")
                ic_re, ic_im = _cnum(b.get("InitialCondition", "0"), where + ".InitialCondition")
                if Fraction(ic_re) != 0 or Fraction(ic_im) != 0:
                    # recordings (G4 ud_ic, G4C three-delay chains): each part starts at its own part of the
                    # initial condition -- recorded on unsigned 12-32-bit and signed 34-64-bit parts with values exact
                    # in the part type (checked after the compile, _cplx_114_post)
                    post.append(("ud_ic", n, (X + "re", X + "im"), (Fraction(ic_re), Fraction(ic_im))))
                for k, (nm, ic) in enumerate(((X + "re", ic_re), (X + "im", ic_im))):
                    add[nm] = dict(b, InitialCondition=ic)
                    W(part(ins[0], k), nm)
            elif t == "Gain":
                gr, gi = _cnum(b.get("Gain", "1"), where + ".Gain")
                inh_p = str(b.get("ParamDataTypeStr", G.fc._INHERIT_RULE)).strip() == G.fc._INHERIT_RULE
                if Fraction(gi) == 0 or not inh_p:
                    _cplx_recorded(b, where, (("RndMeth", "Floor", ("Floor",)),
                                              ("SaturateOnIntegerOverflow", "off", ("off",))),
                                   "recordings")
                if Fraction(gi) == 0:
                    for k, nm in enumerate((X + "re", X + "im")):
                        add[nm] = dict(b, Gain=gr)
                        W(part(ins[0], k), nm)
                    parts[n] = ((X + "re", 1), (X + "im", 1))
                else:
                    T = str(b.get("OutDataTypeStr", G.fc._INHERIT_RULE)).strip()
                    inh_o = T == G.fc._INHERIT_RULE
                    if inh_p:
                        # recordings (G4 gain, 12 runs: Gain 0.5+0.25i on unsigned 12-32 / signed 34-64-bit
                        # parts, output internal rule or fixdt, Floor / Nearest / Zero / Convergent, saturation on /
                        # off, host and ASIC/FPGA): the parameter type is the internal rule's best precision of both
                        # parts together (input signedness and word length; dut_rtp); each of the four partial
                        # products is rounded to the output type with the block's RndMeth (and saturation), then
                        # re = agr - bgi, im = agi + bgr in the output type with the block's overflow mode. With the
                        # internal-rule output the output type is the real Gain's product type for that parameter
                        _cplx_gain_114(b, where, gr, gi, T)
                        post.append(("g4_parts", n, "a complex Gain with ParamDataTypeStr 'Inherit: Inherit via "
                                     "internal rule'", part(ins[0], 0)))
                    else:
                        for k in ("ParamDataTypeStr", "OutDataTypeStr"):
                            if G._parse_fixdt(str(b.get(k, ""))) is None:
                                _refuse(where + "." + k, "a complex gain needs an explicit fixdt %s (observed)" % k)
                    sat_ = str(b.get("SaturateOnIntegerOverflow", "off")).strip()
                    a, bb = part(ins[0], 0), part(ins[0], 1)
                    for nm, x, g in (("agr", a, gr), ("bgi", bb, gi), ("agi", a, gi), ("bgr", bb, gr)):
                        add[X + nm] = dict(b, Gain=g)
                        if inh_p:
                            add[X + nm]["_GainVec"] = "[%s %s]" % (gr, gi)
                        W(x, X + nm)
                    if sat_ == "on":
                        # the partial products' own saturation is not separated from the final one by the
                        # answered only where no partial product can overflow (checked after the compile)
                        post.append(("gain_sat", n, tuple(X + k_ for k_ in ("agr", "bgi", "agi", "bgr")),
                                     (part(ins[0], 0), part(ins[0], 1))))
                    for nm, signs, p1, p2 in (("re", "+-", "agr", "bgi"), ("im", "++", "agi", "bgr")):
                        add[X + nm] = {"type": "Sum", "Inputs": signs,
                                       "OutDataTypeStr": G.fc._SAME_FIRST if inh_o else T,
                                       "AccumDataTypeStr": G.fc._SAME_FIRST if inh_o else T,
                                       "RndMeth": b.get("RndMeth", "Floor") if inh_p else "Floor",
                                       "SaturateOnIntegerOverflow": sat_ if inh_p else "off"}
                        W((X + p1, 1), X + nm, 1)
                        W((X + p2, 1), X + nm, 2)
                    parts[n] = ((X + "re", 1), (X + "im", 1))
            elif t == "Sum":
                if not (str(b.get("Inputs", "++")).strip() in ("++", "2") and
                        str(b.get("OutDataTypeStr", G.fc._INHERIT_RULE)).strip() == G.fc._INHERIT_RULE and
                        str(b.get("SaturateOnIntegerOverflow", "off")).strip() == "off"):
                    # an explicit output type, '+-', saturation on -- each part the real Sum
                    # with the block's parameters (reading A); the part types and the hardware are checked after
                    # the compile (_cplx_t111_post)
                    _cplx_sum_t111(b, where, ins, cplx)
                    sums.append((n, [part(x, 0) for x in ins], dict(b)))
                else:
                    # shape ('++', internal rule, no saturation): one part type for both inputs was
                    # recorded (sfix16_En8; recordings '++' likewise) -- checked after the compile
                    sums.append((n, [part(x, 0) for x in ins], dict(b, _lab2_6=True)))
                for k, nm in enumerate((X + "re", X + "im")):
                    add[nm] = dict(b)
                    for j, x in enumerate(ins):
                        if x[0] not in cplx and k == 1:
                            # recordings (the NCO-mixer shape): a real input is a complex one with a zero imaginary
                            # part of its own type -- its imaginary part is 0 in that type (a Gain 0 keeping the type)
                            z = X + "zero%d" % (j + 1)
                            add[z] = {"type": "Gain", "Gain": "0", "OutDataTypeStr": "Inherit: Same as input",
                                      "RndMeth": "Floor", "SaturateOnIntegerOverflow": "off"}
                            W(part(x, 0), z)
                            W((z, 1), nm, j + 1)
                            continue
                        W(part(x, k), nm, j + 1)
                parts[n] = ((X + "re", 1), (X + "im", 1))
            elif t == "Product":
                if str(b.get("Inputs", "2")).strip() not in ("2", "**"):
                    _refuse(where + ".Inputs", "complex Product of two inputs (**) only")
                # recordings (G4 recordings, saturation on in 6 of 12): each part's sum (complex x
                # complex) or product (complex x real) saturates as the real block; the partial products are exact
                sat_ = str(b.get("SaturateOnIntegerOverflow", "off")).strip()
                if sat_ not in ("on", "off"):
                    _refuse(where + ".SaturateOnIntegerOverflow", "on or off", "BAD_PARAM")
                T = b.get("OutDataTypeStr", G.fc._INHERIT_RULE)
                if sat_ == "on":
                    post.append(("g4_parts", n, "a saturating complex Product",
                                 part(next(x for x in ins if x[0] in cplx), 0)))
                if any(x[0] not in cplx for x in ins):
                    # recordings (the NCO-mixer shape): complex x real -- each part times the real input, one
                    # product per part with the block's own output type and rounding
                    for nm, k in (("re", 0), ("im", 1)):
                        add[X + nm] = {"type": "Product", "Inputs": "2", "OutDataTypeStr": T,
                                       "RndMeth": b.get("RndMeth", "Floor"), "SaturateOnIntegerOverflow": sat_}
                        for j, x in enumerate(ins):
                            W(part(x, k) if x[0] in cplx else part(x, 0), X + nm, j + 1)
                    parts[n] = ((X + "re", 1), (X + "im", 1))
                    continue
                (ar, ai), (cr, ci) = [(part(x, 0), part(x, 1)) for x in ins]
                prods = {"ac": (ar, cr), "bd": (ai, ci), "ad": (ar, ci), "bc": (ai, cr)}
                for nm, (x, y) in prods.items():
                    add[X + nm] = {"type": "Product", "Inputs": "2", "OutDataTypeStr": T,
                                   "RndMeth": b.get("RndMeth", "Floor"), "SaturateOnIntegerOverflow": "off"}
                    W(x, X + nm, 1)
                    W(y, X + nm, 2)
                    exact.append((n, X + nm, x, y))
                for nm, signs, p1, p2 in (("re", "+-", "ac", "bd"), ("im", "++", "ad", "bc")):
                    add[X + nm] = {"type": "Sum", "Inputs": signs, "OutDataTypeStr": G.fc._SAME_FIRST,
                                   "AccumDataTypeStr": G.fc._SAME_FIRST, "RndMeth": "Floor",
                                   "SaturateOnIntegerOverflow": sat_}
                    W((X + p1, 1), X + nm, 1)
                    W((X + p2, 1), X + nm, 2)
                parts[n] = ((X + "re", 1), (X + "im", 1))
    if pending:
        raise G.GraphError("UNSUPPORTED", "design.blocks." + sorted(pending)[0], "a complex loop without a Unit Delay")
    c2ma = {}
    for n, b in blocks.items():
        if b.get("type") == "ComplexToMagnitudeAngle":   # recordings (engine/fxcordic.py): magnitude and angle cores
            import fxcordic
            p_ = fxcordic.params(n, b)
            if src.get((n, 1), (None,))[0] not in cplx:
                raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "Complex to Magnitude-Angle of a real signal "
                                   "(form the complex input with Real-Imag to Complex)")
            X = INTERNAL + n + "/"
            for k_ in ("mag", "ang"):
                add[X + k_] = fxcordic.core("c2ma", k_, p_)
                W(part(src[(n, 1)], 0), X + k_, 1)
                W(part(src[(n, 1)], 1), X + k_, 2)
            c2ma[n] = ((X + "mag", 1), (X + "ang", 1))
    out_blocks = {k: v for k, v in blocks.items() if k not in cplx and v.get("type") not in ("ComplexToRealImag",
                                                                                             "ComplexToMagnitudeAngle")}
    out_blocks.update(add)
    out_lines, moved = [], []
    for l in lines:
        if l["dst"] in cplx or blocks[l["dst"]].get("type") in ("ComplexToRealImag", "ComplexToMagnitudeAngle"):
            continue
        if l["src"] in c2ma:
            sp_ = int(l.get("sport", 1))
            if sp_ not in (1, 2):
                raise G.GraphError("BAD_CONNECTION", "design.lines", "output port %d of %s does not exist"
                                   % (sp_, l["src"]))
            p_ = c2ma[l["src"]][sp_ - 1]
            moved.append(((l["src"], sp_), p_))
            out_lines.append(dict({k: v for k, v in l.items() if k != "sport"}, src=p_[0]))
            continue
        if blocks[l["src"]].get("type") == "ComplexToRealImag":
            c = src[(l["src"], 1)]
            p_ = part(c, _c2ri_parts(blocks[l["src"]])[int(l.get("sport", 1)) - 1])
            moved.append(((l["src"], int(l.get("sport", 1))), (p_[0], p_[1])))
            nl = {k: v for k, v in l.items() if k != "sport"}
            nl["src"] = p_[0]
            if p_[1] != 1:
                nl["sport"] = p_[1]
            out_lines.append(nl)
            continue
        if l["src"] in cplx:
            continue   # into another complex block or a converter: wired through the parts above
        out_lines.append(l)
    # a Complex to Real-Imag carries its input's parts (its outputs 1 and 2): typed like them (fxgraph
    # _compiled_types); internal converters of a complex Outport (INTERNAL names) are not design blocks
    c2ri = {n: tuple(part(src[(n, 1)], k) for k in _c2ri_parts(b)) for n, b in blocks.items()
            if b.get("type") == "ComplexToRealImag" and not n.startswith(INTERNAL) and (n, 1) in src}
    import parameter_bounds
    return {"blocks": out_blocks, "lines": out_lines + new_lines,
            parameter_bounds.PORT_KEY: _repointed(spec, moved)}, {"exact": exact, "pairs": pairs,
                                                                     "parts": dict(parts, **c2ri),
                                                                     "c2ma": c2ma, "sums": sums, "post": post,
                                                                     "region": _cplx_regions(blocks, lines, cplx)}


def _cplx_regions(blocks, lines, cplx):
    """{Real-Imag to Complex: [(block, kind)]}: every block its complex signal reaches (through the complex blocks)
    and every consumer of a complex signal on the way; kind is the block type, 'GainC' for a complex-valued Gain,
    'Sum with a real input' for a Sum not all of whose inputs are complex."""
    users = {}
    for l in lines:
        users.setdefault(l["src"], []).append(l["dst"])
    out = {}
    for n in cplx:
        if blocks[n].get("type") != "RealImagToComplex":
            continue
        seen, todo, reg = {n}, [n], []
        while todo:
            x = todo.pop()
            for y in users.get(x, ()):
                if y in seen:
                    continue
                seen.add(y)
                t = blocks.get(y, {}).get("type")
                if t == "Gain" and y in cplx:
                    try:
                        t = "GainC" if Fraction(_cnum(blocks[y].get("Gain", "1"), "design.blocks.%s.Gain" % y)[1]) \
                            else "Gain"
                    except G.GraphError:
                        t = "GainC"
                    if t == "GainC" and str(blocks[y].get("ParamDataTypeStr", G.fc._INHERIT_RULE)).strip() != \
                            G.fc._INHERIT_RULE:
                        # show complex Gains on unsigned / wide parts with the internal-rule
                        # parameter type only (the explicit one: recordings signed parts)
                        t = "GainC with an explicit ParamDataTypeStr"
                if t == "Sum" and any(l["src"] not in cplx for l in lines if l["dst"] == y):
                    real_ = sorted(int(l.get("dport", 1)) for l in lines if l["dst"] == y and l["src"] not in cplx)
                    # a real first input, complex others, these sign strings
                    t = "Sum with a real first input" if real_ == [1] and _cplx_sum_signs(blocks[y]) in \
                        CPLX_SUM_REAL_FIRST else "Sum with a real input"
                reg.append((y, t))
                if y in cplx:
                    todo.append(y)
        out[n] = reg
    return out


# recordings (every step of 19 runs, host and ASIC/FPGA): a Real-Imag to Complex of two
# parts of one type, unsigned 16-32 bits or signed 17-33 bits, carried through zero-IC Unit Delays, real-valued Gains
# and Complex to Real-Imag to the Outports -- each part the real signal of its type (unsigned zero-extended, wide parts
# exact); recordings: sfix32 parts into a complex Sum.
# unsigned 12-32-bit and signed
# 34-64-bit parts (ufix12, ufix16_En8, ufix24_En2, uint32, sfix34_En20, sfix48_En20, int64, sfix64_En62) into a
# complex Sum, complex x complex / complex x real Product, complex-valued Gain, Unit Delay (nonzero IC), Mux / Demux;
# G3 / N3: signed 17-32-bit parts into a complex Sum.
CPLX_WIDE_PARTS = {1: (17, 64), 0: (12, 32)}
CPLX_WIDE_AFTER = ("UnitDelay", "Gain", "ComplexToRealImag", "Outport")   # recordings (unsigned 16-32, signed 17-33)
CPLX_G4_AFTER = ("UnitDelay", "Sum", "Product", "GainC", "Mux", "Demux", "ComplexToRealImag", "Outport")
CPLX_SUM_SIGNED_WL = 32   # signed parts up to 32 bits into a complex Sum
# recordings (22 runs) and recordings (76 runs, 512 steps, host and ASIC/FPGA): a complex
# Sum, each part the real Sum with the block's parameters -- accumulated input by input in the accumulator type from
# the first '+' input, overflowing at every step, then cast to the output type
CPLX_SUM_T111 = {"Inputs": ("++", "+-", "-+", "--", "+++", "++-", "+-+", "-++", "-+-", "++++", "2", "3", "4"),
                 "RndMeth": ("Floor", "Nearest", "Zero", "Convergent"),
                 "SaturateOnIntegerOverflow": ("off", "on"), "out_wl": (3, 33), "acc_wl": (2, 33)}
# a real first input with complex others (recorded sign configurations: '-+', '--', '+-', '++-', '+-+', '-++', '-+-')
CPLX_SUM_REAL_FIRST = ("-+", "--", "+-", "++-", "+-+", "-++", "-+-")
_FIXDT_LIT = re.compile(r"^fixdt\(\s*([01])\s*,\s*(\d+)\s*,\s*(\d+)\s*\)$")
_CPLX_SUM_SRC = "recordings"


def _cplx_sum_signs(b):
    v = str(b.get("Inputs", "++")).strip()
    return v.replace("|", "") if v.startswith("|") else v


def _cplx_sum_t111(b, where, ins, cplx):
    """A complex Sum outside the recordings shape: answered in the recordings domain only."""
    src_ = _CPLX_SUM_SRC
    sg = _cplx_sum_signs(b)
    # a leading '|' (rectangular icon) is recorded (recordings '|++', recordings runs) and
    # reads as the sign list without it -- the native recordings is this shape (declared)
    if sg not in CPLX_SUM_T111["Inputs"]:
        _refuse(where + ".Inputs", "Inputs %s on a complex signal: only %s is recorded (%s)"
                % (b.get("Inputs"), " / ".join(CPLX_SUM_T111["Inputs"]), src_))
    for k, default in (("RndMeth", "Floor"), ("SaturateOnIntegerOverflow", "off")):
        v = str(b.get(k, default)).strip()
        if v not in CPLX_SUM_T111[k]:
            _refuse(where + "." + k, "%s %s on a complex signal: only %s is recorded (%s)"
                    % (k, v, " / ".join(CPLX_SUM_T111[k]), src_))
    od = str(b.get("OutDataTypeStr", G.fc._INHERIT_RULE)).strip()
    m = _FIXDT_LIT.match(od)
    lo, hi = CPLX_SUM_T111["out_wl"]
    if od not in (G.fc._INHERIT_RULE, G.fc._SAME_FIRST) and not (
            m and lo <= int(m.group(2)) <= hi and int(m.group(3)) <= int(m.group(2))):
        _refuse(where + ".OutDataTypeStr", "OutDataTypeStr %s on a complex signal: recorded Inherit: Inherit via "
                "internal rule, Inherit: Same as first input and fixdt(s, %d..%d, 0..word length) only (%s)"
                % (od, lo, hi, src_))
    ad = str(b.get("AccumDataTypeStr", G.fc._INHERIT_RULE)).strip()
    m = _FIXDT_LIT.match(ad)
    lo, hi = CPLX_SUM_T111["acc_wl"]
    if ad != G.fc._INHERIT_RULE and not (m and lo <= int(m.group(2)) <= hi):
        _refuse(where + ".AccumDataTypeStr", "AccumDataTypeStr %s on a complex Sum with an explicit output type, "
                "'-' inputs, more than two inputs or saturation: recorded Inherit: Inherit via internal rule and "
                "fixdt(s, %d..%d, f) only (%s)" % (ad, lo, hi, src_))
    real = [j for j, x in enumerate(ins) if x[0] not in cplx]
    if real and (real != [0] or sg not in CPLX_SUM_REAL_FIRST):
        _refuse(where, "a real input into a complex Sum with an explicit output type, '-' inputs, more than two "
                "inputs or saturation: recorded with a real first input and complex others for Inputs %s only (%s)"
                % (" / ".join(CPLX_SUM_REAL_FIRST), src_))


def _cplx_t111_post(d, info, port_block):
    """The compiled-type side of the complex Sum openings: the Sum's input part
    types and the hardware."""
    for n, ins, b in info.get("cplx_sums", []):
        where = "design.blocks." + n
        sg = _cplx_sum_signs(b)
        if b.get("_lab2_6"):
            # shape ('++', internal rule, no saturation): recorded with one part type and with
            # two different part types on the inputs (recordings Simulink runs, recordings,
            # equal) -- answered as before; the native recordings is this shape
            continue
        for s, sp in ins:
            t = d.otype.get(port_block(s, sp, where))
            if t is None or t == G.BOOL:
                _refuse(where, 'a complex Sum outside the recordings shape on %s parts: not recorded'
                        % (G._tstr(t) if t else "untyped"))
            if t[0] and 16 < t[1] < CPLX_SUM_SIGNED_WL and not (len(ins) == 2 and sg in ("++", "+-", "-+", "2")):
                # the native recordings (sfix17 parts into '-++'): signed 17-31-bit parts are recorded
                # into two-input complex Sums only (17 / 24 / 31 bits); G3 has 8 / 16 / 32-bit parts
                _refuse(where, ('a complex Sum %s on %s parts: signed 17-31-bit '
                    'parts are recorded into two-input complex Sums only; '
                    '3-4 inputs are recorded with 8 / 16 / 32-bit parts') % (b.get("Inputs"), G._tstr(t)))
            if t[0] and t[1] <= CPLX_SUM_SIGNED_WL:
                continue
            # unsigned 12-32-bit / signed 34-64-bit parts
            lo_, hi_ = (34, 64) if t[0] else CPLX_WIDE_PARTS[0]
            if not (lo_ <= t[1] <= hi_ and sg in ("++", "+-", "2") and len(ins) == 2 and
                    str(b.get("OutDataTypeStr", G.fc._INHERIT_RULE)).strip() in (G.fc._INHERIT_RULE, G.fc._SAME_FIRST)
                    and str(b.get("AccumDataTypeStr", G.fc._INHERIT_RULE)).strip() == G.fc._INHERIT_RULE):
                _refuse(where, "a complex Sum on %s parts: recorded with signed parts of at most %d bits and, for "
                        "unsigned 12-32-bit and signed 34-64-bit parts, with two complex inputs '++' / '+-', the "
                        "internal-rule or Same-as-first output and the internal-rule accumulator only (%s)"
                        % (G._tstr(t), CPLX_SUM_SIGNED_WL, _CPLX_SUM_SRC))
        if G.fc._asic(getattr(d, "hw", None)):
            # recordings: 28 ASIC/FPGA runs (every Inputs string, the four RndMeth, saturation on / off,
            # explicit accumulator, Same as first); explicit fixdt outputs on ASIC/FPGA are all signed
            m = _FIXDT_LIT.match(str(b.get("OutDataTypeStr", "")).strip())
            if m and m.group(1) == "0":
                _refuse(where, "a complex Sum with an unsigned fixdt output on ASIC/FPGA: recorded with signed fixdt "
                        "outputs only on ASIC/FPGA")


def _cplx_gain_114(b, where, gr, gi, T):
    """A complex-valued Gain with the internal-rule parameter type: the recordings domain."""
    src_ = 'recorded gain configurations'
    if not (Fraction(gr) > 0 and Fraction(gi) > 0):
        _refuse(where + ".Gain", "a complex Gain %s with ParamDataTypeStr 'Inherit: Inherit via internal rule': recorded "
                "with positive real and imaginary parts only (%s); give an explicit ParamDataTypeStr with Floor and no "
                "saturation" % (b.get("Gain"), src_))
    rm = str(b.get("RndMeth", "Floor")).strip()
    if rm not in ("Floor", "Nearest", "Zero", "Convergent"):
        _refuse(where + ".RndMeth", "RndMeth %s on a complex Gain: only Floor / Nearest / Zero / Convergent is "
                "recorded (%s)" % (rm, src_))
    if str(b.get("SaturateOnIntegerOverflow", "off")).strip() not in ("on", "off"):
        _refuse(where + ".SaturateOnIntegerOverflow", "on or off", "BAD_PARAM")
    if T != G.fc._INHERIT_RULE and G._parse_fixdt(T) is None:
        _refuse(where + ".OutDataTypeStr", "OutDataTypeStr %s on a complex Gain: recorded Inherit: Inherit via internal "
                "rule and fixdt only (%s)" % (T, src_))


def _cplx_114_post(d, info, port_block):
    """recordings after the compile: a complex Unit Delay's nonzero initial condition must be exact in the
    part type (unsigned 12-32 / signed 34-64-bit parts, as recorded); a saturating complex Gain is answered only where
    no partial product can overflow the output type (the recordings do not show whether the partial products
    saturate on their own)."""
    for kind, n, blks, x in info.get("cplx_post", []):
        where = "design.blocks." + n
        if kind == "g4_parts":
            t = d.otype.get(port_block(x[0], x[1], where))
            if not (isinstance(t, tuple) and ((not t[0] and CPLX_WIDE_PARTS[0][0] <= t[1] <= CPLX_WIDE_PARTS[0][1]) or
                                              (t[0] and 34 <= t[1] <= 64))):
                _refuse(where, "%s on %s parts: recorded on unsigned 12-32-bit and signed 34-64-bit parts only "
                        "" % (blks, G._tstr(t) if isinstance(t, tuple) else "untyped"))
        elif kind == "ud_ic":
            for nm, v in zip(blks, x):
                t = d.otype.get(nm)
                if t is None or t == G.BOOL:
                    _refuse(where + ".InitialCondition", "a complex Unit Delay with a nonzero initial condition on "
                            "untyped parts: not recorded")
                lo_, hi_ = (34, 64) if t[0] else CPLX_WIDE_PARTS[0]
                if not lo_ <= t[1] <= hi_:
                    _refuse(where + ".InitialCondition", ('a complex Unit Delay with initial condition %s on %s parts: a nonzero initial '
                        'condition is recorded on unsigned 12-32-bit and signed 34-64-bit parts only (recorded '
                        'delay and gain-chain configurations; the other recorded initial conditions were zero)') % (
                                G._fmt_num(v), G._tstr(t)))
                q = v * Fraction(2) ** t[2]
                lo, hi = (-(1 << (t[1] - 1)), (1 << (t[1] - 1)) - 1) if t[0] else (0, (1 << t[1]) - 1)
                if q.denominator != 1 or not lo <= q <= hi:
                    _refuse(where + ".InitialCondition", "a complex Unit Delay initial condition part %s that is not "
                            "exact in its type %s: recorded with exact values only"
                            % (G._fmt_num(v), G._tstr(t)))
        elif kind == "gain_sat":
            T = d.otype.get(blks[0])
            for nm in blks:
                (s_, sp_) = x[0] if nm.endswith(("agr", "agi")) else x[1]
                ti = d.otype.get(port_block(s_, sp_, where))
                pt = d._gain_param_type(nm, ti) if hasattr(d, "_gain_param_type") and isinstance(ti, tuple) else None
                if not (isinstance(T, tuple) and isinstance(ti, tuple) and isinstance(pt, tuple)):
                    _refuse(where + ".SaturateOnIntegerOverflow", "a saturating complex Gain whose types are not known")
                g = Fraction(G.fc._quant_param(float(Fraction(str(d.params[nm]["Gain"]))), *pt)) / Fraction(2) ** pt[2]
                ilo, ihi = ((-(1 << (ti[1] - 1)), (1 << (ti[1] - 1)) - 1) if ti[0] else (0, (1 << ti[1]) - 1))
                ps = [Fraction(v_) / Fraction(2) ** ti[2] * g for v_ in (ilo, ihi)]
                tlo, thi = ((-(1 << (T[1] - 1)), (1 << (T[1] - 1)) - 1) if T[0] else (0, (1 << T[1]) - 1))
                if not all(Fraction(tlo) / Fraction(2) ** T[2] <= p_ <= Fraction(thi) / Fraction(2) ** T[2] for p_ in ps):
                    _refuse(where + ".SaturateOnIntegerOverflow", ('a saturating complex Gain whose partial product (%s part x %s) '
                        'can overflow the output type %s: recorded only where no partial '
                        'product overflows, so whether the partial products saturate on '
                        'their own is not known; widen the output type or turn saturation off')
                            % (G._tstr(ti), G._fmt_num(g), G._tstr(T)))


def _cplx_wide_ok(d, info, n, ta):
    """None when the merge n of parts ta is in the recordings domain, else the reason."""
    if ta == G.BOOL:
        return "boolean parts are not recorded"
    lo, hi = CPLX_WIDE_PARTS[1 if ta[0] else 0]
    if not lo <= ta[1] <= hi or not 0 <= ta[2] <= ta[1]:
        return "recorded with unsigned 12-32-bit and signed 17-64-bit parts (fraction length 0..word length) only"
    if getattr(d, "multirate", False):
        return "recorded in single-rate designs only"
    g4 = not ta[0] or ta[1] >= 34
    for y, t in info.get("cplx_region", {}).get(n, ()):
        if g4 and t in CPLX_G4_AFTER:
            continue
        if (not ta[0] and ta[1] >= 16 or ta[0] and ta[1] <= 33) and t in CPLX_WIDE_AFTER:
            continue
        if ta[0] and ta[1] <= CPLX_SUM_SIGNED_WL and t in ("Sum", "Sum with a real first input"):
            continue
        return "after the merge recorded through %s only; %s (%s) is not" % (
            "Unit Delay, real-valued Gain and Complex to Real-Imag to the Outports, and a complex Sum"
            if ta[0] and ta[1] <= 33 else
            "complex Sum, Product, complex-valued Gain, Unit Delay, Mux / Demux and Complex to Real-Imag to the Outports"
            + (" (and real-valued Gain for 16-32 bits)" if not ta[0] else ""),
            y, "complex-valued Gain" if t == "GainC" else ("complex-valued Gain with an explicit ParamDataTypeStr" if t.startswith("GainC") else t))
    return None


def _on_loop(spec, n):
    """True when block n's output reaches one of its own inputs through the design's lines."""
    succ = {}
    for ln in spec.get("lines", []):
        succ.setdefault(ln["src"], set()).add(ln["dst"])
    seen, todo = set(), list(succ.get(n, ()))
    while todo:
        m = todo.pop()
        if m == n:
            return True
        if m not in seen:
            seen.add(m)
            todo.extend(succ.get(m, ()))
    return False


def es_kernel_check(info):
    """refuse a run with a stateful exact kernel inside an enabled subsystem (see _enabled_subsystems)."""
    for owner, t in (info or {}).get("es_kernels") or []:
        _refuse("design.blocks." + owner, "%s inside an enabled subsystem runs on its exact kernel, which is not gated "
                "by the enable (it would keep running while disabled); not supported -- use a setting that lowers to "
                "basic blocks (docs/en/HDL_BLOCKS.md) or move the block out of the enabled subsystem"
                % {"NCOCore": "HDL NCO", "HDLFIRCore": "HDL Discrete FIR", "RAMCore": "HDL RAM",
                   "MLFn": "MATLAB Function with persistent state"}[t])


def check_control_rates(spec, per, offs):
    """Refuse a block with control ports whose controls run on another grid than its data (see below)."""
    # a block with control ports (a Delay with an
    # enable / reset port, the synchronous delays, the Enabled Synchronous Tapped Delay, the HDL Counter) whose
    # controls run on another grid than its data, or whose data has an offset: Simulink gave other values (all 8
    # port combinations, resettable synchronous delays at step 1) or refused (StateEnabledOrTriggeredSubsystem...,
    # InconsistentBlockBasedTs) -- refused
    CTRL = ("UnitDelayEnabledSynchronous", "UnitDelayResettableSynchronous", "UnitDelayEnabledResettableSynchronous",
            "TappedDelayEnabledSynchronous", "HDLCounter")
    blocks_ = spec.get("blocks") or {}
    lines_ = spec.get("lines", [])
    for n, b in blocks_.items():
        if not (isinstance(b, dict) and b.get("type") == "EnablePort" and "/" in n):
            continue
        # recordings (Unit Delay -> Gain -> Delay(2), held and reset): an enable slower than the data
        # (period 2 on data at 1, also with offset 1) was equal to Simulink; an enable faster than the data (period 1
        # on data at 2) differed from step 2 -- refused, as are mixed data rates and an offset on the data
        sub_ = n.rsplit("/", 1)[0] + "/"
        en_src = {ln["src"] for ln in lines_ if ln.get("dst") == n}
        data = {ln["src"] for ln in lines_ if ln.get("dst", "").startswith(sub_) and not ln["src"].startswith(sub_)
                and ln.get("dst") != n}
        pd = {per.get(s_) for s_ in data if per.get(s_) is not None}
        od = {offs.get(s_) for s_ in data if offs.get(s_)}
        pe = {per.get(s_) for s_ in en_src if per.get(s_) is not None}
        ok = len(pd) <= 1 and not od and len(pe) <= 1 and (
            not pd or not pe or (Fraction(next(iter(pe))) / Fraction(next(iter(pd)))).denominator == 1)
        if not ok:
            raise G.GraphError("UNSUPPORTED_RATE", "design.blocks." + n, "enabled subsystem %s: data at periods %s%s, "
                               "enable at %s -- recorded with one data rate and an enable at the "
                               "same or a multiple period only (an enable faster than the data gave other values); "
                               "run the enable at the data's period or a multiple of it" % (
                                   sub_[:-1], sorted(map(str, pd)), ", data offsets %s" % sorted(map(str, od))
                                   if od else "", sorted(map(str, pe))))
    for n, b in blocks_.items():
        t = b.get("type") if isinstance(b, dict) else None
        ported = t == "Delay" and (str(b.get("ShowEnablePort", "off")) == "on" or
                                   str(b.get("ExternalReset", "None")) != "None")
        if not (ported or t in CTRL):
            continue
        if (t == "UnitDelayEnabledSynchronous" or (t == "Delay" and str(b.get("ExternalReset", "None")) == "None")) \
                and not _on_loop(spec, n):
            # an enable alone on another grid (or with an
            # offset) -- Delay with enable and no reset, Enabled Synchronous Unit Delay -- was equal to Simulink in
            # every audited unit (48, recordings); every unit that differed had a reset port. Still refused:
            # the Enabled Synchronous Tapped Delay (Simulink: StateEnabledOrTriggeredSubsystemMustBeSingleRate) and
            # an enabled delay on a feedback loop (Simulink: InconsistentBlockBasedTs on part of the grids)
            continue
        ins = sorted((int(ln.get("dport", 1)), ln["src"]) for ln in spec.get("lines", []) if ln.get("dst") == n)
        ps_ = {per.get(s_) for _p, s_ in ins if per.get(s_) is not None}
        os_ = {offs.get(s_) for _p, s_ in ins if offs.get(s_)}
        if len(ps_) > 1 or os_:
            raise G.GraphError("UNSUPPORTED_RATE", "design.blocks." + n, "%s whose inputs run at periods %s%s: a "
                               "control on another grid than the data (or an offset) gave other values than "
                               "Simulink or was refused by it; give the data and its controls "
                               "one rate" % (t, sorted(map(str, ps_)), ", offsets %s" % sorted(map(str, os_))
                                             if os_ else ""))


# unsigned integer and ufix1 controls of a
# Delay act as non-zero = true -- every mode, enable and reset, equal to Simulink (uint8 / uint16 / ufix1); signed
# ones (int8 / int32) act by their sign (enable > 0, edges of the sign: _pdelay), equal on every recorded step;
# other signed widths were not recorded
A26_CTRL_TYPES = {"external reset": ((0, 8, 0), (0, 16, 0), (0, 1, 0), (1, 8, 0), (1, 32, 0)),
                  "enable": ((0, 16, 0), (0, 1, 0), (1, 8, 0), (1, 32, 0))}   # int8 / int32: by sign (see _pdelay)


def post_compile_checks(d, info):
    """Checks that need compiled types: an HDL Counter's controls must be boolean and its load_val
    must have the count's type (Simulink:DataType:InputPortDataTypeMismatch); a synchronous delay's initial
    condition is rounded to the nearest value of the input type (0.3 -> 77 at 2^-8, 0.0039 -> 1) and one outside the
    type's range is an error (SimulinkFixedPoint:util:fxpParameterOverflow)."""
    if getattr(d, "multirate", False):
        lat_ = sorted(x[len(INTERNAL):-len("/kf1")] for x in d.names if x.startswith(INTERNAL) and x.endswith("/kf1"))
        if lat_:
            # recordings (silent wrong): a Discrete FIR 'Lattice MA' at a slower rate (after a fast-to-slow Rate
            # Transition or a multirate Downsample) stepped its stage delays at the base step -- recorded at the
            # base step of single-rate designs only, as
            # fxiir.post_compile refuses the other filters
            _refuse("design.blocks." + lat_[0], "Discrete FIR 'Lattice MA' in a multirate design: recorded at the base "
                    "step of single-rate designs only; record it at its rate to open it")
    for n_, st_ in info.get("counter_st_ports", ()):
        pers_ = {str(d.params[x].get("SampleTime", "-1")).strip() for x in d.names if d.btype.get(x) == "Inport"}
        if getattr(d, "multirate", False) or not pers_ or any(_norm_num(v) != _norm_num(st_) for v in pers_):
            _refuse("design.blocks." + n_, "HDL Counter with input ports and CountSampTime %s: recorded with the ports "
                    "sampled at the counter's period only; here the inputs run at %s"
                    % (st_, ", ".join(sorted(pers_)) or "the base step"))
    import fxaction
    fxaction.post_compile(d, info, _port_block, lambda d_, s_: d_.type_str(s_) if s_ in d_.otype else "?")
    import fxtrigger
    fxtrigger.post_compile(d, info, _port_block)
    import fxprelookup
    fxprelookup.post_compile(d, info, _port_block)
    import fxiir
    fxiir.post_compile(d, info, _port_block)
    for n, ic, (s, sp) in info.get("repeat_ic", []):
        t = d.otype.get(_port_block(info, s, sp, "design.blocks." + n))
        try:
            v = float(str(ic).strip())
        except ValueError:
            continue   # the block's own reading reports a bad spelling
        if isinstance(t, tuple):
            G._param_overflow(v, t, "design.blocks.%s.ic" % n, "Repeat %s: ic" % n,
                              mode=(getattr(d, "hw", None) or {}).get("ParameterOverflowMsg", G.PARAM_OVERFLOW_DEFAULT))
    for n, combo, (s, sp) in info.get("pdelay_meta", []):
        if n in info.get("trig_delays", ()):
            continue   # a triggered / resettable subsystem's own lowering
        # Enabled-delay domain (engine/delayen_domain.json): where
        # neither the recordings reach, refused -- ASIC/FPGA only for the enable alone and with a Level or Rising
        # reset, data of at most 32 bits (recorded: 8 and 16 bits), a feedback
        # loop only for the recorded shapes (a Level reset alone, the enable with a Rising reset;
        # the enable with a Falling / Either / Level hold reset)
        if G.fc._asic(getattr(d, "hw", None)) and combo not in PDELAY_ASIC:
            _refuse("design.blocks." + n, "Delay with ShowEnablePort %s and ExternalReset %r on ASIC/FPGA: recorded "
                    "on the host only (ASIC/FPGA: the enable alone and with a Level or Rising reset); record it to "
                    "open it" % combo)
        t_ = d.otype.get(_port_block(info, s, sp, "design.blocks." + n))
        if isinstance(t_, tuple) and len(t_) == 3 and t_[1] > 32:
            _refuse("design.blocks." + n, "Delay with an enable / reset port on %d-bit data: recorded up to 32 bits; "
                    "record it to open it" % t_[1])
        if n in info.get("pdelay_loop", ()) and combo not in PDELAY_LOOP:
            _refuse("design.blocks." + n, "Delay with ShowEnablePort %s and ExternalReset %r inside a feedback loop: "
                    "not recorded (recorded loops); record it to open it" % combo)
    combos_ = {n_: c_ for n_, c_, _u in info.get("pdelay_meta", [])}
    for n, port, (s, sp) in info.get("bool_port", []):
        sb = _port_block(info, s, sp, "design.blocks." + n)
        if port == "enable" and d.otype.get(sb) != G.BOOL and combos_.get(n, ("on", "None"))[1] in \
                ("Falling", "Either", "Level hold") and n not in info.get("trig_delays", ()):
            # merge of a previous version (the regression check): a non-boolean enable is recorded alone and with a Level or
            # Rising reset only;
            # with a Falling / Either / Level hold reset it is not -- refused
            _refuse("design.blocks." + n, "Delay enable input of type %s with ExternalReset %r: a non-boolean enable is "
                    "recorded alone and with a Level or Rising reset only; use a boolean enable or record it"
                    % (d.type_str(sb) if sb in d.otype else "?", combos_[n][1]))
        if port == "enable" and d.otype.get(sb) == (0, 8, 0):
            continue   # recordings (main): a uint8 enable, non-zero enables
        if d.otype.get(sb) in A26_CTRL_TYPES.get(port, ()):
            continue
        if d.otype.get(sb) != G.BOOL:
            _refuse("design.blocks." + n, "Delay %s input has type %s: only a boolean %s input (and a uint8 "
                    "enable) was recorded"
                    % (port, d.type_str(sb) if sb in d.otype else "?", port))
    # recordings (silent wrong; class: every enable the lowering builds -- enabled subsystems and the synchronous
    # delays): Simulink enables on > 0 for any type; the typed pass marks a signed enable for the > 0 lowering, but
    # one computed from the block's own output (a feedback through a Unit Delay) has no type then and was lowered
    # as ~= 0. Now its compiled type decides: the design is lowered again with those enables marked
    # (Design.compile catches RELOWER once)
    relower = set()
    for n, (s, sp), marked in info.get("es_enable", []):
        t = d.otype.get(_port_block(info, s, sp, "design.blocks." + n))
        if not marked and isinstance(t, tuple) and t[0] == 1:
            relower.add(n)
            info.setdefault("relower_types", {})[n] = G._tstr(t) if not str(G._tstr(t)).startswith("fixdt") else \
                G._tstr(t)
    for n, c, (s, sp), marked in info.get("sync_ctrl", []):
        t = d.otype.get(_port_block(info, s, sp, "design.blocks." + n))
        if c == "E" and not marked and isinstance(t, tuple) and t[0] == 1 and tuple(t) in SYNC_CTRL_TYPES:
            relower.add(n)
    if relower:
        e_ = G.GraphError("RELOWER", "design", "signed enables to lower with the > 0 rule: %s" % sorted(relower))
        e_.relower = sorted(relower)
        e_.relower_types = {n: "fixdt(%d,%d,%d)" % tuple(d.otype[_port_block(info, s, sp, "design.blocks." + n)])
                            for n, (s, sp), m in info.get("es_enable", []) if n in relower}
        e_.relower_types.update({n: "fixdt(%d,%d,%d)" % tuple(d.otype[_port_block(info, s, sp, "design.blocks." + n)])
                                 for n, c, (s, sp), m in info.get("sync_ctrl", []) if n in relower and c == "E"})
        raise e_
    for n, c, (s, sp), marked in info.get("sync_ctrl", []):
        # the controls of a synchronous delay whose type the typed pass could not see (a control
        # computed from the delay's own output): a type Simulink refuses, or a signed enable that was not marked
        # for the > 0 rule (it was enabled by any non-zero value, silently), is refused here
        t = d.otype.get(_port_block(info, s, sp, "design.blocks." + n))
        if t is None or t == G.BOOL:
            continue
        if tuple(t) not in SYNC_CTRL_TYPES:
            raise G.GraphError("PORT_TYPE_INVALID", "design.blocks." + n, "%s input of type %s (Simulink: "
                               'DataType:Invalid%sInputDataType -- boolean, ufix1 and 8 / 16 / 32-bit integers only'
                               ')' % ("enable" if c == "E" else "reset", G._tstr(t),
                                                               "Enable" if c == "E" else "Reset"))
        if c == "E" and t[0] == 1 and not marked:
            # the reason said "computed from the delay's own output" also for an enable straight
            # from an Inport (a Tapped Delay Enabled Synchronous): the block simply was not marked for the rule
            u_ = n.split("\x1d")[0].lstrip("\x1d") or n   # a Tapped Delay's lowered stage names its own block
            _refuse("design.blocks." + u_, "a signed enable (%s) on %s: Simulink enables it when the value is > 0; "
                    "that rule is not applied to this block here, so it is not supported -- give a "
                    "boolean or unsigned enable" % (G._tstr(t), u_))
    for n, port, (s, sp) in info["bool_ctrl"]:
        # recordings (i): the type of the source's own port (a Partly serial FIR's valid is its port 2; the check
        # read port 1's type and refused the chain into a FIR Decimator)
        sb = _port_block(info, s, sp, "design.blocks." + n)
        if d.otype.get(sb) != G.BOOL:
            raise G.GraphError("TYPE_MISMATCH", "design.blocks.%s" % n, "HDL Counter input %s has type %s; it must be "
                               "boolean (Simulink:DataType:InputPortDataTypeMismatch)"
                               % (port, d.type_str(sb) if sb in d.otype else "?"))
    for n, (s, sp), t in info["same_type"]:
        sb = _port_block(info, s, sp, "design.blocks." + n)    # its own port (as bool_ctrl above)
        if d.otype.get(sb) != t:
            raise G.GraphError("TYPE_MISMATCH", "design.blocks.%s" % n, "load_val has type %s; it must have the "
                               "count's type fixdt(%d,%d,%d) (Simulink:DataType:InputPortDataTypeMismatch)"
                               % (d.type_str(sb) if sb in d.otype else "?", t[0], t[1], t[2]))
    for n, p, x, y in info.get("cplx_exact", []):
        ta, tb = d.otype.get(_port_block(info, x[0], x[1], "design.blocks." + n)), \
            d.otype.get(_port_block(info, y[0], y[1], "design.blocks." + n))
        tp = d.otype.get(p)
        if None in (ta, tb, tp) or G.BOOL in (ta, tb):
            continue
        if tp[2] != ta[2] + tb[2] or tp[1] - tp[0] < (ta[1] - ta[0]) + (tb[1] - tb[0]) + (ta[0] or tb[0]):
            _refuse("design.blocks." + n, "complex Product whose partial products %s x %s are rounded or cut to %s "
                    "(only exact partial products are observed)" % (G._tstr(ta), G._tstr(tb), G._tstr(tp)))
    for n, (s, sp), boolean in info.get("drp", []):
        if (d.otype.get(s) == G.BOOL) != boolean:
            _refuse("design.blocks." + n, "Detect Rise Positive: the input's type (%s) is only known after compilation "
                    "and differs from the structural guess; a boolean input must come from a comparison or logic "
                    "block, not straight from a design input" % (d.type_str(s) if s in d.otype else "?"))
    if info.get("fifo"):
        import fxhdlfifo
        fxhdlfifo.post_compile(d, info, lambda s, sp, w: _port_block(info, s, sp, w))
    for n, A, din, addrs, we in info.get("ram", []):
        # the type of the source's own port (it was always port 1 of the source block)
        pb = lambda x: _port_block(info, x[0], x[1] if len(x) > 1 else 1, "design.blocks." + n)   # noqa: E731
        if d.otype.get(pb(we)) != G.BOOL:
            _refuse("design.blocks." + n, "RAM write enable of type %s (boolean observed)" % d.type_str(pb(we)))
        for a in addrs:
            if d.otype.get(pb(a)) != (0, A, 0):
                _refuse("design.blocks." + n, "RAM address of type %s; the address must be ufix%d (AddressBits %d): "
                        'Simulink refuses other address types too (Simulink:DataType:InputPortDataTypeMismatch'
                        '); convert it with a Data Type Conversion to ufix%d' % (d.type_str(pb(a)), A, A,
                                                                                               A))
        din = (pb(din), 1)
        dt = d.otype.get(din[0])
        deep = info.get("ram_deep", {}).get(n)
        if deep is not None and not deep[3]:
            if (A, dt) not in RAM_DEEP_DATA:
                _refuse("design.blocks." + n, 'a RAM with %d address bits and data %s: recorded %s'
                        '' % (A, d.type_str(din[0]), ", ".join("%d bits / %s" % (a_, t_) for a_, t_ in
                                                                       sorted(RAM_DEEP_DATA, key=str))))
            if _norm_num(deep[1]) != 0 and (A, dt, deep[2]) not in RAM_DEEP_INIT:
                _refuse("design.blocks." + n + ".RAMInitialValue", "a deep RAM with a non-zero initial value was "
                        'recorded as a Simple dual port RAM with 10 address bits and sfix16_En8 data only'
                        "")
        if isinstance(dt, tuple) and deep is None and (dt[1] == 32 and (dt not in RAM_DATA32_TYPES or A != 8)):
            _refuse("design.blocks." + n, "RAM of data %s with %d address bits: the new widths were recorded as pairs "
                    'only -- int32 / sfix32_En16 / uint32 data with 8 address bits, sfix16_En8 data with 10 or 16'
                    "" % (d.type_str(din[0]), A))
        if isinstance(dt, tuple) and dt[1] not in RAM_DATA_BITS:
            # RAM data was recorded at 8 and 16 bits, 12, 18 and 24
            # bits and boolean
            _refuse("design.blocks." + n, 'RAM of %d-bit data: recorded with %s-bit and boolean data only'
                    "" % (dt[1], " / ".join(map(str, RAM_DATA_BITS))))
    def port_block(s, sp, where):
        return _port_block(info, s, sp, where)
    import block_lowering
    block_lowering.post_compile(d, info, port_block)
    for n, (s, sp), (s2, sp2) in info.get("cplx_pairs", []):
        ta, tb = d.otype.get(port_block(s, sp, "design.blocks." + n)), d.otype.get(port_block(s2, sp2,
                                                                                               "design.blocks." + n))
        if ta != tb:
            # Simulink stopped all four recorded merges whose parts differ in
            # type (InputPortDataTypeMismatch) and accepted none
            raise G.GraphError("TYPE_MISMATCH", "design.blocks." + n, "Real-Imag to Complex of a real part %s and an "
                               "imaginary part %s: both parts must have the same type -- Simulink rejects the model "
                               "(Simulink:DataType:InputPortDataTypeMismatch)"
                               % (G._tstr(ta) if ta and ta != G.BOOL else ta, G._tstr(tb) if tb and tb != G.BOOL else tb))
        pa_, pb_ = port_block(s, sp, "design.blocks." + n), port_block(s2, sp2, "design.blocks." + n)
        nco_out_ = n in (info.get("nco_cplx_direct") or ()) and all(_nco_owner(d, x_) == n for x_ in (pa_, pb_)) and all(
            d.btype.get(y_) == "Outport" for y_ in d.names if any(z_ in (pa_, pb_) for z_ in (d.src.get(y_) or ())))
        # recordings (20/12) and recordings (18/16): an HDL NCO's Complex exponential output carried
        # straight to the model's outputs -- no complex arithmetic on the parts -- runs at the NCO's own recorded types
        wide_ = ta is not None and (ta == G.BOOL or not ta[0] or ta[1] > CPLX_MAX_WL)
        own_ = {_nco_owner(d, x_) for x_ in (pa_, pb_)}
        nco_src_ = len(own_) == 1 and None not in own_   # both parts made by one HDL NCO
        if wide_ and nco_src_ and not nco_out_:
            # an HDL NCO's wide complex output through a Gain / Complex to
            # Real-Imag was never recorded -- recordings opened Real-Imag to Complex merges, not NCO outputs
            _refuse("design.blocks." + n, "an HDL NCO's Complex exponential output of %s parts: recorded straight to "
                    "the model's outputs only; anything after it is not "
                    "recorded" % G._tstr(ta))
        why_ = _cplx_wide_ok(d, info, n, ta) if wide_ and not nco_out_ else None
        if why_:
            # opened unsigned 16-32 / signed 17-33-bit merges on a recorded path (_cplx_wide_ok)
            _refuse("design.blocks." + n, ('a complex signal of %s parts: complex parts are recorded signed '
                'and at most %d bits and, merged by Real-Imag to Complex, '
                'wider or unsigned for the following recorded configurations: %s') % (G._tstr(ta) if ta != G.BOOL else "boolean",
                                                                CPLX_MAX_WL, why_))
    _cplx_t111_post(d, info, port_block)
    _cplx_114_post(d, info, port_block)
    for n, srcs in info.get("dtd", []):
        blks = [port_block(s, sp, "design.blocks." + n) for s, sp in srcs]
        ts = [d.otype.get(x) for x in blks]
        if len(set(ts)) > 1 or any(t is None for t in ts):
            raise G.GraphError("TYPE_MISMATCH", "design.blocks." + n, "Data Type Duplicate inputs have the types %s "
                               "(Simulink:DataType:InputPortDataTypeMismatch; the engine does not back-propagate "
                               "types)" % ", ".join(d.type_str(x) if x in d.otype else "?" for x in blks))
    for n, port, (s, sp) in info.get("int_index", []):
        t = d.otype.get(s)
        if t == G.BOOL or t is None or t[2] != 0:
            raise G.GraphError("PORT_TYPE_INVALID", "design.blocks." + n, "Direct Lookup index %s has type %s; it "
                               "must be an integer type (Simulink:blocks:DirectLookupNdInvIndexPortType)"
                               % (port, d.type_str(s) if s in d.otype else "?"))
    for n, ic, s in info["ic_grid"]:
        t = d.otype.get(n)
        if t == G.BOOL and n.startswith(INTERNAL):
            # Detect Rise Positive on a boolean input: vinit 0 or 1
            if str(ic).strip() not in ("0", "1", "0.0", "1.0"):
                _refuse("design.blocks." + n, "vinit %r with a boolean input (0 or 1 observed)" % ic)
            continue
        if t == G.BOOL and n in info.get("ic_bool_ok", ()):
            continue    # boolean data, initial value 0 / 1 (checked in _sync)
        if t == G.BOOL and ((info.get("_spec") or {}).get("blocks", {}).get(n) or {}).get("type") == "EnabledOutport":
            # the reason named a synchronous delay the design does not have
            _refuse("design.blocks.%s.InitialOutput" % n, "a boolean conditional-subsystem output with InitialOutput %r: "
                    "0 and 1 were recorded; not supported" % ic)
        if t == G.BOOL:
            _refuse("design.blocks." + n, "synchronous delay of a boolean signal (not observed)")
        if t is None:
            continue
        try:
            # the value as the block's own reading took it ('[3]', '1+1', '2^3' are numbers there)
            import parameter_bounds
            v = Fraction(float(str(parameter_bounds.number(ic, "design.blocks.%s.InitialCondition" % n)))) * \
                (Fraction(2) ** t[2])
        except (ValueError, OverflowError):
            raise G.GraphError("BAD_PARAM", "design.blocks.%s.InitialCondition" % n, "numeric literal required")
        lo, hi = (-(1 << (t[1] - 1)), (1 << (t[1] - 1)) - 1) if t[0] else (0, (1 << t[1]) - 1)
        edge_tie = v == hi + Fraction(1, 2) or v == lo - Fraction(1, 2)   # max + 0.5, min - 0.5
        if (v > hi + 1 or v <= lo - 1) and n not in info.get("ram_ic", ()):
            # recordings (five delay blocks, sfix16_En8 / ufix8_En4): beyond max + 1 LSB or at / below
            # min - 1 LSB Simulink stops with a parameter overflow
            # (the code of the engine's one parameter-overflow check, fxgraph_lib._param_overflow: PORT_TYPE_INVALID --
            # a Tapped Delay and its Enabled Synchronous form answered the same stop with different codes)
            raise G.GraphError("PORT_TYPE_INVALID", "design.blocks.%s.InitialCondition" % n, ('initial condition %s overflows the input type %s '
                '(Simulink: SimulinkFixedPoint:util:fxpParameterOverflow; '
                'allowed from min - 0.5 LSB to max + 1 LSB)') % (ic, d.type_str(n)))
        if lo - 1 < v < lo - Fraction(1, 2) and n not in info.get("ram_ic", ()):
            _refuse("design.blocks.%s.InitialCondition" % n, "initial condition %s lies between min - 1 LSB and min "
                    "- 0.5 LSB of %s: not recorded (min - 0.5 LSB starts from the minimum, min - 1 "
                    "LSB is an error)" % (ic, d.type_str(n)))
        if v.denominator == 2 and n not in info.get("ic_ties_nearest", ()) and not edge_tie:
            # exactly half-way between two values of the type -- no such initial value was recorded
            # (the port Delay's ties are recorded: Nearest, +0.5 LSB -> 1, -0.5 -> 0, +1.5 -> 2)
            _refuse("design.blocks.%s.InitialCondition" % n, "initial condition %s is exactly half-way between two "
                    "values of %s; its rounding was not recorded" % (ic, d.type_str(n)))
        r = int(v.numerator * 2 + v.denominator) // (2 * v.denominator)   # nearest, a tie towards +inf
        if not lo <= r <= hi and n in info.get("ram_ic", ()):
            # a System RAM's initial value outside its
            # data type is accepted by Simulink and saturated silently (-1 on ufix18 -> 0, 300 on uint8 -> 255);
            # not opened here -- refused as not computed, not as a Simulink error
            own = n[len(INTERNAL):].rsplit("/", 1)[0] if n.startswith(INTERNAL) else n
            _refuse("design.blocks.%s.RAMInitialValue" % own, "RAM initial value %s is outside the data type %s: "
                    "Simulink saturates it silently in a System RAM where the parameter-overflow "
                    "diagnostic allows; not computed here -- write a value the type holds"
                    % (ic, d.type_str(n)))
        if lo - Fraction(1, 2) <= v <= hi + 1 and n not in info.get("ram_ic", ()):
            # from min - 0.5 LSB to max + 1 LSB Simulink runs from the nearest value,
            # saturated to the type (a precision-loss warning); it was refused here because the value was rounded
            # before the check
            continue
        if not lo <= r <= hi:
            _refuse("design.blocks.%s.InitialCondition" % n, "initial condition %s is outside the input type %s "
                    "(not recorded for this block) -- write a value the type holds" % (ic, d.type_str(n)))


# ---------------------------------------------------------------- Bit Slice / Bit Concat / Bit Shift (native)
# Output types depend on the input word lengths, so these run as native kernels on the Python path. Rules from
# recordings (every recorded step agrees):
#   Bit Slice  y = bits ridx..lidx of the input's two's-complement pattern, as an unsigned integer of
#              lidx - ridx + 1 bits; bits above the word (lidx = word length) are 0; ridx > lidx is refused.
#   Bit Concat y = the inputs' bit patterns side by side, input 1 most significant; a boolean is 1 bit; unsigned
#              integer of the summed word lengths.
#   Bit Shift  y keeps the input type; logical shifts move the bit pattern (left: bits out of the word are lost),
#              arithmetic right keeps the sign; N = word length is accepted (0, or the sign for arithmetic).
#   Bit Rotate / Bit Set / Bit Clear: y keeps the input type and works on the
#              stored integer's W-bit pattern: rotate left / right by N mod W; set / clear bit iBit (a bit outside the
#              word has no effect). Bit Rotate has no C-core instruction (Python path, like Bit Concat).
BIT_TYPES = ("BitSlice", "BitConcat", "BitShift", "BitRotate", "BitSet", "BitClear")
BIT_DEFAULTS = {"BitSlice": {"lidx": "7", "ridx": "0"}, "BitConcat": {"numInputs": "2"},
                "BitShift": {"mode": "Shift Left Logical", "N": "0"}}
SHIFT_MODES = ("Shift Left Logical", "Shift Right Logical", "Shift Right Arithmetic")
ROTATE_MODES = ("Rotate Left", "Rotate Right")


def _int_param(p, k, where):
    try:
        return _pint(p[k], where + "." + k)
    except KeyError:
        raise G.GraphError("BAD_PARAM", where + "." + k, "integer %s required" % k)


def bit_ninputs(bt, p, where):
    if bt != "BitConcat":
        return 1
    n = _int_param(p, "numInputs", where)
    if not 1 <= n <= 128:
        raise G.GraphError("BAD_PARAM", where + ".numInputs", "1..128 inputs")
    return n


CAT_SWEPT_FL = (-16, 48)   # -2 .. 8; recordings (22 models): -16 .. 48
# the Bit Slice's recorded inputs and ranges
BITSLICE_MAX_IN, BITSLICE_FL, BITSLICE_MAX_L, BITSLICE_MAX_R = 128, (-3, 31), 200, 127


def _cat_recorded_types(m, ts, where):
    """recordings (80 models equal): 1 .. 128 inputs -- boolean, signed / unsigned of 1 .. 64 bits, fraction lengths
    -2 .. 8 -- concatenated into 1 .. 265 bits (the widths: parameter_bounds.WIDTHS). Kept as before recordings: 2 .. 4 inputs
    with any fraction length. Like the other recorded-range checks, off when
    parameter_bounds.ENFORCE is (the regression check's internal-consistency runs)."""
    import parameter_bounds as LB
    if not LB.ENFORCE[0] or 2 <= m <= 4:
        return
    for k, t in enumerate(ts):
        if t != G.BOOL and not CAT_SWEPT_FL[0] <= t[2] <= CAT_SWEPT_FL[1]:
            _refuse(where + ".numInputs", "Bit Concat of %d inputs with input %d of type %s: recorded with fraction "
                    'lengths %d.. %d for counts other than 2.. 4' % (
                        m, k + 1, G._tstr(t), CAT_SWEPT_FL[0], CAT_SWEPT_FL[1]))


def bit_out_type(bt, p, ts, where):
    for t in ts:
        if t == G.BOOL and bt in ("BitSet", "BitClear") and str(p.get("iBit", "")).strip() == "0":
            # Bit Set / Bit Clear of bit 0 of a boolean -- a boolean that
            # is always true / always false (other bit indices on a boolean not recorded)
            return G.BOOL
        if t == G.BOOL and bt != "BitConcat":
            _refuse(where, "%s of a boolean signal (not observed)" % bt)
    if bt == "BitShift":
        if p.get("mode") not in SHIFT_MODES:
            raise G.GraphError("BAD_PARAM", where + ".mode", "one of %s" % ", ".join(SHIFT_MODES))
        nn = _int_param(p, "N", where)
        if not 0 <= nn <= ts[0][1]:
            _refuse(where + ".N", "shift %d outside 0..word length %d (not observed)" % (nn, ts[0][1]))
        return ts[0]
    if bt == "BitRotate":
        if p.get("mode") not in ROTATE_MODES:
            raise G.GraphError("BAD_PARAM", where + ".mode", "one of %s" % ", ".join(ROTATE_MODES))
        _int_param(p, "N", where)   # the recorded range: parameter_bounds.BOUNDS
        return ts[0]
    if bt in ("BitSet", "BitClear"):
        _int_param(p, "iBit", where)
        return ts[0]
    if bt == "BitSlice":
        hi, lo = _int_param(p, "lidx", where), _int_param(p, "ridx", where)
        if lo > hi:
            raise G.GraphError("BAD_PARAM", where + ".ridx", "ridx %d > lidx %d (Simulink refuses it)" % (lo, hi))
        if lo < 0:
            raise G.GraphError("BAD_PARAM", where + ".ridx", "a negative bit position (Simulink refuses it)")
        # recordings (171 models equal): the bits ridx..lidx of the input's two's-complement pattern, as
        # an unsigned integer of lidx - ridx + 1 bits; bits above the word read 0 (no sign extension), whatever the
        # input's sign and fraction length -- recorded on inputs of 1 .. 128 bits, fraction lengths -2 .. 31 (-3), lidx up
        # to 200 (ridx up to 127); fraction length -3 from recordings (fixdt(1,7,-3),
        # bits 0..6 -- the recordings bound -2 had dropped it)
        if INTERNAL not in where and (ts[0][1] > BITSLICE_MAX_IN or not BITSLICE_FL[0] <= ts[0][2] <= BITSLICE_FL[1]
                                      or hi > BITSLICE_MAX_L or lo > BITSLICE_MAX_R):
            # (a lowering's own slices -- Bit Concat's -- take the bit pattern of any word)
            _refuse(where + ".lidx", 'bit range %d..%d of a %s input: recorded inputs of 1.. %d bits with fraction '
                    'lengths %d.. %d, lidx up to %d and ridx up to %d' % (
                        lo, hi, G._tstr(ts[0]), BITSLICE_MAX_IN, BITSLICE_FL[0], BITSLICE_FL[1], BITSLICE_MAX_L,
                        BITSLICE_MAX_R))
        return (0, hi - lo + 1, 0)
    _cat_recorded_types(len(ts), ts, where)
    w = sum(1 if t == G.BOOL else t[1] for t in ts)
    if w > G.MAX_WL:
        raise G.GraphError("UNSUPPORTED_WIDTH", where, "concatenated word of %d bits" % w)
    return (0, w, 0)


def bit_kernel(bt, p, ts):
    if bt in ("BitSet", "BitClear") and ts and ts[0] == G.BOOL:
        val = 1 if bt == "BitSet" else 0   # bit 0 of a boolean (bit_out_type): set -> true, clear -> false
        return lambda v: val
    if bt == "BitShift":
        s, w = ts[0][0], ts[0][1]
        nn, mode = int(str(p["N"]).strip()), p["mode"]
        mask = (1 << w) - 1

        def wrap(v):
            v &= mask
            return v - (1 << w) if s and v >> (w - 1) else v
        if mode == "Shift Left Logical":
            return lambda v: wrap((v[0] & mask) << nn)
        if mode == "Shift Right Logical":
            return lambda v: wrap((v[0] & mask) >> nn)
        return lambda v: wrap(v[0] >> nn)
    if bt in ("BitRotate", "BitSet", "BitClear"):
        s, w = ts[0][0], ts[0][1]
        mask = (1 << w) - 1

        def wrap2(v):
            v &= mask
            return v - (1 << w) if s and v >> (w - 1) else v
        if bt == "BitRotate":
            k = int(str(p["N"]).strip()) % w
            if p["mode"] == "Rotate Right":
                k = (w - k) % w
            return lambda v: wrap2((((v[0] & mask) << k) | ((v[0] & mask) >> (w - k))) if k else v[0])
        ib_ = int(str(p["iBit"]).strip())
        # a bit outside the word: no effect (iBit 12 on uint8; recordings: iBit -1 and iBit = word length too)
        bit = (1 << ib_) & mask if ib_ >= 0 else 0
        if bt == "BitSet":
            return lambda v: wrap2(v[0] | bit)
        return lambda v: wrap2(v[0] & ~bit)
    if bt == "BitSlice":
        w = ts[0][1]
        hi, lo = int(str(p["lidx"]).strip()), int(str(p["ridx"]).strip())
        m = (1 << (hi - lo + 1)) - 1
        return lambda v: ((v[0] & ((1 << w) - 1)) >> lo) & m
    ws = [1 if t == G.BOOL else t[1] for t in ts]

    def cat(v):
        y = 0
        for x, w in zip(v, ws):
            y = (y << w) | (int(x) & ((1 << w) - 1))
        return y
    return cat


def bit_vm(bt, p, ts, n, src):
    """The C-core instruction for Bit Shift / Bit Slice (None: Python path only)."""
    if bt in ("BitConcat", "BitRotate") or (ts and ts[0] == G.BOOL):
        return None
    s, w = ts[0][0], ts[0][1]
    if bt in ("BitSet", "BitClear"):
        # BITW at the input word length: OR with the bit / AND with every other bit, sign-wrapped into the type
        ib_ = int(str(p["iBit"]).strip())
        bit = (1 << ib_) & ((1 << w) - 1) if ib_ >= 0 else 0
        if bt == "BitSet":
            return ("BITW", [n], (1, s, w, 1, bit, list(src)))
        return ("BITW", [n], (0, s, w, 1, ((1 << w) - 1) & ~bit, list(src)))
    if bt == "BitShift":
        nn, mode = int(str(p["N"]).strip()), p["mode"]
        if mode == "Shift Left Logical":
            return ("SHIFT", [n, src[0]], "%d %d %d %d" % (1, nn, s, w))
        if mode == "Shift Right Arithmetic" or not s or nn == 0:
            return ("SHIFT", [n, src[0]], "%d %d %d %d" % (0, nn, s, w))
        if nn >= w:
            return ("SHIFT", [n, src[0]], "%d %d %d %d" % (1, 127, s, w))   # every bit shifted out: 0
        return ("EXTR", [n, src[0]], "%d %d %d" % (nn, w - nn, 0))         # logical right of a signed word
    hi, lo = int(str(p["lidx"]).strip()), int(str(p["ridx"]).strip())
    if lo >= w or w > 126 or hi - lo + 1 > 126:
        return None   # words over the core's 126 bits stay on the Python path (EXTR took them: PROTOCOL)
    nb = min(hi, w - 1) - lo + 1   # a bit above the word reads 0
    return ("EXTR", [n, src[0]], "%d %d %d" % (lo, nb, 0))



# ---------------------------------------------------------------- enabled subsystems
ES_DEFAULTS = {"EnablePort": {"StatesWhenEnabling": "held", "ShowOutputPort": "off"},
               "StateControl": {"StateControl": "Classic"},
               "EnabledOutport": {"OutputWhenDisabled": "held", "InitialOutput": "[]"}}
ES_STATEFUL_OK = ("UnitDelay", "Hold")


def _owner(name, subs):
    """Innermost enabled subsystem (prefix) holding block `name` (internal names count by the block they belong to)."""
    base = name.lstrip(INTERNAL)
    best = None
    for p in subs:
        if base.startswith(p + "/") and (best is None or len(p) > len(best)):
            best = p
    return best


def _enabled_subsystems(spec, info):
    """A flat design marks an enabled subsystem <S> by a block <S>/... of type EnablePort (its one input is the enable
    signal); <S>/... of type StateControl (Classic | Synchronous) and EnabledOutport blocks (one input from inside,
    the subsystem's output; OutputWhenDisabled held | reset, InitialOutput) complete it. Rules (every
    sample): Classic -- disabled, the states hold and every output is held (held) or InitialOutput (reset), before the
    first enable the held value is InitialOutput; StatesWhenEnabling reset puts every state back to its initial value in
    the step the enable rises, then computes as usual. Synchronous -- the combinational part runs every step, only the
    states update just when enabled, OutputWhenDisabled and StatesWhenEnabling have no effect. Nested: a subsystem
    runs only when every enclosing enable is true; a non-zero enable is true."""
    blocks = dict(spec["blocks"])
    lines = list(spec["lines"])
    ports = {n: b for n, b in blocks.items() if b.get("type") == "EnablePort"}
    subs = {}
    for n, b in ports.items():
        if "/" not in n:
            _refuse("design.blocks." + n, "an EnablePort must sit inside a subsystem level (S/Enable)", "BAD_PARAM")
        S = n.rsplit("/", 1)[0]
        if S in subs:
            _refuse("design.blocks." + n, "two EnablePort blocks in %s" % S, "BAD_PARAM")
        subs[S] = n
    for ram in info.get("ram", []):
        if subs and _owner(ram[0], subs) is not None and ram[0] in info.get("ram_deep", {}):
            # the deep core (RAMCore) would keep writing while disabled; RAMs in enabled subsystems were recorded with
            # 3 address bits only (held / reset outputs, Synchronous) -- those are lowered to
            # Holds and Unit Delays, which the enabled subsystem gates like any state
            _refuse("design.blocks." + ram[0], "a deep HDL RAM (%d address bits) inside an enabled subsystem (recorded "
                    "with 3 address bits only)" % info["ram_deep"][ram[0]][0])
    for n, b in blocks.items():
        if b.get("type") in ("StateControl", "EnabledOutport") and (_owner(n, subs) is None or
                                                                    n.rsplit("/", 1)[0] not in subs):
            _refuse("design.blocks." + n, "%s outside an enabled subsystem (no EnablePort at %s)"
                    % (b["type"], n.rsplit("/", 1)[0]), "BAD_PARAM")
    params = {}
    for n, b in blocks.items():
        t = b.get("type")
        if t in ES_TYPES:
            p = dict(ES_DEFAULTS[t], **{k: str(v) for k, v in b.items() if k != "type" and not (
                k == "_EnSigned" and v is MARK) and k != "_Tie" and k != "_EnType"})   # _Tie: the vector pass's element marker
            for k_ in sorted(set(p) - set(ES_DEFAULTS[t])):
                if G.DESIGN_RANGE_RE.match(k_):
                    # OutMin / OutMax of a conditional subsystem's Outport are real parameters
                    if str(p[k_]).strip() not in ("[]", ""):
                        _refuse("design.blocks.%s.%s" % (n, k_), "design range %s = %r is not supported: Simulink "
                                "uses design minimum / maximum values to choose types and to check parameters, which "
                                "this engine does not model; leave it '[]'" % (k_, str(p[k_]).strip()))
                    del p[k_]
            bad = sorted(set(p) - set(ES_DEFAULTS[t]))
            if bad:
                _refuse("design.blocks.%s.%s" % (n, bad[0]), "unknown parameter %s for %s" % (bad[0], t), "BAD_PARAM")
            if t == "EnablePort" and p["ShowOutputPort"] != "off":
                # a real EnablePort parameter (the library's 'off'); 'on' adds an output port
                _refuse("design.blocks.%s.ShowOutputPort" % n, "EnablePort ShowOutputPort %r (the enable signal as an "
                        "output) is not supported" % p["ShowOutputPort"])
            params[n] = p
    sync = {}
    for S in subs:
        scs = [n for n, b in blocks.items() if b.get("type") == "StateControl" and n.rsplit("/", 1)[0] == S]
        mode = params[scs[0]]["StateControl"] if scs else "Classic"
        if mode not in ("Classic", "Synchronous"):
            _refuse("design.blocks." + scs[0], "StateControl %r (Classic or Synchronous)" % mode, "BAD_PARAM")
        sync[S] = mode == "Synchronous"
    for n, b in ((info.get("_spec") or {}).get("blocks") or blocks).items():
        # recordings (class: the HDL synchronous delays -- Unit Delay Enabled / Resettable Synchronous and the
        # Tapped Delay Enabled Synchronous's stages -- inside a conditionally executed subsystem): Simulink allows them
        # only under a State Control set to Synchronous (NoHardwareInsideCondExecSystemWithClassic); answered before
        if b.get("type") in SYNC and not n.startswith(INTERNAL):
            S_ = _owner(n, subs)
            if S_ is not None and not sync[S_]:
                u_ = n.split("\x1d")[0]
                _refuse("design.blocks." + u_, "%s inside the Classic enabled subsystem %s: Simulink rejects it "
                        "(NoHardwareInsideCondExecSystemWithClassic); add a State Control block set to Synchronous "
                        "in %s" % (b_type_of(b), S_, S_), "PORT_TYPE_INVALID")
    for S in subs:
        if sync[S] and any(S.startswith(T + "/") and not sync[T] for T in subs):
            # a Synchronous enabled subsystem inside a Classic one -- Simulink rejects it
            # (NoHardwareInsideCondExecSystemWithClassic)
            _refuse("design.blocks." + S, "a Synchronous enabled subsystem inside a Classic one: Simulink rejects "
                    "it (NoHardwareInsideCondExecSystemWithClassic); make the outer one Synchronous too",
                    "PORT_TYPE_INVALID")
        if params[subs[S]]["StatesWhenEnabling"] not in ("held", "reset"):
            _refuse("design.blocks.%s.StatesWhenEnabling" % subs[S], "held or reset", "BAD_PARAM")
    src = {}
    for ln in lines:
        src[(ln["dst"], int(ln.get("dport", 1)))] = ln["src"]
    # connections no Simulink model can hold -- the
    # engine answered them and the export could not write them (roundtrip defect, MATLAB_ERROR)
    for ln in lines:
        s_, d_ = ln["src"], ln["dst"]
        if d_ in ports and s_.startswith(d_.rsplit("/", 1)[0] + "/"):
            # X3b: a subsystem's enable is a signal from outside it
            _refuse("design.blocks." + d_, "the enable of %s is driven from inside it (%s); an enable comes from "
                    "outside the subsystem" % (d_.rsplit("/", 1)[0], s_), "BAD_CONNECTION")
        if blocks.get(s_, {}).get("type") == "EnabledOutport":
            S_ = s_.rsplit("/", 1)[0]
            if d_.startswith(S_ + "/"):
                # X4: a subsystem's Outport leaves the subsystem
                _refuse("design.blocks." + s_, "the Outport %s drives %s inside its own subsystem; an Outport's "
                        "signal leaves the subsystem" % (s_, d_), "BAD_CONNECTION")
            for E in subs:
                if E != S_ and S_.startswith(E + "/") and not d_.startswith(E + "/"):
                    # X3: an inner Outport wired past the enabled subsystem around it
                    _refuse("design.blocks." + s_, "the Outport %s of %s drives %s outside the enabled subsystem %s "
                            "around it, without passing through an Outport of %s" % (s_, S_, d_, E, E),
                            "BAD_CONNECTION")
    add, drop, new_lines = {}, set(), []

    def L(a, b, port=1):
        new_lines.append({"src": a, "dst": b, "dport": port})
    # effective enable of each level: boolean NOT(NOT e), ANDed with the enclosing level's
    en = {}
    for S in sorted(subs, key=lambda x: x.count("/")):
        X = INTERNAL + S + "/es/"
        e_src = src.get((subs[S], 1))
        if e_src is None:
            _refuse("design.blocks." + subs[S], "the enable input is not connected", "BAD_CONNECTION")
        info.setdefault("es_enable", []).append((subs[S], (e_src, 1) if isinstance(e_src, str) else tuple(e_src),
                                                 _marked(blocks.get(subs[S], {}), "_EnSigned")))
        if _marked(blocks.get(subs[S], {}), "_EnSigned"):
            # a signed enable is true only when > 0 (Classic and Synchronous); sync_ctrl_mark marks it
            add[X + "pos"] = {"type": "CompareToConstant", "relop": ">", "const": "0", "OutDataTypeStr": "boolean"}
            et_ = blocks.get(subs[S], {}).get("_EnType")
            if isinstance(et_, str):
                # an enable from the subsystem's own output (a loop), lowered again with its compiled
                # type -- a conversion to that same type (an identity) gives the comparison a type source in the loop
                add[X + "ty"] = {"type": "DataTypeConversion", "OutDataTypeStr": et_, "RndMeth": "Floor",
                                 "SaturateOnIntegerOverflow": "off"}
                L(e_src, X + "ty")
                L(X + "ty", X + "pos")
            else:
                L(e_src, X + "pos")
            e = X + "pos"
        else:
            add[X + "n1"] = {"type": "LogicalOperator", "Operator": "NOT", "Inputs": "1"}
            add[X + "n2"] = {"type": "LogicalOperator", "Operator": "NOT", "Inputs": "1"}
            L(e_src, X + "n1")
            L(X + "n1", X + "n2")
            e = X + "n2"
        outer = _owner(S, {k: v for k, v in subs.items() if k != S})
        if outer is not None:
            add[X + "and"] = {"type": "LogicalOperator", "Operator": "AND", "Inputs": "2"}
            L(en[outer], X + "and", 1)
            L(e, X + "and", 2)
            e = X + "and"
        en[S] = e
        drop.add(subs[S])
        if params[subs[S]]["StatesWhenEnabling"] == "reset" and not sync[S]:
            # recordings (high): the edge detector must run on the data's grid. With an enable slower than the data
            # (recordings shape) an inherited-period detector ran on the enable's grid, so `rise` stayed true for
            # the whole enable period and every data step in it put the states back (Simulink 1 2 3 4 ..., engine 1 1
            # 2 3 ...). The enable is ANDed with an always-true signal of a data input of the subsystem (d == d), which
            # puts the detector on the data grid: `rise` is true on the first data step of an enabling only
            ein = e
            # (internal blocks carry the INTERNAL prefix: compare the names without it)
            dl = next((ln for ln in lines if ln["dst"].lstrip(INTERNAL).startswith(S + "/")
                       and not ln["src"].lstrip(INTERNAL).startswith(S + "/")
                       and ln["dst"] != subs[S] and blocks.get(ln["dst"], {}).get("type") != "EnablePort"), None)
            if dl is not None:
                d_ = {"src": dl["src"]}
                if int(dl.get("sport", 1)) != 1:
                    d_["sport"] = int(dl["sport"])
                add[X + "dtrue"] = {"type": "RelationalOperator", "Operator": "=="}
                new_lines.append(dict(d_, dst=X + "dtrue", dport=1))
                new_lines.append(dict(d_, dst=X + "dtrue", dport=2))
                add[X + "edata"] = {"type": "LogicalOperator", "Operator": "AND", "Inputs": "2"}
                L(e, X + "edata", 1)
                L(X + "dtrue", X + "edata", 2)
                ein = X + "edata"
            add[X + "prev"] = {"type": "UnitDelay", "InitialCondition": "0", "SampleTime": "-1"}
            L(ein, X + "prev")
            add[X + "notprev"] = {"type": "LogicalOperator", "Operator": "NOT", "Inputs": "1"}
            L(X + "prev", X + "notprev")
            add[X + "rise"] = {"type": "LogicalOperator", "Operator": "AND", "Inputs": "2"}
            L(ein, X + "rise", 1)
            L(X + "notprev", X + "rise", 2)
    redirect = {}   # old source name -> new source name (an output now comes from a different block)
    for n, b in blocks.items():
        S = _owner(n, subs)
        if S is None or b.get("type") in ES_TYPES:
            continue
        t = b.get("type")
        X = INTERNAL + n.lstrip(INTERNAL) + "/es/"
        if t == "UnitDelay":
            x = src.get((n, 1))
            # the state takes its input only when enabled (Classic and Synchronous alike): a Hold of the input on the
            # enable keeps the last enabled input, which is the state (its type comes from the input, no loop)
            st = X + "state"
            ic = b.get("InitialCondition", "0")
            add[X + "gate"] = {"type": "Hold", "InitialCondition": ic}
            L(x, X + "gate", 1)
            L(en[S], X + "gate", 2)
            drop.add((n, 1))
            if params[subs[S]]["StatesWhenEnabling"] == "reset" and not sync[S]:
                # the block's output is its initial value in the step the enable rises (the state is reset)
                add[st] = dict(b)
                L(X + "gate", st)
                add[X + "f"] = {"type": "RelationalOperator", "Operator": "~="}
                L(st, X + "f", 1)
                L(st, X + "f", 2)
                add[X + "ic"] = {"type": "Hold", "InitialCondition": ic}
                L(st, X + "ic", 1)
                L(X + "f", X + "ic", 2)
                add[n] = {"type": "Switch", "Criteria": "u2 ~= 0", "OutDataTypeStr": G.fc._SAME_FIRST}
                L(X + "ic", n, 1)
                L(INTERNAL + S + "/es/rise", n, 2)
                L(st, n, 3)
            else:
                add[n] = dict(b)
                L(X + "gate", n)
        elif t == "Delay" and not sync[S] and params[subs[S]]["StatesWhenEnabling"] in ("held", "reset") and \
                str(b.get("ShowEnablePort", "off")) == "off" and str(b.get("ExternalReset", "None")) == "None" and \
                str(b.get("DelayLengthSource", "Dialog")) == "Dialog" and not str(b.get("InitialCondition", "0")).strip().startswith("["):
            # a Delay of length N inside an enabled subsystem (held)
            # is N gated stages, each like the Unit Delay above (a Hold of its input on the enable, then one step)
            try:
                N = int(str(b.get("DelayLength", "2")).strip())
            except ValueError:
                _refuse("design.blocks." + n.lstrip(INTERNAL) + ".DelayLength", "integer DelayLength", "BAD_PARAM")
            if not 1 <= N <= 64:
                _refuse("design.blocks." + n.lstrip(INTERNAL) + ".DelayLength", "1 .. 64 inside an enabled subsystem")
            ic = b.get("InitialCondition", "0")
            prev = src.get((n, 1))
            drop.add((n, 1))
            for k in range(1, N + 1):
                g = X + "gate%d" % k
                add[g] = {"type": "Hold", "InitialCondition": ic}
                L(prev, g, 1)
                L(en[S], g, 2)
                z = n if k == N else X + "z%d" % k
                if params[subs[S]]["StatesWhenEnabling"] == "reset":
                    # every stage is reset as the Unit Delay's (its output is
                    # its initial value in the step the enable rises)
                    st, f_, icb = X + "st%d" % k, X + "f%d" % k, X + "ic%d" % k
                    add[st] = {"type": "UnitDelay", "InitialCondition": ic, "SampleTime": str(b.get("SampleTime", "-1"))}
                    L(g, st)
                    add[f_] = {"type": "RelationalOperator", "Operator": "~="}
                    L(st, f_, 1)
                    L(st, f_, 2)
                    add[icb] = {"type": "Hold", "InitialCondition": ic}
                    L(st, icb, 1)
                    L(f_, icb, 2)
                    add[z] = {"type": "Switch", "Criteria": "u2 ~= 0", "OutDataTypeStr": G.fc._SAME_FIRST}
                    L(icb, z, 1)
                    L(INTERNAL + S + "/es/rise", z, 2)
                    L(st, z, 3)
                else:
                    add[z] = {"type": "UnitDelay", "InitialCondition": ic, "SampleTime": str(b.get("SampleTime", "-1"))}
                    L(g, z)
                prev = z
        elif t == "Hold":
            reset = params[subs[S]]["StatesWhenEnabling"] == "reset" and not sync[S]
            mem = [m[0] for m in info.get("ram", []) + info.get("fifo", [])]
            if reset and (not any(n.startswith(INTERNAL + m + "/") for m in mem) or
                          str(b.get("InitialCondition", "0")).strip() not in ("0", "0.0")):
                _refuse("design.blocks." + n.lstrip(INTERNAL), "a block with a held state (synchronous delay, Repeat, "
                        "or a RAM with a non-zero initial value) inside an enabled subsystem with StatesWhenEnabling "
                        "reset (recorded for HDL RAM / FIFO words starting at 0 only)")
            e0 = src.get((n, 2))
            add[X + "en"] = {"type": "LogicalOperator", "Operator": "AND", "Inputs": "2"}
            L(e0, X + "en", 1)
            L(en[S], X + "en", 2)
            drop.add((n, 2))
            add[n] = dict(b)
            drop.add((n, 1))
            if reset:
                # recordings (an HDL RAM / FIFO, StatesWhenEnabling
                # reset): the held word goes back to its initial value (0) in the step the enable rises -- the Hold
                # loads 0 then unless it is written in that step
                x = src.get((n, 1))
                add[X + "f"] = {"type": "RelationalOperator", "Operator": "~="}
                L(x, X + "f", 1)
                L(x, X + "f", 2)
                add[X + "ic"] = {"type": "Hold", "InitialCondition": b.get("InitialCondition", "0")}   # 0 in x's type
                L(x, X + "ic", 1)
                L(X + "f", X + "ic", 2)
                add[X + "nen"] = {"type": "LogicalOperator", "Operator": "NOT", "Inputs": "1"}
                L(X + "en", X + "nen", 1)
                add[X + "clr"] = {"type": "LogicalOperator", "Operator": "AND", "Inputs": "2"}
                L(INTERNAL + S + "/es/rise", X + "clr", 1)
                L(X + "nen", X + "clr", 2)
                add[X + "x"] = {"type": "Switch", "Criteria": "u2 ~= 0", "OutDataTypeStr": G.fc._SAME_FIRST}
                L(X + "ic", X + "x", 1)
                L(X + "clr", X + "x", 2)
                L(x, X + "x", 3)
                add[X + "ld"] = {"type": "LogicalOperator", "Operator": "OR", "Inputs": "2"}
                L(X + "en", X + "ld", 1)
                L(INTERNAL + S + "/es/rise", X + "ld", 2)
                L(X + "x", n, 1)
                L(X + "ld", n, 2)
            else:
                L(src.get((n, 1)), n, 1)
                L(X + "en", n, 2)
        elif t == "MLFn" and sync[S] and _ml_write_before_read(b.get("Script")):
            # recordings (claims): when the persistent is written in the else of its `if isempty` initialisation,
            # Simulink stops with PersistentVariableInitializationTooComplex (3 / 4) -- the message names that one
            elsew = _ml_isempty_else(b.get("Script"))
            _refuse("design.blocks." + str(b.get("Owner", n)).lstrip(INTERNAL), "a MATLAB Function that writes a "
                    "persistent before an output reads it, inside a Synchronous subsystem: Simulink rejects it (%s); "
                    "compute the output from the old value first, then update the persistent%s"
                    % (('Stateflow:cdr:PersistentVariableInitializationTooComplex '
                        'for an update in the else of the `if isempty` initialisation') if elsew else
                       'Stateflow:cdr:WriteToPersistentVariableInOutputsFunction',
                       " outside that if" if elsew else ""), "PORT_TYPE_INVALID")
        elif t == "MLFn" and _ml_persistent(b.get("Script")):
            # a MATLAB Function with persistent state on its
            # interpreter cores keeps running while the subsystem is disabled (held: 1 / 1, reset: 5 / 5 packages
            # FAIL); its compiled form in a held subsystem is gated like its parts (3 / 3 PASS)
            info.setdefault("es_kernels", []).append((str(b.get("Owner", n)).lstrip(INTERNAL), t))
        elif t in ("NCOCore", "HDLFIRCore", "RAMCore"):
            # these exact kernels keep state and stepped on every
            # base step inside an enabled subsystem -- the kernel HDL NCO (dither, increment / offset / reset ports,
            # phase output, quantiser > 14 bits) and the exact-kernel HDL FIR ran while disabled (packages 29 / 29 and
            # 1 / 1 FAIL); the deep RAM kernel likewise. Refused when the design runs (es_kernel_check); a type probe
            # of a block the typed rewrite lowers to basic blocks (gated like their parts) still compiles
            info.setdefault("es_kernels", []).append((str(b.get("Owner", n)).lstrip(INTERNAL), t))
        elif t in G.DSTATEFUL + ("Delay", "DiscreteFir", "Lookup1D", "Lookup2D") and t not in ("Lookup1D", "Lookup2D") \
                or t in G.MR_BLOCKS:
            _refuse("design.blocks." + n.lstrip(INTERNAL), "%s inside an enabled subsystem is not supported yet (Unit "
                    "Delay, HDL Counter, synchronous delays and stateless blocks are)" % t)
    for n, b in blocks.items():
        if b.get("type") != "EnabledOutport":
            continue
        S = n.rsplit("/", 1)[0]
        X = INTERNAL + n + "/es/"
        x = src.get((n, 1))
        if x is None:
            _refuse("design.blocks." + n, "EnabledOutport input is not connected", "BAD_CONNECTION")
        p = params[n]
        ic = p["InitialOutput"].strip()
        if ic in ("[]", ""):
            if p["OutputWhenDisabled"] == "reset":
                # Simulink refuses a reset Outport without an initial output
                # (Simulink:blocks:NoInitOutForOutportReset)
                _refuse("design.blocks." + n, "an enabled subsystem Outport that resets when disabled needs an "
                        "InitialOutput (Simulink: NoInitOutForOutportReset)", "BAD_PARAM")
            xn = x[0] if isinstance(x, tuple) else x
            drv = blocks.get(xn, {}).get("type")
            if drv == "EnabledOutport" and str(params.get(xn, blocks[xn]).get("InitialOutput", "[]")).strip() in ("[]", ""):
                drv = None   # both Outports at [] (recordings: 1 / 1 equal)
            if drv in ("Constant", "EnabledOutport") and not xn.lstrip(INTERNAL).startswith(S.lstrip(INTERNAL) + "/"):
                # recordings (b06 n09 a root Constant, b04 a16 a
                # sibling subsystem's Outport, each reaching the Outport through this subsystem's Inport): Simulink
                # starts from 0 there, as the engine does -- only a driver inside the subsystem sets the start
                drv = None
            if drv in ("Constant", "EnabledOutport"):
                # recordings (7 / 7 wrong): with InitialOutput [] Simulink takes the
                # initial output from the block driving the Outport -- a Constant's value from step 0, a nested
                # conditional Outport's value (which it then follows while disabled); the rule is not determined by
                # the recordings beyond these shapes (a Gain, Unit Delay or Sum in between gave 0, as here)
                _refuse("design.blocks.%s.InitialOutput" % n, "an enabled subsystem Outport with InitialOutput [] "
                        "driven directly by a %s: Simulink starts it from that block's value, which is not modelled; "
                        "give InitialOutput explicitly" % ("Constant" if drv == "Constant" else
                                                           "conditional subsystem's Outport"))
        ic = "0" if ic in ("[]", "") else ic
        try:
            _num(ic, "design.blocks.%s.InitialOutput" % n)
        except G.GraphError as e_:
            if e_.code != "BAD_PARAM":
                raise
            # Simulink evaluates '1+1', '2^3', '[3]', 'single(2)', '[1 2]' here and runs; only a plain
            # number is read by this engine -- not supported, not a request error
            _refuse("design.blocks.%s.InitialOutput" % n, "InitialOutput %r: only a plain number is supported (Simulink "
                    "evaluates this expression; write its value)" % ic)
        drop.add((n, 1))
        if sync[S]:
            add[n] = {"type": "Hold", "InitialCondition": ic}   # Synchronous: passes every step
            add[X + "t"] = {"type": "LogicalOperator", "Operator": "NOT", "Inputs": "1"}
            add[X + "f"] = {"type": "RelationalOperator", "Operator": "~="}
            L(x, X + "f", 1)
            L(x, X + "f", 2)
            L(X + "f", X + "t")
            L(x, n, 1)
            L(X + "t", n, 2)
        elif p["OutputWhenDisabled"] == "held":
            add[n] = {"type": "Hold", "InitialCondition": ic}
            L(x, n, 1)
            L(en[S], n, 2)
        elif p["OutputWhenDisabled"] == "reset":
            add[X + "f"] = {"type": "RelationalOperator", "Operator": "~="}
            L(x, X + "f", 1)
            L(x, X + "f", 2)
            add[X + "ic"] = {"type": "Hold", "InitialCondition": ic}
            L(x, X + "ic", 1)
            L(X + "f", X + "ic", 2)
            add[n] = {"type": "Switch", "Criteria": "u2 ~= 0", "OutDataTypeStr": G.fc._SAME_FIRST}
            L(x, n, 1)
            L(en[S], n, 2)
            L(X + "ic", n, 3)
        else:
            _refuse("design.blocks.%s.OutputWhenDisabled" % n, "held or reset", "BAD_PARAM")
        info["ic_grid"].append((n, ic, None))
        if str(ic).strip() in ("0", "1", "0.0", "1.0"):
            # a boolean Outport (held, InitialOutput 1) -- its initial value is that boolean
            info.setdefault("ic_bool_ok", set()).add(n)
    # signals may leave an enabled subsystem only through an EnabledOutport
    for ln in lines:
        a, b = ln["src"], ln["dst"]
        Sa, Sb = _owner(a, subs), _owner(b, subs)
        if Sa is not None and blocks[a].get("type") != "EnabledOutport" and Sa != Sb and \
                not (Sb is not None and Sb.startswith(Sa + "/")):
            _refuse("design.lines", "%s leaves the enabled subsystem %s without an EnabledOutport" % (a, Sa),
                    "BAD_CONNECTION")
    out_blocks = {k: v for k, v in blocks.items() if k not in drop and v.get("type") not in ("StateControl", "EnablePort")}
    out_blocks.update(add)
    out_lines = [l for l in lines if (l["dst"], int(l.get("dport", 1))) not in drop and l["dst"] not in subs.values()
                 and blocks.get(l["dst"], {}).get("type") != "StateControl"]
    return {"blocks": out_blocks, "lines": out_lines + new_lines}


# ---------------------------------------------------------------- MATLAB Function (engine/fxmlsem.py)
ML_DEFAULTS = {"Script": None, "TreatAsFi": "Fixed-point", "InputFimath": "",
               "EmlDefaultFimath": "Same as MATLAB Default", "SaturateOnIntegerOverflow": "on"}
_ML_CACHE = {}


def _ml_err(e, where):
    from fxgraph_lib import GraphError
    return GraphError(e.code, where + ".Script", "line %d: %s" % (e.line, e.message))


def _ml_ast(script, where):
    import fxmlfn
    if not isinstance(script, str) or not script.strip():
        _refuse(where + ".Script", "the MATLAB Function needs its script text", "MISSING_FIELD")
    hit = _ML_CACHE.get(("ast", script))
    if hit is None:
        try:
            hit = fxmlfn.validate(fxmlfn.parse(script))
        except fxmlfn.MlError as e:
            raise _ml_err(e, where)
        if len(_ML_CACHE) > 256:
            _ML_CACHE.clear()
        _ML_CACHE[("ast", script)] = hit
    return hit


def _ml_main(script, where):
    ast = _ml_ast(script, where)
    return next(f for f in ast["functions"] if f["name"] == ast["main"])


def _ml_widths(b, key):
    import json
    v = b.get(key)
    return json.loads(v) if v else None


def ml_inputs(b, where):
    """Input port names: one per argument, or one per element when the vector pass gave the widths (_InWidths /
    InWidths: a vector argument a of width w comes in as a[1] .. a[w])."""
    names = list(_ml_main(b.get("Script"), where)["inputs"])
    ws = _ml_widths(b, "_InWidths") or _ml_widths(b, "InWidths")
    if not ws:
        return names
    return [nm if w == 1 else "%s[%d]" % (nm, j) for nm, w in zip(names, ws) for j in range(1, w + 1)]


def _ml_out_key(k):
    return "OutDataTypeStr" if k == 1 else "OutDataTypeStr%d" % k


def _mlfn(n, b, X, src, add, wire, info, where, ports):
    """MATLAB Function block (v1 subset): one internal MLFn block per output port, each running the whole function
    on the same inputs (the persistent state evolves identically in every copy); output k > 1 is X + 'y<k>'."""
    main = _ml_main(b.get("Script"), where)
    nout = len(main["outputs"])
    outs = {_ml_out_key(k) for k in range(1, nout + 1)}
    p = {}
    for k, v in b.items():
        if k == "type":
            continue
        if k not in ML_DEFAULTS and k not in outs and k not in ("_InWidths", "_OutWidths"):
            _refuse(where + "." + k, "unknown parameter %s for MATLABFunction (Script, %s, TreatAsFi, InputFimath, "
                    "EmlDefaultFimath, SaturateOnIntegerOverflow)" % (k, "/".join(sorted(outs))), "BAD_PARAM")
        if not isinstance(v, str):
            _refuse(where + "." + k, "text value required", "BAD_PARAM")
        p[k] = v
    if p.get("SaturateOnIntegerOverflow", "on") not in ("on", "off"):
        _refuse(where + ".SaturateOnIntegerOverflow", "on or off", "BAD_PARAM")
    if nout == 0:
        _refuse(where + ".Script", "the function has no output", "UNSUPPORTED")
    common = {k: v for k, v in p.items() if k not in outs and k not in ("_InWidths", "_OutWidths")}
    in_names = ml_inputs(b, where)
    ow = _ml_widths(b, "_OutWidths") or [1] * nout
    if "_InWidths" in p:
        common["InWidths"] = p["_InWidths"]
    names = {}
    flat = 0
    for k in range(1, nout + 1):
        for e in range(1, ow[k - 1] + 1):
            # one core per output element; flat output port numbers (the vector pass's numbering) k1e1, k1e2, ...
            flat += 1
            nm = n if flat == 1 else (X + "y%d" % k if ow[k - 1] == 1 else X + "y%d_%d" % (k, e))
            blk = dict(common, type="MLFn", Port=str(k), Ports=str(nout), Owner=n)
            if ow[k - 1] > 1:
                blk["Elem"] = str(e)
            for j in range(1, nout + 1):   # every copy checks every declared output type (Simulink refuses any)
                if _ml_out_key(j) in p:
                    blk["_Out%d" % j] = p[_ml_out_key(j)]
            if _ml_out_key(k) in p:
                blk["OutDataTypeStr"] = p[_ml_out_key(k)]
            add[nm] = blk
            names[flat] = nm
            for j, inp in enumerate(in_names):
                wire(src[inp], nm, j + 1)
    if flat > 1:
        info.setdefault("ports", {})[n] = {k: v for k, v in names.items() if k > 1}


def _ml_sem_type(t, where, k):
    from fxgraph_lib import BOOL
    if t == BOOL:
        return ("bool",)
    s, wl, fl = t
    if fl == 0 and wl in (8, 16, 32):   # Simulink names fixdt(s, 8/16/32, 0) int8 ... uint32: an integer class
        return ("int", ("int%d" if s else "uint%d") % wl)
    return ("fi", s, wl, fl)


def _ml_eng_type(t):
    from fxgraph_lib import BOOL
    import fxmlsem
    if t[0] == "bool":
        return BOOL
    if t[0] == "int":
        s, wl = fxmlsem.INT_CLASSES[t[1]]
        return (s, wl, 0)
    if t[0] != "fi":
        # a double output (e.g. y = 1 in every branch, recordings): the engine has no double
        # signals
        raise G_unsupported("a MATLAB Function output of type %s: double signals are not supported (fixed point, "
                            "integers and boolean only); give the value a fixed-point or integer type in the code"
                            % fxmlsem.tstr(t))
    return (t[1], t[2], t[3])


def G_unsupported(msg):
    from fxgraph_lib import GraphError
    return GraphError("UNSUPPORTED", "MATLAB Function output", msg)


def _ml_declared(text, where):
    import fxmlsem
    import re
    s = text.strip()
    if s == "Inherit: Same as Simulink":
        return None
    m = re.fullmatch(r"fixdt\(\s*([01])\s*,\s*(\d+)\s*,\s*(-?\d+)\s*\)", s)
    if m:
        return ("fi", int(m.group(1)), int(m.group(2)), int(m.group(3)))
    if s in fxmlsem.INT_CLASSES:
        return ("int", s)
    if s == "boolean":
        return ("bool",)
    _refuse(where, "MATLAB Function output type %r (Inherit: Same as Simulink, fixdt(s,w,f), int8 ... uint32, "
            "boolean)" % text)


def _ml_program(p, ts, where):
    import fxmlsem
    import fxmlfn
    if any(t is None for t in ts):
        _refuse(where, "an input type is not known", "INTERNAL")
    ws = _ml_widths(p, "InWidths")
    if ws:   # vector arguments: their elements must share one type (a vector signal has one type)
        sem_in, i = [], 0
        for w in ws:
            el = ts[i:i + w]
            i += w
            if any(t != el[0] for t in el):
                _refuse(where, "the elements of a vector input have different types %s" % (el,), "TYPE_MISMATCH")
            st = _ml_sem_type(el[0], where, i)
            sem_in.append(("vec", w, st) if w > 1 else st)
    else:
        sem_in = [_ml_sem_type(t, where, k) for k, t in enumerate(ts)]
    nout = int(p.get("Ports", "1"))
    declared = []
    for j in range(1, nout + 1):
        txt = p.get("OutDataTypeStr") if j == int(p.get("Port", "1")) else p.get("_Out%d" % j)
        declared.append(_ml_declared(txt, where + "." + _ml_out_key(j)) if txt else None)
    chart = {"TreatAsFi": p.get("TreatAsFi", "Fixed-point"), "InputFimath": p.get("InputFimath", ""),
             "EmlDefaultFimath": p.get("EmlDefaultFimath", "Same as MATLAB Default"),
             "SaturateOnIntegerOverflow": p.get("SaturateOnIntegerOverflow", "on") == "on"}
    try:
        return fxmlsem.compile(p["Script"], sem_in, chart, declared)
    except fxmlfn.MlError as e:
        raise _ml_err(e, where)


def ml_out_type(p, ts, where):
    where = _owner_where(p, where)
    prog = _ml_program(p, ts, where)
    t = prog.out_types[int(p.get("Port", "1")) - 1]
    if t[0] == "vec":
        if not p.get("Elem"):
            _refuse(where, "output %s is a vector, but the block was not given its width" % p.get("Port"), "INTERNAL")
        t = t[2]
    return _ml_eng_type(t)


class MlKernel:
    """Python path of one MLFn output: the whole function per step (exact integers, engine/fxmlsem.py)."""

    def __init__(self, p, ts, where):
        self.p, self.ts, self.where = p, list(ts), _owner_where(p, where)
        self.k = int(p.get("Port", "1")) - 1
        self.reset()

        self.ws = _ml_widths(p, "InWidths")
        self.e = int(p["Elem"]) - 1 if p.get("Elem") else None

    def reset(self):
        self.prog = _ml_program(self.p, self.ts, self.where)

    def __call__(self, v):
        if self.ws:
            g, i = [], 0
            for w in self.ws:
                g.append(list(v[i:i + w]) if w > 1 else v[i])
                i += w
            v = g
        import fxmlfn
        try:
            y = self.prog.step(v)[self.k]
        except fxmlfn.MlError as e:
            # a refusal the interpreter raises while running (its run-time guards) is a structured
            # refusal of this block, not an INTERNAL error of the tool
            raise G.GraphError(e.code, self.where, "line %d: %s" % (e.line, e.message))
        return y[self.e] if self.e is not None else y


# ---------------------------------------------------------------- DSP HDL FIR Filter / FIR Decimator
def _hdlfir(n, b, X, src, add, wire, info, where):
    """Internal HDLFIRCore blocks (port 1 data = the block's own name, port 2 valid = X + 'valid', port 3 ready =
    X + 'ready' for a partly serial filter) running the same exact kernel (engine/fxhdlfir.py)."""
    import json
    import fxhdlfir
    p = fxhdlfir.params(b["type"], b, where)
    txt = json.dumps(p, sort_keys=True)
    outs = [(1, n), (2, X + "valid")]
    if b["type"] == "HDLFIRFilter" and p["FilterStructure"] == "Partly serial systolic":
        outs.append((3, X + "ready"))   # the partly serial filter's ready output
    for k, nm in outs:
        add[nm] = {"type": "HDLFIRCore", "Kind": b["type"], "Port": str(k), "Params": txt, "Owner": n}
        wire(src["data"], nm, 1)
        wire(src["valid"], nm, 2)
        if "reset" in src:
            wire(src["reset"], nm, 3)
    info.setdefault("ports", {})[n] = {k: nm for k, nm in outs if k > 1}


def _neg(x):
    if isinstance(x, str):
        t = x.strip()
        return t[1:] if t.startswith("-") else "-" + t
    return -x


def _fir_lowered(d):
    """{user DSP HDL FIR: (taps, coefficient texts, decimation factor, data source)} for the FIRs fir_lower rewrote
    into engine blocks (<X>f/p0 .. the coefficient Gains; a decimator's phase counter limit <X>f/phase/max = D - 1),
    from the structure only (cached on d)."""
    c = getattr(d, "_lab2_fir_lowered", None)
    if c is not None:
        return c
    c = {}
    for o in d.names:
        x = INTERNAL + o + "/f/"
        if o.startswith(INTERNAL) or (x + "p0") not in d.names:
            continue
        k = 0
        while (x + "p%d" % k) in d.names:
            k += 1
        coefs = [str(d.params[x + "p%d" % j].get("Gain", "")) for j in range(k)]
        mx = d.params.get(INTERNAL + x + "phase/max", {}).get("Value")
        try:
            dec = int(float(mx)) + 1 if mx is not None else 1
        except ValueError:
            continue
        src = (d.src.get(x + "p0") or [None])[0]
        if src is not None:
            c[o] = (k, coefs, dec, src)
    try:
        d._lab2_fir_lowered = c
    except AttributeError:
        pass
    return c


def _fir_bits(d, owner):
    """Block cost of a DSP HDL FIR Filter / FIR Decimator, an estimate from its parameters and structure (the words
    of Simulink's HDL implementation were not recorded, only its values and latency): the input delay line (taps - 1
    words of the input), the coefficients (taps words: the input's word length when inherited, else the given type),
    m multipliers' products (input word + coefficient word each; m = taps, ceil(taps / 2) for literally symmetric or
    antisymmetric coefficients, ceil(taps / NumCycles) partly serial, ceil(taps / (DecimationFactor x NumCycles)) for
    the decimator), m accumulator stages and the output register (each the output's word), and the valid bit. Linear in
    the input and output words: per_signal {data source: taps - 1 + taps (inherited coefficients) + m, the FIR: m + 1}."""
    import json as _json
    import math
    import fxhdlfir
    try:
        p = _json.loads(d.params[owner].get("Params", "{}"))
        c = fxhdlfir.coefficients(p, "design.blocks." + owner)
    except Exception:   # noqa: BLE001 -- the block's own compile reports it
        return None
    src = (d.src.get(owner) or [None])[0]
    if src is None:
        return None
    return _fir_estimate(len(c), c, d.params[owner].get("Kind"), p, src, owner)


def _fir_estimate(n, c, kind, p, src, owner):
    """The words of _fir_bits for n taps of coefficients c (values or texts compared as written)."""
    import math
    import fxhdlfir
    sym = c == c[::-1] or c == [_neg(x) for x in c[::-1]]
    st = p.get("FilterStructure", "Direct form systolic")
    if kind == "HDLFIRDecimator":   # (NumCycles > 1, recordings: the products shared over NumCycles steps too)
        m = math.ceil(n / (int(p.get("DecimationFactor", "2")) * max(int(p.get("NumCycles", "1") or 1), 1)))
    elif st == "Partly serial systolic":
        m = math.ceil(n / max(int(p.get("NumCycles", "2") or 2), 1))
    else:
        m = math.ceil(n / 2) if sym else n
    ct = fxhdlfir._fixdt(p.get("CoefficientsDataTypeStr", ""))
    cw = ct[1] if ct else None
    bits = (n * cw if cw else 0) + (m * cw if cw else 0) + 1
    per = {src: (n - 1) + (0 if cw else n) + m + (0 if cw else m), owner: m + 1}
    return bits, ("DSP HDL FIR (estimate): input delay line %d + coefficients %d + %d products + %d accumulator "
                  "stages + output register + valid, by word" % (n - 1, n, m, m)), per


def _fir_plan(p, ts, where):
    import json
    import fxhdlfir
    try:
        params = json.loads(p.get("Params", "{}"))
    except ValueError:
        _refuse(where, "internal parameters", "INTERNAL")
    return fxhdlfir.Plan(p.get("Kind"), params, ts, where)


def fir_out_type(p, ts, where):
    where = _owner_where(p, where)
    from fxgraph_lib import BOOL
    pl = _fir_plan(p, ts, where)
    return pl.otype if p.get("Port", "1") == "1" else BOOL


def fir_kernel(p, ts, where):
    where = _owner_where(p, where)
    import fxhdlfir
    return fxhdlfir.Kernel(_fir_plan(p, ts, where), int(p.get("Port", "1")) - 1)


# ---------------------------------------------------------------- DSP HDL NCO (engine/fxhdlnco.py)
def _nco(spec):
    """Each HDLNCO becomes one internal NCOCore per real output (sin / cos / valid) on the NCO's inputs; a complex
    exponential output is a Real-Imag to Complex (named as the NCO) of the cos and sin cores. Lines out of output
    port k are re-pointed at the block of port k."""
    import json
    import fxhdlnco
    blocks = dict(spec["blocks"])
    lines = list(spec.get("lines", []))
    remap = {}
    new = []
    for n, b in spec["blocks"].items():
        if not isinstance(b, dict) or b.get("type") != "HDLNCO":
            continue
        where = "design.blocks." + n
        p, _a, _q, _inc = fxhdlnco.params(b, where)
        ins = fxhdlnco.inputs(p)
        feeds = {}
        for ln in lines:
            if ln.get("dst") == n:
                k = int(ln.get("dport", 1))
                if not 1 <= k <= len(ins):
                    _refuse(where, "input port %d does not exist (%s)" % (k, ", ".join(ins)), "BAD_CONNECTION")
                feeds[k] = (ln["src"], int(ln.get("sport", 1)))
        for k in range(1, len(ins) + 1):
            if k not in feeds:
                _refuse(where, "input port %d (%s) is not connected" % (k, ins[k - 1]), "BAD_CONNECTION")
        X = INTERNAL + n + "/"
        txt = json.dumps(p, sort_keys=True)
        del blocks[n]
        names = {}
        for k, sel in enumerate(fxhdlnco.ports(p)):
            cores = ["cos", "sin"] if sel == "cplx" else [sel]
            for c in cores:
                nm = X + c
                if nm in blocks:
                    _refuse(where, "internal name clash %r" % nm, "INTERNAL")
                blocks[nm] = {"type": "NCOCore", "Sel": c, "Params": txt, "Owner": n}
                for j in range(1, len(ins) + 1):
                    s_, sp = feeds[j]
                    new.append(dict({"src": s_, "dst": nm, "dport": j}, **({"sport": sp} if sp != 1 else {})))
            if sel == "cplx":
                blocks[n] = {"type": "RealImagToComplex"}
                new += [{"src": X + "cos", "dst": n, "dport": 1}, {"src": X + "sin", "dst": n, "dport": 2}]
                names[k + 1] = n
            else:
                names[k + 1] = X + sel
        if n not in blocks:   # the NCO's own name stays usable as port 1 of a real output
            names_1 = names[1]
            blocks[n] = blocks.pop(names_1)
            for ln in new:
                if ln["dst"] == names_1:
                    ln["dst"] = n
            names[1] = n
        remap[n] = names
    out = []
    for ln in lines:
        if ln.get("dst") in remap:
            continue
        s_ = ln.get("src")
        if s_ in remap:
            k = int(ln.get("sport", 1))
            if k not in remap[s_]:
                _refuse("design.lines", "output port %d of %s does not exist" % (k, s_), "BAD_CONNECTION")
            ln = {kk: v for kk, v in ln.items() if kk != "sport"}
            ln["src"] = remap[s_][k]
        out.append(ln)
    import parameter_bounds
    return {"blocks": blocks, "lines": out + new,
            parameter_bounds.PORT_KEY: _repointed(spec, [((s_, k), (t_, 1)) for s_, nm in remap.items()
                                                    for k, t_ in nm.items()])}


def _nco_plan(p, where):
    import json
    import fxhdlnco
    try:
        params = json.loads(p.get("Params", "{}"))
    except ValueError:
        _refuse(where, "internal parameters", "INTERNAL")
    return fxhdlnco.params(params, where)


def nco_ninputs(p):
    import fxhdlnco
    return len(fxhdlnco.inputs(_nco_plan(p, "design.blocks")[0]))


def nco_out_type(p):
    from fxgraph_lib import BOOL
    import fxhdlnco
    if p.get("Sel") == "phase":   
        return fxhdlnco.phase_type(_nco_plan(p, "design.blocks")[0])
    return BOOL if p.get("Sel") == "valid" else fxhdlnco.out_type(_nco_plan(p, "design.blocks")[0])


def nco_kernel(p, ts, where):
    where = _owner_where(p, where)
    import fxhdlnco
    pp, a, q, inc = _nco_plan(p, where)
    return fxhdlnco.Kernel(pp, a, q, inc, p.get("Sel"), ts, where)


# ---------------------------------------------------------------- System RAM address width from the address signal
RAM_AUTO = "Inherit: from address"


def _auto_rams(spec):
    return [n for n, b in spec.get("blocks", {}).items() if isinstance(b, dict) and b.get("type") == "HDLRAM"
            and str(b.get("AddressBits", "")).strip() == RAM_AUTO]


def _fir_blocks(spec):
    return [n for n, b in spec.get("blocks", {}).items() if isinstance(b, dict)
            and b.get("type") in ("HDLFIRFilter", "HDLFIRDecimator")]


def ram_auto_provisional(spec):
    """(original spec or None, spec to lower now): RAMs with AddressBits 'Inherit: from address' get a provisional
    width (the ports and names do not depend on it); compile() replaces it by the address type's width. DSP HDL
    FIR blocks are kept as their Python cores until compile() knows their input types (fir_lower)."""
    if not isinstance(spec, dict) or not isinstance(spec.get("blocks"), dict):
        return None, spec
    import block_lowering
    import fxiir
    import fxlattice
    if block_lowering.typed_blocks(spec) or fxiir.inherited(spec) or fxlattice.has(spec):
        # (+ recordings inherited filter types; recordings Lattice MA FIRs)
        orig = spec
        spec = fxlattice.provisional(fxiir.provisional(block_lowering.provisional(spec)))
        if _auto_rams(spec):
            spec = ram_auto_provisional(spec)[1]
        return orig, spec
    if not _auto_rams(spec):
        later = any(isinstance(b, dict) and b.get("type") in ("HDLNCO", "MATLABFunction", "BitConcat", "HDLRAM",
                                                               "EnablePort") + SYNC
                    for b in spec["blocks"].values())
        return (spec if (_fir_blocks(spec) or later) else None), spec
    blocks = dict(spec["blocks"])
    for n in _auto_rams(spec):
        blocks[n] = dict(blocks[n], AddressBits="2")
    return spec, dict(spec, blocks=blocks)


def typed_rewrite(spec, hw, in_types, mlfn=True):
    """The rewrites that need input types: inherited RAM address widths, then the DSP HDL FIR lowering. mlfn=False
    keeps every MATLAB Function on its interpreter cores (Design.compile's fallback, recordings)."""
    import block_lowering
    spec0_ = spec
    spec = ram_auto_resolve(spec, hw, in_types)
    spec = ram_bool_mark(spec, hw, in_types)
    spec = sync_ctrl_mark(spec, hw, in_types)
    import fxiir
    import fxlattice
    # Discrete FIR 'Lattice MA' lowered to Gains / Sums / Unit Delays (engine/fxlattice.py)
    spec = fxlattice.typed(spec, lambda sp, ports: _probe(fxlattice.provisional(fxiir.provisional(
        block_lowering.provisional(sp))), hw, in_types, ports, allow_loop=False), hw)
    spec = fxiir.typed(spec, lambda sp, ports: _probe(fxiir.provisional(block_lowering.provisional(sp)), hw, in_types,
                                                      ports, allow_loop=True), hw)
    spec = block_lowering.typed_rewrite(spec, lambda sp, ports: _probe(fxiir.provisional(block_lowering.provisional(sp)), hw, in_types, ports,
                                                                 allow_loop=True), hw)
    spec = nco_lower(spec, hw, in_types)
    if mlfn:
        spec = mlfn_lower(spec, hw, in_types)
    spec = bitconcat_lower(spec, hw, in_types)
    spec = fir_lower(spec, hw, in_types)
    # what these rewrites added is a recorded lowering's helpers (counted under the user block); the user's
    # types before them
    if isinstance(spec, dict) and isinstance(spec.get("blocks"), dict):
        import parameter_bounds as _lb
        prior = spec0_.get(_lb.HELPERS_KEY) or {} if isinstance(spec0_, dict) else {}
        user = dict(prior.get("user") or {})
        user.update({n_: b_.get("type") for n_, b_ in (spec0_.get("blocks") or {}).items()
                     if isinstance(b_, dict) and INTERNAL not in n_ and n_ not in user})
        new = sorted(set(prior.get("helpers") or ()) | {n_ for n_ in spec["blocks"] if INTERNAL in n_ and
                                                        n_ not in (spec0_.get("blocks") or {})})
        spec = dict(spec, **{_lb.HELPERS_KEY: {"user": user, "helpers": new, "nco_direct": sorted(
            set(prior.get("nco_direct") or ()) | _nco_cplx_direct(spec0_))}})
    return spec


CAT_MAX_W = 120   # Bit Concat words lowered (the regression check: Python, C core and generated C agree)


def bitconcat_lower(spec, hw, in_types):
    """Bit Concat rewritten into verified blocks once its input types are known, so it runs on
    the C core and in fxcodegen as well: input k's bit pattern (Bit Slice of all its bits; a boolean is its 0 / 1)
    converted exactly to ufix(W), shifted left by the widths of the inputs after it, and the fields added -- they do
    not overlap, so the sum is the concatenation. A Bit Concat in a loop, of an unknown input type or of a word
    above the C core's widths keeps its Python kernel (bit_kernel)."""
    import fxgraph_lib as G
    cats = [n for n, b in spec.get("blocks", {}).items() if isinstance(b, dict) and b.get("type") == "BitConcat"]
    if not cats:
        return spec
    blocks = dict(spec["blocks"])
    lines = list(spec.get("lines", []))
    for n in cats:
        b = blocks[n]
        try:
            m = _pint(b.get("numInputs", "2"), "design.blocks.%s.numInputs" % n)
        except ValueError:
            continue   # the block's own checks report it
        if not 2 <= m <= 128 or set(b) - {"type", "numInputs"}:
            continue
        ts = _probe(dict(spec, blocks=blocks, lines=lines), hw, in_types, [(n, k) for k in range(1, m + 1)])
        if not ts or any(t is None for t in ts):
            continue
        _cat_recorded_types(m, ts, "design.blocks." + n)
        ws = [1 if t == G.BOOL else t[1] for t in ts]
        W = sum(ws)
        if W > CAT_MAX_W:
            continue   # wider words: the Python kernel (exact at any width up to MAX_WL)
        src = {int(l.get("dport", 1)): l for l in lines if l["dst"] == n}
        if sorted(src) != list(range(1, m + 1)):
            continue
        lines = [l for l in lines if l["dst"] != n]
        ut = "fixdt(0,%d,0)" % W
        blocks[n] = {"type": "Sum", "Inputs": "+" * m, "OutDataTypeStr": ut, "AccumDataTypeStr": ut,
                     "RndMeth": "Floor", "SaturateOnIntegerOverflow": "off"}
        off = W
        for k in range(1, m + 1):
            off -= ws[k - 1]
            ln = src[k]
            tap = (ln["src"], int(ln.get("sport", 1)))
            pre = INTERNAL + n + "/cat%d" % k
            if ts[k - 1] != G.BOOL:
                blocks[pre + "_bits"] = {"type": "BitSlice", "lidx": str(ws[k - 1] - 1), "ridx": "0"}
                lines.append(dict({"src": tap[0], "dst": pre + "_bits", "dport": 1},
                                  **({"sport": tap[1]} if tap[1] != 1 else {})))
                tap = (pre + "_bits", 1)
            blocks[pre + "_u"] = {"type": "DataTypeConversion", "OutDataTypeStr": ut, "RndMeth": "Floor",
                                  "SaturateOnIntegerOverflow": "off"}
            lines.append(dict({"src": tap[0], "dst": pre + "_u", "dport": 1},
                              **({"sport": tap[1]} if tap[1] != 1 else {})))
            tap = (pre + "_u", 1)
            if off:
                blocks[pre + "_sh"] = {"type": "BitShift", "mode": "Shift Left Logical", "N": str(off)}
                lines.append({"src": tap[0], "dst": pre + "_sh", "dport": 1})
                tap = (pre + "_sh", 1)
            lines.append({"src": tap[0], "dst": n, "dport": k})
    return dict(spec, blocks=blocks, lines=lines)


def _ml_persistent(script):
    """Whether a MATLAB Function script declares persistent state: a 'persistent' statement anywhere in the parse
    tree (`b = a; persistent c` on one line was missed by the line-start match); when the text does not
    parse, a 'persistent' word at a statement start (line start or after; /,) outside comments."""
    try:
        import fxmlfn
        ast = fxmlfn.parse(str(script or ""))
        return any(st.get("op") == "persistent" for f in ast["functions"] for st in fxmlfn._all_stmts(f["body"]))
    except Exception:   # noqa: BLE001
        pass
    for ln in str(script or "").splitlines():
        if re.search(r"(^|[;,])\s*persistent\b", ln.split("%", 1)[0]):
            return True
    return False


def _ml_isempty_else(script):
    """Whether the main function initialises a persistent in `if isempty(p) ... else ... end` (an else / elseif arm)."""
    try:
        import fxmlfn
        ast = fxmlfn.parse(str(script or ""))
    except Exception:   # noqa: BLE001
        return False
    main = next(f for f in ast["functions"] if f["name"] == ast["main"])
    for st in fxmlfn._all_stmts(main["body"]):
        if st.get("op") == "if" and st.get("arms"):
            c = st["arms"][0][0]
            if c.get("op") == "call" and c.get("name") == "isempty" and (st.get("else") or len(st["arms"]) > 1):
                return True
    return False


def _ml_write_before_read(script):
    """a persistent of the main function written (anywhere but the `then` branch of its
    `if isempty` initialisation) before an output assignment that reads it, directly or through local variables --
    Simulink rejects that inside a Synchronous subsystem (Stateflow:cdr:WriteToPersistentVariableInOutputsFunction;
    recordings: the one-line `if isempty(c), c = ...; end`, a write inside another if or in the isempty's
    else, `t = c; y = t` -- 7 / 7 rejected); the output-before-update form is fine. Read from the parse tree (the
    line scan before it lost track of one-line ifs); the line scan stays as a fallback when the text does not parse."""
    try:
        import fxmlfn
        ast = fxmlfn.parse(str(script or ""))
    except Exception:   # noqa: BLE001 -- the MATLAB Function's own refusal comes later; fall back to the line scan
        return _ml_write_before_read_lines(script)
    main = next(f for f in ast["functions"] if f["name"] == ast["main"])
    outs = set(main["outputs"])
    pers = set()
    for st in fxmlfn._all_stmts(main["body"]):
        if st.get("op") == "persistent":
            pers |= set(st.get("names", []))
    if not pers or not outs:
        return False

    def names(e):
        return {x["name"] for x in fxmlfn._nodes(e) if isinstance(x, dict) and x.get("op") == "var"}

    def walk(stmts, written, taint):
        for st in stmts:
            op = st.get("op")
            if op == "assign":
                rv = names(st.get("expr")) | names(st.get("index"))
                dirty = bool(rv & written) or any(taint.get(v) for v in rv)
                tg = st.get("target")
                if tg in outs and dirty:
                    return True
                if tg in pers:
                    written.add(tg)
                else:
                    taint[tg] = dirty or (taint.get(tg, False) and st.get("index") is not None)
                continue
            if op == "if":
                arms = st.get("arms", [])
                for k, (cond, body) in enumerate(arms):
                    init = k == 0 and cond.get("op") == "call" and cond.get("name") == "isempty" and \
                        any(a.get("op") == "var" and a.get("name") in pers for a in cond.get("args", []))
                    if init:
                        continue   # the initialisation itself
                    if walk(body, written, taint):
                        return True
                if walk(st.get("else") or [], written, taint):
                    return True
                continue
            for key in ("body", "otherwise"):
                if isinstance(st.get(key), list) and walk(st[key], written, taint):
                    return True
            for case in st.get("cases", []) or []:
                body = case[1] if isinstance(case, (list, tuple)) and len(case) > 1 else None
                if isinstance(body, list) and walk(body, written, taint):
                    return True
        return False
    if walk(main["body"], set(), {}):
        return True
    return _ml_write_before_read_lines(script)


def _ml_write_before_read_lines(script):
    """The line scan of recordings (kept as a fallback; the parse-tree reading above is the main one)."""
    lines = [ln.split("%", 1)[0].rstrip() for ln in str(script or "").splitlines()]
    hdr = next((ln for ln in lines if ln.strip().startswith("function")), "")
    m = re.match(r"\s*function\s+(?:\[([^\]]*)\]|(\w+))\s*=", hdr)
    outs = set(re.findall(r"\w+", (m.group(1) or m.group(2) or "") if m else ""))
    pers = set()
    for ln in lines:
        if re.match(r"\s*persistent\b", ln):
            pers |= set(re.findall(r"\w+", ln.split("persistent", 1)[1]))
    if not pers or not outs:
        return False
    written, depth_init, after_header = set(), 0, False
    for ln in lines:
        t = ln.strip()
        if t.startswith("function"):
            if after_header:
                break   # a local function: only the main function is looked at
            after_header = True
            continue
        if re.match(r"if\s+isempty\s*\(", t):
            depth_init += 1
            continue
        if depth_init and re.match(r"(if|for|while|switch)\b", t):
            depth_init += 1
            continue
        if depth_init and t == "end":
            depth_init -= 1
            continue
        if depth_init:
            continue
        mm = re.match(r"(\w+)\s*(?:\(:\))?\s*=(?!=)\s*(.*)$", t)
        if not mm:
            continue
        lhs, rhs = mm.group(1), mm.group(2)
        if lhs in outs and written & set(re.findall(r"\w+", rhs)):
            return True
        if lhs in pers:
            written.add(lhs)
    return False


def _in_sync_es(blocks, n):
    """Whether block n lies inside an enabled subsystem whose State Control is Synchronous."""
    parts = n.lstrip(INTERNAL).split("/")[:-1]
    for k in range(1, len(parts) + 1):
        pre = "/".join(parts[:k]) + "/"
        en = any(isinstance(b, dict) and b.get("type") == "EnablePort" and m.startswith(pre) and "/" not in m[len(pre):]
                 for m, b in blocks.items())
        sc = any(isinstance(b, dict) and b.get("type") == "StateControl" and m.startswith(pre) and
                 "/" not in m[len(pre):] and str(b.get("StateControl", "Classic")).strip() == "Synchronous"
                 for m, b in blocks.items())
        if en and sc:
            return True
    return False


def _in_reset_es(blocks, n):
    """Whether block n lies inside an enabled subsystem whose EnablePort resets the states on enabling."""
    parts = n.lstrip(INTERNAL).split("/")[:-1]
    for k in range(1, len(parts) + 1):
        pre = "/".join(parts[:k]) + "/"
        for m, b in blocks.items():
            if isinstance(b, dict) and b.get("type") == "EnablePort" and m.startswith(pre) and \
                    "/" not in m[len(pre):] and str(b.get("StatesWhenEnabling", "held")).strip() == "reset":
                return True
    return False


def mlfn_lower(spec, hw, in_types):
    """MATLAB Function blocks (scalar, or with vector arguments as the vector pass flattened them) compiled into engine blocks (engine/fxmlgen.py) once their input types are
    known; a construct the compiler does not express (NotLowerable) keeps the interpreter cores (Python path)."""
    import fxmlgen
    fns = [n for n, b in spec.get("blocks", {}).items() if isinstance(b, dict) and b.get("type") == "MATLABFunction"]
    if not fns:
        return spec
    blocks = dict(spec["blocks"])
    lines = list(spec.get("lines", []))
    for n in fns:
        b = blocks[n]
        where = "design.blocks." + n
        if _ml_persistent(b.get("Script")) and (_in_reset_es(blocks, n) or (
                _in_sync_es(blocks, n) and _ml_write_before_read(b.get("Script")))):
            # persistent state in an enabled subsystem that resets its states is not reset by the
            # compiled form (5 / 5 packages FAIL); it stays on the interpreter cores, refused at run time
            # (es_kernel_check)
            continue
        try:
            main = _ml_main(b.get("Script"), where)
        except Exception:   # noqa: BLE001 -- the cores report it
            continue
        nin, nout = len(main["inputs"]), len(main["outputs"])
        # a vector argument comes element by element on consecutive ports (engine/fxvector.py: _InWidths)
        iw = _ml_widths(b, "_InWidths") or [1] * nin
        if len(iw) != nin:
            continue
        nports = sum(iw)
        ts = _probe(dict(spec, blocks=blocks, lines=lines), hw, in_types, [(n, k) for k in range(1, nports + 1)])
        if not ts or any(t is None for t in ts):
            continue
        p = {k: str(v) for k, v in b.items() if k != "type" and not k.startswith("OutDataTypeStr")
             and k not in ("_InWidths", "_OutWidths")}
        if "_InWidths" in b:
            p["InWidths"] = str(b["_InWidths"])
        p.update(Ports=str(nout), Port="1")
        for j in range(1, nout + 1):
            txt = b.get(_ml_out_key(j))
            if txt:
                p["OutDataTypeStr" if j == 1 else "_Out%d" % j] = txt
        try:
            prog = _ml_program(p, ts, where)
            src = {int(l.get("dport", 1)): (l["src"], int(l.get("sport", 1))) for l in lines if l["dst"] == n}
            groups, k0 = [], 1
            for w in iw:
                groups.append([src[k] for k in range(k0, k0 + w)] if w > 1 else src[k0])
                k0 += w
            g, outs = fxmlgen.lower(prog, n, groups, INTERNAL)
            ow = _ml_widths(b, "_OutWidths") or [1] * nout
            if len(outs) != sum(ow):
                continue
        except fxmlgen.NotLowerable:
            continue
        except Exception:   # noqa: BLE001 -- the cores report it when the design compiles
            if os.environ.get("FX_NO_MLFN_FALLBACK"):
                raise   # the compiler's tests: a crash in the compiler must show, not fall back (mutation testing)
            continue
        new, nl = dict(g.blocks), list(g.lines)
        names = {}
        for k, (kind, s_, et) in outs.items():
            nm = n if k == 1 else INTERNAL + n + "/m/y%d" % k
            if et == "bool":   # identity: NOT(NOT x)
                inv = INTERNAL + n + "/m/inv%d" % k
                new[inv] = {"type": "LogicalOperator", "Operator": "NOT", "Inputs": "1"}
                new[nm] = {"type": "LogicalOperator", "Operator": "NOT", "Inputs": "1"}
                nl.append(dict({"src": s_[0], "dst": inv, "dport": 1}, **({"sport": s_[1]} if s_[1] != 1 else {})))
                nl.append({"src": inv, "dst": nm, "dport": 1})
            else:
                new[nm] = {"type": "DataTypeConversion", "OutDataTypeStr": "fixdt(%d,%d,%d)" % et, "RndMeth": "Floor",
                           "SaturateOnIntegerOverflow": "off"}   # identity
                nl.append(dict({"src": s_[0], "dst": nm, "dport": 1}, **({"sport": s_[1]} if s_[1] != 1 else {})))
            names[k] = nm
        out = []
        ok_ = True
        for l in lines:
            if l["dst"] == n:
                continue
            if l["src"] == n:
                k = int(l.get("sport", 1))
                if k not in names:
                    ok_ = False
                    break
                l = {kk: v for kk, v in l.items() if kk != "sport"}
                l["src"] = names[k]
            out.append(l)
        if not ok_:
            continue
        lines = out + nl
        del blocks[n]
        blocks.update(new)
        spec = dict(spec, **{__import__("parameter_bounds").PORT_KEY: _repointed(spec, [((n, k), (t_, 1))
                                                                                for k, t_ in names.items()])})
    return dict(spec, blocks=blocks, lines=lines)


def nco_lower(spec, hw, in_types):
    """DSP HDL NCOs with the increment from the dialog and a quantiser of at most 14 bits are rewritten into engine
    blocks every path supports (recordings-verified rules of engine/fxhdlnco.py):
      acc     a signed AccumulatorWL-bit register advancing by the increment on valid (Sum + an enabled delay, wrap)
      phase   acc (+ PhaseOffset, wrap), its bit pattern read as ufix<A> with A - Q fraction bits, floored to ufix<Q>
      value   a 1-D lookup table, flat, evenly spaced 0 .. 2^Q - 1: round-half-up(sin / cos(2 pi q / 2^Q) * 2^FL)
      out     valid ? value: 0, and valid, both delayed 6 steps
    The increment port (its valid must be always high: checked at run time) and larger quantisers keep the Python
    cores."""
    import json
    import math
    import fxhdlnco
    ncos = [n for n, b in spec.get("blocks", {}).items() if isinstance(b, dict) and b.get("type") == "HDLNCO"]
    if not ncos:
        return spec
    blocks = dict(spec["blocks"])
    lines = list(spec.get("lines", []))
    for n in ncos:
        b = blocks[n]
        where = "design.blocks." + n
        try:
            p, A, Q, inc = fxhdlnco.params(b, where)
        except Exception:   # noqa: BLE001 -- the cores report it when the design compiles
            continue
        if inc is None or Q > 14 or p.get("PhasePort") == "on" or p.get("PhaseOffsetSource") == "Input port" or \
                p.get("ResetAction") == "on" or p.get("DitherSource") == "Property":
            continue   # the phase output and the phase offset port stay on the exact kernel
        ts = _probe(dict(spec, blocks=blocks, lines=lines), hw, in_types, [(n, 1)])
        import fxgraph_lib as G
        if not ts or ts[0] != G.BOOL:
            continue   # the cores refuse a non-boolean valid
        WL, FL = int(p["OutputWL"]), int(p["OutputFL"])
        off = int(p["PhaseOffset"])
        x = INTERNAL + n + "/l/"
        src = {int(l.get("dport", 1)): (l["src"], int(l.get("sport", 1))) for l in lines if l["dst"] == n}
        vin = src[1]
        new, nl = {}, []

        def w(s_, d_, port=1):
            if isinstance(s_, tuple):
                nl.append(dict({"src": s_[0], "dst": d_, "dport": port}, **({"sport": s_[1]} if s_[1] != 1 else {})))
            else:
                nl.append({"src": s_, "dst": d_, "dport": port})

        def wrap(v):
            v %= 1 << A
            return v - (1 << A) if v >> (A - 1) else v
        acc_t = "fixdt(1,%d,0)" % A
        out_t = "fixdt(1,%d,%d)" % (WL, FL)
        new[x + "inc"] = {"type": "Constant", "Value": str(wrap(inc)), "OutDataTypeStr": acc_t}
        new[x + "sum"] = {"type": "Sum", "Inputs": "++", "OutDataTypeStr": acc_t, "AccumDataTypeStr": acc_t,
                          "RndMeth": "Floor", "SaturateOnIntegerOverflow": "off"}
        new[x + "acc"] = {"type": "UnitDelayEnabledSynchronous", "InitialCondition": "0"}
        w(x + "acc", x + "sum", 1)
        w(x + "inc", x + "sum", 2)
        w(x + "sum", x + "acc", 1)
        w(vin, x + "acc", 2)
        ph = x + "acc"
        if off:
            new[x + "off"] = {"type": "Constant", "Value": str(wrap(off)), "OutDataTypeStr": acc_t}
            new[x + "ph"] = {"type": "Sum", "Inputs": "++", "OutDataTypeStr": acc_t, "AccumDataTypeStr": acc_t,
                             "RndMeth": "Floor", "SaturateOnIntegerOverflow": "off"}
            w(x + "acc", x + "ph", 1)
            w(x + "off", x + "ph", 2)
            ph = x + "ph"
        new[x + "pat"] = {"type": "DataTypeConversion", "OutDataTypeStr": "fixdt(0,%d,%d)" % (A, A - Q),
                          "ConvertRealWorld": "Stored Integer (SI)", "RndMeth": "Floor",
                          "SaturateOnIntegerOverflow": "off"}
        w(ph, x + "pat")
        new[x + "q"] = {"type": "DataTypeConversion", "OutDataTypeStr": "fixdt(0,%d,0)" % Q, "RndMeth": "Floor",
                        "SaturateOnIntegerOverflow": "off"}
        w(x + "pat", x + "q")
        new[x + "zero"] = {"type": "Constant", "Value": "0", "OutDataTypeStr": out_t}
        hi = (1 << (WL - 1)) - 1
        lo = -hi   # saturated symmetrically (FL = WL - 1: sin -1 gives -(2^(WL-1) - 1)), as the core

        def table(fn):
            # as the core (engine/fxhdlnco.lut_value; the compressed table with LUTCompress on)
            sel_ = "sin" if fn is math.sin else "cos"
            vals = [fxhdlnco.lut_value(k, Q, sel_, WL, FL, p.get("LUTCompress") == "on") / float(1 << FL)
                    for k in range(1 << Q)]
            return "[" + " ".join(repr(v) for v in vals) + "]"
        outs = {}
        for sel in [c for c in fxhdlnco.ports(p) if c != "valid"]:
            parts = ["cos", "sin"] if sel == "cplx" else [sel]
            for c in parts:
                lut, sw = x + "lut_" + c, x + "sel_" + c
                new[lut] = {"type": "Lookup1D", "Table": table(math.sin if c == "sin" else math.cos),
                            "BreakpointsSpecification": "Even spacing", "BreakpointsForDimension1FirstPoint": "0",
                            "BreakpointsForDimension1Spacing": "1", "InterpMethod": "Flat", "ExtrapMethod": "Clip",
                            "OutDataTypeStr": out_t, "TableDataTypeStr": "Inherit: Same as output",
                            "RndMeth": "Floor", "SaturateOnIntegerOverflow": "off"}
                w(x + "q", lut)
                new[sw] = {"type": "Switch", "Criteria": "u2 ~= 0", "OutDataTypeStr": "Inherit: Inherit via internal rule"}
                w(lut, sw, 1)
                w(vin, sw, 2)
                w(x + "zero", sw, 3)
                o = x + "out_" + c
                new[o] = {"type": "Delay", "DelayLength": str(fxhdlnco.LATENCY), "InitialCondition": "0"}
                w(sw, o)
                outs[c] = o
        new[x + "valid"] = {"type": "Delay", "DelayLength": str(fxhdlnco.LATENCY), "InitialCondition": "0"}
        w(vin, x + "valid")
        # output ports as the cores number them: sin / cos / cplx, then valid; port 1 keeps the NCO's name
        names = {}
        for k, sel in enumerate(fxhdlnco.ports(p)):
            if sel == "cplx":
                new[x + "c"] = {"type": "RealImagToComplex"}
                w(outs["cos"], x + "c", 1)
                w(outs["sin"], x + "c", 2)
                names[k + 1] = x + "c"
            else:
                names[k + 1] = outs[sel] if sel != "valid" else x + "valid"
        first = names[1]
        new[n] = new.pop(first)
        nl = [dict(l, src=n) if l["src"] == first else (dict(l, dst=n) if l["dst"] == first else l) for l in nl]
        names[1] = n
        spec = dict(spec, **{__import__("parameter_bounds").PORT_KEY: _repointed(spec, [((n, k), (t_, 1))
                                                                                for k, t_ in names.items()])})
        out = []
        for l in lines:
            if l["dst"] == n:
                continue
            if l["src"] == n:
                k = int(l.get("sport", 1))
                if k not in names:
                    return spec   # a port the cores would refuse: keep them
                l = {kk: v for kk, v in l.items() if kk != "sport"}
                l["src"] = names[k]
            out.append(l)
        lines = out + nl
        del blocks[n]
        blocks.update(new)
        _ = json
    return dict(spec, blocks=blocks, lines=lines)


def _probe(spec, hw, in_types, ports, allow_loop=False):
    """Compiled types of the signals feeding (block, dport) for each of ports, from the part of the design that
    drives them (None for an input that is not connected)."""
    import fxgraph_lib as G
    lines = spec.get("lines", [])
    feeds = {}
    for ln in lines:
        feeds.setdefault(ln.get("dst"), []).append(ln)
    srcs = []
    for blk, port in ports:
        ln = next((l for l in feeds.get(blk, []) if int(l.get("dport", 1)) == port), None)
        srcs.append(ln)
    seen, stack = set(), [l["src"] for l in srcs if l is not None]
    while stack:
        x = stack.pop()
        if x in seen:
            continue
        seen.add(x)
        stack += [l["src"] for l in feeds.get(x, [])]
    if not allow_loop and any(blk in seen for blk, _ in ports):
        return None   # a block feeding itself: no probe (its cores stay as they are)
    # allow_loop (block_lowering.typed_rewrite): the spec holds type-faithful placeholders for the blocks still to be
    # rewritten (block_lowering.provisional), so the loop through the block is typed with its placeholder in place
    pb = {k: v for k, v in spec["blocks"].items() if k in seen}
    pl = [l for l in lines if l.get("dst") in seen and l.get("src") in seen]
    names = []
    for k, ln in enumerate(srcs):
        if ln is None:
            names.append(None)
            continue
        nm = INTERNAL + "probe%d" % k
        pb[nm] = {"type": "Outport"}
        pl.append(dict({"src": ln["src"], "dst": nm}, **({"sport": ln["sport"]} if "sport" in ln else {})))
        names.append(nm)
    try:
        d = G.Design({"blocks": pb, "lines": pl}, hw)
        d.compile({k: v for k, v in in_types.items() if k in d.inports})
    except G.GraphError:
        return None   # the full compile reports it
    return [d.otype.get(nm) if nm else None for nm in names]   # a complex source has no probe type


def fir_lower(spec, hw, in_types):
    """DSP HDL FIR Filter / FIR Decimator blocks whose every valid pattern has a recorded timing rule (a gap rule:
    transposed, systolic asymmetric or symmetric with an even tap count, decimators with a recorded (a, b)) are
    rewritten into engine blocks every path supports, so the C core, fxcodegen and fxverilog run them; the others
    (partly serial, symmetric odd tap counts, D3: their valid pattern is checked at run time) keep the Python core.
    The rewrite, for the plan of engine/fxhdlfir.py (coefficients c_k, output type, gap (a, b), factor D):
      taps     t_0 = x, t_k = Unit Delay Enabled Synchronous(t_{k-1}, valid)        (the k-th previous valid sample)
      sum      s = Sum_k Gain(t_k, c_k)  -- products and sum exact (types from the plan)
      a later  q = a enabled delays of s; e = a enabled delays of true (e: at least a valid samples before)
      phase    decimator: an HDL Counter of the valid samples modulo D, kept when it equals (a + D - 1) mod D
      out      ok = valid AND e AND phase; data = Delay_b(ok ? q : 0); valid = Delay_b(ok)
    which is the kernel's rule: the kept sample with valid index i comes out b steps after the valid input i + a."""
    import fxhdlfir
    firs = _fir_blocks(spec)
    if not firs:
        return spec
    blocks = dict(spec["blocks"])
    lines = list(spec.get("lines", []))
    for n in firs:
        b = blocks[n]
        where = "design.blocks." + n
        ts = _probe(dict(spec, blocks=blocks, lines=lines), hw, in_types, [(n, 1), (n, 2)])
        if ts is None or any(t is None for t in ts):
            continue
        try:
            p = fxhdlfir.params(b["type"], b, where)
            pl = fxhdlfir.Plan(b["type"], p, ts, where)
        except Exception:   # noqa: BLE001 -- the cores report it when the design compiles
            continue
        if pl.gap is None or pl.serial or pl.cast is not None or pl.reset or pl.exact:
            # an explicit output type stays on the exact kernel, which rounds the full sum; so does a
            # decimator with NumCycles > 1 (recorded with valid exactly every NumCycles steps, which the
            # kernel checks at run time)
            continue
        a, bb = pl.gap
        x = INTERNAL + n + "/f/"
        src = {int(l.get("dport", 1)): (l["src"], int(l.get("sport", 1))) for l in lines if l["dst"] == n}
        new, nl = {}, []

        def w(s_, d_, port=1):
            if isinstance(s_, tuple):
                nl.append(dict({"src": s_[0], "dst": d_, "dport": port}, **({"sport": s_[1]} if s_[1] != 1 else {})))
            else:
                nl.append({"src": s_, "dst": d_, "dport": port})
        xin, vin = src[1], src[2]
        _, wl, xfl = ts[0]
        ot = pl.otype

        def fx(t):
            return "fixdt(%d,%d,%d)" % t
        taps = [xin]
        for k in range(1, len(pl.c)):
            nm = x + "t%d" % k
            new[nm] = {"type": "UnitDelayEnabledSynchronous", "InitialCondition": "0"}
            w(taps[-1], nm, 1)
            w(vin, nm, 2)
            taps.append(nm)
        prods = []
        for k, ck in enumerate(pl.c):
            if ck == 0:
                continue
            nm = x + "p%d" % k
            new[nm] = {"type": "Gain", "Gain": repr(ck * 2.0 ** -(pl.otype[2] - xfl)),   # exact: |ck| < 2^24
                       "ParamDataTypeStr": "fixdt(1,%d,%d)" % (wl, pl.otype[2] - xfl),
                       "OutDataTypeStr": fx((1, 2 * wl, pl.otype[2])), "RndMeth": "Floor",
                       "SaturateOnIntegerOverflow": "off"}
            w(taps[k], nm)
            prods.append(nm)
        sm = x + "sum"
        new[sm] = {"type": "Sum", "Inputs": "+" * len(prods), "OutDataTypeStr": fx(ot), "AccumDataTypeStr": fx(ot),
                   "RndMeth": "Floor", "SaturateOnIntegerOverflow": "off"}
        for k, pn in enumerate(prods):
            w(pn, sm, k + 1)
        cand = sm
        conds = [vin]
        if a > 0:
            one = x + "true"
            new[one] = {"type": "RelationalOperator", "Operator": "=="}      # x == x: always true, boolean
            w(xin, one, 1)
            w(xin, one, 2)
            e = one
            for k in range(1, a + 1):
                q = x + "q%d" % k
                new[q] = {"type": "UnitDelayEnabledSynchronous", "InitialCondition": "0"}
                w(cand, q, 1)
                w(vin, q, 2)
                cand = q
                en = x + "e%d" % k
                new[en] = {"type": "UnitDelayEnabledSynchronous", "InitialCondition": "0"}
                w(e, en, 1)
                w(vin, en, 2)
                e = en
            conds.append(e)
        if pl.dec > 1:
            D = pl.dec
            cw = max(1, (D - 1).bit_length())
            cn = x + "phase"
            new[cn] = {"type": "HDLCounter", "CountType": "Count limited", "CountInit": "0", "CountStep": "1",
                       "CountMax": str(D - 1), "CountEnbPort": "on", "CountDataType": "Unsigned",
                       "CountWordLen": str(cw), "CountFracLen": "0"}
            w(vin, cn, 1)
            ph = x + "kept"
            new[ph] = {"type": "CompareToConstant", "relop": "==", "const": str((a + D - 1) % D),
                       "OutDataTypeStr": "boolean"}
            w(cn, ph)
            conds.append(ph)
        if len(conds) > 1:
            ok = x + "ok"
            new[ok] = {"type": "LogicalOperator", "Operator": "AND", "Inputs": str(len(conds))}
            for k, c_ in enumerate(conds):
                w(c_, ok, k + 1)
        else:
            ok = conds[0]
        z = x + "zero"
        new[z] = {"type": "Constant", "Value": "0", "OutDataTypeStr": fx(ot)}
        sel = x + "sel"
        new[sel] = {"type": "Switch", "Criteria": "u2 ~= 0", "OutDataTypeStr": "Inherit: Inherit via internal rule"}
        w(cand, sel, 1)
        w(ok, sel, 2)
        w(z, sel, 3)
        vn = x + "valid"
        new[n] = {"type": "Delay", "DelayLength": str(bb), "InitialCondition": "0"}
        w(sel, n)
        new[vn] = {"type": "Delay", "DelayLength": str(bb), "InitialCondition": "0"}
        w(ok, vn)
        lines = [l for l in lines if l["dst"] != n]
        out = []
        for l in lines:
            if l["src"] == n and int(l.get("sport", 1)) == 2:
                l = {k: v for k, v in l.items() if k != "sport"}
                l["src"] = vn
            elif l["src"] == n and int(l.get("sport", 1)) != 1:
                return spec   # a port the lowering has no block for: keep the cores
            out.append(l)
        lines = out + nl
        del blocks[n]
        blocks.update(new)
        # the valid output (port 2) moved to its own Delay
        spec = dict(spec, **{__import__("parameter_bounds").PORT_KEY: _repointed(spec, [((n, 2), (vn, 1))])})
    return dict(spec, blocks=blocks, lines=lines)


# recordings (recorded: Simulink:DataType:InvalidEnableInputDataType / InvalidResetInputDataType on ufix2, ufix4,
# ufix12, ufix33, sfix2, sfix8_En4, ufix8_En3): the control types Simulink takes
SYNC_CTRL_TYPES = {(0, 1, 0)} | {(s_, w_, 0) for s_ in (0, 1) for w_ in (8, 16, 32)}
# the same set for an enabled subsystem's enable; recordings: ufix3 and
# sfix4 enables too (true when > 0; the signed one with negative values, held and reset)
ES_ENABLE_TYPES = tuple(SYNC_CTRL_TYPES) + ((0, 3, 0), (1, 4, 0))


def sync_ctrl_mark(spec, hw, in_types):
    """The synchronous delays' enable / reset inputs once their types are known: a type Simulink refuses
    is PORT_TYPE_INVALID; a signed enable is marked so the lowering enables on E > 0."""
    import fxgraph_lib as G
    dl = [n for n, b in spec.get("blocks", {}).items() if isinstance(b, dict) and b.get("type") in SYNC
          and not _marked(b, "_EnSigned") and not _marked(b, "_BoolData") and INTERNAL not in n]
    blocks = dict(spec["blocks"])
    # an enabled subsystem's enable (recordings boolean / uint8, recordings int8 / 16 / 32, uint16 / 32,
    # ufix1): true when > 0 -- a signed one is marked so its lowering compares with 0; other types are refused
    for n, b in spec.get("blocks", {}).items():
        if isinstance(b, dict) and b.get("type") == "EnablePort" and INTERNAL not in n and not _marked(b, "_EnSigned"):
            try:
                ts = _probe(spec, hw, in_types, [(n, 1)])
            except G.GraphError:
                continue
            if not ts or ts[0] is None or ts[0] == G.BOOL:
                continue
            if tuple(ts[0]) not in ES_ENABLE_TYPES:
                raise G.GraphError("UNSUPPORTED", "design.blocks." + n, "an enabled subsystem's enable of type %s: "
                                   'recorded boolean, ufix1 and 8 / 16 / 32-bit integers only'
                                   '' % G._tstr(ts[0]))
            if ts[0][0] == 1:
                blocks[n] = dict(blocks[n], _EnSigned=MARK)
    if not dl:
        return dict(spec, blocks=blocks)
    for n in dl:
        names = n_inputs(blocks[n], "design.blocks." + n)
        ports = [(n, k + 1) for k, nm in enumerate(names) if nm in ("E", "R", "u")]
        # each port on its own -- a probe returns None when the signals it needs pass through the
        # block itself, so a delay on a data loop lost its enable's mark too (post_compile_checks refuses what
        # stays unmarked: a control computed from the delay's own output)
        if "R" in names:
            # recordings (recorded): a reset computed from the delay's own output is an
            # algebraic loop for Simulink (Simulink:Engine:UnableToSolveAlgLoop); the engine had answered
            rsrc = [ln.get("src") for ln in spec.get("lines", []) if ln.get("dst") == n
                    and int(ln.get("dport", 1)) == names.index("R") + 1]
            seen, stack = set(), list(rsrc)
            feeds = {}
            for ln in spec.get("lines", []):
                feeds.setdefault(ln.get("dst"), []).append(ln.get("src"))
            while stack:
                x = stack.pop()
                if x in seen:
                    continue
                seen.add(x)
                stack += feeds.get(x, [])
            if n in seen:
                # recordings (reset = own output < 2 through a Compare To Constant, enable
                # from an input) runs in Simulink, so "Simulink cannot solve it" was untrue in general: what decides
                # between the two records is not known -- refused as not computed
                _refuse("design.blocks." + n, "the reset of %s is computed from its own output: Simulink stops on "
                        'some such loops (Simulink:Engine:UnableToSolveAlgLoop) and runs others'
                        '; which, is not recorded -- not computed. Compute the reset from other signals'
                        % b_type_of(blocks[n]))
        ts = []
        for pt in ports:
            try:
                r_ = _probe(spec, hw, in_types, [pt])
            except G.GraphError:
                r_ = None
            ts.append(r_[0] if r_ else None)
        for (nn, k), ty in zip(ports, ts):
            nm = names[k - 1]
            if nm == "u":
                if ty == G.BOOL:
                    # boolean data, initial value 0 / 1 -- the
                    # lowering builds the reset from logic gates (a Switch of booleans is not recorded)
                    blocks[n] = dict(blocks[n], _BoolData=MARK)
                continue
            if ty is None or ty == G.BOOL:
                continue
            if tuple(ty) not in SYNC_CTRL_TYPES:
                raise G.GraphError("PORT_TYPE_INVALID", "design.blocks." + n, "%s input of type %s (Simulink: "
                                   "DataType:Invalid%sInputDataType -- boolean, ufix1 and 8 / 16 / 32-bit integers "
                                   "only)" % ("enable" if nm == "E" else "reset", G._tstr(ty),
                                                          "Enable" if nm == "E" else "Reset"))
            if nm == "E" and ty[0] == 1:
                blocks[n] = dict(blocks[n], _EnSigned=MARK)
    return dict(spec, blocks=blocks)


def ram_bool_mark(spec, hw, in_types):
    """recordings (12 RAMs of boolean data, every step as for fixed-point data): a
    RAM whose data input is boolean is marked so its lowering stores the data as uint8 (a Data Type Conversion of
    one boolean input) and gives each output back as boolean (Compare To Constant ~= 0) -- the engine's Multiport
    Switch takes no boolean data (a previous version: not recorded), the uint8 one is recorded."""
    import fxgraph_lib as G
    rams = [n for n, b in spec.get("blocks", {}).items() if isinstance(b, dict) and b.get("type") == "HDLRAM"
            and "_BoolData" not in b]
    if not rams:
        return spec
    blocks = dict(spec["blocks"])
    for n in rams:
        try:
            ts = _probe(spec, hw, in_types, [(n, 1)])
        except G.GraphError:
            continue   # the RAM's own compile reports it
        if ts and ts[0] == G.BOOL:
            blocks[n] = dict(blocks[n], _BoolData="on")
    return dict(spec, blocks=blocks)


def ram_auto_resolve(spec, hw, in_types):
    """The spec with each 'Inherit: from address' RAM's AddressBits set to the word length of its write address
    (an unsigned integer type), found by compiling the part of the design that drives the address."""
    import fxgraph_lib as G
    lines = spec.get("lines", [])
    feeds = {}
    for ln in lines:
        feeds.setdefault(ln.get("dst"), []).append(ln)
    auto = _auto_rams(spec)
    blocks = dict(spec["blocks"])
    for n in auto:
        where = "design.blocks." + n
        port = 2   # wr_addr (dual-port RAMs) / addr (single port)
        ln = next((l for l in feeds.get(n, []) if int(l.get("dport", 1)) == port), None)
        if ln is None:
            _refuse(where, "the address input is not connected", "BAD_CONNECTION")
        seen, stack = set(), [ln["src"]]
        while stack:
            x = stack.pop()
            if x in seen:
                continue
            if x in auto:
                _refuse(where, "the address depends on the output of a RAM whose address width is inherited; give "
                        "AddressBits")
            seen.add(x)
            stack += [l["src"] for l in feeds.get(x, [])]
        probe = INTERNAL + "probe"
        pb = {k: v for k, v in spec["blocks"].items() if k in seen}
        pb[probe] = {"type": "Outport"}
        pl = [l for l in lines if l.get("dst") in seen and l.get("src") in seen]
        pl.append(dict({"src": ln["src"], "dst": probe}, **({"sport": ln["sport"]} if "sport" in ln else {})))
        d = G.Design({"blocks": pb, "lines": pl}, hw)
        d.compile({k: v for k, v in in_types.items() if k in d.inports})
        t = d.otype[probe]
        if t == G.BOOL or t[0] != 0 or t[2] != 0:
            _refuse(where, "the address signal has the type %s: an unsigned integer type (ufixN) is required for an "
                    "inherited address width" % (d.type_str(probe),), "PORT_TYPE_INVALID")
        blocks[n] = dict(blocks[n], AddressBits=str(t[1]))
    return dict(spec, blocks=blocks)


def _port_block(info, s, sp, where):
    """The engine block carrying output port sp of user block s (the port was dropped, so a counter's
    boolean hit compared as its count, and a port that does not exist was accepted)."""
    if s in info.get("alias", {}):   # an identity block removed by the lowering (engine/block_lowering.py)
        s, sp = info["alias"][s]
    if sp == 1:
        return info.get("first", {}).get(s, s)
    m = info.get("ports", {}).get(s, {}).get(sp) or (info.get("port2", {}).get(s) if sp == 2 else None)
    if m is None:
        import fxgraph_lib as G
        raise G.GraphError("BAD_CONNECTION", where, "output port %d of %s does not exist" % (sp, s))
    return m


def _owner_where(p, where):
    """Errors of an internal core block name the user's block (its Owner)."""
    o = p.get("Owner")
    return "design.blocks." + o if o else where


# ---------------------------------------------------------------- assumed defaults of recorded lowered blocks
def lab2_assumed_defaults(spec):
    """{user block: {parameter: value}} -- the parameters a the extended blocks of the design leaves out and the values the
    lowering uses for them (the lowering replaces these blocks, so fxgraph's own assumed_defaults
    cannot see them). Only blocks recorded lowers; a plain Delay (no enable / reset port) is not one of them."""
    import fxhdlfir
    import fxhdlnco
    tables = {"HDLNCO": fxhdlnco.DEFAULTS, "HDLFIRFilter": fxhdlfir.FIR_DEFAULTS,
              "HDLFIRDecimator": fxhdlfir.DEC_DEFAULTS, "HDLRAM": RAM_DEFAULTS, "HDLCounter": COUNTER_DEFAULTS,
              "DetectRisePositive": DRP_DEFAULTS, "DirectLookupND": DLUT_DEFAULTS, "Repeat": REPEAT_DEFAULTS}
    tables.update({t: SYNC_DEFAULTS for t in SYNC})
    tables.update(BIT_DEFAULTS)
    import block_lowering   # Bias and the other recordings blocks list their own dialog values (the Bias value)
    tables.update({t: v for t, v in block_lowering.DEFAULTS.items() if t not in tables})
    out = {}
    for n, b in (spec.get("blocks") or {}).items():
        if not isinstance(b, dict) or any(ch in n for ch in G.RESERVED_CHARS):
            continue
        t = b.get("type")
        tab = tables.get(t)
        if t == "Delay" and (str(b.get("ShowEnablePort", "off")).strip() == "on" or
                             str(b.get("ExternalReset", "None")).strip() != "None"):
            tab = PDELAY_DEFAULTS
        if not tab:
            continue
        tab = dict(tab, **LIBRARY_DEFAULTS.get(t, {}))   # the library's value where it differs
        miss = {k: v for k, v in tab.items() if v is not None and k not in b}
        if miss:
            out[n] = dict(sorted(miss.items()))
    return out


# ---------------------------------------------------------------- block cost (recorded hook, fxgopt_lib.cost_hooks)
# a DSP HDL NCO was counted 34 bits in one setting (the kernel block: its output only)
# and 229,765 in another (the lowered design: a full-period lookup table counted entry by entry). One rule from the
# parameters, whatever the lowering: the phase accumulator (AccumulatorWL), the quantised phase (Q bits), one
# quarter-wave table of 2^(Q-2) output words (the HDL implementation's symmetry), and the pipeline of LATENCY steps
# of the outputs and their valid. The HDL RAM and the FIR blocks keep the ordinary counting (None): their word
# lengths come from the compiled input types, which the hook must not read.
def _nco_owner(d, name):
    s = name.lstrip(INTERNAL)
    if "/l/" in s:
        o = s.split("/l/")[0]
        if o in d.names and d.btype.get(INTERNAL + o + "/l/acc") is not None or INTERNAL + o + "/l/acc" in d.names:
            return o
        return None
    if d.btype.get(name) == "NCOCore":   # the exact kernel (increment port, large quantisers): its Owner
        return d.params.get(name, {}).get("Owner") or None
    if d.btype.get(name) == "HDLNCO" or (INTERNAL + name + "/l/acc") in d.names:
        return name
    return None


def _ram_owners(d):
    """{user HDL RAM: (words, read ports)} from the lowered structure (cached on d): the per-address lowering's word
    registers <X>w0 .. and write selects <X>wsel0 .., or a deep RAM's RAMCore blocks (their Owner and A)."""
    c = getattr(d, "_lab2_ram_owners", None)
    if c is not None:
        return c
    c = {}
    for n in d.names:
        if n.startswith(INTERNAL) or (INTERNAL + n + "/wsel0") not in d.names:
            continue
        w = 0
        while (INTERNAL + n + "/w%d" % w) in d.names:
            w += 1
        c[n] = (w, 2 if (INTERNAL + n + "/rd_dout") in d.names or (INTERNAL + n + "/u8_rd_dout") in d.names else 1)
    for n in d.names:
        if d.btype.get(n) == "RAMCore":
            o = d.params.get(n, {}).get("Owner")
            try:
                a_ = int(d.params[n].get("A", "0"))
            except (TypeError, ValueError):
                continue
            if o and o in d.names:
                w, k = c.get(o, (1 << a_, 0))
                c[o] = (w, k + 1)
    # HDL FIFO (engine/fxhdlfifo.py): its data registers <X>q0 .., the count register <X>count
    for n in d.names:
        if not n.startswith(INTERNAL) and (INTERNAL + n + "/hout") in d.names and (INTERNAL + n + "/count") in d.names:
            w = 0
            while (INTERNAL + n + "/q%d" % w) in d.names:
                w += 1
            c[n] = ("fifo", w)
    try:
        d._lab2_ram_owners = c
    except AttributeError:
        pass
    return c


# the HDL Counter lowering (its output register is the Unit Delay under the user's name; constants,
# up / down adders and the next-value cast as helpers) and the Bit Concat lowering (shifted / cast parts) -- counted
# by the documented rule as one block of the output's word length, not as their helper blocks
_HELPERS = {"UnitDelay": re.compile(r"^\x1d(.+)/(init|step|max|up|down|next)$"),
            "Sum": re.compile(r"^\x1d(.+)/cat\d+_(bits|sh|u)$")}   # (Bit Concat: a Sum of shifted parts)


def _counter_owner(d, name):
    """The user HDL Counter an internal block belongs to (its output register is the Unit Delay under the user's
    name, fed by the lowering's '<X>next' cast; every '<X>...' block is its helper), else None."""
    if not name.startswith(INTERNAL):
        return None
    o = name[len(INTERNAL):].split("/")[0]
    if d.btype.get(o) == "UnitDelay" and (INTERNAL + o + "/next") in d.names:
        return o
    return None


def _helper_owner(d, name):
    """(owner, True) for a helper block of a lowered HDL Counter / Bit Concat, (name, False) for such an owner (it has
    helpers), else None."""
    o = _counter_owner(d, name)
    if o is not None:
        return o, True
    if d.btype.get(name) == "UnitDelay" and (INTERNAL + name + "/next") in d.names and not name.startswith(INTERNAL):
        # (a counter inside another lowering -- a DSP HDL FIR decimator's phase -- belongs to that block)
        return name, False
    for bt, rx in _HELPERS.items():
        m = rx.match(name)
        if m and d.btype.get(m.group(1)) == bt and not m.group(1).startswith(INTERNAL):
            return m.group(1), True
    if d.btype.get(name) in _HELPERS:
        own = getattr(d, "_lab2_helper_owners", None)
        if own is None:
            own = set()
            for x in d.names:
                for bt, rx in _HELPERS.items():
                    m = rx.match(x)
                    if m and d.btype.get(m.group(1)) == bt and not m.group(1).startswith(INTERNAL):
                        own.add(m.group(1))
            try:
                d._lab2_helper_owners = own
            except AttributeError:
                pass
        if name in own:
            return name, False
    return None


def _lab2_groups(d):
    """{user block: [helpers present in d]} for every user block a recorded lowering expanded (its helpers: the internal
    names the lowering added, '<X>...' under the longest user name), cached on d. recordings: those helpers are not
    hardware of their own -- the user block is counted by its outputs (and a Direct Lookup Table by its table)."""
    c = getattr(d, "_lab2_cost_groups", None)
    if c is not None:
        return c
    info = getattr(d, "hdl_info", None) or {}
    users = info.get("user_types") or {}
    c = {}
    for h in info.get("lab2_helpers") or ():
        if h not in d.btype:
            continue
        own = _owner_by_name(h, users)
        if own is not None:
            c.setdefault(own, []).append(h)
    # recorded blocks lowered before this pass (fxvector: a Tapped Delay's stages '<name>\x1dd1' ...; the user's
    # the extended blocks as Design recorded them before any lowering, parameter_bounds.user_blocks)
    early = getattr(d, "_lab2_user", None) or {}
    if early:
        ut_ = {u: (v[0] if isinstance(v, (tuple, list)) else v) for u, v in early.items()}
        for x in d.names:
            if INTERNAL in x and not any(x in v for v in c.values()):
                u = _owner_by_name(x, ut_)
                if u is not None and x.lstrip(INTERNAL).startswith(u + INTERNAL):
                    c.setdefault(u, []).append(x)
    try:
        d._lab2_cost_groups = c
    except AttributeError:
        pass
    return c


_PORT_TYPES = ("TriggerPort", "EnablePort", "ActionPort", "ResetPort")


def _owner_by_name(h, users):
    """The user block a helper name belongs to: '<user>/...' or '<user>\x1d...' (after the leading marks), a
    subsystem's control logic ('<sub>/es/...', 'trg_<sub>_<port>_...') to that subsystem's port block."""
    s_ = h.lstrip(INTERNAL)
    own = max((u for u in users if s_.startswith(u + "/") or s_.startswith(u + INTERNAL)), key=len, default=None)
    if own is not None and users.get(own) not in ("SubSystem",):
        return own
    for u, t in users.items():
        if t in _PORT_TYPES and "/" in u:
            sub = u.rsplit("/", 1)[0]
            if s_.startswith(sub + "/es/") or s_.startswith("trg_" + u.replace("/", "_") + "_") or \
                    s_.startswith("trg_" + sub.replace("/", "_") + "_"):
                return u
    return own


def _generic_owner(d, name):
    g = _lab2_groups(d)
    if name in g:
        return name
    if INTERNAL in name:
        for u, hs in g.items():
            if name in hs:
                return u
    return None


def _generic_bits(d, owner):
    """The recordings count of a user block a recorded lowering expanded: the word of each of its output signals (the user
    block's own name when it is a block of d; a helper whose signal leaves the group), a Direct Lookup Table's
    table entries too (the documented lookup rule); 0 bits of its own."""
    hs = set(_lab2_groups(d).get(owner, ()))
    grp = hs | ({owner} if owner in d.btype else set())
    outs = {owner: 1} if owner in d.btype else {}
    for x in d.names:
        if x in grp:
            continue
        for s_ in d.src.get(x, ()) or ():
            if s_ in hs:
                outs[s_] = 1
    ut = ((getattr(d, "hdl_info", None) or {}).get("user_types") or {}).get(owner) or \
        ((getattr(d, "_lab2_user", None) or {}).get(owner) or (None,))[0]
    if ut == "DirectLookupND":
        for h in hs:
            if re.match(r"^t\d+$", h.replace(INTERNAL, "").split("/")[-1]) and d.btype.get(h) == "Constant":
                outs[h] = 1
    return 0, ("%s: its outputs%s (the lowering's helper blocks are not hardware of their own)"
               % (ut or "lowered block", " and its table" if ut == "DirectLookupND" else "")), outs


def cost_owner(d, name):
    """The user HDL NCO, HDL RAM, HDL FIFO, DSP HDL FIR, HDL Counter or Bit Concat an (internal or output) block of
    the compiled design belongs to; for any other block a recorded lowering expanded, that user block; else
    None."""
    o = _cost_owner_specific(d, name)
    if o is not None:
        return o
    return _generic_owner(d, name)


def _cost_owner_specific(d, name):
    o = _nco_owner(d, name)
    if o is not None:
        return o
    h = _helper_owner(d, name)
    if h is not None:
        return h[0]
    if d.btype.get(name) == "HDLFIRCore":
        return d.params.get(name, {}).get("Owner") or None
    firs = _fir_lowered(d)
    if name in firs:
        return name
    bare = name.lstrip(INTERNAL)
    if bare != name:
        best = None
        for f in firs:
            if bare.startswith(f + "/f/") and (best is None or len(f) > len(best)):
                best = f
        if best is not None:
            return best
    rams = _ram_owners(d)
    if name in rams:
        return name
    if name.startswith(INTERNAL):
        best = None
        for r in rams:
            if name.startswith(INTERNAL + r + "/") and (best is None or len(r) > len(best)):
                best = r
        return best
    return None


def inner_bits(d, owner):
    """(bits, explanation) of a user HDL NCO from its parameters (see above); for a user HDL RAM (bits,
    explanation, per_signal): its words and output registers, each as wide as the data -- {the RAM's output block:
    words + read ports}, priced by fxgopt_lib at that block's compiled word length (boolean data: 1 bit; the
    lowering's uint8 store of a boolean word is not hardware); else None."""
    if _cost_owner_specific(d, owner) is None and owner in _lab2_groups(d):
        return _generic_bits(d, owner)
    h = _helper_owner(d, owner)
    if h is not None and not h[1]:
        hit = INTERNAL + owner + "/hit"
        if d.btype.get(owner) == "UnitDelay" and hit in d.names:
            # the count_hit output is the block's second output: its word (boolean, 1) counts as an output
            return 0, ("HDL Counter: its output register and its hit output (the lowering's helper blocks are not "
                       "hardware of their own)"), {owner: 1, hit: 1}
        return 0, ("%s: its output register (the lowering's helper blocks are not hardware of their own)"
                   % ("HDL Counter" if d.btype.get(owner) == "UnitDelay" else "Bit Concat")), {owner: 1}
    if d.btype.get(owner) == "HDLFIRCore":
        return _fir_bits(d, owner)
    firs = _fir_lowered(d)
    if owner in firs:
        n, coefs, dec, src = firs[owner]
        return _fir_estimate(n, coefs, "HDLFIRDecimator" if dec > 1 else "HDLFIRFilter",
                             {"DecimationFactor": str(dec)}, src, owner)
    rams = _ram_owners(d)
    if owner in rams and rams[owner][0] == "fifo":
        import fxhdlfifo
        nq = rams[owner][1]
        B = fxhdlfifo.count_bits(nq)
        return B, ("HDL FIFO: %d data registers + the Out register, each the data's word length, + a %d-bit count"
                   % (nq, B)), {owner: nq + 1}
    if owner in rams:
        w, k = rams[owner]
        return 0, ("HDL RAM: %d words + %d output register%s, each the data's word length" % (w, k, "s" if k > 1
                                                                                                else "")), {owner: w + k}
    import fxhdlnco
    import json as _json
    core = None
    if d.btype.get(owner) == "NCOCore":
        try:
            core = dict(_json.loads(d.params[owner]["Params"]), type="HDLNCO")
        except Exception:   # noqa: BLE001
            return None
    if d.btype.get(owner) == "HDLNCO" or core is not None:
        try:
            p, A, Q, _inc = fxhdlnco.params(core or d.params[owner], "design.blocks." + owner)
        except Exception:   # noqa: BLE001 -- the block's own compile reports it
            return None
        WL = int(p["OutputWL"])
        nw = 2 if p.get("Waveform") in ("Sine and cosine", "Complex exponential") else 1
    else:
        x = INTERNAL + owner + "/l/"
        acc_t = str(d.params.get(x + "inc", {}).get("OutDataTypeStr", ""))
        luts = [n for n in d.names if n.startswith(x + "lut_")]
        if not acc_t.startswith("fixdt(") or not luts:
            return None
        A = int(acc_t[6:-1].split(",")[1])
        table = str(d.params[luts[0]].get("Table", "[]")).strip("[] ")
        entries = len(table.split()) if table else 0
        Q = max(entries - 1, 1).bit_length()
        zero = d.params.get(x + "zero", {}).get("OutDataTypeStr", "fixdt(1,16,14)")
        WL = int(str(zero)[6:-1].split(",")[1]) if str(zero).startswith("fixdt(") else 16
        nw = len(luts)
    lut = (1 << max(Q - 2, 0)) * WL
    pipe = fxhdlnco.LATENCY * (WL * nw + 1)
    bits = A + Q + lut + pipe
    return bits, ("HDL NCO: phase accumulator %d + quantised phase %d + quarter-wave table 2^%d x %d = %d + pipeline "
                  "%d x (%d x %d + 1) = %d" % (A, Q, max(Q - 2, 0), WL, lut, fxhdlnco.LATENCY, WL, nw, pipe))

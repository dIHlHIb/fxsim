"""Validate popup options on supported blocks. Export checks covered the options listed below.
Native handling of surrounding spaces is unverified, so options with surrounding spaces are refused."""
import fxgraph_lib as G

POPUPS = {
    "ActionPort": ("InitializeStates",),
    "BiquadFilter": ("IIRFiltStruct", "outputMode"),
    "BitReduce": ("mode",),
    "ComplexToMagnitudeAngle": ("ApproximationMethod",),
    "MagnitudeAngleToComplex": ("ApproximationMethod",),
    "DiscreteFilter": ("FilterStructure",),
    "HDLFIFO": ("mode", "pop_msg", "push_msg"),
    "HDLFIRDecimator": ("FilterStructure", "NumeratorSource", "SimulateUsing", "roundingMode"),
    "HDLFIRFilter": ("CoeffMultiplier", "CoeffMultiplierSystolic", "FilterStructure", "NumeratorSource",
                     "SerializationOption", "SimulateUsing", "roundingMode"),
    "HDLNCO": ("AccumulatorDataType", "AccumulatorSigned", "DitherSource", "OutputDataType", "OutputSigned",
               "OverflowAction", "PhaseIncrementSource", "PhaseOffsetSource", "RoundingMethod", "SimulateUsing",
               "Waveform"),
    "Interpolation_n-D": ("ExtrapMethod", "InternalRulePriority", "InterpMethod"),
    "PreLookup": ("BreakpointsSpecification", "ExtrapMethod"),
    "ResetPort": ("ResetTriggerType",),
    "TriggerPort": ("StatesWhenEnabling", "TriggerType"),
    "Reshape": ("OutputDimensionality",),
    "Sqrt": ("AlgorithmType",),
}


# free text, not an expression: MATLAB code (a MATLAB Function's Script, callbacks) spans lines and its comments may
# hold any character
TEXT_PARAMS = frozenset(("Script", "AssertionFailFcn", "MaskDisplay", "OpenFcn", "InitFcn", "StartFcn", "StopFcn",
                         "CopyFcn", "DeleteFcn", "LoadFcn", "PreSaveFcn", "PostSaveFcn", "CloseFcn", "MoveFcn",
                         "NameChangeFcn", "ParentCloseFcn", "PreCopyFcn", "PreDeleteFcn", "UndoDeleteFcn",
                         "ModelCloseFcn", "DestroyFcn", "ClipboardFcn", "ContinueFcn", "PauseFcn"))


def _bad_char(v):
    """A character MATLAB rejects in a parameter value: outside printable ASCII (tab, newline and CR are fine).
    recordings (b): trailing whitespace -- vertical tab / form feed included -- is trimmed by MATLAB ('int8(3)\\x0b'
    and '3\\x0b' run, bit-equal); a leading one is an error."""
    v = v.rstrip(" \t\n\r\x0b\x0c")
    return next((ch for ch in v if ord(ch) > 126 or (ord(ch) < 32 and ch not in "\t\n\r")), None)


def check(design):
    if not (isinstance(design, dict) and isinstance(design.get("blocks"), dict)):
        return
    for n, b in design["blocks"].items():
        if not isinstance(b, dict):
            continue
        # recordings (BitMask '3' + U+00A0 answered): main checks the parameters that pass its _check_params; the blocks
        # recorded lowers are checked here, at the entry every tool goes through (cosmetic parameters excepted)
        for k, v in b.items():
            if k == "type" or k in G.COSMETIC_COMMON or k in TEXT_PARAMS or not isinstance(v, str):
                continue
            ch = _bad_char(v)
            if ch is not None:
                raise G.GraphError("BAD_PARAM", "design.blocks.%s.%s" % (n, k), "a character outside printable ASCII "
                                   "(%r) in %r: MATLAB rejects it" % (ch, v))
            # a line break inside the value (leading / trailing ones are trimmed). Outside brackets
            # MATLAB ends the expression there (Delay '2\n+1': BlkParamEvalErr, it was read as 2+1); inside brackets
            # it starts a new row ('[1\n2\n3]' is a column), which readers took as a space -- not supported
            t_ = v.strip()
            if "\n" in t_ or "\r" in t_:
                depth, inside = 0, False
                for c_ in t_:
                    if c_ in "[{(":
                        depth += 1
                    elif c_ in "]})":
                        depth -= 1
                    elif c_ in "\r\n":
                        if depth <= 0:
                            raise G.GraphError("BAD_PARAM", "design.blocks.%s.%s" % (n, k), "a line break inside "
                                               "%r: MATLAB ends the expression there and Simulink rejects the value "
                                               "(BlkParamEvalErr)" % v)
                        inside = True
                if inside:
                    raise G.GraphError("UNSUPPORTED", "design.blocks.%s.%s" % (n, k), "%r: a line break inside "
                                       "brackets starts a new row in MATLAB; not supported here -- write the rows "
                                       "with ';' (or the values on one line)" % v)
        for k in POPUPS.get(b.get("type"), ()):
            v = b.get(k)
            if isinstance(v, str) and v.strip() and v != v.strip():
                raise G.GraphError("UNSUPPORTED", "design.blocks.%s.%s" % (n, k), "%s %s=%r: surrounding spaces are "
                                   "not supported here; write the option exactly" % (b["type"], k, v))

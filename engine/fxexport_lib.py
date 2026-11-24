"""fxexport: build a new Simulink .slx model from scratch out of an fxgraph design, plus a verification package
(inputs, fxsim's expected stored integers, a self-contained MATLAB check script).

The .slx layout follows what R2026b itself writes, as observed in the models under recordings (zip of OPC parts:
JSON model-level parts, one XML part per system, SID#out:k / SID#in:k line endpoints, library blocks as
Reference blocks with their mask values under InstanceData). See docs/en/EXPORT.md.

Rules (every one of them is a refusal, never a guess):
  * only block types with a template in TEMPLATES are exported; a template whose Simulink form was never observed
    in a genuine file is flagged "observed": false in the manifest;
  * every parameter the engine uses is written explicitly (the design's value, else the engine's default), so the
    model never falls back on a Simulink default that the engine did not assume;
  * the production hardware must be one Simulink device profile (host x86-64 Windows64, or ASIC/FPGA);
  * the export is read back with the project's own .slx reader and re-simulated: any difference in structure,
    parameters, compiled types or outputs refuses the export (EXPORT_ROUNDTRIP_MISMATCH).
"""
import hashlib
import io
import json
import re
import zipfile
from fractions import Fraction
from xml.sax.saxutils import escape, quoteattr

import fxgraph_lib as G
import fxslx_lib as S
import blockspec as BS
import fxvector as V

FORMAT_IO = "fxexport-io/1"
RELEASE = "R2026b"
CC_VERSION = "26.1.1"


class ExportError(G.GraphError):
    pass


# ---------------------------------------------------------------- block templates
# engine type -> (Simulink BlockType, library (SourceBlock, SourceType, LibraryVersion) or None,
#                 extra fixed parameters written for every instance, observed in a genuine R2026b file?)
_LOGIC = "simulink/Logic and Bit\nOperations/"
TEMPLATES = {
    "Gain": ("Gain", None, {}, True),
    "Sum": ("Sum", None, {}, True),
    "Product": ("Product", None, {}, True),
    "DataTypeConversion": ("DataTypeConversion", None, {}, True),
    "Saturation": ("Saturate", None, {}, True),
    "RelationalOperator": ("RelationalOperator", None, {}, True),
    "MinMax": ("MinMax", None, {}, True),
    "Abs": ("Abs", None, {}, True),
    "UnaryMinus": ("UnaryMinus", None, {}, True),
    "UnitDelay": ("UnitDelay", None, {}, True),
    "Constant": ("Constant", None, {}, True),
    # the enable / reset ports are written as the design gives them (HIDDEN_DEFAULTS: off / None)
    # (a previous version, recordings: a length / initial condition from an input port is answered -- written as
    # the design has it; the extra input ports come from the design's lines)
    "Delay": ("Delay", None, {}, True),
    "ShiftArithmetic": ("ArithShift", None, {}, True),   # BitShiftNumberSource as the design has it
    "LogicalOperator": ("Logic", None, {}, True),
    "Switch": ("Switch", None, {}, True),
    "MultiportSwitch": ("MultiPortSwitch", None, {}, True),
    "DiscreteFir": ("DiscreteFir", None, {"CoefSource": "Dialog parameters"}, True),
    # (InitialConditionSource is the design's since a previous version answers an external initial condition; the
    # library value is internal)
    "DiscreteIntegrator": ("DiscreteIntegrator", None, {}, True),
    "Signum": ("Signum", None, {}, True),
    "Rounding": ("Rounding", None, {}, True),
    "Lookup1D": ("Lookup_n-D", None, {"NumberOfTableDimensions": "1"}, True),
    "Lookup2D": ("Lookup_n-D", None, {"NumberOfTableDimensions": "2"}, True),
    # the same block with 3 / 4 dimensions
    "LookupND": ("Lookup_n-D", None, {}, True),
    "Downsample": ("DownSample", None, {"InputProcessing": "Elements as channels (sample based)"}, True),
    "RateTransition": ("RateTransition", None, {}, True),
    "Mux": ("Mux", None, {}, True),
    "Demux": ("Demux", None, {}, True),
    "Selector": ("Selector", None, {}, True),
    "CompareToConstant": ("Reference", (_LOGIC + "Compare\nTo Constant", "Compare To Constant", "10.1"), {}, True),
    "CounterLimited": ("Reference", ("simulink/Sources/Counter\nLimited", "Counter Limited", "10.1"), {}, True),
    "CounterFreeRunning": ("Reference", ("simulink/Sources/Counter\nFree-Running", "Counter Free-Running", "10.1"),
                           {}, True),
    "DetectChange": ("Reference", (_LOGIC + "Detect\nChange", "Detect Change", "10.1"),
                     {"InputProcessing": "Elements as channels (sample based)"}, True),
    "DetectIncrease": ("Reference", (_LOGIC + "Detect\nIncrease", "Detect Increase", "10.1"),
                       {"InputProcessing": "Elements as channels (sample based)"}, True),
    "DetectDecrease": ("Reference", (_LOGIC + "Detect\nDecrease", "Detect Decrease", "10.1"),
                       {"InputProcessing": "Elements as channels (sample based)"}, True),
    "BitwiseOperator": ("Reference", (_LOGIC + "Bitwise\nOperator", "Bitwise Operator", "10.1"), {}, True),
    "ExtractBits": ("Reference", (_LOGIC + "Extract Bits", "Extract Bits", "10.1"), {}, True),
    "Upsample": ("Reference", ("dspsigops/Upsample", "Upsample", "13.2"),
                 {"FactorSource": "Dialog parameter", "InputProcessing": "Elements as channels (sample based)"}, True),
    "FIRDecimation": ("Reference", ("dspmlti4/FIR\nDecimation", "FIR Decimation", "14.2"),
                      {"FilterSource": "Dialog parameters", "InputProcessing": "Elements as channels (sample based)",
                       "framing": "Allow multirate processing"}, True),
    "FIRInterpolation": ("Reference", ("dspmlti4/FIR\nInterpolation", "FIR Interpolation", "14.2"),
                         {"FilterSource": "Dialog parameters",
                          "InputProcessing": "Elements as channels (sample based)",
                          "framing": "Allow multirate processing"}, True),
    "CICDecimation": ("Reference", ("dspmlti4/CIC\nDecimation", "CIC Decimation", "14.2"),
                      {"filtFrom": "Dialog parameters"}, True),
    # HDL library blocks
    "HDLCounter": ("Reference", ("hdlsllib/Sources/HDL Counter", "HDL Counter", "13.5"), {}, True),
    "UnitDelayEnabledSynchronous": ("Reference", ("hdlsllib/Discrete/Unit Delay Enabled\nSynchronous",
                                                  "Unit Delay Enabled Synchronous", "13.5"), {}, True),
    "UnitDelayEnabledResettableSynchronous": ("Reference", ("hdlsllib/Discrete/Unit Delay Enabled Resettable"
                                                            "\nSynchronous", "Unit Delay Enabled Resettable Synchronous",
                                                            "13.5"), {}, True),
    "UnitDelayResettableSynchronous": ("Reference", ("hdlsllib/Discrete/Unit Delay Resettable\nSynchronous",
                                                     "Unit Delay Resettable Synchronous", "13.5"), {}, True),
    "BitSlice": ("Reference", ("hdlsllib/Logic and Bit\nOperations/Bit Slice", "Bit Slice", "13.5"), {}, True),
    "BitConcat": ("Reference", ("hdlsllib/Logic and Bit\nOperations/Bit Concat", "Bit Concat", "13.5"), {}, True),
    "BitShift": ("Reference", ("hdlsllib/Logic and Bit\nOperations/Bit Shift", "Bit Shift", "13.5"), {}, True),
    # recordings (library paths as recorded, recordings -- the samples of that recording are
    # not these models, so the written blocks are not yet checked against a Simulink-saved file)
    "BitRotate": ("Reference", ("hdlsllib/Logic and Bit\nOperations/Bit Rotate", "Bit Rotate", "13.5"), {}, True),
    "BitSet": ("Reference", ("simulink/Logic and Bit\nOperations/Bit Set", "Bit Set", "10.1"), {}, True),
    "BitClear": ("Reference", ("simulink/Logic and Bit\nOperations/Bit Clear", "Bit Clear", "10.1"), {}, True),
    # complex converters
    "RealImagToComplex": ("RealImagToComplex", None, {}, True),
    "ComplexToRealImag": ("ComplexToRealImag", None, {}, True),
    # recordings (the native saved models): CORDIC complex <-> magnitude-angle
    "ComplexToMagnitudeAngle": ("ComplexToMagnitudeAngle", None, {}, True),
    "MagnitudeAngleToComplex": ("MagnitudeAngleToComplex", None, {}, True),
    # virtual buses; element names are written as the names of the input lines
    "BusCreator": ("BusCreator", None, {}, True),
    "BusSelector": ("BusSelector", None, {}, True),
    "BusAssignment": ("BusAssignment", None, {}, True),
    # exported only for runs in which it never fails (export_package)
    "Assertion": ("Assertion", None, {}, True),   # recordings (AssignedSignals by name)
    # enabled subsystems and the masked HDL RAMs (library chosen by RAMType)
    "EnablePort": ("EnablePort", None, {}, True),
    # recordings (the native saved models): triggered / resettable subsystems
    "TriggerPort": ("TriggerPort", None, {}, True),
    "ResetPort": ("ResetPort", None, {}, True),
    # recordings (the native saved models): Switch Case action subsystems, Merge, Data Store
    "SwitchCase": ("SwitchCase", None, {}, True),
    "ActionPort": ("ActionPort", None, {}, True),
    "Merge": ("Merge", None, {}, True),
    "DataStoreMemory": ("DataStoreMemory", None, {}, True),
    "DataStoreRead": ("DataStoreRead", None, {}, True),
    "DataStoreWrite": ("DataStoreWrite", None, {}, True),
    # Prelookup (index, fraction) and Interpolation Using Prelookup
    "PreLookup": ("PreLookup", None, {}, True),
    "Interpolation_n-D": ("Interpolation_n-D", None, {}, True),
    # a previous version: the same two blocks in the engine's spelling (read back in recorded;
    # fxgraph answers either spelling with the rule set that covers the shape)
    "Prelookup": ("PreLookup", None, {}, True),
    "InterpolationUsingPrelookup": ("Interpolation_n-D", None, {}, True),
    # recordings (recordings.slx): the For Each block of an atomic subsystem level
    "ForEach": ("ForEach", None, {}, True),
    "StateControl": ("StateControl", None, {}, True),
    "EnabledOutport": ("Outport", None, {}, True),
    "HDLRAM": ("Reference", None, {}, True),
    # recorded configurations (recorded samples: block types, library paths, versions)
    "SignalSpecification": ("SignalSpecification", None, {}, True),
    "Bias": ("Bias", None, {}, True),
    "ManualSwitch": ("ManualSwitch", None, {}, True),
    "Reshape": ("Reshape", None, {}, True),
    "Math": ("Math", None, {}, True),
    "Ground": ("Ground", None, {}, True),
    "BitReduce": ("Reference", ("hdlsllib/Logic and Bit\nOperations/Bit Reduce", "Bit Reduce", "13.5"), {}, True),
    "IncrementRealWorld": ("Reference", ("simulink/Additional Math\n& Discrete/Additional Math:\nIncrement - "
                                         "Decrement/Increment\nReal World", "Real World Value Increment", "10.1"), {},
                           True),
    "DecrementRealWorld": ("Reference", ("simulink/Additional Math\n& Discrete/Additional Math:\nIncrement - "
                                         "Decrement/Decrement\nReal World", "Real World Value Decrement", "10.1"), {},
                           True),
    "IncrementStoredInteger": ("Reference", ("simulink/Additional Math\n& Discrete/Additional Math:\nIncrement - "
                                             "Decrement/Increment\nStored Integer", "Stored Integer Value Increment",
                                             "10.1"), {}, True),
    "DecrementStoredInteger": ("Reference", ("simulink/Additional Math\n& Discrete/Additional Math:\nIncrement - "
                                             "Decrement/Decrement\nStored Integer", "Stored Integer Value Decrement",
                                             "10.1"), {}, True),
    # recorded samples (library path, source type and version as saved)
    "TappedDelay": ("Reference", ("simulink/Discrete/Tapped Delay", "Tapped Delay Line", "10.1"), {}, True),
    "TappedDelayEnabledSynchronous": ("Reference", ("hdlsllib/Discrete/Tapped Delay Enabled Synchronous",
                                                    "Tapped Delay Enabled Synchronous", "13.5"), {}, True),
    # DSP HDL blocks (library path, source type and version as in recorded samples)
    "HDLFIRFilter": ("Reference", ("dsphdlfiltering2/Discrete FIR Filter", "Discrete FIR Filter", "12.1"), {}, True),
    "HDLFIRDecimator": ("Reference", ("dsphdlfiltering2/FIR Decimator", "FIR Decimator", "12.1"), {}, True),
    "HDLNCO": ("Reference", ("dsphdlsigops2/NCO", "NCO", "12.0"), {}, True),
    # HDL FIFO (recorded samples: library path, source type and version as saved)
    "HDLFIFO": ("Reference", ("hdlsllib/HDL RAMs/HDL FIFO", "HDL_FIFO", "13.5"), {}, True),
    
    "DataTypeDuplicate": ("DataTypeDuplicate", None, {}, True),
    "Concatenate": ("Concatenate", None, {}, True),
    "DirectLookupND": ("LookupNDDirect", None, {}, True),
    "DetectRisePositive": ("Reference", (_LOGIC + "Detect Rise\nPositive", "Detect Rise Positive", "10.1"), {}, True),
    # the engine's basic partitions recordings (
    # recorded samples: block types, library paths and versions as
    # saved)
    "CompareToZero": ("Reference", (_LOGIC + "Compare\nTo Zero", "Compare To Zero", "10.1"), {}, True),
    "DetectRiseNonnegative": ("Reference", (_LOGIC + "Detect Rise\nNonnegative", "Detect Rise Nonnegative", "10.1"), {},
                              True),
    "DetectFallNegative": ("Reference", (_LOGIC + "Detect Fall\nNegative", "Detect Fall Negative", "10.1"), {}, True),
    "DetectFallNonpositive": ("Reference", (_LOGIC + "Detect Fall\nNonpositive", "Detect Fall Nonpositive", "10.1"), {},
                              True),
    "WrapToZero": ("Reference", ("sl_discontinuities/Wrap To Zero", "Wrap To Zero", "3.1",
                                 "simulink/Discontinuities/Wrap To Zero"), {}, True),
    "Sqrt": ("Sqrt", None, {}, True),
    # built-in blocks; only the design's parameters are written
    # (the library values of the rest are the ones fxiir assumes, recordings)
    "ZeroOrderHold": ("ZeroOrderHold", None, {}, True),   # recordings (engine/fxzoh.py)
    "BiquadFilter": ("Reference", ("dsparch4/Biquad Filter", "Biquad Filter", "15.3"), {}, True),   
    "DiscreteFilter": ("DiscreteFilter", None, {}, True),
    "DiscreteTransferFcn": ("DiscreteTransferFcn", None, {}, True),
    "Trigonometry": ("Trigonometry", None, {}, True),
    "DeadZone": ("DeadZone", None, {}, True),
    "RateLimiter": ("RateLimiter", None, {}, True),
    "Repeat": ("Reference", ("dspsigops/Repeat", "Repeat", "13.2"), {}, True),
    # never seen in a genuine file: the library path is the Decimation one's sibling (a wrong path fails to load
    # loudly, it cannot change values)
    # recordings (x07): without InputProcessing / framing the library's frame-based single-rate defaults apply
    # and the block outputs frames of R samples; the values below are the ones recordings show the engine's rules
    # with (and recordings needed for multirate processing)
    "CICInterpolation": ("Reference", ("dspmlti4/CIC\nInterpolation", "CIC Interpolation", "14.2"),
                         {"filtFrom": "Dialog parameters", "InputProcessing": "Elements as channels (sample based)",
                          "framing": "Allow multirate processing", "OutputFrames": "off"}, True),
}

# parameters the engine keeps in DEFAULTS that are not Simulink dialog parameters of the block (never written)
_NOT_SIMULINK = {"Hold": None}
# exported blocks without an output port (no compiled output type exists)
NO_OUTPUT_TYPES = ("Outport", "EnabledOutport", "DataTypeDuplicate", "EnablePort", "StateControl", "Terminator",
                   "Assertion", "ForEach", "ActionPort", "DataStoreMemory", "DataStoreWrite", "TriggerPort", "ResetPort")
# DSP mask types: the legacy Mode / WordLength / FracLength triple is written consistently with the DataTypeStr
_DSP_TYPED = ("firstCoeff", "prodOutput", "accum", "output")

# the System RAMs as saved in recordings (recordings*_system_*.slx); a non-zero initial value was recorded for the
# Simple Dual Port one (the other two)
_SYSRAM_LIB = {"Simple dual port": ("hdlsllib/HDL RAMs/Simple Dual Port\nRAM System", "hdl.RAM", "13.5"),
               "Dual port": ("hdlsllib/HDL RAMs/Dual Port RAM System", "hdl.RAM", "13.5"),
               "Single port": ("hdlsllib/HDL RAMs/Single Port RAM System", "hdl.RAM", "13.5")}
_SYSRAM_FIXED = {"AsyncRead": "off", "allowBusInputs": "off", "WritesHaveOutput": "on", "NumReads": "0",
                 "NumWrites": "1", "SimulateUsing": "Code generation", "ModelRAMDelay": "on", "VectorAccess": "Parallel"}
_RAM_LIB = {"Simple dual port": ("hdlsllib/HDL RAMs/Simple Dual Port RAM", "Simple Dual-port RAM", "13.5"),
            "Dual port": ("hdlsllib/HDL RAMs/Dual Port RAM", "Dual-port RAM", "13.5"),
            "Single port": ("hdlsllib/HDL RAMs/Single Port RAM", "Single-port RAM", "13.5")}

# blocks whose period is a dialog parameter that Simulink defaults to something other than "inherited"
# (Unit Delay and Discrete-Time Integrator 1, Constant inf); always written
_SAMPLE_TIME_PARAM = {"UnitDelay": "-1", "Delay": "-1", "DiscreteFir": "-1", "Gain": None, "Sum": None,
                      "Constant": "inf", "DiscreteIntegrator": "-1"}
_COUNTERS = ("CounterLimited", "CounterFreeRunning")

# parameters the engine reads with a default of its own when a design omits them, which is not (or not known to be)
# Simulink's default: written explicitly. InputSameDT / AllPortsSameDT: the engine never requires equal input types
# when they are absent, the saved R2026b files write "off" out (so Simulink's default is "on"); Lookup1D: the
# engine's inherited-type rules read an absent InternalRulePriority as Speed (ref/fxcore.py)
HIDDEN_DEFAULTS = {"Delay": {"ExternalReset": "None", "ShowEnablePort": "off"},
                   # recordings-a Zero-Order Hold saved without SampleTime loads 1 (get_param), the library block
                   # is -1 (inherited) -- always written
                   "ZeroOrderHold": {"SampleTime": "-1"},
                   # recordings (export): a Selector left without Indices / InputPortWidth is the library block's
                   # [1 3] of 3 to the engine; a saved block without them loads the
                   # built-in 1 / -1 -- written, so the model is the design
                   "Selector": {"Indices": "[1 3]", "InputPortWidth": "3"},
                   # Write the library default explicitly. Omitting it from a saved Downsample block loads 'off'.
                   "Downsample": {"AllowArbitraryFixedInput": "on"},
                   "Sum": {"InputSameDT": "off"}, "Product": {"InputSameDT": "off"}, "MinMax": {"InputSameDT": "off"},
                   "Switch": {"InputSameDT": "off"}, "MultiportSwitch": {"InputSameDT": "off"},
                   # recordings (R2026b built-in vs
                   # library defaults): a block saved without these loads the built-in value, not the library's --
                   # RelOp RndMeth Nearest (library Simplest, the engine's value: a package differed 4 / 36),
                   # OutDataTypeStr 'Inherit: Logical' (library boolean), Mux Inputs / Demux Outputs 4 (library and
                   # engine 2), Discrete-Time Integrator InitialConditionSetting Output (library Auto)
                   "RelationalOperator": {"InputSameDT": "off", "RndMeth": "Simplest", "OutDataTypeStr": "boolean"},
                   "LogicalOperator": {"AllPortsSameDT": "off", "OutDataTypeStr": "boolean"},
                   "Mux": {"Inputs": "2"}, "Demux": {"Outputs": "2"},
                   "DiscreteIntegrator": {"InitialConditionSetting": "Auto"},
                   # UseLastTableValue: the engine reads it left out as 'off' (fxgraph_lib, p.get(...) == "on") and
                   # the reader assumes the dialog's 'off' for a file without it; a 1-D table's file omits it
                   "Lookup1D": {"InternalRulePriority": "Speed", "InputSameDT": "off", "UseLastTableValue": "off"},
                   "Lookup2D": {"InputSameDT": "off"}}

# The exporter maps built-in engine types to library paths. Library-linked blocks use TEMPLATES SourceBlock.
# The check compares omitted parameters with fresh library defaults and reports blocks without a library path.
LIBRARY_PATHS = {
    # recordings (the engine's basic-partition blocks, library paths as recorded)
    "Sqrt": "simulink/Math\nOperations/Sqrt", "Trigonometry": "simulink/Math\nOperations/Trigonometric\nFunction",
    "DeadZone": "simulink/Discontinuities/Dead Zone", "RateLimiter": "simulink/Discontinuities/Rate Limiter",
    "Abs": "simulink/Math Operations/Abs", "Assertion": "simulink/Model\nVerification/Assertion",
    "Bias": "simulink/Math\nOperations/Bias", "Concatenate": "simulink/Commonly\nUsed Blocks/Vector\nConcatenate",
    "Constant": "simulink/Sources/Constant", "DataTypeConversion": "simulink/Signal Attributes/Data Type Conversion",
    "DataTypeDuplicate": "simulink/Signal\nAttributes/Data Type\nDuplicate", "Delay": "simulink/Discrete/Delay",
    "DirectLookupND": "simulink/Lookup\nTables/Direct Lookup\nTable (n-D)",
    "DiscreteFir": "simulink/Discrete/Discrete FIR Filter",
    "DiscreteIntegrator": "simulink/Discrete/Discrete-Time Integrator", "Downsample": "dspsigops/Downsample",
    "Gain": "simulink/Math Operations/Gain", "LogicalOperator": "simulink/Logic and Bit Operations/Logical Operator",
    "Lookup1D": "simulink/Lookup Tables/1-D Lookup Table", "Lookup2D": "simulink/Lookup Tables/2-D Lookup Table",
    "ManualSwitch": "simulink/Signal\nRouting/Manual Switch", "MinMax": "simulink/Math Operations/MinMax",
    "MultiportSwitch": "simulink/Signal Routing/Multiport Switch", "Product": "simulink/Math Operations/Product",
    "RateTransition": "simulink/Signal Attributes/Rate Transition",
    "RelationalOperator": "simulink/Logic and Bit Operations/Relational Operator",
    "Reshape": "simulink/Math\nOperations/Reshape", "Rounding": "simulink/Math Operations/Rounding Function",
    "Saturation": "simulink/Discontinuities/Saturation",
    "ShiftArithmetic": "simulink/Logic and Bit Operations/Shift Arithmetic", "Signum": "simulink/Math Operations/Sign",
    "Sum": "simulink/Math Operations/Sum", "Switch": "simulink/Signal Routing/Switch",
    "UnaryMinus": "simulink/Math Operations/Unary Minus", "UnitDelay": "simulink/Discrete/Unit Delay",
    # the recorded R2026b dump of library blocks (Mux /
    # Demux were not compared in Simulink without a path)
    "Mux": "simulink/Signal Routing/Mux", "Demux": "simulink/Signal Routing/Demux",
    "ComplexToRealImag": "simulink/Math Operations/Complex to Real-Imag", "Ground": "simulink/Sources/Ground", "Math": "simulink/Math Operations/Math Function", "RealImagToComplex": "simulink/Math Operations/Real-Imag to Complex", "Selector": "simulink/Signal Routing/Selector", "SignalSpecification": "simulink/Signal Attributes/Signal Specification"}
# parameters the export writes on purpose with a value other than the library block's (listed in the manifest with
# the reason, not compared): the engine never requires equal input types, the 1-D / 2-D Lookup library block does
# parameters whose engine default differs from the recorded library value (reported to recorded, which
# owns the engine tables): a design that leaves one out is not exported -- writing the engine's value would change the
# request's meaning, and the check script would stop at step 2b
OMITTED_REFUSED = {}   # (a previous version took the library's Sqrt OutDataTypeStr / SaturateOnIntegerOverflow: none left)
LIBRARY_DEVIATIONS = {
    ("Lookup1D", "InputSameDT"): "written 'off' (library 'on'): the engine does not require the inputs to share a "
                                 "type; a 1-D table has one input, so the value cannot change a result",
    ("Lookup2D", "InputSameDT"): "written 'off' (library 'on'): the engine does not require the two inputs to share "
                                 "a type; with 'on' Simulink refuses differing input types at compile time rather "
                                 "than computing differently"}


def library_layer(orig_blocks, written):
    """(io.json library_defaults entries, manifest library_deviations, manifest library_defaults_unchecked) for the
    request's own blocks: every parameter the request left out whose written value is the export's table default
    (engine table, JSON library defaults, HIDDEN_DEFAULTS, fxhdl.LIBRARY_DEFAULTS) -- a value derived from the design
    (port maps, one-input Sum signs, sample times) differs from the table and is not listed."""
    import fxhdl
    checks, devs, unchecked = [], [], []
    for name in sorted(written):
        full, et, p = written[name]
        ob = orig_blocks.get(name)
        if not isinstance(ob, dict) or any(ch in name for ch in G.RESERVED_CHARS):
            continue   # a block the export added (observation, lowering)
        tbl = {}
        for src in (BS.DEFAULTS.get(et, {}), BS.JSON_LIBRARY_DEFAULTS.get(et, {}), HIDDEN_DEFAULTS.get(et, {}),
                    fxhdl.LIBRARY_DEFAULTS.get(et, {})):
            tbl.update({k: str(v) for k, v in src.items()})
        left = sorted(k for k, v in tbl.items() if k not in ob and k in p and str(p[k]).strip() == v.strip()
                      and k not in TEMPLATES[et][2])
        if not left:
            continue
        lib = TEMPLATES[et][1][0] if TEMPLATES[et][1] else LIBRARY_PATHS.get(et)
        for k in left:
            if (et, k) in LIBRARY_DEVIATIONS:
                devs.append({"block": full, "param": k, "value": str(p[k]), "reason": LIBRARY_DEVIATIONS[(et, k)]})
            elif lib is None:
                unchecked.append({"block": full, "type": et, "param": k, "value": str(p[k])})
            else:
                checks.append({"block": full, "lib": lib, "param": k, "value": str(p[k])})
    return checks, devs, unchecked


# engine table entries that are not parameters of the Simulink block (no recordings get_param dump of the block has them):
# never written; a design asking for another value is refused
NOT_A_PARAMETER = {"CICInterpolation": {"RateOptions": "Allow multirate processing"},
                   # the CIC Decimation mask has no InputProcessing either -- get_param on it
                   # is Simulink:Commands:ParamUnknown, which stopped w02's check script before any sample
                   "CICDecimation": {"InputProcessing": "Elements as channels (sample based)"}}

# only layout / presentation parameters are not written (listed in the manifest); every other parameter of the
# design is written as given -- blockspec.META_ANY is not a safe "no effect" list for export (the engine reads
# InternalRulePriority, FractionDataTypeStr and TapSumDataTypeStr from it; dropping the first silently changed a
# lookup table's fraction rule, found by the regression check on recordings)
_DROPPED = set(BS.COSMETIC) | {"Port"}


def _canon(obj):
    return json.dumps(obj, sort_keys=True, separators=(",", ":"))


def _uuid(seed, tag):
    h = hashlib.sha256((seed + "|" + tag).encode()).hexdigest()
    return "%s-%s-4%s-a%s-%s" % (h[:8], h[8:12], h[13:16], h[17:20], h[20:32])


# ---------------------------------------------------------------- hardware profiles
HOST_DEVICE = "Intel->x86-64 (Windows64)"
_ASIC_HW = {"ProdBitPerPointer": 32, "ProdBitPerPtrDiffT": 32, "ProdBitPerSizeT": 32, "ProdEndianess": "Unspecified",
            "ProdEqTarget": "off", "ProdHWDeviceType": "ASIC/FPGA", "ProdIntDivRoundTo": "Undefined",
            "ProdLargestAtomicFloat": "None", "ProdWordSize": 32, "TargetBitPerPointer": 64,
            "TargetBitPerPtrDiffT": 64, "TargetBitPerSizeT": 64, "TargetEndianess": "LittleEndian",
            "TargetHWDeviceType": "Custom Processor->MATLAB Host Computer", "TargetIntDivRoundTo": "Zero",
            "TargetLongLongMode": "on", "TargetWordSize": 64}


# integer sizes Simulink reports under ASIC/FPGA (recordings show them; the engine uses none of them there)
_ASIC_SIZES = {"ProdBitPerChar": "8", "ProdBitPerShort": "16", "ProdBitPerInt": "32", "ProdBitPerLong": "32",
               "ProdBitPerLongLong": "64", "ProdLongLongMode": "off"}
# the x86-64 (Windows64) production settings (recordings show them with get_param)
_HOST_VALUES = {"ProdBitPerChar": "8", "ProdBitPerShort": "16", "ProdBitPerInt": "32", "ProdBitPerLong": "32",
                "ProdBitPerLongLong": "64", "ProdLongLongMode": "off", "ProdWordSize": "64", "ProdIntDivRoundTo": "Zero",
                "ProdShiftRightIntArith": "on", "ProdBitPerFloat": "32", "ProdBitPerDouble": "64",
                "ProdBitPerPointer": "64", "ProdBitPerSizeT": "64", "ProdBitPerPtrDiffT": "64",
                "ProdEndianess": "LittleEndian", "ProdLargestAtomicInteger": "Char", "ProdLargestAtomicFloat": "None"}


def hw_profile(hw):
    """('host' | 'asic', HardwareCC properties, settings the check script verifies). The engine's word sizes must
    be exactly those of the named Simulink device; anything else is refused."""
    import fxcore as fc
    hw = hw or {}
    if not isinstance(hw, dict):
        raise ExportError("BAD_PARAM", "hw", "hw must be an object")
    # a previous version: the engine refuses a parameter that overflows its type, as Simulink's default diagnostic does
    # (ParameterOverflowMsg 'error'); 'warning' / 'none' saturate instead. The saved form of that diagnostic in a
    # model's configuration is not recorded, so only the default is exported -- and the check script verifies it
    pom = str(hw.get("ParameterOverflowMsg", "error")).strip()   # the request's own setting (harnesses may change the process default)
    if pom != "error":
        raise ExportError("UNSUPPORTED_EXPORT", "hw.ParameterOverflowMsg", "ParameterOverflowMsg %r: only Simulink's "
                          "default 'error' is exported (the model's diagnostic setting is not written yet)" % pom)
    words = fc._hw_words({"hw": hw})
    dev = str(hw.get("ProdHWDeviceType", ""))
    if fc._asic(hw):
        for k, v in hw.items():
            want = _ASIC_HW.get(k, _ASIC_SIZES.get(k))
            if k.startswith("Prod") and k not in ("ProdHWDeviceType", "ProdEqTarget") and str(want) != str(v):
                raise ExportError("UNSUPPORTED_EXPORT", "hw." + k, "%s=%r with ASIC/FPGA (only the device defaults are "
                                  "exported)" % (k, v))
        # a saved .slx spells the device 'ASIC/FPGA->ASIC/FPGA' (vendor->type); the same device as 'ASIC/FPGA'
        if dev not in ("ASIC/FPGA", "ASIC/FPGA->ASIC/FPGA"):
            raise ExportError("UNSUPPORTED_EXPORT", "hw.ProdHWDeviceType", "device %r: only ASIC/FPGA or %s" % (dev, HOST_DEVICE))
        return "asic", dict(_ASIC_HW), {"ProdHWDeviceType": "ASIC/FPGA", "ParameterOverflowMsg": "error",
                                        "ParameterDowncastMsg": "error"}
    if dev not in ("", HOST_DEVICE):
        raise ExportError("UNSUPPORTED_EXPORT", "hw.ProdHWDeviceType", "device %r: only ASIC/FPGA or %s (the host "
                          "default)" % (dev, HOST_DEVICE))
    for k, v in hw.items():
        if not k.startswith("Prod") or k in ("ProdHWDeviceType", "ProdEqTarget"):
            continue
        if k not in _HOST_VALUES or str(v) != _HOST_VALUES[k]:
            raise ExportError("UNSUPPORTED_EXPORT", "hw." + k, "%s=%r is not the %s value (%s)"
                              % (k, v, HOST_DEVICE, _HOST_VALUES.get(k, "not a setting of that device")))
    if words != [8, 16, 32]:
        raise ExportError("UNSUPPORTED_EXPORT", "hw", "native word sizes %s differ from the %s device (8, 16, 32 "
                          "with long long off)" % (words, HOST_DEVICE))
    return "host", {"ProdHWDeviceType": HOST_DEVICE}, {
        "ProdHWDeviceType": HOST_DEVICE, "ProdBitPerChar": "8", "ProdBitPerShort": "16", "ProdBitPerInt": "32",
        "ProdBitPerLong": "32", "ProdLongLongMode": "off", "ParameterOverflowMsg": "error",
        "ParameterDowncastMsg": "error"}


# ---------------------------------------------------------------- parameter planning
_BAD_NAME = re.compile(r"[\x00-\x1f\x7f]")


def _check_name(name, where):
    if not isinstance(name, str) or not name or _BAD_NAME.search(name) or name != name.strip():
        raise ExportError("UNSUPPORTED_EXPORT", where, "block name %r cannot be a Simulink block name (empty, "
                          "control characters or surrounding blanks)" % (name,))
    if any(not seg or seg != seg.strip() for seg in name.split("/")):
        raise ExportError("UNSUPPORTED_EXPORT", where, "block name %r: an empty or blank-padded subsystem level" % name)


def _type_str(t):
    if t == G.BOOL:
        return "boolean"
    return "fixdt(%d,%d,%d)" % t


def _in_type(rec, where):
    """An input record's type: G.BOOL for {"type": "boolean", "ints": [0 / 1 ...]} (fxgraph's boolean Inport form),
    else (s, wl, fl)"""
    if isinstance(rec, dict) and rec.get("type") == "boolean":
        return G.BOOL
    try:
        return (int(rec["s"]), int(rec["wl"]), int(rec["fl"]))
    except (KeyError, TypeError, ValueError):
        raise ExportError("BAD_INPUT", where, "an input is {s, wl, fl, ints} or {\"type\": \"boolean\", \"ints\": "
                          "[0, 1, ...]}")


def block_params(name, b, et, nconnected=None):
    """(Simulink parameters to write {name: text}, dropped parameter names). Every engine-semantic parameter is
    written: the design's value, else the engine default."""
    where = "design.blocks." + name
    tmpl = TEMPLATES[et]
    p = {}
    for k, v in BS.DEFAULTS.get(et, {}).items():
        p[k] = str(v)
    # a parameter the design leaves out means the Simulink library default (as the engine reads it,
    # fxgraph_lib.check_user_design), not the engine's own table default -- write that value
    p.update(BS.JSON_LIBRARY_DEFAULTS.get(et, {}))
    p.update(HIDDEN_DEFAULTS.get(et, {}))
    if et in ("PreLookup", "Interpolation_n-D"):
        # a block saved without IntermediateResultsDataTypeStr / InternalRulePriority loads the
        # built-in 'Inherit: Same as output' / 'Precision', not the library block's internal rule / Speed that the
        # engine uses (an unsigned falling table gave 12 instead of 11, recordings) -- every value the lowering assumes
        # (engine/fxprelookup.py) is written, so the model is the design
        import fxprelookup
        for k_, v_ in (fxprelookup.PL_DEFAULTS if et == "PreLookup" else fxprelookup.IP_DEFAULTS).items():
            if v_ is not None:
                p[k_] = v_
    if et == "HDLCounter" and "CountFrom" not in b:
        # with CountFromType 'Initial value' the counter's from-value follows CountInit (the engine
        # reads it so, fxhdl); the library default 0 written next to CountInit 10 counting down to -20 was a value the
        # read-back refuses (initoutofrange) and that the design never had -- written as CountInit
        p["CountFrom"] = str(b.get("CountInit", p.get("CountInit", "0")))
    if et == "Prelookup" and str(b.get("FractionDataTypeStr", "Inherit")).strip().startswith("Inherit"):
        # the library's fraction rule follows the hardware (host ufix32_En31, ASIC / FPGA
        # ufix<w>_En<w-1>); the engine resolves it the same way (fxgraph_lib._prelookup_check) -- written as the rule,
        # not as the host's type
        p["FractionDataTypeStr"] = "Inherit: Inherit via internal rule"
    for (t_, k_), why in OMITTED_REFUSED.items():
        if t_ == et and k_ not in b:
            raise ExportError("UNSUPPORTED_EXPORT", where + "." + k_, "%s left out: %s; write %s in the design"
                              % (k_, why, k_))
    dropped = []
    for k, v in b.items():
        if k == "type":
            continue
        if k.startswith("_"):
            raise ExportError("UNSUPPORTED_EXPORT", where + "." + k, "internal parameter")
        if k in _DROPPED and not (k == "Priority" and et in ("DataStoreRead", "DataStoreWrite")):
            # (a Data Store's Priority decides its read / write order, recordings: written)
            dropped.append(k)
            continue
        p[k] = G._param_text(v, where + "." + k)
    for k, v in tmpl[2].items():
        if p.get(k, v) != v:
            raise ExportError("UNSUPPORTED_EXPORT", where + "." + k, "%s=%r (the export writes %r)" % (k, p[k], v))
        p[k] = v
    # a Delay / Shift Arithmetic whose length, initial condition or shift comes from a port
    # needs the hidden InputPortMap Simulink saves (recorded samples: u0,p1 / u0,p4 / u0,p1) -- without it
    # the model does not compile (STD_EXCEPTION invalid vector subscript)
    if et == "Delay":
        ls_ = str(p.get("DelayLengthSource", "Dialog")).strip() == "Input port"
        is_ = str(p.get("InitialConditionSource", "Dialog")).strip() == "Input port"
        if ls_ and is_:
            raise ExportError("UNSUPPORTED_EXPORT", where + ".InitialConditionSource", "a Delay with both its length "
                              "and its initial condition from ports: the saved port map was not recorded")
        if ls_ or is_:
            p["InputPortMap"] = "u0,p1" if ls_ else "u0,p4"
    if et == "ShiftArithmetic" and str(p.get("BitShiftNumberSource", "Dialog")).strip() == "Input port":
        p["InputPortMap"] = "u0,p1"
    if et == "DiscreteIntegrator" and str(p.get("ShowStatePort", "off")).strip() == "on":
        # the state port (a previous version) answered INTERNAL here -- its saved port layout was not
        # recorded; refused until a saved model shows it
        raise ExportError("UNSUPPORTED_EXPORT", where + ".ShowStatePort", "a Discrete-Time Integrator with its state "
                          "port shown: the saved model's port layout was not recorded; record one to export it")
    # sample time: explicit everywhere Simulink's omitted value is not "inherited"
    if et == "HDLFIFO" and (p.get("push_msg"), p.get("pop_msg")) not in (("Ignore", "Ignore"), ("Warning", "Warning")):
        raise ExportError("UNSUPPORTED_EXPORT", where + ".push_msg", "an HDL FIFO is exported with push_msg and "
                          "pop_msg both 'Ignore' or both 'Warning' (Error stops the simulation)")
    if et == "HDLCounter" and str(p.get("CountSampTime", "1")).strip() not in ("1", "2", "3", "4"):
        raise ExportError("UNSUPPORTED_EXPORT", where + ".CountSampTime", "HDL Counter sample time %r (1, and 2 / 3 / 4 "
                          "in a design of counters and Outports only)" % p.get("CountSampTime"))
    if et in _COUNTERS:
        ts = str(p.pop("SampleTime", p.get("tsamp", "1"))).strip()
        p.pop("tsamp", None)
        if ts in ("1", "-1"):
            p["tsamp"] = "1"
        elif re.match(r"^[1-9][0-9]*$", ts):
            # a whole period of its own (a Counter
            # Limited at tsamp 4 in a multirate model compiles to period 4 and counts once per hit): written as given;
            # export_package admits it only in a multirate design whose engine period for the block is this one
            p["tsamp"] = ts
        else:
            raise ExportError("UNSUPPORTED_EXPORT", where + ".tsamp", "counter sample time %r (a whole period only: "
                              "1 / -1, or a whole period of its own in a multirate design)" % ts)
    elif et in _SAMPLE_TIME_PARAM and "SampleTime" not in p and _SAMPLE_TIME_PARAM[et] is not None:
        p["SampleTime"] = _SAMPLE_TIME_PARAM[et]
    if et == "Delay" and (p.get("ShowEnablePort", "off") == "on" or p.get("ExternalReset", "None") != "None"):
        # without its InputPortMap a Delay with an enable / reset port does not compile
        # (SL_SERVICES:utils:STD_EXCEPTION, 'invalid vector subscript'); Simulink saves u0, then e6 for the enable
        # port and r5 for the reset port, in port order (every recorded combination)
        p["InputPortMap"] = ",".join(["u0"] + (["e6"] if p.get("ShowEnablePort") == "on" else []) +
                                     (["r5"] if p.get("ExternalReset", "None") != "None" else []))
    if et == "DiscreteIntegrator" and "SampleTime" not in b:
        # the engine's table default "1" is not used as a period (an absent SampleTime is inherited on every path)
        p["SampleTime"] = "-1"
    if et in ("BusSelector", "BusAssignment") and isinstance(p.get("OutputSignals" if et == "BusSelector" else
                                                                   "AssignedSignals"), str):
        # Simulink stores the element list without spaces (' q, p ' is saved 'q,p'); written so
        k_ = "OutputSignals" if et == "BusSelector" else "AssignedSignals"
        p[k_] = ",".join(x.strip() for x in p[k_].split(","))
    if et == "LookupND":
        for d in range(1, int(str(p.get("NumberOfTableDimensions", "3")).strip() or 3) + 1):
            if "BreakpointsForDimension%d" % d not in p:
                raise ExportError("UNSUPPORTED_EXPORT", where + ".BreakpointsForDimension%d" % d,
                                  "explicit breakpoints are required")
    if et in ("Lookup1D", "Lookup2D"):
        dims = (1,) if et == "Lookup1D" else (1, 2)
        even = p.get("BreakpointsSpecification", "Explicit values") == "Even spacing"
        for d in dims:
            k = "BreakpointsForDimension%d" % d
            if not even and k not in p:
                raise ExportError("UNSUPPORTED_EXPORT", where + "." + k, "explicit breakpoints are required")
        if et == "Lookup1D":
            for k in ("BreakpointsForDimension2DataTypeStr", "BreakpointsForDimension2FirstPoint",
                      "BreakpointsForDimension2Spacing", "UseLastTableValue"):
                if k in p and k not in b:
                    del p[k]
    if et in ("FIRDecimation", "FIRInterpolation"):
        for pre in _DSP_TYPED:
            ts = p.get(pre + "DataTypeStr")
            t = G.fc._parse_fixdt(ts) if ts else None
            if t is not None:
                p[pre + "Mode"] = "Binary point scaling"
                p[pre + "WordLength"] = str(t[1])
                p[pre + "FracLength"] = str(t[2])
            elif ts and str(ts).strip().startswith("Inherit: ") and pre + "Mode" not in b:
                # 'Inherit: X' written without its *Mode sets the mode X in Simulink and in the engine
                #; the export wrote the string only, so the saved model kept the mode's library default
                # (EXPORT_ROUNDTRIP_MISMATCH, or PARAM_MISMATCH for an accumulator 'Same as product output')
                p[pre + "Mode"] = str(ts).strip()[len("Inherit: "):]
    for k, v in NOT_A_PARAMETER.get(et, {}).items():
        if p.pop(k, v) != v:
            raise ExportError("UNSUPPORTED_EXPORT", where + "." + k, "%s=%r: the Simulink block has no such parameter "
                              "(only the engine's %r can be exported)" % (k, b.get(k), v))
    if et == "MinMax" and "Inputs" not in b and nconnected == 1:
        # the engine reads an absent Inputs with one connected input as a one-input MinMax (the library block's own
        # default, engine/fxvector._one_input); the table default "2" would add an unconnected port. (A Sum with
        # Inputs left out is the library '|++' since recordings: two ports -- one line is refused by
        # the engine, never exported; recorded)
        p["Inputs"] = "1"
    if et == "Sum" and "|" in p.get("Inputs", ""):
        p["Inputs"] = p["Inputs"].replace("|", "")
    if et == "Mux" and nconnected:
        # the engine sizes a Mux from its lines; a library Mux has Inputs ports (2 when left out) and a
        # third line has nowhere to go -- the model would not be the request's
        txt = str(p.get("Inputs", "2")).strip()
        try:
            k = len(txt.strip("[]").replace(",", " ").split()) if txt.startswith("[") else int(float(txt))
        except ValueError:
            k = None
        if k is not None and k != nconnected:
            raise ExportError("UNSUPPORTED_EXPORT", where + ".Inputs", "Mux with %d connected inputs and Inputs %s%s: "
                              "write Inputs as the number of inputs" % (nconnected, txt, "" if "Inputs" in b else
                                                                         " (left out: the library's 2)"))
    return p, sorted(dropped)


# ---------------------------------------------------------------- the model tree
class _System:
    def __init__(self, path):
        self.path = path            # tuple of subsystem names below DUT ("" = the root model)
        self.blocks = []            # (sid, xml text)
        self.lines = {}             # (src sid, src port) -> [(dst sid, dst port)]
        self.names = set()
        self.sid_of = {}            # local block name -> sid
        self.children = {}          # subsystem name -> _System
        self.ports_in = {}          # signal key -> inport number (inside this subsystem)
        self.ports_out = {}         # signal key -> outport number
        self.inport_sid = {}        # number -> sid of the Inport block inside
        self.outport_sid = {}
        self.signal_names = {}   # (design source, port) -> line name (bus element names)
        self.line_name = {}      # (src sid, port) -> line name in this system


class _Builder:
    def __init__(self, seed):
        self.sid = 0
        self.seed = seed
        self.zorder = 0

    def next_sid(self):
        self.sid += 1
        return self.sid

    def pos(self, k):
        x, y = 60 + 140 * (k % 8), 40 + 90 * (k // 8)
        return "[%d, %d, %d, %d]" % (x, y, x + 50, y + 40)


def _p(name, value):
    return '    <P Name=%s>%s</P>\n' % (quoteattr(name), escape(str(value)))


def _block_xml(bt, name, sid, params, zorder, pos, nin=None, nout=None, lib=None, inner=None, extra="", enable=False,
               mask=""):
    s = '  <Block BlockType=%s Name=%s SID="%d">\n' % (quoteattr(bt), quoteattr(name), sid)
    if nin is not None or nout is not None or enable:
        attrs = []
        if nin:
            attrs.append('in="%d"' % nin)
        if nout:
            attrs.append('out="%d"' % nout)
        if enable:
            attrs.append({"ifaction": 'ifaction="1"', "trigger": 'trigger="1"', "Reset": 'reset="1"'}.get(
                enable, 'enable="1"'))
        if attrs:
            s += '    <PortCounts %s/>\n' % " ".join(attrs)
    s += _p("Position", pos) + _p("ZOrder", zorder)
    if lib is not None:
        if len(lib) > 3:   # a library link saved through a forwarding library (Wrap To Zero: sl_discontinuities)
            s += _p("LibrarySourceBlock", lib[3])
        s += _p("LibraryVersion", lib[2]) + _p("SourceBlock", lib[0]) + _p("SourceType", lib[1])
        s += "    <InstanceData>\n"
        for k, v in params:
            s += "  " + _p(k, v)
        s += "    </InstanceData>\n"
    else:
        for k, v in params:
            s += _p(k, v)
    s += mask   # a subsystem's Mask element comes before its System reference (the saved For Each models)
    if inner is not None:
        s += '    <System Ref="system_%d"/>\n' % inner
    s += extra + "  </Block>\n"
    return s


_BT = re.compile(r'BlockType="([^"]+)"')
_PORT = re.compile(r'<P Name="Port">(\d+)</P>')


def _block_order(item):
    """Simulink loads a system's Inport (and Outport) blocks in file order and expects that order to follow their port
    numbers (Port 2 written before Port 1 dropped a line and failed with BadInportNum). Inports first
    by port, then the other blocks by SID, then Outports by port."""
    sid, x = item
    bt = _BT.search(x).group(1)
    m = _PORT.search(x)
    port = int(m.group(1)) if m else 1
    if bt == "Inport":
        return (0, port, sid)
    if bt == "Outport":
        return (2, port, sid)
    return (1, 0, sid)


def _system_xml(sys, root_extra=""):
    out = ['<?xml version="1.0" encoding="utf-8"?>\n<System>\n', root_extra]
    for _, x in sorted(sys.blocks, key=_block_order):
        out.append(x)
    z = 0
    for (ssid, sport), dsts in sorted(sys.lines.items()):
        z += 1
        nm_ = getattr(sys, "line_name", {}).get((ssid, sport))
        out.append("  <Line>\n" + (_p("Name", nm_) if nm_ else "") + _p("ZOrder", z) + _p("Src", "%d#out:%d" % (ssid, sport)))
        dsts = sorted(dsts, key=lambda d: (d[0], str(d[1])))

        def ep(d):
            return "%d#%s" % (d[0], d[1]) if d[1] in ("enable", "ifaction", "trigger", "Reset") else "%d#in:%d" % d
        if len(dsts) == 1:
            out.append(_p("Dst", ep(dsts[0])))
        else:
            for d in dsts:
                out.append("    <Branch>\n  " + _p("Dst", ep(d)) + "    </Branch>\n")
        out.append("  </Line>\n")
    out.append("</System>\n")
    return "".join(out)


def _unique(base, taken):
    n, k = base, 1
    while n in taken:
        k += 1
        n = "%s_%d" % (base, k)
    taken.add(n)
    return n


# block types with more than one output port (some only with an option on): their Outports keep 'Inherit: auto'
MULTI_OUTPUT = ("ComplexToRealImag", "ComplexToMagnitudeAngle", "HDLCounter", "HDLNCO", "HDLRAM", "HDLFIRFilter", "HDLFIRDecimator", "HDLFIFO", "Demux",
                "BusSelector", "MATLABFunction", "ForEach")


def build_model(design, hw, model_name, inports, outports, solver, out_periods, dut="DUT", out_types=None):
    """Build the .slx bytes. inports: [(name, period text, type (s,wl,fl), variable)], outports: [name] in port
    order; solver: {FixedStep, StopTime, SolverMode or None}. Returns (bytes, block_param_checks, plan info)."""
    blocks, lines = design["blocks"], design.get("lines", [])
    seed = hashlib.sha256(_canon({"design": design, "hw": hw, "name": model_name, "solver": solver}).encode()).hexdigest()
    B = _Builder(seed)
    root = _System(None)
    top = _System(())
    systems = {(): top}
    checks = []          # (block path below the model, parameter, value) for the check script
    dropped = {}
    written = {}   # design block -> (model path, engine type, {Simulink parameter: text written})
    unobserved = set()
    nin_of = {}
    lib_sids = []

    def sys_for(path):
        if path in systems:
            return systems[path]
        parent = sys_for(path[:-1])
        s = _System(path)
        systems[path] = s
        parent.children[path[-1]] = s
        if path[-1] in parent.names:
            raise ExportError("UNSUPPORTED_EXPORT", "design.blocks." + "/".join(path), "a block and a subsystem "
                              "level share the name %r" % path[-1])
        parent.names.add(path[-1])
        parent.sid_of[path[-1]] = B.next_sid()
        return s

    # 1. design blocks
    order = list(blocks)
    for idx, name in enumerate(order):
        b = blocks[name]
        _check_name(name, "design.blocks." + name)
        et = b.get("type")
        if et not in TEMPLATES and et != "MATLABFunction" and et not in ("Inport", "Outport"):
            raise ExportError("UNSUPPORTED_EXPORT", "design.blocks.%s.type" % name, "block type %r has no export "
                              "template" % (et,))
        segs = name.split("/")
        if et in ("Inport", "Outport") and len(segs) > 1:
            raise ExportError("UNSUPPORTED_EXPORT", "design.blocks." + name, "%s inside a subsystem level; design "
                              "ports must be at the top level" % et)
        s = sys_for(tuple(segs[:-1]))
        local = segs[-1]
        if local in s.names:
            raise ExportError("UNSUPPORTED_EXPORT", "design.blocks." + name, "name clash with a subsystem level")
        s.names.add(local)
        s.sid_of[local] = B.next_sid()
    # For Each levels: a vector parameter inside is the subsystem's one mask parameter K,
    # iterated element by element (IterateSubsysMaskParameter) -- recorded for a Gain's Gain, the other partitioned
    # parameters (Unit Delay initial condition, Saturation limits, Constant value) by the probe recordings
    fe_levels = {tuple(n.split("/")[:-1]) for n, b in blocks.items() if b.get("type") == "ForEach"}
    fe_mask = {}
    for n, b in blocks.items():
        lvl = tuple(n.split("/")[:-1])
        if lvl not in fe_levels:
            continue
        for k, v in b.items():
            if k != "type" and isinstance(v, str) and re.match(r"^\s*\[.*[\s,].*\]\s*$", v):
                import fxvector
                if k not in fxvector.VECPARAM.get(b.get("type"), ()) or lvl in fe_mask:
                    raise ExportError("UNSUPPORTED_EXPORT", "design.blocks.%s.%s" % (n, k), ('a vector parameter inside a For Each is exported as '
                        "the subsystem's one iterated mask parameter (one "
                        'parameter the engine partitions: %s); recorded for a '
                        'Gain, the other parameters also matched recorded output') % ", ".join(
                                          "%s.%s" % (t, x) for t, xs in sorted(fxvector.VECPARAM.items()) for x in xs))
                fe_mask[lvl] = (n, k, v.strip())
    # port numbers of the design ports (top level of DUT)
    in_no = {n: k + 1 for k, (n, _, _, _) in enumerate(inports)}
    out_no = {n: k + 1 for k, n in enumerate(outports)}
    # number of input ports of each design block, from the lines
    for ln in lines:
        nin_of[ln["dst"]] = max(nin_of.get(ln["dst"], 0), int(ln.get("dport", 1)))
    nout_of = {}
    for ln in lines:
        nout_of[ln["src"]] = max(nout_of.get(ln["src"], 0), int(ln.get("sport", 1)))

    # 2. lines, routed through the subsystem levels
    def endpoint_sys(name):
        segs = name.split("/")
        return tuple(segs[:-1]), segs[-1]

    def out_port_of(s, key):
        """Outport of subsystem s carrying signal key (created on first use)."""
        if key not in s.ports_out:
            k = len(s.ports_out) + 1
            s.ports_out[key] = k
            s.outport_sid[k] = B.next_sid()
        return s.ports_out[key]

    def in_port_of(s, key):
        if key not in s.ports_in:
            k = len(s.ports_in) + 1
            s.ports_in[key] = k
            s.inport_sid[k] = B.next_sid()
        return s.ports_in[key]

    # enabled subsystems: an EnabledOutport is its subsystem's Outport k (numbered in design order)
    enabled = {}
    for name in order:
        t = blocks[name]["type"]
        if t in ("EnablePort", "StateControl", "EnabledOutport", "ActionPort", "TriggerPort", "ResetPort"):
            path = tuple(name.split("/")[:-1])
            if not path:
                raise ExportError("UNSUPPORTED_EXPORT", "design.blocks." + name, "%s at the top level" % t)
            enabled.setdefault(path, {"outs": {}})
            if t == "ActionPort":
                enabled[path]["action"] = True   # the subsystem's port is an ifaction port
            if t in ("TriggerPort", "ResetPort"):   # #trigger / #Reset ports
                enabled[path]["port"] = "trigger" if t == "TriggerPort" else "Reset"
            if t == "EnabledOutport":
                sS = systems[path]
                k = len(enabled[path]["outs"]) + 1
                enabled[path]["outs"][name] = k
                sS.ports_out[("enout", name)] = k
                sS.outport_sid[k] = sS.sid_of[name.split("/")[-1]]
    for ln in lines:
        src, dst = ln["src"], ln["dst"]
        sport, dport = int(ln.get("sport", 1)), int(ln.get("dport", 1))
        if blocks.get(dst, {}).get("type") == "BusCreator" and blocks[dst].get("ElementNames"):
            nms = [x.strip() for x in str(blocks[dst]["ElementNames"]).split(",")]
            if dport <= len(nms):
                sname = systems[endpoint_sys(dst)[0]].signal_names
                key0 = (src, sport)
                if sname.get(key0, nms[dport - 1]) != nms[dport - 1]:
                    raise ExportError("UNSUPPORTED_EXPORT", "design.lines", "signal %s feeds two Bus Creators under "
                                      "different element names" % src)
                sname[key0] = nms[dport - 1]
        if src not in blocks or dst not in blocks:
            raise ExportError("BAD_CONNECTION", "design.lines", "unknown block in line %s -> %s" % (src, dst))
        sp, sl = endpoint_sys(src)
        dp, dl = endpoint_sys(dst)
        if blocks[src]["type"] == "EnabledOutport":
            # leaves through the subsystem's output port: start one level up, at the subsystem block
            sport = enabled[sp]["outs"][src]
            sp, sl = sp[:-1], sp[-1]
        if blocks[dst]["type"] in ("EnablePort", "ActionPort", "TriggerPort", "ResetPort"):
            # ends at the subsystem block's enable (or action, recordings; trigger / reset) port, one level up
            dport = {"EnablePort": "enable", "ActionPort": "ifaction", "TriggerPort": "trigger",
                     "ResetPort": "Reset"}[blocks[dst]["type"]]
            dp, dl = dp[:-1], dp[-1]
        key = (src, sport)
        common = 0
        while common < min(len(sp), len(dp)) and sp[common] == dp[common]:
            common += 1
        # climb from the source up to the common level
        cur_sid, cur_port = systems[sp].sid_of[sl], sport
        for lvl in range(len(sp), common, -1):
            s = systems[sp[:lvl]]
            k = out_port_of(s, key)
            s.lines.setdefault((cur_sid, cur_port), [])
            pair = (s.outport_sid[k], 1)
            if pair not in s.lines[(cur_sid, cur_port)]:
                s.lines[(cur_sid, cur_port)].append(pair)
            parent = systems[sp[:lvl - 1]]
            cur_sid, cur_port = parent.sid_of[sp[lvl - 1]], k
        # descend into the destination's level
        for lvl in range(common + 1, len(dp) + 1):
            parent = systems[dp[:lvl - 1]]
            s = systems[dp[:lvl]]
            k = in_port_of(s, key)
            pair = (parent.sid_of[dp[lvl - 1]], k)
            lst = parent.lines.setdefault((cur_sid, cur_port), [])
            if pair not in lst:
                lst.append(pair)
            cur_sid, cur_port = s.inport_sid[k], 1
        final = systems[dp]
        if blocks[dst]["type"] == "BusCreator" and (src, sport) in final.signal_names:
            final.line_name[(cur_sid, cur_port)] = final.signal_names[(src, sport)]
        lst = final.lines.setdefault((cur_sid, cur_port), [])
        pair = (final.sid_of[dl], dport)
        if pair in lst:
            raise ExportError("BAD_CONNECTION", "design.lines", "duplicate line %s -> %s:%s" % (src, dst, dport))
        lst.append(pair)

    # 3. block XML for the design blocks
    k_pos = 0
    mlfns = []
    for name in order:
        b = blocks[name]
        et = b["type"]
        path, local = endpoint_sys(name)
        s = systems[path]
        sid = s.sid_of[local]
        k_pos += 1
        full = "/".join((dut,) + path + (local,))
        if et == "Inport":
            t = next(t for n, _, t, _ in inports if n == name)
            prm = [("Port", str(in_no[name])), ("OutDataTypeStr", _type_str(t))] if in_no[name] != 1 else \
                [("OutDataTypeStr", _type_str(t))]
            # no SampleTime: the default (-1, inherited) takes the rate of the From Workspace source outside the DUT
            for kk, vv in prm:
                if kk != "Port":
                    checks.append((full, kk, vv))
            extra = [k for k in b if k not in ("type", "SampleTime", "OutDataTypeStr", "PortDimensions")]
            w = _width(b)
            if w > 1 or "PortDimensions" in b:
                # an explicit PortDimensions 1 is written as given (main, export roundtrip of recordings
                # G: the library default is -1, inherited, so leaving it out is not the same model)
                prm.append(("PortDimensions", str(w)))
                checks.append((full, "PortDimensions", str(w)))
            if extra:
                dropped[name] = sorted(extra)
            s.blocks.append((sid, _block_xml("Inport", local, sid, prm, k_pos, B.pos(k_pos))))
            continue
        if et == "Outport":
            prm = [("Port", str(out_no[name]))] if out_no[name] != 1 else []
            ot = (out_types or {}).get(name)
            own = str(b.get("OutDataTypeStr", "")).strip()
            if own and own != "Inherit: auto":
                # the design's own Outport type is part of the request (it back-propagates into an
                # inherited source); it was dropped, so every such design failed the roundtrip (34 / 34)
                ot = None
                prm.append(("OutDataTypeStr", own))
                checks.append((full, "OutDataTypeStr", own))
            drv = [ln["src"] for ln in lines if ln["dst"] == name]
            drv_t = blocks.get(drv[0], {}).get("type") if drv else None
            if drv_t in MULTI_OUTPUT or (drv and nout_of.get(drv[0], 1) > 1):
                # a multi-output block reads back as one output of a Demux, where a typed Outport is refused
                #: left at 'Inherit: auto' as before
                ot = None
            if ot and drv and not _on_cycle(drv[0], lines):
                # recordings (ii): a typed DUT Outport is a constraint Simulink checks against the driver's own rule
                # (a back-propagated Constant into an internal-rule Sum: InvDisagreeInternalRuleDType, though the
                # untyped design is answered like Simulink's); only a driver on a feedback loop needs the type
                # (an all-inherited loop compiles as double behind an untyped DUT Outport)
                ot = None
            if ot and (ot == "boolean" or ot.startswith("fixdt(")):
                # a feedback loop whose types are all inherited, leaving the DUT through an
                # Outport at 'Inherit: auto', compiled as double in Simulink (the same blocks at the top level, as
                # recorded: sfix16_En8); written with its compiled type, the Outport gives Simulink the top-level
                # typing (the native recordings). The reader carries it back as a constraint (a previous version:
                # design.port_types, checked after compiling), which the roundtrip then verifies.
                prm.append(("OutDataTypeStr", ot))
            extra = [k for k in b if k not in ("type",) and not (k == "OutDataTypeStr" and own and own != "Inherit: auto")]
            if extra:
                dropped[name] = sorted(extra)
            s.blocks.append((sid, _block_xml("Outport", local, sid, prm, k_pos, B.pos(k_pos), nin=1)))
            continue
        if et == "MATLABFunction":
            # R2026b stores a MATLAB Function as a SubSystem with SFBlockType 'MATLAB Function' whose system holds the Stateflow
            # S-Function, and a Stateflow chart with the script (mlfn_parts below)
            xml, ml_ = _mlfn_block(name, local, b, sid, k_pos, B.pos(k_pos), full, len(mlfns) + 1)
            mlfns.append(dict(ml_, parent=path))
            s.blocks.append((sid, xml))
            continue
        bt, lib, _fixed, observed = TEMPLATES[et]
        if not observed:
            unobserved.add(et)
        if et == "ForEach":
            prm = [("DisableCoverage", "on")]
            if path in fe_mask:
                prm += [("IterateSubsysMaskParameter", "on"), ("SubsysMaskParameterIterationDimension", "2")]
            extra = [k for k in b if k != "type"]
            if extra:
                raise ExportError("UNSUPPORTED_EXPORT", "design.blocks.%s.%s" % (name, extra[0]), "For Each "
                                  "parameters are written as recorded (element-wise partition); %s is not" % extra[0])
            for kk, vv in prm:
                checks.append((full, kk, vv))
            xml = _block_xml("ForEach", local, sid, prm, k_pos, B.pos(k_pos))
            # an empty PortCounts element, as R2026b saves the For Each block (recordings.slx)
            xml = xml.replace('">\n', '">\n    <PortCounts/>\n', 1)
            s.blocks.append((sid, xml))
            continue
        if et in ("EnablePort", "StateControl", "EnabledOutport", "ActionPort", "TriggerPort", "ResetPort"):
            # a TriggerPort written with in / out port counts crashed MATLAB at compile (access
            # violation); the saved models give it an empty <PortCounts/>, the ResetPort none
            p, dr = block_params(name, b, et, 0)
            prm = sorted(p.items())
            if et == "EnabledOutport":
                k = enabled[path]["outs"][name]
                prm = ([("Port", str(k))] if k != 1 else []) + prm
            for kk, vv in prm:
                if kk != "Port":
                    checks.append((full, kk, vv))
            xml = _block_xml(bt, local, sid, prm, k_pos, B.pos(k_pos), nin=1 if et == "EnabledOutport" else None)
            if et == "TriggerPort":
                xml = xml.replace('">\n', '">\n    <PortCounts/>\n', 1)
            s.blocks.append((sid, xml))
            continue
        sysram = et == "HDLRAM" and str(b.get("AddressBits", "")).strip() == "Inherit: from address"
        if sysram and not _needs_sysram(b):
            raise ExportError("UNSUPPORTED_EXPORT", "design.blocks.%s.AddressBits" % name, "an inherited address width "
                              "whose width the engine could not resolve (a vector design or an untyped address); "
                              "give AddressBits")
        if et == "HDLRAM":
            lib = (_SYSRAM_LIB if sysram else _RAM_LIB)[str(b.get("RAMType", "Simple dual port"))]
            if not sysram and str(b.get("RAMType")) == "Dual port" and \
                    str(b.get("WriteOutputValue", "New data")) == "Old data":
                raise ExportError("UNSUPPORTED_EXPORT", "design.blocks.%s.WriteOutputValue" % name, "the masked Dual "
                                  "Port RAM has no WriteOutputValue (its write port gives new data); for Old data use "
                                  "AddressBits 'Inherit: from address' (the System RAM)")
            if not sysram and _norm(b.get("RAMInitialValue", "0")) != _norm("0"):
                raise ExportError("UNSUPPORTED_EXPORT", "design.blocks.%s.RAMInitialValue" % name, "the masked RAM "
                                  "starts at 0 (a non-zero initial value needs the System RAM, not exported)")
        p, dr = block_params(name, b, et, nin_of.get(name, 0))
        if path in fe_mask and fe_mask[path][0] == name:
            p[fe_mask[path][1]] = "K"   # the subsystem's iterated mask parameter (written on the SubSystem below)
        if et == "BusCreator":
            p.pop("ElementNames", None)   # not a block parameter: the input lines carry the names
        if et == "HDLRAM" and sysram:
            # the System RAM as R2026b saves it: its width is the
            # address signal's
            p = dict(_SYSRAM_FIXED, RAMType=str(b.get("RAMType", "Simple dual port")),
                     WriteOutputValue=str(b.get("WriteOutputValue", "New data")),
                     RAMInitialValue=str(b.get("RAMInitialValue", "0")))
        elif et == "HDLRAM":
            p = {"ram_size": str(p["AddressBits"])}
            if b.get("RAMType") == "Single port":
                p["dout_type"] = str(b.get("WriteOutputValue", "New data"))
        if dr:
            dropped[name] = dr
        prm = sorted(p.items())
        for kk, vv in prm:
            checks.append((full, kk, vv))
        if not (et == "HDLRAM" or (path in fe_mask and fe_mask[path][0] == name)):
            written[name] = (full, et, dict(p))
        nin = nin_of.get(name, 0)
        if et == "DataTypeDuplicate":
            nout = 0
        elif et == "HDLRAM":
            nout = 2 if b.get("RAMType") == "Dual port" else 1
        elif et == "ComplexToRealImag":
            import fxhdl
            nout = len(fxhdl._c2ri_parts(b))   # 'Real' / 'Imag' have one output port
        elif et == "ComplexToMagnitudeAngle":
            nout = 2   # 'Magnitude and angle' (the recorded output)
        elif et == "BusSelector":
            nout = 1 if str(b.get("OutputAsBus", "off")) == "on" else \
                len([x for x in str(b.get("OutputSignals", "")).split(",") if x.strip()])
        elif et == "HDLFIFO":
            import fxhdlfifo
            nout = len(fxhdlfifo.outputs(fxhdlfifo.params(b, "design.blocks." + name)))
        elif et == "HDLCounter":
            # the count_hit line (source port 2) was dropped on load because the block was written
            # with one output port
            nout = 2 if str(p.get("CountHitOutputPort", "off")) == "on" else 1
        elif et in ("HDLFIRFilter", "HDLFIRDecimator"):
            nout = 3 if (et == "HDLFIRFilter" and str(p.get("FilterStructure")) == "Partly serial systolic") else 2
        elif et == "HDLNCO":
            wf = str(p.get("Waveform", "Sine"))
            # a complex-exponential NCO: one complex output (into a complex Outport, compared by part since the
            # complex-Outport export) and the valid output
            nout = (2 if wf == "Sine and cosine" else 1) + (1 if str(p.get("PhasePort", "off")) == "on" else 0) + 1
        elif et == "SwitchCase":
            import fxaction   # one port per case, then the default port
            nout = len(fxaction._cases(p.get("CaseConditions", "{1}"), "design.blocks." + name)) + \
                (1 if str(p.get("ShowDefaultCase", "on")) == "on" else 0)
        elif et in ("DataStoreMemory", "DataStoreWrite"):
            nout = None
        elif et in ("PreLookup", "Prelookup"):
            nout = 2   # index, fraction
        elif et == "Demux":
            ov = str(p.get("Outputs", "2")).strip()
            if ov.startswith("["):
                nout = len(ov.strip("[]").replace(",", " ").split())
            else:
                # recordings: Outputs '2/1' ended in INTERNAL here (float('2/1')); the engine's own integer
                # reading, and a refusal when it is not one
                import fxnum
                c_ = fxnum.canonical(ov, "int")
                if isinstance(c_, fxnum.NumError):
                    raise ExportError("UNSUPPORTED_EXPORT", "design.blocks.%s.Outputs" % name, "Demux Outputs %r: "
                                      "only an integer count or a list of widths is exported" % ov)
                nout = int(c_)
        else:
            nout = 1
        if nout_of.get(name, 1) > max(nout or 0, 1):
            raise ExportError("UNSUPPORTED_EXPORT", "design.blocks." + name, "a line leaves output port %d of a block "
                              "written with %d output ports" % (nout_of[name], nout))
        if et in ("Mux",):
            nin = max(nin, int(float(str(p.get("Inputs", "2")).strip()))) if not str(p.get("Inputs", "2")).strip().startswith("[") else nin
        lib_sids.append((name, sid))
        s.blocks.append((sid, _block_xml(bt, local, sid, prm, k_pos, B.pos(k_pos), nin=nin or None, nout=nout,
                                         lib=lib)))

    # 4. subsystem blocks and their port blocks
    def emit_ports(s, fe=False):
        for key, k in sorted(s.ports_in.items(), key=lambda kv: kv[1]):
            nm = _unique("In%d" % k, s.names)
            prm = [("Port", str(k))] if k != 1 else []
            if fe:   # element-wise partition (the saved For Each models)
                prm += [("Partition", "on"), ("PartitionDimension", "1"), ("PartitionWidth", "1"),
                        ("PartitionOffset", "0")]
            s.blocks.append((s.inport_sid[k], _block_xml("Inport", nm, s.inport_sid[k], prm, 0, B.pos(0))))
        for key, k in sorted(s.ports_out.items(), key=lambda kv: kv[1]):
            if key[0] == "enout":
                continue   # the EnabledOutport design block is this port
            nm = _unique("Out%d" % k, s.names)
            prm = ([("Port", str(k))] if k != 1 else []) + ([("ConcatenationDimension", "1")] if fe else [])
            s.blocks.append((s.outport_sid[k], _block_xml("Outport", nm, s.outport_sid[k], prm, 0, B.pos(0), nin=1)))
    for path, s in sorted(systems.items()):
        if path:
            fe = path in fe_levels
            emit_ports(s, fe)
            parent = systems[path[:-1]]
            sid = parent.sid_of[path[-1]]
            en_ = (enabled[path].get("port") or ("ifaction" if enabled[path].get("action") else True)) \
                if path in enabled else False
            mask = ""
            if path in fe_mask:
                mask = ('    <Mask>\n      <Display/>\n      <MaskParameter Name="K" Type="edit">\n'
                        '        <Prompt>K</Prompt>\n        <Value>%s</Value>\n      </MaskParameter>\n    </Mask>\n'
                        % escape(fe_mask[path][2]))
            parent.blocks.append((sid, _block_xml("SubSystem", path[-1], sid,
                                                  [("ContentPreviewEnabled", "off")] +
                                                  ([("TreatAsAtomicUnit", "on")] if (en_ or fe) else []), 0,
                                                  B.pos(0), nin=len(s.ports_in) or None,
                                                  nout=len(s.ports_out) or None, inner=sid, enable=en_, mask=mask)))

    # 5. the root model: From Workspace sources, the DUT subsystem, root Outports
    dut_sid = B.next_sid()
    root_names = {dut}
    zr = 0
    for k, (n, period, t, var) in enumerate(inports):
        zr += 1
        sid = B.next_sid()
        nm = _unique("src_" + n.replace("/", "_"), root_names)
        prm = [("VariableName", var), ("SampleTime", period), ("Interpolate", "off"),
               ("OutputAfterFinalValue", "Holding final value")]
        for kk, vv in prm:
            checks.append((nm, kk, vv))
        root.blocks.append((sid, _block_xml("FromWorkspace", nm, sid, prm, zr, B.pos(zr), nout=1)))
        root.lines[(sid, 1)] = [(dut_sid, in_no[n])]
    root_out = []
    for k, n in enumerate(outports):
        zr += 1
        sid = B.next_sid()
        nm = _unique(n, root_names)
        root_out.append(nm)
        prm = ([("Port", str(k + 1))] if k else []) + [("SampleTime", out_periods[k])]
        checks.append((nm, "SampleTime", out_periods[k]))
        root.blocks.append((sid, _block_xml("Outport", nm, sid, prm, zr, B.pos(zr), nin=1)))
        root.lines[(dut_sid, out_no[n])] = [(sid, 1)]
    root.blocks.append((dut_sid, _block_xml("SubSystem", dut, dut_sid, [("ContentPreviewEnabled", "off")], 0,
                                            B.pos(0), nin=len(inports) or None, nout=len(outports) or None,
                                            inner=dut_sid)))
    # the DUT's own ports are the design's Inport / Outport blocks (they are numbered by in_no / out_no)
    systems_by_sid = {dut_sid: top}
    for path, s in systems.items():
        if path:
            systems_by_sid[systems[path[:-1]].sid_of[path[-1]]] = s

    # 6. parts
    parts = {}
    sid_hi = B.sid
    root_xml = _system_xml(root, _p("ReportName", "simulink-default.rpt").replace("    ", "  ", 1)
                           + _p("SIDHighWatermark", sid_hi).replace("    ", "  ", 1))
    parts["simulink/systems/system_root.xml"] = root_xml
    child_of = {"system_root": [dut_sid]}
    for sid, s in systems_by_sid.items():
        parts["simulink/systems/system_%d.xml" % sid] = _system_xml(s)
        kids = []
        for nm, c in s.children.items():
            kids.append(s.sid_of[nm])
        if kids:
            child_of["system_%d" % sid] = sorted(kids)
    for ml_ in mlfns:
        parts["simulink/systems/system_%d.xml" % ml_["sid"]] = ml_["system"]
        owner_sid = dut_sid if not ml_["parent"] else systems[ml_["parent"][:-1]].sid_of[ml_["parent"][-1]]
        child_of.setdefault("system_%d" % owner_sid, []).append(ml_["sid"])
        child_of["system_%d" % owner_sid].sort()
    for owner, kids in child_of.items():
        rel = ['<?xml version="1.0" encoding="UTF-8" standalone="yes" ?>\n<Relationships xmlns="http://schemas.'
               'openxmlformats.org/package/2006/relationships">\n']
        for kid in kids:
            rel.append('  <Relationship Id="system_%d" Target="system_%d.xml" Type="http://schemas.mathworks.com/'
                       'simulink/2010/relationships/system"/>\n' % (kid, kid))
        rel.append("</Relationships>\n")
        parts["simulink/systems/_rels/%s.xml.rels" % owner] = "".join(rel)
    libs = []
    for name, sid in lib_sids:
        et = blocks[name]["type"]
        lib_ = _RAM_LIB[str(blocks[name].get("RAMType", "Simple dual port"))] if et == "HDLRAM" else TEMPLATES[et][1]
        if lib_ is not None:
            libs.append(("$bdroot/" + dut + "/" + name, lib_[0], str(sid)))
    parts.update(_model_parts(seed, model_name, solver, hw_profile(hw)[1], root_out, libs,
                              sorted(n for n in parts if n.startswith("simulink/systems/") and n.endswith(".xml"))))
    if mlfns:
        _mlfn_parts(parts, mlfns)
    data = _zip(parts)
    info = {"dropped_parameters": dropped, "unobserved_templates": sorted(unobserved), "root_outports": root_out,
            "dut": dut, "written": written}
    return data, checks, info


_ML_DEFAULT_FIMATH = {"Other:UserSpecified": "FimathUserSpecified"}
_ML_TREAT = {"Fixed-point & Integer": "FI_AND_INT"}
_ML_KEYS = ("Script", "EmlDefaultFimath", "InputFimath", "TreatAsFi", "SaturateOnIntegerOverflow")


def _mlfn_block(name, local, b, sid, zorder, pos, full, number):
    """(SubSystem block XML, info) of a MATLAB Function block: its inner system (Inports named after the
    function's arguments, the Stateflow S-Function, its Demux / Terminator, Outports), as R2026b saves it."""
    import fxhdl
    extra = sorted(k for k in b if k != "type" and k not in _ML_KEYS and not k.startswith("OutDataTypeStr"))
    if extra:
        raise ExportError("UNSUPPORTED_EXPORT", "design.blocks.%s.%s" % (name, extra[0]), "MATLAB Function parameter "
                          "%s is not exported" % extra[0])
    main = fxhdl._ml_main(b.get("Script", ""), "design.blocks.%s.Script" % name)
    ins, outs = main["inputs"], main["outputs"]
    if not outs:
        raise ExportError("UNSUPPORTED_EXPORT", "design.blocks." + name, "a MATLAB Function without an output")
    prm = [("ErrorFcn", "Stateflow.Translate.translate"), ("PermitHierarchicalResolution", "ExplicitOnly"),
           ("TreatAsAtomicUnit", "on"), ("SFBlockType", "MATLAB Function"), ("ContentPreviewEnabled", "on")]
    xml = _block_xml("SubSystem", local, sid, prm, zorder, pos, nin=len(ins) or None, nout=len(outs), inner=sid)
    o = "%d::" % sid
    blk = []
    for k, a in enumerate(ins):
        blk.append('  <Block BlockType="Inport" Name=%s SID="%s%d">\n' % (quoteattr(a), o, k + 1)
                   + (_p("Port", k + 1) if k else "") + _p("Position", "[20, %d, 40, %d]" % (101 + 35 * k, 119 + 35 * k))
                   + _p("ZOrder", -(k + 1)) + "  </Block>\n")
    nsf = len(outs) + 1
    sf = ('  <Block BlockType="S-Function" Name=" SFunction " SID="%s%d">\n' % (o, 100)
          + _p("Tag", "Stateflow S-Function %d" % (number + 1))
          + ('    <PortCounts in="%d" out="%d"/>\n' % (len(ins), nsf) if ins else '    <PortCounts out="%d"/>\n' % nsf)
          + _p("Position", "[180, 100, 230, %d]" % (160 + 20 * len(outs))) + _p("ZOrder", 16)
          + _p("FunctionName", "sf_sfun") + _p("PortCounts", "[%d %d]" % (len(ins), nsf))
          + _p("SFunctionDeploymentMode", "off") + _p("EnableBusSupport", "off") + _p("SFcnIsStateOwnerBlock", "off")
          + "    <PortProperties>\n"
          + "".join('      <Port Type="out" Index="%d">\n        <P Name="Name">%s</P>\n      </Port>\n'
                    % (k + 2, escape(y)) for k, y in enumerate(outs))
          + "    </PortProperties>\n  </Block>\n")
    blk.append(sf)
    blk.append('  <Block BlockType="Demux" Name=" Demux " SID="%s101">\n    <PortCounts in="1" out="1"/>\n' % o
               + _p("Position", "[270, 230, 320, 270]") + _p("ZOrder", 17) + _p("Outputs", 1) + "  </Block>\n")
    blk.append('  <Block BlockType="Terminator" Name=" Terminator " SID="%s102">\n' % o
               + _p("Position", "[460, 241, 480, 259]") + _p("ZOrder", 18) + "  </Block>\n")
    for k, y in enumerate(outs):
        blk.append('  <Block BlockType="Outport" Name=%s SID="%s%d">\n' % (quoteattr(y), o, 50 + k)
                   + (_p("Port", k + 1) if k else "") + _p("Position", "[460, %d, 480, %d]" % (101 + 35 * k, 119 + 35 * k))
                   + _p("ZOrder", -(50 + k)) + "  </Block>\n")
    ln, z = [], 0
    for k in range(len(ins)):
        z += 1
        ln.append("  <Line>\n" + _p("ZOrder", z) + _p("Src", "%s%d#out:1" % (o, k + 1))
                  + _p("Dst", "%s100#in:%d" % (o, k + 1)) + "  </Line>\n")
    for k, y in enumerate(outs):
        z += 1
        ln.append("  <Line>\n" + _p("Name", y) + _p("ZOrder", z) + _p("Labels", "[0, 0]")
                  + _p("Src", "%s100#out:%d" % (o, k + 2)) + _p("Dst", "%s%d#in:1" % (o, 50 + k)) + "  </Line>\n")
    z += 1
    ln.append("  <Line>\n" + _p("ZOrder", z) + _p("Src", "%s101#out:1" % o) + _p("Dst", "%s102#in:1" % o)
              + "  </Line>\n")
    z += 1
    ln.append("  <Line>\n" + _p("ZOrder", z) + _p("Src", "%s100#out:1" % o) + _p("Dst", "%s101#in:1" % o)
              + "  </Line>\n")
    system = ('<?xml version="1.0" encoding="utf-8"?>\n<System>\n' + _p("SIDHighWatermark", 102).replace("    ", "  ", 1)
              + "".join(blk) + "".join(ln) + "</System>\n")
    return xml, {"sid": sid, "system": system, "name": local, "path": full, "block": b, "ins": ins, "outs": outs,
                 "fname": main["name"], "number": number}


def _ml_data(ssid, nm, scope, dtype):
    return ('    <data SSID="%d" name=%s>\n      <P Name="scope">%s</P>\n      <props>\n        <array>\n'
            '          <P Name="size">-1</P>\n        </array>\n        <type>\n'
            '          <P Name="method">SF_INHERITED_TYPE</P>\n          <P Name="primitive">SF_DOUBLE_TYPE</P>\n'
            '        </type>\n        <P Name="complexity">SF_COMPLEX_INHERITED</P>\n%s        <unit>\n'
            '          <P Name="name">inherit</P>\n        </unit>\n      </props>\n'
            '      <P Name="dataType">%s</P>\n    </data>\n'
            % (ssid, quoteattr(nm), scope, '        <P Name="frame">SF_FRAME_NO</P>\n' if scope == "OUTPUT_DATA" else "",
               escape(dtype)))


def _mlfn_parts(parts, mlfns):
    """The Stateflow machine, one chart per MATLAB Function, their relationships and content types (as R2026b
    saves them, recorded samples; the reader engine/fxslx_ml.py reads them back)."""
    # machine / target ids after every chart / instance id (id 1 / 2 failed a Stateflow
    # assertion at load); R2026b saves e.g. machine 6, target 14
    mid = 11 + 2 * (len(mlfns) - 1) + 1
    mach = ['<?xml version="1.0" encoding="utf-8"?>\n<Stateflow>\n  <machine id="%d">\n' % mid
            + '    <P Name="sfVersion">82650000</P>\n    <Children>\n      <target id="%d" name="sfun">\n' % (mid + 1)
            + '        <P Name="description">Default Simulink S-Function Target.</P>\n      </target>\n']
    inst, rels, ct = [], [], []
    for k, ml in enumerate(mlfns):
        cid, iid = 10 + 2 * k, 11 + 2 * k
        mach.append('      <chart Ref="chart_%d"/>\n' % cid)
        inst.append('  <instance id="%d">\n    <P Name="machine">%d</P>\n    <P Name="name">%s</P>\n'
                    '    <P Name="chart">%d</P>\n  </instance>\n' % (iid, mid, escape(ml["path"]), cid))
        rels.append('  <Relationship Id="chart_%d" Target="chart_%d.xml" Type="http://schemas.mathworks.com/'
                    'simulinkModel/2018/relationships/stateflowMachine"/>\n' % (cid, cid))
        ct.append('  <Override ContentType="application/vnd.mathworks.stateflow.chart+xml" '
                  'PartName="/simulink/stateflow/chart_%d.xml"/>\n' % cid)
        b = ml["block"]
        eml = [_p("isEML", 1), _p("script", b.get("Script", "")),
               '        <P Name="editorLayout" Class="double">Matrix(4,1)\n[10.0, 5.0, 700.0, 500.0]</P>\n']
        if b.get("InputFimath"):
            eml.append(_p("fimathString", b["InputFimath"]))
        dfm = str(b.get("EmlDefaultFimath", "Same as MATLAB Default"))
        if dfm != "Same as MATLAB Default":
            if dfm not in _ML_DEFAULT_FIMATH:
                raise ExportError("UNSUPPORTED_EXPORT", "design.blocks.%s.EmlDefaultFimath" % ml["name"], dfm)
            eml.append(_p("emlDefaultFimath", _ML_DEFAULT_FIMATH[dfm]))
        tr = str(b.get("TreatAsFi", "Fixed-point"))
        if tr != "Fixed-point":
            if tr not in _ML_TREAT:
                raise ExportError("UNSUPPORTED_EXPORT", "design.blocks.%s.TreatAsFi" % ml["name"], tr)
            eml.append(_p("treatAsFi", _ML_TREAT[tr]))
        if str(b.get("SaturateOnIntegerOverflow", "on")) in ("off", "0", "False", "false"):
            eml.append(_p("saturateOnIntegerOverflow", 0))
        nd = len(ml["ins"]) + len(ml["outs"])
        data = "".join(_ml_data(4 + j, a, "INPUT_DATA", "Inherit: Same as Simulink") for j, a in enumerate(ml["ins"]))
        for j, y in enumerate(ml["outs"]):
            dt = b.get("OutDataTypeStr" if j == 0 else "OutDataTypeStr%d" % (j + 1), "Inherit: Same as Simulink")
            data += _ml_data(4 + len(ml["ins"]) + j, y, "OUTPUT_DATA", dt)
        chart = ('<?xml version="1.0" encoding="utf-8"?>\n<chart id="%d">\n' % cid
                 # the chart's name is the block's path in the model, as the instance's
                 + _p("name", ml["path"]).replace("    ", "  ", 1)
                 + "  <P Name=\"screen\">[1 1 3600 1200 1.180555555555556]</P>\n"
                 + '  <P Name="viewObj">%d</P>\n  <P Name="toolbarMode">LIBRARY_TOOLBAR</P>\n' % cid
                 + '  <P Name="ssIdHighWaterMark">%d</P>\n' % (3 + nd)
                 + '  <P Name="decomposition">CLUSTER_CHART</P>\n  <P Name="type">EML_CHART</P>\n'
                 + '  <P Name="chartFileNumber">%d</P>\n  <P Name="disableImplicitCasting">1</P>\n' % (ml["number"] + 1)
                 + '  <eml>\n    <P Name="name">%s</P>\n  </eml>\n  <Children>\n' % escape(ml["fname"])
                 + '    <state SSID="1">\n      <P Name="labelString">eML_blk_kernel()</P>\n'
                 '      <P Name="position">[18 64.5 118 66]</P>\n      <P Name="fontSize">12</P>\n'
                 '      <P Name="superState">SUBCHART</P>\n      <P Name="subviewer">%d</P>\n'
                 '      <P Name="type">FUNC_STATE</P>\n      <P Name="decomposition">CLUSTER_STATE</P>\n'
                 '      <eml>\n' % cid
                 + "".join("    " + x if not x.startswith("        ") else x for x in eml)
                 + "      </eml>\n    </state>\n" + data
                 + '    <transition SSID="2">\n      <P Name="labelString">{eML_blk_kernel();}</P>\n'
                 '      <P Name="labelPosition">[28.125 13.875 102.544 14.964]</P>\n      <P Name="fontSize">12</P>\n'
                 '      <src>\n        <P Name="intersection">[0 0 1 0 23.5747 14.625 0 0]</P>\n      </src>\n'
                 '      <dst>\n        <P Name="SSID">3</P>\n'
                 '        <P Name="intersection">[1 0 -1 0 23.5747 42.5747 0 0]</P>\n      </dst>\n'
                 '      <P Name="midPoint">[23.5747 24.9468]</P>\n'
                 '      <P Name="dataLimits">[21.175 25.975 14.625 42.575]</P>\n'
                 '      <P Name="subviewer">%d</P>\n      <P Name="drawStyle">SMART</P>\n'
                 '      <slide>\n        <P Name="sticky">BOTH_STICK</P>\n      </slide>\n'
                 '      <P Name="executionOrder">1</P>\n    </transition>\n' % cid
                 + '    <junction SSID="3">\n      <P Name="position">[23.5747 49.5747 7]</P>\n'
                 '      <P Name="subviewer">%d</P>\n      <P Name="type">CONNECTIVE_JUNCTION</P>\n'
                 '    </junction>\n  </Children>\n</chart>\n' % cid)
        parts["simulink/stateflow/chart_%d.xml" % cid] = chart
    mach.append("    </Children>\n  </machine>\n")
    parts["simulink/stateflow/machine.xml"] = "".join(mach + inst) + "</Stateflow>\n"
    parts["simulink/stateflow/_rels/machine.xml.rels"] = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes" ?>\n<Relationships xmlns="http://schemas.'
        'openxmlformats.org/package/2006/relationships">\n' + "".join(rels) + "</Relationships>\n")
    k = "simulink/_rels/blockDiagram.json.rels"
    parts[k] = parts[k].replace("</Relationships>", '  <Relationship Id="stateflow_machine" Target="stateflow/'
                                'machine.xml" Type="http://schemas.mathworks.com/simulinkModel/2014/relationships/'
                                'stateflowMachine"/>\n</Relationships>')
    ct.append('  <Override ContentType="application/vnd.mathworks.stateflow.machine+xml" '
              'PartName="/simulink/stateflow/machine.xml"/>\n')
    parts["[Content_Types].xml"] = parts["[Content_Types].xml"].replace("</Types>", "".join(ct) + "</Types>")


def _zip(parts):
    buf = io.BytesIO()
    order = ["[Content_Types].xml", "_rels/.rels"] + sorted(n for n in parts if n not in ("[Content_Types].xml",
                                                                                          "_rels/.rels"))
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        for n in order:
            zi = zipfile.ZipInfo(n, date_time=(2026, 1, 1, 0, 0, 0))
            zi.compress_type = zipfile.ZIP_DEFLATED
            z.writestr(zi, parts[n].encode("utf-8") if isinstance(parts[n], str) else parts[n])
    return buf.getvalue()


def _json(o):
    return json.dumps(o, indent=2, ensure_ascii=False)


def _model_parts(seed, name, solver, hwcc, root_out, libs, system_parts):
    P = {}
    ct = ['<?xml version="1.0" encoding="UTF-8" standalone="yes" ?>\n'
          '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">\n'
          '  <Default ContentType="application/vnd.mathworks.simulink.configSetInfo+json" Extension="json"/>\n'
          '  <Default ContentType="application/vnd.openxmlformats-package.relationships+xml" Extension="rels"/>\n'
          '  <Default ContentType="application/vnd.mathworks.simulink.mf0+xml" Extension="xml"/>\n'
          '  <Override ContentType="application/vnd.openxmlformats-package.core-properties+xml" PartName="/metadata/coreProperties.xml"/>\n'
          '  <Override ContentType="application/vnd.mathworks.package.coreProperties+xml" PartName="/metadata/mwcoreProperties.xml"/>\n'
          '  <Override ContentType="application/vnd.mathworks.package.corePropertiesExtension+xml" PartName="/metadata/mwcorePropertiesExtension.xml"/>\n'
          '  <Override ContentType="application/vnd.mathworks.package.corePropertiesReleaseInfo+xml" PartName="/metadata/mwcorePropertiesReleaseInfo.xml"/>\n'
          '  <Override ContentType="application/vnd.mathworks.simulink.mdl+json" PartName="/simulink/bddefaults.json"/>\n'
          '  <Override ContentType="application/vnd.mathworks.simulink.mdl+json" PartName="/simulink/blockDiagram.json"/>\n'
          '  <Override ContentType="application/vnd.mathworks.simulink.configSet+json" PartName="/simulink/configSet0.json"/>\n'
          '  <Override ContentType="application/vnd.mathworks.simulink.graphicalInterface+json" PartName="/simulink/graphicalInterface.json"/>\n']
    for sp in system_parts:
        ct.append('  <Override ContentType="application/vnd.mathworks.simulink.mdl+xml" PartName="/%s"/>\n' % sp)
    ct.append("</Types>\n")
    P["[Content_Types].xml"] = "".join(ct)
    P["_rels/.rels"] = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes" ?>\n'
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">\n'
        '  <Relationship Id="blockDiagram" Target="simulink/blockDiagram.json" Type="http://schemas.mathworks.com/simulink/2010/relationships/blockDiagram"/>\n'
        '  <Relationship Id="blockDiagramDefaults" Target="simulink/bddefaults.json" Type="http://schemas.mathworks.com/simulink/2017/relationships/blockDiagramDefaults"/>\n'
        '  <Relationship Id="configSetInfo" Target="simulink/configSetInfo.json" Type="http://schemas.mathworks.com/simulink/2014/relationships/configSetInfo"/>\n'
        '  <Relationship Id="modelDictionary" Target="simulink/modelDictionary.xml" Type="http://schemas.mathworks.com/simulinkModel/2016/relationships/modelDictionary"/>\n'
        '  <Relationship Id="rId1" Target="metadata/mwcoreProperties.xml" Type="http://schemas.mathworks.com/package/2012/relationships/coreProperties"/>\n'
        '  <Relationship Id="rId2" Target="metadata/mwcorePropertiesExtension.xml" Type="http://schemas.mathworks.com/package/2014/relationships/corePropertiesExtension"/>\n'
        '  <Relationship Id="rId3" Target="metadata/mwcorePropertiesReleaseInfo.xml" Type="http://schemas.mathworks.com/package/2019/relationships/corePropertiesReleaseInfo"/>\n'
        '  <Relationship Id="rId4" Target="metadata/coreProperties.xml" Type="http://schemas.openxmlformats.org/package/2006/relationships/metadata/core-properties"/>\n'
        '</Relationships>\n')
    P["simulink/_rels/blockDiagram.json.rels"] = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes" ?>\n'
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">\n'
        '  <Relationship Id="graphicalInterface" Target="graphicalInterface.json" Type="http://schemas.mathworks.com/simulinkModel/2014/relationships/graphicalInterface"/>\n'
        '  <Relationship Id="system_root" Target="systems/system_root.xml" Type="http://schemas.mathworks.com/simulink/2010/relationships/system"/>\n'
        '</Relationships>\n')
    P["simulink/_rels/configSetInfo.json.rels"] = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes" ?>\n'
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">\n'
        '  <Relationship Id="configSet0" Target="configSet0.json" Type="http://schemas.mathworks.com/simulink/2014/relationships/configSet"/>\n'
        '</Relationships>\n')
    P["metadata/coreProperties.xml"] = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes" ?>\n'
        '<cp:coreProperties xmlns:cp="http://schemas.openxmlformats.org/package/2006/metadata/core-properties" '
        'xmlns:dc="http://purl.org/dc/elements/1.1/" xmlns:dcmitype="http://purl.org/dc/dcmitype/" '
        'xmlns:dcterms="http://purl.org/dc/terms/" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">\n'
        '  <cp:category>model</cp:category>\n'
        '  <dcterms:created xsi:type="dcterms:W3CDTF">2026-01-01T00:00:00Z</dcterms:created>\n'
        '  <dc:creator>fxsim</dc:creator>\n  <cp:lastModifiedBy>fxsim</cp:lastModifiedBy>\n'
        '  <dcterms:modified xsi:type="dcterms:W3CDTF">2026-01-01T00:00:00Z</dcterms:modified>\n'
        '  <cp:revision>1.0</cp:revision>\n  <cp:version>%s</cp:version>\n</cp:coreProperties>\n' % RELEASE)
    P["metadata/mwcoreProperties.xml"] = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes" ?>\n'
        '<mwcoreProperties xmlns="http://schemas.mathworks.com/package/2012/coreProperties">\n'
        '  <contentType>application/vnd.mathworks.simulink.model</contentType>\n'
        '  <contentTypeFriendlyName>Simulink Model</contentTypeFriendlyName>\n'
        '  <matlabRelease>%s</matlabRelease>\n</mwcoreProperties>\n' % RELEASE)
    P["metadata/mwcorePropertiesExtension.xml"] = (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes" ?>\n'
        '<mwcoreProperties xmlns="http://schemas.mathworks.com/package/2014/corePropertiesExtension">\n'
        '  <uuid>%s</uuid>\n</mwcoreProperties>\n' % _uuid(seed, "package"))
    P["metadata/mwcorePropertiesReleaseInfo.xml"] = (
        '<?xml version="1.0" encoding="UTF-8"?>\n<!-- Version information for MathWorks %s Release -->\n'
        '<MathWorks_version_info>\n  <version>26.2.0.3433840</version>\n'
        '  <release>%s</release>\n  <description>Update 1</description>\n  <date>Oct 16 2026</date>\n'
        '  <checksum>4265391219</checksum>\n</MathWorks_version_info>\n' % (RELEASE, RELEASE))
    # This ReleaseInfo format preserves the saved solver, step and stop time on R2026b Update 1.
    # The XML comment is significant to that reader.
    P["simulink/bddefaults.json"] = _json({"_mw_element_name": "BlockDiagramDefaults"})
    P["simulink/blockDiagram.json"] = _json({"BlockDiagram": {
        "EngineSettings": {"IsSoftwareComponent": False},
        "LastSavedArchitecture": "win64",
        "ModelUUID": _uuid(seed, "model"),
        "SLXCompressionType": "None",
        "SimulationSettings": {"SimulationMode": "normal"},
        "System": {"Ref": "system_root"}}})
    P["simulink/configSetInfo.json"] = _json({"ConfigSetInfo": [{"Active": True, "ConfigSetName": "Configuration",
                                                                 "PartName": "/simulink/configSet0.json"}]})
    gi = {}
    if libs:
        gi["ExternalFileReferences"] = [{"Path": p, "Reference": r, "SID": sid, "Type": "LIBRARY_BLOCK"}
                                        for p, r, sid in libs]
    gi.update({"PreCompExecutionDomainType": "Unset", "RootOutports": [{"Name": n} for n in root_out],
               "SimulinkSubDomainType": "Simulink", "SolverName": "FixedStepDiscrete",
               "_mw_element_name": "GraphicalInterface"})
    P["simulink/graphicalInterface.json"] = _json(gi)
    P["simulink/modelDictionary.xml"] = (
        '<?xml version="1.0" encoding="UTF-8"?>\n<MF0 version="1.1" defaultPackage="slid" '
        'packageUris="http://schema.mathworks.com/mf0/slid/R2025a_20251111">\n'
        '  <System type="System" uuid="%s">\n    <Interface type="Interface" uuid="%s"/>\n  </System>\n</MF0>\n'
        % (_uuid(seed, "system"), _uuid(seed, "interface")))
    sol = {"FixedStep": solver["FixedStep"], "SolverName": "FixedStepDiscrete", "StopTime": solver["StopTime"]}
    if solver.get("SolverMode"):
        sol["SolverMode"] = solver["SolverMode"]
    sol["Version"] = CC_VERSION
    hw = dict(hwcc)
    hw["Version"] = CC_VERSION
    comps = [("Simulink.SolverCC", sol),
             ("Simulink.DataIOCC", {"SaveFormat": "Dataset", "SaveOutput": "on", "SaveTime": "on",
                                    "Version": CC_VERSION}),
             ("Simulink.OptimizationCC", {"Version": CC_VERSION}),
             # every exported model had ParameterOverflowMsg / ParameterDowncastMsg 'none', so
             # a parameter outside its type saturated silently in Simulink; they are Simulink's defaults ('error') now,
             # as the engine assumes (a previous version). Precision loss stays 'none': it only reports (a default 'warning'
             # would fail the check's no-warning rule on any inexact parameter) and changes no value
             ("Simulink.DebuggingCC", {"IntegerOverflowMsg": "none", "IntegerSaturationMsg": "none",
                                       "ParameterDowncastMsg": "error", "ParameterOverflowMsg": "error",
                                       "ParameterPrecisionLossMsg": "none", "Version": CC_VERSION}),
             ("Simulink.HardwareCC", hw),
             ("Simulink.ModelReferenceCC", {"Version": CC_VERSION})]
    P["simulink/configSet0.json"] = _json({
        "_id": "1", "_mw_element_name": "ConfigSet", "_mw_element_type": "COSObject",
        "_object_class": "Simulink.ConfigSet",
        "_properties": {"Components": {"_array_class": "Simulink.ConfigComponent", "_dimensions": [len(comps), 1],
                                       "_elements": [{"_id": str(k + 2), "_object_class": c, "_properties": pr}
                                                     for k, (c, pr) in enumerate(comps)]},
                        "ConfigPrmDlgPosition": [0, 0, 0, 0], "CurrentDlgPage": "Solver", "ExtraOptions": "",
                        "Name": "Configuration", "Version": CC_VERSION}})
    return P


# ================================================================ the verification package
def _bins(v, s, wl):
    """Two's complement bit string of stored integer v (wl bits)."""
    return format(v % (1 << wl), "0%db" % wl)


def _tuple_str(t):
    return "boolean" if t == G.BOOL else "%d,%d,%d" % t


def _period_text(fr):
    """A period / time as a decimal literal MATLAB reads back exactly (dyadic) or as the nearest double."""
    if fr.denominator & (fr.denominator - 1) == 0:
        return repr(float(fr)) if float(fr) != int(fr) else str(int(fr))
    return repr(float(fr))


def _binary(fr):
    """whether a Fraction is a binary fraction (exact as a double)"""
    d = fr.denominator
    return d & (d - 1) == 0


def _input_stamps(per, off, n):
    """the From Workspace time stamps of an input's n samples. A period and offset that are binary fractions: the
    hit times themselves (exact as doubles). Otherwise (inputs at 0.1 and 0.3 -- the stamp 0.9 is the
    double nearest 9/10, while Simulink's own hit there was 0.8999...; From Workspace without interpolation then
    returned the previous sample): sample k > 0 is stamped half a period before its hit, so each hit picks its own
    sample whatever the rounding of the hit time (sample 0 stays at the offset)."""
    if _binary(per) and _binary(off):
        return [float(off + k * per) for k in range(n)]
    return [float(off)] + [float(off + k * per - per / 2) for k in range(1, n)]


def _counter_ts(b):
    """a Counter Limited / Free-Running block's stated sample time text (SampleTime, else tsamp; absent: '1', as
    block_params writes it)"""
    return str(b.get("SampleTime", b.get("tsamp", "1"))).strip()


def _counter_stated_period(b):
    """the counter's stated period as the engine's sample_times text ('3'), '-1' for inherited, or None when it is
    no whole period (an offset, a fraction, an expression)"""
    ts = _counter_ts(b)
    return ts if ts == "-1" or re.match(r"^[1-9][0-9]*$", ts) else None


def _sample_time_text(st):
    """Inport SampleTime as written by the design ('1', '2', '[2, 1]', '1/4'): (Simulink text, period, offset)."""
    v = str(st).strip()
    off = Fraction(0)
    if v.startswith("["):
        parts = v.strip("[]").replace(",", " ").split()
        if len(parts) != 2:
            raise ExportError("UNSUPPORTED_EXPORT", "SampleTime", "sample time %r" % st)
        per, off = Fraction(parts[0]), Fraction(parts[1])
    else:
        per = Fraction(v)
    if per <= 0:
        raise ExportError("UNSUPPORTED_EXPORT", "SampleTime", "inherited / non-positive design input period %r" % st)
    txt = _period_text(per) if off == 0 else "[%s %s]" % (_period_text(per), _period_text(off))
    return txt, per, off


def model_name_ok(name):
    return isinstance(name, str) and re.match(r"^[A-Za-z][A-Za-z0-9_]{0,59}$", name) is not None


def plan_ports(design, inputs):
    """Design Inports (in port order) and Outports (in port order). Port order: the design's Port parameter when
    given, else the block name."""
    blocks = design["blocks"]

    def key(n):
        p = blocks[n].get("Port")
        try:
            return (0, int(str(p)), n) if p is not None else (1, 0, n)
        except ValueError:
            return (1, 0, n)
    ins = sorted((n for n, b in blocks.items() if b.get("type") == "Inport"), key=key)
    outs = sorted((n for n, b in blocks.items() if b.get("type") == "Outport"), key=key)
    if not outs:
        raise ExportError("UNSUPPORTED_EXPORT", "design", "a design without Outport has nothing to verify (name "
                          "internal blocks in \"observe\" to bring them out as outputs)")
    return ins, outs


MATLAB_TEMPLATE = r"""function r = %(fn)s(pkgdir)
%% %(fn)s  Self-contained check of an fxsim export package (generated by fxexport, format %(fmt)s).
%%   r = %(fn)s()        uses the files next to this script
%%   r = %(fn)s(pkgdir)  uses the package in pkgdir
%% Opens %(model)s.slx, verifies the model settings and every exported block parameter, feeds the recorded inputs
%% (From Workspace, stored integers rebuilt bit for bit), simulates, and compares every sample of every root
%% output (stored integer bit pattern, sample time) and every compiled type with fxsim's expected values in
%% %(model)s_io.json. Prints exactly one JSON line and returns it as a struct. verdict is PASS only when the
%% expectation is complete (every root output and element, non-empty samples, a type for every design block or a
%% verified reason why it has none) and every comparison was made and agreed; INCOMPLETE when every sample agreed
%% but a compiled type could not be compared; FAIL otherwise (a difference, an error, an incomplete expectation).
%% Before anything else the model and io files must hash to manifest.json's sha256 (else PACKAGE_DIGEST_MISMATCH),
%% and every sample's decimal integer in the io file must be the value of its bit pattern (else IO_INCONSISTENT).
if nargin < 1 || isempty(pkgdir), pkgdir = fileparts(mfilename('fullpath')); end
mdl = '%(model)s';
r = struct('tool', 'fxexport-check', 'format', '%(fmt)s', 'model', mdl, 'verdict', 'FAIL', 'reason', '', ...
           'first_diff', [], 'outputs_checked', 0, 'samples_checked', 0, 'types_checked', 0, ...
           'params_checked', 0, 'model_sha256_checked', false, 'library_defaults_checked', 0, ...
           'package_digest_checked', false, ...
           'matlab', version());
r.types_unchecked = {};
r.warnings_allowed = {};
vars = {};
try
    %% Verify the model and I/O file digests against this manifest.
    [why, dd] = package_digest(pkgdir, mdl);
    if ~isempty(why)
        r.reason = why; r.first_diff = dd;
        if strcmp(why, 'PACKAGE_DIGEST_UNCHECKED'), r.verdict = 'INCOMPLETE'; end
        emit(r); return
    end
    r.package_digest_checked = true;
    io = jsondecode(fileread(fullfile(pkgdir, [mdl '_io.json'])));
    why = io_incomplete(io);   %% the expectation itself must be complete before anything can PASS
    if ~isempty(why)
        r.reason = 'IO_INCOMPLETE'; r.first_diff = struct('message', why);
        emit(r); return
    end
    %% Check each decimal integer against its stored bit pattern and declared type.
    why = io_inconsistent(io);
    if ~isempty(why)
        r.reason = 'IO_INCONSISTENT'; r.first_diff = struct('message', why);
        emit(r); return
    end
    mfile = fullfile(pkgdir, [mdl '.slx']);
    [ok, sha] = sha256_of(mfile);
    if ok
        r.model_sha256_checked = true;
        if ~strcmpi(sha, io.model_sha256)
            r.reason = 'MODEL_SHA256_MISMATCH'; r.first_diff = struct('expected', io.model_sha256, 'actual', sha);
            emit(r); return
        end
    end
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    %% Compile in a fresh temporary build folder so warnings are visible on every run.
    fgc = Simulink.fileGenControl('getConfig');
    %% Restore explicit cache folders after the check. Reset folders that used the session default.
    fgc_default = strcmp(char(fgc.CacheFolder), pwd) && strcmp(char(fgc.CodeGenFolder), pwd);
    gen = tempname; mkdir(gen);
    Simulink.fileGenControl('set', 'CacheFolder', gen, 'CodeGenFolder', gen);
    cg = onCleanup(@() restore_filegen(fgc, fgc_default, gen, mdl)); %%#ok<NASGU>
    %% Unload cached MEX files before compiling. This clears all MEX files in the session.
    clear mex
    lastwarn('', '');
    load_system(mfile);
    cl = onCleanup(@() cleanup(mdl));
    [wmsg, wid] = lastwarn;
    if ~isempty(wmsg)
        %% Treat a load warning as a failed check. The model configuration may be incomplete.
        r.reason = 'LOAD_WARNING'; r.first_diff = struct('id', wid, 'message', wmsg);
        emit(r); return
    end
    %% 0. every design block of the expectation exists; the root Inports are exactly the expected inputs
    db = cellstr(io.design_blocks);
    for k = 1:numel(db)
        if getSimulinkBlockHandle([mdl '/' db{k}]) == -1
            r.reason = 'IO_INCOMPLETE'; r.first_diff = struct('message', ['no block ' db{k}]);
            emit(r); return
        end
    end
    %% Reject unexpected blocks inside the DUT.
    extra = dut_extra(mdl, io);
    if ~isempty(extra)
        r.reason = 'MODEL_EXTRA_BLOCK';
        r.first_diff = struct('message', 'blocks in the model that are not in the exported design', 'blocks', {extra});
        emit(r); return
    end
    %% Verify one root From Workspace block per input and check its variable name.
    fw = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'FromWorkspace');
    fwv = cellfun(@(b) get_param(b, 'VariableName'), fw, 'UniformOutput', false);
    nrin = numel(find_system(mdl, 'SearchDepth', 1, 'BlockType', 'Inport'));
    want = cell(1, numel(io.inputs));
    for k = 1:numel(io.inputs), u = item(io.inputs, k); want{k} = u.variable; end
    if nrin ~= 0 || numel(fw) ~= numel(io.inputs) || ~isempty(setxor(fwv(:)', want))
        r.reason = 'IO_INCOMPLETE';
        r.first_diff = struct('message', 'the root From Workspace blocks differ from the expected inputs', ...
            'expected', {want}, 'actual', {fwv(:)'}, 'root_inports', nrin);
        emit(r); return
    end
    %% 1. model settings
    cfg = io.config; f = fieldnames(cfg);
    for k = 1:numel(f)
        got = get_param(mdl, f{k}); if isnumeric(got), got = num2str(got); end
        if ~strcmp(strtrim(got), strtrim(cfg.(f{k})))
            r.reason = 'CONFIG_MISMATCH'; r.first_diff = struct('param', f{k}, 'expected', cfg.(f{k}), 'actual', got);
            emit(r); return
        end
    end
    %% 2. block parameters as written by the export (a parameter Simulink did not take is a FAIL, not a guess)
    bp = io.block_params;
    for k = 1:numel(bp)
        e = item(bp, k);
        got = get_param([mdl '/' e.block], e.param); if isnumeric(got), got = num2str(got); end
        if ~strcmp(strtrim(got), strtrim(e.value)) && ~same_number(got, e.value)
            r.reason = 'PARAM_MISMATCH'; r.first_diff = struct('block', e.block, 'param', e.param, ...
                'expected', e.value, 'actual', got);
            emit(r); return
        end
        r.params_checked = r.params_checked + 1;
    end
    %% Compare exported defaults for omitted parameters with a fresh library block.
    [bad, n, why] = library_defaults(io);
    r.library_defaults_checked = n;
    if ~isempty(why)
        r.reason = 'LIBRARY_DEFAULT_UNCHECKED'; r.verdict = 'INCOMPLETE'; r.first_diff = why;
        emit(r); return
    end
    if ~isempty(bad)
        r.reason = 'ASSUMED_DEFAULT_NOT_LIBRARY'; r.first_diff = bad;
        emit(r); return
    end
    %% 3. inputs: stored integers rebuilt from their bit patterns, checked bit for bit before use
    tsl = cell(1, numel(io.inputs));
    for k = 1:numel(io.inputs)
        u = item(io.inputs, k);
        bins = cellstr(u.bins);
        n = numel(bins);
        if isfield(u, 'type') && strcmp(u.type, 'boolean')
            v = logical(double(u.nums(:)));   %% a boolean input: 0 / 1 per sample
            vb = cellstr(num2str(double(v(:))));
        else
            v = fi(zeros(n, 1), u.s, u.wl, u.fl);
            if u.wl <= 53 && abs(u.fl) < 900
                v = fi(double(u.nums(:)) .* 2^(-u.fl), u.s, u.wl, u.fl);
            else
                v.bin = char(bins);
            end
            vb = cellstr(bin(v(:)));
        end
        if ~isequal(strtrim(vb), bins(:))
            r.reason = 'INPUT_BUILD_FAILED'; r.first_diff = struct('input', u.port);
            emit(r); return
        end
        v = reshape(v, numel(u.t), u.width); %% a vector input: one column per element
        tsl{k} = timeseries(v, double(u.t(:)));
    end
    for k = 1:numel(io.inputs)
        u = item(io.inputs, k);
        assignin('base', u.variable, tsl{k}); vars{end + 1} = u.variable; %%#ok<AGROW>
    end
    cv = onCleanup(@() clear_vars(vars));
    %% 4. compiled types of every design block
    ct = io.compiled_types;
    pt = io.port_types;
    %% every warning of compiling and simulating is seen and fails the check (warnings switched on
    %% for the check, the caller's state restored after it)
    wst = warning; cw = onCleanup(@() warning(wst)); warning('on', 'all'); %%#ok<NASGU>
    %% the one warning a package predicts (an Extract Bits taking one bit of a signed input warns
    %% sfix_dtpropBitLess from its internal DTProp1) is passed over for the listed blocks only -- matched on the id
    %% and the block path in the message (warning('off', id) does not stop this diagnostic); the
    %% values and types are compared as usual
    wa = {};
    if isfield(io, 'warnings_allowed'), wa = io.warnings_allowed; end
    lastwarn('', '');
    feval(mdl, [], [], [], 'compile');
    compiled = true;
    %% every expected further-port entry must be a port the block has, consumed exactly once
    used = false(1, numel(pt));
    try
        for k = 1:numel(ct)
            e = item(ct, k);
            p = get_param([mdl '/' e.block], 'CompiledPortDataTypes');
            got = '';
            if ~isempty(p) && ~isempty(p.Outport)
                got = p.Outport{1};
            else
                %% a masked library block exposes no CompiledPortDataTypes: the compiled type of its first output
                %% port itself
                ph = get_param([mdl '/' e.block], 'PortHandles');
                if ~isempty(ph.Outport)
                    got = get_param(ph.Outport(1), 'CompiledPortDataType');
                end
            end
            if isempty(got)
                %% no compiled type to compare: the verdict can only be INCOMPLETE
                r.types_unchecked{end + 1} = e.block;
                continue
            end
            if ~strcmp(type_key(got), e.type)
                r.reason = 'COMPILED_TYPE_MISMATCH';
                r.first_diff = struct('block', e.block, 'expected', e.type, 'actual', got);
                feval(mdl, [], [], [], 'term'); compiled = false;
                emit(r); return
            end
            r.types_checked = r.types_checked + 1;
            %% every further output port of the block, each with its own expected type
            ph = get_param([mdl '/' e.block], 'PortHandles');
            for q = 2:numel(ph.Outport)
                want = '';
                for j = 1:numel(pt)
                    f = item(pt, j);
                    if strcmp(f.block, e.block) && f.port == q
                        if used(j)
                            r.reason = 'PORT_SET_MISMATCH';
                            r.first_diff = struct('block', e.block, 'port', q, 'message', 'expected twice');
                            feval(mdl, [], [], [], 'term'); compiled = false;
                            emit(r); return
                        end
                        want = f.type; used(j) = true;
                    end
                end
                gq = get_param(ph.Outport(q), 'CompiledPortDataType');
                if isempty(want) || strcmp(want, 'unknown') || isempty(gq)
                    r.types_unchecked{end + 1} = sprintf('%%s:%%d', e.block, q);
                    continue
                end
                if ~strcmp(type_key(gq), want)
                    r.reason = 'COMPILED_TYPE_MISMATCH';
                    r.first_diff = struct('block', e.block, 'port', q, 'expected', want, 'actual', gq);
                    feval(mdl, [], [], [], 'term'); compiled = false;
                    emit(r); return
                end
                r.types_checked = r.types_checked + 1;
            end
        end
        %% Compare the compiled period of each signal entering a root Outport with the expected period.
        for k = 1:numel(io.outputs)
            o = item(io.outputs, k);
            if ~isfield(o, 'period'), continue; end
            ph = get_param([mdl '/' o.root_block], 'PortHandles');
            lh = get_param(ph.Inport(1), 'Line'); sp = get_param(lh, 'SrcPortHandle');
            cts = get_param(sp, 'CompiledSampleTime');
            if iscell(cts), cts = cts{1}; end
            %% The sampling grid must capture every change in the compiled signal.
            o_off = 0; if isfield(o, 'offset'), o_off = double(o.offset); end
            if ~period_covered(double(cts), double(o.period), o_off)
                r.reason = 'SAMPLE_TIME_MISMATCH';
                r.first_diff = struct('port', o.port, 'expected_period', o.period, 'compiled_period', mat2str(double(cts)));
                feval(mdl, [], [], [], 'term'); compiled = false;
                emit(r); return
            end
        end
        k = find(~used, 1);
        if ~isempty(k)
            %% Each expected port must exist and have a compiled type.
            f = item(pt, k);
            r.reason = 'PORT_SET_MISMATCH';
            r.first_diff = struct('block', f.block, 'port', f.port, 'message', ...
                'an expected output port the block does not have in Simulink');
            feval(mdl, [], [], [], 'term'); compiled = false;
            emit(r); return
        end
        %% Read port properties used for exemptions while the model is compiled.
        te0 = io.type_exempt; ex_ok = false(1, numel(te0));
        for k = 1:numel(te0)
            e = item(te0, k);
            ph = get_param([mdl '/' e.block], 'PortHandles');
            if strcmp(e.reason, 'action signal') && ~isempty(ph.Outport)
                %% a Switch Case drives action ports, whose compiled type is 'action'
                cp = get_param([mdl '/' e.block], 'CompiledPortDataTypes');
                ex_ok(k) = all(strcmp(cp.Outport, 'action'));
            elseif strcmp(e.reason, 'bus signal') && ~isempty(ph.Outport)
                %% a Bus Creator's outputs must compile to a virtual bus
                ex_ok(k) = all(arrayfun(@(h) strcmp(get_param(h, 'CompiledBusType'), 'VIRTUAL_BUS'), ph.Outport));
            end
        end
    catch err
        if compiled, feval(mdl, [], [], [], 'term'); end
        rethrow(err)
    end
    feval(mdl, [], [], [], 'term');
    [wmsg, wid] = lastwarn;
    if ~isempty(wmsg) && allowed_warning(wa, mdl, wid, wmsg)
        %% the predicted one; an earlier warning it may have hidden shows again in the run's WarningDiagnostics
        r.warnings_allowed{end + 1} = wid; wmsg = '';
    end
    if ~isempty(wmsg)
        r.reason = 'COMPILE_WARNING'; r.first_diff = struct('id', wid, 'message', wmsg);
        emit(r); return
    end
    %% 4b. exempt blocks: 'no output port' must be true in Simulink; any other reason leaves the types incomplete
    te = io.type_exempt;
    for k = 1:numel(te)
        e = item(te, k);
        ph = get_param([mdl '/' e.block], 'PortHandles');
        par = get_param([mdl '/' e.block], 'Parent');
        mlfn = strcmp(e.reason, 'MATLAB Function internal') && strcmp(get_param(par, 'SFBlockType'), 'MATLAB Function');
        if ~((strcmp(e.reason, 'no output port') && isempty(ph.Outport)) || mlfn || ex_ok(k))
            r.types_unchecked{end + 1} = e.block;
        end
    end
    %% 5. simulate and compare every root output
    lastwarn('', '');
    so = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
    [wmsg, wid] = lastwarn;
    if ~isempty(wmsg) && allowed_warning(wa, mdl, wid, wmsg)
        %% an allowed run-time warning is reported like a compile-time one
        if ~any(strcmp(r.warnings_allowed, wid)), r.warnings_allowed{end + 1} = wid; end
        wmsg = '';
    end
    try
        %% every warning of the run (compile included), each judged on its own
        wd = so.SimulationMetadata.ExecutionInfo.WarningDiagnostics;
        for q = 1:numel(wd)
            m_ = wd(q).Diagnostic.message; i_ = wd(q).Diagnostic.identifier;
            if allowed_warning(wa, mdl, i_, m_)
                if ~any(strcmp(r.warnings_allowed, i_)), r.warnings_allowed{end + 1} = i_; end
            elseif isempty(wmsg)
                wmsg = m_; wid = i_;
            end
        end
    catch
    end
    if ~isempty(wmsg)
        %% a warning during the run (an Assertion, an overflow diagnostic, ...) is not a PASS
        r.reason = 'SIM_WARNING'; r.first_diff = struct('id', wid, 'message', wmsg);
        emit(r); return
    end
    yout = so.get('yout');
    outs = io.outputs;
    if numElements(yout) ~= io.root_outputs
        r.reason = 'OUTPUT_COUNT_MISMATCH';
        r.first_diff = struct('expected', io.root_outputs, 'actual', numElements(yout));
        emit(r); return
    end
    for k = 1:numel(outs)
        o = item(outs, k);
        el = yout.get(o.index);
        bpath = el.BlockPath.getBlock(1);
        if ~strcmp(bpath, [mdl '/' o.root_block])
            r.reason = 'OUTPUT_ORDER_MISMATCH';
            r.first_diff = struct('index', k, 'expected', [mdl '/' o.root_block], 'actual', bpath);
            emit(r); return
        end
        d = el.Values.Data; tt = el.Values.Time(:);
        %% Read complexity before indexing. Indexing a complex integer array with zero imaginary parts can return a real array.
        dcplx = ~isreal(d);
        if ndims(d) == 3, d = reshape(permute(d, [3 1 2]), size(d, 3), []); end %% w x 1 x n -> n x w
        if size(d, 2) ~= o.width
            r.reason = 'OUTPUT_WIDTH_MISMATCH';
            r.first_diff = struct('port', o.port, 'expected', o.width, 'actual', size(d, 2));
            emit(r); return
        end
        d = d(:, o.element);
        pc = part_code(o);
        if (pc > 0) ~= dcplx
            r.reason = 'OUTPUT_COMPLEXITY_MISMATCH';
            r.first_diff = struct('port', o.port, 'expected_complex', pc > 0, 'actual_complex', dcplx);
            emit(r); return
        end
        if pc == 1, d = real(d); elseif pc == 2, d = imag(d); end
        if strcmp(o.type, 'boolean')
            if ~islogical(d)
                r.reason = 'OUTPUT_TYPE_MISMATCH';
                r.first_diff = struct('port', o.port, 'expected', 'boolean', 'actual', class(d));
                emit(r); return
            end
            got = cellstr(char(double(d) + '0'));
            want = cellstr(o.bins);
        else
            if isinteger(d), d = fi(d); end %% built-in integer classes (uint8, int16, ...) are s,wl,0
            if ~isfi(d)
                r.reason = 'OUTPUT_TYPE_MISMATCH';
                r.first_diff = struct('port', o.port, 'expected', o.type, 'actual', class(d));
                emit(r); return
            end
            gt = sprintf('%%d,%%d,%%d', d.Signed, d.WordLength, d.FractionLength);
            if ~strcmp(gt, o.type)
                r.reason = 'OUTPUT_TYPE_MISMATCH';
                r.first_diff = struct('port', o.port, 'expected', o.type, 'actual', gt);
                emit(r); return
            end
            got = cellstr(bin(d));
            want = cellstr(o.bins);
        end
        want = want(:); got = got(:); wt = double(o.t(:));
        n = min(numel(want), numel(got));
        kd = find(~strcmp(want(1:n), got(1:n)), 1);
        kt = find(abs(wt(1:min(n, numel(tt))) - tt(1:min(n, numel(tt)))) > 1e-9 * max(1, abs(wt(1:min(n, numel(tt))))), 1);
        if ~isempty(kt) && (isempty(kd) || kt <= kd)
            r.reason = 'SAMPLE_TIME_MISMATCH';
            r.first_diff = struct('port', o.port, 'element', o.element, 'sample', kt - 1, 'expected_t', wt(kt), ...
                'actual_t', tt(kt));
            emit(r); return
        end
        if ~isempty(kd)
            r.reason = 'VALUE_MISMATCH';
            r.first_diff = struct('port', o.port, 'element', o.element, 'part', o.part, 'sample', kd - 1, 't', wt(kd), ...
                'expected', item(o.ints, kd), ...
                'expected_bits', want{kd}, 'actual', bits_to_dec(got{kd}, strcmp(o.type(1), '1')), 'actual_bits', got{kd});
            emit(r); return
        end
        if numel(want) ~= numel(got)
            r.reason = 'SAMPLE_COUNT_MISMATCH';
            r.first_diff = struct('port', o.port, 'expected', numel(want), 'actual', numel(got));
            emit(r); return
        end
        r.outputs_checked = r.outputs_checked + 1;
        r.samples_checked = r.samples_checked + numel(want);
    end
    if ~isempty(r.types_unchecked)
        %% every sample agreed, but some compiled types could not be compared: not a full PASS
        r.verdict = 'INCOMPLETE'; r.reason = 'TYPES_UNCHECKED';
    elseif ~r.model_sha256_checked
        %% An unavailable model digest leaves the check incomplete.
        r.verdict = 'INCOMPLETE'; r.reason = 'MODEL_SHA256_UNCHECKED';
    else
        r.verdict = 'PASS';
    end
catch err
    r.verdict = 'FAIL'; r.reason = 'MATLAB_ERROR';
    r.first_diff = struct('id', err.identifier, 'message', err.message);
end
emit(r);
end

function y = period_covered(cts, te, te_off)
%% Return true if the expected sampling grid captures all values at the compiled period and offset.
y = false;
if nargin < 3, te_off = 0; end
if isempty(cts) || ~isfinite(te) || te <= 0, return, end
tc = cts(1); off = 0;
if numel(cts) > 1, off = cts(2); end
if isinf(tc) && tc > 0, y = true; return, end   %% a constant
if ~isfinite(tc) || tc <= 0 || tc < te * (1 - 1e-9), return, end
k = tc / te; q = (off - te_off) / te;
y = abs(k - round(k)) <= 1e-9 * max(1, k) && abs(q - round(q)) <= 1e-9 * max(1, abs(q));
end

function y = same_number(a, b)
%% two numeric literals with the same value ('0.5' and '.5'); never evaluates anything but numeric text
y = false;
pat = '^\s*\[?[-+0-9.eE ,;]*\]?\s*$';
if isempty(regexp(a, pat, 'once')) || isempty(regexp(b, pat, 'once')), return, end
x1 = str2num(a); x2 = str2num(b); %%#ok<ST2NM>
y = ~isempty(x1) && isequal(x1, x2);
end

function c = part_code(o)
%% 0: a real output; 1 / 2: the real / imaginary part of a complex one; -1: not a valid part
c = 0;
if isfield(o, 'part') && ~isempty(o.part)
    c = find(strcmp(o.part, {'re', 'im'}));
    if isempty(c), c = -1; end
end
end

function why = io_incomplete(io)
%% '' when the expectation is complete: fields present, at least one output, every root output covered by exactly
%% its elements 1..width, every element non-empty with as many times and integers as bit patterns, a non-empty
%% type list that together with the exemptions accounts for exactly the design blocks
why = '';
need = {'model_sha256', 'config', 'block_params', 'inputs', 'outputs', 'compiled_types', 'root_outputs', ...
        'design_blocks', 'type_exempt', 'port_types'};
for k = 1:numel(need)
    if ~isfield(io, need{k}), why = ['missing field ' need{k}]; return, end
end
nr = double(io.root_outputs);
if nr < 1 || numel(io.outputs) < 1, why = 'no outputs'; return, end
widths = zeros(1, nr); seen = zeros(1, nr); keys = zeros(1, numel(io.outputs)); cx = -ones(1, nr);
for k = 1:numel(io.outputs)
    o = item(io.outputs, k);
    if o.index < 1 || o.index > nr, why = sprintf('output entry %%d: root index %%d', k, o.index); return, end
    if widths(o.index) == 0, widths(o.index) = o.width; end
    if o.width ~= widths(o.index) || o.element < 1 || o.element > o.width
        why = sprintf('output entry %%d: element %%d of width %%d', k, o.element, o.width); return
    end
    pc = part_code(o);
    if pc < 0 || (cx(o.index) >= 0 && cx(o.index) ~= (pc > 0))
        why = sprintf('output entry %%d: part (a root output is all real, or every element re and im)', k); return
    end
    cx(o.index) = (pc > 0);
    seen(o.index) = seen(o.index) + 1;
    keys(k) = o.index * 1e6 + o.element * 4 + pc;
    nb = numel(cellstr(o.bins));
    if nb < 1 || nb ~= numel(o.t) || nb ~= numel(o.ints)
        why = sprintf('output %%s element %%d: %%d samples, %%d times, %%d integers', o.port, o.element, nb, ...
            numel(o.t), numel(o.ints)); return
    end
end
if numel(unique(keys)) ~= numel(keys), why = 'an output element is listed twice'; return, end
for i = 1:nr
    need = widths(i) * (1 + (cx(i) > 0));
    if widths(i) == 0 || seen(i) ~= need
        why = sprintf('root output %%d: %%d of %%d element parts listed', i, seen(i), need); return
    end
end
if numel(io.compiled_types) < 1, why = 'empty compiled type list'; return, end
typed = cell(1, numel(io.compiled_types));
for k = 1:numel(io.compiled_types), e = item(io.compiled_types, k); typed{k} = e.block; end
ex = cell(1, numel(io.type_exempt));
for k = 1:numel(io.type_exempt), e = item(io.type_exempt, k); ex{k} = e.block; end
db = cellstr(io.design_blocks);
if numel(unique(typed)) ~= numel(typed), why = 'a block is typed twice'; return, end
pk = cell(1, numel(io.port_types));
for k = 1:numel(io.port_types)
    e = item(io.port_types, k); pk{k} = sprintf('%%s#%%d', e.block, e.port);
    if e.port < 2 || ~any(strcmp(typed, e.block))
        why = 'port types: a port below 2, or a block without its first-port type'; return
    end
end
if numel(unique(pk)) ~= numel(pk), why = 'port types: a port listed twice'; return, end
if ~isempty(setxor(union(typed, ex), db)) || ~isempty(intersect(typed, ex))
    why = 'the type list and its exemptions do not account for exactly the design blocks'; return
end
end

function [why, d] = package_digest(pkgdir, mdl)
%% '' when manifest.json gives exactly one sha256 for the model file and one for the io file (in its "files" object)
%% and each file hashes to it; 'PACKAGE_DIGEST_MISMATCH' (a file or digest missing, or a different hash) or
%% 'PACKAGE_DIGEST_UNCHECKED' (a file that could not be hashed) with the details in d
why = ''; d = [];
mf = fullfile(pkgdir, 'manifest.json');
if ~isfile(mf)
    why = 'PACKAGE_DIGEST_MISMATCH'; d = struct('file', 'manifest.json', 'message', 'no manifest.json'); return
end
blk = regexp(fileread(mf), '"files"\s*:\s*\{([^{}]*)\}', 'tokens', 'once');
if isempty(blk)
    why = 'PACKAGE_DIGEST_MISMATCH'; d = struct('file', 'manifest.json', 'message', 'no "files" object'); return
end
fl = {[mdl '.slx'], [mdl '_io.json']};
for k = 1:numel(fl)
    tok = regexp(blk{1}, ['"' regexptranslate('escape', fl{k}) '"\s*:\s*"([0-9a-f]{64})"'], 'tokens');
    if numel(tok) ~= 1
        why = 'PACKAGE_DIGEST_MISMATCH';
        d = struct('file', fl{k}, 'message', 'manifest.json gives no single sha256 for this file'); return
    end
    f = fullfile(pkgdir, fl{k});
    if ~isfile(f)
        why = 'PACKAGE_DIGEST_MISMATCH'; d = struct('file', fl{k}, 'message', 'file missing'); return
    end
    [ok, h] = sha256_of(f);
    if ~ok
        why = 'PACKAGE_DIGEST_UNCHECKED'; d = struct('file', fl{k}, 'message', 'the file could not be hashed'); return
    end
    if ~strcmp(h, tok{1}{1})
        why = 'PACKAGE_DIGEST_MISMATCH'; d = struct('file', fl{k}, 'expected', tok{1}{1}, 'actual', h); return
    end
end
end

function why = io_inconsistent(io)
%% '' when every input and output sample's decimal integer (ints; an input's nums too, when given) is the value of
%% its bit pattern (bins) in the declared type: a boolean takes '0' / '1', a fixed-point type wl characters 0 / 1
%% read as two's complement when signed
why = '';
for k = 1:numel(io.inputs)
    u = item(io.inputs, k);
    if isfield(u, 'type') && strcmp(u.type, 'boolean')
        s = -1; wl = 1;
    else
        s = double(u.s); wl = double(u.wl);
    end
    nums = []; if isfield(u, 'nums'), nums = u.nums; end
    why = samples_inconsistent(u.bins, u.ints, nums, s, wl);
    if ~isempty(why), why = sprintf('input %%s: %%s', u.port, why); return, end
end
for k = 1:numel(io.outputs)
    o = item(io.outputs, k);
    if strcmp(o.type, 'boolean')
        s = -1; wl = 1;
    else
        tv = sscanf(o.type, '%%d,%%d,%%d');
        if numel(tv) ~= 3 || ~any(tv(1) == [0 1]) || tv(2) < 1
            why = sprintf('output %%s element %%d: type %%s', o.port, o.element, o.type); return
        end
        s = tv(1); wl = tv(2);
    end
    why = samples_inconsistent(o.bins, o.ints, [], s, wl);
    if ~isempty(why), why = sprintf('output %%s element %%d %%s: %%s', o.port, o.element, o.part, why); return, end
end
end

function why = samples_inconsistent(bins, ints, nums, s, wl)
%% s: 1 signed, 0 unsigned, -1 boolean; wl: bits per pattern
why = '';
if isempty(bins), b = {}; else, b = cellstr(bins); end
if isempty(ints), d = {}; else, d = cellstr(ints); end
b = b(:); d = d(:); n = numel(b);
if numel(d) ~= n, why = sprintf('%%d bit patterns, %%d integers', n, numel(d)); return, end
if ~isempty(nums) && numel(nums) ~= n, why = sprintf('%%d bit patterns, %%d numbers', n, numel(nums)); return, end
if n == 0, return, end
if any(cellfun(@numel, b) ~= wl), why = sprintf('a bit pattern that is not %%d bits', wl); return, end
B = char(b);
if ~all(B(:) == '0' | B(:) == '1'), why = 'a bit pattern with a character other than 0 / 1'; return, end
if s < 0
    txt = b;
elseif wl <= 53
    %% exact in doubles: every partial sum is an integer below 2^53
    v = double(B == '1') * (2 .^ (wl - 1:-1:0))';
    if s == 1, v = v - double(B(:, 1) == '1') * 2^wl; end
    txt = arrayfun(@(x) sprintf('%%.0f', x), v, 'UniformOutput', false);
else
    txt = cellfun(@(x) bits_to_dec(x, s == 1), b, 'UniformOutput', false);
end
kd = find(~strcmp(txt(:), d), 1);
if ~isempty(kd)
    why = sprintf('sample %%d: bits %%s are %%s, the integer says %%s', kd - 1, b{kd}, txt{kd}, d{kd}); return
end
if ~isempty(nums)
    nt = arrayfun(@(x) sprintf('%%.0f', x), double(nums(:)), 'UniformOutput', false);
    kd = find(~strcmp(nt(:), d), 1);
    if ~isempty(kd)
        why = sprintf('sample %%d: the number %%s, the integer %%s', kd - 1, nt{kd}, d{kd}); return
    end
end
end

function ok = allowed_warning(wa, mdl, wid, wmsg)
%% a warning the package lists (id and blocks) whose message names one of those blocks
ok = false;
for k = 1:numel(wa)
    e = item(wa, k);
    if ~strcmp(e.id, wid), continue, end
    %% a warning whose message names no block (HDL FIFO push full / pop empty) -- by its id, listed
    %% only for the designs that hold such a block
    if isfield(e, 'id_only') && isequal(e.id_only, true), ok = true; return, end
    bl = e.blocks; if ischar(bl), bl = {bl}; end
    for j = 1:numel(bl)
        b = bl{j}; if iscell(b), b = b{1}; end
        if contains(wmsg, [mdl '/' b '/']) || contains(wmsg, [mdl '/' b ':']) || contains(wmsg, [mdl '/' b '''']) ...
                || contains(wmsg, [mdl '/' b '<'])
            ok = true; return
        end
        %% a MATLAB Function's constant-folding warning names the function by its block name only
        lf = regexp(b, '[^/]+$', 'match', 'once');
        if strcmp(wid, 'Coder:builtins:ConstantFoldingOverFlow') && (contains(wmsg, ['"' lf '"']) ...
                || contains(wmsg, ['"' b '"']))   %% 'function "DUT/F"' on some platforms
            ok = true; return
        end
    end
end
end

function x = item(a, k)
if iscell(a), x = a{k}; elseif isstruct(a), x = a(k); else, x = a(k); end
if iscell(x) && numel(x) == 1, x = x{1}; end
end

function key = type_key(t)
%% Simulink compiled type name -> 's,w,f' (or 'boolean'); anything else is returned as 'other:<name>'.
if strcmp(t, 'boolean'), key = 'boolean'; return, end
tok = regexp(t, '^(s|u)fix(\d+)$', 'tokens', 'once');
if ~isempty(tok), key = sprintf('%%d,%%s,0', strcmp(tok{1}, 's'), tok{2}); return, end
tok = regexp(t, '^(s|u)fix(\d+)_(En|E)(\d+)$', 'tokens', 'once');
if ~isempty(tok)
    f = str2double(tok{4}); if strcmp(tok{3}, 'E'), f = -f; end
    key = sprintf('%%d,%%s,%%d', strcmp(tok{1}, 's'), tok{2}, f); return
end
tok = regexp(t, '^(u?)int(\d+)$', 'tokens', 'once');
if ~isempty(tok), key = sprintf('%%d,%%s,0', isempty(tok{1}), tok{2}); return, end
key = ['other:' t];
end

function s = bits_to_dec(b, signed)
%% exact decimal text of a two's complement bit pattern (any width)
neg = signed && b(1) == '1';
if neg, b = char('0' + ('1' - b)); end %% one's complement, +1 below
v = 0; %% little-endian base-1e9 limbs
for c = b
    carry = (c == '1');
    for j = 1:numel(v)
        x = v(j) * 2 + carry; v(j) = mod(x, 1e9); carry = floor(x / 1e9);
    end
    if carry, v(end + 1) = carry; end %%#ok<AGROW>
end
if neg
    carry = 1;
    for j = 1:numel(v)
        x = v(j) + carry; v(j) = mod(x, 1e9); carry = floor(x / 1e9);
    end
    if carry, v(end + 1) = carry; end
end
s = sprintf('%%d', v(end));
for j = numel(v) - 1:-1:1, s = [s sprintf('%%09d', v(j))]; end %%#ok<AGROW>
if neg, s = ['-' s]; end
end

function [ok, h] = sha256_of(f)
%% Use Java for SHA-256 when available. Validate the MATLAB fallback against both FIPS 180-2 test vectors before use.
ok = false; h = '';
try
    fid = fopen(f, 'r'); data = fread(fid, inf, '*uint8'); fclose(fid);
catch
    return
end
try
    md = java.security.MessageDigest.getInstance('SHA-256');
    md.update(typecast(data, 'int8'));
    d = typecast(md.digest(), 'uint8');
    h = lower(reshape(dec2hex(d, 2)', 1, [])); ok = true; return
catch
end
try
    if ~strcmp(sha256_plain(uint8('abc')), ...
               'ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad') || ...
       ~strcmp(sha256_plain(uint8('abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq')), ...
               '248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1')
        return
    end
    h = sha256_plain(data); ok = true;
catch
    h = '';
end
end

function h = sha256_plain(m)
K = uint32(hex2dec({'428a2f98' '71374491' 'b5c0fbcf' 'e9b5dba5' '3956c25b' '59f111f1' '923f82a4' 'ab1c5ed5' ...
    'd807aa98' '12835b01' '243185be' '550c7dc3' '72be5d74' '80deb1fe' '9bdc06a7' 'c19bf174' ...
    'e49b69c1' 'efbe4786' '0fc19dc6' '240ca1cc' '2de92c6f' '4a7484aa' '5cb0a9dc' '76f988da' ...
    '983e5152' 'a831c66d' 'b00327c8' 'bf597fc7' 'c6e00bf3' 'd5a79147' '06ca6351' '14292967' ...
    '27b70a85' '2e1b2138' '4d2c6dfc' '53380d13' '650a7354' '766a0abb' '81c2c92e' '92722c85' ...
    'a2bfe8a1' 'a81a664b' 'c24b8b70' 'c76c51a3' 'd192e819' 'd6990624' 'f40e3585' '106aa070' ...
    '19a4c116' '1e376c08' '2748774c' '34b0bcb5' '391c0cb3' '4ed8aa4a' '5b9cca4f' '682e6ff3' ...
    '748f82ee' '78a5636f' '84c87814' '8cc70208' '90befffa' 'a4506ceb' 'bef9a3f7' 'c67178f2'}))';
H = uint32(hex2dec({'6a09e667' 'bb67ae85' '3c6ef372' 'a54ff53a' '510e527f' '9b05688c' '1f83d9ab' '5be0cd19'}))';
m = reshape(uint8(m), 1, []);
n = numel(m);
lb = zeros(1, 8, 'uint8'); nb = n * 8;
for i = 8:-1:1, lb(i) = uint8(mod(nb, 256)); nb = floor(nb / 256); end
m = [m uint8(128) zeros(1, mod(55 - n, 64), 'uint8') lb];
q = reshape(double(m), 4, [])';
W = uint32(q(:, 1) * 16777216 + q(:, 2) * 65536 + q(:, 3) * 256 + q(:, 4))';
for blk = 1:numel(W) / 16
    w = zeros(1, 64, 'uint32');
    w(1:16) = W((blk - 1) * 16 + (1:16));
    for t = 17:64
        s0 = bitxor(bitxor(rotr32(w(t - 15), 7), rotr32(w(t - 15), 18)), bitshift(w(t - 15), -3));
        s1 = bitxor(bitxor(rotr32(w(t - 2), 17), rotr32(w(t - 2), 19)), bitshift(w(t - 2), -10));
        w(t) = add32([w(t - 16); s0; w(t - 7); s1]);
    end
    a = H(1); b = H(2); c = H(3); d = H(4); e = H(5); f = H(6); g = H(7); hh = H(8);
    for t = 1:64
        S1 = bitxor(bitxor(rotr32(e, 6), rotr32(e, 11)), rotr32(e, 25));
        ch = bitxor(bitand(e, f), bitand(bitcmp(e), g));
        t1 = add32([hh; S1; ch; K(t); w(t)]);
        S0 = bitxor(bitxor(rotr32(a, 2), rotr32(a, 13)), rotr32(a, 22));
        mj = bitxor(bitxor(bitand(a, b), bitand(a, c)), bitand(b, c));
        t2 = add32([S0; mj]);
        hh = g; g = f; f = e; e = add32([d; t1]); d = c; c = b; b = a; a = add32([t1; t2]);
    end
    H = add32([H; a b c d e f g hh]);
end
h = lower(reshape(dec2hex(H, 8)', 1, []));
end

function y = rotr32(x, n)
y = bitor(bitshift(x, -n), bitshift(x, 32 - n));
end

function y = add32(x)
%% column sums modulo 2^32 (every call stacks its terms in rows) (doubles are exact far beyond 5 * 2^32)
y = uint32(mod(sum(double(x), 1), 4294967296));
end

function extra = dut_extra(mdl, io)
%% every block under the DUT is a design block, or a plain subsystem holding design blocks (walked into), or the
%% Inport / Outport of such a nested subsystem (its structure: the exporter lists the blocks inside, not its ports).
%% blocks under a mask and commented-out blocks are seen too, the DUT carries no mask (the exporter
%% writes none), and the model root holds only the DUT, the From Workspace sources and the root Outports.
db = cellfun(@(x) [mdl '/' x], cellstr(io.design_blocks), 'UniformOutput', false);
db = db(:)';
top = [mdl '/' io.dut];
extra = {};
rt = find_system(mdl, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'IncludeCommented', 'on');
for k = 1:numel(rt)
    b = rt{k};
    if strcmp(b, mdl) || strcmp(b, top), continue, end
    if ~any(strcmp(get_param(b, 'BlockType'), {'FromWorkspace', 'Outport'})) || ~strcmp(get_param(b, 'Commented'), 'off')
        extra{end + 1} = b; %%#ok<AGROW>
    end
end
if strcmp(get_param(top, 'Mask'), 'on')
    extra{end + 1} = [top ' (mask)'];
end
queue = {top};
while ~isempty(queue)
    s = queue{1}; queue(1) = [];
    ch = find_system(s, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'IncludeCommented', 'on');
    for k = 1:numel(ch)
        b = ch{k};
        if strcmp(b, s), continue, end
        if ~strcmp(get_param(b, 'Commented'), 'off')
            extra{end + 1} = b; %%#ok<AGROW>
            continue
        end
        if any(strcmp(db, b)), continue, end
        bt = get_param(b, 'BlockType');
        if strcmp(bt, 'SubSystem') && any(strncmp(db, [b '/'], numel(b) + 1))
            queue{end + 1} = b; %%#ok<AGROW>
        elseif ~strcmp(s, top) && any(strcmp(bt, {'Inport', 'Outport'}))
            continue
        else
            extra{end + 1} = b; %%#ok<AGROW>
        end
    end
end
end

function [bad, n, why] = library_defaults(io)
%% io.library_defaults: {block, lib, param, value}; a scratch model holds one fresh block per library path
bad = []; n = 0; why = [];
if ~isfield(io, 'library_defaults') || isempty(io.library_defaults), return, end
ld = io.library_defaults;
tmp = 'fxexport_libdefaults_scratch';
if bdIsLoaded(tmp), close_system(tmp, 0); end
new_system(tmp);
c = onCleanup(@() close_system(tmp, 0));
made = containers.Map();
for k = 1:numel(ld)
    e = item(ld, k);
    if ~isKey(made, e.lib)
        nb = [tmp '/b' num2str(made.Count + 1)];
        try
            load_system(strtok(e.lib, '/'));
            add_block(e.lib, nb);
        catch err
            why = struct('message', 'cannot place the library block', 'lib', e.lib, 'error', err.message);
            return
        end
        made(e.lib) = nb;
    end
    try
        lv = get_param(made(e.lib), e.param);
    catch err
        why = struct('message', 'the library block has no such parameter', 'lib', e.lib, 'param', e.param, ...
                     'error', err.message);
        return
    end
    if isnumeric(lv), lv = num2str(lv); end
    if ~ischar(lv)
        why = struct('message', 'the library value is not text', 'lib', e.lib, 'param', e.param);
        return
    end
    a = strtrim(lv); b = strtrim(e.value);
    if strcmp(e.param, 'Inputs'), a = strrep(a, '|', ''); b = strrep(b, '|', ''); end   %% '|' is icon layout only
    if ~strcmp(a, b) && ~same_number(a, b)
        bad = struct('block', e.block, 'param', e.param, 'package', e.value, 'library', lv, 'lib', e.lib);
        return
    end
    n = n + 1;
end
lastwarn('', '');   %% placing library blocks must not leave a warning for the simulation check
end

function cleanup(mdl)
try, feval(mdl, [], [], [], 'term'); catch, end
try, close_system(mdl, 0); catch, end
end

function restore_filegen(fgc, fgc_default, gen, mdl)
%% the model is closed first (its build artefacts live in gen), then the folders are restored (or reset to the session
%% default when they were the default) and gen removed
try, feval(mdl, [], [], [], 'term'); catch, end
try, close_system(mdl, 0); catch, end
if fgc_default
    try, Simulink.fileGenControl('reset'); catch, end
else
    try, Simulink.fileGenControl('setConfig', 'config', fgc); catch, end
end
try, rmdir(gen, 's'); catch, end
end

function clear_vars(vars)
for k = 1:numel(vars), evalin('base', ['clear ' vars{k}]); end
end

function emit(r)
fprintf('%%s\n', jsonencode(r));
end
"""


def matlab_script(model):
    return MATLAB_TEMPLATE % {"fn": "check_" + model, "model": model, "fmt": FORMAT_IO}


# ================================================================ export = engine run + model + roundtrip
def _observe(design, names):
    """Add one Outport per observed block (request field "observe"): internal signals become root outputs of the
    exported model so that the check script compares them too. Returns (design, {outport: block})."""
    if not isinstance(names, list) or not all(isinstance(x, str) for x in names):
        raise ExportError("BAD_PARAM", "observe", "list of block names")
    if not names:
        return design, {}
    blocks = dict(design["blocks"])
    lines = list(design.get("lines", []))
    added = {}
    for n in names:
        if n not in blocks:
            raise ExportError("BAD_PARAM", "observe", "no block %r" % n)
        if blocks[n].get("type") == "Outport":
            continue
        o = n.replace("/", "_") + "_obs"
        while o in blocks:
            o += "_"
        blocks[o] = {"type": "Outport"}
        lines.append({"src": n, "dst": o, "dport": 1})
        added[o] = n
    # every other design field (port_types, the reader's port constraints, ...) is kept -- the
    # observed design is the same design with more outputs, never a less constrained one
    return dict(design, blocks=blocks, lines=lines), added


def _width(b):
    v = str(b.get("PortDimensions", "1")).strip().strip("[]").strip()
    try:
        w = int(v)
    except ValueError:
        raise ExportError("UNSUPPORTED_EXPORT", "PortDimensions", "port dimensions %r (a scalar or a vector width)" % v)
    return 1 if w == -1 else w


def _strip_index(n):
    m = re.match(r"^(.*)\[(\d+)\]$", n)
    return m.group(1) if m else n


def output_summary(io_obj, outs):
    """The one per-port output summary of a package (manifest.json outputs[] and the fxexport result's outputs[]):
    per root output port its name, root Outport block, width, sample count, compiled type and the clock the check
    expects it on -- period and offset in model-time units, taken from the io file's own entries for that port (one
    entry per element, all on the same clock), so the manifest, the CLI result and <model>_io.json agree."""
    rows = []
    for n in outs:
        es = [o for o in io_obj["outputs"] if o["port"] == n]
        clocks = {(o["period"], o["offset"]) for o in es}
        if len(clocks) != 1:
            raise ExportError("INTERNAL", "design.blocks." + n, "output elements on different clocks %s" % sorted(clocks))
        types = {o["type"] for o in es}
        rows.append({"port": n, "root_block": es[0]["root_block"], "width": len(es),   # entries: a complex port counts its two parts
                     "samples": len(es[0]["bins"]),
                     "type": types.pop() if len(types) == 1 else "mixed",
                     "period": es[0]["period"], "offset": es[0]["offset"]})
    return rows


def io_incomplete(io):
    """Python mirror of the check script's io_incomplete: '' when the expectation is complete, else
    the reason. Used on every package before it is returned, and by the negative-package tests."""
    need = ("model_sha256", "config", "block_params", "inputs", "outputs", "compiled_types", "root_outputs",
            "design_blocks", "type_exempt", "port_types")
    for k in need:
        if k not in io:
            return "missing field " + k
    nr = int(io["root_outputs"])
    if nr < 1 or not io["outputs"]:
        return "no outputs"
    widths, seen, keys, cx = {}, {}, set(), {}
    for k, o in enumerate(io["outputs"]):
        if not 1 <= int(o["index"]) <= nr:
            return "output entry %d: root index %s" % (k + 1, o["index"])
        w = widths.setdefault(o["index"], o["width"])
        if o["width"] != w or not 1 <= o["element"] <= o["width"]:
            return "output entry %d: element %s of width %s" % (k + 1, o["element"], o["width"])
        part = o.get("part", "")
        if part not in ("", "re", "im") or cx.setdefault(o["index"], part != "") != (part != ""):
            return "output entry %d: part %r (a root output is all real, or every element re and im)" % (k + 1, part)
        if (o["index"], o["element"], part) in keys:
            return "an output element is listed twice"
        keys.add((o["index"], o["element"], part))
        seen[o["index"]] = seen.get(o["index"], 0) + 1
        nb = len(o["bins"])
        if nb < 1 or nb != len(o["t"]) or nb != len(o["ints"]):
            return "output %s element %s: %d samples, %d times, %d integers" % (o["port"], o["element"], nb,
                                                                                len(o["t"]), len(o["ints"]))
    for i in range(1, nr + 1):
        need = widths.get(i, 0) * (2 if cx.get(i) else 1)
        if not widths.get(i) or seen.get(i) != need:
            return "root output %d: %d of %s element parts listed" % (i, seen.get(i, 0), need)
    if not io["compiled_types"]:
        return "empty compiled type list"
    typed = [e["block"] for e in io["compiled_types"]]
    ex = [e["block"] for e in io["type_exempt"]]
    if len(set(typed)) != len(typed):
        return "a block is typed twice"
    pk = [(e["block"], int(e["port"])) for e in io["port_types"]]
    if len(set(pk)) != len(pk) or any(p < 2 for _b, p in pk) or not {b for b, _p in pk} <= set(typed):
        return "port types: a port listed twice, a port below 2, or a block without its first-port type"
    if (set(typed) | set(ex)) != set(io["design_blocks"]) or set(typed) & set(ex):
        return "the type list and its exemptions do not account for exactly the design blocks"
    return ""


def _engine_semantics_open(design, inputs):
    """Configurations whose engine semantics are not recorded (matrix Gain): exporting them would hand
    Simulink a model that disagrees with the expected values, and the roundtrip cannot see it (the read-back uses the
    same semantics). The engine refuses them too since a previous version; kept here as a second guard."""
    periods = set()   # anything that can give an inherited period other than 1
    for n, b in design.get("blocks", {}).items():
        if not isinstance(b, dict):
            continue
        if b.get("type") == "Inport" and str(b.get("SampleTime", "-1")).strip() not in ("-1", "1", "[1 0]", "[1,0]"):
            periods.add("%s %s" % (n, b.get("SampleTime")))
        if b.get("type") in G.MR_BLOCKS:
            periods.add("%s (%s)" % (n, b.get("type")))
    _ = inputs
    # a bus into a root Outport -- Simulink logs a structure there (the check script reads arrays and
    # stopped with nonExistentField 'Data'); not exported
    blocks_ = design.get("blocks", {})
    src_ = {(l.get("dst"), int(l.get("dport", 1))): l.get("src") for l in design.get("lines", []) if isinstance(l, dict)}

    def _is_bus(x, depth=0):
        bx = blocks_.get(x) if isinstance(blocks_.get(x), dict) else {}
        t_ = bx.get("type")
        if depth > 100:
            return False
        if t_ == "BusCreator" or (t_ == "BusSelector" and str(bx.get("OutputAsBus", "off")) == "on"):
            return True
        if t_ in ("UnitDelay", "BusAssignment"):
            return _is_bus(src_.get((x, 1)), depth + 1)
        return False
    for n, b in blocks_.items():
        if isinstance(b, dict) and b.get("type") == "Outport" and "/" not in n and _is_bus(src_.get((n, 1))):
            raise ExportError("UNSUPPORTED_EXPORT", "design.blocks." + n, "a bus into the root Outport %s: Simulink "
                              "logs a structure there, which the package check does not read; select the elements "
                              "with a Bus Selector, one Outport each" % n)
    for n, b in design.get("blocks", {}).items():
        if not isinstance(b, dict):
            continue
        t = b.get("type")
        # recordings (Discrete-Time Integrator period): fixed in the engine (K*T;
        # other periods UNSUPPORTED_RATE on every path), so the export follows the engine again
        # a previous version answers explicit DTI output types, so a DTI with a period of its own now reaches the export;
        # the exported single-rate model would carry it and the project's reader refuses that (multi-rate model):
        # refused here, the engine's answer is not exportable as a single-rate model
        if t == "DiscreteIntegrator" and str(b.get("SampleTime", "-1")).strip() not in ("-1", "1", "[1 0]", "[1,0]"):
            raise ExportError("UNSUPPORTED_EXPORT", "design.blocks.%s.SampleTime" % n, "a Discrete-Time Integrator "
                              "with its own period %s: not exported (give SampleTime -1 to inherit the input's rate)"
                              % b.get("SampleTime"))
        # For an explicit-output matrix Gain, round each product into the output type.
        if str(b.get("DiagnosticForOutOfRangeInput", "None")).strip() == "Warning" and t in (
                "LookupND", "Lookup1D", "Lookup2D", "PreLookup", "Interpolation_n-D", "Prelookup",
                "InterpolationUsingPrelookup"):   # (the Direct Lookup Table exports with it: recordings)
            # Simulink warns when an input leaves the breakpoints; the warning is not recorded, so the
            # package check cannot allow it and prints FAIL (SIM_WARNING)
            raise ExportError("UNSUPPORTED_EXPORT", "design.blocks.%s.DiagnosticForOutOfRangeInput" % n, "a lookup "
                              "with DiagnosticForOutOfRangeInput 'Warning' is not exported (Simulink warns for an "
                              "input outside the breakpoints; that warning is not recorded); use 'None'")


def _resolve_ram_widths(design, hw, inputs):
    """A System RAM with AddressBits 'Inherit: from address' takes its write address's word length; the engine
    resolves it at compile time (engine/fxhdl.py ram_auto_resolve) and runs it exactly as the masked RAM of that
    width (both kinds alike), so the export writes that RAM with the width given. Returns (design,
    {block: width}); a vector design or an address the engine cannot type keeps the explicit refusal below."""
    import fxhdl
    design, resolved = _sysram_for_init(design, hw, inputs)
    auto = [n for n, b in design["blocks"].items() if isinstance(b, dict) and b.get("type") == "HDLRAM"
            and str(b.get("AddressBits", "")).strip() == "Inherit: from address" and not _needs_sysram(b)]
    if not auto or any("[" in k for k in inputs):
        return design, resolved
    try:
        types = {k: _in_type(v, "inputs." + k) for k, v in inputs.items()}
        out = fxhdl.ram_auto_resolve(design, hw, types)
    except (G.GraphError, ExportError, KeyError, TypeError):
        return design, resolved   # the engine reports it when the design is run
    # only the RAMs written in the masked form take the resolved width; a RAM that needs the System form
    # (Old data, a non-zero initial value) keeps 'Inherit: from address' (it was resolved along with the others and
    # then refused, with advice the design already followed)
    blocks = dict(out["blocks"])
    for n, b in design["blocks"].items():
        if isinstance(b, dict) and b.get("type") == "HDLRAM" and n not in auto and n in blocks:
            blocks[n] = b
    out = dict(out, blocks=blocks)
    return out, dict(resolved, **{n: out["blocks"][n]["AddressBits"] for n in auto})


SFIX_BITLESS = "SimulinkFixedPoint:util:sfix_dtpropBitLess"


DEMUX_NONUNIFORM = "Simulink:blocks:MuxNonuniformDistribution"


def _demux_uneven(design, res):
    """a Demux whose output count does not divide its input width (the recorded 4 into 3,
    recordings) is answered like Simulink, which warns while compiling (id from recordings); allowed for it."""
    import fxvector as V
    out = []
    for n, b in design["blocks"].items():
        if not isinstance(b, dict) or b.get("type") != "Demux":
            continue
        txt = str(b.get("Outputs", "2")).strip()
        if txt.startswith("["):
            continue
        src = next((ln for ln in design.get("lines", []) if ln["dst"] == n), None)
        w = None
        if src is not None:
            ks = [k for k in (res.get("compiled_types") or {}) if k.startswith(src["src"] + "[")]
            w = len(ks) or None
        try:
            k = int(float(txt))
        except ValueError:
            continue
        if w and w % k and (w, k) in V.DEMUX_UNEVEN:
            out.append(n)
    return [{"id": DEMUX_NONUNIFORM, "blocks": sorted(out)}] if out else []


SWITCH_IGNORES_THRESHOLD = "Simulink:blocks:SwitchIgnoringThreshold"


def _switch_bool_control(design, res):
    """recordings: a Switch whose control (port 2) is boolean ignores its criterion and threshold -- input 1 when the
    control is true, input 3 when false -- and Simulink warns so while compiling (recorded identifier, 9 / 9 packages
    equal in value). The engine answers the same values; the package allows that warning for these blocks only."""
    ct = res.get("compiled_types") or {}
    out = []
    for n, b in design["blocks"].items():
        if not isinstance(b, dict) or b.get("type") != "Switch":
            continue
        if str(b.get("Criteria", BS.DEFAULTS.get("Switch", {}).get("Criteria", ""))).strip() == "u2 ~= 0":
            continue
        src = [ln for ln in design.get("lines", []) if ln["dst"] == n and int(ln.get("dport", 1)) == 2]
        if len(src) == 1 and int(src[0].get("sport", 1)) == 1 and ct.get(src[0]["src"]) == "boolean":
            out.append(n)
    return [{"id": SWITCH_IGNORES_THRESHOLD, "blocks": sorted(out)}] if out else []


MPS_OUT_OF_RANGE = "Simulink:blocks:MPSwitchControlInputOORIndexValue"


def _engine_expected_warnings(design, res):
    """a previous version: fxgraph's simulink_warnings.expected lists the warnings Simulink gives
    on this design in the recorded shapes (SwitchIgnoringThreshold, MPSwitchDeadPorts,
    MPSwitchControlInputOORIndexValue); the package allows each for its blocks. Unchecked shapes are not allowed: a
    warning there still fails the check (SIM_WARNING)."""
    by = {}
    for e in ((res.get("simulink_warnings") or {}).get("expected") or []):
        b = str(e.get("block", "")).split("\x1d")[0].split("[")[0]
        if e.get("id") and b in (design.get("blocks") or {}):
            by.setdefault(e["id"], set()).add(b)
    return [{"id": i, "blocks": sorted(bs)} for i, bs in sorted(by.items())]


FIFO_WARNINGS = ("hdlsllib:hdlsllib:FIFOPushFull", "hdlsllib:hdlsllib:FIFOPopEmpty")


def _fifo_warnings(design, res):
    """an HDL FIFO with push_msg / pop_msg 'Warning' (the library default) warns at a push
    into a full FIFO and a pop from an empty one -- hdlsllib:hdlsllib:FIFOPushFull / FIFOPopEmpty, whose message names
    no block ('attempt to add to a full FIFO'), each with Simulink:blocks:AssertionAssert from the FIFO's own
    Diagnostics/Assertion block (named in its message). The values are the same as with Ignore. Allowed
    for these blocks only: the two FIFO ids by id, the assertion by the block path."""
    out = []
    for n, b in design["blocks"].items():
        if isinstance(b, dict) and b.get("type") == "HDLFIFO" and \
                (str(b.get("push_msg", "Warning")).strip(), str(b.get("pop_msg", "Warning")).strip()) == ("Warning",
                                                                                                         "Warning"):
            out.append(n)
    if not out:
        return []
    return [{"id": i, "blocks": sorted(out), "id_only": True} for i in FIFO_WARNINGS] + \
        [{"id": "Simulink:blocks:AssertionAssert", "blocks": sorted(out)}]


def _mps_default_warning(design, res):
    """a Multiport Switch with DiagnosticForDefault 'Warning' takes its default port on an out-of-range
    control, and Simulink warns at run time (6 packages FAILed SIM_WARNING, values equal). The id is the one Simulink
    gives with 'Error'; allowed for these blocks only (a wrong id
    still FAILs: values are compared either way)."""
    out = []
    for n, b in design["blocks"].items():
        if isinstance(b, dict) and b.get("type") == "MultiportSwitch" and \
                str(b.get("DiagnosticForDefault", "")).strip() == "Warning":
            out.append(n)
    return [{"id": MPS_OUT_OF_RANGE, "blocks": sorted(out)}] if out else []


ML_CONST_FOLD = "Coder:builtins:ConstantFoldingOverFlow"
DS_PRIORITY = "Simulink:Engine:UnableToHonorPriorityInTask0"


def _ds_priority_warning(design, res):
    """recordings: a Data Store whose write has the lower Priority although a data path
    leads from its read to its write -- Simulink follows the data path and warns UnableToHonorPriorityInTask0 while
    compiling (the engine follows the data path too, fxaction). Allowed for those read / write blocks."""
    import fxaction
    blocks, lines = design["blocks"], design.get("lines", [])
    out = []
    by = {}
    for n, b in blocks.items():
        if isinstance(b, dict) and b.get("type") in ("DataStoreRead", "DataStoreWrite"):
            by.setdefault(str(b.get("DataStoreName", "A")), {})[b["type"]] = n
    for parts in by.values():
        r, w = parts.get("DataStoreRead"), parts.get("DataStoreWrite")
        if not (r and w) or not fxaction._reaches(lines, r, w):
            continue
        try:
            if int(blocks[w].get("Priority", "")) < int(blocks[r].get("Priority", "")):
                out += [r, w]
        except ValueError:
            continue
    return [{"id": DS_PRIORITY, "blocks": sorted(out)}] if out else []


def _ml_const_fold_overflow(design, res):
    """a MATLAB Function with fi(c, s, wl, fl) of a literal c outside its type (fi(-3, 1, 12, 10)) is
    folded to the saturated value -- recorded (fi(10, 1, 8, 4) = 127 every
    step) -- but Simulink warns while compiling (id from native verification packages; with some fimath settings it does not
    warn), and the check failed on it (3 packages). Allowed for these blocks only; values are compared either
    way. The warning names the function by its block name ('function "MF"')."""
    import math
    import fxmlfn as P
    out = []
    for n, b in design["blocks"].items():
        if not isinstance(b, dict) or b.get("type") != "MATLABFunction" or not isinstance(b.get("Script"), str):
            continue
        try:
            ast = P.parse(b["Script"])
        except P.MlError:
            continue
        for e in P._nodes(ast):
            a = e.get("args") or []
            if e.get("op") != "call" or e.get("name") != "fi" or len(a) < 4:
                continue
            neg = a[0].get("op") == "neg" and (a[0].get("args") or [{}])[0].get("op") == "num"
            if not (a[0].get("op") == "num" or neg) or any(x.get("op") != "num" for x in a[1:4]):
                continue
            try:
                c = P.num_value((a[0]["args"][0] if neg else a[0])["text"])
                sg, wl, fl = (int(Fraction(x["text"])) for x in a[1:4])
            except (TypeError, ValueError, ZeroDivisionError):
                continue
            if c is None or sg not in (0, 1) or wl < 1:
                continue
            c = -c if neg else c
            lo, hi = (-(1 << (wl - 1)), (1 << (wl - 1)) - 1) if sg else (0, (1 << wl) - 1)
            v = c * Fraction(2) ** fl
            if not lo <= v <= hi or not lo <= math.floor(v + Fraction(1, 2)) <= hi:
                out.append(n)
                break
        if n not in out and _ml_cast_of_constant(ast, P):
            out.append(n)
    return [{"id": ML_CONST_FOLD, "blocks": sorted(out)}] if out else []


def _ml_cast_of_constant(ast, P):
    """cast(c, 'like', x) of a compile-time double c is folded by the block fimath, and
    Simulink warns ConstantFoldingOverFlow while compiling when c does not fit x's type (12 / 78 records: 1.5 into
    sfix16_En15, -0.3 / -1 into ufix8_En7). x's type is known only after compiling, so the warning is allowed for
    every such cast whose c is built from literals (+ - * / ^, unary minus) or is a variable assigned only such an
    expression; values are compared either way."""
    lit_ops = {"num", "neg", "+", "-", "*", "/", "^"}

    def literal(e):
        return all(x.get("op") in lit_ops for x in P._nodes(e) if isinstance(x, dict) and "op" in x)

    for fn in ast.get("functions", []):
        consts = {}
        for st in P._nodes(fn.get("body", [])):
            if isinstance(st, dict) and st.get("op") == "assign" and isinstance(st.get("target"), str):
                consts.setdefault(st["target"], []).append(literal(st.get("expr") or {"op": "?"}))
        for e in P._nodes(fn.get("body", [])):
            a = e.get("args") or [] if isinstance(e, dict) else []
            if not (isinstance(e, dict) and e.get("op") == "call" and e.get("name") == "cast" and len(a) == 3 and
                    a[1].get("op") == "str" and a[1].get("text") == "like"):
                continue
            c = a[0]
            if c.get("op") == "var" and c.get("name") not in fn.get("inputs", []) and all(consts.get(c["name"], [0])):
                return True
            if c.get("op") != "var" and literal(c):
                return True
    return False


def _one_bit_signed_extract(design, res):
    """an Extract Bits taking a one-bit field of a signed input is answered like Simulink (values and
    types), but Simulink warns while compiling it (SimulinkFixedPoint:util:sfix_dtpropBitLess, from the block's
    internal DTProp1: the field widened to two bits; id from recordings) and the check fails on any
    warning. Returns the warnings the package allows: that id, for these blocks only."""
    out = []
    ct = res.get("compiled_types") or {}
    for n, b in design["blocks"].items():
        if not isinstance(b, dict) or b.get("type") != "ExtractBits":
            continue
        src = next((ln["src"] for ln in design.get("lines", []) if ln["dst"] == n), None)
        st = G.fc._parse_fixdt(ct.get(src, "")) if src and ct.get(src, "").startswith("fixdt(") else None
        if not st or st[0] != 1:
            continue
        mode = str(b.get("bitsToExtract", "Upper half"))
        if mode in ("Upper half", "Lower half"):
            width = st[1] // 2
        elif mode == "Range of bits":
            try:
                lo_hi = [int(float(x)) for x in str(b.get("bitIdxRange", "[0 7]")).strip("[] ").replace(",", " ").split()]
                width = abs(lo_hi[0] - lo_hi[-1]) + 1
            except ValueError:
                continue
        else:
            try:
                width = int(float(str(b.get("numBits", "8"))))
            except ValueError:
                continue
        if width == 1:
            out.append(n)
    return [{"id": SFIX_BITLESS, "blocks": sorted(out)}] if out else []


def _on_cycle(start, lines):
    """Whether block start lies on a directed cycle of the design's lines."""
    succ = {}
    for ln in lines:
        succ.setdefault(ln["src"], set()).add(ln["dst"])
    seen, stack = set(), list(succ.get(start, ()))
    while stack:
        n = stack.pop()
        if n == start:
            return True
        if n in seen:
            continue
        seen.add(n)
        stack.extend(succ.get(n, ()))
    return False


def _needs_sysram(b):
    """Only the System RAM has a non-zero initial value and, for the Dual Port RAM, WriteOutputValue 'Old data'
    (the masked Dual Port RAM's write port gives the new data; recordings)."""
    return _norm(b.get("RAMInitialValue", "0")) != _norm("0") or (
        str(b.get("RAMType")) == "Dual port" and str(b.get("WriteOutputValue", "New data")) == "Old data")


def _sysram_for_init(design, hw, inputs):
    """A RAM with a non-zero initial value is written as the System RAM (the masked RAM starts at 0), whose address
    width is the address signal's: an explicit AddressBits must equal it, and becomes 'Inherit: from address' (the
    same RAM; the reader reads a System RAM so). Recorded for the Simple Dual Port RAM only."""
    import fxhdl
    init = [n for n, b in design["blocks"].items() if isinstance(b, dict) and b.get("type") == "HDLRAM"
            and _needs_sysram(b)]
    if not init:
        return design, {}
    # a non-zero initial value was recorded for the Simple Dual Port RAM; the Dual Port and
    # Single Port System RAMs are written from their recorded templates the same way and checked in native testing
    #, as the engine accepts them under the same rule
    explicit = [n for n in init if str(design["blocks"][n].get("AddressBits", "")).strip() != "Inherit: from address"]
    if not explicit:
        return design, {}
    probe = dict(design, blocks=dict(design["blocks"]))
    for n in explicit:
        probe["blocks"][n] = dict(probe["blocks"][n], AddressBits="Inherit: from address")
    try:
        types = {k: _in_type(v, "inputs." + k) for k, v in inputs.items()}
        widths = fxhdl.ram_auto_resolve(probe, hw, types)
    except (G.GraphError, KeyError, TypeError):
        raise ExportError("UNSUPPORTED_EXPORT", "design.blocks.%s.AddressBits" % explicit[0], "the System RAM "
                          "(non-zero initial value) takes its width from the address signal, whose type could not "
                          "be found")
    for n in explicit:
        if str(widths["blocks"][n]["AddressBits"]) != str(design["blocks"][n]["AddressBits"]).strip():
            raise ExportError("UNSUPPORTED_EXPORT", "design.blocks.%s.AddressBits" % n, "AddressBits %s differs from "
                              "the address word length %s: the System RAM (needed for a non-zero initial value) "
                              "takes the address width" % (design["blocks"][n]["AddressBits"],
                                                           widths["blocks"][n]["AddressBits"]))
    return probe, {n: {"AddressBits": "Inherit: from address (= %s)" % design["blocks"][n]["AddressBits"]}
                   for n in explicit}


def export_package(req, run_fxgraph, read_slx, list_slx):
    """Validate, simulate, build, self-check. run_fxgraph(request) -> fxgraph response dict (raises on error);
    read_slx(bytes, scope) -> (design, info); list_slx(bytes) -> list response. Returns (files {name: bytes},
    summary dict). Nothing is returned unless the roundtrip agrees."""
    allowed = {"design", "inputs", "hw", "nsteps", "stop_time", "name", "out", "observe"}
    for ln_ in ((req.get("design") or {}).get("lines") or []) if isinstance(req.get("design"), dict) else []:
        for k_ in ("sport", "dport"):
            if isinstance(ln_, dict) and k_ in ln_ and not str(ln_[k_]).strip().lstrip("-").isdigit():
                # a named port (the integrator's 'state') answered INTERNAL here
                raise ExportError("UNSUPPORTED_EXPORT", "design.lines", "a line on the named port %s %r: the saved "
                                  "model's port layout for it was not recorded; record one to export it"
                                  % (k_, ln_[k_]))
    extra = sorted(set(req) - allowed)
    if extra:
        raise ExportError("BAD_PARAM", extra[0], "unknown request field %s (%s)" % (extra[0], ", ".join(sorted(allowed))))
    for k in ("design", "name"):
        if k not in req:
            raise ExportError("MISSING_FIELD", k, "%s is required" % k)
    name = req["name"]
    if isinstance(req.get("inputs"), dict):
        # (the native minimal example: fxgraph also takes {"type": {"s", "wl", "fl"}, "ints"} --
        # docs/en/AGENT_GUIDE.md -- and the export refused a request fxgraph had answered; the same form here, flattened)
        flat_ = {}
        for k_, v_ in req["inputs"].items():
            if isinstance(v_, dict) and isinstance(v_.get("type"), dict):
                v_ = dict({kk: vv for kk, vv in v_.items() if kk != "type"}, **v_["type"])
            flat_[k_] = v_
        req = dict(req, inputs=flat_)
    if not (isinstance(name, str) and re.match(r"^[A-Za-z][A-Za-z0-9_]{0,49}$", name)):
        raise ExportError("BAD_PARAM", "name", "model name: a letter then letters, digits or _ (at most 50)")
    design = req["design"]
    if not isinstance(design, dict) or not isinstance(design.get("blocks"), dict):
        raise ExportError("MISSING_FIELD", "design.blocks", "design {blocks, lines} is required")
    G.check_user_design(design)
    request_design = design   # the request as given (its hash, its explicit parameters)
    # a the extended blocks's left-out parameters are the library block's defaults -- written out, so the
    # package states the values the engine simulated (it wrote the engine's table value: DitherSource 'None')
    import fxhdl
    design = fxhdl.with_library_defaults(design)
    original = design
    design, observed = _observe(design, req.get("observe", []))
    hw = req.get("hw") or {}
    hw_kind, _cc, hw_checks = hw_profile(hw)
    design, resolved = _resolve_ram_widths(design, hw, req.get("inputs") or {})
    _engine_semantics_open(design, req.get("inputs") or {})
    # 1. the engine's own answer (same request as fxgraph would get); its refusal is the export's refusal
    fx = {"design": design, "hw": hw, "outputs": "outports"}
    req = dict(req, design=design)
    if "inputs" in req:
        fx["inputs"] = req["inputs"]
    for k in ("nsteps", "stop_time"):
        if k in req:
            fx[k] = req[k]
    if observed:
        # the request as written is validated first -- its refusal is the export's refusal, and the
        # observed run must give the same answer on every original output
        r0 = run_fxgraph(dict(fx, design=original))
    res = run_fxgraph(fx)
    if observed:
        for k, v in (r0.get("outputs") or {}).items():
            if res["outputs"].get(k) != v:
                raise ExportError("INTERNAL", "observe", "output %s differs once %s are observed" % (k, sorted(observed)))
        if r0.get("assertion_failures") != res.get("assertion_failures"):
            raise ExportError("INTERNAL", "observe", "the assertion results differ once blocks are observed")
    allowed_warnings = _one_bit_signed_extract(design, res) + _switch_bool_control(design, res) + \
        _demux_uneven(design, res) + _mps_default_warning(design, res) + _ml_const_fold_overflow(design, res) + _ds_priority_warning(design, res) + \
        _fifo_warnings(design, res)
    for w_ in _engine_expected_warnings(design, res):   # merged into an entry of the same id (no duplicates)
        same = [a for a in allowed_warnings if a.get("id") == w_["id"] and set(a) == {"id", "blocks"}]
        if same:
            same[0]["blocks"] = sorted(set(same[0]["blocks"]) | set(w_["blocks"]))
        else:
            allowed_warnings.append(w_)
    failing = sorted(b for b, v in (res.get("assertion_failures") or {}).items() if v)
    if failing:
        # Simulink warns at each failing step; the check script compares values
        # and types, not diagnostics, so a package could not show that the failures agree
        raise ExportError("UNSUPPORTED_EXPORT", "design.blocks." + failing[0], "the assertion fails during the run "
                          "(steps %s); only runs in which no Assertion fails are exported"
                          % ", ".join(map(str, res["assertion_failures"][failing[0]][:5])))
    if (res.get("metrics") or {}).get("engine") == "python-float" or \
            any(isinstance(v, dict) and v.get("type") in ("double", "single") for v in (res.get("outputs") or {}).values()):
        # a previous version: fxgraph answers floating-point designs (engine/fxfloatsim.py); the package format and the
        # check script carry fixed-point / boolean signals only, so such a design is not exported (it said "no samples")
        raise ExportError("UNSUPPORTED_EXPORT", "design", "a floating-point (double / single) design: fxgraph answers "
                          "it, but export packages carry fixed-point and boolean signals only; export of "
                          "floating-point designs is not supported yet")
    # a package must expect at least one sample on every output (stop_time 0 with an input period 2
    # and offset 1 gave an empty expectation)
    empty = sorted(k for k, v in (res.get("outputs") or {}).items() if not (v.get("ints") or v.get("bool")))
    if not res.get("outputs") or empty:
        raise ExportError("UNSUPPORTED_EXPORT", "outputs", "the simulation gives no samples on %s (check nsteps / "
                          "stop_time against the input periods); a package with an empty expectation proves nothing"
                          % (", ".join(empty) or "any output"))
    ins, outs = plan_ports(design, req.get("inputs") or {})
    # vector outputs come back scalarised as "y[1]", "y[2]", ... (engine/fxvector): one root Outport, compared
    # element by element
    elems, cplx = {}, set()
    for key in res["outputs"]:
        base = _strip_index(key)
        if base not in outs and key.endswith((".re", ".im")) and key[:-3] in outs:
            # a scalar complex Outport: the engine gives it as <y>.re / <y>.im; the
            # package expects one complex element with both parts (the check script compares real() and imag())
            cplx.add(key[:-3])
            elems.setdefault(key[:-3], []).append(key)
            continue
        if base not in outs and key.endswith((".re", ".im")) and _strip_index(key[:-3]) in outs:
            raise ExportError("UNSUPPORTED_EXPORT", "design.blocks." + _strip_index(key[:-3]), "a complex vector "
                              "Outport is not exported; split it with a Complex to Real-Imag in the design")
        if base not in outs:
            raise ExportError("INTERNAL", "design", "engine output %r is not a design Outport" % key)
        elems.setdefault(base, []).append(key)
    for n in outs:
        ks = elems.get(n, [])
        if n in cplx:
            if sorted(ks) != [n + ".im", n + ".re"]:
                raise ExportError("INTERNAL", "design.blocks." + n, "complex output parts %s" % sorted(ks))
            elems[n] = [n + ".re", n + ".im"]
            continue
        if ks == [n]:
            continue
        idx = sorted(int(k[len(n) + 1:-1]) for k in ks)
        if not ks or idx != list(range(1, len(ks) + 1)):
            raise ExportError("UNSUPPORTED_EXPORT", "design.blocks." + n, "output %s: elements %s" % (n, sorted(ks)))
        elems[n] = ["%s[%d]" % (n, j) for j in idx]
    multirate = "sample_times" in res
    inputs = req.get("inputs") or {}
    in_plan = []
    in_keys = {}
    for k, n in enumerate(ins):
        st = design["blocks"][n].get("SampleTime", "1")
        txt, per, off = _sample_time_text(st)
        w = _width(design["blocks"][n])
        if w == 1 and n not in inputs and "%s[1]" % n in inputs and "PortDimensions" not in design["blocks"][n]:
            # a design read from a model whose vector input comes from a From Workspace has an Inport
            # without PortDimensions (Simulink infers the width); the inputs give it element by element -- the
            # export writes that width on the Inport
            while "%s[%d]" % (n, w + 1) in inputs:
                w += 1
            design = dict(design, blocks=dict(design["blocks"], **{n: dict(design["blocks"][n], PortDimensions=str(w))}))
        keys = [n] if w == 1 else ["%s[%d]" % (n, j) for j in range(1, w + 1)]
        if any(kk not in inputs for kk in keys):
            raise ExportError("MISSING_FIELD", "inputs." + n, "inputs %s required" % keys)
        tys = {_in_type(inputs[kk], "inputs." + kk) for kk in keys}
        if len(tys) != 1:
            # a vector Inport carries one data type for every element
            raise ExportError("UNSUPPORTED_EXPORT", "inputs." + n, "elements of vector input %s have types %s"
                              % (n, sorted(tys)))
        in_plan.append((n, txt, tys.pop(), "%s_in%d" % (name, k + 1), per, off))
        in_keys[n] = keys
    if multirate:
        stop = Fraction(res["metrics"]["stop_time"])
        solver = {"FixedStep": "auto", "StopTime": _period_text(stop), "SolverMode": "SingleTasking"}
        for n, b in design["blocks"].items():
            got = res.get("sample_times", {}).get(n)
            if b.get("type") in _COUNTERS:
                # a counter is written with tsamp = its own stated period (block_params): exported when the engine
                # runs it at exactly that period -- a stated whole period P (no offset) at P, or 1 / -1 at 1. Anything
                # else (a period the engine derived that the block does not state, e.g. -1 inheriting a slower /
                # faster rate; an offset; a non-whole period) is not what the written model would run
                st = _counter_stated_period(b)
                if st is None or got != (st if st != "-1" else "1"):
                    raise ExportError("UNSUPPORTED_EXPORT", "design.blocks." + n, "counter running at period %s in a "
                                      "multirate design with its sample time %r (exported only at its own stated "
                                      "whole period: tsamp P running at P, or 1 / -1 running at 1)" % (got, _counter_ts(b)))
            elif b.get("type") == "HDLCounter" and got != "1":
                raise ExportError("UNSUPPORTED_EXPORT", "design.blocks." + n, "counter running at period %s in a "
                                  "multirate design (the export writes its sample time 1)" % got)
        out_periods = []
        for n in outs:
            p = res["outputs"][elems[n][0]]["period"]
            if p == "constant":
                raise ExportError("UNSUPPORTED_EXPORT", "design.blocks." + n, "an Outport with a constant sample "
                                  "time (no periodic samples to compare)")
            # an output on an offset grid (an input [2 1] into an accumulator) gets the offset too --
            # the first engine sample's time modulo the period (the root Outport wrote '2' and the check found its
            # samples at 0, 2, ... instead of 1, 3, ...)
            ts0 = res["outputs"][elems[n][0]].get("t") or ["0"]
            o_off = Fraction(str(ts0[0])) % Fraction(p)
            out_periods.append(_period_text(Fraction(p)) if o_off == 0 else
                               "[%s %s]" % (_period_text(Fraction(p)), _period_text(o_off)))
    else:
        nsteps = res["metrics"]["nsteps"]
        pers = {p for (_, _, _, _, p, _) in in_plan} or {Fraction(1)}
        offs = {o for (_, _, _, _, _, o) in in_plan}
        if len(pers) != 1 or offs - {Fraction(0)}:
            raise ExportError("UNSUPPORTED_EXPORT", "design", "single-rate design with input periods %s / offsets %s"
                              % (sorted(map(str, pers)), sorted(map(str, offs))))
        P = pers.pop()
        for n, b in design["blocks"].items():
            if b.get("type") in _COUNTERS and _counter_ts(b) not in ("1", "-1"):
                # single-rate designs keep the base step 1 only (a counter with a whole period of its own is
                # exported in a multirate design, where the engine gives its period)
                raise ExportError("UNSUPPORTED_EXPORT", "design.blocks.%s.tsamp" % n, "counter sample time %r in a "
                                  "single-rate design (the base step 1 only)" % _counter_ts(b))
        if P != 1 and (any(b.get("type") in _COUNTERS for b in design["blocks"].values()) or
                       any(b.get("type") == "HDLCounter" and Fraction(str(b.get("CountSampTime", "1")).strip()) != P
                           for b in design["blocks"].values())):
            # (an HDL Counter whose CountSampTime is the inputs' period: recordings, below)
            raise ExportError("UNSUPPORTED_EXPORT", "design", "counters with a base step other than 1")
        hst = {str(b.get("CountSampTime", "1")).strip() for b in design["blocks"].values() if b.get("type") == "HDLCounter"}
        if hst - {"1"}:
            # an HDL Counter without ports at CountSampTime 2 / 3 / 4 alone in the model
            # (fixed step auto) -- its period is the design's only rate: the engine's steps are the counter's samples
            # (t = k * period). Exported only in that recorded shape: one counter (no input ports -- the engine refuses
            # ports there) and Outports, nothing else (another block could carry its own rate)
            others = sorted(n for n, b in design["blocks"].items() if b.get("type") not in ("HDLCounter", "Outport",
                                                                                              "Inport"))
            ncnt = sum(1 for b in design["blocks"].values() if b.get("type") == "HDLCounter")
            # (a counter with input ports, the inputs at the counter's own period --
            # the design's one rate is that period, the engine's steps are its samples)
            in_ok = not in_plan or (len(hst) == 1 and P == Fraction(next(iter(hst))))
            if len(hst) != 1 or ncnt != 1 or others or not in_ok:
                raise ExportError("UNSUPPORTED_EXPORT", "design", ('HDL Counter sample time %s exported '
                    'only in a design of one counter, '
                    'Outports and Inports at its period%s') % ("/".join(sorted(hst)), "; other blocks: %s"
                                                                % ", ".join(others) if others else ""))
            P = Fraction(hst.pop())
        solver = {"FixedStep": _period_text(P), "StopTime": _period_text((nsteps - 1) * P), "SolverMode": "SingleTasking"}
        out_periods = [_period_text(P)] * len(outs)
    data, checks, info = build_model(design, hw, name, [(n, t, ty, v) for (n, t, ty, v, _, _) in in_plan], outs,
                                     solver, out_periods, out_types=(None if multirate or V.has_vectors(design) else res.get("compiled_types")))
    # 2. roundtrip through the project's own reader + engine
    rt = roundtrip(data, design, hw, req, res, in_plan, info["dut"], run_fxgraph, read_slx, list_slx)
    # 3. the package
    sha = hashlib.sha256(data).hexdigest()
    io_obj = {"format": FORMAT_IO, "model": name, "model_sha256": sha, "dut": info["dut"],
              "config": dict({"SolverName": "FixedStepDiscrete", "FixedStep": solver["FixedStep"],
                              "StopTime": solver["StopTime"], "SolverMode": solver["SolverMode"]}, **hw_checks),
              "inputs": [], "outputs": [], "compiled_types": [], "block_params": []}
    for (n, txt, ty, var, per, off) in in_plan:
        # element-major for a vector input: all samples of element 1, then element 2, ... (the script reshapes n x w)
        cols = [inputs[kk]["ints"] for kk in in_keys[n]]
        stamps = _input_stamps(per, off, len(cols[0]))
        ints = [x for c in cols for x in c]
        if ty == G.BOOL:   # a boolean Inport: 0 / 1 per sample
            if any(x not in (0, 1) for x in ints):
                raise ExportError("BAD_INPUT", "inputs." + n, "a boolean input takes 0 or 1")
            e = {"port": n, "variable": var, "sample_time": txt, "type": "boolean", "width": len(cols),
                 "t": stamps,
                 "bins": [str(x) for x in ints], "ints": [str(x) for x in ints], "nums": list(ints)}
        else:
            e = {"port": n, "variable": var, "sample_time": txt, "s": ty[0], "wl": ty[1], "fl": ty[2],
                 "width": len(cols),
                 "t": stamps,
                 "bins": [_bins(x, ty[0], ty[1]) for x in ints],
                 "ints": [str(x) for x in ints],
                 "nums": [x for x in ints] if ty[1] <= 53 else []}
        io_obj["inputs"].append(e)
    for k, n in enumerate(outs):
        for j, key in enumerate(elems[n]):
            o = res["outputs"][key]
            part = key[-2:] if n in cplx else ""
            j = 0 if n in cplx else j
            if o["type"] == "boolean":
                vals, tkey, bins = o["bool"], "boolean", [str(int(x)) for x in o["bool"]]
            else:
                vals, tkey = o["ints"], "%d,%d,%d" % (o["s"], o["wl"], o["fl"])
                bins = [_bins(x, o["s"], o["wl"]) for x in vals]
            ts = [float(Fraction(t)) for t in o["t"]] if "t" in o else [float(i * Fraction(solver["FixedStep"]))
                                                                         for i in range(len(vals))]
            io_obj["outputs"].append({"port": n, "index": k + 1, "element": j + 1,
                                      "width": 1 if n in cplx else len(elems[n]), "part": part,
                                      "root_block": info["root_outports"][k], "type": tkey,
                                      "t": ts, "bins": bins, "ints": [str(int(x)) for x in vals],
                                      # the period the signal reaching the root Outport must compile
                                      # to (the root Outport's own SampleTime resampled a faster signal unnoticed)
                                      "period": float(Fraction(str(out_periods[k]).strip("[]").split()[0])),
                                      # the offset of an output on an offset grid ([0.3 0.1]), which
                                      # the compiled sample time carries too (the check compared it with 0)
                                      "offset": float(Fraction((str(out_periods[k]).strip("[]").split() + ["0"])[1]))})
    io_obj["root_outputs"] = len(outs)
    types, mixed = {}, set()
    for b, t in sorted(res["compiled_types"].items()):
        if any(ch in b for ch in G.RESERVED_CHARS):
            # internal blocks of a lowered library block (engine/fxhdl.py) have no Simulink path;
            # the library block itself is listed under its own name
            continue
        base = _strip_index(b)
        if base.endswith((".re", ".im")) and design["blocks"].get(base[:-3], {}).get("type") == "Outport":
            continue   # the parts of a complex Outport: an Outport has no compiled output type
        if design["blocks"].get(base, {}).get("type") in ("Outport", "EnabledOutport"):
            continue   # an Outport block has no output port, so no compiled output type (recordings re-run)
        tt = G.fc._parse_fixdt(t) if t != "boolean" else G.BOOL
        key = "boolean" if t == "boolean" else ("%d,%d,%d" % tt if tt else "other:" + t)
        if base in types and types[base] != key:
            mixed.add(base)   # a bus of mixed element types (a Unit Delay on a bus): no single compiled type
        types[base] = key
    io_obj["compiled_types"] = [{"block": info["dut"] + "/" + b, "type": t} for b, t in sorted(types.items())
                                if b not in mixed]
    # the output ports after the first, each with its own compiled type ('unknown' when the engine
    # cannot name it: the check then leaves the types incomplete); the check script also requires one entry for
    # every further output port Simulink shows on a typed block
    io_obj["warnings_allowed"] = [dict(w_, blocks=[info["dut"] + "/" + b for b in w_["blocks"]])
                                  for w_ in allowed_warnings]
    io_obj["port_types"] = []
    for b, ports in sorted((res.get("compiled_port_types") or {}).items()):
        if b not in types or b in mixed or any(ch in b for ch in G.RESERVED_CHARS):
            continue
        for k, t in sorted(ports.items(), key=lambda x: int(x[0])):
            tt = (G.fc._parse_fixdt(t) if t != "boolean" else G.BOOL) if t else None
            key = "unknown" if t is None else ("boolean" if t == "boolean" else ("%d,%d,%d" % tt if tt else "other:" + t))
            io_obj["port_types"].append({"block": info["dut"] + "/" + b, "port": int(k), "type": key})
    # the type list must account for every exported design block -- typed, or exempt with a reason
    # the check script verifies ('no output port': the Simulink block has none; anything else leaves it INCOMPLETE)
    written = sorted(b for b, x in design["blocks"].items() if not any(ch in b for ch in G.RESERVED_CHARS)
                     and x.get("type") not in _NOT_SIMULINK)
    typed = {b for b in types if b not in mixed}
    io_obj["design_blocks"] = [info["dut"] + "/" + b for b in written]
    io_obj["type_exempt"] = [{"block": info["dut"] + "/" + b,
                              # a block carrying a bus of mixed element types outputs a virtual
                              # bus -- 'bus signal', verified by CompiledBusType (was 'mixed bus': never verified)
                              "reason": "bus signal" if b in mixed else (
                                  "no output port" if design["blocks"][b].get("type") in NO_OUTPUT_TYPES
                                  else "action signal" if design["blocks"][b].get("type") == "SwitchCase"
                                  # a Bus Creator's output is a virtual bus (no data type of its own;
                                  # its elements are typed where they are selected) -- verified by CompiledBusType
                                  else "bus signal" if design["blocks"][b].get("type") in ("BusCreator", "BusAssignment")
                                  or (design["blocks"][b].get("type") == "BusSelector" and
                                      str(design["blocks"][b].get("OutputAsBus", "off")) == "on")
                                  else "no compiled type from the engine")}
                             for b in written if b not in typed]
    # a MATLAB Function block's own inner blocks (the Stateflow S-Function and its Demux)
    # are listed, exempt as the function's structure (the check script confirms the parent is a MATLAB Function)
    for b in written:
        if design["blocks"][b].get("type") == "MATLABFunction":
            for inner in (" SFunction ", " Demux "):
                nm_ = info["dut"] + "/" + b + "/" + inner
                io_obj["design_blocks"].append(nm_)
                io_obj["type_exempt"].append({"block": nm_, "reason": "MATLAB Function internal"})
    if not io_obj["outputs"] or not io_obj["compiled_types"]:
        raise ExportError("UNSUPPORTED_EXPORT", "design", "nothing to compare (no outputs or no typed block)")
    io_obj["block_params"] = [{"block": b, "param": p, "value": v} for b, p, v in checks]
    lib_checks, lib_devs, lib_unchecked = library_layer(request_design["blocks"], info.get("written", {}))
    io_obj["library_defaults"] = lib_checks
    io_obj["expected_note"] = ("outputs: fxsim stored integers (bins = two's complement bit patterns, ints = exact "
                               "decimal); inputs: the same as fed to fxsim; compiled types as s,wl,fl")
    why = io_incomplete(io_obj)
    if why:
        raise ExportError("INTERNAL", "design", "the package expectation is incomplete: " + why)
    io_bytes = (json.dumps(io_obj, separators=(",", ":")) + "\n").encode()
    mfile = matlab_script(name).encode()
    files = {name + ".slx": data, name + "_io.json": io_bytes, "check_%s.m" % name: mfile}
    manifest = {"format": "fxexport-manifest/1", "model": name, "release": RELEASE, "dut": info["dut"],
                "files": {f: hashlib.sha256(b).hexdigest() for f, b in sorted(files.items())},
                "design_sha256": hashlib.sha256(_canon(design).encode()).hexdigest(),
                # the request's design exactly as given (design_sha256 is the design after the
                # export wrote the library defaults out and added observation blocks)
                "request_sha256": hashlib.sha256(_canon(request_design).encode()).hexdigest(),
                "library_defaults_checked_in_simulink": len(lib_checks),
                "library_deviations": lib_devs, "library_defaults_unchecked": lib_unchecked,
                "hw_profile": hw_kind, "multirate": multirate, "solver": solver,
                "inputs": [{"port": n, "variable": v, "sample_time": t} for (n, t, _, v, _, _) in in_plan],
                "outputs": output_summary(io_obj, outs),
                "dropped_parameters": info["dropped_parameters"],
                # the error reporting contract: the parameters the design left out and the values the engine assumed for them
                # (fxgraph assumed_defaults) -- the export writes exactly these values into the model
                "assumed_defaults": {k: v for k, v in res.get("assumed_defaults", {}).items()
                                     if not any(ch in k for ch in G.RESERVED_CHARS)},   # the user's blocks only
                # a System RAM's inherited address width, written as the width the engine resolved
                "resolved_parameters": {n: (w if isinstance(w, dict) else {"AddressBits": w})
                                        for n, w in sorted(resolved.items())},
                "observation_outports": observed,
                "unobserved_templates": info["unobserved_templates"],
                "roundtrip": rt,
                "how_to_verify": "in MATLAB R2026b: cd to this folder and run check_%s -- prints one JSON line, "
                                 "verdict PASS / FAIL" % name}
    files["manifest.json"] = (json.dumps(manifest, indent=1, sort_keys=True) + "\n").encode()
    summary = {"model": name, "files": manifest["files"], "blocks": len(design["blocks"]), "multirate": multirate,
               "hw_profile": hw_kind, "outputs": manifest["outputs"], "roundtrip": rt,
               "unobserved_templates": info["unobserved_templates"], "dropped_parameters": info["dropped_parameters"],
               "assumed_defaults": manifest["assumed_defaults"]}
    return files, summary


def _norm(v):
    v = str(v).strip()
    if BS_NUM.match(v):
        try:
            return repr(float(v))
        except ValueError:
            return v
    return re.sub(r"\s+", " ", v)


BS_NUM = re.compile(r"^\s*[-+]?(\d+(\.\d*)?|\.\d+)([eE][-+]?\d+)?\s*$")
# parameters the reader consumes or rewrites (compared separately or not engine-relevant)
_RT_SKIP = {"SampleTime", "tsamp", "InputProcessing", "NumberOfTableDimensions", "InitialConditionSource",
            "ExternalReset", "DelayLengthSource", "ShowEnablePort", "BitShiftNumberSource", "CoefSource",
            "FilterSource", "framing", "filtFrom", "FactorSource", "Port", "OutDataTypeStr@Inport"}


def _effective(name, b, nconnected=None):
    et = b["type"]
    p = {k: _norm(v) for k, v in dict(BS.DEFAULTS.get(et, {}), **BS.JSON_LIBRARY_DEFAULTS.get(et, {}),
                                      **HIDDEN_DEFAULTS.get(et, {})).items()}
    if et in ("PreLookup", "Interpolation_n-D"):   # the values the lowering assumes (written by the export)
        import fxprelookup
        p.update({k: _norm(v) for k, v in (fxprelookup.PL_DEFAULTS if et == "PreLookup" else
                                           fxprelookup.IP_DEFAULTS).items() if v is not None})
    if et == "Prelookup" and str(b.get("FractionDataTypeStr", "Inherit")).strip().startswith("Inherit"):
        p["FractionDataTypeStr"] = _norm("Inherit: Inherit via internal rule")   # (as written, block_params)
    if et == "HDLCounter" and "CountFrom" not in b:
        p["CountFrom"] = _norm(str(b.get("CountInit", p.get("CountInit", "0"))))   # (as written, block_params)
    if et == "MinMax" and "Inputs" not in b and nconnected == 1:
        p["Inputs"] = _norm("1")   # engine/fxvector._one_input (a Sum: the library '|++', see above)
    for k, v in b.items():
        if k == "type" or k in _DROPPED:
            continue
        p[k] = _norm(v)
    if et == "Sum" and "Inputs" in p:
        p["Inputs"] = p["Inputs"].replace("|", "")
    for k_ in ("OutputSignals", "AssignedSignals"):   # compared as Simulink stores them
        if et in ("BusSelector", "BusAssignment") and isinstance(p.get(k_), str):
            p[k_] = ",".join(x.strip() for x in p[k_].split(","))
    for k in list(p):
        if k in _RT_SKIP or re.match(r"^(firstCoeff|prodOutput|accum|output)(Mode|WordLength|FracLength)$", k):
            del p[k]
    if et == "Inport":
        p.pop("OutDataTypeStr", None)
    if et == "Outport":
        return {}
    if et == "HDLRAM" and p.get("RAMType") == _norm("Simple dual port") or (
            et == "HDLRAM" and p.get("RAMType") == _norm("Dual port")
            and p.get("AddressBits") != _norm("Inherit: from address")):
        # a Simple dual port RAM has no write-port output, the masked Dual Port RAM no WriteOutputValue (the System
        # Dual Port RAM has it)
        p.pop("WriteOutputValue", None)
    for k in NOT_A_PARAMETER.get(et, {}):
        p.pop(k, None)
    if et in ("Lookup1D", "Lookup2D") and p.get("BreakpointsSpecification") == _norm("Even spacing"):
        p.pop("BreakpointsForDimension1", None)
        p.pop("BreakpointsForDimension2", None)
    for k in list(p):
        if (et == "Lookup1D" and BS._UNUSED_DIM.match(k)) or (et == "Lookup2D" and BS._UNUSED_DIM2.match(k)):
            del p[k]
    return p


def _same_arith(x, y):
    """a parameter written as literal arithmetic ('1/3*3') is read back as its value ('1.0'), as the
    reader evaluates it (fxslx_lib, engine/fxnum._literal_arith); equal when the arithmetic gives that value."""
    import fxnum
    if x is None or y is None:
        return False
    for u, v in ((x, y), (y, x)):
        lit = fxnum._literal_arith(str(u))
        su = str(u).strip()
        if lit is None and su.startswith("[") and su.endswith("]") and su.count("[") == 1:
            # main fxslx_lib (export roundtrip of recordings): a matrix whose elements are
            # each literal arithmetic ('[1/3 0.1;-0.7 2/3]') is read back as each element's double
            lit = fxnum._elementwise_arith(su[1:-1])
        if lit is not None and _norm(lit) == _norm(v):
            return True
    # Compare integer dialog expressions by their evaluated values.
    from fractions import Fraction
    import fxcore

    def _val(t):
        t = str(t).strip()
        m = re.fullmatch(r"\[\s*([^\[\];,\s]+)\s*\]", t)
        if m:
            t = m.group(1)
        try:
            return Fraction(t)
        except (ValueError, ZeroDivisionError):
            pass
        try:
            r = fxcore.matlab_int_expr(t)
        except Exception:  # noqa: BLE001 -- an expression the evaluator does not take is simply not equal
            r = None
        return r[0] if r else None
    for u, v in ((x, y), (y, x)):
        try:
            fv = Fraction(str(v).strip())
        except (ValueError, ZeroDivisionError):
            continue
        if _val(u) == fv:
            return True
    return False


def _carried(el):
    """Parameters the project's reader carries into the design for this Block element."""
    bt = el.get("BlockType")
    if bt == "Reference":
        src = S._params(el).get("SourceBlock", "")
        lib = BS.LIBRARY.get(src.split("/")[-1].replace("\n", " "))
        keys = set(lib[1]) if lib else set()
    else:
        keys = set(BS.TYPEMAP.get(bt, ("", ()))[1])
    return keys | set(BS.META_VALUES)


def roundtrip(data, design, hw, req, res, in_plan, dut, run_fxgraph, read_slx, list_slx):
    """Read the exported model back with the project's reader and compare structure, every parameter, the input
    periods, compiled types and every output sample with the engine's answer on the original design."""
    try:
        back, info = read_slx(data, dut)
        _, _, elements = S._read_model(S.Model(data), dut)
    except G.GraphError as e:
        explicit = sorted(n for n, b in design["blocks"].items() if isinstance(b, dict)
                          and b.get("type") != "Inport" and str(b.get("SampleTime", "-1")).strip() not in ("-1", ""))
        if "several sample times" in e.message and explicit:
            # explicit sample times on blocks of a slower domain (a Gain '[4 0]' and a Unit Delay '4'
            # after a Downsample) -- the engine answers like Simulink, but the reader the round trip rests on takes
            # one sample time per scope: a refusal of the export, not a defect of the package
            # a Rate Transition with an inherited output period takes it from such a block -- then the
            # period belongs on the Rate Transition (OutPortSampleTime) instead (same result, recordings rt*)
            rts = sorted(n for n, b in design["blocks"].items() if isinstance(b, dict) and b.get("type") ==
                         "RateTransition" and (str(b.get("OutPortSampleTimeOpt", "Specify")) == "Inherit" or
                                               str(b.get("OutPortSampleTime", "-1")).strip() in ("-1", "")))
            raise ExportError("UNSUPPORTED_EXPORT", "design.blocks." + explicit[0], "explicit SampleTime on %s in a "
                              "multirate design: the export's round trip reads one sample time per subsystem; leave "
                              "the SampleTime of these blocks inherited (-1) -- they take the rate of their inputs%s"
                              % (", ".join(explicit[:5]),
                                 "; Rate Transition %s inherits its output period from them: write that period as "
                                 "its OutPortSampleTime instead (the same model in Simulink)" % ", ".join(rts)
                                 if rts else ""))
        raise ExportError("EXPORT_ROUNDTRIP_MISMATCH", "roundtrip.read", "the project's reader refuses the exported "
                          "model (%s at %s: %s)" % (e.code, e.where, e.message[:200]))
    ob, bb = design["blocks"], back["blocks"]
    # the engine's spelling of Prelookup / Interpolation Using Prelookup is read back in recorded (the same Simulink blocks;
    # fxgraph answers either with the rule set that covers the shape)
    spell = {"Prelookup": "PreLookup", "InterpolationUsingPrelookup": "Interpolation_n-D"}
    ob = {n: (dict(b, type=spell[b["type"]]) if isinstance(b, dict) and b.get("type") in spell else b)
          for n, b in ob.items()}
    if set(ob) != set(bb):
        raise ExportError("EXPORT_ROUNDTRIP_MISMATCH", "roundtrip.blocks", "block sets differ: only in design %s, "
                          "only in export %s" % (sorted(set(ob) - set(bb))[:5], sorted(set(bb) - set(ob))[:5]))
    nparams = 0
    for n in sorted(ob):
        if ob[n]["type"] != bb[n]["type"]:
            raise ExportError("EXPORT_ROUNDTRIP_MISMATCH", "roundtrip.blocks." + n, "type %s read back as %s"
                              % (ob[n]["type"], bb[n]["type"]))
        ncon = sum(1 for l in design.get("lines", []) if l["dst"] == n)
        a, b = _effective(n, ob[n], ncon), _effective(n, bb[n], ncon)
        if design["blocks"][n].get("type") in spell:
            # a parameter of the engine's spelling the recorded type's table does not list: compared as written
            a_main = _effective(n, design["blocks"][n], ncon)   # the engine's table: the values the export wrote
            a.update({k: v for k, v in a_main.items() if a.get(k) is None})
        el = elements.get(n)
        raw = S._params(el[1]) if el else {}
        carried = _carried(el[1]) if el else set()
        if ob[n]["type"] == "MATLABFunction":
            # the reader takes a MATLAB Function's settings from its chart, where a setting at its
            # default is absent ('SaturateOnIntegerOverflow: on' was "written None"); both sides with the defaults
            import fxhdl
            dfl = {k: _norm(v) for k, v in fxhdl.ML_DEFAULTS.items() if v is not None and k in _ML_KEYS}
            a, b = dict(dfl, **a), dict(dfl, **b)
            carried = carried | set(_ML_KEYS)
        for k in sorted(set(a) | set(b)):
            if k == "BitShiftNumber" and ob[n]["type"] == "ShiftArithmetic" and \
                    str(ob[n].get("BitShiftNumberSource", "Dialog")).strip() == "Input port":
                continue   # the shift comes from the port; the dialog value is not used
            if k in bb[n] or k in carried:
                if a.get(k) != b.get(k) and not _same_arith(a.get(k), b.get(k)):
                    raise ExportError("EXPORT_ROUNDTRIP_MISMATCH", "roundtrip.blocks.%s.%s" % (n, k),
                                      "design %r, read back %r" % (a.get(k), b.get(k)))
            elif _norm(raw.get(k, "")) != a[k]:
                # a parameter the reader does not carry: it must stand in the file exactly as the engine used it
                raise ExportError("EXPORT_ROUNDTRIP_MISMATCH", "roundtrip.blocks.%s.%s" % (n, k),
                                  "design %r, written %r" % (a[k], raw.get(k)))
            nparams += 1
    def lset(ls):
        return sorted((l["src"], int(l.get("sport", 1)), l["dst"], int(l.get("dport", 1))) for l in ls)
    if lset(design.get("lines", [])) != lset(back.get("lines", [])):
        a, b = set(lset(design.get("lines", []))), set(lset(back.get("lines", [])))
        raise ExportError("EXPORT_ROUNDTRIP_MISMATCH", "roundtrip.lines", "lines differ: only in design %s, only in "
                          "export %s" % (sorted(a - b)[:4], sorted(b - a)[:4]))
    if (info.get("hw") or {}) != {"ProdHWDeviceType": hw_profile(hw)[1]["ProdHWDeviceType"]} and \
            not (hw_profile(hw)[0] == "asic" and info.get("hw", {}).get("ProdHWDeviceType") == "ASIC/FPGA"):
        raise ExportError("EXPORT_ROUNDTRIP_MISMATCH", "roundtrip.hw", "hardware read back as %s" % info.get("hw"))
    # input periods: written on the From Workspace sources at the model's top level
    top = {e["name"]: e for e in list_slx(data)["top_level"]}
    for (n, txt, ty, var, per, off) in in_plan:
        src = [e for e in top.values() if e["type"] == "FromWorkspace" and e["params"].get("VariableName") == var]
        if len(src) != 1 or src[0]["params"].get("SampleTime") != txt:
            raise ExportError("EXPORT_ROUNDTRIP_MISMATCH", "roundtrip.inputs." + n, "source of input %s not found "
                              "with sample time %s" % (n, txt))
        back["blocks"][n] = dict(back["blocks"][n])
        if "SampleTime" in design["blocks"][n]:
            back["blocks"][n]["SampleTime"] = design["blocks"][n]["SampleTime"]
    # the reader drops a block's SampleTime equal to the model's step, and in
    # a multirate design with an Inport that written period is what seeds a loop no input reaches (the HDL-counter-style
    # Unit Delay loop) -- the design's explicit periods are put back on the blocks read back, as for the inputs above
    for n, b_ in design["blocks"].items():
        if isinstance(b_, dict) and "SampleTime" in b_ and n in back["blocks"] and \
                "SampleTime" not in back["blocks"][n] and b_.get("type") not in ("Inport", "Outport"):
            back["blocks"][n] = dict(back["blocks"][n], SampleTime=b_["SampleTime"])
    # re-simulate the design as read back
    fx = {"design": back, "hw": info.get("hw") or {}, "outputs": "outports"}
    for k in ("inputs", "nsteps", "stop_time"):
        if k in req:
            fx[k] = req[k]
    try:
        r2 = run_fxgraph(fx)
    except G.GraphError as e:
        raise ExportError("EXPORT_ROUNDTRIP_MISMATCH", "roundtrip.simulate", "the read-back design is refused "
                          "(%s: %s)" % (e.code, e.message[:200]))
    if r2["compiled_types"] != res["compiled_types"]:
        k = next(k for k in sorted(set(r2["compiled_types"]) | set(res["compiled_types"]))
                 if r2["compiled_types"].get(k) != res["compiled_types"].get(k))
        raise ExportError("EXPORT_ROUNDTRIP_MISMATCH", "roundtrip.types." + k, "compiled type %s, read back %s"
                          % (res["compiled_types"].get(k), r2["compiled_types"].get(k)))
    if r2.get("compiled_port_types") != res.get("compiled_port_types"):
        raise ExportError("EXPORT_ROUNDTRIP_MISMATCH", "roundtrip.port_types", "compiled types of further output "
                          "ports %s, read back %s" % (res.get("compiled_port_types"), r2.get("compiled_port_types")))
    for n, o in res["outputs"].items():
        o2 = r2["outputs"].get(n)
        if o2 != o:
            raise ExportError("EXPORT_ROUNDTRIP_MISMATCH", "roundtrip.outputs." + n, "output differs after the "
                              "roundtrip")
    return {"blocks": len(ob), "lines": len(design.get("lines", [])), "params_compared": nparams,
            "outputs_equal": sorted(res["outputs"]), "reader_assumed_defaults": info.get("assumed_defaults", {}),
            "reader_ignored_parameters": info.get("ignored_parameters", {})}

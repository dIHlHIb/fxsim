"""One registry of block support: the engine's semantic parameters, defaults and allowed values, the
.slx names and adapters, and the optimiser's type roles, in one place so that the JSON path, the .slx reader and
fxopt cannot drift apart (the .slx reader dropped two arithmetic flags the JSON path refuses).

Moved here unchanged from engine/fxgraph_lib.py, engine/fxslx_lib.py and engine/fxgopt_lib.py, which import these
names back; check() states the cross-table rules and is run by the regression check."""
import re

# the same strings as ref/fxcore.py _INHERIT_RULE / _SAME_FIRST (the regression check checks they agree)
INHERIT_RULE = "Inherit: Inherit via internal rule"
SAME_FIRST = "Inherit: Same as first input"

# ================================================================ engine: semantic parameters (engine/fxgraph_lib.py)
DEFAULTS = {
    "Gain": {"OutDataTypeStr": INHERIT_RULE, "ParamDataTypeStr": INHERIT_RULE,
             "RndMeth": "Floor", "SaturateOnIntegerOverflow": "off"},
    "Sum": {"Inputs": "++", "OutDataTypeStr": INHERIT_RULE, "AccumDataTypeStr": INHERIT_RULE,
            "RndMeth": "Floor", "SaturateOnIntegerOverflow": "off"},
    "Product": {"Inputs": "2", "OutDataTypeStr": INHERIT_RULE, "RndMeth": "Floor",
                "SaturateOnIntegerOverflow": "off"},
    "DataTypeConversion": {"OutDataTypeStr": "Inherit: Inherit via back propagation",
                           "ConvertRealWorld": "Real World Value (RWV)", "RndMeth": "Floor",
                           "SaturateOnIntegerOverflow": "off"},
    "Saturation": {"RndMeth": "Floor"},
    "RelationalOperator": {"Operator": ">="},
    "MinMax": {"Function": "min", "Inputs": "2", "OutDataTypeStr": INHERIT_RULE, "RndMeth": "Floor",
               "SaturateOnIntegerOverflow": "off"},
    "Abs": {"SaturateOnIntegerOverflow": "off"},
    "UnaryMinus": {"SaturateOnIntegerOverflow": "off"},
    "UnitDelay": {"InitialCondition": "0"},
    # internal blocks of the multirate lowering (engine/fxlower.py): never read from a model
    "Hold": {"InitialCondition": "0"},   # y = enable ? x : previous y   (inputs: x, enable)
    "Gate": {},                          # y = enable ? x : 0            (inputs: x, enable)
    # internal: one output port of a MATLAB Function block (lowered by engine/fxhdl.py, semantics engine/fxmlsem.py)
    # DSP HDL filters (lowered by engine/fxhdl.py; the same tables as engine/fxhdlfir.py, Numerator required)
    "HDLFIRFilter": {'NumeratorSource': 'Property', 'NumeratorPrototype': '[]', 'FilterStructure': 'Direct form systolic', 'SerializationOption': 'Minimum number of cycles between valid input samples', 'NumCycles': '2', 'NumberOfMultipliers': '2', 'roundingMode': 'Floor', 'overflowMode': 'off', 'CoefficientsDataTypeStr': 'Inherit: Same word length as input', 'OutputDataTypeStr': 'Inherit: Inherit via internal rule', 'LockScale': 'off', 'ResetInputPort': 'off', 'HDLGlobalReset': 'off', 'CoeffMultiplier': 'Multiplier', 'CoeffMultiplierSystolic': 'Multiplier', 'UseMultForPow2Coeff': 'on', 'FrameInputSymmOptim': 'on', 'SimulateUsing': 'Code generation'},
    "HDLFIRDecimator": {'NumeratorSource': 'Property', 'NumeratorPrototype': '[]', 'FilterStructure': 'Direct form systolic', 'DecimationFactor': '2', 'NumCycles': '1', 'OptimizeSymmCoeff': 'on', 'roundingMode': 'Floor', 'overflowMode': 'off', 'CoefficientsDataTypeStr': 'Inherit: Same word length as input', 'OutputDataTypeStr': 'Inherit: Inherit via internal rule', 'LockScale': 'off', 'ResetInputPort': 'off', 'HDLGlobalReset': 'off', 'SimulateUsing': 'Code generation'},
    # internal: one output port of a DSP HDL FIR Filter / FIR Decimator (engine/fxhdl.py, engine/fxhdlfir.py)
    "HDLNCO": {'PhaseIncrementSource': 'Property', 'PhaseIncrement': '100', 'Frequency': '510', 'FreqResolution': '0.05', 'SamplingFreq': '4000', 'PhaseOffsetSource': 'Property', 'PhaseOffset': '0', 'PhaseOffsetR': 'pi/3', 'DitherSource': 'None', 'NumDitherBits': '4', 'SamplesPerFrame': '1', 'LUTCompress': 'off', 'ResetAction': 'off', 'Waveform': 'Sine', 'PhasePort': 'off', 'RoundingMethod': 'Floor', 'OverflowAction': 'Wrap', 'AccumulatorDataType': 'Binary point scaling', 'AccumulatorSigned': 'yes', 'AccumulatorWL': '16', 'AccumulatorFL': '0', 'PhaseQuantization': 'on', 'NumQuantizerAccumulatorBits': '12', 'OutputDataType': 'Binary point scaling', 'OutputSigned': 'yes', 'OutputWL': '16', 'OutputFL': '14', 'SimulateUsing': 'Code generation'},   # DSP HDL NCO (lowered by engine/fxhdl.py; the table of engine/fxhdlnco.py)
    "RAMCore": {"Mode": "read", "A": "10", "Init": "0", "Owner": ""},   # internal: one output of a deep HDL RAM (engine/fxhdl.py)
    "NCOCore": {"Sel": "sin", "Params": "{}", "Owner": ""},   # internal: one real output of an HDL NCO (engine/fxhdlnco.py)
    "HDLFIRCore": {"Kind": "HDLFIRFilter", "Port": "1", "Params": "{}", "Owner": ""},
    "PLCore": {"Params": ""}, "PLInterp": {"Params": ""},   # recordings (engine/fxprelookup.py)
    "CordicCore": {"Params": ""},   # complex <-> magnitude-angle CORDIC (engine/fxcordic.py)
    "MLFn": {"Script": "", "Port": "1", "Ports": "1", "Owner": "", "InWidths": "", "Elem": "", "TreatAsFi": "Fixed-point", "InputFimath": "",
             "EmlDefaultFimath": "Same as MATLAB Default", "SaturateOnIntegerOverflow": "on",
             "OutDataTypeStr": "Inherit: Same as Simulink"},
    # recorded configurations multirate (engine/fxmultirate.py; timing rules recordings)
    "Downsample": {"DownsamplingFactor": "2", "SampleOffset": "0", "InitialConditions": "0",
                   "RateOptions": "Allow multirate processing"},
    "Upsample": {"UpsamplingFactor": "2", "SampleOffset": "0", "InitialConditions": "0",
                 "RateOptions": "Allow multirate processing"},
    "RateTransition": {"Integrity": "on", "Deterministic": "on", "InitialCondition": "0",
                       "OutPortSampleTimeOpt": "Multiple of input port sample time", "OutPortSampleTimeMultiple": "1",
                       "OutPortSampleTime": "-1"},
    "FIRDecimation": {"FilterSource": "Dialog parameters", "framing": "Allow multirate processing",
                      "D": "2", "filtStruct": "Direct form", "roundingMode": "Floor", "overflowMode": "off",
                      "outputBufInitCond": "0", "firstCoeffMode": "Same word length as input",
                      "firstCoeffDataTypeStr": "Inherit: Same word length as input",
                      "prodOutputMode": "Inherit via internal rule", "prodOutputDataTypeStr": "Inherit: Inherit via internal rule",
                      "accumMode": "Inherit via internal rule", "accumDataTypeStr": "Inherit: Inherit via internal rule",
                      "outputMode": "Same as accumulator", "outputDataTypeStr": "Inherit: Same as accumulator"},
    "CICDecimation": {"R": "2", "M": "1", "N": "2", "ftype": "Decimator", "filterInternals": "Full precision",
                      "BPS": "[]", "FLPS": "[]", "outputWordLength": "16", "outputFracLength": "0",
                      "RateOptions": "Allow multirate processing"},
    "CICInterpolation": {"R": "2", "M": "1", "N": "2", "ftype": "Interpolator", "filterInternals": "Full precision",
                         "BPS": "[]", "FLPS": "[]", "outputWordLength": "16", "outputFracLength": "0",
                         "RateOptions": "Allow multirate processing"},
    "FIRInterpolation": {"FilterSource": "Dialog parameters", "framing": "Allow multirate processing",
                         "L": "2", "roundingMode": "Floor", "overflowMode": "off", "outputBufInitCond": "0",
                         "firstCoeffMode": "Same word length as input",
                         "firstCoeffDataTypeStr": "Inherit: Same word length as input",
                         "prodOutputMode": "Inherit via internal rule",
                         "prodOutputDataTypeStr": "Inherit: Inherit via internal rule",
                         "accumMode": "Inherit via internal rule", "accumDataTypeStr": "Inherit: Inherit via internal rule",
                         "outputMode": "Same as accumulator", "outputDataTypeStr": "Inherit: Same as accumulator"},
    "Inport": {},
    "Outport": {},
    # HDL library bit operations (engine/fxhdl.py; recorded dialog defaults)
    "HDLCounter": {"CountType": "Count limited", "CountInit": "0", "CountStep": "1", "CountMax": "25",
                   "CountFromType": "Initial value", "CountFrom": "0", "CountResetPort": "off", "CountLoadPort": "off",
                   "CountEnbPort": "off", "CountDirPort": "off", "CountHitOutputPort": "off",
                   "CountDataType": "Unsigned", "CountWordLen": "8", "CountFracLen": "0", "CountSampTime": "1"},
    "UnitDelayEnabledSynchronous": {"InitialCondition": "0.0", "SampleTime": "-1"},
    "UnitDelayEnabledResettableSynchronous": {"InitialCondition": "0.0", "SampleTime": "-1"},
    "UnitDelayResettableSynchronous": {"InitialCondition": "0.0", "SampleTime": "-1"},
    "RealImagToComplex": {"Input": "Real and imag", "ConstantPart": "0"},
    "ComplexToRealImag": {"Output": "Real and imag"},
    "EnablePort": {"StatesWhenEnabling": "held"},
    "StateControl": {"StateControl": "Classic"},
    "EnabledOutport": {"OutputWhenDisabled": "held", "InitialOutput": "[]"},
    "HDLRAM": {"RAMType": "Simple dual port", "AddressBits": "8", "WriteOutputValue": "New data",
               "RAMInitialValue": "0"},
    "HDLFIFO": {"fifo_size": "10", "mode": "Classic", "ratio": "1", "dataDim": "1", "push_msg": "Warning",
                "pop_msg": "Warning", "show_empty": "on", "show_full": "on", "show_num": "on", "rst_port": "off"},   # recordings (engine/fxhdlfifo.py)
    "DetectRisePositive": {"vinit": "0", "InputProcessing": "Elements as channels (sample based)",
                           "OutDataTypeStr": "boolean"},
    "DirectLookupND": {"NumberOfTableDimensions": "1", "InputsSelectThisObjectFromTable": "Element", "Table": "[]",
                       "TableDataTypeStr": "Inherit: Inherit from 'Table data'",
                       "DiagnosticForOutOfRangeInput": "Warning", "RemoveProtectionInput": "off"},
    "Repeat": {"N": "5", "ic": "0", "InputProcessing": "Columns as channels (frame based)",
               "RateOptions": "Enforce single-rate processing", "FactorSource": "Dialog parameter"},
    "DataTypeDuplicate": {"NumInputPorts": "2"},
    "Concatenate": {"NumInputs": "2", "Mode": "Vector", "ConcatenateDimension": "1"},
    "BitSlice": {"lidx": "7", "ridx": "0"},
    "BitConcat": {"numInputs": "2"},
    "BitShift": {"mode": "Shift Left Logical", "N": "0"},
    # the library defaults were not recorded, so the parameters are REQUIRED (no
    # engine default; SEMANTIC_EXTRA lists them)
    "BitRotate": {},
    "BitSet": {},
    "BitClear": {},
    # block types (rules in recordings). Defaults below are only conveniences; designs
    # converted from Simulink carry every parameter explicitly.
    "Constant": {"OutDataTypeStr": "Inherit: Inherit from 'Constant value'"},
    "Delay": {"InitialCondition": "0"},
    # internal: an integrator whose state port feeds its own initial condition through a Gain (engine lowering)
    "DTIStateFB": {"IntegratorMethod": "Integration: Forward Euler", "gainval": "1.0", "ExternalReset": "rising",
                   "InitialCondition": "0", "OutDataTypeStr": "Inherit: Inherit via internal rule", "RndMeth": "Floor",
                   "SaturateOnIntegerOverflow": "off", "LimitOutput": "off", "SampleTime": "-1"},
    "DTIStateFBGain": {"IntegratorMethod": "Integration: Forward Euler", "gainval": "1.0", "ExternalReset": "rising",
                       "InitialCondition": "0", "OutDataTypeStr": "Inherit: Inherit via internal rule", "RndMeth": "Floor",
                       "SaturateOnIntegerOverflow": "off", "LimitOutput": "off", "SampleTime": "-1"},
    # internal: a Delay with its length or initial condition from a port (engine/fxgraph_lib.py lowering)
    "DelayPort": {"InitialCondition": "0", "DelayLength": "2", "DelayLengthSource": "Dialog",
                  "DelayLengthUpperLimit": "100", "InitialConditionSource": "Dialog"},
    # recordings (foundation completeness): BitShiftNumber / BitShiftDirection left out was refused
    # MISSING_FIELD; the library block has them (8, 'Bidirectional')
    "ShiftArithmetic": {"BinPtShiftNumber": "0", "BitShiftNumber": "8", "BitShiftDirection": "Bidirectional"},
    "LogicalOperator": {"Operator": "AND", "Inputs": "2"},
    "CompareToConstant": {"OutDataTypeStr": "boolean"},
    "Switch": {"Criteria": "u2 >= Threshold", "Threshold": "0", "OutDataTypeStr": INHERIT_RULE,
               "RndMeth": "Floor", "SaturateOnIntegerOverflow": "off"},
    "MultiportSwitch": {"DataPortOrder": "One-based contiguous", "Inputs": "3",
                        "DataPortForDefault": "Last data port", "OutDataTypeStr": INHERIT_RULE,
                        "RndMeth": "Floor", "SaturateOnIntegerOverflow": "off",
                        # 'None' = an out-of-range control takes the default port (all 12,640 recorded
                        # blocks); the library default is 'Error' (JSON_LIBRARY_DEFAULTS): Simulink stops there
                        "DiagnosticForDefault": "None"},
    "DiscreteFir": {"CoefDataTypeStr": "Inherit: Same word length as input", "ProductDataTypeStr": INHERIT_RULE,
                    "AccumDataTypeStr": INHERIT_RULE, "StateDataTypeStr": "Inherit: Same as accumulator",
                    "OutDataTypeStr": "Inherit: Same as accumulator", "RndMeth": "Floor",
                    "SaturateOnIntegerOverflow": "off", "InitialStates": "0", "FilterStructure": "Direct form",
                    "TapSumDataTypeStr": "Inherit: Same as input"},   # assumed default (recordings always states it)
    # recorded configurations ; evaluated per step through ref/fxcore.py (Python path)
    "CounterLimited": {"uplimit": "7"},
    "CounterFreeRunning": {"NumBits": "16"},
    "DetectChange": {"vinit": "0", "OutDataTypeStr": "boolean"},
    "DetectIncrease": {"vinit": "0", "OutDataTypeStr": "boolean"},
    "DetectDecrease": {"vinit": "0", "OutDataTypeStr": "boolean"},
    "Signum": {},
    # library defaults (Compare To Zero relop '<=', boolean output;
    # Detect Rise / Fall vinit 0, boolean output)
    "CompareToZero": {"relop": "<=", "OutDataTypeStr": "boolean"},
    "DetectRisePositive": {"vinit": "0", "OutDataTypeStr": "boolean"},
    "DetectRiseNonnegative": {"vinit": "0", "OutDataTypeStr": "boolean"},
    "DetectFallNegative": {"vinit": "0", "OutDataTypeStr": "boolean"},
    "DetectFallNonpositive": {"vinit": "0", "OutDataTypeStr": "boolean"},
    # library defaults -- the saved sample models leave out Start -0.5, End 0.5,
    # slew limits 1 / -1 and initial condition 0, and the reader's replay of them matches the records; Wrap To Zero's
    # Threshold's library default is 255 (the native library-defaults recording)
    # saved models leave out Operator sin and NumberOfIterations 11
    "Trigonometry": {"Operator": "sin", "ApproximationMethod": "None", "NumberOfIterations": "11"},
    "DeadZone": {"LowerValue": "-0.5", "UpperValue": "0.5", "SaturateOnIntegerOverflow": "on"},
    "RateLimiter": {"RisingSlewLimit": "1", "FallingSlewLimit": "-1", "InitialCondition": "0",
                    "SampleTimeMode": "inherited"},
    "WrapToZero": {"Threshold": "255"},   # the library Threshold
    # the Sqrt block's library defaults
    # library dialog defaults (also the saved samples of
    # they store 'Inherit: Inherit via internal rule' and
    # SaturateOnIntegerOverflow 'off' when set, so neither is the default)
    "Sqrt": {"Operator": "sqrt", "OutDataTypeStr": "Inherit: Same as first input",
             "RndMeth": "Floor", "SaturateOnIntegerOverflow": "on"},
    "Rounding": {"Operator": "floor"},
    "BitwiseOperator": {"logicop": "AND", "UseBitMask": "on", "NumInputPorts": "1", "BitMask": "bin2dec('11011001')",
                        "BitMaskRealWorld": "Stored Integer"},
    "ExtractBits": {"bitsToExtract": "Upper half", "numBits": "8", "bitIdxRange": "[0 7]",
                    "outScalingMode": "Preserve fixed-point scaling"},
    "DiscreteIntegrator": {"IntegratorMethod": "Integration: Forward Euler", "gainval": "1.0", "ExternalReset": "none",
                           "InitialConditionSource": "internal", "InitialCondition": "0",
                           "OutDataTypeStr": INHERIT_RULE, "RndMeth": "Floor", "SaturateOnIntegerOverflow": "off",
                           "LimitOutput": "off", "UpperSaturationLimit": "inf", "LowerSaturationLimit": "-inf",
                           "SampleTime": "1"},
    "Lookup2D": {"BreakpointsSpecification": "Explicit values",
                 "BreakpointsForDimension1DataTypeStr": "Inherit: Same as corresponding input",
                 "BreakpointsForDimension2DataTypeStr": "Inherit: Same as corresponding input",
                 "BreakpointsForDimension1FirstPoint": "0", "BreakpointsForDimension1Spacing": "1",
                 "BreakpointsForDimension2FirstPoint": "0", "BreakpointsForDimension2Spacing": "1",
                 "TableDataTypeStr": "Inherit: Same as output", "IntermediateResultsDataTypeStr": "Inherit: Same as output",
                 "FractionDataTypeStr": INHERIT_RULE, "InterpMethod": "Linear point-slope", "ExtrapMethod": "Clip",
                 "UseLastTableValue": "off", "InternalRulePriority": "Speed",
                 "OutDataTypeStr": SAME_FIRST, "RndMeth": "Floor", "SaturateOnIntegerOverflow": "off"},
    "Lookup1D": {"BreakpointsSpecification": "Explicit values",
                 "BreakpointsForDimension1DataTypeStr": "Inherit: Same as corresponding input",
                 "BreakpointsForDimension1FirstPoint": "0", "BreakpointsForDimension1Spacing": "1",
                 "TableDataTypeStr": "Inherit: Same as output", "IntermediateResultsDataTypeStr": "Inherit: Same as output",
                 "FractionDataTypeStr": INHERIT_RULE, "InterpMethod": "Linear point-slope", "ExtrapMethod": "Clip",
                 # used by the inherited intermediate / fraction rules (ref/fxcore.py); was only in META_ANY, so
                 # reported as ignored; library default Speed
                 "InternalRulePriority": "Speed",
                 "OutDataTypeStr": SAME_FIRST, "RndMeth": "Floor", "SaturateOnIntegerOverflow": "off"},
    # Prelookup + Interpolation Using Prelookup (
    # recordings); the Prelookup's fraction output is the internal block PrelookupF
    "Prelookup": {"BreakpointsSpecification": "Explicit values", "BreakpointsDataSource": "Dialog",
                  "BreakpointDataTypeStr": "Inherit: Same as input", "FractionDataTypeStr": "fixdt(0,32,31)",
                  "IndexDataTypeStr": "uint32", "ExtrapMethod": "Clip", "UseLastBreakpoint": "off",
                  "OutputSelection": "Index and fraction", "IndexSearchMethod": "Binary search",
                  "BeginIndexSearchUsingPreviousIndexResult": "off", "DiagnosticForOutOfRangeInput": "None",
                  "RemoveProtectionInput": "off", "RndMeth": "Floor", "LockScale": "off",
                  "BreakpointMin": "[]", "BreakpointMax": "[]"},
    "InterpolationUsingPrelookup": {"NumberOfTableDimensions": "2", "TableSource": "Dialog",
                                    "TableSpecification": "Explicit values", "LookupTableObject": "",
                                    "InterpMethod": "Linear point-slope", "ExtrapMethod": "Clip",
                                    "DiagnosticForOutOfRangeInput": "None", "RequireIndexFractionAsBus": "off",
                                    "RemoveProtectionIndex": "off", "ValidIndexMayReachLast": "off",
                                    "NumSelectionDims": "0", "OutMin": "[]", "OutMax": "[]",
                                    "OutDataTypeStr": "Inherit: Inherit via internal rule", "InternalRulePriority": "Speed",
                                    "LockScale": "off", "RndMeth": "Floor", "SaturateOnIntegerOverflow": "off",
                                    "TableMin": "[]", "TableMax": "[]", "TableDataTypeStr": "Inherit: Same as output",
                                    "IntermediateResultsDataTypeStr": INHERIT_RULE},
    # n-D Lookup Table, 3 / 4 dimensions
    "LookupND": dict({"BreakpointsSpecification": "Explicit values", "TableDataTypeStr": "Inherit: Same as output",
                      "IntermediateResultsDataTypeStr": "Inherit: Same as output", "FractionDataTypeStr": INHERIT_RULE,
                      "InterpMethod": "Linear point-slope", "ExtrapMethod": "Clip", "InternalRulePriority": "Speed",
                      "UseLastTableValue": "off", "OutDataTypeStr": SAME_FIRST, "RndMeth": "Floor",
                      "SaturateOnIntegerOverflow": "off", "NumberOfTableDimensions": "3"},
                     **{"BreakpointsForDimension%dDataTypeStr" % k: "Inherit: Same as corresponding input"
                        for k in range(1, 5)}),
}

# Behaviour notes printed under the block's table in docs/en/BLOCKS.md (the regression check): what is
# answered and what is refused (release docs: descriptions of supported configurations)
DEFAULTS["PrelookupF"] = dict(DEFAULTS["Prelookup"])   # internal: a Prelookup's fraction output (fxgraph_lib lowering)

BLOCK_NOTES = {
    "Sqrt": (
        "Runs on the Python engine only: there is no C-core form, so `\"engine\": \"c\"` refuses it "
        "(`UNSUPPORTED_WIDTH`, \"Python path only\") and `auto` runs it in Python. Answered with Operator `sqrt`, "
        "AlgorithmType `Exact`, an inherited intermediate type, and an explicit fixed-point output type -- or the "
        "library default output `Inherit: Same as first input` for the input types, roundings and hardware that "
        "were recorded."),
    "DeadZone": (
        "Simulink `simulink/Discontinuities/Dead Zone`. Answered for one fixed-point or integer scalar input of 4 to 48 "
        "bits with fraction length -2 .. word length - 1; output type = input type. Start (`LowerValue`) and End "
        "(`UpperValue`) are rounded to nearest (ties up) into the input type; up to one LSB above the maximum they "
        "saturate (with a quantisation note), beyond that -- or one LSB or more below the minimum -- Simulink stops "
        "with `fxpParameterOverflow` and the engine refuses with `PORT_TYPE_INVALID` at that parameter. Output: 0 "
        "where Start <= u <= End, u - End above, u - Start below; that difference saturates "
        "(`SaturateOnIntegerOverflow` `on`, the default) or wraps (`off`) into the input type -- it leaves the type "
        "when Start > 0 or End < 0 (e.g. unsigned u below a positive Start). Refused: Start above End (after "
        "rounding; not recorded), other types, vectors, boolean / double inputs. Evidence: Simulink R2026b recordings "
        "of 50 + 48 designs (single, after a Gain, before a Sum; periods 1 and 2; host and ASIC/FPGA)."),
    "RateLimiter": (
        "Simulink `simulink/Discontinuities/Rate Limiter`. Answered for one fixed-point or integer scalar input of 4 "
        "to 48 bits with fraction length -2 .. word length - 1, `SampleTimeMode` `inherited`, in a single-rate design at "
        "sample period 1 or 2 (also inside feedback loops); output type = input type. With T the period, R = "
        "`RisingSlewLimit` x T is rounded to nearest (ties up) into the input type (Simulink's Weighted Rise Limit: "
        "its overflow is `fxpParameterOverflow`, refused as `PORT_TYPE_INVALID`), F = `FallingSlewLimit` x T is not "
        "held in the input type (-1 runs with an unsigned input), the initial condition is rounded into the input "
        "type. Each step: d = u - y_prev (y_prev = initial condition at the first step); y = y_prev + R if d > R, "
        "y_prev + F if d < F, else u (never overflows). Refused: a negative rising or positive falling limit, a "
        "falling limit whose magnitude x T exceeds the input type's maximum by more than one LSB or has a fraction "
        "of an LSB of one half or more (below one half it rounds to the whole LSBs, recorded at -0.25 LSB), at period 2 any limit x T that is not a whole number of LSBs within the type, "
        "periods other than 1 and 2, multirate designs, other types, vectors. Evidence: Simulink R2026b recordings of "
        "50 + 16 designs (incl. 9 Unit Delay loops and 7 at period 2)."),
    "LookupND": (
        "Simulink `simulink/Lookup Tables/n-D Lookup Table` with 3 or 4 dimensions (`NumberOfTableDimensions`; use "
        "Lookup1D / Lookup2D for 1 / 2). One scalar input per dimension, all of one type; explicit, strictly "
        "increasing breakpoints per dimension (2 to 4 points); `Table` written reshape([values], [n1 n2 ...]) "
        "(column-major, as MATLAB), non-negative values. Breakpoints and table are rounded to nearest into their "
        "types. Flat / Nearest pick a table value per dimension; Linear point-slope interpolates dimension 1 first, "
        "then 2, ..., every step y0 + floor(f (y1 - y0)) in the output type with wrap, f floored to as many fraction "
        "bits as the output word (16, and the falling term from the magnitude, when an unsigned 8 / 16-bit table "
        "type is the output type). ExtrapMethod must be Clip: Simulink rejects linear extrapolation on fixed point "
        "(the library default). Answered only for the recorded type combinations (input / breakpoints / table / "
        "output): sfix8_En4 inherited; sfix8_En4 / fixdt(1,12,6) / fixdt(1,8,4) / inherited; ufix8_En4 / inherited / "
        "fixdt(0,8,4) / inherited; sfix12_En6 / fixdt(1,8,4) / fixdt(1,12,6) / fixdt(1,16,8); sfix12_En6 / "
        "inherited / fixdt(0,8,4) / fixdt(1,12,6); ufix8_En4 / fixdt(1,12,6) / fixdt(1,8,4) / inherited -- with "
        "the inherited intermediate and fraction types, InternalRulePriority Speed, RndMeth Simplest, no "
        "saturation, UseLastTableValue off, host, outside loops. Nearest needs breakpoints of the input's type "
        "(Simulink PreLookupNearestUAndBpDataTypeMismatch). Evidence: Simulink R2026b dense recording of 60 models "
        "(30 run, 47,700 steps)."),
    "Prelookup": (
        "Simulink `simulink/Lookup Tables/Prelookup`: one scalar fixed-point input, two outputs -- port 1 the "
        "index k (uint32), port 2 the fraction f. Explicit, strictly increasing breakpoints (2 to 4 points), rounded "
        "to nearest into the breakpoint type (inherited = the input type). k = the interval the input falls in "
        "(clipped to the first / last interval); f = (u - bp[k]) / (bp[k+1] - bp[k]) floored into the fraction type, "
        "0 below the first breakpoint and 1 at or above the last. ExtrapMethod must be Clip: Simulink rejects linear "
        "extrapolation on fixed point (the library default). Answered only for the recorded types: input "
        "sfix8_En4 / sfix12_En6 / ufix8_En4, breakpoints inherited / fixdt(1,12,6) / fixdt(1,8,4), fraction "
        "fixdt(0,32,31) (the default) / fixdt(0,16,15) / fixdt(0,8,7), index uint32, the other parameters at their "
        "defaults, host; and inputs int16 / sfix16_En8 / sfix16_En14 / ufix8_En4 with inherited breakpoints (2 to "
        "4, 5, 9 or 17 points) and the library fraction type (left out or 'Inherit: Inherit via internal rule': "
        "ufix32_En31 on the host, ufix<w>_En<w-1> of the input's word length on ASIC / FPGA), host and ASIC / FPGA. "
        "Not word-length optimised (no double reference). Evidence: Simulink R2026b dense recordings of 84 + 60 + "
        "288 models (the lookup / matrix dense map, the Interpolation Using Prelookup map, the additional coverage "
        "map)."),
    "InterpolationUsingPrelookup": (
        "Simulink `simulink/Lookup Tables/Interpolation Using Prelookup` with 1 or 2 dimensions; inputs 2d-1 / 2d "
        "are the k / f outputs of one Prelookup per dimension (the recorded wiring). `Table` a vector (1-D) or "
        "reshape([values], [n1 n2]) (column-major), non-negative values, rounded to nearest into the table type "
        "(inherited = the output type). The output type must be explicit, except the recorded back-propagated "
        "form: 1-D, Clip, fed by one fixed-point Prelookup on an Inport, every destination an untyped Outport, as "
        "recorded (engine/ipl_double_domain.json) -- Simulink compiles it to double and it runs on the "
        "floating-point path as y0 + f (y1 - y0) in double (Flat: the table value). Linear point-slope interpolates dimension 1 first, then 2, each step y0 + floor(f (y1 - y0)) in "
        "an intermediate type with wrap (the table type with a 32-bit fraction; otherwise the table's integer bits "
        "in 16 or 32 bits), then floored into the output type; Flat / Nearest pick a table value. ExtrapMethod must "
        "be Clip (Simulink CannotExtrapData). Answered only for the recorded table / output combinations: "
        "Linear point-slope -- s12En6 / s16En8, u8En4 / s12En6, s8En4 / s8En4, s8En4 / s16En8, u8En4 / u12En6, "
        "u8En4 / s16En10, s12En6 / s12En6, s12En6 / s12En4, u16En8 / s12En6 and s16En8 / s8En4 (both with the 32-bit "
        "fraction only: for a 16-bit table with a 16 / 8-bit fraction the intermediate type is not settled); Flat / Nearest -- s12En6 / s16En8, u8En4 / s12En6; the other parameters at their "
        "defaults, host, outside loops. After a Prelookup on int16 / sfix16_En8 / sfix16_En14 / ufix8_En4 with the "
        "library fraction type (1-D, host and ASIC / FPGA): table / output s16En8 / s32En16 and int16 / int16, Linear "
        "point-slope and Flat, signed table values; there a signed 16-bit table with a 16 / 8-bit fraction (ASIC) "
        "interpolates in the table type. Not word-length optimised. Evidence: Simulink R2026b dense recordings of "
        "84 + 60 + 288 models every answered step equal)."),
    "WrapToZero": (
        "Simulink `simulink/Discontinuities/Wrap To Zero` (a masked subsystem). Answered for one fixed-point or "
        "integer scalar input of 4 to 48 bits with fraction length -2 .. word length; output type = input type. "
        "`Threshold` (library default 255) is rounded to nearest (ties up) into the input "
        "type, with the same overflow rule as Dead Zone. Output: 0 where u > Threshold, else u. Refused: other "
        "types, vectors. Evidence: Simulink R2026b recordings of 50 + 12 designs (thresholds inside the type, at "
        "its maximum, on half LSBs)."),
}

REQUIRED = {"BitRotate": ("mode", "N"), "BitSet": ("iBit",), "BitClear": ("iBit",), "HDLFIRFilter": ("Numerator",), "HDLFIRDecimator": ("Numerator",), "FIRDecimation": ("h",), "FIRInterpolation": ("h",), "Lookup1D": ("Table",),
            "LookupND": ("Table", "NumberOfTableDimensions"), "Prelookup": ("BreakpointsData",), "PrelookupF": ("BreakpointsData",),
            "InterpolationUsingPrelookup": ("Table",),
            "Lookup2D": ("Table",)}
# (Constant Value, Compare To Constant relop / const, Discrete FIR Coefficients, Saturation limits, Shift Arithmetic:
# library values from the library, JSON_LIBRARY_DEFAULTS / DEFAULTS. A required parameter left out whose library
# default is an expression the engine does not evaluate is refused as not supported, naming that default.)
REQUIRED_LIBRARY_TEXT = {("FIRDecimation", "h"): "designMultirateFIR(1,2)",
                         ("FIRInterpolation", "h"): "designMultirateFIR(3,1)",
                         ("Lookup1D", "Table"): "tanh([-5:5])",
                         ("Lookup2D", "Table"): "[4 5 6;16 19 20;10 18 23]"}

# recordings (recordings minimal / recordings): a JSON design that leaves
# a parameter out means the block as it comes from the Simulink library, whose defaults can differ from the values a
# saved .slx omits (DEFAULTS above: the .slx reader keeps those, e.g. recordings needs '>=' there). Recorded
# library defaults that differ: Relational Operator '<=' (int8 [1 2 3] vs [2 2 2] gives [1 1 0]), Switch
# 'u2 > Threshold' ([20 10 10]), MinMax one input (two inputs connected: Simulink InvSimulinkObjectName). Applied to
# JSON designs only.
#
# recordings (a new block of each type dragged from the R2026b library, every dialog
# parameter read with get_param; 175 pairs against DEFAULTS, 137 the same): the 30 that differ in value are all
# below (the four that differ only in spelling -- 0 / 0.0, '++' / '|++' -- are left as they are). The multirate
# library blocks default to single-rate processing, which Simulink rejects for these scalar uses and the
# engine does not take either; a JSON design that leaves those parameters out is refused the same way.
JSON_LIBRARY_DEFAULTS = {
    "RelationalOperator": {"Operator": "<="},
    "Delay": {"DelayLength": "2"},   # the library DelayLength -- was required
    # recordings (foundation completeness): these were refused MISSING_FIELD (a malformed request) when
    # left out, but the library block has a plain value that every reader here takes
    "Constant": {"Value": "1"},
    "Saturation": {"UpperLimit": "0.5", "LowerLimit": "-0.5"},
    "CompareToConstant": {"relop": "<=", "const": "3.0"},
    "DiscreteFir": {"OutDataTypeStr": "Inherit: Same as input", "TapSumDataTypeStr": "Inherit: Inherit via internal rule",
                    "Coefficients": "[0.5 0.5]"},
    "Gain": {"Gain": "1"},   # a Gain without Gain ended in BAD_PARAM "KeyError: 'Gain'" (library value 1)
    "Switch": {"Criteria": "u2 > Threshold"},
    "MinMax": {"Inputs": "1"},
    "MultiportSwitch": {"DiagnosticForDefault": "Error"},
    "CICDecimation": {"BPS": "[16 16 16 16]", "FLPS": "0", "outputWordLength": "32",
                      "RateOptions": "Enforce single-rate processing"},
    "CICInterpolation": {"BPS": "[16 16 16 16]", "FLPS": "0", "outputWordLength": "32"},
    "DiscreteIntegrator": {"SampleTime": "-1"},
    "Downsample": {"RateOptions": "Enforce single-rate processing"},
    "Upsample": {"UpsamplingFactor": "3", "RateOptions": "Enforce single-rate processing"},
    "FIRDecimation": {"FilterSource": "Auto", "framing": "Enforce single-rate processing", "outputMode": "Same as input",
                      "outputDataTypeStr": "Inherit: Same as input"},
    "FIRInterpolation": {"FilterSource": "Auto", "framing": "Enforce single-rate processing", "L": "3",
                         "outputMode": "Same as input", "outputDataTypeStr": "Inherit: Same as input"},
    "Lookup1D": {"BreakpointsForDimension1FirstPoint": "1", "ExtrapMethod": "Linear", "RndMeth": "Simplest"},
    "LookupND": {"ExtrapMethod": "Linear", "RndMeth": "Simplest"},
    # FractionDataTypeStr: the library's internal rule (32 bits on the host as before; the input's word on ASIC,
    # recordings -- fxcore.prelookup_fraction_type)
    "Prelookup": {"ExtrapMethod": "Linear", "FractionDataTypeStr": "Inherit: Inherit via internal rule"},
    "PrelookupF": {"ExtrapMethod": "Linear", "FractionDataTypeStr": "Inherit: Inherit via internal rule"},
    "InterpolationUsingPrelookup": {"ExtrapMethod": "Linear"},
    "Lookup2D": {"BreakpointsForDimension1FirstPoint": "1", "BreakpointsForDimension2FirstPoint": "1",
                 "ExtrapMethod": "Linear", "RndMeth": "Simplest"},
    "RateTransition": {"OutPortSampleTimeOpt": "Specify"},
    # recordings (recorded): the library HDL NCO dithers and takes its
    # increment from a port; the library Direct Lookup Table (n-D) is two-dimensional (engine/fxhdl.LIBRARY_DEFAULTS
    # applies them to the lowered blocks)
    "HDLNCO": {"DitherSource": "Property", "PhaseIncrementSource": "Input port"},
    "DirectLookupND": {"NumberOfTableDimensions": "2", "Table": "[4 5 6;16 19 20;10 18 23]"},   # + recordings table
}


# Library defaults of parameters the engine does not read but whose library value it does not support (review
# recordings): the library Downsample, Upsample, FIR Decimation and CIC Interpolation are frame based. Written out, that
# value is refused (META_VALUES); left out, a JSON design is refused the same way instead of being answered sample
# based (the frame-based scalar case is not recorded). FIR Interpolation is not listed: frame based + multirate is
# recorded equal. CIC Interpolation's rate option 'framing' (library 'Enforce single-rate processing') is
# accepted only at the recorded 'Allow multirate processing' (sample based, multirate);
# 'Maintain input frame rate' is recorded on CIC Decimation only (the regression check recordings; recordings corrected the
# earlier claim), per block type in META_VALUES_BY_TYPE; left out or any other value is refused.
_FRAME = "Columns as channels (frame based)"
JSON_LIBRARY_UNSUPPORTED = {
    "Downsample": {"InputProcessing": _FRAME},
    "Upsample": {"InputProcessing": _FRAME},
    "FIRDecimation": {"InputProcessing": _FRAME},
    "CICInterpolation": {"InputProcessing": _FRAME, "framing": "Enforce single-rate processing"},
}
# recordings (the dialog tables of freshly placed library blocks): parameters whose library value no
# recording shows. A JSON design that leaves one out is refused (an export must not write the engine's
# guess into the model as if it were the request). Manual Switch's position is not a dialog parameter -- the table has
# only varsize / SampleTime; a saved model leaves CurrentSetting out at the built-in 1, which is
# the .slx reader's evidence, but the library block's own value is not recorded.
JSON_LIBRARY_UNRECORDED = {"ManualSwitch": ("sw",)}
# the value with_saved_defaults writes for them (the engine's long-standing assumption)
_SAVED_UNSUPPORTED = {"InputProcessing": "Elements as channels (sample based)", "framing": "Allow multirate processing"}


def with_saved_defaults(design):
    """A copy of a JSON design with every left-out parameter of JSON_LIBRARY_DEFAULTS written out as the engine's
    earlier default (DEFAULTS). For designs the recordings wrote, generated or obtained from simulations before recordings
    -- their intent is those values -- never for replaying a Simulink recording (there a left-out parameter is the
    library's)."""
    import copy
    d = copy.deepcopy(design)
    for b in d.get("blocks", {}).values():
        if isinstance(b, dict):
            for k, v in JSON_LIBRARY_DEFAULTS.get(b.get("type"), {}).items():
                if k in DEFAULTS[b["type"]]:   # Gain's Gain had no earlier default (it was required)
                    b.setdefault(k, DEFAULTS[b["type"]][k])
            for k in JSON_LIBRARY_UNSUPPORTED.get(b.get("type"), {}):
                b.setdefault(k, _SAVED_UNSUPPORTED[k])
    return d


# ---------------------------------------------------------------- parameter whitelist
# Parameters the engine uses, beyond DEFAULTS / REQUIRED, per block type.
SEMANTIC_EXTRA = {"BitRotate": ("mode", "N"), "BitSet": ("iBit",), "BitClear": ("iBit",), "Gain": ("Gain",), "Saturation": ("UpperLimit", "LowerLimit", "OutDataTypeStr"),
                  # (no longer REQUIRED from the library -- library defaults -- but still read)
                  "CompareToConstant": ("relop", "const"), "DiscreteFir": ("Coefficients",),
                  "Constant": ("Value",), "Delay": ("DelayLength",), "Abs": ("OutDataTypeStr", "RndMeth"),
                  # Unary Minus has no OutDataTypeStr / RndMeth (Simulink ParamUnknown) -- BAD_PARAM
                  "RelationalOperator": ("OutDataTypeStr", "RndMeth"),
                  "LogicalOperator": ("OutDataTypeStr",), "DataTypeConversion": ("OutDataTypeStr",),
                  "Lookup1D": ("BreakpointsForDimension1", "Table"),
                  "LookupND": ("BreakpointsForDimension1", "BreakpointsForDimension2", "BreakpointsForDimension3",
                               "BreakpointsForDimension4", "Table", "NumberOfTableDimensions"),
                  "Lookup2D": ("BreakpointsForDimension1", "BreakpointsForDimension2", "Table"), "Inport": (), "Outport": ("OutDataTypeStr",),
                  "MultiportSwitch": ("DataPortIndices",),
                  "Sqrt": ("Iterations", "IntermediateResultsDataTypeStr", "RsqrtAlgorithmType", "AlgorithmType",
                           "SqrtAlgorithmType")}

# Simulink parameters without effect on a single-rate scalar fixed-point simulation: accepted, reported back
META_ANY = {"OutMin", "OutMax", "ParamMin", "ParamMax", "CoefMin", "CoefMax", "TableMin", "TableMax", "LockScale",
            "IconShape", "IconDisplay", "StateName", "StateStorageClass", "StateMustResolveToSignalObject",
            "CodeGenStateStorageTypeQualifier", "RTWStateStorageTypeQualifier", "SignalName", "StorageClass",
            "MustResolveToSignalObject", "ZeroCross", "DiagnosticForDelayLength", "RemoveDelayLengthCheckInGeneratedCode",
            "UseCircularBuffer", "DelayLengthUpperLimit", "Port", "Unit", "LinearizeAsGain", "CheckOORBitShift",
            "DiagnosticForOORShift", "CollapseDim",
            "IndexSearchMethod", "BeginIndexSearchUsingPreviousIndexResult", "MaximumIndicesForEachDimension",
            "BreakpointsForDimension1Min", "BreakpointsForDimension1Max", "InternalRulePriority", "FractionDataTypeStr",
            "TapSumDataTypeStr", "SignalType", "PortDimensions", "VarSizeSig", "BusOutputAsStruct", "BusVirtuality",
            "DataMode", "Description", "Tag", "Priority", "InputPortMap", "OutputPortMap", "Position", "ZOrder",
            "OutputSaturationStatus",
            "VectorParamsAs1DForOutWhenUnconnected", "OutputWhenUnconnectedValue", "InitialOutput", "OutputFunctionCall",
            "EnsureOutportIsVirtual", "OutputWhenDisabled", "OutputWhenUnConnected"}

# multirate blocks : parameters with no effect on a sample-based scalar simulation
# parameters that are harmless only at certain values (anything else changes behaviour the engine does not model)
# with 'Error' Simulink stops at the first input outside the
# breakpoints (Simulink:blocks:LookupNdOutofRangeInputError); the engine clamps and answers, so only None / Warning
# (both clamp, no effect on the values) are accepted
META_VALUES = {"DiagnosticForOutOfRangeInput": ("None", "Warning"), "CollapseMode": ("All dimensions",), "filtFrom": ("Dialog parameters",), "framing": ("Allow multirate processing", "Maintain input frame rate"), "InputProcessing": ("Elements as channels (sample based)", "Inherited"),
               "VectorParams1D": ("on", "off"), "AllowDiffInputSizes": ("off",), "AllPortsSameDT": ("on", "off"),
               "ExternalReset": ("None", "none"), "ShowEnablePort": ("off",), "PreventDirectFeedthrough": ("off",),
               "BitShiftNumberSource": ("Dialog",), "IgnoreLimit": ("off",), "ShowSaturationPort": ("off",),
               "ShowStatePort": ("off",), "InitialConditionSetting": ("Auto", "State"),
               "NumberOfTableDimensions": ("1",), "DataSpecification": ("Table and breakpoints",),
               "LookupTableObject": ("",), "BreakpointsForDimension1Source": ("Dialog",), "TableSource": ("Dialog",),
               "UseOneInputPortForAllInputData": ("off",), "RemoveProtectionInput": ("off",),
               "UseLastTableValue": ("off", "on"), "ApplyFullPrecisionForLinearInterpolation": ("off",),
               "ApplyAccurateEfficientRounding": ("off",), "SupportTunableTableSize": ("off",),
               "SupportTunableSize": ("off",), "InputSameDT": ("on", "off"),
               # scalar signals: element-wise and matrix multiplication coincide
               # "Input port" sources make it a different block (parameter sweep after recordings: they were
               # accepted and ignored on the JSON path); only the dialog sources are recorded
               "DelayLengthSource": ("Dialog",), "InitialConditionSource": ("Dialog", "internal"),
               "CoefSource": ("Dialog parameters",),

               "Multiplication": ("Element-wise(.*)", "Matrix(*)", "Element-wise(K.*u)", "Matrix(K*u)", "Matrix(u*K)",
                                  "Matrix(K*u) (u vector)")}

_UNUSED_DIM2 = re.compile(r"^(BreakpointsForDimension([3-9]|[12]\d|30)(DataTypeStr|FirstPoint|Spacing|Min|Max|Source)?|"
                          r"BreakpointsForDimension[12](Min|Max|Source)|N([1-9]|[12]\d|30))$")

_UNUSED_DIM = re.compile(r"^(BreakpointsForDimension([2-9]|[12]\d|30)(DataTypeStr|FirstPoint|Spacing|Min|Max|Source)?|N([1-9]|[12]\d|30))$")


# ================================================================ .slx reader (engine/fxslx_lib.py)
# Simulink BlockType -> engine block type, and the parameters that are carried over
# recorded configurations (lowered by engine/block_lowering.py): the engine parameters are the library dialog's
import block_lowering as _L211  # noqa: E402
DEFAULTS.update({k: dict(v) for k, v in _L211.DEFAULTS.items()})

TYPEMAP = {
    
    "SignalSpecification": ("SignalSpecification", ("Dimensions", "OutDataTypeStr")),
    "Math": ("Math", ("Operator", "OutDataTypeStr", "SaturateOnIntegerOverflow")),
    "Assertion": ("Assertion", ("Enabled", "AssertionFailFcn", "StopWhenAssertionFail")),
    "Bias": ("Bias", ("Bias", "SaturateOnIntegerOverflow")),
    "ManualSwitch": ("ManualSwitch", ("sw",)),
    "Reshape": ("Reshape", ("OutputDimensionality", "OutputDimensions")),
    "BusAssignment": ("BusAssignment", ("AssignedSignals",)),
    "Ground": ("Ground", ()),
    "Gain": ("Gain", ("Gain", "OutDataTypeStr", "ParamDataTypeStr", "RndMeth", "SaturateOnIntegerOverflow",
                      "Multiplication")),
    "Sum": ("Sum", ("Inputs", "OutDataTypeStr", "AccumDataTypeStr", "RndMeth", "SaturateOnIntegerOverflow")),
    "Product": ("Product", ("Inputs", "OutDataTypeStr", "RndMeth", "SaturateOnIntegerOverflow")),
    "DataTypeConversion": ("DataTypeConversion", ("OutDataTypeStr", "ConvertRealWorld", "RndMeth",
                                                  "SaturateOnIntegerOverflow")),
    "Saturate": ("Saturation", ("UpperLimit", "LowerLimit", "RndMeth", "OutDataTypeStr")),
    # RndMeth sets the lossy cast; recordings: recorded integer output types are carried
    "RelationalOperator": ("RelationalOperator", ("Operator", "RndMeth", "OutDataTypeStr")),
    "MinMax": ("MinMax", ("Function", "Inputs", "OutDataTypeStr", "RndMeth", "SaturateOnIntegerOverflow")),
    "Abs": ("Abs", ("OutDataTypeStr", "RndMeth", "SaturateOnIntegerOverflow")),
    "UnaryMinus": ("UnaryMinus", ("SaturateOnIntegerOverflow",)),
    "UnitDelay": ("UnitDelay", ("InitialCondition",)),
    # pure wiring, resolved away after flattening (v1 structural blocks)
    "Goto": ("Goto", ("GotoTag", "TagVisibility")),
    "Mux": ("Mux", ("Inputs",)),
    "Demux": ("Demux", ("Outputs",)),
    "Selector": ("Selector", ("Indices", "IndexOptions", "NumberOfDimensions", "IndexMode", "OutputSizes",
                              "InputPortWidth")),   # the library InputPortWidth is 3; read the saved one
    "From": ("From", ("GotoTag",)),
    # wiring only: marks where a 'scoped' Goto tag becomes visible (resolved away with From / Goto)
    "GotoTagVisibility": ("GotoTagVisibility", ("GotoTag",)),
    "DownSample": ("Downsample", ("DownsamplingFactor", "SampleOffset", "InitialConditions", "RateOptions")),
    "RateTransition": ("RateTransition", ("Integrity", "Deterministic", "InitialCondition", "OutPortSampleTimeOpt",
                                          "OutPortSampleTimeMultiple", "OutPortSampleTime")),
    # PortDimensions: a design input declared as a vector (engine/fxvector); dropping it read a vector input as a scalar
    "Inport": ("Inport", ("OutDataTypeStr", "PortDimensions")),
    "FromWorkspace": ("Inport", ("VariableName",)),   # a data source: the caller supplies its samples
    "Outport": ("Outport", ()),
    "Constant": ("Constant", ("Value", "OutDataTypeStr")),
    "Delay": ("Delay", ("DelayLength", "InitialCondition", "DelayLengthSource", "InitialConditionSource",
                        "ExternalReset", "ShowEnablePort", "DelayLengthUpperLimit")),
    "ArithShift": ("ShiftArithmetic", ("BitShiftNumber", "BinPtShiftNumber", "BitShiftDirection",
                                       "BitShiftNumberSource")),
    "Logic": ("LogicalOperator", ("Operator", "Inputs", "OutDataTypeStr")),
    "Switch": ("Switch", ("Criteria", "Threshold", "OutDataTypeStr", "RndMeth", "SaturateOnIntegerOverflow")),
    "MultiPortSwitch": ("MultiportSwitch", ("DataPortOrder", "Inputs", "DataPortForDefault", "DiagnosticForDefault", "OutDataTypeStr",
                                            "RndMeth", "SaturateOnIntegerOverflow", "DataPortIndices")),
    # n-D Lookup Table: Lookup1D / Lookup2D by NumberOfTableDimensions
    "Lookup_n-D": ("Lookup_n-D", ("NumberOfTableDimensions", "BreakpointsSpecification", "BreakpointsForDimension1", "BreakpointsForDimension2",
                          "BreakpointsForDimension1FirstPoint", "BreakpointsForDimension1Spacing",
                          "BreakpointsForDimension2FirstPoint", "BreakpointsForDimension2Spacing", "Table",
                          "InterpMethod", "ExtrapMethod", "UseLastTableValue", "InternalRulePriority",
                          "BreakpointsForDimension1DataTypeStr", "BreakpointsForDimension2DataTypeStr",
                          "TableDataTypeStr", "IntermediateResultsDataTypeStr", "FractionDataTypeStr",
                          "OutDataTypeStr", "RndMeth", "SaturateOnIntegerOverflow",
                          # 3 / 4 dimensions (read as LookupND)
                          "BreakpointsForDimension3", "BreakpointsForDimension4",
                          "BreakpointsForDimension3DataTypeStr", "BreakpointsForDimension4DataTypeStr")),
    "DiscreteIntegrator": ("DiscreteIntegrator", ("IntegratorMethod", "gainval", "InitialCondition",
                                                  "InitialConditionSource", "ExternalReset", "OutDataTypeStr",
                                                  "RndMeth", "SaturateOnIntegerOverflow", "LimitOutput",
                                                  "UpperSaturationLimit", "LowerSaturationLimit")),
    "Signum": ("Signum", ()),
    # recordings (engine/fxaction.py): Switch Case action subsystems, Merge, Data Store; If refused there
    "SwitchCase": ("SwitchCase", ("CaseConditions", "ShowDefaultCase")),
    "ActionPort": ("ActionPort", ("InitializeStates",)),
    "Merge": ("Merge", ("Inputs", "InitialOutput")),
    "DataStoreMemory": ("DataStoreMemory", ("DataStoreName", "InitialValue", "OutDataTypeStr")),
    "DataStoreRead": ("DataStoreRead", ("DataStoreName", "Priority", "IndexOptions", "Indices", "OutputSizes")),
    "DataStoreWrite": ("DataStoreWrite", ("DataStoreName", "Priority", "IndexOptions", "Indices", "OutputSizes")),
    "If": ("If", ()),
    # recordings (engine/fxzoh.py)
    "ZeroOrderHold": ("ZeroOrderHold", ("SampleTime",)),
    # recordings (engine/fxiir.py)
    "DiscreteFilter": ("DiscreteFilter", ("Numerator", "Denominator", "InitialStates", "InitialDenominatorStates",
                                          "FilterStructure", "a0EqualsOne", "RndMeth", "SaturateOnIntegerOverflow",
                                          "StateDataTypeStr", "NumCoefDataTypeStr", "DenCoefDataTypeStr",
                                          "NumProductDataTypeStr", "DenProductDataTypeStr", "NumAccumDataTypeStr",
                                          "DenAccumDataTypeStr", "MultiplicandDataTypeStr", "OutDataTypeStr")),
    "DiscreteTransferFcn": ("DiscreteTransferFcn", ("Numerator", "Denominator", "InitialStates",
                                                    "InitialDenominatorStates", "FilterStructure", "a0EqualsOne",
                                                    "RndMeth", "SaturateOnIntegerOverflow", "StateDataTypeStr",
                                                    "NumCoefDataTypeStr", "DenCoefDataTypeStr", "NumProductDataTypeStr",
                                                    "DenProductDataTypeStr", "NumAccumDataTypeStr",
                                                    "DenAccumDataTypeStr", "MultiplicandDataTypeStr", "OutDataTypeStr")),
    # recordings (engine/fxprelookup.py)
    "PreLookup": ("PreLookup", ("BreakpointsSpecification", "BreakpointsData", "BreakpointsFirstPoint",
                                "BreakpointsSpacing", "BreakpointsNumPoints", "BreakpointDataTypeStr",
                                "IndexDataTypeStr", "FractionDataTypeStr", "ExtrapMethod", "UseLastBreakpoint",
                                "RndMeth")),
    "Interpolation_n-D": ("Interpolation_n-D", ("NumberOfTableDimensions", "Table", "InterpMethod", "ExtrapMethod",
                                                "TableDataTypeStr", "OutDataTypeStr", "IntermediateResultsDataTypeStr",
                                                "RndMeth", "InternalRulePriority", "SaturateOnIntegerOverflow")),
    # a saved model writes the algorithm as RsqrtAlgorithmType; the engine reads it as
    # AlgorithmType -- dropping it answered a Newton-Raphson block as Exact
    "Sqrt": ("Sqrt", ("Operator", "AlgorithmType", "SqrtAlgorithmType", "RsqrtAlgorithmType", "Iterations", "IntermediateResultsDataTypeStr",
                      "OutDataTypeStr", "RndMeth", "SaturateOnIntegerOverflow")),
    "Rounding": ("Rounding", ("Operator",)),
    # complex converters (engine/fxhdl.py carries complex signals as two real parts)
    "RealImagToComplex": ("RealImagToComplex", ("Input", "ConstantPart")),
    "ComplexToRealImag": ("ComplexToRealImag", ("Output",)),
    # virtual buses: element names come from the lines feeding the Bus Creator
    "BusCreator": ("BusCreator", ("Inputs", "ElementNames")),
    "BusSelector": ("BusSelector", ("OutputSignals", "OutputAsBus")),
    # enabled subsystems: the reader turns an enabled subsystem's Outports into EnabledOutport
    "EnablePort": ("EnablePort", ("StatesWhenEnabling",)),
    # triggered / resettable subsystems (engine/fxtrigger.py)
    "TriggerPort": ("TriggerPort", ("TriggerType", "StatesWhenEnabling")),
    # recordings (engine/fxcordic.py)
    "ComplexToMagnitudeAngle": ("ComplexToMagnitudeAngle", ("Output", "ApproximationMethod", "NumberOfIterations",
                                                            "ScaleReciprocalGainFactor", "SampleTime")),
    "MagnitudeAngleToComplex": ("MagnitudeAngleToComplex", ("Input", "ApproximationMethod", "NumberOfIterations",
                                                            "ScaleReciprocalGainFactor", "SampleTime")),
    "ResetPort": ("ResetPort", ("ResetTriggerType",)),
    "StateControl": ("StateControl", ("StateControl",)),
    "EnabledOutport": ("EnabledOutport", ("OutputWhenDisabled", "InitialOutput")),
    # recordings (engine/fxhdl.py, engine/fxvector.py)
    "DataTypeDuplicate": ("DataTypeDuplicate", ("NumInputPorts",)),
    "Concatenate": ("Concatenate", ("NumInputs", "Mode", "ConcatenateDimension")),
    "LookupNDDirect": ("DirectLookupND", ("NumberOfTableDimensions", "InputsSelectThisObjectFromTable", "Table",
                                          "TableDataTypeStr", "DiagnosticForOutOfRangeInput", "RemoveProtectionInput")),
    "Trigonometry": ("Trigonometry", ("Operator", "ApproximationMethod", "NumberOfIterations")),
    "DeadZone": ("DeadZone", ("LowerValue", "UpperValue", "SaturateOnIntegerOverflow")),
    "RateLimiter": ("RateLimiter", ("RisingSlewLimit", "FallingSlewLimit", "InitialCondition", "SampleTimeMode")),
    "DiscreteFir": ("DiscreteFir", ("Coefficients", "CoefSource", "FilterStructure", "InitialStates",
                                    "CoefDataTypeStr", "ProductDataTypeStr", "AccumDataTypeStr",
                                    "StateDataTypeStr", "OutDataTypeStr", "RndMeth", "SaturateOnIntegerOverflow",
                                    # structures: the tap-sum type of the symmetric forms
                                    "TapSumDataTypeStr")),
}

# library (Reference) blocks recognised by the tail of their SourceBlock path
LIBRARY = {"Compare To Constant": ("CompareToConstant", ("relop", "const", "OutDataTypeStr")),
           # recordings (engine/fxiir.py): dsparch4/Biquad Filter
           "Biquad Filter": ("BiquadFilter", ('FilterSource', 'CoeffSource', 'FilterObject', 'IIRFiltStruct', 'BiQuadCoeffs', 'ScaleValues', 'IC', 'ICnum', 'ICden', 'a0Flag', 'optimizeScaleValues', 'ScaleValueMode', 'InputProcessing', 'roundingMode', 'overflowMode', 'stageInputMode', 'stageInputWordLength', 'stageInputFracLength', 'stageOutputMode', 'stageOutputWordLength', 'stageOutputFracLength', 'multiplicandMode', 'multiplicandWordLength', 'multiplicandFracLength', 'firstCoeffMode', 'firstCoeffWordLength', 'firstCoeffFracLength', 'secondCoeffMode', 'secondCoeffWordLength', 'secondCoeffFracLength', 'scaleValueFracLength', 'prodOutputMode', 'prodOutputWordLength', 'prodOutputFracLength', 'denProdOutputFracLength', 'accumMode', 'accumWordLength', 'accumFracLength', 'denAccumFracLength', 'memoryMode', 'memoryWordLength', 'memoryFracLength', 'secondMemoryFracLength', 'outputMode', 'outputWordLength', 'outputFracLength', 'LockScale', 'allowOverrides')),
           # recordings (simulink and hdlsllib versions alike)
           "Compare To Zero": ("CompareToZero", ("relop", "OutDataTypeStr")),
           "Bit Reduce": ("BitReduce", ("mode",)),
           "Increment Real World": ("IncrementRealWorld", ()), "Decrement Real World": ("DecrementRealWorld", ()),
           "Increment Stored Integer": ("IncrementStoredInteger", ()),
           "Decrement Stored Integer": ("DecrementStoredInteger", ()),
           # samptime read too: a changed sample time must reach the engine's check
           "Tapped Delay": ("TappedDelay", ("vinit", "NumDelays", "DelayOrder", "includeCurrent", "samptime")),
           "Tapped Delay Enabled Synchronous": ("TappedDelayEnabledSynchronous",
                                                ("vinit", "NumDelays", "DelayOrder", "includeCurrent", "samptime")),
           "Wrap To Zero": ("WrapToZero", ("Threshold",)),
           "Detect Rise Positive": ("DetectRisePositive", ("vinit", "OutDataTypeStr", "InputProcessing")),
           "Detect Rise Nonnegative": ("DetectRiseNonnegative", ("vinit", "OutDataTypeStr", "InputProcessing")),
           "Detect Fall Negative": ("DetectFallNegative", ("vinit", "OutDataTypeStr", "InputProcessing")),
           "Detect Fall Nonpositive": ("DetectFallNonpositive", ("vinit", "OutDataTypeStr", "InputProcessing")),
           # the mask parameters are all kept under InstanceData
           "Counter Limited": ("CounterLimited", ("uplimit", "tsamp")),
           "Counter Free-Running": ("CounterFreeRunning", ("NumBits", "tsamp")),
           "Detect Change": ("DetectChange", ("vinit", "OutDataTypeStr", "InputProcessing")),
           "Detect Increase": ("DetectIncrease", ("vinit", "OutDataTypeStr", "InputProcessing")),
           "Detect Decrease": ("DetectDecrease", ("vinit", "OutDataTypeStr", "InputProcessing")),
           "Bitwise Operator": ("BitwiseOperator", ("logicop", "UseBitMask", "NumInputPorts", "BitMask",
                                                   "BitMaskRealWorld")),
           "Extract Bits": ("ExtractBits", ("bitsToExtract", "numBits", "bitIdxRange", "outScalingMode")),
           # recorded configurations multirate library blocks (dspsigops / dspmlti4); parameter names from recordings (untested on a real
           # .slx until the recorded samples arrive)
           "Upsample": ("Upsample", ("UpsamplingFactor", "SampleOffset", "InitialConditions", "RateOptions")),
           "FIR Decimation": ("FIRDecimation", ("h", "D", "filtStruct", "roundingMode", "overflowMode",
                                                "outputBufInitCond", "firstCoeffMode", "firstCoeffDataTypeStr",
                                                "prodOutputMode", "prodOutputDataTypeStr", "accumMode",
                                                "accumDataTypeStr", "outputMode", "outputDataTypeStr")),
           "FIR Interpolation": ("FIRInterpolation", ("h", "L", "roundingMode", "overflowMode", "outputBufInitCond",
                                                      "firstCoeffMode", "firstCoeffDataTypeStr", "prodOutputMode",
                                                      "prodOutputDataTypeStr", "accumMode", "accumDataTypeStr",
                                                      "outputMode", "outputDataTypeStr")),
           "CIC Decimation": ("CICDecimation", ("R", "M", "N", "ftype", "filterInternals", "BPS", "FLPS",
                                                "outputWordLength", "outputFracLength", "RateOptions")),
           # HDL library blocks (mask values under InstanceData)
           # DSP HDL filters (dsphdlfiltering2 only; recordings)
           "Discrete FIR Filter": ("HDLFIRFilter", ('NumeratorSource', 'Numerator', 'NumeratorPrototype', 'FilterStructure', 'SerializationOption', 'NumCycles', 'NumberOfMultipliers', 'roundingMode', 'overflowMode', 'CoefficientsDataTypeStr', 'OutputDataTypeStr', 'LockScale', 'ResetInputPort', 'HDLGlobalReset', 'CoeffMultiplier', 'CoeffMultiplierSystolic', 'UseMultForPow2Coeff', 'FrameInputSymmOptim', 'SimulateUsing')),
           "NCO": ("HDLNCO", ('PhaseIncrementSource', 'PhaseIncrement', 'Frequency', 'FreqResolution', 'SamplingFreq', 'PhaseOffsetSource', 'PhaseOffset', 'PhaseOffsetR', 'DitherSource', 'NumDitherBits', 'SamplesPerFrame', 'LUTCompress', 'ResetAction', 'Waveform', 'PhasePort', 'RoundingMethod', 'OverflowAction', 'AccumulatorDataType', 'AccumulatorSigned', 'AccumulatorWL', 'AccumulatorFL', 'PhaseQuantization', 'NumQuantizerAccumulatorBits', 'OutputDataType', 'OutputSigned', 'OutputWL', 'OutputFL', 'SimulateUsing')),
           "FIR Decimator": ("HDLFIRDecimator", ('NumeratorSource', 'Numerator', 'NumeratorPrototype', 'FilterStructure', 'DecimationFactor', 'NumCycles', 'OptimizeSymmCoeff', 'roundingMode', 'overflowMode', 'CoefficientsDataTypeStr', 'OutputDataTypeStr', 'LockScale', 'ResetInputPort', 'HDLGlobalReset', 'SimulateUsing')),
           "HDL Counter": ("HDLCounter", ("CountType", "CountInit", "CountStep", "CountMax", "CountFromType", "CountFrom",
                                          "CountResetPort", "CountLoadPort", "CountEnbPort", "CountDirPort",
                                          "CountHitOutputPort", "CountDataType", "CountWordLen", "CountFracLen",
                                          "CountSampTime")),
           "Unit Delay Enabled Synchronous": ("UnitDelayEnabledSynchronous", ("InitialCondition", "SampleTime")),
           "Unit Delay Enabled Resettable Synchronous": ("UnitDelayEnabledResettableSynchronous",
                                                         ("InitialCondition", "SampleTime")),
           "Unit Delay Resettable Synchronous": ("UnitDelayResettableSynchronous", ("InitialCondition", "SampleTime")),
           "Bit Slice": ("BitSlice", ("lidx", "ridx")),
           "Bit Concat": ("BitConcat", ("numInputs",)),
           "Bit Shift": ("BitShift", ("mode", "N")),
           "Bit Rotate": ("BitRotate", ("mode", "N")),   
           "Bit Set": ("BitSet", ("iBit",)),
           "Bit Clear": ("BitClear", ("iBit",)),
           # masked HDL RAMs (the System versions take the address width from the address type: refused)
           "Simple Dual Port RAM": ("HDLRAM", ("ram_size",)),
           # the System RAMs: the address width is the address signal's
           "Simple Dual Port RAM System": ("HDLRAM", ("RAMType", "WriteOutputValue", "RAMInitialValue")),
           "Dual Port RAM System": ("HDLRAM", ("RAMType", "WriteOutputValue", "RAMInitialValue")),
           "Single Port RAM System": ("HDLRAM", ("RAMType", "WriteOutputValue", "RAMInitialValue")),
           "Dual Port RAM": ("HDLRAM", ("ram_size",)),
           "Single Port RAM": ("HDLRAM", ("ram_size", "dout_type")),
           # 
           "HDL FIFO": ("HDLFIFO", ("fifo_size", "mode", "ratio", "dataDim", "push_msg", "pop_msg", "show_empty",
                                    "show_full", "show_num", "rst_port")),
           
           "Detect Rise Positive": ("DetectRisePositive", ("vinit", "InputProcessing", "OutDataTypeStr")),
           "Repeat": ("Repeat", ("N", "ic", "InputProcessing", "RateOptions", "FactorSource")),
           # a test-bench source read as an Inport (fxslx_lib: sample based, literal Ts)
           "Signal From Workspace": ("Inport", ("SampleTime",)),
           "CIC Interpolation": ("CICInterpolation", ("R", "M", "N", "ftype", "filterInternals", "BPS", "FLPS",
                                                      "outputWordLength", "outputFracLength", "RateOptions"))}

# parameters whose value must be a fixed set for the engine to be faithful
MUST = {   # Delay: enable / reset ports lowered by recorded (engine/fxhdl.py _port_delay), a length / IC from a
        # port by main; the two together are refused by _port_delay
        # (was {"Delay": {"DelayLengthSource": "Dialog", "InitialConditionSource": "Dialog"}}: the enable / reset ports:
        # carried to the recorded lowering (engine/fxhdl.py _port_delay), which accepts the recorded ones; recorded
        # keeps the length / IC from the dialog (main answers port-sourced ones, recordings -- not combined
        # with recorded port delays here)
        # DiscreteIntegrator: reset / external initial condition read since recordings (checked by the
        # engine: fxgraph_lib _check_validated)
        # ArithShift BitShiftNumberSource: 'Input port' read since recordings (checked by the engine)
        # FilterStructure: the engine validates the structure itself (direct, transposed, symmetric, antisymmetric;
        # recordings; recordings) and refuses the others (lattice) with UNSUPPORTED
        "DiscreteFir": {"CoefSource": "Dialog parameters"},
        "EnablePort": {"ShowOutputPort": "off"}}

VIRTUAL = ("SubSystem",)

NUMERIC = {"Gain": ("Gain",), "Saturate": ("UpperLimit", "LowerLimit"), "UnitDelay": ("InitialCondition",),
           "Delay": ("DelayLength", "InitialCondition"), "Switch": ("Threshold",),
           "ArithShift": ("BitShiftNumber", "BinPtShiftNumber")}

VECTOR_OK = {"Gain": ("Gain",), "Saturate": ("UpperLimit", "LowerLimit"), "UnitDelay": ("InitialCondition",),
             "Delay": ("InitialCondition",)}

# Simulink's own defaults for parameters that a .slx omits when they keep their default value (only where the
# engine has no safe default of its own). Delay: DelayLength 2 -- a saved model records
# no DelayLength yet its initial condition is a width x 2 matrix, which Simulink accepts only with delay length 2
# (SimulinkBlocks:Delay:ICDimensionAndDelayLengthMismatch).
SLX_DEFAULTS = {"Delay": {"DelayLength": "2"},
                # a saved Prelookup omits ExtrapMethod and Simulink runs it on fixed point -- the
                # built-in value is Clip (the library block's Linear, which a JSON design that leaves it out gets, is
                # refused on fixed point); the saved Interpolation writes its Clip out
                "PreLookup": {"ExtrapMethod": "Clip"},
                # a saved Bias omits SaturateOnIntegerOverflow exactly when it is on (the
                # satoff models write 'off'); the library dialog's 'off' is not what an omitted one means
                "Bias": {"SaturateOnIntegerOverflow": "on"},
                # Discrete FIR: Simulink's own defaults where they differ from the engine's JSON defaults (recordings
                # dialog of the default-typed models: the output type is "Same as input", not "Same as accumulator";
                # the tap-sum type "Inherit via internal rule"); a model that omits them was read with the engine's
                # values before -- the output type silently wrong
                # (TapSumDataTypeStr also recordings.) Coefficients: recordings (b04 libcoef_*,
                # b01 recordings): a FIR left at the library coefficients saves none, and was refused MISSING_FIELD;
                # the built-in default [0.5 0.5] is recorded in recordings and
                # the recordings table, and the same designs with an explicit [0.5 0.5] equal Simulink (recordings rows)
                "DiscreteFir": {"OutDataTypeStr": "Inherit: Same as input",
                                "TapSumDataTypeStr": "Inherit: Inherit via internal rule",
                                "Coefficients": "[0.5 0.5]"},
                # built-in defaults of what recordings omit, each checked against the recorded
                # dialog of the same model (sample defaults)
                "DataTypeDuplicate": {"NumInputPorts": "2"},
                "Concatenate": {"NumInputs": "2", "Mode": "Vector"},
                "LookupNDDirect": {"NumberOfTableDimensions": "2", "InputsSelectThisObjectFromTable": "Element",
                                   "TableDataTypeStr": "Inherit: Inherit from 'Table data'",
                                   "DiagnosticForOutOfRangeInput": "Warning", "RemoveProtectionInput": "off"},
                # follow-up: a saved Selector without InputPortWidth ran on a width-4 Mux output
                # (equal), so the omitted value is not the library's 3; taken as
                # inherited. An omitted Indices is refused (_adapt_block): the built-in value is not recorded here
                "Selector": {"InputPortWidth": "-1"},
                # recorded samples (R2026b): a Data Type Conversion set to RndMeth 'Zero' saves no
                # RndMeth, one set to SaturateOnIntegerOverflow 'on' saves none, while Floor / off are written out
                #; the recorded outputs are reproduced with Zero / on. A Constant
                # with Value '1' saves no Value. (The JSON defaults stay the library block's Floor / off.)
                "DataTypeConversion": {"RndMeth": "Zero", "SaturateOnIntegerOverflow": "on"},
                # recordings (get_param of built-in/<type>, what a
                # saved file omits; recordings show): Switch, Multiport Switch and Abs
                # saturate by default; a Discrete FIR's omitted output type is 'Same as input' (DEFAULTS keeps the
                # engine's older 'Same as accumulator' for JSON designs written against it)
                "Switch": {"SaturateOnIntegerOverflow": "on"},
                # recordings (get_param of built-in/<type>):
                # Multiport Switch stops on an out-of-range control by default ('Error'; the reader assumed 'None'),
                # a MinMax has one input, a Saturation limits +-0.5, a Gain 1, a Shift Arithmetic 0 bits
                # bidirectional (these three were refused as missing), RateTransition 'Specify'
                "MultiPortSwitch": {"SaturateOnIntegerOverflow": "on", "DiagnosticForDefault": "Error"},
                "MinMax": {"Inputs": "1"},
                "Saturate": {"UpperLimit": "0.5", "LowerLimit": "-0.5"},
                "Gain": {"Gain": "1"},
                "ArithShift": {"BitShiftNumber": "0", "BinPtShiftNumber": "0", "BitShiftDirection": "Bidirectional"},
                "RateTransition": {"OutPortSampleTimeOpt": "Specify"},
                "Abs": {"SaturateOnIntegerOverflow": "on"},
                "Constant": {"Value": "1"},
                # n-D Lookup Table (the 1-D files save NumberOfTableDimensions 1, the 2-D ones
                # omit it; the others are Simulink's dialog defaults, checked against the recorded outputs where they
                # matter, see the regression check). InternalRulePriority: the files write "Speed" out, so it
                # is not the default and the default is not known -- a sentinel the inherited-type rules refuse.
                "Lookup_n-D": {"NumberOfTableDimensions": "2", "BreakpointsSpecification": "Explicit values",
                               "InterpMethod": "Linear point-slope", "ExtrapMethod": "Linear",
                               "UseLastTableValue": "off", "SaturateOnIntegerOverflow": "off",
                               # RndMeth Floor: three files write "Simplest" out (so it is not the default) and the
                               # two files that omit it are reproduced by Floor (and by Simplest), 200/200 each
                               "RndMeth": "Floor",
                               # recordings (evidence/defaults/matlab.jsonl): the built-in default is 'Precision'
                               "InternalRulePriority": "Precision"}}

# parameters a model must set explicitly because their defaults are unsupported
SLX_UNKNOWN_DEFAULT = {"Lookup_n-D": ("Table",)}

# block types whose SampleTime is a dialog parameter; absent -> reported as an assumed default
HAS_SAMPLE_TIME = ("UnitDelay", "Delay", "Constant", "FromWorkspace", "DiscreteFir", "Inport")

# parameters that only describe layout / presentation (never reported as ignored)
COSMETIC = {"Position", "ZOrder", "Port", "ContentPreviewEnabled", "BackgroundColor", "ForegroundColor",
            "ShowName", "NamePlacement", "FontName", "FontSize", "FontWeight", "FontAngle", "Orientation",
            "DropShadow", "HideAutomaticName", "LibraryVersion", "SourceType", "SourceBlock", "InputPortMap",
            "OutputPortMap", "Description", "Tag", "AttributesFormatString", "BlockMirror", "BlockRotation",
            "Commented", "Priority", "IconDisplay", "Name", "SID"}


# ================================================================ optimiser type roles (engine/fxgopt_lib.py)
import fxrecords as _R  # noqa: E402
# Decision-variable type parameters. Blocks with a compiled record (Sum, Gain, Product, Discrete FIR) take them from
# the record's TYPE_ROLES -- one source for the optimiser and the compiled types; the rest are listed here.
VARBLOCKS = {**{bt: tuple(k for k, _ in rc.TYPE_ROLES) for bt, rc in _R.RECORD_OF.items()},
             "DataTypeConversion": ("OutDataTypeStr",),
             "MinMax": ("OutDataTypeStr",), "Switch": ("OutDataTypeStr",), "MultiportSwitch": ("OutDataTypeStr",),
             "Constant": ("OutDataTypeStr",),
             "Lookup1D": ("OutDataTypeStr",), "Lookup2D": ("OutDataTypeStr",),
             # explicit output types validated by recordings
             "Saturation": ("OutDataTypeStr",), "Abs": ("OutDataTypeStr",)}

INNER = ("ParamDataTypeStr", "AccumDataTypeStr", "CoefDataTypeStr", "ProductDataTypeStr")  # weight 1 each

LUTS = ("Lookup1D", "Lookup2D")

# 1-D Lookup Table: every stored or computed type is a decision variable and written as an explicit fixdt
# (inherited breakpoint / fraction types left the cost UNRESOLVED and above the bound).
# Nearest keeps the breakpoint type (the engine requires the input's type there, Simulink:blocks:
# PreLookupNearestUAndBpDataTypeMismatch).
LUT1D_BP, LUT1D_TAB, LUT1D_IT, LUT1D_FR = ("BreakpointsForDimension1DataTypeStr", "TableDataTypeStr",
                                           "IntermediateResultsDataTypeStr", "FractionDataTypeStr")

LUT2D_BP = ("BreakpointsForDimension1DataTypeStr", "BreakpointsForDimension2DataTypeStr")


# Numeric value parameters: parsed by one function (engine/fxnum.py) before any block sees them --
# literals and bracketed lists of literals accepted (canonical form), MATLAB expressions UNSUPPORTED, anything else
# BAD_PARAM. 'scalar' must be one value; 'any' may be a list or matrix.
# port-count parameters (read by int() / str.isdigit(), which take '0_3' and non-ASCII digits as 3):
# block -> (parameter, operator characters kept as written -- '' when the parameter is a count only)
PORT_COUNTS = {"Sum": ("Inputs", "+-|"), "Product": ("Inputs", "*/|"), "MinMax": ("Inputs", ""),
               "LogicalOperator": ("Inputs", ""), "MultiportSwitch": ("Inputs", ""),
               "BitwiseOperator": ("NumInputPorts", ""), "Mux": ("Inputs", "")}

NUMERIC_VALUES = {"Gain": {"Gain": "any"}, "Saturation": {"UpperLimit": "any", "LowerLimit": "any"},
           "UnitDelay": {"InitialCondition": "any"}, "Delay": {"InitialCondition": "any", "DelayLength": "int"},
           "DiscreteFir": {"Coefficients": "vector", "InitialStates": "vector"},
           "DiscreteIntegrator": {"InitialCondition": "scalar", "gainval": "scalar", "UpperSaturationLimit": "scalar",
                                  "LowerSaturationLimit": "scalar"},
           "Switch": {"Threshold": "scalar"},
           "DeadZone": {"LowerValue": "scalar", "UpperValue": "scalar"}, "Trigonometry": {"NumberOfIterations": "int"}, "WrapToZero": {"Threshold": "scalar"},
           "RateLimiter": {"RisingSlewLimit": "scalar", "FallingSlewLimit": "scalar", "InitialCondition": "scalar"}, "CompareToConstant": {"const": "typed"}, "Constant": {"Value": "typed_any"},
           "CounterLimited": {"uplimit": "int"}, "CounterFreeRunning": {"NumBits": "int"},
           "DetectChange": {"vinit": "scalar"}, "DetectIncrease": {"vinit": "scalar"},
           "DetectRisePositive": {"vinit": "scalar"}, "DetectRiseNonnegative": {"vinit": "scalar"},
           "DetectFallNegative": {"vinit": "scalar"}, "DetectFallNonpositive": {"vinit": "scalar"},
           "DetectDecrease": {"vinit": "scalar"},
           "Hold": {"InitialCondition": "scalar"}, "RateTransition": {"InitialCondition": "scalar"},
           "Downsample": {"InitialConditions": "scalar", "DownsamplingFactor": "int", "SampleOffset": "int"},
           "Upsample": {"InitialConditions": "scalar", "UpsamplingFactor": "int", "SampleOffset": "int"},
           "Prelookup": {"BreakpointsData": "vector"},
           "PrelookupF": {"BreakpointsData": "vector"},
           "InterpolationUsingPrelookup": {"Table": "any", "NumberOfTableDimensions": "int"},
           "LookupND": dict({"Table": "any", "NumberOfTableDimensions": "int"},
                            **{"BreakpointsForDimension%d" % k: "vector" for k in range(1, 5)}),
           "Lookup1D": {"Table": "vector", "BreakpointsForDimension1": "vector",
                        "BreakpointsForDimension1FirstPoint": "scalar", "BreakpointsForDimension1Spacing": "scalar"},
           "Lookup2D": {"Table": "any", "BreakpointsForDimension1": "vector", "BreakpointsForDimension2": "vector",
                        "BreakpointsForDimension1FirstPoint": "scalar", "BreakpointsForDimension1Spacing": "scalar",
                        "BreakpointsForDimension2FirstPoint": "scalar", "BreakpointsForDimension2Spacing": "scalar"},
           "FIRDecimation": {"h": "vector", "outputBufInitCond": "scalar", "D": "int"},
           "FIRInterpolation": {"h": "vector", "outputBufInitCond": "scalar", "L": "int"},
           "CICDecimation": {"R": "int", "M": "int", "N": "int", "outputWordLength": "int", "outputFracLength": "int",
                             "BPS": "vector", "FLPS": "vector"},
           "CICInterpolation": {"R": "int", "M": "int", "N": "int", "outputWordLength": "int",
                                "outputFracLength": "int", "BPS": "vector", "FLPS": "vector"},
           "ExtractBits": {"numBits": "int"},   # bitIdxRange: read (and checked) only in 'Range of bits' mode
           "ShiftArithmetic": {"BitShiftNumber": "int", "BinPtShiftNumber": "int"}}


# ================================================================ cross-table rules
STRUCTURAL = ("Inport", "Outport", "Goto", "From", "GotoTagVisibility", "Mux", "Demux", "Selector", "BusCreator",
              "BusSelector")
SLX_ADAPTED = {"Lookup_n-D": ("Lookup1D", "Lookup2D")}   # one .slx block type, engine type chosen by the adapter


def engine_params(et):
    """Every parameter name the engine accepts for engine block type et."""
    return (set(DEFAULTS.get(et, {})) | set(REQUIRED.get(et, ())) | set(SEMANTIC_EXTRA.get(et, ())) | set(META_ANY)
            | set(META_VALUES) | {"SampleTime", "type"})


# block types recorded lowers before the engine sees the design (engine/fxaction.py, engine/fxprelookup.py,
# engine/fxtrigger.py, the complex pass of engine/fxhdl.py with engine/fxcordic.py): read from .slx, checked by their
# own pass, never engine blocks themselves
LOWERED_FIRST = ("SwitchCase", "ActionPort", "Merge", "DataStoreMemory", "DataStoreRead", "DataStoreWrite", "If",
                 "PreLookup", "Interpolation_n-D", "DiscreteFilter", "DiscreteTransferFcn",
                 "ZeroOrderHold", "BiquadFilter", "TriggerPort", "ResetPort", "ComplexToMagnitudeAngle",
                 "MagnitudeAngleToComplex")


def check():
    """Problems between the tables (empty list = consistent)."""
    bad = []
    for table, entries in (("TYPEMAP", TYPEMAP), ("LIBRARY", LIBRARY)):
        for src, (et, keys) in entries.items():
            targets = SLX_ADAPTED.get(src, (et,))
            for t in targets:
                if t not in DEFAULTS and t not in STRUCTURAL and t not in LOWERED_FIRST:
                    bad.append("%s %s -> %s: not an engine block type" % (table, src, t))
                    continue
                if t in STRUCTURAL or t in LOWERED_FIRST:
                    continue
                # parameters the adapter consumes itself (NumberOfTableDimensions, tsamp, InputProcessing, ...)
                consumed = {"NumberOfTableDimensions", "tsamp", "InputProcessing", "InitialConditionSource",
                            "ExternalReset", "VariableName", "SampleTime",
                            "ram_size", "dout_type"}   # the masked RAMs' mask values, converted by the reader
                extra = sorted(k for k in keys if k not in engine_params(t) and k not in consumed
                               and not _UNUSED_DIM2.match(k) and not re.match(r"BreakpointsForDimension2", k))
                if extra and t != "Lookup1D":
                    bad.append("%s %s -> %s carries parameters the engine does not know: %s" % (table, src, t, extra))
    for et, keys in VARBLOCKS.items():
        for k in keys:
            if k not in engine_params(et):
                bad.append("VARBLOCKS %s.%s is not an engine parameter of %s" % (et, k, et))
    for et, rc in _R.RECORD_OF.items():   # every record type role names a record field holding a type
        fields = getattr(rc, "__dataclass_fields__", {})
        for k, f in rc.TYPE_ROLES:
            if f not in fields:
                bad.append("record %s type role %s -> %s is not a field" % (rc.__name__, k, f))
            if k not in engine_params(et):
                bad.append("record %s type role %s is not an engine parameter of %s" % (rc.__name__, k, et))
    for et in ("Lookup1D", "Lookup2D"):
        for k in (LUT1D_TAB, LUT1D_IT, LUT1D_FR) + ((LUT1D_BP,) if et == "Lookup1D" else LUT2D_BP):
            if k not in engine_params(et):
                bad.append("lookup type role %s.%s is not an engine parameter" % (et, k))
    return bad


# parameter names and enumerated values per block type (they were matched by name across
# all block types). LIB_DIALOG_PARAMS: the parameters of each library block's dialog (
# libdefaults.json "dialog"); a META_ANY / META_VALUES name outside the block's own dialog is BAD_PARAM.
# META_VALUES_BY_TYPE: the values accepted per block type = META_VALUES limited to the library default and the values
# recorded for that block type in recordings (scan of every accepted recordings record); InputSameDT /
# AllPortsSameDT stay as they are (semantic, per-type rules of recordings). Generated once; edit by hand only with
# a recording behind it -- or, as for Gain / Product Multiplication, the block's own options where the modes coincide
# for scalars (vector signals in a matrix mode are refused by the vector pass).
# Parameters a recording shows Simulink accepting on a block but which this engine does not read (so they are refused
# as not supported, not as an unknown name): Prelookup ProcessOutOfRangeInput 'Clip to range'
# (16 models run).
RECORDED_VALID_PARAMS = {"Prelookup": frozenset(["ProcessOutOfRangeInput"])}
LIB_DIALOG_PARAMS = {
    'Abs': frozenset(['LockScale', 'OutDataTypeStr', 'OutMax', 'OutMin', 'RndMeth', 'SampleTime', 'SaturateOnIntegerOverflow', 'ZeroCross']),
    'BitwiseOperator': frozenset(['BitMask', 'BitMaskRealWorld', 'NumInputPorts', 'UseBitMask', 'logicop']),
    'CICDecimation': frozenset(['BPS', 'FLPS', 'M', 'N', 'R', 'RateOptions', 'filtFrom', 'filterInternals', 'filtobj', 'framing', 'ftype', 'launchFVT', 'outputFracLength', 'outputWordLength']),
    'CICInterpolation': frozenset(['BPS', 'FLPS', 'InputProcessing', 'M', 'N', 'OutputFrames', 'R', 'filtFrom', 'filterInternals', 'filtobj', 'framing', 'ftype', 'launchFVT', 'outputFracLength', 'outputWordLength']),
    'CompareToConstant': frozenset(['OutDataTypeStr', 'ZeroCross', 'const', 'relop']),
    'CompareToZero': frozenset(['OutDataTypeStr', 'ZeroCross', 'relop']),
    # dialogs
    'DeadZone': frozenset(['LowerValue', 'UpperValue', 'SaturateOnIntegerOverflow', 'LinearizeAsGain', 'ZeroCross',
                           'SampleTime']),
    'RateLimiter': frozenset(['RisingSlewLimit', 'FallingSlewLimit', 'SampleTimeMode', 'InitialCondition',
                              'LinearizeAsGain']),
    'WrapToZero': frozenset(['Threshold']),
    'Trigonometry': frozenset(['Operator', 'ApproximationMethod', 'NumberOfIterations', 'OutputSignalType',
                               'SampleTime']),
    'DetectRisePositive': frozenset(['InputProcessing', 'OutDataTypeStr', 'vinit']),
    'DetectRiseNonnegative': frozenset(['InputProcessing', 'OutDataTypeStr', 'vinit']),
    'DetectFallNegative': frozenset(['InputProcessing', 'OutDataTypeStr', 'vinit']),
    'DetectFallNonpositive': frozenset(['InputProcessing', 'OutDataTypeStr', 'vinit']),
    'Constant': frozenset(['LockScale', 'OutDataTypeStr', 'OutMax', 'OutMin', 'SampleTime', 'Value', 'VectorParams1D']),
    'CounterFreeRunning': frozenset(['NumBits', 'tsamp']),
    'CounterLimited': frozenset(['tsamp', 'uplimit']),
    'DataTypeConversion': frozenset(['ConvertRealWorld', 'LockScale', 'OutDataTypeStr', 'OutMax', 'OutMin', 'RndMeth', 'SampleTime', 'SaturateOnIntegerOverflow']),
    'Delay': frozenset(['CodeGenStateStorageTypeQualifier', 'DelayLength', 'DelayLengthSource', 'DelayLengthUpperLimit', 'DiagnosticForDelayLength', 'ExternalReset', 'InitialCondition', 'InitialConditionSource', 'InputProcessing', 'PreventDirectFeedthrough', 'RemoveDelayLengthCheckInGeneratedCode', 'SampleTime', 'ShowEnablePort', 'StateMustResolveToSignalObject', 'StateName', 'StateSignalObject', 'StateStorageClass', 'UseCircularBuffer']),
    'DelayPort': frozenset(['CodeGenStateStorageTypeQualifier', 'DelayLength', 'DelayLengthSource', 'DelayLengthUpperLimit', 'DiagnosticForDelayLength', 'ExternalReset', 'InitialCondition', 'InitialConditionSource', 'InputProcessing', 'PreventDirectFeedthrough', 'RemoveDelayLengthCheckInGeneratedCode', 'SampleTime', 'ShowEnablePort', 'StateMustResolveToSignalObject', 'StateName', 'StateSignalObject', 'StateStorageClass', 'UseCircularBuffer']),
    'DetectChange': frozenset(['InputProcessing', 'OutDataTypeStr', 'vinit']),
    'DetectDecrease': frozenset(['InputProcessing', 'OutDataTypeStr', 'vinit']),
    'DetectIncrease': frozenset(['InputProcessing', 'OutDataTypeStr', 'vinit']),
    'DiscreteFir': frozenset(['AccumDataTypeStr', 'CoefDataTypeStr', 'CoefMax', 'CoefMin', 'CoefSource', 'Coefficients', 'ExternalReset', 'FilterStructure', 'InitialStates', 'InputProcessing', 'LockScale', 'OutDataTypeStr', 'OutMax', 'OutMin', 'ProductDataTypeStr', 'RTWStateStorageTypeQualifier', 'RndMeth', 'SampleTime', 'SaturateOnIntegerOverflow', 'ShowEnablePort', 'StateDataTypeStr', 'StateMustResolveToSignalObject', 'StateName', 'StateSignalObject', 'StateStorageClass', 'TapSumDataTypeStr']),
    'DiscreteIntegrator': frozenset(['ExternalReset', 'IgnoreLimit', 'InitialCondition', 'InitialConditionSetting', 'InitialConditionSource', 'IntegratorMethod', 'LimitOutput', 'LockScale', 'LowerSaturationLimit', 'OutDataTypeStr', 'OutMax', 'OutMin', 'RTWStateStorageTypeQualifier', 'RndMeth', 'SampleTime', 'SaturateOnIntegerOverflow', 'ShowSaturationPort', 'ShowStatePort', 'StateMustResolveToSignalObject', 'StateName', 'StateSignalObject', 'StateStorageClass', 'UpperSaturationLimit', 'gainval']),
    'DTIStateFB': frozenset(['ExternalReset', 'IgnoreLimit', 'InitialCondition', 'InitialConditionSetting', 'InitialConditionSource', 'IntegratorMethod', 'LimitOutput', 'LockScale', 'LowerSaturationLimit', 'OutDataTypeStr', 'OutMax', 'OutMin', 'RTWStateStorageTypeQualifier', 'RndMeth', 'SampleTime', 'SaturateOnIntegerOverflow', 'ShowSaturationPort', 'ShowStatePort', 'StateMustResolveToSignalObject', 'StateName', 'StateSignalObject', 'StateStorageClass', 'UpperSaturationLimit', 'gainval', '_SFGain', '_SFGainOut']),
    'DTIStateFBGain': frozenset(['ExternalReset', 'IgnoreLimit', 'InitialCondition', 'InitialConditionSetting', 'InitialConditionSource', 'IntegratorMethod', 'LimitOutput', 'LockScale', 'LowerSaturationLimit', 'OutDataTypeStr', 'OutMax', 'OutMin', 'RTWStateStorageTypeQualifier', 'RndMeth', 'SampleTime', 'SaturateOnIntegerOverflow', 'ShowSaturationPort', 'ShowStatePort', 'StateMustResolveToSignalObject', 'StateName', 'StateSignalObject', 'StateStorageClass', 'UpperSaturationLimit', 'gainval', '_SFGain', '_SFGainOut']),
    'Downsample': frozenset(['AllowArbitraryFixedInput', 'DownsamplingFactor', 'FactorSource', 'InitialConditions', 'InputProcessing', 'RateOptions', 'SampleOffset']),
    'ExtractBits': frozenset(['bitIdxRange', 'bitsToExtract', 'numBits', 'outScalingMode']),
    'FIRDecimation': frozenset(['AllowArbitraryInputLength', 'D', 'FilterObject', 'FilterSource', 'InputProcessing', 'LockScale', 'accumDataTypeStr', 'accumFracLength', 'accumLastDataTypeStr', 'accumMode', 'accumWordLength', 'filtStruct', 'firstCoeffDataTypeStr', 'firstCoeffFracLength', 'firstCoeffLastDataTypeStr', 'firstCoeffMax', 'firstCoeffMin', 'firstCoeffMode', 'firstCoeffWordLength', 'framing', 'h', 'outputBufInitCond', 'outputDataTypeStr', 'outputFracLength', 'outputLastDataTypeStr', 'outputMax', 'outputMin', 'outputMode', 'outputWordLength', 'overflowMode', 'prodOutputDataTypeStr', 'prodOutputFracLength', 'prodOutputLastDataTypeStr', 'prodOutputMode', 'prodOutputWordLength', 'roundingMode']),
    'FIRInterpolation': frozenset(['FilterObject', 'FilterSource', 'InputProcessing', 'L', 'LockScale', 'accumDataTypeStr', 'accumFracLength', 'accumLastDataTypeStr', 'accumMode', 'accumWordLength', 'firstCoeffDataTypeStr', 'firstCoeffFracLength', 'firstCoeffLastDataTypeStr', 'firstCoeffMax', 'firstCoeffMin', 'firstCoeffMode', 'firstCoeffWordLength', 'framing', 'h', 'outputBufInitCond', 'outputDataTypeStr', 'outputFracLength', 'outputLastDataTypeStr', 'outputMax', 'outputMin', 'outputMode', 'outputWordLength', 'overflowMode', 'prodOutputDataTypeStr', 'prodOutputFracLength', 'prodOutputLastDataTypeStr', 'prodOutputMode', 'prodOutputWordLength', 'roundingMode']),
    'Gain': frozenset(['Gain', 'LockScale', 'Multiplication', 'OutDataTypeStr', 'OutMax', 'OutMin', 'ParamDataTypeStr', 'ParamMax', 'ParamMin', 'RndMeth', 'SampleTime', 'SaturateOnIntegerOverflow']),
    'Inport': frozenset(['BusOutputAsStruct', 'BusVirtuality', 'DataMode', 'ExecSpecEvents', 'IconDisplay', 'InputConnect', 'Interpolate', 'LatchByDelayingOutsideSignal', 'LatchInputForFeedbackSignals', 'LockScale', 'MessageQueueCapacity', 'MessageQueueOverwriting', 'MessageQueueType', 'MessageQueueUseDefaultAttributes', 'OutDataTypeStr', 'OutMax', 'OutMin', 'OutputFunctionCall', 'Port', 'PortDimensions', 'SampleTime', 'SignalType', 'Unit', 'VarSizeSig']),
    'LogicalOperator': frozenset(['AllPortsSameDT', 'IconShape', 'Inputs', 'Operator', 'OutDataTypeStr', 'SampleTime']),
    'Lookup1D': frozenset(['ApplyAccurateEfficientRounding', 'ApplyFullPrecisionForLinearInterpolation', 'BeginIndexSearchUsingPreviousIndexResult', 'BreakpointsForDimension1', 'BreakpointsForDimension10', 'BreakpointsForDimension10DataTypeStr', 'BreakpointsForDimension10FirstPoint', 'BreakpointsForDimension10Max', 'BreakpointsForDimension10Min', 'BreakpointsForDimension10Spacing', 'BreakpointsForDimension11', 'BreakpointsForDimension11DataTypeStr', 'BreakpointsForDimension11FirstPoint', 'BreakpointsForDimension11Max', 'BreakpointsForDimension11Min', 'BreakpointsForDimension11Spacing', 'BreakpointsForDimension12', 'BreakpointsForDimension12DataTypeStr', 'BreakpointsForDimension12FirstPoint', 'BreakpointsForDimension12Max', 'BreakpointsForDimension12Min', 'BreakpointsForDimension12Spacing', 'BreakpointsForDimension13', 'BreakpointsForDimension13DataTypeStr', 'BreakpointsForDimension13FirstPoint', 'BreakpointsForDimension13Max', 'BreakpointsForDimension13Min', 'BreakpointsForDimension13Spacing', 'BreakpointsForDimension14', 'BreakpointsForDimension14DataTypeStr', 'BreakpointsForDimension14FirstPoint', 'BreakpointsForDimension14Max', 'BreakpointsForDimension14Min', 'BreakpointsForDimension14Spacing', 'BreakpointsForDimension15', 'BreakpointsForDimension15DataTypeStr', 'BreakpointsForDimension15FirstPoint', 'BreakpointsForDimension15Max', 'BreakpointsForDimension15Min', 'BreakpointsForDimension15Spacing', 'BreakpointsForDimension16', 'BreakpointsForDimension16DataTypeStr', 'BreakpointsForDimension16FirstPoint', 'BreakpointsForDimension16Max', 'BreakpointsForDimension16Min', 'BreakpointsForDimension16Spacing', 'BreakpointsForDimension17', 'BreakpointsForDimension17DataTypeStr', 'BreakpointsForDimension17FirstPoint', 'BreakpointsForDimension17Max', 'BreakpointsForDimension17Min', 'BreakpointsForDimension17Spacing', 'BreakpointsForDimension18', 'BreakpointsForDimension18DataTypeStr', 'BreakpointsForDimension18FirstPoint', 'BreakpointsForDimension18Max', 'BreakpointsForDimension18Min', 'BreakpointsForDimension18Spacing', 'BreakpointsForDimension19', 'BreakpointsForDimension19DataTypeStr', 'BreakpointsForDimension19FirstPoint', 'BreakpointsForDimension19Max', 'BreakpointsForDimension19Min', 'BreakpointsForDimension19Spacing', 'BreakpointsForDimension1DataTypeStr', 'BreakpointsForDimension1FirstPoint', 'BreakpointsForDimension1Max', 'BreakpointsForDimension1Min', 'BreakpointsForDimension1Source', 'BreakpointsForDimension1Spacing', 'BreakpointsForDimension2', 'BreakpointsForDimension20', 'BreakpointsForDimension20DataTypeStr', 'BreakpointsForDimension20FirstPoint', 'BreakpointsForDimension20Max', 'BreakpointsForDimension20Min', 'BreakpointsForDimension20Spacing', 'BreakpointsForDimension21', 'BreakpointsForDimension21DataTypeStr', 'BreakpointsForDimension21FirstPoint', 'BreakpointsForDimension21Max', 'BreakpointsForDimension21Min', 'BreakpointsForDimension21Spacing', 'BreakpointsForDimension22', 'BreakpointsForDimension22DataTypeStr', 'BreakpointsForDimension22FirstPoint', 'BreakpointsForDimension22Max', 'BreakpointsForDimension22Min', 'BreakpointsForDimension22Spacing', 'BreakpointsForDimension23', 'BreakpointsForDimension23DataTypeStr', 'BreakpointsForDimension23FirstPoint', 'BreakpointsForDimension23Max', 'BreakpointsForDimension23Min', 'BreakpointsForDimension23Spacing', 'BreakpointsForDimension24', 'BreakpointsForDimension24DataTypeStr', 'BreakpointsForDimension24FirstPoint', 'BreakpointsForDimension24Max', 'BreakpointsForDimension24Min', 'BreakpointsForDimension24Spacing', 'BreakpointsForDimension25', 'BreakpointsForDimension25DataTypeStr', 'BreakpointsForDimension25FirstPoint', 'BreakpointsForDimension25Max', 'BreakpointsForDimension25Min', 'BreakpointsForDimension25Spacing', 'BreakpointsForDimension26', 'BreakpointsForDimension26DataTypeStr', 'BreakpointsForDimension26FirstPoint', 'BreakpointsForDimension26Max', 'BreakpointsForDimension26Min', 'BreakpointsForDimension26Spacing', 'BreakpointsForDimension27', 'BreakpointsForDimension27DataTypeStr', 'BreakpointsForDimension27FirstPoint', 'BreakpointsForDimension27Max', 'BreakpointsForDimension27Min', 'BreakpointsForDimension27Spacing', 'BreakpointsForDimension28', 'BreakpointsForDimension28DataTypeStr', 'BreakpointsForDimension28FirstPoint', 'BreakpointsForDimension28Max', 'BreakpointsForDimension28Min', 'BreakpointsForDimension28Spacing', 'BreakpointsForDimension29', 'BreakpointsForDimension29DataTypeStr', 'BreakpointsForDimension29FirstPoint', 'BreakpointsForDimension29Max', 'BreakpointsForDimension29Min', 'BreakpointsForDimension29Spacing', 'BreakpointsForDimension2DataTypeStr', 'BreakpointsForDimension2FirstPoint', 'BreakpointsForDimension2Max', 'BreakpointsForDimension2Min', 'BreakpointsForDimension2Source', 'BreakpointsForDimension2Spacing', 'BreakpointsForDimension3', 'BreakpointsForDimension30', 'BreakpointsForDimension30DataTypeStr', 'BreakpointsForDimension30FirstPoint', 'BreakpointsForDimension30Max', 'BreakpointsForDimension30Min', 'BreakpointsForDimension30Spacing', 'BreakpointsForDimension3DataTypeStr', 'BreakpointsForDimension3FirstPoint', 'BreakpointsForDimension3Max', 'BreakpointsForDimension3Min', 'BreakpointsForDimension3Source', 'BreakpointsForDimension3Spacing', 'BreakpointsForDimension4', 'BreakpointsForDimension4DataTypeStr', 'BreakpointsForDimension4FirstPoint', 'BreakpointsForDimension4Max', 'BreakpointsForDimension4Min', 'BreakpointsForDimension4Spacing', 'BreakpointsForDimension5', 'BreakpointsForDimension5DataTypeStr', 'BreakpointsForDimension5FirstPoint', 'BreakpointsForDimension5Max', 'BreakpointsForDimension5Min', 'BreakpointsForDimension5Spacing', 'BreakpointsForDimension6', 'BreakpointsForDimension6DataTypeStr', 'BreakpointsForDimension6FirstPoint', 'BreakpointsForDimension6Max', 'BreakpointsForDimension6Min', 'BreakpointsForDimension6Spacing', 'BreakpointsForDimension7', 'BreakpointsForDimension7DataTypeStr', 'BreakpointsForDimension7FirstPoint', 'BreakpointsForDimension7Max', 'BreakpointsForDimension7Min', 'BreakpointsForDimension7Spacing', 'BreakpointsForDimension8', 'BreakpointsForDimension8DataTypeStr', 'BreakpointsForDimension8FirstPoint', 'BreakpointsForDimension8Max', 'BreakpointsForDimension8Min', 'BreakpointsForDimension8Spacing', 'BreakpointsForDimension9', 'BreakpointsForDimension9DataTypeStr', 'BreakpointsForDimension9FirstPoint', 'BreakpointsForDimension9Max', 'BreakpointsForDimension9Min', 'BreakpointsForDimension9Spacing', 'BreakpointsSpecification', 'DataSpecification', 'DiagnosticForOutOfRangeInput', 'ExtrapMethod', 'FractionDataTypeStr', 'IndexSearchMethod', 'InputSameDT', 'IntermediateResultsDataTypeStr', 'InternalRulePriority', 'InterpMethod', 'LockScale', 'LookupTableObject', 'MaximumIndicesForEachDimension', 'N1', 'N10', 'N11', 'N12', 'N13', 'N14', 'N15', 'N16', 'N17', 'N18', 'N19', 'N2', 'N20', 'N21', 'N22', 'N23', 'N24', 'N25', 'N26', 'N27', 'N28', 'N29', 'N3', 'N30', 'N4', 'N5', 'N6', 'N7', 'N8', 'N9', 'NumberOfTableDimensions', 'OutDataTypeStr', 'OutMax', 'OutMin', 'RemoveProtectionInput', 'RndMeth', 'SampleTime', 'SaturateOnIntegerOverflow', 'SupportTunableSize', 'SupportTunableTableSize', 'Table', 'TableDataTypeStr', 'TableMax', 'TableMin', 'TableSource', 'UseLastTableValue', 'UseOneInputPortForAllInputData']),
    'Lookup2D': frozenset(['ApplyAccurateEfficientRounding', 'ApplyFullPrecisionForLinearInterpolation', 'BeginIndexSearchUsingPreviousIndexResult', 'BreakpointsForDimension1', 'BreakpointsForDimension10', 'BreakpointsForDimension10DataTypeStr', 'BreakpointsForDimension10FirstPoint', 'BreakpointsForDimension10Max', 'BreakpointsForDimension10Min', 'BreakpointsForDimension10Spacing', 'BreakpointsForDimension11', 'BreakpointsForDimension11DataTypeStr', 'BreakpointsForDimension11FirstPoint', 'BreakpointsForDimension11Max', 'BreakpointsForDimension11Min', 'BreakpointsForDimension11Spacing', 'BreakpointsForDimension12', 'BreakpointsForDimension12DataTypeStr', 'BreakpointsForDimension12FirstPoint', 'BreakpointsForDimension12Max', 'BreakpointsForDimension12Min', 'BreakpointsForDimension12Spacing', 'BreakpointsForDimension13', 'BreakpointsForDimension13DataTypeStr', 'BreakpointsForDimension13FirstPoint', 'BreakpointsForDimension13Max', 'BreakpointsForDimension13Min', 'BreakpointsForDimension13Spacing', 'BreakpointsForDimension14', 'BreakpointsForDimension14DataTypeStr', 'BreakpointsForDimension14FirstPoint', 'BreakpointsForDimension14Max', 'BreakpointsForDimension14Min', 'BreakpointsForDimension14Spacing', 'BreakpointsForDimension15', 'BreakpointsForDimension15DataTypeStr', 'BreakpointsForDimension15FirstPoint', 'BreakpointsForDimension15Max', 'BreakpointsForDimension15Min', 'BreakpointsForDimension15Spacing', 'BreakpointsForDimension16', 'BreakpointsForDimension16DataTypeStr', 'BreakpointsForDimension16FirstPoint', 'BreakpointsForDimension16Max', 'BreakpointsForDimension16Min', 'BreakpointsForDimension16Spacing', 'BreakpointsForDimension17', 'BreakpointsForDimension17DataTypeStr', 'BreakpointsForDimension17FirstPoint', 'BreakpointsForDimension17Max', 'BreakpointsForDimension17Min', 'BreakpointsForDimension17Spacing', 'BreakpointsForDimension18', 'BreakpointsForDimension18DataTypeStr', 'BreakpointsForDimension18FirstPoint', 'BreakpointsForDimension18Max', 'BreakpointsForDimension18Min', 'BreakpointsForDimension18Spacing', 'BreakpointsForDimension19', 'BreakpointsForDimension19DataTypeStr', 'BreakpointsForDimension19FirstPoint', 'BreakpointsForDimension19Max', 'BreakpointsForDimension19Min', 'BreakpointsForDimension19Spacing', 'BreakpointsForDimension1DataTypeStr', 'BreakpointsForDimension1FirstPoint', 'BreakpointsForDimension1Max', 'BreakpointsForDimension1Min', 'BreakpointsForDimension1Source', 'BreakpointsForDimension1Spacing', 'BreakpointsForDimension2', 'BreakpointsForDimension20', 'BreakpointsForDimension20DataTypeStr', 'BreakpointsForDimension20FirstPoint', 'BreakpointsForDimension20Max', 'BreakpointsForDimension20Min', 'BreakpointsForDimension20Spacing', 'BreakpointsForDimension21', 'BreakpointsForDimension21DataTypeStr', 'BreakpointsForDimension21FirstPoint', 'BreakpointsForDimension21Max', 'BreakpointsForDimension21Min', 'BreakpointsForDimension21Spacing', 'BreakpointsForDimension22', 'BreakpointsForDimension22DataTypeStr', 'BreakpointsForDimension22FirstPoint', 'BreakpointsForDimension22Max', 'BreakpointsForDimension22Min', 'BreakpointsForDimension22Spacing', 'BreakpointsForDimension23', 'BreakpointsForDimension23DataTypeStr', 'BreakpointsForDimension23FirstPoint', 'BreakpointsForDimension23Max', 'BreakpointsForDimension23Min', 'BreakpointsForDimension23Spacing', 'BreakpointsForDimension24', 'BreakpointsForDimension24DataTypeStr', 'BreakpointsForDimension24FirstPoint', 'BreakpointsForDimension24Max', 'BreakpointsForDimension24Min', 'BreakpointsForDimension24Spacing', 'BreakpointsForDimension25', 'BreakpointsForDimension25DataTypeStr', 'BreakpointsForDimension25FirstPoint', 'BreakpointsForDimension25Max', 'BreakpointsForDimension25Min', 'BreakpointsForDimension25Spacing', 'BreakpointsForDimension26', 'BreakpointsForDimension26DataTypeStr', 'BreakpointsForDimension26FirstPoint', 'BreakpointsForDimension26Max', 'BreakpointsForDimension26Min', 'BreakpointsForDimension26Spacing', 'BreakpointsForDimension27', 'BreakpointsForDimension27DataTypeStr', 'BreakpointsForDimension27FirstPoint', 'BreakpointsForDimension27Max', 'BreakpointsForDimension27Min', 'BreakpointsForDimension27Spacing', 'BreakpointsForDimension28', 'BreakpointsForDimension28DataTypeStr', 'BreakpointsForDimension28FirstPoint', 'BreakpointsForDimension28Max', 'BreakpointsForDimension28Min', 'BreakpointsForDimension28Spacing', 'BreakpointsForDimension29', 'BreakpointsForDimension29DataTypeStr', 'BreakpointsForDimension29FirstPoint', 'BreakpointsForDimension29Max', 'BreakpointsForDimension29Min', 'BreakpointsForDimension29Spacing', 'BreakpointsForDimension2DataTypeStr', 'BreakpointsForDimension2FirstPoint', 'BreakpointsForDimension2Max', 'BreakpointsForDimension2Min', 'BreakpointsForDimension2Source', 'BreakpointsForDimension2Spacing', 'BreakpointsForDimension3', 'BreakpointsForDimension30', 'BreakpointsForDimension30DataTypeStr', 'BreakpointsForDimension30FirstPoint', 'BreakpointsForDimension30Max', 'BreakpointsForDimension30Min', 'BreakpointsForDimension30Spacing', 'BreakpointsForDimension3DataTypeStr', 'BreakpointsForDimension3FirstPoint', 'BreakpointsForDimension3Max', 'BreakpointsForDimension3Min', 'BreakpointsForDimension3Source', 'BreakpointsForDimension3Spacing', 'BreakpointsForDimension4', 'BreakpointsForDimension4DataTypeStr', 'BreakpointsForDimension4FirstPoint', 'BreakpointsForDimension4Max', 'BreakpointsForDimension4Min', 'BreakpointsForDimension4Spacing', 'BreakpointsForDimension5', 'BreakpointsForDimension5DataTypeStr', 'BreakpointsForDimension5FirstPoint', 'BreakpointsForDimension5Max', 'BreakpointsForDimension5Min', 'BreakpointsForDimension5Spacing', 'BreakpointsForDimension6', 'BreakpointsForDimension6DataTypeStr', 'BreakpointsForDimension6FirstPoint', 'BreakpointsForDimension6Max', 'BreakpointsForDimension6Min', 'BreakpointsForDimension6Spacing', 'BreakpointsForDimension7', 'BreakpointsForDimension7DataTypeStr', 'BreakpointsForDimension7FirstPoint', 'BreakpointsForDimension7Max', 'BreakpointsForDimension7Min', 'BreakpointsForDimension7Spacing', 'BreakpointsForDimension8', 'BreakpointsForDimension8DataTypeStr', 'BreakpointsForDimension8FirstPoint', 'BreakpointsForDimension8Max', 'BreakpointsForDimension8Min', 'BreakpointsForDimension8Spacing', 'BreakpointsForDimension9', 'BreakpointsForDimension9DataTypeStr', 'BreakpointsForDimension9FirstPoint', 'BreakpointsForDimension9Max', 'BreakpointsForDimension9Min', 'BreakpointsForDimension9Spacing', 'BreakpointsSpecification', 'DataSpecification', 'DiagnosticForOutOfRangeInput', 'ExtrapMethod', 'FractionDataTypeStr', 'IndexSearchMethod', 'InputSameDT', 'IntermediateResultsDataTypeStr', 'InternalRulePriority', 'InterpMethod', 'LockScale', 'LookupTableObject', 'MaximumIndicesForEachDimension', 'N1', 'N10', 'N11', 'N12', 'N13', 'N14', 'N15', 'N16', 'N17', 'N18', 'N19', 'N2', 'N20', 'N21', 'N22', 'N23', 'N24', 'N25', 'N26', 'N27', 'N28', 'N29', 'N3', 'N30', 'N4', 'N5', 'N6', 'N7', 'N8', 'N9', 'NumberOfTableDimensions', 'OutDataTypeStr', 'OutMax', 'OutMin', 'RemoveProtectionInput', 'RndMeth', 'SampleTime', 'SaturateOnIntegerOverflow', 'SupportTunableSize', 'SupportTunableTableSize', 'Table', 'TableDataTypeStr', 'TableMax', 'TableMin', 'TableSource', 'UseLastTableValue', 'UseOneInputPortForAllInputData']),
    'MinMax': frozenset(['CollapseDim', 'CollapseMode', 'Function', 'InputSameDT', 'Inputs', 'LockScale', 'OutDataTypeStr', 'OutMax', 'OutMin', 'RndMeth', 'SampleTime', 'SaturateOnIntegerOverflow', 'ZeroCross']),
    'MultiportSwitch': frozenset(['AllowDiffInputSizes', 'DataPortForDefault', 'DataPortIndices', 'DataPortOrder', 'DiagnosticForDefault', 'InputSameDT', 'Inputs', 'LockScale', 'OutDataTypeStr', 'OutMax', 'OutMin', 'RndMeth', 'SampleTime', 'SaturateOnIntegerOverflow']),
    'Outport': frozenset(['BusOutputAsStruct', 'BusVirtuality', 'DataMode', 'EnsureOutportIsVirtual', 'IconDisplay', 'InitialOutput', 'LockScale', 'MustResolveToSignalObject', 'OutDataTypeStr', 'OutMax', 'OutMin', 'OutputFunctionCall', 'OutputWhenDisabled', 'OutputWhenUnConnected', 'OutputWhenUnconnectedValue', 'Port', 'PortDimensions', 'SampleTime', 'SignalName', 'SignalObject', 'SignalType', 'StorageClass', 'Unit', 'VarSizeSig', 'VectorParamsAs1DForOutWhenUnconnected']),
    'Product': frozenset(['CollapseDim', 'CollapseMode', 'InputSameDT', 'Inputs', 'LockScale', 'Multiplication', 'OutDataTypeStr', 'OutMax', 'OutMin', 'RndMeth', 'SampleTime', 'SaturateOnIntegerOverflow']),
    'RateTransition': frozenset(['Deterministic', 'InitialCondition', 'Integrity', 'OutPortSampleTime', 'OutPortSampleTimeMultiple', 'OutPortSampleTimeOpt']),
    'RelationalOperator': frozenset(['InputSameDT', 'Operator', 'OutDataTypeStr', 'RndMeth', 'SampleTime', 'ZeroCross']),
    'Rounding': frozenset(['Operator', 'SampleTime']),
    'Saturation': frozenset(['LinearizeAsGain', 'LockScale', 'LowerLimit', 'OutDataTypeStr', 'OutMax', 'OutMin', 'OutputSaturationStatus', 'RndMeth', 'SampleTime', 'UpperLimit', 'ZeroCross']),
    'ShiftArithmetic': frozenset(['BinPtShiftNumber', 'BitShiftDirection', 'BitShiftNumber', 'BitShiftNumberSource', 'CheckOORBitShift', 'DiagnosticForOORShift']),
    'Signum': frozenset(['SampleTime', 'ZeroCross']),
    'Sqrt': frozenset(['AlgorithmType', 'SqrtAlgorithmType', 'RsqrtAlgorithmType', 'IntermediateResultsDataTypeStr', 'Iterations', 'LockScale', 'Operator',
                       'OutDataTypeStr', 'OutMax', 'OutMin', 'OutputSignalType', 'RndMeth', 'SampleTime',
                       'SaturateOnIntegerOverflow']),
    'Sum': frozenset(['AccumDataTypeStr', 'CollapseDim', 'CollapseMode', 'IconShape', 'InputSameDT', 'Inputs', 'LockScale', 'OutDataTypeStr', 'OutMax', 'OutMin', 'RndMeth', 'SampleTime', 'SaturateOnIntegerOverflow']),
    'Switch': frozenset(['AllowDiffInputSizes', 'Criteria', 'InputSameDT', 'LockScale', 'OutDataTypeStr', 'OutMax', 'OutMin', 'RndMeth', 'SampleTime', 'SaturateOnIntegerOverflow', 'Threshold', 'ZeroCross']),
    'UnaryMinus': frozenset(['SampleTime', 'SaturateOnIntegerOverflow']),
    'UnitDelay': frozenset(['CodeGenStateStorageTypeQualifier', 'InitialCondition', 'InputProcessing', 'SampleTime', 'StateMustResolveToSignalObject', 'StateName', 'StateSignalObject', 'StateStorageClass']),
    'Upsample': frozenset(['FactorSource', 'InitialConditions', 'InputProcessing', 'MaxUpsamplingFactor', 'RateOptions', 'SampleOffset', 'UpsamplingFactor', 'mode']),
}
LIB_DIALOG_PARAMS["LookupND"] = LIB_DIALOG_PARAMS["Lookup2D"]   # the same Simulink block (Lookup_n-D)
# Prelookup / Interpolation Using Prelookup had no dialog list, so every "no effect" name was accepted
# (Simulink: ParamUnknown); their own parameters (the recorded dialogs, recordings show)
LIB_DIALOG_PARAMS["Prelookup"] = frozenset(set(DEFAULTS["Prelookup"]) | {
    "BreakpointsData", "BreakpointsFirstPoint", "BreakpointsSpacing", "BreakpointsNumPoints", "BreakpointObject",
    "OutputOnlyTheIndex", "SampleTime", "LockScale"})
LIB_DIALOG_PARAMS["PrelookupF"] = LIB_DIALOG_PARAMS["Prelookup"]
LIB_DIALOG_PARAMS["InterpolationUsingPrelookup"] = frozenset(set(DEFAULTS["InterpolationUsingPrelookup"]) | {
    "Table", "TableObject", "SampleTime", "LockScale", "TableMin", "TableMax", "OutMin", "OutMax"})
META_VALUES_BY_TYPE = {
    'CICDecimation': {'framing': ('Maintain input frame rate',)},
    'CICInterpolation': {'InputProcessing': ('Elements as channels (sample based)',), 'framing': ('Allow multirate processing',)},
    'Constant': {'VectorParams1D': ('on', 'off')},   # 'off': recordings
    'Delay': {'DelayLengthSource': ('Dialog',), 'ExternalReset': ('None',), 'InitialConditionSource': ('Dialog',), 'InputProcessing': ('Elements as channels (sample based)',), 'PreventDirectFeedthrough': ('off',), 'ShowEnablePort': ('off',)},
    'DetectChange': {'InputProcessing': ('Elements as channels (sample based)',)},
    'DetectDecrease': {'InputProcessing': ('Elements as channels (sample based)',)},
    'DetectIncrease': {'InputProcessing': ('Elements as channels (sample based)',)},
    'DiscreteFir': {'CoefSource': ('Dialog parameters',), 'ExternalReset': ('None',), 'InputProcessing': ('Elements as channels (sample based)',), 'ShowEnablePort': ('off',)},
    'DiscreteIntegrator': {'ExternalReset': ('none', 'rising', 'falling', 'either', 'level', 'sampled level'), 'IgnoreLimit': ('off',), 'InitialConditionSetting': ('Auto',), 'InitialConditionSource': ('internal', 'external'), 'ShowSaturationPort': ('off',), 'ShowStatePort': ('off',)},   # reset / external IC: recordings (recordings low)
    'Downsample': {'InputProcessing': ('Elements as channels (sample based)',)},
    'FIRDecimation': {'InputProcessing': ('Elements as channels (sample based)',), 'framing': ('Allow multirate processing',)},
    'FIRInterpolation': {'InputProcessing': ('Elements as channels (sample based)',), 'framing': ('Allow multirate processing',)},
    'Gain': {'Multiplication': ('Element-wise(K.*u)', 'Matrix(K*u)', 'Matrix(u*K)', 'Matrix(K*u) (u vector)')},
    'Lookup1D': {'ApplyAccurateEfficientRounding': ('off',), 'ApplyFullPrecisionForLinearInterpolation': ('off',), 'BreakpointsForDimension1Source': ('Dialog',), 'DataSpecification': ('Table and breakpoints',), 'LookupTableObject': ('',), 'NumberOfTableDimensions': ('1',), 'RemoveProtectionInput': ('off',), 'SupportTunableSize': ('off',), 'SupportTunableTableSize': ('off',), 'TableSource': ('Dialog',), 'UseLastTableValue': ('off', 'on'), 'UseOneInputPortForAllInputData': ('off',)},
    'Lookup2D': {'ApplyAccurateEfficientRounding': ('off',), 'ApplyFullPrecisionForLinearInterpolation': ('off',), 'BreakpointsForDimension1Source': ('Dialog',), 'DataSpecification': ('Table and breakpoints',), 'LookupTableObject': ('',), 'NumberOfTableDimensions': ('2',), 'RemoveProtectionInput': ('off',), 'SupportTunableSize': ('off',), 'SupportTunableTableSize': ('off',), 'TableSource': ('Dialog',), 'UseLastTableValue': ('off', 'on'), 'UseOneInputPortForAllInputData': ('off',)},
    'MinMax': {'CollapseMode': ('All dimensions',)},
    'MultiportSwitch': {'AllowDiffInputSizes': ('off',)},
    'Product': {'CollapseMode': ('All dimensions',), 'Multiplication': ('Element-wise(.*)', 'Matrix(*)')},
    'ShiftArithmetic': {'BitShiftNumberSource': ('Dialog', 'Input port')},
    'Sum': {'CollapseMode': ('All dimensions',)},
    'Switch': {'AllowDiffInputSizes': ('off',)},
    'UnitDelay': {'InputProcessing': ('Elements as channels (sample based)',)},
    'Upsample': {'InputProcessing': ('Elements as channels (sample based)',)},
}
META_VALUES_BY_TYPE["LookupND"] = dict(META_VALUES_BY_TYPE["Lookup2D"], NumberOfTableDimensions=("3", "4"),
                                      **{"BreakpointsForDimension%dSource" % k: ("Dialog",) for k in range(1, 5)})

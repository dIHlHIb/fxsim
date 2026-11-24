# Block reference (generated)

Query the block registry with the fxgraph request `{"catalogue": true}`.

For every block type a design may use (`"type"` in design JSON):

- **parameters**: the names accepted in design JSON (Simulink dialog parameter names). Any other name is
  refused with `BAD_PARAM`.
- **defaults**: the value a JSON design gets when it leaves the parameter out -- the default of a block
  freshly taken from the Simulink R2026b library. Every result lists the ones a design relied on in
  `assumed_defaults`.
- **required**: no default; leaving it out gives `MISSING_FIELD`.
- **must write**: the library default exists but is not supported; leaving it out gives `UNSUPPORTED`
  (write a supported value).
- **values**: parameters accepted with these values only.
- A blank default means the parameter has no single library value in the registry (typically an output
  type whose rule depends on the inputs); the result's `compiled_types` shows the type that was used.

Every block also takes `SampleTime` (except Outport). Types: `fixdt(s,w,f)`, `int8` .. `uint32`,
`boolean`, and the `Inherit: ...` strings a parameter offers in Simulink.

## Abs

| parameter | default | note |
|---|---|---|
| `OutDataTypeStr` |  |  |
| `RndMeth` |  |  |
| `SampleTime` |  |  |
| `SaturateOnIntegerOverflow` | `off` |  |

## Assertion

| parameter | default | note |
|---|---|---|
| `AssertionFailFcn` | `` |  |
| `Enabled` | `on` |  |
| `SampleTime` |  |  |
| `StopWhenAssertionFail` | `on` |  |

## Bias

| parameter | default | note |
|---|---|---|
| `Bias` | `0.0` |  |
| `SampleTime` |  |  |
| `SaturateOnIntegerOverflow` | `off` |  |

## BitClear

| parameter | default | note |
|---|---|---|
| `SampleTime` |  |  |
| `iBit` |  | required |

## BitConcat

| parameter | default | note |
|---|---|---|
| `SampleTime` |  |  |
| `numInputs` | `2` |  |

## BitReduce

| parameter | default | note |
|---|---|---|
| `SampleTime` |  |  |
| `mode` | `AND` |  |

## BitRotate

| parameter | default | note |
|---|---|---|
| `N` |  | required |
| `SampleTime` |  |  |
| `mode` |  | required |

## BitSet

| parameter | default | note |
|---|---|---|
| `SampleTime` |  |  |
| `iBit` |  | required |

## BitShift

| parameter | default | note |
|---|---|---|
| `N` | `0` |  |
| `SampleTime` |  |  |
| `mode` | `Shift Left Logical` |  |

## BitSlice

| parameter | default | note |
|---|---|---|
| `SampleTime` |  |  |
| `lidx` | `7` |  |
| `ridx` | `0` |  |

## BitwiseOperator

| parameter | default | note |
|---|---|---|
| `BitMask` | `bin2dec('11011001')` |  |
| `BitMaskRealWorld` | `Stored Integer` |  |
| `NumInputPorts` | `1` |  |
| `SampleTime` |  |  |
| `UseBitMask` | `on` |  |
| `logicop` | `AND` |  |

## BusAssignment

| parameter | default | note |
|---|---|---|
| `AssignedSignals` | `signal1` |  |
| `SampleTime` |  |  |

## CICDecimation

| parameter | default | note |
|---|---|---|
| `BPS` | `[16 16 16 16]` |  |
| `FLPS` | `0` |  |
| `M` | `1` |  |
| `N` | `2` |  |
| `R` | `2` |  |
| `RateOptions` | `Enforce single-rate processing` |  |
| `SampleTime` |  |  |
| `filtFrom` |  | values: `Dialog parameters` |
| `filterInternals` | `Full precision` |  |
| `framing` |  | values: `Maintain input frame rate` |
| `ftype` | `Decimator` |  |
| `outputFracLength` | `0` |  |
| `outputWordLength` | `32` |  |

## CICInterpolation

| parameter | default | note |
|---|---|---|
| `BPS` | `[16 16 16 16]` |  |
| `FLPS` | `0` |  |
| `InputProcessing` |  | must write; values: `Elements as channels (sample based)` |
| `M` | `1` |  |
| `N` | `2` |  |
| `R` | `2` |  |
| `RateOptions` | `Allow multirate processing` |  |
| `SampleTime` |  |  |
| `filtFrom` |  | values: `Dialog parameters` |
| `filterInternals` | `Full precision` |  |
| `framing` |  | must write; values: `Allow multirate processing` |
| `ftype` | `Interpolator` |  |
| `outputFracLength` | `0` |  |
| `outputWordLength` | `32` |  |

## CompareToConstant

| parameter | default | note |
|---|---|---|
| `OutDataTypeStr` | `boolean` |  |
| `SampleTime` |  |  |
| `const` | `3.0` |  |
| `relop` | `<=` |  |

## CompareToZero

| parameter | default | note |
|---|---|---|
| `OutDataTypeStr` | `boolean` |  |
| `SampleTime` |  |  |
| `relop` | `<=` |  |

## ComplexToRealImag

| parameter | default | note |
|---|---|---|
| `Output` | `Real and imag` |  |
| `SampleTime` |  |  |

## Concatenate

| parameter | default | note |
|---|---|---|
| `ConcatenateDimension` | `1` |  |
| `Mode` | `Vector` |  |
| `NumInputs` | `2` |  |
| `SampleTime` |  |  |

## Constant

| parameter | default | note |
|---|---|---|
| `OutDataTypeStr` | `Inherit: Inherit from 'Constant value'` |  |
| `SampleTime` |  |  |
| `Value` | `1` |  |
| `VectorParams1D` |  | values: `off` / `on` |

## CounterFreeRunning

| parameter | default | note |
|---|---|---|
| `NumBits` | `16` |  |
| `SampleTime` |  |  |

## CounterLimited

| parameter | default | note |
|---|---|---|
| `SampleTime` |  |  |
| `uplimit` | `7` |  |

## DataTypeConversion

| parameter | default | note |
|---|---|---|
| `ConvertRealWorld` | `Real World Value (RWV)` |  |
| `OutDataTypeStr` | `Inherit: Inherit via back propagation` |  |
| `RndMeth` | `Floor` |  |
| `SampleTime` |  |  |
| `SaturateOnIntegerOverflow` | `off` |  |

## DataTypeDuplicate

| parameter | default | note |
|---|---|---|
| `NumInputPorts` | `2` |  |
| `SampleTime` |  |  |

## DeadZone

| parameter | default | note |
|---|---|---|
| `LowerValue` | `-0.5` |  |
| `SampleTime` |  |  |
| `SaturateOnIntegerOverflow` | `on` |  |
| `UpperValue` | `0.5` |  |

Simulink `simulink/Discontinuities/Dead Zone`. Answered for one fixed-point or integer scalar input of 4 to 48 bits with fraction length -2 .. word length - 1; output type = input type. Start (`LowerValue`) and End (`UpperValue`) are rounded to nearest (ties up) into the input type; up to one LSB above the maximum they saturate (with a quantisation note), beyond that -- or one LSB or more below the minimum -- Simulink stops with `fxpParameterOverflow` and the engine refuses with `PORT_TYPE_INVALID` at that parameter. Output: 0 where Start <= u <= End, u - End above, u - Start below; that difference saturates (`SaturateOnIntegerOverflow` `on`, the default) or wraps (`off`) into the input type -- it leaves the type when Start > 0 or End < 0 (e.g. unsigned u below a positive Start). Refused: Start above End (after rounding; not recorded), other types, vectors, boolean / double inputs. Evidence: Simulink R2026b recordings of 50 + 48 designs (single, after a Gain, before a Sum; periods 1 and 2; host and ASIC/FPGA).

## DecrementRealWorld

| parameter | default | note |
|---|---|---|
| `SampleTime` |  |  |

## DecrementStoredInteger

| parameter | default | note |
|---|---|---|
| `SampleTime` |  |  |

## Delay

| parameter | default | note |
|---|---|---|
| `DelayLength` | `2` |  |
| `DelayLengthSource` |  | values: `Dialog` |
| `ExternalReset` |  | values: `None` |
| `InitialCondition` | `0` |  |
| `InitialConditionSource` |  | values: `Dialog` |
| `InputProcessing` |  | values: `Elements as channels (sample based)` |
| `PreventDirectFeedthrough` |  | values: `off` |
| `SampleTime` |  |  |
| `ShowEnablePort` |  | values: `off` |

## DetectChange

| parameter | default | note |
|---|---|---|
| `InputProcessing` |  | values: `Elements as channels (sample based)` |
| `OutDataTypeStr` | `boolean` |  |
| `SampleTime` |  |  |
| `vinit` | `0` |  |

## DetectDecrease

| parameter | default | note |
|---|---|---|
| `InputProcessing` |  | values: `Elements as channels (sample based)` |
| `OutDataTypeStr` | `boolean` |  |
| `SampleTime` |  |  |
| `vinit` | `0` |  |

## DetectFallNegative

| parameter | default | note |
|---|---|---|
| `InputProcessing` |  | values: `Elements as channels (sample based)` / `Inherited` |
| `OutDataTypeStr` | `boolean` |  |
| `SampleTime` |  |  |
| `vinit` | `0` |  |

## DetectFallNonpositive

| parameter | default | note |
|---|---|---|
| `InputProcessing` |  | values: `Elements as channels (sample based)` / `Inherited` |
| `OutDataTypeStr` | `boolean` |  |
| `SampleTime` |  |  |
| `vinit` | `0` |  |

## DetectIncrease

| parameter | default | note |
|---|---|---|
| `InputProcessing` |  | values: `Elements as channels (sample based)` |
| `OutDataTypeStr` | `boolean` |  |
| `SampleTime` |  |  |
| `vinit` | `0` |  |

## DetectRiseNonnegative

| parameter | default | note |
|---|---|---|
| `InputProcessing` |  | values: `Elements as channels (sample based)` / `Inherited` |
| `OutDataTypeStr` | `boolean` |  |
| `SampleTime` |  |  |
| `vinit` | `0` |  |

## DetectRisePositive

| parameter | default | note |
|---|---|---|
| `InputProcessing` |  | values: `Elements as channels (sample based)` / `Inherited` |
| `OutDataTypeStr` | `boolean` |  |
| `SampleTime` |  |  |
| `vinit` | `0` |  |

## DirectLookupND

| parameter | default | note |
|---|---|---|
| `DiagnosticForOutOfRangeInput` | `Warning` |  |
| `InputsSelectThisObjectFromTable` | `Element` |  |
| `NumberOfTableDimensions` | `2` |  |
| `RemoveProtectionInput` | `off` |  |
| `SampleTime` |  |  |
| `Table` | `[4 5 6;16 19 20;10 18 23]` |  |
| `TableDataTypeStr` | `Inherit: Inherit from 'Table data'` |  |

## DiscreteFir

| parameter | default | note |
|---|---|---|
| `AccumDataTypeStr` | `Inherit: Inherit via internal rule` |  |
| `CoefDataTypeStr` | `Inherit: Same word length as input` |  |
| `CoefSource` |  | values: `Dialog parameters` |
| `Coefficients` | `[0.5 0.5]` |  |
| `ExternalReset` |  | values: `None` |
| `FilterStructure` | `Direct form` |  |
| `InitialStates` | `0` |  |
| `InputProcessing` |  | values: `Elements as channels (sample based)` |
| `OutDataTypeStr` | `Inherit: Same as input` |  |
| `ProductDataTypeStr` | `Inherit: Inherit via internal rule` |  |
| `RndMeth` | `Floor` |  |
| `SampleTime` |  |  |
| `SaturateOnIntegerOverflow` | `off` |  |
| `ShowEnablePort` |  | values: `off` |
| `StateDataTypeStr` | `Inherit: Same as accumulator` |  |
| `TapSumDataTypeStr` | `Inherit: Inherit via internal rule` |  |

## DiscreteIntegrator

| parameter | default | note |
|---|---|---|
| `ExternalReset` | `none` | values: `either` / `falling` / `level` / `none` / `rising` / `sampled level` |
| `IgnoreLimit` |  | values: `off` |
| `InitialCondition` | `0` |  |
| `InitialConditionSetting` |  | values: `Auto` |
| `InitialConditionSource` | `internal` | values: `external` / `internal` |
| `IntegratorMethod` | `Integration: Forward Euler` |  |
| `LimitOutput` | `off` |  |
| `LowerSaturationLimit` | `-inf` |  |
| `OutDataTypeStr` | `Inherit: Inherit via internal rule` |  |
| `RndMeth` | `Floor` |  |
| `SampleTime` | `-1` |  |
| `SaturateOnIntegerOverflow` | `off` |  |
| `ShowSaturationPort` |  | values: `off` |
| `ShowStatePort` |  | values: `off` |
| `UpperSaturationLimit` | `inf` |  |
| `gainval` | `1.0` |  |

## Downsample

| parameter | default | note |
|---|---|---|
| `DownsamplingFactor` | `2` |  |
| `InitialConditions` | `0` |  |
| `InputProcessing` |  | must write; values: `Elements as channels (sample based)` |
| `RateOptions` | `Enforce single-rate processing` |  |
| `SampleOffset` | `0` |  |
| `SampleTime` |  |  |

## EnablePort

| parameter | default | note |
|---|---|---|
| `SampleTime` |  |  |
| `StatesWhenEnabling` | `held` |  |

## EnabledOutport

| parameter | default | note |
|---|---|---|
| `InitialOutput` | `[]` |  |
| `OutputWhenDisabled` | `held` |  |
| `SampleTime` |  |  |

## ExtractBits

| parameter | default | note |
|---|---|---|
| `SampleTime` |  |  |
| `bitIdxRange` | `[0 7]` |  |
| `bitsToExtract` | `Upper half` |  |
| `numBits` | `8` |  |
| `outScalingMode` | `Preserve fixed-point scaling` |  |

## FIRDecimation

| parameter | default | note |
|---|---|---|
| `D` | `2` |  |
| `FilterSource` | `Auto` |  |
| `InputProcessing` |  | must write; values: `Elements as channels (sample based)` |
| `SampleTime` |  |  |
| `accumDataTypeStr` | `Inherit: Inherit via internal rule` |  |
| `accumMode` | `Inherit via internal rule` |  |
| `filtStruct` | `Direct form` |  |
| `firstCoeffDataTypeStr` | `Inherit: Same word length as input` |  |
| `firstCoeffMode` | `Same word length as input` |  |
| `framing` | `Enforce single-rate processing` | values: `Allow multirate processing` |
| `h` |  | required |
| `outputBufInitCond` | `0` |  |
| `outputDataTypeStr` | `Inherit: Same as input` |  |
| `outputMode` | `Same as input` |  |
| `overflowMode` | `off` |  |
| `prodOutputDataTypeStr` | `Inherit: Inherit via internal rule` |  |
| `prodOutputMode` | `Inherit via internal rule` |  |
| `roundingMode` | `Floor` |  |

## FIRInterpolation

| parameter | default | note |
|---|---|---|
| `FilterSource` | `Auto` |  |
| `InputProcessing` |  | values: `Elements as channels (sample based)` |
| `L` | `3` |  |
| `SampleTime` |  |  |
| `accumDataTypeStr` | `Inherit: Inherit via internal rule` |  |
| `accumMode` | `Inherit via internal rule` |  |
| `firstCoeffDataTypeStr` | `Inherit: Same word length as input` |  |
| `firstCoeffMode` | `Same word length as input` |  |
| `framing` | `Enforce single-rate processing` | values: `Allow multirate processing` |
| `h` |  | required |
| `outputBufInitCond` | `0` |  |
| `outputDataTypeStr` | `Inherit: Same as input` |  |
| `outputMode` | `Same as input` |  |
| `overflowMode` | `off` |  |
| `prodOutputDataTypeStr` | `Inherit: Inherit via internal rule` |  |
| `prodOutputMode` | `Inherit via internal rule` |  |
| `roundingMode` | `Floor` |  |

## Gain

| parameter | default | note |
|---|---|---|
| `Gain` | `1` |  |
| `Multiplication` |  | values: `Element-wise(K.*u)` / `Matrix(K*u)` / `Matrix(K*u) (u vector)` / `Matrix(u*K)` |
| `OutDataTypeStr` | `Inherit: Inherit via internal rule` |  |
| `ParamDataTypeStr` | `Inherit: Inherit via internal rule` |  |
| `RndMeth` | `Floor` |  |
| `SampleTime` |  |  |
| `SaturateOnIntegerOverflow` | `off` |  |

## Ground

| parameter | default | note |
|---|---|---|
| `SampleTime` |  |  |

## HDLCounter

| parameter | default | note |
|---|---|---|
| `CountDataType` | `Unsigned` |  |
| `CountDirPort` | `off` |  |
| `CountEnbPort` | `off` |  |
| `CountFracLen` | `0` |  |
| `CountFrom` | `0` |  |
| `CountFromType` | `Initial value` |  |
| `CountHitOutputPort` | `off` |  |
| `CountInit` | `0` |  |
| `CountLoadPort` | `off` |  |
| `CountMax` | `25` |  |
| `CountResetPort` | `off` |  |
| `CountSampTime` | `1` |  |
| `CountStep` | `1` |  |
| `CountType` | `Count limited` |  |
| `CountWordLen` | `8` |  |
| `SampleTime` |  |  |

## HDLFIFO

| parameter | default | note |
|---|---|---|
| `SampleTime` |  |  |
| `dataDim` | `1` |  |
| `fifo_size` | `10` |  |
| `mode` | `Classic` |  |
| `pop_msg` | `Warning` |  |
| `push_msg` | `Warning` |  |
| `ratio` | `1` |  |
| `rst_port` | `off` |  |
| `show_empty` | `on` |  |
| `show_full` | `on` |  |
| `show_num` | `on` |  |

## HDLFIRDecimator

| parameter | default | note |
|---|---|---|
| `CoefficientsDataTypeStr` | `Inherit: Same word length as input` |  |
| `DecimationFactor` | `2` |  |
| `FilterStructure` | `Direct form systolic` |  |
| `HDLGlobalReset` | `off` |  |
| `LockScale` | `off` |  |
| `NumCycles` | `1` |  |
| `Numerator` |  | required |
| `NumeratorPrototype` | `[]` |  |
| `NumeratorSource` | `Property` |  |
| `OptimizeSymmCoeff` | `on` |  |
| `OutputDataTypeStr` | `Inherit: Inherit via internal rule` |  |
| `ResetInputPort` | `off` |  |
| `SampleTime` |  |  |
| `SimulateUsing` | `Code generation` |  |
| `overflowMode` | `off` |  |
| `roundingMode` | `Floor` |  |

## HDLFIRFilter

| parameter | default | note |
|---|---|---|
| `CoeffMultiplier` | `Multiplier` |  |
| `CoeffMultiplierSystolic` | `Multiplier` |  |
| `CoefficientsDataTypeStr` | `Inherit: Same word length as input` |  |
| `FilterStructure` | `Direct form systolic` |  |
| `FrameInputSymmOptim` | `on` |  |
| `HDLGlobalReset` | `off` |  |
| `LockScale` | `off` |  |
| `NumCycles` | `2` |  |
| `NumberOfMultipliers` | `2` |  |
| `Numerator` |  | required |
| `NumeratorPrototype` | `[]` |  |
| `NumeratorSource` | `Property` |  |
| `OutputDataTypeStr` | `Inherit: Inherit via internal rule` |  |
| `ResetInputPort` | `off` |  |
| `SampleTime` |  |  |
| `SerializationOption` | `Minimum number of cycles between valid input samples` |  |
| `SimulateUsing` | `Code generation` |  |
| `UseMultForPow2Coeff` | `on` |  |
| `overflowMode` | `off` |  |
| `roundingMode` | `Floor` |  |

## HDLNCO

| parameter | default | note |
|---|---|---|
| `AccumulatorDataType` | `Binary point scaling` |  |
| `AccumulatorFL` | `0` |  |
| `AccumulatorSigned` | `yes` |  |
| `AccumulatorWL` | `16` |  |
| `DitherSource` | `Property` |  |
| `FreqResolution` | `0.05` |  |
| `Frequency` | `510` |  |
| `LUTCompress` | `off` |  |
| `NumDitherBits` | `4` |  |
| `NumQuantizerAccumulatorBits` | `12` |  |
| `OutputDataType` | `Binary point scaling` |  |
| `OutputFL` | `14` |  |
| `OutputSigned` | `yes` |  |
| `OutputWL` | `16` |  |
| `OverflowAction` | `Wrap` |  |
| `PhaseIncrement` | `100` |  |
| `PhaseIncrementSource` | `Input port` |  |
| `PhaseOffset` | `0` |  |
| `PhaseOffsetR` | `pi/3` |  |
| `PhaseOffsetSource` | `Property` |  |
| `PhasePort` | `off` |  |
| `PhaseQuantization` | `on` |  |
| `ResetAction` | `off` |  |
| `RoundingMethod` | `Floor` |  |
| `SampleTime` |  |  |
| `SamplesPerFrame` | `1` |  |
| `SamplingFreq` | `4000` |  |
| `SimulateUsing` | `Code generation` |  |
| `Waveform` | `Sine` |  |

## HDLRAM

| parameter | default | note |
|---|---|---|
| `AddressBits` | `8` |  |
| `RAMInitialValue` | `0` |  |
| `RAMType` | `Simple dual port` |  |
| `SampleTime` |  |  |
| `WriteOutputValue` | `New data` |  |

## IncrementRealWorld

| parameter | default | note |
|---|---|---|
| `SampleTime` |  |  |

## IncrementStoredInteger

| parameter | default | note |
|---|---|---|
| `SampleTime` |  |  |

## Inport

| parameter | default | note |
|---|---|---|
| `SampleTime` |  |  |

## InterpolationUsingPrelookup

| parameter | default | note |
|---|---|---|
| `DiagnosticForOutOfRangeInput` | `None` | values: `None` / `Warning` |
| `ExtrapMethod` | `Linear` |  |
| `IntermediateResultsDataTypeStr` | `Inherit: Inherit via internal rule` |  |
| `InternalRulePriority` | `Speed` |  |
| `InterpMethod` | `Linear point-slope` |  |
| `LockScale` | `off` |  |
| `LookupTableObject` | `` | values: `` |
| `NumSelectionDims` | `0` |  |
| `NumberOfTableDimensions` | `2` | values: `1` |
| `OutDataTypeStr` | `Inherit: Inherit via internal rule` |  |
| `OutMax` | `[]` |  |
| `OutMin` | `[]` |  |
| `RemoveProtectionIndex` | `off` |  |
| `RequireIndexFractionAsBus` | `off` |  |
| `RndMeth` | `Floor` |  |
| `SampleTime` |  |  |
| `SaturateOnIntegerOverflow` | `off` |  |
| `Table` |  | required |
| `TableDataTypeStr` | `Inherit: Same as output` |  |
| `TableMax` | `[]` |  |
| `TableMin` | `[]` |  |
| `TableSource` | `Dialog` | values: `Dialog` |
| `TableSpecification` | `Explicit values` |  |
| `ValidIndexMayReachLast` | `off` |  |

Simulink `simulink/Lookup Tables/Interpolation Using Prelookup` with 1 or 2 dimensions; inputs 2d-1 / 2d are the k / f outputs of one Prelookup per dimension (the recorded wiring). `Table` a vector (1-D) or reshape([values], [n1 n2]) (column-major), non-negative values, rounded to nearest into the table type (inherited = the output type). The output type must be explicit, except the recorded back-propagated form: 1-D, Clip, fed by one fixed-point Prelookup on an Inport, every destination an untyped Outport, as recorded (engine/ipl_double_domain.json) -- Simulink compiles it to double and it runs on the floating-point path as y0 + f (y1 - y0) in double (Flat: the table value). Linear point-slope interpolates dimension 1 first, then 2, each step y0 + floor(f (y1 - y0)) in an intermediate type with wrap (the table type with a 32-bit fraction; otherwise the table's integer bits in 16 or 32 bits), then floored into the output type; Flat / Nearest pick a table value. ExtrapMethod must be Clip (Simulink CannotExtrapData). Answered only for the recorded table / output combinations: Linear point-slope -- s12En6 / s16En8, u8En4 / s12En6, s8En4 / s8En4, s8En4 / s16En8, u8En4 / u12En6, u8En4 / s16En10, s12En6 / s12En6, s12En6 / s12En4, u16En8 / s12En6 and s16En8 / s8En4 (both with the 32-bit fraction only: for a 16-bit table with a 16 / 8-bit fraction the intermediate type is not settled); Flat / Nearest -- s12En6 / s16En8, u8En4 / s12En6; the other parameters at their defaults, host, outside loops. After a Prelookup on int16 / sfix16_En8 / sfix16_En14 / ufix8_En4 with the library fraction type (1-D, host and ASIC / FPGA): table / output s16En8 / s32En16 and int16 / int16, Linear point-slope and Flat, signed table values; there a signed 16-bit table with a 16 / 8-bit fraction (ASIC) interpolates in the table type. Not word-length optimised. Evidence: Simulink R2026b dense recordings of 84 + 60 + 288 models every answered step equal).

## LogicalOperator

| parameter | default | note |
|---|---|---|
| `AllPortsSameDT` |  | values: `off` / `on` |
| `Inputs` | `2` |  |
| `Operator` | `AND` |  |
| `OutDataTypeStr` |  |  |
| `SampleTime` |  |  |

## Lookup1D

| parameter | default | note |
|---|---|---|
| `ApplyAccurateEfficientRounding` |  | values: `off` |
| `ApplyFullPrecisionForLinearInterpolation` |  | values: `off` |
| `BreakpointsForDimension1` |  |  |
| `BreakpointsForDimension1DataTypeStr` | `Inherit: Same as corresponding input` |  |
| `BreakpointsForDimension1FirstPoint` | `1` |  |
| `BreakpointsForDimension1Source` |  | values: `Dialog` |
| `BreakpointsForDimension1Spacing` | `1` |  |
| `BreakpointsSpecification` | `Explicit values` |  |
| `DataSpecification` |  | values: `Table and breakpoints` |
| `DiagnosticForOutOfRangeInput` |  | values: `None` / `Warning` |
| `ExtrapMethod` | `Linear` |  |
| `FractionDataTypeStr` | `Inherit: Inherit via internal rule` |  |
| `InputSameDT` |  | values: `off` / `on` |
| `IntermediateResultsDataTypeStr` | `Inherit: Same as output` |  |
| `InternalRulePriority` | `Speed` |  |
| `InterpMethod` | `Linear point-slope` |  |
| `LookupTableObject` |  | values: `` |
| `NumberOfTableDimensions` |  | values: `1` |
| `OutDataTypeStr` | `Inherit: Same as first input` |  |
| `RemoveProtectionInput` |  | values: `off` |
| `RndMeth` | `Simplest` |  |
| `SampleTime` |  |  |
| `SaturateOnIntegerOverflow` | `off` |  |
| `SupportTunableSize` |  | values: `off` |
| `SupportTunableTableSize` |  | values: `off` |
| `Table` |  | required |
| `TableDataTypeStr` | `Inherit: Same as output` |  |
| `TableSource` |  | values: `Dialog` |
| `UseLastTableValue` |  | values: `off` / `on` |
| `UseOneInputPortForAllInputData` |  | values: `off` |

## Lookup2D

| parameter | default | note |
|---|---|---|
| `ApplyAccurateEfficientRounding` |  | values: `off` |
| `ApplyFullPrecisionForLinearInterpolation` |  | values: `off` |
| `BreakpointsForDimension1` |  |  |
| `BreakpointsForDimension1DataTypeStr` | `Inherit: Same as corresponding input` |  |
| `BreakpointsForDimension1FirstPoint` | `1` |  |
| `BreakpointsForDimension1Source` |  | values: `Dialog` |
| `BreakpointsForDimension1Spacing` | `1` |  |
| `BreakpointsForDimension2` |  |  |
| `BreakpointsForDimension2DataTypeStr` | `Inherit: Same as corresponding input` |  |
| `BreakpointsForDimension2FirstPoint` | `1` |  |
| `BreakpointsForDimension2Spacing` | `1` |  |
| `BreakpointsSpecification` | `Explicit values` |  |
| `DataSpecification` |  | values: `Table and breakpoints` |
| `DiagnosticForOutOfRangeInput` |  | values: `None` / `Warning` |
| `ExtrapMethod` | `Linear` |  |
| `FractionDataTypeStr` | `Inherit: Inherit via internal rule` |  |
| `InputSameDT` |  | values: `off` / `on` |
| `IntermediateResultsDataTypeStr` | `Inherit: Same as output` |  |
| `InternalRulePriority` | `Speed` |  |
| `InterpMethod` | `Linear point-slope` |  |
| `LookupTableObject` |  | values: `` |
| `NumberOfTableDimensions` |  | values: `2` |
| `OutDataTypeStr` | `Inherit: Same as first input` |  |
| `RemoveProtectionInput` |  | values: `off` |
| `RndMeth` | `Simplest` |  |
| `SampleTime` |  |  |
| `SaturateOnIntegerOverflow` | `off` |  |
| `SupportTunableSize` |  | values: `off` |
| `SupportTunableTableSize` |  | values: `off` |
| `Table` |  | required |
| `TableDataTypeStr` | `Inherit: Same as output` |  |
| `TableSource` |  | values: `Dialog` |
| `UseLastTableValue` | `off` | values: `off` / `on` |
| `UseOneInputPortForAllInputData` |  | values: `off` |

## LookupND

| parameter | default | note |
|---|---|---|
| `ApplyAccurateEfficientRounding` |  | values: `off` |
| `ApplyFullPrecisionForLinearInterpolation` |  | values: `off` |
| `BreakpointsForDimension1` |  |  |
| `BreakpointsForDimension1DataTypeStr` | `Inherit: Same as corresponding input` |  |
| `BreakpointsForDimension1Source` |  | values: `Dialog` |
| `BreakpointsForDimension2` |  |  |
| `BreakpointsForDimension2DataTypeStr` | `Inherit: Same as corresponding input` |  |
| `BreakpointsForDimension3` |  |  |
| `BreakpointsForDimension3DataTypeStr` | `Inherit: Same as corresponding input` |  |
| `BreakpointsForDimension4` |  |  |
| `BreakpointsForDimension4DataTypeStr` | `Inherit: Same as corresponding input` |  |
| `BreakpointsSpecification` | `Explicit values` |  |
| `DataSpecification` |  | values: `Table and breakpoints` |
| `DiagnosticForOutOfRangeInput` |  | values: `None` / `Warning` |
| `ExtrapMethod` | `Linear` |  |
| `FractionDataTypeStr` | `Inherit: Inherit via internal rule` |  |
| `InputSameDT` |  | values: `off` / `on` |
| `IntermediateResultsDataTypeStr` | `Inherit: Same as output` |  |
| `InternalRulePriority` | `Speed` |  |
| `InterpMethod` | `Linear point-slope` |  |
| `LookupTableObject` |  | values: `` |
| `NumberOfTableDimensions` | `3` | required; values: `3` / `4` |
| `OutDataTypeStr` | `Inherit: Same as first input` |  |
| `RemoveProtectionInput` |  | values: `off` |
| `RndMeth` | `Simplest` |  |
| `SampleTime` |  |  |
| `SaturateOnIntegerOverflow` | `off` |  |
| `SupportTunableSize` |  | values: `off` |
| `SupportTunableTableSize` |  | values: `off` |
| `Table` |  | required |
| `TableDataTypeStr` | `Inherit: Same as output` |  |
| `TableSource` |  | values: `Dialog` |
| `UseLastTableValue` | `off` | values: `off` / `on` |
| `UseOneInputPortForAllInputData` |  | values: `off` |

Simulink `simulink/Lookup Tables/n-D Lookup Table` with 3 or 4 dimensions (`NumberOfTableDimensions`; use Lookup1D / Lookup2D for 1 / 2). One scalar input per dimension, all of one type; explicit, strictly increasing breakpoints per dimension (2 to 4 points); `Table` written reshape([values], [n1 n2 ...]) (column-major, as MATLAB), non-negative values. Breakpoints and table are rounded to nearest into their types. Flat / Nearest pick a table value per dimension; Linear point-slope interpolates dimension 1 first, then 2, ..., every step y0 + floor(f (y1 - y0)) in the output type with wrap, f floored to as many fraction bits as the output word (16, and the falling term from the magnitude, when an unsigned 8 / 16-bit table type is the output type). ExtrapMethod must be Clip: Simulink rejects linear extrapolation on fixed point (the library default). Answered only for the recorded type combinations (input / breakpoints / table / output): sfix8_En4 inherited; sfix8_En4 / fixdt(1,12,6) / fixdt(1,8,4) / inherited; ufix8_En4 / inherited / fixdt(0,8,4) / inherited; sfix12_En6 / fixdt(1,8,4) / fixdt(1,12,6) / fixdt(1,16,8); sfix12_En6 / inherited / fixdt(0,8,4) / fixdt(1,12,6); ufix8_En4 / fixdt(1,12,6) / fixdt(1,8,4) / inherited -- with the inherited intermediate and fraction types, InternalRulePriority Speed, RndMeth Simplest, no saturation, UseLastTableValue off, host, outside loops. Nearest needs breakpoints of the input's type (Simulink PreLookupNearestUAndBpDataTypeMismatch). Evidence: Simulink R2026b dense recording of 60 models (30 run, 47,700 steps).

## ManualSwitch

| parameter | default | note |
|---|---|---|
| `SampleTime` |  |  |
| `sw` |  | required |

## Math

| parameter | default | note |
|---|---|---|
| `Operator` | `exp` |  |
| `OutDataTypeStr` | `Inherit: Same as first input` |  |
| `RndMeth` | `Floor` |  |
| `SampleTime` |  |  |
| `SaturateOnIntegerOverflow` | `on` |  |

## MinMax

| parameter | default | note |
|---|---|---|
| `CollapseMode` |  | values: `All dimensions` |
| `Function` | `min` |  |
| `InputSameDT` |  | values: `off` / `on` |
| `Inputs` | `1` |  |
| `OutDataTypeStr` | `Inherit: Inherit via internal rule` |  |
| `RndMeth` | `Floor` |  |
| `SampleTime` |  |  |
| `SaturateOnIntegerOverflow` | `off` |  |

## MultiportSwitch

| parameter | default | note |
|---|---|---|
| `AllowDiffInputSizes` |  | values: `off` |
| `DataPortForDefault` | `Last data port` |  |
| `DataPortIndices` |  |  |
| `DataPortOrder` | `One-based contiguous` |  |
| `DiagnosticForDefault` | `Error` |  |
| `InputSameDT` |  | values: `off` / `on` |
| `Inputs` | `3` |  |
| `OutDataTypeStr` | `Inherit: Inherit via internal rule` |  |
| `RndMeth` | `Floor` |  |
| `SampleTime` |  |  |
| `SaturateOnIntegerOverflow` | `off` |  |

## Outport

| parameter | default | note |
|---|---|---|
| `OutDataTypeStr` |  |  |

## Prelookup

| parameter | default | note |
|---|---|---|
| `BeginIndexSearchUsingPreviousIndexResult` | `off` |  |
| `BreakpointDataTypeStr` | `Inherit: Same as input` |  |
| `BreakpointMax` | `[]` |  |
| `BreakpointMin` | `[]` |  |
| `BreakpointsData` |  | required |
| `BreakpointsDataSource` | `Dialog` |  |
| `BreakpointsSpecification` | `Explicit values` |  |
| `DiagnosticForOutOfRangeInput` | `None` | values: `None` / `Warning` |
| `ExtrapMethod` | `Linear` |  |
| `FractionDataTypeStr` | `Inherit: Inherit via internal rule` |  |
| `IndexDataTypeStr` | `uint32` |  |
| `IndexSearchMethod` | `Binary search` |  |
| `LockScale` | `off` |  |
| `OutputSelection` | `Index and fraction` |  |
| `RemoveProtectionInput` | `off` | values: `off` |
| `RndMeth` | `Floor` |  |
| `SampleTime` |  |  |
| `UseLastBreakpoint` | `off` |  |

Simulink `simulink/Lookup Tables/Prelookup`: one scalar fixed-point input, two outputs -- port 1 the index k (uint32), port 2 the fraction f. Explicit, strictly increasing breakpoints (2 to 4 points), rounded to nearest into the breakpoint type (inherited = the input type). k = the interval the input falls in (clipped to the first / last interval); f = (u - bp[k]) / (bp[k+1] - bp[k]) floored into the fraction type, 0 below the first breakpoint and 1 at or above the last. ExtrapMethod must be Clip: Simulink rejects linear extrapolation on fixed point (the library default). Answered only for the recorded types: input sfix8_En4 / sfix12_En6 / ufix8_En4, breakpoints inherited / fixdt(1,12,6) / fixdt(1,8,4), fraction fixdt(0,32,31) (the default) / fixdt(0,16,15) / fixdt(0,8,7), index uint32, the other parameters at their defaults, host; and inputs int16 / sfix16_En8 / sfix16_En14 / ufix8_En4 with inherited breakpoints (2 to 4, 5, 9 or 17 points) and the library fraction type (left out or 'Inherit: Inherit via internal rule': ufix32_En31 on the host, ufix<w>_En<w-1> of the input's word length on ASIC / FPGA), host and ASIC / FPGA. Not word-length optimised (no double reference). Evidence: Simulink R2026b dense recordings of 84 + 60 + 288 models (the lookup / matrix dense map, the Interpolation Using Prelookup map, the additional coverage map).

## Product

| parameter | default | note |
|---|---|---|
| `CollapseMode` |  | values: `All dimensions` |
| `InputSameDT` |  | values: `off` / `on` |
| `Inputs` | `2` |  |
| `Multiplication` |  | values: `Element-wise(.*)` / `Matrix(*)` |
| `OutDataTypeStr` | `Inherit: Inherit via internal rule` |  |
| `RndMeth` | `Floor` |  |
| `SampleTime` |  |  |
| `SaturateOnIntegerOverflow` | `off` |  |

## RateLimiter

| parameter | default | note |
|---|---|---|
| `FallingSlewLimit` | `-1` |  |
| `InitialCondition` | `0` |  |
| `RisingSlewLimit` | `1` |  |
| `SampleTime` |  |  |
| `SampleTimeMode` | `inherited` |  |

Simulink `simulink/Discontinuities/Rate Limiter`. Answered for one fixed-point or integer scalar input of 4 to 48 bits with fraction length -2 .. word length - 1, `SampleTimeMode` `inherited`, in a single-rate design at sample period 1 or 2 (also inside feedback loops); output type = input type. With T the period, R = `RisingSlewLimit` x T is rounded to nearest (ties up) into the input type (Simulink's Weighted Rise Limit: its overflow is `fxpParameterOverflow`, refused as `PORT_TYPE_INVALID`), F = `FallingSlewLimit` x T is not held in the input type (-1 runs with an unsigned input), the initial condition is rounded into the input type. Each step: d = u - y_prev (y_prev = initial condition at the first step); y = y_prev + R if d > R, y_prev + F if d < F, else u (never overflows). Refused: a negative rising or positive falling limit, a falling limit whose magnitude x T exceeds the input type's maximum by more than one LSB or has a fraction of an LSB of one half or more (below one half it rounds to the whole LSBs, recorded at -0.25 LSB), at period 2 any limit x T that is not a whole number of LSBs within the type, periods other than 1 and 2, multirate designs, other types, vectors. Evidence: Simulink R2026b recordings of 50 + 16 designs (incl. 9 Unit Delay loops and 7 at period 2).

## RateTransition

| parameter | default | note |
|---|---|---|
| `Deterministic` | `on` |  |
| `InitialCondition` | `0` |  |
| `Integrity` | `on` |  |
| `OutPortSampleTime` | `-1` |  |
| `OutPortSampleTimeMultiple` | `1` |  |
| `OutPortSampleTimeOpt` | `Specify` |  |
| `SampleTime` |  |  |

## RealImagToComplex

| parameter | default | note |
|---|---|---|
| `ConstantPart` | `0` |  |
| `Input` | `Real and imag` |  |
| `SampleTime` |  |  |

## RelationalOperator

| parameter | default | note |
|---|---|---|
| `InputSameDT` |  | values: `off` / `on` |
| `Operator` | `<=` |  |
| `OutDataTypeStr` |  |  |
| `RndMeth` |  |  |
| `SampleTime` |  |  |

## Repeat

| parameter | default | note |
|---|---|---|
| `FactorSource` | `Dialog parameter` |  |
| `InputProcessing` | `Columns as channels (frame based)` |  |
| `N` | `5` |  |
| `RateOptions` | `Enforce single-rate processing` |  |
| `SampleTime` |  |  |
| `ic` | `0` |  |

## Reshape

| parameter | default | note |
|---|---|---|
| `OutputDimensionality` | `1-D array` |  |
| `OutputDimensions` | `[1,1]` |  |
| `SampleTime` |  |  |

## Rounding

| parameter | default | note |
|---|---|---|
| `Operator` | `floor` |  |
| `SampleTime` |  |  |

## Saturation

| parameter | default | note |
|---|---|---|
| `LowerLimit` | `-0.5` |  |
| `OutDataTypeStr` |  |  |
| `RndMeth` | `Floor` |  |
| `SampleTime` |  |  |
| `UpperLimit` | `0.5` |  |

The input is first converted to the output type (`OutDataTypeStr`, rounded by `RndMeth`), then limited to `[LowerLimit, UpperLimit]` in that type: a value just above the upper limit can round down to it before the comparison, so size the limits and the output type together. A value beyond the output type's range is limited, never wrapped. Both limits must fit the output type (`UpperLimit` 10 in `fixdt(1,8,4)` is refused: Simulink stops with fxpParameterOverflow). There is no `SaturateOnIntegerOverflow` parameter (writing it is `BAD_PARAM`): the block always limits.

## ShiftArithmetic

| parameter | default | note |
|---|---|---|
| `BinPtShiftNumber` | `0` |  |
| `BitShiftDirection` | `Bidirectional` |  |
| `BitShiftNumber` | `8` |  |
| `BitShiftNumberSource` |  | values: `Dialog` / `Input port` |
| `SampleTime` |  |  |

## SignalSpecification

| parameter | default | note |
|---|---|---|
| `Dimensions` | `-1` |  |
| `OutDataTypeStr` | `Inherit: auto` |  |
| `SampleTime` |  |  |

## Signum

| parameter | default | note |
|---|---|---|
| `SampleTime` |  |  |

## Sqrt

| parameter | default | note |
|---|---|---|
| `AlgorithmType` |  |  |
| `IntermediateResultsDataTypeStr` |  |  |
| `Iterations` |  |  |
| `Operator` | `sqrt` |  |
| `OutDataTypeStr` | `Inherit: Same as first input` |  |
| `RndMeth` | `Floor` |  |
| `RsqrtAlgorithmType` |  |  |
| `SampleTime` |  |  |
| `SaturateOnIntegerOverflow` | `on` |  |
| `SqrtAlgorithmType` |  |  |

Runs on the Python engine only: there is no C-core form, so `"engine": "c"` refuses it (`UNSUPPORTED_WIDTH`, "Python path only") and `auto` runs it in Python. Answered with Operator `sqrt`, AlgorithmType `Exact`, an inherited intermediate type, and an explicit fixed-point output type -- or the library default output `Inherit: Same as first input` for the input types, roundings and hardware that were recorded.

## StateControl

| parameter | default | note |
|---|---|---|
| `SampleTime` |  |  |
| `StateControl` | `Classic` |  |

## Sum

| parameter | default | note |
|---|---|---|
| `AccumDataTypeStr` | `Inherit: Inherit via internal rule` |  |
| `CollapseMode` |  | values: `All dimensions` |
| `InputSameDT` |  | values: `off` / `on` |
| `Inputs` | `++` |  |
| `OutDataTypeStr` | `Inherit: Inherit via internal rule` |  |
| `RndMeth` | `Floor` |  |
| `SampleTime` |  |  |
| `SaturateOnIntegerOverflow` | `off` |  |

## Switch

| parameter | default | note |
|---|---|---|
| `AllowDiffInputSizes` |  | values: `off` |
| `Criteria` | `u2 > Threshold` |  |
| `InputSameDT` |  | values: `off` / `on` |
| `OutDataTypeStr` | `Inherit: Inherit via internal rule` |  |
| `RndMeth` | `Floor` |  |
| `SampleTime` |  |  |
| `SaturateOnIntegerOverflow` | `off` |  |
| `Threshold` | `0` |  |

## TappedDelay

| parameter | default | note |
|---|---|---|
| `DelayOrder` | `Oldest` |  |
| `NumDelays` | `4` |  |
| `SampleTime` |  |  |
| `includeCurrent` | `off` |  |
| `samptime` | `-1` |  |
| `vinit` | `0.0` |  |

## TappedDelayEnabledSynchronous

| parameter | default | note |
|---|---|---|
| `DelayOrder` | `Oldest` |  |
| `NumDelays` | `4` |  |
| `SampleTime` |  |  |
| `includeCurrent` | `off` |  |
| `samptime` | `-1` |  |
| `vinit` | `0.0` |  |

## Trigonometry

| parameter | default | note |
|---|---|---|
| `ApproximationMethod` | `None` |  |
| `NumberOfIterations` | `11` |  |
| `Operator` | `sin` |  |
| `SampleTime` |  |  |

## UnaryMinus

| parameter | default | note |
|---|---|---|
| `SampleTime` |  |  |
| `SaturateOnIntegerOverflow` | `off` |  |

## UnitDelay

| parameter | default | note |
|---|---|---|
| `InitialCondition` | `0` |  |
| `InputProcessing` |  | values: `Elements as channels (sample based)` |
| `SampleTime` |  |  |

## UnitDelayEnabledResettableSynchronous

| parameter | default | note |
|---|---|---|
| `InitialCondition` | `0.0` |  |
| `SampleTime` | `-1` |  |

## UnitDelayEnabledSynchronous

| parameter | default | note |
|---|---|---|
| `InitialCondition` | `0.0` |  |
| `SampleTime` | `-1` |  |

## UnitDelayResettableSynchronous

| parameter | default | note |
|---|---|---|
| `InitialCondition` | `0.0` |  |
| `SampleTime` | `-1` |  |

## Upsample

| parameter | default | note |
|---|---|---|
| `InitialConditions` | `0` |  |
| `InputProcessing` |  | must write; values: `Elements as channels (sample based)` |
| `RateOptions` | `Enforce single-rate processing` |  |
| `SampleOffset` | `0` |  |
| `SampleTime` |  |  |
| `UpsamplingFactor` | `3` |  |

## WrapToZero

| parameter | default | note |
|---|---|---|
| `SampleTime` |  |  |
| `Threshold` | `255` |  |

Simulink `simulink/Discontinuities/Wrap To Zero` (a masked subsystem). Answered for one fixed-point or integer scalar input of 4 to 48 bits with fraction length -2 .. word length; output type = input type. `Threshold` (library default 255) is rounded to nearest (ties up) into the input type, with the same overflow rule as Dead Zone. Output: 0 where u > Threshold, else u. Refused: other types, vectors. Evidence: Simulink R2026b recordings of 50 + 12 designs (thresholds inside the type, at its maximum, on half LSBs).

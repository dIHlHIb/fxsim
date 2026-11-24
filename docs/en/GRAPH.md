# fxgraph simulation within recorded configuration domains

`python3 -I engine/fxgraph < request.json` simulates a block diagram step by step. Bit-exact comparisons apply within the supported, recorded domains. The engine refuses configurations outside its supported ranges. A range can include combinations that were not individually tested.
Each result reports outputs, compiled types, overflow events and hardware cost. Supply one JSON request (which can span several lines) or a JSONL batch with one request and one result per line. [Ground rules and errors](README.md)

## Request

```json
{"design": {"blocks": {"x": {"type": "Inport"},
                       "S": {"type": "Sum", "Inputs": "++", "OutDataTypeStr": "fixdt(1,20,16)"},
                       "D": {"type": "UnitDelay", "InitialCondition": "0"},
                       "A": {"type": "Gain", "Gain": "0.75", "ParamDataTypeStr": "fixdt(1,16,14)",
                             "OutDataTypeStr": "fixdt(1,20,16)"},
                       "y": {"type": "Outport"}},
            "lines": [{"src": "x", "dst": "S", "dport": 1}, {"src": "A", "dst": "S", "dport": 2},
                      {"src": "S", "dst": "D"}, {"src": "D", "dst": "A"}, {"src": "S", "dst": "y"}]},
 "inputs": {"x": {"s": 1, "wl": 16, "fl": 14, "ints": [0, 1627, 3190]}},
 "diagnostics": true, "cost": true}
```

| Field | Meaning | Default |
|---|---|---|
| `design` | `blocks` (name -> `{"type": ..., parameters}`) and `lines` | required |
| `inputs` | one sequence per Inport (forms below) | required when the design has Inports |
| `hw` | target hardware, see README | host |
| `nsteps` | number of steps; may shorten the inputs, not lengthen them | input length |
| `outputs` | `"outports"`, `"all"` (every block), or a list of block names to record | `"outports"` |
| `diagnostics` | per-block overflow / saturation / division-by-zero counts | `false` |
| `cost` | hardware cost of the design as given | `false` |
| `detail` | `"summary"`: answer with the summary only and write the full result to a file | `"full"` |
| `engine` | `"auto"` (C core when possible), `"c"`, `"python"`; results are identical. Some shapes run on the Python path only (`"c"` is refused with `UNSUPPORTED_WIDTH`): see "Python path only" below | `"auto"` |
| `stop_time` | multirate designs only: simulate up to this time (a decimal or rational string) | |
| `catalogue` | `true` alone: return the block catalogue instead of simulating | |

### Blocks and lines

- Block names are the keys of `blocks`. `type` is the Simulink block type; parameters use the Simulink dialog names
  and Simulink-style string values (`"0.75"`, `"[0.25 0.5 0.25]"`, `"fixdt(1,20,16)"`). Numeric parameters may also be
  plain arithmetic on numeric literals (`"2^-8"`, `"1/3"`, `"[1 2 3]*0.5"`, and list elements such as
  `"[1/3 0.1; -0.7 2/3]"`), evaluated in double precision as MATLAB does; workspace variables, functions and constants such as `pi` are refused. The full list of types,
  parameters and defaults is [BLOCKS.md](BLOCKS.md), also returned by `{"catalogue": true}`.
- A line is `{"src", "dst", "sport", "dport"}`; ports count from 1 and default to 1. An input port driven twice is
  `BAD_CONNECTION`; an undriven input port is refused (`UNSUPPORTED`: Simulink grounds it to 0 and runs, which is not
  supported here -- connect it). A feedback loop must pass through a Unit Delay, a Delay or a (Forward Euler) Discrete-Time Integrator
  (`ALGEBRAIC_LOOP`, which names the blocks on the cycle).
- Data types: `fixdt(s,w,f)` (word length 1..4096; see [ENGINE.md](ENGINE.md) for which path runs wide types), `int8` .. `uint32`, `boolean`, and the `Inherit: ...` strings
  the parameter offers in Simulink (`Inherit: Inherit via internal rule`, `Inherit: Same as input`, ...). Floating
  point signals (`double`, `single`) run on the floating-point path, for the recorded shapes only (see
  [Floating-point signals](#floating-point-signals-double--single) below); other floating-point designs are refused.
- `Inherit: Inherit via back propagation` on a Constant or a Data Type Conversion (the latter's library default)
  takes its type from the one block it drives: the other input of a Relational Operator, of a Switch's data ports, or
  of a two- / three-input Sum or two-input MinMax (three inputs: the other input in the larger word class -- 8 / 16 / 32 bits -- on a tie the lower port, whatever
  the sign or fraction length, as recorded in Simulink); the
  explicit output type of a Gain; of a two- / three-input Product, the other input's type as for a Sum (a boolean
  other input counts as 8 bits and makes the source boolean); an Outport's explicit type (also through one Unit
  Delay). A Sum, MinMax or Product with an explicit output type gives that output type instead when its word class
  (the smallest native word holding it: 8 / 16 / 32 / 64 bits) is larger than the other input's (Sum / MinMax: host,
  signed types up to 32 bits; Product: host and ASIC/FPGA, up to 128 bits). On ASIC/FPGA an internal-rule Sum whose
  output reaches an Outport with an explicit type in a larger word class is rejected, as Simulink rejects it. The value is then quantised as for an explicit type. Destinations that would make it double (a Gain with
  the internal rule, an untyped Outport, a Switch / Multiport Switch control port), more than one destination, and
  other blocks are refused by the fixed-point engine; the recorded floating-point shapes (a Constant or Data Type
  Conversion into untyped Outports, see Floating-point signals) run on the floating-point path in double instead. A Sum's `Inherit: Same as accumulator` output is its accumulator type. When the internal-rule
  Sum or the MinMax the source feeds drives a Switch or Multiport Switch (directly or through blocks with inherited
  types), the design is refused: Simulink compiles source and block as double there (control port), takes the other
  data port's type (data port) or rejects the model. Two recorded shapes are answered: a Data Type Conversion with an
  inherited type in between ends the chain (the source keeps the other input's type); and on the host an
  internal-rule Sum reaching a Switch data port only through Unit Delays, Abs, Saturation, Unary Minus or
  internal-rule Sums, when the Switch's two data inputs then have the same type (otherwise refused: Simulink
  rejects it where the types disagree).
- A two-input Sum or Product with back propagation takes a typed Outport's type (also through a Unit Delay); into a
  Gain, a Product input or an untyped Outport it takes its first input's type, or the second input's when that one's
  word class is larger (host, inputs up to 32 bits; recorded with `RndMeth` Floor and saturation off only -- other
  settings are refused).
- An accumulator Sum (two inputs; it feeds a Unit Delay that feeds it back and drives nothing else) whose other
  destinations are Outports with an explicit type: with back propagation the Sum and the delay take the Outports'
  type (also with an internal-rule Gain or an explicit Data Type Conversion branch); Outports of different types are
  rejected as in Simulink; with only an explicit Data Type Conversion before an untyped Outport the Sum takes its
  external input's type. With the internal rule the loop starts from the typed Outport's type T: answered with T where
  the Sum's internal rule on its external input and T gives T again, rejected as in Simulink otherwise. Answered for
  the recorded settings only: external input signed 8, 12 or 16 bits with fraction 2, 4 or 8, or `fixdt(0,16,8)`;
  `Inputs` `++` or `+-`; `RndMeth` Floor; a 32-bit host or ASIC/FPGA; Outport types signed 7 to 17, 24 or 32 bits
  with fraction 2, 4, 8, 13 or 16 (internal rule: 8, 12, 16, 24 or 32 bits, fraction 2, 4, 8 or 16), or
  `fixdt(0,16,8)`; other settings are refused.
- A Constant of type `boolean` is true for any non-zero value (`true` / `false` may be written); a Data Type
  Conversion to `boolean` gives (input != 0), whatever its rounding and overflow settings.
- A Delay with an enable port and / or an external reset (`ShowEnablePort` `on`: port 2 the enable; `ExternalReset`
  set: the next port the reset): at an enabled step (enable above 0) it outputs its oldest state and shifts the input
  in; at a disabled step it holds its output and its state, and before the first enabled step that held output is 0,
  not the initial condition. Without an enable port every step is enabled. A reset acts at enabled steps only, before
  the output, filling the state with the initial condition: `Level` when the reset is non-zero at this step or the
  previous one, `Level hold` when it is non-zero at this step, `Rising` / `Falling` / `Either` on an edge of the
  reset's sign. Answered with a dialog length and initial condition, data of 8 to 32 bits, outside feedback loops,
  for the recorded combinations: the enable alone, the enable with a `Level` reset and a `Rising` reset alone at any
  length 1 to 64; the enable with a `Rising` reset and `Falling` / `Either` / `Level` / `Level hold` alone at lengths
  1, 2, 3, 4, 5, 6, 8, 16, 24, 32, 33 and 64; the enable with a `Falling` / `Either` / `Level hold` reset at lengths 2,
  5 and 33 (host only); a boolean enable and reset (a uint8 enable also alone and with a Level reset); host hardware, plus ASIC for the
  enable alone and with a `Level` or `Rising` reset. Other forms (other lengths or port types, the length or initial
  condition from a port as well) are refused. Runs on the C core (engine c / auto) as on Python.
- A Mux's inputs must share one data type (Simulink rejects mixed types); a Selector's `InputPortWidth` must be the
  input's width or `-1`. Left out, a Selector takes the library values `Indices` `[1 3]` and `InputPortWidth` 3
  (listed in `assumed_defaults`); `IndexParamArray` is not taken -- give `Indices`. `IndexMode` `Zero-based` counts the indices from 0.
- Discrete-Time Integrator methods: Integration or Accumulation, Forward Euler, Backward Euler or Trapezoidal
  (T = 1). With h_k = round(K u_k) in the state type (K/2 for Trapezoidal, whose internal-rule type has one fraction
  bit more): Forward Euler y_k = x_k, x_k+1 = x_k + h_k; Backward Euler x_k = x_k-1 + h_k, y_k = x_k; Trapezoidal
  y_k = x_k-1 + h_k, x_k = y_k + h_k (each clamped to the output limits when LimitOutput is on). Backward Euler and
  Trapezoidal take the internal-rule output type only and are refused inside feedback loops (not recorded); they
  are answered for the recorded input types and gain values only: sfix16_En8, sfix12_En4, sfix8_En2, ufix12_En6,
  sfix20_En12 with gain 1, 0.5, 0.25 or 2; sfix8_En4 with 0.5 or 1; sfix32_En16, ufix8_En4, sfix24_En0, int8 with
  0.125. (Gains such as 0.1, 1.5 or 3 were recorded too, but K x T is not a power of two there, so the internal-rule
  output is not a binary-point type and the engine refuses them.)
- Discrete-Time Integrator external reset and initial condition (Forward Euler, no output limits, internal-rule
  output, outside feedback loops; recorded inputs: sfix16_En8, sfix12_En4, sfix8_En2, ufix12_En6, sfix20_En12 with
  gain 1 or 0.5, a boolean reset alone on sfix32_En16 / ufix8_En4 with gain 0.25, an external initial condition alone
  on sfix24_En0 / int8 with gain 2): ports u, then the reset (boolean / int8 / uint8), then the initial condition,
  which must have the output type (Simulink rejects another). Reset triggers -- rising: from <= 0 to > 0; falling:
  from > 0 to <= 0; either; level: non-zero, or a change from non-zero to zero; sampled level: non-zero (the previous
  value starts at 0). On a trigger the state becomes the initial condition (the external one at that step) before
  the output. The state port (a line with `"sport": "state"`) is refused (the recorded feedback shape
  through a Gain was not reproduced for the integrator's and the Gain's saturation, rounding and sample time); take
  the integrator output instead.
- A Discrete-Time Integrator's output type is the internal rule or an explicit fixdt; other `Inherit:` rules are
  refused. With `LimitOutput` on, its limits must be finite numbers inside the state type (the library default
  `-inf` / `inf` is a parameter overflow in Simulink).
- Explicit types are the safe choice. Inherited types inside a feedback loop are resolved as Simulink does, and
  give `TYPE_MISMATCH` where Simulink reports a data type mismatch.
- Names of blocks inside subsystems (from `.slx`) are paths: `SUD/Q0_S`.
- Complex to Magnitude-Angle / Magnitude-Angle to Complex (recorded shapes only): ApproximationMethod
  `CORDIC` (`None` takes floating point only and is refused on fixed point, as Simulink does), `NumberOfIterations`
  3 / 6 / 11 (11 is refused on an 8-bit input, as Simulink does), `ScaleReciprocalGainFactor` on / off, 'Magnitude
  and angle', inputs of one of sfix8_En5, sfix12_En9, sfix16_En13. Outputs: magnitude / re / im in (1, w+2, f), the
  angle in (1, w, w-3) -- the same as the CORDIC atan2. The complex input comes from Real-Imag to Complex; the
  complex output goes to Complex to Real-Imag or an Outport (`<name>.re` / `<name>.im`).
- Triggered and resettable subsystems (recorded shapes only): a level `S` holds `S/Trigger` of type `TriggerPort` (`TriggerType` rising / falling /
  either) or `S/Reset` of type `ResetPort` (`ResetTriggerType` rising / falling / either), with one line from an int8
  control into it. Rising is negative -> zero or positive, or zero -> positive unless that zero came straight from a
  negative value one step earlier; falling is the mirror; either is both; no edge at step 0. A triggered subsystem
  runs once in each trigger step (its states and its `EnabledOutport` outputs `S/Out1` ... hold between triggers); a
  resettable one runs every step and its Unit Delays start from their initial condition in a reset step. Inside: Sum,
  Gain and Unit Delay blocks only; single rate.
- Switch Case action subsystems, Merge and Data Store (recorded shapes only):
  - `{"type": "SwitchCase", "CaseConditions": "{1, [2 3]}", "ShowDefaultCase": "on"}` on an int8 input: output port
    k is active when the input equals one of case k's values, the last port (default) when none does. The default
    port is required; a value in two cases is refused.
  - An action subsystem `S` holds a block `S/Action` of type `ActionPort` (`InitializeStates` `held` or `reset`)
    driven by one Switch Case output, and outputs `S/Out1` ... of type `EnabledOutport`; its other blocks are `S/...`.
    It runs only in the steps its port is active. `held`: its states keep their values while it does not run;
    `reset`: they return to their initial values when it starts running again.
  - `{"type": "Merge", "Inputs": "3", "InitialOutput": "0"}`: every input from an action subsystem of one Switch
    Case, together covering all its ports (default included); the output is the running subsystem's output. A Merge
    fed from outside an action subsystem is refused as Simulink refuses it (`InvMergeConnNonCondSubsysSrc`).
  - Data Store: one `DataStoreMemory` (`DataStoreName`, `InitialValue`, an explicit `fixdt` `OutDataTypeStr`), one
    `DataStoreRead` and one `DataStoreWrite` of that name (written in the store's type). A data path from the read to
    the write, or a lower read `Priority`, reads first: the read gives the value written in the previous step
    (`InitialValue` at step 0). A lower write `Priority` gives the value written in the same step. Equal or missing
    priorities without a data path are refused.
  - The If block is refused (Simulink refuses an If input of a fixdt type; its behaviour on int8 / uint8 / boolean
    inputs, which Simulink runs, is not recorded); use a Switch Case on an integer. fxexport writes them as Simulink saves them (action subsystems with an action port; a Data Store's `Priority` kept).
- Discrete Filter, Discrete Transfer Fcn and Biquad Filter are answered only in the recorded shapes, in a single-rate design at the base step on the host (a Discrete Filter
  in the shapes of the first input class below also on ASIC / FPGA, where an inherited product is the full product of
  its operands and an inherited accumulator adds ceil(log2(terms)) guard bits to it), 2 to 4
  coefficients per polynomial (at least one delay, a0 = 1 or 2 -- Simulink keeps the coefficients and divides the
  feedback sum by a0 --, numerator not all zero; Biquad: up to 2 sections). Two
  input classes: a signed input of 12 to 16 bits with 8 to 14 fraction bits, or exactly fixdt(1,8,6),
  fixdt(1,10,8), fixdt(1,12,10) (a dense recording of every input value). On the first: explicit internal types as
  the wide set (state fixdt(1,24,16), coefficients fixdt(1,16,14), products fixdt(1,32,26), accumulators
  fixdt(1,40,26), multiplicand fixdt(1,24,16), output fixdt(1,16,10)) or the narrow set (12,8 / 10,8 / 20,12 /
  20,12 / 12,8 / 12,8), any structure; inherited types all left to the library, optionally with the output
  fixdt(1,16,10), Direct form II only. On the small inputs: Direct form II / Transfer Fcn with everything
  inherited, or only the output fixdt(1,16,12), or only both products fixdt(1,24,18), or the dense explicit set
  (state fixdt(1,24,18), coefficients fixdt(1,16,14) / fixdt(1,16,13), products and accumulators fixdt(1,32,24),
  multiplicand fixdt(1,24,18), output fixdt(1,16,12)); Direct form I / II transposed with only the products or the
  dense set; Floor and wrap there. A Discrete Filter on exactly fixdt(1,16,14) / fixdt(1,16,8) inputs, any of the three
  structures, also with only the products and accumulators explicit -- fixdt(1,32,28) / fixdt(1,40,28) or
  fixdt(1,24,20) / fixdt(1,32,20) -- or those of fixdt(1,32,28) with the state and output fixdt(1,32,28) too, the rest
  left to the library (the output then the numerator accumulator). And on exactly fixdt(1,32,16) / fixdt(1,24,12) /
  fixdt(1,18,9) / fixdt(1,20,16) / fixdt(1,8,0) / fixdt(1,16,15) inputs (host): Direct form II / Transfer Fcn with every
  internal type left to the library -- an inherited product is 32 bits, or 64 when the operand words add up to more
  than 32, the accumulator likewise -- optionally with the output fixdt(1,32,24) / fixdt(1,24,16); a Biquad with the
  accumulator's output type or those two. Direct form I / II transposed with inherited types and a written output are
  refused (not reproduced at every step). A non-zero InitialStates only in its recorded combinations. Biquad:
  output the accumulator's type or fixdt(1,16,10) (small inputs: fixdt(1,16,12), Direct form II transposed,
  and there a scalar IC: every state starts at it); scale values typed by the coefficients' magnitude rule, ScaleValues 1 or one per section plus one. Outside them the refusal names the recording that would
  open the shape. Zero-Order Hold: SampleTime -1, 1, 2, 3 or 4. A Mux of signals at different sample periods is
  refused (Simulink runs the vector at the fastest).
- Discrete Filter (`DiscreteFilter`) and Discrete Transfer Fcn (`DiscreteTransferFcn`): `Numerator`,
  `Denominator` (lists of coefficients), one signed fixed-point scalar input; the internal types
  `StateDataTypeStr`, `NumCoefDataTypeStr`, `DenCoefDataTypeStr`, `NumProductDataTypeStr`, `DenProductDataTypeStr`,
  `NumAccumDataTypeStr`, `DenAccumDataTypeStr`, `MultiplicandDataTypeStr`, `OutDataTypeStr` explicit `fixdt` or left
  to the library; `RndMeth` Floor (default) / Nearest / Zero, `SaturateOnIntegerOverflow` on / off (default off), and
  the leading denominator coefficient 1 after quantisation. Either every internal type is explicit, or all are left
  to the library (the output type may still be written); other mixes are refused (not recorded). A coefficient
  outside an explicit coefficient type is refused as Simulink stops (`PORT_TYPE_INVALID`).
  - The library's types: coefficients at the best precision of the whole coefficient list with the input's word
    length; state and multiplicand = the input type; products 32 bits with the summed fraction lengths;
    accumulators 32 bits with their product's fraction length; output = the numerator accumulator (Direct form II;
    Direct form I and II transposed need an explicit `OutDataTypeStr` -- Simulink refuses both inherited).
  - `FilterStructure`: Discrete Filter `Direct form II` (default), `Direct form I`, `Direct form II transposed`;
    Discrete Transfer Fcn `Direct form II` (default). `Direct form I transposed` is refused.
  - The coefficients are rounded (nearest) to their coefficient types; every product is cast to its product type,
    sums are formed in the accumulator types, the state is cast to the state type and the result to the output type.
  - A Discrete Filter's coefficients are powers of z^-1 (aligned left); a Discrete Transfer Fcn's are powers of z,
    so a shorter numerator is aligned right (`[1]` over `[1 -0.9]` is z^-1 / (1 - 0.9 z^-1)).
  - `InitialStates`: one scalar, every state (Direct form I: the input history, with `InitialDenominatorStates` 0).
- Biquad Filter (`BiquadFilter`, the DSP library block): `BiQuadCoeffs` (one row `[b0 b1 b2 1 a1 a2]` per
  section), `ScaleValues` (one per section, optionally one more for the output; a gain of 1 is skipped),
  `IIRFiltStruct` `Direct form II transposed` (default), `Direct form II` or `Direct form I`; one signed fixed-point
  input. Every data type stays at the library's mode except the output (`outputMode` `Same as accumulator`, or
  `Binary point scaling` with `outputWordLength` / `outputFracLength`): then every product, accumulator, state and
  section input / output is in the input type, numerator and denominator coefficients at their best precision with
  the input's word length, scale values with 14 fraction bits; Floor, wrap. `Direct form I transposed` and other
  type modes are refused.
- Zero-Order Hold (`ZeroOrderHold`, `SampleTime`): -1 (inherited) passes the input unchanged, 1 too when it is fed
  by an Inport at the base step (after a slower signal Simulink holds at the base step: refused); an integer k >= 2 makes a
  multirate design whose output at t = 0, k, 2k, ... is the input at that time (offset 0), in the input's type. Its
  input must be an Inport at the base step (the recorded shape).
- Signal From Workspace (in a saved `.slx`; not a JSON block type -- use an Inport): read as an Inport whose data the
  request gives, in the type of its data `X`. `info.workspace_sources` gives `X`, `info.workspace_source_settings`
  its `Ts` and `OutputAfterFinalValue`: at t = i*Ts the block outputs X(i); after the data, 0 (`Setting to zero`),
  the last value (`Holding final value`) or X(i mod n) (`Cyclic repetition`) -- give the input for the whole run.

### Inputs

`ints` are stored integers: real value x 2^fl, already quantised.

- Fixed point: `{"s": 1, "wl": 16, "fl": 14, "ints": [...]}` or `{"type": {"s": 1, "wl": 16, "fl": 14}, "ints": [...]}`.
  `s`, `wl`, `fl` and every sample must be JSON integers.
- Boolean: `{"type": "boolean", "ints": [0, 1, 1, 0]}`. Booleans drive logic and selection blocks freely; into
  arithmetic only the recorded host shapes are answered (a Gain or Data Type Conversion of the boolean, a Sum or
  Product of the boolean and one fixed-point operand, a three-input Sum with one boolean -- counted as uint8 0 / 1 --
  and two fixed-point operands over the recorded types, sign strings, output types, `RndMeth` Floor / Nearest / Zero /
  Convergent / Ceiling and accumulators internal rule / fixdt(1,32,16) / fixdt(1,12,4); `Inherit: Same as first input`
  after a boolean first operand is rejected as in Simulink); others are refused -- convert with a Data Type Conversion
  to `uint8` first. A boolean into a Gain with internal-rule parameter and output types: the parameter (= output)
  type is the shortest word holding the gain exactly at best precision -- on the host the first of 8 / 16 / 32 bits
  (32 when none is exact), on ASIC/FPGA any length (gain 1 there is refused).
- A boolean into a Relational Operator is compared as the number 0 / 1, exactly (against a boolean or fixed point of
  up to 32 bits). Boolean data mixed with fixed-point data on a two-data-port Switch / Multiport Switch: under the
  internal rule the boolean counts as `uint8`; the data input with the greater integer range gives the output type
  (boolean output: the selected value != 0; fixed-point output: a selected boolean becomes 0 / 1); a fixed-point type
  of exactly `uint8`'s range is refused. A Switch with `Inherit: Same as first input` takes the first data port's type.
- A one-input Logical Operator (AND, OR, NAND, NOR, XOR, NXOR with `Inputs` 1) over a vector reduces its elements to
  one boolean; NOT works element by element. A Multiport Switch takes vector data ports with a scalar control.
  Relational Operator `isNaN` is always false and `isFinite` always true on fixed point / boolean (`isInf` is refused).
- Bitwise Operator `BitMask` and integer parameters (`BitShiftNumber`, `numBits`, ...) may be written as MATLAB
  evaluates them: `0xFF`, `0b1010`, `2^4-1`, `uint8(200)`, `intmax('uint8')`, `bitshift(1,3)`, `bitcmp(uint8(5))`,
  `pow2(5)`; an integer-class mask acts by its value, like a plain number. `true` / `false` for an integer parameter is
  refused (Simulink makes it boolean and Shift Arithmetic rejects it).
- Bit blocks on vectors: Bitwise Operator with one input and no mask reduces the elements (AND / OR / XOR / NAND / NOR
  across them); with a mask or two inputs it works element by element. Shift Arithmetic works per element, also with a
  vector `BitShiftNumber`. A negative `BitShiftNumber` with direction Left / Right leaves the input unchanged.
- Extract Bits: `numBits` 0 acts as 1; a field reaching below bit 0 (a negative index, or more bits from the MSB than
  the word has) shifts the input up; a descending `bitIdxRange` [a b] extracts bit a alone.
- A Constant written `true` / `false` with its default type is boolean.
- Compare To Zero (`relop`, output boolean or uint8) compares the input's value with 0 (a boolean as 0 / 1). Detect
  Rise Positive / Rise Nonnegative / Fall Negative / Fall Nonpositive output (u REL 0) AND NOT the previous step's
  (u REL 0); the previous result starts as `vinit` != 0 and is held in the output type, so a `vinit` the output type
  cannot hold (uint8, -1) is a parameter overflow as in Simulink.
- Dead Zone, Rate Limiter, Wrap To Zero (scalar fixed-point / integer inputs of 4 to 48 bits, fraction length -2 to
  the word length - 1; non-finite parameters, Dead Zone / Wrap To Zero inside a feedback loop or a multirate design
  refused; output type = input type; full rules and evidence under each block in BLOCKS.md). The parameters
  are rounded to nearest (ties up) into the input type, and an overflow beyond one LSB is a parameter overflow, as in
  Simulink (`PORT_TYPE_INVALID` at the parameter). A negative Start with an unsigned input, Threshold 10 in
  sfix8_En4, or a rising slew of 4 in sfix8_En8 are examples.
  - Dead Zone: 0 between Start and End, u - End above, u - Start below; that difference saturates
    (`SaturateOnIntegerOverflow` on, the library default) or wraps (off) into the input type, which matters when
    Start > 0 or End < 0. Library defaults are -0.5 / 0.5. Start above End is refused.
  - Wrap To Zero: 0 where u > Threshold, else u. Threshold defaults to the library's 255.
  - Rate Limiter: single-rate designs at sample period 1 or 2, inherited sample-time mode. With T the period, the
    output is y_prev + R*T where u - y_prev > R*T, y_prev + F*T where u - y_prev < F*T, and u otherwise (y_prev
    starts at the initial condition). Defaults are R = 1, F = -1 and initial condition 0. R*T is held in the input
    type (Simulink's Weighted Rise Limit); the falling limit is not: -1 runs with an unsigned input. A positive
    falling limit, a negative rising limit, a falling limit whose magnitude (times T) exceeds the input type or has a
    fraction of an LSB of one half or more, and at period 2 a limit times 2 that is not a whole number of LSBs in the type are refused; a falling limit times 2
    beyond the type is Simulink's parameter overflow (Weighted Fall Limit).
- Trigonometric Function: Operator `atan2` with ApproximationMethod CORDIC, two inputs of one fixed-point type, 8 to
  32 bits, 1 to min(word length - 1, 24) iterations. The output is sfix(w, w-3) for signed inputs and sfix(w, w-2)
  for unsigned ones.
  - On the x axis the output is exact: 0 for x >= 0, and pi rounded to nearest for x < 0.
  - Otherwise a point with x < 0 is negated and +-pi is added at the end. A point with y < 0 is then mirrored above
    the axis and the angle negated.
  - The vectoring iterations use arithmetic right shifts, and the arctangent table is rounded to nearest in the
    output type.
  - Simulink rejects two inputs of different types and iterations equal to the word length.
  - sin / cos / sincos with CORDIC and other approximations are refused for now.
- Trigonometric Function sin / cos with ApproximationMethod CORDIC: one signed fixed-point input of 8 to 32 bits,
  1 to min(11, word length - 2) iterations; output sfix(w, w-2). The angle is floored to w-2 fraction bits and
  rotated by CORDIC (floor shifts, arctangents rounded at w-2 bits). Only angles within +-pi/2 are answered: a step
  beyond +-pi/2 (Simulink's quadrant reduction is not reproduced bit-exactly), or within 4 steps of
  2^-(w-2) of +-pi/2, is refused at that step -- reduce the angle to [-pi/2, pi/2] before the block and apply the
  sign yourself; an angle outside [-2 pi, 2 pi] stops the run as in
  Simulink. More iterations than the word length are rejected as Simulink rejects them; sincos is refused.
- 1-D / 2-D Lookup Table with `TableSource` and / or `BreakpointsForDimension1Source` (2-D: also
  `BreakpointsForDimension2Source`) `Input port`: the inputs first (1-D: u; 2-D: u1, u2), then the table (1-D: a
  vector of n; 2-D: an m x n matrix signal, `PortDimensions` `[m n]`), then the breakpoints (vectors of n, or m and n). Each step uses that step's port values, in the port
  signals' own types, computed as the dialog block computes it; breakpoints that are not strictly increasing stop
  the run at that step (`SIM_DIAGNOSTIC`), as Simulink's simulation stops. Answered for the recorded settings only
  (`engine/lutport_domain.json`: port and input types, table / output types, interpolation, extrapolation, internal-rule
  priority, hardware, number of points); n-D tables from ports are refused. Python path only. A dialog table may
  be written `reshape([v1 ... vk], m, n)` with numeric literals (filled column by column, as MATLAB does).
- Multiport Switch `DataPortOrder` `Specify indices`: `DataPortIndices` is a cell, `{7,9}`, with one distinct index
  per data port (`Inputs` = their number); Simulink rejects a numeric vector `[7 9]` (InvCaseConditions), and so
  does the engine. The control selects the port whose index equals it, else the default port (the last data port,
  or the additional one). Recorded and answered: a uint8 / uint16 control with 2 to 6 indices in 0 .. 15, or an
  int8 control with indices in -4 .. 15; data ports sfix16_En8, sfix12_En4 or uint8 (mixed); DiagnosticForDefault
  None or Warning; and every other recorded setting -- control type, data port types, output type, default port,
  DiagnosticForDefault (Error stops the run at the first control value matching no index, as Simulink does), index
  order and number, hardware -- with indices inside the range recorded for it (`engine/msh_domain.json`). Not inside
  feedback loops. Python path only.
- Delay with `DelayLengthSource` `Input port` (a uint8 / uint16 / uint32 / int8 / int16 / ufix4 length on port 2,
  an integer `DelayLengthUpperLimit` 1 to 64 that port can hold; each step's length is clamped into 0 .. the limit,
  length 0 passes the input through) or `InitialConditionSource` `Input port` (`DelayLength` 1 to 32; the port's
  first value, in the input's type, fills the first `DelayLength` steps), for data of 8 to 32 bits with a fraction
  of 0 up to the word length. With the length from a port, `PreventDirectFeedthrough` `on` reads a length below 1
  as 1, and `ExternalReset` `Rising` (a boolean reset on port 3) fills the history with the initial condition at a
  step where the reset changes from 0, before the output (a length 0 at such a step is refused); these two are
  answered only as recorded -- host, int8 data, an int8 length port, `DelayLengthUpperLimit` 4, a dialog initial
  condition, `DiagnosticForDelayLength` `None`, not both at once; inside a feedback loop only the recorded accumulator forms; Shift Arithmetic with `BitShiftNumberSource` `Input port` (an int8 shift of -7 to 7
  on port 2: Left / Right shift by k > 0 and pass the value for k <= 0, Bidirectional shifts right for k > 0 and
  left for k < 0). Data sfix16_En8, sfix12_En4, ufix12_En6, sfix8_En2, sfix20_En12 or int16 (the recorded types);
  also every other recorded setting (data and shift-port types, direction, `BinPtShiftNumber`, hardware), each with
  the run-time shift range recorded for it (`engine/msh_domain.json`); and, by the general rule, any data fixdt of 8
  to 32 bits with a fraction of 0 up to the word length, an integer shift port of 8 to 32 signed or 4 to 20 or 32
  unsigned bits, any direction, `BinPtShiftNumber` -3 to 2, host or ASIC: the stored value shifted by the port value (left:
  wrapped into the word; right: arithmetic). The port value is saturated to int16; where the right-shift amount
  minus `BinPtShiftNumber` overflows 16-bit two's complement (or is -32768), the shift goes the way the wrapped value
  points (0 or above, or -32768: right; below 0: left). A shift port with a fraction (fixed point with a slope other
  than 1) is rejected as Simulink rejects it (`PORT_TYPE_INVALID`); with a uint32 shift port the run stops
  (`SIM_DIAGNOSTIC`, as Simulink's ShiftNonIntParam) at the first step whose port value is 2^31 or more. `CheckOORBitShift` `on`: with `DiagnosticForOORShift`
  `Error` the run stops (`SIM_DIAGNOSTIC`) at the first step whose shift is outside 0 to 31 (Left / Right) or -31 to
  31 (Bidirectional); with `Warning` the values are unchanged and `simulink_warnings` lists `ShiftOutOfRange` with
  that step and value. Not inside feedback loops; Python path only.
- Gain `Multiplication` `Matrix(K*u)`: `Gain` is an m x n matrix (`[a b; c d]`) and the input a vector of n, the output
  a vector of m (row i = the sum of K(i,j) * u(j)). The parameter type is the whole matrix's best precision (internal
  rule) or explicit. With the internal-rule output the products are exact and the sum takes the product type. With an
  explicit output type every product is first rounded into the output type (by `RndMeth`, then wrapped or saturated),
  and the row is accumulated in the output type, wrapping or saturating at every addition (rounding the exact sum
  once is wrong). Answered inside the recorded range: input fixdt with 4 to 32 bits and a fraction of
  0 up to the word length (signed or unsigned); an m x n matrix with m 1 to 4 and n 2 to 4; parameter type internal
  rule or one of fixdt(1,8,0) / fixdt(1,8,5) / fixdt(1,8,6) / fixdt(1,8,7) / fixdt(1,16,8) / fixdt(1,16,12) /
  fixdt(1,16,14) / fixdt(1,16,15); output internal rule, 'Same as input' or fixdt with 6 to 32 bits and a fraction of
  0 up to the word length; any `RndMeth`, saturation on or off, host or ASIC hardware. An unsigned 32-bit output type is
  answered only with an explicit parameter type, and either products whose fraction is below the output's, or products
  that need no shift into the output and are 25 to 40 bits wide with fraction 0, or 24 to 40 bits wide with fraction 4.
  Integer products into fixdt(0,32,0): when the parameter type and the input type are both built-in integers (8, 16 or
  32 bits, fraction 0) and the products have at most 24 bits, the row is summed exactly and saturated or wrapped once,
  as Simulink does; otherwise each product is cast into the output first. Answered with the parameter types
  fixdt(1,4,0), fixdt(1,6,0), fixdt(1,8,0), fixdt(1,12,0), fixdt(1,16,0) and fixdt(0,8,0), signed 4- to 32-bit or
  unsigned 4- to 16-bit integer inputs, products of at most 40 bits, any `RndMeth`. The parameter type fixdt(1,16,0)
  with the output fixdt(0,32,4) is answered for signed 16- to 24-bit inputs with fraction 4, 1 to 3 rows by 2 or 3
  columns, Floor, saturation on, host hardware. Outside it, the individually recorded settings are still answered, and five more regions are answered over their recorded values (each of input type, parameter type, output type, `RndMeth`, saturation and shape among the values recorded crossed with another axis in that region on that hardware, engine/matgain_regions.json; a value recorded only in one-axis variants is answered only in its recorded setting): products rounded into a fixdt(0,32,f) output; products whose fraction equals the output's; an internal-rule parameter into fixdt(0,32,0) (an unsigned input gives an unsigned parameter, so a negative gain is a parameter overflow, as in Simulink); integer products of other parameter / input types into fixdt(0,32,0); and matrices of 5 or more rows or columns; any other
  width, shape or parameter type is refused, naming the field that is outside. Other matrix modes and Product `Matrix(*)` (matrix signals) are refused.
- Demux `Outputs`: a number k splits the vector evenly into k outputs; a bracketed list gives the widths, so `[4]` is
  one output of width 4. A 0 width or fewer than one output is rejected as in Simulink; `-1` widths and bus selection
  mode are not supported.
- MinMax with three or more inputs: on ASIC/FPGA the exact selection, Floor-cast to the output type. On other
  hardware Simulink sometimes selects on a grid coarser than the output, or wraps an input outside the output's range;
  the same answer is given there only when every input has at
  least the output's fraction bits and a range inside the output type, and not when the first two inputs are
  unsigned and a later one is signed (Simulink then compares in an unsigned intermediate type) -- otherwise refused
  (UNSUPPORTED).
- A matrix parameter value (`[1 2; 3 4]`: more than one row and column) needs a matrix signal of the same shape
  (below); against a 1-D vector, or a matrix of another shape, it is rejected as Simulink rejects it
  (`TYPE_MISMATCH`, PortDimsMismatch; a Unit Delay's initial value: `BAD_PARAM`). Gain `Multiplication: Matrix(K*u)`
  is described above. Row (`[1 2 3]`) and column (`[1; 2; 3]`) vectors are accepted on vector signals; a Gain's row or
  column K on a scalar input gives a vector of numel(K) elements, each K(i) times the input (with `Multiplication`
  `Matrix(K*u)`, an m x 1 K).
- Sqrt: Operator `sqrt` with `AlgorithmType` Exact (the default) and an explicit fixed-point output type: the exact root
  of the input's value, rounded by RndMeth (Floor, Zero, Ceiling, Nearest; Convergent and Round give the Nearest
  answer, as the root never falls exactly half way when the input has at most twice the output's fraction bits) and always saturated to the output type.
  An input fraction length above twice the output's is answered for Floor / Zero / Ceiling (the correctly rounded
  root) and refused for Nearest (Simulink's result there is not determined). A negative input
  value fails the run at that step, as Simulink's simulation fails (fxpSqrt_negativeinputnotsupported). The internal
  output rule, Newton-Raphson (an iterative result, not the rounded root) and `rSqrt` are refused for now; `Inherit: Same as input`, `signedSqrt` on fixed point, `rSqrt` with Exact and a fixed
  rSqrt intermediate type are rejected as Simulink rejects them.
- Signum of an unsigned input with a fraction length or an unusual word (`ufixN_Ek`) gives `sfixN` (0 / 1); `uint8`,
  `uint16`, `uint32`, `uint64`-shaped inputs are refused (Simulink keeps built-in integers unsigned).
- Relational / Logical Operator output: `boolean` (default), or `fixdt(1,16,0)` (also written `fixdt(1,16)`),
  `fixdt(0,8,0)`, `fixdt(0,1,0)`, `fixdt(1,32,0)` holding 0 / 1. `Inherit: Logical (see Configuration Parameters:
  Optimization)` is refused as not supported (Simulink accepts it; its result type is not recorded).
- A Constant left at its default type is double, which the engine does not simulate -- except as one input of a
  two-input Relational Operator against fixed point of up to 40 bits: then the signal's real value is compared
  exactly with the Constant's double value (`compiled_types` shows the Constant as `double`). Inside the
  recorded floating-point shapes (see Floating-point signals below, e.g. a default Constant into a Gain) the design
  runs on the floating-point path in double. Anywhere else it is refused (`not_supported`, `where` at the Constant's
  `OutDataTypeStr` or at the first unrecorded block): set `OutDataTypeStr` to a fixed-point type (e.g.
  `fixdt(1,16,8)`) or to `Inherit: Inherit via back propagation`. The conversion of a double design to fixed point
  is fxopt's job (OPTIMIZER.md).
- A vector Inport `x` of width w: give `x[1]` .. `x[w]`; vector outputs come back as `y[1]` .. `y[w]`.
  `PortDimensions` may be left out: an Inport without it that is fed `x[1]` .. `x[w]` (w of 2 or more, no `x` itself)
  takes the width w from the data, and the reply lists it under `inferred_port_dimensions`.
- A matrix Inport `x` (`PortDimensions` `[m n]`, m and n above 1): give its m*n elements as `x[1]` .. `x[m*n]` in
  column-major order (MATLAB's `x(:)`: element (i, j) is `x[(j-1)*m + i]`); matrix outputs come back the same way, and
  matrix parameters are read column by column. Answered for the recorded settings only (`engine/matrix_domain.json`):
  Gain (element-wise, matrix gain), two-input Sum / Product (element-wise), Abs, Saturation (matrix limit), Data Type
  Conversion and Unit Delay (matrix initial value), with a matrix Constant operand where recorded, on 2x2, 2x3 and
  3x2 matrices, and a one-input Sum / Product collapsing a matrix -- `CollapseMode` `All dimensions` (one value),
  `Specified dimension` with `CollapseDim` 1 (each column: 1 x n) or 2 (each row: m x 1). On a 1-D vector a collapse
  over `CollapseDim` 2 is rejected as Simulink rejects it (`BAD_PARAM`). Every non-scalar operand must have the same
  shape; on a matrix Inport's signal a 1-D vector parameter and a scalar signal with a matrix operand are refused (not
  recorded); a double-valued matrix Constant is refused (the floating-point path has no matrices). Matrices made
  otherwise keep the vector rules above, element by element in column-major order: a matrix Constant with a
  fixed-point output (`[1 2; 3 4]` gives `y[1]` .. `y[4]` = 1 3 2 4) and a Reshape to `[m n]`.
- Multirate designs: each Inport's sequence holds its own samples, one per hit of the Inport's sample time
  (`SampleTime` on the Inport, default 1). Inports may have different lengths.

## Result

The first key after `status` is `summary`; details follow.

```json
{"status": "ok",
 "summary": {"outputs": {"y": {"type": "fixdt(1,8,4)", "steps": 4, "min": 0.625, "max": 6.25, "last": 6.25},
                         "flag": {"type": "boolean", "steps": 4, "ones": 2}},
             "blocks": 6, "engine": "python", "blocks_with_assumed_defaults": ["D", "S"],
             "blocks_with_ignored_parameters": [], "parameters_quantized": 0,
             "simulink_warnings": 0, "simulink_warnings_unchecked": 0,
             "diagnostics": {"blocks_wrapped": [], "blocks_saturated": [], "total_wraps": 0, "total_saturations": 0,
                             "blocks_divided_by_zero": [], "first_event": null},
             "hwbits": 41},
 "compiled_types": {...}, "ignored_parameters": {...}, "assumed_defaults": {...},
 "parameter_quantization": {...}, "simulink_warnings": {...}, "accumulator_types": {...}, "internal_types": {...}, "schedule": [...], "outputs": {...},
 "metrics": {"nsteps": 4, "nblocks": 6, "engine": "python"},
 "hw": {"ProdHWDeviceType": "Intel->x86-64 (Windows64)"}, "hw_default": true,
 "timing_ms": {...}, "cost": {...}, "diagnostics": {...}, "input_sha256": "..."}
```

| Key | Contents |
|---|---|
| `summary.outputs` | per recorded block: `type`, `steps` (sample count) and the real-value range (`min`, `max`, `last`; `ones` for a boolean); multirate outputs add `period` and, on an offset grid, `offset` (exact strings, the same as in `outputs`); single-rate outputs carry neither |
| `outputs` | per recorded block: `{"type", "s", "wl", "fl", "ints"}`, or `{"type": "boolean", "bool": [...]}`; multirate outputs add `t` (sample times, exact, as strings) and `period` |
| `compiled_types` | the compiled output type of every block |
| `accumulator_types` | the accumulator type of every Sum |
| `internal_types` | per Discrete FIR: `coefficient`, `product`, `accumulator`, `state` types (a one-coefficient FIR has no accumulator or state in its arithmetic: coefficient and product only) |
| `assumed_defaults` | per block, the parameters the design left out and the values used |
| `assertion_failures` | per Assertion block whose `StopWhenAssertionFail` is `off`: the steps at which its input was 0 (Simulink warns at each). With `on`, the run stops instead: error `ASSERTION_FAILED` naming the first such step. A vector input fails at a step when any element is 0, once per step |
| `ignored_parameters` | per block, accepted parameters that do not change the result |
| `parameter_quantization` | per parameter location (`design.blocks.<name>.<parameter>`), a parameter the engine does not use as written: `{"written", "type", "used", "why"}`, where `why` is "rounded to the type's precision" (Simulink warns: parameter precision loss) or "saturated to the type's bound" (up to one LSB above the maximum, or with `hw.ParameterOverflowMsg` `warning` / `none`). `summary.parameters_quantized` counts them. A list parameter is reported per element, 1-based, a scalar unindexed (`...BreakpointsForDimension1[2]`, a Discrete FIR's `Coefficients[2]` -- Lattice MA too, a Discrete Filter's or Discrete Transfer Fcn's `Numerator[1]` / `Denominator[2]`, a FIR Decimation / Interpolation's `h[3]`, a Biquad Filter's `BiQuadCoeffs[j]` for one section or `BiQuadCoeffs[r,c]`, its `ScaleValues[j]` and `IC`); filter coefficients are reported against the coefficient type actually used, written or chosen by the library rule; a Discrete-Time Integrator's InitialCondition and saturation limits are reported against its state type; a Bias value and a Tapped Delay's `vinit` (`vinit[j]` for a vector) against the input type |
| `simulink_warnings` | the warnings Simulink gives on this design, from recordings; the values are not affected. `expected`: `[{"block", "id", "when", ...}]`, one entry per block and warning: `Simulink:blocks:SwitchIgnoringThreshold` (`compile`): a Switch with a boolean control and criterion `u2 > Threshold` / `u2 >= Threshold` ignores the threshold (it acts as `u2 ~= 0`); Simulink warns unless the criterion already means that on 0 / 1 (`>` with 0 <= Threshold < 1, `>=` with 0 < Threshold <= 1). `Simulink:blocks:MPSwitchDeadPorts` (`compile`, with `dead_indices`): a Multiport Switch control type cannot represent some data port's index (a boolean into a one-based switch or into three or more ports, ufix2 into five ports...), whatever DiagnosticForDefault says. `Simulink:blocks:MPSwitchControlInputOORIndexValue` (`run`, with `first_step` and `control_value`): DiagnosticForDefault `Warning` and the control leaves the data ports' range at some step; Simulink warns once per block (its message names the value, not the step; `first_step` is the engine's). Compile-time warnings are also repeated once as the simulation starts. The rules answer only the recorded shapes: scalar Switches with thresholds -1, -0.5, 0, 0.25, 0.5, 0.75, 1, 1.5, 2; scalar zero- / one-based Multiport Switches with a boolean, ufix1 or ufix2 control (up to 6 data ports; boolean up to 4) or a uint8, int8, ufix4 or int16 control (up to 4 data ports). `unchecked`: `[{"block", "why"}]`, any other Switch with a boolean control or Multiport Switch (another threshold or control type, more ports, Specify indices, a vector, or the run-time warning on the multirate path), with the design to record. An export check that fails on any warning should expect these. `summary.simulink_warnings` / `summary.simulink_warnings_unchecked` count them |
| `schedule` | the execution order |
| `hw`, `hw_default` | the hardware used (pass it back unchanged); `hw_default: true` when none was given |
| `sample_times` | multirate designs: the period of every block; with an offset it is written as Simulink's `[period offset]` (e.g. `[3/10 1/10]`), and that output also carries `offset` |

With `"detail": "summary"` the answer holds only `summary`, `hw` and `full_result`: a file path (relative to the
tree directory, named by the request hash, so a retry finds the same file) holding the full result. Use it for long
runs.

### Diagnostics (`"diagnostics": true`)

- An event is a value leaving the type of one of a block's quantisation points (output, accumulator, product,
  cast), then wrapped or saturated as the block's overflow setting says. A Free-Running Counter's designed wrap is
  not an event.
- `diagnostics.blocks.<b>`: `wraps`, `saturations`, `divisions_by_zero`, the first step of each (steps from 0),
  `type`, and the output range seen, `min` / `max` (real values) and `stored_min` / `stored_max`.
- `summary.diagnostics.first_event` is `[step, block, "wrap" | "saturation"]`; events at the same step are ordered
  as Simulink reports them: data flow first, then, between independent branches, the branch feeding the
  lower-numbered Outport (or Sum / block input port) first.
- Counted on the C core when the design runs there (`"engine": "auto"` or `"c"`), with the same counts, first steps
  and ranges as the Python path (`"engine": "python"`); designs the C core does not take run on the Python path under
  `auto` and are refused under `c`. Outputs are identical to a run without diagnostics. Multirate designs: the same fields with times instead of steps (`first_wrap_time`,
  `first_saturation_time`, `first_division_by_zero_time`, and `first_event` as `[time, block, kind]`; times in
  seconds as exact rational strings); a CIC filter's integrator and section wraps are its design, not events; so are an HDL
  Counter's wrap-around and an HDL NCO's phase-accumulator wraps.
- Diagnostics name your blocks only. A block that is built from several internal operations (HDL RAM / FIFO / FIR,
  MATLAB Function, Bias, a Delay with a reset, a Tapped Delay) reports their events as its own; helpers of an enabled
  subsystem report under the subsystem's name. A block on a vector signal is reported per element, `G[1]`, `G[2]`,
  ... (in `compiled_types`, `accumulator_types`, `diagnostics` and `first_event` alike), as its outputs are.
- No events means none on these inputs. It is not a proof for other inputs: use fxcert or fxopt with input ranges.

### Python path only

These run on the Python path only: `"engine": "c"` is refused with `UNSUPPORTED_WIDTH` ("Python path only"), and `"auto"` (the default) and `"python"` answer them with the same results. A MATLAB Function whose body the compiler cannot express as engine blocks (engine/fxmlgen.py; straight-line code, if / for and persistent state compile and run on C); Bit Rotate; HDL NCO with dither, a reset port, a phase offset port, the phase output, the phase increment from an input port, or NumQuantizerAccumulatorBits 16 (HDL NCO runs on C with the increment as a dialog property, a quantiser under 16 bits and none of those ports); DSP HDL FIR / FIR Decimator with an explicit output type, a reset port or partly serial; deep HDL RAMs (9 to 16 address bits); Prelookup + Interpolation Using Prelookup (PreLookup / Interpolation_n-D); the CORDIC blocks; Bit Concat over 120 bits and Bit Slice over 128; Extract Bits with a field below bit 0; Multiport Switch with `Specify indices`; Delay with the initial condition from a port; Shift Arithmetic with the shift from a port; Discrete-Time Integrator with Backward Euler / Trapezoidal, a reset or an external initial condition; Switch with boolean data.

### High-frequency blocks: runnable examples

`docs/en/HIGHFREQ_EXAMPLES.md` holds one request per high-frequency block (IIR filters, Lattice MA, HDL NCO, HDL
Counter). Run a request with fxgraph to inspect its output values and types. The final three requests demonstrate unsupported configurations.

### Cost (`"cost": true`)

- `cost.hwbits`: every block's output word length (times the length of a Delay; Outports not counted) plus the word
  lengths inside it: Gain parameter, Sum accumulator, Discrete FIR coefficient x taps + product x taps +
  accumulator x (1 + states when the state type is the accumulator's), lookup breakpoints x entries + table x
  entries. fxopt's `hwbits` uses the same counting.
- `cost.by_block`: `output_bits`, `inner_bits`, `hwbits` per block.
- The count is over the blocks of your design, at their native word lengths. Blocks the engine creates when it
  lowers one of your blocks are not counted on their own. An HDL Counter is its output register plus its hit output
  (1 bit) when shown. A Bit Concat is its output word. HDL NCO / RAM / FIFO / DSP HDL FIR are counted by their own
  rules (stored words x count). Pure wiring and routing blocks (Mux, Demux, Selector, Bus Creator / Selector,
  Concatenate, Reshape) cost 0.
- `cost.hw_estimate`: predicted FPGA resources (DSP48E1, LUT, FF) and ASIC gates. These are model predictions, not
  synthesis results.
- Not available for multirate designs (fxopt's `baseline.hwbits` gives the cost there).

### Block catalogue (`{"catalogue": true}`)

Returns `{"status": "ok", "blocks": {type: {"parameters", "defaults", "required", "must_write", "values"}}}` -- the
data of [BLOCKS.md](BLOCKS.md). A misspelt block type gives `UNKNOWN_BLOCK_TYPE` with close names and every
accepted type.

## Floating-point signals (double / single)

A design whose signals include `double` or `single` (a floating-point input, a block with `OutDataTypeStr` `double` /
`single`, a Constant or Data Type Conversion at its library default, back propagation into an untyped Outport) runs on
the floating-point path -- Python only (`"engine": "c"` is refused with `UNSUPPORTED_WIDTH`), and only for the block
shapes Simulink was recorded on (Constant, Data Type Conversion, Gain, Sum, Product, Abs, Unary Minus, MinMax, Switch,
1-D / 2-D / n-D Lookup Table, Outport: the input types, output rules, rounding / saturation and hardware of `engine/float_domain.json`). Any other
shape goes to the fixed-point engine, which refuses floating-point signals.
- Input: `{"type": "double" | "single", "vals": [...]}` or `{"type": ..., "hex": [IEEE bit patterns]}`; fixed-point
  inputs as usual.
- Result: each floating-point Outport gives `{"type", "vals", "hex"}` -- `vals` is `null` for NaN / Inf, `hex` is the
  exact IEEE bit pattern (compare on it); `compiled_types` gives every block's type; `metrics.engine` is
  `python-float`.
- Rules (from the recordings): IEEE arithmetic; with any double input the block computes in double and rounds once
  into a single output, an all-single chain rounds every step; a NaN operand propagates with its sign, an invalid
  operation gives the negative default NaN, negation leaves a NaN unchanged; float -> fixed point rounds the exact
  value by `RndMeth` (`Simplest` toward zero), saturates or wraps, NaN gives 0, +-Inf the type's limit with saturation
  on and 0 with it off.
- Lookup Table on floating-point signals (1-D / 2-D / 3-D): Linear point-slope `(y1 - y0) * f + y0`, Linear
  Lagrange `(1 - f) * y0 + f * y1`, Flat; dimension 1 first; single rounds every operation; Linear extrapolation only
  when input, table and output are all floating point (otherwise refused as Simulink rejects it, `PORT_TYPE_INVALID`);
  `Nearest` takes the upper table value unless `u - bp[i] < bp[i+1] - u` (a tie and a NaN go up) and does not
  extrapolate; Linear point-slope into a fixed-point output is answered with `IntermediateResultsDataTypeStr`
  `double`, `fixdt(1,32,20)` or `fixdt(1,40,30)` (computed in the input's float type, one rounding by `RndMeth` into the
  output). With the default `Inherit: Same as output` it is answered for 1-D and 2-D tables (2 to 9 breakpoints per
  dimension), double or single inputs, a double, `fixdt(1,16,2)` or `fixdt(1,16,8)` table (a fixed-point table value
  is stored to the nearest step, a tie rounded up), an output `fixdt(1,16,2..12)`, `fixdt(0,8,4)`, `fixdt(0,12,4)` or
  `fixdt(1,32,20)`, any `RndMeth`, Clip, `UseLastTableValue` on or off, host hardware, and on ASIC / FPGA with
  `Floor`, `Nearest` or `Simplest` and an output of 8, 16 or 32 bits: each interpolation
  casts `y0` into the output type and `(y1 - y0) * f` (computed in double for a double table, in the input's type for a
  fixed-point one, the operands converted to that type first) into it as well, then adds in the output type (saturating
  or wrapping; `Simplest` casts a double table value toward zero and a fixed-point one down); when the table's type is the
  output type the product is only rounded, not range-limited, before the sum; a later dimension interpolates the earlier
  results as values of the output type, also when `UseLastTableValue` took a table value directly; with saturation off
  the stages wrap in the output's container word (8 / 16 / 32 bits) and only the result wraps into its own word
  length (so a `fixdt(0,12,4)` output carries a 16-bit value between dimensions). 3-D tables are answered only as
  recorded: double input and table into `fixdt(1,16,6)`, `Simplest`, saturation off, 3 or 5 breakpoints per
  dimension; or single input, a `fixdt(1,16,8)` table into `fixdt(1,32,20)` or `fixdt(0,8,4)`, `Floor` / `Nearest` /
  `Convergent`, 3 to 5 breakpoints per dimension, `UseLastTableValue` off, host. Other 3-D tables and other types
  are refused.
- A fixed-point 3-D n-D Lookup Table outside the recorded type combinations is answered dimension by dimension within
  the recorded span only: Flat / Nearest, or Linear point-slope with the internal-rule
  intermediate type and a fixed-point table; Clip; inputs among the recorded types; tables `fixdt(0,16,12)` /
  `fixdt(1,16,14)` / `fixdt(1,16,8)` (double tables Flat / Nearest only); 2 to 5 points per dimension; host and ASIC as
  recorded. Mixed input types need `InputSameDT` off, as in Simulink. Anything else is refused.
- A 1-D Lookup Table with `OutDataTypeStr` `Inherit: Inherit via back propagation` whose destinations are only
  untyped Outports or Data Type Conversions with an explicit `fixdt` compiles to double, as in Simulink (neither
  passes a type back): the table is stored in its own type (a tie rounded up; `Inherit: Same as output` keeps it
  double), the breakpoints in the input's type, then the double rule above. Answered on host hardware for fixed-point
  inputs of 8 to 18 bits, tables of 10 to 32 bits (or double), Flat / Nearest / Linear point-slope, Clip (Linear
  extrapolation with a fixed-point input or table is rejected as Simulink rejects it), `UseLastTableValue` off or on; a
  Data Type Conversion after it only into `fixdt(1,24,12)` or `fixdt(1,16,14)` with `Floor`, saturation off, or into
  `fixdt(0,16,8)` / `fixdt(1,32,16)` with `Floor`, `Nearest` or `Zero`, saturation off or on.
- A Sum, Product, MinMax or Switch whose operands mix a double (for example a Constant at its library default) with
  fixed-point operands of 6 to 32 bits compiles to double and computes in double, each fixed-point operand entering
  as its exact value (host and ASIC/FPGA). A Delay with such operands is refused.
- Fixed-point 1-D Lookup Table, Linear point-slope, `IntermediateResultsDataTypeStr` `Inherit: Inherit via internal
  rule`, `InternalRulePriority` `Speed` on host hardware: with a signed 16-bit table of fraction 10 to 14, or a signed
  32-bit table of fraction 0 with `UseLastTableValue` off, the intermediate type is signed 32 bits with the table's
  fraction (one bit less for the 32-bit table), for inputs of 8 to 16 bits, the outputs `Inherit: Same as first
  input`, `fixdt(1,16,8)` or `fixdt(1,20,14)`, `Floor` / `Nearest` / `Zero` / `Simplest`, saturation off, inherited
  breakpoint and fraction types; other Speed tables only for the recorded tuples with `Simplest` or `Floor`.
- Fixed-point 1-D / 2-D Lookup Table, Linear point-slope, a falling segment on an unsigned intermediate type, with the
  fraction type `fixdt(0,16,15)`, inputs of 8 to 16 bits and Clip: on host hardware with the intermediate type equal
  to the table type the slope term comes from the step's magnitude; otherwise it is cast into the intermediate type
  with the block's overflow mode (wrapped, or 0 with saturation on). Answered for the recorded combinations of
  dimensions, table-to-intermediate relation (same / narrower / wider / signed), intermediate word, hardware and
  saturation (`engine/lu_domain.json`); everything else is refused.

## Multirate designs

Downsample, Upsample, Rate Transition, FIR Decimation / Interpolation and CIC filters with integer rate ratios.
Rate options must be written out (`InputProcessing: "Elements as channels (sample based)"`, `RateOptions: "Allow
multirate processing"`; the library defaults are frame-based or single-rate and are refused). With sample-based input,
Upsample and Repeat run at P / L whichever `RateOptions` is written, as in Simulink. A Rate Transition with
`OutPortSampleTimeOpt` `Inherit` is answered under SingleTasking when it feeds Outports only (none with its own
`SampleTime`) and its input has no offset: it runs at its input's period and passes the input through. Otherwise a
Rate Transition left at the library's `OutPortSampleTime` -1 (or `OutPortSampleTimeOpt` Inherit) takes the period of
the one block it feeds when that block has an explicit `SampleTime`; for an export, write the period on the Rate
Transition instead (the export cannot read back several explicit periods in one subsystem). A Rate Transition's
`InitialCondition` beyond its type is refused when Simulink stops on it (slow-to-fast, beyond the maximum + 1 LSB) and
answered fast-to-slow, where the value is never output. Give `stop_time`
instead of `nsteps`; inputs hold one sample per hit of their Inport. Multirate runs use the Python path and offer
neither diagnostics nor cost.

- **Sample times with an offset** (`SampleTime` `[P o]`, 0 <= o < P; otherwise rejected as Simulink rejects it,
  `BAD_PARAM`): on an Inport; and, with `hw.SolverMode` SingleTasking, on the recorded blocks and relations
  (`engine/offset_domain.json`): a Unit Delay, Gain, Sum or Delay at the `[P o]` it inherits; a Unit Delay or Gain at
  `[P 0]` fed from an offset block of the same period (it reads that block's last hit, its initial value before the
  first); a Unit Delay or Outport at `[P o]` fed at the base rate (it samples its input at o + kP); a Constant at its
  own `[P o]` feeding Outports. A block at `[P o]` executes at t = o + kP. Under Auto the same blocks run at the same times,
  but a connection between different sample times without a rate-changing block is rejected as Simulink rejects it
  (`Simulink:DataTransfer:IllegalIPortRateTrans`); MultiTasking is refused. Under Auto only recorded block settings
  are answered; a Rate Transition for the recorded Integrity / Deterministic combinations with a factor of 2 to 4
  either way: slow to fast with Integrity off outputs 0 at the first fast step and then runs one fast step behind
  the SingleTasking timing (y[j] = x[(j-1) div K]), as Simulink does; the other recorded combinations time as under
  SingleTasking.
- **Decimating FIR** (`FIRDecimation`, factor D, coefficients h): the output at time k·D (input sample period 1)
  is y[k] = sum over j of h[j]·x[k·D − j], with zero initial states; the first output is at t = 0. The delay line
  keeps the input type; products, accumulator and output follow the block's types (inherited product = full
  precision widened to a native word, accumulator = product + ceil(log2(taps per phase)) widened likewise; the
  polyphase branches are summed from the last phase to the first). An interpolating FIR (factor L) outputs at
  every input-rate / L step the taps of the active phase.
- **Wrap-around accumulators** (a Sum + Unit Delay or integrator meant to wrap, CIC sections) simulate exactly;
  their wraps show up in `summary.diagnostics` (`blocks_wrapped`, `total_wraps`) like any other wrap. fxopt's range
  mode accepts an integrator-comb chain it recognises (fed from an Inport) and proves the values leaving it: the
  result is `bounded` with the chain's Sums named in `guarantees.overflow.wraps_intended_in`, and fxcert
  accepts the same proven chains. Range mode refuses every other wrap-around accumulator (see OPTIMIZER.md,
  "Integrator-comb chains" and "Designs fxopt's range mode refuses").

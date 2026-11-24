# fxsim detailed guide

fxsim is a command-line, JSON-in / JSON-out tool set that simulates discrete fixed-step fixed-point block
diagrams and chooses word lengths. Bit-for-bit comparisons with Simulink R2026b cover the recorded configurations. Some accepted
combinations fall within a tested parameter range without an individual recording. Export your design
and run its supplied checker in Simulink to verify it with your inputs.

`examples/` next to this file holds every request used here, each with `<name>.result.json` = a wrapper
`{command, exit_code, wall_s, note, stdout}` whose `stdout` is the tool's JSON output as observed
in the example runs (long lists trimmed where indicated). Numbers quoted below come
from those files; points taken from the docs without a run are marked *(documented)*. The reference docs are in
`docs/en/`; section 13 says which doc to open for depth.

## 1. What fxsim is, and what it is not

| It is | It is not |
|---|---|
| Bit-exact simulation for the recorded fixed-point and boolean configurations, single-rate and integer multirate, scalars, vectors, feedback through delays | An unrestricted floating-point simulator. The recorded floating-point shapes are listed in `docs/en/GRAPH.md`. |
| Overflow / saturation / divide-by-zero counts per block with the first step; compiled type of each block; assumed defaults | A GUI, or a continuous-time / variable-step solver |
| Word-length optimisation with error tolerance, hardware cost (hwbits, FPGA DSP/LUT, ASIC gates), overflow proof, worst-case error search; lookup-table design (fxapprox) | A code generator. `fxcodegen` / Verilog exist only as internal cross-checks: never deliver their output |
| Read `.slx`, write types back, export a fresh `.slx` + MATLAB check script | A loader for any Simulink model: unrecorded blocks / options are refused |

**Supported domains.** Some configurations are admitted by exact recorded tuples, others by ranges derived from parameter sweeps. A combination inside those ranges may have no individual recording. Read the diagnostic and the block’s documented domain when a request is refused. Never work around a refusal by hand-computing numbers.

## 2. Ground rules (read before the first call)

- **Locate the tree**: the directory that contains `engine/fxgraph` and `docs/en/AGENT_GUIDE.md`. Call it
  `$FX`. Nothing to install: Python standard library + a prebuilt C core. Only if you get `CORE_UNAVAILABLE`
  / `CORE_FAILED`, run `sh engine/build.sh` (system clang) once.
- **Run as** `python3 -I $FX/engine/<tool> < request.json`. Relative paths in a request (fxslx `path` / `out`,
  fxexport `out`, fxopt `log_dir`) are taken from `$FX` whatever the current directory, and paths in results are
  relative to `$FX`. Absolute paths work.
- **stdin only**: fxgraph, fxslx, fxcert, fxexport, fxapprox read the request from stdin; any command-line argument
  gives `BAD_PARAM` (`where: "argv"`, exit 2). fxopt also takes the request path as its one argument.
- One JSON object per request; fxgraph and fxopt also take JSONL (one request per line -> one result per line,
  exit code 1 if any line failed).
- Exit code 0 = `"status": "ok"`; 1 = `"status": "error"` with `code`, `where`, `message`, `input_sha256`,
  `refusal_kind`, `next_step` (section 6).
- Use `runs/` for JSON tool outputs. Batch scripts can open files and save flat models outside the package with the caller’s filesystem permissions. Relative binary file names resolve against the working directory.
- **Same input -> same output** (except timing / `run` fields). Retrying is safe; parallel processes share no state.
- Unknown request fields and misspelt block parameters are errors, never silently ignored.
- fxopt starts `workers` evaluation processes (default: all cores). On a shared machine pass `"workers": 2`.

## 3. The workflow

1. **Design**: write the design JSON (section 4) or read a `.slx` (section 10).
2. **Simulate** with fxgraph, `"diagnostics": true` -> read `summary` first (section 5).
3. **Fix** what diagnostics show (wraps, saturations, assumed defaults you did not intend). Set every block's
   `RndMeth` / `SaturateOnIntegerOverflow` now: fxopt never changes them.
4. **Optimise** with fxopt: inputs (+ ranges), error tolerance, optional per-block limits, cost model (section 7).
5. **Check the plan**: `warnings` (never deliver `ERROR_BOUND_VIOLATED`), `guarantees`,
   `summary.max_error_over_tolerance` (the headroom; not `binding`).
6. **Assemble the solution design**: your blocks with `solution.blocks` overlaid, your original lines (7.4).
7. **Verify** with fxcert at the user's tolerance - the default loop, not an option (section 8).
8. **Export** with fxexport -> package with `.slx`, expected I/O and `check_<name>.m` (section 9).
9. Run the replay in the exported directory. Accept only the printed `fxexport-check` JSON verdict `PASS` for this model.
   On macOS R2026b Update 1, MATLAB can exit 0 on a mismatch. Use the [native check command](SKILL.md#export-bind-and-check) for automation.
   For a design read from `.slx`, also `write_types` the solution into a copy of the user's model.

## 4. Writing a design

### 4.1 Shape (example `01_sim_iir.json`: y = x + 0.75*y[n-1])
```json
{"design": {"blocks": {
   "x": {"type": "Inport"},
   "S": {"type": "Sum", "Inputs": "++", "OutDataTypeStr": "fixdt(1,20,16)"},
   "D": {"type": "UnitDelay", "InitialCondition": "0"},
   "A": {"type": "Gain", "Gain": "0.75", "ParamDataTypeStr": "fixdt(1,16,14)", "OutDataTypeStr": "fixdt(1,20,16)"},
   "y": {"type": "Outport"}},
  "lines": [{"src": "x", "dst": "S", "dport": 1}, {"src": "A", "dst": "S", "dport": 2},
            {"src": "S", "dst": "D"}, {"src": "D", "dst": "A"}, {"src": "S", "dst": "y"}]},
 "inputs": {"x": {"s": 1, "wl": 16, "fl": 14, "ints": [0, 1627, 3190, "..."]}},
 "diagnostics": true}
```
- Block names are the keys; `type` is the Simulink BlockType spelling below; parameters use the Simulink
  dialog names and Simulink-style string values (`"0.75"`, `"[0.02 0.06 0.16]"`, `"fixdt(1,20,16)"`).
- `sport` / `dport` default to 1. Every input port must be driven exactly once (`BAD_CONNECTION` otherwise).
  A feedback loop must pass through a UnitDelay / Delay (`ALGEBRAIC_LOOP` otherwise).
- Type strings: `fixdt(s,w,f)` (w 1..4096 in fxgraph; fxopt chooses 2..62), `int8` ... `uint32`
  (DataTypeConversion), `boolean`, `Inherit: Inherit via internal rule`, `Inherit: Same as input`,
  `Inherit: Same as first input`. Explicit types are the safe default; inherited types inside a loop often give
  `TYPE_MISMATCH` (example 23).
- Hierarchy is flat: a block named `SUD/Q0_S` is block `Q0_S` inside subsystem `SUD`.

### 4.2 Inputs (`ints` are stored integers = real value x 2^fl, already quantised)
- Fixed point: `{"s": 1, "wl": 16, "fl": 14, "ints": [...]}` or `{"type": {"s":1,"wl":16,"fl":14}, "ints": [...]}`.
  `s`, `wl`, `fl`, samples must be JSON integers (`"16"` -> `BAD_TYPE`).
- Boolean: `{"type": "boolean", "ints": [0, 1, ...]}` - fxgraph and fxexport only (a Switch control, an NCO's
  valid). fxopt refuses boolean Inports (`UNSUPPORTED` at `inputs.<name>.type`, with the way round): see the NCO recipe (11.1).
- Vector port `x` of width 2: give `"x[1]"` and `"x[2]"`; outputs come back as `y[1]`, `y[2]`.
- fxopt and fxcert also accept `{"kind": "sine", "amplitude": A, "omega": w, "n": N, "type": {...}}`
  (stored value floor(A*sin(w*n)*2^fl), n = 0..N-1, `omega` in rad/sample; optional `phase`, `offset`).
- When you generate `ints` yourself for a range-based check, keep them inside the quantised range
  (floor(-0.95*2^15) = -31130 lies outside a range written as -31129/32768); fxcert flags such sets with
  `inside_input_ranges: false`, and the overflow claim does not cover them.
- fxgraph runs as many steps as the inputs are long (`nsteps` may shorten, not lengthen).

### 4.3 Block cheat sheet (the most used blocks; the full lists are in `docs/en/BLOCKS.md` / `HDL_BLOCKS.md`)
| Block `type` | Parameters used here | Notes |
|---|---|---|
| `Gain` | `Gain`, `ParamDataTypeStr`, `OutDataTypeStr`, `RndMeth`, `SaturateOnIntegerOverflow` | |
| `Sum` | `Inputs` (`"++"`, `"+-"`, `"+++--"`), `AccumDataTypeStr`, `OutDataTypeStr`, `RndMeth`, `SaturateOnIntegerOverflow` | `-` ports subtract: an IIR's `- a1*y[n-1]` can be a Gain `a1` into a `-` port |
| `Product` | `Inputs` (`"**"`, `"*/"`), `OutDataTypeStr` | divide by zero saturates and is counted (`blocks_divided_by_zero`) |
| `UnitDelay` / `Delay` | `InitialCondition` / `DelayLength`, `InitialCondition` | |
| `DiscreteFir` | `Coefficients`, `CoefDataTypeStr`, `ProductDataTypeStr`, `AccumDataTypeStr`, `OutDataTypeStr`, `RndMeth`, `SaturateOnIntegerOverflow` | omitted output type = **Same as input** |
| `DataTypeConversion` | `OutDataTypeStr` (write it explicitly), `RndMeth`, `SaturateOnIntegerOverflow`, `ConvertRealWorld` | omitted type -> the library default 'Inherit: Inherit via back propagation', refused (`UNSUPPORTED`) where the destination gives no fixed-point type, e.g. an untyped Outport |
| `Saturation` | `UpperLimit`, `LowerLimit` (required), `OutDataTypeStr`, `RndMeth` | converts the input to the output type first (rounded by `RndMeth`; beyond that type's range it is limited, never wrapped), then limits in that type: a value just above the upper limit can round down to it. Both limits must fit the output type (else refused: Simulink stops, fxpParameterOverflow). **No `SaturateOnIntegerOverflow` parameter** (`BAD_PARAM`): it always limits |
| `Constant` | `Value`, `OutDataTypeStr` | for a fixed-point Constant, give its intended output type explicitly |
| `Switch` | `Criteria` (`"u2 ~= 0"`, `"u2 >= Threshold"`), `Threshold`, `OutDataTypeStr` | port 1 = data if true, 2 = control, 3 = data if false; omitted Criteria = `u2 > Threshold` |
| `RelationalOperator` | `Operator` | omitted = **`<=`** (Simulink library default) |
| `Lookup1D` | `BreakpointsForDimension1`, `Table`, `BreakpointsForDimension1DataTypeStr`, `TableDataTypeStr`, `OutDataTypeStr`, `InterpMethod` (`"Linear point-slope"`), `ExtrapMethod` (`"Clip"`) | write `ExtrapMethod: "Clip"`: omitted = Linear = refused; breakpoints must fit their type |
| `DiscreteIntegrator` | `gainval` (power of 2), `OutDataTypeStr` (explicit fixdt accepted) | |
| `Downsample` | `DownsamplingFactor`, `SampleOffset`, `InitialConditions`, `InputProcessing: "Elements as channels (sample based)"`, `RateOptions: "Allow multirate processing"` | the last two must be written |
| `DiscreteFilter` / `DiscreteTransferFcn` | `Numerator`, `Denominator` (a0 = 1 or 2), `FilterStructure` (`"Direct form II"` default, `"Direct form I"`, `"Direct form II transposed"`), internal `*DataTypeStr` (explicit or left to the library), `OutDataTypeStr`, `RndMeth`, `SaturateOnIntegerOverflow` | Discrete Filter coefficients are powers of z^-1, Transfer Fcn of z (shorter numerator aligned right); Direct form I / II transposed need a recorded explicit type set (or, on fixdt(1,8,6) / (1,10,8) / (1,12,10) inputs, both products fixdt(1,24,18)); recorded shapes in recordings, rules in GRAPH.md |
| `BiquadFilter` | `BiQuadCoeffs` (`"[b0 b1 b2 1 a1 a2; ...]"`), `ScaleValues`, `IIRFiltStruct`, `outputMode: "Binary point scaling"` + `outputWordLength` / `outputFracLength` | all other types stay at the library's (= the input type: narrow; scale the input accordingly) |
| `ComplexToMagnitudeAngle` / `MagnitudeAngleToComplex` | `ApproximationMethod: "CORDIC"`, `NumberOfIterations` (3 / 6 / 11), `ScaleReciprocalGainFactor` | inputs sfix8_En5 / sfix12_En9 / sfix16_En13 only (recorded); magnitude / re / im (1, w+2, f), angle (1, w, w-3) |
| `TriggerPort` / `ResetPort` (in `S/Trigger` / `S/Reset`) | `TriggerType` / `ResetTriggerType`: rising, falling, either | an int8 control line into it; inside `S` only Sum / Gain / Unit Delay; triggered outputs are `EnabledOutport` (held); rules in GRAPH.md |
| `ZeroOrderHold` | `SampleTime` (-1 / 1: pass-through; integer k: output every k steps) | input from an Inport at the base step |
| `FIRDecimation` | `h`, `D`, `FilterSource: "Dialog parameters"`, `framing: "Allow multirate processing"`, `InputProcessing: "Elements as channels (sample based)"` | internals are not optimised by fxopt |
| `HDLNCO` | `PhaseIncrementSource: "Property"`, `PhaseIncrement`, `Waveform`, `AccumulatorWL`, `NumQuantizerAccumulatorBits`, `DitherSource`, `OutputWL`, `OutputFL` | input = boolean valid; outputs data..., valid (last port); output fixdt(1, OutputWL, OutputFL), WL 8..32 (default 16, 14); latency 6; fxopt refuses designs containing it (11.1) |

Other supported types (Abs, MinMax, Signum, shifts, logic, HDL counters, synchronous delays, bit ops, RAM, FIFO,
buses, For Each, complex, CIC, Rate Transition, Upsample, Repeat, MATLAB Function subset; Switch Case action
subsystems into a Merge, and Data Store read / write -- GRAPH.md "Blocks and lines"): `docs/en/BLOCKS.md`,
`docs/en/HDL_BLOCKS.md`, `docs/en/GRAPH.md`, `docs/en/MATLAB_FUNCTION.md`. A Constant may hold a vector or matrix
(`"[1 2; 3 4]"`: elements in column order, as MATLAB stores them). The If block is refused (Simulink takes no
fixed-point If input): branch with a Switch Case on an integer. Unknown types give `UNKNOWN_BLOCK_TYPE`;
misspelt parameters give `BAD_PARAM` naming the parameter.

### 4.4 Omitted parameters mean the Simulink *library* default
A JSON design that leaves a parameter out gets the value a fresh block from the R2026b library has.
fxgraph and fxexport responses list them in `assumed_defaults` (`{block: {param: value}}`; fxgraph's summary
names the blocks in `blocks_with_assumed_defaults`; fxopt does not list them). Read them. Surprises seen on
this tree: `RelationalOperator` `<=`, `DiscreteFir` output = input type, `Lookup1D` `RndMeth: Simplest` and `ExtrapMethod: Linear`, arithmetic
`RndMeth: Floor` + `SaturateOnIntegerOverflow: off`, multirate blocks `InputProcessing` = frame based
(refused; the message says "write InputProcessing out"). Writing parameters explicitly is cheaper than debugging.
`summary.parameters_quantized` counts parameters (gains, table values) that do not fit their type exactly
(`parameter_quantization` gives each one): check it when a coefficient type is narrow.

### 4.5 Target hardware (`hw`, same field for every tool)
- Omit it, or `{}`: host `Intel->x86-64 (Windows64)`; the response echoes `hw` and `hw_default: true`.
- FPGA / ASIC target: `"hw": {"ProdHWDeviceType": "ASIC/FPGA"}`. Inherited types differ (observed: the same
  inherited Sum is `fixdt(1,32,30)` on host and `fixdt(1,35,32)` on ASIC/FPGA). If the user says FPGA/ASIC,
  pass it to **every** call: fxexport silently exports a host model if you forget it.
- Anything else (ARM, other int sizes) -> `UNSUPPORTED` at `hw.ProdHWDeviceType` (example 24).

## 5. fxgraph: simulate

Request fields: `design`, `inputs`, optional `hw`, `nsteps`, `outputs` (`"outports"` default, `"all"`, or a list
of block names to record internal signals), `diagnostics`, `cost`, `detail` (`"summary"` | `"full"`),
`engine` (`auto` default; `c` / `python`), `stop_time` (multirate only).

**Read in layers.** The first key after `status` is `summary`:
```json
"summary": {"outputs": {"y": {"type": "fixdt(1,16,15)", "steps": 48, "min": -0.9759, "max": 0.9623, "last": 0.3790}},
            "blocks": 5, "engine": "python", "blocks_with_assumed_defaults": ["A", "S", "y"],
            "blocks_with_ignored_parameters": [], "parameters_quantized": 0,
            "diagnostics": {"blocks_wrapped": ["S"], "blocks_saturated": [], "total_wraps": 6,
                            "total_saturations": 0, "blocks_divided_by_zero": [], "first_event": [6, "S", "wrap"]}}
```
(example `02_overflow_summary.json`). With `"detail": "summary"` the response is only `summary`, `hw` and
`full_result` - a file path (relative to `$FX`, named by the input hash, retry-safe) holding the full result.
Use summary mode for long runs; open the file only for what you need.

Full result keys: `compiled_types` (every block), `compiled_port_types`, `internal_types`, `accumulator_types`
(Sum blocks), `assumed_defaults`, `ignored_parameters` (accepted but irrelevant parameters - check them),
`schedule`, `outputs` (`{"type","s","wl","fl","ints"}` or `{"type": "boolean", "bool": [...]}`), `metrics`, `hw`,
`cost`, `diagnostics`.

**Diagnostics** (`"diagnostics": true`). Read `summary.engine` for the execution path used.
- An *event* is a value leaving the type of one of the block's quantisation points (output, accumulator,
  product, cast) at a step, then wrapped or saturated as the block says. A block can count several per step.
- `summary.diagnostics.first_event = [step, block, "wrap"|"saturation"]`, steps counted from 0.
- `diagnostics.blocks.<b>`: `wraps`, `saturations`, `divisions_by_zero`, `first_*_step`, `type`, output
  `min` / `max` (real values) and `stored_min` / `stored_max`. Use min/max to size integer bits.
- Example 02: Sum output `fixdt(1,16,15)` on a signal that peaks at 1.68 -> 6 wraps, first at step 6.
  With `SaturateOnIntegerOverflow: "on"` the same design reports 22 saturations, first at step 6.
- Multirate designs *(documented)*: the same fields with times instead of steps (`first_event` = `[time, block, kind]`).
- Some wraps are the design (an NCO's or CIC's phase / integrator accumulators wrap by construction): read
  which block wrapped before treating it as a fault.
- Zero events means "none on these inputs". It is not a proof for other inputs; use fxcert / fxopt ranges.

**Cost** (`"cost": true`, single-rate only): `cost.hwbits` (the same counting as fxopt), `cost.by_block`
(`output_bits`, `inner_bits`, `hwbits`), `cost.hw_estimate` (DSP48E1, LUT, FF, gates: **predicted, not
synthesised**). The Inport counts too (example 01: 124 = x 16 + A 36 + S 52 + D 20).

Multirate designs (Downsample, FIRDecimation, ...): inputs are samples at each Inport's own period
(Inport `SampleTime`, default 1); outputs carry `t` (exact sample times as strings) and `period`; `stop_time`
replaces `nsteps`. Example 11.

## 6. Errors and refusals

Every error of every tool: `{"status": "error", "code", "where", "message", "input_sha256", "refusal_kind",
"next_step"}`, also appended to `runs/*_errors.jsonl`. `where` is a JSON path such as `design.blocks.A.RndMet`
or `inputs.x.wl`. Act on `refusal_kind`:

- `request` - your request is wrong (or asks for the impossible) at `where`: fix it and retry (ex. 20-22, 26).
- `simulink_rejects` - Simulink would also reject or stop this (ex. 23 `TYPE_MISMATCH`): change the types /
  inputs the message names.
- `not_supported` - not verified bit-exact, so refused (ex. 24, 25, 27): use the supported form the message
  names; if impossible, report `where` + `message` to the user as a limitation.
- `defect` - an fxsim bug: use no number from that call; report `input_sha256` + the request. After
  `CORE_FAILED`, retry with `"engine": "python"`.

The code list per kind is in `docs/en/README.md` ("Errors"). `next_step` is generic per kind; the specific advice
is in `message`. For fxopt's `INFEASIBLE` / `NO_SOLUTION` read the whole `message`: it gives error / tolerance per
input set at the widest plan and the ways out, cheapest first (example 26: "allowing wider word lengths is the
first thing to try"); `NO_SOLUTION` also says to keep `ranges_from_inputs`. `TIMEOUT` / `OOM`: retry smaller.

## 7. fxopt: choose word lengths

### 7.1 Request (example `03_opt_fir.json`, 8-tap FIR)
```json
{"design": {...},
 "inputs": {"x": {"kind": "sine", "amplitude": 0.9, "omega": 0.05, "n": 2048, "type": {"s":1,"wl":16,"fl":14}}},
 "input_ranges": {"x": {"type": {"s":1,"wl":16,"fl":14}, "min": -0.9000244140625, "max": 0.9}},
 "error": {"max_abs": 0.0009765625},
 "wordlengths": [4, 6, 8, 10, 12, 14, 16, 18, 20, 24, 28, 32],
 "block_wordlengths": {"F": [2, 16]},
 "time_budget_s": 20, "workers": 2}
```
| Field | Meaning |
|---|---|
| `inputs` / `input_sets` | one set / a list of **1..64** sets (more -> `BAD_PARAM`); may be given together (all are judged; every set, every port must meet the tolerance). At least one of `inputs`, `input_sets`, `input_ranges` is required. Keys = Inport names. An `input_sets` entry is normally `{Inport: definition}`; it may also be the wrapper `{"inputs": {...}, "tolerance_scale": k}` (that set judged against k x tol); `{"inputs": {...}}` without `tolerance_scale` is the same wrapper at 1 x - except in a design with an Inport named `inputs`, where it is read as that Inport's data. |
| `input_ranges` | "any sequence within [min, max]": enables the overflow proof, the generated stress sets (`range: ...`) and the worst-case error search. Quantised as ceil(min*2^fl)..floor(max*2^fl) (0.9 became 0.89996337890625). Omitted -> the envelope of the given inputs is used (`ranges.source`). |
| `stress_tolerance_scale` | k >= 1: the range stress sets (`range: ...`) and the error search are judged against k x tol; your `inputs` / `input_sets` stay at 1 x. For "within 2^-12 on the signal, 16 x that at full scale". `guarantees.contract.scaled_input_sets` lists the scaled sets. **Never set `ranges_from_inputs: false` to escape a stress miss**: it drops the overflow proof. |
| `error` | `{"max_abs": tol}` for all Outports, or `{"outports": {"y": tol}}` for listed ports only. Default 2^-10 *(documented)*. If the request omits a tolerance, start with about one output LSB, for example 2^-10. |
| `reference` | default `"ideal"` = double simulation of the same graph; `{"kind": "design"}` = the given fixed-point design. |
| `wordlengths` | allowed word lengths for every decision variable (default 6, 8, ..., 32; each 2..62). |
| `block_wordlengths` | `{block: [min, max]}` per-block cap, e.g. "output <= 16 bits". Prefer it to `fixed_blocks` (pinned blocks are not optimised). |
| `cost` | `{"hwbits": 1}` (default), `{"hwbits": a, "mult_area": b}`, `{"fpga_xc7": 1}` (DSP48E1 count first, then LUT+FF), `{"asic_gates": 1}` - mutually exclusive; the last two are predictions. |
| `hw` | as in 4.5; pass `"ASIC/FPGA"` for FPGA work. |
| `input_variants` | default `["negated", "reversed"]`: every given set is also judged negated and time-reversed. `[]` turns it off. |
| `time_budget_s`, `time_limit_s` | budget for optimisation passes and a whole-request limit checked between stages (section 7.5). |
| `workers` | concurrent evaluation processes (default all cores). |
| `error_search` | default on in range mode; `false` skips the worst-case search *(documented)*. |

**Limits** (all *documented* in OPTIMIZER.md, the first two observed): word lengths 2..62 (`[8, 64]` ->
`BAD_PARAM "2..62"`); fxopt never changes `RndMeth` / `SaturateOnIntegerOverflow` (set them first); on ASIC/FPGA a
Lookup input may be at most 7 fraction bits finer than its breakpoints; fxopt refuses boolean Inports and any design
containing an `HDLNCO` (11.1) or a Discrete Filter / Transfer Fcn / Biquad / Zero-Order Hold (`UNSUPPORTED`;
run those in fxgraph with the recorded types); fxcert refuses multirate designs (example 27).

Decision variables: output / parameter / accumulator / product / coefficient types of Gain, Sum, Product,
DataTypeConversion, MinMax, Switch, Constant, Saturation, Abs, DiscreteFir, Lookup1D (breakpoint, table,
intermediate, fraction, output). Delays inherit. A design with nothing to choose (e.g. only an FIRDecimation,
whose internals are not variables) gives `UNSUPPORTED "no decision variables"`.

**Float -> fixed** (example 09): write the types you want chosen as `"double"` (`OutDataTypeStr`,
`ParamDataTypeStr`, `AccumDataTypeStr`), give the Inports fixed-point `type`s (the ADC format). The reference
is the double simulation of the same graph. `baseline.hwbits` is then null with reason `UNSUPPORTED: data type 'double' ...`.

### 7.2 Read the result in this order
1. `summary` - `hwbits`, `baseline_hwbits`, `max_abs_diff`, `tolerance`, **`max_error_over_tolerance`** (+
   `max_error_over_tolerance_from`), `overflow`, `error`, `warnings` (codes), `modular_blocks`, `binding`,
   `blocks_retyped`. Example 03: hwbits 366 vs baseline 688, max_abs_diff 0.000829 <= 0.000977,
   `max_error_over_tolerance` 0.99625 ("error search on the final plan"), overflow `bounded`, error `sampled`,
   warnings `["ERROR_BOUND_SAMPLED"]`.
2. `warnings` (top level, with messages):
   - `ERROR_BOUND_VIOLATED` - the error search found an in-range input over the tolerance: **not a valid answer**.
   - `ERROR_BOUND_SAMPLED` - the tolerance was checked only on the judged sets (normal; state it when delivering).
   - `OVERFLOW_NOT_PROVEN` / `RANGE_MODE_FAILED` (ex. 15) / `OVERFLOW_POSSIBLE` - no overflow proof;
     `BUDGET_EXHAUSTED` - search cut by time (the last two *documented*).
3. `guarantees.overflow.status`: `bounded` (proved for every sequence inside the ranges, every step, from the
   design's initial state), `sampled` (no ranges / proof abandoned: only the listed sequences),
   `unknown` (proof failed; not "will overflow"), `violated`, `unsupported` (the last three *documented*).
   An overflow `violated` comes with a `witness` (set, block, kind, count, first step) and `observed_overflows`:
   replay that set in fxgraph with diagnostics, fix the type, and never deliver the plan.
   `guarantees.error.status`: `exhaustive` (every stored input of a one-input memoryless design over its range: a
   real bound; example 14), `sampled`, or `violated`. `guarantees.contract` states the exact claim.
4. **Headroom = `summary.max_error_over_tolerance`**: the larger of the judged sets and the error search's last
   round (`_from` says which). `binding` is only the judged set with the largest ratio (`source: "error_search"`
   when a search input exceeds the tolerance). Example 03: `binding` 0.849 on `range: random uniform`, but the
   search on the final plan reached 0.99625 - headroom 0.4 %, not 15 %. `error_by_input_set` gives every set x
   outport with its own tolerance and scale. Each `error_search.rounds[k]` carries `worst_input` (fxgraph inputs
   form, also when within the tolerance): replay the last one with fxcert, or add it to `input_sets`. Above 0.9:
   rerun with tol x 0.9 or add sets, then fxcert (section 8; OPTIMIZER.md "Closing a sampled error bound").
5. `solution.blocks` (types to apply), `solution.compiled_types`, `hwbits_by_block`, `hwbits_breakdown`,
   `hw_estimate`, `outports.<port>` (max error, where, type), `ranges`, `error_search`, `search`, `time`.

### 7.3 Counterexample input
When the error search finds an in-range input over the tolerance and cannot absorb it, the plan is reported
with `error: "violated"`, `ERROR_BOUND_VIOLATED`, and `guarantees.error.violation.counterexample_input`
(fxgraph `inputs` format). Example 12 (example 03 with `time_limit_s` 2.5 so no re-optimisation fits):
hwbits 350, `max_error_over_tolerance` 1.015, 31-sample counterexample. Replaying it through fxcert unchanged
(example 12b) confirms 0.000988 > 0.000977. To fix: rerun with more time, or add the counterexample to
`input_sets` (accepted as is) - the full example 03 run absorbed it and costs 366.

### 7.4 Assemble the solution design
Take your `design.blocks`, overlay each block's keys from `solution.blocks`, keep your `lines`. `solution.blocks`
lists every block (example 13 includes the Downsample `DS`), but overlaying keeps any parameter you wrote that the
plan does not mention. Simulate the assembled design once with fxgraph (`cost` must equal `hwbits`; example 03:
366 = 366).

### 7.5 Time
`time_budget_s` bounds only the optimisation passes; the uniform start, overflow proof/repair,
independent re-verification and each error-search round always run to completion, so wall time can be
several times the budget. The engine checks `time_limit_s` (>= budget) between stages. Rounds predicted to overrun are
not started (`error_search.rounds[-1].stopped` says so) - which can leave `ERROR_BOUND_VIOLATED` (example 12).
`budget_exhausted: true` or `deterministic: false` indicates a run that may differ on retry. Check feasibility and allow more
time if needed. Progress of a long run: `runs/fxopt/<sha>/<attempt>/state.json` (`state`,
`elapsed_s`); the answer is only stdout or `result.json` after `state: "done"` (never `pass_result.json`).
Wall times of the examples are in their `.result.json` (`wall_s`).

### 7.6 Range proofs have limits
Read `guarantees.overflow.status`; do not predict it from the shape of the design. Linear loops are often
proved `bounded` (IIR biquads, example 07) but a linear loop can stay `unknown` when the chosen types leave the proof open, and a non-linear loop whose state is held by a
`Saturation` can be proved `bounded` (seen: a saturated AGC). When the proof fails (AGC loop with
Product/Abs, example 15) fxopt falls back to the given sequences (`ranges.source:
"none (sampled sequences only)"`, `envelope_attempt_failed` says why) and warns `OVERFLOW_NOT_PROVEN` and
`RANGE_MODE_FAILED`. Then add more `input_sets` (other amplitudes, steps), re-simulate with diagnostics, and say
so when delivering. For one-input memoryless designs the error search is exhaustive over the range (example 14:
`error: "exhaustive"`, no `ERROR_BOUND_SAMPLED`).

## 8. fxcert: verify a given design (the default check after fxopt)
```json
{"design": <solution design>, "input_ranges": {"x": {"type": {"s":1,"wl":16,"fl":14}, "min": -0.9000244140625, "max": 0.9}},
 "input_sets": [{"x": {"ints": [...]}}, {"x": {"ints": [...]}}], "error": {"max_abs": 0.0009765625}}
```
- **The loop**: run fxcert at the user's tolerance on the fxopt input sets plus a long random in-range set; if a
  set fails, add it to fxopt's `input_sets` and rerun fxopt until both agree. Multirate designs (decimation)
  cannot be certified (`UNSUPPORTED`, example 27). Cheap independent check when every `Downsample` feeds an
  Outport directly: copy the solution design, delete the `Downsample` and connect its source to that Outport,
  and run fxcert on the copy (seconds). A Downsample does not change values, so the copy's output contains every
  decimated sample: `bounded` and a passing error on the copy cover the decimated design (conservatively - the
  copy is also judged on the samples the Downsample drops). It does not apply when blocks run after the
  Downsample or to `FIRDecimation`; there rely on fxopt's own checks and the Simulink export check.
- **When to stop verifying** (deliver at this point; more rounds rarely change the answer): fxopt has no
  `ERROR_BOUND_VIOLATED` and overflow is not `violated`; `summary.max_error_over_tolerance` <= 0.9, or <= 1
  with one fxcert pass (user tolerance, fxopt's sets + one long random in-range set) where no set fails; the
  assembled design's fxgraph `cost` equals `hwbits`; the export is written. Rerun fxopt only when one of these
  fails or the user asked for a lower cost - not to re-confirm a passing plan. Two tighten rounds (tol x 0.9,
  then add the failing set) are normally enough; if the third still fails, deliver nothing and say what fails.
- Result: `overflow.status` (`bounded` / `unknown` / `violated` / `unsupported`, with `slot_bounds` and the
  claim) and `error.status` (`sampled` / `violated` / `not_checked`) with per-set results under
  `error.input_sets[k].outports.<port>` (`max_abs_diff`, `at_sample`, `within`) and `inside_input_ranges`. They are
  separate: one never implies the other. Example 04: bounded + sampled.
- `input_sets` entries may be flat `{"s","wl","fl","ints"}`, `{"type", "ints"}`, `{"ints"}` (type from
  `input_ranges`) or a sine generator; fxopt's `counterexample_input` replays unchanged (example 12b). An entry
  `{"inputs": {...}, "tolerance_scale": k}` judges that set against k x tol, as in fxopt.
- Use the tolerance the user asked for (fxopt may have optimised against a tighter internal margin).
- **Intended saturation vs "error above k x tol counts as overflow"**: fxopt / fxcert count a type that
  saturates (`SaturateOnIntegerOverflow: on` reached) as overflow (`violated`, with a witness), while a
  `Saturation` block clipping at its limits is not overflow and the ideal reference clips too (error 0 there).
  So when the requirement wants clipping, put it in a `Saturation` block with the required limits and keep the
  types wide enough never to saturate; when the requirement says an error above k x tol is an overflow, check
  with `stress_tolerance_scale: k` and deliver only with overflow not `violated`. If the request asks for both
  in a way that conflicts, deliver the version that meets the stated error rule and say which clipping it drops.

## 9. fxexport: Simulink model + one-command MATLAB check
```json
{"design": <solution design>, "inputs": {"x": {"s":1,"wl":16,"fl":14,"ints": [...]}}, "hw": {...},
 "name": "fir8_opt", "out": "runs/export/fir8_opt"}
```
- `name` *(documented)*: starts with a letter; letters, digits, `_`; <= 50 chars. `out`: a **new** directory under `$FX`
  (`OUT_EXISTS` otherwise; nothing half-written). `observe: ["Q0_S"]` adds internal signals as extra root
  outputs (`Q0_S_obs`); *(documented)* required when the design has no Outport; multirate takes `stop_time`.
- Before writing, fxexport reads the model back with its own reader and re-simulates it; any difference ->
  `EXPORT_ROUNDTRIP_MISMATCH` (defect, no package). Success shows `roundtrip.outputs_equal`.
- Package (example 05, `05_export_fir.package.txt`): `fir8_opt.slx` (From Workspace -> subsystem `DUT` ->
  root Outports, FixedStepDiscrete, stop time = last sample), `fir8_opt_io.json` (inputs and expected outputs
  as bit strings, compiled types, block parameters, settings), `check_fir8_opt.m`, `manifest.json`
  (includes `assumed_defaults` = values written for omitted parameters, `hw_profile`: `host` or `asic`).
- **Running the check** (MATLAB R2026b with Simulink and the toolboxes the blocks need - Fixed-Point Designer,
  DSP HDL Toolbox for HDL FIR/NCO blocks): copy the whole directory if MATLAB runs elsewhere, then run
  `matlab -batch "check_<name>;"` there, or use `check_command` in an open MATLAB session with the current export path.
  Both `check_batch_command` and `check_batch_command_here` can exit 0 on FAIL. A missing, malformed or duplicate
  checker record leaves the export unverified. Read the top-level `verdict` from the single printed JSON record:
  `PASS` (every sample, compiled type, setting and parameter matched), `INCOMPLETE` (all samples matched but some
  compiled type could not be read), or `FAIL` with `reason` (`VALUE_MISMATCH`, `COMPILED_TYPE_MISMATCH`,
  `PARAM_MISMATCH`, `CONFIG_MISMATCH`, `COMPILE_WARNING`, `IO_INCOMPLETE`, `MATLAB_ERROR`, ...) and `first_diff`.
- The package verifies the exported model, not a user's own `.slx` with types written back (section 10).
- `UNSUPPORTED_EXPORT`: the design runs in fxsim but that part cannot be exported yet; keep it verified in fxsim
  only and say so.

## 10. Working from an existing `.slx`
Export example 05 to generate an eight-tap FIR model. Examples 06 list that model and read its DUT subsystem. Example 08 changes the FIR output type in a separate file. The [agent guide](../../docs/en/AGENT_GUIDE.md) gives the commands and the readback check.

For an existing model, list its subsystems with fxslx and read the relevant scope. Pass the returned design, target hardware and your input sequences to fxopt. Use the selected data types from its solution to write a copy. Inspect the written and skipped parameters, then read and verify the copy with your inputs. Example 07 is a separate, self-contained optimisation of a two-biquad design supplied as JSON. Example 07b exports that design with an observation port.

## 11. Daily task recipes (all run on this tree)
| Task | How | Example |
|---|---|---|
| FIR / filter chain | `DiscreteFir`, or Gain + Sum + UnitDelay; fxopt with sine inputs + `input_ranges` | 03 |
| IIR / biquads | explicit types inside loops (or let fxopt choose); linear loops are often proved `bounded`, not always (7.6): read `guarantees.overflow.status` | 01, 07 |
| Decimation | `DiscreteFir` + `Downsample` (fxopt optimises the FIR; `FIRDecimation` alone has no variables) | 13, 13b |
| NCO mixing | 11.1: NCO in fxgraph -> mixer alone in fxopt -> overlay -> fxgraph + fxexport the whole design | 16a, 16b, 16c |
| AGC / non-linear loop | works; the overflow proof may fail (example 15: Product/Abs loop -> `sampled`) or hold (a loop held by a `Saturation` can be `bounded`, 7.6): read `guarantees.overflow.status`, add input sets when it is `sampled` | 15 |
| Limiter + LUT | `Saturation` -> `Lookup1D` (Clip); fxopt chooses breakpoint / table / fraction types | 10, 14 |
| Function lookup table (sqrt, sin, tanh, ...) | `python3 -I engine/fxapprox` -> a Lookup1D `block` to place in the design; designed for the host. Check it with fxgraph on the target `hw` (OPTIMIZER.md "fxapprox"); on ASIC/FPGA the input may be at most 7 fraction bits finer than the breakpoints, which fxapprox does not enforce | *(documented)* |
| Float design -> fixed | `"double"` types + fixed-point Inport types, fxopt | 09 |
| FPGA target / cost | `"hw": {"ProdHWDeviceType": "ASIC/FPGA"}`, `"cost": {"fpga_xc7": 1}`; export with the same `hw` | 09, 09b |
| "Output <= 16 bits" | `block_wordlengths` | 03 |
| "Looser error at full scale" | `stress_tolerance_scale` (7.1) | - |
| "How much does this hand change cost?" | fxgraph `"cost": true` on both versions | 01 |

### 11.1 NCO mixer (fxopt refuses a design containing `HDLNCO`, and boolean Inports)
1. Simulate the whole design (valid Inport -> `HDLNCO` -> `Product` with the signal) with fxgraph and record the
   NCO output: `"outputs": ["N", "y"]` (example 16a: sfix16_En14 sine, first sample after the 6-step latency).
2. Optimise the mixer alone: a design with two fixed-point Inports (`x`, and `c` = the NCO's output `ints` in
   its type) into the `Product`, `"OutDataTypeStr": "double"` for what fxopt should choose, `input_ranges` for both
   (example 16b: Product `fixdt(1,12,11)`, max_error_over_tolerance 0.50, overflow `bounded`).
3. Overlay the plan's types onto the blocks of the same name in the whole design, then fxgraph it and export it
   with fxexport (example 16c). The plan's claims cover the NCO samples you fed in, so feed a long enough run
   (several periods of both signals).

## 12. Common mistakes
- Trusting zero diagnostics events as a proof: they cover only the simulated inputs.
- Delivering a plan with `ERROR_BOUND_VIOLATED`, or claiming the error bound holds for all inputs when
  `error` is `sampled` (it is a claim about the judged sequences; overflow `bounded` and error `exhaustive` are
  the proved parts).
- Reading `binding` as the headroom: use `summary.max_error_over_tolerance`.
- Switching `ranges_from_inputs` off to pass the stress sets: use `stress_tolerance_scale`.
- Writing `input_ranges` as the literal amplitude: ranges are quantised (floor(-0.9*2^14) = -14746 is
  -0.9000244140625, so a literal -0.9 misses one stored value). Use the quantised min/max or let fxopt
  take the envelope of the inputs.
- Forgetting `hw` in one of the calls (types and the exported hardware profile change silently).
- Leaving parameters out and getting a library default (section 4.4); not reading `assumed_defaults`.
- Expecting fxopt to pick the rounding or overflow mode: it keeps the design's.
- Using `solution.blocks` as the whole design instead of overlaying it (section 7.4).
- Re-using an export `out` directory (`OUT_EXISTS`) - pick a new one each time.
- Treating either optimiser limit as an exact wall-clock deadline. A stage already in progress runs to completion.
- Starting several fxopt runs with default `workers` on a shared machine.
- Using fxcodegen / Verilog output as a deliverable (out of product scope), or hand-editing numbers after a
  refusal instead of changing the design.
- Assuming the MATLAB check passed because fxexport succeeded: only `verdict: "PASS"` from MATLAB is the
  bit-exact confirmation; until then report "verified in fxsim, Simulink check pending".

## 13. Where to read more (in `$FX/docs/en/`, English)
- `README.md` and `AGENT_GUIDE.md` - the overview; README lists every error code by `refusal_kind`.
- `GRAPH.md` - fxgraph request/response, multirate, diagnostics, cost, layered results; `BLOCKS.md` and
  `HDL_BLOCKS.md` - the supported blocks and their recorded parameters.
- `OPTIMIZER.md` - fxopt, fxcert and fxapprox: error tolerance, stress scale, input ranges, hardware cost,
  multirate, error search, closing a sampled bound, limits.
- `EXPORT.md` - package contents, check-script verdicts and reasons, export rules.
- `SLX.md` - `.slx` reading / writing rules and limits; `MATLAB_FUNCTION.md` - the MATLAB Function subset.
- `ENGINE.md` - the bit-exact arithmetic rules (rounding, inheritance, lookup tables, multirate timing).

## 14. Example index (`examples/`: `python3 -I $FX/engine/<tool> < <skill dir>/examples/<file>`)
| File | Tool | Shows |
|---|---|---|
| 01_sim_iir | fxgraph | design shape, diagnostics + cost on a clean run |
| 02_overflow_summary | fxgraph | `detail: summary`, wraps and `first_event` |
| 03_opt_fir (+ `.solution_design`) | fxopt | ranges, `block_wordlengths`, binding vs `max_error_over_tolerance` |
| 04_cert_fir | fxcert | overflow `bounded` + error `sampled` |
| 05_export_fir (+ `.package.txt`) | fxexport | package contents, manifest, `check_batch_command` |
| 06_slx_list / 06_slx_read | fxslx | list and read the FIR model generated by example 05 |
| 07_slx_opt (+ `.solution_design`), 07b_export_observe | fxopt, fxexport | `.slx` design, linear-loop proof, `observe` |
| 08_slx_write_types | fxslx | change the FIR output type in a copy of the model generated by example 05 |
| 09_float_to_fixed (+ `.solution_design`), 09b_export_fpga | fxopt, fxexport | double -> fixed, `fpga_xc7` cost, ASIC/FPGA export |
| 10_blocks_mix | fxgraph | Saturation, Lookup1D, Product, Switch, RelationalOperator, Abs, DTC, Delay |
| 11_multirate_firdec | fxgraph | FIRDecimation, sample times `t` / `period` |
| 12_opt_violated, 12b_cert_counterexample | fxopt, fxcert | `ERROR_BOUND_VIOLATED`, counterexample replay |
| 13_opt_decimator (+ `.solution_design`), 13b_export_decimator | fxopt, fxexport | DiscreteFir + Downsample, overlay rule |
| 14_opt_limiter_lut | fxopt | LUT type choice, error `exhaustive` |
| 15_opt_agc_float | fxopt | an AGC loop with Product / Abs whose overflow proof fails: `sampled`, `OVERFLOW_NOT_PROVEN`, `RANGE_MODE_FAILED` (not every non-linear loop: 7.6) |
| 16a_nco_sim, 16b_opt_mixer, 16c_export_oscillator_mix | fxgraph, fxopt, fxexport | the NCO mixer route (11.1) |
| 20-27_err_* | fxgraph, fxopt, fxcert | one file per error kind of section 6 (27: fxcert refuses multirate) |

Exports (05, 07b, 09b, 13b, 16c) give `OUT_EXISTS` when re-run on the same tree: change `out`. Wall times in the
result files depend on the machine and its load; `run` / `time` fields differ between runs.

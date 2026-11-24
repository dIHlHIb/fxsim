# fxsim quickstart

This page gets you to a correct answer in a few calls. Every request (blocks tagged `json-run <tool>`) and every
command (`sh-run`) on it is run against the engine and its result checked before each release, so they work as
written. The one exception is the MATLAB command in section 3, which needs MATLAB. Run tools from the tree directory (the one
holding `engine/fxgraph`): `python3 -I engine/fxgraph < request.json`. Exit code 0 means `"status": "ok"`. Look
things up in [SKILL.md](SKILL.md) and `docs/en/` (GRAPH.md, BLOCKS.md, OPTIMIZER.md, EXPORT.md).

## 1. Five pitfalls to avoid

**1. `ints` are stored integers, not real values.** A fixed-point input is `{"s": 1, "wl": 16, "fl": 14, "ints":
[...]}`, where each sample is real value x 2^fl, already rounded (0.25 in `fixdt(1,16,14)` is `4096`). `s`, `wl`, `fl`
and the samples must be JSON integers: `0.25` is `BAD_INPUT` ("stored integer out of range or not an integer"), and
`"16"` is `BAD_TYPE`.

```json-run fxgraph expect=BAD_INPUT
{"design": {"blocks": {"x": {"type": "Inport"}, "y": {"type": "Outport"}}, "lines": [{"src": "x", "dst": "y"}]},
 "inputs": {"x": {"s": 1, "wl": 16, "fl": 14, "ints": [0.25, 0.5]}}}
```

**2. A parameter you leave out takes the Simulink library default.** Some defaults are surprising: a 1-D Lookup
extrapolates `Linear`, which Simulink rejects on fixed point (`PORT_TYPE_INVALID`, so write `"ExtrapMethod":
"Clip"`). A Relational Operator defaults to `<=`. A Discrete FIR's output defaults to `Same as input`, so the sum
wraps silently. Arithmetic blocks default to `RndMeth: Floor` and no saturation. Every result lists what you relied
on in `assumed_defaults`, so read it.

```json-run fxgraph expect=PORT_TYPE_INVALID
{"design": {"blocks": {"x": {"type": "Inport"},
   "L": {"type": "Lookup1D", "BreakpointsForDimension1": "[0 1 2]", "Table": "[0 1 2]"}, "y": {"type": "Outport"}},
  "lines": [{"src": "x", "dst": "L"}, {"src": "L", "dst": "y"}]},
 "inputs": {"x": {"s": 1, "wl": 8, "fl": 4, "ints": [1, 2]}}}
```

**3. Write explicit fixed-point types.** A Constant without `OutDataTypeStr` is `double`, which the engine refuses
outside a few recorded shapes (`UNSUPPORTED`). A parameter that does not fit its type stops the run, as in Simulink
(`PORT_TYPE_INVALID`: for example, Gain `3` in `fixdt(1,16,14)`). Inherited types around a feedback loop often
disagree (`TYPE_MISMATCH`). Type strings are `fixdt(s,w,f)`, `int8` .. `uint32`, `boolean` or `Inherit: ...`. A
string such as `sfix16_En14` is `BAD_TYPE`.

```json-run fxgraph expect=UNSUPPORTED
{"design": {"blocks": {"x": {"type": "Inport"}, "c": {"type": "Constant", "Value": "0.3"},
   "S": {"type": "Sum", "Inputs": "++", "OutDataTypeStr": "fixdt(1,17,14)"}, "y": {"type": "Outport"}},
  "lines": [{"src": "x", "dst": "S"}, {"src": "c", "dst": "S", "dport": 2}, {"src": "S", "dst": "y"}]},
 "inputs": {"x": {"s": 1, "wl": 16, "fl": 14, "ints": [0, 4096]}}}
```

**4. Wire every port exactly once, and give one input per root Inport.** An undriven input port is `UNSUPPORTED`
(Simulink grounds it, which fxsim does not support). A port driven twice is `BAD_CONNECTION`. A cycle without a Unit
Delay / Delay is `ALGEBRAIC_LOOP`. Every Inport needs an `inputs` entry under its own block name (a missing one is
`MISSING_FIELD`; a name that is not an Inport is `BAD_INPUT`). `nsteps` can shorten the inputs but not lengthen them
(`BAD_INPUT`).

```json-run fxgraph expect=ALGEBRAIC_LOOP
{"design": {"blocks": {"x": {"type": "Inport"}, "S": {"type": "Sum", "Inputs": "++", "OutDataTypeStr": "fixdt(1,16,4)"},
   "G": {"type": "Gain", "Gain": "0.5", "OutDataTypeStr": "fixdt(1,16,4)"}, "y": {"type": "Outport"}},
  "lines": [{"src": "x", "dst": "S"}, {"src": "G", "dst": "S", "dport": 2}, {"src": "S", "dst": "G"}, {"src": "S", "dst": "y"}]},
 "inputs": {"x": {"s": 1, "wl": 8, "fl": 4, "ints": [1]}}}
```

**5. Read the verdict line and handle refusals.** The check prints `FXSIM_VERDICT PASS`, `FXSIM_VERDICT FAIL` or `FXSIM_VERDICT INCOMPLETE`. On macOS, MATLAB R2026b can return process status 0 after a failed model check.
A refusal (`refusal_kind: "not_supported"`) means the configuration has not been verified bit-exact against
Simulink. Change to the supported form that `message` names, or report the limitation. Do not compute the numbers
yourself.

## 2. One template per block

Each template is a complete fxgraph request. Inputs are `fixdt(1,16,14)` unless stated otherwise. Copy one, change
the parameters, and keep the explicit types. `BLOCKS.md` lists every parameter and its default.

Gain:
```json-run fxgraph
{"design": {"blocks": {"x": {"type": "Inport"}, "y": {"type": "Outport"},
   "G": {"type": "Gain", "Gain": "0.75", "ParamDataTypeStr": "fixdt(1,16,14)", "OutDataTypeStr": "fixdt(1,16,14)",
         "RndMeth": "Floor", "SaturateOnIntegerOverflow": "off"}},
  "lines": [{"src": "x", "dst": "G"}, {"src": "G", "dst": "y"}]},
 "inputs": {"x": {"s": 1, "wl": 16, "fl": 14, "ints": [0, 4096, 8192, 16383, -16384, -300]}}, "nsteps": 6}
```

Sum (`Inputs` `"+-"` gives a - b; port 1 = a, port 2 = b):
```json-run fxgraph
{"design": {"blocks": {"a": {"type": "Inport"}, "b": {"type": "Inport"}, "y": {"type": "Outport"},
   "S": {"type": "Sum", "Inputs": "+-", "AccumDataTypeStr": "fixdt(1,17,14)", "OutDataTypeStr": "fixdt(1,17,14)",
         "RndMeth": "Floor", "SaturateOnIntegerOverflow": "off"}},
  "lines": [{"src": "a", "dst": "S", "dport": 1}, {"src": "b", "dst": "S", "dport": 2}, {"src": "S", "dst": "y"}]},
 "inputs": {"a": {"s": 1, "wl": 16, "fl": 14, "ints": [0, 4096, 8192, 16383, -16384, -300]},
            "b": {"s": 1, "wl": 16, "fl": 14, "ints": [8192, 8192, -4096, 16000, 100, -16384]}}, "nsteps": 6}
```

Product (`"**"` multiplies; `"*/"` divides, and a division by zero saturates and is counted):
```json-run fxgraph
{"design": {"blocks": {"a": {"type": "Inport"}, "b": {"type": "Inport"}, "y": {"type": "Outport"},
   "P": {"type": "Product", "Inputs": "**", "OutDataTypeStr": "fixdt(1,32,28)", "RndMeth": "Floor",
         "SaturateOnIntegerOverflow": "off"}},
  "lines": [{"src": "a", "dst": "P", "dport": 1}, {"src": "b", "dst": "P", "dport": 2}, {"src": "P", "dst": "y"}]},
 "inputs": {"a": {"s": 1, "wl": 16, "fl": 14, "ints": [0, 4096, 8192, 16383, -16384, -300]},
            "b": {"s": 1, "wl": 16, "fl": 14, "ints": [8192, 8192, -4096, 16000, 100, -16384]}}, "nsteps": 6}
```

Unit Delay (the output is the input of the previous step and keeps the input's type):
```json-run fxgraph
{"design": {"blocks": {"x": {"type": "Inport"}, "D": {"type": "UnitDelay", "InitialCondition": "0"}, "y": {"type": "Outport"}},
  "lines": [{"src": "x", "dst": "D"}, {"src": "D", "dst": "y"}]},
 "inputs": {"x": {"s": 1, "wl": 16, "fl": 14, "ints": [0, 4096, 8192, 16383, -16384, -300]}}, "nsteps": 6}
```

Delay (`DelayLength` steps; the library default is 2):
```json-run fxgraph
{"design": {"blocks": {"x": {"type": "Inport"}, "D": {"type": "Delay", "DelayLength": "3", "InitialCondition": "0"},
   "y": {"type": "Outport"}},
  "lines": [{"src": "x", "dst": "D"}, {"src": "D", "dst": "y"}]},
 "inputs": {"x": {"s": 1, "wl": 16, "fl": 14, "ints": [0, 4096, 8192, 16383, -16384, -300]}}, "nsteps": 6}
```

Discrete FIR (write every type, because the output defaults to the input type):
```json-run fxgraph
{"design": {"blocks": {"x": {"type": "Inport"}, "y": {"type": "Outport"},
   "F": {"type": "DiscreteFir", "Coefficients": "[0.25 0.5 0.25]", "CoefDataTypeStr": "fixdt(1,16,15)",
         "ProductDataTypeStr": "fixdt(1,32,29)", "AccumDataTypeStr": "fixdt(1,34,29)", "OutDataTypeStr": "fixdt(1,18,14)",
         "RndMeth": "Floor", "SaturateOnIntegerOverflow": "off"}},
  "lines": [{"src": "x", "dst": "F"}, {"src": "F", "dst": "y"}]},
 "inputs": {"x": {"s": 1, "wl": 16, "fl": 14, "ints": [0, 4096, 8192, 16383, -16384, -300]}}, "nsteps": 6}
```

Biquad Filter (one row `[b0 b1 b2 1 a1 a2]` per section). Only the output type can be set, and every internal type
is the input type, so give a wide input (here `fixdt(1,24,12)`). fxopt refuses this block:
```json-run fxgraph
{"design": {"blocks": {"x": {"type": "Inport"}, "y": {"type": "Outport"},
   "B": {"type": "BiquadFilter", "BiQuadCoeffs": "[0.0675 0.135 0.0675 1 -1.143 0.4128]",
         "outputMode": "Binary point scaling", "outputWordLength": "32", "outputFracLength": "24"}},
  "lines": [{"src": "x", "dst": "B"}, {"src": "B", "dst": "y"}]},
 "inputs": {"x": {"s": 1, "wl": 24, "fl": 12, "ints": [0, 4096, 8192, -8192, 100000, -100000]}}, "nsteps": 6}
```

1-D Lookup Table (always write `"ExtrapMethod": "Clip"`; the breakpoints must fit their type):
```json-run fxgraph
{"design": {"blocks": {"x": {"type": "Inport"}, "y": {"type": "Outport"},
   "L": {"type": "Lookup1D", "BreakpointsForDimension1": "[-2 -1 0 1 2]", "Table": "[-0.96 -0.76 0 0.76 0.96]",
         "BreakpointsForDimension1DataTypeStr": "fixdt(1,16,13)", "TableDataTypeStr": "fixdt(1,16,14)",
         "OutDataTypeStr": "fixdt(1,16,14)", "InterpMethod": "Linear point-slope", "ExtrapMethod": "Clip"}},
  "lines": [{"src": "x", "dst": "L"}, {"src": "L", "dst": "y"}]},
 "inputs": {"x": {"s": 1, "wl": 16, "fl": 14, "ints": [0, 4096, 8192, 16383, -16384, -300]}}, "nsteps": 6}
```

Data Type Conversion (write `OutDataTypeStr`: left out, the type is inherited by back propagation, and into an untyped
Outport that means `double`):
```json-run fxgraph
{"design": {"blocks": {"x": {"type": "Inport"}, "y": {"type": "Outport"},
   "C": {"type": "DataTypeConversion", "OutDataTypeStr": "fixdt(1,8,6)", "RndMeth": "Nearest",
         "SaturateOnIntegerOverflow": "on"}},
  "lines": [{"src": "x", "dst": "C"}, {"src": "C", "dst": "y"}]},
 "inputs": {"x": {"s": 1, "wl": 16, "fl": 14, "ints": [0, 4096, 8192, 16383, -16384, -300]}}, "nsteps": 6}
```

Saturation (it always limits. It has no `SaturateOnIntegerOverflow` parameter, and writing one is `BAD_PARAM`. Both
limits must fit the output type):
```json-run fxgraph
{"design": {"blocks": {"x": {"type": "Inport"}, "y": {"type": "Outport"},
   "Q": {"type": "Saturation", "UpperLimit": "0.5", "LowerLimit": "-0.5", "OutDataTypeStr": "Inherit: Same as input",
         "RndMeth": "Floor"}},
  "lines": [{"src": "x", "dst": "Q"}, {"src": "Q", "dst": "y"}]},
 "inputs": {"x": {"s": 1, "wl": 16, "fl": 14, "ints": [0, 4096, 8192, 16383, -16384, -300]}}, "nsteps": 6}
```

MinMax (the `Function` default is `min`):
```json-run fxgraph
{"design": {"blocks": {"a": {"type": "Inport"}, "b": {"type": "Inport"}, "y": {"type": "Outport"},
   "M": {"type": "MinMax", "Function": "max", "Inputs": "2", "OutDataTypeStr": "fixdt(1,16,14)", "RndMeth": "Floor",
         "SaturateOnIntegerOverflow": "off"}},
  "lines": [{"src": "a", "dst": "M", "dport": 1}, {"src": "b", "dst": "M", "dport": 2}, {"src": "M", "dst": "y"}]},
 "inputs": {"a": {"s": 1, "wl": 16, "fl": 14, "ints": [0, 4096, 8192, 16383, -16384, -300]},
            "b": {"s": 1, "wl": 16, "fl": 14, "ints": [8192, 8192, -4096, 16000, 100, -16384]}}, "nsteps": 6}
```

Switch (port 1 is passed when the criterion on port 2 holds, port 3 otherwise. The default criterion is
`u2 > Threshold`):
```json-run fxgraph
{"design": {"blocks": {"a": {"type": "Inport"}, "c": {"type": "Inport"}, "b": {"type": "Inport"}, "y": {"type": "Outport"},
   "W": {"type": "Switch", "Criteria": "u2 >= Threshold", "Threshold": "0", "OutDataTypeStr": "fixdt(1,16,14)"}},
  "lines": [{"src": "a", "dst": "W", "dport": 1}, {"src": "c", "dst": "W", "dport": 2}, {"src": "b", "dst": "W", "dport": 3},
            {"src": "W", "dst": "y"}]},
 "inputs": {"a": {"s": 1, "wl": 16, "fl": 14, "ints": [0, 4096, 8192, 16383, -16384, -300]},
            "c": {"s": 1, "wl": 16, "fl": 14, "ints": [8192, 8192, -4096, 16000, 100, -16384]},
            "b": {"s": 1, "wl": 16, "fl": 14, "ints": [1, 2, 3, 4, 5, 6]}}, "nsteps": 6}
```

HDL NCO. The input is a boolean valid signal. Port 1 is the waveform, `fixdt(1, OutputWL, OutputFL)`, and the last
port is the valid output. The latency is 6 steps. fxopt refuses designs that contain this block:
```json-run fxgraph
{"design": {"blocks": {"v": {"type": "Inport"}, "y": {"type": "Outport"}, "vo": {"type": "Outport"},
   "N": {"type": "HDLNCO", "PhaseIncrementSource": "Property", "PhaseIncrement": "4096", "AccumulatorWL": "16",
         "NumQuantizerAccumulatorBits": "12", "Waveform": "Sine", "DitherSource": "None", "OutputWL": "16", "OutputFL": "14"}},
  "lines": [{"src": "v", "dst": "N"}, {"src": "N", "dst": "y"}, {"src": "N", "sport": 2, "dst": "vo"}]},
 "inputs": {"v": {"type": "boolean", "ints": [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]}}, "nsteps": 10}
```

Add `"diagnostics": true` to count wraps and saturations per block (`summary.diagnostics`). Add `"outputs": "all"`
to record every block's output.

## 3. The command chain: simulate, optimise, export, check in Simulink

The example is a 3-tap FIR. Save the three requests below under `runs/qs/`, then run the commands that follow them.

`runs/qs/fir_sim.json`, the simulation:
```json-run fxgraph save=runs/qs/fir_sim.json
{"design": {"blocks": {"x": {"type": "Inport"}, "y": {"type": "Outport"},
   "F": {"type": "DiscreteFir", "Coefficients": "[0.25 0.5 0.25]", "CoefDataTypeStr": "fixdt(1,16,15)",
         "ProductDataTypeStr": "fixdt(1,32,29)", "AccumDataTypeStr": "fixdt(1,34,29)", "OutDataTypeStr": "fixdt(1,18,14)",
         "RndMeth": "Floor", "SaturateOnIntegerOverflow": "off"}},
  "lines": [{"src": "x", "dst": "F"}, {"src": "F", "dst": "y"}]},
 "inputs": {"x": {"s": 1, "wl": 16, "fl": 14, "ints": [0, 4096, 8192, 14745, -14746, -8192, 12000, -300]}},
 "diagnostics": true, "cost": true}
```

`runs/qs/fir_opt.json`, the word-length optimisation (the same design, a sine input, an input range and a
tolerance):
```json-run fxopt save=runs/qs/fir_opt.json
{"design": {"blocks": {"x": {"type": "Inport"}, "y": {"type": "Outport"},
   "F": {"type": "DiscreteFir", "Coefficients": "[0.25 0.5 0.25]", "CoefDataTypeStr": "fixdt(1,16,15)",
         "ProductDataTypeStr": "fixdt(1,32,29)", "AccumDataTypeStr": "fixdt(1,34,29)", "OutDataTypeStr": "fixdt(1,18,14)",
         "RndMeth": "Floor", "SaturateOnIntegerOverflow": "off"}},
  "lines": [{"src": "x", "dst": "F"}, {"src": "F", "dst": "y"}]},
 "inputs": {"x": {"kind": "sine", "amplitude": 0.9, "omega": 0.05, "n": 512, "type": {"s": 1, "wl": 16, "fl": 14}}},
 "input_ranges": {"x": {"type": {"s": 1, "wl": 16, "fl": 14}, "min": -0.9, "max": 0.9}},
 "error": {"max_abs": 0.0009765625}, "time_budget_s": 10, "workers": 2}
```

Read `summary` (`hwbits`, `max_error_over_tolerance`, `overflow`, `warnings`) and never deliver a plan whose
warnings include `ERROR_BOUND_VIOLATED`. Copy `solution.blocks` over your design's blocks and keep your lines. On this
tree, fxopt chose the types written into the export request below, cutting 280 hwbits to 124.

`runs/qs/fir_export.json`, the export of the optimised design (`out` must be a new directory inside the tree):
```json-run fxexport save=runs/qs/fir_export.json
{"name": "fir3", "out": "runs/qs/fir3",
 "design": {"blocks": {"x": {"type": "Inport"}, "y": {"type": "Outport"},
   "F": {"type": "DiscreteFir", "Coefficients": "[0.25 0.5 0.25]", "CoefDataTypeStr": "fixdt(0,6,6)",
         "ProductDataTypeStr": "fixdt(1,12,12)", "AccumDataTypeStr": "fixdt(1,14,13)", "OutDataTypeStr": "fixdt(1,12,11)",
         "RndMeth": "Floor", "SaturateOnIntegerOverflow": "off"}},
  "lines": [{"src": "x", "dst": "F"}, {"src": "F", "dst": "y"}]},
 "inputs": {"x": {"s": 1, "wl": 16, "fl": 14, "ints": [0, 4096, 8192, 14745, -14746, -8192, 12000, -300]}},
 "nsteps": 8}
```

The commands (fxopt also takes the request path as its argument):
```sh-run
python3 -I engine/fxgraph < runs/qs/fir_sim.json > runs/qs/fir_sim.out.json
python3 -I engine/fxopt runs/qs/fir_opt.json > runs/qs/fir_opt.out.json
python3 -I engine/fxexport < runs/qs/fir_export.json > runs/qs/fir_export.out.json
python3 -I -c "import json; print(json.load(open('runs/qs/fir_export.out.json'))['check_batch_command'])"
```

Expected result of the simulation (`runs/qs/fir_sim.out.json`, `outputs.y`; stored integers of fixdt(1,18,14)):
```json-expect runs/qs/fir_sim.out.json
{"y": {"type": "fixdt(1,18,14)", "ints": [0, 1024, 4096, 8806, 5734, -5735, -4783, 3877]}}
```

The export writes `runs/qs/fir3/` (the model `fir3.slx`, `fir3_io.json`, `check_fir3.m` and `manifest.json`).
Its `outputs` (also in `manifest.json`) gives each root output's compiled `type`, `period`, `offset` and `samples`.
Its result gives three MATLAB commands: `check_command` (for an interactive session), `check_batch_command` (for a
terminal) and `check_batch_command_here` (run inside the package directory after copying it). On a machine with
MATLAB, Simulink and Fixed-Point Designer, run the check from the package root and read its verdict:

```sh
(cd runs/qs/fir3 && matlab -batch "check_fir3;") | python3 -I docs/en/read_verdict.py fir3
```

The reader prints the check script's JSON record and exits 0 only for `PASS`. How to read the result:
- `"verdict": "PASS"`: every output sample and every compiled type is bit-exact in Simulink. This is the only
  verdict that verifies the package.
- `"verdict": "FAIL"`: there is a difference. `first_diff` gives the block, the step, and the expected and actual
  values.
- `"verdict": "INCOMPLETE"`: everything compared was equal, but something could not be compared (`reason`).
- `Unverified replay: missing fxexport-check record`: the check did not run to the end (MATLAB error or license
  problem).

On macOS, MATLAB R2026b can return process status 0 after a failed model check, so use the reader's exit status and printed verdict. See `docs/en/EXPORT.md`. For FPGA / ASIC, pass `"hw": {"ProdHWDeviceType": "ASIC/FPGA"}` to every call.

## 4. Several inputs

Each signal is its own root Inport, and each Inport has its own entry in `"inputs"`, keyed by the Inport's block name
(see the Sum, Product, MinMax and Switch templates). Each entry has its own type. All entries must be equally long
(`BAD_INPUT` otherwise), and `nsteps` may shorten them. A boolean signal is `{"type": "boolean", "ints": [0, 1, ...]}`. A vector Inport of
width n (`PortDimensions`) takes one entry per element, `"x[1]"` .. `"x[n]"`, all of one type, and its outputs come
back as `y[1]` .. `y[n]`:

```json-run fxgraph
{"design": {"blocks": {"x": {"type": "Inport", "PortDimensions": "2"}, "y": {"type": "Outport"},
   "G": {"type": "Gain", "Gain": "0.5", "ParamDataTypeStr": "fixdt(1,16,14)", "OutDataTypeStr": "fixdt(1,16,14)"}},
  "lines": [{"src": "x", "dst": "G"}, {"src": "G", "dst": "y"}]},
 "inputs": {"x[1]": {"s": 1, "wl": 16, "fl": 14, "ints": [4096, 8192]},
            "x[2]": {"s": 1, "wl": 16, "fl": 14, "ints": [-4096, 100]}}}
```

**From Workspace.** In a JSON design there is no From Workspace block and no workspace variable. `"type":
"FromWorkspace"` is `UNKNOWN_BLOCK_TYPE`, and a parameter that names a variable (`"Gain": "k"`) is `UNSUPPORTED`.
Use an Inport and put the data in `inputs`. Data comes only from `ints` (or a `kind: "sine"` generator for fxopt /
fxcert). There is no reference to a variable or a file. In a saved `.slx`, fxslx reads each top-level From
Workspace block as an Inport of the same block name. `info.workspace_sources` maps that name to the MATLAB variable,
and `info.inport_types` gives its type. The data goes in `inputs` under the block name, not under the variable name.
The exported model above has such a source:

```sh-run
echo '{"op": "read", "path": "runs/qs/fir3/fir3.slx"}' | python3 -I engine/fxslx > runs/qs/fir3_read.out.json
```

Here `info.workspace_sources` is `{"src_x": "fir3_in1"}`, so a simulation of the read-back design takes
`"inputs": {"src_x": {...}}`.

## 5. The common refusal codes

Every error has `code`, `where` (a JSON path into the request), `message` (the specific fix), `refusal_kind` and
`next_step`. Act on `message`.

| Code | Kind | Meaning | Usual fix |
|---|---|---|---|
| `MISSING_FIELD` | request | a required field is missing: `design`, an Inport's `inputs` entry (or `x[k]` element), a parameter `BLOCKS.md` marks required (such as Bit Clear's `iBit`) | add it at `where` |
| `BAD_PARAM` | request | an unknown request field or block parameter (misspelt, or one the block lacks, such as `SaturateOnIntegerOverflow` on Saturation), or a bad value | use the names in `BLOCKS.md` |
| `BAD_TYPE` | request | a malformed type: a non-integer `wl`, or a type string such as `sfix16_En14` | `fixdt(s,w,f)`, `int8` .. `uint32`, `boolean`, `Inherit: ...` |
| `BAD_INPUT` | request | the input data is wrong: a non-integer or out-of-range sample, an entry for a block that is not an Inport, `nsteps` longer than the data | stored integers within the type, one entry per Inport |
| `BAD_CONNECTION` | request | a port driven twice, or a line to a block or port that does not exist | one line per input port |
| `UNKNOWN_BLOCK_TYPE` | request | `type` is not a supported block type (the message suggests the nearest names and lists every accepted type) | use the `type` spelling in `BLOCKS.md` |
| `ALGEBRAIC_LOOP` | request | a cycle without a delay (the message names its blocks) | put a Unit Delay / Delay in the loop |
| `PORT_TYPE_INVALID` | simulink_rejects | Simulink rejects this too: a parameter that does not fit its type, Lookup `ExtrapMethod` `Linear` on fixed point | widen the type or change the value or setting the message names |
| `TYPE_MISMATCH` | simulink_rejects | inherited types do not agree, typically around a loop | write explicit types on the loop's blocks |
| `UNSUPPORTED` | not_supported | not verified bit-exact against Simulink: an unrecorded parameter value or shape, a `double` signal outside the recorded shapes, an undriven port, a workspace variable, hardware other than the host or `ASIC/FPGA`, fxopt on a design with HDL NCO / Biquad / Discrete Filter | use the supported form the message names, or report the limitation |
| `UNSUPPORTED_WIDTH` | not_supported | `"engine": "c"` on a design the C core cannot take (intermediates wider than 126 bits, Python-only shapes) | leave `engine` at `auto` |
| `UNSUPPORTED_EXPORT` | not_supported | fxexport cannot write this design (for example `hw.ParameterOverflowMsg` other than the default) | simulate it with fxgraph, and do not claim a Simulink check |
| `INTERNAL`, `CORE_FAILED` | defect | an fxsim bug: do not use any number from that call | report `input_sha256` with the request; after `CORE_FAILED`, try `"engine": "python"` |

Here are two of them, run on the engine:

```json-run fxgraph expect=BAD_PARAM
{"design": {"blocks": {"x": {"type": "Inport"}, "G": {"type": "Gain", "Gain": "0.5", "RndMet": "Floor"},
   "y": {"type": "Outport"}}, "lines": [{"src": "x", "dst": "G"}, {"src": "G", "dst": "y"}]},
 "inputs": {"x": {"s": 1, "wl": 16, "fl": 14, "ints": [0, 4096]}}}
```

```json-run fxgraph expect=UNSUPPORTED_WIDTH
{"design": {"blocks": {"a": {"type": "Inport"}, "b": {"type": "Inport"}, "y": {"type": "Outport"},
   "P": {"type": "Product", "Inputs": "**", "OutDataTypeStr": "fixdt(1,200,180)"}},
  "lines": [{"src": "a", "dst": "P"}, {"src": "b", "dst": "P", "dport": 2}, {"src": "P", "dst": "y"}]},
 "inputs": {"a": {"s": 1, "wl": 100, "fl": 90, "ints": [1, 2]}, "b": {"s": 1, "wl": 100, "fl": 90, "ints": [3, 4]}},
 "engine": "c"}
```

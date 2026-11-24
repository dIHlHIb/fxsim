# fxexport: build a Simulink model and a one-command check

`python3 -I engine/fxexport < request.json` turns an fxsim design into a new Simulink model (`.slx`) together with
the inputs, the outputs fxsim computed, and a self-contained MATLAB script that replays the inputs in Simulink and
compares every output sample and every compiled data type bit for bit. Ground rules and errors: [README.md](README.md).

Use it as the last step of the workflow: design and optimise in fxsim, export, run the check script in MATLAB.

**In short, for whoever hands the package over:** the check script simulates the exported model in Simulink on the
packaged inputs and compares, bit for bit, every output sample and the compiled data type of every design block with
what fxsim computed. `PASS` means all of them are equal, so Simulink R2026b computes exactly what fxsim computed
for these inputs (for other inputs, re-export). You need no MATLAB to export; the person or job running the script
needs MATLAB, Simulink and Fixed-Point Designer.

## Request

```json
{"name": "iir1", "out": "runs/export/iir1",
 "design": {"blocks": {...}, "lines": [...]},
 "inputs": {"x": {"s": 1, "wl": 16, "fl": 14, "ints": [100, -2000, 16383]},
            "en": {"type": "boolean", "ints": [1, 0, 1]}},
 "nsteps": 3, "hw": {}}
```

| Field | Meaning |
|---|---|
| `name` | model name: a valid MATLAB identifier (letters, digits, `_`, starting with a letter) |
| `out` | a **new** directory for the package, inside the tree. Relative paths resolve against the tree directory; an existing directory is refused (`OUT_EXISTS`) |
| `design`, `inputs`, `hw` | exactly as for fxgraph ([GRAPH.md](GRAPH.md)); a boolean design input is `{"type": "boolean", "ints": [0, 1, ...]}` |
| `nsteps` / `stop_time` | as for fxgraph: `nsteps` for a single-rate design, `stop_time` (seconds) for a multirate one |

- **Vector inputs** are given element by element: `"x[1]": {...}, "x[2]": {...}`, all of one type. The Inport may
  declare `PortDimensions`; if it does not, the width is taken from the elements given and written on the Inport.
- **`hw.ParameterOverflowMsg`**: leave it out (Simulink's default, `error`: a parameter that does not fit its type
  stops the run, in fxsim and in Simulink). `warning` / `none` are refused at export (`UNSUPPORTED_EXPORT`).

The design is first run through fxgraph. A design fxgraph refuses is refused here with the same code (its message
starts with `engine:`).

## The package

| File | Contents |
|---|---|
| `<name>.slx` | the model: your design inside a subsystem `DUT`, fed by From Workspace blocks at the top level |
| `<name>_io.json` | the inputs (bit patterns, exact decimal integers), every expected output sample with its type, the compiled type of every design block, every block parameter written, the model settings to verify, the model's sha256 |
| `check_<name>.m` | the self-contained check script (needs only MATLAB, Simulink and Fixed-Point Designer) |
| `manifest.json` | the file digests and the export's own information (below) |

The exported SLX includes the R2026b Update 1 ReleaseInfo XML record for compatibility. On that release, six models loaded with default solver settings when the XML comment was missing. Restoring the comment restored their saved settings. The exporter preserves the tested record, including its build number, release, update, date and checksum. The creator field identifies fxsim.

## Running the check

The result gives two commands:

- `check_command`, for an interactive MATLAB session: `cd('<package path>'); r = check_<name>; fprintf(...)`.
  The path is absolute, so MATLAB's own current directory does not matter.
- `check_batch_command`, for a terminal or a script: `matlab -batch "..."`.

Every command prints, after the script's JSON line, one fixed line: `FXSIM_VERDICT PASS`, `FXSIM_VERDICT FAIL` or
`FXSIM_VERDICT INCOMPLETE`. **Read the verdict line.** On macOS, MATLAB R2026b can return process status 0 after a failed model check.
- `check_batch_command_here`: the same without the absolute path, to run from inside the package directory --
  use it after copying or moving the package (the other two keep the original location).

The script prints one JSON line and returns the same record as `r`; the commands add the `FXSIM_VERDICT` line from
`r.verdict`. Use the packaged `docs/en/read_verdict.py` to check the single structured verdict.

| `verdict` | Meaning |
|---|---|
| `PASS` | every output sample and every compiled type equal fxsim's, and Simulink gave no warning while compiling or simulating |
| `FAIL` | a difference; `first_diff` says where (block, step, expected and actual value or type) |
| `INCOMPLETE` | everything that could be compared was equal, but something could not be compared (`reason` says what) |

Only `PASS` verifies the package. Before anything else the script checks that the model file and the io file hash to
the sha256 in `manifest.json` (`PACKAGE_DIGEST_MISMATCH` otherwise: an io file from another package, or an edited
one), then that every sample's decimal integer in the io file is the value of its bit pattern in the declared type
(`IO_INCONSISTENT` otherwise, even when the digest was recomputed after the edit). It also checks its own inputs
(rebuilt from the bit patterns and compared bit for bit) and that the model file is the one exported (sha256).

- **Further output ports.** A block with more than one output port has each further port's compiled type checked
  on its own. A port without an expected type leaves the verdict at `INCOMPLETE`. An expected port the block does not have, or one
  listed twice, is a `FAIL` (`PORT_SET_MISMATCH`): every expected type must be one Simulink showed.
- **Warnings.** Warnings are switched on for the check and restored afterwards. A warning while compiling
  (`COMPILE_WARNING`) or simulating (`SIM_WARNING`, for example a failing Assertion) is a `FAIL`.
- **Fresh build every run.** A cached build hides compile-time warnings (a MATLAB Function that warned on its first
  compile is silent on the next one). So the script builds into a new temporary cache and code generation folder
  (deleted at the end, MATLAB's own settings restored) and runs `clear mex` before compiling. Two side effects:
  every run regenerates the model's code (a few seconds more), and `clear mex` unloads every MEX file loaded in the
  MATLAB session the script runs in (they load again when next used). Run the check in its own session
  (`check_batch_command`) if that matters.
- **Defaults compared with the library.** A `PASS` also needs every parameter your request left out, and
  that the export wrote from fxsim's own table of library defaults, to equal a freshly placed library block in your
  Simulink (`io.json` `library_defaults`: block, library path, parameter, value). A difference is a `FAIL`
  (`ASSUMED_DEFAULT_NOT_LIBRARY`, `first_diff` names the block, the parameter, the package value and the library
  value). Without this step a drifted fxsim default would pass, because the expected outputs were computed with the
  same drifted value. A library block that cannot be placed, or that lacks the parameter, leaves the verdict at
  `INCOMPLETE` (`LIBRARY_DEFAULT_UNCHECKED`). The result counts the comparisons in `library_defaults_checked`.
  Packages exported before this step have no `library_defaults` and skip it.
- **What `PASS` does not say.** `PASS` means the model equals the package's own expectation, and (with the step
  above) that the defaults fxsim filled in are Simulink's. It does not compare the model with your request: keep
  `manifest.request_sha256` (below) to tie a package to the request it came from.
- **observe.** Observing internal blocks adds outputs and changes nothing else in the design: a design the engine
  refuses is refused with or without `observe`.

## Result

`{"status": "ok", ...}` with:

| Key | Contents |
|---|---|
| `out`, `out_abs` | the package directory, relative to the tree and absolute |
| `files` | every package file with its sha256, `manifest.json` included |
| `check_command`, `check_batch_command`, `verdicts` | see "Running the check" |
| `assumed_defaults` | per block, the parameters your design left out and the value used for each: the export writes exactly these values into the model, so Simulink computes with what fxsim computed |
| `dropped_parameters` | layout / presentation parameters of your design that are not written |
| `request_sha256` (manifest) | sha256 of your request's `design` exactly as given, in fxexport's canonical JSON (keys sorted, no spaces). `design_sha256` is the design after the export wrote the library defaults out and added observation blocks, so it differs from your request's whenever it did either |
| `library_deviations` (manifest) | parameters written on purpose with a value other than the library block's, each with the reason (today: a Lookup's `InputSameDT` 'off') |
| `library_defaults_unchecked` (manifest) | defaults written for block types without a recorded library path: the check cannot compare them |
| `roundtrip` | the export's self-check (below) |
| `outputs`, `blocks`, `multirate`, `hw_profile` | a summary of what was exported |
| `outputs` (result and manifest) | one row per root output port, the same in the result and in `manifest.json`: `port`, `root_block`, `width` (elements; a complex port counts its two parts), `samples` (per element), and each output's compiled `type` (`s,wl,fl` or `boolean`) and clock: `period` and `offset` in model-time units, the values `<model>_io.json` gives for that port and the compiled sample time `[period offset]` the check expects of its root Outport. In a multirate design this tells which output runs at which rate (a Downsample by 2 gives period 2; an output fed from an Inport at `[2 1]` gives period 2, offset 1); a single-rate design gives the base step and offset 0 |

## The self-check

Before writing anything, the export reads the model it built back with fxsim's own `.slx` reader, compares every
block, line and parameter with your design, re-simulates it and compares every compiled type and every output sample
with the original run. Any difference is `EXPORT_ROUNDTRIP_MISMATCH` and no package is written: that is a defect of
the export, not of your design; report it with `input_sha256`.

## What the model looks like

- Each design block is written as the Simulink library block it stands for, with the dialog parameters fxsim used,
  including library defaults for the parameters your design omitted.
- Inputs come from From Workspace blocks (fixed-point or logical time series, sample time from the design).
- In a scalar, single-rate design, an Outport inside `DUT` whose driver lies on a feedback loop is written with its
  compiled type. Simulink otherwise types a loop whose types are all inherited differently once its output leaves a
  subsystem through an `Inherit: auto` port. Every other DUT Outport keeps `Inherit: auto`, as the design has it: a
  typed one would be a constraint Simulink checks against the driver's own rule. An Outport driven by a block with
  several outputs keeps `Inherit: auto` too.
- An Extract Bits that takes one bit of a signed input makes Simulink warn while compiling
  (`SimulinkFixedPoint:util:sfix_dtpropBitLess`: the field is widened to two bits inside the block). The package
  lists that warning under `warnings_allowed` (id and blocks), and the check passes over a warning with that id whose
  message names one of those blocks; values and types are compared as usual. Any other warning still fails the
  check.
- Two more warnings are listed the same way when they apply: a Switch whose control is boolean
  (`Simulink:blocks:SwitchIgnoringThreshold`: Simulink ignores the criterion and threshold) and a Demux of 4 elements
  into 3 outputs (`Simulink:blocks:MuxNonuniformDistribution`), and a Multiport Switch with `DiagnosticForDefault`
  `Warning` whose control leaves the data ports' range (`Simulink:blocks:MPSwitchControlInputOORIndexValue`: Simulink
  takes the default port and warns). The check's `warnings_allowed` reports the allowed ids it met, at compile or run
  time.
- The model's diagnostics are Simulink's defaults where they change results: `ParameterOverflowMsg` and
  `ParameterDowncastMsg` are `error`, and the check script verifies both. `ParameterPrecisionLossMsg` is `none`
  (it only reports an inexact parameter such as a gain of 0.3; with the default `warning` every such package would
  fail the no-warning rule).
- Vector and multirate designs, enabled subsystems, buses, complex signals, MATLAB Function blocks, the DSP HDL
  blocks (FIR, FIR Decimator, NCO), RAMs, counters and the bit-operation blocks are exported as their recorded
  Simulink forms.

## Refusals

`UNSUPPORTED_EXPORT` (`refusal_kind: not_supported`) means the design runs in fxsim but this part cannot yet be
written as a model whose check script verifies it. Change what `where` / `message` name, or keep that part verified
in fxsim only. Among them:

- a block or parameter value without an export template (the message names it);
- hardware settings other than the device defaults of the supported devices;
- single-rate designs whose inputs have different periods or offsets;
- a design without Outports, or whose simulation gives no samples to compare.

| Code | `refusal_kind` | Meaning |
|---|---|---|
| `UNSUPPORTED_EXPORT` | `not_supported` | see above |
| `EXPORT_ROUNDTRIP_MISMATCH` | `defect` | the self-check failed; no package was written; please report it |
| `OUT_EXISTS` | `request` | `out` already exists; give a new directory |
| `BAD_INPUT` | `request` | an input record is neither `{s, wl, fl, ints}` nor `{"type": "boolean", "ints": [...]}` |
| any fxgraph code | as in [README.md](README.md) | the design itself was refused by the engine |

Every error result carries `refusal_kind` and `next_step`, like every other tool.

## Common refusals and how to rewrite the design

The engine refuses what Simulink was not recorded doing, rather than guessing. The refusals an agent meets most
often, with the change that is accepted (the message always names the block and parameter):

| Refusal (message excerpt) | Rewrite |
|---|---|
| `hw.ParameterOverflowMsg 'none' ...` (`UNSUPPORTED_EXPORT`) | leave `ParameterOverflowMsg` out; give parameters that fit their types |
| `... does not fit ...` / `PORT_TYPE_INVALID` on a parameter | the parameter overflows its type (Simulink stops too): widen the type or change the value |
| `a line enters input 3 of a Mux with 2 inputs` | write the Mux's `Inputs` (a count, or a list of widths such as `[2 1 1]`) |
| `Demux: width 5 not divisible by 2` | give `Outputs` as a list of widths (`[3 2]`, or `[-1 2]`: -1 takes the rest) |
| `Multiport Switch with a vector control` | use a scalar control (one port picked for every element) |
| `NumDelays 6: recorded 1, 2, 3, 4, 5, 8` | use a recorded count, or chain Tapped Delays |
| `an enabled subsystem Outport with InitialOutput [] driven directly by a Constant` | give `InitialOutput` explicitly |
| `... inputs run at periods [1, 2] ...` (`UNSUPPORTED_RATE`) on a Delay with ports or a synchronous delay | give the data and its enable / reset one rate (Rate Transition in front) |
| `EnablePort on a vector signal` | reduce the enable to a scalar first (Logical Operator with one input over the vector) |
| `Selector: one dimension only` | split the selection into one-dimensional Selectors |
| `a counter type without integer bits ... CountInit ...` | start the HDL Counter at 0, or give the count type an integer bit |
| `Math Function square of a complex signal` | split with Complex to Real-Imag and compute on the parts |
| `a complex Inport` | build complex signals inside the design with Real-Imag to Complex |
| `Ground feeding anything but one Sum input` | feed a Constant 0 of the wanted type instead |

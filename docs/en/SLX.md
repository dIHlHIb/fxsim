# fxslx: reading and writing Simulink models

`python3 -I engine/fxslx < request.json` reads a Simulink `.slx` file into fxsim's design JSON and writes chosen
data types back into a copy of the model. Ground rules and errors: [README.md](README.md).

## Requests

```json
{"op": "list", "path": "model.slx"}
{"op": "read", "path": "model.slx", "scope": "SUD"}
{"op": "write_types", "path": "model.slx", "scope": "SUD", "out": "runs/model_opt.slx",
 "blocks": {"Q0_a1": {"OutDataTypeStr": "fixdt(1,18,12)", "ParamDataTypeStr": "fixdt(1,16,13)"}}}
```

`path` may be absolute or relative to the tree directory. `scope` (optional) is the path of a subsystem to read as
the design (its ports become the design's Inports / Outports); nested subsystems are joined by `/`, and a `/` inside
a block name is written `//`.

### list

`{"status": "ok", "top_level": [...], "subsystems": [...]}`: every top-level block with the parameters written in
the file (`params`; for a subsystem, `inputs` = what drives each input port), and every subsystem path usable as
`scope`, and `unsupported`: what a `read` will refuse, found while listing (each with `where`, `code`,
`message`) -- `list` itself goes on past them. Use it when a model has its own signal sources at the top level: read
the subsystem under test, one that holds none of the unsupported places.

### read

`{"status": "ok", "design": {"blocks", "lines", "port_types"?}, "info": {...}, "file_sha256": "..."}`. `design` goes
to fxgraph / fxopt as it is; pass `info.hw` as their `hw`.

| `info` key | Contents |
|---|---|
| `hw` | the production hardware of the model's active configuration set |
| `solver` | the solver, fixed step and the design's explicit sample times that were checked; `stop_time`: the model's StopTime as written (null when the file omits it; a simulation request gives `nsteps` / `stop_time` itself) |
| `inport_types` | the data type of every Inport that declares one (use it for the inputs) |
| `assumed_defaults`, `assumed_values` | parameters the file leaves out and the values used (a saved model omits a parameter that has its default) |
| `ignored_parameters` | parameters in the file that the engine does not use, including a subsystem's own parameters; check them |
| `origin` | for every design block, the file part, SID and model path |
| `workspace_sources` | From Workspace blocks read as Inports, with their variable names |
| `scope`, `parts`, `blocks`, `lines`, `has_block_defaults`, `simulink_defaults_filled` | bookkeeping |

`design.port_types` lists data types declared on ports of flattened subsystems (a typed Inport or Outport inside a
subsystem): a port never converts, so fxgraph checks after compiling that the signal has that type, and reports
`TYPE_MISMATCH` otherwise. When such a port is driven by a design Inport, its type is also in `info.inport_types`.

What is read and what is refused:

- Sample times (and `FixedStep`) may be numeric literals or plain arithmetic on them (`1/30.72e6`); a workspace
  variable or function is refused.
- The contents of MATLAB Function blocks and charts (line endpoints written `<SID>::<N>`) are not read.
- Only `FixedStepDiscrete` solver models (`FixedStep` a number or `auto`); the active configuration set
  (`configSetInfo.json`) is the one read. Several sample times make a multirate design; a sample time `0` on a
  discrete block, or an offset where none is supported, is refused.
- Refused: commented-out blocks, variant subsystems, data type override (other than `UseLocalSettings` / `Off`),
  enable / trigger / state ports, block types and parameter values that are not supported (see
  [BLOCKS.md](BLOCKS.md)), parameter expressions other than those below, block names containing `/`.
- A parameter the file leaves out has the block type's built-in default (what Simulink omits when saving), which can
  differ from the library dialog's: Data Type Conversion rounds toward `Zero` and saturates, Switch / Multiport
  Switch / Abs saturate, a Multiport Switch stops on an out-of-range control (`DiagnosticForDefault: Error`), a
  MinMax has one input, a Discrete FIR's output type is `Inherit: Same as input`, a Constant's value is 1, a Gain's 1,
  a Saturation's limits +-0.5, a Lookup Table's internal rule priority `Precision`.
  `info.simulink_defaults_filled` lists every such parameter.
- Numeric parameters may be literals, plain arithmetic on literals (`2^14`, `1/3`, `[1 2]*0.5`, `[1 2 3]*2+1`), or names of
  **mask parameters of the enclosing subsystems** and of **model workspace variables saved in the file**
  (`simulink/modelWorkspace.mxarray`, real double scalars and rows): `Gain: K`, `Value: K/2`, a lookup `Table: h*40`.
  A colon range may take part in the arithmetic (`[1:4]/8`) when every element is an exact double. `nextpow2(x)` (the
  smallest integer p with 2^p >= |x|, 0 for 0; `nextpow2(N-1)`) is evaluated for a scalar double argument; a list argument is refused. A name is looked up
  from the nearest mask outwards, then in the model workspace; a mask value may itself name an outer mask parameter.
  Only Edit mask parameters of masks without initialization code are read (tunable or not). A name defined only in the base
  workspace is not in the `.slx`, so it is refused. `info.named_parameters` lists every parameter read this way
  (as written, value used); `info.unresolved_names` says why other names were not.
- With `scope`, the surroundings of the subsystem are checked too: its own SubSystem block and the subsystems around
  it (commented, variant, own sample time, data type override, conditionally executed or iterated), and the blocks
  its inputs are computed from (their sample times). When every input is driven directly by a block with its own
  sample time and these differ or carry an offset, the subsystem is a multirate design and the sample times are
  carried onto its Inports. Inputs computed through From / Data Store Read outside the scope are refused.

### Model Reference blocks

A Model block (Model Reference) is read as the referenced model's content placed under the block's name, like a
subsystem, within the recorded shapes below (in the recorded Model Reference models the results equal the subsystem's bit for bit, and
every instance keeps its own states). A referenced model is not in general the same as a subsystem: it is compiled on
its own, has its own name scope and runs at its own fixed step -- the differences the reader knows are enforced below;
anything else is not claimed to match.

- The referenced model is looked up as `<ModelName>.slx` in the directory of the model being read; if it is not
  there, the read is refused (`FILE_NOT_FOUND`).
- Every root Inport of the referenced model must declare its data type (the referenced model is compiled on its own;
  Simulink refuses an inherited one with InputPortDataTypeMismatch), and the declared type must equal the type of the
  signal driving the port (a mismatch is refused as `TYPE_MISMATCH`).
- A model argument takes the instance's value; the instance must give every argument (an argument left at the
  referenced model's default is not supported: `UNSUPPORTED_PARAM`). A parameter equal to the argument's name, or a
  mask value naming it, takes the instance's value, unless a mask parameter of the same name inside the referenced
  model is nearer.
- Names inside the referenced model resolve in its own masks and its own model workspace; the parent's masks and
  workspace are not visible there (Simulink: BlkParamUndefined).
- A root Inport of the referenced model with explicit `PortDimensions` is refused (the dimension check against the
  driving signal is not modelled).
- A Model block in a model with more than one rate (a multirate block, or an input at another period, also one
  coming from outside the scope read) is refused: Simulink runs the referenced model at its own fixed step, which the
  inlined read does not model.
- A root Inport of a referenced model left at PortDimensions -1 is scalar there (the model is compiled on its own); a
  vector driving it is refused as Simulink refuses it.
- Goto tags do not cross a model boundary: a From inside a referenced model whose tag only matches a Goto of the
  parent is refused (Simulink runs it with the From giving 0).
- A design range (OutMin / OutMax) on a subsystem's or a referenced model's port is refused, as on any block.
- Nesting is limited to 8 levels.

### write_types

Writes data-type parameters into a copy of the model and returns:

```json
{"status": "ok", "out": "runs/model_opt.slx", "blocks_written": 1, "params_written": 2,
 "written": {"Q0_a1": {"OutDataTypeStr": {"old": null, "new": "fixdt(1,18,12)"}, "ParamDataTypeStr": {...}}},
 "type_params_not_written": {}, "other_keys_not_written": {}, "warnings": [],
 "file_sha256": "...", "out_sha256": "..."}
```

- `blocks` uses the design block names of `read` (with the same `scope`). An fxopt `solution.blocks` can be passed
  as it is: only data-type parameters are written; other keys are listed in `other_keys_not_written`, other
  `*DataTypeStr` keys in `type_params_not_written`. A change of rounding or saturation made in fxsim is therefore
  not written back: make it in the model.
- Written parameters: `OutDataTypeStr`, `ParamDataTypeStr`, `AccumDataTypeStr`, `CoefDataTypeStr`,
  `ProductDataTypeStr`, the lookup table types (`BreakpointsForDimension1DataTypeStr`,
  `BreakpointsForDimension2DataTypeStr`, `TableDataTypeStr`, `IntermediateResultsDataTypeStr`,
  `FractionDataTypeStr`), each only on a block type that has it (`BAD_PARAM` otherwise). Values: `fixdt(s,w,f)`,
  `int8` .. `uint32`, `boolean` or a plain `Inherit: ...` string (`BAD_TYPE` otherwise).
- An unknown block name is `BAD_PARAM`. A request that writes nothing succeeds with a warning (the output is a copy
  of the input).
- `out` must lie inside the tree directory (real path, symbolic links resolved), not in its version-control or
  reference-data directories, and
  not be the input. An existing file is replaced only under `runs/` (a retried request writes the same bytes);
  anywhere else it is `BAD_PARAM`.
- Only the Block elements that `read` resolved are edited; every other part of the archive is copied byte for
  byte. The edited XML parts are re-serialised (attribute order and whitespace may differ from the original).

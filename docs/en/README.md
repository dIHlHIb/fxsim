# fxsim documentation (English)

fxsim simulates discrete, fixed-step, fixed-point block diagrams and chooses word lengths. Its supported domains are based on bit-for-bit comparisons with Simulink R2026b. Every tool reads one JSON request and writes one JSON result. It is meant to be driven by an
agent: no GUI, no interaction, stable error codes, the same output for the same input.

| Page | Contents |
|---|---|
| [../../skill/fxsim/QUICKSTART.md](../../skill/fxsim/QUICKSTART.md) | **start here**: five common pitfalls, one runnable request per common block, the command chain to a Simulink PASS / FAIL, refusal codes |
| [Quick reference](../../skill/fxsim/SKILL.md) | a runnable graph, common block templates, input binding, export and native verdict |
| [Worked examples](WORKED_EXAMPLES.md) | feedback, ports, clocks and word lengths |
| this page | ground rules shared by every tool, errors, the usual workflow |
| [AGENT_GUIDE.md](AGENT_GUIDE.md) | editing an exported FIR model and designing from scratch, choosing inputs, reading results, pitfalls |
| [GRAPH.md](GRAPH.md) | `engine/fxgraph`: design JSON, simulation, diagnostics, cost, the block catalogue |
| [BLOCKS.md](BLOCKS.md) | every block type with its parameters and defaults (generated from the engine) |
| [HDL_BLOCKS.md](HDL_BLOCKS.md) | the settings accepted for HDL RAM, HDL Counter, Delay with ports, DSP HDL FIR / NCO (generated from the engine) |
| [MATLAB_FUNCTION.md](MATLAB_FUNCTION.md) | the MATLAB Function subset: statements, calls, fixed-point rules, properties read from a `.slx` |
| [OPTIMIZER.md](OPTIMIZER.md) | `engine/fxopt` (choose word lengths), `engine/fxcert` (check a given design) and `engine/fxapprox` (design a lookup table for a function) |
| [SLX.md](SLX.md) | `engine/fxslx`: read a Simulink `.slx`, write chosen types back |
| [EXPORT.md](EXPORT.md) | `engine/fxexport`: build a new Simulink model from a design, with a one-command bit-exact check in MATLAB |
| [ENGINE.md](ENGINE.md) | semantics, execution paths (C core / Python), limits, environment variables, determinism |

The release includes the engine, reference arithmetic, documentation and agent skill.

## Ground rules

- **The tree** is the directory holding `engine/fxgraph`. Run tools from anywhere: `python3 -I engine/<tool>
  < request.json`. Only the Python standard library and a C core are used; if a result says `CORE_UNAVAILABLE`,
  run `sh engine/build.sh` once (system C compiler).
- **Requests come on standard input.** fxgraph, fxcert and fxslx take no command-line arguments (any argument is
  a `BAD_PARAM` error at `where: "argv"`, exit code 2). fxopt also accepts the request file path as its one
  argument. A request may be pretty-printed over several lines; fxgraph and fxopt also take a JSONL batch (several
  requests, one JSON object per line) and answer one line each.
- **Exit code** 0 with `"status": "ok"`, otherwise 1 (2 for argument errors) with `"status": "error"`.
- **Relative paths** in a request (fxslx `path` / `out`, fxopt `log_dir`) are taken from the tree directory,
  whatever the current directory. Paths in results (`full_result`, `run.run_dir`, `out`) are relative to the tree
  directory too. Absolute paths work everywhere.
- **Output locations.** Use `runs/` for JSON tool outputs. The MATLAB-style batch interface resolves relative file names against the caller’s working directory. Binary file writes and flat model saves use the caller’s filesystem permissions and can write outside the package.
- **Deterministic**: the same request gives the same result, apart from timing fields (`timing_ms`, `run`). Retrying
  is safe; processes share no state, so many can run at once.
- **Strict requests**: an unknown request field, an unknown field of the design, of a line or of `error`, an unknown
  block parameter or a malformed value is an error, never silently ignored. Parameters that Simulink accepts but that do not change the result are listed in
  `ignored_parameters`.
- **Omitted parameters** take the default of a block freshly placed from the Simulink library; every result lists
  the ones the design relied on in `assumed_defaults`. Read that list: library defaults are sometimes surprising
  (Relational Operator `<=`, Lookup Table extrapolation `Linear`).
- **Target hardware** (`hw`, the same field in every tool): omitted means the host, `Intel->x86-64 (Windows64)`;
  `{"ProdHWDeviceType": "ASIC/FPGA"}` for FPGA / ASIC targets. Inherited types differ between the two, so give the
  same `hw` to every call.
- **Parameter overflow** (`hw.ParameterOverflowMsg`, Simulink's diagnostic of the same name): `"error"` (default,
  as in a new Simulink model) refuses a parameter that does not fit its type -- one LSB or more below the minimum,
  or more than one LSB above the maximum -- with `PORT_TYPE_INVALID`; `"warning"` / `"none"` saturate it to the
  type's bound instead. Either way, `parameter_quantization` in the result lists every parameter not used as
  written. Exported packages support the default only.
- **Tasking mode** (`hw.SolverMode`): `SingleTasking` (the default here, and what exported packages set) or `Auto`, a
  new Simulink model's default, which runs a multirate model multitasking. Under `Auto` an Upsample's output comes one
  slow period later, and a line joining two different sample times (period or offset) without a rate-changing block
  is rejected as Simulink rejects it; it is answered for the block types recorded with `Auto`
  (`engine/auto_domain.json`). `"MultiTasking"` is refused. Single-rate designs are the same in every mode.
- **Coverage and refusals.** Some blocks accept exact recorded parameter tuples. Others accept combinations inside swept parameter ranges. A sweep checks selected points and may include combinations that have never been tested together. See [HDL coverage](HDL_BLOCKS.md) for examples. Do not work around a refusal by computing values yourself.

## Errors

```json
{"status": "error", "code": "PORT_TYPE_INVALID", "where": "design.blocks.L.ExtrapMethod",
 "message": "Simulink rejects linear extrapolation on fixed-point signals (Simulink:blocks:OnlyExtrapFloatAndHalf); set ExtrapMethod to Clip -- note: ExtrapMethod was left out, so the library default 'Linear' was used",
 "input_sha256": "...", "refusal_kind": "simulink_rejects", "next_step": "..."}
```

`where` is a path into the request. Every error of every tool carries `refusal_kind` and `next_step`:

| `refusal_kind` | Codes | Meaning |
|---|---|---|
| `request` | `BAD_JSON`, `MISSING_FIELD`, `BAD_PARAM`, `BAD_TYPE`, `BAD_INPUT`, `BAD_CONNECTION`, `UNKNOWN_BLOCK_TYPE`, `ALGEBRAIC_LOOP`, `LIMIT_EXCEEDED`, `TIMEOUT`, `OOM`, `FILE_NOT_FOUND`, `BAD_SLX`, `BAD_MATLAB`, `INFEASIBLE`, `NO_SOLUTION`, `REFERENCE_NOT_FINITE`, `LOG_DIR_EXISTS`, `START_TYPE_NEEDED` | The request is wrong (or asks for more than is possible) at `where`. Fix it and retry. |
| `simulink_rejects` | `TYPE_MISMATCH`, `PORT_TYPE_INVALID`, `SIM_DIAGNOSTIC` | Simulink rejects this too (or stops the run). Change the types / inputs named in the message. |
| `not_supported` | `UNSUPPORTED`, `UNSUPPORTED_BLOCK`, `UNSUPPORTED_PARAM`, `UNSUPPORTED_RATE`, `UNSUPPORTED_ROUNDING`, `UNSUPPORTED_WIDTH`, `UNSUPPORTED_MATLAB`, `TYPE_UNRESOLVED`, `UNSUPPORTED_EXPORT` | Not verified against Simulink, so not answered. Use a supported form (the message says which); otherwise report `where` and `message` as a limitation. |
| `defect` | `INTERNAL`, `CORE_FAILED`, `CORE_UNAVAILABLE`, `VERIFY_FAILED`, `UNRELATED_FAILURE` | A defect of fxsim. Do not use any number from that call; report `input_sha256` with the request. After `CORE_FAILED`, `"engine": "python"` usually works. |

Errors are also appended to a log under `runs/` (`graph_errors.jsonl`, `fxopt_errors.jsonl`, `cert_errors.jsonl`, `slx_errors.jsonl`).

## The usual workflow

1. Write the design JSON ([GRAPH.md](GRAPH.md)) or read it from a model ([SLX.md](SLX.md)).
2. Simulate with fxgraph, `"diagnostics": true, "cost": true`; read `summary` first.
3. Fix what the diagnostics show (wraps, saturations, defaults you did not mean).
4. Choose word lengths with fxopt: representative inputs and/or input ranges, an error tolerance, optional per-block
   word-length limits and a hardware cost model ([OPTIMIZER.md](OPTIMIZER.md)).
5. Read the plan's `summary`, `warnings` and `guarantees`; never deliver a plan with `ERROR_BOUND_VIOLATED`.
6. Overlay `solution.blocks` on your design (keep your lines) and, if you like, check it independently with fxcert.
7. For a model read from `.slx`, write the chosen types into a copy of it with `fxslx write_types`.
8. To verify in Simulink, export the design with fxexport and run the check script it writes: one MATLAB command, PASS / FAIL / INCOMPLETE ([EXPORT.md](EXPORT.md)).

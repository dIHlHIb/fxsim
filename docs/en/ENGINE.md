# The engine: semantics, execution paths, limits

What happens inside `engine/fxgraph` (and inside fxopt / fxcert, which use the same engine), and the settings that
control it. Ground rules: [README.md](README.md); the request: [GRAPH.md](GRAPH.md).

## Semantics

A design is a discrete, fixed-step, single-tasking block diagram, simulated as Simulink does with the
`FixedStepDiscrete` solver.
Multirate designs follow Simulink's `SingleTasking` mode by default (`hw.SolverMode`, see README); `Auto` (multitasking
for a multirate model) is simulated for the recorded block types, with an Upsample's output one slow period later and
direct connections between different sample times rejected; `MultiTasking` is refused.

1. **Compile** (once per request). Inport types come from the input records. Explicit types are taken as written;
   inherited types follow Simulink's rules for each block (internal rule, same as input, same as first input, ...)
   on the chosen hardware (`hw`: the host's long word is 32 bits wide for the internal rule, ASIC/FPGA has no native
   word). A cycle of inherited types is resolved as Simulink does, starting from a block of the loop with a known
   input; when Simulink would report a data type mismatch, the engine reports `TYPE_MISMATCH`, and when the result
   would depend on where the resolution starts, `UNSUPPORTED`. Parameters (gains, limits, initial conditions,
   coefficients, tables) are then quantised into their types.
2. **Schedule**. Blocks are ordered by data dependence; a Unit Delay / Delay outputs its state, so it breaks a
   loop. Ties are ordered by name, so the order is deterministic (`schedule` in the result). A loop without a delay
   is `ALGEBRAIC_LOOP`. A Discrete-Time Integrator (Forward Euler, the only method accepted) has no direct feedthrough
   either: a loop closed through it runs, the integrator being updated after its input and read as its state.
3. **Run**. At each step every block computes its output in schedule order with exact integer arithmetic on stored
   integers: products and sums in their accumulator types, rounding (`Floor`, `Ceiling`, `Zero`, `Nearest`, `Round`,
   `Convergent`, `Simplest`) and overflow handling (wrap, or saturate with `SaturateOnIntegerOverflow: "on"`) exactly
   where the block applies them. State blocks update at the end of the step.

Multirate designs are lowered to the base step (the greatest common divisor of the periods): a slow block runs on
the steps of its own sample hits and holds its output in between, exactly as Simulink's sample-and-hold semantics.
Outputs are reported at their own sample times.

Only behaviour that has been recorded against Simulink R2026b is answered; every other configuration is refused
with `UNSUPPORTED` and a reason. The refusals are part of the contract, not missing features to work around.

## Execution paths

| Path | Used when | Notes |
|---|---|---|
| C core (`engine/fxgraph_core`) | `engine: "auto"` (default) or `"c"`, single-rate, no diagnostics, every intermediate fits 126 bits | fast; `"c"` on a design it cannot take is `UNSUPPORTED_WIDTH` |
| Python path | `engine: "python"`, diagnostics, multirate designs, wider intermediates | exact at any width up to 4096 bits |

Both paths give bit-identical results; `summary.engine` / `metrics.engine` say which one ran. `CORE_FAILED` is a
defect of the C core: `"engine": "python"` answers the same request.

The C core is built by `sh engine/build.sh` (the system C compiler, C99, no other dependencies). Without it, `auto`
falls back to the Python path; `CORE_UNAVAILABLE` (fxopt) means the core is needed and missing.

## Limits

| Limit | Value | When exceeded |
|---|---|---|
| word length of any type | 1..4096 bits (signed types at least 2) | `BAD_TYPE` / `UNSUPPORTED_WIDTH` |
| blocks per design | 100,000 | `LIMIT_EXCEEDED` |
| steps per run | 10,000,000 | `LIMIT_EXCEEDED` |
| time per request | `FXGRAPH_TIMEOUT_S`, default 600 s | `TIMEOUT` |
| memory per process | `FX_MEM_BYTES`, default 4 GiB | `OOM` |

Individual blocks have narrower limits. For example, MinMax supports input, output and ideal common types up to 128 bits. A refusal identifies the type or width that exceeded the supported range.

## Environment variables

| Variable | Effect |
|---|---|
| `FXGRAPH_TIMEOUT_S` | per-request time limit of fxgraph |
| `FX_MEM_BYTES` | per-process memory limit of every tool |
| `FXOPT_WORKERS` | caps fxopt's default number of evaluation processes |
| `FXGRAPH_ERRLOG`, `FXGRAPH_RUNLOG`, `FXGRAPH_RESULTS` | paths of fxgraph's error log, run log and the directory of `detail: "summary"` result files (defaults under `runs/`; `/dev/null` turns a log off) |

## Determinism and logs

- The same request gives the same result on every run and machine, apart from timing fields. fxopt's result
  additionally depends only on its own request fields (`settings.non_default` lists any non-default internal
  setting; it should be empty), not on the number of processes; `deterministic: false` marks a result cut short by
  a time limit.
- Requests are independent: no state is shared between processes, so any number may run at once.
- fxgraph appends one line per run to `runs/graph_runs.jsonl` (status, request hash, time, the summary) and one per error to
  `runs/graph_errors.jsonl`; the other tools keep their own error logs under `runs/`. The logs are for diagnosis
  only; no tool reads them back.

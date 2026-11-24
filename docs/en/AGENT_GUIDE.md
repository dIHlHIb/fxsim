# Agent guide: from a model to verified word lengths

**Agents: read the skill first** - [../../skill/fxsim/SKILL.md](../../skill/fxsim/SKILL.md) is the daily workflow in
one page (request fields, limits, how to read results, when to stop verifying, common mistakes), with runnable
examples in `skill/fxsim/examples/`. Use the FIR export below to learn how to read and change a saved model.
Reference pages: [README.md](README.md) (ground rules, errors), [GRAPH.md](GRAPH.md), [OPTIMIZER.md](OPTIMIZER.md),
[SLX.md](SLX.md), [BLOCKS.md](BLOCKS.md), [ENGINE.md](ENGINE.md).

## Read and edit an SLX model

From the package root, build the engine once with `sh engine/build.sh`. The first command below creates an eight-tap FIR model from the supplied design and inputs. It needs a new output directory at `runs/export/fir8_opt`. If that export already exists from this example, start with the second command.

```sh
python3 -I engine/fxexport < skill/fxsim/examples/05_export_fir.json
python3 -I engine/fxslx < skill/fxsim/examples/06_slx_list.json
python3 -I engine/fxslx < skill/fxsim/examples/06_slx_read.json
python3 -I engine/fxslx < skill/fxsim/examples/08_slx_write_types.json
```

The list result identifies the `DUT` subsystem. Reading that scope returns three blocks and their connections, plus compiled-model settings available in the file. The write request changes the FIR block’s output type to `fixdt(1,16,14)` in a copy. Read the copy and check the value of `design.blocks.F.OutDataTypeStr`.

```sh
echo '{"op":"read","path":"runs/fir8_typed.slx","scope":"DUT"}' | python3 -I engine/fxslx
```

For a model of your own, list its subsystems and pass the desired path as `scope`. Supply representative input sequences to fxgraph and fxopt. Apply the selected parameters from `solution.blocks` with `write_types`. Verify the edited design against your inputs and tolerance and use the native checker for the final export. [SLX reference](SLX.md) · [Verification and export](EXPORT.md)


## Design from scratch

1. Write the design JSON ([GRAPH.md](GRAPH.md)); take parameter names from [BLOCKS.md](BLOCKS.md) or
   `{"catalogue": true}`. Write every parameter that matters explicitly: library defaults are sometimes surprising
   (Relational Operator `<=`, Lookup Table `ExtrapMethod: Linear`, multirate blocks frame-based).
2. If the design is floating point, write its types as `"double"` and give fixed-point Inport types; fxopt converts
   it ([OPTIMIZER.md](OPTIMIZER.md)).
3. Simulate with `"diagnostics": true, "cost": true` and read `summary`: `blocks_with_assumed_defaults`,
   `diagnostics.first_event`, `hwbits`.
4. Optimise, then overlay `solution.blocks` on your design (keep your lines) and simulate the result.
5. Check it with fxcert at the user's tolerance (single-rate designs), then export it with fxexport and run the
   package's `check_batch_command` in MATLAB ([EXPORT.md](EXPORT.md)).

## Choosing inputs and the tolerance

- Inputs should be representative of the real signal. For a model with its own Sine Wave source, generate the same
  sequence: `{"kind": "sine", "amplitude": A, "omega": Frequency, "n": N, "type": {...}}` at sample time 1.
- Several typical signals: `input_sets` (each must meet the tolerance). Any signal within an amplitude:
  `input_ranges` (an overflow proof over every sequence inside, and a worst-case error search).
- The tolerance is an absolute error on each Outport (`error.max_abs`, or per outport `error.outports`). The
  reference is the same design in double precision.
- Limit one block's word lengths with `block_wordlengths: {"Out": [2, 16]}` rather than `fixed_blocks` (fixed
  blocks keep their given types, and the guarantees are only as good as those types).

## Reading results: what to trust

| You see | It means |
|---|---|
| `overflow: "bounded"` | proved: no wrap or saturation for any input inside the ranges, from the stated initial state -- except in the Sums listed in `guarantees.overflow.wraps_intended_in` (an integrator-comb chain that wraps by design; the values leaving the chain are proved) |
| `overflow: "unknown"` | not proved, not disproved; run fxcert, or narrow the ranges |
| `overflow` / `error: "violated"` | a real counterexample, given in the result; never deliver this plan |
| `error: "sampled"` | within tolerance on the judged sets only |
| `error: "exhaustive"` | within tolerance for every input inside the range (every stored value was tried) |
| `summary.max_error_over_tolerance` | the headroom: the worst ratio over the judged sets and the error search's last round (`max_error_over_tolerance_from`); `binding` is only the judged set with the largest ratio. Above 0.9, rerun with the tolerance x 0.9 or add sets |
| zero diagnostics events in fxgraph | none on these inputs; not a proof |
| `hw_estimate` | a model's prediction, not a synthesis result |

When you simulate a `bounded` plan, its wraps (fxgraph `summary.diagnostics.blocks_wrapped`, `total_wraps`) are
expected exactly when every wrapped block is in `wraps_intended_in`; a wrap in any other block is a real overflow.
fxcert re-proves such chains from the design and gives the same exception (`summary.wraps_intended_in`,
`overflow.modular_regions[].proven`); with `"modular": false` it counts every wrap as overflow (see OPTIMIZER.md).

## Pitfalls

- Give the same `hw` to every call; inherited types differ between host and ASIC/FPGA.
- A refusal (`not_supported`) is a statement that the configuration was never verified; change the design as the
  message says, or report the limitation. Do not hand-compute the numbers.
- `nsteps` can only shorten the inputs. Multirate designs take `stop_time` and per-Inport sample counts.
- fxopt's `time_budget_s` covers the optimisation only; the error search comes on top. `time_limit_s` stops the request
  between stages (no new search round or re-optimisation that would end after it) but does not interrupt a running
  stage, so give it some slack; and `workers` (or `FXOPT_WORKERS`) on a shared machine.
- Results are deterministic: a retry gives the same answer, so retry only after changing the request.

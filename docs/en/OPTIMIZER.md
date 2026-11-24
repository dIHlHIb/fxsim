# fxopt and fxcert: choosing and checking word lengths

- `python3 -I engine/fxopt < request.json` (or `engine/fxopt request.json`) chooses the data types of a design so
  that the error against an ideal reference stays within a tolerance at the lowest hardware cost, and states what is
  guaranteed about overflow and error.
- `python3 -I engine/fxcert < request.json` checks a design as given: an overflow proof over input ranges and the
  error on given input sets. It changes nothing.

Ground rules and errors: [README.md](README.md). Design JSON: [GRAPH.md](GRAPH.md).

## fxopt request

```json
{"design": {"blocks": {...}, "lines": [...]},
 "inputs": {"x": {"kind": "sine", "amplitude": 0.9, "omega": 0.05, "n": 2048, "type": {"s": 1, "wl": 16, "fl": 14}}},
 "error": {"max_abs": 0.0009765625},
 "block_wordlengths": {"F": [2, 18]},
 "time_budget_s": 60, "workers": 2}
```

| Field | Meaning | Default |
|---|---|---|
| `design` | the design; its type parameters are the starting point and may be `double` (a floating-point design to convert) | required |
| `inputs` | one input set: per Inport a sequence (`{"s","wl","fl","ints"}` / `{"type", "ints"}`) or a generator (below) | one of `inputs`, `input_sets`, `input_ranges` |
| `input_sets` | a list of 1..64 input sets, each judged on its own; an entry `{"inputs": {...}, "tolerance_scale": k}` judges that set (and its variants) against k x the tolerance; `{"inputs": {...}}` alone is the same wrapper at 1 x (fxcert reads both forms too) | |
| `stress_tolerance_scale` | k >= 1: the stress sets derived from the input ranges and the error search (its search and its counterexamples) are judged against k x the tolerance; `inputs` / `input_sets` stay at 1 x. For a requirement such as "within 2^-12 on the signal, within 16 x that at full scale" | 1 |
| `input_ranges` | per Inport `{"type": {"s","wl","fl"}, "min": lo, "max": hi}`: any sequence inside the range | from the inputs (below) |
| `input_variants` | extra sets made from each given set: `"negated"`, `"reversed"`; `[]` for the given sets only | both |
| `error` | `{"max_abs": t}` (every Outport), `{"outports": {"y": t}}` (only those), or both (listed ones by their own tolerance, the others by `max_abs`) | `max_abs` 2^-10 |
| `reference` | `"ideal"`: the same design in double precision; `{"kind": "design"}`: the design as given | `"ideal"` |
| `wordlengths` | the word lengths the optimiser may choose (each 2..62) | 6, 8, ..., 32 |
| `block_wordlengths` | `{block: [min, max]}`: limit every type of that block | |
| `fixed_blocks` | blocks whose types are kept as given | |
| `allow_unsigned` | unsigned types where a signal is never negative | `true` |
| `cost` | the objective: `{"hwbits": 1}`, `{"hwbits": a, "mult_area": b}` (`mult_area`: sum of data word x coefficient word over the multipliers -- each Gain, each Product step, each FIR tap; a symmetric FIR has one multiplier per coefficient pair, fed by the tap sum), `{"fpga_xc7": 1}` (DSP48E1 first, then LUT + FF) or `{"asic_gates": 1}` | `{"hwbits": 1}` |
| `hwbits_slack` | with a hardware cost model: how much more hwbits than the hwbits-optimal plan is allowed (0..1) | 0.05 |
| `initial_state` | `"design"` (the design's initial conditions) or `"any"` (any state value) for the overflow proof | `"design"` |
| `range_test` | `{"length": n, "seed": k}` of the generated stress sequences in range mode | 4096, 0 |
| `ranges_from_inputs` | `false`: no input ranges from the inputs' envelope -- and no overflow proof; keep it on unless a wrap-around integrator cannot be range-proven (see "Designs fxopt's range mode refuses"). A tolerance the stress inputs cannot meet is relaxed with `stress_tolerance_scale`, not by switching this off | `true` |
| `error_search` | `false`: no worst-case error search | `true` |
| `hw` | target hardware | host |
| `time_budget_s` | budget for the optimisation itself (the error search comes on top) | 85 |
| `time_limit_s` | a stop condition for the whole request (>= `time_budget_s`): checked between stages, it does not interrupt one (see below) | none |
| `reoptimisation` | after an error-search counterexample: `"cold"` re-runs the full search (at least as long as the first pass), `"warm"` starts from the previous plan, widened just enough. Warm is usually much faster on large designs, but on some designs it gives a costlier plan than cold, so it is not the default. `"auto"`: warm, then a cold one as well when the warm plan costs more than 2 % over the plan before the counterexample, keeping the cheaper; the last re-optimisation the loop allows is always cold | `"cold"` |
| `modular` | `true`: integrator-comb chains (CIC built from Sums and delays, also with a Downsample between the integrators and the combs) are typed for exact wrap-around, see below; `false`: every signal is sized not to overflow | `true` |
| `workers` | evaluation processes computing at once; `FXOPT_WORKERS=N` in the environment caps the default | available cores |
| `max_rounds`, `seed` | refinement round limit; seed (recorded; results do not depend on it) | 200, 0 |
| `log_dir` | a new directory under `runs/` for the candidate log (an existing one gives `LOG_DIR_EXISTS`) | `runs/fxopt/<hash>/<attempt>/` |

### Inputs

- Stored-integer sequences as in fxgraph, or a sine generator
  `{"kind": "sine", "amplitude": A, "omega": w, "n": N, "phase": p, "offset": c, "type": {"s","wl","fl"}}`:
  stored value floor((c + A sin(w n + p)) 2^fl) for n = 0 .. N-1; `w` and `p` in radians per sample (for a Simulink
  Sine Wave at sample time 1, `w` is its Frequency); `phase` and `offset` default to 0.
- `inputs` and `input_sets` combine: `inputs` is the first set (label `inputs`), `input_sets` follow
  (`input_sets[k]`). Sets may have different lengths; the Inports of one set have the same length.
- Every Inport needs a fixed-point type, also when the rest of the design is `double`.
- **Range mode.** Without `input_ranges`, the ranges are the envelope of the given sets **including their
  variants** (the negated set makes a signed range about symmetric). In range mode the plan is proved free of
  overflow for every sequence inside the ranges where the proof succeeds, and stress sequences are added.

## fxopt result

The first key after `status` is `summary`:

```json
"summary": {"hwbits": 72, "baseline_hwbits": 96, "max_abs_diff": 0.00083, "tolerance": 0.001,
            "max_error_over_tolerance": 0.827, "overflow": "bounded", "error": "sampled",
            "warnings": ["ERROR_BOUND_SAMPLED"],
            "binding": {"outport": "y", "input_set": "range: random uniform", "error_over_tolerance": 0.827},
            "blocks_retyped": ["A", "S"]}
```

| Key | Contents |
|---|---|
| `solution.blocks` | every block of the design with the chosen types; use it as the new `blocks` with your own `lines` (also for multirate designs, where it lists every original block) |
| `solution.compiled_types` | the compiled type of every block in the plan |
| `hwbits`, `hwbits_by_block`, `hwbits_breakdown` | the plan's cost (counting as fxgraph's `cost.hwbits`) |
| `baseline` | `{"hwbits": ...}` of the design as given (`null` with a reason when it does not compile) |
| `max_abs_diff`, `max_error_over_tolerance`, `outports` | the plan's error; evaluate it using `max_error_over_tolerance` (<= 1 passes) |
| `summary.max_error_over_tolerance` | the larger of the judged input sets' ratio and the error search's last round on the final plan; `summary.max_error_over_tolerance_from` says which (the top-level field covers the judged input sets only) |
| `binding` | the constraint that binds: outport, input set, sample and error / tolerance. When the error search found an input over the tolerance, it points to that input: `source: "error_search"`, `outport: null` (the ratio is the worst over the outports, each normalised to its tolerance), the set-based one under `judged_sets_binding` |
| `guarantees` | `contract` (what was assumed; `scaled_input_sets` lists the sets judged against tolerance x `tolerance_scale` / `stress_tolerance_scale` (above 1 relaxed, below 1 tightened)), `overflow`, `error`, each with `status` and `claim` |
| `error_by_input_set` | every input set x outport: `max_abs_diff`, `at_sample`, the set's own `tolerance` (scale included), `tolerance_scale` and `error_over_tolerance` -- `binding` is the largest ratio here |
| `error_search` | the worst-case error search: method, rounds (each with `worst_error_over_tolerance` and `worst_input`, the worst input that round found in fxgraph's inputs form -- replayable even when it is within the tolerance), counterexamples added |
| `warnings` | present only when there is something to say (below) |
| `auto_caps` | word-length caps the optimiser added itself to stay inside what the engine supports |
| `hw_estimate`, `mult_area_proxy`, `cost` | predicted FPGA / ASIC resources (not synthesised) and the objective used |
| `ranges`, `search`, `run`, `settings`, `time`, `deterministic` | the ranges used, search statistics, processes and timing, non-default settings; `deterministic: false` when a time limit cut the search |

**Guarantee statuses**

- `overflow.status`: `bounded` (proved for every sequence inside the ranges, from the stated initial state),
  `unknown` (not proved, and no input set overflowed; not a claim that an overflow happens), `violated` (an input
  inside the ranges overflows: `witness` names the input set, block, kind, count and first step; `observed_overflows`
  lists every wrap / saturation the plan produced on in-range sets outside the intended modular regions -- replay the
  set in fxgraph with `"diagnostics": true`), `sampled` (no ranges: observed on the given sets only). Where the range
  proof cannot fit a quantisation point, the optimiser widens that block within the allowed word lengths (also under
  `block_wordlengths`) and proves again; `search` / the proof list what it widened.
- `error.status`: `sampled` (within tolerance on every judged set; nothing claimed for other inputs), `exhaustive`
  (the error search tried every stored input value of the range on the final plan -- one input, a memoryless design
  or one whose only state is input-independent counters -- so the bound holds for every input inside the range),
  `violated` (an input inside the ranges exceeds the tolerance: `violation.counterexample_input`, in fxgraph's input
  form, replays it).

**Warnings**: `ERROR_BOUND_VIOLATED` (do not deliver this plan), `ERROR_BOUND_SAMPLED`, `OVERFLOW_POSSIBLE`,
`OVERFLOW_NOT_PROVEN`, `RANGE_MODE_FAILED` (the plan for the input envelope could not be made and the plan rests on the
sampled sets; the full reason is in the message and in `ranges.envelope_attempt_failed`), `BUDGET_EXHAUSTED`.

**INFEASIBLE** says, at the widest allowed plan, the error / tolerance of every input set and which ones miss it: when
only stress sets (`range: ...`) miss, `stress_tolerance_scale` or narrower `input_ranges` can help; when the given sets
themselves miss, a type the optimiser does not choose (a filter's internal type, a fixed block) or the tolerance must
change.

**Errors specific to fxopt**: `INFEASIBLE` (even the widest allowed word lengths miss the tolerance: widen
`wordlengths` / `block_wordlengths` or the tolerance; with `reference` `{"kind": "design"}` the design's own
explicit types -- parameters and signals -- are tried too, at their own word lengths, so the design itself is a
candidate; the message gives the smallest error any tried candidate reached and names a design type whose word length is
not in `wordlengths`), `NO_SOLUTION`, `REFERENCE_NOT_FINITE` (the double reference
reaches an infinite value, e.g. a division by zero), `LOG_DIR_EXISTS`, `UNSUPPORTED` "no data type to choose" (with
the reason: fixed blocks, multirate filter internals, blocks without a type parameter).

### Converting a floating-point design

Write the types to choose as `"double"` (`OutDataTypeStr`, `AccumDataTypeStr`, `ParamDataTypeStr`, ...), give the
Inports fixed-point types in the inputs, and keep `reference: "ideal"`: the reference is then the floating-point
design itself. `solution.blocks` is the fixed-point design.

### Multirate designs

fxopt optimises multirate designs (including the overflow proof). The internal types of FIR / CIC rate-change
filters are not decision variables; the type of a filter's input is kept when its internal types depend on it.

### What hwbits counts, and which bits fxopt chooses

| Block | Counted in hwbits | Chosen by fxopt |
|---|---|---|
| every block except Outport | its output word length (a Delay: x DelayLength) | yes where listed below, else it follows its input |
| Mux, Demux, Selector, Bus Creator / Selector and other pure wiring / routing blocks | 0: the vector is lowered to its elements and each element's blocks are counted, reported under the user block (`cost.by_block` / `hwbits_by_block` key `G`, with `parts` = the number of element / helper blocks summed); the routing itself costs nothing | -- |
| Gain | + parameter word | output and parameter types |
| Sum | + accumulator word | output and accumulator types |
| Product, Data Type Conversion, MinMax, Switch, Multiport Switch, Constant, Saturation, Abs | output only | output type |
| Discrete FIR | + coefficient x taps, product x taps, accumulator x (1 + states when the state type is 'Same as accumulator'); explicit state types x states | coefficient, product, accumulator and output types; the **state type is not chosen**: 'Same as accumulator' follows the accumulator, an explicit one stays as given (so it is a design choice worth checking) |
| 1-D / 2-D Lookup Table | + breakpoints x entries, table x entries; with linear point-slope interpolation also the intermediate and the fraction word: the type each parameter names -- written out, 'Same as output' (the output's word), or the inherited rule's result -- so one design written two ways costs the same | breakpoint, table, intermediate, fraction and output types |
| Unit Delay, Delay, Unary Minus, Shift Arithmetic, Discrete-Time Integrator (internal rule) | output | no: inherited from the input |
| Inport | its word length | no: the input type is given |
| FIR Decimation / Interpolation | their output + coefficient x taps, product x taps, accumulator, input delay line (taps - 1 input words) | no: internal types are not chosen; the input type a filter's internal types depend on is kept |
| CIC Decimation / Interpolation | their output + every integrator / comb section word + M x each comb's word (its differential delay) | no, as above |

`hwbits_by_block` in the result (and `cost.by_block` in fxgraph) gives the split per block.

What fxopt never changes: every block's rounding (`RndMeth`) and overflow mode (`SaturateOnIntegerOverflow`) stay as
the design gives them -- choose them before optimising. Word lengths are 2..62 bits. On ASIC/FPGA hardware a 1-D / 2-D
Lookup Table's input may be at most 7 fraction bits finer than its breakpoints (and no finer than the fraction type);
wider gaps are refused there, so a plan keeps the input and breakpoint types within that.

### Closing a "sampled" error bound

`error: "sampled"` with `ERROR_BOUND_SAMPLED` means the tolerance was checked on the judged sets only. A default
recipe that settles it in one or two runs:

1. Keep range mode (the default: the inputs' envelope, or explicit `input_ranges`) and `error_search` on. The search
   then looks for inputs inside the ranges that exceed the tolerance (on designs with loops or integrators it also
   tries long constant inputs) and re-optimises with what it finds; `error_search.status` says how it ended.
2. Give, besides the representative signal, `input_sets` with: a full-scale step of each sign, a slow ramp across
   the range, alternating maximum / minimum, and for integrating designs a small constant held for several thousand
   samples. `input_variants` already adds the negated and time-reversed copies.
   If the requirement allows the full-scale stress inputs a looser error (say 16 x), say so with
   `"stress_tolerance_scale": 16` (the generated stress sets and the error search) and, for your own stress sets,
   `{"inputs": {...}, "tolerance_scale": 16}` in `input_sets`; the representative signal stays at 1 x. fxcert takes the
   same `input_sets` entry form.
3. Keep a margin: if `max_error_over_tolerance` of the plan is above 0.9, re-run with the tolerance multiplied by
   0.9 (the reported figures are exact for the judged inputs, not a bound for others).
4. Check the plan independently with fxcert: the same `input_ranges`, the sets above plus a long random set inside
   the ranges. fxcert judges the error against the ideal (double-precision) design only; with `reference`
   `{"kind": "design"}` its error figures do not apply -- rely on fxopt's own per-set re-check, which uses the
   reference design. Stop when fxcert reports overflow `bounded` (with fxopt's intended wraps listed in
  `wraps_intended_in`) and every set within tolerance; if a set fails, add it to
   `input_sets` and run fxopt once more.

A one-input memoryless design (or one whose only state is input-independent counters) is searched exhaustively:
`error: "exhaustive"` is a bound for every input inside the range.

### Run time, progress and parallel runs

| Setting | Meaning |
|---|---|
| `time_budget_s` (default 85) | budget for the optimisation passes; the error search comes on top (a fixed amount of work, not cut by the budget) |
| `time_limit_s` | a stop condition, not a pre-emptive cap: no error-search round or re-optimisation starts that would end after it (estimated from the previous one), but a stage already running -- setup, the first optimisation pass (bounded by `time_budget_s`), the range proof, the final checks -- runs to its end, so the request can take longer (a 1 s limit took 3.6 s) |
| `workers` (default: the available cores; `FXOPT_WORKERS` caps it) | evaluation processes computing at once; results do not depend on it |

- Rough cost (one core, small designs): fxopt evaluates candidates on the C core, about 5-30 s for a FIR or biquad
  with up to 100,000 input samples and the default budget; each input set, variant and range stress set is one more
  evaluator process. fxcert simulates every input set on the Python path: about 0.2 ms per sample and set for an
  8-tap FIR (100,000 samples: about 20 s per set) -- keep certification sets to a few tens of thousands of samples.
- Several fxopt runs at once: keep the sum of their `workers` at or below the number of cores.
- Progress: a running fxopt writes `runs/fxopt/<request hash>/<attempt>/state.json` (or `<log_dir>/state.json`):
  `state` (running / done / error), `phase`, `elapsed_s`, refreshed at least every 10 s; the final result is
  `result.json` beside it.

### Why a plan can be wider than a hand-made design

- **The overflow proof costs bits in feedback loops.** In range mode (the default) fxopt sizes every signal so that
  no input inside the ranges can overflow it; a hand design sized on the simulated signals is narrower but carries
  no such claim. On a leaky integrator and a resonator: 114 / 158 hwbits with the proof, 108 / 133 with
  `"ranges_from_inputs": false` (overflow then only checked on the given inputs).
- **Wrap-around that cancels downstream** (an integrator before a comb, CIC style): without input ranges fxopt also
  tries types with fewer integer bits than the signal's simulated range and keeps them when every judged output
  stays within the tolerance; the result lists them in `search.msb_trimmed` and warns `WRAP_RELIED_ON`.

### Integrator-comb chains (CIC built from basic blocks)

A chain of Sums that only add or subtract (saturation off) and delays (initial condition 0, no reset or enable),
with a feedback loop inside -- the integrators and combs of a CIC filter -- is exact in two's-complement arithmetic
when every register has the same integer width: the integrators may wrap, the values leaving the chain stay right as
long as that width holds the largest value that leaves it. fxopt finds such chains itself (`"modular": true`, the
default) when every input into the chain is a fixed-point Inport and every signal leaving it has a finite impulse
response, and then:
- gives every Sum of the chain the same integer bits (sign included): enough for the proven range of every leaving
  signal -- the sum over the impulse response taps times the input's extreme (its `input_ranges` entry, else its
  type's full range) -- widened by the worst-case rounding error of the chosen fraction lengths (the Hogenauer
  bound: each pruned LSB times the leaving signal's response to an error at that Sum); one more integer bit when
  the search's plan needs it;
- lets the search choose each Sum's fraction length (output and accumulator): later stages keep fewer LSBs, as in
  Hogenauer pruning.
For a CIC with N stages, differential delay M and rate change R the integer width is the textbook one: input
integer bits plus N log2(RM), plus at most one bit for the pruning. The result lists the chains in
`modular_regions` (blocks, integer bits, chosen types, the proven range and rounding error bound of each leaving
signal, `proven`), `summary.modular_blocks`, and `guarantees.overflow.wraps_intended_in` names the Sums that wrap by
design. Range mode accepts such a chain: with `input_ranges`, or by default on the envelope of the given inputs,
the overflow proof takes the chain's leaving values from its own proof and the rest of the design as usual, so
`guarantees.overflow` can be `bounded` with the chain's wraps inside. With `"ranges_from_inputs": false` (no range
proof) the search may narrow the chain below its proof when every judged output stays within the tolerance
(`search.msb_trimmed`, warning `WRAP_RELIED_ON`), cheaper but checked on the given inputs only; the chain is then
reported with `proven` false.

By hand (or for a chain fxopt does not recognise, e.g. an input that is not an Inport): give every Sum of the chain
the same integer bits as above, list the chain's blocks in `fixed_blocks`, and keep the saturation off. Range mode
still refuses such a design (UNSUPPORTED: the loop does not settle), whether or not `input_ranges` is given or the
chain is fixed: run it with `"ranges_from_inputs": false` and no `input_ranges`. The price is the proof: the rest
of the design is sized on the given inputs only, so `guarantees.overflow` and `error` are `sampled` (warnings
`OVERFLOW_NOT_PROVEN`, `ERROR_BOUND_SAMPLED`) and `modular_blocks` is empty -- give input sets that reach the
extremes. (A 3-block chain fed from a Gain, 120 -> 104 hwbits this way, sampled.)

### Designs fxopt's range mode refuses

- **Wrap-around accumulators** outside an integrator-comb chain (an integrator meant to wrap whose value leaves
  without a comb, a phase accumulator, Free-Running counters): no overflow proof exists for them, so range mode refuses and fxopt falls back to the given input sets
  (`guarantees.overflow.status: "sampled"`, `ranges.envelope_attempt_failed` says why). Give representative
  `input_sets`, or `"ranges_from_inputs": false` to skip the attempt.
- **Loops closed through a Discrete-Time Integrator** (PI, leaky integrators) are handled by range mode: a limited
  integrator is bounded by its limits, an unlimited one is treated as a linear state; the overflow proof then
  covers them like any loop.

## fxcert request and result

```json
{"design": {...}, "hw": {...},
 "input_ranges": {"x": {"type": {"s": 1, "wl": 16, "fl": 14}, "min": -1, "max": 1}},
 "initial_state": "design",
 "input_sets": [{"x": {"s": 1, "wl": 16, "fl": 14, "ints": [...]}}],
 "error": {"max_abs": 0.001}}
```

- `input_ranges` are required (the overflow proof is over them). `input_sets` and `error` are optional; an input in
  a set may be written as in fxgraph (`{"s","wl","fl","ints"}`, so fxopt's `counterexample_input` replays as it is),
  as `{"type", "ints"}`, as `{"ints"}` (the type then comes from `input_ranges`) or as a sine generator.
- `error.outports` must be an object `{Outport: tolerance}`. With `max_abs`, boolean Outports are not judged and are
  listed in `error.outports_not_judged`; asking a tolerance for a boolean Outport is refused.
- Result: `summary` first (`overflow`, `error`, `overflow_witness` block / step, `max_error_over_tolerance`,
  `worst` outport / input set / sample, `sets_outside_input_ranges`, `outports_not_judged`), then `overflow`
  (`status` bounded / unknown / violated / unsupported, `claim`, `witness`) and `error` (`status` sampled /
  violated / not_checked, per input set and outport `max_abs_diff`, `at_sample`, `tolerance`, `within`).
- `error.input_sets[k].inside_input_ranges` is `false` when set k has samples outside the quantised ranges (e.g.
  floor(0.95 sin(.) 2^15) reaches -31130 while a range written as -31129/32768 stops at -31129). Its error figures
  are still exact for that sequence, but the overflow verdict does not cover it.
- Passing input sets never makes the overflow status `bounded`, and an overflow proof says nothing about the error.
- Wrap-around regions (`modular`, default `true`): fxcert finds the same regions fxopt reports in `modular_regions`
  (+ / - Sums and delays with a loop, e.g. a CIC's integrators and combs) and proves each one again from the design:
  every Sum signed with the same integer bits (output and accumulator; a wider Sum does not undo a narrower one's
  wrap), and every value leaving the region, widened by the
  rounding error bound, inside those bits. A proven region's wraps are intended and not counted: the status can be
  `bounded` and `summary.wraps_intended_in` lists the region's blocks; `overflow.modular_regions` gives each
  region's `proven` flag and leaving ranges. A region that fails the proof is reported with `proven: false` and its
  wraps count as overflow. Every wrap outside a proven region still counts. `"modular": false` counts every wrap;
  `initial_state` `any` turns the exemption off (the proof assumes the chain starts at rest).
- Multirate designs are not certified (`UNSUPPORTED`).

## fxapprox: a lookup table for a function

`python3 -I engine/fxapprox < request.json` designs a 1-D Lookup Table (linear interpolation, clip) that approximates
a function on an input range within an absolute tolerance, with as little storage as it finds: the breakpoints, the
number of points, the table values and the data types. The result's `block` is a Lookup1D ready to place in a
design (connect its input and output).

| Field | Meaning | Default |
|---|---|---|
| `function` | one of `sqrt`, `rsqrt`, `reciprocal`, `exp`, `log`, `log2`, `sin`, `cos`, `tanh`, `atan`, `sigmoid` (by name; no expression is evaluated) | `function` or `reference` |
| `reference` | `{"block": {...}}`: a Lookup1D as in a design (breakpoints, table, types, interpolation, extrapolation) to **compress**: the new table must stay within the tolerance of this block's own bit-exact output | `function` or `reference` |
| `input` | `{"type": {"s", "wl", "fl"}, "min": lo, "max": hi}`: the lookup's input type and range (with a `reference`, `min` / `max` may be left out: the whole input type) | required |
| `output` | `{"type": {"s", "wl", "fl"}}`: the lookup's output type | the reference's output type; for a function, its range at the largest allowed word length |
| `error` | `{"max_abs": t}` | required |
| `spacing` | breakpoint layouts to try: `"explicit"` (any values), `"even"` (first point and spacing), `"pow2"` (even with a power-of-2 spacing: the index is a bit slice, no divider) | all three |
| `wordlengths` | word lengths for the breakpoints and the table | 8 .. 16 |
| `rounding` | the block's rounding method | `"Floor"` |
| `time_budget_s` | stop trying further word-length pairs after this | 60 |

How it works and what is guaranteed:
- every candidate is simulated bit-exactly (the same Lookup1D as fxgraph) on **every** stored input value of the
  input type in `[min, max]`; `max_abs_error`, `at_input` and `error_over_tolerance` come from that run. A range
  with more than 2^20 input values is refused (UNSUPPORTED);
- breakpoints: for explicit spacing, segments as long as a double-precision estimate allows, then checked exactly
  (tightened and retried when the exact check fails); for even spacing, the fewest points found by bisection;
- table values are centred on each segment's deviation band (not f at the breakpoint), which roughly halves the
  interpolation error of a convex or concave function;
- the block saturates its output (an interpolated value just past the output range does not wrap); it interpolates
  in an intermediate type with at least the output's precision (not stored, so free in memory), signed for a falling
  table (the unsigned case is not recorded); a near miss is refined by centring the bit-exact error per segment;
- `memory_bits`: explicit spacing counts points x (breakpoint word + table word), even and power-of-2 spacing points
  x table word + 2 x breakpoint word (the first point and the spacing), as Fixed-Point Designer does;
- a `reference` the engine cannot simulate (an unsupported block setting) is refused with REFERENCE_UNSUPPORTED and
  the engine's reason;
- table values are clipped into the table type before the block is emitted (a centred value just past the type's
  range would stop a default Simulink model with a parameter overflow error; the stored values are the same);
- breakpoint types whose fraction length is 11 or more bits below the input's are not used: the engine refuses
  linear interpolation there (Simulink's fraction for such inputs is not recorded and differs by 1 LSB);
- the table is designed for the x86-64 host (there is no `hw` field: inherited lookup types differ on ASIC / FPGA
  targets, so check it with fxgraph on the target's `hw` before using it there). On ASIC / FPGA the block's input may
  be at most 7 fraction bits finer than its breakpoints (fxgraph refuses a larger gap there): fxapprox does
  not apply that limit, so for an ASIC target give an input type at most 7 fraction bits finer than the breakpoint
  types you allow, or convert the input (Data Type Conversion) to fewer fraction bits before the table;
- no table within the tolerance: `NO_SOLUTION` with the number of candidates checked, how many blocks the engine
  refused and why, and which word-length pairs were not tried and why; widen `wordlengths` or loosen the tolerance.
  The message says which case it is: "among the N candidates checked" when every word-length pair was searched,
  "before the time budget ran out" when the search stopped early (not a proof; raise `time_budget_s`). A
  quantisation step at or above the tolerance does not by itself rule a table out: values the types hold exactly
  give error 0, and every candidate is checked bit-exactly.

For a function approximation, try `{"function":"tanh","input":{"type":{"s":1,"wl":16,"fl":12},"min":-4,"max":3.999755859375},"output":{"type":{"s":1,"wl":16,"fl":14}},"error":{"max_abs":0.0009765625}}`. Inspect the returned breakpoint and table widths, the total `memory_bits` and the maximum error over the specified inputs.

For table compression, supply the existing Lookup1D block as `reference` and set the input type and error tolerance. The error is measured against the value of that block at each stored input.

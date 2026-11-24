# fxsim v0.1-preview

fxsim is an independent rewrite of a subset of the fixed-point DSP design workflow in MathWorks Fixed-Point Designer and Simulink. Claude and Codex did most of the development. With fxsim, an agent can write fixed-point arithmetic, build a discrete model, try word lengths and export a candidate for a final check in the original tools.

Industrial design software is expensive, and much of its interface was designed for an engineer at a workstation. We wanted an agent to be able to try a change, inspect its result and retry on its own. With familiar commands, machine-readable results and a small local process for each run, several agents can explore different designs on the same machine. The goal is to fit more design iterations into a day, shorten development cycles and let engineers finish more designs. The measurements below show how far this first version gets.

**If you already write MATLAB, the workflow for supported scripts is essentially the same.** Write `fi`, `fimath` and `numerictype` expressions or use Simulink commands such as `new_system`, `add_block` and `sim`. Run the same `.m` file with `fxsim-matlab -batch`. This dedicated wrapper is still rough. It supports a subset of MATLAB and flat, discrete models. We are sharing the first version early so people can try it while we improve the interface and expand the engine.

**Final pass counts were on par** (fxsim round 3: 28/43, 65.1%, against MATLAB/Simulink 27/43, 62.8%) on 43 reused tasks, with the fxsim entry point developed using examples from the suite and the MATLAB results taken from round 1. Both workflows allowed composition from simpler operations. [Three-round results and timing](#what-an-agent-completed)

## Try the same code with fxsim

On macOS, extract the ZIP into an empty writable directory and open a shell there. The build uses Python 3, its standard library and the system Clang C99 compiler. MATLAB is required only for the independent native replay. From the package root, run

```sh
shasum -a 256 -c MANIFEST.sha256 && sh engine/build.sh
export PATH="$PWD/engine:$PATH"
mkdir -p runs/entry-demo
cd runs/entry-demo
```

### An eight-bit accumulator

Save this function as `accumulate_frame.m`. It adds 31 and 2 using a signed eight-bit accumulator with two fractional bits. Under wrap arithmetic the result is −31, stored as the integer −124.

```matlab
function y = accumulate_frame(x)
T = numerictype(1,8,2);
F = fimath('RoundingMethod','Floor','OverflowAction','Wrap', ...
           'SumMode','SpecifyPrecision','SumWordLength',8,'SumFractionLength',2);
u = fi(x,T,F);
y = fi(0,T,F);
for k = 1:numel(u)
    y(:) = y + u(k);
end
end
```

Check all eight bits. The command prints `10000100`.

```sh
fxsim-matlab -batch "y=accumulate_frame([31;2]); assert(strcmp(bin(y),'10000100')); disp(bin(y));"
```

### Build, simulate and save a model

Save this script as `delay_demo.m` in the same directory. It connects an input to a Unit Delay and an output, applies three samples half a second apart, and checks both the values and the times before saving the model.

```matlab
m='delay_demo';
new_system(m);
set_param(m,'SolverType','Fixed-step','Solver','FixedStepDiscrete','FixedStep','0.5','StopTime','1',...
    'SaveOutput','on','SaveTime','on','SaveFormat','Dataset','ReturnWorkspaceOutputs','on');
add_block('simulink/Sources/In1',[m '/x'],'OutDataTypeStr','fixdt(1,8,2)','SampleTime','0.5');
add_block('simulink/Discrete/Unit Delay',[m '/d'],'InitialCondition','0');
add_block('simulink/Sinks/Out1',[m '/y']);
add_line(m,'x/1','d/1','autorouting','on');
add_line(m,'d/1','y/1');
ds=Simulink.SimulationData.Dataset;
ds=ds.addElement(timeseries(fi([1;-2;3],1,8,2),(0:0.5:1)'), 'x');
set_param(m,'LoadExternalInput','on','ExternalInput','ds');
out=sim(m);
sig=out.yout.getElement(1).Values;
assert(isequal(double(sig.Data),[0;1;-2]));
assert(isequal(sig.Time,[0;0.5;1]));
save_system(m,[m '.slx']);
close_system(m,0);
```

```sh
fxsim-matlab -batch "run('delay_demo.m');"
```

The expected output values are `[0;1;-2]` at times `[0;0.5;1]`. The command writes `delay_demo.slx`, `delay_demo_io.json`, `delay_demo_manifest.json` and `check_delay_demo.m`. To check this export, bring all four files to a host with licensed MATLAB R2026b, Simulink and Fixed-Point Designer. From their directory, run

```sh
matlab -batch "check_delay_demo;" > native-check.log 2>&1 && grep -Fx 'PASS delay_demo flat model output replay' native-check.log
```

The checker compares the saved model's output types, sample times and stored bits with the expected values. A successful check produces the exact line shown in the command. For the supported script forms and model settings, see the [MATLAB syntax guide](skill/fxsim/MATLAB.md) and the [Python API guide](skill/fxsim/PYTHON.md).

### Explore word lengths

For a three-tap FIR filter, the quickstart supplies the design, the eight expected samples and the commands to simulate, search word lengths and export the result. From the package root, create `runs/qs/` and follow section 3. The JSON tools report outputs and diagnostics that an agent can read directly. [FIR quickstart](skill/fxsim/QUICKSTART.md#3-the-command-chain-simulate-optimise-export-check-in-simulink)

After the export, run the native check from the package root on a licensed host. Read the checker's verdict. On macOS, MATLAB R2026b can return process status 0 after a failed model check.

```sh
(cd runs/qs/fir3 && matlab -batch "check_fir3;") | python3 -I docs/en/read_verdict.py fir3
```

The reader requires one matching `PASS` record. For `FAIL`, inspect `reason` and `first_diff`. For `INCOMPLETE`, inspect `reason`. A missing record leaves the model unverified. [Export guide](docs/en/EXPORT.md)

## What an agent completed

On November 23–24, 2026, Sonnet 5.5 attempted the same 43 design tasks with each workflow, with at most 40 turns per task. The agent could build an equivalent design from simpler supported blocks or operations when a named block was missing. A pass required the submitted design or function and its requested results to satisfy the grader on replay in MATLAB/Simulink R2026b.

| Workflow | First submission passed | Final submission passed | Median end-to-end time, all 43 | Median turns | Estimated model cost, 43 scored runs |
|---|---:|---:|---:|---:|---:|
| MATLAB/Simulink R2026b, round 1 baseline | 23/43 | 27/43 | 105 s | 13 | $6.78 |
| fxsim round 1 | 18/43 | 20/43 | 150 s | 19 | $15.87 |
| fxsim round 2 | 18/43 | 21/43 | 143 s | 18 | $14.63 |
| fxsim round 3 | 27/43 | 28/43 | 82 s | 11 | $9.09 |

Round 2 revised the instructions and input validation. Repeating one interrupted host run brought that round to 19 first and 22 final passes for an additional $0.18. Round 3 added the MATLAB-style command and revised the tool description. Only one task used `butter` and `roots`, both during exploration when the interface returned recorded MATLAB outputs, and counting that task as an fxsim failure gives both workflows 27 final passes out of 43. An interruption split that round into two batches with three and 40 scored tasks. Interrupted attempts added about $0.22. The table counts one scored run per task per workflow. Both later rounds reuse the original MATLAB results, including four invalid final submissions. The preview includes subsequent model-saving fixes.

In round 3, both workflows passed 24 tasks. Four passed only with fxsim, three only with MATLAB, and 12 with neither. On the 24 shared passes, median end-to-end times were 76 s and 112 s. Their ratio was 0.68. Comparing each task with its own counterpart gave a median fxsim/MATLAB ratio of 0.96. Across all 43 tasks, median time inside tool calls was 7.0 s with fxsim and 5.6 s with MATLAB.

Review of the 15 unsuccessful fxsim final submissions found zero failures confirmed to have been caused by an incorrect fxsim numerical result. Five lacked a native numerical metric for comparison. These figures measure the workflow on this suite. Several things changed between rounds, and performance on new tasks remains open.

## Which features have been checked?

The November 23, 2026 inventory lists 210 feature rows. Each row describes a particular operation and scope, such as a rounding rule, data shape or sample-time configuration. The earlier checks on November 21–22 gave the following results.

| Earlier evidence for the row | Basic | High-frequency beyond basic | Cross-cutting | Total |
|---|---:|---:|---:|---:|
| Verified for a specified subset of configurations | 87 | 56 | 9 | 152 |
| Refused in whole or in part | 15 | 18 | 3 | 36 |
| Known issue | 3 | 9 | 8 | 20 |
| Accepted, verification pending | 0 | 1 | 0 | 1 |
| Unimplemented | 0 | 1 | 0 | 1 |
| **Feature rows** | **105** | **85** | **20** | **210** |

For example, the scalar Sum row covers explicit accumulator and output types. A test of one configuration only supports a claim within its tested range. The appendix preserves the scope, date and status of each row, including later issues and pending whole-feature and composition assessments. [Full feature inventory](#full-feature-inventory)

### How often did the engine complete a design directly?

On November 23, 2026, an independent evaluation tested the round 1 engine on high-frequency features beyond the basic tier. For each profile, the denominator counts in-scope designs the native tools could run. The numerator counts designs fxsim completed directly and correctly under the promised backend and export requirements. A refusal on a native-runnable design counts as a miss.

| How the tested configurations were sampled | Direct completions | Completion rate | One-sided 97.5% lower bound |
|---|---:|---:|---:|
| Block frequencies in 88 HDL example models | 1,972/2,103 | 93.77% | 92.65% |
| Block frequencies from a profile of design tasks | 1,840/2,160 | 85.19% | 83.62% |

Both lower bounds cleared the 80% target. All 3,840 returned answers passed native export replay. There were zero severe errors in 3,805 distinct answered designs across the two profiles, with a one-sided 97.5% upper bound of 0.097%.

**The high-frequency tier failed its formal assessment.** Two checks exceeded the 1% upper limit. Refusal reasons were inaccurate in 9 of 888 designs (upper bound 1.92%). Required diagnostics were missing or wrong in 49 of 860 (7.46%). False acceptance passed at 0/393 and internal errors or timeouts at 0/4,617. The batch issued no optimisation guarantees, leaving that check unassessed. These rates describe direct support for the configurations sampled under each profile. Some feature configurations lacked a sampler. For completion with agent-built alternatives, the whole-task evidence is the 43-task comparison above.

An earlier basic-tier development sample (November 21) completed 64/71 HDL-example-profile and 98/132 task-profile designs directly. Formal basic and high-frequency results for the packaged engine, and a population estimate with composition, are still pending.

## Simulation time and memory on two small designs

On November 24, 2026, we ran the preview engine and MATLAB/Simulink R2026b on one Apple M4 Mac. We tested two single-rate designs, a 128-sample Unit Delay and a 15-tap FIR filter with 4,096 samples. Each ends in a fixed-point conversion. The repeat workload ran the same 12-bit output eight times. The sweep used the same input and changed the output conversion from 9 through 16 bits, once per width.

Each table entry is **correct results per second** for one batch of eight requests, shared among the indicated number of workers. Timing ran from batch dispatch through verification of the last result. We checked the output type and every stored integer against an independent integer reference. All 288 timed outputs passed, as did six separate smoke checks. Each configuration ran once, so run-to-run variation is unknown.

fxsim started a fresh `fxgraph` process for every request. Warm MATLAB kept one session per worker, loaded the model and completed a checked warm-up before timing. It used Fast Restart for repeats and `set_param` plus `sim` for the sweep. Cold MATLAB started a new session for each request, with startup and model loading inside the timed interval. All arms used normal operating-system file caches.

| Design | Workload | Workers | fxsim | Warm MATLAB | Cold MATLAB |
|---|---|---:|---:|---:|---:|
| Unit Delay | Repeat | 1 | 18.39 | 10.56 | 0.055 |
| Unit Delay | Repeat | 2 | 24.47 | 15.58 | 0.068 |
| Unit Delay | Repeat | 4 | 34.74 | 18.73 | 0.093 |
| Unit Delay | Sweep | 1 | 17.90 | 4.25 | 0.045 |
| Unit Delay | Sweep | 2 | 23.32 | 6.01 | 0.065 |
| Unit Delay | Sweep | 4 | 40.57 | 9.54 | 0.101 |
| FIR | Repeat | 1 | 18.71 | 12.23 | 0.037 |
| FIR | Repeat | 2 | 20.87 | 14.65 | 0.063 |
| FIR | Repeat | 4 | 41.61 | 20.49 | 0.110 |
| FIR | Sweep | 1 | 11.90 | 2.49 | 0.042 |
| FIR | Sweep | 2 | 20.20 | 3.54 | 0.057 |
| FIR | Sweep | 4 | 38.77 | 10.53 | 0.111 |

With one worker, the four design/workload combinations gave the following ranges of median request latency. The memory and CPU ranges cover all three worker counts.

| Measurement | fxsim | Warm MATLAB | Cold MATLAB |
|---|---:|---:|---:|
| Median time from request dispatch to checked output, one worker | 0.052–0.080 s | 0.073–0.398 s | 16.42–24.52 s |
| Largest single child process peak RSS in each batch | 36.2–38.1 MB | 1.86–2.30 GB | 1.86–2.28 GB |
| Total child-process CPU time per eight requests | 0.40–0.85 s | 21.92–199.73 s | 173.75–377.53 s |

CPU time covers each child process from startup to exit, including the warm sessions’ setup and warm-up. With four workers, peak sampled RSS summed over the benchmark’s process groups was 6.73–7.37 GB for warm MATLAB and 7.35–8.15 GB for cold MATLAB (decimal units). The 100 ms sampler captured at most one timed fxsim sample per four-worker batch, too few to estimate its concurrent memory use.

These timings cover simulation and output checking on the two designs. Time spent by an agent writing, running and revising a solution is reported in the [43-task comparison](#what-an-agent-completed).

## Known limits and verification

- The wrapper supports selected `fi` operations and flat fixed-step models. General MATLAB programs, interactive sessions and arbitrary third-party SLX edits are outside its current scope. For graph support, check the block, parameter and type combination you need. [Block and graph reference](docs/en/GRAPH.md)
- Unsupported configurations can return a code, location and reason. Preserve the request when reporting a refusal. The formal assessment found cases with inaccurate reasons or missing diagnostics. On its assessed revision, some host MinMax requests with three or more mixed-sign inputs and non-Floor rounding were refused.
- On the November 23 assessment revision, the lookup-table optimiser could discard a feasible table when the output quantisation step equalled the requested tolerance. Treat a failed search in this case as inconclusive.
- On Apple silicon, November 24 tests against R2026b found differences in floating-point NaN sign bits after 0/0, subtraction with a NaN, Unary Minus and Abs of a negative NaN. MinMax can also select a different zero sign on a +0 / −0 tie. These cases concern single and double signals. Floating-point designs are outside the exporter's scope.
- On November 24, the independent replay and the packaged checker each passed all 20 real-design exports from the preview engine (1,963 samples) and rejected all 20 corrupted copies. Across all regression slices run without MATLAB since the round 1 engine, the suite recorded zero incorrect values and zero internal errors. The engine refused the tested ASIC-target lattice moving-average configuration at entry. A separate check of accepted configurations could not run because its reference tree lacked a file. On macOS with R2026b, the saved `delay_demo` flat model passed native replay. Flipping one expected output bit in its I/O file triggered the integrity check and produced no PASS line. Use the check supplied with your own export to verify its inputs and outputs.

The self-contained FIR is the recommended starting point. Some other manual examples use external models. [Agent guide](docs/en/AGENT_GUIDE.md) · [Worked examples](docs/en/WORKED_EXAMPLES.md)

## Development and licence

Claude (Anthropic) contributed implementation and coordination. Codex (OpenAI) contributed the MATLAB-style interface, independent review, evaluation tools and packaging. The team wrote the software from scratch and checked its behaviour against input/output observations from licensed native tools.

We plan to improve compatibility, broaden the supported designs and make the wrapper easier to use. The same approach may extend to related engineering software. fxsim is distributed under the MIT License. MATLAB and Simulink are MathWorks trademarks. fxsim is an independent project. [Licence](LICENSE) · [Trademark notice](NOTICE)

## Full feature inventory

<details>
<summary>Show all 210 feature rows from the November 23, 2026 assessment</summary>

The last two columns refer to the round 1 engine. “Pending” means a whole-row conclusion has yet to be established for that assessment. Earlier evidence carries its own date.

| Feature and scope | Tier | Earlier slice and date | Direct assessment | Verified composition |
|---|---|---|---|---|
| Sum, scalar, explicit output / accumulator types, every rounding mode, saturation on / off, words to 128 bits | Basic | Verified slice · 2026-11-21 | pending | pending |
| Sum with inherited types (internal rule, same as first input, same as accumulator), host | Basic | Verified slice · 2026-11-21 | pending | pending |
| Product, multiply and divide, scalar, explicit and inherited types, division by zero | Basic | Verified slice · 2026-11-21 | pending | pending |
| Gain, scalar; parameter type explicit or inherited; output explicit or inherited | Basic | Verified slice · 2026-11-21 | pending | pending |
| Abs, Unary Minus, Saturation, scalar | Basic | Verified slice · 2026-11-21 | pending | pending |
| Boolean operands in arithmetic (Sum / Product / Gain of a boolean) | Basic | Verified slice · 2026-11-21 | pending | pending |
| Inherited arithmetic types on ASIC/FPGA target | Basic | Verified slice · 2026-11-21 | pending | pending |
| Vector / element-wise arithmetic (vector Sum, vector Gain, sum of elements) | Basic | Verified slice · 2026-11-21 | pending | pending |
| Matrix multiplication (Gain Matrix(K*u), Product Matrix(*)) | Basic | Verified slice · 2026-11-21 | pending | pending |
| double / single signals through arithmetic blocks (simulating a floating-point design in fxgraph) | Basic | Refused in whole or part · 2026-11-22 | pending | pending |
| Type inheritance by back propagation into / out of Sum, Product, Gain (incl. feedback loops) | Basic | Verified slice · 2026-11-21 | pending | pending |
| Constant, scalar, explicit fixed-point or boolean type, literal or literal arithmetic value | Basic | Verified slice · 2026-11-21 | pending | pending |
| Constant with "Inherit via back propagation" into Sum / MinMax / Product / Relational / Switch / typed Outport | Basic | Verified slice · 2026-11-21 | pending | pending |
| Constant with its library default type (inherit from value = double) | Basic | Refused in whole or part · 2026-11-22 | pending | pending |
| Vector / matrix Constant | Basic | Verified slice · 2026-11-21 | pending | pending |
| Extra numeric parameter syntax: scalar nextpow2 and recorded colon/list arithmetic (ordinary literals are covered by the scalar Constant row) | Basic | Verified slice · 2026-11-21 | pending | pending |
| Saved real-scalar/row mask and model-workspace parameters, without initialization code | Basic | Verified slice · 2026-11-21 | pending | pending |
| Arbitrary parameter functions/constants and unsaved base-workspace names | Basic | Refused in whole or part · 2026-11-22 | pending | pending |
| Data Type Conversion, real-world value, explicit type, every rounding mode, saturation | Basic | Verified slice · 2026-11-21 | pending | pending |
| Data Type Conversion, stored-integer mode | Basic | Verified slice · 2026-11-21 | pending | pending |
| Data Type Conversion with back propagation (its library default) | Basic | Verified slice · 2026-11-21 | pending | pending |
| Data Type Conversion to / from boolean | Basic | Verified slice · 2026-11-21 | pending | pending |
| Data Type Conversion to / from double or single | Basic | Refused in whole or part · 2026-11-22 | pending | pending |
| Vector Data Type Conversion | Basic | Verified slice · 2026-11-21 | pending | pending |
| Relational Operator, scalar, all operators, mixed types | Basic | Verified slice · 2026-11-21 | pending | pending |
| MinMax, two inputs | Basic | Verified slice · 2026-11-21 | pending | pending |
| MinMax with three or more inputs / over a vector | Basic | Verified slice · 2026-11-21 | Certain mixed-sign host requests refused (see above), whole row pending | pending |
| Compare To Constant (boolean or uint8 output) | Basic | Verified slice · 2026-11-21 | pending | pending |
| Compare To Zero | Basic | Verified slice · 2026-11-21 | pending | pending |
| Switch, three criteria, mixed data types, boolean control | Basic | Verified slice · 2026-11-21 | pending | pending |
| Multiport Switch, contiguous one- / zero-based ports, default-port diagnostic | Basic | Verified slice · 2026-11-21 | pending | pending |
| Multiport Switch with "Specify indices" | Basic | Refused in whole or part · 2026-11-22 | pending | pending |
| Manual Switch | Basic | Verified slice · 2026-11-21 | pending | pending |
| Comparison / selection on vectors | Basic | Verified slice · 2026-11-21 | pending | pending |
| Inherited types of comparison / selection blocks inside feedback loops | Basic | Verified slice · 2026-11-21 | pending | pending |
| Logical Operator, boolean inputs, 1 .. N inputs, all operators | Basic | Verified slice · 2026-11-21 | pending | pending |
| Logical Operator on non-boolean inputs / AllPortsSameDT | Basic | Verified slice · 2026-11-21 | pending | pending |
| Logical Operator with a non-boolean output type | Basic | Verified slice · 2026-11-21 | pending | pending |
| Vector logic | Basic | Verified slice · 2026-11-21 | pending | pending |
| Assertion (stops the run; non-stopping form) | Basic | Verified slice · 2026-11-21 | pending | pending |
| Unit Delay, any type, initial condition | Basic | Verified slice · 2026-11-21 | pending | pending |
| Delay, dialog length N, scalar initial condition | Basic | Verified slice · 2026-11-21 | pending | pending |
| Delay on vectors / per-step initial conditions | Basic | Verified slice · 2026-11-21 | pending | pending |
| Delay with an enable port | Basic | Verified slice · 2026-11-21 | pending | pending |
| Delay with an external reset (Rising / Falling / Either / Level / Level hold) | Basic | Verified slice · 2026-11-21 | pending | pending |
| Delay length or initial condition from an input port | Basic | Verified slice · 2026-11-21 | pending | pending |
| Detect Change / Increase / Decrease / Rise Positive | Basic | Verified slice · 2026-11-21 | pending | pending |
| Detect Fall Negative / Fall Nonpositive / Rise Nonnegative | Basic | Verified slice · 2026-11-21 | pending | pending |
| Discrete-Time Integrator, Forward Euler, gain, explicit or inherited type | Basic | Verified slice · 2026-11-21 | pending | pending |
| Forward Euler integrator inside feedback loops (PI, AGC) | Basic | Verified slice · 2026-11-21 | pending | pending |
| Integrator output limits | Basic | Verified slice · 2026-11-21 | pending | pending |
| Backward Euler / Trapezoidal / accumulation methods | Basic | Verified slice · 2026-11-21 | pending | pending |
| Integrator external reset / external initial condition / state port | Basic | Verified slice · 2026-11-21 | pending | pending |
| Counter Limited | Basic | Verified slice · 2026-11-21 | pending | pending |
| Counter Free-Running | Basic | Verified slice · 2026-11-21 | pending | pending |
| Counters / integrators in multirate designs (sample time) | Basic | Verified slice · 2026-11-21 | pending | pending |
| Extract Bits: upper / lower half, MSB / LSB ranges | Basic | Verified slice · 2026-11-21 | pending | pending |
| Extract Bits: range of bits, one index, scaling modes | Basic | Verified slice · 2026-11-21 | pending | pending |
| Bitwise Operator with a mask | Basic | Known issue · 2026-11-22 | pending | pending |
| Bitwise Operator with several inputs | Basic | Verified slice · 2026-11-21 | pending | pending |
| Shift Arithmetic, dialog shift (left / right / bidirectional, binary point) | Basic | Verified slice · 2026-11-21 | pending | pending |
| Shift amount from an input port | Basic | Refused in whole or part · 2026-11-22 | pending | pending |
| Signum | Basic | Verified slice · 2026-11-21 | pending | pending |
| Bit operations on wide words (over 64 bits) | Basic | Verified slice · 2026-11-21 | pending | pending |
| Extract Bits / Bitwise on vectors | Basic | Verified slice · 2026-11-21 | pending | pending |
| 1-D lookup, Flat / Nearest | Basic | Verified slice · 2026-11-21 | pending | pending |
| 1-D linear, explicit fraction and intermediate types, Clip | Basic | Verified slice · 2026-11-21 | pending | pending |
| 1-D linear, inherited fraction (library default), signed breakpoints, UseLastTableValue off, host; ordinary-gap slice excludes the two finer-input regions below; explicit spacing/hardware/width predicates must be attached | Basic | Verified slice · 2026-11-21 | pending | pending |
| 1-D linear, inherited fraction, UseLastTableValue on, rounding Nearest / Round / Ceiling / Convergent / Simplest; ordinary-gap slice excludes the two finer-input regions below; explicit spacing/hardware/width predicates must be attached | Basic | Verified slice · 2026-11-21 | pending | pending |
| 1-D linear, inherited fraction, UseLastTableValue on, Floor / Zero (what fxapprox emits); ordinary-gap slice excludes the two finer-input regions below; explicit spacing/hardware/width predicates must be attached | Basic | Verified slice · 2026-11-21 | pending | pending |
| 1-D host Speed inherited fraction, unsigned breakpoints, derived fraction F<16 | Basic | Refused in whole or part · 2026-11-22 | pending | pending |
| Same unsigned-breakpoint region with F>=16, within recorded range (including required spacing/width checks) | Basic | Verified slice · 2026-11-21 | pending | pending |
| 1-D finer input: gap8..15 >=F, host/signed input/inherited fraction/rising table/Floor or Nearest; all declared intermediate widths | Basic | Verified slice · 2026-11-21 | pending | pending |
| Finer-input regions outside that accepted predicate, including gap>=16 and unrecorded hardware/sign/rounding/table combinations | Basic | Refused in whole or part · 2026-11-22 | pending | pending |
| 1-D, intermediate type "Inherit via internal rule" | Basic | Refused in whole or part · 2026-11-22 | pending | pending |
| 1-D linear interpolation with linear extrapolation (the library default) | Basic | Refused in whole or part · 2026-11-22 | pending | pending |
| 1-D Lagrange / cubic spline / Akima | Basic | Refused in whole or part · 2026-11-22 | pending | pending |
| 2-D lookup, explicit types (linear, flat) | Basic | Verified slice · 2026-11-21 | pending | pending |
| 2-D internal-rule intermediate / inherited fraction: host Precision, table=output, gap0..7 recorded combinations | Basic | Verified slice · 2026-11-21 | pending | pending |
| 2-D other unrecorded internal-rule regions, including Speed and ASIC | Basic | Refused in whole or part · 2026-11-22 | pending | pending |
| Direct Lookup Table, 1-D / 2-D element | Basic | Known issue · 2026-11-22 | pending | pending |
| n-D (3 and more) lookups, Prelookup + Interpolation Using Prelookup | Basic | Verified slice · 2026-11-21 | pending | pending |
| Table / breakpoints from ports, lookup objects, tunable size | Basic | Refused in whole or part · 2026-11-22 | pending | pending |
| Goto / From (in .slx; JSON uses lines) | Basic | Verified slice · 2026-11-21 | pending | pending |
| Mux / Concatenate (building vectors) | Basic | Verified slice · 2026-11-21 | pending | pending |
| Demux / Selector (splitting vectors) | Basic | Verified slice · 2026-11-21 | pending | pending |
| Reshape | Basic | Verified slice · 2026-11-21 | pending | pending |
| Vector Inports / Outports, width propagation | Basic | Verified slice · 2026-11-21 | pending | pending |
| Matrices (2-D signals), multidimensional Concatenate / Selector, Assignment | Basic | Refused in whole or part · 2026-11-22 | pending | pending |
| Ground / Terminator | Basic | Verified slice · 2026-11-21 | pending | pending |
| Merge, data stores, If / Switch Case action routing | Basic | Known issue · 2026-11-22 | pending | pending |
| Inport / Outport, fixed point; a typed Outport is checked like Simulink | Basic | Verified slice · 2026-11-21 | pending | pending |
| Signal Specification (type / dimension check) | Basic | Verified slice · 2026-11-21 | pending | pending |
| Data Type Duplicate | Basic | Verified slice · 2026-11-21 | pending | pending |
| Untyped Outport fed by a back-propagating block (double) | Basic | Refused in whole or part · 2026-11-22 | pending | pending |
| Inport sample time and dimension declarations | Basic | Verified slice · 2026-11-21 | pending | pending |
| Math Function: square, magnitude^2, reciprocal (inherited or the one recorded output type) | Basic | Verified slice · 2026-11-21 | pending | pending |
| Math Function square / reciprocal with any explicit output type | Basic | Verified slice · 2026-11-21 | pending | pending |
| Math Function mod / rem on integer classes | Basic | Verified slice · 2026-11-21 | pending | pending |
| Square root (Sqrt block / Math sqrt) on fixed point | Basic | Verified slice · 2026-11-21 | pending | pending |
| Trigonometric Function (sin / cos / atan2, CORDIC) | Basic | Verified slice · 2026-11-21 | pending | pending |
| Bias | Basic | Verified slice · 2026-11-21 | pending | pending |
| Increment / Decrement (real world, stored integer) | Basic | Verified slice · 2026-11-21 | pending | pending |
| Repeat (multirate) | Basic | Verified slice · 2026-11-21 | pending | pending |
| Dead Zone, Rate Limiter, Wrap To Zero and similar | Basic | Verified slice · 2026-11-21 | pending | pending |
| Unit Delay Enabled / Resettable / Enabled Resettable Synchronous (445 uses in the HDL examples) | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| Tapped Delay / Tapped Delay Enabled Synchronous | High-frequency | Known issue · 2026-11-22 | pending | pending |
| Downsample (factor, offset, initial condition), multirate | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| Upsample | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| Rate Transition, zero offset, Integrity / Deterministic combinations, initial condition | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| Rate Transition with an offset input and Integrity on | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| Mixed sample-time grids: equal non-zero offsets, different periods | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| Zero-Order Hold | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| Frame-based processing (the library defaults of these blocks) | High-frequency | Refused in whole or part · 2026-11-22 | pending | pending |
| A block's own SampleTime slower than its input | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| Fractional / non-binary periods | High-frequency | Known issue · 2026-11-22 | pending | pending |
| Virtual subsystem hierarchy and masks without initialization code; see the row on saved mask and model-workspace parameters for numeric values | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| Enabled subsystem (held / reset states, output when disabled) | High-frequency | Known issue · 2026-11-22 | pending | pending |
| State Control inside an enabled subsystem | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| Triggered / function-call / resettable / action subsystems | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| Bus Creator / Bus Selector / Bus Assignment | High-frequency | Known issue · 2026-11-22 | pending | pending |
| For Each subsystem | High-frequency | Refused in whole or part · 2026-11-22 | pending | pending |
| Model Reference | High-frequency | Accepted, verification pending · 2026-11-21 | pending | pending |
| RAM / FIFO / Delay (N) inside conditional subsystems | High-frequency | Refused in whole or part · 2026-11-22 | pending | pending |
| Real-Imag to Complex / Complex to Real-Imag, both ports (309 uses in the HDL examples) | High-frequency | Known issue · 2026-11-22 | pending | pending |
| Complex signals through Gain, Product, Unit Delay, conj, Sum (internal rule) | High-frequency | Known issue · 2026-11-22 | pending | pending |
| Complex Sum with explicit types; complex into Delay (N), Switch, FIR, lookup, conversion | High-frequency | Refused in whole or part · 2026-11-22 | pending | pending |
| Complex to Magnitude-Angle and back | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| Complex Inports (complex stimulus) | High-frequency | Refused in whole or part · 2026-11-22 | pending | pending |
| MATLAB Function: scalar fi arithmetic, casts, fimath choices | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| MATLAB Function: integer classes, saturation, conversions | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| MATLAB Function: if / switch / for with constant bounds | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| MATLAB Function: persistent scalar state | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| MATLAB Function: structs of scalar fields (register bundles) | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| MATLAB Function: row vectors (state and outputs) | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| MATLAB Function: local functions | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| MATLAB Function: bit functions (bitand, bitsliceget, bitconcat, shifts) | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| MATLAB Function: while / break, matrices, 2-D indexing, division by a signal | High-frequency | Refused in whole or part · 2026-11-22 | pending | pending |
| MATLAB Function: fixed point above 32 bits; double outputs | High-frequency | Refused in whole or part · 2026-11-22 | pending | pending |
| MATLAB Function: other built-in calls (all, any, sum, numel, ...) | High-frequency | Refused in whole or part · 2026-11-22 | pending | pending |
| HDL Counter, Count limited | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| HDL Counter, Free running | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| HDL Counter, Modulo | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| HDL Counter control ports (reset, load, enable, direction) | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| HDL Counter count_hit output | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| HDL Counter signed / fractional counts | High-frequency | Known issue · 2026-11-22 | pending | pending |
| HDL Counter with CountSampTime other than 1 | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| HDL Counter beyond the swept word / fraction lengths | High-frequency | Refused in whole or part · 2026-11-22 | pending | pending |
| Bit Slice (193 uses in the HDL examples) | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| Bit Concat, 1 .. 128 inputs, to 326 bits | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| Bit Shift / Bit Rotate | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| Bit Set / Bit Clear / Bit Reduce | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| Boolean BitSlice/Shift/Rotate/Set/Clear and otherwise unrecorded widths; boolean BitConcat is covered by the Bit Concat row | High-frequency | Refused in whole or part · 2026-11-22 | pending | pending |
| HDL FIFO, Classic, recorded sizes and data types | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| HDL FIFO, other sizes / data types | High-frequency | Refused in whole or part · 2026-11-22 | pending | pending |
| HDL FIFO FWFT mode / reset port / ratio | High-frequency | Refused in whole or part · 2026-11-22 | pending | pending |
| HDL RAM (Simple dual, Dual, Single port), up to 8 address bits, recorded data widths | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| Deep RAMs (9 .. 16 address bits), recorded combinations | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| RAM with other data widths / unrecorded deep combinations | High-frequency | Refused in whole or part · 2026-11-22 | pending | pending |
| RAM System blocks, initial values | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| True dual port / tri-port RAM (see RAM, FIFO and Delay inside conditional subsystems) | High-frequency | Refused in whole or part · 2026-11-22 | pending | pending |
| Discrete FIR, direct form, inherited internals | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| Discrete FIR, explicit coefficient / product / accumulator / output types | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| Discrete FIR, symmetric / antisymmetric | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| Discrete FIR, transposed | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| Discrete FIR: lattice, coefficients from a port, reset / enable ports | High-frequency | Known issue · 2026-11-22 | pending | pending |
| Discrete FIR with wide types (to 128 bits) | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| FIR Decimation | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| FIR Interpolation | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| CIC Decimation | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| CIC Interpolation | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| IIR blocks (Discrete Filter, Discrete Transfer Fcn, Biquad) | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| Vector (multichannel) FIR | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| DSP HDL FIR, Direct form systolic, signed 12 / 16 / 18 / 24-bit inputs | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| DSP HDL FIR, Direct form transposed | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| DSP HDL FIR, Partly serial (tabled NumCycles / taps) | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| DSP HDL FIR, explicit output / coefficient types, rounding, saturation | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| DSP HDL FIR reset port and valid gaps | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| DSP HDL FIR on other input types | High-frequency | Refused in whole or part · 2026-11-22 | pending | pending |
| DSP HDL FIR Decimator, NumCycles 1 | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| DSP HDL FIR Decimator, NumCycles over 1 (11 symmetric keys) | High-frequency | Refused in whole or part · 2026-11-22 | pending | pending |
| Decimator factors other than 2 / 3 / 4 / 8, asymmetric taps with NumCycles over 1, transposed decimator | High-frequency | Refused in whole or part · 2026-11-22 | pending | pending |
| HDL NCO, dialog increment, sine / cosine, no dither within the recorded range | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| HDL NCO, complex exponential output | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| HDL NCO with dither | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| HDL NCO with a phase increment port | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| HDL NCO phase offset (dialog / port) and phase output | High-frequency | Known issue · 2026-11-22 | pending | pending |
| HDL NCO reset port / several options together | High-frequency | Verified slice · 2026-11-21 | pending | pending |
| HDL NCO LUT compression / phase quantisation off | High-frequency | Refused in whole or part · 2026-11-22 | pending | pending |
| Other signal sources (Signal From Workspace 167 uses in the HDL examples, Sine Wave, PN Sequence) | High-frequency | Unimplemented · 2026-11-22 | pending | pending |
| fxopt: word lengths with an overflow guarantee (range mode), single rate | Cross-cutting | Verified slice · 2026-11-21 | pending | pending |
| fxopt / fxcert error bounds against the ideal design (exhaustive, sampled, bounded) | Cross-cutting | Verified slice · 2026-11-21 | pending | pending |
| Converting a floating-point design (types written "double") | Cross-cutting | Verified slice · 2026-11-21 | pending | pending |
| Hardware cost: hwbits and FPGA / ASIC estimates | Cross-cutting | Known issue · 2026-11-22 | pending | pending |
| fxcert on a given design | Cross-cutting | Known issue · 2026-11-22 | pending | pending |
| Function-to-LUT approximation outside the known failing low-word unsigned requests; lookup dependency applies | Cross-cutting | Verified slice · 2026-11-21 | Known issue with pruning at one quantization step | pending |
| Low-word unsigned function approximation under constrained table word lengths (sqrt, sin and tanh cases) | Cross-cutting | Known issue · 2026-11-22 | pending | pending |
| fxapprox: table compression | Cross-cutting | Verified slice · 2026-11-21 | pending | pending |
| Optimisation and proof on multirate designs / CIC chains | Cross-cutting | Known issue · 2026-11-22 | pending | pending |
| Range proofs for other wrap-around accumulators / unsettled loops | Cross-cutting | Refused in whole or part · 2026-11-22 | pending | pending |
| Complete information: compiled types, assumed defaults, per-block overflow counts and first step | Cross-cutting | Known issue · 2026-11-22 | pending | pending |
| Export to Simulink with a one-command PASS / FAIL check | Cross-cutting | Known issue · 2026-11-22 | pending | pending |
| .slx read and write-back of chosen types | Cross-cutting | Verified slice · 2026-11-21 | pending | pending |
| Production hardware other than x86-64 and ASIC/FPGA | Cross-cutting | Refused in whole or part · 2026-11-22 | pending | pending |
| Library defaults honoured for omitted parameters | Cross-cutting | Known issue · 2026-11-22 | pending | pending |
| Optimisation against the quantised input design reference, respecting fixed/allowed types and tolerance | Cross-cutting | Verified slice · 2026-11-21 | pending | pending |
| Cold/warm/auto re-optimisation, stop budget and cost choice under the same guarantee | Cross-cutting | Verified slice · 2026-11-21 | pending | pending |
| Strict input/refusal contract: structured stable error, location, reason and actionable supported boundary | Cross-cutting | Known issue · 2026-11-22 | Known issue with infeasibility explanations | pending |
| Agent batch, summary/detail, deterministic retry and independent concurrent requests | Cross-cutting | Verified slice · 2026-11-21 | pending | pending |
| Overflow diagnostics and cost for multirate designs (complete-information contract) | Cross-cutting | Refused in whole or part · 2026-11-22 | pending | pending |

</details>

---
name: fxsim
description: Build and simulate fixed-point DSP block diagrams with fxsim, inspect range and overflow, choose word lengths, read or retype a Simulink model, and export a model with a native replay check.
---

# fxsim quick reference

## Use it like MATLAB

For an existing MATLAB function file, keep its name and call it from its directory with `fxsim-matlab -batch "y=my_filter([0.5;-0.5]); disp(double(y));"`. For a script, use `fxsim-matlab -batch "run('my_script.m')"`. See [MATLAB.md](MATLAB.md) for supported arithmetic and refusals.

## Python modeling

From the unpacked package root, use Python to build a model and save a real SLX.
```python
from fxsim import new_system, add_block, add_line, set_param, get_param, sim, save_system
from fxsim import fi, fimath, numerictype, fixdt, double, storedInteger
m = new_system("delay_demo")
add_block("simulink/Sources/In1", m+"/x", "OutDataTypeStr", str(fixdt(1,8,2)))
d = add_block("simulink/Discrete/Unit Delay", m+"/d")
set_param(d, "InitialCondition", "0")
add_block("simulink/Sinks/Out1", m+"/y")
add_line(m, "x/1", "d/1")
add_line(m, "d/1", "y/1")
x = fi([0.5, -0.75, 1], numerictype(1,8,2), fimath())
out = sim(m, inputs={"x": x})
assert out.yout.getElement(1).Values.storedInteger == [0, 2, -3]
assert out.tout == [0, 1, 2] and get_param(d, "InitialCondition") == "0"
package = save_system(m, "delay_demo.slx")
```
See [PYTHON.md](PYTHON.md) for logged signals, fixed-point arithmetic, and saving a flat model.

## JSON command-line tools

New here? [QUICKSTART.md](QUICKSTART.md) has one runnable request per common block, the full simulate -> optimise -> export -> check chain, and the common refusal codes. The detailed guide is [GUIDE.md](GUIDE.md).

Work from the directory containing `engine/fxgraph`. In a fresh shell call, start from that root again. Write requests there or in a writable subdirectory. The examples use Python's standard library. Relative export paths resolve under the tool tree.

**Three details to get right on the first call.** Key each input by its Inport name. Supply stored integers in `ints` (physical value = `integer * 2**(-fl)`). Build a separate export request with the same design, inputs, horizon and hardware. `diagnostics`, `outputs` and `cost` belong to simulation.

### Simulate two inputs

Save this as `mix.json`. Block names are keys, dialog parameters sit beside `type`, and line ports count from 1.

```json
{"design":{"blocks":{
  "a":{"type":"Inport"}, "b":{"type":"Inport"},
  "mix":{"type":"Sum","Inputs":"+-","OutDataTypeStr":"fixdt(1,12,4)"},
  "y":{"type":"Outport"}},
  "lines":[{"src":"a","dst":"mix","dport":1},{"src":"b","dst":"mix","dport":2},{"src":"mix","dst":"y"}]},
 "inputs":{"a":{"s":1,"wl":8,"fl":4,"ints":[8,16,4]},"b":{"s":1,"wl":8,"fl":4,"ints":[4,4,8]}},
 "diagnostics":true}
```

`python3 -I engine/fxgraph < mix.json > sim.json`. Check `status`, then `summary.diagnostics` and `outputs.y`. This example returns stored codes `[4,12,-4]` with four fraction bits.

| Add to `design.blocks` | JSON template | Wiring |
|---|---|---|
| Scale | `{"type":"Gain","Gain":"0.5","ParamDataTypeStr":"fixdt(1,8,5)","OutDataTypeStr":"fixdt(1,12,4)"}` | One input |
| Delay | `{"type":"UnitDelay","InitialCondition":"0"}` | One sample of state |
| Multiply | `{"type":"Product","Inputs":"**","OutDataTypeStr":"fixdt(1,16,8)"}` | Factors on ports 1 and 2 |
| Cast | `{"type":"DataTypeConversion","OutDataTypeStr":"fixdt(1,8,4)","RndMeth":"Nearest","SaturateOnIntegerOverflow":"on"}` | One input |
| Select | `{"type":"Switch","Criteria":"u2 ~= 0","OutDataTypeStr":"fixdt(1,12,4)"}` | True data, control, false data on ports 1, 2, 3 |

### Export, bind and check

```sh
python3 - <<'PY' > export.json
import json
q = json.load(open("mix.json"))
shared = ("design", "inputs", "hw", "nsteps", "stop_time")
export = {k: q[k] for k in shared if k in q}
print(json.dumps(dict(export, name="mixdemo", out="runs/mixdemo")))
PY
python3 -I engine/fxexport < export.json
```

On `status: ok`, open `runs/mixdemo/manifest.json`. Each ordered `inputs` row binds a design `port` to its generated From Workspace `variable` at `sample_time`. Match source channels by name and keep each port's width and clock. `outputs` gives each root block and width. The native checker supplies the packaged inputs itself.

On the native host, from the tool root with the export in place, run:

```sh
(cd runs/mixdemo && matlab -batch "check_mixdemo;") | python3 -I docs/en/read_verdict.py mixdemo
```

Accept exactly one matching `fxexport-check` JSON record with `verdict: "PASS"`. MATLAB can exit 0 after a mismatch. Read `reason` and `first_diff` on FAIL. INCOMPLETE or an unreadable verdict leaves the native check pending. A PASS covers this model and stimulus.

| If a call returns | Next action |
|---|---|
| `BAD_JSON`, `BAD_PARAM`, `BAD_INPUT` | Inspect `where` and `message`. Check the tool's keys, input names, lengths and types. |
| `TYPE_MISMATCH`, `PORT_TYPE_INVALID` | Inspect the named connections, type or parameter range. Preserve the requested arithmetic. |
| `UNSUPPORTED`, `UNSUPPORTED_EXPORT` | Read `next_step`. Use a documented equivalent when it meets the design requirements, or report the unsupported configuration. |
| `OUT_EXISTS` | Choose a fresh `out` directory and keep the prior export. |

For feedback, multirate timing, word-length selection and port binding, start with the [worked examples](../../docs/en/WORKED_EXAMPLES.md). For other block fields, look up the block heading in the [catalogue](../../docs/en/BLOCKS.md). The [detailed skill guide](GUIDE.md) covers optimisation, certification, SLX input and the rest of the shipped examples.

# Worked examples by design task

Run each block from the directory containing `engine/`. The JSON requests below are in `skill/fxsim/examples/`. For a new design use the [quick reference](../../skill/fxsim/SKILL.md).

## Simulate a stateful design with several stimuli

The feedback example adds the input to a scaled, one-sample-delayed output. Change the inlet samples on the same graph and call fxgraph once for each independent experiment. Each call starts with the design's initial state. Channels that interact in one design belong together in the same request.

```python
import json
import subprocess
from pathlib import Path

q = json.loads(Path("skill/fxsim/examples/01_sim_iir.json").read_text())
for stimulus in ([0, 0, 0], [8192, 0, 0]):
    q["inputs"]["x"]["ints"] = stimulus
    call = subprocess.run(["python3", "-I", "engine/fxgraph"], input=json.dumps(q),
                          text=True, capture_output=True)
    r = json.loads(call.stdout)
    if r["status"] != "ok":
        raise SystemExit(r)
    y = r["outputs"]["y"]
    print([v * 2.0**(-y["fl"]) for v in y["ints"]], r["summary"]["diagnostics"]["first_event"])
```

The zero input stays at zero. The impulse produces `[0.5, 0.375, 0.28125]`. Both runs report no overflow event. For a file of little-endian binary64 physical samples, the standard library can read it with `struct.iter_unpack("<d", path.read_bytes())`. Quantise to the declared inlet type using the design's rounding and overflow rules. A file of stored codes already contains the quantised values.

## Bind a native model to external data

Export the quick reference's two-inlet model, then inspect its compact manifest. Match each source signal to its named port. The generated variable belongs to that port for this export.

```python
import json
from pathlib import Path

package = Path("runs/mixdemo")
manifest = json.loads((package / "manifest.json").read_text())
for row in manifest["inputs"]:
    print(row["port"], row["variable"], row["sample_time"])
for row in manifest["outputs"]:
    print(row["port"], row["root_block"], row["width"])
```

Here `a` binds to `mixdemo_in1` and `b` to `mixdemo_in2`. Use the supplied check script to reconstruct the packaged time series. For a vector inlet, the graph request supplies `x[1]`, `x[2]`, … by element, and the export binds the vector to one workspace variable. In `<model>_io.json`, each input row records width, times and samples. Each output row identifies its one-based root `index`, `element` and any complex `part`. Fixed-point `ints` there are exact decimal strings. Convert with `int(v)` before arithmetic. The output rows of this export contain `["4","12","-4"]` at type `1,12,4`. Native verification uses the [verdict command](../../skill/fxsim/SKILL.md#export-bind-and-check).

## Inspect multirate output on its own clock

```sh
python3 -I engine/fxgraph < skill/fxsim/examples/11_multirate_firdec.json > rate.json
python3 - <<'PY'
import json
r = json.load(open("rate.json"))
if r["status"] != "ok":
    raise SystemExit(r)
y = r["outputs"]["y"]
print(len(y["ints"]), y["t"][:4], y["period"])
PY
```

This shipped decimator receives 32 samples and returns 16, beginning at times `0, 2, 4, 6`. Align later comparisons on `t`. When setting `stop_time`, include a sample for every input tick through that time, including the endpoint.

## Choose word lengths and replay the chosen design

The shipped FIR request gives fxopt input ranges, a sine stimulus, an error budget and two workers. Inspect the plan's error guarantee and overflow guarantee individually. A sampled error bound covers the tested inputs. A violated bound disqualifies the plan.

```sh
python3 -I engine/fxopt < skill/fxsim/examples/03_opt_fir.json > plan.json
python3 - <<'PY'
import json
from pathlib import Path

examples = Path("skill/fxsim/examples")
q = json.loads((examples / "03_opt_fir.json").read_text())
plan = json.loads(Path("plan.json").read_text())
if plan["status"] != "ok":
    raise SystemExit(plan)
s = plan["summary"]
print(s["hwbits"], s["overflow"], s["error"], s["max_error_over_tolerance"])
if s["error"] == "violated" or s["max_error_over_tolerance"] > 1:
    raise SystemExit("plan exceeds its error tolerance")
design = q["design"]
for name, settings in plan["solution"]["blocks"].items():
    design["blocks"][name].update(settings)
inputs = {"x": {"s": 1, "wl": 16, "fl": 14, "ints": [0, 4096, -4096, 0]}}
Path("replay.json").write_text(json.dumps(dict(design=design, inputs=inputs,
                                             hw=q.get("hw", {}), cost=True)))
PY
python3 -I engine/fxgraph < replay.json > replay-result.json
```

Check `replay-result.json` for `status: ok` and compare `summary.hwbits` with the plan. This four-sample replay exercises the assembled design. The original range and error evidence stays in the plan. For an existing model, the [SLX workflow](SLX.md) takes the selected block settings and writes types into a model copy.

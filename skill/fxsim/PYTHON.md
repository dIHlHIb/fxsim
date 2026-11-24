# Python modeling and fixed-point data

Run Python from the worktree containing `fxsim/`, `engine/` and `ref/`. If running a script in a subdirectory, add the worktree to `PYTHONPATH`. The engine writes its run log under the writable root selected by `FXSIM_ROOT` or by the package location. The first screen of SKILL.md contains a complete delay model.

`fi(x,T,F)` and `fi(x,s,w,f,F)` quantize scalars and one-dimensional Python sequences. Use `fi(x,T,'RoundingMethod','Floor','OverflowAction','Wrap')` for named properties. A local arithmetic policy can be copied with `fimath(F,'OverflowAction','Saturate')`. The available modes include FullPrecision, SpecifyPrecision, KeepMSB and KeepLSB for products and sums. Give word and fraction lengths for specified precision. The release arithmetic handles real add, subtract, elementwise multiply, scalar expansion, negate, abs, dot, divide(T,a,b), bitsll and bitsra. Two vectors of equal length multiply elementwise. Use `dot(a,b)` or `a @ b` for a real vector inner product.

```python
from fxsim import fi, fimath, numerictype, double, storedInteger
F = fimath('RoundingMethod','Floor','OverflowAction','Wrap',
           'SumMode','SpecifyPrecision','SumWordLength',8,'SumFractionLength',3)
T = numerictype(1,8,3)
a = fi([1.25,-0.5],T,F)
b = fi(0.5,T,F)
assert storedInteger(fi(a*b,T,F)) == [5,-2]
assert double(a[0]+b) == 1.75
```

Use Python's zero-based indexing and slicing on fi sequences. Values are immutable. Use lists to hold delay lines and scalar fi states. `x.WordLength`, `x.FractionLength` and `x.Signed` give the type. `double(x)` and `storedInteger(x)` return a number for one element, a list for several. The methods `x.double()` and `x.storedInteger()` always return lists. Complex construction quantizes each component with the declared type. Use `real(x)` and `imag(x)` for component arithmetic. Types and intermediate results support 1–128 bits, at least two when signed, with fraction lengths from −128 to 128. Matrix operations, general complex arithmetic and implicit mixed fi/float arithmetic are outside this subset.

Use Simulink library paths and original dialog text in `add_block`. Use `set_param(block,'Gain','0.5')` and `add_line(m,'u/1','G/1','autorouting','on')`. The supported path and parameter list is in `fxsim/catalog.json`. `design_json(m)` returns the explicit graph with default connection port numbers omitted. `sim(m, inputs={'u':u})` uses the input length by default. Set model StopTime or pass `stop_time` for a specific horizon in model time. Inputs must share a positive zero-phase clock to expose the solver grid. A multirate stream keeps its own `.Values.Time`.

```python
p = get_param(m+"/d", "PortHandles").Outport
set_param(p, "DataLogging", "on", "DataLoggingNameMode", "Custom", "DataLoggingName", "memory")
out = sim(m, inputs={"x": x})
assert out.logsout.getElement("memory").Values.Time == out.tout
set_param(p, "DataLogging", "off")
```

`out.get('yout').getElement(1).Values` uses a 1-based Dataset index. Root outputs can also be looked up by block name. `out.tout` is the solver clock. Each signal has `Data`, `Time`, `DataType` and fixed-point or boolean `storedInteger` codes. Vector outputs expose their component names. Catch `FxError` and inspect `.detail` for the engine code, location and reason.

`save_system(m,'modelname.slx',inputs=signals)` writes a flat model, companion I/O, a manifest with output port order, period and offset, and a native check function. The filename stem must match the model. The default is the model filename. To export a replay package, pass a new directory name, for example `save_system(m, "delay_package", inputs=signals)`. The returned dictionary contains the package path, model, manifest and checker command. The last successful simulation's inputs are reused if omitted. The flat writer accepts scalar real/boolean outputs and requires internal logging to be turned off for saving.

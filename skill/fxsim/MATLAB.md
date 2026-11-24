# Using the MATLAB batch entry point

Use `fxsim-matlab -batch` exactly as you would use `matlab -batch` for supported scripts. From the unpacked package root, run `export PATH="$PWD/engine:$PATH"` once in the current shell. Scripts can run from any working directory. Each invocation starts a fresh workspace. Failures exit nonzero and print the operation, source line and structured error with the engine's code, location and message.

```sh
fxsim-matlab -batch "v=fi([-1.25;2.5],1,8,2); assert(isequal(double(storedInteger(v)),[-5;10]));"
fxsim-matlab -batch "run('delay_demo.m')"
```

Save this numeric function as `accumulate_frame.m`.

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

Run `fxsim-matlab -batch "y=accumulate_frame([31;2]); assert(strcmp(bin(y),'10000100'));"` from that directory. The eight-bit accumulator wraps to the stored integer -124.

Save this example as `delay_demo.m`. The same statements serve as a native MATLAB test on a licensed host.

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

The destination contains the flat model, `<model>_io.json`, `<model>_manifest.json` and `check_<model>.m`. Each root output has a physical period and offset. The native checker reads that same SLX and verifies the supplied stimulus, output bits, types, shapes and times. It enables Dataset output for the check. In a fresh MATLAB session in that directory, call `check_delay_demo`. To compare against an independently saved model with the same name, pass the absolute path of that model to the check function. It runs both files, compares root block types, dialog readbacks, port connections and selected model settings, and reports layout differences. A newly issued manifest records the native check as pending.

### Supported script forms

- Function files can take arguments and call local functions. Script and function execution supports numeric arrays, cells, scalar structs, element and column-major indexing, indexed assignment and vector growth, loops, conditional branches, switch cases, early returns, and expression-only anonymous functions. Each batch starts a new workspace.
- Binary-point `fi` supports six rounding rules, saturate/wrap, the four product and sum modes, and CastBeforeSum. The supported operations include explicit casts, elementwise add/subtract/multiply, scalar products, comparisons, unary minus, vector sums, stored integers and local fimath. Use `fi(x,T,F)` for explicit type and arithmetic, and `bin` on a scalar to inspect its stored bits. Word lengths up to 128 bits and fraction lengths from -128 to 128 are accepted by the numeric frontend. Intermediate types outside these bounds cause a refusal.
- Helpers cover real mathematical functions, integer operations, file-relative binary input, and explicit real/imaginary parts for supported complex operations. Complex fixed-point products quantize each real product and the real and imaginary sums under the selected fimath. Built-in calls to `butter` and `roots` return `UNSUPPORTED` with the function name.
- Use flat block paths from the modeling catalog and the MATLAB dialog text. Use `new_system`, `add_block`, `add_line`, `delete_line`, `set_param`, `get_param`, `sim`, `save_system`, `open_system`, `load_system`, `close_system` and `bdclose`. A saved fxsim flat package can be edited after reopening. Arbitrary third-party SLX import, general handles and interactive windows are outside this entry point.
- Scalar root streams use a positive common input period, discrete fixed-step solver and an explicit or supported inherited input type. External Dataset elements bind in root port order. `yout` follows Outport order, `tout` follows the supported solver grid, and `logsout` contains explicitly marked single-output signals with custom names. Read `.Values.Time` and `.Values.Data` for column vectors. Non-scalar or complex logged signals and persistent internal logging in saved models need additional coverage.
- Explicitly typed From Workspace sources support scalar timeseries on their declared grid. If a source variable is absent at save time, its name and parameters remain in the SLX. The checker uses a typed zero witness and lists those sources in the manifest. Running the model itself requires the variable. Use the desired stimulus before saving to bind the native check to that input.
- Model save currently requires an exportable design and an output with a clock. A workspace session, third-party toolbox functions, full MATLAB language coverage and arbitrary `fi` arithmetic are outside the current implementation. Unsupported constructs and settings produce named errors. For replay counts and known differences, see [Known limits and verification](../../README.md#known-limits-and-verification).

### Test a function from a separate script

Use `addpath('helpers')` to call a function in a subfolder named `helpers`. The current folder takes precedence over added folders. A script run by path starts in its own folder, so relative input filenames resolve there. A function file can call a local function, and a script can define local functions after its statements.

A single foreground batch can read the stimulus, evaluate several parameter choices, and print each check. Use `tic; ...; t=toc;` around a section or save a timer with `h=tic; ...; t=toc(h);`. Waiting for a foreground process returns when its results are available.

- Read binary samples with `fid=fopen('samples.bin','rb','ieee-le'); assert(fid>=0); [x,n]=fread(fid,[32 4],'double'); fclose(fid);`. Scalar counts and `Inf` return columns. A two-element size reads complete columns. The input class can be retained with `'*int16'`. Binary writes accept `fwrite(fid,x,'double')`. File operations support native, little-endian, and big-endian data. Relative file names resolve against the working directory. With the caller’s permissions, write mode creates or truncates the named file and append mode creates or extends it, including outside the package.
- Double checks include `mean`, `find`, `log`, `log2`, `log10`, `cumsum`, vector `conv`, and zero-state `filter(b,a,x)`. Fixed-point inputs require an explicit conversion for double-only helpers. `getenv` reads a named environment variable.
- Row and column vectors follow MATLAB's implicit expansion rules. For example, multiplying a column elementwise by a row produces a matrix. Match their orientation to obtain pairwise products. Large real outer operations support scalar indexing, size queries, sum and mean up to 128 million elements. Other materialized arrays retain the 2 million element limit.

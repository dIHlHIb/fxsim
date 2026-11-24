# MATLAB Function blocks

`engine/fxgraph` runs a `MATLABFunction` block whose script stays inside a recorded subset. `engine/fxslx` reads
such blocks from a saved `.slx`. Every rule below comes from Simulink R2026b recordings; anything outside the subset is refused,
never guessed.

## The block in a design

```json
{"type": "MATLABFunction", "Script": "function [y, z] = f(a, b)\n...",
 "OutDataTypeStr": "Inherit: Same as Simulink",
 "TreatAsFi": "Fixed-point",
 "EmlDefaultFimath": "Same as MATLAB Default",
 "InputFimath": "fimath('RoundingMethod','Floor','OverflowAction','Wrap')",
 "SaturateOnIntegerOverflow": "on"}
```

- Inputs are numbered in the order of the main function's arguments, outputs in the order of its return values
  (`sport` on lines). The k-th output's declared type is `OutDataTypeStr` for k = 1 and `OutDataTypeStrk` after that.
- Signals map to MATLAB types as follows:
  - `fixdt(s, 8/16/32, 0)` is int8 .. uint32;
  - other fixed-point signals are `fi`;
  - boolean is `logical`.

  There are no double signals: a function whose output is double is refused (`UNSUPPORTED`).
- **Errors:**
  - `UNSUPPORTED_MATLAB`: a construct or rule outside the recorded subset;
  - `BAD_MATLAB`: code that MATLAB itself rejects.

  The error's `where` is `design.blocks.<name>.Script`, and the message gives the line.
- **Execution paths:**
  - An exact interpreter (`engine/fxmlsem.py`) is the reference.
  - Most functions are also compiled into engine blocks (`engine/fxmlgen.py`), so they run on the C core and in the
    code generators.
  - A construct the compiler cannot express exactly stays on the interpreter (Python path) and gives the same
    answer.
  - `engine/fxexport` writes MATLAB Function blocks as Simulink saves them (the function's script, block fimath,
    TreatAsFi, integer saturation and declared output types) and reads the package back before it is handed out.
    In our 21-package test set, Simulink matched fxsim bit for bit across 960 steps and reported no warnings.

## Language subset

**Statements:**
- `x = e`, `x(:) = e` (keeps x's type, quantises e into it).
- `[p, q] = g(...)` and `[p, ~] = g(...)` of a local function. Refused when g has persistent variables, or when an
  output name is also read by the arguments.
- `if` / `elseif` / `else`; `switch v` / `case c` / `case {c1, c2}` / `otherwise`:
  - the subject is an integer variable or expression (`switch (u)`, `switch uint8(u)`, `switch st.m`, evaluated
    once); cases are numeric constants or constant variables (`K = 3; case K`), lists with commas or spaces;
  - a switch on a fixed-point value is refused, as Simulink refuses it.
- `for k = a:b` / `a:s:b` with integer constant bounds: literals, constant variables (`N = 8; for k = 0:N-1`) or
  fixed-point properties (`1:a.WordLength`).
- `persistent p` with initialisation under `if isempty(p)`.
- `coder.extrinsic('f', ...)`, then `warning(...)` calls whose message is text, variables and `num2str` (it changes no
  value).
- `st.f(:) = e`: keeps the field's type, quantised and overflowed by the block fimath.
- A bare `coder.unroll` line.

**Expressions:**
- Numbers and variables; `+ - *` and `.*`; unary `-`; comparisons; `&& || & | ~`.
- `^` / `.^`:
  - of constants (folded when exact in double);
  - `a ^ 2` of a fixed-point value.
- `x(:)` of a scalar.

**Fixed-point properties:** `x.WordLength`, `x.FractionLength`, `x.Signed` and `x.numerictype` (inside `fi`) of a
fixed-point value, folded at compile time. A double variable assigned once at the top level of its function from a
constant expression (`k = a.WordLength - 4`) is a constant.

**Structs of scalar fields:**
- Built with `st = struct('a', e1, 'b', e2)`, also `persistent` (register bundles); fields read and written as
  `st.a`.
- Also built field by field (`s.a = u; s.b = u + 1`, a persistent one inside its `if isempty(r)`), with Simulink's
  conditions:
  - the same field order on every path;
  - no field read before it is assigned;
  - every field of a persistent struct given in its initialisation.

  A field created in a loop or on some paths only is refused.
- Assigned as a whole to another struct (`st = next`), when both have the same field names, order, count and types.
  Simulink rejects any difference; the engine does too.
  - `next = st` followed by field writes copies the values.
  - `st = struct('c', f(st.c), ...)` reads st as it was before the statement.
- Refused:
  - fields of input buses;
  - a struct passed, returned or indexed;
  - a copy that is never changed, made inside a loop, or given new fields.

**Calls:**
- `fi`, `ufi`, `numerictype`, `fimath`, `hdlfimath` (also without parentheses, or as `F = hdlfimath`), `cast(x, 'like',
  y)`, `removefimath`.
- `int8 .. uint32`, `logical`, `true`, `false`, `isempty`.
- `min`, `max`, `abs`, `floor`, `ceil` (signed fixed point with a fraction), `nextpow2` and `log2` (constants),
  `getmsb`, `bitcmp` (fixed point), `mod(integer, positive integer constant)` (`mod` of a fixed-point value is
  refused, as Simulink does).
- Bit functions: `bitand`, `bitor`, `bitxor`, `bitshift`, `bitsll`, `bitsrl`, `bitsra`, `bitsliceget`, `bitconcat`,
  `bitget`, `bitset`.
- The file's own local functions.

**Row vectors (state and outputs):** `fi(zeros(1, n), s, w, f)`, `zeros(1, n, 'like', x)`, `int16(zeros(1, n))`,
`false(1, n)`; reads `h(k)` (k constant or computed at run time; out of range is refused when it happens) and
`h(a:b)`; writes `h(k) = e` (kept in the element type, by the block fimath) and `h(a:b) = v`; rows `[a, h(1:3)]`;
a vector output. These run on the Python path.

**Refused:**
- `while`, `try`, `break` / `continue` / `return`;
- matrices, columns `[a; b]`, stepped slices and indexing with two indices;
- cell arrays, strings outside the calls that take name / value pairs, function handles;
- division other than by a double constant of at least 1; transpose, `global`, `eval`, and any other call.

## Arithmetic (as recorded)

- **Default fimath:** Nearest (ties toward +inf), Saturate.
  - With `EmlDefaultFimath = Other:UserSpecified`, the block's `InputFimath` replaces it. `hdlfimath` is Floor /
    Wrap.
  - `ProductMode` / `SumMode` `FullPrecision` are accepted (products and sums are always full precision).
- **Full precision:**
  - Sums take the larger fraction length and one more integer bit than the larger operand.
  - Products add word and fraction lengths.
- **fi and a double constant:**
  - with `+` / `-`, the constant is quantised to the fi's type;
  - with `*`, it takes the fi's word length at the best precision: the largest fraction length at which the
    constant fits, one less when rounding it to nearest there would overflow (0.99999 in 16 bits is 1.0 at 2^-14).
- **`fi(x, s, w, f)` with no fimath argument:**
  - a fixed-point `x` keeps its own (or the block's) fimath;
  - a constant or integer `x` always rounds Nearest / Saturate (`fi(0.3, 1, 8, 4)` = 5/16 even under a Floor / Wrap
    block fimath).
- **Integers:**
  - the same class on both sides;
  - saturate or wrap by `SaturateOnIntegerOverflow`, the conversions `int8()` .. `uint32()` included;
  - an integer op a double constant is computed in IEEE double, then rounded half away from zero (`int16(90) * 0.35`
    = 31).
- **Division by a double constant:** the result keeps the dividend's type. A fixed-point value is rounded by the
  fimath (Nearest by default, so 128/3 -> 43); an integer is divided in double and rounded half away from zero
  (101/3 -> 34).
- **`cast(x, 'like', y)`** takes y's type and rounds by the block fimath. x may be fixed point, an integer, or a
  compile-time double (a literal, a constant expression such as `0.25 + 2^-16`, or a double variable assigned one
  constant once): `cast(0.7071, 'like', a)` is 0.7071 quantised to a's type by the block fimath (default: Nearest,
  Saturate; a Floor / Wrap block fimath floors and wraps: 1.5 into sfix16_En15 gives 32767, or -16384 with Wrap),
  and `cast(c, 'like', a) * a` is the full-precision product. A constant that does not fit a's type makes Simulink
  warn `Coder:builtins:ConstantFoldingOverFlow` while compiling; the export package allows that warning for the
  block. A double that is not a compile-time constant, or a y carrying its own fimath, is refused.
- **Name / value fimath properties:** `fi(x, s, w, f, 'OverflowAction', 'Saturate')` takes the properties it does
  not name from x's fimath; `fimath('Name', value)` from the block fimath.
- **Truth values:** a fixed-point or integer value used as a condition or a logical operand is true when non-zero.
- **Fixed-point values are limited to 67 bits** (any port, variable or intermediate); a wider one is refused. This
  covers the usual wide forms: a multiply-add `fi(a*b + c*c, 1, 16, 8)` of 32-bit inputs (65 bits), a 64-bit
  accumulator `s(:) = s + a*b`, a 3-tap FIR on 32-bit samples (67 bits), and output ports of a full-precision product
  or sum up to 64 bits (a sum of two 63-bit values is 64 bits, as in Simulink).

## Block and chart properties read from a `.slx`

- `vectorOutputs1D`, `supportVariableSizing`: 0 / 1.
- `FimathForFiConstructors`: both values; no recorded difference.
- Chart `updateMethod`:
  - inherited;
  - DISCRETE with a `sampleTime` equal to the model's step (another one makes a multirate model, refused; none at
    all is refused, as Simulink does);
  - CONTINUOUS.
- `allowDirectFeedthrough` 0 is refused.
- Editor-only properties (`windowPosition`, `viewLimits`, `screen`, ...) are ignored.
- Any other chart or function property is refused.

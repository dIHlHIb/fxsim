# HDL-style blocks: accepted settings (generated)

These settings come from the engine’s parameter tables.

What is accepted comes from Simulink R2026b recordings, in two forms:

- **sets**: the exact values or combinations recorded (tables below marked so, e.g. the deep RAMs, the
  wide counters, the FIR input types);
- **ranges**: for some numeric parameters (the HDL Counter's word length, limit, step, initial value), any
  value between the lowest and the highest recorded one, each parameter checked on its own -- so a
  combination inside the ranges may not have been recorded as such.

Anything outside is refused with `UNSUPPORTED`; the message says what is recorded. Parameter names and
defaults: [BLOCKS.md](BLOCKS.md).

## HDLRAM (HDL RAMs: Simple dual port, Dual port, Single port)

- Behaviour: the output is registered (the read of step t appears at t + 1, the initial value before);
  an unwritten word reads the initial value; a read and a write of one address in the same step give the
  old value on the read port; the write port (Dual / Single port) gives the new or old data per
  `WriteOutputValue`.
- AddressBits 2, 3, 4, 5, 6, 7, 8 (Simulink refuses fewer than 2); up to 8 bits: data of 3, 8, 12, 16, 18, 24 bits or boolean; 32-bit data
  only as int32, sfix32_En16, uint32, with 8 address bits.
- Deep RAMs (AddressBits 9, 10, 11, 12, 13, 14, 15, 16; Python path only), each combination as recorded (initial value 0,
  `WriteOutputValue` New data; Old data on a Single port RAM with 9 / 10 / 13 address bits):

| AddressBits | data |
|---|---|
| 9 | uint16 |
| 9 | sfix16_En8 |
| 9 | boolean |
| 10 | uint8 |
| 10 | int16 |
| 10 | sfix16_En8 |
| 10 | int32 |
| 10 | boolean |
| 11 | uint8 |
| 11 | sfix16_En15 |
| 11 | sfix16_En8 |
| 12 | sfix16_En8 |
| 12 | int8 |
| 13 | sfix16_En8 |
| 13 | boolean |
| 14 | uint8 |
| 14 | sfix16_En8 |
| 15 | sfix16_En8 |
| 15 | boolean |
| 16 | sfix16_En8 |

- A non-zero initial value on a deep RAM: Simple dual port, 10 address bits, sfix16_En8 data.
- Inside an enabled subsystem (held or reset outputs, StatesWhenEnabling held or reset, Synchronous): the
  RAMs of up to 8 address bits, gated like any state (with StatesWhenEnabling reset every word returns to
  0 when the enable rises); deep RAMs are refused there.
- Addresses: exactly ufix(AddressBits) (uint8 for 8 bits), as Simulink requires (other address types are
  refused there with InputPortDataTypeMismatch; the engine refuses them too).
- The library blocks Simple Dual Port / Dual Port / Single Port RAM System run as the RAM of their type; the
  Dual Port RAM System also with RAMType Single port or Simple dual port. True dual port and Simple tri
  port are refused.

## HDLCounter

- Each step the counter outputs its value, then updates: reset > load > enable; Free running adds / subtracts
  the step modulo the word; Count limited goes back to `CountInit` when the value is exactly `CountMax`
  (whatever the direction), otherwise steps and wraps; Modulo counts `CountInit` .. `CountMax`.
- `CountSampTime` 1 only; it is the period of a counter without input ports. A counter with input ports
  runs at its ports' rate (all ports at one period, no offset; recorded: ports at period 2 or 3 count
  once per port sample). count_hit is the condition AND NOT reset AND NOT load; a Count
  limited hit also fires when the next step leaves the word. Modulo takes fraction length 0 only.
  A negative step, or the direction port false, counts the other way in every type. count_hit and the
  direction port together are refused (Simulink: mutually exclusive).
- **Swept ranges** (a systematic recording of 108 counters): inside these word and fraction lengths any
  CountMax, CountInit and CountStep the type holds is accepted (CountFrom keeps the table below). A Free
  running counter does not use CountMax: any value is accepted, even one the word cannot hold.
  The fraction length is at most the word length when unsigned and below it when signed (Simulink refuses
  the rest). CountSampTime other than 1 is refused (recorded, not supported yet):

| CountType | unsigned word lengths | signed word lengths | fraction lengths |
|---|---|---|---|
| Count limited | 1 .. 34 | 2 .. 32 | 0 .. 4 |
| Free running | 1 .. 33 | 5 .. 32 | 0 .. 3 |
| Modulo | 1 .. 32 | 2 .. 32 | 0 .. 0 |

Outside the sweep (e.g. CountSampTime other than 1 is refused anyway, a signed Free running counter below 5 bits):

| parameter | recorded range (each on its own) |
|---|---|
| `CountWordLen` | 4 .. 16 |
| `CountFracLen` | 0 .. 2 |
| `CountMax` | 5 .. 25 |
| `CountStep` | -1 .. 5 |
| `CountInit` | -3 .. 10 |
| `CountFrom` | -3 .. 10 |

Beyond those ranges, these exact settings only (unsigned, fraction length 0, no count_hit output):

| CountType | CountWordLen | CountMax | CountStep | CountInit | ports (reset, load, enable, direction) |
|---|---|---|---|---|---|
| Count limited | 1 | 1 | 1 | 0 | on off on off |
| Count limited | 10 | 1023 | 1 | 0 | off off off off |
| Count limited | 10 | 1023 | 1 | 923 | off off off off |
| Count limited | 16 | 65535 | 1 | 0 | off off off off |
| Count limited | 16 | 65535 | 1 | 0 | on on on on |
| Count limited | 16 | 65535 | 1 | 65435 | off off off off |
| Count limited | 16 | 65535 | 1 | 65500 | on off on off |
| Count limited | 2 | 3 | 1 | 0 | off off on off |
| Count limited | 20 | 25 | 1 | 0 | off off off off |
| Count limited | 3 | 7 | 1 | 0 | off off off off |
| Count limited | 3 | 7 | 1 | 0 | off off on off |
| Count limited | 8 | 200 | 1 | 0 | off off off off |
| Count limited | 8 | 200 | 1 | 0 | off off off on |
| Count limited | 8 | 239 | 1 | 200 | on off on off |
| Count limited | 8 | 255 | 1 | 0 | off off off off |
| Free running | 12 | - | 100 | 0 | off off off off |
| Free running | 2 | - | 1 | 0 | on off on off |
| Free running | 20 | - | 1 | 0 | off off off off |
| Free running | 20 | - | 30000 | 0 | off off off off |
| Free running | 24 | - | 1 | 0 | off off off off |
| Free running | 24 | - | 1000 | 0 | off off off off |
| Free running | 24 | - | 100000 | 0 | off off off off |
| Free running | 3 | - | -1 | 0 | off on on off |
| Free running | 3 | - | 1 | 0 | off off off off |
| Free running | 3 | - | 1 | 0 | on off on off |
| Free running | 32 | - | 1 | 0 | off off off off |
| Free running | 32 | - | 1 | 0 | off off on off |
| Free running | 32 | - | 1 | 0 | on on on on |
| Free running | 32 | - | 50000000 | 0 | off off off off |
| Free running | 9 | - | 1 | 480 | on off on off |
| Modulo | 10 | 1000 | 1 | 0 | off off off off |
| Modulo | 10 | 1000 | 1 | 900 | off off off off |
| Modulo | 24 | 4915199 | 1 | 0 | off off on off |
| Modulo | 24 | 4915199 | 1 | 4915190 | off off on off |

## UnitDelayEnabledSynchronous, UnitDelayEnabledResettableSynchronous, UnitDelayResettableSynchronous

- Enable and reset inputs: boolean, ufix1 or 8 / 16 / 32-bit integers (others refused like Simulink); a
  signed enable enables only when it is > 0, a reset is any non-zero value.

## Delay with an enable and / or reset port

- `DelayLength` 1 .. 64 with (ShowEnablePort, ExternalReset) off / Rising; on / Level; on / None (the recorded sweep).
- `DelayLength` 1, 2, 3, 4, 6, 8, 16, 24, 32, 64 with the other recorded combinations: off / Either; off / Falling; off / Level; off / Level hold; on / Either; on / Falling; on / Level hold; on / Rising.
- Simulink's behaviour: with the enable low the output holds its previous value (0 before
  the first enabled step) and the line holds; a Level reset drops its step's input when the next step is
  enabled; the other resets take it.
- A uint8 enable is accepted besides a boolean one (non-zero enables).
- A scalar initial condition; an out-of-range one is refused like Simulink (parameter overflow).

## HDLFIRFilter and HDLFIRDecimator (DSP HDL Toolbox)

- Structures: Direct form systolic, Direct form transposed, Partly serial systolic; Partly serial `NumCycles` 2, 3, 4, 5, 6, 7, 8, 9, 11, 12, 16, 19; Decimator factors 2, 3, 4, 8.
- Inputs: signed 12, 16, 18, 24 bits on every structure; further types only with the recorded structures and coefficients:

| input | structure | coefficient sets |
|---|---|---|
| ufix16_En16 | Direct form systolic | [0.1 -0.2 0.35 0.35 -0.2 0.1]; [0.1 0.2 0.2 0.1] |
| ufix16_En16 | Direct form transposed | [0.1 -0.2 0.35 0.35 -0.2 0.1] |
| uint8 | Direct form systolic | [0.1 -0.2 0.35 0.35 -0.2 0.1]; [0.1 0.2 0.2 0.1]; [0.4 0.4 0.4 0.4 0.4 0.4]; [0.9 -0.7 0.6 0.6 -0.7 0.9]; [-0.1 -0.2 -0.2 -0.1] |
| uint8 | Direct form transposed | [0.1 -0.2 0.35 0.35 -0.2 0.1] |
| sfix14_En13 | Direct form systolic | [0.1 -0.2 0.35 0.35 -0.2 0.1] |
| sfix20_En19 | Direct form systolic | [0.1 -0.2 0.35 0.35 -0.2 0.1] |

- The full-precision output is signed unless the input is unsigned and every coefficient non-negative.
- Explicit output / coefficient types, rounding and saturation: recorded on input sfix16_En15 only; on
  transposed / Partly serial only Floor, Nearest, and Floor with saturation.
- The FIR timing tables are an accept-list of measured configurations and their pipeline latencies. The partly serial filter and decimator refuse configurations that require a missing table entry. A tuple specifies the architecture, tap count, symmetry and cycle count, plus the rate factor and coefficient class where applicable. Each accepted input stream is filtered by convolution with its own coefficients. The latency determines when each result becomes valid.
- Partly serial latencies are tabled per (NumCycles, taps, symmetric); `valid` must follow the recorded
  rhythm (every NumCycles steps for the newer entries). Reset port: Direct form systolic; transposed and
  Partly serial (and the Decimator by 2) with the recorded coefficients only.
- Partly serial with `valid` always high (from step 0): with N = min(NumCycles, taps), `ready` is 1 every
  2N steps and the two inputs at those steps and the next are taken, the others dropped; recorded for
  (NumCycles, taps) (2, 6 sym), (2, 8), (3, 6 sym), (3, 8), (4, 6 sym), (4, 8), (8, 6 sym), (8, 8) on input sfix16_En15.
- `ready` is low for min(NumCycles, taps) - 1 steps after each valid sample; a sample sooner than
  NumCycles steps after the previous one is refused (also where `ready` is already high again).
- Settings recorded by the NumCycles sweep are taken on input sfix16_En15 with every
  other parameter at its default: Partly serial (NumCycles, taps) (3, 5), (3, 12), (3, 12 sym), (3, 32), (3, 32 sym), (5, 5), (5, 9), (5, 9 sym), (5, 24), (5, 24 sym), (6, 8 sym), (6, 12), (6, 12 sym), (6, 32), (6, 32 sym), (7, 4), (7, 4 sym), (7, 9), (7, 9 sym), (7, 16), (7, 16 sym), (8, 5), (8, 8 sym), (8, 12), (8, 12 sym), (8, 24), (8, 24 sym), (9, 9), (9, 9 sym), (11, 8), (11, 8 sym), (12, 8 sym), (12, 9), (12, 16), (12, 16 sym), (12, 32), (12, 32 sym), (16, 4 sym), (16, 9), (16, 9 sym), (16, 16 sym), (16, 24), (16, 24 sym), (19, 16), (19, 16 sym).
- Decimator `NumCycles` 1 (default) or, with `valid` exactly every NumCycles steps from step 0, (factor, NumCycles, taps): (2, 2, 32 sym), (2, 5, 8 sym), (3, 2, 8 sym), (3, 3, 16 sym), (3, 6, 24 sym), (4, 2, 16 sym), (4, 4, 24 sym), (4, 7, 32 sym), (8, 2, 16 sym), (8, 8, 24 sym), (8, 11, 32 sym).
  With NumCycles above 1 the coefficients must be truly symmetric (not antisymmetric), every polyphase
  branch (taps k, k+D, ...) must hold a nonzero tap and, over two or more free coefficients, two distinct
  nonzero magnitudes. When every branch reads the same forwards and backwards the latency
  moves by a recorded offset per key measured in clock steps: (factor, NumCycles, taps) (2, 2, 32) +0, (2, 5, 8) -1, (3, 2, 8) +0, (3, 3, 16) +0, (3, 6, 24) -3, (4, 2, 16) -1, (4, 4, 24) -2, (4, 7, 32) -3, (8, 8, 24) +0, (8, 11, 32) -1; that class is
  supported on those keys only. The factor-8, two-cycle, 16-tap palindromic case is refused.
- Coefficients are quantised from the double each literal denotes, as Simulink does (a 17-digit literal can
  be a rounding tie as a double although its decimal text is not).

## HDLNCO (DSP HDL Toolbox)

- With the increment from the dialog, PhaseOffset 0, no dither and no phase offset / reset / phase ports
  (the recorded sweep): AccumulatorWL 6 .. 65; NumQuantizerAccumulatorBits 4 .. 19 and not above
  AccumulatorWL; outputs fixdt(1, WL, FL) with WL 8 .. 32, FL >= 6 and WL - FL 1 .. 4.
- The output is round-half-up(sin * 2^FL), saturated symmetrically to +-(2^(WL-1) - 1): with FL = WL - 1,
  sin = -1 gives -(2^(WL-1) - 1). A dialog increment is evaluated as a double (integers above 2^53 round),
  then added modulo 2^AccumulatorWL. Quantiser bits above the accumulator's are refused like Simulink.
- With exactly one of these on (the recorded sweep): dither of 1 / 4 / 6 / 8 bits, a phase increment
  port (int8 / int16 / int32 / uint16 not wider than the accumulator, or the accumulator's own width), the
  phase output (the quantised phase, ufix of the quantiser bits), a phase offset (a dialog value or a
  ufix(AccumulatorWL) port): AccumulatorWL 8 .. 48, NumQuantizerAccumulatorBits 6 .. 16, the same outputs;
  valid gaps allowed (the accumulator and the dither advance on valid samples only; an output due for an
  invalid sample is 0).
- Otherwise (a reset port, or several of these together): AccumulatorWL 12, 16, 20, 24, 32;
  NumQuantizerAccumulatorBits 8, 10, 12, 14, 16, 18; output types fixdt(1,12,10), fixdt(1,16,14), fixdt(1,18,16).
- A Complex exponential output is a complex signal: parts of at most 16 bits.
- In the other settings, dither: 1, 4, 6 bits with a 16-bit accumulator (and 4 bits with a 20-bit one).
- In the other settings, a phase increment port (port type, AccumulatorWL, quantiser bits): uint16 16/12; uint16 20/12; int16 16/12; int16 20/12; int32 32/12; int32 32/16; int8 16/12.
- In the other settings, valid gaps with an increment port only for int16 16/12, Sine and cosine, default output.

## HDLFIFO (HDL RAMs: HDL FIFO)

- Classic mode. Every output is registered: a step shows the state after the step before (Out 0, Empty
  true, Full false, Num 0 at the start). Each step pops first (Pop on a non-empty FIFO takes the head,
  the next Out; an empty Pop does nothing), then pushes (Push writes In when the FIFO is not full or the
  same step popped; otherwise the write is lost). Num is ufix(ceil(log2(fifo_size + 1))); Out has In's
  type.
- fifo_size 4 .. 128 (Simulink rejects 2 and 3); In boolean or fixed point of 8 .. 32 bits with
  0 <= FL < WL. Inside an enabled subsystem it is gated like any state.
- Output ports (show_empty, show_full, show_num): (off, off, off); (off, off, on); (off, on, off); (on, off, off); (on, on, off); (on, on, on).
- push_msg / pop_msg: Ignore / Ignore; Warning / Warning (the values are the same); Error is refused.
- Refused: FWFT mode, the reset port, ratio or dataDim other than 1, Push / Pop other than boolean.
- Export: only with push_msg and pop_msg Ignore (with Warning, Simulink warns when the FIFO is pushed full or
  popped empty, and the check script fails on any warning).

## Bit operations (Bit Slice, Bit Concat, Bit Shift, Bit Rotate, Bit Set, Bit Clear)

- **BitSlice**: the bits ridx .. lidx of the input's two's-complement pattern, as an unsigned integer of
  lidx - ridx + 1 bits; bits above the input word read 0 (no sign extension). Inputs of 1 .. 128 bits,
  signed or unsigned, fraction lengths -3 .. 31 (the result ignores the scaling); 0 <= ridx <= lidx,
  lidx up to 200, ridx up to 127. ridx > lidx and negative indices are refused like Simulink.
- **BitConcat**: the inputs' bit patterns side by side, input 1 most significant; a boolean is 1 bit; the
  output is an unsigned integer as wide as the inputs together (shown uint8 / 16 / 32 / 64 at those
  widths). numInputs 1 .. 128; each input boolean or a fixed-point word, output up to 326 bits, inputs up to 64 bits. Fraction lengths: any
  for 2 .. 4 inputs; -16 .. 48 for other counts.
- **BitShift** (Shift Left Logical, Shift Right Logical, Shift Right Arithmetic): N 0 .. 95; output up to 96 bits. **BitRotate** (Rotate Left, Rotate Right): N 0 .. 64; output up to 128 bits.
- **BitSet**: iBit -1 .. 51; **BitClear**: iBit -1 .. 51; output up to 128 bits.
- Bit Slice, Bit Shift, Bit Rotate, Bit Set and Bit Clear take no boolean input. Words over 126 bits
  (Bit Slice) or 120 bits (Bit Concat) run on the Python path only.

## Math Function, Repeat, Increment / Decrement Real World

- **Math Function** (fixed-point inputs):
  - `square`, `magnitude^2`, `reciprocal`: any fixed-point input, the block's RndMeth and saturation, an
    explicit output type or the internal rule; element by element on vectors. The reciprocal's internal
    rule keeps the input's signedness, fraction length wl - 1 - s - fl, and on host hardware rounds the
    word length up to the next native word (8 / 16 / 32); 1 / 0 is the type's maximum.
  - `rem` / `mod`: integer classes only (int8 .. uint32, either sign; other types are refused as Simulink
    refuses them), a constant or signal divisor. rem truncates (rem(u, 0) = 0), mod floors (mod(u, 0) = u);
    the output keeps the input's class whatever OutDataTypeStr, RndMeth or saturation say; vectors element
    by element.
  - `transpose`: the input unchanged with its type, whatever OutDataTypeStr says; vectors element by
    element; booleans too. `hermitian`: scalar sfix16_En8 only.
- **Repeat**: sample based (either RateOptions) or frame based multirate on a scalar: each input repeated N
  times at period P / N, starting with u[0]. Frame based single rate (the library default) outputs a vector
  and is refused; frame based on a vector too.
- **Increment / Decrement Real World**: 1.0 is put in the input type like a Bias -- nearest, half up,
  saturated to the maximum (an LSB of 2 adds one LSB, an LSB of 4 or more adds nothing; a type without
  integer bits adds its maximum); where 1.0 is beyond the maximum + 1 LSB Simulink stops and fxsim refuses.
- **Bit Set / Bit Clear**: iBit -1 .. 51 (-1, and a bit at or above the word length, leave the input
  unchanged); bit 0 of a boolean gives true / false. iBit and the other bit indices may be written in MATLAB
  integer forms (int8(4), 2+3, 3.0).

## Complex Constant

- A Constant whose `Value` is complex -- `complex(a)`, `complex(a, b)`, `a+bi`, `a-bi` or `bi` with numeric
  literals -- is a complex signal of its `OutDataTypeStr`; `complex(0)` and `complex(1)` are complex too (zero
  imaginary part). Both parts are quantized to that type like a real Constant. It feeds Complex to Real-Imag,
  Sum and Product like any complex signal; a complex root Outport reports `<name>.re` and `<name>.im`.
- `OutDataTypeStr` must be a fixed-point type: with the default `Inherit: Inherit from 'Constant value'` the
  Constant is complex double in Simulink, which fxsim does not simulate (refused, UNSUPPORTED).
- The export writes the Value text as given.
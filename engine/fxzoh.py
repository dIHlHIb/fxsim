"""Zero-Order Hold: one input, SampleTime -1 / 1 (the base step: the input passes
unchanged) or an integer k >= 2 (output at t = 0, k, 2k, ...: the input's value at that time, offset 0; the output
type is the input type). Lowered before the engine sees the design: a Signal Specification (identity) at the base
step, a Downsample by k (offset 0, sample based, multirate) otherwise. The recorded input was an Inport at the base
step; other sources are refused."""
TYPES = ("ZeroOrderHold",)


def has(spec):
    return any(isinstance(b, dict) and b.get("type") in TYPES for b in (spec.get("blocks") or {}).values())


def rewrite(spec, info):
    import fxgraph_lib as G
    blocks = dict(spec["blocks"])
    lines = spec.get("lines", [])
    for n in [n for n, b in blocks.items() if isinstance(b, dict) and b.get("type") in TYPES]:
        b = blocks[n]
        where = "design.blocks." + n
        extra = sorted(set(b) - {"type", "SampleTime"})
        if extra:
            raise G.GraphError("BAD_PARAM", where + "." + extra[0], "unknown parameter %s for ZeroOrderHold" % extra[0])
        ts = str(b.get("SampleTime", "-1")).strip()
        src = [ln for ln in lines if ln.get("dst") == n]
        if len(src) != 1:
            raise G.GraphError("BAD_CONNECTION", where, "Zero-Order Hold takes exactly one input")
        if ts == "-1":   # inherited: the input's rate, unchanged
            blocks[n] = {"type": "SignalSpecification"}
            continue
        if ts == "1":
            # A hold with period one is an identity only for a base-rate input here.
            s_ = blocks.get(src[0]["src"])
            if not (isinstance(s_, dict) and s_.get("type") == "Inport" and str(s_.get("SampleTime", "-1")).strip()
                    in ("-1", "1")):
                raise G.GraphError("UNSUPPORTED", where + ".SampleTime", ('Zero-Order Hold at SampleTime 1 fed by %s: only '
                    'an Inport at the base step was recorded (after '
                    'a slower signal Simulink holds at the base step)') % src[0]["src"])
            blocks[n] = {"type": "SignalSpecification"}
            continue
        try:
            k = int(ts)
            if str(k) != ts or k < 2:
                raise ValueError
            if k > 4:   # 2, 3, 4 recorded (recorded shapes only)
                raise G.GraphError("UNSUPPORTED", where + ".SampleTime", "Zero-Order Hold SampleTime %d: 2, 3 and 4 "
                                   "are supported." % k)
        except ValueError:
            raise G.GraphError("UNSUPPORTED", where + ".SampleTime", "Zero-Order Hold SampleTime %r: use -1, 1, 2, 3 or 4" % ts)
        s_ = blocks.get(src[0]["src"])
        if not (isinstance(s_, dict) and s_.get("type") == "Inport" and str(s_.get("SampleTime", "-1")).strip()
                in ("-1", "1")):
            raise G.GraphError("UNSUPPORTED", where, "Zero-Order Hold at SampleTime %s fed by %s: only an Inport at the "
                               "base step was recorded; feed it from an Inport or use a Downsample"
                               % (ts, src[0]["src"]))
        blocks[n] = {"type": "Downsample", "DownsamplingFactor": str(k), "SampleOffset": "0", "InitialConditions": "0",
                     "InputProcessing": "Elements as channels (sample based)",
                     "RateOptions": "Allow multirate processing"}
    return dict(spec, blocks=blocks)

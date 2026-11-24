"""Runtime helpers of the SolverMode Auto support domain (engine/auto_domain.json, written by
the regression check from the Auto recordings): the Rate Transition tuple and a block's mode signature, the
same functions the domain was built with. Kept apart from the regression check so the engine does not import the
build-time recording readers."""
def _period(text):
    """A sample-time text's period as a Fraction (None for -1 / inherited / not a plain number or [P o])."""
    from fractions import Fraction
    t = str(text).strip()
    if t.startswith("["):
        t = t.strip("[]").split()[0] if t.strip("[]").split() else ""
    try:
        v = Fraction(t)
    except (ValueError, ZeroDivisionError):
        return None
    return v if v > 0 else None


def rt_tuple(params, pin, pout):
    """(Integrity, Deterministic, direction, K) of a Rate Transition from its input / output periods: 'f2s' when the
    output is slower, 's2f' when faster, 'same'; K the integer ratio (None when not an integer)."""
    if pin is None or pout is None:
        return None
    r = pout / pin if pout >= pin else pin / pout
    k = int(r) if r.denominator == 1 else None
    d_ = "same" if pout == pin else ("f2s" if pout > pin else "s2f")
    return [str(params.get("Integrity", "on")).strip(), str(params.get("Deterministic", "on")).strip(), d_, k]


NUM = __import__("re").compile(r"^\s*[-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?\s*$|^\s*\[[-+0-9.eE ,;]*\]\s*$")


def mode_sig(bt, params):
    """A block's mode parameters under Auto (the domain listed only block types, so an unrecorded Rate
    Transition variant passed): every parameter with the library default filled in, except numeric values and data
    types, as sorted (name, value) pairs."""
    import blockspec as B_
    merged = dict(B_.DEFAULTS.get(bt, {}))
    merged.update({k: v for k, v in params.items() if k != "type"})
    return [[k, str(v).strip()] for k, v in sorted(merged.items())
            if not NUM.match(str(v)) and not k.endswith("DataTypeStr") and
            k not in ("SampleTime", "OutPortSampleTimeOpt")]   # the rates themselves are checked on the compiled periods

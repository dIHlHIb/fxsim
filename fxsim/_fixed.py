import math
import struct
from dataclasses import dataclass

from ._backend import FxError, core, options, require, root


@dataclass(frozen=True)
class numerictype:
    SignednessBool: int
    WordLength: int
    FractionLength: int

    def __post_init__(self):
        s, w, f = self.SignednessBool, self.WordLength, self.FractionLength
        require(type(s) in (bool, int) and s in (0, 1) and type(w) is int and type(f) is int,
                "numerictype", "Use binary-point (signed, wordLength, fractionLength).", "BAD_TYPE")
        require(not (s and w == 1), "numerictype", "Signed fi requires at least two bits.",
                "fixed:numerictype:invalidMinWordLengthCodegen")
        require(1 <= w <= 64 and -6 <= f <= w + 6, "numerictype", "Outside the recorded data constructor domain.")

    def record(self):
        return dict(s=int(self.SignednessBool), wl=self.WordLength, fl=self.FractionLength)

    def __str__(self):
        return "fixdt({s},{wl},{fl})".format(**self.record())


@dataclass(frozen=True)
class _Fimath:
    RoundingMethod: str
    OverflowAction: str


def fimath(*args, **kwargs):
    opts = options(args, kwargs, "fimath")
    supported = {"RoundingMethod": ("Nearest", "Floor", "Ceiling", "Round", "Zero", "Convergent"),
                 "OverflowAction": ("Saturate", "Wrap"), "SumMode": ("FullPrecision",),
                 "ProductMode": ("FullPrecision",)}
    for key, val in opts.items():
        require(key in supported and val in supported[key], "fimath." + key, f"Unsupported {key}={val!r}.")
    return _Fimath(opts.get("RoundingMethod", "Nearest"), opts.get("OverflowAction", "Saturate"))


@dataclass(frozen=True)
class FiData:
    numerictype: numerictype
    codes: tuple

    def storedInteger(self):
        return list(self.codes)

    def double(self):
        return [math.ldexp(float(code), -self.numerictype.FractionLength) for code in self.codes]

    def record(self):
        return dict(self.numerictype.record(), ints=self.storedInteger())


def fi(value, *args):
    """Quantize a scalar or real sample sequence with the release fi core."""
    fm = args[-1] if args and isinstance(args[-1], _Fimath) else None
    typ = args[:-1] if fm is not None else args
    if len(typ) == 1 and isinstance(typ[0], numerictype):
        target = typ[0]
    else:
        require(len(typ) == 3, "fi", "Use fi(value,s,w,f[,F]) or fi(value,T[,F]).")
        target = numerictype(*typ)
    engine = core(root())
    mathopts = fm or fimath()
    fmt = dict(rm=mathopts.RoundingMethod, om=mathopts.OverflowAction)
    if isinstance(value, FiData):
        require(fm is not None, "fi", "Supply an explicit fimath when casting fi data.")
        source = value.numerictype.record()
        cases = (dict(op="cast", a=engine._out(source["s"], source["wl"], source["fl"], n)) for n in value.codes)
    else:
        def literals():
            for v in value if isinstance(value, (list, tuple, range)) else (value,):
                require(type(v) in (int, float) and (type(v) is not int or v.bit_length() <= 1023)
                        and math.isfinite(v), "fi", "Expected finite real double samples.", "BAD_INPUT")
                require(type(v) is not int or int(float(v)) == v, "fi", "Integer is inexact as binary64; use stored codes.", "BAD_INPUT")
                yield dict(op="quantize", x=struct.pack(">d", float(v)).hex())
        cases = literals()
    codes = []
    for case in cases:
        result = engine.eval_case(dict(case, t=target.record(), fm=fmt))
        if "err" in result:
            raise FxError("NATIVE_REJECTION", "fi", result["err"], native_id=result["err"])
        codes.append(engine._decode(result)[3])
    return FiData(target, tuple(codes))

"""Python spellings of the recordings binary-point fi operations."""
from dataclasses import asdict, dataclass
from functools import lru_cache
import os

from . import _fixed
from ._backend import FxError, core, options, require, root

@dataclass(frozen=True)
class numerictype(_fixed.numerictype):
    """Binary-point type for the recordings arithmetic core, including wide intermediates."""

    def __post_init__(self):
        s, w, f = self.SignednessBool, self.WordLength, self.FractionLength
        require(type(s) in (int, bool) and s in (0, 1) and type(w) is int and type(f) is int,
                "numerictype", "Expected signedness, integer word length and fraction length.", "BAD_TYPE")
        require(not (s and w == 1), "numerictype", "A signed fi value needs two or more bits.",
                "fixed:numerictype:invalidMinWordLengthCodegen")
        require(1 <= w <= 128 and -128 <= f <= 128, "numerictype",
                "This Python interface supports 1–128 bits and fraction lengths from -128 to 128.")


@dataclass(frozen=True)
class _Fixdt(numerictype):
    """Simulink binary-point dialog type."""


def fixdt(signed, word_length, fraction_length):
    return _Fixdt(signed, word_length, fraction_length)


@dataclass(frozen=True)
class _Math:
    RoundingMethod: str = "Nearest"
    OverflowAction: str = "Saturate"
    ProductMode: str = "FullPrecision"
    SumMode: str = "FullPrecision"
    ProductWordLength: int = 32
    ProductFractionLength: int = 30
    SumWordLength: int = 32
    SumFractionLength: int = 30
    CastBeforeSum: bool = True

    def record(self):
        return dict(rm=self.RoundingMethod, om=self.OverflowAction, pm=self.ProductMode,
                    sm=self.SumMode, pwl=self.ProductWordLength, pfl=self.ProductFractionLength,
                    swl=self.SumWordLength, sfl=self.SumFractionLength, cbs=int(self.CastBeforeSum))


def fimath(*args, **kwargs):
    base = args[0] if args and isinstance(args[0], _Math) else None
    values = options(args[1:] if base else args, kwargs, "fimath")
    fields = asdict(base or _Math())
    modes = ("FullPrecision", "SpecifyPrecision", "KeepLSB", "KeepMSB")
    for key, value in values.items():
        require(key in fields, "fimath." + key, "Unknown arithmetic property.")
        if key in ("SumMode", "ProductMode"):
            valid = value in modes
        elif key == "OverflowAction":
            valid = value in ("Saturate", "Wrap")
        elif key == "RoundingMethod":
            valid = value in ("Nearest", "Floor", "Ceiling", "Zero", "Round", "Convergent")
        elif key == "CastBeforeSum":
            valid = type(value) in (bool, int) and value in (0, 1)
        else:
            valid = type(value) is int and (1 <= value <= 128 if key.endswith("WordLength") else -128 <= value <= 128)
        require(valid, "fimath." + key, f"Unsupported {key}={value!r}.")
        fields[key] = value
    return _Math(**fields)


@lru_cache(maxsize=4)
def _engine_for(configured):
    return core(root())


def _engine():
    return _engine_for(os.environ.get("FXSIM_ROOT"))


def _math_for(a, b):
    require(a.local is None or b.local is None or a.local == b.local,
            "fimath", "Operands have different local fimath settings.")
    return a.local or b.local


@dataclass(frozen=True, eq=False)
class _Value(_fixed.FiData):
    local: object = None
    imaginary: tuple = None

    @property
    def WordLength(self):
        return self.numerictype.WordLength

    @property
    def FractionLength(self):
        return self.numerictype.FractionLength

    @property
    def Signed(self):
        return bool(self.numerictype.SignednessBool)

    @property
    def int(self):
        return storedInteger(self)

    @property
    def real(self):
        return _Value(self.numerictype, self.codes, self.local)

    @property
    def imag(self):
        return _Value(self.numerictype, self.imaginary or (0,) * len(self), self.local)

    def record(self):
        require(self.imaginary is None, "inputs", "Bind real and imaginary graph components separately.")
        return super().record()

    def storedInteger(self):
        require(self.imaginary is None, "storedInteger", "Use real and imag for complex storage.")
        return list(self.codes)

    def double(self):
        values = super().double()
        if self.imaginary is not None:
            return [complex(a, b) for a, b in zip(values, self.imag.double())]
        return values

    def __len__(self):
        return len(self.codes)

    def __getitem__(self, key):
        def take(seq):
            out = seq[key]
            return out if isinstance(key, slice) else (out,)
        return _Value(self.numerictype, take(self.codes), self.local,
                      None if self.imaginary is None else take(self.imaginary))

    def __add__(self, other):
        return _elementwise("add", self, other)

    def __sub__(self, other):
        return _elementwise("sub", self, other)

    def __mul__(self, other):
        return _elementwise("mul", self, other)

    def __matmul__(self, other):
        return dot(self, other)

    def __neg__(self):
        return _elementwise("neg", self)

    def __abs__(self):
        return _elementwise("abs", self)

    def __bool__(self):
        raise FxError("UNSUPPORTED", "fi truth value", "Compare explicit stored or physical values.")

    def __eq__(self, other):
        raise FxError("UNSUPPORTED", "fi equality", "Compare explicit stored or physical values.")

    def _bits(self, j):
        t = self.numerictype
        return _engine()._out(t.SignednessBool, t.WordLength, t.FractionLength, self.codes[j])


def _convert_results(rows, local):
    engine = _engine()
    codes = []
    typ = None
    for row in rows:
        if "err" in row:
            raise FxError("NATIVE_REJECTION", "fi", row["err"], native_id=row["err"])
        s, w, f, n = engine._decode(row)
        if typ is None:
            typ = numerictype(s, w, f)
        codes.append(n)
    require(typ is not None, "fi arithmetic", "Empty arithmetic inputs are unsupported.")
    return _Value(typ, tuple(codes), local)


def _real_arg(value, where):
    require(isinstance(value, _Value) and value.imaginary is None and len(value) > 0,
            where, "Use nonempty real fi operands.")


def _elementwise(op, a, b=None, **params):
    _real_arg(a, op)
    count = len(a)
    local = a.local
    if b is not None:
        _real_arg(b, op)
        count = max(count, len(b))
        require(len(a) in (1, count) and len(b) in (1, count), op, "Incompatible vector lengths.")
        local = _math_for(a, b)
    engine = _engine()
    fm = (local or fimath()).record()
    def rows():
        for i in range(count):
            case = dict(op=op, fm=fm, a=a._bits(0 if len(a) == 1 else i), **params)
            if b is not None:
                case["b"] = b._bits(0 if len(b) == 1 else i)
            yield engine.eval_case(case)
    return _convert_results(rows(), local)


def dot(a, b):
    _real_arg(a, "dot")
    _real_arg(b, "dot")
    require(len(a) == len(b), "dot", "Use two vectors of equal length.")
    local = _math_for(a, b)
    fm = local or fimath()
    require(fm.SumMode == "FullPrecision" or fm.OverflowAction == "Wrap" or not fm.CastBeforeSum,
            "dot", "Saturating limited-precision accumulation with CastBeforeSum is outside the recorded safe subset.")
    def vec(x):
        return dict(x.numerictype.record(), bins=[x._bits(i)["bin"] for i in range(len(x))])
    ans = _engine().eval_case(dict(op="dot", fm=fm.record(), av=vec(a), bv=vec(b)))
    return _convert_results([ans], local)


def divide(t, a, b):
    require(type(t) is numerictype, "divide", "Specify the result numerictype.")
    return _elementwise("divide", a, b, t=t.record())


def bitsra(x, k):
    _real_arg(x, "bitsra")
    require(type(k) is int and 0 <= k <= x.WordLength, "bitsra", "Expected a nonnegative shift up to the word length.")
    return _elementwise("bitshift", x, k=-k)


def bitsll(x, k):
    _real_arg(x, "bitsll")
    require(type(k) is int and 0 <= k <= x.WordLength, "bitsll", "Expected a nonnegative shift up to the word length.")
    # Logical shifts always discard bits that leave the stored word.
    eng = _engine()
    rows = (eng.eval_case(dict(op="bitshift", a=x._bits(i), k=k, fm={"om":"Wrap"})) for i in range(len(x)))
    return _convert_results(rows, x.local)


def fi(value, *args, **kwargs):
    require(not args or not isinstance(args[0], _Fixdt), "fi", "Use numerictype for fi; fixdt is a Simulink dialog type.")
    count = 1 if args and type(args[0]) is numerictype else 3
    require(len(args) >= count, "fi", "Use fi(x,s,w,f) or fi(x,T), optionally followed by fimath.")
    t = args[0] if count == 1 else numerictype(*args[:count])
    rest = args[count:]
    local = fimath(*rest, **kwargs) if rest or kwargs else None
    quant = local or fimath()
    policy = _fixed.fimath("RoundingMethod", quant.RoundingMethod, "OverflowAction", quant.OverflowAction)
    if isinstance(value, _Value):
        require(local is not None or value.local is None, "fi cast", "Specify the target fimath for a locally configured input.")
        rr = _fixed.fi(value.real, t, policy)
        im = _fixed.fi(value.imag, t, policy).codes if value.imaginary is not None else None
        return _Value(t, rr.codes, local, im)
    seq = value if isinstance(value, (list, tuple, range)) else (value,)
    cmplx = any(isinstance(x, complex) for x in seq)
    if cmplx:
        require(all(type(x) in (int, float, complex) for x in seq), "fi", "Expected a numeric sequence.")
    rr = _fixed.fi([x.real for x in seq] if cmplx else seq, t, policy)
    im = _fixed.fi([x.imag for x in seq], t, policy).codes if cmplx else None
    return _Value(t, rr.codes, local, im)


def real(x):
    return x.real


def imag(x):
    return x.imag


def storedInteger(x):
    data = x.storedInteger()
    return data[0] if len(data) == 1 else data


def double(x):
    if isinstance(x, _Value):
        data = x.double()
        return data[0] if len(data) == 1 else data
    if isinstance(x, (list, tuple, range)):
        return [float(v) for v in x]
    return float(x)

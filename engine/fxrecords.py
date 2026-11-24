"""Compiled block records beyond the lookup family (lookup records are in
engine/fxlutplan.py). The compiler builds one named, validated, immutable record per block from its parameters and
types; consumers read named fields instead of re-deriving values from the parameters or unpacking positional tuples.
A record that breaks its own invariants is a compiler defect (RecordError), never a user error.

Families migrated so far:
  CounterRecord  Counter Limited / Counter Free-Running: the limit the count wraps after, and the output type.
  DetectRecord   Detect Change / Increase / Decrease: the relation, the initial value (as given and quantised to the
                 input type), and the input / output types.
  ProductRecord  Product: the operator string (one "*" or "/" per input), the operand and output types, the rounding
                 of the result and of a quotient, and the overflow handling.
  FirRecord      Discrete FIR: the coefficient, product, accumulator, state, output and tap-sum types, and the
                 coefficients as stored integers of the coefficient type.
  HoldRecord     Hold (multirate lowering): the initial value as a stored integer of the signal type (or boolean).
  GainRecord     Gain: the gain as given, the parameter type and its stored integer, the input and output types.
  SumRecord      Sum: the signs, the input types, the accumulator type and where it came from (explicit, "Same as
                 first input", internal rule), the output type and the first added input.
  DtiRecord      Discrete-Time Integrator (accepted configurations: Forward Euler, internal-rule output): the step
                 constant K*T, the initial condition and limits (as given, and as stored integers of the state type)."""
from dataclasses import dataclass
from fractions import Fraction


class RecordError(Exception):
    """A compile record that does not satisfy its invariants."""


def _need(cond, what):
    if not cond:
        raise RecordError("compile record: " + what)

# Parameter records: what a block's parameters alone fix (no types needed). Built when the design is constructed, so
# the paths that never compile a design -- the optimiser's double reference and range analysis run on the
# uncompiled design -- read the same values as the compiled records, which are built from them.


@dataclass(frozen=True)
class CounterParams:
    limit: int          # the count goes 0 .. limit, 0 ...

    def __post_init__(self):
        _need(isinstance(self.limit, int) and self.limit >= 0, "counter limit %r" % (self.limit,))


@dataclass(frozen=True)
class DetectParams:
    relop: str
    vinit: float

    def __post_init__(self):
        _need(self.relop in ("~=", ">", "<"), "detect relation %r" % (self.relop,))


@dataclass(frozen=True)
class FirParams:
    coefs: tuple        # the coefficients as given (floats)
    init: tuple         # unquantised initial states, most recent first (ntaps - 1 of them)
    structure: str      # FilterStructure

    def __post_init__(self):
        _need(len(self.coefs) >= 1, "FIR without coefficients")
        _need(len(self.init) == max(len(self.coefs) - 1, 0) or len(self.coefs) == 1,
              "FIR initial states %d for %d taps" % (len(self.init), len(self.coefs)))


@dataclass(frozen=True)
class HoldParams:
    ic: float           # the held value before the first enable (multirate lowering, engine/fxlower.py)


@dataclass(frozen=True)
class DtiParams:
    gain: Fraction      # gainval * sample period
    ic: float           # initial condition, clamped to the limits when limiting is on
    limits: object      # None or (lower, upper) floats

    def __post_init__(self):
        _need(isinstance(self.gain, Fraction), "integrator gain %r" % (self.gain,))




@dataclass(frozen=True)
class CounterRecord:
    block: str
    kind: str          # "CounterLimited" | "CounterFreeRunning"
    limit: int         # the count goes 0, 1, ..., limit, 0, ...
    out_type: tuple    # (s, w, f): unsigned integer, f = 0

    def __post_init__(self):
        _need(self.kind in ("CounterLimited", "CounterFreeRunning"), "counter kind %r" % (self.kind,))
        t = self.out_type
        _need(isinstance(t, tuple) and len(t) == 3 and t[0] == 0 and t[2] == 0 and t[1] >= 1,
              "counter %s output type %r must be an unsigned integer" % (self.block, t))
        _need(isinstance(self.limit, int) and 0 <= self.limit <= (1 << t[1]) - 1,
              "counter %s limit %r outside its output type %r" % (self.block, self.limit, t))
        if self.kind == "CounterFreeRunning":
            _need(self.limit == (1 << t[1]) - 1, "free-running counter %s must wrap at 2^%d - 1" % (self.block, t[1]))


DETECT_OPS = {"DetectChange": "~=", "DetectIncrease": ">", "DetectDecrease": "<"}


@dataclass(frozen=True)
class DetectRecord:
    block: str
    kind: str          # DetectChange | DetectIncrease | DetectDecrease
    relop: str         # "~=" | ">" | "<": u[k] relop u[k-1]
    vinit: float       # initial condition as given (the double reference uses it unquantised)
    vinit_stored: int  # the initial condition quantised to the input type (nearest, saturate)
    in_type: tuple
    out_type: object   # boolean or an (s, w, f) tuple

    def __post_init__(self):
        _need(self.kind in DETECT_OPS and self.relop == DETECT_OPS[self.kind], "detect %s relation %r for %s"
              % (self.block, self.relop, self.kind))
        t = self.in_type
        _need(isinstance(t, tuple) and len(t) == 3, "detect %s input type %r must be fixed point" % (self.block, t))
        lo, hi = (-(1 << (t[1] - 1)), (1 << (t[1] - 1)) - 1) if t[0] else (0, (1 << t[1]) - 1)
        _need(isinstance(self.vinit_stored, int) and lo <= self.vinit_stored <= hi,
              "detect %s initial value %r outside its input type %r" % (self.block, self.vinit_stored, t))


@dataclass(frozen=True)
class DtiRecord:
    block: str
    gain: object        # Fraction: the constant multiplying the input each step, gainval * sample period
    ic: float           # initial condition as given, clamped to the limits when limiting is on (double reference)
    limits: object      # None, or (lower, upper) as given (floats; -inf / inf allowed)
    x0_stored: int      # initial state, stored integer of the state type
    limits_stored: object   # None, or (lower, upper) stored integers of the state type
    in_type: tuple
    out_type: tuple

    def __post_init__(self):
        from fractions import Fraction
        _need(isinstance(self.gain, Fraction) and self.gain > 0, "integrator %s gain %r" % (self.block, self.gain))
        t = self.out_type
        _need(isinstance(t, tuple) and len(t) == 3, "integrator %s output type %r" % (self.block, t))
        lo, hi = (-(1 << (t[1] - 1)), (1 << (t[1] - 1)) - 1) if t[0] else (0, (1 << t[1]) - 1)
        _need(lo <= self.x0_stored <= hi, "integrator %s initial state outside %r" % (self.block, t))
        if self.limits_stored is not None:
            a, b = self.limits_stored
            _need(lo <= a <= hi and lo <= b <= hi, "integrator %s limits outside %r" % (self.block, t))
            _need(self.limits is not None, "integrator %s stored limits without limits" % self.block)


@dataclass(frozen=True)
class SumRecord:
    # the type parameters an optimiser may set, each with the record field holding its compiled type
    TYPE_ROLES = (("AccumDataTypeStr", "acc_type"), ("OutDataTypeStr", "out_type"))
    block: str
    signs: tuple          # "+" / "-" per input
    in_types: tuple       # (s, w, f) per input
    acc_type: tuple       # the accumulator every input is cast into and every partial sum is kept in
    acc_source: str       # "explicit" | "same_first" | "internal_rule"
    out_type: tuple
    first: object         # index of the first "+" input (the accumulator starts from it), or None (all minus)

    def __post_init__(self):
        _need(len(self.signs) == len(self.in_types) and all(sg in "+-" for sg in self.signs),
              "sum %s signs %r for %d inputs" % (self.block, self.signs, len(self.in_types)))
        _need(self.acc_source in ("explicit", "same_first", "internal_rule"), "sum %s accumulator source %r"
              % (self.block, self.acc_source))
        a = self.acc_type
        _need(isinstance(a, tuple) and len(a) == 3 and a[1] >= 1, "sum %s accumulator type %r" % (self.block, a))
        _need(self.first == (self.signs.index("+") if "+" in self.signs else None),
              "sum %s first added input %r" % (self.block, self.first))


@dataclass(frozen=True)
class GainRecord:
    # the type parameters an optimiser may set, each with the record field holding its compiled type
    TYPE_ROLES = (("ParamDataTypeStr", "param_type"), ("OutDataTypeStr", "out_type"))
    block: str
    gain: float           # the gain as given (the double reference)
    param_type: tuple     # (s, w, f) the gain is quantised to (explicit, internal rule, or named for boolean inputs)
    param_stored: int     # the quantised gain, a stored integer of param_type
    in_type: object       # (s, w, f) or boolean
    out_type: object

    def __post_init__(self):
        t = self.param_type
        _need(isinstance(t, tuple) and len(t) == 3 and t[1] >= 1, "gain %s parameter type %r" % (self.block, t))
        lo, hi = (-(1 << (t[1] - 1)), (1 << (t[1] - 1)) - 1) if t[0] else (0, (1 << t[1]) - 1)
        _need(isinstance(self.param_stored, int) and lo <= self.param_stored <= hi,
              "gain %s stored parameter %r outside %r" % (self.block, self.param_stored, t))


@dataclass(frozen=True)
class ProductRecord:
    # the type parameters an optimiser may set, each with the record field holding its compiled type
    TYPE_ROLES = (("OutDataTypeStr", "out_type"),)
    block: str
    ops: str              # one "*" / "/" per input, in port order
    in_types: tuple       # (s, w, f) per input (booleans as the converted type the route uses)
    out_type: tuple
    rounding: str         # the block's rounding of products
    rounding_quotient: str   # the rounding of a division ("Simplest" divides toward zero)
    saturate: bool

    def __post_init__(self):
        _need(len(self.ops) == len(self.in_types) and self.ops and not set(self.ops) - set("*/"),
              "product %s operators %r for %d inputs" % (self.block, self.ops, len(self.in_types)))
        _need(isinstance(self.out_type, tuple) and len(self.out_type) == 3, "product %s output type %r"
              % (self.block, self.out_type))


@dataclass(frozen=True)
class FirRecord:
    # the type parameters an optimiser may set, each with the record field holding its compiled type
    TYPE_ROLES = (("CoefDataTypeStr", "coef_type"), ("ProductDataTypeStr", "product_type"),
                  ("AccumDataTypeStr", "acc_type"), ("OutDataTypeStr", "out_type"))
    block: str
    in_type: tuple
    coef_type: tuple
    product_type: tuple
    acc_type: tuple
    state_type: tuple
    out_type: tuple
    tapsum_type: tuple
    coefs_stored: tuple   # coefficients quantised to coef_type

    def __post_init__(self):
        for nm in ("in_type", "coef_type", "product_type", "acc_type", "state_type", "out_type", "tapsum_type"):
            t = getattr(self, nm)
            _need(isinstance(t, tuple) and len(t) == 3, "FIR %s %s %r" % (self.block, nm, t))
        c = self.coef_type
        lo, hi = (-(1 << (c[1] - 1)), (1 << (c[1] - 1)) - 1) if c[0] else (0, (1 << c[1]) - 1)
        _need(all(lo <= q <= hi for q in self.coefs_stored), "FIR %s stored coefficient outside %r" % (self.block, c))


@dataclass(frozen=True)
class HoldRecord:
    block: str
    ic_stored: int
    out_type: object      # (s, w, f) or boolean

    def __post_init__(self):
        _need(isinstance(self.ic_stored, int), "hold %s initial value %r" % (self.block, self.ic_stored))


# block type -> its compiled record class: the optimiser's decision variables of a block are its record's TYPE_ROLES
# See block_keys in engine/fxgopt_lib.py.
RECORD_OF = {"Sum": SumRecord, "Gain": GainRecord, "Product": ProductRecord, "DiscreteFir": FirRecord}


def compiled_role_types(rec):
    """{type parameter: compiled (s, w, f)} of a compiled record, in TYPE_ROLES order."""
    return {k: getattr(rec, f) for k, f in type(rec).TYPE_ROLES}

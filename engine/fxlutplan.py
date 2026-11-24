"""Compiled lookup blocks: one named, validated, immutable record per 1-D / 2-D Lookup Table.

The compiler (engine/fxgraph_lib.py) builds the record once from the block's parameters and its input types; every
consumer -- the C core protocol (fxgraph_lib._fmt_ins), the generated C (fxcodegen), the sized RTL (fxrtl), the
128-bit Verilog (fxverilog), the overflow prover (fxprove) and the optimiser's cost counter (fxgopt_lib.inner_wl) --
reads named fields instead of unpacking a positional tuple. The record states, explicitly:
  types       every type the block works in, with where it came from (explicit parameter, "Same as ...", inherited
              rule of recordings) and the type as declared (for the cost counter);
  quantisation points  the fraction, the signed slope term, the interpolated value and the output conversion, each with
              its type, rounding and overflow handling (term_type is the definition the back ends read; the C core and ref/fxcore.py qs() derive the
              same type themselves and are cross-checked against it -- recordings);
  integers    breakpoints, table values (and the table at the table's precision for the steps) as stored integers.
Construction validates the record (types, lengths, ordering, value ranges), so a consumer never sees a half-built or
inconsistent one; the records are frozen (fields are tuples)."""
from dataclasses import dataclass, field

RM_NAMES = ("Floor", "Ceiling", "Zero", "Nearest", "Round", "Convergent")
MAX_WL = 4096   # = fxgraph_lib.MAX_WL (checked by the regression check)


class PlanError(Exception):
    """A record that does not satisfy its own invariants (a compiler defect, never a user error)."""


def _need(cond, what):
    if not cond:
        raise PlanError("lookup compile record: " + what)


def _type(t, what):
    _need(isinstance(t, tuple) and len(t) == 3 and all(isinstance(v, int) and not isinstance(v, bool) for v in t),
          "%s must be an (s, w, f) tuple of ints, got %r" % (what, t))
    # the front end's word-length contract (fxgraph_lib.MAX_WL; recordings: a 257-bit input the Python path
    # accepts became INTERNAL here)
    _need(t[0] in (0, 1) and 1 <= t[1] <= MAX_WL, "%s %r: signedness 0/1 and word 1..%d" % (what, t, MAX_WL))
    return t


def _table_type(t):
    """A table type is fixed point or "double" (a double table keeps exact steps; the C core then is not used)."""
    return t if t == "double" else _type(t, "table type")


def _word(t, what):
    _need(isinstance(t, tuple), "%s %r has no word length for the cost counter" % (what, t))
    return t[1]


def trange(t):
    s, w = t[0], t[1]
    return (-(1 << (w - 1)), (1 << (w - 1)) - 1) if s else (0, (1 << w) - 1)


def _ints(v, what):
    v = tuple(v)
    _need(all(isinstance(x, int) and not isinstance(x, bool) for x in v), "%s must be integers" % what)
    return v


def _increasing(v, what):
    _need(len(v) >= 2 and all(b > a for a, b in zip(v, v[1:])), "%s must be at least 2 and strictly increasing" % what)


def _within(v, t, what):
    lo, hi = trange(t)
    _need(all(lo <= x <= hi for x in v), "%s outside %r" % (what, t))


def _source(s, what):
    _need(s in ("explicit", "same as output", "same as input", "inherited rule", "internal rule"),
          "%s source %r" % (what, s))
    return s


def _check_cast(tok, frm, to, rounding, saturate, what):
    """The C-core cast token 'sh rm s wl sat' must be the one the named types imply (an 8-bit cast with
    a 16-bit out_type was accepted and the output wrapped)."""
    want = (frm[2] - to[2], RM_NAMES.index(rounding), to[0], to[1], 1 if saturate else 0)
    try:
        got = tuple(int(v) for v in str(tok).split())
    except ValueError:
        got = None
    _need(got == want, "%s cast %r does not match %r -> %r (%s, %s): expected %r"
          % (what, tok, frm, to, rounding, "saturate" if saturate else "wrap", " ".join(map(str, want))))


def _check_align(sa, sb, in_t, bp_t, what):
    """Input and breakpoints are aligned at the finer of their two fractions."""
    F = max(in_t[2], bp_t[2])
    _need((sa, sb) == (F - in_t[2], F - bp_t[2]), "%s alignment shifts (%d, %d) do not match input %r and breakpoints "
          "%r: expected (%d, %d)" % (what, sa, sb, in_t, bp_t, F - in_t[2], F - bp_t[2]))


def _check_dsh(dsh, table_type, it, what):
    tf = table_type[2] if isinstance(table_type, tuple) else it[2]
    _need(dsh == max(tf, it[2]) - it[2], "%s step shift %d does not match table %r and intermediate %r"
          % (what, dsh, table_type, it))


@dataclass(frozen=True)
class LutIndex:
    """1-D Flat / Nearest table (C core LUT): u << sa clipped to the breakpoints << sb, segment k, k or k + 1 by the
    method; the output value is the table value already cast to the output type."""
    block: str
    method: str                 # "Flat" | "Nearest"
    in_type: tuple
    bp_type: tuple
    table_type: tuple
    out_type: tuple
    sa: int                     # input and breakpoints aligned at a common fraction: shifts
    sb: int
    bps: tuple                  # breakpoints, stored integers of bp_type
    outs: tuple                 # table values as output-type integers
    # the table values rounded into the output type BEFORE any wrap / saturation (min, max); the prover
    # checks this conversion as a quantisation point (outs alone hides a saturated table value)
    table_conv: tuple = None
    op: str = field(default="LUT", init=False)

    def __post_init__(self):
        _need(self.method in ("Flat", "Nearest"), "method %r" % self.method)
        for k in ("in_type", "bp_type", "out_type"):
            _type(getattr(self, k), k)
        _table_type(self.table_type)
        object.__setattr__(self, "bps", _ints(self.bps, "breakpoints"))
        object.__setattr__(self, "outs", _ints(self.outs, "outputs"))
        _increasing(self.bps, "breakpoints")
        _need(len(self.outs) == len(self.bps), "one output value per breakpoint")
        _need(self.sa >= 0 and self.sb >= 0, "alignment shifts >= 0")
        _check_align(self.sa, self.sb, self.in_type, self.bp_type, "index table")
        _within(self.bps, self.bp_type, "breakpoints")
        _within(self.outs, self.out_type, "output values")

    @property
    def mode(self):
        return 0 if self.method == "Flat" else 1

    def stored_bits(self):
        """Storage cost: table entries x table word + breakpoints x breakpoint word."""
        return len(self.bps) * (_word(self.table_type, "table type") + self.bp_type[1])


def _named_word(declared, source, compiled, out_type):
    """The word a lookup's intermediate type parameter names (hardware cost by the
    user's design, one design written two ways costs the same): the declared type; 'Same as output' names the
    output type ('Same as output' and the output type written out cost 134 and 146); the internal rule
    its compiled result. Not the word the engine computes in (12 bits declared run in a 16-bit word here)."""
    if declared is not None:
        return declared[1]
    if source == "same as output" and out_type:
        return out_type[1]
    return compiled[1]


@dataclass(frozen=True)
class LutLinear:
    """1-D Linear point-slope table (C core LUTL):
      fraction  f = sat_frac(round((u - b_k) << ff / (b_k+1 - b_k)))           (frac_type, always saturating)
      term      p = ovf(round(f * slope_k / 2^(ff + dsh)))                       (term_type, the block's overflow mode)
      value     y = ovf(t_k + p)                                                 (intermediate type)
      output    cast(y) with out_cast; with use_last on, u at the last breakpoint gives the last table value."""
    block: str
    in_type: tuple
    bp_type: tuple
    table_type: tuple
    intermediate: tuple
    intermediate_source: str
    intermediate_declared: object   # (s, w, f) as written, or None (not an explicit type)
    frac_type: tuple
    frac_source: str
    frac_declared: object
    out_type: tuple
    out_cast: str                   # cast token 'sh rm s wl sat' intermediate -> output (C core format)
    rounding: str
    saturate: bool
    use_last: bool
    sa: int
    sb: int
    bps: tuple
    tab: tuple                      # table values as intermediate-type integers
    slopes: tuple                   # per segment, the step at fraction max(table, intermediate); last entry 0
    dsh: int                        # that fraction minus the intermediate fraction
    table_conv: tuple = None        # table -> intermediate rounding before wrap / saturation (min, max)
    # a falling segment's term is formed from the magnitude, y0 - round(f * (y0 - y1)) (host, 32-bit
    # table type); the generators do not implement it and refuse such a plan when a slope is negative
    magnitude_slope: bool = False
    # with an inherited fraction type a fraction that rounds to 1 is 1 (not saturated below it)
    frac_carry: bool = False
    frac_round1_stop: bool = False   # a fraction rounding to 1 stops the run (unrecorded)
    op: str = field(default="LUTL", init=False)

    def __post_init__(self):
        for k in ("in_type", "bp_type", "intermediate", "frac_type", "out_type"):
            _type(getattr(self, k), k)
        _table_type(self.table_type)
        for k in ("intermediate_declared", "frac_declared"):
            if getattr(self, k) is not None:
                _type(getattr(self, k), k)
        _source(self.intermediate_source, "intermediate")
        _source(self.frac_source, "fraction")
        _need(self.rounding in RM_NAMES, "rounding %r" % self.rounding)
        for k in ("bps", "tab", "slopes"):
            object.__setattr__(self, k, _ints(getattr(self, k), k))
        _increasing(self.bps, "breakpoints")
        _need(len(self.tab) == len(self.bps) == len(self.slopes) and self.slopes[-1] == 0,
              "one table value and one slope per breakpoint (last slope 0)")
        _need(self.sa >= 0 and self.sb >= 0 and self.dsh >= 0, "shifts >= 0")
        _within(self.bps, self.bp_type, "breakpoints")
        _within(self.tab, self.intermediate, "table values")
        _need(self.frac_type[0] == 0, "the interpolation fraction type is unsigned")
        _check_cast(self.out_cast, self.intermediate, self.out_type, self.rounding, self.saturate, "output")
        _check_align(self.sa, self.sb, self.in_type, self.bp_type, "1-D linear")
        _check_dsh(self.dsh, self.table_type, self.intermediate, "1-D linear")

    @property
    def rm(self):
        return RM_NAMES.index(self.rounding)

    @property
    def term_type(self):
        """Signed slope term: the intermediate word, one bit wider when the intermediate type is unsigned."""
        it = self.intermediate
        return (1, it[1] + (0 if it[0] else 1), it[2])

    def stored_bits(self):
        """Cost counter: entries x (table word + breakpoint word) + the intermediate and fraction words as compiled
        -- declared or inherited (one design written two ways costs the same; recordings had 134 with the library default 'Same as output' against 146 with the same type written out)."""
        return len(self.bps) * (_word(self.table_type, "table type") + self.bp_type[1]) + \
            _named_word(self.intermediate_declared, self.intermediate_source, self.intermediate, self.out_type) + \
            (self.frac_declared or self.frac_type)[1]


@dataclass(frozen=True)
class Lut2D:
    """2-D table (C core LUT2). Index methods pick per dimension; Linear point-slope interpolates along dimension 1
    at the two neighbouring columns (step from the table at its own precision, tab_steps), then along dimension 2,
    every stage in the intermediate type; fraction per dimension as the 1-D table."""
    block: str
    method: str
    in_types: tuple                 # ((s, w, f), (s, w, f))
    bp_types: tuple
    table_type: tuple
    out_type: tuple
    shifts: tuple                   # (sa1, sb1, sa2, sb2)
    bps: tuple                      # (dimension 1 integers, dimension 2 integers)
    tab: tuple                      # row-major; output-type integers (index) or intermediate-type integers (linear)
    intermediate: object = None     # linear only
    intermediate_source: object = None
    intermediate_declared: object = None
    frac_type: object = None
    frac_source: object = None
    frac_declared: object = None
    out_cast: object = None
    rounding: object = None
    saturate: bool = False
    use_last: bool = False
    tab_steps: tuple = ()           # linear: row-major table at fraction max(table, intermediate)
    dsh: int = 0
    table_conv: tuple = None        # table -> output (index) / intermediate (linear) before wrap / saturation
    op: str = field(default="LUT2", init=False)

    def __post_init__(self):
        _need(self.method in ("Flat", "Nearest", "Linear point-slope"), "method %r" % self.method)
        _need(len(self.in_types) == 2 and len(self.bp_types) == 2 and len(self.bps) == 2 and len(self.shifts) == 4,
              "two dimensions")
        object.__setattr__(self, "in_types", tuple(_type(t, "input type") for t in self.in_types))
        object.__setattr__(self, "bp_types", tuple(_type(t, "breakpoint type") for t in self.bp_types))
        _table_type(self.table_type)
        _type(self.out_type, "output type")
        object.__setattr__(self, "shifts", _ints(self.shifts, "shifts"))
        _need(all(v >= 0 for v in self.shifts), "shifts >= 0")
        object.__setattr__(self, "bps", tuple(_ints(b, "breakpoints") for b in self.bps))
        for b, t in zip(self.bps, self.bp_types):
            _increasing(b, "breakpoints")
            _within(b, t, "breakpoints")
        for d_ in (0, 1):
            _check_align(self.shifts[2 * d_], self.shifts[2 * d_ + 1], self.in_types[d_], self.bp_types[d_],
                         "2-D dimension %d" % (d_ + 1))
        object.__setattr__(self, "tab", _ints(self.tab, "table"))
        _need(len(self.tab) == len(self.bps[0]) * len(self.bps[1]), "table size = n1 x n2")
        if self.linear:
            _type(self.intermediate, "intermediate type")
            _type(self.frac_type, "fraction type")
            for k in ("intermediate_declared", "frac_declared"):
                if getattr(self, k) is not None:
                    _type(getattr(self, k), k)
            _source(self.intermediate_source, "intermediate")
            _source(self.frac_source, "fraction")
            _need(self.rounding in RM_NAMES, "rounding %r" % self.rounding)
            _need(self.frac_type[0] == 0, "the interpolation fraction type is unsigned")
            object.__setattr__(self, "tab_steps", _ints(self.tab_steps, "table steps"))
            _need(len(self.tab_steps) == len(self.tab) and self.dsh >= 0, "table at its precision: same size")
            _within(self.tab, self.intermediate, "table values")
            _check_cast(self.out_cast, self.intermediate, self.out_type, self.rounding, self.saturate, "output")
            _check_dsh(self.dsh, self.table_type, self.intermediate, "2-D linear")
        else:
            _within(self.tab, self.out_type, "table values")
            _need(self.tab_steps == () and self.intermediate is None and self.frac_type is None,
                  "index methods have no interpolation types")

    @property
    def linear(self):
        return self.method == "Linear point-slope"

    @property
    def mode(self):
        return {"Flat": 0, "Nearest": 1, "Linear point-slope": 2}[self.method]

    @property
    def rm(self):
        return RM_NAMES.index(self.rounding) if self.linear else 0

    @property
    def term_type(self):
        """Interpolation terms of the 2-D table stay in the intermediate type."""
        return self.intermediate

    def core_fields(self):
        """The C core's linear header (ff fs fw rm sat its itw ulast itf dsh) and output cast; index methods send
        the neutral values the core ignores."""
        if not self.linear:
            return (0, 0, 1, 0, 0, 0, 1, 1, 0, 0), "0 0 1 64 0", [0] * len(self.tab)
        ft, it = self.frac_type, self.intermediate
        return (ft[2], ft[0], ft[1], self.rm, 1 if self.saturate else 0, it[0], it[1], 1 if self.use_last else 0,
                it[2], self.dsh), self.out_cast, list(self.tab_steps)

    def stored_bits(self):
        n1, n2 = len(self.bps[0]), len(self.bps[1])
        c = n1 * n2 * _word(self.table_type, "table type") + n1 * self.bp_types[0][1] + n2 * self.bp_types[1][1]
        if self.linear:   # (the type each rule names -- recordings)
            c += _named_word(self.intermediate_declared, self.intermediate_source, self.intermediate, self.out_type) + \
                (self.frac_declared or self.frac_type)[1]
        return c

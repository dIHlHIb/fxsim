"""MATLAB Function v1 subset: semantics, from recordings (recorded rules).

compile(text, in_types, chart) -> Program: a static type pass over the syntax tree of engine/fxmlfn.py (types are
fixed at compile time, as Simulink fixes them: a variable keeps the type of its first assignment, branches and loops
must agree), then Program.step(inputs) evaluates one sample with exact integers. Types:
  ("fi", s, wl, fl)   fixed point          ("int", cls)   int8 ... uint32       ("bool",)
Every rule below is one recordings show; anything else is refused with UNSUPPORTED_MATLAB (code, line) at compile:
  default fimath Nearest (half up) / Saturate; EmlDefaultFimath 'Other:UserSpecified' with InputFimath Floor / Wrap
  replaces it for fi(), y(:) = and fimath(a); products and sums are full precision; fi +/- double constant: the
  constant in the fi's own type; fi * double constant: the constant with the fi's word length, best precision;
  integers saturate (SaturateOnIntegerOverflow on) or wrap; integer op double constant rounds half away from zero;
  int + fi: the int as sfixN_En0; TreatAsFi 'Fixed-point & Integer' makes integer inputs fi; int16(fi) etc. round
  half away from zero and saturate; bitshift left saturates or wraps by the fimath, right is arithmetic; bitsll wraps;
  bitsrl logical; bitsra arithmetic; bitsliceget / bitconcat / bitget unsigned; bitset keeps the type; bitand / bitor /
  bitxor need equal types; min / max exact (the result type taken only where the first operand's type is also the
  union type); abs and unary minus keep the type and saturate or wrap by the fimath; comparisons are exact."""
import math
from fractions import Fraction

import fxmlfn as P

INT_CLASSES = {"int8": (1, 8), "int16": (1, 16), "int32": (1, 32), "uint8": (0, 8), "uint16": (0, 16),
               "uint32": (0, 32)}
NEAREST_SAT = ("Nearest", "Saturate")
HDLFIMATH = ("Floor", "Wrap")     # hdlfimath: Floor / Wrap, full-precision products / sums


class Refuse(P.MlError):
    pass


def _refuse(line, msg, code="UNSUPPORTED_MATLAB"):
    raise P.MlError(code, line or 0, 1, msg)


# ---------------------------------------------------------------- exact helpers
def _round(v, rm):
    """v: Fraction -> integer by rounding mode."""
    fl = v.numerator // v.denominator
    r = v - fl
    if r == 0:
        return fl
    if rm == "Floor":
        return fl
    if rm == "Ceiling":
        return fl + 1
    if rm == "Zero":
        return fl if v > 0 else fl + 1
    half = Fraction(1, 2)
    if r > half:
        return fl + 1
    if r < half:
        return fl
    if rm == "Nearest":
        return fl + 1                       # half toward +inf
    if rm == "Round":
        return fl + 1 if v > 0 else fl      # half away from zero
    if rm == "Convergent":
        return fl if fl % 2 == 0 else fl + 1
    raise ValueError(rm)


def _rng(s, wl):
    return (-(1 << (wl - 1)), (1 << (wl - 1)) - 1) if s else (0, (1 << wl) - 1)


def _ovf(n, s, wl, om):
    lo, hi = _rng(s, wl)
    if lo <= n <= hi:
        return n
    if om == "Saturate":
        return max(lo, min(hi, n))
    n %= 1 << wl
    return n - (1 << wl) if s and n >> (wl - 1) else n


def quant(value, t, fm):
    """Exact real value (Fraction) -> stored integer of fi type t under fimath fm (rounding, overflow)."""
    _, s, wl, fl = t
    return _ovf(_round(value * (Fraction(2) ** fl), fm[0]), s, wl, fm[1])


def real(t, n):
    """Stored value n of type t -> exact real value."""
    if t[0] == "fi":
        return Fraction(n) / (Fraction(2) ** t[3])
    return Fraction(n)


def fi_of_int(t):
    s, wl = INT_CLASSES[t[1]]
    return ("fi", s, wl, 0)


def tstr(t):
    if t[0] == "vec":
        return "%s[%d]" % (tstr(t[2]), t[1])
    if t[0] == "bool":
        return "boolean"
    if t[0] == "int":
        return t[1]
    if t[0] != "fi":
        return {"dbl": "double", "fm": "fimath", "nt": "numerictype", "str": "text"}.get(t[0], t[0])
    _, s, wl, fl = t
    if fl == 0 and wl in (8, 16, 32):
        return ("int%d" if s else "uint%d") % wl
    return "%sfix%d%s" % ("s" if s else "u", wl, "" if fl == 0 else ("_En%d" % fl if fl > 0 else "_E%d" % -fl))


def _best_fl(c, s, wl):
    """Best-precision fraction length of constant c in a word of wl bits: the largest with which c fits, one less
    when c rounded to nearest there would overflow the word (recorded: 0.99999 in 16 bits is 1.0 at
    2^-14, not 32767 at 2^-15)."""
    fl = _best_fl_fit(c, s, wl)
    lo, hi = _rng(s, wl)
    if not lo <= _round(Fraction(c) * (Fraction(2) ** fl), "Nearest") <= hi:
        fl -= 1
    return fl


def _best_fl_fit(c, s, wl):
    """Largest fraction length with which constant c fits a word of wl bits (sign included when s)."""
    c = Fraction(c)
    if c == 0:
        return wl - s
    a = abs(c)
    # integer bits needed (excluding sign): smallest k with a < 2^k (for positive) / a <= 2^k (for negative)
    k = 0
    while (a >= Fraction(2) ** k) if c > 0 else (a > Fraction(2) ** k):
        k += 1
    while k > -4096 and ((a < Fraction(2) ** (k - 1)) if c > 0 else (a <= Fraction(2) ** (k - 1))):
        k -= 1
    return wl - s - k


# ---------------------------------------------------------------- compile: types
def _has_persistent(node):
    """Whether a function's AST declares a persistent variable anywhere."""
    if isinstance(node, dict):
        if node.get("op") == "persistent" or node.get("kind") == "persistent" or node.get("type") == "persistent":
            return True
        return any(_has_persistent(v) for v in node.values())
    if isinstance(node, (list, tuple)):
        return any(_has_persistent(v) for v in node)
    return False


class Program:
    def __init__(self, ast, in_types, chart, out_types):
        self.ast = ast
        self.chart = chart
        self.funcs = {f["name"]: f for f in ast["functions"]}
        # typing rewrites nodes in place (folded x.WordLength, a ^ 2, hdlfimath
        # arguments), so a second argument-type copy of a local function is taken from its untyped form
        import copy as _copy
        self._pristine = {n: _copy.deepcopy(f) for n, f in self.funcs.items()}
        # a local function is typed once per argument type signature
        # (Simulink specialises it: g(a) + g(c) = g(c) + g(a)); {name: {signature: the typed copy's name}}
        self._spec = {}
        self.main = self.funcs[ast["main"]]
        if len(in_types) != len(self.main["inputs"]):
            _refuse(self.main["line"], "%d inputs given, the function takes %d" % (len(in_types),
                                                                                   len(self.main["inputs"])), "BAD_MATLAB")
        user = chart.get("EmlDefaultFimath", "Same as MATLAB Default")
        if user not in ("Same as MATLAB Default", "Other:UserSpecified"):
            _refuse(0, "EmlDefaultFimath %r" % user)
        self.default_fm = NEAREST_SAT if user == "Same as MATLAB Default" else _parse_fimath_text(
            chart.get("InputFimath", ""))
        self.int_sat = chart.get("SaturateOnIntegerOverflow", True) in (True, "on", 1)
        treat = chart.get("TreatAsFi", "Fixed-point")
        if treat not in ("Fixed-point", "Fixed-point & Integer"):
            _refuse(0, "TreatAsFi %r" % treat)
        self.int_as_fi = treat == "Fixed-point & Integer"
        def as_fi(t):
            if t[0] == "vec":
                return ("vec", t[1], as_fi(t[2]))
            return fi_of_int(t) if (self.int_as_fi and t[0] == "int") else t
        self.in_types = [as_fi(t) for t in in_types]

        def base(t):
            return base(t[2]) if t[0] == "vec" else t
        # the main function's inputs that are integer classes, made fi by 'Fixed-point & Integer' --
        # fi() of a value built from them keeps the integer rule (Nearest / Saturate), not the block fimath
        self.int_inputs = {nm for nm, t in zip(self.main["inputs"], in_types) if self.int_as_fi and base(t)[0] == "int"}
        # unsigned - unsigned is recorded only under the default fimath with no fimath anywhere in
        # the function (no fimath / hdlfimath / setfimath call, no string argument such as 'OverflowAction')
        def _fm_free(x):
            if isinstance(x, dict):
                if x.get("op") == "str" or (x.get("op") == "call" and x.get("name") in ("fimath", "hdlfimath",
                                                                                         "setfimath")):
                    return False
                return all(_fm_free(v) for v in x.values())
            if isinstance(x, list):
                return all(_fm_free(v) for v in x)
            return True
        self.usub_sat = user == "Same as MATLAB Default" and _fm_free(ast)
        self.persist = {}
        self.types = {}          # (function, variable) -> type
        self.persistent = set()
        # (function, variable) -> the fimath a fi variable carries when it is not the default one (
        # it travels through + and *; two different ones are refused, as by Simulink)
        self.fa = {}
        # (function, variable) -> the value of a double variable that holds a compile-time constant (assigned once,
        # at the top level of its function: k = a.WordLength - 4, recordings)
        self.dconst = {}
        self.fmvar = {}     # (function, variable) -> the fimath a variable F = hdlfimath holds
        env = dict(zip(self.main["inputs"], self.in_types))
        env = self._block(self.main, self.main["body"], env, set())
        self.out_types = []
        for k, o in enumerate(self.main["outputs"]):
            if o not in env:
                _refuse(self.main["line"], "output %s is not assigned" % o, "BAD_MATLAB")
            t = env[o]
            want = out_types[k] if out_types and k < len(out_types) else None
            if want is not None and t[0] == "vec":
                _refuse(self.main["line"], "a declared type on the vector output %s (not recorded)" % o)
            if want is not None and want != t:
                _refuse(self.main["line"], "output %s has type %s, the port is declared %s (Simulink: incompatible "
                        "variable types; convert in the code)" % (o, tstr(t), tstr(want)))
            self.out_types.append(t)

    # -- statements (static)
    def _block(self, f, stmts, env, loopvars):
        env = dict(env)
        for s in stmts:
            op = s["op"]
            if op == "persistent":
                for nm in s["names"]:
                    self.persistent.add((f["name"], nm))
            elif op == "assign" and s.get("index") is not None:
                # h(k) = e / h(a:b) = vector -- the element type kept, e quantised and overflowed
                # by the block fimath like x(:) = e
                nm = s["target"]
                vt = env.get(nm) or self.types.get((f["name"], nm))
                if vt is None:
                    _refuse(s["line"], "%s(...) = before %s has a type" % (nm, nm), "BAD_MATLAB")
                if vt[0] != "vec":
                    _refuse(s["line"], "indexing the scalar %s (not recorded)" % nm)
                kind, val = self._index(f, s["index"], env, s["line"], vt[1], nm)
                t = self._type(f, s["expr"], env, s["line"])
                et = vt[2]
                if s["expr"].get("_fa") is not None:
                    _refuse(s["line"], "%s(...) = a value carrying its own fimath (not recorded)" % nm)
                if kind == "range":
                    if t[0] != "vec" or t[1] != val[1] - val[0] + 1 or t[2] != et:
                        _refuse(s["line"], "%s(%d:%d) = %s (recorded: a slice of the same element type)"
                                % (nm, val[0], val[1], tstr(t)))
                elif t[0] == "vec" or not (t[0] in ("fi", "int", "dbl") and et[0] in ("fi", "int") or
                                           t == et == ("bool",)):
                    _refuse(s["line"], "%s(k) = a %s into a %s element (not recorded)" % (nm, tstr(t), tstr(et)))
                elif et[0] == "int" and t not in (et, ("dbl",)):
                    _refuse(s["line"], "%s(k) = a %s into a %s element (not recorded)" % (nm, tstr(t), tstr(et)))
                s["_ix"] = (kind, val)
                s["_t"] = vt
            elif op == "assign":
                nm = s["target"]
                ex = s["expr"]
                t = self._type(f, ex, env, s["line"])
                fa = ex.get("_fa")
                key = (f["name"], nm)
                if s["whole"]:
                    if fa is not None or self.fa.get(key) is not None:
                        _refuse(s["line"], "%s(:) = with a value or variable that carries its own fimath (not "
                                "recorded)" % nm)
                else:
                    if key in self.fa and self.fa[key] != fa or (key in self.types and key not in self.fa
                                                                 and fa is not None):
                        _refuse(s["line"], "%s gets values with different fimaths (not recorded)" % nm)
                    self.fa[key] = fa
                if s["whole"]:
                    if t[0] == "vec" or env.get(nm, ("",))[0] == "vec":
                        _refuse(s["line"], "%s(:) = with a vector (not recorded)" % nm)
                    if t[0] not in ("fi", "int", "dbl") and not (t == ("bool",) and env.get(nm) == ("bool",)):
                        # (st.b(:) = ~st.b, st.b(:) = true -- a boolean into a boolean)
                        _refuse(s["line"], "%s(:) = of a %s" % (nm, tstr(t)))
                    if nm not in env:
                        _refuse(s["line"], "%s(:) = before %s has a type" % (nm, nm), "BAD_MATLAB")
                    if env[nm][0] != "fi" and t != env[nm]:
                        _refuse(s["line"], "%s(:) = for a non fixed-point variable of another type" % nm)
                    s["_t"] = env[nm]
                else:
                    old = self.types.get(key)
                    if s.get("_struct_copy") and old is not None and old != t:
                        # st = next with next.c sfix10_En4, st.c sfix8_En4
                        _refuse(s["line"], "struct mismatch: the field %s is %s, %s.%s is %s (Simulink: incompatible "
                                "field types)" % (nm, tstr(old), s["_struct_copy"], nm.split(".", 1)[1], tstr(t)),
                                "BAD_MATLAB")
                    self._fix(f, nm, t, s["line"], env)
                    env[nm] = t
                    if t == ("dbl",) and stmts is f["body"] and self._assign_count(f, nm) == 1:
                        v = self._fold(f, ex, env)
                        if v is not None:
                            self.dconst[key] = v
                    if t == ("fm",):
                        if stmts is not f["body"] or self._assign_count(f, nm) != 1:
                            _refuse(s["line"], "a fimath variable assigned more than once or in a branch (not "
                                    "recorded)")
                        if ex.get("op") == "var" and (f["name"], ex["name"]) in self.fmvar:
                            self.fmvar[key] = self.fmvar[(f["name"], ex["name"])]
                        elif ex.get("op") == "call" and ex.get("_fm") is not None:
                            self.fmvar[key] = ex["_fm"]
                        else:
                            _refuse(s["line"], "a fimath variable from %s (recorded: F = hdlfimath)"
                                    % (ex.get("name") or ex.get("op")))
            elif op == "nop":
                pass
            elif op == "if":
                branches = []
                for cond, body in s["arms"]:
                    ct = self._type(f, cond, env, s["line"])
                    if ct[0] in ("fi", "int"):
                        # recordings: `if a` with a fixed-point a is true when a ~= 0
                        _nonzero(cond)
                        ct = self._type(f, cond, env, s["line"])
                    if ct[0] not in ("bool",):
                        _refuse(s["line"], "an if condition of type %s (boolean observed)" % tstr(ct))
                    e2 = dict(env)
                    # recordings (recordings row): a persistent typed in an earlier arm (`if isempty(c), c = fi(...);
                    # else c(:) = c + a;`) has that type in the later arms -- its type is the program's, not the path's
                    for b_ in branches:
                        for k_, v_ in b_.items():
                            if k_ not in e2 and (f["name"], k_) in self.persistent:
                                e2[k_] = v_
                    init = P._is_isempty(cond)
                    branches.append(self._block(f, body, e2, loopvars))
                e_else = dict(env)
                for b_ in branches:
                    for k_, v_ in b_.items():
                        if k_ not in e_else and (f["name"], k_) in self.persistent:
                            e_else[k_] = v_
                branches.append(self._block(f, s["else"], e_else, loopvars) if s["else"] is not None else dict(env))
                merged = dict(env)
                for b in branches:
                    for k, v in b.items():
                        if k in merged and merged[k] != v:
                            _refuse(s["line"], "%s gets the types %s and %s on different paths (Simulink: incompatible "
                                    "variable types)" % (k, tstr(merged[k]), tstr(v)))
                        merged[k] = v
                env = merged
            elif op == "for":
                if s.get("bounds") is not None:
                    # N = 8; for k = 1:N, 0:N-1, 1:a.WordLength -- folded to integers
                    vals = [self._fold(f, x, env) for x in s["bounds"]]
                    if any(v is None or isinstance(v, bool) or v.denominator != 1 for v in vals):
                        _refuse(s["line"], "for-loop bounds must be integer constants (constant variables, fi "
                                "properties and literal arithmetic are folded)")
                    lo, hi, st_ = (int(v) for v in vals)
                    if st_ == 0:
                        _refuse(s["line"], "a for step of 0", "BAD_MATLAB")
                    s["lo"], s["hi"], s["step"] = lo, hi, st_
                env[s["var"]] = ("dbl",)
                env = self._block(f, s["body"], env, loopvars | {s["var"]})
                env = self._block(f, s["body"], env, loopvars | {s["var"]})   # stable after a second pass
        return env

    def _fix(self, f, nm, t, line, env):
        key = (f["name"], nm)
        old = self.types.get(key)
        if old is not None and old != t:
            _refuse(line, "%s gets the type %s after %s (Simulink: incompatible variable types; use %s(:) = to keep "
                    "the type)" % (nm, tstr(t), tstr(old), nm))
        self.types[key] = t

    def _assign_count(self, f, nm):
        n = [0]

        def walk(st):
            for x in st:
                if x["op"] == "assign" and x["target"] == nm:
                    n[0] += 1
                elif x["op"] == "if":
                    for _c, b in x["arms"]:
                        walk(b)
                    walk(x["else"] or [])
                elif x["op"] == "for":
                    n[0] += 1 if x["var"] == nm else 0
                    walk(x["body"])
        walk(f["body"])
        return n[0]

    def _fold(self, f, e, env):
        """The value of a compile-time constant expression (Fraction; a bool for .Signed), else None: numbers,
        unary minus, + - * of constants, c ^ integer, nextpow2(c), x.WordLength / .FractionLength / .Signed of a
        fixed-point x, and constant double variables (self.dconst)."""
        op = e.get("op")
        if op == "num":
            # a literal beyond the doubles is Inf in MATLAB -- not folded;
            # a literal is its binary64 value
            return P.num_value(e["text"])
        if op == "neg":
            v = self._fold(f, e["args"][0], env)
            return None if v is None or isinstance(v, bool) else -v
        if op in ("+", "-", "*"):
            a, b = (self._fold(f, x, env) for x in e["args"])
            if a is None or b is None or isinstance(a, bool) or isinstance(b, bool):
                return None
            # MATLAB computes constant arithmetic in double (0.3 / 0.1 is 2.9999999999999996)
            r = float(a) + float(b) if op == "+" else float(a) - float(b) if op == "-" else float(a) * float(b)
            return Fraction(r) if math.isfinite(r) else None
        if op == "^":
            a, b = (self._fold(f, x, env) for x in e["args"])
            if a is None or b is None or isinstance(a, bool) or isinstance(b, bool) or b.denominator != 1 or \
                    abs(b) > 64 or (a == 0 and b < 0):
                return None
            v = a ** int(b)
            # MATLAB computes it in double: fold only when that is exact (10 ^ -1 is the literal 0.1 the engine
            # already reads as 1/10; 3 ^ 40 is not a double)
            if v.denominator & (v.denominator - 1) == 0 and Fraction(float(v)) != v:
                return None
            return v
        if op == "/":
            a, b = (self._fold(f, x, env) for x in e["args"])
            if a is None or b is None or isinstance(a, bool) or isinstance(b, bool) or b == 0:
                return None
            r = float(a) / float(b)       # MATLAB divides in double
            return Fraction(r) if math.isfinite(r) else None
        if op == "call" and e["name"] == "log2" and len(e["args"]) == 1:
            v = self._fold(f, e["args"][0], env)
            if v is None or isinstance(v, bool) or v <= 0:
                return None
            return Fraction(math.log2(float(v)))     # log2(256) = 8, log2(100) = 6.64...
        if op == "call" and e["name"] == "nextpow2" and len(e["args"]) == 1:
            v = self._fold(f, e["args"][0], env)
            if v is None or isinstance(v, bool):
                return None
            # the smallest p with 2^p >= |v| (nextpow2(0) = 0); recordings: nextpow2(100) = 7
            a, p_ = abs(v), 0
            if a == 0:
                return Fraction(0)
            while Fraction(2) ** p_ < a:
                p_ += 1
            while p_ > -1100 and Fraction(2) ** (p_ - 1) >= a:
                p_ -= 1
            return Fraction(p_)
        if op == "prop" and e["name"] in ("WordLength", "FractionLength", "Signed"):
            b = e["base"]
            t = env.get(b["name"]) if b["name"] in env else self.types.get((f["name"], b["name"]))
            if t is None or t[0] != "fi":
                return None
            return {"WordLength": Fraction(t[2]), "FractionLength": Fraction(t[3]), "Signed": bool(t[1])}[e["name"]]
        if op == "var" and (f["name"], e["name"]) in self.dconst and not e.get("colon"):
            return self.dconst[(f["name"], e["name"])]
        return None

    def _cint(self, f, e, env, line, log2_trunc=False):
        """An integer constant argument (fi / numerictype parameters), folded (_fold)."""
        v = self._fold(f, e, env)
        if v is None or isinstance(v, bool):
            return _int_const(e, line)
        if v.denominator != 1 and v > 0 and log2_trunc and e.get("op") == "call" and e.get("name") == "log2":
            # fi(a, 1, 16, log2(100)) is sfix16_En6 (6.64 -> 6, no error); recorded
            # for a positive log2 directly as the argument only
            return int(v)
        if v.denominator != 1 and any(n.get("op") == "call" and n.get("name") == "log2" for n in P._nodes(e)):
            _refuse(line, "a non-integer log2(...) inside a larger expression as an integer argument (recorded: "
                    "log2 directly as the fraction length)")
        if v.denominator != 1:
            _refuse(line, "an integer constant is required here", "BAD_MATLAB")
        return int(v)

    # -- expressions (static): the type, and an annotation the evaluator uses
    def _type(self, f, e, env, line):
        self._ct = None
        t = self._type_inner(f, e, env, line)
        e["_t"] = t
        if e["op"] not in ("var", "index", "+", "-", "*", ".*", "concat") and not (
                e["op"] == "call" and e.get("_vec_ok")) and any(
                isinstance(x, dict) and x.get("_t", ("",))[0] == "vec" for x in e.get("args", [])):
            _refuse(line, '%s on a vector (recordings show element-wise + -.*, vector * scalar and x(k) '
                    "only)" % (e.get("name") or e["op"]))
        if e["op"] in ("+", "-", "*", ".*") and self._ct:
            e["_ctc"] = self._ct
        e["_fa"] = self._fa_expr(f, e, line)
        return t

    def _fa_expr(self, f, e, line):
        """The non-default fimath a value carries (None: the default / block fimath)."""
        op = e["op"]
        args = e.get("args", [])
        if op == "var":
            return self.fa.get((f["name"], e["name"]))
        if op == "index":
            return self.fa.get((f["name"], e["base"]["name"]))
        if op in ("num", "str"):
            return None
        fas = [x.get("_fa") for x in args if isinstance(x, dict)]
        if op in ("+", "-", "*", ".*"):
            got = {x for x in fas if x is not None}
            if len(got) > 1:
                _refuse(line, "operands with different fimaths (Simulink: the embedded.fimath of both operands must "
                        "be equal)")
            fa = got.pop() if got else None
            if fa is not None and e.get("_ctc"):
                c, ct = e["_ctc"]
                cv = _const_value(c, line)
                if real(ct, quant(cv, ct, NEAREST_SAT)) != cv:
                    _refuse(line, "a constant that is not exact in %s with an operand carrying its own fimath (its "
                            "rounding there was not recorded)" % tstr(ct))
            return fa
        if op in ("==", "~=", "<", "<=", ">", ">=", "&&", "||", "&", "|", "not"):
            return None
        if op == "call":
            nm = e["name"]
            if nm == "fi":
                if e.get("_fm_explicit"):
                    return e["_fm"] if e["_fm"] not in (None, self.default_fm) else None
                return args[0].get("_fa")
            if nm in ("removefimath", "fimath", "numerictype", "isempty", "logical", "true", "false") or \
                    nm in INT_CLASSES:
                return None
        if any(x is not None for x in fas):
            _refuse(line, "%s of a value that carries its own fimath (only +, -, *, fi() and removefimath() were "
                    "recorded)" % (e.get("name") or op))
        return None

    def _type_inner(self, f, e, env, line):
        op = e["op"]
        if op in ("^", "prop") or op == "call" and e["name"] in ("nextpow2", "log2"):
            v = self._fold(f, e, env)
            if isinstance(v, bool):
                # x.Signed: a boolean constant
                e.clear()
                e.update({"op": "var", "name": "true" if v else "false"})
                return ("bool",)
            if v is not None:
                e.clear()
                e.update({"op": "num", "text": str(v)})
                return ("dbl",)
            if op == "^":
                return self._power(f, e, env, line)
            if op == "prop":
                if e["name"] == "numerictype":
                    _refuse(line, "x.numerictype outside fi(x, x.numerictype, ...) (not recorded)")
                _refuse(line, "%s.%s of a value that is not fixed point (recorded: fi values)"
                        % (e["base"]["name"], e["name"]))
            _refuse(line, "%s of a value that is not a constant (recorded: a constant)" % e["name"])
        if op == "num":
            return ("dbl",)
        if op == "var":
            if e["name"] in ("true", "false"):
                return ("bool",)
            if e["name"] not in env:
                if (f["name"], e["name"]) in self.persistent:
                    return self.types.get((f["name"], e["name"])) or _refuse(e.get("line"), "persistent %s used before "
                                                                                           "its initialisation" % e["name"])
                _refuse(e.get("line"), "%s has no value here" % e["name"], "BAD_MATLAB")
            t = env[e["name"]]
            if e.get("colon") and t[0] == "vec":
                _refuse(e.get("line"), "%s(:) of a vector in an expression (recorded: scalars, where it is %s)"
                        % (e["name"], e["name"]))
            if t == ("dbl",) and e["name"] not in self._loopvars_all(f) and (f["name"], e["name"]) not in self.dconst:
                _refuse(e.get("line"), "a double variable at run time (only constants and loop indices)")
            return t
        if op == "str":
            return ("str",)
        args = e.get("args", [])
        if op == "index":
            bt = self._type(f, e["base"], env, line)
            if bt[0] != "vec":
                _refuse(line, "indexing the scalar %s (not recorded)" % e["base"]["name"])
            kind, val = self._index(f, args[0], env, line, bt[1], e["base"]["name"])
            if kind == "range":
                e["_r"] = val
                return ("vec", val[1] - val[0] + 1, bt[2])
            if kind == "const":
                e["_k"] = val
            else:
                e["_kx"] = True     # a run-time index (recordings buffer), checked when it runs
            return bt[2]
        if op == "range":
            _refuse(line, "a range a:b outside an index (not recorded)")
        if op == "concat":
            # [a, h(1:3)]: scalars and row vectors of one element type
            ets, n = [], 0
            for x in args:
                t_ = self._type(f, x, env, line)
                ets.append(t_[2] if t_[0] == "vec" else t_)
                n += t_[1] if t_[0] == "vec" else 1
            if len(set(ets)) != 1 or ets[0][0] not in ("fi", "int", "bool"):
                _refuse(line, "[ ] of elements of types %s (recorded: one fixed-point type)"
                        % ", ".join(sorted(set(tstr(t_) for t_ in ets))))
            return ("vec", n, ets[0])
        if op == "/" and self._fold(f, e, env) is not None:
            # constant / constant, in double (a + 1/3: recordings)
            v = self._fold(f, e, env)
            e.clear()
            e.update({"op": "num", "text": str(v)})
            return ("dbl",)
        if op == "/":
            # fixed point or integer / a double constant; the result keeps the dividend's type
            # (a / 3: sfix16_En8, Nearest: 128 / 3 -> 43; u / 3: int16, in double then half away: 101 / 3 -> 34)
            a, b = (self._type(f, x, env, line) for x in args)
            if b != ("dbl",) or a[0] not in ("fi", "int"):
                _refuse(line, "division %s / %s (recorded: a fixed-point or integer value divided by a double "
                        "constant)" % (tstr(a), tstr(b)))
            c = self._fold(f, args[1], env)
            if c is None or isinstance(c, bool) or c < 1:
                _refuse(line, "division by %s (recorded: constants 2, 3, 4)" % (c if c is not None else
                                                                                   "a non-constant"))
            if a[0] == "fi" and args[0].get("_fa") is not None:
                _refuse(line, "division of a value carrying its own fimath (not recorded)")
            if a[0] == "fi" and c.denominator != 1:
                # dividing by a constant not exact in the word length -- whether Simulink rounds the
                # divisor first (as for a * c) was not recorded (2, 3, 4 only)
                _refuse(line, "fixed-point division by %s (recorded: integer constants)" % c)
            e["_c"] = c
            return a
        if op in ("+", "-", "*", ".*"):
            a, b = (self._type(f, x, env, line) for x in args)
            sop = "*" if op == ".*" else op
            if a[0] == "vec" or b[0] == "vec":
                # element by element: equal widths, or a vector and a scalar
                if a[0] == "vec" and b[0] == "vec":
                    if a[1] != b[1]:
                        _refuse(line, "vectors of widths %d and %d" % (a[1], b[1]), "BAD_MATLAB")
                    if op == "*":
                        _refuse(line, "vector * vector is a matrix product in MATLAB (only .* and vector * scalar "
                                "were recorded)")
                w = a[1] if a[0] == "vec" else b[1]
                ea = a[2] if a[0] == "vec" else a
                eb = b[2] if b[0] == "vec" else b
                return ("vec", w, self._arith(sop, ea, eb, line, args))
            return self._arith(sop, a, b, line, args)
        if op == "neg":
            a = self._type(f, args[0], env, line)
            if a[0] in ("fi", "int") and _signed(a):
                return a
            if a == ("dbl",) and args[0]["op"] == "num":
                return a
            _refuse(line, "unary minus of %s" % tstr(a))
        if op in ("==", "~=", "<", "<=", ">", ">="):
            for x in args:
                t = self._type(f, x, env, line)
                if t[0] not in ("fi", "int", "dbl", "bool"):
                    _refuse(line, "comparison of %s" % tstr(t))
                if e.get("switch") and x is not args[0] and x.get("case_const"):
                    v = self._fold(f, x, env)
                    if v is None or isinstance(v, bool):
                        # show K = 3; case K only: a case value computed at run time is not
                        _refuse(line, "case %s is not a constant (recorded: a constant variable)" % x["name"])
                if e.get("switch") and x is args[0] and t[0] != "int":
                    # Simulink rejects a switch on a fixed-point value ("non-fi must
                    # be a constant"); only an integer switch was recorded
                    _refuse(line, "switch on a %s value (recorded: integers; Simulink rejects a fixed-point one)"
                            % tstr(t), "BAD_MATLAB" if t[0] == "fi" else "UNSUPPORTED_MATLAB")
            return ("bool",)
        if op in ("&&", "||", "&", "|", "not"):
            for x in args:
                t = self._type(f, x, env, line)
                if t[0] in ("fi", "int"):
                    # a fixed-point or integer operand is true when ~= 0 (a && u > 0,
                    # ~a, a | u)
                    _nonzero(x)
                    t = self._type(f, x, env, line)
                if t[0] != "bool":
                    _refuse(line, "logical operation on %s (boolean operands observed)" % tstr(t))
            return ("bool",)
        if op == "call":
            return self._call(f, e, env, line)
        _refuse(line, "expression %r" % op)

    def _index(self, f, ix, env, line, n, name):
        """('const', k) / ('range', (lo, hi)) / ('run', None) for an index of a vector of n elements."""
        if ix.get("op") == "range":
            lo, hi = (self._fold(f, x, env) for x in ix["args"])
            if any(v is None or isinstance(v, bool) or v.denominator != 1 for v in (lo, hi)):
                _refuse(line, "%s(a:b) with bounds that are not integer constants (not recorded)" % name)
            lo, hi = int(lo), int(hi)
            if not 1 <= lo <= hi <= n:
                _refuse(line, "%s(%d:%d) outside 1..%d" % (name, lo, hi, n), "BAD_MATLAB")
            return "range", (lo, hi)
        v = self._fold(f, ix, env)
        if v is not None and not isinstance(v, bool):
            if v.denominator != 1:
                _refuse(line, "a non-integer index of %s" % name, "BAD_MATLAB")
            if not 1 <= v <= n:
                _refuse(line, "%s(%d) outside 1..%d" % (name, int(v), n), "BAD_MATLAB")
            return "const", int(v)
        it = self._type(f, ix, env, line)
        if it[0] == "int" or it[0] == "fi" and it[3] <= 0 or it == ("dbl",):
            return "run", None
        _refuse(line, "an index of %s of type %s (recorded: integers)" % (name, tstr(it)))

    def _power(self, f, e, env, line):
        """a ^ 2 / a .^ 2 of a fixed-point scalar: a * a at full precision (sfix16_En8
        gives sfix32_En16); other exponents and operands are not recorded."""
        b, x = e["args"]
        k = self._fold(f, x, env)
        bt = self._type(f, b, env, line)
        if k != 2 or isinstance(k, bool) or bt[0] != "fi":
            _refuse(line, "power of %s with exponent %s (recorded: a fixed-point value squared, and constants)"
                    % (tstr(bt), k if k is not None else "a non-constant"))
        import copy
        e.clear()
        e.update({"op": "*", "args": [b, copy.deepcopy(b)]})
        return self._type_inner(f, e, env, line)

    def _loopvars_all(self, f):
        out = set()

        def walk(stmts):
            for s in stmts:
                if s["op"] == "for":
                    out.add(s["var"])
                    walk(s["body"])
                elif s["op"] == "if":
                    for _, b in s["arms"]:
                        walk(b)
                    if s["else"]:
                        walk(s["else"])
        walk(f["body"])
        return out

    def _arith(self, op, a, b, line, args):
        if a[0] == "bool" or b[0] == "bool":
            _refuse(line, "arithmetic on a boolean")
        if a == ("dbl",) and b == ("dbl",):
            if all(x["op"] in ("num", "neg") for x in args):
                return ("dbl",)
            _refuse(line, "double arithmetic at run time")
        if a[0] == "int" and b[0] == "int":
            if a != b:
                _refuse(line, "integers of different classes (%s, %s)" % (a[1], b[1]))
            return a
        if a[0] == "int" and b == ("dbl",) or b[0] == "int" and a == ("dbl",):
            _const_value(args[1] if a[0] == "int" else args[0], line)
            return a if a[0] == "int" else b
        plain_fi = a[0] == "fi" and b[0] == "fi"   # show two fixed-point signals (no constant, no integer)
        if a[0] == "int":
            a = fi_of_int(a)
        if b[0] == "int":
            b = fi_of_int(b)
        if a == ("dbl",) or b == ("dbl",):
            ft, c = (b, args[0]) if a == ("dbl",) else (a, args[1])
            cval = _const_value(c, line)
            if op in ("+", "-"):
                ct = ft   # the constant in the fi's own type
                lo, hi = _rng(ct[1], ct[2])
                if not lo <= cval * (Fraction(2) ** ct[3]) <= hi:
                    _refuse(line, "the constant %s does not fit %s (its overflow was not recorded)" % (cval, tstr(ct)))
            else:
                ct = ("fi", ft[1], ft[2], _best_fl(cval, ft[1], ft[2]))
            # rounded with the current fimath (default Nearest, block fimath Floor)
            self._ct = (c, ct)
            a, b = (ct, ft) if a == ("dbl",) else (ft, ct)
        # full precision (recordings for mixed signedness): the result is signed if an operand is;
        # integer bits (sign excluded) the larger + 1 for a sum, fraction the larger; a product adds the word lengths
        sa, sb = a[1], b[1]
        if not sa and not sb and op == "-" and not (self.usub_sat and plain_fi):
            # recordings (f15/probe recordings): the type is the unsigned sum rule and a negative difference follows the
            # fimath's overflow action -- recorded only under the default fimath (Saturate: 0)
            _refuse(line, "unsigned - unsigned (%s, %s): a negative difference follows the fimath's overflow action; "
                    "recorded only for two fixed-point signals under the default fimath (saturates to 0), not with a "
                    "constant, an integer class or a fimath in the function; cast an operand to a signed type first"
                    % (tstr(a), tstr(b)))
        # unsigned * unsigned: the full-precision product, unsigned (equal)
        s = 1 if (sa or sb) else 0
        if op == "*":
            return ("fi", s, a[2] + b[2], a[3] + b[3])
        fl = max(a[3], b[3])
        if sa != sb:
            # signed +/- unsigned gives
            # WL = max(WL - FL) + max(FL) + 2 -- the signed operand's integer part counts its sign bit
            # (sfix12_En4 + ufix12_En6 -> sfix16_En6; subtracting it gave sfix15_En6)
            ib = max(a[2] - a[3], b[2] - b[3])
        else:
            ib = max(a[2] - a[3] - sa, b[2] - b[3] - sb)
        return ("fi", s, s + ib + 1 + fl, fl)

    def _call(self, f, e, env, line):
        nm = e["name"]
        args = e["args"]
        if nm in self.funcs:
            g = self.funcs[nm]
            ts = [self._type(f, x, env, line) for x in args]
            # the AST carries one set of type annotations, so a second argument type signature gets
            # its own typed copy of the function (it ran with the last call's types before: silently wrong)
            specs = self._spec.setdefault(nm, {})
            sig = tuple(ts)
            if sig not in specs:
                if not specs:
                    specs[sig] = nm
                else:
                    if _has_persistent(g):
                        _refuse(line, "local function %s with persistent variables called with a second argument "
                                "type signature (%s): not recorded" % (nm, ", ".join(tstr(t) for t in sig)))
                    import copy
                    cp = copy.deepcopy(self._pristine[nm])
                    cp["name"] = "%s[signature %d]" % (nm, len(specs) + 1)
                    self.funcs[cp["name"]] = cp
                    specs[sig] = cp["name"]
            e["_spec"] = specs[sig]
            g = self.funcs[e["_spec"]]
            sub = dict(zip(g["inputs"], ts))
            sub = self._block(g, g["body"], sub, set())
            if len(g["outputs"]) != 1:
                _refuse(line, "a subfunction call must return one value")
            return sub[g["outputs"][0]]
        if nm == "isempty":
            return ("bool",)
        if nm == "zeros" or nm == "false" and len(args) == 2:
            # zeros(1, n), zeros(1, n, 'like', x), false(1, n) -- row vectors
            if len(args) not in (2, 4) or (len(args) == 4 and (args[2].get("op") != "str" or args[2]["text"] != "like")):
                _refuse(line, "%s(1, n) or zeros(1, n, 'like', x) only (recorded)" % nm)
            r_, n_ = (self._fold(f, x, env) for x in args[:2])
            if r_ != 1 or n_ is None or isinstance(n_, bool) or n_.denominator != 1 or not 1 <= n_ <= 64:
                _refuse(line, "%s with sizes other than 1 x n (recorded: row vectors)" % nm)
            e["_n"] = int(n_)
            if nm == "false":
                if len(args) != 2:
                    _refuse(line, "false(1, n) only")
                return ("vec", int(n_), ("bool",))
            if len(args) == 4:
                lt = self._type(f, args[3], env, line)
                if lt[0] not in ("fi", "int"):
                    _refuse(line, "zeros(..., 'like', x) of a %s (recorded: fixed point)" % tstr(lt))
                return ("vec", int(n_), lt)
            return ("vec", int(n_), ("dbl",))
        if nm == "hdlfimath":
            if args:
                _refuse(line, "hdlfimath with arguments")
            e["_fm"] = HDLFIMATH
            return ("fm",)
        if nm == "getmsb":
            # the top bit, ufix1 -- bitget(x, word length)
            xt = self._type(f, args[0], env, line) if len(args) == 1 else None
            if xt is None or xt[0] != "fi":
                _refuse(line, "getmsb of a fixed-point value only (recorded)")
            e["name"] = "bitget"
            e["args"] = [args[0], {"op": "num", "text": str(xt[2])}]
            return self._call(f, e, env, line)
        if nm == "bitcmp":
            # bitwise complement in the operand's own type (sfix16_En8; uint8 255 - v) -- bitxor
            # with the all-ones pattern of that type
            xt = self._type(f, args[0], env, line) if len(args) == 1 else None
            if xt is None or xt[0] != "fi":
                _refuse(line, "bitcmp of a fixed-point value only (recorded)")
            _, s_, wl_, fl_ = xt
            ones = Fraction(-1 if s_ else (1 << wl_) - 1, 1 << fl_) if fl_ >= 0 else None
            if ones is None:
                _refuse(line, "bitcmp of a type with a negative fraction length (not recorded)")
            cst = {"op": "call", "name": "fi", "args": [{"op": "num", "text": str(ones)}] + [
                {"op": "num", "text": str(v)} for v in (s_, wl_, fl_)], "line": e.get("line"), "col": e.get("col")}
            e["name"] = "bitxor"
            e["args"] = [args[0], cst]
            return self._call(f, e, env, line)
        if nm == "mod":
            # mod(int16 u, 8) / mod(u, int16(5)): the class, mathematical mod (non-negative for a
            # positive divisor); mod(fi, double) is rejected by Simulink
            if len(args) != 2:
                _refuse(line, "mod(x, m)")
            xt = self._type(f, args[0], env, line)
            mt = self._type(f, args[1], env, line)
            if xt[0] == "fi":
                _refuse(line, "mod of a fixed-point value and a double (Simulink: fixed-point and floating-point "
                        "inputs cannot be combined)", "BAD_MATLAB")
            m_ = args[1]
            if m_.get("op") == "call" and m_.get("name") == xt[1] and len(m_["args"]) == 1:
                m_ = m_["args"][0]
            mv = self._fold(f, m_, env)
            if xt[0] != "int" or mt not in (("dbl",), xt) or mv is None or isinstance(mv, bool) or \
                    mv.denominator != 1 or mv < 1:
                _refuse(line, "mod(%s, %s) (recorded: an integer class and a positive integer constant)"
                        % (tstr(xt), tstr(mt)))
            lo_, hi_ = _rng(*INT_CLASSES[xt[1]])
            if mv > hi_:
                # a divisor beyond the class (int16(40000) saturates to 32767 in MATLAB; mod(int8, 300)
                # gave 299) -- not recorded
                _refuse(line, "mod by %s, beyond the %s range (not recorded)" % (mv, xt[1]))
            e["_m"] = int(mv)
            return xt
        if nm == "ufi":
            # ufi(x, wl, fl) = fi(x, 0, wl, fl) (Nearest, saturating at 0 / the top)
            if len(args) != 3:
                _refuse(line, "ufi(x, wl, fl) only")
            e["name"] = "fi"
            e["args"] = [args[0], {"op": "num", "text": "0"}] + list(args[1:])
            return self._call(f, e, env, line)
        if nm in ("floor", "ceil"):
            if len(args) != 1:
                _refuse(line, "%s(x)" % nm)
            xt = self._type(f, args[0], env, line)
            # of sfix16_En8, floor gives sfix8 (int8) and ceil sfix9 (one more bit:
            # 127.996 goes up to 128); recorded on a signed fixed-point value with a fraction
            if xt[0] != "fi" or not xt[1] or not 0 < xt[3] < xt[2]:
                _refuse(line, "%s of %s (recorded: a signed fixed-point value with a fraction)" % (nm, tstr(xt)))
            # recordings (recorded): floor of sfixN_En(N-1) is sfix2 -- at least 2 bits
            return ("fi", 1, max(2, xt[2] - xt[3] + (1 if nm == "ceil" else 0)), 0)
        if nm in ("numerictype",):
            if len(args) != 3:
                _refuse(line, "numerictype(s, wl, fl) only")
            vals = [self._cint(f, x, env, line) for x in args]
            e["_nt"] = tuple(vals)
            return ("nt",)
        if nm == "fimath":
            if len(args) == 1 and args[0]["op"] == "var":
                t = self._type(f, args[0], env, line)
                if t[0] != "fi":
                    _refuse(line, "fimath of a %s" % tstr(t))
                e["_fm"] = args[0].get("_fa")   # the operand's fimath (None: the default / block one)
                return ("fm",)
            e["_fm"] = _parse_fimath_args(args, line, self.default_fm)
            return ("fm",)
        if nm == "fi":
            if not args:
                _refuse(line, "fi() without arguments")
            xt = self._type(f, args[0], env, line)
            rest = args[1:]
            fm = None
            if rest and rest[-1].get("op") == "call" and rest[-1]["name"] in ("fimath", "hdlfimath"):
                self._type(f, rest[-1], env, line)
                fm = rest[-1]["_fm"]
                e["_fm_explicit"] = True
                rest = rest[:-1]
            elif rest and rest[-1].get("op") == "var" and (f["name"], rest[-1]["name"]) in self.fmvar:
                # F = hdlfimath; fi(a, 1, 10, 6, F)
                fm = self.fmvar[(f["name"], rest[-1]["name"])]
                e["_fm_explicit"] = True
                rest = rest[:-1]
                args = args[:-1]
                e["args"] = args
            if len(rest) > 3 and len(rest) % 2 == 1 and all(x.get("op") == "str" for x in rest[3::2]) and fm is None:
                # fi(x, s, wl, fl, 'OverflowAction', 'Saturate', ...): the fimath property pairs set the result's
                # fimath, as fimath(...) would
                fm = _parse_fimath_args(rest[3:], line, args[0].get("_fa") or self.default_fm)
                e["_fm_explicit"] = True
                rest = rest[:3]
            if len(rest) == 1 and rest[0].get("op") == "prop" and rest[0]["name"] == "numerictype":
                # fi(x, a.numerictype, ...): a's type (the register bundle's fields)
                b = rest[0]["base"]
                bt = env.get(b["name"]) or self.types.get((f["name"], b["name"]))
                if bt is None or bt[0] != "fi":
                    _refuse(line, "%s.numerictype of a value that is not fixed point" % b["name"])
                s, wl, fl = bt[1], bt[2], bt[3]
                rest[0].clear()
                rest[0].update({"op": "call", "name": "numerictype", "args": [{"op": "num", "text": str(x)} for x in
                                                                             (s, wl, fl)], "_nt": (s, wl, fl)})
            elif len(rest) == 1 and rest[0].get("op") == "call" and rest[0]["name"] == "numerictype":
                self._type(f, rest[0], env, line)
                s, wl, fl = rest[0]["_nt"]
            elif len(rest) == 3:
                s, wl = (self._cint(f, x, env, line) for x in rest[:2])
                fl = self._cint(f, rest[2], env, line, log2_trunc=True)
            else:
                _refuse(line, "fi(x, s, wl, fl[, fimath]) or fi(x, numerictype(s, wl, fl)[, fimath]) only")
            if xt[0] == "vec" and xt[2] == ("dbl",) and args[0].get("op") == "call" and args[0]["name"] == "zeros":
                # fi(zeros(1, n), s, wl, fl): a row vector of fixed-point zeros
                e["_vec_ok"] = True
                e["_fm"] = fm
                e["_qfm"] = NEAREST_SAT
                return ("vec", xt[1], ("fi", s, wl, fl))
            if xt[0] not in ("fi", "int", "dbl"):
                _refuse(line, "fi of %s" % tstr(xt))
            if xt == ("dbl",):
                _const_value(args[0], line)
            if s not in (0, 1) or not 1 <= wl <= 128:
                _refuse(line, "fi type out of range", "BAD_MATLAB")
            e["_fm"] = fm
            # without a fimath argument: the operand's own fimath when it carries one (fi(c + a,
            # 1, 8, 4) rounds by c's Floor / Wrap)
            e["_qfm"] = args[0].get("_fa")
            int_like = xt[0] == "fi" and xt[3] == 0 and xt[2] in (8, 16, 32) and args[0].get("_fa") is None
            if xt[0] != "fi" or int_like:
                # recordings (recordings re-check): the integer rule follows x's type under BOTH
                # TreatAsFi settings -- an sfix16_En0 product under the default 'Fixed-point' saturates as well (it was
                # applied only with integer inputs and 'Fixed-point & Integer': 9 / 9 differed)
                # recordings (54 fresh designs, recordings/fresh recordings* / v* / q*; it decided by how x was written,
                # recordings, and over-corrected 26 / 54): with TreatAsFi 'Fixed-point & Integer' (integer inputs made
                # fi), fi(x, ...) takes the integer rule exactly when x's type is that of an integer class -- word
                # length 8 / 16 / 32, fraction length 0 (int8 * int8 -> sfix16 saturates, int8 + int8 -> sfix9 and
                # int8 * int16 -> sfix24 wrap by the block fimath, a * 3 -> sfix16_En5 wraps, z = a * b; fi(z, ...)
                # saturates)
                # fi(constant / integer, ...) without a fimath argument
                # quantises Nearest / Saturate even when the block fimath is Floor / Wrap, with either
                # FimathForFiConstructors setting (fi(0.3, 1, 8, 4) = 5/16, fi(10, 1, 8, 4) = 127, fi(int16 u,
                # 1, 8, 4) saturates); a fixed-point operand keeps the block / its own fimath
                e["_qfm"] = NEAREST_SAT
            return ("fi", s, wl, fl)
        if nm == "removefimath":
            if len(args) != 1:
                _refuse(line, "removefimath(x)")
            xt = self._type(f, args[0], env, line)
            if xt[0] != "fi":
                _refuse(line, "removefimath of a %s" % tstr(xt))
            return xt
        if nm == "cast":
            if len(args) != 3 or args[1].get("op") != "str" or args[1]["text"] != "like":
                _refuse(line, "cast(x, 'like', y) only")
            xt, yt = self._type(f, args[0], env, line), self._type(f, args[2], env, line)
            if yt[0] == "fi" and xt == ("dbl",):
                # recordings (78 / 78): cast(c, 'like', a) of a compile-time double c (a literal, a
                # constant expression, or a double variable assigned it once) is c quantised to a's type by the
                # block fimath (default Nearest / Saturate; a UserSpecified Floor / Wrap floors and wraps), folded
                # at compile time. Recorded with a plain block input as a; a 'like' value carrying its own fimath
                # was not recorded.
                cv = self._fold(f, args[0], env)
                if cv is None or isinstance(cv, bool):
                    _refuse(line, "cast of a double that is not a compile-time constant (recorded: constants)")
                if args[2].get("_fa") is not None or (args[2].get("op") == "var" and
                                                      (f["name"], args[2]["name"]) in self.fa):
                    _refuse(line, "cast(c, 'like', x) of a constant with an x carrying its own fimath (recorded: "
                            "the block fimath)")
                e["_cval"] = cv
                return yt
            if yt[0] != "fi" or xt[0] not in ("fi", "int"):
                _refuse(line, "cast to a %s / of a %s (fixed point observed)" % (tstr(yt), tstr(xt)))
            return yt
        if nm in INT_CLASSES:
            if len(args) != 1:
                _refuse(line, "%s(x)" % nm)
            xt = self._type(f, args[0], env, line)
            if xt[0] == "vec" and xt[2] == ("dbl",) and args[0].get("op") == "call" and args[0]["name"] == "zeros":
                e["_vec_ok"] = True     # int16(zeros(1, n))
                return ("vec", xt[1], ("int", nm))
            if xt[0] not in ("fi", "int", "dbl"):
                _refuse(line, "%s of %s" % (nm, tstr(xt)))
            if xt == ("dbl",):
                _const_value(args[0], line)
            return ("int", nm)
        if nm in ("true", "false"):
            return ("bool",)
        if nm == "logical":
            self._type(f, args[0], env, line)
            return ("bool",)
        ts = [self._type(f, x, env, line) if x.get("op") != "str" else ("str",) for x in args]
        if nm in ("bitshift", "bitsll", "bitsrl", "bitsra"):
            if len(args) != 2 or ts[0][0] not in ("fi", "int"):
                _refuse(line, "%s(x, n) of a fixed-point or integer x" % nm)
            n = _int_const(args[1], line)
            if ts[0][0] == "int" and nm in ("bitshift", "bitsll") and n > 0:
                _refuse(line, "%s left of an integer class is not observed" % nm)
            if nm != "bitshift" and n < 0:
                _refuse(line, "%s with a negative shift" % nm)
            e["_n"] = n
            return ts[0]
        if nm == "bitsliceget":
            if len(args) != 3 or ts[0][0] != "fi":
                _refuse(line, "bitsliceget(x, hi, lo) of a fixed-point x")
            hi, lo = _int_const(args[1], line), _int_const(args[2], line)
            if not 1 <= lo <= hi <= ts[0][2]:
                _refuse(line, "bitsliceget bit range %d..%d outside 1..%d" % (lo, hi, ts[0][2]), "BAD_MATLAB")
            e["_r"] = (hi, lo)
            return ("fi", 0, hi - lo + 1, 0)
        if nm == "bitconcat":
            if len(args) < 2 or any(t[0] != "fi" for t in ts):
                _refuse(line, "bitconcat of fixed-point operands")
            return ("fi", 0, sum(t[2] for t in ts), 0)
        if nm in ("bitget", "bitset"):
            if len(args) != 2 or ts[0][0] != "fi":
                _refuse(line, "%s(x, n) of a fixed-point x (bitset(x, n, v) not observed)" % nm)
            n = _int_const(args[1], line)
            if not 1 <= n <= ts[0][2]:
                _refuse(line, "%s bit %d outside 1..%d" % (nm, n, ts[0][2]), "BAD_MATLAB")
            e["_n"] = n
            return ("fi", 0, 1, 0) if nm == "bitget" else ts[0]
        if nm in ("bitand", "bitor", "bitxor") and len(args) == 2 and ts[0][0] == "int" and ts[0] == ts[1]:
            # bitand(int32(u), int32(3)), bitand(uint8(.), uint8(3)) --
            # the class's two's-complement pattern, the result in the class
            return ts[0]
        if nm in ("bitand", "bitor", "bitxor"):
            if len(args) != 2 or ts[0][0] != "fi" or ts[0] != ts[1]:
                _refuse(line, "%s needs two fixed-point operands of the same type (Simulink: matching operand "
                        "types)" % nm)
            return ts[0]
        if nm in ("min", "max"):
            if len(args) != 2:
                _refuse(line, "%s(a, b) only" % nm)
            a, b = ts
            if a[0] == "int" and a == b:
                return a
            if a[0] == "int" and b[0] == "int":
                _refuse(line, "%s of integers of different classes (%s, %s)" % (nm, a[1], b[1]))
            if a[0] == "int":
                a = fi_of_int(a)
            if b[0] == "int":
                b = fi_of_int(b)
            if a[0] != "fi" or b[0] != "fi":
                _refuse(line, "%s of %s and %s (not observed)" % (nm, tstr(a), tstr(b)))
            # the union, whatever the order: signed if either is, integer bits (sign excluded)
            # and fraction bits the larger
            fl = max(a[3], b[3])
            sg = 1 if (a[1] or b[1]) else 0
            ib = max(a[2] - a[3] - a[1], b[2] - b[3] - b[1])
            return ("fi", sg, sg + ib + fl, fl)
        if nm == "abs":
            if len(args) != 1 or ts[0][0] not in ("fi", "int"):
                _refuse(line, "abs of a fixed-point or integer value")
            return ts[0]
        _refuse(line, "call of %s" % nm)

    # ---------------------------------------------------------------- run
    def step(self, inputs):
        """inputs: stored integers (bools 0 / 1) in port order -> output stored integers."""
        env = {}
        for nm, t, v in zip(self.main["inputs"], self.in_types, inputs):
            env[nm] = (t, [int(x) for x in v]) if t[0] == "vec" else (t, int(v))
        env = self._run(self.main, self.main["body"], env)
        return [env[o][1] for o in self.main["outputs"]]

    def _run(self, f, stmts, env):
        for s in stmts:
            op = s["op"]
            if op == "persistent":
                for nm in s["names"]:
                    if (f["name"], nm) in self.persist:
                        env[nm] = self.persist[(f["name"], nm)]
            elif op == "assign" and s.get("_ix") is not None:
                nm = s["target"]
                vt = s["_t"]
                kind, val = s["_ix"]
                cur = list(env[nm][1])
                v = self._eval(f, s["expr"], env)
                if kind == "range":
                    cur[val[0] - 1:val[1]] = list(v[1])
                else:
                    k = val if kind == "const" else self._run_index(f, s["index"], env, vt[1], nm, s["line"])
                    cur[k - 1] = self._convert(v, vt[2], self.default_fm)[1]
                env[nm] = (vt, cur)
                if (f["name"], nm) in self.persistent:
                    self.persist[(f["name"], nm)] = env[nm]
            elif op == "assign":
                nm = s["target"]
                v = self._eval(f, s["expr"], env)
                if s["whole"]:
                    t = s["_t"]
                    v = self._convert(v, t, self._fm_of(env.get(nm)))
                env[nm] = v
                if (f["name"], nm) in self.persistent:
                    self.persist[(f["name"], nm)] = v
            elif op == "if":
                for cond, body in s["arms"]:
                    if self._truth(self._eval(f, cond, env)):
                        env = self._run(f, body, env)
                        break
                else:
                    if s["else"] is not None:
                        env = self._run(f, s["else"], env)
            elif op == "for":
                k = s["lo"]
                while (k <= s["hi"]) if s["step"] > 0 else (k >= s["hi"]):
                    env[s["var"]] = (("dbl",), Fraction(k))
                    env = self._run(f, s["body"], env)
                    k += s["step"]
        return env

    def _run_index(self, f, ix, env, n, name, line):
        """A run-time index: an integer 1..n, else refused when it happens (an out-of-range index was not
        recorded: the recorded ring buffer stays in range)."""
        v = self._val(self._eval(f, ix, env))
        if v.denominator != 1 or not 1 <= v <= n:
            _refuse(line or 0, "%s(%s) outside 1..%d at run time (an out-of-range index was not recorded)"
                    % (name, v, n))
        return int(v)

    def _fm_of(self, v):
        return self.default_fm

    def _truth(self, v):
        return v[1] != 0

    def _convert(self, v, t, fm):
        """Value v -> type t (y(:) = ...): quantise with fimath fm (integers: round half away, saturate / wrap)."""
        x = real(*v) if v[0] != ("dbl",) else v[1]
        if t[0] == "fi":
            return (t, quant(x, t, fm))
        if t[0] == "int":
            s, wl = INT_CLASSES[t[1]]
            return (t, _ovf(_round(x, "Round"), s, wl, "Saturate" if self.int_sat else "Wrap"))
        return (t, 1 if x != 0 else 0)

    def _eval(self, f, e, env):
        op = e["op"]
        t = e.get("_t")
        if op == "num":
            return (("dbl",), P.num_value(e["text"]))
        if op == "var":
            if e["name"] == "true":
                return (("bool",), 1)
            if e["name"] == "false":
                return (("bool",), 0)
            if e["name"] not in env:
                # a persistent read before it is assigned on this step's path (MATLAB: s is [];
                # its behaviour is not recorded) -- refused, never read as 0 (the validator refuses these first)
                _refuse(e.get("line", 0), "%s is read before it is assigned (a persistent variable not initialised on "
                        "this path)" % e["name"])
            return env[e["name"]]
        args = e.get("args", [])
        if op == "index":
            v = env[e["base"]["name"]]
            if "_r" in e:
                lo, hi = e["_r"]
                return (("vec", hi - lo + 1, v[0][2]), list(v[1][lo - 1:hi]))
            k = e["_k"] if "_k" in e else self._run_index(f, args[0], env, v[0][1], e["base"]["name"], e.get("line"))
            return (v[0][2], v[1][k - 1])
        if op == "concat":
            out = []
            for x in args:
                v = self._eval(f, x, env)
                out += list(v[1]) if v[0][0] == "vec" else [v[1]]
            return (t, out)
        if op == "/":
            a = self._eval(f, args[0], env)
            if t[0] == "int":
                s_, wl_ = INT_CLASSES[t[1]]
                return (t, _ovf(int_dbl("/", a[1], e["_c"], True), s_, wl_, "Saturate" if self.int_sat else "Wrap"))
            # rounded by the fimath: the default Nearest, a block Floor / Wrap one by Floor
            #
            return (t, quant(self._val(a) / Fraction(float(e["_c"])), t, self.default_fm))
        if op in ("+", "-", "*", ".*"):
            a, b = (self._eval(f, x, env) for x in args)
            sop = "*" if op == ".*" else op
            if t[0] == "vec":
                def el(v, j):
                    return (v[0][2], v[1][j]) if v[0][0] == "vec" else v
                return (t, [self._arith_val(sop, el(a, j), el(b, j), t[2])[1] for j in range(t[1])])
            return self._arith_val(sop, a, b, t)
        if op == "neg":
            a = self._eval(f, args[0], env)
            if a[0] == ("dbl",):
                return (a[0], -a[1])
            if a[0][0] == "fi":
                return (a[0], _ovf(-a[1], a[0][1], a[0][2], self.default_fm[1]))
            s, wl = INT_CLASSES[a[0][1]]
            return (a[0], _ovf(-a[1], s, wl, "Saturate" if self.int_sat else "Wrap"))
        if op in ("==", "~=", "<", "<=", ">", ">="):
            a, b = (self._val(self._eval(f, x, env)) for x in args)
            r = {"==": a == b, "~=": a != b, "<": a < b, "<=": a <= b, ">": a > b, ">=": a >= b}[op]
            return (("bool",), int(r))
        if op in ("&&", "||", "&", "|"):
            a = self._truth(self._eval(f, args[0], env))
            if op == "&&" and not a:
                return (("bool",), 0)
            if op == "||" and a:
                return (("bool",), 1)
            b = self._truth(self._eval(f, args[1], env))
            return (("bool",), int(a and b) if op in ("&&", "&") else int(a or b))
        if op == "not":
            return (("bool",), int(not self._truth(self._eval(f, args[0], env))))
        if op == "call":
            return self._call_val(f, e, env)
        raise RuntimeError(op)

    def _val(self, v):
        return v[1] if v[0] == ("dbl",) else (real(v[0], v[1]) if v[0][0] in ("fi", "int") else Fraction(v[1]))

    def _arith_val(self, op, a, b, rt):
        va, vb = self._val(a), self._val(b)
        r = va + vb if op == "+" else va - vb if op == "-" else va * vb
        if rt == ("dbl",):
            return (rt, r)
        if rt[0] == "int":
            s, wl = INT_CLASSES[rt[1]]
            if a[0] == ("dbl",) or b[0] == ("dbl",):
                # in double, then rounded half away from zero
                n = int_dbl(op, vb if a[0] == ("dbl",) else va, va if a[0] == ("dbl",) else vb, a[0] != ("dbl",))
            else:
                n = int(r)
            return (rt, _ovf(n, s, wl, "Saturate" if self.int_sat else "Wrap"))
        # fixed point, full precision; a double constant is first put in its type
        if a[0] == ("dbl",) or b[0] == ("dbl",):
            ft = b[0] if a[0] == ("dbl",) else a[0]
            if ft[0] == "int":
                ft = fi_of_int(ft)
            c = a[1] if a[0] == ("dbl",) else b[1]
            ct = ft if op in ("+", "-") else ("fi", ft[1], ft[2], _best_fl(c, ft[1], ft[2]))
            cq = real(ct, quant(c, ct, self.default_fm))
            fv = self._val(b if a[0] == ("dbl",) else a)
            if a[0] == ("dbl",):
                r = cq + fv if op == "+" else cq - fv if op == "-" else cq * fv
            else:
                r = fv + cq if op == "+" else fv - cq if op == "-" else fv * cq
        n = r * (Fraction(2) ** rt[3])
        assert n.denominator == 1, "full precision result not exact"
        if op == "-" and not rt[1]:
            # unsigned - unsigned under the default fimath: a negative difference saturates to 0
            assert self.usub_sat
            return (rt, max(0, int(n)))
        return (rt, int(n))

    def _call_val(self, f, e, env):
        nm = e["name"]
        args = e["args"]
        rt = e.get("_t")
        if nm in self.funcs:
            g = self.funcs[e.get("_spec", nm)]
            vals = [self._eval(f, x, env) for x in args]
            sub = self._run(g, g["body"], dict(zip(g["inputs"], vals)))
            return sub[g["outputs"][0]]
        if nm == "isempty":
            v = args[0]["name"]
            return (("bool",), int((f["name"], v) not in self.persist and v not in env))
        if nm in ("numerictype", "fimath", "hdlfimath"):
            return (rt, 0)
        if rt is not None and rt[0] == "vec" and nm in ("zeros", "false", "fi") + tuple(INT_CLASSES):
            return (rt, [Fraction(0) if rt[2] == ("dbl",) else 0] * rt[1])
        if nm == "mod":
            x = self._eval(f, args[0], env)
            return (rt, x[1] % e["_m"])
        if nm in ("floor", "ceil"):
            x = self._eval(f, args[0], env)
            return (rt, _round(self._val(x), "Floor" if nm == "floor" else "Ceiling"))
        if nm == "fi":
            x = self._eval(f, args[0], env)
            if e.get("_fm_explicit"):
                fm = e.get("_fm") or self.default_fm
            else:
                fm = e.get("_qfm") or self.default_fm
            return (rt, quant(self._val(x), rt, fm))
        if nm == "removefimath":
            return self._eval(f, args[0], env)
        if nm == "cast":
            if "_cval" in e:     # a compile-time double
                return (rt, quant(e["_cval"], rt, self.default_fm))
            x = self._eval(f, args[0], env)
            return (rt, quant(self._val(x), rt, self.default_fm))
        if nm in INT_CLASSES:
            x = self._eval(f, args[0], env)
            s, wl = INT_CLASSES[nm]
            # recordings (recorded): int8() .. uint32() wrap when SaturateOnIntegerOverflow is off
            return (rt, _ovf(_round(self._val(x), "Round"), s, wl, "Saturate" if self.int_sat else "Wrap"))
        if nm in ("true", "false"):
            return (("bool",), int(nm == "true"))
        if nm == "logical":
            return (("bool",), int(self._truth(self._eval(f, args[0], env))))
        vals = [self._eval(f, x, env) if x.get("op") != "str" else None for x in args]
        x = vals[0]
        if nm == "bitshift":
            n = e["_n"]
            t = x[0] if x[0][0] == "fi" else fi_of_int(x[0])
            if n >= 0:
                om = self.default_fm[1] if x[0][0] == "fi" else ("Saturate" if self.int_sat else "Wrap")
                return (x[0], _ovf(x[1] << n, t[1], t[2], om))
            return (x[0], x[1] >> -n)
        if nm in ("bitsll", "bitsrl", "bitsra"):
            t = x[0] if x[0][0] == "fi" else fi_of_int(x[0])
            n = e["_n"]
            s, wl = t[1], t[2]
            if nm == "bitsll":
                return (x[0], _ovf(x[1] << n, s, wl, "Wrap"))
            if nm == "bitsra":
                return (x[0], x[1] >> n)
            return (x[0], _ovf((x[1] % (1 << wl)) >> n, s, wl, "Wrap"))
        if nm == "bitsliceget":
            hi, lo = e["_r"]
            wl = x[0][2]
            return (rt, ((x[1] % (1 << wl)) >> (lo - 1)) & ((1 << (hi - lo + 1)) - 1))
        if nm == "bitconcat":
            out = 0
            for v in vals:
                w = v[0][2]
                out = (out << w) | (v[1] % (1 << w))
            return (rt, out)
        if nm == "bitget":
            wl = x[0][2]
            return (rt, ((x[1] % (1 << wl)) >> (e["_n"] - 1)) & 1)
        if nm == "bitset":
            t = x[0]
            p = (x[1] % (1 << t[2])) | (1 << (e["_n"] - 1))
            return (t, _ovf(p, t[1], t[2], "Wrap"))
        if nm in ("bitand", "bitor", "bitxor") and x[0][0] == "int":
            s_, wl_ = INT_CLASSES[x[0][1]]
            m = (1 << wl_) - 1
            pa, pb = x[1] & m, vals[1][1] & m
            p_ = pa & pb if nm == "bitand" else pa | pb if nm == "bitor" else pa ^ pb
            return (x[0], _ovf(p_, s_, wl_, "Wrap"))
        if nm in ("bitand", "bitor", "bitxor"):
            t = x[0]
            m = (1 << t[2]) - 1
            pa, pb = x[1] & m, vals[1][1] & m
            p = pa & pb if nm == "bitand" else pa | pb if nm == "bitor" else pa ^ pb
            return (t, _ovf(p, t[1], t[2], "Wrap"))
        if nm in ("min", "max"):
            va, vb = self._val(x), self._val(vals[1])
            v = min(va, vb) if nm == "min" else max(va, vb)
            if rt[0] == "int":
                return (rt, int(v))
            return (rt, int(v * (Fraction(2) ** rt[3])))
        if nm == "abs":
            if x[0][0] == "fi":
                return (x[0], _ovf(abs(x[1]), x[0][1], x[0][2], self.default_fm[1]))
            s, wl = INT_CLASSES[x[0][1]]
            return (x[0], _ovf(abs(x[1]), s, wl, "Saturate" if self.int_sat else "Wrap"))
        raise RuntimeError(nm)


def _signed(t):
    return t[1] if t[0] == "fi" else INT_CLASSES[t[1]][0]


def _round_half_away_double(v):
    """MATLAB round() of a double v (exact on the double value)."""
    import math
    fl = math.floor(v)
    frac = v - fl          # exact for |v| < 2^52
    if frac > 0.5 or frac == 0.5 and v > 0:
        return fl + 1
    return fl


def int_dbl(op, x, c, int_first):
    """An integer x op a double constant c: MATLAB computes it in IEEE double, then rounds half
    away from zero (int16(90) * 0.35 = 31.499999999999996 -> 31; the exact value 31.5 would give 32); the class's
    saturation follows."""
    cd = float(c)
    if op == "*":
        v = x * cd
    elif op == "+":
        v = x + cd
    elif op == "/":
        v = x / cd if int_first else cd / x      # u / 3 in double, then half away (101 / 3 -> 34)
    else:
        v = x - cd if int_first else cd - x
    return _round_half_away_double(v)


def int_dbl_same_as_exact(op, it, c, int_first):
    """Whether the double rule (int_dbl) gives the exact value's half-away rounding for every value of the
    integer class (the compiled blocks compute the exact one): a constant exact in double with an exact product,
    else every value of a class of up to 16 bits is tried; a 32-bit class with another constant: False."""
    s_, wl = INT_CLASSES[it[1]]
    if Fraction(float(c)) == c and (op in ("+", "-") or abs(c.numerator).bit_length() + wl <= 53):
        return True
    if wl > 16:
        return False
    lo, hi = _rng(s_, wl)
    p_, q_ = c.numerator, c.denominator
    for x in range(lo, hi + 1):
        if op == "*":
            n2 = 2 * x * p_
        elif op == "+":
            n2 = 2 * (x * q_ + p_)
        else:
            n2 = 2 * (x * q_ - p_) if int_first else 2 * (p_ - x * q_)
        d2 = 2 * q_
        ex = (n2 + d2 // 2) // d2 if n2 >= 0 else -((-n2 + d2 // 2) // d2)
        if ex != int_dbl(op, x, c, int_first):
            return False
    return True


def _finite_double(v):
    try:
        return math.isfinite(float(v))
    except OverflowError:
        return False


def _nonzero(x):
    """Rewrites the expression node x in place into (x ~= 0): a fixed-point or integer value used as a truth value."""
    orig = dict(x)
    x.clear()
    x.update({"op": "~=", "args": [orig, {"op": "num", "text": "0"}]})


def _const_value(e, line):
    if e["op"] == "num" or e["op"] == "neg" and e["args"][0]["op"] == "num":
        v = P.num_value((e if e["op"] == "num" else e["args"][0])["text"])
        if v is None:
            _refuse(line, "a constant beyond the doubles (Inf in MATLAB)")
        return v if e["op"] == "num" else -v
    _refuse(line, "a double value that is not a constant")


def _int_const(e, line):
    v = _const_value(e, line)
    if v.denominator != 1:
        _refuse(line, "an integer constant is required here", "BAD_MATLAB")
    return int(v)


def _parse_fimath_args(args, line, base=NEAREST_SAT):
    """The (rounding, overflow) of fimath name / value pairs; the properties not named come from base (
    recorded: the block fimath for fimath(...), the operand's fimath for fi(x, s, w, f, 'Name', value))."""
    if len(args) % 2:
        _refuse(line, "fimath(name, value, ...) pairs")
    rm, om = base
    for k in range(0, len(args), 2):
        a, b = args[k], args[k + 1]
        if a.get("op") != "str" or b.get("op") != "str":
            _refuse(line, "fimath(name, value, ...) with text names and values")
        if a["text"] == "RoundingMethod" and b["text"] in ("Nearest", "Floor", "Ceiling", "Zero", "Round",
                                                           "Convergent"):
            rm = b["text"]
        elif a["text"] == "OverflowAction" and b["text"] in ("Saturate", "Wrap"):
            om = b["text"]
        else:
            _refuse(line, "fimath property %s = %s (only RoundingMethod / OverflowAction)" % (a["text"], b["text"]))
    return (rm, om)


def _parse_fimath_text(text):
    """The block's InputFimath text, e.g. fimath('RoundingMethod','Floor','OverflowAction','Wrap')."""
    import re
    if (text or "").strip() in ("hdlfimath", "hdlfimath()"):
        # the block fimath hdlfimath = Floor / Wrap with
        # full-precision products and sums (recordings gives the same values for hdlfimath in the code)
        return HDLFIMATH
    # the whole text is parsed -- fimath('Name', 'Value', ...) with quoted pairs (and '...' line
    # continuations); anything else (Name=Value syntax, numbers, an unknown word) is refused, never defaulted
    t = re.sub(r"\.\.\.[^\n]*\n", " ", text or "").strip()
    m = re.fullmatch(r"fimath\s*\((.*)\)", t, re.S)
    if not m:
        _refuse(0, "InputFimath %r is not fimath('Name', 'Value', ...) or hdlfimath (other spellings are not read)"
                % (text or "")[:80])
    inner = m.group(1).strip()
    # elements: 'Name', 'Value' pairs, or Name='Value' (the same values as the
    # quoted spelling); anything else refused
    toks = re.findall(r"'([^']*)'|(,)|(=)|([A-Za-z]\w*)|(\S)", inner)
    pairs, quoted, i = [], [], 0
    while i < len(toks):
        q, comma, eq, ident, other = toks[i]
        if other:
            _refuse(0, "InputFimath %r: only 'Name', 'Value' or Name='Value' pairs are read" % inner[:80])
        if ident:
            if i + 2 >= len(toks) or not toks[i + 1][2] or toks[i + 2][1] or toks[i + 2][2] or toks[i + 2][3] \
                    or toks[i + 2][4]:
                _refuse(0, "InputFimath %r: Name='Value' with a quoted value only" % inner[:80])
            pairs.append((ident, toks[i + 2][0]))
            i += 3
        elif comma:
            i += 1
            continue
        else:
            quoted.append(q)
            i += 1
            continue
        if i < len(toks) and not toks[i][1]:
            _refuse(0, "InputFimath %r: a missing comma" % inner[:80])
    if len(quoted) % 2:
        _refuse(0, "InputFimath %r: names and values must come in pairs" % inner[:80])
    pairs += list(zip(quoted[0::2], quoted[1::2]))
    rm, om = NEAREST_SAT
    for k, v in pairs:
        if k == "RoundingMethod" and v not in ("Nearest", "Floor", "Ceiling", "Zero", "Round", "Convergent") or \
                k == "OverflowAction" and v not in ("Saturate", "Wrap"):
            _refuse(0, "InputFimath %s = %r" % (k, v))
        if k == "RoundingMethod":
            rm = v
        elif k == "OverflowAction":
            om = v
        elif k in ("ProductMode", "SumMode") and v == "FullPrecision":
            # written out with the block fimath; full precision is what the
            # engine computes anyway
            pass
        else:
            _refuse(0, "InputFimath property %s (only RoundingMethod / OverflowAction)" % k)
    return (rm, om)


MAX_RECORDED_WL = 67   # up to 32 bits; recordings: intermediates of 33 .. 67 bits (multiply-add, 64-bit accumulators, 3-tap FIR) and output ports of 40 .. 64 bits, both fimaths, 34 / 34 equal on both paths


def _widest(node, out):
    if isinstance(node, dict):
        t = node.get("_t")
        if isinstance(t, tuple):
            out.append(t)
        for v in node.values():
            _widest(v, out)
    elif isinstance(node, (list, tuple)):
        for v in node:
            _widest(v, out)
    return out


def _wl(t):
    if not isinstance(t, tuple) or not t:
        return 0
    if t[0] == "fi":
        return t[2]
    if t[0] == "vec":
        return _wl(t[2])
    return 0


def compile(text, in_types, chart=None, out_types=None):
    ast = P.validate(P.parse(text))
    prog = Program(ast, in_types, chart or {}, out_types or [])
    # refuse what is not recorded: a fixed-point value (a port, a variable, an intermediate result) wider than the
    # widest recorded is not computed
    ts = list(prog.types.values()) + list(prog.in_types) + list(prog.out_types) + _widest(list(prog.funcs.values()), [])   # every typed copy too
    w = max([_wl(t) for t in ts] or [0])
    if w > MAX_RECORDED_WL:
        _refuse(0, 'a %d-bit fixed-point value: MATLAB Function values are recorded up to %d bits'
                   '; not computed until Simulink is recorded there' % (w, MAX_RECORDED_WL))
    return prog


def widths(text, in_widths):
    """Output widths of a MATLAB Function from its input widths, without types (the engine's vector pass needs them
    before types are known): x(k) is a scalar, + - * .* the wider operand, a call or other operator is scalar (a
    vector argument is refused later, when the types are compiled)."""
    ast = P.validate(P.parse(text))
    funcs = {f["name"]: f for f in ast["functions"]}
    main = funcs[ast["main"]]
    if len(in_widths) != len(main["inputs"]):
        raise P.MlError("BAD_MATLAB", main["line"], 1, "%d inputs given, the function takes %d"
                        % (len(in_widths), len(main["inputs"])))

    def zn(e):   # zeros(1, n) / false(1, n) with a literal n
        a = e.get("args", [])
        if len(a) >= 2 and a[1].get("op") == "num" and Fraction(a[1]["text"]).denominator == 1:
            return int(Fraction(a[1]["text"]))
        return 1

    def ew(e, env):
        op = e.get("op")
        if op == "var":
            return env.get(e["name"], 1)
        if op in ("+", "-", "*", ".*"):
            return max(ew(x, env) for x in e["args"])
        if op == "call" and (e["name"] == "zeros" or e["name"] == "false" and e.get("args")):
            return zn(e)
        if op == "call" and (e["name"] == "fi" or e["name"] in INT_CLASSES) and e.get("args") and \
                e["args"][0].get("op") == "call" and e["args"][0].get("name") == "zeros":
            return zn(e["args"][0])
        if op == "concat":
            return sum(ew(x, env) for x in e["args"])
        if op == "index" and e.get("args") and e["args"][0].get("op") == "range":
            r = e["args"][0]["args"]
            if all(x.get("op") == "num" for x in r):
                return int(Fraction(r[1]["text"])) - int(Fraction(r[0]["text"])) + 1
        return 1

    def block(stmts, env):
        env = dict(env)
        for s_ in stmts:
            if s_["op"] == "assign" and not s_["whole"] and s_.get("index") is None:
                env[s_["target"]] = ew(s_["expr"], env)
            elif s_["op"] == "if":
                outs = [block(b, env) for _, b in s_["arms"]] + [block(s_["else"] or [], env)]
                for o in outs:
                    for k, v in o.items():
                        env[k] = max(env.get(k, 1), v)
            elif s_["op"] == "for":
                env = block(s_["body"], block(s_["body"], env))
        return env
    env = block(main["body"], dict(zip(main["inputs"], in_widths)))
    return [env.get(o, 1) for o in main["outputs"]]

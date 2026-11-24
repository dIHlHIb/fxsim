"""MATLAB Function v1 subset compiled into engine blocks (the C core, fxcodegen and fxverilog then run it), from the
type-annotated syntax tree of engine/fxmlsem.py -- the verified semantics, which stays the reference: every block
below computes exactly what the interpreter computes.

  + - * .* (fixed point)      Sum / Product with the full-precision result type (exact)
  integer + - * (one class)    Sum / Product in the class, saturating or wrapping (exact result, then the class)
  integer op double constant   Sum / Product, Round to the class (the constant exact in binary, else not lowered)
  fi op double constant        the constant as a Constant of its quantised type (fxmlsem _ctc)
  comparisons                  both sides converted exactly to a common type, Relational Operator
  && || & | ~                  Logical Operator
  if / elseif / else           both sides computed, a Switch per variable that differs
  for (constant bounds)        unrolled
  persistent                   a Unit Delay of the value at the end of the step; isempty(s) = NOT s's own
                               'assigned yet' flag (a register, set on every path that assigns s)
  fi() / cast / int8() ... / y(:) =   Data Type Conversion with the rounding and overflow of the semantics
  - x, abs, min, max           Unary Minus / Abs (overflow by the fimath), Min/Max on the exact union type
  subfunctions                 inlined
  bit functions                Bit Shift / Bit Slice / Bit Concat / Bitwise Operator (bitfun)
  vectors                      element by element (+ - .* and vector * scalar, x(k), branches, outputs per element)
  an integer op an inexact double constant, a comparison with one: dyadic stand-ins with the same results
Anything else (a vector persistent, a constant needing more than 53 bits, ...) raises NotLowerable: the block keeps
the interpreter (Python path)."""
import math
from fractions import Fraction

import fxmlsem as M

RM = {"Nearest": "Nearest", "Floor": "Floor", "Ceiling": "Ceiling", "Zero": "Zero", "Round": "Round",
      "Convergent": "Convergent"}


class NotLowerable(Exception):
    pass


_SET = "\x00set:"   # environment key of a persistent variable's "assigned yet" flag (not a MATLAB name)


BITFUNS = ("bitsll", "bitsrl", "bitsra", "bitshift", "bitsliceget", "bitget", "bitconcat", "bitset", "bitand", "bitor",
           "bitxor")


def eng(t):
    """fxmlsem type -> engine type ((s, wl, fl) or 'bool')."""
    import fxgraph_lib as G
    if t[0] == "bool":
        return G.BOOL
    if t[0] == "int":
        s, wl = M.INT_CLASSES[t[1]]
        return (s, wl, 0)
    if t[0] == "fi":
        return (t[1], t[2], t[3])
    raise NotLowerable("type %r" % (t,))


def fx(t):
    return "fixdt(%d,%d,%d)" % t


class Gen:
    def __init__(self, prog, prefix):
        self.prog = prog
        self.x = prefix
        self.k = 0
        self.blocks, self.lines = {}, []
        self.regs = {}       # persistent (function, name) -> (UnitDelay block, type)
        self.setregs = {}    # persistent (function, name) -> the register of its "assigned yet" flag
        self.first_sig = None

    def name(self, tag):
        self.k += 1
        return "%s%d_%s" % (self.x, self.k, tag)

    def blk(self, tag, b, *ins):
        n = self.name(tag)
        self.blocks[n] = b
        for k, s in enumerate(ins):
            self.wire(s, n, k + 1)
        return (n, 1)

    def wire(self, s, d, port=1):
        self.lines.append(dict({"src": s[0], "dst": d, "dport": port}, **({"sport": s[1]} if s[1] != 1 else {})))

    # -- constants and conversions
    def const(self, v, t):
        """A Constant of engine type t holding exactly v."""
        import fxgraph_lib as G
        if t == G.BOOL:
            z = self.const(0, (0, 1, 0))
            return self.blk("b", {"type": "RelationalOperator", "Operator": "==" if v else "~="}, z, z)
        s, wl, fl = t
        if wl > 53:
            raise NotLowerable("a constant of %d bits" % wl)
        q = Fraction(v) * (Fraction(2) ** fl)
        if q.denominator != 1:
            raise NotLowerable("constant not exact")
        return self.blk("k", {"type": "Constant", "Value": repr(float(Fraction(v))), "OutDataTypeStr": fx(t)})

    def dtc(self, s, t, rm="Floor", sat="off"):
        return self.blk("c", {"type": "DataTypeConversion", "OutDataTypeStr": fx(t), "RndMeth": rm,
                              "SaturateOnIntegerOverflow": sat}, s)

    def first(self):
        if self.first_sig is None:
            z = self.const(0, (0, 1, 0))
            d = self.blk("f", {"type": "UnitDelay", "InitialCondition": "1"}, z)
            self.first_sig = self.blk("f", {"type": "CompareToConstant", "relop": "~=", "const": "0",
                                            "OutDataTypeStr": "boolean"}, d)
        return self.first_sig

    # -- expressions: ("sig", (block, port), engine type) or ("const", Fraction)
    def expr(self, f, e, env):
        op = e["op"]
        # a value carrying its own fimath (e["_fa"]): the semantics only lets it reach + - * (full precision, any
        # constant exact -- else refused there), fi() (quantised by that fimath: e["_qfm"] / e["_fm"], used below)
        # and removefimath(); every other use is refused by fxmlsem, so it compiles like any other value
        if op == "num":
            if M.P.num_value(e["text"]) is None:
                raise NotLowerable("Inf")
            # the decimal value: the blocks below either use it where it gives the double's results for every
            # input (comparisons, fixed-point constants: a value of the grid never lies between a short decimal and
            # its double) or check that first (an integer op a constant: fxmlsem.int_dbl_same_as_exact against
            # MATLAB's double rule, recordings)
            return ("const", Fraction(e["text"]))
        if op == "var":
            if e["name"] in ("true", "false"):
                return ("sig", self.const(1 if e["name"] == "true" else 0, "bool"), "bool")
            v = env.get(e["name"])
            if v is None:
                raise NotLowerable("%s undefined" % e["name"])
            return v
        if op == "index":
            if "_k" not in e:
                raise NotLowerable("a slice or a run-time index (interpreter)")
            v = env.get(e["base"]["name"])
            if v is None or v[0] != "vec":
                raise NotLowerable("indexing %s" % e["base"]["name"])
            return v[1][e["_k"] - 1]
        args = e.get("args", [])
        t = e.get("_t")
        if op in ("+", "-", "*", ".*") and t[0] == "vec":
            # element by element: element j of each vector operand, a scalar operand as it
            # is, through the scalar rules below with the element type
            vals = [self.expr(f, x, env) for x in args]
            out = []
            for j in range(t[1]):
                sub_env, sub_args = dict(env), []
                for i, (x, v) in enumerate(zip(args, vals)):
                    nm = "\x00e%d" % i
                    xt = x["_t"]
                    sub_env[nm] = v[1][j] if v[0] == "vec" else v
                    sub_args.append({"op": "var", "name": nm, "_t": xt[2] if xt[0] == "vec" else xt})
                se = {"op": op, "args": sub_args, "_t": t[2]}
                if "_ctc" in e:
                    se["_ctc"] = e["_ctc"]
                out.append(self.expr(f, se, sub_env))
            return ("vec", out)
        if op in ("+", "-", "*", ".*"):
            if t == ("dbl",):
                a, b = (self.expr(f, x, env) for x in args)
                if a[0] != "const" or b[0] != "const":
                    raise NotLowerable("double arithmetic")
                return ("const", a[1] + b[1] if op == "+" else a[1] - b[1] if op == "-" else a[1] * b[1])
            if t[0] not in ("fi", "int"):
                raise NotLowerable("result %r" % (t,))
            rt = eng(t)
            sides, side_ts = [], []
            for k, x in enumerate(args):
                v = self.expr(f, x, env)
                if v[0] == "const":
                    if t[0] == "int":
                        c = v[1]
                        if not M.int_dbl_same_as_exact(op if op != ".*" else "*", t, c, k == 1):
                            # MATLAB rounds the double result; the blocks below compute the exact
                            # one -- the interpreter keeps these (int16 * 0.35, int32 * 0.3, ...)
                            raise NotLowerable("an integer op a double constant whose double rounding differs")
                        if c.denominator & (c.denominator - 1):
                            c = self.int_const_stand_in(op, c, args, env, f)
                        fl = max(0, c.denominator.bit_length() - 1)
                        mag = abs(c.numerator) * (1 << fl) // c.denominator
                        ct = (1, max(2, mag.bit_length() + 2), fl)
                        sides.append(self.const(c, ct))
                        side_ts.append(ct)
                    else:
                        cnode, ctt = e["_ctc"]
                        cq = M.real(ctt, M.quant(v[1], ctt, self.prog.default_fm))
                        sides.append(self.const(cq, eng(ctt)))
                        side_ts.append(eng(ctt))
                else:
                    sides.append(v[1])
                    side_ts.append(v[2])
            const_int = t[0] == "int" and any(self.expr(f, x, env)[0] == "const" for x in args)
            sat = "off"
            if t[0] == "int":
                sat = "on" if self.prog.int_sat else "off"
            rm = "Round" if const_int else "Floor"
            if op in ("*", ".*"):
                b = {"type": "Product", "Inputs": "**", "OutDataTypeStr": fx(rt), "RndMeth": rm,
                     "SaturateOnIntegerOverflow": sat}
            else:
                b = {"type": "Sum", "Inputs": "+" + ("+" if op == "+" else "-"), "OutDataTypeStr": fx(rt),
                     "RndMeth": rm, "SaturateOnIntegerOverflow": sat}
                if t[0] == "fi" and op == "-" and not rt[0]:
                    # unsigned - unsigned (default fimath): the exact difference in a signed accumulator, then the
                    # unsigned result type saturating -- a negative difference gives 0 (fxmlsem _arith_val)
                    fl = max(st[2] for st in side_ts)
                    ib = max(st[1] - st[2] for st in side_ts)
                    b["AccumDataTypeStr"] = fx((1, ib + 1 + fl, fl))
                    b["SaturateOnIntegerOverflow"] = "on"
                elif t[0] == "fi":
                    b["AccumDataTypeStr"] = fx(rt)
                elif all(st != "bool" for st in side_ts):
                    # the exact sum, then the class's saturation / wrap (fxmlsem): left to the inherited accumulator
                    # a uint32 minus a negative constant came out wrong (a - -8 = 0 at a = 0)
                    fl = max(st[2] for st in side_ts)
                    ib = max(st[1] - st[2] - st[0] for st in side_ts)
                    b["AccumDataTypeStr"] = fx((1, ib + 2 + fl, fl))
            return ("sig", self.blk("a", b, *sides), rt)
        if op == "neg":
            v = self.expr(f, args[0], env)
            if v[0] == "const":
                return ("const", -v[1])
            st = args[0]["_t"]
            sat = (self.prog.default_fm[1] == "Saturate") if st[0] == "fi" else self.prog.int_sat
            return ("sig", self.blk("n", {"type": "UnaryMinus", "SaturateOnIntegerOverflow": "on" if sat else "off"},
                                    v[1]), v[2])
        if op in ("==", "~=", "<", "<=", ">", ">="):
            vs = [self.expr(f, x, env) for x in args]
            sts = [x["_t"] for x in args]
            if any(st[0] == "bool" for st in sts):
                raise NotLowerable("boolean comparison")
            sig = [v for v in vs if v[0] == "sig"]
            if len(sig) == 1:
                # a constant not exact in binary against a value on the grid 2^-fl: the midpoint of the two grid
                # values around it equals no grid value and lies on the same side of each, so every comparison
                # (== false, ~= true, < <= > >=) gives the same truth with it (exact comparison, fxmlsem)
                g = Fraction(2) ** sig[0][2][2] if sig[0][2] != "bool" else None
                vs = [("const", (math.floor(v[1] * g) + Fraction(1, 2)) / g)
                      if v[0] == "const" and g is not None and v[1].denominator & (v[1].denominator - 1) else v
                      for v in vs]
            u = self.union([eng(st) if st != ("dbl",) else None for st in sts], vs)
            sides = []
            for v in vs:
                if v[0] == "const":
                    sides.append(self.const(v[1], u))
                elif v[2] != u:
                    sides.append(self.dtc(v[1], u))   # exact: u holds both
                else:
                    sides.append(v[1])
            return ("sig", self.blk("r", {"type": "RelationalOperator", "Operator": op}, *sides), "bool")
        if op in ("&&", "||", "&", "|"):
            a, b = (self.expr(f, x, env)[1] for x in args)
            return ("sig", self.blk("l", {"type": "LogicalOperator", "Operator": "AND" if op in ("&&", "&") else "OR",
                                          "Inputs": "2"}, a, b), "bool")
        if op == "not":
            a = self.expr(f, args[0], env)[1]
            return ("sig", self.blk("l", {"type": "LogicalOperator", "Operator": "NOT", "Inputs": "1"}, a), "bool")
        if op == "call":
            return self.call(f, e, env)
        raise NotLowerable(op)

    def union(self, ts, vs):
        """The common exact type of the operands of a comparison (constants must be exact in it)."""
        known = [t for t in ts if t is not None]
        if not known:
            raise NotLowerable("comparison of constants")
        fl = max(t[2] for t in known)
        for v in vs:
            if v[0] == "const":
                c = v[1]
                if c.denominator & (c.denominator - 1):
                    raise NotLowerable("comparison with an inexact constant")
                fl = max(fl, c.denominator.bit_length() - 1)
        ib = max(t[1] - t[2] - t[0] for t in known)
        for v in vs:
            if v[0] == "const":
                ib = max(ib, int(abs(v[1])).bit_length() + 1)
        return (1, 1 + ib + 1 + fl, fl)

    def int_const_stand_in(self, op, c, args, env, f):
        """An integer op a double constant c = p/q that is not exact in binary: the semantics rounds the exact result
        half away from zero (fxmlsem _arith_val). A dyadic stand-in that gives the same rounding for every input:
          + / -: no exact result is a tie (that needs c - 1/2 integer, a dyadic c), so round(a +- c) = a +- K with K
                 the nearest integer to c;
          *:     every exact a*p/q that is not a tie lies at least 1/(2q) from each half-integer. c' = c moved away
                 from zero by delta, 0 < delta < 2^-F with |a| * 2^-F <= 1/(2q) for the input's largest |a|, keeps
                 each non-tie on its side and pushes each tie away from zero -- exactly the half-away rounding."""
        if op in ("+", "-"):
            return Fraction(math.floor(c + Fraction(1, 2)))
        ints = [x["_t"] for x in args if x.get("_t", ("",))[0] == "int"]
        if len(ints) != 1:
            raise NotLowerable("an integer op an inexact double constant")
        s_, wl = M.INT_CLASSES[ints[0][1]]
        amax = (1 << (wl - 1)) if s_ else (1 << wl) - 1
        F = (2 * c.denominator * amax - 1).bit_length()   # 2^F >= 2 q amax
        n = math.ceil(abs(c) * (1 << F))
        if n.bit_length() + 2 > 53:
            raise NotLowerable("an integer times a double constant needing %d bits" % (n.bit_length() + 2))
        return Fraction(n if c > 0 else -n, 1 << F)

    def call(self, f, e, env):
        nm = e["name"]
        args = e["args"]
        t = e.get("_t")
        P = self.prog
        if nm in P.funcs:
            g = P.funcs[e.get("_spec", nm)]
            sub = {a: self.expr(f, x, env) for a, x in zip(g["inputs"], args)}
            sub = self.stmts(g, g["body"], sub)
            return sub[g["outputs"][0]]
        if nm == "isempty":
            a = args[0] if args else {}
            flag = env.get(_SET + a.get("name", "")) if a.get("op") == "var" else None
            if flag is None or flag[0] != "sig":
                raise NotLowerable("isempty of something other than a persistent variable")
            return ("sig", self.blk("ie", {"type": "LogicalOperator", "Operator": "NOT", "Inputs": "1"}, flag[1]), "bool")
        if nm == "removefimath":
            return self.expr(f, args[0], env)
        if nm == "cast" and "_cval" in e:
            # a compile-time double quantised by the block fimath
            rt = eng(t)
            return ("sig", self.const(M.real(t, M.quant(e["_cval"], t, P.default_fm)), rt), rt)
        if nm in ("fi", "cast") or nm in M.INT_CLASSES:
            src = args[0]
            v = self.expr(f, src, env)
            rt = eng(t)
            if nm == "fi":
                fm = (e.get("_fm") or P.default_fm) if e.get("_fm_explicit") else (e.get("_qfm") or P.default_fm)
                rm, sat = RM[fm[0]], "on" if fm[1] == "Saturate" else "off"
                if v[0] == "const":   # the constant as the double MATLAB holds
                    return ("sig", self.const(M.real(t, M.quant(Fraction(float(v[1])), t, fm)), rt), rt)
            elif nm == "cast":
                # by the block fimath, as the interpreter (it was Nearest / Saturate here only)
                rm, sat = RM[P.default_fm[0]], "on" if P.default_fm[1] == "Saturate" else "off"
            else:
                # int8() .. uint32() wrap when SaturateOnIntegerOverflow is off
                rm, sat = "Round", "on" if P.int_sat else "off"
                if v[0] == "const":
                    s, wl = M.INT_CLASSES[nm]
                    n_ = M._ovf(M._round(Fraction(float(v[1])), "Round"), s, wl, "Saturate" if P.int_sat else "Wrap")
                    return ("sig", self.const(n_, rt), rt)
            return ("sig", self.dtc(v[1], rt, rm, sat), rt)
        if nm in ("floor", "ceil"):
            # the integer type has room for every result (fxmlsem): no overflow
            v = self.expr(f, args[0], env)
            rt = eng(t)
            if v[0] == "const":
                raise NotLowerable("floor / ceil of a constant")
            return ("sig", self.dtc(v[1], rt, "Floor" if nm == "floor" else "Ceiling", "off"), rt)
        if nm == "logical":
            v = self.expr(f, args[0], env)
            if v[2] == "bool":
                return v
            return ("sig", self.blk("z", {"type": "CompareToConstant", "relop": "~=", "const": "0",
                                          "OutDataTypeStr": "boolean"}, v[1]), "bool")
        if nm in ("true", "false"):
            if args:
                raise NotLowerable("false(1, n): a vector (interpreter)")
            return ("sig", self.const(1 if nm == "true" else 0, "bool"), "bool")
        if nm == "abs":
            v = self.expr(f, args[0], env)
            st = args[0]["_t"]
            sat = (P.default_fm[1] == "Saturate") if st[0] == "fi" else P.int_sat
            return ("sig", self.blk("abs", {"type": "Abs", "SaturateOnIntegerOverflow": "on" if sat else "off"},
                                    v[1]), v[2])
        if nm in ("min", "max"):
            vs = [self.expr(f, x, env) for x in args]
            if any(v[0] == "const" for v in vs):
                raise NotLowerable("min / max with a constant")
            rt = eng(t)
            sides = [v[1] if v[2] == rt else self.dtc(v[1], rt) for v in vs]   # exact: rt is the union
            return ("sig", self.blk("mm", {"type": "MinMax", "Function": nm, "Inputs": "2", "OutDataTypeStr": fx(rt),
                                           "RndMeth": "Floor", "SaturateOnIntegerOverflow": "off"}, *sides), rt)
        if nm in BITFUNS:
            return self.bitfun(f, e, env)
        raise NotLowerable("call of %s" % nm)

    def bitfun(self, f, e, env):
        """The bit functions (rules in engine/fxmlsem.py) as the engine's bit blocks, each with
        the interpreter's rule on the stored integer's bit pattern: bitsll / bitsrl / bitsra and bitshift (right:
        arithmetic; left: wraps, or saturates under a Saturate fimath -- a Gain by 2^n) = Bit Shift; bitsliceget /
        bitget = Bit Slice; bitconcat = Bit Concat; bitand / bitor / bitxor = Bitwise Operator of two inputs; bitset
        = Bitwise Operator OR with a stored-integer mask."""
        nm, args = e["name"], e["args"]
        x = self.expr(f, args[0], env)
        if x[0] != "sig" or x[2] == "bool":
            raise NotLowerable("%s of a constant or boolean" % nm)
        xt = x[2]
        wl = xt[1]
        if nm in ("bitsll", "bitsrl", "bitsra", "bitshift"):
            n = e["_n"]
            if nm == "bitshift":
                if n < 0:
                    nm, n = "bitsra", -n
                elif n == 0:
                    return x
                elif args[0]["_t"][0] == "fi" and self.prog.default_fm[1] == "Saturate":
                    if n > 52:
                        raise NotLowerable("a saturating bitshift by %d" % n)
                    g = self.blk("bs", {"type": "Gain", "Gain": str(1 << n), "ParamDataTypeStr": "fixdt(0,%d,0)" % (n + 1),
                                        "OutDataTypeStr": fx(xt), "RndMeth": "Floor",
                                        "SaturateOnIntegerOverflow": "on"}, x[1])
                    return ("sig", g, xt)
                elif args[0]["_t"][0] == "fi":
                    nm = "bitsll"
                else:
                    raise NotLowerable("bitshift left of an integer class")
            mode = {"bitsll": "Shift Left Logical", "bitsrl": "Shift Right Logical",
                    "bitsra": "Shift Right Arithmetic"}[nm]
            # a shift by n >= the word length equals one by the word length (logical: 0, arithmetic right: the
            # sign; fxmlsem), and Bit Shift takes N up to the word length
            return ("sig", self.blk("sh", {"type": "BitShift", "mode": mode, "N": str(min(n, wl))}, x[1]), xt)
        rt = eng(e["_t"])
        if nm in ("bitsliceget", "bitget"):
            hi, lo = e["_r"] if nm == "bitsliceget" else (e["_n"], e["_n"])
            return ("sig", self.blk("sl", {"type": "BitSlice", "lidx": str(hi - 1), "ridx": str(lo - 1)}, x[1]), rt)
        if nm == "bitconcat":
            vs = [x] + [self.expr(f, a, env) for a in args[1:]]
            if any(v[0] != "sig" or v[2] == "bool" for v in vs):
                raise NotLowerable("bitconcat of a constant or boolean")
            return ("sig", self.blk("cat", {"type": "BitConcat", "numInputs": str(len(vs))}, *[v[1] for v in vs]), rt)
        if nm == "bitset":
            n = e["_n"]
            if n > 53:
                raise NotLowerable("bitset of bit %d" % n)
            return ("sig", self.blk("bset", {"type": "BitwiseOperator", "logicop": "OR", "UseBitMask": "on",
                                             "NumInputPorts": "1", "BitMask": str(1 << (n - 1)),
                                             "BitMaskRealWorld": "Stored Integer"}, x[1]), xt)
        y = self.expr(f, args[1], env)
        if y[0] != "sig" or y[2] != xt:
            raise NotLowerable("%s of operands of different types" % nm)
        return ("sig", self.blk("bw", {"type": "BitwiseOperator", "logicop": nm[3:].upper(), "UseBitMask": "off",
                                       "NumInputPorts": "2"}, x[1], y[1]), xt)

    def merge(self, a, c, b):
        """The value of a branch: a where the condition c holds, else b."""
        if a == b:
            return a
        if a[0] == "const" or b[0] == "const":
            raise NotLowerable("a constant assigned in a branch")
        if a[2] != b[2]:
            raise NotLowerable("branches of different types")
        if a[2] == "bool":
            # a boolean (a persistent's assigned flag, a logical variable): (c AND a) OR (NOT c AND b) -- the
            # engine's Switch takes no boolean data inputs
            nc = self.blk("mn", {"type": "LogicalOperator", "Operator": "NOT", "Inputs": "1"}, c)
            ta = self.blk("ma", {"type": "LogicalOperator", "Operator": "AND", "Inputs": "2"}, c, a[1])
            tb = self.blk("mb", {"type": "LogicalOperator", "Operator": "AND", "Inputs": "2"}, nc, b[1])
            return ("sig", self.blk("mo", {"type": "LogicalOperator", "Operator": "OR", "Inputs": "2"}, ta, tb), "bool")
        # the type both branches carry, written out (an inherited Switch type inside the loop of a
        # persistent register had nothing to start the type resolution from)
        return ("sig", self.blk("sw", {"type": "Switch", "Criteria": "u2 ~= 0",
                                       "OutDataTypeStr": "boolean" if a[2] == "bool" else fx(a[2])},
                                a[1], c, b[1]), a[2])

    # -- statements
    def stmts(self, f, stmts, env):
        env = dict(env)
        P = self.prog
        for s in stmts:
            op = s["op"]
            if op == "persistent":
                if f is not P.main:
                    raise NotLowerable("persistent in a subfunction")
                for nm in s["names"]:
                    key = (f["name"], nm)
                    if key not in self.regs:
                        t = P.types.get(key)
                        if t is None:
                            raise NotLowerable("persistent %s without a type" % nm)
                        r = self.name("p_" + nm)
                        self.blocks[r] = {"type": "UnitDelay", "InitialCondition": "0"}
                        self.regs[key] = (r, eng(t))
                        # has nm been assigned yet (isempty(nm) is its negation), per variable as the interpreter
                        # keeps it (fxmlsem): false until a step assigns nm, then true for good (one
                        # global "first step" signal answered isempty wrongly after a conditional or a second test)
                        fr = self.name("set_" + nm)
                        self.blocks[fr] = {"type": "UnitDelay", "InitialCondition": "0"}
                        self.setregs[key] = fr
                    r, et = self.regs[key]
                    if _SET + nm not in env:
                        # the register's value only where the step first declares nm; a declaration met again (a
                        # loop body, unrolled) keeps the value and flag assigned earlier in this step (
                        # each unrolled iteration re-read the start-of-step value)
                        env[nm] = ("sig", (r, 1), et)
                        env[_SET + nm] = ("sig", (self.setregs[key], 1), "bool")
            elif op == "assign" and (s.get("index") is not None or s.get("_ix") is not None):
                raise NotLowerable("an indexed assignment (interpreter)")
            elif op == "assign":
                nm = s["target"]
                v = self.expr(f, s["expr"], env)
                if s["whole"]:
                    t = s["_t"]
                    rt = eng(t)
                    if v[0] == "const":
                        if t[0] == "fi":
                            v = ("sig", self.const(M.real(t, M.quant(v[1], t, P.default_fm)), rt), rt)
                        else:
                            raise NotLowerable("integer (:) = constant")
                    elif t[0] == "fi":
                        fm = P.default_fm
                        v = ("sig", self.dtc(v[1], rt, RM[fm[0]], "on" if fm[1] == "Saturate" else "off"), rt)
                    elif t[0] == "int":
                        v = ("sig", self.dtc(v[1], rt, "Round", "on" if P.int_sat else "off"), rt)
                    else:
                        raise NotLowerable("(:) = of a boolean")
                env[nm] = v
                if (f["name"], nm) in self.regs:
                    env[_SET + nm] = ("sig", self.const(1, "bool"), "bool")   # assigned on this path
            elif op == "if":
                arms = []
                for cond, body in s["arms"]:
                    c = self.expr(f, cond, env)
                    arms.append((c[1], self.stmts(f, body, env)))
                merged = self.stmts(f, s["else"], env) if s["else"] is not None else dict(env)
                for c, benv in reversed(arms):
                    out = {}
                    for k in set(benv) & set(merged):
                        a, b = benv[k], merged[k]
                        if a[0] == "vec" or b[0] == "vec":
                            if a[0] != b[0] or len(a[1]) != len(b[1]):
                                raise NotLowerable("a vector and a scalar merged in a branch")
                            out[k] = ("vec", [self.merge(x, c, y) for x, y in zip(a[1], b[1])])
                        else:
                            out[k] = self.merge(a, c, b)
                    merged = out
                env = merged
            elif op == "for":
                k = s["lo"]
                while (k <= s["hi"]) if s["step"] > 0 else (k >= s["hi"]):
                    env[s["var"]] = ("const", Fraction(k))
                    env = self.stmts(f, s["body"], env)
                    k += s["step"]
                    if self.k > 20000:
                        raise NotLowerable("too large unrolled")
            elif op == "nop":
                pass     # coder.extrinsic / coder.unroll / an extrinsic warning(...): no value effect (fxmlfn)
            else:
                raise NotLowerable(op)
        return env


def _plain_persistent(main):
    """persistent is compiled only in its plain form: at the top level of the function (not inside an if or a loop),
    each variable declared once, and not read or assigned before its declaration. Anything else keeps the
    interpreter until MATLAB's behaviour for those placements is recorded (the recordings: a
    declaration inside an if lost its value at the merge -- a silent wrong value; recordings 's loop case is one of
    these too). recordings: an unrelated statement before the declaration (b = a + 1; y = int8(0)) and a second
    persistent after the first one's initialisation are plain."""
    body = main["body"]
    declared, used = set(), set()

    def names_in(node, out):
        if isinstance(node, dict):
            if node.get("op") == "var":
                out.add(node.get("name"))
            if node.get("op") == "assign":
                out.add(node.get("target"))
            if node.get("op") == "for":
                out.add(node.get("var"))
            for v in node.values():
                names_in(v, out)
        elif isinstance(node, list):
            for v in node:
                names_in(v, out)
        return out

    def nested(stmts):
        for st in stmts:
            if st["op"] == "persistent":
                raise NotLowerable("persistent declared inside an if or a loop")
            if st["op"] == "if":
                for _c, b in st["arms"]:
                    nested(b)
                if st.get("else") is not None:
                    nested(st["else"])
            elif st["op"] == "for":
                nested(st["body"])

    for st in body:
        if st["op"] == "persistent":
            for nm in st["names"]:
                if nm in declared:
                    raise NotLowerable("a persistent variable declared twice")
                if nm in used:
                    raise NotLowerable("persistent %s declared after a statement that uses it" % nm)
                declared.add(nm)
        else:
            nested([st])
            names_in(st, used)

def lower(prog, n, in_srcs, internal):
    """(blocks, lines, {output port: (block, port)}) computing MATLAB Function n from its input signals in_srcs
    (one (block, port) per argument); the outputs are new blocks named by the caller. Raises NotLowerable."""
    _plain_persistent(prog.main)
    g = Gen(prog, internal + n + "/m/")
    env = {}
    for nm, s, t in zip(prog.main["inputs"], in_srcs, prog.in_types):
        if t[0] == "vec":   # s: one (block, port) per element
            if not isinstance(s, list) or len(s) != t[1]:
                raise NotLowerable("vector input %s" % nm)
            env[nm] = ("vec", [("sig", x, eng(t[2])) for x in s])
        else:
            env[nm] = ("sig", s[0] if isinstance(s, list) else s, eng(t))
    env = g.stmts(prog.main, prog.main["body"], env)
    for (fn, nm), fr in g.setregs.items():
        fl = env.get(_SET + nm)
        if fl is None or fl[0] != "sig":
            raise NotLowerable("persistent %s: its assigned flag is lost" % nm)
        if fl[1] == (fr, 1):
            # never assigned in a step (e.g. only inside an empty loop): the flag would be a register fed by itself,
            # a type loop with nothing to start from (TYPE_UNRESOLVED) -- the interpreter keeps the block
            raise NotLowerable("persistent %s is never assigned" % nm)
        g.wire(fl[1], fr, 1)
    for (fn, nm), (r, et) in g.regs.items():
        v = env.get(nm)
        if v is None or v[0] != "sig":
            raise NotLowerable("persistent %s has no value at the end of the step" % nm)
        if v[2] != et:
            raise NotLowerable("persistent %s changes type" % nm)
        # into the register through an identity cast of its own type: the loop register -> ... -> register then
        # holds a block whose output type is given (a loop of inherited types was refused)
        src = v[1] if et == "bool" else g.dtc(v[1], et)
        g.wire(src, r, 1)
    outs = {}
    port = 0
    for k, o in enumerate(prog.main["outputs"]):
        v = env.get(o)
        ot = prog.out_types[k]
        els = v[1] if v is not None and v[0] == "vec" else [v]
        if v is None or (ot[0] == "vec") != (v[0] == "vec") or (ot[0] == "vec" and len(els) != ot[1]):
            raise NotLowerable("output %s" % o)
        for x in els:   # a vector output: one port per element, in order (engine/fxhdl.py _mlfn)
            if x is None or x[0] != "sig":
                raise NotLowerable("output %s" % o)
            port += 1
            outs[port] = x
    return g, outs

"""One parser for every numeric block parameter (one rule, every path).

A parameter is a MATLAB numeric literal ("0.5", "-3e-2", "Inf"), or a bracketed list / matrix of them
("[0.5 -0.25]", "[1 2; 3 4]", "[0.5]"), or a scalar with a transpose ("0.5'"). Those are accepted and given back in a
canonical form the engine's float() calls read ("[0.5]" -> "0.5"). Anything else is either
  * a MATLAB expression -- operators, function calls, workspace variables ("2^-8", "pi/4", "h", "1+1", "--1") --
    valid for Simulink, not evaluated by the engine: UNSUPPORTED; or
  * not MATLAB at all ("1_000", "1.5.5", "abc def", non-ASCII digits): BAD_PARAM.
"""
import re

_NUM = r"[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?|[+-]?(?:Inf|inf|NaN|nan)"
_NUM_RE = re.compile(r"^(?:%s)$" % _NUM)
_TOKEN = re.compile(r"\s+|(?P<num>(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?)|(?P<id>[A-Za-z][A-Za-z0-9_]*)"
                    r"|(?P<op>\.\^|\.\*|\./|[-+*/^'(),;\[\]:])")


class NumError(Exception):
    def __init__(self, code, message):
        Exception.__init__(self, message)
        self.code, self.message = code, message


def _balanced(t):
    depth = []
    for ch in t:
        if ch in "([":
            depth.append(ch)
        elif ch in ")]":
            if not depth or depth.pop() != ("(" if ch == ")" else "["):
                return False
    return not depth


def _classify_other(t):
    """UNSUPPORTED for text that tokenises as a MATLAB expression with an operator or a name, else BAD_PARAM
    (unbalanced brackets are a MATLAB syntax error: BAD_PARAM)."""
    if "%" in t:
        # MATLAB reads '%' to the end of the line as a comment ('8 %{' is 8) -- not taken here
        return NumError("UNSUPPORTED", "a MATLAB comment ('%%') in %r is not read here; write the value without it"
                        % t[:60])
    if '"' in t or re.search(r"(^|[^\w\])\.'])'", t):
        # a string literal ('hex2dec("FF")' is 255 in MATLAB): an expression this engine does not evaluate
        return NumError("UNSUPPORTED", "MATLAB expression %r (with a string literal) is not evaluated; give a numeric "
                        "literal" % t[:60])
    if not _balanced(t):
        return NumError("BAD_PARAM", "unbalanced brackets: %r" % t[:60])
    if re.search(r"[~\\]|==|[;,]\s*$", t):
        # '0.3;' / '1,' (a trailing separator), '~0', '1==1', '3\6' are MATLAB Simulink evaluates --
        # not taken here, not a malformed request
        return NumError("UNSUPPORTED", "MATLAB expression %r is not evaluated; give a numeric literal" % t[:60])
    if re.search(r"\[[^\]]*\[", t):   # nested brackets ('[[0.5]]') are a MATLAB concatenation: an expression
        return NumError("UNSUPPORTED", "MATLAB expression %r is not evaluated; give a numeric literal" % t[:60])
    pos, seen_op_or_id, nums = 0, False, 0
    while pos < len(t):
        m = _TOKEN.match(t, pos)
        if not m or m.end() == pos:
            return NumError("BAD_PARAM", "not a number or a MATLAB expression: %r" % t[:60])
        if m.group("id") or (m.group("op") and m.group("op") not in "[],;"):
            seen_op_or_id = True   # list punctuation alone does not make an expression ('[1.5.5 0.5]': BAD_PARAM)
        if m.group("num"):
            nums += 1
        pos = m.end()
    if seen_op_or_id:
        return NumError("UNSUPPORTED", "MATLAB expression %r is not evaluated; give a numeric literal" % t[:60])
    return NumError("BAD_PARAM", "not a number: %r" % t[:60])


FUNCTIONS = ("nextpow2",)   # functions _arith_value evaluates (the .slx reader leaves their names in place)
_ATOK = re.compile(r"\s*(?:(?P<num>(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?)|(?P<lst>\[[^\[\]]*\])"
                   r"|(?P<fn>nextpow2)(?=\s*\()|(?P<op>\.\^|\.\*|\./|[-+*/^()]))")


def _arith_value(t):
    """Value of plain arithmetic on numeric literals, as MATLAB computes it in double: a float, or a matrix of floats
    (a tuple of row tuples) when a bracketed list of literals takes part. Operators + - * / and .* ./ .^ ^ with
    MATLAB's precedence (^ binds tighter than a unary sign, which binds tighter than * /, then + -; all left
    associative), parentheses. A list combines element-wise with a scalar or with a list of the same shape; a matrix
    product or division between two lists, ^ on a list, and a power whose exact result is not a double give None, as
    does anything else (names, functions other than nextpow2 of a scalar, ranges). recordings: '[1 2 3]*2+1' is [3 5 7], not [1 2 3]*(2+1)."""
    from fractions import Fraction
    toks, pos = [], 0
    t = t.strip()
    while pos < len(t):
        m = _ATOK.match(t, pos)
        if not m or m.end() == pos:
            return None
        if m.group("num"):
            toks.append(("n", float(m.group("num"))))
        elif m.group("fn"):
            toks.append(("f", m.group("fn")))
        elif m.group("lst"):
            rows = _elements(m.group("lst")[1:-1])
            if rows is None or not rows:
                return None
            if any(":" in e for r in rows for e in r):
                # a colon range inside the list ('[1:4]/8') is expanded when every element is exact in
                # binary64 (_range_values); one row only
                if len(rows) != 1:
                    return None
                flat = []
                for e in rows[0]:
                    vals = _range_values(e) if ":" in e else [e]
                    if vals is None:
                        return None
                    flat += vals
                rows = [flat]
            if any(len(r) != len(rows[0]) for r in rows):
                return None
            toks.append(("n", tuple(tuple(float(e) for e in r) for r in rows)))
        else:
            toks.append(("o", m.group("op")))
        pos = m.end()
    k = [0]

    def peek():
        return toks[k[0]] if k[0] < len(toks) else (None, None)

    def ismat(x):
        return isinstance(x, tuple)

    def lift(x, shape):
        return x if ismat(x) else tuple(tuple(x for _ in range(shape[1])) for _ in range(shape[0]))

    def elementwise(a, b, f):
        if not ismat(a) and not ismat(b):
            return f(a, b)
        sa = (len(a), len(a[0])) if ismat(a) else None
        sb = (len(b), len(b[0])) if ismat(b) else None
        if sa and sb and sa != sb:
            raise ValueError("shapes differ")
        shape = sa or sb
        a, b = lift(a, shape), lift(b, shape)
        return tuple(tuple(f(x, y) for x, y in zip(ra, rb)) for ra, rb in zip(a, b))

    def exact_pow(b, e):
        if e == int(e) and abs(e) > 2200 and abs(b) not in (0.0, 1.0):
            # a power far outside the double range (nested chains reach 3^(3^27)): not evaluated exactly
            raise ValueError("power outside the double range")
        exact = Fraction(b) ** int(e) if e == int(e) and (b != 0 or e >= 0) else None
        if exact is None or Fraction(float(exact)) != exact:
            raise ValueError("power not exact in double")
        return float(exact)

    def unary():
        kind, v = peek()
        if kind == "o" and v in "+-":
            k[0] += 1
            x = unary()
            return elementwise(x, -1.0, lambda p, q: p * q) if v == "-" else x
        return power()

    def power():
        # recordings (MATLAB R2026b, 29,066 chains): unsigned exponents fold left; a
        # signed exponent takes as its operand the rest of the chain up to -- not including -- the chain's final
        # exponent, and inside that span a signed exponent takes the rest of the span (2^-1^-1^3 = (2^-(1^-1))^3 =
        # 0.125; 2^-3^2 = (2^-3)^2). 29,052 / 29,055 chains equal; two signs on one exponent (2^--1^3) are not
        # modelled (refused)
        b = atom()
        items = []
        while peek() in (("o", "^"), ("o", ".^")):
            op = peek()[1]
            k[0] += 1
            kind, v = peek()
            sign, nsign = 1, 0
            while kind == "o" and v in "+-":
                k[0] += 1
                sign = -sign if v == "-" else sign
                nsign += 1
                kind, v = peek()
            if nsign > 1:
                raise ValueError("two signs on one exponent")
            items.append((op, nsign == 1, float(sign), atom()))

        def pw(x, op, e):
            if op == "^" and (ismat(x) or ismat(e)):
                raise ValueError("matrix power")
            return elementwise(x, e, exact_pow)

        def fold(x, its, end):
            i_ = 0
            while i_ < len(its):
                op, signed, sign, a = its[i_]
                if not signed or i_ >= end:
                    x = pw(x, op, elementwise(a, sign, lambda p, q: p * q))
                    i_ += 1
                    continue
                operand = fold(a, its[i_ + 1:end], end - i_ - 1) if end > i_ + 1 else a
                x = pw(x, op, elementwise(operand, sign, lambda p, q: p * q))
                i_ = end
            return x
        return fold(b, items, len(items) - 1)

    def atom():
        kind, v = peek()
        k[0] += 1
        if kind == "n":
            return v
        if kind == "f":
            # nextpow2(x): the smallest integer p with 2^p >= |x|, nextpow2(0) = 0 (22
            # values -- 0, negative, fractional, 1e-300 -> -996, 2^53 + 2 -> 54 -- all follow it; HDL Counter word
            # lengths are written nextpow2(N-1), recorded configurations). A list argument is not recorded
            # (None); Inf / NaN give a non-finite result, refused below
            if peek() != ("o", "("):
                raise ValueError
            k[0] += 1
            x = expr()
            if peek() != ("o", ")"):
                raise ValueError
            k[0] += 1
            if ismat(x) or x != x or x in (float("inf"), float("-inf")):
                raise ValueError("nextpow2 argument not recorded")
            a = abs(Fraction(x))
            if a == 0:
                return 0.0
            p = a.numerator.bit_length() - a.denominator.bit_length()
            while Fraction(2) ** p < a:
                p += 1
            while Fraction(2) ** (p - 1) >= a:
                p -= 1
            return float(p)
        if (kind, v) == ("o", "("):
            r = expr()
            if peek() != ("o", ")"):
                raise ValueError
            k[0] += 1
            return r
        raise ValueError

    def term():
        r = unary()
        while peek()[0] == "o" and peek()[1] in ("*", "/", ".*", "./"):
            op = peek()[1]
            k[0] += 1
            b = unary()
            if op == "*" and ismat(r) and ismat(b):
                raise ValueError("matrix product")
            if op == "/" and ismat(b):
                raise ValueError("matrix division")
            r = elementwise(r, b, (lambda p, q: p * q) if op in ("*", ".*") else (lambda p, q: p / q))
        return r

    def expr():
        r = term()
        while peek()[0] == "o" and peek()[1] in ("+", "-"):
            op = peek()[1]
            k[0] += 1
            b = term()
            r = elementwise(r, b, (lambda p, q: p + q) if op == "+" else (lambda p, q: p - q))
        return r
    try:
        v = expr()
    except (ValueError, ZeroDivisionError, OverflowError, IndexError):
        return None
    if k[0] != len(toks):
        return None
    flat = [x for r in v for x in r] if ismat(v) else [v]
    if any(x != x or x in (float("inf"), float("-inf")) for x in flat):
        return None
    return v


def _scalar_arith(t):
    """Value (a float, IEEE double as MATLAB computes it) of plain arithmetic on numeric literals (_arith_value); None
    for anything else, a list result included. Real models write 2^14, 1/3, -0.5*3 (coverage per block)."""
    v = _arith_value(t)
    return None if v is None or isinstance(v, tuple) else v


def _literal_arith(t):
    """Canonical text of plain arithmetic on numeric literals and bracketed lists of literals (_arith_value): a scalar
    as repr, a list as "[a b; c d]"; None otherwise."""
    v = _arith_value(t)
    if v is None:
        return None
    if not isinstance(v, tuple):
        return repr(v)
    return "[" + "; ".join(" ".join(repr(x) for x in r) for r in v) + "]"


def _elementwise_arith(body):
    """A bracket body whose elements are each plain arithmetic on numeric literals ('1/3 0.1; -0.7 2/3', '-5*2^-12 3'):
    the list of their MATLAB double values as "[a b; c d]", else None. Elements are separated by white space or
    commas as MATLAB reads them; an element that is a bare operator ('1 - 2', where MATLAB's white-space rule decides
    between one and two elements) or anything but literals and + - * / ^ () gives None (the text stays refused).
    every Gain Matrix written with such elements is equal once evaluated."""
    if "..." in body or "[" in body or "]" in body:
        return None
    rows = []
    for row in body.replace("\r\n", ";").replace("\r", ";").replace("\n", ";").split(";"):
        els = [e for e in re.split(r"[\s,]+", row.strip()) if e]
        if not els:
            continue
        vals = []
        for e in els:
            if not re.match(r"^[-+]?[0-9.(]", e) or re.search(r"[-+*/^]$", e) or ":" in e:
                return None
            v = _arith_value(e)
            if v is None or isinstance(v, tuple):
                return None
            vals.append(repr(float(v)))
        rows.append(vals)
    if not rows or any(len(r) != len(rows[0]) for r in rows):
        return None
    return "[" + "; ".join(" ".join(r) for r in rows) + "]"


_RANGE_RE = re.compile(r"^(?:%s)(?::(?:%s)){1,2}$" % (_NUM, _NUM))


def _elements(body):
    """Rows of literal texts of a bracket body, or None if an element is not a literal. A colon range "a:b" /
    "a:s:b" of literals counts as a literal element here (the readers expand it: ref/fxcore.py _vec, the .slx
    reader); it is kept as written."""
    if "..." in body:   # a MATLAB line continuation inside brackets: not read
        return None
    rows = []
    # a newline inside brackets starts a new row in MATLAB (it was read as a column space, so a 2 x 2
    # matrix became a row of 4 -- FIR coefficients became another filter)
    for row in body.replace("\r\n", ";").replace("\r", ";").replace("\n", ";").split(";"):
        els = [e for e in re.split(r"[\s,]+", row.replace(" :", ":").replace(": ", ":").strip()) if e]
        if not all(_NUM_RE.match(e) or _RANGE_RE.match(e) for e in els):
            return None
        if els:   # an empty row (a trailing ';') is ignored, as MATLAB does
            rows.append(els)
    return rows


# typed constants the engine reads itself (ref/fxcore.py _const_value / the Constant block; recordings:
# fi(value, s, wl, fl) and int8(...) .. uint32(...)): passed through as written
_TYPED = re.compile(r"^(?:fi\(\s*%s\s*,\s*[01]\s*,\s*[0-9]+\s*,\s*-?[0-9]+\s*\)|u?int(?:8|16|32)\(\s*%s\s*\))$"
                    % ("(?:%s)" % _NUM, "(?:%s)" % _NUM))


def _range_values(e):
    """Values of a colon range of literals when every one is exactly representable in binary (then MATLAB's double
    arithmetic gives the same values), else None."""
    from fractions import Fraction
    # MATLAB rounds each literal to binary64 before the colon
    # ('[9007199254740993:9007199254740994]' is 2^53 : 2^53 + 2, three elements); exact decimal Fractions gave two
    parts = [Fraction(float(x)) for x in e.split(":")]
    a, st, b = (parts[0], Fraction(1), parts[1]) if len(parts) == 2 else parts
    if st == 0 or (b - a) / st > 100000:
        return None
    q = (b - a) / st
    if q.denominator != 1 and abs(q - round(q)) < Fraction(1, 1 << 20):
        # an end a hair short of (or past) a step -- MATLAB's colon applies a floating tolerance there
        # that is not recorded; refused rather than guessed either way
        return None
    vals, k = [], 0
    while (st > 0 and a + k * st <= b) or (st < 0 and a + k * st >= b):
        v = a + k * st
        if v.denominator & (v.denominator - 1) or Fraction(float(v)) != v:
            # not exactly a binary64 value (here 2^53 + 1): MATLAB's element is a rounded double -- refused
            return None
        vals.append(v)
        k += 1
    return [repr(float(v)) if v.denominator != 1 else str(int(v)) for v in vals]


def port_count(text, signs=""):
    """A port-count parameter (int() / str.isdigit() read '0_3' and non-ASCII digits as 3): the
    canonical integer text, or -- when signs names the block's operator characters ('+-|' for Sum, '*/' for Product)
    -- the operator string as written. NumError otherwise (the same codes as every numeric parameter)."""
    t = str(text).strip() if text is not None else ""
    if signs and t and not set(t) - set(signs):
        return t
    if signs and (t.startswith("[") or (t[:1] and t[:1] in "+-")):   # recorded: "" in "+-" is True
        # a previous version: Simulink rejects Sum Inputs '[1]' and Product
        # Inputs '+2' at set_param (Simulink:Commands:BadSetParamValue); the canonical integer read them as counts.
        # The recorded forms are rejected, their unrecorded mirrors (Sum '+2', Product '[2]') are not answered
        recorded = (t.startswith("[") and "+" in signs) or (t[:1] in ("+", "-") and "*" in signs)
        return NumError("BAD_PARAM" if recorded else "UNSUPPORTED",
                        "Inputs %r: a bracketed or signed number of inputs %s; give the count as plain digits or the "
                        "operator signs" % (t[:40], "is rejected by Simulink (BadSetParamValue)" if recorded else
                                            "is not recorded"))
    return canonical(t, "int")


def dims(text):
    """Port dimensions as a signal width: one integer (-1: inherited), or [n] / [n 1] / [1 n] (a row or column
    vector); a matrix is UNSUPPORTED. Canonical integer text, or NumError."""
    c = canonical(text, "vector")
    if isinstance(c, NumError):
        return c
    els = c.strip("[]").split() if c.startswith("[") else [c]
    vals = []
    for e in els:
        v = canonical(e, "int")
        if isinstance(v, NumError):
            return v
        vals.append(int(v))
    if any(v == 0 or v < -1 for v in vals) or (len(vals) > 1 and -1 in vals):
        # '0' and '-2' were taken as a scalar signal
        return NumError("UNSUPPORTED", "port dimensions %r: a positive width (or -1, inherited) is required"
                        % str(text)[:60])
    if len(vals) == 1:
        return str(vals[0])
    if len(vals) == 2 and 1 in vals:
        return str(vals[0] * vals[1])
    return NumError("UNSUPPORTED", "port dimensions %r: only scalar and vector signals are supported" % str(text)[:60])


def _fi_with_expr(t):
    """fi(<plain arithmetic with sqrt / pi>, s, w, f) that ref/fxcore.py evaluates (for example, fi(1/sqrt(2),1,16,15))."""
    try:
        import fxcore
        return fxcore._parse_fi(t) is not None
    except (ImportError, ValueError):
        return False


def canonical(text, kind="any"):
    """The canonical numeric text of a parameter value, or NumError. kind (blockspec.NUMERIC_VALUES):
      'scalar'  one value;          'int'     one integral value (2.0 / 2e0 / 2. are 2);
      'typed'   one value, or a typed constant fi(...) / int8(...) .. uint32(...) the block reads itself;
      'vector'  one row or column (a column is written as a row, colon ranges of binary-exact literals expanded);
      'any'     a list or matrix as written (colon ranges kept for their readers);
      'typed_any' as 'any', or a typed constant (the Constant block's value: fi(...), recordings)."""
    if kind == "typed_any":   # the Constant block reads fi(...) itself, not int8(...)
        kind = "typed" if str(text).strip().startswith("fi(") else "any"
    if text is not None:
        # the native expression lane, MATLAB eval semantics. The
        # block-parameter path applies Simulink's own trimming first (fxgraph_lib param text rules, recordings)
        raw_ = str(text)
        body_ = raw_.rstrip(" \t\r\n\x0b\x0c")   # trailing white space ends the statement harmlessly ('3\x0b' is 3)
        depth_ = 0
        for ch in body_:
            if ch in "[({":
                depth_ += 1
            elif ch in "])}":
                depth_ = max(depth_ - 1, 0)
            elif ch in "\r\n" and depth_ == 0:
                # '\n8', '-\n8': MATLAB:m_unexpected_sep; '2\n+1': MATLAB:m_missing_variable_or_function
                return NumError("BAD_PARAM", "a line break before or inside the value %r ends the MATLAB statement; "
                                "MATLAB's eval rejects it" % raw_[:60])
            elif ch in "\x0b\x0c":
                return NumError("UNSUPPORTED", "a vertical tab / form feed before or inside the value %r is not taken "
                                "here; write the value without it" % raw_[:60])
        text = body_ if body_ != raw_ and not body_.strip("[]") == "" else text
    if text is not None and any(ord(ch) > 126 or (ord(ch) < 32 and ch not in "\t\n\r") for ch in str(text)):
        # a no-break space, a vertical tab ... -- MATLAB rejects them (InvalidWhitespace)
        return NumError("BAD_PARAM", "a character outside printable ASCII in %r: MATLAB rejects it" % str(text)[:60])
    t = str(text).strip() if text is not None else ""
    if not t:
        return NumError("BAD_PARAM", "a number is required")
    if re.search(r",\s*,", t):   # (a leading comma '[,1 2]' is valid MATLAB)
        # a doubled or leading comma inside a list ('[0.5,,0.25]', '[,,1.5]') is a MATLAB syntax
        # error (the empty element was dropped and the list answered)
        return NumError("BAD_PARAM", "an empty list element (a doubled comma) in %r is a MATLAB syntax "
                        "error" % t[:60])
    m_rs = re.match(r"^reshape\(\s*\[([^\[\]]*)\]\s*,\s*(?:(\d+)\s*,\s*(\d+)|\[\s*(\d+)[\s,]+(\d+)\s*\])\s*\)$", t)
    if m_rs and kind == "any":
        # reshape([v1 ... vk], m, n) of numeric literals: MATLAB fills the m x n matrix column by column
        # (2-D Lookup Table dialog tables written this way, every design compared)
        els = [e for e in re.split(r"[\s,]+", m_rs.group(1).strip()) if e]
        rows_, cols_ = (int(m_rs.group(2)), int(m_rs.group(3))) if m_rs.group(2) else (int(m_rs.group(4)),
                                                                                       int(m_rs.group(5)))
        vals_ = [canonical(e, "scalar") for e in els]
        if rows_ >= 1 and cols_ >= 1 and len(els) == rows_ * cols_ and \
                not any(isinstance(v, NumError) for v in vals_):
            return "[%s]" % "; ".join(" ".join(vals_[j * rows_ + i] for j in range(cols_)) for i in range(rows_))
        if len(els) != rows_ * cols_:
            return NumError("BAD_PARAM", "reshape of %d values into %d x %d: MATLAB rejects it (the number of elements "
                            "must not change)" % (len(els), rows_, cols_))
    if "..." in t:
        # MATLAB's line continuation inside a list ('[1.5 ...\n -2]') is run by Simulink -- a spelling
        # not taken here, not a malformed request
        return NumError("UNSUPPORTED", "a '...' line continuation in %r is not supported here; write the list on one "
                        "line" % t[:60])
    if kind == "int" and t in ("true", "false"):
        # a logical literal makes the parameter boolean,
        # which Shift Arithmetic rejects (InvFixptAndNonBoolBuiltinPortDType); not recorded as accepted anywhere
        return NumError("UNSUPPORTED", "a logical value %r for an integer parameter: Simulink makes it boolean, which "
                        "blocks such as Shift Arithmetic reject; write the number" % t)
    if kind == "int" and not _NUM_RE.match(t):
        # an integer count written as 0x2, int8(3), true, ... (MATLAB evaluates it to its value)
        try:
            import fxcore
            r = fxcore.matlab_int_expr(t)
        except ImportError:
            r = None
        if r is not None and r[1] == "logical":
            # '(true)' / '((true))' in an integer parameter -- Simulink rejects a boolean there, as
            # it does the bare 'true' (refused above by text)
            return NumError("UNSUPPORTED", "a logical value %r for an integer parameter: Simulink makes it boolean, "
                            "which these parameters reject; write the number" % t)
        if r is not None and r[0].denominator == 1:
            return str(int(r[0]))
    if _TYPED.match(t) or (kind in ("typed", "typed_any") and _fi_with_expr(t)):
        if kind in ("typed", "typed_any"):
            return t
        return NumError("UNSUPPORTED", "a typed constant %r is not supported for this parameter" % t[:60])
    scalar_t = t[:-1].strip() if t.endswith("'") else t   # a transpose changes nothing for a scalar
    out = None
    if _NUM_RE.match(scalar_t):
        out = scalar_t
    elif re.match(r"^\[\s*\]$", scalar_t):   # the empty list is a MATLAB value; the block decides whether it fits
        return "[]" if kind in ("vector", "any") else NumError("BAD_PARAM", "a value is required, got []")
    elif scalar_t.startswith("[") and scalar_t.endswith("]"):
        rows = _elements(scalar_t[1:-1])
        if rows is None:
            lit = _literal_arith(t)
            if lit is None and not t.endswith("'"):
                lit = _elementwise_arith(scalar_t[1:-1])
            return _classify_other(t) if lit is None else canonical(lit, kind)
        if any(len(r) != len(rows[0]) for r in rows) or not rows or not rows[0]:
            return NumError("BAD_PARAM", "rows of different lengths, or an empty list: %r" % t[:60])
        transposed = t.endswith("'")
        if any(":" in e for r in rows for e in r):
            if kind == "vector" and len(rows) == 1:
                vals = []
                for e in rows[0]:
                    if ":" in e:
                        v = _range_values(e)
                        if v is None:
                            return NumError("UNSUPPORTED", "colon range %r with steps not exact in binary is not "
                                            "evaluated; write the values out" % e)
                        vals += v
                    else:
                        vals.append(e)
                rows = [vals]
            elif kind in ("any",) and not transposed:
                return t   # a range: the readers expand it, as before
            else:
                return NumError("UNSUPPORTED", "a colon range %r is not evaluated here; write the values out" % t[:60])
        n = sum(len(r) for r in rows)
        if n == 1:
            out = rows[0][0]
        elif kind in ("scalar", "int", "typed"):
            # a vector where the engine takes one value (Switch Threshold '[0 1]') is valid in Simulink,
            # which expands the signal -- not supported here, not a malformed request
            return NumError("UNSUPPORTED", "one value is supported here, got %d (a vector parameter is not "
                            "supported for this parameter)" % n)
        else:
            if transposed and not (kind == "vector"):
                return NumError("UNSUPPORTED", "a transposed list %r is not evaluated; write it out" % t[:60])
            if kind == "vector":
                if len(rows) > 1 and len(rows[0]) > 1:
                    return NumError("BAD_PARAM", "a vector required, got a %d x %d matrix" % (len(rows), len(rows[0])))
                return "[" + " ".join(e for r in rows for e in r) + "]"   # a column (or a transposed row) as a row
            return "[" + "; ".join(" ".join(r) for r in rows) + "]"
    else:
        lit = _literal_arith(t)
        if lit is None:
            return _classify_other(t)
        return canonical(lit, kind)
    if kind == "int":
        try:
            v = float(out)
        except ValueError:
            return NumError("BAD_PARAM", "an integer is required, got %r" % t[:60])
        if v != v or v in (float("inf"), float("-inf")) or v != int(v):
            return NumError("BAD_PARAM", "an integer is required, got %r" % t[:60])
        return str(int(v))
    return out

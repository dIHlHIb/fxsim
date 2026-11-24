"""MATLAB Function blocks, v1 subset: a parser for HDL-style fixed-point scalar code.

This module only parses: it turns the function text into a small AST, and refuses everything outside the subset with
a stable code (UNSUPPORTED_MATLAB) and the line / column. It assigns no numeric meaning to anything -- the semantics
of every construct (default fimath, type inheritance, overflow, persistent initial values, ...) wait for recordings
of recordings and are implemented construct by construct as it arrives.

Subset (accepted by the parser):
  function header with scalar in / out arguments; subfunctions
  statements: assignment (x = e; x(:) = e), if / elseif / else, for k = a:b or a:s:b with constant bounds,
              persistent declarations, `if isempty(p) ... end` initialisation
  expressions: numbers, names, + - * (binary), unary -, comparisons (== ~= < <= > >=), logical (&& || & | ~),
               parentheses, calls of the functions in ALLOWED_CALLS and of the file's own subfunctions
Refused: while, switch, try, break / continue / return, function handles, matrices / vectors ([...], indexing other
than x(:)), cell arrays ({}), structs (a.b), strings ('...' or "..."), division / power / transpose, global, eval and
any other call, element-wise operators ./, ranges outside a for header.
Opened since by recordings: switch / case /
otherwise on a variable, ^ and .^, x(:) in expressions, x.WordLength / .FractionLength / .Signed / .numerictype,
coder.extrinsic / coder.unroll and extrinsic warning('...') calls, [a, b] = g(...), structs of scalar fields
(desugared to one variable per field), hdlfimath, ufi, floor, ceil, nextpow2, struct."""
import re
from fractions import Fraction

import fxmldefassign as DA

ALLOWED_CALLS = {"removefimath", "fi", "numerictype", "fimath", "cast", "bitand", "bitor", "bitxor", "bitshift", "bitsll", "bitsrl",
                 "bitsra", "bitsliceget", "bitconcat", "bitget", "bitset", "min", "max", "abs", "isempty",
                 "int8", "int16", "int32", "uint8", "uint16", "uint32", "logical", "true", "false",
                 # 
                 "nextpow2", "hdlfimath", "ufi", "floor", "ceil", "struct",
                 
                 "log2", "getmsb", "bitcmp", "mod",
                 
                 "zeros"}
# string arguments are only allowed as the name / value pairs of these (e.g. fimath('RoundingMethod', 'Floor'),
# cast(x, 'like', y))
STRING_ARG_CALLS = {"fi", "numerictype", "fimath", "cast", "struct", "zeros"}
# fi properties read as `x.Name` (numerictype: recordings)
FI_PROPS = ("WordLength", "FractionLength", "Signed", "numerictype")


class MlError(Exception):
    def __init__(self, code, line, col, message):
        Exception.__init__(self, "%s at %d:%d: %s" % (code, line, col, message))
        self.code, self.line, self.col, self.message = code, line, col, message


def _refuse(tok, msg, code="UNSUPPORTED_MATLAB"):
    raise MlError(code, tok[2], tok[3], msg)


# ---------------------------------------------------------------- tokens: (kind, text, line, col)
_TOKEN = re.compile(r"""
    (?P<ws>[ \t]+) | (?P<cont>\.\.\.[^\n]*\n) | (?P<comment>%[^\n]*) | (?P<nl>\n|;|,) |
    (?P<num>(\d+\.\d*|\.\d+|\d+)([eE][-+]?\d+)?) |
    (?P<str>'[^'\n]*'|"[^"\n]*") |
    (?P<name>[A-Za-z_][A-Za-z0-9_]*) |
    (?P<op>==|~=|<=|>=|&&|\|\||\.\*|\./|\.\^|\.'|[-+*/\\^<>=~&|()\[\]{}:@.'])
""", re.X)
KEYWORDS = {"function", "end", "if", "elseif", "else", "for", "while", "switch", "case", "otherwise", "try", "catch",
            "break", "continue", "return", "persistent", "global"}


def tokenize(text):
    out, line, pos, line_start = [], 1, 0, 0
    prev_sig = None
    while pos < len(text):
        m = _TOKEN.match(text, pos)
        if not m:
            raise MlError("UNSUPPORTED_MATLAB", line, pos - line_start + 1, "unexpected character %r" % text[pos])
        kind = m.lastgroup
        tok_text = m.group(0)
        col = pos - line_start + 1
        if kind == "cont":
            line += 1
            line_start = m.end()
        elif kind == "nl":
            out.append(("nl", tok_text, line, col))
            if tok_text == "\n":
                line += 1
                line_start = m.end()
        elif kind in ("ws", "comment"):
            pass
        else:
            if kind == "op" and tok_text == "'" and prev_sig and (prev_sig[0] in ("name", "num") or
                                                                   prev_sig[1] in (")", "]")):
                out.append(("op", "'", line, col))   # transpose (refused by the parser)
            elif kind == "str" or (kind == "op" and tok_text == "'"):
                if kind != "str":
                    raise MlError("UNSUPPORTED_MATLAB", line, col, "unterminated string")
                out.append(("str", tok_text, line, col))
            else:
                if kind == "name" and tok_text in KEYWORDS:
                    kind = "kw"
                out.append((kind, tok_text, line, col))
            prev_sig = out[-1]
        pos = m.end()
    out.append(("eof", "", line, pos - line_start + 1))
    return out


# ---------------------------------------------------------------- parser
class _P:
    def __init__(self, toks):
        self.t, self.i = toks, 0
        self.extrinsic = set()    # names declared with coder.extrinsic (in the function being parsed)

    def peek(self, k=0):
        return self.t[min(self.i + k, len(self.t) - 1)]

    def next(self):
        tok = self.peek()
        self.i += 1
        return tok

    def accept(self, kind, text=None):
        tok = self.peek()
        if tok[0] == kind and (text is None or tok[1] == text):
            self.i += 1
            return tok
        return None

    def expect(self, kind, text=None, what=None):
        tok = self.accept(kind, text)
        if tok is None:
            _refuse(self.peek(), "expected %s, found %r" % (what or text or kind, self.peek()[1]), "BAD_MATLAB")
        return tok

    def skip_nl(self):
        while self.peek()[0] == "nl":
            self.i += 1


def parse(text):
    """MATLAB Function text -> {"functions": [{"name", "inputs", "outputs", "body"}], "main": name}."""
    p = _P(tokenize(text))
    funcs = []
    p.skip_nl()
    while p.peek()[0] != "eof":
        funcs.append(_function(p))
        p.skip_nl()
    if not funcs:
        raise MlError("BAD_MATLAB", 1, 1, "no function")
    names = [f["name"] for f in funcs]
    if len(set(names)) != len(names):
        raise MlError("BAD_MATLAB", 1, 1, "two functions with the same name")
    ast = {"functions": funcs, "main": funcs[0]["name"]}
    _expand_massign(ast)
    for f in funcs:
        _desugar_structs(f)
    _check_calls(ast)
    return ast


def _function(p):
    kw = p.expect("kw", "function")
    p.extrinsic = set()
    outs = []
    if p.accept("op", "["):
        while not p.accept("op", "]"):
            outs.append(p.expect("name", what="output name")[1])
            p.accept("nl", ",")
        p.expect("op", "=")
    elif p.peek(1)[1] == "=":
        outs.append(p.expect("name")[1])
        p.expect("op", "=")
    name = p.expect("name", what="function name")[1]
    ins = []
    if p.accept("op", "("):
        while not p.accept("op", ")"):
            tok = p.expect("name", what="input name")
            ins.append(tok[1])
            p.accept("nl", ",")
    body = _block(p, ("end", "function", "eof"))
    p.accept("kw", "end")
    return {"name": name, "inputs": ins, "outputs": outs, "body": body, "line": kw[2]}


def _block(p, stops):
    out = []
    while True:
        p.skip_nl()
        tok = p.peek()
        if tok[0] == "eof" and "eof" in stops:
            return out
        if tok[0] == "kw" and tok[1] in stops:
            return out
        st = _statement(p)
        out.extend(st if isinstance(st, list) else [st])


def _statement(p):
    tok = p.peek()
    if tok[0] == "kw":
        if tok[1] == "if":
            return _if(p)
        if tok[1] == "for":
            return _for(p)
        if tok[1] == "switch":
            return _switch(p)
        if tok[1] == "persistent":
            p.next()
            names = []
            while p.peek()[0] == "name":
                names.append(p.next()[1])
            if not names:
                _refuse(tok, "persistent without a name", "BAD_MATLAB")
            return {"op": "persistent", "names": names, "line": tok[2]}
        _refuse(tok, "%s is not in the supported MATLAB Function subset" % tok[1])
    if tok[0] == "op" and tok[1] == "[":
        return _massign(p)
    if tok[0] != "name":
        _refuse(tok, "a statement must start with an assignment target, found %r" % tok[1], "BAD_MATLAB")
    if tok[1] == "coder" and p.peek(1)[1] == "." and p.peek(2)[0] == "name":
        return _coder(p)
    if p.peek(1)[1] == "(" and tok[1] in p.extrinsic:
        # a bare call of a function declared coder.extrinsic (warning('x') runs in
        # MATLAB, the block's values are unchanged); recorded: warning with text arguments only
        p.next()
        p.next()
        # warning(['limit ', num2str(u), ' reached']) -- the message runs in MATLAB and changes no
        # value. Its arguments are texts, or a row [ ... ] of texts and num2str / int2str of an expression
        # (review of recordings: an unbalanced message or a bare value in it was accepted)
        reads = []

        def msg_item():
            a = p.next()
            if a[0] == "str":
                return
            if a[0] == "name" and a[1] in ("num2str", "int2str") and p.accept("op", "("):
                x = _expr(p)
                p.expect("op", ")")
                reads.extend(n for n in _nodes(x) if n.get("op") == "var")
                return
            _refuse(a, "%r in the message of %s (recorded: texts and num2str(...))" % (a[1], tok[1]))
        if not p.accept("op", ")"):
            while True:
                if p.accept("op", "["):
                    while True:
                        msg_item()
                        if p.accept("op", "]"):
                            break
                        p.expect("nl", ",", "',' between the pieces of the message")
                else:
                    msg_item()
                if p.accept("op", ")"):
                    break
                p.expect("nl", ",", "',' between the arguments of %s" % tok[1])
        if tok[1] != "warning":
            _refuse(tok, "a call of the extrinsic function %s (recorded: warning)" % tok[1])
        _end_of_statement(p)
        # the variables the message reads must hold values there (validate), like any read
        return {"op": "nop", "what": "extrinsic call", "name": tok[1], "reads": reads, "line": tok[2]}
    target = p.next()
    if p.peek()[1] == "." and p.peek(1)[0] == "name" and p.peek(2)[1] == "=":
        p.next()
        fd = p.next()
        p.next()
        e = _expr(p)
        _end_of_statement(p)
        return {"op": "assign", "target": target[1], "field": fd[1], "whole": False, "expr": e, "line": target[2],
                "col": target[3]}
    if p.peek()[1] == "." and p.peek(1)[0] == "name" and [t[1] for t in (p.peek(2), p.peek(3), p.peek(4),
                                                                         p.peek(5))] == ["(", ":", ")", "="]:
        # st.c(:) = a keeps the field's type (quantised and overflowed by the block fimath)
        p.next()
        fd = p.next()
        for _ in range(4):
            p.next()
        e = _expr(p)
        _end_of_statement(p)
        return {"op": "assign", "target": target[1], "field": fd[1], "whole": True, "expr": e, "line": target[2],
                "col": target[3]}
    whole = False
    index = None
    if p.accept("op", "("):
        if p.peek()[1] == ":" and p.peek(1)[1] == ")":
            p.next()
            p.next()
            whole = True
        else:
            # h(1) = e, h(p+1) = e, h(2:4) = h(1:3) -- one index or a range a:b of a row vector
            index = _index_arg(p)
            p.expect("op", ")")
    if p.peek()[1] != "=":
        _refuse(p.peek(), "only assignments are supported as statements (a bare expression or call is not)")
    p.next()
    e = _expr(p)
    end = p.peek()
    if end[0] not in ("nl", "eof") and not (end[0] == "kw" and end[1] in ("end", "else", "elseif")):
        _refuse(end, "unexpected %r after the assignment" % end[1], "BAD_MATLAB")
    st_ = {"op": "assign", "target": target[1], "whole": whole, "expr": e, "line": target[2], "col": target[3]}
    if index is not None:
        st_["index"] = index
    return st_


def _index_arg(p):
    """One index of a row vector: an expression, or a range a:b."""
    if p.peek()[1] == ",":
        _refuse(p.peek(), "two indices (matrices) are not supported (recorded: row vectors)")
    a = _expr(p)
    if p.accept("op", ":"):
        b = _expr(p)
        if p.peek()[1] == ":":
            _refuse(p.peek(), "a stepped range as an index (not recorded)")
        a = {"op": "range", "args": [a, b]}
    if p.peek()[1] == ",":
        _refuse(p.peek(), "two indices (matrices) are not supported (recorded: row vectors)")
    return a


def _massign(p):
    """[p, q] = g(...) / [p, ~] = g(...): the outputs of a local function; expanded
    after parsing (_expand_massign)."""
    tok = p.expect("op", "[")
    targets = []
    while not p.accept("op", "]"):
        t = p.next()
        if t[0] == "name":
            targets.append(t[1])
        elif t[0] == "op" and t[1] == "~":
            targets.append(None)
        else:
            _refuse(t, "expected an output name or ~ in [...] =, found %r" % t[1], "BAD_MATLAB")
        p.accept("nl", ",")
    p.expect("op", "=")
    e = _expr(p)
    _end_of_statement(p)
    if e.get("op") != "call":
        _refuse(tok, "[...] = needs a call of a local function", "UNSUPPORTED_MATLAB")
    if not any(targets):
        _refuse(tok, "[...] = without an output name", "BAD_MATLAB")
    return {"op": "massign", "targets": targets, "expr": e, "line": tok[2], "col": tok[3]}


def _expand_massign(ast):
    """[p, q] = g(x) -> p = g{1}(x); q = g{2}(x), where g{k} is a copy of the local function g returning its k-th
    output only. Exact as long as calling g twice is the same as calling it once: refused when g (or what it calls)
    has persistent variables."""
    funcs = {f["name"]: f for f in ast["functions"]}

    def stateful(n, seen=()):
        g = funcs[n]
        if any(st["op"] == "persistent" for st in _all_stmts(g["body"])):
            return True
        return any(stateful(c["name"], seen + (n,)) for c in _calls(g["body"], []) if c["name"] in funcs and
                   c["name"] not in seen + (n,))

    def copy_for(n, k):
        import copy
        nm = "%s{%d}" % (n, k + 1)
        if nm not in funcs:
            g = copy.deepcopy(funcs[n])
            g["name"] = nm
            g["outputs"] = [funcs[n]["outputs"][k]]
            funcs[nm] = g
            ast["functions"].append(g)
        return nm

    def walk(stmts):
        out = []
        for st in stmts:
            if st["op"] == "massign":
                e = st["expr"]
                n = e["name"]
                if n not in funcs or n == ast["main"]:
                    raise MlError("UNSUPPORTED_MATLAB", st["line"], st["col"], "[...] = %s(...): only a local "
                                  "function's outputs (recorded)" % n)
                g = funcs[n]
                if len(st["targets"]) > len(g["outputs"]):
                    raise MlError("BAD_MATLAB", st["line"], st["col"], "%s returns %d values, %d asked"
                                  % (n, len(g["outputs"]), len(st["targets"])))
                if stateful(n):
                    # each output's copy would keep its own persistent state
                    raise MlError("UNSUPPORTED_MATLAB", st["line"], st["col"], "[...] = %s(...) of a function with "
                                  "persistent variables (not recorded)" % n)
                named = [t for t in st["targets"] if t]
                read = {x["name"] for x in _nodes(e["args"]) if x.get("op") == "var"}
                if len(set(named)) != len(named) or set(named) & read:
                    # [p, q] = g(p, q) -- the copies run one after the other, the second would read the
                    # new p
                    raise MlError("UNSUPPORTED_MATLAB", st["line"], st["col"], "[...] = %s(...) assigning a name "
                                  "twice or one its arguments read (not recorded)" % n)
                import copy
                for k, t in enumerate(st["targets"]):
                    if t is None:
                        continue
                    c = copy.deepcopy(e)
                    c["name"] = copy_for(n, k)
                    out.append({"op": "assign", "target": t, "whole": False, "expr": c, "line": st["line"],
                                "col": st["col"]})
                continue
            if st["op"] == "if":
                st["arms"] = [(c, walk(b)) for c, b in st["arms"]]
                if st["else"] is not None:
                    st["else"] = walk(st["else"])
            elif st["op"] == "for":
                st["body"] = walk(st["body"])
            out.append(st)
        return out
    k = 0
    while k < len(ast["functions"]):      # the copies made on the way are walked too
        f = ast["functions"][k]
        f["body"] = walk(f["body"])
        k += 1


def _desugar_structs(f):
    """Structs of scalar fields (a persistent register bundle st = struct('cnt', ...,
    'flag', ...), its fields read and written): each field becomes a variable named 'st.cnt' (no MATLAB name has a
    dot), so the semantics and the compiler see plain scalars:
      st = struct('a', e1, 'b', e2)  ->  st.a = e1; st.b = e2      persistent st  ->  persistent st.a st.b
      isempty(st)  ->  isempty(st.a)                               st.a (read / write) -> the variable st.a
    Checked here: every constructor of st names the same fields in the same order, its values do not read st, a
    field outside them is refused, st is never used as a whole (passed, returned, copied, indexed). Whether st
    holds a value where it is read is checked on the struct as a whole (validate: f["_structs"])."""
    import copy
    fields = {}

    def ctor(e):
        return isinstance(e, dict) and e.get("op") == "call" and e.get("name") == "struct"

    for st in _all_stmts(f["body"]):
        if st["op"] == "assign" and not st.get("field") and ctor(st["expr"]):
            e = st["expr"]
            a = e["args"]
            if len(a) % 2 or not a or any(x.get("op") != "str" for x in a[0::2]):
                raise MlError("UNSUPPORTED_MATLAB", st["line"], st["col"], "struct('name', value, ...) with text "
                              "field names only")
            names = [x["text"] for x in a[0::2]]
            if len(set(names)) != len(names):
                raise MlError("BAD_MATLAB", st["line"], st["col"], "a field named twice in struct(...)")
            if fields.setdefault(st["target"], names) != names:
                raise MlError("UNSUPPORTED_MATLAB", st["line"], st["col"], "%s built with different fields (not "
                              "recorded)" % st["target"])
            for v in a[1::2]:
                if v.get("op") == "str" or ctor(v):
                    raise MlError("UNSUPPORTED_MATLAB", st["line"], st["col"], "a struct field holding a text or "
                                  "a struct (recorded: scalar values)")
    _fieldwise(f, fields)
    tmp_no = [0]
    if not fields:
        for e in _nodes(f["body"]):
            if e.get("op") == "assign" and e.get("field"):
                # y.a = ... of a variable that is no struct built here
                raise MlError("UNSUPPORTED_MATLAB", e["line"], e["col"], "structure fields is not in the supported "
                              "subset (%s.%s = ...: only fields of a struct(...) built in the function)"
                              % (e["target"], e["field"]))
            if e.get("op") == "field":
                raise MlError("UNSUPPORTED_MATLAB", e["line"], e["col"], "structure fields is not in the supported "
                              "subset (%s.%s: only fields of a struct(...) built in the function, and fi "
                              "properties)" % (e["base"]["name"], e["name"]))
        return
    if set(fields) & {st["var"] for st in _all_stmts(f["body"]) if st["op"] == "for"}:
        raise MlError("UNSUPPORTED_MATLAB", f["line"], 1, "a loop variable that is also a struct (not recorded)")
    if set(fields) & (set(f["inputs"]) | set(f["outputs"])):
        raise MlError("UNSUPPORTED_MATLAB", f["line"], 1, "a struct input or output (not recorded)")

    def fname(base, fd, line, col):
        if fd not in fields[base]:
            raise MlError("BAD_MATLAB", line, col, "%s has no field %s" % (base, fd))
        return "%s.%s" % (base, fd)

    def expr(e):
        if isinstance(e, list):
            return [expr(x) for x in e]
        if not isinstance(e, dict):
            return e
        op = e.get("op")
        if op == "field":
            b = e["base"]["name"]
            if b not in fields:
                raise MlError("UNSUPPORTED_MATLAB", e["line"], e["col"], "structure fields is not in the supported "
                              "subset (%s is not a struct built in the function)" % b)
            return {"op": "var", "name": fname(b, e["name"], e["line"], e["col"]), "group": b, "line": e["line"],
                    "col": e["col"]}
        if op == "call" and e["name"] == "isempty" and len(e["args"]) == 1 and \
                e["args"][0].get("op") == "var" and e["args"][0]["name"] in fields:
            a0 = e["args"][0]
            return dict(e, args=[{"op": "var", "name": "%s.%s" % (a0["name"], fields[a0["name"]][0]),
                                  "group": a0["name"], "line": a0.get("line"), "col": a0.get("col")}])
        if op == "var" and e["name"] in fields:
            raise MlError("UNSUPPORTED_MATLAB", e.get("line", 0), e.get("col", 1), "the struct %s used as a whole "
                          "(recorded: its fields)" % e["name"])
        if ctor(e):
            raise MlError("UNSUPPORTED_MATLAB", e.get("line", 0), e.get("col", 1), "struct(...) other than as the "
                          "value of a variable")
        return {k: (expr(v) if k != "op" else v) for k, v in e.items()}

    def walk(stmts):
        out = []
        for st in stmts:
            op = st["op"]
            if op == "persistent":
                names = []
                for nm in st["names"]:
                    names += ["%s.%s" % (nm, x) for x in fields[nm]] if nm in fields else [nm]
                out.append(dict(st, names=names, groups={("%s.%s" % (nm, x)): nm for nm in st["names"]
                                                          if nm in fields for x in fields[nm]}))
            elif op == "assign" and st["target"] in fields and not st.get("field") and not st.get("whole") and \
                    st["expr"].get("op") == "var" and st["expr"]["name"] in fields:
                out += _whole_struct_assign(st, fields)
            elif op == "assign" and st["target"] in fields and not st.get("field"):
                if st.get("whole") or not ctor(st["expr"]):
                    raise MlError("UNSUPPORTED_MATLAB", st["line"], st["col"], "%s assigned other than by "
                                  "struct(...) (recorded: the constructor and field writes)" % st["target"])
                a = st["expr"]["args"]
                vals = [expr(v) for v in a[1::2]]
                if any(n.get("group") == st["target"] for v in vals for n in _nodes(v)):
                    # st = struct('c', f(st.c), 'f', ~st.f) reads the
                    # struct of before the statement in every value (equal step by step to building next field
                    # by field, then st = next): the values go to temporaries first, then into st
                    tmp_no[0] += 1
                    tmp = "%%struct%d" % tmp_no[0]
                    for nmx, v in zip(fields[st["target"]], vals):
                        out.append({"op": "assign", "target": "%s.%s" % (tmp, nmx), "whole": False, "expr": v,
                                    "line": st["line"], "col": st["col"]})
                    vals = [{"op": "var", "name": "%s.%s" % (tmp, nmx), "line": st["line"], "col": st["col"]}
                            for nmx in fields[st["target"]]]
                for k, (nmx, v) in enumerate(zip(fields[st["target"]], vals)):
                    out.append({"op": "assign", "target": "%s.%s" % (st["target"], nmx), "group": st["target"],
                                "ctor": k, "whole": False, "expr": v, "line": st["line"], "col": st["col"]})
            elif op == "assign" and st.get("field"):
                if st["target"] not in fields:
                    raise MlError("UNSUPPORTED_MATLAB", st["line"], st["col"], "structure fields is not in the "
                                  "supported subset (%s is not a struct built in the function)" % st["target"])
                out.append(dict({"op": "assign", "target": fname(st["target"], st["field"], st["line"], st["col"]),
                                 "group": st["target"], "whole": bool(st.get("whole")), "expr": expr(st["expr"]),
                                 "line": st["line"],
                                 "col": st["col"]}, **({"ctor": st["_create"]} if "_create" in st else {})))
            elif op == "if":
                out.append(dict(st, arms=[(expr(c), walk(b)) for c, b in st["arms"]],
                                **{"else": walk(st["else"]) if st["else"] is not None else None}))
            elif op == "for":
                out.append(dict(st, body=walk(st["body"])))
            elif op == "assign":
                out.append(dict(st, expr=expr(st["expr"])))
            else:
                out.append(st)
        return out
    f["body"] = walk(f["body"])
    f["_structs"] = {k: list(v) for k, v in fields.items()}


def _whole_struct_assign(st, fields):
    """dst = src of two structs: Simulink accepts it when both have the same field names in the same
    order (next = st, then a field changed); a different order
, an extra field or a field missing where src is assigned
 is rejected; a different field type is rejected by the
    semantics (st["_struct_copy"]). Becomes dst.a = src.a; dst.b = src.b, the first one creating dst."""
    dst, src = st["target"], st["expr"]["name"]
    if dst == src:
        raise MlError("UNSUPPORTED_MATLAB", st["line"], st["col"], "%s = %s (not recorded)" % (dst, src))
    fd, fs = fields[dst], fields[src]
    have = st.get("_src_have", {}).get(src, fs)
    if have != fs:
        missing = [x for x in fs if x not in have]
        raise MlError("BAD_MATLAB", st["line"], st["col"], "structure field '%s.%s' is undefined where %s is "
                      "assigned to %s (Simulink: undefined on some execution paths)" % (src, missing[0], src, dst))
    if fd != fs and st.get("_copy_into"):
        raise MlError("UNSUPPORTED_MATLAB", st["line"], st["col"], "%s copied from %s and then given other fields "
                      "(not recorded)" % (dst, src))
    if fd != fs:
        for k, (a, b) in enumerate(zip(fd, fs)):
            if a != b:
                raise MlError("BAD_MATLAB", st["line"], st["col"], "struct mismatch in %s = %s: field %d is '%s' on "
                              "the left and '%s' on the right (Simulink: the field names and their order must be the "
                              "same)" % (dst, src, k + 1, a, b))
        extra = (fs[len(fd):] or fd[len(fs):])[0]
        raise MlError("BAD_MATLAB", st["line"], st["col"], "struct mismatch in %s = %s: the field '%s' is on one "
                      "side only (Simulink: the structs must have the same fields)" % (dst, src, extra))
    return [{"op": "assign", "target": "%s.%s" % (dst, x), "group": dst, "ctor": k, "whole": False,
             "_struct_copy": src,
             "expr": {"op": "var", "name": "%s.%s" % (src, x), "group": src, "line": st["line"], "col": st["col"]},
             "line": st["line"], "col": st["col"]} for k, x in enumerate(fd)]


def _fieldwise(f, fields):
    """Structs built field by field, without struct(...): s.a = u; s.b = u + 1 builds s with fields
    (a, b), as struct('a', u, 'b', u + 1) would (same values and types, recordings equals
    recordings). Simulink's conditions, checked here on every control path:
      - the fields are created in the same order on all paths (rejected;
        recordings: accepted);
      - a field is read only after it was assigned (rejected);
      - a persistent struct gets all its fields in its `if isempty(r)` initialisation (
        a field set nowhere else is invalid there).
    Creating a field inside a loop, or on some paths only, was not recorded and is refused. Adds the structs to
    fields (name -> field order) and marks each creating assignment with st["_create"] = its index."""
    fw = []
    built = set(fields)       # built by struct(...): the structs a field-built one may be copied from
    for st in _all_stmts(f["body"]):
        if st["op"] == "assign" and st.get("field") and st["target"] not in fields and st["target"] not in fw:
            fw.append(st["target"])
    if not fw:
        return
    ins = set(f["inputs"]) | set(f["outputs"])
    pers = {n for st in _all_stmts(f["body"]) if st["op"] == "persistent" for n in st["names"]}
    loopvars = {st["var"] for st in _all_stmts(f["body"]) if st["op"] == "for"}
    for b in fw:
        if b in loopvars:
            raise MlError("UNSUPPORTED_MATLAB", f["line"], 1, "%s is both a loop variable and a struct (not "
                          "recorded)" % b)
        if b in ins:
            raise MlError("UNSUPPORTED_MATLAB", f["line"], 1, "fields of the port %s (a struct / bus port, not "
                          "recorded)" % b)
        order = []
        init_seen = [False]

        def reads(e, have, line):
            for n in _nodes(e):
                if n.get("op") == "field" and n["base"]["name"] == b and n["name"] not in have:
                    raise MlError("BAD_MATLAB", n.get("line", line), n.get("col", 1), "structure field '%s' of %s "
                                  "is read before it is assigned (Simulink: invalid structure field)" % (n["name"], b))

        def walk(stmts, have, in_loop, init):
            have = list(have)
            for st in stmts:
                op = st["op"]
                if op == "assign":
                    reads(st["expr"], have if b not in pers or init else order, st["line"])
                    if st["expr"].get("op") == "var" and st["expr"]["name"] == b and not st.get("field"):
                        # b assigned whole to another struct: the fields it holds there
                        st.setdefault("_src_have", {})[b] = list(have if b not in pers or init else order)
                    if st["target"] == b and not st.get("field") and not st.get("whole") and \
                            st["expr"].get("op") == "var" and st["expr"]["name"] in built and not have:
                        # b = st, a copy of a struct(...)-built struct, then fields changed (recordings
                        # recordings)
                        if in_loop or b in pers:
                            raise MlError("UNSUPPORTED_MATLAB", st["line"], st["col"], "%s copied from a struct %s "
                                          % (b, "inside a loop" if in_loop else "into a persistent struct") +
                                          "(not recorded)")
                        have = list(fields[st["expr"]["name"]])
                        st["_copy_into"] = True
                        if not order:
                            order.extend(have)
                        elif order != have:
                            raise MlError("BAD_MATLAB", st["line"], st["col"], "the fields of %s are assigned in "
                                          "a different order on different paths (Simulink: struct fields must be "
                                          "assigned in the same order on all control-flow paths)" % b)
                    elif st["target"] == b and st.get("field"):
                        fd = st["field"]
                        if fd not in have:
                            if in_loop:
                                raise MlError("UNSUPPORTED_MATLAB", st["line"], st["col"], "field %s.%s created "
                                              "inside a loop (not recorded)" % (b, fd))
                            if b in pers and not init:
                                raise MlError("BAD_MATLAB", st["line"], st["col"], "field %s.%s is not assigned in "
                                              "the `if isempty(%s)` initialisation (Simulink: invalid structure "
                                              "field)" % (b, fd, b))
                            st["_create"] = len(have)
                            have.append(fd)
                            if init or b not in pers:
                                if len(order) < len(have):
                                    order.append(fd)
                                elif order[len(have) - 1] != fd:
                                    raise MlError("BAD_MATLAB", st["line"], st["col"], "the fields of %s are "
                                                  "assigned in a different order on different paths (Simulink: "
                                                  "struct fields must be assigned in the same order on all "
                                                  "control-flow paths)" % b)
                    elif st["target"] == b:
                        raise MlError("UNSUPPORTED_MATLAB", st["line"], st["col"], "%s assigned as a whole after "
                                      "being built field by field (not recorded)" % b)
                elif op == "if":
                    outs = []
                    after_init = False
                    for c, body in st["arms"]:
                        reads(c, have if b not in pers or init else order, st["line"])
                        is_init = b in pers and _is_isempty(c) == b
                        if is_init and (init_seen[0] or init or in_loop):
                            # a second `if isempty(r)` (after r exists, or nested in
                            # the first) never runs once r is set -- a field created there would be read as 0
                            raise MlError("UNSUPPORTED_MATLAB", st["line"], 1, "a second `if isempty(%s)` "
                                          "initialisation of the persistent struct %s (not recorded)" % (b, b))
                        if is_init:
                            init_seen[0] = True
                        # the arms after the initialisation see the struct of the step before
                        outs.append(walk(body, order if after_init else have, in_loop, init or is_init))
                        if is_init:
                            outs[-1] = None     # the other paths hold the struct from the step before
                            after_init = True
                    outs.append(walk(st["else"], order if after_init else have, in_loop, init)
                                if st["else"] is not None else (list(order) if after_init else have))
                    real = [o for o in outs if o is not None]
                    if any(o != real[0] for o in real):
                        longest = max(real, key=len)
                        if all(o == longest[:len(o)] for o in real):
                            raise MlError("UNSUPPORTED_MATLAB", st["line"], 1, "fields of %s created on some paths "
                                          "only (not recorded)" % b)
                        raise MlError("BAD_MATLAB", st["line"], 1, "the fields of %s are assigned in a different "
                                      "order on different paths (Simulink: struct fields must be assigned in the "
                                      "same order on all control-flow paths)" % b)
                    have = real[0] if real else have
                    if b in pers and not init and any(o is None for o in outs):
                        have = list(order)
                elif op == "for":
                    walk(st["body"], have, True, init)
            return have
        walk(f["body"], [], False, False)
        if b in pers and not order:
            raise MlError("UNSUPPORTED_MATLAB", f["line"], 1, "persistent %s built field by field outside its "
                          "`if isempty(%s)` initialisation (not recorded)" % (b, b))
        fields[b] = order


def _nodes(e):
    if isinstance(e, dict):
        yield e
        for k, v in e.items():
            if k != "op":
                yield from _nodes(v)
    elif isinstance(e, (list, tuple)):
        for v in e:
            yield from _nodes(v)


def _struct_view(f):
    """f as the definite-assignment checks see it: every field variable st.x renamed to its struct st (a struct
    holds a value or not as a whole); a constructor is one assignment of st, a field write reads st first (the
    struct must exist) and assigns it."""
    import copy
    g = copy.deepcopy(f)

    def ren(e):
        for n in _nodes(e):
            if n.get("op") == "var" and n.get("group"):
                n["name"] = n["group"]

    def walk(stmts):
        out = []
        for st in stmts:
            if st["op"] == "persistent" and st.get("groups"):
                names = []
                for nm in st["names"]:
                    nm = st["groups"].get(nm, nm)
                    if nm not in names:
                        names.append(nm)
                st = dict(st, names=names)
            elif st["op"] == "assign" and st.get("group"):
                ren(st["expr"])
                e = st["expr"]
                if st.get("ctor") != 0:
                    # a field write, or a later field of the constructor: the struct exists already
                    e = {"op": "&&", "args": [e, {"op": "var", "name": st["group"], "line": st["line"],
                                                  "col": st["col"]}]}
                st = dict(st, target=st["group"], expr=e)
            elif st["op"] == "assign":
                ren(st["expr"])
            elif st["op"] == "if":
                for c, _b in st["arms"]:
                    ren(c)
                st = dict(st, arms=[(c, walk(b)) for c, b in st["arms"]],
                          **{"else": walk(st["else"]) if st["else"] is not None else None})
            elif st["op"] == "for":
                st = dict(st, body=walk(st["body"]))
            out.append(st)
        return out
    g["body"] = walk(g["body"])
    return g


def _all_stmts(stmts):
    for st in stmts:
        yield st
        if st["op"] == "if":
            for _c, b in st["arms"]:
                yield from _all_stmts(b)
            yield from _all_stmts(st["else"] or [])
        elif st["op"] == "for":
            yield from _all_stmts(st["body"])


def _end_of_statement(p):
    end = p.peek()
    if end[0] not in ("nl", "eof") and not (end[0] == "kw" and end[1] in ("end", "else", "elseif")):
        _refuse(end, "unexpected %r after the statement" % end[1], "BAD_MATLAB")


def _coder(p):
    """coder.extrinsic('f', ...) declares functions that run in MATLAB; coder.unroll (bare, before a for) is a
    code-generation hint. recordings: neither changes a value; the declared names may then be
    called as statements (see _statement)."""
    tok = p.next()
    p.next()
    what = p.next()
    if what[1] == "extrinsic":
        p.expect("op", "(")
        names = []
        while True:
            a = p.next()
            if a[0] != "str":
                _refuse(a, "coder.extrinsic takes function names as texts", "BAD_MATLAB")
            names.append(a[1][1:-1])
            if p.accept("op", ")"):
                break
            p.expect("nl", ",", "',' between arguments")
        p.extrinsic.update(names)
        _end_of_statement(p)
        return {"op": "nop", "what": "coder.extrinsic", "names": names, "line": tok[2]}
    if what[1] == "unroll" and p.peek()[1] != "(":
        _end_of_statement(p)
        return {"op": "nop", "what": "coder.unroll", "line": tok[2]}
    _refuse(what, "coder.%s is not in the supported subset (recorded: coder.extrinsic, a bare coder.unroll)"
            % what[1])


def _switch(p):
    """switch v / case c / case {c1, c2} / otherwise / end on a variable, with constant cases (recordings
    recordings: int16 u), rewritten as if / elseif / else on v == c; the semantics refuse a switch on a fixed-point
    value, as Simulink does (its case constants are compared fi == non-fi), and anything but an integer."""
    tok = p.expect("kw", "switch")
    subj = _expr(p)
    pre = []
    if subj.get("op") == "var" and not subj.get("colon"):
        var = subj
    else:
        # switch (u), switch uint8(u), switch st.m -- the subject is evaluated once, into a
        # variable no MATLAB code can name
        p.switch_no = getattr(p, "switch_no", 0) + 1
        nm = "%%switch%d" % p.switch_no
        pre = [{"op": "assign", "target": nm, "whole": False, "expr": subj, "line": tok[2], "col": tok[3]}]
        var = {"op": "var", "name": nm, "line": tok[2], "col": tok[3]}
    p.skip_nl()
    arms, orelse = [], None
    while True:
        p.skip_nl()
        if p.accept("kw", "case"):
            vals = []
            if p.accept("op", "{"):
                while True:
                    vals.append(_case_const(p))
                    if p.accept("op", "}"):
                        break
                    if p.peek()[1] == "-":
                        # {1 -2} is two values, {1 - 2} one; the tokens do not keep the spacing
                        _refuse(p.peek(), "a minus inside a case list without commas (write {1, -2} or {1 - 2} "
                                "with commas around the elements)")
                    if p.peek()[0] in ("num", "name"):
                        continue      # case {1 2}, separated by a space
                    p.expect("nl", ",", "',' between case values")
            else:
                vals.append(_case_const(p))
            cond = None
            for c in vals:
                cmp_ = {"op": "==", "args": [dict(var), c], "switch": True, "line": tok[2]}
                cond = cmp_ if cond is None else {"op": "||", "args": [cond, cmp_]}
            arms.append((cond, _block(p, ("case", "otherwise", "end"))))
            continue
        if p.accept("kw", "otherwise"):
            orelse = _block(p, ("end",))
        p.expect("kw", "end", "end of switch")
        break
    if not arms:
        _refuse(tok, "a switch without a case", "BAD_MATLAB")
    return pre + [{"op": "if", "arms": arms, "else": orelse, "line": tok[2]}]


def _case_const(p):
    neg = bool(p.accept("op", "-"))
    tok = p.peek()
    if tok[0] == "name" and not neg and p.peek(1)[1] not in ("(", "."):
        # case K with K = 3 -- a constant variable, folded by the semantics (else refused there)
        p.next()
        return {"op": "var", "name": tok[1], "case_const": True, "line": tok[2], "col": tok[3]}
    if tok[0] != "num":
        _refuse(tok, "case values must be numeric constants (recorded)")
    p.next()
    n = {"op": "num", "text": tok[1]}
    return {"op": "neg", "args": [n]} if neg else n


def _condition(e):
    """An if / elseif condition: & and | short-circuit there like && and || (
    with `a > 0 & g(a) > 0` MATLAB does not call g when a <= 0 -- the same 16 steps as the && program). The chain of
    & / | / && / || operators at the top of the condition is rewritten; elsewhere (an assignment) & stays
    element-wise and evaluates both sides."""
    # only the chain of & / | at the top of the condition -- not under ~, ==, && or || (an & in
    # the right operand of || evaluates both sides). The rewritten node keeps sc_from: for
    # definite assignment MATLAB still reads both operands (`if isempty(t) | t > a` refused at
    # the t, as without the short circuit)
    if isinstance(e, dict) and e.get("op") in ("&", "|") and len(e.get("args", [])) == 2:
        return dict(e, op={"&": "&&", "|": "||"}[e["op"]], sc_from=e["op"], args=[_condition(x) for x in e["args"]])
    return e


def _if(p):
    tok = p.expect("kw", "if")
    arms = []
    cond = _condition(_expr(p))
    body = _block(p, ("elseif", "else", "end"))
    arms.append((cond, body))
    orelse = None
    while True:
        if p.accept("kw", "elseif"):
            c = _condition(_expr(p))
            arms.append((c, _block(p, ("elseif", "else", "end"))))
            continue
        if p.accept("kw", "else"):
            orelse = _block(p, ("end",))
        p.expect("kw", "end", "end of if")
        break
    return {"op": "if", "arms": arms, "else": orelse, "line": tok[2]}


def _for(p):
    tok = p.expect("kw", "for")
    var = p.expect("name", what="loop variable")[1]
    p.expect("op", "=")
    parts = [_expr(p)]
    while p.accept("op", ":"):
        parts.append(_expr(p))
    if len(parts) not in (2, 3):
        _refuse(tok, "for needs a range a:b or a:s:b with constant bounds")
    body = _block(p, ("end",))
    p.expect("kw", "end", "end of for")
    lo, hi, step = (parts[0], parts[1], {"op": "num", "text": "1"}) if len(parts) == 2 else \
        (parts[0], parts[2], parts[1])
    vals = [_literal_int(x) for x in (lo, hi, step)]
    if all(v is not None for v in vals):
        lo, hi, step = vals
        if step == 0:
            _refuse(tok, "a for step of 0", "BAD_MATLAB")
        return {"op": "for", "var": var, "lo": lo, "hi": hi, "step": step, "body": body, "line": tok[2]}
    if any(_literal_value(x) is not None and _literal_value(x).denominator != 1 for x in (lo, hi, step)):
        _refuse(tok, "for-loop bounds must be integer constants")
    # bounds from constant variables or fi properties (N = 8; for k = 1:N / 0:N-1 / 1:a.WordLength),
    # resolved by the semantics (fxmlsem folds them to integers or refuses); the checks here assume the body may
    # run any number of times
    return {"op": "for", "var": var, "lo": None, "hi": None, "step": None, "bounds": [lo, hi, step], "body": body,
            "line": tok[2]}


def num_value(text):
    """The value of a numeric literal as MATLAB holds it: the nearest binary64 double (
    127.49999999999999999 is 127.5, so int8() of it is the same as of 127.5). Folded nodes carry an exact
    'p/q' text of a double already. None for a literal beyond the doubles (Inf in MATLAB)."""
    import math
    try:
        f = float(text)
    except ValueError:
        return Fraction(text)
    return Fraction(f) if math.isfinite(f) else None


def _literal_value(e):
    """A Fraction for an expression of numeric literals and + - * / unary minus, else None."""
    from fractions import Fraction
    op = e.get("op")
    if op == "num":
        return num_value(e["text"])
    if op == "neg":
        v = _literal_value(e["args"][0])
        return None if v is None else -v
    if op in ("+", "-", "*", "/"):
        a, b = (_literal_value(x) for x in e["args"])
        if a is None or b is None or op == "/" and b == 0:
            return None
        # MATLAB computes in double (review of recordings: 1:0.3/0.1 runs twice -- 2.9999999999999996)
        import math
        try:
            fa, fb = float(a), float(b)
        except OverflowError:
            return None
        r = fa + fb if op == "+" else fa - fb if op == "-" else fa * fb if op == "*" else fa / fb
        return Fraction(r) if math.isfinite(r) else None
    return None


def _literal_int(e):
    v = _literal_value(e)
    return int(v) if v is not None and v.denominator == 1 else None


def _const_expr(p):
    """A constant integer bound: an integer literal, optionally negated."""
    neg = bool(p.accept("op", "-"))
    tok = p.peek()
    if tok[0] != "num" or not re.match(r"^\d+$", tok[1]):
        _refuse(tok, "for-loop bounds must be integer constants")
    p.next()
    return -int(tok[1]) if neg else int(tok[1])


# precedence climbing: || < && < | < & < comparisons < + - < * < unary
_BIN = [("||",), ("&&",), ("|",), ("&",), ("==", "~=", "<", "<=", ">", ">="), ("+", "-"), ("*", ".*", "/")]
_REFUSED_OPS = {"\\": "left division",
                "./": "element-wise division", "'": "transpose", ".'": "transpose",
                "@": "function handles", "{": "cell arrays", "[": "matrices / vectors", ".": "structure fields"}


def _expr(p, level=0):
    if level == len(_BIN):
        return _unary(p)
    left = _expr(p, level + 1)
    while True:
        tok = p.peek()
        if tok[0] == "op" and tok[1] in _BIN[level]:
            p.next()
            right = _expr(p, level + 1)
            left = {"op": tok[1], "args": [left, right]}
            continue
        if tok[0] == "op" and tok[1] in _REFUSED_OPS:
            _refuse(tok, "%s is not in the supported subset" % _REFUSED_OPS[tok[1]])
        return left


def _unary(p):
    tok = p.peek()
    if tok[0] == "op" and tok[1] in ("-", "+", "~"):
        p.next()
        a = _unary(p)
        return a if tok[1] == "+" else {"op": "neg" if tok[1] == "-" else "not", "args": [a]}
    return _power(p)


def _power(p):
    """a ^ b and a .^ b: binds tighter than unary minus (-a^2 is -(a^2)), left to
    right; the exponent may carry a sign (2^-1). The semantics decide which operands are recorded."""
    left = _primary(p)
    while p.peek()[0] == "op" and p.peek()[1] in ("^", ".^"):
        tok = p.next()
        neg = p.accept("op", "-")
        if not neg:
            p.accept("op", "+")
        right = _primary(p)
        if neg:
            right = {"op": "neg", "args": [right]}
        left = {"op": "^", "elementwise": tok[1] == ".^", "args": [left, right], "line": tok[2], "col": tok[3]}
    return left


def _primary(p):
    tok = p.next()
    if tok[0] == "num":
        return {"op": "num", "text": tok[1]}
    if tok[0] == "op" and tok[1] == "(":
        e = _expr(p)
        p.expect("op", ")")
        return e
    if tok[0] == "str":
        _refuse(tok, "strings are only allowed as name / value arguments of fi, numerictype, fimath and cast")
    if tok[0] == "name":
        if p.peek()[1] == "(" and p.peek(1)[1] == ":" and p.peek(2)[1] == ")":
            # x(:) in an expression: all elements; the semantics accept it on a
            # scalar only, where it is x itself
            p.next(), p.next(), p.next()
            return {"op": "var", "name": tok[1], "colon": True, "line": tok[2], "col": tok[3]}
        if tok[1] == "hdlfimath" and p.peek()[1] != "(":
            # a function called without parentheses
            return {"op": "call", "name": "hdlfimath", "args": [], "line": tok[2], "col": tok[3]}
        if p.peek()[1] == "." and p.peek(1)[0] == "name" and p.peek(1)[1] in FI_PROPS:
            p.next()
            pr = p.next()
            return {"op": "prop", "name": pr[1], "base": {"op": "var", "name": tok[1], "line": tok[2], "col": tok[3]},
                    "line": tok[2], "col": tok[3]}
        if p.peek()[1] == "." and p.peek(1)[0] == "name" and p.peek(2)[1] != "(":
            p.next()
            fd = p.next()
            return {"op": "field", "name": fd[1], "base": {"op": "var", "name": tok[1], "line": tok[2],
                                                           "col": tok[3]}, "line": tok[2], "col": tok[3]}
        if p.accept("op", "("):
            args = []
            if not p.accept("op", ")"):
                while True:
                    if p.peek()[0] == "str":
                        s_ = p.next()
                        args.append({"op": "str", "text": s_[1][1:-1], "line": s_[2], "col": s_[3]})
                    else:
                        a_ = _expr(p)
                        if p.accept("op", ":"):
                            # x(a:b) -- a range, kept for indexing a vector; refused elsewhere
                            a_ = {"op": "range", "args": [a_, _expr(p)]}
                            if p.peek()[1] == ":":
                                _refuse(p.peek(), "a stepped range a:s:b as an index (not recorded)")
                        args.append(a_)
                    if p.accept("op", ")"):
                        break
                    p.expect("nl", ",", "',' between arguments")
            return {"op": "call", "name": tok[1], "args": args, "line": tok[2], "col": tok[3]}
        if p.peek()[0] == "op" and p.peek()[1] in ("{", "."):
            _refuse(p.peek(), "%s is not in the supported subset" % _REFUSED_OPS[p.peek()[1]])
        return {"op": "var", "name": tok[1], "line": tok[2], "col": tok[3]}
    if tok[0] == "op" and tok[1] == "[":
        # [a, h(1:3)]: a row of scalars and row vectors separated by commas
        items = []
        if p.accept("op", "]"):
            _refuse(tok, "an empty matrix [] (not recorded)")
        while True:
            items.append(_expr(p))
            if p.accept("op", "]"):
                break
            nx = p.peek()
            if nx[1] == ";":
                _refuse(nx, "a column or matrix [a; b] (recorded: a row [a, b])")
            if not p.accept("nl", ","):
                _refuse(nx, "elements of [ ] separated other than by commas (recorded: [a, b])")
        return {"op": "concat", "args": items, "line": tok[2], "col": tok[3]}
    if tok[0] == "op" and tok[1] in _REFUSED_OPS:
        _refuse(tok, "%s is not in the supported subset" % _REFUSED_OPS[tok[1]])
    _refuse(tok, "unexpected %r in an expression" % tok[1], "BAD_MATLAB")


def _check_calls(ast):
    """Every call is an allowed function or one of the file's own subfunctions; strings only as name / value
    arguments of fi / numerictype / fimath / cast."""
    own = {f["name"] for f in ast["functions"]}
    inputs = set()

    def walk(e):
        if isinstance(e, dict):
            if e.get("op") == "call" and (e["name"] in inputs or e["name"] in varnames and e["name"] not in
                                          ALLOWED_CALLS) and e["name"] not in own:
                # x(k) of an input: indexing a vector port; of a local / persistent
                # vector (h(4), buf(p + 1), h(1:3)); one index or range only
                if len(e["args"]) != 1 or e["args"][0].get("op") == "str":
                    raise MlError("UNSUPPORTED_MATLAB", e["line"], e["col"], "indexing %s: one index only"
                                  % e["name"])
                base = {"op": "var", "name": e["name"], "line": e["line"], "col": e["col"]}
                idx = e["args"]
                e.clear()
                e.update({"op": "index", "base": base, "args": idx, "line": base["line"], "col": base["col"]})
                walk(idx)
                return
            if e.get("op") == "call" and e["name"] in varnames:
                # a variable named like a function shadows it in MATLAB (g = u;
                # g(g) indexes g) -- the call was made instead
                raise MlError("UNSUPPORTED_MATLAB", e["line"], e["col"], "%s is a variable here and a function "
                              "name: indexing a variable is not supported, rename it" % e["name"])
            if e.get("op") == "call":
                if e["name"] not in ALLOWED_CALLS and e["name"] not in own:
                    raise MlError("UNSUPPORTED_MATLAB", e["line"], e["col"], "call of %s is not in the supported "
                                  "subset" % e["name"])
                if e["name"] not in STRING_ARG_CALLS and any(a.get("op") == "str" for a in e["args"]):
                    a = next(a for a in e["args"] if a.get("op") == "str")
                    raise MlError("UNSUPPORTED_MATLAB", a["line"], a["col"], "a string argument to %s" % e["name"])
            for v in e.values():
                walk(v)
        elif isinstance(e, (list, tuple)):
            for v in e:
                walk(v)
    varnames = set()
    for f in ast["functions"]:
        inputs.clear()
        inputs.update(f["inputs"])
        varnames.clear()
        for st in _all_stmts(f["body"]):
            if st["op"] == "assign":
                varnames.add(st["target"].split(".")[0])
            elif st["op"] == "for":
                varnames.add(st["var"])
            elif st["op"] == "persistent":
                varnames.update(n.split(".")[0] for n in st["names"])
        varnames.update(f["outputs"])
        varnames.update(n for n in f["inputs"] if n in own or n in ALLOWED_CALLS)
        walk(f["body"])


# ---------------------------------------------------------------- static validation (no numeric semantics)
def validate(ast):
    """Structural checks of a parsed program, still without any numeric meaning (BAD_MATLAB / UNSUPPORTED_MATLAB with
    the line): every variable is assigned before it is read on every path; every output is assigned; a persistent
    variable is declared before use and initialised under `if isempty(p)`; isempty only takes a persistent variable;
    a loop variable is not assigned inside its loop; subfunction calls have the right number of arguments and return
    one value; no recursion; nothing is assigned to an input. Returns ast."""
    funcs = {f["name"]: f for f in ast["functions"]}

    def calls_of(e, out):
        if isinstance(e, dict):
            if e.get("op") == "call" and e["name"] in funcs:
                out.add(e["name"])
            for v in e.values():
                calls_of(v, out)
        elif isinstance(e, (list, tuple)):
            for v in e:
                calls_of(v, out)
        return out
    graph = {n: calls_of(f["body"], set()) for n, f in funcs.items()}

    def cyc(n, path):
        for m in graph[n]:
            if m in path:
                raise MlError("UNSUPPORTED_MATLAB", funcs[m]["line"], 1, "recursion (%s)" % " -> ".join(path + [m]))
            cyc(m, path + [m])
    for n in funcs:
        cyc(n, [n])
    for f in ast["functions"]:
        _validate_function(_struct_view(f) if f.get("_structs") else f, funcs)
    return ast


def _expr_names(e, out):
    """Variable reads of an expression (the argument of isempty is not a read: it tests whether p is initialised)."""
    if isinstance(e, dict):
        if e.get("op") == "call" and e.get("name") == "isempty":
            return out
        if e.get("op") == "var":
            out.append(e)
        for k, v in e.items():
            if k != "op":
                _expr_names(v, out)
    elif isinstance(e, (list, tuple)):
        for v in e:
            _expr_names(v, out)
    return out


def _persistent_placement(f):
    """MATLAB's placement rule for persistent: a declaration only at the top level of
    the function, and before any other reference to the variable -- a second declaration, an assignment, a read, or
    the variable being an input or output all count as references. MATLAB refuses everything else at compile time
    (inside an if or a loop: "PERSISTENT declarations are only allowed at the top level"; otherwise "variable
    referenced before its persistent declaration")."""
    seen = set(f["inputs"]) | set(f["outputs"])

    def refs(node, out):
        if isinstance(node, dict):
            if node.get("op") == "var":
                out.add(node.get("name"))
            elif node.get("op") == "assign":
                out.add(node.get("target"))
            elif node.get("op") == "for":
                out.add(node.get("var"))
            for k, v in node.items():
                if k != "op":
                    refs(v, out)
        elif isinstance(node, list):
            for v in node:
                refs(v, out)
        return out

    def nested(stmts):
        for st in stmts:
            if st["op"] == "persistent":
                raise MlError("BAD_MATLAB", st["line"], 1, "persistent declarations are only allowed at the top "
                              "level of a function (MATLAB refuses one inside an if or a loop)")
            if st["op"] == "if":
                for _c, b in st["arms"]:
                    nested(b)
                if st.get("else") is not None:
                    nested(st["else"])
            elif st["op"] == "for":
                nested(st["body"])

    def in_order(stmts):
        # every statement in source order, nested ones included: MATLAB reports a reference before the declaration
        # first (at the second declaration), the placement otherwise (x1, x2)
        for st in stmts:
            if st["op"] == "persistent":
                for nm in st["names"]:
                    if nm in seen:
                        raise MlError("BAD_MATLAB", st["line"], 1, "variable %s is referenced before its persistent "
                                      "declaration (a second declaration, an earlier use, or a port name)" % nm)
                    seen.add(nm)
            elif st["op"] == "if":
                for c, b in st["arms"]:
                    refs(c, seen)
                    in_order(b)
                if st.get("else") is not None:
                    in_order(st["else"])
            elif st["op"] == "for":
                seen.add(st["var"])
                in_order(st["body"])
            else:
                refs(st, seen)

    in_order(f["body"])
    nested([st for st in f["body"] if st["op"] != "persistent"])


def _validate_function(f, funcs):
    _persistent_placement(f)
    persistent = set()
    inputs = set(f["inputs"])

    def check_expr(e, defined, line):
        for v in _expr_names(e, []):
            if v["name"] in persistent:
                continue   # whether a persistent variable holds a value here: fxmldefassign
            if v["name"] not in defined and v["name"] not in ("true", "false", "pi"):
                raise MlError("BAD_MATLAB", v.get("line", line), v.get("col", 1), "%s is read before it is assigned"
                              % v["name"])
        for c in _calls(e, []):
            if c["name"] in funcs:
                g = funcs[c["name"]]
                if len(c["args"]) != len(g["inputs"]):
                    raise MlError("BAD_MATLAB", c["line"], c["col"], "%s takes %d arguments, %d given"
                                  % (c["name"], len(g["inputs"]), len(c["args"])))
                if len(g["outputs"]) != 1:
                    raise MlError("UNSUPPORTED_MATLAB", c["line"], c["col"], "a subfunction call in an expression "
                                  "must return exactly one value")
            if c["name"] == "isempty":
                a = c["args"]
                if len(a) != 1 or a[0].get("op") != "var" or a[0]["name"] not in persistent:
                    raise MlError("UNSUPPORTED_MATLAB", c["line"], c["col"], "isempty only of a persistent variable "
                                  "(the register initialisation)")

    def block(stmts, defined, loopvars):
        """Returns the set of names certainly assigned after the block."""
        d = set(defined)
        for s in stmts:
            op = s["op"]
            if op == "persistent":
                for nm in s["names"]:
                    if nm in inputs or nm in f["outputs"]:
                        raise MlError("BAD_MATLAB", s["line"], 1, "%s is a port and cannot be persistent" % nm)
                    persistent.add(nm)
            elif op == "assign":
                if s["target"] in inputs:
                    raise MlError("UNSUPPORTED_MATLAB", s["line"], 1, "assignment to the input %s" % s["target"])
                if s["target"] in loopvars:
                    raise MlError("BAD_MATLAB", s["line"], 1, "assignment to the loop variable %s inside its loop"
                                  % s["target"])
                check_expr(s["expr"], d | persistent_ready(d), s["line"])
                if s["whole"] and s["target"] not in d and s["target"] not in persistent:
                    raise MlError("BAD_MATLAB", s["line"], 1, "%s(:) = ... before %s is assigned" % (s["target"], s["target"]))
                d.add(s["target"])
            elif op == "if":
                outs = []
                for cond, body in s["arms"]:
                    # (where a persistent variable holds a value is decided by fxmldefassign)
                    check_expr(cond, d, s["line"])
                    outs.append(block(body, d, loopvars))
                outs.append(block(s["else"], d, loopvars) if s["else"] is not None else set(d))
                for (cond, body), sub in zip(s["arms"], outs[:len(s["arms"])]):
                    init = _is_isempty(cond)
                    if init and init in sub:
                        initialised.add(init)   # for the "initialised under if isempty(p)" requirement below
                        # the other persistents set in the same first-call arm
                        # (DA decides whether they are held where read)
                        initialised.update(v for v in sub if v in persistent)
                d = set.intersection(*outs)
            elif op == "nop" and s.get("reads"):
                check_expr(s["reads"], d, s["line"])     # a warning message's variables
            elif op == "for":
                d.add(s["var"])
                inner = block(s["body"], d, loopvars | {s["var"]})
                # the body may run zero times only for an empty range; constant bounds tell
                n = None if s["lo"] is None else len(range(s["lo"], s["hi"] + (1 if s["step"] > 0 else -1),
                                                              s["step"]))
                if n:
                    d = inner
        return d

    initialised = set()

    def persistent_ready(d):
        return set()   # a persistent variable is readable only where d says it is assigned
    final = block(f["body"], set(f["inputs"]), set())
    try:
        DA.check(f, persistent)
    except DA.Undefined as e:
        # MATLAB refuses this at compile time, at the first unguarded read ("persistent variable
        # undefined on some execution paths"); recordings: decided by a definite-assignment analysis
        if e.kind == "before_use":   # no path has assigned it yet
            raise MlError("BAD_MATLAB", e.line, e.col, "persistent %s must be assigned before it is used (only an "
                          "isempty(%s) check may come first)" % (e.name, e.name))
        raise MlError("BAD_MATLAB", e.line, e.col, "persistent %s is undefined on some execution paths (initialise "
                      "it under `if isempty(%s)` on every path, or guard the read with ~isempty)" % (e.name, e.name))
    for o in f["outputs"]:
        if o not in final:
            raise MlError("BAD_MATLAB", f["line"], 1, "output %s is not assigned on every path" % o)
    for p in persistent:
        if p not in initialised:
            raise MlError("UNSUPPORTED_MATLAB", f["line"], 1, "persistent %s is not initialised under `if "
                          "isempty(%s)`" % (p, p))


def _calls(e, out):
    if isinstance(e, dict):
        if e.get("op") == "call":
            out.append(e)
        for k, v in e.items():
            if k != "op":
                _calls(v, out)
    elif isinstance(e, (list, tuple)):
        for v in e:
            _calls(v, out)
    return out


def _is_isempty(cond):
    if isinstance(cond, dict) and cond.get("op") == "call" and cond["name"] == "isempty" and len(cond["args"]) == 1 \
            and cond["args"][0].get("op") == "var":
        return cond["args"][0]["name"]
    return None


# ---------------------------------------------------------------- survey: every parse refusal of a script
_BLOCK_KW = {"if": "if", "elseif": "elseif", "while": "if", "switch": "if", "case": "elseif", "for": "if"}


def survey(text, limit=200):
    """Every parse refusal of a script, not only the first (when checking coverage, parse stops at the first refusal,
    so a script shows one construct). After each refusal the statement holding it is neutralised and the script is
    parsed again: a plain statement is dropped; a refused block header keeps its block (`if` / `while` / `switch`
    / `for` header -> `if true`, `elseif` / `case` -> `elseif true`, `otherwise` -> `else`, `catch` dropped, `try`
    -> `if true`). Numbers are approximate after the first entry (a dropped statement can hide or cause a later
    one); the first entry is exactly what parse raises. Returns [{"code", "line", "col", "message"}]; a tokenizer
    or function-header refusal ends the list. Never used to accept anything."""
    out = []
    try:
        toks = tokenize(text)
    except MlError as e:
        return [{"code": e.code, "line": e.line, "col": e.col, "message": e.message}]
    for _ in range(limit):
        try:
            p = _P(toks)
            ast = {"functions": [], "main": None}
            p.skip_nl()
            while p.peek()[0] != "eof":
                ast["functions"].append(_function(p))
                p.skip_nl()
            # the post-parse rewrites refuse too (structs, [..] =): reported, not neutralised (approximate)
            for step in (lambda: _expand_massign(ast),) + tuple((lambda f=f: _desugar_structs(f))
                                                                 for f in ast["functions"]):
                try:
                    step()
                except MlError as e:
                    out.append({"code": e.code, "line": e.line, "col": e.col, "message": e.message})
            _check_calls(ast)
            return out
        except MlError as e:
            out.append({"code": e.code, "line": e.line, "col": e.col, "message": e.message})
            toks = _neutralise(toks, e.line, e.col)
            if toks is None:
                return out
    return out


def _neutralise(toks, line, col):
    """toks with the statement holding the token at (line, col) neutralised (see survey); None if it cannot be."""
    at = next((i for i, t in enumerate(toks) if (t[2], t[3]) >= (line, col)), len(toks) - 1)
    depth, starts = 0, [0]
    for i, t in enumerate(toks):     # statement starts: after a depth-0 separator
        if t[0] == "op" and t[1] in "([{":
            depth += 1
        elif t[0] == "op" and t[1] in ")]}":
            depth = max(0, depth - 1)
        elif t[0] == "nl" and (depth == 0 or t[1] in ("\n", ";")):
            depth = 0
            starts.append(i + 1)
    s = max(x for x in starts if x <= at)
    e = min([x for x in starts if x > s] + [len(toks) - 1])
    while e < len(toks) - 1 and toks[e - 1][0] != "nl":
        e += 1
    head = toks[s]
    if head[0] == "kw" and head[1] == "function":
        return None
    if head[0] == "eof":
        return None
    ln, c = head[2], head[3]
    if head[0] == "kw" and head[1] in _BLOCK_KW:
        rep = [("kw", _BLOCK_KW[head[1]], ln, c), ("name", "true", ln, c + 1), ("nl", "\n", ln, c + 2)]
    elif head[0] == "kw" and head[1] == "otherwise":
        rep = [("kw", "else", ln, c), ("nl", "\n", ln, c + 1)]
    elif head[0] == "kw" and head[1] == "try":
        rep = [("kw", "if", ln, c), ("name", "true", ln, c + 1), ("nl", "\n", ln, c + 2)]
    else:
        rep = []
    if not rep and e <= s:
        return None
    new = toks[:s] + rep + toks[e:]
    return None if new == toks else new

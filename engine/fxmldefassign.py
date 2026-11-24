"""Definite assignment of persistent variables in a MATLAB Function.

At the start of a step a persistent variable p is either empty (the first step) or holds the last step's value; the
program cannot tell which except through isempty(p). A read of p -- or p(:) = ..., which needs p's type -- is
accepted only where p holds a value on every path that reaches it in this step:
  - after p = ... on every path;
  - where a condition proves p non-empty: inside `if ~isempty(p)`, in the arms after an `if isempty(p)` arm (and
    in its else), inside `... && ~isempty(p)`, on the right of `~isempty(p) && ...`;
  - after an if whose every path ends with p holding a value (an `if isempty(p)` arm that assigns p on all of its
    own paths, the other paths holding p because isempty(p) was false).
if / elseif / else merge by intersection, a path with no else counts as an empty else; a for loop (constant range)
with no iteration is skipped, otherwise its body is checked at the loop head's fixed point -- the entry facts
intersected with the body's outcome until stable (an entry fact such as "p is empty" holds on the first
iteration only).
Anything not proven is refused, at the first such read in source order -- MATLAB refuses these at compile time
("persistent variable undefined on some execution paths", recordings, at that read's line and column).

The one place that decides this: fxmlfn.validate calls check(); the interpreter (fxmlsem) and the compiler
(fxmlgen) run only programs that passed it, and the interpreter's own run-time guard refuses structurally."""


class Undefined(Exception):
    """kind: "some_paths" when an assignment to the variable comes earlier in the step (MATLAB:
    "undefined on some execution paths"), "before_use" when none does ("must be assigned before it is used; only an
    isempty check may come first")."""

    def __init__(self, name, line, col, kind="some_paths"):
        Exception.__init__(self, name)
        self.name, self.line, self.col, self.kind = name, line, col, kind


_ASSIGNED = []   # per check: every assignment of the function as (line, col, name), in source order


def _kind(name, line, col):
    """MATLAB's reason: source order, not execution -- "some_paths" when
    an assignment to name appears earlier in the source (any branch, any loop, run or not), else "before_use"."""
    return "some_paths" if any(n == name and (l, c) < (line, col) for l, c, n in _ASSIGNED[-1]) else "before_use"


def _assignments(node, out):
    if isinstance(node, dict):
        if node.get("op") == "assign":
            out.append((node.get("line", 0), node.get("col", 1), node["target"]))
        for k, v in node.items():
            if k != "op":
                _assignments(v, out)
    elif isinstance(node, (list, tuple)):
        for v in node:
            _assignments(v, out)
    return out


def _facts(cond, persistent, held=frozenset()):
    """(set proven when cond is true, set proven when cond is false) of non-empty persistent variables, given held
    (an operand that cannot be false / true given held makes the other operand decide: `~isempty(t) && isempty(s)`
    false with t holding a value proves s non-empty)."""
    if not isinstance(cond, dict):
        return set(), set()
    op = cond.get("op")
    args = cond.get("args", [])
    if op == "call" and cond.get("name") == "isempty" and len(args) == 1 and args[0].get("op") == "var" \
            and args[0]["name"] in persistent:
        return {("empty", args[0]["name"])}, {args[0]["name"]}   # true: p is empty; false: p holds a value
    if op == "not" and len(args) == 1:
        t, f = _facts(args[0], persistent, held)
        return f, t
    if op in ("&&", "||") and cond.get("sc_from"):
        return set(), set()   # a rewritten & / |: nothing proven either way (not recorded; refusing is the safe side)
    if op == "&&" and len(args) == 2:
        ta, fa = _facts(args[0], persistent, held)
        tb, fb = _facts(args[1], persistent, set(held) | ta)
        if _always(args[0], set(held), persistent):
            false = fb          # a never false: the && is false only through b (a's false facts would be vacuous)
        elif _always(args[1], set(held) | ta, persistent):
            false = fa
        else:
            false = fa & fb
        return ta | tb, false
    if op == "||" and len(args) == 2:
        ta, fa = _facts(args[0], persistent, held)
        tb, fb = _facts(args[1], persistent, set(held) | fa)
        if _never(args[0], set(held), persistent):
            true = tb           # a never true: the || is true only through b
        elif _never(args[1], set(held) | fa, persistent):
            true = ta
        else:
            true = ta & tb
        return true, fa | fb
    return set(), set()


def _never(cond, held, persistent):
    """True when cond cannot be true given held: isempty(p) of a p holding a value (e.g. an `elseif isempty(p)`
    inside `if ~isempty(p)`), and the && / || / ~ combinations of such conditions. Such an arm never runs: its
    reads are not checked and it does not take part in the merge (as the path enumeration of the regression check)."""
    if not isinstance(cond, dict):
        return False
    if cond.get("sc_from"):
        return False   # a rewritten & / | decides nothing for the analysis
    op, args = cond.get("op"), cond.get("args", [])
    if op == "call" and cond.get("name") == "isempty" and len(args) == 1 and args[0].get("op") == "var":
        return args[0]["name"] in persistent and args[0]["name"] in held
    if op == "&&" and len(args) == 2:
        return _never(args[0], held, persistent) or _never(args[1], held | _facts(args[0], persistent, held)[0], persistent)
    if op == "||" and len(args) == 2:
        return _never(args[0], held, persistent) and _never(args[1], held | _facts(args[0], persistent, held)[1], persistent)
    if op == "not" and len(args) == 1:
        return _always(args[0], held, persistent)
    return False


def _always(cond, held, persistent):
    """True when cond cannot be false given held (~isempty(p) of a p holding a value, isempty(p) of a p known to be
    empty -- inside `if isempty(p)` before p is assigned --, ...)."""
    if not isinstance(cond, dict):
        return False
    if cond.get("sc_from"):
        return False   # a rewritten & / | decides nothing for the analysis
    op, args = cond.get("op"), cond.get("args", [])
    if op == "call" and cond.get("name") == "isempty" and len(args) == 1 and args[0].get("op") == "var":
        return ("empty", args[0]["name"]) in held
    if op == "not" and len(args) == 1:
        return _never(args[0], held, persistent)
    if op == "&&" and len(args) == 2:
        return _always(args[0], held, persistent) and _always(args[1], held | _facts(args[0], persistent, held)[0],
                                                            persistent)
    if op == "||" and len(args) == 2:
        return _always(args[0], held, persistent) or _always(args[1], held | _facts(args[0], persistent, held)[1],
                                                           persistent)
    return False


def _reads(e, held, persistent):
    """Check the persistent reads of expression e against held (the variables holding a value here); && and ||
    evaluate their right operand only after the left one's outcome."""
    if isinstance(e, dict):
        op = e.get("op")
        if op == "call" and e.get("name") == "isempty":
            return   # isempty(p) tests p, it does not read it
        if op == "var":
            if e.get("name") in persistent and e["name"] not in held:
                raise Undefined(e["name"], e.get("line", 0), e.get("col", 1),
                                _kind(e["name"], e.get("line", 0), e.get("col", 1)))
            return
        if op in ("&&", "||") and len(e.get("args", [])) == 2 and e.get("sc_from"):
            # an & / | of an if condition: it short-circuits when run, but MATLAB's definite assignment reads both
            # operands with nothing proven by the left one
            _reads(e["args"][0], held, persistent)
            _reads(e["args"][1], held, persistent)
            return
        if op in ("&&", "||") and len(e.get("args", [])) == 2:
            a, b = e["args"]
            _reads(a, held, persistent)
            if (op == "&&" and _never(a, held, persistent)) or (op == "||" and _always(a, held, persistent)):
                return   # the right operand is never evaluated (short-circuit decided by known facts)
            t, f = _facts(a, persistent, held)
            _reads(b, held | (t if op == "&&" else f), persistent)
            return
        for k, v in e.items():
            if k != "op":
                _reads(v, held, persistent)
    elif isinstance(e, (list, tuple)):
        for v in e:
            _reads(v, held, persistent)


def _block(stmts, held, persistent):
    """The persistent variables holding a value after stmts on every path, given held before them (held also
    carries ("empty", p) where p is known to be empty)."""
    h = set(held)
    for s in stmts:
        op = s["op"]
        if op == "assign":
            _reads(s["expr"], h, persistent)
            if s["target"] in persistent:
                if s.get("whole") and s["target"] not in h:   # p(:) = ... needs p's type
                    raise Undefined(s["target"], s.get("line", 0), s.get("col", 1),
                                    _kind(s["target"], s.get("line", 0), s.get("col", 1)))
                h.add(s["target"])
                h.discard(("empty", s["target"]))
        elif op == "if":
            outs = []
            falses = set()   # proven by every earlier condition being false
            rest = True      # some path still reaches the next arm
            for cond, body in s["arms"]:
                if not rest:
                    break
                _reads(cond, h | falses, persistent)
                t, f = _facts(cond, persistent, h | falses)
                arm = None
                if not _never(cond, h | falses, persistent):
                    arm = _block(body, h | falses | t, persistent)
                    outs.append(arm)
                if _always(cond, h | falses, persistent):
                    rest = False
                falses |= f
                x = _first_call_check(cond, persistent)
                if x is not None and arm is not None and x in arm and (
                        _defines(_FUNC[-1], x) == _defines(body, x) or _later_only(s, body, x)):
                    # (only when x is assigned nowhere else: an earlier assignment of x would skip this arm on the
                    # first call -- the regression check's enumeration finds such programs)
                    # recordings (recorded, accepted): `if isempty(buf) buf = ..; p = ..; end`
                    # then p read -- isempty(buf) false means a later call, so every persistent the arm assigns
                    # on all its paths holds a value (one assigned only in an
                    # elseif arm is still undefined on the first call, refused)
                    falses |= {v for v in arm if isinstance(v, str) and v in persistent}
            if rest:
                outs.append(_block(s["else"], h | falses, persistent) if s.get("else") is not None else h | falses)
            h = set.intersection(*outs) if outs else h
        elif op == "for":
            if s["lo"] is None:
                # bounds the semantics resolve later: the body may run any number of times, so
                # what holds after the loop is what holds before it and after each iteration
                n = None
            else:
                n = len(range(s["lo"], s["hi"] + (1 if s["step"] > 0 else -1), s["step"]))
            if n == 0:
                continue   # the body never runs (it was checked anyway)
            # what holds at the top of the body must hold on every iteration, not only the first -- a
            # fact ("p is empty") true on entry is gone once an iteration assigns p. The loop head is the fixed
            # point of entry & back edge: start from the entry facts and intersect with the body's outcome until
            # nothing changes (the sets only shrink); the body's reads are checked at that fixed point
            top = set(h)
            while True:
                out = _block_quiet(s["body"], top, persistent)
                nxt = top & out if (n is None or n > 1) else top
                if nxt == top:
                    break
                top = nxt
            h = _block(s["body"], top, persistent)
            if n is None:
                h = h & top     # zero iterations possible: only what held at the loop head
    return h


_FUNC = []   # per check: the function body (for _defines)


def _max_line(node):
    m = 0
    if isinstance(node, dict):
        m = node.get("line", 0) or 0
        for k, v in node.items():
            if k != "op":
                m = max(m, _max_line(v))
    elif isinstance(node, (list, tuple)):
        for v in node:
            m = max(m, _max_line(v))
    return m


def _later_only(ifstmt, body, x):
    """recordings (two persistents initialised under one `if isempty(d1)`, then reassigned later -- a FIR delay line,
    an FSM with a counter; Simulink runs both): the `if isempty(x)` statement is a top-level statement of the
    function and every assignment of x outside its arm comes after the if in the source, so nothing before it can
    give x a value -- on the first call the arm runs and assigns the others too."""
    if not any(st is ifstmt for st in _FUNC[-1]):
        return False
    end = _max_line(ifstmt)
    inside = {(l, c) for l, c, n in _assignments(body, []) if n == x}
    return all(l > end for l, c, n in _ASSIGNED[-1] if n == x and (l, c) not in inside)


def _defines(node, x):
    """How many statements of node give x a value from nothing (x = e; not x(k) = e or x(:) = e, which need x)."""
    n = 0
    if isinstance(node, dict):
        if node.get("op") == "assign" and node.get("target") == x and not node.get("whole") and \
                node.get("index") is None:
            n += 1
        for k, v in node.items():
            if k != "op":
                n += _defines(v, x)
    elif isinstance(node, (list, tuple)):
        for v in node:
            n += _defines(v, x)
    return n


def _first_call_check(cond, persistent):
    """X for a condition isempty(X) of a persistent X, else None."""
    if isinstance(cond, dict) and cond.get("op") == "call" and cond.get("name") == "isempty" and \
            len(cond.get("args", [])) == 1 and cond["args"][0].get("op") == "var" and \
            cond["args"][0]["name"] in persistent:
        return cond["args"][0]["name"]
    return None


def _block_quiet(stmts, held, persistent):
    """_block for the fixed point: the outcome only (a read the weaker fixed point refuses is reported by the
    checking pass); the variables assigned so far are not recorded."""
    _ASSIGNED.append(_ASSIGNED[-1])
    try:
        return _block(stmts, held, persistent)
    except Undefined:
        # a refused read inside: the checking pass at the (weaker) fixed point refuses it too; the outcome then
        # does not matter -- return the strongest assumption so the iteration still shrinks from the entry facts
        return set(held)
    finally:
        _ASSIGNED.pop()


def check(f, persistent):
    """Raise Undefined at the first read of a persistent variable of function f not proven to hold a value."""
    if persistent:
        _ASSIGNED.append(_assignments(f["body"], []))
        _FUNC.append(f["body"])
        try:
            _block(f["body"], set(), set(persistent))
        finally:
            _ASSIGNED.pop()
            _FUNC.pop()

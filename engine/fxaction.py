"""Switch Case action subsystems, Merge and Data Store blocks, lowered to blocks the
engine already runs. Rules from recordings (the native 17 models, every step):

- Switch Case (input int8; CaseConditions a cell of integers or integer vectors, e.g. '{1, [2 3]}'; ShowDefaultCase
  on): output port k is active when the input equals one of case k's values; the default port when none does. Lowered
  to Compare To Constant / Logical Operator blocks, one boolean per port.
- An action subsystem S is marked by a block S/... of type ActionPort (InitializeStates held | reset), driven by a
  Switch Case output; its outputs are EnabledOutport blocks S/... . It runs as an enabled subsystem whose enable is the
  Switch Case port: held -- its states keep their values while it does not run; reset -- they go back to their initial
  values in the step it starts running again (the enabled subsystem's StatesWhenEnabling).
- Merge (Inputs n, InitialOutput 0 or []): each input from an action subsystem of one Switch Case that has a default
  port, the n inputs covering every port of it -- exactly one runs each step, and the Merge passes its output.
  Simulink refuses a Merge fed by a block outside a conditionally executed subsystem
  (Simulink:blocks:InvMergeConnNonCondSubsysSrc); other Merges were not recorded.
- Data Store (one DataStoreMemory with InitialValue and an explicit fixed-point OutDataTypeStr, one DataStoreRead and
  one DataStoreWrite of its DataStoreName): read before write gives the value written in the previous step
  (InitialValue at step 0), write before read the value written in the same step. The order: a data path from the
  read to the write forces read first (Simulink warns UnableToHonorPriorityInTask0 if the Priority says otherwise),
  else the lower Priority runs first; equal or missing priorities were not recorded.
- If: refused, with Switch Case on an integer as the way round. Simulink refuses an If input of a fixdt type
  (Simulink:DataType:InvInputPortDataType); it runs int8 / uint8 / boolean inputs,
  whose If semantics are not recorded here.
"""
import re

import fxgraph_lib as G

INTERNAL = "\x1d"
TYPES = ("SwitchCase", "ActionPort", "Merge", "DataStoreMemory", "DataStoreRead", "DataStoreWrite", "If",
         "IfActionPort", "SwitchCaseActionPort")
PARAMS = {"SwitchCase": {"CaseConditions": "{1}", "ShowDefaultCase": "on"},
          "ActionPort": {"InitializeStates": "held"},
          "Merge": {"Inputs": "2", "InitialOutput": "[]"},
          "DataStoreMemory": {"DataStoreName": "A", "InitialValue": "0", "OutDataTypeStr": None},
          "DataStoreRead": {"DataStoreName": "A", "Priority": "", "IndexOptions": "Index vector (dialog)",
                            "Indices": "1", "OutputSizes": "1"},
          "DataStoreWrite": {"DataStoreName": "A", "Priority": "", "IndexOptions": "Index vector (dialog)",
                             "Indices": "1", "OutputSizes": "1"}}
# a read / write of the whole store only (the saved models' defaults); element selection not recorded
DS_WHOLE = {"IndexOptions": "Index vector (dialog)", "Indices": "1", "OutputSizes": "1"}


def has(spec):
    return isinstance(spec, dict) and isinstance(spec.get("blocks"), dict) and any(
        isinstance(b, dict) and b.get("type") in TYPES for b in spec["blocks"].values())


def _refuse(where, msg, code="UNSUPPORTED"):
    raise G.GraphError(code, where, msg)


def _params(n, b):
    t = b["type"]
    p = dict(PARAMS[t])
    for k, v in b.items():
        if k == "type":
            continue
        if k == "Priority" and t in ("DataStoreRead", "DataStoreWrite") and \
                not re.fullmatch(r"[+-]?[0-9]+", str(v).strip()) and str(v).strip() != "":
            # Simulink rejects the value itself (PriorityErr: '' or an integer literal)
            _refuse("design.blocks.%s.%s" % (n, k), "Priority %r: Simulink rejects it (PriorityErr: an empty value or "
                    "an integer literal)" % str(v).strip(), "BAD_PARAM")
        if k not in p and G.DESIGN_RANGE_RE.match(k) and t != "DataStoreMemory":
            # Data Store Read / Write and Merge have no OutMin / OutMax (Simulink set_param fails)
            _refuse("design.blocks.%s.%s" % (n, k), "%s has no parameter %s (Simulink rejects it)" % (t, k),
                    "BAD_PARAM")
        if k not in p and G.DESIGN_RANGE_RE.match(k) and str(v).strip() not in ("[]", ""):
            # a real parameter (Data Store Memory OutMin / OutMax), not an unknown one
            _refuse("design.blocks.%s.%s" % (n, k), "design range %s = %r is not supported: Simulink uses design "
                    "minimum / maximum values to choose types and to check parameters, which this engine does not "
                    "model; leave it '[]'" % (k, str(v).strip()))
        if k not in p and G.DESIGN_RANGE_RE.match(k):
            continue   # '[]': the library value
        if k not in p:
            _refuse("design.blocks.%s.%s" % (n, k), "unknown parameter %s for %s (accepted: %s)"
                    % (k, t, ", ".join(sorted(PARAMS[t]))), "BAD_PARAM")
        p[k] = str(v).strip()
    return p


def _cases(text, where):
    t = text.strip()
    if not (t.startswith("{") and t.endswith("}")):
        _refuse(where, "CaseConditions must be a cell array of integers, e.g. '{1, [2 3]}'", "BAD_PARAM")
    body, out, k = t[1:-1].strip(), [], 0
    if re.search(r"[;:]", body) and re.fullmatch(r"[-\d\s,;:\[\]]*", body):
        # recordings: '{[1;2], 3}', '{1:3, 4}' are valid CaseConditions (Simulink runs them); only the forms
        # '{1, [2 3]}' are read here -- not supported, not a malformed request
        _refuse(where, "CaseConditions %r: column vectors (;) and ranges (:) are valid in Simulink but not read here; "
                "write each case as an integer or a row of integers, e.g. '{[1 2], 3}'" % text)
    for m in re.finditer(r"\[([^\]]*)\]|(-?\d+)", body):
        vals = m.group(1).replace(",", " ").split() if m.group(1) is not None else [m.group(2)]
        try:
            out.append([int(v) for v in vals])
        except ValueError:
            _refuse(where, "case values must be integers (recorded: '{1, [2 3]}')", "BAD_PARAM")
    rest = re.sub(r"\[[^\]]*\]|-?\d+|,|\s", "", body)
    if rest or not out or any(not c for c in out):
        _refuse(where, "CaseConditions must be a cell array of integers or integer vectors, e.g. '{1, [2 3]}'",
                "BAD_PARAM")
    flat = [v for c in out for v in c]
    if len(flat) != len(set(flat)):
        # Simulink rejects the parameter itself (set_param: InvCaseDuplicateConditions)
        _refuse(where, "a value in two cases: Simulink rejects these CaseConditions (InvCaseDuplicateConditions)",
                "BAD_PARAM")
    return out


NO_FEEDTHROUGH = ("UnitDelay", "Delay", "Memory", "DiscreteIntegrator", "TappedDelay")


def _reaches(lines, a, b, blocks=None):
    """True when a data path leads from block a to block b (with blocks: a path with direct feedthrough only -- one
    through a Unit Delay does not order the two, recordings)."""
    nxt = {}
    for ln in lines:
        nxt.setdefault(ln["src"], []).append(ln["dst"])
    seen, stack = set(), [a]
    while stack:
        x = stack.pop()
        if x == b:
            return True
        if x in seen:
            continue
        seen.add(x)
        if blocks is not None and x != a and isinstance(blocks.get(x), dict) and \
                blocks[x].get("type") in NO_FEEDTHROUGH:
            continue
        stack += nxt.get(x, [])
    return False


def rewrite(spec, info):
    """spec with the recordings blocks lowered; type checks queued in info["action_checks"]."""
    blocks = dict(spec["blocks"])
    lines = [dict(ln) for ln in spec.get("lines", [])]
    checks = info.setdefault("action_checks", [])
    for n, b in blocks.items():
        t = b.get("type") if isinstance(b, dict) else None
        if t == "If":
            # Simulink runs an If on int8 / uint8 / boolean inputs, so the refusal names only what
            # is known -- a fixed-point (fixdt) input is refused by Simulink, the If's own semantics are not
            # simulated here
            _refuse("design.blocks." + n, ('If block: not simulated by fxsim (Simulink refuses an If input '
                'of a fixdt type -- Simulink:DataType:InvInputPortDataType -- '
                'and its behaviour on integer / boolean inputs is not recorded); '
                'branch with a Switch Case on an integer signal instead'))
        if t in ("IfActionPort", "SwitchCaseActionPort"):
            _refuse("design.blocks.%s.type" % n, "write an action subsystem's port as type ActionPort", "BAD_PARAM")
    into = {}
    for ln in lines:
        into[(ln["dst"], int(ln.get("dport", 1)))] = ln
    # Switch Case: one boolean per output port
    sc_ports = {}   # switch case -> [boolean block per output port]
    for n, b in list(blocks.items()):
        if b.get("type") != "SwitchCase":
            continue
        p = _params(n, b)
        where = "design.blocks." + n
        cases = _cases(p["CaseConditions"], where + ".CaseConditions")
        if any(not -128 <= v <= 127 for c in cases for v in c):
            # recordings (helper D6b): a case value outside the int8 input's range was refused through the internal
            # comparison as a Simulink parameter overflow, but Simulink runs it (the case never matches)
            _refuse(where + ".CaseConditions", "a case value outside the int8 range -128 .. 127: Simulink runs it (the "
                    "case never matches) but this was not recorded -- not computed")
        if p["ShowDefaultCase"] != "on":
            _refuse(where + ".ShowDefaultCase", "a Switch Case without its default port was not recorded "
                    "(ShowDefaultCase on)")
        src = into.get((n, 1))
        if src is None:
            _refuse(where, "input port 1 is not connected", "BAD_CONNECTION")
        checks.append(("switch_case", n, (src["src"], int(src.get("sport", 1)))))
        X = INTERNAL + n + "/"
        outs = []
        for i, vals in enumerate(cases):
            terms = []
            for j, v in enumerate(vals):
                c = X + "c%dv%d" % (i + 1, j + 1)
                blocks[c] = {"type": "CompareToConstant", "relop": "==", "const": str(v), "OutDataTypeStr": "boolean"}
                lines.append({"src": src["src"], "sport": int(src.get("sport", 1)), "dst": c, "dport": 1})
                terms.append(c)
            o = X + "case%d" % (i + 1)
            blocks[o] = {"type": "LogicalOperator", "Operator": "OR", "Inputs": str(max(2, len(terms)))}
            for k in range(max(2, len(terms))):
                lines.append({"src": terms[min(k, len(terms) - 1)], "dst": o, "dport": k + 1})
            outs.append(o)
        d = X + "default"
        blocks[d] = {"type": "LogicalOperator", "Operator": "NOR", "Inputs": str(max(2, len(outs)))}
        for k in range(max(2, len(outs))):
            lines.append({"src": outs[min(k, len(outs) - 1)], "dst": d, "dport": k + 1})
        outs.append(d)
        sc_ports[n] = outs
        del blocks[n]
    lines = [ln for ln in lines if not (ln["dst"] in sc_ports)]
    # action subsystems -> enabled subsystems driven by the Switch Case booleans
    action = {}   # subsystem -> (switch case, port)
    for n, b in list(blocks.items()):
        if b.get("type") != "ActionPort":
            continue
        p = _params(n, b)
        where = "design.blocks." + n
        if "/" not in n:
            _refuse(where, "an ActionPort must sit inside a subsystem level (S/Action)", "BAD_PARAM")
        if p["InitializeStates"] not in ("held", "reset"):
            _refuse(where + ".InitializeStates", "held or reset", "BAD_PARAM")
        ln = into.get((n, 1))
        if ln is None or ln["src"] not in sc_ports:
            _refuse(where, "an action subsystem must be driven by a Switch Case output (an If is not supported: see "
                    "the If block's refusal)")
        k = int(ln.get("sport", 1))
        if not 1 <= k <= len(sc_ports[ln["src"]]):
            _refuse("design.lines", "output port %d of Switch Case %s does not exist" % (k, ln["src"]),
                    "BAD_CONNECTION")
        S = n.rsplit("/", 1)[0]
        if S in action:
            _refuse(where, "two ActionPort blocks in %s" % S, "BAD_PARAM")
        action[S] = (ln["src"], k)
        blocks[n] = {"type": "EnablePort", "StatesWhenEnabling": p["InitializeStates"]}
    for ln in lines:
        s = ln.get("src")
        if s in sc_ports:
            k = int(ln.get("sport", 1))
            if not 1 <= k <= len(sc_ports[s]):
                _refuse("design.lines", "output port %d of Switch Case %s does not exist" % (k, s), "BAD_CONNECTION")
            if blocks.get(ln["dst"], {}).get("type") != "EnablePort":
                _refuse("design.lines", "Switch Case %s output %d drives %s: a Switch Case output drives an action "
                        "subsystem's ActionPort only" % (s, k, ln["dst"]), "BAD_CONNECTION")
            ln["src"] = sc_ports[s][k - 1]
            ln.pop("sport", None)
    # Merge
    for n, b in list(blocks.items()):
        if b.get("type") != "Merge":
            continue
        p = _params(n, b)
        where = "design.blocks." + n
        try:
            m = int(p["Inputs"])
        except ValueError:
            m = 0
        if m < 2:
            _refuse(where + ".Inputs", "an integer of at least 2", "BAD_PARAM")
        if p["InitialOutput"] not in ("0", "[]"):
            _refuse(where + ".InitialOutput", "InitialOutput 0 or [] (recorded; with a default case it is never "
                    "used)")
        for ln in lines:
            if ln.get("dst") == n:
                s_, sp_ = ln["src"], int(ln.get("sport", 1))
                other = [x["dst"] for x in lines if x.get("src") == s_ and int(x.get("sport", 1)) == sp_ and
                         x.get("dst") != n]
                if other and isinstance(blocks.get(s_), dict) and blocks[s_].get("type") == "EnabledOutport":
                    # (an input from outside an action subsystem is refused below: InvMergeConnNonCondSubsysSrc)
                    # a signal into a Merge input may not branch to another block (Simulink:
                    # InvMergeConnMultDst); it was answered
                    _refuse(where, "Merge %s input from %s also feeds %s: Simulink rejects a Merge input that branches "
                            "(InvMergeConnMultDst)" % (n, s_, ", ".join(sorted(other))), "PORT_TYPE_INVALID")
        srcs, owners = [], []
        for k in range(1, m + 1):
            ln = into.get((n, k))
            if ln is None:
                # Simulink grounds an unconnected Merge input and runs (it was called a request
                # error)
                _refuse(where, "Merge input port %d is not connected; Simulink grounds an unconnected input and runs, "
                        "which is not supported here -- connect every Merge input" % k)
            s = ln["src"]
            S = s.rsplit("/", 1)[0] if "/" in s else None
            if blocks.get(s, {}).get("type") != "EnabledOutport" or S not in action:
                _refuse(where, "Merge input %d comes from %s, not from an action subsystem's output: Simulink "
                        "refuses a Merge fed from outside a conditionally executed subsystem "
                        "(Simulink:blocks:InvMergeConnNonCondSubsysSrc)" % (k, s), "TYPE_MISMATCH")
            if str(blocks[s].get("OutputWhenDisabled", "held")).strip() != "held":
                # recordings (helper D4): Simulink refuses a Merge fed by an output reset when disabled
                _refuse(where, "Merge input %d comes from %s, whose OutputWhenDisabled is %r: Simulink refuses a "
                        "Merge fed by an output that is reset when its subsystem does not run "
                        "(Simulink:blocks:InvMergeConnSrcResetOnDisable); use 'held'"
                        % (k, s, blocks[s].get("OutputWhenDisabled")), "TYPE_MISMATCH")
            srcs.append(s)
            owners.append(action[S])
        scs = {o[0] for o in owners}
        ports = sorted(o[1] for o in owners)
        if len(scs) != 1 or ports != list(range(1, len(sc_ports[next(iter(scs))]) + 1)):
            _refuse(where, "a Merge whose inputs do not cover every port (default included) of one Switch Case: some "
                    "step runs none or two of them -- not recorded (recordings cover one Switch Case with "
                    "its default)")
        sc = next(iter(scs))
        act = [sc_ports[sc][o[1] - 1] for o in owners]
        checks.append(("merge", n, srcs))
        lines = [ln for ln in lines if ln["dst"] != n]
        X = INTERNAL + n + "/"
        prev = srcs[-1]
        for k in range(m - 2, -1, -1):
            sw = n if k == 0 else X + "sel%d" % (k + 1)
            blocks[sw] = {"type": "Switch", "Criteria": "u2 ~= 0", "OutDataTypeStr": "Inherit: Same as first input"}
            lines.append({"src": srcs[k], "dst": sw, "dport": 1})
            lines.append({"src": act[k], "dst": sw, "dport": 2})
            lines.append({"src": prev, "dst": sw, "dport": 3})
            prev = sw
    # Data stores
    stores = {}
    for n, b in blocks.items():
        if b.get("type") in ("DataStoreMemory", "DataStoreRead", "DataStoreWrite"):
            p = _params(n, b)
            stores.setdefault(p["DataStoreName"], {}).setdefault(b["type"], []).append((n, p))
    # recordings (silent wrong, helper D1-D3): a Data Store read / write inside an action or enabled subsystem was run
    # as if it executed every step, with the Priority compared across subsystem levels (D1a: Simulink r = 192, 6393,
    # 6393, 6393; engine 192, 6393, 2958, -5541). Only stores whose blocks all sit outside conditionally executed
    # subsystems, at one level, were recorded -- the rest is refused
    conditional = {n.rsplit("/", 1)[0] for n, b in blocks.items() if isinstance(b, dict) and "/" in n
                   and b.get("type") in ("EnablePort", "ActionPort", "TriggerPort", "StateControl")}
    for name, parts in stores.items():
        mem, rd, wr = parts.get("DataStoreMemory", []), parts.get("DataStoreRead", []), parts.get("DataStoreWrite", [])
        where = "design.blocks." + (mem or rd or wr)[0][0]
        allb = [x for lst in (mem, rd, wr) for x, _ in lst]
        for x in allb:
            if any(x.startswith(c + "/") for c in conditional):
                _refuse("design.blocks." + x, "data store %r: a Data Store block inside an action or enabled "
                        "subsystem runs only in the steps that subsystem runs; not recorded -- keep the "
                        "Data Store Memory, Read and Write outside conditionally executed subsystems" % name)
        if len(mem) > 1:
            # two stores of one name (one per model instance, or a parent store next to a
            # referenced model's own) -- separate stores in Simulink; the inlined read sees one name twice
            _refuse(where, "data store %r: %d Data Store Memory blocks of that name (%s) -- separate stores (one per "
                    "scope or model instance); only one store of a name was recorded" % (name, len(mem), ", ".join(
                        x for x, _ in mem)))
        if len({x.rsplit("/", 1)[0] if "/" in x else "" for x in allb}) != 1:
            _refuse(where, "data store %r: its Memory, Read and Write sit at different subsystem levels; only one "
                    "level was recorded (Priority is not compared across levels)" % name)
        if len(mem) != 1:
            _refuse(where, "data store %r: %d Data Store Memory blocks (one required)" % (name, len(mem)), "BAD_PARAM")
        if len(rd) != 1 or len(wr) != 1:
            _refuse(where, 'data store %r with %d reads and %d writes: one read and one write recorded'
                    "" % (name, len(rd), len(wr)))
        (mn, mp), (rn, rp), (wn, wp) = mem[0], rd[0], wr[0]
        for bn_, bp_ in ((rn, rp), (wn, wp)):
            for k_, v_ in DS_WHOLE.items():
                if bp_[k_] != v_:
                    _refuse("design.blocks.%s.%s" % (bn_, k_), "data store element selection (%s %r) was not "
                            "recorded: read / write the whole store" % (k_, bp_[k_]))
        if not mp["InitialValue"].strip().strip("[]").strip():
            # an empty initial value -- refused at the store (it pointed at the lowered read)
            _refuse("design.blocks.%s.InitialValue" % mn, "data store %r with an empty initial value %r: a numeric "
                    "initial value was recorded" % (name, mp["InitialValue"]))
        if len(mp["InitialValue"].strip().strip("[]").replace(",", " ").replace(";", " ").split()) > 1:
            # a vector store (Simulink runs it); the refusal came from the lowered Unit Delay
            _refuse("design.blocks.%s.InitialValue" % mn, "data store %r with the initial value %s (a vector store): "
                    "only a scalar store was recorded" % (name, mp["InitialValue"]))
        if not mp["OutDataTypeStr"] or not mp["OutDataTypeStr"].startswith("fixdt("):
            _refuse("design.blocks.%s.OutDataTypeStr" % mn, "give the data store an explicit fixdt type (recorded: "
                    "fixdt(1,16,8))")
        wl = into.get((wn, 1))
        if wl is None:
            _refuse("design.blocks." + wn, "input port 1 is not connected", "BAD_CONNECTION")
        if _reaches(lines, rn, wn, blocks):
            read_first = True    # the data path decides (Simulink warns if the Priority says otherwise)
        else:
            # recordings (silent wrong): a path from the read to the write only through a Unit Delay does not
            # order them -- Simulink then runs the write first unless the Priority says the read (recorded: no
            # priority and read 2 / write 1 write first, read 1 / write 2 read first); the Priority decides, as with
            # no path (an unprioritised one of these is refused below)
            try:
                pr, pw = int(rp["Priority"]), int(wp["Priority"])
            except ValueError:
                pr = pw = None
            if pr is None or pr == pw:
                _refuse("design.blocks.%s.Priority" % rn, "data store %r: with no direct-feedthrough data path from "
                        "the read to the write (none, or one through a Unit Delay), give the read and the write "
                        "different Priority values (the lower runs first); equal or missing priorities were not "
                        "recorded" % name)
            read_first = pr < pw
        checks.append(("data_store", wn, (wl["src"], int(wl.get("sport", 1))), mp["OutDataTypeStr"]))
        if read_first:
            blocks[rn] = {"type": "UnitDelay", "InitialCondition": mp["InitialValue"]}
            # the written signal converted to the store type first (a no-op: the write's input has
            # the store type, checked below) -- the delay of a pure loop read -> Unit Delay -> write had no type
            # source (an inheritance cycle) although the store's type is explicit
            cv_ = INTERNAL + rn + "/store"
            blocks[cv_] = {"type": "DataTypeConversion", "OutDataTypeStr": mp["OutDataTypeStr"], "RndMeth": "Floor",
                           "SaturateOnIntegerOverflow": "off"}
            lines.append({"src": wl["src"], "sport": int(wl.get("sport", 1)), "dst": cv_, "dport": 1})
            lines.append({"src": cv_, "dst": rn, "dport": 1})
            del blocks[mn], blocks[wn]
            lines = [ln for ln in lines if ln["dst"] != wn]
            continue
        else:
            blocks[rn] = {"type": "DataTypeConversion", "OutDataTypeStr": mp["OutDataTypeStr"], "RndMeth": "Floor",
                          "SaturateOnIntegerOverflow": "off"}
        lines.append({"src": wl["src"], "sport": int(wl.get("sport", 1)), "dst": rn, "dport": 1})
        del blocks[mn], blocks[wn]
        lines = [ln for ln in lines if ln["dst"] != wn]
    for ln in lines:
        if ln.get("sport") == 1:
            ln.pop("sport")
    return dict(spec, blocks=blocks, lines=lines)


def post_compile(d, info, port_block, type_str):
    """The type checks the rules were recorded with."""
    for c in info.get("action_checks", []):
        if c[0] == "switch_case":
            s = port_block(info, c[2][0], c[2][1], "design.blocks." + c[1])
            if d.otype.get(s) != (1, 8, 0):
                _refuse("design.blocks." + c[1], 'Switch Case input of type %s: recorded with int8 only'
                        "" % (type_str(d, s)))
        elif c[0] == "merge":
            ts = {d.otype.get(port_block(info, s, 1, "design.blocks." + c[1])) for s in c[2]}
            if len(ts) != 1:
                _refuse("design.blocks." + c[1], "Merge inputs of different types (not recorded)")
        elif c[0] == "data_store":
            s = port_block(info, c[2][0], c[2][1], "design.blocks." + c[1])
            want = G._fixdt(c[3], "design.blocks.%s.OutDataTypeStr" % c[1])
            if d.otype.get(s) != want:
                _refuse("design.blocks." + c[1], "Data Store Write input of type %s into a %s store: a conversion "
                        "on writing was not recorded -- convert it first" % (type_str(d, s), c[3]))

"""Triggered and resettable subsystems (high-frequency partition recordings), lowered to blocks the engine already runs.
Rules from recordings (108 models: 72 that Simulink runs, every step and type
equal after this lowering):

- Edge rule (TriggerType / ResetTriggerType rising | falling | either, on an int8 control): rising when the control
  goes from negative to zero or positive, or from zero to positive -- except that a zero reached from a negative value
  in the step before does not rise again (-1, 0, 1 is one edge; -1, 0, 0, 1 is two); falling is the mirror; either is
  both; no edge at step 0. That is exactly the engine's Delay ExternalReset edge detection (recorded in recordings), so
  the edge pulse is a length-1 Delay with IC 1 on a constant 0 whose ExternalReset is the trigger type, ANDed with
  "not the first step".
- A triggered subsystem S (a block S/... of type TriggerPort, driven by the control) runs once in each trigger step:
  an enabled subsystem with that pulse as its enable, StatesWhenEnabling 'held'; its outputs are EnabledOutport blocks
  S/... with OutputWhenDisabled 'held' (they hold between triggers; a state output shows the value from before the
  update).
- A resettable subsystem S (a block S/... of type ResetPort) runs every step; in a reset step its Unit Delays start
  from their initial condition: each becomes a length-1 Delay whose ExternalReset is the reset type, reset by the
  control.

Recorded and answered only: one TriggerPort or ResetPort per subsystem level (no EnablePort with it, no nesting);
inside, Sum / Gain / Unit Delay blocks (and EnabledOutport for a triggered subsystem); the control an int8 signal;
single-rate designs. Everything else is refused with what recording would open it. (Fully inherited types in the
recorded loop were rejected by Simulink -- InputPortDataTypeMismatch -- and are refused by the engine's own typing.)
"""
import fxgraph_lib as G

INTERNAL = "\x1d"
TYPES = ("TriggerPort", "ResetPort")
PARAMS = {"TriggerPort": {"TriggerType": "rising", "StatesWhenEnabling": "held"}, "ResetPort": {"ResetTriggerType": "rising"}}
EDGES = {"rising": "Rising", "falling": "Falling", "either": "Either"}
INNER = {"TriggerPort": ("Sum", "Gain", "UnitDelay", "EnabledOutport"), "ResetPort": ("Sum", "Gain", "UnitDelay")}
REC = "recordings"


def has(spec):
    return isinstance(spec, dict) and isinstance(spec.get("blocks"), dict) and any(
        isinstance(b, dict) and b.get("type") in TYPES for b in spec["blocks"].values())


def _refuse(where, msg, code="UNSUPPORTED"):
    raise G.GraphError(code, where, msg)


def rewrite(spec, info):
    blocks = dict(spec["blocks"])
    lines = [dict(ln) for ln in spec.get("lines", [])]
    ports = [(n, b) for n, b in blocks.items() if isinstance(b, dict) and b.get("type") in TYPES]
    seen = {}
    for n, b in ports:
        t = b["type"]
        where = "design.blocks." + n
        if "/" not in n:
            _refuse(where, "a %s must sit inside a subsystem level (S/Trigger, S/Reset)" % t, "BAD_PARAM")
        extra = sorted(set(b) - {"type"} - set(PARAMS[t]))
        if extra:
            _refuse(where + "." + extra[0], "unknown parameter %s for %s (accepted: %s)"
                    % (extra[0], t, ", ".join(PARAMS[t])), "BAD_PARAM")
        if t == "TriggerPort" and str(b.get("StatesWhenEnabling", "held")).strip() != "held":
            _refuse(where + ".StatesWhenEnabling", "TriggerPort StatesWhenEnabling %r: 'held' was recorded (%s)"
                    % (b.get("StatesWhenEnabling"), REC))
        key = next(iter(PARAMS[t]))
        kind = str(b.get(key, PARAMS[t][key])).strip()
        if kind not in EDGES:
            _refuse(where + "." + key, "%s %s %r: rising, falling and either were recorded (%s); %s"
                    % (t, key, kind, REC, "function-call triggers are not supported" if kind == "function-call"
                       else "record this one to open it"))
        S = n.rsplit("/", 1)[0]
        if S in seen:
            _refuse(where, "two trigger / reset ports in %s: one per subsystem was recorded" % S, "BAD_PARAM")
        seen[S] = (n, t, kind)
        inner = [x for x in blocks if x.startswith(S + "/") and x != n]
        for x in inner:
            bt = blocks[x].get("type") if isinstance(blocks[x], dict) else None
            if "/" in x[len(S) + 1:]:
                _refuse("design.blocks." + x, "a subsystem nested in the %s subsystem %s: not recorded"
                        % ("triggered" if t == "TriggerPort" else "resettable", S))
            if bt not in INNER[t]:
                _refuse("design.blocks." + x, "%s inside the %s subsystem %s: recorded with %s only (%s); record "
                        "this block there to open it" % (bt, "triggered" if t == "TriggerPort" else "resettable", S,
                                                          ", ".join(INNER[t]), REC))
            if str(blocks[x].get("SampleTime", "-1")).strip() not in ("-1", ""):
                _refuse("design.blocks.%s.SampleTime" % x, "a block with its own SampleTime inside the subsystem %s: "
                        "not recorded (blocks there inherit)" % S)
            if bt == "UnitDelay" and len(str(blocks[x].get("InitialCondition", "0")).strip().strip("[]").replace(
                    ",", " ").replace(";", " ").split()) > 1:
                # recordings (d): a vector initial condition (Simulink runs it) was refused at an internal name
                _refuse("design.blocks.%s.InitialCondition" % x, "a Unit Delay with the initial condition %s in the "
                        "subsystem %s: one scalar value was recorded (%s); record this one to open it"
                        % (blocks[x].get("InitialCondition"), S, REC))
            if bt == "EnabledOutport":
                if str(blocks[x].get("OutputWhenDisabled", "held")).strip() != "held":
                    # recordings (c): Simulink accepts 'reset' here and runs; not recorded
                    _refuse("design.blocks.%s.OutputWhenDisabled" % x, "a triggered subsystem's output with "
                            "OutputWhenDisabled %r: 'held' was recorded (%s); record this one to open it"
                            % (blocks[x].get("OutputWhenDisabled"), REC))
                if str(blocks[x].get("InitialOutput", "[]")).strip() not in ("[]", "0", ""):
                    _refuse("design.blocks.%s.InitialOutput" % x, "InitialOutput %s: only the default (0) was "
                            "recorded (%s)" % (blocks[x].get("InitialOutput"), REC))
        if any(isinstance(blocks.get(x), dict) and blocks[x].get("type") in ("EnablePort", "ActionPort")
               for x in inner):
            _refuse(where, "a trigger / reset port together with an enable or action port: not recorded")
        src = [ln for ln in lines if ln.get("dst") == n]
        if len(src) != 1:
            _refuse(where, "the %s takes exactly one control line" % t, "BAD_CONNECTION")
        info.setdefault("trigger_ctrl", []).append((n, (src[0]["src"], int(src[0].get("sport", 1)))))
        # internal names outside the subsystem's path (a name under S/ would make the detector part of S)
        pre = INTERNAL + "trg_" + n.replace("/", "_") + "_"
        if t == "TriggerPort":
            # the edge pulse: Delay(1, IC 1) of a constant 0 reset by the control's edges, and not at step 0
            blocks[pre + "z0"] = {"type": "Constant", "Value": "0", "OutDataTypeStr": "uint8"}
            blocks[pre + "one"] = {"type": "Constant", "Value": "1", "OutDataTypeStr": "uint8"}
            blocks[pre + "first"] = {"type": "UnitDelay", "InitialCondition": "0"}
            blocks[pre + "edge"] = {"type": "Delay", "DelayLength": "1", "InitialCondition": "1",
                                    "ExternalReset": EDGES[kind]}
            info.setdefault("trig_delays", set()).add(pre + "edge")   # (not a user Delay: the port-Delay gate skips it)
            blocks[pre + "e"] = {"type": "LogicalOperator", "Operator": "AND", "Inputs": "2"}
            lines = [ln for ln in lines if ln.get("dst") != n]
            lines += [{"src": pre + "z0", "dst": pre + "edge"},
                      dict({"src": src[0]["src"], "dst": pre + "edge", "dport": 2},
                           **({"sport": src[0]["sport"]} if "sport" in src[0] else {})),
                      {"src": pre + "one", "dst": pre + "first"},
                      {"src": pre + "edge", "dst": pre + "e"}, {"src": pre + "first", "dst": pre + "e", "dport": 2},
                      {"src": pre + "e", "dst": n}]
            blocks[n] = {"type": "EnablePort", "StatesWhenEnabling": "held"}
        else:
            lines = [ln for ln in lines if ln.get("dst") != n]
            blocks.pop(n)
            for x in inner:
                if blocks[x].get("type") == "UnitDelay":
                    ic = blocks[x].get("InitialCondition", "0")
                    nb = {"type": "Delay", "DelayLength": "1", "InitialCondition": ic, "ExternalReset": EDGES[kind]}
                    blocks[x] = nb
                    info.setdefault("trig_delays", set()).add(x)   # (the subsystem's state: not a user Delay)
                    lines.append(dict({"src": src[0]["src"], "dst": x, "dport": 2},
                                      **({"sport": src[0]["sport"]} if "sport" in src[0] else {})))
    return dict(spec, blocks=blocks, lines=lines)


def post_compile(d, info, port_block):
    """The control is an int8 signal (recorded)."""
    for n, (s, sp) in info.get("trigger_ctrl", []):
        t = d.otype.get(port_block(info, s, sp, "design.blocks." + n))
        if tuple(t or ()) != (1, 8, 0):
            _refuse("design.blocks." + n, "a trigger / reset control of type %s: an int8 control was recorded (%s); "
                    "convert it to int8 (Data Type Conversion) or record this type to open it"
                    % (d.type_str(port_block(info, s, sp, "design.blocks." + n)) if t else t, REC))
        if getattr(d, "multirate", False):
            _refuse("design.blocks." + n, "a triggered / resettable subsystem in a multirate design: not recorded")

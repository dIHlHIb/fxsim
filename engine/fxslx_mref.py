"""Model Reference blocks in a saved .slx.

A Model block is read as the referenced model's content placed under the block's name (like a subsystem): Simulink
gives the same results bit for bit as the subsystem with the same content (recordings vs
recordings), every instance keeps its own state (two instances, two accumulators, two counters from 0), and
model arguments take the instance's value (k = 0.5 / 0.25, same as the subsystem with the literals).

Differences from a subsystem that the reader enforces:
- every root Inport of the referenced model must declare its type: the referenced model is compiled on its own, and
  an inherited Inport is not given the parent's signal type (Simulink refuses with
  InputPortDataTypeMismatch; the same content as a subsystem is accepted). Refused here (UNSUPPORTED).
- the declared type must equal the type of the signal driving the port (Simulink: InputPortDataTypeMismatch): the
  reader adds a Constant of the declared type and a Data Type Duplicate over it and the driving signal, both under
  the instance's name, so the engine refuses a mismatch with TYPE_MISMATCH.
- a model argument must be given by the instance (an argument left at the referenced model's default is not
  supported); a block parameter equal to the argument's name takes the instance's value, and a mask value naming it
  is evaluated with that value.

The referenced file is supplied by the caller (refs: model name -> bytes, or None if not available)."""

MODEL_P = {"ModelNameDialog", "ModelName", "ModelFile", "ParameterArgumentNames", "ParameterArgumentValues",
           "UsingDefaultArgumentValue", "SimulationMode", "Position", "ZOrder", "ContentPreviewEnabled",
           "ModelReferenceVersion", "PortCounts", "BackgroundColor", "ForegroundColor", "ShowName", "Description"}
LISTS_OK = {"VisibleInports": "on", "OutputPortOutputBusAsStructs": "off", "InputPortLatchByCopyingInsideSignal": "off",
            "InputPortLatchByDelayingOutsideSignal": "off", "OutputPortUnitExprs": "inherit",
            "InputPortUnitExprs": "inherit"}
MAX_DEPTH = 8


def _err(code, where, msg):
    import fxslx_lib as S
    return S.SlxError(code, where, msg)


def _split(text):
    return [x.strip() for x in (text or "").split(",") if x.strip()]


def instance(sys, sid):
    """(referenced model name, {argument: value text}) of Model block sid, after checking its settings."""
    import fxslx_lib as S
    b = sys.blocks[sid]
    where = sys.where(sid)
    p = S._params(b)
    for k, v in p.items():
        if k not in MODEL_P:
            raise _err("UNSUPPORTED_PARAM", where + "." + k, "Model block parameter %s=%r is not supported (not seen "
                       "in the recorded Model Reference models)" % (k, v))
    if p.get("SimulationMode", "Normal") != "Normal":
        raise _err("UNSUPPORTED_PARAM", where + ".SimulationMode", "simulation mode %r (Normal only)"
                   % p["SimulationMode"])
    if any(x.strip() not in ("0", "") for x in str(p.get("UsingDefaultArgumentValue", "0")).split(",")):
        # one flag per argument ('0,0' with two arguments, none using its default)
        raise _err("UNSUPPORTED_PARAM", where + ".UsingDefaultArgumentValue", "an instance leaving a model argument "
                   "at the referenced model's default: not supported -- give every argument a value on the Model block")
    for lst in b.findall("List"):
        want = LISTS_OK.get(lst.get("ListType"))
        if lst.get("ListType") in ("InputPortNames", "OutputPortNames"):
            continue
        for q in lst.findall("P"):
            if want is None or (q.text or "") != want:
                raise _err("UNSUPPORTED_PARAM", where + "." + str(lst.get("ListType")), "Model block port setting "
                           "%s=%r is not supported" % (lst.get("ListType"), q.text))
    name = p.get("ModelNameDialog") or p.get("ModelName") or ""
    name = name.strip()
    if name.endswith(".slx"):
        name = name[:-4]
    if not name or "/" in name or "\\" in name:
        raise _err("BAD_SLX", where + ".ModelNameDialog", "referenced model name %r" % name)
    def _arg_list(key):
        # two or more arguments are saved as an Array of cells (ParameterArgumentNames Array 1*2),
        # not as one comma text -- the text form only was read, so a two-argument instance was refused
        for q in b.findall("P"):
            if q.get("Name") == key:
                arr = q.find("Array")
                if arr is not None:
                    return [(c.text or "").strip() for c in arr.iter() if c is not arr and c.tag in ("Cell", "P")
                            and (c.text or "").strip()]
        return _split(p.get(key))
    names, values = _arg_list("ParameterArgumentNames"), _arg_list("ParameterArgumentValues")
    if len(names) != len(values):
        # an instance leaving an argument at the referenced model's default saves no value for it
        # (Simulink runs it); not supported -- it was called a malformed file
        raise _err("UNSUPPORTED_PARAM", where + ".ParameterArgumentValues", "the instance gives %d of %d model "
                   "argument values (%s): an argument left at the referenced model's default is not supported; give "
                   "every argument a value on the Model block" % (len(values), len(names), ", ".join(names)))
    for v in values:
        if not S._NUM.match(v):
            raise _err("UNSUPPORTED_PARAM", where + ".ParameterArgumentValues", "argument value %r is an expression; "
                       "only numeric literals are supported" % v)
    return name, dict(zip(names, values))


def model_args(m):
    """Argument names a referenced model declares (blockDiagram.json ModelReferenceSettings.ParameterArgumentNames)."""
    import json
    raw = m.members.get("simulink/blockDiagram.json")
    if raw is None:
        return []
    try:
        j = json.loads(raw.decode("utf-8"))
    except ValueError:
        raise _err("BAD_SLX", "simulink/blockDiagram.json", "not valid JSON")
    found = []

    def walk(o):
        if isinstance(o, dict):
            for k, v in o.items():
                if k == "ModelReferenceSettings" and isinstance(v, dict) and isinstance(
                        v.get("ParameterArgumentNames"), str):
                    found.extend(_split(v["ParameterArgumentNames"]))
                walk(v)
        elif isinstance(o, list):
            for v in o:
                walk(v)
    walk(j)
    return sorted(set(found))


def open_model(refs, name, where, chain):
    import fxslx_lib as S
    if name in chain:
        raise _err("UNSUPPORTED", where, "recursive model reference %s" % " -> ".join(chain + [name]))
    if len(chain) >= MAX_DEPTH:
        raise _err("LIMIT_EXCEEDED", where, "model references nested deeper than %d" % MAX_DEPTH)
    data = refs(name) if refs is not None else None
    if data is None:
        raise _err("FILE_NOT_FOUND", where, "referenced model %s.slx is not available (put it next to the model, or "
                   "pass it to the reader)" % name)
    m = S.Model(data)
    solver, step = S._solver(m)
    if solver is not None and solver not in S.SUPPORTED_SOLVERS:
        raise _err("UNSUPPORTED", where, "referenced model %s uses solver %s" % (name, solver))
    m.ref_step = step
    return m


def substitute(own, args, where):
    """Parameters equal to an argument's name take the instance's value; a parameter that uses an argument inside an
    expression is refused (only whole-value use was recorded)."""
    import re
    out = dict(own)
    for k, v in own.items():
        t = (v or "").strip()
        if t in args:
            out[k] = args[t]
        else:
            for a in args:
                if re.search(r"(?<![A-Za-z0-9_])%s(?![A-Za-z0-9_])" % re.escape(a), t):
                    raise _err("UNSUPPORTED_PARAM", where + "." + k, "model argument %s inside the expression %r "
                               "(only a parameter equal to the argument is supported)" % (a, v))
    return out

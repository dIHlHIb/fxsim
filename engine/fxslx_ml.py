"""MATLAB Function blocks in a saved .slx: the SubSystem with SFBlockType 'MATLAB Function' becomes one
MATLABFunction design block (engine/fxhdl.py, semantics engine/fxmlsem.py).

Format (76 R2026b files): simulink/stateflow/machine.xml lists the charts and one instance per
block (instance name = the block's path in the model, chart = the chart id); simulink/stateflow/chart_<id>.xml holds
the script (state/eml/script), the block's fimath settings (eml: emlDefaultFimath, fimathString, treatAsFi,
saturateOnIntegerOverflow -- absent when they have the default value) and one data object per argument
(scope INPUT_DATA / OUTPUT_DATA, dataType). The block's own system holds Inports / Outports named after the arguments,
plus the generated S-Function, Demux and Terminator, which the reader does not use.

Anything else in these parts (other chart types, parameters, data scopes, explicit input types, sizes, a property
not seen in the 76 files) is refused: an unknown setting could change the function's arithmetic."""
CHART_P = {"name", "screen", "viewObj", "toolbarMode", "ssIdHighWaterMark", "decomposition", "type", "chartFileNumber",
           "disableImplicitCasting", "supportVariableSizing",
           # the editor window's screen position, like screen / viewObj --
           # presentation only, no effect on the function's arithmetic
           "windowPosition",
           # the editor's visible region -- presentation only
           "viewLimits",
           # updateMethod DISCRETE (with sampleTime) / CONTINUOUS; checked below
           "updateMethod", "sampleTime"}
CHART_MUST = {"type": "EML_CHART", "decomposition": "CLUSTER_CHART", "disableImplicitCasting": "1"}
EML_P = {"isEML", "script", "editorLayout", "fimathString", "emlDefaultFimath", "treatAsFi",
         "saturateOnIntegerOverflow", "vectorOutputs1D", "fimathForFiConstructors"}
# recordings (mf/): supportVariableSizing (chart; written only when 0) and vectorOutputs1D (eml; written only
# when 1) change no value, type or element -- vectorOutputs1D = 1 makes a vector output 1-D N instead of N x 1, and
# the engine's vectors are 1-D; both accepted with the recorded values 0 / 1 only
PROP_VALUES = {"supportVariableSizing": ("0", "1"), "vectorOutputs1D": ("0", "1"),
               # recordings (15 models): written only for 'MATLAB factory default'; neither setting
               # changed any recorded value (fi of a fixed-point operand keeps its fimath, fi of a constant or an
               # integer rounds Nearest / Saturate either way -- fxmlsem)
               "fimathForFiConstructors": ("FimathMatlabFactoryDefault",)}
DEFAULT_FIMATH = {None: "Same as MATLAB Default", "FimathUserSpecified": "Other:UserSpecified"}
TREAT = {None: "Fixed-point", "FI_AND_INT": "Fixed-point & Integer"}
SAT = {None: "on", "1": "on", "0": "off"}
DATA_P = {"scope", "dataType"}


def _err(code, where, msg):
    import fxslx_lib as S
    return S.SlxError(code, where, msg)


def _ps(el):
    return {p.get("Name"): (p.text or "") for p in el.findall("P")}


def _tree(m, name, where):
    t = m._tree(name)
    if t is None:
        raise _err("BAD_SLX", where, "Stateflow part %s not found" % name)
    return t.getroot()


def block(m, sys, sid, path):
    """Design block for the MATLAB Function block sid of system sys; path: its model path ('Sub/MF')."""
    import fxhdl
    where = sys.where(sid)
    mach = _tree(m, "simulink/stateflow/machine.xml", where)
    inst = [i for i in mach.iter("instance") if _ps(i).get("name") == path]
    if len(inst) != 1:
        raise _err("BAD_SLX", where, "%d Stateflow instances named %r (one expected)" % (len(inst), path))
    cid = _ps(inst[0]).get("chart")
    refs = [c.get("Ref") for c in mach.iter("chart") if c.get("Ref")]
    chart = None
    for ref in refs:
        r = _tree(m, "simulink/stateflow/%s.xml" % ref, where)
        if r.get("id") == cid:
            chart = r
    if chart is None:
        raise _err("BAD_SLX", where, "chart %s of the MATLAB Function not found" % cid)
    cp = _ps(chart)
    for k, v in cp.items():
        if k not in CHART_P:
            raise _err("UNSUPPORTED_PARAM", where + "." + k, "chart property %s=%r is not supported (not seen in "
                       "the recorded MATLAB Function models)" % (k, v))
    for k, ok in PROP_VALUES.items():
        if k in cp and cp[k].strip() not in ok:
            raise _err("UNSUPPORTED_PARAM", where + "." + k, "chart property %s=%r (recorded 0 / 1)" % (k, cp[k]))
    for k, v in CHART_MUST.items():
        if cp.get(k) != v:
            raise _err("UNSUPPORTED_PARAM", where + "." + k, "chart %s=%r (only %r)" % (k, cp.get(k), v))
    # sampleTime is written only with updateMethod DISCRETE (Simulink rejects
    # DISCRETE without one: Stateflow:cdr:SampleUpdateWasSpecifiedButNoSampleTimeProvided); DISCRETE runs at that
    # period (2: half the outputs), CONTINUOUS gave the inherited values in the recorded discrete model. The
    # period goes back to the reader, which requires it to be the model's single rate
    um = cp.get("updateMethod")
    period = None
    if um not in (None, "DISCRETE", "CONTINUOUS"):
        raise _err("UNSUPPORTED_PARAM", where + ".updateMethod", "chart updateMethod=%r (recorded: DISCRETE, "
                   "CONTINUOUS, inherited)" % um)
    if um == "DISCRETE":
        if not cp.get("sampleTime", "").strip():
            raise _err("BAD_PARAM", where + ".sampleTime", "chart update DISCRETE without a sample time (Simulink: "
                       "SampleUpdateWasSpecifiedButNoSampleTimeProvided)")
        period = cp["sampleTime"].strip()
    elif "sampleTime" in cp:
        raise _err("UNSUPPORTED_PARAM", where + ".sampleTime", "chart sampleTime=%r with updateMethod %r (not "
                   "recorded: Simulink writes it only with DISCRETE)" % (cp["sampleTime"], um or "INHERITED"))
    kids = chart.find("Children")
    states = [s for s in kids.findall("state")] if kids is not None else []
    if len(states) != 1 or states[0].find("eml") is None:
        raise _err("UNSUPPORTED", where, "a MATLAB Function chart with %d states (one function state expected)"
                   % len(states))
    ep = _ps(states[0].find("eml"))
    for k, v in ep.items():
        if k not in EML_P:
            raise _err("UNSUPPORTED_PARAM", where + "." + k, "MATLAB Function property %s=%r is not supported" % (k, v))
    for k, ok in PROP_VALUES.items():
        if k in ep and ep[k].strip() not in ok:
            raise _err("UNSUPPORTED_PARAM", where + "." + k, "MATLAB Function property %s=%r (recorded 0 / 1)"
                       % (k, ep[k]))
    if ep.get("isEML") != "1" or not ep.get("script", "").strip():
        raise _err("BAD_SLX", where, "the function state carries no MATLAB script")
    for k, table in (("emlDefaultFimath", DEFAULT_FIMATH), ("treatAsFi", TREAT), ("saturateOnIntegerOverflow", SAT)):
        if ep.get(k) not in table:
            raise _err("UNSUPPORTED_PARAM", where + "." + k, "%s=%r (only %s)" % (
                k, ep.get(k), ", ".join(repr(x) for x in table if x is not None) + " or absent"))
    script = ep["script"]
    nb = {"type": "MATLABFunction", "Script": script}
    if ep.get("emlDefaultFimath") is not None:
        nb["EmlDefaultFimath"] = DEFAULT_FIMATH[ep["emlDefaultFimath"]]
    if ep.get("fimathString") is not None:
        nb["InputFimath"] = ep["fimathString"]
    if ep.get("treatAsFi") is not None:
        nb["TreatAsFi"] = TREAT[ep["treatAsFi"]]
    if ep.get("saturateOnIntegerOverflow") is not None:
        nb["SaturateOnIntegerOverflow"] = SAT[ep["saturateOnIntegerOverflow"]]
    main = fxhdl._ml_main(script, where)
    data = {}
    for d in kids.findall("data"):
        dp = _ps(d)
        nm = d.get("name")
        for k in dp:
            if k not in DATA_P:
                raise _err("UNSUPPORTED_PARAM", where + "." + nm + "." + k, "data property %s is not supported" % k)
        size = [p.text for p in d.findall("props/array/P") if p.get("Name") == "size"]
        if size and size != ["-1"]:
            raise _err("UNSUPPORTED", where + "." + nm, "argument %s has a fixed size %s (inherited scalars only)"
                       % (nm, size[0]))
        cplx = [p.text for p in d.findall("props/P") if p.get("Name") == "complexity"]
        if cplx and cplx[0] not in ("SF_COMPLEX_INHERITED", "SF_COMPLEX_NO"):
            raise _err("UNSUPPORTED", where + "." + nm, "complex argument %s" % nm)
        data[nm] = dp
    ins = [n for n, dp in data.items() if dp.get("scope") == "INPUT_DATA"]
    outs = [n for n, dp in data.items() if dp.get("scope") == "OUTPUT_DATA"]
    other = [n for n, dp in data.items() if dp.get("scope") not in ("INPUT_DATA", "OUTPUT_DATA")]
    if other:
        raise _err("UNSUPPORTED", where + "." + other[0], "data %s of scope %s (parameters and local data are not "
                   "supported)" % (other[0], data[other[0]].get("scope")))
    if sorted(ins) != sorted(main["inputs"]) or sorted(outs) != sorted(main["outputs"]):
        raise _err("BAD_SLX", where, "the chart's data (%s / %s) do not match the function's arguments (%s / %s)" % (
            ins, outs, main["inputs"], main["outputs"]))
    for n in ins:
        if data[n].get("dataType", "Inherit: Same as Simulink") != "Inherit: Same as Simulink":
            raise _err("UNSUPPORTED", where + "." + n, "input %s declares its own type %r (inherited only)"
                       % (n, data[n]["dataType"]))
    for k, n in enumerate(main["outputs"]):
        dt = data[n].get("dataType", "Inherit: Same as Simulink")
        if dt != "Inherit: Same as Simulink":
            nb[fxhdl._ml_out_key(k + 1)] = dt
    # port order: the block's own Inports / Outports, numbered, carry the argument names
    import fxslx_lib as S
    part, el = m.subsystem(sys.part, sys.blocks[sid])
    for kind, names in (("Inport", main["inputs"]), ("Outport", main["outputs"])):
        got = []
        for x in el.findall("Block"):
            if x.get("BlockType") == kind:
                try:
                    got.append((int(S._params(x).get("Port", "1")), x.get("Name")))
                except ValueError:
                    raise _err("BAD_SLX", where, "bad Port number in the MATLAB Function's own system")
        got.sort()
        if [nm for _, nm in got] != list(names) or [k for k, _ in got] != list(range(1, len(names) + 1)):
            raise _err("UNSUPPORTED", where, "%s order %s differs from the function's arguments %s (reordered ports "
                       "are not supported)" % (kind, [nm for _, nm in got], list(names)))
    return nb, len(main["outputs"]), period


def is_ml(sys, sid):
    import fxslx_lib as S
    return sys.btype(sid) == "SubSystem" and S._params(sys.blocks[sid]).get("SFBlockType") is not None


def check_sf_type(sys, sid):
    import fxslx_lib as S
    t = S._params(sys.blocks[sid]).get("SFBlockType")
    if t != "MATLAB Function":
        raise _err("UNSUPPORTED_BLOCK", sys.where(sid), "Stateflow block %s is not supported (MATLAB Function only)"
                   % t)


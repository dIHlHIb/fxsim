"""Python interface to the same flat diagrams, engine and SLX writer as batch."""
from copy import deepcopy
from dataclasses import replace
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace

from . import _delivery, _model as graph, _msave
from ._backend import FxError, options, require
from ._marray import Array
from ._mbuild import DEFAULT, FROM, Port, State
from ._pyfixed import numerictype

_states = {}
_last_inputs = {}


def _state(path):
    name = path.path if isinstance(path, Port) else path
    require(isinstance(name, str), "model", "Use a model or model/block path.", "NO_MODEL")
    name = name.split("/")[0]
    require(name in _states, name, "Create the model with new_system first.", "NO_MODEL")
    return _states[name]


def new_system(name):
    graph.new_system(name)
    _states[name] = State(name)
    return name


def close_system(name, save=0):
    require(save in (0, 1), "close_system", "Use 0 or 1 for the save flag.")
    if save:
        save_system(name)
    graph.close_system(name)
    del _states[name]
    _last_inputs.pop(name, None)


def _params(args, kwargs, path):
    opts = options(args, kwargs, path)
    if "Position" in opts:
        v = opts["Position"]
        require(isinstance(v, (list, tuple)) and len(v) == 4 and all(type(n) is int for n in v),
                "Position", "Use four integer coordinates.", "BAD_PARAM")
        opts["Position"] = Array(1, 4, tuple(map(float, v)))
    return {k: str(v) if isinstance(v, numerictype) else v for k, v in opts.items()}


def add_block(source, dest, *args, **kwargs):
    state = _state(dest)
    require(isinstance(source, str), "add_block", "Use a library path.", "BAD_PARAM")
    require(dest.partition("/")[2] not in state.blocks, dest, "Block already exists.", "BLOCK_EXISTS")
    state.edit(dest, _params(args, kwargs, dest), source.replace("\n", " "))
    return dest


class _Outport(Port):
    def __call__(self, index):
        require(type(index) is int and index == self.number, self.path, "Unknown output port.", "BAD_CONNECTION")
        return self


def set_param(path, *args, **kwargs):
    state = _state(path)
    opts = _params(args, kwargs, str(path))
    if isinstance(path, Port):
        state.log(path, opts)
    elif path == state.name:
        state.configure(opts)
    else:
        state.edit(path, opts)


def get_param(path, key):
    if isinstance(path, str) and path in graph.CATALOG:
        values = graph.DEFAULTS.get(graph.CATALOG[path]["kind"], {})
        require(key in values, path, "No recorded library readback for this parameter.")
        return values[key]
    state = _state(path)
    if isinstance(path, Port):
        values = state.logs.get(path.path.partition("/")[2], {})
        require(key in values or key == "DataLogging", path.path, "Unknown logging parameter.")
        return values.get(key, "off")
    if path == state.name and key not in state.settings and key in graph.SETTINGS:
        return graph.SETTINGS[key]
    value = state.get(path, key)
    if key == "PortHandles":
        port = value["Outport"]
        return SimpleNamespace(Outport=_Outport(port.path, port.number))
    return list(map(int, value.data)) if isinstance(value, Array) else value


def add_line(name, source, dest, *args, **kwargs):
    opts = options(args, kwargs, "add_line")
    pairs = [item for pair in opts.items() for item in pair]
    _state(name).connect(source, dest, pairs)


def design_json(name):
    """Serialize explicit graph parameters and port connections in QUICKSTART format."""
    state = _state(name)
    model = graph.model(name)
    blocks = {}
    for n, b in model.blocks.items():
        params = state.blocks[n]["params"]
        blocks[n] = {k: v for k, v in b.items() if k != "Port" or "Port" in params}
        require(state.blocks[n]["source"] != FROM, "design_json", "Workspace source needs a simulation binding.")
    # A single-input block's destination port and a first source port default to one.
    multi = {e["dst"] for e in model.lines if e["dport"] > 1}
    lines = []
    for e in model.lines:
        row = {"src": e["src"], "dst": e["dst"]}
        if e["sport"] != 1:
            row["sport"] = e["sport"]
        if e["dst"] in multi:
            row["dport"] = e["dport"]
        lines.append(row)
    return dict(blocks=deepcopy(blocks), lines=lines)


def _request(state, data, nsteps, stop_time):
    require(state.settings.get("LoadExternalInput", "off") == "off", "ExternalInput",
            "Pass named Python inputs to sim. Named MATLAB workspaces use the batch entry.")
    configured_stop = state.settings.get("StopTime") if nsteps is None and stop_time is None else None
    if data is None and not graph.model(state.name).ports("Inport"):
        data = {}
    if data is None:
        bound = replace(state, settings=dict(state.settings, Solver="FixedStepDiscrete"))
        if nsteps is not None:
            require(type(nsteps) is int and nsteps > 0, "nsteps", "Expected a positive step count.", "BAD_PARAM")
            dt = graph.input_clock({"design":{"blocks":graph.model(state.name).blocks}})
            bound.settings["StopTime"] = str((nsteps - 1) * dt)
        elif stop_time is not None:
            bound.settings["StopTime"] = str(stop_time)
        data, _, _, _ = bound.inputs({}, saving=False)
    for n, b in state.blocks.items():
        if b["source"] == FROM:
            require(b["params"].get("Interpolate") == "off" and b["params"].get("VariableName"),
                    n, "Specify VariableName and disable interpolation for a workspace source.")
    req = graph.request(state.name, data, nsteps, stop_time if configured_stop is None else configured_stop)
    for b in req["design"]["blocks"].values():
        if b["type"] in ("Downsample", "Upsample", "Repeat", "CICDecimation", "FIRInterpolation", "FIRDecimation", "RateTransition"):
            defaults = graph.DEFAULTS.get(b["type"], {})
            for key in ("RateOptions", "framing", "InputProcessing", "OutPortSampleTimeOpt"):
                if key in defaults:
                    b.setdefault(key, defaults[key])
    return req


def _grid(state, req, raw):
    dt = graph.input_clock(req)
    metrics = raw["metrics"]
    if "stop_time" in metrics:
        stop = Fraction(metrics["stop_time"])
    else:
        # A single-rate graph returns step numbers. A common external clock anchors
        # those steps; never put a conflicting explicit source or block on that grid.
        for n, b in req["design"]["blocks"].items():
            for key in ("SampleTime", "tsamp", "samptime"):
                if key in b and str(b[key]).strip().lower() not in ("-1", "inf"):
                    require(graph.clock(b[key]) == (dt, 0), n+"."+key,
                            "The engine returned steps without confirming this physical clock.")
        stop = (metrics["nsteps"] - 1) * dt
    return dt, stop, state.grid_step(req, raw, dt, stop)


class SimulationOutput:
    def get(self, name):
        require(name in vars(self) and name in ("yout", "tout", "logsout"), str(name),
                "Unknown or disabled workspace output.", "NO_SIGNAL")
        return getattr(self, name)


def sim(name, *args, inputs=None, nsteps=None, stop_time=None, diagnostics=False, **kwargs):
    state = _state(name)
    if args and isinstance(args[0], dict):
        require(inputs is None, "sim", "Supply the inputs once.", "BAD_PARAM")
        inputs, args = args[0], args[1:]
    state = replace(state, settings=dict(state.settings))
    state.configure(options(args, kwargs, "sim"))
    req = _request(state, inputs, nsteps, stop_time)
    req["diagnostics"] = diagnostics
    names = graph.model(name).ports("Outport")
    cfg = {**DEFAULT, **state.settings}
    logs = {}
    for n, o in state.logs.items():
        if cfg["SignalLogging"] == "on" and o.get("DataLogging") == "on":
            label = o.get("DataLoggingName")
            require(o.get("DataLoggingNameMode") == "Custom" and label and label not in logs,
                    n, "Supply a unique custom signal logging name.")
            logs[label] = n
    if logs:
        req["outputs"] = list(dict.fromkeys([*names, *logs.values()]))
    raw = graph.run_model("fxgraph", req)
    dt, stop, step = _grid(state, req, raw)
    out = SimulationOutput()
    out.raw = raw
    if cfg["SaveOutput"] == "on":
        out.yout = graph.SimulationOutput(raw, req, names).yout
    if cfg["SaveTime"] == "on":
        out.tout = [i * step for i in range(int(stop / step) + 1)]
    if cfg["SignalLogging"] == "on":
        logged = {}
        for label, n in logs.items():
            dataset = graph.SimulationOutput(raw, req, [n]).yout
            stream = dataset.get(n)
            logged[label] = SimpleNamespace(Name=label, Values=stream.Values)
        out.logsout = graph.Dataset(logged)
    out.synthetic_inputs = list(req["inputs"]) if inputs is None else []
    _last_inputs[name] = (deepcopy(req["inputs"]), tuple(out.synthetic_inputs))
    return out


def _delivery_request(name, inputs, nsteps, stop_time):
    require(isinstance(inputs, dict), "inputs", "Specify the model inputs for replay.", "BAD_INPUT")
    state = _state(name)
    require(not any(b["source"] == FROM for b in state.blocks.values()) and
            not any(v.get("DataLogging") == "on" for v in state.logs.values()),
            "save_system", "Use the named flat SLX for this model.")
    defaults = (("SaveTime", "on"), ("SaveOutput", "on"), ("SignalLogging", "on"), ("LoadExternalInput", "off"))
    require(all(state.settings.get(k, v) == v for k, v in defaults), "save_system",
            "Save a named flat SLX to preserve these logging or workspace settings.")
    return _request(state, inputs, nsteps, stop_time), state.settings.get("FixedStep", "auto")


def save_system(name, destination=None, *, inputs=None, nsteps=None, stop_time=None, **options):
    require(not options, "save_system", "Unknown save option: " + ", ".join(sorted(options)), "BAD_PARAM")
    state = _state(name)
    synthetic = ()
    if inputs is None:
        if name in _last_inputs:
            inputs, synthetic = _last_inputs[name]
        else:
            synthetic = graph.model(name).ports("Inport")
    path = Path(destination) if destination is not None else (state.path or Path(name + ".slx"))
    if path.suffix.lower() != ".slx":
        return _delivery.save_system(name, path, inputs=inputs, nsteps=nsteps, stop_time=stop_time,
                                     _prepare=_delivery_request)
    req = _request(state, inputs, nsteps, stop_time)
    raw = graph.run_model("fxgraph", req)
    dt, stop, _ = _grid(state, req, raw)
    # Materialize the simulation horizon in the saved model as a native dialog literal.
    native_stop = str(int(stop)) if stop.denominator == 1 else str(float(stop))
    require(Fraction(native_stop) == stop, "StopTime", "Horizon needs a finite native decimal literal.")
    flat = replace(state, settings={**state.settings, "Solver":"FixedStepDiscrete", "StopTime":native_stop})
    path = path.absolute()
    _msave.save(flat, path, {}, prepared=(req, dt, stop, list(synthetic)))
    state.path = path
    state.dirty = False
    return dict(entry=path, manifest=path.with_name(name + "_manifest.json"),
                check=path.with_name("check_" + name + ".m"), io=path.with_name(name + "_io.json"))

import json
import re
import struct
from copy import deepcopy
from dataclasses import dataclass, field
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace

from ._backend import FxError, options, require, run
from ._fixed import FiData, numerictype

BASE = Path(__file__).parent
CATALOG = json.loads((BASE / "catalog.json").read_text())
DEFAULTS = json.loads((BASE / "library_defaults.json").read_text())
ID = re.compile(r"[A-Za-z][A-Za-z0-9_]{0,59}")
SETTINGS = {"SolverType": "Fixed-step", "Solver": "FixedStepDiscrete", "FixedStep": "auto", "StartTime": "0"}
ROOTS = ("Inport", "Outport")
_models = {}


@dataclass
class _Model:
    blocks: dict = field(default_factory=dict)
    sources: dict = field(default_factory=dict)
    lines: list = field(default_factory=list)
    settings: dict = field(default_factory=lambda: dict(SETTINGS))

    def ports(self, kind):
        return sorted((n for n, b in self.blocks.items() if b["type"] == kind),
                      key=lambda n: int(self.blocks[n]["Port"]))


def new_system(name):
    require(isinstance(name, str) and ID.fullmatch(name), "new_system", "Expected a plain model name.", "BAD_PARAM")
    require(name not in _models, name, "Model already exists.", "MODEL_EXISTS")
    _models[name] = _Model()
    return name


def model(name):
    require(isinstance(name, str) and name in _models, str(name), "Create the model with new_system first.", "NO_MODEL")
    return _models[name]


def close_system(name):
    model(name)
    del _models[name]


def _path(path):
    require(isinstance(path, str) and path.count("/") == 1, str(path), "Use model/block in a flat model.")
    m, n = path.split("/")
    require(n and n == n.strip() and re.fullmatch(r"[A-Za-z0-9_ -]+", n), path,
            "Use letters, digits, spaces, underscores or hyphens in flat block names.")
    return model(m), n


def _parameters(m, n, source, values):
    kind = CATALOG[source]["kind"]
    result = {}
    for k, v in values.items():
        require(k in CATALOG[source]["params"], n + "." + k, f"Unknown {kind} dialog parameter {k}.", "BAD_PARAM")
        require(isinstance(v, (str, numerictype)), n + "." + k, "Use the Simulink dialog value as text.", "BAD_PARAM")
        result[k] = str(v)
    if "Port" in result:
        require(kind in ROOTS and re.fullmatch(r"[1-9][0-9]*", result["Port"]), n + ".Port", "Expected positive port number.", "BAD_PARAM")
        require(all(m.blocks[p]["Port"] != result["Port"] for p in m.ports(kind) if p != n),
                n + ".Port", "Port already in use. Use distinct port numbers.", "BAD_PARAM")
    return result


def add_block(source, dest, *args, **kwargs):
    m, n = _path(dest)
    require(n not in m.blocks, dest, "Block already exists.", "BLOCK_EXISTS")
    require(isinstance(source, str), dest, "Expected a library path.", "BAD_PARAM")
    source = source.replace("\n", " ")
    require(source in CATALOG, source, "Library path outside this facade.", "UNKNOWN_BLOCK_TYPE")
    kind = CATALOG[source]["kind"]
    params = _parameters(m, n, source, options(args, kwargs, dest))
    if kind in ROOTS and "Port" not in params:
        used = {int(m.blocks[p]["Port"]) for p in m.ports(kind)}
        params["Port"] = str(next(i for i in range(1, len(used) + 2) if i not in used))
    m.blocks[n] = dict(type=kind, **params)
    m.sources[n] = source
    return dest


def set_param(path, *args, **kwargs):
    params = options(args, kwargs, path)
    if path in _models:
        for k, v in params.items():
            if k == "StopTime":
                require(isinstance(v, str) and rational(v, k) >= 0, k, "Use a nonnegative literal StopTime.")
            else:
                require(k in SETTINGS and v == SETTINGS[k], path + "." + k, "Unsupported model setting.")
        model(path).settings.update(params)
        return
    m, n = _path(path)
    require(n in m.blocks, path, "Unknown block.", "NO_BLOCK")
    m.blocks[n].update(_parameters(m, n, m.sources[n], params))


def get_param(path, name):
    if path in _models:
        values = model(path).settings
    else:
        m, n = _path(path)
        require(n in m.blocks, path, "Unknown block.", "NO_BLOCK")
        if name == "Name":
            return n
        values = dict(DEFAULTS.get(m.blocks[n]["type"], {}), **m.blocks[n])
        require(name != "type", path, "Use the Simulink dialog name.", "BAD_PARAM")
    require(name in values, path + "." + name, "No explicit value or recorded library readback.")
    return values[name]


def add_line(name, source, dest, *args, **kwargs):
    m = model(name)
    route = options(args, kwargs, "add_line")
    require(all(k == "autorouting" and v in ("on", "off") for k, v in route.items()),
            "add_line", "Only autorouting on/off is accepted.")
    def endpoint(s):
        match = re.fullmatch(r"([^/]+)/([1-9][0-9]*)", s) if isinstance(s, str) else None
        require(match and match[1] in m.blocks, str(s), "Expected an existing block/port.", "BAD_CONNECTION")
        return match[1], int(match[2])
    a, ap = endpoint(source)
    b, bp = endpoint(dest)
    require(all((l["dst"], l["dport"]) != (b, bp) for l in m.lines), dest, "Input port is already driven.", "BAD_CONNECTION")
    m.lines.append(dict(src=a, sport=ap, dst=b, dport=bp))


def rational(text, where):
    try:
        return Fraction(str(text))
    except (ValueError, ZeroDivisionError):
        raise FxError("BAD_PARAM", where, f"Expected a finite number or fraction, got {text!r}.")


def clock(text):
    words = str(text).strip().strip("[]").replace(",", " ").split()
    require(len(words) in (1, 2), "SampleTime", "Expected a period or [period offset].")
    return rational(words[0], "SampleTime"), rational(words[1], "SampleTime") if len(words) == 2 else Fraction(0)


def _input_type(data):
    if data.get("type") == "boolean":
        return "boolean"
    if all(k in data for k in ("s", "wl", "fl")):
        return "fixdt({s},{wl},{fl})".format(**data)
    nested = data.get("type")
    if isinstance(nested, dict) and all(k in nested for k in ("s", "wl", "fl")):
        return "fixdt({s},{wl},{fl})".format(**nested)
    return nested


def request(name, inputs, nsteps=None, stop_time=None):
    m = model(name)
    require(isinstance(inputs, dict), "inputs", "Supply a mapping from input name to fi or stored data.", "BAD_INPUT")
    data = {k: v.record() if isinstance(v, FiData) else deepcopy(v) for k, v in inputs.items()}
    for k, item in data.items():
        require(isinstance(k, str) and isinstance(item, dict), "inputs", "Expected named signal records.", "BAD_INPUT")
    blocks = deepcopy(m.blocks)
    for n, b in blocks.items():
        if b["type"] == "Lookup1D":
            b.setdefault("InputSameDT", "on")
        if b["type"] != "Inport":
            continue
        dtype = b.pop("OutDataTypeStr", "Inherit: auto").replace(" ", "")
        if b.get("SampleTime") == "-1":
            del b["SampleTime"]  # the external sample series supplies its default unit clock
        if dtype != "Inherit:auto":
            if re.fullmatch(r"u?int(8|16|32|64)", dtype):
                dtype = f"fixdt({int(not dtype.startswith('u'))},{dtype.lstrip('uint')},0)"
            for key, item in data.items():
                if key == n or re.fullmatch(re.escape(n) + r"\[[1-9][0-9]*\]", key):
                    require(dtype == _input_type(item), "inputs." + key, "Input data and Inport.OutDataTypeStr disagree.", "TYPE_MISMATCH")
    for kind in ROOTS:
        ports = m.ports(kind)
        require([int(blocks[n]["Port"]) for n in ports] == list(range(1, len(ports) + 1)), kind,
                "Root port numbers must be contiguous.", "BAD_PARAM")
    req = dict(design=dict(blocks=blocks, lines=deepcopy(m.lines)), inputs=data)
    native_stop = m.settings.get("StopTime")
    require(sum(v is not None for v in (nsteps, stop_time, native_stop)) <= 1, "StopTime", "Choose one simulation horizon.", "BAD_PARAM")
    if nsteps is not None:
        req["nsteps"] = nsteps
    if stop_time is not None or native_stop is not None:
        req["stop_time"] = str(stop_time if native_stop is None else native_stop)
    return req


def input_clock(req):
    clocks = {clock(b.get("SampleTime", "1")) for b in req["design"]["blocks"].values() if b["type"] == "Inport"}
    require(len(clocks) <= 1, "SampleTime", "A common input clock is required here.")
    period, offset = next(iter(clocks), (Fraction(1), Fraction(0)))
    require(period > 0 and not offset, "SampleTime", "Expected a positive input period and zero phase.")
    return period


def run_model(tool, req):
    # fxgraph and fxexport run different schedulers for single- and multirate graphs.
    try:
        result = run(tool, req)
    except FxError as e:
        if (e.code, e.where) != ("BAD_PARAM", "stop_time") or "only for multirate" not in e.message:
            raise
        short = dict(req)
        stop = rational(short.pop("stop_time"), "StopTime")
        require(stop >= 0, "StopTime", "Expected a nonnegative horizon.", "BAD_PARAM")
        steps = stop / input_clock(req)
        require(steps.denominator == 1, "StopTime", "An explicit single-rate stop time must fall on the input clock.")
        short["nsteps"] = int(steps) + 1
        return run(tool, short)
    if "stop_time" in req and tool == "fxgraph" and "stop_time" not in result["metrics"]:
        raise FxError("UNSUPPORTED", "StopTime", "Backend did not confirm the requested time horizon.")
    return result


class Dataset:
    def __init__(self, signals):
        self._signals = signals

    @property
    def numElements(self):
        return len(self._signals)

    def get(self, name):
        if type(name) is int:
            return self.getElement(name)
        require(name in self._signals, str(name), "Unknown output.", "NO_SIGNAL")
        return self._signals[name]

    def getElement(self, key):
        if type(key) is int:
            require(1 <= key <= len(self._signals), "yout", "Output index out of range.", "NO_SIGNAL")
            key = list(self._signals)[key - 1]
        return self.get(key)


class SimulationOutput:
    def __init__(self, raw, req, names):
        self.raw = raw
        signals = {}
        for name in names:
            # Keep vector elements and complex parts identifiable by their engine names.
            keys = [k for k in raw["outputs"] if k == name or k.startswith((name+"[", name+"."))]
            for k in sorted(keys, key=lambda s: [int(t) if t.isdigit() else t for t in re.split(r"(\d+)", s)]):
                row = raw["outputs"][k]
                if "ints" in row:
                    codes = row["ints"]
                    data = [n * Fraction(2) ** -row["fl"] for n in codes]
                elif "bool" in row:
                    codes, data = row["bool"], list(map(bool, row["bool"]))
                else:
                    fmt = {"single": ">f", "double": ">d"}[row["type"]]
                    codes = None
                    data = [struct.unpack(fmt, bytes.fromhex(bits))[0] for bits in row["hex"]]
                times = (list(map(Fraction, row["t"])) if "t" in row else
                         [i * input_clock(req) for i in range(len(data))])
                signals[k] = SimpleNamespace(Name=k, Values=SimpleNamespace(Data=data, Time=times,
                                              DataType=row["type"], storedInteger=codes))
        self.yout = Dataset(signals)

    def get(self, name):
        require(name == "yout", str(name), "Use yout for logged root outputs.", "NO_SIGNAL")
        return self.yout


def sim(name, inputs=None, *, nsteps=None, stop_time=None, diagnostics=False):
    req = request(name, {} if inputs is None else inputs, nsteps, stop_time)
    req["diagnostics"] = diagnostics
    return SimulationOutput(run_model("fxgraph", req), req, model(name).ports("Outport"))

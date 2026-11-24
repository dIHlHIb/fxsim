"""fxslx: read Simulink .slx models into fxgraph design JSON, and write data types back .

Format handled (as publicly described; see docs/en/SLX.md): an .slx file is a zip archive; block
diagrams are XML. A system holds Block elements (attributes BlockType, Name, SID; parameters as
<P Name="...">value</P> children) and Line elements whose endpoints are written either as
"SID#out:k" / "SID#in:k" (P Name="Src"/"Dst", with nested Branch elements for fan-out) or, in the
older style, as SrcBlock/SrcPort/DstBlock/DstPort by block name. A SubSystem block's content is a
nested System element, a System element with a Ref attribute naming a file in simulink/systems/, or
simulink/systems/system_<SID>.xml. Parameters equal to their default may be omitted; defaults come from
BlockParameterDefaults (simulink/bddefaults.xml or inside blockdiagram.xml), else the engine's own.

Subsystems are flattened (names become "Sub/Inner"), virtual subsystem ports are removed by rewiring.
Only the block types the engine supports are accepted; anything else is reported with a stable code.
"""
import hashlib
import io
import math
import re
import zipfile
import xml.etree.ElementTree as ET

import fxgraph_lib as G
import fxslx_ml as ML  # noqa: E402  (MATLAB Function blocks)
import fxslx_mref as MR  # noqa: E402  (Model Reference blocks)
from blockspec import TYPEMAP, LIBRARY, MUST, VIRTUAL, NUMERIC, VECTOR_OK, SLX_DEFAULTS, SLX_UNKNOWN_DEFAULT, HAS_SAMPLE_TIME, COSMETIC, _SAVED_UNSUPPORTED  # noqa: E402,F401 -- the block-support registry

MAX_MEMBER_BYTES = 256 << 20

_NUM = re.compile(r"^\s*[-+]?(\d+(\.\d*)?|\.\d+)([eE][-+]?\d+)?\s*$")
_NUM1 = r"[-+]?(\d+(\.\d*)?|\.\d+)([eE][-+]?\d+)?"
# a numeric row vector literal "[a b c]" / "[a, b, c]" (v1 vector signals: per-element parameters, engine/fxvector)
# a complex literal (Gain 0.5+0.25i)
_CPLXNUM = re.compile(r"^\s*([-+]?(\d+(\.\d*)?|\.\d+)([eE][-+]?\d+)?\s*[-+]\s*(\d+(\.\d*)?|\.\d+)?([eE][-+]?\d+)?"
                      r"|[-+]?((\d+(\.\d*)?|\.\d+)([eE][-+]?\d+)?)?)\s*[ij]\s*$")
# a pure imaginary literal (2i, -i, 0.5i) as well -- the engine takes it, the export writes it
# inside the brackets only spaces / tabs separate elements -- a newline starts a new row in MATLAB, so
# a value with one is not a vector literal (fxnum reads it as a matrix and the block refuses it)
_VECNUM = re.compile(r"^\s*\[[ \t]*%s([ \t]*[,;]?[ \t]*%s)*[ \t]*\]\s*$" % (_NUM1, _NUM1))   # rows separated by ";"
_EP = re.compile(r"^(\d+(?::\d+)*)#(in|out):(\d+)$")
# an enabled subsystem's enable port, or an action subsystem's action port (#ifaction,
# read as the enable of the enabled subsystem the action subsystem is lowered to)
_EP_ENABLE = re.compile(r"^(\d+(?::\d+)*)#(enable|ifaction|trigger|Reset)$")


def _src_key(kv):
    """Order of connections (dst sid, port) -> source: a port may be a number or a name (real models: 'enable',
    'trigger'); comparing the two raised TypeError (real R2026b models: INTERNAL on scoped reads)."""
    (d, p), (s_, sp) = kv
    num = lambda v: (0, v, "") if isinstance(v, int) else (1, 0, str(v))
    return (str(d), num(p), str(s_), num(sp))


class SlxError(G.GraphError):
    pass


# ---------------------------------------------------------------- execution context
# Only single-rate discrete models are accepted: the engine runs every block once per step.
SUPPORTED_SOLVERS = ("FixedStepDiscrete",)
_ST = re.compile(r"^\s*\[?\s*([-+]?(\d+(\.\d*)?|\.\d+)([eE][-+]?\d+)?|inf|Inf)\s*(?:[ ,]\s*([-+]?(\d+(\.\d*)?|\.\d+)([eE][-+]?\d+)?))?\s*\]?\s*$")


HW_KEYS = ("ProdHWDeviceType", "ProdBitPerChar", "ProdBitPerShort", "ProdBitPerInt", "ProdBitPerLong",
           "ProdBitPerLongLong", "ProdLongLongMode")


def _config_parts(m):
    """The configuration part the model runs with (the first configSet*.json in name order was
    read, whatever configSetInfo.json marks as active -- an inactive set's hardware or solver could be taken).
    One entry of configSetInfo.json is "Active" and names its part; a file with several configuration parts and
    no such entry is refused."""
    import json
    parts = sorted(n for n in m.members if re.match(r"^simulink/configSet\d*\.json$", n))
    raw = m.members.get("simulink/configSetInfo.json")
    if raw is None:
        if len(parts) > 1:
            raise SlxError("UNSUPPORTED", "simulink/configSetInfo.json", "several configuration sets and no "
                           "configSetInfo.json naming the active one")
        return parts
    try:
        entries = json.loads(raw.decode("utf-8")).get("ConfigSetInfo")
    except (ValueError, AttributeError):
        raise SlxError("BAD_SLX", "simulink/configSetInfo.json", "configuration index is not valid JSON")
    if not isinstance(entries, list) or not all(isinstance(e, dict) for e in entries):
        raise SlxError("BAD_SLX", "simulink/configSetInfo.json", "ConfigSetInfo list expected")
    active = [e for e in entries if e.get("Active") is True]
    if len(active) != 1:
        if not entries and len(parts) <= 1:
            return parts
        raise SlxError("UNSUPPORTED", "simulink/configSetInfo.json", "%d active configuration sets (exactly one "
                       "expected)" % len(active))
    part = active[0].get("PartName")
    part = part.lstrip("/") if isinstance(part, str) else None
    if part not in m.members:
        # e.g. a configuration reference (the settings live in a workspace variable, not in the file)
        raise SlxError("UNSUPPORTED", "simulink/configSetInfo.json", "the active configuration set %r is not a "
                       "part of the file (a configuration reference?): its solver and hardware cannot be read"
                       % active[0].get("ConfigSetName"))
    return [part]


def _hardware(m):
    """Production-hardware settings of the model configuration (R2026b configSet*.json), as an engine hw dict
    (bit sizes as integers); {} when the file carries none. Inherited internal rules depend on them."""
    import json
    for part in _config_parts(m):
        try:
            j = json.loads(m.members[part].decode("utf-8"))
        except ValueError:
            continue
        found = {}

        def walk(o):
            if isinstance(o, dict):
                for k, v in o.items():
                    if k in HW_KEYS and isinstance(v, (str, int, float)):
                        found.setdefault(k, v)
                    walk(v)
            elif isinstance(o, list):
                for v in o:
                    walk(v)
        walk(j)
        if found:
            return {k: (int(float(v)) if k.startswith("ProdBitPer") else str(v)) for k, v in found.items()}
    # XML configuration sets (an ASIC/FPGA setting kept there was missed and the host rule applied, a
    # 1-LSB error reported as ok); read the same fields from <P Name="..."> elements
    for part in sorted(n for n in m.members if re.match(r"^simulink/configSet\d*\.xml$", n)):
        t = m._tree(part)
        if t is None:
            raise SlxError("BAD_SLX", part, "configuration part is not valid XML; the production hardware (which "
                           "inherited types depend on) cannot be determined")
        found = {}
        for el in t.iter("P"):
            k = el.get("Name")
            if k in HW_KEYS and k not in found:
                found[k] = (el.text or "").strip()
        if found:
            try:
                return {k: (int(float(v)) if k.startswith("ProdBitPer") else str(v)) for k, v in found.items()}
            except ValueError:
                raise SlxError("BAD_SLX", part, "production hardware word sizes are not numbers: %s" % found)
    # no hardware field in any configuration set: Simulink leaves default settings out of the file, so the default
    # production hardware (x86-64 host) applies -- every genuine R2026b file in recordings follows this (66 files)
    other = [n for n in m.members if re.match(r"^simulink/configSet", n) and not re.search(r"\.(json|xml|rels)$", n)]
    if other:
        raise SlxError("UNSUPPORTED", other[0], "configuration set in a format the reader does not parse; the "
                       "production hardware cannot be determined")
    return {}


def _solver(m):
    """(solver name, fixed step text) from the model configuration (R2026b configSet*.json, or <P> elements
    of an XML configuration set); (None, None) if the file carries none."""
    import json
    for part in _config_parts(m):
        try:
            j = json.loads(m.members[part].decode("utf-8"))
        except ValueError:
            raise SlxError("BAD_SLX", part, "configuration part is not valid JSON")
        found = {}

        def walk(o):
            if isinstance(o, dict):
                for k, v in o.items():
                    if k in ("SolverName", "FixedStep") and isinstance(v, str):
                        found.setdefault(k, v)
                    walk(v)
            elif isinstance(o, list):
                for v in o:
                    walk(v)
        walk(j)
        if "SolverName" in found:
            return found["SolverName"], (_time_text(found["FixedStep"]) if "FixedStep" in found else None)
    for part in sorted(n for n in m.members if n.endswith(".xml")):
        t = m._tree(part)
        if t is None:
            continue
        name = step = None
        for el in t.iter("P"):
            if el.get("Name") == "SolverName" and name is None:
                name = (el.text or "").strip()
            elif el.get("Name") == "FixedStep" and step is None:
                step = _time_text((el.text or "").strip())
        if name:
            return name, step
    return None, None


def _stop_time(m):
    """The model's StopTime text from its configuration (configSet*.json, or <P Name="StopTime"> of an XML part);
    None when the file does not write it (agents asked for it to size their runs)."""
    import json
    for part in _config_parts(m):
        try:
            j = json.loads(m.members[part].decode("utf-8"))
        except ValueError:
            raise SlxError("BAD_SLX", part, "configuration part is not valid JSON")
        stack = [j]
        while stack:
            o = stack.pop()
            if isinstance(o, dict):
                if isinstance(o.get("StopTime"), str):
                    return o["StopTime"].strip()
                stack.extend(o.values())
            elif isinstance(o, list):
                stack.extend(o)
    for part in sorted(n for n in m.members if n.endswith(".xml")):
        t = m._tree(part)
        if t is None:
            continue
        for el in t.iter("P"):
            if el.get("Name") == "StopTime":
                return (el.text or "").strip()
    return None


# Simulink's effective SampleTime of a block whose saved model omits it (R2026b, recordings: the
# built-in default, and get_param after deleting SampleTime from the saved XML; Simulink writes the parameter out
# whenever the dialog value -1 differs from the built-in default, so an omitted one is the built-in default)
OMITTED_SAMPLE_TIME = {"UnitDelay": "1", "DiscreteIntegrator": "1", "Delay": "-1", "DiscreteFir": "-1",
                       "Constant": "inf", "FromWorkspace": "0", "Gain": "-1", "Sum": "-1", "Inport": "-1"}
# the value checked against the base step: a From Workspace's 0 (continuous) runs at the fixed step of the
# FixedStepDiscrete solver, which is what the reader accepts; Unit Delay / Discrete-Time Integrator 1 is a real period
_OMITTED_CHECK = dict(OMITTED_SAMPLE_TIME, FromWorkspace="-1")


# semantic parameters whose built-in default (what a saved file's omission means) is not recorded
_ABSENT_UNKNOWN = {("Downsample", "RateOptions"), ("Upsample", "RateOptions"), ("CICDecimation", "RateOptions")}
_ABSENT_MSG = ("%s is not written in the file, so it is the %s block's built-in default -- single-rate / frame based"
               ", which the engine does not model; write the sample-based multirate "
               "value explicitly in the model")


def _check_sample_time(where, text, base, bt):
    """Accept an inherited (-1) sample time, the base step, [base 0], or inf for a Constant."""
    mm = _ST.match(text or "")
    if not mm:
        raise SlxError("UNSUPPORTED_PARAM", where + ".SampleTime", "sample time %r is not a numeric literal; only "
                       "single-rate discrete models are supported" % text)
    period, offset = mm.group(1), mm.group(5)
    if offset is not None and float(offset) != 0:
        raise SlxError("UNSUPPORTED_PARAM", where + ".SampleTime", "sample time offset %s (only 0 is supported)" % offset)
    if period in ("inf", "Inf"):
        if bt != "Constant":
            raise SlxError("UNSUPPORTED_PARAM", where + ".SampleTime", "infinite sample time on %s" % bt)
        return
    v = float(period)
    if v == -1:
        return
    if v <= 0:
        # 0 (continuous) or another negative value on a discrete block was accepted when
        # FixedStep is auto
        raise SlxError("UNSUPPORTED_PARAM", where + ".SampleTime", "sample time %s on %s (a discrete period, or -1 "
                       "for inherited, is required)" % (period, bt))
    if base is None:
        # FixedStep 'auto': the step is the blocks' own period; more than one period is refused after the walk
        # ("several sample times ...") -- the native recordings run FixedStepDiscrete with FixedStep auto and every
        # block at 1, which the reader refused before
        # (recorded: recordings -- From Workspace at 1, the Repeat makes the faster rate itself)
        return
    if v != base:
        raise SlxError("UNSUPPORTED_PARAM", where + ".SampleTime", "sample time %s differs from the model's "
                       "base step %s: multi-rate models are not supported" % (period, base))


_TIME_KEYS = ("SampleTime", "SystemSampleTime", "tsamp")
_ARITH_TOK = re.compile(r"\s*(?:(\d+(?:\.\d*)?(?:[eE][-+]?\d+)?|\.\d+(?:[eE][-+]?\d+)?)|([-+*/()]))")


def _arith(text):
    """Value of plain arithmetic on numeric literals (+ - * / and parentheses), or None for anything else --
    workspace variables and functions stay unread. Real models write sample times as 1/30.72e6 (
    R2026b HDL examples)."""
    from fractions import Fraction
    toks, pos, t = [], 0, str(text).strip()
    while pos < len(t):
        m = _ARITH_TOK.match(t, pos)
        if not m or m.end() == pos:
            return None
        toks.append(Fraction(m.group(1)) if m.group(1) else m.group(2))
        pos = m.end()
    k = [0]

    def peek():
        return toks[k[0]] if k[0] < len(toks) else None

    def atom():
        v = peek()
        if v is None:
            raise ValueError
        k[0] += 1
        if v in ("-", "+"):
            a = atom()
            return -a if v == "-" else a
        if v == "(":
            r = expr()
            if peek() != ")":
                raise ValueError
            k[0] += 1
            return r
        if isinstance(v, str):
            raise ValueError
        return v

    def term():
        r = atom()
        while peek() in ("*", "/"):
            op = peek()
            k[0] += 1
            b = atom()
            r = r * b if op == "*" else r / b
        return r

    def expr():
        r = term()
        while peek() in ("+", "-"):
            op = peek()
            k[0] += 1
            b = term()
            r = r + b if op == "+" else r - b
        return r
    try:
        v = expr()
    except (ValueError, ZeroDivisionError):
        return None
    return v if k[0] == len(toks) else None


def _time_text(text):
    """A sample time written with literal arithmetic, as a numeric literal (or [period offset]); else as it is."""
    t = str(text or "").strip()
    if _ST.match(t) or not t:
        return text
    inner = t[1:-1] if t.startswith("[") and t.endswith("]") else t
    parts = [x for x in re.split(r"\s*[,\s]\s*(?![^()]*\))", inner.strip()) if x]
    vals = [_arith(x) for x in parts]
    if not vals or len(vals) > 2 or any(v is None for v in vals):
        return text
    lit = [repr(float(v)) for v in vals]
    return lit[0] if len(lit) == 1 else "[%s %s]" % tuple(lit)


def _params(el):
    """Block parameters: <P> children, plus a library (Reference) block's own parameters, which a saved
    model keeps under <InstanceData> (seen in R2026b files). Sample times written as literal arithmetic are read as
    their value."""
    d = {p.get("Name"): (p.text or "") for p in el.findall("P")}
    inst = el.find("InstanceData")
    if inst is not None:
        for p in inst.findall("P"):
            d.setdefault(p.get("Name"), p.text or "")
    for k in _TIME_KEYS:
        if k in d:
            d[k] = _time_text(d[k])
    return d


class Model:
    """Parsed .slx: systems by key, the root system key, block defaults, and the raw zip members."""

    def __init__(self, data):
        try:
            self.zf = zipfile.ZipFile(io.BytesIO(data))
        except zipfile.BadZipFile:
            raise SlxError("BAD_SLX", "file", "not a zip archive (an .slx file is a zip of XML parts)")
        self.members = {}
        for info in self.zf.infolist():
            if info.file_size > MAX_MEMBER_BYTES:
                raise SlxError("LIMIT_EXCEEDED", info.filename, "archive member too large")
            self.members[info.filename] = self.zf.read(info.filename)
        self.trees = {}
        self.defaults = {}
        bd = self._tree("simulink/blockdiagram.xml")
        if bd is None and "simulink/systems/system_root.xml" not in self.members:
            raise SlxError("BAD_SLX", "simulink/blockdiagram.xml", "missing block diagram part "
                           "(neither blockdiagram.xml nor systems/system_root.xml)")
        # R2026b saves the model-level parts as JSON (blockDiagram.json, bddefaults.json); the latter held
        # no block parameter defaults in every file seen, so omitted parameters take Simulink's built-in
        # defaults -- reported as info.assumed_defaults
        self.json_parts = sorted(n for n in self.members if n.startswith("simulink/") and n.endswith(".json"))
        for part in ("simulink/bddefaults.xml", "simulink/blockdiagram.xml"):
            t = self._tree(part)
            if t is None:
                continue
            for bpd in t.iter("BlockParameterDefaults"):
                for b in bpd.findall("Block"):
                    self.defaults.setdefault(b.get("BlockType"), {}).update(_params(b))
        root = self._tree("simulink/systems/system_root.xml")
        if root is not None:
            self.root = ("simulink/systems/system_root.xml", root.getroot() if root.getroot().tag == "System"
                         else root.getroot().find("System"))
        else:
            if bd is None:
                raise SlxError("BAD_SLX", "simulink/blockdiagram.xml", "no root system found")
            r = bd.getroot()
            sysel = r if r.tag == "System" else r.find("System")
            if sysel is None:
                sysel = r.find("Model/System")
            if sysel is None:
                raise SlxError("BAD_SLX", "simulink/blockdiagram.xml", "no root System found")
            self.root = ("simulink/blockdiagram.xml", sysel)

    def _tree(self, name):
        if name in self.trees:
            return self.trees[name]
        raw = self.members.get(name)
        if raw is None:
            return None
        try:
            t = ET.ElementTree(ET.fromstring(raw))
        except ET.ParseError as e:
            raise SlxError("BAD_SLX", name, "XML parse error: %s" % e)
        self.trees[name] = t
        return t

    def subsystem(self, part, block):
        """(part, System element) holding a SubSystem block's content."""
        sysel = block.find("System")
        if sysel is not None and sysel.get("Ref"):
            name = "simulink/systems/%s.xml" % sysel.get("Ref")
        elif sysel is not None:
            return part, sysel
        else:
            name = "simulink/systems/system_%s.xml" % block.get("SID")
        t = self._tree(name)
        if t is None:
            raise SlxError("BAD_SLX", name, "content of subsystem %r not found" % block.get("Name"))
        r = t.getroot()
        return name, (r if r.tag == "System" else r.find("System"))


class _Sys:
    """One system level: blocks by SID / name, connectivity at port level."""

    def __init__(self, model, part, el, path, parent=None, owner_sid=None, problems=None):
        """problems: a list to collect line errors into instead of raising (the list op: discovery goes on)."""
        self.model, self.part, self.el, self.path, self.parent, self.owner = model, part, el, path, parent, owner_sid
        self.up = (parent, owner_sid)   # kept when a scope read cuts .parent: masks around the scope still apply
        self.blocks, self.byname = {}, {}
        for b in el.findall("Block"):
            sid = b.get("SID") or b.get("Name")
            self.blocks[sid] = b
            self.byname[b.get("Name")] = sid
        self.src = {}  # (dst sid, in port) -> (src sid, out port); in port "enable" for an enabled subsystem
        self.signal_name = {}   # (dst sid, in port) -> the name of the line feeding it (bus element names)
        # an enabled subsystem: holds an EnablePort; its Outports are real blocks (EnabledOutport)
        self.enabled = any(b.get("BlockType") in ("EnablePort", "ActionPort", "TriggerPort") for b in self.blocks.values())
        for ln in el.iter("Line"):
            if ln not in el.findall("Line"):
                continue
            if problems is None:
                self._line(ln)
            else:
                try:
                    self._line(ln)
                except SlxError as e:
                    problems.append({"where": e.where, "code": e.code, "message": e.message})

    def where(self, sid):
        return "slx:" + "/".join(self.path + [self.blocks[sid].get("Name")]) if sid in self.blocks else "slx:" + "/".join(self.path)

    def _ep(self, text, kind):
        me = _EP_ENABLE.match((text or "").strip())
        if me and kind == "in":
            g = me.group(1)
            # #trigger / #Reset: the control of a triggered / resettable subsystem
            kd = "enable" if me.group(2) in ("enable", "ifaction") else me.group(2)
            return (g.split(":")[-1] if g not in self.blocks else g), kd
        m = _EP.match((text or "").strip())
        if not m and "::" in (text or ""):
            # real R2026b models: "<SID>::<N>#out:<k>" appears in the contents of MATLAB
            # Function blocks and similar containers (a chart's own ports) -- not read
            raise SlxError("UNSUPPORTED", "slx:" + "/".join(self.path) + " line", "line endpoint %r belongs to the "
                           "contents of a MATLAB Function block or chart (written <SID>::<N>), which the reader does "
                           "not read" % text)
        if not m:
            raise SlxError("UNSUPPORTED", "slx:" + "/".join(self.path) + " line",
                           "line endpoint %r is not a data port (enable/trigger/state ports are not supported)" % text)
        sid = m.group(1).split(":")[-1] if m.group(1) not in self.blocks else m.group(1)
        if m.group(2) != kind:
            raise SlxError("BAD_SLX", "slx:" + "/".join(self.path), "endpoint %r is not an %s port" % (text, kind))
        return sid, int(m.group(3))

    def _line(self, ln):
        p = _params(ln)
        if "Src" in p:
            src = self._ep(p["Src"], "out")
            dsts = []
            if "Dst" in p:
                dsts.append(self._ep(p["Dst"], "in"))
            for br in ln.iter("Branch"):
                bp = _params(br)
                if "Dst" in bp:
                    dsts.append(self._ep(bp["Dst"], "in"))
        elif "SrcBlock" in p:
            try:
                src = (self.byname[p["SrcBlock"]], int(p.get("SrcPort", "1")))
            except (KeyError, ValueError):
                raise SlxError("BAD_SLX", "slx:" + "/".join(self.path), "line source %r not found" % p.get("SrcBlock"))
            dsts = []
            nodes = [p] + [_params(br) for br in ln.iter("Branch")]
            for q in nodes:
                if "DstBlock" in q:
                    try:
                        dsts.append((self.byname[q["DstBlock"]], int(q.get("DstPort", "1"))))
                    except (KeyError, ValueError):
                        raise SlxError("BAD_SLX", "slx:" + "/".join(self.path), "line destination %r not found"
                                       % q.get("DstBlock"))
        else:
            raise SlxError("BAD_SLX", "slx:" + "/".join(self.path), "line without a source")
        for d in dsts:
            if d in self.src:
                raise SlxError("BAD_SLX", self.where(d[0]), "input port %s driven twice" % (d[1],))
            self.src[d] = src
            if p.get("Name"):
                self.signal_name[d] = p["Name"]

    def btype(self, sid):
        return self.blocks[sid].get("BlockType")

    def port_no(self, sid):
        p = _params(self.blocks[sid])
        try:
            return int(p.get("Port", "1"))
        except ValueError:
            raise SlxError("BAD_SLX", self.where(sid), "bad Port number")


def list_model(data):
    """.slx bytes -> {"top_level": [{name, type}], "subsystems": [path]}: what can be read and which scope to choose
    (recordings cold start: a model with its own signal source at the top level cannot be read whole, and its subsystem
    names were only visible by unzipping the file)."""
    m = Model(data)
    problems = []   # recordings (real models): list failed whole at the first unsupported line
    top = _Sys(m, m.root[0], m.root[1], [], problems=problems)
    subs = []

    def walk(sys):
        for sid, b in sys.blocks.items():
            if sys.btype(sid) == "SubSystem":
                path = sys.path + [b.get("Name")]
                subs.append("/".join(x.replace("/", "//") for x in path))   # usable as "scope"
                try:
                    part, el = m.subsystem(sys.part, b)
                except SlxError as e:
                    problems.append({"where": e.where, "code": e.code, "message": e.message})
                    continue
                walk(_Sys(m, part, el, path, sys, sid, problems=problems))
    walk(top)
    # recordings, second cold start: the guide says to build the input from the model's own signal source, but the
    # source's parameters (Sine Wave amplitude, frequency, sample time) were only visible by unzipping the file;
    # every top-level block now comes with its parameters as written in the file (the model's
    # BlockParameterDefaults merged under them), and every top-level subsystem with what drives its inputs
    out = []
    for sid, b in top.blocks.items():
        bt = top.btype(sid)
        e = {"name": b.get("Name"), "type": bt}
        if bt == "SubSystem":
            e["inputs"] = {str(port): {"block": top.blocks[s_].get("Name"), "port": sp}
                           for (d_, port), (s_, sp) in sorted(top.src.items(), key=_src_key) if d_ == sid and s_ in top.blocks}
        else:
            prm = dict(m.defaults.get(bt, {}))
            prm.update(_params(b))
            e["params"] = {k: v[:200] for k, v in sorted(prm.items()) if k not in COSMETIC}
        out.append(e)
    return {"top_level": out, "subsystems": subs,
            # what a read will refuse, found while listing (a read of a scope that contains one of these fails)
            "unsupported": problems,
            "params_note": "parameters as written in the file (and its block defaults); a parameter the file "
                           "omits has the block's Simulink default value"}


def read(data, scope=None, refs=None):
    """.slx bytes -> (design JSON for fxgraph, info). scope: path of a subsystem to extract as the design
    (its ports become the design's Inports/Outports), e.g. "Chain"; default: the whole model.
    refs: model name -> .slx bytes of a referenced model (Model blocks, engine/fxslx_mref.py), or None."""
    design, info, _ = _read_model(Model(data), scope, refs)
    return design, info


# DSP HDL NCO library paths: dsphdlsrcs2/NCO links to dsphdlsigops2/NCO, the only
# difference is the library default of PhaseIncrementSource (used when a saved model omits the parameter)
NCO_LIBS = {"dsphdlsigops2/NCO": "Input port", "dsphdlsrcs2/NCO": "Property"}


# For Each subsystems
_FOREACH_KEYS = {"DisableCoverage", "IterateSubsysMaskParameter", "SubsysMaskParameterIterationDimension"}
_FOREACH_PORT = {"Inport": {"Partition": "on", "PartitionDimension": "1", "PartitionWidth": "1",
                            "PartitionOffset": "0"},
                 "Outport": {"ConcatenationDimension": "1"}}


def _foreach_of(sys):
    """The SID of the For Each block of this system level, or None."""
    return next((x for x in sys.blocks if sys.btype(x) == "ForEach"), None)


def _foreach_block(sys, sid):
    """The engine's ForEach marker (engine/fxvector.py: element-wise partition, outputs concatenated); a parameter
    the recordings did not show is refused."""
    own = {k: v for k, v in _params(sys.blocks[sid]).items() if k not in COSMETIC}
    for k in own:
        if k not in _FOREACH_KEYS:
            raise SlxError("UNSUPPORTED_PARAM", sys.where(sid) + "." + k, "For Each parameter %s is not supported" % k)
    if own.get("IterateSubsysMaskParameter", "off") == "on" and \
            own.get("SubsysMaskParameterIterationDimension", "1") != "2":
        raise SlxError("UNSUPPORTED_PARAM", sys.where(sid) + ".SubsysMaskParameterIterationDimension",
                       "only the iteration along dimension 2 of a row-vector mask parameter was recorded")
    return {"type": "ForEach"}


def _foreach_port(sys, sid, bt):
    own = _params(sys.blocks[sid])
    for k, v in _FOREACH_PORT[bt].items():
        if own.get(k, "off" if k == "Partition" else v) != v:
            raise SlxError("UNSUPPORTED_PARAM", sys.where(sid) + "." + k, "For Each %s %s=%r (only %r: element by "
                           "element)" % (bt, k, own.get(k, "off"), v))


def _foreach_mask_args(sub, block):
    """{mask parameter: value} of a masked For Each subsystem that iterates its mask parameter: the inner blocks
    take the vector (engine/fxvector.py partitions a vector parameter element by element). One parameter only."""
    fe = _foreach_of(sub)
    if fe is None or _params(sub.blocks[fe]).get("IterateSubsysMaskParameter", "off") != "on":
        return {}
    mask = block.find("Mask")
    mps = mask.findall("MaskParameter") if mask is not None else []
    if len(mps) != 1:
        raise SlxError("UNSUPPORTED", sub.where(fe), "a For Each iterating %d mask parameters (one was recorded)"
                       % len(mps))
    v = mps[0].find("Value")
    return {mps[0].get("Name"): (v.text or "").strip() if v is not None else ""}


# SubSystem parameters the reader acts on (checked or refused); the others are listed in info.ignored_parameters
_SUBSYS_READ = ("SystemSampleTime", "Commented", "Variant", "VariantControl", "DataTypeOverride", "Ports")
# blocks that make the subsystem holding them run conditionally or repeatedly
_CONDITIONAL = ("EnablePort", "TriggerPort", "ActionPort", "ResetPort", "ForIterator", "WhileIterator", "ForEach",
                "StateEnablePort")
_DTO_OK = ("UseLocalSettings", "Off")


def scope_path(scope):
    """Names along a subsystem path; a '/' inside a block name is written '//' (as in Simulink block paths)."""
    return [x.replace("//", "/") for x in re.split(r"(?<!/)/(?!/)", scope)]


def _check_model_override(m):
    """a data type override (model or subsystem level) replaces every fixed-point type below it
    by double / single / scaled double -- the engine would simulate the written types."""
    for part in m.json_parts + ["simulink/blockdiagram.xml"]:
        raw = m.members.get(part)
        if raw is None:
            continue
        for mm in re.finditer(rb'"DataTypeOverride"\s*:\s*"([^"]*)"|<P Name="DataTypeOverride">([^<]*)</P>', raw):
            v = (mm.group(1) or mm.group(2) or b"").decode("utf-8", "replace").strip()
            if v not in _DTO_OK:
                raise SlxError("UNSUPPORTED", "model.DataTypeOverride", "data type override %r (only %s)"
                               % (v, " / ".join(_DTO_OK)))


_IDENT = re.compile(r"(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?|([A-Za-z_]\w*)")


def _mat_scalars(raw):
    """Variables of a model workspace saved in the .slx (simulink/modelWorkspace.mxarray, recordings:
    a MATLAB level-5 data element -- 4 bytes 00 01 'I' 'M', 4 bytes, then one miMATRIX struct whose fields are the
    variables). Returns {name: value text} for real double scalars and row vectors, {name: None} for any other
    field (not read); None when the part cannot be parsed."""
    import struct
    if len(raw) < 16 or raw[:4] != b"\x00\x01IM":
        return None
    nums = {1: "b", 2: "B", 3: "h", 4: "H", 5: "i", 6: "I", 7: "f", 9: "d", 12: "q", 13: "Q"}

    def elem(buf, pos):
        typ, n = struct.unpack_from("<II", buf, pos)
        if typ >> 16:   # small data element: 2-byte type, 2-byte size, 4 bytes of data
            return typ & 0xFFFF, buf[pos + 4:pos + 4 + (typ >> 16)], pos + 8
        end = pos + 8 + n
        if end > len(buf):
            raise ValueError("truncated")
        return typ, buf[pos + 8:end], end + (-n % 8)

    def values(typ, data):
        if typ not in nums:
            raise ValueError("data type %d" % typ)
        f = nums[typ]
        return list(struct.unpack("<%d%s" % (len(data) // struct.calcsize(f), f), data[:len(data) // struct.calcsize(f) * struct.calcsize(f)]))

    def matrix(data):
        _t, flags, q = elem(data, 0)
        cls, cplx = flags[0], bool(flags[1] & 0x08)
        _t, dims, q = elem(data, q)
        dims = values(_t, dims)
        _t, name, q = elem(data, q)
        return cls, cplx, dims, q
    try:
        typ, body, _ = elem(raw, 8)
        if typ != 14:
            return None
        cls, _c, dims, q = matrix(body)
        if cls != 2 or dims != [1, 1]:   # mxSTRUCT_CLASS, 1 x 1
            return None
        _t, fl, q = elem(body, q)
        flen = values(_t, fl)[0]
        _t, names, q = elem(body, q)
        fields = [names[i:i + flen].split(b"\x00")[0].decode("ascii") for i in range(0, len(names), flen)]
        out = {}
        for nm in fields:
            typ, fb, q = elem(body, q)
            out[nm] = None
            if typ != 14 or not fb:
                continue
            fcls, fc_, fd, fq = matrix(fb)
            if fcls != 6 or fc_ or len(fd) != 2 or fd[0] != 1 or fd[1] < 1:   # real double row
                continue
            t2, re_, _ = elem(fb, fq)
            v = [float(x) for x in values(t2, re_)]
            if len(v) != fd[1] or any(x != x or x in (float("inf"), float("-inf")) for x in v):
                continue
            out[nm] = repr(v[0]) if len(v) == 1 else "[" + " ".join(repr(x) for x in v) + "]"
        return out
    except (ValueError, IndexError, struct.error, UnicodeDecodeError):
        return None


def _mask_vars(block):
    """{name: raw value text, or None with the reason in why} of a SubSystem block's mask (
    <Mask><MaskParameter Name= Type="edit"><Value>3</Value>, also with Tunable="off"). Only Edit
    parameters of a mask without initialization
    code were recorded; the others are kept as unusable (a block naming one is refused)."""
    mk = block.find("Mask")
    if mk is None:
        return {}, {}
    init = mk.find("Initialization")
    out, why = {}, {}
    for mp in mk.findall("MaskParameter"):
        nm = mp.get("Name")
        if not nm:
            continue
        v = mp.find("Value")
        # Tunable on is the attribute-free form; Tunable="off" gives the same values and types
        extra = set(mp.attrib) - {"Name", "Type"} - ({"Tunable"} if mp.get("Tunable") == "off" else set())
        if init is not None and (init.text or "").strip():
            out[nm], why[nm] = None, "its mask has initialization code (not recorded)"
        elif mp.get("Type", "edit") != "edit" or extra:
            out[nm], why[nm] = None, "mask parameter of type %r%s (only Edit parameters were recorded)" % (
                mp.get("Type"), (" with " + ", ".join(sorted(extra))) if extra else "")
        else:
            out[nm] = (v.text or "").strip() if v is not None else ""
    return out, why


def _workspace_chain(m, sys):
    """The name scopes around a system, innermost first: the masks of the SubSystem blocks around it (each a
    (vars, why) pair), then the model workspace saved in the file."""
    chain, s = [], sys
    while s is not None and s.up[0] is not None:
        if getattr(s, "mref", None):
            # recordings (silent wrong): a referenced model's names are its own -- the masks and workspace of
            # the parent model are not visible inside it (Simulink: BlkParamUndefined when only the parent has it).
            # recordings (silent wrong): its model arguments take the instance's values -- a scope between its
            # masks and its workspace (the workspace holds only the default), so a mask value naming one gets it
            if getattr(s, "args", None):
                chain.append(({k: str(v) for k, v in s.args.items()}, {}))
            break
        parent, owner = s.up
        chain.append(_mask_vars(parent.blocks[owner]))
        s = parent
    m = getattr(s, "model", None) or m   # the model whose workspace applies (the referenced one inside a Model block)
    if not hasattr(m, "_mws"):
        raw = m.members.get("simulink/modelWorkspace.mxarray")
        vs = _mat_scalars(raw) if raw is not None else {}
        m._mws = ({k: v for k, v in (vs or {}).items()},
                  {k: "a model workspace variable that is not a real double scalar or row" for k, v in (vs or {}).items()
                   if v is None})
        if vs is None:
            m._mws_bad = True
    chain.append(m._mws)
    return chain


def _l2_unresolved_why(text, why):
    """Explain why the reader cannot evaluate an integer parameter in the saved model."""
    import fxnum
    if "%" in text:
        return "it holds a MATLAB comment ('%'), which the reader does not strip"
    m_ = re.search(r"\b([A-Za-z]\w*)\s*\.\s*([A-Za-z]\w*)", text)
    if m_:
        return "%s.%s is a struct field (struct parameters are not read)" % (m_.group(1), m_.group(2))
    for f_ in re.finditer(r"\b([A-Za-z]\w*)\s*\(", text):
        if f_.group(1) not in fxnum.FUNCTIONS:
            return "the function %s(...) is not evaluated (only literal arithmetic and %s)" % (
                f_.group(1), ", ".join(fxnum.FUNCTIONS))
    return why


def _row_elements(text):
    """The elements of a one-row bracket literal '[e1 e2 ...]' by MATLAB's rule (commas, or spaces -- a space before an
    unspaced '+' / '-' starts a new element, 'a - 1' and 'a-1' are one), or None for anything else."""
    t = text.strip()
    if not (t.startswith("[") and t.endswith("]")) or ";" in t:
        return None
    body, out, cur, depth, i = t[1:-1], [], "", 0, 0
    while i < len(body):
        ch = body[i]
        if ch in "([":
            depth += 1
        elif ch in ")]":
            depth -= 1
        if depth == 0 and ch == ",":
            out.append(cur)
            cur = ""
        elif depth == 0 and ch.isspace():
            j = i
            while j < len(body) and body[j].isspace():
                j += 1
            nxt = body[j] if j < len(body) else ""
            prev = cur.rstrip()[-1:] if cur.strip() else ""
            binary = prev in "+-*/^(" or (nxt in "*/^") or (nxt in "+-" and j + 1 < len(body) and
                                                             body[j + 1].isspace())
            if cur.strip() and nxt and not binary:
                out.append(cur)
                cur = ""
            else:
                cur += " "
            i = j
            continue
        else:
            cur += ch
        i += 1
    if cur.strip():
        out.append(cur)
    return [e.strip() for e in out if e.strip()] or None


def _core_index_vector(v):
    """The core's reading of a MATLAB row of integers (ref/fxcore.py _index_vector), or None."""
    import fxcore
    try:
        return fxcore._index_vector(str(v))
    except Exception:   # noqa: BLE001
        return None


def _matlab_int(v):
    """The integer a MATLAB integer form evaluates to ('int8(3)', '0x2', '[2]'; ref/fxcore.py matlab_int_expr), as text,
    or None; true / false are left out (the engine refuses them for integer parameters)."""
    import fxcore
    v1 = str(v).strip()
    if v1.startswith("[") and v1.endswith("]") and v1.count("[") == 1:
        v1 = v1[1:-1].strip()
    if v1 in ("true", "false"):
        return None
    try:
        r_ = fxcore.matlab_int_expr(v1)
    except Exception:   # noqa: BLE001 -- not a form it knows
        return None
    return str(int(r_[0])) if r_ is not None and r_[0].denominator == 1 else None


def _eval_names(text, chain, depth=0):
    """(value text, None) of a parameter that names variables of the scopes in chain (masks around the block, the
    model workspace), each replaced by its value and the result evaluated as literal arithmetic (engine/fxnum.py:
    MATLAB's double arithmetic, exact powers); (None, reason) when a name is not found or not usable, or the result
    is not a number or a row of numbers. A mask value is evaluated in the scopes outside its own mask."""
    import fxnum
    if depth > 16:
        return None, "names nested more than 16 levels"
    els = _row_elements(text)
    if els is not None and any(_IDENT.search(e) and any(x.group(1) for x in _IDENT.finditer(e)) for e in els):
        # In saved models, '[0 NumBits-1]' -- each element of a row evaluated on its own, then the row
        vals = []
        for e in els:
            if _NUM.match(e):
                vals.append(e)
                continue
            v, why = _eval_names(e, chain, depth + 1)
            if v is None and why == "no names":
                v = fxnum._literal_arith(e)
                why = "%r is not literal arithmetic on numbers" % e
            if v is None or not _NUM.match(str(v)):
                return None, why
            try:
                v = str(int(float(v))) if float(v) == int(float(v)) and abs(float(v)) < 2 ** 53 else str(v)
            except (ValueError, OverflowError):
                v = str(v)
            vals.append(v)
        return "[%s]" % " ".join(vals), None
    out, pos, used = [], 0, False
    for mt in _IDENT.finditer(text):
        if not mt.group(1):
            continue
        nm = mt.group(1)
        if nm in fxnum.FUNCTIONS and text[mt.end():].lstrip().startswith("("):
            # a function the literal evaluator knows (nextpow2): kept as written, unless a scope defines a
            # variable of that name (then MATLAB would index the variable -- not read)
            if any(nm in vs for vs, _ in chain):
                return None, "%s names both a variable and a function" % nm
            continue
        for i, (vs, why) in enumerate(chain):
            if nm in vs:
                if vs[nm] is None:
                    return None, "%s: %s" % (nm, why.get(nm, "not usable"))
                lit = None if (_NUM.match(vs[nm]) or _VECNUM.match(vs[nm])) else fxnum._literal_arith(vs[nm])
                v, err = (vs[nm], None) if (_NUM.match(vs[nm]) or _VECNUM.match(vs[nm])) else \
                    (lit, None) if lit is not None and (_NUM.match(lit) or _VECNUM.match(lit)) else \
                    _eval_names(vs[nm], chain[i + 1:], depth + 1)
                if v is None:
                    return None, "%s = %r: %s" % (nm, vs[nm], err)
                break
        else:
            return None, ("%s is not defined in an enclosing mask or the model workspace saved in the file (a "
                          "base-workspace variable is not stored in the .slx)" % nm)
        out.append(text[pos:mt.start()])
        out.append(v.strip() if v.strip().startswith("[") else "(%s)" % v.strip())
        pos, used = mt.end(), True
    if not used:
        return None, "no names"
    t = "".join(out) + text[pos:]
    if _NUM.match(t.strip("() ")) and t.count("(") == t.count(")"):
        r = fxnum._literal_arith(t)
        return (r, None) if r is not None else (None, "%r is not literal arithmetic" % t)
    if _VECNUM.match(t):
        return t.strip(), None
    r = fxnum._literal_arith(t)
    if r is None or not (_NUM.match(r) or _VECNUM.match(r)):
        return None, "%r is not literal arithmetic on numbers" % t
    return r, None


def _read_model(m, scope, refs=None):
    """Returns (design, info, elements) where elements maps design block name -> (part, Block element)."""
    top = _Sys(m, m.root[0], m.root[1], [])
    outer = []   # the scope's own SubSystem block and those around it: (system, sid, system inside the block)
    scope_parent = scope_owner = None
    if scope:
        cur = top
        for name in scope_path(scope):
            sid = cur.byname.get(name)
            if sid is None or cur.btype(sid) != "SubSystem":
                raise SlxError("BAD_PARAM", "scope", "no subsystem %r" % scope)
            part, el = m.subsystem(cur.part, cur.blocks[sid])
            nxt = _Sys(m, part, el, cur.path + [name], cur, sid)
            outer.append((cur, sid, nxt))
            cur = nxt
        top = cur
        scope_parent, scope_owner = top.parent, top.owner
        top.parent = None  # the scope's own Inports/Outports are the design boundary
    _check_model_override(m)
    solver, step = _solver(m)
    if solver is None:
        raise SlxError("UNSUPPORTED", "model", "no solver configuration found: cannot confirm a single-rate "
                       "discrete model (the engine runs every block once per step)")
    if solver not in SUPPORTED_SOLVERS:
        raise SlxError("UNSUPPORTED", "model.SolverName", "solver %s (only %s is supported)"
                       % (solver, ", ".join(SUPPORTED_SOLVERS)))
    base = None
    if step is not None and step.strip() not in ("auto", ""):
        if not _NUM.match(step):
            raise SlxError("UNSUPPORTED", "model.FixedStep", "fixed step %r is not a numeric literal" % step)
        base = float(step)
    blocks, lines, origin, inport_types, assumed, elements = {}, [], {}, {}, {}, {}
    assumed_v = {}   # block -> {parameter: the value the engine used} (second cold start)
    filled = {}
    ignored = {}
    info_sources = {}
    info_sws = {}
    mref_ft = {}   # Model block (flattened name) -> its inputs are direct feedthrough
    named, unresolved = {}, {}   # parameters read through mask / model workspace names; names not found
    systems = []

    referenced = {}

    def walk(sys):
        systems.append(sys)
        for sid, b in sys.blocks.items():
            if ML.is_ml(sys, sid):   # a MATLAB Function block is one design block (engine/fxslx_ml.py)
                ML.check_sf_type(sys, sid)
                continue
            if sys.btype(sid) == "SubSystem":
                part, el = sys.model.subsystem(sys.part, b)
                sub = _Sys(sys.model, part, el, sys.path + [b.get("Name")], sys, sid)
                sub.args, sub.chain = getattr(sys, "args", {}), getattr(sys, "chain", [])
                margs = _foreach_mask_args(sub, b)
                if margs:
                    sub.args = dict(sub.args, **margs)
                walk(sub)
            elif sys.btype(sid) == "ModelReference":   # the referenced model's content under this name
                name, args = MR.instance(sys, sid)
                chain = getattr(sys, "chain", [])
                rm = MR.open_model(refs, name, sys.where(sid), chain)
                if rm.ref_step not in (None, "auto", "") and step not in (None, "auto", "") and \
                        float(rm.ref_step) != float(step):
                    raise SlxError("UNSUPPORTED", sys.where(sid), "referenced model %s has the fixed step %s, the "
                                   "model %s" % (name, rm.ref_step, step))
                declared = MR.model_args(rm)
                if sorted(args) != declared:
                    raise SlxError("UNSUPPORTED_PARAM", sys.where(sid), "instance arguments %s, the referenced model "
                                   "declares %s (every argument must be given by the instance)" % (sorted(args),
                                                                                                    declared))
                referenced[name] = hashlib.sha256(refs(name)).hexdigest()
                sub = _Sys(rm, rm.root[0], rm.root[1], sys.path + [b.get("Name")], sys, sid)
                sub.args, sub.chain, sub.mref = args, chain + [name], name
                # Simulink treats a Model block input as direct feedthrough unless the port goes
                # straight into a delay -- recorded per instance for the loop check below
                ins_ = [x for x in sub.blocks if sub.btype(x) == "Inport"]
                ft_ = any(sub.btype(d_) not in ("UnitDelay", "Delay", "Memory") for (d_, _p), (s_, _sp) in
                          sub.src.items() if s_ in ins_)
                mref_ft["/".join(sys.path[len(top.path):] + [b.get("Name")])] = ft_
                if ft_:
                    # a loop in this system back to the Model block with no delay on it: Simulink stops with
                    # UnableToSolveAlgLoop (the inlined content may hold the delay; it was answered). Other blocks of
                    # the system: a delay breaks the path, anything else (a subsystem, another Model block) carries it
                    nofb_ = ("UnitDelay", "Delay", "Memory", "DiscreteIntegrator", "TappedDelay")
                    nxt_ = {}
                    for (d_, _p), (s_, _sp) in sys.src.items():
                        if sys.btype(d_) not in nofb_:
                            nxt_.setdefault(s_, set()).add(d_)
                    seen_, stack_ = set(), list(nxt_.get(sid, ()))
                    while stack_:
                        x_ = stack_.pop()
                        if x_ == sid:
                            raise SlxError("PORT_TYPE_INVALID", sys.where(sid), "a loop back into the Model block %s "
                                           "with no delay outside it: Simulink treats a Model block input as direct "
                                           "feedthrough (unless the port goes straight into a delay) and stops with "
                                           "UnableToSolveAlgLoop" % b.get("Name"))
                        if x_ in seen_:
                            continue
                        seen_.add(x_)
                        stack_ += list(nxt_.get(x_, ()))
                walk(sub)
    walk(top)
    children = {}
    for s in systems:
        if s.parent is not None:
            children[(id(s.parent), s.owner)] = s

    def name_of(sys, sid):
        return "/".join(sys.path[len(top.path):] + [sys.blocks[sid].get("Name")])

    def label_up(sys, dsid, dport, seen=0):
        """the name of the signal entering input dport of block dsid as Simulink sees it -- the label of
        that line, else the label carried through virtual connections upstream: a subsystem Inport (the line into the
        subsystem's port), a subsystem's output (the line into its Outport, also atomic / enabled subsystems), a From
        (the line into its local or global Goto). A real block in between ends the search (None)."""
        if seen > 64:
            return None
        nm = sys.signal_name.get((dsid, dport))
        if nm:
            return nm
        src = sys.src.get((dsid, dport))
        if src is None:
            return None
        s_, sp_ = src
        bt_ = sys.btype(s_)
        if bt_ == "Inport" and sys.parent is not None:
            return label_up(sys.parent, sys.owner, sys.port_no(s_), seen + 1)
        if bt_ == "SubSystem" and (id(sys), s_) in children:
            inner = children[(id(sys), s_)]
            outs = [x for x in inner.blocks if inner.btype(x) == "Outport" and inner.port_no(x) == sp_]
            return label_up(inner, outs[0], 1, seen + 1) if len(outs) == 1 else None
        if bt_ == "From":
            tag = _params(sys.blocks[s_]).get("GotoTag", "")
            here = [x for x in sys.blocks if sys.btype(x) == "Goto" and _params(sys.blocks[x]).get("GotoTag", "") == tag
                    and _params(sys.blocks[x]).get("TagVisibility", "local") == "local"]
            glob = [(s2, x) for s2 in systems for x in s2.blocks if s2.btype(x) == "Goto"
                    and _params(s2.blocks[x]).get("GotoTag", "") == tag
                    and _params(s2.blocks[x]).get("TagVisibility", "local") == "global"]
            if len(here) == 1:
                return label_up(sys, here[0], 1, seen + 1)
            if not here and len(glob) == 1:
                return label_up(glob[0][0], glob[0][1], 1, seen + 1)
        return None

    def resolve(sys, sid, port, seen=0):
        """Real (system, sid) driving output `port` of block sid in sys, after removing virtual ports."""
        if seen > 256:
            raise SlxError("BAD_SLX", sys.where(sid), "subsystem port chain too deep")
        bt = sys.btype(sid)
        if ML.is_ml(sys, sid):
            return (sys, sid) if port == 1 else (sys, sid, port)
        if bt in ("SubSystem", "ModelReference"):
            inner = children[(id(sys), sid)]
            outs = [x for x in inner.blocks if inner.btype(x) == "Outport" and inner.port_no(x) == port]
            if len(outs) != 1:
                raise SlxError("BAD_SLX", sys.where(sid), "subsystem output %d has no unique Outport" % port)
            if inner.enabled:
                return (inner, outs[0])   # the enabled subsystem's output block itself (EnabledOutport)
            src = inner.src.get((outs[0], 1))
            if src is None:
                raise SlxError("BAD_CONNECTION", inner.where(outs[0]), "Outport is not connected")
            return resolve(inner, src[0], src[1], seen + 1)
        if bt == "Inport" and sys.parent is not None:
            pn = sys.port_no(sid)
            src = sys.parent.src.get((sys.owner, pn))
            if src is None:
                raise SlxError("BAD_CONNECTION", sys.parent.where(sys.owner), "subsystem input %d is not connected" % pn)
            return resolve(sys.parent, src[0], src[1], seen + 1)
        if bt in ("BusSelector", "ComplexToRealImag", "SwitchCase", "If", "PreLookup", "ComplexToMagnitudeAngle"):   # recordings; recordings
            return (sys, sid) if port == 1 else (sys, sid, port)
        hdl_hit = bt == "Reference" and port >= 2 and \
            _params(sys.blocks[sid]).get("SourceBlock", "").split("/")[-1].replace("\n", " ") in (
                "HDL Counter", "Dual Port RAM", "Dual Port RAM System", "Discrete FIR Filter", "FIR Decimator", "NCO",
                "HDL FIFO")
        # count_hit; the Dual Port RAM's read port; the DSP HDL filters' valid, the NCO's second / third
        # output; a port the block does not have is refused when the block is lowered
        if port != 1 and bt != "Demux" and not hdl_hit:
            raise SlxError("UNSUPPORTED", sys.where(sid), "output port %d (only single-output blocks supported)" % port)
        return (sys, sid) if port == 1 else (sys, sid, port)

    mref_checks = []
    mref_scalar_inports = []   # a referenced model's root Inport at -1 is scalar there
    explicit = set()
    counter_own = set()   # whole periods > 1 on counters' tsamp (checked after the walk)
    typed_outports = []
    typed_inports = []
    around = set()   # periods of the blocks around a scope

    def surroundings(sys, sid, cone):
        """Checks on a block outside the design that still decides how the design runs (a
        "scope" read looked at nothing outside the subsystem): the scope's SubSystem block, the subsystems around
        it, and (cone) the blocks its inputs are computed from, whose sample time the scope's Inports inherit."""
        b = sys.blocks[sid]
        bt, own, where_ = sys.btype(sid), _params(b), sys.where(sid)
        if str(own.get("Commented", "off")).strip() != "off":
            raise SlxError("UNSUPPORTED", where_ + ".Commented", "commented-out block (Commented = %r) %s the scope"
                           % (own.get("Commented"), "driving" if cone else "around"))
        if bt == "SubSystem" and (own.get("Variant") == "on" or own.get("VariantControl")):
            raise SlxError("UNSUPPORTED_BLOCK", where_, "variant subsystems are not supported")
        if str(own.get("DataTypeOverride", "Off")).strip() not in _DTO_OK:
            raise SlxError("UNSUPPORTED", where_ + ".DataTypeOverride", "data type override %r (only %s)"
                           % (own["DataTypeOverride"], " / ".join(_DTO_OK)))
        st = own.get("SampleTime", own.get("SystemSampleTime"))
        if st is None and "SampleTime" not in m.defaults.get(bt, {}):
            st = _OMITTED_CHECK.get(bt)
        for st_ in (st, own.get("tsamp")):
            mm = _ST.match(st_ or "")
            if st_ is None or (cone and mm and mm.group(1) not in ("inf", "Inf") and float(mm.group(1)) == 0
                               and not (mm.group(5) and float(mm.group(5)) != 0)):
                continue   # a continuous source under the fixed-step discrete solver runs at the model step
            try:
                _check_sample_time(where_, st_, base, "Constant" if cone else bt)
            except SlxError as e:
                if not cone:
                    raise
                raise SlxError(e.code, e.where, "%s -- this block is outside the scope and an input of the scope "
                               "is computed from it, so the scope does not run as a single-rate design on its own; "
                               "read the whole model" % e.message)
            if mm and float(mm.group(1).replace("Inf", "inf")) > 0:
                # kept apart from the design's own periods: these do not reach the engine
                around.add(float(mm.group(1).replace("Inf", "inf")))

    for osys, osid, inner in outer:
        surroundings(osys, osid, False)
        if inner is not top:
            bad = sorted(inner.btype(x) for x in inner.blocks if inner.btype(x) in _CONDITIONAL)
            if bad:
                raise SlxError("UNSUPPORTED", osys.where(osid), "the scope lies inside a conditionally executed or "
                               "iterated subsystem (%s)" % bad[0])
    # A scope whose inputs are each driven directly by a block with its own written sample time, and those differ
    # from the model step or from each other (or carry an offset): the scope is a multirate design -- the driver's
    # sample time is carried onto the scope's Inport (the model is replayed this way; the checkers used to add
    # the periods by hand). Any other surrounding that changes how the scope runs is refused.
    carried = {}
    if scope_parent is not None:
        direct = {}
        for (d_, p_), (s_, _sp) in scope_parent.src.items():
            if d_ == scope_owner and scope_parent.btype(s_) not in ("SubSystem", "Inport", "From", "DataStoreRead"):
                o_ = _params(scope_parent.blocks[s_])
                mm_ = _ST.match(o_.get("SampleTime") or "")
                if mm_ and mm_.group(1) not in ("inf", "Inf") and float(mm_.group(1)) > 0 and \
                        str(o_.get("Commented", "off")).strip() == "off":
                    direct[p_] = (o_["SampleTime"].strip(), float(mm_.group(1)), float(mm_.group(5) or 0))
        ports_ = [top.port_no(x) for x in top.blocks if top.btype(x) == "Inport"]
        if ports_ and all(p_ in direct for p_ in ports_) and (
                any(v[2] != 0 for v in direct.values()) or len({v[1] for v in direct.values()}) > 1 or
                (base is not None and any(v[1] != base for v in direct.values()))):
            carried = {p_: v[0] for p_, v in direct.items()}
    if scope_parent is not None and not carried:
        lazy, seen_up = {}, set()

        def up(sys, sid, port, depth=0):
            if (id(sys), sid, port) in seen_up:
                return
            seen_up.add((id(sys), sid, port))
            if depth > 4096:
                raise SlxError("BAD_SLX", sys.where(sid), "signal chain too deep")
            bt = sys.btype(sid)
            if bt == "SubSystem":
                surroundings(sys, sid, True)
                key = (id(sys), sid)
                if key not in lazy:
                    part_, el_ = m.subsystem(sys.part, sys.blocks[sid])
                    lazy[key] = _Sys(m, part_, el_, sys.path + [sys.blocks[sid].get("Name")], sys, sid)
                inner = lazy[key]
                bad = sorted(inner.btype(x) for x in inner.blocks if inner.btype(x) in _CONDITIONAL)
                if bad:
                    raise SlxError("UNSUPPORTED", sys.where(sid), "a scope input is computed in a conditionally "
                                   "executed or iterated subsystem (%s): its sample hits cannot be traced" % bad[0])
                for x in inner.blocks:
                    if inner.btype(x) == "Outport" and inner.port_no(x) == port and (x, 1) in inner.src:
                        up(inner, *inner.src[(x, 1)], depth=depth + 1)
                return
            if bt == "Inport" and sys.parent is not None:
                src = sys.parent.src.get((sys.owner, sys.port_no(sid)))
                if src is not None:
                    up(sys.parent, src[0], src[1], depth + 1)
                return
            if bt in ("From", "DataStoreRead"):
                raise SlxError("UNSUPPORTED", sys.where(sid), "a scope input is computed through a %s block outside "
                               "the scope: its sample time cannot be traced (read the whole model, or wire the "
                               "signal directly)" % bt)
            surroundings(sys, sid, True)
            for (d_, _p), (s_, sp_) in sorted(sys.src.items(), key=_src_key):
                if d_ == sid:
                    up(sys, s_, sp_, depth + 1)

        import sys as _pysys
        lim = _pysys.getrecursionlimit()
        _pysys.setrecursionlimit(max(lim, 20000))
        try:
            for (d_, _p), (s_, sp_) in sorted(scope_parent.src.items(), key=_src_key):
                if d_ == scope_owner:
                    up(scope_parent, s_, sp_)
        finally:
            _pysys.setrecursionlimit(lim)

    for sys in systems:
        for sid, b in sys.blocks.items():
            bt = sys.btype(sid)
            own = _params(b)
            if "/" in (b.get("Name") or ""):
                # design names are paths joined by '/', a '/' inside a name collides with them
                raise SlxError("UNSUPPORTED", sys.where(sid), "block name contains '/': rename the block (design "
                               "block names are subsystem paths joined by '/')")
            if str(own.get("DataTypeOverride", "Off")).strip() not in _DTO_OK:
                raise SlxError("UNSUPPORTED", sys.where(sid) + ".DataTypeOverride", "data type override %r (only %s)"
                               % (own["DataTypeOverride"], " / ".join(_DTO_OK)))
            if bt == "SubSystem":
                extra_ = sorted(k for k in own if k not in COSMETIC and k not in _SUBSYS_READ)
                if extra_:   # F3: a subsystem's own parameters never reached ignored_parameters
                    ignored[name_of(sys, sid)] = extra_
            if bt == "Inport" and sys.parent is not None and "OutDataTypeStr" in own and \
                    not str(own["OutDataTypeStr"]).strip().startswith("Inherit"):
                # F4: a typed Inport of a flattened subsystem is the same constraint as a typed Outport
                typed_inports.append((sys, sid, str(own["OutDataTypeStr"]).strip()))
            st = own.get("SampleTime", own.get("SystemSampleTime"))
            if bt == "ZeroOrderHold":
                st = None   # its SampleTime is the hold's output rate (engine/fxzoh.py checks it)
            if st is None and "SampleTime" not in sys.model.defaults.get(bt, {}) and bt in OMITTED_SAMPLE_TIME:
                # Simulink's value for the omitted parameter (R2026b
                # get_param gives 1 on all 124 Unit Delays a saved model left without one) -- a real period, so
                # it must match the base step like an explicit one
                st = _OMITTED_CHECK[bt]
            # a counter's period is its "tsamp", not SampleTime -- it takes part in the
            # single-rate check like any other period (a counter at 1 among blocks at 2 was read as single-rate)
            for k_st, st_ in enumerate((st, own.get("tsamp"))):
                if st_ is None:
                    continue
                _check_sample_time(sys.where(sid), st_, base, bt)
                if _ST.match(st_) and float(_ST.match(st_).group(1).replace("Inf", "inf")) > 0:
                    if k_st == 1 and re.match(r"^\s*[2-9]\s*$|^\s*[1-9][0-9]+\s*$", st_):
                        # a counter's own whole period other than 1: kept apart and checked once the design's
                        # blocks are known -- admitted with a rate-changing block in the design (a multirate
                        # design: the engine runs the counter at its tsamp), refused as a second rate otherwise
                        counter_own.add(float(st_))
                    else:
                        explicit.add(float(_ST.match(st_).group(1)))
            # recordings (runs/recordings): parameters that change Simulink's result but were dropped
            where_ = sys.where(sid)
            if str(own.get("Commented", "off")).strip() != "off":
                # 'on' removes the block (its readers see 0), 'through' wires input to output: not modelled
                raise SlxError("UNSUPPORTED", where_ + ".Commented", "commented-out block (Commented = %r): "
                               "uncomment or delete it in the model" % own.get("Commented"))
            if bt in ("RelationalOperator", "Logic") and "OutDataTypeStr" in own and \
                    str(own["OutDataTypeStr"]).strip() != "boolean" and \
                    str(own["OutDataTypeStr"]).replace(" ", "") not in G.LOGIC_OUT_TYPES:
                raise SlxError("UNSUPPORTED", where_ + ".OutDataTypeStr", "%s output type %r (only boolean is "
                               "recorded)" % (bt, own["OutDataTypeStr"]))
            if bt == "Outport" and "OutDataTypeStr" in own and \
                    not str(own["OutDataTypeStr"]).strip().startswith("Inherit"):
                # a port does not convert: its declared type must be the compiled type of its signal -- carried to
                # the engine, which checks it after compile (recorded, recordings; it was refused outright)
                typed_outports.append((sys, sid, str(own["OutDataTypeStr"]).strip()))
            for k_, ok_ in (("filtFrom", "Dialog parameters"), ("FactorSource", "Dialog parameter"),
                            ("inputOffset", "0")):
                if k_ in own and str(own[k_]).strip() != ok_:
                    raise SlxError("UNSUPPORTED", where_ + "." + k_, "%s = %r (only %r is recorded)"
                                   % (k_, own[k_], ok_))
            if bt == "SubSystem" and (own.get("Variant") == "on" or own.get("VariantControl")):
                raise SlxError("UNSUPPORTED_BLOCK", sys.where(sid), "variant subsystems are not supported")
    explicit.discard(float("inf"))
    around.discard(float("inf"))
    if base is None and len(explicit | around) > 1:
        raise SlxError("UNSUPPORTED", "model", "several sample times %s%s: multi-rate models are not supported"
                       % (sorted(explicit | around), " (in the scope and in the blocks its inputs are computed "
                          "from)" if around - explicit else ""))
    # A Discrete-Time Integrator (integration method) that inherits its sample time integrates with the model's
    # step T. The engine takes T = 1 unless a period is written; a model whose step is not 1 (FixedStep != 1, or
    # FixedStep auto with no period anywhere: Simulink then uses stop time / 50) would integrate wrongly -- refused
    # (not recorded). A DTI with its own SampleTime, or accumulation, is not affected.
    period = base if base is not None else (next(iter(explicit)) if explicit else None)
    for sys_ in systems:
        for sid, b in sys_.blocks.items():
            if sys_.btype(sid) != "DiscreteIntegrator":
                continue
            own = _params(b)
            st_ = str(own.get("SampleTime", _OMITTED_CHECK["DiscreteIntegrator"])).strip()
            meth = str(own.get("IntegratorMethod", "Integration: Forward Euler"))
            # an explicit period on any block reaches the engine (Design takes it as T); only a
            # model with no period anywhere leaves T to the model step
            # the export's own model feeds the DUT from From Workspace blocks at their periods (around);
            # when every such period is 1 the integrator inherits step 1 -- the recorded case (it was refused there,
            # so a Discrete-Time Integrator > Downsample package failed its own round trip)
            if st_ == "-1" and meth.startswith("Integration") and not explicit and period != 1 and \
                    not (around and all(float(x) == 1.0 for x in around)):
                raise SlxError("UNSUPPORTED", sys_.where(sid) + ".SampleTime", "Discrete-Time Integrator "
                               "(integration) inheriting its sample time in a model whose step is %s: write the "
                               "block's SampleTime or FixedStep 1 (T = step is not recorded)"
                               % ("not given (FixedStep auto with no period: Simulink uses stop time / 50)"
                                  if period is None else period))
    for sys in systems:
        for sid, b in sys.blocks.items():
            bt = sys.btype(sid)
            if ML.is_ml(sys, sid):
                name = name_of(sys, sid)
                if name in blocks:
                    raise SlxError("BAD_SLX", sys.where(sid), "duplicate block name after flattening: %s" % name)
                if getattr(sys, "mref", None) or getattr(sys, "chain", None):
                    raise SlxError("UNSUPPORTED", sys.where(sid), "a MATLAB Function inside a referenced model")
                blocks[name], _n, mlp = ML.block(m, sys, sid, "/".join(sys.path + [b.get("Name")]))
                if mlp is not None:
                    # a chart updated DISCRETE at its own sample time: it must be the model's single rate (the
                    # engine runs every block once per step; T = 1 when nothing writes a period)
                    _check_sample_time(sys.where(sid), mlp, base, "MATLABFunction")
                    mm_ = _ST.match(mlp)
                    if mm_ is None or float(mm_.group(1)) != (period if period is not None else 1.0):
                        raise SlxError("UNSUPPORTED_PARAM", sys.where(sid) + ".sampleTime", "MATLAB Function "
                                       "chart sample time %s differs from the model's step %s: multi-rate models "
                                       "are not supported" % (mlp, period if period is not None else 1))
                origin[name] = {"part": sys.part, "sid": sid, "path": "/".join(sys.path + [b.get("Name")])}
                elements[name] = (sys.part, b)
                continue
            if bt in ("SubSystem", "ModelReference"):
                continue
            if bt in ("Inport", "Outport") and str(_params(b).get("SignalType", "auto")).strip() in ("real", "complex") \
                    and sys.parent is not None:
                # a subsystem port's SignalType is a check Simulink makes against the signal's
                # complexity (InputPortComplexSignalMismatch); the flattened read does not keep the port -- refused
                raise SlxError("UNSUPPORTED", sys.where(sid) + ".SignalType", "a subsystem %s with SignalType %r (the "
                               "complexity check is not modelled; use 'auto')" % (bt, _params(b).get("SignalType")))
            if bt in ("Inport", "Outport") and sys.parent is not None:
                for k_, v_ in sorted(_params(b).items()):
                    if G.DESIGN_RANGE_RE.match(k_) and str(v_).strip() not in ("[]", ""):
                        # the port of a flattened subsystem or of a referenced model goes away
                        # with its design range (Simulink checks it: PortDesignMax/MinPropagationIsOutOfDTRange)
                        raise SlxError("UNSUPPORTED", sys.where(sid) + "." + k_, "design range %s = %r on a %s %s is "
                                       "not supported: Simulink uses design minimum / maximum values to check the "
                                       "signal, which this engine does not model; leave it '[]'"
                                       % (k_, v_, "referenced model's" if getattr(sys, "mref", None) else "subsystem",
                                          bt))
            if bt == "Inport" and getattr(sys, "mref", None) and sys.parent is not None:
                # root Inport of a referenced model: a declared type, checked against the driving signal
                dt = _params(b).get("OutDataTypeStr", "Inherit: auto").strip()
                if dt.startswith("Inherit") or G._parse_fixdt(dt) is None and dt not in G.fc._NAMED_INT:
                    raise SlxError("UNSUPPORTED", sys.where(sid), "root Inport of the referenced model %s has the type "
                                   "%r: a referenced model is compiled on its own and needs declared Inport types "
                                   "(Simulink refuses this model with InputPortDataTypeMismatch when the inherited "
                                   "type differs)" % (sys.mref, dt))
                pd_ = str(_params(b).get("PortDimensions", "-1")).strip()
                mref_scalar_inports.append((name_of(sys, sid), sys.where(sid), pd_))
                if pd_ not in ("-1", ""):
                    # Simulink checks a declared dimension against the driving signal
                    # (PortDimsMismatch) -- not modelled for a referenced model's port; was answered
                    raise SlxError("UNSUPPORTED", sys.where(sid) + ".PortDimensions", "root Inport of the referenced "
                                   "model %s declares PortDimensions %s: the dimension check against the driving "
                                   "signal is not modelled; leave it -1 (inherited)" % (sys.mref, pd_))
                nm = name_of(sys, sid)
                blocks[nm] = {"type": "Constant", "Value": "0", "OutDataTypeStr": dt}
                blocks[nm + " (type check)"] = {"type": "DataTypeDuplicate", "NumInputPorts": "2"}
                pn = sys.port_no(sid)
                r = resolve(sys.parent, *sys.parent.src[(sys.owner, pn)]) if (sys.owner, pn) in sys.parent.src \
                    else None
                if r is None:
                    raise SlxError("BAD_CONNECTION", sys.parent.where(sys.owner), "model input %d is not connected"
                                   % pn)
                mref_checks.append((nm, r))
                origin[nm] = {"part": sys.part, "sid": sid, "path": "/".join(sys.path + [b.get("Name")]),
                              "model": sys.mref, "role": "Inport type check"}
                continue
            if bt in ("Inport", "Outport") and sys.parent is not None and _foreach_of(sys):
                _foreach_port(sys, sid, bt)   # element-wise partition / concatenation only
            if bt == "ForEach":
                name = name_of(sys, sid)
                blocks[name] = _foreach_block(sys, sid)
                origin[name] = {"part": sys.part, "sid": sid, "path": "/".join(sys.path + [b.get("Name")])}
                elements[name] = (sys.part, b)
                continue
            if bt in ("Inport", "Outport") and sys.parent is not None and not (bt == "Outport" and sys.enabled):
                continue  # virtual port of a flattened subsystem
            if bt == "Terminator":
                continue   # a sink for an unused signal: no effect
            if bt == "EnablePort" and sys.parent is None:
                raise SlxError("UNSUPPORTED", sys.where(sid), "the chosen scope is itself an enabled subsystem: its enable "
                               "comes from outside the design; choose the enclosing system as the scope")
            if bt == "Outport" and sys.enabled and sys.parent is not None:
                bt = "EnabledOutport"
            own = _params(b)
            if bt == "Reference":
                src = own.get("SourceBlock", "")
                lib = LIBRARY.get(src.split("/")[-1].replace("\n", " "))
                if lib is None:
                    name_ = src.split("/")[-1].replace("\n", " ")
                    if name_ in ("Sine", "Cosine"):
                        raise SlxError("UNSUPPORTED_BLOCK", sys.where(sid), "%s is not supported yet " % name_)
                    raise SlxError("UNSUPPORTED_BLOCK", sys.where(sid), "library block %s is not supported" % (src or "?"))
                et, keys = lib
                if et in ("HDLFIRFilter", "HDLFIRDecimator") and not src.startswith("dsphdlfiltering2/"):
                    raise SlxError("UNSUPPORTED_BLOCK", sys.where(sid), "library block %s (the DSP HDL filters are "
                                   "read from dsphdlfiltering2 only)" % src)
                if src.split("/")[-1].replace("\n", " ") == "Signal From Workspace":
                    # a test-bench source like From Workspace -- read as an
                    # Inport whose data the request gives; sample based, one sample per frame, a literal period only
                    nsamps = str(own.get("nsamps", "1")).strip()
                    if nsamps != "1":
                        raise SlxError("UNSUPPORTED_PARAM", sys.where(sid) + ".nsamps", "Signal From Workspace with "
                                       "%s samples per frame (frame-based output) is not supported" % nsamps)
                    ts = str(_time_text(own.get("Ts", "1"))).strip()
                    try:
                        if float(ts) <= 0:
                            raise ValueError
                    except ValueError:
                        raise SlxError("UNSUPPORTED_PARAM", sys.where(sid) + ".Ts", "Signal From Workspace sample "
                                       "time %r: a positive numeric literal is required" % ts)
                    info_sources.setdefault(name_of(sys, sid), own.get("X", ""))
                    # the output is X's type; at i*Ts it is X(i), after the data 0 / the last
                    # value / X(i mod n) by OutputAfterFinalValue -- reported so the caller can give the run's input
                    info_sws[name_of(sys, sid)] = {"Ts": ts, "OutputAfterFinalValue": str(own.get(
                        "OutputAfterFinalValue", "Setting to zero")).strip()}
                    own = {"SampleTime": ts} if ts != "1" else {}
                if et == "HDLNCO":
                    if src not in NCO_LIBS:
                        raise SlxError("UNSUPPORTED_BLOCK", sys.where(sid), "library block %s (the DSP HDL NCO is read "
                                       "from %s only; dspsrcs4/NCO is another block)" % (src, " / ".join(NCO_LIBS)))
                    if "PhaseIncrementSource" not in own:   # the linked library block's own default
                        own = dict(own, PhaseIncrementSource=NCO_LIBS[src])
            elif bt not in TYPEMAP:
                raise SlxError("UNSUPPORTED_BLOCK", sys.where(sid), "block type %s is not supported" % bt)
            else:
                et, keys = TYPEMAP[bt]
            if getattr(sys, "args", None):
                # recordings (silent wrong): a mask parameter of a subsystem inside the referenced model is
                # nearer than the model argument of the same name -- it is not substituted there
                near_, s_ = set(), sys
                while s_ is not None and s_.up[0] is not None and not getattr(s_, "mref", None):
                    near_ |= set(_mask_vars(s_.up[0].blocks[s_.up[1]])[0] or {})   # masks inside the referenced model
                    s_ = s_.up[0]
                # recordings (silent wrong; class: anything Simulink derives from a model argument when it
                # compiles the referenced model, which it does once, with the argument's DEFAULT value -- an
                # inherited parameter type, best-precision scaling): the names this block's parameters use that are
                # model arguments, directly or through a mask value inside the referenced model
                dep_, s_ = {k for k in sys.args if k not in near_}, sys
                masks_ = []
                while s_ is not None and s_.up[0] is not None and not getattr(s_, "mref", None):
                    masks_.append(_mask_vars(s_.up[0].blocks[s_.up[1]])[0] or {})
                    s_ = s_.up[0]
                for _ in range(8):
                    for mv_ in masks_:
                        for nm_, txt_ in mv_.items():
                            if nm_ not in dep_ and any(x.group(1) in dep_ for x in _IDENT.finditer(str(txt_ or ""))
                                                       if x.group(1)):
                                dep_.add(nm_)
                used_ = sorted(k for k, v in own.items() if any(x.group(1) in dep_ for x in _IDENT.finditer(str(v))
                                                                  if x.group(1)))
                for k_ in used_:
                    pt_ = str(own.get("ParamDataTypeStr", "Inherit: Inherit via internal rule")).strip()
                    explicit_ = G._parse_fixdt(pt_) is not None or pt_ in G.fc._NAMED_INT
                    if not (bt == "Gain" and k_ == "Gain" and explicit_):
                        # recorded: a Gain whose ParamDataTypeStr is explicit -- the
                        # instance value is then quantised into that type, as Simulink does
                        raise SlxError("UNSUPPORTED", sys.where(sid) + "." + k_, "%s %s=%r uses the model argument(s) "
                                       "%s: Simulink compiles the referenced model once, deriving types from the "
                                       "argument's default value, which is not modelled -- recorded only for a Gain "
                                       "with an explicit ParamDataTypeStr; give the parameter type explicitly"
                                       % (bt, k_, own[k_], ", ".join(sorted(dep_ & {x.group(1) for x in _IDENT.finditer(
                                           str(own[k_])) if x.group(1)}))))
                own = MR.substitute(own, {k: v for k, v in sys.args.items() if k not in near_}, sys.where(sid))
            p = dict(SLX_DEFAULTS.get(bt, {}))
            p.update(sys.model.defaults.get(bt, {}))
            p.update(own)
            for k in SLX_DEFAULTS.get(bt, {}):
                if k not in own and k not in sys.model.defaults.get(bt, {}):
                    filled.setdefault(name_of(sys, sid), []).append(k)
            for k, v in MUST.get(bt, {}).items():
                if p.get(k, v) != v:
                    raise SlxError("UNSUPPORTED_PARAM", sys.where(sid) + "." + k, "%s=%r (only %r is supported)" % (k, p[k], v))
            name0 = name_of(sys, sid)
            for k in keys:
                if k not in p and k in G.DEFAULTS.get(et, {}):
                    assumed.setdefault(name0, []).append(k)
                    assumed_v.setdefault(name0, {})[k] = str(G.DEFAULTS[et][k])

            if (bt in HAS_SAMPLE_TIME or bt == "DiscreteIntegrator") and "SampleTime" not in p:
                assumed.setdefault(name0, []).append("SampleTime")
                assumed_v.setdefault(name0, {})["SampleTime"] = OMITTED_SAMPLE_TIME.get(
                    bt, "-1 (inherited; Simulink's value for an omitted one not recorded for this block type)")
            skip = set(keys) | COSMETIC | {"SampleTime"} | set(MUST.get(bt, {})) | set(G.META_VALUES)  # carried
            extra = sorted(k for k in own if k not in skip)
            if extra:
                ignored[name0] = extra
            # a numeric parameter naming mask parameters of the enclosing subsystems or
            # model workspace variables saved in the file is read as its value
            import parameter_bounds as _l2b    # recorded HDL blocks: their integer parameters (CountWordLen, lidx, ...)
            # (a lookup's engine type is chosen later by the adapter -- its table / breakpoint keys
            # are those of both lookup types)
            nkeys = set(G.NUMERIC_VALUES.get(et, {})) | (set(G.NUMERIC_VALUES["Lookup1D"]) |
                                                         set(G.NUMERIC_VALUES["Lookup2D"]) if bt == "Lookup_n-D" else set())
            for k in sorted(set(NUMERIC.get(bt, ())) | nkeys | set(_l2b.NAMED_NUMERIC.get(et, ()))):
                v = p.get(k)
                if v is None or not _IDENT.search(v) or not any(x.group(1) for x in _IDENT.finditer(v)):
                    continue
                chain = _workspace_chain(m, sys)
                val, why = _eval_names(v, chain)
                if val is not None and k in _l2b.NAMED_NUMERIC.get(et, ()):
                    # recorded integer parameters take integer text (the workspace stores doubles: n = 8 is '8.0');
                    # a value that is not an integer stays as it is and is refused by the bound checks
                    try:
                        fv = float(val)
                        if fv == int(fv):
                            val = str(int(fv))
                    except (ValueError, OverflowError):
                        pass
                if val is not None:
                    p[k] = val
                    if k in own:
                        own[k] = val
                    named.setdefault(name_of(sys, sid), {})[k] = [v, val]
                elif why != "no names" and any(x.group(1) in vs for x in _IDENT.finditer(v) if x.group(1)
                                               for vs, _w in chain):
                    raise SlxError("UNSUPPORTED_PARAM", sys.where(sid) + "." + k, "parameter %s=%r: %s" % (
                        k, v, _l2_unresolved_why(v, why) if k in _l2b.NAMED_NUMERIC.get(et, ()) else why))
                elif et == "ExtractBits" and k == "bitIdxRange" and _core_index_vector(v) is not None:
                    # recordings (a previous version, ref/fxcore.py _index_vector): the core evaluates the
                    # MATLAB forms of a range ('[int8(1) 2]', '[0x1 2]', '[true 2]' ...) element by element; a text it
                    # reads is passed on unchanged (names it cannot read keep the reader's refusal and its reason)
                    pass
                elif k in _l2b.NAMED_NUMERIC.get(et, ()) and _matlab_int(v) is not None:
                    # numBits 'int8(3)' -- the MATLAB
                    # integer forms main reads (ref/fxcore.py matlab_int_expr) apply to the recorded parameters too
                    val = _matlab_int(v)
                    p[k] = val
                    if k in own:
                        own[k] = val
                elif k in _l2b.NAMED_NUMERIC.get(et, ()):
                    # recordings (the error reporting contract): a recorded integer parameter
                    # whose names cannot be resolved says why, not "integer literal required"
                    raise SlxError("UNSUPPORTED_PARAM", sys.where(sid) + "." + k, "parameter %s=%r is not evaluated: "
                                   "%s; write the number" % (k, v, _l2_unresolved_why(v, why)))
                else:
                    unresolved.setdefault(name_of(sys, sid), {})[k] = why
            for k in NUMERIC.get(bt, ()):
                v = p.get(k)
                cplx_ok = bt in ("Gain", "UnitDelay") and v is not None and _CPLXNUM.match(v)   # recorded: complex literals
                if v is not None and not _NUM.match(v) and not (k in VECTOR_OK.get(bt, ()) and _VECNUM.match(v)) \
                        and not cplx_ok:
                    # coverage per block: plain arithmetic of numeric literals (2^14, 1/3,
                    # [1 2]*0.5) is read as its value; names and functions stay refused
                    import fxnum
                    lit = fxnum._literal_arith(v)
                    if lit is not None and (_NUM.match(lit) or (k in VECTOR_OK.get(bt, ()) and _VECNUM.match(lit))):
                        p[k] = lit
                        if k in own:
                            own[k] = lit
                        v = lit
                if v is not None and not _NUM.match(v) and not (k in VECTOR_OK.get(bt, ()) and _VECNUM.match(v)):
                    # integer forms MATLAB evaluates -- 0x2,
                    # uint8(4), '3;', '[2]' -- are read as their value; true / false are left to the engine (refused)
                    import fxcore
                    v1 = v.strip()
                    if v1.startswith("[") and v1.endswith("]") and v1.count("[") == 1:
                        v1 = v1[1:-1].strip()
                    r_ = fxcore.matlab_int_expr(v1) if v1 not in ("true", "false") else None
                    if r_ is not None and r_[0].denominator == 1:
                        lit = str(int(r_[0]))
                        p[k] = lit
                        if k in own:
                            own[k] = lit
                        v = lit
                if v is not None and not _NUM.match(v) and not (k in VECTOR_OK.get(bt, ()) and _VECNUM.match(v)) \
                        and not cplx_ok:
                    why_ = (unresolved.get(name_of(sys, sid)) or {}).get(k)
                    raise SlxError("UNSUPPORTED_PARAM", sys.where(sid) + "." + k,
                                   "parameter %s=%r is an expression; only numeric literals%s are supported%s"
                                   % (k, v, ", names of enclosing mask parameters and saved model workspace variables",
                                      (" (%s)" % why_) if why_ and why_ != "no names" else ""))
            if bt == "BusCreator":
                # bus element names: the names of the signals feeding the Bus Creator, carried
                # through subsystem ports and Goto / From as Simulink does (7 / 7 swapped when only the
                # line at the Bus Creator was read); an unnamed input is Simulink's signal<j>
                k = int(p.get("Inputs", "2"))
                nms = [label_up(sys, sid, j) for j in range(1, k + 1)]
                if any(nms):
                    own = dict(own, ElementNames=",".join(nm or "signal%d" % j for j, nm in enumerate(nms, 1)))
                    p["ElementNames"] = own["ElementNames"]
            if bt == "UnitDelay" and "InitialCondition" not in p and "X0" in p:
                p["InitialCondition"] = p["X0"]
            if bt == "FromWorkspace":
                sources = info_sources.setdefault(name_of(sys, sid), p.get("VariableName", ""))
            if bt == "Inport" and p.get("OutDataTypeStr"):
                t = G._parse_fixdt(p["OutDataTypeStr"])
                if t:
                    inport_types[name_of(sys, sid)] = {"s": t[0], "wl": t[1], "fl": t[2]}
            nb = {"type": et}
            if et == "Inport" and str(p.get("SampleTime", "-1")).strip() not in ("-1", ""):
                nb["SampleTime"] = str(p["SampleTime"]).strip()  # multirate designs need the input period 
            elif bt == "Inport" and sys is top and carried.get(sys.port_no(sid)):
                nb["SampleTime"] = carried[sys.port_no(sid)]   # the period of the block driving the scope input
            for k in keys:
                if k in p and not (et == "Inport" and k in ("OutDataTypeStr", "VariableName")):
                    nb[k] = p[k]
            # recordings (silent wrong): a design range (ParamMax ...) and the lookup search / out-of-range settings
            # were listed as ignored and never reached the engine's checks (recordings closed the JSON entry only);
            # carried so the .slx path goes through the same checks
            for k in sorted(own):
                v_ = str(own[k]).strip()
                if (G.DESIGN_RANGE_RE.match(k) and v_ not in ("[]", "")) or \
                        (k in ("DiagnosticForOutOfRangeInput", "IndexSearchMethod") and
                         v_ not in ("None", "Binary search", "")):
                    nb[k] = own[k]
                    if k in ignored.get(name0, []):
                        ignored[name0] = [x for x in ignored[name0] if x != k]
                        if not ignored[name0]:
                            ignored.pop(name0)
            if et == "Sum" and "Inputs" in nb:
                nb["Inputs"] = nb["Inputs"].replace("|", "")
            if et == "DiscreteFir" and "Coefficients" not in p and p.get("CoefSource", "Dialog parameters") in (
                    "Dialog parameters", "Dialog"):
                # recordings (b04 libcoef_*, b01 recordings, 8 models): a saved file leaves the coefficients out when
                # they are the block's default [0.5 0.5]; the run then failed MISSING_FIELD, labelled a request error
                nb["Coefficients"] = "[0.5 0.5]"
                assumed.setdefault(name0, []).append("Coefficients")
                assumed_v.setdefault(name0, {})["Coefficients"] = "[0.5 0.5]"
            pk_ = {"Demux": "Outputs", "Mux": "Inputs"}.get(et)
            if pk_ and pk_ not in p:
                # recordings (6 directed + 20 models) and recordings: a saved file
                # leaves Outputs / Inputs out when it is the block type's built-in default, 4 -- not the library's 2,
                # which the engine took (a 4-output Demux read as 2 outputs of width 2: silently wrong wiring)
                nb[pk_] = "4"
                assumed.setdefault(name0, []).append(pk_)
                assumed_v.setdefault(name0, {})[pk_] = "4"
            # every parameter whose allowed values the engine restricts travels with the block, so the
            # engine's own validator (fxgraph_lib META_VALUES) refuses the same configurations as on the JSON path
            # (the .slx path had dropped ApplyFullPrecisionForLinearInterpolation / ApplyAccurateEfficientRounding)
            for k in own:
                if k in G.META_VALUES and k not in nb:
                    nb[k] = p[k]
            # a saved file leaves InputSameDT / AllPortsSameDT out only when it equals the block type's
            # built-in default, which for these types is 'on' (R2026b writes 'off' explicitly: 49 / 49 Relational
            # Operator, 4 / 4 Switch, 2 / 2 Multiport Switch, 8 / 8 Logical Operator saved blocks; recorded: recordings
            # slx_inputsamedt_default, get_param('built-in/<type>') gives 'on' and the stripped Switch model fails to compile) -- carried as 'on' so differing inputs are rejected as Simulink would
            sd = {"RelationalOperator": "InputSameDT", "Switch": "InputSameDT", "MultiportSwitch": "InputSameDT",
                  "LogicalOperator": "AllPortsSameDT"}.get(et)
            if sd and sd not in nb:
                nb[sd] = p.get(sd, "on")
                if sd not in p:
                    assumed.setdefault(name0, []).append(sd)
                    assumed_v.setdefault(name0, {})[sd] = "on"
            # every parameter whose library default differs (JSON_LIBRARY_DEFAULTS) travels with the
            # block -- the file's own value where it has one (even if the reader does not otherwise keep it, e.g. a
            # FIR's FilterSource), else the saved-file default -- so the engine never applies the JSON library default
            # to a saved model (a MinMax that leaves Inputs out is the one-input collapse form: left out, as before)
            for k in G.JSON_LIBRARY_DEFAULTS.get(et, {}):
                if k not in nb and (et, k) != ("MinMax", "Inputs"):
                    if k not in p and (et, k) in _ABSENT_UNKNOWN:
                        raise SlxError("UNSUPPORTED", sys.where(sid) + "." + k, _ABSENT_MSG % (k, et))
                    if k not in p and (et, k) == ("MultiportSwitch", "DiagnosticForDefault"):
                        # recordings (argued): a saved file leaves it out when it is the block type's default,
                        # 'Error' -- the safe reading (the run stops on an out-of-range control instead of
                        # taking the default port silently)
                        nb[k] = "Error"
                        continue
                    nb[k] = p[k] if k in p else G.DEFAULTS[et][k]
            # the same for the library values the engine refuses when a JSON design leaves them out
            for k in G.JSON_LIBRARY_UNSUPPORTED.get(et, {}):
                if k not in nb:
                    if k not in p:
                        # a saved file leaves a parameter out only when it is the built-in default
                        #; the built-in default of these is not recorded -- refused, not assumed sample based
                        raise SlxError("UNSUPPORTED", sys.where(sid) + "." + k, _ABSENT_MSG % (k, et))
                    nb[k] = p[k]
            # carried so the engine checks them (a root Outport's SignalType against the signal's
            # complexity; conj with OutputSignalType real on a complex signal)
            if et in ("Outport", "Inport") and str(p.get("SignalType", "auto")).strip() in ("real", "complex"):
                nb["SignalType"] = str(p["SignalType"]).strip()
            if et == "Math" and "OutputSignalType" in p and "OutputSignalType" not in nb:
                nb["OutputSignalType"] = str(p["OutputSignalType"]).strip()
            nb = _adapt_block(bt, et, nb, p, sys.where(sid))
            name = name_of(sys, sid)
            if name in blocks:
                raise SlxError("BAD_SLX", sys.where(sid), "duplicate block name after flattening: %s" % name)
            blocks[name] = nb
            origin[name] = {"part": sys.part, "sid": sid, "path": "/".join(sys.path + [b.get("Name")])}
            if getattr(sys, "chain", None):
                origin[name]["models"] = list(sys.chain)   # the referenced models it sits in (Goto tags stay inside)
            elements[name] = (sys.part, b)
    for sys in systems:
        for (dsid, dport), (ssid, sport) in sorted(sys.src.items(), key=_src_key):
            bt = sys.btype(dsid)
            if bt == "SubSystem" and dport in ("enable", "trigger", "Reset"):
                # the enable line of an enabled subsystem feeds its EnablePort block (trigger / reset: its
                # TriggerPort / ResetPort)
                inner = children[(id(sys), dsid)]
                want_ = {"enable": ("EnablePort", "ActionPort"), "trigger": ("TriggerPort",),
                         "Reset": ("ResetPort",)}[dport]
                eps = [x for x in inner.blocks if inner.btype(x) in want_]
                if len(eps) != 1:
                    raise SlxError("BAD_SLX", sys.where(dsid), "enable port line without one EnablePort inside")
                r = resolve(sys, ssid, sport)
                ln = {"src": name_of(r[0], r[1]), "dst": name_of(inner, eps[0]), "dport": 1}
                if len(r) == 3:
                    ln["sport"] = r[2]
                lines.append(ln)
                continue
            if dport == "enable":
                raise SlxError("UNSUPPORTED", sys.where(dsid), "an enable port on a %s block" % bt)
            if bt == "Terminator":
                continue
            if (bt in ("SubSystem", "ModelReference") and not ML.is_ml(sys, dsid)) or (
                    bt == "Outport" and sys.parent is not None and not sys.enabled):
                continue  # consumed through resolve() from the inside / outside
            r = resolve(sys, ssid, sport)
            ln = {"src": name_of(r[0], r[1]), "dst": name_of(sys, dsid), "dport": dport}
            if len(r) == 3:
                ln["sport"] = r[2]   # Demux outputs (vector signals, engine/fxvector.py)
            lines.append(ln)
    for nm, r in mref_checks:
        ln = {"src": name_of(r[0], r[1]), "dst": nm + " (type check)", "dport": 2}
        if len(r) == 3:
            ln["sport"] = r[2]
        lines += [{"src": nm, "dst": nm + " (type check)", "dport": 1}, ln]
        # a referenced model is compiled on its own -- its root Inport left at -1 is scalar there,
        # and a vector driving it is refused by Simulink (PortDimsMismatch); a Signal Specification of one element
        # over the driving signal makes the engine check it
        blocks[nm + " (dims check)"] = {"type": "SignalSpecification", "Dimensions": "1"}
        lines.append(dict(ln, dst=nm + " (dims check)", dport=1))
        origin[nm + " (dims check)"] = dict(origin.get(nm, {}), role="Inport dimension check")
    port_types = []
    for sys_, sid_, tv in typed_outports:
        if sys_.parent is None:
            nm = name_of(sys_, sid_)
            if nm in blocks:
                blocks[nm]["OutDataTypeStr"] = tv
            continue
        src_ = sys_.src.get((sid_, 1))   # an Outport inside a flattened subsystem: its driver carries the constraint
        if src_ is None:
            continue
        r_ = resolve(sys_, src_[0], src_[1])
        if len(r_) == 3:
            raise SlxError("UNSUPPORTED", sys_.where(sid_) + ".OutDataTypeStr", "a typed subsystem Outport driven by "
                           "one output of a Demux")
        port_types.append({"block": name_of(r_[0], r_[1]), "type": tv, "where": sys_.where(sid_) + ".OutDataTypeStr"})
    for sys_, sid_, tv in typed_inports:
        src_ = sys_.parent.src.get((sys_.owner, sys_.port_no(sid_)))
        if src_ is None:
            continue
        r_ = resolve(sys_.parent, src_[0], src_[1])
        if len(r_) == 3:
            raise SlxError("UNSUPPORTED", sys_.where(sid_) + ".OutDataTypeStr", "a typed subsystem Inport driven by "
                           "one output of a Demux")
        drv = name_of(r_[0], r_[1])
        if blocks.get(drv, {}).get("type") == "Inport" and drv not in inport_types and G._parse_fixdt(tv):
            # the design input feeding it can only have this type: offered to the caller like a typed root Inport
            t_ = G._parse_fixdt(tv)
            inport_types[drv] = {"s": t_[0], "wl": t_[1], "fl": t_[2]}
        port_types.append({"block": name_of(r_[0], r_[1]), "type": tv, "where": sys_.where(sid_) + ".OutDataTypeStr"})
    blocks, lines = _resolve_goto(blocks, lines, origin)
    lines.sort(key=lambda l: (l["dst"], l["dport"]))
    if counter_own and (base is not None or not any(isinstance(b_, dict) and b_.get("type") in G.MR_BLOCKS
                                                     for b_ in blocks.values())) and \
            len(explicit | around | counter_own) > 1:
        raise SlxError("UNSUPPORTED", "model", "several sample times %s (a counter's tsamp among them) without a "
                       "rate-changing block: multi-rate models are not supported"
                       % sorted(explicit | around | counter_own))
    explicit |= counter_own
    if referenced:
        try:
            base_ = float(step) if step not in (None, "auto", "") else 1.0
        except ValueError:
            base_ = 1.0
        mr_ = sorted(n_ for n_, b_ in blocks.items() if isinstance(b_, dict) and b_.get("type") in G.MR_BLOCKS)

        def _other_rate(st):
            # the period carried onto an Inport of the scope (a source outside it at rate 2) counts
            # too -- the recordings check looked only inside the scope
            m_ = _ST.match(str(st).strip()) if st is not None else None
            try:
                return m_ is not None and float(m_.group(1)) not in (-1.0, base_)
            except (ValueError, TypeError):
                return False
        mr_ += sorted(n_ for n_, b_ in blocks.items() if isinstance(b_, dict) and _other_rate(b_.get("SampleTime")))
        if mr_ or any(x != base_ for x in explicit):
            # recordings (silent wrong): Simulink runs a referenced model with inherited sample times at its
            # own fixed step (every base step, the input held); the reader inlines it, so it inherited a slower
            # source's rate -- a Model block in a multirate model is not supported
            raise SlxError("UNSUPPORTED", "slx", "a Model block (%s) in a model with more than one rate (%s): Simulink "
                           "runs the referenced model at its own fixed step, which the inlined read does not model"
                           % (", ".join(sorted(referenced)), ", ".join(mr_) if mr_ else "sample times %s"
                              % sorted(explicit)))
    info = {"blocks": len(blocks), "lines": len(lines), "inport_types": inport_types, "origin": origin,
            "scope": scope or "", "parts": sorted(m.members),
            # parameters absent from the model and from its BlockParameterDefaults: the engine's own default
            # was used (not verified against Simulink) -- callers should check these
            "assumed_defaults": {k: sorted(v) for k, v in sorted(assumed.items())},
            # the values used for them: a saved model omits a parameter that has its default value, so these are
            # the block's Simulink defaults (Floor rounding, wrap, zero initial condition, inherited sample time);
            # nothing needs writing back
            "assumed_values": {k: dict(sorted(v.items())) for k, v in sorted(assumed_v.items())},
            # parameters the file omits that were filled with Simulink's default (SLX_DEFAULTS)
            "simulink_defaults_filled": {k: sorted(v) for k, v in sorted(filled.items())},
            "has_block_defaults": bool(m.defaults),
            # execution context that was checked, and model parameters the engine does not use
            "solver": {"name": solver, "fixed_step": step, "explicit_sample_times": sorted(explicit),
                       # the configuration's StopTime as written (None: not written in the file -- Simulink's own
                       # default then applies; a simulation request still gives nsteps / stop_time itself)
                       "stop_time": _stop_time(m)},
            "hw": _hardware(m),
            "ignored_parameters": ignored,
            "workspace_sources": info_sources,  # From Workspace blocks read as Inports: block -> variable
            "workspace_source_settings": info_sws,   # Signal From Workspace: sample time, after the final value
            # parameters written as names of enclosing mask parameters / model
            # workspace variables, read as values: block -> {parameter: [as written, value used]}
            "named_parameters": {k: dict(sorted(v.items())) for k, v in sorted(named.items())},
            "unresolved_names": {k: dict(sorted(v.items())) for k, v in sorted(unresolved.items())},
            "referenced_models": dict(sorted(referenced.items()))}   # referenced: name -> sha256 of the file read

    design_ = {"blocks": blocks, "lines": lines}
    if port_types:
        for e_ in port_types:
            if e_["block"] not in blocks:   # e.g. driven through a From: the constraint has no block to sit on
                raise SlxError("UNSUPPORTED", e_["where"], "a typed subsystem port driven through a Goto / From")
        design_["port_types"] = port_types
    return design_, info, elements


_RANGE = re.compile(r"^\s*\[?\s*(%s)\s*:\s*(%s)\s*(?::\s*(%s))?\s*\]?\s*$" % (_NUM1, _NUM1, _NUM1))


def _literal_vector(v, where):
    """A numeric vector literal: "[a b c]", or the colon form "[a:b]" / "[a:s:b]" expanded; else refused."""
    m = _RANGE.match(v)
    if m:
        # the end used a 1e-9 tolerance ('[0:1:0.9999999995]' gave [0 1]); one range reader for every
        # path now (engine/fxnum.py _range_values: exact for steps and ends exact in binary, else refused)
        import fxnum
        groups = [g for g in (m.group(1), m.group(5), m.group(9)) if g is not None]
        vals = fxnum._range_values(":".join(groups))
        if vals is None:
            raise SlxError("UNSUPPORTED_PARAM", where, "colon range %r with a step or end not exact in binary is "
                           "not evaluated; write the values out" % v)
        return "[%s]" % " ".join(vals)
    if _VECNUM.match(v) or _NUM.match(v):
        return v
    import fxnum
    lit = fxnum._literal_arith(v)   # '[1 2 3]*2+1' (literal arithmetic, MATLAB precedence)
    if lit is not None and (_VECNUM.match(lit) or _NUM.match(lit)):
        return lit
    raise SlxError("UNSUPPORTED_PARAM", where, "parameter %r is an expression; only numeric literals and literal "
                   "arithmetic are supported" % v)


def _adapt_block(bt, et, nb, p, where):
    """Block-type specific reading of the blocks added for recordings"""
    if et == "ZeroOrderHold" and "SampleTime" not in nb:
        # recordings-a Zero-Order Hold saved without SampleTime loads as 1 (get_param), not the library's -1
        nb = dict(nb, SampleTime="1")
    if et == "CompareToConstant" and p.get("SourceBlock", "").split("/")[-1].replace("\n", " ") == "Compare To Zero":
        # Compare To Constant with 0; the library defaults relop <=, boolean in the library model.
        return {"type": "CompareToConstant", "relop": str(p.get("relop", "<=")), "const": "0",
                "OutDataTypeStr": str(p.get("OutDataTypeStr", "boolean"))}
    if bt == "Selector" and "Indices" not in nb and str(nb.get("IndexOptions", "Index vector (dialog)")) == \
            "Index vector (dialog)":
        # a saved model writes only what differs from the block type's built-in values -- a Selector
        # element without Indices means the built-in 1 (element 1, PASS), not the library
        # block's [1 3]
        return dict(nb, Indices="1")
    if bt == "ManualSwitch":
        # the position: a saved model writes CurrentSetting (0: input 2) when it is not the default 1
        return {"type": "ManualSwitch", "sw": str(p.get("sw", p.get("CurrentSetting", "1"))).strip()}
    if bt == "Lookup_n-D":
        for k in SLX_UNKNOWN_DEFAULT["Lookup_n-D"]:
            if k not in p:
                raise SlxError("UNSUPPORTED_PARAM", where + "." + k, "Set %s explicitly. Its library default is unsupported." % k)
        dims = str(p.get("NumberOfTableDimensions", "2")).strip()
        if dims not in ("1", "2", "3", "4"):
            raise SlxError("UNSUPPORTED_BLOCK", where, "n-D Lookup Table with %s dimensions (1 to 4 only)" % dims)
        if dims in ("3", "4"):
            # 3 / 4 dimensions are the engine's LookupND, the table written
            # reshape([values], [n1 n2 ...]) as the engine reads it
            nb = dict(nb, type="LookupND", NumberOfTableDimensions=dims)
            for k in list(nb):
                m = re.match(r"BreakpointsForDimension(\d+)", k)
                if m and int(m.group(1)) > int(dims):
                    del nb[k]
            for d_ in range(1, int(dims) + 1):
                k = "BreakpointsForDimension%d" % d_
                if k not in nb:
                    raise SlxError("UNSUPPORTED_PARAM", where + "." + k, "Specify the breakpoints explicitly. Their library default is unsupported.")
                nb[k] = _literal_vector(nb[k], where + "." + k)
            try:
                G.fc.lutnd_table(nb["Table"])
            except ValueError as e:
                raise SlxError("UNSUPPORTED_PARAM", where + ".Table", "table %r: %s" % (nb["Table"], e))
            return nb
        nb = {k: v for k, v in nb.items() if k != "NumberOfTableDimensions"}
        nb["type"] = "Lookup1D" if dims == "1" else "Lookup2D"
        keep_dims = (1,) if dims == "1" else (1, 2)
        for k in list(nb):
            m = re.match(r"BreakpointsForDimension(\d+)", k)
            if m and int(m.group(1)) not in keep_dims:
                del nb[k]
        spacing = nb.get("BreakpointsSpecification", "Explicit values") == "Even spacing"
        for d_ in keep_dims:
            k = "BreakpointsForDimension%d" % d_
            if spacing:
                nb.pop(k, None)
            elif k not in nb:
                raise SlxError("UNSUPPORTED_PARAM", where + "." + k, "Specify the breakpoints explicitly. Their library default is unsupported.")
            else:
                nb[k] = _literal_vector(nb[k], where + "." + k)
        if dims == "1":
            nb["Table"] = _literal_vector(nb["Table"], where + ".Table")
        elif not re.match(r"^\s*\[[-+0-9.eE ,;\s]*\]\s*$", nb["Table"]):
            raise SlxError("UNSUPPORTED_PARAM", where + ".Table", "table %r is an expression" % nb["Table"])
        return nb
    if et in ("CounterLimited", "CounterFreeRunning"):
        ts_ = str(nb.pop("tsamp", "1")).strip()
        # the library default -1 (inherited) runs at the model step in a single-rate model
        # a whole period of its own (tsamp 4 runs
        # at period 4 in a multirate model) is kept for the engine, which runs the counter at it; anything else (an
        # offset, a fraction, an expression) is refused
        if ts_ not in ("1", "-1") and not re.match(r"^[1-9][0-9]*$", ts_):
            raise SlxError("UNSUPPORTED_PARAM", where + ".tsamp", "counter sample time %r (the model step or a whole "
                           "period only)" % ts_)
        if ts_ != "-1":
            # an explicit period is the design's (a multirate design needs it: the engine refuses a
            # counter left inherited there); it was dropped, so an exported Counter -> Downsample failed its read-back
            nb["tsamp"] = ts_
    if et in ("DetectChange", "DetectIncrease", "DetectDecrease"):
        ip = nb.pop("InputProcessing", "Elements as channels (sample based)")
        if ip != "Elements as channels (sample based)":
            raise SlxError("UNSUPPORTED_PARAM", where + ".InputProcessing", "%r (sample based only)" % ip)
    # multirate DSP blocks: the engine's rules are the sample-based multirate ones; the library defaults
    # are frame based / single rate (a CIC Interpolation left at them outputs frames), so the
    # settings must be in the file
    # FilterSource: its default 'Auto' does not use the h parameter at all (the
    # output differs completely); only 'Dialog parameters' is the engine's rule
    need = {"FIRDecimation": (("InputProcessing", "Elements as channels (sample based)"),
                              ("framing", "Allow multirate processing"), ("FilterSource", "Dialog parameters")),
            "FIRInterpolation": (("InputProcessing", "Elements as channels (sample based)"),
                                 ("framing", "Allow multirate processing"), ("FilterSource", "Dialog parameters")),
            "CICInterpolation": (("InputProcessing", "Elements as channels (sample based)"),
                                 ("framing", "Allow multirate processing")),
            "CICDecimation": (("RateOptions", "Allow multirate processing"),),
            # Upsample: sample based, both rate options give the multirate result (
            # recordings); the value only has to be in the file
            "Upsample": (("InputProcessing", "Elements as channels (sample based)"), ("RateOptions", None)),
            # Downsample: both rate options are engine rules (7 single-rate ones);
            # the value only has to be in the file
            "Downsample": (("RateOptions", None), ("InputProcessing", "Elements as channels (sample based)"))}
    for k, v in need.get(et, ()):
        if v is None:
            if k not in p:
                raise SlxError("UNSUPPORTED_PARAM", where + "." + k, "Set %s explicitly. Its library default is unsupported." % k)
            continue
        if p.get(k) != v:
            raise SlxError("UNSUPPORTED_PARAM", where + "." + k, "%s=%r (only %r: the library default is frame based / "
                           "single rate, which the engine does not model)" % (k, p.get(k, "(library default)"), v))
    if et == "CICInterpolation" and p.get("OutputFrames", "off") != "off":
        raise SlxError("UNSUPPORTED_PARAM", where + ".OutputFrames", "OutputFrames on is not supported")
    if et == "HDLRAM" and p.get("SourceBlock", "").split("/")[-1].replace("\n", " ").endswith("RAM System"):
        # these settings were recorded; the address width comes from the address type
        # at compile time (engine/fxhdl.py ram_auto_resolve)
        lib = p.get("SourceBlock", "").split("/")[-1].replace("\n", " ")
        want_rt = {"Simple Dual Port RAM System": "Simple dual port", "Dual Port RAM System": "Dual port",
                   "Single Port RAM System": "Single port"}[lib]
        # recordings (dprs_*): the Dual Port RAM System set to Single port / Simple dual port runs as the RAM of
        # that type (the hdl.RAM behind the three blocks is one object); True dual port and Simple tri port were
        # recorded once each, with ports Simulink pairs asymmetrically -- refused
        if lib == "Dual Port RAM System" and str(p.get("RAMType", want_rt)).strip() in ("Single port",
                                                                                        "Simple dual port"):
            want_rt = str(p["RAMType"]).strip()
        fixed = {"RAMType": want_rt, "AsyncRead": "off", "allowBusInputs": "off", "WritesHaveOutput": "on",
                 "NumReads": "0", "NumWrites": "1", "ModelRAMDelay": "on", "VectorAccess": "Parallel"}
        for k, v in fixed.items():
            if str(p.get(k, v)).strip() != v:
                raise SlxError("UNSUPPORTED_PARAM", where + "." + k, "%s=%r (only %r was recorded)" % (k, p[k], v))
        if str(p.get("SimulateUsing", "Code generation")) not in ("Code generation", "Interpreted execution"):
            raise SlxError("UNSUPPORTED_PARAM", where + ".SimulateUsing", "SimulateUsing=%r" % p["SimulateUsing"])
        nb = {"type": "HDLRAM", "RAMType": want_rt, "AddressBits": "Inherit: from address",
              "RAMInitialValue": str(p.get("RAMInitialValue", "0")).strip(),
              "WriteOutputValue": str(p.get("WriteOutputValue", "New data")).strip()}
        return nb
    if et == "HDLRAM":
        lib = p.get("SourceBlock", "").split("/")[-1].replace("\n", " ")
        nb = {"type": "HDLRAM", "RAMType": {"Simple Dual Port RAM": "Simple dual port", "Dual Port RAM": "Dual port",
                                            "Single Port RAM": "Single port"}[lib],
              "AddressBits": str(p.get("ram_size", "8")).strip(),   # the mask default
              "RAMInitialValue": "0"}   # the masked RAMs start at 0
        if lib == "Single Port RAM":
            nb["WriteOutputValue"] = p.get("dout_type", "New data")
        return nb
    # DiscreteIntegrator: InitialConditionSource / ExternalReset travel with the block since recordings
    # (they were dropped when MUST pinned them to internal / none -- with MUST lifted, dropping a reset would be wrong)
    return nb


def _resolve_goto(blocks, lines, origin):
    """From / Goto are wires: every From is replaced by the source of the Goto with the same tag. 'local'
    visibility pairs a From with the Goto of the same (sub)system; 'scoped' pairs it with the Goto whose tag is made
    visible by a Goto Tag Visibility block in the From's system or one of its ancestors (the nearest one wins);
    'global' anywhere in the model."""
    gotos, froms, scopes = {}, [], {}
    # the driver's output port travels with it (a Goto fed by a Demux output 2 is that signal)
    src_of = {(l["dst"], l["dport"]): (l["src"], l.get("sport", 1)) for l in lines}

    def parent_of(n):
        # coverage per block: a block at the model's top level has the path of its own name;
        # its system is the root (""), not the name -- a local Goto and a From both at the top level did not pair
        path = origin[n]["path"]
        return path.rsplit("/", 1)[0] if "/" in path else ""
    for n, b in blocks.items():
        if b["type"] == "GotoTagVisibility":
            key = (b.get("GotoTag", ""), parent_of(n))
            if key in scopes:
                raise SlxError("BAD_SLX", "blocks." + n, "two Goto Tag Visibility blocks for tag %s in one system" % key[0])
            scopes[key] = n
    for n, b in blocks.items():
        if b["type"] == "Goto":
            vis = b.get("TagVisibility", "local")
            if vis not in ("local", "global", "scoped"):
                raise SlxError("UNSUPPORTED_BLOCK", "blocks." + n, "Goto with %s tag visibility" % vis)
            tag = b.get("GotoTag", "")
            if vis == "scoped":
                root = _scope_root(tag, parent_of(n), scopes)
                if root is None:
                    raise SlxError("BAD_SLX", "blocks." + n, "scoped Goto %s without a Goto Tag Visibility block above it" % tag)
                key = (tag, "scoped", root)
            else:
                key = (tag, "local", parent_of(n)) if vis == "local" else (tag, "global", "")
            if key in gotos:
                raise SlxError("BAD_SLX", "blocks." + n, "two Goto blocks with tag %s in one scope" % tag)
            gotos[key] = n
        elif b["type"] == "From":
            froms.append(n)
    if not gotos and not froms and not scopes:
        return blocks, lines
    feed = {}
    for f in froms:
        parent = parent_of(f)
        tag = blocks[f].get("GotoTag", "")
        root = _scope_root(tag, parent, scopes)
        g = gotos.get((tag, "local", parent)) or (gotos.get((tag, "scoped", root)) if root is not None else None) \
            or gotos.get((tag, "global", ""))
        if g is not None and origin[g].get("models", []) != origin[f].get("models", []):
            # recordings (silent wrong): a Goto tag does not cross a model boundary -- the From inside a
            # referenced model was wired to the parent's Goto (Simulink runs it with the From unmatched, giving 0)
            raise SlxError("UNSUPPORTED", "blocks." + f, "From %s (tag %s) has no Goto in its own model (the Goto %s "
                           "is in another model; a tag does not cross a Model block); Simulink runs it with the From "
                           "giving 0, which is not supported here" % (f, tag, g))
        if g is None:
            # a valid file -- Simulink runs a From without a Goto (it gives 0); not supported here
            raise SlxError("UNSUPPORTED", "blocks." + f, "From %s (tag %s) has no matching Goto; Simulink runs it with "
                           "the From giving 0, which is not supported here" % (f, tag))
        s = src_of.get((g, 1))
        if s is None:
            raise SlxError("BAD_SLX", "blocks." + g, "Goto input is not connected")
        feed[f] = s
    def res(x, port):
        seen = set()
        while x in feed:   # a From has one output: its line's port is 1, the Goto's driver gives the real one
            if x in seen:
                raise SlxError("BAD_SLX", "blocks." + x, "From/Goto cycle")
            seen.add(x)
            x, port = feed[x]
        return x, port

    def rewire(l):
        src, port = res(l["src"], l.get("sport", 1))
        nl = {k: v for k, v in l.items() if k != "sport"}
        nl["src"] = src
        if port != 1:
            nl["sport"] = port
        return nl
    new_lines = [rewire(l) for l in lines if blocks[l["dst"]]["type"] not in ("Goto",)]
    drop = set(froms) | set(gotos.values()) | set(scopes.values())
    return {n: b for n, b in blocks.items() if n not in drop}, new_lines


def _scope_root(tag, system, scopes):
    """Nearest system at or above `system` holding a Goto Tag Visibility block for `tag` (None if none)."""
    path = system
    while True:
        if (tag, path) in scopes:
            return path
        if path == "":
            return None
        path = path.rsplit("/", 1)[0] if "/" in path else ""   # up to the model's root system ("")


# data-type parameters write_types may set (the fxslx CLI uses the same list)
WRITABLE_TYPES = ("OutDataTypeStr", "ParamDataTypeStr", "AccumDataTypeStr", "CoefDataTypeStr", "ProductDataTypeStr",
                  # 1-D / 2-D Lookup Table types
                  "BreakpointsForDimension1DataTypeStr", "BreakpointsForDimension2DataTypeStr", "TableDataTypeStr",
                  "IntermediateResultsDataTypeStr", "FractionDataTypeStr")


_INHERIT = re.compile(r"^Inherit: [A-Za-z][A-Za-z ,()\-]*$")


def write_types(data, assignments, scope=None, report=None):
    """Return new .slx bytes with data-type parameters set. assignments: {design block name: {param: value}}
    using the names produced by read(); only data-type parameters (output, parameter, accumulator, and the
    Discrete FIR coefficient / product types) may be set, and only on a block whose type has that parameter.
    The Block elements edited are exactly the ones read() resolved (no lookup by name), every other archive
    member is copied byte for byte; edited XML parts are re-serialised. report (a dict), if given, receives
    {block: {param: {"old": text or None, "new": text}}} for every parameter written."""
    allowed = WRITABLE_TYPES
    m = Model(data)
    design, info, elements = _read_model(m, scope)
    touched = set()
    for name, params in assignments.items():
        if name not in elements:
            raise SlxError("BAD_PARAM", "blocks." + name, "no such block in the model")
        part, b = elements[name]
        bt = b.get("BlockType")
        if bt == "Reference":
            lib = LIBRARY.get(_params(b).get("SourceBlock", "").split("/")[-1].replace("\n", " "))
            keys = lib[1] if lib else ()
        else:
            keys = TYPEMAP.get(bt, (None, ()))[1]
        for k, v in params.items():
            if k not in allowed:
                raise SlxError("BAD_PARAM", "blocks.%s.%s" % (name, k), "only data-type parameters can be written")
            if k not in keys:
                # e.g. AccumDataTypeStr was added to a Gain, which has no such parameter
                raise SlxError("BAD_PARAM", "blocks.%s.%s" % (name, k), "a %s block has no parameter %s"
                               % (design["blocks"][name]["type"], k))
            if not isinstance(v, str) or (G._parse_fixdt(v) is None and not _INHERIT.match(v)
                                          and v not in G.fc._NAMED_INT and v != "boolean"):
                raise SlxError("BAD_TYPE", "blocks.%s.%s" % (name, k), "fixdt(s,w,f), an integer type or an Inherit: string required")
        inst = b.find("InstanceData")   # library (Reference) blocks keep their mask parameters there
        for k, v in params.items():
            # F9: a Reference block may carry the parameter directly, under InstanceData, or both (the reader
            # prefers the direct one) -- every copy is set; a new one goes where the block keeps its parameters
            ps = [p for t_ in ((b, inst) if inst is not None else (b,)) for p in t_.findall("P")
                  if p.get("Name") == k]
            old = _params(b).get(k)
            if ps:
                for p in ps:
                    p.text = v
            else:
                e = ET.SubElement(inst if (bt == "Reference" and inst is not None) else b, "P", {"Name": k})
                e.text = v
            if report is not None:
                report.setdefault(name, {})[k] = {"old": old, "new": v}
        touched.add(part)
    out = io.BytesIO()
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
        for info_z in m.zf.infolist():
            raw = m.members[info_z.filename]
            if info_z.filename in touched:
                raw = ET.tostring(m.trees[info_z.filename].getroot(), encoding="utf-8", xml_declaration=True)
            zi = zipfile.ZipInfo(info_z.filename, date_time=info_z.date_time)
            zi.compress_type = zipfile.ZIP_DEFLATED
            z.writestr(zi, raw)
    return out.getvalue()

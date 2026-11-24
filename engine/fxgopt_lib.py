"""fxopt for general block diagrams : word-length search on the fxgraph engine.

Decision variables: OutDataTypeStr of every non-fixed Gain / Sum / Product / DataTypeConversion / Saturation / Abs /
MinMax block, ParamDataTypeStr of every Gain, AccumDataTypeStr of every Sum. Signedness and fraction
lengths follow from word lengths and the ranges of a double-precision simulation of the same graph.
Search: uniform-precision start (all signals share one LSB, all parameters one word length; both
chosen by parallel bit-exact sweeps) -> problem-independent bit-exact refinement shared with the
chain optimiser (fxopt_lib.refine). Cost (hwbits): compiled output word length of every block except
Outports (booleans count 1) + Gain parameter word lengths + Sum accumulator word lengths.
"""
import json
import math
import os
import subprocess
import threading
import time

import fxgraph_lib as G
import fxopt_lib as L
from fxopt_lib import OptError
from blockspec import VARBLOCKS, INNER, LUTS, LUT1D_BP, LUT1D_TAB, LUT1D_IT, LUT1D_FR, LUT2D_BP  # noqa: E402,F401 -- the block-support registry

PASS = ("UnitDelay", "Delay", "UnaryMinus", "ShiftArithmetic", "Signum", "Rounding",
        "DiscreteIntegrator", "Hold", "Gate")  # output word = input word


def _passes(d, b):
    """Return whether a block shares its input word for cost accounting."""
    bt = d.btype[b]
    if bt not in PASS:
        return False
    if bt == "DiscreteIntegrator":
        return str(d.params[b].get("OutDataTypeStr", "Inherit: Inherit via internal rule")).strip().startswith("Inherit")
    return True


def _lut_table_follows_output(p):
    return str(p.get("TableDataTypeStr", "Inherit: Same as output")).startswith("Inherit")


def lut_bits_from_params(d, b, ts):
    """Compute lookup storage from parameters when there is no compiled record."""
    bt = d.btype[b]
    p = d.params[b]
    tt = d.otype[b] if _lut_table_follows_output(p) else G._fixdt(p["TableDataTypeStr"], "")
    c = _lut_entries(p, bt) * tt[1]
    if bt == "Lookup2D" and ts and G.BOOL not in ts[:2]:
        for dim in (1, 2):
            bt_ = G.fc._lut_type(p["BreakpointsForDimension%dDataTypeStr" % dim], tuple(ts[dim - 1]))
            c += len(_lut2d_bps(p, dim)) * bt_[1]
        if p.get("InterpMethod") == "Linear point-slope":
            for k in (LUT1D_IT, LUT1D_FR):
                t = G._fixdt(str(p.get(k, "")), "")
                c += t[1] if t else 0
    if bt == "Lookup1D" and ts and ts[0] != G.BOOL:
        # stored breakpoints (compiled width of their type) and, for linear interpolation, the explicit
        # intermediate and fraction types (as written; inherited ones are not counted here)
        bt_ = G.fc._lut_type(p["BreakpointsForDimension1DataTypeStr"], tuple(ts[0]))
        c += _lut_entries(p, bt) * bt_[1]
        if p.get("InterpMethod") == "Linear point-slope":
            for k in (LUT1D_IT, LUT1D_FR):
                t = G._fixdt(str(p.get(k, "")), "")
                c += t[1] if t else 0
    return c


def block_keys(bt, p):
    """Decision-variable type parameters of a block."""
    if bt == "Lookup2D":
        keys = [] if p.get("InterpMethod") == "Nearest" else list(LUT2D_BP)
        keys.append(LUT1D_TAB)
        if p.get("InterpMethod") == "Linear point-slope":
            keys += [LUT1D_IT, LUT1D_FR]
        return tuple(keys) + ("OutDataTypeStr",)
    if bt == "Lookup1D":
        keys = [] if p.get("InterpMethod") == "Nearest" else [LUT1D_BP]
        keys.append(LUT1D_TAB)
        if p.get("InterpMethod") == "Linear point-slope":
            keys += [LUT1D_IT, LUT1D_FR]
        return tuple(keys) + ("OutDataTypeStr",)
    import fxrecords as R
    if bt in R.RECORD_OF:   # from the compiled record's type roles, not a hand table
        return tuple(k for k, _ in R.RECORD_OF[bt].TYPE_ROLES)
    return VARBLOCKS.get(bt, ())


def _lut2d_bps(p, dim):
    if p.get("BreakpointsSpecification") == "Even spacing":
        t = G.fc._mat(p["Table"])
        n = len(t) if dim == 1 else len(t[0])
        x0 = float(p["BreakpointsForDimension%dFirstPoint" % dim])
        dx = float(p["BreakpointsForDimension%dSpacing" % dim])
        return tuple(x0 + k * dx for k in range(n))
    return tuple(G.fc._vec(p["BreakpointsForDimension%d" % dim]))


def _lut_table_values(p, bt):
    return tuple(v for r in G.fc._mat(p["Table"]) for v in r) if bt == "Lookup2D" else tuple(G.fc._vec(p["Table"]))


def _lut1d_bps(p):
    if p.get("BreakpointsSpecification") == "Even spacing":
        n = len(G.fc._vec(p["Table"]))
        x0, dx = float(p["BreakpointsForDimension1FirstPoint"]), float(p["BreakpointsForDimension1Spacing"])
        return tuple(x0 + k * dx for k in range(n))
    return tuple(G.fc._vec(p["BreakpointsForDimension1"]))


def _lut_entries(p, bt="Lookup1D"):
    if bt == "Lookup2D":
        return sum(len(r) for r in G.fc._mat(p["Table"]))
    return len(G.fc._vec(p["Table"]))


def _ints_from_spec(name, spec, prefix="inputs"):
    where = prefix + "." + name
    if not isinstance(spec, dict):
        raise OptError("MISSING_FIELD", where, "input definition object required")
    if spec.get("type") == "boolean":
        # a boolean Inport (a valid / enable line) was refused as BAD_PARAM "{s, wl, fl} required",
        # which reads like a malformed request; fxopt chooses word lengths of fixed-point signals only
        raise OptError("UNSUPPORTED", where + ".type", "a boolean Inport is not supported by fxopt (it optimises "
                       "fixed-point word lengths and judges fixed-point inputs); optimise the fixed-point part without "
                       "the boolean input (e.g. hold the enable on with a boolean Constant), then write the chosen types "
                       "into the full design and check it with fxgraph")
    if {"s", "wl", "fl"} & set(spec):
        # fxgraph's form {s, wl, fl, ints} is accepted too
        if "type" in spec:
            raise OptError("BAD_PARAM", where, "give the type either as \"type\": {s, wl, fl} or as s / wl / fl, "
                           "not both")
        spec = dict({k: v for k, v in spec.items() if k not in ("s", "wl", "fl")},
                    type={k: spec[k] for k in ("s", "wl", "fl") if k in spec})
    extra = sorted(set(spec) - {"kind", "type", "amplitude", "omega", "n", "phase", "offset", "ints"})
    if extra:
        raise OptError("BAD_PARAM", where + "." + extra[0], "unknown field %s; an input is {\"type\": {\"s\": 1, "
                       "\"wl\": 16, \"fl\": 14}, \"ints\": [...]} (or {\"s\", \"wl\", \"fl\", \"ints\"}, as "
                       "fxgraph takes it), or {\"kind\": \"sine\", \"type\": ..., \"amplitude\", \"omega\", "
                       "\"n\"}" % extra[0])
    s, w, f = L.strict_type(spec.get("type", {"s": 1, "wl": 16, "fl": 14}), where + ".type")
    if not (2 <= w <= 64):
        raise OptError("BAD_PARAM", where + ".type.wl", "2..64 required")
    lo, hi = (-(1 << (w - 1)), (1 << (w - 1)) - 1) if s else (0, (1 << w) - 1)
    kind = spec.get("kind", "ints")
    if kind == "sine":
        amp, om = float(L.strict_number(spec, "amplitude", where)), float(L.strict_number(spec, "omega", where))
        n = L.strict_number(spec, "n", where, integer=True)
        ph = float(L.strict_number(spec, "phase", where, default=0.0))
        off = float(L.strict_number(spec, "offset", where, default=0.0))
        if not (1 <= n <= 2_000_000):
            raise OptError("LIMIT_EXCEEDED", where + ".n", "1..2000000 samples")
        ints = [int(math.floor((off + amp * math.sin(om * i + ph)) * 2.0 ** f)) for i in range(n)]
    elif kind == "ints":
        ints = L.strict_ints(spec.get("ints"), where + ".ints")
    else:
        raise OptError("BAD_PARAM", where + ".kind", "sine or ints")
    if any(v < lo or v > hi for v in ints):
        raise OptError("BAD_INPUT", where + ".ints", "stored integer out of range for the input type")
    return (s, w, f), ints



_PDELAY = "Delay with an enable / reset port"   # recordings (engine/fxhdl.py _port_delay): not checked in fxopt
_LAB2_NOT_OPTIMISED = {"HDLRAM", "DetectRisePositive", "HDLCounter", "Repeat", _PDELAY,
                       # recordings (recorded): lowered under the user's name into a Gain / Sum, fxopt wrote that block's
                       # ParamDataTypeStr onto them (BAD_PARAM); their output type is their input's anyway
                       "Bias", "IncrementRealWorld", "DecrementRealWorld", "IncrementStoredInteger",
                       "DecrementStoredInteger",
                       # recordings (engine/fxiir.py, fxzoh.py): lowered into internal blocks under the user's
                       # name; their types stay as the design gives them (not decision variables: only two explicit
                       # type sets are recorded). Biquad Filter was missing: fxopt wrote OutDataTypeStr onto it and
                       # blamed the request (BAD_PARAM)
                       "DiscreteFilter", "DiscreteTransferFcn", "BiquadFilter", "ZeroOrderHold"}


def _lab2_types(spec):
    """the extended blocks types of a design (fxhdl / block_lowering / the vector pass's the extended blocks), 'Math' excepted."""
    import fxhdl
    import block_lowering
    kinds = set(fxhdl.TYPES + fxhdl.ES_TYPES + fxhdl.CPLX_TYPES + fxhdl.CORDIC_TYPES + ("HDLNCO",) + block_lowering.UNTYPED + block_lowering.TYPED +
                ("TappedDelay", "TappedDelayEnabledSynchronous", "Concatenate", "BusAssignment", "BusCreator",
                 "BusSelector", "ForEach", "BitConcat", "BitSlice", "BitShift", "BitRotate", "BitSet", "BitClear",
                 "DiscreteFilter", "DiscreteTransferFcn", "BiquadFilter", "ZeroOrderHold"))
    blocks = spec.get("blocks") if isinstance(spec, dict) else None
    out = {b.get("type") for b in (blocks or {}).values() if isinstance(b, dict) and b.get("type") in kinds}
    if any(fxhdl.port_delay(b) for b in (blocks or {}).values()):
        out.add(_PDELAY)
    return out


def _with_vec_key(spec, base):
    """a candidate design keeps the vector pass's record of the extended blocks it replaced
    (parameter_bounds.VEC_KEY), so their recorded word lengths are checked on every candidate as in fxgraph."""
    import parameter_bounds
    # and the rows it found fed by a vector (VEC_ROWS_KEY: the boolean-vector rule of recordings)
    for key in (parameter_bounds.VEC_KEY, parameter_bounds.VEC_ROWS_KEY):
        if isinstance(base, dict) and key in base:
            spec[key] = base[key]
    return spec

def _variant(kind, ints, types):
    """A cheap variant of one input set (G9): every Inport negated (saturating; unsigned: mirrored inside the type,
    lo + hi - x) or time-reversed."""
    out = {}
    for name, xs in ints.items():
        s, w, _ = types[name]
        lo, hi = (-(1 << (w - 1)), (1 << (w - 1)) - 1) if s else (0, (1 << w) - 1)
        if kind == "reversed":
            out[name] = list(reversed(xs))
        elif s:
            out[name] = [min(hi, -x) for x in xs]
        else:
            # mirrored inside the set's own envelope (lo + hi - x of the type widened the range
            # of a 0..100 uint16 input to the whole type, +43 % hwbits)
            mn, mx = (min(xs), max(xs)) if xs else (lo, hi)
            out[name] = [mn + mx - x for x in xs]
    return out


def multirate_filter_bits(d, b):
    """Word lengths inside a multirate filter that count in hwbits (they counted their
    output only). FIR Decimation / Interpolation, as Discrete FIR: coefficient x taps + product x taps + accumulator
    (one tap: none) + the input delay line (taps - 1 words of the input). CIC: every integrator / comb section's
    word, plus each comb's differential delay (M words of its width). From the filter's resolved types
    (engine/fxmultirate.py fir_types / cic_types) for its compiled input type; 0 while that is not known."""
    import fxmultirate as MR
    ts = d._in_types(b)
    if not ts or ts[0] == G.BOOL:
        return 0
    bt, p, where = d.btype[b], d.params[b], "design.blocks." + b
    xt = tuple(ts[0])
    if bt.startswith("FIR"):
        ct, pt, at, _ot = MR.fir_types(p, xt, where, G.fc._asic(d.hw))
        n = len(MR._coefs(p, where))
        return n * ct[1] + n * pt[1] + (at[1] if n > 1 else 0) + (n - 1) * xt[1]
    dec = bt == "CICDecimation"
    secs, _ot = MR.cic_types(p, xt, dec, where)
    N, M = int(p["N"]), int(p["M"])
    combs = secs[N:] if dec else secs[:N]
    return sum(w for w, _f in secs) + M * sum(w for w, _f in combs)


def hook_cost(d, v):
    """An owner's cost from cost_hooks: its bits + count x the compiled word length of each per_signal block."""
    return v[0] + sum(c * (1 if d.otype.get(b) == G.BOOL else d.otype[b][1]) for b, c in v[2].items() if b in d.otype)


def cost_hooks(d):
    """The block-cost hook for blocks a lowering replaces (recorded HDL blocks, including lowering): an optional
    engine/fxhdl.py may define
      cost_owner(d, name) -> the user block an internal block of the lowered design belongs to, or None;
      inner_bits(d, owner) -> (bits, explanation) for that user block's whole count (its output registers and every
                              stored word x its count, a table as entries x width), or None.
    Both must answer from the design's parameters and structure only, not from compiled types (fxopt calls them on
    the design before and after compiling). A word that follows a signal's type goes in an optional third element,
    per_signal {block of d: count}: the owner then costs bits + sum(count x that block's output word length), linear
    in fxopt's decision variables (the signal's weight grows by count). Returns ({internal block: owner} skipped by the counter,
    {owner: (bits, explanation)}); a block whose owner's inner_bits is None stays counted as before. Without fxhdl
    (main) both are empty."""
    try:
        import fxhdl
    except ImportError:
        return {}, {}
    owner_of, bits_of = getattr(fxhdl, "cost_owner", None), getattr(fxhdl, "inner_bits", None)
    if owner_of is None or bits_of is None:
        return {}, {}
    skip, owners = {}, {}
    for b in d.names:
        o = owner_of(d, b)
        if o is None:
            continue
        if o not in owners:
            r = bits_of(d, o)
            # (bits, explanation[, per_signal]): per_signal = {block of d: count} adds count x the word length of that
            # block's output signal (recorded: a RAM / HDL FIR whose words follow its input type)
            owners[o] = None if r is None else (int(r[0]), str(r[1]),
                                                {str(k): int(v) for k, v in (r[2] if len(r) > 2 and r[2] else {}).items()})
        if owners[o] is not None:
            skip[b] = o
    return skip, {o: v for o, v in owners.items() if v is not None}


class GraphProblem:
    def _pin_typed_ports(self):
        """a typed Outport (or a typed subsystem port, design.port_types) is a constraint -- the block
        driving it must output exactly that type. Its output type is pinned to the declared one and not optimised
        (fxopt's own starting plan was compared with the port and the design refused with a false TYPE_MISMATCH).
        Returns {(block, 'OutDataTypeStr')}."""
        spec = self.spec
        if not (isinstance(spec, dict) and isinstance(spec.get("blocks"), dict) and isinstance(spec.get("lines"), list)):
            return set()
        want = {}
        for n, b in spec["blocks"].items():
            if isinstance(b, dict) and b.get("type") == "Outport":
                t = str(b.get("OutDataTypeStr", "Inherit: auto")).strip()
                if not t.startswith("Inherit"):
                    src = [ln.get("src") for ln in spec["lines"] if ln.get("dst") == n]
                    if src:
                        want.setdefault(src[0], (t, "design.blocks.%s.OutDataTypeStr" % n))
        for k, e in enumerate(spec.get("port_types") or []):
            if isinstance(e, dict) and e.get("block") in spec["blocks"]:
                want.setdefault(e["block"], (str(e.get("type")), "design.port_types[%d]" % k))
        if not want:
            return set()
        blocks = dict(spec["blocks"])
        pinned = set()
        srcs = {}
        for ln in spec["lines"]:
            srcs.setdefault(ln.get("dst"), []).append(ln.get("src"))
        for b0, (t, where) in list(want.items()):
            b = b0
            seen = set()
            # a delay keeps its input's type: the constraint moves to the block that sets the type
            while isinstance(blocks.get(b), dict) and blocks[b].get("type") in ("UnitDelay", "Delay") and \
                    len(srcs.get(b, [])) >= 1 and b not in seen:
                seen.add(b)
                b = srcs[b][0]
            if b != b0:
                del want[b0]
                want.setdefault(b, (t, where))
        for b, (t, where) in want.items():
            blk = blocks.get(b)
            if not isinstance(blk, dict):
                continue
            if blk.get("type") in ("Inport",):
                continue   # an input's type is given, checked by the engine
            if "OutDataTypeStr" not in block_keys(blk.get("type"), blk):
                continue   # the driver's type follows its input: the engine checks the port on every candidate
            blocks[b] = dict(blk, OutDataTypeStr=t)
            pinned.add((b, "OutDataTypeStr"))
        self.spec = dict(spec, blocks=blocks)
        return pinned


    def __init__(self, req):
        self.req = req
        self.spec = req["design"]
        self.hw = req.get("hw")
        self.pinned = self._pin_typed_ports()
        self.vec_info = None
        import fxvector
        if isinstance(self.spec, dict) and isinstance(self.spec.get("blocks"), dict) and fxvector.has_vectors(self.spec):
            # vector signals / Mux / Demux / Selector: optimise the scalarised design, one variable per block
            try:
                low, self.vec_info = fxvector.scalarise(self.spec)
            except G.GraphError as e:
                raise OptError(e.code, e.where, e.message)
            self.spec_vec, self.spec = self.spec, low
        try:
            self.d0 = G.Design(self.spec, self.hw)
        except G.GraphError as e:
            raise OptError(e.code, e.where, e.message)
        d0 = self.d0
        self._hook_skip, self._hook_bits = cost_hooks(d0)
        self.mr_info = None
        lab2_types = _lab2_types(getattr(self, "spec_vec", None) or self.spec)
        if d0.multirate and lab2_types:
            # the multirate lowering removes the extended blocks before their recorded word lengths are
            # checked (a 24-bit Gain into a 16-bit Reshape / DTD / Manual Switch row was accepted); not supported
            raise OptError("UNSUPPORTED", "design", "the extended blocks (%s) in a design with several sample rates are "
                           "not optimised (fxgraph runs it)" % ", ".join(sorted(lab2_types)))
        if lab2_types & _LAB2_NOT_OPTIMISED:
            # recordings (asides): these ended in INTERNAL errors in fxopt
            raise OptError("UNSUPPORTED", "design", "%s is not optimised by fxopt (fxgraph runs it)"
                           % ", ".join(sorted(lab2_types & _LAB2_NOT_OPTIMISED)))
        if d0.multirate and self.vec_info is not None:
            # recordings (aside): the multirate lowering of a scalarised vector design ended in INTERNAL 'D' (a
            # Tapped Delay / Concatenate next to a Downsample); not supported here
            raise OptError("UNSUPPORTED", "design", "a design with vector signals and several sample rates is not "
                           "optimised (fxgraph runs it)")
        if d0.multirate:
            # optimise the single-rate lowering (engine/fxlower.py); the solution keeps the original blocks
            import fxlower
            first = req.get("inputs") or (req.get("input_sets") or [None])[0] or {}
            if L.is_set_wrapper(first, d0.inports):
                first = first.get("inputs") or {}   # a set with its own tolerance multiple
            peek = {}
            for name in d0.inports:   # filter lowering needs the input types (checked again below)
                t = (first.get(name) or {}).get("type", {"s": 1, "wl": 16, "fl": 14}) if isinstance(first, dict) else {}
                peek[name] = (t.get("s"), t.get("wl"), t.get("fl")) if isinstance(t, dict) else None
            try:
                low, self.mr_info = fxlower.lower(self.spec, self.hw, peek if all(
                    v and all(isinstance(x, int) for x in v) for v in peek.values()) else None)
                self.spec_orig, self.spec = self.spec, low
                self.d0 = d0 = G.Design(low, self.hw)
                # the filters' inner word lengths (multirate_filter_bits) are fixed by the lowering: a constant
                self._mr_filter_bits = 0
                if self.mr_info.get("filters") and all(v for v in peek.values()):
                    # start types 'double' downstream of the filters (a Gain after a FIR Decimation) do
                    # not change the filters' inner types -- a placeholder fixed type there for this compile; doubles
                    # upstream of a filter would, and are refused with the reason (was BAD_TYPE "unsupported data
                    # type string 'double'", a request error)
                    ups, todo = set(), list(self.mr_info["filters"])
                    while todo:
                        x = todo.pop()
                        for ln in self.spec_orig.get("lines", []):
                            if ln.get("dst") == x and ln.get("src") not in ups:
                                ups.add(ln["src"])
                                todo.append(ln["src"])
                    blks = {}
                    for b_, p_ in self.spec_orig["blocks"].items():
                        q_ = dict(p_)
                        for k_, v_ in p_.items():
                            if k_.endswith("DataTypeStr") and str(v_).strip() in ("double", "single"):
                                if b_ in ups or b_ in self.mr_info["filters"]:
                                    raise OptError("START_TYPE_NEEDED", "design.blocks.%s.%s" % (b_, k_), "start type %r "
                                                   "upstream of the multirate filter(s) %s: their inner word lengths "
                                                   "follow this type -- give it a fixed-point start type (fixdt)"
                                                   % (v_, ", ".join(sorted(self.mr_info["filters"]))))
                                q_[k_] = "fixdt(1,32,16)"
                        blks[b_] = q_
                    dorig_ = G.Design(dict(self.spec_orig, blocks=blks), self.hw)
                    dorig_.compile(peek)
                    self.mr_filter_bits = {f_: multirate_filter_bits(dorig_, f_) for f_ in self.mr_info["filters"]}
                    self._mr_filter_bits = sum(self.mr_filter_bits.values())
            except G.GraphError as e:
                raise OptError(e.code, e.where, e.message)
        # Scale the tolerance for generated stress inputs. Given sets keep their own scale.
        sk = req.get("stress_tolerance_scale", 1)
        if not isinstance(sk, (int, float)) or isinstance(sk, bool) or not math.isfinite(sk) or sk < 1:
            raise OptError("BAD_PARAM", "stress_tolerance_scale", "a JSON number >= 1 (the multiple of the tolerance "
                           "the stress inputs derived from input_ranges are judged against), got %s %r ("
                           "write 16, not \"16\")" % (type(sk).__name__, sk))
        self.stress_scale = float(sk)
        # one input set ("inputs") and/or several ("input_sets"); every set must meet the error bound
        sets = []
        if req.get("inputs") is not None:
            sets.append(("inputs", req.get("inputs")))
        extra = req.get("input_sets")
        if extra is not None:
            if not isinstance(extra, list) or not extra or len(extra) > 64:
                raise OptError("BAD_PARAM", "input_sets", "list of 1..64 input definitions {Inport: definition}")
            # the error search's counterexamples (appended after the caller's sets by
            # fxopt's error_search_loop) are labelled as such -- they were indistinguishable from the caller's
            ns = int(req.get("_n_search_sets", 0))
            own = len(extra) - ns
            for k, e in enumerate(extra):
                where = "input_sets[%d]" % k if k < own else "error_search[%d]" % (k - own)
                if L.is_set_wrapper(e, d0.inports):
                    sc_ = e.get("tolerance_scale", 1)
                    if not isinstance(sc_, (int, float)) or isinstance(sc_, bool) or not math.isfinite(sc_) or \
                            sc_ <= 0 or not isinstance(e.get("inputs"), dict) or set(e) - {"inputs", "tolerance_scale"}:
                        raise OptError("BAD_PARAM", where, "{\"inputs\": {Inport: definition}, \"tolerance_scale\": "
                                       "positive number} required")
                    sets.append((where, e["inputs"], float(sc_)))
                else:
                    sets.append((where, e) if k < own else (where, e, self.stress_scale))
        # plans fitted to one training signal broke on its negation / time reversal; every
        # given set is also judged negated (mirrored inside the type for unsigned inputs) and reversed by default
        variants = req.get("input_variants", ["negated", "reversed"])
        if not (isinstance(variants, list) and all(v in ("negated", "reversed") for v in variants)):
            raise OptError("BAD_PARAM", "input_variants", "list of 'negated' / 'reversed' ([] to judge the given "
                           "sets only)")
        self.input_variants = sorted(set(variants))
        self.range_spec = req.get("input_ranges")
        if not sets and self.range_spec is None:
            raise OptError("MISSING_FIELD", "inputs", "inputs, input_sets or input_ranges required")
        if not d0.inports:
            raise OptError("UNSUPPORTED", "design", "design needs at least one Inport")
        self.in_types, self.scenarios = None, []
        for set_ in sets:
            where, ins = set_[0], set_[1]
            scale = set_[2] if len(set_) > 2 else 1.0
            if not isinstance(ins, dict):
                raise OptError("MISSING_FIELD", where, "object {Inport: definition} required")
            types, ints, n = {}, {}, None
            for name in d0.inports:
                t, xs = _ints_from_spec(name, ins.get(name), where)
                if n is not None and len(xs) != n and self.mr_info is None:
                    raise OptError("BAD_INPUT", where + "." + name, "all inputs of one set must have the same length")
                n = len(xs)
                types[name], ints[name] = t, xs
            extra_names = sorted(set(ins) - set(d0.inports))
            if extra_names:
                raise OptError("BAD_INPUT", where + "." + extra_names[0], "no Inport named %s" % extra_names[0])
            if self.in_types is None:
                self.in_types = types
            elif types != self.in_types:
                raise OptError("BAD_INPUT", where, "every input set must use the same Inport types")
            for label, vints in [(where, ints)] + [(where + "~" + v, _variant(v, ints, types))
                                                    for v in self.input_variants]:
                if self.mr_info is not None:   # samples at each Inport's rate -> every base step (held)
                    import fxlower
                    vints = fxlower.expand_inputs(self.mr_info, vints)
                vn = min(len(v) for v in vints.values())
                vints = {k: v[:vn] for k, v in vints.items()}
                if label != where and any(self._same(sc["ints"], vints) for sc in self.scenarios):
                    continue   # e.g. a symmetric signal: its variant is already judged
                self.scenarios.append({"label": label, "ints": vints, "nsteps": vn, "scale": scale})
        if isinstance(self.range_spec, dict):   # G9 variants stay inside explicitly given input ranges (saturated)
            for sc in self.scenarios:
                if "~" not in sc["label"]:
                    continue
                for name, r in self.range_spec.items():
                    if name in sc["ints"] and isinstance(r, dict) and name in (self.in_types or {}):
                        try:
                            f_ = self.in_types[name][2]
                            lo_, hi_ = math.ceil(float(r["min"]) * 2.0 ** f_), math.floor(float(r["max"]) * 2.0 ** f_)
                        except (KeyError, TypeError, ValueError, OverflowError):
                            continue   # malformed ranges are reported by the range parser below
                        sc["ints"][name] = [min(hi_, max(lo_, v)) for v in sc["ints"][name]]
        if self.range_spec is not None:
            self._add_range_scenarios(req)
        self.use_scenario(0)
        self.given_sets = len(sets)
        self.set_scales = {st[0]: st[2] for st in sets if len(st) > 2}
        if self.vec_info is not None and self.in_types:
            # the elements of one vector Inport have one data type in Simulink (accepted)
            seen_t = {}
            for name, (orig, _k) in self.vec_info["element"].items():
                if name in self.in_types:
                    if seen_t.setdefault(orig, self.in_types[name]) != self.in_types[name]:
                        raise OptError("BAD_INPUT", "inputs." + name, "the elements of the vector Inport %s must "
                                       "have the same data type" % orig)
        wls = req.get("wordlengths", list(range(6, 33, 2)))
        if not (isinstance(wls, list) and wls and all(isinstance(w, int) and 2 <= w <= 62 for w in wls)):
            raise OptError("BAD_PARAM", "wordlengths", "list of integers 2..62 required")
        self.wls = sorted(set(wls))
        err = req.get("error", {})
        if not isinstance(err, dict):
            raise OptError("BAD_PARAM", "error", "object required")
        extra_ = sorted(set(err) - {"max_abs", "outports"})
        if extra_:
            # error {"max": 1} ran silently at the default 2^-10
            raise OptError("BAD_PARAM", "error." + extra_[0], "unknown error field %s (max_abs, outports)" % extra_[0])
        tol = err.get("max_abs", 2.0 ** -10)
        per = err.get("outports")
        if per is not None and not isinstance(per, dict):
            raise OptError("BAD_PARAM", "error.outports", "object {Outport: tolerance} required")
        self.tols = {}
        for o in d0.outports:
            v = per.get(o, tol) if isinstance(per, dict) else tol
            try:
                v = float(v)
            except (TypeError, ValueError):
                raise OptError("BAD_PARAM", "error.outports." + o, "number required")
            if not v > 0:
                raise OptError("BAD_PARAM", "error", "tolerances must be positive")
            self.tols[o] = v
        if isinstance(per, dict):
            for o in per:
                if o not in self.tols:
                    raise OptError("BAD_PARAM", "error.outports." + o, "no Outport named %s" % o)
            if "max_abs" not in err:
                self.tols = {o: self.tols[o] for o in per}  # outports alone: only the listed outports are judged
            # with max_abs given too, the outports not listed are judged against max_abs (
            # they were silently not judged and max_abs ignored)
        if not self.tols:
            raise OptError("UNSUPPORTED", "design", "design needs at least one Outport to judge")
        self.tol = min(self.tols.values())
        fixed = req.get("fixed_blocks", [])
        if not isinstance(fixed, list) or any(f not in d0.btype for f in fixed):
            raise OptError("BAD_PARAM", "fixed_blocks", "list of block names required")
        self.fixed = set(fixed) | (set(self.mr_info["internal"]) if self.mr_info else set())
        # the blocks the block-cost hook counts under their owner keep their types (they are the owner's internals)
        self.fixed |= set(self._hook_skip)
        if self.mr_info and self.mr_info.get("filters"):
            # the lowering wrote the filters' internal types for the input type they had; keep that input type
            dorig = G.Design(self.spec_orig, self.hw)
            for f_ in self.mr_info["filters"]:
                fp = dorig.params[f_]
                if dorig.btype[f_].startswith("FIR"):
                    explicit = fp["firstCoeffMode"] == "Binary point scaling" and all(
                        fp[k] == "Binary point scaling" for k in ("prodOutputMode", "accumMode", "outputMode"))
                else:
                    explicit = fp["filterInternals"] == "Binary point scaling"
                if explicit:
                    continue   # explicit internal types: the input type may change freely
                src = dorig.src[f_][0]
                while _passes(dorig, src) or dorig.btype[src] in ("Downsample", "Upsample", "RateTransition"):
                    src = dorig.src[src][0]
                self.fixed.add(self.mr_info["carrier"].get(src, src))
        self.vars = []
        for b in d0.names:
            if b in self.fixed:
                continue
            if any(ch in b for ch in G.RESERVED_CHARS) and b not in self.spec["blocks"]:
                # recordings (recorded): an internal block a recorded lowering created (a synchronous delay's reset
                # select '\x1dD/rst') is not in the user's design -- its type follows the lowering, as the
                # cost-hook owners' internals do; it was a variable and fxopt stopped with KeyError
                continue
            if d0.params[b].get("_Tie"):
                continue   # an element copy of a vector block: its types follow the first copy
            for key in block_keys(d0.btype[b], d0.params[b]):
                if (b, key) in self.pinned:
                    continue   # the type a typed port declares
                self.vars.append((b, key))
        if not self.vars:
            # said "all blocks fixed?" when nothing was fixed -- say why each block has no type
            why = []
            if fixed:
                why.append("fixed_blocks: %s" % ", ".join(sorted(fixed)[:8]))
            filt = sorted((self.mr_info or {}).get("filters") or [])
            if filt:
                why.append("the internal types of multirate filters (%s) are not optimised; the input type a filter "
                           "depends on is kept" % ", ".join(filt[:8]))
            rest = sorted(b for b in d0.names if not any(ch in b for ch in G.RESERVED_CHARS) and b not in fixed
                          and not block_keys(d0.btype[b], d0.params[b]) and d0.btype[b] not in ("Inport", "Outport"))
            if rest:
                why.append("blocks without a data-type parameter to choose: %s" % ", ".join(rest[:8]))
            raise OptError("UNSUPPORTED", "design", "no data type to choose in this design (%s)"
                           % ("; ".join(why) or "every block's type is given by its inputs"))
        self.nvar = len(self.vars)
        # a word-length range per block ({block: [lo, hi]}); every decision variable of the block
        # (output, accumulator, parameter, ...) is held inside it by repair(), so the search, the cost and the proof
        # all see the capped design (unlike fixed_blocks, the block stays optimised and range-proven)
        bw = req.get("block_wordlengths", {})
        if not isinstance(bw, dict):
            raise OptError("BAD_PARAM", "block_wordlengths", "object {block: [min word length, max word length]}")
        self._caps = {}
        for b, r in bw.items():
            where = "block_wordlengths." + str(b)
            if b not in d0.btype or d0.btype[b] in ("Inport", "Outport"):
                raise OptError("BAD_PARAM", where, "no optimised block named %s" % b)
            if b in self.fixed:
                raise OptError("BAD_PARAM", where, "%s is in fixed_blocks: its types are not optimised" % b)
            if not (isinstance(r, list) and len(r) == 2 and all(isinstance(x, int) and not isinstance(x, bool)
                                                                for x in r) and 2 <= r[0] <= r[1] <= 62):
                raise OptError("BAD_PARAM", where, "[min, max] word lengths, integers 2 <= min <= max <= 62")
            ok = [w for w in self.wls if r[0] <= w <= r[1]]
            if not ok:
                raise OptError("BAD_PARAM", where, "no word length of the wordlengths set %s lies in %s"
                               % (self.wls, r))
            idx = [i for i, (vb, _k) in enumerate(self.vars) if vb == b]
            if not idx:
                raise OptError("BAD_PARAM", where, "%s has no optimised data type" % b)
            for i in idx:
                self._caps[i] = (ok[0], ok[-1])
        # a 2-D linear table with a falling segment and a 32-bit table type is refused on the host
        # (Floor / Ceiling / Simplest / Nearest); the search starts from the widest word, so such a table's types are
        # kept below 32 bits automatically (reported as auto_caps)
        self.auto_caps = {}
        if not G.fc._asic(self.hw):
            for b in d0.names:
                p_ = d0.params[b]
                if d0.btype[b] != "Lookup2D" or p_.get("InterpMethod") != "Linear point-slope" or \
                        p_.get("RndMeth", "Floor") not in ("Floor", "Ceiling", "Simplest", "Nearest") or b in self.fixed:
                    continue
                try:
                    t2 = G.fc._mat(p_["Table"])
                except (ValueError, KeyError):
                    continue
                if any(t2[i + 1][j] < t2[i][j] for i in range(len(t2) - 1) for j in range(len(t2[0]))) or \
                        any(t2[i][j + 1] < t2[i][j] for i in range(len(t2)) for j in range(len(t2[0]) - 1)):
                    ok = [w for w in self.wls if w <= 31]
                    if ok:
                        for i, (vb, _k) in enumerate(self.vars):
                            if vb == b:
                                lo_, hi_ = self._caps.get(i, (ok[0], ok[-1]))
                                self._caps[i] = (max(lo_, ok[0]), min(hi_, ok[-1]))
                        self.auto_caps[b] = [ok[0], ok[-1]]
        self.block_wordlengths = {b: list(r) for b, r in bw.items()}
        self._repair_cache = {}
        self.unsigned_ok = bool(req.get("allow_unsigned", True))
        slack = req.get("hwbits_slack", 0.05)
        if not isinstance(slack, (int, float)) or isinstance(slack, bool) or not 0 <= slack <= 1:
            raise OptError("BAD_PARAM", "hwbits_slack", "number in 0..1 required")
        self.hwbits_slack = float(slack)   # a hardware-model choice may cost at most this much more hwbits
        cost = req.get("cost", {"hwbits": 1})
        self.cost_model = None
        if isinstance(cost, dict) and set(cost) & {"fpga_xc7", "asic_gates"}:
            # target-hardware cost models (engine/hwcost.py); one model per request, weight 1
            if len(cost) != 1 or list(cost.values())[0] != 1:
                raise OptError("BAD_PARAM", "cost", '{"fpga_xc7": 1} or {"asic_gates": 1} on its own')
            self.cost_model = list(cost)[0]
            self.cost_w = (0.0, 0.0)
            self._hwcache = {}
        elif not (isinstance(cost, dict) and cost and set(cost) <= {"hwbits", "mult_area"} and
                  all(isinstance(v, (int, float)) and not isinstance(v, bool) and v >= 0 for v in cost.values())
                  and any(v > 0 for v in cost.values())):
            raise OptError("BAD_PARAM", "cost", '{"hwbits": a, "mult_area": b} with non-negative weights, or '
                           '{"fpga_xc7": 1}, or {"asic_gates": 1} required')
        else:
            self.cost_w = (float(cost.get("hwbits", 0)), float(cost.get("mult_area", 0)))
        ref = req.get("reference", "ideal")
        if ref == "ideal":
            self.reference = "ideal"
        elif isinstance(ref, dict) and ref.get("kind") == "design":
            self.reference = "design"
        else:
            raise OptError("BAD_PARAM", "reference", '"ideal" or {"kind": "design"}')
        self._weights()
        self.incr = G.IncrementalCompiler(self.spec, self.hw, self.in_types)

    def guarantees(self, proof=None):
        """What the result guarantees : overflow and error are reported separately, each with a status
        bounded | sampled | unknown | unsupported; passing tests never upgrades a status to bounded."""
        # a set judged against a relaxed tolerance (tolerance_scale / stress_tolerance_scale) says so
        sets = [dict({"label": sc["label"], "samples": sc["nsteps"]},
                     **({"tolerance_scale": sc["scale"]} if sc.get("scale", 1.0) != 1.0 else {}))
                for sc in self.scenarios]
        scaled = any(sc.get("scale", 1.0) != 1.0 for sc in self.scenarios)
        contract = {"inputs": {n: {"type": "fixdt(%d,%d,%d)" % self.in_types[n]} for n in self.d0.inports},
                    "error_reference": self.reference if self.reference == "ideal" else "design",
                    "error_metric": "max_abs_diff per outport, every input set judged on its own",
                    "tolerances": dict(self.tols),
                    **({"scaled_input_sets": {sc["label"]: sc["scale"] for sc in self.scenarios
                                              if sc.get("scale", 1.0) != 1.0}} if scaled else {}),
                    "initial_state": "design initial conditions, reset before every input set"}
        g = {"contract": contract,
             "error": {"status": "sampled", "input_sets": sets,
                       "claim": ("max_abs_diff <= tolerance for every judged outport, checked bit-exactly on each "
                                 "input set above; no bound is claimed for any other input sequence" if not scaled else
                                 "max_abs_diff <= tolerance x the set's tolerance_scale for every judged outport, "
                                 "checked bit-exactly on each input set above -- the sets with a tolerance_scale "
                                 "(listed in input_sets) are judged against that multiple of the tolerance (above 1: "
                                 "relaxed, below 1: tightened), the others against the tolerance itself; no bound is "
                                 "claimed for any other input sequence")},
             "overflow": {"status": "sampled", "input_sets": sets,
                          "claim": "observed on the input sets above only; no claim for other inputs"}}
        if self.range_spec is not None:
            g["contract"]["input_ranges"] = self.range_summary
            g["contract"]["initial_state"] = proof["contract"]["initial_state"] if proof else self.initial_state
            g["contract"]["horizon"] = "unbounded (every time step)"
            g["contract"]["overflow_definition"] = proof["contract"]["overflow_definition"] if proof else None
            ov = {"status": proof["status"] if proof else "unknown", "input_sets": sets,
                  "ideal_bounds_used_for_sizing": self.bound_method + " (search heuristic, not part of the proof)"}
            if proof:
                ov["proof"] = {k: proof[k] for k in ("fingerprint", "points_checked", "loops", "points_not_proven",
                                                    "reasons", "reason", "witness") if k in proof}
            ov["claim"] = {"violated": "an input sequence inside input_ranges overflows a quantisation point (see "
                                       "proof.witness); the design is NOT overflow-free for these ranges",
                           "bounded": "proven for every input sequence inside input_ranges, at every time step: no "
                                      "quantisation point of this design wraps or saturates",
                           "unknown": "the proof did not succeed (bounds are conservative; this is not a proof that "
                                      "an overflow happens); only the input sets above were observed",
                           "unsupported": "the design contains elements without a range proof; only the input sets "
                                          "above were observed"}[ov["status"]]
            g["overflow"] = ov
        return g

    @staticmethod
    def _same(a, b):
        return a.keys() == b.keys() and all(a[k] == b[k] for k in a)

    def use_scenario(self, k):
        """Make input set k the current one (in_ints / nsteps are read by the pool and the reference)."""
        sc = self.scenarios[k]
        self.in_ints, self.nsteps = sc["ints"], sc["nsteps"]
        self.cur_scale = sc.get("scale", 1.0)   # the set's tolerance multiple (tolerance_scale)

    def _add_range_scenarios(self, req):
        """Range mode : set up input_ranges; implemented in fxrange (bounds) and here (stress sets)."""
        import fxrange as R
        R.add_range_scenarios(self, req)

    # ---- cost: weights per variable + constant part (structural, no compile needed)
    def _root(self, n):
        seen = set()
        while _passes(self.d0, n):
            if n in seen:
                raise OptError("TYPE_UNRESOLVED", "design.blocks." + n, "cycle of type-inheriting blocks")
            seen.add(n)
            n = self.d0.src[n][0]
        return n

    def _weights(self):
        d0 = self.d0
        # Count each output word and the storage for delays and FIR taps.
        self.outw = {b: 1 for b in d0.names if not _passes(d0, b) and d0.btype[b] != "Outport" and self.costed(b)}
        for b in d0.names:
            if _passes(d0, b) and self.costed(b):
                k = int(d0.params[b].get("DelayLength", "1")) if d0.btype[b] == "Delay" else 1
                # (a root that is not costed itself -- the output conversion inside a lowered multirate filter,
                # carried by the filter's Hold -- starts from 0, not 1)
                r_ = self._root(b)
                self.outw[r_] = self.outw.get(r_, 1 if self.costed(r_) else 0) + k
        for _o, v in getattr(self, "_hook_bits", {}).items():   # cost-hook words that follow a signal's type
            for blk, cnt in v[2].items():
                if blk in d0.names:
                    r_ = self._root(blk)
                    self.outw[r_] = self.outw.get(r_, 1 if self.costed(r_) else 0) + cnt
        self._fir_state_const = 0
        self._fir_state_acc = {}
        for b in d0.names:
            if d0.btype[b] == "DiscreteFir" and d0.src[b] and b not in self.fixed:   # fixed: in inner_wl
                n = len(G.fc._dfir_coefs(d0.params[b]))
                st = str(d0.params[b].get("StateDataTypeStr", "Inherit: Same as accumulator"))
                if st == "Inherit: Same as accumulator":
                    self._fir_state_acc[b] = n - 1   # weight on the block's accumulator variable (below)
                elif st.startswith("Inherit"):
                    r = self._root(d0.src[b][0])
                    self.outw[r] = self.outw.get(r, 1) + (n - 1)
                else:
                    t = G._fixdt(st, "")
                    self._fir_state_const += (n - 1) * (t[1] if t else 0)
        self.vindex = {v: i for i, v in enumerate(self.vars)}
        self.weight = []
        for b, key in self.vars:
            w = self.outw[b] if key == "OutDataTypeStr" else 1  # INNER keys count once
            if d0.btype[b] == "DiscreteFir" and key in ("CoefDataTypeStr", "ProductDataTypeStr"):
                w = len(G.fc._dfir_coefs(d0.params[b]))   # one coefficient and one product per tap
            elif d0.btype[b] == "DiscreteFir" and key == "AccumDataTypeStr":
                # + the states when they are 'Same as accumulator'; one coefficient: no accumulator
                w = (1 if len(G.fc._dfir_coefs(d0.params[b])) > 1 else 0) + self._fir_state_acc.get(b, 0)
            elif d0.btype[b] == "Lookup1D" and key in (LUT1D_BP, LUT1D_TAB):
                w = _lut_entries(d0.params[b], d0.btype[b])   # one stored word per breakpoint / table entry
            elif d0.btype[b] == "Lookup2D" and key == LUT1D_TAB:
                w = _lut_entries(d0.params[b], d0.btype[b])
            elif d0.btype[b] == "Lookup2D" and key in LUT2D_BP:
                w = len(_lut2d_bps(d0.params[b], 1 if key == LUT2D_BP[0] else 2))
            elif d0.btype[b] in LUTS and key == "OutDataTypeStr" and (b, LUT1D_TAB) not in self.vindex and \
                    _lut_table_follows_output(d0.params[b]):
                w += _lut_entries(d0.params[b], d0.btype[b])  # table storage: one word per entry, in the output type
            self.weight.append(w)
        # vector element copies tied to a base block carry the same types: count them with the base's variables
        copies = {}
        for b in d0.names:
            base = d0.params[b].get("_Tie")
            if base:
                copies[base] = copies.get(base, 0) + 1
        if copies:
            self.weight = [w * (1 + copies.get(b, 0)) for w, (b, key) in zip(self.weight, self.vars)]

    def const_cost(self, types):
        """Cost of everything that is not a decision variable (Inports, booleans, fixed blocks, blocks costed by the
        block-cost hook)."""
        d0 = self.d0
        c = sum(v[0] for v in getattr(self, "_hook_bits", {}).values()) + getattr(self, "_mr_filter_bits", 0)
        for b, w in self.outw.items():
            bt = d0.btype[b]
            if (b, "OutDataTypeStr") in self.vindex:
                continue
            base = d0.params[b].get("_Tie")
            if base and (base, "OutDataTypeStr") in self.vindex:
                continue   # counted with the base block's variable
            if bt == "Inport":
                c += w * self.in_types[b][1]
            elif bt in ("RelationalOperator", "LogicalOperator"):
                c += w * 1
            else:
                t = types[b]
                c += w * (1 if t == G.BOOL else t[1])
        c += self._fir_state_const
        for b in d0.names:
            if b in self.fixed:
                # only costed blocks: the lowering's internal blocks of a multirate FIR / CIC filter (Gains, Sums,
                # conversions) are not counted by compiled_hwbits either -- counting their inner word lengths here
                # ended every fxopt run on such a design INTERNAL "structural cost != compiled cost" (multirate
                # recordings)
                if self.costed(b):
                    c += self.fixed_inner_wl.get(b, 0)
            elif d0.btype[b] in LUTS and (b, LUT1D_TAB) not in self.vindex and not _lut_table_follows_output(d0.params[b]):
                c += _lut_entries(d0.params[b], d0.btype[b]) * G._fixdt(d0.params[b]["TableDataTypeStr"], "")[1]
            if d0.btype[b] == "Lookup2D" and (b, LUT2D_BP[0]) not in self.vindex and b not in self.fixed:
                for dim in (1, 2):   # breakpoints kept in their given type (Nearest)
                    src = d0.src[b][dim - 1]
                    if types.get(src) not in (None, G.BOOL):
                        bt_ = G.fc._lut_type(d0.params[b]["BreakpointsForDimension%dDataTypeStr" % dim], types[src])
                        if isinstance(bt_, tuple):
                            c += len(_lut2d_bps(d0.params[b], dim)) * bt_[1]
            if d0.btype[b] == "Lookup1D" and (b, LUT1D_BP) not in self.vindex and b not in self.fixed:
                # breakpoints kept in their given type (Nearest): counted at the compiled width of that type
                bt_ = G.fc._lut_type(d0.params[b]["BreakpointsForDimension1DataTypeStr"], types[d0.src[b][0]]) \
                    if types.get(d0.src[b][0]) not in (None, G.BOOL) else None
                if isinstance(bt_, tuple):
                    c += _lut_entries(d0.params[b], d0.btype[b]) * bt_[1]
        return c

    def hwbits(self, wl):
        wl = self.repair(wl)
        return self._const + sum(w * x for w, x in zip(self.weight, wl))

    # ---- ranges from the double reference (set by the pool)
    def set_ranges(self, q):
        """q: {block: (min, max)} of the double-precision simulation."""
        d0 = self.d0
        self._needs_signed = set()
        for n in d0.names:
            if d0.btype[n] == "UnaryMinus":
                self._needs_signed.add(self._root(d0.src[n][0]))
        self.range = q
        # a Discrete-Time Integrator (internal rule) keeps its state in the input's word with the slope times K,
        # so the variable it inherits from must also cover the integrator's range divided by K
        extra = {}
        for n in d0.names:
            if d0.btype[n] == "DiscreteIntegrator":
                K = float(d0.param_record(n).gain)
                r = self._root(n)
                lo, hi = q[n][0] / K, q[n][1] / K
                if r in extra:
                    lo, hi = min(lo, extra[r][0]), max(hi, extra[r][1])
                extra[r] = (lo, hi)
        slack = 2 * self.tol
        # vector element copies share their base block's types: its range covers every copy
        q = dict(q)
        self._tie_src = {}
        for b in d0.names:
            base = d0.params[b].get("_Tie")
            if base and b in q and base in q:
                (a0, b0), (a1, b1) = q[base], q[b]
                q[base] = (min(a0, a1), max(b0, b1))
            if base:
                # ... and so do the copies' hidden slots (a Discrete FIR's per-tap products): sized from element 1
                # alone, a vector FIR whose first element has the smaller range came out INFEASIBLE (overflow
                # recordings)
                pre = b + "\x1f"
                for hk in [k for k in q if k.startswith(pre)]:
                    bk = base + "\x1f" + hk[len(pre):]
                    if bk in q:
                        (a0, b0), (a1, b1) = q[bk], q[hk]
                        q[bk] = (min(a0, a1), max(b0, b1))
        for b in d0.names:   # a saturating accumulator's bound sums its sources: the copies' sources count too
            base = d0.params[b].get("_Tie")
            if base and len(d0.src[b]) == len(d0.src[base]):
                for sb_, s_ in zip(d0.src[base], d0.src[b]):
                    if sb_ in q and s_ in q and sb_ != s_:
                        (a0, b0), (a1, b1) = q[sb_], q[s_]
                        self._tie_src = getattr(self, "_tie_src", {})
                        self._tie_src[(base, sb_)] = (min(a0, a1, self._tie_src.get((base, sb_), (a0, b0))[0]),
                                                      max(b0, b1, self._tie_src.get((base, sb_), (a0, b0))[1]))
        self.vtype = []
        for b, key in self.vars:
            if key == "ParamDataTypeStr":
                gv = d0.params[b].get("_GainVec")
                if gv:   # a vector Gain: every element shares this type, so it must hold all element values
                    self.vtype.append(("coefs", tuple(float(v) for v in str(gv).strip("[]").replace(",", " ").split())))
                    continue
                g = float(d0.params[b]["Gain"])
                self.vtype.append(("param", g))
                continue
            if key == "CoefDataTypeStr":
                self.vtype.append(("coefs", tuple(G.fc._dfir_coefs(d0.params[b]))))
                continue
            if key == LUT1D_BP:
                self.vtype.append(("coefs", _lut1d_bps(d0.params[b])))
                continue
            if key in LUT2D_BP:
                self.vtype.append(("coefs", _lut2d_bps(d0.params[b], 1 if key == LUT2D_BP[0] else 2)))
                continue
            if key == LUT1D_TAB:
                self.vtype.append(("coefs", _lut_table_values(d0.params[b], d0.btype[b])))
                continue
            if key == LUT1D_FR:   # the interpolation fraction lies in [0, 1]; 1 must be representable unless the
                # last table value is used at the last breakpoint (UseLastTableValue on)
                self.vtype.append(("frac", d0.params[b].get("UseLastTableValue") == "on"))
                continue
            if key == LUT1D_IT:   # holds the table values, the interpolated value and the slope term
                # f * (y1 - y0), which is formed in this word (signed): |step| between neighbouring table values,
                # along either dimension of a 2-D table (a step of -1.2 saturated at -1)
                tv = _lut_table_values(d0.params[b], d0.btype[b])
                if d0.btype[b] == "Lookup2D":
                    t2 = G.fc._mat(d0.params[b]["Table"])
                    steps = [t2[i + 1][j] - t2[i][j] for i in range(len(t2) - 1) for j in range(len(t2[0]))] + \
                            [t2[i][j + 1] - t2[i][j] for i in range(len(t2)) for j in range(len(t2[0]) - 1)]
                else:
                    steps = [tv[i + 1] - tv[i] for i in range(len(tv) - 1)]
                ms = max([abs(x) for x in steps] or [0.0])
                lo, hi = min(min(tv), q[b][0], -ms), max(max(tv), q[b][1], ms)
                signed = lo < 0 or not self.unsigned_ok
                m = max(abs(lo), abs(hi)) + slack
                self.vtype.append(("sig", 1 if signed else 0, L.int_bits(m) if signed else max(L.int_bits(m) - 1, -60)))
                continue
            if key == "ProductDataTypeStr":  # Discrete FIR products: ranges of the hidden per-tap slots
                nc = len(G.fc._dfir_coefs(d0.params[b]))
                ps = [q[G._hid(b, "p%d" % k)] for k in range(nc)]
                lo, hi = min(x[0] for x in ps), max(x[1] for x in ps)
            else:
                lo, hi = q[b]
            if key == "OutDataTypeStr" and b in extra:
                lo, hi = min(lo, extra[b][0]), max(hi, extra[b][1])
            if key == "AccumDataTypeStr" and d0.btype[b] == "DiscreteFir" and \
                    d0.params[b].get("SaturateOnIntegerOverflow", "off") == "on":
                nc = len(G.fc._dfir_coefs(d0.params[b]))
                m = sum(max(abs(q[G._hid(b, "p%d" % k)][0]), abs(q[G._hid(b, "p%d" % k)][1])) for k in range(nc))
                lo, hi = -m, m
            if key == "AccumDataTypeStr" and d0.params[b].get("SaturateOnIntegerOverflow", "off") == "on":
                # saturating accumulator must hold every partial sum: bound by the sum of input magnitudes
                ts_ = getattr(self, "_tie_src", {})
                m = sum(max(abs(ts_.get((b, s), q[s])[0]), abs(ts_.get((b, s), q[s])[1])) for s in d0.src[b])
                lo, hi = -m, m
            if not (math.isfinite(lo) and math.isfinite(hi)):
                raise OptError("REFERENCE_NOT_FINITE", "design.blocks." + b,
                               "the double-precision reference of this block is not finite (division by zero?)")
            # a signal that reaches a Unary Minus (directly or through type-inheriting blocks) must be signed:
            # Simulink rejects an unsigned input there (PortValidationFixptError)
            signed = (lo < 0) or not self.unsigned_ok or (key == "OutDataTypeStr" and b in self._needs_signed)
            m = max(abs(lo), abs(hi)) + slack
            ib = L.int_bits(m) if signed else max(L.int_bits(m) - 1, -60)
            self.vtype.append(("sig", 1 if signed else 0, ib))
        # modular regions (engine/fxmodular.py, fxopt): every Sum type of an integrator-comb chain keeps the shared
        # integer width the wrap argument needs (its range from double simulation would be the unwrapped one); the
        # search chooses each register's fraction length (Hogenauer pruning)
        self._mod_vars = set()
        for r in (getattr(self, "req", None) or {}).get("_modular") or []:
            for i, (b, key) in enumerate(self.vars):
                if b in r["blocks"] and key in ("OutDataTypeStr", "AccumDataTypeStr") and self.vtype[i][0] == "sig":
                    self.vtype[i] = ("sig", 1, r["ib"])
                    self._mod_vars.add(i)
        self._par = {}
        # with reference 'design' the design's own parameter types are a point of the search space
        # (at their own word length a parameter takes the design's type, which reproduces the yardstick exactly);
        # otherwise a Gain 0.3 typed fixdt(1,4,2) -- 0.25 in the design -- is re-quantised from 0.3 at every word
        # length, no candidate holds 0.25, and a request the design itself solves with zero error was INFEASIBLE
        self.pins = {}
        if self.reference == "design":
            for i, (b, key) in enumerate(self.vars):
                t0 = G.fc._named_type(str(d0.params[b].get(key, "")))
                # signal types too -- the design itself (error 0 against itself) is then a candidate
                if self.vtype[i][0] in ("param", "coefs", "sig") and isinstance(t0, tuple) and t0[1] in self.wls:
                    self.pins[i] = tuple(t0)

    def var_type(self, i, w):
        vt = self.vtype[i]
        pin = self.pins.get(i) if hasattr(self, "pins") else None
        if pin is not None and pin[1] == w:
            return pin
        if vt[0] == "frac":
            return (0, w, w) if vt[1] else (0, w, w - 1)
        if vt[0] == "coefs":
            key = (i, w)
            r = self._par.get(key)
            if r is None:
                cs = vt[1]
                s = 1 if any(c < 0 for c in cs) else 0
                fls = [L.best_fl(repr(c), s, w) for c in cs if c != 0]
                r = (s, w, min(fls) if fls else w - s)
                self._par[key] = r
            return r
        if vt[0] == "param":
            key = (i, w)
            r = self._par.get(key)
            if r is None:
                g = vt[1]
                if g == 0:
                    r = (1, w, w - 1)
                else:
                    s = 1 if g < 0 else 0
                    r = (s, w, L.best_fl(repr(g), s, w))
                self._par[key] = r
            return r
        _, s, ib = vt
        return (s, w, w - ib)

    def blocks(self, wl):
        wl = self.repair(wl)
        out = {}
        for b, p in self.spec["blocks"].items():
            out[b] = dict(p)
        for i, (b, key) in enumerate(self.vars):
            out[b][key] = "fixdt(%d,%d,%d)" % self.var_type(i, wl[i])
        for b, p in self.spec["blocks"].items():   # vector element copies take the first copy's types
            base = p.get("_Tie")
            if base:
                for key in block_keys(p["type"], p):
                    if key in out[base]:
                        out[b][key] = out[base][key]
        return out

    def overrides(self, wl):
        wl = self.repair(wl)
        ov = {}
        for i, (b, key) in enumerate(self.vars):
            ov.setdefault(b, {})[key] = "fixdt(%d,%d,%d)" % self.var_type(i, wl[i])
        for b, p in self.spec["blocks"].items():
            base = p.get("_Tie")
            if base and base in ov:
                ov[b] = dict(ov[base])
        return ov

    def design(self, wl):
        spec = _with_vec_key({"blocks": self.blocks(wl), "lines": self.spec["lines"]}, self.spec)
        d = G.Design(spec, self.hw)
        d.compile(self.in_types)
        return d

    def reference_waveforms(self):
        """Judged outputs as floats: ideal double simulation, or the bit-exact output of the design with the
        data types given in the request (fxpopt's yardstick when it starts from an existing fixed-point design)."""
        d0 = self.d0
        if self.reference == "ideal":
            return d0.run_double({n: [math.ldexp(v, -self.in_types[n][2]) for v in self.in_ints[n]]
                                  for n in d0.inports}, self.nsteps, list(self.tols))
        try:
            d = G.Design(self.spec, self.hw)
            d.compile(self.in_types)
            judged = list(self.tols)
            if d.vm_ok:
                logs = G.run_fast(d, self.in_ints, self.nsteps, judged)[0]
            else:
                logs = d.run(self.in_ints, self.nsteps, judged)
        except G.GraphError as e:
            raise OptError(e.code, "reference." + e.where, "the given design cannot be simulated: " + e.message)
        out = {}
        for o in judged:
            t = d.otype[o]
            if t == G.BOOL:
                raise OptError("UNSUPPORTED", "error.outports." + o, "boolean outputs cannot be judged")
            if any(float(v) != v for v in logs[o]):
                # a 62-bit reference output (2^60 + 1) became 2^60 in double, so a plan 1 .. 3 off was
                # reported exact; values the comparison cannot hold exactly are refused
                raise OptError("UNSUPPORTED", "error.outports." + o, "the reference design's output %s has values "
                               "with more than 53 significant bits (%s); they cannot be compared exactly: narrow the "
                               "output type or the inputs" % (o, G._tstr(t)))
            out[o] = [math.ldexp(v, -t[2]) for v in logs[o]]
        return out

    def _find_preadds(self):
        """Two-input Sums that feed only Gains, with both their accumulator and output types decision variables:
        [(sum, accumulator var index, output var index)]."""
        d0 = self.d0
        users = {x: [] for x in d0.names}
        for x in d0.names:
            for s_ in d0.src[x]:
                users[s_].append(x)
        out = []
        for b in d0.names:
            if d0.btype[b] != "Sum" or len(d0.src[b]) != 2 or not users[b]:
                continue
            if not all(d0.btype[u] == "Gain" for u in users[b]):
                continue
            ia, io = self.vindex.get((b, "AccumDataTypeStr")), self.vindex.get((b, "OutDataTypeStr"))
            if ia is None or io is None:
                continue
            out.append((b, ia, io))
        return out

    def _exchange_groups(self):
        """Move groups of the pair exchange (fxopt_lib._pair_exchange): each variable with the variables of its own
        block and of the blocks wired directly to it (both directions). Variables a repair overwrites (a tied
        pre-add output) are left out."""
        d0 = self.d0
        dead = {io for _b, _ia, io in self._preadds} if self._preadd_mode == "tie" else set()
        by_block = {}
        for i, (b, _k) in enumerate(self.vars):
            if i not in dead:
                by_block.setdefault(b, []).append(i)
        near = {b: set(d0.src.get(b, ())) for b in d0.names}
        for b in d0.names:
            for s_ in d0.src.get(b, ()):
                near.setdefault(s_, set()).add(b)
        groups = []
        for b, idx in by_block.items():
            others = [j for nb in sorted(near.get(b, ())) for j in by_block.get(nb, ())]
            for i in idx:
                js = [j for j in idx if j != i] + [j for j in others if j != i]
                if js:
                    groups.append((i, js))
        return groups

    def _signal_type(self, x, wl):
        """Type of signal x under candidate wl when it is a decision variable or an input (else None)."""
        r = self._root(x)
        i = self.vindex.get((r, "OutDataTypeStr"))
        if i is not None:
            return self.var_type(i, wl[i])
        if self.d0.btype[r] == "Inport":
            t = self.in_types.get(r)
            return tuple(t) if t and t != G.BOOL else None
        return None

    def repair(self, wl):
        """wl with every pre-adder's accumulator and output word raised to the smallest allowed word that keeps the
        finest input fraction (no low bit dropped before the multiplier; the integer bits follow the proven range)."""
        pre = getattr(self, "_preadds", None)
        caps = getattr(self, "_caps", None)
        if not pre and not caps:
            return wl
        key = tuple(wl)
        hit = self._repair_cache.get(key)
        if hit is not None:
            return hit
        out = self._capped(list(wl))
        if not pre:
            out = tuple(out) if isinstance(wl, tuple) else out
            self._repair_cache[key] = out
            return out
        if self._preadd_mode == "tie":
            # Tie the pre-add output type to its accumulator type.
            for b, ia, io in pre:
                if self.vtype[ia] == self.vtype[io]:
                    out[io] = out[ia]
            out = self._capped(out)
            out = tuple(out) if isinstance(wl, tuple) else out
            self._repair_cache[key] = out
            return out
        for b, ia, io in pre:
            ts = [self._signal_type(x, out) for x in self.d0.src[b]]
            if any(t is None for t in ts):
                continue
            need_f = max(t[2] for t in ts)
            for i in (ia, io):
                vt = self.vtype[i]
                if vt[0] != "sig":
                    continue
                need_w = need_f + vt[2]   # fraction = w - integer bits
                if out[i] < need_w:
                    up = [w for w in self.wls if w >= need_w]
                    if up:
                        out[i] = up[0]
        out = self._capped(out)
        out = tuple(out) if isinstance(wl, tuple) else out
        if len(self._repair_cache) > 200000:
            self._repair_cache.clear()
        self._repair_cache[key] = out
        return out

    def _capped(self, out):
        for i, (lo, hi) in (getattr(self, "_caps", None) or {}).items():
            out[i] = min(hi, max(lo, out[i]))
        return out

    def finish_setup(self):
        """Constant cost part needs the compiled types of fixed blocks (any candidate will do)."""
        flag = self.req.get("preadd_full_precision", "tie") if hasattr(self, "req") else "tie"
        if flag not in (True, False, "penalty", "tie"):
            raise OptError("BAD_PARAM", "preadd_full_precision", "true, false, \"penalty\" or \"tie\"")
        self._repair_cache = {}
        self._preadd_list = self._find_preadds()
        self._preadd_mode = flag if flag in ("penalty", "tie") else None
        self._preadds = self._preadd_list if flag in (True, "tie") else []   # repaired in repair()
        self.exchange_groups = self._exchange_groups()
        wl = [max(self.wls)] * self.nvar
        try:
            d = self.design(wl)
        except G.GraphError as e:
            raise OptError(e.code, e.where, e.message)
        # a fixed block whose type is inherited from optimised blocks changes width with the candidate,
        # so its cost is not the constant the search assumes (the run ended INTERNAL "structural cost differs from
        # compiled cost"): refused, naming the block
        user_fixed = [b for b in self.fixed if not b.startswith("\x1e") and b in d.otype]
        if user_fixed and len(self.wls) > 1:
            try:
                d_lo = self.design([min(self.wls)] * self.nvar)
            except G.GraphError:
                d_lo = None
            if d_lo is not None:
                for b in sorted(user_fixed):
                    if d.btype[b] in ("Inport", "Outport"):
                        continue
                    w_hi = (d.otype[b] if d.otype[b] == G.BOOL else d.otype[b][1], self.inner_wl(d, b))
                    w_lo = (d_lo.otype[b] if d_lo.otype[b] == G.BOOL else d_lo.otype[b][1], self.inner_wl(d_lo, b))
                    if w_hi != w_lo:
                        raise OptError("UNSUPPORTED", "fixed_blocks", "fixed block %s inherits its data type from "
                                       "optimised blocks (its width changes with the candidate): give it an explicit "
                                       "type, or do not fix it" % b)
        # a boolean outport has no error metric (as fxcert says); judging one used to crash the pool (INTERNAL)
        bools = sorted(o for o in self.tols if d.otype.get(o) == G.BOOL)
        if bools:
            raise OptError("UNSUPPORTED", "error.outports." + bools[0], "boolean outport %s has no error metric; "
                           "list only fixed-point outports in error.outports" % bools[0])
        self.fixed_inner_wl = {b: self.inner_wl(d, b) for b in self.fixed}
        self._setup_mult_terms(d)
        self._const = 0
        self._const = self.const_cost(d.otype)
        # cross-check the structural cost against the compiled design
        exact = self.compiled_hwbits(d)
        if exact != self.hwbits(wl):
            raise OptError("INTERNAL", "cost", "structural cost %d != compiled cost %d" % (self.hwbits(wl), exact))
        # ... and at the other end of the word-length range: a block whose stored bits follow another block's type
        # (a Lookup with breakpoints 'Same as corresponding input' behind a computed
        # signal) is not in the linear cost model -- the run used to end INTERNAL "structural cost differs from
        # compiled cost" after the whole search
        if len(self.wls) > 1:
            wl_lo = [min(self.wls)] * self.nvar
            try:
                d_lo = self.design(wl_lo)
            except G.GraphError:
                d_lo = None
            if d_lo is not None and self.compiled_hwbits(d_lo) != self.hwbits(wl_lo):
                luts = sorted(b for b in d_lo.names if d_lo.btype[b] in ("Lookup1D", "Lookup2D") and any(
                    str(v).startswith("Inherit: Same as corresponding input")
                    for k, v in d_lo.params[b].items() if k.startswith("BreakpointsForDimension") and k.endswith("DataTypeStr")))
                raise OptError("UNSUPPORTED", "design" + (".blocks." + luts[0] if luts else ""),
                               "the cost of a block follows another block's data type in a way the cost model does "
                               "not cover%s" % (" (lookup table %s with breakpoints 'Inherit: Same as corresponding "
                                                "input' behind an optimised signal: give the breakpoints an "
                                                "explicit type)" % ", ".join(luts[:3]) if luts else ""))

    @staticmethod
    def inner_wl(d, b):
        """Word lengths inside a block that count in hwbits: Gain parameter, Sum accumulator, Discrete FIR
        coefficient + product + accumulator."""
        bt = d.btype[b]
        ts = d._in_types(b)
        if bt == "Gain":
            return d.gain_record(b).param_type[1]   # the compiled record (engine/fxrecords.GainRecord)
        if bt == "Sum":
            if b in getattr(d, "acc_types", {}):
                return d.acc_types[b][1]
            t = G._fixdt(d.params[b]["AccumDataTypeStr"], "")
            return t[1] if t else 0
        if bt == "DiscreteFir":
            # the compiled types (the direct-form rule re-derived here missed the symmetric structures'
            # product / accumulator and counted inherited states at the input width; the counter reads compiled
            # types and resolves 'Same as accumulator' to the accumulator)
            n = len(G.fc._dfir_coefs(d.params[b]))
            ctp, pt, at, st = d.fir_types[b]
            # one coefficient: no accumulator in the arithmetic, so none in the cost
            return n * ctp[1] + n * pt[1] + (at[1] if n > 1 else 0) + (n - 1) * st[1]
        if bt in ("FIRDecimation", "FIRInterpolation", "CICDecimation", "CICInterpolation"):
            return multirate_filter_bits(d, b)
        if bt in LUTS:
            rec = d.lut_record(b)
            if rec is not None:   # the compiled record's types (engine/fxlutplan.py stored_bits)
                return rec.stored_bits()
            return lut_bits_from_params(d, b, ts)
        return 0

    def cost(self, wl):
        if self.cost_model:
            return self.hw_estimate(wl)["cost"]
        a, b = self.cost_w
        return a * self.hwbits(wl) + (b * self.mult_area(wl) if b else 0.0) + self.preadd_penalty(wl)

    def preadd_penalty(self, wl):
        """preadd_full_precision "penalty" (the default): a pre-adder whose accumulator or output drops low bits of
        its inputs is charged its output word in bits -- about the fabric adder it then needs, since the DSP48E1
        pre-adder cannot take it -- so the search keeps the full fraction when that costs fewer bits than the adder.
        Not part of hwbits."""
        if getattr(self, "_preadd_mode", None) != "penalty" or not self._preadd_list:
            return 0
        c = 0
        for b, ia, io in self._preadd_list:
            ts = [self._signal_type(x, wl) for x in self.d0.src[b]]
            if any(t is None for t in ts):
                continue
            need_f = max(t[2] for t in ts)
            if any(self.var_type(i, wl[i])[2] < need_f for i in (ia, io)):
                c += wl[io]
        return c

    def hw_estimate(self, wl):
        """Predicted resources of candidate wl under the models (an estimate, not a synthesis result).
        fpga_xc7: DSP48E1 first, then LUT + FF (cost = 1e6 x DSP + LUT + FF); asic_gates: generic gates + DFF."""
        import hwcost as H
        wl = self.repair(wl)
        key = (tuple(wl), tuple(self.vtype))
        r = self._hwcache.get(key) if hasattr(self, "_hwcache") else None
        if r is not None:
            return r
        try:
            dz = self.design(wl)
            inv = H.design_inventory(dz)
            ab = H.preadd_absorbed(dz, inv, H.load_model()["dsp_rule"]) if isinstance(inv, dict) else {}
            p = H.predict(inv, absorbed_bits=sum(ab.values())) if ab else H.predict(inv)
        except G.GraphError:
            return {"cost": math.inf}
        r = {"dsp48e1": p["dsp"], "lut": round(p["lut_dsp"]), "ff": p["ff_dsp"], "lut_nodsp": round(p["lut_nodsp"]),
             "generic_gates": round(p["gates"]), "dff": p["features"]["reg_bits"]}
        model = self.cost_model or "fpga_xc7"
        # fpga_xc7 is the pair (DSP48E1, LUT + FF) in lexicographic order; the scalar 1e6 x DSP + (LUT + FF)
        # equals that order only while LUT + FF < 1e6, so larger predictions are refused, not mis-ranked
        if model == "fpga_xc7" and self.cost_model and r["lut"] + r["ff"] >= 1e6:
            raise OptError("UNSUPPORTED", "cost", "fpga_xc7: predicted LUT + FF %d >= 10^6, outside the domain where the "
                           "scalar cost is lexicographic" % (r["lut"] + r["ff"]))
        # asic_gates optimises the combinational generic-gate count; DFF bits are reported separately
        r["cost"] = (1e6 * r["dsp48e1"] + r["lut"] + r["ff"]) if model == "fpga_xc7" else r["generic_gates"]
        if not hasattr(self, "_hwcache"):
            self._hwcache = {}
        if len(self._hwcache) > 200000:
            self._hwcache.clear()
        self._hwcache[key] = r
        return r

    def _setup_mult_terms(self, d):
        """Multiplier area proxy terms: sum of (data word x coefficient word) over Gains, every two-operand
        product step, and every Discrete FIR tap (counted per tap). A term is a pair of word-length
        getters: ('var', index) or ('const', wl)."""
        d0 = self.d0

        def wl_of(src):
            r = self._root(src)
            i = self.vindex.get((r, "OutDataTypeStr"))
            if i is not None:
                return ("var", i)
            t = d.otype[r] if r in d.otype else None
            if t == G.BOOL:
                return None
            return ("const", t[1])
        terms = []
        for b in d0.names:
            bt = d0.btype[b]
            src = d0.src[b]
            if bt == "Gain":
                x = wl_of(src[0])
                i = self.vindex.get((b, "ParamDataTypeStr"))
                pw = ("var", i) if i is not None else ("const", self.inner_wl(d, b))
                if x is not None:
                    terms.append((x, pw, 1))
            elif bt == "Product":
                ins = [wl_of(x) for x in src]
                if len(ins) == 1 or any(x is None for x in ins):
                    continue
                o = self.vindex.get((b, "OutDataTypeStr"))
                ow = ("var", o) if o is not None else ("const", d.otype[b][1])
                terms.append((ins[0], ins[1], 1))
                for x in ins[2:]:
                    terms.append((ow, x, 1))
            elif bt == "DiscreteFir":
                x = wl_of(src[0])
                i = self.vindex.get((b, "CoefDataTypeStr"))
                nc = len(G.fc._dfir_coefs(d0.params[b]))
                if i is None:
                    cw = ("const", G.fc._dfir_types(d0.params[b], d._in_types(b)[0], G.fc._dfir_coefs(d0.params[b]),
                                                     d.words)[0][1])
                else:
                    cw = ("var", i)
                if x is None:
                    continue
                fs_ = d0.params[b].get("FilterStructure", "Direct form")
                if fs_ in ("Direct form symmetric", "Direct form antisymmetric"):
                    # recordings (pre-existing): one multiplier per coefficient pair, fed by the tap sum (an explicit
                    # tap-sum type has its own word; an inherited one follows the input), plus the centre tap
                    tts = str(d0.params[b].get("TapSumDataTypeStr", "Inherit: Same as input"))
                    tw = G._fixdt(tts, "") if not tts.startswith("Inherit") else None
                    terms.append((("const", tw[1]) if tw else x, cw, nc // 2))
                    if nc % 2:
                        terms.append((x, cw, 1))
                else:
                    terms.append((x, cw, nc))
        self._mult_terms = terms

    def mult_area(self, wl):
        wl = self.repair(wl)
        tot = 0
        for a, b, c in self._mult_terms:
            wa = wl[a[1]] if a[0] == "var" else a[1]
            wb = wl[b[1]] if b[0] == "var" else b[1]
            tot += c * wa * wb
        return tot

    def costed(self, b):
        """Count user blocks and the lowered blocks that carry their outputs.
        Exclude timing and control blocks introduced by multirate lowering."""
        if b in getattr(self, "_hook_skip", {}):   # counted under its owner by the block-cost hook (cost_hooks)
            return False
        if not b.startswith("\x1e"):
            return True
        info = getattr(self, "mr_info", None) or {}
        return b in set((info.get("carrier") or {}).values())

    def compiled_hwbits(self, d):
        c = sum(hook_cost(d, v) for v in cost_hooks(d)[1].values()) + getattr(self, "_mr_filter_bits", 0)
        for b in d.names:
            bt = d.btype[b]
            if bt == "Outport" or not self.costed(b):
                continue
            t = d.otype[b]
            k = int(d.params[b].get("DelayLength", "1")) if bt == "Delay" else 1   # a Delay's state registers
            c += k * (1 if t == G.BOOL else t[1])
            c += self.inner_wl(d, b)
        return c


class GraphPool:
    """Persistent `fxgraph_core -s` workers holding the inputs and the double reference."""

    def __init__(self, pb, nworkers, apply_ranges=True):
        if not os.path.exists(G.CORE):
            raise OptError("CORE_UNAVAILABLE", "engine/fxgraph_core", "core binary missing (run engine/build.sh)")
        self.pb = pb
        self.scale = getattr(pb, "cur_scale", 1.0)   # this pool's input set is judged against scale x the tolerance
        d0 = pb.d0
        slot, dins, ddly = G.vm_double_instructions(d0)
        self.slot = slot
        self.judge = list(pb.tols)
        head = []
        for n in d0.inports:
            head.append("IN %d %d %d %s" % (slot[n], pb.in_types[n][2], pb.nsteps, " ".join(map(str, pb.in_ints[n]))))
        head.append("DREF %d %d %s %d %s %d %s" % (len(d0.slots), len(dins), " ".join(dins), len(ddly), " ".join(ddly),
                                                   len(self.judge), " ".join(str(slot[o]) for o in self.judge)))
        data = ("\n".join(head) + "\n").encode()
        self.procs = []
        for _ in range(nworkers):
            p = subprocess.Popen([G.CORE, "-s"], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                 stderr=subprocess.DEVNULL, bufsize=1 << 16)
            p.stdin.write(data)
            p.stdin.flush()
            self.procs.append(p)
        q = None
        for p in self.procs:
            rq = {}
            while True:
                line = p.stdout.readline().decode()
                if not line:
                    raise OptError("CORE_FAILED", "core", "worker exited during DREF")
                parts = line.split()
                if parts[0] == "E":
                    raise OptError("CORE_FAILED", "core", line.strip())
                if parts[0] == "D":
                    break
                rq[int(parts[1])] = (float(parts[2]), float(parts[3]))
            q = q or rq
        names = d0.slots
        self.q = {names[i]: v for i, v in q.items()}
        if apply_ranges:
            pb.set_ranges(self.q)
        self.refwave = None
        if pb.reference == "design":
            self.refwave = pb.reference_waveforms()
            data = "".join("REFV %d %d %s\n" % (slot[o], pb.nsteps, " ".join(repr(v) for v in self.refwave[o]))
                           for o in self.judge).encode()
            for p in self.procs:
                p.stdin.write(data)
                p.stdin.flush()
        self.n = nworkers
        self.evals = 0
        self.core_ns = 0
        self.procs_lock = threading.Lock()

    def close(self):
        for p in self.procs:
            try:
                p.stdin.write(b"END\n")
                p.stdin.close()
            except OSError:
                pass
        for p in self.procs:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()

    def line(self, wl, full=False):
        """Candidate -> core line, or None if it cannot be built / is too wide (treated as infeasible).
        full=True builds it through a complete Design compile (reference path for the self-test)."""
        pb = self.pb
        try:
            if full:
                d = pb.design(wl)
                ok, fast, otype = d.vm_ok, d.vm_fast, d.otype
                slot, ins, dly = G.vm_instructions(d)
                wide, k = [], 0
                for n in d.order:
                    if n in d.vm:
                        c = len(G._flat(d.vm[n]))
                        if n in d.vm_wide:
                            wide.extend(range(k, k + c))
                        k += c
            else:
                ins, dly, otype, ok, fast, wide = pb.incr.program(pb.overrides(wl))
                slot = pb.incr.slot
        except G.GraphError:
            return None
        if not ok:
            return None
        ev = " ".join("%d %d" % (slot[o], otype[o][2]) for o in self.judge)
        fast = 1 if (fast and not os.environ.get("FXGRAPH_NO_FAST")) else 0
        wtxt = " ".join(map(str, wide)) if fast else ""
        return "C %d %d %d %d %s %s %d %s %d %s\n" % (len(slot), len(ins), fast, len(wide) if fast else 0, wtxt,
                                                      " ".join(ins), len(dly), " ".join(dly), len(self.judge), ev)

    def _run(self, wi, lines, out):
        p = self.procs[wi]
        try:
            p.stdin.write("".join(lines).encode())
            p.stdin.flush()
            res = []
            for _ in lines:
                res.append(self._read_result(p))
            out[wi] = res
        except Exception as e:  # noqa: BLE001
            out[wi] = e

    def _read_result(self, p):
        """One candidate's answer, strictly: an 'R' line, exactly one 'M' line for every judged
        output and nothing else, then 'D'; NaN errors are refused (inf means overflow and is kept)."""
        rec = {"start": 0}
        worst = None
        seen = set()
        while True:
            ln = p.stdout.readline().decode()
            if not ln:
                raise OptError("CORE_FAILED", "core", "worker exited")
            parts = ln.split()
            if not parts:
                raise OptError("CORE_FAILED", "core", "empty line from worker")
            if parts[0] == "E":
                raise OptError("CORE_FAILED", "core", ln.strip())
            if parts[0] == "R" and len(parts) == 2 and "core_ns" not in rec:
                rec["core_ns"] = int(parts[1])
            elif parts[0] == "M" and len(parts) >= 5:
                sl, mx, mean, var = int(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])
                if not (0 <= sl < len(self.pb.d0.slots)):
                    raise OptError("CORE_FAILED", "core", "error record for unknown slot %d" % sl)
                name = self.pb.d0.slots[sl]
                if name not in self.pb.tols or name in seen:
                    raise OptError("CORE_FAILED", "core", "unexpected or duplicate error record for %s" % name)
                if mx != mx:
                    raise OptError("CORE_FAILED", "core", "NaN error for %s" % name)
                seen.add(name)
                k = self.pb.tol / self.pb.tols[name] / getattr(self, "scale", 1.0)  # normalise to the strictest
                # tolerance (and this pool's tolerance_scale)
                if worst is None or mx * k > worst[0]:
                    worst = (mx * k, mean * k, var * k * k)
            elif parts[0] == "D" and len(parts) == 1:
                break
            else:
                raise OptError("CORE_FAILED", "core", "unexpected worker line %r" % ln[:60])
        if seen != set(self.pb.tols) or "core_ns" not in rec:
            raise OptError("CORE_FAILED", "core", "worker result incomplete: errors for %s, judged %s"
                           % (sorted(seen), sorted(self.pb.tols)))
        rec["err"], rec["mean"], rec["var"] = worst
        return rec

    def evaluate(self, base, cands, lines=None):
        """lines: the candidates' core lines when the caller built them (MultiPool: once for all input sets, in one
        thread -- building a line compiles through the problem's shared incremental compiler)."""
        if lines is None:
            lines = [self.line(c) for c in cands]
        res = [None] * len(cands)
        todo = [i for i, l in enumerate(lines) if l is not None]
        for i, l in enumerate(lines):
            if l is None:
                res[i] = {"err": math.inf, "mean": 0.0, "var": 0.0, "core_ns": 0, "start": 0}
        assign = [[] for _ in range(self.n)]
        for j, i in enumerate(todo):
            assign[j % self.n].append(i)
        out = [None] * self.n
        ths = []
        for wi in range(self.n):
            if assign[wi]:
                th = threading.Thread(target=self._run, args=(wi, [lines[i] for i in assign[wi]], out))
                th.start()
                ths.append(th)
        for th in ths:
            th.join()
        for wi in range(self.n):
            if not assign[wi]:
                continue
            if isinstance(out[wi], OptError):
                raise out[wi]
            if isinstance(out[wi], Exception):
                raise OptError("CORE_FAILED", "core", str(out[wi]))
            for i, r in zip(assign[wi], out[wi]):
                res[i] = r
                self.core_ns += r["core_ns"]
        self.evals += len(cands)
        return res


def prove_and_repair(pb, pool, wl, max_rounds=24):
    """Range mode: prove the absence of overflow on the final candidate; for every quantisation point
    the proof cannot fit, widen the decision variable whose declared type holds it (fraction length kept, so the
    values do not change unless they overflowed) and prove again. Returns (wl, record, proof, repairs)."""
    import fxprove as P
    d0 = pb.d0
    wl = list(wl)
    idx = {v: i for i, v in enumerate(pb.vars)}
    repairs = []
    for rnd in range(max_rounds + 1):
        spec = _with_vec_key({"blocks": pb.blocks(wl), "lines": pb.spec["lines"]}, pb.spec)
        proof = P.prove(spec, pb.hw, pb.in_types, pb.range_ints, pb.initial_state,
                        modular=(getattr(pb, "req", None) or {}).get("_modular"))
        if proof["status"] not in ("unknown", "violated") or not proof.get("points_not_proven") or rnd == max_rounds:
            break
        changed = set()
        # compare against the effective word lengths (block_wordlengths caps and pre-add ties applied, as
        # pb.blocks does); the raw search word (16 under a [58, 62] cap) never matched the proven point's type, so the
        # point was skipped and the plan kept a 58-bit accumulator that wraps on an in-range input
        wl_eff = list(pb.repair(wl)) if hasattr(pb, "repair") else list(wl)
        caps = getattr(pb, "_caps", None) or {}
        for pt in proof["points_not_proven"]:
            slot = pt["slot"]
            b = slot.split("/")[0]
            if pt["role"] == "accumulator":
                key = "AccumDataTypeStr"
            elif "/" in slot and d0.btype[b] == "DiscreteFir" and slot.split("/")[1].startswith("p"):
                key = "ProductDataTypeStr"
            else:
                key = "OutDataTypeStr"
            i = idx.get((b, key))
            if i is None and key == "OutDataTypeStr":
                i = idx.get((pb._root(b), key))
            if i is None or i in changed or pb.vtype[i][0] != "sig":
                continue
            s, w, f = pb.var_type(i, wl_eff[i])
            if (pt["type_min"], pt["type_max"]) != P.trange(s, w):
                continue  # the point is not stored in this variable's type
            lo, hi = pt["min"], pt["max"]
            s2 = 1 if lo < 0 else s
            w2 = w
            while not (P.trange(s2, w2)[0] <= lo and hi <= P.trange(s2, w2)[1]):
                w2 += 1
            up = [x for x in pb.wls if x >= w2 and (i not in caps or x <= caps[i][1])]
            if not up:  # never leave the requested word-length set (nor a block_wordlengths cap)
                if not any(r.get("variable") == "%s.%s" % (b, key) and r.get("to") is None for r in repairs):
                        repairs.append({"variable": "%s.%s" % (b, key), "from": "fixdt(%d,%d,%d)" % (s, w, f),
                                    "to": None, "point": pt["where"], "refused": "needs %d bits, above the largest "
                                    "allowed word length %d" % (w2, min(pb.wls[-1], caps[i][1]) if i in caps
                                                                else pb.wls[-1])})
                continue
            w2 = up[0]
            pb.vtype[i] = ("sig", s2, w2 - f)
            repairs.append({"variable": "%s.%s" % (b, key), "from": "fixdt(%d,%d,%d)" % (s, w, f),
                            "to": "fixdt(%d,%d,%d)" % (s2, w2, f), "point": pt["where"]})
            wl[i] = w2
            changed.add(i)
        if not changed:
            break
    assert all(wl[i] in pb.wls for i in range(len(wl)) if pb.vtype[i][0] == "sig"), "word length outside the set"
    rec = pool.evaluate(None, [wl])[0]
    return wl, rec, proof, repairs


class MultiPool:
    """One GraphPool per input set . A candidate's error is its worst (tolerance-normalised) error over all
    sets; ranges are the hull over all sets, widened by the proven bounds in range mode."""

    def __init__(self, pb, nworkers):
        self.pb = pb
        k = len(pb.scenarios)
        per = max(1, nworkers // k)
        # every input set keeps its own worker processes (they hold its inputs and reference), so
        # there are max(workers, input sets) processes; evaluate() runs the pools in batches so that at most
        # `workers` of them compute at once (before: one pool at a time -- fewer busy than asked when sets <= workers)
        self.batch = max(1, nworkers // per)
        self.pools = []
        try:
            for i in range(k):
                pb.use_scenario(i)
                self.pools.append(GraphPool(pb, per, apply_ranges=False))
        finally:
            pb.use_scenario(0)
        q = {}
        for pool in self.pools:
            for name, (lo, hi) in pool.q.items():
                if name in q:
                    a, b = q[name]
                    q[name] = (min(a, lo) if lo == lo and a == a else math.nan,
                               max(b, hi) if hi == hi and b == b else math.nan)
                else:
                    q[name] = (lo, hi)
        self.simulated_ranges = dict(q)
        bounds = getattr(pb, "bounds", None)
        if bounds:  # range mode: the proven worst case decides the integer bits
            for name, (lo, hi) in bounds.items():
                a, b = q.get(name, (lo, hi))
                if a == a and b == b:
                    q[name] = (min(a, lo), max(b, hi))
        pb.set_ranges(q)
        self.n = sum(p.n for p in self.pools)
        self.evals = 0

    @property
    def procs(self):
        return [p for pool in self.pools for p in pool.procs]

    @property
    def core_ns(self):
        return sum(pool.core_ns for pool in self.pools)

    @property
    def busy_at_once(self):
        return max(sum(p.n for p in self.pools[i:i + self.batch]) for i in range(0, len(self.pools), self.batch))

    def evaluate(self, base, cands):
        per_pool = [None] * len(self.pools)
        errs = []

        lines = [self.pools[0].line(c) for c in cands]   # the same for every input set (built once, one thread)

        def one(i):
            try:
                per_pool[i] = self.pools[i].evaluate(base, cands, lines)
            except BaseException as e:   # noqa: BLE001 -- re-raised in the caller's thread
                errs.append(e)
        for i0 in range(0, len(self.pools), self.batch):
            idx = list(range(i0, min(i0 + self.batch, len(self.pools))))
            if len(idx) == 1:
                one(idx[0])
            else:
                ths = [threading.Thread(target=one, args=(i,)) for i in idx]
                for th in ths:
                    th.start()
                for th in ths:
                    th.join()
            if errs:
                raise errs[0]
        out = []
        for recs in zip(*per_pool):
            worst = max(recs, key=lambda r: r["err"])
            # every set's error, so INFEASIBLE can say which input set binds
            out.append(dict(worst, core_ns=sum(r["core_ns"] for r in recs), per_set=[r["err"] for r in recs]))
        self.evals += len(cands)
        return out

    def close(self):
        for pool in self.pools:
            pool.close()


def uniform(pb, F, pw):
    """All signals share the LSB 2^-F (word length = range bits + F, clamped to the allowed set),
    all parameters word length pw."""
    wl = []
    for i, vt in enumerate(pb.vtype):
        if vt[0] in ("param", "coefs"):
            wl.append(pw)
            continue
        if vt[0] == "frac":   # fraction in [0, 1]: F fraction bits (+1 integer bit when 1 must be representable)
            need = F + (0 if vt[1] else 1)
            wl.append(next((x for x in pb.wls if x >= need), pb.wls[-1]))
            continue
        need = vt[2] + F
        w = next((x for x in pb.wls if x >= need), pb.wls[-1])
        wl.append(w)
    return wl


MULTISTART = int(os.environ.get("FXOPT_MULTISTART", "2"))


# Warm re-optimisation starts from the preceding solution.
# It can reduce search time at the expense of hardware cost.
# Cold re-optimisation remains the default.

WARM_REOPT = os.environ.get("FXOPT_WARM_REOPT", "0") != "0"
WARM_REOPT_ENV = WARM_REOPT   # the process setting; a request's "reoptimisation" sets WARM_REOPT for that request
# Warm re-optimisation can also improve both time and hardware cost.
# Select it per request with "reoptimisation": "warm".

WARM_WIDEN_STEPS = 4


def warm_start(pb, pool, log, warm, deadline, max_rounds, progress=None):
    """Descent from a previous solution `warm` (a word-length vector of this problem). The new problem differs in
    its input sets or tolerance, so the previous solution may now break the tolerance: it is widened uniformly
    (every variable 1, 2, ... steps up the allowed set, all in one parallel batch) and the cheapest feasible
    widening is refined. None when no widening up to WARM_WIDEN_STEPS steps is feasible (the caller then runs the
    cold search)."""
    if warm is None or len(warm) != pb.nvar or any(w not in pb.wls for w in warm):
        return None
    t0 = time.monotonic()
    idx = {w: i for i, w in enumerate(pb.wls)}
    cands = [[pb.wls[min(len(pb.wls) - 1, idx[w] + k)] for w in warm] for k in range(WARM_WIDEN_STEPS + 1)]
    res = pool.evaluate(None, cands)
    for c, r in zip(cands, res):
        log.add("warm", pb, c, r)
    pick = next((k for k, r in enumerate(res) if r["err"] <= pb.tol), None)
    if pick is None:
        return None
    stats = {"rounds": 0, "phase_s": {"warm_start": round(time.monotonic() - t0, 3)},
             "after_uniform": {"hwbits": pb.hwbits(cands[pick]), "max_abs_diff": res[pick]["err"]},
             "warm_start": {"widened_steps": pick, "from_hwbits": pb.hwbits(list(warm))}}
    out = L.refine(pb, pool, log, (cands[pick], res[pick]), deadline, max_rounds, stats, progress)
    return out


def optimise(pb, pool, log, deadline, max_rounds=200, progress=None, warm=None):
    if warm is not None and WARM_REOPT:
        res = warm_start(pb, pool, log, warm, deadline, max_rounds, progress)
        if res is not None:
            return res
    stats = {"rounds": 0, "phase_s": {}}
    t0 = time.monotonic()
    top = [pb.wls[-1]] * pb.nvar
    # 1) common LSB with the widest parameters, 2) narrowest common parameter word length,
    # 3) re-sweep the LSB with those parameters; every sweep is one parallel bit-exact batch
    Fs = list(range(-8, 64))
    cands = [uniform(pb, F, pb.wls[-1]) for F in Fs]
    res = pool.evaluate(None, cands)
    feas = [(pb.cost(c), F, c, r) for F, c, r in zip(Fs, cands, res) if r["err"] <= pb.tol]
    for c, r in zip(cands, res):
        log.add("uniform", pb, c, r)
    if not feas and pb.pins:
        # the parameters at the design's own types (reference 'design') -- the widest parameters can
        # be further from the yardstick than the design's narrow ones
        cands = []
        for F in Fs:
            c = uniform(pb, F, pb.wls[-1])
            for i, pt in pb.pins.items():
                c[i] = pt[1]
            cands.append(c)
        res = pool.evaluate(None, cands)
        for c, r in zip(cands, res):
            log.add("uniform", pb, c, r)
        feas = [(pb.cost(c), F, c, r) for F, c, r in zip(Fs, cands, res) if r["err"] <= pb.tol]
    if not feas:
        r = pool.evaluate(None, [top])[0]
        log.add("uniform", pb, top, r)
        if r["err"] > pb.tol:
            rd = getattr(pb.design(top), "runtime_diag", None) if math.isinf(r["err"]) else None
            if rd:
                # an input set (or a range stress sequence) drives a strict Multiport Switch out of range
                raise OptError("SIM_DIAGNOSTIC", "design.blocks." + rd[0], "an input set or a sequence inside the "
                               "input ranges drives the control of Multiport Switch %s out of range of its data ports, "
                               "where Simulink stops with an error (DiagnosticForDefault 'Error' -- the default when "
                               "left out -- or a single data port): restrict the control's inputs / input_ranges, "
                               "or set DiagnosticForDefault to 'None'" % ", ".join(rd[:4]))
            # the smallest error any tried candidate reached (not only the widest one's), and a
            # design parameter whose own word length is outside wordlengths named
            best_err = min([r["err"]] + [x["err"] for x in res if math.isfinite(x["err"])])
            off = []
            if pb.reference == "design":
                for i, (b_, key) in enumerate(pb.vars):
                    t0 = G.fc._named_type(str(pb.d0.params[b_].get(key, "")))
                    if isinstance(t0, tuple) and t0[1] not in pb.wls:
                        off.append("%s.%s %s" % (b_, key, pb.d0.params[b_][key]))
            # which input set binds at the widest plan (the representative or a stress set -- with a
            # stress set only, stress_tolerance_scale can help), and how far each one is from its tolerance
            sets_txt = ""
            ps = r.get("per_set")
            scen = getattr(pb, "scenarios", None) or []
            if ps and len(ps) == len(scen):
                rows = sorted(((e / pb.tol if pb.tol else e, sc["label"]) for e, sc in zip(ps, scen)), reverse=True)
                over = [lab for q, lab in rows if q > 1]
                sets_txt = ("; at the widest plan, error / tolerance per input set: %s; over the tolerance: %s%s"
                            % (", ".join("%s %.3g" % (lab, q) for q, lab in rows), ", ".join(over) or "none",
                               " -- only stress sets (range: ...) miss it: stress_tolerance_scale relaxes those, or "
                               "narrow input_ranges" if over and all(lab.startswith("range:") for lab in over) else
                               " -- the given input sets themselves miss it: wider word lengths, a type the optimiser does not choose (a "
                               "filter's internal type, a fixed block), a signal that grows beyond the widest word (an "
                               "integrator), or the tolerance must change; fxgraph with diagnostics on the widest plan "
                               "names the blocks that wrap or saturate" if over else ""))
            # the cap that binds first -- the widest plan used the largest allowed word length
            cap_txt = ("; the widest plan gives every chosen type %d bits, the largest in wordlengths %s: allowing "
                       "wider word lengths is the first thing to try" % (pb.wls[-1], json.dumps(list(pb.wls))))
            raise OptError("INFEASIBLE", "error", "no allowed word lengths reach the tolerance: the best tried gives "
                           "max_abs_diff %.6g > %.6g (the widest %.6g)%s%s%s%s"
                           % (best_err, pb.tol, r["err"], cap_txt, sets_txt,
                              " (with the block_wordlengths caps %s: widen them or the tolerance)"
                              % json.dumps(pb.block_wordlengths) if getattr(pb, "block_wordlengths", None) else "",
                              "; the design's own types %s have word lengths outside wordlengths, so the design itself "
                              "is not a candidate -- add them" % ", ".join(off[:4]) if off else ""))
        best = (top, r)
        F0 = None
    else:
        _, F0, c, r = min(feas, key=lambda t: (t[0], t[1]))
        best = (c, r)
    if F0 is not None:
        cands = [uniform(pb, F0, w) for w in pb.wls]
        res = pool.evaluate(None, cands)
        for c, r in zip(cands, res):
            log.add("uniform", pb, c, r)
            if r["err"] <= pb.tol and pb.cost(c) < pb.cost(best[0]):
                best = (c, r)
        pw = next((c[i] for i, vt in enumerate(pb.vtype) if vt[0] in ("param", "coefs")), None) if best else None
        if pw is not None:
            cands = [uniform(pb, F, pw) for F in Fs]
            res = pool.evaluate(None, cands)
            for c, r in zip(cands, res):
                log.add("uniform", pb, c, r)
                if r["err"] <= pb.tol and pb.cost(c) < pb.cost(best[0]):
                    best = (c, r)
    stats["phase_s"]["uniform"] = round(time.monotonic() - t0, 3)
    stats["after_uniform"] = {"hwbits": pb.hwbits(best[0]), "max_abs_diff": best[1]["err"]}
    res = L.refine(pb, pool, log, best, deadline, max_rounds, stats, progress)
    # multi-start (the descent from the cheapest uniform start ends in a local optimum):
    # also descend from the uniform starts with a 1 and 2 bits finer common LSB; keep the cheapest result
    if F0 is not None and MULTISTART > 0 and not res["timed_out"]:
        pw0 = next((best[0][i] for i, vt in enumerate(pb.vtype) if vt[0] in ("param", "coefs")), pb.wls[-1])
        for dF in range(1, MULTISTART + 1):
            if time.monotonic() > deadline:
                res["timed_out"] = True   # skipped starts are a budget cut, not a deterministic end
                break
            c = uniform(pb, F0 + dF, pw0)
            r = pool.evaluate(None, [c])[0]
            log.add("multistart", pb, c, r)
            if r["err"] > pb.tol:
                continue
            st2 = {"rounds": 0, "phase_s": {}}
            alt = L.refine(pb, pool, log, (c, r), deadline, max_rounds, st2, progress)
            stats["rounds"] += st2["rounds"]
            if alt["timed_out"]:
                res["timed_out"] = True
            if L._cost(pb, alt["wl"]) < L._cost(pb, res["wl"]):
                alt["stats"] = res["stats"]
                res = dict(res, wl=alt["wl"], rec=alt["rec"])
    if MSB_TRIM and getattr(pb, "range_spec", None) is None and not res.get("timed_out"):
        res = trim_msb(pb, pool, log, res, deadline, max_rounds, stats, progress)
    return res


MSB_TRIM = os.environ.get("FXOPT_MSB_TRIM", "1") == "1"


def trim_msb(pb, pool, log, res, deadline, max_rounds, stats, progress=None):
    """Try removing integer bits when overflow cancellation keeps every observed
    output within tolerance. Refine the plan after trimming. This search applies
    when there is no input-range proof to preserve."""
    wl, rec = list(res["wl"]), res["rec"]
    # a modular region's integer width is where its proof starts; with no input ranges the trim may still go below
    # it on the judged sets (fxopt then reports that region as sampled, not proven)
    sig = [i for i, vt in enumerate(pb.vtype) if vt[0] == "sig"]
    res = dict(res, _vt0={i: pb.vtype[i] for i in sig})
    order = sorted(sig, key=lambda i: -pb.weight[i])
    changed_any = False
    for _round in range(64):
        changed = False
        for i in order:
            if time.monotonic() > deadline:
                break
            _, s, ib = pb.vtype[i]
            if ib <= (1 if s else 0) - 8:
                continue
            old_vt, old_w = pb.vtype[i], wl[i]
            pb.vtype[i] = ("sig", s, ib - 1)
            w2 = old_w - 1 if (old_w - 1) in pb.wls else old_w   # the same LSB, one integer bit less
            cand = list(wl)
            cand[i] = w2
            pb._repair_cache = {}
            r = pool.evaluate(None, [cand])[0]
            log.add("msb-trim", pb, cand, r)
            if r["err"] <= pb.tol and L._cost(pb, cand) <= L._cost(pb, wl):
                wl, rec, changed, changed_any = cand, r, True, True
            else:
                pb.vtype[i] = old_vt
                pb._repair_cache = {}
        if not changed:
            break
    vt0 = res.pop("_vt0")
    if not changed_any:
        return res
    st = {"rounds": 0, "phase_s": {}}
    alt = L.refine(pb, pool, log, (wl, rec), deadline, max_rounds, st, progress)
    stats["rounds"] = stats.get("rounds", 0) + st["rounds"]
    trimmed = sorted("%s.%s" % pb.vars[i] for i in sig if pb.vtype[i] != vt0[i])
    if L._cost(pb, alt["wl"]) < L._cost(pb, res["wl"]):
        stats["msb_trimmed"] = trimmed
        return dict(res, wl=alt["wl"], rec=alt["rec"], timed_out=res.get("timed_out") or alt.get("timed_out"))
    for i, vt in vt0.items():   # no gain: the plan keeps its own types (they are read from vtype later)
        pb.vtype[i] = vt
    pb._repair_cache = {}
    return res


# Resource estimates depend on the design.
# The margin is a planning heuristic for estimated LUT, FF and gate savings.
HW_MARGIN = 0.30


def optimise_hw(pb, pool, log, t_start, deadline, max_rounds=200):
    """cost models: their landscape is rough (DSP counts and partial products jump with single bits), so a
    plain descent can end above the hwbits solution even under its own metric. Search under hwbits first, then
    descend from that solution under the hardware model, and also search under the model from scratch; return
    the cheapest feasible result under the model (never worse than the hwbits solution, as predicted)."""
    model = pb.cost_model
    mid = t_start + 0.4 * (deadline - t_start)
    pb.cost_model, keep_w = None, pb.cost_w
    pb.cost_w = (1.0, 0.0)
    try:
        base = optimise(pb, pool, log, mid, max_rounds)
    finally:
        pb.cost_model, pb.cost_w = model, keep_w
    st = {"rounds": 0, "phase_s": {}}
    alt = L.refine(pb, pool, log, (base["wl"], base["rec"]), t_start + 0.7 * (deadline - t_start), max_rounds, st)
    fresh = optimise(pb, pool, log, deadline, max_rounds)
    tried = [("hwbits search", base), ("descent from the hwbits solution", dict(base, wl=alt["wl"], rec=alt["rec"])),
             ("search under the model", fresh)]
    # the model's LUT / gate figures are rough: leave the hwbits solution only for a
    # predicted saving above HW_MARGIN of the model cost (any DSP48E1 saved counts in fpga_xc7)
    c0 = pb.cost(base["wl"])
    name, best = "hwbits search", base
    hb_cap = pb.hwbits(base["wl"]) * (1 + pb.hwbits_slack)
    for n, r in tried[1:]:
        if pb.hwbits(r["wl"]) > hb_cap:
            continue
        c = pb.cost(r["wl"])
        cb = pb.cost(best["wl"])
        if model == "fpga_xc7" and c // 1e6 < cb // 1e6:
            name, best = n, r
        elif (model != "fpga_xc7" or c // 1e6 == cb // 1e6) and c < cb - HW_MARGIN * (cb % 1e6 if model == "fpga_xc7"
                                                                                          else cb):
            name, best = n, r
    del c0
    best = dict(best)
    best["timed_out"] = base["timed_out"] or alt["timed_out"] or fresh["timed_out"]
    best["stats"] = dict(base["stats"], rounds=base["stats"]["rounds"] + st["rounds"] + fresh["stats"]["rounds"])
    best["stats"]["hw_search"] = {n: pb.cost(r["wl"]) for n, r in tried}
    best["stats"]["hw_search_hwbits"] = {n: pb.hwbits(r["wl"]) for n, r in tried}
    best["stats"]["hw_search_hwbits_cap"] = hb_cap
    best["stats"]["hw_search_kept"] = name
    return best


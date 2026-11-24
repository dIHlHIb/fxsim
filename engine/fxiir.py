"""Discrete Filter / Discrete Transfer Fcn, lowered to engine blocks.

What is answered (fitted to the recordings, every recorded step equal):
- Discrete Filter: FilterStructure 'Direct form II' (the library default), 'Direct form I', 'Direct form II
  transposed'; Discrete Transfer Fcn: 'Direct form II' (its library default; the numerator is a polynomial in z, so a
  shorter numerator is aligned to the right -- recordings).
- a0 = 1 (the leading denominator coefficient after quantisation); a0 != 1 is not answered (recordings differs
  by one LSB from an exact normalisation).
- the internal types (StateDataTypeStr, NumCoefDataTypeStr, DenCoefDataTypeStr, NumProductDataTypeStr,
  DenProductDataTypeStr, NumAccumDataTypeStr, DenAccumDataTypeStr, MultiplicandDataTypeStr, OutDataTypeStr) explicit
  fixdt or left to the library (typed(), below: resolved from the input type when compile() knows it);
- RndMeth Floor / Nearest / Zero, SaturateOnIntegerOverflow on / off; one signed fixed-point scalar input;
  InitialStates a scalar (Direct form I: the numerator states; InitialDenominatorStates 0; Direct form II and II
  transposed: every state).
Arithmetic: every product is cast to its product type, sums are formed in the accumulator type, the state
is the denominator sum cast to the state type (Direct form II), the output the numerator sum cast to the output type.
- Direct form II: w = DenAcc(x - sum a_k w[n-k]) -> State; y = NumAcc(sum b_k w[n-k]) -> Out.
- Direct form I: v = NumAcc(sum b_k x[n-k]) -> DenAcc; y = Out(DenAcc(v - sum a_k y[n-k])); x history in the input
  type (initial: InitialStates), y history in the output type (initial: InitialDenominatorStates).
- Direct form II transposed: y = Out(NumAcc(b0 x + s1)); s_k = State(DenAcc(NumProd(b_k x) + s_{k+1} - DenProd(a_k y))).
'Direct form I transposed' is refused: 3 of its 7 recordings (narrow types, Floor) are not reproduced yet."""
import json
import math
import os
import re
from fractions import Fraction

TYPES = ("DiscreteFilter", "DiscreteTransferFcn")
TYPE_KEYS = ("StateDataTypeStr", "NumCoefDataTypeStr", "DenCoefDataTypeStr", "NumProductDataTypeStr",
             "DenProductDataTypeStr", "NumAccumDataTypeStr", "DenAccumDataTypeStr", "MultiplicandDataTypeStr",
             "OutDataTypeStr")
PARAMS = ("Numerator", "Denominator", "InitialStates", "InitialDenominatorStates", "FilterStructure", "a0EqualsOne",
          "RndMeth", "SaturateOnIntegerOverflow", "SampleTime", "NumeratorSource", "DenominatorSource",
          "InitialStatesSource", "InputProcessing", "ExternalReset", "LockScale", "NumCoefMin", "NumCoefMax",
          "DenCoefMin", "DenCoefMax", "OutMin", "OutMax") + TYPE_KEYS
# the library values
DEFAULTS = {"Numerator": "[1]", "Denominator": "[1 0.5]", "InitialStates": "0", "InitialDenominatorStates": "0",
            "FilterStructure": "Direct form II", "a0EqualsOne": "off", "RndMeth": "Floor",
            "SaturateOnIntegerOverflow": "off", "SampleTime": "-1", "NumeratorSource": "Dialog",
            "DenominatorSource": "Dialog", "InitialStatesSource": "Dialog",
            "InputProcessing": "Elements as channels (sample based)", "ExternalReset": "None", "LockScale": "off",
            "StateDataTypeStr": "Inherit: Same as input", "MultiplicandDataTypeStr": "Inherit: Same as input",
            "NumCoefDataTypeStr": "Inherit: Inherit via internal rule",
            "DenCoefDataTypeStr": "Inherit: Inherit via internal rule",
            "NumProductDataTypeStr": "Inherit: Inherit via internal rule",
            "DenProductDataTypeStr": "Inherit: Inherit via internal rule",
            "NumAccumDataTypeStr": "Inherit: Inherit via internal rule",
            "DenAccumDataTypeStr": "Inherit: Inherit via internal rule",
            "OutDataTypeStr": "Inherit: Inherit via internal rule", "NumCoefMin": "[]", "NumCoefMax": "[]",
            "DenCoefMin": "[]", "DenCoefMax": "[]", "OutMin": "[]", "OutMax": "[]"}
INTERNAL = "\x1d"
_FIX = re.compile(r"^fixdt\(\s*([01])\s*,\s*(\d+)\s*,\s*(-?\d+)\s*\)$")


def has(spec):
    return any(isinstance(b, dict) and b.get("type") in TYPES + ("BiquadFilter",)
               for b in (spec.get("blocks") or {}).values())


def _refuse(where, msg, code="UNSUPPORTED"):
    import fxgraph_lib as G
    raise G.GraphError(code, where, msg)


def _vals(text, where):
    t = str(text).strip()
    if t.startswith("[") and t.endswith("]"):
        t = t[1:-1]
    parts = [x for x in re.split(r"[\s,]+", t.strip()) if x]
    out = []
    for x in parts:
        try:
            out.append(Fraction(x))
        except ValueError:
            _refuse(where, "a numeric literal list is required, got %r" % str(text)[:60], "BAD_PARAM")
    if not out:
        _refuse(where, "at least one coefficient is required", "BAD_PARAM")
    return out


def _quant(v, t):
    """A parameter value in its type (Simulink: nearest, ties up; saturated -- the overflow rule is the engine's)."""
    s, w, f = t
    q = v * Fraction(2) ** f
    n = (q + Fraction(1, 2)).numerator // (q + Fraction(1, 2)).denominator
    lo, hi = (-(1 << (w - 1)), (1 << (w - 1)) - 1) if s else (0, (1 << w) - 1)
    return Fraction(min(max(n, lo), hi)) / Fraction(2) ** f


def _lit(v):
    return repr(float(v)) if v.denominator & (v.denominator - 1) == 0 else str(v)


def coef_note(notes, loc, written, used, t):
    """recordings (parameter_quantization): a coefficient used other than as written -- rounded into its coefficient type
    (inherited: the type the library rule chose; or written), or saturated to its bound -- is reported under the
    user's parameter location in fxgraph_lib._param_note's form (the lowered Gains carry the value already used)."""
    import fxgraph_lib as G
    written, used = Fraction(written), Fraction(used)
    if used == written:
        return
    s_, w_, f_ = t
    lo, hi = (-(1 << (w_ - 1)), (1 << (w_ - 1)) - 1) if s_ else (0, (1 << w_) - 1)
    x = written * Fraction(2) ** f_
    notes[loc] = {"written": G._fmt_num(written), "type": G._tstr(tuple(t)), "used": G._fmt_num(used),
                  "why": "saturated to the type's bound" if x < lo or x > hi else "rounded to the type's precision"}


def elem_loc(where, key, j, n):
    """design.blocks.<name>.<key>[j] (1-based) for element j of a list parameter of n elements; a scalar unindexed."""
    return "%s.%s" % (where, key) if n == 1 else "%s.%s[%d]" % (where, key, j + 1)


def _with_notes(spec, notes):
    import parameter_bounds
    return dict(spec, **{parameter_bounds.NOTES_KEY: notes}) if notes else spec


def rewrite(spec, info):
    spec = bq_rewrite(spec, info)
    return _rewrite(spec, info)


def _rewrite(spec, info):
    """spec with each Discrete Filter / Discrete Transfer Fcn replaced by Gain / Sum / Unit Delay / Data Type
    Conversion blocks named <block>\\x1d..., the output keeping the block's own name (a Data Type Conversion)."""
    import fxgraph_lib as G
    import parameter_bounds
    blocks = dict(spec["blocks"])
    lines = [dict(ln) for ln in spec.get("lines", [])]
    pnotes = dict(spec.get(parameter_bounds.NOTES_KEY) or {})
    for n in [n for n, b in blocks.items() if isinstance(b, dict) and b.get("type") in TYPES]:
        b = blocks[n]
        t = b["type"]
        where = "design.blocks." + n
        unknown = sorted(set(b) - {"type", "_Pat", "_InCls"} - set(PARAMS))
        if unknown:
            _refuse(where + "." + unknown[0], "unknown parameter %s for %s" % (unknown[0], t), "BAD_PARAM")
        p = dict(DEFAULTS, **{k: str(v) for k, v in b.items() if k != "type"})
        for k, v in (("NumeratorSource", "Dialog"), ("DenominatorSource", "Dialog"), ("InitialStatesSource", "Dialog"),
                     ("InputProcessing", "Elements as channels (sample based)"), ("ExternalReset", "None"),
                     ("SampleTime", "-1"), ("a0EqualsOne", "off"), ("LockScale", "off")):
            if p[k].strip() != v:
                _refuse(where + "." + k, "%s %s %r: only %r was recorded" % (t, k, p[k], v))
        for k in ("NumCoefMin", "NumCoefMax", "DenCoefMin", "DenCoefMax", "OutMin", "OutMax"):
            if p[k].strip() not in ("[]", ""):
                _refuse(where + "." + k, "%s %s: design minimum / maximum values are not supported" % (t, k))
        ty = {}
        for k in TYPE_KEYS:
            m = _FIX.match(p[k].strip())
            if not m:
                _refuse(where + "." + k, "%s %s %r: an explicit fixdt or the library's %r is answered"
                    % (t, k, p[k], DEFAULTS[k]))
            ty[k] = (int(m.group(1)), int(m.group(2)), int(m.group(3)))
            if k == "StateDataTypeStr" and ty[k] == XT_UNSIGNED_STATE and \
                    p["FilterStructure"].strip() == "Direct form II":   #
                _refuse(where + "." + k, ('%s Direct form II with the unsigned state %s: Simulink '
                    'rejects it (Simulink:Parameters:InvParamSetting / '
                    'UdtEvalsToUnsupportedFixdtSignMode); write a signed state type') % (t, p[k].strip()), "PORT_TYPE_INVALID")
            if not ty[k][0] or not 2 <= ty[k][1] <= 64:
                _refuse(where + "." + k, "%s %s %r: signed types of 2 .. 64 bits were recorded" % (t, k, p[k]))
        rm = p["RndMeth"].strip()
        # (the dtw class: its recorded cells carry their own RndMeth, Convergent among them -- recordings,
        # checked against engine/dtf_wide_domain.json in _type_one; the dfw class likewise, against
        # engine/dfi_wide_domain.json, only where the numerator sum is not rounded -- fxiir.dfw_agree; the xt class
        # against its recorded cells in engine/iir_xt_domain.json)
        if rm not in ("Floor", "Nearest", "Zero") and not (rm == "Convergent" and b.get("_InCls") in ("dtw", "dfw", "xt")):
            _refuse(where + ".RndMeth", "%s RndMeth %r: Floor, Nearest and Zero were recorded" % (t, rm))
        sat = p["SaturateOnIntegerOverflow"].strip()
        if sat not in ("on", "off"):
            _refuse(where + ".SaturateOnIntegerOverflow", "on or off", "BAD_PARAM")
        st = p["FilterStructure"].strip()
        allowed = ("Direct form II",) if t == "DiscreteTransferFcn" else \
            ("Direct form II", "Direct form I", "Direct form II transposed")
        if st not in allowed:
            _refuse(where + ".FilterStructure", "%s FilterStructure %r: answered for %s ("
                    "'Direct form I transposed' is not reproduced yet)" % (t, st, ", ".join(allowed)))
        pat = b.get("_Pat") or next((k for k, v in REC_EXPLICIT.items() if all(ty[k_] == v[k_] for k_ in _K)), None)
        if pat is None:
            _refuse(where, ('%s with the explicit internal types %s: only the three recorded type sets '
                'were answered (for example, State fixdt(1,24,16), coefficients fixdt(1,16,14), '
                'products fixdt(1,32,26), accumulators fixdt(1,40,26), multiplicand fixdt(1,24,16), '
                'output fixdt(1,16,10); another recording used State fixdt(1,24,18), '
                'coefficients fixdt(1,16,14) / fixdt(1,16,13), products and accumulators '
                'fixdt(1,32,24), output fixdt(1,16,12)); record a filter with these types to open it') % (t, ", ".join("%s %s" % (k, p[k]) for k in _K)))
        cls_all = (b.get("_InCls") or ("dense" if pat == "dense" else "old")).split("+")
        cls_ok = [c for c in cls_all if (t, st, pat, c) in REC_PATTERNS]
        cls = cls_ok[0] if cls_ok else cls_all[0]
        if not cls_ok:
            _refuse(where, "%s %s with %s on a %s input: not recorded"
                    "; record this combination to open it" % (t, st, _pat_text(pat), "small (8 / 10 / 12-bit dense map)"
                                                              if cls == "dense" else "12 .. 16-bit"))
        if pat in ("dense", "inherited+prod") or cls == "dense":
            # the dense map ran the library rounding / overflow only
            if rm != "Floor" or sat != "off":
                _refuse(where + ".RndMeth", "%s with %s: recorded with RndMeth Floor and SaturateOnIntegerOverflow off "
                        "only; record this rounding to open it" % (t, _pat_text(pat)))
        # recordings-a coefficient outside its coefficient type stops Simulink (fxpParameterOverflow, default
        # diagnostics) -- the common parameter-overflow rule, as for a Gain parameter
        for key, tk in (("Numerator", "NumCoefDataTypeStr"), ("Denominator", "DenCoefDataTypeStr")):
            for v in _vals(p[key], where + "." + key):
                G._param_overflow(float(v), ty[tk], where + "." + key, "%s %s: %s in %s" % (t, n, key, tk),
                                  mode=G.PARAM_OVERFLOW_DEFAULT)
        for key in ("Numerator", "Denominator"):
            if len(_vals(p[key], where + "." + key)) > REC_MAX_COEFS:
                _refuse(where + "." + key, ('%s with %d coefficients in %s: up to %d were '
                    'recorded; record a filter of this order to open it') % (t, len(_vals(p[key], where + "." + key)),
                                                                          key, REC_MAX_COEFS))
        num = [_quant(v, ty["NumCoefDataTypeStr"]) for v in _vals(p["Numerator"], where + ".Numerator")]
        den = [_quant(v, ty["DenCoefDataTypeStr"]) for v in _vals(p["Denominator"], where + ".Denominator")]
        for key, tk, used in (("Numerator", "NumCoefDataTypeStr", num), ("Denominator", "DenCoefDataTypeStr", den)):
            raw = _vals(p[key], where + "." + key)
            for j, (v, q) in enumerate(zip(raw, used)):
                coef_note(pnotes, elem_loc(where, key, j, len(raw)), v, q, ty[tk])
        if t == "DiscreteTransferFcn":
            if len(num) > len(den):
                _refuse(where + ".Numerator", "the numerator's order must not exceed the denominator's (Simulink: "
                        "denOrderGTEnumOrder)", "BAD_PARAM")
            num = [Fraction(0)] * (len(den) - len(num)) + num
        if den[0] not in (1, 2):
            # recordings (every a0 = 2 case equal): the coefficients are
            # kept as given and the feedback sum is divided by a0 (norm() below); only a0 = 2 was recorded
            _refuse(where + ".Denominator", "%s with a leading denominator coefficient %s (after quantisation to "
                    "DenCoefDataTypeStr): a0 = 1 and a0 = 2 were recorded"
                    "; record this a0 to open it, or divide the coefficients by a0" % (t, _lit(den[0])))
        if len(_vals(p["Numerator"], where + ".Numerator")) == 1 and len(_vals(p["Denominator"], where +
                                                                               ".Denominator")) == 1:
            # a filter without states (a gain) was not recorded
            _refuse(where, "%s without delays (one numerator and one denominator coefficient): not recorded; record "
                    "one to open it (or use a Gain)" % t)
        ic = _vals(p["InitialStates"], where + ".InitialStates")
        cls_ic = [c for c in cls_ok if (t, st, pat, c) in REC_IC_NONZERO]
        if ic and len(ic) == 1 and ic[0] != 0 and cls_ic:
            cls = cls_ic[0]
        elif ic and len(ic) == 1 and ic[0] != 0:
            # a non-zero initial state outside the recorded combinations
            _refuse(where + ".InitialStates", "%s %s with InitialStates %s and %s: a non-zero initial state was "
                    "recorded only in other combinations; record this one to open it"
                    % (t, st, _lit(ic[0]), _pat_text(pat)))
        dic = _vals(p["InitialDenominatorStates"], where + ".InitialDenominatorStates")
        if len(ic) != 1 or len(dic) != 1 or dic[0] != 0:
            _refuse(where + ".InitialStates", "%s initial states: a scalar InitialStates and "
                    "InitialDenominatorStates 0 were recorded" % t)
        src = [ln for ln in lines if ln.get("dst") == n]
        if len(src) != 1 or int(src[0].get("dport", 1)) != 1:
            _refuse(where, "%s takes exactly one input" % t, "BAD_CONNECTION")
        x = (src[0]["src"], int(src[0].get("sport", 1)))
        info.setdefault("iir_inputs", []).append((n, x))
        info.setdefault("iir_class", {})[n] = cls
        info.setdefault("iir_type", {})[n] = (t, st, pat)
        R = {"RndMeth": rm, "SaturateOnIntegerOverflow": sat}
        fx = lambda k: "fixdt(%d,%d,%d)" % ty[k]
        new, nl = {}, []
        pre = INTERNAL + n + "/"

        def w(s_, d, dp=1):
            nl.append(dict({"src": s_[0] if isinstance(s_, tuple) else s_, "dst": d, "dport": dp},
                           **({"sport": s_[1]} if isinstance(s_, tuple) and s_[1] != 1 else {})))

        def gain(tag, k_coef, k_prod, v, inp):
            nm = pre + tag
            new[nm] = dict({"type": "Gain", "Gain": _lit(v), "ParamDataTypeStr": fx(k_coef), "OutDataTypeStr": fx(k_prod)}, **R)
            w(inp, nm)
            return nm

        def total(tag, signs, ins, k_acc, out_t):
            nm = pre + tag
            new[nm] = dict({"type": "Sum", "Inputs": signs, "AccumDataTypeStr": fx(k_acc), "OutDataTypeStr": out_t}, **R)
            for j, s_ in enumerate(ins):
                w(s_, nm, j + 1)
            return nm

        def delay(tag, inp, ic_=Fraction(0)):
            nm = pre + tag
            new[nm] = {"type": "UnitDelay", "InitialCondition": _lit(ic_)}
            w(inp, nm)
            return nm

        def conv(nm, k, inp):
            new[nm] = dict({"type": "DataTypeConversion", "OutDataTypeStr": fx(k) if k in ty else k}, **R)
            w(inp, nm)
            return nm

        def norm(nm, k, inp):
            """The block's own 1 / a0 (the coefficients are stored as given, a0 = 2
            divides the feedback sum): a Gain of 1 / a0 straight into the type k; a0 = 1 is a plain conversion."""
            if den[0] == 1:
                return conv(nm, k, inp)
            new[nm] = dict({"type": "Gain", "Gain": _lit(1 / den[0]), "ParamDataTypeStr": "fixdt(1,2,1)",
                            "OutDataTypeStr": fx(k) if k in ty else k}, **R)
            w(inp, nm)
            return nm

        L = max(len(num), len(den)) - 1
        num_ = num + [Fraction(0)] * (L + 1 - len(num))
        den_ = den + [Fraction(0)] * (L + 1 - len(den))
        if st == "Direct form II":
            # w0 = State(DenAcc(x - sum a_k w_k)); the delays hold w in the state type
            wsum = pre + "wsum"
            w0 = pre + "w0"
            ws = [w0]
            for k in range(1, L + 1):
                ws.append(delay("w%d" % k, ws[-1], ic[0]))
            dterms = [x] + [gain("a%d" % k, "DenCoefDataTypeStr", "DenProductDataTypeStr", den_[k], ws[k])
                            for k in range(1, L + 1) if den_[k] != 0]
            total("wsum", "+" + "-" * (len(dterms) - 1), dterms, "DenAccumDataTypeStr", fx("DenAccumDataTypeStr"))
            norm(w0, "StateDataTypeStr", wsum)
            nterms = [gain("b%d" % k, "NumCoefDataTypeStr", "NumProductDataTypeStr", num_[k], ws[k])
                      for k in range(L + 1) if num_[k] != 0]
            if not nterms:
                _refuse(where + ".Numerator", ('an all-zero numerator: Simulink runs it (output '
                    '0), its output type is not recorded -- not answered'))
            total("ysum", "+" * len(nterms), nterms, "NumAccumDataTypeStr", fx("NumAccumDataTypeStr"))
            conv(n, "OutDataTypeStr", pre + "ysum")
        elif st == "Direct form I":
            xs = [x]
            for k in range(1, len(num)):
                xs.append(delay("x%d" % k, xs[-1], ic[0]))
            nterms = [gain("b%d" % k, "NumCoefDataTypeStr", "NumProductDataTypeStr", num[k], xs[k])
                      for k in range(len(num)) if num[k] != 0]
            if not nterms:
                _refuse(where + ".Numerator", ('an all-zero numerator: Simulink runs it (output '
                    '0), its output type is not recorded -- not answered'))
            total("vsum", "+" * len(nterms), nterms, "NumAccumDataTypeStr", fx("NumAccumDataTypeStr"))
            ys = [n]
            for k in range(1, len(den)):
                ys.append(delay("y%d" % k, ys[-1], dic[0]))
            dterms = [pre + "vsum"] + [gain("a%d" % k, "DenCoefDataTypeStr", "DenProductDataTypeStr", den[k], ys[k])
                                       for k in range(1, len(den)) if den[k] != 0]
            total("ysum", "+" + "-" * (len(dterms) - 1), dterms, "DenAccumDataTypeStr", fx("DenAccumDataTypeStr"))
            norm(n, "OutDataTypeStr", pre + "ysum")
        else:   # Direct form II transposed
            # y = Out(NumAcc(b0 x + s1)); s_k = State(DenAcc(b_k x + s_{k+1} - a_k y)); s_k held by a Unit Delay
            s_in = {}
            for k in range(L, 0, -1):
                terms, signs = [gain("b%d" % k, "NumCoefDataTypeStr", "NumProductDataTypeStr", num_[k], x)], "+"
                if k < L:
                    terms.append(pre + "s%d" % (k + 1))
                    signs += "+"
                terms.append(gain("a%d" % k, "DenCoefDataTypeStr", "DenProductDataTypeStr", den_[k], n))
                signs += "-"
                total("t%d" % k, signs, terms, "DenAccumDataTypeStr", fx("DenAccumDataTypeStr"))
                conv(pre + "c%d" % k, "StateDataTypeStr", pre + "t%d" % k)
                s_in[k] = delay("s%d" % k, pre + "c%d" % k, ic[0])   # every state starts at InitialStates
            terms = [gain("b0", "NumCoefDataTypeStr", "NumProductDataTypeStr", num_[0], x)]
            if L:
                terms.append(pre + "s1")
            total("ysum", "+" * len(terms), terms, "NumAccumDataTypeStr", fx("NumAccumDataTypeStr"))
            norm(n, "OutDataTypeStr", pre + "ysum")
        lines = [ln for ln in lines if ln.get("dst") != n] + nl
        blocks.pop(n)
        blocks.update(new)
    return _with_notes(dict(spec, blocks=blocks, lines=lines), pnotes)


# recordings (after recordings): only the recorded shapes are answered. recordings: one
# signed fixed-point input of 12 or 16 bits with 8 .. 14 fraction bits, coefficient lists of up to 4 (Biquad: up to 2
# sections), a single-rate design at the base step. Outside them the refusal says what recording would open it.
REC_IN_WL, REC_IN_FL, REC_MAX_COEFS, REC_MAX_SECTIONS = (12, 16), (8, 14), 4, 2
# recordings (silent wrong outside the recordings): explicit internal types are answered only as the two
# recorded sets; inherited types only all left to the library, or with the recorded output fixdt(1,16,10); a
# non-zero InitialStates only with the (block, structure, pattern) it was recorded in; at least one delay
_K = ("StateDataTypeStr", "NumCoefDataTypeStr", "DenCoefDataTypeStr", "NumProductDataTypeStr", "DenProductDataTypeStr",
      "NumAccumDataTypeStr", "DenAccumDataTypeStr", "MultiplicandDataTypeStr", "OutDataTypeStr")
REC_EXPLICIT = {"wide": dict(zip(_K, ((1, 24, 16), (1, 16, 14), (1, 16, 14), (1, 32, 26), (1, 32, 26), (1, 40, 26),
                                      (1, 40, 26), (1, 24, 16), (1, 16, 10)))),
                "narrow": dict(zip(_K, ((1, 12, 8), (1, 10, 8), (1, 10, 8), (1, 20, 12), (1, 20, 12), (1, 20, 12),
                                        (1, 20, 12), (1, 12, 8), (1, 12, 8))))}
REC_INHERITED_OUT = (1, 16, 10)
# recordings (every stored value of each input type once, 256 .. 4096 steps): the
# input class "dense" is exactly these three types; the old class is the box above
REC_DENSE_IN = ((1, 8, 6), (1, 10, 8), (1, 12, 10))
# recordings (host, 1024 steps, 54 designs every step equal): three explicit product / accumulator sets, the
# rest left to the library (G1: state and output explicit too), Direct form II / I / II transposed, inputs below
_G105_KEYS = ("NumProductDataTypeStr", "DenProductDataTypeStr", "NumAccumDataTypeStr", "DenAccumDataTypeStr")
REC_G105 = (dict(zip(_G105_KEYS + ("StateDataTypeStr", "OutDataTypeStr"), ["fixdt(1,32,28)"] * 6)),
            dict(zip(_G105_KEYS, ("fixdt(1,32,28)", "fixdt(1,32,28)", "fixdt(1,40,28)", "fixdt(1,40,28)"))),
            dict(zip(_G105_KEYS, ("fixdt(1,24,20)", "fixdt(1,24,20)", "fixdt(1,32,20)", "fixdt(1,32,20)"))))
REC_G105_IN = ((1, 16, 14), (1, 16, 8))
# recordings (host, 512 steps): signed inputs beyond the 12 .. 16-bit class -- every internal type
# left to the library (inherited products are 32 bits, or 64 when the operand words add up to more than 32; the output
# the numerator accumulator), or that with the output fixdt(1,32,24) / fixdt(1,24,16): Direct form II, Transfer Fcn,
# Biquad. (Direct form I / II transposed with a written output: 9 of 72 differ by one or a few LSB -- refused)
REC_KP_IN = ((1, 32, 16), (1, 24, 12), (1, 18, 9), (1, 20, 16), (1, 8, 0), (1, 16, 15))
REC_KP_OUT = ((1, 32, 24), (1, 24, 16))
# Direct form I with every internal type inherited and the output written (host): the (input, output) pairs recorded
# -- recordings (its six inputs into 32/24, 24/16, 512 steps) and recordings (six
# inputs into 16/10, 16/12, 24/16, 32/24, 8192 steps); every step of all 132 equal with DFI_ACC (see rewrite)
REC_DFI_PAIRS = {(i, o) for i in REC_KP_IN for o in REC_KP_OUT} | \
    {(i, o) for i in ((1, 8, 6), (1, 10, 8), (1, 12, 10), (1, 16, 15), (1, 16, 8), (1, 24, 12))
     for o in ((1, 16, 10), (1, 16, 12), (1, 24, 16), (1, 32, 24))}
REC_DFI_IN = {i for i, _ in REC_DFI_PAIRS}
# the coefficient sets those recordings used (Floor, no saturation): independent native testing
# found the rule wrong on other coefficients inside the
# envelope (tiny / not exactly quantisable ones: the inherited coefficient types follow the values) and with Nearest
# + saturation -- only these sets are taken
REC_DFI_COEFS = {((0.0675, 0.135, 0.0675), (1.0, -1.143, 0.4128)), ((0.2929, 0.5858, 0.2929), (1.0, 0.0, 0.1716)),
                 ((0.25, 0.25), (1.0, -0.5)), ((0.2929, 0.5858, 0.2929), (1.0, -0.4, 0.1716))}
# its coefficient sets (the recordings sets): first / second order, numerator magnitudes up to 0.59, denominator
# coefficients up to 1.143, zero initial states, one Biquad section -- the class is answered within that envelope only
# (the recorded lesson: an opening covers each parameter's recorded range, not only the rule)
KP_MAX_TAPS, KP_MAX_NUM, KP_MAX_DEN = 3, 0.6, 1.15


# the coefficient sets recordings show (c1 / c2 / c3, Floor, no saturation): since the native
# slice of the DF I opening (other coefficients inside the
# envelope differ -- the inherited coefficient types follow the values) the kp class takes these sets only
KP_COEF_SETS = {((0.0675, 0.135, 0.0675), (1.0, -1.143, 0.4128)), ((0.2929, 0.5858, 0.2929), (1.0, 0.0, 0.1716)),
                ((0.25, 0.25), (1.0, -0.5))}
KP_BQ_SETS = {((0.0675, 0.135, 0.0675), (1.0, -1.143, 0.4128)), ((0.2929, 0.5858, 0.2929), (1.0, 0.0, 0.1716)),
              ((0.25, 0.25, 0.0), (1.0, -0.5, 0.0))}   # (Biquad rows, b2 / a2 written as 0)


def _strip0(v):
    v = list(v)
    while len(v) > 1 and v[-1] == 0:
        v.pop()
    return tuple(float(x) for x in v)


def kp_coverage(num, den, where, sets=KP_COEF_SETS):
    """sets None: the envelope only (Discrete Filter DF II / Transfer Fcn's kp class)"""
    if len(num) > KP_MAX_TAPS or len(den) > KP_MAX_TAPS or max(abs(v) for v in num) > KP_MAX_NUM or \
            (len(den) > 1 and max(abs(v) for v in den[1:]) > KP_MAX_DEN):
        _refuse(where, "a filter on this input type was recorded with first / second "
                "order sections, numerator magnitudes up to %s and denominator coefficients up to %s only; record "
                "these coefficients to open them" % (KP_MAX_NUM, KP_MAX_DEN))
    # compared exactly, element by element ([1 -0.5 0] is a second-order denominator, not the recorded
    # first-order [1 -0.5]); a Biquad passes its rows padded as recorded (KP_BQ_SETS)
    if sets is not None and (tuple(float(x) for x in num), tuple(float(x) for x in den)) not in set(sets):
        _refuse(where, "a filter on this input type was recorded with three coefficient "
                "sets only (%s): the inherited coefficient types follow the values, other sets inside that envelope "
                "differ; record these coefficients to "
                "open them" % "; ".join("%s / %s" % (list(a_), list(b_)) for a_, b_ in sorted(sets)))
REC_EXPLICIT["dense"] = dict(zip(_K, ((1, 24, 18), (1, 16, 14), (1, 16, 13), (1, 32, 24), (1, 32, 24), (1, 32, 24),
                                      (1, 32, 24), (1, 24, 18), (1, 16, 12))))
REC_DENSE_OUT, REC_DENSE_PROD = (1, 16, 12), (1, 24, 18)   # the dense map's output-only and product-only types
# (block, structure, pattern, input class): the inherited / output-only / product-only patterns and the explicit sets,
# as recorded equal at every step. Direct form I / II transposed with inherited types and a written output are not
# here: the dense map has them off by one LSB from step 1 (sfix8_En6) / 26 / 702 (sfix10_En8), rule not found
_DFS = (("DiscreteFilter", "Direct form II"), ("DiscreteTransferFcn", "Direct form II"))
_DFT = (("DiscreteFilter", "Direct form I"), ("DiscreteFilter", "Direct form II transposed"))
REC_PATTERNS = ({(t, st, p_, "old") for t, st in _DFS + _DFT for p_ in ("wide", "narrow")} |
                {(t, st, p_, "old") for t, st in _DFS for p_ in ("inherited", "inherited+out")} |
                {(t, st, p_, "dense") for t, st in _DFS for p_ in ("inherited", "inherited+out", "inherited+prod",
                                                                    "dense")} |
                {(t, st, p_, "dense") for t, st in _DFT for p_ in ("inherited+prod", "dense")} |
                {("DiscreteFilter", st, "g105", "g105") for st in ("Direct form II", "Direct form I",
                                                                    "Direct form II transposed")} |
                {(t, st, p_, "kp") for t, st in _DFS for p_ in ("inherited", "inherited+out")} |
                {(t, st, p_, "iw") for t, st in _DFS for p_ in ("inherited", "inherited+out")} |
                {("DiscreteFilter", "Direct form I", "inherited+out", "iw")} |
                {("DiscreteTransferFcn", "Direct form II", p_, "dtw") for p_ in ("inherited", "inherited+out")} |
                {(t, "Direct form II", "xt", "xt") for t in TYPES} |   #
                {("DiscreteFilter", "Direct form I", "inherited+out", "dfi"),
                 ("DiscreteTransferFcn", "Direct form II", "inherited+out", "dfi")} |
                {("DiscreteFilter", "Direct form I", "inherited+out", "dfw")})   # (engine/dfi_wide_domain.json)
REC_IC_NONZERO = {("DiscreteFilter", "Direct form I", "wide", "old"),
                  ("DiscreteFilter", "Direct form II transposed", "narrow", "old"),
                  ("DiscreteFilter", "Direct form II", "inherited+out", "old"),
                  ("DiscreteTransferFcn", "Direct form II", "wide", "old"),
                  ("DiscreteTransferFcn", "Direct form II", "narrow", "old"),
                  ("DiscreteTransferFcn", "Direct form II", "inherited", "old"),
                  ("DiscreteTransferFcn", "Direct form II", "inherited+out", "old")} | \
    {x for x in REC_PATTERNS if x[3] == "dense"}   # the dense map ran every pattern with InitialStates 0.5 too


_IW = _IW_BITS = None


def iw_domain():
    """{(block, structure, input, output or None, asic)} recorded by recordings (engine/iir_wide_domain.json,
    generated by the regression check)"""
    global _IW, _IW_BITS
    if _IW is None:
        d = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "iir_wide_domain.json")))
        _IW = {(c[0], c[1], tuple(c[2]), tuple(c[3]) if c[3] else None, bool(c[4])) for c in d["cells"]}
        _IW_BITS = {k: {g: set(v) for g, v in gv.items()} for k, gv in d["coef_int_bits"].items()}
    return _IW


def iw_coef_ok(block, st, num_t, den_t):
    """the library coefficient types (best precision at the input's word) follow the values;
    the iw class takes a design only when both types have integer bits (word - fraction) seen in that block /
    structure's runs (engine/iir_wide_domain.json coef_int_bits)."""
    iw_domain()
    r = _IW_BITS.get("%s|%s" % (block, st), {})
    return num_t[1] - num_t[2] in r.get("num", ()) and den_t[1] - den_t[2] in r.get("den", ())


_DTW = None


def dtw_domain():
    """{(input, written output or None, asic, RndMeth, saturation): {"counts", "num_bits", "den_bits"}} -- the
    Discrete Transfer Fcn wide class (engine/dtf_wide_domain.json, generated by the regression check from
    the recordings; recordings)"""
    global _DTW
    if _DTW is None:
        d = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "dtf_wide_domain.json")))
        _DTW = {(tuple(c["cell"][0]), tuple(c["cell"][1]) if c["cell"][1] else None, bool(c["cell"][2]), c["cell"][3],
                 c["cell"][4]): {"counts": {tuple(x) for x in c["counts"]}, "num_bits": set(c["num_bits"]),
                                 "den_bits": set(c["den_bits"]),
                                 "shapes": {tuple(x) for x in c.get("shapes", [])}} for c in d["cells"]}
    return _DTW


def dtw_reason(t_in, tout_given, asic, rm, sat, num, den, tnc, tdc):
    """None when a Discrete Transfer Fcn (Direct form II, every internal type inherited but the output, a0 = 1,
    InitialStates 0) lies in the dtw class's recorded domain, else the reason it does not. num / den: the coefficient
    values quantised to tnc / tdc (the library's best-precision types at the input's word).
    - the cell (input, written output or none, hardware, RndMeth, saturation) was recorded and model reading (a)
      reproduced it (the regression check);
    - (numerator count, denominator count) and the coefficient types' integer bits were recorded in that cell;
    - the worst-case sums (state = input type, |w| <= 2^(I-1)) must fit their accumulators, except the denominator
      sum on the host with saturation off: recordings (12 designs, s16En14 / s32En30, Floor / Nearest,
      2,192 overflowing steps) witnessed it wrapping in the accumulator before the state cast (reading (a); the
      'saturate the accumulator first' reading differs on all 12). The numerator accumulator overflowed on two host
      16-bit designs only, the denominator one with saturation on or on ASIC never: those must fit."""
    c = dtw_domain().get((tuple(t_in), tout_given, bool(asic), rm, sat))
    if c is None:
        return "this (input, output, hardware, RndMeth, saturation) cell was not recorded"
    if (len(num), len(den)) not in c["counts"]:
        return "%d numerator / %d denominator coefficients: recorded in this cell with %s" % (
            len(num), len(den), ", ".join("%d / %d" % x for x in sorted(c["counts"])))
    if (len(num), len(den), tnc[1] - tnc[2], tdc[1] - tdc[2]) not in c["shapes"]:
        # (the recorded whole tuple of counts and integer bits, not each value seen separately)
        return "%d / %d coefficients with %d / %d integer bits (numerator / denominator): not recorded together in " \
            "this cell (recorded: %s)" % (len(num), len(den), tnc[1] - tnc[2], tdc[1] - tdc[2],
                                          ", ".join("%d/%d %d/%d" % x for x in sorted(c["shapes"])))
    if tnc[1] - tnc[2] not in c["num_bits"] or tdc[1] - tdc[2] not in c["den_bits"]:
        return "coefficient types with %d / %d integer bits (numerator / denominator): recorded in this cell with " \
            "%s / %s" % (tnc[1] - tnc[2], tdc[1] - tdc[2], sorted(c["num_bits"]), sorted(c["den_bits"]))
    if den[0] != 1:
        return "a0 = %s after quantisation: a0 = 1 was recorded in this class" % _lit(den[0])
    w_, f_ = t_in[1], t_in[2]
    top = Fraction(2) ** (w_ - 1 - f_)   # |x|, |w| <= top (state = input type)

    def acc_t(tc, n_terms):
        pw = tc[1] + w_
        tp = (pw, tc[2] + f_) if asic else (32 if pw <= 32 else 64, tc[2] + f_)
        return (tp[0] + (n_terms - 1).bit_length(), tp[1]) if asic else (32 if tp[0] <= 32 else 64, tp[1])
    for nm, acc, bound, n_t in (("numerator", acc_t(tnc, len(num)), sum(abs(v) for v in num) * top, len(num)),
                                ("denominator", acc_t(tdc, len(den)), top + sum(abs(v) for v in den[1:]) * top,
                                 len(den))):
        lsb = Fraction(2) ** -acc[1]
        if nm == "denominator" and not asic and sat == "off":
            continue
        if bound + (n_t + 1) * lsb >= Fraction(2) ** (acc[0] - 1 - acc[1]):
            return "the %s accumulator (%d bits, %d fraction) can overflow with these coefficients: an " \
                "accumulator overflow is not witnessed in this class" % (nm, acc[0], acc[1])
    return None


_DFW = _DFW_IN = None


def dfw_domain():
    """{(input, written output, asic, RndMeth, saturation): {(numerator count, denominator count, numerator coefficient
    integer bits, denominator coefficient integer bits): {"num_ovf": bool, "den_ovf": bool}}} -- the Discrete Filter
    Direct form I class 'dfw' (engine/dfi_wide_domain.json, generated by the regression check from the
    recordings; recordings)"""
    global _DFW, _DFW_IN
    if _DFW is None:
        d = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "dfi_wide_domain.json")))
        _DFW_IN = {tuple(t) for t in d["inputs"]}
        _DFW = {(tuple(c["cell"][0]), tuple(c["cell"][1]), bool(c["cell"][2]), c["cell"][3], c["cell"][4]):
                {tuple(s["shape"]): {"num_ovf": s["num_ovf"], "den_ovf": s["den_ovf"]} for s in c["shapes"]}
                for c in d["cells"]}
    return _DFW


def dfw_types(t_in, tout, asic, tnc, tdc, n_num, n_den):
    """Rule R of recordings (Discrete Filter Direct form I, every internal type inherited, the output
    written): (numerator product, numerator accumulator, denominator product, denominator accumulator, the
    denominator accumulator of the 'two 64-bit products: numerator fraction, capped' candidate or None).
    - products exact: fraction = the operands' fractions added; host 32 bits, or 64 when the operand words add up to
      more; ASIC the operand words added;
    - numerator accumulator at the numerator product's fraction: host its product's word, ASIC the product plus
      ceil(log2(numerator count)) guard bits;
    - denominator accumulator: host -- the fraction of the product type with more integer bits (equal: the coarser),
      32 bits, 64 when that fraction and the larger integer bits do not fit in 32; ASIC -- the denominator product's
      fraction, wide enough for the numerator sum's and the denominator product's integer bits plus
      ceil(log2(denominator count)) guard bits."""
    if asic:
        tnp = (1, tnc[1] + t_in[1], tnc[2] + t_in[2])
        tdp = (1, tdc[1] + tout[1], tdc[2] + tout[2])
        tna = (1, tnp[1] + (n_num - 1).bit_length(), tnp[2])
        tda = (1, tdp[2] + max(tna[1] - tna[2], tdp[1] - tdp[2]) + (n_den - 1).bit_length(), tdp[2])
        return tnp, tna, tdp, tda, None
    tnp = (1, 32 if tnc[1] + t_in[1] <= 32 else 64, tnc[2] + t_in[2])
    tdp = (1, 32 if tdc[1] + tout[1] <= 32 else 64, tdc[2] + tout[2])
    i_n, i_d = tnp[1] - tnp[2], tdp[1] - tdp[2]
    f = tnp[2] if i_n > i_d else tdp[2] if i_d > i_n else min(tnp[2], tdp[2])
    tda = (1, 32 if f + max(i_n, i_d) <= 32 else 64, f)
    v4 = None
    if tnp[1] == tdp[1] == 64:
        v4 = (1, 64, min(tnp[2], 64 - max(tnc[1] + t_in[1] - tnp[2], tdc[1] + tout[1] - tdp[2])))
    return tnp, tnp, tdp, tda, v4


def dfw_agree(asic, rm, tnp, tdp, tda, tout):
    """None where the four candidate rules that reproduce every recorded run (
    R with the Convergent numerator-sum cast toward zero / toward +inf, R with an exact 64-bit join, R with the
    capped numerator fraction for two 64-bit products) and R itself give the same answer at every step; else why not.
    The numerator sum is cast into the denominator accumulator; it loses bits when that fraction is coarser than the
    numerator product's ('lossy'). Lossy + Convergent: the candidates round that cast differently. Lossy + Nearest /
    Zero with a 64-bit denominator accumulator or two 64-bit host products: the exact-join / capped-fraction candidates
    round once where R rounds twice. Lossy + Floor: nested floors agree when the output fraction is not finer than the
    accumulator's. (The overflow side is dfw_reason's: those two candidates must not overflow at all.)"""
    lossy = tda[2] < tnp[2]
    v34 = tda[1] == 64 or (not asic and tnp[1] == 64 and tdp[1] == 64)
    if lossy and rm == "Convergent":
        return ('RndMeth Convergent with the numerator sum rounded into the denominator accumulator '
            '(fraction %d to %d): the candidate rules that reproduce every recording '
            'differ there (the distinguishing value occurred at step 1979 in one recording)') % (tnp[2], tda[2]), v34
    if lossy and rm != "Floor" and v34:
        return "RndMeth %s with the numerator sum rounded into a 64-bit denominator accumulator (fraction %d to %d): " \
            "the candidate rules that reproduce every recording (one rounding or two) are not separated by any " \
            "recording there" % (rm, tnp[2], tda[2]), v34
    if lossy and tout[2] > tda[2]:
        return "an output fraction (%d) finer than the denominator accumulator's (%d): not recorded" % (
            tout[2], tda[2]), v34
    return None, v34


def dfw_reason(t_in, tout, asic, rm, sat, num, den, tnc, tdc):
    """None when a Discrete Filter Direct form I (every internal type inherited, the output written, a0 = 1,
    InitialStates / InitialDenominatorStates 0) lies in the dfw class's declared domain, else the reason it does not.
    num / den: the coefficient values quantised to tnc / tdc (the library's best-precision types at the input's word).
    - the cell (input, output, hardware, RndMeth, saturation) and, in it, the shape (numerator count, denominator
      count, the coefficient types' integer bits) were recorded, every recorded run of that shape reproduced by every
      surviving candidate (the regression check);
    - dfw_agree: the surviving candidates agree on this design;
    - every type fits 64 bits;
    - the worst-case sums (|x| <= 2^(I_in-1), |y| <= 2^(I_out-1)) fit their accumulators. The numerator accumulator
      may overflow where that was witnessed in a recorded run of the same cell and shape (every candidate shares it;
      recordings wrap at 32 bits). The denominator sum must always fit -- R's accumulator and the
      capped-fraction candidate's: where it overflows the candidates (and a host denominator accumulator of 64 bits
      always instead of 32 / 64, which no recording separates) differ, and no recording witnessed it in 32 bits."""
    if den[0] != 1:
        return "a0 = %s after quantisation: a0 = 1 was recorded in this class" % _lit(den[0])
    tnp, tna, tdp, tda, v4 = dfw_types(t_in, tout, asic, tnc, tdc, len(num), len(den))
    for nm, t in (("numerator product", tnp), ("numerator accumulator", tna), ("denominator product", tdp),
                  ("denominator accumulator", tda)):
        if t[1] > 64:
            return "the %s would need %d bits (over 64): not answered" % (nm, t[1])
    why, _ = dfw_agree(asic, rm, tnp, tdp, tda, tout)
    if why:
        return why
    c = dfw_domain().get((tuple(t_in), tuple(tout), bool(asic), rm, sat))
    if c is None:
        return "this (input, output, hardware, RndMeth, saturation) cell was not recorded"
    shape = (len(num), len(den), tnc[1] - tnc[2], tdc[1] - tdc[2])
    if shape not in c:
        return "%d numerator / %d denominator coefficients with coefficient types of %d / %d integer bits: recorded " \
            "in this cell as %s" % (shape + ("; ".join("%d / %d coefficients, %d / %d integer bits" % s
                                                        for s in sorted(c)),))
    xm = Fraction(2) ** (t_in[1] - 1 - t_in[2])
    ym = Fraction(2) ** (tout[1] - 1 - tout[2])
    top = lambda t: Fraction(2) ** (t[1] - 1 - t[2])
    bn = sum(abs(v) for v in num) * xm
    if bn + (len(num) + 1) * Fraction(2) ** -tna[2] >= top(tna):
        if not c[shape]["num_ovf"]:
            return "the numerator accumulator (%d bits, %d fraction) can overflow with these coefficients: not " \
                "witnessed in this cell" % (tna[1], tna[2])
        bn = top(tna)
    bd = bn + sum(abs(v) for v in den[1:]) * ym
    for t in (tda,) + ((v4,) if v4 else ()):
        if bd + (len(num) + len(den) + 1) * Fraction(2) ** -t[2] >= top(t):
            return "the denominator accumulator (%d bits, %d fraction) can overflow with these coefficients, where " \
                "the candidate rules that reproduce every recording are not separated: not answered" % (t[1], t[2])
    return None


def wide_recorded(block, t_in, asic):
    """The wide-input cells recorded for this block type on this input (refusal text): the iw cells (engine/
    iir_wide_domain.json: structure, written output, hardware), Discrete Filter's Direct form I dfw cells (engine/
    dfi_wide_domain.json) and Discrete Transfer Fcn's dtw cells (engine/dtf_wide_domain.json), on this hardware."""
    fx = lambda t: "fixdt(%d,%d,%d)" % tuple(t)
    t_in = tuple(t_in)
    cells = set()
    for c in iw_domain():
        if c[0] == block and c[2] == t_in and c[4] == bool(asic):
            cells.add((c[1], "the output %s" % fx(c[3]) if c[3] else "the output inherited"))
    if block == "DiscreteFilter":
        dfw_domain()
        for k in _DFW or {}:
            if k[0] == t_in and k[2] == bool(asic):
                cells.add(("Direct form I", "the output %s" % fx(k[1])))
    if block == "DiscreteTransferFcn":
        for k in dtw_domain():
            if k[0] == t_in and k[2] == bool(asic):
                cells.add(("Direct form II", "the output %s" % fx(k[1]) if k[1] else "the output inherited"))
    hw_ = "ASIC / FPGA" if asic else "the host"
    if not cells:
        return "no %s cell was recorded on this input for %s" % (block, hw_)
    return "%s on this input for %s was recorded only as %s (every internal type but the output left to the " \
        "library, InitialStates 0)" % (block, hw_, "; ".join("%s with %s" % c for c in sorted(cells)))


def in_classes(t):
    """The recorded input classes ('old', 'dense') a filter input type belongs to (fixdt(1,12,10) is in both)."""
    if not isinstance(t, tuple) or len(t) != 3 or not t[0]:
        return ()
    return tuple(c for c, ok in (("old", REC_IN_WL[0] <= t[1] <= REC_IN_WL[1] and REC_IN_FL[0] <= t[2] <= REC_IN_FL[1]),
                                 ("dense", tuple(t) in REC_DENSE_IN), ("g105", tuple(t) in REC_G105_IN),
                                 ("kp", tuple(t) in REC_KP_IN), ("dfi", tuple(t) in REC_DFI_IN),
                                 ("iw", tuple(t) in {c[2] for c in iw_domain()}),
                                 ("dtw", tuple(t) in {c[0] for c in dtw_domain()}),
                                 ("dfw", dfw_domain() is not None and tuple(t) in _DFW_IN)) if ok)


def _pat_text(pat):
    return {"wide": "the wide explicit type set", "narrow": "the narrow explicit type set",
            "dense": "the dense map's explicit type set", "inherited": "every internal type left to the library",
            "inherited+out": "inherited types with the output written",
            "inherited+prod": "inherited types with the products written",
            "g105": 'recorded explicit product / accumulator sets'}.get(pat, str(pat))


def post_compile(d, info, port_block):
    """The recorded input (one signed fixed-point scalar of the recorded sizes) and rate (single rate)."""
    import fxgraph_lib as G
    for n, (s, sp) in info.get("iir_inputs", []):
        pb = port_block(info, s, sp, "design.blocks." + n)
        t = d.otype.get(pb)
        if t is None or t == G.BOOL or not isinstance(t, tuple) or not t[0]:
            _refuse("design.blocks." + n, ('filter input of type %s: a signed fixed-point input was '
                'recorded; record the same filter on this input type to open it')
                    % (d.type_str(pb) if t else t))
        want = info.get("iir_class", {}).get(n, "old")
        if want == "xt" and tuple(t) not in xt_inputs((info.get("iir_type", {}).get(n) or (None,))[0]):
            _refuse("design.blocks." + n, "filter input %s with written internal types: not an input of a recorded "
                    "cell; record it to open it" % G._tstr(t))
        if want != "xt" and want not in in_classes(t):
            # recordings: 12 .. 16 bits, 8 .. 14 fraction bits; recordings: exactly
            # fixdt(1,8,6), fixdt(1,10,8), fixdt(1,12,10), for the patterns it ran
            _refuse("design.blocks." + n, "filter input %s with this filter's types: recorded for signed inputs of "
                    "%s only; record the filter on a %s input to open it"
                    % (G._tstr(t), "fixdt(1,8,6), fixdt(1,10,8), fixdt(1,12,10)"
                       if want == "dense" else "%d .. %d bits with %d .. %d fraction bits"
                       % (REC_IN_WL[0], REC_IN_WL[1], REC_IN_FL[0], REC_IN_FL[1]), G._tstr(t)))
        if getattr(d, "multirate", False):
            _refuse("design.blocks." + n, "a filter in a multirate design: recorded at the base step of single-rate "
                    "designs only; record it at its rate to open it")
        import fxcore as fc
        if want == "iw":
            ty_ = (info.get("iir_type", {}).get(n) or (None,))[0]
            asic_ = bool(fc._asic(getattr(d, "hw", None) or {}))
            if n in info.get("iw_bq_sections", {}) and (("BiquadFilter", "biquad", tuple(t), None, asic_) not in iw_domain() or
                                          asic_ and info.get("iw_bq_sections", {}).get(n) != 2):
                _refuse("design.blocks." + n, "BiquadFilter on %s on this hardware with %d section(s): "
                        "not recorded (one or two sections on the host, two on ASIC / FPGA); "
                        "record it to open it" % (G._tstr(t), info.get("iw_bq_sections", {}).get(n, 0)))
        if fc._asic(getattr(d, "hw", None) or {}) and want not in ("iw", "dtw", "dfw", "xt") and not (
                want == "old" and (info.get("iir_type", {}).get(n) or (None,))[0] == "DiscreteFilter"):
            # on ASIC/FPGA the inherited types are not the host's 32-bit rule. recordings: the
            # Discrete Filters of the recorded recorded shapes re-recorded on ASIC / FPGA (39 answered, every
            # step and type equal with the ASIC rule of _type_one); Transfer Fcn, Biquad, the dense map's and
            # the recorded explicit type sets are recorded on the host only
            _refuse("design.blocks." + n, "a filter on ASIC/FPGA hardware: recorded on the host only for this block "
                    "and type set (Discrete Filter in the recorded shapes); record it on this "
                    "hardware to open it")


# ---- the library's inherited internal types (30 accepted designs that leave them to the
# library, every step equal under this rule; the output type of the all-inherited Direct form II designs, 32 bits,
# and the coefficient types (dut_rtp) are recorded directly):
# - Num / Den coefficient: 'Inherit via internal rule' = best precision of the whole coefficient vector at the input's
#   word length (sfix16_En17 for [0.0675 0.135 0.0675] on a 16-bit input);
# - State / Multiplicand: 'Same as input';
# - product: 32 bits, the coefficient's fraction length + its operand's (Direct form II: the state; Direct form I:
#   the input for the numerator, the output for the denominator; Direct form II transposed: the input / the output);
# - accumulator: 32 bits, its product's fraction length; output 'Inherit via internal rule' = the numerator
#   accumulator (Direct form II only: Simulink refuses it when the denominator product is inherited too in Direct
#   form I and II transposed, IIRFilterCirularFixedPointSpecNotAllowed, 8 recordings).
INHERIT_RULE = "Inherit: Inherit via internal rule"
SAME = "Inherit: Same as input"


def _inherited(b):
    p = dict(DEFAULTS, **{k: str(v) for k, v in b.items() if k != "type"})
    return [k for k in TYPE_KEYS if not _FIX.match(p[k].strip())]


def inherited(spec):
    """Filters whose types need the input type: inherited internal types; every Biquad Filter without its _In."""
    return [n for n, b in (spec.get("blocks") or {}).items() if isinstance(b, dict) and (
        (b.get("type") in TYPES and _inherited(b)) or (b.get("type") == "BiquadFilter" and "_In" not in b))]


def provisional(spec):
    """Each filter with inherited types stands as a Gain 1 until compile() knows its input type (the probe)."""
    blocks = dict(spec["blocks"])
    for n in inherited(spec):
        blocks[n] = {"type": "Gain", "Gain": "1", "OutDataTypeStr": SAME}
    return dict(spec, blocks=blocks)


def typed(spec, probe, hw=None):
    """spec with every inherited internal type of a filter written as the fixdt the library rule gives (above), from
    the filter's input type (probe(spec, [(block, port)]) -> [type])."""
    import fxgraph_lib as G
    import fxcore as fc
    names = inherited(spec)
    if not names:
        return spec
    # recordings (silent wrong): a filter after another one was typed while the first still stood as its Gain 1
    # placeholder (the input type, not the first filter's output type). Filters are resolved in data-flow order --
    # each one only when no unresolved filter is upstream of it; a filter upstream of itself (a loop through one) is
    # refused, its placeholder having the wrong type
    up = {}
    for ln in spec.get("lines", []):
        up.setdefault(ln.get("dst"), set()).add(ln.get("src"))

    def ancestors(n):
        seen, stack = set(), list(up.get(n, ()))
        while stack:
            x = stack.pop()
            if x not in seen:
                seen.add(x)
                stack.extend(up.get(x, ()))
        return seen
    anc = {n: ancestors(n) for n in names}
    for n in names:
        if n in anc[n]:
            _refuse("design.blocks." + n, "%s with types that depend on its input type inside a feedback loop through "
                    "itself: not recorded; write its internal types (or the Biquad's input) explicitly"
                    % spec["blocks"][n]["type"])
    todo = list(names)
    while todo:
        ready = [n for n in todo if not (anc[n] & set(todo))]
        if not ready:
            _refuse("design.blocks." + todo[0], "filters whose types depend on each other: not recorded")
        n = ready[0]
        todo.remove(n)
        spec = dict(spec, blocks=dict(spec["blocks"], **{n: _type_one(spec, n, probe, hw)}))
    return spec


def _type_one(spec, n, probe, hw=None):
    import fxgraph_lib as G
    import fxcore as fc
    asic = fc._asic(hw or {})
    blocks = {n: None}
    if True:
        b = dict(spec["blocks"][n])
        where = "design.blocks." + n
        pr = probe(spec, [(n, 1)])
        if pr is None:   # the part feeding it does not compile: the full compile reports that block's refusal
            return b
        t_in = pr[0]
        if b["type"] == "BiquadFilter":
            if not isinstance(t_in, tuple) or not t_in[0]:
                _refuse(where, 'BiquadFilter input of type %s: a signed fixed-point input was recorded'
                        "" % (G._tstr(t_in) if isinstance(t_in, tuple) else t_in))
            return dict(b, _In="fixdt(%d,%d,%d)" % t_in)
        p = dict(DEFAULTS, **{k: str(v) for k, v in b.items() if k != "type"})
        if not isinstance(t_in, tuple) or not t_in[0]:
            _refuse(where, "%s input of type %s: a signed fixed-point input was recorded"
                    % (b["type"], G._tstr(t_in) if isinstance(t_in, tuple) else t_in))
        st = p["FilterStructure"].strip()
        for k in TYPE_KEYS:
            v = p[k].strip()
            ok = _FIX.match(v) or v == (SAME if k in ("StateDataTypeStr", "MultiplicandDataTypeStr") else INHERIT_RULE)
            if not ok:
                _refuse(where + "." + k, "%s %s %r: an explicit fixdt or the library's %r was recorded"
                        % (b["type"], k, v, DEFAULTS[k]))
        inh = {k for k in TYPE_KEYS if not _FIX.match(p[k].strip())}
        xt_b = xt_type(b, p, st, tuple(t_in), bool(asic), inh, where)
        if xt_b is not None:
            return xt_b
        classes = in_classes(t_in)
        cls = "dense" if classes == ("dense",) else "old"   # refined below by the written output / products
        if not classes:
            _refuse(where, ('%s input %s: recorded for signed inputs of %d.. %d bits with '
                '%d.. %d fraction bits and for fixdt(1,8,6), fixdt(1,10,8), '
                'fixdt(1,12,10); record the filter on this input to open it')
                    % (b["type"], G._tstr(t_in), REC_IN_WL[0], REC_IN_WL[1], REC_IN_FL[0], REC_IN_FL[1]))
        prods = {"NumProductDataTypeStr", "DenProductDataTypeStr"}
        g105 = b["type"] == "DiscreteFilter" and "g105" in classes and any(
            all(_FIX.match(p[k].strip()) and p[k].strip().replace(" ", "") == v for k, v in g_.items()) and
            inh == set(TYPE_KEYS) - set(g_) for g_ in REC_G105)
        if g105:
            cls = "g105"
        elif inh == set(TYPE_KEYS) - prods and "dense" in classes and all(
                _FIX.match(p[k].strip()) and tuple(int(g) for g in _FIX.match(p[k].strip()).groups()) ==
                REC_DENSE_PROD for k in prods):
            cls = "dense"   # both products fixdt(1,24,18), the rest inherited
        elif inh not in (set(TYPE_KEYS), set(TYPE_KEYS) - {"OutDataTypeStr"}):
            # recordings (silent wrong): an explicit product type with the accumulator left to the library is not
            # the fitted rule (Simulink widens the accumulator differently); only the recorded patterns are answered
            _refuse(where + "." + sorted(set(TYPE_KEYS) - inh)[0], "%s with some internal types explicit and others "
                    "left to the library: not recorded -- give every internal type explicitly, or leave all of them "
                    "(the output type may still be written)%s" % (b["type"], XT_NOTE if st == "Direct form II" and
                                                                   b["type"] in TYPES else ""))
        allowed = ("Direct form II",) if b["type"] == "DiscreteTransferFcn" else \
            ("Direct form II", "Direct form I", "Direct form II transposed")
        if st not in allowed:   # (checked again by rewrite; here before any claim about Simulink's refusals)
            _refuse(where + ".FilterStructure", "%s FilterStructure %r: answered for %s"
                    % (b["type"], st, ", ".join(allowed)))
        if p["OutDataTypeStr"].strip() == INHERIT_RULE and st != "Direct form II" and not g105 and \
                not (cls == "dense" and "NumProductDataTypeStr" not in inh):   # prod_only: recorded (dense map)
            if p["DenProductDataTypeStr"].strip() == INHERIT_RULE:   # recorded for Direct form I and II transposed
                _refuse(where + ".OutDataTypeStr", "%s %s with the output and the denominator product types both "
                        "inherited via the internal rule: Simulink refuses it (IIRFilterCirularFixedPointSpecNot"
                        "Allowed); write OutDataTypeStr" % (b["type"], st), "TYPE_MISMATCH")
            _refuse(where + ".OutDataTypeStr", "%s %s with an inherited output type: not recorded; write "
                    "OutDataTypeStr" % (b["type"], st))
        fix = lambda t: "fixdt(%d,%d,%d)" % t

        def coef_t(key, txt):
            if _FIX.match(p[key].strip()):
                m = _FIX.match(p[key].strip())
                return (int(m.group(1)), int(m.group(2)), int(m.group(3)))
            vals = [float(v) for v in _vals(txt, where + "." + key.replace("CoefDataTypeStr", "erator"))]
            nz = [v for v in vals if v != 0] or [0.0]
            return (1, t_in[1], min(fc._best_precision_fl(v, 1, t_in[1]) for v in nz))

        def given(key, dflt):
            m = _FIX.match(p[key].strip())
            return (int(m.group(1)), int(m.group(2)), int(m.group(3))) if m else dflt
        tnc = coef_t("NumCoefDataTypeStr", p["Numerator"])
        tdc = coef_t("DenCoefDataTypeStr", p["Denominator"])
        tst = given("StateDataTypeStr", t_in)
        tmu = given("MultiplicandDataTypeStr", t_in)
        tout_given = given("OutDataTypeStr", None)
        # recordings (380 models, 310 run): the recorded (block, structure, input, written output or
        # none, hardware) cells, every internal type inherited -- the iw class (engine/iir_wide_domain.json)
        # (every recorded cell starts from zero states: a non-zero InitialStates keeps its earlier class or refusal)
        iw_ = all(k_ in inh for k_ in TYPE_KEYS if k_ != "OutDataTypeStr") and st != "Direct form II transposed" and \
            (b["type"], st, tuple(t_in), tout_given, bool(asic)) in iw_domain() and \
            all(v_ == 0 for v_ in _vals(p.get("InitialStates", "0"), where + ".InitialStates")) and \
            all(v_ == 0 for v_ in _vals(p.get("InitialDenominatorStates", "0"), where + ".InitialDenominatorStates")) \
            and iw_coef_ok(b["type"], st, tnc, tdc)
        # the dfw class: Discrete Filter Direct form I, every internal type
        # but the output inherited, on the recorded cells / shapes of engine/dfi_wide_domain.json where the surviving
        # candidate rules agree (rule R; it replaces DFI_ACC there, the dfi / iw classes keep their answers elsewhere)
        dfw_ = False
        if "dfw" in classes:
            why_ = "not Direct form I with every internal type but the output left to the library and the output " \
                "written" if b["type"] == "DiscreteFilter" else "the Direct form I wide class recorded on this input " \
                "(dfw) is Discrete Filter's; " + wide_recorded(b["type"], t_in, asic)
            if b["type"] == "DiscreteFilter" and st == "Direct form I" and tout_given is not None and \
                    all(k_ in inh for k_ in TYPE_KEYS if k_ != "OutDataTypeStr"):
                why_ = "InitialStates / InitialDenominatorStates not 0"
                if all(v_ == 0 for v_ in _vals(p.get("InitialStates", "0"), where + ".InitialStates")) and \
                        all(v_ == 0 for v_ in _vals(p.get("InitialDenominatorStates", "0"),
                                                    where + ".InitialDenominatorStates")):
                    nq_ = [_quant(v, tnc) for v in _vals(p["Numerator"], where + ".Numerator")]
                    why_ = "an all-zero numerator" if all(v_ == 0 for v_ in nq_) else \
                        dfw_reason(tuple(t_in), tout_given, bool(asic), str(p.get("RndMeth", "Floor")).strip(),
                                   str(p.get("SaturateOnIntegerOverflow", "off")).strip(), nq_,
                                   [_quant(v, tdc) for v in _vals(p["Denominator"], where + ".Denominator")],
                                   tnc, tdc)
            dfw_ = why_ is None
            if not dfw_:
                classes = tuple(c_ for c_ in classes if c_ != "dfw")
                # a Direct form I of this pattern is answered only by the dfw class, the iw class's recorded cells
                # or the dfi class's recorded (input, output) pairs (host): elsewhere the dfw reason is the true one
                dfi_shape_ = b["type"] == "DiscreteFilter" and st == "Direct form I" and tout_given is not None and \
                    all(k_ in inh for k_ in TYPE_KEYS if k_ != "OutDataTypeStr")
                if not classes or dfi_shape_ and not iw_ and not (
                        not asic and (tuple(t_in), tout_given) in REC_DFI_PAIRS):
                    _refuse(where, ('%s %s on a %s input outside the recorded '
                        'Direct form I domain (engine/dfi_wide_domain.json): '
                        '%s; record this one to open it') % (b["type"], st, G._tstr(t_in), why_))
        if not iw_ and "iw" in classes:
            # the iw input class holds only inside its recorded cells: outside them the input counts for the other
            # classes alone (else the generic rules would answer an unrecorded cell)
            classes = tuple(c_ for c_ in classes if c_ != "iw")
            if not classes:
                _refuse(where, "%s %s on a %s input: native recordings cover this input in other cells only "
                        "(supported %s configurations: structure, written output, hardware; every internal type "
                        "inherited, InitialStates 0, coefficient magnitudes of the recorded scale); record this one "
                        "to open it" % (b["type"], st, G._tstr(t_in), b["type"]))
        # the dtw class: Discrete Transfer Fcn on the recorded cells of
        # recordings (engine/dtf_wide_domain.json)
        # (an iw cell recorded here too takes the dtw class: its RndMeth is checked per cell, Convergent included)
        dtw_ = False
        if "dtw" in classes:
            # for a Discrete Filter the dtw class does not apply at all (it is Discrete Transfer Fcn's); say what
            # was recorded for the block on this input instead
            why_ = "not Direct form II with every internal type but the output left to the library" \
                if b["type"] == "DiscreteTransferFcn" else "the Direct form II wide class recorded on this input " \
                "(dtw) is Discrete Transfer Fcn's; " + wide_recorded(b["type"], t_in, asic)
            if b["type"] == "DiscreteTransferFcn" and st == "Direct form II" and \
                    all(k_ in inh for k_ in TYPE_KEYS if k_ != "OutDataTypeStr"):
                why_ = "InitialStates not 0"
                if all(v_ == 0 for v_ in _vals(p.get("InitialStates", "0"), where + ".InitialStates")) and \
                        all(v_ == 0 for v_ in _vals(p.get("InitialDenominatorStates", "0"),
                                                    where + ".InitialDenominatorStates")):
                    why_ = dtw_reason(tuple(t_in), tout_given, bool(asic), str(p.get("RndMeth", "Floor")).strip(),
                                      str(p.get("SaturateOnIntegerOverflow", "off")).strip(),
                                      [_quant(v, tnc) for v in _vals(p["Numerator"], where + ".Numerator")],
                                      [_quant(v, tdc) for v in _vals(p["Denominator"], where + ".Denominator")],
                                      tnc, tdc)
            dtw_ = why_ is None
            if not dtw_:
                classes = tuple(c_ for c_ in classes if c_ != "dtw")
                if not classes:
                    _refuse(where, ('%s %s on a %s input outside the recorded wide-input domain '
                        '(engine/dtf_wide_domain.json): %s; record this one to open it')
                            % (b["type"], st, G._tstr(t_in), why_))
        # a recorded (input, output) pair of the DF I / Transfer Fcn class is judged by its own whitelist,
        # before the "inherited product over 32 bits" refusal (recordings ran those 64-bit products)
        pair_rec_ = iw_ or dtw_ or dfw_ or not asic and tout_given is not None and (tuple(t_in), tout_given) in REC_DFI_PAIRS and \
            ((b["type"] == "DiscreteFilter" and st == "Direct form I") or b["type"] == "DiscreteTransferFcn")
        x_num = tst if st == "Direct form II" else t_in
        tnp = given("NumProductDataTypeStr", None)
        if tnp is None:
            if tnc[1] + x_num[1] > 32 and "kp" not in classes and not pair_rec_:
                _refuse(where + ".NumProductDataTypeStr", "%s inherited product type with operands of %d and %d bits "
                        "(over 32): not recorded; write the product type" % (b["type"], tnc[1], x_num[1]))
            # recordings (ASIC / FPGA, 9 all-inherited designs' output types): the product at full precision
            # (both operands' words), the accumulator its product plus ceil(log2(terms)) guard bits; host: 32 bits
            # (on the host 32 bits, or 64 when the operand words add up to more)
            tnp = (1, tnc[1] + x_num[1], tnc[2] + x_num[2]) if asic else \
                (1, 32 if tnc[1] + x_num[1] <= 32 else 64, tnc[2] + x_num[2])
        n_num = len(_vals(p["Numerator"], where + ".Numerator"))
        tna = given("NumAccumDataTypeStr", (1, tnp[1] + (n_num - 1).bit_length(), tnp[2]) if asic else
                    (1, 32 if tnp[1] <= 32 else 64, tnp[2]))   # (host: 32 bits, 64 over a 32-bit product)
        tout = tout_given or tna
        x_den = tst if st == "Direct form II" else tout
        tdp = given("DenProductDataTypeStr", None)
        if tdp is None:
            if tdc[1] + x_den[1] > 32 and "kp" not in classes and not pair_rec_:
                _refuse(where + ".DenProductDataTypeStr", "%s inherited product type with operands of %d and %d bits "
                        "(over 32): not recorded; write the product type" % (b["type"], tdc[1], x_den[1]))
            tdp = (1, tdc[1] + x_den[1], tdc[2] + x_den[2]) if asic else \
                (1, 32 if tdc[1] + x_den[1] <= 32 else 64, tdc[2] + x_den[2])
        n_den = len(_vals(p["Denominator"], where + ".Denominator"))
        tda = given("DenAccumDataTypeStr", (1, tdp[1] + (n_den - 1).bit_length(), tdp[2]) if asic else
                    (1, 32 if tdp[1] <= 32 else 64, tdp[2]))   # (host: 32 bits, 64 over a 32-bit product)
        dfi_ = not dfw_ and not asic and st == "Direct form I" and tout_given is not None and \
            b["type"] == "DiscreteFilter" and \
            all(k_ in inh for k_ in TYPE_KEYS if k_ != "OutDataTypeStr") and \
            ((tuple(t_in), tout_given) in REC_DFI_PAIRS or iw_)
        if dfi_ and not iw_:
            try:
                cs_ = (tuple(float(v) for v in _vals(p["Numerator"], where + ".Numerator")),
                       tuple(float(v) for v in _vals(p["Denominator"], where + ".Denominator")))
            except Exception:   # noqa: BLE001
                cs_ = None
            if cs_ not in REC_DFI_COEFS or str(p.get("RndMeth", "Floor")).strip() != "Floor" or \
                    str(p.get("SaturateOnIntegerOverflow", "off")).strip() != "off":
                _refuse(where, "%s Direct form I with inherited internal types and the output written: recorded with "
                        "four coefficient sets only, Floor, no "
                        'saturation -- the inherited coefficient types follow the values, other sets differ'
                        "; record these coefficients to "
                        "open them" % b["type"])
        if dfi_:
            # DFI_ACC -- recordings (96 designs x 8192 steps), recordings (36 x 512) and
            # the recorded a0 = 1 Direct form I (3): numerator and denominator accumulate in one type whose
            # fraction is the numerator product's when its word is at least the denominator product's, else the finer
            # of the two; the word is 32, or 64 when that fraction and the most integer bits either product needs do
            # not fit in 32. (An earlier rule kept the integer bits but took the fraction from the
            # word: one LSB off at K's step 2931.)
            fr_ = tnp[2] if tnp[1] >= tdp[1] else max(tnp[2], tdp[2])
            ib_ = max(tnp[1] - tnp[2], tdp[1] - tdp[2])
            try:   # (the dfi class's own recorded pairs and coefficient sets keep their validated answer)
                dfi_rec_ = (tuple(t_in), tout_given) in REC_DFI_PAIRS and \
                    (tuple(float(v) for v in _vals(p["Numerator"], where + ".Numerator")),
                     tuple(float(v) for v in _vals(p["Denominator"], where + ".Denominator"))) in REC_DFI_COEFS and \
                    str(p.get("RndMeth", "Floor")).strip() == "Floor" and \
                    str(p.get("SaturateOnIntegerOverflow", "off")).strip() == "off"
            except Exception:   # noqa: BLE001
                dfi_rec_ = False
            if iw_ and fr_ + ib_ > 64 and not dfi_rec_:
                # recordings (the native independent slice): with the
                # fraction and integer bits over 64 the one 64-bit accumulator overflows where Simulink's does not
                # (sfix32_En31 in, fixdt(1,32,16) out: step 4 differs); the rule is unknown there -- refused
                _refuse(where, "%s Direct form I with inherited internal types on a %s input: the accumulator would "
                        "need %d fraction and %d integer bits (over 64); Simulink's type there is not recorded"
                        "; write the accumulator types or "
                        "record it to open it" % (b["type"], G._tstr(t_in), fr_, ib_))
            tna = tda = (1, 32 if fr_ + ib_ <= 32 else 64, fr_)
        if dfw_:
            # rule R (dfw_types): the numerator sum in its own accumulator, cast into the denominator accumulator
            tnp, tna, tdp, tda, _ = dfw_types(tuple(t_in), tout_given, bool(asic), tnc, tdc, n_num, n_den)
        rec_outs = {c: o for c, o in (("old", REC_INHERITED_OUT), ("dense", REC_DENSE_OUT)) if c in classes}
        rec_outs0_ = {c: o for c, o in (("old", REC_INHERITED_OUT), ("dense", REC_DENSE_OUT)) if c in classes}
        dtf_ = not asic and b["type"] == "DiscreteTransferFcn" and tout_given is not None and \
            all(k_ in inh for k_ in TYPE_KEYS if k_ != "OutDataTypeStr") and (tuple(t_in), tout_given) in REC_DFI_PAIRS \
            and tout_given not in rec_outs0_.values() and not ("kp" in classes and tout_given in REC_KP_OUT)
        # (a pair an earlier recording already covers keeps that class: the recorded 16/12 outputs on
        # its inputs, outputs of class kp)
        if dtw_:
            cls = "dtw"
        elif dfw_:
            cls = "dfw"
        elif iw_:
            cls = "iw"
        elif dtf_:
            # recordings (60 Discrete Transfer Fcn designs x 8192 steps, every step equal): with
            # the output written and the internal types inherited, on the same recorded (input, output) pairs and
            # coefficient sets as DF I (REC_DFI_PAIRS / REC_DFI_COEFS) the Direct form II rule above holds as it is
            cls = "dfi"
        elif dfi_:
            cls = "dfi"
        elif tout_given is not None and "NumProductDataTypeStr" in inh:
            match = [c for c, o in rec_outs.items() if o == tout_given] or \
                (["kp"] if "kp" in classes and tout_given in REC_KP_OUT else [])
            if not match:
                _refuse(where + ".OutDataTypeStr", "%s with inherited internal types and the output %s on a %s input: "
                        "recorded with the output %s only; record this output type to open it"
                        % (b["type"], fix(tout_given), G._tstr(t_in), " / ".join(fix(o) for o in rec_outs.values())))
            cls = match[0] if match else "old"   # (the refusal above raises outside the harness)
        elif "NumProductDataTypeStr" in inh:
            # every type inherited: either class's records apply. ('dfi' is the written-output DF I / Transfer Fcn
            # class only; joined here it made 'kp+dfi', which skipped the kp class's recorded-coefficient check and
            # answered any coefficients on inputs of class kp)
            cls = "+".join(c_ for c_ in classes if c_ != "dfi") or "dfi"
        b["_Pat"] = "g105" if g105 else "inherited+prod" if "NumProductDataTypeStr" not in inh else \
            "inherited+out" if tout_given is not None else "inherited"
        b["_InCls"] = cls
        if cls in ("kp", "dfi"):   # (dfi: its own four recorded sets, REC_DFI_COEFS, checked above)
            kp_coverage([float(v) for v in _vals(p["Numerator"], where + ".Numerator")],
                        [float(v) for v in _vals(p["Denominator"], where + ".Denominator")], where + ".Numerator",
                        sets=REC_DFI_COEFS if cls == "dfi" else
                        (KP_COEF_SETS | REC_DFI_COEFS if b["type"] == "DiscreteTransferFcn" and
                         (tuple(t_in), tout_given) in REC_DFI_PAIRS else KP_COEF_SETS))
            # (recordings recheck: the kp class matches the recorded coefficient sets exactly again --
            # one-digit perturbations such as [0.25 0.2499] were answered after recordings; values equal, but unrecorded
            # coefficients are the free-coefficient mechanism, a leak under the recorded support criteria. recordings keeps the
            # rounding / saturation extension, which it tested on the recorded sets)
            # recordings (the native independent slice, 144 designs,
            # every answered one equal): Discrete Filter DF II / Transfer Fcn's kp class holds across the coefficient
            # envelope (edge, tiny, not exactly quantisable values) and with RndMeth Nearest / Zero and saturation on;
            # the DF I class keeps its recorded sets and Floor (its slice found the rule wrong elsewhere)
            rm_ok_ = ("Floor",) if cls == "dfi" else ("Floor", "Nearest", "Zero")
            sat_ok_ = ("off",) if cls == "dfi" else ("off", "on")
            if str(p.get("RndMeth", "Floor")).strip() not in rm_ok_ or \
                    str(p.get("SaturateOnIntegerOverflow", "off")).strip() not in sat_ok_:
                _refuse(where + ".RndMeth", "a filter of this class was recorded with RndMeth %s and saturation %s "
                        "only; record this setting to open it" % (" / ".join(rm_ok_), " / ".join(sat_ok_)))
        for k, t in (("NumCoefDataTypeStr", tnc), ("DenCoefDataTypeStr", tdc), ("StateDataTypeStr", tst),
                     ("MultiplicandDataTypeStr", tmu), ("NumProductDataTypeStr", tnp), ("DenProductDataTypeStr", tdp),
                     ("NumAccumDataTypeStr", tna), ("DenAccumDataTypeStr", tda), ("OutDataTypeStr", tout)):
            b[k] = fix(t)
        return b


# ---- the xt class: Discrete Transfer Fcn / Discrete Filter, Direct form II,
# with some or all internal types written. Support domain:
# engine/iir_xt_domain.json, generated by the regression check from the recordings (each cell one recorded
# (block, input, hardware, RndMeth, saturation, written types) tuple whose every output sample an independent model
# reading reproduced; per cell the recorded coefficient shapes and the resolved internal types).
XT_KEYS = ("StateDataTypeStr", "NumCoefDataTypeStr", "DenCoefDataTypeStr", "NumProductDataTypeStr",
           "DenProductDataTypeStr", "NumAccumDataTypeStr", "DenAccumDataTypeStr", "OutDataTypeStr")
# recordings (both blocks, host and ASIC, saturation off / on): a written unsigned state stops Simulink
XT_UNSIGNED_STATE = (0, 16, 8)
XT_NOTE = (('; Direct form II with written internal types is answered only for the recorded '
    '(block, input, hardware, RndMeth, saturation, written types) tuples in '
    'engine/iir_xt_domain.json -- this tuple is not among them; record it to open it'))
_XT = None


def xt_domain():
    """{cell key (json text): {"cell", "shapes", "types", "sources"}} (engine/iir_xt_domain.json)"""
    global _XT
    if _XT is None:
        d = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "iir_xt_domain.json")))
        _XT = {json.dumps(c["cell"]): c for c in d["cells"]}
    return _XT


def _xt_written(p):
    return {k: tuple(int(g) for g in _FIX.match(p[k].strip()).groups()) for k in XT_KEYS if _FIX.match(p[k].strip())}


def xt_type(b, p, st, t_in, asic, inh, where):
    """b with every internal type resolved (the xt class) when the filter lies in its recorded domain; None when
    the design is not of this class's shape (some internal type other than the output written, Direct form II,
    Discrete Transfer Fcn / Discrete Filter) so the other classes decide; a refusal with the reason otherwise."""
    import fxgraph_lib as G
    import fxcore as fc
    if b["type"] not in TYPES or st != "Direct form II":
        return None
    written = _xt_written(p)
    ws_ = written.get("StateDataTypeStr")
    if ws_ is not None and not ws_[0]:
        if ws_ == XT_UNSIGNED_STATE:
            _refuse(where + ".StateDataTypeStr", ('%s Direct form II with the unsigned state %s: Simulink '
                'rejects it (Simulink:Parameters:InvParamSetting / '
                'UdtEvalsToUnsupportedFixdtSignMode); write a signed state type') % (b["type"], p["StateDataTypeStr"].strip()),
                    "PORT_TYPE_INVALID")
        _refuse(where + ".StateDataTypeStr", "%s with an unsigned state type: only fixdt(0,16,8) was recorded (a "
                "Simulink error); write a signed state type" % b["type"])
    if not set(written) - {"OutDataTypeStr"}:
        return None   # every internal type inherited (the output may be written): the other classes
    if "MultiplicandDataTypeStr" not in inh:
        _refuse(where + ".MultiplicandDataTypeStr", ('%s Direct form II with a written MultiplicandDataTypeStr '
            'and other internal types written: not recorded (the '
            'recorded models inherit this type); leave it to the library') % b["type"])
    wide_ = [k for k in XT_KEYS if k in written and written[k][1] > 64]
    if wide_:
        # (Simulink runs fixdt(1,72,40) accumulators; the engine holds up to 64 bits here)
        _refuse(where + "." + wide_[0], "%s %s %s: types over 64 bits are not supported by the engine for this "
                "block (Simulink runs it); use a type of at most 64 bits" % (b["type"], wide_[0], p[wide_[0]].strip()))
    rm = str(p.get("RndMeth", "Floor")).strip()
    sat = str(p.get("SaturateOnIntegerOverflow", "off")).strip()
    key = json.dumps([b["type"], list(t_in), bool(asic), rm, sat, [[k, list(written[k])] for k in XT_KEYS
                                                                   if k in written]])
    cell = xt_domain().get(key)
    wtxt = ", ".join("%s %s" % (k, p[k].strip()) for k in XT_KEYS if k in written)
    if cell is None:
        return None   # (the earlier classes' explicit patterns -- the recorded sets, the dense map's products -- or
        # their refusal, which names this class's domain: recordings)
        _refuse(where, ('%s Direct form II with the written internal types %s on a %s input (%s, RndMeth '
            '%s, saturation %s): this combination was not recorded -- the explicit-type '
            'class (engine/iir_xt_domain.json) takes only the recorded (block, input, '
            'hardware, rounding, saturation, written types) tuples; record this one to open it')
                % (b["type"], wtxt, G._tstr(t_in), "ASIC/FPGA" if asic else "host", rm, sat))
    for k_ in ("InitialStates", "InitialDenominatorStates"):
        if any(v != 0 for v in _vals(p.get(k_, "0"), where + "." + k_)):
            _refuse(where + "." + k_, "%s with written internal types and a non-zero %s: recorded with zero initial "
                    "states only; record it to open it" % (b["type"], k_))
    nv = _vals(p["Numerator"], where + ".Numerator")
    dv = _vals(p["Denominator"], where + ".Denominator")

    def coef_t(k, vals):
        if k in written:
            return written[k]
        nz = [float(v) for v in vals if v != 0] or [0.0]
        return (1, t_in[1], min(fc._best_precision_fl(v, 1, t_in[1]) for v in nz))
    tnc, tdc = coef_t("NumCoefDataTypeStr", nv), coef_t("DenCoefDataTypeStr", dv)
    num, den = [_quant(v, tnc) for v in nv], [_quant(v, tdc) for v in dv]
    a0 = den[0]
    shape = [len(nv), len(dv), list(tnc), list(tdc), int(a0) if a0.denominator == 1 else None]
    if shape not in cell["shapes"]:
        _refuse(where, "%s with these written types: %d numerator / %d denominator coefficients in %s / %s with a0 "
                "= %s: not recorded in this cell (recorded: %s); the library's coefficient types follow the values -- "
                "record this shape to open it" % (b["type"], len(nv), len(dv), G._tstr(tnc), G._tstr(tdc), _lit(a0),
                                                  "; ".join("%d / %d in %s / %s, a0 = %s" % (
                                                      s_[0], s_[1], G._tstr(tuple(s_[2])), G._tstr(tuple(s_[3])),
                                                      s_[4]) for s_ in cell["shapes"])))
    # the types (the regression check's reading; recordings: a denominator accumulator left to the
    # library with a written product takes the input's integer bits and fraction too, host 64 bits keeping the
    # integer bits)
    tst = written.get("StateDataTypeStr", t_in)

    def prod(k, tc):
        if k in written:
            return written[k]
        return (1, tc[1] + tst[1], tc[2] + tst[2]) if asic else (1, 32 if tc[1] + tst[1] <= 32 else 64, tc[2] + tst[2])
    tnp, tdp = prod("NumProductDataTypeStr", tnc), prod("DenProductDataTypeStr", tdc)
    tna = written.get("NumAccumDataTypeStr") or ((1, tnp[1] + (len(nv) - 1).bit_length(), tnp[2]) if asic else
                                                 (1, 32 if tnp[1] <= 32 else 64, tnp[2]))
    ib_ = max(tdp[1] - tdp[2], t_in[1] - t_in[2])
    fl_ = max(tdp[2], t_in[2])
    tda = written.get("DenAccumDataTypeStr") or ((1, ib_ + fl_ + (len(dv) - 1).bit_length(), fl_) if asic else
                                                 (1, 32, fl_) if ib_ + fl_ <= 32 else (1, 64, min(fl_, 64 - ib_)))
    T = {"StateDataTypeStr": tst, "NumCoefDataTypeStr": tnc, "DenCoefDataTypeStr": tdc, "NumProductDataTypeStr": tnp,
         "DenProductDataTypeStr": tdp, "NumAccumDataTypeStr": tna, "DenAccumDataTypeStr": tda,
         "OutDataTypeStr": written.get("OutDataTypeStr", tna)}
    if any(tuple(cell["types"][k]) != v for k, v in T.items()):
        _refuse(where, "%s: the resolved internal types differ from the recorded cell's -- not answered" % b["type"])
    # an inherited accumulator whose word no recording shows (the denominator one always; the numerator one when the
    # output is written): its word is not separated by the recordings (the same outputs were observed with 36, 37 and 64 bits),
    # so it is taken only when the worst-case sum cannot leave the narrowest candidate (integer bits as resolved,
    # no guard bits)
    wmax = Fraction(2) ** (tst[1] - tst[2] - 1)
    for nm, k_acc, tp, coefs, xin in (("denominator", "DenAccumDataTypeStr", tdp, den[1:], True),
                                      ("numerator", "NumAccumDataTypeStr", tnp, num,
                                       False)):
        if k_acc in written or (nm == "numerator" and "OutDataTypeStr" not in written):
            continue
        ib = max(tp[1] - tp[2], t_in[1] - t_in[2]) if xin else tp[1] - tp[2]
        bound = (Fraction(2) ** (t_in[1] - t_in[2] - 1) if xin else 0) + sum(
            min(Fraction(2) ** (tp[1] - tp[2] - 1), abs(c) * wmax) for c in coefs) + \
            (len(coefs) + 2) * Fraction(2) ** -min(tp[2], t_in[2] if xin else tp[2])
        if bound >= Fraction(2) ** (ib - 1):
            _refuse(where + "." + k_acc, "%s with %s left to the library and these coefficients: the %s sum can "
                    "exceed %d integer bits, where the accumulator words the recordings leave open (with / without "
                    "guard bits, 32 / 64 bits) give different results; write %s or record such an overflow to open it"
                    % (b["type"], k_acc, nm, ib, k_acc))
    b = dict(b)
    for k, v in T.items():
        b[k] = "fixdt(%d,%d,%d)" % v
    b["MultiplicandDataTypeStr"] = "fixdt(%d,%d,%d)" % t_in   # (not used by Direct form II)
    b["_Pat"] = "xt"
    b["_InCls"] = "xt"
    return b


def xt_inputs(block):
    return {tuple(c["cell"][1]) for c in xt_domain().values() if c["cell"][0] == block}


# ---- Biquad Filter (dsparch4/Biquad Filter, recordings: 21 of its 24 recordings reproduced at every step)
# Answered: BiQuadCoeffs (one row [b0 b1 b2 1 a1 a2] per section, a0 = 1), ScaleValues (one per section, or one more
# for the output; a gain of 1 is skipped, as optimizeScaleValues 'on' does), IIRFiltStruct 'Direct form II
# transposed' (default), 'Direct form II', 'Direct form I'; every data type at the library's mode except the output
# (outputMode 'Same as accumulator' or 'Binary point scaling' with outputWordLength / outputFracLength); Floor,
# wrap. With those modes every product, accumulator, state and section input / output is in the input type;
# numerator coefficients at the best precision of all of them at the input's word length, the denominator's likewise
# (own fraction length), scale values at the input's word length with scaleValueFracLength 14. Direct form II
# transposed feeds the accumulator (before the output cast) back; Direct form I keeps the section outputs.
BQ_DEFAULTS = {"FilterSource": "Dialog parameters", "CoeffSource": "Specify via dialog", "FilterObject": "BQF",
               "IIRFiltStruct": "Direct form II transposed", "BiQuadCoeffs": "[1 0.3 0.4 1 0.1 0.2]",
               "ScaleValues": "1", "IC": "0", "ICnum": "0", "ICden": "0", "a0Flag": "Warning",
               "optimizeScaleValues": "on", "ScaleValueMode": "Specify via input port (g)",
               "InputProcessing": "Columns as channels (frame based)", "roundingMode": "Floor", "overflowMode": "off",
               "stageInputMode": "Same as input", "stageInputWordLength": "16", "stageInputFracLength": "15",
               "stageOutputMode": "Same as section input", "stageOutputWordLength": "16",
               "stageOutputFracLength": "15", "multiplicandMode": "Same as output", "multiplicandWordLength": "32",
               "multiplicandFracLength": "30", "firstCoeffMode": "Same word length as input",
               "firstCoeffWordLength": "16", "firstCoeffFracLength": "15", "secondCoeffMode": "Same as numerator",
               "secondCoeffWordLength": "16", "secondCoeffFracLength": "15", "scaleValueFracLength": "14",
               "prodOutputMode": "Same as input", "prodOutputWordLength": "32", "prodOutputFracLength": "30",
               "denProdOutputFracLength": "30", "accumMode": "Same as product output", "accumWordLength": "32",
               "accumFracLength": "30", "denAccumFracLength": "30", "memoryMode": "Same as accumulator",
               "memoryWordLength": "16", "memoryFracLength": "15", "secondMemoryFracLength": "15",
               "outputMode": "Same as accumulator", "outputWordLength": "16", "outputFracLength": "15",
               "LockScale": "off", "allowOverrides": "on"}
_BQ_FIXED = ("FilterSource", "CoeffSource", "ICnum", "ICden", "optimizeScaleValues", "InputProcessing",
             "roundingMode", "overflowMode", "stageInputMode", "stageOutputMode", "multiplicandMode",
             "firstCoeffMode", "secondCoeffMode", "scaleValueFracLength", "prodOutputMode", "accumMode", "memoryMode",
             "LockScale")


def _bq_rows(text, where):
    t = str(text).strip()
    if t.startswith("[") and t.endswith("]"):
        t = t[1:-1]
    rows = [r for r in t.split(";") if r.strip()]
    out = [_vals(r, where) for r in rows]
    if not out or any(len(r) != 6 for r in out):
        _refuse(where, "one row [b0 b1 b2 a0 a1 a2] per section is required", "BAD_PARAM")
    return out


def bq_coef_fl(vals, wl):
    """The Biquad's coefficient fraction length at word length wl ('Same word length as input'), from the values
    alone: wl - 2 - floor(log2 |v|), a negative power of two one bit finer (wl - 1 - log2 |v|), the smallest over the
    values; all zero: wl - 1. The rounded value is then saturated (1.99994 at 10 bits is sfix10_En8, 511/256).
    the stored coefficient types of all 186 recordings (the regression check
    dense); the Discrete Filter's rule differs (it widens for the rounded value: sfix10_En7 there)."""
    import math
    fls = []
    for v in vals:
        v = Fraction(v)
        if v != 0:
            k = math.log2(abs(v))
            fls.append(wl - 1 - int(k) if v < 0 and k.is_integer() else wl - 2 - math.floor(k))
    return min(fls) if fls else wl - 1


def bq_rewrite(spec, info):
    """Each Biquad Filter whose input type is known (typed(): the hidden _In) as Gain / Sum / Unit Delay / DTC
    blocks named <block>\\x1d..., the output keeping the block's name."""
    import fxcore as fc
    import parameter_bounds
    blocks = dict(spec["blocks"])
    lines = [dict(ln) for ln in spec.get("lines", [])]
    pnotes = dict(spec.get(parameter_bounds.NOTES_KEY) or {})
    for n in [n for n, b in blocks.items() if isinstance(b, dict) and b.get("type") == "BiquadFilter"]:
        b = blocks[n]
        where = "design.blocks." + n
        unknown = sorted(set(b) - {"type", "_In"} - set(BQ_DEFAULTS))
        if unknown:
            _refuse(where + "." + unknown[0], "unknown parameter %s for BiquadFilter" % unknown[0], "BAD_PARAM")
        p = dict(BQ_DEFAULTS, **{k: str(v).strip() for k, v in b.items() if k != "type"})
        for k in _BQ_FIXED:
            if p[k] != BQ_DEFAULTS[k]:
                _refuse(where + "." + k, "BiquadFilter %s %r: only the library value %r was recorded"
                        % (k, p[k], BQ_DEFAULTS[k]))
        st = p["IIRFiltStruct"]
        if st not in ("Direct form II transposed", "Direct form II", "Direct form I"):
            _refuse(where + ".IIRFiltStruct", "BiquadFilter IIRFiltStruct %r: answered for Direct form II transposed "
                    "(default), Direct form II, Direct form I" % st)
        m = _FIX.match(p.get("_In", ""))
        if not m:
            _refuse(where, "BiquadFilter input type unknown (internal)", "INTERNAL")
        t_in = (int(m.group(1)), int(m.group(2)), int(m.group(3)))
        if p["outputMode"] == "Same as accumulator":
            t_out = t_in
        elif p["outputMode"] == "Binary point scaling":
            try:
                t_out = (1, int(p["outputWordLength"]), int(p["outputFracLength"]))
            except ValueError:
                _refuse(where + ".outputWordLength", "integer word / fraction lengths are required", "BAD_PARAM")
            if not 2 <= t_out[1] <= 62:
                _refuse(where + ".outputWordLength", "2 .. 62 bits", "BAD_PARAM")
        else:
            _refuse(where + ".outputMode", "BiquadFilter outputMode %r: 'Same as accumulator' or 'Binary point "
                    "scaling' were recorded" % p["outputMode"])
        rows = _bq_rows(p["BiQuadCoeffs"], where + ".BiQuadCoeffs")
        ns = len(rows)
        if ns > REC_MAX_SECTIONS:
            _refuse(where + ".BiQuadCoeffs", "BiquadFilter with %d sections: up to %d were recorded; "
                    "record one with %d sections to open it" % (ns, REC_MAX_SECTIONS, ns))
        wl = t_in[1]
        fln = bq_coef_fl([v for r in rows for v in r[:3]], wl)
        fld = bq_coef_fl([v for r in rows for v in r[4:]], wl)
        tnc, tdc = (1, wl, fln), (1, wl, fld)
        for r in rows:
            if _quant(r[3], tdc) != 1 and r[3] != 1:
                # Simulink does not divide by a0 -- it uses a0 = 1 and warns
                # (dsp:dspblkbiquad:a0NotEqualOneWarn); the values are then those of a0 = 1, with a warning the export
                # check would fail on
                _refuse(where + ".BiQuadCoeffs", "BiquadFilter a0 = %s: Simulink ignores a0 (uses 1) and warns "
                        "(a0NotEqualOneWarn); divide the section by a0 and write a0 = 1" % _lit(r[3]))
        classes = in_classes(t_in)
        # in_classes also names the Discrete Filter / Transfer Fcn input
        # classes (dfi, g105, dtw ...); a Biquad was recorded on the old box, the dense map, class kp and
        # the recorded cells (iw) only -- any other class answered it (e.g. fixdt(1,31,29), (1,9,7))
        classes = tuple(c_ for c_ in classes if c_ in ("old", "dense", "kp", "iw"))
        if not classes:
            _refuse(where, 'BiquadFilter input fixdt(%d,%d,%d): recorded for signed inputs of %d.. %d bits with %d.. '
                    '%d fraction bits and for fixdt(1,8,6), fixdt(1,10,8), fixdt(1,12,10)'
                    "; record it on this input to open it"
                    % (t_in + (REC_IN_WL[0], REC_IN_WL[1], REC_IN_FL[0], REC_IN_FL[1])))
        def bq_ct_(vals):   # the library coefficient type: best precision of all of them at the input's word
            nz = [float(v) for v in vals if v != 0] or [0.0]
            return (1, t_in[1], min(fc._best_precision_fl(v, 1, t_in[1]) for v in nz))
        iw_bq_ = t_out == t_in and any(("BiquadFilter", "biquad", tuple(t_in), None, a_) in iw_domain()
                                       for a_ in (False, True)) and len(rows) <= 2 and \
            all(str(p.get(k_, v_)).strip() == v_ for k_, v_ in BQ_DEFAULTS.items() if k_ != "BiQuadCoeffs") and \
            iw_coef_ok("BiquadFilter", "biquad", bq_ct_([v for r_ in rows for v in r_[:3]]),
                       bq_ct_([v for r_ in rows for v in r_[4:6]]))
        if not iw_bq_ and "iw" in classes:
            classes = tuple(c_ for c_ in classes if c_ != "iw")
            if not classes:
                _refuse(where, ('BiquadFilter on a %s input: native recordings cover this input '
                    'with every parameter but BiQuadCoeffs at the library default, the '
                    'output in the input type, one or two sections and coefficient '
                    'magnitudes of the recorded scale only; record this one to open it') % ("fixdt(%d,%d,%d)" % tuple(t_in)))
        rec_outs = {c: o for c, o in (("old", (1, 16, 10)), ("dense", REC_DENSE_OUT)) if c in classes}
        if iw_bq_:
            # recorded: every parameter but BiQuadCoeffs at the library default, one or two sections (one section on
            # the host only); the hardware of the recorded cell is checked in post_compile, which knows it
            info.setdefault("iw_bq_sections", {})[n] = len(rows)
            cls = "iw"   # Biquad with the library types on this input (and hardware)
        elif t_out == t_in:
            cls = "old" if "old" in classes else "dense" if "dense" in classes else classes[0]
        elif "kp" in classes and t_out in REC_KP_OUT:
            cls = "kp"   # recordings (Biquad, host)
        elif t_out in rec_outs.values():
            cls = [c for c, o in rec_outs.items() if o == t_out][0]
        else:
            # a Direct form I with a lossy output type differs (the cast output is fed back) -- only
            # the recorded outputs are answered
            _refuse(where + ".outputMode", "BiquadFilter output fixdt(%d,%d,%d) on a fixdt(%d,%d,%d) input: recorded "
                    "with the accumulator's type and %s only; record this output type to open it"
                    % (t_out + t_in + (" / ".join("fixdt(%d,%d,%d)" % o for o in rec_outs.values()),)))
        if cls == "kp":
            if len(rows) != 1:
                _refuse(where + ".BiQuadCoeffs", "BiquadFilter on this input type: one section recorded "
                        "")
            kp_coverage(list(rows[0][:3]), [1.0] + list(rows[0][4:]), where + ".BiQuadCoeffs", sets=KP_BQ_SETS)
        icv = _vals(p["IC"], where + ".IC")
        if len(icv) != 1:
            _refuse(where + ".IC", "BiquadFilter IC %s: a scalar IC was recorded" % p["IC"])
        if icv[0] != 0:
            # IC 0.5 on the small inputs (Direct form II transposed, ScaleValues 1), every
            # state of every section starting at IC in the input type -- recorded equal there only
            if "dense" in classes and t_out in (t_in, REC_DENSE_OUT):
                cls = "dense"
            else:
                _refuse(where + ".IC", "BiquadFilter IC %s on a fixdt(%d,%d,%d) input with this output: a non-zero IC "
                        'was recorded on fixdt(1,8,6), fixdt(1,10,8), fixdt(1,12,10) inputs only'
                        "; record it to open it" % ((p["IC"],) + t_in))
        ic0 = _quant(icv[0], t_in)
        if cls == "dense" and st != "Direct form II transposed":
            # ran the library structure only
            _refuse(where + ".IIRFiltStruct", "BiquadFilter %s on a fixdt(%d,%d,%d) input: the small inputs were "
                    "recorded with Direct form II transposed only" % ((st,) + t_in))
        sv = _vals(p["ScaleValues"], where + ".ScaleValues")
        if not ((len(sv) == 1 and sv[0] == 1) or len(sv) == ns + 1):
            # a scalar gain other than 1 quantises differently; one value per section is
            # refused by Simulink (2 sections) -- only 1, or one per section plus the output's, were recorded
            _refuse(where + ".ScaleValues", "BiquadFilter ScaleValues %s for %d section(s): recorded as 1 or as %d values "
                    "(one per section and one for the output); record this form to open it" % (p["ScaleValues"], ns, ns + 1))
        if len(sv) == 1 and ns > 1 and sv[0] != 1:
            _refuse(where + ".ScaleValues", "one scale value for %d sections: not recorded; give one per section "
                    "(and one for the output)" % ns)
        if len(sv) not in (1, ns, ns + 1):
            _refuse(where + ".ScaleValues", "%d scale values for %d sections (one per section, or one more)"
                    % (len(sv), ns), "BAD_PARAM")
        # the scale values' type: the coefficients' magnitude rule over the values other than 1 (a gain of 1 is
        # skipped, optimizeScaleValues) at the input's word length -- the stored types of every recording
        # (recordings [0.0675 1] at 12 bits: sfix12_En14; recordings [0.5 0.75 1] at 8 bits: sfix8_En7)
        tsc = (1, wl, bq_coef_fl([v for v in sv if v != 1], wl))
        if len(sv) == ns + 1 and sv[ns] != 1 and (sv[ns] <= 0 or not math.log2(sv[ns]).is_integer()):
            # recordings (silent wrong, confirmed natively): the output scale value 0.3 was off by 1 LSB at some
            # steps; only 1 and 0.5 were recorded there (section scale values 0.0675 / 0.75 are recorded and equal)
            _refuse(where + ".ScaleValues", "BiquadFilter output scale value %s (the last of ScaleValues): recorded as 1 "
                    "or a power of two only; scale the coefficients instead, or record this value to open it"
                    % ("%g" % float(sv[ns])))
        sv0_ = list(sv)
        sv = [v if v == 1 else _quant(v, tsc) for v in sv]
        # recordings (parameter_quantization): the coefficients (b into the numerator type, a0 / a1 / a2 into the
        # denominator type -- a0 is used as 1), the scale values and IC as used, under the user's locations
        for i_, r_ in enumerate(rows):
            for c_, v_ in enumerate(r_):
                loc_ = "%s.BiQuadCoeffs[%d]" % (where, c_ + 1) if ns == 1 else \
                    "%s.BiQuadCoeffs[%d,%d]" % (where, i_ + 1, c_ + 1)
                coef_note(pnotes, loc_, v_, Fraction(1) if c_ == 3 else _quant(v_, tnc if c_ < 3 else tdc),
                          tnc if c_ < 3 else tdc)
        for j_, (v_, q_) in enumerate(zip(sv0_, sv)):
            coef_note(pnotes, elem_loc(where, "ScaleValues", j_, len(sv0_)), v_, q_, tsc)
        coef_note(pnotes, where + ".IC", icv[0], ic0, t_in)
        src = [ln for ln in lines if ln.get("dst") == n]
        if len(src) != 1:
            _refuse(where, "BiquadFilter takes exactly one input", "BAD_CONNECTION")
        x = (src[0]["src"], int(src[0].get("sport", 1)))
        info.setdefault("iir_inputs", []).append((n, x))
        info.setdefault("iir_class", {})[n] = cls
        R = {"RndMeth": "Floor", "SaturateOnIntegerOverflow": "off"}
        fx = lambda t: "fixdt(%d,%d,%d)" % t
        new, nl = {}, []
        pre = INTERNAL + n + "/"

        def w(s_, d, dp=1):
            nl.append(dict({"src": s_[0] if isinstance(s_, tuple) else s_, "dst": d, "dport": dp},
                           **({"sport": s_[1]} if isinstance(s_, tuple) and s_[1] != 1 else {})))

        def gain(tag, v, tc, inp):
            nm = pre + tag
            new[nm] = dict({"type": "Gain", "Gain": _lit(v), "ParamDataTypeStr": fx(tc), "OutDataTypeStr": fx(t_in)}, **R)
            w(inp, nm)
            return nm

        def total(tag, signs, ins):
            nm = pre + tag
            new[nm] = dict({"type": "Sum", "Inputs": signs, "AccumDataTypeStr": fx(t_in), "OutDataTypeStr": fx(t_in)}, **R)
            for j, s_ in enumerate(ins):
                w(s_, nm, j + 1)
            return nm

        def delay(tag, inp):
            nm = pre + tag
            new[nm] = {"type": "UnitDelay", "InitialCondition": _lit(ic0)}   # every state starts at IC
            w(inp, nm)
            return nm

        def conv(nm, t, inp):
            new[nm] = dict({"type": "DataTypeConversion", "OutDataTypeStr": fx(t)}, **R)
            w(inp, nm)
            return nm

        final_g = sv[ns] if len(sv) == ns + 1 and sv[ns] != 1 else None
        v = x
        for k, r in enumerate(rows):
            g = sv[k] if k < len(sv) and len(sv) > 1 or (len(sv) == 1 and k == 0) else Fraction(1)
            if g != 1:
                v = gain("g%d" % k, g, tsc, v)
            b0, b1, b2, _a0, a1, a2 = r
            b0, b1, b2 = (_quant(c, tnc) for c in (b0, b1, b2))   # stored saturated (the edge value 1.99994 in
            a1, a2 = (_quant(c, tdc) for c in (a1, a2))         # sfix10_En8 is 511/256)
            last = k == ns - 1 and final_g is None
            t_sec = t_out if last else t_in
            yk = n if last else pre + "y%d" % k
            if st == "Direct form II transposed":
                acc = pre + "acc%d" % k
                s1, s2 = pre + "s1_%d" % k, pre + "s2_%d" % k
                total("acc%d" % k, "++", [gain("b0_%d" % k, b0, tnc, v), s1])
                t2 = total("t2_%d" % k, "+-", [gain("b2_%d" % k, b2, tnc, v), gain("a2_%d" % k, a2, tdc, acc)])
                delay("s2_%d" % k, t2)
                t1 = total("t1_%d" % k, "++-", [gain("b1_%d" % k, b1, tnc, v), s2, gain("a1_%d" % k, a1, tdc, acc)])
                delay("s1_%d" % k, t1)
                conv(yk, t_sec, acc)
            elif st == "Direct form II":
                wsum = pre + "w%d" % k
                d1, d2 = pre + "d1_%d" % k, pre + "d2_%d" % k
                total("w%d" % k, "+--", [v, gain("a1_%d" % k, a1, tdc, d1), gain("a2_%d" % k, a2, tdc, d2)])
                delay("d1_%d" % k, wsum)
                delay("d2_%d" % k, d1)
                acc = total("acc%d" % k, "+++", [gain("b0_%d" % k, b0, tnc, wsum), gain("b1_%d" % k, b1, tnc, d1),
                                                  gain("b2_%d" % k, b2, tnc, d2)])
                conv(yk, t_sec, acc)
            else:   # Direct form I: input and section-output histories
                x1 = delay("x1_%d" % k, v)
                x2 = delay("x2_%d" % k, x1)
                y1 = pre + "y1_%d" % k
                y2 = pre + "y2_%d" % k
                acc = total("acc%d" % k, "+++--", [gain("b0_%d" % k, b0, tnc, v), gain("b1_%d" % k, b1, tnc, x1),
                                                    gain("b2_%d" % k, b2, tnc, x2), gain("a1_%d" % k, a1, tdc, y1),
                                                    gain("a2_%d" % k, a2, tdc, y2)])
                conv(yk, t_sec, acc)
                delay("y1_%d" % k, yk)
                delay("y2_%d" % k, y1)
            v = yk
        if final_g is not None:
            conv(n, t_out, gain("gout", final_g, tsc, v))
        lines = [ln for ln in lines if ln.get("dst") != n] + nl
        blocks.pop(n)
        blocks.update(new)
    return _with_notes(dict(spec, blocks=blocks, lines=lines), pnotes)

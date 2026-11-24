#!/usr/bin/env -S python3 -I
"""skill/fxsim/QUICKSTART.md runs as written: every fenced block tagged
  json-run <tool> [expect=<CODE>] [save=<path>]   a request; run through engine/<tool>; status ok (or the error
                                                  code given by expect=); save= only writes it to that path,
                                                  for the sh-run commands that run it
  sh-run                                          shell commands, run from the tree directory one line at a time
  json-expect <out.json>                          outputs a preceding sh-run wrote, compared field by field
is executed. The doc's scratch directory runs/qs/ is replaced by a fresh runs/check_quickstart_<pid>/ (deleted at
the end). Sane outputs: an fxgraph answer has every Outport, nsteps samples each, stored integers inside their type,
and an explicit OutDataTypeStr of the block is its compiled type; an fxopt answer has solution.blocks and no
ERROR_BOUND_VIOLATED; an fxexport answer has the .slx and three check commands that print the FXSIM_VERDICT line and
set no exit status; an fxslx read of the exported model lists its From Workspace source. The section 2 templates
cover the block types the page promises. Prints one 'quickstart ...' count line, then FAILURES: n."""
import json
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DOC = os.path.join(ROOT, "skill", "fxsim", "QUICKSTART.md")
ENV = dict(os.environ, FXGRAPH_ERRLOG=os.devnull, FXGRAPH_RUNLOG=os.devnull)
TEMPLATE_TYPES = {"Gain", "Sum", "Product", "UnitDelay", "Delay", "DiscreteFir", "BiquadFilter", "Lookup1D",
                  "DataTypeConversion", "Saturation", "MinMax", "Switch", "HDLNCO"}
FAILS = []
SAVED = {}


def expect(label, ok, detail=""):
    if not ok:
        FAILS.append(label)
        print("FAIL", label, str(detail)[:300])


def blocks(text):
    """(info string, body, line number, section number) of every json-run / sh-run fenced block"""
    out, section = [], 0
    lines = text.split("\n")
    i = 0
    while i < len(lines):
        m = re.match(r"## (\d+)\.", lines[i])
        if m:
            section = int(m.group(1))
        if lines[i].startswith("```json-run") or lines[i].startswith("```sh-run") or \
                lines[i].startswith("```json-expect"):
            start, info, body = i + 1, lines[i][3:].strip(), []
            i += 1
            while not lines[i].startswith("```"):
                body.append(lines[i])
                i += 1
            out.append((info, "\n".join(body), start, section))
        i += 1
    return out


def last_json(text):
    try:
        return json.loads(text.strip().splitlines()[-1])
    except (ValueError, IndexError):
        return {"status": "crash", "text": text[-300:]}


def fits(ints, s, wl):
    lo, hi = (-(1 << (wl - 1)), (1 << (wl - 1)) - 1) if s else (0, (1 << wl) - 1)
    return all(isinstance(v, int) and lo <= v <= hi for v in ints)


def sane_graph(label, q, o):
    d = q["design"]
    outports = [b for b, v in d["blocks"].items() if v["type"] == "Outport"]
    n = q.get("nsteps") or min(len(v["ints"]) for v in q["inputs"].values())
    for p in outports:
        names = [k for k in o.get("outputs", {}) if k == p or k.startswith(p + "[")]
        expect("%s: output %s present" % (label, p), names, list(o.get("outputs", {})))
        for k in names:
            y = o["outputs"][k]
            if y.get("type") == "boolean":
                vals = y.get("bool", y.get("ints", []))
                expect("%s: %s boolean with %d samples" % (label, k, n), len(vals) == n and set(vals) <= {0, 1, True, False}, y)
            else:
                expect("%s: %s has %d stored integers inside %s" % (label, k, n, y.get("type")),
                       len(y.get("ints", [])) == n and fits(y["ints"], y["s"], y["wl"]), y)
    ct = o.get("compiled_types", {})
    for b, v in d["blocks"].items():
        t = v.get("OutDataTypeStr", "")
        if t.startswith("fixdt("):
            got = [c for k, c in ct.items() if k == b or k.startswith(b + "[")]   # vector blocks: b[1], b[2], ...
            expect("%s: %s compiled as its OutDataTypeStr %s" % (label, b, t), got and set(got) == {t.replace(" ", "")}, got)
    expect("%s: assumed_defaults listed" % label, isinstance(o.get("assumed_defaults"), dict))


def main():
    text = open(DOC, encoding="utf-8").read()
    scratch = "runs/check_quickstart_%d/" % os.getpid()
    sub = lambda s: s.replace("runs/qs/", scratch)
    os.makedirs(os.path.join(ROOT, scratch))
    counts = {"json_run": 0, "ok": 0, "refusals_as_documented": 0, "sh_lines": 0, "sections": 0}
    template_types = set()
    try:
        heads = re.findall(r"^## (\d+)\. (.*)$", text, re.M)
        counts["sections"] = len(heads)
        expect("five numbered sections in order", [int(h[0]) for h in heads] == [1, 2, 3, 4, 5], heads)
        expect("five pitfalls", len(re.findall(r"^\*\*[1-5]\. ", text, re.M)) == 5)
        expect("QUICKSTART.md under 350 lines", text.count("\n") < 350, text.count("\n"))
        for info, body, line, section in blocks(text):
            label = "QUICKSTART.md:%d" % line
            if info == "sh-run":
                for cmd in [c for c in body.split("\n") if c.strip()]:
                    counts["sh_lines"] += 1
                    p = subprocess.run(["bash", "-c", sub(cmd)], cwd=ROOT, env=ENV, capture_output=True, text=True,
                                       timeout=900)
                    expect("%s: '%s' exits 0" % (label, cmd[:60]), p.returncode == 0, p.stderr[-300:])
                    m = re.search(r">\s*(\S+\.out\.json)\s*$", cmd)
                    if m:
                        o = json.load(open(os.path.join(ROOT, sub(m.group(1)))))
                        expect("%s: %s status ok" % (label, m.group(1)), o.get("status") == "ok",
                               (o.get("code"), o.get("message")))
                        req = [r for r in re.findall(r"(runs/qs/\S+\.json)", cmd) if not r.endswith(".out.json")]
                        check_out(label, m.group(1), o, SAVED.get(req[0]) if req else None)
                    elif "check_batch_command" in cmd:
                        expect("%s: the printed command prints FXSIM_VERDICT" % label,
                               p.stdout.startswith("matlab -batch") and "FXSIM_VERDICT" in p.stdout, p.stdout)
                continue
            parts = info.split()
            if parts[0] == "json-expect":   # the outputs a preceding sh-run wrote, compared field by field
                counts["expected_results"] = counts.get("expected_results", 0) + 1
                want = json.loads(body)
                try:
                    got = (json.load(open(os.path.join(ROOT, sub(parts[1])))).get("outputs") or {})
                except (OSError, ValueError) as e:
                    got = {}
                    expect("%s: %s readable" % (label, parts[1]), False, e)
                for k_, w_ in want.items():
                    g_ = got.get(k_) or {}
                    expect("%s: %s %s as documented" % (label, parts[1], k_),
                           all(g_.get(f_) == v_ for f_, v_ in w_.items()), {f_: g_.get(f_) for f_ in w_})
                continue
            tool, opts = parts[1], dict(a.split("=", 1) for a in parts[2:])
            counts["json_run"] += 1
            q = json.loads(body)
            if "out" in q:
                q["out"] = sub(q["out"])
            if "save" in opts:   # saved for the sh-run chain, which runs and checks it
                with open(os.path.join(ROOT, sub(opts["save"])), "w") as f:
                    json.dump(q, f)
                SAVED[opts["save"]] = q
                continue
            p = subprocess.run([sys.executable, "-I", os.path.join(HERE, tool)], input=json.dumps(q), cwd=ROOT,
                               env=ENV, capture_output=True, text=True, timeout=900)
            o = last_json(p.stdout)
            if "expect" in opts:
                ok = o.get("status") == "error" and o.get("code") == opts["expect"] and bool(o.get("refusal_kind")) and \
                    bool(o.get("where")) and bool(o.get("next_step"))
                expect("%s: refused %s" % (label, opts["expect"]), ok, (o.get("code"), o.get("message")))
                counts["refusals_as_documented"] += ok
                continue
            ok = o.get("status") == "ok" and p.returncode == 0
            expect("%s: %s answers ok" % (label, tool), ok, (o.get("code"), o.get("where"), o.get("message")))
            counts["ok"] += ok
            if not ok:
                continue
            if tool == "fxgraph":
                sane_graph(label, q, o)
                if section == 2:
                    template_types |= {v["type"] for v in q["design"]["blocks"].values()}
            else:
                check_out(label, tool, o, q)
        missing = sorted(TEMPLATE_TYPES - template_types)
        expect("section 2 has a template for every promised block", not missing, missing)
    finally:
        shutil.rmtree(os.path.join(ROOT, scratch), ignore_errors=True)
    counts["template_types"] = len(template_types & TEMPLATE_TYPES)
    print("quickstart " + json.dumps(counts, sort_keys=True))
    print("FAILURES: %d" % len(FAILS))
    return 1 if FAILS else 0


def check_out(label, what, o, q=None):
    if "fxopt" in what:
        sol = (o.get("solution") or {}).get("blocks")
        expect("%s: fxopt gives solution.blocks" % label, isinstance(sol, dict) and sol)
        expect("%s: fxopt plan is not ERROR_BOUND_VIOLATED" % label,
               "ERROR_BOUND_VIOLATED" not in (o.get("summary") or {}).get("warnings", []), o.get("summary"))
    elif "export" in what:
        expect("%s: export wrote a model" % label, any(f.endswith(".slx") for f in o.get("files", {})), o.get("files"))
        for k in ("check_command", "check_batch_command", "check_batch_command_here"):
            c = o.get(k, "")
            expect("%s: %s prints the FXSIM_VERDICT line, no exit()" % (label, k),
                   "fprintf('FXSIM_VERDICT %s\\n', r.verdict)" in c and "exit(" not in c, c)
    elif "read" in what:
        ws = (o.get("info") or {}).get("workspace_sources") or {}
        expect("%s: fxslx reads the From Workspace source as an Inport" % label,
               ws and all(o["design"]["blocks"].get(b, {}).get("type") == "Inport" for b in ws), ws)
    elif "sim" in what:
        expect("%s: fxgraph summary present" % label, isinstance(o.get("summary"), dict))
        expect("%s: the simulated request was saved by a json-run block" % label, q is not None)
        if q is not None:
            sane_graph(label, q, o)


if __name__ == "__main__":
    sys.exit(main())

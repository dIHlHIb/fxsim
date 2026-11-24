#!/usr/bin/env -S python3 -I
"""The recorded span of linear Lookup Tables with a falling segment on an unsigned intermediate type
(120 models run): per design the key [dimensions, relation of the
table type to the intermediate type (same / narrower / wider / signed), intermediate word, ASIC, saturation]; answered
only with the explicit fraction type fixdt(0,16,15) the recordings use, inputs of 8..16 bits, Clip. Writes
engine/lu_domain.json; --check compares it with the recordings."""
import json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "ref"))
import fxcore as fc  # noqa: E402
OUT = os.path.join(HERE, "lu_domain.json")
D0 = os.path.join(ROOT, "golden", "lut_g1235_native", "g235")


def relation(it, tt):
    if tuple(tt) == tuple(it):
        return "same"
    if tt[0] == 1:
        return "signed"
    return "narrower" if tt[1] < it[1] else ("wider" if tt[1] > it[1] else "other")


def key(dims, it, tt, asic, sat):
    return [int(dims), relation(it, tt), int(it[1]), bool(asic), str(sat)]


def domain():
    keys = set()
    cases = {c["id"]: c for f in ("LU_0", "LU_1") for c in json.load(open(os.path.join(D0, "cases", f + ".json")))}
    for f in ("LU_0", "LU_1"):
        for ln in open(os.path.join(D0, "rec_%s.jsonl" % f)):
            if not ln.strip():
                continue
            r = json.loads(ln)
            if r["status"] != "ok":
                continue
            c = cases[r["id"]]
            b = [x for x in c["blocks"] if x["name"] == c["dut"]][0]
            p = b["params"]
            if str(p.get("FractionDataTypeStr", "")).strip() != "fixdt(0,16,15)":
                continue
            it, tt = fc._named_type(p["IntermediateResultsDataTypeStr"]), fc._named_type(p["TableDataTypeStr"])
            dims = 2 if "2-D" in b["lib"] else 1
            asic = str((r.get("hw") or {}).get("ProdHWDeviceType", "")).startswith("ASIC")
            keys.add(json.dumps(key(dims, it, tt, asic, p.get("SaturateOnIntegerOverflow", "off"))))
    return sorted(json.loads(k) for k in keys)


if __name__ == "__main__":
    d = domain()
    if "--check" in sys.argv:
        ok = json.load(open(OUT)) == d
        print("lu domain %d keys, %s" % (len(d), "matches the recordings" if ok else "DIFFERS"))
        print("FAILURES: %d" % (0 if ok else 1))
        sys.exit(0 if ok else 1)
    json.dump(d, open(OUT, "w"), indent=0)
    print("wrote %d keys" % len(d))

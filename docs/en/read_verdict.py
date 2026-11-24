"""Read one fxexport-check verdict from MATLAB standard output."""

import argparse
import json
import sys


def unique_fields(pairs):
    fields = dict(pairs)
    if len(fields) != len(pairs):
        raise ValueError("duplicate JSON field")
    return fields


def invalid_constant(token):
    raise ValueError(f"invalid JSON constant {token}")


def read_verdict(lines, model):
    record = None
    for line in lines:
        if not line.lstrip().startswith("{"):
            sys.stderr.write(line)
            continue
        obj = json.loads(line, object_pairs_hook=unique_fields, parse_constant=invalid_constant)
        if obj.get("tool") != "fxexport-check":
            continue
        if record is not None:
            raise ValueError("multiple fxexport-check records")
        record = obj
    if record is None:
        raise ValueError("missing fxexport-check record")
    if record.get("model") != model or record.get("format") != "fxexport-io/1":
        raise ValueError("unexpected model or checker format")
    if record.get("verdict") not in ("PASS", "FAIL", "INCOMPLETE"):
        raise ValueError("missing or unknown verdict")
    print(json.dumps(record, ensure_ascii=True))
    return 0 if record["verdict"] == "PASS" else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", help="model name returned by fxexport")
    try:
        sys.exit(read_verdict(sys.stdin, parser.parse_args().model))
    except ValueError as error:
        sys.exit(f"Unverified replay: {error}")

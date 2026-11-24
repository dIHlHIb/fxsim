"""Shared command-line guard for the tools that read their request from standard input only (fxgraph, fxslx,
fxcert, fxcodegen, fxchain). recordings: given a request path as an argument they used to wait on
stdin forever without a word; now any argument is a structured BAD_PARAM error (exit 2), like every other error.
fxopt is different: it takes the request path as its one optional argument."""
import hashlib
import json
import os
import sys


def reject_arguments(tool, errlog=None):
    """Return an exit code (and print the error record) when the tool got command-line arguments, else None."""
    if len(sys.argv) <= 1:
        return None
    rec = {"status": "error", "code": "BAD_PARAM", "where": "argv",
           "message": "%s reads its request from standard input only (got the argument %r); use: engine/%s < request.json"
                      % (tool, sys.argv[1][:200], tool),
           "input_sha256": hashlib.sha256(b"").hexdigest(),
           # same fields as every other error
           "refusal_kind": "request", "next_step": "give the request on standard input, no command-line arguments"}
    if errlog:
        try:
            os.makedirs(os.path.dirname(errlog), exist_ok=True)
            with open(errlog, "a") as f:
                f.write(json.dumps(rec) + "\n")
        except OSError:
            pass
    sys.stdout.write(json.dumps(rec) + "\n")
    sys.stdout.flush()
    return 2

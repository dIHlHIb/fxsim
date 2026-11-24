import importlib.util
import json
import os
import subprocess
import sys
from functools import lru_cache
from pathlib import Path


class FxError(ValueError):
    """The complete fxsim refusal is available as .detail."""
    def __init__(self, code, where, message, **extra):
        self.code, self.where, self.message = code, where, message
        self.detail = dict(code=code, where=where, message=message, **extra)
        super().__init__(f"Error using fxsim at {where} [{code}]\n{message}")


def require(condition, where, message, code="UNSUPPORTED"):
    if not condition:
        raise FxError(code, where, message)


def options(args, kwargs, where):
    require(len(args) % 2 == 0, where, "Expected parameter name/value pairs.", "BAD_PARAM")
    values = dict(kwargs)
    for key, value in zip(args[::2], args[1::2]):
        require(isinstance(key, str) and key not in values, where,
                f"Invalid or duplicate parameter {key!r}.", "BAD_PARAM")
        values[key] = value
    return values


def root():
    configured = os.environ.get("FXSIM_ROOT")
    choices = [Path(configured)] if configured else [Path(__file__).absolute().parents[1], Path.cwd(), Path(__file__).resolve().parents[1]]
    for path in choices:
        if (path / "engine/fxgraph").is_file() and (path / "ref/fxcore.py").is_file():
            return path.resolve()
    raise FxError("ENGINE_NOT_FOUND", "FXSIM_ROOT", "Set FXSIM_ROOT to the writable release worktree.")


@lru_cache(maxsize=2)
def core(tree):
    spec = importlib.util.spec_from_file_location("_fxsim_core", tree / "ref/fxcore.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def run(tool, request):
    tree = root()
    try:
        payload = json.dumps(request, allow_nan=False)
    except (TypeError, ValueError) as exc:
        raise FxError("BAD_INPUT", tool, str(exc)) from exc
    p = subprocess.run([sys.executable, "-I", "-B", str(tree / "engine" / tool)], cwd=tree,
                       input=payload, text=True, capture_output=True)
    try:
        reply = json.loads(p.stdout)
    except ValueError:
        raise FxError("ENGINE_FAILED", tool, p.stderr.strip() or f"Invalid response (exit {p.returncode}).")
    if reply.get("status") == "error":
        raise FxError(**reply)
    require(p.returncode == 0 and reply.get("status") == "ok", tool, "Incomplete engine response.", "ENGINE_FAILED")
    return reply

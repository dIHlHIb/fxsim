import json
import shutil
import tempfile
from fractions import Fraction
from pathlib import Path

from ._backend import FxError, require, root
from ._model import request, run_model


def _export(folder, name, inputs, *, nsteps=None, stop_time=None, prepare=None):
    # fxexport only writes inside the engine tree. Its own readback must succeed first.
    work = root() / "runs"
    work.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="fxexport-", dir=work) as scratch:
        dest = Path(scratch) / "package"
        req, fixed_step = (prepare(name, inputs, nsteps, stop_time) if prepare is not None
                           else (request(name, inputs, nsteps, stop_time), "auto"))
        result = run_model("fxexport", dict(req, name=name, out=str(dest)))
        if fixed_step != "auto":
            manifest = json.loads((dest / "manifest.json").read_text())
            actual = manifest["solver"]["FixedStep"]
            require(actual != "auto" and Fraction(str(actual)) == Fraction(fixed_step),
                    "FixedStep", "The workspace export uses a different solver step. Save a named flat SLX.")
        shutil.copytree(dest, folder)
    return result["check_batch_command_here"]


def save_system(name, directory=None, *, inputs=None, nsteps=None, stop_time=None, _prepare=None):
    """Export the model, stimulus, manifest and native replay checker to a new directory."""
    inputs = {} if inputs is None else inputs
    dest = Path(directory if directory is not None else name).absolute()
    require(not dest.exists(), str(dest), "Use a new package directory.", "OUT_EXISTS")
    dest.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".fxsim-", dir=dest.parent) as scratch:
        stage = Path(scratch) / "package"
        check = _export(stage, name, inputs, nsteps=nsteps, stop_time=stop_time, prepare=_prepare)
        try:
            stage.rename(dest)
        except OSError:
            if dest.exists():
                raise FxError("OUT_EXISTS", str(dest), "Use a new package directory.") from None
            raise
    return dict(package=dest, manifest=dest / "manifest.json", entry=dest / (name + ".slx"),
                check_batch_command_here=check)

"""Familiar Python entry points for the fxsim release worktree."""
from ._backend import FxError
from ._pyfixed import bitsll, bitsra, divide, dot, double, fi, fimath, fixdt, imag, numerictype, real, storedInteger
from ._python import add_block, add_line, close_system, design_json, get_param, new_system, save_system, set_param, sim

__all__ = ["FxError", "fi", "fimath", "numerictype", "fixdt", "double", "storedInteger", "real", "imag",
           "bitsra", "bitsll", "dot", "divide", "new_system", "close_system", "add_block", "add_line",
           "set_param", "get_param", "sim", "save_system", "design_json"]

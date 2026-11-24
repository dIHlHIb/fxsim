"""Report refusal locations on the blocks named in the request."""
import fxgraph_lib as G


def user_loc(req, where, message):
    """A refusal raised on a lowering's internal block (a Tapped Delay's stage
    'B\x1dd1') is located on the request's own block -- a stage's InitialCondition is the Tapped Delay's vinit; any
    other internal name is reported as its owner block -- and the message names the owner, not the internal block."""
    import re
    if not isinstance(where, str) or not any(ch in where for ch in G.RESERVED_CHARS):
        return where, message
    ub = ((req or {}).get("design") or {}).get("blocks") if isinstance(req, dict) else None
    ub = ub if isinstance(ub, dict) else {}
    m = re.match(r"design\.blocks\.([^\x1d\x1e\x1f]+)\x1dd(\d+)\.InitialCondition$", where)
    if m and (ub.get(m.group(1)) or {}).get("type") in ("TappedDelay", "TappedDelayEnabledSynchronous"):
        n = m.group(1)
        msg = re.sub(r"UnitDelay(EnabledSynchronous)? %s\x1dd%s: InitialCondition" % (re.escape(n), m.group(2)),
                     "%s %s: vinit" % (ub[n]["type"], n), str(message))
        return "design.blocks.%s.vinit" % n, msg
    m = re.match(r"design\.blocks\.([^.]+)(\..*)?$", where)
    if m and any(ch in m.group(1) for ch in G.RESERVED_CHARS):
        own = owner(m.group(1))
        if own in ub:
            msg = str(message).replace(m.group(1), "%s (an internal stage of its lowering)" % own)
            return "design.blocks.%s" % own, msg
    return where, message


def owner(x):
    """The model's own block an internal name belongs to ("W\x1d2" -> "W", "S[1]" -> "S")."""
    return x.split("\x1d")[0].split("\x1e")[0].split("\x1f")[0].split("[")[0] or \
        x.lstrip("".join(G.RESERVED_CHARS)).split("/")[0].split("\x1d")[0].split("[")[0]

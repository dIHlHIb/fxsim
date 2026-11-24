"""HDL FIFO (hdlsllib/HDL RAMs/HDL FIFO), Classic mode, lowered into engine blocks.

Rules, every one from recordings (recordings.jsonl, 12 Classic models):
  - every output is registered: step k shows the state after step k - 1's operation; at step 0 Out = 0,
    Empty = 1, Full = 0, Num = 0;
  - a step: pop first -- Pop and not empty takes the head, which is the next step's Out (an empty Pop does nothing,
    Out keeps its value); then push -- Push writes In at the tail when the FIFO is not full or when this step
    popped (full, push and pop together: the write is kept; fifo2_s4_int16 tells it apart); else the write is lost;
  - Empty = (count == 0), Full = (count == fifo_size), Num = count, typed ufix(ceil(log2(fifo_size + 1)));
    Out has In's type.
The FIFO becomes fifo_size data registers (q0 the head), a count register and the Out register:
  popped = Pop and count > 0;  pushed = Push and (count < N or popped);  w = count - popped (the write slot)
  q_i' = In when pushed and w == i, else q_(i+1) when popped, else q_i;  Out' = q0 when popped, else Out.
Each data register is a Hold (typed by In, as the HDL RAM's words) followed by a Unit Delay.

Recorded and accepted: fifo_size 4 .. 128 (recordings 4 / 8 / 10 / 16, recordings 5 / 32 / 64 / 128; 2 and 3
are rejected by Simulink); In boolean or fixed point of 8 .. 32 bits (int16 / sfix16_En8, then int8 / uint8 / uint16 /
int32 / sfix16_En15 / sfix12 / boolean); the output ports all on, Out only, Out and Empty, Out and Full, Out and Num,
Out, Empty and Full; push_msg / pop_msg both Warning or both Ignore (the values are equal);
ratio 1, dataDim 1, rst_port off. Refused: FWFT (recorded, its rule not found), push_msg / pop_msg Error (Simulink
stops the simulation at a push into a full FIFO), anything not recorded."""
import fxgraph_lib as G

DEFAULTS = {"fifo_size": "10", "mode": "Classic", "ratio": "1", "dataDim": "1", "push_msg": "Warning",
            "pop_msg": "Warning", "show_empty": "on", "show_full": "on", "show_num": "on", "rst_port": "off"}
# fifo_size recorded 4 / 8 / 10 / 16 and 5 / 32 / 64 / 128 (Num = ufix(ceil(log2(N + 1))),
# 128 -> uint8); Simulink rejects 2 and 3 (hdlsllib:hdlsllib:fifoSizeError, recordings)
SIZE_MIN, SIZE_MAX = 4, 128
# In: int16 / sfix16_En8, int8 / uint8 / uint16 / int32 / sfix16_En15 / sfix12 / boolean (
# depth 8 / 10) -- a FIFO stores the word as it is; accepted: boolean, or fixed point of 8 .. 32 bits, 0 <= FL <= WL - 1
DATA_WL = (8, 32)
# (show_empty, show_full, show_num) recorded (Out + Full, Out + Num, Out + Empty + Full)
PORT_SETS = {("on", "on", "on"), ("off", "off", "off"), ("on", "off", "off"), ("off", "on", "off"),
             ("off", "off", "on"), ("on", "on", "off")}
MSG_SETS = {("Warning", "Warning"), ("Ignore", "Ignore")}
INPUTS = ["In", "Push", "Pop"]


def _refuse(where, msg, code="UNSUPPORTED"):
    raise G.GraphError(code, where, msg)


def count_bits(n):
    """ceil(log2(n + 1)): the word length of Num (4 -> 3, 8 -> 4, 10 -> 4, 16 -> 5)."""
    return n.bit_length()


def params(b, where):
    p = dict(DEFAULTS, **{k: str(v).strip() for k, v in b.items() if k != "type"})
    bad = sorted(set(p) - set(DEFAULTS))
    if bad:
        _refuse(where + "." + bad[0], "unknown parameter %s for HDLFIFO" % bad[0], "BAD_PARAM")
    for k in ("show_empty", "show_full", "show_num", "rst_port"):
        if p[k] not in ("on", "off"):
            _refuse(where + "." + k, "on or off", "BAD_PARAM")
    for k in ("push_msg", "pop_msg"):
        if p[k] not in ("Ignore", "Warning", "Error"):
            _refuse(where + "." + k, "Ignore, Warning or Error", "BAD_PARAM")
    if p["mode"] not in ("Classic", "FWFT"):
        _refuse(where + ".mode", "Classic or FWFT", "BAD_PARAM")
    return p


def outputs(p):
    """The output port names in order (Out, then the ports switched on)."""
    return ["Out"] + [nm for k, nm in (("show_empty", "Empty"), ("show_full", "Full"), ("show_num", "Num"))
                      if p[k] == "on"]


def lower(n, b, X, src, add, wire, info, where):
    import parameter_bounds
    p = params(b, where)
    if p["mode"] != "Classic":
        _refuse(where + ".mode", "FWFT: recorded, but its behaviour when full was not worked out; "
                "only Classic is supported")
    if p["rst_port"] != "off":
        _refuse(where + ".rst_port", "the reset port was not recorded")
    if p["ratio"] != "1" or p["dataDim"] != "1":
        _refuse(where + (".ratio" if p["ratio"] != "1" else ".dataDim"), "ratio and dataDim other than 1 were not "
                "recorded")
    if (p["push_msg"], p["pop_msg"]) not in MSG_SETS:
        _refuse(where + ".push_msg", "push_msg / pop_msg %s / %s: recorded both Warning or both Ignore (Error stops "
                "the simulation at a push into a full FIFO, hdlsllib:hdlsllib:FIFOPushFull)"
                % (p["push_msg"], p["pop_msg"]))
    if (p["show_empty"], p["show_full"], p["show_num"]) not in PORT_SETS:
        _refuse(where + ".show_full", 'output ports Out%s: recorded all four, Out only, or Out and Empty'
                "" % "".join(", " + x for x in outputs(p)[1:]))
    try:
        N = parameter_bounds.pint(p["fifo_size"], where + ".fifo_size")
    except ValueError:
        _refuse(where + ".fifo_size", "integer FIFO size required", "BAD_PARAM")
    if N in (2, 3):
        _refuse(where + ".fifo_size", 'fifo_size %d: Simulink rejects it (hdlsllib:hdlsllib:fifoSizeError'
                ')' % N, "BAD_PARAM")
    if not SIZE_MIN <= N <= SIZE_MAX:
        _refuse(where + ".fifo_size", 'fifo_size %d: recorded %d.. %d'
                % (N, SIZE_MIN, SIZE_MAX))
    B = count_bits(N)
    CT, WT = "fixdt(0,%d,0)" % B, "fixdt(1,%d,0)" % (B + 2)
    din, push, pop = src["In"], src["Push"], src["Pop"]
    info.setdefault("fifo", []).append((n, N, din, push, pop))

    def blk(nm, d, *ins):
        add[X + nm] = d
        for k, s in enumerate(ins):
            wire(s, X + nm, k + 1)
        return X + nm

    def logic(nm, op, *ins):
        return blk(nm, {"type": "LogicalOperator", "Operator": op, "Inputs": str(len(ins))}, *ins)

    def eq(nm, s, k):
        return blk(nm, {"type": "CompareToConstant", "relop": "==", "const": str(k), "OutDataTypeStr": "boolean"}, s)

    def dtc(nm, s, ty):
        return blk(nm, {"type": "DataTypeConversion", "OutDataTypeStr": ty, "RndMeth": "Floor",
                        "SaturateOnIntegerOverflow": "off"}, s)

    def reg(name, s):
        add[name] = {"type": "UnitDelay", "InitialCondition": "0", "SampleTime": "-1"}
        wire(s, name)
        return name

    cnt = X + "count"
    empty = eq("empty", cnt, 0)
    full = eq("full", cnt, N)
    popped = logic("popped", "AND", pop, logic("nonempty", "NOT", empty))
    pushed = logic("pushed", "AND", push, logic("room", "OR", logic("notfull", "NOT", full), popped))
    npop = logic("notpopped", "NOT", popped)
    nxt = blk("count_next", {"type": "Sum", "Inputs": "++-", "OutDataTypeStr": WT, "AccumDataTypeStr": WT,
                             "SaturateOnIntegerOverflow": "off", "RndMeth": "Floor"},
              dtc("count_w", cnt, WT), dtc("push_w", pushed, WT), dtc("pop_w", popped, WT))
    reg(cnt, dtc("count_c", nxt, CT))
    at = [eq("at%d" % k, cnt, k) for k in range(N + 1)]
    q = [X + "q%d" % k for k in range(N)]
    for k in range(N):
        # the write slot is count - popped: count == k + 1 when this step popped, count == k when it did not
        slot = logic("slot%d" % k, "OR", logic("slotp%d" % k, "AND", popped, at[k + 1]),
                     logic("slotn%d" % k, "AND", npop, at[k]))
        we = logic("we%d" % k, "AND", pushed, slot)
        data = din if k == N - 1 else blk("d%d" % k, {"type": "Switch", "Criteria": "u2 ~= 0",
                                                       "OutDataTypeStr": G.fc._SAME_FIRST}, din, we, q[k + 1])
        hold = blk("h%d" % k, {"type": "Hold", "InitialCondition": "0"}, data, logic("en%d" % k, "OR", we, popped))
        reg(q[k], hold)
    reg(n, blk("hout", {"type": "Hold", "InitialCondition": "0"}, q[0], popped))
    ports = {}
    for k, nm in enumerate(outputs(p)[1:]):
        ports[k + 2] = {"Empty": empty, "Full": full, "Num": cnt}[nm]
    info.setdefault("ports", {})[n] = ports


def post_compile(d, info, port_block):
    for n, N, (s, sp), push, pop in info.get("fifo", []):
        for nm, (cs, csp) in (("Push", push), ("Pop", pop)):
            ct = d.otype.get(port_block(cs, csp, "design.blocks." + n))
            if ct != G.BOOL:
                _refuse("design.blocks." + n, "HDL FIFO %s input of type %s: recorded boolean only"
                        % (nm, d.type_str(port_block(cs, csp, "design.blocks." + n)) if ct is not None else "?"))
        t = d.otype.get(port_block(s, sp, "design.blocks." + n))
        ok = t == G.BOOL or (isinstance(t, (tuple, list)) and DATA_WL[0] <= t[1] <= DATA_WL[1] and 0 <= t[2] < t[1])
        if not ok:
            _refuse("design.blocks." + n, 'HDL FIFO with data %s: recorded boolean and fixed point of %d.. %d bits '
                    "with 0 <= FL < WL"
                    % (d.type_str(port_block(s, sp, "design.blocks." + n)) if t is not None else "?",
                       DATA_WL[0], DATA_WL[1]))

"""Binary-point arithmetic for the MATLAB batch facade, backed by the recordings core."""
from dataclasses import dataclass,replace
from ._backend import FxError,options
from functools import lru_cache
import itertools
from . import _mkernel
import math
import operator
import re

from . import _fixed as base, _pyfixed as fx
from ._marray import Array, integer, need, scalar

OPS = {"+":"add", "-":"sub", "*":"mul", ".*":"mul", "==":"eq", "~=":"ne",
       "<":"lt", ">":"gt", "<=":"le", ">=":"ge"}
CMPS = {"eq","ne","lt","gt","le","ge"}
INTEGER = re.compile(r"u?int(?:8|16|32|64)$")


@dataclass(frozen=True)
class IQ:
    real: int
    imag: int


def complex_data(x):
    return bool(x.data and isinstance(x.data[0],(complex,IQ))) if isfi(x) else any(isinstance(v,complex) for v in x.data)


def combine(x,y):
    need(isinstance(x,Array) and isinstance(y,Array) and x.kind==y.kind and x.fimath==y.fimath
         and (x.rows,x.cols)==(y.rows,y.cols) and not complex_data(x) and not complex_data(y),"complex")
    need(isfi(x) or x.kind=="double","complex input type")
    ctor=IQ if isfi(x) else complex
    return replace(x,data=tuple(ctor(r,i) for r,i in zip(x.data,y.data)))


def component(x,attr):
    return replace(x,data=tuple(getattr(v,attr) for v in x.data))


def isfi(a):
    return isinstance(a, Array) and isinstance(a.kind, base.numerictype)


def fmarg(v):
    return integer(v) if isinstance(v, Array) else v


def make_fimath(args):
    if len(args)==1 and isinstance(args[0],fx._Math): return args[0]
    return fx.fimath(*(fmarg(v) for v in args))


@lru_cache(maxsize=512)
def mathopts(fm):
    return (fm or fx._Math()).record()


def cast(x, target, local):
    need(isinstance(x,Array) and x.kind != "cell", "fi input")
    if isfi(x) and x.kind==target and x.fimath==local: return x
    engine = fx._engine()
    t = (target.SignednessBool, target.WordLength, target.FractionLength)
    fm = mathopts(local)
    if complex_data(x):
        return combine(cast(component(x,"real"),target,local),cast(component(x,"imag"),target,local))
    if isfi(x):
        codes = (x.data if x.kind == target else
                 tuple(map(_mkernel.convert_type(x.kind,target,fm),x.data)))
    elif x.kind == "double":
        scale=math.ldexp(1.,t[2])
        limit=_mkernel.conversion(t[2],target,fm)
        def nearest(v):
            lo=math.floor(v)
            return lo+int(v-lo>=0.5)
        def away(v):
            k=nearest(abs(v))
            return -k if v<0 else k
        rnd={"Nearest":nearest,"Round":away,"Convergent":round,"Floor":math.floor,"Ceiling":math.ceil,"Zero":math.trunc}[fm["rm"]]
        def quant(v):
            scaled=v*scale
            if math.isfinite(scaled) and (scaled!=0 or v==0):
                return limit(rnd(scaled))
            need(math.isfinite(v),"fi input","Use finite real inputs.")
            num,den=v.as_integer_ratio()
            return engine._cast(num,den.bit_length()-1,*t,fm)
        if len(x.data)>16 and isinstance(x.data,tuple) and x.data.count(x.data[0])==len(x.data):
            codes=(quant(x.data[0]),)*len(x.data)
        else:codes=tuple(map(quant,x.data))
    else:
        need(x.kind=="logical" or INTEGER.fullmatch(x.kind), "fi input class")
        codes = tuple(engine._cast(int(v),0,*t,fm) for v in x.data)
    return Array(x.rows,x.cols,codes,target,local)


def construct(args):
    need(args and isinstance(args[0],Array),"fi")
    x=args[0]
    if len(args)==1:
        if isfi(x): return x
        need(isinstance(x.kind,str) and INTEGER.fullmatch(x.kind),"fi")
        return cast(x,fx.numerictype(int(x.kind.startswith("int")),int(x.kind.lstrip("uint")),0),None)
    if isinstance(args[1],str):
        props=options(args[1:],{},"fi")
        keys=("Signed","WordLength","FractionLength")
        need(all(k in props for k in keys),"fi numeric properties")
        target=fx.numerictype(*(fmarg(props.pop(k)) for k in keys))
        fm=fx.fimath(**{k:fmarg(v) for k,v in props.items()}) if props else None
        return cast(x,target,fm)
    if isinstance(args[1],fx.numerictype):
        target, rest = args[1],args[2:]
    else:
        need(len(args)>=4,"fi", "Specify signedness, word length and fraction length.")
        target, rest = fx.numerictype(*map(integer,args[1:4])),args[4:]
    fm = make_fimath(rest) if rest else None
    if fm is None and isfi(x):
        fm=x.fimath
    return cast(x,target,fm)


def pair(a,b):
    need(isfi(a) and isfi(b),"fi operand classes")
    if a.fimath and b.fimath and a.fimath!=b.fimath:
        raise FxError("NATIVE_REJECTION","fimath","Conflicting local fimath settings.",native_id="fixed:fi:fimathMismatch")
    shape=b if len(a.data)==1 else a
    need(shape.data and (len(a.data)==1 or len(b.data)==1 or (a.rows,a.cols)==(b.rows,b.cols)),
         "fi shape")
    return shape,a.fimath or b.fimath


def element(op,a,b=None,**params):
    need(isfi(a),"fi operand")
    if not a.data or b is not None and not b.data:
        empty=a if not a.data else b
        need(b is None or (a.rows,a.cols)==(b.rows,b.cols) or len(a.data)==1 or len(b.data)==1,"empty fi shape")
        def exemplar(v): return replace(v,rows=1,cols=1,data=(v.data[0] if v.data else 0,))
        probe=element(op,exemplar(a),exemplar(b) if b is not None else None,**params)
        return replace(probe,rows=empty.rows,cols=empty.cols,data=())
    if isinstance(a.data[0],IQ) or (b is not None and b.data and isinstance(b.data[0],IQ)):
        need(b is not None and op in ("add","mul"),"complex fi "+op)
        ar,ai=component(a,"real"),component(a,"imag")
        br,bi=component(b,"real"),component(b,"imag")
        if op=="add": return combine(element("add",ar,br),element("add",ai,bi))
        need(isinstance(a.data[0],IQ) and isinstance(b.data[0],IQ),"mixed real/complex fi product")
        return combine(element("sub",element("mul",ar,br),element("mul",ai,bi)),
                       element("add",element("mul",ar,bi),element("mul",ai,br)))
    shape,local=pair(a,b) if b is not None else (a,a.fimath)
    engine=fx._engine()
    def bits(x,j):
        t=x.kind
        return engine._out(t.SignednessBool,t.WordLength,t.FractionLength,x.data[0 if len(x.data)==1 else j])
    if op in ("add","sub","mul","neg","abs","bitshift"):
        typ,operation=_mkernel.plan(op,a.kind,b.kind if b is not None else None,local,params.get("k",0))
        left=a.data if len(a.data)!=1 else itertools.repeat(a.data[0],len(shape.data))
        if b is None:
            codes=tuple(map(operation,left))
        else:
            right=b.data if len(b.data)!=1 else itertools.repeat(b.data[0],len(shape.data))
            codes=tuple(map(operation,left,right))
        return Array(shape.rows,shape.cols,codes,typ,local)
    comp=op in CMPS
    case=dict(fm=mathopts(local),op="cmp" if comp else op,**params)
    if comp:
        case["cmp"]=op
    result=[]
    for j in range(len(shape.data)):
        case["a"]=bits(a,j)
        if b is not None:
            case["b"]=bits(b,j)
        r=engine.eval_case(case)
        if comp:
            result.append(bool(r["bool"]))
        else:
            s,w,f,v=engine._decode(r)
            result.append(v)
    return Array(shape.rows,shape.cols,tuple(result),"logical" if comp else fx.numerictype(s,w,f),
                 None if comp else local)


def compare_double(op,x,d,left):
    need(isfi(x) and isinstance(d,Array) and d.kind=="double" and len(d.data)==1
         and not complex_data(x) and math.isfinite(d.data[0]),"mixed comparison operands")
    fn={"<":operator.lt,"<=":operator.le,"==":operator.eq,"~=":operator.ne,">=":operator.ge,">":operator.gt}[op]
    num,den=d.data[0].as_integer_ratio()
    f=x.kind.FractionLength
    rhs=num<<max(f,0)
    scale=den<<max(-f,0)
    values=[]
    for code in x.data:
        lhs=code*scale
        exact=fn(lhs,rhs) if left else fn(rhs,lhs)
        if int(float(code))!=code:
            rounded=math.ldexp(float(code),-f)
            as_double=fn(rounded,d.data[0]) if left else fn(d.data[0],rounded)
            need(exact==as_double,"wide mixed comparison","Native probes do not distinguish the wide rounding paths here.")
        values.append(exact)
    return Array(x.rows,x.cols,tuple(values),"logical")


def binary(op,a,b):
    need(op in OPS, "fi "+op)
    if isfi(a) and isinstance(b,Array) and b.kind=="double":
        if OPS[op] in CMPS: return compare_double(op,a,b,True)
        need(op in ("+","*") and len(b.data)==1,"mixed fi/double "+op)
        t=a.kind
        if op=="*":
            v=b.data[0]
            need(isinstance(v,(float,int)) and math.isfinite(v),"mixed fi multiply")
            f=(t.WordLength-int(t.SignednessBool) if v==0 or v<0 and not t.SignednessBool else
               fx._engine()._best_precision_fl(v,t.SignednessBool,t.WordLength))
            t=fx.numerictype(t.SignednessBool,t.WordLength,f)
        b=cast(b,t,a.fimath)
    elif isfi(b) and isinstance(a,Array) and a.kind=="double":
        if OPS[op] in CMPS: return compare_double(op,b,a,False)
        need(op=="-" and len(a.data)==1,"mixed double/fi "+op)
        a=cast(a,b.kind,b.fimath)
    need(isfi(a) and isfi(b),"fi operand classes")
    if op=="*" and len(a.data)>1 and len(b.data)>1:
        need(a.rows==1 and b.cols==1 and a.cols==b.rows,"fi matrix product")
        result=fx.dot(fx._Value(a.kind,a.data,a.fimath),fx._Value(b.kind,b.data,b.fimath))
        return Array(1,1,result.codes,result.numerictype,result.local)
    return element(OPS[op],a,b)


def vsum(x):
    need(isfi(x) and (x.rows==1 or x.cols==1) and len(x.data)>=2,"sum(fi) vector")
    e=fx._engine()
    t=x.kind
    v=dict(t.record(),bins=[e._out(t.SignednessBool,t.WordLength,t.FractionLength,n)["bin"] for n in x.data])
    r=e.eval_case(dict(op="sum",av=v,fm=mathopts(x.fimath)))
    s,w,f,n=e._decode(r)
    return Array(1,1,(n,),fx.numerictype(s,w,f),x.fimath)


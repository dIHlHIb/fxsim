"""Numeric and array builtins shared by function and script execution."""
import math
import operator
import re
import cmath
import itertools
from dataclasses import replace
from functools import lru_cache

from . import _marith as ar, _pyfixed as fx
from ._marray import Array, LIMIT, binary, integer, need, number, scalar

MISSING = object()
INT = ar.INTEGER


def round_away(v):
    if isinstance(v,int):
        return v
    rem,whole=math.modf(v)
    return int(whole)+(1 if rem>=0.5 else -1 if rem<=-0.5 else 0)


@lru_cache(maxsize=8)
def bounds(kind):
    w=int(kind.lstrip("uint"))
    signed=kind.startswith("int")
    low=-(1<<(w-1)) if signed else 0
    high=(1<<(w-int(signed)))-1
    return low,high

def clamp_int(v,kind):
    low,high=bounds(kind)
    z=v if isinstance(v,int) else round_away(v)
    return low if z<low else high if z>high else z


def intbinary(op,a,b):
    ia,ib=(isinstance(v.kind,str) and INT.fullmatch(v.kind) for v in (a,b))
    kind=a.kind if ia else b.kind
    need(a.kind==b.kind or (not ia and a.kind=="double" and len(a.data)==1)
         or (not ib and b.kind=="double" and len(b.data)==1),"integer classes")
    need(len(a.data)==1 or len(b.data)==1 or (a.rows,a.cols)==(b.rows,b.cols),"integer shape")
    need(op!="*" or min(len(a.data),len(b.data))==1,"integer matrix product")
    if op in ("^",".^"):
        need(ia and not ib and len(a.data)==len(b.data)==1 and 0<=integer(b)<=128,"integer power")
        return Array(1,1,(clamp_int(a.data[0]**integer(b),kind),),kind)
    fn={"+":operator.add,"-":operator.sub,"*":operator.mul,".*":operator.mul,
        "<":operator.lt,">":operator.gt,"<=":operator.le,">=":operator.ge,"==":operator.eq,"~=":operator.ne}.get(op)
    need(fn is not None,"integer "+op)
    shape=b if len(a.data)==1 else a
    cmp=op in ("<",">","<=",">=","==","~=")
    if not cmp and (not ia or not ib) and not (a if not ia else b).data[0].is_integer():
        # Keep this path exact even around int64 limits.
        from fractions import Fraction
        conv=Fraction
    else:
        conv=(lambda x:x) if cmp else int
    def one(i):
        r=fn(conv(a.data[0 if len(a.data)==1 else i]),conv(b.data[0 if len(b.data)==1 else i]))
        if cmp: return r
        if not isinstance(r,int) and r.denominator!=1:
            mag=(abs(r.numerator)*2+r.denominator)//(2*r.denominator)
            r=-mag if r<0 else mag
        return clamp_int(int(r),kind)
    return Array(shape.rows,shape.cols,tuple(one(i) for i in range(len(shape.data))),"logical" if cmp else kind)


def dims(args):
    if len(args)==1 and isinstance(args[0],Array) and (args[0].rows,args[0].cols)==(1,2):
        args=[scalar(v) for v in args[0].data]
    need(len(args) in (1,2),"array dimensions")
    h,w=integer(args[0]),integer(args[-1])
    need(h>=0 and w>=0 and h*w<=LIMIT,"array dimensions")
    return h,w


def reduce_values(n,x,dimension=None):
    need(isinstance(x,Array) and x.kind!="cell" and x.data,n)
    if ar.isfi(x):
        need(n=="sum" and dimension is None,n+" fi")
        return ar.vsum(x)
    dim=dimension or (2 if x.rows==1 else 1)
    need(dim in (1,2),n+" dimension")
    shape=(1,x.cols) if dim==1 else (x.rows,1)
    seq=([x.at(r,c) for r in range(x.rows)] for c in range(x.cols)) if dim==1 else (
         [x.at(r,c) for c in range(x.cols)] for r in range(x.rows))
    fn={"sum":lambda q: sum(q,0.0),"prod":math.prod,"max":max,"min":min,"any":any,"all":all}[n]
    if (dim==1 and x.cols==1) or (dim==2 and x.rows==1):
        seq=(x.data,)
    kind="double" if n in ("sum","prod") else "logical" if n in ("any","all") else x.kind
    return Array(*shape,tuple(fn(v) for v in seq),kind)


def builtin(n,args):
    a=args
    x=a[0] if a else None
    if n=="sscanf":
        need(len(a)==2 and isinstance(x,str) and isinstance(a[1],str),n)
        m=re.fullmatch(r"([a-zA-Z]*)%d",a[1]); need(m,n)
        v=re.fullmatch(re.escape(m[1])+r"([+-]?[0-9]+)",x)
        return scalar(int(v[1])) if v else Array(0,1,())
    if n=="strcmp" and len(a)==2 and isinstance(x,str) and isinstance(a[1],Array) and a[1].kind=="cell":
        need(all(isinstance(v,str) for v in a[1].data),n)
        return replace(a[1],kind="logical",data=tuple(x==v for v in a[1].data))
    if n=="dec2bin":
        need(len(a)==2 and isinstance(x,Array) and x.kind=="double" and all(int(v)==v and v>=0 for v in x.data),n)
        w=integer(a[1]); need(1<=w<=64 and len(x.data)*w<=LIMIT and all(v<2**w for v in x.data),n)
        rows=[format(int(v),f"0{w}b") for v in x.data]
        return Array(len(rows),w,tuple(row[j] for j in range(w) for row in rows),"char")
    if n=="bin2dec":
        need(len(a)==1 and isinstance(x,Array) and x.kind=="char" and 1<=x.cols<=53 and all(v in ("0","1") for v in x.data),n)
        return Array(x.rows,1,tuple(float(int("".join(x.at(i,j) for j in range(x.cols)),2)) for i in range(x.rows)))
    if n=="fliplr":
        need(len(a)==1 and isinstance(x,Array),n)
        return replace(x,data=tuple(x.at(i,j) for j in reversed(range(x.cols)) for i in range(x.rows)))
    if n=="cumsum":
        need(len(a)==1 and isinstance(x,Array) and (x.rows==1 or x.cols==1)
             and isinstance(x.kind,str) and INT.fullmatch(x.kind),n)
        vals=tuple(itertools.accumulate(x.data))
        need(all(v==clamp_int(v,x.kind) for v in vals),n+" overflow")
        return replace(x,data=vals)
    if n=="idivide":
        need(len(a)==3 and isinstance(x,Array) and isinstance(a[1],Array) and x.kind==a[1].kind
             and INT.fullmatch(x.kind) and len(a[1].data)==1 and a[2] in ("fix","floor"),n)
        k=a[1].data[0]; need(k!=0,n)
        return replace(x,data=tuple(clamp_int(v//k if a[2]=="floor" else abs(v)//abs(k)*(-1 if (v<0)!=(k<0) else 1),x.kind) for v in x.data))
    if n in ("sign","abs") and isinstance(x,Array) and isinstance(x.kind,str) and INT.fullmatch(x.kind):
        need(len(a)==1,n)
        return replace(x,data=tuple(clamp_int(abs(v) if n=="abs" else (v>0)-(v<0),x.kind) for v in x.data))
    if n in ("char","lower"):
        need(len(a)==1 and isinstance(x,str),n)
        return x.lower() if n=="lower" else x
    if n in ("strcmpi","contains","startsWith","strncmp"):
        need(len(a)==(3 if n=="strncmp" else 2) and all(isinstance(v,str) for v in a[:2]),n)
        y=a[1]
        if n=="strncmp":
            k=integer(a[2]); need(k>=0,n)
            flag=x[:k]==y[:k]
        else:
            flag={"strcmpi":x.lower()==y.lower(),"contains":y in x,"startsWith":x.startswith(y)}[n]
        return scalar(flag,"logical")
    if n=="isa":
        need(len(a)==2 and isinstance(a[1],str),n)
        kind="embedded.fi" if ar.isfi(x) else x.kind if isinstance(x,Array) else "char" if isinstance(x,str) else None
        need(kind is not None,n)
        return scalar(kind==a[1],"logical")
    if n in ("zeros","ones","cell"):
        kind="cell" if n=="cell" else "double"
        if a and isinstance(a[-1],str):
            kind=a[-1]; need(n!="cell" and (kind=="double" or INT.fullmatch(kind)),n)
            a=a[:-1]
        h,w=dims(a)
        v=Array(0,0,()) if kind=="cell" else (float(n=="ones") if kind=="double" else int(n=="ones"))
        return Array(h,w,(v,)*(h*w),kind)
    if n=="repmat":
        need(len(a) in (2,3) and isinstance(x,Array),n)
        r,c=dims(a[1:])
        need(len(x.data)*r*c<=LIMIT,n)
        return Array(x.rows*r,x.cols*c,tuple(x.at(i%x.rows,j%x.cols) for j in range(x.cols*c)
                                                for i in range(x.rows*r)),x.kind,x.fimath)
    if INT.fullmatch(n):
        need(len(a)==1 and isinstance(x,Array) and x.kind!="cell",n)
        if ar.isfi(x):
            # Scale with integer arithmetic so raw 64-bit inputs do not lose bits.
            from fractions import Fraction
            f=x.kind.FractionLength
            vals=(Fraction(v,1<<f) if f>=0 else v<<-f for v in x.data)
            def convert(v):
                if isinstance(v,Fraction):
                    mag=(2*abs(v.numerator)+v.denominator)//(2*v.denominator)
                    v=-mag if v<0 else mag
                return clamp_int(v,n)
        else:
            vals=x.data
            convert=lambda v:clamp_int(v,n)
        return Array(x.rows,x.cols,tuple(convert(v) for v in vals),n)
    if n in ("min","max") and len(a)==2:
        y=a[1]
        if isinstance(x,Array) and isinstance(y,Array) and isinstance(x.kind,str) and isinstance(y.kind,str) and x.kind!=y.kind:
            i=x if INT.fullmatch(x.kind) else y
            d=y if i is x else x
            need(INT.fullmatch(i.kind) and d.kind=="double" and len(d.data)==1 and
                 d.data[0]==clamp_int(d.data[0],i.kind),n+" integer bound")
            v=replace(d,data=(int(d.data[0]),),kind=i.kind)
            x,y=(i,v) if i is x else (v,i)
        need(isinstance(x,Array) and isinstance(y,Array) and x.kind==y.kind and isinstance(x.kind,str)
             and x.kind!="cell" and (len(x.data)==1 or len(y.data)==1 or (x.rows,x.cols)==(y.rows,y.cols)),n)
        q=y if len(x.data)==1 else x
        fn=min if n=="min" else max
        return Array(q.rows,q.cols,tuple(fn(x.data[0 if len(x.data)==1 else i],y.data[0 if len(y.data)==1 else i])
                                        for i in range(len(q.data))),x.kind)
    if n in ("sum","prod","min","max","any","all"):
        need(len(a)==1 or len(a)==2 and n in ("sum","prod","any","all"),n)
        return reduce_values(n,x,integer(a[1]) if len(a)==2 else None)
    if n=="mod":
        need(len(a)==2 and all(isinstance(v,Array) for v in a),n)
        y=a[1]
        if isinstance(x.kind,str) and INT.fullmatch(x.kind):
            need(len(y.data)==1 and (y.kind==x.kind or y.kind=="double") and y.data[0]!=0
                 and y.data[0]==clamp_int(y.data[0],x.kind),n)
            return replace(x,data=tuple(v%int(y.data[0]) for v in x.data))
        need(x.kind==y.kind=="double",n)
        need(len(y.data)==1 and y.data[0]!=0 and all(v.is_integer() and abs(v)<=2**53 for v in (*x.data,*y.data)),n)
        return replace(x,data=tuple(v%y.data[0] for v in x.data))
    if n in ("sin","cos","exp","sqrt","atan","ceil","log2","sign"):
        need(len(a)==1 and isinstance(x,Array) and x.kind=="double",n)
        fn=(lambda v:(v>0)-(v<0)) if n=="sign" else getattr(math,n)
        return replace(x,data=tuple(cmath.exp(v) if isinstance(v,complex) and n=="exp" else float(fn(v)) for v in x.data))
    if n in ("real","imag"):
        need(len(a)==1 and isinstance(x,Array) and x.kind!="cell",n)
        data=tuple(getattr(v,n) for v in x.data)
        return replace(x,data=data)
    if n=="bitshift" and isinstance(x,Array) and isinstance(x.kind,str) and INT.fullmatch(x.kind):
        need(len(a)==2,n)
        k=integer(a[1]); w=int(x.kind.lstrip("uint"))
        need(abs(k)<=w,n)
        def shift(v):
            if k<0: return v>>-k
            z=(v<<k)% (1<<w)
            return z-(1<<w) if x.kind.startswith("int") and z>>(w-1) else z
        return replace(x,data=tuple(shift(v) for v in x.data))
    if n=="error":
        from ._backend import FxError
        need(len(a)==1 and isinstance(x,str),n)
        raise FxError("MATLAB_ERROR","error",x)
    return MISSING


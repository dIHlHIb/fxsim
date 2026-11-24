"""Exact stored-integer kernels for the recorded real fi arithmetic operations."""
from functools import lru_cache
import operator

from . import _pyfixed as fx


def conversion(source_fl, target, opts):
    signed, width, fl = target.SignednessBool, target.WordLength, target.FractionLength
    engine = fx._engine()
    low = -(1 << (width-1)) if signed else 0
    high = (1 << (width-int(signed)))-1
    mask = (1 << width)-1
    shift = fl-source_fl
    if opts["om"]=="Wrap":
        def limit(n):
            return ((n-low)&mask)+low
    else:
        def limit(n):
            return low if n<low else high if n>high else n
    if shift==0:
        return limit
    if shift>0:
        return lambda n:limit(n<<shift)
    if opts["rm"]=="Floor":
        return lambda n:limit(n>>(-shift))
    rnd=opts["rm"]
    return lambda n:limit(engine._round_div(n,-shift,rnd))


def identity(n):
    return n


def convert_type(source,target,opts):
    shift=target.FractionLength-source.FractionLength
    # A widening exact cast cannot change the stored code except to append zeros.
    source_signed=source.SignednessBool
    if shift>=0 and (target.SignednessBool or not source_signed) and (
        target.WordLength-int(target.SignednessBool)>=source.WordLength-int(source_signed)+shift
    ):
        return identity if shift==0 else lambda n:n<<shift
    return conversion(source.FractionLength,target,opts)


@lru_cache(maxsize=1024)
def plan(op, a, b, local, shift=0):
    fm=(local or fx._Math()).record()
    eng=fx._engine()
    if op in ("neg","abs","bitshift"):
        limit=conversion(a.FractionLength,a,fm)
        if op=="neg": return a,lambda x:limit(-x)
        if op=="abs": return a,lambda x:limit(abs(x))
        if shift<=0: return a,lambda x:x>>(-shift)
        return a,lambda x:limit(x<<shift)
    fa,fb=a.FractionLength,b.FractionLength
    signed=int(a.SignednessBool or b.SignednessBool)
    if op=="mul":
        w=a.WordLength+b.WordLength
        fl=fa+fb
        tw,tf=eng._mode_type(fm["pm"],fm["pwl"],fm["pfl"],w,fl)
        target=fx.numerictype(signed,tw,tf)
        if (w,fl)==(tw,tf):return target,operator.mul
        cast=conversion(fl,target,fm)
        return target,lambda x,y:cast(x*y)
    fn={"add":operator.add,"sub":operator.sub}[op]
    fl=max(fa,fb)
    full=max(a.WordLength-fa,b.WordLength-fb)+fl+1+int(a.SignednessBool!=b.SignednessBool)
    tw,tf=eng._mode_type(fm["sm"],fm["swl"],fm["sfl"],full,fl)
    target=fx.numerictype(signed,tw,tf)
    if fm["sm"]!="FullPrecision" and fm["cbs"]:
        left=convert_type(a,target,fm)
        right=convert_type(b,target,fm)
        limit=conversion(tf,target,fm)
        if left is identity and right is identity:
            if fm["om"]=="Wrap":
                lo=-(1<<(tw-1)) if signed else 0
                mask=(1<<tw)-1
                if op=="add":return target,lambda x,y:((x+y-lo)&mask)+lo
                return target,lambda x,y:((x-y-lo)&mask)+lo
            if op=="add":return target,lambda x,y:limit(x+y)
            return target,lambda x,y:limit(x-y)
        return target,lambda x,y:limit(fn(left(x),right(y)))
    cast=conversion(fl,target,fm)
    round_sig=identity
    if fm["sm"]!="FullPrecision":
        mw=max(a.WordLength,b.WordLength,fm["swl"])
        precision=53 if mw<=52 else max(mw,64)+1
        round_sig=lambda n:eng._round_sig(n,precision)
    return target,lambda x,y:cast(round_sig(fn(x<<(fl-fa),y<<(fl-fb))))

"""Double-precision helper operations for batch scripts."""
import bisect
import cmath
import itertools
import math
from dataclasses import replace

from ._marray import Array, LIMIT, integer, need


def axis(x, arg):
    dim = integer(arg) if arg is not None else (2 if x.rows == 1 else 1)
    need(dim in (1, 2), "dimension")
    return dim


def lanes(x, dim):
    if dim == 1:
        for col in range(x.cols):
            yield range(col * x.rows, (col + 1) * x.rows)
    else:
        for row in range(x.rows):
            yield range(row, len(x.data), x.rows)


def numeric(x, name):
    need(isinstance(x, Array) and x.kind == "double", name)
    return x


def vector(x, name):
    numeric(x, name)
    need(x.data and (x.rows == 1 or x.cols == 1), name)


def shaped(values, row, kind="double"):
    data = tuple(values)
    return Array(1, len(data), data, kind) if row else Array(len(data), 1, data, kind)


def log_value(name, v):
    if not isinstance(v,complex) and math.isnan(v):return math.nan
    if v == 0:
        return -math.inf
    if not isinstance(v, complex) and v >= 0:
        return getattr(math, name)(v)
    if name == "log2":
        return cmath.log(v) / math.log(2)
    return getattr(cmath, name)(v)


def builtin(name, a, outputs):
    x = a[0] if a else None
    if name in ("hypot","atan2"):
        need(len(a)==2 and outputs==1,name)
        y=a[1]
        numeric(x,name); numeric(y,name)
        need(not any(isinstance(v,complex) for z in a for v in z.data),name)
        need((x.rows,x.cols)==(y.rows,y.cols) or len(x.data)==1 or len(y.data)==1,name+" shape")
        shape=y if len(x.data)==1 else x
        def values(z):
            return itertools.repeat(z.data[0],len(shape.data)) if len(z.data)==1 else z.data
        return replace(shape,data=tuple(map(getattr(math,name),values(x),values(y))))
    if name=="bitsra" and isinstance(x,Array) and isinstance(x.kind,str) and x.kind.startswith(("uint","int")):
        need(len(a)==2 and outputs==1,name)
        k=integer(a[1])
        need(0<=k<=int(x.kind.lstrip("uint")),name)
        return replace(x,data=tuple(i>>k for i in x.data))
    if name in ("log", "log2", "log10"):
        need(len(a) == 1 and (outputs == 1 or name=="log2" and outputs==2), name)
        numeric(x, name)
        if outputs==2:
            need(all(isinstance(v,(float,int)) and math.isfinite(v) for v in x.data),name)
            pairs=tuple(map(math.frexp,x.data))
            return replace(x,data=tuple(v[0] for v in pairs)),replace(x,data=tuple(float(v[1]) for v in pairs))
        return replace(x, data=tuple(log_value(name, v) for v in x.data))
    if name == "find":
        need(1 <= len(a) <= 3 and outputs in (1, 2, 3), name)
        need(isinstance(x, Array) and (x.kind in ("logical", "double") or
             isinstance(x.kind, str) and x.kind.startswith(("int", "uint"))), name)
        k = integer(a[1]) if len(a) > 1 else len(x.data)
        direction = a[2] if len(a) == 3 else "first"
        need(k >= 0 and direction in ("first", "last"), name)
        indices = range(len(x.data))
        if direction == "last":
            indices = reversed(indices)
        hits = list(itertools.islice((i for i in indices if x.data[i]), min(k,len(x.data))))
        if direction == "last":
            hits.reverse()
        def pack(values, kind="double"):
            if not hits and ((x.rows, x.cols) in ((1, 1), (0, 0))):
                return Array(0, 0, (), kind)
            return shaped(values, x.rows == 1, kind)
        if outputs == 1:
            return pack(float(i + 1) for i in hits)
        rc = pack(float(i % x.rows + 1) for i in hits), pack(float(i // x.rows + 1) for i in hits)
        return rc if outputs == 2 else (*rc, pack((x.data[i] for i in hits), x.kind))
    if name == "mean" or name == "cumsum" and isinstance(x, Array) and x.kind == "double":
        need(len(a) in (1, 2) and outputs == 1, name)
        need(isinstance(x, Array) and x.kind in ("double", "logical"), name)
        if name == "mean" and len(a)==1 and (x.rows,x.cols)==(0,0):
            return Array(1,1,(math.nan,))
        dim = axis(x, a[1] if len(a) == 2 else None)
        if name == "mean":
            values = tuple(sum(x.data[i] for i in ix) / len(ix) if ix else math.nan for ix in lanes(x, dim))
            return Array(1, x.cols, values) if dim == 1 else Array(x.rows, 1, values)
        result = list(x.data)
        for ix in lanes(x, dim):
            total = 0.
            for i in ix:
                total += x.data[i]
                result[i] = total
        return replace(x, data=tuple(result))
    if name == "conv":
        need(len(a) in (2, 3) and outputs == 1, name)
        y = a[1]
        vector(x, name)
        vector(y, name)
        mode = a[2] if len(a) == 3 else "full"
        need(mode in ("full", "same", "valid"), name)
        n, m = len(x.data), len(y.data)
        need(n + m - 1 <= LIMIT, "convolution size")
        start, count = {"full": (0, n+m-1), "same": (m//2, n), "valid": (m-1,max(0,n-m+1))}[mode]
        def element(k):
            return sum(x.data[i]*y.data[k-i] for i in range(max(0,k-m+1), min(k+1,n)))
        return shaped((element(k) for k in range(start,start+count)),
                      (x.cols != 1 or y.cols != 1) if mode == "full" else x.rows == 1)
    if name == "filter":
        need(len(a) == 3 and outputs == 1, name)
        b, den, signal = a
        vector(b, name)
        vector(den, name)
        numeric(signal, name)
        need(den.data[0] != 0, name)
        length = max(len(b.data),len(den.data))
        def coefficients(v):
            return [c/den.data[0] for c in v.data] + [0.]*(length - len(v.data))
        bn, an = coefficients(b), coefficients(den)
        result = [0.] * len(signal.data)
        for ix in lanes(signal, axis(signal, None)):
            state = [0.] * length
            for i in ix:
                v = signal.data[i]
                y = bn[0]*v + state[0]
                for j in range(1,length):
                    state[j-1] = (bn[j]*v + state[j]) - an[j]*y
                result[i] = y
        return replace(signal,data=tuple(result))
    if name == "histc":
        need(len(a)==2 and outputs==1, name)
        vector(x,name)
        edges=a[1]
        vector(edges,name)
        need(all(not isinstance(v,complex) and not math.isnan(v) for v in edges.data) and
             all(b>a for a,b in zip(edges.data,edges.data[1:])),name)
        counts=[0.]*len(edges.data)
        for v in x.data:
            if edges.data[0] <= v <= edges.data[-1]:
                counts[bisect.bisect_right(edges.data,v)-1]+=1.
        return shaped(counts,x.rows==1)
    return None


def solve_small(a,b):
    """Solve well-conditioned real two-equation systems by partial pivoting."""
    numeric(a,"left division")
    numeric(b,"left division")
    need(a.rows==a.cols==b.rows==2 and b.cols>0,"left division dimensions")
    need(all(isinstance(v,(int,float)) and math.isfinite(v) for x in (a,b) for v in x.data),"left division inputs")
    p,q=(1,0) if abs(a.at(1,0))>abs(a.at(0,0)) else (0,1)
    u,v=a.at(p,0),a.at(p,1)
    threshold=max(map(abs,a.data))*math.sqrt(math.ulp(1.))
    need(abs(u)>threshold,"left division condition")
    multiplier=a.at(q,0)/u
    pivot=a.at(q,1)-multiplier*v
    need(math.isfinite(pivot) and abs(pivot)>threshold,"left division condition")
    answer=[]
    for j in range(b.cols):
        y=(b.at(q,j)-multiplier*b.at(p,j))/pivot
        x=(b.at(p,j)-v*y)/u
        need(math.isfinite(x) and math.isfinite(y),"left division result")
        answer.extend((x,y))
    return Array(2,b.cols,tuple(answer))

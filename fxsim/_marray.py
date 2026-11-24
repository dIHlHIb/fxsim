"""Two-dimensional MATLAB values. Data is in MATLAB linear (column-major) order."""
import math
import operator
from dataclasses import dataclass
from fractions import Fraction
from functools import cached_property

from ._backend import FxError


def need(test, where, message=None):
    if not test:
        raise FxError("UNSUPPORTED", where, message or f"{where} not supported by fxsim")


LIMIT = 2000000


class BlockData:
    """Immutable chunked storage for copy-on-write updates to large MATLAB arrays."""
    width=256

    def __init__(self,blocks,count):
        self.blocks=blocks
        self.count=count

    def __len__(self):
        return self.count

    def __iter__(self):
        return (v for block in self.blocks for v in block)

    def __getitem__(self,index):
        if isinstance(index,slice):
            return tuple(self[i] for i in range(*index.indices(self.count)))
        if index<0: index+=self.count
        return self.blocks[index//self.width][index%self.width]

    def __eq__(self,other):
        return len(self)==len(other) and all(x==y for x,y in zip(self,other))

    def __mul__(self,n):
        return tuple(self)*n

    @classmethod
    def update(cls,data,offsets,values):
        if len(data)<1024 or len(offsets)>len(data)//8:
            result=list(data)
            for i,v in zip(offsets,values): result[i]=v
            return tuple(result)
        blocks=list(data.blocks) if isinstance(data,cls) else [
            data[i:i+cls.width] for i in range(0,len(data),cls.width)]
        changed={}
        for i,v in zip(offsets,values):
            j,k=divmod(i,cls.width)
            if j not in changed: changed[j]=list(blocks[j])
            changed[j][k]=v
        for j,items in changed.items(): blocks[j]=tuple(items)
        return cls(tuple(blocks),len(data))


@dataclass(frozen=True)
class Array:
    rows: int
    cols: int
    data: tuple
    kind: object = "double"
    fimath: object = None

    def __post_init__(self):
        need(self.rows >= 0 and self.cols >= 0 and self.rows*self.cols == len(self.data)
             and len(self.data) <= LIMIT, "array dimensions")

    @cached_property
    def true_offsets(self):
        return tuple(i for i,v in enumerate(self.data) if v)

    def at(self, r, c):
        return self.data[c*self.rows+r]

    def transpose(self):
        return Array(self.cols, self.rows, tuple(self.at(i,j) for i in range(self.rows)
                                                for j in range(self.cols)), self.kind,self.fimath)


def scalar(x, kind="double"):
    return Array(1, 1, (float(x) if kind == "double" else x,), kind)


def number(x):
    need(isinstance(x, Array) and x.kind in ("double", "logical") and len(x.data)==1, "real scalar")
    return x.data[0]


def integer(x):
    v = number(x)
    need(math.isfinite(v) and int(v) == v, "integer argument")
    return int(v)


def grid(rows, cell=False):
    if not rows:
        return Array(0,0,(), "cell" if cell else "double")
    if cell:
        n = len(rows[0])
        need(all(len(r)==n for r in rows), "cell dimensions")
        return Array(len(rows), n, tuple(r[c] for c in range(n) for r in rows), "cell")
    if all(isinstance(v,str) for r in rows for v in r):
        need(len(rows)==1, "multirow character array")
        return "".join(rows[0])
    need(all(isinstance(v,Array) for r in rows for v in r), "mixed concatenation")
    from . import _marith as ar
    fixed=[v for row in rows for v in row if ar.isfi(v)]
    if fixed and any(not ar.isfi(v) for row in rows for v in row):
        t=fixed[0]
        need(all(len(r)==1 for r in rows) and all((ar.isfi(v) and v.kind==t.kind and v.fimath==t.fimath)
             or (v.kind=="double" and all(k==0 for k in v.data)) for r in rows for v in r),
             "mixed fi concatenation")
        rows=[[v if ar.isfi(v) else ar.cast(v,t.kind,t.fimath) for v in row] for row in rows]
    kind = rows[0][0].kind
    fm=rows[0][0].fimath
    need(all((v.kind,v.fimath)==(kind,fm) for r in rows for v in r), "mixed array classes or fimath")
    tiles = []
    for row in rows:
        h = row[0].rows
        need(all(v.rows == h for v in row), "horizontal concatenation")
        tiles.append(Array(h,sum(v.cols for v in row),tuple(x for v in row for x in v.data),kind,fm))
    w = tiles[0].cols
    need(all(v.cols==w for v in tiles), "vertical concatenation")
    return Array(sum(v.rows for v in tiles),w,
                 tuple(v.at(i,j) for j in range(w) for v in tiles for i in range(v.rows)),kind,fm)


def sequence(a, step, b):
    # Accept grids whose endpoints and increments share an exactly representable
    # dyadic lattice. Leave MATLAB's tolerance/endpoint compensation to native.
    first,delta,last=(Fraction.from_float(float(number(v))) for v in (a,step,b))
    unit=max(x.denominator for x in (first,delta,last))
    need(delta and max(abs(x*unit) for x in (first,delta,last))<=2**53
         and unit<=2**32, "colon range")
    count=max(0,int((last-first)//delta)+1)
    need(count<=LIMIT,"colon length")
    if count==0: return Array(1,0,())
    start=int(first*unit); stride=int(delta*unit)
    return Array(1,count,tuple((start+i*stride)/unit for i in range(count)))


def binary(op, a, b):
    need(isinstance(a,Array) and isinstance(b,Array) and a.kind in ("double","logical")
         and b.kind in ("double","logical"), op)
    comparison = op in ("==","~=","<",">","<=",">=")
    logical=op in ("&","|")
    need(comparison or logical or a.kind==b.kind=="double" or op==".*", op+" operand class")
    if op == "*" and len(a.data)!=1 and len(b.data)!=1:
        need(a.cols==b.rows and a.rows*b.cols*max(1,a.cols)<=LIMIT, "matrix product")
        return Array(a.rows,b.cols,tuple(sum(a.at(i,k)*b.at(k,j) for k in range(a.cols))
                                        for j in range(b.cols) for i in range(a.rows)))
    if op=="\\" or op=="/" and len(b.data)!=1:
        from ._mplain import solve_small
        if op=="/":return solve_small(b.transpose(),a.transpose()).transpose()
        if len(a.data)==1:return binary("/",b,a)
        return solve_small(a,b)
    if op in ("/","^"):
        need(len(b.data)==1 and (op!="/" or b.data[0]!=0), op)
        if op=="^":
            need(len(a.data)==1, "matrix power")
    if len(a.data)==1:
        rows,cols = b.rows,b.cols
        left,right = a.data*(rows*cols),b.data
    elif len(b.data)==1:
        rows,cols = a.rows,a.cols
        left,right = a.data,b.data*(rows*cols)
    else:
        if (a.rows,a.cols)==(b.rows,b.cols):
            rows,cols,left,right = a.rows,a.cols,a.data,b.data
        else:
            need(a.rows in (1,b.rows) or b.rows==1,"row expansion")
            need(a.cols in (1,b.cols) or b.cols==1,"column expansion")
            rows=a.rows if b.rows==1 else b.rows
            cols=a.cols if b.cols==1 else b.cols
            if rows*cols>LIMIT:
                from ._mouter import Outer
                return Outer.create(a,b,op)
            def expanded(v):
                return (v.at(0 if v.rows==1 else i,0 if v.cols==1 else j) for j in range(cols) for i in range(rows))
            left,right=expanded(a),expanded(b)
    fn = {"+":operator.add,"-":operator.sub,"*":operator.mul,".*":operator.mul,
          "/":operator.truediv,"./":operator.truediv,"^":operator.pow,".^":operator.pow,
          "&":lambda x,y:bool(x) and bool(y),"|":lambda x,y:bool(x) or bool(y),
          "==":operator.eq,"~=":operator.ne,"<":operator.lt,">":operator.gt,
          "<=":operator.le,">=":operator.ge}[op]
    try:
        values = tuple(map(fn,left,right))
        if not comparison and not logical:
            try: valid=all(map(math.isfinite,values))
            except TypeError: valid=all(isinstance(v,(int,float,complex)) and math.isfinite(v.real) and math.isfinite(v.imag) for v in values)
            need(valid,op+" nonfinite or complex")
    except ArithmeticError:
        need(False, op+" numeric domain")
    return Array(rows,cols,values,"logical" if comparison or logical else "double")


ALL = object()


def selection(a, indexes):
    def offsets(k, size):
        if k is ALL:
            return range(size)
        if isinstance(k,Array) and k.kind=="logical":
            need(len(k.data)==size,"logical index shape")
            return k.true_offsets
        need(isinstance(k,Array) and k.kind=="double" and (not k.data or len(indexes)==1 or k.rows==1 or k.cols==1)
             and all(float(x).is_integer() and 1<=x<=size for x in k.data), "array index")
        return [int(x)-1 for x in k.data]
    if len(indexes)==1:
        k = indexes[0]
        where = offsets(k,len(a.data))
        if k is ALL:
            h,w = len(where),1
        elif k.kind=="logical":
            h,w=(1,len(where)) if a.rows==1 and k.rows==1 else (len(where),1)
        elif len(a.data)!=1 and (a.rows==1 or a.cols==1) and (k.rows==1 or k.cols==1):
            h,w = (1,len(where)) if a.rows==1 else (len(where),1)
        else:
            h,w = k.rows,k.cols
        return h,w,where
    need(len(indexes)==2, "index rank")
    rs,cs = offsets(indexes[0],a.rows),offsets(indexes[1],a.cols)
    return len(rs),len(cs),[c*a.rows+r for c in cs for r in rs]


def index(a, indexes, braces=False):
    char = isinstance(a,str)
    if char:
        a = Array(1,len(a),tuple(a),"char")
    need(isinstance(a,Array), "index target")
    if len(indexes)==1 and indexes[0] is ALL and not braces and not char:
        return Array(len(a.data),1,a.data,a.kind,a.fimath)
    h,w,off = selection(a,indexes)
    if braces:
        need(a.kind=="cell" and len(off)==1, "cell contents")
        return a.data[off[0]]
    values = tuple(map(a.data.__getitem__,off))
    if char:
        need(h==1, "character index shape")
        return "".join(values)
    return Array(h,w,values,a.kind,a.fimath)


def typed_equal(a,b):
    # isequal allows different numeric classes when all corresponding values agree.
    if isinstance(a,Array) and isinstance(b,Array):
        if a.kind=="cell" or b.kind=="cell":
            return (a.kind==b.kind and (a.rows,a.cols)==(b.rows,b.cols)
                    and all(typed_equal(x,y) for x,y in zip(a.data,b.data)))
        if not isinstance(a.kind,str) or not isinstance(b.kind,str):
            need(a.kind==b.kind,"isequal mixed fi")
        return (a.rows,a.cols,a.data)==(b.rows,b.cols,b.data)
    return type(a)==type(b) and a==b

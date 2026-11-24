"""Large real row-by-column operations without an allocation per matrix entry."""
from dataclasses import dataclass
from itertools import repeat
import math
import operator
from ._marray import Array, LIMIT, need

OPS={"+":operator.add,"-":operator.sub,".*":operator.mul,"<":operator.lt,">":operator.gt,
     "<=":operator.le,">=":operator.ge,"==":operator.eq,"~=":operator.ne}


@dataclass(frozen=True)
class Outer:
    col: Array
    row: Array
    op: str
    reverse: bool

    @property
    def rows(self):return self.col.rows

    @property
    def cols(self):return self.row.cols

    @property
    def kind(self):return "double" if self.op in ("+","-",".*") else "logical"

    def at(self,i,j):
        x,y=self.col.data[i],self.row.data[j]
        return OPS[self.op](y,x) if self.reverse else OPS[self.op](x,y)

    def reduce(self,name,dim):
        need(dim in (1,2),name)
        axis=self.col if dim==1 else self.row
        across=self.row if dim==1 else self.col
        reverse=self.reverse if dim==1 else not self.reverse
        fn=OPS[self.op]
        def one(v):
            vals=map(fn,repeat(v),axis.data) if reverse else map(fn,axis.data,repeat(v))
            total=sum(vals,0.)
            return total/len(axis.data) if name=="mean" else total
        result=tuple(map(one,across.data))
        return Array(1,self.cols,result) if dim==1 else Array(self.rows,1,result)

    @classmethod
    def create(cls,a,b,op):
        need(op in OPS and a.kind==b.kind=="double","large implicit expansion")
        reverse=a.rows==1
        col,row=(b,a) if reverse else (a,b)
        need(col.cols==row.rows==1 and col.rows*row.cols<=128_000_000,"implicit expansion size")
        need(all(isinstance(v,(float,int)) and math.isfinite(v) for x in (a,b) for v in x.data),"large implicit expansion inputs")
        if op in ("+","-",".*"):
            need(all(math.isfinite(OPS[op](x,y)) for x in (min(a.data),max(a.data)) for y in (min(b.data),max(b.data))),"large implicit expansion range")
        return cls(col,row,op,reverse)

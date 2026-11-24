"""Lower bounded scalar numeric loops using parsed nodes and typed stored values.

All changes are local until the loop succeeds, including indexed and slice writes.
Only constant unit-stride subranges are accepted for vector shifts.
"""
import math
from . import _marith as ar, _mkernel as kernel, _pyfixed as fx
from ._marray import Array, LIMIT, scalar
from ._mloop import Lower, Outside, compiled, ensure, finite, offset, shift
from ._mhelpers import clamp_int


def integral(v):
    ensure(isinstance(v,(int,float)) and math.isfinite(v) and int(v)==v and abs(v)<=2**53)
    return int(v)


def modulus(a,b):
    ensure(a.is_integer() and b.is_integer() and abs(a)<=2**53 and abs(b)<=2**53 and b!=0)
    return a%b


def unit_range(a,b):
    a,b=integral(a),integral(b)
    ensure(0 < b-a+1 <= LIMIT)
    return map(float,range(a,b+1))


class Typed(Lower):
    def __init__(self, workspace, closure_type):
        super().__init__(workspace)
        self.closure_type=closure_type
        self.lexical=None

    def bind(self,value):
        k="helper"+str(len(self.args))
        self.args[k]=value
        return k

    def describe(self,name):
        if name not in self.kinds and ar.isfi(self.ws.get(name)):
            x=self.ws[name]
            ensure(not ar.complex_data(x))
            many=len(x.data)!=1
            self.kinds[name]=((x.kind,x.fimath),many)
            self.args[self.slot(name)]=x.data if many else x.data[0]
        return super().describe(name)

    def expr(self,e):
        tag=e[0]
        if self.lexical is not None and tag=="var":
            args,captured=self.lexical
            if e[1] in args:return args[e[1]]
            v=captured.get(e[1])
            ensure(isinstance(v,Array) and v.kind=="double" and len(v.data)==1)
            return repr(finite(v.data[0])),"double"
        if tag=="binary":
            op=e[1]
            l,a=self.expr(e[2])
            r,b=self.expr(e[3])
            if isinstance(a,tuple) or isinstance(b,tuple):
                ensure(isinstance(a,tuple) and isinstance(b,tuple) and op in ("+","-","*",".*"))
                ensure(a[1] is None or b[1] is None or a[1]==b[1])
                fm=a[1] or b[1]
                typ,fun=kernel.plan(ar.OPS[op],a[0],b[0],fm)
                return f"{self.bind(fun)}({l},{r})",(typ,fm)
            if op=="/":
                ensure(a==b=="double")
                return f"finite({l}/{r})","double"
            return self.binary_expr(op,e[2],e[3],l,a,r,b)
        if tag=="apply" and e[1][0]=="var" and e[2]=="(":
            name=e[1][1]
            args=e[3]
            obj=self.ws.get(name)
            if self.lexical is not None:
                ensure(name=="mod" and name not in self.lexical[1] and name not in self.lexical[0])
            elif isinstance(obj,self.closure_type):
                ensure(len(args)==len(obj.names))
                bound=dict(zip(obj.names,[self.expr(v) for v in args]))
                self.lexical=bound,obj.workspace
                try:return self.expr(obj.expr)
                finally:self.lexical=None
            if name=="mod" and name not in self.ws and name not in self.kinds and len(args)==2:
                (v,a),(w,b)=[self.expr(t) for t in args]
                ensure(a==b=="double")
                return f"mod({v},{w})","double"
            if name=="fi" and name not in self.ws and name not in self.kinds and len(args)==3:
                ensure(args[1][0]==args[2][0]=="var")
                typ=self.ws.get(args[1][1]);fm=self.ws.get(args[2][1])
                ensure(isinstance(typ,fx.numerictype) and isinstance(fm,fx._Math))
                ensure(args[1][1] not in self.changed and args[2][1] not in self.changed)
                v,kind=self.expr(args[0])
                if isinstance(kind,tuple):
                    cast=kernel.convert_type(kind[0],typ,ar.mathopts(fm))
                    return f"{self.bind(cast)}({v})",(typ,fm)
                ensure(args[0][0]=="number" and kind=="double")
                value=ar.cast(scalar(float(args[0][1])),typ,fm)
                return repr(value.data[0]),(typ,fm)
            if name=="double" and name not in self.ws and name not in self.kinds and len(args)==1:
                v,kind=self.expr(args[0])
                if isinstance(kind,tuple):
                    return f"ldexp({v},{-kind[0].FractionLength})","double"
                return f"float({v})","double"
        return super().expr(e)

    def span(self,name,e):
        x=self.ws.get(name)
        ensure(isinstance(x,Array) and len(x.data)>1 and (x.rows==1 or x.cols==1))
        ensure(e[0]=="range" and e[2][0]=="number" and float(e[2][1])==1)
        def bound(node):
            if node==("end",):return len(x.data)
            ensure(node[0]=="number")
            return integral(float(node[1]))
        first,last=bound(e[1]),bound(e[3])
        ensure(1<=first<=last<=len(x.data))
        return first-1,last

    def slice_write(self,dest,expr,pad):
        if dest[0]=="apply" and dest[1][0]=="var" and dest[2]=="(" and len(dest[3])==1 and dest[3][0][0]=="range":
            ensure(expr[0]=="apply" and expr[1][0]=="var" and expr[2]=="(" and len(expr[3])==1)
            lhs,rhs=dest[1][1],expr[1][1]
            ensure(self.describe(lhs)==self.describe(rhs))
            a,b=self.span(lhs,dest[3][0])
            c,d=self.span(rhs,expr[3][0])
            ensure(b-a==d-c)
            self.changed.add(lhs)
            self.lines.append(f"{pad}{self.slot(lhs)}[{a}:{b}]={self.slot(rhs)}[{c}:{d}]")
            return True
        if dest[0]=="var" and expr[0]=="grid" and not expr[2]:
            rows=expr[1]
            ensure(len(rows)==2 and all(len(row)==1 for row in rows))
            part=rows[0][0]
            name=dest[1]
            ensure(part[0]=="apply" and part[1]==dest and part[2]=="(" and len(part[3])==1)
            start,end=self.span(name,part[3][0])
            ensure(self.ws[name].cols==1 and end-start+1==self.ws[name].rows)
            val,kind=self.expr(rows[1][0])
            ensure(self.describe(name)==(kind,True))
            slot=self.slot(name)
            self.changed.add(name)
            self.lines.append(f"{pad}{slot}={slot}[{start}:{end}]+[{val}]")
            return True
        return False

    def body(self,body,indent=1):
        pad="    "*indent
        if not body:self.lines.append(pad+"pass")
        for step in body:
            _,kind,*items=step
            if kind=="for":
                var,expr,yes,_=items
                ensure(expr[0]=="range" and expr[2][0]=="number" and float(expr[2][1])==1)
                a,ak=self.expr(expr[1]); b,bk=self.expr(expr[3])
                ensure(ak==bk=="double")
                if var in self.ws or var in self.kinds:ensure(self.describe(var)==("double",False))
                self.kinds[var]=("double",False)
                self.changed.add(var)
                self.lines.append(f"{pad}for {self.slot(var)} in span({a},{b}):")
                self.body(yes,indent+1)
                continue
            if kind!="statement":
                super().body([step],indent)
                continue
            dest,expr,muted=items
            ensure(muted and dest)
            if self.slice_write(dest,expr,pad):continue
            rhs,t=self.expr(expr)
            if dest[0]=="var":
                name=dest[1]
                if name in self.ws or name in self.kinds:ensure(self.describe(name)==(t,False))
                self.kinds[name]=t,False
                lhs=self.slot(name)
            else:
                ensure(dest[0]=="apply" and dest[1][0]=="var" and dest[2]=="(")
                name=dest[1][1]; old,many=self.describe(name)
                if isinstance(old,tuple) and isinstance(t,tuple):
                    cast=kernel.convert_type(t[0],old[0],ar.mathopts(old[1]))
                    rhs=f"{self.bind(cast)}({rhs})"
                else:ensure(t==old)
                if many:lhs=f"{self.slot(name)}[{self.subscript(name,dest[3])}]"
                else:
                    ensure(dest[3]==[("all",)])
                    lhs=self.slot(name)
            self.changed.add(name)
            self.lines.append(f"{pad}{lhs}={rhs}")

    def run(self,var,values,body):
        ensure(values and var not in ("true","false"))
        self.kinds[var]=("double",False)
        self.changed.add(var)
        self.lines.append(f"for {self.slot(var)} in iterator:")
        self.body(body)
        env=dict(self.args,iterator=values)
        for name in self.changed:
            if self.kinds[name][1]:
                slot=self.slot(name)
                env[slot]=list(env[slot])
        helpers=dict(__builtins__={},off=offset,finite=finite,clip=clamp_int,shift=shift,
                     float=float,ldexp=math.ldexp,mod=modulus,span=unit_range)
        exec(compiled("\n".join(self.lines)),helpers,env)
        new={}
        for name in self.changed:
            slot=self.slot(name)
            if slot not in env:continue
            typ,many=self.kinds[name]
            k,fm=typ if isinstance(typ,tuple) else (typ,None)
            if many:
                x=self.ws[name]
                new[name]=Array(x.rows,x.cols,tuple(env[slot]),k,fm)
            else:new[name]=Array(1,1,(env[slot],),k,fm)
        self.ws.update(new)


def try_typed(workspace,var,values,body,closure_type):
    if len(values)<64:return False
    try:
        Typed(workspace,closure_type).run(var,values,body)
        return True
    except (Outside,KeyError,NameError,ValueError,TypeError,AttributeError,ArithmeticError):
        return False

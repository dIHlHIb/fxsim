"""Specialize long, scalar integer for-loops without changing MATLAB value semantics.

Only typed scalar assignments, reads and in-bounds writes into existing arrays,
and scalar if/else are lowered. Every numeric identifier in generated Python is
an internal slot, never user text. On a domain miss the interpreter re-executes
the untouched workspace with its normal source diagnostics.
"""
import math
import operator
from functools import lru_cache
from ._marray import Array,scalar
from ._mhelpers import INT,bounds,clamp_int


class Outside(Exception):
    pass


def ensure(test):
    if not test: raise Outside()


def offset(v,n):
    if not (1<=v<=n and int(v)==v): raise Outside()
    return int(v)-1


def shift(x,k,kind):
    lo,hi=bounds(kind)
    w=(hi-lo).bit_length()
    if int(k)!=k or abs(k)>w: raise Outside()
    if k<0:return x>>-int(k)
    return ((x<<int(k))-lo)%(hi-lo+1)+lo


def finite(v):
    if not math.isfinite(v):raise Outside()
    return v


@lru_cache(maxsize=64)
def compiled(source):
    return compile(source,"<fxsim scalar loop>","exec")


class Lower:
    def __init__(self,workspace):
        self.ws=workspace
        self.slots={}
        self.kinds={}
        self.args={}
        self.changed=set()
        self.lines=[]

    def slot(self,name):
        if name not in self.slots:self.slots[name]="v"+str(len(self.slots))
        return self.slots[name]

    def describe(self,name):
        if name in self.kinds:return self.kinds[name]
        x=self.ws.get(name)
        ensure(isinstance(x,Array) and isinstance(x.kind,str) and (x.kind in ("double","logical") or INT.fullmatch(x.kind)))
        if x.kind=="double":
            ensure(all(isinstance(v,(int,float)) and math.isfinite(v) for v in x.data))
        many=len(x.data)!=1
        self.kinds[name]=x.kind,many
        self.args[self.slot(name)]=x.data if many else x.data[0]
        return self.kinds[name]

    def expr(self,e):
        tag=e[0]
        if tag=="number":
            v=float(e[1]);ensure(math.isfinite(v));return repr(v),"double"
        if tag=="var":
            kind,many=self.describe(e[1]);ensure(not many)
            return self.slot(e[1]),kind
        if tag=="unary":
            arg,t=self.expr(e[2]);ensure(e[1] in ("-","+"))
            ensure(t!="logical")
            res=f"({e[1]}{arg})"
            return (f"clip({res},{t!r})" if INT.fullmatch(t) else res),t
        if tag=="binary":
            _,op,l,r=e
            x,a=self.expr(l);y,b=self.expr(r)
            return self.binary_expr(op,l,r,x,a,y,b)
        ensure(tag=="apply" and e[1][0]=="var" and e[2]=="(")
        name=e[1][1]; args=e[3]
        if name in self.ws or name in self.kinds:
            kind,many=self.describe(name);ensure(many)
            at=self.subscript(name,args);return f"{self.slot(name)}[{at}]",kind
        if name=="bitshift" and len(args)==2:
            x,t=self.expr(args[0]);k,u=self.expr(args[1]);ensure(INT.fullmatch(t) and u=="double")
            return f"shift({x},{k},{t!r})",t
        if INT.fullmatch(name) and len(args)==1:
            x,t=self.expr(args[0]);ensure(isinstance(t,str));return f"clip({x},{name!r})",name
        if name=="double" and len(args)==1:
            x,t=self.expr(args[0]);return f"float({x})","double"
        raise Outside()

    def binary_expr(self,op,l,r,x,a,y,b):
        if op in ("<","<=","==","~=",">=",">"):
            # Integer relational operations need matching classes, except for a scalar double.
            ensure(a==b or "double" in (a,b))
            sym="!=" if op=="~=" else op
            return f"({x}{sym}{y})","logical"
        if op=="^":
            ensure(a==b=="double")
            return f"finite({x}**{y})","double"
        ensure(op in ("+","-","*",".*") and a!="logical" and b!="logical")
        sym="*" if op==".*" else op
        if a==b=="double":return f"finite({x}{sym}{y})","double"
        if a!=b:
            # MATLAB permits integer arithmetic with scalar double. Only an
            # exact integral literal is specialized. All other cases fall back.
            if a=="double" and l[0]=="number" and float(l[1]).is_integer():x,a=repr(int(float(l[1]))),b
            elif b=="double" and r[0]=="number" and float(r[1]).is_integer():y,b=repr(int(float(r[1]))),a
        ensure(a==b and INT.fullmatch(a))
        return f"clip({x}{sym}{y},{a!r})",a

    def subscript(self,name,args):
        x=self.ws[name]
        if len(args)==1:
            expr,t=self.expr(args[0]);ensure(t=="double")
            return f"off({expr},{len(x.data)})"
        ensure(len(args)==2)
        r,rt=self.expr(args[0]);c,ct=self.expr(args[1]);ensure(rt==ct=="double")
        return f"(off({r},{x.rows})+{x.rows}*off({c},{x.cols}))"

    def body(self,body,indent=1):
        pad="    "*indent
        if not body:self.lines.append(pad+"pass")
        for _,kind,*items in body:
            if kind=="if":
                _,cond,yes,no=items
                code,t=self.expr(cond);ensure(t in ("logical","double"))
                self.lines.append(pad+"if "+code+":")
                self.body(yes,indent+1)
                if no:
                    self.lines.append(pad+"else:")
                    self.body(no,indent+1)
                continue
            ensure(kind=="statement")
            dest,expr,muted=items;ensure(dest and muted)
            rhs,t=self.expr(expr)
            if dest[0]=="var":
                name=dest[1]
                if name in self.ws or name in self.kinds:ensure(self.describe(name)==(t,False))
                self.kinds[name]=(t,False)
                self.changed.add(name);lhs=self.slot(name)
            else:
                ensure(dest[0]=="apply" and dest[1][0]=="var" and dest[2]=="(")
                name=dest[1][1];ensure(self.describe(name)==(t,True))
                self.changed.add(name);lhs=f"{self.slot(name)}[{self.subscript(name,dest[3])}]"
            self.lines.append(f"{pad}{lhs}={rhs}")

    def run(self,var,values,body):
        ensure(values and var not in ("true","false"))
        self.kinds[var]=("double",False)
        self.changed.add(var)
        self.lines.append(f"for {self.slot(var)} in iterator:")
        self.body(body)
        # Keep MATLAB copies independent even when two names share one Array.
        env=dict(self.args,iterator=values)
        for k in self.changed:
            if self.kinds[k][1]:env[self.slot(k)]=list(env[self.slot(k)])
        code=compiled("\n".join(self.lines))
        exec(code,{"__builtins__":{},"clip":clamp_int,"off":offset,"shift":shift,"float":float,"finite":finite},env)
        outputs={}
        for name in self.changed:
            label=self.slot(name)
            if label not in env:continue
            kind,many=self.kinds[name]
            if many:
                x=self.ws[name]
                outputs[name]=Array(x.rows,x.cols,tuple(env[label]),kind)
            else: outputs[name]=scalar(env[label],kind)
        self.ws.update(outputs)


def try_loop(workspace,var,values,body):
    if len(values)<256:return False
    try:
        Lower(workspace).run(var,values,body)
    except (Outside,KeyError,NameError,ValueError,TypeError,ArithmeticError):
        return False
    return True


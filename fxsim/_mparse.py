"""Lexer and recursive descent for the batch subset. Never evaluate source as Python."""
from dataclasses import dataclass
import re

from ._backend import FxError

IDENT = re.compile(r"[A-Za-z][A-Za-z0-9_]*")
NUM = re.compile(r"(?:[0-9]+(?:\.(?![*/^'])[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?")
OPS = ("...","==","~=","<=",">=",".*","./",".^",".'","&&","||")
P = {"||":-4,"&&":-3,"|":-2,"&":-1,"==":1,"~=":1,"<=":1,">=":1,"<":1,">":1,":":2,"+":3,"-":3,"*":4,"/":4,"\\":4,".*":4,"./":4,"^":6,".^":6}
KEYWORDS = set("function if for end else elseif while try catch switch case otherwise return break continue global persistent classdef parfor spmd arguments".split())


@dataclass(frozen=True)
class Token:
    s: str
    line: int
    gap: bool


def reject(t, reason=None):
    raise FxError("UNSUPPORTED",t.s,reason or f"{t.s} not supported by fxsim",line=t.line)


def lex(text):
    pos,line,gap,prev = 0,1,False,""
    while pos < len(text):
        c=text[pos]
        if c in " \t\r":
            pos+=1
            gap=True
            continue
        if text.startswith(("%{","%}"),pos):
            reject(Token(text[pos:pos+2],line,gap))
        if c=="%" or text.startswith("...",pos):
            cont = c!="%"
            end=text.find("\n",pos)
            pos=len(text) if end==-1 else end+int(cont)
            line+=int(cont and end!=-1)
            gap=True
            continue
        at=pos
        transpose = c=="'" and not gap and (prev in (")","]","}","'",".'") or IDENT.fullmatch(prev) or NUM.fullmatch(prev))
        if c=="'" and not transpose:
            pos+=1
            while pos<len(text) and text[pos]!="\n":
                pos+=1
                if text[pos-1]=="'":
                    if text[pos:pos+1]!="'":
                        break
                    pos+=1
            else:
                reject(Token("character literal",line,gap),"Unterminated character literal")
        elif (m:=IDENT.match(text,pos)) or (m:=NUM.match(text,pos)):
            pos=m.end()
        else:
            op=next((o for o in OPS if text.startswith(o,pos)),None)
            pos+=len(op) if op else 1
        prev=text[at:pos]
        yield Token(prev,line,gap)
        if c=="\n":
            line+=1
        gap=False
    yield Token("<EOF>",line,gap)


class Parser:
    def __init__(self,text):
        self.tokens=list(lex(text))
        self.i=0

    @property
    def t(self):
        return self.tokens[self.i]

    def pop(self,expected=None):
        t=self.t
        if expected is not None and t.s!=expected:
            reject(t,f"Expected {expected!r}, found {t.s!r}")
        self.i+=1
        return t.s

    def name(self):
        if not IDENT.fullmatch(self.t.s) or self.t.s in KEYWORDS:
            reject(self.t)
        return self.pop()

    def expr(self,prec=-4,in_grid=False):
        s=self.t.s
        if s=="@":
            self.pop()
            node=("lambda",self.arguments(),self.expr(in_grid=in_grid))
        elif s in ("-","+","~") and not (s=="~" and self.tokens[self.i+1].s in (",","]")):
            self.pop()
            node=("unary",s,self.expr(5,in_grid))
        elif s=="(":
            self.pop()
            node=self.expr()
            self.pop(")")
        elif s in ("[","{"):
            node=self.grid()
        elif s.startswith("'") and len(s)>1:
            node=("char",self.pop()[1:-1].replace("''","'"))
        elif s=="~" and self.tokens[self.i+1].s in (",","]"):
            self.pop()
            node=("discard",)
        elif s=="end":
            self.pop()
            node=("end",)
        elif NUM.fullmatch(s):
            node=("number",self.pop())
            if not self.t.gap and self.t.s in ("i","j"):
                self.pop()
                node=("imag",node)
        else:
            node=("var",self.name())
        while True:
            s=self.t.s
            if in_grid and self.t.gap:
                if s in ("+","-") and not self.tokens[self.i+1].gap:
                    return node
                if s in ("(","{"):
                    return node
            if s==".":
                self.pop()
                node=("field",node,self.name())
            elif s in ("(","{"):
                self.pop()
                end=")" if s=="(" else "}"
                args=[]
                if self.t.s!=end:
                    while True:
                        if self.t.s==":":
                            self.pop()
                            args.append(("all",))
                        else:
                            args.append(self.expr())
                        if self.t.s!=",":
                            break
                        self.pop()
                self.pop(end)
                node=("apply",node,s,args)
            elif s in ("'",".'"):
                op=self.pop()
                node=("transpose",node,op=="'")
            elif s in P and P[s]>=prec:
                self.pop()
                rhs=self.expr(P[s]+1,in_grid)
                if s==":":
                    step=("number","1")
                    if self.t.s==":":
                        self.pop()
                        step,rhs=rhs,self.expr(P[s]+1,in_grid)
                    node=("range",node,step,rhs)
                else:
                    node=("binary",s,node,rhs)
            else:
                return node

    def grid(self):
        first=self.pop()
        last="]" if first=="[" else "}"
        rows=[]
        while self.t.s!=last:
            if self.t.s=="\n":
                self.pop()
                continue
            row=[self.expr(in_grid=True)]
            while self.t.s not in (last,";","\n"):
                if self.t.s==",":
                    self.pop()
                elif not self.t.gap:
                    reject(self.t,"Expected an array element separator")
                row.append(self.expr(in_grid=True))
            rows.append(row)
            if self.t.s!=last:
                self.pop()
        self.pop(last)
        return ("grid",rows,first=="{")

    def separator(self):
        if self.t.s not in (",",";","\n","<EOF>"):
            reject(self.t)
        if self.t.s=="<EOF>":
            return False
        return self.pop()==";"

    def block(self,nested=False,script=False):
        statements=[]
        while self.t.s!="<EOF>":
            t=self.t
            if t.s in (";",",","\n"):
                self.pop()
                continue
            if script and t.s=="function":
                break
            if nested and t.s in ("else","elseif","end","case","otherwise","function"):
                break
            if t.s in ("for","while","if"):
                kind=self.pop()
                var=None
                if kind=="for":
                    var=self.name()
                    self.pop("=")
                branches=[]
                cond=self.expr()
                self.separator()
                body=self.block(True)
                while kind=="if" and self.t.s=="elseif":
                    self.pop()
                    test=self.expr()
                    self.separator()
                    branches.append((test,self.block(True)))
                other=[]
                if kind=="if" and self.t.s=="else":
                    self.pop()
                    self.separator()
                    other=self.block(True)
                self.pop("end")
                self.separator()
                for test,branch in reversed(branches):
                    other=[(t.line,"if",None,test,branch,other)]
                statements.append((t.line,kind,var,cond,body,other))
                continue
            if t.s=="switch":
                self.pop()
                expr=self.expr()
                self.separator()
                cases=[]
                while self.t.s in (";","\n"): self.pop()
                while self.t.s=="case":
                    self.pop()
                    label=self.expr()
                    self.separator()
                    cases.append((label,self.block(True)))
                fallback=[]
                if self.t.s=="otherwise":
                    self.pop()
                    self.separator()
                    fallback=self.block(True)
                self.pop("end")
                self.separator()
                statements.append((t.line,"switch",expr,cases,fallback))
                continue
            if t.s in ("return","break","continue"):
                kind=self.pop()
                self.separator()
                statements.append((t.line,kind))
                continue
            if t.s=="persistent":
                self.pop()
                names=[self.name()]
                while IDENT.fullmatch(self.t.s):
                    names.append(self.name())
                self.separator()
                statements.append((t.line,"persistent",names))
                continue
            # Support bdclose all, a common command-form close.
            if t.s in ("cd","addpath") and self.tokens[self.i+1].gap and self.tokens[self.i+1].s not in ("(","=",";","\n",",","<EOF>"):
                command=self.pop()
                pieces=[]
                while self.t.s not in (";","\n",",","<EOF>"):
                    if self.t.gap and pieces:
                        reject(self.t,"Use parentheses for paths containing spaces or multiple arguments")
                    pieces.append(self.pop())
                path="".join(pieces)
                if path.startswith("'") and path.endswith("'"):
                    path=path[1:-1].replace("''","'")
                node=("apply",("var",command),"(",[("char",path)])
            elif t.s=="bdclose" and self.tokens[self.i+1].s=="all":
                self.pop()
                self.pop()
                node=("apply",("var","bdclose"),"(",[("char","all")])
            else:
                node=self.expr()
            dest=None
            if self.t.s=="=":
                self.pop()
                dest,node=node,self.expr()
            muted=self.separator()
            statements.append((t.line,"statement",dest,node,muted))
        return statements

    def arguments(self):
        self.pop("(")
        args=[]
        if self.t.s!=")":
            args.append(self.name())
            while self.t.s==",":
                self.pop()
                args.append(self.name())
        self.pop(")")
        return args

    def functions(self):
        while self.t.s in ("\n",";"):
            self.pop()
        result={}
        script=self.t.s!="function"
        main=self.block(script=True) if script else None
        while self.t.s=="function":
            self.pop()
            if self.t.s=="[":
                self.pop()
                outputs=[self.name()]
                while self.t.s!="]":
                    if self.t.s==",": self.pop()
                    outputs.append(self.name())
                self.pop("]")
                self.pop("=")
                name=self.name()
            else:
                name=self.name()
                outputs=[]
                if self.t.s=="=":
                    self.pop()
                    outputs,name=[name],self.name()
            args=self.arguments() if self.t.s=="(" else []
            self.separator()
            body=self.block(True)
            if script and self.t.s!="end":reject(self.t,"A local function in a script requires end")
            if self.t.s=="end":
                self.pop()
                self.separator()
            while self.t.s in (";","\n"):
                self.pop()
            if name in result:
                reject(self.t,"Duplicate function definition")
            result[name]=((name,args,outputs),body)
        if self.t.s!="<EOF>":
            reject(self.t)
        if script:
            return None,main,result
        header,body=next(iter(result.values()))
        return header,body,result

    def program(self):
        header,body,_=self.functions()
        return header,body

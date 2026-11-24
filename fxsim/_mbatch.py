"""Command-line entry point for the supported MATLAB batch language."""
import json
import math
import os
import re
import sys
import zipfile
from dataclasses import dataclass, replace
from functools import lru_cache
import time
from itertools import repeat
from pathlib import Path

from . import _mio, _mplain, _mloop, _mtypedloop
from . import _marith as ar, _pyfixed as fx, _mhelpers as helpers
from . import _fixed as fixed, _model as api, _msave
from ._backend import FxError, core, options, root
from ._marray import BlockData, ALL, LIMIT, Array, binary, grid, index, integer, need, number, scalar, selection, sequence, typed_equal
from ._mbuild import Dataset, Port, Series, SimOutput, State
from ._mouter import Outer
from ._mparse import IDENT, Parser

VOID=object()


@dataclass(frozen=True)
class Handle:
    path: str


@dataclass(frozen=True)
class Fixdt:
    numeric_type: fixed.numerictype


def arity(args,n,name):
    need(len(args)==n,name,f"Expected {n} arguments, got {len(args)}")


def real_data(x):
    need(isinstance(x,Array) and x.kind!="cell","numeric data")
    if isinstance(x.kind,fixed.numerictype):
        f=-x.kind.FractionLength
        if not ar.complex_data(x):return tuple(map(math.ldexp,x.data,repeat(f)))
        return tuple(complex(math.ldexp(v.real,f),math.ldexp(v.imag,f)) for v in x.data)
    return tuple(v if isinstance(v,complex) else float(v) for v in x.data)


class Flow(Exception):
    def __init__(self,kind):
        self.kind=kind


@dataclass
class Closure:
    names: list
    expr: tuple
    workspace: dict


@lru_cache(maxsize=64)
def parse_file(path,source):
    return Parser(source).functions()


def fmt_values(fmt,args):
    need(isinstance(fmt,str),"format string")
    fmt=re.sub(r"\\([nrt\\])",lambda m:{"n":"\n","r":"\r","t":"\t","\\":"\\"}[m[1]],fmt)
    need(not re.search(r"\\[^nrt\\]",fmt),"format escape")
    vals=[]
    for a in args:
        if isinstance(a,str):
            vals.append(a)
        elif isinstance(a,Array) and isinstance(a.kind,str) and re.fullmatch("u?int(8|16|32|64)",a.kind):
            vals.extend(a.data)
        else:
            vals.extend(real_data(a))
    fields=re.findall(r"%(?:%|[-+ 0]*\d*(?:\.\d+)?[difsge])",fmt)
    need(len(fields)==fmt.count("%")-fields.count("%%"),"format conversion")
    width=sum(f!="%%" for f in fields)
    need((not vals and width==0) or width and len(vals)%width==0,"format arity")
    out=[]
    for start in range(0,len(vals) or 1,width or 1):
        v=vals[start:start+width]
        for form,x in zip((f for f in fields if f!="%%"),v):
            need(isinstance(x,str) if form.endswith("s") else isinstance(x,(int,float)), "format argument")
            if form[-1] in "di":
                need(x==int(x),"integer format")
        out.append(fmt%tuple(v))
    return "".join(out)


class Session:
    def __init__(self):
        self.workspace={}
        self.models={}
        self.libraries=set()
        self.files=[]
        self.index_end=[]
        self.locals=[]
        self.persist={}
        self.handles={}
        self.search_path=[]
        self.last_tic=None
        self.timer_ids=set()
        self.directory_files={}
        self.resolved_names={}
        self.requested=1

    def locate(self,name):
        if name in self.resolved_names:
            return self.resolved_names[name]
        for directory in [Path.cwd(),*self.search_path]:
            if directory not in self.directory_files:
                self.directory_files[directory]={p.stem:p for p in directory.glob("*.m") if p.is_file()}
            if name in self.directory_files[directory]:
                self.resolved_names[name]=self.directory_files[directory][name]
                return self.resolved_names[name]
        self.resolved_names[name]=None
        return None

    def state(self,path):
        if isinstance(path,(Handle,Port)):
            path=path.path
        need(isinstance(path,str),"model path")
        name=path.split("/")[0]
        if name not in self.models:
            raise FxError("NO_MODEL",name,"Model is not loaded")
        return self.models[name]

    def read_model(self,arg):
        need(isinstance(arg,str),"load_system name")
        if arg in {path.split("/")[0] for path in api.CATALOG}:
            self.libraries.add(arg)
            return Handle(arg)
        if arg in self.models:
            return Handle(arg)
        path=Path(arg)
        if path.suffix=="":
            path=path.with_suffix(".slx")
        need(path.suffix==".slx","load_system file type")
        path=path.resolve()
        spec=_msave.load(path)
        name=path.stem
        if name in self.models:
            need(self.models[name].path==path,"model name conflict")
            return Handle(name)
        self.call("new_system",[name])
        s=self.models[name]
        try:
            s.configure(spec["settings"])
            for n,b in spec["blocks"].items():
                opts=dict(b["params"])
                if "Position" in opts:
                    opts["Position"]=Array(1,4,tuple(map(float,opts["Position"])))
                s.edit(name+"/"+n,opts,b["source"])
            for e in spec["lines"]:
                api.add_line(name,f"{e['src']}/{e['sport']}",f"{e['dst']}/{e['dport']}")
            s.logs=spec["logs"]
            s.path,s.dirty=path,False
        except BaseException:
            api.close_system(name)
            del self.models[name]
            raise
        return Handle(name)

    def file(self,path,args,script=False):
        path=Path(path).resolve()
        need(path.suffix==".m" and path.is_file() and len(self.files)<32,"run",f"Cannot load script or function {path.name}")
        before=Path.cwd()
        self.files.append(path)
        old=self.workspace
        try:
            header,body,functions=parse_file(path,path.read_text())
            self.locals.append(functions)
            if script:
                need(header is None,"run","A function file requires a function call")
                need(path.stem not in functions,"run","A script cannot define a local function with its own name")
                os.chdir(path.parent)
                self.resolved_names.clear()
                self.execute(body)
                return VOID
            need(header and header[0]==path.stem,path.stem,"Function/file name mismatch")
            return self.invoke(header,body,args)
        except FxError as e:
            e.detail.setdefault("file",path.name)
            raise
        finally:
            self.workspace=old
            if len(self.locals)==len(self.files):
                self.locals.pop()
            self.files.pop()
            if script and Path.cwd()==path.parent:
                os.chdir(before)
                self.resolved_names.clear()

    def invoke(self,header,body,args):
        name,params,outputs=header
        need(len(args)<=len(params),name,"Too many input arguments")
        before=self.workspace
        requested=self.requested
        self.requested=1
        prev_keep=getattr(self,"active_keep",None)
        keep=self.persist.setdefault((self.files[-1],name),{})
        self.active_keep=keep
        self.workspace=dict(zip(params,args),nargin=scalar(len(args)),nargout=scalar(requested))
        self.workspace.update(keep)
        try:
            try:
                self.execute(body)
            except Flow as e:
                need(e.kind=="return",e.kind,"Loop control outside a loop")
            chosen=outputs[:max(requested,1)]
            need(requested<=max(1,len(outputs)),name,"Too many output arguments")
            need(all(k in self.workspace for k in chosen),name,"Function output unassigned")
            values=[self.workspace[k] for k in chosen]
            return tuple(values) if requested>1 else values[0] if values else VOID
        finally:
            for k in keep:
                if k in self.workspace: keep[k]=self.workspace[k]
            self.workspace=before
            self.requested=requested
            self.active_keep=prev_keep

    def call(self,name,a):
        try:
            return self.dispatch(name,a)
        except FxError as e:
            e.detail.setdefault("operation",name)
            raise
        except (ValueError,TypeError,AttributeError,IndexError,KeyError,OverflowError,OSError,zipfile.BadZipFile) as e:
            err=FxError("UNSUPPORTED",name,str(e) or f"Invalid arguments for {name}",operation=name)
            raise err from e

    def dispatch(self,n,a):
        if self.locals and n in self.locals[-1]:
            return self.invoke(*self.locals[-1][n],a)
        if n!="run" and IDENT.fullmatch(n):
            found=self.locate(n)
            if found is not None:
                return self.file(found,a)
        if n in ("tic","toc"):
            now=time.perf_counter_ns()
            if n=="tic":
                arity(a,0,n)
                self.timer_ids.add(now)
                return scalar(now,"uint64")
            need(len(a)<=1,n)
            if a:
                token=a[0]
                need(isinstance(token,Array) and token.kind=="uint64" and len(token.data)==1 and token.data[0] in self.timer_ids,n)
                start=token.data[0]
            else:
                start=self.last_tic
            need(start is not None,n)
            return scalar((now-start)*1e-9)
        if n=="cd":
            need(len(a)<=1 and (not a or isinstance(a[0],str)),n)
            before=os.getcwd()
            if a:
                os.chdir(Path(a[0]).expanduser())
                self.resolved_names.clear()
            return before
        if n=="getenv":
            need(len(a)==1 and isinstance(a[0],str),n)
            return os.environ.get(a[0],"")
        if n=="addpath":
            need(a and all(isinstance(v,str) for v in a),n)
            last=a[-1] in ("-end","-begin")
            paths=a[:-1] if last else a
            dirs=list(dict.fromkeys(Path(d).expanduser().resolve() for v in paths for d in v.split(os.pathsep)))
            need(dirs and all(d.is_dir() for d in dirs),n)
            previous=[d for d in self.search_path if d not in dirs]
            oldpath=os.pathsep.join(map(str,self.search_path))
            self.resolved_names.clear()
            self.search_path=previous+dirs if a[-1]=="-end" else dirs+previous
            return oldpath
        if a and isinstance(a[0],Outer):
            x=a[0]
            if n in ("sum","mean"):
                need(len(a) in (1,2) and self.requested==1,n)
                return x.reduce(n,integer(a[1]) if len(a)==2 else 1)
            if n=="size" and len(a)==1 and self.requested==2:
                return scalar(x.rows),scalar(x.cols)
            if n=="size" and len(a)==1: return Array(1,2,(float(x.rows),float(x.cols)))
            if n=="size" and len(a)==2:
                d=integer(a[1])
                need(d>=1,n)
                return scalar(x.rows if d==1 else x.cols if d==2 else 1)
            if n=="numel" and len(a)==1:return scalar(x.rows*x.cols)
            if n=="length" and len(a)==1:return scalar(max(x.rows,x.cols))
            if n=="class" and len(a)==1:return x.kind
            need(False,"large implicit expansion "+n)
        value=_mplain.builtin(n,a,self.requested)
        if value is not None: return value
        if n=="min" and self.requested==2 and len(a)==1:
            x=a[0]
            need(isinstance(x,Array) and x.kind=="double" and x.data and (x.rows==1 or x.cols==1),n)
            val=min(x.data)
            return scalar(val),scalar(x.data.index(val)+1)
        if n=="deal":
            need(len(a)==self.requested,n)
            return tuple(a) if self.requested>1 else a[0]
        if n=="size" and len(a)==1 and self.requested==2 and isinstance(a[0],Array):
            return scalar(a[0].rows),scalar(a[0].cols)
        if n in ("fi","fimath","numerictype"):
            if n=="fi": return ar.construct(a)
            if n=="fimath":
                if len(a)==1 and ar.isfi(a[0]): return a[0].fimath or fx._Math()
                return ar.make_fimath(a)
            if len(a)==1 and ar.isfi(a[0]): return a[0].kind
            arity(a,3,n)
            return fx.numerictype(*map(integer,a))
        if n in ("setfimath","removefimath"):
            need(len(a)==(2 if n=="setfimath" else 1) and ar.isfi(a[0]),n)
            need(n!="setfimath" or isinstance(a[1],fx._Math),n)
            return replace(a[0],fimath=a[1] if len(a)==2 else None)
        if n=="isfimathlocal":
            need(len(a)==1 and ar.isfi(a[0]),n)
            return scalar(a[0].fimath is not None,"logical")
        if n=="complex":
            need(len(a)==2,n)
            return ar.combine(*a)
        if n=="typecast":
            arity(a,2,n)
            return _mio.typecast(*a)
        if n=="reinterpretcast":
            if a and isinstance(a[0],Array) and isinstance(a[0].kind,str) and ar.INTEGER.fullmatch(a[0].kind):
                a=[ar.construct([a[0]]),*a[1:]]
            need(len(a)==2 and ar.isfi(a[0]) and isinstance(a[1],fx.numerictype)
                 and a[0].kind.WordLength==a[1].WordLength,n)
            t=a[1]
            need(not ar.complex_data(a[0]),n+" complex")
            vals=tuple(fx._engine()._overflow(v,t.SignednessBool,t.WordLength,"Wrap") for v in a[0].data)
            return replace(a[0],kind=t,data=vals)
        if n=="divide":
            need(len(a)==3 and isinstance(a[0],fx.numerictype),n)
            return ar.element("divide",a[1],a[2],t=a[0].record())
        if n in ("abs","bitshift","bitsra","bitsll") and a and ar.isfi(a[0]):
            if n=="abs":
                arity(a,1,n)
                return ar.element("abs",a[0])
            arity(a,2,n)
            k=integer(a[1])
            need(-128<=k<=128 and (n=="bitshift" or k>=0),n)
            if n=="bitsll":
                t=a[0].kind
                data=tuple(fx._engine()._overflow(v<<k,t.SignednessBool,t.WordLength,"Wrap") for v in a[0].data)
                return replace(a[0],data=data)
            return ar.element("bitshift",a[0],k= -k if n=="bitsra" else k)
        if n in ("reshape","isreal","class","iscell","isfi"):
            need(a and isinstance(a[0],Array),n)
            x=a[0]
            if n=="reshape":
                need(len(a)==3,n)
                h,w=helpers.dims(a[1:])
                return replace(x,rows=h,cols=w)
            arity(a,1,n)
            if n=="isreal": return scalar(not ar.complex_data(x),"logical")
            if n=="class": return "embedded.fi" if ar.isfi(x) else x.kind
            return scalar({"isfi":ar.isfi(x),"iscell":x.kind=="cell","isreal":all(not isinstance(v,complex) for v in x.data)}[n],"logical")
        if n in ("fopen","fclose","fread","fwrite"):
            value=_mio.io_call(self.handles,n,a,self.requested)
            if n=="fclose":
                self.resolved_names.clear()
                self.directory_files.clear()
            return value
        value=helpers.builtin(n,a)
        if value is not helpers.MISSING:
            return value
        if n=="run":
            arity(a,1,n)
            need(isinstance(a[0],str),n)
            path=Path(a[0])
            if not path.suffix: path=path.with_suffix(".m")
            if path.parent==Path(".") and not path.is_file() and path.suffix==".m":
                path=self.locate(path.stem) or path
            return self.file(path,[],True)
        if n in ("new_system","open_system","load_system","bdIsLoaded"):
            arity(a,1,n)
            if n=="new_system":
                need(a[0] not in {p.split("/")[0] for p in api.CATALOG},"new_system library name")
                name=api.new_system(a[0])
                self.models[name]=State(name)
                return Handle(name)
            if n=="bdIsLoaded":
                return scalar(a[0] in self.models or a[0] in self.libraries,"logical")
            return self.read_model(a[0])
        if n in ("close_system","bdclose"):
            if n=="bdclose" and a==["all"]:
                self.libraries.clear()
                for m in list(self.models):
                    self.call("close_system",[m,scalar(0)])
                return VOID
            need(len(a) in (1,2) if n=="close_system" else len(a)==1,n)
            if a[0] in self.libraries:
                need(len(a)==1 or len(a)==2 and integer(a[1])==0,n)
                self.libraries.remove(a[0])
                return VOID
            s=self.state(a[0])
            if len(a)==2:
                flag=integer(a[1])
                need(flag in (0,1),n)
                if flag:
                    self.call("save_system",[s.name])
            elif n=="close_system":
                need(not s.dirty,n,"Closing an unsaved model requires close_system(model,0) or save_system")
            api.close_system(s.name)
            del self.models[s.name]
            return VOID
        if n=="add_block":
            need(len(a)>=2,n)
            src,dest=a[:2]
            need(isinstance(dest,str) and isinstance(src,str),"add_block paths")
            s=self.state(dest)
            need(dest.partition("/")[2] not in s.blocks,dest,"Block already exists")
            s.edit(dest,options(a[2:],{},n),src)
            self.libraries.add(src.split("/")[0])
            return Handle(dest)
        if n in ("get_param","set_param"):
            need(len(a)>=2,n)
            p=a[0]
            if isinstance(p,str) and p in api.CATALOG and n=="get_param":
                arity(a,2,n)
                self.libraries.add(p.split("/")[0])
                defaults=api.DEFAULTS.get(api.CATALOG[p]["kind"],{})
                need(a[1] in defaults,p+"."+a[1])
                return defaults[a[1]]
            s=self.state(p)
            if isinstance(p,Port):
                need(n=="set_param",n)
                s.log(p,options(a[1:],{},n))
                return VOID
            p=p.path if isinstance(p,Handle) else p
            if n=="get_param":
                arity(a,2,n)
                return s.get(p,a[1])
            opts=options(a[1:],{},n)
            if p==s.name:
                s.configure(opts)
            else:
                s.edit(p,opts)
            return VOID
        if n in ("add_line","delete_line"):
            need(len(a)>=3,n)
            m=a[0]
            s=self.state(m)
            if n=="add_line":
                s.connect(a[1],a[2],a[3:])
            else:
                arity(a,3,n)
                lines=api.model(m).lines
                hits=[l for l in lines if (f"{l['src']}/{l['sport']}",f"{l['dst']}/{l['dport']}")==tuple(a[1:])]
                need(len(hits)==1,n,"Line does not exist")
                lines.remove(hits[0])
            s.dirty=True
            return VOID
        if n=="save_system":
            need(len(a) in (1,2),n)
            s=self.state(a[0])
            path=Path(a[1]) if len(a)==2 else s.path or Path(s.name+".slx")
            if not path.suffix:
                path=path.with_suffix(".slx")
            _msave.save(s,path.absolute(),self.workspace)
            return VOID
        if n=="sim":
            need(a,n)
            s=self.state(a[0])
            opts=options(a[1:],{},n)
            old,dirty=s.settings,s.dirty
            try:
                s.settings=dict(old)
                s.configure(opts)
                return s.simulate(self.workspace)
            finally:
                s.settings,s.dirty=old,dirty
        if n=="Simulink.SimulationData.Dataset":
            arity(a,0,n)
            return Dataset()
        if n=="timeseries":
            arity(a,2,n)
            x,t=a
            need(isinstance(x,Array) and isinstance(t,Array) and t.kind=="double" and
                 (x.rows==1 or x.cols==1) and (t.rows==1 or t.cols==1) and len(x.data)==len(t.data),n)
            return Series(Array(len(x.data),1,x.data,x.kind,x.fimath),Array(len(t.data),1,t.data))
        if n=="addElement" and isinstance(a[0],Dataset):
            need(len(a) in (2,3) and isinstance(a[1],Series) and (len(a)==2 or isinstance(a[2],str)),n)
            return Dataset(a[0].elements+({"Name":a[2] if len(a)==3 else "","Values":a[1]},))
        if n in ("getElement","get") and isinstance(a[0],Dataset):
            arity(a,2,n)
            return a[0].get(a[1])
        if n=="numElements" and isinstance(a[0],Dataset):
            arity(a,1,n)
            return scalar(len(a[0].elements))
        if n=="fixdt":
            arity(a,3,n)
            return Fixdt(fixed.numerictype(*map(integer,a)))
        if n=="bin":
            arity(a,1,n)
            x=a[0]
            need(isinstance(x,Array) and isinstance(x.kind,fixed.numerictype) and len(x.data)==1 and not ar.complex_data(x),n)
            w=x.kind.WordLength
            return format(x.data[0]%(1<<w),f"0{w}b")
        if n in ("double","storedInteger","logical"):
            arity(a,1,n)
            x=a[0]
            if n=="storedInteger":
                need(isinstance(x,Array) and isinstance(x.kind,fixed.numerictype),n)
                need(x.kind.WordLength<=64 and not ar.complex_data(x),n)
                w=next(w for w in (8,16,32,64) if w>=x.kind.WordLength)
                return Array(x.rows,x.cols,x.data,("int" if x.kind.SignednessBool else "uint")+str(w))
            data=real_data(x)
            return Array(x.rows,x.cols,tuple(map(bool,data)) if n=="logical" else data,"logical" if n=="logical" else "double")
        if n in ("size","numel","length"):
            need(len(a)==1 or n=="size" and len(a)==2,n)
            v=a[0]
            need(isinstance(v,(str,Array)),n)
            shape=(1,len(v)) if isinstance(v,str) else (v.rows,v.cols)
            if n=="size":
                if len(a)==2:
                    dim=integer(a[1])
                    need(dim>0,n)
                    return scalar(shape[dim-1] if dim<=2 else 1)
                return Array(1,2,tuple(map(float,shape)))
            return scalar(math.prod(shape) if n=="numel" else max(shape) if all(shape) else 0)
        if n in ("zeros","ones"):
            need(1<=len(a)<=2,n)
            if len(a)==1 and isinstance(a[0],Array) and (a[0].rows,a[0].cols)==(1,2):
                a=[scalar(v) for v in a[0].data]
            h,w=integer(a[0]),integer(a[-1])
            need(h>=0 and w>=0 and h*w<=LIMIT,n)
            return Array(h,w,(float(n=="ones"),)*(h*w))
        if n in ("floor","round","abs"):
            arity(a,1,n)
            x=a[0]
            need(isinstance(x,Array) and x.kind=="double",n)
            if n=="floor" and ar.complex_data(x):
                return replace(x,data=tuple(complex(math.floor(v.real),math.floor(v.imag)) for v in x.data))
            def rounded(v):
                frac,whole=math.modf(v)
                return whole+math.copysign(1,v) if abs(frac)>=0.5 else whole
            fn={"floor":math.floor,"round":rounded,"abs":abs}[n]
            return Array(x.rows,x.cols,tuple(float(fn(v)) for v in x.data))
        if n=="bitshift":
            arity(a,2,n)
            x=a[0]
            k=integer(a[1])
            need(x.kind=="double" and abs(k)<=63 and all(v.is_integer() and 0<=v<2**53 for v in x.data),n)
            codes=[(int(v)<<k) % 2**64 if k>=0 else int(v)>>-k for v in x.data]
            need(all(int(float(v))==v for v in codes),n+" precision")
            return Array(x.rows,x.cols,tuple(map(float,codes)))
        if n=="isempty":
            arity(a,1,n)
            v=a[0]
            need(isinstance(v,(str,Array,dict)),n)
            return scalar(not v.data if isinstance(v,Array) else not v if isinstance(v,str) else False,"logical")
        if n=="strcmp":
            arity(a,2,n)
            need(all(isinstance(v,str) for v in a),n)
            return scalar(a[0]==a[1],"logical")
        if n=="struct":
            d=options(a,{},n)
            need(all(IDENT.fullmatch(k) and not (isinstance(v,Array) and v.kind=="cell") for k,v in d.items()),n)
            return d
        if n=="isequal":
            need(len(a)>=2,n)
            return scalar(all(typed_equal(a[0],v) for v in a[1:]),"logical")
        if n=="assert":
            need(len(a) in (1,2) and isinstance(a[0],Array) and a[0].kind in ("logical","double"),n)
            if not a[0].data or not all(a[0].data):
                raise FxError("ASSERTION_FAILED",n,str(a[1]) if len(a)==2 else "Assertion failed")
            return VOID
        if n in ("fprintf","sprintf"):
            need(a,n)
            if n=="fprintf" and isinstance(a[0],Array):
                key=integer(a[0])
                need(key in self.handles and len(a)>=2,n)
                stream,_=self.handles[key]
                need(stream.writable(),n)
                data=fmt_values(a[1],a[2:]).encode("utf-8")
                return scalar(stream.write(data))
            msg=fmt_values(a[0],a[1:])
            if n=="sprintf":
                return msg
            sys.stdout.write(msg)
            return scalar(len(msg.encode()))
        if n=="num2str":
            arity(a,1,n)
            v=integer(a[0])
            need(abs(v)<=2**53,n,"Only integer scalar default text is recorded")
            return str(v)
        if n=="disp":
            arity(a,1,n)
            print(display(a[0]))
            return VOID
        if n in ("pwd","mfilename","fileparts","fullfile"):
            if n=="pwd":
                arity(a,0,n)
                return str(Path.cwd())
            if n=="mfilename":
                need(a==["fullpath"] and self.files,n)
                return str(self.files[-1].with_suffix(""))
            if n=="fileparts":
                arity(a,1,n)
                return os.path.dirname(a[0])
            need(a and all(isinstance(v,str) for v in a),n)
            return os.path.join(*a)
        need(False,n)

    def field(self,x,k):
        if isinstance(x,dict):
            need(k in x,k)
            return x[k]
        if isinstance(x,Series) and k in ("Time","Data"):
            return getattr(x,k)
        if isinstance(x,Dataset) and k=="numElements":
            return scalar(len(x.elements))
        if isinstance(x,Array) and isinstance(x.kind,fixed.numerictype):
            if k=="numerictype": return x.kind
            if k=="fimath": return x.fimath or fx._Math()
            if k=="int": return self.call("storedInteger",[x])
            if k=="Signed":
                return scalar(bool(x.kind.SignednessBool),"logical")
            x=x.kind
        if isinstance(x,Fixdt):
            x=x.numeric_type
        if isinstance(x,fixed.numerictype) and k in ("SignednessBool","WordLength","FractionLength"):
            return scalar(getattr(x,k))
        if isinstance(x,fx._Math) and k in x.__dataclass_fields__:
            v=getattr(x,k)
            return v if isinstance(v,str) else scalar(v,"logical" if isinstance(v,bool) else "double")
        need(False,k)

    def eval(self,node):
        t=node[0]
        if t=="lambda":
            return Closure(node[1],node[2],dict(self.workspace))
        if t=="char":
            return node[1]
        if t=="number":
            v=float(node[1])
            need(math.isfinite(v),"number")
            return scalar(v)
        if t=="imag":
            return Array(1,1,(complex(0,number(self.eval(node[1]))),))
        if t=="end":
            need(self.index_end,"end outside array index")
            return scalar(self.index_end[-1])
        if t=="all":
            return ALL
        if t=="var":
            k=node[1]
            if k in self.workspace:
                return self.workspace[k]
            if k in ("true","false"):
                return scalar(k=="true","logical")
            if k.lower()=="inf":
                return scalar(math.inf)
            if k=="pi":
                return scalar(math.pi)
            return self.call(k,[])
        if t=="grid":
            return grid([[self.eval(v) for v in row] for row in node[1]],node[2])
        if t=="range":
            return sequence(*(self.eval(v) for v in node[1:]))
        if t=="binary" and node[1] in ("&&","||"):
            left=bool(number(self.eval(node[2])))
            if (node[1]=="&&" and not left) or (node[1]=="||" and left):
                return scalar(left,"logical")
            return scalar(bool(number(self.eval(node[3]))),"logical")
        if t=="binary":
            a,b=self.eval(node[2]),self.eval(node[3])
            if ar.isfi(a) or ar.isfi(b):
                return ar.binary(node[1],a,b)
            if isinstance(a,Array) and isinstance(b,Array) and any(isinstance(v.kind,str) and ar.INTEGER.fullmatch(v.kind) for v in (a,b)):
                return helpers.intbinary(node[1],a,b)
            return binary(node[1],a,b)
        if t=="unary":
            v=self.eval(node[2])
            if ar.isfi(v) and node[1]=="-": return ar.element("neg",v)
            if isinstance(v,Array) and isinstance(v.kind,str) and ar.INTEGER.fullmatch(v.kind) and node[1]=="-":
                return replace(v,data=tuple(helpers.clamp_int(-x,v.kind) for x in v.data))
            need(isinstance(v,Array) and (v.kind=="double" or node[1]=="~" and v.kind=="logical"),node[1])
            if node[1]=="~":
                return Array(v.rows,v.cols,tuple(not n for n in v.data),"logical")
            return v if node[1]=="+" else Array(v.rows,v.cols,tuple(-n for n in v.data))
        if t=="transpose":
            v=self.eval(node[1])
            need(isinstance(v,Array),"transpose")
            out=v.transpose()
            if node[2] and ar.complex_data(out):
                need(not ar.isfi(out),"complex fi conjugate transpose")
                out=replace(out,data=tuple(n.conjugate() for n in out.data))
            return out
        if t=="field":
            if qualified(node)=="Simulink.SimulationData.Dataset":
                return self.call(qualified(node),[])
            return self.field(self.eval(node[1]),node[2])
        _,base,op,args=node
        name=qualified(base)
        if op=="(" and (base[0]=="var" and name not in self.workspace or name=="Simulink.SimulationData.Dataset"):
            previous=self.requested
            self.requested=1
            try: arguments=[self.eval(v) for v in args]
            finally: self.requested=previous
            return self.call(name,arguments)
        if base[0]=="field" and op=="(":
            x=self.eval(base[1])
            if isinstance(x,fx._Math) and base[2] in ("add","sub","mpy"):
                argv=[self.eval(v) for v in args]
                need(len(argv)==2 and all(ar.isfi(v) for v in argv),"fimath method")
                return ar.element({"add":"add","sub":"sub","mpy":"mul"}[base[2]],*(replace(v,fimath=x) for v in argv))
            if isinstance(x,Dataset):
                return self.call(base[2],[x]+[self.eval(v) for v in args])
            if isinstance(x,SimOutput) and base[2]=="get":
                argv=[self.eval(v) for v in args]
                arity(argv,1,"get")
                return self.field(x,argv[0])
            v=self.field(x,base[2])
        else:
            v=self.eval(base)
        if isinstance(v,Outer):
            need(op=="(" and len(args)==2,"large implicit expansion index")
            indices=[]
            for arg,length in zip(args,(v.rows,v.cols)):
                self.index_end.append(length)
                try:indices.append(integer(self.eval(arg))-1)
                finally:self.index_end.pop()
            i,j=indices
            need(0<=i<v.rows and 0<=j<v.cols,"array index")
            return scalar(v.at(i,j),v.kind)
        if isinstance(v,Closure) and op=="(":
            argv=[self.eval(arg) for arg in args]
            need(len(argv)==len(v.names),"anonymous function")
            before=self.workspace
            self.workspace=dict(v.workspace,**dict(zip(v.names,argv)))
            try: return self.eval(v.expr)
            finally: self.workspace=before
        argv=self.index_args(v,args)
        if isinstance(v,Dataset) and op=="{":
            arity(argv,1,"Dataset index")
            return v.get(argv[0])
        if isinstance(v,Port) and op=="(" and len(argv)==1 and integer(argv[0])==1:
            return v
        return index(v,argv,op=="{")

    def index_args(self,value,args):
        if isinstance(value,Array):
            shape=value.rows,value.cols
        elif isinstance(value,str):
            shape=1,len(value)
        else:
            return [self.eval(a) for a in args]
        need(len(args) in (1,2),"array index rank")
        sizes=(math.prod(shape),) if len(args)==1 else shape
        values=[]
        for a,n in zip(args,sizes):
            self.index_end.append(n)
            try:
                values.append(self.eval(a))
            finally:
                self.index_end.pop()
        return values

    def assign(self,lhs,value):
        need(value is not VOID,"assignment","Command has no supported return value")
        t=lhs[0]
        if t=="discard": return
        if t=="var":
            self.workspace[lhs[1]]=value
        elif t=="field":
            parent=lhs[1]
            obj={} if parent[0]=="var" and parent[1] not in self.workspace else self.eval(parent)
            if ar.isfi(obj) and lhs[2]=="fimath" and isinstance(value,fx._Math):
                self.assign(parent,replace(obj,fimath=value))
                return
            if ar.isfi(obj) and lhs[2]=="int":
                need(isinstance(value,Array) and isinstance(value.kind,str) and ar.INTEGER.fullmatch(value.kind)
                     and (obj.rows,obj.cols)==(value.rows,value.cols),"fi.int assignment")
                t=obj.kind
                need(all(fx._engine()._overflow(v,t.SignednessBool,t.WordLength,"Saturate")==v for v in value.data),"fi.int range")
                self.assign(parent,replace(obj,data=value.data))
                return
            need(type(obj) is dict,lhs[2])
            self.assign(parent,{**obj,lhs[2]:value})
        elif t=="apply":
            _,base,op,args=lhs
            if base[0]=="var" and base[1] not in self.workspace:
                if op=="{": old=Array(0,0,(),"cell")
                else:
                    need(isinstance(value,Array),"indexed assignment type")
                    old=replace(value,rows=0,cols=0,data=())
            elif base[0]=="field" and isinstance(self.eval(base[1]),dict) and base[2] not in self.eval(base[1]):
                old=Array(0,0,(),"cell" if op=="{" else "double")
            else:
                old=self.eval(base)
            need(isinstance(old,Array),"indexed assignment")
            argv=self.index_args(old,args)
            old=self.grow(old,argv)
            h,w,inds=selection(old,argv)
            if op=="{":
                need(old.kind=="cell" and len(inds)==1,"cell assignment")
                values=[value]
            else:
                if ar.isfi(old):
                    value=ar.cast(value,old.kind,old.fimath)
                    if ar.complex_data(value) and not ar.complex_data(old):
                        old=ar.combine(old,replace(old,data=(0,)*len(old.data)))
                    elif ar.complex_data(old) and not ar.complex_data(value):
                        value=ar.combine(value,replace(value,data=(0,)*len(value.data)))
                need(isinstance(value,Array) and value.kind==old.kind and (len(value.data)==1 or
                     len(value.data)==len(inds) and (len(args)==1 or (value.rows,value.cols)==(h,w))),"indexed assignment shape")
                values=value.data if len(value.data)!=1 else value.data*len(inds)
            data=BlockData.update(old.data,inds,values)
            self.assign(base,replace(old,data=data))
        else:
            need(False,"assignment target")

    def grow(self,old,argv):
        bounds=[max(v.data,default=0) if isinstance(v,Array) and v.kind=="double" else 0 for v in argv]
        h,w=old.rows,old.cols
        if len(argv)==1 and bounds[0]>len(old.data):
            need(h==1 or w==1 or h==w==0,"indexed growth")
            if w==1 and h!=1: h=int(bounds[0])
            else: h,w=1,int(bounds[0])
        elif len(argv)==2:
            h,w=max(h,int(bounds[0])),max(w,int(bounds[1]))
        if (h,w)==(old.rows,old.cols): return old
        need(h*w<=LIMIT,"array growth")
        fill=Array(0,0,()) if old.kind=="cell" else 0.0 if old.kind=="double" else 0
        return replace(old,rows=h,cols=w,data=tuple(old.at(r,c) if r<old.rows and c<old.cols else fill
                                                      for c in range(w) for r in range(h)))

    def execute(self,body):
        for line,kind,*args in body:
            try:
                if kind=="statement":
                    dest,expr,muted=args
                    # Opaque simulation objects are readable, never writable as structs.
                    if dest and dest[0]=="grid":
                        need(not dest[2] and len(dest[1])==1,"multiple assignment")
                        targets=dest[1][0]
                        before=self.requested
                        self.requested=len(targets)
                        try: value=self.eval(expr)
                        finally: self.requested=before
                        need(isinstance(value,tuple) and len(value)==len(targets),"multiple assignment")
                        for target,item in zip(targets,value): self.assign(target,item)
                        continue
                    bare=expr[1] if expr[0]=="var" else qualified(expr[1]) if expr[0]=="apply" else ""
                    value=self.eval(expr)
                    if dest is None and bare in ("tic","toc","cd","addpath") and bare not in self.workspace and not (self.locals and bare in self.locals[-1]) and self.locate(bare) is None:
                        if bare=="tic": self.last_tic=value.data[0]
                        if bare=="toc": print(f"Elapsed time is {number(value):.6f} seconds.")
                        if bare=="cd" and (expr[0]=="var" or not expr[3]): print(value)
                        continue
                    if dest:
                        self.assign(dest,value)
                    elif value is not VOID and not isinstance(value,Handle):
                        self.workspace["ans"]=value
                    if not muted and value is not VOID and not isinstance(value,Handle):
                        print(display(value))
                elif kind in ("break","return","continue"):
                    raise Flow(kind)
                elif kind=="persistent":
                    need(self.files and self.locals,"persistent")
                    # The invocation owns this frame, including recursive calls.
                    keep=self.active_keep
                    for k in args[0]:
                        self.workspace[k]=keep.setdefault(k,Array(0,0,()))
                elif kind=="switch":
                    expr,cases,other=args
                    key=self.eval(expr)
                    for label,branch in cases:
                        v=self.eval(label)
                        items=v.data if isinstance(v,Array) and v.kind=="cell" else (v,)
                        if any(typed_equal(key,item) for item in items):
                            self.execute(branch)
                            break
                    else:
                        self.execute(other)
                else:
                    var,cond,yes,no=args
                    v=self.eval(cond)
                    if kind=="if":
                        need(isinstance(v,Array) and v.kind in ("logical","double"),"if")
                        self.execute(yes if v.data and all(v.data) else no)
                    elif kind=="while":
                        count=0
                        while isinstance(v,Array) and v.kind in ("double","logical") and v.data and all(v.data):
                            count+=1
                            need(count<=LIMIT,"while iteration limit")
                            if self.loop_body(yes): break
                            v=self.eval(cond)
                        need(isinstance(v,Array) and v.kind in ("double","logical"),"while condition")
                    else:
                        need(isinstance(v,Array) and v.kind=="double" and v.rows==1,"for")
                        guarded=("bitshift","double","fi","mod","int8","int16","int32","int64","uint8","uint16","uint32","uint64")
                        shadowed=len(v.data)>=64 and any(self.locals and k in self.locals[-1] or self.locate(k) for k in guarded)
                        if not shadowed and _mloop.try_loop(self.workspace,var,v.data,yes): continue
                        if not shadowed and _mtypedloop.try_typed(self.workspace,var,v.data,yes,Closure): continue
                        for n in v.data:
                            self.workspace[var]=scalar(n)
                            if self.loop_body(yes): break
            except FxError as e:
                e.detail.setdefault("line",line)
                raise

    def loop_body(self,body):
        try: self.execute(body)
        except Flow as e:
            if e.kind=="return": raise
            return e.kind=="break"
        return False

    def run(self,text):
        self.directory_files.clear()
        self.resolved_names.clear()
        hdr,body,local=Parser(text).functions()
        need(hdr is None and not local,"batch function declaration")
        try:
            self.execute(body)
        except Flow as e:
            need(e.kind=="return",e.kind)
        finally:
            for stream,_ in self.handles.values(): stream.close()
            self.handles.clear()


def qualified(node):
    if node[0]=="var":
        return node[1]
    if node[0]=="field":
        return qualified(node[1])+"."+node[2]
    return ""


def display(v):
    if isinstance(v,Outer): need(False,"display large implicit expansion")
    if isinstance(v,str):
        return v
    if isinstance(v,Array) and v.kind!="cell":
        vals=real_data(v)
        return "\n".join("   ".join(format(vals[c*v.rows+r],".12g") for c in range(v.cols)) for r in range(v.rows))
    return type(v).__name__


def main(argv=None):
    a=sys.argv[1:] if argv is None else argv
    while a and a[0] in ("-nodisplay","-nosplash"):
        a=a[1:]
    if len(a)!=2 or a[0]!="-batch":
        print('Usage: fxsim-matlab -batch "<statements>"',file=sys.stderr)
        return 2
    try:
        Session().run(a[1])
        return 0
    except (FxError,OSError,RecursionError,ValueError,TypeError,ArithmeticError,IndexError,KeyError,AttributeError) as e:
        d=e.detail if isinstance(e,FxError) else dict(code="UNSUPPORTED",where="batch",message=str(e))
        prefix="not supported by fxsim: " if d["code"]=="UNSUPPORTED" and "not supported by fxsim" not in d["message"] else ""
        print(f"Error using {d.get('operation',d['where'])}\n{prefix}{d['message']}\n"+json.dumps(d,ensure_ascii=False),file=sys.stderr)
        return 1

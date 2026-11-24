"""Flat model state and MATLAB data/logging adaptation around the recordings."""
import re
import math
from dataclasses import dataclass, field
import struct
from fractions import Fraction
from pathlib import Path

from . import _model as api
from ._backend import FxError, options
from ._fixed import numerictype
from ._marray import Array, LIMIT, integer, need, scalar

IN = "simulink/Sources/In1"
OUT = "simulink/Sinks/Out1"
FROM = "simulink/Sources/From Workspace"
ADD = "simulink/Math Operations/Add"
SUM = "simulink/Math Operations/Sum"
DT = re.compile(r"fixdt\(\s*([01])\s*,\s*(\d+)\s*,\s*(-?\d+)\s*\)")
ONOFF = ("on","off")
CONFIG = {"SolverType":("Fixed-step",),"Solver":("FixedStepDiscrete",),"StartTime":("0",),
          "SaveFormat":("Dataset",),"SaveOutput":ONOFF,"SaveTime":ONOFF,"SaveState":("off",),
          "ReturnWorkspaceOutputs":("on",),"SignalLogging":ONOFF,"SignalLoggingName":("logsout",),
          "OutputSaveName":("yout",),"TimeSaveName":("tout",),"LoadExternalInput":ONOFF,
          "MinMaxOverflowLogging":("UseLocalSettings",),"DataTypeOverride":("UseLocalSettings",)}
DEFAULT = {"StopTime":"10.0","StartTime":"0.0","SaveFormat":"Dataset","SaveOutput":"on","SaveTime":"on",
           "SignalLogging":"on","OutputSaveName":"yout","TimeSaveName":"tout","SignalLoggingName":"logsout",
           "LoadExternalInput":"off"}


@dataclass(frozen=True)
class Series:
    Data: Array
    Time: Array


@dataclass(frozen=True)
class Dataset:
    elements: tuple = ()

    def get(self,key):
        if isinstance(key,str):
            match=[e for e in self.elements if e["Name"]==key]
            need(len(match)==1,"Dataset.getElement",f"No unique logged signal named {key!r}")
            return match[0]
        i=integer(key)
        need(1<=i<=len(self.elements),"Dataset.getElement index")
        return self.elements[i-1]


class SimOutput(dict):
    """Named workspace variables returned from sim."""


@dataclass(frozen=True)
class Port:
    path: str
    number: int


@dataclass
class State:
    name: str
    settings: dict = field(default_factory=dict)
    blocks: dict = field(default_factory=dict)
    logs: dict = field(default_factory=dict)
    path: Path = None
    dirty: bool = True

    def configure(self,opts):
        for key,v in opts.items():
            if key in ("StopTime","FixedStep"):
                need(isinstance(v,str) and (key=="FixedStep" and v=="auto" or
                     re.fullmatch(r"(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?",v)
                     and (Fraction(v)>0 or key=="StopTime" and Fraction(v)==0)),key,v)
            elif key=="ExternalInput":
                need(isinstance(v,str) and api.ID.fullmatch(v),key)
            else:
                need(key in CONFIG and v in CONFIG[key],key,f"{key}={v!r} not supported by fxsim")
        self.settings.update(opts)
        self.dirty=True

    def edit(self,path,opts,source=None):
        local=path.partition("/")[2]
        old=self.blocks.get(local)
        if source is None:
            need(old is not None,path)
            source=old["source"]
        backend = IN if source==FROM else SUM if source==ADD else source
        if backend not in api.CATALOG:
            raise FxError("UNKNOWN_BLOCK_TYPE",source,"Library block not supported by fxsim")
        p=dict(opts)
        if "Position" in p:
            pos=p.pop("Position")
            need(isinstance(pos,Array) and (pos.rows,pos.cols)==(1,4)
                 and all(isinstance(v,float) and v.is_integer() for v in pos.data), "Position")
            opts=dict(opts,Position=[int(v) for v in pos.data])
        if source==FROM:
            need(set(p)<={"VariableName","SampleTime","OutDataTypeStr","Interpolate","OutputAfterFinalValue"},source)
            if "VariableName" in p:
                need(isinstance(p["VariableName"],str) and api.ID.fullmatch(p.pop("VariableName")),"VariableName")
            if "Interpolate" in p:
                need(p.pop("Interpolate")=="off","From Workspace.Interpolate")
            if "OutputAfterFinalValue" in p:
                need(p.pop("OutputAfterFinalValue") in ("Holding final value","Setting to zero"),"OutputAfterFinalValue")
            # Mixed root external-input ports and internal workspace sources have
            # different port-number domains; never collapse them into one list.
            need(not any(b["source"]==IN for b in self.blocks.values()),"mixed root sources")
        elif source==IN:
            need(not any(b["source"]==FROM for b in self.blocks.values()),"mixed root sources")
        if old is None or not self.blocks.get(local):
            if source==ADD:
                p.setdefault("IconShape","rectangular")
                p.setdefault("Inputs","++")
            api.add_block(backend,path,**p)
            self.blocks[local]={"source":source,"params":{}}
        else:
            api.set_param(path,**p)
        self.blocks[local]["params"].update(opts)
        self.dirty=True

    def get(self,path,key):
        if path==self.name:
            if key in self.settings:
                return self.settings[key]
            need(key in DEFAULT,key)
            return DEFAULT[key]
        n=path.partition("/")[2]
        need(n in self.blocks,path)
        b=self.blocks[n]
        if key=="PortHandles":
            # One-output blocks only; multi-output sampling must use explicit ports.
            need(b["source"]!=OUT and api.model(self.name).blocks[n]["type"] not in ("HDLNCO",),"PortHandles")
            return {"Outport":Port(path,1)}
        if key in b["params"]:
            v=b["params"][key]
            return Array(1,4,tuple(map(float,v))) if key=="Position" else v
        if b["source"]==FROM:
            need(key=="Name",key)
        return api.get_param(path,key)

    def connect(self,source,dest,args):
        m=api.model(self.name)
        for endpoint,is_output in ((source,True),(dest,False)):
            match=re.fullmatch(r"([^/]+)/([1-9][0-9]*)",endpoint) if isinstance(endpoint,str) else None
            if not match or match[1] not in m.blocks:
                raise FxError("BAD_CONNECTION",str(endpoint),"Expected an existing block/port")
            b=m.blocks[match[1]]
            kind=b["type"]
            if is_output:
                ports=0 if kind=="Outport" else 1
                if kind=="HDLNCO":
                    ports=2+int(b.get("Waveform")=="Sine and cosine")+int(b.get("PhasePort")=="on")
            else:
                ports=0 if kind in ("Constant","Inport","CounterLimited") else 3 if kind=="Switch" else 1
                if kind in ("Sum","Product","MinMax","Mux"):
                    desc=b.get("Inputs",api.DEFAULTS.get(kind,{}).get("Inputs","2"))
                    ports=int(desc) if desc.isdecimal() else len(desc.replace("|",""))
                if kind=="ShiftArithmetic" and b.get("BitShiftNumberSource")=="Input port":
                    ports=2
                if kind=="Delay":
                    ports=1+int(b.get("DelayLengthSource")=="Input port")+int(b.get("ShowEnablePort")=="on")+int(b.get("InitialConditionSource")=="Input port")+int(b.get("ExternalReset","None")!="None")
                if kind=="HDLNCO":
                    ports=1+sum(b.get(k)=="Input port" for k in ("PhaseIncrementSource","PhaseOffsetSource","DitherSource"))
            if int(match[2])>ports:
                raise FxError("BAD_CONNECTION",endpoint,"Port is absent or outside the supported block configuration")
        api.add_line(self.name,source,dest,*args)

    def log(self,port,opts):
        for k,v in opts.items():
            need(k in ("DataLogging","DataLoggingNameMode","DataLoggingName"),k)
            need(isinstance(v,str) and (v in ONOFF if k=="DataLogging" else
                 v=="Custom" if k=="DataLoggingNameMode" else bool(v)),k)
        self.logs.setdefault(port.path.split("/")[1],{}).update(opts)
        self.dirty=True

    def inputs(self,workspace,saving=False):
        need(self.settings.get("Solver")=="FixedStepDiscrete","Solver","Select the recorded FixedStepDiscrete solver")
        m=api.model(self.name)
        ports=m.ports("Inport")
        cfg={**DEFAULT,**self.settings}
        stop=Fraction(cfg["StopTime"])
        dataset=None
        if cfg["LoadExternalInput"]=="on":
            dataset=workspace.get(cfg.get("ExternalInput"))
            need(isinstance(dataset,Dataset) and len(dataset.elements)==len(ports),"ExternalInput")
        data={}
        synthetic=[]
        periods=set()
        for i,n in enumerate(ports):
            b=self.blocks[n]
            opts=b["params"]
            st=opts.get("SampleTime","-1")
            series=dataset.elements[i]["Values"] if dataset else None
            if b["source"]==FROM:
                need(dataset is None and opts.get("Interpolate")=="off",n+".Interpolate")
                var=opts.get("VariableName")
                need(var,n+".VariableName")
                series=workspace.get(var)
                if series is None:
                    need(saving,var,"Workspace timeseries is missing")
                    synthetic.append(n)
            if series is not None:
                need(isinstance(series,Series),n+".timeseries")
            if st=="-1":
                need(series is not None and len(series.Time.data)>=2,n+".SampleTime")
                dt=Fraction(str(series.Time.data[1]))
            else:
                dt,offset=api.clock(st)
                need(offset==0,n+".SampleTime offset")
            need(dt>0 and 0<=stop/dt<LIMIT,n+".SampleTime")
            periods.add(dt)
            count=int(stop/dt)+1
            if series:
                t=series.Time.data
                x=series.Data
                need(x.cols==1 and series.Time.cols==1 and len(x.data)==len(t)
                     and all(v==float(j*dt) for j,v in enumerate(t)),n+".timeseries grid")
                if len(t)<count:
                    need(b["source"]==FROM and len(t)>0,n+".input horizon")
                    fill=0 if opts.get("OutputAfterFinalValue")=="Setting to zero" else x.data[-1]
                else:
                    fill=0
                codes=list(x.data[:count])+[fill]*max(0,count-len(t))
                if x.kind=="logical":
                    data[n]=dict(type="boolean",ints=list(map(int,codes)))
                else:
                    need(isinstance(x.kind,numerictype),n+".timeseries type")
                    data[n]=dict(x.kind.record(),ints=codes)
            else:
                typ=opts.get("OutDataTypeStr","")
                if typ=="boolean":
                    data[n]=dict(type="boolean",ints=[0]*count)
                else:
                    mt=DT.fullmatch(typ)
                    need(mt,n+".OutDataTypeStr","Explicit root type required for zero input")
                    data[n]=dict(numerictype(*map(int,mt.groups())).record(),ints=[0]*count)
        need(len(periods)==1,"input clock","One common positive root sample clock is required")
        dt=periods.pop()
        step=self.settings.get("FixedStep","auto")
        need(step=="auto" or (dt/Fraction(step)).denominator==1,"FixedStep","The step must divide the input period")
        return data,dt,stop,synthetic

    def request_data(self,workspace,saving=False):
        data,dt,stop,synth=self.inputs(workspace,saving)
        req=api.request(self.name,data,stop_time=str(stop))
        for b in req["design"]["blocks"].values():
            if b["type"] in ("Downsample","Upsample","CICDecimation","FIRInterpolation","FIRDecimation","RateTransition"):
                defaults=api.DEFAULTS.get(b["type"],{})
                for key in ("RateOptions","framing","InputProcessing","OutPortSampleTimeOpt"):
                    if key in defaults:
                        b.setdefault(key,defaults[key])
        for n in data:
            if self.blocks[n]["params"].get("SampleTime","-1")=="-1":
                req["design"]["blocks"][n]["SampleTime"]=str(float(dt))
        return req,dt,stop,synth


    def grid_step(self,req,raw,dt,stop):
        step=self.settings.get("FixedStep","auto")
        clocks=[dt]
        if "stop_time" in raw["metrics"]:
            all_rows=api.run_model("fxgraph",dict(req,outputs="all"))["outputs"]
            need(set(api.model(self.name).blocks)<=set(all_rows),"multirate solver grid")
            for row in all_rows.values():
                if row.get("period")=="constant":
                    continue
                need("period" in row,"multirate period")
                clocks.append(Fraction(row["period"]))
                if row.get("offset"):
                    clocks.append(Fraction(row["offset"]))
        scale=math.lcm(*(c.denominator for c in clocks))
        base=Fraction(math.gcd(*(int(c*scale) for c in clocks)),scale)
        solver=base if step=="auto" else Fraction(step)
        need(solver>0 and (base/solver).denominator==1,"FixedStep","Step does not divide a compiled signal clock")
        need((stop/solver).denominator==1 and stop/solver<LIMIT,"StopTime","StopTime must land on the supported solver grid")
        return solver

    def simulate(self,workspace):
        req,dt,stop,_=self.request_data(workspace)
        observed=[n for n,v in self.logs.items() if v.get("DataLogging")=="on"]
        if observed:
            req["outputs"]=list(dict.fromkeys(api.model(self.name).ports("Outport")+observed))
        raw=api.run_model("fxgraph",req)
        solver=self.grid_step(req,raw,dt,stop)
        result=SimOutput()
        m=api.model(self.name)
        logged=[]
        cfg={**DEFAULT,**self.settings}
        outs=m.ports("Outport")
        need(set(outs+observed)==set(raw["outputs"]),"vector or complex output shape")
        series={}
        for n in raw["outputs"]:
            row=raw["outputs"][n]
            if "ints" in row:
                typ=numerictype(row["s"],row["wl"],row["fl"])
                arr=Array(len(row["ints"]),1,tuple(row["ints"]),typ)
            elif "bool" in row:
                arr=Array(len(row["bool"]),1,tuple(map(bool,row["bool"])),"logical")
            elif row["type"] in ("single","double"):
                fmt=">d" if row["type"]=="double" else ">f"
                values=tuple(struct.unpack(fmt,bytes.fromhex(h))[0] for h in row["hex"])
                arr=Array(len(values),1,values,row["type"])
            else:
                need(False,n+".Data type")
            times=tuple(float(Fraction(x)) for x in row["t"]) if "t" in row else tuple(float(i*dt) for i in range(arr.rows))
            ser=Series(arr,Array(len(times),1,times))
            series[n]=ser
        outputs=[{"Name":"","Values":series[n]} for n in outs]
        for src in observed:
            opts=self.logs[src]
            need(opts.get("DataLoggingNameMode")=="Custom" and opts.get("DataLoggingName"),"DataLoggingName")
            logged.append({"Name":opts["DataLoggingName"],"Values":series[src]})
        if cfg["SaveOutput"]=="on":
            result["yout"]=Dataset(tuple(outputs))
        if cfg["SignalLogging"]=="on" and logged:
            result["logsout"]=Dataset(tuple(logged))

        if cfg["SaveTime"]=="on":
            count=int(stop/solver)+1
            result["tout"]=Array(count,1,tuple(float(i*solver) for i in range(count)))
        return result

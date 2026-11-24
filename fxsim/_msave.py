"""Write a flat SLX and a checker for the exported model."""
import copy
import hashlib
import json
import os
import shutil
import tempfile
import zipfile
from pathlib import Path
from xml.dom import minidom
from xml.etree import ElementTree as ET

from . import _model as api
from ._backend import root
from ._marray import need
from ._mbuild import ADD, FROM, IN, OUT, SUM


# A plain SLX Block starts with the built-in block's dialog values. These
# R2026b library values differ from them. Reference blocks have a SourceBlock
# link and obtain their omitted dialog values from that library directly.
_LIBRARY_OVERRIDES = {
    "ArithShift": {"BitShiftNumber": "8"},
    "DataTypeConversion": {"RndMeth": "Floor", "SaturateOnIntegerOverflow": "off"},
    "DownSample": {"AllowArbitraryFixedInput": "on"},
    "Lookup_n-D": {"NumberOfTableDimensions": "1", "BreakpointsForDimension1": "[-5:5]",
                   "Table": "tanh([-5:5])", "InternalRulePriority": "Speed", "RndMeth": "Simplest"},
    "Mux": {"Inputs": "2", "DisplayOption": "bar"},
    "Selector": {"InputPortWidth": "3", "Indices": "[1 3]"},
    "Switch": {"Criteria": "u2 > Threshold", "InputSameDT": "off", "SaturateOnIntegerOverflow": "off"},
    "UnitDelay": {"SampleTime": "-1"},
}
_STRUCTURAL = {"Position", "ZOrder", "SourceBlock", "SourceType", "LibraryVersion",
               "LibrarySourceBlock", "InputPortMap", "OutputPortMap"}


def restore_block(block, spec, model_block):
    source = spec["source"]
    if source == FROM:
        block.set("BlockType", "FromWorkspace")
    params = dict(_LIBRARY_OVERRIDES.get(block.get("BlockType"), {}))
    if source == ADD:
        params["IconShape"] = "rectangular"
    elif source == SUM:
        params["Inputs"] = "|++"
    if source in (IN, OUT) and model_block["Port"] != "1":
        params["Port"] = model_block["Port"]
    params.update(spec["params"])

    instance = block.find("InstanceData")
    holders = [block] if instance is None else [block, instance]
    for holder in holders:
        for prop_node in holder.findall("P"):
            if prop_node.get("Name") not in _STRUCTURAL:
                holder.remove(prop_node)
    if "Position" in params:
        prop(block, "Position", "[" + ", ".join(map(str, params.pop("Position"))) + "]")
    for key, value in params.items():
        prop(holders[-1], key, value)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prop(el,k,v):
    a=el.find(f"./P[@Name='{k}']")
    if a is None:
        a=ET.SubElement(el,"P",Name=k)
    a.text=str(v)


def flat_content_types(data, removed_part):
    # Simulink expects the exporter's default namespace on the OPC type table.
    namespace = "http://schemas.openxmlformats.org/package/2006/content-types"
    with minidom.parseString(data) as document:
        overrides = document.getElementsByTagNameNS(namespace, "Override")
        matches = [node for node in overrides if node.getAttribute("PartName") == "/" + removed_part]
        need(len(matches) == 1, "export subsystem content type")
        document.documentElement.removeChild(matches[0])
        declaration = b'<?xml version="1.0" encoding="UTF-8" standalone="yes" ?>\n'
        return declaration + document.documentElement.toxml(encoding="utf-8") + b"\n"


def flatten(src,dest,state):
    with zipfile.ZipFile(src) as z:
        parts={n:z.read(n) for n in z.namelist()}
    base="simulink/systems/"
    rootkey=base+"system_root.xml"
    outer=ET.fromstring(parts[rootkey])
    dut=outer.find("Block[@BlockType='SubSystem']")
    need(dut is not None,"export DUT")
    child=base+dut.find("System").get("Ref")+".xml"
    flat=ET.fromstring(parts.pop(child))
    need(set(b.get("Name") for b in flat.findall("Block"))==set(state.blocks) and not flat.findall("Block/System"),"flat export topology")
    for p in reversed(outer.findall("P")):
        if p.get("Name") in ("ReportName","SIDHighWatermark"):
            flat.insert(0,copy.deepcopy(p))
    model_blocks = api.model(state.name).blocks
    for block in flat.findall("Block"):
        name = block.get("Name")
        restore_block(block, state.blocks[name], model_blocks[name])
    parts[rootkey]=ET.tostring(flat,encoding="utf-8",xml_declaration=True)
    parts.pop(base+"_rels/system_root.xml.rels")
    parts["[Content_Types].xml"]=flat_content_types(parts["[Content_Types].xml"], child)
    gi_key="simulink/graphicalInterface.json"
    gi=json.loads(parts[gi_key])
    gi["RootOutports"]=[{"Name":n} for n in api.model(state.name).ports("Outport")]
    if any(b["source"]==IN for b in state.blocks.values()):
        gi["RootInports"]=[{"Name":n} for n in api.model(state.name).ports("Inport")]
    for lib in gi.get("ExternalFileReferences",[]):
        lib["Path"]=lib["Path"].replace("$bdroot/"+dut.get("Name")+"/","$bdroot/",1)
    parts[gi_key]=json.dumps(gi).encode()
    cfgkey="simulink/configSet0.json"
    cfg=json.loads(parts[cfgkey])
    comps={c["_object_class"].split(".")[-1]:c["_properties"] for c in cfg["_properties"]["Components"]["_elements"]}
    for params in comps.values():
        for k in list(params):
            if k!="Version":
                del params[k]
    comps["SolverCC"]["SolverName"]="FixedStepDiscrete"
    for k,v in state.settings.items():
        if k in ("FixedStep","StartTime","StopTime"):
            comps["SolverCC"][k]=v
        elif k not in ("Solver","SolverType","DataTypeOverride","MinMaxOverflowLogging"):
            comps["DataIOCC"][k]=v
        # UseLocalSettings is the native omitted/default value for both options.
    parts[cfgkey]=json.dumps(cfg).encode()
    with zipfile.ZipFile(dest,"w",zipfile.ZIP_DEFLATED) as z:
        for n,data in parts.items():
            info=zipfile.ZipInfo(n,date_time=(2026,1,1,0,0,0))
            z.writestr(info,data,compress_type=zipfile.ZIP_DEFLATED)


def load(path):
    manifest=path.with_name(path.stem+"_manifest.json")
    need(manifest.is_file(),"load_system","Arbitrary native SLX import not supported by fxsim; missing flat-model manifest")
    data=json.loads(manifest.read_text())
    need(data.get("format")=="fxsim-flat/1" and data.get("model")==path.stem and path.is_file()
         and data["files"].get(path.name)==digest(path),"load_system integrity")
    # The blueprint is bound into the SLX via a digest, so changed settings or
    # rewiring in a sidecar cannot silently replace what the SLX contains.
    with zipfile.ZipFile(path) as z:
        expected=z.comment.decode()
    need(expected==blueprint_hash(data["blueprint"]),"load_system blueprint digest")
    return data["blueprint"]


def blueprint_hash(spec):
    return hashlib.sha256(json.dumps(spec,sort_keys=True,separators=(",",":")).encode()).hexdigest()


def save(state,path,workspace,prepared=None):
    import fcntl
    name=state.name
    need(path.suffix==".slx" and path.stem==name,"save_system name","Model renaming is outside this subset")
    need(path.parent.is_dir(),"save_system destination")
    need(not any(opts.get("DataLogging")=="on" for opts in state.logs.values()),"saving logged internal ports")
    req,dt,stop,synthetic=prepared if prepared is not None else state.request_data(workspace,saving=True)
    # A successful export of the witness alone cannot establish that the step
    # requested for the flat model is valid for all its compiled signals.
    raw=api.run_model("fxgraph",req)
    state.grid_step(req,raw,dt,stop)
    m=api.model(name)
    blueprint=dict(settings=state.settings,blocks=state.blocks,lines=m.lines,logs=state.logs)
    runs=root()/"runs"
    runs.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(dir=runs,prefix="batch-export-") as tmp:
        pkg=Path(tmp)/"pkg"
        api.run_model("fxexport",dict(req,name=name,out=str(pkg)))
        witness=json.loads((pkg/"manifest.json").read_text())
        io=json.loads((pkg/(name+"_io.json")).read_text())
        need(all(o["width"]==1 and not o["part"] for o in io["outputs"]),"export non-scalar or complex logging")
        with tempfile.TemporaryDirectory(dir=path.parent,prefix=".fxsim-save-") as stage:
            stage=Path(stage)
            entry=stage/path.name
            flatten(pkg/path.name,entry,state)
            with zipfile.ZipFile(entry,"a") as z:
                z.comment=blueprint_hash(blueprint).encode()
            inputs=io["inputs"]
            for row in inputs:
                b=state.blocks[row["port"]]
                row["source"]=b["source"]
                if b["source"]==FROM:
                    row["variable"]=b["params"]["VariableName"]
            flat_io=dict(model=name,model_sha256=digest(entry),inputs=inputs,outputs=io["outputs"])
            iofile=stage/(name+"_io.json")
            iofile.write_text(json.dumps(flat_io)+"\n")
            check=stage/("check_"+name+".m")
            check.write_text(Path(__file__).with_name("flat_check.m.in").read_text().replace("__MODEL__",name))
            manifest=stage/(name+"_manifest.json")
            files={f.name:digest(f) for f in (entry,iofile,check)}
            manifest.write_text(json.dumps(dict(format="fxsim-flat/1",model=name,files=files,
                blueprint=blueprint,outputs=witness["outputs"],check_only_zero_sources=synthetic,
                native_check="pending"),indent=2)+"\n")
            # Directory lock works across independent batch writers on POSIX,
            # needs no permanent lock file, and commits the manifest last.
            fd=os.open(path.parent,os.O_RDONLY)
            try:
                fcntl.flock(fd,fcntl.LOCK_EX)
                targets=[path.parent/f.name for f in (entry,iofile,check,manifest)]
                if any(p.exists() or p.is_symlink() for p in targets):
                    need(all(p.is_file() and not p.is_symlink() for p in targets),"save_system existing files")
                    previous=json.loads(targets[-1].read_text())
                    need(previous.get("format")=="fxsim-flat/1" and previous.get("model")==name and
                         all(previous["files"].get(p.name)==digest(p) for p in targets[:-1]),"save_system existing package")
                backup=stage/"backup"
                backup.mkdir()
                installed=[]
                try:
                    for dst in targets:
                        if dst.exists():
                            shutil.copy2(dst,backup/dst.name)
                        os.replace(stage/dst.name,dst)
                        installed.append(dst)
                except OSError:
                    for dst in reversed(installed):
                        if (backup/dst.name).exists():
                            os.replace(backup/dst.name,dst)
                        else:
                            dst.unlink()
                    raise
            finally:
                os.close(fd)
    state.path=path
    state.dirty=False

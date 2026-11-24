"""Run from a built preview with python3 -I engine/check_flat_save.py."""
import os
import re
import subprocess
import tempfile
from pathlib import Path
from xml.etree import ElementTree as ET
from zipfile import ZipFile


ROOT = Path(__file__).resolve().parents[1]
STRUCTURE = {"Position", "ZOrder", "SourceBlock", "SourceType", "LibraryVersion",
             "LibrarySourceBlock", "InputPortMap", "OutputPortMap"}


def check_blocks(path, expected):
    with ZipFile(path) as archive:
        system = ET.fromstring(archive.read("simulink/systems/system_root.xml"))
    blocks = system.findall("Block")
    assert len(blocks) == len(expected) and {b.get("Name") for b in blocks} == set(expected)
    for block in blocks:
        name = block.get("Name")
        kind, params = expected[name]
        assert block.get("BlockType") == kind, name
        written = {}
        for node in block.findall("P") + block.findall("InstanceData/P"):
            key = node.get("Name")
            if key not in STRUCTURE:
                assert key not in written, (name, key)
                written[key] = node.text or ""
        assert written == params, (name, written, params)
    return system


def run(directory, program):
    env = dict(os.environ, FXSIM_ROOT=str(ROOT), PYTHONDONTWRITEBYTECODE="1")
    subprocess.run([str(ROOT / "engine/fxsim-matlab"), "-batch", program],
                   cwd=directory, check=True, env=env)


def verify(directory):
    readme = (ROOT / "README.md").read_text()
    demo, = [s for s in re.findall(r"```matlab\n(.*?)\n```", readme, re.S)
             if "m='delay_demo';" in s]
    (directory / "delay_demo.m").write_text(demo + "\n")
    run(directory, "run('delay_demo.m');")
    dtype = "fixdt(1,8,2)"
    expected = {"x": ("Inport", {"OutDataTypeStr": dtype, "SampleTime": "0.5"}),
                "d": ("UnitDelay", {"InitialCondition": "0", "SampleTime": "-1"}),
                "y": ("Outport", {})}
    check_blocks(directory / "delay_demo.slx", expected)

    # Reopen and edit the model in a fresh process.
    run(directory, """
        load_system('delay_demo.slx');
        ds=Simulink.SimulationData.Dataset;
        ds=ds.addElement(timeseries(fi([1;-2;3],1,8,2),(0:0.5:1)'), 'x');
        set_param('delay_demo/d','InitialCondition','2','Position',[15 30 45 60]);
        save_system('delay_demo'); close_system('delay_demo',0);
    """)
    expected["d"][1]["InitialCondition"] = "2"
    tree = check_blocks(directory / "delay_demo.slx", expected)
    position = tree.find("Block[@Name='d']/P[@Name='Position']")
    assert position is not None and position.text == "[15, 30, 45, 60]"

    # The input series supplies the second model's root type and rate.
    run(directory, """
        m='params'; new_system(m);
        set_param(m,'SolverType','Fixed-step','Solver','FixedStepDiscrete','StopTime','2');
        add_block('simulink/Sources/In1',[m '/x']);
        add_block('simulink/Discrete/Unit Delay',[m '/d'],'SampleTime','1');
        add_block('simulink/Signal Attributes/Data Type Conversion',[m '/c'],'OutDataTypeStr','fixdt(1,8,2)');
        add_block('simulink/Math Operations/Gain',[m '/g'],'Gain','1','OutDataTypeStr','fixdt(1,8,2)');
        add_block('simulink/Sinks/Out1',[m '/out']);
        add_block('simulink/Sinks/Out1',[m '/tap']);
        set_param([m '/c'],'RndMeth','Zero');
        add_line(m,'x/1','d/1'); add_line(m,'d/1','c/1');
        add_line(m,'c/1','g/1'); add_line(m,'g/1','out/1'); add_line(m,'x/1','tap/1');
        ds=Simulink.SimulationData.Dataset;
        ds=ds.addElement(timeseries(fi([1;2;3],1,8,2),(0:2)'), 'x');
        set_param(m,'LoadExternalInput','on','ExternalInput','ds');
        save_system(m); close_system(m,0);
    """)
    check_blocks(directory / "params.slx", {
        "x": ("Inport", {}), "d": ("UnitDelay", {"SampleTime": "1"}),
        "c": ("DataTypeConversion", {"OutDataTypeStr": dtype, "RndMeth": "Zero",
                                     "SaturateOnIntegerOverflow": "off"}),
        "g": ("Gain", {"Gain": "1", "OutDataTypeStr": dtype}),
        "out": ("Outport", {}), "tap": ("Outport", {"Port": "2"}),
    })


if __name__ == "__main__":
    with tempfile.TemporaryDirectory() as tmp:
        verify(Path(tmp))
    print("PASS flat save block parameters")

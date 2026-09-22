"""Check generated syntax, KiCad connectivity, and critical daughterboard invariants."""

from pathlib import Path
import json
import subprocess
import tempfile
import xml.etree.ElementTree as ET
import sexpdata

ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = ROOT.parent
CLI = Path(r"C:\Program Files\KiCad\9.0\bin\kicad-cli.exe")


def validate():
    root = sexpdata.loads((PROJECT_ROOT/'XG_Mobile_USB_Hub.kicad_sch').read_text(encoding='utf-8'))
    children = [item for item in root if isinstance(item,list) and str(item[0]) == 'sheet']
    child_files = [str(field[2]) for child in children for field in child
                   if isinstance(field,list) and str(field[0]) == 'property' and field[1] == 'Sheetfile']
    assert sorted(child_files) == ['XG_Mobile_USB_Hub_MCU.kicad_sch','XG_Mobile_USB_Hub_Power.kicad_sch','XG_Mobile_USB_Hub_USB_Hub.kicad_sch']
    assert {path.name for path in PROJECT_ROOT.glob('XG_Mobile_USB_Hub*.kicad_sch')} == set(child_files+['XG_Mobile_USB_Hub.kicad_sch'])
    assert not list(ROOT.glob('*.kicad_sch')), 'Obsolete nested schematics remain'
    direct_wires = 0
    for path in PROJECT_ROOT.glob('XG_Mobile_USB_Hub*.kicad_sch'):
        parsed = sexpdata.loads(path.read_text(encoding='utf-8'))
        assert str(parsed[0]) == 'kicad_sch', path
        for item in parsed:
            if isinstance(item,list) and str(item[0])=='wire':
                points = next(field for field in item if isinstance(field,list) and str(field[0])=='pts')[1:]
                length = abs(float(points[0][1])-float(points[1][1]))+abs(float(points[0][2])-float(points[1][2]))
                direct_wires += length>5.09
    assert direct_wires>=100, 'Expected directly wired functional groups, not only label stubs'
    expected = [component for component in json.loads((ROOT/'connectivity.json').read_text()) if component['physical']]
    assert {component['sheet'] for component in expected} == {'Power','MCU','USB_Hub'}
    with tempfile.TemporaryDirectory() as directory:
        netlist_path = Path(directory)/'netlist.xml'
        subprocess.run([str(CLI), 'sch', 'export', 'netlist', '--format', 'kicadxml', '--output', str(netlist_path), str(PROJECT_ROOT/'XG_Mobile_USB_Hub.kicad_sch')], check=True)
        netlist = ET.parse(netlist_path)
    actual = {(node.get('ref'), node.get('pin')): net.get('name').lstrip('/')
              for net in netlist.findall('.//nets/net') for node in net.findall('node')}
    errors = [(component['ref'], pin, net, actual.get((component['ref'], pin)))
              for component in expected for pin, net in component['nets'].items()
              if net is not None and actual.get((component['ref'], pin)) != net]
    assert not errors, errors
    assert len(netlist.findall('.//components/comp')) == len(expected)
    components = {component['ref']: component for component in expected}
    for ref in ['J600','J700','J1200']:
        nets = components[ref]['nets']
        assert nets['A6'] == nets['B6']
        assert nets['A7'] == nets['B7']
        assert len({nets[pin] for pin in ['A4','A9','B4','B9']}) == 1
    assert components['U501']['nets']['9'] == 'VCORE'
    assert components['U501']['nets']['2'] == 'HUB_VBUS_DET'
    assert components['U101']['nets']['5'] == components['U1300']['nets']['31'] == 'I2C_SDA'
    assert components['U101']['nets']['6'] == components['U1300']['nets']['30'] == 'I2C_SCL'
    assert components['U101']['nets']['9'] == 'GND'
    assert components['U101']['nets']['14'] == 'PD_VCC2'
    assert components['U1300']['nets']['18'] == components['U1200']['nets']['1'] == 'MCU_CC1'
    assert components['U1300']['nets']['17'] == components['U1200']['nets']['3'] == 'MCU_CC2'
    assert components['U600']['nets']['30'] == components['U700']['nets']['30'] == 'V5'
    assert components['J1100']['kind'] == 'USB_A'
    for base,port in [(800,3),(900,4),(1000,5),(1100,6)]:
        bulk = components[f'C{base+4}']
        assert bulk['value'] == '220uF 10V +/-20% low ESR'
        assert bulk['nets'] == {'1':f'P{port}_VBUS','2':'GND'}
    assert components['L101']['mpn'] == 'APS1040M2R2A'
    assert abs(0.6*(1+45.3/10)-3.3) < 0.03
    assert abs(0.6*(1+10/1.37)-5) < 0.03
    assert 1.09 < 0.8*(1+43.7/100) < 1.21
    print(f"PASS: {len(expected)} components and {sum(net is not None for component in expected for net in component['nets'].values())} connected pins match KiCad export")


if __name__ == '__main__':
    validate()
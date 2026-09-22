"""Check generated syntax, KiCad connectivity, and critical daughterboard invariants."""

from pathlib import Path
import json
import subprocess
import tempfile
import xml.etree.ElementTree as ET
import sexpdata

ROOT = Path(__file__).resolve().parent
CLI = Path(r"C:\Program Files\KiCad\9.0\bin\kicad-cli.exe")


def validate():
    for path in ROOT.glob('*.kicad_sch'):
        parsed = sexpdata.loads(path.read_text(encoding='utf-8'))
        assert str(parsed[0]) == 'kicad_sch', path
    expected = [component for component in json.loads((ROOT/'connectivity.json').read_text()) if component['physical']]
    with tempfile.TemporaryDirectory() as directory:
        netlist_path = Path(directory)/'netlist.xml'
        subprocess.run([str(CLI), 'sch', 'export', 'netlist', '--format', 'kicadxml', '--output', str(netlist_path), str(ROOT/'XG_Mobile_USB_Hub.kicad_sch')], check=True)
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
"""Check generated syntax, KiCad connectivity, and critical daughterboard invariants."""

from pathlib import Path
import csv
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
    local_labels = []
    global_labels = []
    power_symbols = []
    for path in PROJECT_ROOT.glob('XG_Mobile_USB_Hub*.kicad_sch'):
        parsed = sexpdata.loads(path.read_text(encoding='utf-8'))
        assert str(parsed[0]) == 'kicad_sch', path
        for item in parsed:
            if isinstance(item,list) and str(item[0]) in ['label','global_label']:
                (local_labels if str(item[0])=='label' else global_labels).append(str(item[1]))
                effect = next(field for field in item if isinstance(field,list) and str(field[0])=='effects')
                font = next(field for field in effect if isinstance(field,list) and str(field[0])=='font')
                size = next(field for field in font if isinstance(field,list) and str(field[0])=='size')
                assert size[1:] == [1.27,1.27]
            if isinstance(item,list) and str(item[0])=='symbol':
                fields = {str(field[0]):field for field in item if isinstance(field,list)}
                properties = {str(field[1]):str(field[2]) for field in item
                              if isinstance(field,list) and str(field[0])=='property'}
                if properties.get('Reference','').startswith('#PWR'):
                    power_symbols.append(item)
                    assert str(fields['in_bom'][1]) == str(fields['on_board'][1]) == 'no'
            if isinstance(item,list) and str(item[0])=='wire':
                points = next(field for field in item if isinstance(field,list) and str(field[0])=='pts')[1:]
                length = abs(float(points[0][1])-float(points[1][1]))+abs(float(points[0][2])-float(points[1][2]))
                direct_wires += length>5.09
    assert direct_wires>=100, 'Expected directly wired functional groups, not only label stubs'
    assert len(local_labels)>100 and len(power_symbols)>100
    assert not {'GND','VIN12','V5','V3V3','VCORE'} & set(local_labels+global_labels)
    expected = [component for component in json.loads((ROOT/'connectivity.json').read_text()) if component['physical']]
    assert {component['sheet'] for component in expected} == {'Power','MCU','USB_Hub'}
    net_sheets = {}
    for component in expected:
        for net in component['nets'].values():
            if net:
                net_sheets.setdefault(net,set()).add(component['sheet'])
    assert all(len(net_sheets[net])==1 for net in local_labels)
    assert all(len(net_sheets[net])>1 for net in global_labels)
    with tempfile.TemporaryDirectory() as directory:
        netlist_path = Path(directory)/'netlist.xml'
        subprocess.run([str(CLI), 'sch', 'export', 'netlist', '--format', 'kicadxml', '--output', str(netlist_path), str(PROJECT_ROOT/'XG_Mobile_USB_Hub.kicad_sch')], check=True)
        netlist = ET.parse(netlist_path)
    actual_full = {(node.get('ref'), node.get('pin')): net.get('name')
              for net in netlist.findall('.//nets/net') for node in net.findall('node')}
    actual = {pin:net.rsplit('/',1)[-1] for pin,net in actual_full.items()}
    expected_groups,actual_groups = {},{}
    for component in expected:
        for pin,net in component['nets'].items():
            if net is not None:
                node = (component['ref'],pin)
                expected_groups.setdefault(net,set()).add(node)
                actual_groups.setdefault(actual_full.get(node),set()).add(node)
    assert {frozenset(nodes) for nodes in expected_groups.values()} == {frozenset(nodes) for nodes in actual_groups.values()}, 'Net split or short after label conversion'
    errors = [(component['ref'], pin, net, actual.get((component['ref'], pin)))
              for component in expected for pin, net in component['nets'].items()
              if net is not None and actual.get((component['ref'], pin)) != net]
    assert not errors, errors
    assert len(netlist.findall('.//components/comp')) == len(expected)
    components = {component['ref']: component for component in expected}
    sourced_ics = {
        'U101': ('LM51772RHAR','C41383743'),
        'U201': ('TPS40305DRCR','C140285'),
        'U301': ('TPS56A37RPAR','C22392669'),
        'U401': ('TPS62130RGTR','C43590'),
        'U501': ('USB7206CT/KDX','C3210691'),
        'U600': ('HD3SS3220IRNHR','C701817'),
        'U700': ('HD3SS3220IRNHR','C701817'),
        'U1200': ('TCPP03-M20','C3662955'),
        'U1201': ('HD3SS3212IRKSR','C544517'),
        'U1300': ('STM32G071CBT6','C432212'),
    }
    for ref, identifiers in sourced_ics.items():
        assert (components[ref]['mpn'],components[ref]['lcsc']) == identifiers, ref
    with (ROOT/'BOM.csv').open(newline='',encoding='utf-8') as stream:
        bom = csv.DictReader(stream)
        assert 'Part_Number' in bom.fieldnames and 'MPN' not in bom.fieldnames
        bom_rows = list(bom)
    assert len(bom_rows) == len(expected)
    for component in netlist.findall('.//components/comp'):
        fields = {field.get('name'):field.text or '' for field in component.findall('./fields/field')}
        assert 'MPN' not in fields
        assert fields['Part_Number'] == components[component.get('ref')]['mpn']
        assert fields.get('LCSC','') == components[component.get('ref')]['lcsc']
    for ref in ['J600','J700','J1200']:
        nets = components[ref]['nets']
        assert nets['A6'] == nets['B6']
        assert nets['A7'] == nets['B7']
        assert len({nets[pin] for pin in ['A4','A9','B4','B9']}) == 1
    assert components['U501']['nets']['9'] == 'VCORE'
    assert components['U501']['nets']['2'] == 'HUB_VBUS_DET'
    assert components['U101']['nets']['5'] == components['U1300']['nets']['46'] == 'I2C_SDA'
    assert components['U101']['nets']['6'] == components['U1300']['nets']['45'] == 'I2C_SCL'
    assert components['U101']['nets']['9'] == 'GND'
    assert components['U101']['nets']['14'] == 'PD_VCC2'
    assert components['U1300']['nets']['28'] == components['U1200']['nets']['1'] == 'MCU_CC1'
    assert components['U1300']['nets']['27'] == components['U1200']['nets']['3'] == 'MCU_CC2'
    assert components['U1300']['footprint'] == 'Package_QFP:LQFP-48_7x7mm_P0.5mm'
    assert set(components['U1300']['nets']) == {str(pin) for pin in range(1,49)}
    for pin in ['4','5','6']:
        assert components['U1300']['nets'][pin] == 'V3V3'
    for pin in ['7','29','32']:
        assert components['U1300']['nets'][pin] == 'GND'
    for pin,net in {'10':'MCU_NRST','35':'SWDIO','36':'SWCLK','38':'UP_FAULT_N',
                    '39':'PD_NFLT','40':'UP_MUX_SEL','41':'UP_MUX_OEN','48':'3V3_PG'}.items():
        assert components['U1300']['nets'][pin] == net
    for ref in ['C1304','C1305','C1306']:
        assert components[ref]['nets'] == {'1':'V3V3','2':'GND'}
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
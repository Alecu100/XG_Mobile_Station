"""Generate the editable KiCad daughterboard design and its connectivity manifest."""

from pathlib import Path
import csv
import json
import uuid
import sexpdata


ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = ROOT.parent
PROJECT = "XG_Mobile_USB_Hub"
NAMESPACE = uuid.UUID("eb5758b8-41e4-43f1-b10f-1330e4bc0380")
LIBRARY = {}
SHEETS = []


def uid(name):
    return str(uuid.uuid5(NAMESPACE, name))


def quote(value):
    return json.dumps(str(value), ensure_ascii=True)


def effects(size=1.0, extra=""):
    return f"(effects (font (size {size} {size})) {extra})"


def define(name, pins, footprint="", datasheet="", shape="ic"):
    LIBRARY[name] = dict(pins=pins, footprint=footprint, datasheet=datasheet, shape=shape)


def sheet(name, title, notes):
    page = dict(name=name, title=title, notes=notes, parts=[])
    SHEETS.append(page)
    return page


def part(page, ref, kind, nets, value=None, mpn="", lcsc="", dnp=False, footprint=None):
    specification = LIBRARY[kind]
    assert set(nets) == {str(pin[0]) for pin in specification["pins"]}, (ref, nets)
    page["parts"].append(dict(ref=ref, kind=kind, nets=nets, value=value or kind,
                              mpn=mpn or value or kind, lcsc=lcsc, dnp=dnp,
                              physical=kind != 'Supply_Flag',
                              footprint=specification['footprint'] if footprint is None else footprint))


def passive(page, ref, value, first, second, mpn="", kind=None, lcsc=""):
    kind = kind or ref[0]
    part(page, ref, kind, {"1": first, "2": second}, value, mpn or 'SELECTION_REQUIRED', lcsc)


def symbol_definition(name, embedded=True):
    spec = LIBRARY[name]
    pins = spec["pins"]
    count = (len(pins) + 1) // 2
    half_height = max(5.08, (count + 1) * 1.27)
    half_width = 20.32 if spec["shape"] == "ic" else 5.08
    lib_name = f"Daughterboard:{name}" if embedded else name
    drawing = []
    if spec["shape"] == "C":
        for location in (-0.635, 0.635):
            drawing.append(f"(polyline (pts (xy {location} -2.54) (xy {location} 2.54)) (stroke (width 0.254) (type default)) (fill (type none)))")
        for start, end in ((-5.08, -0.635), (0.635, 5.08)):
            drawing.append(f"(polyline (pts (xy {start} 0) (xy {end} 0)) (stroke (width 0) (type default)) (fill (type none)))")
    else:
        drawing.append(f"(rectangle (start {-half_width} {half_height if spec['shape']=='ic' else 1.27}) (end {half_width} {-half_height if spec['shape']=='ic' else -1.27}) (stroke (width 0.254) (type default)) (fill (type background)))")
    geometry = []
    for index, (number, label, electrical) in enumerate(pins):
        left = index < count
        row = index if left else index - count
        position_y = (count - 1) * 1.27 - row * 2.54 if len(pins) > 2 else 0
        position_x = (-1 if left else 1) * (half_width + 5.08)
        angle = 0 if left else 180
        drawing.append(f"(pin {electrical} line (at {position_x} {position_y} {angle}) (length 5.08) (name {quote(label)} {effects()}) (number {quote(number)} {effects()}))")
        geometry.append((str(number), position_x, position_y, left))
    spec["geometry"] = geometry
    spec["height"] = half_height * 2
    physical = 'no' if name == 'Supply_Flag' else 'yes'
    return f'''(symbol {quote(lib_name)} (pin_names (offset 0.508)) (in_bom {physical}) (on_board {physical})
      (property "Reference" "U" (at 0 {half_height+5.08} 0) {effects()})
      (property "Value" {quote(name)} (at 0 {half_height+2.54} 0) {effects()})
      (property "Footprint" {quote(spec['footprint'])} (at 0 0 0) {effects(extra='hide')})
      (property "Datasheet" {quote(spec['datasheet'])} (at 0 0 0) {effects(extra='hide')})
      (symbol {quote(name+'_1_1')} {' '.join(drawing)}))'''


def header(identity, title, paper='A2'):
    paper_spec = '"User" ' + paper[5:] if paper.startswith('User ') else quote(paper)
    return f'''(kicad_sch (version 20231120) (generator "eeschema")
    (uuid {quote(identity)}) (paper {paper_spec})
      (title_block (title {quote(title)}) (rev "DRAFT A")
        (comment 1 "Engineering draft - not released for manufacture")
        (comment 2 "12 V input / 100 W upstream USB-PD / 2 USB-C + 4 USB-A downstream"))'''


def text(content, xpos, ypos, size=1.27):
    return f"(text {quote(content)} (at {xpos} {ypos} 0) {effects(size, '(justify left top)')} (uuid {quote(uid(f'text/{xpos}/{ypos}/{content}'))}))"


def wire(identity, start, end):
    start = tuple(round(value,4) for value in start)
    end = tuple(round(value,4) for value in end)
    assert start != end
    return f'(wire (pts (xy {start[0]} {start[1]}) (xy {end[0]} {end[1]})) (stroke (width 0) (type default)) (uuid {quote(uid(identity))}))'


def net_label(identity, net, xpos, ypos, angle=0):
    justify = '(justify left)' if angle==0 else '(justify right)'
    return f'(global_label {quote(net)} (shape input) (at {round(xpos,4)} {round(ypos,4)} {angle}) {effects(0.9,justify)} (uuid {quote(uid(identity))}))'


def passive_groups(section):
    rails = {'GND','VIN12','V5','V3V3','VCORE','PD_RAW','PD_VCC1','PD_VCC2'}
    candidates = [part for part in section['parts'] if part['kind'] in ['R','C','L','F']]
    owners = {part['ref']:part['ref'] for part in candidates}

    def owner(ref):
        while owners[ref] != ref:
            ref = owners[ref]
        return ref

    def boundary(net):
        return net in rails or net.endswith(('_VBUS','_12V'))

    for index,part in enumerate(candidates):
        nets = set(part['nets'].values())
        for other in candidates[:index]:
            other_nets = set(other['nets'].values())
            if nets == other_nets or any(not boundary(net) for net in nets & other_nets):
                owners[owner(part['ref'])] = owner(other['ref'])
    groups = {}
    for part in candidates:
        groups.setdefault(owner(part['ref']),[]).append(part)

    def path_nodes(parts):
        neighbors = {}
        for part in parts:
            first,second = part['nets'].values()
            neighbors.setdefault(first,set()).add(second)
            neighbors.setdefault(second,set()).add(first)
        ends = [net for net,near in neighbors.items() if len(near)==1]
        if len(ends)!=2 or len(neighbors)>5 or any(len(near)>2 for near in neighbors.values()):
            return None
        nodes = [next((net for net in ends if net!='GND'),ends[0])]
        while len(nodes)<len(neighbors):
            remaining = neighbors[nodes[-1]]-set(nodes)
            if not remaining:
                return None
            nodes.append(next(iter(remaining)))
        return nodes

    result = []
    for parts in groups.values():
        nodes = path_nodes(parts)
        if len(parts)>1 and nodes:
            if len(nodes)==2:
                result.extend((parts[index:index+4],nodes) for index in range(0,len(parts),4))
            else:
                result.append((parts,nodes))
        else:
            pairs = {}
            for part in parts:
                pairs.setdefault(tuple(sorted(part['nets'].values())),[]).append(part)
            result.extend((bank,path_nodes(bank)) for bank in pairs.values() if len(bank)>1)
    return result


def section_layout(section, left, top):
    networks = passive_groups(section)
    membership = {part['ref']:index for index,(parts,nodes) in enumerate(networks) for part in parts}
    attachments = {}
    attached_refs = set()
    rails = {'GND','VIN12','V5','V3V3','VCORE','PD_RAW','PD_VCC1','PD_VCC2'}
    for device in section['parts']:
        if not device['ref'].startswith('U'):
            continue
        taken = []
        for part in section['parts']:
            if part['kind'] not in ['R','C'] or part['ref'] in membership or part['ref'] in attached_refs:
                continue
            signals = [(number,net) for number,net in part['nets'].items()
                       if net not in rails and not net.endswith(('_VBUS','_12V'))]
            if len(signals)!=1:
                continue
            part_pin,net = signals[0]
            for pin,offset_x,offset_y,pin_left in LIBRARY[device['kind']]['geometry']:
                if device['nets'][pin]!=net or any(side==pin_left and abs(offset_y-height)<17.78 for side,height in taken):
                    continue
                attachments.setdefault(device['ref'],[]).append((part,part_pin,pin,offset_x,offset_y,pin_left))
                attached_refs.add(part['ref'])
                taken.append((pin_left,offset_y))
                break
    emitted = set()
    placements, drawings, connected = {}, [], set()
    cursor_x, cursor_y, row_height = 0,round((top+29)/2.54)*2.54,0
    for component in section['parts']:
        if component['ref'] in attached_refs:
            continue
        group_index = membership.get(component['ref'])
        if group_index is not None and group_index in emitted:
            continue
        if group_index is None:
            spec = LIBRARY[component['kind']]
            width = 254 if component['ref'] in attachments else 139.7
            height = max(spec['height']+17.78,30.48)
        else:
            emitted.add(group_index)
            parts,nodes = networks[group_index]
            lanes = {}
            for part in parts:
                edge = min(nodes.index(net) for net in part['nets'].values())
                lanes.setdefault(edge,[]).append(part)
            width = (len(nodes)-1)*50.8+40.64
            height = max(len(bank) for bank in lanes.values())*25.4+15.24
        if cursor_x+width>558.8:
            cursor_x,cursor_y,row_height = 0,cursor_y+row_height+7.62,0
        origin = round((left+cursor_x)/2.54)*2.54
        if group_index is None:
            device_x = origin+(127 if component['ref'] in attachments else 63.5)
            placements[component['ref']] = (device_x,cursor_y)
            device_y = cursor_y+spec['height']/2+7.62
            for part,part_pin,pin,offset_x,offset_y,pin_left in attachments.get(component['ref'],[]):
                direction = -1 if pin_left else 1
                passive_x,passive_y = device_x+direction*60.96,device_y-offset_y
                rotation = 0 if (part_pin=='2')==pin_left else 180
                placements[part['ref']] = (passive_x,passive_y-LIBRARY[part['kind']]['height']/2-7.62,rotation,True)
                near_x = passive_x-direction*10.16
                drawings.append(wire(part['ref']+'/to-device',(device_x+offset_x,passive_y),(near_x,passive_y)))
                drawings.append(net_label(part['ref']+'/signal',part['nets'][part_pin],device_x+offset_x+direction*7.62,passive_y,0 if pin_left else 180))
                other_pin = '2' if part_pin=='1' else '1'
                far_x = passive_x+direction*10.16
                end_x = far_x+direction*5.08
                drawings.append(wire(part['ref']+'/to-rail',(far_x,passive_y),(end_x,passive_y)))
                drawings.append(net_label(part['ref']+'/rail',part['nets'][other_pin],end_x,passive_y,0 if pin_left else 180))
                connected.update([(component['ref'],pin),(part['ref'],'1'),(part['ref'],'2')])
        else:
            rail_points = {net:[] for net in nodes}
            for edge,bank in lanes.items():
                for row,part in enumerate(bank):
                    center_x = origin+20.32+edge*50.8+25.4
                    center_y = cursor_y+17.78+row*25.4
                    placements[part['ref']] = (center_x,center_y-LIBRARY[part['kind']]['height']/2-7.62)
                    for number,offset_x,offset_y,pin_left in LIBRARY[part['kind']]['geometry']:
                        net = part['nets'][number]
                        rail_x = origin+20.32+nodes.index(net)*50.8
                        pin_x = center_x+offset_x
                        expected_left = rail_x<center_x
                        if expected_left != pin_left:
                            placements[part['ref']] = (center_x,placements[part['ref']][1],180)
                            pin_x = center_x-offset_x
                        drawings.append(wire(part['ref']+'/'+number+'/direct',(pin_x,center_y),(rail_x,center_y)))
                        rail_points[net].append(center_y)
                        connected.add((part['ref'],number))
            for net,points in rail_points.items():
                rail_x = origin+20.32+nodes.index(net)*50.8
                rail_top = cursor_y+7.62
                drawings.append(net_label(section['name']+'/'+str(group_index)+'/'+net,net,rail_x,rail_top))
                previous = rail_top
                for index,rail_y in enumerate(sorted(set(points))):
                    drawings.append(wire(section['name']+'/'+str(group_index)+'/'+net+'/'+str(index),(rail_x,previous),(rail_x,rail_y)))
                    if len(points)>1:
                        drawings.append(f'(junction (at {round(rail_x,4)} {round(rail_y,4)}) (diameter 0) (color 0 0 0 0) (uuid {quote(uid(section["name"]+str(group_index)+net+str(index)+"junction"))}))')
                    previous = rail_y
        cursor_x += width
        row_height = max(row_height,height)
    return placements,drawings,connected,cursor_y+row_height+12


def generate():
    root_id = uid('USB_C_Daughterboard')
    root = [header(root_id, PROJECT.replace('_', ' '), 'A3'), '(lib_symbols)']
    manifest = []
    layout_report = []
    for page_number, page in enumerate(SHEETS, 2):
        page_id = uid(page['name'])
        sheet_id = uid('sheet/' + page['name'])
        names = sorted({component['kind'] for component in page['parts']})
        symbols = '\n'.join(symbol_definition(name) for name in names)
        placements = {}
        direct_wires = []
        direct_pins = set()
        section_text = []
        column_bottoms = [45,45]
        for section in page['sections']:
            panel = min(range(2), key=lambda index:column_bottoms[index])
            left, top = 12+panel*584.2, column_bottoms[panel]
            section_text.append(text(section['title'],left,top,2))
            section_text.append(text(section['notes'],left,top+7,1.1))
            section_positions,section_wires,section_pins,bottom = section_layout(section,left,top)
            placements.update(section_positions)
            direct_wires.extend(section_wires)
            direct_pins.update(section_pins)
            column_bottoms[panel] = bottom
        page_height = max(297,max(column_bottoms)+55)
        output = [header(page_id, page['title'], f'User 1189 {page_height}'), '(lib_symbols ' + symbols + ')']
        output.append(text(page['title'], 12, 15, 2.54))
        output.append(text(page['notes'], 12, 23))
        output.extend(section_text)
        output.extend(direct_wires)
        for component in page['parts']:
            ref, kind = component['ref'], component['kind']
            spec = LIBRARY[kind]
            xpos,ypos,*rotation = placements[ref]
            symbol_angle = rotation[0] if rotation else 0
            center_y = ypos + spec['height'] / 2 + 7.62
            assert center_y + spec['height']/2 < page_height-55, (page['name'],ref)
            component_id = uid(ref)
            properties = []
            for key, value in [('Reference', ref), ('Value', component['value']), ('Footprint', component['footprint']), ('Datasheet', spec['datasheet']), ('MPN', component['mpn']), ('LCSC', component['lcsc'])]:
                prop_y = ypos if key == 'Reference' else ypos+2.54
                if len(rotation)>1 and rotation[1]:
                    prop_y = center_y-(5.08 if key=='Reference' else 2.54)
                properties.append(f"(property {quote(key)} {quote(value)} (at {xpos} {prop_y} 0) {effects(extra='' if key in ('Reference','Value') else 'hide')})")
            output.append(f'''(symbol (lib_id {quote('Daughterboard:'+kind)}) (at {xpos} {center_y} {symbol_angle}) (unit 1)
              (in_bom {'yes' if component['physical'] else 'no'}) (on_board {'yes' if component['physical'] else 'no'}) (dnp {'yes' if component['dnp'] else 'no'}) (uuid {quote(component_id)})
              {' '.join(properties)}
              (instances (project {quote(PROJECT)} (path {quote('/'+root_id+'/'+sheet_id)} (reference {quote(ref)}) (unit 1)))))''')
            for number, offset_x, offset_y, left in spec['geometry']:
                if (ref,number) in direct_pins:
                    continue
                pin_x, pin_y = round(xpos+offset_x, 4), round(center_y-offset_y, 4)
                net = component['nets'][number]
                if net is None:
                    output.append(f"(no_connect (at {pin_x} {pin_y}) (uuid {quote(uid(ref+'/'+number+'/nc'))}))")
                    continue
                end_x = round(pin_x + (-5.08 if left else 5.08), 4)
                output.append(f"(wire (pts (xy {pin_x} {pin_y}) (xy {end_x} {pin_y})) (stroke (width 0) (type default)) (uuid {quote(uid(ref+'/'+number+'/wire'))}))")
                angle = 0 if left else 180
                output.append(f'''(global_label {quote(net)} (shape input) (at {end_x} {pin_y} {angle})
                  {effects(0.9, '(justify left)' if left else '(justify right)')} (uuid {quote(uid(ref+'/'+number+'/label'))}))''')
            manifest.append(dict(sheet=page['name'], **component))
        output.append(')')
        layout_report.append({'sheet':page['name'],
                      'directly_wired_components':sorted({ref for ref,number in direct_pins}),
                      'directly_wired_pins':len(direct_pins),
                      'global_labels':sum(item.count('(global_label ') for item in output)})
        (PROJECT_ROOT / (PROJECT+'_'+page['name']+'.kicad_sch')).write_text('\n'.join(output)+'\n', encoding='utf-8')
        index = page_number-2
        xpos, ypos_root = 20, 65+index*55
        root.append(f'''(sheet (at {xpos} {ypos_root}) (size 160 30) (stroke (width 0) (type default)) (fill (color 0 0 0 0))
          (uuid {quote(sheet_id)})
          (property "Sheetname" {quote(page['title'])} (at {xpos} {ypos_root-1.27} 0) {effects(1.27,'(justify left bottom)')})
          (property "Sheetfile" {quote(PROJECT+'_'+page['name']+'.kicad_sch')} (at {xpos} {ypos_root+31.27} 0) {effects(1.27,'(justify left top)')})
          (instances (project {quote(PROJECT)} (path {quote('/'+root_id)} (page {quote(page_number)})))))''')
    root.append(text('USB-C DAUGHTERBOARD / ENGINEERING DRAFT', 20, 15, 3))
    root.append(text('Upstream: USB-C, USB 3.2 Gen 2 data UFP, USB-PD power source, 20 V / 5 A maximum.\nDownstream: 2 Type-C + 3 Type-A SuperSpeed, 1 Type-A USB 2.0.\nUse two GPU power inputs OR one EPS input. Not for simultaneous independent supplies.', 20, 24))
    root.append('(sheet_instances (path "/" (page "1")))')
    root.append(')')
    (PROJECT_ROOT / (PROJECT+'.kicad_sch')).write_text('\n'.join(root)+'\n', encoding='utf-8')
    (ROOT / 'Daughterboard.kicad_sym').write_text('(kicad_symbol_lib (version 20231120) (generator "kicad_symbol_editor")\n'+'\n'.join(symbol_definition(name, False) for name in sorted(LIBRARY))+'\n)\n', encoding='utf-8')
    project_path = PROJECT_ROOT / (PROJECT+'.kicad_pro')
    if not project_path.exists():
        project_path.write_text(json.dumps({'meta':{'filename':PROJECT+'.kicad_pro','version':1}},indent=2)+'\n',encoding='utf-8')
    (ROOT / 'connectivity.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf-8')
    (ROOT / 'Schematic_Layout_Report.json').write_text(json.dumps(layout_report,indent=2)+'\n',encoding='utf-8')
    with (ROOT / 'BOM.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['Reference', 'Value', 'MPN', 'LCSC', 'Footprint', 'DNP', 'Sheet'])
        for component in manifest:
            if not component['physical']:
                continue
            writer.writerow([component['ref'], component['value'], component['mpn'], component['lcsc'], component['footprint'], component['dnp'], component['sheet']])
    print(f"Generated {len(SHEETS)} sheets, {len(manifest)} components")


for device, footprint in [('R','Resistor_SMD:R_0603_1608Metric'), ('C','Capacitor_SMD:C_0603_1608Metric'), ('L',''), ('F','')]:
    define(device, [('1', '', 'passive'), ('2', '', 'passive')], footprint, shape=device)

define('TPS56A37RPAR', [('1','EN','input'), ('2','FB','input'), ('3','AGND','power_in'), ('4','PG','open_collector'), ('5','SS','output'), ('6','SW','power_out'), ('7','BOOT','passive'), ('8','VIN','power_in'), ('9','PGND','power_in'), ('10','MODE','input')], datasheet='https://www.ti.com/lit/ds/symlink/tps56a37.pdf')
power33 = sheet('Power_3V3', '3.3 V / 4 A - TPS56A37', 'WEBENCH Design104 values. EN intentionally floating per datasheet section 6.3.5.\nAGND/PGND join at regulator. Verify capacitance after DC-bias derating and inductor thermal limits.')
part(power33, 'U301', 'TPS56A37RPAR', {'1':None,'2':'3V3_FB','3':'GND','4':'3V3_PG','5':'3V3_SS','6':'3V3_SW','7':'3V3_BOOT','8':'VIN12','9':'GND','10':'3V3_MODE'})
passive(power33,'L301','2.2uH','3V3_SW','V3V3','VLP8040T-2R2N')
for ref in ['C301','C302']:
    passive(power33,ref,'10uF 25V','VIN12','GND','MSAST21GBB5106MTNA01')
passive(power33,'C303','100nF 25V','VIN12','GND','C0603C104M3VACTU')
passive(power33,'C304','100nF 16V','3V3_BOOT','3V3_SW','EMK107B7104KA-T')
for ref in ['C305','C306']:
    passive(power33,ref,'15uF 16V','V3V3','GND','C3225X5R1C156M250AA')
passive(power33,'R301','45.3k','V3V3','3V3_FB','CRCW080545K3FKEA')
passive(power33,'R302','10k','3V3_FB','GND','RC0201FR-0710KL')
passive(power33,'C307','150pF','V3V3','3V3_FB','C0603C151K3GACTU')
passive(power33,'R303','52.3k','3V3_MODE','GND','CRCW040252K3FKED')
passive(power33,'C308','22nF','3V3_SS','GND','GRM155R71C223KA01D')
passive(power33,'R304','100k','V3V3','3V3_PG','CRCW0402100KFKED')


def standard(name, library, original=None):
    source = Path(r'C:\Program Files\KiCad\9.0\share\kicad\symbols') / (library+'.kicad_sym')
    entries = {str(item[1]): item for item in sexpdata.loads(source.read_text(encoding='utf-8'))
               if isinstance(item, list) and str(item[0]) == 'symbol'}
    entry = entries[original or name]
    properties = {}
    pins = {}

    def collect(symbol):
        for item in symbol:
            if not isinstance(item, list):
                continue
            if str(item[0]) == 'extends':
                collect(entries[str(item[1])])
            elif str(item[0]) == 'property':
                properties[str(item[1])] = str(item[2])
            elif str(item[0]) == 'symbol':
                for pin in item:
                    if isinstance(pin, list) and str(pin[0]) == 'pin':
                        fields = {str(field[0]): field[1] for field in pin if isinstance(field, list)}
                        pins[str(fields['number'])] = (str(fields['number']), str(fields['name']), str(pin[1]))
    collect(entry)
    define(name, list(pins.values()), properties.get('Footprint', ''), properties.get('Datasheet', ''))


standard('CSD17577Q3A', 'Transistor_FET')
define('CSD17304Q3', LIBRARY['CSD17577Q3A']['pins'], LIBRARY['CSD17577Q3A']['footprint'], 'https://www.ti.com/lit/ds/symlink/csd17304q3.pdf')
define('TPS40305DRCR', [('1','VDD','power_in'),('2','EN_SS','input'),('3','PGOOD','open_collector'),('4','COMP','output'),('5','FB','input'),('6','BOOT','passive'),('7','HDRV','output'),('8','SW','passive'),('9','LDRV_OC','output'),('10','BP','power_out'),('11','GND_EP','power_in')], 'Package_SON:VSON-10-1EP_3x3mm_P0.5mm_EP1.2x2mm', 'https://www.ti.com/lit/ds/symlink/tps40305.pdf')
power5 = sheet('Power_5V', '5 V / 12 A - TPS40305', 'WEBENCH Design105 topology and values; 1.2 MHz. MOSFET drain pad numbering follows TI NexFET land pattern.\n5 V allocation: two Type-C ports at 3 A, USB-A SDP loads, plus VCONN/mux overhead. BC1.2 disabled.')
part(power5,'U201','TPS40305DRCR',dict(zip(map(str,range(1,12)),['VIN12','5V_SS','5V_PG','5V_COMP','5V_FB','5V_BOOT','5V_HG','5V_SW','5V_LG','5V_BP','GND'])))
part(power5,'Q201','CSD17304Q3',{'1':'5V_SW','2':'5V_SW','3':'5V_SW','4':'5V_HG','5':'VIN12'})
part(power5,'Q202','CSD17577Q3A',{'1':'GND','2':'GND','3':'GND','4':'5V_LG','5':'5V_SW'})
passive(power5,'L201','800nH','5V_SW','V5','XAL7070-801MEB')
for number in range(201,204):
    passive(power5,f'C{number}','10uF 25V','VIN12','GND','GRM31CR71E106KA12L')
for ref,value,first,second,mpn in [
    ('C204','1uF 25V','VIN12','GND','C1005X5R1E105K050BC'),
    ('C205','2.2uF 10V','5V_BP','GND','GRM21BR71A225KA01L'),
    ('C206','100nF 16V','5V_BOOT','5V_SW','GRM155R71C104KA88D'),
    ('C207','100uF 10V','V5','GND','C3216X5R1A107M160AC'),
    ('C208','3.3nF','5V_SS','GND','GRM033R71A332KA01D'),
    ('R201','100k','5V_BP','5V_PG','CRCW0402100KFKED'),
    ('R202','7.5k','5V_LG','GND','CRCW04027K50FKED'),
    ('R203','10k','V5','5V_FB','RC0201FR-0710KL'),
    ('R204','1.37k','5V_FB','GND','CRCW04021K37FKED'),
    ('C209','120pF','5V_COMP','5V_FB','GRM0335C1H121JA01D'),
    ('C210','4.3nF','5V_COMP','5V_COMP_MID','C0603C432J5GAC7867'),
    ('R205','2.26k','5V_COMP_MID','5V_FB','CRCW04022K26FKED'),
    ('R206','274','V5','5V_FF_MID','CRCW0402274RFKED'),
    ('C211','1nF','5V_FF_MID','5V_FB','GRM1555C1H102JA01J')]:
    passive(power5,ref,value,first,second,mpn)

standard('TPS62130', 'Regulator_Switching')
core = sheet('Power_Core', '1.15 V / 3 A - USB7206C core', 'TPS62130, 12 V input. 0.8 V reference x (1 + 43.7k/100k) = 1.1496 V.\nHub VCORE allowed range 1.09-1.21 V. Hold HUB_RESET_N low until both hub rails are stable.')
part(core,'U401','TPS62130',{'1':'CORE_SW','2':'CORE_SW','3':'CORE_SW','4':'CORE_PG','5':'CORE_FB','6':'GND','7':'GND','8':'GND','9':'CORE_SS','10':'VIN12','11':'VIN12','12':'VIN12','13':'V3V3','14':'VCORE','15':'GND','16':'GND','17':'GND'})
passive(core,'L401','2.2uH','CORE_SW','VCORE','XAL5030-222MEC')
passive(core,'R401','43.7k 0.1%','VCORE','CORE_FB')
passive(core,'R402','100k 0.1%','CORE_FB','GND')
passive(core,'R403','10k','V3V3','CORE_PG')
passive(core,'C401','10uF 25V','VIN12','GND','GRM31CR71E106KA12L')
passive(core,'C402','100nF 25V','VIN12','GND')
passive(core,'C403','22uF 6.3V','VCORE','GND','GRM21BR60J226ME39L')
passive(core,'C404','22uF 6.3V','VCORE','GND','GRM21BR60J226ME39L')
passive(core,'C405','3.3nF','CORE_SS','GND')


hub_names = '''RESET_N VBUS_DET PF31 NC4 USB2_DP1 USB2_DM1 USB3_TXP1 USB3_TXN1 VCORE USB3_RXP1 USB3_RXN1 NC12 NC13 USB2_DP2 USB2_DM2 USB3_TXP2 USB3_TXN2 VCORE USB3_RXP2 USB3_RXN2 CFG_STRAP1 CFG_STRAP2 CFG_STRAP3 TESTEN VCORE VDD33 USB2_DP3 USB2_DM3 USB3_TXP3 USB3_TXN3 VCORE USB3_RXP3 USB3_RXN3 USB2_DP4 USB2_DM4 USB3_TXP4 USB3_TXN4 VCORE USB3_RXP4 USB3_RXN4 USB2_DM6 USB2_DP6 VDD33 PF3 PF4 PF5 PF6 PF7 PF8 PF9 PF10 PF11 VDD33 PF12 VCORE PF13 PF14 PF15 PF16 PF17 PF18 VDD33 TEST1 TEST2 TEST3 PF19 VDD33 SPI_CLK SPI_CE_N SPI_D0 SPI_D1 SPI_D2 SPI_D3 PF29 PF26 PF27 PF28 VCORE VDD33 NC80 USB2_DP5 USB2_DM5 USB3_TXP5 USB3_TXN5 VCORE USB3_RXP5 USB3_RXN5 VDD33 USB2UP_DP USB2UP_DM USB3UP_TXP USB3UP_TXN VCORE USB3UP_RXP USB3UP_RXN ATEST XTALO XTALI VDD33 RBIAS VSS_EP'''.split()
assert len(hub_names) == 101
define('USB7206CT_KDX', [(str(index),name,'power_in' if name in ['VCORE','VDD33','VSS_EP'] else 'bidirectional') for index,name in enumerate(hub_names,1)], datasheet='https://ww1.microchip.com/downloads/aemDocuments/documents/NCS/ProductDocuments/DataSheets/USB7206C-Data-Sheet-DS00003850.pdf')
hub = sheet('Hub', 'USB7206C - six downstream ports', 'Port 1/2: Type-C. Ports 3/4/5: Type-A SuperSpeed. Port 6: Type-A USB 2.0.\nConfiguration 3 per DS00003850F Tables 3-4/3-5. RESET and VBUS_DET driven by PD MCU, not high-voltage VBUS.')
hub_nets = {}
port_controls = {'PF17':1,'PF16':2,'PF15':3,'PF14':4,'PF13':5,'PF28':6}
for index,name in enumerate(hub_names,1):
    if name in ['VCORE','VDD33','VSS_EP']:
        net = {'VCORE':'VCORE','VDD33':'V3V3','VSS_EP':'GND'}[name]
    elif name.startswith('USB2_') or name.startswith('USB3_'):
        net = f"P{name[-1]}_{name.split('_')[1][:-1]}"
    elif name.startswith('USB2UP_') or name.startswith('USB3UP_'):
        net = 'UP_'+name.split('_')[1]
    elif name in port_controls:
        net = f'P{port_controls[name]}_CTL'
    elif name.startswith('NC') or name in ['ATEST','PF8','PF9','PF10','PF11','PF12']:
        net = None
    elif name == 'TESTEN':
        net = 'GND'
    else:
        net = 'HUB_'+name
    hub_nets[str(index)] = net
part(hub,'U501','USB7206CT_KDX',hub_nets,mpn='USB7206CT/KDX',lcsc='C3210691')
define('Crystal', [('1','XI','passive'),('3','XO','passive'),('2','GND','passive'),('4','GND','passive')])
part(hub,'Y501','Crystal',{'1':'HUB_XTALI','3':'HUB_XTALO','2':'GND','4':'GND'},'25MHz CL=18pF',mpn='ABM8-25.000MHZ-B2-T',footprint='Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm')
passive(hub,'C501','27pF C0G - tune CL','HUB_XTALI','GND')
passive(hub,'C502','27pF C0G - tune CL','HUB_XTALO','GND')
passive(hub,'R501','12k 1%','HUB_RBIAS','GND')
straps = [('CFG_STRAP1','10k','GND'),('CFG_STRAP2','200k','GND'),('CFG_STRAP3','200k','GND'),('SPI_CE_N','200k','GND'),('SPI_D0','200k','GND'),('TEST1','10k','V3V3'),('TEST2','10k','V3V3'),('TEST3','10k','V3V3')]
for index,(name,value,rail) in enumerate(straps,502):
    passive(hub,f'R{index}',value,'HUB_'+name,rail)
for index,name in enumerate(['PF3','PF4','PF5','PF6','PF7','PF18','PF19','PF31','SPI_CLK','SPI_D1','SPI_D2','SPI_D3','PF29'],510):
    passive(hub,f'R{index}','100k','HUB_'+name,'GND')
hub_nets['75'] = 'I2C_SCL'
hub_nets['76'] = 'I2C_SDA'
passive(hub,'R523','100k','HUB_RESET_N','GND')
passive(hub,'R524','100k','HUB_VBUS_DET','GND')
decoupling = sheet('Hub_Decoupling', 'Hub decoupling and transmit coupling', 'One 100 nF capacitor per VDD33/VCORE pin, placed at the relevant pin. Bulk capacitors near hub.\n220 nF series capacitors on each hub SuperSpeed TX lane only. Route 90 ohm differential pairs without stubs.')
for index in range(503,512):
    passive(decoupling,f'C{index}','100nF','VCORE','GND')
for index in range(512,521):
    passive(decoupling,f'C{index}','100nF','V3V3','GND')
passive(decoupling,'C521','10uF 6.3V','VCORE','GND')
passive(decoupling,'C522','10uF 6.3V','V3V3','GND')
for port_index,prefix in enumerate(['UP','P1','P2','P3','P4','P5']):
    for polarity_index,polarity in enumerate(['P','N']):
        passive(decoupling,f'C{523+2*port_index+polarity_index}','220nF 10V',prefix+'_TX'+polarity,prefix+'_TX'+polarity+'_AC')


for name,library in [('TPD4E05U06DQA','Power_Protection'),('74LVC1G04','74xGxx'),('74LVC1G08','74xGxx'),('USB3_A','Connector'),('USB_A','Connector'),('USB_C_Receptacle','Connector'),('2N7002','Transistor_FET')]:
    standard(name,library)
for name in ['USB3_A','USB_A']:
    LIBRARY[name]['pins'] = [(number,label,'passive') for number,label,electrical in LIBRARY[name]['pins']]
define('HD3SS3220IRNHR', [(str(index),name,'power_in' if name in ['VCC33','VDD5','GND','EP'] else 'bidirectional') for index,name in enumerate('CC2 CC1 CURRENT_MODE PORT VBUS_DET TXp TXn VCC33 RXp RXn DIR ENn_MUX GND RX1n RX1p TX1n TX1p RX2n RX2p TX2n TX2p ADDR OUT3 VCONN_FAULT_N OUT1 OUT2 ID EN_GND ENn_CC VDD5 EP'.split(),1)], datasheet='https://www.ti.com/lit/ds/symlink/hd3ss3220.pdf')
define('TPS259470LRPW', [(str(index),name,electrical) for index,name,electrical in [
    (1,'EN','input'),(2,'OVLO','input'),(3,'AUXOFF','open_collector'),(4,'FLT_N','open_collector'),(5,'IN','power_in'),(6,'OUT','power_out'),(7,'DVDT','passive'),(8,'GND','power_in'),(9,'ILM','passive'),(10,'ITIMER','passive')]], datasheet='https://www.ti.com/lit/ds/symlink/tps25947.pdf')


def esd(page, ref, nets):
    connections = {'1':nets[0],'2':nets[1],'3':'GND','4':nets[2],'5':nets[3],'6':None,'7':None,'8':'GND','9':None,'10':None}
    part(page,ref,'TPD4E05U06DQA',connections,mpn='TPD4E05U06DQAR')


def type_c(page,ref,prefix):
    mapping = {'SHIELD':'GND','GND':'GND','VBUS':prefix+'_VBUS','CC1':prefix+'_CC1','CC2':prefix+'_CC2','D+':prefix+'_DP','D-':prefix+'_DM','SBU1':None,'SBU2':None}
    for signal in ['RX1','RX2','TX1','TX2']:
        mapping[signal+'+'] = prefix+'_'+signal+'P'
        mapping[signal+'-'] = prefix+'_'+signal+'N'
    part(page,ref,'USB_C_Receptacle',{number:mapping[label] for number,label,electrical in LIBRARY['USB_C_Receptacle']['pins']},value='USB-C 24-pin 5A / 10Gbps',mpn='CONNECTOR_SELECTION_REQUIRED',footprint='')


def port_power(page,base,prefix,enable,limit):
    part(page,f'U{base+1}','TPS259470LRPW',{'1':enable,'2':prefix+'_OVLO','3':None,'4':prefix+'_CTL','5':'V5','6':prefix+'_VBUS','7':prefix+'_DVDT','8':'GND','9':prefix+'_ILM','10':None})
    passive(page,f'R{base+1}','36.5k','V5',prefix+'_OVLO')
    passive(page,f'R{base+2}','10k',prefix+'_OVLO','GND')
    passive(page,f'R{base+3}',limit,prefix+'_ILM','GND')
    passive(page,f'C{base+1}','2.2nF',prefix+'_DVDT','GND')
    passive(page,f'C{base+2}','1uF 10V','V5','GND')
    passive(page,f'C{base+3}','10uF 10V',prefix+'_VBUS','GND')


for port_index,base in [(1,600),(2,700)]:
    prefix = f'P{port_index}'
    page = sheet(f'USB_C{port_index}',f'Downstream Type-C {port_index} - 5 V / 3 A', 'HD3SS3220 GPIO mode: ADDR open, PORT high, CURRENT_MODE 10k to 5 V.\nVBUS enable = hub PRT_CTL AND attached. DIR has required 200k pull-up. eFuse nominal 3.27 A, latch-off.')
    type_c(page,f'J{base}',prefix)
    part(page,f'U{base}','HD3SS3220IRNHR',dict(zip(map(str,range(1,32)),[
        prefix+'_CC2',prefix+'_CC1',prefix+'_CURRENT',prefix+'_PORT',prefix+'_VBUS_DET',prefix+'_TXP_AC',prefix+'_TXN_AC','V3V3',prefix+'_RXP',prefix+'_RXN',prefix+'_DIR','GND','GND',prefix+'_RX1N',prefix+'_RX1P',prefix+'_TX1N',prefix+'_TX1P',prefix+'_RX2N',prefix+'_RX2P',prefix+'_TX2N',prefix+'_TX2P',None,None,prefix+'_VCONN_FLT',None,None,prefix+'_ID_N','GND','GND','V5','GND'])),lcsc='C701817')
    part(page,f'U{base+2}','74LVC1G04',{'1':None,'2':prefix+'_ID_N','3':'GND','4':prefix+'_ATTACHED','5':'V3V3'},mpn='SN74LVC1G04DBVR',footprint='Package_TO_SOT_SMD:SOT-23-5')
    part(page,f'U{base+3}','74LVC1G08',{'1':prefix+'_ATTACHED','2':prefix+'_CTL','3':'GND','4':prefix+'_EN','5':'V3V3'},mpn='SN74LVC1G08DBVR',footprint='Package_TO_SOT_SMD:SOT-23-5')
    port_power(page,base,prefix,prefix+'_EN','1.02k 1%')
    for offset,value,first,second in [(4,'10k','V5',prefix+'_CURRENT'),(5,'10k','V5',prefix+'_PORT'),(6,'200k','V3V3',prefix+'_DIR'),(7,'900k',prefix+'_VBUS',prefix+'_VBUS_DET'),(8,'10k','V3V3',prefix+'_ID_N'),(9,'10k','V3V3',prefix+'_VCONN_FLT'),(10,'100k',prefix+'_EN','GND'),(11,'330 0.25W',prefix+'_VBUS',prefix+'_DISCH')]:
        passive(page,f'R{base+offset}',value,first,second)
    part(page,f'Q{base}','2N7002',{'1':prefix+'_ID_N','2':'GND','3':prefix+'_DISCH'})
    for offset,rail in [(4,'V5'),(5,'V3V3'),(6,'V3V3'),(7,'V3V3')]:
        passive(page,f'C{base+offset}','100nF',rail,'GND')
    passive(page,f'C{base+8}','4.7uF 10V','V5','GND')
    for offset,nets in enumerate([[prefix+'_TX1P',prefix+'_TX1N',prefix+'_RX1P',prefix+'_RX1N'],[prefix+'_TX2P',prefix+'_TX2N',prefix+'_RX2P',prefix+'_RX2N'],[prefix+'_DP',prefix+'_DM',None,None]]):
        esd(page,f'D{base+offset}',nets)

for port_index,base in [(3,800),(4,900),(5,1000),(6,1100)]:
    prefix = f'P{port_index}'
    speed = 'USB 2.0 / 500 mA' if port_index == 6 else 'SuperSpeed / 900 mA'
    page = sheet(f'USB_A{port_index-2}',f'Downstream Type-A {port_index-2} - {speed}', 'TPS259470L provides per-port overcurrent protection and reverse-current blocking.\nFLT_N returns to the hub combined PRT_CTL/overcurrent pin. BC1.2 disabled. Connector selection/footprint pending.')
    nets = {'1':prefix+'_VBUS','2':prefix+'_DM','3':prefix+'_DP','4':'GND'}
    if port_index == 6:
        nets['5'] = 'GND'
        part(page,f'J{base}','USB_A',nets,mpn='USB_A_CONNECTOR_SELECTION_REQUIRED',footprint='')
    else:
        nets.update({'5':prefix+'_RXN','6':prefix+'_RXP','7':'GND','8':prefix+'_TXN_AC','9':prefix+'_TXP_AC','10':'GND'})
        part(page,f'J{base}','USB3_A',nets,mpn='USB3_A_CONNECTOR_SELECTION_REQUIRED',footprint='')
    port_power(page,base,prefix,prefix+'_CTL','5.11k 1%' if port_index == 6 else '3.01k 1%')
    passive(page,f'C{base+4}','220uF 10V +/-20% low ESR',prefix+'_VBUS','GND',
            mpn='USB_A_BULK_CAP_SELECTION_REQUIRED')
    page['parts'][-1]['footprint'] = ''
    esd(page,f'D{base}',[prefix+'_DP',prefix+'_DM',None,None])
    if port_index != 6:
        esd(page,f'D{base+1}',[prefix+'_TXP_AC',prefix+'_TXN_AC',prefix+'_RXP',prefix+'_RXN'])


lm_names = 'VCC1 SS_ATRK SYNC DTRK SDA SCL MODE CFG2 ADDR CDC NFLT RT COMP FB VIN_FB ISET AGND VOUT ISNSN ISNSP CSB CSA SW1 HO1 HB1 NC26 LO1 PGND VCC2 LO2 HB2 HO2 SW2 NC34 DRV VIN EN NRST NC39 BIAS EP'.split()
define('LM51772RHAR',[(str(index),name,'power_in' if name in ['VIN','BIAS','AGND','PGND','EP'] else 'bidirectional') for index,name in enumerate(lm_names,1)],datasheet='https://www.ti.com/lit/ds/symlink/lm51772.pdf')
pd = sheet('PD_BuckBoost','LM51772 - programmable 5-20 V / 100 W','WEBENCH Design103 adapted for I2C (ADDR=GND, address 0x6A), internal feedback and MCU enable/reset.\nAPS1040M2R2A replaces 1.8 uH. Compensation and current limit require recalculation/testing before power-up.\nAll bridge FETs standardized to CSD17577Q3A (30 V). Not an EPR design. Maximum contract 20 V / 5 A.')
part(pd,'U101','LM51772RHAR',dict(zip(map(str,range(1,42)),[
    'PD_VCC1','PD_SS','GND','GND','I2C_SDA','I2C_SCL','PD_VCC2','GND','GND','GND','PD_NFLT','PD_RT','PD_COMP','PD_VCC2','GND','PD_VCC2','GND','PD_RAW','GND','GND','PD_CSB','PD_CSA','PD_SW1','PD_HO1','PD_HB1',None,'PD_LO1','GND','PD_VCC2','PD_LO2','PD_HB2','PD_HO2','PD_SW2',None,None,'VIN12','PD_EN','PD_NRST',None,'VIN12','GND'])),lcsc='C41383743')
for ref,gate,source,drain in [('Q101','PD_HO1','PD_SW1','VIN12'),('Q102','PD_LO1','GND','PD_SW1'),('Q103','PD_HO2','PD_SW2','PD_RAW'),('Q104','PD_LO2','GND','PD_SW2')]:
    part(pd,ref,'CSD17577Q3A',{'1':source,'2':source,'3':source,'4':gate,'5':drain})
passive(pd,'R101','2m 2W - REVIEW OCP','PD_SW1','PD_L_IN','WSR22L000FEA')
passive(pd,'L101','2.2uH 12.12A / Isat 18.18A','PD_L_IN','PD_SW2','APS1040M2R2A',lcsc='C47327242')
for ref,value,first,second,mpn in [
    ('R102','10','PD_SW1','PD_CSA','CRCW060310R0FKEA'),('R103','10','PD_L_IN','PD_CSB','CRCW060310R0FKEA'),
    ('C101','180pF','PD_CSA','PD_CSB','C0805C181K5GACTU'),('R104','51k','PD_RT','GND','RC0603FR-0751KL'),
    ('R105','4.75k - REVIEW LOOP','PD_COMP_MID','GND','CRCW04024K75FKED'),('C102','120nF - REVIEW LOOP','PD_COMP','PD_COMP_MID','GRM188R71C124KA01D'),
    ('C103','220pF - REVIEW LOOP','PD_COMP','GND','CC0402JRNPO8BN221'),('C104','33nF','PD_SS','GND','GRM155R61C333KA01D'),
    ('C105','22uF 6.3V','PD_VCC1','GND','GRM188R60J226MEA0D'),('C106','22uF 6.3V','PD_VCC2','GND','GRM188R60J226MEA0D'),
    ('C107','100nF 16V','PD_HB1','PD_SW1','EMK107B7104KA-T'),('C108','100nF 16V','PD_HB2','PD_SW2','EMK107B7104KA-T'),
    ('C109','100nF 25V','VIN12','GND',''),('C110','10uF 25V','VIN12','GND','MSAST32NSB5106KTNA01'),
    ('C111','27uF 25V','VIN12','GND','25SVPF27MX'),('C112','120uF 35V','PD_RAW','GND','35SVPF120M'),
    ('C113','120uF 35V','PD_RAW','GND','35SVPF120M'),('C114','10uF 50V','PD_RAW','GND','C3225X7R1H106M250AC'),
    ('C115','10uF 50V','PD_RAW','GND','C3225X7R1H106M250AC'),('R106','10k','V3V3','PD_NFLT',''),
    ('R107','100k','PD_EN','GND',''),('R108','100k','PD_NRST','GND','')]:
    passive(pd,ref,value,first,second,mpn)

standard('TCPP03-M20','Interface_USB')
define('HD3SS3212IRKSR',[(str(index),name,'power_in' if name in ['VCC','GND','EP'] else 'bidirectional') for index,name in enumerate('RSVD1 OEN A0P A0N GND VCC A1P A1N SEL RSVD2 GND C1N C1P C0N C0P B1N B1P B0N B0P GND EP'.split(),1)],datasheet='https://www.ti.com/lit/ds/symlink/hd3ss3212.pdf')
upstream = sheet('USB_Upstream','Upstream Type-C - USB data UFP / PD source','STM32 UCPD negotiates PD and manages data-role swap to UFP while keeping power-source role.\nTCPP03 source path: common-source back-to-back FETs. Sink path not fitted. 7 mOhm gives nominal 6 A hardware OCP.\n100 W requires identified 5 A e-marked cable; otherwise cap contract to 3 A. Hardware OVP ceiling ~22 V.')
type_c(upstream,'J1200','UP')
part(upstream,'U1200','TCPP03-M20',{'1':'MCU_CC1','2':'V5','3':'MCU_CC2','4':'PD_IANA','5':'UP_GATE','6':'UP_FET_SOURCE','7':None,'8':'GND','9':'UP_VBUS','10':'UP_ISENSE','11':'UP_OVP','12':'GND','13':'UP_CC2','14':'UP_CBIAS','15':'UP_CC1','16':'GND','17':'I2C_SDA','18':'I2C_SCL','19':'UP_FAULT_N','20':'TCPP_EN','21':'GND'},lcsc='C3662955')
for ref,drain in [('Q1200','PD_RAW'),('Q1201','UP_ISENSE')]:
    part(upstream,ref,'CSD17577Q3A',{'1':'UP_FET_SOURCE','2':'UP_FET_SOURCE','3':'UP_FET_SOURCE','4':'UP_GATE','5':drain})
passive(upstream,'R1200','7m 1W 1%','UP_ISENSE','UP_VBUS','WSL25127L000FEA')
passive(upstream,'R1201','10k 1%','UP_VBUS','UP_OVP')
passive(upstream,'R1202','560 1%','UP_OVP','GND')
passive(upstream,'R1203','10k','V3V3','UP_FAULT_N')
passive(upstream,'R1204','100k','TCPP_EN','GND')
passive(upstream,'R1205','100k','UP_GATE','UP_FET_SOURCE')
passive(upstream,'C1200','100nF 50V','UP_CBIAS','GND')
passive(upstream,'C1201','330pF 50V','UP_CC1','GND')
passive(upstream,'C1202','330pF 50V','UP_CC2','GND')
passive(upstream,'C1203','2.2uF 50V','UP_VBUS','GND')
passive(upstream,'C1204','100nF 10V','V5','GND')
passive(upstream,'C1205','1uF 10V','V5','GND')
part(upstream,'U1201','HD3SS3212IRKSR',dict(zip(map(str,range(1,22)),[None,'UP_MUX_OEN','UP_TXP_AC','UP_TXN_AC','GND','V3V3','UP_RXP','UP_RXN','UP_MUX_SEL',None,'GND','UP_RX2N','UP_RX2P','UP_TX2N','UP_TX2P','UP_RX1N','UP_RX1P','UP_TX1N','UP_TX1P','GND','GND'])))
passive(upstream,'R1206','100k','V3V3','UP_MUX_OEN')
passive(upstream,'R1207','100k','UP_MUX_SEL','GND')
passive(upstream,'C1206','100nF','V3V3','GND')
passive(upstream,'R1208','100k 0.1%','UP_VBUS','UP_ADC')
passive(upstream,'R1209','10k 0.1%','UP_ADC','GND')
passive(upstream,'C1207','1nF','UP_ADC','GND')
for offset,nets in enumerate([['UP_TX1P','UP_TX1N','UP_RX1P','UP_RX1N'],['UP_TX2P','UP_TX2N','UP_RX2P','UP_RX2N'],['UP_DP','UP_DM',None,None]]):
    esd(upstream,f'D{1200+offset}',nets)


standard('STM32G071KBT6N','MCU_ST_STM32G0','STM32G071K_8-B_TxN')
mcu = sheet('PD_MCU','STM32G071 - USB-PD policy and power control','STM32G071KBT6N LQFP32: PA8=UCPD1_CC1, PB15=UCPD1_CC2, PB6/PB7=I2C1, PA13/PA14=SWD.\nSource-only power path. UCPD dead-battery pins grounded; disable dead-battery function in firmware.\nBoot from flash using option bytes; preserve NRST function. Firmware required; no PD stack is generated here.')
part(mcu,'U1300','STM32G071KBT6N',dict(zip(map(str,range(1,33)),[
    '3V3_PG',None,None,'V3V3','GND','MCU_NRST','UP_ADC','PD_IANA','PD_RAW_ADC','GPU1_ADC','GPU2_ADC','EPS_ADC','HUB_RESET_N','HUB_VBUS_DET','PD_EN','PD_NRST','MCU_CC2','MCU_CC1','GND','TCPP_EN','GND','P1_ID_N','P2_ID_N','SWDIO','SWCLK','UP_FAULT_N','PD_NFLT','UP_MUX_SEL','UP_MUX_OEN','I2C_SCL','I2C_SDA','CORE_PG'])))
define('SWD_5',[(str(index),name,'passive') for index,name in enumerate(['VTREF','SWDIO','SWCLK','NRST','GND'],1)])
part(mcu,'J1300','SWD_5',{'1':'V3V3','2':'SWDIO','3':'SWCLK','4':'MCU_NRST','5':'GND'},mpn='SWD 1x5 2.54mm',footprint='Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical')
define('Reset_Button',[('1','NRST','passive'),('2','GND','passive')],shape='R')
part(mcu,'SW1300','Reset_Button',{'1':'MCU_NRST','2':'GND'},mpn='RESET_SWITCH_SELECTION_REQUIRED')
passive(mcu,'R1300','10k','V3V3','MCU_NRST')
passive(mcu,'C1300','100nF','MCU_NRST','GND')
passive(mcu,'C1301','100nF','V3V3','GND')
passive(mcu,'C1302','4.7uF 6.3V','V3V3','GND')
passive(mcu,'R1301','4.7k','V3V3','I2C_SCL')
passive(mcu,'R1302','4.7k','V3V3','I2C_SDA')
passive(mcu,'R1303','100k 0.1%','PD_RAW','PD_RAW_ADC')
passive(mcu,'R1304','10k 0.1%','PD_RAW_ADC','GND')
passive(mcu,'C1303','1nF','PD_RAW_ADC','GND')

define('GPU_8PIN',[(str(index),'12V' if index<=3 else ('SENSE' if index in [4,8] else 'GND'),'passive') for index in range(1,9)])
define('EPS_8PIN',[(str(index),'GND' if index<=4 else '12V','passive') for index in range(1,9)])
define('Power_Selector',[('1','GPU_PAIR','passive'),('2','COMMON','passive'),('3','EPS','passive')])
inputs = sheet('Power_Input','12 V input - two GPU 8-pin OR one EPS 8-pin','Choose GPU pair OR EPS with a break-before-make selector rated >=30 A, >=24 VDC. Do not switch under load.\nGPU pair MUST come from the same PSU. Both GPU connectors required for full output. Do not combine separate PSUs.\nConnector pin numbering is the electrical interface: verify keying, mating face and selected manufacturer drawing before layout.')
for index in [1,2]:
    part(inputs,f'J140{index}','GPU_8PIN',{str(pin):(f'GPU{index}_12V' if pin<=3 else 'GND') for pin in range(1,9)},mpn='PCIe_GPU_8PIN_KEYED_HEADER_SELECTION_REQUIRED')
    passive(inputs,f'F140{index}','12A >=32VDC',f'GPU{index}_12V','GPU_PAIR',mpn='FUSE_AND_HOLDER_SELECTION_REQUIRED')
part(inputs,'J1403','EPS_8PIN',{str(pin):('GND' if pin<=4 else 'EPS_12V') for pin in range(1,9)},mpn='EPS_CPU_8PIN_KEYED_HEADER_SELECTION_REQUIRED')
passive(inputs,'F1403','25A >=32VDC','EPS_12V','EPS_FUSED',mpn='FUSE_AND_HOLDER_SELECTION_REQUIRED')
part(inputs,'SW1400','Power_Selector',{'1':'GPU_PAIR','2':'VIN12','3':'EPS_FUSED'},'SPDT 30A >=24VDC BBM',mpn='DC_POWER_SELECTOR_SELECTION_REQUIRED')
passive(inputs,'C1400','100uF 25V polymer','VIN12','GND',mpn='INPUT_BULK_CAP_SELECTION_REQUIRED')
passive(inputs,'C1401','100nF 25V','VIN12','GND')
for index,prefix in enumerate(['GPU1','GPU2','EPS']):
    passive(inputs,f'R{1400+2*index}','100k',prefix+'_12V',prefix+'_ADC')
    passive(inputs,f'R{1401+2*index}','10k',prefix+'_ADC','GND')
    passive(inputs,f'C{1402+index}','1nF',prefix+'_ADC','GND')

define('Supply_Flag',[('1','SUPPLY','power_out')])
for index,(page,net) in enumerate([(inputs,'VIN12'),(inputs,'GND'),(power5,'V5'),(power33,'V3V3'),(core,'VCORE')],1):
    part(page,f'#FLG{index:03}','Supply_Flag',{'1':net},value=net+' supply')


def set_pin_types(name, types, labels=None):
    labels = labels or {}
    LIBRARY[name]['pins'] = [(number,labels.get(number,label),types.get(number,electrical))
                            for number,label,electrical in LIBRARY[name]['pins']]


set_pin_types('USB7206CT_KDX', {str(number):'input' for number in [1,2,21,22,23,24,63,64,65,98,100]} | {'97':'output'})
set_pin_types('HD3SS3220IRNHR', {str(number):'input' for number in [3,4,5,12,22,29]} |
              {str(number):'open_collector' for number in [11,23,24,25,26,27]} | {'28':'power_in'}, {'28':'GND'})
set_pin_types('HD3SS3212IRKSR', {'1':'passive','2':'input','9':'input','10':'passive'})
set_pin_types('LM51772RHAR', {str(number):'input' for number in [3,4,6,7,9,14,15,16,18,19,20,21,22,37,38]} |
              {str(number):'output' for number in [13,24,27,30,32,35]} |
              {str(number):'passive' for number in [2,8,10,12,23,25,26,31,33,34,39]} |
              {'1':'power_out','29':'power_out','11':'open_collector'})

define('TVS_Bidirectional',[('1','LINE','passive'),('2','GND','passive')],
       'Diode_SMD:D_SMB', 'https://www.littelfuse.com/products/tvs-diodes/surface-mount/smbj')
part(upstream,'D1203','TVS_Bidirectional',{'1':'UP_VBUS','2':'GND'},'SMBJ22CA',mpn='SMBJ22CA')
part(inputs,'D1400','TVS_Bidirectional',{'1':'VIN12','2':'GND'},'SMBJ13CA',mpn='SMBJ13CA')
for port_index,base in [(1,600),(2,700),(3,800),(4,900),(5,1000),(6,1100)]:
    page = next(page for page in SHEETS if page['name'] == (f'USB_C{port_index}' if port_index<3 else f'USB_A{port_index-2}'))
    part(page,f'D{base+9}','TVS_Bidirectional',{'1':f'P{port_index}_VBUS','2':'GND'},'SMBJ5.0CA',mpn='SMBJ5.0CA')


def apply_bom_footprints():
    packages = {}
    verified = {
        'U501':'Package_DFN_QFN:VQFN-100-1EP_12x12mm_P0.4mm_EP8x8mm',
        'U600':'Package_DFN_QFN:Texas_RNH0030A_WQFN-30-1EP_2.5x4.5mm_P0.4mm_EP1.2x3.2mm',
        'U700':'Package_DFN_QFN:Texas_RNH0030A_WQFN-30-1EP_2.5x4.5mm_P0.4mm_EP1.2x3.2mm',
        'C403':'Capacitor_SMD:C_0805_2012Metric',
        'C404':'Capacitor_SMD:C_0805_2012Metric',
    }
    for path in (ROOT/'Reference'/'WEBENCH').glob('*.csv'):
        with path.open(encoding='utf-8-sig') as stream:
            for item in csv.DictReader(stream):
                description = item['Description']
                if 'Package: ' in description:
                    packages[item['Part Number']] = description.split('Package: ')[1].split()[0]
    sizes = {'0201':'0201_0603Metric','0402':'0402_1005Metric','0603':'0603_1608Metric','0805':'0805_2012Metric','1206':'1206_3216Metric','1210':'1210_3225Metric','1210_280':'1210_3225Metric'}
    for page in SHEETS:
        for component in page['parts']:
            package = packages.get(component['mpn'])
            if package in sizes and component['kind'] == 'C':
                component['footprint'] = 'Capacitor_SMD:C_'+sizes[package]
            if component['kind'] == 'R':
                for size in sizes:
                    if size in component['mpn']:
                        component['footprint'] = 'Resistor_SMD:R_'+sizes[size]
                        break
            if component['ref'] == 'R101':
                component['footprint'] = ''
            elif component['ref'] == 'R1200':
                component['footprint'] = 'Resistor_SMD:R_2512_6332Metric'
            elif component['ref'] in ['R611','R711']:
                component['footprint'] = 'Resistor_SMD:R_1206_3216Metric'
            elif component['ref'] in ['C111','C112','C113','C1400']:
                component['footprint'] = ''
            if component['ref'] in verified:
                component['footprint'] = verified[component['ref']]


def consolidate_sheets():
    original = {page['name']:page for page in SHEETS}
    groups = [
        ('Power','Power Supplies and Inputs', ['Power_Input','PD_BuckBoost','Power_5V','Power_3V3','Power_Core']),
        ('MCU','MCU and Upstream PD Control', ['PD_MCU','USB_Upstream']),
        ('USB_Hub','USB Hub and Downstream Ports', ['Hub','Hub_Decoupling','USB_C1','USB_C2','USB_A1','USB_A2','USB_A3','USB_A4']),
    ]
    SHEETS.clear()
    for name,title,blocks in groups:
        page = sheet(name,title,'Named nets connect the circuit blocks on this sheet and across the hierarchy.')
        page['sections'] = [original[block] for block in blocks]
        for block in blocks:
            for component in original[block]['parts']:
                component['block'] = block
                page['parts'].append(component)


if __name__ == '__main__':
    apply_bom_footprints()
    consolidate_sheets()
    generate()
"""Generate an unrouted placement draft using KiCad 9's bundled Python."""

from pathlib import Path
import argparse
import json
import subprocess
import tempfile
import uuid
import xml.etree.ElementTree as ET

import pcbnew


ROOT = Path(__file__).resolve().parent
PROJECT = 'XG_Mobile_USB_Hub'
KICAD = Path(r'C:\Program Files\KiCad\9.0')
NAMESPACE = uuid.UUID('eb5758b8-41e4-43f1-b10f-1330e4bc0380')
REGIONS = {
    'USB_C1': (51, 64, 18, 21), 'USB_C2': (71, 64, 18, 21),
    'USB_A1': (91, 64, 18, 21), 'USB_A2': (111, 64, 18, 21),
    'USB_A3': (131, 64, 18, 21), 'USB_A4': (151, 64, 18, 21),
    'PD_BuckBoost': (51, 87, 37, 27), 'Hub': (91, 87, 37, 27),
    'Hub_Decoupling': (91, 87, 37, 27),
    'Power_5V': (131, 87, 38, 27),
    'Power_3V3': (131, 115, 18, 14), 'Power_Core': (151, 115, 18, 14),
    'PD_MCU': (91, 115, 37, 14),
    'USB_Upstream': (51, 115, 37, 14),
    'Power_Input': (51, 87, 37, 27),
}


def point(xpos, ypos):
    return pcbnew.VECTOR2I(pcbnew.FromMM(xpos), pcbnew.FromMM(ypos))


def label(board, value, xpos, ypos, size=1, layer=pcbnew.Dwgs_User):
    item = pcbnew.PCB_TEXT(board)
    item.SetText(value)
    item.SetPosition(point(xpos, ypos))
    item.SetTextSize(point(size, size))
    item.SetTextThickness(pcbnew.FromMM(0.15))
    item.SetLayer(layer)
    board.Add(item)


def rectangle(board, xpos, ypos, width, height, layer):
    for start, end in [((xpos,ypos),(xpos+width,ypos)),
                       ((xpos+width,ypos),(xpos+width,ypos+height)),
                       ((xpos+width,ypos+height),(xpos,ypos+height)),
                       ((xpos,ypos+height),(xpos,ypos))]:
        line = pcbnew.PCB_SHAPE()
        line.SetShape(pcbnew.SHAPE_T_SEGMENT)
        line.SetStart(point(*start))
        line.SetEnd(point(*end))
        line.SetLayer(layer)
        line.SetWidth(pcbnew.FromMM(0.1))
        board.Add(line)


def generate():
    manifest = json.loads((ROOT/'connectivity.json').read_text())
    components = [component for component in manifest if component['physical']]
    with tempfile.TemporaryDirectory() as directory:
        netlist_path = Path(directory)/'netlist.xml'
        subprocess.run([str(KICAD/'bin/kicad-cli.exe'), 'sch', 'export', 'netlist',
                        '--format', 'kicadxml', '--output', str(netlist_path),
                        str(ROOT/(PROJECT+'.kicad_sch'))], check=True)
        exported = ET.parse(netlist_path)
    assignments = {(node.get('ref'),node.get('pin')):net.get('name').lstrip('/')
                   for net in exported.findall('.//nets/net') for node in net.findall('node')}
    board = pcbnew.BOARD()
    board.SetCopperLayerCount(4)
    board.GetDesignSettings().SetBoardThickness(pcbnew.FromMM(1.6))
    nets = {}
    for name in sorted({net for component in components for net in component['nets'].values() if net}):
        item = pcbnew.NETINFO_ITEM(board, name)
        board.Add(item)
        nets[name] = item
    rectangle(board, 50, 50, 120, 80, pcbnew.Edge_Cuts)
    label(board, '120 x 80 mm / UNROUTED PLACEMENT DRAFT / NOT FOR FABRICATION', 110, 44, 1.4)
    label(board, 'FRONT: DOWNSTREAM USB', 110, 48)
    label(board, 'REAR: UPSTREAM USB-C + POWER INPUTS', 110, 133)
    label(board, '4 copper layers are provisional; no impedance stack-up is specified.', 110, 137)
    for index, name in enumerate(['USB-C 1','USB-C 2','USB-A 1','USB-A 2','USB-A 3','USB-A 4']):
        xpos = 52+20*index
        rectangle(board, xpos, 51, 16, 11, pcbnew.Dwgs_User)
        label(board, name+'\nRESERVED', xpos+8, 56.5, 0.9)
    for xpos,width,name in [(52,14,'UP USB-C'),(69,19,'GPU 1'),(91,19,'GPU 2'),(113,19,'EPS')]:
        rectangle(board, xpos, 117, width, 12, pcbnew.Dwgs_User)
        label(board, name+'\nRESERVED', xpos+width/2, 123, 0.8)
    report = {'board_mm':[120,80], 'routed':False, 'missing':[], 'staged':[],
              'loaded':[], 'connected_pad_count':0, 'footprint_pad_mismatches':[]}
    cursors = {}
    staging_index = 0
    for component in components:
        ref = component['ref']
        footprint = None
        library_name = component['footprint']
        reason = 'No footprint assigned'
        if ':' in library_name:
            library, name = library_name.split(':',1)
            footprint = pcbnew.FootprintLoad(str(KICAD/'share/kicad/footprints'/(library+'.pretty')),name)
            if footprint:
                expected = set(component['nets'])
                actual = {pad.GetNumber() for pad in footprint.Pads() if pad.GetNumber()}
                if expected != actual:
                    reason = 'Pad numbers differ: schematic='+','.join(sorted(expected))+'; footprint='+','.join(sorted(actual))
                    report['footprint_pad_mismatches'].append({'ref':ref,'reason':reason})
                    footprint = None
            else:
                reason = 'Footprint library entry not found: '+library_name
        if footprint is None:
            footprint = pcbnew.FOOTPRINT(board)
            footprint.SetFPID(pcbnew.LIB_ID('Daughterboard_Unresolved',ref))
            footprint.SetAttributes(pcbnew.FP_EXCLUDE_FROM_POS_FILES | pcbnew.FP_EXCLUDE_FROM_BOM)
            report['missing'].append({'ref':ref,'mpn':component['mpn'],'reason':reason})
        else:
            footprint.SetFPID(pcbnew.LIB_ID(*library_name.split(':',1)))
            report['loaded'].append(ref)
            for pad in footprint.Pads():
                number = pad.GetNumber()
                net = component['nets'].get(number)
                if net:
                    assert assignments.get((ref,number)) == net, (ref,number,net)
                    pad.SetNet(nets[net])
                    report['connected_pad_count'] += 1
        footprint.SetReference(ref)
        footprint.SetValue(component['value'])
        path = pcbnew.KIID_PATH()
        for name in ['USB_C_Daughterboard','sheet/'+component['sheet'],ref]:
            path.push_back(pcbnew.KIID(str(uuid.uuid5(NAMESPACE,name))))
        footprint.SetPath(path)
        footprint.SetSheetname(component['sheet'])
        footprint.SetSheetfile(component['sheet']+'.kicad_sch')
        footprint.Reference().SetTextSize(point(0.8,0.8))
        footprint.Reference().SetTextThickness(pcbnew.FromMM(0.12))
        footprint.Reference().SetLayer(pcbnew.F_Fab)
        footprint.Value().SetVisible(False)
        board.Add(footprint)
        if ref == 'J1300':
            footprint.SetOrientationDegrees(90)
        missing = not list(footprint.Pads())
        block = component.get('block',component['sheet'])
        region = REGIONS[block]
        xpos,ypos,width,height = region
        cursor = cursors.setdefault(block,[0,0,0])
        box = footprint.GetBoundingBox(False,False)
        item_width = max(2.5,pcbnew.ToMM(box.GetWidth()))+1
        item_height = max(2,pcbnew.ToMM(box.GetHeight()))+1.4
        if cursor[0]+item_width > width:
            cursor[0] = 0
            cursor[1] += cursor[2]
            cursor[2] = 0
        bottom_overflow = not missing and cursor[1]+item_height > height
        if bottom_overflow:
            xpos,ypos,width,height = 51,64,118,21
            cursor = cursors.setdefault('__bottom_overflow',[0,0,0])
            if cursor[0]+item_width > width:
                cursor[0] = 0
                cursor[1] += cursor[2]
                cursor[2] = 0
        if missing or cursor[1]+item_height > height:
            stage_x = 185+(staging_index%6)*22
            stage_y = 54+(staging_index//6)*12
            footprint.SetPosition(point(stage_x,stage_y))
            rectangle(board,stage_x-9,stage_y-4,18,9,pcbnew.Dwgs_User)
            label(board,ref+(' / NO PACKAGE' if missing else ' / TO PLACE'),stage_x,stage_y+4,0.7)
            staging_index += 1
            report['staged'].append(ref)
        else:
            center = box.GetCenter()
            footprint.SetPosition(point(xpos+cursor[0]+item_width/2-pcbnew.ToMM(center.x),
                                        ypos+cursor[1]+item_height/2-pcbnew.ToMM(center.y)))
            if bottom_overflow or block in ['Hub_Decoupling','Power_Input','USB_Upstream','PD_MCU']:
                footprint.Flip(footprint.GetPosition(),False)
            actual_center = footprint.GetBoundingBox(False,False).GetCenter()
            target_center = point(xpos+cursor[0]+item_width/2,ypos+cursor[1]+item_height/2)
            footprint.Move(target_center-actual_center)
            cursor[0] += item_width
            cursor[2] = max(cursor[2],item_height)
    label(board,'STAGING / UNRESOLVED PACKAGES - NOT PART OF THE BOARD OUTLINE',240,46,1.2)
    pcbnew.SaveBoard(str(ROOT/(PROJECT+'.kicad_pcb')),board)
    (ROOT/'PCB_Placement_Report.json').write_text(json.dumps(report,indent=2)+'\n')
    saved = pcbnew.LoadBoard(str(ROOT/(PROJECT+'.kicad_pcb')))
    assert len(list(saved.GetFootprints())) == len(components)
    assert len(list(saved.GetTracks())) == 0
    lookup = {component['ref']:component for component in components}
    for footprint in saved.GetFootprints():
        if footprint.GetReference() in report['loaded']:
            bounds = footprint.GetBoundingBox(False,False)
            assert pcbnew.ToMM(bounds.GetLeft()) >= 50
            assert pcbnew.ToMM(bounds.GetRight()) <= 170
            assert pcbnew.ToMM(bounds.GetTop()) >= 50
            assert pcbnew.ToMM(bounds.GetBottom()) <= 130
        for pad in footprint.Pads():
            if pad.GetNumber():
                assert pad.GetNetname() == (lookup[footprint.GetReference()]['nets'][pad.GetNumber()] or '')
    print(f"PASS: {len(components)} schematic-linked items; {len(report['loaded'])} loaded footprints; "
          f"{len(report['missing'])} unresolved; {report['connected_pad_count']} connected pads; no tracks.")


def relink_sheets():
    board_path = ROOT/(PROJECT+'.kicad_pcb')
    board = pcbnew.LoadBoard(str(board_path))
    components = {item['ref']:item for item in json.loads((ROOT/'connectivity.json').read_text()) if item['physical']}

    def geometry_snapshot(source):
        return {footprint.GetReference():(footprint.m_Uuid.AsString(),
                footprint.GetPosition().x,footprint.GetPosition().y,
                footprint.GetOrientationDegrees(),footprint.GetLayer(),
                sorted((pad.GetNumber(),pad.GetPosition().x,pad.GetPosition().y,pad.GetNetname())
                       for pad in footprint.Pads())) for footprint in source.GetFootprints()}

    before = geometry_snapshot(board)
    assert set(before) == set(components)
    track_count = len(list(board.GetTracks()))
    for footprint in board.GetFootprints():
        component = components[footprint.GetReference()]
        path = pcbnew.KIID_PATH()
        for name in ['USB_C_Daughterboard','sheet/'+component['sheet'],component['ref']]:
            path.push_back(pcbnew.KIID(str(uuid.uuid5(NAMESPACE,name))))
        footprint.SetPath(path)
        footprint.SetSheetname(component['sheet'])
        footprint.SetSheetfile(component['sheet']+'.kicad_sch')
    pcbnew.SaveBoard(str(board_path),board)
    saved = pcbnew.LoadBoard(str(board_path))
    assert geometry_snapshot(saved) == before
    assert len(list(saved.GetTracks())) == track_count
    for footprint in saved.GetFootprints():
        component = components[footprint.GetReference()]
        assert footprint.GetSheetfile() == component['sheet']+'.kicad_sch'
        assert footprint.GetPath().AsString() == '/'.join(['']+[str(uuid.uuid5(NAMESPACE,name))
            for name in ['USB_C_Daughterboard','sheet/'+component['sheet'],component['ref']]])
    print(f'PASS: {len(before)} PCB links updated; footprint UUIDs, placement, pad nets and track count preserved')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--relink-sheets',action='store_true',help='Update hierarchy paths without regenerating placement')
    args = parser.parse_args()
    if args.relink_sheets:
        relink_sheets()
    else:
        generate()
"""
Trace keep-out / rule area  --  runs INSIDE KiCad (pcbnew Python API, KiCad 9).

Builds a keep-out (rule area) around the SELECTED track(s) that forbids copper zone fill, so any
GND / other pour is pushed CLEARANCE away from the trace copper. The area is the union of every
selected track (segments, arcs, and their vias) inflated by CLEARANCE -- so selecting BOTH nets of a
differential pair wraps the pair as one unit, keeping the clearance on each OUTER side and leaving no
pour between the two traces. One keep-out is created per copper layer the selection occupies.

USAGE (PCB editor > Tools > Scripting Console):
    - Select the track segments (one net, or both nets of a diff pair), then:
        exec(open(r'd:/Repos/XG_Mobile_Station/plugins/trace_keepout.py').read())
    - CLEARANCE (mm) is the distance to keep other pours off the trace copper edge (default 0.3).
    - Override any knob WITHOUT editing this file: set a PARAMS dict first, e.g.
        PARAMS = dict(CLEARANCE=0.3, APPLY=False); exec(open(r'.../trace_keepout.py').read())
      (headless: run(board, apply=False, CLEARANCE=0.3)).
    - APPLY = False = dry-run (report only). Set FILL = True to re-fill zones so pours retreat now;
      otherwise press B in the editor after applying.
    - Undo: Edit > Undo, or select the added "trace_keepout_*" zones and Delete, or git checkout.

Can also be dropped in KiCad's scripting/plugins folder (Tools > External Plugins).
"""
try:
    import pcbnew
except ImportError:
    raise SystemExit("Run this from KiCad's PCB editor Scripting Console (pcbnew not importable).")

# ----------------------------------------------------------------------------- parameters (mm)
APPLY            = True
CLEARANCE        = 0.30       # distance to keep other pours away from the trace copper edge
NAME             = "trace_keepout"   # name prefix for the created rule-area zones
BLOCK_POUR       = True       # forbid copper zone fill inside the area (the point of this tool)
BLOCK_VIAS       = False      # also forbid vias inside the area
BLOCK_TRACKS     = False      # also forbid tracks inside the area
BLOCK_PADS       = False      # also forbid pads inside the area
BLOCK_FOOTPRINTS = False      # also forbid footprints inside the area
INCLUDE_VIAS     = True       # include selected vias in the keep-out (on the trace layers they touch)
FILL             = False      # re-fill all zones after applying (so pours retreat immediately)
ARC_ERROR        = 0.005      # max chord error when flattening arcs / rounded ends

# ----------------------------------------------------------------------------- overrides + refresh
def _apply_overrides(overrides):
    """Override any UPPERCASE knob at call time: run(..., CLEARANCE=0.2) or PARAMS=dict(CLEARANCE=0.2)."""
    if not overrides:
        return
    g = globals(); bad = []
    for k, v in overrides.items():
        if k.isupper() and k in g:
            g[k] = v
        else:
            bad.append(k)
    if bad:
        valid = ", ".join(sorted(n for n in g if n.isupper() and not n.startswith("_")))
        print("[params] ignored: %s\n[params] valid knobs: %s" % (", ".join(sorted(bad)), valid))

def _refresh(board):
    """Rebuild connectivity, then redraw. Both guarded so it's a no-op when run headless."""
    try:
        board.BuildConnectivity()
    except Exception:
        pass
    try:
        pcbnew.Refresh()
    except Exception:
        pass

# ----------------------------------------------------------------------------- main
def run(board=None, apply=None, **overrides):
    _apply_overrides(overrides)
    if board is None:
        board = pcbnew.GetBoard()
    if apply is None:
        apply = APPLY

    clr = pcbnew.FromMM(max(0.0, CLEARANCE))
    err = pcbnew.FromMM(max(0.001, ARC_ERROR))

    segs = [t for t in board.GetTracks() if t.IsSelected() and not isinstance(t, pcbnew.PCB_VIA)]
    vias = [t for t in board.GetTracks() if t.IsSelected() and isinstance(t, pcbnew.PCB_VIA)]
    if not segs and not vias:
        print("Nothing selected. Select the track segment(s) to wrap (one net or a diff pair) and re-run.")
        return

    # union the inflated shape of every selected track, grouped by the copper layer it sits on
    per_layer = {}   # layer_id -> SHAPE_POLY_SET (union)
    nets = set()
    for t in segs:
        layer = t.GetLayer()
        u = per_layer.get(layer)
        if u is None:
            u = per_layer[layer] = pcbnew.SHAPE_POLY_SET()
        tp = pcbnew.SHAPE_POLY_SET()
        t.TransformShapeToPolygon(tp, layer, clr, err, pcbnew.ERROR_OUTSIDE)
        u.BooleanAdd(tp)
        nets.add(t.GetNetname() or "<none>")
    if INCLUDE_VIAS:
        for v in vias:
            for layer in list(per_layer.keys()):     # only the layers the traces already occupy
                if v.IsOnLayer(layer):
                    tp = pcbnew.SHAPE_POLY_SET()
                    v.TransformShapeToPolygon(tp, layer, clr, err, pcbnew.ERROR_OUTSIDE)
                    per_layer[layer].BooleanAdd(tp)
            nets.add(v.GetNetname() or "<none>")

    kind = "diff pair" if len(nets) == 2 else ("%d nets" % len(nets) if len(nets) != 1 else "single net")
    layer_names = ", ".join(board.GetLayerName(l) for l in per_layer)
    print("keep-out: %s (%s), %d seg + %d via, CLEARANCE=%.3f mm, layers: %s"
          % (kind, "/".join(sorted(nets)), len(segs), len(vias), CLEARANCE, layer_names))

    if not apply:
        for layer, u in per_layer.items():
            print("  %-6s islands=%d" % (board.GetLayerName(layer), u.OutlineCount()))
        print("DRY RUN: set APPLY = True (or run(apply=True)) to add the keep-out zones.")
        return

    made = 0
    for layer, u in per_layer.items():
        u.Simplify()
        if u.OutlineCount() == 0:
            continue
        zone = pcbnew.ZONE(board)
        zone.SetLayer(layer)
        zone.SetIsRuleArea(True)
        zone.SetDoNotAllowCopperPour(BLOCK_POUR)
        zone.SetDoNotAllowVias(BLOCK_VIAS)
        zone.SetDoNotAllowTracks(BLOCK_TRACKS)
        zone.SetDoNotAllowPads(BLOCK_PADS)
        zone.SetDoNotAllowFootprints(BLOCK_FOOTPRINTS)
        try:
            zone.SetZoneName("%s_%s" % (NAME, board.GetLayerName(layer)))
        except Exception:
            pass
        zpoly = zone.Outline()
        zpoly.RemoveAllContours()
        zpoly.BooleanAdd(u)
        try:
            zone.HatchBorder()
        except Exception:
            pass
        board.Add(zone)
        made += 1

    if FILL:
        try:
            filler = pcbnew.ZONE_FILLER(board)
            filler.Fill(board.Zones())
        except Exception as e:
            print("zone re-fill skipped (%s); press B in the editor to re-fill." % e)

    _refresh(board)
    print("APPLIED: %d keep-out zone(s) added -- review, re-fill zones (B), run DRC, then save (Ctrl+S)." % made)
    return made


try:
    class TraceKeepoutPlugin(pcbnew.ActionPlugin):
        def defaults(self):
            self.name = "Trace keep-out (rule area)"
            self.category = "Modify PCB"
            self.description = "Keep-out around selected trace(s) that forbids other pours (CLEARANCE away)."
            self.show_toolbar_button = True

        def Run(self):
            run(pcbnew.GetBoard(), apply=True)
except Exception:
    pass

if __name__ == "__main__":
    run(**globals().get("PARAMS", {}))
else:
    try:
        TraceKeepoutPlugin().register()
    except Exception:
        pass

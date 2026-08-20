"""
GND via-fence generator  --  runs INSIDE KiCad (pcbnew Python API, KiCad 9).

Places GND through-vias along BOTH sides of every high-speed diff-pair trace: an even fence at the
preferred pitch that hugs clearance obstacles (crossings / fan-ins) and fills gaps, while staying a
hard minimum away from every HS trace (any layer), existing via, and non-GND pad.

PARAMETERS are the constants below (via size/drill, clearance, pitch, floors, which nets...).

USAGE (PCB editor > Tools > Scripting Console):
    exec(open(r'd:/Repos/XG_Mobile_Station/plugins/via_fence.py').read())
  - Set APPLY = False for a dry-run (prints stats, changes nothing).
  - Override any knob WITHOUT editing this file: set a PARAMS dict first, e.g.
        PARAMS = dict(VIA_SIZE=0.25, PREF=0.7, GAP_MIN=0.2, APPLY=False)
        exec(open(r'.../via_fence.py').read())
      (headless: run(board, apply=False, VIA_SIZE=0.25)).
  - If USE_SELECTION and you have track SEGMENTS selected, ONLY those segments are fenced and a fence
    via is dropped only where it collides with ANOTHER SELECTED segment (select a diff pair -> the
    outboard sides get fenced, the inboard vias drop out; select one segment -> both sides fenced).
    Otherwise every net whose name matches HS_NET_REGEX is fenced. AVOID_VIAS / AVOID_PADS also keep
    fence vias off existing vias / non-GND pads.
  - Undo via git checkout of the board file.

Can also be dropped in the KiCad scripting/plugins folder (Tools > External Plugins).
"""
import math
import re
import collections

try:
    import pcbnew
except ImportError:
    raise SystemExit("Run this from KiCad's PCB editor Scripting Console (pcbnew not importable).")

# ----------------------------------------------------------------------------- parameters (mm)
APPLY         = True
USE_SELECTION = True          # if track SEGMENTS are selected: fence ONLY those segments and check
                              # trace collisions ONLY against the other selected segments; else HS_NET_REGEX
AVOID_VIAS    = True          # also keep fence vias off existing vias (avoid overlaps/stacking)
AVOID_PADS    = True          # also keep fence vias off non-GND pads
HS_NET_REGEX  = r"^/?HS[IO]"  # nets to fence when nothing is selected (regex on net name)
GND_NET_NAME  = "GND"
VIA_SIZE      = 0.30          # fence via diameter
VIA_DRILL     = 0.20          # fence via drill
GAP_MIN       = 0.21          # min via-edge -> HS-trace-edge (any layer); also to non-GND vias
PREF          = 0.80          # preferred pitch along the fence (band ~0.7-0.9)
TIGHT_MIN     = 0.50          # hard via-to-via floor (pack to this before leaving a > GAP_CAP hole)
GAP_CAP       = 1.00          # fill so no fence gap runs wider than this where a via legally fits
HOLE_MIN      = 0.30          # min hole-to-hole copper to an existing via
PAD_CLEAR     = 0.20          # min via-edge -> non-GND pad edge
DS            = 0.10          # flank sampling step
CELL          = 1.0
EXPAND        = 2.0
VIA_R         = VIA_SIZE / 2.0

# ----------------------------------------------------------------------------- geometry helpers
def dist_seg(p, a, b):
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx - ax, by - ay; L2 = dx * dx + dy * dy
    if L2 < 1e-15:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))

def circle_from_3(a, m, b):
    ax, ay = a; bx, by = m; cx, cy = b
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-12:
        return None
    a2, b2, c2 = ax * ax + ay * ay, bx * bx + by * by, cx * cx + cy * cy
    ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d
    uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d
    return (ux, uy), math.hypot(ax - ux, ay - uy)

def build_arc(a, mid, b):
    cr = circle_from_3(a, mid, b)
    if cr is None:
        return None
    c, r = cr
    s = math.atan2(a[1] - c[1], a[0] - c[0])
    mm_ = math.atan2(mid[1] - c[1], mid[0] - c[0])
    e = math.atan2(b[1] - c[1], b[0] - c[0])
    ccwsm = (mm_ - s) % (2 * math.pi); ccwse = (e - s) % (2 * math.pi)
    ccw, extent = (True, ccwse) if ccwsm <= ccwse else (False, 2 * math.pi - ccwse)
    return dict(c=c, r=r, s=s, e=e, ccw=ccw, extent=extent, a=a, b=b)

def dist_arc(p, arc):
    c = arc["c"]; ang = math.atan2(p[1] - c[1], p[0] - c[0])
    d = (ang - arc["s"]) % (2 * math.pi) if arc["ccw"] else (arc["s"] - ang) % (2 * math.pi)
    if d <= arc["extent"] + 1e-9:
        return abs(math.hypot(p[0] - c[0], p[1] - c[1]) - arc["r"])
    return min(math.hypot(p[0] - arc["a"][0], p[1] - arc["a"][1]),
               math.hypot(p[0] - arc["b"][0], p[1] - arc["b"][1]))

def P2(vec):
    return (pcbnew.ToMM(vec.x), pcbnew.ToMM(vec.y))

def cell(p):
    return (int(p[0] // CELL), int(p[1] // CELL))

# ----------------------------------------------------------------------------- fence engine
class Fence:
    def __init__(self, board):
        self.board = board
        gnet = board.FindNet(GND_NET_NAME)
        if gnet is None:
            raise SystemExit("GND net %r not found on the board." % GND_NET_NAME)
        self.gnd = gnet.GetNetCode()
        hs_re = re.compile(HS_NET_REGEX)
        # segment-level selection: fence (and collide against) ONLY the selected track segments
        self.selected_scope = USE_SELECTION and any(
            t.IsSelected() and not isinstance(t, pcbnew.PCB_VIA) for t in board.GetTracks())

        # HS traces (only the selected segments when a selection is in scope)
        self.traces = []
        self.evias = []
        for t in board.GetTracks():
            if isinstance(t, pcbnew.PCB_VIA):
                self.evias.append((P2(t.GetPosition()), pcbnew.ToMM(t.GetWidth(t.TopLayer())) / 2.0, t.GetNetCode()))
                continue
            if self.selected_scope:
                if not t.IsSelected():
                    continue
            elif not hs_re.match(t.GetNetname() or ""):
                continue
            net = t.GetNetCode()
            hw = pcbnew.ToMM(t.GetWidth()) / 2.0
            if isinstance(t, pcbnew.PCB_ARC):
                a, mid, b = P2(t.GetStart()), P2(t.GetMid()), P2(t.GetEnd())
                arc = build_arc(a, mid, b)
                if arc is None:
                    tr = dict(kind="seg", a=a, b=b, hw=hw, net=net)
                    self._bb(tr, [a, b])
                else:
                    tr = dict(kind="arc", arc=arc, hw=hw, net=net)
                    self._bb(tr, [a, mid, b])
            else:
                a, b = P2(t.GetStart()), P2(t.GetEnd())
                tr = dict(kind="seg", a=a, b=b, hw=hw, net=net)
                self._bb(tr, [a, b])
            self.traces.append(tr)

        # non-GND pads (rotated rects)
        self.pads = []
        for fp in board.GetFootprints():
            for pad in fp.Pads():
                if pad.GetNetCode() == self.gnd:
                    continue
                pos = P2(pad.GetPosition()); sz = pad.GetSize()
                self.pads.append(dict(cx=pos[0], cy=pos[1], sx=pcbnew.ToMM(sz.x), sy=pcbnew.ToMM(sz.y),
                                      ang=math.radians(pad.GetOrientationDegrees())))

        self._hash()

    def _bb(self, t, pts):
        xs = [q[0] for q in pts]; ys = [q[1] for q in pts]
        t["bb"] = (min(xs), min(ys), max(xs), max(ys))

    def _hash(self):
        self.grid = collections.defaultdict(list)
        for i, t in enumerate(self.traces):
            x0, y0, x1, y1 = t["bb"]
            for cx in range(int((x0 - EXPAND) // CELL), int((x1 + EXPAND) // CELL) + 1):
                for cy in range(int((y0 - EXPAND) // CELL), int((y1 + EXPAND) // CELL) + 1):
                    self.grid[(cx, cy)].append(i)
        self.evgrid = collections.defaultdict(list)
        for (c, r, net) in self.evias:
            self.evgrid[cell(c)].append((c, r, net))
        self.padgrid = collections.defaultdict(list)
        for i, pd in enumerate(self.pads):
            rb = 0.5 * math.hypot(pd["sx"], pd["sy"]) + VIA_R + PAD_CLEAR
            for gx in range(int((pd["cx"] - rb) // CELL), int((pd["cx"] + rb) // CELL) + 1):
                for gy in range(int((pd["cy"] - rb) // CELL), int((pd["cy"] + rb) // CELL) + 1):
                    self.padgrid[(gx, gy)].append(i)

    def trace_gap(self, p, t):
        d = dist_seg(p, t["a"], t["b"]) if t["kind"] == "seg" else dist_arc(p, t["arc"])
        return d - t["hw"] - VIA_R

    def clears_all(self, p):
        mn = 1e9
        for i in self.grid.get(cell(p), ()):
            g = self.trace_gap(p, self.traces[i])
            if g < mn:
                mn = g
            if g < GAP_MIN - 1e-9:
                return False, g
        return True, mn

    def clears_vias(self, p):
        cx, cy = cell(p)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for (c, r, net) in self.evgrid.get((cx + dx, cy + dy), ()):
                    copper = VIA_R + r + (0.05 if net == self.gnd else GAP_MIN)
                    hole = VIA_DRILL / 2.0 + (r - 0.05) + HOLE_MIN
                    if math.hypot(p[0] - c[0], p[1] - c[1]) < max(copper, hole) - 1e-9:
                        return False
        return True

    def pad_gap(self, p):
        mn = 1e9
        for i in self.padgrid.get(cell(p), ()):
            pd = self.pads[i]
            dx, dy = p[0] - pd["cx"], p[1] - pd["cy"]; ca, sa = math.cos(pd["ang"]), math.sin(pd["ang"])
            u = dx * ca - dy * sa; v = dx * sa + dy * ca
            d = math.hypot(max(abs(u) - pd["sx"] / 2, 0.0), max(abs(v) - pd["sy"] / 2, 0.0)) - VIA_R
            if d < mn:
                mn = d
        return mn

    def valid(self, p):
        ok, _ = self.clears_all(p)
        if not ok:
            return False
        if AVOID_VIAS and not self.clears_vias(p):
            return False
        if AVOID_PADS and self.pad_gap(p) < PAD_CLEAR - 1e-9:
            return False
        return True

    # ---- chain each net's pieces into one continuous centre-line, offset to both flanks ----
    @staticmethod
    def _endpoints(t):
        return (t["a"], t["b"]) if t["kind"] == "seg" else (t["arc"]["a"], t["arc"]["b"])

    @staticmethod
    def _piece_points(t, forward):
        if t["kind"] == "seg":
            a, b = (t["a"], t["b"]) if forward else (t["b"], t["a"])
            L = math.hypot(b[0] - a[0], b[1] - a[1]); n = max(1, int(math.ceil(L / DS)))
            pts = [(a[0] + (k / n) * (b[0] - a[0]), a[1] + (k / n) * (b[1] - a[1])) for k in range(n + 1)]
        else:
            arc = t["arc"]; c = arc["c"]; r = arc["r"]
            L = r * arc["extent"]; n = max(1, int(math.ceil(L / DS)))
            pts = []
            for k in range(n + 1):
                f = k / n; ang = arc["s"] + (f * arc["extent"] if arc["ccw"] else -f * arc["extent"])
                pts.append((c[0] + r * math.cos(ang), c[1] + r * math.sin(ang)))
            if not forward:
                pts.reverse()
        return pts

    def _chains(self, pieces):
        Q = lambda pt: (round(pt[0], 4), round(pt[1], 4))
        adj = collections.defaultdict(list)
        for i, t in enumerate(pieces):
            e0, e1 = self._endpoints(t)
            adj[Q(e0)].append((i, 0)); adj[Q(e1)].append((i, 1))
        used = [False] * len(pieces); chains = []
        def walk(key):
            ch = []
            while True:
                nxt = next(((pi, ei) for (pi, ei) in adj[key] if not used[pi]), None)
                if nxt is None:
                    break
                pi, ei = nxt; used[pi] = True; ch.append((pi, ei == 0))
                e0, e1 = self._endpoints(pieces[pi]); key = Q(e1 if ei == 0 else e0)
            return ch
        for key in [k for k, v in adj.items() if len(v) == 1]:
            if any(not used[pi] for pi, _ in adj[key]):
                c = walk(key)
                if c:
                    chains.append(c)
        for i in range(len(pieces)):
            if not used[i]:
                c = walk(Q(self._endpoints(pieces[i])[0]))
                if c:
                    chains.append(c)
        return chains

    def _centreline(self, pieces, chain):
        pts = []
        for pi, fwd in chain:
            pp = self._piece_points(pieces[pi], fwd); hw = pieces[pi]["hw"]
            for j, p in enumerate(pp):
                if pts and j == 0:
                    continue
                pts.append((p, hw))
        return pts

    @staticmethod
    def _offset_flank(cl, side):
        out = []; cum = 0.0; prev = None; n = len(cl)
        for i in range(n):
            p, hw = cl[i]
            pa = cl[i - 1][0] if i > 0 else cl[i][0]
            pb = cl[i + 1][0] if i < n - 1 else cl[i][0]
            tx, ty = pb[0] - pa[0], pb[1] - pa[1]; L = math.hypot(tx, ty)
            nx, ny = (-ty / L, tx / L) if L > 1e-12 else (0.0, 0.0)
            d = hw + GAP_MIN + VIA_R
            fp = (p[0] + side * nx * d, p[1] + side * ny * d)
            if prev is not None:
                cum += math.hypot(fp[0] - prev[0], fp[1] - prev[1])
            out.append((cum, fp)); prev = fp
        return out

    def generate(self):
        by_net = collections.defaultdict(list)
        for t in self.traces:
            by_net[t["net"]].append(t)
        flanks = []
        for pieces in by_net.values():
            for chain in self._chains(pieces):
                cl = self._centreline(pieces, chain)
                if len(cl) < 2:
                    continue
                for side in (+1, -1):
                    samps = [(s, p, self.valid(p)) for (s, p) in self._offset_flank(cl, side)]
                    if samps:
                        flanks.append(samps)

        placed = []
        pgrid = collections.defaultdict(list)

        def far(p, floor):
            cx, cy = cell(p)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for q in pgrid.get((cx + dx, cy + dy), ()):
                        if math.hypot(p[0] - q[0], p[1] - q[1]) < floor - 1e-9:
                            return False
            return True

        def commit(p):
            placed.append(p); pgrid[cell(p)].append(p)

        # pass 1 - even fence at preferred pitch along each continuous flank
        for samps in flanks:
            last_s = None
            for s, p, v in samps:
                if not v:
                    continue
                if last_s is not None and s - last_s < PREF - 1e-9:
                    continue
                if far(p, TIGHT_MIN):
                    commit(p); last_s = s

        # pass 2 - hug obstacles: last valid before / first valid after each clearance gap
        for samps in flanks:
            prev_p = None; prev_v = None
            for s, p, v in samps:
                if prev_v is not None:
                    if v and not prev_v and far(p, TIGHT_MIN):
                        commit(p)
                    elif (not v) and prev_v and prev_p is not None and far(prev_p, TIGHT_MIN):
                        commit(prev_p)
                if v:
                    prev_p = p
                prev_v = v

        def covered(p, d):
            cx, cy = cell(p); span = int(d // CELL) + 1
            for dx in range(-span, span + 1):
                for dy in range(-span, span + 1):
                    for q in pgrid.get((cx + dx, cy + dy), ()):
                        if math.hypot(p[0] - q[0], p[1] - q[1]) <= d - 1e-9:
                            return True
                    if AVOID_VIAS:
                        for (c, r, net) in self.evgrid.get((cx + dx, cy + dy), ()):
                            if net == self.gnd and math.hypot(p[0] - c[0], p[1] - c[1]) <= d - 1e-9:
                                return True
            return False

        # pass 3 - fill: no fence gap wider than GAP_CAP where a via legally fits
        for samps in flanks:
            for s, p, v in samps:
                if not v or covered(p, GAP_CAP):
                    continue
                if far(p, TIGHT_MIN):
                    commit(p)

        return placed


def _apply_overrides(overrides):
    """Override any UPPERCASE knob at call time: run(..., PREF=0.7) or PARAMS=dict(PREF=0.7)."""
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

def run(board=None, apply=None, **overrides):
    _apply_overrides(overrides)
    global VIA_R
    VIA_R = VIA_SIZE / 2.0          # keep derived value in sync if VIA_SIZE was overridden
    if board is None:
        board = pcbnew.GetBoard()
    if apply is None:
        apply = APPLY
    f = Fence(board)
    scope = ("SELECTION (%d segments)" % len(f.traces)) if f.selected_scope else HS_NET_REGEX
    if not f.traces:
        print("No HS traces matched (scope=%s). Nothing to fence." % scope)
        return []
    placed = f.generate()
    mn = 1e9
    for p in placed:
        _, g = f.clears_all(p)
        mn = min(mn, g)
    pminlist = [f.pad_gap(p) for p in placed]
    pmin = min([x for x in pminlist if x < 1e8], default=None)
    print("scope=%s  HS traces=%d  existing vias=%d  non-GND pads=%d"
          % (scope, len(f.traces), len(f.evias), len(f.pads)))
    print("fence vias placed = %d   min via->trace clearance = %.4f mm (target >= %.3f)"
          % (len(placed), (mn if placed else 0), GAP_MIN))
    if pmin is not None:
        print("min via->non-GND-pad clearance = %.4f mm (target >= %.3f)" % (pmin, PAD_CLEAR))

    if apply and placed:
        for (x, y) in placed:
            v = pcbnew.PCB_VIA(board)
            v.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y)))
            v.SetWidth(pcbnew.FromMM(VIA_SIZE))
            v.SetDrill(pcbnew.FromMM(VIA_DRILL))
            v.SetViaType(pcbnew.VIATYPE_THROUGH)
            v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
            v.SetNetCode(f.gnd)
            board.Add(v)
        try:
            pcbnew.Refresh()
        except Exception:
            pass
        print("APPLIED: inserted %d GND fence vias -- review, then save (Ctrl+S)." % len(placed))
    else:
        print("DRY RUN: set APPLY = True (or call run(apply=True)) to insert the vias.")
    return placed


try:
    class ViaFencePlugin(pcbnew.ActionPlugin):
        def defaults(self):
            self.name = "GND via-fence generator"
            self.category = "Modify PCB"
            self.description = "Fence high-speed diff pairs with GND stitching vias."
            self.show_toolbar_button = True

        def Run(self):
            run(pcbnew.GetBoard(), apply=True)
except Exception:
    pass

if __name__ == "__main__":
    run(**globals().get("PARAMS", {}))
else:
    try:
        ViaFencePlugin().register()
    except Exception:
        pass

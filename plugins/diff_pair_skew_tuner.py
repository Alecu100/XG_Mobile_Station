"""
Differential-pair skew tuner  --  runs INSIDE KiCad (pcbnew Python API, KiCad 9).

Length-matches PCIe diff pairs (nulls internal / intra-pair skew) by adding small trapezoidal
"meander" bumps on the SHORTER trace, staying clear of pads/vias/other copper.

TWO MODES (auto-detected; force via MODE below):
  * SELECTION -- select some track segments of a diff pair, run the script, and it measures the
    pair's intra-pair skew and adds meanders ON THE SELECTED SEGMENTS. If they don't have room for
    the full correction it adds as much as fits and reports the residual. Select the SHORTER net's
    segments (the one that needs lengthening); the partner net is found by name (P/N, +/-, _P/_N).
  * AUTO -- no selection: tunes every TARGET_GROUPS pair, placing meanders on the straights nearest
    RETIMER_REF.

All constraints/distances are the constants below (clearances, heights, spacing, corner margin...).

USAGE (in the PCB editor):
    Tools > Scripting Console, then:
        exec(open(r'd:/Repos/XG_Mobile_Station/plugins/diff_pair_skew_tuner.py').read())
    Set APPLY = False for a dry-run (prints the plan, changes nothing).
    Override any knob WITHOUT editing this file: set a PARAMS dict first, e.g.
        PARAMS = dict(THICKEN_FACTOR=2.2, THICKEN_STEPS=6, VIA_CLEAR=0.25, PAD_CLEAR=0.6, APPLY=False)
        exec(open(r'.../diff_pair_skew_tuner.py').read())
      (headless: run(board, apply=False, THICKEN_FACTOR=2.2)).
    UNDO the last apply (e.g. to tune a different segment):  PARAMS = dict(ACTION="revert")  then re-exec.
    LIST past applies:  PARAMS = dict(ACTION="history").  Stored in <board>.skewtune_history.json.
    Also undoable via git checkout of the board file (console edits aren't always on Ctrl+Z).

Can also be dropped in the KiCad scripting/plugins folder to appear under Tools > External Plugins.
"""
import math
import collections
import json
import os

try:
    import pcbnew
except ImportError:
    raise SystemExit("Run this from KiCad's PCB editor Scripting Console (pcbnew not importable here).")

# ----------------------------------------------------------------------------- knobs (mm)
APPLY         = True          # False = dry-run (report only, no board changes)
ACTION        = "tune"        # "tune" | "revert" (undo the last apply) | "history" (list past applies)
MODE          = "auto-detect" # "auto-detect" = use selection if any else auto | "selection" | "auto"
TARGET_GROUPS = ("HSIT", "HSOL")   # AUTO mode: net-name groups (short net of each P/N pair gets the meander)
RETIMER_REF   = "U1"          # component whose pads define "near the retimer" ordering (AUTO mode)

CLEAR       = 0.1016          # min bump edge -> other-net copper edge
VIA_CLEAR   = 0.21            # min bump edge -> any via copper edge
PARTNER_VIA_CLEAR = 0.1016    # min PARTNER-thickening edge -> via copper edge -- looser than the meander's
                              #   VIA_CLEAR so the mirror can swell toward a GND via fence (DRC netclass min)
PAD_CLEAR   = 0.50            # min bump edge -> non-own-net pad edge (esp. U1 pads)
BUMP_GAP    = 0.30            # min clearance to a different net's meander excursion
THICKEN_FACTOR = 1.5          # max meander width = THICKEN_FACTOR x the diff-pair trace width (relative)
THICKEN_STEPS  = 4            # sub-segments used to taper each 45deg slope (higher = smoother gradient)
PARTNER_THICKEN = True        # also thicken the non-meandered partner: mirror the bumps (swell opposite
                              #   each bump, back to normal between) on the partner's own centerline --
                              #   length-neutral, so skew stays nulled and the intra-pair gap is kept
PARTNER_FACTOR  = None        # partner max width factor; None = match THICKEN_FACTOR (the meander)
THICKEN_MIN_SKEW = 0.15       # if a pair's skew correction is below this, DON'T thicken (plain w0 meander
                              #   + skip the partner mirror) -- a tiny bump has no room to taper nicely
H_MAX       = 0.30            # max bump height (outboard excursion)
H_TARGET    = 0.22            # preferred (small) trapezoid height for distribution
W_TOP       = 0.22            # trapezoid flat-top length
GAP_BUMPS   = 0.30            # min gap between consecutive bumps on the same trace
MARGIN      = 0.25            # min distance from a bump to a trace corner (host run end)
SKEW_FLOOR  = 0.010           # skip pairs whose |skew| is below this (fab tolerance)
CLR_MARGIN  = 0.010           # extra breathing room beyond the hard clearances (DRC safety)
PARTNER_TOL = 0.020           # a bump may reach the pair's own min gap, but not push INTO the partner
PAD_ANCHOR  = True            # measure skew pad-anchor to pad-anchor (add each net-end -> pad-centre gap)
                              #   so the target matches KiCad's Routed Length even if a trace doesn't
                              #   land dead-centre on a pad; a no-op when it already ends on the anchor
EPS         = 1e-6
STEP        = 0.04            # arc/pad sampling step
SLOPE       = 2 * (math.sqrt(2) - 1)   # trapezoid added length per bump = SLOPE * h

# ----------------------------------------------------------------------------- geometry helpers
def dist_seg(p, a, b):
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx - ax, by - ay; L2 = dx * dx + dy * dy
    if L2 < 1e-15:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))

def seg_seg_dist(p1, p2, p3, p4):
    def ccw(a, b, c):
        return (c[1] - a[1]) * (b[0] - a[0]) - (b[1] - a[1]) * (c[0] - a[0])
    if (ccw(p1, p3, p4) * ccw(p2, p3, p4) < 0) and (ccw(p1, p2, p3) * ccw(p1, p2, p4) < 0):
        return 0.0
    return min(dist_seg(p1, p3, p4), dist_seg(p2, p3, p4), dist_seg(p3, p1, p2), dist_seg(p4, p1, p2))

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
    return dict(c=c, r=r, s=s, ccw=ccw, extent=extent, a=a, b=b)

def dist_arc(p, arc):
    c = arc["c"]; ang = math.atan2(p[1] - c[1], p[0] - c[0])
    d = (ang - arc["s"]) % (2 * math.pi) if arc["ccw"] else (arc["s"] - ang) % (2 * math.pi)
    if d <= arc["extent"] + 1e-9:
        return abs(math.hypot(p[0] - c[0], p[1] - c[1]) - arc["r"])
    return min(math.hypot(p[0] - arc["a"][0], p[1] - arc["a"][1]),
               math.hypot(p[0] - arc["b"][0], p[1] - arc["b"][1]))

def pad_dist(p, pad):
    cx, cy, sx, sy, ang = pad["cx"], pad["cy"], pad["sx"], pad["sy"], pad["ang"]
    dx, dy = p[0] - cx, p[1] - cy; ca, sa = math.cos(ang), math.sin(ang)
    u = dx * ca - dy * sa; v = dx * sa + dy * ca
    return math.hypot(max(abs(u) - sx / 2, 0.0), max(abs(v) - sy / 2, 0.0))

def sample_edge(p0, p1):
    L = math.hypot(p1[0] - p0[0], p1[1] - p0[1]); n = max(1, int(math.ceil(L / STEP)))
    return [(p0[0] + (k / n) * (p1[0] - p0[0]), p0[1] + (k / n) * (p1[1] - p0[1])) for k in range(n + 1)]

# ----------------------------------------------------------------------------- board readout
def to_mm(v):
    return pcbnew.ToMM(v)

def P2(vec):
    return (pcbnew.ToMM(vec.x), pcbnew.ToMM(vec.y))

def read_board(board):
    data = dict(segs=[], arcs=[], vias=[], pads=[], u1pads={}, netname={},
                seg_by_net=collections.defaultdict(list), arc_by_net=collections.defaultdict(list),
                length=collections.Counter())
    for t in board.GetTracks():
        net = t.GetNetCode(); data["netname"][net] = t.GetNetname()
        if isinstance(t, pcbnew.PCB_VIA):
            data["vias"].append(dict(c=P2(t.GetPosition()), r=to_mm(t.GetWidth(t.TopLayer())) / 2.0, net=net))
            continue
        if isinstance(t, pcbnew.PCB_ARC):
            a, mid, b = P2(t.GetStart()), P2(t.GetMid()), P2(t.GetEnd())
            arc = build_arc(a, mid, b)
            if arc is None:
                continue
            d = dict(a=a, b=b, w=to_mm(t.GetWidth()), layer=t.GetLayer(), net=net, arc=arc, obj=t)
            data["arcs"].append(d); data["arc_by_net"][net].append(d)
        else:
            d = dict(a=P2(t.GetStart()), b=P2(t.GetEnd()), w=to_mm(t.GetWidth()),
                     layer=t.GetLayer(), net=net, obj=t)
            data["segs"].append(d); data["seg_by_net"][net].append(d)
        data["length"][net] += to_mm(t.GetLength())
    for fp in board.GetFootprints():
        is_u1 = (fp.GetReference() == RETIMER_REF)
        for pad in fp.Pads():
            pos = P2(pad.GetPosition()); sz = pad.GetSize()
            rec = dict(cx=pos[0], cy=pos[1], sx=to_mm(sz.x), sy=to_mm(sz.y),
                       ang=math.radians(pad.GetOrientationDegrees()), net=pad.GetNetCode(), obj=pad)
            data["pads"].append(rec)
            if is_u1:
                data["u1pads"][pad.GetNetCode()] = pos
    data["pad_stub"] = collections.Counter()          # net-end -> pad-anchor gap, so len == KiCad's
    if PAD_ANCHOR:
        verts = collections.defaultdict(list)
        for s in data["segs"]:
            verts[s["net"]].extend((s["a"], s["b"]))
        for a in data["arcs"]:
            verts[a["net"]].extend((a["a"], a["b"]))
        for pd in data["pads"]:
            vs = verts.get(pd["net"])
            if not vs:
                continue
            ax, ay = pd["cx"], pd["cy"]
            dmin = min(math.hypot(v[0] - ax, v[1] - ay) for v in vs)
            if dmin <= 0.5 * math.hypot(pd["sx"], pd["sy"]) + 0.05:   # a track end lands in this pad
                data["pad_stub"][pd["net"]] += dmin
    data["name2net"] = {v: k for k, v in data["netname"].items()}
    return data

# ----------------------------------------------------------------------------- spatial hash
CELL = 0.6
def cells(x0, y0, x1, y1, pad=0.7):
    for cx in range(int((x0 - pad) // CELL), int((x1 + pad) // CELL) + 1):
        for cy in range(int((y0 - pad) // CELL), int((y1 + pad) // CELL) + 1):
            yield (cx, cy)

class Oracle:
    """Clearance oracle over board copper (+ meanders added as we go)."""
    def __init__(self, data):
        self.grid = collections.defaultdict(list)
        for s in data["segs"]:
            self._add("seg", s, (min(s["a"][0], s["b"][0]), min(s["a"][1], s["b"][1]),
                                 max(s["a"][0], s["b"][0]), max(s["a"][1], s["b"][1])))
        for a in data["arcs"]:
            r = a["arc"]["r"]; c = a["arc"]["c"]
            self._add("arc", a, (c[0] - r, c[1] - r, c[0] + r, c[1] + r))
        for v in data["vias"]:
            self._add("via", v, (v["c"][0], v["c"][1], v["c"][0], v["c"][1]))
        for p in data["pads"]:
            rr = 0.5 * math.hypot(p["sx"], p["sy"])
            self._add("pad", p, (p["cx"] - rr, p["cy"] - rr, p["cx"] + rr, p["cy"] + rr))

    def _add(self, kind, obj, bb):
        for c in cells(*bb):
            self.grid[c].append((kind, obj))

    def add_bump(self, a, b, w, layer, net):
        o = dict(a=a, b=b, w=w, layer=layer, net=net)
        self._add("bump", o, (min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])))

    def edge_ok(self, p0, p1, layer, skip_nets, hb, via_clear=None):
        """min clearance slack of a new edge (>=0 means it meets every rule)."""
        vc = VIA_CLEAR if via_clear is None else via_clear
        worst = 1e9; seen = set()
        x0, y0 = min(p0[0], p1[0]), min(p0[1], p1[1]); x1, y1 = max(p0[0], p1[0]), max(p0[1], p1[1])
        for c in cells(x0, y0, x1, y1):
            for (kind, obj) in self.grid.get(c, ()):
                oid = id(obj)
                if oid in seen:
                    continue
                seen.add(oid)
                if kind == "seg":
                    if obj["net"] in skip_nets or obj["layer"] != layer:
                        continue
                    slack = seg_seg_dist(p0, p1, obj["a"], obj["b"]) - hb - obj["w"] / 2 - CLEAR
                elif kind == "bump":
                    if obj["net"] in skip_nets or obj["layer"] != layer:
                        continue
                    slack = seg_seg_dist(p0, p1, obj["a"], obj["b"]) - hb - obj["w"] / 2 - BUMP_GAP
                elif kind == "arc":
                    if obj["net"] in skip_nets or obj["layer"] != layer:
                        continue
                    slack = min(dist_arc(p, obj["arc"]) for p in sample_edge(p0, p1)) - hb - obj["w"] / 2 - CLEAR
                elif kind == "via":
                    if obj["net"] in skip_nets:
                        continue
                    slack = dist_seg(obj["c"], p0, p1) - hb - obj["r"] - vc
                else:  # pad
                    if obj["net"] in skip_nets or not obj["obj"].IsOnLayer(layer):
                        continue
                    slack = min(pad_dist(p, obj) for p in sample_edge(p0, p1)) - hb - PAD_CLEAR
                if slack < worst:
                    worst = slack
        return worst

# ----------------------------------------------------------------------------- path -> runs
def order_path(data, net):
    ss = []
    for s in data["seg_by_net"][net]:
        ss.append(dict(kind="seg", a=s["a"], b=s["b"], w=s["w"], layer=s["layer"], ref=s))
    for a in data["arc_by_net"][net]:
        ss.append(dict(kind="arc", a=a["a"], b=a["b"], w=a["w"], layer=a["layer"], ref=a))
    pad = data["u1pads"].get(net)
    if not pad:
        return ss
    cur = pad; path = []; used = [False] * len(ss)
    for _ in range(len(ss)):
        best = -1; flip = False; bd = 1e9
        for i, s in enumerate(ss):
            if used[i]:
                continue
            d1 = math.hypot(s["a"][0] - cur[0], s["a"][1] - cur[1])
            d2 = math.hypot(s["b"][0] - cur[0], s["b"][1] - cur[1])
            if d1 < bd:
                bd, best, flip = d1, i, False
            if d2 < bd:
                bd, best, flip = d2, i, True
        if best < 0 or bd > 0.25:
            break
        s = dict(ss[best]); used[best] = True
        if flip:
            s["a"], s["b"] = s["b"], s["a"]
        path.append(s); cur = s["b"]
    return path

def plen(path):
    return sum(math.hypot(s["b"][0] - s["a"][0], s["b"][1] - s["a"][1]) for s in path)

def build_runs(data, path):
    def has_via_at(pt):
        return any(math.hypot(v["c"][0] - pt[0], v["c"][1] - pt[1]) < 0.06 for v in data["vias"])
    runs = []; cur = None
    for el in path:
        if el["kind"] != "seg":
            cur = None; continue
        d = math.hypot(el["b"][0] - el["a"][0], el["b"][1] - el["a"][1])
        if d < 1e-6:
            continue
        dv = ((el["b"][0] - el["a"][0]) / d, (el["b"][1] - el["a"][1]) / d)
        if cur is not None:
            dot = cur["dir"][0] * dv[0] + cur["dir"][1] * dv[1]
            cont = math.hypot(el["a"][0] - cur["b"][0], el["a"][1] - cur["b"][1]) < 1e-4
            if dot > 0.9999 and cont and not has_via_at(el["a"]):
                cur["b"] = el["b"]; cur["members"].append(el["ref"]); continue
        cur = dict(a=el["a"], b=el["b"], w=el["w"], layer=el["layer"], dir=dv, members=[el["ref"]])
        runs.append(cur)
    return runs

def _closest_point(p, a, b):
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx - ax, by - ay; L2 = dx * dx + dy * dy
    if L2 < 1e-15:
        return (ax, ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return (ax + t * dx, ay + t * dy)

def partner_slack(p0, p1, hb, paired_path, layer):
    """Min clearance slack of an excursion edge vs the partner path (catches wrong-side placement)."""
    worst = 1e9
    for s in paired_path:
        if s.get("layer") != layer:
            continue
        sl = seg_seg_dist(p0, p1, s["a"], s["b"]) - hb - s["w"] / 2 - CLEAR
        if sl < worst:
            worst = sl
    return worst

def choose_outboard(seg, paired_path):
    A, B = seg["a"], seg["b"]; M = ((A[0] + B[0]) / 2, (A[1] + B[1]) / 2)
    ux, uy = B[0] - A[0], B[1] - A[1]; L = math.hypot(ux, uy)
    if L < 1e-9:
        return +1
    ux, uy = ux / L, uy / L; nx, ny = -uy, ux
    best = None; bd = 1e9                         # nearest POINT on the partner path to the run midpoint
    for s in paired_path:
        cp = _closest_point(M, s["a"], s["b"])
        d = math.hypot(cp[0] - M[0], cp[1] - M[1])
        if d < bd:
            bd, best = d, cp
    if best is None:
        return +1
    return -1 if ((best[0] - M[0]) * nx + (best[1] - M[1]) * ny) > 0 else +1

# ----------------------------------------------------------------------------- trapezoid + placement
def _gwidth(o, w0, h, factor=None):
    # Gradual taper: width ramps LINEARLY with the outboard fraction o/h, from w0 (at the pair) to
    # factor*w0 (at the flat top). 2*o+w0 is a clearance guard that keeps the inboard edge off the
    # pair near the base; it no longer forces an abrupt jump to max width.
    f = THICKEN_FACTOR if factor is None else factor
    wmax = f * w0
    frac = 0.0 if h <= 1e-9 else max(0.0, min(1.0, o / h))
    ramp = w0 + (wmax - w0) * frac
    return max(w0, min(ramp, 2.0 * o + w0, wmax))

def build_bumps(A, B, w0, side, N, h, s0, thicken=True):
    ux = B[0] - A[0]; uy = B[1] - A[1]; L = math.hypot(ux, uy); ux, uy = ux / L, uy / L
    nx, ny = -uy * side, ux * side
    def P(t, o=0.0):
        return (A[0] + ux * t + nx * o, A[1] + uy * t + ny * o)
    edges = []; steps = max(1, THICKEN_STEPS); fac = None if thicken else 1.0
    pitch = (2 * h + W_TOP) + GAP_BUMPS
    edges.append((A, P(s0), w0, "base"))
    for k in range(N):
        s = s0 + k * pitch
        for j in range(steps):                       # up-slope: thickens moving AWAY from the pair
            o1, o2 = h * j / steps, h * (j + 1) / steps
            edges.append((P(s + o1, o1), P(s + o2, o2), _gwidth(o1, w0, h, fac), "exc"))
        edges.append((P(s + h, h), P(s + h + W_TOP, h), _gwidth(h, w0, h, fac), "exc"))   # flat top
        base = s + h + W_TOP
        for j in range(steps):                       # down-slope: thins back toward the pair
            o1, o2 = h * (steps - j) / steps, h * (steps - j - 1) / steps
            edges.append((P(base + (h - o1), o1), P(base + (h - o2), o2), _gwidth(o2, w0, h, fac), "exc"))
        if k < N - 1:
            edges.append((P(s + 2 * h + W_TOP), P(s0 + (k + 1) * pitch), w0, "base"))
    edges.append((P(s0 + (N - 1) * pitch + 2 * h + W_TOP), B, w0, "base"))
    return edges

def try_place(oracle, run, side, k, h, skip_nets, paired_path=(), thicken=True):
    A, B = run["a"], run["b"]; w0 = run["w"]; layer = run["layer"]
    segL = math.hypot(B[0] - A[0], B[1] - A[1])
    F = k * (2 * h + W_TOP) + (k - 1) * GAP_BUMPS
    if F > segL - 2 * MARGIN + 1e-9:
        return None
    s0 = MARGIN
    while s0 <= segL - F - MARGIN + 1e-9:
        edges = build_bumps(A, B, w0, side, k, h, s0, thicken)
        mn = 1e9; pworst = 1e9
        for (p0, p1, w, kind) in edges:
            if kind == "exc":
                m = oracle.edge_ok(p0, p1, layer, skip_nets, w / 2)
                if m < mn:
                    mn = m
                ps = partner_slack(p0, p1, w / 2, paired_path, layer)   # <0 only if pushing into partner
                if ps < pworst:
                    pworst = ps
            if mn < CLR_MARGIN:
                break
        if mn >= CLR_MARGIN and pworst >= -PARTNER_TOL:
            return dict(run=run, members=run["members"], edges=edges, layer=layer,
                        N=k, h=h, s0=s0, side=side, minclr=mn, pair=skip_nets)
        s0 += 0.1
    return None

def distribute(oracle, runs, paired_path, skip_nets, M, h, thicken=True):
    placements = []; left = M
    for run in runs:
        if left <= 0:
            break
        pref = choose_outboard(run, paired_path)
        segL = math.hypot(run["b"][0] - run["a"][0], run["b"][1] - run["a"][1])
        cap = int((segL - 2 * MARGIN + GAP_BUMPS) // (2 * h + W_TOP + GAP_BUMPS))
        placed = None
        for side in (pref, -pref):                   # prefer outboard; fall back to the other side
            k = min(cap, left)
            while k >= 1:
                p = try_place(oracle, run, side, k, h, skip_nets, paired_path, thicken)
                if p:
                    placed = p; break
                k -= 1
            if placed:
                break
        if placed:
            placements.append(placed); left -= placed["N"]
    return placements, left

def dU1_of(data, p, short_net):
    u1p = data["u1pads"].get(short_net)
    if not u1p:
        return 0.0
    A, B = p["run"]["a"], p["run"]["b"]; Ls = math.hypot(B[0] - A[0], B[1] - A[1])
    F = p["N"] * (2 * p["h"] + W_TOP) + (p["N"] - 1) * GAP_BUMPS
    t = p["s0"] + F / 2
    bc = (A[0] + (B[0] - A[0]) / Ls * t, A[1] + (B[1] - A[1]) / Ls * t)
    return math.hypot(bc[0] - u1p[0], bc[1] - u1p[1])

# ----------------------------------------------------------------------------- main
# ----------------------------------------------------------------------------- diff-pair / selection helpers
def run_len(r):
    return math.hypot(r["b"][0] - r["a"][0], r["b"][1] - r["a"][1])

def diff_partner(name):
    """The complementary net name of a diff pair, by suffix convention (P/N, +/-, _P/_N)."""
    if not name:
        return None
    for a, b in (("_P", "_N"), ("_N", "_P")):
        if name.endswith(a):
            return name[:-2] + b
    for a, b in (("P", "N"), ("N", "P"), ("+", "-"), ("-", "+")):
        if name.endswith(a):
            return name[:-1] + b
    return None

def order_chain(els):
    """Order/orient loose segments head-to-tail into chains (build_runs then merges collinear)."""
    Q = lambda p: (round(p[0], 4), round(p[1], 4))
    adj = collections.defaultdict(list)
    for i, e in enumerate(els):
        adj[Q(e["a"])].append((i, 0)); adj[Q(e["b"])].append((i, 1))
    used = [False] * len(els); out = []
    def walk(key):
        while True:
            nxt = next(((i, end) for (i, end) in adj[key] if not used[i]), None)
            if nxt is None:
                break
            i, end = nxt; used[i] = True; e = dict(els[i])
            if end == 1:
                e["a"], e["b"] = e["b"], e["a"]
            out.append(e); key = Q(e["b"])
    for key, lst in list(adj.items()):
        if sum(1 for (i, _) in lst if not used[i]) == 1:
            walk(key)
    for i in range(len(els)):
        if not used[i]:
            walk(Q(els[i]["a"]))
    return out

def distribute_selection(oracle, runs, paired_path, skip, skew, short_net, thicken=True):
    """Add up to `skew` of length across the given runs; exact if it fits, else as much as possible."""
    placements = []; added = 0.0
    for run_ in runs:
        if added >= skew - 1e-4:
            break
        pref = choose_outboard(run_, paired_path)
        segL = run_len(run_); remaining = skew - added
        cap = int((segL - 2 * MARGIN + GAP_BUMPS) // (2 * H_TARGET + W_TOP + GAP_BUMPS))
        if cap < 1:
            continue
        need = int(math.ceil(remaining / (SLOPE * H_TARGET)))
        placed = None
        for side in (pref, -pref):                   # prefer outboard; fall back to the other side
            for k in range(min(cap, need), 0, -1):
                h = min(H_MAX, remaining / (SLOPE * k)) if k >= need else H_TARGET
                p = try_place(oracle, run_, side, k, h, skip, paired_path, thicken)
                if p:
                    placed = (p, k, h); break
            if placed:
                break
        if placed:
            p, k, h = placed
            for (p0, p1, w, kind) in p["edges"]:
                oracle.add_bump(p0, p1, w, p["layer"], short_net)
            placements.append(p); added += k * SLOPE * h
    return placements, added

# ----------------------------------------------------------------------------- partner mirror
def plan_partner_mirror(oracle, p, pruns):
    """Mirror the meander onto the partner: swell its width opposite each bump (where the meander has
    moved away, so there is room), back to w0 opposite the gaps -- on the partner's own straight
    centerline, so its length (and the pair skew) is unchanged and the intra-pair gap is preserved.
    Returns (partner_run, edges) or None if no aligned straight partner run covers the bump span."""
    factor_p = PARTNER_FACTOR if PARTNER_FACTOR else THICKEN_FACTOR
    if factor_p <= 1.0 + 1e-9:
        return None
    A, B = p["run"]["a"], p["run"]["b"]
    ux, uy = B[0] - A[0], B[1] - A[1]; L = math.hypot(ux, uy)
    if L < 1e-6:
        return None
    ux, uy = ux / L, uy / L
    h = p["h"]; N = p["N"]; s0 = p["s0"]; layer = p["layer"]
    pitch_b = (2 * h + W_TOP) + GAP_BUMPS
    s_lo, s_hi = s0, s0 + (N - 1) * pitch_b + (2 * h + W_TOP)
    best = None; bestd = 1e9
    for R in pruns:
        if R["layer"] != layer:
            continue
        vx, vy = R["b"][0] - R["a"][0], R["b"][1] - R["a"][1]; Lv = math.hypot(vx, vy)
        if Lv < 1e-6:
            continue
        vx, vy = vx / Lv, vy / Lv
        if abs(vx * uy - vy * ux) > 0.02:                # not parallel to the meander run
            continue
        sa = (R["a"][0] - A[0]) * ux + (R["a"][1] - A[1]) * uy
        sb = (R["b"][0] - A[0]) * ux + (R["b"][1] - A[1]) * uy
        if min(sa, sb) > s_lo + 1e-3 or max(sa, sb) < s_hi - 1e-3:
            continue                                     # doesn't span the bumps
        perp = abs((R["a"][0] - A[0]) * (-uy) + (R["a"][1] - A[1]) * ux)
        if perp < 0.05 or perp > 1.0:                    # not the adjacent partner run
            continue
        if perp < bestd:
            bestd, best = perp, (R, sa, vx, vy)
    if not best:
        return None
    R, sa, vx, vy = best
    w0p = R["w"]; duv = vx * ux + vy * uy
    def ppt(s):
        t = (s - sa) * duv
        return (R["a"][0] + vx * t, R["a"][1] + vy * t)
    def wprof(s):
        for k in range(N):
            b0 = s0 + k * pitch_b
            if b0 - 1e-9 <= s <= b0 + 2 * h + W_TOP + 1e-9:
                if s <= b0 + h:
                    o = s - b0
                elif s <= b0 + h + W_TOP:
                    o = h
                else:
                    o = (b0 + 2 * h + W_TOP) - s
                return _gwidth(max(0.0, o), w0p, h, factor_p)
        return w0p
    steps = max(1, THICKEN_STEPS)
    sb_end = (R["b"][0] - A[0]) * ux + (R["b"][1] - A[1]) * uy
    smin, smax = min(sa, sb_end), max(sa, sb_end)
    cuts = set([smin, smax])
    for k in range(N):
        b0 = s0 + k * pitch_b
        for j in range(steps + 1):                        # uniform sub-segs on BOTH slopes and the flat
            cuts.add(b0 + h * j / steps)
            cuts.add(b0 + h + W_TOP * j / steps)
            cuts.add(b0 + h + W_TOP + h * j / steps)
    cuts = sorted(c for c in cuts if smin - 1e-9 <= c <= smax + 1e-9)
    def bump_of(s):
        for k in range(N):
            b0 = s0 + k * pitch_b
            if b0 - 1e-9 <= s <= b0 + 2 * h + W_TOP + 1e-9:
                return k
        return -1
    # Per sub-seg: the mirror target width, clamped to what actually fits (clearance to other nets and
    # to the meander). Sub-segs are uniform length so the clamp is fair -- a single long flat-top would
    # otherwise be over-clamped by its worst point while short neighbours escape, sawtoothing the edge.
    raw = []                                              # [q0, q1, wm, k, offset-from-bump-centre]
    for i in range(len(cuts) - 1):
        s_a, s_b = cuts[i], cuts[i + 1]
        if s_b - s_a < 1e-6:
            continue
        sm = 0.5 * (s_a + s_b); k = bump_of(sm); wm = wprof(sm)
        q0, q1 = ppt(s_a), ppt(s_b)
        if k >= 0 and wm > w0p + 1e-9:
            slack = oracle.edge_ok(q0, q1, layer, p["pair"], wm / 2, via_clear=PARTNER_VIA_CLEAR)
            for (m0, m1, mw, mk) in p["edges"]:           # keep CLEAR from the meander itself (same pair)
                dd = seg_seg_dist(q0, q1, m0, m1) - wm / 2 - mw / 2 - CLEAR
                if dd < slack:
                    slack = dd
            if slack < CLR_MARGIN:
                wm = max(w0p, wm + 2.0 * (slack - CLR_MARGIN))
        coff = abs(sm - (s0 + k * pitch_b + h + 0.5 * W_TOP)) if k >= 0 else 0.0
        raw.append([q0, q1, wm, k, coff])
    # Shape each swell into a clean, symmetric, single-peak bulge: walking outward from the bump centre
    # the width may only stay level or shrink, and the two symmetric sub-segs at each offset share the
    # smaller width. This erases any residual clamp sawtooth while never widening past what fits.
    for k in range(N):
        groups = {}
        for j, r in enumerate(raw):
            if r[3] == k:
                groups.setdefault(round(r[4], 6), []).append(j)
        run = wprof(s0 + k * pitch_b + h + 0.5 * W_TOP)
        for key in sorted(groups):
            run = min(run, min(raw[j][2] for j in groups[key]))
            for j in groups[key]:
                raw[j][2] = run
    edges = [(q0, q1, wm, "pexc" if wm > w0p + 1e-9 else "pbase") for (q0, q1, wm, k, coff) in raw]
    if not any(k == "pexc" for (_, _, _, k) in edges):
        return None
    return R, edges

def _plan_partners(data, oracle, placements, short_net, partner_net):
    if not PARTNER_THICKEN:
        return
    pruns = build_runs(data, order_path(data, partner_net))
    used = set()
    for p in placements:
        pm = plan_partner_mirror(oracle, p, pruns)
        if not pm:
            continue
        R, pedges = pm
        key = tuple(sorted(id(m["obj"]) for m in R["members"]))
        if key in used:
            continue
        used.add(key)
        p["pnet"] = partner_net
        p["premoves"] = [m["obj"] for m in R["members"]]
        p["pedges"] = pedges
        for (q0, q1, w, kind) in pedges:
            if kind == "pexc":
                oracle.add_bump(q0, q1, w, p["layer"], partner_net)

# ----------------------------------------------------------------------------- modes
def run_auto(data, oracle):
    n2n = data["name2net"]; results = []
    print("== AUTO: tuning %s pairs, meanders near %s ==" % ("/".join(TARGET_GROUPS), RETIMER_REF))
    print("pair     add_um short    M bumps  h_um runs  dU1(min-max) clr_um  status")
    for grp in TARGET_GROUPS:
        for lane in range(8):
            nP = n2n.get("/%s%dP" % (grp, lane)); nN = n2n.get("/%s%dN" % (grp, lane))
            if nP is None or nN is None:
                continue
            pathP, pathN = order_path(data, nP), order_path(data, nN)
            lP = plen(pathP) + data["pad_stub"][nP]
            lN = plen(pathN) + data["pad_stub"][nN]
            add = abs(lP - lN)
            short_net, short_path, paired_path = (nN, pathN, pathP) if lN < lP else (nP, pathP, pathN)
            if add < SKEW_FLOOR:
                print("%s%-7d%7.0f %-8s below floor, skip" % (grp, lane, add * 1000, "-"))
                continue
            skip = {nP, nN}
            runs = [r for r in build_runs(data, short_path) if run_len(r) >= W_TOP + 2 * MARGIN]
            Nmin = max(1, int(math.ceil(add / (SLOPE * H_MAX))))
            Mtarget = max(Nmin, int(math.ceil(add / (SLOPE * H_TARGET))))
            chosen = None; used_M = 0; used_h = 0.0
            thick = add >= THICKEN_MIN_SKEW
            for M in range(Mtarget, Nmin - 1, -1):
                h = add / (SLOPE * M)
                if h > H_MAX + 1e-9:
                    continue
                pl, left = distribute(oracle, runs, paired_path, skip, M, h, thick)
                if left == 0:
                    chosen, used_M, used_h = pl, M, h
                    break
            name = data["netname"][short_net].split("/")[-1]
            if not chosen:
                print("%s%-7d%7.0f %-8s NO FEASIBLE DISTRIBUTION" % (grp, lane, add * 1000, name))
                continue
            for p in chosen:
                for (p0, p1, w, kind) in p["edges"]:
                    oracle.add_bump(p0, p1, w, p["layer"], short_net)
            if thick:
                _plan_partners(data, oracle, chosen, short_net, (nP if short_net == nN else nN))
            results.append((short_net, chosen, add, add))
            dU1s = sorted(dU1_of(data, p, short_net) for p in chosen)
            nb = sum(p["N"] for p in chosen); mnclr = min(p["minclr"] for p in chosen)
            print("%s%-7d%7.0f %-8s%2d%6d%6.0f%5d   %4.2f-%-4.2f %5.0f  OK"
                  % (grp, lane, add * 1000, name, used_M, nb, used_h * 1000, len(chosen),
                     dU1s[0], dU1s[-1], mnclr * 1000))
    return results

def run_selection(data, oracle, sel):
    n2n = data["name2net"]; results = []
    by_net = collections.defaultdict(list)
    for s in sel:
        by_net[s["net"]].append(s)
    print("== SELECTION: %d track segment(s) on %d net(s) ==" % (len(sel), len(by_net)))
    print("pair         skew_um  add_um  resid_um  bumps  status")
    handled = set()
    for net in list(by_net):
        if net in handled:
            continue
        nm = data["netname"].get(net, "")
        pname = diff_partner(nm); partner = n2n.get(pname) if pname else None
        if partner is None:
            print("  %-10s no diff-pair partner (%s) -- skip" % (nm.split("/")[-1], pname))
            continue
        handled.add(net); handled.add(partner)
        lA = data["length"][net] + data["pad_stub"][net]
        lB = data["length"][partner] + data["pad_stub"][partner]
        skew = abs(lA - lB)
        short = net if lA < lB else partner
        base = data["netname"][short].split("/")[-1]
        if skew < SKEW_FLOOR:
            print("  %-10s skew %.0fum below floor, skip" % (base, skew * 1000))
            continue
        skip = {net, partner}
        other = partner if short == net else net       # the non-meandered (longer) net of the pair
        paired_path = order_path(data, other)
        host_segs = by_net.get(short, [])
        if host_segs:                                  # place meanders on the selected shorter-net segments
            host_path = order_chain([dict(kind="seg", a=s["a"], b=s["b"], w=s["w"],
                                          layer=s["layer"], ref=s) for s in host_segs])
        else:                                          # only the longer net was selected -> tune the whole shorter net
            print("  %-10s selected the longer net; tuning the shorter net %s over its full length" % (nm.split("/")[-1], base))
            host_path = order_path(data, short)
        runs = [r for r in build_runs(data, host_path) if run_len(r) >= W_TOP + 2 * MARGIN]
        thick = skew >= THICKEN_MIN_SKEW
        placements, added = distribute_selection(oracle, runs, paired_path, skip, skew, short, thick)
        if thick:
            _plan_partners(data, oracle, placements, short, other)
        results.append((short, placements, skew, added))
        nb = sum(p["N"] for p in placements)
        resid = skew - added
        status = "OK" if resid < 0.01 else "PARTIAL -- not enough room in selection"
        print("  %-10s %7.0f %7.0f %9.0f %6d  %s" % (base, skew * 1000, added * 1000, resid * 1000, nb, status))
    return results

def _apply_overrides(overrides):
    """Override any UPPERCASE knob at call time: run(..., PAD_CLEAR=0.6) or PARAMS=dict(PAD_CLEAR=0.6)."""
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
        board.BuildConnectivity()      # stop the ratsnest engine dereferencing removed items -> crash
    except Exception:
        pass
    try:
        pcbnew.Refresh()
    except Exception:
        pass

# ----------------------------------------------------------------------------- undo history
def _history_path(board):
    f = board.GetFileName()
    return (f + ".skewtune_history.json") if f else None

def _load_history(board):
    p = _history_path(board)
    if p and os.path.exists(p):
        try:
            with open(p, "r") as fh:
                return json.load(fh)
        except Exception:
            return []
    return []

def _save_history(board, hist):
    p = _history_path(board)
    if not p:
        print("[history] board has no filename yet (save it once) -- undo history disabled.")
        return
    try:
        with open(p, "w") as fh:
            json.dump(hist, fh)
    except Exception as e:
        print("[history] could not write %s: %s" % (p, e))

def _track_data(t):
    d = dict(layer=t.GetLayer(), width=t.GetWidth(), net=t.GetNetname(),
             start=[t.GetStart().x, t.GetStart().y], end=[t.GetEnd().x, t.GetEnd().y])
    if isinstance(t, pcbnew.PCB_ARC):
        d["kind"] = "arc"; m = t.GetMid(); d["mid"] = [m.x, m.y]
    else:
        d["kind"] = "seg"
    return d

def _record_history(board, removed, added, results):
    if not (removed or added):
        return
    hist = _load_history(board)
    hist.append(dict(note="%d net(s) tuned, +%d / -%d tracks" % (len(results), len(added), len(removed)),
                     removed=removed, added=added))
    _save_history(board, hist)
    print("[history] saved undo step #%d (%s)" % (len(hist), os.path.basename(_history_path(board) or "")))

def _netcode(board, name):
    try:
        ni = board.FindNet(name)
        if ni is not None:
            return ni.GetNetCode()
    except Exception:
        pass
    for t in board.GetTracks():
        if t.GetNetname() == name:
            return t.GetNetCode()
    return 0

def _pt_near(v, xy, tol=2000):
    return abs(v.x - xy[0]) <= tol and abs(v.y - xy[1]) <= tol

def _find_track(tracks, d):
    want_arc = (d["kind"] == "arc")
    for t in tracks:
        if isinstance(t, pcbnew.PCB_VIA):
            continue
        if isinstance(t, pcbnew.PCB_ARC) != want_arc:
            continue
        if t.GetLayer() != d["layer"] or abs(t.GetWidth() - d["width"]) > 500:
            continue
        fwd = _pt_near(t.GetStart(), d["start"]) and _pt_near(t.GetEnd(), d["end"])
        rev = _pt_near(t.GetStart(), d["end"]) and _pt_near(t.GetEnd(), d["start"])
        if not (fwd or rev):
            continue
        if want_arc and not _pt_near(t.GetMid(), d.get("mid", [0, 0])):
            continue
        return t
    return None

def _make_track(board, d):
    if d["kind"] == "arc":
        t = pcbnew.PCB_ARC(board)
        t.SetStart(pcbnew.VECTOR2I(int(d["start"][0]), int(d["start"][1])))
        t.SetMid(pcbnew.VECTOR2I(int(d["mid"][0]), int(d["mid"][1])))
        t.SetEnd(pcbnew.VECTOR2I(int(d["end"][0]), int(d["end"][1])))
    else:
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pcbnew.VECTOR2I(int(d["start"][0]), int(d["start"][1])))
        t.SetEnd(pcbnew.VECTOR2I(int(d["end"][0]), int(d["end"][1])))
    t.SetWidth(int(d["width"])); t.SetLayer(d["layer"])
    t.SetNetCode(_netcode(board, d["net"]))
    return t

def revert(board=None, steps=1):
    """Undo the last apply(es): delete the tracks it added, restore the ones it replaced."""
    if board is None:
        board = pcbnew.GetBoard()
    hist = _load_history(board)
    if not hist:
        print("No skew-tune history to revert.")
        return []
    n = min(steps, len(hist)); missing = 0
    tracks = list(board.GetTracks())          # snapshot once (repeated GetTracks can choke swig)
    for _ in range(n):
        entry = hist.pop()
        for d in entry.get("added", []):
            t = _find_track(tracks, d)
            if t is not None:
                try: t.ClearSelected()
                except Exception: pass
                board.Remove(t)
            else:
                missing += 1
        for d in entry.get("removed", []):
            board.Add(_make_track(board, d))
    _save_history(board, hist)
    _refresh(board)
    print("Reverted %d step(s)%s; %d left. Review, then save (Ctrl+S)."
          % (n, (" (%d added track(s) not found)" % missing) if missing else "", len(hist)))
    return hist

def history(board=None):
    """List the saved undo steps (most recent last)."""
    if board is None:
        board = pcbnew.GetBoard()
    hist = _load_history(board)
    if not hist:
        print("No skew-tune history.")
        return hist
    print("Skew-tune history (%s):" % os.path.basename(_history_path(board) or ""))
    for i, e in enumerate(hist, 1):
        print("  #%d  %s" % (i, e.get("note", "")))
    print("Undo the most recent with:  PARAMS=dict(ACTION='revert')  then re-exec.")
    return hist

def run(board=None, apply=None, **overrides):
    _apply_overrides(overrides)
    if board is None:
        board = pcbnew.GetBoard()
    if apply is None:
        apply = APPLY
    if ACTION == "revert":
        return revert(board)
    if ACTION == "history":
        return history(board)
    data = read_board(board)
    oracle = Oracle(data)
    sel = [s for s in data["segs"] if s["obj"].IsSelected()]
    use_sel = (MODE == "selection") or (MODE == "auto-detect" and sel)
    if use_sel and not sel:
        print("MODE wants a selection but no track segments are selected."); return []
    if use_sel:
        results = run_selection(data, oracle, sel)
    else:
        if not data["u1pads"]:
            print("WARNING: no pads for RETIMER_REF=%r -- AUTO placement won't be U1-ordered." % RETIMER_REF)
        results = run_auto(data, oracle)

    total = sum(sum(p["N"] for p in pl) for _, pl, _, _ in results)
    minslack = 1e9; viol = 0
    for net, placements, skew, added in results:
        for p in placements:
            for (p0, p1, w, kind) in p["edges"]:
                if kind != "exc":
                    continue
                sl = oracle.edge_ok(p0, p1, p["layer"], p["pair"], w / 2)
                minslack = min(minslack, sl)
                if sl < -1e-4:
                    viol += 1
            for (p0, p1, w, kind) in p.get("pedges", []):
                if kind != "pexc":
                    continue
                sl = oracle.edge_ok(p0, p1, p["layer"], p["pair"], w / 2, via_clear=PARTNER_VIA_CLEAR)
                minslack = min(minslack, sl)
                if sl < -1e-4:
                    viol += 1
    print("\n%d meander(s), %d bumps; self-check min clearance = %.1f um, violations = %d"
          % (len(results), total, (minslack * 1000 if results else 0), viol))

    if apply and results and viol == 0:
        removed = set(); h_removed = []; h_added = []
        for short_net, placements, skew, added in results:
            for p in placements:
                for m in p["members"]:
                    obj = m["obj"]
                    if id(obj) in removed:        # one host seg can back several bumps
                        continue
                    removed.add(id(obj))
                    h_removed.append(_track_data(obj))
                    try: obj.ClearSelected()      # GUI: don't leave a removed item selected
                    except Exception: pass
                    board.Remove(obj)
                for (p0, p1, w, kind) in p["edges"]:
                    t = pcbnew.PCB_TRACK(board)
                    t.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(p0[0]), pcbnew.FromMM(p0[1])))
                    t.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(p1[0]), pcbnew.FromMM(p1[1])))
                    t.SetWidth(pcbnew.FromMM(w))
                    t.SetLayer(p["layer"])
                    t.SetNetCode(short_net)
                    board.Add(t); h_added.append(_track_data(t))
                for obj in p.get("premoves", []):        # partner segs replaced by the mirrored width
                    if id(obj) in removed:
                        continue
                    removed.add(id(obj))
                    h_removed.append(_track_data(obj))
                    try: obj.ClearSelected()
                    except Exception: pass
                    board.Remove(obj)
                for (p0, p1, w, kind) in p.get("pedges", []):
                    t = pcbnew.PCB_TRACK(board)
                    t.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(p0[0]), pcbnew.FromMM(p0[1])))
                    t.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(p1[0]), pcbnew.FromMM(p1[1])))
                    t.SetWidth(pcbnew.FromMM(w))
                    t.SetLayer(p["layer"])
                    t.SetNetCode(p["pnet"])
                    board.Add(t); h_added.append(_track_data(t))
        _record_history(board, h_removed, h_added, results)
        _refresh(board)
        print("APPLIED to the board -- review, then save (Ctrl+S).")
        print("Undo:  PARAMS=dict(ACTION='revert'); exec(open(...).read())   (to tune a different segment)")
    elif viol:
        print("NOT applied: self-check found violations.")
    else:
        print("DRY RUN: set APPLY = True (or call run(apply=True)) to modify the board.")
    return results


# Optional: expose as a Tools > External Plugins button when dropped in the plugins folder.
try:
    class DiffPairSkewTuner(pcbnew.ActionPlugin):
        def defaults(self):
            self.name = "Differential-pair skew tuner"
            self.category = "Modify PCB"
            self.description = "Add trapezoidal meanders to null differential-pair intra-pair skew."
            self.show_toolbar_button = True

        def Run(self):
            run(pcbnew.GetBoard(), apply=True)
except Exception:
    pass

if __name__ == "__main__":
    # exec'd in the Scripting Console: tune the open board now
    run(**globals().get("PARAMS", {}))
else:
    # imported from the plugins folder: register the toolbar button instead of running
    try:
        DiffPairSkewTuner().register()
    except Exception:
        pass

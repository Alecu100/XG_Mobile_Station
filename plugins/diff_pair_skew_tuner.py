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
    Undo via git checkout of the board file (console edits aren't always on Ctrl+Z).

Can also be dropped in the KiCad scripting/plugins folder to appear under Tools > External Plugins.
"""
import math
import collections

try:
    import pcbnew
except ImportError:
    raise SystemExit("Run this from KiCad's PCB editor Scripting Console (pcbnew not importable here).")

# ----------------------------------------------------------------------------- knobs (mm)
APPLY         = True          # False = dry-run (report only, no board changes)
MODE          = "auto-detect" # "auto-detect" = use selection if any else auto | "selection" | "auto"
TARGET_GROUPS = ("HSIT", "HSOL")   # AUTO mode: net-name groups (short net of each P/N pair gets the meander)
RETIMER_REF   = "U1"          # component whose pads define "near the retimer" ordering (AUTO mode)

CLEAR       = 0.1016          # min bump edge -> other-net copper edge
VIA_CLEAR   = 0.21            # min bump edge -> any via copper edge
PAD_CLEAR   = 0.50            # min bump edge -> non-own-net pad edge (esp. U1 pads)
BUMP_GAP    = 0.30            # min clearance to a different net's meander excursion
THICKEN_FACTOR = 1.5          # max meander width = THICKEN_FACTOR x the diff-pair trace width (relative)
THICKEN_STEPS  = 4            # sub-segments used to taper each 45deg slope (higher = smoother gradient)
H_MAX       = 0.30            # max bump height (outboard excursion)
H_TARGET    = 0.22            # preferred (small) trapezoid height for distribution
W_TOP       = 0.22            # trapezoid flat-top length
GAP_BUMPS   = 0.30            # min gap between consecutive bumps on the same trace
MARGIN      = 0.25            # min distance from a bump to a trace corner (host run end)
SKEW_FLOOR  = 0.020           # skip pairs whose |skew| is below this (fab tolerance)
CLR_MARGIN  = 0.010           # extra breathing room beyond the hard clearances (DRC safety)
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

    def edge_ok(self, p0, p1, layer, skip_nets, hb):
        """min clearance slack of a new edge (>=0 means it meets every rule)."""
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
                    slack = dist_seg(obj["c"], p0, p1) - hb - obj["r"] - VIA_CLEAR
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

def choose_outboard(seg, paired_path):
    A, B = seg["a"], seg["b"]; M = ((A[0] + B[0]) / 2, (A[1] + B[1]) / 2)
    ux, uy = B[0] - A[0], B[1] - A[1]; L = math.hypot(ux, uy); ux, uy = ux / L, uy / L
    nx, ny = -uy, ux
    best = None; bd = 1e9
    for s in paired_path:
        d = dist_seg(M, s["a"], s["b"])
        if d < bd:
            bd, best = d, s
    if best is None:
        return +1
    pm = ((best["a"][0] + best["b"][0]) / 2, (best["a"][1] + best["b"][1]) / 2)
    return -1 if ((pm[0] - M[0]) * nx + (pm[1] - M[1]) * ny) > 0 else +1

# ----------------------------------------------------------------------------- trapezoid + placement
def _gwidth(o, w0, h):
    # Gradual taper: width ramps LINEARLY with the outboard fraction o/h, from w0 (at the pair) to
    # THICKEN_FACTOR*w0 (at the flat top). 2*o+w0 is a clearance guard that keeps the inboard edge
    # off the pair near the base; it no longer forces an abrupt jump to max width.
    wmax = THICKEN_FACTOR * w0
    frac = 0.0 if h <= 1e-9 else max(0.0, min(1.0, o / h))
    ramp = w0 + (wmax - w0) * frac
    return max(w0, min(ramp, 2.0 * o + w0, wmax))

def build_bumps(A, B, w0, side, N, h, s0):
    ux = B[0] - A[0]; uy = B[1] - A[1]; L = math.hypot(ux, uy); ux, uy = ux / L, uy / L
    nx, ny = -uy * side, ux * side
    def P(t, o=0.0):
        return (A[0] + ux * t + nx * o, A[1] + uy * t + ny * o)
    edges = []; steps = max(1, THICKEN_STEPS)
    pitch = (2 * h + W_TOP) + GAP_BUMPS
    edges.append((A, P(s0), w0, "base"))
    for k in range(N):
        s = s0 + k * pitch
        for j in range(steps):                       # up-slope: thickens moving AWAY from the pair
            o1, o2 = h * j / steps, h * (j + 1) / steps
            edges.append((P(s + o1, o1), P(s + o2, o2), _gwidth(o1, w0, h), "exc"))
        edges.append((P(s + h, h), P(s + h + W_TOP, h), _gwidth(h, w0, h), "exc"))   # flat top
        base = s + h + W_TOP
        for j in range(steps):                       # down-slope: thins back toward the pair
            o1, o2 = h * (steps - j) / steps, h * (steps - j - 1) / steps
            edges.append((P(base + (h - o1), o1), P(base + (h - o2), o2), _gwidth(o2, w0, h), "exc"))
        if k < N - 1:
            edges.append((P(s + 2 * h + W_TOP), P(s0 + (k + 1) * pitch), w0, "base"))
    edges.append((P(s0 + (N - 1) * pitch + 2 * h + W_TOP), B, w0, "base"))
    return edges

def try_place(oracle, run, side, k, h, skip_nets):
    A, B = run["a"], run["b"]; w0 = run["w"]; layer = run["layer"]
    segL = math.hypot(B[0] - A[0], B[1] - A[1])
    F = k * (2 * h + W_TOP) + (k - 1) * GAP_BUMPS
    if F > segL - 2 * MARGIN + 1e-9:
        return None
    s0 = MARGIN
    while s0 <= segL - F - MARGIN + 1e-9:
        edges = build_bumps(A, B, w0, side, k, h, s0)
        mn = 1e9
        for (p0, p1, w, kind) in edges:
            if kind == "exc":
                m = oracle.edge_ok(p0, p1, layer, skip_nets, w / 2)
                if m < mn:
                    mn = m
            if mn < CLR_MARGIN:
                break
        if mn >= CLR_MARGIN:
            return dict(run=run, members=run["members"], edges=edges, layer=layer,
                        N=k, h=h, s0=s0, side=side, minclr=mn, pair=skip_nets)
        s0 += 0.1
    return None

def distribute(oracle, runs, paired_path, skip_nets, M, h):
    placements = []; left = M
    for run in runs:
        if left <= 0:
            break
        side = choose_outboard(run, paired_path)
        segL = math.hypot(run["b"][0] - run["a"][0], run["b"][1] - run["a"][1])
        cap = int((segL - 2 * MARGIN + GAP_BUMPS) // (2 * h + W_TOP + GAP_BUMPS))
        k = min(cap, left)
        while k >= 1:
            p = try_place(oracle, run, side, k, h, skip_nets)
            if p:
                placements.append(p); left -= k; break
            k -= 1
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

def distribute_selection(oracle, runs, paired_path, skip, skew, short_net):
    """Add up to `skew` of length across the given runs; exact if it fits, else as much as possible."""
    placements = []; added = 0.0
    for run_ in runs:
        if added >= skew - 1e-4:
            break
        side = choose_outboard(run_, paired_path)
        segL = run_len(run_); remaining = skew - added
        cap = int((segL - 2 * MARGIN + GAP_BUMPS) // (2 * H_TARGET + W_TOP + GAP_BUMPS))
        if cap < 1:
            continue
        need = int(math.ceil(remaining / (SLOPE * H_TARGET)))
        placed = None
        for k in range(min(cap, need), 0, -1):
            h = min(H_MAX, remaining / (SLOPE * k)) if k >= need else H_TARGET
            p = try_place(oracle, run_, side, k, h, skip)
            if p:
                placed = (p, k, h); break
        if placed:
            p, k, h = placed
            for (p0, p1, w, kind) in p["edges"]:
                oracle.add_bump(p0, p1, w, p["layer"], short_net)
            placements.append(p); added += k * SLOPE * h
    return placements, added

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
            lP, lN = plen(pathP), plen(pathN)
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
            for M in range(Mtarget, Nmin - 1, -1):
                h = add / (SLOPE * M)
                if h > H_MAX + 1e-9:
                    continue
                pl, left = distribute(oracle, runs, paired_path, skip, M, h)
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
        lA, lB = data["length"][net], data["length"][partner]
        skew = abs(lA - lB)
        short = net if lA < lB else partner
        base = data["netname"][short].split("/")[-1]
        if skew < SKEW_FLOOR:
            print("  %-10s skew %.0fum below floor, skip" % (base, skew * 1000))
            continue
        host_segs = by_net.get(short, [])
        if not host_segs:
            print("  %-10s you selected the LONGER net; select the shorter net %s instead" % (nm.split("/")[-1], base))
            continue
        skip = {net, partner}
        paired_path = order_path(data, partner)
        els = [dict(kind="seg", a=s["a"], b=s["b"], w=s["w"], layer=s["layer"], ref=s) for s in host_segs]
        runs = [r for r in build_runs(data, order_chain(els)) if run_len(r) >= W_TOP + 2 * MARGIN]
        placements, added = distribute_selection(oracle, runs, paired_path, skip, skew, short)
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

def run(board=None, apply=None, **overrides):
    _apply_overrides(overrides)
    if board is None:
        board = pcbnew.GetBoard()
    if apply is None:
        apply = APPLY
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
    print("\n%d meander(s), %d bumps; self-check min clearance = %.1f um, violations = %d"
          % (len(results), total, (minslack * 1000 if results else 0), viol))

    if apply and results and viol == 0:
        removed = set()
        for short_net, placements, skew, added in results:
            for p in placements:
                for m in p["members"]:
                    obj = m["obj"]
                    if id(obj) in removed:        # one host seg can back several bumps
                        continue
                    removed.add(id(obj))
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
                    board.Add(t)
        _refresh(board)
        print("APPLIED to the board -- review, then save (Ctrl+S).")
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

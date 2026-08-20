"""
Differential-pair fillet  --  runs INSIDE KiCad (pcbnew Python API, KiCad 9).

Rounds the corners of the SELECTED diff-pair track segments. The two traces are filleted as
CONCENTRIC arcs sharing one center: the INNER trace by RADIUS, the OUTER trace by
RADIUS + (diff-pair gap + trace width) = RADIUS + pitch. This keeps the gap constant through
the bend (equal phase / matched coupling). A single selected trace is filleted normally by RADIUS.

USAGE (PCB editor > Tools > Scripting Console):
    - Click-select the track segments of the pair (both nets) around the corner(s), then:
        exec(open(r'd:/Repos/XG_Mobile_Station/plugins/fillet_diffpair.py').read())
    - RADIUS is the inner radius; PITCH auto-measures (gap+width) unless you set it.
    - Override any knob WITHOUT editing this file: set a PARAMS dict first, e.g.
        PARAMS = dict(RADIUS=0.15, APPLY=False); exec(open(r'.../fillet_diffpair.py').read())
      (headless: run(board, apply=False, RADIUS=0.15)).
    - APPLY = False for a dry-run. Undo via git checkout of the board file.

Can also be dropped in KiCad's scripting/plugins folder (Tools > External Plugins).
"""
import math
import collections

try:
    import pcbnew
except ImportError:
    raise SystemExit("Run this from KiCad's PCB editor Scripting Console (pcbnew not importable).")

# ----------------------------------------------------------------------------- parameters (mm)
APPLY     = True
RADIUS    = 0.30          # inner fillet radius (the "specific amount"); outer = RADIUS + pitch
PITCH     = None          # center-to-center diff-pair spacing (gap+width); None = auto-measure
ANGLE_MIN = 3.0           # deg; skip near-straight vertices below this deflection
FIT       = 0.95          # a fillet may consume at most this fraction of an adjacent segment

# ----------------------------------------------------------------------------- vector helpers
def sub(a, b): return (a[0] - b[0], a[1] - b[1])
def add(a, b): return (a[0] + b[0], a[1] + b[1])
def mul(a, s): return (a[0] * s, a[1] * s)
def dot(a, b): return a[0] * b[0] + a[1] * b[1]
def cross(a, b): return a[0] * b[1] - a[1] * b[0]
def norm(a): return math.hypot(a[0], a[1])
def unit(a):
    n = norm(a)
    return (a[0] / n, a[1] / n) if n > 1e-12 else (0.0, 0.0)

def dist_seg(p, a, b):
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx - ax, by - ay; L2 = dx * dx + dy * dy
    if L2 < 1e-15:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))

def P2(v): return (pcbnew.ToMM(v.x), pcbnew.ToMM(v.y))
def V2(p): return pcbnew.VECTOR2I(pcbnew.FromMM(p[0]), pcbnew.FromMM(p[1]))

# ----------------------------------------------------------------------------- read selection
def read_selection(board):
    nets = collections.defaultdict(list)   # netcode -> [seg dict]
    for t in board.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA) or isinstance(t, pcbnew.PCB_ARC):
            continue
        if not t.IsSelected():
            continue
        nets[t.GetNetCode()].append(dict(a=P2(t.GetStart()), b=P2(t.GetEnd()),
                                         w=pcbnew.ToMM(t.GetWidth()), layer=t.GetLayer(),
                                         net=t.GetNetCode(), name=t.GetNetname(), obj=t))
    return nets

def order_net(segs):
    """Order the segments head-to-tail; return (vertices, width, layer, net, orig_objs) or None."""
    Q = lambda p: (round(p[0], 5), round(p[1], 5))
    adj = collections.defaultdict(list)
    for i, s in enumerate(segs):
        adj[Q(s["a"])].append((i, 0)); adj[Q(s["b"])].append((i, 1))
    ends = [k for k, v in adj.items() if len(v) == 1]
    start = ends[0] if ends else Q(segs[0]["a"])
    used = [False] * len(segs); verts = []; objs = []; key = start; first = True
    for _ in range(len(segs) + 1):
        nxt = next(((i, e) for (i, e) in adj[key] if not used[i]), None)
        if nxt is None:
            break
        i, e = nxt; used[i] = True; s = segs[i]
        a, b = (s["a"], s["b"]) if e == 0 else (s["b"], s["a"])
        if first:
            verts.append(a); first = False
        verts.append(b); objs.append(s); key = Q(b)
    if len(objs) != len(segs):        # selection isn't a single clean chain
        return None
    return verts, segs[0]["w"], segs[0]["layer"], segs[0]["net"], objs

def find_corners(verts):
    """Interior vertices with a real direction change; returns list of dicts."""
    cs = []
    for i in range(1, len(verts) - 1):
        prev, V, nxt = verts[i - 1], verts[i], verts[i + 1]
        e1 = unit(sub(prev, V)); e2 = unit(sub(nxt, V))
        c = max(-1.0, min(1.0, dot(e1, e2)))
        beta = math.acos(c)                       # interior angle at the corner
        defl = math.degrees(math.pi - beta)       # deflection from straight
        if defl < ANGLE_MIN or beta < 1e-3:
            continue
        indir = unit(sub(V, prev)); outdir = unit(sub(nxt, V))
        cs.append(dict(i=i, prev=prev, V=V, nxt=nxt, e1=e1, e2=e2, beta=beta,
                       turn=cross(indir, outdir), indir=indir))
    return cs

def fillet_center(c, r):
    """Inner fillet at corner c, radius r: return (center, P1, P2, mid, T)."""
    beta = c["beta"]; V = c["V"]; e1 = c["e1"]; e2 = c["e2"]
    T = r / math.tan(beta / 2.0)
    bis = unit(add(e1, e2))
    C = add(V, mul(bis, r / math.sin(beta / 2.0)))
    P1 = add(V, mul(e1, T)); P2 = add(V, mul(e2, T))
    mid = add(C, mul(unit(sub(V, C)), r))
    return C, P1, P2, mid, T

def outer_from_center(c, C):
    """Project shared center C onto the outer corner's two segment lines -> tangent pts + arc."""
    V = c["V"]; e1 = c["e1"]; e2 = c["e2"]
    P1 = add(V, mul(e1, dot(sub(C, V), e1)))
    P2 = add(V, mul(e2, dot(sub(C, V), e2)))
    r = (norm(sub(C, P1)) + norm(sub(C, P2))) / 2.0
    mid = add(C, mul(unit(sub(V, C)), r))
    T = max(norm(sub(V, P1)), norm(sub(V, P2)))
    return P1, P2, mid, T

def seg_len_ok(verts, i, T):
    """T must not eat more than FIT of either adjacent segment."""
    l1 = norm(sub(verts[i], verts[i - 1])); l2 = norm(sub(verts[i + 1], verts[i]))
    return T <= FIT * l1 and T <= FIT * l2

def rebuild(verts, fillets):
    """fillets: {i: (P1, P2, mid)}. Return list of ('seg',a,b) / ('arc',a,mid,b)."""
    out = []; cur = verts[0]
    for i in range(1, len(verts) - 1):
        if i in fillets:
            P1, P2, mid = fillets[i]
            if norm(sub(cur, P1)) > 1e-4:
                out.append(("seg", cur, P1))
            out.append(("arc", P1, mid, P2)); cur = P2
        else:
            out.append(("seg", cur, verts[i])); cur = verts[i]
    if norm(sub(cur, verts[-1])) > 1e-4:
        out.append(("seg", cur, verts[-1]))
    return out

def measure_pitch(A, B):
    ds = []
    for sa in A:
        if norm(sub(sa["b"], sa["a"])) < 0.3:
            continue
        mid = mul(add(sa["a"], sa["b"]), 0.5)
        ds.append(min(dist_seg(mid, sb["a"], sb["b"]) for sb in B))
    ds.sort()
    return ds[len(ds) // 2] if ds else None

# ----------------------------------------------------------------------------- overrides + main
def _apply_overrides(overrides):
    """Override any UPPERCASE knob at call time: run(..., RADIUS=0.15) or PARAMS=dict(RADIUS=0.15)."""
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
    nets = read_selection(board)
    if not nets:
        print("Nothing selected. Select the diff-pair track segments around the corner(s) and re-run.")
        return
    if len(nets) > 2:
        print("Selected %d nets; select just one pair (2 nets) or one trace." % len(nets)); return

    ordered = {}
    for code, segs in nets.items():
        r = order_net(segs)
        if r is None:
            print("Net %s: selection is not a single connected chain -- select a contiguous run." %
                  segs[0]["name"]); return
        ordered[code] = r

    plan = collections.defaultdict(dict)   # code -> {i: (P1,P2,mid)}
    done = 0; skipped = 0

    if len(nets) == 2:
        (ca, ra), (cb, rb) = list(ordered.items())
        vA, wA, lA, nA, oA = ra; vB, wB, lB, nB, oB = rb
        p = PITCH if PITCH else measure_pitch(nets[ca], nets[cb])
        if not p:
            print("Could not measure diff-pair pitch; set PITCH manually."); return
        cornersA = find_corners(vA); cornersB = find_corners(vB)
        print("pair %s / %s  pitch=%.4f mm  inner R=%.3f outer R=%.3f  cornersA=%d cornersB=%d"
              % (ra[3] and nets[ca][0]["name"], nets[cb][0]["name"], p, RADIUS, RADIUS + p,
                 len(cornersA), len(cornersB)))
        for cA in cornersA:
            cB = min(cornersB, key=lambda c: norm(sub(c["V"], cA["V"])), default=None)
            if cB is None or norm(sub(cB["V"], cA["V"])) > 2.5 * p or cA["turn"] * cB["turn"] <= 0:
                skipped += 1; continue
            # inner = the vertex on the concave (turn-center) side
            indir = cA["indir"]; ncen = mul((-indir[1], indir[0]), 1.0 if cA["turn"] > 0 else -1.0)
            a_inner = dot(sub(cA["V"], cB["V"]), ncen) > 0
            cin, cout = (cA, cB) if a_inner else (cB, cA)
            vin, vout = (vA, vB) if a_inner else (vB, vA)
            code_in, code_out = (ca, cb) if a_inner else (cb, ca)
            C, P1i, P2i, midi, Ti = fillet_center(cin, RADIUS)
            P1o, P2o, mido, To = outer_from_center(cout, C)
            if not (seg_len_ok(vin, cin["i"], Ti) and seg_len_ok(vout, cout["i"], To)):
                skipped += 1; continue
            plan[code_in][cin["i"]] = (P1i, P2i, midi)
            plan[code_out][cout["i"]] = (P1o, P2o, mido)
            done += 1
    else:   # single trace: normal fillet by RADIUS
        code, (v, w, l, n, o) = list(ordered.items())[0]
        print("single trace %s  R=%.3f  corners=%d" % (nets[code][0]["name"], RADIUS, len(find_corners(v))))
        for c in find_corners(v):
            C, P1, P2, mid, T = fillet_center(c, RADIUS)
            if not seg_len_ok(v, c["i"], T):
                skipped += 1; continue
            plan[code][c["i"]] = (P1, P2, mid); done += 1

    print("fillets: %d applied, %d skipped (didn't fit / unmatched)" % (done, skipped))
    if not apply or done == 0:
        print("DRY RUN (or nothing to do): set APPLY = True to modify the board." if not apply else "")
        return

    for code, (verts, w, layer, net, objs) in ordered.items():
        if code not in plan:
            continue
        edges = rebuild(verts, plan[code])
        for s in objs:
            try: s["obj"].ClearSelected()        # GUI: don't leave a removed item selected
            except Exception: pass
            board.Remove(s["obj"])
        for e in edges:
            if e[0] == "seg":
                t = pcbnew.PCB_TRACK(board)
                t.SetStart(V2(e[1])); t.SetEnd(V2(e[2]))
            else:
                t = pcbnew.PCB_ARC(board)
                t.SetStart(V2(e[1])); t.SetMid(V2(e[2])); t.SetEnd(V2(e[3]))
            t.SetWidth(pcbnew.FromMM(w)); t.SetLayer(layer); t.SetNetCode(net)
            board.Add(t)
    _refresh(board)
    print("APPLIED -- review, run DRC, then save (Ctrl+S).")


try:
    class FilletDiffPairPlugin(pcbnew.ActionPlugin):
        def defaults(self):
            self.name = "Fillet differential pair"
            self.category = "Modify PCB"
            self.description = "Fillet selected diff-pair corners (inner R, outer R+pitch, concentric)."
            self.show_toolbar_button = True

        def Run(self):
            run(pcbnew.GetBoard(), apply=True)
except Exception:
    pass

if __name__ == "__main__":
    run(**globals().get("PARAMS", {}))
else:
    try:
        FilletDiffPairPlugin().register()
    except Exception:
        pass

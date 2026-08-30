"""
Differential-pair fillet  --  runs INSIDE KiCad (pcbnew Python API, KiCad 9).

Rounds the SHARP corners of SELECTED tracks. Handles:
  * MULTIPLE diff pairs at once -- selected nets are grouped into pairs by name (P/N, +/-),
    each pair filleted independently; leftover single nets are filleted on their own.
  * Pairs that are already length-tuned (meander ARCS in the run) -- arcs are kept as-is and only
    the straight seg-to-seg routing corners are rounded.
  * Vias / multi-layer routes -- each net is split into per-LAYER runs; a via is a run end, so the
    FANOUT dog-legs into vias get filleted too.
For a matched pair corner the two traces are filleted as CONCENTRIC arcs sharing one center (inner
by RADIUS, outer by RADIUS + pitch) so the gap stays constant. Where the pair diverges (e.g. a
fanout that breaks out to separate vias) each corner is filleted independently by RADIUS. Fillets
auto-shrink to fit short segments (so tight fanout stubs still round).

USAGE (PCB editor > Tools > Scripting Console):
    - Select the track segments to round (one or many pairs; include the arcs/vias in the run,
      it's fine), then:
        exec(open(r'd:/Repos/XG_Mobile_Station/plugins/fillet_diffpair.py').read())
    - RADIUS is the inner radius; PITCH auto-measures (gap+width) per pair unless you set it.
    - Override any knob WITHOUT editing this file: set a PARAMS dict first, e.g.
        PARAMS = dict(RADIUS=0.2, APPLY=False); exec(open(r'.../fillet_diffpair.py').read())
      (headless: run(board, apply=False, RADIUS=0.2)).
    - APPLY = False for a dry-run. Undo via Edit > Undo or git checkout of the board file.

Can also be dropped in KiCad's scripting/plugins folder (Tools > External Plugins).
"""
import math
import collections

try:
    import pcbnew
except ImportError:
    raise SystemExit("Run this from KiCad's PCB editor Scripting Console (pcbnew not importable).")

# ----------------------------------------------------------------------------- parameters (mm)
APPLY      = True
RADIUS     = 0.30         # inner fillet radius; outer diff-pair arc = RADIUS + pitch
PITCH      = None         # center-to-center diff-pair spacing (gap+width); None = auto-measure per pair
ANGLE_MIN  = 3.0          # deg; skip near-straight vertices below this deflection
FIT        = 0.90         # a fillet may consume at most this fraction of an adjacent segment
MIN_RADIUS = 0.05         # don't place a fillet whose (shrunk) radius falls below this
PAIR_DIST  = 2.5          # match a P corner to an N corner within this many pitches (same turn)

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

# ----------------------------------------------------------------------------- fillet geometry
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

def adj_len(c):
    """Shorter of the two segments meeting at corner c."""
    return min(norm(sub(c["V"], c["prev"])), norm(sub(c["nxt"], c["V"])))

# ----------------------------------------------------------------------------- read + chain build
def read_selection(board):
    """Selected tracks (segments + arcs, not vias) grouped by netcode; each carries kind/mid/obj."""
    edges = collections.defaultdict(list)
    for t in board.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA) or not t.IsSelected():
            continue
        if isinstance(t, pcbnew.PCB_ARC):
            edges[t.GetNetCode()].append(dict(kind="arc", a=P2(t.GetStart()), b=P2(t.GetEnd()),
                                              mid=P2(t.GetMid()), w=pcbnew.ToMM(t.GetWidth()),
                                              layer=t.GetLayer(), net=t.GetNetCode(),
                                              name=t.GetNetname(), obj=t))
        else:
            edges[t.GetNetCode()].append(dict(kind="seg", a=P2(t.GetStart()), b=P2(t.GetEnd()),
                                              mid=None, w=pcbnew.ToMM(t.GetWidth()),
                                              layer=t.GetLayer(), net=t.GetNetCode(),
                                              name=t.GetNetname(), obj=t))
    return edges

def build_chains(elist):
    """Order the net's selected edges head-to-tail, PER LAYER; return a chain dict per run.
    Arcs stay in the chain (kept as-is); vias aren't here, so a via is just a run end."""
    Q = lambda p: (round(p[0], 4), round(p[1], 4))
    chains = []
    bylayer = collections.defaultdict(list)
    for e in elist:
        bylayer[e["layer"]].append(e)
    for layer, edges in bylayer.items():
        adj = collections.defaultdict(list)
        for i, e in enumerate(edges):
            adj[Q(e["a"])].append(i); adj[Q(e["b"])].append(i)
        used = [False] * len(edges)
        def walk(node):
            verts = []; kinds = []; mids = []; objs = []; first = True
            while True:
                nxt = next((i for i in adj[node] if not used[i]), None)
                if nxt is None:
                    break
                e = edges[nxt]; used[nxt] = True
                a, b = (e["a"], e["b"]) if Q(e["a"]) == node else (e["b"], e["a"])
                if first:
                    verts.append(a); first = False
                verts.append(b); kinds.append(e["kind"]); mids.append(e["mid"]); objs.append(e)
                node = Q(b)
            return verts, kinds, mids, objs
        order = [n for n, l in adj.items() if len(l) == 1] + [Q(e["a"]) for e in edges]
        for node in order:
            if any(not used[i] for i in adj[node]):
                v, k, m, o = walk(node)
                if o:
                    chains.append(dict(layer=layer, verts=v, kinds=k, mids=m, objs=o,
                                       net=o[0]["net"], name=o[0]["name"], fillets={}))
    return chains

def find_corners(chain):
    """Interior vertices where two STRAIGHT segments meet with a real direction change."""
    verts = chain["verts"]; kinds = chain["kinds"]; cs = []
    for i in range(1, len(verts) - 1):
        if kinds[i - 1] != "seg" or kinds[i] != "seg":   # leave arc (tuned/rounded) junctions alone
            continue
        prev, V, nxt = verts[i - 1], verts[i], verts[i + 1]
        e1 = unit(sub(prev, V)); e2 = unit(sub(nxt, V))
        c = max(-1.0, min(1.0, dot(e1, e2)))
        beta = math.acos(c)
        if math.degrees(math.pi - beta) < ANGLE_MIN or beta < 1e-3:
            continue
        indir = unit(sub(V, prev)); outdir = unit(sub(nxt, V))
        cs.append(dict(i=i, prev=prev, V=V, nxt=nxt, e1=e1, e2=e2, beta=beta,
                       turn=cross(indir, outdir), indir=indir,
                       bis=unit(add(e1, e2)), chain=chain))   # bis -> concave side, traversal-invariant
    return cs

def measure_pitch(A, B):
    """Median center-to-center spacing between two nets' straight segments."""
    ds = []
    for sa in A:
        if sa["kind"] != "seg" or norm(sub(sa["b"], sa["a"])) < 0.3:
            continue
        mid = mul(add(sa["a"], sa["b"]), 0.5)
        cand = [dist_seg(mid, sb["a"], sb["b"]) for sb in B if sb["kind"] == "seg"]
        if cand:
            ds.append(min(cand))
    ds.sort()
    return ds[len(ds) // 2] if ds else None

def diff_partner(name):
    """Partner net name for a diff-pair member, or None."""
    base = name.rsplit("/", 1)[-1]
    pre = name[:len(name) - len(base)]
    for p, n in (("P", "N"), ("+", "-"), ("_P", "_N")):
        if base.endswith(p):
            return pre + base[:-len(p)] + n
        if base.endswith(n):
            return pre + base[:-len(n)] + p
    return None

# ----------------------------------------------------------------------------- fillet planners
def plan_single(c):
    """Fillet one corner by RADIUS, shrunk to fit its shorter neighbour. Returns True if planned."""
    Tmax = FIT * adj_len(c)
    r = min(RADIUS, Tmax * math.tan(c["beta"] / 2.0))
    if r < MIN_RADIUS:
        return False
    _, P1, P2, mid, _ = fillet_center(c, r)
    c["chain"]["fillets"][c["i"]] = (P1, P2, mid)
    return True

def plan_concentric(cA, cB):
    """Concentric fillet for a matched pair corner (inner RADIUS, outer RADIUS+pitch); shrink to fit."""
    # inner = the corner whose vertex sits on its own concave side relative to the partner (bis is
    # traversal-invariant, unlike turn/indir which flip with chain ordering direction)
    a_inner = dot(sub(cA["V"], cB["V"]), cA["bis"]) > 0
    cin, cout = (cA, cB) if a_inner else (cB, cA)
    r = RADIUS
    for _ in range(24):
        C, P1i, P2i, midi, Ti = fillet_center(cin, r)
        P1o, P2o, mido, To = outer_from_center(cout, C)
        if Ti <= FIT * adj_len(cin) and To <= FIT * adj_len(cout):
            cin["chain"]["fillets"][cin["i"]] = (P1i, P2i, midi)
            cout["chain"]["fillets"][cout["i"]] = (P1o, P2o, mido)
            return True
        r *= 0.85
        if r < MIN_RADIUS:
            return False
    return False

# ----------------------------------------------------------------------------- overrides + refresh
def _apply_overrides(overrides):
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
    try: board.BuildConnectivity()      # stop ratsnest engine dereferencing removed items -> crash
    except Exception: pass
    try: pcbnew.Refresh()
    except Exception: pass

def _move_end(obj, frm, to):
    """Move whichever end of the track object is at point `frm` (mm) to `to` (mm)."""
    if norm(sub(P2(obj.GetStart()), frm)) <= norm(sub(P2(obj.GetEnd()), frm)):
        obj.SetStart(V2(to))
    else:
        obj.SetEnd(V2(to))

# ----------------------------------------------------------------------------- main
def run(board=None, apply=None, **overrides):
    _apply_overrides(overrides)
    if board is None:
        board = pcbnew.GetBoard()
    if apply is None:
        apply = APPLY
    edges = read_selection(board)
    if not edges:
        print("Nothing selected. Select the track segments to fillet and re-run.")
        return
    chains_by_net = {net: build_chains(elist) for net, elist in edges.items()}
    name_by_net = {net: elist[0]["name"] for net, elist in edges.items()}
    net_by_name = {nm.rsplit("/", 1)[-1]: net for net, nm in name_by_net.items()}

    pairs = []; singles = []; seen = set()
    for net, nm in name_by_net.items():
        if net in seen:
            continue
        pbase = diff_partner(nm)
        pbase = pbase.rsplit("/", 1)[-1] if pbase else None
        pnet = net_by_name.get(pbase)
        if pnet is not None and pnet != net and pnet not in seen:
            pairs.append((net, pnet)); seen.add(net); seen.add(pnet)
        else:
            singles.append(net); seen.add(net)

    done = 0; skipped = 0
    print("selection: %d net(s) -> %d pair(s) + %d single(s)" % (len(edges), len(pairs), len(singles)))

    for a, b in pairs:
        p = PITCH if PITCH else measure_pitch(edges[a], edges[b])
        na = name_by_net[a].rsplit("/", 1)[-1]; nb = name_by_net[b].rsplit("/", 1)[-1]
        if not p:
            print("  pair %s/%s: no pitch (traces not parallel?) -- filleting each independently" % (na, nb))
            p = 0.0
        cornersA = [c for ch in chains_by_net[a] for c in find_corners(ch)]
        cornersB = [c for ch in chains_by_net[b] for c in find_corners(ch)]
        usedB = [False] * len(cornersB)
        pd = 0
        for cA in cornersA:
            best = -1; bestd = 1e9
            for j, cB in enumerate(cornersB):
                if usedB[j] or cB["chain"]["layer"] != cA["chain"]["layer"]:
                    continue
                d = norm(sub(cA["V"], cB["V"]))
                if d < bestd and (p and d <= PAIR_DIST * p) and dot(cA["bis"], cB["bis"]) > 0.5:
                    bestd = d; best = j
            if best >= 0:
                cB = cornersB[best]; usedB[best] = True
                if plan_concentric(cA, cB):
                    done += 2; pd += 1
                else:                                   # too tight for a proper concentric bend -> skip
                    skipped += 2                        #   (never fillet a pair bend with mismatched radii)
            else:
                if plan_single(cA): done += 1
                else: skipped += 1
        for j, cB in enumerate(cornersB):
            if not usedB[j]:
                if plan_single(cB): done += 1
                else: skipped += 1
        print("  pair %s/%s pitch=%.4f cornersA=%d cornersB=%d concentric=%d"
              % (na, nb, p, len(cornersA), len(cornersB), pd))

    for net in singles:
        cs = [c for ch in chains_by_net[net] for c in find_corners(ch)]
        for c in cs:
            ok = plan_single(c); done += ok; skipped += (not ok)
        print("  single %s corners=%d" % (name_by_net[net].rsplit("/", 1)[-1], len(cs)))

    print("total: %d corner(s) rounded, %d skipped (too tight)" % (done, skipped))
    if not apply or done == 0:
        print("DRY RUN: set APPLY = True (or run(apply=True)) to modify the board." if not apply
              else "Nothing to fillet.")
        return

    added = 0
    for chains in chains_by_net.values():
        for ch in chains:
            if not ch["fillets"]:
                continue
            verts = ch["verts"]; objs = ch["objs"]
            for i, (P1, P2, mid) in ch["fillets"].items():
                eb = objs[i - 1]["obj"]; ea = objs[i]["obj"]
                w = eb.GetWidth()
                try: eb.ClearSelected(); ea.ClearSelected()
                except Exception: pass
                _move_end(eb, verts[i], P1)
                _move_end(ea, verts[i], P2)
                t = pcbnew.PCB_ARC(board)
                t.SetStart(V2(P1)); t.SetMid(V2(mid)); t.SetEnd(V2(P2))
                t.SetWidth(w); t.SetLayer(ch["layer"]); t.SetNetCode(ch["net"])
                board.Add(t); added += 1
    _refresh(board)
    print("APPLIED %d fillet arc(s) -- review, run DRC, then save (Ctrl+S)." % added)


try:
    class FilletDiffPairPlugin(pcbnew.ActionPlugin):
        def defaults(self):
            self.name = "Fillet differential pair(s)"
            self.category = "Modify PCB"
            self.description = "Fillet selected diff-pair / fanout corners (concentric where matched)."
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

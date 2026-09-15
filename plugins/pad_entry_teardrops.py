"""
Pad-entry teardrops for KiCad 9, with differential-pair gap preservation.

The script replaces each SELECTED straight track that ends in a same-net pad with a
short, stepped-width taper. For a differential pair, select both pad-entry tracks.
As each track widens, its centerline moves away from its partner by half the added
width. The inboard copper edge therefore stays where the original trace edge was,
so the pair gap cannot shrink as the traces fan apart into the pads.

Usage in PCB Editor > Tools > Scripting Console:
    PARAMS = dict(APPLY=False)  # optional dry run
    exec(open(r'd:/Repos/XG_Mobile_Station/plugins/pad_entry_teardrops.py').read())

Select only the final straight segment(s) entering the pads. Select both members of
a differential pair for symmetric tapers. Review the result and run DRC before save.
The operation is undoable with Edit > Undo while the board remains open.
"""
import collections
import math

try:
    import pcbnew
except ImportError:
    raise SystemExit("Run this from KiCad's PCB editor Scripting Console (pcbnew not importable).")


# Parameters (mm unless noted)
APPLY = True
USE_SELECTION = True
LENGTH = 0.80            # distance over which the trace widens and moves outward
STEPS = 8                # width/offset stages; higher values make a smoother transition
MAX_FACTOR = 2.0         # maximum width relative to the entering trace
PAD_FILL = 0.82          # maximum fraction of pad half-span used in the outboard direction
MIN_WIDTH_GAIN = 0.015   # skip tapers whose useful width increase is smaller than this
MIN_SEGMENT = 0.025      # do not create extremely short stage segments
PAIR_ANGLE = 12.0        # maximum direction mismatch for paired pad-entry tracks (degrees)
PAIR_DISTANCE = 2.0      # maximum pad-entry separation considered a local pair
PAIR_TOL = 0.00001       # numerical tolerance only; reject any measurable pair-gap reduction
END_TOL = 0.03           # fallback tolerance when testing whether an endpoint lies in a pad
BOUNDARY_STEPS = 40      # binary-search iterations for the pad-outline crossing


def add(a, b): return (a[0] + b[0], a[1] + b[1])
def sub(a, b): return (a[0] - b[0], a[1] - b[1])
def mul(a, scale): return (a[0] * scale, a[1] * scale)
def dot(a, b): return a[0] * b[0] + a[1] * b[1]
def norm(a): return math.hypot(a[0], a[1])


def unit(a):
    length = norm(a)
    return (a[0] / length, a[1] / length) if length > 1e-12 else (0.0, 0.0)


def p2(vector):
    return (pcbnew.ToMM(vector.x), pcbnew.ToMM(vector.y))


def v2(point):
    return pcbnew.VECTOR2I(pcbnew.FromMM(point[0]), pcbnew.FromMM(point[1]))


def smoothstep(value):
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def segment_distance(first_start, first_end, second_start, second_end):
    """Minimum centerline distance between two finite line segments."""
    def point_distance(point, start, end):
        delta = sub(end, start)
        length2 = dot(delta, delta)
        if length2 < 1e-15:
            return norm(sub(point, start))
        fraction = max(0.0, min(1.0, dot(sub(point, start), delta) / length2))
        return norm(sub(point, add(start, mul(delta, fraction))))

    def cross(first, second):
        return first[0] * second[1] - first[1] * second[0]

    first_delta = sub(first_end, first_start)
    second_delta = sub(second_end, second_start)
    denominator = cross(first_delta, second_delta)
    if abs(denominator) > 1e-12:
        between = sub(second_start, first_start)
        first_fraction = cross(between, second_delta) / denominator
        second_fraction = cross(between, first_delta) / denominator
        if 0.0 <= first_fraction <= 1.0 and 0.0 <= second_fraction <= 1.0:
            return 0.0
    return min(point_distance(first_start, second_start, second_end),
               point_distance(first_end, second_start, second_end),
               point_distance(second_start, first_start, first_end),
               point_distance(second_end, first_start, first_end))


def diff_partner(name):
    """Return the complementary net name using common KiCad P/N conventions."""
    if not name:
        return None
    for first, second in (("_P", "_N"), ("_N", "_P")):
        if name.endswith(first):
            return name[:-2] + second
    for first, second in (("P", "N"), ("N", "P"), ("+", "-"), ("-", "+")):
        if name.endswith(first):
            return name[:-1] + second
    return None


def _pad_half_span(pad, direction):
    """Projected half-span of a rotated pad bounding rectangle along direction."""
    size = pad.GetSize()
    half_x, half_y = pcbnew.ToMM(size.x) / 2.0, pcbnew.ToMM(size.y) / 2.0
    angle = math.radians(pad.GetOrientationDegrees())
    local_x = (math.cos(angle), math.sin(angle))
    local_y = (-math.sin(angle), math.cos(angle))
    return abs(dot(direction, local_x)) * half_x + abs(dot(direction, local_y)) * half_y


def _endpoint_pad(pads, track, endpoint):
    """Find the same-net, same-layer pad containing a track endpoint."""
    point = v2(endpoint)
    candidates = []
    for pad in pads:
        if not pad.IsOnLayer(track.GetLayer()):
            continue
        center = p2(pad.GetPosition())
        try:
            hit = pad.HitTest(point)
        except Exception:
            size = pad.GetSize()
            radius = 0.5 * math.hypot(pcbnew.ToMM(size.x), pcbnew.ToMM(size.y)) + END_TOL
            hit = norm(sub(endpoint, center)) <= radius
        if hit:
            candidates.append((norm(sub(endpoint, center)), pad))
    return min(candidates, key=lambda item: item[0])[1] if candidates else None


def _pad_hit(pad, point):
    """True when a centerline point lies within the actual pad shape."""
    try:
        return pad.HitTest(v2(point))
    except Exception:
        center = p2(pad.GetPosition())
        size = pad.GetSize()
        radius = 0.5 * math.hypot(pcbnew.ToMM(size.x), pcbnew.ToMM(size.y)) + END_TOL
        return norm(sub(point, center)) <= radius


def _pad_entry(pad, outside, inside):
    """First centerline point entering the pad, found from an outside-to-inside segment."""
    if _pad_hit(pad, outside) or not _pad_hit(pad, inside):
        return None
    low, high = 0.0, 1.0
    delta = sub(inside, outside)
    for _ in range(max(8, int(BOUNDARY_STEPS))):
        middle = 0.5 * (low + high)
        point = add(outside, mul(delta, middle))
        if _pad_hit(pad, point):
            high = middle
        else:
            low = middle
    # Step a few internal units into the pad so integer rounding cannot leave
    # the bridge endpoint microscopically outside the pad outline.
    fraction = min(1.0, high + 2.0 / max(norm(delta) * 1e6, 1.0))
    return add(outside, mul(delta, fraction))


def _candidate(board, track, endpoint, other_end, pad):
    """Describe a taper directed from the routed trace toward its pad."""
    entry = _pad_entry(pad, other_end, endpoint)
    if entry is None:
        return None
    direction = unit(sub(entry, other_end))
    length = norm(sub(entry, other_end))
    if length < max(2.0 * MIN_SEGMENT, 0.05):
        return None
    return dict(
        obj=track, net=track.GetNetCode(), name=track.GetNetname(), layer=track.GetLayer(),
        pad=pad, end=entry, pad_end=endpoint, far=other_end, direction=direction,
        normal=(-direction[1], direction[0]), length=length,
        width=pcbnew.ToMM(track.GetWidth()), partner=None, side=None,
    )


def read_candidates(board):
    """Read selected straight track endpoints that terminate in same-net pads."""
    result = []
    pads_by_net = collections.defaultdict(list)
    for footprint in board.GetFootprints():
        for pad in footprint.Pads():
            pads_by_net[pad.GetNetCode()].append(pad)
    for track in board.GetTracks():
        if isinstance(track, (pcbnew.PCB_VIA, pcbnew.PCB_ARC)):
            continue
        if USE_SELECTION and not track.IsSelected():
            continue
        start, end = p2(track.GetStart()), p2(track.GetEnd())
        net_pads = pads_by_net.get(track.GetNetCode(), ())
        start_pad = _endpoint_pad(net_pads, track, start)
        end_pad = _endpoint_pad(net_pads, track, end)
        if start_pad:
            candidate = _candidate(board, track, start, end, start_pad)
            if candidate:
                result.append(candidate)
        if end_pad and end_pad is not start_pad:
            candidate = _candidate(board, track, end, start, end_pad)
            if candidate:
                result.append(candidate)
    return result


def pair_candidates(candidates):
    """Match local P/N pad entries and choose each member's outboard direction."""
    by_name = collections.defaultdict(list)
    for candidate in candidates:
        by_name[candidate["name"]].append(candidate)
    paired = 0
    for candidate in candidates:
        if candidate["partner"] is not None:
            continue
        partner_name = diff_partner(candidate["name"])
        choices = []
        for other in by_name.get(partner_name, ()):
            if other["partner"] is not None or other["layer"] != candidate["layer"]:
                continue
            alignment = abs(dot(candidate["direction"], other["direction"]))
            if alignment < math.cos(math.radians(PAIR_ANGLE)):
                continue
            distance = norm(sub(candidate["end"], other["end"]))
            if distance <= PAIR_DISTANCE:
                choices.append((distance, other))
        if not choices:
            continue
        other = min(choices, key=lambda item: item[0])[1]
        candidate["partner"] = other
        other["partner"] = candidate
        toward_other = sub(other["end"], candidate["end"])
        candidate["side"] = -1.0 if dot(toward_other, candidate["normal"]) > 0.0 else 1.0
        toward_candidate = sub(candidate["end"], other["end"])
        other["side"] = -1.0 if dot(toward_candidate, other["normal"]) > 0.0 else 1.0
        paired += 1
    return paired


def _single_side(candidate):
    """For a single trace, widen toward the roomiest side of its own pad."""
    center = p2(candidate["pad"].GetPosition())
    projection = dot(sub(candidate["end"], center), candidate["normal"])
    return -1.0 if projection > 0.0 else 1.0


def _target_width(candidate):
    """Largest width that keeps the shifted endpoint conservatively inside its pad."""
    side = candidate["side"] if candidate["side"] is not None else _single_side(candidate)
    outboard = mul(candidate["normal"], side)
    center = p2(candidate["pad"].GetPosition())
    endpoint_projection = dot(sub(candidate["end"], center), outboard)
    room = PAD_FILL * _pad_half_span(candidate["pad"], outboard) - endpoint_projection
    pad_limited = room + candidate["width"] / 2.0
    return max(candidate["width"], min(MAX_FACTOR * candidate["width"], pad_limited))


def plan_candidate(candidate):
    """Build connected, staged track geometry while holding the inboard edge fixed."""
    width0 = candidate["width"]
    width1 = _target_width(candidate)
    if width1 - width0 < MIN_WIDTH_GAIN:
        return None
    side = candidate["side"] if candidate["side"] is not None else _single_side(candidate)
    outboard = mul(candidate["normal"], side)
    taper_length = min(LENGTH, candidate["length"] * 0.80)
    stages = max(2, min(int(STEPS), int(taper_length / MIN_SEGMENT)))
    start = sub(candidate["end"], mul(candidate["direction"], taper_length))

    nodes = []
    widths = []
    for index in range(stages + 1):
        fraction = index / float(stages)
        shaped = smoothstep(fraction)
        width = width0 + (width1 - width0) * shaped
        offset = (width - width0) / 2.0
        point = add(add(start, mul(candidate["direction"], taper_length * fraction)),
                    mul(outboard, offset))
        nodes.append(point)
        widths.append(width)

    # Make the last stage full width. Each widening starts at a shared node whose
    # center has already moved outward, so its inboard edge never crosses inward.
    if stages >= 2:
        widths[-2] = width1
        prior_fraction = (stages - 1) / float(stages)
        nodes[-2] = add(add(start, mul(candidate["direction"], taper_length * prior_fraction)),
                        mul(outboard, (width1 - width0) / 2.0))
    pieces = []
    for index in range(stages):
        segment_width = widths[index]
        if index == stages - 1:
            segment_width = width1
        pieces.append((nodes[index], nodes[index + 1], segment_width))
    bridge = (nodes[-1], candidate["pad_end"], width1)
    return dict(candidate=candidate, start=start, end=nodes[-1], width0=width0,
                width1=width1, pieces=pieces, bridge=bridge, side=side, length=taper_length)


def verify_pair_gaps(plans):
    """Reject both members unless their staged copper preserves the original edge gap."""
    by_candidate = {id(plan["candidate"]): plan for plan in plans}
    rejected = set()
    checked = set()
    for plan in plans:
        candidate = plan["candidate"]
        partner = candidate["partner"]
        if partner is None or id(candidate) in checked:
            continue
        other = by_candidate.get(id(partner))
        if other is None:
            continue
        checked.add(id(candidate))
        checked.add(id(partner))
        original_gap = (segment_distance(candidate["far"], candidate["end"],
                                         partner["far"], partner["end"])
                        - candidate["width"] / 2.0 - partner["width"] / 2.0)
        # The bridge runs beneath its own pad copper from the outline crossing to
        # the original endpoint. Pair-route clearance is meaningful only through
        # the pad boundary; inside it, the fixed pad geometry controls clearance.
        first_geometry = [(candidate["far"], plan["start"], candidate["width"])] + plan["pieces"]
        second_geometry = [(partner["far"], other["start"], partner["width"])] + other["pieces"]
        tapered_gap = min(segment_distance(a0, a1, b0, b1) - aw / 2.0 - bw / 2.0
                          for a0, a1, aw in first_geometry
                          for b0, b1, bw in second_geometry)
        label = candidate["name"].rsplit("/", 1)[-1]
        print("  pair-gap %-16s original=%.4f mm tapered=%.4f mm"
              % (label, original_gap, tapered_gap))
        if tapered_gap + PAIR_TOL < original_gap:
            print("    REJECTED: taper would reduce the differential-pair edge gap")
            rejected.add(id(candidate))
            rejected.add(id(partner))
    return [plan for plan in plans if id(plan["candidate"]) not in rejected]


def _apply_overrides(overrides):
    if not overrides:
        return
    globals_ = globals()
    bad = []
    for name, value in overrides.items():
        if name.isupper() and name in globals_:
            globals_[name] = value
        else:
            bad.append(name)
    if bad:
        valid = ", ".join(sorted(name for name in globals_ if name.isupper() and not name.startswith("_")))
        print("[params] ignored: %s\n[params] valid knobs: %s" % (", ".join(sorted(bad)), valid))


def _refresh(board):
    try:
        board.BuildConnectivity()
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

    candidates = read_candidates(board)
    if not candidates:
        scope = "selected" if USE_SELECTION else "board"
        print("No straight %s track endpoint enters a same-net pad." % scope)
        return 0
    pair_count = pair_candidates(candidates)
    plans = [plan for plan in (plan_candidate(candidate) for candidate in candidates) if plan]
    plans = verify_pair_gaps(plans)
    print("pad entries: %d candidate(s), %d differential pair(s), %d taper(s) planned"
          % (len(candidates), pair_count, len(plans)))
    for plan in plans:
        candidate = plan["candidate"]
        label = candidate["name"].rsplit("/", 1)[-1]
        kind = "paired/outboard" if candidate["partner"] is not None else "single"
        print("  %-20s %s  %.3f -> %.3f mm over %.3f mm (%d stages)"
              % (label, kind, plan["width0"], plan["width1"], plan["length"], len(plan["pieces"])))

    if not apply or not plans:
        print("DRY RUN: set APPLY=True after reviewing the plan." if not apply else "Nothing changed.")
        return len(plans)

    # A track with pads at both ends would produce two plans. Avoid partially
    # modifying such an object unless its two tapers leave a straight middle.
    by_object = collections.defaultdict(list)
    for plan in plans:
        by_object[id(plan["candidate"]["obj"])].append(plan)
    safe_plans = []
    for group in by_object.values():
        if len(group) == 1:
            safe_plans.extend(group)
        else:
            print("  skip %s: both pad ends selected; split the trace first"
                  % group[0]["candidate"]["name"].rsplit("/", 1)[-1])

    made = 0
    for plan in safe_plans:
        candidate = plan["candidate"]
        original = candidate["obj"]
        far = candidate["far"]
        try:
            original.ClearSelected()
        except Exception:
            pass
        if norm(sub(p2(original.GetStart()), far)) <= norm(sub(p2(original.GetEnd()), far)):
            original.SetEnd(v2(plan["start"]))
        else:
            original.SetStart(v2(plan["start"]))
        original.SetWidth(pcbnew.FromMM(plan["width0"]))
        for start, end, width in plan["pieces"]:
            track = pcbnew.PCB_TRACK(board)
            track.SetStart(v2(start))
            track.SetEnd(v2(end))
            track.SetWidth(pcbnew.FromMM(width))
            track.SetLayer(candidate["layer"])
            track.SetNetCode(candidate["net"])
            board.Add(track)
            made += 1
        start, end, width = plan["bridge"]
        if norm(sub(end, start)) >= 1e-6:
            bridge = pcbnew.PCB_TRACK(board)
            bridge.SetStart(v2(start))
            bridge.SetEnd(v2(end))
            bridge.SetWidth(pcbnew.FromMM(width))
            bridge.SetLayer(candidate["layer"])
            bridge.SetNetCode(candidate["net"])
            board.Add(bridge)
            made += 1
    _refresh(board)
    print("APPLIED: %d staged track segment(s). Review pad overlap and run DRC before save." % made)
    return made


try:
    class PadEntryTeardropPlugin(pcbnew.ActionPlugin):
        def defaults(self):
            self.name = "Pad-entry teardrops (diff-pair safe)"
            self.category = "Modify PCB"
            self.description = "Widen selected pad entries while preserving differential-pair gap."
            self.show_toolbar_button = True

        def Run(self):
            run(pcbnew.GetBoard(), apply=True)
except Exception:
    pass


if __name__ == "__main__":
    run(**globals().get("PARAMS", {}))
else:
    try:
        PadEntryTeardropPlugin().register()
    except Exception:
        pass
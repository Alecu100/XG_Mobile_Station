"""
Pad-entry tapers for KiCad 9, with differential-pair gap preservation.

The script replaces each SELECTED straight track or arc that ends in a same-net pad
with a continuous filled copper outline. For a differential pair, select both entries.
The outline follows a tangent-matched curve and widens into the actual pad shape.
Both members are rejected if their polygon edges reduce the original pair gap.
Paired flares are shifted outward to preserve the original facing copper boundary
at every transverse section, including the wider clearance next to the pads.
CONSTANT_WIDTH is a separate, staged-track mode for redriver reference-loss routing.

Usage in PCB Editor > Tools > Scripting Console:
    PARAMS = dict(APPLY=False)  # optional dry run
    exec(open(r'd:/Repos/XG_Mobile_Station/plugins/pad_entry_teardrops.py').read())

Retimer HSO capacitor entries C38-C53 use LENGTH=0.45. Curved HSI entries require
more room (LENGTH=0.55); any folded outline and its partner are left unchanged.
These lengths are board-specific. Refill zones and run DRC after applying.

Progression and pad-size scaling (normal polygon mode):
    PARAMS = dict(APPLY=False, GROWTH_BIAS=1.5, AUTO_LENGTH=True,
                  LENGTH_PER_PAD_WIDTH=0.8, MIN_LENGTH=0.10, MAX_LENGTH=0.80)
GROWTH_BIAS=1 retains the original profile; larger values delay widening and
lateral departure until closer to the pad. Unsafe profiles are rejected.
AUTO_LENGTH uses the pad's central cross-section perpendicular to the entry.
LENGTH_PER_PAD_WIDTH is the exposed taper length / pad-width ratio. For a 0.20 mm
cross-section, 0.8 gives 0.16 mm; for 0.56 mm it gives about 0.45 mm.
Set AUTO_LENGTH=False to use LENGTH in mm. Available routing can shorten either
request. Total copper extends farther under the pad; pad positions never move.

Quad-redriver RX entries (select pads 29/30, 32/33, 36/37, 39/40 entry items):
    PARAMS = dict(APPLY=True, LENGTH=0.55, TARGET_WIDTH=0.13,
                  EXTEND_PATH=True, CONSTANT_WIDTH=True, ENTRY_JOG=0.05,
                  SHIFT_PAD_FANIN=True, PAD_ESCAPE=False)
Use the measured lateral-ground-loss length plus ENTRY_JOG for LENGTH. This board's
16 RX pairs require 0.525-0.635 mm total, corresponding to 0.475-0.585 mm wide sections.

Fine-pitch BGA escape (select both members of each differential pair):
    PARAMS = dict(APPLY=True, LENGTH=0.80, PAD_ESCAPE=True,
                  PAD_ENTRY_WIDTH=0.20, STEPS=24)
The existing routed width is used at the route end and increases monotonically
toward the pad. The existing centerline path provides the pair fanout, without
round-ended staged tracks or an abrupt width step.

Select only the final track or arc entering each pad. Select both members of
a differential pair for symmetric tapers. Review the result and run DRC before save.
Test on a copy first: direct pcbnew edits do not create an editor undo transaction.
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
LENGTH = 0.45            # exposed-route length consumed by the pad transition
AUTO_LENGTH = False
LENGTH_PER_PAD_WIDTH = 0.8
MIN_LENGTH = 0.10
MAX_LENGTH = 0.80
GROWTH_BIAS = 1.0
STEPS = 8                # legacy stages; continuous outlines use at least 64 samples
TARGET_WIDTH = None      # pad-end width; None fits the pad (constant mode uses MAX_FACTOR)
EXTEND_PATH = False      # consume unique upstream items to reach LENGTH
CONSTANT_WIDTH = False   # use a uniform wide section after an original-width entry jog
ENTRY_JOG = 0.05         # lateral transition length before a CONSTANT_WIDTH section
PAD_NECK = 0.0           # return to original width over this distance before the pad boundary
SHIFT_PAD_FANIN = False  # move a unique under-pad continuation with the widened endpoint
PAD_ESCAPE = False       # widen from the existing routed trace toward a fine-pitch pad
PAD_ENTRY_WIDTH = None   # required wider pad-end width when PAD_ESCAPE is enabled
MAX_FACTOR = 2.0         # maximum width relative to the entering trace
PAD_FILL = 0.82          # maximum fraction of pad half-span used in the outboard direction
MIN_WIDTH_GAIN = 0.005   # skip tapers whose useful width increase is smaller than this
MIN_SEGMENT = 0.025      # do not create extremely short stage segments
PAIR_ANGLE = 12.0        # maximum direction mismatch for paired pad-entry tracks (degrees)
PAIR_DISTANCE = 2.0      # maximum pad-entry separation considered a local pair
PAIR_TOL = 0.00001       # numerical tolerance only; reject any measurable pair-gap reduction
END_TOL = 0.03           # fallback tolerance when testing whether an endpoint lies in a pad
BOUNDARY_STEPS = 40      # binary-search iterations for the pad-outline crossing
ARC_SAMPLES = 64         # chords used to prove clearance for each tapered arc stage


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


def _item_key(item):
    try:
        return item.m_Uuid.AsString()
    except Exception:
        return str(getattr(item, "this", id(item)))


def smoothstep(value):
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def _pad_cross_section(pad, tangent):
    center = p2(pad.GetPosition())
    normal = (-tangent[1], tangent[0])
    spans = []
    for sign in (-1.0, 1.0):
        low, high = 0.0, norm(p2(pad.GetSize()))
        for _ in range(BOUNDARY_STEPS):
            middle = (low + high) / 2.0
            if _pad_hit(pad, add(center, mul(normal, sign * middle))):
                low = middle
            else:
                high = middle
        spans.append(low)
    return 2.0 * min(spans)


def _requested_length(candidate):
    if AUTO_LENGTH and not CONSTANT_WIDTH:
        if not (0 < MIN_LENGTH <= MAX_LENGTH and LENGTH_PER_PAD_WIDTH > 0):
            raise ValueError("Auto length requires 0 < MIN_LENGTH <= MAX_LENGTH and a positive ratio")
        span = _pad_cross_section(candidate["pad"], candidate["direction"])
        return max(MIN_LENGTH, min(MAX_LENGTH, LENGTH_PER_PAD_WIDTH * span))
    if not math.isfinite(float(LENGTH)) or LENGTH <= 0:
        raise ValueError("LENGTH must be positive and finite")
    return float(LENGTH)


def _growth_profile(fraction):
    if not math.isfinite(float(GROWTH_BIAS)) or GROWTH_BIAS < 1.0:
        raise ValueError("GROWTH_BIAS must be finite and at least 1")
    return smoothstep(fraction) ** (2.0 * GROWTH_BIAS)


def _circle_from_3(first, middle, last):
    ax, ay = first; bx, by = middle; cx, cy = last
    denominator = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(denominator) < 1e-12:
        return None
    a2 = ax * ax + ay * ay
    b2 = bx * bx + by * by
    c2 = cx * cx + cy * cy
    center = ((a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / denominator,
              (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / denominator)
    return center, norm(sub(first, center))


def _arc_path(track, inside_at_start):
    """Return distance-parametric geometry directed from the route toward the pad."""
    start, middle, end = p2(track.GetStart()), p2(track.GetMid()), p2(track.GetEnd())
    circle = _circle_from_3(start, middle, end)
    if circle is None:
        return None
    center, radius = circle
    start_angle = math.atan2(start[1] - center[1], start[0] - center[0])
    middle_angle = math.atan2(middle[1] - center[1], middle[0] - center[0])
    end_angle = math.atan2(end[1] - center[1], end[0] - center[0])
    ccw_middle = (middle_angle - start_angle) % (2.0 * math.pi)
    ccw_end = (end_angle - start_angle) % (2.0 * math.pi)
    if ccw_middle <= ccw_end:
        sign, extent = 1.0, ccw_end
    else:
        sign, extent = -1.0, 2.0 * math.pi - ccw_end
    if inside_at_start:
        start_angle = end_angle
        sign = -sign
    length = radius * extent

    def point_at(distance):
        angle = start_angle + sign * max(0.0, min(length, distance)) / radius
        return (center[0] + radius * math.cos(angle), center[1] + radius * math.sin(angle))

    def tangent_at(distance):
        angle = start_angle + sign * max(0.0, min(length, distance)) / radius
        return (-sign * math.sin(angle), sign * math.cos(angle))

    return dict(length=length, point_at=point_at, tangent_at=tangent_at,
                far=point_at(0.0), pad_end=point_at(length))


def _segment_path(track, toward_start):
    far = p2(track.GetEnd() if toward_start else track.GetStart())
    near = p2(track.GetStart() if toward_start else track.GetEnd())
    delta = sub(near, far)
    length = norm(delta)
    direction = unit(delta)
    return dict(length=length,
                point_at=lambda distance: add(far, mul(direction, max(0.0, min(length, distance)))),
                tangent_at=lambda distance: direction, far=far, pad_end=near)


def _route_component(track, toward):
    start, end = p2(track.GetStart()), p2(track.GetEnd())
    if norm(sub(start, toward)) <= 0.002:
        path = (_arc_path(track, True) if isinstance(track, pcbnew.PCB_ARC)
                else _segment_path(track, True))
        toward_at_start = True
    elif norm(sub(end, toward)) <= 0.002:
        path = (_arc_path(track, False) if isinstance(track, pcbnew.PCB_ARC)
                else _segment_path(track, False))
        toward_at_start = False
    else:
        return None
    if path is None:
        return None
    path.update(obj=track, kind="arc" if isinstance(track, pcbnew.PCB_ARC) else "seg",
                toward_at_start=toward_at_start)
    return path


def _compose_candidate_path(candidate, components):
    offsets = []
    total = 0.0
    for component in components:
        offsets.append(total)
        total += component["length"]

    def locate(distance):
        bounded = max(0.0, min(total, distance))
        for index in range(len(components) - 1, -1, -1):
            if bounded + 1e-12 >= offsets[index]:
                return components[index], bounded - offsets[index]
        return components[0], 0.0

    candidate["components"] = components
    candidate["component_offsets"] = offsets
    candidate["point_at"] = lambda distance: locate(distance)[0]["point_at"](locate(distance)[1])
    candidate["tangent_at"] = lambda distance: locate(distance)[0]["tangent_at"](locate(distance)[1])
    candidate["component_at"] = locate
    candidate["component_breaks"] = offsets[1:]
    candidate["length"] = total
    candidate["path_end"] = total
    candidate["far"] = components[0]["far"]
    candidate["kind"] = "path" if len(components) > 1 else components[0]["kind"]


def _extend_candidate(board, candidate):
    candidate["terminal_kind"] = candidate["kind"]
    candidate["terminal_point_at"] = candidate["point_at"]
    candidate["terminal_tangent_at"] = candidate["tangent_at"]
    candidate["terminal_entry"] = candidate["path_end"]
    candidate["terminal_total"] = candidate.get("total_path", candidate["path_end"])
    downstream = []
    for track in board.GetTracks():
        if (isinstance(track, pcbnew.PCB_VIA) or _item_key(track) == _item_key(candidate["obj"]) or
                track.GetNetCode() != candidate["net"] or track.GetLayer() != candidate["layer"]):
            continue
        start, end = p2(track.GetStart()), p2(track.GetEnd())
        if norm(sub(start, candidate["pad_end"])) <= 0.002 and _pad_hit(candidate["pad"], end):
            downstream.append((track, True))
        elif norm(sub(end, candidate["pad_end"])) <= 0.002 and _pad_hit(candidate["pad"], start):
            downstream.append((track, False))
    candidate["downstream"] = downstream[0] if len(downstream) == 1 else None
    base = dict(length=candidate["path_end"], point_at=candidate["point_at"],
                tangent_at=candidate["tangent_at"], far=candidate["far"],
                pad_end=candidate["end"], obj=candidate["obj"], kind=candidate["kind"],
                toward_at_start=(norm(sub(p2(candidate["obj"].GetStart()), candidate["pad_end"])) <
                                 norm(sub(p2(candidate["obj"].GetEnd()), candidate["pad_end"]))))
    components = [base]
    used = {_item_key(candidate["obj"])}
    requested_length = _requested_length(candidate)
    candidate["requested_length"] = requested_length
    required = (requested_length + MIN_SEGMENT if CONSTANT_WIDTH
                else max(requested_length / 0.80, requested_length + MIN_SEGMENT))
    while (EXTEND_PATH or PAD_ESCAPE or not CONSTANT_WIDTH) and sum(component["length"] for component in components) + 1e-9 < required:
        join = components[0]["far"]
        choices = []
        for track in board.GetTracks():
            if (isinstance(track, pcbnew.PCB_VIA) or _item_key(track) in used or
                    track.GetNetCode() != candidate["net"] or track.GetLayer() != candidate["layer"] or
                    abs(pcbnew.ToMM(track.GetWidth()) - candidate["width"]) > 0.0005):
                continue
            component = _route_component(track, join)
            if component is not None:
                choices.append(component)
        if len(choices) != 1:
            break
        component = choices[0]
        components.insert(0, component)
        used.add(_item_key(component["obj"]))
    _compose_candidate_path(candidate, components)
    candidate["direction"] = candidate["tangent_at"](candidate["path_end"])
    candidate["normal"] = (-candidate["direction"][1], candidate["direction"][0])


def _arc_piece_points(first, middle, last, samples):
    """Sample a three-point circular arc in its actual start-to-end direction."""
    circle = _circle_from_3(first, middle, last)
    if circle is None:
        return [first, last]
    center, radius = circle
    start_angle = math.atan2(first[1] - center[1], first[0] - center[0])
    middle_angle = math.atan2(middle[1] - center[1], middle[0] - center[0])
    end_angle = math.atan2(last[1] - center[1], last[0] - center[0])
    ccw_middle = (middle_angle - start_angle) % (2.0 * math.pi)
    ccw_end = (end_angle - start_angle) % (2.0 * math.pi)
    if ccw_middle <= ccw_end:
        sign, extent = 1.0, ccw_end
    else:
        sign, extent = -1.0, 2.0 * math.pi - ccw_end
    count = max(2, int(samples))
    return [(center[0] + radius * math.cos(start_angle + sign * extent * index / count),
             center[1] + radius * math.sin(start_angle + sign * extent * index / count))
            for index in range(count + 1)]


def _piece_lines(piece):
    """Flatten a straight or curved constant-width piece for clearance proof."""
    if len(piece) < 4 or piece[3] is None:
        return [(piece[0], piece[1], piece[2])]
    points = _arc_piece_points(piece[0], piece[3], piece[1], ARC_SAMPLES)
    return [(points[index], points[index + 1], piece[2]) for index in range(len(points) - 1)]


def _geometry_lines(pieces):
    return [line for piece in pieces for line in _piece_lines(piece)]


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


def _pad_entry_path(pad, point_at, length):
    """Final outside-to-inside pad crossing along a distance-parametric path."""
    outside, inside = point_at(0.0), point_at(length)
    if _pad_hit(pad, outside) or not _pad_hit(pad, inside):
        return None
    low, high = 0.0, length
    samples = max(16, int(math.ceil(length / 0.02)))
    for index in range(samples - 1, -1, -1):
        distance = length * index / samples
        if not _pad_hit(pad, point_at(distance)):
            low = distance
            high = length * (index + 1) / samples
            break
    for _ in range(max(8, int(BOUNDARY_STEPS))):
        middle = 0.5 * (low + high)
        if _pad_hit(pad, point_at(middle)):
            high = middle
        else:
            low = middle
    return min(length, high + 0.000002)


def _candidate(board, track, endpoint, other_end, pad):
    """Describe a taper directed from the routed trace toward its pad."""
    total_length = norm(sub(endpoint, other_end))
    direction0 = unit(sub(endpoint, other_end))
    point_at = lambda distance: add(other_end, mul(direction0, distance))
    tangent_at = lambda distance: direction0
    entry_distance = _pad_entry_path(pad, point_at, total_length)
    if entry_distance is None:
        return None
    entry = point_at(entry_distance)
    direction = tangent_at(entry_distance)
    length = entry_distance
    if length < max(2.0 * MIN_SEGMENT, 0.05):
        return None
    candidate = dict(
        obj=track, net=track.GetNetCode(), name=track.GetNetname(), layer=track.GetLayer(),
        pad=pad, end=entry, pad_end=endpoint, far=other_end, direction=direction,
        normal=(-direction[1], direction[0]), length=length,
        width=pcbnew.ToMM(track.GetWidth()), partner=None, side=None, kind="seg",
        point_at=point_at, tangent_at=tangent_at, path_end=entry_distance,
    )
    _extend_candidate(board, candidate)
    return candidate


def _arc_candidate(board, track, endpoint, pad, inside_at_start):
    path = _arc_path(track, inside_at_start)
    if path is None:
        return None
    entry_distance = _pad_entry_path(pad, path["point_at"], path["length"])
    if entry_distance is None or entry_distance < max(2.0 * MIN_SEGMENT, 0.05):
        return None
    entry = path["point_at"](entry_distance)
    direction = path["tangent_at"](entry_distance)
    candidate = dict(
        obj=track, net=track.GetNetCode(), name=track.GetNetname(), layer=track.GetLayer(),
        pad=pad, end=entry, pad_end=endpoint, far=path["far"], direction=direction,
        normal=(-direction[1], direction[0]), length=entry_distance,
        width=pcbnew.ToMM(track.GetWidth()), partner=None, side=None, kind="arc",
        point_at=path["point_at"], tangent_at=path["tangent_at"],
        path_end=entry_distance, total_path=path["length"],
    )
    _extend_candidate(board, candidate)
    return candidate


def read_candidates(board):
    """Read selected straight track endpoints that terminate in same-net pads."""
    result = []
    pads_by_net = collections.defaultdict(list)
    for footprint in board.GetFootprints():
        for pad in footprint.Pads():
            pads_by_net[pad.GetNetCode()].append(pad)
    for track in board.GetTracks():
        if isinstance(track, pcbnew.PCB_VIA):
            continue
        if USE_SELECTION and not track.IsSelected():
            continue
        start, end = p2(track.GetStart()), p2(track.GetEnd())
        net_pads = pads_by_net.get(track.GetNetCode(), ())
        start_pad = _endpoint_pad(net_pads, track, start)
        end_pad = _endpoint_pad(net_pads, track, end)
        if start_pad:
            candidate = (_arc_candidate(board, track, start, start_pad, True)
                         if isinstance(track, pcbnew.PCB_ARC)
                         else _candidate(board, track, start, end, start_pad))
            if candidate:
                result.append(candidate)
        if end_pad and end_pad is not start_pad:
            candidate = (_arc_candidate(board, track, end, end_pad, False)
                         if isinstance(track, pcbnew.PCB_ARC)
                         else _candidate(board, track, end, start, end_pad))
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
            first_direction = (candidate["tangent_at"](0.0)
                               if candidate["kind"] != "seg" else candidate["direction"])
            second_direction = (other["tangent_at"](0.0)
                                if other["kind"] != "seg" else other["direction"])
            alignment = abs(dot(first_direction, second_direction))
            if CONSTANT_WIDTH and alignment < math.cos(math.radians(PAIR_ANGLE)):
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
    requested = MAX_FACTOR * candidate["width"] if TARGET_WIDTH is None else float(TARGET_WIDTH)
    return max(candidate["width"], min(requested, pad_limited))


def plan_candidate(candidate):
    """Build connected, staged track geometry while holding the inboard edge fixed."""
    width0 = candidate["width"]
    if PAD_ESCAPE or not CONSTANT_WIDTH:
        return plan_pad_escape(candidate)
    width1 = _target_width(candidate)
    if width1 - width0 < MIN_WIDTH_GAIN:
        return None
    side = candidate["side"] if candidate["side"] is not None else _single_side(candidate)
    taper_length = min(LENGTH, (candidate["length"] - MIN_SEGMENT
                               if CONSTANT_WIDTH else candidate["length"] * 0.80))
    stage_count = max(2, min(int(STEPS), int(taper_length / MIN_SEGMENT)))
    path_start = candidate["path_end"] - taper_length

    def tapered_point(distance, width):
        base = candidate["point_at"](distance)
        tangent = candidate["tangent_at"](distance)
        outboard = mul((-tangent[1], tangent[0]), side)
        return add(base, mul(outboard, (width - width0) / 2.0))

    if CONSTANT_WIDTH:
        jog_length = min(max(MIN_SEGMENT, float(ENTRY_JOG)), taper_length / 2.0)
        distances = [path_start, path_start + jog_length, candidate["path_end"]]
    else:
        jog_length = 0.0
        distances = [path_start + taper_length * index / stage_count
                     for index in range(stage_count + 1)]
    neck_length = min(max(0.0, float(PAD_NECK)), taper_length / 2.0)
    if neck_length > 0.0:
        neck_start = candidate["path_end"] - neck_length
        neck_steps = max(2, int(math.ceil(neck_length / MIN_SEGMENT)))
        distances.extend(neck_start + neck_length * index / neck_steps
                         for index in range(neck_steps + 1))
    distances.extend(distance for distance in candidate.get("component_breaks", ())
                     if path_start + 1e-9 < distance < candidate["path_end"] - 1e-9)
    distances = sorted(set(round(distance, 12) for distance in distances))
    nodes = []
    widths = []
    for distance in distances:
        if CONSTANT_WIDTH:
            width = width0 if distance <= path_start + 1e-12 else width1
            nodes.append(tapered_point(distance, width))
            widths.append(width)
            continue
        if neck_length > 0.0 and distance >= candidate["path_end"] - neck_length:
            fraction = (candidate["path_end"] - distance) / neck_length
        else:
            rise_length = taper_length - neck_length
            fraction = (distance - path_start) / rise_length
        shaped = smoothstep(max(0.0, min(1.0, fraction)))
        width = width0 + (width1 - width0) * shaped
        nodes.append(tapered_point(distance, width))
        widths.append(width)

    # Make the last stage full width. Each widening starts at a shared node whose
    # center has already moved outward, so its inboard edge never crosses inward.
    if neck_length <= 0.0 and len(distances) >= 3:
        widths[-2] = width1
        nodes[-2] = tapered_point(distances[-2], width1)
    pieces = []
    for index in range(len(distances) - 1):
        narrowing = neck_length > 0.0 and distances[index] >= candidate["path_end"] - neck_length
        segment_width = (width0 if CONSTANT_WIDTH and index == 0 else
                 widths[index + 1] if narrowing else widths[index])
        if neck_length <= 0.0 and index == len(distances) - 2:
            segment_width = width1
        middle = None
        middle_distance = 0.5 * (distances[index] + distances[index + 1])
        component, _ = candidate["component_at"](middle_distance)
        if component["kind"] == "arc" and not (CONSTANT_WIDTH and index == 0):
            middle_width = 0.5 * (widths[index] + widths[index + 1])
            if neck_length <= 0.0 and index == len(distances) - 2:
                middle_width = width1
            middle = tapered_point(middle_distance, middle_width)
        pieces.append((nodes[index], nodes[index + 1], segment_width, middle))

    bridge_middle = None
    if candidate["terminal_kind"] == "arc":
        bridge_distance = 0.5 * (candidate["terminal_entry"] + candidate["terminal_total"])
        base = candidate["terminal_point_at"](bridge_distance)
        tangent = candidate["terminal_tangent_at"](bridge_distance)
        outboard = mul((-tangent[1], tangent[0]), side)
        bridge_middle = add(base, mul(outboard, (width1 - width0) / 4.0))
    bridge_width = width0 if neck_length > 0.0 else width1
    bridge_end = candidate["pad_end"]
    if SHIFT_PAD_FANIN and candidate.get("downstream") is not None and neck_length <= 0.0:
        tangent = candidate["terminal_tangent_at"](candidate["terminal_total"])
        outboard = mul((-tangent[1], tangent[0]), side)
        bridge_end = add(bridge_end, mul(outboard, (width1 - width0) / 2.0))
    bridge = (nodes[-1], bridge_end, bridge_width, bridge_middle)

    baseline = []
    baseline_distances = [path_start + taper_length * index / (stage_count * ARC_SAMPLES)
                          for index in range(stage_count * ARC_SAMPLES + 1)]
    baseline_distances.extend(distance for distance in candidate.get("component_breaks", ())
                              if path_start < distance < candidate["path_end"])
    baseline_distances = sorted(set(baseline_distances))
    previous = candidate["point_at"](baseline_distances[0])
    for distance in baseline_distances[1:]:
        point = candidate["point_at"](distance)
        baseline.append((previous, point, width0))
        previous = point
    active_component, active_distance = candidate["component_at"](path_start)
    prefix_middle = (active_component["point_at"](active_distance / 2.0)
                     if active_component["kind"] == "arc" else None)
    return dict(candidate=candidate, start=nodes[0], end=nodes[-1], width0=width0,
                width1=width1, pieces=pieces, bridge=bridge, baseline=baseline,
                prefix_mid=prefix_middle, active_component=active_component,
                path_start=path_start, side=side, length=taper_length)


def plan_pad_escape(candidate):
    """Curve smoothly from the routed pair into a wider pad entry."""
    route_width = candidate["width"]
    taper_length = min(candidate.get("requested_length", _requested_length(candidate)), candidate["length"] * 0.80)
    _growth_profile(0.5)
    stage_count = max(64, int(STEPS))
    path_start = candidate["path_end"] - taper_length
    side = candidate["side"] if candidate["side"] is not None else _single_side(candidate)
    route_end = candidate["point_at"](path_start)
    pad_end = candidate["pad_end"] if PAD_ESCAPE else p2(candidate["pad"].GetPosition())
    chord = sub(pad_end, route_end)
    chord_length = norm(chord)
    route_tangent = unit(candidate["tangent_at"](path_start))
    pad_tangent = unit(candidate["terminal_tangent_at"](candidate["terminal_total"]))
    if dot(route_tangent, chord) < 0.0:
        route_tangent = mul(route_tangent, -1.0)
    if dot(pad_tangent, chord) < 0.0:
        pad_tangent = mul(pad_tangent, -1.0)
    pad_normal = (-pad_tangent[1], pad_tangent[0])
    pad = candidate["pad"]
    half_widths = []
    for sign in (-1.0, 1.0):
        low, high = 0.0, norm(p2(pad.GetSize()))
        for _ in range(BOUNDARY_STEPS):
            middle = (low + high) / 2.0
            if _pad_hit(pad, add(pad_end, mul(pad_normal, sign * middle))):
                low = middle
            else:
                high = middle
        half_widths.append(low)
    available_width = 2.0 * min(half_widths) * PAD_FILL
    requested = PAD_ENTRY_WIDTH if PAD_ESCAPE else TARGET_WIDTH
    pad_width = available_width if requested is None else min(float(requested), available_width)
    if pad_width <= route_width + MIN_WIDTH_GAIN:
        return None
    handle = min(taper_length / 3.0, chord_length / 3.0)
    control1 = add(route_end, mul(route_tangent, handle))
    control2 = sub(pad_end, mul(pad_tangent, handle))

    def base_point(fraction):
        inverse = 1.0 - fraction
        return add(add(mul(route_end, inverse ** 3),
                       mul(control1, 3.0 * inverse * inverse * fraction)),
                   add(mul(control2, 3.0 * inverse * fraction * fraction),
                       mul(pad_end, fraction ** 3)))

    def base_derivative(fraction):
        inverse = 1.0 - fraction
        derivative = add(add(mul(sub(control1, route_end), 3.0 * inverse * inverse),
                             mul(sub(control2, control1), 6.0 * inverse * fraction)),
                         mul(sub(pad_end, control2), 3.0 * fraction * fraction))
        return derivative

    def curve_point(fraction):
        displacement = sub(base_point(fraction), route_end)
        forward = mul(route_tangent, dot(displacement, route_tangent))
        lateral = sub(displacement, forward)
        blend = smoothstep(fraction) ** (GROWTH_BIAS - 1.0)
        return add(route_end, add(forward, mul(lateral, blend)))

    def curve_tangent(fraction):
        if GROWTH_BIAS == 1.0 or fraction in (0.0, 1.0):
            return unit(base_derivative(fraction))
        displacement = sub(base_point(fraction), route_end)
        lateral = sub(displacement, mul(route_tangent, dot(displacement, route_tangent)))
        derivative = base_derivative(fraction)
        forward = mul(route_tangent, dot(derivative, route_tangent))
        blend = smoothstep(fraction) ** (GROWTH_BIAS - 1.0)
        slope = ((GROWTH_BIAS - 1.0) * smoothstep(fraction) ** (GROWTH_BIAS - 2.0)
                 * 6.0 * fraction * (1.0 - fraction))
        return unit(add(add(forward, mul(sub(derivative, forward), blend)), mul(lateral, slope)))

    fractions = [index / stage_count for index in range(stage_count + 1)]
    widths = [route_width + (pad_width - route_width) * _growth_profile(fraction)
              for fraction in fractions]
    nodes = [curve_point(fraction) for fraction in fractions]

    left_edge = []
    right_edge = []
    for fraction, center, width in zip(fractions, nodes, widths):
        tangent = curve_tangent(fraction)
        normal = (-tangent[1], tangent[0])
        left_edge.append(add(center, mul(normal, width / 2.0)))
        right_edge.append(add(center, mul(normal, -width / 2.0)))
    polygon = left_edge + list(reversed(right_edge))
    if candidate["partner"] is not None:
        weights = [16.0 * fraction ** 2 * (1.0 - fraction) ** 2 for fraction in fractions]
        correction = _facing_edge_correction(candidate, polygon, weights + list(reversed(weights)))
        if correction is None:
            print("  REJECTED %s: cannot preserve the local facing edge" % candidate["name"])
            return None
        polygon = [add(point, mul(correction, weight))
                   for point, weight in zip(polygon, weights + list(reversed(weights)))]
        nodes = [add(point, mul(correction, weight)) for point, weight in zip(nodes, weights)]
    if not _simple_polygon(polygon):
        print("  REJECTED %s: taper outline folds over itself" % candidate["name"])
        return None

    pieces = []
    for index in range(len(fractions) - 1):
        pieces.append((nodes[index], nodes[index + 1],
                   max(widths[index], widths[index + 1]), None))

    bridge = (nodes[-1], pad_end, pad_width, None)

    baseline = []
    baseline_distances = [path_start + taper_length * index / stage_count
                          for index in range(stage_count + 1)]
    baseline_distances.extend(distance for distance in candidate.get("component_breaks", ())
                              if path_start < distance < candidate["path_end"])
    baseline_distances = sorted(set(baseline_distances))
    previous = candidate["point_at"](baseline_distances[0])
    for distance in baseline_distances[1:]:
        point = candidate["point_at"](distance)
        baseline.append((previous, point, route_width))
        previous = point

    active_component, active_distance = candidate["component_at"](path_start)
    prefix_middle = (active_component["point_at"](active_distance / 2.0)
                     if active_component["kind"] == "arc" else None)
    return dict(candidate=candidate, start=nodes[0], end=nodes[-1],
                width0=route_width, width1=pad_width, pieces=pieces, bridge=bridge,
                polygon=polygon, nodes=nodes, widths=widths,
                baseline=baseline, prefix_mid=prefix_middle,
                active_component=active_component, path_start=path_start,
                side=side, length=taper_length)


def _polygon_edges(polygon):
    return list(zip(polygon, polygon[1:] + polygon[:1]))


def _facing_edge_correction(candidate, polygon, weights):
    """Solve an outward-only offset against the original copper's section envelope."""
    origin = p2(candidate["pad"].GetPosition())
    toward = unit(sub(p2(candidate["partner"]["pad"].GetPosition()), origin))
    along = (-toward[1], toward[0])
    if norm(toward) < 0.5:
        return None
    original = pcbnew.SHAPE_POLY_SET()
    items = [candidate["pad"]] + [component["obj"] for component in candidate["components"]]
    if candidate.get("downstream") is not None:
        items.append(candidate["downstream"][0])
    for item in items:
        shape = pcbnew.SHAPE_POLY_SET()
        item.TransformShapeToPolygon(shape, candidate["layer"], 0, 1, pcbnew.ERROR_OUTSIDE)
        original.BooleanAdd(shape)

    def project(point):
        relative = sub(point, origin)
        return (dot(relative, toward), dot(relative, along))

    outlines = [[project(p2(original.Outline(index).CPoint(vertex)))
                 for vertex in range(original.Outline(index).PointCount())]
                for index in range(original.OutlineCount())]
    projected = [project(point) for point in polygon]
    low = min(point[1] for point in projected)
    high = max(point[1] for point in projected)
    depths = sorted(set(point[1] for outline in outlines + [projected]
                        for point in outline if low <= point[1] <= high))
    depths += [(first + last) / 2.0 for first, last in zip(depths, depths[1:])]

    def intersections(outline, depth):
        for index, (start, end) in enumerate(_polygon_edges(outline)):
            if min(start[1], end[1]) - 1e-12 <= depth <= max(start[1], end[1]) + 1e-12:
                if abs(end[1] - start[1]) < 1e-12:
                    yield start[0], index, 0.0
                    yield end[0], index, 1.0
                else:
                    fraction = max(0.0, min(1.0, (depth - start[1]) / (end[1] - start[1])))
                    yield start[0] + fraction * (end[0] - start[0]), index, fraction

    amplitude = 0.0
    for depth in depths:
        original_hits = [hit[0] for outline in outlines for hit in intersections(outline, depth)]
        if not original_hits:
            return None
        limit = max(original_hits)
        for lateral, index, fraction in intersections(projected, depth):
            excess = lateral - limit
            if excess <= 0.000001:
                continue
            weight = weights[index] * (1.0 - fraction) + weights[(index + 1) % len(weights)] * fraction
            if weight <= 1e-12:
                return None
            amplitude = max(amplitude, (excess + 0.000001) / weight)
    return mul(toward, -amplitude)


def _simple_polygon(polygon):
    edges = _polygon_edges(polygon)
    for index, (start, end) in enumerate(edges):
        if norm(sub(end, start)) < 0.000001:
            return False
        for other_index in range(index + 2, len(edges)):
            if index == 0 and other_index == len(edges) - 1:
                continue
            if segment_distance(start, end, *edges[other_index]) < 0.000001:
                return False
    return True


def _polygon_contains(polygon, point):
    inside = False
    for start, end in _polygon_edges(polygon):
        if (start[1] > point[1]) != (end[1] > point[1]):
            crossing = start[0] + (point[1] - start[1]) * (end[0] - start[0]) / (end[1] - start[1])
            if point[0] < crossing:
                inside = not inside
    return inside


def _polygon_gap(first, second):
    if _polygon_contains(first, second[0]) or _polygon_contains(second, first[0]):
        return 0.0
    return min(segment_distance(first_start, first_end, second_start, second_end)
               for first_start, first_end in _polygon_edges(first)
               for second_start, second_end in _polygon_edges(second))


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
            label = candidate["name"].rsplit("/", 1)[-1]
            print("  REJECTED %-16s partner has no viable taper plan" % label)
            rejected.add(id(candidate))
            rejected.add(id(partner))
            continue
        checked.add(id(candidate))
        checked.add(id(partner))
        original_gap = min(segment_distance(a0, a1, b0, b1) - aw / 2.0 - bw / 2.0
                   for a0, a1, aw in plan["baseline"]
                   for b0, b1, bw in other["baseline"])
        # The bridge runs beneath its own pad copper from the outline crossing to
        # the original endpoint. Pair-route clearance is meaningful only through
        # the pad boundary; inside it, the fixed pad geometry controls clearance.
        first_geometry = _geometry_lines(plan["pieces"])
        second_geometry = _geometry_lines(other["pieces"])
        if "polygon" in plan and "polygon" in other:
            tapered_gap = _polygon_gap(plan["polygon"], other["polygon"])
        else:
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
        print("No %s track or arc endpoint enters a same-net pad." % scope)
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
        active = plan["active_component"]
        original = active["obj"]
        active_index = candidate["components"].index(active)
        for component in candidate["components"]:
            try:
                component["obj"].ClearSelected()
            except Exception:
                pass
        for component in candidate["components"][active_index + 1:]:
            board.Remove(component["obj"])
        if active["toward_at_start"]:
            original.SetStart(v2(plan["start"]))
        else:
            original.SetEnd(v2(plan["start"]))
        if active["kind"] == "arc":
            original.SetMid(v2(plan["prefix_mid"]))
        original.SetWidth(pcbnew.FromMM(plan["width0"]))
        downstream = candidate.get("downstream")
        if SHIFT_PAD_FANIN and downstream is not None:
            downstream_track, at_start = downstream
            if at_start:
                downstream_track.SetStart(v2(plan["bridge"][1]))
            else:
                downstream_track.SetEnd(v2(plan["bridge"][1]))
        if "polygon" in plan:
            outline = pcbnew.SHAPE_LINE_CHAIN()
            for point in plan["polygon"]:
                outline.Append(v2(point))
            outline.SetClosed(True)
            polygon = pcbnew.SHAPE_POLY_SET()
            polygon.AddOutline(outline)
            taper = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_POLY)
            taper.SetPolyShape(polygon)
            taper.SetFilled(True)
            taper.SetWidth(0)
            taper.SetLayer(candidate["layer"])
            taper.SetNetCode(candidate["net"])
            board.Add(taper)
            made += 1
            continue
        for start, end, width, middle in plan["pieces"]:
            track = pcbnew.PCB_ARC(board) if middle is not None else pcbnew.PCB_TRACK(board)
            track.SetStart(v2(start))
            if middle is not None:
                track.SetMid(v2(middle))
            track.SetEnd(v2(end))
            track.SetWidth(pcbnew.FromMM(width))
            track.SetLayer(candidate["layer"])
            track.SetNetCode(candidate["net"])
            board.Add(track)
            made += 1
        start, end, width, middle = plan["bridge"]
        if norm(sub(end, start)) >= 1e-6:
            bridge = pcbnew.PCB_ARC(board) if middle is not None else pcbnew.PCB_TRACK(board)
            bridge.SetStart(v2(start))
            if middle is not None:
                bridge.SetMid(v2(middle))
            bridge.SetEnd(v2(end))
            bridge.SetWidth(pcbnew.FromMM(width))
            bridge.SetLayer(candidate["layer"])
            bridge.SetNetCode(candidate["net"])
            board.Add(bridge)
            made += 1
    _refresh(board)
    item_kind = "copper taper polygon(s)" if PAD_ESCAPE or not CONSTANT_WIDTH else "staged track segment(s)"
    print("APPLIED: %d %s. Review pad overlap and run DRC before save." % (made, item_kind))
    return made


try:
    class PadEntryTeardropPlugin(pcbnew.ActionPlugin):
        def defaults(self):
            self.name = "Pad-entry tapers (diff-pair safe)"
            self.category = "Modify PCB"
            self.description = "Taper selected pad entries while preserving differential-pair gap."
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
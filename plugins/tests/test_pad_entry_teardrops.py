import importlib.util
import io
from pathlib import Path
import unittest
from unittest.mock import patch

import pcbnew


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("pad_entry_teardrops", ROOT / "plugins/pad_entry_teardrops.py")
tapers = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tapers)


def assert_facing_edge_preserved(test, candidate, polygon):
    toward = tapers.unit(tapers.sub(tapers.p2(candidate["partner"]["pad"].GetPosition()),
                                   tapers.p2(candidate["pad"].GetPosition())))
    along = (-toward[1], toward[0])
    original = pcbnew.SHAPE_POLY_SET()
    items = [candidate["pad"]] + [component["obj"] for component in candidate["components"]]
    if candidate.get("downstream"):
        items.append(candidate["downstream"][0])
    for item in items:
        item.TransformShapeToPolygon(original, candidate["layer"], 0, 1, pcbnew.ERROR_OUTSIDE)
    outlines = [[tapers.p2(original.Outline(index).CPoint(vertex))
                 for vertex in range(original.Outline(index).PointCount())]
                for index in range(original.OutlineCount())]

    def projected_edges(outlines):
        return [((tapers.dot(first, along), tapers.dot(first, toward)),
                 (tapers.dot(last, along), tapers.dot(last, toward)))
                for outline in outlines for first, last in tapers._polygon_edges(outline)]

    old_edges, new_edges = projected_edges(outlines), projected_edges([polygon])

    def facing(edges, depth):
        return max((first[1] + (last[1] - first[1]) * (depth - first[0]) / (last[0] - first[0])
                    for first, last in edges
                    if min(first[0], last[0]) <= depth <= max(first[0], last[0])
                    and abs(last[0] - first[0]) > 1e-12), default=None)

    low = min(first[0] for first, last in new_edges)
    high = max(first[0] for first, last in new_edges)
    depths = sorted({point[0] for edge in old_edges + new_edges for point in edge
                     if low <= point[0] <= high})
    depths += [(first + last) / 2 for first, last in zip(depths, depths[1:])]
    for depth in depths:
        original_edge, generated_edge = facing(old_edges, depth), facing(new_edges, depth)
        if original_edge is not None and generated_edge is not None:
            test.assertLessEqual(generated_edge - original_edge, 0.000002, candidate["name"])


class BoardAvailabilityTests(unittest.TestCase):
    def test_no_active_board_stops_before_reading_candidates(self):
        with patch.object(pcbnew, "GetBoard", return_value=None), \
                patch.object(tapers, "read_candidates") as read_candidates, \
                patch("sys.stdout", new_callable=io.StringIO) as output:
            self.assertEqual(tapers.run(apply=True), 0)
        read_candidates.assert_not_called()
        self.assertIn("No active PCB board", output.getvalue())
        self.assertIn("Nothing changed", output.getvalue())

    def test_explicit_board_does_not_require_active_editor(self):
        board = pcbnew.BOARD()
        with patch.object(pcbnew, "GetBoard") as get_board, \
                patch("sys.stdout", new_callable=io.StringIO):
            self.assertEqual(tapers.run(board=board, apply=False), 0)
        get_board.assert_not_called()


class CapacitorTeardropTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.board = pcbnew.LoadBoard(str(ROOT / "XG_Mobile_Dock_Retimer.kicad_pcb"))

    def setUp(self):
        tapers._apply_overrides(dict(PAD_ESCAPE=False, CONSTANT_WIDTH=False,
                                    GAP_WIDTH_MODE=False, GAP_START=None, FOLLOW_ROUTE=False,
                                    GAP_FULL_WIDTH=0.4, GAP_MAX_MULTIPLIER=1.5,
                                    AUTO_LENGTH=False, GROWTH_BIAS=1.0,
                                    CURVE_HANDLE_RATIO=1.0 / 3.0, ALIGN_PAD_TRACK=False,
                                    EXTEND_PATH=False, TARGET_WIDTH=None,
                                    SHIFT_PAD_FANIN=False, PAD_NECK=0.0,
                                    LENGTH=0.45, STEPS=8, USE_SELECTION=True))
        self.select_entries(("C40", "C41"))

    def select_entries(self, references):
        pads = [next(pad for pad in self.board.FindFootprintByReference(reference).Pads()
                     if pad.GetNumber() == "1") for reference in references]
        for track in self.board.GetTracks():
            track.ClearSelected()
            if isinstance(track, pcbnew.PCB_VIA):
                continue
            for pad in pads:
                if (track.GetNetCode() == pad.GetNetCode() and
                        pad.HitTest(track.GetStart()) != pad.HitTest(track.GetEnd())):
                    track.SetSelected()
        self.candidates = tapers.read_candidates(self.board)

    def test_mirrored_fanins_are_paired(self):
        self.assertEqual(len(self.candidates), 2)
        self.assertEqual(tapers.pair_candidates(self.candidates), 1)

    def test_gap_width_threshold_and_cap(self):
        self.assertEqual(tapers._gap_multiplier(.1, .1), 1)
        self.assertAlmostEqual(tapers._gap_multiplier(.25, .1), 1.25)
        self.assertEqual(tapers._gap_multiplier(.4, .1), 1.5)
        self.assertEqual(tapers._gap_multiplier(.8, .1), 1.5)
        tapers.GAP_MAX_MULTIPLIER = 1.3
        tapers.GAP_FULL_WIDTH = .5
        self.assertEqual(tapers._gap_multiplier(.6, .1), 1.3)
        tapers.GAP_FULL_WIDTH = .1
        with self.assertRaises(ValueError):
            tapers._gap_multiplier(.2, .1)

    def test_gap_mode_requires_partner(self):
        tapers.GAP_WIDTH_MODE = True
        self.assertIsNone(tapers.plan_candidate(self.candidates[0]))

    def test_gap_mode_preserves_local_edges(self):
        tapers.GAP_WIDTH_MODE = True
        self.test_facing_edges_do_not_encroach_locally()
        tapers.pair_candidates(self.candidates)
        plans = [tapers.plan_candidate(candidate) for candidate in self.candidates]
        self.assertTrue(all(plans))
        self.assertEqual(len(tapers.verify_pair_gaps(plans)), 16)
        for plan in plans:
            self.assertAlmostEqual(plan["widths"][0], plan["width0"])
            self.assertGreater(max(plan["widths"]), plan["width0"])
            self.assertLessEqual(max(plan["widths"]), plan["width0"] * 1.5 + 1e-9)
            for gap, width in zip(plan["reference_gaps"], plan["widths"]):
                self.assertAlmostEqual(width, plan["width0"] * tapers._gap_multiplier(gap, plan["reference_gaps"][0]))

    def test_growth_bias_delays_width(self):
        original = tapers._growth_profile(0.25)
        tapers.GROWTH_BIAS = 1.5
        self.assertLess(tapers._growth_profile(0.25), original)
        self.assertEqual(tapers._growth_profile(0), 0)
        self.assertEqual(tapers._growth_profile(1), 1)
        tapers.GROWTH_BIAS = 0
        with self.assertRaises(ValueError):
            tapers._growth_profile(0.5)

    def test_growth_bias_delays_pair_separation(self):
        results = []
        for bias in (1.0, 1.5, 2.0):
            tapers.GROWTH_BIAS = bias
            self.select_entries(("C40", "C41"))
            tapers.pair_candidates(self.candidates)
            plans = [tapers.plan_candidate(candidate) for candidate in self.candidates]
            self.assertTrue(all(plans))
            self.assertEqual(len(tapers.verify_pair_gaps(plans)), 2)
            results.append((plans[0]["widths"][16],
                            tapers.norm(tapers.sub(plans[0]["nodes"][16], plans[1]["nodes"][16]))))
        for earlier, later in zip(results, results[1:]):
            self.assertLess(later[0], earlier[0])
            self.assertLess(later[1], earlier[1])

    def test_auto_length_biased_facing_edges(self):
        tapers.GROWTH_BIAS = 1.5
        tapers.AUTO_LENGTH = True
        self.test_facing_edges_do_not_encroach_locally()

    def test_auto_length_scales_with_pad(self):
        footprint = pcbnew.FOOTPRINT(self.board)
        pad = pcbnew.PAD(footprint)
        pad.SetAttribute(pcbnew.PAD_ATTRIB_SMD)
        pad.SetDrillSize(tapers.v2((0, 0)))
        pad.SetShape(pcbnew.PAD_SHAPE_RECT)
        pad.SetPosition(tapers.v2((10, 10)))
        candidate = dict(pad=pad, direction=(0, 1))
        tapers.AUTO_LENGTH = True
        for width, expected in ((0.2, 0.16), (0.5, 0.4)):
            pad.SetSize(tapers.v2((width, 0.4)))
            self.assertAlmostEqual(tapers._requested_length(candidate), expected, places=5)
        tapers.AUTO_LENGTH = False
        self.assertEqual(tapers._requested_length(candidate), tapers.LENGTH)

    def test_default_uses_continuous_pad_sized_outline(self):
        tapers.pair_candidates(self.candidates)
        plans = [tapers.plan_candidate(candidate) for candidate in self.candidates]
        for plan in plans:
            self.assertIsNotNone(plan)
            self.assertTrue("polygon" in plan)
            self.assertGreaterEqual(len(plan["polygon"]), 66)
            self.assertGreater(plan["width1"], 2 * plan["width0"])
            self.assertTrue(tapers._simple_polygon(plan["polygon"]))
            half = len(plan["polygon"]) // 2
            for point in (plan["polygon"][half - 1], plan["polygon"][half]):
                self.assertTrue(tapers._pad_hit(plan["candidate"]["pad"], point))
            self.assertTrue(tapers._pad_hit(plan["candidate"]["pad"], plan["end"]))
        self.assertEqual(len(tapers.verify_pair_gaps(plans)), 2)

    def test_all_hso_pairs(self):
        self.select_entries(f"C{index}" for index in range(38, 54))
        self.assertEqual(len(self.candidates), 16)
        self.assertEqual(tapers.pair_candidates(self.candidates), 8)
        plans = [tapers.plan_candidate(candidate) for candidate in self.candidates]
        self.assertTrue(all(plans))
        self.assertEqual(len(tapers.verify_pair_gaps(plans)), 16)

    def test_facing_edges_do_not_encroach_locally(self):
        self.select_entries(f"C{index}" for index in range(38, 54))
        tapers.pair_candidates(self.candidates)
        for candidate in self.candidates:
            plan = tapers.plan_candidate(candidate)
            toward = tapers.unit(tapers.sub(
                tapers.p2(candidate["partner"]["pad"].GetPosition()),
                tapers.p2(candidate["pad"].GetPosition())))
            along = (-toward[1], toward[0])
            original = pcbnew.SHAPE_POLY_SET()
            items = [candidate["pad"]] + [component["obj"] for component in candidate["components"]]
            for item in items:
                item.TransformShapeToPolygon(original, candidate["layer"], 0, 1, pcbnew.ERROR_OUTSIDE)
            outlines = [[tapers.p2(original.Outline(index).CPoint(vertex))
                         for vertex in range(original.Outline(index).PointCount())]
                        for index in range(original.OutlineCount())]

            def facing(polygons, depth):
                hits = []
                for polygon in polygons:
                    for start, end in tapers._polygon_edges(polygon):
                        first = tapers.dot(start, along)
                        last = tapers.dot(end, along)
                        if min(first, last) <= depth <= max(first, last) and abs(last-first) > 1e-12:
                            point = tapers.add(start, tapers.mul(tapers.sub(end, start), (depth-first)/(last-first)))
                            hits.append(tapers.dot(point, toward))
                return max(hits) if hits else None

            depths = [tapers.dot(point, along) for point in plan["polygon"]]
            encroachment = 0.0
            for index in range(1, 1000):
                depth = min(depths) + (max(depths)-min(depths))*index/1000
                old_edge = facing(outlines, depth)
                new_edge = facing([plan["polygon"]], depth)
                if old_edge is not None and new_edge is not None:
                    encroachment = max(encroachment, new_edge-old_edge)
            self.assertLessEqual(encroachment, 0.000002, candidate["name"])

    def test_curved_entries_fail_closed_or_preserve_gap(self):
        tapers.LENGTH = 0.55
        self.select_entries(f"C{index}" for index in range(54, 70))
        self.assertEqual(len(self.candidates), 16)
        self.assertEqual(tapers.pair_candidates(self.candidates), 8)
        plans = [plan for plan in map(tapers.plan_candidate, self.candidates) if plan]
        self.assertGreater(len(plans), 0)
        accepted = tapers.verify_pair_gaps(plans)
        self.assertGreater(len(accepted), 0)
        accepted_candidates = {id(plan["candidate"]) for plan in accepted}
        for plan in accepted:
            self.assertTrue(tapers._simple_polygon(plan["polygon"]))
            self.assertIn(id(plan["candidate"]["partner"]), accepted_candidates)

    def test_constant_width_mode_still_uses_tracks(self):
        tapers.CONSTANT_WIDTH = True
        tapers.TARGET_WIDTH = 0.13
        plan = tapers.plan_candidate(self.candidates[0])
        self.assertIsNotNone(plan)
        self.assertNotIn("polygon", plan)
        for piece in plan["pieces"]:
            self.assertIn(piece[2], (plan["width0"], plan["width1"]))

    def test_follow_route_curved_capacitor_entries(self):
        tapers.FOLLOW_ROUTE = True
        tapers.GAP_WIDTH_MODE = True
        tapers.LENGTH = 0.55
        self.select_entries(f"C{index}" for index in range(54, 70))
        self.assertEqual(tapers.pair_candidates(self.candidates), 8)
        plans = [plan for plan in map(tapers.plan_candidate, self.candidates) if plan]
        accepted = tapers.verify_pair_gaps(plans)
        self.assertGreater(len(accepted), 0)
        paired = {id(plan["candidate"]) for plan in accepted}
        for plan in accepted:
            candidate = plan["candidate"]
            self.assertEqual(candidate["terminal_kind"], "arc")
            self.assertIn(id(candidate["partner"]), paired)
            assert_facing_edge_preserved(self, candidate, plan["polygon"])
            for point, distance in zip(plan["route_nodes"], plan["route_distances"]):
                if distance <= candidate["path_end"]:
                    self.assertLess(tapers.norm(tapers.sub(point, candidate["point_at"](distance))), 1e-9)

    def test_polygon_validation_and_containment(self):
        self.assertFalse(tapers._simple_polygon([(0, 0), (1, 1), (0, 1), (1, 0)]))
        outer = [(0, 0), (2, 0), (2, 2), (0, 2)]
        inner = [(0.5, 0.5), (1, 0.5), (1, 1), (0.5, 1)]
        self.assertEqual(tapers._polygon_gap(outer, inner), 0)


class ConnectorTeardropTests(unittest.TestCase):
    def check_connector(self, follow_route=False):
        board = pcbnew.LoadBoard(str(ROOT / "XG_Mobile_Dock_Retimer.kicad_pcb"))
        tapers._apply_overrides(dict(PAD_ESCAPE=False, CONSTANT_WIDTH=False,
                                    GAP_WIDTH_MODE=True, GAP_START=None,
                                    GAP_FULL_WIDTH=0.4, GAP_MAX_MULTIPLIER=2.0,
                                    AUTO_LENGTH=False, GROWTH_BIAS=1.0,
                                    CURVE_HANDLE_RATIO=0.65, ALIGN_PAD_TRACK=True,
                                    FOLLOW_ROUTE=follow_route,
                                    EXTEND_PATH=False, TARGET_WIDTH=None,
                                    SHIFT_PAD_FANIN=False, PAD_NECK=0.0,
                                    LENGTH=0.45, STEPS=8, USE_SELECTION=True))
        pads = [pad for pad in board.FindFootprintByReference("J7").Pads()
                if pad.GetNetname().startswith("/HSO")]
        pad_geometry = [(tapers.p2(pad.GetPosition()), tapers.p2(pad.GetSize()), pad.GetOrientationDegrees())
                for pad in pads]
        for track in board.GetTracks():
            track.ClearSelected()
            if isinstance(track, pcbnew.PCB_VIA):
                continue
            if any(track.GetNetCode() == pad.GetNetCode() and pad.IsOnLayer(track.GetLayer())
                   and pad.HitTest(track.GetStart()) != pad.HitTest(track.GetEnd()) for pad in pads):
                track.SetSelected()
        candidates = tapers.read_candidates(board)
        continuations = [(candidate["downstream"][0], candidate["pad"], candidate["layer"])
                         for candidate in candidates if candidate.get("downstream")]
        self.assertEqual(len(candidates), 16)
        self.assertEqual(len(continuations), 16)
        self.assertEqual(tapers.pair_candidates(candidates), 8)
        plans = [tapers.plan_candidate(candidate) for candidate in candidates]
        self.assertTrue(all(plans))
        self.assertEqual(len(tapers.verify_pair_gaps(plans)), 16)
        for plan in plans:
            if not follow_route:
                self.assertAlmostEqual(max(plan["widths"]), 0.227, places=5)
            self.assertAlmostEqual(plan["widths"][0], plan["width0"])
            if follow_route:
                candidate = plan["candidate"]
                assert_facing_edge_preserved(self, candidate, plan["polygon"])
                for point, distance in zip(plan["route_nodes"], plan["route_distances"]):
                    if distance <= candidate["path_end"]:
                        self.assertLess(tapers.norm(tapers.sub(point, candidate["point_at"](distance))), 1e-9)
                self.assertTrue(any(component["kind"] == "arc" for component in candidate["components"]))
                self.assertTrue(all(later >= earlier - 1e-6
                                    for earlier, later in zip(plan["widths"], plan["widths"][1:])))
                entry = candidate["path_end"]
                entry_index = plan["route_distances"].index(entry)
                entry_width = plan["widths"][entry_index]
                self.assertAlmostEqual(entry_width, plan["width0"] * tapers._gap_multiplier(
                    plan["reference_gaps"][entry_index], plan["reference_gaps"][0]))
                self.assertGreater(entry_width, plan["width0"])
                self.assertLessEqual(entry_width, 0.227)
                for distance, gap, width in zip(plan["route_distances"], plan["reference_gaps"], plan["widths"]):
                    if distance < entry:
                        start_gap = plan["reference_gaps"][0]
                        entry_gap = plan["reference_gaps"][entry_index]
                        progress = max(0.0, min(1.0, (gap - start_gap) / (entry_gap - start_gap)))
                        expected = plan["width0"] + (entry_width - plan["width0"]) * progress * (2.0 - progress)
                        self.assertAlmostEqual(width, expected)
                        self.assertLess(width, entry_width)
                        if 0 < progress < 0.1:
                            self.assertGreaterEqual((width - plan["width0"]) / (entry_width - plan["width0"]),
                                                    1.8 * progress)
                    else:
                        self.assertAlmostEqual(width, entry_width)
                continue
            track, at_start = plan["candidate"]["downstream"]
            direction = tapers.unit(tapers.sub(
                tapers.p2(track.GetEnd() if at_start else track.GetStart()),
                tapers.p2(track.GetStart() if at_start else track.GetEnd())))
            final_direction = tapers.unit(tapers.sub(plan["nodes"][-1], plan["nodes"][-2]))
            self.assertGreater(tapers.dot(direction, final_direction), 0.99)
        self.assertEqual(tapers.run(board, apply=True), 16)
        self.assertEqual(pad_geometry, [(tapers.p2(pad.GetPosition()), tapers.p2(pad.GetSize()),
                        pad.GetOrientationDegrees()) for pad in pads])
        remaining = {tapers._item_key(track) for track in board.GetTracks()}
        for track, pad, layer in continuations:
            if tapers._item_key(track) not in remaining:
                continue
            copper = pcbnew.SHAPE_POLY_SET()
            pad_copper = pcbnew.SHAPE_POLY_SET()
            track.TransformShapeToPolygon(copper, layer, 0, 1, pcbnew.ERROR_OUTSIDE)
            pad.TransformShapeToPolygon(pad_copper, layer, 0, 1, pcbnew.ERROR_INSIDE)
            copper.BooleanSubtract(pad_copper)
            self.assertEqual(copper.OutlineCount(), 0, pad.GetNumber())

    def test_under_pad_continuations_do_not_protrude_after_apply(self):
        self.check_connector()

    def test_follow_route_preserves_fillets(self):
        self.check_connector(follow_route=True)


if __name__ == "__main__":
    unittest.main()
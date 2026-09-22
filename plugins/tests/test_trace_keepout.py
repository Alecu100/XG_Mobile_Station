import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

import pcbnew


SPEC = importlib.util.spec_from_file_location(
    "trace_keepout", Path(__file__).resolve().parents[1] / "trace_keepout.py")
keepout = importlib.util.module_from_spec(SPEC)
with patch.object(pcbnew.ActionPlugin, "register"):
    SPEC.loader.exec_module(keepout)


class TraceKeepoutTests(unittest.TestCase):
    def check_zone(self, block_pour):
        board = pcbnew.BOARD()
        track = pcbnew.PCB_TRACK(board)
        track.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(10), pcbnew.FromMM(10)))
        track.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(12), pcbnew.FromMM(10)))
        track.SetWidth(pcbnew.FromMM(0.2))
        track.SetLayer(pcbnew.F_Cu)
        board.Add(track)
        track.SetSelected()
        with patch.object(keepout, "_refresh"):
            self.assertEqual(keepout.run(board, apply=True, BLOCK_POUR=block_pour, FILL=False), 1)
        zones = list(board.Zones())
        self.assertEqual(len(zones), 1)
        zone = zones[0]
        getter = getattr(zone, "GetDoNotAllowZoneFills", None)
        if getter is None:
            getter = zone.GetDoNotAllowCopperPour
        self.assertEqual(getter(), block_pour)
        self.assertTrue(zone.GetIsRuleArea())
        self.assertEqual(zone.GetLayer(), pcbnew.F_Cu)
        self.assertGreater(zone.Outline().OutlineCount(), 0)
        self.assertFalse(zone.GetDoNotAllowTracks())
        self.assertFalse(zone.GetDoNotAllowVias())
        self.assertFalse(zone.GetDoNotAllowPads())
        self.assertFalse(zone.GetDoNotAllowFootprints())

    def test_installed_api(self):
        for enabled in (True, False):
            with self.subTest(enabled=enabled):
                self.check_zone(enabled)

    @unittest.skipUnless(hasattr(pcbnew.ZONE, "SetDoNotAllowCopperPour"),
                         "KiCad 9 needed to simulate the renamed KiCad 10 setter")
    def test_renamed_api_without_legacy_setter(self):
        setter = pcbnew.ZONE.SetDoNotAllowCopperPour
        with patch.object(pcbnew.ZONE, "SetDoNotAllowZoneFills", setter, create=True), \
                patch.object(pcbnew.ZONE, "SetDoNotAllowCopperPour", None):
            for enabled in (True, False):
                with self.subTest(enabled=enabled):
                    self.check_zone(enabled)


if __name__ == "__main__":
    unittest.main()
"""Solver-free unit tests for RAPID's geometry, airfoil assignment, stability
math and config handling. Run with:  python -m unittest discover -s tests
"""

import json
import math
import os
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "system_files"))
os.environ["RAPID_CONFIG_FILE"] = "<built-in defaults>"
os.environ.setdefault("RAPID_OUTPUT_DIR", tempfile.mkdtemp(prefix="rapid_test_out_"))

import geometry  # noqa: E402
import airfoil_opti  # noqa: E402
import rapid_config  # noqa: E402

X = [9.0, 0.4, 0.8, 0.5, 4.0, 6.0]
S_REF = 210.0


def half_area(stations):
    return sum(0.5 * (a["chord"] + b["chord"]) * (b["y"] - a["y"]) for a, b in zip(stations[:-1], stations[1:]))


class GeometryTests(unittest.TestCase):
    def test_area_matches_reference(self):
        for n in (3, 5, 9):
            st = geometry.planform_stations(X, S_REF, n, 3.0)
            self.assertAlmostEqual(2 * half_area(st), S_REF, places=6)

    def test_stations_keep_root_kink_tip(self):
        c_root, c_kink, c_tip, b_half, b_kink, _ = geometry.chord_from_geometry(*X[:4], S_REF)
        st = geometry.planform_stations(X, S_REF, 7, 3.0)
        self.assertEqual(len(st), 7)
        self.assertEqual([s["zone"] for s in st].count("KINK"), 1)
        kink = next(s for s in st if s["zone"] == "KINK")
        self.assertAlmostEqual(kink["y"], b_kink)
        self.assertAlmostEqual(kink["chord"], c_kink)
        self.assertAlmostEqual(st[0]["chord"], c_root)
        self.assertAlmostEqual(st[-1]["chord"], c_tip)
        self.assertAlmostEqual(st[-1]["y"], b_half)
        self.assertAlmostEqual(st[-1]["twist"], -3.0)

    def test_mac_matches_closed_form(self):
        mac_closed = geometry.chord_from_geometry(*X[:4], S_REF)[5]
        for n in (3, 6):
            mac, _ = geometry.mac_reference(geometry.planform_stations(X, S_REF, n, 3.0))
            self.assertAlmostEqual(mac, mac_closed, places=9)

    def test_mac_le_single_trapezoid(self):
        # Classic result: x_LE,MAC = (b/2)/3 * (1+2*lambda)/(1+lambda) * tan(sweep_LE)
        cr, ct, b2, sw = 4.0, 2.0, 10.0, 20.0
        t = math.tan(math.radians(sw))
        st = [{"y": 0.0, "chord": cr, "x_le": 0.0}, {"y": b2, "chord": ct, "x_le": b2 * t}]
        lam = ct / cr
        _, x_le = geometry.mac_reference(st)
        self.assertAlmostEqual(x_le, b2 / 3 * (1 + 2 * lam) / (1 + lam) * t, places=9)

    def test_panels_continuous(self):
        st = geometry.planform_stations(X, S_REF, 5, 3.0)
        panels = geometry.panels_from_stations(st, (X[4], X[5]))
        self.assertEqual(len(panels), 4)
        for a, b in zip(panels[:-1], panels[1:]):
            self.assertAlmostEqual(a["tip_chord"], b["root_chord"])
        self.assertAlmostEqual(sum(p["span"] for p in panels), st[-1]["y"])
        self.assertEqual({p["sweep"] for p in panels}, {X[4], X[5]})

    def test_infeasible_root_chord_rejected(self):
        self.assertIsNone(geometry.chord_from_geometry(*X[:4], 1.0))           # half-span < fuselage width
        self.assertIsNone(geometry.chord_from_geometry(*X[:4], 600.0))         # root chord above C_ROOT_MAX
        cfg = {"FUSELAGE_Y": 0.1, "HALF_SPAN_MAX": 5.0, "C_ROOT_MIN": 0.1, "C_ROOT_MAX": 2.0}
        self.assertIsNotNone(geometry.chord_from_geometry(*X[:4], 3.0, cfg))

    def test_spanwise_tess(self):
        st = geometry.planform_stations(X, S_REF, 3, 3.0)
        tess = geometry.spanwise_tess(geometry.panels_from_stations(st, (0, 0)), 24)
        self.assertEqual(len(tess), 2)
        self.assertTrue(all(t >= 3 for t in tess))


class AirfoilAssignmentTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def _write(self, data):
        p = os.path.join(self.tmp, "aa.json")
        with open(p, "w") as f:
            json.dump(data, f)
        return p

    def test_missing_file_defaults_to_baseline(self):
        active = airfoil_opti.load_active_airfoils(os.path.join(self.tmp, "nope.json"))
        self.assertFalse(airfoil_opti.is_custom(active))

    def test_legacy_format_maps_kink(self):
        active = airfoil_opti.load_active_airfoils(self._write({"root": "naca0012", "kink": "naca4412", "tip": "naca2412"}))
        st = geometry.planform_stations(X, S_REF, 3, 3.0)
        self.assertEqual(airfoil_opti.assign_airfoils(st, active, X[1]), ["naca0012", "naca4412", "naca2412"])

    def test_nearest_station(self):
        active = [{"eta": 0.0, "airfoil": "a"}, {"eta": 0.5, "airfoil": "b"}, {"eta": 1.0, "airfoil": "c"}]
        st = [{"eta": e} for e in (0.0, 0.2, 0.3, 0.74, 0.76, 1.0)]
        self.assertEqual(airfoil_opti.assign_airfoils(st, active, 0.4), ["a", "a", "b", "b", "c", "c"])

    def test_paths_saved_project_relative(self):
        dat = os.path.join(airfoil_opti.AIRFOIL_DB, "Reflexed", "mh60.dat")
        p = os.path.join(self.tmp, "aa.json")
        airfoil_opti.save_active_airfoils([{"eta": 0.0, "airfoil": dat}], p)
        with open(p) as f:
            self.assertFalse(os.path.isabs(json.load(f)["stations"][0]["airfoil"]))
        self.assertEqual(os.path.normcase(airfoil_opti.load_active_airfoils(p)[0]["airfoil"]), os.path.normcase(dat))

    def test_thickness(self):
        t = airfoil_opti.airfoil_thickness(os.path.join(airfoil_opti.AIRFOIL_DB, "Reflexed", "mh60.dat"))
        self.assertAlmostEqual(t, 0.101, delta=0.005)


class StallCharacterTests(unittest.TestCase):
    def _polar(self, cls):
        return {a: {"CL": c} for a, c in zip(range(6, 6 + len(cls)), cls)}

    def test_gentle_stall(self):
        # CLmax 1.5 at alpha 14; 2 deg later 1.47 -> 2 % loss
        drop, a, cl = airfoil_opti.stall_drop(self._polar([0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.45, 1.48, 1.5, 1.49, 1.47]), 2.0)
        self.assertEqual((a, cl), (14, 1.5))
        self.assertAlmostEqual(drop, 0.02)

    def test_abrupt_stall(self):
        drop, _, _ = airfoil_opti.stall_drop(self._polar([0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.2, 1.0]), 2.0)
        self.assertAlmostEqual(drop, (1.5 - 1.0) / 1.5)

    def test_fractional_delta_interpolates(self):
        drop, _, _ = airfoil_opti.stall_drop(self._polar([1.0, 1.2, 1.4, 1.2, 1.0]), 1.5)
        self.assertAlmostEqual(drop, (1.4 - 1.1) / 1.4)

    def test_unknown_without_post_stall_points(self):
        self.assertIsNone(airfoil_opti.stall_drop(self._polar([0.9, 1.0, 1.1, 1.2, 1.3]), 2.0)[0])
        self.assertIsNone(airfoil_opti.stall_drop({}, 2.0)[0])


class StabilityTests(unittest.TestCase):
    def test_neutral_point_and_trim_transfer(self):
        import planform_opti as po
        c, x_ref, x_np = 5.0, 2.0, 2.6            # neutral point 0.12 MAC aft of the reference
        cm0 = 0.01
        rows = [{"CL": cl, "CM": cm0 - cl * (x_np - x_ref) / c} for cl in (0.3, 0.5)]
        s = po.longitudinal_stability(rows, x_ref, c)
        self.assertAlmostEqual(s["x_np"], x_np, places=9)
        if po.CG_X is None:
            self.assertAlmostEqual(s["SM"], 0.5 * (po.SM_MIN + po.SM_MAX), places=9)
        # Cm about the CG equals the direct evaluation.
        expected = rows[0]["CM"] + rows[0]["CL"] * (s["x_cg"] - x_ref) / c
        self.assertAlmostEqual(s["CM_trim"], expected, places=12)


class ConfigTests(unittest.TestCase):
    def test_every_editor_field_has_a_default(self):
        for section, key, *_ in rapid_config.PARAM_META:
            self.assertIn(key, rapid_config.DEFAULTS[section])

    def test_bwb_preset_loads_and_omits_cm_max(self):
        path = os.path.join(ROOT, "system_files", "bwb_config.json")
        raw = rapid_config.load_raw_config(path)
        self.assertNotIn("CM_MAX", raw["constraints"])
        cfg = rapid_config.load_effective_config(path)
        self.assertTrue(cfg["bwb"]["BWB_MODE"])
        self.assertEqual(cfg["mesh"], rapid_config.DEFAULTS["mesh"])


if __name__ == "__main__":
    unittest.main()

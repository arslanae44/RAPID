"""Solver-free tests for the transonic layer (transonic.py). Run with:
    python -m unittest discover -s tests
"""

import math
import os
import sys
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "system_files"))
os.environ["RAPID_CONFIG_FILE"] = "<built-in defaults>"

import transonic as T  # noqa: E402
from rapid_config import load_effective_config  # noqa: E402

TC = load_effective_config()["transonic"]
CHART = {"AR": 5.25, "SWEEP_LE": 30.0, "TAPER": 0.30, "WASHOUT": 3.0, "TC_ROOT": 0.12, "TC_TIP": 0.10}


class WaveDragTests(unittest.TestCase):
    def test_korn_unswept(self):
        # M_dd = kappa - t/c - cl/10 for an unswept wing
        self.assertAlmostEqual(float(T.korn_mdd(0.5, 0.10, 0.0, 0.87)), 0.87 - 0.10 - 0.05)

    def test_korn_swept_value(self):
        c = math.cos(math.radians(25.0))
        want = 0.87 / c - 0.11 / c ** 2 - 0.3 / (10 * c ** 3)
        self.assertAlmostEqual(float(T.korn_mdd(0.3, 0.11, 25.0, 0.87)), want)

    def test_mcrit_offset_and_lock(self):
        mdd, mcrit, cdw = T.wave_drag_strips(0.3, 0.1, 20.0, 0.75, 0.87)
        self.assertAlmostEqual(float(mdd - mcrit), (0.1 / 80.0) ** (1 / 3))
        self.assertAlmostEqual(float(cdw), 20.0 * (0.75 - float(mcrit)) ** 4)

    def test_no_wave_drag_below_mcrit(self):
        self.assertEqual(float(T.lock_cdw(0.6, 0.65)), 0.0)

    def test_sweep_reduces_wave_drag(self):
        low = T.wave_drag_strips(0.3, 0.11, 10.0, 0.75, 0.87)[2]
        high = T.wave_drag_strips(0.3, 0.11, 30.0, 0.75, 0.87)[2]
        self.assertGreater(float(low), float(high))

    def test_thicker_section_lowers_mdd(self):
        self.assertGreater(float(T.korn_mdd(0.3, 0.09, 25.0, 0.87)), float(T.korn_mdd(0.3, 0.12, 25.0, 0.87)))


class GeometryTests(unittest.TestCase):
    def test_chart_wing_matches_study_report(self):
        g = T.planform(CHART, TC["S_REF"])
        self.assertAlmostEqual(g["b"], 9.37, places=2)
        self.assertAlmostEqual(g["cr"], 2.75, places=2)
        self.assertAlmostEqual(g["ct"], 0.82, places=2)
        self.assertAlmostEqual(g["sweep_c4"], 25.4, places=1)          # report: 25.4 deg
        self.assertAlmostEqual(g["mac"], 1.96, places=2)               # report: 1.96 m

    def test_area_from_chords(self):
        g = T.planform(CHART, TC["S_REF"])
        self.assertAlmostEqual(g["b"] * (g["cr"] + g["ct"]) / 2.0, TC["S_REF"], places=9)

    def test_sweep_lines_consistent(self):
        self.assertAlmostEqual(T.sweep_at(0.0, 30.0, 0.3, 5.25), 30.0)
        self.assertGreater(T.sweep_at(0.25, 30.0, 0.3, 5.25), T.sweep_at(0.5, 30.0, 0.3, 5.25))

    def test_cruise_condition_reproduces_design_cl(self):
        # weight at start of cruise / (q S) = the design lift coefficient quoted in the brief
        q = T.cruise_state(TC)["q"]
        w = 0.955 * 10060.0 * T.LB_TO_KG * 9.80665
        self.assertAlmostEqual(w / (q * TC["S_REF"]), TC["CL_TARGET"], delta=0.002)


class WeightFuelTests(unittest.TestCase):
    def test_raymer_chart_wing_weight(self):
        w = T.raymer_wing_weight_lb(TC["S_REF"], 5.25, 25.4, 0.30, 0.12, TC)
        self.assertAlmostEqual(w, 962.0, delta=3.0)

    def test_weight_trends(self):
        base = T.raymer_wing_weight_lb(TC["S_REF"], 5.25, 25.0, 0.3, 0.12, TC)
        self.assertGreater(T.raymer_wing_weight_lb(TC["S_REF"], 6.0, 25.0, 0.3, 0.12, TC), base)   # higher AR
        self.assertGreater(T.raymer_wing_weight_lb(TC["S_REF"], 5.25, 35.0, 0.3, 0.12, TC), base)  # more sweep
        self.assertGreater(T.raymer_wing_weight_lb(TC["S_REF"], 5.25, 25.0, 0.3, 0.09, TC), base)  # thinner root

    def test_fuel_volume(self):
        g = T.planform(CHART, TC["S_REF"])
        v = T.wing_fuel_volume_m3(TC["S_REF"], g["b"], 0.3, 0.12, 0.10)
        self.assertAlmostEqual(v, 1.545, delta=0.01)
        self.assertGreater(v, T.wing_fuel_volume_m3(TC["S_REF"], g["b"], 0.3, 0.10, 0.08))

    def test_breguet_trade(self):
        # heavier wing displaces fuel -> lower range factor at equal L/D
        a = T.breguet_range_factor(18.0, 962.0, 962.0, TC)
        b = T.breguet_range_factor(18.0, 1062.0, 962.0, TC)
        self.assertLess(b, a)


class PitchupAndStallTests(unittest.TestCase):
    def test_pitchup_interpolation(self):
        self.assertAlmostEqual(T.pitchup_ar_limit(25.0, TC), 6.5)
        self.assertGreater(T.pitchup_ar_limit(10.0, TC), T.pitchup_ar_limit(40.0, TC))

    def test_stall_onset_first_strip_and_ratio(self):
        eta = np.linspace(0.05, 0.95, 10)
        # strip loads: clearly highest outboard -> tip stall first
        a = 0.2 + 0.2 * eta
        b = 0.6 + 0.6 * eta
        alpha_s, r, eta_first = T.stall_onset(eta, a, b, 1.0, 3.0, 1.6, 1.6)
        self.assertAlmostEqual(float(r.max()), 1.0, places=9)
        self.assertAlmostEqual(eta_first, eta[-1])
        # reversed distribution -> root first
        _, r2, eta_first2 = T.stall_onset(eta, 0.6 - 0.2 * eta, 1.2 - 0.6 * eta, 1.0, 3.0, 1.6, 1.6)
        self.assertAlmostEqual(eta_first2, eta[0])

    def test_lower_tip_clmax_moves_stall_outboard(self):
        eta = np.linspace(0.05, 0.95, 10)
        a, b = 0.3 + 0 * eta, 0.9 + 0 * eta                  # uniform loading
        _, _, ef_hi = T.stall_onset(eta, a, b, 1.0, 3.0, 1.79, 1.72)
        _, _, ef_lo = T.stall_onset(eta, a, b, 1.0, 3.0, 1.79, 1.50)
        self.assertEqual(ef_hi, ef_lo)                       # both tip-first when tip clmax is lower
        self.assertAlmostEqual(ef_lo, eta[-1])


class InducedDragTests(unittest.TestCase):
    def test_recovers_quadratic_with_offset(self):
        k, cl0 = 0.07, 0.05
        f = lambda cl: k * (cl - cl0) ** 2                     # noqa: E731
        r1, r2 = {"CL": 0.2, "CDi": f(0.2)}, {"CL": 0.5, "CDi": f(0.5)}
        self.assertAlmostEqual(T.induced_drag(r1, r2, 0.272), f(0.272), places=12)

    def test_extrapolates_outside_bracket(self):
        f = lambda cl: 0.06 * cl ** 2                          # noqa: E731
        r1, r2 = {"CL": 0.3, "CDi": f(0.3)}, {"CL": 0.5, "CDi": f(0.5)}
        self.assertAlmostEqual(T.induced_drag(r1, r2, 0.2), f(0.2), places=12)


class PolarTests(unittest.TestCase):
    def test_polar_table_blend_and_form_factor(self):
        data = {"Re": 1e7, "root": {"tc": 0.12, "fit": [0.007, 0.0, 0.01]}, "tip": {"tc": 0.10, "fit": [0.006, 0.0, 0.01]}}
        p = T.PolarTable(data)
        self.assertAlmostEqual(float(p.cd(0.3, 0.0)), 0.007 + 0.01 * 0.09)
        self.assertAlmostEqual(float(p.cd(0.3, 1.0)), 0.006 + 0.01 * 0.09)
        self.assertAlmostEqual(float(p.cd(0.3, 0.5)), 0.5 * (0.007 + 0.006) + 0.01 * 0.09)
        self.assertAlmostEqual(float(p.nominal_tc(0.5)), 0.11)
        self.assertGreater(float(T.hoerner_ff(0.12)), float(T.hoerner_ff(0.10)))

    def test_shipped_polars_are_sane(self):
        path = os.path.join(ROOT, TC["POLAR_FILE"])
        if not os.path.exists(path):
            self.skipTest("polars.json not built")
        p = T.PolarTable.load(path)
        for eta in (0.0, 1.0):
            self.assertTrue(0.004 < float(p.cd(0.272, eta)) < 0.010)


class ConstraintTests(unittest.TestCase):
    def _out(self, **kw):
        base = {"Mdd_min": 0.80, "AR_pitchup_limit": 6.0, "stall_eta_first": 0.5, "stall_out_ratio": 0.99,
                "fuel_share": 0.8, "CM": -0.05}
        base.update(kw)
        return base

    def test_feasible_design(self):
        g = T.constraint_values(self._out(), CHART, TC)
        self.assertEqual(len(g), len(T.CONSTRAINT_NAMES))
        self.assertTrue(all(v <= 0 for v in g))

    def test_each_constraint_can_fail(self):
        cases = {0: {"Mdd_min": 0.75}, 1: {"AR_pitchup_limit": 4.0}, 2: {"stall_eta_first": 0.8},
                 3: {"fuel_share": 0.2}, 4: {"CM": -0.5}}
        for i, kw in cases.items():
            g = T.constraint_values(self._out(**kw), CHART, TC)
            self.assertGreater(g[i], 0, T.CONSTRAINT_NAMES[i])
        g = T.constraint_values(self._out(), dict(CHART, TC_TIP=0.13), TC)
        self.assertGreater(g[5], 0)


if __name__ == "__main__":
    unittest.main()

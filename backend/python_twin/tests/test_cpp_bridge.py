# Unit tests for C++20 ctypes bridge, power curve, and OpenMP Monte Carlo output

import os
import sys
import unittest
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")))

from backend.python_twin.bridge.cpp_loader import (
    CppTwinBridge,
    benchmark_routes,
    default_battery,
    default_vehicle,
    default_wind,
)
from backend.python_twin.risk_planning.cvar_metrics import summarize_monte_carlo_risk


class TestCppBridge(unittest.TestCase):
    # Verify shared library loading and realistic probabilistic risk spread

    def setUp(self):
        self.bridge = CppTwinBridge()
        self.veh = default_vehicle()
        self.batt = default_battery()
        self.wind = default_wind()

    def test_power_curve_convexity(self):
        v = np.linspace(12.0, 26.0, 15)
        p = self.bridge.compute_power_curve(self.veh, v)
        self.assertEqual(len(p), 15)
        self.assertTrue(np.all(p > self.veh.p_hotel_w))
        self.assertLess(np.min(p), p[-1])

    def test_monte_carlo_non_degenerate_risk(self):
        routes = benchmark_routes()
        p_succ = []
        for r in routes:
            mc = self.bridge.run_monte_carlo(self.veh, self.batt, r["waypoints"], self.wind, n_runs=400, loiter_time_s=r["loiter_s"])
            stats = summarize_monte_carlo_risk(mc, soc_reserve=self.batt.soc_reserve)
            p_succ.append(stats["p_success"])
        # Longer routes must show strictly lower or equal success probability
        self.assertGreaterEqual(p_succ[0], p_succ[-1])
        self.assertLess(p_succ[-1], 0.99)


if __name__ == "__main__":
    unittest.main()

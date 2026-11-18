# Unit tests for 3D wind vector RTB reversal, crosswind crab penalty, and voltage sag

import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")))

from backend.python_twin.bridge.cpp_loader import (
    CppTwinBridge,
    WindConfigC,
    default_battery,
    default_vehicle,
)


class TestWindAndRtbPhysics(unittest.TestCase):
    # Verify fixes for legacy RTB opposite-wind and zero-voltage-sag bugs

    def setUp(self):
        self.bridge = CppTwinBridge()
        self.veh = default_vehicle()
        self.batt = default_battery()

    def test_reciprocal_rtb_wind_reversal(self):
        wps = [(0.0, 0.0, 150.0), (20000.0, 0.0, 150.0)]
        headwind = WindConfigC(-5.0, 0.0, 0.0, 30.0, 0.0, 0.0)
        tailwind = WindConfigC(5.0, 0.0, 0.0, 30.0, 0.0, 0.0)

        out_head = self.bridge.run_single_mission(self.veh, self.batt, wps, headwind, loiter_time_s=0.0, include_rtb=False)
        out_tail = self.bridge.run_single_mission(self.veh, self.batt, wps, tailwind, loiter_time_s=0.0, include_rtb=False)
        rtb_head = self.bridge.run_single_mission(self.veh, self.batt, wps, headwind, loiter_time_s=0.0, include_rtb=True)

        # Headwind outbound + tailwind return must equal sum of one-way headwind + tailwind times
        self.assertAlmostEqual(rtb_head["total_time_s"], out_head["total_time_s"] + out_tail["total_time_s"], delta=2.0)

    def test_aged_resistance_increases_voltage_sag(self):
        wps = [(0.0, 0.0, 150.0), (20000.0, 0.0, 150.0)]
        calm = WindConfigC(0.0, 0.0, 0.0, 30.0, 0.0, 0.0)
        nom = self.bridge.run_single_mission(self.veh, self.batt, wps, calm, r0_ohm=0.065, include_rtb=True)
        aged = self.bridge.run_single_mission(self.veh, self.batt, wps, calm, r0_ohm=0.160, include_rtb=True)
        self.assertLess(aged["min_v_term_v"], nom["min_v_term_v"])


if __name__ == "__main__":
    unittest.main()

# Unit tests for online C++ UKF assimilation of hidden drag and battery aging faults

import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")))

from backend.python_twin.bridge.cpp_loader import (
    CppTwinBridge,
    benchmark_routes,
    default_battery,
    default_vehicle,
    default_wind,
)
from backend.python_twin.simulation.telemetry_plant import PhysicalUAVPlant


class TestUkfAssimilation(unittest.TestCase):
    # Verify bidirectional twin tracks true SOC and converges onto hidden anomalies

    def test_online_drag_and_resistance_fault_detection(self):
        bridge = CppTwinBridge()
        veh = default_vehicle()
        batt = default_battery()
        wind = default_wind()
        route = benchmark_routes()[0]

        plant = PhysicalUAVPlant(veh, batt, route["waypoints"], wind, v_cruise_ms=18.0, seed=11)
        ukf = bridge.init_ukf(1.0, batt, wind)

        for step in range(80):
            if step == 20:
                plant.inject_fault("icing_drag", 0.010)
                plant.inject_fault("battery_aging", 0.045)
            pkt, truth = plant.step(dt_s=5.0)
            est = bridge.step_ukf(ukf, pkt, veh, batt)

        self.assertAlmostEqual(est["soc"], truth["true_soc"], delta=0.03)
        self.assertAlmostEqual(est["dcd0"], truth["true_dcd0"], delta=0.003)
        self.assertAlmostEqual(est["r0_ohm"], truth["true_r0_ohm"], delta=0.025)


if __name__ == "__main__":
    unittest.main()

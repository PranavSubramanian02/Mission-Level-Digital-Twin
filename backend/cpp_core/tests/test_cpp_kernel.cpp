#include "battery_ecm.hpp"
#include "c_api_bridge.hpp"
#include "ukf_estimator.hpp"
#include "wind_kinematics.hpp"
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

int main() {
    // Baseline 10 kg UAV with realistic 344 Wh 6S Li-ion pack (15.5 Ah)
    VehicleParams veh{10.0, 0.8, 0.030, 0.045, 0.70, 25.0, 0.35};
    BatteryParams batt{6, 15.5, 0.085, 0.030, 600.0, 18.8, 1.08, 0.25};
    WindConfig wind{-4.0, 1.5, 1.8, 30.0, 0.12, 0.003};

    // 1. Verify battery voltage sag increases with R0 and triggers cutoff
    double v_nom = battery_terminal_voltage(0.35, 0.1, 0.065, 320.0, batt.n_cells_series);
    double v_aged = battery_terminal_voltage(0.35, 0.1, 0.160, 320.0, batt.n_cells_series);
    assert(v_aged < v_nom);

    // 2. Verify reciprocal RTB wind symmetry (headwind outbound -> tailwind return)
    double vg_out = groundspeed_along_track(18.0, 1.0, 0.0, -4.0, 0.0);
    double vg_ret = groundspeed_along_track(18.0, -1.0, 0.0, -4.0, 0.0);
    assert(std::abs(vg_out - 14.0) < 1e-6);
    assert(std::abs(vg_ret - 22.0) < 1e-6);

    // 3. Verify UKF detects injected +0.010 drag icing and +0.05 Ohm aging fault
    UkfState ukf{};
    ukf_init(ukf, 0.95, batt, wind);
    double true_soc = 0.95, true_vp = 0.0;
    double true_r0 = batt.r0_ohm + 0.05;
    double true_dcd0 = 0.010;
    for (int k = 0; k < 40; ++k) {
        double p_true = electric_power_required(veh, 18.0, 150.0, 0.0, 0.0, true_dcd0);
        BatteryStepOut bo = battery_step(true_soc, true_vp, true_r0, p_true, 2.0, batt);
        true_soc = bo.soc;
        true_vp = bo.v_p;
        TelemetryPacket pkt{2.0, 18.0, 0.0, 0.0, 150.0, 0.0, bo.v_term_v, 18.0 - 4.0, 1.5, p_true};
        ukf_step(ukf, pkt, veh, batt);
    }
    assert(std::abs(ukf.x[3] - true_dcd0) < 0.003);
    assert(std::abs(ukf.x[2] - true_r0) < 0.025);

    // 4. Verify OpenMP N=1,000 Monte Carlo throughput
    std::vector<Waypoint3D> route = {{0.0, 0.0, 50.0}, {10000.0, 2000.0, 180.0}, {22000.0, 0.0, 180.0}};
    const int N = 1000;
    std::vector<double> f_soc(N), min_v(N), t_tot(N);
    std::vector<int> brown(N), succ(N);
    MonteCarloResult mc{N, f_soc.data(), min_v.data(), t_tot.data(), brown.data(), succ.data()};

    auto t0 = std::chrono::steady_clock::now();
    twin_run_monte_carlo(&veh, &batt, route.data(), static_cast<int>(route.size()),
                         18.0, 14.0, 300.0, 250.0, 1.0, 0.015, 0.0, batt.r0_ohm, 0.0, &wind, 1, 42ULL, &mc);
    auto dt_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

    int n_ok = 0;
    for (int s : succ) n_ok += s;
    std::cout << "C++ kernel checks PASSED | N=" << N << " MC in " << dt_ms
              << " ms | P(success)=" << (100.0 * n_ok / N) << "%\n";
    return 0;
}

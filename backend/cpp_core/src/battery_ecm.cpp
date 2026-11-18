#include "battery_ecm.hpp"
#include <algorithm>
#include <cmath>

double battery_ocv(double soc, int n_cells_series) {
    // Nonlinear Li-ion cell OCV curve scaled by series cell count
    double s = std::clamp(soc, 0.0, 1.0);
    double one_minus_s = 1.0 - s;
    double v_cell = 3.15 + 0.85 * s + 0.20 * (s * s * s) - 0.12 * std::pow(one_minus_s, 4);
    return v_cell * static_cast<double>(n_cells_series);
}

double battery_terminal_voltage(
    double soc,
    double v_p,
    double r0_ohm,
    double p_elec_w,
    int n_cells_series,
    double* out_current
) {
    // Solve R0 * I^2 - (V_oc - V_p) * I + P = 0 for discharge current
    double v_eff = battery_ocv(soc, n_cells_series) - v_p;
    double r0 = std::max(r0_ohm, 1e-4);
    double p = std::max(p_elec_w, 0.0);

    double disc = v_eff * v_eff - 4.0 * r0 * p;
    if (disc < 0.0 || v_eff <= 0.0) {
        if (out_current) *out_current = std::max(v_eff, 0.0) / (2.0 * r0);
        return 0.0;
    }

    double current = (v_eff - std::sqrt(disc)) / (2.0 * r0);
    if (out_current) *out_current = current;
    return v_eff - current * r0;
}

BatteryStepOut battery_step(
    double soc,
    double v_p,
    double r0_ohm,
    double p_elec_w,
    double dt_s,
    const BatteryParams& batt
) {
    double current = 0.0;
    double v_term = battery_terminal_voltage(soc, v_p, r0_ohm, p_elec_w, batt.n_cells_series, &current);

    // Traub Peukert C-rate capacity derating above 1C discharge
    double i_ref = std::max(batt.capacity_ah, 0.1);
    double i_eff = (current > i_ref) ? current * std::pow(current / i_ref, batt.peukert_k - 1.0) : current;
    double soc_next = std::clamp(soc - (i_eff * dt_s) / (3600.0 * batt.capacity_ah), 0.0, 1.0);

    // Exact exponential 1-RC polarization voltage update
    double tau = std::max(batt.r1_ohm * batt.c1_farad, 1e-3);
    double alpha = std::exp(-dt_s / tau);
    double vp_next = alpha * v_p + batt.r1_ohm * (1.0 - alpha) * current;

    bool brownout = (v_term <= batt.v_cutoff_v) || (soc_next <= 0.0);
    return {soc_next, vp_next, current, v_term, brownout};
}

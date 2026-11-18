#pragma once
#include "twin_types.hpp"

// Output snapshot from one electro-thermal battery step
struct BatteryStepOut {
    double soc;
    double v_p;
    double current_a;
    double v_term_v;
    bool brownout;
};

// Open-circuit pack voltage as a nonlinear function of SOC
double battery_ocv(double soc, int n_cells_series);

// Terminal voltage under power draw P without advancing state
double battery_terminal_voltage(
    double soc,
    double v_p,
    double r0_ohm,
    double p_elec_w,
    int n_cells_series,
    double* out_current = nullptr
);

// Advance 1-RC Thevenin battery state over dt_s with Peukert derating
BatteryStepOut battery_step(
    double soc,
    double v_p,
    double r0_ohm,
    double p_elec_w,
    double dt_s,
    const BatteryParams& batt
);

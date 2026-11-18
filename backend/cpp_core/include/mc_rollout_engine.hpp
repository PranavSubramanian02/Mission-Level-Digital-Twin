#pragma once
#include "twin_types.hpp"

// Execute N OpenMP-parallel stochastic 3D mission rollouts
void run_monte_carlo_rollouts(
    const VehicleParams& veh,
    const BatteryParams& batt,
    const Waypoint3D* waypoints,
    int n_waypoints,
    double v_cruise_ms,
    double v_loiter_ms,
    double loiter_time_s,
    double ds_m,
    double soc_init,
    double soc_std,
    double vp_init,
    double r0_mean,
    double dcd0_mean,
    const WindConfig& wind,
    int include_rtb,
    unsigned long long seed,
    MonteCarloResult& out
);

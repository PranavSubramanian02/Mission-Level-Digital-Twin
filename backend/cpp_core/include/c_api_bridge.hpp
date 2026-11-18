#pragma once
#include "twin_types.hpp"

#ifdef __cplusplus
extern "C" {
#endif

// Compute electrical power required across an airspeed array
void twin_compute_power_curve(
    const VehicleParams* veh,
    const double* airspeeds,
    int n_points,
    double alt_m,
    double dcd0,
    double* out_power_w
);

// Run one deterministic 3D mission with optional time-series traces
SingleMissionResult twin_run_single_mission(
    const VehicleParams* veh,
    const BatteryParams* batt,
    const Waypoint3D* waypoints,
    int n_waypoints,
    double v_cruise_ms,
    double v_loiter_ms,
    double loiter_time_s,
    double ds_m,
    double soc_init,
    double vp_init,
    double r0_ohm,
    double dcd0,
    const WindConfig* wind,
    int include_rtb,
    double* out_dist,
    double* out_time,
    double* out_soc,
    double* out_vterm,
    double* out_power,
    int max_trace_pts,
    int* out_trace_len
);

// Initialize C++ 6-state UKF estimator
void twin_ukf_init(
    UkfState* state,
    double soc_init,
    const BatteryParams* batt,
    const WindConfig* wind
);

// Step C++ 6-state UKF with one telemetry packet
void twin_ukf_step(
    UkfState* state,
    const TelemetryPacket* pkt,
    const VehicleParams* veh,
    const BatteryParams* batt
);

// Run OpenMP parallel Monte Carlo mission risk rollouts
void twin_run_monte_carlo(
    const VehicleParams* veh,
    const BatteryParams* batt,
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
    const WindConfig* wind,
    int include_rtb,
    unsigned long long seed,
    MonteCarloResult* out
);

#ifdef __cplusplus
}
#endif

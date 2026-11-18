#pragma once
#include "twin_types.hpp"
#include <random>

// ISA exponential air density at altitude alt_m
double air_density(double alt_m);

// Electrical propulsion + hotel power with climb and banked-turn drag
double electric_power_required(
    const VehicleParams& veh,
    double airspeed_ms,
    double alt_m,
    double climb_rate_ms,
    double bank_rad,
    double dcd0
);

// Groundspeed along 2D unit track accounting for crosswind crab angle
double groundspeed_along_track(
    double airspeed_ms,
    double track_n,
    double track_e,
    double wind_n,
    double wind_e
);

// Simulate 3D waypoint route + banked loiter + optional reciprocal RTB
SingleMissionResult simulate_3d_mission(
    const VehicleParams& veh,
    const BatteryParams& batt,
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
    const WindConfig& wind,
    int include_rtb,
    std::mt19937_64* rng = nullptr,
    double* out_dist = nullptr,
    double* out_time = nullptr,
    double* out_soc = nullptr,
    double* out_vterm = nullptr,
    double* out_power = nullptr,
    int max_trace_pts = 0,
    int* out_trace_len = nullptr
);

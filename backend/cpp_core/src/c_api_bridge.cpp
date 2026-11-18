#include "c_api_bridge.hpp"
#include "mc_rollout_engine.hpp"
#include "ukf_estimator.hpp"
#include "wind_kinematics.hpp"

extern "C" {

void twin_compute_power_curve(
    const VehicleParams* veh,
    const double* airspeeds,
    int n_points,
    double alt_m,
    double dcd0,
    double* out_power_w
) {
    if (!veh || !airspeeds || !out_power_w) return;
    for (int i = 0; i < n_points; ++i) {
        out_power_w[i] = electric_power_required(*veh, airspeeds[i], alt_m, 0.0, 0.0, dcd0);
    }
}

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
) {
    return simulate_3d_mission(
        *veh, *batt, waypoints, n_waypoints,
        v_cruise_ms, v_loiter_ms, loiter_time_s, ds_m,
        soc_init, vp_init, r0_ohm, dcd0, *wind, include_rtb, nullptr,
        out_dist, out_time, out_soc, out_vterm, out_power,
        max_trace_pts, out_trace_len
    );
}

void twin_ukf_init(
    UkfState* state,
    double soc_init,
    const BatteryParams* batt,
    const WindConfig* wind
) {
    if (!state || !batt || !wind) return;
    ukf_init(*state, soc_init, *batt, *wind);
}

void twin_ukf_step(
    UkfState* state,
    const TelemetryPacket* pkt,
    const VehicleParams* veh,
    const BatteryParams* batt
) {
    if (!state || !pkt || !veh || !batt) return;
    ukf_step(*state, *pkt, *veh, *batt);
}

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
) {
    if (!veh || !batt || !waypoints || !wind || !out) return;
    run_monte_carlo_rollouts(
        *veh, *batt, waypoints, n_waypoints,
        v_cruise_ms, v_loiter_ms, loiter_time_s, ds_m,
        soc_init, soc_std, vp_init, r0_mean, dcd0_mean,
        *wind, include_rtb, seed, *out
    );
}

}

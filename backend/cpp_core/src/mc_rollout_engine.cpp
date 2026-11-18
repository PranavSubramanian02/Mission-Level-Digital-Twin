#include "mc_rollout_engine.hpp"
#include "wind_kinematics.hpp"
#include <algorithm>
#include <random>

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
) {
    const int n_runs = out.n_runs;

    // Parallel stochastic rollouts with deterministic per-run RNG seeds
    #pragma omp parallel for schedule(static)
    for (int i = 0; i < n_runs; ++i) {
        std::mt19937_64 rng(seed + static_cast<unsigned long long>(i) * 7919ULL + 17ULL);
        std::normal_distribution<double> norm(0.0, 1.0);

        double soc0 = std::clamp(soc_init + soc_std * norm(rng), 0.05, 1.0);
        double r0 = std::max(r0_mean * (1.0 + wind.r0_sigma_frac * norm(rng)), 0.01);
        double dcd0 = std::max(dcd0_mean + wind.dcd0_sigma * norm(rng), -0.01);

        // Sample synoptic mean wind perturbation plus OU in-flight gusts
        WindConfig run_wind = wind;
        run_wind.wind_north_ms += 0.65 * wind.gust_sigma_ms * norm(rng);
        run_wind.wind_east_ms += 0.65 * wind.gust_sigma_ms * norm(rng);

        SingleMissionResult res = simulate_3d_mission(
            veh, batt, waypoints, n_waypoints,
            v_cruise_ms, v_loiter_ms, loiter_time_s, ds_m,
            soc0, vp_init, r0, dcd0, run_wind, include_rtb, &rng
        );

        out.final_soc[i] = res.final_soc;
        out.min_v_term[i] = res.min_v_term_v;
        out.total_time_s[i] = res.total_time_s;
        out.brownout_flags[i] = res.brownout;
        out.success_flags[i] = res.rtb_ok;
    }
}

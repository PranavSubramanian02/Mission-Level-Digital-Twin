#include "wind_kinematics.hpp"
#include "battery_ecm.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

static constexpr double GRAVITY = 9.81;

double air_density(double alt_m) {
    return 1.225 * std::exp(-std::max(alt_m, 0.0) / 8500.0);
}

double electric_power_required(
    const VehicleParams& veh,
    double airspeed_ms,
    double alt_m,
    double climb_rate_ms,
    double bank_rad,
    double dcd0
) {
    double v = std::max(airspeed_ms, 5.0);
    double rho = air_density(alt_m);
    double q = 0.5 * rho * v * v;
    double weight = veh.mass_kg * GRAVITY;

    // Load factor n = 1 / cos(phi) increases induced drag by n^2
    double cos_phi = std::max(std::cos(bank_rad), 0.5);
    double load_factor = 1.0 / cos_phi;
    double cl = (weight * load_factor) / (q * veh.wing_area_m2);
    double cd = std::max(veh.cd0 + dcd0, 0.005) + veh.k_induced * (cl * cl);
    double drag = q * veh.wing_area_m2 * cd;

    // Include climb potential power and regenerative-free descent clamp
    double p_aero = std::max(drag * v + weight * climb_rate_ms, 0.0);
    double eta = std::max(veh.eta_prop, 0.1);
    return (p_aero / eta) + veh.p_hotel_w;
}

double groundspeed_along_track(
    double airspeed_ms,
    double track_n,
    double track_e,
    double wind_n,
    double wind_e
) {
    // Project 2D wind onto track-parallel and crosswind components
    double w_par = wind_n * track_n + wind_e * track_e;
    double w_perp_n = wind_n - w_par * track_n;
    double w_perp_e = wind_e - w_par * track_e;
    double w_perp_sq = w_perp_n * w_perp_n + w_perp_e * w_perp_e;

    // Crab-angle groundspeed reduction from crosswind
    double v_crab = std::sqrt(std::max(airspeed_ms * airspeed_ms - w_perp_sq, 1.0));
    return std::max(v_crab + w_par, 2.0);
}

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
    std::mt19937_64* rng,
    double* out_dist,
    double* out_time,
    double* out_soc,
    double* out_vterm,
    double* out_power,
    int max_trace_pts,
    int* out_trace_len
) {
    double soc = soc_init;
    double vp = vp_init;
    double t_elapsed = 0.0;
    double dist_elapsed = 0.0;
    double min_v = battery_ocv(soc, batt.n_cells_series);
    bool brownout = false;

    double gust_n = 0.0;
    double gust_e = 0.0;
    std::normal_distribution<double> norm(0.0, 1.0);

    int trace_idx = 0;
    auto push_trace = [&](double d, double t, double s, double v, double p) {
        if (out_dist && trace_idx < max_trace_pts) {
            out_dist[trace_idx] = d;
            out_time[trace_idx] = t;
            out_soc[trace_idx] = s;
            out_vterm[trace_idx] = v;
            out_power[trace_idx] = p;
            trace_idx++;
        }
    };

    auto step_ou_gust = [&](double dt) {
        if (!rng || wind.gust_sigma_ms <= 0.0) return;
        double tau = std::max(wind.ou_tau_s, 1.0);
        double phi = std::exp(-dt / tau);
        double scale = wind.gust_sigma_ms * std::sqrt(std::max(1.0 - phi * phi, 0.0));
        gust_n = phi * gust_n + scale * norm(*rng);
        gust_e = phi * gust_e + scale * norm(*rng);
    };

    auto fly_leg = [&](const Waypoint3D& a, const Waypoint3D& b) {
        double dn = b.north_m - a.north_m;
        double de = b.east_m - a.east_m;
        double dh = b.alt_m - a.alt_m;
        double horiz = std::hypot(dn, de);
        if (horiz < 1e-3) return;

        double un = dn / horiz;
        double ue = de / horiz;
        int n_steps = std::max(1, static_cast<int>(std::ceil(horiz / std::max(ds_m, 10.0))));
        double step_ds = horiz / static_cast<double>(n_steps);
        double step_dh = dh / static_cast<double>(n_steps);

        for (int i = 0; i < n_steps; ++i) {
            double alt = a.alt_m + (i + 0.5) * step_dh;
            double vg = groundspeed_along_track(v_cruise_ms, un, ue, wind.wind_north_ms + gust_n, wind.wind_east_ms + gust_e);
            double dt = step_ds / vg;
            double vz = step_dh / dt;

            double p_elec = electric_power_required(veh, v_cruise_ms, alt, vz, 0.0, dcd0);
            BatteryStepOut bo = battery_step(soc, vp, r0_ohm, p_elec, dt, batt);

            soc = bo.soc;
            vp = bo.v_p;
            t_elapsed += dt;
            dist_elapsed += step_ds;
            min_v = std::min(min_v, bo.v_term_v);
            if (bo.brownout) brownout = true;

            push_trace(dist_elapsed, t_elapsed, soc, bo.v_term_v, p_elec);
            step_ou_gust(dt);
        }
    };

    // Initial point in trace
    if (n_waypoints > 0) {
        double p0 = electric_power_required(veh, v_cruise_ms, waypoints[0].alt_m, 0.0, 0.0, dcd0);
        double v0 = battery_terminal_voltage(soc, vp, r0_ohm, p0, batt.n_cells_series);
        push_trace(0.0, 0.0, soc, v0, p0);
    }

    // 1. Outbound 3D route segments
    for (int w = 0; w < n_waypoints - 1; ++w) {
        fly_leg(waypoints[w], waypoints[w + 1]);
    }

    // 2. Banked loiter orbit at final waypoint
    if (loiter_time_s > 0.0 && n_waypoints > 0) {
        double loit_alt = waypoints[n_waypoints - 1].alt_m;
        int n_loit = std::max(1, static_cast<int>(std::ceil(loiter_time_s / 5.0)));
        double dt_loit = loiter_time_s / static_cast<double>(n_loit);
        for (int i = 0; i < n_loit; ++i) {
            double p_loit = electric_power_required(veh, v_loiter_ms, loit_alt, 0.0, veh.loiter_bank_rad, dcd0);
            BatteryStepOut bo = battery_step(soc, vp, r0_ohm, p_loit, dt_loit, batt);

            soc = bo.soc;
            vp = bo.v_p;
            t_elapsed += dt_loit;
            min_v = std::min(min_v, bo.v_term_v);
            if (bo.brownout) brownout = true;

            push_trace(dist_elapsed, t_elapsed, soc, bo.v_term_v, p_loit);
            step_ou_gust(dt_loit);
        }
    }

    // 3. Reciprocal Return-to-Base (track reverses -> wind vector automatically flips sign)
    if (include_rtb && n_waypoints > 1) {
        for (int w = n_waypoints - 1; w > 0; --w) {
            fly_leg(waypoints[w], waypoints[w - 1]);
        }
    }

    if (out_trace_len) *out_trace_len = trace_idx;
    int rtb_ok = (!brownout && soc >= batt.soc_reserve) ? 1 : 0;
    return {soc, min_v, t_elapsed, dist_elapsed, brownout ? 1 : 0, rtb_ok};
}

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Fixed-wing UAV aerodynamic and mass parameters
typedef struct {
    double mass_kg;
    double wing_area_m2;
    double cd0;
    double k_induced;
    double eta_prop;
    double p_hotel_w;
    double loiter_bank_rad;
} VehicleParams;

// 1-RC Thevenin electro-thermal Li-ion pack parameters
typedef struct {
    int n_cells_series;
    double capacity_ah;
    double r0_ohm;
    double r1_ohm;
    double c1_farad;
    double v_cutoff_v;
    double peukert_k;
    double soc_reserve;
} BatteryParams;

// 3D mission waypoint in local North-East-Altitude frame
typedef struct {
    double north_m;
    double east_m;
    double alt_m;
} Waypoint3D;

// Stochastic 2D wind field and Ornstein-Uhlenbeck gust config
typedef struct {
    double wind_north_ms;
    double wind_east_ms;
    double gust_sigma_ms;
    double ou_tau_s;
    double r0_sigma_frac;
    double dcd0_sigma;
} WindConfig;

// Live noisy onboard telemetry packet for UKF assimilation
typedef struct {
    double dt_s;
    double airspeed_ms;
    double heading_rad;
    double climb_rate_ms;
    double alt_m;
    double bank_rad;
    double meas_v_term_v;
    double meas_vg_north_ms;
    double meas_vg_east_ms;
    double meas_p_elec_w;
} TelemetryPacket;

// 6-state UKF estimate: [SOC, Vp, R0, dCD0, Wn, We] and covariance
typedef struct {
    double x[6];
    double P[36];
} UkfState;

// Single deterministic mission trajectory summary
typedef struct {
    double final_soc;
    double min_v_term_v;
    double total_time_s;
    double total_dist_m;
    int brownout;
    int rtb_ok;
} SingleMissionResult;

// Output buffers for parallel Monte Carlo rollouts
typedef struct {
    int n_runs;
    double* final_soc;
    double* min_v_term;
    double* total_time_s;
    int* brownout_flags;
    int* success_flags;
} MonteCarloResult;

#ifdef __cplusplus
}
#endif

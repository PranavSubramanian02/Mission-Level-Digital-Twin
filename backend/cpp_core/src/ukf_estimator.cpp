#include "ukf_estimator.hpp"
#include "battery_ecm.hpp"
#include "wind_kinematics.hpp"
#include <algorithm>
#include <array>
#include <cmath>

static constexpr int NX = 6;
static constexpr int NZ = 4;
static constexpr int NSIG = 2 * NX + 1;

static bool cholesky_6x6(const std::array<double, 36>& A, std::array<double, 36>& L) {
    L.fill(0.0);
    for (int i = 0; i < NX; ++i) {
        for (int j = 0; j <= i; ++j) {
            double sum = A[i * NX + j];
            for (int k = 0; k < j; ++k) sum -= L[i * NX + k] * L[j * NX + k];
            if (i == j) {
                if (sum <= 1e-12) return false;
                L[i * NX + j] = std::sqrt(sum);
            } else {
                L[i * NX + j] = sum / L[j * NX + j];
            }
        }
    }
    return true;
}

static bool invert_4x4(const std::array<double, 16>& A, std::array<double, 16>& inv) {
    // Gauss-Jordan elimination with partial pivoting for 4x4 innovation covariance
    double aug[NZ][2 * NZ];
    for (int i = 0; i < NZ; ++i) {
        for (int j = 0; j < NZ; ++j) {
            aug[i][j] = A[i * NZ + j];
            aug[i][j + NZ] = (i == j) ? 1.0 : 0.0;
        }
    }
    for (int col = 0; col < NZ; ++col) {
        int piv = col;
        double max_v = std::abs(aug[col][col]);
        for (int r = col + 1; r < NZ; ++r) {
            if (std::abs(aug[r][col]) > max_v) {
                max_v = std::abs(aug[r][col]);
                piv = r;
            }
        }
        if (max_v < 1e-12) return false;
        if (piv != col) {
            for (int j = 0; j < 2 * NZ; ++j) std::swap(aug[col][j], aug[piv][j]);
        }
        double diag = aug[col][col];
        for (int j = 0; j < 2 * NZ; ++j) aug[col][j] /= diag;
        for (int r = 0; r < NZ; ++r) {
            if (r == col) continue;
            double f = aug[r][col];
            for (int j = 0; j < 2 * NZ; ++j) aug[r][j] -= f * aug[col][j];
        }
    }
    for (int i = 0; i < NZ; ++i)
        for (int j = 0; j < NZ; ++j)
            inv[i * NZ + j] = aug[i][j + NZ];
    return true;
}

static void clamp_state(std::array<double, NX>& x) {
    x[0] = std::clamp(x[0], 0.0, 1.0);       // SOC
    x[1] = std::clamp(x[1], 0.0, 4.0);       // Polarization Vp [V]
    x[2] = std::clamp(x[2], 0.01, 0.50);     // Ohmic resistance R0 [Ohm]
    x[3] = std::clamp(x[3], -0.015, 0.05);   // Parasitic drag delta dCD0
    x[4] = std::clamp(x[4], -25.0, 25.0);    // Wind North [m/s]
    x[5] = std::clamp(x[5], -25.0, 25.0);    // Wind East [m/s]
}

void ukf_init(
    UkfState& state,
    double soc_init,
    const BatteryParams& batt,
    const WindConfig& wind
) {
    state.x[0] = std::clamp(soc_init, 0.0, 1.0);
    state.x[1] = 0.0;
    state.x[2] = batt.r0_ohm;
    state.x[3] = 0.0;
    state.x[4] = wind.wind_north_ms;
    state.x[5] = wind.wind_east_ms;

    for (int i = 0; i < 36; ++i) state.P[i] = 0.0;
    state.P[0 * NX + 0] = 0.02 * 0.02;
    state.P[1 * NX + 1] = 0.05 * 0.05;
    state.P[2 * NX + 2] = 0.03 * 0.03;
    state.P[3 * NX + 3] = 0.008 * 0.008;
    state.P[4 * NX + 4] = 2.0 * 2.0;
    state.P[5 * NX + 5] = 2.0 * 2.0;
}

void ukf_step(
    UkfState& state,
    const TelemetryPacket& pkt,
    const VehicleParams& veh,
    const BatteryParams& batt
) {
    // Merwe scaled sigma-point weights
    const double alpha = 0.25;
    const double beta = 2.0;
    const double kappa = 0.0;
    const double lambda = alpha * alpha * (NX + kappa) - NX;
    const double scale = std::sqrt(NX + lambda);

    std::array<double, NSIG> wm{};
    std::array<double, NSIG> wc{};
    wm[0] = lambda / (NX + lambda);
    wc[0] = wm[0] + (1.0 - alpha * alpha + beta);
    for (int i = 1; i < NSIG; ++i) {
        wm[i] = 0.5 / (NX + lambda);
        wc[i] = wm[i];
    }

    // Symmetrize and Cholesky-factor P with jitter safeguard
    std::array<double, 36> P_sym{};
    for (int i = 0; i < NX; ++i)
        for (int j = 0; j < NX; ++j)
            P_sym[i * NX + j] = 0.5 * (state.P[i * NX + j] + state.P[j * NX + i]);

    std::array<double, 36> L{};
    double jitter = 1e-9;
    while (!cholesky_6x6(P_sym, L) && jitter < 1e-1) {
        for (int i = 0; i < NX; ++i) P_sym[i * NX + i] += jitter;
        jitter *= 10.0;
    }

    // Generate 2N + 1 sigma points
    std::array<std::array<double, NX>, NSIG> sig{};
    for (int i = 0; i < NX; ++i) sig[0][i] = state.x[i];
    for (int j = 0; j < NX; ++j) {
        for (int i = 0; i < NX; ++i) {
            double delta = scale * L[i * NX + j];
            sig[1 + j][i] = state.x[i] + delta;
            sig[1 + NX + j][i] = state.x[i] - delta;
        }
        clamp_state(sig[1 + j]);
        clamp_state(sig[1 + NX + j]);
    }

    // Process propagation through 1-RC battery ECM + random walk parameters
    std::array<std::array<double, NX>, NSIG> sig_pred{};
    for (int k = 0; k < NSIG; ++k) {
        BatteryStepOut bo = battery_step(sig[k][0], sig[k][1], sig[k][2], pkt.meas_p_elec_w, pkt.dt_s, batt);
        sig_pred[k] = sig[k];
        sig_pred[k][0] = bo.soc;
        sig_pred[k][1] = bo.v_p;
        clamp_state(sig_pred[k]);
    }

    // Predicted state mean and covariance with process noise Q
    std::array<double, NX> x_pred{};
    for (int k = 0; k < NSIG; ++k)
        for (int i = 0; i < NX; ++i)
            x_pred[i] += wm[k] * sig_pred[k][i];
    clamp_state(x_pred);

    const double dt = std::max(pkt.dt_s, 0.1);
    const std::array<double, NX> Q_diag = {
        1e-7 * dt,        // SOC process noise
        1e-5 * dt,        // Vp process noise
        4e-6 * dt,        // R0 drift allowance for aging/thermal step detection
        2e-6 * dt,        // dCD0 drift allowance for icing/fouling detection
        0.15 * dt,        // Wind North random walk
        0.15 * dt         // Wind East random walk
    };

    std::array<double, 36> P_pred{};
    for (int k = 0; k < NSIG; ++k) {
        for (int i = 0; i < NX; ++i) {
            double di = sig_pred[k][i] - x_pred[i];
            for (int j = 0; j < NX; ++j) {
                P_pred[i * NX + j] += wc[k] * di * (sig_pred[k][j] - x_pred[j]);
            }
        }
    }
    for (int i = 0; i < NX; ++i) P_pred[i * NX + i] += Q_diag[i];

    // Measurement predictions: [V_term, Vg_N, Vg_E, P_elec]
    std::array<std::array<double, NZ>, NSIG> z_sig{};
    std::array<double, NZ> z_pred{};
    double cos_h = std::cos(pkt.heading_rad);
    double sin_h = std::sin(pkt.heading_rad);

    for (int k = 0; k < NSIG; ++k) {
        double p_model = electric_power_required(veh, pkt.airspeed_ms, pkt.alt_m, pkt.climb_rate_ms, pkt.bank_rad, sig_pred[k][3]);
        double v_model = battery_terminal_voltage(sig_pred[k][0], sig_pred[k][1], sig_pred[k][2], pkt.meas_p_elec_w, batt.n_cells_series);
        z_sig[k][0] = v_model;
        z_sig[k][1] = pkt.airspeed_ms * cos_h + sig_pred[k][4];
        z_sig[k][2] = pkt.airspeed_ms * sin_h + sig_pred[k][5];
        z_sig[k][3] = p_model;
        for (int m = 0; m < NZ; ++m) z_pred[m] += wm[k] * z_sig[k][m];
    }

    // Innovation covariance S and cross-covariance Pxz
    const std::array<double, NZ> R_diag = {
        0.04 * 0.04,      // Terminal voltage sensor variance [V^2]
        0.35 * 0.35,      // GPS Vg North variance [(m/s)^2]
        0.35 * 0.35,      // GPS Vg East variance [(m/s)^2]
        6.0 * 6.0         // Power sensor variance [W^2]
    };

    std::array<double, 16> S{};
    std::array<double, NX * NZ> Pxz{};
    for (int k = 0; k < NSIG; ++k) {
        for (int m = 0; m < NZ; ++m) {
            double dz_m = z_sig[k][m] - z_pred[m];
            for (int n = 0; n < NZ; ++n) S[m * NZ + n] += wc[k] * dz_m * (z_sig[k][n] - z_pred[n]);
            for (int i = 0; i < NX; ++i) Pxz[i * NZ + m] += wc[k] * (sig_pred[k][i] - x_pred[i]) * dz_m;
        }
    }
    for (int m = 0; m < NZ; ++m) S[m * NZ + m] += R_diag[m];

    std::array<double, 16> S_inv{};
    if (!invert_4x4(S, S_inv)) {
        for (int i = 0; i < NX; ++i) state.x[i] = x_pred[i];
        for (int i = 0; i < 36; ++i) state.P[i] = P_pred[i];
        return;
    }

    // Kalman gain K = Pxz * S^-1
    std::array<double, NX * NZ> K{};
    for (int i = 0; i < NX; ++i)
        for (int m = 0; m < NZ; ++m)
            for (int n = 0; n < NZ; ++n)
                K[i * NZ + m] += Pxz[i * NZ + n] * S_inv[n * NZ + m];

    const std::array<double, NZ> z_meas = {
        pkt.meas_v_term_v,
        pkt.meas_vg_north_ms,
        pkt.meas_vg_east_ms,
        pkt.meas_p_elec_w
    };

    std::array<double, NX> x_upd = x_pred;
    for (int i = 0; i < NX; ++i)
        for (int m = 0; m < NZ; ++m)
            x_upd[i] += K[i * NZ + m] * (z_meas[m] - z_pred[m]);
    clamp_state(x_upd);

    // Covariance update P = P_pred - K * S * K^T
    std::array<double, NX * NZ> KS{};
    for (int i = 0; i < NX; ++i)
        for (int m = 0; m < NZ; ++m)
            for (int n = 0; n < NZ; ++n)
                KS[i * NZ + m] += K[i * NZ + n] * S[n * NZ + m];

    for (int i = 0; i < NX; ++i) {
        state.x[i] = x_upd[i];
        for (int j = 0; j < NX; ++j) {
            double sub = 0.0;
            for (int m = 0; m < NZ; ++m) sub += KS[i * NZ + m] * K[j * NZ + m];
            state.P[i * NX + j] = P_pred[i * NX + j] - sub;
        }
    }
    // Enforce symmetry and minimum diagonal floor
    for (int i = 0; i < NX; ++i) {
        state.P[i * NX + i] = std::max(state.P[i * NX + i], 1e-10);
        for (int j = i + 1; j < NX; ++j) {
            double avg = 0.5 * (state.P[i * NX + j] + state.P[j * NX + i]);
            state.P[i * NX + j] = avg;
            state.P[j * NX + i] = avg;
        }
    }
}

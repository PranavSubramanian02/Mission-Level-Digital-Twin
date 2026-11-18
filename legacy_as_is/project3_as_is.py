#!/usr/bin/env python3
"""
===========================================================================
MISSION-LEVEL DIGITAL TWIN FOR UAV OPERATIONS
COMPLETE THESIS-GRADE IMPLEMENTATION

As-Is Python Migration of MATLAB Live Script (project3.mlx)
Author of Original Model: Pranav Subramanian (M.Tech)
Target Venue: ICAAV-2026 (ADE-DRDO & AeSI)

PURPOSE:
Physics-based mission feasibility, energy, time, and risk evaluation
framework for UAV operations with full observability.

REFERENCES:
[1] Glaessgen & Stargel, NASA, 2012 (Digital Twin Paradigm)
[2] Beard & McLain, Princeton, 2012 (UAV Modeling)
[3] Raymer, AIAA, 2018 (Performance Estimation)
[4] Langelaan, JGCD, 2007 (Energy-based Guidance)
[5] Frazzoli et al., 2002 (Planning with Disturbances)
===========================================================================
"""

import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import gaussian_kde
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

def mission_model(dist, V_air, V_loiter, loiter_time, P_cruise, rho, S, CD0, k, W, ds, Vw, E_batt_max, SOC_init):
    """
    Local simulation function matching MATLAB mission_model exactly.
    """
    s = np.arange(0, dist + ds, ds)
    SOC = np.zeros(len(s))
    time = np.zeros(len(s))
    power = np.zeros(len(s))

    SOC[0] = SOC_init

    for i in range(1, len(s)):
        Vg = max(V_air + Vw, 2.0)
        dt = ds / Vg
        SOC[i] = SOC[i - 1] - (P_cruise * dt) / E_batt_max
        time[i] = time[i - 1] + dt
        power[i] = P_cruise

    if loiter_time > 0:
        CL = W / (0.5 * rho * (V_loiter ** 2) * S)
        CD = CD0 + k * (CL ** 2)
        D  = 0.5 * rho * (V_loiter ** 2) * S * CD
        # Preserving original MATLAB formula as-is (without / eta_prop)
        P_loiter = D * V_loiter
        SOC[-1] = SOC[-1] - (P_loiter * loiter_time) / E_batt_max
        time[-1] = time[-1] + loiter_time

    return s, SOC, time, power

def run_digital_twin(show_plots=False, save_dir="output_figures"):
    # Reproducibility
    np.random.seed(42)

    if save_dir:
        os.makedirs(save_dir, exist_ok=True)

    # =========================================================================
    # 1. GLOBAL CONSTANTS
    # =========================================================================
    g = 9.81                                # gravity [m/s^2]

    # =========================================================================
    # 2. VEHICLE PARAMETERS
    # =========================================================================
    S        = 0.8                          # wing area [m^2]
    CD0      = 0.03
    k_drag   = 0.045
    eta_prop = 0.7

    mass_empty   = 8.0
    mass_payload = 2.0
    m = mass_empty + mass_payload
    W = m * g

    # =========================================================================
    # 3. BATTERY MODEL
    # =========================================================================
    E_batt_max  = 1200.0 * 3600.0           # [J]
    SOC_init    = 1.0
    SOC_reserve = 0.25

    # =========================================================================
    # 4. ENVIRONMENT MODEL
    # =========================================================================
    h = 120.0                               # altitude [m]
    rho0 = 1.225
    H_scale = 8500.0
    rho = rho0 * np.exp(-h / H_scale)

    wind_mean  = 4.0
    wind_sigma = 1.2
    wind_dir   = np.pi

    # =========================================================================
    # 5. VEHICLE PERFORMANCE MAP
    # =========================================================================
    V_range = np.linspace(10.0, 28.0, 40)
    P_map = np.zeros(len(V_range))

    for i in range(len(V_range)):
        V = V_range[i]
        CL = W / (0.5 * rho * (V ** 2) * S)
        CD = CD0 + k_drag * (CL ** 2)
        D  = 0.5 * rho * (V ** 2) * S * CD
        P_map[i] = D * V / eta_prop

    # Plot: Power Required vs Airspeed
    fig_pwr, ax_pwr = plt.subplots(figsize=(7, 4.5))
    ax_pwr.plot(V_range, P_map / 1000.0, 'b-', linewidth=2)
    ax_pwr.set_xlabel('Airspeed [m/s]')
    ax_pwr.set_ylabel('Power Required [kW]')
    ax_pwr.set_title('Power Required vs Airspeed')
    ax_pwr.grid(True)
    fig_pwr.tight_layout()
    if save_dir:
        fig_pwr.savefig(os.path.join(save_dir, "fig0_power_vs_airspeed.png"), dpi=300)

    # =========================================================================
    # 7. SELECT CRUISE CONDITION
    # =========================================================================
    V_air = 18.0
    CL = W / (0.5 * rho * (V_air ** 2) * S)
    CD = CD0 + k_drag * (CL ** 2)
    Drag = 0.5 * rho * (V_air ** 2) * S * CD
    P_cruise = Drag * V_air / eta_prop

    # =========================================================================
    # 8. MISSION DEFINITIONS
    # =========================================================================
    routes = [
        {'name': 'Route A', 'distance': 30000.0},
        {'name': 'Route B', 'distance': 38000.0},
        {'name': 'Route C', 'distance': 45000.0}
    ]

    ds = 50.0
    loiter_time = 300.0
    V_loiter = 14.0
    ToT_required = 1900.0

    # =========================================================================
    # 9. NOMINAL WIND
    # =========================================================================
    Vw_nominal = wind_mean * np.cos(wind_dir)  # 4 * cos(pi) = -4.0 m/s

    # =========================================================================
    # 10. ROUTE SIMULATION
    # =========================================================================
    results = []

    for r in routes:
        s, SOC, time_arr, power_profile = mission_model(
            r['distance'], V_air, V_loiter, loiter_time,
            P_cruise, rho, S, CD0, k_drag, W, ds, Vw_nominal,
            E_batt_max, SOC_init
        )

        res = {
            'name': r['name'],
            'distance': r['distance'],
            's': s,
            'SOC': SOC,
            'time': time_arr,
            'power': power_profile,
            'SOC_final': SOC[-1],
            'time_total': time_arr[-1],
            'energy_margin': SOC[-1] - SOC_reserve,
            'energy_ok': (SOC[-1] - SOC_reserve) >= 0.0,
            'ToT_ok': time_arr[-1] <= ToT_required
        }
        results.append(res)

    # =========================================================================
    # 11. RETURN-TO-BASE CHECK
    # =========================================================================
    for res in results:
        _, SOC_rtb, _, _ = mission_model(
            res['distance'], V_air, V_loiter, 0.0,
            P_cruise, rho, S, CD0, k_drag, W, ds, Vw_nominal,
            E_batt_max, res['SOC_final']
        )
        res['RTB_ok'] = bool(SOC_rtb[-1] >= SOC_reserve)

    # =========================================================================
    # 12. MONTE CARLO WIND RISK
    # =========================================================================
    N_mc = 300
    SOC_mc = np.zeros((N_mc, len(routes)))

    for r_idx, r in enumerate(routes):
        for k_mc in range(N_mc):
            Vw = (wind_mean + wind_sigma * np.random.randn()) * np.cos(wind_dir)
            _, SOC_tmp, _, _ = mission_model(
                r['distance'], V_air, V_loiter, loiter_time,
                P_cruise, rho, S, CD0, k_drag, W, ds, Vw,
                E_batt_max, SOC_init
            )
            SOC_mc[k_mc, r_idx] = SOC_tmp[-1]

        results[r_idx]['P_success'] = float(np.mean(SOC_mc[:, r_idx] >= SOC_reserve))
        results[r_idx]['risk'] = 1.0 - results[r_idx]['P_success']

    # =========================================================================
    # PLOTS SECTION
    # =========================================================================

    # PLOT 1: SOC vs Distance (Route A)
    fig1, ax1 = plt.subplots(figsize=(7, 4.5))
    ax1.plot(results[0]['s'] / 1000.0, results[0]['SOC'], 'b-', linewidth=2)
    ax1.axhline(SOC_reserve, color='r', linestyle='--', label=f'Reserve ({SOC_reserve*100:.0f}%)')
    ax1.set_xlabel('Distance [km]')
    ax1.set_ylabel('SOC')
    ax1.set_title('SOC vs Distance (Route A)')
    ax1.legend(loc='best')
    ax1.grid(True)
    fig1.tight_layout()
    if save_dir:
        fig1.savefig(os.path.join(save_dir, "plot1_soc_vs_distance.png"), dpi=300)

    # PLOT 2: SOC vs Time (all routes)
    fig2, ax2 = plt.subplots(figsize=(7, 4.5))
    for res in results:
        ax2.plot(res['time'] / 60.0, res['SOC'], linewidth=2, label=res['name'])
    ax2.axhline(SOC_reserve, color='k', linestyle='--', label='Reserve Threshold')
    ax2.set_xlabel('Time [min]')
    ax2.set_ylabel('SOC')
    ax2.set_title('SOC vs Time')
    ax2.legend(loc='best')
    ax2.grid(True)
    fig2.tight_layout()
    if save_dir:
        fig2.savefig(os.path.join(save_dir, "plot2_soc_vs_time.png"), dpi=300)

    # PLOT 3: Power vs Distance (Route A)
    fig3, ax3 = plt.subplots(figsize=(7, 4.5))
    ax3.plot(results[0]['s'] / 1000.0, results[0]['power'] / 1000.0, 'b-', linewidth=2)
    ax3.set_xlabel('Distance [km]')
    ax3.set_ylabel('Power [kW]')
    ax3.set_title('Power Consumption Along Mission')
    ax3.grid(True)
    fig3.tight_layout()
    if save_dir:
        fig3.savefig(os.path.join(save_dir, "plot3_power_vs_distance.png"), dpi=300)

    # PLOT 4: Time-on-Target
    fig4, ax4 = plt.subplots(figsize=(7, 4.5))
    route_names = [res['name'] for res in results]
    tot_vals = [res['time_total'] / 60.0 for res in results]
    bars4 = ax4.bar(route_names, tot_vals, color='royalblue', width=0.5)
    ax4.axhline(ToT_required / 60.0, color='r', linestyle='--', label=f'ToT Limit ({ToT_required/60:.1f} min)')
    ax4.set_ylabel('Time [min]')
    ax4.set_title('Time-on-Target Constraint')
    ax4.legend(loc='best')
    ax4.grid(True, axis='y')
    fig4.tight_layout()
    if save_dir:
        fig4.savefig(os.path.join(save_dir, "plot4_time_on_target.png"), dpi=300)

    # PLOT 5: Energy Margin
    fig5, ax5 = plt.subplots(figsize=(7, 4.5))
    em_vals = [res['energy_margin'] * 100.0 for res in results]
    ax5.bar(route_names, em_vals, color='darkorange', width=0.5)
    ax5.set_ylabel('Energy Margin [%]')
    ax5.set_title('Energy Margin Comparison')
    ax5.grid(True, axis='y')
    fig5.tight_layout()
    if save_dir:
        fig5.savefig(os.path.join(save_dir, "plot5_energy_margin.png"), dpi=300)

    # PLOT 6: Monte Carlo Histogram (Route A)
    fig6, ax6 = plt.subplots(figsize=(7, 4.5))
    ax6.hist(SOC_mc[:, 0], bins=25, color='steelblue', edgecolor='black', alpha=0.7)
    ax6.axvline(SOC_reserve, color='r', linewidth=2, label=f'Reserve ({SOC_reserve})')
    ax6.set_xlabel('Final SOC')
    ax6.set_ylabel('Count')
    ax6.set_title('Monte Carlo SOC Distribution (Route A)')
    ax6.legend(loc='best')
    ax6.grid(True)
    fig6.tight_layout()
    if save_dir:
        fig6.savefig(os.path.join(save_dir, "plot6_monte_carlo_histogram.png"), dpi=300)

    # PLOT 7: SOC PDF (Risk Insight)
    fig7, ax7 = plt.subplots(figsize=(7, 4.5))
    kde = gaussian_kde(SOC_mc[:, 0])
    x_grid = np.linspace(min(SOC_mc[:, 0]) - 0.05, max(SOC_mc[:, 0]) + 0.05, 200)
    ax7.plot(x_grid, kde(x_grid), 'navy', linewidth=2)
    ax7.set_xlabel('Final SOC')
    ax7.set_ylabel('PDF')
    ax7.set_title('Probability Density of Final SOC (Route A)')
    ax7.grid(True)
    fig7.tight_layout()
    if save_dir:
        fig7.savefig(os.path.join(save_dir, "plot7_soc_pdf.png"), dpi=300)

    # PLOT 8: Success Probability
    fig8, ax8 = plt.subplots(figsize=(7, 4.5))
    p_succ_vals = [res['P_success'] * 100.0 for res in results]
    ax8.bar(route_names, p_succ_vals, color='forestgreen', width=0.5)
    ax8.set_ylabel('P(Success) [%]')
    ax8.set_title('Mission Success Probability')
    ax8.set_ylim(0, 105)
    ax8.grid(True, axis='y')
    fig8.tight_layout()
    if save_dir:
        fig8.savefig(os.path.join(save_dir, "plot8_success_probability.png"), dpi=300)

    # PLOT 9: Energy vs Time Trade
    fig9, ax9 = plt.subplots(figsize=(7, 4.5))
    for res in results:
        ax9.scatter(res['time_total'] / 60.0, res['energy_margin'] * 100.0, s=120, label=res['name'])
    ax9.set_xlabel('Time [min]')
    ax9.set_ylabel('Energy Margin [%]')
    ax9.set_title('Energy–Time Trade Space')
    ax9.legend(loc='best')
    ax9.grid(True)
    fig9.tight_layout()
    if save_dir:
        fig9.savefig(os.path.join(save_dir, "plot9_energy_vs_time.png"), dpi=300)

    # PLOT 10: Pareto Front (Energy–Time–Risk)
    fig10 = plt.figure(figsize=(8, 6))
    ax10 = fig10.add_subplot(111, projection='3d')
    for res in results:
        ax10.scatter(-res['energy_margin'], res['time_total'] / 60.0, res['risk'], s=150, label=res['name'])
    ax10.set_xlabel('Energy Cost (−Margin)')
    ax10.set_ylabel('Time [min]')
    ax10.set_zlabel('Risk')
    ax10.set_title('Pareto Front: Energy–Time–Risk')
    ax10.legend(loc='best')
    ax10.grid(True)
    fig10.tight_layout()
    if save_dir:
        fig10.savefig(os.path.join(save_dir, "plot10_pareto_3d.png"), dpi=300)

    # =========================================================================
    # 13. SUMMARY TABLE
    # =========================================================================
    print("\n================ MISSION SUMMARY ================")
    for res in results:
        print(f"{res['name']}")
        print(f" Final SOC      : {res['SOC_final'] * 100:.2f} %")
        print(f" Energy Margin  : {res['energy_margin'] * 100:.2f} %")
        print(f" Time           : {res['time_total'] / 60.0:.1f} min")
        print(f" P(success)     : {res['P_success'] * 100:.2f} %")
        print(f" RTB OK         : {1 if res['RTB_ok'] else 0}")
        print(f" ToT OK         : {1 if res['ToT_ok'] else 0}\n")
    print("=================================================\n")

    if show_plots:
        plt.show()

    return results

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Run Project 3 As-Is Digital Twin Simulation")
    parser.add_argument("--show", action="store_true", help="Display interactive plots")
    parser.add_argument("--save-dir", type=str, default="output_figures", help="Directory to save figures")
    args = parser.parse_args()

    run_digital_twin(show_plots=args.show, save_dir=args.save_dir)

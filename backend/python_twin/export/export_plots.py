# Headless 300 DPI publication exporter upgrading all 11 baseline figures plus UKF twin plots

import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from ..bridge.cpp_loader import (
    CppTwinBridge,
    benchmark_routes,
    default_battery,
    default_vehicle,
    default_wind,
)
from ..risk_planning.cvar_metrics import summarize_monte_carlo_risk
from ..risk_planning.pareto_planner import generate_pareto_frontier
from ..simulation.telemetry_plant import PhysicalUAVPlant


def export_all_publication_figures(output_dir: str = None, dpi: int = 300, n_runs: int = 1000) -> str:
    # Generate all 11 upgraded baseline plots plus live UKF fault-assimilation figures
    if output_dir is None:
        base = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
        output_dir = os.path.join(base, "output_figures")
    os.makedirs(output_dir, exist_ok=True)

    bridge = CppTwinBridge()
    veh = default_vehicle()
    batt = default_battery()
    wind = default_wind()
    routes = benchmark_routes()

    # Figure 0: Electrical Power Required vs Airspeed
    v_range = np.linspace(11.0, 28.0, 40)
    p_nom = bridge.compute_power_curve(veh, v_range, alt_m=150.0, dcd0=0.0)
    p_ice = bridge.compute_power_curve(veh, v_range, alt_m=150.0, dcd0=0.010)
    fig, ax = plt.subplots(figsize=(7.5, 4.8))
    ax.plot(v_range, p_nom, color="#0284c7", linewidth=2.2, label="Nominal Airframe")
    ax.plot(v_range, p_ice, color="#ef4444", linestyle="--", linewidth=2.0, label="Icing Fault (+0.010 CD0)")
    ax.set_xlabel("True Airspeed [m/s]")
    ax.set_ylabel("Total Electrical Power [W]")
    ax.set_title("UAV Electrical Power Curve (Propulsion + Hotel Load)")
    ax.legend()
    ax.grid(True, linestyle=":", alpha=0.5)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "fig0_power_vs_airspeed.png"), dpi=dpi)
    plt.close(fig)

    # Evaluate all 3 routes (nominal trace + N-run C++ OpenMP Monte Carlo)
    route_results = []
    for idx, route in enumerate(routes):
        nom = bridge.run_single_mission(veh, batt, route["waypoints"], wind, loiter_time_s=route["loiter_s"], include_rtb=True)
        mc = bridge.run_monte_carlo(veh, batt, route["waypoints"], wind, n_runs=n_runs, loiter_time_s=route["loiter_s"], include_rtb=True, seed=42 + idx * 101)
        stats = summarize_monte_carlo_risk(mc, soc_reserve=batt.soc_reserve, tot_limit_s=route["tot_limit_s"])
        route_results.append({"route": route, "nom": nom, "mc": mc, "stats": stats})

    names = [r["route"]["name"] for r in route_results]
    colors = ["#0284c7", "#10b981", "#f59e0b"]

    # Plot 1: SOC & Terminal Voltage vs Distance (Route A)
    nom_a = route_results[0]["nom"]
    fig, ax1 = plt.subplots(figsize=(7.5, 4.8))
    ax1.plot(nom_a["dist_m"] / 1000.0, nom_a["soc"] * 100.0, color="#0284c7", linewidth=2.2, label="SOC [%]")
    ax1.axhline(batt.soc_reserve * 100.0, color="#ef4444", linestyle="--", label="Reserve (25%)")
    ax1.set_xlabel("Cumulative Mission Distance (Outbound + RTB) [km]")
    ax1.set_ylabel("State of Charge [%]", color="#0284c7")
    ax2 = ax1.twinx()
    ax2.plot(nom_a["dist_m"] / 1000.0, nom_a["v_term_v"], color="#8b5cf6", linewidth=1.6, alpha=0.85, label="Terminal Voltage [V]")
    ax2.axhline(batt.v_cutoff_v, color="#991b1b", linestyle=":", label=f"Cutoff ({batt.v_cutoff_v:.1f} V)")
    ax2.set_ylabel("Pack Terminal Voltage [V]", color="#8b5cf6")
    ax1.set_title("Route A: SOC Depletion & 1-RC Terminal Voltage Sag")
    ax1.grid(True, linestyle=":", alpha=0.5)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "plot1_soc_vs_distance.png"), dpi=dpi)
    plt.close(fig)

    # Plot 2: SOC vs Time (all routes)
    fig, ax = plt.subplots(figsize=(7.5, 4.8))
    for item, col in zip(route_results, colors):
        ax.plot(item["nom"]["time_s"] / 60.0, item["nom"]["soc"] * 100.0, color=col, linewidth=2.2, label=item["route"]["name"])
    ax.axhline(batt.soc_reserve * 100.0, color="#ef4444", linestyle="--", label="Reserve Threshold (25%)")
    ax.set_xlabel("Mission Time [min]")
    ax.set_ylabel("State of Charge [%]")
    ax.set_title("Battery SOC Trajectory Across 3D Mission Routes (With Banked Loiter & RTB)")
    ax.legend()
    ax.grid(True, linestyle=":", alpha=0.5)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "plot2_soc_vs_time.png"), dpi=dpi)
    plt.close(fig)

    # Plot 3: Electrical Power Profile vs Distance (Route A)
    fig, ax = plt.subplots(figsize=(7.5, 4.8))
    ax.plot(nom_a["dist_m"] / 1000.0, nom_a["power_w"], color="#0284c7", linewidth=2.0)
    ax.set_xlabel("Cumulative Mission Distance [km]")
    ax.set_ylabel("Electrical Power Draw [W]")
    ax.set_title("Route A: 3D Climb, Cruise, Banked Loiter & Descent Power Profile")
    ax.grid(True, linestyle=":", alpha=0.5)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "plot3_power_vs_distance.png"), dpi=dpi)
    plt.close(fig)

    # Plot 4: Mission Time vs Time-on-Target Envelope
    fig, ax = plt.subplots(figsize=(7.5, 4.8))
    t_means = [r["stats"]["time_mean_s"] / 60.0 for r in route_results]
    t_limits = [r["route"]["tot_limit_s"] / 60.0 for r in route_results]
    x = np.arange(len(names))
    ax.bar(x - 0.18, t_means, width=0.35, color="#0284c7", label="Mean Mission Time")
    ax.bar(x + 0.18, t_limits, width=0.35, color="#64748b", alpha=0.65, label="ToT Limit")
    ax.set_xticks(x)
    ax.set_xticklabels(names)
    ax.set_ylabel("Duration [min]")
    ax.set_title("Time-on-Target Compliance Across Mission Profiles")
    ax.legend()
    ax.grid(True, axis="y", linestyle=":", alpha=0.5)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "plot4_time_on_target.png"), dpi=dpi)
    plt.close(fig)

    # Plot 5: Mean Energy Margin vs CVaR-95 Tail Floor
    fig, ax = plt.subplots(figsize=(7.5, 4.8))
    margins = [r["stats"]["energy_margin_mean"] * 100.0 for r in route_results]
    cvar_margins = [(r["stats"]["cvar_95_soc"] - batt.soc_reserve) * 100.0 for r in route_results]
    ax.bar(x - 0.18, margins, width=0.35, color="#10b981", label="Mean Energy Margin [%]")
    ax.bar(x + 0.18, cvar_margins, width=0.35, color="#ef4444", label="CVaR-95 Tail Margin [%]")
    ax.axhline(0.0, color="black", linewidth=1.0)
    ax.set_xticks(x)
    ax.set_xticklabels(names)
    ax.set_ylabel("Reserve Margin [%]")
    ax.set_title("Mean Reserve Margin vs. Coherent CVaR-95% Tail Margin")
    ax.legend()
    ax.grid(True, axis="y", linestyle=":", alpha=0.5)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "plot5_energy_margin.png"), dpi=dpi)
    plt.close(fig)

    # Plot 6: Monte Carlo Final SOC Histogram (Route B boundary case)
    mc_b = route_results[1]["mc"]["final_soc"] * 100.0
    cvar_b = route_results[1]["stats"]["cvar_95_soc"] * 100.0
    fig, ax = plt.subplots(figsize=(7.5, 4.8))
    ax.hist(mc_b, bins=30, color="#0284c7", edgecolor="#0f172a", alpha=0.75)
    ax.axvline(batt.soc_reserve * 100.0, color="#ef4444", linewidth=2.0, linestyle="--", label="Reserve (25%)")
    ax.axvline(cvar_b, color="#f59e0b", linewidth=2.0, linestyle="-.", label=f"CVaR-95% ({cvar_b:.1f}%)")
    ax.set_xlabel("Final SOC [%]")
    ax.set_ylabel("Monte Carlo Count")
    ax.set_title(f"Monte Carlo Terminal SOC Distribution (Route B, N={n_runs})")
    ax.legend()
    ax.grid(True, linestyle=":", alpha=0.5)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "plot6_monte_carlo_histogram.png"), dpi=dpi)
    plt.close(fig)

    # Plot 7: Final SOC KDE Probability Density Across All Routes
    fig, ax = plt.subplots(figsize=(7.5, 4.8))
    for item, col in zip(route_results, colors):
        ax.plot(np.array(item["stats"]["kde_x"]) * 100.0, item["stats"]["kde_pdf"], color=col, linewidth=2.2, label=item["route"]["name"])
    ax.axvline(batt.soc_reserve * 100.0, color="#ef4444", linestyle="--", label="Reserve Threshold (25%)")
    ax.set_xlabel("Final SOC [%]")
    ax.set_ylabel("Probability Density")
    ax.set_title("Stochastic Terminal SOC Probability Densities (OU Wind + Aging)")
    ax.legend()
    ax.grid(True, linestyle=":", alpha=0.5)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "plot7_soc_pdf.png"), dpi=dpi)
    plt.close(fig)

    # Plot 8: Mission Success Probability
    fig, ax = plt.subplots(figsize=(7.5, 4.8))
    p_vals = [r["stats"]["p_success"] * 100.0 for r in route_results]
    ax.bar(names, p_vals, color=["#10b981", "#f59e0b", "#ef4444"], width=0.5)
    ax.set_ylim(0, 108)
    ax.set_ylabel("P(Mission Success) [%]")
    ax.set_title("Mission Success Probability (Reserve + No Voltage Brownout)")
    ax.grid(True, axis="y", linestyle=":", alpha=0.5)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "plot8_success_probability.png"), dpi=dpi)
    plt.close(fig)

    # Plot 9 & Plot 10: Continuous Energy-Time-CVaR Pareto Frontier
    pareto = generate_pareto_frontier(bridge, veh, batt, routes, wind, n_runs=350)
    all_pts = pareto["all_candidates"]
    p_pts = pareto["pareto_points"]
    knee = pareto["knee_point"]

    fig, ax = plt.subplots(figsize=(7.5, 4.8))
    ax.scatter([c["time_min"] for c in all_pts], [-c["energy_cost"] * 100.0 for c in all_pts], color="#94a3b8", alpha=0.5, label="Swept Airspeed Plans")
    ax.scatter([c["time_min"] for c in p_pts], [-c["energy_cost"] * 100.0 for c in p_pts], color="#0284c7", s=70, label="Pareto Non-Dominated")
    if knee:
        ax.scatter(knee["time_min"], -knee["energy_cost"] * 100.0, color="#ef4444", marker="*", s=180, label="Knee Compromise")
    ax.set_xlabel("Mission Time [min]")
    ax.set_ylabel("Mean Energy Margin [%]")
    ax.set_title("Energy-Time Trade Space Across Airspeeds and 3D Routes")
    ax.legend()
    ax.grid(True, linestyle=":", alpha=0.5)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "plot9_energy_vs_time.png"), dpi=dpi)
    plt.close(fig)

    fig = plt.figure(figsize=(8.2, 6.0))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter([c["energy_cost"] * 100.0 for c in all_pts], [c["time_min"] for c in all_pts], [c["cvar_risk"] * 100.0 for c in all_pts], color="#94a3b8", alpha=0.35, label="Dominated")
    ax.scatter([c["energy_cost"] * 100.0 for c in p_pts], [c["time_min"] for c in p_pts], [c["cvar_risk"] * 100.0 for c in p_pts], color="#0284c7", s=65, label="Pareto Front")
    if knee:
        ax.scatter([knee["energy_cost"] * 100.0], [knee["time_min"]], [knee["cvar_risk"] * 100.0], color="#ef4444", marker="*", s=200, label="Knee Point")
    ax.set_xlabel("Energy Deficit Cost [%]")
    ax.set_ylabel("Mission Time [min]")
    ax.set_zlabel("CVaR-95% Tail Deficit [%]")
    ax.set_title("3D Continuous Pareto Front: Energy-Time-CVaR Risk")
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "plot10_pareto_3d.png"), dpi=dpi)
    plt.close(fig)

    # Plot 11 (New Digital Twin): Live C++ UKF Online Fault Assimilation Verification
    plant = PhysicalUAVPlant(veh, batt, routes[1]["waypoints"], wind, v_cruise_ms=18.0)
    ukf = bridge.init_ukf(1.0, batt, wind)
    ts, true_r0, est_r0, true_cd, est_cd = [], [], [], [], []
    for step in range(90):
        if step == 30:
            plant.inject_fault("icing_drag", 0.010)
            plant.inject_fault("battery_aging", 0.045)
        pkt, truth = plant.step(dt_s=5.0)
        est = bridge.step_ukf(ukf, pkt, veh, batt)
        ts.append(truth["sim_time_s"] / 60.0)
        true_r0.append(truth["true_r0_ohm"] * 1000.0)
        est_r0.append(est["r0_ohm"] * 1000.0)
        true_cd.append(truth["true_dcd0"])
        est_cd.append(est["dcd0"])

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8.0, 6.2), sharex=True)
    ax1.plot(ts, true_cd, color="#0f172a", linewidth=2.0, label="True Hidden Icing dCD0")
    ax1.plot(ts, est_cd, color="#0284c7", linestyle="--", linewidth=2.0, label="C++ UKF Estimated dCD0")
    ax1.axvline(2.5, color="#ef4444", linestyle=":", label="Fault Injected (t = 2.5 min)")
    ax1.set_ylabel("Parasitic Drag Delta (dCD0)")
    ax1.set_title("Bidirectional Digital Twin: Online C++ UKF Anomaly Assimilation")
    ax1.legend()
    ax1.grid(True, linestyle=":", alpha=0.5)

    ax2.plot(ts, true_r0, color="#0f172a", linewidth=2.0, label="True Pack Resistance R0 [mOhm]")
    ax2.plot(ts, est_r0, color="#10b981", linestyle="--", linewidth=2.0, label="C++ UKF Estimated R0 [mOhm]")
    ax2.axvline(2.5, color="#ef4444", linestyle=":")
    ax2.set_xlabel("Flight Time [min]")
    ax2.set_ylabel("Internal Resistance [mOhm]")
    ax2.legend()
    ax2.grid(True, linestyle=":", alpha=0.5)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "plot11_ukf_fault_assimilation.png"), dpi=dpi)
    plt.close(fig)

    return output_dir


if __name__ == "__main__":
    export_all_publication_figures()

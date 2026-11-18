# Continuous 3D Energy-Time-CVaR Pareto optimizer and supervisory mission advisor

import numpy as np
from ..bridge.cpp_loader import BatteryParamsC, CppTwinBridge, VehicleParamsC, WindConfigC
from .cvar_metrics import summarize_monte_carlo_risk


def non_dominated_mask(objectives: np.ndarray) -> np.ndarray:
    # Extract Rank-0 Pareto non-dominated mask for minimization objectives
    n = objectives.shape[0]
    keep = np.ones(n, dtype=bool)
    for i in range(n):
        if not keep[i]:
            continue
        dom = np.all(objectives <= objectives[i], axis=1) & np.any(objectives < objectives[i], axis=1)
        if np.any(dom):
            keep[i] = False
    return keep


def find_knee_index(objectives: np.ndarray) -> int:
    # Balanced Pareto compromise closest to normalized utopia point
    mins = np.min(objectives, axis=0)
    maxs = np.max(objectives, axis=0)
    span = np.where((maxs - mins) < 1e-9, 1.0, maxs - mins)
    norm = (objectives - mins) / span
    return int(np.argmin(np.linalg.norm(norm, axis=1)))


def generate_pareto_frontier(
    bridge: CppTwinBridge,
    veh: VehicleParamsC,
    batt: BatteryParamsC,
    routes: list,
    wind: WindConfigC,
    airspeeds: np.ndarray = None,
    n_runs: int = 400,
    seed: int = 42,
) -> dict:
    # Sweep continuous cruise airspeeds across 3D routes to build Energy-Time-CVaR Pareto set
    if airspeeds is None:
        airspeeds = np.linspace(14.5, 25.0, 12)

    candidates = []
    for r_idx, route in enumerate(routes):
        for v_idx, va in enumerate(airspeeds):
            mc = bridge.run_monte_carlo(
                veh, batt, route["waypoints"], wind,
                n_runs=n_runs,
                v_cruise_ms=float(va),
                loiter_time_s=float(route["loiter_s"]),
                include_rtb=True,
                seed=seed + r_idx * 1000 + v_idx * 31,
            )
            stats = summarize_monte_carlo_risk(mc, soc_reserve=batt.soc_reserve, tot_limit_s=route.get("tot_limit_s"))
            candidates.append({
                "route_name": route["name"],
                "airspeed_ms": float(va),
                "energy_cost": float(-stats["energy_margin_mean"]),
                "time_min": float(stats["time_mean_s"] / 60.0),
                "cvar_risk": float(max(stats["cvar_95_deficit"], 0.0)),
                "p_success": float(stats["p_success"]),
                "cvar_95_soc": float(stats["cvar_95_soc"]),
                "soc_mean": float(stats["soc_mean"]),
            })

    obj = np.array([[c["energy_cost"], c["time_min"], c["cvar_risk"]] for c in candidates], dtype=np.float64)
    mask = non_dominated_mask(obj)
    pareto_pts = [c for c, ok in zip(candidates, mask) if ok]
    knee = pareto_pts[find_knee_index(obj[mask])] if pareto_pts else None

    return {
        "all_candidates": candidates,
        "pareto_points": pareto_pts,
        "knee_point": knee,
    }


def evaluate_supervisory_advisory(
    p_success: float,
    cvar_95_soc: float,
    soc_reserve: float = 0.25,
    min_vterm_p05: float = 20.0,
    v_cutoff_v: float = 18.6,
) -> dict:
    # Risk-aware prescriptive advisory: GO / SPEED-OPTIMIZE / ABORT-TO-RTB
    if p_success < 0.70 or cvar_95_soc < (soc_reserve - 0.08) or min_vterm_p05 <= (v_cutoff_v + 0.3):
        return {
            "code": "ABORT_RTB",
            "severity": "CRITICAL",
            "message": "Tail risk exceeds safe return envelope; initiate immediate Return-to-Base.",
        }
    if p_success < 0.90 or cvar_95_soc < soc_reserve:
        return {
            "code": "SPEED_OPTIMIZE",
            "severity": "CAUTION",
            "message": "Elevated EoD risk detected; reduce airspeed toward best-range speed and shorten loiter.",
        }
    return {
        "code": "GO_NOMINAL",
        "severity": "NOMINAL",
        "message": "Mission remains within probabilistic energy and voltage safety envelope.",
    }

# Coherent tail-risk metrics (VaR, CVaR-95%), KDE PDFs, and Point-of-No-Return

import numpy as np
from scipy.stats import gaussian_kde


def compute_var_cvar(losses: np.ndarray, alpha: float = 0.95) -> tuple:
    # Rockafellar & Uryasev (2000) empirical Value-at-Risk and Conditional Value-at-Risk
    arr = np.asarray(losses, dtype=np.float64)
    if arr.size == 0:
        return 0.0, 0.0
    var = float(np.quantile(arr, alpha))
    tail = arr[arr >= var]
    cvar = float(np.mean(tail)) if tail.size > 0 else var
    return var, cvar


def summarize_monte_carlo_risk(mc_data: dict, soc_reserve: float = 0.25, tot_limit_s: float = None, alpha: float = 0.95) -> dict:
    # Aggregate C++ rollout arrays into operational feasibility and tail-risk metrics
    f_soc = np.asarray(mc_data["final_soc"], dtype=np.float64)
    min_v = np.asarray(mc_data["min_v_term"], dtype=np.float64)
    t_tot = np.asarray(mc_data["total_time_s"], dtype=np.float64)
    brown = np.asarray(mc_data["brownout_flags"], dtype=np.int32)
    succ = np.asarray(mc_data["success_flags"], dtype=np.int32)

    # Energy deficit loss: positive values represent reserve shortfall
    soc_loss = soc_reserve - f_soc
    var_loss, cvar_loss = compute_var_cvar(soc_loss, alpha=alpha)
    cvar_soc_floor = float(soc_reserve - cvar_loss)

    p_success = float(np.mean(succ))
    p_brownout = float(np.mean(brown))
    p_tot = float(np.mean(t_tot <= tot_limit_s)) if tot_limit_s is not None else 1.0

    # Smooth probability density estimate for plotting
    if np.std(f_soc) > 1e-5:
        kde = gaussian_kde(f_soc)
        x_grid = np.linspace(max(0.0, float(np.min(f_soc)) - 0.03), min(1.0, float(np.max(f_soc)) + 0.03), 120)
        pdf_vals = kde(x_grid)
    else:
        x_grid = np.linspace(max(0.0, float(np.mean(f_soc)) - 0.05), min(1.0, float(np.mean(f_soc)) + 0.05), 120)
        pdf_vals = np.zeros_like(x_grid)

    return {
        "p_success": p_success,
        "risk": float(1.0 - p_success),
        "p_brownout": p_brownout,
        "p_tot_ok": p_tot,
        "soc_mean": float(np.mean(f_soc)),
        "soc_std": float(np.std(f_soc)),
        "energy_margin_mean": float(np.mean(f_soc) - soc_reserve),
        "var_95_deficit": var_loss,
        "cvar_95_deficit": cvar_loss,
        "cvar_95_soc": cvar_soc_floor,
        "min_vterm_p05": float(np.quantile(min_v, 0.05)),
        "time_mean_s": float(np.mean(t_tot)),
        "time_p95_s": float(np.quantile(t_tot, 0.95)),
        "kde_x": x_grid.tolist(),
        "kde_pdf": pdf_vals.tolist(),
    }


def compute_pnr_fraction(soc_trace: np.ndarray, soc_reserve: float = 0.25) -> float:
    # Estimate Point-of-No-Return fraction along outbound leg from SOC depletion
    if len(soc_trace) < 2:
        return 0.0
    usable = max(float(soc_trace[0]) - soc_reserve, 0.0)
    half_threshold = float(soc_trace[0]) - 0.5 * usable
    below = np.where(soc_trace <= half_threshold)[0]
    if below.size == 0:
        return 1.0
    return float(below[0]) / float(len(soc_trace) - 1)

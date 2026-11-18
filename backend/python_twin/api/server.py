# FastAPI async Digital Twin server with REST, WebSockets, and static frontend hosting

import asyncio
import io
import os
import zipfile
import numpy as np
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from fastapi.staticfiles import StaticFiles

from ..bridge.cpp_loader import (
    CppTwinBridge,
    WindConfigC,
    benchmark_routes,
    default_battery,
    default_vehicle,
    default_wind,
)
from ..export.export_plots import export_all_publication_figures
from ..historian.digital_thread_db import DigitalThreadDB
from ..risk_planning.cvar_metrics import compute_pnr_fraction, summarize_monte_carlo_risk
from ..risk_planning.pareto_planner import evaluate_supervisory_advisory, generate_pareto_frontier
from ..simulation.telemetry_plant import PhysicalUAVPlant
from .schemas import ExportRequest, FaultInjectRequest, ParetoRequest, PreflightRequest, TwinStartRequest, TwinStepRequest

app = FastAPI(title="UAV Mission-Level Digital Twin API", version="1.0.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


class LiveTwinSession:
    # Stateful container binding physical truth plant to C++ UKF and lookahead engine
    def __init__(self):
        self.active = False
        self.run_id = None
        self.plant = None
        self.ukf = None
        self.veh = default_vehicle()
        self.batt = default_battery()
        self.wind = default_wind()
        self.route = benchmark_routes()[0]


_bridge = None
_db = DigitalThreadDB()
_session = LiveTwinSession()


def get_bridge() -> CppTwinBridge:
    global _bridge
    if _bridge is None:
        _bridge = CppTwinBridge()
    return _bridge


def _advance_session(dt_s: float = 5.0, mc_runs: int = 300) -> dict:
    bridge = get_bridge()
    pkt, truth = _session.plant.step(dt_s=dt_s)
    est = bridge.step_ukf(_session.ukf, pkt, _session.veh, _session.batt)

    # Re-run live lookahead Monte Carlo from UKF posterior belief
    rem_wps = _session.plant.remaining_waypoints()
    live_wind = WindConfigC(est["wind_n"], est["wind_e"], _session.wind.gust_sigma_ms, _session.wind.ou_tau_s, 0.10, 0.002)
    mc = bridge.run_monte_carlo(
        _session.veh,
        _session.batt,
        rem_wps,
        live_wind,
        n_runs=mc_runs,
        v_cruise_ms=_session.plant.v_cruise_ms,
        loiter_time_s=_session.route["loiter_s"] if not truth["finished"] else 0.0,
        soc_init=est["soc"],
        soc_std=max(est["std"][0], 0.008),
        vp_init=est["vp"],
        r0_mean=est["r0_ohm"],
        dcd0_mean=est["dcd0"],
        include_rtb=True,
        seed=int(truth["sim_time_s"]) + 42,
    )
    risk = summarize_monte_carlo_risk(mc, soc_reserve=_session.batt.soc_reserve, tot_limit_s=_session.route["tot_limit_s"])
    advisory = evaluate_supervisory_advisory(
        risk["p_success"], risk["cvar_95_soc"], _session.batt.soc_reserve, risk["min_vterm_p05"], _session.batt.v_cutoff_v
    )

    epoch = {
        "sim_time_s": truth["sim_time_s"],
        "dist_m": truth["dist_m"],
        "alt_m": truth["alt_m"],
        "true_soc": truth["true_soc"],
        "est_soc": est["soc"],
        "soc_std": est["std"][0],
        "true_vterm": truth["true_vterm"],
        "est_r0_ohm": est["r0_ohm"],
        "est_dcd0": est["dcd0"],
        "live_p_success": risk["p_success"],
        "live_cvar_95_soc": risk["cvar_95_soc"],
        "advisory_code": advisory["code"],
    }
    if _session.run_id is not None:
        _db.log_epoch(_session.run_id, epoch)

    return {
        "truth": truth,
        "ukf": est,
        "live_risk": risk,
        "advisory": advisory,
    }


@app.get("/api/v1/health")
def health_check():
    return {"status": "ok", "cpp_loaded": _bridge is not None}


@app.post("/api/v1/preflight/evaluate")
def preflight_evaluate(req: PreflightRequest):
    bridge = get_bridge()
    veh = default_vehicle()
    batt = default_battery()
    batt.r0_ohm = req.r0_ohm
    wind = default_wind()
    wind.wind_north_ms = req.wind_north_ms
    wind.wind_east_ms = req.wind_east_ms
    wind.gust_sigma_ms = req.gust_sigma_ms

    # Aerodynamic power curve across airspeed envelope
    v_grid = np.linspace(11.0, 28.0, 35)
    p_curve = bridge.compute_power_curve(veh, v_grid, alt_m=150.0, dcd0=0.0)

    routes_out = []
    for idx, route in enumerate(benchmark_routes()):
        nom = bridge.run_single_mission(
            veh, batt, route["waypoints"], wind,
            v_cruise_ms=req.v_cruise_ms,
            v_loiter_ms=req.v_loiter_ms,
            loiter_time_s=route["loiter_s"],
            include_rtb=req.include_rtb,
        )
        mc = bridge.run_monte_carlo(
            veh, batt, route["waypoints"], wind,
            n_runs=req.n_runs,
            v_cruise_ms=req.v_cruise_ms,
            v_loiter_ms=req.v_loiter_ms,
            loiter_time_s=route["loiter_s"],
            include_rtb=req.include_rtb,
            seed=42 + idx * 101,
        )
        stats = summarize_monte_carlo_risk(mc, soc_reserve=batt.soc_reserve, tot_limit_s=route["tot_limit_s"])
        adv = evaluate_supervisory_advisory(
            stats["p_success"], stats["cvar_95_soc"], batt.soc_reserve, stats["min_vterm_p05"], batt.v_cutoff_v
        )
        pnr = compute_pnr_fraction(nom["soc"], batt.soc_reserve)
        _db.log_mission_run(
            route["name"], req.v_cruise_ms, req.wind_north_ms,
            stats["p_success"], stats["cvar_95_soc"], stats["soc_mean"], stats["min_vterm_p05"], adv["code"]
        )

        # Downsample traces for responsive JSON transport
        stride = max(1, len(nom["dist_m"]) // 160)
        routes_out.append({
            "name": route["name"],
            "waypoints": route["waypoints"],
            "nominal": {
                "final_soc": nom["final_soc"],
                "min_v_term_v": nom["min_v_term_v"],
                "total_time_s": nom["total_time_s"],
                "total_dist_m": nom["total_dist_m"],
                "brownout": nom["brownout"],
                "rtb_ok": nom["rtb_ok"],
                "pnr_fraction": pnr,
                "dist_km": (nom["dist_m"][::stride] / 1000.0).tolist(),
                "time_min": (nom["time_s"][::stride] / 60.0).tolist(),
                "soc": nom["soc"][::stride].tolist(),
                "v_term_v": nom["v_term_v"][::stride].tolist(),
                "power_w": nom["power_w"][::stride].tolist(),
            },
            "risk": stats,
            "advisory": adv,
            "mc_soc_samples": mc["final_soc"][:400].tolist(),
        })

    return {
        "power_curve": {"airspeed_ms": v_grid.tolist(), "power_w": p_curve.tolist()},
        "routes": routes_out,
    }


@app.post("/api/v1/preflight/pareto")
def preflight_pareto(req: ParetoRequest):
    bridge = get_bridge()
    veh = default_vehicle()
    batt = default_battery()
    batt.r0_ohm = req.r0_ohm
    wind = default_wind()
    wind.wind_north_ms = req.wind_north_ms
    wind.wind_east_ms = req.wind_east_ms
    wind.gust_sigma_ms = req.gust_sigma_ms
    return generate_pareto_frontier(bridge, veh, batt, benchmark_routes(), wind, n_runs=req.n_runs_per_point)


@app.post("/api/v1/twin/session/start")
def twin_start(req: TwinStartRequest):
    bridge = get_bridge()
    _session.veh = default_vehicle()
    _session.batt = default_battery()
    _session.wind = default_wind()
    _session.wind.wind_north_ms = req.wind_north_ms
    _session.wind.wind_east_ms = req.wind_east_ms
    _session.wind.gust_sigma_ms = req.gust_sigma_ms
    _session.route = benchmark_routes()[req.route_index]

    _session.plant = PhysicalUAVPlant(
        _session.veh, _session.batt, _session.route["waypoints"], _session.wind, v_cruise_ms=req.v_cruise_ms
    )
    _session.ukf = bridge.init_ukf(1.0, _session.batt, _session.wind)
    _session.run_id = _db.log_mission_run(
        f"LIVE: {_session.route['name']}", req.v_cruise_ms, req.wind_north_ms, 1.0, 0.45, 0.55, 21.0, "GO_NOMINAL"
    )
    _session.active = True
    return {"status": "started", "run_id": _session.run_id, "route": _session.route["name"]}


@app.post("/api/v1/twin/session/step")
def twin_step(req: TwinStepRequest):
    if not _session.active:
        raise HTTPException(status_code=400, detail="No active twin session. Call /api/v1/twin/session/start first.")
    return _advance_session(dt_s=req.dt_s, mc_runs=req.mc_runs)


@app.post("/api/v1/twin/session/inject-fault")
def twin_inject_fault(req: FaultInjectRequest):
    if not _session.active:
        raise HTTPException(status_code=400, detail="No active twin session.")
    _session.plant.inject_fault(req.fault_type, req.magnitude)
    if _session.run_id is not None:
        _db.log_fault(_session.run_id, _session.plant.sim_time_s, req.fault_type, req.magnitude)
    return {"status": "fault_injected", "fault_type": req.fault_type, "magnitude": req.magnitude}


@app.websocket("/ws/twin/telemetry")
async def twin_telemetry_ws(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            if _session.active and not _session.plant.finished and not _session.plant.brownout:
                payload = _advance_session(dt_s=5.0, mc_runs=250)
                await ws.send_json(payload)
            await asyncio.sleep(0.4)
    except WebSocketDisconnect:
        return


@app.get("/api/v1/historian/runs")
def historian_runs(limit: int = 25):
    return {"runs": _db.list_runs(limit=limit)}


@app.get("/api/v1/historian/runs/{run_id}/epochs")
def historian_epochs(run_id: int):
    return {"run_id": run_id, "epochs": _db.get_run_epochs(run_id)}


@app.post("/api/v1/export/figures")
def api_export_figures(req: ExportRequest):
    out_dir = export_all_publication_figures(output_dir=req.output_dir, dpi=req.dpi, n_runs=req.n_runs)
    files = sorted(f for f in os.listdir(out_dir) if f.endswith(".png"))
    return {"status": "exported", "output_dir": out_dir, "count": len(files), "files": files}


@app.get("/api/v1/export/figures.zip")
def api_download_figures_zip():
    out_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "output_figures"))
    if not os.path.isdir(out_dir) or not any(f.endswith(".png") for f in os.listdir(out_dir)):
        export_all_publication_figures(output_dir=out_dir, dpi=300, n_runs=1000)
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
        for fname in sorted(os.listdir(out_dir)):
            if fname.endswith(".png"):
                zf.write(os.path.join(out_dir, fname), arcname=fname)
    buf.seek(0)
    headers = {"Content-Disposition": 'attachment; filename="mission_twin_300dpi_figures.zip"'}
    return StreamingResponse(buf, media_type="application/zip", headers=headers)


# Serve exported 300 DPI publication PNGs for inline browser preview and download
_figures_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "output_figures"))
os.makedirs(_figures_dir, exist_ok=True)
app.mount("/figures", StaticFiles(directory=_figures_dir), name="figures")

# Serve Figma-designed HTML/CSS/JS frontend on root if index.html exists
_frontend_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "frontend"))
if os.path.exists(os.path.join(_frontend_dir, "index.html")):
    app.mount("/", StaticFiles(directory=_frontend_dir, html=True), name="frontend")

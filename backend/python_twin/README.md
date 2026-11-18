# Python Digital Twin Orchestration, Risk Analytics, Historian & FastAPI Service

`backend/python_twin` wraps the compiled C++20 engine (`libdigital_twin.so`) via `ctypes` and provides high-level mission risk quantification, multi-objective Pareto planning, SQLite Digital Thread black-box recording, publication plotting, and an async REST/WebSocket API for the HTML/CSS/JS Mission Ops frontend.

---

## Package Layout

| Subpackage / File | Responsibility |
| :--- | :--- |
| `bridge/cpp_loader.py` | Loads `libdigital_twin.so` via standard-library `ctypes`, defines matching `ctypes.Structure` layouts for `twin_types.hpp`, and handles zero-copy NumPy buffer passing for Monte Carlo rollout arrays. |
| `simulation/telemetry_plant.py` | Physical UAV truth simulator generating noisy onboard sensor streams ($V_a, V_g, h, V_{\text{term}}, I, T$) along 3D routes, supporting live mid-flight fault injection (`icing_drag`, `battery_aging`, `wind_shift`). |
| `risk_planning/cvar_metrics.py` | Post-processes C++ rollout distributions to compute $P(\text{success})$, $\text{VaR}_{95\%}$, coherent tail risk **$\text{CVaR}_{95\%}$** (Rockafellar & Uryasev 2000; Majumdar & Pavone 2019), Gaussian KDE probability densities, and dynamic Point-of-No-Return (PNR). |
| `risk_planning/pareto_planner.py` | Sweeps cruise airspeeds ($V_a \in [14, 26]\,\text{m/s}$) and 3D routes through the C++ OpenMP engine to construct the continuous **3D Energy–Time–CVaR Pareto Frontier** and issue prescriptive **GO / SPEED-OPTIMIZE / RE-ROUTE / ABORT-TO-RTB** advisories (Tezzele et al. 2024). |
| `historian/digital_thread_db.py` | Embedded SQLite relational black-box (`mission_digital_thread.db`) storing `mission_runs`, `telemetry_epochs` (truth vs. UKF estimates & $3\sigma$ bounds), and `injected_faults` for post-flight SQL forensics. |
| `api/schemas.py`<br>`api/server.py` | FastAPI async service exposing typed Pydantic REST endpoints, WebSocket streaming (`/ws/twin/telemetry`), auto-generated Swagger documentation (`/docs`), and static hosting for the HTML/CSS/JS frontend. |
| `export/export_plots.py` | Headless 300 DPI Matplotlib exporter generating all 11 upgraded baseline mission plots plus 3 new Digital Twin UKF assimilation and fault-response figures into `output_figures/`. |
| `tests/` | Automated unit tests (`test_cpp_bridge.py`, `test_wind_rtb_physics.py`, `test_ukf_assimilation.py`) validating FFI marshalling, physics symmetry, and online anomaly tracking. |

---

## Running Tests & Services

```bash
# Run Python unit test suite
python3 -m unittest discover -s backend/python_twin/tests -v

# Batch-export all upgraded publication figures (300 DPI)
python3 -m backend.python_twin.export.export_plots

# Start FastAPI Digital Twin server (OpenAPI docs at http://localhost:8000/docs)
uvicorn backend.python_twin.api.server:app --reload --port 8000
```

# Mission-Level Digital Twin Frontend (Figma to HTML5 / CSS3 / Vanilla JS)

This directory contains the browser-based Mission Operations Console for the UAV Digital Twin. It is designed in **Figma** ([Live Figma Design System & Ops Console](https://www.figma.com/design/iHhRuAaPgPHVQPTT3bxsxU)) and implemented using pure **HTML5, CSS3, and Vanilla JavaScript** with **Plotly.js** and native WebSockets, served directly by the FastAPI backend on `http://localhost:8000` with zero build tooling or Node.js dependencies.

---

## Directory Structure

```text
frontend/
├── README.md
├── index.html            # 4-view aerospace mission operations layout
├── css/
│   ├── tokens.css        # Design tokens (:root variables) exported from Figma
│   └── styles.css        # Cockpit grid, HUD telemetry cards, fault injection bar
└── js/
    ├── api_client.js     # REST fetch() wrappers and /ws/twin/telemetry WebSocket client
    ├── plots.js          # Plotly.js 3D Pareto, Monte Carlo EoD PDF, and UKF 3-sigma plots
    ├── map_canvas.js     # HTML5 Canvas tactical corridor, UAV position, and wind vectors
    └── app.js            # UI state router, fault triggers, and SQL historian table
```

---

## Operational Views

1. **Pre-Flight Risk & Feasibility**: Route A/B/C comparison, SOC and terminal voltage sag profiles, Monte Carlo End-of-Discharge distributions, $P(\text{success})$, and $\text{CVaR}_{95\%}$.
2. **3D Energy–Time–CVaR Pareto Explorer**: Interactive rotatable 3D Pareto frontier across cruise airspeeds and routes with knee-point inspection.
3. **Live Bidirectional Digital Twin**: Real-time WebSocket streaming of physical vs. C++ UKF estimated states with interactive fault injection buttons (icing drag, battery resistance jump, wind shear) and live GO / SPEED-OPTIMIZE / ABORT-TO-RTB advisories.
4. **Digital Thread SQL Historian**: Queries and compares past mission runs logged in SQLite (`mission_digital_thread.db`) and triggers 300 DPI publication figure exports.

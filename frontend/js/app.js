// Main UI controller binding controls, WebSocket stream, plots, and SQL historian

let activeSocket = null;
let cachedRoutes = [];
let cachedPareto = null;
const ukfHistory = { t: [], trueR0: [], estR0: [], trueCd: [], estCd: [] };

function readParams() {
    return {
        v_cruise_ms: parseFloat(document.getElementById("inp-vcruise").value),
        v_loiter_ms: 14.0,
        wind_north_ms: parseFloat(document.getElementById("inp-windn").value),
        wind_east_ms: parseFloat(document.getElementById("inp-winde").value),
        gust_sigma_ms: parseFloat(document.getElementById("inp-gust").value),
        r0_ohm: parseFloat(document.getElementById("inp-r0").value) / 1000.0,
        n_runs: parseInt(document.getElementById("inp-nruns").value, 10),
        include_rtb: true,
    };
}

function setAdvisoryBadge(advisory) {
    const el = document.getElementById("global-advisory");
    el.textContent = advisory.code;
    el.className = "status-pill " + advisory.severity.toLowerCase();
}

function bindSliderReadouts() {
    const pairs = [
        ["inp-vcruise", "val-vcruise", (v) => `${parseFloat(v).toFixed(1)} m/s`],
        ["inp-windn", "val-windn", (v) => `${parseFloat(v).toFixed(1)} m/s`],
        ["inp-winde", "val-winde", (v) => `${parseFloat(v).toFixed(1)} m/s`],
        ["inp-gust", "val-gust", (v) => `${parseFloat(v).toFixed(1)} m/s`],
        ["inp-r0", "val-r0", (v) => `${parseInt(v, 10)} mOhm`],
        ["inp-nruns", "val-nruns", (v) => `${parseInt(v, 10)}`],
    ];
    pairs.forEach(([inpId, lblId, fmt]) => {
        const inp = document.getElementById(inpId);
        const lbl = document.getElementById(lblId);
        inp.addEventListener("input", () => { lbl.textContent = fmt(inp.value); });
    });
}

function switchTab(tabName) {
    document.querySelectorAll(".tab-btn").forEach((b) => b.classList.toggle("active", b.dataset.tab === tabName));
    document.querySelectorAll(".view-section").forEach((s) => s.classList.toggle("active", s.id === `view-${tabName}`));
    const target = document.getElementById(`view-${tabName}`);
    target.querySelectorAll(".js-plotly-plot").forEach((el) => Plotly.Plots.resize(el));
    if (tabName === "pareto") {
        if (cachedPareto) {
            TwinPlots.renderPareto3D(cachedPareto, document.getElementById("sel-pareto-mode").value);
        } else {
            runPareto().catch(() => {});
        }
    }
    if (tabName === "historian") loadHistorianTable();
}

function bindTabs() {
    document.querySelectorAll(".tab-btn").forEach((btn) => {
        btn.addEventListener("click", () => switchTab(btn.dataset.tab));
    });
}

async function runPreflight() {
    const data = await TwinAPI.evaluatePreflight(readParams());
    cachedRoutes = data.routes;
    const mid = data.routes[1] || data.routes[0];
    document.getElementById("kpi-psucc").textContent = `${(mid.risk.p_success * 100).toFixed(1)} %`;
    document.getElementById("kpi-cvar").textContent = `${(mid.risk.cvar_95_soc * 100).toFixed(1)} %`;
    document.getElementById("kpi-vterm").textContent = `${mid.risk.min_vterm_p05.toFixed(2)} V`;
    document.getElementById("kpi-time").textContent = `${(mid.risk.time_mean_s / 60).toFixed(1)} min`;
    setAdvisoryBadge(mid.advisory);

    TwinPlots.renderSocVoltage(data.routes);
    TwinPlots.renderMonteCarloPdf(data.routes);
    TwinPlots.renderRouteBars(data.routes);
    TwinPlots.renderPowerCurve(data.power_curve);
}

async function runPareto() {
    const p = readParams();
    cachedPareto = await TwinAPI.computePareto({
        wind_north_ms: p.wind_north_ms,
        wind_east_ms: p.wind_east_ms,
        gust_sigma_ms: p.gust_sigma_ms,
        r0_ohm: p.r0_ohm,
        n_runs_per_point: 350,
    });
    const mode = document.getElementById("sel-pareto-mode").value;
    TwinPlots.renderPareto3D(cachedPareto, mode);
}

async function startLiveTwin() {
    if (activeSocket) activeSocket.close();
    switchTab("livetwin");
    ukfHistory.t = []; ukfHistory.trueR0 = []; ukfHistory.estR0 = []; ukfHistory.trueCd = []; ukfHistory.estCd = [];

    const p = readParams();
    const routeIdx = parseInt(document.getElementById("sel-live-route").value, 10);
    await TwinAPI.startTwinSession({
        route_index: routeIdx,
        v_cruise_ms: p.v_cruise_ms,
        wind_north_ms: p.wind_north_ms,
        wind_east_ms: p.wind_east_ms,
        gust_sigma_ms: p.gust_sigma_ms,
    });

    activeSocket = TwinAPI.openTelemetrySocket((msg) => {
        const t = msg.truth;
        const u = msg.ukf;
        document.getElementById("live-soc").textContent = `${(u.soc * 100).toFixed(1)} %`;
        document.getElementById("live-soc-std").textContent = `3-sigma: +/- ${(u.std[0] * 300).toFixed(2)} %`;
        document.getElementById("live-r0").textContent = `${(u.r0_ohm * 1000).toFixed(1)} mOhm`;
        document.getElementById("live-r0-true").textContent = `Truth: ${(t.true_r0_ohm * 1000).toFixed(1)} mOhm`;
        document.getElementById("live-cd0").textContent = `${u.dcd0.toFixed(4)}`;
        document.getElementById("live-cd0-true").textContent = `Truth: ${t.true_dcd0.toFixed(4)}`;
        document.getElementById("live-psucc").textContent = `${(msg.live_risk.p_success * 100).toFixed(1)} %`;
        document.getElementById("live-adv-msg").textContent = msg.advisory.code;
        setAdvisoryBadge(msg.advisory);

        ukfHistory.t.push(t.sim_time_s / 60.0);
        ukfHistory.trueR0.push(t.true_r0_ohm * 1000.0);
        ukfHistory.estR0.push(u.r0_ohm * 1000.0);
        ukfHistory.trueCd.push(t.true_dcd0);
        ukfHistory.estCd.push(u.dcd0);
        TwinPlots.renderLiveUkf(ukfHistory);

        const wps = cachedRoutes[routeIdx] ? cachedRoutes[routeIdx].waypoints : [[0, 0, 50], [29000, 1000, 220]];
        TacticalMap.draw("tactical-canvas", wps, t, { wn: u.wind_n, we: u.wind_e });
    }, () => {
        activeSocket = null;
    });
}

async function loadHistorianTable() {
    const data = await TwinAPI.listHistorianRuns(25);
    const tbody = document.getElementById("historian-tbody");
    tbody.innerHTML = data.runs.map((r) => `
        <tr>
            <td>#${r.run_id}</td>
            <td>${r.timestamp_utc.slice(0, 19).replace("T", " ")}</td>
            <td>${r.route_name}</td>
            <td>${r.airspeed_ms.toFixed(1)}</td>
            <td>${(r.p_success * 100).toFixed(1)}%</td>
            <td>${(r.cvar_95_soc * 100).toFixed(1)}%</td>
            <td>${r.min_vterm_p05.toFixed(2)} V</td>
            <td>${r.advisory_code}</td>
        </tr>
    `).join("");
}

window.addEventListener("DOMContentLoaded", () => {
    bindSliderReadouts();
    bindTabs();
    document.getElementById("btn-run-preflight").addEventListener("click", runPreflight);
    document.getElementById("btn-run-pareto").addEventListener("click", runPareto);
    document.getElementById("sel-pareto-mode").addEventListener("change", () => {
        if (cachedPareto) TwinPlots.renderPareto3D(cachedPareto, document.getElementById("sel-pareto-mode").value);
    });
    document.getElementById("btn-twin-start").addEventListener("click", startLiveTwin);
    document.getElementById("btn-twin-stop").addEventListener("click", () => {
        if (activeSocket) {
            activeSocket.close();
            activeSocket = null;
        }
    });
    const ensureLive = async () => {
        if (!activeSocket || activeSocket.readyState !== WebSocket.OPEN) await startLiveTwin();
    };
    document.getElementById("btn-fault-icing").addEventListener("click", async () => {
        await ensureLive();
        await TwinAPI.injectFault("icing_drag", 0.010);
    });
    document.getElementById("btn-fault-aging").addEventListener("click", async () => {
        await ensureLive();
        await TwinAPI.injectFault("battery_aging", 0.045);
    });
    document.getElementById("btn-fault-wind").addEventListener("click", async () => {
        await ensureLive();
        await TwinAPI.injectFault("wind_shift", -4.0);
    });
    document.getElementById("btn-refresh-sql").addEventListener("click", loadHistorianTable);
    document.getElementById("btn-export-figs").addEventListener("click", async () => {
        const btn = document.getElementById("btn-export-figs");
        const banner = document.getElementById("export-status");
        const gallery = document.getElementById("export-gallery");
        btn.disabled = true;
        btn.textContent = "Exporting 12 PNGs...";
        banner.style.display = "block";
        banner.className = "export-banner";
        banner.textContent = "Generating 12 publication-grade 300 DPI figures via C++ OpenMP + Matplotlib...";
        try {
            const res = await TwinAPI.exportFigures(300, 1000);
            const ts = Date.now();
            banner.className = "export-banner success";
            banner.textContent = `Exported ${res.count} figures (300 DPI) to ${res.output_dir}`;
            document.getElementById("global-advisory").textContent = "EXPORTED_300DPI";
            gallery.style.display = "grid";
            gallery.innerHTML = (res.files || []).map((f) => `
                <a class="export-thumb" href="/figures/${f}" target="_blank" download="${f}">
                    <img src="/figures/${f}?t=${ts}" alt="${f}">
                    <span>${f}</span>
                </a>
            `).join("");
        } catch (err) {
            banner.className = "export-banner error";
            banner.textContent = `Export failed: ${err.message}`;
        } finally {
            btn.disabled = false;
            btn.textContent = "Export 300 DPI Figures";
        }
    });
    // Auto-run initial pre-flight sweep on page load
    runPreflight().catch(() => {});
});

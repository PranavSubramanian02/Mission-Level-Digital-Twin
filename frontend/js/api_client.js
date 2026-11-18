// REST and WebSocket client for FastAPI Digital Twin backend

const TwinAPI = {
    async postJson(path, body) {
        const res = await fetch(path, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(body),
        });
        if (!res.ok) throw new Error(`HTTP ${res.status}: ${await res.text()}`);
        return res.json();
    },

    async getJson(path) {
        const res = await fetch(path);
        if (!res.ok) throw new Error(`HTTP ${res.status}: ${await res.text()}`);
        return res.json();
    },

    evaluatePreflight(params) {
        return this.postJson("/api/v1/preflight/evaluate", params);
    },

    computePareto(params) {
        return this.postJson("/api/v1/preflight/pareto", params);
    },

    startTwinSession(params) {
        return this.postJson("/api/v1/twin/session/start", params);
    },

    injectFault(faultType, magnitude) {
        return this.postJson("/api/v1/twin/session/inject-fault", {
            fault_type: faultType,
            magnitude: magnitude,
        });
    },

    listHistorianRuns(limit = 25) {
        return this.getJson(`/api/v1/historian/runs?limit=${limit}`);
    },

    exportFigures(dpi = 300, nRuns = 1000) {
        return this.postJson("/api/v1/export/figures", { dpi: dpi, n_runs: nRuns });
    },

    openTelemetrySocket(onMessage, onClose) {
        const proto = window.location.protocol === "https:" ? "wss" : "ws";
        const ws = new WebSocket(`${proto}://${window.location.host}/ws/twin/telemetry`);
        ws.onmessage = (evt) => onMessage(JSON.parse(evt.data));
        ws.onclose = () => { if (onClose) onClose(); };
        return ws;
    },
};

// Plotly.js dark-aerospace visualizations for risk, 3D Pareto, and live UKF

const TwinPlots = {
    baseLayout(xTitle, yTitle) {
        return {
            paper_bgcolor: "rgba(0,0,0,0)",
            plot_bgcolor: "#070b13",
            font: { color: "#cbd5e1", family: "Segoe UI, Inter, sans-serif", size: 11 },
            margin: { l: 50, r: 45, t: 12, b: 40 },
            xaxis: { title: xTitle, gridcolor: "#1e293b", zerolinecolor: "#334155" },
            yaxis: { title: yTitle, gridcolor: "#1e293b", zerolinecolor: "#334155" },
            legend: { orientation: "h", y: 1.12, font: { size: 10 } },
        };
    },

    renderSocVoltage(routes) {
        const colors = ["#38bdf8", "#10b981", "#f59e0b"];
        const traces = routes.map((r, i) => ({
            x: r.nominal.time_min,
            y: r.nominal.soc.map((s) => s * 100.0),
            mode: "lines",
            name: `${r.name} SOC`,
            line: { color: colors[i % colors.length], width: 2.2 },
        }));
        const rB = routes[1] || routes[0];
        traces.push({
            x: rB.nominal.time_min,
            y: rB.nominal.v_term_v,
            mode: "lines",
            name: "Route B Vterm [V]",
            yaxis: "y2",
            line: { color: "#8b5cf6", width: 1.6, dash: "dot" },
        });
        const layout = this.baseLayout("Flight Time [min]", "SOC [%]");
        layout.yaxis2 = { title: "Voltage [V]", overlaying: "y", side: "right", gridcolor: "rgba(0,0,0,0)" };
        Plotly.newPlot("plot-soc-voltage", traces, layout, { responsive: true });
    },

    renderMonteCarloPdf(routes) {
        const colors = ["#38bdf8", "#10b981", "#f59e0b"];
        const traces = routes.map((r, i) => ({
            x: r.risk.kde_x.map((v) => v * 100.0),
            y: r.risk.kde_pdf,
            mode: "lines",
            fill: "tozeroy",
            name: r.name,
            line: { color: colors[i % colors.length], width: 2 },
        }));
        const layout = this.baseLayout("Terminal SOC [%]", "Density");
        layout.shapes = [{
            type: "line", x0: 25, x1: 25, y0: 0, y1: 1, yref: "paper",
            line: { color: "#ef4444", width: 2, dash: "dash" },
        }];
        Plotly.newPlot("plot-mc-pdf", traces, layout, { responsive: true });
    },

    renderRouteBars(routes) {
        const names = routes.map((r) => r.name);
        const traces = [
            { x: names, y: routes.map((r) => r.risk.p_success * 100.0), name: "P(Success) [%]", type: "bar", marker: { color: "#10b981" } },
            { x: names, y: routes.map((r) => r.risk.cvar_95_soc * 100.0), name: "CVaR-95 SOC [%]", type: "bar", marker: { color: "#38bdf8" } },
        ];
        const layout = this.baseLayout("", "Percent [%]");
        layout.barmode = "group";
        Plotly.newPlot("plot-route-bars", traces, layout, { responsive: true });
    },

    renderPowerCurve(powerCurve) {
        const trace = [{
            x: powerCurve.airspeed_ms,
            y: powerCurve.power_w,
            mode: "lines",
            name: "Electrical Power [W]",
            line: { color: "#38bdf8", width: 2.5 },
        }];
        Plotly.newPlot("plot-power-curve", trace, this.baseLayout("True Airspeed [m/s]", "Power [W]"), { responsive: true });
    },

    renderPareto3D(data, mode = "3d") {
        const container = document.getElementById("plot-pareto-3d");
        const all = data.all_candidates;
        const front = data.pareto_points;
        const knee = data.knee_point;

        if (mode === "2d") {
            container.innerHTML = "";
            const traces = [
                {
                    type: "scatter", mode: "markers", name: "Swept Candidates",
                    x: all.map((c) => c.time_min),
                    y: all.map((c) => -c.energy_cost * 100.0),
                    text: all.map((c) => `${c.route_name}<br>Va: ${c.airspeed_ms.toFixed(1)} m/s<br>P(Succ): ${(c.p_success * 100).toFixed(1)}%<br>CVaR Deficit: ${(c.cvar_risk * 100).toFixed(1)}%`),
                    hoverinfo: "text",
                    marker: { size: 8, color: all.map((c) => c.cvar_risk * 100.0), colorscale: "Portland", opacity: 0.45, showscale: false },
                },
                {
                    type: "scatter", mode: "markers", name: "Pareto Non-Dominated",
                    x: front.map((c) => c.time_min),
                    y: front.map((c) => -c.energy_cost * 100.0),
                    text: front.map((c) => `PARETO: ${c.route_name}<br>Va: ${c.airspeed_ms.toFixed(1)} m/s<br>P(Succ): ${(c.p_success * 100).toFixed(1)}%<br>CVaR Deficit: ${(c.cvar_risk * 100).toFixed(1)}%`),
                    hoverinfo: "text",
                    marker: {
                        size: 11,
                        color: front.map((c) => c.cvar_risk * 100.0),
                        colorscale: "Portland",
                        colorbar: { title: "CVaR-95% Deficit [%]", tickfont: { color: "#cbd5e1" } },
                        line: { color: "#f8fafc", width: 1.5 },
                    },
                },
            ];
            if (knee) {
                traces.push({
                    type: "scatter", mode: "markers", name: `Knee (${knee.route_name}, ${knee.airspeed_ms.toFixed(1)} m/s)`,
                    x: [knee.time_min], y: [-knee.energy_cost * 100.0],
                    marker: { size: 15, color: "#ef4444", symbol: "star", line: { color: "#ffffff", width: 1.5 } },
                });
            }
            const layout = this.baseLayout("Mean Flight Time [min]", "Mean Energy Margin [%]");
            layout.margin = { l: 58, r: 80, t: 28, b: 48 };
            Plotly.newPlot("plot-pareto-3d", traces, layout, { responsive: true });
            return;
        }

        // Zero-WebGL interactive 3D perspective canvas (works in VS Code Simple Browser & all browsers)
        Plotly.purge(container);
        container.innerHTML = "";
        const canvas = document.createElement("canvas");
        canvas.style.width = "100%";
        canvas.style.height = "100%";
        canvas.style.display = "block";
        canvas.style.cursor = "grab";
        container.appendChild(canvas);

        const rect = container.getBoundingClientRect();
        canvas.width = Math.max(rect.width, 720);
        canvas.height = Math.max(rect.height, 460);
        const ctx = canvas.getContext("2d");

        const xs = all.map((c) => c.energy_cost * 100.0);
        const ys = all.map((c) => c.time_min);
        const zs = all.map((c) => c.cvar_risk * 100.0);
        const xMin = Math.min(...xs), xMax = Math.max(...xs) || 1;
        const yMin = Math.min(...ys), yMax = Math.max(...ys) || 1;
        const zMin = 0.0, zMax = Math.max(...zs, 1.0);
        const norm = (v, lo, hi) => ((v - lo) / Math.max(hi - lo, 1e-6)) * 2.0 - 1.0;

        let yaw = -0.68;
        let pitch = 0.42;
        let dragging = false;
        let lastX = 0, lastY = 0;

        const project = (nx, ny, nz) => {
            const cy = Math.cos(yaw), sy = Math.sin(yaw);
            const cp = Math.cos(pitch), sp = Math.sin(pitch);
            const x1 = nx * cy - ny * sy;
            const y1 = nx * sy + ny * cy;
            const z2 = nz * cp - y1 * sp;
            const y2 = nz * sp + y1 * cp;
            const persp = 3.2 / (4.4 - y2);
            const scale = Math.min(canvas.width, canvas.height) * 0.34;
            return {
                sx: canvas.width * 0.48 + x1 * scale * persp,
                sy: canvas.height * 0.54 - z2 * scale * persp,
                depth: y2,
            };
        };

        const drawScene = () => {
            const w = canvas.width, h = canvas.height;
            ctx.fillStyle = "#070b13";
            ctx.fillRect(0, 0, w, h);

            // 3D wireframe bounding cube & floor grid
            const corners = [
                [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
                [-1, -1, 1],  [1, -1, 1],  [1, 1, 1],  [-1, 1, 1],
            ].map((c) => project(c[0], c[1], c[2]));
            const edges = [[0,1],[1,2],[2,3],[3,0],[4,5],[5,6],[6,7],[7,4],[0,4],[1,5],[2,6],[3,7]];
            ctx.strokeStyle = "#1e293b";
            ctx.lineWidth = 1.1;
            edges.forEach(([a, b]) => {
                ctx.beginPath(); ctx.moveTo(corners[a].sx, corners[a].sy); ctx.lineTo(corners[b].sx, corners[b].sy); ctx.stroke();
            });

            // Highlight primary axes (Energy, Time, CVaR)
            const axes = [
                [0, 1, "#38bdf8", `Energy Deficit (${xMin.toFixed(0)}..${xMax.toFixed(0)}%)`],
                [0, 3, "#10b981", `Flight Time (${yMin.toFixed(0)}..${yMax.toFixed(0)} min)`],
                [0, 4, "#f59e0b", `CVaR-95 Deficit (0..${zMax.toFixed(0)}%)`],
            ];
            ctx.font = "11px monospace";
            axes.forEach(([a, b, col, lbl]) => {
                ctx.strokeStyle = col; ctx.lineWidth = 2;
                ctx.beginPath(); ctx.moveTo(corners[a].sx, corners[a].sy); ctx.lineTo(corners[b].sx, corners[b].sy); ctx.stroke();
                ctx.fillStyle = col;
                ctx.fillText(lbl, corners[b].sx + 6, corners[b].sy - 4);
            });

            // Collect and depth-sort 3D scatter markers
            const frontSet = new Set(front.map((p) => `${p.route_name}:${p.airspeed_ms.toFixed(2)}`));
            const pts = all.map((c) => {
                const pr = project(norm(c.energy_cost * 100, xMin, xMax), norm(c.time_min, yMin, yMax), norm(c.cvar_risk * 100, zMin, zMax));
                const isFront = frontSet.has(`${c.route_name}:${c.airspeed_ms.toFixed(2)}`);
                const isKnee = knee && c.route_name === knee.route_name && Math.abs(c.airspeed_ms - knee.airspeed_ms) < 1e-4;
                return { ...pr, c, isFront, isKnee };
            }).sort((a, b) => a.depth - b.depth);

            pts.forEach((p) => {
                ctx.beginPath();
                if (p.isKnee) {
                    ctx.fillStyle = "#ef4444";
                    ctx.arc(p.sx, p.sy, 8, 0, Math.PI * 2);
                    ctx.fill();
                    ctx.strokeStyle = "#ffffff"; ctx.lineWidth = 2; ctx.stroke();
                } else if (p.isFront) {
                    ctx.fillStyle = "#38bdf8";
                    ctx.arc(p.sx, p.sy, 5.5, 0, Math.PI * 2);
                    ctx.fill();
                    ctx.strokeStyle = "#f8fafc"; ctx.lineWidth = 1.2; ctx.stroke();
                } else {
                    ctx.fillStyle = p.c.cvar_risk > 0.05 ? "rgba(245, 158, 11, 0.55)" : "rgba(148, 163, 184, 0.45)";
                    ctx.arc(p.sx, p.sy, 4, 0, Math.PI * 2);
                    ctx.fill();
                }
            });

            // Legend overlay
            ctx.fillStyle = "#cbd5e1";
            ctx.font = "11px Segoe UI, sans-serif";
            ctx.fillText("Drag to Orbit 3D Cube  |  Cyan: Pareto Front  |  Red: Balanced Knee  |  Amber/Gray: Dominated", 18, 22);
            if (knee) {
                ctx.fillStyle = "#fca5a5";
                ctx.font = "11px monospace";
                ctx.fillText(`KNEE: ${knee.route_name} @ ${knee.airspeed_ms.toFixed(1)} m/s | Time=${knee.time_min.toFixed(1)}m | Margin=${(-knee.energy_cost * 100).toFixed(1)}% | CVaR=${(knee.cvar_risk * 100).toFixed(1)}%`, 18, h - 14);
            }
        };

        canvas.addEventListener("mousedown", (e) => { dragging = true; lastX = e.clientX; lastY = e.clientY; canvas.style.cursor = "grabbing"; });
        window.addEventListener("mouseup", () => { dragging = false; canvas.style.cursor = "grab"; });
        canvas.addEventListener("mousemove", (e) => {
            if (!dragging) return;
            yaw += (e.clientX - lastX) * 0.01;
            pitch = Math.max(-1.2, Math.min(1.2, pitch + (e.clientY - lastY) * 0.01));
            lastX = e.clientX; lastY = e.clientY;
            drawScene();
        });
        drawScene();
    },

    renderLiveUkf(history) {
        const t = history.t;
        const traces = [
            { x: t, y: history.trueR0, mode: "lines", name: "True R0 [mOhm]", line: { color: "#f1f5f9", width: 2 } },
            { x: t, y: history.estR0, mode: "lines", name: "UKF R0 [mOhm]", line: { color: "#10b981", width: 2, dash: "dash" } },
            { x: t, y: history.trueCd, mode: "lines", name: "True dCD0", yaxis: "y2", line: { color: "#f59e0b", width: 2 } },
            { x: t, y: history.estCd, mode: "lines", name: "UKF dCD0", yaxis: "y2", line: { color: "#38bdf8", width: 2, dash: "dash" } },
        ];
        const layout = this.baseLayout("Elapsed Time [min]", "R0 [mOhm]");
        layout.yaxis2 = { title: "dCD0", overlaying: "y", side: "right", gridcolor: "rgba(0,0,0,0)" };
        Plotly.newPlot("plot-ukf-live", traces, layout, { responsive: true });
    },
};

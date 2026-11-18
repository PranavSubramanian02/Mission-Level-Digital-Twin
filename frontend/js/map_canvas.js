// HTML5 Canvas tactical corridor map rendering waypoints, UAV track, and wind arrow

const TacticalMap = {
    draw(canvasId, waypoints, uavPos, windVec) {
        const canvas = document.getElementById(canvasId);
        if (!canvas || !waypoints || waypoints.length === 0) return;

        const rect = canvas.getBoundingClientRect();
        canvas.width = Math.max(rect.width, 400);
        canvas.height = Math.max(rect.height, 300);
        const ctx = canvas.getContext("2d");
        const w = canvas.width;
        const h = canvas.height;

        ctx.fillStyle = "#070b13";
        ctx.fillRect(0, 0, w, h);

        // Background radar grid
        ctx.strokeStyle = "#162033";
        ctx.lineWidth = 1;
        for (let x = 0; x < w; x += 40) {
            ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
        }
        for (let y = 0; y < h; y += 40) {
            ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
        }

        // Compute coordinate bounds (East = X, North = Y inverted)
        const easts = waypoints.map((p) => p[1]);
        const norths = waypoints.map((p) => p[0]);
        const minE = Math.min(...easts) - 2500;
        const maxE = Math.max(...easts) + 2500;
        const minN = Math.min(...norths) - 2500;
        const maxN = Math.max(...norths) + 2500;
        const pad = 32;

        const toX = (e) => pad + ((e - minE) / Math.max(maxE - minE, 1)) * (w - 2 * pad);
        const toY = (n) => h - pad - ((n - minN) / Math.max(maxN - minN, 1)) * (h - 2 * pad);

        // Planned 3D corridor polyline
        ctx.strokeStyle = "#38bdf8";
        ctx.lineWidth = 2;
        ctx.setLineDash([6, 4]);
        ctx.beginPath();
        waypoints.forEach((wp, idx) => {
            const px = toX(wp[1]);
            const py = toY(wp[0]);
            if (idx === 0) ctx.moveTo(px, py);
            else ctx.lineTo(px, py);
        });
        ctx.stroke();
        ctx.setLineDash([]);

        // Waypoint markers
        waypoints.forEach((wp, idx) => {
            const px = toX(wp[1]);
            const py = toY(wp[0]);
            ctx.fillStyle = idx === 0 ? "#10b981" : "#38bdf8";
            ctx.beginPath();
            ctx.arc(px, py, 5, 0, Math.PI * 2);
            ctx.fill();
            ctx.fillStyle = "#cbd5e1";
            ctx.font = "11px monospace";
            ctx.fillText(idx === 0 ? "BASE" : `WP${idx} (${wp[2]}m)`, px + 8, py - 6);
        });

        // Active UAV marker
        if (uavPos) {
            const ux = toX(uavPos.east_m);
            const uy = toY(uavPos.north_m);
            ctx.fillStyle = "#ef4444";
            ctx.beginPath();
            ctx.arc(ux, uy, 7, 0, Math.PI * 2);
            ctx.fill();
            ctx.strokeStyle = "#ffffff";
            ctx.lineWidth = 2;
            ctx.stroke();
        }

        // Estimated wind vector indicator (top-right corner)
        if (windVec) {
            const cx = w - 58;
            const cy = 50;
            const scale = 3.2;
            ctx.strokeStyle = "#f59e0b";
            ctx.lineWidth = 2.5;
            ctx.beginPath();
            ctx.moveTo(cx, cy);
            ctx.lineTo(cx + windVec.we * scale, cy - windVec.wn * scale);
            ctx.stroke();
            ctx.fillStyle = "#f59e0b";
            ctx.font = "10px monospace";
            ctx.fillText(`WIND (${windVec.wn.toFixed(1)}, ${windVec.we.toFixed(1)})`, w - 135, 22);
        }
    },
};

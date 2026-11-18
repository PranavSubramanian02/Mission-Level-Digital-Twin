# SQLite Digital Thread black-box historian for mission runs, UKF epochs, and faults

import datetime
import os
import sqlite3


class DigitalThreadDB:
    # Embedded SQL black-box logger for post-flight forensics and replay
    def __init__(self, db_path: str = None):
        if db_path is None:
            base = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
            db_path = os.path.join(base, "mission_digital_thread.db")
        self.db_path = db_path
        self._init_schema()

    def _connect(self):
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        return conn

    def _init_schema(self):
        schema_sql = (
            "CREATE TABLE IF NOT EXISTS mission_runs ("
            "run_id INTEGER PRIMARY KEY AUTOINCREMENT, timestamp_utc TEXT NOT NULL, "
            "route_name TEXT NOT NULL, airspeed_ms REAL NOT NULL, wind_n_ms REAL NOT NULL, "
            "p_success REAL NOT NULL, cvar_95_soc REAL NOT NULL, soc_mean REAL NOT NULL, "
            "min_vterm_p05 REAL NOT NULL, advisory_code TEXT NOT NULL);"
            "CREATE TABLE IF NOT EXISTS telemetry_epochs ("
            "epoch_id INTEGER PRIMARY KEY AUTOINCREMENT, run_id INTEGER NOT NULL, "
            "sim_time_s REAL NOT NULL, dist_m REAL NOT NULL, alt_m REAL NOT NULL, "
            "true_soc REAL NOT NULL, est_soc REAL NOT NULL, soc_std REAL NOT NULL, "
            "true_vterm REAL NOT NULL, est_r0_ohm REAL NOT NULL, est_dcd0 REAL NOT NULL, "
            "live_p_success REAL NOT NULL, live_cvar_95_soc REAL NOT NULL, advisory_code TEXT NOT NULL);"
            "CREATE TABLE IF NOT EXISTS injected_faults ("
            "fault_id INTEGER PRIMARY KEY AUTOINCREMENT, run_id INTEGER NOT NULL, "
            "sim_time_s REAL NOT NULL, fault_type TEXT NOT NULL, magnitude REAL NOT NULL);"
        )
        with self._connect() as conn:
            conn.executescript(schema_sql)

    def log_mission_run(
        self,
        route_name: str,
        airspeed_ms: float,
        wind_n_ms: float,
        p_success: float,
        cvar_95_soc: float,
        soc_mean: float,
        min_vterm_p05: float,
        advisory_code: str,
    ) -> int:
        ts = datetime.datetime.now(datetime.timezone.utc).isoformat()
        sql = (
            "INSERT INTO mission_runs (timestamp_utc, route_name, airspeed_ms, wind_n_ms, "
            "p_success, cvar_95_soc, soc_mean, min_vterm_p05, advisory_code) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)"
        )
        with self._connect() as conn:
            cur = conn.execute(
                sql,
                (ts, route_name, airspeed_ms, wind_n_ms, p_success, cvar_95_soc, soc_mean, min_vterm_p05, advisory_code),
            )
            return int(cur.lastrowid)

    def log_epoch(self, run_id: int, epoch: dict):
        sql = (
            "INSERT INTO telemetry_epochs (run_id, sim_time_s, dist_m, alt_m, true_soc, "
            "est_soc, soc_std, true_vterm, est_r0_ohm, est_dcd0, live_p_success, "
            "live_cvar_95_soc, advisory_code) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)"
        )
        with self._connect() as conn:
            conn.execute(
                sql,
                (
                    run_id,
                    epoch["sim_time_s"],
                    epoch["dist_m"],
                    epoch["alt_m"],
                    epoch["true_soc"],
                    epoch["est_soc"],
                    epoch["soc_std"],
                    epoch["true_vterm"],
                    epoch["est_r0_ohm"],
                    epoch["est_dcd0"],
                    epoch["live_p_success"],
                    epoch["live_cvar_95_soc"],
                    epoch["advisory_code"],
                ),
            )

    def log_fault(self, run_id: int, sim_time_s: float, fault_type: str, magnitude: float):
        sql = "INSERT INTO injected_faults (run_id, sim_time_s, fault_type, magnitude) VALUES (?, ?, ?, ?)"
        with self._connect() as conn:
            conn.execute(sql, (run_id, sim_time_s, fault_type, magnitude))

    def list_runs(self, limit: int = 25) -> list:
        with self._connect() as conn:
            rows = conn.execute("SELECT * FROM mission_runs ORDER BY run_id DESC LIMIT ?", (limit,)).fetchall()
            return [dict(r) for r in rows]

    def get_run_epochs(self, run_id: int) -> list:
        with self._connect() as conn:
            rows = conn.execute("SELECT * FROM telemetry_epochs WHERE run_id = ? ORDER BY sim_time_s ASC", (run_id,)).fetchall()
            return [dict(r) for r in rows]

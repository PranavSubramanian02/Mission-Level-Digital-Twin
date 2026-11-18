# Zero-copy ctypes FFI loader for compiled C++20 libdigital_twin.so

import ctypes
import os
import numpy as np


class VehicleParamsC(ctypes.Structure):
    _fields_ = [
        ("mass_kg", ctypes.c_double),
        ("wing_area_m2", ctypes.c_double),
        ("cd0", ctypes.c_double),
        ("k_induced", ctypes.c_double),
        ("eta_prop", ctypes.c_double),
        ("p_hotel_w", ctypes.c_double),
        ("loiter_bank_rad", ctypes.c_double),
    ]


class BatteryParamsC(ctypes.Structure):
    _fields_ = [
        ("n_cells_series", ctypes.c_int),
        ("capacity_ah", ctypes.c_double),
        ("r0_ohm", ctypes.c_double),
        ("r1_ohm", ctypes.c_double),
        ("c1_farad", ctypes.c_double),
        ("v_cutoff_v", ctypes.c_double),
        ("peukert_k", ctypes.c_double),
        ("soc_reserve", ctypes.c_double),
    ]


class Waypoint3DC(ctypes.Structure):
    _fields_ = [
        ("north_m", ctypes.c_double),
        ("east_m", ctypes.c_double),
        ("alt_m", ctypes.c_double),
    ]


class WindConfigC(ctypes.Structure):
    _fields_ = [
        ("wind_north_ms", ctypes.c_double),
        ("wind_east_ms", ctypes.c_double),
        ("gust_sigma_ms", ctypes.c_double),
        ("ou_tau_s", ctypes.c_double),
        ("r0_sigma_frac", ctypes.c_double),
        ("dcd0_sigma", ctypes.c_double),
    ]


class TelemetryPacketC(ctypes.Structure):
    _fields_ = [
        ("dt_s", ctypes.c_double),
        ("airspeed_ms", ctypes.c_double),
        ("heading_rad", ctypes.c_double),
        ("climb_rate_ms", ctypes.c_double),
        ("alt_m", ctypes.c_double),
        ("bank_rad", ctypes.c_double),
        ("meas_v_term_v", ctypes.c_double),
        ("meas_vg_north_ms", ctypes.c_double),
        ("meas_vg_east_ms", ctypes.c_double),
        ("meas_p_elec_w", ctypes.c_double),
    ]


class UkfStateC(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_double * 6),
        ("P", ctypes.c_double * 36),
    ]


class SingleMissionResultC(ctypes.Structure):
    _fields_ = [
        ("final_soc", ctypes.c_double),
        ("min_v_term_v", ctypes.c_double),
        ("total_time_s", ctypes.c_double),
        ("total_dist_m", ctypes.c_double),
        ("brownout", ctypes.c_int),
        ("rtb_ok", ctypes.c_int),
    ]


class MonteCarloResultC(ctypes.Structure):
    _fields_ = [
        ("n_runs", ctypes.c_int),
        ("final_soc", ctypes.POINTER(ctypes.c_double)),
        ("min_v_term", ctypes.POINTER(ctypes.c_double)),
        ("total_time_s", ctypes.POINTER(ctypes.c_double)),
        ("brownout_flags", ctypes.POINTER(ctypes.c_int)),
        ("success_flags", ctypes.POINTER(ctypes.c_int)),
    ]


def default_vehicle() -> VehicleParamsC:
    # Realistic 10 kg Group-2 fixed-wing UAV
    return VehicleParamsC(10.0, 0.8, 0.030, 0.045, 0.70, 25.0, 0.35)


def default_battery() -> BatteryParamsC:
    # Realistic 344 Wh 6S Li-ion pack (22.2 V nom, 15.5 Ah, 18.8 V cutoff)
    return BatteryParamsC(6, 15.5, 0.085, 0.030, 600.0, 18.8, 1.08, 0.25)


def default_wind() -> WindConfigC:
    # Nominal headwind with Ornstein-Uhlenbeck gusts and aging spread
    return WindConfigC(-4.0, 1.5, 1.8, 30.0, 0.12, 0.003)


def benchmark_routes() -> list:
    # Three 3D multi-waypoint routes spanning nominal to high-risk boundary
    return [
        {
            "name": "Route A (Short ISR)",
            "waypoints": [(0.0, 0.0, 50.0), (10000.0, 2000.0, 150.0), (22000.0, 0.0, 150.0)],
            "loiter_s": 300.0,
            "tot_limit_s": 2400.0,
        },
        {
            "name": "Route B (Mid Corridor)",
            "waypoints": [(0.0, 0.0, 50.0), (14000.0, 3500.0, 220.0), (29000.0, 1000.0, 220.0)],
            "loiter_s": 360.0,
            "tot_limit_s": 3000.0,
        },
        {
            "name": "Route C (Extended Range)",
            "waypoints": [(0.0, 0.0, 50.0), (18000.0, 4000.0, 300.0), (35000.0, 1500.0, 300.0)],
            "loiter_s": 420.0,
            "tot_limit_s": 3600.0,
        },
    ]


class CppTwinBridge:
    # Wraps compiled C++20 libdigital_twin.so entrypoints for Python callers
    def __init__(self, lib_path: str = None):
        if lib_path is None:
            base = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "cpp_core", "build"))
            lib_path = os.path.join(base, "libdigital_twin.so")
        if not os.path.exists(lib_path):
            raise FileNotFoundError(f"Compiled C++ library not found at {lib_path}. Build backend/cpp_core first.")
        self.lib = ctypes.CDLL(lib_path)
        self._bind_signatures()

    def _bind_signatures(self):
        self.lib.twin_compute_power_curve.argtypes = [
            ctypes.POINTER(VehicleParamsC),
            ctypes.POINTER(ctypes.c_double),
            ctypes.c_int,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.POINTER(ctypes.c_double),
        ]
        self.lib.twin_compute_power_curve.restype = None

        self.lib.twin_run_single_mission.argtypes = [
            ctypes.POINTER(VehicleParamsC),
            ctypes.POINTER(BatteryParamsC),
            ctypes.POINTER(Waypoint3DC),
            ctypes.c_int,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.POINTER(WindConfigC),
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_int),
        ]
        self.lib.twin_run_single_mission.restype = SingleMissionResultC

        self.lib.twin_ukf_init.argtypes = [
            ctypes.POINTER(UkfStateC),
            ctypes.c_double,
            ctypes.POINTER(BatteryParamsC),
            ctypes.POINTER(WindConfigC),
        ]
        self.lib.twin_ukf_init.restype = None

        self.lib.twin_ukf_step.argtypes = [
            ctypes.POINTER(UkfStateC),
            ctypes.POINTER(TelemetryPacketC),
            ctypes.POINTER(VehicleParamsC),
            ctypes.POINTER(BatteryParamsC),
        ]
        self.lib.twin_ukf_step.restype = None

        self.lib.twin_run_monte_carlo.argtypes = [
            ctypes.POINTER(VehicleParamsC),
            ctypes.POINTER(BatteryParamsC),
            ctypes.POINTER(Waypoint3DC),
            ctypes.c_int,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.POINTER(WindConfigC),
            ctypes.c_int,
            ctypes.c_ulonglong,
            ctypes.POINTER(MonteCarloResultC),
        ]
        self.lib.twin_run_monte_carlo.restype = None

    @staticmethod
    def _to_wp_array(waypoints: list):
        arr_type = Waypoint3DC * len(waypoints)
        return arr_type(*[Waypoint3DC(float(w[0]), float(w[1]), float(w[2])) for w in waypoints])

    def compute_power_curve(self, veh: VehicleParamsC, airspeeds: np.ndarray, alt_m: float = 150.0, dcd0: float = 0.0) -> np.ndarray:
        v = np.ascontiguousarray(airspeeds, dtype=np.float64)
        out = np.zeros_like(v)
        self.lib.twin_compute_power_curve(
            ctypes.byref(veh),
            v.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            len(v),
            float(alt_m),
            float(dcd0),
            out.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        )
        return out

    def run_single_mission(
        self,
        veh: VehicleParamsC,
        batt: BatteryParamsC,
        waypoints: list,
        wind: WindConfigC,
        v_cruise_ms: float = 18.0,
        v_loiter_ms: float = 14.0,
        loiter_time_s: float = 300.0,
        ds_m: float = 100.0,
        soc_init: float = 1.0,
        vp_init: float = 0.0,
        r0_ohm: float = None,
        dcd0: float = 0.0,
        include_rtb: bool = True,
    ) -> dict:
        wp_arr = self._to_wp_array(waypoints)
        max_pts = 8000
        d_buf = np.zeros(max_pts, dtype=np.float64)
        t_buf = np.zeros(max_pts, dtype=np.float64)
        s_buf = np.zeros(max_pts, dtype=np.float64)
        v_buf = np.zeros(max_pts, dtype=np.float64)
        p_buf = np.zeros(max_pts, dtype=np.float64)
        n_out = ctypes.c_int(0)
        r0 = batt.r0_ohm if r0_ohm is None else float(r0_ohm)

        res = self.lib.twin_run_single_mission(
            ctypes.byref(veh),
            ctypes.byref(batt),
            wp_arr,
            len(waypoints),
            float(v_cruise_ms),
            float(v_loiter_ms),
            float(loiter_time_s),
            float(ds_m),
            float(soc_init),
            float(vp_init),
            r0,
            float(dcd0),
            ctypes.byref(wind),
            1 if include_rtb else 0,
            d_buf.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            t_buf.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            s_buf.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            v_buf.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            p_buf.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            max_pts,
            ctypes.byref(n_out),
        )
        n = n_out.value
        return {
            "final_soc": float(res.final_soc),
            "min_v_term_v": float(res.min_v_term_v),
            "total_time_s": float(res.total_time_s),
            "total_dist_m": float(res.total_dist_m),
            "brownout": bool(res.brownout),
            "rtb_ok": bool(res.rtb_ok),
            "dist_m": d_buf[:n],
            "time_s": t_buf[:n],
            "soc": s_buf[:n],
            "v_term_v": v_buf[:n],
            "power_w": p_buf[:n],
        }

    def run_monte_carlo(
        self,
        veh: VehicleParamsC,
        batt: BatteryParamsC,
        waypoints: list,
        wind: WindConfigC,
        n_runs: int = 1000,
        v_cruise_ms: float = 18.0,
        v_loiter_ms: float = 14.0,
        loiter_time_s: float = 300.0,
        ds_m: float = 250.0,
        soc_init: float = 1.0,
        soc_std: float = 0.015,
        vp_init: float = 0.0,
        r0_mean: float = None,
        dcd0_mean: float = 0.0,
        include_rtb: bool = True,
        seed: int = 42,
    ) -> dict:
        wp_arr = self._to_wp_array(waypoints)
        f_soc = np.zeros(n_runs, dtype=np.float64)
        min_v = np.zeros(n_runs, dtype=np.float64)
        t_tot = np.zeros(n_runs, dtype=np.float64)
        brown = np.zeros(n_runs, dtype=np.int32)
        succ = np.zeros(n_runs, dtype=np.int32)

        mc_out = MonteCarloResultC(
            n_runs,
            f_soc.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            min_v.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            t_tot.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
            brown.ctypes.data_as(ctypes.POINTER(ctypes.c_int)),
            succ.ctypes.data_as(ctypes.POINTER(ctypes.c_int)),
        )
        r0 = batt.r0_ohm if r0_mean is None else float(r0_mean)

        self.lib.twin_run_monte_carlo(
            ctypes.byref(veh),
            ctypes.byref(batt),
            wp_arr,
            len(waypoints),
            float(v_cruise_ms),
            float(v_loiter_ms),
            float(loiter_time_s),
            float(ds_m),
            float(soc_init),
            float(soc_std),
            float(vp_init),
            r0,
            float(dcd0_mean),
            ctypes.byref(wind),
            1 if include_rtb else 0,
            int(seed),
            ctypes.byref(mc_out),
        )
        return {
            "final_soc": f_soc,
            "min_v_term": min_v,
            "total_time_s": t_tot,
            "brownout_flags": brown,
            "success_flags": succ,
        }

    def init_ukf(self, soc_init: float, batt: BatteryParamsC, wind: WindConfigC) -> UkfStateC:
        state = UkfStateC()
        self.lib.twin_ukf_init(ctypes.byref(state), float(soc_init), ctypes.byref(batt), ctypes.byref(wind))
        return state

    def step_ukf(self, state: UkfStateC, pkt: TelemetryPacketC, veh: VehicleParamsC, batt: BatteryParamsC) -> dict:
        self.lib.twin_ukf_step(ctypes.byref(state), ctypes.byref(pkt), ctypes.byref(veh), ctypes.byref(batt))
        x = np.array(state.x[:], dtype=np.float64)
        P = np.array(state.P[:], dtype=np.float64).reshape(6, 6)
        std = np.sqrt(np.maximum(np.diag(P), 1e-12))
        return {
            "soc": float(x[0]),
            "vp": float(x[1]),
            "r0_ohm": float(x[2]),
            "dcd0": float(x[3]),
            "wind_n": float(x[4]),
            "wind_e": float(x[5]),
            "std": std.tolist(),
        }

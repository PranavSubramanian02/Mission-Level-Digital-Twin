# Simulated physical UAV truth asset with noisy sensors and live fault injection

import math
import numpy as np
from ..bridge.cpp_loader import BatteryParamsC, TelemetryPacketC, VehicleParamsC, WindConfigC


class PhysicalUAVPlant:
    # Advances true physical UAV state along 3D route and emits noisy telemetry
    def __init__(
        self,
        veh: VehicleParamsC,
        batt: BatteryParamsC,
        waypoints: list,
        wind: WindConfigC,
        v_cruise_ms: float = 18.0,
        seed: int = 7,
    ):
        self.veh = veh
        self.batt = batt
        self.waypoints = [tuple(float(c) for c in w) for w in waypoints]
        self.v_cruise_ms = float(v_cruise_ms)
        self.rng = np.random.default_rng(seed)

        # True hidden physical state
        self.sim_time_s = 0.0
        self.dist_m = 0.0
        self.leg_idx = 0
        self.north_m, self.east_m, self.alt_m = self.waypoints[0]
        self.soc = 1.0
        self.vp = 0.0
        self.true_r0 = float(batt.r0_ohm)
        self.true_dcd0 = 0.0
        self.true_wind_n = float(wind.wind_north_ms)
        self.true_wind_e = float(wind.wind_east_ms)
        self.gust_n = 0.0
        self.gust_e = 0.0
        self.gust_sigma = float(wind.gust_sigma_ms)
        self.ou_tau = max(float(wind.ou_tau_s), 1.0)
        self.finished = False
        self.brownout = False

    def inject_fault(self, fault_type: str, magnitude: float):
        # Apply hidden physical degradation to test UKF online assimilation
        if fault_type == "icing_drag":
            self.true_dcd0 += float(magnitude)
        elif fault_type == "battery_aging":
            self.true_r0 += float(magnitude)
        elif fault_type == "wind_shift":
            self.true_wind_n += float(magnitude)

    def remaining_waypoints(self) -> list:
        # Active position plus all unvisited waypoints for live lookahead rollouts
        if self.finished:
            return [(self.north_m, self.east_m, self.alt_m)]
        return [(self.north_m, self.east_m, self.alt_m)] + self.waypoints[self.leg_idx + 1 :]

    def _true_power(self, airspeed: float, alt: float, vz: float) -> float:
        rho = 1.225 * math.exp(-max(alt, 0.0) / 8500.0)
        q = 0.5 * rho * airspeed * airspeed
        weight = self.veh.mass_kg * 9.81
        cl = weight / (q * self.veh.wing_area_m2)
        cd = max(self.veh.cd0 + self.true_dcd0, 0.005) + self.veh.k_induced * cl * cl
        p_aero = max(q * self.veh.wing_area_m2 * cd * airspeed + weight * vz, 0.0)
        return p_aero / self.veh.eta_prop + self.veh.p_hotel_w

    def _true_battery_step(self, p_elec: float, dt: float) -> tuple:
        s = min(max(self.soc, 0.0), 1.0)
        ocv = (3.15 + 0.85 * s + 0.20 * s**3 - 0.12 * (1.0 - s) ** 4) * self.batt.n_cells_series
        v_eff = ocv - self.vp
        disc = v_eff * v_eff - 4.0 * self.true_r0 * p_elec
        if disc < 0.0 or v_eff <= 0.0:
            current = max(v_eff, 0.0) / (2.0 * self.true_r0)
            v_term = 0.0
        else:
            current = (v_eff - math.sqrt(disc)) / (2.0 * self.true_r0)
            v_term = v_eff - current * self.true_r0

        i_ref = max(self.batt.capacity_ah, 0.1)
        i_eff = current * ((current / i_ref) ** (self.batt.peukert_k - 1.0)) if current > i_ref else current
        self.soc = min(max(self.soc - (i_eff * dt) / (3600.0 * self.batt.capacity_ah), 0.0), 1.0)
        tau = max(self.batt.r1_ohm * self.batt.c1_farad, 1e-3)
        alpha = math.exp(-dt / tau)
        self.vp = alpha * self.vp + self.batt.r1_ohm * (1.0 - alpha) * current
        if v_term <= self.batt.v_cutoff_v or self.soc <= 0.0:
            self.brownout = True
        return v_term, current

    def step(self, dt_s: float = 5.0) -> tuple:
        # Advance physical truth by dt_s and return (TelemetryPacketC, truth_dict)
        if self.leg_idx >= len(self.waypoints) - 1:
            self.finished = True
            target = self.waypoints[-1]
        else:
            target = self.waypoints[self.leg_idx + 1]

        dn = target[0] - self.north_m
        de = target[1] - self.east_m
        dh = target[2] - self.alt_m
        horiz = math.hypot(dn, de)

        # Update OU atmospheric gusts
        phi = math.exp(-dt_s / self.ou_tau)
        scale = self.gust_sigma * math.sqrt(max(1.0 - phi * phi, 0.0))
        self.gust_n = phi * self.gust_n + scale * float(self.rng.normal())
        self.gust_e = phi * self.gust_e + scale * float(self.rng.normal())
        wn = self.true_wind_n + self.gust_n
        we = self.true_wind_e + self.gust_e

        if horiz > 1.0 and not self.finished:
            un, ue = dn / horiz, de / horiz
            w_par = wn * un + we * ue
            w_perp_sq = (wn - w_par * un) ** 2 + (we - w_par * ue) ** 2
            vg = max(math.sqrt(max(self.v_cruise_ms**2 - w_perp_sq, 1.0)) + w_par, 2.0)
            step_d = min(vg * dt_s, horiz)
            frac = step_d / horiz
            vz = (dh * frac) / dt_s
            self.north_m += dn * frac
            self.east_m += de * frac
            self.alt_m += dh * frac
            self.dist_m += step_d
            heading = math.atan2(vg * ue - we, vg * un - wn)
            vg_n, vg_e = vg * un, vg * ue
            if step_d >= horiz - 1e-3:
                self.leg_idx += 1
                if self.leg_idx >= len(self.waypoints) - 1:
                    self.finished = True
        else:
            vz = 0.0
            heading = 0.0
            vg_n, vg_e = self.v_cruise_ms + wn, we

        p_true = self._true_power(self.v_cruise_ms, self.alt_m, vz)
        v_term_true, current_true = self._true_battery_step(p_true, dt_s)
        self.sim_time_s += dt_s

        # Inject realistic onboard sensor measurement noise
        pkt = TelemetryPacketC(
            dt_s=float(dt_s),
            airspeed_ms=float(self.v_cruise_ms),
            heading_rad=float(heading),
            climb_rate_ms=float(vz),
            alt_m=float(self.alt_m),
            bank_rad=0.0,
            meas_v_term_v=float(v_term_true + self.rng.normal(0.0, 0.03)),
            meas_vg_north_ms=float(vg_n + self.rng.normal(0.0, 0.25)),
            meas_vg_east_ms=float(vg_e + self.rng.normal(0.0, 0.25)),
            meas_p_elec_w=float(p_true + self.rng.normal(0.0, 4.0)),
        )
        truth = {
            "sim_time_s": float(self.sim_time_s),
            "dist_m": float(self.dist_m),
            "north_m": float(self.north_m),
            "east_m": float(self.east_m),
            "alt_m": float(self.alt_m),
            "true_soc": float(self.soc),
            "true_vp": float(self.vp),
            "true_vterm": float(v_term_true),
            "true_current_a": float(current_true),
            "true_power_w": float(p_true),
            "true_r0_ohm": float(self.true_r0),
            "true_dcd0": float(self.true_dcd0),
            "true_wind_n": float(wn),
            "true_wind_e": float(we),
            "brownout": bool(self.brownout),
            "finished": bool(self.finished),
        }
        return pkt, truth

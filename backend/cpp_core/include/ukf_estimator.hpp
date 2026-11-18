#pragma once
#include "twin_types.hpp"

// Initialize 6-state UKF [SOC, Vp, R0, dCD0, Wn, We] and prior covariance
void ukf_init(
    UkfState& state,
    double soc_init,
    const BatteryParams& batt,
    const WindConfig& wind
);

// Assimilate one noisy telemetry packet via Unscented Kalman Filter
void ukf_step(
    UkfState& state,
    const TelemetryPacket& pkt,
    const VehicleParams& veh,
    const BatteryParams& batt
);

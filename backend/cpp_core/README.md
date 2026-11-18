# C++20 Deterministic Physics, UKF Assimilation & OpenMP Rollout Core

`backend/cpp_core` compiles into a shared library (`libdigital_twin.so`) that executes all numerically intensive physics, real-time Bayesian state estimation, and multi-threaded stochastic mission rollouts for the UAV Digital Twin.

---

## Module Responsibilities

| File | Mathematical & Systems Role |
| :--- | :--- |
| `include/twin_types.hpp` | Flat C-compatible POD structs (`VehicleParams`, `BatteryParams`, `Waypoint3D`, `WindConfig`, `TelemetryPacket`, `UkfState`, `RolloutSummary`) ensuring clean ABI transfer across `ctypes`. |
| `include/battery_ecm.hpp`<br>`src/battery_ecm.cpp` | 1-RC Thevenin electro-thermal equivalent circuit model (Plett 2004; Hogge & Bole NASA 2018): propagates $\text{SOC}(t)$ and polarization voltage $V_p(t)$, computes terminal voltage $V_{\text{term}} = V_{\text{oc}}(\text{SOC}) - I R_0(T,\text{SOH}) - V_p$, applies Traub Peukert C-rate derating, and detects under-voltage cutoff brownout ($V_{\text{term}} \le V_{\text{cutoff}}$). |
| `include/wind_kinematics.hpp`<br>`src/wind_kinematics.cpp` | 3D multi-waypoint flight propagator (climb, cruise, banked loiter orbit $n = 1/\cos\phi$, descent, reciprocal RTB). Computes vector wind groundspeed with crosswind crab-angle penalty $V_g = \sqrt{V_a^2 - W_\perp^2} + W_\parallel$ (Langelaan 2007) and temporally correlated Ornstein-Uhlenbeck wind gusts. |
| `include/ukf_estimator.hpp`<br>`src/ukf_estimator.cpp` | Stack-allocated 6-state Unscented Kalman Filter (`std::array`) estimating $\hat{\mathbf{x}} = [\text{SOC}, V_p, R_0, \Delta C_{D0}, W_N, W_E]^T$ and full $6 \times 6$ covariance $P$ via exact Cholesky sigma-point decomposition. Detects hidden icing drag, battery resistance aging, and wind shifts online from noisy sensor packets. |
| `include/mc_rollout_engine.hpp`<br>`src/mc_rollout_engine.cpp` | OpenMP-parallelized Monte Carlo rollout engine (`#pragma omp parallel for`) sampling initial UKF posterior uncertainty and stochastic OU wind fields across $N = 1,000 - 10,000$ independent trajectories to generate terminal SOC, minimum terminal voltage, brownout flags, and Time-on-Target arrays. |
| `include/c_api_bridge.hpp`<br>`src/c_api_bridge.cpp` | `extern "C"` entrypoints (`twin_step_ukf`, `twin_run_single_mission`, `twin_run_monte_carlo`, `twin_compute_power_curve`) invoked directly by Python `ctypes`. |
| `tests/test_cpp_kernel.cpp` | Standalone C++ verification testbench checking energy conservation, voltage sag cutoff, reciprocal RTB wind symmetry, UKF convergence under fault injection, and OpenMP speedup. |

---

## Build Instructions

```bash
cd Project3_Mission_Level_Digital_Twin/backend/cpp_core
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/test_cpp_kernel
```

The build outputs `build/libdigital_twin.so`, which is loaded automatically by `backend/python_twin/bridge/cpp_loader.py`.

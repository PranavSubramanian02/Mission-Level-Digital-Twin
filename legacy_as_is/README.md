# Project 3: Mission-Level Digital Twin for UAV Operations

## 1. Executive Summary & Problem Statement
Mission-level planning for Unmanned Aerial Vehicles (UAVs) requires comprehensive evaluation of energy availability, flight duration, environmental uncertainties, and return-to-base viability before deployment. Traditional flight planning relies on static point-performance estimates that neglect dynamic wind variability and cumulative energy risks along operational corridors.

This project implements a physics-based, mission-level digital twin framework designed to:
1. Model vehicle aerodynamic power demand across cruise and loiter flight regimes.
2. Propagate battery State-of-Charge (SOC) continuously along waypoint routes as a function of spatial distance, ground speed, and power draw.
3. Quantify mission feasibility by assessing Time-on-Target (ToT) constraints and Return-to-Base (RTB) energy margins.
4. Execute Monte Carlo stochastic wind simulations ($N = 300$ runs) to derive empirical probability density functions (PDFs) of final SOC and quantify probability of mission success ($P_{\text{success}}$).
5. Map operational alternatives onto a 3D Pareto frontier balancing Energy Cost, Mission Time, and Failure Risk.

---

## 2. Theoretical Architecture & Mathematical Models

### 2.1 Vehicle Aerodynamic Drag & Propulsion Demand
The platform is modeled as a $10\,\text{kg}$ fixed-wing aircraft operating at cruise altitude $h = 120\,\text{m}$:
- **Atmospheric Density**:
  $$\rho = \rho_0 \exp\left(-\frac{h}{H_{\text{scale}}}\right) = 1.225 \exp\left(-\frac{120}{8500}\right) \approx 1.2078\,\text{kg/m}^3$$
- **Lift & Drag Polar**:
  $$C_L = \frac{W}{\frac{1}{2} \rho V^2 S}, \quad C_D = C_{D0} + k C_L^2$$
  $$D = \frac{1}{2} \rho V^2 S C_D$$
- **Cruise Power Demand**:
  $$P_{\text{cruise}} = \frac{D \cdot V_{\text{air}}}{\eta_{\text{prop}}}$$
  where $\eta_{\text{prop}} = 0.70$, $S = 0.8\,\text{m}^2$, $C_{D0} = 0.03$, and $k = 0.045$.

### 2.2 Mission Propagation & Battery SOC Depletion
For a flight path discretized into spatial segments of length $\Delta s = 50\,\text{m}$:
- **Effective Ground Speed**:
  $$V_g = \max(V_{\text{air}} + V_w, \; 2.0\,\text{m/s})$$
  $$\Delta t = \frac{\Delta s}{V_g}$$
- **Battery State-of-Charge (SOC)**:
  $$\operatorname{SOC}_{i} = \operatorname{SOC}_{i-1} - \frac{P_{\text{cruise}} \Delta t}{E_{\text{batt},\max}}$$
  where $E_{\text{batt},\max} = 1200\,\text{Wh} \times 3600\,\text{s/hr} = 4.32 \times 10^6\,\text{J}$, with a mandatory reserve floor $\operatorname{SOC}_{\text{reserve}} = 0.25$.
- **Loiter Segment Depletion**:
  $$P_{\text{loiter}} = D(V_{\text{loiter}}) \cdot V_{\text{loiter}}$$
  $$\Delta \operatorname{SOC}_{\text{loiter}} = \frac{P_{\text{loiter}} \cdot t_{\text{loiter}}}{E_{\text{batt},\max}}$$

### 2.3 Monte Carlo Wind Uncertainty & Risk Formulation
Wind disturbances along the route are sampled stochastically:
$$V_w = \left(\mu_{\text{wind}} + \sigma_{\text{wind}} \cdot \mathcal{N}(0, 1)\right) \cos(\theta_{\text{wind}})$$
where $\mu_{\text{wind}} = 4.0\,\text{m/s}$, $\sigma_{\text{wind}} = 1.2\,\text{m/s}$, and $\theta_{\text{wind}} = \pi$ (nominal headwind).
- **Probability of Mission Success**:
  $$P_{\text{success}} = \frac{1}{N_{\text{mc}}} \sum_{j=1}^{N_{\text{mc}}} \mathbb{I}\left(\operatorname{SOC}_{\text{final}}^{(j)} \ge \operatorname{SOC}_{\text{reserve}}\right)$$
  $$\text{Risk} = 1 - P_{\text{success}}$$

---

## 3. Mission Route Specifications

| Route | Distance ($R$) | Nominal Time | Final SOC | Energy Margin | ToT Constraint | RTB Viable |
|:---|:---:|:---:|:---:|:---:|:---:|:---:|
| **Route A** | $30.0\,\text{km}$ | $40.7\,\text{min}$ | $89.76\%$ | $+64.76\%$ | Exceeded ($40.7 > 31.7\,\text{min}$) | Yes |
| **Route B** | $38.0\,\text{km}$ | $50.2\,\text{min}$ | $87.22\%$ | $+62.22\%$ | Exceeded ($50.2 > 31.7\,\text{min}$) | Yes |
| **Route C** | $45.0\,\text{km}$ | $58.6\,\text{min}$ | $85.00\%$ | $+60.00\%$ | Exceeded ($58.6 > 31.7\,\text{min}$) | Yes |

*Note: Required Time-on-Target ($\text{ToT}_{\text{required}}$) is $1900\,\text{s}$ ($31.67\,\text{min}$).

---

## 4. Execution & Usage

```bash
# Execute headless digital twin simulation (saves figures to output_figures/)
python3 project3_as_is.py

# Execute with interactive plot display
python3 project3_as_is.py --show
```

### Generated Outputs
The script generates 11 publication-grade figures in `output_figures/`:
1. `fig0_power_vs_airspeed.png`: Aerodynamic power required curve ($P_{\text{req}}$ vs. $V_{\text{air}}$).
2. `plot1_soc_vs_distance.png`: Spatial battery depletion curve along Route A.
3. `plot2_soc_vs_time.png`: Comparative SOC decay across Routes A, B, and C.
4. `plot3_power_vs_distance.png`: Power consumption profile over mission distance.
5. `plot4_time_on_target.png`: Total transit time compared against the ToT limit.
6. `plot5_energy_margin.png`: Usable energy buffer above the reserve threshold ($25\%$).
7. `plot6_monte_carlo_histogram.png`: Final SOC distribution across 300 stochastic wind realizations.
8. `plot7_soc_pdf.png`: Kernel Density Estimation (KDE) of final battery charge.
9. `plot8_success_probability.png`: Statistical success rate ($100\%$ across tested routes).
10. `plot9_energy_vs_time.png`: Trade-space scatter of transit time vs. energy margin.
11. `plot10_pareto_3d.png`: 3D Pareto front spanning Energy Cost, Time, and Risk.

---

## 5. Critical Engineering Audit: Holes, Faults & Simulation Limitations

> [!WARNING]
> The following modeling flaws exist in the original **as-is** MATLAB implementation (`project3.mlx`) and are faithfully replicated in this baseline script.

1. **Missing Propulsion Efficiency in Loiter Power Calculation**:
   - In cruise (line 96): $P_{\text{cruise}} = D \cdot V / \eta_{\text{prop}}$.
   - In loiter (line 315): $P_{\text{loiter}} = D \cdot V_{\text{loiter}}$ (omits division by $\eta_{\text{prop}} = 0.70$).
   - **Result**: Electrical power required during the 300-second loiter phase is **underestimated by $42.8\%$** ($1 / 0.70 \approx 1.428$).
2. **Directional Wind Inversion Error in Return-to-Base (RTB)**:
   - On the outbound leg, wind is a headwind: $V_w = 4 \cos(\pi) = -4.0\,\text{m/s}$.
   - On the return leg (RTB), the UAV reverses heading by $180^\circ$. The relative wind should become a $+4.0\,\text{m/s}$ tailwind.
   - **Result**: The script passes the exact same $V_w = -4.0\,\text{m/s}$ headwind on both outbound and return legs, penalizing return energy by double-counting headwind resistance.
3. **Discrete Cliff Artifact in Temporal Profiles**:
   - Loiter time ($300\,\text{s}$) and energy are added as a lump sum to the single final element `SOC[-1]` and `time[-1]`.
   - **Result**: The SOC vs. Time plot displays an unphysical instantaneous vertical drop at mission termination rather than a gradual slope over the 5-minute loiter duration.
4. **"Digital Twin" Real-Time Observability Claim**:
   - The original code asserts a "thesis-grade digital twin with full observability." In reality, there is no state observer, sensor measurement noise, telemetry packet ingestion, or dynamic kalman filtering. It functions as a 1D pre-flight reachability script.

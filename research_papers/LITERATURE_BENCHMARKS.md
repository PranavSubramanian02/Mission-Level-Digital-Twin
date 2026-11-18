# Academic Literature & Theoretical Benchmarks for Project 3: Mission-Level Digital Twin for UAV Operations

This document catalogs the peer-reviewed literature foundations and mathematical formulations underpinning the upgraded bidirectional Mission-Level Digital Twin.

---

## 1. Open-Access Papers Downloaded Locally (`research_papers/`)

| Paper / Reference | Venue / Identifier | Local PDF | Core Contribution to Project 3 |
| :--- | :--- | :--- | :--- |
| **Kapteyn, M. G., Pretorius, J. V. R., & Willcox, K. E. (2021)**<br>*"A probabilistic graphical model foundation for enabling predictive digital twins at scale"* | **Nature Computational Science**, Vol. 1, No. 5<br>DOI: [`10.1038/s43588-021-00069-0`](https://doi.org/10.1038/s43588-021-00069-0)<br>arXiv: [`2012.05841`](https://arxiv.org/abs/2012.05841) | `Kapteyn_Willcox_2021_Probabilistic_Digital_Twins_Nature_Comp_Sci.pdf` | Defines the formal Dynamic Bayesian Network coupling physical UAV telemetry to virtual state estimation and optimal mission control. |
| **Tezzele, M., Carr, S., Topcu, U., & Willcox, K. E. (2024)**<br>*"Adaptive planning for risk-aware predictive digital twins"* | **arXiv Preprint**<br>arXiv: [`2407.20490`](https://arxiv.org/abs/2407.20490) | `Tezzele_Willcox_2024_Risk_Aware_Predictive_Digital_Twins.pdf` | Couples UAV Bayesian Digital Twin assimilation with risk-averse online mission replanning under rare/severe events. |
| **Hogge, E. F., Bole, B. M., Vazquez, S. L., Kulkarni, C. S. et al. (NASA, 2018/2020)**<br>*"Verification of Prognostic Algorithms to Predict Remaining Flying Time for Electric Unmanned Vehicles"* | **Int. J. Prognostics & Health Management**, Vol. 9, No. 1<br>DOI: [`10.36001/ijphm.2018.v9i1.2700`](https://doi.org/10.36001/ijphm.2018.v9i1.2700) | `Hogge_Bole_NASA_2018_Prognostic_Remaining_Flying_Time_UAV.pdf` | NASA Langley/Ames flight-tested End-of-Discharge (EoD) voltage prognostics and remaining flying time uncertainty bounds. |
| **Alcibar, J., Aizpurua, J. I., Zugastia, E., & Penagarikano, O. (2024)**<br>*"A Hybrid Probabilistic Battery Health Management Approach for Robust Inspection Drone Operations"* | **arXiv Preprint**<br>arXiv: [`2405.00055`](https://arxiv.org/abs/2405.00055) | `Alcibar_2024_Hybrid_Probabilistic_Drone_Battery_PHM.pdf` | Probabilistic Li-Po EoD voltage prediction separating aleatoric (wind gusts) and epistemic (cell aging) uncertainties. |
| **Zhu, T., Wang, H., Cao, Z., See, S. et al. (2025)**<br>*"Towards Intelligent Systems for Battery Management: A Five-Tier Digital Twin Architecture"* | **arXiv Preprint**<br>arXiv: [`2509.02366`](https://arxiv.org/abs/2509.02366) | `Zhu_2025_Five_Tier_Battery_Digital_Twin_Architecture.pdf` | Five-tier Digital Twin maturity blueprint (Descriptive, Diagnostic, Predictive, Prescriptive, Autonomous) for battery systems. |
| **Majumdar, A. & Pavone, M. (2019)**<br>*"How Should a Robot Assess Risk? Towards an Axiomatic Theory of Risk in Robotics"* | **Springer Proceedings in Advanced Robotics (SPAR)**<br>DOI: [`10.1007/978-3-030-28619-4_10`](https://doi.org/10.1007/978-3-030-28619-4_10)<br>arXiv: [`1710.11040`](https://arxiv.org/abs/1710.11040) | `Majumdar_Pavone_2019_Axiomatic_Theory_Risk_Robotics_CVaR.pdf` | Proves why Conditional Value-at-Risk (CVaR) satisfies coherent risk axioms for autonomous vehicles whereas binary failure probability does not. |

---

## 2. Publisher-Hosted Papers (Verified DOIs for Institutional/Manual Download)

| Paper / Reference | Venue | Verified DOI | Role in Project 3 |
| :--- | :--- | :--- | :--- |
| **Glaessgen, E. & Stargel, D. (2012)**<br>*"The Digital Twin Paradigm for Future NASA and U.S. Air Force Vehicles"* | 53rd AIAA/ASME/ASCE/AHS/ASC SDM Conference | `10.2514/6.2012-1818` | Original NASA/USAF Digital Twin paradigm definition (1,918 citations). |
| **Plett, G. L. (2004)**<br>*"Extended Kalman filtering for battery management systems of LiPB-based HEV battery packs: Part 3. State and parameter estimation"* | **Journal of Power Sources**, Vol. 134, No. 2, pp. 277–292 | `10.1016/j.jpowsour.2004.02.033` | 1-RC Thevenin Equivalent Circuit Model and dual/joint Kalman estimation of SOC and internal resistance $R_0$. |
| **Rockafellar, R. T. & Uryasev, S. (2000)**<br>*"Optimization of conditional value-at-risk"* | **The Journal of Risk**, Vol. 2, No. 3, pp. 21–41 | `10.21314/JOR.2000.038` | Mathematical definition and convex optimization formulation of $\text{VaR}_\alpha$ and $\text{CVaR}_\alpha$ (5,672 citations). |
| **Langelaan, J. W. (2007)**<br>*"Long Distance/Duration Trajectory Optimization for Small UAVs"* | AIAA Guidance, Navigation and Control Conference | `10.2514/6.2007-6737` | 2D/3D wind-vector groundspeed projection, crosswind crab-angle penalty, and airspeed energy optimization. |
| **Bole, B., Kulkarni, C. S., & Daigle, M. (2014)**<br>*"Adaptation of an Electrochemistry-based Li-Ion Battery Model to Account for Deterioration Observed Under Randomized Use"* | Annual Conference of the PHM Society, Vol. 6, No. 1 | `10.36001/phmconf.2014.v6i1.2490` | Electro-thermal battery capacity fade and resistance growth under randomized operational discharge profiles. |
| **Singh, S., Chow, Y., Majumdar, A., & Pavone, M. (2019)**<br>*"A Framework for Time-Consistent, Risk-Sensitive Model Predictive Control: Theory and Algorithms"* | **IEEE Transactions on Automatic Control** | `10.1109/TAC.2018.2874704` | Time-consistent dynamic risk assessment for multi-stage trajectory replanning. |

---

## 3. Core Mathematical Formulations

### 1. 1-RC Thevenin Electro-Thermal Battery Model (Plett 2004; Hogge & Bole NASA 2018)
Terminal voltage under load current $I(t)$ with polarization voltage $V_p(t)$ and SOH/temperature-dependent ohmic resistance $R_0$:
$$\dot{\text{SOC}}(t) = -\frac{\eta_c \, I(t)}{3600 \, Q_{\text{eff}}(\text{SOH})}, \qquad \dot{V}_p(t) = -\frac{V_p(t)}{R_1 C_1} + \frac{I(t)}{C_1}$$
$$V_{\text{term}}(t) = V_{\text{oc}}(\text{SOC}) - I(t) R_0(T, \text{SOH}) - V_p(t)$$
Mission fails via **under-voltage brownout** if $V_{\text{term}}(t) \le V_{\text{cutoff}}$ ($3.1\,\text{V/cell}$) OR if terminal $\text{SOC} < \text{SOC}_{\text{reserve}}$.

### 2. 3D Vector Wind Kinematics & Banked Loiter Drag (Langelaan 2007)
For segment unit track vector $\hat{\mathbf{u}}$ and horizontal wind $\mathbf{W} = [W_N, W_E]^T$:
$$W_{\parallel} = \mathbf{W} \cdot \hat{\mathbf{u}}, \qquad W_{\perp} = \|\mathbf{W} - W_{\parallel}\hat{\mathbf{u}}\|$$
Groundspeed with crosswind crab angle $\beta_{\text{crab}} = \arcsin(W_{\perp} / V_a)$:
$$V_g = \sqrt{V_a^2 - W_{\perp}^2} + W_{\parallel}$$
Notice that on Return-to-Base ($\hat{\mathbf{u}}_{\text{RTB}} = -\hat{\mathbf{u}}_{\text{out}}$), $W_{\parallel,\text{RTB}} = -W_{\parallel,\text{out}}$ automatically corrects the legacy opposite-wind sign bug. During banked loiter at bank angle $\phi$ ($n = 1/\cos\phi$):
$$P_{\text{elec,loiter}} = \frac{\frac{1}{2}\rho V_{\text{loit}}^3 S \left(C_{D0} + k \left(\frac{n W}{\frac{1}{2}\rho V_{\text{loit}}^2 S}\right)^2\right)}{\eta_{\text{prop}}} + P_{\text{hotel}}$$

### 3. Conditional Value-at-Risk ($\text{CVaR}_\alpha$) (Rockafellar & Uryasev 2000; Majumdar & Pavone 2019)
Let loss $L = \text{SOC}_{\text{reserve}} - \text{SOC}_{\text{final}}$ (or cutoff voltage deficit) over $N$ stochastic wind + SOH rollouts. At confidence level $\alpha = 0.95$:
$$\text{VaR}_{0.95}(L) = \inf \{ \zeta \in \mathbb{R} : P(L \le \zeta) \ge 0.95 \}$$
$$\text{CVaR}_{0.95}(L) = \mathbb{E}\left[ L \mid L \ge \text{VaR}_{0.95}(L) \right]$$
CVaR quantifies the expected severity of the worst $5\%$ mission outcomes, producing a smooth, strictly informative risk axis for 3D Energy–Time–Risk Pareto optimization.

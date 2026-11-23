# Unified Theory of Relative Guidance: Missile Homing & Orbital Rendezvous

This document establishes the mathematical and control-theoretic foundations of the unified `lvguidance` framework. Both missile homing intercept and spacecraft orbital rendezvous are formulated under a common relative-motion state space, with application-specific constraints and dynamics applied in their respective domain modules.

---

## 1. Problem Formulation and 3D Relative Kinematics

Let $\mathbf{r}_T, \mathbf{v}_T$ be the target state and $\mathbf{r}_C, \mathbf{v}_C$ be the chaser (or missile) state in an inertial Cartesian reference frame.

### 1.1 Relative State
$$\mathbf{x} = \mathbf{r}_T - \mathbf{r}_C, \qquad \dot{\mathbf{x}} = \mathbf{v}_T - \mathbf{v}_C$$

- **Range ($R$):**
  $$R = \|\mathbf{x}\| = \sqrt{\mathbf{x}\cdot\mathbf{x}}$$
- **Line-of-Sight (LOS) Unit Vector ($\hat{\mathbf{l}}$):**
  $$\hat{\mathbf{l}} = \frac{\mathbf{x}}{R}$$
- **Range Rate ($\dot{R}$):**
  $$\dot{R} = \frac{d}{dt}\sqrt{\mathbf{x}\cdot\mathbf{x}} = \hat{\mathbf{l}}\cdot\dot{\mathbf{x}}$$
- **Closing Speed ($V_c$):**
  $$V_c = -\dot{R} = -\hat{\mathbf{l}}\cdot\dot{\mathbf{x}}$$
- **Inertial Line-of-Sight Angular Velocity Vector ($\boldsymbol{\Omega}$):**
  $$\boldsymbol{\Omega} = \frac{\mathbf{x} \times \dot{\mathbf{x}}}{R^2}$$
- **LOS Unit Vector Derivative:**
  $$\dot{\hat{\mathbf{l}}} = \boldsymbol{\Omega} \times \hat{\mathbf{l}}$$
- **Seeker Look Angle ($\eta$):**
  $$\cos\eta = \frac{\mathbf{v}_C \cdot \hat{\mathbf{l}}}{\|\mathbf{v}_C\|}$$

### 1.2 Relative Equations of Motion
$$\ddot{\mathbf{x}} = \ddot{\mathbf{r}}_T - \ddot{\mathbf{r}}_C = \mathbf{a}_T - \mathbf{a}_C + \Delta\mathbf{g}(\mathbf{r}_T, \mathbf{r}_C)$$

where $\Delta\mathbf{g} = \mathbf{g}(\mathbf{r}_T) - \mathbf{g}(\mathbf{r}_C)$ is the differential gravitational acceleration.

**Control Convention:** Every guidance law calculates a free-space demand $\mathbf{u}$ intended to yield:
$$\ddot{\mathbf{x}} = -\mathbf{u} + \mathbf{a}_T + \Delta\mathbf{g}$$
where positive $\mathbf{u}$ along $\hat{\mathbf{l}}$ accelerates the chaser/missile toward the target.

---

## 2. Generalized Optimal Guidance (Bryson-Ho Framework)

Both missile intercept and orbital rendezvous can be derived as special cases of linear-quadratic optimal control minimizing control effort:
$$\min_{\mathbf{u}} J = \frac{1}{2} \int_t^{t_f} \|\mathbf{u}(s)\|^2 \, ds \quad \text{s.t.} \quad \ddot{\mathbf{x}} = -\mathbf{u}$$

Using Pontryagin's Minimum Principle, the optimal acceleration profile is **linear in time-to-go** $\tau = t_f - t$:
$$\mathbf{u}(\tau) = \mathbf{c}_0 + \mathbf{c}_1 \tau$$

Evaluating the solution at the current time yields the master closed-loop guidance law:
$$\boxed{\mathbf{u} = \frac{K_r}{\tau^2}\mathbf{ZEM} - \frac{K_v}{\tau}\mathbf{ZEV} + K_a \mathbf{a}_T^{\perp}}$$

where:
- $\mathbf{ZEM} = \mathbf{x} + \dot{\mathbf{x}}\tau$ is the **Zero-Effort Miss** (projected miss distance if control ceases).
- $\mathbf{ZEV} = \dot{\mathbf{x}} - \mathbf{v}_f$ is the **Zero-Effort Velocity error** relative to terminal velocity $\mathbf{v}_f$.
- $\mathbf{a}_T^{\perp} = \mathbf{a}_T - (\mathbf{a}_T \cdot \hat{\mathbf{l}})\hat{\mathbf{l}}$ is target maneuver acceleration normal to LOS.

### Gain Configuration Mapping:

| Objective | $K_r$ | $K_v$ | $K_a$ | Equivalent Guidance Law |
|---|:---:|:---:|:---:|---|
| **Missile Intercept (Free $v_f$)** | $3$ | $0$ | $0$ | **Optimal Homing Guidance (OGL) / True PN ($N=3$)** |
| **Missile Maneuver Compensation** | $3$ | $0$ | $1.5$ | **Augmented Proportional Navigation (APN)** |
| **Spacecraft Soft Docking ($v_f=0$)** | $6$ | $2$ | $0$ | **ZEM/ZEV Fixed-Time Guidance** |
| **Controlled Docking Speed ($v_f=-v_d\hat{\mathbf{l}}$)** | $6$ | $2$ | $0$ | **Terminal-Velocity Optimal Guidance** |

---

## 3. Proportional Navigation (PN) Family

Proportional navigation nulls the inertial LOS rate $\boldsymbol{\Omega} \to \mathbf{0}$, leading directly to a constant-bearing collision course.

### 3.1 True Proportional Navigation (TPN)
Acceleration is commanded perpendicular to the instantaneous line-of-sight:
$$\mathbf{u}_{\text{TPN}} = N V_c (\boldsymbol{\Omega} \times \hat{\mathbf{l}})$$
where $N \in [3, 5]$ is the effective navigation ratio.

### 3.2 Pure Proportional Navigation (PPN)
Acceleration is commanded perpendicular to the missile velocity vector $\hat{\mathbf{v}}_C$:
$$\mathbf{u}_{\text{PPN}} = N V_c (\boldsymbol{\Omega} \times \hat{\mathbf{v}}_C)$$

### 3.3 Augmented Proportional Navigation (APN)
Augments TPN with a feed-forward compensation term for maneuvering targets:
$$\mathbf{u}_{\text{APN}} = N V_c (\boldsymbol{\Omega} \times \hat{\mathbf{l}}) + \frac{N}{2} \mathbf{a}_T^{\perp}$$
This cancels target evasive accelerations (e.g. constant g turns or sinusoidal weaving), drastically reducing terminal miss distance and required control effort.

### 3.4 Biased Proportional Navigation (Impact Angle Control Guidance / IACG)
Constrains the terminal trajectory to achieve a prescribed impact direction $\hat{\mathbf{v}}_{\text{des}}$:
$$\mathbf{u}_{\text{BPN}} = N V_c (\boldsymbol{\Omega} \times \hat{\mathbf{l}}) + \frac{K_{\text{bias}} V_c}{\tau} (\hat{\mathbf{v}}_{\text{des}} - \hat{\mathbf{v}}_C)^{\perp}$$

---

## 4. Sliding-Mode Guidance (SMG)

Sliding-mode control provides robustness against model uncertainties, target maneuver variations, and unmodeled disturbances.

### 4.1 Speed-Limited Braking Surface (Orbital Rendezvous)
$$v_c(R) = \min\left(\sqrt{2 a_{\text{brake}} R}, \; v_{\max}\right)$$
$$\mathbf{s} = \dot{\mathbf{x}} + v_c(R) \hat{\mathbf{l}}$$
$$\mathbf{u} = k \cdot \text{sat}\left(\frac{\mathbf{s}}{\phi}\right)$$
The vehicle cruises at $v_{\max}$ and automatically switches to a constant deceleration $a_{\text{brake}}$ braking profile so that range and velocity vanish simultaneously.

### 4.2 LOS Angular Rate Nulling Surface (Missile Homing)
$$\mathbf{s} = \boldsymbol{\Omega} \times \hat{\mathbf{l}}$$
$$\mathbf{u} = N V_c \mathbf{s} + k \cdot \text{sat}\left(\frac{\mathbf{s}}{\phi}\right)$$
Drives the LOS angular rate vector to zero with boundary-layer thickness $\phi$ to eliminate actuator chattering.

---

## 5. Glideslope & Phased Guidance

### 5.1 Glideslope Final-Approach Guidance
Flies a corridor aligned with the approach axis $\hat{\mathbf{a}} = \hat{\mathbf{l}}_{\text{handover}}$:
$$\ell = \mathbf{x} \cdot \hat{\mathbf{a}}, \qquad \mathbf{x}_{\perp} = \mathbf{x} - \ell \hat{\mathbf{a}}$$
$$v_c(\ell) = \text{clip}(k_g \ell, v_{\text{dock}}, v_{\max})$$
$$\dot{\mathbf{x}}_{\text{des}} = -v_c(\ell) \hat{\mathbf{a}} - k_{\text{lat}} \mathbf{x}_{\perp}$$
$$\mathbf{u} = k_v (\dot{\mathbf{x}} - \dot{\mathbf{x}}_{\text{des}})$$

### 5.2 Phased Guidance Handover
Switches one-way from midcourse guidance (e.g., OGL or Adaptive ZEM/ZEV) to terminal guidance (Glideslope or TPN) when range drops below $R_{\text{switch}}$ (e.g. 1000 m).

---

## 6. Subsystem Realism Models

### 6.1 Autopilot Dynamics
First-order response lag with acceleration and rate saturation:
$$\tau_a \dot{\mathbf{a}}_c + \mathbf{a}_c = \mathbf{u}_{\text{cmd}}, \quad \|\dot{\mathbf{a}}_c\| \le \dot{a}_{\max}, \quad \|\mathbf{a}_c\| \le a_{\max}$$

### 6.2 Seeker Sensor & Alpha-Beta Tracking Filter
- Field of view check: $\eta \le \eta_{\max}$.
- LOS measurement with additive Gaussian noise $\tilde{\hat{\mathbf{l}}} = \text{normalize}(\hat{\mathbf{l}} + \mathcal{N}(0, \sigma^2))$.
- Discrete-time $\alpha$-$\beta$ filter estimates smooth LOS vector and angular rate $\hat{\boldsymbol{\Omega}}$.

---

## 7. Domain Applications

### 7.1 Missile Homing Intercept (`lvguidance.applications.missile_homing`)
- **Objectives:** Minimum terminal miss distance (CPA), high-g maneuverability.
- **Target Maneuver Library:**
  - `NonManeuveringTarget`: Constant velocity.
  - `ConstantTurnTarget`: Constant lateral $n$-g acceleration.
  - `WeavingTarget`: Sinusoidal weaving $\mathbf{a}_T(t) = A \sin(\omega t + \phi) \hat{\mathbf{n}}$.
  - `BallisticTarget`: Re-entry gravity + aerodynamic drag deceleration.
- **Evaluation:** Closest Point of Approach (CPA), Circular Error Probable (CEP), control effort.

### 7.2 Orbital Rendezvous (`lvguidance.applications.orbital_rendezvous`)
- **Objectives:** Soft docking ($R \le 2\text{ m}$, $v_{\text{rel}} \le 0.3\text{ m/s}$) with minimum fuel expenditure.
- **Environment:** Central two-body Earth gravity, LVLH rotating frame, exact differential gravity feedforward.
- **Evaluation:** Capture success, contact velocity, $\Delta V$ propellant use.

---

## 8. Astrodynamics & Orbital Relative Motion

### 8.1 Clohessy-Wiltshire (Hill-Clohessy-Wiltshire) Equations
For a circular reference target orbit with radius $r_0$ and mean motion $n = \sqrt{\mu / r_0^3}$, the relative motion in the target-centered Local-Vertical / Local-Horizontal (LVLH) frame is linearized as:
$$\begin{aligned}
\ddot{x} - 2n\dot{y} - 3n^2 x &= u_x \\
\ddot{y} + 2n\dot{x} &= u_y \\
\ddot{z} + n^2 z &= u_z
\end{aligned}$$
where:
- $x$ is radial (outward from Earth center),
- $y$ is along-track (direction of target orbital velocity),
- $z$ is cross-track (normal to orbital plane).

*(For the complete, first-principles derivation starting from inertial Newtonian mechanics and the rotating frame Transport Theorem, see [docs/clohessy_wiltshire_derivation.md](file:///e:/2026-11/launch_vehicle_guidance_law/docs/clohessy_wiltshire_derivation.md).)*

The state-space representation is $\dot{\mathbf{x}} = A \mathbf{x} + B \mathbf{u}$. The analytical State Transition Matrix (STM) $\Phi(t)$ provides closed-form mapping:
$$\mathbf{x}(t) = \Phi(t) \mathbf{x}(0) + \int_0^t \Phi(t - \tau) B \mathbf{u}(\tau) d\tau$$

### 8.2 Two-Impulse Clohessy-Wiltshire Transfer
Partitioning the CW STM into $3 \times 3$ blocks:
$$\Phi(t) = \begin{bmatrix} \Phi_{rr}(t) & \Phi_{rv}(t) \\ \Phi_{vr}(t) & \Phi_{vv}(t) \end{bmatrix}$$
To transfer from initial position $\mathbf{r}_0$ to final target position $\mathbf{r}_f$ in transfer time $T_{tof}$:
$$\mathbf{r}_f = \Phi_{rr}(T_{tof})\mathbf{r}_0 + \Phi_{rv}(T_{tof})\mathbf{v}_0^+$$
When $\Phi_{rv}(T_{tof})$ is non-singular ($nT_{tof} \ne 2k\pi$), the required initial velocity is:
$$\mathbf{v}_0^+ = \Phi_{rv}^{-1}(T_{tof}) \left[ \mathbf{r}_f - \Phi_{rr}(T_{tof})\mathbf{r}_0 \right]$$
The two impulsive velocity increments are:
$$\Delta \mathbf{v}_1 = \mathbf{v}_0^+ - \mathbf{v}_0^-, \qquad \Delta \mathbf{v}_2 = \mathbf{v}_f - \left[ \Phi_{vr}(T_{tof})\mathbf{r}_0 + \Phi_{vv}(T_{tof})\mathbf{v}_0^+ \right]$$

### 8.3 Universal Variable Kepler Propagation & Lambert Problem
For arbitrary eccentricities and long-duration transfers where linearization degrades:
1. **Stumpff Functions:**
   $$C(z) = \frac{1 - \cos\sqrt{z}}{z}, \qquad S(z) = \frac{\sqrt{z} - \sin\sqrt{z}}{\sqrt{z}^3}$$
2. **Universal Kepler Equation:** Solved via Newton-Raphson on universal anomaly $\chi$.
3. **Lambert Problem:** Given $\mathbf{r}_1, \mathbf{r}_2$ and transfer time $\Delta t$, computes the exact Keplerian orbit connecting the two points under full two-body gravity.

### 8.4 State-Feedback Optimal Control: LQR and MPC
1. **Linear Quadratic Regulator (LQR):** Discretizes the CW model via Zero-Order Hold (ZOH) $A_d = e^{A \Delta t}, B_d = \int_0^{\Delta t} e^{A s} B ds$. Bryson's rule balances state penalties against thrust capability:
   $$Q = \text{diag}(1/p_{\max}^2, 1/v_{\max}^2), \qquad R = \text{diag}(1/a_{\max}^2)$$
   The optimal state feedback gain $K = (R + B_d^T P B_d)^{-1} B_d^T P A_d$ is obtained by solving the Discrete Algebraic Riccati Equation (DARE).
2. **Model Predictive Control (MPC):** Over horizon $H_p$, solves a box-constrained convex quadratic program:
   $$\min_{\mathbf{u}_{0:H_p-1}} \sum_{k=0}^{H_p} \mathbf{x}_k^T Q \mathbf{x}_k + \sum_{k=0}^{H_p-1} \mathbf{u}_k^T R \mathbf{u}_k \quad \text{s.t.} \quad |u_{k,i}| \le a_{\max}$$
   Solved in milliseconds via bounded least squares (`scipy.optimize.lsq_linear`), providing online actuator saturation handling.

# Theory of the Homing Guidance Laws

This note derives every law in `lvguidance.guidance` and ties each formula to the code. All laws
are *homing* laws: they use only quantities measured relative to the target (range, line of
sight, relative velocity), not the absolute trajectory of either body.

Contents

1. [Problem setup and notation](#1-problem-setup-and-notation)
2. [Gravity feed-forward](#2-gravity-feed-forward)
3. [Velocity pursuit](#3-velocity-pursuit)
4. [PD feedback](#4-pd-feedback)
5. [Energy-optimal ZEM/ZEV guidance](#5-energy-optimal-zemzev-guidance)
6. [Adaptive time-to-go ZEM/ZEV](#6-adaptive-time-to-go-zemzev)
7. [Terminal-velocity optimal guidance](#7-terminal-velocity-optimal-guidance)
8. [Sliding-mode guidance](#8-sliding-mode-guidance)
9. [Terminal approach (glideslope) guidance](#9-terminal-approach-glideslope-guidance)
10. [Phased guidance: midcourse to terminal](#10-phased-guidance-midcourse-to-terminal)
11. [Comparison and how to choose](#11-comparison-and-how-to-choose)
12. [Metrics](#12-metrics)
13. [References](#13-references)

---

## 1. Problem setup and notation

A *chaser* (the launch-vehicle upper stage) must reach a *target* on a circular orbit and arrive
with a small relative velocity. Both bodies move under two-body gravity; only the chaser thrusts.

**Relative state** (inertial components, target minus chaser):

$$
\mathbf{x} = \mathbf{r}_t - \mathbf{r}_c, \qquad \dot{\mathbf{x}} = \mathbf{v}_t - \mathbf{v}_c
$$

Derived quantities used by the laws (`GuidanceState` in `guidance/base.py`):

| Symbol | Meaning | Code |
|---|---|---|
| $R = \lVert\mathbf{x}\rVert$ | range | `state.range` |
| $\hat{\mathbf{l}} = \mathbf{x}/R$ | line-of-sight (LOS) unit vector, chaser to target | `state.los` |
| $\dot R = \hat{\mathbf{l}}\cdot\dot{\mathbf{x}}$ | range rate (negative when closing) | `state.range_rate` |
| $t_{go} = T - t$ | time to the planned arrival $T$ | `state.t_go` |

**Equation of motion.** With the chaser's thrust acceleration $\mathbf{a}_c$,

$$
\ddot{\mathbf{x}} = \mathbf{g}(\mathbf{r}_t) - \mathbf{g}(\mathbf{r}_c) - \mathbf{a}_c
$$

**Control convention.** Every law returns a *free-space command* $\mathbf{u}$ meant to give

$$
\boxed{\ddot{\mathbf{x}} = -\mathbf{u}}
$$

i.e. a pure double integrator. A positive $\mathbf{u}$ along $\hat{\mathbf{l}}$ accelerates the chaser
toward the target. The engine converts $\mathbf{u}$ to the thrust command (section 2) and
saturates it at $a_{max}$ (`Scenario.max_accel`, 0.5 m/s² by default).

**Goal.** Drive $\mathbf{x}\to\mathbf{0}$ with $\dot{\mathbf{x}}$ small (rendezvous), inside the
capture radius `Scenario.capture_radius` (2 m).

> Proportional navigation is an *intercept* law, which aims at zero miss distance without
> constraining the final relative velocity. It is outside the scope of this rendezvous package,
> where the terminal velocity is a requirement.

---

## 2. Gravity feed-forward

The two bodies feel slightly different gravity, so $\mathbf{g}(\mathbf{r}_t)-\mathbf{g}(\mathbf{r}_c)\neq 0$.
For a small separation this is the gravity gradient:

$$
\Delta\mathbf{g} \approx \frac{\mu}{r^3}\left(3\,\hat{\mathbf{r}}\hat{\mathbf{r}}^{T} - I\right)\mathbf{x}
$$

where $r$ is the orbit radius and $\hat{\mathbf{r}}$ the local vertical. Without compensation the
relative motion obeys the Clohessy-Wiltshire equations, with orbital rate $n^2=\mu/r^3$.

The engine applies the *exact* difference (not the linearization) as feed-forward:

$$
\mathbf{a}_c = \mathbf{u} + \underbrace{\mathbf{g}(\mathbf{r}_t)-\mathbf{g}(\mathbf{r}_c)}_{\Delta\mathbf{g}}
\;\;\Rightarrow\;\;
\ddot{\mathbf{x}} = \Delta\mathbf{g} - \mathbf{u} - \Delta\mathbf{g} = -\mathbf{u}
$$

This is why all laws can be designed for a double integrator, and why the relative motion in an
inertial frame is a straight line for the optimal laws. Set `Scenario.gravity_feedforward=False`
to see how each law copes with the raw orbital dynamics. This is a useful exercise, because the
laws were tuned with the compensation on and the results will change.

---

## 3. Velocity pursuit

**Class:** `VelocityPursuit` in `guidance/basic.py`. **Idea:** keep the relative velocity pointed
along the LOS with a speed scheduled on range.

$$
v_c = \min(k_R R,\; v_{max}), \qquad
\mathbf{u} = k_v\left(\dot{\mathbf{x}} + v_c\,\hat{\mathbf{l}}\right)
$$

The desired relative velocity is $\dot{\mathbf{x}}_{des} = -v_c\hat{\mathbf{l}}$ (closing along the LOS).
The command is a first-order velocity loop of time constant $1/k_v$ (5 s with the default
$k_v = 0.15\ \mathrm{s^{-1}}$).

**Behaviour.**

- Far away ($k_R R > v_{max}$): $\dot R = -v_{max}$, a constant-speed cruise.
- Near the target: $\dot R = -k_R R$, i.e. an exponential decay with time constant $1/k_R$.

**Limitation (visible in the results).** At the end of the cruise the schedule demands
a deceleration of

$$
\frac{d v_c}{dt} = k_R\,v_{max} = 0.03\times 80 = 2.4\ \mathrm{m/s^2}
$$

which is almost five times the thrust limit. The loop saturates, cannot brake in time, overshoots,
and has to turn around. The default scenario shows two passes within about 14 m before capture.
A schedule is only consistent with the vehicle if $k_R v_{max}\le a_{max}$, which is exactly what
the braking profile of the sliding-mode law (section 8) enforces.

---

## 4. PD feedback

**Class:** `ProportionalDerivative` in `guidance/basic.py`. **Idea:** treat the chaser as a mass
on a spring-damper anchored at the target.

$$
\mathbf{u} = \omega_n^2\,\mathbf{x} + 2\zeta\omega_n\,\dot{\mathbf{x}}
\quad\Rightarrow\quad
\ddot{\mathbf{x}} + 2\zeta\omega_n\dot{\mathbf{x}} + \omega_n^2\mathbf{x} = \mathbf{0}
$$

For critical damping ($\zeta = 1$) the solution is

$$
\mathbf{x}(t) = \left[\mathbf{x}_0 + (\dot{\mathbf{x}}_0 + \omega_n\mathbf{x}_0)\,t\right]e^{-\omega_n t}
$$

There is no overshoot and the settling time scales as $4/\omega_n$.

**Trade-off.** A high $\omega_n$ arrives quickly but commands $\omega_n^2 R_0$ at the start. With
$\omega_n = 0.012\ \mathrm{rad/s}$ and $R_0 = 26\ \mathrm{km}$ that is $3.7\ \mathrm{m/s^2}$, far above
the limit. The saturated phase is then no longer the linear system above, and the vehicle
overshoots its ideal path and pays extra Δv (188 m/s versus 78 m/s for the optimal laws). A low
$\omega_n$ respects the limit but takes too long. PD has no notion of arrival time or fuel; it
is the baseline that the optimal laws improve on.

---

## 5. Energy-optimal ZEM/ZEV guidance

**Class:** `ZemZevGuidance` in `guidance/optimal.py`.

**Problem.** Minimise the control energy to arrive at the planned time $T$ with zero relative
position and velocity:

$$
\min\; J = \tfrac12\int_t^T \lVert\mathbf{u}\rVert^2\,ds
\quad\text{s.t.}\quad \ddot{\mathbf{x}}=-\mathbf{u},\;\; \mathbf{x}(T)=\mathbf{0},\;\; \dot{\mathbf{x}}(T)=\mathbf{0}
$$

**Solution.** The Hamiltonian is $H=\tfrac12\lVert\mathbf{u}\rVert^2+\boldsymbol\lambda_x\cdot\dot{\mathbf{x}}-\boldsymbol\lambda_v\cdot\mathbf{u}$.
Stationarity gives $\mathbf{u}=\boldsymbol\lambda_v$, and the costate equations
$\dot{\boldsymbol\lambda}_x=\mathbf{0}$, $\dot{\boldsymbol\lambda}_v=-\boldsymbol\lambda_x$ make $\mathbf{u}$
**linear in time**. Write it in terms of time-to-go $\tau = T-t$:

$$
\mathbf{u}(\tau) = \mathbf{c}_0 + \mathbf{c}_1\tau
$$

Integrating the dynamics from the current time (time-to-go $\tau$) to the end gives the two terminal conditions

$$
\dot{\mathbf{x}} = \mathbf{c}_0\tau + \tfrac12\mathbf{c}_1\tau^2, \qquad
\mathbf{x} + \dot{\mathbf{x}}\tau = \tfrac12\mathbf{c}_0\tau^2 + \tfrac13\mathbf{c}_1\tau^3
$$

Solving for the constants,

$$
\mathbf{c}_1 = \frac{12\,(\mathbf{x} + \tfrac12\dot{\mathbf{x}}\tau)}{\tau^3}, \qquad
\mathbf{c}_0 = -\frac{6\mathbf{x}}{\tau^2} - \frac{2\dot{\mathbf{x}}}{\tau}
$$

and evaluating $\mathbf{u}$ at the present instant gives the **feedback law**

$$
\boxed{\mathbf{u} = \frac{6\mathbf{x}}{\tau^2} + \frac{4\dot{\mathbf{x}}}{\tau}
= \frac{6}{\tau^2}\,\mathrm{ZEM} - \frac{2}{\tau}\,\mathrm{ZEV}}
$$

with the two *zero-effort* quantities

$$
\mathrm{ZEM} = \mathbf{x} + \dot{\mathbf{x}}\,\tau \;\;(\text{position error at } T \text{ if you coast}), \qquad
\mathrm{ZEV} = \dot{\mathbf{x}} \;\;(\text{velocity error at } T \text{ if you coast})
$$

(`_energy_optimal` in `optimal.py`.)

**Properties.**

- Re-solving at every step makes it a closed-loop law: disturbances are corrected automatically.
- It is a **time-varying PD controller**: $\omega_n^2 = 6/\tau^2$ and $2\zeta\omega_n = 4/\tau$, so
  the gains stiffen as $\tau\to 0$.
- With gravity compensated, the optimal relative path is a straight line in the inertial frame,
  and $\lVert\mathbf{u}\rVert$ decreases linearly to zero at the midpoint of the manoeuvre
  and then rises again as seen in the component plots when the initial relative velocity is zero.
- **Singularity.** The gains blow up as $\tau\to 0$. The code floors the time-to-go
  at `t_go_min` = 5 s. Near arrival the vehicle is already within metres, so the floor
  costs nothing but removes the numerical spike.
- It needs the planned arrival time $T$ (`Scenario.t_final`). Choosing $T$ trades arrival time
  against $\Delta v$ and peak acceleration (peak $\propto 1/T^2$).

---

## 6. Adaptive time-to-go ZEM/ZEV

**Class:** `AdaptiveTgoGuidance`. **Idea:** the law of section 5, but without a planned arrival
time, so $\tau$ is estimated online from range and closing speed.

Two physically motivated estimates are combined.

1. If the vehicle is closing and decelerates uniformly, it stops after
   $\tau_{stop} = 2R/(-\dot R)$ (the average speed is half the current speed).
2. A nominal accelerate-then-brake manoeuvre with acceleration $a_{nom}$ covers $R$ in
   $\tau_{cap} = 2\sqrt{R/a_{nom}}$ (each half covers $R/2 = \tfrac12 a_{nom}t^2$).

$$
\tau = \max\!\left(\min(\tau_{stop},\,\tau_{cap}),\;\tau_{min}\right)
$$

When the vehicle is not yet closing, $\tau_{stop}$ is huge and $\tau_{cap}$ governs. As the
closing speed grows, $\tau_{stop}$ shrinks and takes over, so the law brakes at the right
moment. The cap matters most at the start: without it, a vehicle at rest would see an enormous
$\tau$ and command almost no thrust.

**Trade-off.** No arrival time is needed, but the arrival time is an *outcome* (956 s in the default
run) and the time-to-go is only an estimate, so the Δv is slightly above the fixed-time optimum.

---

## 7. Terminal-velocity optimal guidance

**Class:** `TerminalVelocityGuidance`. **Idea:** arrive at $T$ with a *prescribed non-zero*
relative velocity $\mathbf{v}_f$ (a controlled docking speed), still minimising energy.

Define the shifted variable $\mathbf{y} = \mathbf{x} + \mathbf{v}_f\tau$. Then
$\dot{\mathbf{y}} = \dot{\mathbf{x}} - \mathbf{v}_f$ and $\ddot{\mathbf{y}} = \ddot{\mathbf{x}}$, and the
terminal conditions become $\mathbf{y}(T)=\mathbf{0}$, $\dot{\mathbf{y}}(T)=\mathbf{0}$, which is the
problem of section 5 in $\mathbf{y}$. Substituting back,

$$
\boxed{\mathbf{u} = \frac{6}{\tau^2}\left(\mathbf{x}+\dot{\mathbf{x}}\tau\right) - \frac{2}{\tau}\left(\dot{\mathbf{x}} - \mathbf{v}_f\right)}
$$

That is, the ZEM is unchanged and the ZEV is measured *against* $\mathbf{v}_f$. In the code

$$
\mathbf{v}_f = -v_{dock}\,\hat{\mathbf{l}}_0
$$

where $\hat{\mathbf{l}}_0$ is the LOS at the first call, so the vehicle closes at $v_{dock}$ (0.5 m/s
by default) along the initial approach line. Final relative speeds were 0.56 m/s against
0.76 m/s for the zero-velocity version, a small difference because both end at a low speed;
the point of the law is that the value is *commanded* and not incidental.

---

## 8. Sliding-mode guidance

**Class:** `SlidingModeGuidance` in `guidance/sliding_mode.py`. **Idea:** define a surface in
velocity space that the relative velocity must follow, and force the state onto it with a
high-gain switching law. This is robust to model error (e.g. imperfect gravity compensation).

**Surface.** The closing-speed schedule is a braking profile, the speed from which constant
deceleration $a_b$ brings the vehicle to rest exactly at the target ($v^2 = 2a_bR$), capped at
$v_{max}$:

$$
v_c(R) = \min\!\left(\sqrt{2a_bR},\; v_{max}\right), \qquad
\mathbf{s} = \dot{\mathbf{x}} + v_c(R)\,\hat{\mathbf{l}}
$$

On $\mathbf{s}=\mathbf{0}$ the vehicle cruises at $v_{max}$ and then brakes at the constant
$a_b$ (0.2 m/s²), so range and speed reach zero together, in finite time. A linear
surface ($v_c = \lambda R$) would give an exponentially slow final approach and, as with
pursuit, demand more braking than the thrust limit allows.

**Control law.**

$$
\mathbf{u} = k\,\mathrm{sat}\!\left(\frac{\mathbf{s}}{\phi}\right), \qquad
\mathrm{sat}(\mathbf{z}) = \begin{cases}\mathbf{z} & \lVert\mathbf{z}\rVert<1\\[2pt] \mathbf{z}/\lVert\mathbf{z}\rVert & \text{otherwise}\end{cases}
$$

**Reaching condition.** For $V=\tfrac12\lVert\mathbf{s}\rVert^2$,
$\dot V = \mathbf{s}\cdot\dot{\mathbf{s}}$. Outside the boundary layer
$\dot{\mathbf{s}} = -\mathbf{u} + \dot{(v_c\hat{\mathbf{l}})}$, and the surface itself moves at
about $a_b$ (plus a term $v_c\dot{\hat{\mathbf{l}}}$ from LOS rotation). Therefore

$$
\dot V \le -\lVert\mathbf{s}\rVert\,(k - a_b - \text{LOS-rotation term})<0
\quad\Rightarrow\quad k > a_b + \dots
$$

The defaults $k=0.4$, $a_b=0.2$ satisfy this with margin, and $k$ must also stay below
$a_{max}$ to be realisable.

**Boundary layer.** A pure sign function would chatter. Inside $\lVert\mathbf{s}\rVert<\phi$ the law is
a linear gain $k/\phi$, which smooths the thrust. The price is a steady tracking
error: since the command must supply roughly $a_b$ to follow the braking surface, the
state settles at $\lVert\mathbf{s}\rVert\approx\phi\,a_b/k = 1\ \mathrm{m/s}$ from the surface. This
is the reason sliding mode arrived fastest (600 s) but with the highest final relative speed of the
well-behaved laws (2.65 m/s), which motivates the terminal law of the next section. Feed-forward
of the surface acceleration would remove most of the offset.

---

## 9. Terminal approach (glideslope) guidance

**Class:** `TerminalApproachGuidance` in `guidance/terminal.py`. **Idea:** for the last stretch,
fly a straight corridor with a closing speed that falls with distance down to a docking speed,
and actively hold the vehicle on the corridor axis.

**Geometry.** The approach axis $\hat{\mathbf{a}}$ is the LOS when the law first runs
(the handover). Split the relative position:

$$
\ell = \mathbf{x}\cdot\hat{\mathbf{a}}, \qquad \mathbf{x}_\perp = \mathbf{x} - \ell\,\hat{\mathbf{a}}
$$

($\ell$ is the longitudinal distance to go and $\mathbf{x}_\perp$ the lateral error.)

**Desired velocity and command.**

$$
v_c(\ell) = \mathrm{clip}\!\left(k_g\,\ell,\; v_{dock},\; v_{max}\right), \qquad
\dot{\mathbf{x}}_{des} = -v_c\,\hat{\mathbf{a}} - k_{lat}\,\mathbf{x}_\perp, \qquad
\mathbf{u} = k_v\left(\dot{\mathbf{x}} - \dot{\mathbf{x}}_{des}\right)
$$

**Closed-loop behaviour** (assuming the velocity loop is fast, $k_v \gg k_g, k_{lat}$):

- *Longitudinal.* $\dot\ell = -k_g\ell$ while $k_g\ell > v_{dock}$: an exponential decay with
  time constant $1/k_g$ ($\approx 67$ s). Below $\ell^*=v_{dock}/k_g = 20$ m the speed is held at
  $v_{dock}$, which gives **finite-time arrival at a known, controlled contact speed**.
  Time from 1 km: $\tfrac{1}{k_g}\ln(1000/20) + 20/v_{dock}\approx 260 + 65\approx 320$ s.
- *Lateral.* $\dot{\mathbf{x}}_\perp = -k_{lat}\mathbf{x}_\perp$: the error dies out with time
  constant $1/k_{lat}$ ($\approx 33$ s), so the vehicle converges onto the axis and stays in the corridor.
- *Thrust.* The largest deceleration is $k_g v_c$; at the 25 m/s cap this is
  $0.015\times 25=0.375\ \mathrm{m/s^2}$, inside the 0.5 limit. This is the design rule:
  $k_g\,v_{max}\le a_{max}$.

**Why a separate terminal law.** Midcourse laws minimise fuel or time over tens of kilometres;
they do not control the contact speed or the lateral offset. The terminal law gives up
optimality for predictability: a bounded, monotonically falling closing speed and a straight
corridor, which is what docking sensors and safety rules require.

---

## 10. Phased guidance: midcourse to terminal

**Class:** `PhasedGuidance` in `guidance/terminal.py`.

$$
\mathbf{u} = \begin{cases}
\mathbf{u}_{mid}(\text{state}) & R > R_{sw} \;\text{(until first crossing)}\\
\mathbf{u}_{term}(\text{state}) & \text{afterwards}
\end{cases}
$$

The handover is **one-way** (latched at the first time $R\le R_{sw}$, default 1 km), which avoids
chattering between laws when the range fluctuates around the threshold. The handover time is stored
as `switch_time` and returned in `SimulationResult.events["handover"]`; the plots draw it as a
dotted line.

**Handover compatibility.** The terminal law is a velocity-tracking law, so a mismatch between the
actual velocity and the glideslope velocity at the switch is absorbed as a transient of size
$k_v\,\Delta v$. Choose $R_{sw}$ such that the midcourse velocity at the switch is close to
$k_g R_{sw}$ (15 m/s at 1 km). The sliding-mode + terminal run arrives at the switch at about
20 m/s, so the thrust briefly reaches the 0.5 m/s² limit, and the adaptive ZEM/ZEV + terminal run
arrives slower and switches smoothly.

---

## 11. Comparison and how to choose

Results for the default scenario (26 km initial range, $a_{max}=0.5\ \mathrm{m/s^2}$, 2 m capture):

| Law | Arrival (s) | Δv (m/s) | Final rel. speed (m/s) | Peak a (m/s²) |
|---|---|---|---|---|
| Velocity pursuit | 889 | 281 | 0.08 | 0.50 |
| PD feedback | 1107 | 188 | 0.02 | 0.50 |
| ZEM/ZEV (fixed time) | 896 | 78 | 0.76 | 0.24 |
| ZEM/ZEV (adaptive $t_{go}$) | 956 | 79 | 0.46 | 0.27 |
| Optimal (docking speed) | 898 | 78 | 0.56 | 0.36 |
| Sliding mode | 600 | 155 | 2.65 | 0.38 |
| Adaptive ZEM/ZEV + terminal | 1120 | 80 | 0.30 | 0.27 |
| Sliding mode + terminal | 814 | 157 | 0.30 | 0.50 |

The two lowest relative speeds in the first two rows (0.08 and 0.02 m/s) are *incidental*: they come
from slow asymptotic convergence, not from a commanded docking speed.

| Need | Use |
|---|---|
| Minimum fuel, arrival time fixed by the mission | ZEM/ZEV (section 5) |
| Minimum fuel, arrival time free | Adaptive ZEM/ZEV (6) |
| Controlled non-zero closing speed at arrival | Terminal-velocity optimal (7) or terminal approach (9) |
| Fastest arrival, robustness to model error | Sliding mode (8) |
| Predictable final corridor and contact speed | Phased with a terminal law (10) |
| A simple baseline to compare against | Velocity pursuit (3), PD (4) |

Rules of thumb:

- Fuel-optimal laws are fragile if the arrival time is infeasible: the command scales as $1/T^2$, so
  a short $T$ saturates the thrust.
- Any scheduled-speed law must satisfy (schedule slope) $\times$ (speed) $\le a_{max}$, or it overshoots
  (sections 3, 4 and 9).
- Optimality, robustness and terminal precision pull in different directions, which is why
  real systems use phased guidance.

---

## 12. Metrics

`simulation/metrics.py` reports, for each run:

| Metric | Definition |
|---|---|
| Captured | final range $\le$ capture radius |
| Arrival time | time of the last sample |
| Final range / relative speed | at the last sample |
| Δv | $\int\lVert\mathbf{a}_c\rVert\,dt$ (thrust acceleration integrated over time; a proxy for propellant) |
| Peak acceleration | $\max\lVert\mathbf{a}_c\rVert$ |
| Saturated fraction | share of samples at the acceleration limit (a sign the law asks for more than the vehicle can give) |

Derived histories used in the plots (`SimulationResult`): LOS azimuth and elevation in the target's LVLH frame,
the inertial LOS rate $\lVert\mathbf{x}\times\dot{\mathbf{x}}\rVert/R^2$, and the approach angle between the closing velocity and the LOS (undefined below 1 cm/s).

---

## 13. References

- W. H. Clohessy and R. S. Wiltshire, "Terminal Guidance System for Satellite Rendezvous," *Journal of the Aerospace Sciences*, 27(9), 1960.
- W. Fehse, *Automated Rendezvous and Docking of Spacecraft*, Cambridge University Press, 2003.
- A. E. Bryson and Y.-C. Ho, *Applied Optimal Control*, Taylor & Francis, 1975 (the linear-in-time optimal control used in section 5).
- J.-J. E. Slotine and W. Li, *Applied Nonlinear Control*, Prentice Hall, 1991 (sliding surfaces, boundary layers).

# relguide

A unified relative guidance and control framework covering **Missile Homing Intercept** and **Spacecraft Orbital Rendezvous / Docking**. Built on a shared mathematical core of relative kinematics, line-of-sight (LOS) dynamics, Keplerian & Clohessy-Wiltshire (CW) astrodynamics, generalized zero-effort miss/velocity (ZEM/ZEV) optimal control, sliding-mode control, and hybrid simulation (continuous thrust RK4 and DOP853 impulsive/continuous integration).

## Architecture

```
relguide/
  core/                          # Universal relative guidance foundation
    astrodynamics/               # Kepler propagation, Lambert solver, CW STM, LVLH frames
    kinematics.py                # 3D line-of-sight vectors, LOS rate, closing speed, t_go
    state.py & base.py           # Unified GuidanceState & GuidanceLaw interface (continuous & impulsive)
    laws/                        # Proportional navigation, ZEM/ZEV, sliding mode, glideslope
    models/                      # Autopilot lag, slew limits, seeker FOV/noise, alpha-beta filter
    simulation/                  # Hybrid simulation: fixed-step RK4 and high-precision solve_ivp (DOP853)
    visualization/               # Shared multi-panel dashboards, trajectory plots, and metrics
  applications/                  # Domain-specific applications
    missile_homing/              # High-speed intercept, maneuvering targets (weaving, ballistic, 4g turn),
                                 # TPN, PPN, APN, OGL, BPN (IACG), Monte Carlo CEP dispersion
    orbital_rendezvous/          # Proximity operations & proximity docking in LEO/GEO:
                                 # Astrodynamics & impulsive transfers (Two-Impulse CW, Lambert, Glideslope)
                                 # Optimal state-feedback & predictive control (LQR, MPC)
                                 # Continuous homing-style laws (ZEM/ZEV, Sliding Mode, Corridor Pursuit)
                                 # Multi-panel 2D/3D visualizations, LOS sphere, and animated GIF playback
examples/
  compare_laws.py                # Master runner: benchmarks both missile and orbital applications
  compare_missile_laws.py        # Dedicated missile homing intercept benchmark
  compare_rendezvous_laws.py     # Dedicated orbital rendezvous benchmark (continuous + impulsive/control)
tests/
  test_core.py                   # Kinematics, autopilot, seeker, and base law tests
  test_astrodynamics.py          # CW STM, Kepler propagator, Lambert solver, LVLH frame round-trip
  test_missile_homing.py         # Intercept accuracy, target maneuvers, Monte Carlo dispersion
  test_orbital_rendezvous.py     # Two-body energy conservation, impulsive & control algorithms, capture tests
docs/
  guidance_theory.md             # Complete mathematical derivations (Bryson-Ho, CW, Lambert, LQR/MPC)
  clohessy_wiltshire_derivation.md # First-principles derivation of CW equations & analytical STM
  rendezvous_merge_plan.md       # Full architecture & migration roadmap from spacecraft_rendezvous
```

## Guidance & Control Algorithm Suite

### 1. Missile Homing Intercept (`relguide.applications.missile_homing`)
| Law | Family | Key Characteristic |
|---|---|---|
| `TrueProportionalNavigation` | Classical | Commands acceleration normal to LOS: $u = N V_c (oldsymbol{\Omega} 	imes \hat{\mathbf{l}})$ |
| `PureProportionalNavigation` | Classical | Commands acceleration normal to missile velocity vector |
| `AugmentedProportionalNavigation` | Modern | Direct target maneuver feedforward: $u_{	ext{TPN}} + rac{N}{2}\mathbf{a}_T^{\perp}$ |
| `OptimalHomingGuidance` | Optimal | Bryson-Ho energy-optimal intercept ($N'=3$, $u = rac{3}{t_{go}^2}\mathbf{ZEM}$) |
| `BiasedProportionalNavigation` | Advanced | Impact Angle Control Guidance (IACG) with smooth terminal fade |
| `SlidingModeHomingGuidance` | Robust | Nulls LOS angular rate with continuous boundary layer |

### 2. Spacecraft Orbital Rendezvous (`relguide.applications.orbital_rendezvous`)
| Algorithm | Actuation | Type | Description |
|---|---|---|---|
| `TwoImpulseCW` | Impulsive | Analytical | Classical Clohessy-Wiltshire 2-burn rendezvous via partitioned STM targeting |
| `LambertTransfer` | Impulsive | Nonlinear Astrodyn. | Universal variable Keplerian zero-rev Lambert solver for large orbital transfers |
| `ImpulsiveGlideslope` | Impulsive | Multi-leg | Line-of-sight safety corridor multi-burn glideslope approach |
| `LQRController` | Continuous | State Feedback | Discrete-time optimal infinite-horizon state feedback via CW ZOH model & DARE |
| `MPCController` | Continuous | Constrained Optimal | Receding horizon optimal control with box thrust limits via bounded least squares |
| `ZemZevGuidance` | Continuous | Optimal | Energy-optimal soft docking ($N'=6,2$, $u = rac{6}{	au^2}\mathbf{ZEM} - rac{2}{	au}\mathbf{ZEV}$) |
| `AdaptiveTgoGuidance` | Continuous | Optimal | ZEM/ZEV with online range-scheduled time-to-go |
| `TerminalVelocityGuidance` | Continuous | Optimal | Arrives with prescribed docking speed along approach axis |
| `SlidingModeGuidance` | Continuous | Robust | Speed-limited sliding surface with braking corridor |
| `TerminalApproachGuidance` | Continuous | Pursuit | Glideslope corridor tracking with lateral error reduction |
| `PhasedGuidance` | Continuous | Hybrid | Midcourse-to-terminal one-way handover manager |

## Installation & Requirements

Requires Python 3.10+ and standard numerical scientific libraries:
```bash
pip install numpy scipy matplotlib pytest
```

## Quick Start

```powershell
# Run the complete test suite (38 unit & regression tests)
pytest

# Run master comparison benchmarks (outputs tables, 2D/3D figures, and GIF)
python examples/compare_laws.py

# Or run individual application benchmarks
python examples/compare_missile_laws.py
python examples/compare_rendezvous_laws.py
```

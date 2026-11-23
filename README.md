# lvguidance

A unified relative guidance framework covering **Missile Homing Intercept** and **Orbital Rendezvous / Docking**. Built on a shared mathematical core of relative kinematics, line-of-sight (LOS) dynamics, generalized zero-effort miss/velocity (ZEM/ZEV) optimal control, and sliding-mode control.

## Architecture

```
lvguidance/
  core/                          # Universal relative guidance foundation
    kinematics.py                # 3D line-of-sight vectors, LOS rate, closing speed, t_go
    state.py & base.py           # Unified GuidanceState & GuidanceLaw interface
    laws/                        # Proportional navigation, ZEM/ZEV, sliding mode, glideslope
    models/                      # Autopilot response lag, slew rate limits, seeker FOV/noise
    simulation/                  # RK4 integrator, stepping engine, trajectory result, metrics
    visualization/               # Shared multi-panel dashboards and comparison plots
  applications/                  # Domain-specific applications
    missile_homing/              # High-speed intercept, maneuvering targets (weaving, constant g),
                                 # TPN, PPN, APN, OGL, Monte Carlo CEP evaluation
    orbital_rendezvous/          # Two-body Earth gravity, LVLH frame, differential gravity,
                                 # docking speed corridors, fuel Δv optimization
examples/
  compare_laws.py                # Master runner: benchmarks both applications
  compare_missile_laws.py        # Dedicated missile homing intercept suite
  compare_rendezvous_laws.py     # Dedicated orbital rendezvous suite
tests/
  test_core.py                   # Kinematics, autopilot, seeker, and base law tests
  test_missile_homing.py         # Intercept accuracy, target maneuvers, Monte Carlo tests
  test_orbital_rendezvous.py     # Orbit conservation, LVLH frame, docking capture tests
docs/
  guidance_theory.md             # Theoretical derivations
  improvement_plan.md            # Architecture roadmap & design specification
```

## Guidance Laws

| Law | Family | Application | Key Characteristic |
|---|---|---|---|
| `TrueProportionalNavigation` | Classical | Missile | Commands normal to LOS: $u = N V_c (\boldsymbol{\Omega} \times \hat{\mathbf{l}})$ |
| `PureProportionalNavigation` | Classical | Missile | Commands normal to missile velocity vector |
| `AugmentedProportionalNavigation` | Modern | Missile | Compensates target maneuver: $u_{\text{TPN}} + \frac{N}{2}\mathbf{a}_T^{\perp}$ |
| `OptimalHomingGuidance` | Optimal | Missile | Energy-optimal intercept (Bryson-Ho $N'=3$, $u = \frac{3}{t_{go}^2}\mathbf{ZEM}$) |
| `SlidingModeHomingGuidance` | Robust | Missile | Nulls LOS angular rate with continuous boundary layer |
| `ZemZevGuidance` | Optimal | Rendezvous | Energy-optimal soft docking ($N'=6,2$, $u = \frac{6}{\tau^2}\mathbf{ZEM} - \frac{2}{\tau}\mathbf{ZEV}$) |
| `AdaptiveTgoGuidance` | Optimal | Rendezvous | ZEM/ZEV with online range-scheduled time-to-go |
| `TerminalVelocityGuidance` | Optimal | Rendezvous | Prescribes non-zero contact speed along docking corridor |
| `SlidingModeGuidance` | Robust | Rendezvous | Speed-limited sliding surface with braking profile |
| `TerminalApproachGuidance` | Terminal | Rendezvous | Glideslope corridor tracking with lateral error reduction |
| `PhasedGuidance` | Hybrid | Both | Midcourse-to-terminal one-way handover manager |

## Run

```powershell
# Run the complete test suite
pytest

# Run master comparison benchmarks
python examples/compare_laws.py

# Or run application-specific benchmarks
python examples/compare_missile_laws.py
python examples/compare_rendezvous_laws.py
```

Generated figure dashboards and metric charts are saved in `outputs/missile/` and `outputs/rendezvous/`.

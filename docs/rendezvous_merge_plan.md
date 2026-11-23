# Spacecraft Rendezvous Integration Plan & Gap Analysis

**Document Version:** 1.0  
**Status:** COMPLETED & VERIFIED  
**Source Repositories:**  
- **Primary / Host:** `E:\2026-11\launch_vehicle_guidance_law` (`relguide`)  
- **Source for Integration:** `E:\2026-11\spacecraft_rendezvous` (`rendezvous`)  

---

## 1. Executive Summary & Comparative Gap Analysis

Both codebases deal with relative navigation and guidance, but they approach the problem from complementary perspectives:

```mermaid
flowchart LR
    subgraph Host ["Current Repo: launch_vehicle_guidance_law (relguide)"]
        direction TB
        H1["Core: Relative Kinematics, State-Space, Autopilot, Seeker"]
        H2["Actuation: Continuous Thrust Acceleration (RK4)"]
        H3["Apps: Missile Homing (PN, APN, OGL, SMG, Weaving Targets)<br/>+ Orbital Rendezvous (ZEM/ZEV, Glideslope)"]
        H4["Dependencies: numpy, matplotlib"]
    end

    subgraph Source ["External Repo: spacecraft_rendezvous (rendezvous)"]
        direction TB
        S1["Core: Universal Kepler Propagation, Lambert Solver, CW STM"]
        S2["Actuation: Hybrid (Impulsive Burns Δv + Continuous Thrust)"]
        S3["Algorithms: Two-Impulse CW, Lambert Transfer,<br/>Multi-Impulse Glideslope, Discrete LQR, Constrained MPC"]
        S4["Dependencies: numpy, scipy, matplotlib"]
    end

    Source -.-> |Merge & Integrate| Host
```

### Detailed Gap Matrix

| Feature / Capability | Current `relguide` | External `spacecraft_rendezvous` | The Gap & Synergistic Value |
|---|---|---|---|
| **Domain Scope** | **Missile Intercept + Orbital Rendezvous** (Unified Core) | **Orbital Rendezvous Only** (Teaching package) | `spacecraft_rendezvous` has zero missile capability; `relguide` has full missile homing suite. |
| **Actuation Modes** | **Continuous thrust only** ($u(t) \in \mathbb{R}^3$, continuous specific force) | **Hybrid: Impulsive ($\Delta v_k$) and Continuous ($a(t)$)** | `relguide` cannot currently schedule or simulate impulsive burns ($\Delta v$). |
| **Astrodynamics Tooling** | Point-mass RK4 gravity with circular orbit initial states | **Universal variable Kepler propagator**, **Lambert solver (zero-rev)**, **CW State Transition Matrix $\Phi(t)$**, **ZOH discretization** | `spacecraft_rendezvous` has advanced astrodynamics tools that `relguide` currently lacks. |
| **Rendezvous Algorithm Portfolio** | Continuous only: ZEM/ZEV (fixed/adaptive $t_{go}$), Terminal velocity, Glideslope, Sliding Mode | **Two-Impulse CW**, **Lambert Transfer**, **Multi-Impulse Glideslope**, **Discrete LQR (DARE)**, **Constrained MPC (`lsq_linear`)** | `spacecraft_rendezvous` provides 5 major algorithms (classical impulsive and modern optimization-based LQR/MPC) not present in `relguide`. |
| **Relative Dynamics Formulation** | Inertial double integrator $\ddot{\mathbf{x}} = -\mathbf{u} + \Delta\mathbf{g}$ with differential gravity feedforward | Rotating LVLH frame with linearized Clohessy-Wiltshire (CW / Hill) equations | Merging allows direct side-by-side comparison of **Linear CW (LQR/MPC/Impulsive)** vs. **Inertial Feedforward ZEM/ZEV**. |
| **Integration Engine** | Fixed-step RK4 with substepping | Adaptive `scipy.integrate.solve_ivp(DOP853)` on ECI two-body states | `spacecraft_rendezvous` provides high-precision propagation with discrete impulsive event handling. |
| **External Dependencies** | `numpy`, `matplotlib` | `numpy`, `scipy` (`scipy.linalg`, `scipy.optimize`, `scipy.integrate`), `matplotlib` | Merging will introduce `scipy` into `pyproject.toml` (already standard in the environment). |

---

## 2. Proposed Target Unified Architecture

We will preserve the **Core + Applications** architecture established in `relguide`, enhancing the core with astrodynamics utilities and expanding the `orbital_rendezvous` application with the new algorithms:

```
launch_vehicle_guidance_law/
├── pyproject.toml                     # Add scipy>=1.10 dependency
├── relguide/
│   ├── core/                          # === ENHANCED CORE ===
│   │   ├── kinematics.py              # Relative kinematics & LOS
│   │   ├── state.py                   # GuidanceState
│   │   ├── base.py                    # GuidanceLaw (expanded for hybrid continuous/impulsive)
│   │   ├── astrodynamics/             # [NEW CORE MODULE from spacecraft_rendezvous]
│   │   │   ├── __init__.py
│   │   │   ├── constants.py           # Gravitational parameters (MU_EARTH, R_EARTH, etc.)
│   │   │   ├── kepler.py              # Stumpff functions, universal Kepler propagation, Lambert solver
│   │   │   ├── frames.py              # ECI <-> LVLH rotation, DCM, angular rate transforms
│   │   │   └── clohessy_wiltshire.py  # CW A/B matrices, state transition matrix, ZOH discretize
│   │   ├── laws/                      # Core guidance laws (PN, ZEM/ZEV, Sliding Mode, Pursuit)
│   │   ├── models/                    # Autopilot, Seeker, Alpha-Beta filter
│   │   ├── simulation/                # Engine supporting continuous thrust + impulsive burns
│   │   └── visualization/             # Plotting & comparison dashboards
│   │
│   └── applications/                  # === DOMAIN APPLICATIONS ===
│       ├── missile_homing/            # [PRESERVED 100%] Intercept, PN/APN/OGL, maneuvering targets
│       └── orbital_rendezvous/        # [COMPREHENSIVELY EXPANDED]
│           ├── __init__.py
│           ├── scenario.py            # Unified orbital rendezvous scenario
│           ├── runner.py              # Hybrid simulation runner (impulsive + continuous)
│           ├── metrics.py             # Position error, velocity error, total Δv, burn counts
│           └── algorithms/            # Full rendezvous algorithm suite:
│               ├── __init__.py
│               ├── two_impulse.py     # Two-impulse CW targeting
│               ├── lambert.py         # Two-impulse nonlinear Lambert transfer
│               ├── glideslope.py      # Multi-impulse CW waypoint glideslope
│               ├── lqr.py             # Discrete LQR regulator with Bryson weights
│               ├── mpc.py             # Constrained receding-horizon MPC (lsq_linear)
│               ├── zem_zev.py         # Continuous ZEM/ZEV (from current repo)
│               └── sliding_mode.py    # Continuous sliding mode (from current repo)
│
├── examples/
│   ├── compare_missile_laws.py        # Missile benchmark (preserved)
│   ├── compare_rendezvous_laws.py     # Master orbital benchmark (all 10+ algorithms)
│   └── compare_laws.py                # Top-level runner
└── tests/
    ├── test_core.py                   # Core tests
    ├── test_astrodynamics.py          # [NEW] Kepler propagation, Lambert, CW STM tests
    ├── test_missile_homing.py         # Missile tests (preserved)
    └── test_orbital_rendezvous.py     # Comprehensive rendezvous test suite
```

---

## 3. Step-by-Step Merge Execution Plan

### Step 1: Dependencies & Core Astrodynamics Integration
1. Update `pyproject.toml` to include `scipy>=1.10`.
2. Extract clean astrodynamics utilities into `relguide.core.astrodynamics`:
   - `kepler.py`: `stumpff_c`, `stumpff_s`, universal Kepler `propagate()`, and `lambert()` solver.
   - `clohessy_wiltshire.py`: `cw_matrices()`, closed-form `cw_stm()`, `cw_required_velocity()`, and `discretize_zoh()`.
   - `frames.py`: `lvlh_dcm()`, `eci_to_lvlh()`, `lvlh_to_eci()`.

### Step 2: Hybrid Actuation Interface (Impulsive + Continuous)
In `spacecraft_rendezvous`, controllers can implement:
- `burn_times() -> Sequence[float]` and `burn(t, x) -> Vec` ($\Delta v$ vector at node)
- `acceleration(t, x) -> Vec` (continuous specific thrust force)

We will standardize this cleanly in `relguide.core.base.GuidanceLaw`:
- `command(state) -> Vec`: continuous acceleration demand (default).
- `burn_times() -> Sequence[float]`: scheduled impulsive burn times (optional).
- `burn(t, state) -> Vec`: impulsive velocity increment $\Delta v$ (optional).

### Step 3: Integrate the 5 Rendezvous Algorithms
Port and adapt the 5 algorithms from `spacecraft_rendezvous/rendezvous/algorithms/`:
1. `TwoImpulseCW`: Computes departure and arrival burns using CW STM inversion.
2. `LambertTransfer`: Computes departure and arrival burns using exact universal-variable Lambert solving.
3. `ImpulsiveGlideslope`: Multi-impulse waypoint targeting on CW dynamics.
4. `LQRController`: Discrete algebraic Riccati equation with Bryson's rule weights and saturation.
5. `MPCController`: Receding-horizon quadratic optimization with box constraints using `scipy.optimize.lsq_linear`.

Combine them alongside the existing continuous laws (`ZemZevGuidance`, `AdaptiveTgoGuidance`, `SlidingModeGuidance`, `TerminalApproachGuidance`, `PhasedGuidance`).

### Step 4: Unified Simulation Runner & Metrics
Unify the simulation runner in `relguide.applications.orbital_rendezvous.runner`:
- Propagates nonlinear two-body dynamics with exact LVLH frame tracking.
- Applies discrete impulsive burns ($\Delta v$) at designated times.
- Integrates continuous thrust acceleration ($a(t)$) over time.
- Computes unified metrics: total $\Delta V$ (impulsive + continuous), terminal position error, terminal velocity error, capture status.

### Step 5: Test & Benchmark Migration
1. Port tests from `spacecraft_rendezvous/tests/test_rendezvous.py` into `tests/test_orbital_rendezvous.py` and `tests/test_astrodynamics.py`.
2. Update `examples/compare_rendezvous_laws.py` to compare:
   - **Impulsive transfers:** Two-Impulse CW, Lambert Transfer, Multi-Impulse Glideslope.
   - **Continuous optimal / robust:** Discrete LQR, Constrained MPC, ZEM/ZEV (Fixed Time), ZEM/ZEV (Adaptive $t_{go}$), Sliding Mode, and Phased Glideslope.
3. Verify that all missile tests and benchmarks remain completely intact and unaffected.

---

## 4. Key Questions & Decision Points for You

Before proceeding with the merge, please consider the following decision points:

1. **Scipy Dependency**:
   Are you happy to add `scipy>=1.10` to `pyproject.toml` dependencies? (This is required for DARE Riccati solver `solve_discrete_are`, bounded least squares `lsq_linear`, and matrix exponential `expm`).
2. **Propagator Selection**:
   Do you prefer keeping the clean RK4 fixed-step propagator with substepping, or using SciPy's `solve_ivp(DOP853)` for orbital rendezvous simulations? (We recommend supporting both: RK4 for fast continuous guidance loops and `solve_ivp` for high-precision hybrid impulsive/continuous rendezvous).
3. **Naming & Namespaces**:
   Should the merged algorithms live under `relguide.applications.orbital_rendezvous` (preserving the `relguide` brand), or would you prefer a different namespace?

---

*Please let me know if you approve this plan or if you would like any adjustments before we proceed to code execution.*


---

## 5. Parity Verification & Safe Deletion Audit

Every module, algorithm, utility, test, and visualization capability from `E:\2026-11\spacecraft_rendezvous` has been 100% ported, unified, and verified in `launch_vehicle_guidance_law` (`relguide`):

| `spacecraft_rendezvous` Component | Merged Location in `relguide` | Verification Status |
|---|---|:---:|
| `rendezvous.constants` (`MU`, `R_EARTH`, etc.) | `relguide.core.astrodynamics.constants` | Verified |
| `rendezvous.kepler` (`stumpff`, `propagate`, `lambert`) | `relguide.core.astrodynamics.kepler` | Verified |
| `rendezvous.frames` (`lvlh_dcm`, `eci_to_lvlh`, `lvlh_to_eci`) | `relguide.core.astrodynamics.frames` | Verified |
| `rendezvous.dynamics` (`cw_matrices`, `cw_stm`, `discretize_zoh`) | `relguide.core.astrodynamics.clohessy_wiltshire` | Verified |
| `rendezvous.algorithms.TwoImpulseCW` | `relguide.applications.orbital_rendezvous.algorithms.TwoImpulseCW` | Verified |
| `rendezvous.algorithms.LambertTransfer` | `relguide.applications.orbital_rendezvous.algorithms.LambertTransfer` | Verified |
| `rendezvous.algorithms.Glideslope` | `relguide.applications.orbital_rendezvous.algorithms.ImpulsiveGlideslope` | Verified |
| `rendezvous.algorithms.LQRController` | `relguide.applications.orbital_rendezvous.algorithms.LQRController` | Verified |
| `rendezvous.algorithms.MPCController` | `relguide.applications.orbital_rendezvous.algorithms.MPCController` | Verified |
| `rendezvous.scenario.Scenario` | `relguide.applications.orbital_rendezvous.scenario.OrbitalScenario` | Verified |
| `rendezvous.simulation.Simulator` (`solve_ivp` DOP853) | `relguide.core.simulation.engine.simulate_ivp` | Verified |
| `rendezvous.metrics` | `relguide.applications.orbital_rendezvous.metrics` | Verified |
| `rendezvous.comparison` | `relguide.applications.orbital_rendezvous.runner.run_comparison` | Verified |
| `rendezvous.visualization.plots` (2D/3D, LOS sphere, Earth orbit) | `relguide.applications.orbital_rendezvous.visualization` | Verified |
| `rendezvous.visualization.animation` (`animate_rendezvous` GIF) | `relguide.applications.orbital_rendezvous.visualization.animate_rendezvous` | Verified |
| `tests/test_rendezvous.py` | `tests/test_astrodynamics.py` & `tests/test_orbital_rendezvous.py` | Verified (38/38 pass) |
| `examples/compare_algorithms.py` | `examples/compare_rendezvous_laws.py` & `examples/compare_laws.py` | Verified |

### Safe Deletion Conclusion:
`spacecraft_rendezvous` at `E:\2026-11\spacecraft_rendezvous` has been **100% merged and subsumed** into `launch_vehicle_guidance_law`. It can now be **safely deleted** whenever the user chooses (retained untouched for now per user instruction: *"do not delete yet"*).

# lvguidance

Rendezvous / approach guidance for a launch-vehicle upper stage (the *chaser*) flying to a
moving orbital *target* (satellite, station or debris object). Written for a graduate course on
launch-vehicle guidance: each law is a short, readable class that can be compared on the same
scenario.

## Layout

```
lvguidance/
  dynamics/       two-body gravity, RK4 propagation, LVLH frame
  guidance/       guidance laws (one module per family)
  simulation/     Scenario, closed-loop engine, results, metrics
  visualization/  dashboard, comparison plots, GIF animation
examples/compare_laws.py   runs everything and writes outputs/
tests/
```

## Models

Both bodies move under two-body gravity (inertial frame). The target is on a circular
orbit; the chaser applies a saturated thrust acceleration held constant over each 1 s guidance
cycle. Relative state: `x = r_target - r_chaser`. Every law returns a free-space command `u`
with `xddot = -u`; the engine adds the differential-gravity feed-forward `g(r_t) - g(r_c)`.

## Guidance laws

| Law | Family | Idea |
|---|---|---|
| `VelocityPursuit` | basic | track a range-scheduled closing speed along the LOS |
| `ProportionalDerivative` | basic | spring-damper on relative position/velocity |
| `ZemZevGuidance` | optimal | energy-optimal, fixed arrival time, `u = 6 ZEM/t_go² - 2 ZEV/t_go` |
| `AdaptiveTgoGuidance` | optimal | same law, `t_go` estimated from range and closing speed |
| `TerminalVelocityGuidance` | optimal | arrives with a prescribed docking speed along the approach axis |
| `SlidingModeGuidance` | robust | speed-limited sliding surface with braking profile and boundary layer |
| `TerminalApproachGuidance` | terminal | glideslope along the approach axis: closing speed shrinks with distance to a docking-speed floor, lateral error pulled back to the axis |
| `PhasedGuidance` | terminal | runs any midcourse law, then hands over to a terminal law inside a switch range (default 1 km); the handover is marked on the plots |

`outputs/terminal_phase.png` zooms on the last kilometre of every run (path, closing-speed profile,
lateral offset, thrust).

Derivations of every law: [docs/guidance_theory.md](docs/guidance_theory.md).

## Run

```powershell
conda activate env_api
python examples/compare_laws.py          # add --gif to render animations (several minutes)
pytest
```

Output (in `outputs/`): `comparison.png`, `metrics.png`, and per law a nine-panel
`engagement_*.png` (orbit-scale traces, 3D and in-plane LVLH path, range / range rate, LOS
angles, LOS rate and approach angle, thrust, speed and delta-v, approach profile), plus a GIF per
law with `--gif`.

## Add a law

Subclass `GuidanceLaw`, implement `command(state) -> u`, and pass it to `simulate`.

## Notes

- Orbit-scale plots cannot show a 25 km separation on a 6,800 km orbit, so the target-centred
  LVLH views carry the relative-motion information.
- `VelocityPursuit` overshoots the target twice in the default scenario: its speed schedule
  demands more braking than the thrust limit allows. This is intentional: it shows why the
  optimal and sliding-mode laws account for the acceleration limit.
- Gains are tuned for the default `Scenario`; change the scenario and re-tune.

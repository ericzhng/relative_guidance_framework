"""Orbital rendezvous and proximity operations package."""

from lvguidance.applications.orbital_rendezvous.algorithms import (
    ImpulsiveGlideslope,
    LambertTransfer,
    LQRController,
    MPCController,
    TwoImpulseCW,
)
from lvguidance.applications.orbital_rendezvous.dynamics import (
    lvlh_rotation,
    to_lvlh,
)
from lvguidance.applications.orbital_rendezvous.metrics import (
    RendezvousMetrics,
    compute_metrics,
    format_rendezvous_table,
    format_table,
)
from lvguidance.applications.orbital_rendezvous.runner import (
    run_comparison,
    simulate_rendezvous,
)
from lvguidance.applications.orbital_rendezvous.scenario import (
    OrbitalScenario,
    circular_state,
    gravity,
)
from lvguidance.applications.orbital_rendezvous.visualization import (
    animate_rendezvous,
    plot_earth_orbit_view,
    plot_los_sphere,
    plot_metrics_comparison,
    plot_range_history,
    plot_spherical_history,
    plot_thrust_profile,
    plot_trajectories_2d,
    plot_trajectories_3d,
)
from lvguidance.core.astrodynamics.constants import MU_EARTH, R_EARTH
from lvguidance.core.astrodynamics.frames import lvlh_dcm, lvlh_to_eci
from lvguidance.core.base import GuidanceLaw
from lvguidance.core.laws.optimal import (
    AdaptiveTgoGuidance,
    TerminalVelocityGuidance,
    ZemZevGuidance,
)
from lvguidance.core.laws.phased import PhasedGuidance
from lvguidance.core.laws.pursuit import (
    ProportionalDerivative,
    TerminalApproachGuidance,
    VelocityPursuit,
)
from lvguidance.core.laws.sliding_mode import SlidingModeGuidance

__all__ = [
    "AdaptiveTgoGuidance",
    "ImpulsiveGlideslope",
    "LQRController",
    "LambertTransfer",
    "MPCController",
    "MU_EARTH",
    "OrbitalScenario",
    "PhasedGuidance",
    "ProportionalDerivative",
    "R_EARTH",
    "RendezvousMetrics",
    "SlidingModeGuidance",
    "TerminalApproachGuidance",
    "TerminalVelocityGuidance",
    "TwoImpulseCW",
    "VelocityPursuit",
    "ZemZevGuidance",
    "animate_rendezvous",
    "circular_state",
    "compute_metrics",
    "default_impulsive_and_control_laws",
    "default_rendezvous_laws",
    "format_rendezvous_table",
    "format_table",
    "gravity",
    "lvlh_dcm",
    "lvlh_rotation",
    "lvlh_to_eci",
    "plot_earth_orbit_view",
    "plot_los_sphere",
    "plot_metrics_comparison",
    "plot_range_history",
    "plot_spherical_history",
    "plot_thrust_profile",
    "plot_trajectories_2d",
    "plot_trajectories_3d",
    "run_comparison",
    "simulate_rendezvous",
    "to_lvlh",
]


def default_rendezvous_laws() -> list[GuidanceLaw]:
    """Return default suite of guidance laws for orbital rendezvous."""
    return [
        VelocityPursuit(),
        ProportionalDerivative(),
        ZemZevGuidance(),
        AdaptiveTgoGuidance(),
        TerminalVelocityGuidance(),
        SlidingModeGuidance(),
        PhasedGuidance(AdaptiveTgoGuidance(), TerminalApproachGuidance()),
        PhasedGuidance(SlidingModeGuidance(), TerminalApproachGuidance()),
    ]


def default_impulsive_and_control_laws() -> list[GuidanceLaw]:
    """Return suite of algorithms ported from spacecraft_rendezvous."""
    return [
        TwoImpulseCW(),
        LambertTransfer(),
        ImpulsiveGlideslope(n_legs=6),
        LQRController(label="LQR (gentle)", pos_tol=100.0, vel_tol=0.5, a_max=5e-3),
        MPCController(label="MPC (aggressive)", pos_tol=100.0, vel_tol=0.5, a_max=5e-3),
    ]

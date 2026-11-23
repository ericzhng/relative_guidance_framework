"""Astrodynamics utilities: Kepler propagation, Lambert solving, CW dynamics and frames."""

from relguide.core.astrodynamics.clohessy_wiltshire import (
    cw_matrices,
    cw_required_velocity,
    cw_stm,
    discretize_zoh,
    mean_motion,
)
from relguide.core.astrodynamics.constants import GEO_RADIUS, J2_EARTH, MU_EARTH, R_EARTH
from relguide.core.astrodynamics.frames import (
    eci_to_lvlh,
    lvlh_angular_velocity,
    lvlh_dcm,
    lvlh_to_eci,
)
from relguide.core.astrodynamics.kepler import (
    propagate_kepler,
    solve_lambert,
    stumpff_c,
    stumpff_s,
)

__all__ = [
    "GEO_RADIUS",
    "J2_EARTH",
    "MU_EARTH",
    "R_EARTH",
    "cw_matrices",
    "cw_required_velocity",
    "cw_stm",
    "discretize_zoh",
    "eci_to_lvlh",
    "lvlh_angular_velocity",
    "lvlh_dcm",
    "lvlh_to_eci",
    "mean_motion",
    "propagate_kepler",
    "solve_lambert",
    "stumpff_c",
    "stumpff_s",
]

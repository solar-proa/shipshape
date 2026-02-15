# Physics computation library for hydrostatics analysis
#
# This library provides reusable functions for computing:
# - Center of Buoyancy (CoB) - centroid of submerged volume
# - Center of Gravity (CoG) - mass-weighted centroid
#
# These functions are designed to be called iteratively by a
# buoyancy equilibrium solver (Newton-Raphson).

from .center_of_buoyancy import (
    compute_center_of_buoyancy,
    compute_cob,
    transform_shape,
    compute_submerged_volume,
    DEFAULT_HULL_COMPONENTS,
)

from .center_of_mass import (
    compute_center_of_gravity,
    compute_cog,
    compute_cog_from_mass_artifact,
)

__all__ = [
    # Center of Buoyancy
    'compute_center_of_buoyancy',
    'compute_cob',
    'transform_shape',
    'compute_submerged_volume',
    'DEFAULT_HULL_COMPONENTS',
    # Center of Gravity
    'compute_center_of_gravity',
    'compute_cog',
    'compute_cog_from_mass_artifact',
]

# Physics computation library for hydrostatics analysis
#
# This library provides reusable functions for computing:
# - Center of Buoyancy (CoB) - centroid of submerged volume
# - Center of Gravity (CoG) - mass-weighted centroid
#
# These functions are designed to be called iteratively by a
# buoyancy equilibrium solver (Newton-Raphson).

from .center_of_buoyancy import (
    load_hull,
    compute_center_of_buoyancy,
    compute_cob,
    transform_shape,
    compute_submerged_volume,
    DEFAULT_HULL_GROUPS,
    DEFAULT_HULL_COMPONENTS,  # deprecated: use DEFAULT_HULL_GROUPS
)

from .center_of_mass import (
    compute_center_of_gravity,
    compute_cog,
    compute_cog_from_mass_artifact,
)

from .geometry import (
    set_backend,
    get_backend,
    get_reader,
)

__all__ = [
    # Center of Buoyancy
    'load_hull',
    'compute_center_of_buoyancy',
    'compute_cob',
    'transform_shape',
    'compute_submerged_volume',
    'DEFAULT_HULL_GROUPS',
    'DEFAULT_HULL_COMPONENTS',  # deprecated
    # Center of Gravity
    'compute_center_of_gravity',
    'compute_cog',
    'compute_cog_from_mass_artifact',
    # Geometry backend
    'set_backend',
    'get_backend',
    'get_reader',
]

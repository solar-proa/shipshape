#!/usr/bin/env python3
"""
Center of buoyancy computation for hydrostatic analysis.

This module provides functions to compute the center of buoyancy (CoB) of a
hull given a particular pose (vertical displacement z, pitch angle, roll angle).

The water plane is assumed to be at z=0 in the world frame. The hull geometry
is transformed according to the pose, and the submerged volume (below z=0) is
computed. The centroid of this submerged volume is the center of buoyancy.

Coordinate system (matching CAD model):
    X: transversal (side-to-side, vaka offset direction)
    Y: longitudinal (fore-aft, bow is positive Y)
    Z: vertical (up is positive)

    Pitch: rotation about X axis (positive = bow up)
    Roll: rotation about Y axis (positive = starboard down)

Usage:
    from shipshape.physics.center_of_buoyancy import load_hull, compute_center_of_buoyancy

    hull = load_hull("artifact/boat.design.FCStd")  # once per run
    result = compute_center_of_buoyancy(
        hull,
        z_displacement=-100,  # mm below water
        pitch_deg=2.0,        # bow up positive
        roll_deg=0.5          # starboard down positive
    )
    # result = {
    #     "CoB": {"x": ..., "y": ..., "z": ...},  # in mm
    #     "submerged_volume_mm3": ...,
    #     "submerged_volume_liters": ...,
    #     "buoyancy_force_N": ...
    # }
"""

import math

from .geometry import get_backend, get_reader


# Physical constants
SALTWATER_DENSITY_KG_M3 = 1025.0  # kg/m³
GRAVITY_M_S2 = 9.81  # m/s²


# =============================================================================
# BUOYANCY-CONTRIBUTING COMPONENTS
# =============================================================================
# These are the hull components that contribute to buoyancy (displace water).
# Patterns are matched case-insensitively against FreeCAD object labels.
#
# From src/design/central.py (vaka):
#   - "Bottom (fiberglass_bottom)"   -> pattern: "bottom"
#   - "Foam_Below_Sole (foam)"       -> pattern: "foam_below_sole"
#   - "Sole (plywood)"               -> pattern: "sole"
#   - "Hull (fiberglass)"            -> pattern: "hull"
#   - "Air_Inside_Vaka (air)"        -> pattern: "air_inside_vaka"
#
# From src/design/mirror.py (ama):
#   - "Ama_pipe_upper (pvc)"         -> pattern: "ama_pipe"
#   - "Ama_pipe_lower (pvc_bottom)"  -> pattern: "ama_pipe"
#   - "Ama_Body_Foam (foam)"         -> pattern: "ama_body_foam"
#   - "Ama_Cone_Upper (pvc)"         -> pattern: "ama_cone"
#   - "Ama_Cone_Lower (pvc_bottom)"  -> pattern: "ama_cone"
#   - "Ama_Cone_Foam (foam)"         -> pattern: "ama_cone_foam"
#
# NOT included (above waterline or not hull):
#   - Rudder, deck, solar panels, rigging, etc.
# =============================================================================

DEFAULT_HULL_GROUPS = {
    # Ama MUST come before vaka: ama parts have "pvc_bottom" in their labels
    # which would falsely match the generic "bottom" pattern if vaka were first.
    # Within each group, more specific patterns come first (e.g. ama_cone_foam
    # before ama_cone).
    "ama": ["ama_pipe", "ama_body_foam", "ama_cone_foam", "ama_cone"],
    "vaka": ["foam_below_sole", "air_inside_vaka", "bottom", "sole", "hull"],
}

# Deprecated: flattened list kept for backward compatibility.
# Prefer DEFAULT_HULL_GROUPS for new code.
DEFAULT_HULL_COMPONENTS = [
    pattern
    for group_patterns in DEFAULT_HULL_GROUPS.values()
    for pattern in group_patterns
]


def _make_rotation_matrix(pitch_deg: float, roll_deg: float) -> list[list[float]]:
    """
    Create a 4×4 rotation matrix for pitch and roll.

    Coordinate system (matching CAD model):
        X: transversal (side-to-side)
        Y: longitudinal (fore-aft, bow is positive Y)
        Z: vertical (up is positive)

    Pitch: rotation about the X axis (positive = bow up)
    Roll: rotation about the Y axis (positive = starboard down / port up)

    Combined rotation: R = Ry(roll) * Rx(pitch)
    """
    pitch_rad = math.radians(pitch_deg)
    roll_rad = math.radians(roll_deg)

    cos_p = math.cos(pitch_rad)
    sin_p = math.sin(pitch_rad)
    cos_r = math.cos(roll_rad)
    sin_r = math.sin(roll_rad)

    return [
        [cos_r,   sin_r * sin_p,  sin_r * cos_p,  0],
        [0,       cos_p,          -sin_p,          0],
        [-sin_r,  cos_r * sin_p,  cos_r * cos_p,  0],
        [0,       0,              0,               1],
    ]


def transform_shape(shape, z_displacement: float, pitch_deg: float,
                    roll_deg: float, rotation_center=None):
    """
    Transform a shape by applying z displacement and pitch/roll rotations.

    Args:
        shape: The shape to transform (opaque backend object)
        z_displacement: Vertical displacement in mm (negative = sink)
        pitch_deg: Pitch angle in degrees (positive = bow up)
        roll_deg: Roll angle in degrees (positive = starboard down)
        rotation_center: (x, y, z) tuple for rotation center (default: shape centroid)

    Returns:
        Transformed copy of the shape
    """
    geo = get_backend()

    if rotation_center is None:
        rotation_center = geo.centroid(shape)

    cx, cy, cz = rotation_center

    # Create a copy to avoid modifying the original
    transformed = geo.copy(shape)

    # Apply rotation if any
    if abs(pitch_deg) > 1e-6 or abs(roll_deg) > 1e-6:
        # Translate to origin
        geo.translate(transformed, -cx, -cy, -cz)

        # Apply rotation
        rot_matrix = _make_rotation_matrix(pitch_deg, roll_deg)
        transformed = geo.transform(transformed, rot_matrix)

        # Translate back
        geo.translate(transformed, cx, cy, cz)

    # Apply z displacement
    geo.translate(transformed, 0, 0, z_displacement)

    return transformed


def compute_submerged_volume(shape, water_level_z: float = 0.0) -> dict:
    """
    Compute the submerged portion of a shape below a water plane.

    Args:
        shape: The shape (already transformed to final pose)
        water_level_z: Z coordinate of the water surface (default: 0)

    Returns:
        Dictionary with:
        - submerged_shape: The submerged portion (or None)
        - volume_mm3: Volume in mm³
        - CoB: Center of buoyancy as {"x", "y", "z"} in mm
    """
    geo = get_backend()

    xmin, ymin, zmin, xmax, ymax, zmax = geo.bounding_box(shape)

    # Check if any part is below water
    if zmin >= water_level_z:
        return {
            "submerged_shape": None,
            "volume_mm3": 0.0,
            "CoB": {"x": 0.0, "y": 0.0, "z": 0.0}
        }

    margin = 1000  # mm margin around the shape
    box_origin = (xmin - margin, ymin - margin, zmin - margin)
    box_height = water_level_z - (zmin - margin)

    if box_height <= 0:
        return {
            "submerged_shape": None,
            "volume_mm3": 0.0,
            "CoB": {"x": 0.0, "y": 0.0, "z": 0.0}
        }

    box_size = (
        (xmax - xmin) + 2 * margin,
        (ymax - ymin) + 2 * margin,
        box_height,
    )

    submerged = geo.intersect_with_box(shape, box_origin, box_size)

    if submerged is None:
        return {
            "submerged_shape": None,
            "volume_mm3": 0.0,
            "CoB": {"x": 0.0, "y": 0.0, "z": 0.0}
        }

    volume_mm3 = geo.volume(submerged)
    cx, cy, cz = geo.centroid(submerged)

    return {
        "submerged_shape": submerged,
        "volume_mm3": volume_mm3,
        "CoB": {"x": cx, "y": cy, "z": cz}
    }


def _build_pattern_list(hull_groups: dict) -> list:
    """
    Flatten hull_groups into an ordered list of (pattern, group_name) tuples.

    Groups are iterated in insertion order. Within each group, patterns are
    kept in order. This list is used for first-match pattern lookup, so more
    specific patterns (e.g. "ama_cone_foam") should come before less specific
    ones (e.g. "ama_cone") inside each group.
    """
    result = []
    for group_name, patterns in hull_groups.items():
        for pattern in patterns:
            result.append((pattern, group_name))
    return result


def load_hull(fcstd_path: str, hull_groups: dict = None, hull_components: list = None) -> dict:
    """
    Load hull geometry from a CAD document.

    Opens the document, extracts hull component shapes (copied so they are
    independent of the document), computes rotation center and per-group
    reference points, then closes the document.

    This should be called **once** per run. The returned dict is passed to
    compute_center_of_buoyancy() which can then be called many times without
    any file I/O.

    Args:
        fcstd_path: Path to the CAD design file
        hull_groups: Dict mapping group names to lists of component name
                     patterns. Defaults to DEFAULT_HULL_GROUPS.
        hull_components: Deprecated. Flat list of component name patterns.
                        Ignored when hull_groups is provided.

    Returns:
        Dictionary with:
        - hull_shapes: list of {"label", "shape", "pattern", "group"} with copied shapes
        - rotation_center: (x, y, z) tuple for pose transformations
        - group_refs_body: dict mapping group name to {"x", "y", "z"} reference point
        - group_total_volumes_mm3: dict mapping group name to total volume in mm3
    """
    geo = get_backend()
    reader = get_reader()

    if hull_groups is None:
        if hull_components is not None:
            hull_groups = {"hull": hull_components}
        else:
            hull_groups = DEFAULT_HULL_GROUPS

    pattern_list = _build_pattern_list(hull_groups)

    doc = reader.open(fcstd_path)
    all_objects = reader.get_objects(doc)

    hull_shapes = []
    processed_labels = set()

    for obj in all_objects:
        if not obj["has_shape"]:
            continue

        if obj["label"] in processed_labels:
            continue

        label_lower = obj["label"].lower()

        matched_pattern = None
        matched_group = None
        for pattern, group_name in pattern_list:
            if pattern.lower() in label_lower:
                matched_pattern = pattern
                matched_group = group_name
                break

        if matched_pattern is None:
            continue

        processed_labels.add(obj["label"])
        hull_shapes.append({
            "label": obj["label"],
            "shape": obj["shape"],
            "pattern": matched_pattern,
            "group": matched_group
        })

    group_names = list(hull_groups.keys())

    if not hull_shapes:
        reader.close(doc)
        return {
            "hull_shapes": [],
            "rotation_center": (0.0, 0.0, 0.0),
            "group_refs_body": {name: {"x": 0.0, "y": 0.0, "z": 0.0} for name in group_names},
            "group_total_volumes_mm3": {name: 0.0 for name in group_names},
        }

    # Compute per-group reference points (body frame) from original geometry.
    group_x_sum = {name: 0.0 for name in group_names}
    group_y_sum = {name: 0.0 for name in group_names}
    group_z_min = {name: None for name in group_names}
    group_count = {name: 0 for name in group_names}

    for hs in hull_shapes:
        g = hs["group"]
        xmin, ymin, zmin, xmax, ymax, zmax = geo.bounding_box(hs["shape"])
        cx, cy, cz = geo.centroid(hs["shape"])

        # Skip mirror copies for z_min calculation
        if "001" not in hs["label"]:
            if group_z_min[g] is None or zmin < group_z_min[g]:
                group_z_min[g] = zmin

        group_x_sum[g] += cx
        group_y_sum[g] += cy
        group_count[g] += 1

    group_refs_body = {}
    for name in group_names:
        cnt = group_count[name]
        group_refs_body[name] = {
            "x": round(group_x_sum[name] / cnt, 2) if cnt > 0 else 0.0,
            "y": round(group_y_sum[name] / cnt, 2) if cnt > 0 else 0.0,
            "z": round(group_z_min[name], 2) if group_z_min[name] is not None else 0.0
        }

    # Compute the combined center of mass for rotation center
    total_volume = 0.0
    wx, wy, wz = 0.0, 0.0, 0.0
    for hs in hull_shapes:
        vol = geo.volume(hs["shape"])
        cx, cy, cz = geo.centroid(hs["shape"])
        total_volume += vol
        wx += cx * vol
        wy += cy * vol
        wz += cz * vol

    if total_volume > 0:
        rotation_center = (wx / total_volume, wy / total_volume, wz / total_volume)
    else:
        rotation_center = (0.0, 0.0, 0.0)

    # Compute total volumes per group
    group_total_volumes_mm3 = {name: 0.0 for name in group_names}
    for hs in hull_shapes:
        group_total_volumes_mm3[hs["group"]] += geo.volume(hs["shape"])

    reader.close(doc)

    return {
        "hull_shapes": hull_shapes,
        "rotation_center": rotation_center,
        "group_refs_body": group_refs_body,
        "group_total_volumes_mm3": group_total_volumes_mm3,
    }


def _transform_ref_point(ref_body, z_disp, pitch, roll, rot_center):
    """Transform a reference point from body to world frame."""
    rcx, rcy, rcz = rot_center
    x = ref_body["x"] - rcx
    y = ref_body["y"] - rcy
    z = ref_body["z"] - rcz

    pitch_rad = math.radians(pitch)
    roll_rad = math.radians(roll)
    cos_p, sin_p = math.cos(pitch_rad), math.sin(pitch_rad)
    cos_r, sin_r = math.cos(roll_rad), math.sin(roll_rad)

    x_new = cos_r * x + sin_r * sin_p * y + sin_r * cos_p * z
    y_new = cos_p * y - sin_p * z
    z_new = -sin_r * x + cos_r * sin_p * y + cos_r * cos_p * z

    return {
        "x": round(x_new + rcx, 2),
        "y": round(y_new + rcy, 2),
        "z": round(z_new + rcz + z_disp, 2)
    }


def compute_center_of_buoyancy(hull: dict, z_displacement: float = 0.0,
                                pitch_deg: float = 0.0, roll_deg: float = 0.0,
                                water_level_z: float = 0.0) -> dict:
    """
    Compute the center of buoyancy for a hull at a given pose.

    Pure geometry computation — no file I/O. The hull dict is obtained from
    load_hull() and can be reused across many calls.

    Args:
        hull: Hull data from load_hull()
        z_displacement: Vertical displacement of the boat in mm (negative = sink)
        pitch_deg: Pitch angle in degrees (positive = bow up)
        roll_deg: Roll angle in degrees (positive = starboard down)
        water_level_z: Z coordinate of the water surface (default: 0)

    Returns:
        Dictionary with:
        - CoB: {"x", "y", "z"} center of buoyancy in mm (world frame)
        - submerged_volume_mm3: Total submerged volume in mm³
        - submerged_volume_liters: Total submerged volume in liters
        - buoyancy_force_N: Buoyancy force in Newtons (saltwater)
        - displacement_kg: Water displaced in kg (saltwater)
        - pose: The input pose parameters
        - hull_refs: per-group body and world reference points
        - total_volumes: per-group total volume in liters
        - components: Per-component breakdown (with "group" field)
    """
    hull_shapes = hull["hull_shapes"]
    rotation_center = hull["rotation_center"]
    group_refs_body = hull["group_refs_body"]
    group_total_volumes_mm3 = hull["group_total_volumes_mm3"]

    if not hull_shapes:
        return {
            "error": "No hull components found",
            "CoB": {"x": 0.0, "y": 0.0, "z": 0.0},
            "submerged_volume_mm3": 0.0,
            "submerged_volume_liters": 0.0,
            "buoyancy_force_N": 0.0,
            "displacement_kg": 0.0,
            "pose": {
                "z_offset_mm": z_displacement,
                "pitch_deg": pitch_deg,
                "roll_deg": roll_deg
            },
            "hull_refs": {},
            "total_volumes": {},
            "components": []
        }

    # Transform each hull shape and compute submerged volume
    total_submerged_volume = 0.0
    wcob_x, wcob_y, wcob_z = 0.0, 0.0, 0.0
    component_results = []

    for hs in hull_shapes:
        # Transform the shape
        transformed = transform_shape(
            hs["shape"],
            z_displacement,
            pitch_deg,
            roll_deg,
            rotation_center
        )

        # Compute submerged portion
        result = compute_submerged_volume(transformed, water_level_z)

        vol = result["volume_mm3"]
        cob = result["CoB"]

        component_results.append({
            "label": hs["label"],
            "pattern": hs["pattern"],
            "group": hs["group"],
            "submerged_volume_mm3": round(vol, 2),
            "submerged_volume_liters": round(vol / 1e6, 4),
            "CoB": {
                "x": round(cob["x"], 2),
                "y": round(cob["y"], 2),
                "z": round(cob["z"], 2)
            }
        })

        # Accumulate for total CoB calculation
        total_submerged_volume += vol
        wcob_x += cob["x"] * vol
        wcob_y += cob["y"] * vol
        wcob_z += cob["z"] * vol

    # Compute combined center of buoyancy
    if total_submerged_volume > 1e-6:
        combined_cob = {
            "x": round(wcob_x / total_submerged_volume, 2),
            "y": round(wcob_y / total_submerged_volume, 2),
            "z": round(wcob_z / total_submerged_volume, 2)
        }
    else:
        combined_cob = {"x": 0.0, "y": 0.0, "z": 0.0}

    # Convert volume to liters (1 liter = 1e6 mm³)
    volume_liters = total_submerged_volume / 1e6

    # Compute buoyancy force
    # F = rho * V * g, where V is in m³
    volume_m3 = total_submerged_volume / 1e9  # mm³ to m³
    displacement_kg = volume_m3 * SALTWATER_DENSITY_KG_M3
    buoyancy_force_N = displacement_kg * GRAVITY_M_S2

    rcx, rcy, rcz = rotation_center

    # Transform per-group reference points to world frame
    hull_refs = {}
    for name, ref_body in group_refs_body.items():
        ref_world = _transform_ref_point(ref_body, z_displacement, pitch_deg, roll_deg, rotation_center)
        hull_refs[name] = {
            "body": {
                "x": round(ref_body["x"], 2),
                "y": round(ref_body["y"], 2),
                "z": round(ref_body["z"], 2)
            },
            "world": ref_world
        }

    # Per-group total volumes in liters
    total_volumes = {
        name: round(vol_mm3 / 1e6, 1)
        for name, vol_mm3 in group_total_volumes_mm3.items()
    }

    return {
        "CoB": combined_cob,
        "submerged_volume_mm3": round(total_submerged_volume, 2),
        "submerged_volume_liters": round(volume_liters, 4),
        "buoyancy_force_N": round(buoyancy_force_N, 2),
        "displacement_kg": round(displacement_kg, 2),
        "pose": {
            "z_offset_mm": z_displacement,
            "pitch_deg": pitch_deg,
            "roll_deg": roll_deg,
            "rotation_center": {
                "x": round(rcx, 2),
                "y": round(rcy, 2),
                "z": round(rcz, 2)
            }
        },
        "hull_refs": hull_refs,
        "total_volumes": total_volumes,
        "components": component_results
    }


# Convenience function for use in iterative solvers
def compute_cob(hull: dict, z: float, pitch: float, roll: float) -> dict:
    """
    Shorthand for compute_center_of_buoyancy with minimal arguments.

    Args:
        hull: Hull data from load_hull()
        z: Vertical displacement in mm
        pitch: Pitch angle in degrees
        roll: Roll angle in degrees

    Returns:
        Same as compute_center_of_buoyancy
    """
    return compute_center_of_buoyancy(
        hull=hull,
        z_displacement=z,
        pitch_deg=pitch,
        roll_deg=roll
    )

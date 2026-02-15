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
    from shipshape.physics.center_of_buoyancy import compute_center_of_buoyancy

    result = compute_center_of_buoyancy(
        fcstd_path="artifact/boat.design.FCStd",
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

import sys
import math

try:
    import FreeCAD as App
    import Part
    from FreeCAD import Base
except ImportError as e:
    print(f"ERROR: {e}", file=sys.stderr)
    print("This module requires FreeCAD (conda-forge or bundled)", file=sys.stderr)
    sys.exit(1)


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

DEFAULT_HULL_COMPONENTS = [
    # Ama (outrigger) - listed FIRST so they match before "bottom" pattern
    # (ama parts have "pvc_bottom" in their labels which would falsely match "bottom")
    # More specific patterns (ama_cone_foam) must come before less specific (ama_cone)
    "ama_pipe",         # Ama body (upper and lower) - underscore to match label
    "ama_body_foam",    # Ama body foam core
    "ama_cone_foam",    # Ama cone foam (before ama_cone!)
    "ama_cone",         # Ama cone ends (upper and lower)
    # Vaka (main hull)
    "foam_below_sole",  # Vaka foam core (before sole!)
    "air_inside_vaka",  # Trapped air volume inside cabin
    "bottom",           # Vaka bottom shell
    "sole",             # Vaka sole (cabin floor)
    "hull",             # Vaka hull sides (freeboard, but may be partially submerged)
]


def _get_all_objects(obj_list):
    """Recursively get all objects including those in groups."""
    all_objs = []
    for obj in obj_list:
        all_objs.append(obj)
        if hasattr(obj, 'Group'):
            all_objs.extend(_get_all_objects(obj.Group))
    return all_objs


def _make_rotation_matrix(pitch_deg: float, roll_deg: float, center: Base.Vector) -> Base.Matrix:
    """
    Create a rotation matrix for pitch and roll about a center point.

    Coordinate system (matching CAD model):
        X: transversal (side-to-side)
        Y: longitudinal (fore-aft, bow is positive Y)
        Z: vertical (up is positive)

    Pitch: rotation about the X axis (positive = bow up)
    Roll: rotation about the Y axis (positive = starboard down / port up)

    The rotation is applied about the given center point.
    """
    # Convert to radians
    pitch_rad = math.radians(pitch_deg)
    roll_rad = math.radians(roll_deg)

    # Rotation about X axis (pitch - bow up/down)
    cos_p = math.cos(pitch_rad)
    sin_p = math.sin(pitch_rad)

    # Rotation about Y axis (roll - heel)
    cos_r = math.cos(roll_rad)
    sin_r = math.sin(roll_rad)

    # Combined rotation matrix: R = Ry(roll) * Rx(pitch)
    # This applies pitch first, then roll
    matrix = Base.Matrix(
        cos_r,              sin_r * sin_p,      sin_r * cos_p,      0,
        0,                  cos_p,              -sin_p,             0,
        -sin_r,             cos_r * sin_p,      cos_r * cos_p,      0,
        0,                  0,                  0,                  1
    )

    return matrix


def transform_shape(shape: Part.Shape, z_displacement: float, pitch_deg: float,
                   roll_deg: float, rotation_center: Base.Vector = None) -> Part.Shape:
    """
    Transform a shape by applying z displacement and pitch/roll rotations.

    Args:
        shape: The FreeCAD shape to transform
        z_displacement: Vertical displacement in mm (negative = sink)
        pitch_deg: Pitch angle in degrees (positive = bow up)
        roll_deg: Roll angle in degrees (positive = starboard down)
        rotation_center: Center point for rotation (default: shape's center of mass)

    Returns:
        Transformed copy of the shape
    """
    if rotation_center is None:
        # Use the shape's center of mass as rotation center
        rotation_center = shape.CenterOfGravity

    # Create a copy to avoid modifying the original
    transformed = shape.copy()

    # Apply rotation if any
    if abs(pitch_deg) > 1e-6 or abs(roll_deg) > 1e-6:
        # Translate to origin
        transformed.translate(Base.Vector(-rotation_center.x, -rotation_center.y, -rotation_center.z))

        # Apply rotation
        rot_matrix = _make_rotation_matrix(pitch_deg, roll_deg, Base.Vector(0, 0, 0))
        transformed = transformed.transformGeometry(rot_matrix)

        # Translate back
        transformed.translate(rotation_center)

    # Apply z displacement
    transformed.translate(Base.Vector(0, 0, z_displacement))

    return transformed


def compute_submerged_volume(shape: Part.Shape, water_level_z: float = 0.0) -> dict:
    """
    Compute the submerged portion of a shape below a water plane.

    Args:
        shape: The FreeCAD shape (already transformed to final pose)
        water_level_z: Z coordinate of the water surface (default: 0)

    Returns:
        Dictionary with:
        - submerged_shape: The submerged portion (Part.Shape or None)
        - volume_mm3: Volume in mm³
        - CoB: Center of buoyancy as {"x", "y", "z"} in mm
    """
    # Get bounding box to determine cutting box dimensions
    bbox = shape.BoundBox

    # Check if any part is below water
    if bbox.ZMin >= water_level_z:
        return {
            "submerged_shape": None,
            "volume_mm3": 0.0,
            "CoB": {"x": 0.0, "y": 0.0, "z": 0.0}
        }

    # Create a large box below the water level to use for intersection
    # The box extends from well below the shape to the water surface
    margin = 1000  # mm margin around the shape
    box_min_x = bbox.XMin - margin
    box_min_y = bbox.YMin - margin
    box_min_z = bbox.ZMin - margin

    box_length = bbox.XLength + 2 * margin
    box_width = bbox.YLength + 2 * margin
    box_height = water_level_z - box_min_z  # From bottom to water surface

    if box_height <= 0:
        return {
            "submerged_shape": None,
            "volume_mm3": 0.0,
            "CoB": {"x": 0.0, "y": 0.0, "z": 0.0}
        }

    # Create the underwater cutting box
    water_box = Part.makeBox(box_length, box_width, box_height,
                             Base.Vector(box_min_x, box_min_y, box_min_z))

    # Intersect the shape with the underwater box
    try:
        submerged = shape.common(water_box)
    except Exception as e:
        print(f"Warning: Boolean intersection failed: {e}", file=sys.stderr)
        return {
            "submerged_shape": None,
            "volume_mm3": 0.0,
            "CoB": {"x": 0.0, "y": 0.0, "z": 0.0}
        }

    # Check if we got a valid result
    if submerged.isNull() or submerged.Volume < 1e-6:
        return {
            "submerged_shape": None,
            "volume_mm3": 0.0,
            "CoB": {"x": 0.0, "y": 0.0, "z": 0.0}
        }

    # Get volume and center of mass of submerged portion
    volume_mm3 = submerged.Volume
    cog = submerged.CenterOfGravity

    return {
        "submerged_shape": submerged,
        "volume_mm3": volume_mm3,
        "CoB": {"x": cog.x, "y": cog.y, "z": cog.z}
    }


def compute_center_of_buoyancy(fcstd_path: str, z_displacement: float = 0.0,
                                pitch_deg: float = 0.0, roll_deg: float = 0.0,
                                water_level_z: float = 0.0,
                                hull_components: list = None) -> dict:
    """
    Compute the center of buoyancy for a hull at a given pose.

    This is the main entry point for buoyancy calculations. It:
    1. Loads the FreeCAD document
    2. Collects hull component shapes (vaka, ama, etc.)
    3. Transforms them according to the pose
    4. Computes the submerged volume and its centroid

    Args:
        fcstd_path: Path to the FreeCAD design file
        z_displacement: Vertical displacement of the boat in mm (negative = sink)
        pitch_deg: Pitch angle in degrees (positive = bow up)
        roll_deg: Roll angle in degrees (positive = starboard down)
        water_level_z: Z coordinate of the water surface (default: 0)
        hull_components: List of component name patterns to include.
                        Defaults to DEFAULT_HULL_COMPONENTS (see top of module).
                        Components are matched case-insensitively against object labels.

    Returns:
        Dictionary with:
        - CoB: {"x", "y", "z"} center of buoyancy in mm (world frame)
        - submerged_volume_mm3: Total submerged volume in mm³
        - submerged_volume_liters: Total submerged volume in liters
        - buoyancy_force_N: Buoyancy force in Newtons (saltwater)
        - displacement_kg: Water displaced in kg (saltwater)
        - pose: The input pose parameters
        - components: Per-component breakdown
    """
    if hull_components is None:
        hull_components = DEFAULT_HULL_COMPONENTS

    # Open the FreeCAD document
    doc = App.openDocument(fcstd_path)

    # Collect all objects
    all_objects = _get_all_objects(doc.Objects)

    # Find hull components by matching against labels
    hull_shapes = []
    component_results = []
    processed_labels = set()

    for obj in all_objects:
        if not hasattr(obj, 'Shape') or obj.Shape.isNull():
            continue

        if obj.Label in processed_labels:
            continue

        label_lower = obj.Label.lower()

        # Check if this object matches any hull component pattern
        is_hull = False
        matched_pattern = None
        for pattern in hull_components:
            if pattern.lower() in label_lower:
                is_hull = True
                matched_pattern = pattern
                break

        if not is_hull:
            continue

        processed_labels.add(obj.Label)
        hull_shapes.append({
            "label": obj.Label,
            "shape": obj.Shape,
            "pattern": matched_pattern
        })

    if not hull_shapes:
        App.closeDocument(doc.Name)
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
            "components": []
        }

    # Compute hull reference points (body frame) from original geometry
    ama_z_min = None
    vaka_x_sum, vaka_y_sum, vaka_z_min, vaka_count = 0.0, 0.0, None, 0

    for hs in hull_shapes:
        bbox = hs["shape"].BoundBox
        cog = hs["shape"].CenterOfGravity
        is_ama = hs["pattern"].startswith("ama")

        if is_ama:
            if "001" not in hs["label"]:
                if ama_z_min is None or bbox.ZMin < ama_z_min:
                    ama_z_min = bbox.ZMin
        else:
            vaka_x_sum += cog.x
            vaka_y_sum += cog.y
            vaka_count += 1
            if vaka_z_min is None or bbox.ZMin < vaka_z_min:
                vaka_z_min = bbox.ZMin

    # Hull reference points in body frame (before transformation)
    ama_ref_body = {
        "x": 0.0,
        "y": 0.0,
        "z": ama_z_min if ama_z_min is not None else 0.0
    }
    vaka_ref_body = {
        "x": vaka_x_sum / vaka_count if vaka_count > 0 else 0.0,
        "y": vaka_y_sum / vaka_count if vaka_count > 0 else 0.0,
        "z": vaka_z_min if vaka_z_min is not None else 0.0
    }

    # Compute the combined center of mass for rotation center
    total_volume = 0.0
    weighted_center = Base.Vector(0, 0, 0)
    for hs in hull_shapes:
        vol = hs["shape"].Volume
        cog = hs["shape"].CenterOfGravity
        total_volume += vol
        weighted_center += Base.Vector(cog.x * vol, cog.y * vol, cog.z * vol)

    if total_volume > 0:
        rotation_center = Base.Vector(
            weighted_center.x / total_volume,
            weighted_center.y / total_volume,
            weighted_center.z / total_volume
        )
    else:
        rotation_center = Base.Vector(0, 0, 0)

    # Compute total volumes per hull type (before transformation)
    total_ama_volume_mm3 = 0.0
    total_vaka_volume_mm3 = 0.0
    for hs in hull_shapes:
        vol = hs["shape"].Volume
        if hs["pattern"].startswith("ama"):
            total_ama_volume_mm3 += vol
        else:
            total_vaka_volume_mm3 += vol

    # Transform each hull shape and compute submerged volume
    total_submerged_volume = 0.0
    weighted_cob = Base.Vector(0, 0, 0)

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
        weighted_cob += Base.Vector(
            cob["x"] * vol,
            cob["y"] * vol,
            cob["z"] * vol
        )

    # Compute combined center of buoyancy
    if total_submerged_volume > 1e-6:
        combined_cob = {
            "x": round(weighted_cob.x / total_submerged_volume, 2),
            "y": round(weighted_cob.y / total_submerged_volume, 2),
            "z": round(weighted_cob.z / total_submerged_volume, 2)
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

    App.closeDocument(doc.Name)

    # Transform hull reference points to world frame
    def transform_ref_point(ref_body, z_disp, pitch, roll, rot_center):
        """Transform a reference point from body to world frame."""
        # Translate to rotation center
        x = ref_body["x"] - rot_center.x
        y = ref_body["y"] - rot_center.y
        z = ref_body["z"] - rot_center.z

        # Apply rotation (R = Ry(roll) * Rx(pitch))
        pitch_rad = math.radians(pitch)
        roll_rad = math.radians(roll)
        cos_p, sin_p = math.cos(pitch_rad), math.sin(pitch_rad)
        cos_r, sin_r = math.cos(roll_rad), math.sin(roll_rad)

        x_new = cos_r * x + sin_r * sin_p * y + sin_r * cos_p * z
        y_new = cos_p * y - sin_p * z
        z_new = -sin_r * x + cos_r * sin_p * y + cos_r * cos_p * z

        # Translate back and apply z displacement
        return {
            "x": round(x_new + rot_center.x, 2),
            "y": round(y_new + rot_center.y, 2),
            "z": round(z_new + rot_center.z + z_disp, 2)
        }

    ama_ref_world = transform_ref_point(ama_ref_body, z_displacement, pitch_deg, roll_deg, rotation_center)
    vaka_ref_world = transform_ref_point(vaka_ref_body, z_displacement, pitch_deg, roll_deg, rotation_center)

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
                "x": round(rotation_center.x, 2),
                "y": round(rotation_center.y, 2),
                "z": round(rotation_center.z, 2)
            }
        },
        "hull_refs": {
            "ama_body": {
                "x": round(ama_ref_body["x"], 2),
                "y": round(ama_ref_body["y"], 2),
                "z": round(ama_ref_body["z"], 2)
            },
            "ama_world": ama_ref_world,
            "vaka_body": {
                "x": round(vaka_ref_body["x"], 2),
                "y": round(vaka_ref_body["y"], 2),
                "z": round(vaka_ref_body["z"], 2)
            },
            "vaka_world": vaka_ref_world
        },
        "total_volumes": {
            "ama_liters": round(total_ama_volume_mm3 / 1e6, 1),
            "vaka_liters": round(total_vaka_volume_mm3 / 1e6, 1)
        },
        "components": component_results
    }


# Convenience function for use in iterative solvers
def compute_cob(fcstd_path: str, z: float, pitch: float, roll: float) -> dict:
    """
    Shorthand for compute_center_of_buoyancy with minimal arguments.

    Args:
        fcstd_path: Path to FreeCAD design file
        z: Vertical displacement in mm
        pitch: Pitch angle in degrees
        roll: Roll angle in degrees

    Returns:
        Same as compute_center_of_buoyancy
    """
    return compute_center_of_buoyancy(
        fcstd_path=fcstd_path,
        z_displacement=z,
        pitch_deg=pitch,
        roll_deg=roll
    )

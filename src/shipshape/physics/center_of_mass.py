#!/usr/bin/env python3
"""
Center of mass (Center of Gravity) computation for hydrostatic analysis.

This module provides functions to compute the center of gravity (CoG) of a
boat from its component masses and positions.

There are two approaches:
1. From FreeCAD geometry + materials: Compute CoG directly from shapes
2. From mass artifact: Load precomputed mass data (faster for iteration)

Usage:
    from shipshape.physics.center_of_mass import compute_center_of_gravity

    # From FreeCAD file
    result = compute_center_of_gravity(
        fcstd_path="artifact/boat.design.FCStd",
        materials_path="constant/material/proa.json"
    )

    # From mass artifact (faster)
    result = compute_cog_from_mass_artifact(
        mass_artifact_path="artifact/boat.mass.json",
        fcstd_path="artifact/boat.design.FCStd"  # needed for positions
    )
"""

import sys
import json

try:
    import FreeCAD as App
    import Part
    from FreeCAD import Base
except ImportError as e:
    print(f"ERROR: {e}", file=sys.stderr)
    print("This module requires FreeCAD (conda-forge or bundled)", file=sys.stderr)
    sys.exit(1)


# Physical constants
GRAVITY_M_S2 = 9.81  # m/s²


def _get_all_objects(obj_list):
    """Recursively get all objects including those in groups."""
    all_objs = []
    for obj in obj_list:
        all_objs.append(obj)
        if hasattr(obj, 'Group'):
            all_objs.extend(_get_all_objects(obj.Group))
    return all_objs


def _extract_material_from_label(label: str) -> str:
    """
    Extract material name from object label.

    Expected format: ComponentName__material_name or ComponentName__material_name_001
    """
    label_lower = label.lower()
    if '__' in label_lower:
        parts = label_lower.split('__')
        if len(parts) >= 2:
            return parts[1].rstrip('_0123456789').strip()
    return None


def _get_global_cog(obj) -> Base.Vector:
    """
    Get the center of gravity in world (global) coordinates.

    There are two cases in the design:
    1. Shape created at origin, then positioned via Placement (e.g., Mast in Rig group)
       - Shape bbox center is near origin in X and Y
       - We need to apply Placement to get world coords

    2. Shape created at final world position (e.g., Hull, Ama parts)
       - Shape bbox is already at final position (far from origin)
       - Placement should NOT be applied (shape CoG is already in world coords)

    We detect case 2 by checking if the shape bbox center is far from origin.
    """
    local_cog = obj.Shape.CenterOfGravity
    bbox = obj.Shape.BoundBox

    # Check if shape is already at world position (bbox center far from origin in X or Y)
    bbox_center_x = (bbox.XMin + bbox.XMax) / 2
    bbox_center_y = (bbox.YMin + bbox.YMax) / 2
    shape_at_origin = abs(bbox_center_x) < 500 and abs(bbox_center_y) < 500

    if shape_at_origin:
        # Shape is at origin - apply placement to get world coords
        if hasattr(obj, 'getGlobalPlacement'):
            global_placement = obj.getGlobalPlacement()
        else:
            global_placement = obj.Placement if hasattr(obj, 'Placement') else App.Placement()

        world_cog = global_placement.multVec(local_cog)
        return world_cog
    else:
        # Shape already at world position - use CoG directly
        return local_cog


def compute_center_of_gravity(fcstd_path: str, materials_path: str) -> dict:
    """
    Compute the center of gravity from FreeCAD geometry and materials.

    This calculates CoG by computing the mass-weighted centroid of all
    components, where mass is determined from volume x density.

    Args:
        fcstd_path: Path to the FreeCAD design file
        materials_path: Path to the materials JSON file

    Returns:
        Dictionary with:
        - CoG: {"x", "y", "z"} center of gravity in mm
        - total_mass_kg: Total mass in kg
        - weight_N: Weight force in Newtons
        - components: Per-component breakdown with positions
    """
    # Load materials
    with open(materials_path, 'r') as f:
        materials_data = json.load(f)
    materials = materials_data["materials"]

    # Open FreeCAD document
    doc = App.openDocument(fcstd_path)
    all_objects = _get_all_objects(doc.Objects)

    # Compute mass and CoG for each component
    total_mass = 0.0
    weighted_position = Base.Vector(0, 0, 0)
    component_results = []
    processed_labels = set()

    for obj in all_objects:
        if not hasattr(obj, 'Shape') or obj.Shape.isNull():
            continue

        if obj.Label in processed_labels:
            continue

        # Get material from label
        mat_key = _extract_material_from_label(obj.Label)
        if not mat_key or mat_key not in materials:
            continue

        mat = materials[mat_key]
        volume_m3 = obj.Shape.Volume / 1e9  # mm³ to m³
        mass_kg = volume_m3 * mat['density_kg_m3']

        # Get center of mass in world coordinates
        cog = _get_global_cog(obj)

        component_results.append({
            "label": obj.Label,
            "material": mat['name'],
            "mass_kg": round(mass_kg, 4),
            "volume_liters": round(volume_m3 * 1000, 4),
            "CoG": {
                "x": round(cog.x, 2),
                "y": round(cog.y, 2),
                "z": round(cog.z, 2)
            }
        })

        # Accumulate for total CoG
        total_mass += mass_kg
        weighted_position += Base.Vector(
            cog.x * mass_kg,
            cog.y * mass_kg,
            cog.z * mass_kg
        )

        processed_labels.add(obj.Label)

    App.closeDocument(doc.Name)

    # Compute combined center of gravity
    if total_mass > 1e-6:
        combined_cog = {
            "x": round(weighted_position.x / total_mass, 2),
            "y": round(weighted_position.y / total_mass, 2),
            "z": round(weighted_position.z / total_mass, 2)
        }
    else:
        combined_cog = {"x": 0.0, "y": 0.0, "z": 0.0}

    weight_N = total_mass * GRAVITY_M_S2

    return {
        "CoG": combined_cog,
        "total_mass_kg": round(total_mass, 4),
        "weight_N": round(weight_N, 2),
        "component_count": len(component_results),
        "components": sorted(component_results, key=lambda x: x['mass_kg'], reverse=True)
    }


def compute_cog_from_mass_artifact(mass_artifact_path: str, fcstd_path: str) -> dict:
    """
    Compute CoG using precomputed mass data and geometry positions.

    This is faster for iterative calculations because the mass data is
    already computed. We just need to look up component positions.

    Args:
        mass_artifact_path: Path to the mass.json artifact
        fcstd_path: Path to the FreeCAD design file (for positions)

    Returns:
        Same structure as compute_center_of_gravity
    """
    # Load mass artifact
    with open(mass_artifact_path, 'r') as f:
        mass_data = json.load(f)

    # Create a map from label to mass
    component_masses = {c['name']: c['mass_kg'] for c in mass_data['components']}

    # Open FreeCAD document to get positions
    doc = App.openDocument(fcstd_path)
    all_objects = _get_all_objects(doc.Objects)

    # Compute CoG using masses from artifact and positions from geometry
    total_mass = 0.0
    weighted_position = Base.Vector(0, 0, 0)
    component_results = []
    processed_labels = set()

    for obj in all_objects:
        if not hasattr(obj, 'Shape') or obj.Shape.isNull():
            continue

        if obj.Label in processed_labels:
            continue

        if obj.Label not in component_masses:
            continue

        mass_kg = component_masses[obj.Label]
        cog = _get_global_cog(obj)

        component_results.append({
            "label": obj.Label,
            "mass_kg": round(mass_kg, 4),
            "CoG": {
                "x": round(cog.x, 2),
                "y": round(cog.y, 2),
                "z": round(cog.z, 2)
            }
        })

        total_mass += mass_kg
        weighted_position += Base.Vector(
            cog.x * mass_kg,
            cog.y * mass_kg,
            cog.z * mass_kg
        )

        processed_labels.add(obj.Label)

    App.closeDocument(doc.Name)

    if total_mass > 1e-6:
        combined_cog = {
            "x": round(weighted_position.x / total_mass, 2),
            "y": round(weighted_position.y / total_mass, 2),
            "z": round(weighted_position.z / total_mass, 2)
        }
    else:
        combined_cog = {"x": 0.0, "y": 0.0, "z": 0.0}

    weight_N = total_mass * GRAVITY_M_S2

    return {
        "CoG": combined_cog,
        "total_mass_kg": round(total_mass, 4),
        "weight_N": round(weight_N, 2),
        "component_count": len(component_results),
        "components": sorted(component_results, key=lambda x: x['mass_kg'], reverse=True)
    }


# Convenience function for use in iterative solvers
def compute_cog(fcstd_path: str, materials_path: str) -> dict:
    """
    Shorthand for compute_center_of_gravity.

    Args:
        fcstd_path: Path to FreeCAD design file
        materials_path: Path to materials JSON file

    Returns:
        Same as compute_center_of_gravity
    """
    return compute_center_of_gravity(fcstd_path, materials_path)

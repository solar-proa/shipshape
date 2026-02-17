#!/usr/bin/env python3
"""
Center of mass (Center of Gravity) computation for hydrostatic analysis.

This module provides functions to compute the center of gravity (CoG) of a
boat from its component masses and positions.

There are two approaches:
1. From CAD geometry + materials: Compute CoG directly from shapes
2. From mass artifact: Load precomputed mass data (faster for iteration)

Usage:
    from shipshape.physics.center_of_mass import compute_center_of_gravity

    # From CAD file
    import json
    with open("constant/material/proa.json") as f:
        materials = json.load(f)
    result = compute_center_of_gravity(
        fcstd_path="artifact/boat.design.FCStd",
        materials=materials
    )

    # From mass artifact (faster)
    with open("artifact/boat.mass.json") as f:
        mass_data = json.load(f)
    result = compute_cog_from_mass_artifact(
        mass_data=mass_data,
        fcstd_path="artifact/boat.design.FCStd"  # needed for positions
    )
"""

from .geometry import get_backend, get_reader


# Physical constants
GRAVITY_M_S2 = 9.81  # m/s²


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


def compute_center_of_gravity(fcstd_path: str, materials: dict) -> dict:
    """
    Compute the center of gravity from CAD geometry and materials.

    This calculates CoG by computing the mass-weighted centroid of all
    components, where mass is determined from volume x density.

    Args:
        fcstd_path: Path to the CAD design file
        materials: The materials dict (full JSON object with a "materials" key)

    Returns:
        Dictionary with:
        - CoG: {"x", "y", "z"} center of gravity in mm
        - total_mass_kg: Total mass in kg
        - weight_N: Weight force in Newtons
        - components: Per-component breakdown with positions
    """
    geo = get_backend()
    reader = get_reader()
    materials = materials["materials"]

    doc = reader.open(fcstd_path)
    all_objects = reader.get_objects(doc)

    total_mass = 0.0
    wp_x, wp_y, wp_z = 0.0, 0.0, 0.0
    component_results = []
    processed_labels = set()

    for obj in all_objects:
        if not obj["has_shape"] or obj["global_cog"] is None:
            continue

        if obj["label"] in processed_labels:
            continue

        mat_key = _extract_material_from_label(obj["label"])
        if not mat_key or mat_key not in materials:
            continue

        mat = materials[mat_key]
        volume_m3 = geo.volume(obj["shape"]) / 1e9  # mm³ to m³
        mass_kg = volume_m3 * mat['density_kg_m3']

        cog_x, cog_y, cog_z = obj["global_cog"]

        component_results.append({
            "label": obj["label"],
            "material": mat['name'],
            "mass_kg": round(mass_kg, 4),
            "volume_liters": round(volume_m3 * 1000, 4),
            "CoG": {
                "x": round(cog_x, 2),
                "y": round(cog_y, 2),
                "z": round(cog_z, 2)
            }
        })

        total_mass += mass_kg
        wp_x += cog_x * mass_kg
        wp_y += cog_y * mass_kg
        wp_z += cog_z * mass_kg

        processed_labels.add(obj["label"])

    reader.close(doc)

    if total_mass > 1e-6:
        combined_cog = {
            "x": round(wp_x / total_mass, 2),
            "y": round(wp_y / total_mass, 2),
            "z": round(wp_z / total_mass, 2)
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


def compute_cog_from_mass_artifact(mass_data: dict, fcstd_path: str) -> dict:
    """
    Compute CoG using precomputed mass data and geometry positions.

    This is faster for iterative calculations because the mass data is
    already computed. We just need to look up component positions.

    Args:
        mass_data: The already-loaded mass artifact dict
        fcstd_path: Path to the CAD design file (for positions)

    Returns:
        Same structure as compute_center_of_gravity
    """
    reader = get_reader()

    component_masses = {c['name']: c['mass_kg'] for c in mass_data['components']}

    doc = reader.open(fcstd_path)
    all_objects = reader.get_objects(doc)

    total_mass = 0.0
    wp_x, wp_y, wp_z = 0.0, 0.0, 0.0
    component_results = []
    processed_labels = set()

    for obj in all_objects:
        if not obj["has_shape"] or obj["global_cog"] is None:
            continue

        if obj["label"] in processed_labels:
            continue

        if obj["label"] not in component_masses:
            continue

        mass_kg = component_masses[obj["label"]]
        cog_x, cog_y, cog_z = obj["global_cog"]

        component_results.append({
            "label": obj["label"],
            "mass_kg": round(mass_kg, 4),
            "CoG": {
                "x": round(cog_x, 2),
                "y": round(cog_y, 2),
                "z": round(cog_z, 2)
            }
        })

        total_mass += mass_kg
        wp_x += cog_x * mass_kg
        wp_y += cog_y * mass_kg
        wp_z += cog_z * mass_kg

        processed_labels.add(obj["label"])

    reader.close(doc)

    if total_mass > 1e-6:
        combined_cog = {
            "x": round(wp_x / total_mass, 2),
            "y": round(wp_y / total_mass, 2),
            "z": round(wp_z / total_mass, 2)
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
def compute_cog(fcstd_path: str, materials: dict) -> dict:
    """
    Shorthand for compute_center_of_gravity.

    Args:
        fcstd_path: Path to CAD design file
        materials: The materials dict (full JSON object with a "materials" key)

    Returns:
        Same as compute_center_of_gravity
    """
    return compute_center_of_gravity(fcstd_path, materials)

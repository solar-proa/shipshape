#!/usr/bin/env python3
"""
Mass analysis - computes mass, volume, and component breakdown from CAD model.
"""

from shipshape.physics.geometry import get_backend, get_reader


def analyze_mass(fcstd_path: str, materials: dict) -> dict:
    """
    Analyze mass properties of a CAD model.

    Args:
        fcstd_path: Path to the CAD design file
        materials: The materials dict (full JSON object with a "materials" key)

    Returns:
        Dictionary with mass analysis results
    """
    geo = get_backend()
    reader = get_reader()

    doc = reader.open(fcstd_path)

    materials = materials["materials"]

    material_weights = {}
    material_volumes = {}
    processed_labels = set()
    all_components = []

    all_objects = reader.get_objects(doc)

    for obj in all_objects:
        if not obj["has_shape"]:
            continue

        if obj["label"] in processed_labels:
            continue

        # Look for material name in label (format: ComponentName__material_name)
        label = obj["label"]
        label_lower = label.lower()
        mat_key = None

        if '__' in label_lower:
            parts = label_lower.split('__')
            if len(parts) >= 2:
                mat_key = parts[1].rstrip('_0123456789').strip()

        if mat_key and mat_key in materials:
            mat = materials[mat_key]
            volume_m3 = geo.volume(obj["shape"]) / 1e9
            volume_liters = volume_m3 * 1000
            weight_kg = volume_m3 * mat['density_kg_m3']

            # Track by material
            if mat_key not in material_weights:
                material_weights[mat_key] = 0
                material_volumes[mat_key] = 0
            material_weights[mat_key] += weight_kg
            material_volumes[mat_key] += volume_liters

            # Track individual component
            all_components.append({
                'name': obj["label"],
                'mass_kg': round(weight_kg, 2),
                'volume_liters': round(volume_liters, 2),
                'material': mat['name']
            })

            processed_labels.add(obj["label"])

    total_mass = sum(material_weights.values())
    total_volume = sum(material_volumes.values())

    # Escapable volume: materials flagged buoyancy_only (e.g. trapped air)
    # contribute to buoyancy but escape when the hull is breached,
    # so they don't count toward unsinkable volume.
    escapable_volume = sum(
        material_volumes.get(mat_key, 0)
        for mat_key, mat in materials.items()
        if mat.get('buoyancy_only', False)
    )
    total_unsinkable_volume = total_volume - escapable_volume

    # Build result
    result = {
        'validator': 'mass',
        'total_mass_kg': round(total_mass, 2),
        'total_volume_liters': round(total_volume, 2),
        'total_unsinkable_volume_liters':
             round(total_unsinkable_volume, 2),
        'total_unsinkable_displacement_saltwater_kg':
             round(total_unsinkable_volume * 1.025, 2),
        'materials': {},
        'components': sorted(all_components, key=lambda x: x['mass_kg'], reverse=True),
        'component_count': len(all_components)
    }

    # Add material breakdown
    for mat_name in sorted(material_weights.keys()):
        result['materials'][mat_name] = {
            'mass_kg': round(material_weights[mat_name], 2),
            'volume_liters': round(material_volumes[mat_name], 2)
        }

    reader.close(doc)
    return result

#!/usr/bin/env python3
"""
Mass analysis - computes mass, volume, and component breakdown from FreeCAD model.
"""

import sys

try:
    import FreeCAD as App
except ImportError as e:
    print(f"ERROR: {e}", file=sys.stderr)
    print("This module requires FreeCAD (conda-forge or bundled)", file=sys.stderr)
    sys.exit(1)


def analyze_mass(fcstd_path: str, materials: dict) -> dict:
    """
    Analyze mass properties of a FreeCAD model.

    Args:
        fcstd_path: Path to the FreeCAD design file
        materials: The materials dict (full JSON object with a "materials" key)

    Returns:
        Dictionary with mass analysis results
    """
    doc = App.openDocument(fcstd_path)

    materials = materials["materials"]

    material_weights = {}
    material_volumes = {}
    processed_labels = set()
    all_components = []

    def get_all_objects(obj_list):
        """Recursively get all objects including those in groups"""
        all_objs = []
        for obj in obj_list:
            all_objs.append(obj)
            if hasattr(obj, 'Group'):
                all_objs.extend(get_all_objects(obj.Group))
        return all_objs

    all_objects = get_all_objects(doc.Objects)

    for obj in all_objects:
        if not hasattr(obj, 'Shape'):
            continue

        if obj.Label in processed_labels:
            continue

        # Look for material name in label (format: ComponentName__material_name)
        label = obj.Label
        label_lower = label.lower()
        mat_key = None

        if '__' in label_lower:
            parts = label_lower.split('__')
            if len(parts) >= 2:
                mat_key = parts[1].rstrip('_0123456789').strip()

        if mat_key and mat_key in materials:
            mat = materials[mat_key]
            volume_m3 = obj.Shape.Volume / 1e9
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
                'name': obj.Label,
                'mass_kg': round(weight_kg, 2),
                'volume_liters': round(volume_liters, 2),
                'material': mat['name']
            })

            processed_labels.add(obj.Label)

    total_mass = sum(material_weights.values())
    total_volume = sum(material_volumes.values())
    total_volume_air = material_volumes['air']
    total_unsinkable_volume = total_volume - total_volume_air

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

    App.closeDocument(doc.Name)
    return result

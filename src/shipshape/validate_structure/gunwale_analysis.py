"""
Gunwale load analysis - how the gunwales carry and distribute aka loads.

The gunwales are load-bearing structural members that:
1. Support the akas where they attach to the vaka
2. Distribute point loads from akas into the hull structure
3. Transfer loads via the fiberglass bond to the hull skin

Gunwale section: 3" wide x 2" high (76.2 x 50.8 mm), glassed to hull.

Analysis models:
1. Beam on elastic foundation - gunwale distributing load along its length
2. Local bearing - aka pressing on gunwale surface
3. Bond shear - fiberglass joint between gunwale and hull
"""

import math
from typing import Dict, Any

from .beam_mechanics import GRAVITY


# Material properties
WOOD_E_MPA = 12000  # Douglas fir/similar, parallel to grain
WOOD_BENDING_STRENGTH_MPA = 50  # Allowable bending stress
WOOD_BEARING_STRENGTH_MPA = 10  # Perpendicular to grain bearing
WOOD_SHEAR_STRENGTH_MPA = 8  # Shear parallel to grain

# Fiberglass bond properties
GLASS_BOND_SHEAR_MPA = 5  # Conservative allowable for glass-to-wood bond
GLASS_LAMINATE_THICKNESS = 3  # mm, typical layup


def get_gunwale_properties(params: Dict[str, Any]) -> Dict[str, Any]:
    """
    Get gunwale section properties.

    Default: 3" x 2" (76.2 x 50.8 mm) solid wood section.
    """
    # Gunwale dimensions (can be overridden in params)
    width = params.get('gunwale_width', 76.2)  # mm (3 inches)
    height = params.get('gunwale_height', 50.8)  # mm (2 inches)

    # Section properties (solid rectangular section)
    area = width * height
    # Moment of inertia about horizontal axis (resists vertical bending)
    Ix = width * height**3 / 12
    # Section modulus
    Sx = width * height**2 / 6

    return {
        'width_mm': width,
        'height_mm': height,
        'area_mm2': area,
        'Ix_mm4': Ix,
        'Sx_mm3': Sx,
        'material': 'wood (Douglas fir or similar)',
        'E_mpa': WOOD_E_MPA,
    }


def get_aka_loads_at_gunwale(params: Dict[str, Any],
                              mass_data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Calculate the loads that akas apply to the gunwales.

    Load cases:
    1. Suspended ama - ama weight pulls down on akas, akas pull up on gunwale
    2. Wave slam - upward force on ama pushes akas down on gunwale
    3. Lifting - crane lift puts compression into akas at gunwale

    The critical load is typically wave slam (highest magnitude).
    """
    # Get outrigger mass
    components = mass_data.get('components', [])
    outrigger_mass = 0.0
    for comp in components:
        name = comp['name']
        mass = comp['mass_kg']
        if any(x in name for x in ['Ama_', 'Solar_', 'Pillar_', 'Spine_',
                                    'Panel_', 'Cross_Brace', 'Stringer']):
            outrigger_mass += mass
        elif 'Aka' in name:
            outrigger_mass += mass * 0.5  # Half of aka mass

    num_akas = params.get('num_akas', 4)

    # Load case 1: Suspended ama (ama out of water)
    # Each aka carries 1/num_akas of the outrigger weight
    suspended_load_per_aka = outrigger_mass * GRAVITY / num_akas

    # Load case 2: Wave slam (from wave_slam analysis)
    # Use the slam force calculation
    ama_length = params.get('ama_length', 9300)
    ama_diameter = params.get('ama_diameter', 430)
    immersion_fraction = 0.3
    effective_area = ama_length * ama_diameter * immersion_fraction / 1e6  # m²

    impact_velocity = 3.0  # m/s
    dynamic_factor = 2.5
    rho_water = 1025  # kg/m³
    Cp = 1.0  # pressure coefficient

    slam_pressure = 0.5 * rho_water * impact_velocity**2 * Cp / 1000  # kPa
    slam_force = slam_pressure * 1000 * effective_area * dynamic_factor  # N
    slam_load_per_aka = slam_force / num_akas

    # The aka applies this load at the gunwale attachment point
    # For the vaka side, it's the reaction force at the inboard support

    return {
        'outrigger_mass_kg': round(outrigger_mass, 1),
        'num_akas': num_akas,
        'suspended_load_per_aka_n': round(suspended_load_per_aka, 0),
        'wave_slam_load_per_aka_n': round(slam_load_per_aka, 0),
        'design_load_per_aka_n': round(max(suspended_load_per_aka, slam_load_per_aka), 0),
        'governing_case': 'wave_slam' if slam_load_per_aka > suspended_load_per_aka else 'suspended_ama',
    }


def calculate_distribution_length(gunwale_props: Dict[str, Any],
                                   hull_stiffness_n_per_mm2: float = 0.5) -> Dict[str, Any]:
    """
    Calculate the characteristic length over which load distributes.

    Model: Beam on elastic foundation (Winkler foundation).
    The hull skin acts as an elastic support under the gunwale.

    Characteristic length λ = (4*E*I / k)^0.25
    where k = foundation stiffness per unit length (N/mm per mm)

    The load spreads over approximately π*λ on each side of the load point.
    """
    E = gunwale_props['E_mpa']  # N/mm²
    I = gunwale_props['Ix_mm4']  # mm⁴
    width = gunwale_props['width_mm']

    # Foundation stiffness k = hull stiffness × gunwale width
    # Hull stiffness is approximately E_hull * t_hull / some_factor
    # Use a conservative estimate based on typical hull construction
    k_per_mm = hull_stiffness_n_per_mm2 * width  # N/mm per mm of length

    # Characteristic length
    lambda_mm = (4 * E * I / k_per_mm) ** 0.25

    # Effective distribution length (load spreads over ~π*λ each side)
    # Total effective length ≈ 2.5 * λ for practical purposes
    effective_length = 2.5 * lambda_mm

    return {
        'characteristic_length_mm': round(lambda_mm, 0),
        'effective_distribution_length_mm': round(effective_length, 0),
        'foundation_stiffness_n_per_mm2': hull_stiffness_n_per_mm2,
    }


def check_gunwale_bending(gunwale_props: Dict[str, Any],
                           load_n: float,
                           distribution_length_mm: float,
                           min_safety_factor: float) -> Dict[str, Any]:
    """
    Check gunwale bending stress at aka attachment.

    Model the gunwale as a beam on elastic foundation with point load.
    Max moment occurs at the load point: M_max = P * λ / 4

    For a beam on elastic foundation with point load P:
    M_max = P / (4 * β) where β = (k / 4EI)^0.25 = 1/λ
    So M_max = P * λ / 4
    """
    E = gunwale_props['E_mpa']
    I = gunwale_props['Ix_mm4']
    S = gunwale_props['Sx_mm3']
    width = gunwale_props['width_mm']

    # Use the characteristic length from distribution calculation
    # λ = effective_length / 2.5
    lambda_mm = distribution_length_mm / 2.5

    # Maximum moment at load point
    M_max_nmm = load_n * lambda_mm / 4
    M_max_nm = M_max_nmm / 1000

    # Bending stress
    stress_mpa = M_max_nmm / S

    # Safety factor
    safety_factor = WOOD_BENDING_STRENGTH_MPA / stress_mpa if stress_mpa > 0 else float('inf')
    passed = safety_factor >= min_safety_factor

    return {
        'component': 'gunwale_bending',
        'model': 'beam on elastic foundation',
        'load_n': round(load_n, 0),
        'characteristic_length_mm': round(lambda_mm, 0),
        'max_moment_nm': round(M_max_nm, 1),
        'section_modulus_mm3': round(S, 0),
        'bending_stress_mpa': round(stress_mpa, 2),
        'allowable_stress_mpa': WOOD_BENDING_STRENGTH_MPA,
        'safety_factor': round(safety_factor, 2),
        'passed': passed,
    }


def check_bearing_stress(gunwale_props: Dict[str, Any],
                          aka_props: Dict[str, Any],
                          load_n: float,
                          min_safety_factor: float) -> Dict[str, Any]:
    """
    Check bearing stress where aka contacts gunwale.

    The aka (aluminum RHS) sits on or passes through the gunwale.
    The contact area determines the bearing stress on the wood.
    """
    # Aka dimensions at the bearing point
    aka_width = aka_props['width_mm']
    aka_height = aka_props['height_mm']
    aka_thickness = aka_props['thickness_mm']

    gunwale_width = gunwale_props['width_mm']

    # Bearing area: aka bottom face on gunwale
    # Effective bearing width is the lesser of aka width and gunwale width
    bearing_width = min(aka_width, gunwale_width)
    # Bearing length along gunwale is the aka "footprint"
    # For RHS sitting on gunwale, this is the aka height (perpendicular to gunwale run)
    bearing_length = aka_height

    bearing_area = bearing_width * bearing_length

    # Bearing stress (perpendicular to grain)
    bearing_stress = load_n / bearing_area

    # Safety factor (use bearing strength perpendicular to grain)
    safety_factor = WOOD_BEARING_STRENGTH_MPA / bearing_stress if bearing_stress > 0 else float('inf')
    passed = safety_factor >= min_safety_factor

    return {
        'component': 'bearing_stress',
        'load_n': round(load_n, 0),
        'bearing_width_mm': round(bearing_width, 1),
        'bearing_length_mm': round(bearing_length, 1),
        'bearing_area_mm2': round(bearing_area, 0),
        'bearing_stress_mpa': round(bearing_stress, 2),
        'allowable_stress_mpa': WOOD_BEARING_STRENGTH_MPA,
        'safety_factor': round(safety_factor, 2),
        'passed': passed,
        'note': 'Wood bearing perpendicular to grain',
    }


def check_bond_shear(gunwale_props: Dict[str, Any],
                      load_n: float,
                      distribution_length_mm: float,
                      min_safety_factor: float) -> Dict[str, Any]:
    """
    Check shear stress in the fiberglass bond between gunwale and hull.

    The load transfers from gunwale to hull via shear in the glass laminate.
    The shear area is the bond width times the distribution length.

    For beam on elastic foundation, the shear distribution is not uniform.
    Peak shear occurs at the load point and decays exponentially.
    Use a conservative effective length of λ (not the full distribution length).
    """
    gunwale_width = gunwale_props['width_mm']

    # Characteristic length
    lambda_mm = distribution_length_mm / 2.5

    # Bond area (both sides of gunwale if glassed on both sides)
    # Assume glass on bottom and possibly one side
    bond_perimeter = gunwale_width + 2 * GLASS_LAMINATE_THICKNESS  # Bottom contact width
    # Effective shear length - use λ as conservative estimate
    effective_shear_length = lambda_mm

    bond_area = bond_perimeter * effective_shear_length

    # Average shear stress over the effective length
    # Peak is higher, but we use average with conservative length
    shear_stress = load_n / bond_area

    # Safety factor
    safety_factor = GLASS_BOND_SHEAR_MPA / shear_stress if shear_stress > 0 else float('inf')
    passed = safety_factor >= min_safety_factor

    return {
        'component': 'bond_shear',
        'model': 'fiberglass bond to hull',
        'load_n': round(load_n, 0),
        'bond_perimeter_mm': round(bond_perimeter, 1),
        'effective_shear_length_mm': round(effective_shear_length, 0),
        'bond_area_mm2': round(bond_area, 0),
        'shear_stress_mpa': round(shear_stress, 2),
        'allowable_stress_mpa': GLASS_BOND_SHEAR_MPA,
        'safety_factor': round(safety_factor, 2),
        'passed': passed,
        'note': 'Glass-to-wood bond, conservative estimate',
    }


def check_aka_spacing(params: Dict[str, Any],
                       distribution_length_mm: float) -> Dict[str, Any]:
    """
    Check if aka spacing is adequate for load distribution.

    If akas are too close together, their load distributions overlap,
    which could overstress the gunwale between them.

    Ideally, aka spacing > 2 × distribution length for independent behavior.
    """
    num_akas = params.get('num_akas', 4)
    spine_length = params.get('panel_stringer_length', 6264)

    if num_akas > 1:
        aka_spacing = spine_length / (num_akas - 1)
    else:
        aka_spacing = spine_length

    # Check ratio
    spacing_ratio = aka_spacing / distribution_length_mm

    # Independent if ratio > 2, overlapping if < 1.5
    if spacing_ratio > 2.0:
        interaction = 'independent'
        note = 'Loads distribute independently, no interaction'
    elif spacing_ratio > 1.5:
        interaction = 'slight_overlap'
        note = 'Slight overlap, acceptable'
    else:
        interaction = 'significant_overlap'
        note = 'Load distributions overlap - consider reinforcement between akas'

    return {
        'aka_spacing_mm': round(aka_spacing, 0),
        'distribution_length_mm': round(distribution_length_mm, 0),
        'spacing_ratio': round(spacing_ratio, 2),
        'interaction': interaction,
        'note': note,
    }


def validate_gunwale_loads(params: Dict[str, Any],
                            mass_data: Dict[str, Any],
                            min_safety_factor: float = 2.0) -> Dict[str, Any]:
    """
    Validate gunwale structural capacity under aka loads.

    Checks:
    1. Gunwale bending at aka attachment
    2. Bearing stress from aka on gunwale
    3. Bond shear (fiberglass to hull)
    4. Aka spacing vs distribution length

    Args:
        params: Design parameters
        mass_data: Mass calculation results
        min_safety_factor: Minimum required safety factor

    Returns:
        Validation results
    """
    # Gunwale properties
    gunwale = get_gunwale_properties(params)

    # Aka properties for bearing calculation
    aka = {
        'width_mm': params['aka_width'],
        'height_mm': params['aka_height'],
        'thickness_mm': params['aka_thickness'],
    }

    # Loads at gunwale
    loads = get_aka_loads_at_gunwale(params, mass_data)
    design_load = loads['design_load_per_aka_n']

    # Distribution length
    distribution = calculate_distribution_length(gunwale)
    dist_length = distribution['effective_distribution_length_mm']

    # Run checks
    bending_result = check_gunwale_bending(gunwale, design_load, dist_length, min_safety_factor)
    bearing_result = check_bearing_stress(gunwale, aka, design_load, min_safety_factor)
    bond_result = check_bond_shear(gunwale, design_load, dist_length, min_safety_factor)
    spacing_result = check_aka_spacing(params, dist_length)

    # Overall pass (spacing is informational, not pass/fail)
    all_passed = (bending_result['passed'] and
                  bearing_result['passed'] and
                  bond_result['passed'])

    # Minimum safety factor
    min_sf = min(
        bending_result['safety_factor'],
        bearing_result['safety_factor'],
        bond_result['safety_factor']
    )

    # Governing component
    sf_list = [
        ('gunwale_bending', bending_result['safety_factor']),
        ('bearing', bearing_result['safety_factor']),
        ('bond_shear', bond_result['safety_factor']),
    ]
    governing = min(sf_list, key=lambda x: x[1])

    return {
        'test_name': 'gunwale_loads',
        'description': 'Gunwale load distribution from aka attachments',
        'passed': all_passed,
        'min_safety_factor_required': min_safety_factor,
        'gunwale_section': {
            'width_mm': gunwale['width_mm'],
            'height_mm': gunwale['height_mm'],
            'material': gunwale['material'],
        },
        'loading': loads,
        'distribution': distribution,
        'checks': {
            'bending': bending_result,
            'bearing': bearing_result,
            'bond_shear': bond_result,
        },
        'aka_spacing': spacing_result,
        'summary': {
            'design_load_n': round(design_load, 0),
            'distribution_length_mm': round(dist_length, 0),
            'min_safety_factor': round(min_sf, 2),
            'governing_component': governing[0],
            'result': 'PASS' if all_passed else 'FAIL',
        }
    }

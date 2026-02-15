"""
Diagonal brace analysis for lateral loading.

The pillar diagonal braces connect the ama structure to the akas.
They resist lateral forces when the boat is tilted.

Two load cases:
1. Compression: Boat on its side, ama horizontal above vaka
   - Ama weight pushes toward vaka, braces in compression
   - Critical check: buckling

2. Tension: Boat inverted, ama dangling below vaka
   - Ama weight pulls away from vaka, braces in tension
   - Critical check: yielding
"""

import math
from typing import Dict, Any

from .beam_mechanics import (
    ALUMINUM_YIELD_STRENGTH_MPA, ALUMINUM_E_MPA, GRAVITY,
    calculate_shs_section_properties
)
from .aka_analysis import extract_outrigger_mass


def calculate_brace_geometry(params: Dict[str, Any]) -> Dict[str, Any]:
    """
    Calculate diagonal brace geometry.

    The braces are at 45° from vertical, connecting pillars to aka level.

    Returns:
        Dictionary with brace geometry
    """
    # Brace cross-section (SHS)
    width = params['stringer_width']  # 25.4 mm
    thickness = params['stringer_thickness']  # 3 mm

    # Vertical offset from aka base to lower brace attachment
    vertical_offset = params['pillar_brace_vertical_offset']  # 500 mm for RP2

    # Spine width (horizontal component at 45°)
    spine_width = params['spine_width']  # 76.2 mm

    # Brace length (as calculated in mirror.py)
    brace_length = math.sqrt(2) * (vertical_offset + spine_width)

    # The brace is rotated 45° around Y axis
    # This means it goes diagonally in the X-Z plane
    # Vertical component = brace_length * sin(45°)
    # Horizontal component = brace_length * cos(45°)
    vertical_rise = brace_length * math.sin(math.radians(45))
    horizontal_run = brace_length * math.cos(math.radians(45))

    # Angle from horizontal (for force resolution)
    angle_from_horizontal_deg = 45.0

    # Number of braces
    num_akas = params.get('num_akas', 4)
    braces_per_aka = 2  # Kuning and Biru sides
    total_braces = num_akas * braces_per_aka

    return {
        'width_mm': width,
        'thickness_mm': thickness,
        'length_mm': brace_length,
        'vertical_rise_mm': vertical_rise,
        'horizontal_run_mm': horizontal_run,
        'angle_from_horizontal_deg': angle_from_horizontal_deg,
        'num_akas': num_akas,
        'braces_per_aka': braces_per_aka,
        'total_braces': total_braces,
    }


def calculate_lateral_force(mass_data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Calculate lateral force from outrigger weight.

    When boat is on its side, the outrigger weight acts as lateral force.
    Uses the same mass extraction as the suspended ama test for consistency.

    Returns:
        Dictionary with force data
    """
    # Get outrigger mass using the same function as aka_analysis
    tip_mass, distributed_mass, _ = extract_outrigger_mass(mass_data)
    outrigger_mass = tip_mass + distributed_mass

    # Total lateral force = weight
    lateral_force_n = outrigger_mass * GRAVITY

    return {
        'outrigger_mass_kg': outrigger_mass,
        'lateral_force_n': lateral_force_n,
    }


def check_compression(brace_geometry: Dict[str, Any],
                      section_props: Dict[str, Any],
                      lateral_force_n: float,
                      min_safety_factor: float) -> Dict[str, Any]:
    """
    Check braces in compression (boat on side, ama above).

    All braces share the lateral load. Each brace experiences
    axial compression = (lateral_force / num_braces) / cos(angle).

    Critical failure mode is Euler buckling.

    Returns:
        Analysis results
    """
    num_braces = brace_geometry['total_braces']
    angle_rad = math.radians(brace_geometry['angle_from_horizontal_deg'])

    # Force per brace (horizontal component)
    horizontal_force_per_brace = lateral_force_n / num_braces

    # Axial force in brace (along brace axis)
    # F_axial = F_horizontal / cos(angle)
    axial_force_n = horizontal_force_per_brace / math.cos(angle_rad)

    # Compressive stress
    area_mm2 = section_props['area_mm2']
    compressive_stress_mpa = axial_force_n / area_mm2

    # Euler buckling stress
    # For pinned-pinned column: P_cr = pi² * E * I / L²
    # sigma_cr = P_cr / A = pi² * E * I / (A * L²)
    I_mm4 = section_props['Ix_mm4']  # Use minimum I for buckling
    L_mm = brace_geometry['length_mm']

    # Effective length factor K = 1.0 for pinned-pinned
    # Could be lower if ends are partially fixed
    K = 1.0
    Le = K * L_mm

    P_euler_n = (math.pi ** 2) * ALUMINUM_E_MPA * I_mm4 / (Le ** 2)
    euler_buckling_stress_mpa = P_euler_n / area_mm2

    # Slenderness ratio
    r_mm = math.sqrt(I_mm4 / area_mm2)  # radius of gyration
    slenderness = Le / r_mm

    # Safety factor against buckling
    sf_buckling = P_euler_n / axial_force_n if axial_force_n > 0 else float('inf')

    # Safety factor against yielding (less critical for slender members)
    sf_yielding = ALUMINUM_YIELD_STRENGTH_MPA / compressive_stress_mpa if compressive_stress_mpa > 0 else float('inf')

    # Governing safety factor is the minimum
    safety_factor = min(sf_buckling, sf_yielding)
    passed = safety_factor >= min_safety_factor

    return {
        'load_case': 'compression',
        'description': 'Boat on side, ama above vaka',
        'force_per_brace_n': round(axial_force_n, 1),
        'compressive_stress_mpa': round(compressive_stress_mpa, 2),
        'euler_buckling_stress_mpa': round(euler_buckling_stress_mpa, 1),
        'euler_buckling_load_n': round(P_euler_n, 1),
        'slenderness_ratio': round(slenderness, 1),
        'safety_factor_buckling': round(sf_buckling, 2),
        'safety_factor_yielding': round(sf_yielding, 2),
        'safety_factor': round(safety_factor, 2),
        'governing_mode': 'buckling' if sf_buckling < sf_yielding else 'yielding',
        'passed': passed,
    }


def check_tension(brace_geometry: Dict[str, Any],
                  section_props: Dict[str, Any],
                  lateral_force_n: float,
                  min_safety_factor: float) -> Dict[str, Any]:
    """
    Check braces in tension (boat inverted, ama dangling below).

    All braces share the lateral load. Each brace experiences
    axial tension = (lateral_force / num_braces) / cos(angle).

    Critical failure mode is yielding.

    Returns:
        Analysis results
    """
    num_braces = brace_geometry['total_braces']
    angle_rad = math.radians(brace_geometry['angle_from_horizontal_deg'])

    # Force per brace (horizontal component)
    horizontal_force_per_brace = lateral_force_n / num_braces

    # Axial force in brace (along brace axis)
    axial_force_n = horizontal_force_per_brace / math.cos(angle_rad)

    # Tensile stress
    area_mm2 = section_props['area_mm2']
    tensile_stress_mpa = axial_force_n / area_mm2

    # Safety factor against yielding
    safety_factor = ALUMINUM_YIELD_STRENGTH_MPA / tensile_stress_mpa if tensile_stress_mpa > 0 else float('inf')
    passed = safety_factor >= min_safety_factor

    return {
        'load_case': 'tension',
        'description': 'Boat inverted, ama dangling below vaka',
        'force_per_brace_n': round(axial_force_n, 1),
        'tensile_stress_mpa': round(tensile_stress_mpa, 2),
        'yield_strength_mpa': ALUMINUM_YIELD_STRENGTH_MPA,
        'safety_factor': round(safety_factor, 2),
        'passed': passed,
    }


def validate_diagonal_braces(params: Dict[str, Any],
                             mass_data: Dict[str, Any],
                             min_safety_factor: float = 2.0) -> Dict[str, Any]:
    """
    Validate diagonal brace structural integrity under lateral loading.

    Checks both compression (boat on side) and tension (boat inverted) cases.

    Args:
        params: Design parameters
        mass_data: Mass calculation results
        min_safety_factor: Minimum required safety factor

    Returns:
        Validation results
    """
    # Brace geometry
    brace_geom = calculate_brace_geometry(params)

    # Section properties
    section_props = calculate_shs_section_properties(
        brace_geom['width_mm'],
        brace_geom['thickness_mm']
    )

    # Lateral force from outrigger weight
    force_data = calculate_lateral_force(mass_data)

    # Check both load cases
    compression_result = check_compression(
        brace_geom, section_props,
        force_data['lateral_force_n'],
        min_safety_factor
    )

    tension_result = check_tension(
        brace_geom, section_props,
        force_data['lateral_force_n'],
        min_safety_factor
    )

    # Overall pass requires both cases to pass
    all_passed = compression_result['passed'] and tension_result['passed']

    # Minimum safety factor across both cases
    min_sf = min(compression_result['safety_factor'], tension_result['safety_factor'])

    return {
        'test_name': 'diagonal_braces',
        'description': 'Pillar diagonal braces under lateral loading (boat tilted)',
        'passed': all_passed,
        'min_safety_factor_required': min_safety_factor,
        'brace_geometry': {
            'section': f"SHS {brace_geom['width_mm']}x{brace_geom['thickness_mm']} mm",
            'length_mm': round(brace_geom['length_mm'], 1),
            'angle_deg': brace_geom['angle_from_horizontal_deg'],
            'num_braces': brace_geom['total_braces'],
        },
        'section_properties': {
            'area_mm2': round(section_props['area_mm2'], 1),
            'Ix_mm4': round(section_props['Ix_mm4'], 0),
            'Sx_mm3': round(section_props['Sx_mm3'], 1),
        },
        'loading': {
            'outrigger_mass_kg': round(force_data['outrigger_mass_kg'], 1),
            'lateral_force_n': round(force_data['lateral_force_n'], 1),
        },
        'compression_check': compression_result,
        'tension_check': tension_result,
        'summary': {
            'min_safety_factor': round(min_sf, 2),
            'governing_case': 'compression' if compression_result['safety_factor'] < tension_result['safety_factor'] else 'tension',
            'result': 'PASS' if all_passed else 'FAIL',
        }
    }

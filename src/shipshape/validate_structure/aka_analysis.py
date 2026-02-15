"""
Aka (cross-beam) structural analysis for suspended ama load case.

This module validates that the akas can support static loads when the
outrigger (ama) is completely suspended with no buoyancy support.
"""

from typing import Dict, Any, List, Tuple

from .beam_mechanics import (
    ALUMINUM_YIELD_STRENGTH_MPA, ALUMINUM_E_MPA, GRAVITY,
    calculate_rhs_section_properties
)


def extract_outrigger_mass(mass_data: Dict[str, Any]) -> Tuple[float, float, List[Dict]]:
    """
    Extract mass of outrigger-side components from mass data.

    Components are categorized by their position along the aka:
    - TIP loads: at the ama end (full moment arm)
    - DISTRIBUTED loads: span along the aka (effective moment arm ≈ L/2)

    Returns:
        Tuple of (tip_mass_kg, distributed_mass_kg, list of component details)
    """
    components = mass_data.get('components', [])

    tip_patterns = [
        ('Ama_pipe', 'pvc'),
        ('Ama_Cone', 'pvc'),
        ('Ama_Body_Foam', 'foam'),
        ('Ama_Cone_Foam', 'foam'),
        ('Pillar_', 'aluminum_tip'),
        ('Pillar_Brace', 'aluminum_tip'),
    ]

    distributed_patterns = [
        ('Panel_', 'solar'),
        ('Stringer_a_', 'aluminum_distributed'),
        ('Stringer_b_', 'aluminum_distributed'),
        ('Cross_Brace', 'aluminum_distributed'),
    ]

    outrigger_components = []
    tip_mass = 0.0
    distributed_mass = 0.0

    for comp in components:
        name = comp['name']

        matched = False
        for pattern, category in tip_patterns:
            if pattern in name:
                outrigger_components.append({
                    'name': name,
                    'mass_kg': comp['mass_kg'],
                    'category': category,
                    'location': 'tip'
                })
                tip_mass += comp['mass_kg']
                matched = True
                break

        if matched:
            continue

        for pattern, category in distributed_patterns:
            if pattern in name:
                outrigger_components.append({
                    'name': name,
                    'mass_kg': comp['mass_kg'],
                    'category': category,
                    'location': 'distributed'
                })
                distributed_mass += comp['mass_kg']
                break

    return tip_mass, distributed_mass, outrigger_components


def analyze_aka_cantilever(params: Dict[str, Any],
                           tip_mass_kg: float,
                           distributed_mass_kg: float,
                           num_akas: int,
                           bending_axis: str = 'strong') -> Dict[str, Any]:
    """
    Analyze aka as a cantilever beam supporting suspended outrigger load.

    Args:
        params: Design parameters
        tip_mass_kg: Mass at ama end
        distributed_mass_kg: Mass distributed along aka
        num_akas: Number of akas sharing the load
        bending_axis: 'strong' or 'weak'

    Returns:
        Analysis results including stress, deflection, and safety factor
    """
    aka_width = params['aka_width']
    aka_height = params['aka_height']
    aka_thickness = params['aka_thickness']
    aka_length = params['aka_length']
    vaka_width = params['vaka_width']

    cantilever_length = aka_length - vaka_width
    section = calculate_rhs_section_properties(aka_width, aka_height, aka_thickness)

    if bending_axis == 'strong':
        S_mm3 = section['Sx_mm3']
        I_mm4 = section['Ix_mm4']
        axis_description = "strong axis (height=101.6mm resists bending)"
    else:
        S_mm3 = section['Sy_mm3']
        I_mm4 = section['Iy_mm4']
        axis_description = "weak axis (width=76.2mm resists bending)"

    total_mass_kg = tip_mass_kg + distributed_mass_kg

    tip_force_n = (tip_mass_kg * GRAVITY) / num_akas
    distributed_force_n = (distributed_mass_kg * GRAVITY) / num_akas
    total_force_per_aka_n = tip_force_n + distributed_force_n

    L_mm = cantilever_length
    M_tip_nmm = tip_force_n * L_mm
    M_distributed_nmm = distributed_force_n * (L_mm / 2)
    M_max_nmm = M_tip_nmm + M_distributed_nmm

    sigma_max_mpa = M_max_nmm / S_mm3

    E_mpa = ALUMINUM_E_MPA
    delta_tip_mm = (tip_force_n * L_mm**3) / (3 * E_mpa * I_mm4)
    delta_distributed_mm = (distributed_force_n * L_mm**3) / (8 * E_mpa * I_mm4)
    delta_max_mm = delta_tip_mm + delta_distributed_mm

    safety_factor = ALUMINUM_YIELD_STRENGTH_MPA / sigma_max_mpa

    return {
        'bending_axis': bending_axis,
        'axis_description': axis_description,
        'section_properties': section,
        'active_section_modulus_mm3': S_mm3,
        'active_moment_of_inertia_mm4': I_mm4,
        'load': {
            'tip_mass_kg': tip_mass_kg,
            'distributed_mass_kg': distributed_mass_kg,
            'total_mass_kg': total_mass_kg,
            'num_akas': num_akas,
            'tip_force_per_aka_n': tip_force_n,
            'distributed_force_per_aka_n': distributed_force_n,
            'total_force_per_aka_n': total_force_per_aka_n
        },
        'geometry': {
            'aka_length_mm': aka_length,
            'aka_length_m': aka_length / 1000,
            'vaka_width_mm': vaka_width,
            'cantilever_length_mm': L_mm,
            'cantilever_length_m': L_mm / 1000
        },
        'moment_breakdown': {
            'M_tip_nmm': M_tip_nmm,
            'M_distributed_nmm': M_distributed_nmm,
            'M_total_nmm': M_max_nmm,
            'tip_moment_arm_mm': L_mm,
            'distributed_moment_arm_mm': L_mm / 2
        },
        'results': {
            'max_bending_moment_nmm': M_max_nmm,
            'max_bending_moment_nm': M_max_nmm / 1000,
            'max_bending_stress_mpa': sigma_max_mpa,
            'max_deflection_mm': delta_max_mm,
            'yield_strength_mpa': ALUMINUM_YIELD_STRENGTH_MPA,
            'safety_factor': safety_factor
        }
    }


def validate_suspended_ama(params: Dict[str, Any],
                           mass_data: Dict[str, Any],
                           min_safety_factor: float = 2.0) -> Dict[str, Any]:
    """
    Validate aka structural integrity for suspended ama load case.

    Args:
        params: Design parameters
        mass_data: Mass calculation results
        min_safety_factor: Minimum required safety factor

    Returns:
        Validation results
    """
    tip_mass, distributed_mass, outrigger_components = extract_outrigger_mass(mass_data)
    total_outrigger_mass = tip_mass + distributed_mass

    panels_per_half = params['panels_longitudinal'] // 2
    akas_per_panel = params.get('akas_per_panel', 1)
    num_akas = 2 * panels_per_half * akas_per_panel

    analysis_strong = analyze_aka_cantilever(params, tip_mass, distributed_mass, num_akas, 'strong')
    analysis_weak = analyze_aka_cantilever(params, tip_mass, distributed_mass, num_akas, 'weak')

    # Only strong axis required for pass/fail
    # Weak axis is informational only - not a realistic load case
    sf_strong = analysis_strong['results']['safety_factor']
    sf_weak = analysis_weak['results']['safety_factor']
    passed_strong = sf_strong >= min_safety_factor
    passed = passed_strong

    mass_by_category = {}
    mass_by_location = {'tip': 0.0, 'distributed': 0.0}
    for comp in outrigger_components:
        cat = comp['category']
        loc = comp['location']
        if cat not in mass_by_category:
            mass_by_category[cat] = 0.0
        mass_by_category[cat] += comp['mass_kg']
        mass_by_location[loc] += comp['mass_kg']

    return {
        'test_name': 'suspended_ama',
        'description': 'Aka bending: ama suspended with no buoyancy support',
        'passed': passed,
        'min_safety_factor_required': min_safety_factor,
        'outrigger_mass': {
            'total_kg': total_outrigger_mass,
            'tip_kg': tip_mass,
            'distributed_kg': distributed_mass,
            'by_category': mass_by_category,
            'by_location': mass_by_location,
            'component_count': len(outrigger_components)
        },
        'analysis': {
            'strong_axis': analysis_strong,
            'weak_axis': analysis_weak
        },
        'summary': {
            'aka_dimensions': f"{params['aka_width']}x{params['aka_height']}x{params['aka_thickness']} mm RHS",
            'aka_length_m': params['aka_length'] / 1000,
            'vaka_width_m': params['vaka_width'] / 1000,
            'cantilever_length_m': analysis_strong['geometry']['cantilever_length_m'],
            'num_akas_total': num_akas,
            'outrigger_mass_kg': round(total_outrigger_mass, 2),
            'tip_mass_kg': round(tip_mass, 2),
            'distributed_mass_kg': round(distributed_mass, 2),
            'strong_axis': {
                'section_modulus_mm3': round(analysis_strong['active_section_modulus_mm3'], 0),
                'max_stress_mpa': round(analysis_strong['results']['max_bending_stress_mpa'], 2),
                'safety_factor': round(sf_strong, 2),
                'max_deflection_mm': round(analysis_strong['results']['max_deflection_mm'], 1),
                'result': 'PASS' if passed_strong else 'FAIL'
            },
            'weak_axis': {
                'section_modulus_mm3': round(analysis_weak['active_section_modulus_mm3'], 0),
                'max_stress_mpa': round(analysis_weak['results']['max_bending_stress_mpa'], 2),
                'safety_factor': round(sf_weak, 2),
                'max_deflection_mm': round(analysis_weak['results']['max_deflection_mm'], 1),
                'note': 'informational only - not used for pass/fail'
            },
            'yield_strength_mpa': ALUMINUM_YIELD_STRENGTH_MPA,
        }
    }

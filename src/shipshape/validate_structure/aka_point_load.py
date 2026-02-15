"""
Aka point load analysis - crew standing on aka during boarding.

Models the aka as a simply supported beam with a point load at center.
This is the worst-case position for bending moment.

Supports:
- Vaka end: where aka attaches to the main hull
- Pillar end: where pillar supports the aka at the ama

The strong axis (height) resists vertical loads from crew standing on top.
"""

import math
from typing import Dict, Any

from .beam_mechanics import (
    ALUMINUM_YIELD_STRENGTH_MPA, ALUMINUM_E_MPA, GRAVITY,
    calculate_rhs_section_properties
)


def calculate_aka_span(params: Dict[str, Any]) -> Dict[str, float]:
    """
    Calculate the simply-supported span of the aka.

    The aka is supported at the vaka (inboard) and at the pillar (outboard).
    The pillar is located at the ama, roughly at the end of the cantilever.

    Returns:
        Dictionary with span geometry
    """
    aka_length = params['aka_length']
    vaka_width = params['vaka_width']

    # Support at vaka: approximately at the gunwale (half vaka width from center)
    vaka_support_from_center = vaka_width / 2

    # Support at pillar: at the ama end
    # The pillar is located at the aka position on the ama
    # For simplicity, assume pillar support is at the aka tip
    pillar_support_from_center = aka_length

    # Span between supports
    span = pillar_support_from_center - vaka_support_from_center

    return {
        'aka_length_mm': aka_length,
        'vaka_support_mm': vaka_support_from_center,
        'pillar_support_mm': pillar_support_from_center,
        'span_mm': span,
    }


def simply_supported_point_load_center(P_n: float, L_mm: float,
                                        E_mpa: float, I_mm4: float,
                                        S_mm3: float) -> Dict[str, float]:
    """
    Analyze simply supported beam with point load at center.

    For point load P at center of span L:
    - Max moment M = P * L / 4 (at center)
    - Max deflection δ = P * L³ / (48 * E * I) (at center)
    - Reactions R = P / 2 (at each support)

    Args:
        P_n: Point load in Newtons
        L_mm: Span length in mm
        E_mpa: Young's modulus in MPa
        I_mm4: Moment of inertia in mm⁴
        S_mm3: Section modulus in mm³

    Returns:
        Analysis results
    """
    # Reactions at supports
    R = P_n / 2

    # Maximum moment at center
    M_max = P_n * L_mm / 4

    # Maximum stress
    sigma_max = M_max / S_mm3

    # Maximum deflection at center
    delta_max = (P_n * L_mm**3) / (48 * E_mpa * I_mm4)

    return {
        'reaction_n': R,
        'max_moment_nmm': M_max,
        'max_moment_nm': M_max / 1000,
        'max_stress_mpa': sigma_max,
        'max_deflection_mm': delta_max,
    }


def validate_aka_point_load(params: Dict[str, Any],
                            crew_mass_kg: float = 150.0,
                            min_safety_factor: float = 2.0) -> Dict[str, Any]:
    """
    Validate aka under point load from crew standing at center.

    This simulates crew boarding or working on the aka.
    The worst case is a point load at the center of the span.

    Args:
        params: Design parameters
        crew_mass_kg: Mass of crew standing on aka (default 150 kg = 2 people)
        min_safety_factor: Minimum required safety factor

    Returns:
        Validation results
    """
    # Aka section properties (strong axis resists vertical load)
    aka_width = params['aka_width']
    aka_height = params['aka_height']
    aka_thickness = params['aka_thickness']

    section_props = calculate_rhs_section_properties(
        aka_width, aka_height, aka_thickness
    )

    # Span geometry
    span_geom = calculate_aka_span(params)
    span_mm = span_geom['span_mm']

    # Point load from crew
    P_n = crew_mass_kg * GRAVITY

    # Analyze as simply supported beam with center load
    # Strong axis (Ix, Sx) resists vertical bending
    analysis = simply_supported_point_load_center(
        P_n, span_mm,
        ALUMINUM_E_MPA,
        section_props['Ix_mm4'],
        section_props['Sx_mm3']
    )

    # Safety factor
    safety_factor = ALUMINUM_YIELD_STRENGTH_MPA / analysis['max_stress_mpa']
    passed = safety_factor >= min_safety_factor

    return {
        'test_name': 'aka_point_load',
        'description': f'Crew ({crew_mass_kg:.0f} kg) standing on aka at center',
        'passed': passed,
        'min_safety_factor_required': min_safety_factor,
        'crew_mass_kg': crew_mass_kg,
        'aka_section': {
            'dimensions': f"{aka_width}x{aka_height}x{aka_thickness} mm RHS",
            'Ix_mm4': round(section_props['Ix_mm4'], 0),
            'Sx_mm3': round(section_props['Sx_mm3'], 1),
        },
        'geometry': {
            'aka_length_mm': span_geom['aka_length_mm'],
            'span_mm': round(span_mm, 0),
            'load_position': 'center of span',
        },
        'loading': {
            'crew_mass_kg': crew_mass_kg,
            'point_load_n': round(P_n, 1),
        },
        'analysis': {
            'support_reaction_n': round(analysis['reaction_n'], 1),
            'max_moment_nm': round(analysis['max_moment_nm'], 1),
            'max_stress_mpa': round(analysis['max_stress_mpa'], 2),
            'max_deflection_mm': round(analysis['max_deflection_mm'], 1),
            'yield_strength_mpa': ALUMINUM_YIELD_STRENGTH_MPA,
        },
        'summary': {
            'max_stress_mpa': round(analysis['max_stress_mpa'], 2),
            'safety_factor': round(safety_factor, 2),
            'max_deflection_mm': round(analysis['max_deflection_mm'], 1),
            'result': 'PASS' if passed else 'FAIL',
        }
    }

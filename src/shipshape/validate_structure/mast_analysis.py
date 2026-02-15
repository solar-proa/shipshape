"""
Mast structural analysis for wind loading.

Models the mast as a propped cantilever:
- Fixed support at mast step (sole)
- Lateral (pinned) support at mast partner (gunwale)
- Wind load applied at sail center of effort

Checks:
1. Bending stress at mast partner level
2. Shear force at partner (transferred to gunwale)
3. Combined compression + bending below partner
4. Buckling of mast section
"""

import math
from typing import Dict, Any

from .beam_mechanics import (
    ALUMINUM_YIELD_STRENGTH_MPA, ALUMINUM_E_MPA, GRAVITY,
    calculate_pipe_section_properties
)

# Air properties
AIR_DENSITY_KG_M3 = 1.225
SAIL_DRAG_COEFFICIENT = 1.15  # Flat plate perpendicular to wind


def knots_to_ms(knots: float) -> float:
    """Convert wind speed from knots to m/s."""
    return knots * 0.514444


def calculate_wind_force(wind_speed_knots: float, sail_area_m2: float) -> float:
    """
    Calculate wind force on sail.

    F = 0.5 * rho * V^2 * Cd * A

    Args:
        wind_speed_knots: Wind speed in knots
        sail_area_m2: Sail area in square meters

    Returns:
        Force in Newtons
    """
    V = knots_to_ms(wind_speed_knots)
    return 0.5 * AIR_DENSITY_KG_M3 * V**2 * SAIL_DRAG_COEFFICIENT * sail_area_m2


def calculate_mast_geometry(params: Dict[str, Any]) -> Dict[str, float]:
    """
    Calculate mast support positions and sail center of effort.

    Returns:
        Dictionary with geometry values in mm
    """
    # Mast step is at mast_base_level (on the sole)
    mast_step_level = params.get('mast_base_level', 212)

    # Mast partner is at aka_base_level (on the gunwale)
    mast_partner_level = params.get('aka_base_level', 1406)

    # Distance from step to partner
    step_to_partner = mast_partner_level - mast_step_level

    # Mast total height
    mast_height = params['mast_height']

    # Sail dimensions
    sail_height = params['sail_height']
    sail_width = params['sail_width']
    sail_area_m2 = (sail_height * sail_width) / 1e6  # Convert mm² to m²

    # Center of effort: assume boom is at partner level,
    # and CE is at 40% of sail height above boom
    ce_above_partner = sail_height * 0.40

    # Height from step to center of effort
    step_to_ce = step_to_partner + ce_above_partner

    # Height from partner to mast top
    partner_to_top = mast_height - step_to_partner

    return {
        'mast_height_mm': mast_height,
        'step_to_partner_mm': step_to_partner,
        'partner_to_top_mm': partner_to_top,
        'ce_above_partner_mm': ce_above_partner,
        'step_to_ce_mm': step_to_ce,
        'sail_height_mm': sail_height,
        'sail_width_mm': sail_width,
        'sail_area_m2': sail_area_m2,
    }


def check_bending_at_partner(wind_force_n: float,
                              ce_above_partner_mm: float,
                              section_props: Dict[str, float]) -> Dict[str, Any]:
    """
    Check bending stress at mast partner level.

    The section above the partner acts as a cantilever.
    M = F * distance_to_CE

    Args:
        wind_force_n: Wind force on sail (N)
        ce_above_partner_mm: Distance from partner to center of effort (mm)
        section_props: Mast section properties

    Returns:
        Analysis results
    """
    M = wind_force_n * ce_above_partner_mm
    S = section_props['S_mm3']
    sigma = M / S

    safety_factor = ALUMINUM_YIELD_STRENGTH_MPA / sigma if sigma > 0 else float('inf')

    return {
        'moment_nmm': M,
        'moment_nm': M / 1000,
        'stress_mpa': sigma,
        'safety_factor': safety_factor,
        'passed': safety_factor >= 2.0
    }


def check_shear_at_partner(wind_force_n: float,
                            section_props: Dict[str, float]) -> Dict[str, Any]:
    """
    Check shear stress at mast partner.

    The full wind force is transferred laterally at the partner.
    For a thin-walled tube: tau_max = 2 * V / A

    Args:
        wind_force_n: Wind force (N)
        section_props: Mast section properties

    Returns:
        Analysis results
    """
    V = wind_force_n
    A = section_props['area_mm2']

    # For thin-walled circular tube, max shear stress ≈ 2V/A
    tau_max = 2 * V / A

    # Shear yield ≈ 0.577 * tensile yield (von Mises)
    shear_yield = 0.577 * ALUMINUM_YIELD_STRENGTH_MPA
    safety_factor = shear_yield / tau_max if tau_max > 0 else float('inf')

    return {
        'shear_force_n': V,
        'shear_stress_mpa': tau_max,
        'shear_yield_mpa': shear_yield,
        'safety_factor': safety_factor,
        'passed': safety_factor >= 2.0
    }


def check_column_below_partner(wind_force_n: float,
                                step_to_partner_mm: float,
                                ce_above_partner_mm: float,
                                mast_mass_kg: float,
                                section_props: Dict[str, float]) -> Dict[str, Any]:
    """
    Check combined compression and bending in mast section below partner.

    The section below the partner experiences:
    - Compression from mast weight above
    - Bending from the reaction moment at the partner

    Uses interaction formula: (sigma_b/sigma_allow) + (sigma_c/sigma_crit) <= 1.0

    Args:
        wind_force_n: Wind force (N)
        step_to_partner_mm: Distance from step to partner (mm)
        ce_above_partner_mm: Distance from partner to CE (mm)
        mast_mass_kg: Total mast mass (kg)
        section_props: Mast section properties

    Returns:
        Analysis results
    """
    # Compressive load (mast weight above partner, roughly 85% of total)
    weight_above = mast_mass_kg * 0.85 * GRAVITY
    A = section_props['area_mm2']
    sigma_c = weight_above / A

    # Bending in the section below partner
    # For propped cantilever with moment M at the prop:
    # The moment is distributed, max at partner = M_partner
    # Below partner, moment reduces linearly to zero at step
    # Max moment below partner ≈ at partner level
    M_at_partner = wind_force_n * ce_above_partner_mm

    # Portion of moment carried below partner depends on fixity
    # For fixed-pinned beam, roughly 50% goes to fixed end
    # Conservative: use full moment at partner
    S = section_props['S_mm3']
    sigma_b = M_at_partner / S

    # Euler buckling stress for column below partner
    # Effective length factor K = 0.7 for fixed-pinned
    I = section_props['I_mm4']
    L = step_to_partner_mm
    K = 0.7
    P_euler = math.pi**2 * ALUMINUM_E_MPA * I / (K * L)**2
    sigma_euler = P_euler / A

    # Interaction check
    interaction = (sigma_b / ALUMINUM_YIELD_STRENGTH_MPA) + (sigma_c / sigma_euler)

    safety_factor = 1.0 / interaction if interaction > 0 else float('inf')

    return {
        'compressive_load_n': weight_above,
        'compressive_stress_mpa': sigma_c,
        'bending_stress_mpa': sigma_b,
        'euler_buckling_stress_mpa': sigma_euler,
        'interaction_ratio': interaction,
        'safety_factor': safety_factor,
        'passed': safety_factor >= 2.0
    }


def check_local_buckling(section_props: Dict[str, float]) -> Dict[str, Any]:
    """
    Check for local wall buckling (crinkling) of thin-walled tube.

    Rule of thumb: D/t < 50 is generally safe for aluminum tubes.
    More precise: sigma_crit = 0.3 * E * t / R

    Args:
        section_props: Mast section properties

    Returns:
        Analysis results
    """
    D = section_props['outer_diameter_mm']
    t = section_props['thickness_mm']
    R = D / 2

    d_over_t = D / t

    # Critical local buckling stress (empirical formula for cylinders)
    sigma_local_crit = 0.3 * ALUMINUM_E_MPA * t / R

    # Compare to yield - local buckling shouldn't govern if D/t < 50
    ratio_to_yield = sigma_local_crit / ALUMINUM_YIELD_STRENGTH_MPA

    passed = d_over_t < 50 and ratio_to_yield > 2.0

    return {
        'd_over_t': d_over_t,
        'critical_stress_mpa': sigma_local_crit,
        'ratio_to_yield': ratio_to_yield,
        'passed': passed,
        'note': 'D/t < 50 recommended' if d_over_t < 50 else 'D/t > 50, local buckling risk'
    }


def validate_mast(params: Dict[str, Any],
                  mass_data: Dict[str, Any],
                  wind_speed_knots: float = 25.0,
                  min_safety_factor: float = 2.0) -> Dict[str, Any]:
    """
    Validate mast structural integrity under wind loading.

    Args:
        params: Design parameters
        mass_data: Mass calculation results
        wind_speed_knots: Design wind speed (default 25 knots)
        min_safety_factor: Minimum required safety factor

    Returns:
        Validation results
    """
    # Mast section properties
    mast_diameter = params['mast_diameter']
    mast_thickness = params['mast_thickness']
    section_props = calculate_pipe_section_properties(mast_diameter, mast_thickness)

    # Geometry
    geometry = calculate_mast_geometry(params)

    # Wind force
    wind_force = calculate_wind_force(wind_speed_knots, geometry['sail_area_m2'])

    # Get mast mass from mass data
    mast_mass = 0.0
    for comp in mass_data.get('components', []):
        if 'Mast__aluminum_' in comp['name'] and 'Handle' not in comp['name']:
            mast_mass += comp['mass_kg']
    # Use half since there are two masts (mirrored)
    mast_mass = mast_mass / 2 if mast_mass > 0 else 50.0  # Default estimate

    # Run checks
    bending_result = check_bending_at_partner(
        wind_force,
        geometry['ce_above_partner_mm'],
        section_props
    )

    shear_result = check_shear_at_partner(
        wind_force,
        section_props
    )

    column_result = check_column_below_partner(
        wind_force,
        geometry['step_to_partner_mm'],
        geometry['ce_above_partner_mm'],
        mast_mass,
        section_props
    )

    local_buckling_result = check_local_buckling(section_props)

    # Overall pass requires all checks to pass
    all_passed = all([
        bending_result['passed'],
        shear_result['passed'],
        column_result['passed'],
        local_buckling_result['passed']
    ])

    # Find minimum safety factor
    min_sf = min(
        bending_result['safety_factor'],
        shear_result['safety_factor'],
        column_result['safety_factor']
    )

    return {
        'test_name': 'mast_wind_loading',
        'description': f'Mast under {wind_speed_knots:.0f} knot wind (sail perpendicular)',
        'passed': all_passed,
        'min_safety_factor_required': min_safety_factor,
        'wind_speed_knots': wind_speed_knots,
        'wind_force_n': round(wind_force, 1),
        'geometry': {
            'mast_diameter_mm': mast_diameter,
            'mast_thickness_mm': mast_thickness,
            'mast_height_mm': geometry['mast_height_mm'],
            'step_to_partner_mm': round(geometry['step_to_partner_mm'], 0),
            'ce_above_partner_mm': round(geometry['ce_above_partner_mm'], 0),
            'sail_area_m2': round(geometry['sail_area_m2'], 2),
            'd_over_t': round(section_props['outer_diameter_mm'] / section_props['thickness_mm'], 1),
        },
        'section_properties': {
            'area_mm2': round(section_props['area_mm2'], 1),
            'I_mm4': round(section_props['I_mm4'], 0),
            'S_mm3': round(section_props['S_mm3'], 0),
        },
        'checks': {
            'bending_at_partner': {
                'moment_nm': round(bending_result['moment_nm'], 1),
                'stress_mpa': round(bending_result['stress_mpa'], 2),
                'safety_factor': round(bending_result['safety_factor'], 2),
                'result': 'PASS' if bending_result['passed'] else 'FAIL'
            },
            'shear_at_partner': {
                'shear_force_n': round(shear_result['shear_force_n'], 1),
                'shear_stress_mpa': round(shear_result['shear_stress_mpa'], 2),
                'safety_factor': round(shear_result['safety_factor'], 2),
                'result': 'PASS' if shear_result['passed'] else 'FAIL'
            },
            'column_below_partner': {
                'compressive_stress_mpa': round(column_result['compressive_stress_mpa'], 2),
                'bending_stress_mpa': round(column_result['bending_stress_mpa'], 2),
                'interaction_ratio': round(column_result['interaction_ratio'], 3),
                'safety_factor': round(column_result['safety_factor'], 2),
                'result': 'PASS' if column_result['passed'] else 'FAIL'
            },
            'local_buckling': {
                'd_over_t': round(local_buckling_result['d_over_t'], 1),
                'note': local_buckling_result['note'],
                'result': 'PASS' if local_buckling_result['passed'] else 'FAIL'
            }
        },
        'summary': {
            'wind_speed_knots': wind_speed_knots,
            'wind_force_n': round(wind_force, 0),
            'min_safety_factor': round(min_sf, 2),
            'safety_factor': round(min_sf, 2),
            'result': 'PASS' if all_passed else 'FAIL'
        }
    }

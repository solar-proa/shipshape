"""
Wave slam analysis - impact load on ama from wave.

Two scenarios:
1. Vertical slam (wave from below) - resisted by diagonal pillar braces
2. Frontal slam (wave from front) - resisted by X-shaped cross-braces

Uses a dynamic amplification factor to account for impact loading.

Vertical slam load path:
  Wave impact → Ama hull → Spine → Pillars → Diagonal braces → Akas → Vaka

Frontal slam load path:
  Wave impact → Ama front → Pillars → Cross-braces → Adjacent pillars → Spine
"""

import math
from typing import Dict, Any

from .beam_mechanics import (
    ALUMINUM_YIELD_STRENGTH_MPA, ALUMINUM_E_MPA, GRAVITY,
    calculate_rhs_section_properties, calculate_shs_section_properties
)
from .aka_analysis import extract_outrigger_mass

# Water properties
WATER_DENSITY_KG_M3 = 1025  # Seawater


def estimate_wave_slam_force(params: Dict[str, Any],
                              impact_velocity_ms: float = 2.0,
                              dynamic_factor: float = 2.5) -> Dict[str, Any]:
    """
    Estimate wave slam force on ama.

    Uses a simplified slam pressure model:
    P_slam = 0.5 × rho × V² × Cp
    where Cp is a slam coefficient (typically 2-5 for hull sections)

    For a cylindrical ama, we use:
    - Reduced slam coefficient (1.5) due to rounded shape shedding water
    - Effective impact area = 50% of projected area (partial immersion)

    Args:
        params: Design parameters
        impact_velocity_ms: Relative velocity of wave impact (m/s)
        dynamic_factor: Dynamic amplification factor (typically 2-3)

    Returns:
        Dictionary with force estimates
    """
    # Ama dimensions
    ama_length_mm = params.get('ama_length', 9300)
    ama_diameter_mm = params.get('ama_diameter', 315)  # PVC pipe diameter

    # Projected area for slam (bottom of ama)
    # Full projected area
    full_projected_area_m2 = (ama_length_mm / 1000) * (ama_diameter_mm / 1000)

    # Effective area reduction for cylindrical shape (water sheds around sides)
    # Use 50% of projected area
    area_reduction_factor = 0.5
    effective_area_m2 = full_projected_area_m2 * area_reduction_factor

    # Slam pressure coefficient
    # Lower for cylindrical hull (1.5) vs flat bottom (3-5)
    Cp_slam = 1.5

    # Slam pressure: P = 0.5 × rho × V² × Cp
    slam_pressure_pa = 0.5 * WATER_DENSITY_KG_M3 * impact_velocity_ms**2 * Cp_slam

    # Static slam force (before dynamic amplification)
    static_slam_force_n = slam_pressure_pa * effective_area_m2

    # Apply dynamic amplification
    dynamic_slam_force_n = static_slam_force_n * dynamic_factor

    return {
        'ama_length_m': ama_length_mm / 1000,
        'ama_diameter_m': ama_diameter_mm / 1000,
        'full_projected_area_m2': round(full_projected_area_m2, 2),
        'area_reduction_factor': area_reduction_factor,
        'effective_area_m2': round(effective_area_m2, 2),
        'impact_velocity_ms': impact_velocity_ms,
        'slam_coefficient': Cp_slam,
        'slam_pressure_kpa': round(slam_pressure_pa / 1000, 2),
        'static_slam_force_n': round(static_slam_force_n, 0),
        'dynamic_factor': dynamic_factor,
        'dynamic_slam_force_n': round(dynamic_slam_force_n, 0),
    }


def check_aka_via_braces(params: Dict[str, Any],
                         slam_force_n: float,
                         min_safety_factor: float) -> Dict[str, Any]:
    """
    Check aka bending under wave slam, with diagonal braces as elastic supports.

    Load path: Wave → Ama/Spine → Pillars → Diagonal braces ↔ Aka

    The braces provide elastic support to the aka at the attachment point.
    This makes the aka a propped cantilever:
    - Fixed at vaka support
    - Elastic spring support at brace attachment (from brace axial stiffness)

    The load is applied at the brace attachment point (where braces push on aka).
    The brace stiffness determines how much load the braces take vs the vaka.
    """
    # Aka section properties
    aka_width = params['aka_width']
    aka_height = params['aka_height']
    aka_thickness = params['aka_thickness']
    section_props = calculate_rhs_section_properties(aka_width, aka_height, aka_thickness)

    # Brace geometry
    pillar_brace_offset = params['pillar_brace_vertical_offset']
    spine_width = params['spine_width']
    stringer_width = params['stringer_width']
    stringer_thickness = params['stringer_thickness']

    brace_length_mm = math.sqrt(2) * (pillar_brace_offset + spine_width)
    brace_angle_rad = math.radians(45)

    # Brace section properties (for stiffness calculation)
    brace_section = calculate_shs_section_properties(stringer_width, stringer_thickness)
    brace_area_mm2 = brace_section['area_mm2']

    # Brace axial stiffness: k_brace = E * A / L
    brace_axial_stiffness = ALUMINUM_E_MPA * brace_area_mm2 / brace_length_mm  # N/mm

    # Vertical stiffness of brace (component in vertical direction)
    # k_vertical = k_axial * sin²(angle) for vertical displacement
    brace_vertical_stiffness = brace_axial_stiffness * math.sin(brace_angle_rad)**2

    # Total vertical stiffness from both braces per aka
    braces_per_aka = 2
    total_brace_stiffness_per_aka = brace_vertical_stiffness * braces_per_aka

    # Horizontal run of brace (distance from pillar to brace-aka attachment)
    brace_horizontal_run = brace_length_mm * math.cos(brace_angle_rad)

    # Aka geometry
    aka_length = params['aka_length']
    vaka_width = params['vaka_width']
    full_cantilever = aka_length - vaka_width / 2

    # Distance from vaka to brace attachment
    a = full_cantilever - brace_horizontal_run  # brace attachment from vaka

    # Aka bending stiffness
    E = ALUMINUM_E_MPA
    I = section_props['Ix_mm4']

    # Number of akas sharing the slam load
    num_akas = params.get('num_akas', 4)

    # Total vertical force per aka from slam
    total_vertical_per_aka = slam_force_n / num_akas

    # Propped cantilever analysis:
    # Fixed at vaka (x=0), spring support at x=a, load P at x=a
    #
    # The deflection at x=a due to load P at x=a (cantilever):
    # delta_cantilever = P * a³ / (3EI)
    #
    # The spring provides reaction R such that:
    # delta_spring = R / k_spring
    # delta_cantilever - delta_spring = 0 (compatibility)
    #
    # So: P * a³ / (3EI) = R / k_spring
    # R = P * a³ * k_spring / (3EI)
    #
    # But R cannot exceed P (spring can't push more than the load)
    # Fraction taken by spring: f = R/P = a³ * k / (3EI)

    k = total_brace_stiffness_per_aka
    fraction_to_braces = (a**3 * k) / (3 * E * I)

    # Cap at 1.0 (braces can't take more than the full load)
    fraction_to_braces = min(fraction_to_braces, 0.95)

    # Force distribution
    force_to_braces = total_vertical_per_aka * fraction_to_braces
    force_to_vaka = total_vertical_per_aka * (1 - fraction_to_braces)

    # Moment at vaka support
    # From the portion that goes to vaka (acting at brace point): M = F_vaka * a
    # The brace portion creates no moment at vaka (it's reacted by the spring)
    M_vaka_nmm = force_to_vaka * a
    M_vaka_nm = M_vaka_nmm / 1000

    # Bending stress at vaka
    S_mm3 = section_props['Sx_mm3']
    bending_stress_mpa = M_vaka_nmm / S_mm3

    # Axial stress from horizontal brace component
    # The braces also push horizontally (compression toward vaka)
    horizontal_per_aka = force_to_braces / math.tan(brace_angle_rad)  # F_h = F_v / tan(45) = F_v
    area_mm2 = section_props['area_mm2']
    axial_stress_mpa = horizontal_per_aka / area_mm2

    # Combined stress
    combined_stress_mpa = bending_stress_mpa + axial_stress_mpa

    # Safety factor
    safety_factor = ALUMINUM_YIELD_STRENGTH_MPA / combined_stress_mpa if combined_stress_mpa > 0 else float('inf')
    passed = safety_factor >= min_safety_factor

    return {
        'component': 'aka (propped by braces)',
        'model': 'cantilever with elastic spring support from braces',
        'brace_stiffness_n_per_mm': round(total_brace_stiffness_per_aka, 1),
        'brace_attachment_from_vaka_mm': round(a, 0),
        'full_cantilever_mm': round(full_cantilever, 0),
        'fraction_to_braces': round(fraction_to_braces, 3),
        'fraction_to_vaka': round(1 - fraction_to_braces, 3),
        'force_to_braces_n': round(force_to_braces, 0),
        'force_to_vaka_n': round(force_to_vaka, 0),
        'moment_at_vaka_nm': round(M_vaka_nm, 1),
        'bending_stress_mpa': round(bending_stress_mpa, 2),
        'axial_stress_mpa': round(axial_stress_mpa, 2),
        'combined_stress_mpa': round(combined_stress_mpa, 2),
        'yield_strength_mpa': ALUMINUM_YIELD_STRENGTH_MPA,
        'safety_factor': round(safety_factor, 2),
        'passed': passed,
    }


def check_diagonal_braces_slam(params: Dict[str, Any],
                                slam_force_n: float,
                                min_safety_factor: float) -> Dict[str, Any]:
    """
    Check diagonal braces under wave slam compression.

    When wave pushes ama up, the diagonal braces are compressed
    (same direction as the "boat on side" case but higher load).
    """
    # Brace section properties
    stringer_width = params['stringer_width']
    stringer_thickness = params['stringer_thickness']
    section_props = calculate_shs_section_properties(stringer_width, stringer_thickness)

    # Brace geometry
    pillar_brace_offset = params['pillar_brace_vertical_offset']
    spine_width = params['spine_width']
    brace_length = math.sqrt(2) * (pillar_brace_offset + spine_width)
    angle_deg = 45.0
    angle_rad = math.radians(angle_deg)

    # Number of braces
    num_akas = params.get('num_akas', 4)
    braces_per_aka = 2
    total_braces = num_akas * braces_per_aka

    # Force per brace
    # Slam force is vertical (upward), braces are at 45°
    # Vertical component per brace = slam_force / total_braces
    # Axial force = vertical / sin(45°)
    vertical_per_brace = slam_force_n / total_braces
    axial_force_n = vertical_per_brace / math.sin(angle_rad)

    # Compressive stress
    area_mm2 = section_props['area_mm2']
    stress_mpa = axial_force_n / area_mm2

    # Euler buckling
    I_mm4 = section_props['Ix_mm4']
    K = 1.0  # pinned-pinned
    Le = K * brace_length
    P_euler_n = (math.pi ** 2) * ALUMINUM_E_MPA * I_mm4 / (Le ** 2)
    euler_stress_mpa = P_euler_n / area_mm2

    # Safety factors
    sf_buckling = P_euler_n / axial_force_n if axial_force_n > 0 else float('inf')
    sf_yielding = ALUMINUM_YIELD_STRENGTH_MPA / stress_mpa if stress_mpa > 0 else float('inf')
    safety_factor = min(sf_buckling, sf_yielding)
    passed = safety_factor >= min_safety_factor

    return {
        'component': 'diagonal_braces',
        'load_direction': 'compression (wave pushing up)',
        'axial_force_per_brace_n': round(axial_force_n, 0),
        'num_braces': total_braces,
        'compressive_stress_mpa': round(stress_mpa, 2),
        'euler_buckling_stress_mpa': round(euler_stress_mpa, 1),
        'safety_factor_buckling': round(sf_buckling, 2),
        'safety_factor_yielding': round(sf_yielding, 2),
        'safety_factor': round(safety_factor, 2),
        'governing_mode': 'buckling' if sf_buckling < sf_yielding else 'yielding',
        'passed': passed,
    }


def check_spine_slam(params: Dict[str, Any],
                      slam_force_n: float,
                      min_safety_factor: float) -> Dict[str, Any]:
    """
    Check spine bending under wave slam.

    The spine is a continuous beam supported by multiple akas (4 for RP2).
    The relevant span for bending is the distance BETWEEN adjacent akas,
    not the full spine length.

    Model: Continuous beam with uniform load, supported at aka positions.
    Maximum moment occurs at supports (negative) and mid-span (positive).
    For continuous beam: M_support ≈ wL²/12, M_midspan ≈ wL²/24
    where L is the span between adjacent akas.
    """
    # Spine section properties
    spine_width = params['spine_width']
    spine_thickness = params.get('spine_thickness', params['aka_thickness'])
    section_props = calculate_shs_section_properties(spine_width, spine_thickness)

    # Spine length and aka positions
    spine_length = params.get('panel_stringer_length', 6264)
    num_akas = params.get('num_akas', 4)

    # Average span between akas
    # With 4 akas, there are 3 spans between them, plus overhangs at ends
    # Approximate: span ≈ spine_length / (num_akas - 1) for interior spans
    if num_akas > 1:
        avg_span = spine_length / (num_akas - 1)
    else:
        avg_span = spine_length

    # Distributed load along spine
    w_n_per_mm = slam_force_n / spine_length

    # For continuous beam with uniform load:
    # Interior support moment (negative): M = w * L² / 12 (approximately)
    # Mid-span moment (positive): M = w * L² / 24
    # Use the larger (support moment) for stress check
    M_support_nmm = w_n_per_mm * avg_span**2 / 12
    M_midspan_nmm = w_n_per_mm * avg_span**2 / 24

    # Use the larger moment (at supports)
    M_max_nmm = M_support_nmm
    M_max_nm = M_max_nmm / 1000

    # Stress
    S_mm3 = section_props['Sx_mm3']
    stress_mpa = M_max_nmm / S_mm3

    # Safety factor
    safety_factor = ALUMINUM_YIELD_STRENGTH_MPA / stress_mpa if stress_mpa > 0 else float('inf')
    passed = safety_factor >= min_safety_factor

    return {
        'component': 'spine',
        'model': 'continuous beam on aka supports',
        'spine_length_mm': round(spine_length, 0),
        'num_akas': num_akas,
        'avg_span_between_akas_mm': round(avg_span, 0),
        'distributed_load_n_per_m': round(w_n_per_mm * 1000, 1),
        'moment_at_support_nm': round(M_support_nmm / 1000, 1),
        'moment_at_midspan_nm': round(M_midspan_nmm / 1000, 1),
        'max_moment_nm': round(M_max_nm, 1),
        'max_stress_mpa': round(stress_mpa, 2),
        'yield_strength_mpa': ALUMINUM_YIELD_STRENGTH_MPA,
        'safety_factor': round(safety_factor, 2),
        'passed': passed,
    }


def validate_wave_slam(params: Dict[str, Any],
                        impact_velocity_ms: float = 2.0,
                        dynamic_factor: float = 2.5,
                        min_safety_factor: float = 2.0) -> Dict[str, Any]:
    """
    Validate structural integrity under wave slam loading.

    Checks multiple components under dynamic impact load:
    1. Aka reverse bending
    2. Diagonal braces in compression
    3. Spine bending

    Args:
        params: Design parameters
        impact_velocity_ms: Wave impact velocity (default 2.0 m/s)
        dynamic_factor: Dynamic amplification (default 2.5)
        min_safety_factor: Minimum required safety factor

    Returns:
        Validation results
    """
    # Estimate wave slam force
    slam_data = estimate_wave_slam_force(params, impact_velocity_ms, dynamic_factor)
    slam_force = slam_data['dynamic_slam_force_n']

    # Check each component
    aka_result = check_aka_via_braces(params, slam_force, min_safety_factor)
    brace_result = check_diagonal_braces_slam(params, slam_force, min_safety_factor)
    spine_result = check_spine_slam(params, slam_force, min_safety_factor)

    # Overall pass
    all_passed = aka_result['passed'] and brace_result['passed'] and spine_result['passed']

    # Minimum safety factor
    min_sf = min(
        aka_result['safety_factor'],
        brace_result['safety_factor'],
        spine_result['safety_factor']
    )

    # Find governing component
    sf_list = [
        ('aka', aka_result['safety_factor']),
        ('diagonal_braces', brace_result['safety_factor']),
        ('spine', spine_result['safety_factor']),
    ]
    governing = min(sf_list, key=lambda x: x[1])

    return {
        'test_name': 'wave_slam',
        'description': f'Wave slam on ama (impact {impact_velocity_ms} m/s, dynamic factor {dynamic_factor}x)',
        'passed': all_passed,
        'min_safety_factor_required': min_safety_factor,
        'wave_slam': slam_data,
        'checks': {
            'aka': aka_result,
            'diagonal_braces': brace_result,
            'spine': spine_result,
        },
        'summary': {
            'slam_force_n': round(slam_force, 0),
            'min_safety_factor': round(min_sf, 2),
            'governing_component': governing[0],
            'result': 'PASS' if all_passed else 'FAIL',
        }
    }


def estimate_frontal_slam_force(params: Dict[str, Any],
                                 impact_velocity_ms: float = 3.0,
                                 dynamic_factor: float = 2.5) -> Dict[str, Any]:
    """
    Estimate frontal wave slam force on ama.

    The frontal area is the ama cross-section (circular).
    This is much smaller than the bottom area, so frontal slam
    forces are typically lower than vertical slam forces.

    Args:
        params: Design parameters
        impact_velocity_ms: Relative velocity of wave impact (m/s)
        dynamic_factor: Dynamic amplification factor

    Returns:
        Dictionary with force estimates
    """
    # Ama frontal area (circular cross-section)
    ama_diameter_mm = params.get('ama_diameter', 315)
    frontal_area_m2 = math.pi * (ama_diameter_mm / 2000) ** 2

    # Slam pressure coefficient for blunt body
    Cp_slam = 2.0

    # Slam pressure: P = 0.5 × rho × V² × Cp
    slam_pressure_pa = 0.5 * WATER_DENSITY_KG_M3 * impact_velocity_ms**2 * Cp_slam

    # Static slam force (before dynamic amplification)
    static_slam_force_n = slam_pressure_pa * frontal_area_m2

    # Apply dynamic amplification
    dynamic_slam_force_n = static_slam_force_n * dynamic_factor

    return {
        'ama_diameter_m': ama_diameter_mm / 1000,
        'frontal_area_m2': round(frontal_area_m2, 4),
        'impact_velocity_ms': impact_velocity_ms,
        'slam_coefficient': Cp_slam,
        'slam_pressure_kpa': round(slam_pressure_pa / 1000, 2),
        'static_slam_force_n': round(static_slam_force_n, 0),
        'dynamic_factor': dynamic_factor,
        'dynamic_slam_force_n': round(dynamic_slam_force_n, 0),
    }


def calculate_cross_brace_geometry(params: Dict[str, Any]) -> Dict[str, Any]:
    """
    Calculate cross-brace (X-brace) geometry between adjacent pillars.

    Cross-braces form X-patterns between neighboring pillars.
    They span in the Y (fore-aft) direction between aka positions.

    Returns:
        Dictionary with geometry
    """
    # Brace diameter (cylindrical tube)
    brace_diameter = params['brace_diameter']  # 5 mm

    # Vertical span
    z_upper = params['spine_base_level'] - params['brace_upper_offset']
    z_lower = params['ama_thickness'] + params['brace_lower_offset']
    vertical_span = z_upper - z_lower

    # Horizontal span (between adjacent akas)
    # Need to calculate aka positions
    # For simplicity, use average spacing
    num_akas = params.get('num_akas', 4)
    spine_length = params.get('panel_stringer_length', 6264)

    if num_akas > 1:
        avg_aka_spacing = spine_length / (num_akas - 1)
    else:
        avg_aka_spacing = spine_length

    # Brace length (diagonal of X)
    brace_length = math.sqrt(vertical_span**2 + avg_aka_spacing**2)

    # Brace angle from horizontal
    angle_rad = math.atan2(vertical_span, avg_aka_spacing)
    angle_deg = math.degrees(angle_rad)

    # Number of X-brace pairs (between adjacent pillars)
    num_x_brace_pairs = num_akas - 1

    # Each X has 2 braces, so total braces
    total_braces = num_x_brace_pairs * 2

    return {
        'brace_diameter_mm': brace_diameter,
        'vertical_span_mm': round(vertical_span, 0),
        'horizontal_span_mm': round(avg_aka_spacing, 0),
        'brace_length_mm': round(brace_length, 0),
        'angle_from_horizontal_deg': round(angle_deg, 1),
        'num_x_brace_pairs': num_x_brace_pairs,
        'total_braces': total_braces,
    }


def check_cross_braces_frontal(params: Dict[str, Any],
                                frontal_force_n: float,
                                min_safety_factor: float) -> Dict[str, Any]:
    """
    Check cross-braces under frontal wave slam.

    When wave hits ama from front (fore-aft direction):
    - Force is transmitted through the pillar structure
    - Cross-braces form X-patterns that resist shear
    - One diagonal in tension, one in compression per X

    For slender braces (high L/r ratio), the compression diagonal
    buckles immediately and becomes ineffective. The tension diagonal
    then takes the full load. This is normal for X-bracing with
    slender members - they act as tension-only braces.
    """
    # Cross-brace geometry
    geom = calculate_cross_brace_geometry(params)

    # Brace section properties (solid circular rod)
    diameter = geom['brace_diameter_mm']
    area_mm2 = math.pi * (diameter / 2) ** 2
    I_mm4 = math.pi * (diameter / 2) ** 4 / 4

    # Check slenderness ratio
    brace_length = geom['brace_length_mm']
    r_gyration = math.sqrt(I_mm4 / area_mm2)  # radius of gyration
    slenderness = brace_length / r_gyration

    # Euler buckling load
    K = 1.0  # pinned-pinned
    Le = K * brace_length
    P_euler_n = (math.pi ** 2) * ALUMINUM_E_MPA * I_mm4 / (Le ** 2)

    # For very slender braces (slenderness > 200), treat as tension-only
    # The compression diagonal buckles, tension diagonal takes all load
    is_tension_only = slenderness > 100

    # Force distribution
    num_x_pairs = geom['num_x_brace_pairs']
    force_per_x_pair = frontal_force_n / num_x_pairs

    angle_rad = math.radians(geom['angle_from_horizontal_deg'])

    if is_tension_only:
        # Tension diagonal takes the full horizontal force per X-pair
        # F_tension = F_horizontal / cos(angle)
        tension_force_per_brace = force_per_x_pair / math.cos(angle_rad)
        compression_force_per_brace = 0  # Buckled, ineffective

        # Stress in tension diagonal
        stress_mpa = tension_force_per_brace / area_mm2

        # Safety factor (tension only - yielding governs)
        safety_factor = ALUMINUM_YIELD_STRENGTH_MPA / stress_mpa if stress_mpa > 0 else float('inf')
        governing_mode = 'yielding (tension-only)'
    else:
        # Both diagonals share load
        axial_force_per_brace = force_per_x_pair / (2 * math.cos(angle_rad))
        tension_force_per_brace = axial_force_per_brace
        compression_force_per_brace = axial_force_per_brace

        stress_mpa = axial_force_per_brace / area_mm2

        # Check both buckling and yielding
        sf_buckling = P_euler_n / compression_force_per_brace if compression_force_per_brace > 0 else float('inf')
        sf_yielding = ALUMINUM_YIELD_STRENGTH_MPA / stress_mpa if stress_mpa > 0 else float('inf')
        safety_factor = min(sf_buckling, sf_yielding)
        governing_mode = 'buckling' if sf_buckling < sf_yielding else 'yielding'

    passed = safety_factor >= min_safety_factor

    return {
        'component': 'cross_braces',
        'geometry': geom,
        'brace_area_mm2': round(area_mm2, 2),
        'slenderness_ratio': round(slenderness, 0),
        'is_tension_only': is_tension_only,
        'force_per_x_pair_n': round(force_per_x_pair, 0),
        'tension_force_per_brace_n': round(tension_force_per_brace, 0),
        'stress_mpa': round(stress_mpa, 2),
        'euler_buckling_load_n': round(P_euler_n, 1),
        'yield_capacity_n': round(ALUMINUM_YIELD_STRENGTH_MPA * area_mm2, 0),
        'safety_factor': round(safety_factor, 2),
        'governing_mode': governing_mode,
        'passed': passed,
    }


def estimate_sideways_slam_force(params: Dict[str, Any],
                                  impact_velocity_ms: float = 3.0,
                                  dynamic_factor: float = 2.5) -> Dict[str, Any]:
    """
    Estimate sideways wave slam force on ama.

    The side area is ama length × diameter - same as vertical slam.
    This is much larger than frontal area, so sideways slam forces
    can be significant.

    Args:
        params: Design parameters
        impact_velocity_ms: Relative velocity of wave impact (m/s)
        dynamic_factor: Dynamic amplification factor

    Returns:
        Dictionary with force estimates
    """
    # Ama side projected area (length × diameter)
    ama_length_mm = params.get('ama_length', 9300)
    ama_diameter_mm = params.get('ama_diameter', 315)

    # Side area (same as bottom area for cylindrical ama)
    full_side_area_m2 = (ama_length_mm / 1000) * (ama_diameter_mm / 1000)

    # Effective area reduction for cylindrical shape
    area_reduction_factor = 0.5
    effective_area_m2 = full_side_area_m2 * area_reduction_factor

    # Slam pressure coefficient (lower for rounded shape)
    Cp_slam = 1.5

    # Slam pressure: P = 0.5 × rho × V² × Cp
    slam_pressure_pa = 0.5 * WATER_DENSITY_KG_M3 * impact_velocity_ms**2 * Cp_slam

    # Static slam force
    static_slam_force_n = slam_pressure_pa * effective_area_m2

    # Apply dynamic amplification
    dynamic_slam_force_n = static_slam_force_n * dynamic_factor

    return {
        'ama_length_m': ama_length_mm / 1000,
        'ama_diameter_m': ama_diameter_mm / 1000,
        'full_side_area_m2': round(full_side_area_m2, 2),
        'area_reduction_factor': area_reduction_factor,
        'effective_area_m2': round(effective_area_m2, 2),
        'impact_velocity_ms': impact_velocity_ms,
        'slam_coefficient': Cp_slam,
        'slam_pressure_kpa': round(slam_pressure_pa / 1000, 2),
        'static_slam_force_n': round(static_slam_force_n, 0),
        'dynamic_factor': dynamic_factor,
        'dynamic_slam_force_n': round(dynamic_slam_force_n, 0),
    }


def check_diagonal_braces_sideways(params: Dict[str, Any],
                                    sideways_force_n: float,
                                    min_safety_factor: float) -> Dict[str, Any]:
    """
    Check diagonal pillar braces under sideways wave slam.

    Sideways force on ama is resisted by the diagonal braces
    connecting pillars to akas. This is the same load path as
    the static lateral loading test, but with higher dynamic forces.

    The braces can be in tension or compression depending on
    which side the wave hits from. Check both cases.
    """
    # Brace section properties (SHS)
    stringer_width = params['stringer_width']
    stringer_thickness = params['stringer_thickness']
    section_props = calculate_shs_section_properties(stringer_width, stringer_thickness)

    # Brace geometry
    pillar_brace_offset = params['pillar_brace_vertical_offset']
    spine_width = params['spine_width']
    brace_length = math.sqrt(2) * (pillar_brace_offset + spine_width)
    angle_deg = 45.0
    angle_rad = math.radians(angle_deg)

    # Number of braces
    num_akas = params.get('num_akas', 4)
    braces_per_aka = 2
    total_braces = num_akas * braces_per_aka

    # Force per brace
    # Sideways force is horizontal, braces are at 45°
    # Horizontal component per brace = sideways_force / total_braces
    # Axial force = horizontal / cos(45°)
    horizontal_per_brace = sideways_force_n / total_braces
    axial_force_n = horizontal_per_brace / math.cos(angle_rad)

    # Stress
    area_mm2 = section_props['area_mm2']
    stress_mpa = axial_force_n / area_mm2

    # Buckling check (for compression case)
    I_mm4 = section_props['Ix_mm4']
    K = 1.0
    Le = K * brace_length
    P_euler_n = (math.pi ** 2) * ALUMINUM_E_MPA * I_mm4 / (Le ** 2)
    euler_stress_mpa = P_euler_n / area_mm2

    # Safety factors
    sf_buckling = P_euler_n / axial_force_n if axial_force_n > 0 else float('inf')
    sf_yielding = ALUMINUM_YIELD_STRENGTH_MPA / stress_mpa if stress_mpa > 0 else float('inf')

    # Compression case governs (buckling)
    safety_factor = min(sf_buckling, sf_yielding)
    governing_mode = 'buckling' if sf_buckling < sf_yielding else 'yielding'
    passed = safety_factor >= min_safety_factor

    return {
        'component': 'diagonal_braces',
        'brace_section': f"SHS {stringer_width}x{stringer_thickness} mm",
        'brace_length_mm': round(brace_length, 0),
        'brace_angle_deg': angle_deg,
        'num_braces': total_braces,
        'axial_force_per_brace_n': round(axial_force_n, 0),
        'stress_mpa': round(stress_mpa, 2),
        'euler_buckling_stress_mpa': round(euler_stress_mpa, 1),
        'euler_buckling_load_n': round(P_euler_n, 0),
        'safety_factor_buckling': round(sf_buckling, 2),
        'safety_factor_yielding': round(sf_yielding, 2),
        'safety_factor': round(safety_factor, 2),
        'governing_mode': governing_mode,
        'passed': passed,
    }


def validate_sideways_wave_slam(params: Dict[str, Any],
                                 impact_velocity_ms: float = 3.0,
                                 dynamic_factor: float = 2.5,
                                 min_safety_factor: float = 2.0) -> Dict[str, Any]:
    """
    Validate structural integrity under sideways wave slam.

    Sideways slam occurs when a wave hits the ama from the side.
    The lateral force is resisted by the diagonal pillar braces
    connecting pillars to akas.

    Args:
        params: Design parameters
        impact_velocity_ms: Wave impact velocity (default 3.0 m/s)
        dynamic_factor: Dynamic amplification (default 2.5)
        min_safety_factor: Minimum required safety factor

    Returns:
        Validation results
    """
    # Estimate sideways slam force
    slam_data = estimate_sideways_slam_force(params, impact_velocity_ms, dynamic_factor)
    slam_force = slam_data['dynamic_slam_force_n']

    # Check diagonal braces
    brace_result = check_diagonal_braces_sideways(params, slam_force, min_safety_factor)

    # Overall result
    all_passed = brace_result['passed']
    safety_factor = brace_result['safety_factor']

    return {
        'test_name': 'sideways_wave_slam',
        'description': f'Sideways wave slam (impact {impact_velocity_ms} m/s, dynamic factor {dynamic_factor}x)',
        'passed': all_passed,
        'min_safety_factor_required': min_safety_factor,
        'sideways_slam': slam_data,
        'checks': {
            'diagonal_braces': brace_result,
        },
        'summary': {
            'slam_force_n': round(slam_force, 0),
            'min_safety_factor': round(safety_factor, 2),
            'governing_component': 'diagonal_braces',
            'result': 'PASS' if all_passed else 'FAIL',
        }
    }


def validate_frontal_wave_slam(params: Dict[str, Any],
                                impact_velocity_ms: float = 3.0,
                                dynamic_factor: float = 2.5,
                                min_safety_factor: float = 2.0) -> Dict[str, Any]:
    """
    Validate structural integrity under frontal wave slam.

    Frontal slam occurs when the ama hits a wave head-on.
    The fore-aft force is resisted by the X-shaped cross-braces
    between adjacent pillars.

    Args:
        params: Design parameters
        impact_velocity_ms: Wave impact velocity (default 3.0 m/s)
        dynamic_factor: Dynamic amplification (default 2.5)
        min_safety_factor: Minimum required safety factor

    Returns:
        Validation results
    """
    # Estimate frontal slam force
    slam_data = estimate_frontal_slam_force(params, impact_velocity_ms, dynamic_factor)
    slam_force = slam_data['dynamic_slam_force_n']

    # Check cross-braces
    cross_brace_result = check_cross_braces_frontal(params, slam_force, min_safety_factor)

    # Overall result
    all_passed = cross_brace_result['passed']
    safety_factor = cross_brace_result['safety_factor']

    return {
        'test_name': 'frontal_wave_slam',
        'description': f'Frontal wave slam (impact {impact_velocity_ms} m/s, dynamic factor {dynamic_factor}x)',
        'passed': all_passed,
        'min_safety_factor_required': min_safety_factor,
        'frontal_slam': slam_data,
        'checks': {
            'cross_braces': cross_brace_result,
        },
        'summary': {
            'slam_force_n': round(slam_force, 0),
            'min_safety_factor': round(safety_factor, 2),
            'governing_component': 'cross_braces',
            'result': 'PASS' if all_passed else 'FAIL',
        }
    }

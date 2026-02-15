"""
Lifting sling analysis - loads when boat is lifted by crane.

V-sling configuration with 4 hooks, each connected to two neighboring akas:
- Each hook has two ropes forming a V shape
- One rope to outer aka, one rope to inner aka
- 4 hooks total (front ama, front vaka, rear ama, rear vaka)
- 8 attachment points on the akas

Load cases:
1. Aka bending from sling forces
2. Local stress at lift point attachments
3. Rope tension (accounting for V-angle)
"""

import math
from typing import Dict, Any

from .beam_mechanics import (
    ALUMINUM_YIELD_STRENGTH_MPA, ALUMINUM_E_MPA, GRAVITY,
    calculate_rhs_section_properties
)


def get_total_boat_mass(mass_data: Dict[str, Any]) -> Dict[str, float]:
    """
    Get total boat mass from mass data.

    Returns breakdown of vaka, outrigger, and total mass.
    """
    components = mass_data.get('components', [])

    vaka_mass = 0.0
    outrigger_mass = 0.0
    other_mass = 0.0

    for comp in components:
        name = comp['name']
        mass = comp['mass_kg']

        # Classify components
        if any(x in name for x in ['Vaka', 'Deck', 'Gunwale', 'Mast', 'Boom']):
            vaka_mass += mass
        elif any(x in name for x in ['Ama_', 'Solar_', 'Pillar_', 'Spine_',
                                      'Panel_', 'Cross_Brace', 'Stringer']):
            outrigger_mass += mass
        elif 'Aka' in name:
            # Akas are shared structure
            vaka_mass += mass * 0.5
            outrigger_mass += mass * 0.5
        else:
            other_mass += mass

    total_mass = vaka_mass + outrigger_mass + other_mass

    return {
        'vaka_mass_kg': vaka_mass,
        'outrigger_mass_kg': outrigger_mass,
        'other_mass_kg': other_mass,
        'total_mass_kg': total_mass,
    }


def calculate_sling_geometry(params: Dict[str, Any]) -> Dict[str, Any]:
    """
    Calculate V-sling attachment geometry.

    4 hooks, each with 2 ropes forming a V to neighboring akas.
    - Front hooks: connect front outer aka to front inner aka
    - Rear hooks: connect rear outer aka to rear inner aka
    - Ama side and vaka side for each pair
    """
    # Aka dimensions
    aka_length = params['aka_length']
    aka_width = params['aka_width']
    aka_height = params['aka_height']
    aka_thickness = params['aka_thickness']
    vaka_width = params['vaka_width']

    # Aka positions along spine
    num_akas = params.get('num_akas', 4)
    spine_length = params.get('panel_stringer_length', 6264)

    # Spacing between adjacent akas
    if num_akas > 1:
        aka_spacing = spine_length / (num_akas - 1)
    else:
        aka_spacing = 0

    # Sling attachment points on each aka
    # Ama end: at aka tip (full aka length from center)
    # Vaka end: at the vaka side of the aka
    ama_end_from_center = aka_length
    vaka_end_from_center = vaka_width / 2  # Inside edge of vaka

    # Span of aka between attachment points (ama end to vaka end)
    aka_sling_span = ama_end_from_center - vaka_end_from_center

    # V-sling geometry
    # Assume hook is centered between the two akas longitudinally
    # and some height above the aka level
    hook_height_above_aka = 2000  # mm, typical crane hook height above boat

    # Horizontal distance from hook to each aka attachment (half the aka spacing)
    horizontal_to_aka = aka_spacing / 2

    # Rope length (from hook to aka attachment point)
    rope_length = math.sqrt(horizontal_to_aka**2 + hook_height_above_aka**2)

    # V-angle from vertical (angle each rope makes with vertical)
    v_angle_rad = math.atan2(horizontal_to_aka, hook_height_above_aka)
    v_angle_deg = math.degrees(v_angle_rad)

    return {
        'aka_length_mm': aka_length,
        'aka_width_mm': aka_width,
        'aka_height_mm': aka_height,
        'aka_thickness_mm': aka_thickness,
        'vaka_width_mm': vaka_width,
        'aka_spacing_mm': aka_spacing,
        'ama_end_from_center_mm': ama_end_from_center,
        'vaka_end_from_center_mm': vaka_end_from_center,
        'aka_sling_span_mm': aka_sling_span,
        'hook_height_mm': hook_height_above_aka,
        'rope_length_mm': rope_length,
        'v_angle_deg': v_angle_deg,
        'num_hooks': 4,
        'num_attachment_points': 8,
    }


def check_aka_bending_lift(params: Dict[str, Any],
                           total_mass_kg: float,
                           min_safety_factor: float) -> Dict[str, Any]:
    """
    Check aka bending when boat is lifted with V-slings.

    Model: All 4 akas participate in the lift.
    Each aka has 2 attachment points (ama end and vaka end).
    The aka is modeled as simply supported at these points.
    Load comes from the boat weight distributed along the aka.

    With V-slings, each attachment point carries 1/8 of the total weight
    (vertically), but the rope pulls at an angle.
    """
    # Aka section properties
    aka_width = params['aka_width']
    aka_height = params['aka_height']
    aka_thickness = params['aka_thickness']
    section_props = calculate_rhs_section_properties(aka_width, aka_height, aka_thickness)

    # Geometry
    geom = calculate_sling_geometry(params)
    sling_span = geom['aka_sling_span_mm']
    vaka_width = geom['vaka_width_mm']
    v_angle_deg = geom['v_angle_deg']
    v_angle_rad = math.radians(v_angle_deg)

    # With 4 akas and 8 attachment points, each aka carries 1/4 of total weight
    # (Each aka has 2 attachments, each carrying 1/8)
    weight_per_aka = total_mass_kg * GRAVITY / 4
    vertical_force_per_attachment = weight_per_aka / 2

    # Rope tension is higher due to V-angle
    # T = F_vertical / cos(angle)
    rope_tension = vertical_force_per_attachment / math.cos(v_angle_rad)

    # The weight of the boat acts primarily at the vaka region
    # Model as point load at the vaka connection point
    vaka_connection_from_ama = sling_span - vaka_width / 4

    # Simply supported beam with point load
    # M_max = P * a * b / L
    a = vaka_connection_from_ama
    b = sling_span - a
    L = sling_span

    M_max_nmm = weight_per_aka * a * b / L
    M_max_nm = M_max_nmm / 1000

    # Stress (strong axis resists vertical bending)
    S_mm3 = section_props['Sx_mm3']
    stress_mpa = M_max_nmm / S_mm3

    # Safety factor
    safety_factor = ALUMINUM_YIELD_STRENGTH_MPA / stress_mpa if stress_mpa > 0 else float('inf')
    passed = safety_factor >= min_safety_factor

    return {
        'component': 'aka_bending',
        'model': 'simply supported beam (all 4 akas participate)',
        'sling_span_mm': round(sling_span, 0),
        'weight_per_aka_n': round(weight_per_aka, 0),
        'vertical_force_per_attachment_n': round(vertical_force_per_attachment, 0),
        'rope_tension_n': round(rope_tension, 0),
        'v_angle_deg': round(v_angle_deg, 1),
        'max_moment_nm': round(M_max_nm, 1),
        'max_stress_mpa': round(stress_mpa, 2),
        'yield_strength_mpa': ALUMINUM_YIELD_STRENGTH_MPA,
        'safety_factor': round(safety_factor, 2),
        'passed': passed,
    }


def check_sling_point_stress(params: Dict[str, Any],
                              total_mass_kg: float,
                              min_safety_factor: float) -> Dict[str, Any]:
    """
    Check local stress at sling attachment points.

    With V-slings: 8 attachment points total.
    Each point sees 1/8 of the total weight (vertically), but rope
    pulls at an angle, increasing the total force.

    Check bearing stress from rope wrapped around aka.
    """
    # Aka section properties
    aka_width = params['aka_width']
    aka_height = params['aka_height']
    aka_thickness = params['aka_thickness']

    # Geometry for V-angle
    geom = calculate_sling_geometry(params)
    v_angle_deg = geom['v_angle_deg']
    v_angle_rad = math.radians(v_angle_deg)

    # Vertical force per attachment point (8 points total)
    vertical_force = total_mass_kg * GRAVITY / 8

    # Rope tension is higher due to V-angle
    rope_tension = vertical_force / math.cos(v_angle_rad)

    # Assume rope/strap wraps around aka with contact width ≈ 50mm
    sling_contact_width = 50  # mm

    # For a rope wrapped around a tube, the load distributes over the wrap
    # Bearing area = contact width × tube perimeter in contact (≈ 180° wrap)
    # Use conservative estimate: load on bottom and two sides
    wrap_length = aka_width + aka_height  # Bottom + one side (rope goes up to hook)
    bearing_area = sling_contact_width * aka_thickness * 3  # Three wall segments

    # Bearing stress
    bearing_stress = rope_tension / bearing_area

    # Local wall bending is less severe with wrap distribution
    # Model as distributed load over the wrap area
    # Much less critical than point load assumption
    wall_span = aka_width - 2 * aka_thickness
    # Distributed load intensity
    w_local = rope_tension / wrap_length  # N/mm
    # Max bending for distributed load: M = w*L²/8
    M_local = w_local * wall_span**2 / 8

    # Section modulus of wall strip
    S_wall = sling_contact_width * aka_thickness**2 / 6
    local_bending_stress = M_local / S_wall

    # Combined stress (bearing dominates for wrapped slings)
    combined_stress = bearing_stress + local_bending_stress * 0.3

    # Safety factor
    safety_factor = ALUMINUM_YIELD_STRENGTH_MPA / combined_stress if combined_stress > 0 else float('inf')
    passed = safety_factor >= min_safety_factor

    return {
        'component': 'sling_attachment_point',
        'num_attachment_points': 8,
        'vertical_force_per_point_n': round(vertical_force, 0),
        'rope_tension_n': round(rope_tension, 0),
        'v_angle_deg': round(v_angle_deg, 1),
        'sling_contact_width_mm': sling_contact_width,
        'bearing_area_mm2': round(bearing_area, 0),
        'bearing_stress_mpa': round(bearing_stress, 2),
        'local_bending_stress_mpa': round(local_bending_stress, 2),
        'combined_stress_mpa': round(combined_stress, 2),
        'yield_strength_mpa': ALUMINUM_YIELD_STRENGTH_MPA,
        'safety_factor': round(safety_factor, 2),
        'passed': passed,
    }


def check_rope_tension(params: Dict[str, Any],
                       total_mass_kg: float,
                       min_safety_factor: float) -> Dict[str, Any]:
    """
    Check rope/strap tension in the V-sling configuration.

    Each of the 8 ropes carries 1/8 of the weight vertically,
    but the rope tension is higher due to the V-angle.

    Assume polyester lifting slings with working load limit.
    """
    # Geometry
    geom = calculate_sling_geometry(params)
    v_angle_deg = geom['v_angle_deg']
    v_angle_rad = math.radians(v_angle_deg)
    rope_length = geom['rope_length_mm']

    # Vertical force per rope
    vertical_force = total_mass_kg * GRAVITY / 8

    # Rope tension (higher due to angle)
    rope_tension = vertical_force / math.cos(v_angle_rad)

    # Typical 50mm polyester flat sling: WLL ≈ 2000 kg (straight pull)
    # In basket hitch (which is similar to our wrap), WLL is doubled
    # But we're using it as a choker/wrap, so use straight pull rating
    # With safety factor built in, breaking strength ≈ 7:1
    sling_wll_kg = 2000  # Working load limit per sling
    sling_breaking_strength_n = sling_wll_kg * GRAVITY * 7  # ≈ 137,000 N

    # Sling working capacity (stay within WLL)
    sling_working_capacity_n = sling_wll_kg * GRAVITY  # ≈ 19,600 N

    # Safety factor based on working load limit (not breaking)
    safety_factor = sling_working_capacity_n / rope_tension if rope_tension > 0 else float('inf')
    passed = safety_factor >= min_safety_factor

    return {
        'component': 'rope_tension',
        'num_ropes': 8,
        'rope_length_mm': round(rope_length, 0),
        'v_angle_deg': round(v_angle_deg, 1),
        'vertical_force_per_rope_n': round(vertical_force, 0),
        'rope_tension_n': round(rope_tension, 0),
        'sling_wll_kg': sling_wll_kg,
        'sling_capacity_n': round(sling_working_capacity_n, 0),
        'safety_factor': round(safety_factor, 2),
        'passed': passed,
        'note': '50mm polyester sling, WLL=2000kg',
    }


def check_global_bending(params: Dict[str, Any],
                         mass_data: Dict[str, Any],
                         total_mass_kg: float,
                         min_safety_factor: float) -> Dict[str, Any]:
    """
    Check global bending of boat structure between lift points.

    With V-slings to all 4 akas, the lift points are better distributed.
    The spans are between adjacent akas (not outer-to-outer).

    Model: Continuous beam with supports at each aka position.
    """
    # Geometry
    geom = calculate_sling_geometry(params)
    aka_spacing = geom['aka_spacing_mm']
    num_akas = params.get('num_akas', 4)

    # Vaka section (approximate as rectangular beam)
    vaka_width_mm = params['vaka_width']

    # Approximate vaka section modulus
    # Assume gunwales + deck form a box beam
    gunwale_height = 100  # Approximate
    S_vaka = vaka_width_mm * gunwale_height**2 / 6

    # Total weight
    total_weight = total_mass_kg * GRAVITY

    # Distributed load along the full span
    full_span = aka_spacing * (num_akas - 1)
    w_n_per_mm = total_weight / full_span if full_span > 0 else 0

    # For continuous beam with multiple supports, max moment is less
    # than simply supported. Use M_max ≈ w * L² / 10 for interior spans
    M_max_nmm = w_n_per_mm * aka_spacing**2 / 10 if aka_spacing > 0 else 0
    M_max_nm = M_max_nmm / 1000

    # Stress
    stress_mpa = M_max_nmm / S_vaka if S_vaka > 0 else 0

    # For wooden/composite structure
    wood_allowable_stress = 25  # MPa

    # Safety factor
    safety_factor = wood_allowable_stress / stress_mpa if stress_mpa > 0 else float('inf')
    passed = safety_factor >= min_safety_factor

    return {
        'component': 'global_boat_bending',
        'model': 'continuous beam with supports at all akas',
        'aka_spacing_mm': round(aka_spacing, 0),
        'num_supports': num_akas,
        'total_weight_n': round(total_weight, 0),
        'distributed_load_n_per_m': round(w_n_per_mm * 1000, 1),
        'max_moment_nm': round(M_max_nm, 1),
        'section_modulus_mm3': round(S_vaka, 0),
        'max_stress_mpa': round(stress_mpa, 2),
        'allowable_stress_mpa': wood_allowable_stress,
        'safety_factor': round(safety_factor, 2),
        'passed': passed,
        'note': 'Vaka modeled as plywood/wood structure',
    }


def validate_lifting_sling(params: Dict[str, Any],
                           mass_data: Dict[str, Any],
                           min_safety_factor: float = 2.0) -> Dict[str, Any]:
    """
    Validate structural integrity when boat is lifted by crane.

    V-sling configuration: 4 hooks, each with 2 ropes to neighboring akas.
    This distributes load across all 4 akas with 8 attachment points.

    Checks:
    1. Aka bending from sling forces
    2. Local stress at sling attachment points
    3. Rope/sling tension
    4. Global bending of boat structure

    Args:
        params: Design parameters
        mass_data: Mass calculation results
        min_safety_factor: Minimum required safety factor

    Returns:
        Validation results
    """
    # Get total boat mass
    mass_breakdown = get_total_boat_mass(mass_data)
    total_mass = mass_breakdown['total_mass_kg']

    # Sling geometry
    sling_geom = calculate_sling_geometry(params)

    # Run checks
    aka_result = check_aka_bending_lift(params, total_mass, min_safety_factor)
    sling_point_result = check_sling_point_stress(params, total_mass, min_safety_factor)
    rope_result = check_rope_tension(params, total_mass, min_safety_factor)
    global_result = check_global_bending(params, mass_data, total_mass, min_safety_factor)

    # Overall pass
    all_passed = (aka_result['passed'] and
                  sling_point_result['passed'] and
                  rope_result['passed'] and
                  global_result['passed'])

    # Minimum safety factor
    min_sf = min(
        aka_result['safety_factor'],
        sling_point_result['safety_factor'],
        rope_result['safety_factor'],
        global_result['safety_factor']
    )

    # Find governing component
    sf_list = [
        ('aka_bending', aka_result['safety_factor']),
        ('sling_point', sling_point_result['safety_factor']),
        ('rope_tension', rope_result['safety_factor']),
        ('global_bending', global_result['safety_factor']),
    ]
    governing = min(sf_list, key=lambda x: x[1])

    return {
        'test_name': 'lifting_sling',
        'description': 'V-sling crane lift (4 hooks to neighboring akas)',
        'passed': all_passed,
        'min_safety_factor_required': min_safety_factor,
        'mass_breakdown': mass_breakdown,
        'sling_geometry': {
            'num_hooks': sling_geom['num_hooks'],
            'num_attachment_points': sling_geom['num_attachment_points'],
            'aka_spacing_m': round(sling_geom['aka_spacing_mm'] / 1000, 2),
            'v_angle_deg': round(sling_geom['v_angle_deg'], 1),
            'rope_length_m': round(sling_geom['rope_length_mm'] / 1000, 2),
        },
        'checks': {
            'aka_bending': aka_result,
            'sling_point': sling_point_result,
            'rope_tension': rope_result,
            'global_bending': global_result,
        },
        'summary': {
            'total_mass_kg': round(total_mass, 1),
            'vertical_force_per_attachment_n': round(total_mass * GRAVITY / 8, 0),
            'min_safety_factor': round(min_sf, 2),
            'governing_component': governing[0],
            'result': 'PASS' if all_passed else 'FAIL',
        }
    }

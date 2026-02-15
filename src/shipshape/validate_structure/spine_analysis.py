"""
Spine structural analysis for "one end of ama supported" load case.

Models the ama as a beam on elastic supports (akas) with one end supported.
Solves for reactions at each aka, then calculates spine bending stress.
"""

import numpy as np
from typing import Dict, Any, List, Tuple

from .beam_mechanics import (
    ALUMINUM_YIELD_STRENGTH_MPA, ALUMINUM_E_MPA, PVC_E_MPA, GRAVITY,
    calculate_rhs_section_properties, calculate_shs_section_properties,
    calculate_pipe_section_properties, cantilever_stiffness
)


def get_aka_positions(params: Dict[str, Any]) -> List[float]:
    """
    Calculate Y positions of all akas along the spine.

    Returns:
        Sorted list of aka Y positions (mm)
    """
    panels_longitudinal = params['panels_longitudinal']
    akas_per_panel = params['akas_per_panel']
    panel_width = params['panel_width']
    crossdeck_width = params['crossdeck_width']
    aka_rim = params['aka_rim']
    spine_length = params['spine_length']

    def aka_y_position(panel_index, aka_index):
        panel_start_y = crossdeck_width / 2 + panel_index * panel_width
        if akas_per_panel == 1:
            return panel_start_y + panel_width / 2
        else:
            aka_spacing = (panel_width - 2 * aka_rim) / (akas_per_panel - 1)
            return panel_start_y + aka_rim + aka_index * aka_spacing

    # Front half aka positions
    front_positions = []
    for i in range(panels_longitudinal // 2):
        for j in range(akas_per_panel):
            front_positions.append(aka_y_position(i, j))

    # Back half (mirrored about center)
    center_y = spine_length / 2
    back_positions = [2 * center_y - y for y in front_positions]

    return sorted(front_positions + back_positions)


def solve_beam_on_elastic_supports(beam_length: float,
                                    beam_EI: float,
                                    support_pos: List[float],
                                    spring_stiffness: float,
                                    loads: List[Tuple[float, float]]) -> Dict[str, Any]:
    """
    Solve for reactions of a beam on elastic supports with one end pinned.

    The beam is pinned at y=0 and has elastic supports (springs) at
    the given positions. Loads are applied as point forces.

    Args:
        beam_length: Total beam length (mm)
        beam_EI: Beam flexural rigidity (N·mm²)
        support_pos: List of elastic support positions (mm)
        spring_stiffness: Stiffness of each spring (N/mm)
        loads: List of (position, force) tuples for point loads (N)

    Returns:
        Dictionary with reactions at each support and deflections
    """
    n_springs = len(support_pos)
    k = spring_stiffness

    def cantilever_deflection(x, load_pos, load_force, EI):
        """Deflection at x due to point load, cantilever from origin."""
        a = load_pos
        P = load_force
        if x >= a:
            return P * a**2 * (3*x - a) / (6 * EI)
        else:
            return P * x**2 * (3*a - x) / (6 * EI)

    # Deflection at each spring due to applied loads (no springs present)
    d0 = np.zeros(n_springs)
    for i in range(n_springs):
        xi = support_pos[i]
        for load_pos, load_force in loads:
            d0[i] += cantilever_deflection(xi, load_pos, load_force, beam_EI)

    # Flexibility matrix: deflection at spring i due to unit upward force at spring j
    F_beam = np.zeros((n_springs, n_springs))
    for i in range(n_springs):
        xi = support_pos[i]
        for j in range(n_springs):
            xj = support_pos[j]
            F_beam[i, j] = -cantilever_deflection(xi, xj, 1.0, beam_EI)

    # System: d0 + F_beam @ R = R / k
    # Rearranging: (F_beam - diag(1/k)) @ R = -d0
    K_inv = np.diag([1.0/k] * n_springs)
    A = F_beam - K_inv
    R = -np.linalg.solve(A, d0)

    # Deflections at spring positions
    deflections = R / k

    # Reaction at fixed support (equilibrium)
    total_load = sum(f for _, f in loads)
    R_fixed = total_load - sum(R)

    return {
        'spring_reactions': R.tolist(),
        'spring_deflections': deflections.tolist(),
        'fixed_support_reaction': R_fixed,
        'support_positions': support_pos
    }


def analyze_spine_bending(aka_positions: List[float],
                          aka_reactions: List[float],
                          spine_props: Dict[str, float]) -> Dict[str, Any]:
    """
    Analyze spine bending due to aka reactions.

    Args:
        aka_positions: Y positions of akas along spine (mm)
        aka_reactions: Forces at each aka (N), positive = pulling down on spine
        spine_props: Dictionary with spine section properties

    Returns:
        Dictionary with bending analysis results
    """
    n = len(aka_positions)

    # Sort by position
    sorted_pairs = sorted(zip(aka_positions, aka_reactions))
    positions = [p for p, _ in sorted_pairs]
    reactions = [r for _, r in sorted_pairs]

    span = positions[-1] - positions[0]

    # Calculate support reactions (spine supported at outer aka positions)
    M_about_left = sum(reactions[i] * (positions[i] - positions[0]) for i in range(n))
    R_right = M_about_left / span
    R_left = sum(reactions) - R_right

    # Calculate moment along the span
    sample_points = np.linspace(positions[0], positions[-1], 100)
    moments = []

    for x in sample_points:
        M = -R_left * (x - positions[0])
        for i in range(n):
            if positions[i] < x:
                M += reactions[i] * (x - positions[i])
        moments.append(M)

    max_moment_idx = np.argmax(np.abs(moments))
    max_moment = moments[max_moment_idx]
    max_moment_pos = sample_points[max_moment_idx]

    S = spine_props['Sx_mm3']  # For SHS, Sx = Sy
    max_stress = abs(max_moment) / S
    safety_factor = ALUMINUM_YIELD_STRENGTH_MPA / max_stress if max_stress > 0 else float('inf')

    return {
        'max_moment_nmm': max_moment,
        'max_moment_nm': max_moment / 1000,
        'max_moment_position_mm': max_moment_pos,
        'max_stress_mpa': max_stress,
        'safety_factor': safety_factor,
        'support_reactions': {'left': R_left, 'right': R_right},
    }


def extract_ama_loads(params: Dict[str, Any],
                      mass_data: Dict[str, Any],
                      aka_positions: List[float]) -> List[Tuple[float, float]]:
    """
    Extract load distribution along the ama from mass data.

    Returns:
        List of (y_position, force_N) tuples
    """
    components = mass_data.get('components', [])
    ama_length = params['ama_length']
    cone_length = params['ama_cone_length']

    loads = []

    # Patterns for outrigger components
    outrigger_patterns = ['Ama_pipe', 'Ama_Body_Foam', 'Ama_Cone',
                          'Pillar_', 'Panel_', 'Stringer_',
                          'Cross_Brace', 'Pillar_Brace']

    for comp in components:
        name = comp['name']
        mass = comp['mass_kg']

        # Check if this is an outrigger component
        if not any(pattern in name for pattern in outrigger_patterns):
            continue

        is_back_half = '_001' in name

        # Determine Y position based on component type
        if 'Ama_pipe' in name or 'Ama_Body_Foam' in name:
            if is_back_half:
                y_pos = (ama_length / 2 + ama_length - cone_length) / 2
            else:
                y_pos = (cone_length + ama_length / 2) / 2

        elif 'Ama_Cone' in name:
            if is_back_half:
                y_pos = ama_length - cone_length / 2
            else:
                y_pos = cone_length / 2

        elif 'Panel_' in name:
            parts = name.replace('Panel_', '').split('_')
            try:
                panel_i = int(parts[0])
                if is_back_half:
                    y_pos = aka_positions[-(panel_i + 1)] if panel_i < len(aka_positions) else ama_length / 2
                else:
                    y_pos = aka_positions[panel_i] if panel_i < len(aka_positions) else ama_length / 2
            except (ValueError, IndexError):
                y_pos = ama_length / 2

        elif 'Pillar_' in name and 'Brace' not in name:
            parts = name.replace('Pillar_', '').split('_')
            try:
                pillar_i = int(parts[0])
                if is_back_half:
                    y_pos = aka_positions[-(pillar_i + 1)] if pillar_i < len(aka_positions) else ama_length / 2
                else:
                    y_pos = aka_positions[pillar_i] if pillar_i < len(aka_positions) else ama_length / 2
            except (ValueError, IndexError):
                y_pos = ama_length / 2

        else:  # Stringers, braces, etc.
            if is_back_half:
                y_pos = (aka_positions[-1] + aka_positions[-2]) / 2 if len(aka_positions) >= 2 else ama_length / 2
            else:
                y_pos = (aka_positions[0] + aka_positions[1]) / 2 if len(aka_positions) >= 2 else ama_length / 2

        loads.append((y_pos, mass * GRAVITY))

    return loads


def validate_one_end_supported(params: Dict[str, Any],
                                mass_data: Dict[str, Any],
                                min_safety_factor: float = 2.0) -> Dict[str, Any]:
    """
    Validate spine structural integrity for one-end-supported load case.

    Args:
        params: Design parameters
        mass_data: Mass calculation results
        min_safety_factor: Minimum required safety factor

    Returns:
        Validation results
    """
    # Geometry
    ama_length = params['ama_length']
    ama_diameter = params['ama_diameter']
    ama_thickness = params['ama_thickness']

    aka_width = params['aka_width']
    aka_height = params['aka_height']
    aka_thickness = params['aka_thickness']
    aka_length = params['aka_length']
    vaka_width = params['vaka_width']

    spine_width = params['spine_width']
    spine_thickness = params['spine_thickness']

    # Aka positions
    aka_positions = get_aka_positions(params)
    n_akas = len(aka_positions)

    # Section properties
    ama_props = calculate_pipe_section_properties(ama_diameter, ama_thickness)
    aka_props = calculate_rhs_section_properties(aka_width, aka_height, aka_thickness)
    spine_props = calculate_shs_section_properties(spine_width, spine_thickness)

    # Beam stiffnesses
    EI_ama = PVC_E_MPA * ama_props['I_mm4']
    cantilever_length = aka_length - vaka_width
    k_aka = cantilever_stiffness(ALUMINUM_E_MPA, aka_props['Ix_mm4'], cantilever_length)

    # Load distribution along ama
    loads = extract_ama_loads(params, mass_data, aka_positions)
    total_load_n = sum(f for _, f in loads)
    total_mass_kg = total_load_n / GRAVITY

    # Solve beam on elastic supports (ama supported at y=0)
    ama_result = solve_beam_on_elastic_supports(
        beam_length=ama_length,
        beam_EI=EI_ama,
        support_pos=aka_positions,
        spring_stiffness=k_aka,
        loads=loads
    )

    # Analyze spine bending
    spine_result = analyze_spine_bending(
        aka_positions=aka_positions,
        aka_reactions=ama_result['spring_reactions'],
        spine_props=spine_props
    )

    passed = bool(spine_result['safety_factor'] >= min_safety_factor)

    return {
        'test_name': 'one_end_supported',
        'description': 'Spine bending: one end of ama supported, other end hanging',
        'passed': passed,
        'min_safety_factor_required': min_safety_factor,
        'geometry': {
            'ama_length_mm': ama_length,
            'spine_length_mm': params['spine_length'],
            'aka_cantilever_mm': cantilever_length,
            'aka_positions_mm': aka_positions,
            'num_akas': n_akas,
        },
        'stiffness': {
            'ama_EI_nmm2': EI_ama,
            'aka_stiffness_n_per_mm': k_aka,
        },
        'ama_analysis': {
            'total_outrigger_mass_kg': round(total_mass_kg, 2),
            'total_load_n': round(total_load_n, 1),
            'fixed_support_reaction_n': round(ama_result['fixed_support_reaction'], 1),
            'aka_reactions_n': [round(r, 1) for r in ama_result['spring_reactions']],
            'aka_deflections_mm': [round(d, 1) for d in ama_result['spring_deflections']],
        },
        'spine_analysis': {
            'section': f"SHS {spine_width}x{spine_width}x{spine_thickness} mm",
            'section_modulus_mm3': round(spine_props['Sx_mm3'], 0),
            'max_moment_nm': round(spine_result['max_moment_nm'], 1),
            'max_moment_position_mm': round(spine_result['max_moment_position_mm'], 0),
            'max_stress_mpa': round(spine_result['max_stress_mpa'], 2),
            'yield_strength_mpa': ALUMINUM_YIELD_STRENGTH_MPA,
            'safety_factor': round(spine_result['safety_factor'], 2),
        },
        'summary': {
            'max_stress_mpa': round(spine_result['max_stress_mpa'], 2),
            'safety_factor': round(spine_result['safety_factor'], 2),
            'result': 'PASS' if passed else 'FAIL'
        }
    }

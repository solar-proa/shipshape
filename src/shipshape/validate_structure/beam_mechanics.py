"""
Common beam mechanics functions for structural analysis.

This module provides section property calculations and beam analysis
functions used by both aka and spine validators.
"""

import numpy as np

# Material properties
ALUMINUM_YIELD_STRENGTH_MPA = 240
ALUMINUM_E_MPA = 69000
PVC_E_MPA = 3000
GRAVITY = 9.81


def calculate_rhs_section_properties(width_mm, height_mm, thickness_mm):
    """
    Calculate section properties for a rectangular hollow section (RHS).

    Args:
        width_mm: Outer width (horizontal dimension)
        height_mm: Outer height (vertical dimension)
        thickness_mm: Wall thickness

    Returns:
        Dictionary with section properties (area, moments of inertia, section moduli)
    """
    b = width_mm
    h = height_mm
    t = thickness_mm

    bi = b - 2 * t
    hi = h - 2 * t

    area = b * h - bi * hi
    Ix = (b * h**3 - bi * hi**3) / 12
    Iy = (h * b**3 - hi * bi**3) / 12
    Sx = Ix / (h / 2)
    Sy = Iy / (b / 2)

    return {
        'area_mm2': area,
        'Ix_mm4': Ix,
        'Iy_mm4': Iy,
        'Sx_mm3': Sx,
        'Sy_mm3': Sy,
        'width_mm': b,
        'height_mm': h,
        'thickness_mm': t
    }


def calculate_shs_section_properties(width_mm, thickness_mm):
    """
    Calculate section properties for a square hollow section (SHS).

    Args:
        width_mm: Outer width (same in both directions)
        thickness_mm: Wall thickness

    Returns:
        Dictionary with section properties
    """
    return calculate_rhs_section_properties(width_mm, width_mm, thickness_mm)


def calculate_pipe_section_properties(outer_diameter_mm, thickness_mm):
    """
    Calculate section properties for a circular hollow section (pipe).

    Args:
        outer_diameter_mm: Outer diameter
        thickness_mm: Wall thickness

    Returns:
        Dictionary with section properties
    """
    d_outer = outer_diameter_mm
    d_inner = outer_diameter_mm - 2 * thickness_mm

    area = np.pi / 4 * (d_outer**2 - d_inner**2)
    I = np.pi / 64 * (d_outer**4 - d_inner**4)
    S = I / (d_outer / 2)

    return {
        'area_mm2': area,
        'I_mm4': I,
        'S_mm3': S,
        'outer_diameter_mm': d_outer,
        'inner_diameter_mm': d_inner,
        'thickness_mm': thickness_mm
    }


def cantilever_stiffness(E_mpa, I_mm4, L_mm):
    """
    Calculate stiffness of a cantilever beam at its tip.

    k = 3EI/L³

    Args:
        E_mpa: Young's modulus (MPa = N/mm²)
        I_mm4: Second moment of area (mm⁴)
        L_mm: Cantilever length (mm)

    Returns:
        Stiffness in N/mm
    """
    return 3 * E_mpa * I_mm4 / L_mm**3


def cantilever_deflection_at_x(x, load_pos, load_force, EI):
    """
    Deflection at position x due to point load on a cantilever beam.

    Cantilever is fixed at origin (x=0), free end at positive x.

    Args:
        x: Position where deflection is calculated (mm)
        load_pos: Position of point load (mm)
        load_force: Magnitude of load (N), positive = downward
        EI: Flexural rigidity (N·mm²)

    Returns:
        Deflection (mm), positive = downward
    """
    a = load_pos
    P = load_force
    if x >= a:
        return P * a**2 * (3*x - a) / (6 * EI)
    else:
        return P * x**2 * (3*a - x) / (6 * EI)


def cantilever_moment(x, tip_force, distributed_force, length):
    """
    Bending moment at position x for a cantilever with tip and distributed loads.

    Args:
        x: Position from fixed end (mm)
        tip_force: Force at tip (N)
        distributed_force: Total distributed force (N), uniformly distributed
        length: Cantilever length (mm)

    Returns:
        Bending moment (N·mm)
    """
    # Moment from tip load
    if x <= length:
        M_tip = tip_force * (length - x)
    else:
        M_tip = 0

    # Moment from distributed load (w = distributed_force / length)
    if x <= length:
        w = distributed_force / length
        M_dist = w * (length - x)**2 / 2
    else:
        M_dist = 0

    return M_tip + M_dist


def simply_supported_moment(x, span, loads):
    """
    Bending moment at position x for a simply supported beam with point loads.

    Supports at x=0 and x=span.

    Args:
        x: Position where moment is calculated (mm)
        span: Distance between supports (mm)
        loads: List of (position, force) tuples, positions relative to left support

    Returns:
        Bending moment (N·mm)
    """
    # Calculate reactions
    # Sum moments about left support
    M_left = sum(F * a for a, F in loads)
    R_right = M_left / span
    R_left = sum(F for _, F in loads) - R_right

    # Moment at x
    M = R_left * x
    for a, F in loads:
        if a < x:
            M -= F * (x - a)

    return M

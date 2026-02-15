"""
Capsize analysis - calculates wind speed at which ama lifts when wind comes from ama side.

For a proa, the critical case is wind from the ama side (negative heel),
where stability is lower. This module calculates the wind speed at which
the heeling moment equals the maximum available righting moment.

This is informational - not a pass/fail criterion.
"""

import math
from typing import Dict, Any, Optional

from .beam_mechanics import GRAVITY

# Air properties
AIR_DENSITY_KG_M3 = 1.225
SAIL_DRAG_COEFFICIENT = 1.15


def knots_to_ms(knots: float) -> float:
    """Convert wind speed from knots to m/s."""
    return knots * 0.514444


def ms_to_knots(ms: float) -> float:
    """Convert wind speed from m/s to knots."""
    return ms / 0.514444


def calculate_wind_force(wind_speed_knots: float, sail_area_m2: float,
                         num_sails: int = 2) -> float:
    """
    Calculate total wind force on sails.

    Args:
        wind_speed_knots: Wind speed in knots
        sail_area_m2: Area of one sail in m²
        num_sails: Number of sails (default 2 for proa)

    Returns:
        Total force in Newtons
    """
    V = knots_to_ms(wind_speed_knots)
    force_per_sail = 0.5 * AIR_DENSITY_KG_M3 * V**2 * SAIL_DRAG_COEFFICIENT * sail_area_m2
    return force_per_sail * num_sails


def calculate_heeling_moment(wind_force_n: float,
                              center_of_effort_height_m: float) -> float:
    """
    Calculate heeling moment from wind force.

    Args:
        wind_force_n: Total wind force on sails (N)
        center_of_effort_height_m: Height of sail center of effort above heeling axis (m)

    Returns:
        Heeling moment in N·m
    """
    return wind_force_n * center_of_effort_height_m


def find_max_righting_moment_negative_heel(gz_data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Find maximum righting moment for negative heel angles (wind from ama side).

    For a proa, negative heel = heeling away from the ama.
    This is the less stable direction.

    Args:
        gz_data: GZ curve data from gz phase

    Returns:
        Dictionary with max righting moment and angle
    """
    gz_curve = gz_data.get('gz_curve', [])

    # Filter to negative heel angles (away from ama)
    negative_heel_points = [
        p for p in gz_curve
        if p.get('converged', False) and p['heel_deg'] < 0
    ]

    if not negative_heel_points:
        return {
            'max_righting_moment_nm': 0,
            'max_righting_angle_deg': None,
            'error': 'No converged negative heel points in GZ data'
        }

    # Find maximum righting moment
    max_rm = max(p['righting_moment_Nm'] for p in negative_heel_points)
    max_point = next(p for p in negative_heel_points if p['righting_moment_Nm'] == max_rm)

    return {
        'max_righting_moment_nm': max_rm,
        'max_righting_angle_deg': max_point['heel_deg'],
        'gz_at_max_m': max_point['gz_m']
    }


def find_capsize_angle(gz_data: Dict[str, Any]) -> Optional[float]:
    """
    Find the capsize angle from GZ summary.

    This is the negative heel angle where GZ crosses zero.

    Args:
        gz_data: GZ curve data

    Returns:
        Capsize angle in degrees, or None if not found
    """
    return gz_data.get('summary', {}).get('capsize_angle_deg')


def estimate_center_of_effort_height(params: Dict[str, Any]) -> float:
    """
    Estimate the height of sail center of effort above the heeling axis.

    The heeling axis is approximately at the waterline.
    CE is roughly at 40% of sail height above the boom.

    Args:
        params: Design parameters

    Returns:
        CE height in meters
    """
    # Mast partner is roughly at deck level
    mast_partner_level_mm = params.get('aka_base_level', 1400)

    # Waterline is roughly at bottom_height + some immersion
    # For simplicity, assume heeling axis is near waterline, about 500mm below partner
    heeling_axis_mm = mast_partner_level_mm - 500

    # Boom is at partner level, CE is 40% of sail height above boom
    sail_height_mm = params.get('sail_height', 5500)
    ce_above_boom_mm = sail_height_mm * 0.40

    # CE height above heeling axis
    ce_height_mm = (mast_partner_level_mm - heeling_axis_mm) + ce_above_boom_mm

    return ce_height_mm / 1000  # Convert to meters


def calculate_ama_lift_windspeed(params: Dict[str, Any],
                                  gz_data: Dict[str, Any]) -> Dict[str, Any]:
    """
    Calculate wind speed at which ama lifts out of water.

    This is the wind speed where heeling moment equals maximum righting moment
    for wind coming from the ama side (negative heel direction).

    Args:
        params: Design parameters
        gz_data: GZ curve data from gz phase

    Returns:
        Analysis results with critical wind speed
    """
    # Sail area
    sail_height = params.get('sail_height', 5500)
    sail_width = params.get('sail_width', 5500)
    sail_area_m2 = (sail_height * sail_width) / 1e6
    num_sails = 2  # Proa has two sails

    # Center of effort height
    ce_height_m = estimate_center_of_effort_height(params)

    # Maximum righting moment from GZ curve (negative heel)
    rm_data = find_max_righting_moment_negative_heel(gz_data)
    max_righting_moment = rm_data['max_righting_moment_nm']

    # Capsize angle from GZ data
    capsize_angle = find_capsize_angle(gz_data)

    # Calculate critical wind speed
    # Heeling moment = 0.5 * rho * V² * Cd * A * num_sails * ce_height
    # At critical: heeling_moment = max_righting_moment
    # V² = max_righting_moment / (0.5 * rho * Cd * A * num_sails * ce_height)

    coefficient = 0.5 * AIR_DENSITY_KG_M3 * SAIL_DRAG_COEFFICIENT * sail_area_m2 * num_sails * ce_height_m

    if coefficient > 0 and max_righting_moment > 0:
        V_squared = max_righting_moment / coefficient
        V_ms = math.sqrt(V_squared)
        critical_windspeed_knots = ms_to_knots(V_ms)
    else:
        critical_windspeed_knots = 0

    # Calculate heeling moment at a reference wind speed (25 knots) for comparison
    reference_wind = 25.0
    reference_force = calculate_wind_force(reference_wind, sail_area_m2, num_sails)
    reference_heeling = calculate_heeling_moment(reference_force, ce_height_m)

    return {
        'test_name': 'ama_lift_windspeed',
        'description': 'Wind speed at which ama lifts (wind from ama side, full sail)',
        'passed': True,  # Always passes - this is informational
        'sail_geometry': {
            'sail_area_m2': round(sail_area_m2, 2),
            'num_sails': num_sails,
            'total_sail_area_m2': round(sail_area_m2 * num_sails, 2),
            'ce_height_m': round(ce_height_m, 2),
        },
        'stability': {
            'max_righting_moment_nm': round(max_righting_moment, 1),
            'max_righting_angle_deg': rm_data.get('max_righting_angle_deg'),
            'capsize_angle_deg': capsize_angle,
        },
        'reference_25_knots': {
            'wind_force_n': round(reference_force, 1),
            'heeling_moment_nm': round(reference_heeling, 1),
            'moment_ratio': round(reference_heeling / max_righting_moment, 2) if max_righting_moment > 0 else None,
        },
        'summary': {
            'ama_lift_windspeed_knots': round(critical_windspeed_knots, 1),
            'result': 'INFO'
        }
    }

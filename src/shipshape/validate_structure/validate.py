"""Run all structural validation tests."""

from typing import Dict, Any

from .aka_analysis import validate_suspended_ama
from .aka_point_load import validate_aka_point_load
from .spine_analysis import validate_one_end_supported
from .mast_analysis import validate_mast
from .brace_analysis import validate_diagonal_braces
from .wave_slam import validate_wave_slam, validate_frontal_wave_slam, validate_sideways_wave_slam
from .lifting_sling import validate_lifting_sling
from .gunwale_analysis import validate_gunwale_loads
from .capsize_analysis import calculate_ama_lift_windspeed


def run_validation(params: Dict[str, Any],
                   mass_data: Dict[str, Any],
                   gz_data: Dict[str, Any] = None,
                   min_safety_factor: float = 2.0,
                   wind_speed_knots: float = 25.0) -> Dict[str, Any]:
    """
    Run all static validation tests.

    Args:
        params: Design parameters
        mass_data: Mass calculation results
        gz_data: GZ curve data (optional, needed for capsize check)
        min_safety_factor: Minimum required safety factor
        wind_speed_knots: Design wind speed for mast test

    Returns:
        Combined validation results
    """
    tests = []

    # Test 1: Suspended ama (aka bending)
    aka_result = validate_suspended_ama(params, mass_data, min_safety_factor)
    tests.append(aka_result)

    # Test 2: Aka point load (crew standing on aka)
    aka_point_result = validate_aka_point_load(params, crew_mass_kg=150.0,
                                                min_safety_factor=min_safety_factor)
    tests.append(aka_point_result)

    # Test 3: One end supported (spine bending)
    spine_result = validate_one_end_supported(params, mass_data, min_safety_factor)
    tests.append(spine_result)

    # Test 4: Mast wind loading
    mast_result = validate_mast(params, mass_data, wind_speed_knots, min_safety_factor)
    tests.append(mast_result)

    # Test 5: Diagonal braces under lateral loading
    brace_result = validate_diagonal_braces(params, mass_data, min_safety_factor)
    tests.append(brace_result)

    # Test 6: Wave slam on ama (vertical, 3 m/s impact, 2.5x dynamic factor)
    wave_slam_result = validate_wave_slam(params, impact_velocity_ms=3.0,
                                           dynamic_factor=2.5,
                                           min_safety_factor=min_safety_factor)
    tests.append(wave_slam_result)

    # Test 7: Frontal wave slam (fore-aft, 3 m/s impact, 2.5x dynamic factor)
    frontal_slam_result = validate_frontal_wave_slam(params, impact_velocity_ms=3.0,
                                                      dynamic_factor=2.5,
                                                      min_safety_factor=min_safety_factor)
    tests.append(frontal_slam_result)

    # Test 8: Sideways wave slam (lateral, 3 m/s impact, 2.5x dynamic factor)
    sideways_slam_result = validate_sideways_wave_slam(params, impact_velocity_ms=3.0,
                                                        dynamic_factor=2.5,
                                                        min_safety_factor=min_safety_factor)
    tests.append(sideways_slam_result)

    # Test 9: Lifting sling (V-sling crane lift)
    lifting_sling_result = validate_lifting_sling(params, mass_data, min_safety_factor)
    tests.append(lifting_sling_result)

    # Test 10: Gunwale load distribution from akas
    gunwale_result = validate_gunwale_loads(params, mass_data, min_safety_factor)
    tests.append(gunwale_result)

    # Test 11: Ama lift wind speed (requires GZ data) - informational only
    if gz_data is not None:
        ama_lift_result = calculate_ama_lift_windspeed(params, gz_data)
        tests.append(ama_lift_result)

    # Overall pass requires all tests to pass
    overall_passed = all(t['passed'] for t in tests)

    return {
        'validator': 'validate-structure',
        'boat_name': params.get('boat_name', 'unknown'),
        'passed': overall_passed,
        'min_safety_factor_required': min_safety_factor,
        'wind_speed_knots': wind_speed_knots,
        'tests': tests,
    }

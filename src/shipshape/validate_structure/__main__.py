#!/usr/bin/env python3
"""
Structural validation for solar proa.

This module runs multiple load case validations including static,
quasi-static (with dynamic factors), and wind loading scenarios.

Each test must pass for the overall validation to pass.
"""

import json
import argparse
import sys
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


def print_aka_report(result: Dict[str, Any]) -> None:
    """Print report for aka/suspended ama test."""
    summary = result['summary']

    print()
    print("-" * 60)
    print(f"TEST: {result['description']}")
    print("-" * 60)

    print(f"\nOutrigger mass: {summary['outrigger_mass_kg']} kg")
    print(f"  Tip loads: {summary['tip_mass_kg']} kg")
    print(f"  Distributed: {summary['distributed_mass_kg']} kg")

    print(f"\nAka: {summary['aka_dimensions']}")
    print(f"  Cantilever length: {summary['cantilever_length_m']:.3f} m")
    print(f"  Number of akas: {summary['num_akas_total']}")

    sa = summary['strong_axis']
    print(f"\nStrong axis (primary):")
    print(f"  Max stress: {sa['max_stress_mpa']:.2f} MPa")
    print(f"  Safety factor: {sa['safety_factor']:.2f}")
    print(f"  Max deflection: {sa['max_deflection_mm']:.1f} mm")
    print(f"  Result: {sa['result']}")

    wa = summary['weak_axis']
    print(f"\nWeak axis (informational):")
    print(f"  Max stress: {wa['max_stress_mpa']:.2f} MPa")
    print(f"  Safety factor: {wa['safety_factor']:.2f}")
    print(f"  Note: {wa['note']}")


def print_aka_point_load_report(result: Dict[str, Any]) -> None:
    """Print report for aka point load (crew standing) test."""
    summary = result['summary']
    geom = result['geometry']
    analysis = result['analysis']

    print()
    print("-" * 60)
    print(f"TEST: {result['description']}")
    print("-" * 60)

    print(f"\nAka: {result['aka_section']['dimensions']}")
    print(f"  Span (vaka to pillar): {geom['span_mm']/1000:.2f} m")
    print(f"  Load position: {geom['load_position']}")

    print(f"\nLoading:")
    print(f"  Crew mass: {result['crew_mass_kg']:.0f} kg")
    print(f"  Point load: {result['loading']['point_load_n']:.0f} N")

    print(f"\nAnalysis (simply supported beam):")
    print(f"  Support reactions: {analysis['support_reaction_n']:.0f} N each")
    print(f"  Max moment: {analysis['max_moment_nm']:.1f} N·m")
    print(f"  Max stress: {analysis['max_stress_mpa']:.2f} MPa")
    print(f"  Max deflection: {analysis['max_deflection_mm']:.1f} mm")

    print(f"\nResult: SF={summary['safety_factor']:.2f} {summary['result']}")


def print_spine_report(result: Dict[str, Any]) -> None:
    """Print report for spine/one-end-supported test."""
    summary = result['summary']
    ama = result['ama_analysis']
    spine = result['spine_analysis']
    geom = result['geometry']

    print()
    print("-" * 60)
    print(f"TEST: {result['description']}")
    print("-" * 60)

    print(f"\nGeometry:")
    print(f"  Ama length: {geom['ama_length_mm']/1000:.2f} m")
    print(f"  Spine length: {geom['spine_length_mm']/1000:.2f} m")
    print(f"  Number of akas: {geom['num_akas']}")

    print(f"\nLoad distribution (elastic model):")
    print(f"  Total outrigger mass: {ama['total_outrigger_mass_kg']} kg")
    print(f"  Fixed support reaction: {ama['fixed_support_reaction_n']:.0f} N")
    print(f"  Aka reactions: {ama['aka_reactions_n']} N")
    print(f"  Aka deflections: {ama['aka_deflections_mm']} mm")

    print(f"\nSpine: {spine['section']}")
    print(f"  Max moment: {spine['max_moment_nm']:.1f} N·m at y={spine['max_moment_position_mm']:.0f} mm")
    print(f"  Max stress: {spine['max_stress_mpa']:.2f} MPa")
    print(f"  Safety factor: {spine['safety_factor']:.2f}")
    print(f"  Result: {summary['result']}")


def print_mast_report(result: Dict[str, Any]) -> None:
    """Print report for mast wind loading test."""
    summary = result['summary']
    geom = result['geometry']
    checks = result['checks']

    print()
    print("-" * 60)
    print(f"TEST: {result['description']}")
    print("-" * 60)

    print(f"\nMast: {geom['mast_diameter_mm']}mm dia x {geom['mast_thickness_mm']}mm wall")
    print(f"  Height: {geom['mast_height_mm']/1000:.2f} m")
    print(f"  Step to partner: {geom['step_to_partner_mm']/1000:.2f} m")
    print(f"  D/t ratio: {geom['d_over_t']:.1f}")

    print(f"\nSail area: {geom['sail_area_m2']:.1f} m²")
    print(f"Wind force: {result['wind_force_n']:.0f} N at {result['wind_speed_knots']:.0f} knots")

    print(f"\nChecks:")
    b = checks['bending_at_partner']
    print(f"  Bending at partner: {b['stress_mpa']:.1f} MPa, SF={b['safety_factor']:.2f} {b['result']}")

    s = checks['shear_at_partner']
    print(f"  Shear at partner:   {s['shear_stress_mpa']:.1f} MPa, SF={s['safety_factor']:.2f} {s['result']}")

    c = checks['column_below_partner']
    print(f"  Column interaction: ratio={c['interaction_ratio']:.3f}, SF={c['safety_factor']:.2f} {c['result']}")

    lb = checks['local_buckling']
    print(f"  Local buckling:     D/t={lb['d_over_t']:.1f} {lb['result']} ({lb['note']})")

    print(f"\nOverall: SF={summary['safety_factor']:.2f} {summary['result']}")


def print_brace_report(result: Dict[str, Any]) -> None:
    """Print report for diagonal brace lateral loading test."""
    summary = result['summary']
    geom = result['brace_geometry']
    loading = result['loading']
    comp = result['compression_check']
    tens = result['tension_check']

    print()
    print("-" * 60)
    print(f"TEST: {result['description']}")
    print("-" * 60)

    print(f"\nBrace: {geom['section']}")
    print(f"  Length: {geom['length_mm']:.0f} mm")
    print(f"  Angle: {geom['angle_deg']:.0f}° from horizontal")
    print(f"  Number of braces: {geom['num_braces']}")

    print(f"\nLateral load (outrigger weight):")
    print(f"  Outrigger mass: {loading['outrigger_mass_kg']:.0f} kg")
    print(f"  Lateral force: {loading['lateral_force_n']:.0f} N")

    print(f"\nCompression check (boat on side, ama above):")
    print(f"  Force per brace: {comp['force_per_brace_n']:.0f} N")
    print(f"  Compressive stress: {comp['compressive_stress_mpa']:.2f} MPa")
    print(f"  Euler buckling stress: {comp['euler_buckling_stress_mpa']:.1f} MPa")
    print(f"  Slenderness ratio: {comp['slenderness_ratio']:.0f}")
    print(f"  Safety factor: {comp['safety_factor']:.2f} ({comp['governing_mode']}) {'PASS' if comp['passed'] else 'FAIL'}")

    print(f"\nTension check (boat inverted, ama dangling):")
    print(f"  Force per brace: {tens['force_per_brace_n']:.0f} N")
    print(f"  Tensile stress: {tens['tensile_stress_mpa']:.2f} MPa")
    print(f"  Safety factor: {tens['safety_factor']:.2f} {'PASS' if tens['passed'] else 'FAIL'}")

    print(f"\nOverall: SF={summary['min_safety_factor']:.2f} ({summary['governing_case']}) {summary['result']}")


def print_wave_slam_report(result: Dict[str, Any]) -> None:
    """Print report for wave slam test."""
    summary = result['summary']
    slam = result['wave_slam']
    aka = result['checks']['aka']
    brace = result['checks']['diagonal_braces']
    spine = result['checks']['spine']

    print()
    print("-" * 60)
    print(f"TEST: {result['description']}")
    print("-" * 60)

    print(f"\nWave slam estimate:")
    print(f"  Impact velocity: {slam['impact_velocity_ms']:.1f} m/s")
    print(f"  Dynamic factor: {slam['dynamic_factor']}x")
    print(f"  Effective area: {slam['effective_area_m2']:.2f} m²")
    print(f"  Slam pressure: {slam['slam_pressure_kpa']:.2f} kPa")
    print(f"  Total slam force: {slam['dynamic_slam_force_n']:.0f} N")

    print(f"\nLoad path: wave → pillars → braces → akas → vaka")

    print(f"\nAka ({aka['model']}):")
    print(f"  Fraction to braces: {aka['fraction_to_braces']:.1%}")
    print(f"  Combined stress: {aka['combined_stress_mpa']:.1f} MPa")
    print(f"  Safety factor: {aka['safety_factor']:.2f} {'PASS' if aka['passed'] else 'FAIL'}")

    print(f"\nDiagonal braces:")
    print(f"  Axial force/brace: {brace['axial_force_per_brace_n']:.0f} N")
    print(f"  Safety factor: {brace['safety_factor']:.2f} ({brace['governing_mode']}) {'PASS' if brace['passed'] else 'FAIL'}")

    print(f"\nSpine ({spine['model']}):")
    print(f"  Span between akas: {spine['avg_span_between_akas_mm']:.0f} mm")
    print(f"  Max stress: {spine['max_stress_mpa']:.1f} MPa")
    print(f"  Safety factor: {spine['safety_factor']:.2f} {'PASS' if spine['passed'] else 'FAIL'}")

    print(f"\nOverall: SF={summary['min_safety_factor']:.2f} ({summary['governing_component']}) {summary['result']}")


def print_frontal_slam_report(result: Dict[str, Any]) -> None:
    """Print report for frontal wave slam test."""
    summary = result['summary']
    slam = result['frontal_slam']
    cb = result['checks']['cross_braces']
    geom = cb['geometry']

    print()
    print("-" * 60)
    print(f"TEST: {result['description']}")
    print("-" * 60)

    print(f"\nFrontal slam estimate:")
    print(f"  Impact velocity: {slam['impact_velocity_ms']:.1f} m/s")
    print(f"  Dynamic factor: {slam['dynamic_factor']}x")
    print(f"  Frontal area: {slam['frontal_area_m2']:.4f} m² (ama cross-section)")
    print(f"  Slam force: {slam['dynamic_slam_force_n']:.0f} N")

    print(f"\nCross-brace geometry:")
    print(f"  Diameter: {geom['brace_diameter_mm']} mm (solid rod)")
    print(f"  Length: {geom['brace_length_mm']:.0f} mm")
    print(f"  Angle: {geom['angle_from_horizontal_deg']:.1f}° from horizontal")
    print(f"  Number of X-pairs: {geom['num_x_brace_pairs']}")

    print(f"\nCross-brace analysis:")
    print(f"  Slenderness ratio: {cb['slenderness_ratio']:.0f}")
    if cb['is_tension_only']:
        print(f"  Mode: Tension-only (compression diagonal buckles)")
    print(f"  Tension force/brace: {cb['tension_force_per_brace_n']:.0f} N")
    print(f"  Yield capacity/brace: {cb['yield_capacity_n']:.0f} N")
    print(f"  Stress: {cb['stress_mpa']:.1f} MPa")
    print(f"  Safety factor: {cb['safety_factor']:.2f} {'PASS' if cb['passed'] else 'FAIL'}")

    print(f"\nOverall: SF={summary['min_safety_factor']:.2f} {summary['result']}")


def print_sideways_slam_report(result: Dict[str, Any]) -> None:
    """Print report for sideways wave slam test."""
    summary = result['summary']
    slam = result['sideways_slam']
    br = result['checks']['diagonal_braces']

    print()
    print("-" * 60)
    print(f"TEST: {result['description']}")
    print("-" * 60)

    print(f"\nSideways slam estimate:")
    print(f"  Impact velocity: {slam['impact_velocity_ms']:.1f} m/s")
    print(f"  Dynamic factor: {slam['dynamic_factor']}x")
    print(f"  Side area: {slam['effective_area_m2']:.2f} m²")
    print(f"  Slam force: {slam['dynamic_slam_force_n']:.0f} N")

    print(f"\nDiagonal pillar braces ({br['brace_section']}):")
    print(f"  Length: {br['brace_length_mm']:.0f} mm at {br['brace_angle_deg']:.0f}°")
    print(f"  Number of braces: {br['num_braces']}")
    print(f"  Axial force/brace: {br['axial_force_per_brace_n']:.0f} N")
    print(f"  Euler buckling load: {br['euler_buckling_load_n']:.0f} N")
    print(f"  Stress: {br['stress_mpa']:.1f} MPa")
    print(f"  Safety factor: {br['safety_factor']:.2f} ({br['governing_mode']}) {'PASS' if br['passed'] else 'FAIL'}")

    print(f"\nOverall: SF={summary['min_safety_factor']:.2f} {summary['result']}")


def print_lifting_sling_report(result: Dict[str, Any]) -> None:
    """Print report for lifting sling (crane lift) test."""
    summary = result['summary']
    geom = result['sling_geometry']
    aka = result['checks']['aka_bending']
    sling_pt = result['checks']['sling_point']
    rope = result['checks']['rope_tension']
    global_bend = result['checks']['global_bending']

    print()
    print("-" * 60)
    print(f"TEST: {result['description']}")
    print("-" * 60)

    print(f"\nV-sling geometry:")
    print(f"  Number of hooks: {geom['num_hooks']}")
    print(f"  Attachment points: {geom['num_attachment_points']} (2 per hook)")
    print(f"  Aka spacing: {geom['aka_spacing_m']:.2f} m")
    print(f"  V-angle from vertical: {geom['v_angle_deg']:.1f}°")
    print(f"  Rope length: {geom['rope_length_m']:.2f} m")

    print(f"\nLoading:")
    print(f"  Total boat mass: {summary['total_mass_kg']:.0f} kg")
    print(f"  Vertical force/attachment: {summary['vertical_force_per_attachment_n']:.0f} N")

    print(f"\nAka bending ({aka['model']}):")
    print(f"  Weight per aka: {aka['weight_per_aka_n']:.0f} N")
    print(f"  Max moment: {aka['max_moment_nm']:.1f} N·m")
    print(f"  Max stress: {aka['max_stress_mpa']:.1f} MPa")
    print(f"  Safety factor: {aka['safety_factor']:.2f} {'PASS' if aka['passed'] else 'FAIL'}")

    print(f"\nSling attachment point:")
    print(f"  Rope tension: {sling_pt['rope_tension_n']:.0f} N (at {sling_pt['v_angle_deg']:.1f}°)")
    print(f"  Contact width: {sling_pt['sling_contact_width_mm']} mm")
    print(f"  Combined stress: {sling_pt['combined_stress_mpa']:.1f} MPa")
    print(f"  Safety factor: {sling_pt['safety_factor']:.2f} {'PASS' if sling_pt['passed'] else 'FAIL'}")

    print(f"\nRope/sling tension:")
    print(f"  Rope tension: {rope['rope_tension_n']:.0f} N")
    print(f"  Sling capacity: {rope['sling_capacity_n']:.0f} N ({rope['note']})")
    print(f"  Safety factor: {rope['safety_factor']:.2f} {'PASS' if rope['passed'] else 'FAIL'}")

    print(f"\nGlobal boat bending ({global_bend['model']}):")
    print(f"  Span between supports: {global_bend['aka_spacing_mm']/1000:.2f} m")
    print(f"  Max stress: {global_bend['max_stress_mpa']:.1f} MPa")
    print(f"  Safety factor: {global_bend['safety_factor']:.2f} {'PASS' if global_bend['passed'] else 'FAIL'}")

    print(f"\nOverall: SF={summary['min_safety_factor']:.2f} ({summary['governing_component']}) {summary['result']}")


def print_gunwale_report(result: Dict[str, Any]) -> None:
    """Print report for gunwale load distribution test."""
    summary = result['summary']
    gunwale = result['gunwale_section']
    loading = result['loading']
    dist = result['distribution']
    bending = result['checks']['bending']
    bearing = result['checks']['bearing']
    bond = result['checks']['bond_shear']
    spacing = result['aka_spacing']

    print()
    print("-" * 60)
    print(f"TEST: {result['description']}")
    print("-" * 60)

    print(f"\nGunwale section: {gunwale['width_mm']:.1f} x {gunwale['height_mm']:.1f} mm")
    print(f"  Material: {gunwale['material']}")

    print(f"\nAka loads at gunwale:")
    print(f"  Suspended ama: {loading['suspended_load_per_aka_n']:.0f} N/aka")
    print(f"  Wave slam: {loading['wave_slam_load_per_aka_n']:.0f} N/aka")
    print(f"  Design load: {loading['design_load_per_aka_n']:.0f} N ({loading['governing_case']})")

    print(f"\nLoad distribution:")
    print(f"  Characteristic length: {dist['characteristic_length_mm']:.0f} mm")
    print(f"  Effective distribution: {dist['effective_distribution_length_mm']:.0f} mm")

    print(f"\nGunwale bending:")
    print(f"  Max moment: {bending['max_moment_nm']:.1f} N·m")
    print(f"  Stress: {bending['bending_stress_mpa']:.2f} MPa (allow {bending['allowable_stress_mpa']} MPa)")
    print(f"  Safety factor: {bending['safety_factor']:.2f} {'PASS' if bending['passed'] else 'FAIL'}")

    print(f"\nBearing at aka ({bearing['note']}):")
    print(f"  Area: {bearing['bearing_area_mm2']:.0f} mm²")
    print(f"  Stress: {bearing['bearing_stress_mpa']:.2f} MPa (allow {bearing['allowable_stress_mpa']} MPa)")
    print(f"  Safety factor: {bearing['safety_factor']:.2f} {'PASS' if bearing['passed'] else 'FAIL'}")

    print(f"\nBond shear ({bond['note']}):")
    print(f"  Bond area: {bond['bond_area_mm2']:.0f} mm²")
    print(f"  Stress: {bond['shear_stress_mpa']:.2f} MPa (allow {bond['allowable_stress_mpa']} MPa)")
    print(f"  Safety factor: {bond['safety_factor']:.2f} {'PASS' if bond['passed'] else 'FAIL'}")

    print(f"\nAka spacing check:")
    print(f"  Spacing: {spacing['aka_spacing_mm']:.0f} mm")
    print(f"  Ratio to distribution: {spacing['spacing_ratio']:.2f}x")
    print(f"  Status: {spacing['interaction']} - {spacing['note']}")

    print(f"\nOverall: SF={summary['min_safety_factor']:.2f} ({summary['governing_component']}) {summary['result']}")


def print_ama_lift_report(result: Dict[str, Any]) -> None:
    """Print report for ama lift wind speed calculation."""
    summary = result['summary']
    sail = result['sail_geometry']
    stab = result['stability']
    ref = result['reference_25_knots']

    print()
    print("-" * 60)
    print(f"INFO: {result['description']}")
    print("-" * 60)

    print(f"\nSail geometry:")
    print(f"  Sail area: {sail['sail_area_m2']:.1f} m² x {sail['num_sails']} sails = {sail['total_sail_area_m2']:.1f} m² total")
    print(f"  CE height: {sail['ce_height_m']:.2f} m above heeling axis")

    print(f"\nStability (from GZ curve, negative heel):")
    print(f"  Max righting moment: {stab['max_righting_moment_nm']:.0f} N·m")
    if stab['max_righting_angle_deg']:
        print(f"  At heel angle: {stab['max_righting_angle_deg']:.1f}°")
    if stab['capsize_angle_deg']:
        print(f"  Capsize angle: {stab['capsize_angle_deg']:.1f}°")

    print(f"\nReference (25 knots from ama side):")
    print(f"  Wind force: {ref['wind_force_n']:.0f} N")
    print(f"  Heeling moment: {ref['heeling_moment_nm']:.0f} N·m")
    if ref['moment_ratio']:
        print(f"  Heeling/Righting ratio: {ref['moment_ratio']:.2f}")

    print(f"\nAma lift wind speed: {summary['ama_lift_windspeed_knots']:.0f} knots")
    print(f"  (wind from ama side, full sail, no crew movement)")


def print_validation_report(results: Dict[str, Any]) -> None:
    """Print a human-readable validation report."""
    print()
    print("=" * 60)
    print("STRUCTURAL VALIDATION")
    print("=" * 60)
    print(f"Boat: {results['boat_name']}")

    # Print individual test reports
    for test in results['tests']:
        if test['test_name'] == 'suspended_ama':
            print_aka_report(test)
        elif test['test_name'] == 'aka_point_load':
            print_aka_point_load_report(test)
        elif test['test_name'] == 'one_end_supported':
            print_spine_report(test)
        elif test['test_name'] == 'mast_wind_loading':
            print_mast_report(test)
        elif test['test_name'] == 'diagonal_braces':
            print_brace_report(test)
        elif test['test_name'] == 'wave_slam':
            print_wave_slam_report(test)
        elif test['test_name'] == 'frontal_wave_slam':
            print_frontal_slam_report(test)
        elif test['test_name'] == 'sideways_wave_slam':
            print_sideways_slam_report(test)
        elif test['test_name'] == 'lifting_sling':
            print_lifting_sling_report(test)
        elif test['test_name'] == 'gunwale_loads':
            print_gunwale_report(test)
        elif test['test_name'] == 'ama_lift_windspeed':
            print_ama_lift_report(test)

    # Overall result
    print()
    print("=" * 60)
    result_str = "PASS" if results['passed'] else "FAIL"
    result_char = "✓" if results['passed'] else "✗"
    print(f"OVERALL RESULT: {result_char} {result_str}")
    print(f"  Required safety factor: {results['min_safety_factor_required']:.1f}")

    for test in results['tests']:
        name = test['test_name']
        passed = test['passed']
        # Get safety factor from appropriate location
        if name == 'suspended_ama':
            sf = test['summary']['strong_axis']['safety_factor']
            status = "✓ PASS" if passed else "✗ FAIL"
            print(f"  {name}: SF={sf:.2f} {status}")
        elif name == 'mast_wind_loading':
            sf = test['summary']['min_safety_factor']
            status = "✓ PASS" if passed else "✗ FAIL"
            print(f"  {name}: SF={sf:.2f} {status}")
        elif name == 'diagonal_braces':
            sf = test['summary']['min_safety_factor']
            status = "✓ PASS" if passed else "✗ FAIL"
            print(f"  {name}: SF={sf:.2f} {status}")
        elif name == 'wave_slam':
            sf = test['summary']['min_safety_factor']
            status = "✓ PASS" if passed else "✗ FAIL"
            print(f"  {name}: SF={sf:.2f} {status}")
        elif name == 'frontal_wave_slam':
            sf = test['summary']['min_safety_factor']
            status = "✓ PASS" if passed else "✗ FAIL"
            print(f"  {name}: SF={sf:.2f} {status}")
        elif name == 'sideways_wave_slam':
            sf = test['summary']['min_safety_factor']
            status = "✓ PASS" if passed else "✗ FAIL"
            print(f"  {name}: SF={sf:.2f} {status}")
        elif name == 'lifting_sling':
            sf = test['summary']['min_safety_factor']
            status = "✓ PASS" if passed else "✗ FAIL"
            print(f"  {name}: SF={sf:.2f} {status}")
        elif name == 'gunwale_loads':
            sf = test['summary']['min_safety_factor']
            status = "✓ PASS" if passed else "✗ FAIL"
            print(f"  {name}: SF={sf:.2f} {status}")
        elif name == 'ama_lift_windspeed':
            wind_speed = test['summary']['ama_lift_windspeed_knots']
            print(f"  {name}: {wind_speed:.0f} knots (INFO)")
        else:
            sf = test['summary']['safety_factor']
            status = "✓ PASS" if passed else "✗ FAIL"
            print(f"  {name}: SF={sf:.2f} {status}")

    print("=" * 60)
    print()


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


def main():
    parser = argparse.ArgumentParser(
        description='Validate structural integrity')
    parser.add_argument('--parameters', required=True,
                        help='Path to parameter JSON file')
    parser.add_argument('--mass', required=True,
                        help='Path to mass JSON file')
    parser.add_argument('--gz',
                        help='Path to GZ curve JSON file (optional, enables capsize check)')
    parser.add_argument('--output', required=True,
                        help='Path to output validation JSON file')
    parser.add_argument('--min-safety-factor', type=float, default=2.0,
                        help='Minimum required safety factor (default: 2.0)')
    parser.add_argument('--wind-speed', type=float, default=25.0,
                        help='Design wind speed in knots for mast test (default: 25)')
    parser.add_argument('--quiet', action='store_true',
                        help='Suppress human-readable output')

    args = parser.parse_args()

    # Load input files
    with open(args.parameters, 'r') as f:
        params = json.load(f)

    with open(args.mass, 'r') as f:
        mass_data = json.load(f)

    # Load GZ data if provided
    gz_data = None
    if args.gz:
        with open(args.gz, 'r') as f:
            gz_data = json.load(f)

    # Run validation
    results = run_validation(params, mass_data, gz_data, args.min_safety_factor, args.wind_speed)

    # Write output
    with open(args.output, 'w') as f:
        json.dump(results, f, indent=2)

    # Print report
    if not args.quiet:
        print_validation_report(results)

    # Exit with appropriate code
    if results['passed']:
        print(f"✓ Validation complete: {args.output}")
        sys.exit(0)
    else:
        print(f"✗ Validation FAILED: {args.output}")
        sys.exit(1)


if __name__ == "__main__":
    main()

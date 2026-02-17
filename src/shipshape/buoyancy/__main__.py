#!/usr/bin/env python3
"""
Buoyancy equilibrium solver - finds the equilibrium pose of a boat.

Uses Newton-Raphson iteration to find the pose (z, pitch, roll) where:
1. Force equilibrium: buoyancy force = weight
2. Moment equilibrium: CoB is directly below CoG (no pitch/roll moments)

Usage:
    python -m shipshape.buoyancy \
        --design artifact/boat.design.FCStd \
        --mass artifact/boat.mass.json \
        --materials constant/material/proa.json \
        --output artifact/boat.buoyancy.json
"""

import sys
import os
import json
import argparse

from shipshape.physics.center_of_buoyancy import load_hull
from shipshape.physics.center_of_mass import compute_center_of_gravity, compute_cog_from_mass_artifact

from .solve import (solve_equilibrium, DEFAULT_MAX_ITERATIONS, DEFAULT_TOLERANCE,
                     DEFAULT_Z_STEP, DEFAULT_ANGLE_STEP)


def main():
    parser = argparse.ArgumentParser(
        description='Find buoyancy equilibrium pose using Newton-Raphson',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('--design', required=True,
                        help='Path to FCStd design file')
    parser.add_argument('--mass', required=False,
                        help='Path to mass.json artifact (optional, faster)')
    parser.add_argument('--materials', required=True,
                        help='Path to materials JSON file')
    parser.add_argument('--output', required=True,
                        help='Path to output JSON file')
    parser.add_argument('--max-iterations', type=int, default=DEFAULT_MAX_ITERATIONS,
                        help=f'Maximum iterations (default: {DEFAULT_MAX_ITERATIONS})')
    parser.add_argument('--tolerance', type=float, default=DEFAULT_TOLERANCE,
                        help=f'Convergence tolerance (default: {DEFAULT_TOLERANCE})')
    parser.add_argument('--z-step', type=float, default=DEFAULT_Z_STEP,
                        help=f'Jacobian z perturbation in mm (default: {DEFAULT_Z_STEP})')
    parser.add_argument('--angle-step', type=float, default=DEFAULT_ANGLE_STEP,
                        help=f'Jacobian angle perturbation in degrees (default: {DEFAULT_ANGLE_STEP})')
    parser.add_argument('--hull-groups',
                        help='Path to hull groups JSON file or inline JSON string '
                             '(maps group names to pattern lists)')
    parser.add_argument('--quiet', action='store_true',
                        help='Suppress progress output')

    args = parser.parse_args()

    if not os.path.exists(args.design):
        print(f"ERROR: Design file not found: {args.design}", file=sys.stderr)
        sys.exit(1)

    verbose = not args.quiet

    if verbose:
        print(f"Solving buoyancy equilibrium: {args.design}")

    # Compute center of gravity
    if verbose:
        print("  Computing center of gravity...")

    with open(args.materials, 'r') as f:
        materials = json.load(f)

    if args.mass and os.path.exists(args.mass):
        with open(args.mass, 'r') as f:
            mass_data = json.load(f)
        cog_result = compute_cog_from_mass_artifact(mass_data, args.design)
    else:
        cog_result = compute_center_of_gravity(args.design, materials)

    if verbose:
        print(f"  CoG: ({cog_result['CoG']['x']:.1f}, {cog_result['CoG']['y']:.1f}, "
              f"{cog_result['CoG']['z']:.1f}) mm")
        print(f"  Total mass: {cog_result['total_mass_kg']:.2f} kg")
        print(f"  Weight: {cog_result['weight_N']:.2f} N")

    # Parse hull groups
    hull_groups = None
    if args.hull_groups:
        if os.path.isfile(args.hull_groups):
            with open(args.hull_groups, 'r') as f:
                hull_groups = json.load(f)
        else:
            hull_groups = json.loads(args.hull_groups)
        if verbose:
            print(f"  Hull groups: {list(hull_groups.keys())}")

    # Load hull geometry (once, reused by solver)
    if verbose:
        print("  Loading hull geometry...")
    hull = load_hull(args.design, hull_groups=hull_groups)

    # Solve equilibrium
    if verbose:
        print("  Running Newton-Raphson solver...")

    result = solve_equilibrium(
        hull,
        cog_result,
        max_iterations=args.max_iterations,
        tolerance=args.tolerance,
        z_step=args.z_step,
        angle_step=args.angle_step,
        verbose=verbose
    )

    # Add validator field
    result['validator'] = 'buoyancy'

    # Write output
    os.makedirs(os.path.dirname(args.output) or '.', exist_ok=True)
    with open(args.output, 'w') as f:
        json.dump(result, f, indent=2)

    if verbose:
        print(f"✓ Buoyancy equilibrium {'found' if result['converged'] else 'NOT CONVERGED'}")
        eq = result['equilibrium']
        print(f"  Equilibrium pose:")
        print(f"    z offset: {eq['z_offset_mm']:.2f} mm")
        print(f"    pitch: {eq['pitch_deg']:.4f}°")
        print(f"    roll: {eq['roll_deg']:.4f}°")
        cog_w = result['center_of_gravity_world']
        cob = result['center_of_buoyancy']
        print(f"  CoG (world): ({cog_w['x']:.1f}, {cog_w['y']:.1f}, {cog_w['z']:.1f}) mm")
        print(f"  CoB (world): ({cob['x']:.1f}, {cob['y']:.1f}, {cob['z']:.1f}) mm")
        print(f"  Submerged volume: {result['submerged_volume_liters']:.2f} liters")
        for gname, gdata in result['hull_groups'].items():
            print(f"    {gname}: {gdata['submerged_volume_liters']:.1f}L / "
                  f"{gdata['total_volume_liters']:.1f}L "
                  f"({gdata['submerged_percent']:.1f}%) "
                  f"@ z={gdata['z_world_mm']:.0f}mm")
        print(f"  Buoyancy force: {result['buoyancy_force_N']:.2f} N")
        print(f"  Output: {args.output}")

    # Exit with error if not converged
    if not result['converged']:
        sys.exit(1)


if __name__ == "__main__":
    main()

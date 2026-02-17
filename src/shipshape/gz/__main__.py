#!/usr/bin/env python3
"""
GZ curve computation - calculates the righting arm curve for stability analysis.

The GZ (righting arm) curve shows how the boat's righting moment varies with heel angle.
For each heel angle:
1. Find equilibrium z (where buoyancy = weight) with fixed roll
2. Compute GZ = horizontal distance between CoB and CoG (transverse)
3. Righting moment = GZ x displacement x g

Usage:
    python -m shipshape.gz \
        --design artifact/boat.design.FCStd \
        --buoyancy artifact/boat.buoyancy.json \
        --output artifact/boat.gz.json \
        --output-png artifact/boat.gz.png
"""

import sys
import os
import json
import argparse

try:
    import FreeCAD as App
except ImportError as e:
    print(f"ERROR: {e}", file=sys.stderr)
    print("This script requires FreeCAD (conda-forge or bundled)", file=sys.stderr)
    sys.exit(1)

from shipshape.physics.center_of_buoyancy import load_hull
from .compute import compute_gz_curve, plot_gz_curve


def main():
    parser = argparse.ArgumentParser(
        description='Compute GZ (righting arm) curve for stability analysis',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('--design', required=True,
                        help='Path to FCStd design file')
    parser.add_argument('--buoyancy', required=True,
                        help='Path to buoyancy.json artifact')
    parser.add_argument('--parameters',
                        help='Path to parameter.json artifact (for LOA/beam, used in period estimates)')
    parser.add_argument('--output', required=True,
                        help='Path to output JSON file')
    parser.add_argument('--output-png',
                        help='Path to output PNG plot (optional, defaults to same name as output with .png)')
    parser.add_argument('--min-heel', type=float, default=-60.0,
                        help='Minimum heel angle in degrees (default: -60)')
    parser.add_argument('--max-heel', type=float, default=65.0,
                        help='Maximum heel angle in degrees (default: 65)')
    parser.add_argument('--heel-step', type=float, default=5.0,
                        help='Heel angle step in degrees (default: 5)')
    parser.add_argument('--hull-groups',
                        help='Path to hull groups JSON file or inline JSON string '
                             '(maps group names to pattern lists)')
    parser.add_argument('--quiet', action='store_true',
                        help='Suppress progress output')

    args = parser.parse_args()

    if not os.path.exists(args.design):
        print(f"ERROR: Design file not found: {args.design}", file=sys.stderr)
        sys.exit(1)

    if not os.path.exists(args.buoyancy):
        print(f"ERROR: Buoyancy file not found: {args.buoyancy}", file=sys.stderr)
        sys.exit(1)

    verbose = not args.quiet

    if verbose:
        print(f"Computing GZ curve: {args.design}")

    # Load buoyancy result
    with open(args.buoyancy) as f:
        buoyancy_result = json.load(f)

    if verbose:
        eq = buoyancy_result['equilibrium']
        print(f"  Equilibrium pose: z={eq['z_offset_mm']:.1f}mm, "
              f"pitch={eq['pitch_deg']:.2f}°, roll={eq['roll_deg']:.2f}°")
        print(f"  Mass: {buoyancy_result['total_mass_kg']:.1f} kg")

    # Generate heel angles with fine steps near 0°
    heel_angles = []
    angle = args.min_heel
    while angle <= args.max_heel + 0.01:
        heel_angles.append(angle)
        if -6 < angle < 5:
            angle += 1.0
        else:
            angle += args.heel_step

    if verbose:
        print(f"  Computing {len(heel_angles)} points from {args.min_heel}° to {args.max_heel}°")

    # Load parameters for period estimates (optional)
    beam_m = None
    loa_m = None
    if args.parameters and os.path.exists(args.parameters):
        with open(args.parameters) as f:
            params = json.load(f)
        beam_m = params.get('beam', 0) / 1000.0
        loa_m = params.get('loa', params.get('vaka_length', params.get('ama_length', 0))) / 1000.0
        if verbose and beam_m and loa_m:
            print(f"  Dimensions: LOA={loa_m:.1f}m, beam={beam_m:.1f}m")

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

    # Load hull geometry (once, reused across all heel angles)
    if verbose:
        print("  Loading hull geometry...")
    hull = load_hull(args.design, hull_groups=hull_groups)

    # Compute GZ curve
    result = compute_gz_curve(
        hull,
        buoyancy_result,
        heel_angles=heel_angles,
        beam_m=beam_m,
        loa_m=loa_m,
        verbose=verbose
    )

    # Write JSON output
    os.makedirs(os.path.dirname(args.output) or '.', exist_ok=True)
    with open(args.output, 'w') as f:
        json.dump(result, f, indent=2)

    if verbose:
        print(f"✓ GZ curve data saved to {args.output}")
        summary = result['summary']
        print(f"  Max GZ: {summary['max_gz_m']*100:.1f} cm at {summary['max_gz_angle_deg']}°")
        print(f"  Range of positive stability: {summary['range_of_positive_stability_deg']}°")
        if summary.get('turtle_angle_deg'):
            print(f"  Turtle angle (ama down): {summary['turtle_angle_deg']:.1f}°")
        if summary.get('capsize_angle_deg'):
            print(f"  Capsize angle (ama up): {summary['capsize_angle_deg']:.1f}°")
        if summary.get('gm_m'):
            print(f"  GM (transverse): {summary['gm_m']*100:.1f} cm")

    # Generate PNG plot
    png_path = args.output_png
    if not png_path:
        png_path = args.output.replace('.json', '.png')

    plot_gz_curve(result, png_path)


if __name__ == "__main__":
    main()

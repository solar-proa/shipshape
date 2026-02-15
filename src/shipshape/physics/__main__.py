#!/usr/bin/env python3
"""
Physics computation CLI - computes center of gravity and center of buoyancy.

Usage:
    # Compute center of gravity
    python -m shipshape.physics cog --design artifact/boat.design.FCStd \
                              --materials constant/material/proa.json \
                              --output artifact/boat.cog.json

    # Compute center of buoyancy at a specific pose
    python -m shipshape.physics cob --design artifact/boat.design.FCStd \
                              --z -100 --pitch 2.0 --roll 0.5 \
                              --output artifact/boat.cob.json
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

from shipshape.physics.center_of_mass import compute_center_of_gravity, compute_cog_from_mass_artifact
from shipshape.physics.center_of_buoyancy import compute_center_of_buoyancy


def cmd_cog(args):
    """Compute center of gravity."""
    print(f"Computing center of gravity: {args.design}")

    if args.mass_artifact:
        # Use precomputed mass data
        print(f"  Using mass artifact: {args.mass_artifact}")
        result = compute_cog_from_mass_artifact(args.mass_artifact, args.design)
    else:
        # Compute from geometry + materials
        if not args.materials:
            print("ERROR: --materials required when not using --mass-artifact", file=sys.stderr)
            sys.exit(1)
        print(f"  Materials: {args.materials}")
        result = compute_center_of_gravity(args.design, args.materials)

    # Add validator field for pipeline compatibility
    result['validator'] = 'cog'

    # Write output
    os.makedirs(os.path.dirname(args.output) or '.', exist_ok=True)
    with open(args.output, 'w') as f:
        json.dump(result, f, indent=2)

    print(f"✓ Center of gravity computed")
    print(f"  CoG: ({result['CoG']['x']:.1f}, {result['CoG']['y']:.1f}, {result['CoG']['z']:.1f}) mm")
    print(f"  Total mass: {result['total_mass_kg']:.2f} kg")
    print(f"  Weight: {result['weight_N']:.2f} N")
    print(f"  Components: {result['component_count']}")
    print(f"  Output: {args.output}")


def cmd_cob(args):
    """Compute center of buoyancy."""
    print(f"Computing center of buoyancy: {args.design}")
    print(f"  Pose: z={args.z} mm, pitch={args.pitch}°, roll={args.roll}°")

    if args.hull_components:
        hull_components = [c.strip() for c in args.hull_components.split(',')]
        print(f"  Hull components: {hull_components}")
    else:
        hull_components = None  # Use default ["vaka", "ama"]

    result = compute_center_of_buoyancy(
        fcstd_path=args.design,
        z_displacement=args.z,
        pitch_deg=args.pitch,
        roll_deg=args.roll,
        water_level_z=args.water_level,
        hull_components=hull_components
    )

    # Add validator field for pipeline compatibility
    result['validator'] = 'cob'

    # Write output
    os.makedirs(os.path.dirname(args.output) or '.', exist_ok=True)
    with open(args.output, 'w') as f:
        json.dump(result, f, indent=2)

    print(f"✓ Center of buoyancy computed")
    print(f"  CoB: ({result['CoB']['x']:.1f}, {result['CoB']['y']:.1f}, {result['CoB']['z']:.1f}) mm")
    print(f"  Submerged volume: {result['submerged_volume_liters']:.2f} liters")
    print(f"  Buoyancy force: {result['buoyancy_force_N']:.2f} N")
    print(f"  Displacement: {result['displacement_kg']:.2f} kg")
    print(f"  Output: {args.output}")


def main():
    parser = argparse.ArgumentParser(
        description='Physics computations for hydrostatic analysis',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    subparsers = parser.add_subparsers(dest='command', help='Available commands')

    # CoG subcommand
    cog_parser = subparsers.add_parser('cog', help='Compute center of gravity')
    cog_parser.add_argument('--design', required=True, help='Path to FCStd design file')
    cog_parser.add_argument('--materials', help='Path to materials JSON file')
    cog_parser.add_argument('--mass-artifact', help='Path to precomputed mass.json (alternative to --materials)')
    cog_parser.add_argument('--output', required=True, help='Path to output JSON file')

    # CoB subcommand
    cob_parser = subparsers.add_parser('cob', help='Compute center of buoyancy')
    cob_parser.add_argument('--design', required=True, help='Path to FCStd design file')
    cob_parser.add_argument('--z', type=float, default=0.0, help='Z displacement in mm (negative = sink)')
    cob_parser.add_argument('--pitch', type=float, default=0.0, help='Pitch angle in degrees (positive = bow up)')
    cob_parser.add_argument('--roll', type=float, default=0.0, help='Roll angle in degrees (positive = starboard down)')
    cob_parser.add_argument('--water-level', type=float, default=0.0, help='Water surface Z coordinate in mm')
    cob_parser.add_argument('--hull-components', help='Comma-separated list of hull component patterns (default: vaka,ama)')
    cob_parser.add_argument('--output', required=True, help='Path to output JSON file')

    args = parser.parse_args()

    if args.command == 'cog':
        cmd_cog(args)
    elif args.command == 'cob':
        cmd_cob(args)
    else:
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()

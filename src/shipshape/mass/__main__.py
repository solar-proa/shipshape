#!/usr/bin/env python3
"""
Mass analysis validator - computes mass, volume, and component breakdown from FreeCAD model.
Outputs JSON artifact for downstream validators.
"""

import sys
import os
import json
import argparse

from .analyze import analyze_mass


def main():
    parser = argparse.ArgumentParser(description='Analyze mass properties of FreeCAD model')
    parser.add_argument('--design', required=True, help='Path to FCStd design file')
    parser.add_argument('--materials', required=True, help='Path to materials JSON file')
    parser.add_argument('--output', required=True, help='Path to output JSON artifact')

    args = parser.parse_args()

    if not os.path.exists(args.design):
        print(f"ERROR: Design file not found: {args.design}", file=sys.stderr)
        sys.exit(1)

    print(f"Analyzing mass properties: {args.design}")
    with open(args.materials, 'r') as f:
        materials = json.load(f)
    result = analyze_mass(args.design, materials)

    # Write JSON output
    os.makedirs(os.path.dirname(args.output) or '.', exist_ok=True)
    with open(args.output, 'w') as f:
        json.dump(result, f, indent=2)

    print(f"✓ Mass analysis complete")
    print(f"  Total mass: {result['total_mass_kg']:.2f} kg")
    print(f"  Components: {result['component_count']}")
    print(f"  Output: {args.output}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""CLI entry point for parameter computation."""

import importlib
import json
import os
import argparse


def main():
    parser = argparse.ArgumentParser(description='Compute parameters')
    parser.add_argument('--compute', required=True,
                        help='Dotted module path providing compute_derived()')
    parser.add_argument('--boat', required=True, help='Path to boat constants')
    parser.add_argument('--configuration', required=True, help='Path to configuration constants')
    parser.add_argument('--output', required=True, help='Path to output JSON artifact')

    args = parser.parse_args()

    module = importlib.import_module(args.compute)

    with open(args.boat, 'r') as b:
        boat_data = json.load(b)

    with open(args.configuration, 'r') as c:
        configuration_data = json.load(c)

    data = boat_data | configuration_data
    data = module.compute_derived(data)

    os.makedirs(os.path.dirname(args.output) or '.', exist_ok=True)
    with open(args.output, 'w') as f:
        json.dump(data, f, indent=2)

    print(f"✓ Parameters complete")
    print(f"  Output: {args.output}")


if __name__ == "__main__":
    main()

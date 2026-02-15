#!/usr/bin/env python3
"""
Compute derived parameters from base parameters.
This module loads base parameters from JSON and computes all derived values.
"""

import json
import os
import argparse
from typing import Dict, Any

def compute_derived(base: Dict[str, Any]) -> Dict[str, Any]:
    """
    Compute all derived parameters from base parameters.
    Returns a complete parameter dictionary with both base and derived values.
    """
    params = base.copy()
    
    # Constants
    mm_in_one_inch = 25.4
    
    # Derived dimensions
    params['mm_in_one_inch'] = mm_in_one_inch
    params['stringer_width'] = base['stringer_width_inches'] * mm_in_one_inch
    params['clamp_width'] = base['clamp_width_inches'] * mm_in_one_inch
    params['clamp_height'] = base['clamp_height_inches'] * mm_in_one_inch
    params['vaka_stringer_width'] = base['vaka_stringer_width_inches'] * mm_in_one_inch
    params['vaka_stringer_height'] = base['vaka_stringer_height_inches'] * mm_in_one_inch
    params['frame_width'] = base['frame_width_inches'] * mm_in_one_inch
    params['frame_depth'] = base['frame_depth_inches'] * mm_in_one_inch
    params['bottom_height'] = base['bottom_height_inches'] * mm_in_one_inch
    
    # Aka length depends on panels and deck
    params['aka_length'] = (base['panel_length'] * base['panels_transversal'] + 
                            base['deck_width'])
    
    # Bottom thickness same as vaka
    params['bottom_thickness'] = base['vaka_thickness']
    
    # Crossdeck dimensions
    params['crossdeck_width'] = base['panel_width'] / base['akas_per_panel']
    params['crossdeck_thickness'] = base['deck_thickness']
    params['crossdeck_length'] = (base['panels_transversal'] * base['panel_length'] +
                                  (base['deck_width'] - base['vaka_width']) / 2 +
                                  params['stringer_width'])
    
    # Cockpit length: distance from center to first aka's inner edge, doubled
    # First aka Y position depends on akas_per_panel:
    # - Single aka: centered in panel at crossdeck_width/2 + panel_width/2
    # - Multiple akas: at rim distance from panel edge at crossdeck_width/2 + aka_rim
    if base.get('akas_per_panel', 1) == 1:
        first_aka_y = params['crossdeck_width'] / 2 + base['panel_width'] / 2
    else:
        first_aka_y = params['crossdeck_width'] / 2 + base['aka_rim']
    params['cockpit_length'] = 2 * first_aka_y - base['aka_width']
    
    # Panel stringer calculations
    params['panel_stringer_offset'] = (base['panel_length'] / 4 - 
                                       params['stringer_width'] / 2)
    params['panel_stringer_length'] = (params['crossdeck_width'] + 
                                       base['panels_longitudinal'] * base['panel_width'])
    
    # Vertical levels (build up from bottom)
    params['clamp_base_level'] = (params['bottom_height'] + base['freeboard'] - 
                                    params['clamp_height'])
    params['vaka_stringer_base_level'] = params['clamp_base_level'] - params['freeboard'] / 2
    params['overhead_base_level'] = (params['clamp_base_level'] + 
                                     params['clamp_height'])
    params['aka_base_level'] = (params['overhead_base_level'] + 
                                base['overhead_thickness'])
    params['stringer_base_level'] = params['aka_base_level'] + base['aka_height']
    params['panel_base_level'] = params['stringer_base_level'] + params['stringer_width']
    params['deck_base_level'] = params['panel_base_level']
    params['deck_level'] = params['deck_base_level'] + base['deck_thickness']
    
    # Spine (uses same sizes as aka)
    params['spine_thickness'] = base['aka_thickness']
    params['spine_width'] = base['aka_width']
    params['spine_base_level'] = params['aka_base_level'] - params['spine_width']
    params['spine_length'] = (base['panel_width'] * base['panels_longitudinal'] + 
                              params['crossdeck_width'] + base['spine_length_extension'])
    
    # Beam calculation
    params['beam'] = (params['aka_length'] + base['aka_cap_thickness'] - 
                     params['spine_width'] + base['ama_diameter'] / 2)
    
    # Pillar (uses same sizes as aka)
    params['pillar_thickness'] = base['aka_thickness']
    params['pillar_width'] = base['aka_width']
    params['pillar_height'] = params['spine_base_level'] - base['ama_thickness']
    
    # Ama cone length: cone starts at outer edge of outermost pillar
    # Calculate Y position of the outermost (last) aka
    last_panel_index = base['panels_longitudinal'] // 2 - 1
    last_aka_index = base.get('akas_per_panel', 1) - 1
    if base.get('akas_per_panel', 1) == 1:
        last_aka_y = (params['crossdeck_width'] / 2
                      + last_panel_index * base['panel_width']
                      + base['panel_width'] / 2)
    else:
        aka_spacing = ((base['panel_width'] - 2 * base['aka_rim'])
                       / (base['akas_per_panel'] - 1))
        last_aka_y = (params['crossdeck_width'] / 2
                      + last_panel_index * base['panel_width']
                      + base['aka_rim'] + last_aka_index * aka_spacing)
    
    # Vaka x offset (distance from ama centerline to vaka centerline)
    params['vaka_x_offset'] = (- params['pillar_width'] / 2
                               + base['panel_length'] * base['panels_transversal']
                               + base['deck_width'] / 2)
    
    # Mast calculations
    params['mast_distance_from_center'] = (base['vaka_length'] / 4 + 
                                           base['mast_distance_from_center_offset'])
    params['mast_base_level'] = params['bottom_height'] + base['sole_thickness']
    params['mast_partner_length'] = (base['vaka_width'] - 
                                     base['mast_partner_vaka_clearance'])
    params['mast_partner_width'] = (base['mast_diameter'] + 
                                    base['mast_partner_width_offset'])
    params['mast_step_outer_diameter'] = (base['mast_diameter'] + 
                                          base['mast_step_diameter_offset'])
    params['mast_step_inner_diameter'] = base['mast_diameter']
    
    # Yard spar height
    params['yard_spar_height'] = (base['mast_height'] -
                                  base['yard_spar_distance_from_top'])

    # Boom length matches yard length (rectangular sails)
    params['boom_length'] = base['yard_length']
    params['sail_area_m2'] = (2 * base['sail_height'] / 1000
                              * base['sail_width'] / 1000)

    # Rudder calculations
    params['rudder_bearing_block_height'] = params['stringer_width']
    params['rudder_vaka_mount_base_level'] = ((params['bottom_height'] + 
                                               base['freeboard']) / 2)
    params['rudder_rib_length'] = (base['rudder_blade_length'] - 
                                   base['rudder_rib_clearance'])
    
    # Tiller dimensions (uses stringer sizes)
    params['tiller_width'] = params['stringer_width']
    params['tiller_thickness'] = base['stringer_thickness']
    params['tiller_length'] = 490  # This was hardcoded in original
    
    return params

def main():
    parser = argparse.ArgumentParser(description='Compute parameters')
    parser.add_argument('--boat', required=True, help='Path to boat constants')
    parser.add_argument('--configuration', required=True, help='Path to configuration constants')
    parser.add_argument('--output', required=True, help='Path to output JSON artifact')
    
    args = parser.parse_args()

    # Load boat parameters
    with open(args.boat, 'r') as b:
        boat_data = json.load(b)
    
    with open(args.configuration, 'r') as c:
        configuration_data = json.load(c)

    data = boat_data | configuration_data
    
    data = compute_derived(data)


    
    # Write JSON output
    os.makedirs(os.path.dirname(args.output) or '.', exist_ok=True)
    with open(args.output, 'w') as f:
        json.dump(data, f, indent=2)
    
    print(f"✓ Parameters complete")
    print(f"  Output: {args.output}")

if __name__ == "__main__":
    main()

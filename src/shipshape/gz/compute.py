#!/usr/bin/env python3
"""
GZ curve computation - calculates the righting arm curve for stability analysis.

The GZ (righting arm) curve shows how the boat's righting moment varies with heel angle.
For each heel angle:
1. Find equilibrium z (where buoyancy = weight) with fixed roll
2. Compute GZ = horizontal distance between CoB and CoG (transverse)
3. Righting moment = GZ x displacement x g

For a proa with outrigger (ama), the stability is asymmetric:
- Positive roll (towards ama): high stability due to ama leverage
- Negative roll (away from ama): lower stability, similar to monohull
"""

import math

from shipshape.physics.center_of_buoyancy import compute_center_of_buoyancy

# Physical constants
GRAVITY_M_S2 = 9.81
SEAWATER_DENSITY_KG_M3 = 1025.0


def estimate_natural_periods(mass_kg: float, submerged_volume_liters: float,
                              draft_m: float, beam_m: float, loa_m: float) -> dict:
    """
    Estimate natural periods using simple empirical formulas.

    These are APPROXIMATE values suitable for initial design. For proas and other
    asymmetric hulls, the standard linearized formulas don't apply well.
    Physical testing is recommended for accurate values.

    Args:
        mass_kg: Total displacement mass
        submerged_volume_liters: Submerged volume at equilibrium
        draft_m: Draft (depth below waterline)
        beam_m: Overall beam (including ama)
        loa_m: Length overall

    Returns:
        Dictionary with estimated periods and notes
    """
    result = {}

    # Estimate waterplane area from submerged volume and draft
    submerged_volume_m3 = submerged_volume_liters / 1000.0
    if draft_m > 0.05:
        waterplane_area_m2 = 0.7 * submerged_volume_m3 / draft_m
    else:
        waterplane_area_m2 = None

    # Heave period: T = 2pi*sqrt(m / rho*g*A_wp)
    if waterplane_area_m2 and waterplane_area_m2 > 0:
        heave_stiffness = SEAWATER_DENSITY_KG_M3 * GRAVITY_M_S2 * waterplane_area_m2
        t_heave = 2 * math.pi * math.sqrt(mass_kg / heave_stiffness)
        result['heave_period_s'] = round(t_heave, 1)
    else:
        result['heave_period_s'] = None

    # Roll period: empirical estimate for wide-beam multihulls
    if beam_m > 0:
        t_roll = 0.35 * beam_m
        result['roll_period_s'] = round(t_roll, 1)
    else:
        result['roll_period_s'] = None

    # Pitch period: empirical estimate based on LOA
    if loa_m > 0:
        t_pitch = 0.4 * math.sqrt(loa_m) + 1.5
        result['pitch_period_s'] = round(t_pitch, 1)
    else:
        result['pitch_period_s'] = None

    result['waterplane_area_m2'] = round(waterplane_area_m2, 2) if waterplane_area_m2 else None
    result['note'] = 'Approximate values - physical testing recommended for proas'

    return result


def transform_point(point: dict, z_displacement: float, pitch_deg: float,
                    roll_deg: float, rotation_center: dict) -> dict:
    """
    Transform a point by z displacement and pitch/roll rotations.
    Same transformation as in buoyancy solver.
    """
    x = point['x'] - rotation_center['x']
    y = point['y'] - rotation_center['y']
    z = point['z'] - rotation_center['z']

    pitch_rad = math.radians(pitch_deg)
    roll_rad = math.radians(roll_deg)

    cos_p = math.cos(pitch_rad)
    sin_p = math.sin(pitch_rad)
    cos_r = math.cos(roll_rad)
    sin_r = math.sin(roll_rad)

    # R = Ry(roll) * Rx(pitch)
    x_new = cos_r * x + sin_r * sin_p * y + sin_r * cos_p * z
    y_new = cos_p * y - sin_p * z
    z_new = -sin_r * x + cos_r * sin_p * y + cos_r * cos_p * z

    return {
        'x': x_new + rotation_center['x'],
        'y': y_new + rotation_center['y'],
        'z': z_new + rotation_center['z'] + z_displacement
    }


def compute_gm_from_gz_curve(gz_data: list, equilibrium_roll_deg: float = 0.0) -> dict:
    """
    Compute metacentric heights from GZ curve data.

    GM (transverse) = dGZ/d_theta at equilibrium (theta in radians)

    Args:
        gz_data: List of GZ curve points from compute_gz_curve
        equilibrium_roll_deg: Equilibrium roll angle

    Returns:
        Dictionary with GM values
    """
    converged = [p for p in gz_data if p.get('converged', False)]
    if len(converged) < 2:
        return {'gm_m': None, 'gm_longitudinal_m': None}

    # Sort by angle
    converged.sort(key=lambda p: p['heel_deg'])

    # Get points on negative side (away from ama, more linear behavior)
    negative_points = [p for p in converged if p['heel_deg'] < 0]

    gm_m = None

    if len(negative_points) >= 2:
        negative_points.sort(key=lambda p: p['heel_deg'], reverse=True)  # Closest to 0 first
        p1, p2 = negative_points[0], negative_points[1]

        d_angle_rad = math.radians(p1['heel_deg'] - p2['heel_deg'])
        d_gz_m = (p1.get('raw_gz_mm', p1['gz_mm']) - p2.get('raw_gz_mm', p2['gz_mm'])) / 1000.0

        if abs(d_angle_rad) > 0.001:
            gm_m = abs(d_gz_m / d_angle_rad)

    # Fallback: try positive side if negative didn't work
    if gm_m is None:
        positive_points = [p for p in converged if p['heel_deg'] > 0]
        if len(positive_points) >= 2:
            positive_points.sort(key=lambda p: p['heel_deg'])  # Closest to 0 first
            p1, p2 = positive_points[0], positive_points[1]

            d_angle_rad = math.radians(p2['heel_deg'] - p1['heel_deg'])
            d_gz_m = (p2.get('raw_gz_mm', p2['gz_mm']) - p1.get('raw_gz_mm', p1['gz_mm'])) / 1000.0

            if abs(d_angle_rad) > 0.001:
                gm_m = abs(d_gz_m / d_angle_rad)

    # Sanity check
    if gm_m is not None and not (0.1 < gm_m < 100.0):
        gm_m = None

    # For longitudinal GM, estimate as 8x GM_T
    gm_longitudinal_m = gm_m * 8.0 if gm_m else None

    return {
        'gm_m': round(gm_m, 4) if gm_m else None,
        'gm_longitudinal_m': round(gm_longitudinal_m, 4) if gm_longitudinal_m else None
    }


def find_equilibrium_z_at_heel(hull: dict, target_weight_N: float,
                               pitch_deg: float, roll_deg: float,
                               z_initial: float = -500.0,
                               tolerance: float = 0.01,
                               max_iterations: int = 30) -> dict:
    """
    Find equilibrium z displacement at a fixed heel (roll) angle.

    Uses bisection to find z where buoyancy = weight.

    Args:
        hull: Hull data from load_hull()
        target_weight_N: Target weight (buoyancy must equal this)
        pitch_deg: Pitch angle (usually 0 for GZ curve)
        roll_deg: Roll (heel) angle
        z_initial: Initial guess for z
        tolerance: Relative tolerance for force balance
        max_iterations: Maximum iterations

    Returns:
        Dictionary with equilibrium z and CoB result
    """
    # Binary search bounds
    z_min, z_max = -5000.0, 500.0  # mm

    # First, bracket the solution
    cob_min = compute_center_of_buoyancy(hull, z_min, pitch_deg, roll_deg)
    cob_max = compute_center_of_buoyancy(hull, z_max, pitch_deg, roll_deg)

    buoyancy_min = cob_min['buoyancy_force_N']
    buoyancy_max = cob_max['buoyancy_force_N']

    # Check if target is within range
    if target_weight_N > buoyancy_min:
        return {
            'converged': False,
            'z_mm': z_min,
            'cob_result': cob_min,
            'error': 'Boat too heavy - cannot float'
        }

    if target_weight_N < buoyancy_max:
        return {
            'converged': False,
            'z_mm': z_max,
            'cob_result': cob_max,
            'error': 'Boat too light - pops out of water'
        }

    # Bisection search
    for iteration in range(max_iterations):
        z_mid = (z_min + z_max) / 2
        cob_mid = compute_center_of_buoyancy(hull, z_mid, pitch_deg, roll_deg)
        buoyancy_mid = cob_mid['buoyancy_force_N']

        force_error = abs(buoyancy_mid - target_weight_N) / target_weight_N

        if force_error < tolerance:
            return {
                'converged': True,
                'z_mm': z_mid,
                'cob_result': cob_mid,
                'iterations': iteration + 1
            }

        if buoyancy_mid > target_weight_N:
            # Too much buoyancy, rise up (less negative z)
            z_min = z_mid
        else:
            # Not enough buoyancy, sink more (more negative z)
            z_max = z_mid

    # Return best result even if not fully converged
    return {
        'converged': False,
        'z_mm': z_mid,
        'cob_result': cob_mid,
        'iterations': max_iterations,
        'error': f'Did not converge, force error: {force_error:.4f}'
    }


def compute_gz_curve(hull: dict, buoyancy_result: dict,
                     heel_angles: list = None,
                     beam_m: float = None, loa_m: float = None,
                     verbose: bool = True) -> dict:
    """
    Compute the GZ curve by sweeping through heel angles.

    For each heel angle:
    1. Find equilibrium z (buoyancy = weight)
    2. Transform CoG to world frame
    3. Compute GZ = CoB_x - CoG_x (transverse separation)

    Args:
        hull: Hull data from load_hull()
        buoyancy_result: Result from buoyancy equilibrium solver
        heel_angles: List of heel angles in degrees (default: -20 to 60)
        beam_m: Overall beam in meters (for period estimates)
        loa_m: Length overall in meters (for period estimates)
        verbose: Print progress

    Returns:
        Dictionary with GZ curve data
    """
    if heel_angles is None:
        heel_angles = (
            list(range(-60, -5, 5)) +
            list(range(-5, 6, 1)) +
            list(range(10, 70, 5))
        )

    # Extract equilibrium data
    eq = buoyancy_result['equilibrium']
    eq_z = eq['z_offset_mm']

    # CoG in body frame
    cog_body = buoyancy_result['center_of_gravity_body']

    # Weight/mass
    weight_N = buoyancy_result['weight_N']
    total_mass_kg = buoyancy_result['total_mass_kg']

    gz_data = []

    for roll_deg in heel_angles:
        if verbose:
            print(f"  Computing GZ at heel = {roll_deg:+.1f}°...", end='', flush=True)

        # Find equilibrium z at this heel angle
        result = find_equilibrium_z_at_heel(
            hull, weight_N,
            pitch_deg=0.0,
            roll_deg=roll_deg,
            z_initial=eq_z
        )

        if not result['converged']:
            if verbose:
                print(f" FAILED: {result.get('error', 'unknown')}")
            gz_data.append({
                'heel_deg': roll_deg,
                'converged': False,
                'gz_m': 0.0,
                'righting_moment_Nm': 0.0,
                'error': result.get('error', 'Did not converge')
            })
            continue

        cob_result = result['cob_result']
        z_eq = result['z_mm']

        # Get rotation center
        rotation_center = cob_result['pose'].get('rotation_center', cog_body)

        # Transform CoG to world frame at this pose
        cog_world = transform_point(cog_body, z_eq, 0.0, roll_deg, rotation_center)

        # CoB is already in world frame
        cob = cob_result['CoB']

        # GZ = transverse righting arm
        raw_gz_mm = cob['x'] - cog_world['x']
        if abs(roll_deg) < 0.01:
            gz_mm = raw_gz_mm
        else:
            sign = 1 if roll_deg > 0 else -1
            gz_mm = sign * raw_gz_mm
        gz_m = gz_mm / 1000.0

        # Righting moment = GZ x weight
        righting_moment_Nm = gz_m * weight_N

        if verbose:
            print(f" GZ = {gz_m*100:.1f} cm, RM = {righting_moment_Nm:.0f} Nm")

        gz_data.append({
            'heel_deg': roll_deg,
            'converged': True,
            'z_eq_mm': round(z_eq, 2),
            'cob_x_mm': round(cob['x'], 2),
            'cob_y_mm': round(cob['y'], 2),
            'cob_z_mm': round(cob['z'], 2),
            'cog_world_x_mm': round(cog_world['x'], 2),
            'cog_world_y_mm': round(cog_world['y'], 2),
            'cog_world_z_mm': round(cog_world['z'], 2),
            'raw_gz_mm': round(raw_gz_mm, 2),
            'gz_mm': round(gz_mm, 2),
            'gz_m': round(gz_m, 4),
            'righting_moment_Nm': round(righting_moment_Nm, 2),
            'buoyancy_force_N': round(cob_result['buoyancy_force_N'], 2),
            'submerged_volume_liters': round(cob_result['submerged_volume_liters'], 2)
        })

    # Compute summary statistics
    converged_points = [p for p in gz_data if p.get('converged', False)]

    if converged_points:
        gz_values = [p['gz_m'] for p in converged_points]
        max_gz = max(gz_values)
        max_gz_idx = gz_values.index(max_gz)
        max_gz_angle = converged_points[max_gz_idx]['heel_deg']

        positive_heel_angles = [p['heel_deg'] for p in converged_points if p['gz_m'] > 0]
        if positive_heel_angles:
            range_positive = max(positive_heel_angles)
        else:
            range_positive = 0

        # Find turtle angle
        turtle_angle = None
        for i in range(1, len(converged_points)):
            if (converged_points[i-1]['heel_deg'] > 0 and
                converged_points[i-1]['gz_m'] > 0 and converged_points[i]['gz_m'] <= 0):
                gz1 = converged_points[i-1]['gz_m']
                gz2 = converged_points[i]['gz_m']
                angle1 = converged_points[i-1]['heel_deg']
                angle2 = converged_points[i]['heel_deg']
                if gz1 != gz2:
                    turtle_angle = angle1 + gz1 * (angle2 - angle1) / (gz1 - gz2)
                else:
                    turtle_angle = angle1
                break

        # Find capsize angle
        capsize_angle = None
        for i in range(len(converged_points) - 1, 0, -1):
            if (converged_points[i]['heel_deg'] < 0 and
                converged_points[i]['gz_m'] > 0 and converged_points[i-1]['gz_m'] <= 0):
                gz1 = converged_points[i-1]['gz_m']
                gz2 = converged_points[i]['gz_m']
                angle1 = converged_points[i-1]['heel_deg']
                angle2 = converged_points[i]['heel_deg']
                if gz1 != gz2:
                    capsize_angle = angle1 + gz1 * (angle2 - angle1) / (gz1 - gz2)
                else:
                    capsize_angle = angle1
                break

        # Detect ama engagement angle via inflection point in CoB X curve.
        # As the boat heels towards the ama, CoB shifts rapidly laterally
        # while the ama submerges, then the rate drops sharply once the ama
        # is fully engaged. We detect this as the largest drop in the rate
        # of lateral CoB shift (most negative second derivative).
        ama_engagement_angle = None
        positive_points = [p for p in converged_points if p['heel_deg'] >= 0]
        if len(positive_points) >= 3:
            rates = []
            for i in range(1, len(positive_points)):
                dc = positive_points[i]['cob_x_mm'] - positive_points[i-1]['cob_x_mm']
                da = positive_points[i]['heel_deg'] - positive_points[i-1]['heel_deg']
                if da > 0:
                    rates.append((positive_points[i-1]['heel_deg'],
                                  positive_points[i]['heel_deg'], dc / da))
            max_drop = 0
            for i in range(1, len(rates)):
                drop = rates[i-1][2] - rates[i][2]
                if drop > max_drop:
                    max_drop = drop
                    ama_engagement_angle = (rates[i-1][1] + rates[i][1]) / 2
            if max_drop < 50:
                ama_engagement_angle = None

        summary = {
            'max_gz_m': round(max_gz, 4),
            'max_gz_angle_deg': max_gz_angle,
            'range_of_positive_stability_deg': range_positive,
            'turtle_angle_deg': round(turtle_angle, 1) if turtle_angle else None,
            'capsize_angle_deg': round(capsize_angle, 1) if capsize_angle else None,
            'ama_engagement_angle_deg': round(ama_engagement_angle, 1) if ama_engagement_angle else None,
            'total_points': len(gz_data),
            'converged_points': len(converged_points)
        }
    else:
        summary = {
            'max_gz_m': 0,
            'max_gz_angle_deg': None,
            'range_of_positive_stability_deg': 0,
            'turtle_angle_deg': None,
            'capsize_angle_deg': None,
            'ama_engagement_angle_deg': None,
            'total_points': len(gz_data),
            'converged_points': 0
        }

    # Compute GM from GZ curve
    eq_roll = eq['roll_deg']
    gm_result = compute_gm_from_gz_curve(gz_data, eq_roll)
    summary['gm_m'] = gm_result['gm_m']
    summary['gm_longitudinal_m'] = gm_result['gm_longitudinal_m']

    # Estimate natural periods if we have the needed dimensions
    natural_periods = {}
    if beam_m and loa_m:
        zero_heel = next((p for p in gz_data if abs(p['heel_deg']) < 0.5
                         and p.get('converged')), None)
        if zero_heel:
            submerged_vol = zero_heel['submerged_volume_liters']
            draft_m = abs(eq_z) / 1000.0
            natural_periods = estimate_natural_periods(
                mass_kg=total_mass_kg,
                submerged_volume_liters=submerged_vol,
                draft_m=draft_m,
                beam_m=beam_m,
                loa_m=loa_m
            )
            if verbose:
                print(f"  Natural period estimates (approximate):")
                print(f"    Heave: {natural_periods.get('heave_period_s', 'N/A')} s")
                print(f"    Roll:  {natural_periods.get('roll_period_s', 'N/A')} s")
                print(f"    Pitch: {natural_periods.get('pitch_period_s', 'N/A')} s")

    return {
        'validator': 'gz',
        'summary': summary,
        'total_mass_kg': total_mass_kg,
        'weight_N': weight_N,
        'equilibrium_pose': eq,
        'center_of_gravity_body': cog_body,
        'natural_periods': natural_periods,
        'gz_curve': gz_data
    }


def plot_gz_curve(gz_result: dict, output_path: str):
    """
    Generate a PNG plot of the GZ curve.

    Args:
        gz_result: Result from compute_gz_curve
        output_path: Path for output PNG file
    """
    import matplotlib
    matplotlib.use('Agg')  # Non-interactive backend
    import matplotlib.pyplot as plt

    # Extract data for plotting
    gz_data = gz_result['gz_curve']
    converged = [p for p in gz_data if p.get('converged', False)]

    if not converged:
        print("Warning: No converged points to plot")
        return

    angles = [p['heel_deg'] for p in converged]
    gz_values = [p['gz_m'] * 100 for p in converged]  # Convert to cm for readability
    rm_values = [p['righting_moment_Nm'] for p in converged]

    # Get equilibrium roll angle
    eq_roll = gz_result['equilibrium_pose']['roll_deg']

    # Create figure with two y-axes
    fig, ax1 = plt.subplots(figsize=(10, 6))

    # Plot GZ curve
    color1 = '#2563eb'
    ax1.set_xlabel('Heel Angle (degrees)', fontsize=12)
    ax1.set_ylabel('GZ Righting Arm (cm)', color=color1, fontsize=12)
    line1, = ax1.plot(angles, gz_values, 'o-', color=color1, linewidth=2,
                      markersize=4, label='GZ (cm)')
    ax1.tick_params(axis='y', labelcolor=color1)
    ax1.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
    ax1.axvline(x=0, color='gray', linestyle='--', alpha=0.5)

    # Mark the equilibrium roll angle
    ax1.axvline(x=eq_roll, color='green', linestyle=':', alpha=0.7, linewidth=2,
                label=f'Equilibrium ({eq_roll:.1f}°)')

    # Add grid
    ax1.grid(True, alpha=0.3)

    # Second y-axis for righting moment
    ax2 = ax1.twinx()
    color2 = '#dc2626'
    ax2.set_ylabel('Righting Moment (Nm)', color=color2, fontsize=12)
    line2, = ax2.plot(angles, rm_values, 's--', color=color2, linewidth=1.5,
                      markersize=3, alpha=0.7, label='RM (Nm)')
    ax2.tick_params(axis='y', labelcolor=color2)

    # Mark ama engagement angle if detected
    ama_angle = gz_result['summary'].get('ama_engagement_angle_deg')
    if ama_angle is not None:
        ax1.axvline(x=ama_angle, color='orange', linestyle=':', alpha=0.7, linewidth=2,
                    label=f'Ama engages ({ama_angle:.1f}°)')

    # Add summary statistics to plot
    summary = gz_result['summary']
    stats_text = (
        f"Max GZ: {summary['max_gz_m']*100:.1f} cm at {summary['max_gz_angle_deg']}°"
    )
    if summary.get('turtle_angle_deg'):
        stats_text += f"\nTurtle: {summary['turtle_angle_deg']:.1f}°"
    if summary.get('capsize_angle_deg'):
        stats_text += f"\nCapsize: {summary['capsize_angle_deg']:.1f}°"
    if ama_angle is not None:
        stats_text += f"\nAma engages: {ama_angle:.1f}°"
    stats_text += f"\nEquilibrium roll: {eq_roll:.1f}°"

    ax1.text(0.02, 0.98, stats_text, transform=ax1.transAxes, fontsize=10,
             verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

    # Title
    mass = gz_result['total_mass_kg']
    plt.title(f'GZ Curve (Righting Arm) - Displacement: {mass:.0f} kg', fontsize=14)

    # Combined legend
    lines = [line1, line2]
    labels = [l.get_label() for l in lines]
    ax1.legend(lines, labels, loc='upper right')

    # Tight layout and save
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"✓ GZ curve plot saved to {output_path}")

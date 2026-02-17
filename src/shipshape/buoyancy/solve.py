#!/usr/bin/env python3
"""
Buoyancy equilibrium solver - finds the equilibrium pose of a boat.

Uses Newton-Raphson iteration to find the pose (z, pitch, roll) where:
1. Force equilibrium: buoyancy force = weight
2. Moment equilibrium: CoB is directly below CoG (no pitch/roll moments)
"""

import math

import numpy as np

from shipshape.physics.center_of_buoyancy import compute_center_of_buoyancy


def extract_group_breakdown(cob_result: dict) -> dict:
    """
    Extract per-group breakdown from CoB result.

    Returns:
        dict mapping group name to:
        - submerged_liters: submerged volume for this group
        - total_liters: total volume for this group
        - z_world: world z-coordinate of group reference point
    """
    # Sum submerged volume per group from components
    group_submerged = {}
    for comp in cob_result.get('components', []):
        g = comp.get('group', 'hull')
        vol = comp.get('submerged_volume_liters', 0.0)
        group_submerged[g] = group_submerged.get(g, 0.0) + vol

    hull_refs = cob_result.get('hull_refs', {})
    total_volumes = cob_result.get('total_volumes', {})

    # Build breakdown for every group present in hull_refs (covers all groups
    # even if they have zero submerged volume).
    all_groups = set(group_submerged) | set(hull_refs) | set(total_volumes)
    breakdown = {}
    for name in sorted(all_groups):
        breakdown[name] = {
            'submerged_liters': round(group_submerged.get(name, 0.0), 1),
            'total_liters': total_volumes.get(name, 0.0),
            'z_world': round(hull_refs.get(name, {}).get('world', {}).get('z', 0.0), 0),
        }

    return breakdown


# Solver parameters
DEFAULT_MAX_ITERATIONS = 50
DEFAULT_TOLERANCE = 1e-3  # Convergence tolerance for residuals
DEFAULT_Z_STEP = 10.0     # mm, for numerical Jacobian
DEFAULT_ANGLE_STEP = 0.1  # degrees, for numerical Jacobian


def transform_point(point: dict, z_displacement: float, pitch_deg: float,
                    roll_deg: float, rotation_center: dict) -> dict:
    """
    Transform a point by z displacement and pitch/roll rotations.

    The rotation is applied about rotation_center, then z displacement is added.
    """
    # Translate to rotation center
    x = point['x'] - rotation_center['x']
    y = point['y'] - rotation_center['y']
    z = point['z'] - rotation_center['z']

    # Apply rotation (same matrix as in center_of_buoyancy.py)
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

    # Translate back and apply z displacement
    return {
        'x': x_new + rotation_center['x'],
        'y': y_new + rotation_center['y'],
        'z': z_new + rotation_center['z'] + z_displacement
    }


def compute_residuals(cog_result: dict, cob_result: dict,
                      z_displacement: float, pitch_deg: float, roll_deg: float) -> np.ndarray:
    """
    Compute equilibrium residuals.

    Both CoG and CoB must be in world coordinates for proper comparison.
    CoG is transformed from body frame using the current pose.

    Returns array of [force_residual, pitch_moment, roll_moment]:
    - force_residual: (buoyancy - weight) / weight  [normalized]
    - pitch_moment: (CoB_y - CoG_y) / 1000  [normalized to meters]
    - roll_moment: (CoB_x - CoG_x) / 1000  [normalized to meters]

    At equilibrium, all residuals should be zero.
    """
    weight_N = cog_result['weight_N']
    buoyancy_N = cob_result['buoyancy_force_N']

    # CoG in body frame (from mass calculation)
    CoG_body = cog_result['CoG']

    # Get rotation center from CoB result (same center used for hull transformation)
    rotation_center = cob_result['pose'].get('rotation_center', CoG_body)

    # Transform CoG to world frame using current pose
    CoG_world = transform_point(CoG_body, z_displacement, pitch_deg, roll_deg, rotation_center)

    # CoB is already in world frame (computed from transformed hull)
    CoB = cob_result['CoB']

    # Force residual (normalized by weight)
    force_residual = (buoyancy_N - weight_N) / weight_N if weight_N > 0 else 0

    # Moment residuals (normalized to meters)
    # For CoB to be directly below CoG, their x and y must match in world frame
    pitch_moment = (CoB['y'] - CoG_world['y']) / 1000.0  # Y offset causes pitch moment
    roll_moment = (CoB['x'] - CoG_world['x']) / 1000.0   # X offset causes roll moment

    return np.array([force_residual, pitch_moment, roll_moment])


def compute_jacobian(hull: dict, cog_result: dict,
                     z: float, pitch: float, roll: float,
                     z_step: float = DEFAULT_Z_STEP,
                     angle_step: float = DEFAULT_ANGLE_STEP) -> np.ndarray:
    """
    Compute Jacobian matrix numerically using central differences.

    J[i,j] = d(residual_i) / d(state_j)

    State = [z, pitch, roll]
    Residuals = [force, pitch_moment, roll_moment]
    """
    J = np.zeros((3, 3))

    # Compute derivatives with respect to z
    cob_plus = compute_center_of_buoyancy(hull, z + z_step, pitch, roll)
    cob_minus = compute_center_of_buoyancy(hull, z - z_step, pitch, roll)
    r_plus = compute_residuals(cog_result, cob_plus, z + z_step, pitch, roll)
    r_minus = compute_residuals(cog_result, cob_minus, z - z_step, pitch, roll)
    J[:, 0] = (r_plus - r_minus) / (2 * z_step)

    # Compute derivatives with respect to pitch
    cob_plus = compute_center_of_buoyancy(hull, z, pitch + angle_step, roll)
    cob_minus = compute_center_of_buoyancy(hull, z, pitch - angle_step, roll)
    r_plus = compute_residuals(cog_result, cob_plus, z, pitch + angle_step, roll)
    r_minus = compute_residuals(cog_result, cob_minus, z, pitch - angle_step, roll)
    J[:, 1] = (r_plus - r_minus) / (2 * angle_step)

    # Compute derivatives with respect to roll
    cob_plus = compute_center_of_buoyancy(hull, z, pitch, roll + angle_step)
    cob_minus = compute_center_of_buoyancy(hull, z, pitch, roll - angle_step)
    r_plus = compute_residuals(cog_result, cob_plus, z, pitch, roll + angle_step)
    r_minus = compute_residuals(cog_result, cob_minus, z, pitch, roll - angle_step)
    J[:, 2] = (r_plus - r_minus) / (2 * angle_step)

    return J


def estimate_initial_z(cog_result: dict, hull: dict) -> float:
    """
    Estimate initial z displacement to get buoyancy roughly equal to weight.

    Uses a simple search to find z where buoyancy is close to weight.
    """
    weight_N = cog_result['weight_N']

    # Binary search for z
    z_min, z_max = -5000.0, 0.0  # Search range in mm

    for _ in range(20):  # Binary search iterations
        z_mid = (z_min + z_max) / 2
        cob = compute_center_of_buoyancy(hull, z_mid, 0, 0)
        buoyancy_N = cob['buoyancy_force_N']

        if buoyancy_N < weight_N:
            z_max = z_mid  # Need to sink more (more negative z)
        else:
            z_min = z_mid  # Need to rise (less negative z)

        # Stop if close enough
        if abs(buoyancy_N - weight_N) / weight_N < 0.01:
            break

    return z_mid


def solve_equilibrium(hull: dict, cog_result: dict,
                      max_iterations: int = DEFAULT_MAX_ITERATIONS,
                      tolerance: float = DEFAULT_TOLERANCE,
                      z_step: float = DEFAULT_Z_STEP,
                      angle_step: float = DEFAULT_ANGLE_STEP,
                      verbose: bool = True) -> dict:
    """
    Find equilibrium pose using Newton-Raphson iteration.

    Args:
        hull: Hull data from load_hull()
        cog_result: Result from compute_center_of_gravity
        max_iterations: Maximum Newton-Raphson iterations
        tolerance: Convergence tolerance for residuals
        verbose: Print progress information

    Returns:
        Dictionary with equilibrium results
    """
    # Initial guess
    if verbose:
        print("  Estimating initial z displacement...")
    z = estimate_initial_z(cog_result, hull)
    pitch = 0.0
    roll = 0.0

    if verbose:
        print(f"  Initial guess: z={z:.1f}mm, pitch={pitch:.2f}°, roll={roll:.2f}°")

    iteration_history = []
    converged = False

    for iteration in range(max_iterations):
        # Compute CoB at current pose
        cob_result = compute_center_of_buoyancy(hull, z, pitch, roll)

        # Compute residuals (CoG is transformed to world frame internally)
        residuals = compute_residuals(cog_result, cob_result, z, pitch, roll)
        residual_norm = np.linalg.norm(residuals)

        # Extract per-group breakdown for diagnostics
        group_breakdown = extract_group_breakdown(cob_result)

        # Record iteration
        iter_record = {
            'iteration': iteration,
            'z_mm': round(z, 2),
            'pitch_deg': round(pitch, 4),
            'roll_deg': round(roll, 4),
            'residual_norm': round(residual_norm, 6),
            'force_residual': round(residuals[0], 6),
            'pitch_residual': round(residuals[1], 6),
            'roll_residual': round(residuals[2], 6),
            'buoyancy_N': round(cob_result['buoyancy_force_N'], 2),
            'submerged_liters': round(cob_result['submerged_volume_liters'], 2),
            'group_breakdown': group_breakdown,
        }
        iteration_history.append(iter_record)

        if verbose:
            print(f"  Iteration {iteration}: z={z:.1f}mm, pitch={pitch:.3f}°, "
                  f"roll={roll:.3f}°, |r|={residual_norm:.4f} "
                  f"[F:{residuals[0]:.3f}, P:{residuals[1]:.3f}, R:{residuals[2]:.3f}]")
            parts = []
            for gname, gdata in group_breakdown.items():
                parts.append(f"{gname}: {gdata['submerged_liters']:.0f}L @ z={gdata['z_world']:.0f}mm")
            print(f"    {', '.join(parts)}")

        # Check convergence
        if residual_norm < tolerance:
            converged = True
            if verbose:
                print(f"  Converged after {iteration + 1} iterations")
            break

        # Compute Jacobian
        J = compute_jacobian(hull, cog_result, z, pitch, roll,
                             z_step=z_step, angle_step=angle_step)

        # Check if Jacobian is singular
        det = np.linalg.det(J)
        if abs(det) < 1e-12:
            if verbose:
                print(f"  Warning: Jacobian nearly singular (det={det:.2e})")
            # Use pseudo-inverse
            try:
                delta = np.linalg.lstsq(J, -residuals, rcond=None)[0]
            except np.linalg.LinAlgError:
                if verbose:
                    print("  Error: Could not solve linear system")
                break
        else:
            # Newton-Raphson update
            delta = np.linalg.solve(J, -residuals)

        # Freeze DOFs whose residual is negligibly small (10x below tolerance).
        # Prevents cross-coupling from corrupting converged components
        # (e.g. roll=0 on a symmetric hull getting kicked by a large pitch error).
        # The tight threshold (tolerance/10) ensures only genuinely converged DOFs
        # are frozen, avoiding premature freezing with loose tolerances.
        dof_names = ['z', 'pitch', 'roll']
        freeze_tol = tolerance * 0.1
        freeze = [abs(residuals[i]) < freeze_tol for i in range(3)]
        if all(freeze):
            worst = int(np.argmax(np.abs(residuals)))
            freeze[worst] = False
        for i in range(3):
            if freeze[i] and abs(delta[i]) > 0:
                if verbose:
                    print(f"    Freezing {dof_names[i]} (residual {residuals[i]:.6f} negligible)")
                delta[i] = 0.0

        # Limit maximum step size for stability
        max_z_step = 200.0  # mm
        max_angle_step = 5.0  # degrees

        step_scale = 1.0
        if abs(delta[0]) > max_z_step:
            step_scale = min(step_scale, max_z_step / abs(delta[0]))
        if abs(delta[1]) > max_angle_step:
            step_scale = min(step_scale, max_angle_step / abs(delta[1]))
        if abs(delta[2]) > max_angle_step:
            step_scale = min(step_scale, max_angle_step / abs(delta[2]))

        if step_scale < 1.0:
            delta = delta * step_scale
            if verbose:
                print(f"    Step scaled by {step_scale:.3f} for stability")

        # Backtracking line search (Armijo condition)
        alpha = 1.0
        min_alpha = 0.01
        backtrack_factor = 0.5

        # Track best step that actually improves residual
        best_alpha = 0.0  # No step by default
        best_norm = residual_norm  # Must beat current residual

        while alpha >= min_alpha:
            # Try step with current alpha
            z_new = z + alpha * delta[0]
            pitch_new = pitch + alpha * delta[1]
            roll_new = roll + alpha * delta[2]

            # Evaluate residual at new point
            cob_new = compute_center_of_buoyancy(hull, z_new, pitch_new, roll_new)
            residuals_new = compute_residuals(cog_result, cob_new, z_new, pitch_new, roll_new)
            new_norm = np.linalg.norm(residuals_new)

            # Track best improvement
            if new_norm < best_norm:
                best_alpha = alpha
                best_norm = new_norm

            # Armijo condition: sufficient decrease
            if new_norm < residual_norm * (1 - 0.1 * alpha):
                break

            alpha *= backtrack_factor

        # Use best alpha found
        if alpha < min_alpha:
            alpha = best_alpha
            if alpha == 0.0:
                if verbose:
                    print(f"    Line search failed: no improvement found, taking small step")
                alpha = min_alpha  # Take minimal step to avoid getting stuck
            else:
                if verbose:
                    print(f"    Line search: using best α={alpha:.3f}")

        # Update state with chosen step size
        z += alpha * delta[0]
        pitch += alpha * delta[1]
        roll += alpha * delta[2]

    # Final CoB computation
    final_cob = compute_center_of_buoyancy(hull, z, pitch, roll)
    final_residuals = compute_residuals(cog_result, final_cob, z, pitch, roll)

    # Transform CoG to world frame for output
    rotation_center = final_cob['pose'].get('rotation_center', cog_result['CoG'])
    cog_world = transform_point(cog_result['CoG'], z, pitch, roll, rotation_center)

    # Extract per-group breakdown
    group_breakdown = extract_group_breakdown(final_cob)

    # Build hull_groups output: one entry per group
    hull_groups_output = {}
    for name, gdata in group_breakdown.items():
        total_l = gdata['total_liters']
        sub_l = gdata['submerged_liters']
        hull_groups_output[name] = {
            'submerged_volume_liters': sub_l,
            'total_volume_liters': total_l,
            'submerged_percent': round(100 * sub_l / total_l, 1) if total_l > 0 else 0.0,
            'z_world_mm': gdata['z_world'],
        }

    return {
        'converged': converged,
        'iterations': len(iteration_history),
        'equilibrium': {
            'z_offset_mm': round(z, 2),
            'pitch_deg': round(pitch, 4),
            'roll_deg': round(roll, 4)
        },
        'center_of_gravity_body': cog_result['CoG'],
        'center_of_gravity_world': {
            'x': round(cog_world['x'], 2),
            'y': round(cog_world['y'], 2),
            'z': round(cog_world['z'], 2)
        },
        'center_of_buoyancy': final_cob['CoB'],
        'total_mass_kg': round(cog_result['total_mass_kg'], 2),
        'weight_N': round(cog_result['weight_N'], 2),
        'buoyancy_force_N': round(final_cob['buoyancy_force_N'], 2),
        'submerged_volume_liters': round(final_cob['submerged_volume_liters'], 2),
        'hull_groups': hull_groups_output,
        'final_residuals': {
            'force': round(final_residuals[0], 6),
            'pitch_moment': round(final_residuals[1], 6),
            'roll_moment': round(final_residuals[2], 6),
            'norm': round(np.linalg.norm(final_residuals), 6)
        },
        'iteration_history': iteration_history
    }

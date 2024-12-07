from typing import Callable, Dict
import numpy as np

import numpy as np

def compute_alpha_beta(R_wb, v_world):
    """
    Compute angle of attack (alpha) and sideslip angle (beta) from the relative velocity in the world frame
    and the body rotation matrix.

    :param R_wb: 3x3 rotation matrix from world to body frame
    :param v_world: 3x1 velocity vector in the world frame
    :return: alpha (angle of attack in radians), beta (sideslip angle in radians)
    """
    # Transform the velocity from world frame to body frame
    v_body = np.dot(R_wb, v_world)

    # Extract velocity components in the body frame
    v_bx, v_by, v_bz = v_body

    # Compute the angle of attack (alpha)
    alpha = np.arctan2(v_bz, v_bx)  # Arctan2 handles the correct quadrant

    # Compute the sideslip angle (beta)
    v_b_magnitude = np.sqrt(v_bx**2 + v_by**2 + v_bz**2)
    beta = np.arcsin(v_by / v_b_magnitude)

    return alpha, beta

# Example usage:
R_wb = np.array([[0.866, -0.5, 0],
                 [0.5, 0.866, 0],
                 [0, 0, 1]])  # Example rotation matrix (from world to body frame)

v_world = np.array([100, 10, -5])  # Example relative velocity in world frame (m/s)

alpha, beta = compute_alpha_beta(R_wb, v_world)

print(f"Angle of Attack (alpha): {np.degrees(alpha):.2f} degrees")
print(f"Sideslip Angle (beta): {np.degrees(beta):.2f} degrees")
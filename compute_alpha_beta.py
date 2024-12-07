from typing import Callable, Dict, Tuple
import numpy as np

import numpy as np

def alpha_beta(v_body:np.array)->Tuple[float, float]:
    """
    Compute angle of attack (alpha) and sideslip angle (beta) from the relative velocity in the body frame
    See https://www.mathworks.com/help/aerotbx/ug/alphabeta.html

    Args:
        v_body(np.array): 3x1 velocity vector in the body frame

    Returns:
        Tuple[float, float]: alpha (angle of attack in radians), beta (sideslip angle in radians)
    """

    # Extract velocity components in the body frame
    v_bx, v_by, v_bz = v_body

    # Compute the angle of attack (alpha)
    alpha = np.arctan2(v_bz, v_bx)  # Arctan2 handles the correct quadrant

    # Compute the sideslip angle (beta)
    v_b_magnitude = np.sqrt(v_bx ** 2 + v_by ** 2 + v_bz ** 2)
    beta = np.arcsin(v_by / v_b_magnitude)
    return alpha, beta

def compute_alpha_beta(R_wb:np.array, v_world:np.array)->Tuple[float, float]:
    """
    Compute angle of attack (alpha) and sideslip angle (beta) from the relative velocity in the world frame
    and the body rotation matrix.
    Args:
        R_wb(np.array): 3x3 rotation matrix from world to body frame
        v_world(np.array): 3x1 velocity vector in the world frame

    Returns:
        Tuple[float, float]: alpha (angle of attack in radians), beta (sideslip angle in radians)
    """

    # Transform the velocity from world frame to body frame
    v_body = np.dot(R_wb, v_world)

    alpha, beta = alpha_beta(v_body = v_body)

    return alpha, beta


import numpy as np

from src.compute_alpha_beta import compute_alpha_beta, alpha_beta
import pytest

def test_alpha_beta():
    # Example taken from matlab documentation
    # https://www.mathworks.com/help/aerotbx/ug/alphabeta.html
    alpha, beta = alpha_beta([84.3905,  33.7562,  10.1269])

    assert alpha == pytest.approx(0.1194, rel=1e-3)

    assert beta ==pytest.approx(0.3780, rel=1e-3)

def test_compute_alpha_beta():
    # Example usage:
    R_wb = np.array([[0.866, -0.5, 0],
                     [0.5, 0.866, 0],
                     [0, 0, 1]])  # Example rotation matrix (from world to body frame)

    v_world = np.array([100, 10, -5])  # Example relative velocity in world frame (m/s)

    alpha, beta = compute_alpha_beta(R_wb, v_world)

    # Non regression value...
    assert alpha==pytest.approx(-0.06119, rel=1e-3)
    assert beta==pytest.approx(0.62239, rel=1e-3)

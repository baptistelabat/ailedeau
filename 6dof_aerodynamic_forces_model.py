from typing import Callable, Dict
import numpy as np


class AeroCoefficients:
    """
    Stores dimensionless aerodynamic coefficients and their dependencies.
    """

    def __init__(self):
        # Translational force coefficients
        self.C_L: Callable[[float], float] = self.cl_curve
        self.C_D: Callable[[float], float] = self.cd_curve
        self.C_Y_beta: float = -0.1  # Side force coefficient slope (per radian of beta)

        # Rotational force derivatives
        self.C_L_q: float = 2.0  # Lift damping due to pitch rate
        self.C_Y_p: float = -0.05  # Side force coupling to roll rate
        self.C_Y_r: float = 0.1  # Side force coupling to yaw rate
        self.C_D_q: float = 0.02  # Drag induced by pitch rate

        # Rotational moment coefficients
        self.C_m_alpha: float = -0.1  # Pitch moment slope (per radian of alpha)
        self.C_l_beta: float = 0.1  # Rolling moment coefficient due to beta
        self.C_l_aileron: float = 0.05  # Rolling moment coefficient due to aileron deflection
        self.C_n_beta: float = -0.02  # Yawing moment coefficient due to beta
        self.C_n_rudder: float = 0.03  # Yawing moment coefficient due to rudder deflection

        # Rotational damping derivatives
        self.C_m_q: float = -5.0  # Pitch damping
        self.C_l_p: float = -0.5  # Roll damping
        self.C_l_r: float = 0.2  # Yaw coupling to roll moment
        self.C_n_p: float = -0.1  # Roll coupling to yaw moment
        self.C_n_r: float = -0.3  # Yaw damping

    @staticmethod
    def cl_curve(alpha: float) -> float:
        """
        Lift coefficient curve as a function of angle of attack (alpha, in radians).
        Assumes a linear region followed by stall behavior.
        """
        if alpha < np.radians(-15):
            return -0.2  # approximate stall behavior
        elif alpha > np.radians(15):
            return 1.2  # approximate stall behavior
        else:
            return 2 * np.pi * alpha  # Linear lift slope (per radian)

    @staticmethod
    def cd_curve(alpha: float) -> float:
        """
        Drag coefficient curve as a function of angle of attack (alpha, in radians).
        Includes quadratic dependence for induced drag.
        """
        base_drag = 0.01
        induced_drag = 0.1 * alpha ** 2
        return base_drag + induced_drag

    def to_dict(self) -> Dict[str, float]:
        """Convert coefficients to a dictionary for easier serialization if needed."""
        return self.__dict__


class AircraftAerodynamics:
    """
    Computes aerodynamic forces and moments for an aircraft.
    """

    def __init__(
            self,
            ref_area: float,
            ref_length_chord: float,
            ref_length_span: float,
            coefficients: AeroCoefficients
    ):
        self.S: float = ref_area
        self.c: float = ref_length_chord
        self.b: float = ref_length_span
        self.coeffs: AeroCoefficients = coefficients

    def compute_forces_and_moments(
            self,
            rho: float,
            velocity: float,
            alpha: float,
            beta: float,
            ang_vel: Dict[str, float],
            control_surfaces: Dict[str, float]
    ) -> Dict[str, np.ndarray]:
        """
        Compute aerodynamic forces and moments including rotational effects.

        :param rho: Air density (kg/m^3).
        :param velocity: Airspeed (m/s).
        :param alpha: Angle of attack (radians).
        :param beta: Sideslip angle (radians).
        :param ang_vel: Angular velocity components {'p', 'q', 'r'} (rad/s).
        :param control_surfaces: Control deflections {'aileron', 'rudder', ...}.
        :return: Forces {'F_x', 'F_y', 'F_z'} and moments {'M_x', 'M_y', 'M_z'}.
        """
        q = 0.5 * rho * velocity ** 2  # Dynamic pressure

        # Translational coefficients
        C_L = self.coeffs.C_L(alpha)
        C_D = self.coeffs.C_D(alpha)
        C_Y = self.coeffs.C_Y_beta * beta

        # Forces in body axes
        F_x = -q * self.S * C_D
        F_y = q * self.S * C_Y
        F_z = -q * self.S * C_L

        # Rotational contributions
        C_l = self.coeffs.C_l_beta * beta + self.coeffs.C_l_aileron * control_surfaces.get("aileron", 0.0)
        C_m = self.coeffs.C_m_alpha * alpha + self.coeffs.C_m_q * self.c / (2 * velocity) * ang_vel["q"]
        C_n = self.coeffs.C_n_beta * beta + self.coeffs.C_n_rudder * control_surfaces.get("rudder", 0.0)

        # Moments in body axes
        M_x = q * self.S * self.b * C_l
        M_y = q * self.S * self.c * C_m
        M_z = q * self.S * self.b * C_n

        return {
            "F_x": F_x,
            "F_y": F_y,
            "F_z": F_z,
            "M_x": M_x,
            "M_y": M_y,
            "M_z": M_z
        }


# Example usage
if __name__ == "__main__":
    # Reference dimensions
    ref_area = 1.0  # m^2
    ref_chord = 1.0  # m
    ref_span = 10.0  # m

    # Coefficients
    aero_coeffs = AeroCoefficients()

    # Aerodynamics model
    aero_model = AircraftAerodynamics(ref_area, ref_chord, ref_span, aero_coeffs)

    # Example conditions
    rho = 1.225  # Air density (kg/m^3)
    velocity = 50.0  # Airspeed (m/s)
    alpha = np.radians(5)  # Angle of attack (deg to rad)
    beta = np.radians(2)  # Sideslip angle (deg to rad)
    angular_velocities = {"p": 0.1, "q": 0.2, "r": 0.05}  # rad/s
    control_inputs = {"aileron": 0.1, "rudder": 0.05}  # rad

    # Compute forces and moments
    forces_moments = aero_model.compute_forces_and_moments(rho, velocity, alpha, beta, angular_velocities,
                                                           control_inputs)

    # Display results
    print("Aerodynamic Forces and Moments:")
    for key, value in forces_moments.items():
        print(f"{key}: {value:.3f}")

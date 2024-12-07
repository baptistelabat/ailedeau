from typing import Callable, Dict
import numpy as np


class AeroCoefficients:
    """
    Stores dimensionless aerodynamic coefficients and their dependencies for a glider.

    These coefficients are used to compute aerodynamic forces and moments, including both translational and rotational effects.
    The coefficients here are typical for gliders, but specific values can vary depending on the design and aerodynamic properties of the aircraft.

    Reference books:
        - Anderson, J.D. (2010). *Introduction to Flight*. McGraw-Hill Education.
        - Abbott, I.H., & Von Doenhoff, A.E. (1959). *Theory of Wing Sections*. Dover Publications.
    """

    def __init__(self):
        # Translational force coefficients:
        self.C_L: Callable[[float], float] = self.cl_curve
        self.C_D: Callable[[float], float] = self.cd_curve
        self.C_Y_beta: float = -0.1  # Side force coefficient slope (per radian of beta)

        # Rotational force derivatives:
        self.C_L_q: float = 2.0  # Lift damping due to pitch rate
        self.C_Y_p: float = -0.05  # Side force coupling to roll rate
        self.C_Y_r: float = 0.1  # Side force coupling to yaw rate
        self.C_D_q: float = 0.02  # Drag induced by pitch rate

        # Rotational moment coefficients:
        self.C_m_alpha: float = -0.1  # Pitch moment slope (per radian of alpha)
        self.C_l_beta: float = 0.1  # Rolling moment coefficient due to beta
        self.C_l_aileron: float = 0.05  # Rolling moment coefficient due to aileron deflection
        self.C_n_beta: float = -0.02  # Yawing moment coefficient due to beta
        self.C_n_rudder: float = 0.03  # Yawing moment coefficient due to rudder deflection

        # Additional rotational moment coefficients for roll, pitch, and yaw damping:
        self.C_l_p: float = -0.1  # Rolling moment coefficient due to roll rate
        self.C_l_r: float = 0.05  # Rolling moment coefficient due to yaw rate
        self.C_m_p: float = -0.15  # Pitch moment coefficient due to roll rate
        self.C_m_r: float = 0.1  # Pitch moment coefficient due to yaw rate
        self.C_n_p: float = -0.1  # Yawing moment coefficient due to roll rate
        self.C_n_r: float = -0.05  # Yawing moment coefficient due to yaw rate

        # Rotational damping derivatives (modeling the damping of angular motion):
        self.C_m_q: float = -5.0  # Pitch damping coefficient (due to pitch rate)
        self.C_l_p: float = -0.5  # Roll damping (due to roll rate)
        self.C_l_r: float = 0.2  # Yaw coupling to roll moment
        self.C_n_p: float = -0.1  # Roll coupling to yaw moment
        self.C_n_r: float = -0.3  # Yaw damping (due to yaw rate)

    @staticmethod
    def cl_curve(alpha: float) -> float:
        """
        Lift coefficient curve as a function of angle of attack (alpha, in radians).
        For small angles of attack, the lift is approximately linear with alpha (2*pi*alpha).
        For higher angles, the coefficient plateaus at the maximum value.
        This model assumes an approximate stall behavior for extreme angles.

        Reference:
            Anderson, J.D. (2010). *Introduction to Flight*.
        """
        if alpha < np.radians(-15):
            return -0.2  # Approximate stall behavior for low alpha
        elif alpha > np.radians(15):
            return 1.5  # Approximate maximum lift coefficient (for gliders)
        else:
            return 2 * np.pi * alpha  # Linear lift slope for small angles

    @staticmethod
    def cd_curve(alpha: float) -> float:
        """
        Drag coefficient curve as a function of angle of attack (alpha, in radians).
        The total drag coefficient is composed of base drag (parasite drag) and induced drag,
        which is quadratic in lift coefficient (C_L). For gliders, induced drag is significant at high angles of attack.

        Formula:
            C_D = C_D0 + k * C_L^2
            Where:
                - C_D0 is the zero-lift drag coefficient (typically 0.02 to 0.03 for gliders).
                - k is a constant that accounts for the induced drag.

        Reference:
            Anderson, J.D. (2010). *Introduction to Flight*.
        """
        base_drag = 0.02  # Zero-lift drag coefficient (gliders have low base drag)
        induced_drag = 0.05 * alpha ** 2  # Induced drag (quadratic dependence on C_L)
        return base_drag + induced_drag

    def to_dict(self) -> Dict[str, float]:
        """
        Convert the aerodynamic coefficients to a dictionary for easier serialization.

        Useful for exporting to formats such as JSON or for debugging purposes.
        """
        return self.__dict__


class AircraftAerodynamics:
    """
    Computes aerodynamic forces and moments for an aircraft, using the provided aerodynamic coefficients.

    The forces and moments are calculated based on the angle of attack, sideslip angle, angular velocities,
    and control surface deflections. The coefficients for lift, drag, side force, and moments are used
    to calculate the translational and rotational forces and moments on the aircraft.

    Reference:
        - Anderson, J.D. (2010). *Introduction to Flight*.
        - Houghton, E.L., & Carpenter, P.W. (2003). *Aerodynamics for Engineering Students*.
    """

    def __init__(
            self,
            ref_area: float,
            ref_length_chord: float,
            ref_length_span: float,
            coefficients: AeroCoefficients
    ):
        """
        Initialize the aircraft aerodynamics model with reference parameters (area, chord, and span)
        and aerodynamic coefficients.
        """
        self.S: float = ref_area  # Reference area (m^2)
        self.c: float = ref_length_chord  # Reference chord length (m)
        self.b: float = ref_length_span  # Reference wingspan (m)
        self.coeffs: AeroCoefficients = coefficients  # Aerodynamic coefficients

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
        Compute aerodynamic forces and moments, including both translational and rotational effects.

        Parameters:
            - rho: Air density (kg/m^3)
            - velocity: Airspeed (m/s)
            - alpha: Angle of attack (radians)
            - beta: Sideslip angle (radians)
            - ang_vel: Angular velocities (rad/s) in 'p', 'q', and 'r' (roll, pitch, yaw rates)
            - control_surfaces: Control deflections (e.g., aileron, rudder) in radians
        Returns:
            A dictionary containing aerodynamic forces ('F_x', 'F_y', 'F_z') and moments ('M_x', 'M_y', 'M_z')
        """
        q = 0.5 * rho * velocity ** 2  # Dynamic pressure

        # Translational forces:
        C_L = self.coeffs.C_L(alpha)
        C_D = self.coeffs.C_D(alpha)
        C_Y = self.coeffs.C_Y_beta * beta

        F_x = -q * self.S * C_D  # Drag force
        F_y = q * self.S * C_Y  # Side force
        F_z = -q * self.S * C_L  # Lift force

        # Rotational contributions (moments):
        C_l = self.coeffs.C_l_beta * beta + self.coeffs.C_l_aileron * control_surfaces.get("aileron", 0.0)
        C_m = self.coeffs.C_m_alpha * alpha + self.coeffs.C_m_q * self.c / (2 * velocity) * ang_vel["q"]
        C_n = self.coeffs.C_n_beta * beta + self.coeffs.C_n_rudder * control_surfaces.get("rudder", 0.0)

        # Moments (torques) in body axes:
        M_x = q * self.S * self.b * C_l  # Rolling moment
        M_y = q * self.S * self.c * C_m  # Pitching moment
        M_z = q * self.S * self.b * C_n  # Yawing moment

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
    ref_area = 10.0  # m^2
    ref_chord = 1.0  # m
    ref_span = 10.0  # m

    # Coefficients
    aero_coeffs = AeroCoefficients()

    # Aerodynamics model
    aero_model = AircraftAerodynamics(ref_area, ref_chord, ref_span, aero_coeffs)

    # Example conditions
    rho = 1.225  # Air density (kg/m^3)
    velocity = 20.0  # Airspeed (m/s)
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

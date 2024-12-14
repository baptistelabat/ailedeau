from exudyn import graphics
from exudyn.rigidBodyUtilities import InertiaCuboid, RotXYZ2RotationMatrix
import exudyn as exu
import numpy as np
from typing import List, Callable, Union, Tuple
import numpy as np

from aerodynamic_forces_model_6dof import AeroCoefficients, AircraftAerodynamics
from compute_alpha_beta import compute_alpha_beta

class KiteSystem:
    def __init__(self, mbs, attachment_body, anchor_point: list[float], line_length: float, line_diameter:float, line_density: float,
                 kite_chord: float, kite_span: float, kite_mass: float, angle_of_key: float,
                 wind_velocity: np.ndarray, fluid_volumetric_mass: float, magnifying_factor: float = 30, gravity: list[float] = [0, 0, 9.81]):
        """
        Initialize the kite system.

        Args:
            mbs: Multi-body system instance.
            attachment_body: body to which line is attached
            anchor_point: Anchor point of the kite line [3x1] [m]
            line_length: Length of the kite line [m]
            line_density: Density of the kite line material [kg/m3]
            kite_chord: Chord length of the kite [m]
            kite_span: Span of the kite [m]
            kite_mass: Mass of the kite [kg]
            angle_of_key: Initial angle of the kite [radians]
            wind_velocity: Wind velocity vector in world frame [3x1] [m/s]
            fluid_volumetric_mass: volumetric mass of fluid [kg/m3]
            magnifying_factor: Factor for visual scaling [-]
            gravity: Gravity vector in world frame [m/s2]
        """
        self.mbs = mbs
        self.attachment_body = attachment_body
        self.anchor_point = anchor_point
        self.line_length = line_length
        self.line_diameter = line_diameter
        self.line_density = line_density
        self.kite_chord = kite_chord
        self.kite_span = kite_span
        self.kite_mass = kite_mass
        self.angle_of_key = angle_of_key
        self.wind_velocity = wind_velocity
        self.fluid_volumetric_mass=fluid_volumetric_mass
        self.magnifying_factor = magnifying_factor
        self.gravity = gravity

        self.create_line()
        self.create_kite()
        self.create_joints()
        self.add_forces()

    def create_line(self) -> None:
        """
        Create line as rigid body

        Returns:
            None
        """
        # Line body properties
        self.line_mass = np.pi/4*self.line_length * self.line_diameter ** 2 * self.line_density
        body_dim = [self.line_diameter, self.line_diameter, self.line_length]
        self.mid_line_point = self.anchor_point + np.array([0, 0, self.line_length * -0.5])

        i_cube0 = InertiaCuboid(density=self.line_density, sideLengths=body_dim)#.Translated(mid_line_point)

        graphics_body0 = graphics.Brick(
            centerPoint=[0, 0, 0],
            size=[self.magnifying_factor * self.line_diameter, self.magnifying_factor * self.line_diameter, self.line_length],
            color=graphics.color.red
        )
        graphics_com0 = graphics.Basis(origin=[0,0,0], length=2 * self.magnifying_factor * self.line_diameter)

        self.line_body = self.mbs.CreateRigidBody(
            inertia=i_cube0,
            referencePosition=self.mid_line_point,
            gravity=self.gravity,
            graphicsDataList=[graphics_com0, graphics_body0]
        )

    def create_kite(self)->None:
        """
        Create kite as rigid body

        Returns:
            None
        """
        # Kite body properties
        thickness = 0.0001
        kite_density = self.kite_mass / (self.kite_chord * self.kite_span * thickness)
        kite_dim = [self.kite_chord, self.kite_span, thickness]

        i_cube_kite = InertiaCuboid(density=kite_density, sideLengths=kite_dim)

        graphics_kite = graphics.Brick(
            centerPoint=[0, 0, 0],
            size=kite_dim,
            color=graphics.color.blue
        )
        graphics_com_kite = graphics.Basis(origin=[0,0,0], length=self.kite_span)

        self.kite_body = self.mbs.CreateRigidBody(
            inertia=i_cube_kite,
            referencePosition=self.anchor_point + np.array([0, 0, -self.line_length]), # Kite up
            referenceRotationMatrix=RotXYZ2RotationMatrix([0, self.angle_of_key, 0]),
            gravity=self.gravity,
            graphicsDataList=[graphics_com_kite, graphics_kite]
        )

    def create_joints(self) -> None:
        """
        Create joints between body

        Returns:
            None
        """
        # Attach line to the ground
        self.mbs.CreateSphericalJoint(
            bodyNumbers=[self.attachment_body, self.line_body],
            position=self.anchor_point
        )

        # Attach kite to the line
        self.mbs.CreateGenericJoint(
            bodyNumbers=[self.line_body, self.kite_body],
            position=[0, 0, -self.line_length]
        )

    def add_forces(self) -> None:
        """
        Add forces and torques to bodies

        Returns:
            None
        """
        def wind_wrench(mbs, t: float, loadVector) -> Tuple[np.array, np.array]:
            """
            Compute the aerodynamic force wrench
            x forward
            y to starboard
            z down

            Args:
                mbs: multi-body system
                t (float): time, unused, kept to match interface
                loadVector (np.array): unused, to be kept to match interface

            Returns:
                Tuple[np.array, np.array]: vector Fx, Fy, Fz and Mx, My, Mz in aerodynamic convention in body frame
            """

            body_velocity_in_world = mbs.GetObjectOutputBody(self.kite_body, localPosition=[0, 0, 0],
                                                             variableType=exu.OutputVariableType.Velocity)

            body_fluid_velocity_in_world = body_velocity_in_world - self.wind_velocity

            # Get rotation matrix
            rotation_matrix = np.reshape(mbs.GetObjectOutputBody(self.kite_body, localPosition=[0, 0, 0],
                                                                 variableType=exu.OutputVariableType.RotationMatrix),
                                         [3, 3])

            # Get angular velocity, which play an important role in damping motion
            angular_velocity = mbs.GetObjectOutputBody(1, localPosition=[0, 0, 0],
                                                       variableType=exu.OutputVariableType.AngularVelocityLocal)

            alpha, beta = compute_alpha_beta(R_wb=rotation_matrix.T, v_world=body_fluid_velocity_in_world)

            # Reference dimensions used to compute go from dimensionless to dimesionful quantities
            ref_area = self.kite_chord * self.kite_span  # m^2
            ref_chord = self.kite_chord  # m
            ref_span = self.kite_span  # m

            # Coefficients
            aero_coeffs = AeroCoefficients()

            # Aerodynamics model
            aero_model = AircraftAerodynamics(ref_area, ref_chord, ref_span, aero_coeffs)

            # Example conditions
            angular_velocities = {"p": angular_velocity[0], "q": angular_velocity[1], "r": angular_velocity[2]}  # rad/s
            control_inputs = {"aileron": 0.0, "rudder": 0.0}  # rad



            # Compute forces and moments
            forces_moments = aero_model.compute_forces_and_moments(rho=self.fluid_volumetric_mass,
                                                                   velocity=np.linalg.norm(body_fluid_velocity_in_world,
                                                                                           2),
                                                                   alpha=alpha,
                                                                   beta=beta,
                                                                   ang_vel=angular_velocities,
                                                                   control_surfaces=control_inputs)

            force_in_windaero_frame = np.array([forces_moments['F_x'], forces_moments['F_y'], forces_moments['F_z']])
            torque_in_windaero_frame = np.array([forces_moments['M_x'], forces_moments['M_y'], forces_moments['M_z']])
            # rotate from wind frame to body frame

            force_fsd_body_frame = np.squeeze(
                np.matmul(RotXYZ2RotationMatrix([0, alpha, beta]).T, np.reshape(force_in_windaero_frame, (3, 1))))

            torque_fsd_body_frame = np.squeeze(
                np.matmul(RotXYZ2RotationMatrix([0, alpha, beta]).T, np.reshape(torque_in_windaero_frame, (3, 1))))

            return force_fsd_body_frame, torque_fsd_body_frame

        def wind_wrench_force_wrapper(mbs, t: float, loadVector: np.ndarray) -> np.ndarray:
            """
            Wrapper function to apply aerodynamic forces via wind_wrench.

            Args:
                mbs: Multi-body system instance.
                t: Current time in the simulation.
                loadVector: Placeholder (not used).

            Returns:
                np.ndarray: The computed force or torque for the specific context.
            """
            force, torque = wind_wrench(mbs, t, loadVector)
            # Return only force for forces, and torque for torques
            return force

        def wind_wrench_torque_wrapper(mbs, t: float, loadVector: np.ndarray) -> np.ndarray:
            """
            Wrapper function to apply aerodynamic torque via wind_wrench.

            Args:
                mbs: Multi-body system instance.
                t: Current time in the simulation.
                loadVector: Placeholder (not used).

            Returns:
                np.ndarray: The computed force or torque for the specific context.
            """
            force, torque = wind_wrench(mbs, t, loadVector)
            # Return only force for forces, and torque for torques
            return torque

        # Add aerodynamic forces to the kite
        self.mbs.CreateForce(
            bodyNumber=self.kite_body,
            localPosition=[0, 0, 0],
            loadVectorUserFunction=lambda mbs, t, _: wind_wrench_force_wrapper(mbs,t, [0,0,0]),
            bodyFixed=True
        )

        # Add aerodynamic torques to the kite
        self.mbs.CreateTorque(
            bodyNumber=self.kite_body,
            localPosition=[0, 0, 0],
            loadVectorUserFunction=lambda mbs, t, _: wind_wrench_torque_wrapper(mbs, t, [0,0, 0]),
            bodyFixed=True
        )

        def drag_force(mbs, t: float, loadVector: np.array) -> np.array:
            """
            Compute the aerodynamic drag of the line (single point model)
            x forward
            y to starboard
            z down

            Args:
                mbs: multi-body system
                t (float): time, unused, kept to match interface
                loadVector (np.array): unused, to be kept to match interface

            Returns:
                np.array: vector Fx, Fy, Fz in aerodynamic convention in world frame
            """
            body_velocity_in_world = mbs.GetObjectOutputBody(self.line_body, localPosition=self.mid_line_point,
                                                             variableType=exu.OutputVariableType.Velocity)

            body_fluid_velocity = body_velocity_in_world - self.wind_velocity

            drag_coefficient = 0.5
            area = self.line_diameter * self.line_length  # Approximate frontal area

            force = -0.5 * self.fluid_volumetric_mass * area * drag_coefficient * np.linalg.norm(body_fluid_velocity, 2) * body_fluid_velocity
            return force

        # Add aerodynamic torques to the kite
        self.mbs.CreateTorque(
            bodyNumber=self.line_body,
            localPosition=[0, 0, 0],
            loadVectorUserFunction=drag_force,
            bodyFixed=True
        )
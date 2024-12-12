# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  3D rigid body tutorial with 2 bodies and revolute joints, using mbs.Create functions throughout;
#           Follows online tutorial without markers
#
# Author:   Johannes Gerstmayr
# Date:     2023-05-16
# Modified: 2024-06-04
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
from typing import Tuple

import exudyn as exu
# to be sure to have all items and functions imported, just do:
# from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics  # only import if it does not conflict
import numpy as np
from exudyn.rigidBodyUtilities import RotXYZ2RotationMatrix
from exudyn.utilities import InertiaCuboid, SensorBody

from aerodynamic_forces_model_6dof import AeroCoefficients, AircraftAerodynamics
from compute_alpha_beta import compute_alpha_beta

CONSTRAIN_TO_2D = False

SC = exu.SystemContainer()
mbs = SC.AddSystem()

# %%++++++++++++++++++++++++++++++++++++++++++++++++++++
# physical parameters
g = [0, 0, 9.81]  # gravity (aeronautic convention, z down)
up = -1

# environmental parameters
rho = 1.2  # Water density in kg/m^3
wind_air_ground_velocity_in_world = np.array([-20, 0, 0])  # Wind speed in m/s

#### Create bodies ##########

# ground body, located at specific position (there could be several ground objects)
oGround = mbs.CreateGround(referencePosition=[0, 0, 0])


anchor_point = [0, 0, 0]  # kite attachment point
line_length = 24  # length
w = 0.001  # width
line_density = 5000
line_mass = line_length * w ** 2 * line_density
bodyDim = [w, w, line_length]  # body dimensions
mid_line_point = anchor_point + np.array([0, 0, line_length * 0.5 * up])  # center of mass, body0
# %%++++++++++++++++++++++++++++++++++++++++++++++++++++
# first link:
iCube0 = InertiaCuboid(density=line_density, sideLengths=bodyDim)
iCube0 = iCube0.Translated(mid_line_point)  # transform COM, COM not at reference point!

# graphics for body
magnifying_factor = 30
graphicsBody0 = graphics.Brick(centerPoint=[0, 0, 0], size=[magnifying_factor * w, magnifying_factor * w , line_length], color=graphics.color.red)
graphicsCOM0 = graphics.Basis(origin=iCube0.com, length=2*magnifying_factor* w)  # COM frame

# create rigid node and body
line_body = mbs.CreateRigidBody(inertia=iCube0,  # includes COM
                                referencePosition=mid_line_point,
                                gravity=g,
                                graphicsDataList=[graphicsCOM0, graphicsBody0])

# create rigid node and kite body
kite_chord = 2 # m
kite_span = 5 # m
kite_mass = 5 # kg
thickness = 0.0001 # m
kite_density = kite_mass / (kite_chord * kite_span * thickness)
angle_of_key = -np.radians(10) # This is needed for a kite, otherwise it would stall and fly backward to the ground
kite_dim = [kite_chord, kite_span, thickness]  # body dimensions
iCubeKite = InertiaCuboid(density=kite_density, sideLengths=bodyDim)
graphicsKite = graphics.Brick(centerPoint=[0, 0, 0], size=kite_dim, color=graphics.color.blue)
graphicsCOMKite = graphics.Basis(origin=iCubeKite.com, length=kite_span)  # COM frame
kite_body = mbs.CreateRigidBody(inertia=iCube0,  # includes COM
                                referencePosition=[0, 0, line_length*up],
                                referenceRotationMatrix=RotXYZ2RotationMatrix([0, angle_of_key, 0] ),
                                gravity=g,
                                graphicsDataList=[graphicsCOMKite, graphicsKite])

#### Create joints ##########
if CONSTRAIN_TO_2D:
    y_axis = [0, 1, 0]
    mbs.CreateRevoluteJoint(bodyNumbers=[oGround, line_body], position=anchor_point,
                            axis= y_axis, axisRadius=0.2 * magnifying_factor* w, axisLength=1.4 *magnifying_factor* w)
else:
    mbs.CreateSphericalJoint(bodyNumbers=[oGround, line_body], position=anchor_point)


mbs.CreateGenericJoint(bodyNumbers=[line_body, kite_body], position=[0, 0, line_length * up], )

#### Create forces ##########

def drag_force(mbs, t:float, loadVector:np.array)->np.array:
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
    body_velocity_in_world = mbs.GetObjectOutputBody(line_body, localPosition=mid_line_point,
                                                     variableType=exu.OutputVariableType.Velocity)

    body_fluid_velocity = body_velocity_in_world - wind_air_ground_velocity_in_world

    drag_coefficient = 0.5
    area = w * line_length  # Approximate frontal area

    force = -0.5 *rho *area * drag_coefficient  * np.linalg.norm(body_fluid_velocity, 2) * body_fluid_velocity
    return force
#%% Add Wind Force
def wind_wrench(mbs, t:float, loadVector) -> Tuple[np.array, np.array]:
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

    body_velocity_in_world = mbs.GetObjectOutputBody(kite_body, localPosition=[0, 0, 0],
                                                     variableType=exu.OutputVariableType.Velocity)

    body_fluid_velocity_in_world = body_velocity_in_world - wind_air_ground_velocity_in_world

    # Get rotation matrix
    rotation_matrix = np.reshape(mbs.GetObjectOutputBody(kite_body, localPosition=[0, 0, 0],
                                                         variableType=exu.OutputVariableType.RotationMatrix), [3, 3])

    # Get angular velocity, which play an important role in damping motion
    angular_velocity = mbs.GetObjectOutputBody(1, localPosition=[0, 0, 0],
                            variableType=exu.OutputVariableType.AngularVelocityLocal)


    alpha, beta = compute_alpha_beta(R_wb = rotation_matrix.T, v_world = body_fluid_velocity_in_world)

    # Reference dimensions used to compute go from dimensionless to dimesionful quantities
    ref_area = kite_chord * kite_span  # m^2
    ref_chord = kite_chord  # m
    ref_span = kite_span  # m

    # Coefficients
    aero_coeffs = AeroCoefficients()

    # Aerodynamics model
    aero_model = AircraftAerodynamics(ref_area, ref_chord, ref_span, aero_coeffs)

    # Example conditions
    angular_velocities = {"p": angular_velocity[0], "q": angular_velocity[1], "r": angular_velocity[2]}  # rad/s
    control_inputs = {"aileron": 0.0, "rudder": 0.0}  # rad

    # Compute forces and moments
    forces_moments = aero_model.compute_forces_and_moments(rho=rho,
                                                           velocity=np.linalg.norm(body_fluid_velocity_in_world, 2),
                                                           alpha=alpha,
                                                           beta=beta,
                                                           ang_vel=angular_velocities,
                                                           control_surfaces=control_inputs)



    force_in_windaero_frame =  np.array( [forces_moments['F_x'], forces_moments['F_y'], forces_moments['F_z']])
    torque_in_windaero_frame = np.array([forces_moments['M_x'], forces_moments['M_y'], forces_moments['M_z']])
    # rotate from wind frame to body frame

    force_fsd_body_frame = np.squeeze(np.matmul(RotXYZ2RotationMatrix([0, alpha, beta]).T, np.reshape(force_in_windaero_frame, (3, 1))))

    torque_fsd_body_frame = np.squeeze(np.matmul(RotXYZ2RotationMatrix([0, alpha, beta]).T, np.reshape(torque_in_windaero_frame, (3, 1))))

    return force_fsd_body_frame, torque_fsd_body_frame

def wind_force(mbs, t:float, loadVector:np.array)->np.array:
    """
    Compute the aerodynamic force vector
    x forward
    y to starboard
    z down

    Args:
        mbs: multi-body system
        t (float): time, unused, kept to match interface
        loadVector (np.array): unused, to be kept to match interface

    Returns:
        np.array: vector Fx, Fy, Fz in aerodynamic convention in body frame
    """
    force, torque = wind_wrench(mbs, t, loadVector)
    return force

def wind_torque(mbs, t:float, loadVector:np.array)->np.array:
    """
    Compute the aerodynamic torque vector
    x forward
    y to starboard
    z down

    Args:
        mbs: multi-body system
        t (float): time, unused, kept to match interface
        loadVector (np.array): unused, to be kept to match interface

    Returns:
        np.array: vector Mx, My, Mz in aerodynamic convention in body frame
    """
    force, torque = wind_wrench(mbs, t, loadVector)
    return torque

mbs.CreateForce(
    bodyNumber=kite_body,
    localPosition=[0, 0, 0],
    loadVectorUserFunction=wind_force,
    bodyFixed=True
)
mbs.CreateTorque(
    bodyNumber=kite_body,
    localPosition=[0, 0, 0],
    loadVectorUserFunction=wind_torque,
    bodyFixed=True
)

mbs.CreateForce(
    bodyNumber=line_body,
    localPosition=mid_line_point,
    loadVectorUserFunction=drag_force,
    bodyFixed=False
)

# position sensor on line
sens1 = mbs.AddSensor(SensorBody(bodyNumber=line_body, localPosition=mid_line_point,
                                 fileName='solution/sensorPos.txt',
                                 outputVariableType=exu.OutputVariableType.Position))

# %%++++++++++++++++++++++++++++++++++++++++++++++++++++++
# assemble system before solving
mbs.Assemble()

mbs.ComputeSystemDegreeOfFreedom(verbose=True)  # print out DOF and further information

simulationSettings = exu.SimulationSettings()  # takes currently set values or default values

tEnd = 10  # simulation time
h = 0.1  # step size
simulationSettings.timeIntegration.numberOfSteps = int(tEnd / h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.solutionSettings.solutionWritePeriod = 0.01  # store every 10 ms

SC.visualizationSettings.window.renderWindowSize = [1600, 1200]
SC.visualizationSettings.openGL.multiSampling = 4
SC.visualizationSettings.openGL.initialModelRotation = RotXYZ2RotationMatrix([np.pi/2, 0, np.pi])
SC.visualizationSettings.openGL.initialZoom = 0.2
SC.visualizationSettings.nodes.showBasis = True

# start solver
mbs.SolveDynamic(simulationSettings=simulationSettings,
                 solverType=exu.DynamicSolverType.TrapezoidalIndex2)

# load solution and visualize
mbs.SolutionViewer()

if True:
    mbs.PlotSensor(sensorNumbers=[sens1], components=[1], closeAll=True)

if True:
    mbs.DrawSystemGraph(useItemTypes=True)  # draw nice graph of system


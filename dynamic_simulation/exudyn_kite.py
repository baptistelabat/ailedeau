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

import exudyn as exu
from exudyn.rigidBodyUtilities import RotXYZ2RotationMatrix
from exudyn.utilities import InertiaCuboid, SensorBody
# to be sure to have all items and functions imported, just do:
# from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics  # only import if it does not conflict
import numpy as np

from aerodynamic_forces_model_6dof import AeroCoefficients, AircraftAerodynamics
from compute_alpha_beta import compute_alpha_beta, alpha_beta

CONSTRAIN_TO_2D = False

SC = exu.SystemContainer()
mbs = SC.AddSystem()

# %%++++++++++++++++++++++++++++++++++++++++++++++++++++
# physical parameters
g = [0, 0, 9.81]  # gravity (aeronautic convention, z down)
L = 24  # length
w = 0.001  # width
bodyDim = [w, w, L]  # body dimensions
p0 = [0, 0, 0]  # origin of pendulum
pMid0 = np.array([0, 0, -L*0.5])  # center of mass, body0

# ground body, located at specific position (there could be several ground objects)
oGround = mbs.CreateGround(referencePosition=[0, 0, 0])

# %%++++++++++++++++++++++++++++++++++++++++++++++++++++
# first link:
iCube0 = InertiaCuboid(density=5000, sideLengths=bodyDim)
iCube0 = iCube0.Translated([0, 0, -L*0.5])  # transform COM, COM not at reference point!

# graphics for body
graphicsBody0 = graphics.Brick(centerPoint=[0, 0, 0], size=[30*w, 30*w , L ], color=graphics.color.red)
graphicsCOM0 = graphics.Basis(origin=iCube0.com, length=2 * w)  # COM frame

# create rigid node and body
b0 = mbs.CreateRigidBody(inertia=iCube0,  # includes COM
                         referencePosition=pMid0,
                         gravity=g,
                         graphicsDataList=[graphicsCOM0, graphicsBody0])
# revolute joint (free z-axis), axis and position given in global coordinates
#  using reference configuration
if CONSTRAIN_TO_2D:
    mbs.CreateRevoluteJoint(bodyNumbers=[oGround, b0], position=[0, 0, 0],
                            axis=[0, 1, 0], axisRadius=0.2 * w, axisLength=1.4 * w)
else:
    mbs.CreateSphericalJoint(bodyNumbers=[oGround, b0], position=[0, 0, 0])

# create rigid node and kite body
chord = 2
span = 5
thickness=0.0001
angle_of_key= -np.radians(10)
kiteDim = [chord, span, thickness]  # body dimensions
iCubeKite = InertiaCuboid(density=5000, sideLengths=bodyDim)
graphicsKite = graphics.Brick(centerPoint=[0, 0, 0], size=[chord, span, thickness], color=graphics.color.blue)
graphicsCOMKite = graphics.Basis(origin=iCubeKite.com, length=span)  # COM frame
b1 = mbs.CreateRigidBody(inertia=iCube0,  # includes COM
                         referencePosition=[0, 0, -L],
                         referenceRotationMatrix=RotXYZ2RotationMatrix([0,angle_of_key, 0] ),
                         gravity=g,
                         graphicsDataList=[graphicsCOMKite, graphicsKite])

mbs.CreateGenericJoint(bodyNumbers=[b0, b1], position=[0, 0, L],)

rho = 1.2  # Water density in kg/m^3
wind_velocity = np.array([-20, 0, 0])  # Wind speed in m/s

#%% Add Wind Force
def WindWrench(mbs, t, loadVector):

    body_velocity_in_world = mbs.GetObjectOutputBody(b1, localPosition=[0, 0, 0],
                            variableType=exu.OutputVariableType.Velocity)

    body_fluid_velocity = body_velocity_in_world-wind_velocity

    rotation_matrix = np.reshape(mbs.GetObjectOutputBody(b1, localPosition=[0, 0, 0],
                            variableType=exu.OutputVariableType.RotationMatrix), [3, 3])

    angular_velocity = mbs.GetObjectOutputBody(1, localPosition=[0, 0, 0],
                            variableType=exu.OutputVariableType.AngularVelocityLocal)

    body_velocity_in_body = mbs.GetObjectOutputBody(b1, localPosition=[0, 0, 0],
                                                     variableType=exu.OutputVariableType.VelocityLocal)

    alpha, beta = compute_alpha_beta(R_wb = rotation_matrix.T, v_world = body_fluid_velocity)

    # alpha, beta = alpha_beta(v_body=body_velocity_in_body)
    if t>3:
        alpha=alpha


    # Reference dimensions
    ref_area = chord*span  # m^2
    ref_chord = chord  # m
    ref_span = span  # m

    # Coefficients
    aero_coeffs = AeroCoefficients()

    # Aerodynamics model
    aero_model = AircraftAerodynamics(ref_area, ref_chord, ref_span, aero_coeffs)

    # Example conditions
    angular_velocities = {"p": angular_velocity[0], "q": angular_velocity[1], "r": angular_velocity[2]}  # rad/s
    control_inputs = {"aileron": 0.0, "rudder": 0.1}  # rad

    # Compute forces and moments
    forces_moments = aero_model.compute_forces_and_moments(rho=rho,
                                                           velocity=np.linalg.norm(body_fluid_velocity, 2),
                                                           alpha=alpha-angle_of_key,
                                                           beta=beta,
                                                           ang_vel=angular_velocities,
                                                           control_surfaces=control_inputs,
                                                           angle_of_key = -np.radians(10))



    force_in_windaero_frame =  np.array( [forces_moments['F_x'], forces_moments['F_y'], forces_moments['F_z']])
    torque_in_windaero_frame = np.array([forces_moments['M_x'], forces_moments['M_y'], forces_moments['M_z']])
    # rotate from wind frame to body frame

    force_fsd_body_frame = np.squeeze(np.matmul(RotXYZ2RotationMatrix([0, alpha, beta]).T, np.reshape(force_in_windaero_frame, (3, 1))))

    torque_fsd_body_frame = np.squeeze(np.matmul(RotXYZ2RotationMatrix([0, alpha, beta]).T, np.reshape(torque_in_windaero_frame, (3, 1))))

    return force_fsd_body_frame, torque_fsd_body_frame

def WindForce(mbs, t, loadVector):
    force, torque = WindWrench(mbs, t, loadVector)
    return force
def WindTorque(mbs, t, loadVector):
    force, torque = WindWrench(mbs, t, loadVector)
    return torque


def drag_force(mbs, t, loadVector):
    body_velocity = mbs.GetObjectOutputBody(b0, localPosition=[0, -0.5*L, 0],
                            variableType=exu.OutputVariableType.Velocity)

    body_fluid_velocity = body_velocity-wind_velocity

    drag_coefficient = 0.5
    area = w * L  # Approximate frontal area

    force = -0.5 * drag_coefficient * rho * np.linalg.norm(body_fluid_velocity, 2)* area * body_fluid_velocity
    return force

mbs.CreateForce(
    bodyNumber=b1,
    localPosition=[0, 0, 0],  # Apply at the tip of the second link
    loadVectorUserFunction=WindForce,
    bodyFixed=True
)
mbs.CreateTorque(
    bodyNumber=b1,
    localPosition=[0, 0, 0],  # Apply at the tip of the second link
    loadVectorUserFunction=WindTorque,
    bodyFixed=True
)
mbs.CreateForce(
    bodyNumber=b0,
    localPosition=[0, 0, -0.5*L],  # Apply at the tip of the second link
    loadVector=[0, 0, 0],
    loadVectorUserFunction=drag_force,
    bodyFixed=False
)


# position sensor at tip of body1
sens1 = mbs.AddSensor(SensorBody(bodyNumber=b0, localPosition=[0, 0, 0.5 * L],
                                 fileName='solution/sensorPos.txt',
                                 outputVariableType=exu.OutputVariableType.Position))

# %%++++++++++++++++++++++++++++++++++++++++++++++++++++++
# assemble system before solving
mbs.Assemble()

mbs.ComputeSystemDegreeOfFreedom(verbose=True)  # print out DOF and further information

simulationSettings = exu.SimulationSettings()  # takes currently set values or default values

tEnd = 30  # simulation time
h = 0.01  # step size
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


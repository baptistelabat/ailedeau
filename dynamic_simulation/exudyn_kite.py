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
from exudyn.utilities import InertiaCuboid, SensorBody
# to be sure to have all items and functions imported, just do:
# from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics  # only import if it does not conflict
import numpy as np

from aerodynamic_forces_model_6dof import AeroCoefficients, AircraftAerodynamics
from compute_alpha_beta import compute_alpha_beta

SC = exu.SystemContainer()
mbs = SC.AddSystem()

# %%++++++++++++++++++++++++++++++++++++++++++++++++++++
# physical parameters
g = [0, -9.81, 0]  # gravity
L = 1  # length
w = 0.1  # width
bodyDim = [w, L, w]  # body dimensions
p0 = [0, 0, 0]  # origin of pendulum
pMid0 = np.array([0, -L*0.5, 0])  # center of mass, body0

# ground body, located at specific position (there could be several ground objects)
oGround = mbs.CreateGround(referencePosition=[0, 0, 0])

# %%++++++++++++++++++++++++++++++++++++++++++++++++++++
# first link:
iCube0 = InertiaCuboid(density=5000, sideLengths=bodyDim)
iCube0 = iCube0.Translated([0, 0.25*L, 0])  # transform COM, COM not at reference point!

# graphics for body
graphicsBody0 = graphics.Brick(centerPoint=[0, 0, 0], size=[0.1*w, L, 0.1*w], color=graphics.color.red)
graphicsCOM0 = graphics.Basis(origin=iCube0.com, length=2 * w)  # COM frame

# create rigid node and body
b0 = mbs.CreateRigidBody(inertia=iCube0,  # includes COM
                         referencePosition=pMid0,
                         gravity=g,
                         graphicsDataList=[graphicsCOM0, graphicsBody0])
# revolute joint (free z-axis), axis and position given in global coordinates
#  using reference configuration
mbs.CreateRevoluteJoint(bodyNumbers=[oGround, b0], position=[0, 0, 0],
                        axis=[0, 0, 1], axisRadius=0.2 * w, axisLength=1.4 * w)



#%% Add Wind Force
def WindForce(mbs, t, loadVector):
    rho = 1000 # Water density in kg/m^3
    body_velocity = mbs.GetObjectOutputBody(b0, localPosition=[0, -0.5*L, 0],
                            variableType=exu.OutputVariableType.Velocity)
    wind_velocity = np.array([5, 0, 0])  # Wind speed in m/s
    body_fluid_velocity = body_velocity-wind_velocity

    rotation_matrix = np.reshape(mbs.GetObjectOutputBody(b0, localPosition=[0, -0.5*L, 0],
                            variableType=exu.OutputVariableType.RotationMatrix), [3,3])

    angular_velocity = mbs.GetObjectOutputBody(b0, localPosition=[0, -0.5*L, 0],
                            variableType=exu.OutputVariableType.AngularVelocityLocal)

    alpha, beta = compute_alpha_beta(R_wb = rotation_matrix, v_world = -body_fluid_velocity)

    # Reference dimensions
    ref_area = 0.2  # m^2
    ref_chord = 0.2  # m
    ref_span = 1.0  # m

    # Coefficients
    aero_coeffs = AeroCoefficients()

    # Aerodynamics model
    aero_model = AircraftAerodynamics(ref_area, ref_chord, ref_span, aero_coeffs)

    # Example conditions
    angular_velocities = {"p": angular_velocity[0], "q": angular_velocity[2], "r": angular_velocity[1]}  # rad/s
    control_inputs = {"aileron": 0., "rudder": 0.0}  # rad

    # Compute forces and moments
    forces_moments = aero_model.compute_forces_and_moments(rho=rho,
                                                           velocity=np.linalg.norm(body_fluid_velocity, 2),
                                                           alpha=alpha + np.radians(10),
                                                           beta=beta,
                                                           ang_vel=angular_velocities,
                                                           control_surfaces=control_inputs)

    drag_coefficient = 0.5
    area = w * L  # Approximate frontal area

    force = -0.5 * drag_coefficient * rho * np.linalg.norm(body_fluid_velocity, 2)* area * body_fluid_velocity

    force = 0.*force + 1*np.array( [-forces_moments['F_x'], -forces_moments['F_y'],forces_moments['F_z']])
    return force  # Wind force acts in the global X direction

mbs.CreateForce(
    bodyNumber=b0,
    localPosition=[0, -0.5*L, 0],  # Apply at the tip of the second link
    loadVector=[0, 0, 0],
    loadVectorUserFunction=WindForce,
    bodyFixed=True
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

tEnd = 10  # simulation time
h = 1e-3  # step size
simulationSettings.timeIntegration.numberOfSteps = int(tEnd / h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.solutionSettings.solutionWritePeriod = 0.01  # store every 10 ms

SC.visualizationSettings.window.renderWindowSize = [1600, 1200]
SC.visualizationSettings.openGL.multiSampling = 4

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


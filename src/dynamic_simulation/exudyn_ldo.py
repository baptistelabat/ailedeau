import exudyn as exu
from exudyn.utilities import *  # includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics  # for visualization
import numpy as np
from scipy.special import cosdg, sindg

SC = exu.SystemContainer()
mbs = SC.AddSystem()

# Background
background = graphics.CheckerBoard(point=[0, 0, -0.1], size=100)
oGround = mbs.AddObject(ObjectGround(referencePosition=[0, 0, 0],
                                     visualization=VObjectGround(graphicsData=[background])))

# Cable parameters
L = 20  # length of each cable segment in m
E = 2e11  # Young's modulus of the cable in N/m^2
rho = 7800  # density of the cable in kg/m^3
b = 0.001  # width of rectangular cable in m
h = 0.001  # height of rectangular cable in m
A = b * h  # cross-sectional area in m^2
I = b * h**3 / 12  # second moment of area in m^4
I = I / 10  # Cable is more flexible than a rod

# ANCF Cable template
cableTemplate = Cable2D(
    physicsMassPerLength=rho * A,
    physicsBendingStiffness=E * I,
    physicsAxialStiffness=E * A,
    physicsBendingDamping=0.02 * E * I,
    useReducedOrderIntegration=0,
    visualization=VCable2D(drawHeight=h),
)

initial_angle_deg = 60
pull = 10

# Cable 1 (connects Ground to Body1)
positionOfNode0 = [0, 0, 0]  # Start of cable 1
positionOfNode1 = [L*sindg(initial_angle_deg), L*cosdg(initial_angle_deg), 0]  # End of cable 1
numberOfElements = 16
ancf1 = GenerateStraightLineANCFCable2D(
    mbs,
    positionOfNode0, positionOfNode1,
    numberOfElements,
    cableTemplate,
    massProportionalLoad=[0, -9.81, 0],  # Gravity
)

# Cable 2 (connects Body1 to Body2)
positionOfNode2 = [L*sindg(initial_angle_deg), L*cosdg(initial_angle_deg), 0]  # Start of cable 2
positionOfNode3 = [2*L*sindg(initial_angle_deg), 2*L*cosdg(initial_angle_deg), 0]  # End of cable 2
ancf2 = GenerateStraightLineANCFCable2D(
    mbs,
    positionOfNode2, positionOfNode3,
    numberOfElements,
    cableTemplate,
    massProportionalLoad=[0, -9.81, 0],  # Gravity
)

# Rigid Body 1 (connected to cable 1 and cable 2)
gBody = graphics.Brick(size=[h, h, h], color=graphics.color.red)
dictBody0 = mbs.CreateRigidBody(referencePosition=[0, 0, 0],
                                inertia=InertiaCuboid(1000, [h, h, h]),
                                graphicsDataList=[gBody],
                                create2D=True, returnDict=True)
dictBody1 = mbs.CreateRigidBody(referencePosition=[L*sindg(initial_angle_deg), L*cosdg(initial_angle_deg), 0],
                                inertia=InertiaCuboid(1000, [0.002, 0.017, 0.0025]),
                                graphicsDataList=[gBody],
                                create2D=True, returnDict=True)

# Rigid Body 2 (connected to cable 2)
dictBody2 = mbs.CreateRigidBody(referencePosition=[2*L*sindg(initial_angle_deg), 2*L*cosdg(initial_angle_deg), 0],
                                inertia=InertiaCuboid(1000, [h, h, h]),
                                graphicsDataList=[gBody],
                                create2D=True, returnDict=True)


# Connections for Cable 1
mANCFFirst1 = mbs.AddMarker(MarkerNodeRigid(nodeNumber=ancf1[0][0]))  # Start of cable 1
mANCFFirstEnd1 = mbs.AddMarker(MarkerNodeRigid(nodeNumber=ancf1[0][-1]))  # End of cable 1
mBody1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=dictBody1['bodyNumber'], localPosition=[0, 0, 0]))
mBody0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=dictBody0['bodyNumber'], localPosition=[0, 0, 0]))
# Joint between Cable 1 and Body 1
mbs.AddObject(GenericJoint(markerNumbers=[mANCFFirstEnd1, mBody1], constrainedAxes=[1, 1, 0, 0, 0, 1]))
mbs.AddObject(GenericJoint(markerNumbers=[mANCFFirst1, mBody0], constrainedAxes=[1, 1, 0, 0, 0, 1]))

# Connections for Cable 2
mANCFFirst2 = mbs.AddMarker(MarkerNodeRigid(nodeNumber=ancf2[0][0]))  # Start of cable 2
mANCFFirstEnd2 = mbs.AddMarker(MarkerNodeRigid(nodeNumber=ancf2[0][-1]))  # End of cable 2
mBody2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=dictBody2['bodyNumber'], localPosition=[0, 0, 0]))

# Joint between Cable 2 and Body 1
mbs.AddObject(GenericJoint(markerNumbers=[mANCFFirst2, mBody1], constrainedAxes=[1, 1, 0, 0, 0, 1]))

# Joint between Cable 2 and Body 2
mbs.AddObject(GenericJoint(markerNumbers=[mANCFFirstEnd2, mBody2], constrainedAxes=[1, 1, 0, 0, 0, 1]))
# === Pulling Load Function ===
def PullingLoad(mbs, t, itemIndex):
    # Pulling force applied to Body2
    return [pull*sindg(initial_angle_deg), pull*cosdg(initial_angle_deg), 0]  # Constant pulling force


# Define the integral error variable
integral_error_x = 0
integral_error_y = 0


# === Altitude Controller Function with Proportional, Integral, and Derivative Control ===
def AltitudeController(mbs, t, itemIndex):
    global integral_error_x, integral_error_y  # Use global variables for error accumulation

    k_p = 1  # Proportional gain
    k_i = 0.001  # Integral gain (to accumulate error)
    k_d = 0.5  # Damping gain (adjust for desired damping effect)

    # Get position and velocity of the controlled object (Body0)
    x_position = mbs.GetMarkerOutput(controlMarker, exu.OutputVariableType.Position)[0]  # x-coordinate
    y_position = mbs.GetMarkerOutput(controlMarker, exu.OutputVariableType.Position)[1]  # y-coordinate
    x_velocity = mbs.GetMarkerOutput(controlMarker, exu.OutputVariableType.Velocity)[0]  # x-velocity
    y_velocity = mbs.GetMarkerOutput(controlMarker, exu.OutputVariableType.Velocity)[1]  # y-velocity

    # Proportional control force (based on position)
    force_proportional = [-k_p * x_position, -k_p * y_position, 0]

    # Integral control force (sum of position errors over time)
    integral_error_x += x_position * h  # Accumulate error over time step (h is the timestep)
    integral_error_y += y_position * h  # Accumulate error over time step (h is the timestep)

    force_integral = [-k_i * integral_error_x, -k_i * integral_error_y, 0]

    # Damping control force (based on velocity)
    force_damping = [-k_d * x_velocity, -k_d * y_velocity, 0]

    # Return the combined control force (proportional + integral + damping)
    pid = [force_proportional[i] + force_integral[i] + force_damping[i] for i in range(3)]
    return np.array(pid) -[pull*sindg(initial_angle_deg), pull*cosdg(initial_angle_deg), 0]


# Apply pulling load at Body2
pullMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=dictBody2['bodyNumber'], localPosition=[0, 0, 0]))
mbs.AddLoad(LoadForceVector(markerNumber=pullMarker, loadVectorUserFunction=PullingLoad))

# Apply altitude control load at Body1
controlMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=dictBody0['bodyNumber'], localPosition=[0, 0, 0]))
mbs.AddLoad(LoadForceVector(markerNumber=controlMarker, loadVectorUserFunction=AltitudeController))

# Assemble and solve the static equilibrium before starting dynamics
mbs.Assemble()
mbs.SolveStatic()  # Solving for static equilibrium

# Setup for dynamic simulation
simulationSettings = exu.SimulationSettings()
tEnd = 50
h = 2e-3
simulationSettings.timeIntegration.numberOfSteps = int(tEnd / h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.solutionSettings.writeSolutionToFile = True
simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime / 1000
simulationSettings.displayComputationTime = False
simulationSettings.timeIntegration.verboseMode = 1

simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-6
simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse

SC.visualizationSettings.nodes.defaultSize = 0.1

# Solving the dynamic simulation
mbs.SolveDynamic(simulationSettings)
mbs.SolutionViewer()

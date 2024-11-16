import exudyn as exu
from exudyn.utilities import *  # includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics  # for visualization
import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()

# Background
background = graphics.CheckerBoard(point=[0, 0, -0.1], size=5)
oGround = mbs.AddObject(ObjectGround(referencePosition=[0, 0, 0],
                                     visualization=VObjectGround(graphicsData=[background])))

# Cable parameters
L = 2  # length of each cable segment in m
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

# Cable 1 (connects Ground to Body1)
positionOfNode0 = [0, 0, 0]  # Start of cable 1
positionOfNode1 = [L, 0, 0]  # End of cable 1
numberOfElements = 16
ancf1 = GenerateStraightLineANCFCable2D(
    mbs,
    positionOfNode0, positionOfNode1,
    numberOfElements,
    cableTemplate,
    massProportionalLoad=[0, -9.81, 0],  # Gravity
)

# Cable 2 (connects Body1 to Body2)
positionOfNode2 = [L, 0, 0]  # Start of cable 2
positionOfNode3 = [2 * L, 0, 0]  # End of cable 2
ancf2 = GenerateStraightLineANCFCable2D(
    mbs,
    positionOfNode2, positionOfNode3,
    numberOfElements,
    cableTemplate,
    massProportionalLoad=[0, -9.81, 0],  # Gravity
)

# Rigid Body 1 (connected to cable 1 and cable 2)
gBody = graphics.Brick(size=[h, h, h], color=graphics.color.red)
dictBody1 = mbs.CreateRigidBody(referencePosition=[L, 0, 0],
                                inertia=InertiaCuboid(1000, [h, h, h]),
                                graphicsDataList=[gBody],
                                create2D=True, returnDict=True)

# Rigid Body 2 (connected to cable 2)
dictBody2 = mbs.CreateRigidBody(referencePosition=[2 * L, 0, 0],
                                inertia=InertiaCuboid(1000, [h, h, h]),
                                graphicsDataList=[gBody],
                                create2D=True, returnDict=True)

# Connections for Cable 1
mANCFFirst1 = mbs.AddMarker(MarkerNodeRigid(nodeNumber=ancf1[0][0]))  # Start of cable 1
mANCFFirstEnd1 = mbs.AddMarker(MarkerNodeRigid(nodeNumber=ancf1[0][-1]))  # End of cable 1
mBody1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=dictBody1['bodyNumber'], localPosition=[0, 0, 0]))

# Joint between Cable 1 and Body 1
mbs.AddObject(GenericJoint(markerNumbers=[mANCFFirstEnd1, mBody1], constrainedAxes=[1, 1, 0, 0, 0, 1]))

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
    if t < 2:
        return [0, 0, 0]  # No force at the beginning
    elif t < 6:
        return [0*1000 * (t-2), 0, 0]  # Linearly increasing pulling force
    else:
        return [0*4000, 0, 0]  # Constant pulling force

# === Altitude Controller Function ===
def AltitudeController(mbs, t, itemIndex):
    k_p = 10  # Proportional gain
    y_position = mbs.GetMarkerOutput(controlMarker, exu.OutputVariableType.Position)[1]  # y-coordinate
    return [0, -k_p * y_position, 0]  # Force proportional to altitude

# Apply pulling load at Body2
pullMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=dictBody2['bodyNumber'], localPosition=[0, 0, 0]))
# mbs.AddLoad(LoadForceVector(markerNumber=pullMarker, loadVectorUserFunction=PullingLoad))

# Apply altitude control load at Body1
controlMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=dictBody2['bodyNumber'], localPosition=[0, 0, 0]))
mbs.AddLoad(LoadForceVector(markerNumber=controlMarker, loadVectorUserFunction=AltitudeController))

# Assemble and Solve
mbs.Assemble()
simulationSettings = exu.SimulationSettings()

tEnd = 10
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

SC.visualizationSettings.nodes.defaultSize = 0.01

mbs.SolveDynamic(simulationSettings)
mbs.SolutionViewer()

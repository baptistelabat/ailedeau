import exudyn as exu
from exudyn.itemInterface import *
import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()

# Background for the ground object (optional for visualization)
rect = [-2, -2, 4, 2]  # xmin, ymin, xmax, ymax
background0 = {'type': 'Line', 'color': [0.1, 0.1, 0.8, 1],
               'data': [rect[0], rect[1], 0, rect[2], rect[1], 0, rect[2], rect[3], 0, rect[0], rect[3], 0, rect[0],
                        rect[1], 0]}  # Background
background1 = {'type': 'Line', 'color': [0.1, 0.1, 0.8, 1], 'data': [0, -1, 0, 2, -1, 0]}  # Background
oGround = mbs.AddObject(
    ObjectGround(referencePosition=[0, 0, 0], visualization=VObjectGround(graphicsData=[background0, background1])))

# Cable parameters
L = 2  # Length of each ANCF element in meters
E = 2.07e11  # Young's modulus of ANCF element in N/m^2
rho = 7800  # Density of ANCF element in kg/m^3
b = 0.001  # Width of rectangular ANCF element in meters
h = 0.001  # Height of rectangular ANCF element in meters
A = b * h  # Cross-sectional area in m^2
I = b * h ** 3 / 12  # Second moment of area in m^4
f = 3 * E * I / L ** 2  # Tip load applied to the ANCF element in N

# Number of elements per cable
nElements = 10  # Number of elements per strand (more elements will make the simulation finer)

# Create cables and nodes for 3 strands (braiding)
cableList = []  # List of cable objects
nodeList = []  # List of nodes for the cables

# Create nodes for each strand and add cable elements
for strand_idx in range(3):  # Create 3 strands (adjust number as needed)
    nodeListForStrand = []  # Nodes for each strand
    nc0 = mbs.AddNode(Point2DS1(referenceCoordinates=[0.5 * strand_idx, 0, 1, 0]))  # Starting point of the strand
    nodeListForStrand.append(nc0)

    # Add cable elements along the length of each strand
    lElem = L / nElements
    for i in range(nElements):
        nLast = mbs.AddNode(Point2DS1(referenceCoordinates=[lElem * (i + 1) + 0.5 * strand_idx, 0, 1, 0]))
        nodeListForStrand.append(nLast)
        # Add the Cable2D element between nodes
        elem = mbs.AddObject(Cable2D(physicsLength=lElem, physicsMassPerLength=rho * A,
                                     physicsBendingStiffness=E * I, physicsAxialStiffness=E * A,
                                     nodeNumbers=[int(nc0) + i, int(nc0) + i + 1]))
        cableList.append(elem)

    # Add the starting and ending markers
    mANCF0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nc0, coordinate=0))
    mANCF1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nc0, coordinate=1))
    mANCF2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nc0, coordinate=3))
    mbs.AddObject(CoordinateConstraint(markerNumbers=[oGround, mANCF0]))
    mbs.AddObject(CoordinateConstraint(markerNumbers=[oGround, mANCF1]))
    mbs.AddObject(CoordinateConstraint(markerNumbers=[oGround, mANCF2]))

# Add gravity to the nodes
for i in range(len(nodeList)):
    m = mbs.AddMarker(MarkerNodePosition(nodeNumber=nodeList[i]))
    fact = 1  # Add (half) weight of two elements to node
    if (i == 0) | (i == len(nodeList) - 1):
        fact = 0.5  # First and last node only weighted half
    mbs.AddLoad(Force(markerNumber=m, loadVector=[0, -40 * 2 * rho * A * fact * lElem, 0]))  # Weight due to gravity

# Contact between the cables: You can add contact between cables if needed
cStiffness = 1e3
cDamping = 0.02 * cStiffness
for i in range(len(cableList)):
    # Define contact between cables (use an actual contact model here)
    mCable = mbs.AddMarker(MarkerBodyCable2DShape(bodyNumber=cableList[i], numberOfSegments=nElements))
    # Example: Create simple "contact" at specific cable intersections or a ground interaction
    if i == 0:
        mbs.AddObject(ObjectContactCoordinate(markerNumbers=[oGround, mCable],
                                              contactStiffness=cStiffness, contactDamping=cDamping, offset=-0.2))

# Assemble the system
mbs.Assemble()

# Simulation settings
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 10000
simulationSettings.timeIntegration.endTime = 0.05
simulationSettings.solutionSettings.writeSolutionToFile = True
simulationSettings.displayComputationTime = True
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8
simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-10

# Run the simulation
exu.StartRenderer()
mbs.SolveDynamic(simulationSettings)
SC.WaitForRenderEngineStopFlag()
exu.StopRenderer()

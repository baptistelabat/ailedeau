# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  ANCFCable test with many cable elements
#
# Author:   Johannes Gerstmayr
# Date:     2023-10-15
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *  # includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics  # only import if it does not conflict
from exudyn.beams import *

import numpy as np

# create an environment for mini example
SC = exu.SystemContainer()
mbs = SC.AddSystem()

oGround = mbs.AddObject(ObjectGround(referencePosition=[0, 0, 0]))
nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0, 0, 0]))

rhoA = 78.
EA = 100000.
EI = 200
length = 20

nCables = 3
for i in range(nCables):
    p0 = np.array([0, i, 0])
    p1 = p0 + [length, 0, 0]

    cable = ObjectANCFCable(
        physicsLength=length,
        physicsMassPerLength=rhoA,
        physicsBendingStiffness=EI * (0.5 + np.random.rand() * 0.25),
        physicsBendingDamping=200000,
        physicsAxialStiffness=EA,
        physicsAxialDamping=200000
    )

    ancf = GenerateStraightLineANCFCable(mbs=mbs,
                                         positionOfNode0=p0, positionOfNode1=p1,
                                         numberOfElements=12,  # converged to 4 digits
                                         cableTemplate=cable,  # this defines the beam element properties
                                         massProportionalLoad=[0, 0, 0],
                                         fixedConstraintsNode0=[1, 1, 1, 0, 1, 1],
                                         # add constraints for pos and rot (r'_y,r'_z)
                                         )
    ancfNodes = ancf[0]
    node = ancfNodes[-1]
    mANCF0 = mbs.AddMarker(MarkerNodePosition(nodeNumber=node))
    if i == 1:
        def braiding(mbs, t: float, loadVector: np.ndarray) -> np.ndarray:
            """
            Wrapper function to apply aerodynamic forces via wind_wrench.

            Args:
                mbs: Multi-body system instance.
                t: Current time in the simulation.
                loadVector: Placeholder (not used).

            Returns:
                np.ndarray: The computed force or torque for the specific context.
            """
            T = 10
            return [0, 100 * np.sin(2 * np.pi * t / T + i * np.pi / 2), 100 * np.cos(2 * np.pi * t / T + i * np.pi / 2)]


        mbs.AddLoad(Force(markerNumber=mANCF0, loadVectorUserFunction=braiding))
    else:
        def braiding1(mbs, t: float, loadVector: np.ndarray) -> np.ndarray:
            """
            Wrapper function to apply aerodynamic forces via wind_wrench.

            Args:
                mbs: Multi-body system instance.
                t: Current time in the simulation.
                loadVector: Placeholder (not used).

            Returns:
                np.ndarray: The computed force or torque for the specific context.
            """
            T = 10
            return [0, 100 * np.sin(-2 * np.pi * t / T + i * np.pi / 2),
                    100 * np.cos(-2 * np.pi * t / T + i * np.pi / 2)]


        mbs.AddLoad(Force(markerNumber=mANCF0, loadVectorUserFunction=braiding1))
# Contact between the cables: You can add contact between cables if needed
# cStiffness = 1e3
# cDamping = 0.02 * cStiffness
# for i in range(len(cableList)):
#     # Define contact between cables (use an actual contact model here)
#     mCable = mbs.AddMarker(MarkerBodyCable2DShape(bodyNumber=cableList[i], numberOfSegments=nElements))
#     # Example: Create simple "contact" at specific cable intersections or a ground interaction
#     # if i == 0:
#     #     mbs.AddObject(ObjectContactCoordinate(markerNumbers=[oGround, mCable],
#     #
# assemble and solve system for default parameters
mbs.Assemble()

endTime = 100
stepSize = 0.5e-2

simulationSettings = exu.SimulationSettings()

# simulationSettings.solutionSettings.writeSolutionToFile = False
simulationSettings.solutionSettings.solutionWritePeriod = 0.02  # data not used
simulationSettings.solutionSettings.binarySolutionFile = True
simulationSettings.solutionSettings.outputPrecision = 6  # float
simulationSettings.solutionSettings.sensorsWritePeriod = 0.002  # data not used
simulationSettings.timeIntegration.verboseMode = 1  # turn off, because of lots of output
simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
simulationSettings.parallel.numberOfThreads = 8
simulationSettings.displayComputationTime = True
simulationSettings.displayStatistics = True

simulationSettings.timeIntegration.numberOfSteps = int(endTime / stepSize)
simulationSettings.timeIntegration.endTime = endTime
simulationSettings.timeIntegration.newton.useModifiedNewton = True

# simulationSettings.timeIntegration.simulateInRealtime = True
# simulationSettings.timeIntegration.realtimeFactor = 0.5

SC.visualizationSettings.general.graphicsUpdateInterval = 0.02
SC.visualizationSettings.window.renderWindowSize = [1200, 1024]
SC.visualizationSettings.nodes.show = False
SC.visualizationSettings.loads.show = False
SC.visualizationSettings.connectors.show = False
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++

exu.StartRenderer()
# mbs.WaitForUserToContinue()

mbs.SolveDynamic(simulationSettings)

SC.WaitForRenderEngineStopFlag()
exu.StopRenderer()  # safely close rendering window!

# mbs.SolutionViewer()

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
# to be sure to have all items and functions imported, just do:
# from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import numpy as np
from exudyn.rigidBodyUtilities import RotXYZ2RotationMatrix

from dynamic_simulation.kite_system import KiteSystem

# Example usage with visualization and rendering:

# Initialize Exudyn system
SC = exu.SystemContainer()
mbs = SC.AddSystem()

anchor_point=[0, 0, 0]
attachment_body = mbs.CreateGround(referencePosition=anchor_point)

# Kite 1
kite1 = KiteSystem(
    mbs,
    attachment_body=attachment_body,
    anchor_point = anchor_point,
    line_length=24,
    line_density=5000,
    line_diameter=0.001,
    kite_chord=2,
    kite_span=5,
    kite_mass=5,
    fluid_volumetric_mass=1.2,
    angle_of_key=-np.radians(10),
    wind_velocity=np.array([-20, 0, 0]),
)

# Kite 2
kite2 = KiteSystem(
    mbs,
    attachment_body=attachment_body,
    anchor_point=[10, 0, 0],
    line_length=24,
    line_diameter = 0.001,
    line_density=5000,
    kite_chord=2,
    kite_span=5,
    kite_mass=5,
    fluid_volumetric_mass=1.2,
    angle_of_key=-np.radians(10),
    wind_velocity=np.array([-20, 0, 0]),
    great_roll_offset_deg=90,
    constrain_to_2d = False
)

# Assembly and simulation setup
mbs.Assemble()

# Simulation settings
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 1000
simulationSettings.timeIntegration.endTime = 30.0
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.solutionSettings.solutionWritePeriod = 0.01

# Visualization settings
SC.visualizationSettings.window.renderWindowSize = [1600, 1200]
SC.visualizationSettings.openGL.multiSampling = 4
SC.visualizationSettings.openGL.initialModelRotation = RotXYZ2RotationMatrix([np.pi / 2, 0, np.pi])
SC.visualizationSettings.openGL.initialZoom = 0.2
SC.visualizationSettings.nodes.showBasis = True

# Solve the dynamic simulation
mbs.SolveDynamic(simulationSettings=simulationSettings)

# Visualize solution and system graph
mbs.SolutionViewer()

if True:  # Optional: Plot results
    mbs.PlotSensor(sensorNumbers=[0], components=[1], closeAll=True)

if True:  # Draw system graph
    mbs.DrawSystemGraph(useItemTypes=True)

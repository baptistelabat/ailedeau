import mujoco
import mujoco_viewer
import numpy as np

# Load model
model = mujoco.MjModel.from_xml_path("mujoco_cable_system.xml")
data = mujoco.MjData(model)

# Viewer for visualization
viewer = mujoco_viewer.MujocoViewer(model, data)

# Simulation parameters
time_step = model.opt.timestep
simulation_time = 10  # seconds
steps = int(simulation_time / time_step)

# Controller variables
pull_force = 10  # pulling force magnitude

# Simulation loop
for _ in range(steps):
    # Apply pulling force to the actuator
    data.ctrl[0] = pull_force

    # Step simulation
    mujoco.mj_step(model, data)

    # Render simulation
    viewer.render()

viewer.close()

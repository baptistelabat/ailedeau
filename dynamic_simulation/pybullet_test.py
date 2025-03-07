import pybullet as p
import pybullet_data
import time

# Connect to PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # For plane and other models

# Create a plane
p.loadURDF("plane.urdf")

# Define pendulum parameters
length1 = 1.0  # Length of first arm
length2 = 1.0  # Length of second arm
mass1 = 1.0    # Mass of the first pendulum bob
mass2 = 1.0    # Mass of the second pendulum bob

# Load the pendulum bobs
bob1 = p.loadURDF("sphere.urdf", basePosition=[0, 0, 0], globalScaling=mass1)
bob2 = p.loadURDF("sphere.urdf", basePosition=[length1, 0, -1], globalScaling=mass2)

# Load the pendulum links as thin cylinders
link1 = p.loadURDF("cylinder.urdf", basePosition=[0.5 * length1, 0, -0.5], globalScaling=0.1)
link2 = p.loadURDF("cylinder.urdf", basePosition=[length1 + 0.5 * length2, 0, -1.5], globalScaling=0.1)

# Set the mass for each body
p.changeDynamics(bob1, -1, mass=mass1)
p.changeDynamics(bob2, -1, mass=mass2)

# Create hinge constraints for the pendulum
p.createConstraint(link1, -1, bob1, -1, p.JOINT_POINT2POINT, [0, 0, -0.5], [0, 0, 0], [0, 0, 0])
p.createConstraint(link2, -1, bob2, -1, p.JOINT_POINT2POINT, [0, 0, -0.5], [0, 0, -1], [0, 0, 0])

# Set gravity to zero
p.setGravity(0, 0, 0)

# Proportional controller gains
Kp = 10.0  # Proportional gain

# Simulation loop
while True:
    # Get the current positions of the bobs
    pos1, _ = p.getBasePositionAndOrientation(bob1)
    pos2, _ = p.getBasePositionAndOrientation(bob2)

    # Calculate the forces needed to keep the bobs floating
    force1 = Kp * (0 - pos1[2])  # Target height is 0
    force2 = Kp * (0 - pos2[2])  # Target height is 0

    # Apply forces to the ends of the pendulum
    p.applyExternalForce(bob1, -1, [0, 0, force1], [0, 0, 0], p.WORLD_FRAME)
    p.applyExternalForce(bob2, -1, [0, 0, force2], [0, 0, 0], p.WORLD_FRAME)

    # Step the simulation
    p.stepSimulation()
    time.sleep(1./240.)  # Sleep to match the simulation frame rate

import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

# Define the parameters
l1 = 1.0  # Length of the first pendulum
l2 = 1.0  # Length of the second pendulum
m1 = 1.0  # Mass of the first bob
m2 = 1.0  # Mass of the second bob
g = 9.81  # Gravitational acceleration

# Define the equations of motion for the double pendulum
def equations_of_motion(state, t):
    theta1, theta2, theta1_dot, theta2_dot = state

    # Derivatives
    delta = theta2 - theta1
    denom1 = (m1 + m2) * l1 - m2 * l1 * np.cos(delta)**2

    theta1_ddot = (-m2 * g * np.sin(theta2) * np.cos(delta) +
                   m2 * l2 * theta2_dot**2 * np.sin(delta) * np.cos(delta) +
                   (m1 + m2) * g * np.sin(theta1)) / denom1

    theta2_ddot = (l1 / l2 * theta1_ddot * np.cos(delta) -
                   g * np.sin(theta2) -
                   (l1 / denom1) * theta1_dot**2 * np.sin(delta)) / (1 - m2 / (m1 + m2) * np.cos(delta)**2)

    return [theta1_dot, theta2_dot, theta1_ddot, theta2_ddot]

# Initial conditions: [theta1, theta2, theta1_dot, theta2_dot]
initial_conditions = [np.pi / 4, np.pi / 4, 0, 0]

# Time vector for the simulation
time_span = np.linspace(0, 10, 240)

# Integrate the equations of motion
solution = odeint(equations_of_motion, initial_conditions, time_span)

# Plot results
plt.figure(figsize=(10, 5))
plt.subplot(211)
plt.plot(time_span, solution[:, 0], label='Theta1 (rad)')
plt.plot(time_span, solution[:, 1], label='Theta2 (rad)')
plt.title('Angles of the Pendulum')
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.legend()

plt.subplot(212)
plt.plot(time_span, solution[:, 2], label='Theta1_dot (rad/s)')
plt.plot(time_span, solution[:, 3], label='Theta2_dot (rad/s)')
plt.title('Angular Velocities of the Pendulum')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.legend()

plt.tight_layout()
plt.show()

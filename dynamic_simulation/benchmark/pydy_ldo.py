import numpy as np
from sympy import symbols, Matrix, diff, cos, sin
from scipy.integrate import odeint
import matplotlib.pyplot as plt

# Define symbolic variables
t = symbols('t')  # Time symbol
L = 20  # Length of the rod (in meters)
g = 9.81  # Gravitational acceleration (m/s^2)

# Define generalized coordinates for each body
theta1, phi1 = symbols('theta1 phi1')  # Angles for body 1 (azimuthal and polar)
theta2, phi2 = symbols('theta2 phi2')  # Angles for body 2 (azimuthal and polar)
theta3, phi3 = symbols('theta3 phi3')  # Angles for body 3 (azimuthal and polar)

# Define generalized velocities (angular velocities for each angle)
theta_dot1, phi_dot1 = symbols('theta_dot1 phi_dot1')
theta_dot2, phi_dot2 = symbols('theta_dot2 phi_dot2')
theta_dot3, phi_dot3 = symbols('theta_dot3 phi_dot3')

# Masses of the bodies
m1 = 1.0  # Mass of body 1
m2 = 1.0  # Mass of body 2
m3 = 1.0  # Mass of body 3

# Define positions of the bodies in 3D (spherical coordinates converted to Cartesian coordinates)
# Body 1
x1 = L * sin(phi1) * cos(theta1)
y1 = L * sin(phi1) * sin(theta1)
z1 = L * cos(phi1)

# Body 2
x2 = L * sin(phi2) * cos(theta2)
y2 = L * sin(phi2) * sin(theta2)
z2 = L * cos(phi2)

# Body 3
x3 = L * sin(phi3) * cos(theta3)
y3 = L * sin(phi3) * sin(theta3)
z3 = L * cos(phi3)

# Define velocities (derivatives of positions with respect to time)
v1 = Matrix([diff(x1, t), diff(y1, t), diff(z1, t)])  # Velocity of body 1
v2 = Matrix([diff(x2, t), diff(y2, t), diff(z2, t)])  # Velocity of body 2
v3 = Matrix([diff(x3, t), diff(y3, t), diff(z3, t)])  # Velocity of body 3

# Kinetic energy of the system
T1 = 0.5 * m1 * v1.dot(v1)  # Kinetic energy of body 1
T2 = 0.5 * m2 * v2.dot(v2)  # Kinetic energy of body 2
T3 = 0.5 * m3 * v3.dot(v3)  # Kinetic energy of body 3

# Potential energy of the system (gravitational potential energy)
U1 = m1 * g * z1
U2 = m2 * g * z2
U3 = m3 * g * z3

# Lagrangian (L = T - U)
Lagrangian = (T1 + T2 + T3) - (U1 + U2 + U3)

# Generalized coordinates and velocities
q = Matrix([theta1, phi1, theta2, phi2, theta3, phi3])
q_dot = Matrix([theta_dot1, phi_dot1, theta_dot2, phi_dot2, theta_dot3, phi_dot3])

# Calculate the equations of motion using the Euler-Lagrange equations
Lagrangian_q = [diff(Lagrangian, q_i) for q_i in q]
Lagrangian_q_dot = [diff(Lagrangian, q_dot_i) for q_dot_i in q_dot]
Lagrangian_q_dot_diff = [diff(L_dot, t) for L_dot in Lagrangian_q_dot]

# Equations of motion (Lagrange equations)
eq1 = Lagrangian_q_dot_diff[0] - Lagrangian_q[0]
eq2 = Lagrangian_q_dot_diff[1] - Lagrangian_q[1]
eq3 = Lagrangian_q_dot_diff[2] - Lagrangian_q[2]
eq4 = Lagrangian_q_dot_diff[3] - Lagrangian_q[3]
eq5 = Lagrangian_q_dot_diff[4] - Lagrangian_q[4]
eq6 = Lagrangian_q_dot_diff[5] - Lagrangian_q[5]

# Simplified equations of motion (these will be second-order ODEs)
equations_of_motion = [eq1, eq2, eq3, eq4, eq5, eq6]


# Convert symbolic equations to numerical functions
# Define a function to compute the right-hand side of the equations (for odeint)
def equations_of_motion_fn(y, t):
    theta1_val, phi1_val, theta2_val, phi2_val, theta3_val, phi3_val, \
        theta_dot1_val, phi_dot1_val, theta_dot2_val, phi_dot2_val, theta_dot3_val, phi_dot3_val = y

    # Update positions based on spherical coordinates
    x1_val = L * np.sin(phi1_val) * np.cos(theta1_val)
    y1_val = L * np.sin(phi1_val) * np.sin(theta1_val)
    z1_val = L * np.cos(phi1_val)

    x2_val = L * np.sin(phi2_val) * np.cos(theta2_val)
    y2_val = L * np.sin(phi2_val) * np.sin(theta2_val)
    z2_val = L * np.cos(phi2_val)

    x3_val = L * np.sin(phi3_val) * np.cos(theta3_val)
    y3_val = L * np.sin(phi3_val) * np.sin(theta3_val)
    z3_val = L * np.cos(phi3_val)

    # Compute velocities (derivatives of positions)
    v1_val = np.array([-L * np.cos(phi1_val) * np.cos(theta1_val) * theta_dot1_val - L * np.sin(phi1_val) * np.sin(
        theta1_val) * phi_dot1_val,
                       -L * np.cos(phi1_val) * np.sin(theta1_val) * theta_dot1_val + L * np.sin(phi1_val) * np.cos(
                           theta1_val) * phi_dot1_val,
                       L * np.cos(phi1_val) * np.cos(theta1_val) * phi_dot1_val])

    v2_val = np.array([-L * np.cos(phi2_val) * np.cos(theta2_val) * theta_dot2_val - L * np.sin(phi2_val) * np.sin(
        theta2_val) * phi_dot2_val,
                       -L * np.cos(phi2_val) * np.sin(theta2_val) * theta_dot2_val + L * np.sin(phi2_val) * np.cos(
                           theta2_val) * phi_dot2_val,
                       L * np.cos(phi2_val) * np.cos(theta2_val) * phi_dot2_val])

    v3_val = np.array([-L * np.cos(phi3_val) * np.cos(theta3_val) * theta_dot3_val - L * np.sin(phi3_val) * np.sin(
        theta3_val) * phi_dot3_val,
                       -L * np.cos(phi3_val) * np.sin(theta3_val) * theta_dot3_val + L * np.sin(phi3_val) * np.cos(
                           theta3_val) * phi_dot3_val,
                       L * np.cos(phi3_val) * np.cos(theta3_val) * phi_dot3_val])

    # Kinetic energy and potential energy (for the numerical solution)
    T1_val = 0.5 * m1 * np.dot(v1_val, v1_val)
    T2_val = 0.5 * m2 * np.dot(v2_val, v2_val)
    T3_val = 0.5 * m3 * np.dot(v3_val, v3_val)

    U1_val = m1 * g * z1_val
    U2_val = m2 * g * z2_val
    U3_val = m3 * g * z3_val

    # Total kinetic and potential energy
    T_total = T1_val + T2_val + T3_val
    U_total = U1_val + U2_val + U3_val

    # Equations of motion (simplified)
    dtheta1_dt = theta_dot1_val
    dphi1_dt = phi_dot1_val
    dtheta2_dt = theta_dot2_val
    dphi2_dt = phi_dot2_val
    dtheta3_dt = theta_dot3_val
    dphi3_dt = phi_dot3_val

    return [dtheta1_dt, dphi1_dt, dtheta2_dt, dphi2_dt, dtheta3_dt, dphi3_dt, 0, 0, 0, 0, 0, 0]


# Initial conditions (angles and velocities)
initial_conditions = [0, np.pi / 4, 0, np.pi / 3, 0, np.pi / 6, 0, 0, 0, 0, 0, 0]

# Time points for integration
time_points = np.linspace(0, 10, 500)

# Solve the system using odeint
solution = odeint(equations_of_motion_fn, initial_conditions, time_points)

# Extract positions and velocities
theta1_vals = solution[:, 0]
phi1_vals = solution[:, 1]
theta2_vals = solution[:, 2]
phi2_vals = solution[:, 3]
theta3_vals = solution[:, 4]
phi3_vals = solution[:, 5]

# Plot the results
plt.figure(figsize=(10, 5))

# Plot positions of the bodies
plt.subplot(1, 2, 1)
plt.plot(time_points, theta1_vals, label='theta1 (Body1)')
plt.plot(time_points, phi1_vals, label='phi1 (Body1)')
plt.plot(time_points, theta2_vals, label='theta2 (Body2)')
plt.plot(time_points, phi2_vals, label='phi2 (Body2)')
plt.plot(time_points, theta3_vals, label='theta3 (Body3)')
plt.plot(time_points, phi3_vals, label='phi3 (Body3)')
plt.xlabel('Time (s)')
plt.ylabel('Angles (rad)')
plt.legend()

# Plot velocities of the bodies
plt.subplot(1, 2, 2)
plt.plot(time_points, np.zeros_like(theta1_vals), label='theta_dot1 (Body1)')
plt.plot(time_points, np.zeros_like(phi1_vals), label='phi_dot1 (Body1)')
plt.plot(time_points, np.zeros_like(theta2_vals), label='theta_dot2 (Body2)')
plt.plot(time_points, np.zeros_like(phi2_vals), label='phi_dot2 (Body2)')
plt.plot(time_points, np.zeros_like(theta3_vals), label='theta_dot3 (Body3)')
plt.plot(time_points, np.zeros_like(phi3_vals), label='phi_dot3 (Body3)')
plt.xlabel('Time (s)')
plt.ylabel('Velocities (rad/s)')
plt.legend()

plt.tight_layout()
plt.show()

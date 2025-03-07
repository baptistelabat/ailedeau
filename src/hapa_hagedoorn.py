"""
According to page 28 in AYRS114
https://www.ayrs.org/repository/AYRS%20114.pdf
# Add formula derived by Hagedoorn
"""
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.patches import FancyArrowPatch, Arc

m=80 # Mass of aquaviator
g = 9.81 # Acceleration of gravity
M = m * g # Weight of aquaviator

lift_over_drag= 5
k = np.arctan(1/lift_over_drag)

foil_line_length =10
kite_line_length =30
foil_line_elevation = np.deg2rad(30)
kite_line_elevation = np.deg2rad(60)

# Point positions
aquaviator_position = foil_line_length * np.exp(1j * foil_line_elevation)
kite_position = aquaviator_position + kite_line_length * np.exp(1j * kite_line_elevation)

# Create the figure
fig, ax = plt.subplots()
ax.plot([0, aquaviator_position.real, kite_position.real], [0, aquaviator_position.imag, kite_position.imag],
        marker='o')

# Add arcs for the angles
def add_arc(center, radius, theta_start, theta_end, color='blue', label=None):
    # Draw the arc
    arc = Arc(center, 2 * radius, 2 * radius, angle=0,
              theta1=np.degrees(theta_start), theta2=np.degrees(theta_end),
              color=color, linewidth=2, linestyle='--')
    ax.add_patch(arc)

    # Add a arrow extending the arc
    arrow_start = (center[0] + radius * np.cos(theta_end - 0.001), center[1] + radius * np.sin(theta_end - 0.001))
    arrow_end = (center[0] + radius * np.cos(theta_end), center[1] + radius * np.sin(theta_end))
    ax.annotate('', xy=arrow_end, xytext=arrow_start, arrowprops=dict(color=color, width=10))

    # Add a label for the angle
    if label:
        label_pos = (center[0] + (radius) * np.cos((theta_start + theta_end) / 2) + 10,
                     center[1] + (radius) * np.sin((theta_start + theta_end) / 2))
        ax.text(label_pos[0], label_pos[1], label, fontsize=12, ha='center')


# Arc for the angle between the center and the aquaviator
add_arc((0, 0), foil_line_length, 0, foil_line_elevation, color='blue', label=r'Foil line elevation $y$')

# Arc for the angle between the aquaviator and the kite
add_arc((aquaviator_position.real, aquaviator_position.imag), kite_line_length, 0,
        kite_line_elevation, color='green', label=r'Kite line elevation $x$')
add_arc((aquaviator_position.real, aquaviator_position.imag), kite_line_length, kite_line_elevation,
        np.pi / 2, color='orange', label=r'Kite finesse angle $k$')

# Add labels for the points and the lengths
ax.text(aquaviator_position.real, aquaviator_position.imag , 'Aquaviator', fontsize=12, ha='center')
ax.text(0, 0, 'Foil', fontsize=12, ha='center', va='top')

# Add labels for the lengths
ax.text(aquaviator_position.real / 2, aquaviator_position.imag / 2 - 1, f'{foil_line_length} m', fontsize=12,
        ha='center')
ax.text((aquaviator_position.real + kite_position.real) / 2, (aquaviator_position.imag + kite_position.imag) / 2 + 1,
        f'{kite_line_length} m', fontsize=12, ha='center')

# Add the label "Kite" at the end of the second segment
ax.text(kite_position.real, kite_position.imag - 2, 'Kite', fontsize=12, ha='center')

# Adjust label positions and presentation
ax.set_title("Figure with angle arcs, arrows, and labels")
ax.set_aspect('equal')
ax.grid(True)

# Show the figure
plt.show()


foil_line_elevation_deg = [0, 15, 30, 45, 60]
for y_deg in foil_line_elevation_deg :
    y = np.radians(y_deg) # describing the angle of the line with horizontal at anchor point.

    # Force pulling on the rope at anchor point
    P = M /(np.cos(y)/np.tan(k) -np.sin(y))

    # Lift of the kite
    L = M/(1-np.tan(k)*np.tan(y))

    # Ratio betwwen current wind and wind necessary to take off
    speed_ratio= np.sqrt(1/(1-np.tan(k)*np.tan(y)))

    # Using trigonometric formula 1/cos(t) = (1+tan(t)**2)**(1/2)
    P_over_M = speed_ratio**2*(np.tan(k)**2+(1-1/speed_ratio**2)**2)**(1/2)# Formula from paper corrected (a power of two was missing)

    # From the formula above we can see that the better the lift over drag ratio, the lower the force
    # When the finesse goes to infinity we have
    # P_over_M = abs(speed_ratio**2-1)
    print(speed_ratio, P_over_M, P/M)

# Now that we have checked the formula used it directly
speed_ratios = np.arange(1, 5.1, 0.2)
force_ratios = []
for speed_ratio in speed_ratios :
    P_over_M = speed_ratio ** 2 * (np.tan(k) ** 2 + (1 - 1 / speed_ratio ** 2) ** 2) ** (
                1 / 2)  # Formula from paper corrected (a power of two was missing)
    force_ratios.append(P_over_M)

plt.plot(speed_ratios, force_ratios)
# plt.show()


"""
According to page 35 in AYRS 114
"""
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
for lift_over_drag in range(2, 10):
    k = np.arctan(1/lift_over_drag)
    v0 = 1
    q = np.radians(np.arange(0,180))
    va=np.sqrt(v0**2*np.cos(k)*np.sin(q)/((np.sin(q)**2-np.sin(k)**2)**(1/2) -np.tan(y)*np.sin(k)))
    ax.plot(q, va)
ax.grid(True)
ax.set_xlabel('')
ax.set_title("A line plot on a polar axis", va='bottom')
plt.show()


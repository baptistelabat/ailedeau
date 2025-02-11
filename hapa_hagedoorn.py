"""
According to page 28 in AYRS114
# Add formula derived by Hagedoorn
"""
import numpy as np
from matplotlib import pyplot as plt

m=100 # Mass of aquaviator
g = 9.81 # Acceleration of gravity
M = m * g # Weight of aquaviator

lift_over_drag= 5
k = np.arctan(1/lift_over_drag)

for y_deg in [0, 15, 30, 45, 60]:
    y = np.radians(y_deg) # descring the angle of the line with horizontal at anchor point.

    # Force pulling on the rope at anchor point
    P = M /(np.cos(y)/np.tan(k) -np.sin(y))

    # Lift of the kite
    L = M/(1-np.tan(k)*np.tan(y))

    # Ratio betwwen current wind and wind necessary to take off
    speed_ratio= np.sqrt(1/(1-np.tan(k)*np.tan(y)))

    # Using trigonometric formula 1/cos(t) = (1+tan(t)**2)**(1/2)
    P_over_M = speed_ratio**2*(np.tan(k)**2+(1-1/speed_ratio**2)**2)**(1/2)# Formula from paper correcteed (a power of two was missing)

    # From the formula above we can see that the better the lift over drag ratio, the lower the force
    # When the finesse goes to infinity we have
    # P_over_M = abs(speed_ratio**2-1)
    print(speed_ratio, P_over_M, P/M)

# Now that we have checked the formula used it directly
speed_ratios = np.arange(1, 5.1, 0.2)
force_ratios = []
for speed_ratio in speed_ratios :
    P_over_M = speed_ratio ** 2 * (np.tan(k) ** 2 + (1 - 1 / speed_ratio ** 2) ** 2) ** (
                1 / 2)  # Formula from paper correcteed (a power of two was missing)
    force_ratios.append(P_over_M)

plt.plot(speed_ratios, force_ratios)
plt.show()

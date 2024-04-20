from scipy.constants import knot
from numpy import degrees, arctan, sqrt

def incidence_plan(effective_aspect_ratio:float, lift_coefficient:float)->float:

    return (10+20/effective_aspect_ratio)*lift_coefficient

max_target_speed_kt = 40
max_target_speed = max_target_speed_kt * knot

max_finesse = 70 # Lift to drag ratio. This value seems very high
finesse_angle=arctan(1/max_finesse)

finesse_angle_deg = degrees(finesse_angle)

lift_coefficient_at_max_finesse = 0.32

effective_aspect_ratio = 3.3

foil_surface= 0.55 # m2

# By definition
drag_coefficient_at_max_finesse = lift_coefficient_at_max_finesse/max_finesse

incidence_of_plan = incidence_plan(effective_aspect_ratio, lift_coefficient_at_max_finesse)

# By definition
span=sqrt(effective_aspect_ratio*foil_surface)

fuselage_area = 0.7*foil_surface

stabilizers_surfaces = 0.25*foil_surface*(1-1/effective_aspect_ratio)

stabilizer_drag_coefficient = 0.03

# Induced drag formula
foil_drag_coefficient_3D=drag_coefficient_at_max_finesse + lift_coefficient_at_max_finesse^2/(np.pi* effective_aspect_ratio)

stabilizer_drag_coefficient_based_on_foil_surface=stabilizer_drag_coefficient*(fuselage_area+stabilizers_surfaces)/foil_surface

total_drag_coefficient_based_on_foil_surface = stabilizer_drag_coefficient_based_on_foil_surface + foil_drag_coefficient_3D+



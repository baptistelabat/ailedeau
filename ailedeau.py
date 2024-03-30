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

drag_coefficient_at_max_finesse = lift_coefficient_at_max_finesse/max_finesse

incidence_of_plan = incidence_plan(effective_aspect_ratio, lift_coefficient_at_max_finesse)

span=sqrt(effective_aspect_ratio*effective_aspect_ratio)




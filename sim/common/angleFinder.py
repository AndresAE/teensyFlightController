import numpy as np
from scipy.optimize import fsolve

# Given parameters
# thetaIn_d = 30.0  # Input angle in radians
# servoToHinge_in = 1.9
# servoHorn_in = 0.34
# hingeToCntrlHorn_in = 0.63
# ctrlHorn_in = 0.9
# coupler = 2.56

# thetaIn_d = 30.0  # Input angle in radians
# servoToHinge_in = 2.25
# servoHorn_in = 0.53
# hingeToCntrlHorn_in = 0.36
# ctrlHorn_in = 0.9
# coupler = 2.56

thetaIn_d = 30.0  # Input angle in radians
servoToHinge_in = 2.1
servoHorn_in = 0.53
hingeToCntrlHorn_in = 0.42
ctrlHorn_in = 0.9
coupler = 2.56

# Define the system of equations


def loop_closure(x, L1, L2, L3, L4, thetaIn_d):
    thetaCoupler_d = x[0]  # Coupler angle
    thetaOut_d = x[1]  # Output angle

    # Loop-closure equations
    F1 = L2*np.sin(np.radians(thetaIn_d)) + L3*np.cos(np.radians(thetaCoupler_d)
                                                      ) - L4*np.sin(np.radians(thetaOut_d)) - L1
    F2 = L2*np.cos(np.radians(thetaIn_d)) - L3 * \
        np.sin(np.radians(thetaCoupler_d)) - L4*np.cos(np.radians(thetaOut_d))
    return [F1, F2]


# Initial guess for theta3 and theta4 (degrees)
x0 = [0, thetaIn_d]

# compute L4 (control horn length) from hinge and control horn geometry
L4 = np.sqrt(hingeToCntrlHorn_in ** 2 + ctrlHorn_in ** 2)

# Solve using scipy's fsolve: pass extra parameters via `args`
solution = fsolve(loop_closure, x0, args=(
    servoToHinge_in, servoHorn_in, coupler, L4, thetaIn_d))

# Display results
print(f"Theta3: {solution[0]:.6f} degrees")
print(f"Theta4: {solution[1]:.6f} degrees")

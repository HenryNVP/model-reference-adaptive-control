# Physical parameters
r = 0.0975                         # Radius of driving wheel, [m]
W = 0.33284                        # Lateral wheel distance, [m]
M = 9                              # Mass of robot, [kg]
I = 0.2641                         # Moment of inertia of robot, [kg.m^2]

"""
Dynamic model parameters:
- Gear ratio n
- Torque constant km, [N.m/A]
- Lumped moment of inertia of DC motor J1 and driving wheel J2, [kg.m^2]
- Lumped viscous damping cefficient b1, b2, [N.s/m]
- Armature resistance R_a, [Ohm]
- Armature voltage of left/right DC motors Vr, Vl, [V]
"""

# Dynamic model parameters
inertia = 1.41                      # n^2 * J1 + J2, [kg.m^2]
viscous = 6.48                      # n^2 * b1 + b2 + km^2 * n^2 / Ra, [N.s/m]
kmn_Ra = 10.47                      # kmn_Ra = km * n / Ra, [N.m/V]
beta = kmn_Ra * r                   # [N.m^2/V]

# Starting state
x0 = 0
y0 = 0
theta0 = 0
v0 = 0
omega0 = 0

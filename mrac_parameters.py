import numpy as np
from scipy.linalg import expm
from robot_parameters import r, W, M, I, inertia, viscous, beta

# Sampling time
dt = 0.1

"""
Dynamic model
    d(xp(t)) / dt = Apc * xp(t) + beta * Lambda * u(t) (Eq. 1)
Control input: u(t) = [Vr ; Vl]

Discretization of (Eq. 1)
    xp(t + 1) = Ap * x(t) + Bp * u(t)
"""

a11_num = viscous
a22_num = viscous
a11_den = inertia + M * r**2 / 2
a22_den = inertia + (2 * I * r**2) / W**2

a11 = - a11_num / a11_den
a22 = - a22_num / a22_den

lambda11 = 1 / (2 * a11_den)
lambda22 = 1 / (W * a22_den)

Apc = np.array([[a11, 0], [0, a22]])
Lambda = np.array([[lambda11, 0], [0, lambda22]])

# Discretization of (Eq. 1)
Ap = expm(Apc * dt)
Bp = np.linalg.inv(Apc) @ (Ap - np.eye(Apc.shape[0])) @ (beta * Lambda)

"""
Model reference
    d(xm(t)) / dt = Amc * xm(t) + Bmc * vd(t) (Eq. 2)

Model reference parameters:
- Reference state vector xm
- Reference state Hurwitz matrix Amc
- Reference input matrix Bmc

Discretization of (Eq. 2)
    x(t + 1) = Am * x(t) + Bm * vd(t)
"""

Amc = np.array([[-10, 0], [0, -10]])  # Reference state Hurwitz matrix
Bmc = np.array([[10, 0], [0, 10]])    # Reference input matrix

# Discretization of (Eq. 2)
Am = expm(Amc * dt)
Bm = np.linalg.inv(Amc) @ (Am - np.eye(Amc.shape[0])) @ Bmc

"""
Adaptive controller
    u(t) = kx(t) * xp(t) + kv(t) * vd(t)         (Eq. 3)
    d(kx(t)) / dt = - beta * P * em(t) * xp'(t)  (Eq. 4)
    
Discretization of (Eq. 4)
    kx(t + 1) = kx(t) - beta * P * em(t) * xp'(t) * dt
"""

# Adaptive law parameters
P = np.diag([0.1, 2])                 # Positive definite matrix

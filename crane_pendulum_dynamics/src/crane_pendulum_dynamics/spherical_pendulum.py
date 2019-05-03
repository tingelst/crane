import numpy as np


def lyapunov_controller(t, q):
    kp = 1.0
    kd = 2.0

    phix, dphix, phiy, dphiy = q

    g = 9.81
    L = 1.0

    cx = np.cos(phix)
    sx = np.sin(phix)
    cy = np.cos(phiy)
    sy = np.sin(phiy)

    uy = - L*cy/cx*(kd*dphix + kp*phix) - 2*L*sy/cx + g*sx*sy**2/cx
    ux = L/cy*(kd*dphiy + kp*phiy) - L*sy*dphix**2 - uy*sx*sy/cy

    return np.array([ux, uy])


def spherical_pendulum_dyn(t, q):
    phix, dphix, phiy, dphiy = q

    g = 9.81
    L = 1.0

    cx = np.cos(phix)
    sx = np.sin(phix)
    cy = np.cos(phiy)
    sy = np.sin(phiy)

    ddx0, ddy0 = lyapunov_controller(t, q)

    ddphix = (-g*sx/L + 2.0*dphix*dphiy*sy)/cy + ddy0*cx/(L*cy)
    ddphiy = -g*cx*sy/L - (ddx0*cy + ddy0*sx*sy)/L

    return np.array([dphix, ddphix, dphiy, ddphiy])

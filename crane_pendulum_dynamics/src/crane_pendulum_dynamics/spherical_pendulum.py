import numpy as np





def spherical_pendulum_dyn(t, q, u=[0.0, 0.0]):
    phix, dphix, phiy, dphiy = q

    g = 9.81
    L = 1.0

    cx = np.cos(phix)
    sx = np.sin(phix)
    cy = np.cos(phiy)
    sy = np.sin(phiy)

    ddx0, ddy0 = u

    ddphix = (-g*sx/L + 2.0*dphix*dphiy*sy)/cy + ddy0*cx/(L*cy)
    ddphiy = -g*cx*sy/L - (ddx0*cy + ddy0*sx*sy)/L

    return np.array([dphix, ddphix, dphiy, ddphiy])

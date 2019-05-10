import numpy as np


def continuous_dynamics(z, g, k, L):

    kp, kd = k

    x0, dx0, y0, dy0, phix, dphix, phiy, dphiy = z

    cx = np.cos(phix)
    sx = np.sin(phix)
    cy = np.cos(phiy)
    sy = np.sin(phiy)

    gx, gy = g

    uy = - L*cy/cx*(kd*dphix + kp*phix) - 2.0*L*sy / \
        cx*dphix*dphiy + 9.81*sx*sy*sy/cx

    # Eq 51-54 in Ecc
    dzdt = np.zeros(8)
    dzdt[1] = dx0  # dx0
    dzdt[2] = g[1] + (L*(kd*dphiy + kp*phiy) - L*cy*sy *
                      dphix*dphix - uy*sx*sy)/cy  # ddx0
    dzdt[3] = dy0  # dy0
    dzdt[4] = g[2] - (L*cy*(kd*dphix + kp*phix) + 2.0*L*sy *
                      dphix*dphiy-9.81*sx*sy*sy)/cx  # ddy0
    dzdt[5] = z[6]  # dphix
    dzdt[6] = - kd*dphix - kp*phix - 9.81/L*cy*sx + gy*cx/(cy*L)  # ddphix
    dzdt[7] = z[8]  # dphiy
    dzdt[8] = - kd*dphiy - kp*phiy - 9.81/L * \
        cx*sy - (gx*cy + gy*sx*sy)/L  # ddphiy

    return dzdt, z


def discrete_dynamics(zk, gk, Ts, k, L):
    '''Repeated Euler's method (Algorithm 1 in ECC)'''
    M = 10
    delta = Ts/M
    zk1 = zk
    for ct in range(M):
        zk1 = zk1 + delta * continuous_dynamics(zk1, gk, k, L)[0]
    return zk1

import numpy as np


def control(z, zref, t, gamma, k, L, last_g, last_gopt):

    # % x=[x, dx, y, dy, phix, dphix, phiy, dphiy] eq 55 in ECC

    x0, dx0, y0, dy0, phix, dphix, phiy, dphiy = z  # Eq. 55 in ECC

    Ts = 0.2  # Time step
    N = 4  # Prediction step (in time N*Ts)
    samptime = 0.6  # Do a new prediction every samptime

    Q = np.diag([1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Eq. 64 in Ecc
    min_g = -gamma*np.ones((N, 2))  # contraint output Eq 59
    max_g = gamma*np.ones((N, 2))  # constraint output Eq 59
    gopt = np.zeros((N, 2))  # init optimal output

    mod = t % samptime
    if mod == 0:
        #     gopt = MPCred(z,zref,Ts,N,last_gopt,Q,min_g,max_g,k,last_g,L); %NMPC
        n = 1
    else:
        gopt = last_gopt
        n = round(mod)/(Ts+1)

    g = gopt[-1] # current NMPC output, eq 56 in ECC

    # Lyapunov damping controller
    kp, kd = k

    cx = np.cos(phix)
    sx = np.sin(phix)
    cy = np.cos(phiy)
    sy = np.sin(phiy)

    uy = - L*cy/cx*(kd*dphix + kp*phix) - 2*L*sy/cx + 9.81*sx*sy**2/cx
    ux = L/cy*(kd*dphiy + kp*phiy) - L*sy*dphix**2 - uy*sx*sy/cy

    # Final controller
    dw = np.array([ux + g[0], uy + g[1]])  # Eq 65-66 in ECC

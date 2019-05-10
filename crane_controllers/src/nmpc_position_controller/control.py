import numpy as np
np.set_printoptions(precision=4, suppress=True)
from nmpc_position_controller.dynamics import discrete_dynamics


def cost(g, z, Ts, N, zref, Q, k, last_g, L):

    R = np.diag([0.1, 0.1])
    zk = z
    gk = g[0].reshape(-1, 1)
    J = 0.0
    zref = zref.reshape(-1, 1)
    print(zref.T)

    last_g = last_g.reshape(-1, 1)

    for i in range(N+1):
        zk1 = discrete_dynamics(zk, gk, Ts, k, L)
        print((zk1-zref).T)
        J += np.dot(np.dot((zk1-zref).T, Q), zk1-zref)[0,0]  # Eq 61 in Ecc
        # print(J)
        # if i == 0:
        #     J += np.dot(np.dot((gk-last_g).T, R), gk-last_g)[0,0]
        # else:
        #     pass
            # J += np.dot(np.dot((gk-last_g).T, R), gk-last_g)


# for ct=1:N
#     zk1 = DicreteDynamic(zk, gk, Ts,k,L);
#     J = J + (zk1-zref)'*Q*(zk1-zref); % Eq 61 in Ecc
#     if ct==1
#         J = J + (gk-last_g')*R*(gk-last_g')';
#     else
#         J = J + (gk-g(ct-1,:))*R*(gk-g(ct-1,:))';
#     end
#     zk = zk1;
#     if ct<N
#         gk = g(ct+1,:);
#     end
# end


def control(z, zref, t, gamma, k, L, last_g, last_gopt):

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

    g = gopt[-1]  # current NMPC output, eq 56 in ECC

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

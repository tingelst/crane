import numpy as np
from numpy.linalg import multi_dot

# discrete xkp1=fk(xk,uk,w,L)
def fk(x,u,w,L):
    fk_out=np.array([x[2],
                     x[3],
                     (2*x[2]*x[3]*np.sin(x[1]) - w*np.sin(x[0]) + (u[1]*np.cos(x[0]))/L)/np.cos(x[1]),
                     - np.cos(x[1])*np.sin(x[1])*np.square(x[2]) - (u[0]*np.cos(x[1]) + u[1]*np.sin(x[0])*np.sin(x[1]))/L - w*np.cos(x[0])*np.sin(x[1]),
                     0,
                     0])
    return fk_out


def Fk(x,u,w,L,dt):
    '''Transition matrix'''
    Fk_out = np.array([[                                                   1,                                                                                                                                                                0,                                    dt,                            0,0,0],
                       [                                                   0,                                                                                                                                                               1,                                       0,                           dt,0,0],
                       [       -(dt*(w*np.cos(x[0]) + (u[1]*np.sin(x[0]))/L))/np.cos(x[1]),              2*dt*x[2]*x[3] + (dt*np.sin(x[1])*(2*x[2]*x[3]*np.sin(x[1]) - w*np.sin(x[0]) + (u[1]*np.cos(x[0]))/L))/np.square(np.cos(x[1]))                             ,      (2*dt*x[3]*np.sin(x[1]))/np.cos(x[1]) + 1, (2*dt*x[2]*np.sin(x[1]))/np.cos(x[1]),0,0],
                       [ dt*(w*np.sin(x[0])*np.sin(x[1]) - (u[1]*np.cos(x[0])*np.sin(x[1]))/L), -dt*(np.square(x[2])*np.square(np.cos(x[1])) - np.square(x[2])*np.square(np.sin(x[1])) - (u[0]*np.sin(x[1]) - u[1]*np.cos(x[1])*np.sin(x[0]))/L + w*np.cos(x[0])*np.cos(x[1])),           -2*dt*x[2]*np.cos(x[1])*np.sin(x[1]),                            1,0,0],
                       [0,0,0,0,1,0],
                       [0,0,0,0,0,1]])
    return Fk_out


def ekf(Lvec, uk, hat_Pkm1, hat_thetakm1, theta, r, dt):
    '''Extended Kalman filter'''
    D = 10  # number of times to do repeated Euler's method
    g = 9.81  # gravity
    L = r  # lenght of pendulum
    u = uk  # acceleration of the crane tip
    x = hat_thetakm1  # estimated pendulum oscillation angles and rates, and bias of pendulum oscillation angles
    # Covariance matrix for measurement noise
    R = np.array([[0.00377597, -0.00210312],
                  [-0.00210312, 0.00125147]])
    # Covariance matrix for process noise
    Q = np.diag([0.00003, 0.00003, 0.0005, 0.0005, 0.0001, 0.0001])
    # Q = np.diag([0.00003, 0.00003, 0.0005, 0.0005, 0.0, 0.0])
    # Observation matrix
    H = np.array([[1, 0, 0, 0, 1, 0],
                  [0, 1, 0, 0, 0, 1]])
    Fi = Fk(x, u, g/r, L, dt)

    # Measurement of payload oscillation angles
    zkp1 = np.array([np.arctan2(-Lvec[1], Lvec[2]),
                     np.arctan2(Lvec[0], np.sqrt(np.square(Lvec[1])+np.square(Lvec[2])))])
    # Repeated Euler's method
    for i in range(D-1):
        x = fk(x, u, g/r, L)*dt/D+x
    barP_kp1 = multi_dot([Fi, hat_Pkm1, Fi.T])+Q
    K_kp1 = multi_dot(
        [barP_kp1, H.T, np.linalg.inv(R+multi_dot([H, barP_kp1, H.T]))])
    hat_thetak = x+np.dot(K_kp1, zkp1-np.dot(H, x))
    hat_Pk = np.dot((np.diag([1, 1, 1, 1, 1, 1])-np.dot(K_kp1, H)), barP_kp1)

    return hat_thetak, hat_Pk, zkp1

#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plot

from crane_pendulum_dynamics.rk4 import rk4 

def simple_pendulum_dyn(t, q):
    '''
    Model representing the dynamical system of a nonlinear pendulum
    :param t: current time
    :param q: current state
    :return: current value for dqdt
    '''
    # define some useful constants
    g = 9.81  # gravity
    l = 5.0  # length of pendulum arm
    m = 2.0  # mass of pendulum weight
    c = 1.0  # dampening factor

    # extract the two states
    x1 = q[0] # theta
    x2 = q[1] # dtheta

    # return a numpy array
    # return np.array([x2, -((g/l)*np.sin(x1) + (c/m)*x2)])
    return np.array([x2, -((g/l)*np.sin(x1))])

def spherical_pendulum_dyn(t, q):
    '''
    Model representing the dynamical system of a nonlinear pendulum
    :param t: current time
    :param q: current state
    :return: current value for dqdt
    '''
    # define some useful constants
    g = 9.81  # gravity
    l = 5.0  # length of pendulum arm
    m = 2.0  # mass of pendulum weight
    c = 1.0  # dampening factor

    # extract the two states
    x1 = q[0] # theta
    x2 = q[1] # dtheta

    # return a numpy array
    # return np.array([x2, -((g/l)*np.sin(x1) + (c/m)*x2)])
    return np.array([x2, -((g/l)*np.sin(x1))])

def spherical_pendulum_dyn(t, q):

    g = 9.81
    L = 1.0

    phix, dphix, phiy, dphiy = q
    cx = np.cos(phix)
    sx = np.sin(phix)
    cy = np.cos(phiy)
    sy = np.sin(phiy)

    ddx0 = 0.0
    ddy0 = 0.0

    ddphix = (-g*sx/L + 2.0*dphix*dphiy*sy)/cy + ddy0*cx/(L*cy)
    ddphiy = -g*cx*sy/L - (ddx0*cy + ddy0*sx*sy)/L

    return np.array([dphix, ddphix, dphiy, ddphiy])


def main_test():
    '''
    Perform a sample simulation and visualization of the pendulum stuff
    '''

    # initialize simulation parameters
    t = 0.0
    dt = 1e-2
    tf = 20.0
    q = np.array([np.pi/6.0, 0, np.pi/4.0, 0])

    # initialize output container to store simulation data
    Nt = int(np.round((tf - t)/dt)) + 1
    solution = np.zeros(shape=(len(q)+1, Nt))
    solution[0, 0] = t
    solution[1:, 0] = q[:]

    # perform main simulation loop
    k = 1
    while t <= tf:

        # update the time and state
        (t, q) = rk4(t, q, dt, spherical_pendulum_dyn)

        # store the solution
        solution[0, k] = t
        solution[1:, k] = q[:]

        # update the index
        k = k + 1


    fig = plot.figure()
    f1 = plot.subplot(211)
    plot.plot(solution[0, :], solution[1, :] *
              (180.0/np.pi), color=(0.6, 0, 1.0))
    ax = plot.gca()
    ax.grid()
    ax.set_ylabel("theta (deg)")

    f2 = plot.subplot(212)
    plot.plot(solution[0, :], solution[2, :] *
              (180.0/np.pi), color=(0.0, 0.6, 1.0))
    ax = plot.gca()
    ax.grid()
    ax.set_ylabel("dtheta (deg/s)")
    ax.set_xlabel("t [s]")

    plot.show()
    fig.savefig("pendulum_history.jpg", dpi=300)


if __name__ == "__main__":
    main_test()

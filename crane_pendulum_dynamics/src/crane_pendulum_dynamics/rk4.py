def rk4(t, q, dt, f, u):
    '''
    Define a 4th order Runge-Kutta integrator
    :param t: time
    :param q: current state vector
    :param dt: time step to use
    :param f: dynamical system model
    :return: a tuple made up of the new time and new state vector
    '''
    k1 = dt*f(t, q, u)
    k2 = dt*f(t + 0.5*dt, q + 0.5*k1, u)
    k3 = dt*f(t + 0.5*dt, q + 0.5*k2, u)
    k4 = dt*f(t + dt, q + k3, u)
    return (t + dt, q + (k1 + 2.0*(k2 + k3) + k4)/6.0)

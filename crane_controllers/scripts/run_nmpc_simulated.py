#!/usr/bin/env python

import numpy as np
import pickle
import matplotlib.pyplot as plt

from nmpc_position_controller.dynamics import discrete_dynamics, continuous_dynamics
from nmpc_position_controller.control import control, cost


def load_data():
    f = open(
        '/home/lars/crane_ws/src/crane/crane_controllers/data/datain.pickle', 'rb')
    datain = pickle.load(f)
    f.close()
    f = open(
        '/home/lars/crane_ws/src/crane/crane_controllers/data/dataout.pickle', 'rb')
    dataout = pickle.load(f)
    f.close()
    return datain, dataout


if __name__ == "__main__":
    datain, dataout = load_data()

    last_gopts = datain['last_gopt']
    last_gs = datain['last_g']
    Ls = datain['L']
    ks = datain['k']
    gammas = datain['gamma']
    ts = datain['t']
    zrefs = datain['zref']
    zs = datain['z']

    dot_wxs = dataout['dot_wx']
    dot_wys = dataout['dot_wy']
    gxs = dataout['gx']
    gys = dataout['gy']

    for i in range(len(ts)):
        z = zs[i]
        zref = zrefs[i]
        t = ts[i]
        gamma = gammas[i]
        k = ks[i]
        L = Ls[i]
        last_g = last_gs[i]
        last_gopt = last_gopts[:,:,i]

        dot_wx = dot_wxs[i]
        dot_wy = dot_wys[i]
        gx = gxs[i]
        gy = gys[i]

        Q = np.diag([1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0])

        if i == 999 :
            # print(z)
            # print(last_g)
            # print(k)
            # print(L)
            
            c = cost(last_gopt, z, 0.2, 4, zref, Q, k, last_g, L)
            print(c)

            # 
            # 
            # dd = discrete_dynamics(z.reshape(-1,1), last_g, 0.2, k, L)
            # cd = continuous_dynamics(z, last_g, k, L)
            # print(dd)
            # print(dd)







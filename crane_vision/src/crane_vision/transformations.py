import numpy as np


def rotx(theta):
    '''Rotation matrix about x-axis'''
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(theta), -np.sin(theta)],
                    [0, np.sin(theta), np.cos(theta)]])
    return R_x


def roty(theta):
    '''Rotation matrix about y-axis'''
    R_y = np.array([[np.cos(theta), 0, np.sin(theta)],
                    [0, 1, 0],
                    [-np.sin(theta), 0, np.cos(theta)]])
    return R_y


def rotz(theta):
    '''Rotation matrix about z-axis'''
    R_z = np.array([[np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta), np.cos(theta), 0],
                    [0, 0, 1]])
    return R_z


def TransMat(R, t):
    '''Homogeneous transformation matrix'''
    T = np.array([[R[0][0],   R[0][1],    R[0][2],   t[0]],
                  [R[1][0],   R[1][1],    R[1][2],   t[1]],
                  [R[2][0],   R[2][1],    R[2][2],   t[2]],
                  [0.0,   0.0,    0.0,   1.0]])
    return T

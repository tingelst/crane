import numpy as np
import cv2


def find_sphere_centers(img, c1, c2):
    # HSV colourspace parameters
    v1_min = 43
    v2_min = 54
    v3_min = 86
    v1_max = 73
    v2_max = 250
    v3_max = 255
    blur_param = 5

    # Gaussian blurred image
    # blur1 = cv2.GaussianBlur(img, (3+(2*blur_param-2), 3+(2*blur_param-2)), 0)
    blur1 = img
    # # Convert from RGB to HSV
    hsv1 = cv2.cvtColor(blur1, cv2.COLOR_BGR2HSV)
    # # Binary image based on colours
    mask1 = cv2.inRange(hsv1, (v1_min, v2_min, v3_min),
                        (v1_max, v2_max, v3_max))
    # Find objects in the image
    _, cnts1, _ = cv2.findContours(
        mask1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(cnts1) > 1:
        # Sort objects based on the contour area and extract the two biggest
        cnts1 = sorted(cnts1, key=cv2.contourArea, reverse=True)[:2]
        # The two objects corresponding to the spheres
        c01 = cnts1[0]
        c02 = cnts1[1]
        # Calculate the moments of the spheres
        M01 = cv2.moments(c01)
        M02 = cv2.moments(c02)
        # Calculate the centroids of the spheres
        if M01["m00"] < 0.000001:
            center01 = c1
        else:
            center01 = (float(M01["m10"] / M01["m00"]),
                        float(M01["m01"] / M01["m00"]))
        if M02["m00"] < 0.000001:
            center02 = c2
        else:
            center02 = (float(M02["m10"] / M02["m00"]),
                        float(M02["m01"] / M02["m00"]))
    else:
        center01 = c1
        center02 = c2

    if center01[1] > center02[1]:
        tmp = center01
        center01 = center02
        center02 = tmp

    return center01, center02, mask1


def dlt(c0, c1, c2, P0, P1, P2):
    '''Direct linear triangulation'''
    # P0, P1, P2 are camera matrices
    uL, sL, vL = np.linalg.svd(np.array([np.dot(c0[0], P0[2, :])-P0[0, :],
                                         np.dot(c0[1], P0[2, :])-P0[1, :],
                                         np.dot(c1[0], P1[2, :])-P1[0, :],
                                         np.dot(c1[1], P1[2, :])-P1[1, :],
                                         np.dot(c2[0], P2[2, :])-P2[0, :],
                                         np.dot(c2[1], P2[2, :])-P2[1, :]]))
    vL = vL.T.conj()
    X = vL[:, 3]
    if np.absolute(vL[3][3]) > 0.000000000000000001:
        X = np.divide(vL[:, 3], vL[3, 3])
    else:
        pass
    return X


def find_line(center01, center02, center11, center12, center21, center22, P0, P1, P2):
    ''' Find  direction vector of the line through the spheres, given in camera coordinates'''
    X1 = dlt(center01, center11, center21, P0, P1, P2)
    X2 = dlt(center02, center12, center22, P0, P1, P2)
    Lc0 = X2[0:3]-X1[0:3]
    return Lc0

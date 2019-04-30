import math
import select
import struct
from socket import socket, gethostbyname, AF_INET, SOCK_DGRAM, SOL_SOCKET, SO_REUSEADDR
from collections import deque
import numpy as np
from operator import xor
import cv2
import imutils
import time
import scipy.io as sio
from scipy import linalg
import math
from math import sin, cos
import matplotlib.pyplot as plt
import sympy
import scipy.io
from matplotlib import style
from IPython.display import display, Latex
from scipy.io import loadmat
import matplotlib.pyplot as plt
import matplotlib.animation as animation
sympy.init_printing(use_latex=True)
import threading 
import queue
from numpy.linalg import multi_dot
current_time = lambda: time.time() 

# Rotation matrix about x-axis
def rotx(theta):
     
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta), -math.sin(theta) ],
                    [0,         math.sin(theta), math.cos(theta)  ]
                    ])
    
    return R_x    

# Rotation matrix about y-axis        
def roty(theta):
    R_y = np.array([[math.cos(theta),    0,      math.sin(theta)  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta),   0,      math.cos(theta)  ]
                    ])
    
    return R_y

# Rotation matrix about z-axis   
def rotz(theta) :          
       
    R_z = np.array([[math.cos(theta),    -math.sin(theta),    0],
                    [math.sin(theta),    math.cos(theta),     0],
                    [0,                     0,                      1]
                    ])
    
    return R_z
                 
# Homogeneous transformation matrix
def TransMat(R , t) :
    T = np.array([[R[0][0],   R[0][1],    R[0][2],   t[0]],
                  [R[1][0],   R[1][1],    R[1][2],   t[1]],
                  [R[2][0],   R[2][1],    R[2][2],   t[2]],
                  [0.0,   0.0,    0.0,   1.0 ]]) 
    return T
                    


def draw(img, corners, imgpts):
     corner = tuple(corners[0].ravel())
     img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
     img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
     img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
     return img
 

# discrete xkp1=fk(xk,uk,w,L)
def fk(x,u,w,L):
    fk_out=np.array([x[2],
                     x[3],
                     (2*x[2]*x[3]*sin(x[1]) - w*sin(x[0]) + (u[1]*cos(x[0]))/L)/cos(x[1]),
                     - cos(x[1])*sin(x[1])*np.square(x[2]) - (u[0]*cos(x[1]) + u[1]*sin(x[0])*sin(x[1]))/L - w*cos(x[0])*sin(x[1]),
                     0,
                     0])
    return fk_out


# Transition matrix
def Fk(x,u,w,L,dt):
    Fk_out = np.array([[                                                   1,                                                                                                                                                                0,                                    dt,                            0,0,0],
                       [                                                   0,                                                                                                                                                               1,                                       0,                           dt,0,0],
                       [       -(dt*(w*cos(x[0]) + (u[1]*sin(x[0]))/L))/cos(x[1]),              2*dt*x[2]*x[3] + (dt*sin(x[1])*(2*x[2]*x[3]*sin(x[1]) - w*sin(x[0]) + (u[1]*cos(x[0]))/L))/np.square(cos(x[1]))                             ,      (2*dt*x[3]*sin(x[1]))/cos(x[1]) + 1, (2*dt*x[2]*sin(x[1]))/cos(x[1]),0,0],
                       [ dt*(w*sin(x[0])*sin(x[1]) - (u[1]*cos(x[0])*sin(x[1]))/L), -dt*(np.square(x[2])*np.square(cos(x[1])) - np.square(x[2])*np.square(sin(x[1])) - (u[0]*sin(x[1]) - u[1]*cos(x[1])*sin(x[0]))/L + w*cos(x[0])*cos(x[1])),           -2*dt*x[2]*cos(x[1])*sin(x[1]),                            1,0,0],
                       [0,0,0,0,1,0],
                       [0,0,0,0,0,1]])
    return Fk_out

# Extended Kalman Filter
def EKF(Lvec,uk,hat_Pkm1,hat_thetakm1,theta,r,dt): 
    D = 10 # number of times to do repeated Euler's method
    g = 9.81 # gravity
    L = r # lenght of pendulum
    u = uk # acceleration of the crane tip
    x = hat_thetakm1 # estimated pendulum oscillation angles and rates, and bias of pendulum oscillation angles
    R = np.array([[ 0.00377597,-0.00210312],[-0.00210312,0.00125147]]) # Covariance matrix for measurement noise
    Q = np.diag([0.00003,0.00003,0.0005,0.0005,0.0001,0.0001]) # Covariance matrix for process noise
    H = np.array([[1,0,0,0,1,0],[0,1,0,0,0,1]]) # Observation matrix
    Fi = Fk(x,u,g/r,L,dt)
    
    zkp1 = np.array([math.atan2(-Lvec[1],Lvec[2]),math.atan2(Lvec[0],math.sqrt(np.square(Lvec[1])+np.square(Lvec[2])))]) # Measurement of payload oscillation angles
    # Repeated Euler's method
    for i in range(D-1):
        x=fk(x,u,g/r,L)*dt/D+x
    barP_kp1=multi_dot([Fi,hat_Pkm1,Fi.T])+Q
    K_kp1=multi_dot([barP_kp1,H.T,np.linalg.inv(R+multi_dot([H,barP_kp1,H.T]))])
    hat_thetak=x+np.dot(K_kp1,zkp1-np.dot(H,x))
    hat_Pk=np.dot((np.diag([1,1,1,1,1,1])-np.dot(K_kp1,H)),barP_kp1)
    
    return hat_thetak, hat_Pk, zkp1  

        
# Direct linear triangulation
def DLT(c0,c1,c2):
    uL, sL, vL = linalg.svd(np.array([np.dot(c0[0],P0[2,:])-P0[0,:],np.dot(c0[1],P0[2,:])-P0[1,:],np.dot(c1[0],P1[2,:])-P1[0,:],np.dot(c1[1],P1[2,:])-P1[1,:],np.dot(c2[0],P2[2,:])-P2[0,:],np.dot(c2[1],P2[2,:])-P2[1,:]]))
    vL=vL.T.conj()
    X=vL[:,3]
    if np.absolute(vL[3][3]) > 0.000000000000000001:
        X = np.divide(vL[:,3],vL[3,3])
    else:
        pass
    return X

# Find  direction vector of the line through the spheres, given in camera coordinates
def FindLine(center01,center02,center11,center12,center21,center22):
    X1 = DLT(center01,center11,center21)
    X2 = DLT(center02,center12,center22) 
    Lc0=X2[0:3]-X1[0:3]
    return Lc0

# Class of parameters that are transfered between MATLAB and Python ov UDP
class ParamSendRecieve(object):
    def __init__(self):
        self.Lhat=0.5 # length of pendulum
        self.q1=0.37 # slew joint
        self.ddx=0.0 # acceleration of crane tip in x-direction
        self.ddy=0.0 # acceleration of crane tip in y-direction
        self.senddata=np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])  # acceleration of crane tip in y-direction


# Class for storing data
class StoreData(object):
    def __init__(self):
        self.hat_phix = list()
        self.hat_phiy = list() 
        self.hat_dphix = list() 
        self.hat_dphiy = list()
        self._dphix = list() 
        self._dphiy = list() 
        self.xList = list() 
        self.yList=list() 
        self.yList2=list()
        self.List_phix = 0 
        self.List_phiy = 0 
        self.tid = list() 
        self.delta_tid = list()
        self.bias_x = list()
        self.bias_y = list()
        self.delta_tid = list()
        
    def update(self,z,Phi,start,tk):
        self.tid.append(current_time()-start)
        self.delta_tid.append(current_time()-tk)
        self.yList.append(math.degrees(z[0]))
        self.yList2.append(math.degrees(z[1]))
        self.hat_phix.append(math.degrees(Phi[0]))
        self.hat_phiy.append(math.degrees(Phi[1]))
        self.hat_dphix.append(math.degrees(Phi[2]))
        self.hat_dphiy.append(math.degrees(Phi[3]))
        self.bias_x.append(math.degrees(Phi[4]))
        self.bias_y.append(math.degrees(Phi[5]))


# Downscale resolution of the cameras
def cam1_set720p():
    vs1.set(3, 1280)
    vs1.set(4, 720)
    
def cam2_set720p():
    vs2.set(3, 1280)
    vs2.set(4, 720)
    
def cam3_set720p():
    vs3.set(3, 1280)
    vs3.set(4, 720) 
    
# Find pixel coordinates that corresponds to the spheres   
def FindCenter(frame1,c1,c2):
    # HSV colourspace parameters
    v1_min = 43
    v2_min = 54
    v3_min = 86
    v1_max = 73
    v2_max = 250
    v3_max = 255
    blur_param = 5
    
    #Gaussian blurred image
    blur1 = cv2.GaussianBlur(frame1, (3+(2*blur_param-2), 3+(2*blur_param-2)), 0) 
    #Convert from RGB to HSV 
    hsv1 = cv2.cvtColor(blur1, cv2.COLOR_BGR2HSV)
    #Binary image based on colours
    mask1 = cv2.inRange(hsv1, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))
    #Find objects in the image
    cnts1 = cv2.findContours(mask1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #Cancel if none objects
    cnts1 = cnts1[0] if imutils.is_cv2() else cnts1[1]
    if len(cnts1) > 1:
        # Sort objects
        cnts1 = sorted(cnts1, key=cv2.contourArea, reverse=True)[:5]
        #The two objects corresponding to the spheres
        c01 = cnts1[0] 
        c02 = cnts1[1]
        #Calculate the moments of the spheres
        M01 = cv2.moments(c01)
        M02 = cv2.moments(c02)
        #Calculate the centorids of the spheres
        if M01["m00"] < 0.000001:
            center01 = c1
        else:
            center01 = (float(M01["m10"] / M01["m00"]), float(M01["m01"] / M01["m00"]))
        if M02["m00"] < 0.000001:
            center02 = c2
        else:
            center02 = (float(M02["m10"] / M02["m00"]), float(M02["m01"] / M02["m00"]))
            
    else:
        center01 = c1
        center02 = c2
        
    if center01[1] > center02[1]:
        tmp = center01
        center01 = center02
        center02 = tmp
    else:
        pass
            
    return center01, center02 # returns the two pixels that corresponds to the two spheres


Obj=ParamSendRecieve()

# object for sending messages to MATLAB
def fnToMATLAB(msgToMatlab, *args):
    cs = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    cs.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    ti =current_time()
    while stop < 1:
        cs.sendto(Obj.senddata, ('127.0.0.1',5001))
        if 0.08-current_time()+ti< 0:
            ti = current_time()
        else:
            time.sleep(0.08-current_time()+ti)
            ti = current_time()
    cs.close()


# object for reading messages from MATLAB
def fnFromMATLAB(msgfromMatlab,*args):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
    s.bind(('',5002))
    while stop < 1:
        data, addr = s.recvfrom(32)
        Obj.Lhat=np.array([struct.unpack('d', data[0:8])[0]])
        Obj.q1=np.array([struct.unpack('d', data[8:16])[0]])
        Obj.ddx=np.array([struct.unpack('d', data[16:24])[0]])
        Obj.ddy=np.array([struct.unpack('d', data[24:32])[0]])
        time.sleep(0.002)
    s.close()


# Declare messages for sending and recieving over UDP
msgfromMatlab = queue.LifoQueue()
msgfromMatlab.put(0.0)  
msgToMatlab = queue.LifoQueue()
msgToMatlab.put(np.array([0.0,0.0,0.0,0.0,0.0,0.0]))
stop =0.0

# Constructors for calling the objects for sending and recieving messages over UDP
threadsend=threading.Thread(target=fnToMATLAB, args=(msgToMatlab, 1))
threadread=threading.Thread(target=fnFromMATLAB, args=(msgfromMatlab,1))
threadsend.start()
threadread.start()





# Declare camera objects
vs1 = cv2.VideoCapture(0)
vs2 = cv2.VideoCapture(2)
vs3 = cv2.VideoCapture(1)

cam1_set720p()
cam2_set720p()
cam3_set720p()    
 

# Camera calibration matrices   
K0=np.array([[ 937,0 , 637.21],
            [0  ,937, 381.54],
             [0  , 0,   1.0]])

K1=np.array([[ 941,0 , 637.21],
             [0  ,941, 349.9],
             [0  , 0,   1.0]])
    
K2=np.array([[ 942,0 , 623.66],
             [0  ,942, 345.69],
             [0  , 0,   1.0]])
    
    
# Translation vectors between cameras
t_21=-np.array([ 233.8, 0, 0])
t_31 = -np.array([ 467, 0,0])
PII=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0]])

# Camera matrices
P0=np.dot(np.dot(K0,PII),TransMat(np.eye(3),np.array([0,0,0])))
P1=np.dot(np.dot(K1,PII),TransMat(np.eye(3),t_21))
P2=np.dot(np.dot(K2,PII),TransMat(np.eye(3),t_31))

cv2.namedWindow("Camera 1",cv2.WINDOW_NORMAL)
cv2.resizeWindow("Camera 1", 768, 432)


#init pixels and states for EKF
center01 = (0, 0)
center02 = (0, 0)
center11 = (0, 0)
center12 = (0, 0)
center21 = (0, 0)
center22 = (0, 0)
hat_Pkm1 = np.diag([0,0,0,0,0,0])
hat_thetakm1 = np.array([0,0,0,0,2.5*math.pi/180,-1.7*math.pi/180])

#Declare store data object
StoredData=StoreData()

# sleep for 2 seconds (start-up cameras)
time.sleep(2.0)

start = current_time() 
tk = start
while 1:
    # Read images from the cameras
    frame1 = vs1.read()
    frame2 = vs2.read()
    frame3 = vs3.read()
    frame1 = frame1[1] 
    frame2 = frame2[1]
    frame3 = frame3[1]
    
    if frame1 is None:
        break
    
    if frame2 is None:
        break
   
    if frame3 is None:
        break
    
    # Find pixels that corresponds to the spheres
    center01, center02 = FindCenter(frame1,center01,center02)
    center11, center12 = FindCenter(frame2,center11,center12)
    center21, center22 = FindCenter(frame3,center21,center22)
    
    # find direction vector of a line through the spheres, given in camera coordinates
    Lc0 = FindLine(center01,center02,center11,center12,center21,center22)
    # find direction vector of a line through the spheres, given in inertial coordinates
    Lvec=np.dot(np.array([[ -cos(Obj.q1), 0, sin(Obj.q1)],[sin(Obj.q1), 0, cos(Obj.q1)],[0, 1,       0]]),Lc0)
    
    if linalg.norm(Lvec) > 0.00001:
        Lvec = Lvec/linalg.norm(Lvec)

    # Extended Kalman Filter
    hat_thetak, hat_Pk, zk = EKF(Lvec,np.array([Obj.ddx,Obj.ddy]),hat_Pkm1,hat_thetakm1,Obj.q1, Obj.Lhat,current_time()-tk)
    # Collect estimated payload oscillation angles and rates, bias of payload oscillation angle, and measurement of payload oscillation angle
    Obj.senddata = np.array([hat_thetak[0],hat_thetak[1],hat_thetak[2],hat_thetak[3],hat_thetak[4],hat_thetak[5],zk[0],zk[1]])
    msgToMatlab.put(hat_thetak)
    Obj.senddata
    hat_thetakm1 = hat_thetak
    hat_Pkm1 = hat_Pk 
    StoredData.update(zk,hat_thetak,start,tk)
    tk = current_time()
      
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        
        break
    
    
    


stop=1
vs1.release()
vs2.release()
vs3.release()    
cv2.destroyAllWindows()

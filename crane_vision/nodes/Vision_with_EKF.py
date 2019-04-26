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

                    


def draw(img, corners, imgpts):
     corner = tuple(corners[0].ravel())
     img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
     img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
     img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
     return img
 


        

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





 

    
    

cv2.namedWindow("Camera 1",cv2.WINDOW_NORMAL)
cv2.resizeWindow("Camera 1", 768, 432)



#Declare store data object
StoredData=StoreData()

# sleep for 2 seconds (start-up cameras)
time.sleep(2.0)

start = current_time() 
tk = start
while 1:
    
    
    
    


stop=1
cv2.destroyAllWindows()

#!/usr/bin/env python

import numpy as np
from math import sin, cos, atan2, pi
from numpy import sqrt, square, linalg, ones, zeros
from matplotlib import pyplot
import time
current_time = lambda: time.time() 

import rospy
from crane_msgs.msg import CraneState, CraneTrajectoryPoint


class TrajectoryObject(object):
    def __init__(self, xi, N, Tstart, amax, vmax, deltaTheta):
        self.tl = zeros([2,N-1])
        self.N = N
        self.vl = zeros(N-1)
        self.deltaT = np.ones(N)*vmax/amax
        self.T = ones(N+1)*Tstart  
        r = sqrt(square(xi[0])+square(xi[1]))
        theta = atan2(xi[1],xi[0])
        self.r =r
        self.theta =theta
        points = zeros([2,N])
        for x in range(0, N):
            points[:,x] = np.array([r*cos(theta + x*deltaTheta/(N-1)),r*sin(theta+x*deltaTheta/(N-1))])
        self.s = linalg.norm(points[:,1]-points[:,0]) 
        self.points =points
        for x in range(0, N):
            if x < N-1:
                self.tl[:,x] = (points[:,x+1]-points[:,x])/self.s
                self.vl[x] = self.s*vmax/(max(abs(points[:,x+1]-points[:,x])))
            if x > 0 and x < N-1:
                self.deltaT[x] = linalg.norm(self.vl[x]*self.tl[:,x]-self.vl[x-1]*self.tl[:,x-1])/amax
                self.T[x] = self.s/self.vl[x-1]+self.T[x-1]+(self.deltaT[x-1]-self.deltaT[x])/2
            if x == N-1:
                self.T[x] = self.s/self.vl[x-1]+self.T[x-1]+(self.deltaT[x-1]-self.deltaT[x])/2
        self.T[N]=self.T[N-1]+self.deltaT[N-1]
        self.st = zeros([2,N-1])
        self.dst = zeros([2,N-1])
        self.ddst = zeros([2,N-1])
        self.xr = xi
        self.vr = np.array([0.0,0.0])  
        j = 0.0
        for x in range(1,N):
            if self.T[x-1] > self.T[x-1]+self.deltaT[x-1]:
                j = 1.0
            elif self.T[x-1]+ self.deltaT[x-1] > self.T[x]:
                j = 1.0
            elif self.T[x] > self.T[x] +self.deltaT[x]:
                j = 1.0
            
        if j > 0.0:
            print('Lower the velocity')
        else:
            print('Ready')
        
    def update(self,t):
        self.xr = self.points[:,0]
        self.vr = np.array([0.0,0.0])
        for i in range(1,self.N): 
            if t <= self.T[i-1]:
                sj = 0.0
                dsj = 0.0
                ddsj = 0.0
            elif self.T[i-1] < t and t <= self.T[i-1] + self.deltaT[i-1]:
                sj=self.vl[i-1]*square(t-self.T[i-1])/(2*self.deltaT[i-1])
                dsj=self.vl[i-1]*(t-self.T[i-1])/self.deltaT[i-1]
                ddsj=self.vl[i-1]/self.deltaT[i-1]
            elif self.T[i-1]+self.deltaT[i-1] < t and t <= self.T[i]:
                sj=self.vl[i-1]*(t-self.deltaT[i-1]/2-self.T[i-1])
                dsj=self.vl[i-1]
                ddsj=0.0
            elif self.T[i] < t and t <= self.T[i] +self.deltaT[i]:
                sj = self.vl[i-1]*(t-self.deltaT[i-1]/2-self.T[i-1]-square(t-self.T[i])/(2*self.deltaT[i]))
                dsj = self.vl[i-1]*(1-(t-self.T[i])/self.deltaT[i])
                ddsj = -self.vl[i-1]/self.deltaT[i]
            else:
                sj = self.s
                dsj = 0.0
                ddsj = 0.0
            
            self.st[:,i-1] = self.tl[:,i-1]*sj
            self.dst[:,i-1]  =self.tl[:,i-1]*dsj
            self.ddst[:,i-1]  = self.tl[:,i-1]*ddsj
            self.xr = self.xr + self.st[:,i-1]
            self.vr = self.vr + self.dst[:,i-1]
            
        return self.xr, self.vr


class TrajectoryPointPublisher(object):
    def __init__(self, rate):
        self._rate = rospy.Rate(rate)
        self._pub = rospy.Publisher( '/crane_trajectory', CraneTrajectoryPoint, queue_size=5)

        self._sub = rospy.Subscriber('/lyapunov_pendulum_damping_controller/state', CraneState, self._callback)
        self._traj = None

        self._state = None

    def _callback(self, msg):
        if self._traj is None:
            xi = np.array([msg.x, msg.y]) # initial crane tip
            deltaTheta = 45*pi/180 # final slew angle = initial slew angle + deltaTheta
            amax=0.2 # max acceleration in x- and y-direction
            vmax=1.5*amax # max velocity in x- and y-direction
            TStart=0.0 # Start time of trajectory
            N = 3 # number of points along circle
            self._traj = TrajectoryObject( xi, N, TStart, amax, vmax, deltaTheta)

    def run(self):
        t0 = current_time()
        while not rospy.is_shutdown():
            if self._traj is not None:
                t = current_time()
                x,v = self._traj.update(t - t0)

                msg = CraneTrajectoryPoint()
                msg.position = x.copy()
                # msg.velocity = v.copy()
                msg.velocity = np.array([0.0, 0.0])

                self._pub.publish(msg)

            self._rate.sleep()



def main():
    rospy.init_node('crane_trajectory_publisher')

    ctp = TrajectoryPointPublisher(10)
    ctp.run()

    rospy.spin()

if __name__ == "__main__":
    main()
    
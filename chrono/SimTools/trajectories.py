"""
library of functions for constructing chrono models
"""

#---------------------------- Imports -----------------------------------------
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

#import models as mdls
import SimTools.models as mdls
import matplotlib.pyplot as plt

#---------------------------- trajectory fxns ---------------------------------
"""
at first, let's not do a check on whether these locations are reachable...
"""

"""
so the plan is to have an analytic function that operates in end-effector space
and generates a target trajectory. then, we have 3 functions that inherit from 
ChFunction, and map through the IK to generate the relationship between time and
each joint angle, and return the joint angle. 

ChFunction circ (t) -> circle -> IK -> \theta1r

we will have to calculate the IK twice, but this isn't so bad.
"""



#add some plotting functions for all the trajectories classes to share
class ChFunction(chrono.ChFunction):
    def __init__(self,actuator):
        chrono.ChFunction.__init__(self)
        self.actuator = actuator
        self.a = mdls.ALEXR()
        
    def EE_location(self,t):
        return (0,0)
    
    def Get_y(self, t):
        Xee,Yee = self.EE_location(t)
        θ1l , θ2l, θ1r , θ2r = self.a.IK_2DOF(Xee,Yee)  #how should we deal with side here - ignore for now??
        if self.actuator == "l":
            return -θ1l                                  #why the heck does this need to be negative???
        elif self.actuator == "r":
            return -θ1r
    
    def plot_EE(self,t=2,mode="xy"):
        ts = np.linspace(0,t,1000)
        xs = np.zeros(ts.shape) ; ys = np.zeros(ts.shape)
        for i,t in enumerate(ts):
            xs[i] , ys[i] = self.EE_location(t)
        
        plt.figure()
        if mode == "xy": plt.plot(xs,ys)
        elif mode == "tx": plt.plot(ts,xs)
        elif mode == "ty": plt.plot(ts,ys)
        
        #plt.scatter(xs, ys, c = plt.cm.jet(ts/max(ts)))
    
    def plot_y(self,t=2):
        ts = np.linspace(0,t,1000)
        ys = np.zeros(ts.shape)
        for i,t in enumerate(ts):
            ys[i] = self.Get_y(t)
        plt.plot(ts,ys)
        
        
class p2p():
    
    @staticmethod
    def minJerkTraj(t,tf,x0,xf):
        th = t/tf
        return x0 + (x0 - xf)*(15*th**4 - 6*th**5 - 10*th**3)
    
    @staticmethod
    def traj(t,x1,y1,x2,y2,dwt1,dwt2,tt1,tt2):
        """
        t - the current time t
        x1,y1 - location of pt 1
        x2,y2 - location of pt 2
        dwt_n  - dwelltime at location n
        tt_n  -  transitTime to point n 
        """
        #setup periodic variables
        p1 = dwt1 + tt1
        p2 = dwt2 + tt2
        P = p1 + p2
        tp = t % P  #time within the period
        
        #determine what to do based on condition
        if tp >= 0 and tp < dwt1:               #dwell at x1
            return (x1,y1)
        
        elif tp >= dwt1 and tp < p1:           #reach for x2
            tp -= dwt1
            x = p2p.minJerkTraj(tp,tt1,x1,x2)
            y = p2p.minJerkTraj(tp,tt1,y1,y2)
            return (x,y)
        
        elif tp >= p1 and tp < p1 + dwt2:      #dwell at x2
            return (x2,y2)
        
        elif tp >= p1 + dwt2 and tp <= P:       #retract to x1
            tp -= (p1 + dwt2)
            x = p2p.minJerkTraj(tp,tt2,x2,x1)
            y = p2p.minJerkTraj(tp,tt2,y2,y1)
            return (x,y)
    
    

class Circle(ChFunction):
    def __init__(self,actuator,period,x,y,r):
        ChFunction.__init__(self,actuator)
        self.period = period
        self.x = x
        self.y = y
        self.r = r
        
    def EE_location(self,t):
        x = self.r*np.cos(-2*np.pi*t/self.period) + self.x - self.r
        y = self.r*np.sin(-2*np.pi*t/self.period) + self.y
        return(x,y)    
        

class point2point(ChFunction):
    """
    go from point 1 to point 2 and back, using a minimum jerk trajectory
    """
    def __init__(self,actuator,x1,y1,x2,y2,dwt1=.5,dwt2=.1,tt1=.5,tt2 = .5):
        ChFunction.__init__(self,actuator)
        self.x1 = x1
        self.x2 = x2
        self.y1 = y1
        self.y2 = y2
        self.dwt1 = dwt1
        self.dwt2 = dwt2
        self.tt1 = tt1
        self.tt2 = tt2
                  
    def EE_location(self,t):
        """
        make a function that reaches between 2 locations indefinitely
        """
        return p2p.traj(t,self.x1,  self.y1,
                          self.x2,  self.y2,
                          self.dwt1,self.dwt2,
                          self.tt1, self.tt2)
        
        
class Star(ChFunction):
    def __init__(self,actuator,x,y,r,npoints,dwt1=.5,dwt2=.1,tt1=.5,tt2 = .5):
        ChFunction.__init__(self,actuator)
        self.x = x
        self.y = y
        self.r = r
        self.npoints = npoints
        self.dwt1 = dwt1
        self.dwt2 = dwt2
        self.tt1 = tt1
        self.tt2 = tt2
        self.xpts,self.ypts = self.xypts()
        
    def xypts(self):
        t = np.linspace(0,1,self.npoints)
        x = self.r*np.cos(-2*np.pi*t) + self.x
        y = self.r*np.sin(-2*np.pi*t) + self.y
        return x,y
        
    def EE_location(self,t):
        P = self.dwt1 + self.dwt2 + self.tt1 + self.tt2
        ind = int(t / P) % self.npoints #which point in the star are we reaching towards?
        tp = t % P  #time within the period?
        return p2p.traj(tp,self.x        ,self.y        ,
                           self.xpts[ind],self.ypts[ind],
                           self.dwt1     ,self.dwt2     ,
                           self.tt1      ,self.tt2      )
        
        
        

class genTraj(ChFunction):
    """
    given a set of points and times, this function will create a cubic spline
    (or something like that) through the various points that are specified. 
    """
    def __init__(self,actuator,xs,ys,ts):
        pass



#----------------------- hit run here to troubleshoot--------------------------
        
if __name__ == "__main__":
    #test circle movement
    _circle = Circle("l",1,.5,-.5,.2)
    _circle.plot_EE()
    
    #test point 2 point movement
    _p2p = point2point("l",0,0,1,1)
    _p2p.plot_EE(mode = "ty")
    
    #test star movement
    _star = Star("l",.5,-.5,.2,7)
    _star.plot_EE(10,mode="xy")

"""
we should really figure out why the model needs to be run with so many minus signs? 
could it be because of the left handed coordinate system? it doesn't seem like 
there is congruence between the "set rotation" function - which sets a rotation 
relative to the global frame, and an angle driving function which supposedly goes 
to the same location??? this should be investigated with an experiment at some point. 
"""







#------------------- public facing functions ----------------------------------


#
#def CubicPoly(Xs,Ys,Vxs,Vys,ts,nSamps):
#    """
#    create a cubic polynomial trajectory of the end effector from a series of 
#    locations (end points and via points), velocities, and times - specifiying 
#    the number of samples.
#    inputs:
#        Xs -  the x - locations of the via points [list or np.array]
#        Ys -  the y - locations of the via points [list or np.array]
#        Vxs - the x velocity at each point - None if it doesn't matter. [list or np.array]
#        Vys - the y velocity at each point - None if it doesn't matter. [list or np.array]
#        ts - the time samples - not strictly uniformly sampled
#        nSamps - the number of samples in the output to uniformly sample between [0 - tf]
#    """
#    pass
#
#def CubicPolyJoints(Xs,Ys,Vxs,Vys,ts,nSamps):
#    """
#    from a specification of end-effector cubic Poly parameters, determine trajectories
#    for each of the 2 driving joints, θ1l and θ1r
#    inputs:
#        same as Cubic Poly
#    outputs:
#        θ1l - time trajectory of  and θ1r
#    """
#    pass
#
#



#
#function [t y yd ydd] = QuinticPolynomial(t0, tf, y0, yf, yd0, ydf, ydd0, yddf, n)
#%QUINTICPOLYNOMIAL creates a quinitic spline fit given the initial time and
#%kinematic parameters. 
#%inputs: 
#    % t0   - initial time
#    % tf   - final time
#    % y0   - inital position
#    % yf   - final position
#    % yd0  - initial velocity
#    % ydf  - final velocity
#    % ydd0 - initial acceleration
#    % yddf - final acceleration
#    % n    - number of samples
#%outputs: 
#    % t  - [1xn] time vector (evenly spaced row vector)
#    % y  - [1xn] position vector
#    % yd - [1xn] velocity vector
#    %ydd - [1xn] acceleration vector
#
#%solve for co-efficients a0 - a5 using matrix inversion
#% Ca = q   ->  a = C/q
#    
#%coefficient matrix  
#C = [1    t0   t0^2   t0^3    t0^4     t0^5
#     0    1    2*t0   3*t0^2  4*t0^3   5*t0^4 
#     0    0    2      6*t0    12*t0^2  20*t0^3
#     1    tf   tf^2   tf^3    tf^4     tf^5
#     0    1    2*tf   3*tf^2  4*tf^3   5*tf^4
#     0    0    2      6*tf    12*tf^2  20*tf^3];
#
#y = [y0 yd0 ydd0 yf ydf yddf]'; %specified positions/velocities
#
#a = C\y; %calculated quintic polynomial coefficients
#
#%----use the quintic spline co-efficients to create time vectors---------
#t = linspace(0,tf,n);
#
#%extract co-efficients
#a0 = a(1); a1 = a(2); a2 = a(3); a3 = a(4); a4 = a(5); a5 = a(6);
#
#%solve for position, velocity, acceleration vectors
#y   =   a0 +   a1*t +    a2*t.^2 +    a3*t.^3 +   a4*t.^4 + a5*t.^5;
#yd  =   a1 + 2*a2*t +  3*a3*t.^2 +  4*a4*t.^3 + 5*a5*t.^4;
#ydd = 2*a2 + 6*a3*t + 12*a4*t.^2 + 20*a5*t.^3;
#
#
#end

    

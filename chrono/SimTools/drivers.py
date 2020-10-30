"""
functions which add driven functionality to a chrono model - either kinematic 
drivers or closed loop torque motors. 
"""
#---------------------------- Imports -----------------------------------------
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np
import copy

#import models as mdls
import SimTools.models as mdls
import matplotlib.pyplot as plt



#---------------------------- trajectory fxns ---------------------------------
"""
at first, let's not do a check on whether these locations are reachable...
"""

"""
what's the plan? - 

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
    def __call__(t,x1,y1,x2,y2,dwt1,dwt2,tt1,tt2):
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
        
        


#
#class point2point(ChFunction):
#    """
#    go from point 1 to point 2 and back, using a minimum jerk trajectory
#    """
#    def __init__(self,x1,y1,x2,y2,dwellTime1=.5,dwellTime2=.1,transitTime1=.5,transitTime2 = .5):
#        ChFunction.__init__(self)
#        self.x1 = x1
#        self.x2 = x2
#        self.y1 = y1
#        self.y2 = y2
#        self.dwt1 = dwellTime1
#        self.dwt2 = dwellTime2
#        self.transitTime1 = transitTime1
#        self.transitTime2 = transitTime2
#                  
#    def EE_location(self,t):
#        """
#        make a function that reaches between 2 locations indefinitely
#        """
#        #setup periodic variables
#        p1 = self.dwt1 + self.transitTime1
#        p2 = self.dwt2 + self.transitTime2
#        P = p1 + p2
#        tp = t % P  #time within the period
#        
#        #determine what to do based on condition
#        if tp >= 0 and tp < self.dwt1:               #dwell at x1
#            return (self.x1,self.y1)
#        
#        elif tp >= self.dwt1 and tp < p1:           #reach for x2
#            tp -= self.dwt1
#            x = minJerkTraj(tp,self.transitTime1,self.x1,self.x2)
#            y = minJerkTraj(tp,self.transitTime1,self.y1,self.y2)
#            return (x,y)
#        
#        elif tp >= p1 and tp < p1 + self.dwt2:      #dwell at x2
#            return (self.x2,self.y2)
#        
#        elif tp >= p1 + self.dwt2 and tp <= P:       #retract to x1
#            tp -= (p1 + self.dwt2)
#            x = minJerkTraj(tp,self.transitTime2,self.x2,self.x1)
#            y = minJerkTraj(tp,self.transitTime2,self.y2,self.y1)
#            return (x,y)
            


class point2point(ChFunction):
    """
    go from point 1 to point 2 and back, using a minimum jerk trajectory
    """
    def __init__(self,actuator,x1,y1,x2,y2,dwellTime1=.5,dwellTime2=.1,transitTime1=.5,transitTime2 = .5):
        ChFunction.__init__(self,actuator)
        self.x1 = x1
        self.x2 = x2
        self.y1 = y1
        self.y2 = y2
        self.dwt1 = dwellTime1
        self.dwt2 = dwellTime2
        self.tt1 = transitTime1
        self.tt2 = transitTime2
                  
    def EE_location(self,t):
        """
        make a function that reaches between 2 locations indefinitely
        """
        return p2p(t,self.x1,  self.y1,
                     self.x2,  self.y2,
                     self.dwt1,self.dwt2,
                     self.tt1, self.tt2)
        
        
class star(ChFunction):
    def __init__(self,actuator,x,y,r,points,dwt1,dwt2,tt1,tt2):
        ChFunction.__init__(self,actuator)
        self.x = x
        self.y = y
        self.r = r
        self.points = points
        self.dwt1 = dwt1
        self.dwt2 = dwt2
        self.tt1 = tt1
        self.tt2 = tt2
        
    def EE_location(t):
        pass
    
    
    


"""
what does this end effector function have to do??
so, to do a star pattern, we start in the middle, then we go to from their to 
each of the spokes on the wheel, using a min-jerk traj - assuming the same dwt
tt for all senarios. 
"""
        

class genTraj():
    """
    given a set of points and times, this function will create a cubic spline
    (or something like that) through the various points that are specified. 
    """



#----------------------- hit run here to troubleshoot--------------------------
        
if __name__ == "__main__":
    p2p = point2point(0,0,1,1)
    p2p.plot_EE(mode = "ty")


"""
we should really figure out why the model needs to be run with so many minus signs? 
could it be because of the left handed coordinate system? it doesn't seem like 
there is congruence between the "set rotation" function - which sets a rotation 
relative to the global frame, and an angle driving function which supposedly goes 
to the same location??? this should be investigated with an experiment at some point. 
"""





#---------------------------- kinematic drivers -------------------------------
def addRevJoints(sys):
    jt = chrono.ChLinkRevolute()
    mdls.add_θ1l_joint(sys,jt)
    
    jt = chrono.ChLinkRevolute()
    mdls.add_θ1r_joint(sys,jt)
    


def addRotationAngleDrivers(sys,θ1l = None ,θ1r = None):
    """
    adds two ChLinkMotorRotationAngle to the system, with their respective fxns
    inputs:
        sys - the chrono system with ALEX Robot 
        θ1l - [ChFunction] - driving the first joint
        θ1r - [ChFunction] - driving the second joint
    """
    
    #joint l
    Drvl = chrono.ChLinkMotorRotationAngle()
    if θ1l != None:
        Drvl.SetAngleFunction(θ1l)  ## this isn't the right thing - need to look up how to do it properly
    mdls.add_θ1l_joint(sys,Drvl)
    
    #joint r
    Drvr = chrono.ChLinkMotorRotationAngle()
    if θ1r != None:
         Drvr.SetAngleFunction(θ1r) 
    mdls.add_θ1r_joint(sys,Drvr)
   


#---------------------------- torque motors -----------------------------------


#---------------------------- controllers -------------------------------------




# could we also have a dynamic model of the motor here? 


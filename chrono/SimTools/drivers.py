"""
functions which add driven functionality to a chrono model - either kinematic 
drivers or closed loop torque motors. 
"""
#---------------------------- Imports -----------------------------------------
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np
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


#refactor this to be inhereted??
def plot_EE(traj):
    ts = np.linspace(0,2,1000)
    c = circle("l",2,.5,-.5,.1)
    xs = np.zeros(ts.shape) ; ys = np.zeros(ts.shape)
    for i,t in enumerate(ts):
        xs[i] , ys[i] = c.EE_location(t)
    #plt.plot(xs,ys)
    #plt.scatter(xs, ys, c = plt.cm.jet(ts/max(ts)))




class circle(chrono.ChFunction):
    def __init__(self,actuator,period,x,y,r):
        chrono.ChFunction.__init__(self)
        self.actuator = actuator
        self.period = period
        self.x = x
        self.y = y
        self.r = r
        self.a = mdls.ALEXR()
        
    def EE_location(self,t):
        x = self.r*np.cos(-2*np.pi*t/self.period) + self.x  -self.r
        y = self.r*np.sin(-2*np.pi*t/self.period) + self.y
        return(x,y)
        
    def Get_y(self, t):
        Xee,Yee = self.EE_location(t)
        θ1l , θ2l, θ1r , θ2r = self.a.IK_2DOF(Xee,Yee)  #how should we deal with side here - ignore for now??
        if self.actuator == "l":
            return -θ1l                                  #why the heck does this need to be negative???
        elif self.actuator == "r":
            return -θ1r


class point2point(chrono.ChFunction):
    """
    go from point 1 to point 2 and back
    """
    def __init__(self,dwellTime,transitTime,x1,y1,x2,y2):
        self.dwellTime = dwellTime
        self.transitTime = transitTime
        self.x = x
        self.y = y
        self.r = r
        
    def EE_location(t):
        pass
        
    def Get_y(self, t):
        y = math.cos(math.pi * x)
        return y



class star(chrono.ChFunction):
    def __init__(self,dwellTime,transitTime,x,y,r):
        self.dwellTime = dwellTime
        self.transitTime = transitTime
        self.x = x
        self.y = y
        self.r = r
        
    def EE_location(t):
        pass
        
    def Get_y(self, t):
        y = math.cos(math.pi * x)
        return y






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
   




    

"""
you will probably always add the same type of driver to both, consider refactoring
here in the future, where you would supply a flag, or the type of driver fxn explicitly, 
it will be more clear in the future. 
"""

def _addRotationAngleDrivers(sys,θ1l = None ,θ1r = None):
    """
    adds two ChLinkMotorRotationAngle to the system, with their respective fxns
    inputs:
        sys - the chrono system with ALEX Robot 
        θ1l - [ChFunction] - driving the first joint
        θ1r - [ChFunction] - driving the second joint
    """
    
    #joint l
    jtl = sys.SearchLink("GB<->L1l")
    sys.RemoveLink(jtl)
    Drvl = chrono.ChLinkMotorRotationAngle()
    if θ1l != None:
        Drvl.SetAngleFunction(θ1l)  ## this isn't the right thing - need to look up how to do it properly
    mdls.add_θ1l_joint(sys,Drvl)
    
    #joint r
    jtr = sys.SearchLink("GB<->L1r")
    sys.RemoveLink(jtr)
    Drvr = chrono.ChLinkMotorRotationAngle()
    if θ1r != None:
         Drvr.SetAngleFunction(θ1r) 
    mdls.add_θ1r_joint(sys,Drvr)
   



#---------------------------- torque motors -----------------------------------


#---------------------------- controllers -------------------------------------




# could we also have a dynamic model of the motor here? 

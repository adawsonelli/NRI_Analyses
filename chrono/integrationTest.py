# -*- coding: utf-8 -*-
"""
test that you can replicate the results of the 2DOF script, with functional approach
"""

#---------------------------- Imports -----------------------------------------
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np
import SimTools as st


#---------------------------- run ---------------------------------------------

#%% effects under gravity

sys = chrono.ChSystemNSC()         #initialize the system
st.models.buildALEXR(sys)          #add ALEXR robot
st.drivers.addRevJoints(sys)       #add passive revolute joints
st.animateSystem(sys)              #visualize the system


#%% source of vibration - example of a situation in which the assembly cannot solve

sys = chrono.ChSystemNSC()               #initialize the system
st.models.buildALEXR(sys)                #add ALEXR robot
st.drivers.addRotationAngleDrivers(sys)  #add rotational drivers
st.animateSystem(sys)                    #visualize the system



#%% ----------------------------- NOTES ---------------------------------------

"""
* figure out if there is a bug in the model, in how joint on theta1l is implemented
* the first link doesn't look entirely correct either. 
* create functions for making links 1 and 2 of different types, but keeping
* the overall code very dry. place this within models. 
"""


"""
problem: when I'm running a simulation in which both the right and left joints are driven, 
why is there bouncing in the simulation sometimes? 
    
solution: I believe that the bouncing in this model came from when the forward kinematics 
didn't exist / can't solve for that particular combination of joints, therefore some combination 
of constraints were violated, and instead of violating the driving joints the non-driven joints 
were violated. this will never happen when we are using the models, because we will
always drive from the inverse kinematics, so there will always be a solution for the forward kinematics. 
it also would not happen if the motors were driven by torque instead of by motion. 

"""

"""
notes: - NRI deliverables for next week - finish the set of test kinematic functions
that you are planning - then debug what's going on with the inertia of the system
(this will require test systems I believe) and what's going on with the negative sign
that you need in the kinematic drivers, does this in any way indicate there is a problem with
the model??? - basically a debugging week, that will setup the inverse dynamics analysis that
you will present the following week.   
"""



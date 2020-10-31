"""
functions which add driven functionality to a chrono model - either kinematic 
drivers or closed loop torque motors. 
"""
#---------------------------- Imports -----------------------------------------
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

#import models as mdls
import SimTools.models as mdls
import matplotlib.pyplot as plt


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


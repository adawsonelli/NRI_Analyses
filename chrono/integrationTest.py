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
sys = chrono.ChSystemNSC()  #initialize the system
sys = st.buildALEXR(sys)    #add ALEXR robot
st.animateSystem(sys)       #visualize the system

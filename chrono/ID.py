# -*- coding: utf-8 -*-
"""
exploring inverse dynamic analysis
"""

#---------------------------- Imports -----------------------------------------
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np
import SimTools as st


#---------------------------- run ---------------------------------------------
# trajectory generation - fully constrained kinematic system.

#%%  circle animation - small circle

#init
x0 = .5 ; y0 = -.5
period = 1.5 ; rad = .1

#setup system
sys = chrono.ChSystemNSC()         #initialize the system
st.models.buildALEXR(sys)          #add ALEXR robot

#add driving function
circ_left  = st.drivers.Circle("l",period,x0,y0,rad) 
circ_right = st.drivers.Circle("r",period,x0,y0,rad)

st.drivers.addRotationAngleDrivers(sys,circ_left,circ_right)

#animate the system
st.animateSystem(sys)              #visualize the system



#%%  circle animation - large circle

#init
x0 = .75 ; y0 = -.6
period = 3 ; rad = .25

#setup system
sys = chrono.ChSystemNSC()         #initialize the system
st.models.buildALEXR(sys)          #add ALEXR robot

#add driving function
circ_left  = st.drivers.Circle("l",period,x0,y0,rad) 
circ_right = st.drivers.Circle("r",period,x0,y0,rad)
st.drivers.addRotationAngleDrivers(sys,circ_left,circ_right)

#animate the system
st.animateSystem(sys)              #visualize the system


#%% reaching - point to point

#init
x1 = .4 ; y1 = -.4 
x2 = .6 ; y2 = -.6

#setup system
sys = chrono.ChSystemNSC()         #initialize the system
st.models.buildALEXR(sys)          #add ALEXR robot

#add driving function
p2p_left  = st.drivers.point2point("l",x1,y1,x2,y2) 
p2p_right = st.drivers.point2point("r",x1,y1,x2,y2)
st.drivers.addRotationAngleDrivers(sys,p2p_left,p2p_right)

#animate the system
st.animateSystem(sys) 


#%% star pattern

#init
x = .5 ;  y = -.5
r = .15;  npoints = 7

#setup system
sys = chrono.ChSystemNSC()         #initialize the system
st.models.buildALEXR(sys)          #add ALEXR robot

#add driving function
p2p_left  = st.drivers.Star("l",x,y,r,npoints) 
p2p_right = st.drivers.Star("r",x,y,r,npoints)
st.drivers.addRotationAngleDrivers(sys,p2p_left,p2p_right)

#animate the system
st.animateSystem(sys) 





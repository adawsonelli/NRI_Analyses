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
period = 1 ; rad = .01

#setup system
sys = chrono.ChSystemNSC()                   #initialize the system
st.models.buildALEXR(sys,eeMass=1)          #add ALEXR robot

#remove gravity
#sys.Set_G_acc(chrono.ChVectorD(0,0,0)) 

#add driving function
circ_left  = st.trajectories.Circle("l",period,x0,y0,rad) 
circ_right = st.trajectories.Circle("r",period,x0,y0,rad)

st.drivers.addRotationAngleDrivers(sys,circ_left,circ_right)

#animate the system
am = st.vis.animationModifiers()
am.addTrace(sys,"EE")
am.addTrace(sys,"L2l")
am.addCOGframes(sys)

#animate the system
st.animateSystem(sys,am)              

#plot the torques
st.plots.plotTorques(sys,10) #ringing coming from ICs??

#plot the torques
st.plots.plotTorques(sys,6,3)  



#%%  circle animation - large circle

#init
x0 = .75 ; y0 = -.6
period = 3 ; rad = .25

#setup system
sys = chrono.ChSystemNSC()         #initialize the system
st.models.buildALEXR(sys)          #add ALEXR robot

#add driving function
circ_left  = st.trajectories.Circle("l",period,x0,y0,rad) 
circ_right = st.trajectories.Circle("r",period,x0,y0,rad)
st.drivers.addRotationAngleDrivers(sys,circ_left,circ_right)



#animate the system
am = st.vis.animationModifiers()
am.addTrace(sys,"EE",tFade=4)
am.addTrace(sys,"L2l",tFade=4)
st.animateSystem(sys,am)              #visualize the system

#plot the torques
#st.plots.plotTorques(sys,3)

#plot the torques
st.plots.plotTorques(sys,6,3)

#%% reaching - point to point

#init
#x1 = .4 ; y1 = -.4 
#x2 = .6 ; y2 = -.6


x1 = .8 ; y1 = -.3 
x2 = .45 ; y2 = -.3

tt1 = 3; tt2 = 3


#setup system
sys = chrono.ChSystemNSC()         #initialize the system
st.models.buildALEXR(sys)          #add ALEXR robot

#add driving function
p2p_left  = st.trajectories.point2point("l",x1,y1,x2,y2,tt1=tt1,tt2=tt2) 
p2p_right = st.trajectories.point2point("r",x1,y1,x2,y2,tt1=tt2,tt2=tt2)
st.drivers.addRotationAngleDrivers(sys,p2p_left,p2p_right)

#animate the system
am = st.vis.animationModifiers()
am.addTrace(sys,"EE")
am.addTrace(sys,"L2l")
am.addCOGframes(sys)
st.animateSystem(sys,am)              #visualize the system

#plot the torques
st.plots.plotTorques(sys,10,3)



#
#%% star pattern

#init
x = .5 ;  y = -.5
r = .15;  npoints = 7

#setup system
sys = chrono.ChSystemNSC()         #initialize the system
st.models.buildALEXR(sys)          #add ALEXR robot

#add driving function
p2p_left  = st.trajectories.Star("l",x,y,r,npoints) 
p2p_right = st.trajectories.Star("r",x,y,r,npoints)
st.drivers.addRotationAngleDrivers(sys,p2p_left,p2p_right)


#animate the system
am = st.vis.animationModifiers()
am.addTrace(sys,"EE")
st.animateSystem(sys,am)              #visualize the system


#plot the torques
#st.plots.plotTorques(sys,20)

#plot the torques
st.plots.plotTorques(sys,10,5)





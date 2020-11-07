"""
plotting functions for different output variables
"""
#---------------------------- Imports -----------------------------------------
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np
import matplotlib.pyplot as plt



#---------------------------- plotting functions ------------------------------


#%matplotlib qt               #plot in separate window
#%matplotlib inline           #plot inline


def plotTorques(system,time):
    """
    plot the torque produced at joints as a function of time:
        θ1l - the first driving joint
        θ1r - the second joint
    inputs: 
        system - the system to be simulated
        time - the time over which the system is to be simulated
    """
    #simulate the system and calculate the torques
    dt = .005
    t =  np.arange(0,time,dt)
    τ1 = np.zeros_like(t)
    τ2 = np.zeros_like(t)   
    θ1l = system.SearchLink("GB<->L1l")
    θ1r = system.SearchLink("GB<->L1r")

    system.SetChTime(0)
    i = 0
    while (system.GetChTime() < time):
        system.DoStepDynamics(0.005)
        τ1 = θ1l.GetMotorTorque()
        τ2 = θ1l.GetMotorTorque()
        i +=1
    
    
    #plot the torques as a function of time. 
    colors = ['#462f7cff',"#78d051ff"]              #dark, light
    plt.figure(figsize = (12,6),facecolor = 'w')
    plt.title(r"$\bf{\tau_{L}(t)}}$ and $\bf{\tau_{R}(t)}}$ ", fontsize = 18)
    plt.xlabel('Time(s)', fontsize=16)
    plt.ylabel('Torque(Nm)', fontsize=16)
    
    plt.plot(t,τ1,label=r"$\tau_L$",color=colors[1],linewidth=4.0)
    plt.plot(t,τ2,label=r"$\tau_R$",color=colors[0],linewidth=4.0)
    
    plt.legend(fontsize = 18)
    plt.grid(alpha = .2)
    


        


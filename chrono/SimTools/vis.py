# -*- coding: utf-8 -*-
"""
contains functions for visualizing the results of a simulation in realtime, 
or graphing different variables over time. 
"""
#---------------------------- Imports -----------------------------------------
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

import numpy as np
import time


#------------------------- class definitions ----------------------------------

"""
capabilities to considering adding to this class
** change the line to be more configurable (thicker, fade out, etc)
** pull out strobe into it's own class, and add the ability to add multiple strobes easily, 
** each with configuration settings, including colors, so that we can look at trajectories 
simultaniously. 
** add the ability to fail gracefully into the class - so if the link isn't there, just print something and don't fail.
** this represents another 3 hr commitment, so I may punt on this refactor for now

"""

 
"""
we should also see what the deal is with the color function, to see if we can adjust the alpha and maybe the color too?
and the thickness
"""

class Trace:
    def __init__(self,system,bodyName,tFade=1.5,freq=10,A0=1):
        self.bodyFrame = system.SearchBody(bodyName)
        self.tFade = tFade
        self.freq = freq
        self.A0 = A0
        self.points = []
        self.trajHistory = None
        self.n = 0
        self.dt = .005
        
    def update(self,IrrApp):
        self.n += 1
        if self.n == 1 or self.n % int((1/self.dt) / self.freq) == 0:
            pos = self.bodyFrame.GetPos()
            posCopy = chrono.ChVectorD(pos.x,pos.y,pos.z)
            self.points.append(posCopy)
            if len(self.points) > int(self.tFade * self.freq):
                self.points.pop(0)
            self.trajHistory = chrono.vector_ChVectorD(self.points)
        chronoirr.ChIrrTools_drawPolyline(IrrApp.GetVideoDriver(),self.trajHistory)


class animationModifiers():
    """
    class which modifies the animation to add effects like tracers or
    reference frame visualizers etc.
    """
    def __init__(self):
        #trace vars
        self.traces = []
        self.traceActive = False
        
        #add COG ref-frames
        #add x,y,z to the animation as a modifier - see ChIrrTools
    
    def addTrace(self,system,bodyName,tFade=1.5,freq=10,A0=1):
        """
        add a trace effect to a body reference frame (usually the EE)
        inputs:
            system - ChSystem being animated
            bodyName - name of the body the strobe will be attached to 
            tfade - time for a dot to fade to zero and be deleted
            freq - number of dots added per second
            A0 - default opacity
        """
        self.traceActive = True
        self.traces.append(Trace(system,bodyName,tFade,freq,A0))
        
       
    def addCOGrfs(self,IrrApp):
        pass
    
    def draw(self,IrrApp):
        """
        draw all the animation modifiers
        """
        if self.traceActive:
            for trace in self.traces:
                trace.update(IrrApp)
    
    
    


#---------------------- function definitions ----------------------------------


def animateSystem(system,am = animationModifiers()):
    """
    animate the system using the chrono 
    """
    # -------------------------------------------------------------------------
    #            Create an Irrlicht application to visualize the system
    # -------------------------------------------------------------------------
    myapplication = chronoirr.ChIrrApp(system, 'PyChrono example', chronoirr.dimension2du(1024,768))
    
    
    
    myapplication.AddTypicalSky()
    myapplication.AddTypicalLogo()
    myapplication.AddTypicalCamera(chronoirr.vector3df(0.6,0.6,0.8))
    myapplication.AddTypicalLights()
    
    
                # ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
    			# in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
    			# If you need a finer control on which item really needs a visualization proxy in
    			# Irrlicht, just use application.AssetBind(myitem); on a per-item basis.
    
    myapplication.AssetBindAll();
    
    			# ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
    			# that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!
    
    myapplication.AssetUpdateAll();
    
    
    
    # -------------------------------------------------------------------------
    #                             Run the simulation
    # -------------------------------------------------------------------------
    
    
    myapplication.SetTimestep(0.005)
    
    
    while(myapplication.GetDevice().run()):
        myapplication.BeginScene()
        myapplication.DrawAll()
        am.draw(myapplication)
        myapplication.DoStep()
        myapplication.EndScene()
        

